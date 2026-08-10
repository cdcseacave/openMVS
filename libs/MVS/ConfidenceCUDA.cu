/*
 * ConfidenceCUDA.cu
 *
 * GPU port of the fusion-faithful confidence recalibration. Two kernels, one thread per reference
 * pixel:
 *   PriorKernel  -- the intra-map geometric prior (ComputeIntraMapPrior): a local depth-plane fit
 *                   (shared ConfRefine::DepthPlaneFit) + slope-aware planarity/quorum + gradient-vs-
 *                   stored-normal agreement.
 *   SweepKernel  -- one-hop multi-view confirmation (AdjustConfidenceSweep): project the pixel into
 *                   each neighbor, apply the 4 soft gates (+ FSV), accumulate K/Pconf/V,
 *                   then the shared ConfRefine::Posterior.
 * The per-pixel arithmetic is the SAME ConfidenceRefine.h used by the CPU (single-precision here);
 * GPU-vs-CPU differences are ULP-level (float vs double exp, FMA), well inside |dROC| <= 0.005.
 *
 * Both kernels are templated on a reference-map accessor so the same code serves two layouts:
 *   RefLinearAcc -- the standalone/fallback path: reference depth/normal/conf uploaded from host
 *                   as linear buffers (RunConfidenceCUDA, unchanged semantics).
 *   RefPackedAcc -- the fused path (T14 resident-buffer reuse): reference depth+normal read from
 *                   PatchMatch's resident Point4 estimates and raw conf derived from its resident
 *                   ZNCC cost buffer (RunConfidenceFusedCUDA); nothing reference-sized is uploaded.
 * On any CUDA error the launchers free everything and return false so the caller falls back.
 */
#include <cuda_runtime.h>
#include <cstdio>
#include <cmath>
#include <vector>

#include "ConfidenceRefine.h"
#include "ConfidenceCUDA.h"

namespace MVS {
namespace CUDA {

using ConfRefine::Params;
using ConfRefine::F3;

// device-resident neighbor descriptor (fused transforms + device map pointers)
struct DevNeighbor {
	float A[9], b[3], Ai[9], bi[3], Rrel[9];
	const float* depth;              // null when texDepth is used instead
	const float* conf;               // null -> no confidence (cN = 1)
	const float* normal;             // null -> no normal gate
	cudaTextureObject_t texDepth;    // nonzero -> read depth from PatchMatch's resident texture
	int width, height;
};

// nearest-pixel round matching SEACAVE::Round2Int(float) = floor(x + 0.5f)
__device__ __forceinline__ int Round2IntDev(float x) { return (int)floorf(x + 0.5f); }

// neighbor depth at integer pixel (x,y), from the linear upload or the resident texture. The
// texture is cudaFilterModeLinear, but a fetch at the exact texel center (x+0.5, y+0.5) has
// interpolation weight exactly 0, so it returns the texel bit-exactly -- identical to the linear
// read, keeping CPU/GPU parity. Callers guarantee (x,y) is inside [0,width)x[0,height).
__device__ __forceinline__ float NbDepthAt(const DevNeighbor& np, int x, int y) {
	return np.depth ? np.depth[y*np.width + x] :
		tex2D<float>(np.texDepth, (float)x + 0.5f, (float)y + 0.5f);
}

// ---- reference-map accessors (kernel template parameter) ----
// Both expose: Depth(idx), HasNormal(), Normal(idx), Conf(idx) plus the operator()(x,y)/inside(x,y)
// depth interface ConfRefine::DepthPlaneFit expects from its accessor argument.

// standalone path: reference maps uploaded from host as linear buffers
struct RefLinearAcc {
	const float* depth;
	const float* normal;   // interleaved x,y,z, or null
	const float* conf;
	int W, H;
	__device__ __forceinline__ float Depth(int idx) const { return depth[idx]; }
	__device__ __forceinline__ int HasNormal() const { return normal != nullptr; }
	__device__ __forceinline__ F3 Normal(int idx) const { return F3{normal[idx*3+0], normal[idx*3+1], normal[idx*3+2]}; }
	__device__ __forceinline__ float Conf(int idx) const { return conf[idx]; }
	__device__ __forceinline__ float operator()(int x, int y) const { return depth[y*W + x]; }
	__device__ __forceinline__ bool inside(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }
};

// fused path: reference read straight from PatchMatch's resident buffers -- Point4 per pixel
// (xyz = normal, w = depth) and the raw ZNCC cost (conf = cost>=1 ? 0 : 1-cost, the exact
// conversion the host unpack loop applies, so CPU/GPU parity is preserved bit-for-bit)
struct RefPackedAcc {
	const float4* dn;
	const float* cost;
	int W, H;
	__device__ __forceinline__ float Depth(int idx) const { return dn[idx].w; }
	__device__ __forceinline__ int HasNormal() const { return 1; }
	__device__ __forceinline__ F3 Normal(int idx) const { const float4 v = dn[idx]; return F3{v.x, v.y, v.z}; }
	__device__ __forceinline__ float Conf(int idx) const { const float c = cost[idx]; return c >= 1.f ? 0.f : 1.f - c; }
	__device__ __forceinline__ float operator()(int x, int y) const { return dn[y*W + x].w; }
	__device__ __forceinline__ bool inside(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }
};

// device port of SceneDensify.cpp SampleDepthBilinear (edge-aware; false -> caller uses nearest).
// The 4 taps go through NbDepthAt (exact texel fetches) -- hardware bilinear CANNOT be used here
// because each tap must pass the validity (>0) and min/max depth-similarity gates individually.
__device__ __forceinline__ bool SampleDepthBilinearDev(const DevNeighbor& np,
                                                        float px, float py, float thDepth, float& d) {
	const int x0 = (int)floorf(px), y0 = (int)floorf(py);
	if (x0 < 0 || y0 < 0 || x0 + 1 >= np.width || y0 + 1 >= np.height) return false;
	const float d00 = NbDepthAt(np, x0, y0), d01 = NbDepthAt(np, x0+1, y0);
	const float d10 = NbDepthAt(np, x0, y0+1), d11 = NbDepthAt(np, x0+1, y0+1);
	if (d00 <= 0.f || d01 <= 0.f || d10 <= 0.f || d11 <= 0.f) return false;
	const float dmin = fminf(fminf(d00, d01), fminf(d10, d11));
	const float dmax = fmaxf(fmaxf(d00, d01), fmaxf(d10, d11));
	if (!ConfRefine::IsDepthSimilarF(dmin, dmax, thDepth)) return false;
	const float wx = px - (float)x0, wy = py - (float)y0;
	d = (d00*(1.f-wx) + d01*wx)*(1.f-wy) + (d10*(1.f-wx) + d11*wx)*wy;
	return true;
}

// ---- intra-map geometric prior (mirrors DepthMapsData::ComputeIntraMapPrior) ----
template <typename RefAcc>
__global__ void PriorKernel(RefAcc ref, int W, int H, float k00, float k11, float k02, float k12,
                            float band, float invKmin,
                            float* priorOut) {
	const int c = blockIdx.x*blockDim.x + threadIdx.x;
	const int r = blockIdx.y*blockDim.y + threadIdx.y;
	if (c >= W || r >= H) return;
	const int idx = r*W + c;
	priorOut[idx] = 0.f;
	float w, wx, wy;
	if (!ConfRefine::DepthPlaneFit(ref, c, r, w, wx, wy))
		return;
	int nInl = 0; float sumE2 = 0.f;
	for (int y = -1; y <= 1; ++y) {
		const int rr = r + y; if (rr < 0 || rr >= H) continue;
		for (int x = -1; x <= 1; ++x) {
			if (x == 0 && y == 0) continue;
			const int cc = c + x; if (cc < 0 || cc >= W) continue;
			const float dN = ref(cc, rr);
			if (dN <= 0.f) continue;
			const float dpred = w + wx*(float)x + wy*(float)y;
			const float e = fabsf(dN - dpred) / w;
			if (e < band) { ++nInl; const float en = e/band; sumE2 += en*en; }
		}
	}
	if (nInl < 3) return;
	const float Pplane = ConfRefine::CRexp(-sumE2 / (float)nInl);
	const float gate = 1.f - ConfRefine::CRexp(-(float)nInl * invKmin);
	float Pnorm = 1.f;
	if (ref.HasNormal()) {
		const F3 nGrad = ConfRefine::NormalFromGrad(k00, k11, k02, k12, c, r, w, wx, wy);
		const F3 sn = ref.Normal(idx);
		Pnorm = fmaxf(0.f, nGrad.x*sn.x + nGrad.y*sn.y + nGrad.z*sn.z);
	}
	float pr = Pplane * Pnorm * gate;
	priorOut[idx] = pr < 0.f ? 0.f : (pr > 1.f ? 1.f : pr);
}

// ---- one-hop multi-view confirmation (mirrors AdjustConfidenceSweep) ----
template <typename RefAcc>
__global__ void SweepKernel(RefAcc ref, const float* priorMap, int W, int H,
                            const DevNeighbor* neigh, int nNeigh, Params p,
                            float maxReprojErrorSq, float* confOut) {
	const int c = blockIdx.x*blockDim.x + threadIdx.x;
	const int r = blockIdx.y*blockDim.y + threadIdx.y;
	if (c >= W || r >= H) return;
	const int idx = r*W + c;
	const float depthRef = ref.Depth(idx);
	if (depthRef <= 0.f) { confOut[idx] = 0.f; return; }
	float rnx = 0.f, rny = 0.f, rnz = 0.f;
	const int hasRefNormal = ref.HasNormal();
	if (hasRefNormal) { const F3 rn = ref.Normal(idx); rnx = rn.x; rny = rn.y; rnz = rn.z; }

	float K = 0.f, Pconf = 0.f;
	int V = 0;
	const float ud = (float)c * depthRef, vd = (float)r * depthRef;
	for (int k = 0; k < nNeigh; ++k) {
		const DevNeighbor np = neigh[k];
		const float qz = np.A[6]*ud + np.A[7]*vd + np.A[8]*depthRef + np.b[2];
		if (qz <= 0.f) continue;
		const float qx = np.A[0]*ud + np.A[1]*vd + np.A[2]*depthRef + np.b[0];
		const float qy = np.A[3]*ud + np.A[4]*vd + np.A[5]*depthRef + np.b[1];
		const float px = qx/qz, py = qy/qz;
		const int xN = Round2IntDev(px), yN = Round2IntDev(py);
		if (xN < 0 || xN >= np.width || yN < 0 || yN >= np.height) continue;
		const float dNn = NbDepthAt(np, xN, yN);
		if (dNn <= 0.f) continue;
		const bool hasNormalGate = (hasRefNormal && np.normal != nullptr);

			const bool gDepthNearest = ConfRefine::IsDepthSimilarF(dNn, qz, p.thDepth);
			if (!gDepthNearest && dNn > qz*(1.f + p.violMargin*p.thDepth)) ++V;
			float dN;
			if (!SampleDepthBilinearDev(np, px, py, p.thDepth, dN))
				dN = dNn;
			const float wD = ConfRefine::SoftDepthW(qz, dN, p.thDepth);
			const float un = (float)xN*dN, vn = (float)yN*dN;
			const float qrz = np.Ai[6]*un + np.Ai[7]*vn + np.Ai[8]*dN + np.bi[2];
			float wR = 0.f;
			if (qrz > 0.f) {
				const float qrx = np.Ai[0]*un + np.Ai[1]*vn + np.Ai[2]*dN + np.bi[0];
				const float qry = np.Ai[3]*un + np.Ai[4]*vn + np.Ai[5]*dN + np.bi[1];
				const float du = qrx/qrz - (float)c, dv = qry/qrz - (float)r;
				wR = ConfRefine::SoftReprojW(du, dv, p.thReproj);
			}
			float wN = 1.f;
			if (hasNormalGate) {
				const float nx = np.Rrel[0]*rnx + np.Rrel[1]*rny + np.Rrel[2]*rnz;
				const float ny = np.Rrel[3]*rnx + np.Rrel[4]*rny + np.Rrel[5]*rnz;
				const float nz = np.Rrel[6]*rnx + np.Rrel[7]*rny + np.Rrel[8]*rnz;
				const float mnx = np.normal[(yN*np.width+xN)*3+0], mny = np.normal[(yN*np.width+xN)*3+1], mnz = np.normal[(yN*np.width+xN)*3+2];
				wN = fmaxf(0.f, nx*mnx + ny*mny + nz*mnz);
			}
			const float cN = np.conf ? np.conf[yN*np.width + xN] : 1.f;
			const float wC = ConfRefine::SoftConfW(cN, p.minConfidence, p.epsConf);
			const float w = wD*wR*wN*wC;
			if (w <= 0.05f) continue;
			K += w; Pconf += w*cN;
	}
	const float Kf = K;
	confOut[idx] = ConfRefine::Posterior(ref.Conf(idx), priorMap[idx], Kf, Pconf, (float)V, p);
}

// RAII: free every cudaMalloc'd pointer on scope exit (success or early return), then consume any
// pending CUDA error so a non-fatal failure (e.g. OOM) does not leak this thread's last-error into a
// later CUDA call on the same reused pool-worker thread (review finding, robustness).
namespace { struct DevBag { std::vector<void*> v; ~DevBag(){ for (void* p : v) cudaFree(p); cudaGetLastError(); } }; }

// shared tail of both launchers: allocate the prior/output buffers, upload the neighbor maps +
// descriptors, launch both kernels on `stream`, download the adjusted confidence and synchronize.
// Every device allocation is tracked in `bag`, freed by the caller's scope exit.
template <typename RefAcc>
static bool LaunchConfidenceKernels(
	int W, int H, const RefAcc& ref,
	float k00, float k11, float k02, float k12,
	const ConfNeighborHost* neighbors, int nNeighbors,
	const Params& params,
	cudaStream_t stream, DevBag& bag, float* confOut)
{
	const size_t nPix = (size_t)W * (size_t)H;
	auto dmalloc = [&](size_t bytes) -> void* {
		void* p = nullptr;
		if (bytes == 0) return nullptr;
		if (cudaMalloc(&p, bytes) != cudaSuccess) return (void*)-1;
		bag.v.push_back(p);
		return p;
	};
	auto up = [&](void* dst, const void* src, size_t bytes) -> bool {
		return dst && cudaMemcpyAsync(dst, src, bytes, cudaMemcpyHostToDevice, stream) == cudaSuccess;
	};

	float* dPrior = (float*)dmalloc(nPix*sizeof(float));
	float* dConf  = (float*)dmalloc(nPix*sizeof(float));
	if (dPrior==(void*)-1 || dConf==(void*)-1) return false;

	// neighbor maps + descriptors
	std::vector<DevNeighbor> hNeigh(nNeighbors);
	for (int k = 0; k < nNeighbors; ++k) {
		const ConfNeighborHost& s = neighbors[k];
		const size_t np = (size_t)s.width * (size_t)s.height;
		DevNeighbor d;
		for (int i = 0; i < 9; ++i) { d.A[i]=s.A[i]; d.Ai[i]=s.Ai[i]; d.Rrel[i]=s.Rrel[i]; }
		for (int i = 0; i < 3; ++i) { d.b[i]=s.b[i]; d.bi[i]=s.bi[i]; }
		d.width = s.width; d.height = s.height;
		// fused path with a valid resident texture: read depth via tex2D, skip the largest upload
		d.texDepth = (cudaTextureObject_t)s.texDepth;
		d.depth = nullptr;
		if (s.texDepth == 0) {
			float* dd = (float*)dmalloc(np*sizeof(float)); if (dd==(void*)-1) return false;
			if (!up(dd, s.depth, np*sizeof(float))) return false; d.depth = dd;
		}
		d.conf = nullptr;
		if (s.conf) { float* dc=(float*)dmalloc(np*sizeof(float)); if (dc==(void*)-1) return false; if (!up(dc,s.conf,np*sizeof(float))) return false; d.conf = dc; }
		d.normal = nullptr;
		if (s.normal) { float* dn=(float*)dmalloc(np*3*sizeof(float)); if (dn==(void*)-1) return false; if (!up(dn,s.normal,np*3*sizeof(float))) return false; d.normal = dn; }
		hNeigh[k] = d;
	}
	DevNeighbor* dNeigh = nullptr;
	if (nNeighbors > 0) {
		dNeigh = (DevNeighbor*)dmalloc(nNeighbors*sizeof(DevNeighbor)); if (dNeigh==(void*)-1) return false;
		if (!up(dNeigh, hNeigh.data(), nNeighbors*sizeof(DevNeighbor))) return false;
	}

	// launch
	const dim3 block(32, 8, 1);
	const dim3 grid((W + block.x - 1)/block.x, (H + block.y - 1)/block.y, 1);
	const float band = params.thDepth * 3.f;
	const float invKmin = 1.f/4.f;
	const float maxReprojErrorSq = params.thReproj * params.thReproj;

	PriorKernel<RefAcc><<<grid, block, 0, stream>>>(ref, W, H,
		k00, k11, k02, k12, band, invKmin, dPrior);
	SweepKernel<RefAcc><<<grid, block, 0, stream>>>(ref, dPrior,
		W, H, dNeigh, nNeighbors, params, maxReprojErrorSq, dConf);

	if (cudaMemcpyAsync(confOut, dConf, nPix*sizeof(float), cudaMemcpyDeviceToHost, stream) != cudaSuccess) return false;
	if (cudaStreamSynchronize(stream) != cudaSuccess) return false;
	if (cudaGetLastError() != cudaSuccess) return false;
	return true;
}

bool RunConfidenceCUDA(
	int W, int H,
	const float* refDepth, const float* refNormal, const float* refConf,
	float k00, float k11, float k02, float k12,
	const ConfNeighborHost* neighbors, int nNeighbors,
	const Params& params,
	float* confOut)
{
	if (W <= 0 || H <= 0 || nNeighbors < 0) return false;
	const size_t nPix = (size_t)W * (size_t)H;
	DevBag bag;
	cudaStream_t stream = 0;
	if (cudaStreamCreate(&stream) != cudaSuccess) return false;
	struct StreamGuard { cudaStream_t s; ~StreamGuard(){ cudaStreamDestroy(s); } } sg{stream};

	auto dmalloc = [&](size_t bytes) -> void* {
		void* p = nullptr;
		if (bytes == 0) return nullptr;
		if (cudaMalloc(&p, bytes) != cudaSuccess) return (void*)-1;
		bag.v.push_back(p);
		return p;
	};
	auto up = [&](void* dst, const void* src, size_t bytes) -> bool {
		return dst && cudaMemcpyAsync(dst, src, bytes, cudaMemcpyHostToDevice, stream) == cudaSuccess;
	};

	// reference maps
	float* dRefDepth = (float*)dmalloc(nPix*sizeof(float));
	float* dRefConf  = (float*)dmalloc(nPix*sizeof(float));
	if (dRefDepth==(void*)-1 || dRefConf==(void*)-1) return false;
	float* dRefNormal = nullptr;
	if (refNormal) { dRefNormal = (float*)dmalloc(nPix*3*sizeof(float)); if (dRefNormal==(void*)-1) return false; }
	if (!up(dRefDepth, refDepth, nPix*sizeof(float))) return false;
	if (!up(dRefConf, refConf, nPix*sizeof(float))) return false;
	if (refNormal && !up(dRefNormal, refNormal, nPix*3*sizeof(float))) return false;

	const RefLinearAcc ref{dRefDepth, dRefNormal, dRefConf, W, H};
	return LaunchConfidenceKernels(W, H, ref, k00, k11, k02, k12,
		neighbors, nNeighbors, params, stream, bag, confOut);
}

bool RunConfidenceFusedCUDA(
	int W, int H,
	const void* devDepthNormals, const float* devCosts,
	float k00, float k11, float k02, float k12,
	const ConfNeighborHost* neighbors, int nNeighbors,
	const Params& params,
	void* stream, float* confOut)
{
	if (W <= 0 || H <= 0 || nNeighbors < 0 || !devDepthNormals || !devCosts || !confOut) return false;
	DevBag bag;
	// Point4 is 4 contiguous floats (x,y,z = normal, w = depth), 16-byte aligned by cudaMalloc
	const RefPackedAcc ref{reinterpret_cast<const float4*>(devDepthNormals), devCosts, W, H};
	return LaunchConfidenceKernels(W, H, ref, k00, k11, k02, k12,
		neighbors, nNeighbors, params,
		(cudaStream_t)stream, bag, confOut);
}

} // namespace CUDA
} // namespace MVS
