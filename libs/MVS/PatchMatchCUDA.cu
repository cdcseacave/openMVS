/*
* PatchMatchCUDA.cu
*
* Copyright (c) 2014-2021 SEACAVE
*
* Author(s):
*
*	  cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*	  You are required to preserve legal notices and author attributions in
*	  that material or in the Appropriate Legal Notices displayed by works
*	  containing it.
*/

#include "PatchMatchCUDA.inl"
#include "Common/DepthGeometry.h"

// static max supported views
#define MAX_VIEWS 32

// samples used to perform views selection
#define NUM_SAMPLES 32

// unified "bad cost" sentinel: returned by ScorePlane when the patch
// cannot be evaluated against a view (out-of-frame, texture-less, or
// degenerate variance), used as the view-pruning threshold in the
// multi-hypothesis joint view selection, and as the all-views-rejected
// fallback in AggregateMultiViewScores. Keeping the three meanings on
// a single name makes the alignment explicit and tunable from one place.
#define fBadCost 1.2f

// patch window radius
#define nSizeHalfWindow 4

// patch stepping
#define nSizeStep 2

// Launch-bounds tuning. Default uses 256 threads/block with 2 resident
// blocks/SM, letting the warp scheduler interleave across blocks while
// one is stalled on tex2D latency (~1.8% per-view kernel time vs the
// historical 512/1 config). Set PATCHMATCHCUDA_LB_256_2=0 to fall back.
#ifndef PATCHMATCHCUDA_LB_256_2
#define PATCHMATCHCUDA_LB_256_2 1
#endif

#if PATCHMATCHCUDA_LB_256_2
#define PATCHMATCHCUDA_BLOCK_H_DIV 4   // BLOCK_H = BLOCK_W / 4 = 8
#define PATCHMATCHCUDA_LAUNCH_BOUNDS __launch_bounds__(256, 2)
#else
#define PATCHMATCHCUDA_BLOCK_H_DIV 2   // BLOCK_H = BLOCK_W / 2 = 16
#define PATCHMATCHCUDA_LAUNCH_BOUNDS __launch_bounds__(512, 1)
#endif


namespace MVS {

namespace CUDA {

#define ImagePixels cudaTextureObject_t
#define RandState curandState

// __constant__ memory admits no dynamic initialization, and nvcc treats the
// user-provided (Eigen) constructors of Camera as one; store the cameras as
// raw bytes of the same size and alignment instead
struct alignas(Camera) CameraConstStorage {
	unsigned char bytes[sizeof(Camera)];
};
static_assert(sizeof(CameraConstStorage) == sizeof(Camera), "Camera constant storage must preserve Camera size");
static_assert(alignof(CameraConstStorage) == alignof(Camera), "Camera constant storage must preserve Camera alignment");

// Cameras and runtime params live in __constant__ memory: warp-broadcast
// reads through the constant cache replace per-thread parameter-stack /
// global-memory traffic. Updated via UploadCameras() / UploadParams()
// before each pyramid-level kernel launch.
//
// IMPORTANT: these symbols are module-global per device, so PatchMatchCUDA
// is single-instance / single-in-flight per device. The current densify
// pipeline guarantees this (one image at a time per device) and the C++
// side enforces it at runtime via an atomic in-flight counter (see
// PatchMatchCUDA::EstimateDepthMap). If multi-stream / multi-instance
// parallel use is ever added, switch to per-instance device buffers
// passed explicitly to kernels.
__constant__ CameraConstStorage g_cameraStorage[MAX_VIEWS + 1];
#define g_cameras reinterpret_cast<const Camera*>(g_cameraStorage)
__constant__ PatchMatch::Params g_params;

// the camera models a kernel instantiation serves: every view pinhole (the plane homography of
// ComputeHomography), or a pinhole or an equirectangular reference whose neighbor views each take
// their own model (the model warp, resolved once per pixel, hypothesis and view)
enum KernelModel { MODEL_PINHOLE, MODEL_PINHOLE_REF, MODEL_SPHERICAL_REF };
template <int MODEL> struct RefModel {
	typedef SEACAVE::Pinhole<float> Type;
	__device__ static Type Get(const Camera& camera) { return camera.model.AsPinhole(); }
};
template <> struct RefModel<MODEL_SPHERICAL_REF> {
	typedef SEACAVE::Spherical<float> Type;
	__device__ static Type Get(const Camera& camera) { return camera.model.AsSpherical(); }
};

// a view as the model kernels see it: its pose relative to the reference (a reference camera-frame
// point X is R*X+t in the view's frame; identity for the reference), its center in the reference frame
// (C = -R^T*t) and the scale from texel-centered pixel coordinates to its texture coordinates (normalized
// for an equirectangular view, else 1)
struct ModelView {
	Matrix3 R;
	Point3 t;
	Point3 C;
	Point2 texScale;
};
struct alignas(ModelView) ModelViewConstStorage {
	unsigned char bytes[sizeof(ModelView)];
};
__constant__ ModelViewConstStorage g_viewStorage[MAX_VIEWS + 1];
#define g_views reinterpret_cast<const ModelView*>(g_viewStorage)

// set/check a bit
__device__ constexpr void SetBit(unsigned& input, unsigned i) {
	input |= (1u << i);
}
__device__ constexpr int IsBitSet(unsigned input, unsigned i) {
	return (input >> i) & 1u;
}
__device__ constexpr int IsBitSet(uint64_t input, unsigned i) {
	return (int)((input >> i) & 1u);
}

// Read-only-cache loaders for planes[]. Safe within a checkerboard pass
// because every offset in `dirs` and `neighborPositions` has odd Manhattan
// parity from the current pixel, so the cells we read are not written in
// this launch -- the __ldg() read-only contract holds and L1 bandwidth
// is freed for the texture work.
__device__ __forceinline__ Point4 LoadPlaneLDG(const Point4* p) {
	const float* f = p->data();
	Point4 r;
	r.x() = __ldg(f + 0);
	r.y() = __ldg(f + 1);
	r.z() = __ldg(f + 2);
	r.w() = __ldg(f + 3);
	return r;
}
__device__ __forceinline__ float LoadPlaneWLDG(const Point4* p) {
	return __ldg(p->data() + 3);
}

// sort the given values array using bubble sort algorithm
__device__ inline void Sort(const float* values, float* sortedValues, int n) {
	for (int i = 0; i < n; ++i)
		sortedValues[i] = values[i];
	do {
		int newn = 0;
		for (int i = 1; i < n; ++i) {
			if (sortedValues[i-1] > sortedValues[i]) {
				Swap(sortedValues[i-1], sortedValues[i]);
				newn = i;
			}
		}
		n = newn;
	} while(n);
}

// find the index of the minimum value in the given values array
__device__ inline int FindMinIndex(const float* values, const int n) {
	float minValue = values[0];
	int minValueIdx = 0;
	for (int i = 1; i < n; ++i) {
		if (minValue > values[i]) {
			minValue = values[i];
			minValueIdx = i;
		}
	}
	return minValueIdx;
}

// convert Probability Density Function (PDF) to Cumulative Distribution Function (CDF)
__device__ inline void PDF2CDF(float* probs, const int numProbs) {
	float probSum = 0.f;
	for (int i = 0; i < numProbs; ++i)
		probSum += probs[i];
	const float invProbSum = 1.f / probSum;
	float sumProb = 0.f;
	for (int i = 0; i < numProbs-1; ++i) {
		sumProb += probs[i] * invProbSum;
		probs[i] = sumProb;
	}
	probs[numProbs-1] = 1.f;
}
/*----------------------------------------------------------------*/


// generate a random unit vector (Marsaglia's method on the unit sphere);
// algebraically unit-length: |n|^2 = 4q1^2(1-s) + 4q2^2(1-s) + (1-2s)^2 = 1
__device__ inline Point3 GenerateRandomUnitVector(RandState* randState)
{
	float q1, q2, s;
	do {
		q1 = 2.f * curand_uniform(randState) - 1.f;
		q2 = 2.f * curand_uniform(randState) - 1.f;
		s = q1 * q1 + q2 * q2;
	} while (s >= 1.f);
	const float sq = sqrtf(1.f - s);
	return Point3(
		2.f * q1 * sq,
		2.f * q2 * sq,
		1.f - 2.f * s);
}

// generate a random normal in the half-space facing the camera along the pixel's viewing ray
// (viewDirection() returns it, any length); the distribution is isotropic, so it is the same in
// the ray's local frame for every camera model
template <typename VIEWDIR>
__device__ inline Point3 GenerateRandomNormal(const VIEWDIR& viewDirection, RandState* randState)
{
	const Point3 normal = GenerateRandomUnitVector(randState);
	return normal.dot(viewDirection()) > 0.f ? Point3(-normal) : normal;
}

// randomly perturb a normal (algorithmically unit-preserving);
// Rodrigues rotation around a Marsaglia-unit axis by a random small angle
template <typename VIEWDIR>
__device__ inline Point3 GeneratePerturbedNormal(const VIEWDIR& viewDirection, const Point3& normal, RandState* randState, const float perturbation)
{
	// random angle in [-perturbation/2, +perturbation/2]
	const float theta = (curand_uniform(randState) - 0.5f) * perturbation;
	float sinT, cosT;
	__sincosf(theta, &sinT, &cosT);

	// rodrigues' rotation formula
	const Point3 axis = GenerateRandomUnitVector(randState);
	const float aDotN = axis.dot(normal);
	const Point3 axCrossN = axis.cross(normal);
	const Point3 normalPerturbed = normal * cosT + axCrossN * sinT + axis * (aDotN * (1.f - cosT));

	// keep the perturbed normal in the camera-facing half-space
	return normalPerturbed.dot(viewDirection()) >= 0.f ? normal : normalPerturbed;
}

// randomly perturb a depth, sampling uniformly from the intersection of the
// perturbation window [(1-p)d, (1+p)d] with the valid range [fDepthMin, fDepthMax]
__device__ inline float GeneratePerturbedDepth(float depth, RandState* randState, const float perturbation)
{
	const float lo = fmaxf((1.f - perturbation) * depth, g_params.fDepthMin);
	const float hi = fminf((1.f + perturbation) * depth, g_params.fDepthMax);
	return lo + curand_uniform(randState) * (hi - lo);
}

// compose tho homography matrix that transforms a point from reference to source camera through the given plane
__device__ inline Matrix3 ComputeHomography(const CUDA::Camera& refCamera, const CUDA::Camera& trgCamera, const Point2& p, const Point4& plane)
{
	const Point3 X = refCamera.model.TransformPointI2C(p, plane.w());
	const Point3 normal = plane.topLeftCorner<3,1>();
	// guard against plane passing through (or near) the reference camera center:
	// normal.dot(X) -> 0 makes t infinite and the resulting H NaN
	const float denom = normal.dot(X);
	const float safeDenom = fabsf(denom) < FLT_EPSILON ? copysignf(FLT_EPSILON, denom) : denom;
	const Point3 t = (refCamera.pose.C - trgCamera.pose.C) / safeDenom;
	const Matrix3 H = trgCamera.pose.R * (refCamera.pose.R.transpose() + t*normal.transpose());
	return trgCamera.model.K() * H * refCamera.model.K().inverse();
}

// weight a neighbor texel based on color similarity and distance to the center texel
__device__ inline float ComputeBilateralWeight4(int idx, float pix, float centerPix)
{
	// spatial Gaussian for the 5x5 patch (sample positions in {-4,-2,0,2,4};
	// sigmaSpatial = -1/18) precomputed: exp(-(dx*dx + dy*dy) / 18);
	// row-major over (i, j) with i as the outer index
	static constexpr float spatialLUT[25] = {
		0.169013f, 0.329193f, 0.411112f, 0.329193f, 0.169013f,
		0.329193f, 0.641180f, 0.800737f, 0.641180f, 0.329193f,
		0.411112f, 0.800737f, 1.000000f, 0.800737f, 0.411112f,
		0.329193f, 0.641180f, 0.800737f, 0.641180f, 0.329193f,
		0.169013f, 0.329193f, 0.411112f, 0.329193f, 0.169013f,
	};
	constexpr float sigmaColor = -1.f / (2.f * 25.f/255.f*25.f/255.f);
	const float colorDistSq = Square(pix - centerPix);
	return spatialLUT[idx] * __expf(colorDistSq * sigmaColor);
}
__device__ inline float ComputeBilateralWeight(int xDist, int yDist, float pix, float centerPix)
{
	constexpr float sigmaSpatial = -1.f / (2.f * (nSizeHalfWindow-1)*(nSizeHalfWindow-1));
	constexpr float sigmaColor = -1.f / (2.f * 25.f/255.f*25.f/255.f);
	const float spatialDistSq = float(xDist * xDist + yDist * yDist);
	const float colorDistSq = Square(pix - centerPix);
	return __expf(spatialDistSq * sigmaSpatial + colorDistSq * sigmaColor);
}

// compute the geometric consistency weight
__device__ inline float GeometricConsistencyWeight(const ImagePixels depthImage, const CUDA::Camera& refCamera, const CUDA::Camera& trgCamera, const Point4& plane, const Point2i& p)
{
	if (depthImage == NULL)
		return 0.f;
	constexpr float maxDist = 4.f;
	const Point3 forwardPoint = refCamera.TransformPointI2W(p.cast<float>(), plane.w());
	const Point2 trgPt = trgCamera.TransformPointW2I(forwardPoint);
	const float trgDepth = tex2D<float>(depthImage, trgPt.x() + 0.5f, trgPt.y() + 0.5f);
	if (trgDepth == 0.f)
		return maxDist;
	const Point3 trgX = trgCamera.TransformPointI2W(trgPt, trgDepth);
	const Point2 backwardPoint = refCamera.TransformPointW2I(trgX);
	const Point2 diff = p.cast<float>() - backwardPoint;
	const float distSq = diff.squaredNorm();
	return min(maxDist, sqrtf(distSq + sqrtf(distSq)*2.f));
}

// number of samples in the (2*halfWin/step + 1)^2 reference patch
#define N_PATCH_SAMPLES ((2 * nSizeHalfWindow / nSizeStep + 1) * (2 * nSizeHalfWindow / nSizeStep + 1))

// Per-pixel reference-patch state. Depends only on the reference image at p,
// so it is invariant across source views and plane hypotheses. Compute once
// at the top of ProcessPixel / InitializePixelScore and reuse for every
// ScorePlane call (eliminates ~13*nNumViews redundant ref tex2D fetches and
// 25*13*nNumViews bilateral-weight evaluations per pixel).
struct RefPatchCache {
	float weight[N_PATCH_SAMPLES];        // bilateral weight per patch sample
	float weightRefPix[N_PATCH_SAMPLES];  // weight * refPix per sample
	float sumRef;                         // Σ weight * refPix
	float bilateralWeightSum;             // Σ weight
	float varRef;                         // sumRefRef*Σw - sumRef^2
};

// reference image texel at pixel x: an equirectangular reference texture takes normalized
// coordinates, wrapping the columns across the seam
template <int MODEL>
__device__ inline float FetchRefTexel(const ImagePixels refImage, int x, int y)
{
	if constexpr (MODEL == MODEL_SPHERICAL_REF)
		return tex2D<float>(refImage, (x + 0.5f) * g_views[0].texScale.x(), (y + 0.5f) * g_views[0].texScale.y());
	else
		return tex2D<float>(refImage, x + 0.5f, y + 0.5f);
}

template <int MODEL>
__device__ inline void ComputeRefPatchCache(const ImagePixels refImage, const Point2i& p, RefPatchCache& cache)
{
	const float refCenterPix = FetchRefTexel<MODEL>(refImage, p.x(), p.y());
	float sumRef = 0.f, sumRefRef = 0.f, bws = 0.f;
	int idx = 0;
	#pragma unroll
	for (int i = -nSizeHalfWindow; i <= nSizeHalfWindow; i += nSizeStep) {
		#pragma unroll
		for (int j = -nSizeHalfWindow; j <= nSizeHalfWindow; j += nSizeStep) {
			const float refPix = FetchRefTexel<MODEL>(refImage, p.x() + j, p.y() + i);
			#if nSizeHalfWindow == 4
			const float w = ComputeBilateralWeight4(idx, refPix, refCenterPix);
			#else
			const float w = ComputeBilateralWeight(j, i, refPix, refCenterPix);
			#endif
			const float wRef = w * refPix;
			cache.weight[idx] = w;
			cache.weightRefPix[idx] = wRef;
			sumRef += wRef;
			sumRefRef += wRef * refPix;
			bws += w;
			++idx;
		}
	}
	cache.sumRef = sumRef;
	cache.bilateralWeightSum = bws;
	cache.varRef = sumRefRef * bws - sumRef * sumRef;
}

// target texels of the patch against the reference ones: the patch warp gives the homogeneous target
// texture coordinates of the top-left texel (X) and their steps along a patch row (dx) and down a patch
// column (dy); offset moves the dehomogenized coordinates onto the texel centers
struct PatchSums {
	float sumTrg = 0.f, sumTrgTrg = 0.f, sumRefTrg = 0.f;
};
__device__ inline void SampleWarpedPatch(const RefPatchCache& cache, const ImagePixels trgImage, Point3 X, const Point3& dx, const Point3& dy, const float offsetX, const float offsetY, PatchSums& sums)
{
	// inline hnormalized() as RCP + 2 FMAs (the texel-center offset rides into the FMA)
	// replaces 2 IEEE divisions per sample in the 25-sample patch walk, the hottest inner loop
	Point3 baseX(X);
	int idx = 0;
	#pragma unroll
	for (int i = -nSizeHalfWindow; i <= nSizeHalfWindow; i += nSizeStep) {
		#pragma unroll
		for (int j = -nSizeHalfWindow; j <= nSizeHalfWindow; j += nSizeStep) {
			const float invZ = __fdividef(1.f, X.z());
			const float trgPx = X.x() * invZ + offsetX;
			const float trgPy = X.y() * invZ + offsetY;
			const float trgPix = tex2D<float>(trgImage, trgPx, trgPy);
			const float w = cache.weight[idx];
			const float wTrg = w * trgPix;
			sums.sumTrg += wTrg;
			sums.sumTrgTrg += wTrg * trgPix;
			sums.sumRefTrg += cache.weightRefPix[idx] * trgPix;
			++idx;
			X += dx;
		}
		baseX += dy;
		X = baseX;
	}
}

// weighted ZNCC cost of the sampled patch, blended with the depth prior in texture-less patches
__device__ inline float ScoreNCC(const RefPatchCache& cache, const PatchSums& sums, const float depth, const float lowDepth)
{
	if (lowDepth <= 0 && cache.varRef < 1e-8f)
		return fBadCost;
	const float sumTrg = sums.sumTrg, sumTrgTrg = sums.sumTrgTrg, sumRefTrg = sums.sumRefTrg;
	const float varTrg = sumTrgTrg * cache.bilateralWeightSum - sumTrg * sumTrg;
	const float varRefTrg = cache.varRef * varTrg;
	if (varRefTrg < 1e-16f)
		return fBadCost;
	const float covarTrgRef = sumRefTrg * cache.bilateralWeightSum - cache.sumRef * sumTrg;
	float ncc = 1.f - covarTrgRef * rsqrtf(varRefTrg);

	// apply depth prior weight based on patch textureless;
	// hard-cap the prior on medium to well-textured patches:
	// 0.0025 is the optimum tested on several GT datasets
	if (lowDepth > 0 && cache.varRef < 0.0025f) {
		const float deltaDepth(min((fabsf(lowDepth-depth) / lowDepth), 0.5f));
		constexpr float smoothSigmaDepth(-1.f / (1.f * 0.02f)); // 0.12: patch texture variance below 0.02 (0.12^2) is considered texture-less
		const float factorDeltaDepth(__expf(cache.varRef * smoothSigmaDepth));
		ncc = (1.f-factorDeltaDepth)*ncc + factorDeltaDepth*deltaDepth;
	}
	return max(0.f, min(2.f, ncc));
}

// compute photometric score using weighted ZNCC; uses precomputed reference cache
__device__ float ScorePlane(const RefPatchCache& cache, const CUDA::Camera& refCamera, const ImagePixels trgImage, const CUDA::Camera& trgCamera, const Point2i& p, const Point4& plane, const float lowDepth)
{
	Matrix3 H = ComputeHomography(refCamera, trgCamera, p.cast<float>(), plane);
	{
		const Point3 ptH = H * p.cast<float>().homogeneous();
		const float invZ = __fdividef(1.f, ptH.z());
		const float ptX = ptH.x() * invZ;
		const float ptY = ptH.y() * invZ;
		if (ptX >= trgCamera.size.x() || ptX < 0.f || ptY >= trgCamera.size.y() || ptY < 0.f)
			return fBadCost;
	}
	const Point3 X = H * Point2(p.x()-nSizeHalfWindow, p.y()-nSizeHalfWindow).homogeneous();
	H *= float(nSizeStep);
	PatchSums sums;
	SampleWarpedPatch(cache, trgImage, X, H.col(0), H.col(1), 0.5f, 0.5f, sums);
	return ScoreNCC(cache, sums, plane.w(), lowDepth);
}

// per-pixel reference state of the model kernels: the reference camera model and its ray frame at the
// pixel (CameraModels.h RayFrame: the ray derivatives along x and y, then the ray); an equirectangular
// reference builds both, and the rays of the patch texels, from the sines and cosines of the column's
// longitude and the row's elevation (Params::refTrig), as the CPU estimator does
template <int MODEL>
struct RefPixel {
	typename RefModel<MODEL>::Type camera;
	Matrix3 F;
	__device__ RefPixel(const Point2i& p) : camera(RefModel<MODEL>::Get(g_cameras[0])) {
		if constexpr (MODEL == MODEL_SPHERICAL_REF) {
			const float2 lon = __ldg(g_params.refTrig + p.x()), el = __ldg(g_params.refTrig + g_cameras[0].size.x() + p.y());
			F = camera.RayFrame(lon.x, lon.y, el.x, el.y);
		} else {
			F = camera.RayFrame(p.cast<float>());
		}
	}
	// the ray of the pixel at offset (j,i) of p, inside the image but for the wrapping columns
	__device__ Point3 TexelRay(const Point2i& p, int j, int i) const {
		if constexpr (MODEL == MODEL_SPHERICAL_REF) {
			const int width = g_cameras[0].size.x();
			const int c = SEACAVE::WrapColumn(p.x() + j, width);
			const float2 lon = __ldg(g_params.refTrig + c), el = __ldg(g_params.refTrig + width + p.y() + i);
			return SEACAVE::Spherical<float>::Ray(lon.x, lon.y, el.x, el.y);
		} else {
			return camera.Ray(Point2(float(p.x() + j), float(p.y() + i)));
		}
	}
};
// the pinhole kernels: the plane homography needs no per-pixel state
template <>
struct RefPixel<MODEL_PINHOLE> {
	__device__ RefPixel(const Point2i&) {}
};

// the viewing ray of the reference pixel, any length
template <int MODEL>
__device__ inline Point3 ViewDirection(const RefPixel<MODEL>& ref, const Point2i& p)
{
	if constexpr (MODEL == MODEL_PINHOLE)
		return g_cameras[0].model.ViewDirection(p);
	else
		return ref.F.col(2);
}

// Method A: each texel ray of the reference patch intersected with the plane and projected exactly
// into the equirectangular target; false if a texel ray meets the plane behind the reference camera
template <int MODEL>
__device__ __forceinline__ bool SampleExactPatch(const RefPatchCache& cache, const RefPixel<MODEL>& ref, const ImagePixels trgImage, const SEACAVE::Spherical<float>& trg, const ModelView& view, const Point3& normal, const float nX, const Point2i& p, PatchSums& sums)
{
	int idx = 0;
	#pragma unroll
	for (int i = -nSizeHalfWindow; i <= nSizeHalfWindow; i += nSizeStep) {
		#pragma unroll
		for (int j = -nSizeHalfWindow; j <= nSizeHalfWindow; j += nSizeStep, ++idx) {
			const Point3 ray = ref.TexelRay(p, j, i);
			const float den = normal.dot(ray);
			if (den >= 0.f)
				return false;
			Point2 x;
			trg.ProjectFast(view.R * ray * (nX / den) + view.t, x);
			const float trgPix = tex2D<float>(trgImage, (x.x() + 0.5f) * view.texScale.x(), (x.y() + 0.5f) * view.texScale.y());
			const float w = cache.weight[idx];
			const float wTrg = w * trgPix;
			sums.sumTrg += wTrg;
			sums.sumTrgTrg += wTrg * trgPix;
			sums.sumRefTrg += cache.weightRefPix[idx] * trgPix;
		}
	}
	return true;
}

// photometric score of the plane against a neighbor view of any camera model: the patch warp
// M = trg.PixelHomography(H*r0)*H*F (DepthGeometry.h PlanePatchWarp), F the reference ray frame at p
// and H*F = R*F + t*(n^T*F)/(n.X); into an equirectangular target the tangent-plane warp (Method B)
// up to the elevation threshold, the exact per-texel warp (Method A) above it, as the CPU estimator
template <int MODEL>
__device__ __forceinline__ float ScorePlaneModel(const RefPatchCache& cache, RefPixel<MODEL>& ref, const ImagePixels trgImage, const int imgId, const Point2i& p, const Point4& plane, const float lowDepth)
{
	const ModelView& view = g_views[imgId];
	const Point3 normal(plane.topLeftCorner<3,1>());
	const float nX = plane.w() * normal.dot(ref.F.col(2)); // n.X of the plane point at p
	if (nX >= 0.f)
		return fBadCost; // a propagated plane not facing the reference camera at p
	const Matrix3 HF = view.R * ref.F + view.t * ((normal.transpose() * ref.F) / nX);
	const Point3 q0(HF.col(2)); // the target ray of the patch center
	const Camera& trgCamera = g_cameras[imgId];
	PatchSums sums;
	if (IsBitSet(g_params.sphericalViews, imgId)) {
		const SEACAVE::Spherical<float> trg(trgCamera.model.AsSpherical());
		if (Square(q0.y()) > g_params.fSinSqWarpMaxElevation * q0.squaredNorm()) {
			if (!SampleExactPatch(cache, ref, trgImage, trg, view, normal, nX, p, sums))
				return fBadCost;
		} else {
			const float rho = sqrtf(q0.x() * q0.x() + q0.z() * q0.z());
			Point2 xc;
			trg.ProjectFast(q0, xc, rho);
			Matrix3 M = trg.PixelHomography(q0, xc, rho) * HF;
			M.row(0) *= view.texScale.x();
			M.row(1) *= view.texScale.y();
			const Point3 X = M * Point3(float(-nSizeHalfWindow), float(-nSizeHalfWindow), 1.f);
			const Point3 dx = M.col(0) * float(nSizeStep), dy = M.col(1) * float(nSizeStep);
			// the homogeneous coordinate is affine over the patch: every texel is in front of the
			// tangent plane (its ray less than 90 degrees off the center) if the corners are
			constexpr float steps = float(2 * nSizeHalfWindow / nSizeStep);
			if (X.z() <= 0.f || X.z() + dx.z() * steps <= 0.f || X.z() + dy.z() * steps <= 0.f || X.z() + (dx.z() + dy.z()) * steps <= 0.f)
				return fBadCost;
			// the texture wraps the columns across the seam and clamps the rows
			SampleWarpedPatch(cache, trgImage, X, dx, dy, 0.5f * view.texScale.x(), 0.5f * view.texScale.y(), sums);
		}
	} else {
		// the patch center in front of a pinhole target and inside its image
		if (q0.z() <= 0.f)
			return fBadCost;
		const Matrix3 M = trgCamera.model.AsPinhole().PixelHomography(q0) * HF;
		const Point3 center = M.col(2);
		const float invZ = __fdividef(1.f, center.z());
		const float ptX = center.x() * invZ;
		const float ptY = center.y() * invZ;
		if (ptX >= trgCamera.size.x() || ptX < 0.f || ptY >= trgCamera.size.y() || ptY < 0.f)
			return fBadCost;
		const Point3 X = M * Point3(float(-nSizeHalfWindow), float(-nSizeHalfWindow), 1.f);
		SampleWarpedPatch(cache, trgImage, X, M.col(0) * float(nSizeStep), M.col(1) * float(nSizeStep), 0.5f, 0.5f, sums);
	}
	return ScoreNCC(cache, sums, plane.w(), lowDepth);
}

// geometric consistency against a neighbor depth-map of any camera model: the reference point moved
// into the neighbor's frame and projected, the neighbor depth sampled there, the neighbor point at
// that depth along the same ray projected back; the reprojection distance (columns wrapped for an
// equirectangular reference) mapped to [0,4], 4 where the neighbor holds no depth
template <int MODEL>
__device__ __forceinline__ float GeometricConsistencyWeightModel(const ImagePixels depthImage, const RefPixel<MODEL>& ref, const int imgId, const Point4& plane, const Point2i& p)
{
	if (depthImage == NULL)
		return 0.f;
	constexpr float maxDist = 4.f;
	const ModelView& view = g_views[imgId];
	const Point3 X1 = view.R * (ref.F.col(2) * plane.w()) + view.t;
	Point2 x1;
	float depth1Expected;
	if (IsBitSet(g_params.sphericalViews, imgId)) {
		g_cameras[imgId].model.AsSpherical().ProjectFast(X1, x1);
		depth1Expected = SEACAVE::Spherical<float>::Depth(X1);
	} else {
		if (!g_cameras[imgId].model.AsPinhole().Project(X1, x1))
			return maxDist;
		depth1Expected = SEACAVE::Pinhole<float>::Depth(X1);
	}
	const float depth1 = tex2D<float>(depthImage, (x1.x() + 0.5f) * view.texScale.x(), (x1.y() + 0.5f) * view.texScale.y());
	if (depth1 == 0.f)
		return maxDist;
	Point2 xb;
	if (!ref.camera.ProjectFast(view.R.transpose() * (X1 * (depth1 / depth1Expected) - view.t), xb))
		return maxDist;
	float dx = p.x() - xb.x();
	if constexpr (MODEL == MODEL_SPHERICAL_REF) {
		const float width = float(g_cameras[0].size.x());
		dx -= width * rintf(dx / width);
	}
	const float distSq = Square(dx) + Square(p.y() - xb.y());
	return min(maxDist, sqrtf(distSq + sqrtf(distSq)*2.f));
}

// compute photometric score for all neighbor images;
// GEOM-templated so geom-consistency loop is dead-code eliminated when off
template <bool GEOM, int MODEL>
__device__ __forceinline__ void MultiViewScorePlane(const RefPatchCache& cache, RefPixel<MODEL>& ref, const ImagePixels* images, const ImagePixels* depthImages, const Point2i& p, const Point4& plane, const float lowDepth, float* costVector)
{
	const int nNumViews = g_params.nNumViews;
	if constexpr (MODEL == MODEL_PINHOLE) {
		for (int imgId = 1; imgId <= nNumViews; ++imgId)
			costVector[imgId-1] = ScorePlane(cache, g_cameras[0], images[imgId], g_cameras[imgId], p, plane, lowDepth);
		if (GEOM) {
			for (int imgId = 0; imgId < nNumViews; ++imgId)
				costVector[imgId] += 0.1f * GeometricConsistencyWeight(depthImages[imgId], g_cameras[0], g_cameras[imgId+1], plane, p);
		}
	} else {
		// the view gate, as the CPU estimator's (ScorePixel): a view seeing the hypothesis point at a
		// triangulation angle below fMinAngle does not constrain its depth and scores as a view that does
		// not see the patch, unless every view does: then none is gated (next to the epipoles, where the
		// true depth gates every view, the closer hypotheses some view sees at a larger angle would win)
		const Point3 X(ref.F.col(2) * plane.w());
		const float sqNormX = X.squaredNorm();
		uint32_t gatedViews = 0;
		for (int imgId = 1; imgId <= nNumViews; ++imgId) {
			const Point3 D(X - g_views[imgId].C);
			if (X.dot(D) > g_params.fCosMinViewAngle * sqrtf(sqNormX * D.squaredNorm()))
				gatedViews |= 1u << (imgId-1);
		}
		if (__popc(gatedViews) == nNumViews)
			gatedViews = 0;
		for (int imgId = 1; imgId <= nNumViews; ++imgId)
			costVector[imgId-1] = (gatedViews >> (imgId-1)) & 1u ? fBadCost : ScorePlaneModel(cache, ref, images[imgId], imgId, p, plane, lowDepth);
		if (GEOM) {
			for (int imgId = 0; imgId < nNumViews; ++imgId)
				costVector[imgId] += 0.1f * GeometricConsistencyWeightModel(depthImages[imgId], ref, imgId+1, plane, p);
		}
	}
}
// same as above, but interpolate the plane to current pixel position
template <bool GEOM, int MODEL>
__device__ __forceinline__ float MultiViewScoreNeighborPlane(const RefPatchCache& cache, RefPixel<MODEL>& ref, const ImagePixels* images, const ImagePixels* depthImages, const Point2i& p, const Point2i& np, Point4 plane, const float lowDepth, float* costVector)
{
	plane.w() = SEACAVE::InterpolatePlaneDepth(RefModel<MODEL>::Get(g_cameras[0]), p, np, plane.w(), plane.topLeftCorner<3,1>(), g_params.fDepthMin, g_params.fDepthMax);
	MultiViewScorePlane<GEOM, MODEL>(cache, ref, images, depthImages, p, plane, lowDepth, costVector);
	return plane.w();
}

// aggregate photometric scores from MC-sampled views into one per-pixel
// cost: the MC-weighted mean over views with viewWeights > 0. Sentinel
// views (cost == fBadCost from ScorePlane: out-of-frame, occlusion, or
// degenerate variance) are included at their raw cost, pulling the mean
// upward and disadvantaging plane hypotheses that fail to project many
// views. NUM_SAMPLES = sum(viewWeights[]) by construction in
// ProcessPixel (NUM_SAMPLES MC draws each increment one viewWeights[])
__device__ inline float AggregateMultiViewScores(const unsigned* viewWeights, const float* costVector, int numViews)
{
	float cost = 0;
	for (int imgId = 0; imgId < numViews; ++imgId)
		if (viewWeights[imgId])
			cost += viewWeights[imgId] * costVector[imgId];
	return cost / float(NUM_SAMPLES);
}

// Per-pixel update for ACMH-style patch-match stereo on GPU; reference:
//   "Multi-View Stereo with Asymmetric Checkerboard Propagation and
//    Multi-Hypothesis Joint View Selection", Xu & Tao, 2018.
//
// Each call performs (for a single pixel):
//   1. Adaptive neighbor sampling - 8 directional patterns (4 near + 4 far);
//      pick the best plane in each direction and score it against all views
//      into costArray[posId][imgId].
//   2. Multi-hypothesis joint view selection:
//        - Build viewSelectionPriors[j] from neighbors' selectedViews bitmasks.
//        - For each view, count agreeing/disagreeing neighbor planes and form
//          samplingProbs[imgId] = prior * Gaussian-weighted local agreement.
//        - PDF2CDF normalizes; NUM_SAMPLES Monte-Carlo draws populate
//          viewWeights[imgId] (= count of times imgId was sampled).
//   3. Plane comparison - aggregate each of the 8 neighbor planes + the
//      current plane against the shared viewWeights; pick the lowest.
//   4. Plane refinement - perturb depth/normal, re-score against the same
//      viewWeights, keep if it lowers the aggregate cost.
//
// The shared viewWeights basis across (3) and (4) ensures plane hypotheses
// are evaluated on a consistent view-selection footing within this pixel.
//
// An equirectangular reference (MODEL_SPHERICAL_REF) wraps the neighbor columns across the seam, which
// keeps the red/black parity as its width is even, and does not cross the poles (over a pole the
// column moves by half the width, which flips the parity when the width is twice the height); its
// rows next to the poles are not estimated, as on the CPU.
template <bool GEOM, int MODEL>
__device__ __forceinline__ void ProcessPixel(const ImagePixels* images, const ImagePixels* depthImages, Point4* planes, const float* lowDepths, float* costs, RandState* randStates, unsigned* selectedViews, const Point2i& p, const int iter)
{
	const int width = g_cameras[0].size.x();
	const int height = g_cameras[0].size.y();
	if (p.x() >= width || p.y() >= height)
		return;
	if constexpr (MODEL == MODEL_SPHERICAL_REF) {
		if (p.y() < nSizeHalfWindow || p.y() >= height - nSizeHalfWindow)
			return;
	}
	const int idx = Point2Idx(p, width);
	RandState* randState = &randStates[idx];
	float lowDepth = 0;
	if (g_params.bLowResProcessed)
		lowDepth = lowDepths[idx];
	// reference-patch state is invariant across views and hypotheses; cache once
	RefPatchCache refCache;
	ComputeRefPatchCache<MODEL>(images[0], p, refCache);
	RefPixel<MODEL> ref(p);

	// adaptive sampling: 0 up-near, 1 down-near, 2 left-near, 3 right-near, 4 up-far, 5 down-far, 6 left-far, 7 right-far
	static constexpr int2 dirs[8][11] = {
		{{ 0,-1},{-1,-2},{ 1,-2},{-2,-3},{ 2,-3},{-3,-4},{ 3,-4}},
		{{ 0, 1},{-1, 2},{ 1, 2},{-2, 3},{ 2, 3},{-3, 4},{ 3, 4}},
		{{-1, 0},{-2,-1},{-2, 1},{-3,-2},{-3, 2},{-4,-3},{-4, 3}},
		{{ 1, 0},{ 2,-1},{ 2, 1},{ 3,-2},{ 3, 2},{ 4,-3},{ 4, 3}},
		{{0,-3},{0,-5},{0,-7},{0,-9},{0,-11},{0,-13},{0,-15},{0,-17},{0,-19},{0,-21},{0,-23}},
		{{0, 3},{0, 5},{0, 7},{0, 9},{0, 11},{0, 13},{0, 15},{0, 17},{0, 19},{0, 21},{0, 23}},
		{{-3,0},{-5,0},{-7,0},{-9,0},{-11,0},{-13,0},{-15,0},{-17,0},{-19,0},{-21,0},{-23,0}},
		{{ 3,0},{ 5,0},{ 7,0},{ 9,0},{ 11,0},{ 13,0},{ 15,0},{ 17,0},{ 19,0},{ 21,0},{ 23,0}}
	};
	static constexpr int numDirs[8] = {7, 7, 7, 7, 11, 11, 11, 11};
	constexpr bool bWrapCols = MODEL == MODEL_SPHERICAL_REF;
	const int neighborPositions[4] = {
		idx - width,
		idx + width,
		bWrapCols && p.x() == 0 ? idx - 1 + width : idx - 1,
		bWrapCols && p.x() == width - 1 ? idx + 1 - width : idx + 1,
	};
	bool valid[8] = {false, false, false, false, false, false, false, false};
	int positions[8];
	float neighborDepths[8];
	float costArray[8][MAX_VIEWS];

	for (int posId=0; posId<8; ++posId) {
		const int2* samples = dirs[posId];
		Point2i bestNx; float bestConf(FLT_MAX);
		for (int dirId=0; dirId<numDirs[posId]; ++dirId) {
			const int2& offset = samples[dirId];
			Point2i np(p.x()+offset.x, p.y()+offset.y);
			if constexpr (bWrapCols) {
				if (!(np.y()>=0 && np.y()<height))
					continue;
				np.x() = SEACAVE::WrapColumn(np.x(), width);
			} else {
				if (!(np.x()>=0 && np.y()>=0 && np.x()<width && np.y()<height))
					continue;
			}
			const int nidx = Point2Idx(np, width);
			const float nconf = costs[nidx];
			if (bestConf > nconf) {
				bestNx = np;
				bestConf = nconf;
			}
		}
		if (bestConf < FLT_MAX) {
			valid[posId] = true;
			positions[posId] = Point2Idx(bestNx, width);
			neighborDepths[posId] = MultiViewScoreNeighborPlane<GEOM, MODEL>(refCache, ref, images, depthImages, p, bestNx, LoadPlaneLDG(&planes[positions[posId]]), lowDepth, costArray[posId]);
		}
	}

	// multi-hypothesis view selection
	float viewSelectionPriors[MAX_VIEWS] = {};
	const int nNumViews = g_params.nNumViews;
	for (int posId = 0; posId < 4; ++posId) {
		if (valid[posId]) {
			const unsigned selectedView = selectedViews[neighborPositions[posId]];
			for (int j = 0; j < nNumViews; ++j)
				viewSelectionPriors[j] += (IsBitSet(selectedView, j) ? 0.9f : 0.1f);
		}
	}
	float samplingProbs[MAX_VIEWS];
	const float thCost = 0.8f * __expf(Square((float)iter) / (-2.f * 4.f*4.f));
	for (int imgId = 0; imgId < nNumViews; ++imgId) {
		float sumW = 0;
		unsigned count = 0;
		unsigned countBad = 0;
		for (int posId = 0; posId < 8; posId++) {
			if (valid[posId]) {
				if (costArray[posId][imgId] < thCost) {
					sumW += __expf(Square(costArray[posId][imgId]) / (-2.f * 0.3f*0.3f));
					++count;
				} else if (costArray[posId][imgId] >= fBadCost) {
					++countBad;
				}
			}
		}
		if (count > 2 && countBad < 3) {
			samplingProbs[imgId] = viewSelectionPriors[imgId] * sumW / count;
		} else if (countBad < 3) {
			samplingProbs[imgId] = viewSelectionPriors[imgId] * __expf(Square(thCost) / (-2.f * 0.4f*0.4f));
		} else {
			samplingProbs[imgId] = 0.f;
		}
	}
	PDF2CDF(samplingProbs, nNumViews);
	unsigned viewWeights[MAX_VIEWS] = {};
	for (int sample = 0; sample < NUM_SAMPLES; ++sample) {
		const float randProb = curand_uniform(randState);
		for (int imgId = 0; imgId < nNumViews; ++imgId) {
			if (samplingProbs[imgId] > randProb) {
				++viewWeights[imgId];
				break;
			}
		}
	}

	// propagate best neighbor plane
	Point4& plane = planes[idx];
	float& cost = costs[idx];
	unsigned newSelectedViews = 0;
	for (int imgId = 0; imgId < nNumViews; ++imgId)
		if (viewWeights[imgId])
			SetBit(newSelectedViews, imgId);
	float finalCosts[8];
	for (int posId = 0; posId < 8; ++posId)
		finalCosts[posId] = valid[posId] ? AggregateMultiViewScores(viewWeights, costArray[posId], nNumViews) : FLT_MAX;
	const int minCostIdx = FindMinIndex(finalCosts, 8);
	float costVector[MAX_VIEWS];
	MultiViewScorePlane<GEOM, MODEL>(refCache, ref, images, depthImages, p, plane, lowDepth, costVector);
	cost = AggregateMultiViewScores(viewWeights, costVector, nNumViews);
	if (finalCosts[minCostIdx] < cost) {
		ASSERT(valid[minCostIdx]);
		plane = LoadPlaneLDG(&planes[positions[minCostIdx]]);
		plane.w() = neighborDepths[minCostIdx];
		cost = finalCosts[minCostIdx];
		selectedViews[idx] = newSelectedViews;
	}
	const float depth = plane.w();

	// refine estimate
	constexpr float perturbationDepth = 0.005f;
	constexpr float perturbationNormal = 0.01f * (float)M_PI;
	const float depthPerturbed = GeneratePerturbedDepth(depth, randState, perturbationDepth);
	const auto viewDirection = [&]() { return ViewDirection(ref, p); };
	const Point3 perturbedNormal = GeneratePerturbedNormal(viewDirection, plane.topLeftCorner<3,1>(), randState, perturbationNormal);
	const Point3 normalRand = GenerateRandomNormal(viewDirection, randState);
	int numValidPlanes = 3;
	Point3 surfaceNormal = Point3::Zero();
	if (valid[0] && valid[1] && valid[2] && valid[3]) {
		// estimate normal from the depth plane through the 4 adjacent neighbors
		SEACAVE::DepthPlaneFit fit;
		fit.Add( 0,-1, LoadPlaneWLDG(&planes[neighborPositions[0]]) - depth);
		fit.Add( 0, 1, LoadPlaneWLDG(&planes[neighborPositions[1]]) - depth);
		fit.Add(-1, 0, LoadPlaneWLDG(&planes[neighborPositions[2]]) - depth);
		fit.Add( 1, 0, LoadPlaneWLDG(&planes[neighborPositions[3]]) - depth);
		surfaceNormal = SEACAVE::NormalFromDepthGradient(RefModel<MODEL>::Get(g_cameras[0]), p, depth, fit.Gradient());
		numValidPlanes = 4;
	}
	constexpr int numPlanes = 4;
	const float depths[numPlanes] = {depthPerturbed, depth, depth, depth};
	const Point3 normals[numPlanes] = {plane.topLeftCorner<3,1>(), perturbedNormal, normalRand, surfaceNormal};
	for (int i = 0; i < numValidPlanes; ++i) {
		Point4 newPlane;
		newPlane.topLeftCorner<3,1>() = normals[i];
		newPlane.w() = depths[i];
		MultiViewScorePlane<GEOM, MODEL>(refCache, ref, images, depthImages, p, newPlane, lowDepth, costVector);
		const float costPlane = AggregateMultiViewScores(viewWeights, costVector, nNumViews);
		if (cost > costPlane) {
			cost = costPlane;
			plane = newPlane;
		}
	}
}

// the pinhole kernels leave the inlining of ProcessPixel to the compiler (whose choice the
// register allocation of their tuned SASS depends on); the model kernels must inline it to keep
// the register budget of their launch bounds
template <bool GEOM>
__device__ void ProcessPixelPinhole(const ImagePixels* images, const ImagePixels* depthImages, Point4* planes, const float* lowDepths, float* costs, RandState* randStates, unsigned* selectedViews, const Point2i& p, const int iter)
{
	ProcessPixel<GEOM, MODEL_PINHOLE>(images, depthImages, planes, lowDepths, costs, randStates, selectedViews, p, iter);
}

// compute the score of the current plane estimate
template <bool GEOM, int MODEL>
__device__ __forceinline__ void InitializePixelScore(const ImagePixels *images, const ImagePixels* depthImages, Point4* planes, const float* lowDepths, float* costs, RandState* randStates, unsigned* selectedViews, const Point2i& p)
{
	const int width = g_cameras[0].size.x();
	const int height = g_cameras[0].size.y();
	if (p.x() >= width || p.y() >= height)
		return;
	const int idx = Point2Idx(p, width);
	if constexpr (MODEL == MODEL_SPHERICAL_REF) {
		// the rows next to the poles are not estimated (ProcessPixel): no depth, never a neighbor
		if (p.y() < nSizeHalfWindow || p.y() >= height - nSizeHalfWindow) {
			planes[idx] = Point4::Zero();
			costs[idx] = FLT_MAX;
			selectedViews[idx] = 0;
			return;
		}
	}
	float lowDepth = 0;
	if (g_params.bLowResProcessed)
		lowDepth = lowDepths[idx];
	// reference-patch state is invariant across views and hypotheses; cache once
	RefPatchCache refCache;
	ComputeRefPatchCache<MODEL>(images[0], p, refCache);
	RefPixel<MODEL> ref(p);
	const auto viewDirection = [&]() { return ViewDirection(ref, p); };
	// initialize estimate randomly if not set
	RandState* randState = &randStates[idx];
	curand_init(1234/*threadIdx.x*/, p.y(), p.x(), randState);
	Point4& plane = planes[idx];
	float depth = plane.w();
	if (depth <= 0.f) {
		// generate random plane
		plane.topLeftCorner<3,1>() = GenerateRandomNormal(viewDirection, randState);
		plane.w() = curand_uniform(randState) * (g_params.fDepthMax - g_params.fDepthMin) + g_params.fDepthMin;
	} else if (plane.topLeftCorner<3,1>().dot(viewDirection()) >= 0.f) {
		// generate random normal
		plane.topLeftCorner<3,1>() = GenerateRandomNormal(viewDirection, randState);
	}
	// compute costs
	const int nNumViews = g_params.nNumViews;
	const int nInitTopK = g_params.nInitTopK;
	float costVector[MAX_VIEWS];
	MultiViewScorePlane<GEOM, MODEL>(refCache, ref, images, depthImages, p, plane, lowDepth, costVector);
	// select best views
	float costVectorSorted[MAX_VIEWS];
	Sort(costVector, costVectorSorted, nNumViews);
	float cost = 0.f;
	for (int i = 0; i < nInitTopK; ++i)
		cost += costVectorSorted[i];
	const float costThreshold = costVectorSorted[nInitTopK - 1];
	unsigned& selectedView = selectedViews[idx];
	selectedView = 0;
	for (int imgId = 0; imgId < nNumViews; ++imgId)
		if (costVector[imgId] <= costThreshold)
			SetBit(selectedView, imgId);
	costs[idx] = cost / nInitTopK;
}

// kernels are GEOM- and MODEL-templated; nvcc emits separate binaries with the
// geom-consistency loop eliminated when off and the camera models resolved;
// runtime params come from __constant__ g_params (uploaded per pyramid level by UploadParams())
template <bool GEOM, int MODEL>
__global__ PATCHMATCHCUDA_LAUNCH_BOUNDS void InitializeScore(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, Point4* planes, const float* lowDepths, float* costs, curandState* randStates, unsigned* selectedViews)
{
	const Point2i p = GetThreadIndex2();
	InitializePixelScore<GEOM, MODEL>((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p);
}

// traverse image in a back/red checkerboard pattern
template <bool GEOM, int MODEL>
__global__ PATCHMATCHCUDA_LAUNCH_BOUNDS void BlackPixelProcess(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, Point4* planes, const float* lowDepths, float* costs, curandState* randStates, unsigned* selectedViews, const int iter)
{
	Point2i p = GetThreadIndex2();
	p.y() = p.y() * 2 + (threadIdx.x % 2 == 0 ? 0 : 1);
	if constexpr (MODEL == MODEL_PINHOLE)
		ProcessPixelPinhole<GEOM>((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p, iter);
	else
		ProcessPixel<GEOM, MODEL>((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p, iter);
}
template <bool GEOM, int MODEL>
__global__ PATCHMATCHCUDA_LAUNCH_BOUNDS void RedPixelProcess(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, Point4* planes, const float* lowDepths, float* costs, curandState* randStates, unsigned* selectedViews, const int iter)
{
	Point2i p = GetThreadIndex2();
	p.y() = p.y() * 2 + (threadIdx.x % 2 == 0 ? 1 : 0);
	if constexpr (MODEL == MODEL_PINHOLE)
		ProcessPixelPinhole<GEOM>((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p, iter);
	else
		ProcessPixel<GEOM, MODEL>((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p, iter);
}

// filter depth/normals
__global__ void FilterPlanes(Point4* planes, float* costs, unsigned* selectedViews, int width, int height)
{
	const Point2i p = GetThreadIndex2();
	if (p.x() >= width || p.y() >= height)
		return;
	const int idx = Point2Idx(p, width);
	// filter estimates if the score is not good enough
	Point4& plane = planes[idx];
	float conf = costs[idx];
	if (plane.w() <= 0 || conf >= g_params.fThresholdKeepCost) {
		conf = 0;
		plane = Point4::Zero();
		selectedViews[idx] = 0;
	}
}
/*----------------------------------------------------------------*/


// upload host cameras / params into their __constant__ symbols on cudaStream
__host__ void PatchMatch::UploadCameras()
{
	const size_t n = cameras.size();
	ASSERT(n <= MAX_VIEWS + 1);
	CUDA_CHECK(cudaMemcpyToSymbolAsync(g_cameraStorage, cameras.data(), sizeof(Camera) * n, 0, cudaMemcpyHostToDevice, cudaStream));
	if (params.sphericalViews == 0)
		return; // every view pinhole: the plane homography kernels
	// the views relative to the reference, for the model kernels
	ModelView views[MAX_VIEWS + 1];
	const Pose& ref = cameras[0].pose;
	for (size_t i = 0; i < n; ++i) {
		const Camera& camera = cameras[i];
		ModelView& view = views[i];
		view.R = camera.pose.R * ref.R.transpose();
		view.t = camera.pose.R * (ref.C - camera.pose.C);
		view.C = -(view.R.transpose() * view.t);
		view.texScale = (params.sphericalViews >> i) & 1u ?
			Point2(1.f / camera.size.x(), 1.f / camera.size.y()) : Point2(1.f, 1.f);
	}
	CUDA_CHECK(cudaMemcpyToSymbolAsync(g_viewStorage, views, sizeof(ModelView) * n, 0, cudaMemcpyHostToDevice, cudaStream));
}
__host__ void PatchMatch::UploadParams()
{
	CUDA_CHECK(cudaMemcpyToSymbolAsync(g_params, &params, sizeof(Params), 0, cudaMemcpyHostToDevice, cudaStream));
}

__host__ void PatchMatch::RunCUDA(float* ptrCostMap, uint32_t* ptrViewsMap)
{
	const unsigned width = cameras[0].size.x();
	const unsigned height = cameras[0].size.y();

	constexpr unsigned BLOCK_W = 32;
	// BLOCK_H is selected by PATCHMATCHCUDA_LB_256_2 (build-time toggle)
	constexpr unsigned BLOCK_H = (BLOCK_W / PATCHMATCHCUDA_BLOCK_H_DIV);

	const dim3 blockSize(BLOCK_W, BLOCK_H, 1);
	const dim3 gridSizeFull((width + BLOCK_W - 1) / BLOCK_W, (height + BLOCK_H - 1) / BLOCK_H, 1);
	const dim3 gridSizeCheckerboard((width + BLOCK_W - 1) / BLOCK_W, ((height / 2) + BLOCK_H - 1) / BLOCK_H, 1);

	// refresh constant-memory params for this pyramid level
	UploadParams();

	// dispatch templated kernels by bGeomConsistency and the camera models
	#define LAUNCH_MODEL(KERNEL, MODEL, GRID, ...) { \
			if (params.bGeomConsistency) \
				KERNEL<true, MODEL><<<GRID, blockSize, 0, cudaStream>>>(__VA_ARGS__); \
			else \
				KERNEL<false, MODEL><<<GRID, blockSize, 0, cudaStream>>>(__VA_ARGS__); \
		}
	#define LAUNCH_GEOM(KERNEL, GRID, ...) { \
			if (params.sphericalViews == 0) \
				LAUNCH_MODEL(KERNEL, MODEL_PINHOLE, GRID, __VA_ARGS__) \
			else if (params.sphericalViews & 1u) \
				LAUNCH_MODEL(KERNEL, MODEL_SPHERICAL_REF, GRID, __VA_ARGS__) \
			else \
				LAUNCH_MODEL(KERNEL, MODEL_PINHOLE_REF, GRID, __VA_ARGS__) \
		}
	// an equirectangular reference wraps the propagation across the seam: the red/black parity of
	// a pixel and its wrapped neighbors agrees only for an even width
	ASSERT(!(params.sphericalViews & 1u) || width % 2 == 0);

	// Pure queueing path: stream ordering on cudaStream already chains kernels;
	// caller (EstimateDepthMap) syncs the stream once before reading results.
	LAUNCH_GEOM(InitializeScore, gridSizeFull, cudaTextureImages, cudaTextureDepths, cudaDepthNormalEstimates, cudaLowDepths, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews);

	for (int iter = 0; iter < params.nEstimationIters; ++iter) {
		LAUNCH_GEOM(BlackPixelProcess, gridSizeCheckerboard, cudaTextureImages, cudaTextureDepths, cudaDepthNormalEstimates, cudaLowDepths, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, iter);
		LAUNCH_GEOM(RedPixelProcess, gridSizeCheckerboard, cudaTextureImages, cudaTextureDepths, cudaDepthNormalEstimates, cudaLowDepths, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, iter);
	}

	#undef LAUNCH_GEOM
	#undef LAUNCH_MODEL

	if (params.fThresholdKeepCost > 0)
		FilterPlanes<<<gridSizeFull, blockSize, 0, cudaStream>>>(cudaDepthNormalEstimates, cudaDepthNormalCosts, cudaSelectedViews, width, height);

	cudaMemcpyAsync(depthNormalEstimates, cudaDepthNormalEstimates, sizeof(Point4) * width * height, cudaMemcpyDeviceToHost, cudaStream);
	if (ptrCostMap)
		cudaMemcpyAsync(ptrCostMap, cudaDepthNormalCosts, sizeof(float) * width * height, cudaMemcpyDeviceToHost, cudaStream);
	if (ptrViewsMap)
		cudaMemcpyAsync(ptrViewsMap, cudaSelectedViews, sizeof(uint32_t) * width * height, cudaMemcpyDeviceToHost, cudaStream);
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS
