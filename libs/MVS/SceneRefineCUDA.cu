/*
* SceneRefineCUDA.cu
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
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
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#include "SceneRefineCUDA.inl"
#include "SceneRefineCommon.h"

#include <cuda_fp16.h>


namespace MVS {

namespace CUDA {


// D E V I C E   H E L P E R S ////////////////////////////////////////

// Read a float pixel from a 32F image surface (the images and the warped image are stored in
// float exactly like the CPU's Image32F: they used to be half-float, whose 2^-11 relative
// quantisation, amplified by 1/sqrt(varA*varB) in low-variance ZNCC windows and by 1/(N.d) at
// grazing pixels, produced per-vertex photometric gradients up to 10x off the CPU's)
__device__ inline float readSurfFloat(cudaSurfaceObject_t surf, int x, int y) {
	float v;
	surf2Dread(&v, surf, x * (int)sizeof(float), y);
	return v;
}

/*----------------------------------------------------------------*/


// K E R N E L S ////////////////////////////////////////////////////

// Project a face's 3 vertices into the camera; false if a vertex is behind the camera or the
// face is back-facing (culled). CPU (MeshRefine::RasterMesh::ProjectVertex) accepts a vertex as
// soon as it is in front of the camera and lets RasterizeTriangleBary clip the triangle to the
// image -- a face straddling the near-plane itself (as opposed to merely straddling the image
// border, which both backends handle via bbox clipping) is rare on a coarse refinement mesh, so
// it is simply dropped here rather than near-plane clipped.
// The CPU's SEACAVE::EdgeFunction(x0,x1,x2) = (x2-x0).cross(x1-x0) = (x2-x0).x*(x1-x0).y -
// (x2-x0).y*(x1-x0).x (Util.inl, TPoint2::cross), evaluated with the same four subtractions,
// two products and one subtraction, every one explicitly rounded so nvcc cannot fuse any of
// them into an fma: the CPU (MSVC /fp:precise) does not, and the two backends' pixel coverage
// must agree bit for bit -- see pixelBary.
__device__ inline float edgeFunction(const Point2& x0, const Point2& x1, const Point2& x2)
{
	const float ax = __fsub_rn(x2.x(), x0.x()), ay = __fsub_rn(x2.y(), x0.y());
	const float bx = __fsub_rn(x1.x(), x0.x()), by = __fsub_rn(x1.y(), x0.y());
	return __fsub_rn(__fmul_rn(ax, by), __fmul_rn(ay, bx));
}

// Back-face cull exactly as TImage::RasterizeTriangleBary (CULL=true): a triangle is kept iff
// its EdgeFunction(p0,p1,p2) > 0 (outward face orientation, y-down pixel coordinates). This
// used to keep the opposite sign, i.e. only the back faces seen through holes: ~0.1 % of the
// faces, empty face maps, and an auto-decimation that always hit its floor.
struct ProjectedFace {
	float z0, z1, z2;  // camera-space depths of the vertices
	Point2 p0, p1, p2; // image-space vertices
	float invArea;     // 1 / EdgeFunction(p0,p1,p2), positive for a kept (front) face
};
// MeshRefine::RasterMesh::ProjectVertex (SceneRefine.cpp): camera-space point, rejected if
// behind the camera, then its pixel coordinates. The CPU does this in double and casts to
// float; the float path here gives the same face maps on Tiny pixel for pixel (raw maps of
// view 3: 0 of 197,624 covered pixels differ), so no double copy of the camera is carried.
__device__ inline bool projectVertex(const Camera& camera, const Point3& X, Point2& pti, float& z)
{
	const Point3 Xc = camera.pose.TransformPointW2C(X);
	if (Xc.z() <= 0.f) return false;
	pti = camera.model.TransformPointC2I(Xc);
	z = Xc.z();
	return true;
}
__device__ inline bool projectFace(const Point3* __restrict__ vertices, const Point3u& face, const Camera& camera, ProjectedFace& pf)
{
	if (!projectVertex(camera, vertices[face.x()], pf.p0, pf.z0) ||
		!projectVertex(camera, vertices[face.y()], pf.p1, pf.z1) ||
		!projectVertex(camera, vertices[face.z()], pf.p2, pf.z2))
		return false;
	const float area = edgeFunction(pf.p0, pf.p1, pf.p2);
	if (area <= 0.f) return false;
	pf.invArea = __fdiv_rn(1.f, area);
	return true;
}

// Perspective-correct barycentric coordinates and depth of pixel (ix,iy) in a projected face;
// false if the pixel centre is outside the triangle. This is TImage::RasterizeTriangleBary's
// inclusion test, expression for expression: each barycentric is the edge function of the pixel
// against the two OTHER vertices times 1/area, rejected as soon as one is negative (the CPU's
// formulation, so the two backends' coverage decisions are the same arithmetic; on Tiny the
// resulting face maps are identical to the CPU's pixel for pixel). Then
// SEACAVE::PerspectiveCorrectBarycentricCoordinates (Util.inl): pb_i = b_i*z_j*z_k, each
// divided by their sum, and MeshRefine::RasterMesh::ComputeDepth (Mesh.h): the left-to-right
// blend of the 3 vertex depths. Every operation is an explicit-rounding intrinsic: nvcc may
// contract a*b+c into an fma differently per kernel (the two rasterizer passes below need the
// bit-identical depth for the same (ix,iy,face); an earlier per-pixel resolve kernel missed
// the key by an ulp for that reason) and the CPU (MSVC /fp:precise) does not contract.
__device__ inline bool pixelBary(int ix, int iy, const ProjectedFace& pf, float& nb0, float& nb1, float& nb2, float& depth)
{
	const Point2 p((float)ix, (float)iy);
	const float b0 = __fmul_rn(edgeFunction(pf.p1, pf.p2, p), pf.invArea);
	if (b0 < 0.f) return false;
	const float b1 = __fmul_rn(edgeFunction(pf.p2, pf.p0, p), pf.invArea);
	if (b1 < 0.f) return false;
	const float b2 = __fmul_rn(edgeFunction(pf.p0, pf.p1, p), pf.invArea);
	if (b2 < 0.f) return false;
	const float z0 = pf.z0, z1 = pf.z1, z2 = pf.z2;
	const float pb0 = __fmul_rn(__fmul_rn(b0, z1), z2);
	const float pb1 = __fmul_rn(__fmul_rn(b1, z0), z2);
	const float pb2 = __fmul_rn(__fmul_rn(b2, z0), z1);
	const float sum = __fadd_rn(__fadd_rn(pb0, pb1), pb2);
	nb0 = __fdiv_rn(pb0, sum); nb1 = __fdiv_rn(pb1, sum); nb2 = __fdiv_rn(pb2, sum);
	depth = __fadd_rn(__fadd_rn(__fmul_rn(nb0, z0), __fmul_rn(nb1, z1)), __fmul_rn(nb2, z2));
	return true;
}

// The projected face's bounding box with ±0.5 padding, clamped to the shared border margin
// (Refine::Border, the same margin the per-pixel window-statistics kernels require -- was a
// hardcoded 5px border); the accepted pixel range is [Border, size-Border) exactly as the CPU's
// per-pixel test in MeshRefine::RasterMesh::Raster (SceneRefine.cpp), so the inclusive bbox ends
// at size-Border-1 (an inclusive clamp at size-Border rasterised one extra row/column the CPU
// rejects, and the warped values it put there leaked into every 7x7 window statistic within
// HalfSize of it). false if the clipped box is empty. The rasterizer and the face-parallel
// photometric accumulation share it so that the second visits exactly the pixels the first could
// have covered -- and, since Border == HalfSize, the photometric kernel's old per-pixel border
// test is implied by the box rather than repeated.
__device__ inline bool faceBBox(const ProjectedFace& pf, const Camera& camera,
	int& ixMin, int& ixMax, int& iyMin, int& iyMax)
{
	const int border = Refine::Border;
	ixMin = max(__float2int_ru(fminf(fminf(pf.p0.x(), pf.p1.x()), pf.p2.x()) - 0.5f), border);
	ixMax = min(__float2int_rd(fmaxf(fmaxf(pf.p0.x(), pf.p1.x()), pf.p2.x()) + 0.5f), camera.size.x() - border - 1);
	iyMin = max(__float2int_ru(fminf(fminf(pf.p0.y(), pf.p1.y()), pf.p2.y()) - 0.5f), border);
	iyMax = min(__float2int_rd(fmaxf(fmaxf(pf.p0.y(), pf.p1.y()), pf.p2.y()) + 0.5f), camera.size.y() - border - 1);
	return ixMin <= ixMax && iyMin <= iyMax;
}

// 1. ProjectMesh — 1D, 1 thread per visible face, launched twice. Pass 1 (RESOLVE=false):
// every covered pixel receives one 64-bit key (depth bits << 32 | position in faceIDs) through
// a single atomicMin, so the nearest face wins and, at exactly equal depth, the face that comes
// first in the camera's face list -- the CPU's RasterMesh keeps the first face it rasterises
// (`depth > z`, strict) walking that same list (octree-traversal order, not ascending ids), so
// this is the same tie-break. Pass 2 (RESOLVE=true): the same threads redo the same arithmetic
// and the one whose key is the pixel's winner writes depth/face/bary -- one writer per pixel,
// no payload race. The previous version did an atomicMin on the depth alone and then wrote
// faceMap/baryMap non-atomically: two faces could both win the depth race in sequence and the
// farther one's payload could land last -- a nondeterministic face map (11 vertices changed
// visibility between two identical Tiny runs).
template <bool RESOLVE>
__global__ void kernelProjectMesh(
	const Point3* __restrict__ vertices,
	const Point3u* __restrict__ faces,
	const uint32_t* __restrict__ faceIDs,
	unsigned long long* __restrict__ projKey,
	float* __restrict__ depthMap,
	uint32_t* __restrict__ faceMap,
	uint16_t* __restrict__ baryMap,
	Camera camera,
	uint32_t numFacesView)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numFacesView) return;

	const uint32_t faceID = faceIDs[tid];
	ProjectedFace pf;
	if (!projectFace(vertices, faces[faceID], camera, pf)) return;

	int ixMin, ixMax, iyMin, iyMax;
	if (!faceBBox(pf, camera, ixMin, ixMax, iyMin, iyMax)) return;

	const int width = camera.size.x();
	for (int iy = iyMin; iy <= iyMax; ++iy) {
		for (int ix = ixMin; ix <= ixMax; ++ix) {
			float nb0, nb1, nb2, depth;
			if (!pixelBary(ix, iy, pf, nb0, nb1, nb2, depth)) continue;
			// depth > 0 (all three z's are), so its bit pattern orders like the float itself
			const unsigned long long key = ((unsigned long long)__float_as_uint(depth) << 32) | (unsigned)tid;
			const int pixIdx = iy * width + ix;
			if (RESOLVE) {
				if (projKey[pixIdx] != key) continue;
				depthMap[pixIdx] = depth;
				faceMap[pixIdx] = faceID;
				baryMap[pixIdx * 3 + 0] = __half_as_ushort(__float2half(nb0));
				baryMap[pixIdx * 3 + 1] = __half_as_ushort(__float2half(nb1));
				baryMap[pixIdx * 3 + 2] = __half_as_ushort(__float2half(nb2));
			} else {
				atomicMin(&projKey[pixIdx], key);
			}
		}
	}
}


// 2. ResolveProjection — 2D, 1 thread per pixel, after both ProjectMesh passes: pixels no face
// covered get 0 / NO_ID / 0; every other pixel must have received its payload from exactly the
// thread that produced the winning key (the host presets faceMap to NO_ID before pass 2, so a
// stale face id from the previous iteration cannot satisfy this check by accident).
__global__ void kernelResolveProjection(
	const uint32_t* __restrict__ faceIDs,
	const unsigned long long* __restrict__ projKey,
	float* __restrict__ depthMap,
	uint32_t* __restrict__ faceMap,
	uint16_t* __restrict__ baryMap,
	int width, int height)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	const int pixIdx = y * width + x;
	const unsigned long long key = projKey[pixIdx];
	if (key == ~0ull) {
		depthMap[pixIdx] = 0.f;
		faceMap[pixIdx] = (uint32_t)-1;
		baryMap[pixIdx * 3 + 0] = baryMap[pixIdx * 3 + 1] = baryMap[pixIdx * 3 + 2] = 0;
		return;
	}
	ASSERT(faceMap[pixIdx] == faceIDs[(uint32_t)key] && __float_as_uint(depthMap[pixIdx]) == (unsigned)(key >> 32));
}


// 3. ImageMeshWarp — 2D, texture + 2 surfaces
__global__ void kernelImageMeshWarp(
	const float* __restrict__ depthMapA,
	const float* __restrict__ depthMapB,
	const uint8_t* __restrict__ keepA, // per-pixel keep-mask of image A, NULL if disabled (keep everything)
	const uint8_t* __restrict__ keepB, // per-pixel keep-mask of image B, NULL if disabled (keep everything)
	uint8_t* __restrict__ mask,
	Camera camA,
	Camera camB,
	cudaTextureObject_t texImageB,
	cudaSurfaceObject_t surfImageProj)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= camA.size.x() || y >= camA.size.y()) return;

	const int pixIdx = y * camA.size.x() + x;
	// invalid pixels stay 0: kernelComputeWindowStats masks them out of every window sum, so their
	// value is never read -- the CPU's imageAB is zero-filled for the same reason
	float convergePix = 0.f;
	uint8_t convergeMask = 0;

	// a masked-out pixel of A never seeds a warp sample, before any back-projection work -- same
	// test as the CPU's MeshRefine::ImageMeshWarp
	if (!keepA || keepA[pixIdx]) {
		const float depthA = depthMapA[pixIdx];
		if (depthA > 0.f) {
			const Point3 X_world = camA.TransformPointI2W(Point2((float)x, (float)y), depthA);
			const Point3 Xc_B = camB.pose.TransformPointW2C(X_world);
			const float pz = Xc_B.z();

			if (pz > 0.f) {
				const Point2 projB = camB.model.TransformPointC2I(Xc_B);
				const float xB = projB.x(), yB = projB.y();
				// B-side border rule shared with the CPU (MeshRefine::IsDepthSimilar): the rounded
				// nearest tap read below must be inside the Refine::Border margin, which is the window
				// half-size the per-pixel window statistics also need around it; the bound is one pixel
				// tighter than the tap itself needs so that any accepted xB/yB rounds into the margin
				// (the old CPU per-corner test independently continue'd past a failing corner instead of
				// rejecting the whole block, letting a corner as close as Border-1 px slip through).
				// The range test is done in FLOAT, before the int conversion, and that is load-bearing:
				// pz can be positive but arbitrarily small at a grazing projection, making xB/yB huge,
				// and __float2int_rd then saturates to INT_MAX -- whereupon the old "ixB + 1 < ..."
				// test overflowed to INT_MIN and PASSED, indexing depthMapB with a wrapped offset
				// (compute-sanitizer: a 4-byte read 3.4 GB past the depth map). Since Border and the
				// sizes are integers, "xB >= Border && xB < size-Border-1" is exactly the floor-based
				// test it replaces, and it rejects huge values and NaN instead of wrapping.
				if (xB >= (float)Refine::Border && yB >= (float)Refine::Border &&
					xB < (float)(camB.size.x() - Refine::Border - 1) &&
					yB < (float)(camB.size.y() - Refine::Border - 1)) {
					const int ixB = __float2int_rd(xB); // floor
					const int iyB = __float2int_rd(yB);
					ASSERT(ixB >= Refine::Border && iyB >= Refine::Border &&
						ixB + 1 < camB.size.x() - Refine::Border && iyB + 1 < camB.size.y() - Refine::Border);
					const int widthB = camB.size.x();
					const int idxB = iyB * widthB + ixB;
					const int k((xB-ixB >= 0.5f ? 1 : 0) + (yB-iyB >= 0.5f ? 2 : 0));
					const int tapIdxB(idxB + (k & 1) + (k >> 1) * widthB);
					const float depthB(depthMapB[tapIdxB]);
					// the same rounded tap read above (shared with the CPU's IsDepthSimilar) also
					// gates the B-side keep-mask
					const bool consistent(depthB > 0.f && depthB*1.0002f >= pz && (!keepB || keepB[tapIdxB]));

					if (consistent) {
						// +0.5: tex2D with non-normalised coords + linear filtering samples texel
						// centres at integer+0.5; CPU TImage::sample treats integer coords as pixel
						// centres, so every tex2D fetch at a CPU-convention coordinate needs this offset
						convergePix = tex2D<float>(texImageB, xB + 0.5f, yB + 0.5f);
						convergeMask = 1;
					}
				}
			}
		}
	}

	surf2Dwrite(convergePix, surfImageProj, x * (int)sizeof(float), y);
	mask[pixIdx] = convergeMask;
}


// 4. ComputeWindowStats — 2D, one thread per pixel: the six MASKED window sums, the two
// rejection gates, ZNCC, its derivative and this pair-direction's reliability sums, all in one
// pass over the 7x7 window. Replaces the former five kernels (mean/var/cov/zncc/dzncc) and the
// six full-image buffers they exchanged. Only successfully warped pixels enter the sums, so a
// window straddling an occlusion boundary is described by the pixels that actually matched
// instead of by image A's own values (the old unmasked statistics drove ZNCC towards 1 exactly
// there). Writes maskOut rather than editing mask in place: the window loop of a neighbouring
// thread is still reading mask.
__global__ void kernelComputeWindowStats(
	const uint8_t* __restrict__ mask,
	uint8_t* __restrict__ maskOut,
	float* __restrict__ dzncc,
	float* __restrict__ znccOut, // parity diagnostic only, may be NULL
	float* __restrict__ confOut, // reliability weight, also a parity diagnostic; NULL unless read
	cudaSurfaceObject_t surfImageA,
	cudaSurfaceObject_t surfImageProj,
	float* __restrict__ blockSums, // 2 floats per block: this block's reliability partials
	float gateMeanDiff, float gateVarRatio,
	int width, int height)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	// the 7x7 windows of the 16x16 threads of a block overlap heavily, so the (image A, warped
	// image B, mask) triple of the 22x22 tile they span is staged in shared memory once instead
	// of being re-read 49 times per thread from global/surface memory -- the naive version cost
	// 1.4-1.8x the wall of the five separate kernels this one replaces
	constexpr int Block = 16;
	constexpr int Tile = Block + 2*Refine::HalfSize;
	__shared__ float sA[Tile*Tile], sB[Tile*Tile], sW[Tile*Tile];
	const int x0 = blockIdx.x * Block - Refine::HalfSize;
	const int y0 = blockIdx.y * Block - Refine::HalfSize;
	for (int i = threadIdx.y * Block + threadIdx.x; i < Tile*Tile; i += Block*Block) {
		const int gx = x0 + i % Tile, gy = y0 + i / Tile;
		float a(0.f), b(0.f), w(0.f);
		// invalid samples are staged as zeros so the accumulation below needs no branch: they
		// contribute nothing to any of the six sums, which is exactly what masking them means
		if (gx >= 0 && gy >= 0 && gx < width && gy < height && mask[gy * width + gx] == 1) {
			a = readSurfFloat(surfImageA, gx, gy);
			b = readSurfFloat(surfImageProj, gx, gy);
			w = 1.f;
		}
		sA[i] = a; sB[i] = b; sW[i] = w;
	}
	__syncthreads();

	// per-thread contribution to this block's reliability sums; stays 0 for every pixel outside
	// the image (the grid is rounded up to the block size), outside the valid border, unmasked or
	// gated -- exactly the pixels the CPU's ScoreMesh S skips; no early "return" here so every
	// thread in the block reaches the reduction below
	float contribR(0.f), contribRZ(0.f);
	if (x < width && y < height) {
		const int pixIdx = y * width + x;
		float dz(0.f), zn(0.f), cf(0.f);
		uint8_t valid(0);
		if (x >= Refine::HalfSize && y >= Refine::HalfSize &&
			x < width - Refine::HalfSize && y < height - Refine::HalfSize && mask[pixIdx] == 1)
		{
			const int centre = (threadIdx.y + Refine::HalfSize) * Tile + threadIdx.x + Refine::HalfSize;
			float n(0.f), sumA(0.f), sumB(0.f), sumAA(0.f), sumBB(0.f), sumAB(0.f);
			for (int dy = -Refine::HalfSize; dy <= Refine::HalfSize; ++dy) {
				const int row = centre + dy*Tile;
				for (int dx = -Refine::HalfSize; dx <= Refine::HalfSize; ++dx) {
					const int t = row + dx;
					const float a = sA[t], b = sB[t];
					n += sW[t]; sumA += a; sumB += b; sumAA += a*a; sumBB += b*b; sumAB += a*b;
				}
			}
			Refine::WindowStats s;
			if (Refine::WindowStatsFromSums(n, sumA, sumB, sumAA, sumBB, sumAB, gateMeanDiff, gateVarRatio, s)) {
				Refine::ZnccAndDerivative(s, n, sA[centre], sB[centre], zn, dz, cf);
				valid = 1;
				contribR = cf;
				contribRZ = cf * (1.f - zn);
			}
		}
		dzncc[pixIdx] = dz;
		maskOut[pixIdx] = valid;
		if (znccOut) znccOut[pixIdx] = zn;
		if (confOut) confOut[pixIdx] = cf;
	}

	// shared-memory block reduction (a fixed tree over a fixed thread mapping, so its result does
	// not depend on the schedule); sized for the 16x16 block LaunchComputeWindowStats always
	// launches. The block's two partials go to its OWN slot rather than into a global atomicAdd:
	// kernelReduceBlockSums below then folds the slots in block order. The atomicAdd version was
	// the last float accumulation in the refinement whose order varied run to run, and it showed:
	// with the per-vertex terms made reproducible, two identical Tiny runs still printed S deltas
	// differing in the third digit even though the mesh they produced was byte-identical.
	__shared__ float sSumR[256];
	__shared__ float sSumRZ[256];
	const int tid = threadIdx.y * blockDim.x + threadIdx.x;
	sSumR[tid] = contribR;
	sSumRZ[tid] = contribRZ;
	__syncthreads();
	for (int stride = (blockDim.x * blockDim.y) >> 1; stride > 0; stride >>= 1) {
		if (tid < stride) {
			sSumR[tid] += sSumR[tid + stride];
			sSumRZ[tid] += sSumRZ[tid + stride];
		}
		__syncthreads();
	}
	if (tid == 0) {
		const int idxBlock = blockIdx.y * gridDim.x + blockIdx.x;
		blockSums[idxBlock*2 + 0] = sSumR[0];
		blockSums[idxBlock*2 + 1] = sSumRZ[0];
	}
}


// 4b. ReduceBlockSums — a single block, right after ComputeWindowStats: adds this
// pair-direction's per-block partials into the two persistent ScoreMesh accumulators in a fixed
// order (each thread walks a strided, fixed subset, then a fixed shared-memory tree), so S is
// the same number on every run. The pair-directions themselves are already ordered: their kernel
// launches serialize on the default stream.
__global__ void kernelReduceBlockSums(
	const float* __restrict__ blockSums,
	float* __restrict__ sumR,
	float* __restrict__ sumRZ,
	uint32_t numBlocks)
{
	constexpr int Threads = 256;
	__shared__ float sR[Threads];
	__shared__ float sRZ[Threads];
	const int tid = threadIdx.x;
	float r(0.f), rz(0.f);
	for (uint32_t i = (uint32_t)tid; i < numBlocks; i += Threads) {
		r += blockSums[i*2 + 0];
		rz += blockSums[i*2 + 1];
	}
	sR[tid] = r;
	sRZ[tid] = rz;
	__syncthreads();
	for (int stride = Threads >> 1; stride > 0; stride >>= 1) {
		if (tid < stride) {
			sR[tid] += sR[tid + stride];
			sRZ[tid] += sRZ[tid + stride];
		}
		__syncthreads();
	}
	if (tid == 0) {
		sumR[0] += sR[0];
		sumRZ[0] += sRZ[0];
	}
}


// fetch one texel of a float texture at its exact centre (tex2D pixel-centre convention, +0.5),
// or 0 for a tap outside the image -- the out-of-range rule BilinearGradient below matches
__device__ inline float texelOrZero(cudaTextureObject_t tex, int x, int y, int width, int height)
{
	if (x < 0 || x >= width || y < 0 || y >= height)
		return 0.f;
	return tex2D<float>(tex, (float)x + 0.5f, (float)y + 0.5f);
}

// derivative of the bilinear reconstruction of the image texture at (px,py), from four
// point-sampled texel fetches (never the texture's own linear filtering, which would blend
// VALUES rather than give the four taps a derivative is built from) -- the same taps, weights
// and out-of-range convention (a tap outside the image contributes nothing) as the CPU's
// MeshRefine::BilinearGradient (SceneRefine.cpp), so this is the derivative of the exact value
// the warp itself samples with the Linear sampler, not of a smoothed stencil estimate of it
__device__ inline void bilinearGradient(cudaTextureObject_t texImage, int width, int height, float px, float py, float& gx, float& gy)
{
	const int x0 = __float2int_rd(px), y0 = __float2int_rd(py);
	const float dx = px - (float)x0, dy = py - (float)y0;
	const float v00 = texelOrZero(texImage, x0,   y0,   width, height);
	const float v01 = texelOrZero(texImage, x0+1, y0,   width, height);
	const float v10 = texelOrZero(texImage, x0,   y0+1, width, height);
	const float v11 = texelOrZero(texImage, x0+1, y0+1, width, height);
	gx = (1.f-dy)*(v01-v00) + dy*(v11-v10);
	gy = (1.f-dx)*(v10-v00) + dx*(v11-v01);
}

// 9a. the per-pixel half of the photometric gradient, factored out of the two kernels below so
// that it is a PURE FUNCTION OF THE PIXEL: nothing it returns depends on which thread computes it
// or in what order, which is what lets the accumulation be atomic-free and bit-reproducible.
// The caller has already established that this pixel is inside the valid border, masked in, and
// covered by the face whose normal it passes in. Returns false if the pixel contributes nothing.
struct PhotoPixel {
	float g; // the scalar the face's three corners share before their barycentrics (the CPU's sg)
	float foot; // scene units per pixel at camera A (Camera::GetFootprintWorld = depth/focalLength)
};
__device__ inline bool computePhotoPixel(
	int x, int y, int pixIdx, float depth, const Point3& normal,
	const float* __restrict__ dznccMap,
	const Camera& camA,
	const Camera& camB,
	cudaTextureObject_t texImageB, // sampled directly in bilinear-derivative mode (nImageGradient == 3)
	cudaTextureObject_t texGradXB, // precomputed stencil (ComputeRefineImageGradient); unused in that mode
	cudaTextureObject_t texGradYB,
	bool bBilinearGrad,
	float regScale,
	PhotoPixel& out)
{
	// View direction in world space (normalized)
	const Point3 camRay = camA.model.TransformPointI2C(Point2((float)x, (float)y));
	const Point3 worldDir = camA.pose.R.transpose() * camRay;
	const Point3 viewDir = worldDir.normalized();

	const float viewDotNormal = viewDir.dot(normal);
	if (viewDotNormal > -0.1f)
		return false;

	// Back-project to 3D and forward-project to camera B
	const Point3 X_world = camA.TransformPointI2W(Point2((float)x, (float)y), depth);
	const Point3 Xc_B = camB.pose.TransformPointW2C(X_world);
	const float pz = Xc_B.z();

	// mask==1 means the warp already ran this exact back-/forward-projection chain for this pixel
	// and rejected a non-positive depth in B, so pz can only be positive here; the producer is the
	// warp on both backends (kernelImageMeshWarp above, MeshRefine::ImageMeshWarp on the CPU)
	ASSERT(pz > 0.f);
	const Point2 projB = camB.model.TransformPointC2I(Xc_B);

	// Jacobian d(u,v)/d(X_world): KR = K * R
	const Matrix3 KR = camB.model.K() * camB.pose.R;
	const Point3 p = camB.model.K() * Xc_B; // raw projection before perspective divide
	const float pz2 = pz * pz;

	// du/dX = (KR.row(0)*pz - KR.row(2)*px) / pz², same for dv/dX
	const Point3 dudX = (KR.row(0).transpose() * pz - KR.row(2).transpose() * p.x()) / pz2;
	const Point3 dvdX = (KR.row(1).transpose() * pz - KR.row(2).transpose() * p.y()) / pz2;

	// Image derivatives at the projected point: either bilinear samples of the precomputed
	// gradient images (ComputeRefineImageGradient, the CPU's estimator and sampling exactly;
	// forward differences of the image texture used to give a per-pixel magnitude that differed
	// from the CPU by a factor of 0.6-5), or (bBilinearGrad) the derivative of the bilinear
	// interpolant of the raw image itself, with no precomputed stencil to sample
	// (+0.5: tex2D pixel-centre convention, see ImageMeshWarp)
	float dx, dy;
	if (bBilinearGrad) {
		bilinearGradient(texImageB, camB.size.x(), camB.size.y(), projB.x(), projB.y(), dx, dy);
	} else {
		dx = tex2D<float>(texGradXB, projB.x() + 0.5f, projB.y() + 0.5f);
		dy = tex2D<float>(texGradYB, projB.x() + 0.5f, projB.y() + 0.5f);
	}

	// 3D gradient = dzncc * J^T * [dx, dy]
	const Point3 gradDir = dx * dudX + dy * dvdX;
	const float dz = dznccMap[pixIdx];
	const Point3 grad = dz * gradDir;

	// Project gradient along view direction, scale by 1/dot(viewDir, normal)
	const float projMag = grad.dot(viewDir) / viewDotNormal;

	out.g = regScale * projMag;
	out.foot = depth / camA.model.f.x();
	return true;
}


// 9c. AccumulateFacePhoto — 1D, one thread per mesh face; the first half of the atomic-free
// photometric accumulation. Each thread reduces ITS OWN face's pixels into registers and writes
// one private slot per output, so there is not a single atomic and not a single float sum whose
// order depends on the schedule. That is the whole point: float addition is not associative, so
// the atomicAdd scatter this replaces produced a different per-vertex gradient on every run (the
// campaign's CUDA noise floor was ~0.001-0.002 F1 against 0.0001-0.0009 on the CPU, and two
// identical Tiny runs diverged by iteration 7 of the first scale).
//
// The thread projects its face and walks its clipped bounding box exactly as kernelProjectMesh
// did, in a fixed row-then-column order, and keeps the pixels whose faceMap entry is this face --
// which is also what makes iterating over ALL faces (rather than over this view's cameraFaces)
// correct AND cheap: faceMap only ever holds ids the rasterizer wrote, so a face outside this
// camera's list can never match a pixel, and every thread writes its slot (zeros included) so no
// per-pair-direction memset is needed either.
__global__ void kernelAccumulateFacePhoto(
	const Point3* __restrict__ vertices,
	const Point3u* __restrict__ faces,
	const Point3* __restrict__ normals,
	const float* __restrict__ depthMap,
	const uint32_t* __restrict__ faceMap,
	const uint16_t* __restrict__ baryMap,
	const float* __restrict__ dznccMap,
	const uint8_t* __restrict__ mask,
	float* __restrict__ faceAcc, // 3 per face: Sum over the face's pixels of g_p*b_c, one per corner
	float* __restrict__ facePixels, // 1 per face: how many pixels contributed (0 = the face contributed nothing)
	float* __restrict__ faceFoot, // 1 per face: min footprint over the face's pixels; only read where facePixels > 0
	float* __restrict__ sgMap, // WP2 parity diagnostic: per-pixel scalar handed to the 3 vertices (CPU's sg); NULL in production
	Camera camA,
	Camera camB,
	cudaTextureObject_t texImageB,
	cudaTextureObject_t texGradXB,
	cudaTextureObject_t texGradYB,
	bool bBilinearGrad,
	float regScale,
	uint32_t numFaces)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numFaces) return;

	float sum0 = 0.f, sum1 = 0.f, sum2 = 0.f;
	float pixels = 0.f, foot = 0.f;

	ProjectedFace pf;
	int ixMin, ixMax, iyMin, iyMax;
	if (projectFace(vertices, faces[tid], camA, pf) && faceBBox(pf, camA, ixMin, ixMax, iyMin, iyMax)) {
		const Point3& normal = normals[tid];
		const int width = camA.size.x();
		for (int iy = iyMin; iy <= iyMax; ++iy) {
			for (int ix = ixMin; ix <= ixMax; ++ix) {
				const int pixIdx = iy * width + ix;
				if (faceMap[pixIdx] != (uint32_t)tid || mask[pixIdx] != 1)
					continue;
				// this pixel's winning rasterizer key was this face's, so its payload is this face's
				// perspective-correct depth and barycentrics -- a positive depth by construction
				const float depth = depthMap[pixIdx];
				ASSERT(depth > 0.f);
				PhotoPixel pp;
				if (!computePhotoPixel(ix, iy, pixIdx, depth, normal, dznccMap,
						camA, camB, texImageB, texGradXB, texGradYB, bBilinearGrad, regScale, pp))
					continue;
				if (sgMap)
					sgMap[pixIdx] = pp.g;
				const float bary0 = __half2float(*reinterpret_cast<const __half*>(&baryMap[pixIdx * 3 + 0]));
				const float bary1 = __half2float(*reinterpret_cast<const __half*>(&baryMap[pixIdx * 3 + 1]));
				const float bary2 = __half2float(*reinterpret_cast<const __half*>(&baryMap[pixIdx * 3 + 2]));
				sum0 += pp.g * bary0;
				sum1 += pp.g * bary1;
				sum2 += pp.g * bary2;
				// per-vertex footprint at camera A, scene units per pixel: min over every
				// contributing pixel of every pair-direction, matching the CPU's min-of-mins
				// (MeshRefine::ComputePhotometricGradient/ThProcessPair). min is exact and
				// associative, so this half of it is reproducible for free.
				foot = pixels == 0.f ? pp.foot : fminf(foot, pp.foot);
				pixels += 1.f;
			}
		}
	}

	faceAcc[tid*3 + 0] = sum0;
	faceAcc[tid*3 + 1] = sum1;
	faceAcc[tid*3 + 2] = sum2;
	facePixels[tid] = pixels;
	faceFoot[tid] = foot;
}


// 9d. GatherVertexPhoto — 1D, one thread per vertex; the second half. It walks the vertex's
// incident faces in the fixed order Mesh::ListIncidentFaces produced (uploaded per scale, see
// MeshRefineCUDA::ListVertexFacesPost) and folds in each face's private accumulator, so a
// vertex's sum is a fixed sequence of float additions. Only this thread writes this vertex, so
// the per-pair-direction bookkeeping kernelUpdatePhotoGradNorm used to do in a separate launch
// (the +1 on photoGradNorm for a direction that saw the vertex) folds in here for free -- and
// with it the whole photoGradPixels buffer, which now lives in the `pixels` register below.
//
// Every corner whose vertex id matches is folded in, not just the first: a degenerate face
// listing the same vertex twice handed that vertex both corners' shares under the old scatter,
// and vertexFaces lists such a face only once.
__global__ void kernelGatherVertexPhoto(
	const Point3u* __restrict__ faces,
	const Point3* __restrict__ normals,
	const uint32_t* __restrict__ vertFaces,
	const uint32_t* __restrict__ vertFaceSizes,
	const uint32_t* __restrict__ vertFacePointers,
	const float* __restrict__ faceAcc,
	const float* __restrict__ facePixels,
	const float* __restrict__ faceFoot,
	Point3* __restrict__ photoGrad,
	float* __restrict__ photoGradNorm,
	float* __restrict__ footprint,
	uint32_t numVertices)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numVertices) return;

	const uint32_t ptr = vertFacePointers[tid];
	const uint32_t numIncident = vertFaceSizes[tid];
	Point3 grad = Point3::Zero();
	float pixels = 0.f, foot = 0.f;
	for (uint32_t i = 0; i < numIncident; ++i) {
		const uint32_t idxFace = vertFaces[ptr + i];
		const Point3u& face = faces[idxFace];
		const uint32_t fv[3] = { face.x(), face.y(), face.z() };
		// the adjacency is the transpose of the face list: a face this vertex is incident to must
		// name it back, or the two uploads describe different meshes
		ASSERT(fv[0] == (uint32_t)tid || fv[1] == (uint32_t)tid || fv[2] == (uint32_t)tid);
		const float facePix = facePixels[idxFace];
		if (facePix == 0.f)
			continue; // nothing was accumulated, so every other slot of this face is a zero
		const Point3& normal = normals[idxFace];
		for (int c = 0; c < 3; ++c) {
			if (fv[c] != (uint32_t)tid)
				continue;
			grad += normal * faceAcc[idxFace*3 + c];
			foot = pixels == 0.f ? faceFoot[idxFace] : fminf(foot, faceFoot[idxFace]);
			pixels += facePix;
		}
	}
	if (pixels == 0.f)
		return; // this pair-direction saw nothing of this vertex

	photoGrad[tid] += grad;
	// exactly the CPU's `photoGradNorm[idxVert] += 1.f;`: one count per pair-direction that
	// contributed at least one pixel to this vertex
	photoGradNorm[tid] += 1.f;
	if (foot < footprint[tid])
		footprint[tid] = foot;
}


// 10. FinalizePhotoGrad — 1D, once after every pair-direction of this ScoreMesh() has run:
// resolves the footprint sentinel now that photoGradNorm holds its final value (contract:
// footprint[v] > 0 exactly where photoGradNorm[v] > 0, matches CPU ScoreMesh); doing it inside a
// per-pair-direction launch would clobber the sentinel for a vertex a later direction is the
// first to touch. The per-pair-direction half this kernel used to carry (the +1 on
// photoGradNorm) now lives in kernelGatherVertexPhoto, which already holds that direction's
// pixel count in a register.
__global__ void kernelFinalizePhotoGrad(
	const float* __restrict__ photoGradNorm,
	float* __restrict__ footprint,
	uint32_t numVertices)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numVertices) return;
	if (photoGradNorm[tid] == 0.f)
		footprint[tid] = 0.f;
}


// 11. ComputeSmoothnessGradient — 1D
__global__ void kernelComputeSmoothnessGradient(
	const Point3* __restrict__ vertices,
	const uint32_t* __restrict__ vertVertices,
	const uint32_t* __restrict__ vertSizes,
	const uint32_t* __restrict__ vertPointers,
	const uint8_t* __restrict__ vertBoundary,
	Point3* __restrict__ smoothGrad,
	uint32_t numVertices,
	uint8_t mode)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numVertices) return;

	// a boundary vertex's own gradient is zeroed (matches CPU
	// MeshRefine::ComputeSmoothnessGradient1/2, SceneRefine.cpp); vertSizes[] always holds the
	// TRUE valence for every vertex (boundary or not) so that OTHER vertices' valence-weighted
	// sum below stays correct when one of their neighbours happens to be a boundary vertex, whose
	// valence the weight term divides by
	if (vertBoundary[tid]) {
		smoothGrad[tid] = Point3::Zero();
		return;
	}

	const uint32_t numNeighbors = vertSizes[tid];
	if (numNeighbors == 0) {
		smoothGrad[tid] = Point3::Zero();
		return;
	}
	const uint32_t ptr = vertPointers[tid];

	// (1/N)*sum(neighbors) - vertex, the CPU's sign convention
	// (ComputeSmoothnessGradient1/2), so that the two backends' smoothGrad1/2
	// dumps compare directly; CombineAllGradients subtracts the rigidity term
	const float invN = 1.f / (float)numNeighbors;
	Point3 result = -vertices[tid];
	float totalWeight = 1.f;
	for (uint32_t i = 0; i < numNeighbors; ++i) {
		const uint32_t ni = vertVertices[ptr + i];
		result += vertices[ni] * invN;
		if (mode != 0) {
			// Valence-weighted: accumulate 1/(Ni*N) where Ni = TRUE valence of neighbor
			// (boundary neighbours are included, exactly as CPU's vertexVertices[ni].GetSize());
			// adjacency is symmetric, so a neighbour lists this vertex back and Ni >= 1
			ASSERT(vertSizes[ni] > 0);
			totalWeight += invN / (float)vertSizes[ni];
		}
	}
	if (mode != 0)
		result /= totalWeight;
	smoothGrad[tid] = result;
}


// 12. CombineGradients — 1D
__global__ void kernelCombineGradients(
	Point3* __restrict__ photoGrad,
	const float* __restrict__ photoGradNorm,
	const Point3* __restrict__ smoothGrad,
	uint32_t numVertices,
	float smoothWeight)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numVertices) return;

	// photoGradNorm (c_v) is exactly zero at a vertex no pair-direction saw
	const float norm = photoGradNorm ? photoGradNorm[tid] : 1.f;
	if (norm > 0.f)
		photoGrad[tid] = photoGrad[tid] / norm + smoothWeight * smoothGrad[tid];
	else
		photoGrad[tid] = smoothWeight * smoothGrad[tid];
}


// 13. CombineAllGradients — 1D
__global__ void kernelCombineAllGradients(
	Point3* __restrict__ photoGrad,
	const float* __restrict__ photoGradNorm,
	const Point3* __restrict__ smoothGrad1,
	const Point3* __restrict__ smoothGrad2,
	uint32_t numVertices,
	float rigidity,
	float elasticity)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numVertices) return;

	// NULL normalizer: see kernelCombineGradients
	const float norm = photoGradNorm ? photoGradNorm[tid] : 1.f;
	if (norm > 0.f)
		photoGrad[tid] = photoGrad[tid] / norm + elasticity * smoothGrad2[tid] - rigidity * smoothGrad1[tid];
	else
		photoGrad[tid] = elasticity * smoothGrad2[tid] - rigidity * smoothGrad1[tid];
}


// 14. ComputeFaceNormal — 1D, 1 thread per face
__global__ void kernelComputeFaceNormal(
	const Point3* __restrict__ vertices,
	const Point3u* __restrict__ faces,
	Point3* __restrict__ normals,
	uint32_t numFaces)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numFaces) return;
	const Point3u& face = faces[tid];
	const Point3 v0 = vertices[face.x()];
	const Point3 v1 = vertices[face.y()];
	const Point3 v2 = vertices[face.z()];
	const Point3 e1 = v1 - v0;
	const Point3 e2 = v2 - v0;
	const Point3 n = e1.cross(e2);
	normals[tid] = n.normalized();
}
/*----------------------------------------------------------------*/


// H O S T   L A U N C H E R S ////////////////////////////////////////

void LaunchProjectMesh(
	const Point3* vertices, const Point3u* faces, const uint32_t* faceIDs,
	unsigned long long* projKey, float* depthMap, uint32_t* faceMap, uint16_t* baryMap,
	const Camera& camera, uint32_t numFacesView, bool resolve)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numFacesView + blockSize - 1) / blockSize;
	if (resolve)
		kernelProjectMesh<true><<<numBlocks, blockSize>>>(
			vertices, faces, faceIDs, projKey, depthMap, faceMap, baryMap, camera, numFacesView);
	else
		kernelProjectMesh<false><<<numBlocks, blockSize>>>(
			vertices, faces, faceIDs, projKey, depthMap, faceMap, baryMap, camera, numFacesView);
}

void LaunchResolveProjection(
	const uint32_t* faceIDs, const unsigned long long* projKey,
	float* depthMap, uint32_t* faceMap, uint16_t* baryMap, int width, int height)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelResolveProjection<<<grid, block>>>(faceIDs, projKey, depthMap, faceMap, baryMap, width, height);
}

void LaunchImageMeshWarp(
	const float* depthMapA, const float* depthMapB,
	const uint8_t* keepA, const uint8_t* keepB, uint8_t* mask,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB,
	cudaSurfaceObject_t surfImageProj)
{
	const dim3 block(16, 16);
	const dim3 grid((camA.size.x() + block.x - 1) / block.x, (camA.size.y() + block.y - 1) / block.y);
	kernelImageMeshWarp<<<grid, block>>>(depthMapA, depthMapB, keepA, keepB, mask, camA, camB, texImageB, surfImageProj);
}

void LaunchComputeWindowStats(
	const uint8_t* mask, uint8_t* maskOut, float* dzncc, float* zncc, float* conf,
	cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj,
	float* sumR, float* sumRZ, float* blockSums, float gateMeanDiff, float gateVarRatio, int width, int height)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputeWindowStats<<<grid, block>>>(mask, maskOut, dzncc, zncc, conf,
		surfImageA, surfImageProj, blockSums, gateMeanDiff, gateVarRatio, width, height);
	// fold this pair-direction's per-block partials into S's accumulators in block order
	kernelReduceBlockSums<<<1, 256>>>(blockSums, sumR, sumRZ, grid.x*grid.y);
}

void LaunchAccumulateFacePhoto(
	const Point3* vertices, const Point3u* faces, const Point3* normals,
	const float* depthMap, const uint32_t* faceMap, const uint16_t* baryMap,
	const float* dzncc, const uint8_t* mask,
	float* faceAcc, float* facePixels, float* faceFoot, float* sgMap,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB, cudaTextureObject_t texGradXB, cudaTextureObject_t texGradYB,
	bool bBilinearGrad, float regScale, uint32_t numFaces)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numFaces + blockSize - 1) / blockSize;
	kernelAccumulateFacePhoto<<<numBlocks, blockSize>>>(
		vertices, faces, normals, depthMap, faceMap, baryMap, dzncc, mask,
		faceAcc, facePixels, faceFoot, sgMap, camA, camB, texImageB, texGradXB, texGradYB, bBilinearGrad, regScale, numFaces);
}

void LaunchGatherVertexPhoto(
	const Point3u* faces, const Point3* normals,
	const uint32_t* vertFaces, const uint32_t* vertFaceSizes, const uint32_t* vertFacePointers,
	const float* faceAcc, const float* facePixels, const float* faceFoot,
	Point3* photoGrad, float* photoGradNorm, float* footprint, uint32_t numVertices)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelGatherVertexPhoto<<<numBlocks, blockSize>>>(
		faces, normals, vertFaces, vertFaceSizes, vertFacePointers,
		faceAcc, facePixels, faceFoot, photoGrad, photoGradNorm, footprint, numVertices);
}

void LaunchFinalizePhotoGrad(const float* photoGradNorm, float* footprint, uint32_t numVertices)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelFinalizePhotoGrad<<<numBlocks, blockSize>>>(photoGradNorm, footprint, numVertices);
}

void LaunchComputeSmoothnessGradient(
	const Point3* vertices, const uint32_t* vertVertices, const uint32_t* vertSizes, const uint32_t* vertPointers,
	const uint8_t* vertBoundary, Point3* smoothGrad, uint32_t numVertices, uint8_t mode)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelComputeSmoothnessGradient<<<numBlocks, blockSize>>>(vertices, vertVertices, vertSizes, vertPointers, vertBoundary, smoothGrad, numVertices, mode);
}

void LaunchCombineGradients(Point3* photoGrad, const float* photoGradNorm, const Point3* smoothGrad, uint32_t numVertices, float smoothWeight)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelCombineGradients<<<numBlocks, blockSize>>>(photoGrad, photoGradNorm, smoothGrad, numVertices, smoothWeight);
}

void LaunchCombineAllGradients(
	Point3* photoGrad, const float* photoGradNorm, const Point3* smoothGrad1, const Point3* smoothGrad2,
	uint32_t numVertices, float rigidity, float elasticity)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelCombineAllGradients<<<numBlocks, blockSize>>>(photoGrad, photoGradNorm, smoothGrad1, smoothGrad2, numVertices, rigidity, elasticity);
}

void LaunchComputeFaceNormal(
	const Point3* vertices, const Point3u* faces, Point3* normals, uint32_t numFaces)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numFaces + blockSize - 1) / blockSize;
	kernelComputeFaceNormal<<<numBlocks, blockSize>>>(vertices, faces, normals, numFaces);
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS
