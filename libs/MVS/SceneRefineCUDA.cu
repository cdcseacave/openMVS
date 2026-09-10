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

#include <float.h>


namespace MVS {

namespace CUDA {


// D E V I C E   H E L P E R S ////////////////////////////////////////

// read a float pixel from a 32F image surface (the images and the warped image are stored in
// float exactly like the CPU's Image32F: half floats lose too much in low-variance ZNCC windows
// and at grazing pixels)
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

// back-face cull exactly as TImage::RasterizeTriangleBary (CULL=true): a triangle is kept iff
// its EdgeFunction(p0,p1,p2) > 0 (outward face orientation, y-down pixel coordinates)
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

// 1. ProjectMesh — 1D, one thread per mesh face, launched twice. Pass 1 (RESOLVE=false):
// every covered pixel receives one 64-bit key (depth bits << 32 | face id) through a single
// atomicMin, so the nearest face wins and, at exactly equal depth, the lower face id. Pass 2
// (RESOLVE=true): the same threads redo the same arithmetic and the one whose key is the pixel's
// winner writes depth/face/bary -- one writer per pixel, no payload race, hence a deterministic
// face map. The whole mesh is rasterized into every view: a face behind the camera or off the
// image costs its thread three projections and an early exit, cheaper than the host-side
// frustum cull that used to shortlist the faces (an octree over the mesh rebuilt for every
// evaluation, plus one face-list upload per view, while the GPU sat idle). The CPU's RasterMesh
// keeps the first face it rasterises in that cull's (octree-traversal) order, so the two
// backends can differ only on an exact depth tie: a pixel centre exactly on a shared edge, where
// either face hands the pixel to the same two vertices.
// Pass 2 also leaves, per view, one bit per face -- set iff the face wrote at least one pixel
// -- collected by a warp ballot (no atomics): the photometric accumulation of every
// pair-direction that has this view as its reference reads them to skip the faces the view does
// not see before touching them at all, which in a scene of hundreds of views is most of the mesh
// for every view. The barycentrics are not stored: the accumulation recomputes them (pixelBary,
// the same arithmetic) where it needs them, in float like the CPU's BaryMap, instead of the
// half-precision map that used to cost 8 bytes per pixel of every view.
template <bool RESOLVE>
__global__ void kernelProjectMesh(
	const Point3* __restrict__ vertices,
	const Point3u* __restrict__ faces,
	unsigned long long* __restrict__ projKey,
	float* __restrict__ depthMap,
	uint32_t* __restrict__ faceMap,
	uint32_t* __restrict__ ownerBits, // RESOLVE only, may be NULL: bit f set iff face f wrote at least one pixel
	Camera camera,
	uint32_t numFaces)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	// no early return before the ballot at the end: every lane of the warp has to reach it
	bool owner = false;
	ProjectedFace pf;
	int ixMin, ixMax, iyMin, iyMax;
	if (tid < (int)numFaces && projectFace(vertices, faces[tid], camera, pf) && faceBBox(pf, camera, ixMin, ixMax, iyMin, iyMax)) {
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
					faceMap[pixIdx] = (uint32_t)tid;
					owner = true;
				} else {
					atomicMin(&projKey[pixIdx], key);
				}
			}
		}
	}
	if (RESOLVE && ownerBits) {
		// one word per warp of 32 consecutive faces, stored by lane 0: the buffer holds
		// (numFaces+31)/32 words per view, so the warps past the last face store nothing
		const unsigned ballot = __ballot_sync(0xffffffffu, owner);
		if ((tid & 31) == 0 && tid < (int)numFaces)
			ownerBits[tid >> 5] = ballot;
	}
}


#ifdef _DEBUG
// 2. CheckProjection — 2D, 1 thread per pixel, Debug only, after both ProjectMesh passes: every
// covered pixel must hold the payload of exactly the face that produced its winning key (the
// host presets faceMap to NO_ID and depthMap to 0 before pass 2, so a stale value from the
// previous evaluation cannot satisfy this by accident)
__global__ void kernelCheckProjection(
	const unsigned long long* __restrict__ projKey,
	const float* __restrict__ depthMap,
	const uint32_t* __restrict__ faceMap,
	int width, int height)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	const int pixIdx = y * width + x;
	const unsigned long long key = projKey[pixIdx];
	if (key == ~0ull) {
		ASSERT(faceMap[pixIdx] == (uint32_t)-1);
	} else {
		ASSERT(faceMap[pixIdx] == (uint32_t)key && __float_as_uint(depthMap[pixIdx]) == (unsigned)(key >> 32));
	}
}
#endif


// 3. CompactOwners — the dense per-view lists of the faces the rasterizer's owner bits mark,
// built once per evaluation after every view is rasterized: kernelCountOwners (one block per
// view) popcounts the view's words, kernelScanViewCounts (one block) turns the counts into the
// views' offsets into one packed list (numViews+1 entries, the last the total the host sizes
// the list by), and kernelCompactOwners (one block per view) writes the face ids, in face order.
// The per-direction accumulation then runs one thread per OWNER face of its reference view
// instead of one per mesh face exiting on the bit: a view owns a fraction of the faces (a
// quarter to a third on Ignatius at level 1, where the cameras ring one object; less in a scene
// of hundreds of views along a path), and the exits left the warps that did reach the pixel
// loop 4-5 active lanes of 32 (Nsight Compute, Ignatius level 1, both scales: 4.4 threads per
// warp-instruction, 13 of 20 warp cycles stalled on the scattered loads, compute 27 % and
// memory 37 % of peak, the kernel 55 % of the GPU time); with the lists, 9.4/7.7 lanes and 2.3x
// fewer instructions issued.
// The list order does not touch the result -- a face still reduces its own pixels in raster
// order into its own slot, and the fold order across directions is the launch order -- and
// face order is kept because consecutive faces are neighbours on the mesh, so a warp's 32
// boxes land near one another in the image.

// exclusive prefix sum of one value per thread over the block (blockDim.x a multiple of 32, at
// most 1024): returns the thread's exclusive prefix and leaves the block total in `total`;
// warpSums is 32 words of shared memory, free for reuse when this returns
__device__ inline uint32_t blockScanExclusive(uint32_t v, uint32_t& total, uint32_t* warpSums)
{
	const int lane = threadIdx.x & 31, warp = threadIdx.x >> 5, numWarps = blockDim.x >> 5;
	uint32_t incl = v;
	#pragma unroll
	for (int d = 1; d < 32; d <<= 1) {
		const uint32_t t = __shfl_up_sync(0xffffffffu, incl, d);
		if (lane >= d) incl += t;
	}
	if (lane == 31) warpSums[warp] = incl;
	__syncthreads();
	if (warp == 0) {
		uint32_t s = lane < numWarps ? warpSums[lane] : 0u;
		#pragma unroll
		for (int d = 1; d < 32; d <<= 1) {
			const uint32_t t = __shfl_up_sync(0xffffffffu, s, d);
			if (lane >= d) s += t;
		}
		if (lane < numWarps) warpSums[lane] = s;
	}
	__syncthreads();
	const uint32_t prefix = (warp ? warpSums[warp-1] : 0u) + incl - v;
	total = warpSums[numWarps-1];
	__syncthreads();
	return prefix;
}

__global__ void kernelCountOwners(
	const uint32_t* __restrict__ ownerBits, // numWords per view, view-major
	uint32_t* __restrict__ counts, // 1 per view
	uint32_t numWords)
{
	__shared__ uint32_t warpSums[32];
	const uint32_t* words = ownerBits + (size_t)blockIdx.x * numWords;
	uint32_t n = 0;
	for (uint32_t w = threadIdx.x; w < numWords; w += blockDim.x)
		n += __popc(words[w]);
	uint32_t total;
	blockScanExclusive(n, total, warpSums);
	if (threadIdx.x == 0)
		counts[blockIdx.x] = total;
}

__global__ void kernelScanViewCounts(
	const uint32_t* __restrict__ counts,
	uint32_t* __restrict__ offsets, // numViews+1: the exclusive prefix sum of counts, then the total
	uint32_t numViews)
{
	__shared__ uint32_t warpSums[32];
	uint32_t running = 0;
	for (uint32_t base = 0; base < numViews; base += blockDim.x) {
		const uint32_t i = base + threadIdx.x;
		uint32_t total;
		const uint32_t prefix = blockScanExclusive(i < numViews ? counts[i] : 0u, total, warpSums);
		if (i < numViews)
			offsets[i] = running + prefix;
		running += total;
	}
	if (threadIdx.x == 0)
		offsets[numViews] = running;
}

__global__ void kernelCompactOwners(
	const uint32_t* __restrict__ ownerBits,
	const uint32_t* __restrict__ offsets,
	uint32_t* __restrict__ ownerList, // offsets[numViews] entries: every view's owner faces, in face order
	uint32_t numWords)
{
	__shared__ uint32_t warpSums[32];
	const uint32_t* words = ownerBits + (size_t)blockIdx.x * numWords;
	uint32_t* list = ownerList + offsets[blockIdx.x];
	uint32_t running = 0;
	for (uint32_t base = 0; base < numWords; base += blockDim.x) {
		const uint32_t w = base + threadIdx.x;
		uint32_t bits = w < numWords ? words[w] : 0u;
		uint32_t total;
		uint32_t pos = running + blockScanExclusive(__popc(bits), total, warpSums);
		while (bits) {
			list[pos++] = w*32 + (uint32_t)(__ffs(bits) - 1);
			bits &= bits - 1;
		}
		running += total;
	}
	// what kernelCountOwners counted for this view
	ASSERT(running == offsets[blockIdx.x+1] - offsets[blockIdx.x]);
}


// 4. SortOwnersTile — one block per view, right after the compaction: the view's owner list
// reordered by the image tile (2^tileShiftX x 2^tileShiftY pixels, row-major) the face's
// clipped box starts in, so that the 32 lanes of a warp of the accumulation walk boxes a tile
// or two apart at most. In face order the lanes' boxes were scattered over the image and every
// load fetched its own lines (Nsight Compute: 4 of 32 bytes of every sector used, L1 hit 52-54 %,
// 18-21 of 25-28 warp cycles on the long scoreboard even with a chunk's loads in flight). The
// projection is the accumulation's own (projectFace + faceBBox). A counting sort in shared
// memory: the bucket histogram, its exclusive scan, and a scatter whose order WITHIN a bucket is
// the atomics' -- the result does not depend on the list order (see kernelCompactOwners).
__global__ void kernelSortOwnersTile(
	const Point3* __restrict__ vertices,
	const Point3u* __restrict__ faces,
	const Camera* __restrict__ cameras, // one per view
	const uint32_t* __restrict__ offsets,
	const uint32_t* __restrict__ listIn,
	uint32_t* __restrict__ keys, // scratch, one per list entry
	uint32_t* __restrict__ listOut,
	int tileShiftX, int tileShiftY)
{
	extern __shared__ uint32_t hist[]; // the view's tile count + 32 words for the scan
	const Camera camera = cameras[blockIdx.x];
	const uint32_t off = offsets[blockIdx.x], n = offsets[blockIdx.x+1] - off;
	const int tilesX = (camera.size.x() + (1 << tileShiftX) - 1) >> tileShiftX;
	const int numBuckets = tilesX * ((camera.size.y() + (1 << tileShiftY) - 1) >> tileShiftY);
	uint32_t* warpSums = hist + numBuckets;
	for (int b = threadIdx.x; b < numBuckets; b += blockDim.x)
		hist[b] = 0;
	__syncthreads();
	for (uint32_t i = threadIdx.x; i < n; i += blockDim.x) {
		const uint32_t idxFace = listIn[off + i];
		ProjectedFace pf;
		int ixMin(0), ixMax(-1), iyMin(0), iyMax(-1);
		const bool seen(projectFace(vertices, faces[idxFace], camera, pf) && faceBBox(pf, camera, ixMin, ixMax, iyMin, iyMax));
		ASSERT(seen); (void)seen; // it owns a pixel of this view
		const uint32_t key = (uint32_t)((iyMin >> tileShiftY) * tilesX + (ixMin >> tileShiftX));
		keys[off + i] = key;
		atomicAdd(&hist[key], 1u);
	}
	__syncthreads();
	// exclusive scan: hist[b] becomes the bucket's first position
	uint32_t running = 0;
	for (int base = 0; base < numBuckets; base += blockDim.x) {
		const int b = base + threadIdx.x;
		uint32_t total;
		const uint32_t prefix = blockScanExclusive(b < numBuckets ? hist[b] : 0u, total, warpSums);
		if (b < numBuckets)
			hist[b] = running + prefix;
		running += total;
	}
	__syncthreads();
	for (uint32_t i = threadIdx.x; i < n; i += blockDim.x) {
		const uint32_t pos = atomicAdd(&hist[keys[off + i]], 1u);
		listOut[off + pos] = listIn[off + i];
	}
}


// 5. ImageMeshWarp — 2D, texture + 2 surfaces
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
				// nearest tap read below must be inside the Refine::Border margin the per-pixel window
				// statistics need around it; the bound is one pixel tighter than the tap itself needs
				// so that any accepted xB/yB rounds into the margin. The test is in FLOAT, before the
				// int conversion: pz can be positive but arbitrarily small at a grazing projection,
				// making xB/yB huge, and __float2int_rd saturates to INT_MAX, which an int test would
				// overflow past; this form rejects huge values and NaN instead of wrapping
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

// the per-pixel half of the photometric gradient: g, the scalar the covering face's three
// corners share before their barycentrics (the CPU's sg). A PURE FUNCTION OF THE PIXEL: nothing
// it returns depends on which thread computes it or in what order, which is what lets the
// face-parallel accumulation be atomic-free and bit-reproducible. The caller has already
// established that this pixel is inside the valid border, masked in, and covered by the face
// whose normal it passes in; dz is the pixel's ZNCC derivative. Returns false if the pixel
// contributes nothing (the surface is seen at a grazing angle).
__device__ inline bool computePhotoPixel(
	int x, int y, float depth, const Point3& normal, float dz,
	const Camera& camA,
	const Camera& camB,
	cudaTextureObject_t texImageB, // sampled directly in bilinear-derivative mode (nImageGradient == 3)
	cudaTextureObject_t texGradXB, // precomputed stencil (ComputeRefineImageGradient); unused in that mode
	cudaTextureObject_t texGradYB,
	bool bBilinearGrad,
	float regScale,
	float& g)
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
	const Point3 grad = dz * gradDir;

	// Project gradient along view direction, scale by 1/dot(viewDir, normal)
	const float projMag = grad.dot(viewDir) / viewDotNormal;

	g = regScale * projMag;
	return true;
}


// 6. ComputeWindowStats — 2D, one thread per pixel: the six MASKED window sums, the two
// rejection gates, ZNCC, its derivative, this pair-direction's reliability sums and the pixel's
// photometric term, all in one pass over the 7x7 window. Replaces the former five kernels
// (mean/var/cov/zncc/dzncc) and the six full-image buffers they exchanged. Only successfully
// warped pixels enter the sums, so a window straddling an occlusion boundary is described by the
// pixels that actually matched instead of by image A's own values (the old unmasked statistics
// drove ZNCC towards 1 exactly there). Writes maskOut rather than editing mask in place: the
// window loop of a neighbouring thread is still reading mask. The photometric term used to be
// computed by the face-parallel accumulation kernel, one thread walking every pixel of its face:
// a warp there waits for its largest face, while here every pixel is one thread of equal work.
__global__ void kernelComputeWindowStats(
	const uint8_t* __restrict__ mask,
	uint8_t* __restrict__ maskOut, // the pixels that contribute a photometric term
	float* __restrict__ pixelGrad, // their term g (computePhotoPixel), 0 elsewhere
	float* __restrict__ blockSums, // 2 floats per block: this block's reliability partials
	cudaSurfaceObject_t surfImageA,
	cudaSurfaceObject_t surfImageProj,
	const Point3* __restrict__ normals,
	const uint32_t* __restrict__ faceMap,
	const float* __restrict__ depthMap,
	Camera camA,
	Camera camB,
	cudaTextureObject_t texImageB,
	cudaTextureObject_t texGradXB,
	cudaTextureObject_t texGradYB,
	bool bBilinearGrad,
	float regScale,
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
	bool any = false; // this thread staged a masked sample
	for (int i = threadIdx.y * Block + threadIdx.x; i < Tile*Tile; i += Block*Block) {
		const int gx = x0 + i % Tile, gy = y0 + i / Tile;
		float a(0.f), b(0.f), w(0.f);
		// invalid samples are staged as zeros so the accumulation below needs no branch: they
		// contribute nothing to any of the six sums, which is exactly what masking them means
		if (gx >= 0 && gy >= 0 && gx < width && gy < height && mask[gy * width + gx] == 1) {
			a = readSurfFloat(surfImageA, gx, gy);
			b = readSurfFloat(surfImageProj, gx, gy);
			w = 1.f;
			any = true;
		}
		sA[i] = a; sB[i] = b; sW[i] = w;
	}
	// a tile without a single masked sample -- the warp reaches none of its pixels, the common
	// case in a scene of hundreds of views each seeing a small part of the surface -- contributes
	// nothing: its outputs are the zeros the loops below would produce, written here instead;
	// the vote is a barrier of its own, so the block leaves together
	if (!__syncthreads_or(any)) {
		if (x < width && y < height) {
			pixelGrad[y * width + x] = 0.f;
			maskOut[y * width + x] = 0;
		}
		if (threadIdx.x == 0 && threadIdx.y == 0) {
			const int idxBlock = blockIdx.y * gridDim.x + blockIdx.x;
			blockSums[idxBlock*2 + 0] = 0.f;
			blockSums[idxBlock*2 + 1] = 0.f;
		}
		return;
	}
	__syncthreads();

	// per-thread contribution to this block's reliability sums; stays 0 for every pixel outside
	// the image (the grid is rounded up to the block size), outside the valid border, unmasked or
	// gated -- exactly the pixels the CPU's ScoreMesh S skips; no early "return" here so every
	// thread in the block reaches the reduction below
	float contribR(0.f), contribRZ(0.f);
	if (x < width && y < height) {
		const int pixIdx = y * width + x;
		float g(0.f);
		uint8_t contributes(0);
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
				float zn, dz, cf;
				Refine::ZnccAndDerivative(s, n, sA[centre], sB[centre], zn, dz, cf);
				contribR = cf;
				contribRZ = cf * (1.f - zn);
				// mask==1 means the rasterizer covered this pixel (the warp seeds nothing where
				// depthMap is 0), so faceMap holds the covering face; a grazing view of it
				// contributes to S (above, like the CPU) but no photometric term
				const float depth = depthMap[pixIdx];
				ASSERT(depth > 0.f);
				if (computePhotoPixel(x, y, depth, normals[faceMap[pixIdx]], dz,
						camA, camB, texImageB, texGradXB, texGradYB, bBilinearGrad, regScale, g))
					contributes = 1;
			}
		}
		pixelGrad[pixIdx] = g;
		maskOut[pixIdx] = contributes;
	}

	// shared-memory block reduction (a fixed tree over a fixed thread mapping, so its result does
	// not depend on the schedule); sized for the 16x16 block LaunchComputeWindowStats always
	// launches. The block's two partials go to its OWN slot rather than into a global atomicAdd,
	// and kernelReduceBlockSums below folds the slots in order, so S is reproducible too.
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


// 7. ReduceBlockSums — a single block, once per ScoreMesh() after every pair-direction: adds
// the per-block partials of all of them (each direction's blocks occupy their own slots, in
// launch order) into the two accumulators S is computed from, in a fixed order (each thread walks
// a strided, fixed subset of the slots, then a fixed shared-memory tree), so S is the same number
// on every run.
__global__ void kernelReduceBlockSums(
	const float* __restrict__ blockSums,
	uint32_t numSlots,
	float* __restrict__ sumR,
	float* __restrict__ sumRZ)
{
	constexpr int Threads = 1024;
	__shared__ float sR[Threads];
	__shared__ float sRZ[Threads];
	const int tid = threadIdx.x;
	float r(0.f), rz(0.f);
	for (uint32_t i = (uint32_t)tid; i < numSlots; i += Threads) {
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
		sumR[0] = sR[0];
		sumRZ[0] = sRZ[0];
	}
}


// see kernelAccumulateFacePhoto's tail: stamp vertex v with this direction and count it once
__device__ inline void countDirection(uint32_t* __restrict__ vertexStamp, float* __restrict__ photoCount, uint32_t v, uint32_t direction)
{
	if (atomicExch(&vertexStamp[v], direction) != direction)
		photoCount[v] += 1.f;
}

// 8. AccumulateFacePhoto — 1D, one thread per face the reference view OWNS (its slice of the
// list kernelCompactOwners built and kernelSortOwnersTile ordered); the per-pair-direction half
// of the atomic-free photometric accumulation. Each thread reduces ITS OWN face's pixels into
// registers and folds them into its private slots, so there is not a single atomic and not a
// single float sum whose order depends on the schedule -- float addition is not associative,
// and an atomicAdd scatter gives a different per-vertex gradient on every run. The slots
// accumulate across the pair-directions of one ScoreMesh() (one writer per face, in launch
// order, so that too is a fixed sequence), which lets the per-vertex gather run once per
// ScoreMesh() instead of once per direction.
//
// A face this view does not see -- behind the camera, off the image, back-facing or occluded --
// is not in the list (kernelProjectMesh pass 2 left its owner bit clear). The rest project their
// face and walk its clipped bounding box exactly as kernelProjectMesh did, keeping the pixels
// whose faceMap entry is this face (faceMap only ever holds ids the rasterizer wrote); a kept
// pixel's barycentrics and depth are recomputed from the same projection, the same arithmetic
// the rasterizer keyed the pixel with, so they are the rasterizer's bit for bit, in a fixed
// row-then-column order. The per-pixel term itself comes
// from kernelComputeWindowStats, one balanced thread per pixel; here it is only weighted by the
// barycentrics. (Eight lanes sharing a face, each taking every eighth pixel of the box with a
// fixed shuffle tree over their partials, was measured at 49/88 us per launch against 26/64 us
// for this form on Ignatius at level 1, scales 0/1: on the small boxes most lanes idle, and the
// extra threads and shuffles cost more than the divergence they remove.)
__device__ inline void accumulateFacePhoto(
	uint32_t tid, // the face, one of the reference view's owners
	const Point3* __restrict__ vertices,
	const Point3u* __restrict__ faces,
	const uint32_t* __restrict__ faceMap,
	const float* __restrict__ pixelGrad,
	const uint8_t* __restrict__ mask,
	float* __restrict__ faceAcc, // 3 per face: Sum over the face's pixels of g_p*b_c, one per corner
	float* __restrict__ facePixels, // 1 per face: how many pixels contributed (0 = the face contributed nothing)
	float* __restrict__ faceFoot, // 1 per face: min footprint over the face's pixels; only read where facePixels > 0
	uint32_t* __restrict__ vertexStamp, // 1 per vertex: the last direction that reached it (~0u at the start of an evaluation)
	float* __restrict__ photoCount, // 1 per vertex: how many directions reached it
	uint32_t direction, // this pair-direction's index within the evaluation
	const Camera& camA)
{
	// it owns a pixel, so the rasterizer's projection of it succeeded
	const Point3u face = faces[tid];
	ProjectedFace pf;
	int ixMin(0), ixMax(-1), iyMin(0), iyMax(-1);
	const bool seen(projectFace(vertices, face, camA, pf) && faceBBox(pf, camA, ixMin, ixMax, iyMin, iyMax));
	ASSERT(seen); (void)seen;
	float sum0 = 0.f, sum1 = 0.f, sum2 = 0.f;
	float pixels = 0.f, foot = FLT_MAX;
	const int width = camA.size.x();
	// the box row by row, in chunks of RowChunk pixels whose face ids, mask bytes and terms are
	// loaded together before any is tested, so that a chunk's loads are in flight at once: one
	// load per pixel between branches paid a full memory round trip per pixel (Nsight Compute:
	// 21 of 28 warp cycles per instruction on the long scoreboard, the lanes' boxes being
	// scattered over the image). The pixels are still folded in raster order, so the sums are
	// the same sequence of float additions
	constexpr int RowChunk = 4;
	for (int iy = iyMin; iy <= iyMax; ++iy) {
		for (int ix0 = ixMin; ix0 <= ixMax; ix0 += RowChunk) {
			const int base = iy * width + ix0;
			const int n = min(RowChunk, ixMax - ix0 + 1);
			uint32_t f[RowChunk]; uint8_t m[RowChunk]; float g[RowChunk];
			#pragma unroll
			for (int k = 0; k < RowChunk; ++k) {
				const bool in = k < n;
				f[k] = in ? faceMap[base + k] : (uint32_t)-1;
				m[k] = in ? mask[base + k] : (uint8_t)0;
				g[k] = in ? pixelGrad[base + k] : 0.f;
			}
			#pragma unroll
			for (int k = 0; k < RowChunk; ++k) {
				if (f[k] != tid || m[k] != 1)
					continue;
				// this pixel's winning rasterizer key was this face's, computed by pixelBary
				// from this very projection, so the same call reproduces its
				// perspective-correct barycentrics and depth
				float nb0, nb1, nb2, depth;
				const bool inside(pixelBary(ix0 + k, iy, pf, nb0, nb1, nb2, depth));
				ASSERT(inside); (void)inside;
				sum0 += g[k] * nb0;
				sum1 += g[k] * nb1;
				sum2 += g[k] * nb2;
				// per-vertex footprint at camera A, scene units per pixel
				// (Camera::GetFootprintWorld = depth/focalLength): min over every contributing
				// pixel of every pair-direction, matching the CPU's min-of-mins
				// (MeshRefine::ComputePhotometricGradient/ThProcessPair); min is exact and
				// associative, so this half of it is reproducible for free
				foot = fminf(foot, depth / camA.model.f.x());
				pixels += 1.f;
			}
		}
	}
	if (pixels == 0.f)
		return; // nothing to fold in; the slots keep what the earlier directions left
	faceAcc[tid*3 + 0] += sum0;
	faceAcc[tid*3 + 1] += sum1;
	faceAcc[tid*3 + 2] += sum2;
	facePixels[tid] += pixels;
	faceFoot[tid] = fminf(faceFoot[tid], foot);
	// the direction counts once per vertex it reaches (the CPU's `photoGradNorm[idxVert] += 1.f`
	// per pair-direction): of the faces of the vertex contributing to this direction, exactly
	// one thread gets the previous stamp back from the exchange and counts, so the count grows
	// by an integer 1 exactly once per direction whatever the schedule -- reproducible, and no
	// per-direction kernel over the vertices (one launch in four, host-bound at the coarse
	// scales) to clear the marks and count them
	countDirection(vertexStamp, photoCount, face.x(), direction);
	countDirection(vertexStamp, photoCount, face.y(), direction);
	countDirection(vertexStamp, photoCount, face.z(), direction);
}
__global__ void kernelAccumulateFacePhoto(
	const Point3* __restrict__ vertices,
	const Point3u* __restrict__ faces,
	const uint32_t* __restrict__ ownerList, // every view's owner faces (kernelSortOwnersTile)
	const uint32_t* __restrict__ ownerOffsets, // the views' slices of it, numViews+1 entries
	uint32_t idxView, // the reference view
	const uint32_t* __restrict__ faceMap,
	const float* __restrict__ pixelGrad,
	const uint8_t* __restrict__ mask,
	float* __restrict__ faceAcc,
	float* __restrict__ facePixels,
	float* __restrict__ faceFoot,
	uint32_t* __restrict__ vertexStamp,
	float* __restrict__ photoCount,
	uint32_t direction,
	Camera camA)
{
	// the view's slice is read here, not passed: the launch is recorded once per scale into a
	// graph (MeshRefineCUDA::ScoreMesh) while the slices move with the mesh, so the grid is the
	// slice's size at the recording with a margin, and the stride loop covers whatever it has
	// grown to since
	const uint32_t off = ownerOffsets[idxView], numOwners = ownerOffsets[idxView+1] - off;
	for (uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x; idx < numOwners; idx += gridDim.x * blockDim.x)
		accumulateFacePhoto(ownerList[off + idx], vertices, faces, faceMap, pixelGrad, mask,
			faceAcc, facePixels, faceFoot, vertexStamp, photoCount, direction, camA);
}


// 9. GatherVertexPhoto — 1D, one thread per vertex, once per ScoreMesh() after every
// pair-direction. It walks the vertex's incident faces in the fixed order Mesh::ListIncidentFaces
// produced (uploaded per scale, see MeshRefineCUDA::ListVertexFacesPost) and folds in each
// face's slots, so a vertex's sum is a fixed sequence of float additions. The footprint sentinel
// is resolved here: 0 exactly where no face contributed, which is exactly where no direction
// counted the vertex (contract: footprint[v] > 0 iff photoCount[v] > 0, matches CPU ScoreMesh).
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
	float* __restrict__ footprint,
	uint32_t numVertices)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numVertices) return;

	const uint32_t ptr = vertFacePointers[tid];
	const uint32_t numIncident = vertFaceSizes[tid];
	Point3 grad = Point3::Zero();
	float foot = FLT_MAX;
	bool seen = false;
	for (uint32_t i = 0; i < numIncident; ++i) {
		const uint32_t idxFace = vertFaces[ptr + i];
		const Point3u& face = faces[idxFace];
		const uint32_t fv[3] = { face.x(), face.y(), face.z() };
		// the adjacency is the transpose of the face list: a face this vertex is incident to must
		// name it back, or the two uploads describe different meshes
		ASSERT(fv[0] == (uint32_t)tid || fv[1] == (uint32_t)tid || fv[2] == (uint32_t)tid);
		if (facePixels[idxFace] == 0.f)
			continue; // nothing was accumulated, so every other slot of this face is a zero
		const Point3& normal = normals[idxFace];
		for (int c = 0; c < 3; ++c)
			if (fv[c] == (uint32_t)tid)
				grad += normal * faceAcc[idxFace*3 + c];
		foot = fminf(foot, faceFoot[idxFace]);
		seen = true;
	}
	photoGrad[tid] = grad;
	footprint[tid] = seen ? foot : 0.f;
}


// 10. ComputeSmoothnessGradient — 1D
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

	// (1/N)*sum(neighbors - vertex), the CPU's sign convention (ComputeSmoothnessGradient1/2) and
	// its accumulation of differences rather than of coordinates, which would lose precision to
	// the subtraction of the centre afterwards; the stepper mixes it with the bi-laplacian by rho
	const float invN = 1.f / (float)numNeighbors;
	const Point3 center = vertices[tid];
	Point3 result = Point3::Zero();
	float totalWeight = 1.f;
	for (uint32_t i = 0; i < numNeighbors; ++i) {
		const uint32_t ni = vertVertices[ptr + i];
		result += vertices[ni] - center;
		if (mode != 0) {
			// Valence-weighted: accumulate 1/(Ni*N) where Ni = TRUE valence of neighbor
			// (boundary neighbours are included, exactly as CPU's vertexVertices[ni].GetSize());
			// adjacency is symmetric, so a neighbour lists this vertex back and Ni >= 1
			ASSERT(vertSizes[ni] > 0);
			totalWeight += invN / (float)vertSizes[ni];
		}
	}
	result *= invN;
	if (mode != 0)
		result /= totalWeight;
	smoothGrad[tid] = result;
}


// 11. ComputeFaceNormal — 1D, 1 thread per face
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

// 12. FaceHistogram -- 1D over the pixels of one view, after the rasterization: the rasterized
// area of every face in it, in pixels, exactly the pixel count the host's ListFaceAreas took
// from the downloaded face map; integer atomics, so the counts are exact whatever the schedule
__global__ void kernelFaceHistogram(
	const uint32_t* __restrict__ faceMap,
	uint32_t* __restrict__ hist,
	uint32_t numPixels)
{
	const uint32_t p = blockIdx.x * blockDim.x + threadIdx.x;
	if (p >= numPixels) return;
	const uint32_t f = faceMap[p];
	if (f != (uint32_t)-1)
		atomicAdd(&hist[f], 1u);
}


// 13. ReduceFaceAreasPair -- 1D, one thread per face, once per pair: the smaller of the face's
// two rasterized areas in the pair (a pair resolves a face only as well as its worse view) folded
// into the largest over the pairs, exactly ReduceFaceAreasOverPairs (SceneRefineCommon.cpp), the
// truncation to 16 bits included (the host counts in uint16_t)
__global__ void kernelReduceFaceAreasPair(
	const uint32_t* __restrict__ histA,
	const uint32_t* __restrict__ histB,
	uint16_t* __restrict__ maxAreas,
	uint32_t numFaces)
{
	const uint32_t f = blockIdx.x * blockDim.x + threadIdx.x;
	if (f >= numFaces) return;
	const uint16_t a = (uint16_t)histA[f], b = (uint16_t)histB[f];
	const uint16_t pairArea = a < b ? a : b;
	if (maxAreas[f] < pairArea)
		maxAreas[f] = pairArea;
}


void LaunchProjectMesh(
	const Point3* vertices, const Point3u* faces,
	unsigned long long* projKey, float* depthMap, uint32_t* faceMap, uint32_t* ownerBits,
	const Camera& camera, uint32_t numFaces, bool resolve)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numFaces + blockSize - 1) / blockSize;
	if (resolve)
		kernelProjectMesh<true><<<numBlocks, blockSize>>>(vertices, faces, projKey, depthMap, faceMap, ownerBits, camera, numFaces);
	else
		kernelProjectMesh<false><<<numBlocks, blockSize>>>(vertices, faces, projKey, depthMap, faceMap, NULL, camera, numFaces);
}

#ifdef _DEBUG
void LaunchCheckProjection(
	const unsigned long long* projKey,
	const float* depthMap, const uint32_t* faceMap, int width, int height)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelCheckProjection<<<grid, block>>>(projKey, depthMap, faceMap, width, height);
}
#endif

void LaunchImageMeshWarp(
	const float* depthMapA, const float* depthMapB,
	const uint8_t* keepA, const uint8_t* keepB, uint8_t* mask,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB,
	cudaSurfaceObject_t surfImageProj,
	cudaStream_t stream)
{
	const dim3 block(16, 16);
	const dim3 grid((camA.size.x() + block.x - 1) / block.x, (camA.size.y() + block.y - 1) / block.y);
	kernelImageMeshWarp<<<grid, block, 0, stream>>>(depthMapA, depthMapB, keepA, keepB, mask, camA, camB, texImageB, surfImageProj);
}

uint32_t LaunchComputeWindowStats(
	const uint8_t* mask, uint8_t* maskOut, float* pixelGrad, float* blockSums,
	cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj,
	const Point3* normals, const uint32_t* faceMap, const float* depthMap,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB, cudaTextureObject_t texGradXB, cudaTextureObject_t texGradYB,
	bool bBilinearGrad, float regScale, float gateMeanDiff, float gateVarRatio, int width, int height,
	cudaStream_t stream)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputeWindowStats<<<grid, block, 0, stream>>>(mask, maskOut, pixelGrad, blockSums,
		surfImageA, surfImageProj, normals, faceMap, depthMap, camA, camB,
		texImageB, texGradXB, texGradYB, bBilinearGrad, regScale, gateMeanDiff, gateVarRatio, width, height);
	return grid.x*grid.y;
}

void LaunchReduceBlockSums(const float* blockSums, uint32_t numSlots, float* sumR, float* sumRZ, cudaStream_t stream)
{
	kernelReduceBlockSums<<<1, 1024, 0, stream>>>(blockSums, numSlots, sumR, sumRZ);
}

void LaunchCountOwners(const uint32_t* ownerBits, uint32_t* counts, uint32_t numWords, uint32_t numViews)
{
	kernelCountOwners<<<numViews, 1024>>>(ownerBits, counts, numWords);
}

void LaunchScanViewCounts(const uint32_t* counts, uint32_t* offsets, uint32_t numViews)
{
	kernelScanViewCounts<<<1, 1024>>>(counts, offsets, numViews);
}

void LaunchCompactOwners(const uint32_t* ownerBits, const uint32_t* offsets, uint32_t* ownerList, uint32_t numWords, uint32_t numViews)
{
	kernelCompactOwners<<<numViews, 1024>>>(ownerBits, offsets, ownerList, numWords);
}

void LaunchSortOwnersTile(
	const Point3* vertices, const Point3u* faces, const Camera* cameras,
	const uint32_t* offsets, const uint32_t* listIn, uint32_t* keys, uint32_t* listOut,
	int tileShiftX, int tileShiftY, uint32_t maxBuckets, uint32_t numViews)
{
	ASSERT(maxBuckets <= 8192); // static limit of the dynamic shared memory
	kernelSortOwnersTile<<<numViews, 1024, sizeof(uint32_t)*(maxBuckets + 32)>>>(
		vertices, faces, cameras, offsets, listIn, keys, listOut, tileShiftX, tileShiftY);
}

void LaunchAccumulateFacePhoto(
	const Point3* vertices, const Point3u* faces,
	const uint32_t* ownerList, const uint32_t* ownerOffsets, uint32_t idxView,
	const uint32_t* faceMap, const float* pixelGrad, const uint8_t* mask,
	float* faceAcc, float* facePixels, float* faceFoot,
	uint32_t* vertexStamp, float* photoCount, uint32_t direction,
	const Camera& camA, uint32_t numBlocks, cudaStream_t stream)
{
	// small blocks: a launch is a fraction of the mesh, and the box walks are uneven
	kernelAccumulateFacePhoto<<<numBlocks, AccumulateBlockSize, 0, stream>>>(
		vertices, faces, ownerList, ownerOffsets, idxView, faceMap, pixelGrad, mask,
		faceAcc, facePixels, faceFoot, vertexStamp, photoCount, direction, camA);
}

void LaunchFaceHistogram(const uint32_t* faceMap, uint32_t* hist, uint32_t numPixels)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numPixels + blockSize - 1) / blockSize;
	kernelFaceHistogram<<<numBlocks, blockSize>>>(faceMap, hist, numPixels);
}

void LaunchReduceFaceAreasPair(const uint32_t* histA, const uint32_t* histB, uint16_t* maxAreas, uint32_t numFaces)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numFaces + blockSize - 1) / blockSize;
	kernelReduceFaceAreasPair<<<numBlocks, blockSize>>>(histA, histB, maxAreas, numFaces);
}

void LaunchGatherVertexPhoto(
	const Point3u* faces, const Point3* normals,
	const uint32_t* vertFaces, const uint32_t* vertFaceSizes, const uint32_t* vertFacePointers,
	const float* faceAcc, const float* facePixels, const float* faceFoot,
	Point3* photoGrad, float* footprint, uint32_t numVertices, cudaStream_t stream)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelGatherVertexPhoto<<<numBlocks, blockSize, 0, stream>>>(
		faces, normals, vertFaces, vertFaceSizes, vertFacePointers,
		faceAcc, facePixels, faceFoot, photoGrad, footprint, numVertices);
}

void LaunchComputeSmoothnessGradient(
	const Point3* vertices, const uint32_t* vertVertices, const uint32_t* vertSizes, const uint32_t* vertPointers,
	const uint8_t* vertBoundary, Point3* smoothGrad, uint32_t numVertices, uint8_t mode)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelComputeSmoothnessGradient<<<numBlocks, blockSize>>>(vertices, vertVertices, vertSizes, vertPointers, vertBoundary, smoothGrad, numVertices, mode);
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
