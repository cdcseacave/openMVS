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

// Atomic add a Point3 to a device array
__device__ inline void atomicAddPoint3(Point3* addr, const Point3& val) {
	atomicAdd(&addr->x(), val.x());
	atomicAdd(&addr->y(), val.y());
	atomicAdd(&addr->z(), val.z());
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

	// Bounding box with ±0.5 padding, clamped to the shared border margin (Refine::Border, the
	// same margin the per-pixel window-statistics kernels require -- was a hardcoded 5px border);
	// the accepted pixel range is [Border, size-Border) exactly as the CPU's per-pixel test in
	// MeshRefine::RasterMesh::Raster (SceneRefine.cpp), so the inclusive bbox ends at size-Border-1
	// (an inclusive clamp at size-Border rasterised one extra row/column the CPU rejects, and the
	// warped values it put there leaked into every 7x7 window statistic within HalfSize of it)
	const int border = Refine::Border;
	const int ixMin = max(__float2int_ru(fminf(fminf(pf.p0.x(), pf.p1.x()), pf.p2.x()) - 0.5f), border);
	const int ixMax = min(__float2int_rd(fmaxf(fmaxf(pf.p0.x(), pf.p1.x()), pf.p2.x()) + 0.5f), camera.size.x() - border - 1);
	const int iyMin = max(__float2int_ru(fminf(fminf(pf.p0.y(), pf.p1.y()), pf.p2.y()) - 0.5f), border);
	const int iyMax = min(__float2int_rd(fmaxf(fmaxf(pf.p0.y(), pf.p1.y()), pf.p2.y()) + 0.5f), camera.size.y() - border - 1);
	if (ixMin > ixMax || iyMin > iyMax) return;

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
	uint8_t* __restrict__ mask,
	Camera camA,
	Camera camB,
	cudaTextureObject_t texImageB,
	cudaSurfaceObject_t surfImageA,
	cudaSurfaceObject_t surfImageProj)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= camA.size.x() || y >= camA.size.y()) return;

	const int pixIdx = y * camA.size.x() + x;
	// default to A's own pixel: the CPU copies imageA into imageAB before warping (ThProcessPair),
	// so the 7x7 window statistics next to a mask edge see A's values where B could not be warped
	float convergePix = 0.f;
	uint8_t convergeMask = 0;

	surf2Dread(&convergePix, surfImageA, x * (int)sizeof(float), y);

	const float depthA = depthMapA[pixIdx];
	if (depthA > 0.f) {
		const Point3 X_world = camA.TransformPointI2W(Point2((float)x, (float)y), depthA);
		const Point3 Xc_B = camB.pose.TransformPointW2C(X_world);
		const float pz = Xc_B.z();

		if (pz > 0.f) {
			const Point2 projB = camB.model.TransformPointC2I(Xc_B);
			const float xB = projB.x(), yB = projB.y();
			// B-side border rule shared with the CPU (MeshRefine::IsDepthSimilar): the whole 2x2
			// read below must be inside the Refine::Border margin, tested as a single block (the
			// old CPU per-corner test independently continue'd past a failing corner instead of
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

				// the CPU's IsDepthSimilar (SceneRefine.cpp): the point is kept iff one of the 4
				// depths around its projection is valid and not more than Refine::DepthConstBias
				// in FRONT of it -- an occlusion test; a surface behind the point is accepted.
				// This used to be a symmetric 1 % relative test (the CPU's dead #ifndef branch),
				// which rejected ~100 grazing pixels per pair on Tiny that the CPU keeps and gave
				// 10 vertices a 5-10x smaller photometric gradient than the CPU's.
				bool consistent = false;
				for (int k=0; k<4 && !consistent; ++k) {
					const float depthB = depthMapB[idxB + (k & 1) + (k >> 1) * widthB];
					if (depthB > 0.f && depthB + Refine::DepthConstBias >= pz)
						consistent = true;
				}

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

	surf2Dwrite(convergePix, surfImageProj, x * (int)sizeof(float), y);
	mask[pixIdx] = convergeMask;
}


// 4. ComputeImageMean — 2D
__global__ void kernelComputeImageMean(
	const uint8_t* __restrict__ mask,
	float* __restrict__ imageMean,
	cudaSurfaceObject_t surfImage,
	int width, int height, int halfSize)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	const int pixIdx = y * width + x;
	if (x < halfSize || y < halfSize || x >= width - halfSize || y >= height - halfSize || mask[pixIdx] != 1) {
		imageMean[pixIdx] = 0.f;
		return;
	}

	const float windowArea = (float)(2 * halfSize + 1) * (float)(2 * halfSize + 1);
	float sum = 0.f;
	for (int dy = -halfSize; dy <= halfSize; ++dy)
		for (int dx = -halfSize; dx <= halfSize; ++dx)
			sum += readSurfFloat(surfImage, x + dx, y + dy);
	imageMean[pixIdx] = sum / windowArea;
}


// 5. ComputeImageVar — 2D
__global__ void kernelComputeImageVar(
	const float* __restrict__ imageMean,
	const uint8_t* __restrict__ mask,
	float* __restrict__ imageVar,
	cudaSurfaceObject_t surfImage,
	int width, int height, int halfSize)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	const int pixIdx = y * width + x;
	if (x < halfSize || y < halfSize || x >= width - halfSize || y >= height - halfSize || mask[pixIdx] != 1) {
		imageVar[pixIdx] = 0.f;
		return;
	}

	const float windowArea = (float)(2 * halfSize + 1) * (float)(2 * halfSize + 1);
	const float mean = imageMean[pixIdx];
	float sum = 0.f;
	for (int dy = -halfSize; dy <= halfSize; ++dy) {
		for (int dx = -halfSize; dx <= halfSize; ++dx) {
			const float diff = readSurfFloat(surfImage, x + dx, y + dy) - mean;
			sum += diff * diff;
		}
	}
	imageVar[pixIdx] = fmaxf(sum / windowArea, 1e-4f);
}


// 6. ComputeImageCov — 2D
__global__ void kernelComputeImageCov(
	const float* __restrict__ imageMeanA,
	const float* __restrict__ imageMeanB,
	const uint8_t* __restrict__ mask,
	float* __restrict__ imageCov,
	cudaSurfaceObject_t surfImageA,
	cudaSurfaceObject_t surfImageProj,
	int width, int height, int halfSize)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	const int pixIdx = y * width + x;
	if (x < halfSize || y < halfSize || x >= width - halfSize || y >= height - halfSize || mask[pixIdx] != 1) {
		imageCov[pixIdx] = 0.f;
		return;
	}

	const float windowArea = (float)(2 * halfSize + 1) * (float)(2 * halfSize + 1);
	const float meanA = imageMeanA[pixIdx], meanB = imageMeanB[pixIdx];
	float sum = 0.f;
	for (int dy = -halfSize; dy <= halfSize; ++dy)
		for (int dx = -halfSize; dx <= halfSize; ++dx)
			sum += (readSurfFloat(surfImageA, x+dx, y+dy) - meanA) * (readSurfFloat(surfImageProj, x+dx, y+dy) - meanB);
	imageCov[pixIdx] = sum / windowArea;
}


// 7. ComputeImageZNCC — 2D
__global__ void kernelComputeImageZNCC(
	const float* __restrict__ imageCov,
	const float* __restrict__ imageVarA,
	const float* __restrict__ imageVarB,
	const uint8_t* __restrict__ mask,
	float* __restrict__ imageZNCC,
	int width, int height, int halfSize)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	const int pixIdx = y * width + x;
	if (x < halfSize || y < halfSize || x >= width - halfSize || y >= height - halfSize || mask[pixIdx] != 1) {
		imageZNCC[pixIdx] = 0.f;
		return;
	}
	imageZNCC[pixIdx] = imageCov[pixIdx] / sqrtf(imageVarA[pixIdx] * imageVarB[pixIdx]);
}


// 8. ComputeImageDZNCC — 2D
__global__ void kernelComputeImageDZNCC(
	const float* __restrict__ meanA,
	const float* __restrict__ meanB,
	const float* __restrict__ varA,
	const float* __restrict__ varB,
	const float* __restrict__ zncc,
	const uint8_t* __restrict__ mask,
	float* __restrict__ dzncc,
	cudaSurfaceObject_t surfImageA,
	cudaSurfaceObject_t surfImageProj,
	int width, int height, int halfSize)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	const int pixIdx = y * width + x;
	if (x < halfSize || y < halfSize || x >= width - halfSize || y >= height - halfSize || mask[pixIdx] != 1) {
		dzncc[pixIdx] = 0.f;
		return;
	}

	// pointwise form, matching CPU ComputeLocalZNCC (SceneRefine.cpp); the windowed/box-averaged
	// variant the CPU used to carry behind an "#else" was deleted, so both backends keep one formula
	const float pixA = readSurfFloat(surfImageA, x, y);
	const float pixB = readSurfFloat(surfImageProj, x, y);
	const float invSqrtVarAVarB = 1.f / sqrtf(varA[pixIdx] * varB[pixIdx]);
	const float ZNCCinvVarB = zncc[pixIdx] / varB[pixIdx];
	const float dZ = (pixA - meanA[pixIdx]) * invSqrtVarAVarB - (pixB - meanB[pixIdx]) * ZNCCinvVarB;

	dzncc[pixIdx] = -Refine::ZnccReliability(varA[pixIdx], varB[pixIdx]) * dZ;
}


// 9. ComputePhotometricGradient — 2D, texture + atomicAdd
__global__ void kernelComputePhotometricGradient(
	const Point3u* __restrict__ faces,
	const Point3* __restrict__ normals,
	const float* __restrict__ depthMap,
	const uint32_t* __restrict__ faceMap,
	const uint16_t* __restrict__ baryMap,
	const float* __restrict__ dznccMap,
	const uint8_t* __restrict__ mask,
	Point3* __restrict__ photoGrad,
	float* __restrict__ photoGradPixels,
	float* __restrict__ sgMap, // WP2 parity diagnostic: per-pixel scalar handed to the 3 vertices (CPU's sg); NULL in production
	Camera camA,
	Camera camB,
	cudaTextureObject_t texGradXB,
	cudaTextureObject_t texGradYB,
	float regScale,
	int width, int height)
{
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;
	// exclude the border margin where the window statistics feeding dznccMap are not valid (CPU
	// ComputePhotometricGradient only loops [HalfSize, size-HalfSize)); without this, a border
	// pixel with dzncc==0 still contributed a zero-gradient sample that inflated photoGradPixels
	// (the normalisation denominator) without adding anything to photoGrad -- diluting the average
	if (x < Refine::HalfSize || y < Refine::HalfSize || x >= width - Refine::HalfSize || y >= height - Refine::HalfSize) return;

	const int pixIdx = y * width + x;
	if (mask[pixIdx] != 1) return;

	const float depth = depthMap[pixIdx];
	const uint32_t faceID = faceMap[pixIdx];
	// a masked pixel is by construction one ProjectMesh covered, so it carries a real face
	ASSERT(faceID != (uint32_t)-1);
	const float bary0 = __half2float(*reinterpret_cast<const __half*>(&baryMap[pixIdx * 3 + 0]));
	const float bary1 = __half2float(*reinterpret_cast<const __half*>(&baryMap[pixIdx * 3 + 1]));
	const float bary2 = __half2float(*reinterpret_cast<const __half*>(&baryMap[pixIdx * 3 + 2]));

	const Point3u& face = faces[faceID];
	const Point3& normal = normals[faceID];

	// View direction in world space (normalized)
	const Point3 camRay = camA.model.TransformPointI2C(Point2((float)x, (float)y));
	const Point3 worldDir = camA.pose.R.transpose() * camRay;
	const Point3 viewDir = worldDir.normalized();

	const float viewDotNormal = viewDir.dot(normal);
	if (viewDotNormal > -0.1f) return;

	// Back-project to 3D and forward-project to camera B
	const Point3 X_world = camA.TransformPointI2W(Point2((float)x, (float)y), depth);
	const Point3 Xc_B = camB.pose.TransformPointW2C(X_world);
	const float pz = Xc_B.z();

	// mask==1 means kernelImageMeshWarp already ran this exact back-/forward-projection chain for
	// this pixel and required a positive depth in B, so pz can only be positive here (the CPU
	// states the same contract: ASSERT(depthB > 0) in MeshRefine::ComputePhotometricGradient)
	ASSERT(pz > 0.f);
	const Point2 projB = camB.model.TransformPointC2I(Xc_B);

	// Jacobian d(u,v)/d(X_world): KR = K * R
	const Matrix3 KR = camB.model.K() * camB.pose.R;
	const Point3 p = camB.model.K() * Xc_B; // raw projection before perspective divide
	const float pz2 = pz * pz;

	// du/dX = (KR.row(0)*pz - KR.row(2)*px) / pz², same for dv/dX
	const Point3 dudX = (KR.row(0).transpose() * pz - KR.row(2).transpose() * p.x()) / pz2;
	const Point3 dvdX = (KR.row(1).transpose() * pz - KR.row(2).transpose() * p.y()) / pz2;

	// Image derivatives at the projected point: bilinear samples of the precomputed gradient
	// images (ComputeRefineImageGradient, the CPU's estimator and sampling exactly; forward
	// differences of the image texture used to give a per-pixel magnitude that differed from
	// the CPU by a factor of 0.6-5) (+0.5: tex2D pixel-centre convention, see ImageMeshWarp)
	const float dx = tex2D<float>(texGradXB, projB.x() + 0.5f, projB.y() + 0.5f);
	const float dy = tex2D<float>(texGradYB, projB.x() + 0.5f, projB.y() + 0.5f);

	// 3D gradient = dzncc * J^T * [dx, dy]
	const float dz = dznccMap[pixIdx];
	const Point3 grad = dz * (dx * dudX + dy * dvdX);

	// Project gradient along view direction, scale by 1/dot(viewDir, normal)
	const float projMag = grad.dot(viewDir) / viewDotNormal;
	if (sgMap)
		sgMap[pixIdx] = regScale * projMag;

	// Distribute to 3 vertices weighted by bary coords × regScale × normal
	atomicAddPoint3(&photoGrad[face.x()], (regScale * bary0 * projMag) * normal);
	atomicAddPoint3(&photoGrad[face.y()], (regScale * bary1 * projMag) * normal);
	atomicAddPoint3(&photoGrad[face.z()], (regScale * bary2 * projMag) * normal);

	atomicAdd(&photoGradPixels[face.x()], 1.f);
	atomicAdd(&photoGradPixels[face.y()], 1.f);
	atomicAdd(&photoGradPixels[face.z()], 1.f);
}


// 10. UpdatePhotoGradNorm — 1D
__global__ void kernelUpdatePhotoGradNorm(
	float* __restrict__ photoGradNorm,
	const float* __restrict__ photoGradPixels,
	uint32_t numVertices)
{
	const int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid >= (int)numVertices) return;
	if (photoGradPixels[tid] > 0.f)
		photoGradNorm[tid] += 1.f;
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

	// a boundary vertex's own gradient is zeroed (matches CPU MeshRefine::ComputeSmoothnessGradient1/2,
	// SceneRefine.cpp); vertSizes[] always holds the TRUE valence for every vertex
	// (boundary or not) so that OTHER vertices' valence-weighted sum below stays correct when one
	// of their neighbours happens to be a boundary vertex (it used to be uploaded as 0 for
	// boundary vertices, which turned that neighbour's 1/vertSizes[ni] term into +inf)
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
	const float invN = 1.f / (float)numNeighbors;

	// Both modes: (1/N)*sum(neighbors) - vertex, the CPU's sign convention
	// (ComputeSmoothnessGradient1/2), so that the two backends' smoothGrad1/2
	// dumps compare directly; CombineAllGradients subtracts the rigidity term
	Point3 result = -vertices[tid];
	float totalWeight = 1.f;
	for (uint32_t i = 0; i < numNeighbors; ++i) {
		const uint32_t ni = vertVertices[ptr + i];
		result += vertices[ni] * invN;
		if (mode != 0) {
			// Valence-weighted: accumulate 1/(Ni*N) where Ni = TRUE valence of neighbor
			// (boundary neighbours are included, exactly as CPU's vertexVertices[ni].GetSize())
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

	const float norm = photoGradNorm[tid];
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

	const float norm = photoGradNorm[tid];
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
	const float* depthMapA, const float* depthMapB, uint8_t* mask,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB, cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj)
{
	const dim3 block(16, 16);
	const dim3 grid((camA.size.x() + block.x - 1) / block.x, (camA.size.y() + block.y - 1) / block.y);
	kernelImageMeshWarp<<<grid, block>>>(depthMapA, depthMapB, mask, camA, camB, texImageB, surfImageA, surfImageProj);
}

void LaunchComputeImageMean(const uint8_t* mask, float* imageMean, cudaSurfaceObject_t surfImage, int width, int height, int halfSize)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputeImageMean<<<grid, block>>>(mask, imageMean, surfImage, width, height, halfSize);
}

void LaunchComputeImageVar(const float* imageMean, const uint8_t* mask, float* imageVar, cudaSurfaceObject_t surfImage, int width, int height, int halfSize)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputeImageVar<<<grid, block>>>(imageMean, mask, imageVar, surfImage, width, height, halfSize);
}

void LaunchComputeImageCov(
	const float* imageMeanA, const float* imageMeanB, const uint8_t* mask, float* imageCov,
	cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj, int width, int height, int halfSize)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputeImageCov<<<grid, block>>>(imageMeanA, imageMeanB, mask, imageCov, surfImageA, surfImageProj, width, height, halfSize);
}

void LaunchComputeImageZNCC(const float* imageCov, const float* imageVarA, const float* imageVarB, const uint8_t* mask, float* imageZNCC, int width, int height, int halfSize)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputeImageZNCC<<<grid, block>>>(imageCov, imageVarA, imageVarB, mask, imageZNCC, width, height, halfSize);
}

void LaunchComputeImageDZNCC(
	const float* meanA, const float* meanB, const float* varA, const float* varB, const float* zncc,
	const uint8_t* mask, float* dzncc, cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj, int width, int height, int halfSize)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputeImageDZNCC<<<grid, block>>>(meanA, meanB, varA, varB, zncc, mask, dzncc, surfImageA, surfImageProj, width, height, halfSize);
}

void LaunchComputePhotometricGradient(
	const Point3u* faces, const Point3* normals,
	const float* depthMap, const uint32_t* faceMap, const uint16_t* baryMap,
	const float* dzncc, const uint8_t* mask,
	Point3* photoGrad, float* photoGradPixels, float* sgMap,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texGradXB, cudaTextureObject_t texGradYB, float regScale, int width, int height)
{
	const dim3 block(16, 16);
	const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
	kernelComputePhotometricGradient<<<grid, block>>>(
		faces, normals, depthMap, faceMap, baryMap, dzncc, mask,
		photoGrad, photoGradPixels, sgMap, camA, camB, texGradXB, texGradYB, regScale, width, height);
}

void LaunchUpdatePhotoGradNorm(float* photoGradNorm, const float* photoGradPixels, uint32_t numVertices)
{
	const int blockSize = 256;
	const int numBlocks = ((int)numVertices + blockSize - 1) / blockSize;
	kernelUpdatePhotoGradNorm<<<numBlocks, blockSize>>>(photoGradNorm, photoGradPixels, numVertices);
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
