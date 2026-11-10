/*
* SceneRefineCUDA.inl
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

#ifndef _MVS_SCENEREFINECUDA_INL_
#define _MVS_SCENEREFINECUDA_INL_


// I N C L U D E S /////////////////////////////////////////////////

#include "CUDA/Camera.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

namespace CUDA {

// Launchers of the mesh refinement kernels (see the kernels in SceneRefineCUDA.cu for the
// contracts). Those taking a stream are the ones MeshRefineCUDA::ScoreMesh records into the
// evaluation graph; the others run on the legacy stream.

// the rasterizer over the whole mesh, launched twice per view: resolve=false does an atomicMin
// of the (depth, face id) key into projKey (one 64-bit word per pixel, pre-filled with ~0ull);
// resolve=true lets the thread holding each pixel's winning key write its face (faceMap
// pre-filled with NO_ID; no depth is stored, the kernels recompute it from the face) and, when ownerBits is not NULL, sets bit f of it for
// every face f that wrote a pixel ((numFaces+31)/32 words, every word written)
void LaunchProjectMesh(
	const Point3* vertices, const Point3u* faces,
	unsigned long long* projKey, uint32_t* faceMap, uint32_t* ownerBits,
	const Camera& camera, uint32_t numFaces, bool resolve);

#ifdef _DEBUG
// after both passes: every covered pixel received its payload
void LaunchCheckProjection(
	const Point3* vertices, const Point3u* faces,
	const unsigned long long* projKey, const uint32_t* faceMap,
	const Camera& camera, int width, int height);
#endif

// the dense per-view owner lists, once per evaluation after every view is rasterized: counts
// receives per view the popcount of its numWords owner words and offsets (numViews+1 entries)
// their exclusive prefix sum then the total; ownerList (sized by the host to that total)
// receives every view's owner faces in face order from its offset on; then every view's slice
// of listIn is reordered into listOut by the image tile (2^tileShiftX x 2^tileShiftY pixels)
// the face's box starts in, keys being scratch of the same size, with at most maxBuckets tiles
// in any view, cameras one Camera per view
void LaunchCountOwners(const uint32_t* ownerBits, uint32_t* counts, uint32_t* offsets, uint32_t numWords, uint32_t numViews);
void LaunchCompactOwners(const uint32_t* ownerBits, const uint32_t* offsets, uint32_t* ownerList, uint32_t numWords, uint32_t numViews);
void LaunchSortOwnersTile(
	const Point3* vertices, const Point3u* faces, const Camera* cameras,
	const uint32_t* offsets, const uint32_t* listIn, uint32_t* keys, uint32_t* listOut,
	int tileShiftX, int tileShiftY, uint32_t maxBuckets, uint32_t numViews);

// the depth map of a view (0 where uncovered) recomputed from its face map (pixelDepth: the
// depth the rasterizer computed), once per accumulation group for the reference view
void LaunchFaceDepth(
	const Point3* vertices, const Point3u* faces, const uint32_t* faceMap, float* depthMap,
	const Camera& camera, cudaStream_t stream);

// image B warped into A through the mesh, and mask, the pixels that made it; depthMapA is A's
// depth map (LaunchFaceDepth), B's depth at the tap is recomputed from faceMapB; keepA/keepB are
// the per-pixel keep-masks (one byte per pixel, non-zero = keep), NULL if disabled
void LaunchImageMeshWarp(
	const Point3* vertices, const Point3u* faces,
	const float* depthMapA, const uint32_t* faceMapB,
	const uint8_t* keepA, const uint8_t* keepB, uint8_t* mask,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB,
	cudaSurfaceObject_t surfImageProj,
	cudaStream_t stream);

// the masked window statistics, rejection gates, ZNCC, its derivative and the pixel's
// photometric term in one pass: a pixel that contributes one adds g_p, the scalar the covering
// face's corners share before their barycentrics, to gradSum and sets dirBit in dirBits (the
// reference view's sums over its pair-directions, cleared by the host before the first one,
// read by LaunchAccumulateFacePhoto after the last). blockSums receives this pair-direction's per-block
// partials of the two reliability sums S is reduced from, 2 floats per 16x16 block from slot 0
// of the passed pointer on; returns the number of blocks written. gradMode is
// OPTREFINE::nImageGradient: texImageB is differenced directly for the central difference (1) and
// the derivative of the bilinear interpolant (3); texGradXB/texGradYB, the precomputed stencil
// textures, exist and are read only for the stencils that do not commute with the sampling (0, 2).
uint32_t LaunchComputeWindowStats(
	const uint8_t* mask, float* gradSum, uint32_t* dirBits, uint32_t dirBit, float* blockSums,
	cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj,
	const Point3* normals, const uint32_t* faceMap, const float* depthMap,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB, cudaTextureObject_t texGradXB, cudaTextureObject_t texGradYB,
	int gradMode, float regScale, float gateMeanDiff, float gateVarRatio, int width, int height,
	cudaStream_t stream);

// the atomic-free photometric accumulation of one reference view's pair-directions (up to 32,
// one bit each): one thread per face the reference view idxView owns (its slice of ownerList,
// read from ownerOffsets on the device; numOwners sizes the grid, with a margin, for the
// evaluations that replay the launch) folds its contributing pixels (gradSum: their terms summed
// over the directions; dirBits: the directions they contribute to) into its private slots,
// faceAcc (3 floats per face, one per corner: Sum g_p*b_c), facePixels (the contributing
// (pixel, direction) count) and faceFoot (the min footprint, read only where facePixels > 0),
// accumulated across the groups of one evaluation (cleared by the host once per evaluation), and
// counts each direction once per vertex in photoCount through vertexStamp (one 64-bit word per
// vertex: the group and its directions seen, set to ~0 by the host once per evaluation)
void LaunchAccumulateFacePhoto(
	const Point3* vertices, const Point3u* faces,
	const uint32_t* ownerList, const uint32_t* ownerOffsets, uint32_t idxView,
	const uint32_t* faceMap, const float* gradSum, const uint32_t* dirBits,
	float* faceAcc, float* facePixels, float* faceFoot,
	unsigned long long* vertexStamp, float* photoCount, uint32_t group,
	const Camera& camA, uint32_t numOwners, cudaStream_t stream);

// once per evaluation after every pair-direction: sumR/sumRZ receive the sums of the numSlots
// partials, in slot order ...
void LaunchReduceBlockSums(const float* blockSums, uint32_t numSlots, float* sumR, float* sumRZ, cudaStream_t stream);
// ... and one thread per vertex folds its incident faces' slots, in the fixed order of
// vertFaces/vertFaceSizes/vertFacePointers (Mesh::ListIncidentFaces, flattened like
// vertVertices), into photoGrad and footprint, the latter 0 exactly where no face contributed
void LaunchGatherVertexPhoto(
	const Point3u* faces, const Point3* normals,
	const uint32_t* vertFaces, const uint32_t* vertFaceSizes, const uint32_t* vertFacePointers,
	const float* faceAcc, const float* facePixels, const float* faceFoot,
	Point3* photoGrad, float* footprint, uint32_t numVertices, cudaStream_t stream);

// mode selects the level (0 - level 1, over vertex positions; nonzero - level 2, over
// smoothGrad1)
void LaunchComputeSmoothnessGradient(
	const Point3* vertices, const uint32_t* vertVertices,
	const uint32_t* vertSizes, const uint32_t* vertPointers,
	const uint8_t* vertBoundary, Point3* smoothGrad, uint32_t numVertices, uint8_t mode);

void LaunchComputeFaceNormal(
	const Point3* vertices, const Point3u* faces,
	Point3* normals, uint32_t numFaces);

// the preparation's projected face areas (MeshRefineCUDA::ListFaceAreas): hist, zeroed by the
// caller, receives the rasterized pixel count of every face in one view's face map ...
void LaunchFaceHistogram(const uint32_t* faceMap, uint32_t* hist, uint32_t numPixels);
// ... and maxAreas, zeroed once by the caller, the largest over the pairs of the smaller of a
// pair's two counts, truncated to 16 bits like the host's uint16_t counters
void LaunchReduceFaceAreasPair(const uint32_t* histA, const uint32_t* histB, uint16_t* maxAreas, uint32_t numFaces);
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS

#endif // _MVS_SCENEREFINECUDA_INL_
