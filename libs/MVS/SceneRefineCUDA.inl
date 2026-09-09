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

// Launcher function declarations for all mesh refinement CUDA kernels

// rasterizer over the whole mesh, one thread per face, launched twice: resolve=false does an
// atomicMin of the (depth, face id) key into projKey (one 64-bit word per pixel, pre-filled with
// ~0ull); resolve=true lets the thread holding each pixel's winning key write depth/face (faceMap
// pre-filled with NO_ID, depthMap with 0) and, when ownerBits is not NULL, sets bit f of it for
// every face f that wrote a pixel ((numFaces+31)/32 words, every word written, no clearing
// needed) -- the bits LaunchAccumulateFacePhoto skips the unseen faces by. In Debug,
// LaunchCheckProjection then asserts every covered pixel received its payload.
void LaunchProjectMesh(
	const Point3* vertices, const Point3u* faces,
	unsigned long long* projKey, float* depthMap, uint32_t* faceMap, uint32_t* ownerBits,
	const Camera& camera, uint32_t numFaces, bool resolve);

#ifdef _DEBUG
void LaunchCheckProjection(
	const unsigned long long* projKey,
	const float* depthMap, const uint32_t* faceMap, int width, int height);
#endif

// keepA/keepB are the per-pixel keep-masks of image A/B (one byte per pixel, non-zero = keep),
// NULL if disabled -- see kernelImageMeshWarp
void LaunchImageMeshWarp(
	const float* depthMapA, const float* depthMapB,
	const uint8_t* keepA, const uint8_t* keepB, uint8_t* mask,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB,
	cudaSurfaceObject_t surfImageProj);

// masked window statistics, rejection gates, ZNCC, its derivative and the pixel's photometric
// term in one pass: pixelGrad receives g_p, the scalar the covering face's three corners share
// before their barycentrics (computePhotoPixel), and maskOut the pixels that contribute one -- a
// SECOND mask buffer (the window loops still read mask), pruned of the pixels whose window held
// fewer than Refine::MinWindowCount valid samples, that failed a gate or that see the surface at
// a grazing angle; kernelAccumulateFacePhoto reads both. blockSums receives this pair-direction's
// per-block partials of the two reliability sums S is reduced from, 2 floats per 16x16 block from
// slot 0 of the passed pointer on; the number of blocks written is returned, and
// LaunchReduceBlockSums folds every direction's slots once per ScoreMesh() in slot order, so
// that S is a fixed sequence of additions. texImageB is sampled directly (four texel fetches)
// when bBilinearGrad asks for the derivative of the bilinear interpolant instead of the
// precomputed gradient stencil (OPTREFINE::nImageGradient == 3, where texGradXB/texGradYB carry
// no texture and are unused).
uint32_t LaunchComputeWindowStats(
	const uint8_t* mask, uint8_t* maskOut, float* pixelGrad, float* blockSums,
	cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj,
	const Point3* normals, const uint32_t* faceMap, const float* depthMap,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB, cudaTextureObject_t texGradXB, cudaTextureObject_t texGradYB,
	bool bBilinearGrad, float regScale, float gateMeanDiff, float gateVarRatio, int width, int height);

// sumR/sumRZ receive the sums of the numSlots partials
void LaunchReduceBlockSums(const float* blockSums, uint32_t numSlots, float* sumR, float* sumRZ);

// the photometric accumulation, atomic-free so that the per-vertex sums are bit-reproducible run
// to run (float addition is not associative). Per pair-direction: one thread per MESH face,
// exiting on the reference view's owner bit (LaunchProjectMesh) unless the face owns a pixel
// there, folds that face's contributing pixels -- barycentrics and depth recomputed from its
// projection, see kernelAccumulateFacePhoto -- into its private slots -- faceAcc, 3 floats per
// face (one per corner: Sum g_p*b_c), facePixels, the contributing pixel count, and faceFoot,
// the min footprint (only read where facePixels > 0) -- accumulated ACROSS the pair-directions
// of one ScoreMesh() (cleared by the host once per call), and marks the face's three vertices in
// vertexSeen.
void LaunchAccumulateFacePhoto(
	const Point3* vertices, const Point3u* faces, const uint32_t* ownerBits,
	const uint32_t* faceMap, const float* pixelGrad, const uint8_t* mask,
	float* faceAcc, float* facePixels, float* faceFoot, uint8_t* vertexSeen,
	const Camera& camA, uint32_t numFaces);

// right after it: one thread per vertex counts this pair-direction in photoCount for every
// marked vertex and clears the mark (exactly the CPU's `photoGradNorm[idxVert] += 1.f`)
void LaunchCountSeenVertices(uint8_t* vertexSeen, float* photoCount, uint32_t numVertices);

// once per ScoreMesh(), after every pair-direction: one thread per vertex folds its incident
// faces' slots in the fixed order Mesh::ListIncidentFaces produced (vertFaces/vertFaceSizes/
// vertFacePointers, the same flattening as vertVertices) into photoGrad and footprint, the latter
// 0 exactly where no face contributed (contract: footprint[v] > 0 iff photoCount[v] > 0)
void LaunchGatherVertexPhoto(
	const Point3u* faces, const Point3* normals,
	const uint32_t* vertFaces, const uint32_t* vertFaceSizes, const uint32_t* vertFacePointers,
	const float* faceAcc, const float* facePixels, const float* faceFoot,
	Point3* photoGrad, float* footprint, uint32_t numVertices);

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
