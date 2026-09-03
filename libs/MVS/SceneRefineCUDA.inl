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

// rasterizer, per visible face, launched twice: resolve=false does an atomicMin of the
// (depth, position-in-faceIDs) key into projKey (one 64-bit word per pixel, pre-filled with
// ~0ull); resolve=true lets the thread holding each pixel's winning key write depth/face/bary
// (faceMap pre-filled with NO_ID, depthMap with 0). In Debug, LaunchCheckProjection then asserts
// every covered pixel received its payload.
void LaunchProjectMesh(
	const Point3* vertices, const Point3u* faces, const uint32_t* faceIDs,
	unsigned long long* projKey, float* depthMap, uint32_t* faceMap, uint16_t* baryMap,
	const Camera& camera, uint32_t numFacesView, bool resolve);

#ifdef _DEBUG
void LaunchCheckProjection(
	const uint32_t* faceIDs, const unsigned long long* projKey,
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

// masked window statistics, rejection gates, ZNCC and its derivative in one pass; maskOut is a
// SECOND mask buffer (the window loops still read mask), pruned of the pixels whose window held
// fewer than Refine::MinWindowCount valid samples or that failed a gate -- every consumer
// downstream reads maskOut. zncc/conf are the parity diagnostic's optional outputs (NULL in
// production); sumR/sumRZ are the device scalars ScoreMesh reduces into S, and blockSums is the
// scratch this launch's two kernels hand them over in -- 2 floats per 16x16 block of the largest
// view, so that S is a fixed sequence of additions.
void LaunchComputeWindowStats(
	const uint8_t* mask, uint8_t* maskOut, float* dzncc, float* zncc, float* conf,
	cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj,
	float* sumR, float* sumRZ, float* blockSums, float gateMeanDiff, float gateVarRatio, int width, int height);

// the photometric accumulation, in two atomic-free halves so that the per-vertex sums are
// bit-reproducible run to run (float addition is not associative). First half: one thread per
// MESH face (not per face of this view -- see kernelAccumulateFacePhoto), reducing that face's
// pixels into private
// per-face slots; every thread writes its slots, so no buffer needs clearing between
// pair-directions. faceAcc holds 3 floats per face (one per corner), facePixels/faceFoot one;
// faceFoot is only read where facePixels > 0. texImageB is the image B texture, sampled directly
// (four texel fetches) when bBilinearGrad asks for the derivative of the bilinear interpolant
// instead of the precomputed gradient stencil (OPTREFINE::nImageGradient == 3, where
// texGradXB/texGradYB carry no texture and are unused).
void LaunchAccumulateFacePhoto(
	const Point3* vertices, const Point3u* faces, const Point3* normals,
	const float* depthMap, const uint32_t* faceMap, const uint16_t* baryMap,
	const float* dzncc, const uint8_t* mask,
	float* faceAcc, float* facePixels, float* faceFoot, float* sgMap,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB, cudaTextureObject_t texGradXB, cudaTextureObject_t texGradYB,
	bool bBilinearGrad, float regScale,
	uint32_t numFaces);

// Second half: one thread per vertex, folding its incident faces' slots in the fixed order
// Mesh::ListIncidentFaces produced (vertFaces/vertFaceSizes/vertFacePointers, the same flattening
// as vertVertices). It also does this pair-direction's photoGradNorm += 1 bookkeeping.
void LaunchGatherVertexPhoto(
	const Point3u* faces, const Point3* normals,
	const uint32_t* vertFaces, const uint32_t* vertFaceSizes, const uint32_t* vertFacePointers,
	const float* faceAcc, const float* facePixels, const float* faceFoot,
	Point3* photoGrad, float* photoGradNorm, float* footprint,
	uint32_t numVertices);

// once per ScoreMesh(), after every pair-direction: resolves the footprint sentinel now that
// photoGradNorm holds its final per-vertex count for this ScoreMesh() call.
void LaunchFinalizePhotoGrad(const float* photoGradNorm, float* footprint, uint32_t numVertices);

// mode selects the level (0 - level 1, over vertex positions; nonzero - level 2, over
// smoothGrad1)
void LaunchComputeSmoothnessGradient(
	const Point3* vertices, const uint32_t* vertVertices,
	const uint32_t* vertSizes, const uint32_t* vertPointers,
	const uint8_t* vertBoundary, Point3* smoothGrad, uint32_t numVertices, uint8_t mode);

void LaunchCombineGradients(
	Point3* photoGrad, const float* photoGradNorm,
	const Point3* smoothGrad, uint32_t numVertices, float smoothWeight);

void LaunchCombineAllGradients(
	Point3* photoGrad, const float* photoGradNorm,
	const Point3* smoothGrad1, const Point3* smoothGrad2,
	uint32_t numVertices, float rigidity, float elasticity);

void LaunchComputeFaceNormal(
	const Point3* vertices, const Point3u* faces,
	Point3* normals, uint32_t numFaces);
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS

#endif // _MVS_SCENEREFINECUDA_INL_
