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
// (faceMap pre-filled with NO_ID). LaunchResolveProjection then zeroes the uncovered pixels
// (0 / NO_ID / 0) and, in Debug, asserts every covered pixel received its payload.
void LaunchProjectMesh(
	const Point3* vertices, const Point3u* faces, const uint32_t* faceIDs,
	unsigned long long* projKey, float* depthMap, uint32_t* faceMap, uint16_t* baryMap,
	const Camera& camera, uint32_t numFacesView, bool resolve);

void LaunchResolveProjection(
	const uint32_t* faceIDs, const unsigned long long* projKey,
	float* depthMap, uint32_t* faceMap, uint16_t* baryMap, int width, int height);

void LaunchImageMeshWarp(
	const float* depthMapA, const float* depthMapB, uint8_t* mask,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texImageB,
	cudaSurfaceObject_t surfImageA,
	cudaSurfaceObject_t surfImageProj);

void LaunchComputeImageMean(
	const uint8_t* mask, float* imageMean,
	cudaSurfaceObject_t surfImage,
	int width, int height, int halfSize);

void LaunchComputeImageVar(
	const float* imageMean, const uint8_t* mask, float* imageVar,
	cudaSurfaceObject_t surfImage,
	int width, int height, int halfSize);

void LaunchComputeImageCov(
	const float* imageMeanA, const float* imageMeanB,
	const uint8_t* mask, float* imageCov,
	cudaSurfaceObject_t surfImageA,
	cudaSurfaceObject_t surfImageProj,
	int width, int height, int halfSize);

void LaunchComputeImageZNCC(
	const float* imageCov, const float* imageVarA, const float* imageVarB,
	const uint8_t* mask, float* imageZNCC,
	int width, int height, int halfSize);

void LaunchComputeImageDZNCC(
	const float* meanA, const float* meanB,
	const float* varA, const float* varB, const float* zncc,
	const uint8_t* mask, float* dzncc,
	cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj,
	int width, int height, int halfSize);

void LaunchComputePhotometricGradient(
	const Point3u* faces, const Point3* normals,
	const float* depthMap, const uint32_t* faceMap, const uint16_t* baryMap,
	const float* dzncc, const uint8_t* mask,
	Point3* photoGrad, float* photoGradPixels, float* sgMap,
	const Camera& camA, const Camera& camB,
	cudaTextureObject_t texGradXB, cudaTextureObject_t texGradYB, float regScale,
	int width, int height);

void LaunchUpdatePhotoGradNorm(
	float* photoGradNorm, const float* photoGradPixels, uint32_t numVertices);

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
