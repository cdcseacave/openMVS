/*
* PatchMatchCUDA.cpp
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

#include "Common.h"
#include "PatchMatchCUDA.h"
#include "DepthMap.h"

#ifdef _USE_CUDA

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

PatchMatchCUDA::PatchMatchCUDA(int device)
{
	// initialize CUDA device if needed
	if (CUDA::devices.IsEmpty())
		CUDA::initDevice(device);
}

PatchMatchCUDA::~PatchMatchCUDA()
{
	Release();
}

void PatchMatchCUDA::Release()
{
	if (images.empty())
		return;

	FOREACH(i, cudaImageArrays) {
		cudaDestroyTextureObject(textureImages[i]);
		cudaFreeArray(cudaImageArrays[i]);
	}
	cudaImageArrays.clear();

	if (params.bGeomConsistency) {
		FOREACH(i, cudaDepthArrays) {
			cudaDestroyTextureObject(textureDepths[i]);
			cudaFreeArray(cudaDepthArrays[i]);
		}
		cudaDepthArrays.clear();
	}

	images.clear();
	cameras.clear();

	ReleaseCUDA();
}

void PatchMatchCUDA::ReleaseCUDA()
{
	cudaFree(cudaTextureImages);
	cudaFree(cudaCameras);
	cudaFree(cudaDepthNormalEstimates);
	cudaFree(cudaDepthNormalCosts);
	cudaFree(cudaRandStates);
	cudaFree(cudaSelectedViews);

	if (params.bGeomConsistency)
		cudaFree(cudaTextureDepths);

	delete[] depthNormalEstimates;
	delete[] depthNormalCosts;
}

void PatchMatchCUDA::Init(bool geom_consistency)
{
	if (geom_consistency) {
		params.bGeomConsistency = true;
		params.nEstimationIters = 1;
	} else {
		params.bGeomConsistency = false;
		params.nEstimationIters = OPTDENSE::nEstimationIters;
	}
}

void PatchMatchCUDA::AllocatePatchMatchCUDA(const cv::Mat1f& image)
{
	const size_t num_images = images.size();
	CUDA::checkCudaCall(cudaMalloc((void**)&cudaTextureImages, sizeof(cudaTextureObject_t) * num_images));
	CUDA::checkCudaCall(cudaMalloc((void**)&cudaCameras, sizeof(Camera) * num_images));
	if (params.bGeomConsistency)
		CUDA::checkCudaCall(cudaMalloc((void**)&cudaTextureDepths, sizeof(cudaTextureObject_t) * (num_images-1)));

	const size_t size = image.size().area();
	depthNormalEstimates = new Point4[size];
	CUDA::checkCudaCall(cudaMalloc((void**)&cudaDepthNormalEstimates, sizeof(Point4) * size));

	depthNormalCosts = new float[size];
	CUDA::checkCudaCall(cudaMalloc((void**)&cudaDepthNormalCosts, sizeof(float) * size));

	CUDA::checkCudaCall(cudaMalloc((void**)&cudaRandStates, sizeof(curandState) * size));
	CUDA::checkCudaCall(cudaMalloc((void**)&cudaSelectedViews, sizeof(unsigned) * size));
}

void PatchMatchCUDA::AllocateImageCUDA(size_t i, const cv::Mat1f& image, bool bInitImage, bool bInitDepthMap)
{
	const cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);

	if (bInitImage) {
		CUDA::checkCudaCall(cudaMallocArray(&cudaImageArrays[i], &channelDesc, image.cols, image.rows));

		struct cudaResourceDesc resDesc;
		memset(&resDesc, 0, sizeof(cudaResourceDesc));
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = cudaImageArrays[i];

		struct cudaTextureDesc texDesc;
		memset(&texDesc, 0, sizeof(cudaTextureDesc));
		texDesc.addressMode[0] = cudaAddressModeWrap;
		texDesc.addressMode[1] = cudaAddressModeWrap;
		texDesc.filterMode = cudaFilterModeLinear;
		texDesc.readMode  = cudaReadModeElementType;
		texDesc.normalizedCoords = 0;

		CUDA::checkCudaCall(cudaCreateTextureObject(&textureImages[i], &resDesc, &texDesc, NULL));
	}

	if (params.bGeomConsistency && i > 0) {
		if (!bInitDepthMap) {
			textureDepths[i-1] = 0;
			cudaDepthArrays[i-1] = NULL;
			return;
		}

		CUDA::checkCudaCall(cudaMallocArray(&cudaDepthArrays[i-1], &channelDesc, image.cols, image.rows));

		struct cudaResourceDesc resDesc;
		memset(&resDesc, 0, sizeof(cudaResourceDesc));
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = cudaDepthArrays[i-1];

		struct cudaTextureDesc texDesc;
		memset(&texDesc, 0, sizeof(cudaTextureDesc));
		texDesc.addressMode[0] = cudaAddressModeWrap;
		texDesc.addressMode[1] = cudaAddressModeWrap;
		texDesc.filterMode = cudaFilterModeLinear;
		texDesc.readMode  = cudaReadModeElementType;
		texDesc.normalizedCoords = 0;

		CUDA::checkCudaCall(cudaCreateTextureObject(&textureDepths[i-1], &resDesc, &texDesc, NULL));
	}
}

void PatchMatchCUDA::EstimateDepthMap(DepthData& depthData)
{
	TD_TIMER_STARTD();

	ASSERT(depthData.images.size() > 1);
	const IIndex prevNumImages = (IIndex)images.size();
	const IIndex numImages = depthData.images.size();
	params.nNumViews = (int)numImages-1;
	params.nInitTopK = std::min(params.nInitTopK, params.nNumViews);
	params.fDepthMin = depthData.dMin;
	params.fDepthMax = depthData.dMax;
	if (prevNumImages < numImages) {
		images.resize(numImages);
		cameras.resize(numImages);
		cudaImageArrays.resize(numImages);
		textureImages.resize(numImages);
	}
	if (params.bGeomConsistency && cudaDepthArrays.size() < numImages-1) {
		cudaDepthArrays.resize(numImages-1);
		textureDepths.resize(numImages-1);
	}
	for (IIndex i = 0; i < numImages; ++i) {
		const DepthData::ViewData& view = depthData.images[i];
		Image32F image = view.image;
		Camera camera;
		camera.K = Eigen::Map<const SEACAVE::Matrix3x3::EMat>(view.camera.K.val).cast<float>();
		camera.R = Eigen::Map<const SEACAVE::Matrix3x3::EMat>(view.camera.R.val).cast<float>();
		camera.C = Eigen::Map<const SEACAVE::Point3::EVec>(view.camera.C.ptr()).cast<float>();
		camera.height = image.rows;
		camera.width = image.cols;
		// store camera and image
		if (i == 0 && prevNumImages < numImages) {
			// allocate/reallocate PatchMatch CUDA memory
			if (prevNumImages > 0)
				ReleaseCUDA();
			AllocatePatchMatchCUDA(image);
		}
		if (i >= prevNumImages) {
			// allocate image CUDA memory
			AllocateImageCUDA(i, image, true, !view.depthMap.empty());
		} else
		if (images[i].size() != image.size()) {
			// reallocate image CUDA memory
			cudaDestroyTextureObject(textureImages[i]);
			cudaFreeArray(cudaImageArrays[i]);
			if (params.bGeomConsistency && i > 0) {
				cudaDestroyTextureObject(textureDepths[i-1]);
				cudaFreeArray(cudaDepthArrays[i-1]);
			}
			AllocateImageCUDA(i, image, true, !view.depthMap.empty());
		} else
		if (params.bGeomConsistency && i > 0 && (view.depthMap.empty() != (cudaDepthArrays[i-1] == NULL))) {
			// reallocate depth CUDA memory
			if (cudaDepthArrays[i-1]) {
				cudaDestroyTextureObject(textureDepths[i-1]);
				cudaFreeArray(cudaDepthArrays[i-1]);
			}
			AllocateImageCUDA(i, image, false, !view.depthMap.empty());
		}
		CUDA::checkCudaCall(cudaMemcpy2DToArray(cudaImageArrays[i], 0, 0, image.ptr<float>(), image.step[0], image.cols*sizeof(float), image.rows, cudaMemcpyHostToDevice));
		if (params.bGeomConsistency && i > 0 && !view.depthMap.empty()) {
			// set previously computed depth-map
			DepthMap depthMap(view.depthMap);
			if (depthMap.size() != image.size())
				cv::resize(depthMap, depthMap, image.size(), 0, 0, cv::INTER_LINEAR);
			CUDA::checkCudaCall(cudaMemcpy2DToArray(cudaDepthArrays[i-1], 0, 0, depthMap.ptr<float>(), depthMap.step[0], sizeof(float)*depthMap.cols, depthMap.rows, cudaMemcpyHostToDevice));
		}
		images[i] = std::move(image);
		cameras[i] = std::move(camera);
	}
	if (params.bGeomConsistency && cudaDepthArrays.size() > numImages-1) {
		for (IIndex i = numImages; i < prevNumImages; ++i) {
			// free image CUDA memory
			cudaDestroyTextureObject(textureDepths[i-1]);
			cudaFreeArray(cudaDepthArrays[i-1]);
		}
		cudaDepthArrays.resize(numImages-1);
		textureDepths.resize(numImages-1);
	}
	if (prevNumImages > numImages) {
		for (IIndex i = numImages; i < prevNumImages; ++i) {
			// free image CUDA memory
			cudaDestroyTextureObject(textureImages[i]);
			cudaFreeArray(cudaImageArrays[i]);
		}
		images.resize(numImages);
		cameras.resize(numImages);
		cudaImageArrays.resize(numImages);
		textureImages.resize(numImages);
	}

	// setup CUDA memory
	CUDA::checkCudaCall(cudaMemcpy(cudaTextureImages, textureImages.data(), sizeof(cudaTextureObject_t)*numImages, cudaMemcpyHostToDevice));
	CUDA::checkCudaCall(cudaMemcpy(cudaCameras, cameras.data(), sizeof(Camera)*numImages, cudaMemcpyHostToDevice));
	if (params.bGeomConsistency) {
		// set previously computed depth-maps
		ASSERT(depthData.depthMap.size() == depthData.GetView().image.size());
		CUDA::checkCudaCall(cudaMemcpy(cudaTextureDepths, textureDepths.data(), sizeof(cudaTextureObject_t)*(numImages-1), cudaMemcpyHostToDevice));
	}

	// load depth-map and normal-map into CUDA memory
	for (int r = 0; r < depthData.depthMap.rows; ++r) {
		for (int c = 0; c < depthData.depthMap.cols; ++c) {
			const Normal& n = depthData.normalMap(r,c);
			const int index = r * depthData.depthMap.cols + c;
			Point4& depthNormal = depthNormalEstimates[index];
			depthNormal.topLeftCorner<3,1>() = Eigen::Map<const Normal::EVec>(n.ptr());
			depthNormal.w() = depthData.depthMap(r,c);
		}
	}
	CUDA::checkCudaCall(cudaMemcpy(cudaDepthNormalEstimates, depthNormalEstimates, sizeof(Point4) * depthData.depthMap.size().area(), cudaMemcpyHostToDevice));

	// run CUDA patch-match
	RunCUDA();
	CUDA::checkCudaCall(cudaGetLastError());

	// load depth-map, normal-map and confidence-map from CUDA memory
	const float fNCCThresholdKeep(!params.bGeomConsistency && OPTDENSE::nEstimationGeometricIters ?
		OPTDENSE::fNCCThresholdKeepCUDA * 1.5f : OPTDENSE::fNCCThresholdKeepCUDA);
	if (depthData.confMap.empty())
		depthData.confMap.create(depthData.depthMap.size());
	for (int r = 0; r < depthData.depthMap.rows; ++r) {
		for (int c = 0; c < depthData.depthMap.cols; ++c) {
			const int index = r * depthData.depthMap.cols + c;
			const Point4& depthNormal = depthNormalEstimates[index];
			ASSERT(std::isfinite(depthNormal.w()));
			// check if the score is good enough
			// and that the cross-estimates is close enough to the current estimate
			float& conf = depthData.confMap(r,c);
			conf = depthNormalCosts[index];
			ASSERT(std::isfinite(conf));
			if (depthNormal.w() <= 0 || conf >= fNCCThresholdKeep) {
				conf = 0;
				depthData.depthMap(r,c) = 0;
				depthData.normalMap(r,c) = Normal::ZERO;
			} else {
				depthData.depthMap(r,c) = depthNormal.w();
				depthData.normalMap(r,c) = depthNormal.topLeftCorner<3,1>();
				// converted ZNCC [0-2] score, where 0 is best, to [0-1] confidence, where 1 is best
				conf = conf>=1.f ? 0.f : 1.f-conf;
			}
		}
	}

	DEBUG_EXTRA("Depth-map for image %3u %s: %dx%d (%s)", depthData.images.front().GetID(),
		depthData.images.GetSize() > 2 ?
		String::FormatString("estimated using %2u images", depthData.images.size()-1).c_str() :
		String::FormatString("with image %3u estimated", depthData.images[1].GetID()).c_str(),
		images.front().cols, images.front().rows, TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/

#endif // _USE_CUDA
