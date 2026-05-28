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


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("PtchMtch"));

namespace MVS {

namespace CUDA {

// Kernel-launch serializer for the CUDA backend. The kernels read cameras/params
// from module-global __constant__ memory (g_cameras / g_params, see
// PatchMatchCUDA.cu), which is shared by every PatchMatch instance on the
// device. Concurrent overlap would race the __constant__ writes against an
// in-flight kernel's reads.
//
// Strategy: a global cudaEvent_t chains worker N+1's kernels behind worker N's
// kernels on the GPU side. A tiny host mutex covers only the queueing sequence
// {wait-event, upload-cameras, queue-kernels, record-event} so the host releases
// after queueing (~1ms) rather than after kernel execution (~80-400ms). Each
// worker then cudaStreamSynchronize's its own stream outside the mutex before
// reading results into its per-instance pinned buffer.
namespace {
std::mutex g_patchMatchCudaMutex;
cudaEvent_t g_constMemReady = nullptr;
std::once_flag g_constMemEventInit;
} // anonymous namespace

PatchMatch::PatchMatch()
	: cudaStream(0)
{
	// initialize CUDA device if needed
	if (SEACAVE::CUDA::devices.IsEmpty())
		SEACAVE::CUDA::initDevices(SEACAVE::CUDA::desiredDeviceIDs);
	CUDA_CHECK(cudaStreamCreate(&cudaStream));
}

PatchMatch::~PatchMatch()
{
	Release();
	if (cudaStream)
		cudaStreamDestroy(cudaStream);
}

void PatchMatch::Release()
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

	for (float*& p : hostImageStaging) if (p) cudaFreeHost(p);
	hostImageStaging.clear();
	hostImageStagingArea.clear();
	for (float*& p : hostDepthPriorStaging) if (p) cudaFreeHost(p);
	hostDepthPriorStaging.clear();
	hostDepthPriorStagingArea.clear();

	ReleaseCUDA();
}

// pinned staging wins on large images (driver-internal staging stall scales with
// area) but loses on small ones (cudaHostAlloc + explicit memcpy overhead is fixed)
void PatchMatch::StagedUploadCvMat(cudaArray_t dst, const cv::Mat1f& src,
	std::vector<float*>& slots, std::vector<size_t>& areas, size_t slotIdx)
{
	ASSERT(src.type() == CV_32FC1);
	const size_t area = (size_t)src.rows * (size_t)src.cols;
	const size_t rowBytes = (size_t)src.cols * sizeof(float);
	constexpr size_t kPinnedStagingThresholdArea = 1500000;
	if (area < kPinnedStagingThresholdArea) {
		CUDA_CHECK(cudaMemcpy2DToArrayAsync(dst, 0, 0, src.ptr<float>(), src.step[0],
			rowBytes, src.rows, cudaMemcpyHostToDevice, cudaStream));
		return;
	}
	if (slots.size() <= slotIdx) slots.resize(slotIdx + 1, nullptr);
	if (areas.size() <= slotIdx) areas.resize(slotIdx + 1, 0);
	if (slots[slotIdx] == nullptr || areas[slotIdx] < area) {
		if (slots[slotIdx]) CUDA_CHECK(cudaFreeHost(slots[slotIdx]));
		CUDA_CHECK(cudaHostAlloc((void**)&slots[slotIdx], area * sizeof(float), cudaHostAllocDefault));
		areas[slotIdx] = area;
	}
	float* dstPinned = slots[slotIdx];
	if (src.isContinuous() && src.step[0] == rowBytes) {
		memcpy(dstPinned, src.ptr<float>(), area * sizeof(float));
	} else {
		for (int r = 0; r < src.rows; ++r)
			memcpy(dstPinned + (size_t)r * src.cols, src.ptr<float>(r), rowBytes);
	}
	CUDA_CHECK(cudaMemcpy2DToArrayAsync(dst, 0, 0, dstPinned, rowBytes,
		rowBytes, src.rows, cudaMemcpyHostToDevice, cudaStream));
}

void PatchMatch::ReleaseCUDA()
{
	cudaFree(cudaTextureImages);
	cudaFree(cudaDepthNormalEstimates);
	cudaFree(cudaDepthNormalCosts);
	cudaFree(cudaRandStates);
	cudaFree(cudaSelectedViews);
	if (params.bGeomConsistency)
		cudaFree(cudaTextureDepths);
	if (depthNormalEstimates) {
		cudaFreeHost(depthNormalEstimates);
		depthNormalEstimates = NULL;
	}
}

void PatchMatch::Init(bool bGeomConsistency)
{
	if (bGeomConsistency) {
		params.bGeomConsistency = true;
		params.nEstimationIters = 1;
	} else {
		params.bGeomConsistency = false;
		params.nEstimationIters = OPTDENSE::nEstimationIters;
	}
}

void PatchMatch::AllocatePatchMatchCUDA(const cv::Mat1f& image)
{
	const size_t num_images = images.size();
	CUDA_CHECK(cudaMalloc((void**)&cudaTextureImages, sizeof(cudaTextureObject_t) * num_images));
	if (params.bGeomConsistency)
		CUDA_CHECK(cudaMalloc((void**)&cudaTextureDepths, sizeof(cudaTextureObject_t) * (num_images-1)));

	const size_t size = image.size().area();
	// pin estimates buffer so the H<->D copies on cudaStream run as true DMA-async without driver staging
	CUDA_CHECK(cudaHostAlloc((void**)&depthNormalEstimates, sizeof(Point4) * size, cudaHostAllocDefault));
	CUDA_CHECK(cudaMalloc((void**)&cudaDepthNormalEstimates, sizeof(Point4) * size));

	CUDA_CHECK(cudaMalloc((void**)&cudaDepthNormalCosts, sizeof(float) * size));
	CUDA_CHECK(cudaMalloc((void**)&cudaSelectedViews, sizeof(unsigned) * size));
	CUDA_CHECK(cudaMalloc((void**)&cudaRandStates, sizeof(curandState) * size));
}

void PatchMatch::AllocateImageCUDA(size_t i, const cv::Mat1f& image, bool bInitImage, bool bInitDepthMap)
{
	const cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);

	if (bInitImage) {
		CUDA_CHECK(cudaMallocArray(&cudaImageArrays[i], &channelDesc, image.cols, image.rows));

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

		CUDA_CHECK(cudaCreateTextureObject(&textureImages[i], &resDesc, &texDesc, NULL));
	}

	if (params.bGeomConsistency && i > 0) {
		if (!bInitDepthMap) {
			textureDepths[i-1] = 0;
			cudaDepthArrays[i-1] = NULL;
			return;
		}

		CUDA_CHECK(cudaMallocArray(&cudaDepthArrays[i-1], &channelDesc, image.cols, image.rows));

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

		CUDA_CHECK(cudaCreateTextureObject(&textureDepths[i-1], &resDesc, &texDesc, NULL));
	}
}

void PatchMatch::EstimateDepthMap(DepthData& depthData)
{
	TD_TIMER_STARTD();

	ASSERT(depthData.images.size() > 1);

	// multi-resolution
	DepthData& fullResDepthData(depthData);
	const unsigned totalScaleNumber(params.bGeomConsistency ? 0u : OPTDENSE::nSubResolutionLevels);
	DepthMap lowResDepthMap;
	NormalMap lowResNormalMap;
	ViewsMap lowResViewsMap;
	IIndex prevNumImages = (IIndex)images.size();
	const IIndex numImages = depthData.images.size();
	params.nNumViews = (int)numImages-1;
	params.nInitTopK = MINF(params.nInitTopK, params.nNumViews);
	params.fDepthMin = depthData.dMin;
	params.fDepthMax = depthData.dMax;
	if (prevNumImages < numImages) {
		images.resize(numImages);
		cameras.resize(numImages);
		cudaImageArrays.resize(numImages);
		textureImages.resize(numImages);
	}
	if (params.bGeomConsistency && cudaDepthArrays.size() < (size_t)params.nNumViews) {
		cudaDepthArrays.resize(params.nNumViews);
		textureDepths.resize(params.nNumViews);
	}
	const int maxPixelViews(MINF(params.nNumViews, 4));
	for (unsigned scaleNumber = totalScaleNumber+1; scaleNumber-- > 0; ) {
		// initialize
		const float scale = 1.f / POWI(2, scaleNumber);
		DepthData currentDepthData(DepthMapsData::ScaleDepthData(fullResDepthData, scale));
		DepthData& depthData(scaleNumber==0 ? fullResDepthData : currentDepthData);
		const Image8U::Size size(depthData.images.front().image.size());
		params.bLowResProcessed = false;
		if (scaleNumber != totalScaleNumber) {
			// all resolutions, but the smallest one, if multi-resolution is enabled
			params.bLowResProcessed = true;
			// INTER_NEAREST preserves [dMin, dMax] / normalized-normals / correct-view-IDs
			cv::resize(lowResDepthMap, depthData.depthMap, size, 0, 0, cv::INTER_NEAREST);
			cv::resize(lowResNormalMap, depthData.normalMap, size, 0, 0, cv::INTER_NEAREST);
			cv::resize(lowResViewsMap, depthData.viewsMap, size, 0, 0, cv::INTER_NEAREST);
			CUDA_CHECK(cudaMallocAsync((void**)&cudaLowDepths, sizeof(float) * size.area(), cudaStream));
		} else {
			if (totalScaleNumber > 0) {
				// smallest resolution, when multi-resolution is enabled
				fullResDepthData.depthMap.release();
				fullResDepthData.normalMap.release();
				fullResDepthData.confMap.release();
				fullResDepthData.viewsMap.release();
			}
			// smallest resolution if multi-resolution is enabled; highest otherwise
			if (depthData.viewsMap.empty())
				depthData.viewsMap.create(size);
		}
		if (scaleNumber == 0) {
			// highest resolution
			if (depthData.confMap.empty())
				depthData.confMap.create(size);
		}

		// set keep threshold to:
		params.fThresholdKeepCost = OPTDENSE::fNCCThresholdKeep;
		if (totalScaleNumber) {
			// multi-resolution enabled
			if (scaleNumber > 0 && scaleNumber != totalScaleNumber) {
				// all sub-resolutions, but the smallest and highest
				params.fThresholdKeepCost = 0.f; // disable filtering
			} else if (scaleNumber == totalScaleNumber || (!params.bGeomConsistency && OPTDENSE::nEstimationGeometricIters)) {
				// smallest sub-resolution OR highest resolution and geometric consistency is not running but enabled
				params.fThresholdKeepCost = OPTDENSE::fNCCThresholdKeep*1.2f;
			}
		} else {
			// multi-resolution disabled
			if (!params.bGeomConsistency && OPTDENSE::nEstimationGeometricIters) {
				// geometric consistency is not running but enabled
				params.fThresholdKeepCost = OPTDENSE::fNCCThresholdKeep*1.2f;
			}
		}

		for (IIndex i = 0; i < numImages; ++i) {
			const DepthData::ViewData& view = depthData.images[i];
			const Image32F image = view.image;
			const Camera camera(
				Eigen::Map<const SEACAVE::Matrix3x3::EMat>(view.camera.K.val).cast<float>(),
				Eigen::Map<const SEACAVE::Matrix3x3::EMat>(view.camera.R.val).cast<float>(),
				Eigen::Map<const SEACAVE::Point3::EVec>(view.camera.C.ptr()).cast<float>(),
				image.cols, image.rows);
			// store camera and image
			if (i == 0 && (prevNumImages < numImages || images[0].size() != image.size())) {
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
			// large images stage through per-instance pinned slot for a truly-async
			// H->D DMA on cudaStream; small images fall through to direct pageable
			// DMA inside StagedUploadCvMat (driver-internal staging is cheaper)
			StagedUploadCvMat(cudaImageArrays[i], image, hostImageStaging, hostImageStagingArea, (size_t)i);
			if (params.bGeomConsistency && i > 0 && !view.depthMap.empty()) {
				// set previously computed depth-map
				DepthMap depthMap(view.depthMap);
				if (depthMap.size() != image.size())
					cv::resize(depthMap, depthMap, image.size(), 0, 0, cv::INTER_LINEAR);
				StagedUploadCvMat(cudaDepthArrays[i-1], depthMap, hostDepthPriorStaging, hostDepthPriorStagingArea, (size_t)(i-1));
			}
			images[i] = std::move(image);
			cameras[i] = std::move(camera);
		}
		if (params.bGeomConsistency && cudaDepthArrays.size() > numImages - 1) {
			for (IIndex i = numImages; i < prevNumImages; ++i) {
				// free image CUDA memory
				cudaDestroyTextureObject(textureDepths[i-1]);
				cudaFreeArray(cudaDepthArrays[i-1]);
			}
			cudaDepthArrays.resize(params.nNumViews);
			textureDepths.resize(params.nNumViews);
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
		prevNumImages = numImages;

		// setup CUDA memory (queued on cudaStream)
		CUDA_CHECK(cudaMemcpyAsync(cudaTextureImages, textureImages.data(), sizeof(cudaTextureObject_t) * numImages, cudaMemcpyHostToDevice, cudaStream));
		if (params.bGeomConsistency) {
			// set previously computed depth-maps
			ASSERT(depthData.depthMap.size() == depthData.GetView().image.size());
			CUDA_CHECK(cudaMemcpyAsync(cudaTextureDepths, textureDepths.data(), sizeof(cudaTextureObject_t) * params.nNumViews, cudaMemcpyHostToDevice, cudaStream));
		}

		// load depth-map and normal-map into CUDA memory
		for (int r = 0; r < depthData.depthMap.rows; ++r) {
			const int baseIndex = r * depthData.depthMap.cols;
			for (int c = 0; c < depthData.depthMap.cols; ++c) {
				const Normal& n = depthData.normalMap(r, c);
				const int index = baseIndex + c;
				Point4& depthNormal = depthNormalEstimates[index];
				depthNormal.topLeftCorner<3, 1>() = Eigen::Map<const Normal::EVec>(n.ptr());
				depthNormal.w() = depthData.depthMap(r, c);
			}
		}
		// pinned host buffer => DMA-async on cudaStream
		CUDA_CHECK(cudaMemcpyAsync(cudaDepthNormalEstimates, depthNormalEstimates, sizeof(Point4) * depthData.depthMap.size().area(), cudaMemcpyHostToDevice, cudaStream));

		// load low resolution depth-map into CUDA memory
		if (params.bLowResProcessed) {
			ASSERT(depthData.depthMap.isContinuous());
			CUDA_CHECK(cudaMemcpyAsync(cudaLowDepths, depthData.depthMap.ptr<float>(), sizeof(float) * depthData.depthMap.size().area(), cudaMemcpyHostToDevice, cudaStream));
		}

		// run CUDA patch-match: GPU-side event chains successive workers'
		// kernel sequences so the next worker's __constant__ writes wait for
		// the previous worker's kernels to finish reading them. Host mutex
		// covers only the queueing window, not kernel execution.
		ASSERT(!depthData.viewsMap.empty());
		std::call_once(g_constMemEventInit, []() {
			CUDA_CHECK(cudaEventCreateWithFlags(&g_constMemReady, cudaEventDisableTiming));
		});
		{
			std::lock_guard<std::mutex> queueLock(g_patchMatchCudaMutex);
			CUDA_CHECK(cudaStreamWaitEvent(cudaStream, g_constMemReady, 0));
			UploadCameras();
			RunCUDA(depthData.confMap.getData(), (uint32_t*)depthData.viewsMap.getData());
			CUDA_CHECK(cudaEventRecord(g_constMemReady, cudaStream));
		}
		// wait for our own kernels + D2H copies to finish before the unpack loop
		// reads from the pinned host buffer
		CUDA_CHECK(cudaStreamSynchronize(cudaStream));
		CUDA_CHECK(cudaGetLastError());
		if (params.bLowResProcessed)
			CUDA_CHECK(cudaFreeAsync(cudaLowDepths, cudaStream));

		// load depth-map, normal-map and confidence-map from CUDA memory
		for (int r = 0; r < depthData.depthMap.rows; ++r) {
			for (int c = 0; c < depthData.depthMap.cols; ++c) {
				const int index = r * depthData.depthMap.cols + c;
				const Point4& depthNormal = depthNormalEstimates[index];
				const Depth depth = depthNormal.w();
				ASSERT(ISFINITE(depth));
				depthData.depthMap(r, c) = depth;
				depthData.normalMap(r, c) = depthNormal.topLeftCorner<3, 1>();
				if (scaleNumber == 0) {
					// converted ZNCC [0-2] score, where 0 is best, to [0-1] confidence, where 1 is best
					ASSERT(!depthData.confMap.empty());
					float& conf = depthData.confMap(r, c);
					conf = conf >= 1.f ? 0.f : 1.f - conf;
					// map pixel views from bit-mask to index
					ASSERT(!depthData.viewsMap.empty());
					ViewsID& views = depthData.viewsMap(r, c);
					if (depth > 0) {
						const uint32_t bitviews(*reinterpret_cast<const uint32_t*>(views.val));
						int j = 0;
						for (int i = 0; i < 32; ++i) {
							if (bitviews & (1 << i)) {
								views[j] = i;
								if (++j == maxPixelViews)
									break;
							}
						}
						while (j < 4)
							views[j++] = 255;
					} else
						views = ViewsID(255, 255, 255, 255);
				}
			}
		}

		// remember sub-resolution estimates for next iteration
		if (scaleNumber > 0) {
			lowResDepthMap = depthData.depthMap;
			lowResNormalMap = depthData.normalMap;
			lowResViewsMap = depthData.viewsMap;
		}
	}

	// apply ignore mask
	if (OPTDENSE::nIgnoreMaskLabel >= 0) {
		const DepthData::ViewData& view = depthData.GetView();
		BitMatrix mask;
		if (DepthEstimator::ImportIgnoreMask(*view.pImageData, depthData.depthMap.size(), (uint8_t)OPTDENSE::nIgnoreMaskLabel, mask))
			depthData.ApplyIgnoreMask(mask);
	}

	DEBUG_EXTRA("Depth-map for image %3u %s: %dx%d (%s)", depthData.images.front().GetID(),
		depthData.images.GetSize() > 2 ?
		String::FormatString("estimated using %2u images", depthData.images.size()-1).c_str() :
		String::FormatString("with image %3u estimated", depthData.images[1].GetID()).c_str(),
		images.front().cols, images.front().rows, TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS

#pragma pop_macro("VERBOSE")

#endif // _USE_CUDA
