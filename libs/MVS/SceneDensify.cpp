/*
* SceneDensify.cpp
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

#include "Common.h"
#include "Scene.h"
#include "SceneDensify.h"
#include "PatchMatchCUDA.h"
#include "DMapCache.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define DENSE_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

// Dense3D data.events
enum EVENT_TYPE {
	EVT_FAIL = 0,
	EVT_CLOSE,

	EVT_PROCESSIMAGE,

	EVT_ESTIMATEDEPTHMAP,
	EVT_OPTIMIZEDEPTHMAP,
	EVT_SAVEDEPTHMAP,

	EVT_FILTERDEPTHMAP,
	EVT_ADJUSTDEPTHMAP,
};

class EVTFail : public Event
{
public:
	EVTFail() : Event(EVT_FAIL) {}
};
class EVTClose : public Event
{
public:
	EVTClose() : Event(EVT_CLOSE) {}
};

class EVTProcessImage : public Event
{
public:
	IIndex idxImage;
	EVTProcessImage(IIndex _idxImage) : Event(EVT_PROCESSIMAGE), idxImage(_idxImage) {}
};

class EVTEstimateDepthMap : public Event
{
public:
	IIndex idxImage;
	EVTEstimateDepthMap(IIndex _idxImage) : Event(EVT_ESTIMATEDEPTHMAP), idxImage(_idxImage) {}
};
class EVTOptimizeDepthMap : public Event
{
public:
	IIndex idxImage;
	EVTOptimizeDepthMap(IIndex _idxImage) : Event(EVT_OPTIMIZEDEPTHMAP), idxImage(_idxImage) {}
};
class EVTSaveDepthMap : public Event
{
public:
	IIndex idxImage;
	EVTSaveDepthMap(IIndex _idxImage) : Event(EVT_SAVEDEPTHMAP), idxImage(_idxImage) {}
};

class EVTFilterDepthMap : public Event
{
public:
	IIndex idxImage;
	EVTFilterDepthMap(IIndex _idxImage) : Event(EVT_FILTERDEPTHMAP), idxImage(_idxImage) {}
};
class EVTAdjustDepthMap : public Event
{
public:
	IIndex idxImage;
	EVTAdjustDepthMap(IIndex _idxImage) : Event(EVT_ADJUSTDEPTHMAP), idxImage(_idxImage) {}
};
/*----------------------------------------------------------------*/


// convert the ZNCC score to a weight used to average the fused points
inline float Conf2Weight(float conf, Depth depth) {
	return 1.f/(MAXF(1.f-conf,0.03f)*depth*depth);
}
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////


DepthMapsData::DepthMapsData(Scene& _scene)
	:
	scene(_scene),
	arrDepthData(_scene.images.GetSize())
{
} // constructor

DepthMapsData::~DepthMapsData()
{
} // destructor
/*----------------------------------------------------------------*/

// compute visibility for the reference image (the first image in "images")
// and select the best views for reconstructing the depth-map;
// extract also all 3D points seen by the reference image
bool DepthMapsData::SelectViews(DepthData& depthData)
{
	// find and sort valid neighbor views
	const IIndex idxImage((IIndex)(&depthData-arrDepthData.Begin()));
	ASSERT(depthData.neighbors.IsEmpty());
	if (scene.images[idxImage].neighbors.empty() &&
		!scene.SelectNeighborViews(idxImage, depthData.points, OPTDENSE::nMinViews, OPTDENSE::nMinViewsTrustPoint>1?OPTDENSE::nMinViewsTrustPoint:2, FD2R(OPTDENSE::fOptimAngle), OPTDENSE::nPointInsideROI))
		return false;
	depthData.neighbors.CopyOf(scene.images[idxImage].neighbors);

	// remove invalid neighbor views
	const float fMinArea(OPTDENSE::fMinArea);
	const float fMinScale(0.2f), fMaxScale(3.2f);
	const float fMinAngle(FD2R(OPTDENSE::fMinAngle));
	const float fMaxAngle(FD2R(OPTDENSE::fMaxAngle));
	const unsigned nMaxViews(MAXF(OPTDENSE::nMaxViews, OPTDENSE::nNumViews));
	if (!Scene::FilterNeighborViews(depthData.neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews)) {
		DEBUG_EXTRA("error: reference image %3u has no good images in view", idxImage);
		return false;
	}
	return true;
} // SelectViews
/*----------------------------------------------------------------*/

// select target image for the reference image (the first image in "images"),
// initialize images data, and initialize depth-map and normal-map;
// if idxNeighbor is not NO_ID, only the reference image and the given neighbor are initialized;
// if numNeighbors is not 0, only the first numNeighbors neighbors are initialized;
// otherwise all are initialized;
// if loadImages, the image data is also setup
// if loadDepthMaps is 1, the depth-maps are loaded from disk,
// if 0, the reference depth-map is initialized from sparse point cloud,
// and if -1, the depth-maps are not initialized
// returns false if there are no good neighbors to estimate the depth-map
bool DepthMapsData::InitViews(DepthData& depthData, IIndex idxNeighbor, IIndex numNeighbors, bool loadImages, int loadDepthMaps)
{
	const IIndex idxImage((IIndex)(&depthData-arrDepthData.Begin()));
	ASSERT(!depthData.neighbors.IsEmpty());

	// set this image the first image in the array
	depthData.images.Empty();
	depthData.images.Reserve(depthData.neighbors.GetSize()+1);
	depthData.images.AddEmpty();

	if (idxNeighbor != NO_ID) {
		// set target image as the given neighbor
		const ViewScore& neighbor = depthData.neighbors[idxNeighbor];
		DepthData::ViewData& viewTrg = depthData.images.AddEmpty();
		viewTrg.pImageData = &scene.images[neighbor.ID];
		viewTrg.scale = neighbor.scale;
		viewTrg.camera = viewTrg.pImageData->camera;
		if (loadImages) {
			viewTrg.pImageData->image.toGray(viewTrg.image, cv::COLOR_BGR2GRAY, true);
			if (DepthData::ViewData::ScaleImage(viewTrg.image, viewTrg.image, viewTrg.scale))
				viewTrg.camera = viewTrg.pImageData->GetCamera(scene.platforms, viewTrg.image.size());
		} else {
			if (DepthData::ViewData::NeedScaleImage(viewTrg.scale))
				viewTrg.camera = viewTrg.pImageData->GetCamera(scene.platforms, Image8U::computeResize(viewTrg.pImageData->image.size(), viewTrg.scale));
		}
		DEBUG_EXTRA("Reference image %3u paired with image %3u", idxImage, neighbor.ID);
	} else {
		// initialize all neighbor views too (global reconstruction is used)
		const float fMinScore(MAXF(depthData.neighbors.First().score*OPTDENSE::fViewMinScoreRatio, OPTDENSE::fViewMinScore));
		for (const ViewScore& neighbor: depthData.neighbors) {
			if ((numNeighbors && depthData.images.GetSize() > numNeighbors) ||
				(neighbor.score < fMinScore))
				break;
			DepthData::ViewData& viewTrg = depthData.images.AddEmpty();
			viewTrg.pImageData = &scene.images[neighbor.ID];
			viewTrg.scale = neighbor.scale;
			viewTrg.camera = viewTrg.pImageData->camera;
			if (loadImages) {
				viewTrg.pImageData->image.toGray(viewTrg.image, cv::COLOR_BGR2GRAY, true);
				if (DepthData::ViewData::ScaleImage(viewTrg.image, viewTrg.image, viewTrg.scale))
					viewTrg.camera = viewTrg.pImageData->GetCamera(scene.platforms, viewTrg.image.size());
			} else {
				if (DepthData::ViewData::NeedScaleImage(viewTrg.scale))
					viewTrg.camera = viewTrg.pImageData->GetCamera(scene.platforms, Image8U::computeResize(viewTrg.pImageData->image.size(), viewTrg.scale));
			}
		}
		#if TD_VERBOSE != TD_VERBOSE_OFF
		// print selected views
		if (g_nVerbosityLevel > 2) {
			String msg;
			for (IIndex i=1; i<depthData.images.size(); ++i)
				msg += String::FormatString(" %3u(%.2fscl)", depthData.images[i].GetID(), depthData.images[i].scale);
			VERBOSE("Reference image %3u paired with %u views:%s (%u shared points)", idxImage, depthData.images.size()-1, msg.c_str(), depthData.points.GetSize());
		} else
		DEBUG_EXTRA("Reference image %3u paired with %u views", idxImage, depthData.images.size()-1);
		#endif
	}
	if (depthData.images.size() < 2) {
		depthData.images.Release();
		return false;
	}

	// initialize reference image as well
	DepthData::ViewData& viewRef = depthData.images.front();
	viewRef.scale = 1;
	viewRef.pImageData = &scene.images[idxImage];
	viewRef.camera = viewRef.pImageData->camera;
	if (loadImages)
		viewRef.pImageData->image.toGray(viewRef.image, cv::COLOR_BGR2GRAY, true);
	depthData.size = viewRef.pImageData->image.size();

	// initialize views
	for (IIndex i=1; i<depthData.images.size(); ++i) {
		DepthData::ViewData& view = depthData.images[i];
		if (loadDepthMaps > 0) {
			// load known depth-map
			String imageFileName;
			IIndexArr IDs;
			cv::Size imageSize;
			Depth dMin, dMax;
			NormalMap normalMap;
			ConfidenceMap confMap;
			ViewsMap viewsMap;
			ImportDepthDataRaw(ComposeDepthFilePath(view.GetID(), "dmap"),
				imageFileName, IDs, imageSize, view.cameraDepthMap.K, view.cameraDepthMap.R, view.cameraDepthMap.C,
				dMin, dMax, view.depthMap, normalMap, confMap, viewsMap, 1);
			ASSERT(viewRef.image.size() == view.depthMap.size());
		}
		view.Init(viewRef.camera);
	}

	if (loadDepthMaps > 0) {
		// load known depth-map and normal-map
		String imageFileName;
		IIndexArr IDs;
		cv::Size imageSize;
		Camera camera;
		ConfidenceMap confMap;
		ViewsMap viewsMap;
		if (!ImportDepthDataRaw(ComposeDepthFilePath(viewRef.GetID(), "dmap"),
				imageFileName, IDs, imageSize, camera.K, camera.R, camera.C, depthData.dMin, depthData.dMax,
				depthData.depthMap, depthData.normalMap, confMap, viewsMap, 3))
			return false;
		ASSERT(viewRef.image.size() == depthData.depthMap.size());
		ASSERT(depthData.normalMap.empty() || viewRef.image.size() == depthData.normalMap.size());
		if (depthData.normalMap.empty()) {
			// estimate normal map
			EstimateNormalMap(viewRef.camera.K, depthData.depthMap, depthData.normalMap);
		}
	} else if (loadDepthMaps == 0) {
		// initialize depth and normal maps
		if (OPTDENSE::nMinViewsTrustPoint < 2 || depthData.points.empty()) {
			// compute depth range and initialize known depths, else random
			const Image8U::Size size(viewRef.image.size());
			depthData.depthMap.create(size); depthData.depthMap.memset(0);
			depthData.normalMap.create(size);
			if (depthData.points.empty()) {
				// all values will be initialized randomly
				depthData.dMin = 1e-1f;
				depthData.dMax = 1e+2f;
			} else {
				// initialize with the sparse point-cloud
				const int nPixelArea(2); // half windows size around a pixel to be initialize with the known depth
				depthData.dMin = FLT_MAX;
				depthData.dMax = 0;
				FOREACHPTR(pPoint, depthData.points) {
					const PointCloud::Point& X = scene.pointcloud.points[*pPoint];
					const Point3 camX(viewRef.camera.TransformPointW2C(Cast<REAL>(X)));
					const ImageRef x(ROUND2INT(viewRef.camera.TransformPointC2I(camX)));
					const float d((float)camX.z);
					const ImageRef sx(MAXF(x.x-nPixelArea,0), MAXF(x.y-nPixelArea,0));
					const ImageRef ex(MINF(x.x+nPixelArea,size.width-1), MINF(x.y+nPixelArea,size.height-1));
					for (int y=sx.y; y<=ex.y; ++y) {
						for (int x=sx.x; x<=ex.x; ++x) {
							depthData.depthMap(y,x) = d;
							depthData.normalMap(y,x) = Normal::ZERO;
						}
					}
					if (depthData.dMin > d)
						depthData.dMin = d;
					if (depthData.dMax < d)
						depthData.dMax = d;
				}
				depthData.dMin *= 0.9f;
				depthData.dMax *= 1.1f;
			}
		} else {
			ASSERT(!depthData.points.empty());
			// compute rough estimates using the sparse point-cloud
			InitDepthMap(depthData);
		}
	}
	return true;
} // InitViews
/*----------------------------------------------------------------*/

// roughly estimate depth and normal maps by triangulating the sparse point cloud
// and interpolating normal and depth for all pixels
bool DepthMapsData::InitDepthMap(DepthData& depthData)
{
	TD_TIMER_STARTD();

	ASSERT(depthData.images.GetSize() > 1 && !depthData.points.IsEmpty());
	const DepthData::ViewData& image(depthData.GetView());
	TriangulatePoints2DepthMap(image, scene.pointcloud, depthData.points, depthData.depthMap, depthData.normalMap, depthData.dMin, depthData.dMax, OPTDENSE::bAddCorners, OPTDENSE::bInitSparse);
	depthData.dMin *= 0.9f;
	depthData.dMax *= 1.1f;

	#if TD_VERBOSE != TD_VERBOSE_OFF
	// save rough depth map as image
	if (g_nVerbosityLevel > 4) {
		ExportDepthMap(ComposeDepthFilePath(image.GetID(), "init.png"), depthData.depthMap);
		ExportNormalMap(ComposeDepthFilePath(image.GetID(), "init.normal.png"), depthData.normalMap);
		ExportPointCloud(ComposeDepthFilePath(image.GetID(), "init.ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
	}
	#endif

	DEBUG_ULTIMATE("Depth-map %3u roughly estimated from %u sparse points: %dx%d (%s)", image.GetID(), depthData.points.size(), image.image.width(), image.image.height(), TD_TIMER_GET_FMT().c_str());
	return true;
} // InitDepthMap
/*----------------------------------------------------------------*/


// initialize the confidence map (NCC score map) with the score of the current estimates
void* STCALL DepthMapsData::ScoreDepthMapTmp(void* arg)
{
	DepthEstimator& estimator = *((DepthEstimator*)arg);
	IDX idx;
	while ((idx=(IDX)Thread::safeInc(estimator.idxPixel)) < estimator.coords.GetSize()) {
		const ImageRef& x = estimator.coords[idx];
		if (!estimator.PreparePixelPatch(x) || !estimator.FillPixelPatch()) {
			estimator.depthMap0(x) = 0;
			estimator.normalMap0(x) = Normal::ZERO;
			estimator.confMap0(x) = 2.f;
			continue;
		}
		Depth& depth = estimator.depthMap0(x);
		Normal& normal = estimator.normalMap0(x);
		const Normal viewDir(Cast<float>(static_cast<const Point3&>(estimator.X0)));
		if (!ISINSIDE(depth, estimator.dMin, estimator.dMax)) {
			// init with random values
			depth = estimator.RandomDepth(estimator.dMinSqr, estimator.dMaxSqr);
			normal = estimator.RandomNormal(viewDir);
		} else if (normal.dot(viewDir) >= 0) {
			// replace invalid normal with random values
			normal = estimator.RandomNormal(viewDir);
		}
		ASSERT(ISEQUAL(norm(normal), 1.f));
		estimator.confMap0(x) = estimator.ScorePixel(depth, normal);
	}
	return NULL;
}
// run propagation and random refinement cycles
void* STCALL DepthMapsData::EstimateDepthMapTmp(void* arg)
{
	DepthEstimator& estimator = *((DepthEstimator*)arg);
	IDX idx;
	while ((idx=(IDX)Thread::safeInc(estimator.idxPixel)) < estimator.coords.GetSize())
		estimator.ProcessPixel(idx);
	return NULL;
}
// remove all estimates with too big score and invert confidence map
void* STCALL DepthMapsData::EndDepthMapTmp(void* arg)
{
	DepthEstimator& estimator = *((DepthEstimator*)arg);
	IDX idx;
	MAYBEUNUSED const float fOptimAngle(FD2R(OPTDENSE::fOptimAngle));
	while ((idx=(IDX)Thread::safeInc(estimator.idxPixel)) < estimator.coords.GetSize()) {
		const ImageRef& x = estimator.coords[idx];
		ASSERT(estimator.depthMap0(x) >= 0);
		Depth& depth = estimator.depthMap0(x);
		float& conf = estimator.confMap0(x);
		// check if the score is good enough
		// and that the cross-estimates is close enough to the current estimate
		if (depth <= 0 || conf >= OPTDENSE::fNCCThresholdKeep) {
			conf = 0;
			depth = 0;
			estimator.normalMap0(x) = Normal::ZERO;
		} else {
			#if 1
			// converted ZNCC [0-2] score, where 0 is best, to [0-1] confidence, where 1 is best
			conf = conf>=1.f ? 0.f : 1.f-conf;
			#else
			#if 1
			FOREACH(i, estimator.images)
				estimator.scores[i] = ComputeAngle<REAL,float>(estimator.image0.camera.TransformPointI2W(Point3(x,depth)).ptr(), estimator.image0.camera.C.ptr(), estimator.images[i].view.camera.C.ptr());
			#if DENSE_AGGNCC == DENSE_AGGNCC_NTH
			const float fCosAngle(estimator.scores.GetNth(estimator.idxScore));
			#elif DENSE_AGGNCC == DENSE_AGGNCC_MEAN
			const float fCosAngle(estimator.scores.mean());
			#elif DENSE_AGGNCC == DENSE_AGGNCC_MIN
			const float fCosAngle(estimator.scores.minCoeff());
			#else
			const float fCosAngle(estimator.idxScore ?
				std::accumulate(estimator.scores.begin(), &estimator.scores.PartialSort(estimator.idxScore), 0.f) / estimator.idxScore :
				*std::min_element(estimator.scores.cbegin(), estimator.scores.cend()));
			#endif
			const float wAngle(MINF(POW(ACOS(fCosAngle)/fOptimAngle,1.5f),1.f));
			#else
			const float wAngle(1.f);
			#endif
			#if 1
			conf = wAngle/MAXF(conf,1e-2f);
			#else
			conf = wAngle/(depth*SQUARE(MAXF(conf,1e-2f)));
			#endif
			#endif
		}
	}
	return NULL;
}

DepthData DepthMapsData::ScaleDepthData(const DepthData& inputDeptData, float scale) {
	ASSERT(scale <= 1);
	if (scale == 1)
		return inputDeptData;
	DepthData rescaledDepthData(inputDeptData);
	FOREACH (idxView, rescaledDepthData.images) {
		DepthData::ViewData& viewData = rescaledDepthData.images[idxView];
		ASSERT(viewData.depthMap.empty() || viewData.image.size() == viewData.depthMap.size());
		cv::resize(viewData.image, viewData.image, cv::Size(), scale, scale, cv::INTER_AREA);
		viewData.camera = viewData.pImageData->camera;
		viewData.camera.K = viewData.camera.GetScaledK(viewData.pImageData->GetSize(), viewData.image.size());
		if (!viewData.depthMap.empty()) {
			cv::resize(viewData.depthMap, viewData.depthMap, viewData.image.size(), 0, 0, cv::INTER_AREA);
			viewData.cameraDepthMap = viewData.pImageData->camera;
			viewData.cameraDepthMap.K = viewData.cameraDepthMap.GetScaledK(viewData.pImageData->GetSize(), viewData.image.size());
		}
		viewData.Init(rescaledDepthData.images[0].camera);
	}
	if (!rescaledDepthData.depthMap.empty())
		cv::resize(rescaledDepthData.depthMap, rescaledDepthData.depthMap, cv::Size(), scale, scale, cv::INTER_NEAREST);
	if (!rescaledDepthData.normalMap.empty())
		cv::resize(rescaledDepthData.normalMap, rescaledDepthData.normalMap, cv::Size(), scale, scale, cv::INTER_NEAREST);
	return rescaledDepthData;
}

// estimate depth-map using propagation and random refinement with NCC score
// as in: "Accurate Multiple View 3D Reconstruction Using Patch-Based Stereo for Large-Scale Scenes", S. Shen, 2013
// The implementations follows closely the paper, although there are some changes/additions.
// Given two views of the same scene, we note as the "reference image" the view for which a depth-map is reconstructed, and the "target image" the other view.
// As a first step, the whole depth-map is approximated by interpolating between the available sparse points.
// Next, the depth-map is passed from top/left to bottom/right corner and the opposite sens for each of the next steps.
// For each pixel, first the current depth estimate is replaced with its neighbor estimates if the NCC score is better.
// Second, the estimate is refined by trying random estimates around the current depth and normal values, keeping the one with the best score.
// The estimation can be stopped at any point, and usually 2-3 iterations are enough for convergence.
// For each pixel, the depth and normal are scored by computing the NCC score between the patch in the reference image and the wrapped patch in the target image, as dictated by the homography matrix defined by the current values to be estimate.
// In order to ensure some smoothness while locally estimating each pixel, a bonus is added to the NCC score if the estimate for this pixel is close to the estimates for the neighbor pixels.
// Optionally, the occluded pixels can be detected by extending the described iterations to the target image and removing the estimates that do not have similar values in both views.
//  - nGeometricIter: current geometric-consistent estimation iteration (-1 - normal patch-match)
bool DepthMapsData::EstimateDepthMap(IIndex idxImage, int nGeometricIter)
{
	#ifdef _USE_CUDA
	if (pmCUDA) {
		pmCUDA->EstimateDepthMap(arrDepthData[idxImage]);
		return true;
	}
	#endif // _USE_CUDA

	TD_TIMER_STARTD();

	const unsigned nMaxThreads(scene.nMaxThreads);
	const unsigned iterBegin(nGeometricIter < 0 ? 0u : OPTDENSE::nEstimationIters+(unsigned)nGeometricIter);
	const unsigned iterEnd(nGeometricIter < 0 ? OPTDENSE::nEstimationIters : iterBegin+1);

	// init threads
	ASSERT(nMaxThreads > 0);
	cList<DepthEstimator> estimators;
	estimators.reserve(nMaxThreads);
	cList<SEACAVE::Thread> threads;
	if (nMaxThreads > 1)
		threads.resize(nMaxThreads-1); // current thread is also used
	volatile Thread::safe_t idxPixel;

	// Multi-Resolution : 
	DepthData& fullResDepthData(arrDepthData[idxImage]);
	const unsigned totalScaleNumber(nGeometricIter < 0 ? OPTDENSE::nSubResolutionLevels : 0u);
	DepthMap lowResDepthMap;
	NormalMap lowResNormalMap;
	#if DENSE_NCC == DENSE_NCC_WEIGHTED
	DepthEstimator::WeightMap weightMap0;
	#else
	Image64F imageSum0;
	#endif
	DepthMap currentSizeResDepthMap;
	for (unsigned scaleNumber = totalScaleNumber+1; scaleNumber-- > 0; ) {
		// initialize
		float scale = 1.f / POWI(2, scaleNumber);
		DepthData currentDepthData(ScaleDepthData(fullResDepthData, scale));
		DepthData& depthData(scaleNumber==0 ? fullResDepthData : currentDepthData);
		ASSERT(depthData.images.size() > 1);
		const DepthData::ViewData& image(depthData.images.front());
		ASSERT(!image.image.empty() && !depthData.images[1].image.empty());
		const Image8U::Size size(image.image.size());
		if (scaleNumber != totalScaleNumber) {
			cv::resize(lowResDepthMap, depthData.depthMap, size, 0, 0, OPTDENSE::nIgnoreMaskLabel >= 0 ? cv::INTER_NEAREST : cv::INTER_LINEAR);
			cv::resize(lowResNormalMap, depthData.normalMap, size, 0, 0, cv::INTER_NEAREST);
			depthData.depthMap.copyTo(currentSizeResDepthMap);
		}
		else if (totalScaleNumber > 0) {
			fullResDepthData.depthMap.release();
			fullResDepthData.normalMap.release();
			fullResDepthData.confMap.release();
		}
		depthData.confMap.create(size);

		// init integral images and index to image-ref map for the reference data
		#if DENSE_NCC == DENSE_NCC_WEIGHTED
		weightMap0.clear();
		weightMap0.resize(size.area()-(size.width+1)*DepthEstimator::nSizeHalfWindow);
		#else
		cv::integral(image.image, imageSum0, CV_64F);
		#endif
		if (prevDepthMapSize != size || OPTDENSE::nIgnoreMaskLabel >= 0) {
			BitMatrix mask;
			if (OPTDENSE::nIgnoreMaskLabel >= 0 && DepthEstimator::ImportIgnoreMask(*image.pImageData, depthData.depthMap.size(), (uint16_t)OPTDENSE::nIgnoreMaskLabel, mask))
				depthData.ApplyIgnoreMask(mask);
			DepthEstimator::MapMatrix2ZigzagIdx(size, coords, mask, MAXF(64,(int)nMaxThreads*8));
			#if 0 && !defined(_RELEASE)
			// show pixels to be processed
			Image8U cmask(size);
			cmask.memset(0);
			for (const DepthEstimator::MapRef& x: coords)
				cmask(x.y, x.x) = 255;
			cmask.Show("cmask");
			#endif
			prevDepthMapSize = size;
		}

		// initialize the reference confidence map (NCC score map) with the score of the current estimates
		{
			// create working threads
			idxPixel = -1;
			ASSERT(estimators.empty());
			while (estimators.size() < nMaxThreads) {
				estimators.emplace_back(iterBegin, depthData, idxPixel,
					#if DENSE_NCC == DENSE_NCC_WEIGHTED
					weightMap0,
					#else
					imageSum0,
					#endif
					coords);
				estimators.Last().lowResDepthMap = currentSizeResDepthMap;
			}
			ASSERT(estimators.size() == threads.size()+1);
			FOREACH(i, threads)
				threads[i].start(ScoreDepthMapTmp, &estimators[i]);
			ScoreDepthMapTmp(&estimators.back());
			// wait for the working threads to close
			FOREACHPTR(pThread, threads)
				pThread->join();
			estimators.clear();
			#if TD_VERBOSE != TD_VERBOSE_OFF
			// save rough depth map as image
			if (g_nVerbosityLevel > 4 && nGeometricIter < 0) {
				ExportDepthMap(ComposeDepthFilePath(image.GetID(), "rough.png"), depthData.depthMap);
				ExportNormalMap(ComposeDepthFilePath(image.GetID(), "rough.normal.png"), depthData.normalMap);
				ExportPointCloud(ComposeDepthFilePath(image.GetID(), "rough.ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
			}
			#endif
		}

		// run propagation and random refinement cycles on the reference data
		for (unsigned iter=iterBegin; iter<iterEnd; ++iter) {
			// create working threads
			idxPixel = -1;
			ASSERT(estimators.empty());
			while (estimators.size() < nMaxThreads) {
				estimators.emplace_back(iter, depthData, idxPixel,
					#if DENSE_NCC == DENSE_NCC_WEIGHTED
					weightMap0,
					#else
					imageSum0,
					#endif
					coords);
				estimators.Last().lowResDepthMap = currentSizeResDepthMap;
			}
			ASSERT(estimators.size() == threads.size()+1);
			FOREACH(i, threads)
				threads[i].start(EstimateDepthMapTmp, &estimators[i]);
			EstimateDepthMapTmp(&estimators.back());
			// wait for the working threads to close
			FOREACHPTR(pThread, threads)
				pThread->join();
			estimators.clear();
			#if 1 && TD_VERBOSE != TD_VERBOSE_OFF
			// save intermediate depth map as image
			if (g_nVerbosityLevel > 4) {
				String path(ComposeDepthFilePath(image.GetID(), "iter")+String::ToString(iter));
				if (nGeometricIter >= 0)
					path += String::FormatString(".geo%d", nGeometricIter);
				ExportDepthMap(path+".png", depthData.depthMap);
				ExportNormalMap(path+".normal.png", depthData.normalMap);
				ExportPointCloud(path+".ply", *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
			}
			#endif
		}

		// remember sub-resolution estimates for next iteration
		if (scaleNumber > 0) {
			lowResDepthMap = depthData.depthMap;
			lowResNormalMap = depthData.normalMap;
		}
	}

	DepthData& depthData(fullResDepthData);
	// remove all estimates with too big score and invert confidence map
	{
		const float fNCCThresholdKeep(OPTDENSE::fNCCThresholdKeep);
		if (nGeometricIter < 0 && OPTDENSE::nEstimationGeometricIters)
			OPTDENSE::fNCCThresholdKeep *= 1.333f;
		// create working threads
		idxPixel = -1;
		ASSERT(estimators.empty());
		while (estimators.size() < nMaxThreads)
			estimators.emplace_back(0, depthData, idxPixel,
				#if DENSE_NCC == DENSE_NCC_WEIGHTED
				weightMap0,
				#else
				imageSum0,
				#endif
				coords);
		ASSERT(estimators.size() == threads.size()+1);
		FOREACH(i, threads)
			threads[i].start(EndDepthMapTmp, &estimators[i]);
		EndDepthMapTmp(&estimators.back());
		// wait for the working threads to close
		FOREACHPTR(pThread, threads)
			pThread->join();
		estimators.clear();
		OPTDENSE::fNCCThresholdKeep = fNCCThresholdKeep;
	}

	DEBUG_EXTRA("Depth-map for image %3u %s: %dx%d (%s)", depthData.images.front().GetID(),
		depthData.images.size() > 2 ?
			String::FormatString("estimated using %2u images", depthData.images.size()-1).c_str() :
			String::FormatString("with image %3u estimated", depthData.images[1].GetID()).c_str(),
		depthData.depthMap.cols, depthData.depthMap.rows, TD_TIMER_GET_FMT().c_str());
	return true;
} // EstimateDepthMap
/*----------------------------------------------------------------*/


// filter out small depth segments from the given depth map
bool DepthMapsData::RemoveSmallSegments(DepthData& depthData)
{
	const float fDepthDiffThreshold(OPTDENSE::fDepthDiffThreshold*0.7f);
	unsigned speckle_size = OPTDENSE::nSpeckleSize;
	DepthMap& depthMap = depthData.depthMap;
	NormalMap& normalMap = depthData.normalMap;
	ConfidenceMap& confMap = depthData.confMap;
	ASSERT(!depthMap.empty());
	const ImageRef size(depthMap.size());

	// allocate memory on heap for dynamic programming arrays
	TImage<bool> done_map(size, false);
	CAutoPtrArr<ImageRef> seg_list(new ImageRef[size.x*size.y]);
	unsigned seg_list_count;
	unsigned seg_list_curr;
	ImageRef neighbor[4];

	// for all pixels do
	for (int u=0; u<size.x; ++u) {
		for (int v=0; v<size.y; ++v) {
			// if the first pixel in this segment has been already processed => skip
			if (done_map(v,u))
				continue;

			// init segment list (add first element
			// and set it to be the next element to check)
			seg_list[0] = ImageRef(u,v);
			seg_list_count = 1;
			seg_list_curr  = 0;

			// add neighboring segments as long as there
			// are none-processed pixels in the seg_list;
			// none-processed means: seg_list_curr<seg_list_count
			while (seg_list_curr < seg_list_count) {
				// get address of current pixel in this segment
				const ImageRef addr_curr(seg_list[seg_list_curr]);
				const Depth& depth_curr = depthMap(addr_curr);

				if (depth_curr>0) {
					// fill list with neighbor positions
					neighbor[0] = ImageRef(addr_curr.x-1, addr_curr.y  );
					neighbor[1] = ImageRef(addr_curr.x+1, addr_curr.y  );
					neighbor[2] = ImageRef(addr_curr.x  , addr_curr.y-1);
					neighbor[3] = ImageRef(addr_curr.x  , addr_curr.y+1);

					// for all neighbors do
					for (int i=0; i<4; ++i) {
						// get neighbor pixel address
						const ImageRef& addr_neighbor(neighbor[i]);
						// check if neighbor is inside image
						if (addr_neighbor.x>=0 && addr_neighbor.y>=0 && addr_neighbor.x<size.x && addr_neighbor.y<size.y) {
							// check if neighbor has not been added yet
							bool& done = done_map(addr_neighbor);
							if (!done) {
								// check if the neighbor is valid and similar to the current pixel
								// (belonging to the current segment)
								const Depth& depth_neighbor = depthMap(addr_neighbor);
								if (depth_neighbor>0 && IsDepthSimilar(depth_curr, depth_neighbor, fDepthDiffThreshold)) {
									// add neighbor coordinates to segment list
									seg_list[seg_list_count++] = addr_neighbor;
									// set neighbor pixel in done_map to "done"
									// (otherwise a pixel may be added 2 times to the list, as
									//  neighbor of one pixel and as neighbor of another pixel)
									done = true;
								}
							}
						}
					}
				}

				// set current pixel in seg_list to "done"
				++seg_list_curr;

				// set current pixel in done_map to "done"
				done_map(addr_curr) = true;
			} // end: while (seg_list_curr < seg_list_count)

			// if segment NOT large enough => invalidate pixels
			if (seg_list_count < speckle_size) {
				// for all pixels in current segment invalidate pixels
				for (unsigned i=0; i<seg_list_count; ++i) {
					depthMap(seg_list[i]) = 0;
					if (!normalMap.empty()) normalMap(seg_list[i]) = Normal::ZERO;
					if (!confMap.empty()) confMap(seg_list[i]) = 0;
				}
			}
		}
	}

	return true;
} // RemoveSmallSegments
/*----------------------------------------------------------------*/

// try to fill small gaps in the depth map
bool DepthMapsData::GapInterpolation(DepthData& depthData)
{
	const float fDepthDiffThreshold(OPTDENSE::fDepthDiffThreshold*2.5f);
	unsigned nIpolGapSize = OPTDENSE::nIpolGapSize;
	DepthMap& depthMap = depthData.depthMap;
	NormalMap& normalMap = depthData.normalMap;
	ConfidenceMap& confMap = depthData.confMap;
	ASSERT(!depthMap.empty());
	const ImageRef size(depthMap.size());

	// 1. Row-wise:
	// for each row do
	for (int v=0; v<size.y; ++v) {
		// init counter
		unsigned count = 0;

		// for each element of the row do
		for (int u=0; u<size.x; ++u) {
			// get depth of this location
			const Depth& depth = depthMap(v,u);

			// if depth not valid => count and skip it
			if (depth <= 0) {
				++count;
				continue;
			}
			if (count == 0)
				continue;

			// check if speckle is small enough
			// and value in range
			if (count <= nIpolGapSize && (unsigned)u > count) {
				// first value index for interpolation
				int u_curr(u-count);
				const int u_first(u_curr-1);
				// compute mean depth
				const Depth& depthFirst = depthMap(v,u_first);
				if (IsDepthSimilar(depthFirst, depth, fDepthDiffThreshold)) {
					#if 0
					// set all values with the average
					const Depth avg((depthFirst+depth)*0.5f);
					do {
						depthMap(v,u_curr) = avg;
					} while (++u_curr<u);						
					#else
					// interpolate values
					const Depth diff((depth-depthFirst)/(count+1));
					Depth d(depthFirst);
					const float c(confMap.empty() ? 0.f : MINF(confMap(v,u_first), confMap(v,u)));
					if (normalMap.empty()) {
						do {
							depthMap(v,u_curr) = (d+=diff);
							if (!confMap.empty()) confMap(v,u_curr) = c;
						} while (++u_curr<u);						
					} else {
						Point2f dir1, dir2;
						Normal2Dir(normalMap(v,u_first), dir1);
						Normal2Dir(normalMap(v,u), dir2);
						const Point2f dirDiff((dir2-dir1)/float(count+1));
						do {
							depthMap(v,u_curr) = (d+=diff);
							dir1 += dirDiff;
							Dir2Normal(dir1, normalMap(v,u_curr));
							if (!confMap.empty()) confMap(v,u_curr) = c;
						} while (++u_curr<u);						
					}
					#endif
				}
			}

			// reset counter
			count = 0;
		}
	}

	// 2. Column-wise:
	// for each column do
	for (int u=0; u<size.x; ++u) {

		// init counter
		unsigned count = 0;

		// for each element of the column do
		for (int v=0; v<size.y; ++v) {
			// get depth of this location
			const Depth& depth = depthMap(v,u);

			// if depth not valid => count and skip it
			if (depth <= 0) {
				++count;
				continue;
			}
			if (count == 0)
				continue;

			// check if gap is small enough
			// and value in range
			if (count <= nIpolGapSize && (unsigned)v > count) {
				// first value index for interpolation
				int v_curr(v-count);
				const int v_first(v_curr-1);
				// compute mean depth
				const Depth& depthFirst = depthMap(v_first,u);
				if (IsDepthSimilar(depthFirst, depth, fDepthDiffThreshold)) {
					#if 0
					// set all values with the average
					const Depth avg((depthFirst+depth)*0.5f);
					do {
						depthMap(v_curr,u) = avg;
					} while (++v_curr<v);						
					#else
					// interpolate values
					const Depth diff((depth-depthFirst)/(count+1));
					Depth d(depthFirst);
					const float c(confMap.empty() ? 0.f : MINF(confMap(v_first,u), confMap(v,u)));
					if (normalMap.empty()) {
						do {
							depthMap(v_curr,u) = (d+=diff);
							if (!confMap.empty()) confMap(v_curr,u) = c;
						} while (++v_curr<v);						
					} else {
						Point2f dir1, dir2;
						Normal2Dir(normalMap(v_first,u), dir1);
						Normal2Dir(normalMap(v,u), dir2);
						const Point2f dirDiff((dir2-dir1)/float(count+1));
						do {
							depthMap(v_curr,u) = (d+=diff);
							dir1 += dirDiff;
							Dir2Normal(dir1, normalMap(v_curr,u));
							if (!confMap.empty()) confMap(v_curr,u) = c;
						} while (++v_curr<v);						
					}
					#endif
				}
			}

			// reset counter
			count = 0;
		}
	}
	return true;
} // GapInterpolation
/*----------------------------------------------------------------*/


// filter depth-map, one pixel at a time, using confidence based fusion or neighbor pixels
bool DepthMapsData::FilterDepthMap(DepthData& depthDataRef, const IIndexArr& idxNeighbors, bool bAdjust)
{
	TD_TIMER_STARTD();

	// count valid neighbor depth-maps
	ASSERT(depthDataRef.IsValid() && !depthDataRef.IsEmpty());
	const IIndex N = idxNeighbors.GetSize();
	ASSERT(OPTDENSE::nMinViewsFilter > 0 && scene.nCalibratedImages > 1);
	const IIndex nMinViews(MINF(OPTDENSE::nMinViewsFilter,scene.nCalibratedImages-1));
	const IIndex nMinViewsAdjust(MINF(OPTDENSE::nMinViewsFilterAdjust,scene.nCalibratedImages-1));
	if (N < nMinViews || N < nMinViewsAdjust) {
		DEBUG("error: depth map %3u can not be filtered", depthDataRef.GetView().GetID());
		return false;
	}

	// project all neighbor depth-maps to this image
	const DepthData::ViewData& imageRef = depthDataRef.images.First();
	const Image8U::Size sizeRef(depthDataRef.depthMap.size());
	const Camera& cameraRef = imageRef.camera;
	DepthMapArr depthMaps(N);
	ConfidenceMapArr confMaps(N);
	FOREACH(n, depthMaps) {
		DepthMap& depthMap = depthMaps[n];
		depthMap.create(sizeRef);
		depthMap.memset(0);
		ConfidenceMap& confMap = confMaps[n];
		if (bAdjust) {
			confMap.create(sizeRef);
			confMap.memset(0);
		}
		const IIndex idxView = depthDataRef.neighbors[idxNeighbors[(IIndex)n]].ID;
		const DepthData& depthData = arrDepthData[idxView];
		const Camera& camera = depthData.images.First().camera;
		const Image8U::Size size(depthData.depthMap.size());
		for (int i=0; i<size.height; ++i) {
			for (int j=0; j<size.width; ++j) {
				const ImageRef x(j,i);
				const Depth depth(depthData.depthMap(x));
				if (depth == 0)
					continue;
				ASSERT(depth > 0);
				const Point3 X(camera.TransformPointI2W(Point3(x.x,x.y,depth)));
				const Point3 camX(cameraRef.TransformPointW2C(X));
				if (camX.z <= 0)
					continue;
				#if 0
				// set depth on the rounded image projection only
				const ImageRef xRef(ROUND2INT(cameraRef.TransformPointC2I(camX)));
				if (!depthMap.isInside(xRef))
					continue;
				Depth& depthRef(depthMap(xRef));
				if (depthRef != 0 && depthRef < camX.z)
					continue;
				depthRef = camX.z;
				if (bAdjust)
					confMap(xRef) = depthData.confMap(x);
				#else
				// set depth on the 4 pixels around the image projection
				const Point2 imgX(cameraRef.TransformPointC2I(camX));
				const ImageRef xRefs[4] = {
					ImageRef(FLOOR2INT(imgX.x), FLOOR2INT(imgX.y)),
					ImageRef(FLOOR2INT(imgX.x), CEIL2INT(imgX.y)),
					ImageRef(CEIL2INT(imgX.x), FLOOR2INT(imgX.y)),
					ImageRef(CEIL2INT(imgX.x), CEIL2INT(imgX.y))
				};
				for (int p=0; p<4; ++p) {
					const ImageRef& xRef = xRefs[p];
					if (!depthMap.isInside(xRef))
						continue;
					Depth& depthRef(depthMap(xRef));
					if (depthRef != 0 && depthRef < (Depth)camX.z)
						continue;
					depthRef = (Depth)camX.z;
					if (bAdjust)
						confMap(xRef) = depthData.confMap(x);
				}
				#endif
			}
		}
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (g_nVerbosityLevel > 3)
			ExportDepthMap(MAKE_PATH(String::FormatString("depthRender%04u.%04u.png", depthDataRef.GetView().GetID(), idxView)), depthMap);
		#endif
	}

	const float thDepthDiff(OPTDENSE::fDepthDiffThreshold*1.2f);
	DepthMap newDepthMap(sizeRef);
	ConfidenceMap newConfMap(sizeRef);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	size_t nProcessed(0), nDiscarded(0);
	#endif
	if (bAdjust) {
		// average similar depths, and decrease confidence if depths do not agree
		// (inspired by: "Real-Time Visibility-Based Fusion of Depth Maps", Merrell, 2007)
		for (int i=0; i<sizeRef.height; ++i) {
			for (int j=0; j<sizeRef.width; ++j) {
				const ImageRef xRef(j,i);
				const Depth depth(depthDataRef.depthMap(xRef));
				if (depth == 0) {
					newDepthMap(xRef) = 0;
					newConfMap(xRef) = 0;
					continue;
				}
				ASSERT(depth > 0);
				#if TD_VERBOSE != TD_VERBOSE_OFF
				++nProcessed;
				#endif
				// update best depth and confidence estimate with all estimates
				float posConf(depthDataRef.confMap(xRef)), negConf(0);
				Depth avgDepth(depth*posConf);
				unsigned nPosViews(0), nNegViews(0);
				unsigned n(N);
				do {
					const Depth d(depthMaps[--n](xRef));
					if (d == 0) {
						if (nPosViews + nNegViews + n < nMinViews)
							goto DiscardDepth;
						continue;
					}
					ASSERT(d > 0);
					if (IsDepthSimilar(depth, d, thDepthDiff)) {
						// average similar depths
						const float c(confMaps[n](xRef));
						avgDepth += d*c;
						posConf += c;
						++nPosViews;
					} else {
						// penalize confidence
						if (depth > d) {
							// occlusion
							negConf += confMaps[n](xRef);
						} else {
							// free-space violation
							const DepthData& depthData = arrDepthData[depthDataRef.neighbors[idxNeighbors[n]].ID];
							const Camera& camera = depthData.images.First().camera;
							const Point3 X(cameraRef.TransformPointI2W(Point3(xRef.x,xRef.y,depth)));
							const ImageRef x(ROUND2INT(camera.TransformPointW2I(X)));
							if (depthData.confMap.isInside(x)) {
								const float c(depthData.confMap(x));
								negConf += (c > 0 ? c : confMaps[n](xRef));
							} else
								negConf += confMaps[n](xRef);
						}
						++nNegViews;
					}
				} while (n);
				ASSERT(nPosViews+nNegViews >= nMinViews);
				// if enough good views and positive confidence...
				if (nPosViews >= nMinViewsAdjust && posConf > negConf && ISINSIDE(avgDepth/=posConf, depthDataRef.dMin, depthDataRef.dMax)) {
					// consider this pixel an inlier
					newDepthMap(xRef) = avgDepth;
					newConfMap(xRef) = posConf - negConf;
				} else {
					// consider this pixel an outlier
					DiscardDepth:
					newDepthMap(xRef) = 0;
					newConfMap(xRef) = 0;
					#if TD_VERBOSE != TD_VERBOSE_OFF
					++nDiscarded;
					#endif
				}
			}
		}
	} else {
		// remove depth if it does not agree with enough neighbors
		const float thDepthDiffStrict(OPTDENSE::fDepthDiffThreshold*0.8f);
		const unsigned nMinGoodViewsProc(75), nMinGoodViewsDeltaProc(65);
		const unsigned nDeltas(4);
		const unsigned nMinViewsDelta(nMinViews*(nDeltas-2));
		const ImageRef xDs[nDeltas] = { ImageRef(-1,0), ImageRef(1,0), ImageRef(0,-1), ImageRef(0,1) };
		for (int i=0; i<sizeRef.height; ++i) {
			for (int j=0; j<sizeRef.width; ++j) {
				const ImageRef xRef(j,i);
				const Depth depth(depthDataRef.depthMap(xRef));
				if (depth == 0) {
					newDepthMap(xRef) = 0;
					newConfMap(xRef) = 0;
					continue;
				}
				ASSERT(depth > 0);
				#if TD_VERBOSE != TD_VERBOSE_OFF
				++nProcessed;
				#endif
				// check if very similar with the neighbors projected to this pixel
				{
					unsigned nGoodViews(0);
					unsigned nViews(0);
					unsigned n(N);
					do {
						const Depth d(depthMaps[--n](xRef));
						if (d > 0) {
							// valid view
							++nViews;
							if (IsDepthSimilar(depth, d, thDepthDiffStrict)) {
								// agrees with this neighbor
								++nGoodViews;
							}
						}
					} while (n);
					if (nGoodViews < nMinViews || nGoodViews < nViews*nMinGoodViewsProc/100) {
						#if TD_VERBOSE != TD_VERBOSE_OFF
						++nDiscarded;
						#endif
						newDepthMap(xRef) = 0;
						newConfMap(xRef) = 0;
						continue;
					}
				}
				// check if similar with the neighbors projected around this pixel
				{
					unsigned nGoodViews(0);
					unsigned nViews(0);
					for (unsigned d=0; d<nDeltas; ++d) {
						const ImageRef xDRef(xRef+xDs[d]);
						unsigned n(N);
						do {
							const Depth d(depthMaps[--n](xDRef));
							if (d > 0) {
								// valid view
								++nViews;
								if (IsDepthSimilar(depth, d, thDepthDiff)) {
									// agrees with this neighbor
									++nGoodViews;
								}
							}
						} while (n);
					}
					if (nGoodViews < nMinViewsDelta || nGoodViews < nViews*nMinGoodViewsDeltaProc/100) {
						#if TD_VERBOSE != TD_VERBOSE_OFF
						++nDiscarded;
						#endif
						newDepthMap(xRef) = 0;
						newConfMap(xRef) = 0;
						continue;
					}
				}
				// enough good views, keep it
				newDepthMap(xRef) = depth;
				newConfMap(xRef) = depthDataRef.confMap(xRef);
			}
		}
	}
	if (!SaveDepthMap(ComposeDepthFilePath(imageRef.GetID(), "filtered.dmap"), newDepthMap) ||
		!SaveConfidenceMap(ComposeDepthFilePath(imageRef.GetID(), "filtered.cmap"), newConfMap))
		return false;

	DEBUG("Depth map %3u filtered using %u other images: %u/%u depths discarded (%s)",
		imageRef.GetID(), N, nDiscarded, nProcessed, TD_TIMER_GET_FMT().c_str());
	return true;
} // FilterDepthMap
/*----------------------------------------------------------------*/


// estimate normal-maps based on the depth-maps;
// loads and saves the depth-data from/to disk
void DepthMapsData::EstimateNormalMaps()
{
	#ifdef DENSE_USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for shared(bAbort)
	for (int64_t i=0; i<(int64_t)scene.images.size(); ++i) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
		const IIndex idxImage((IIndex)i);
	#else
	FOREACH(idxImage, scene.images) {
	#endif
		DepthData& depthData = arrDepthData[idxImage];
		if (!depthData.IsValid())
			continue;
		const String fileName(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
		const bool bEmpty(depthData.IsEmpty());
		if (bEmpty && !depthData.Load(fileName)) {
			#ifdef DENSE_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return;
			#endif
		}
		ASSERT(!depthData.IsEmpty());
		ASSERT(!scene.images[idxImage].neighbors.empty());
		if (depthData.normalMap.empty()) {
			EstimateNormalMap(depthData.images.front().camera.K, depthData.depthMap, depthData.normalMap);
			if (!depthData.Save(fileName)) {
				#ifdef DENSE_USE_OPENMP
				bAbort = true;
				#pragma omp flush (bAbort)
				continue;
				#else
				return;
				#endif
			}
		}
		if (bEmpty)
			depthData.Release();
	}
	#ifdef DENSE_USE_OPENMP
	if (bAbort)
		return;
	#endif
} // EstimateNormalMaps


// fuse all depth-maps by simply projecting them in a 3D point cloud
// in the world coordinate space
void DepthMapsData::MergeDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal)
{
	TD_TIMER_STARTD();

	// estimate total number of 3D points that will be generated
	size_t nPointsEstimate(0);
	for (const DepthData& depthData: arrDepthData)
		if (depthData.IsValid())
			nPointsEstimate += (size_t)depthData.depthMap.size().area()*7/10;

	// fuse all depth-maps
	size_t nDepthMaps(0), nDepths(0);
	pointcloud.points.reserve(nPointsEstimate);
	pointcloud.pointViews.reserve(nPointsEstimate);
	if (bEstimateColor)
		pointcloud.colors.reserve(nPointsEstimate);
	if (bEstimateNormal)
		pointcloud.normals.reserve(nPointsEstimate);
	Util::Progress progress(_T("Merged depth-maps"), arrDepthData.size());
	GET_LOGCONSOLE().Pause();
	FOREACH(idxImage, arrDepthData) {
		TD_TIMER_STARTD();
		DepthData& depthData = arrDepthData[idxImage];
		ASSERT(depthData.GetView().GetLocalID(scene.images) == idxImage);
		if (!depthData.IsValid())
			continue;
		if (depthData.IncRef(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap")) == 0)
			return;
		ASSERT(!depthData.IsEmpty());
		if (bEstimateNormal && depthData.normalMap.empty())
			EstimateNormalMaps();
		const DepthData::ViewData& image = depthData.GetView();
		const size_t nNumPointsPrev(pointcloud.points.size());
		for (int i=0; i<depthData.depthMap.rows; ++i) {
			for (int j=0; j<depthData.depthMap.cols; ++j) {
				// ignore invalid depth
				const ImageRef x(j,i);
				const Depth depth(depthData.depthMap(x));
				if (depth == 0)
					continue;
				ASSERT(ISINSIDE(depth, depthData.dMin, depthData.dMax));
				// create the corresponding 3D point
				pointcloud.points.emplace_back(image.camera.TransformPointI2W(Point3(Cast<float>(x),depth)));
				pointcloud.pointViews.emplace_back().push_back(idxImage);
				if (bEstimateColor)
					pointcloud.colors.emplace_back(image.pImageData->image(x));
				if (bEstimateNormal)
					depthData.GetNormal(x, pointcloud.normals.emplace_back());
				++nDepths;
			}
		}
		depthData.DecRef();
		++nDepthMaps;
		ASSERT(pointcloud.points.size() == pointcloud.pointViews.size());
		DEBUG_ULTIMATE("Depths map for reference image %3u merged using %u depths maps: %u new points (%s)",
			idxImage, depthData.images.size()-1, pointcloud.points.size()-nNumPointsPrev, TD_TIMER_GET_FMT().c_str());
		progress.display(idxImage+1);
	}
	GET_LOGCONSOLE().Play();
	progress.close();

	DEBUG_EXTRA("Depth-maps merged: %u depth-maps, %u depths, %u points (%d%%%%) (%s)",
		nDepthMaps, nDepths, pointcloud.points.size(), ROUND2INT(100.f*pointcloud.points.size()/nDepths), TD_TIMER_GET_FMT().c_str());
} // MergeDepthMaps
/*----------------------------------------------------------------*/


// compute available memory to be used for depth-data caching
//  - numDMapsReserveFusion: maximum number of depth-maps for which to reserve memory for fusion
size_t GetAvailableMemory(const DepthDataArr& arrDepthData, const BoolArr& fusedDMaps, IIndex numDMapsReserveFusion, size_t currentCacheMemory = 0)
{
	size_t resolution(0);
	IIndex numDMaps(0);
	FOREACH(idxImage, arrDepthData) {
		const DepthData& depthData = arrDepthData[idxImage];
		if (!depthData.IsValid())
			continue;
		if (fusedDMaps[idxImage])
			continue;
		resolution += depthData.size.area();
		if (++numDMaps >= numDMapsReserveFusion)
			break;
	}
	if (numDMaps == 0)
		return 0;
	const Util::MemoryInfo memInfo(Util::GetMemoryInfo());
	const size_t neededPointCloudMemory(ROUND2INT<size_t>(resolution * (1/*depth*/+1/*color*/+3/*normal*/+1/*confidence*/) * 4/*bytes*/ * 0.35/*unique pixels per depth-map*/));
	const size_t freeMemory(currentCacheMemory + memInfo.freePhysical);
	const size_t safetyMemory(MAXF(ROUND2INT<size_t>(memInfo.totalPhysical * 0.08), size_t(1*1024*1024*1024ull)/*1GB*/));
	const size_t neededMemory(neededPointCloudMemory + safetyMemory);
	const size_t minDMapsMemory(resolution / numDMaps * 8/*min dmaps in memory*/ * (1/*depth*/ + 3/*normal*/ + 1/*confidence*/) * 4/*bytes*/);
	if (freeMemory < neededMemory) {
		DEBUG("warning: not enough memory to cache depth-maps (%luMB needed, %luMB available)", neededMemory/1024/1024, freeMemory/1024/1024);
		return MINF(currentCacheMemory, minDMapsMemory);
	}
	return freeMemory - neededMemory;
} // GetAvailableMemory

// finds the best depth-map to fuse next that maximizes the number of neighbors already in cache
std::tuple<unsigned, unsigned, unsigned> FetchBestNextDMapIndex(const DepthDataArr& arrDepthData, const DMapCache& cacheDMaps, const BoolArr& fusedDMaps) {
	const IIndexArr cachedImages = cacheDMaps.GetCachedImageIndices(true);
	IIndex bestImageIdx = NO_ID;
	unsigned bestImageScore = 0, bestImageSize = std::numeric_limits<unsigned>::max();
	FOREACH(idxImage, arrDepthData) {
		const DepthData& depthData = arrDepthData[idxImage];
		if (!depthData.IsValid())
			continue;
		if (fusedDMaps[idxImage])
			continue;
		ASSERT(!depthData.neighbors.empty());
		IIndexArr cachedNeighbors;
		if (!cachedImages.empty()) {
			IIndexArr neighbors(0, depthData.neighbors.size());
			for (ViewScore& neighbor: depthData.neighbors)
				neighbors.push_back(neighbor.ID);
			neighbors.Sort();
			std::set_intersection(neighbors.begin(), neighbors.end(),
				cachedImages.begin(), cachedImages.end(),
				std::back_inserter(cachedNeighbors));
		}
		if (bestImageScore < cachedNeighbors.size() ||
			(bestImageScore == cachedNeighbors.size() && bestImageSize > depthData.neighbors.size())) {
			bestImageScore = cachedNeighbors.size();
			bestImageSize = depthData.neighbors.size();
			bestImageIdx = idxImage;
		}
	}
	return std::make_tuple(bestImageIdx, bestImageScore, static_cast<unsigned>(cachedImages.size()));
} // FetchBestNextDMapIndex

// fuse all valid depth-maps in the same 3D point cloud;
// join points very likely to represent the same 3D point and
// filter out points blocking the view
void DepthMapsData::FuseDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal)
{
	TD_TIMER_STARTD();

	struct Proj {
		union {
			uint32_t idxPixel;
			struct {
				uint16_t x, y; // image pixel coordinates
			};
		};
		inline Proj() {}
		inline Proj(uint32_t _idxPixel) : idxPixel(_idxPixel) {}
		inline Proj(const ImageRef& ir) : x(ir.x), y(ir.y) {}
		inline ImageRef GetCoord() const { return ImageRef(x,y); }
	};
	typedef SEACAVE::cList<Proj,const Proj&,0,4,uint32_t> ProjArr;
	typedef SEACAVE::cList<ProjArr,const ProjArr&,1,65536> ProjsArr;

	// fuse all depth-maps, processing the best connected images first
	const unsigned nMinViewsFuse(MINF(OPTDENSE::nMinViewsFuse, arrDepthData.size()));
	const float normalError(COS(FD2R(OPTDENSE::fNormalDiffThreshold)));
	const IIndex numDMapsReserveFusion(10);
	CLISTDEF0(Depth*) invalidDepths(0, 32);
	size_t nDepths(0);
	typedef TImage<cuint32_t> DepthIndex;
	typedef cList<DepthIndex> DepthIndexArr;
	DepthIndexArr arrDepthIdx(arrDepthData.size());
	const size_t nPointsEstimate(arrDepthData.size() * 9000); //TODO: better estimate number of points
	ProjsArr projs(0, nPointsEstimate);
	pointcloud.points.reserve(nPointsEstimate);
	pointcloud.pointViews.reserve(nPointsEstimate);
	pointcloud.pointWeights.reserve(nPointsEstimate);
	unsigned depthDataLoadFlags(HeaderDepthDataRaw::HAS_DEPTH | HeaderDepthDataRaw::HAS_CONF);
	if (bEstimateColor)
		pointcloud.colors.reserve(nPointsEstimate);
	if (bEstimateNormal) {
		pointcloud.normals.reserve(nPointsEstimate);
		depthDataLoadFlags |= HeaderDepthDataRaw::HAS_NORMAL;
	}
	Util::Progress progress(_T("Fused depth-maps"), arrDepthData.size());
	GET_LOGCONSOLE().Pause();
	BoolArr fusedDMaps(arrDepthData.size());
	fusedDMaps.Memset(0);
	DMapCache cacheDMaps(arrDepthData, depthDataLoadFlags, GetAvailableMemory(arrDepthData, fusedDMaps, numDMapsReserveFusion));
	unsigned totalNumImageNeighborsInCache = 0, totalNumImagesInCache = 0;
	IIndex numDMapsFused = 0;
	for (; numDMapsFused < arrDepthData.size(); ++numDMapsFused) {
		TD_TIMER_STARTD();
		// find the best depth-map to fuse next as the one with the most neighbors already in cache
		const auto [idxImage, numImageNeighborsInCache, numImagesInCache] = FetchBestNextDMapIndex(arrDepthData, cacheDMaps, fusedDMaps);
		if (idxImage == NO_ID)
			break; // no more depth-maps to fuse (only invalid depth-maps left)
		totalNumImageNeighborsInCache += numImageNeighborsInCache;
		totalNumImagesInCache += numImagesInCache;
		// fuse depth-map
		cacheDMaps.UseImage(idxImage);
		cacheDMaps.SkipMemoryCheckIdxImage(idxImage);
		const DepthData& depthData(arrDepthData[idxImage]);
		ASSERT(depthData.GetView().GetLocalID(scene.images) == idxImage);
		ASSERT(!depthData.IsEmpty());
		if (bEstimateNormal && depthData.normalMap.empty())
			EstimateNormalMaps();
		ASSERT(!depthData.images.empty() && !depthData.neighbors.empty());
		IIndex numNeighbors(0);
		#ifdef DENSE_USE_OPENMP
		#pragma omp parallel for
		for (int64_t i=0; i<(int64_t)depthData.neighbors.size(); ++i) {
			const ViewScore& neighbor = depthData.neighbors[(IIndex)i];
		#else
		for (const ViewScore& neighbor: depthData.neighbors) {
		#endif
			const DepthData& depthDataB(arrDepthData[neighbor.ID]);
			if (!depthDataB.IsValid())
				continue;
			cacheDMaps.UseImage(neighbor.ID);
			if (depthDataB.IsEmpty())
				continue;
			if (++numNeighbors >= OPTDENSE::nMaxViewsFuse)
				#ifdef DENSE_USE_OPENMP
				continue;
				#else
				break;
				#endif
			DepthIndex& depthIdxs = arrDepthIdx[neighbor.ID];
			if (!depthIdxs.empty())
				continue;
			depthIdxs.create(depthDataB.depthMap.size());
			depthIdxs.memset((uint8_t)NO_ID);
		}
		ASSERT(!depthData.IsEmpty());
		const Image& imageData = *depthData.images.front().pImageData;
		ASSERT(&imageData-scene.images.data() == idxImage);
		ASSERT(depthData.depthMap.size() == depthData.size && imageData.GetSize() == depthData.size);
		DepthIndex& depthIdxs = arrDepthIdx[idxImage];
		if (depthIdxs.empty()) {
			depthIdxs.create(depthData.size);
			depthIdxs.memset((uint8_t)NO_ID);
		}
		const size_t nNumPointsPrev(pointcloud.points.size());
		for (int i=0; i<depthData.size.height; ++i) {
			for (int j=0; j<depthData.size.width; ++j) {
				const ImageRef x(j,i);
				const Depth depth(depthData.depthMap(x));
				if (depth == 0)
					continue;
				++nDepths;
				ASSERT(ISINSIDE(depth, depthData.dMin, depthData.dMax));
				uint32_t& idxPoint = depthIdxs(x);
				if (idxPoint != NO_ID)
					continue;
				// create the corresponding 3D point
				idxPoint = (uint32_t)pointcloud.points.size();
				PointCloud::Point& point = pointcloud.points.emplace_back();
				point = imageData.camera.TransformPointI2W(Point3(Point2f(x),depth));
				PointCloud::ViewArr& views = pointcloud.pointViews.emplace_back();
				views.emplace_back(idxImage);
				PointCloud::WeightArr& weights = pointcloud.pointWeights.emplace_back();
				REAL confidence(weights.emplace_back(Conf2Weight(depthData.confMap.empty() ? 1.f : depthData.confMap(x),depth)));
				ProjArr& pointProjs = projs.emplace_back();
				pointProjs.emplace_back(Proj(x));
				const PointCloud::Normal normal(!depthData.normalMap.empty() ? Cast<Normal::Type>(imageData.camera.R.t() * Cast<REAL>(depthData.normalMap(x))) : Normal(0, 0, -1));
				ASSERT(ISEQUAL(norm(normal), 1.f));
				// check the projection in the neighbor depth-maps
				Point3 X(point*confidence);
				Pixel32F C(Cast<float>(imageData.image(x))*confidence);
				PointCloud::Normal N(normal*confidence);
				invalidDepths.clear();
				for (const ViewScore& neighbor: depthData.neighbors) {
					const IIndex idxImageB(neighbor.ID);
					DepthData& depthDataB = arrDepthData[idxImageB];
					if (depthDataB.IsEmpty())
						continue;
					const Image& imageDataB = scene.images[idxImageB];
					const Point3f pt(imageDataB.camera.ProjectPointP3(point));
					if (pt.z <= 0)
						continue;
					const ImageRef xB(ROUND2INT(pt.x/pt.z), ROUND2INT(pt.y/pt.z));
					DepthMap& depthMapB = depthDataB.depthMap;
					if (!depthMapB.isInside(xB))
						continue;
					Depth& depthB = depthMapB(xB);
					if (depthB == 0)
						continue;
					uint32_t& idxPointB = arrDepthIdx[idxImageB](xB);
					if (idxPointB != NO_ID)
						continue;
					if (IsDepthSimilar(pt.z, depthB, OPTDENSE::fDepthDiffThreshold)) {
						// check if normals agree
						const PointCloud::Normal normalB(!depthData.normalMap.empty() ? Cast<Normal::Type>(imageDataB.camera.R.t() * Cast<REAL>(depthDataB.normalMap(xB))) : Normal(0, 0, -1));
						ASSERT(ISEQUAL(norm(normalB), 1.f));
						if (normal.dot(normalB) > normalError) {
							// add view to the 3D point
							ASSERT(views.FindFirst(idxImageB) == PointCloud::ViewArr::NO_INDEX);
							const float confidenceB(Conf2Weight(depthDataB.confMap.empty() ? 1.f : depthDataB.confMap(xB),depthB));
							const IIndex idx(views.InsertSort(idxImageB));
							weights.InsertAt(idx, confidenceB);
							pointProjs.InsertAt(idx, Proj(xB));
							idxPointB = idxPoint;
							X += imageDataB.camera.TransformPointI2W(Point3(Point2f(xB),depthB))*REAL(confidenceB);
							if (bEstimateColor)
								C += Cast<float>(imageDataB.image(xB))*confidenceB;
							if (bEstimateNormal)
								N += normalB*confidenceB;
							confidence += confidenceB;
							continue;
						}
					}
					if (pt.z < depthB) {
						// discard depth
						invalidDepths.emplace_back(&depthB);
					}
				}
				if (views.size() < nMinViewsFuse) {
					// remove point
					FOREACH(v, views) {
						const IIndex idxImageB(views[v]);
						const ImageRef x(pointProjs[v].GetCoord());
						ASSERT(arrDepthIdx[idxImageB].isInside(x) && arrDepthIdx[idxImageB](x).idx != NO_ID);
						arrDepthIdx[idxImageB](x).idx = NO_ID;
					}
					projs.pop_back();
					pointcloud.pointWeights.pop_back();
					pointcloud.pointViews.pop_back();
					pointcloud.points.pop_back();
				} else {
					// this point is valid, store it
					const REAL nrm(REAL(1)/confidence);
					point = X*nrm;
					ASSERT(ISFINITE(point));
					if (bEstimateColor)
						pointcloud.colors.emplace_back((C*(float)nrm).cast<uint8_t>());
					if (bEstimateNormal)
						pointcloud.normals.emplace_back(normalized(N*(float)nrm));
					// invalidate all neighbor depths that do not agree with it
					for (Depth* pDepth: invalidDepths)
						*pDepth = 0;
				}
			}
		}
		fusedDMaps[idxImage] = true;
		ASSERT(pointcloud.points.size() == pointcloud.pointViews.size() && pointcloud.points.size() == pointcloud.pointWeights.size() && pointcloud.points.size() == projs.size());
		DEBUG_ULTIMATE("Depths map for reference image %3u fused using %u depths maps: %u new points, %u/%u cached images (%s)",
			idxImage, depthData.images.size()-1, pointcloud.points.size()-nNumPointsPrev, numImageNeighborsInCache, numImagesInCache, TD_TIMER_GET_FMT().c_str());
		progress.display(numDMapsFused);
		// ensure enough memory is available for the next depth-maps chunk
		cacheDMaps.SkipMemoryCheckIdxImage();
		if (numDMapsFused % numDMapsReserveFusion == 0)
			cacheDMaps.SetMaxMemory(GetAvailableMemory(arrDepthData, fusedDMaps, numDMapsReserveFusion, cacheDMaps.GetUsedMemory()));
	}
	GET_LOGCONSOLE().Play();
	progress.close();
	arrDepthIdx.Release();
	cacheDMaps.ClearCache();	

	DEBUG_EXTRA("Depth-maps fused and filtered: %u depth-maps, %u depths, %u points (%d%%%%), %.2f hits in %.2f cached (%s)",
		numDMapsFused, nDepths, pointcloud.points.size(), ROUND2INT((100.f*pointcloud.points.size())/nDepths),
		static_cast<double>(totalNumImageNeighborsInCache) / numDMapsFused,
		static_cast<double>(totalNumImagesInCache) / numDMapsFused, TD_TIMER_GET_FMT().c_str());

	if (bEstimateNormal && !pointcloud.points.empty() && pointcloud.normals.empty()) {
		// estimate normal also if requested (quite expensive if normal-maps not available)
		TD_TIMER_STARTD();
		pointcloud.normals.resize(pointcloud.points.size());
		const int64_t nPoints((int64_t)pointcloud.points.size());
		#ifdef DENSE_USE_OPENMP
		#pragma omp parallel for
		#endif
		for (int64_t i=0; i<nPoints; ++i) {
			PointCloud::WeightArr& weights = pointcloud.pointWeights[i];
			ASSERT(!weights.empty());
			IIndex idxView(0);
			float bestWeight = weights.front();
			for (IIndex idx=1; idx<weights.size(); ++idx) {
				const PointCloud::Weight& weight = weights[idx];
				if (bestWeight < weight) {
					bestWeight = weight;
					idxView = idx;
				}
			}
			const DepthData& depthData(arrDepthData[pointcloud.pointViews[i][idxView]]);
			ASSERT(depthData.IsValid() && !depthData.IsEmpty());
			depthData.GetNormal(projs[i][idxView].GetCoord(), pointcloud.normals[i]);
		}
		DEBUG_EXTRA("Normals estimated for the dense point-cloud: %u normals (%s)", pointcloud.GetSize(), TD_TIMER_GET_FMT().c_str());
	}
} // FuseDepthMaps


// fuse all valid depth-maps in the same 3D point cloud;
// join points very likely to represent the same 3D point and
// filter out points blocking the view
void DepthMapsData::DenseFuseDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool _bEstimateNormal)
{
	TD_TIMER_STARTD();

	typedef SEACAVE::BitMatrix UseMask;
	typedef CLISTDEFIDX(UseMask,IIndex) UseMaskArr;

	// fuse all depth-maps, processing the best connected images first
	const unsigned nMinViewsFuse(MINF(OPTDENSE::nMinViewsFuse, arrDepthData.size()));
	const float normalError(COS(FD2R(OPTDENSE::fNormalDiffThreshold)));
	const float minConfidence(1.f - OPTDENSE::fNCCThresholdKeep);
	const float maxReprojErrorSq(SQUARE(OPTDENSE::fDepthReprojectionErrorThreshold));
	const IIndex numDMapsReserveFusion(10);
	const bool bEstimateNormal(true); // always estimate normals as they are needed for the fusion
	size_t nDepths(0);
	UseMaskArr arrUseMask(arrDepthData.size());
	const size_t nPointsEstimate(arrDepthData.size() * 9000); //TODO: better estimate number of points
	pointcloud.points.reserve(nPointsEstimate);
	pointcloud.pointViews.reserve(nPointsEstimate);
	pointcloud.pointWeights.reserve(nPointsEstimate);
	unsigned depthDataLoadFlags(HeaderDepthDataRaw::HAS_DEPTH | HeaderDepthDataRaw::HAS_CONF);
	if (bEstimateColor)
		pointcloud.colors.reserve(nPointsEstimate);
	if (bEstimateNormal) {
		pointcloud.normals.reserve(nPointsEstimate);
		depthDataLoadFlags |= HeaderDepthDataRaw::HAS_NORMAL;
	}
	Util::Progress progress(_T("Dense fused depth-maps"), arrDepthData.size());
	GET_LOGCONSOLE().Pause();
	BoolArr fusedDMaps(arrDepthData.size());
	fusedDMaps.Memset(0);
	DMapCache cacheDMaps(arrDepthData, depthDataLoadFlags, GetAvailableMemory(arrDepthData, fusedDMaps, numDMapsReserveFusion));
	unsigned totalNumImageNeighborsInCache = 0, totalNumImagesInCache = 0;
	BoolArr neighbors(arrDepthData.size());
	PointCloud::Point refPoint;
	PointCloud::Normal refNormal;
	CLISTDEF0IDX(float, unsigned) fusedPoints[3];
	PointCloud::ViewArr fusedViews;
	FloatArr fusedWeights;
	Point3d fusedNormal;
	Pixel32F fusedColor;
	const auto FusePoint = [&](IIndex ID, const ImageRef& x, unsigned fuseDepth) -> void {
		const auto lambda = [&](IIndex ID, const ImageRef& x, unsigned fuseDepth, const auto& FusePointImpl) -> void {
			const DepthData& depthData = arrDepthData[ID];
			if (!Image8U::isInside(x, depthData.size))
				return;
			// ignore pixel if not estimated
			ASSERT(depthData.depthMap.size() == depthData.size);
			const Depth depth = depthData.depthMap(x);
			if (depth <= Depth(0))
				return;
			ASSERT(ISINSIDE(depth, depthData.dMin * 0.95f, depthData.dMax * 1.05f));
			// ignore pixel if already fused
			UseMask& useMask = arrUseMask[ID];
			if (useMask(x))
				return;
			// ignore pixel if not confident
			const float conf(depthData.confMap.empty() ? 1.f : depthData.confMap(x));
			if (conf < minConfidence)
				return;
			const DepthData::ViewData& image = depthData.GetView();
			// if the fusion depth is greater than zero, the initial reference pixel
			// has already been added and we need to check for consistency
			PointCloud::Normal normal;
			if (fuseDepth > 0) {
				// project reference point into current view
				const Point3f pt(image.camera.ProjectPointP3(refPoint));
				// check if depth agrees with current depth
				ASSERT(pt.z > Depth(0) || !IsDepthSimilar(depth, pt.z, OPTDENSE::fDepthDiffThreshold));
				if (!IsDepthSimilar(depth, pt.z, OPTDENSE::fDepthDiffThreshold))
					return;
				// check reprojection error of the reference point in the current view
				const Point2f diff(pt.x / pt.z - float(x.x), pt.y / pt.z - float(x.y));
				if (normSq(diff) > maxReprojErrorSq)
					return;
				// check if normals agree
				normal = image.camera.R.t() * Cast<REAL>(depthData.normalMap(x));
				ASSERT(ISEQUAL(norm(normal), 1.f));
				if (refNormal.dot(normal) < normalError)
					return;
			} else {
				normal = image.camera.R.t() * Cast<REAL>(depthData.normalMap(x));
				ASSERT(ISEQUAL(norm(normal), 1.f));
			}
			// set the current pixel as visited
			useMask.set(x);
			// compute 3D location of the current depth
			const PointCloud::Point X(image.camera.TransformPointI2W(Point3(REAL(x.x), REAL(x.y), REAL(depth))));
			// accumulate statistics for fused point
			{
				fusedPoints[0].push_back(X(0));
				fusedPoints[1].push_back(X(1));
				fusedPoints[2].push_back(X(2));
				const float weight(Conf2Weight(conf, depth));
				const auto it(fusedViews.InsertSortUnique(ID));
				if (it.second)
					fusedWeights[it.first] += weight;
				else
					fusedWeights.InsertAt(it.first, weight);
				if (bEstimateNormal)
					fusedNormal += Cast<double>(normal);
				if (bEstimateColor)
					fusedColor += Cast<float>(image.pImageData->image(x));
			}
			// remember the first pixel as the reference.
			if (fuseDepth == 0) {
				refPoint = X;
				refNormal = normal;
			}
			// do not traverse the graph infinitely in one branch and
			// limit the maximum number of pixels fused in one point
			// to avoid stack overflow
			if (++fuseDepth >= OPTDENSE::nMaxFuseDepth || fusedPoints[0].size() >= OPTDENSE::nMaxPointsFuse)
				return;
			// traverse the neighbors graph by projecting the point into other views
			for (const ViewScore& neighbor : image.pImageData->neighbors) {
				const IIndex nextID(neighbor.ID);
				ASSERT(nextID != ID);
				if (!neighbors[nextID])
					continue;
				const DepthData& nextDepthData = arrDepthData[nextID];
				const ImageRef nextx(ROUND2INT(nextDepthData.GetCamera().ProjectPointP(X)));
				FusePointImpl(nextID, nextx, fuseDepth, FusePointImpl);
			}
		};
		lambda(ID, x, fuseDepth, lambda);
	};
	// loop over each depth-map
	IIndex numDMapsFused = 0;
	for (; numDMapsFused < arrDepthData.size(); ++numDMapsFused) {
		TD_TIMER_STARTD();
		// find the best depth-map to fuse next as the one with the most neighbors already in cache
		const auto [idxImage, numImageNeighborsInCache, numImagesInCache] = FetchBestNextDMapIndex(arrDepthData, cacheDMaps, fusedDMaps);
		if (idxImage == NO_ID)
			break; // no more depth-maps to fuse (only invalid depth-maps left)
		totalNumImageNeighborsInCache += numImageNeighborsInCache;
		totalNumImagesInCache += numImagesInCache;
		// fuse depth-map
		cacheDMaps.UseImage(idxImage);
		cacheDMaps.SkipMemoryCheckIdxImage(idxImage);
		const DepthData& depthData(arrDepthData[idxImage]);
		ASSERT(depthData.GetView().GetLocalID(scene.images) == idxImage);
		ASSERT(!depthData.IsEmpty());
		if (bEstimateNormal && depthData.normalMap.empty())
			EstimateNormalMaps();
		// make sure all neighbors are cached
		neighbors.Memset(0);
		neighbors[idxImage] = true;
		IIndex numNeighbors(0);
		ASSERT(!depthData.images.empty() && !depthData.neighbors.empty());
		#ifdef DENSE_USE_OPENMP
		#pragma omp parallel for
		for (int64_t i=0; i<(int64_t)depthData.neighbors.size(); ++i) {
			const ViewScore& neighbor = depthData.neighbors[(IIndex)i];
		#else
		for (const ViewScore& neighbor: depthData.neighbors) {
		#endif
			const DepthData& depthDataB(arrDepthData[neighbor.ID]);
			if (!depthDataB.IsValid())
				continue;
			cacheDMaps.UseImage(neighbor.ID);
			if (depthDataB.IsEmpty())
				continue;
			neighbors[neighbor.ID] = true;
			if (++numNeighbors >= OPTDENSE::nMaxViewsFuse)
				#ifdef DENSE_USE_OPENMP
				continue;
				#else
				break;
				#endif
			UseMask& useMask = arrUseMask[neighbor.ID];
			if (!useMask.empty())
				continue;
			useMask.create(depthDataB.depthMap.size());
			useMask.memset(0);
		}
		ASSERT(!depthData.IsEmpty());
		const Image& imageData = *depthData.images.front().pImageData;
		ASSERT(&imageData-scene.images.data() == idxImage);
		ASSERT(depthData.depthMap.size() == depthData.size && imageData.GetSize() == depthData.size);
		UseMask& useMask = arrUseMask[idxImage];
		if (useMask.empty()) {
			useMask.create(depthData.size);
			useMask.memset(0);
		}
		// try to fuse each depth estimate
		const size_t nNumPointsPrev(pointcloud.points.size());
		for (int i=0; i<depthData.size.height; ++i) {
			for (int j=0; j<depthData.size.width; ++j) {
				FusePoint(idxImage, ImageRef(j,i), 0);
				if (fusedPoints[0].size() >= OPTDENSE::nMinPixelsFuse && fusedViews.size() >= nMinViewsFuse) {
					// create the corresponding 3D point
					pointcloud.points.emplace_back(
						fusedPoints[0].GetMedian(),
						fusedPoints[1].GetMedian(),
						fusedPoints[2].GetMedian()
					);
					ASSERT(fusedViews.size() == fusedWeights.size());
					PointCloud::WeightArr& weights = pointcloud.pointWeights.AddEmpty();
					for (float weight: fusedWeights)
						weights.push_back(weight);
					pointcloud.pointViews.emplace_back(fusedViews);
					if (bEstimateNormal)
						pointcloud.normals.emplace_back(normalized(fusedNormal));
					if (bEstimateColor)
						pointcloud.colors.emplace_back((fusedColor/(float)fusedPoints[0].size()).cast<uint8_t>());
				}
				if (!fusedViews.empty()) {
					nDepths += fusedViews.size();
					fusedPoints[0].clear();
					fusedPoints[1].clear();
					fusedPoints[2].clear();
					fusedViews.clear();
					fusedWeights.clear();
					fusedNormal = Point3d::ZERO;
					fusedColor = Pixel32F::BLACK;
				}
			}
		}
		fusedDMaps[idxImage] = true;
		ASSERT(pointcloud.points.size() == pointcloud.pointViews.size() && pointcloud.points.size() == pointcloud.pointWeights.size());
		DEBUG_ULTIMATE("Depths map for reference image %3u fused using %u depths maps: %u new points, %u/%u cached images (%s)",
			idxImage, depthData.images.size() - 1, pointcloud.points.size() - nNumPointsPrev, numImageNeighborsInCache, numImagesInCache, TD_TIMER_GET_FMT().c_str());
		progress.display(numDMapsFused);
		// ensure enough memory is available for the next depth-maps chunk
		cacheDMaps.SkipMemoryCheckIdxImage();
		if (numDMapsFused % numDMapsReserveFusion == 0)
			cacheDMaps.SetMaxMemory(GetAvailableMemory(arrDepthData, fusedDMaps, numDMapsReserveFusion, cacheDMaps.GetUsedMemory()));
	}
	GET_LOGCONSOLE().Play();
	progress.close();
	arrUseMask.Release();
	cacheDMaps.ClearCache();
	if (!_bEstimateNormal)
		pointcloud.normals.Release();

	DEBUG_EXTRA("Depth-maps dense fused and filtered: %u depth-maps, %u depths, %u points (%d%%%%), %.2f hits in %.2f cached (%s)",
		numDMapsFused, nDepths, pointcloud.points.size(), ROUND2INT((100.f*pointcloud.points.size())/nDepths),
		static_cast<double>(totalNumImageNeighborsInCache) / numDMapsFused,
		static_cast<double>(totalNumImagesInCache) / numDMapsFused, TD_TIMER_GET_FMT().c_str());
} // DenseFuseDepthMaps
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

DenseDepthMapData::DenseDepthMapData(Scene& _scene, int _nFusionMode)
	: scene(_scene), depthMaps(_scene), idxImage(0), sem(1), nEstimationGeometricIter(-1), nFusionMode(_nFusionMode)
{
	if (nFusionMode < 0) {
		STEREO::SemiGlobalMatcher::CreateThreads(scene.nMaxThreads);
		if (nFusionMode == -1)
			OPTDENSE::nOptimize = 0;
	}
}
DenseDepthMapData::~DenseDepthMapData()
{
	if (nFusionMode < 0)
		STEREO::SemiGlobalMatcher::DestroyThreads();
}

void DenseDepthMapData::SignalCompleteDepthmapFilter()
{
	ASSERT(idxImage > 0);
	if (Thread::safeDec(idxImage) == 0)
		sem.Signal((unsigned)images.GetSize()*2);
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

static void* DenseReconstructionEstimateTmp(void*);
static void* DenseReconstructionFilterTmp(void*);

bool Scene::DenseReconstruction(int nFusionMode, bool bCrop2ROI, float fBorderROI)
{
	DenseDepthMapData data(*this, nFusionMode);

	// estimate depth-maps
	if (!ComputeDepthMaps(data))
		return false;
	if (ABS(nFusionMode) == 1)
		return true;

	// fuse all depth-maps
	pointcloud.Release();
	switch (OPTDENSE::nFuseFilter) {
	case OPTDENSE::FUSE_NOFILTER:
		// merge depth-maps
		data.depthMaps.MergeDepthMaps(pointcloud, OPTDENSE::nEstimateColors == 2, OPTDENSE::nEstimateNormals == 2);
		break;
	case OPTDENSE::FUSE_FILTER:
		// fuse depth-maps
		data.depthMaps.FuseDepthMaps(pointcloud, OPTDENSE::nEstimateColors == 2, OPTDENSE::nEstimateNormals == 2);
		break;
	case OPTDENSE::FUSE_DENSEFILTER:
		// dense fuse depth-maps
		data.depthMaps.DenseFuseDepthMaps(pointcloud, OPTDENSE::nEstimateColors == 2, OPTDENSE::nEstimateNormals == 2);
	}
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (g_nVerbosityLevel > 2) {
		// print number of points with 3+ views
		size_t nPoints1m(0), nPoints2(0), nPoints3p(0);
		FOREACHPTR(pViews, pointcloud.pointViews) {
			switch (pViews->GetSize())
			{
			case 0:
			case 1:
				++nPoints1m;
				break;
			case 2:
				++nPoints2;
				break;
			default:
				++nPoints3p;
			}
		}
		VERBOSE("Dense point-cloud composed of:\n\t%u points with 1- views\n\t%u points with 2 views\n\t%u points with 3+ views", nPoints1m, nPoints2, nPoints3p);
	}
	#endif

	if (!pointcloud.IsEmpty()) {
		if (bCrop2ROI && IsBounded()) {
			TD_TIMER_START();
			const size_t numPoints = pointcloud.GetSize();
			const OBB3f ROI(fBorderROI == 0 ? obb : (fBorderROI > 0 ? OBB3f(obb).EnlargePercent(fBorderROI) : OBB3f(obb).Enlarge(-fBorderROI)));
			pointcloud.RemovePointsOutside(ROI);
			VERBOSE("Point-cloud trimmed to ROI: %u points removed (%s)",
				numPoints-pointcloud.GetSize(), TD_TIMER_GET_FMT().c_str());
		}
		if (pointcloud.colors.IsEmpty() && OPTDENSE::nEstimateColors == 1)
			EstimatePointColors(images, pointcloud);
		if (pointcloud.normals.IsEmpty() && OPTDENSE::nEstimateNormals == 1)
			EstimatePointNormals(images, pointcloud);
	}

	if (OPTDENSE::bRemoveDmaps) {
		// delete all depth-map files
		FOREACH(i, images) {
			const DepthData& depthData = data.depthMaps.arrDepthData[i];
			if (!depthData.IsValid())
				continue;
			File::deleteFile(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
		}
	}
	return true;
} // DenseReconstruction
/*----------------------------------------------------------------*/

// do first half of dense reconstruction: depth map computation
// results are saved to "data"
bool Scene::ComputeDepthMaps(DenseDepthMapData& data)
{
	// compute point-cloud from the existing mesh
	if (!mesh.IsEmpty() && !ImagesHaveNeighbors()) {
		SampleMeshWithVisibility();
		mesh.Release();
	}
	
	// compute point-cloud from the existing mesh
	if (IsEmpty() && !ImagesHaveNeighbors()) {
		VERBOSE("warning: empty point-cloud, rough neighbor views selection based on image pairs baseline");
		EstimateNeighborViewsPointCloud();
	}

	{
	// maps global view indices to our list of views to be processed
	IIndexArr imagesMap;

	// prepare images for dense reconstruction (load if needed)
	{
		TD_TIMER_START();
		data.images.Reserve(images.GetSize());
		imagesMap.Resize(images.GetSize());
		#ifdef DENSE_USE_OPENMP
		bool bAbort(false);
		#pragma omp parallel for shared(data, bAbort)
		for (int_t ID=0; ID<(int_t)images.GetSize(); ++ID) {
			#pragma omp flush (bAbort)
			if (bAbort)
				continue;
			const IIndex idxImage((IIndex)ID);
		#else
		FOREACH(idxImage, images) {
		#endif
			// skip invalid, uncalibrated or discarded images
			Image& imageData = images[idxImage];
			if (!imageData.IsValid()) {
				#ifdef DENSE_USE_OPENMP
				#pragma omp critical
				#endif
				imagesMap[idxImage] = NO_ID;
				continue;
			}
			// map image index
			#ifdef DENSE_USE_OPENMP
			#pragma omp critical
			#endif
			{
				imagesMap[idxImage] = data.images.GetSize();
				data.images.Insert(idxImage);
			}
			// reload image at the appropriate resolution
			unsigned nResolutionLevel(OPTDENSE::nResolutionLevel);
			const unsigned nMaxResolution(imageData.RecomputeMaxResolution(nResolutionLevel, OPTDENSE::nMinResolution, OPTDENSE::nMaxResolution));
			if (!imageData.ReloadImage(nMaxResolution)) {
				#ifdef DENSE_USE_OPENMP
				bAbort = true;
				#pragma omp flush (bAbort)
				continue;
				#else
				return false;
				#endif
			}
			imageData.UpdateCamera(platforms);
			// print image camera
			DEBUG_ULTIMATE("K%d = \n%s", idxImage, cvMat2String(imageData.camera.K).c_str());
			DEBUG_LEVEL(3, "R%d = \n%s", idxImage, cvMat2String(imageData.camera.R).c_str());
			DEBUG_LEVEL(3, "C%d = \n%s", idxImage, cvMat2String(imageData.camera.C).c_str());
		}
		#ifdef DENSE_USE_OPENMP
		if (bAbort || data.images.IsEmpty()) {
		#else
		if (data.images.IsEmpty()) {
		#endif
			VERBOSE("error: preparing images for dense reconstruction failed (errors loading images)");
			return false;
		}
		VERBOSE("Preparing images for dense reconstruction completed: %d images (%s)", images.GetSize(), TD_TIMER_GET_FMT().c_str());
	}

	// select images to be used for dense reconstruction
	{
		TD_TIMER_START();
		// for each image, find all useful neighbor views
		IIndexArr invalidIDs;
		#ifdef DENSE_USE_OPENMP
		#pragma omp parallel for shared(data, invalidIDs)
		for (int_t ID=0; ID<(int_t)data.images.GetSize(); ++ID) {
			const IIndex idx((IIndex)ID);
		#else
		FOREACH(idx, data.images) {
		#endif
			const IIndex idxImage(data.images[idx]);
			ASSERT(imagesMap[idxImage] != NO_ID);
			DepthData& depthData(data.depthMaps.arrDepthData[idxImage]);
			if (!data.depthMaps.SelectViews(depthData)) {
				#ifdef DENSE_USE_OPENMP
				#pragma omp critical
				#endif
				invalidIDs.InsertSort(idx);
			}
		}
		RFOREACH(i, invalidIDs) {
			const IIndex idx(invalidIDs[i]);
			imagesMap[data.images.Last()] = idx;
			imagesMap[data.images[idx]] = NO_ID;
			data.images.RemoveAt(idx);
		}
		ASSERT(!data.images.IsEmpty());
		VERBOSE("Selecting images for dense reconstruction completed: %d images (%s)", data.images.GetSize(), TD_TIMER_GET_FMT().c_str());
	}
	}

	#ifdef _USE_CUDA
	// initialize CUDA
	if (SEACAVE::CUDA::desiredDeviceID >= -1 && data.nFusionMode >= 0) {
		data.depthMaps.pmCUDA = new MVS::CUDA::PatchMatch(SEACAVE::CUDA::desiredDeviceID);
		if (SEACAVE::CUDA::devices.IsEmpty())
			data.depthMaps.pmCUDA.Release();
		else
			data.depthMaps.pmCUDA->Init(false);
	}
	#endif // _USE_CUDA

	// initialize the queue of images to be processed
	const int nOptimize(OPTDENSE::nOptimize);
	if (OPTDENSE::nEstimationGeometricIters && data.nFusionMode >= 0)
		OPTDENSE::nOptimize = 0;
	data.idxImage = 0;
	ASSERT(data.events.IsEmpty());
	data.events.AddEvent(new EVTProcessImage(0));
	// start working threads
	data.progress = new Util::Progress("Estimated depth-maps", data.images.GetSize());
	GET_LOGCONSOLE().Pause();
	if (nMaxThreads > 1) {
		// multi-thread execution
		cList<SEACAVE::Thread> threads(2);
		FOREACHPTR(pThread, threads)
			pThread->start(DenseReconstructionEstimateTmp, (void*)&data);
		FOREACHPTR(pThread, threads)
			pThread->join();
	} else {
		// single-thread execution
		DenseReconstructionEstimate((void*)&data);
	}
	GET_LOGCONSOLE().Play();
	if (!data.events.IsEmpty())
		return false;
	data.progress.Release();

	if (data.nFusionMode >= 0) {
		#ifdef _USE_CUDA
		// initialize CUDA
		if (data.depthMaps.pmCUDA && OPTDENSE::nEstimationGeometricIters) {
			data.depthMaps.pmCUDA->Release();
			data.depthMaps.pmCUDA->Init(true);
		}
		#endif // _USE_CUDA
		while (++data.nEstimationGeometricIter < (int)OPTDENSE::nEstimationGeometricIters) {
			// initialize the queue of images to be geometric processed
			if (data.nEstimationGeometricIter+1 == (int)OPTDENSE::nEstimationGeometricIters)
				OPTDENSE::nOptimize = nOptimize;
			data.idxImage = 0;
			ASSERT(data.events.IsEmpty());
			data.events.AddEvent(new EVTProcessImage(0));
			// start working threads
			data.progress = new Util::Progress("Geometric-consistent estimated depth-maps", data.images.GetSize());
			GET_LOGCONSOLE().Pause();
			if (nMaxThreads > 1) {
				// multi-thread execution
				cList<SEACAVE::Thread> threads(2);
				FOREACHPTR(pThread, threads)
					pThread->start(DenseReconstructionEstimateTmp, (void*)&data);
				FOREACHPTR(pThread, threads)
					pThread->join();
			} else {
				// single-thread execution
				DenseReconstructionEstimate((void*)&data);
			}
			GET_LOGCONSOLE().Play();
			if (!data.events.IsEmpty())
				return false;
			data.progress.Release();
			// replace raw depth-maps with the geometric-consistent ones
			for (IIndex idx: data.images) {
				const DepthData& depthData(data.depthMaps.arrDepthData[idx]);
				if (!depthData.IsValid())
					continue;
				const String rawName(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
				File::deleteFile(rawName);
				File::renameFile(ComposeDepthFilePath(depthData.GetView().GetID(), "geo.dmap"), rawName);
			}
		}
		data.nEstimationGeometricIter = -1;
	}

	if ((OPTDENSE::nOptimize & OPTDENSE::ADJUST_FILTER) != 0) {
		// initialize the queue of depth-maps to be filtered
		data.sem.Clear();
		data.idxImage = data.images.GetSize();
		ASSERT(data.events.IsEmpty());
		FOREACH(i, data.images)
			data.events.AddEvent(new EVTFilterDepthMap(i));
		// start working threads
		data.progress = new Util::Progress("Filtered depth-maps", data.images.GetSize());
		GET_LOGCONSOLE().Pause();
		if (nMaxThreads > 1) {
			// multi-thread execution
			cList<SEACAVE::Thread> threads(MINF(nMaxThreads, (unsigned)data.images.GetSize()));
			FOREACHPTR(pThread, threads)
				pThread->start(DenseReconstructionFilterTmp, (void*)&data);
			FOREACHPTR(pThread, threads)
				pThread->join();
		} else {
			// single-thread execution
			DenseReconstructionFilter((void*)&data);
		}
		GET_LOGCONSOLE().Play();
		if (!data.events.IsEmpty())
			return false;
		data.progress.Release();
	}
	return true;
} // ComputeDepthMaps
/*----------------------------------------------------------------*/

void* DenseReconstructionEstimateTmp(void* arg) {
	const DenseDepthMapData& dataThreads = *((const DenseDepthMapData*)arg);
	dataThreads.scene.DenseReconstructionEstimate(arg);
	return NULL;
}

// initialize the dense reconstruction with the sparse point cloud
void Scene::DenseReconstructionEstimate(void* pData)
{
	DenseDepthMapData& data = *((DenseDepthMapData*)pData);
	while (true) {
		CAutoPtr<Event> evt(data.events.GetEvent());
		switch (evt->GetID()) {
		case EVT_PROCESSIMAGE: {
			const EVTProcessImage& evtImage = *((EVTProcessImage*)(Event*)evt);
			if (evtImage.idxImage >= data.images.size()) {
				if (nMaxThreads > 1) {
					// close working threads
					data.events.AddEvent(new EVTClose);
				}
				return;
			}
			// select views to reconstruct the depth-map for this image
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			const bool depthmapComputed(data.nFusionMode < 0 || (data.nFusionMode >= 0 && data.nEstimationGeometricIter < 0 && File::access(ComposeDepthFilePath(data.scene.images[idx].ID, "dmap"))));
			// initialize images pair: reference image and the best neighbor view
			ASSERT(data.neighborsMap.IsEmpty() || data.neighborsMap[evtImage.idxImage] != NO_ID);
			if (!data.depthMaps.InitViews(depthData, data.neighborsMap.IsEmpty()?NO_ID:data.neighborsMap[evtImage.idxImage], OPTDENSE::nNumViews, !depthmapComputed, depthmapComputed ? -1 : (data.nEstimationGeometricIter >= 0 ? 1 : 0))) {
				// process next image
				data.events.AddEvent(new EVTProcessImage((IIndex)Thread::safeInc(data.idxImage)));
				break;
			}
			// try to load already compute depth-map for this image
			if (depthmapComputed && data.nFusionMode >= 0) {
				if (OPTDENSE::nOptimize & OPTDENSE::OPTIMIZE) {
					if (!depthData.Load(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"))) {
						VERBOSE("error: invalid depth-map '%s'", ComposeDepthFilePath(depthData.GetView().GetID(), "dmap").c_str());
						exit(EXIT_FAILURE);
					}
					// optimize depth-map
					data.events.AddEventFirst(new EVTOptimizeDepthMap(evtImage.idxImage));
				}
				// process next image
				data.events.AddEvent(new EVTProcessImage((uint32_t)Thread::safeInc(data.idxImage)));
			} else {
				// estimate depth-map
				data.events.AddEventFirst(new EVTEstimateDepthMap(evtImage.idxImage));
			}
			break; }

		case EVT_ESTIMATEDEPTHMAP: {
			const EVTEstimateDepthMap& evtImage = *((EVTEstimateDepthMap*)(Event*)evt);
			// request next image initialization to be performed while computing this depth-map
			data.events.AddEvent(new EVTProcessImage((uint32_t)Thread::safeInc(data.idxImage)));
			// extract depth map
			data.sem.Wait();
			if (data.nFusionMode >= 0) {
				// extract depth-map using Patch-Match algorithm
				data.depthMaps.EstimateDepthMap(data.images[evtImage.idxImage], data.nEstimationGeometricIter);
			} else {
				// extract disparity-maps using SGM algorithm
				if (data.nFusionMode == -1) {
					data.sgm.Match(*this, data.images[evtImage.idxImage], OPTDENSE::nNumViews);
				} else {
					// fuse existing disparity-maps
					const IIndex idx(data.images[evtImage.idxImage]);
					DepthData& depthData(data.depthMaps.arrDepthData[idx]);
					data.sgm.Fuse(*this, data.images[evtImage.idxImage], OPTDENSE::nNumViews, 2, depthData.depthMap, depthData.confMap);
					if (OPTDENSE::nEstimateNormals == 2)
						EstimateNormalMap(depthData.images.front().camera.K, depthData.depthMap, depthData.normalMap);
					depthData.dMin = ZEROTOLERANCE<float>(); depthData.dMax = FLT_MAX;
				}
			}
			data.sem.Signal();
			if (OPTDENSE::nOptimize & OPTDENSE::OPTIMIZE) {
				// optimize depth-map
				data.events.AddEventFirst(new EVTOptimizeDepthMap(evtImage.idxImage));
			} else {
				// save depth-map
				data.events.AddEventFirst(new EVTSaveDepthMap(evtImage.idxImage));
			}
			break; }

		case EVT_OPTIMIZEDEPTHMAP: {
			const EVTOptimizeDepthMap& evtImage = *((EVTOptimizeDepthMap*)(Event*)evt);
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			#if TD_VERBOSE != TD_VERBOSE_OFF
			// save depth map as image
			if (g_nVerbosityLevel > 3)
				ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "raw.png"), depthData.depthMap);
			#endif
			// apply filters
			if (OPTDENSE::nOptimize & (OPTDENSE::REMOVE_SPECKLES)) {
				TD_TIMER_START();
				if (data.depthMaps.RemoveSmallSegments(depthData)) {
					DEBUG_ULTIMATE("Depth-map %3u filtered: remove small segments (%s)", depthData.GetView().GetID(), TD_TIMER_GET_FMT().c_str());
				}
			}
			if (OPTDENSE::nOptimize & (OPTDENSE::FILL_GAPS)) {
				TD_TIMER_START();
				if (data.depthMaps.GapInterpolation(depthData)) {
					DEBUG_ULTIMATE("Depth-map %3u filtered: gap interpolation (%s)", depthData.GetView().GetID(), TD_TIMER_GET_FMT().c_str());
				}
			}
			// save depth-map
			data.events.AddEventFirst(new EVTSaveDepthMap(evtImage.idxImage));
			break; }

		case EVT_SAVEDEPTHMAP: {
			const EVTSaveDepthMap& evtImage = *((EVTSaveDepthMap*)(Event*)evt);
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			#if TD_VERBOSE != TD_VERBOSE_OFF
			// save depth map as image
			if (g_nVerbosityLevel > 2) {
				ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "png"), depthData.depthMap);
				ExportConfidenceMap(ComposeDepthFilePath(depthData.GetView().GetID(), "conf.png"), depthData.confMap);
				ExportPointCloud(ComposeDepthFilePath(depthData.GetView().GetID(), "ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
				if (g_nVerbosityLevel > 4) {
					ExportNormalMap(ComposeDepthFilePath(depthData.GetView().GetID(), "normal.png"), depthData.normalMap);
					depthData.confMap.Save(ComposeDepthFilePath(depthData.GetView().GetID(), "conf.pfm"));
				}
			}
			#endif
			// save compute depth-map for this image
			if (!depthData.depthMap.empty())
				depthData.Save(ComposeDepthFilePath(depthData.GetView().GetID(), data.nEstimationGeometricIter < 0 ? "dmap" : "geo.dmap"));
			depthData.ReleaseImages();
			depthData.Release();
			data.progress->operator++();
			break; }

		case EVT_CLOSE: {
			return; }

		default:
			ASSERT("Should not happen!" == NULL);
		}
	}
} // DenseReconstructionEstimate
/*----------------------------------------------------------------*/

void* DenseReconstructionFilterTmp(void* arg) {
	DenseDepthMapData& dataThreads = *((DenseDepthMapData*)arg);
	dataThreads.scene.DenseReconstructionFilter(arg);
	return NULL;
}

// filter estimated depth-maps
void Scene::DenseReconstructionFilter(void* pData)
{
	DenseDepthMapData& data = *((DenseDepthMapData*)pData);
	CAutoPtr<Event> evt;
	while ((evt=data.events.GetEvent(0)) != NULL) {
		switch (evt->GetID()) {
		case EVT_FILTERDEPTHMAP: {
			const EVTFilterDepthMap& evtImage = *((EVTFilterDepthMap*)(Event*)evt);
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			if (!depthData.IsValid()) {
				data.SignalCompleteDepthmapFilter();
				break;
			}
			// make sure all depth-maps are loaded
			depthData.IncRef(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
			const unsigned numMaxNeighbors(8);
			IIndexArr idxNeighbors(0, depthData.neighbors.GetSize());
			FOREACH(n, depthData.neighbors) {
				const IIndex idxView = depthData.neighbors[n].ID;
				DepthData& depthDataPair = data.depthMaps.arrDepthData[idxView];
				if (!depthDataPair.IsValid())
					continue;
				if (depthDataPair.IncRef(ComposeDepthFilePath(depthDataPair.GetView().GetID(), "dmap")) == 0) {
					// signal error and terminate
					data.events.AddEventFirst(new EVTFail);
					return;
				}
				idxNeighbors.Insert(n);
				if (idxNeighbors.GetSize() == numMaxNeighbors)
					break;
			}
			// filter the depth-map for this image
			if (data.depthMaps.FilterDepthMap(depthData, idxNeighbors, OPTDENSE::bFilterAdjust)) {
				// load the filtered maps after all depth-maps were filtered
				data.events.AddEvent(new EVTAdjustDepthMap(evtImage.idxImage));
			}
			// unload referenced depth-maps
			FOREACHPTR(pIdxNeighbor, idxNeighbors) {
				const IIndex idxView = depthData.neighbors[*pIdxNeighbor].ID;
				DepthData& depthDataPair = data.depthMaps.arrDepthData[idxView];
				depthDataPair.DecRef();
			}
			depthData.DecRef();
			data.SignalCompleteDepthmapFilter();
			break; }

		case EVT_ADJUSTDEPTHMAP: {
			const EVTAdjustDepthMap& evtImage = *((EVTAdjustDepthMap*)(Event*)evt);
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			ASSERT(depthData.IsValid());
			data.sem.Wait();
			// load filtered maps
			if (depthData.IncRef(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap")) == 0 ||
				!LoadDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.dmap"), depthData.depthMap) ||
				!LoadConfidenceMap(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.cmap"), depthData.confMap))
			{
				// signal error and terminate
				data.events.AddEventFirst(new EVTFail);
				return;
			}
			ASSERT(depthData.GetRef() == 1);
			File::deleteFile(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.dmap").c_str());
			File::deleteFile(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.cmap").c_str());
			#if TD_VERBOSE != TD_VERBOSE_OFF
			// save depth map as image
			if (g_nVerbosityLevel > 2) {
				ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.png"), depthData.depthMap);
				ExportPointCloud(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
			}
			#endif
			// save filtered depth-map for this image
			depthData.Save(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap"));
			depthData.DecRef();
			data.progress->operator++();
			break; }

		case EVT_FAIL: {
			data.events.AddEventFirst(new EVTFail);
			return; }

		default:
			ASSERT("Should not happen!" == NULL);
		}
	}
} // DenseReconstructionFilter
/*----------------------------------------------------------------*/

// filter point-cloud based on camera-point visibility intersections
void Scene::PointCloudFilter(int thRemove)
{
	TD_TIMER_STARTD();

	typedef TOctree<PointCloud::PointArr,PointCloud::Point::Type,3,uint32_t> Octree;
	struct Collector {
		typedef Octree::IDX_TYPE IDX;
		typedef PointCloud::Point::Type Real;
		typedef TCone<Real,3> Cone;
		typedef TSphere<Real,3> Sphere;
		typedef TConeIntersect<Real,3> ConeIntersect;

		Cone cone;
		const ConeIntersect coneIntersect;
		const PointCloud& pointcloud;
		IntArr& visibility;
		PointCloud::Index idxPoint;
		Real distance;
		int weight;
		#ifdef DENSE_USE_OPENMP
		uint8_t pcs[sizeof(CriticalSection)];
		#endif

		Collector(const Cone::RAY& ray, Real angle, const PointCloud& _pointcloud, IntArr& _visibility)
			: cone(ray, angle), coneIntersect(cone), pointcloud(_pointcloud), visibility(_visibility)
		#ifdef DENSE_USE_OPENMP
		{ new(pcs) CriticalSection; }
		~Collector() { reinterpret_cast<CriticalSection*>(pcs)->~CriticalSection(); }
		inline CriticalSection& GetCS() { return *reinterpret_cast<CriticalSection*>(pcs); }
		#else
		{}
		#endif
		inline void Init(PointCloud::Index _idxPoint, const PointCloud::Point& X, int _weight) {
			const Real thMaxDepth(1.02f);
			idxPoint =_idxPoint;
			const PointCloud::Point::EVec D((PointCloud::Point::EVec&)X-cone.ray.m_pOrig);
			distance = D.norm();
			cone.ray.m_vDir = D/distance;
			cone.maxHeight = MaxDepthDifference(distance, thMaxDepth);
			weight = _weight;
		}
		inline bool Intersects(const Octree::POINT_TYPE& center, Octree::Type radius) const {
			return coneIntersect(Sphere(center, radius*Real(SQRT_3)));
		}
		inline void operator() (const IDX* idices, IDX size) {
			const Real thSimilar(0.01f);
			Real dist;
			FOREACHRAWPTR(pIdx, idices, size) {
				const PointCloud::Index idx(*pIdx);
				if (coneIntersect.Classify(pointcloud.points[idx], dist) == VISIBLE && !IsDepthSimilar(distance, dist, thSimilar)) {
					if (dist > distance)
						visibility[idx] += pointcloud.pointViews[idx].size();
					else
						visibility[idx] -= weight;
				}
			}
		}
	};
	typedef CLISTDEF2(Collector) Collectors;

	// create octree to speed-up search
	Octree octree(pointcloud.points, [](Octree::IDX_TYPE size, Octree::Type /*radius*/) {
		return size > 128;
	});
	IntArr visibility(pointcloud.GetSize()); visibility.Memset(0);
	Collectors collectors; collectors.reserve(images.size());
	FOREACH(idxView, images) {
		const Image& image = images[idxView];
		const Ray3f ray(Cast<float>(image.camera.C), Cast<float>(image.camera.Direction()));
		const float angle(float(image.ComputeFOV(0)/image.width));
		collectors.emplace_back(ray, angle, pointcloud, visibility);
	}

	// run all camera-point visibility intersections
	Util::Progress progress(_T("Point visibility checks"), pointcloud.GetSize());
	#ifdef DENSE_USE_OPENMP
	#pragma omp parallel for //schedule(dynamic)
	for (int64_t i=0; i<(int64_t)pointcloud.GetSize(); ++i) {
		const PointCloud::Index idxPoint((PointCloud::Index)i);
	#else
	FOREACH(idxPoint, pointcloud.points) {
	#endif
		const PointCloud::Point& X = pointcloud.points[idxPoint];
		const PointCloud::ViewArr& views = pointcloud.pointViews[idxPoint];
		for (PointCloud::View idxView: views) {
			Collector& collector = collectors[idxView];
			#ifdef DENSE_USE_OPENMP
			Lock l(collector.GetCS());
			#endif
			collector.Init(idxPoint, X, (int)views.size());
			octree.Collect(collector, collector);
		}
		++progress;
	}
	progress.close();

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (g_nVerbosityLevel > 2) {
		// print visibility stats
		UnsignedArr counts(0, 64);
		for (int views: visibility) {
			if (views > 0)
				continue;
			while (counts.size() <= IDX(-views))
				counts.push_back(0);
			++counts[-views];
		}
		String msg;
		msg.reserve(64*counts.size());
		FOREACH(c, counts)
			if (counts[c])
				msg += String::FormatString("\n\t% 3u - % 9u", c, counts[c]);
		VERBOSE("Visibility lengths (%u points):%s", pointcloud.GetSize(), msg.c_str());
		// save outlier points
		PointCloud pc;
		RFOREACH(idxPoint, pointcloud.points) {
			if (visibility[idxPoint] <= thRemove) {
				pc.points.push_back(pointcloud.points[idxPoint]);
				pc.colors.push_back(pointcloud.colors[idxPoint]);
			}
		}
		pc.Save(MAKE_PATH("scene_dense_outliers.ply"));
	}
	#endif

	// filter points
	const size_t numInitPoints(pointcloud.GetSize());
	RFOREACH(idxPoint, pointcloud.points) {
		if (visibility[idxPoint] <= thRemove)
			pointcloud.RemovePoint(idxPoint);
	}

	DEBUG_EXTRA("Point-cloud filtered: %u/%u points (%d%%%%) (%s)", pointcloud.points.size(), numInitPoints, ROUND2INT((100.f*pointcloud.points.GetSize())/numInitPoints), TD_TIMER_GET_FMT().c_str());
} // PointCloudFilter
/*----------------------------------------------------------------*/
