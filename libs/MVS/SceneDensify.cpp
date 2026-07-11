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
#include "PatchMatchMetal.h"
#include "DMapCache.h"
#include "ConfidenceRefine.h"
#include "ConfidenceCUDA.h"
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <vector>
#ifdef _USE_SSE
#include <smmintrin.h> // SSE4.1 (_mm_floor_ps) for the vectorized confirmation sweep
#endif

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define DENSE_USE_OPENMP
#endif

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ScnDense"));

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

// per-pixel fusion-label values written to the .flabel maps (kept in sync with scripts/python/EvalConfidence.py)
namespace {
enum FusionLabel : uint8_t {
	LABEL_INVALID          = 0, // no depth estimate
	LABEL_OUTLIER          = 1, // seen by a neighbor but geometrically not confirmed
	LABEL_AMBIGUOUS        = 2, // not covered by any neighbor (cannot decide)
	LABEL_WEAK_INLIER      = 3, // confirmed by >=2 views but too few views/pixels to pass strict fusion (fusion false-negative)
	LABEL_CONFIDENT_INLIER = 4, // would survive strict DenseFuse (>=nMinViewsFuse views and >=nMinPixelsFuse pixels)
};
// minimal raw map writer for the evaluation harness;
// header: int32 magic('LMAP'=0x50414D4C), int32 width, int32 height, int32 elemType (1=uint8, 2=uint16, 4=float32); then row-major data
template <typename T>
bool SaveRawMap(const String& fileName, const TImage<T>& map, int32_t elemType) {
	FILE* f = fopen(fileName.c_str(), "wb");
	if (f == NULL)
		return false;
	const int32_t hdr[4] = { 0x50414D4C, (int32_t)map.cols, (int32_t)map.rows, elemType };
	bool ok = fwrite(hdr, sizeof(hdr), 1, f) == 1;
	for (int r = 0; ok && r < map.rows; ++r)
		ok = fwrite(map.template ptr<T>(r), sizeof(T), (size_t)map.cols, f) == (size_t)map.cols;
	fclose(f);
	return ok;
}
} // namespace



// S T R U C T S ///////////////////////////////////////////////////


DepthMapsData::DepthMapsData(Scene& _scene)
	:
	scene(_scene),
	arrDepthData(_scene.images.GetSize())
	#ifdef _USE_CUDA
	, pmCUDANextIdx((Thread::safe_t)-1)
	, pmCUDAEpoch(0)
	#endif // _USE_CUDA
	#ifdef _USE_METAL
	, pmMetalNextIdx((Thread::safe_t)-1)
	, pmMetalEpoch(0)
	#endif // _USE_METAL
{
} // constructor

DepthMapsData::~DepthMapsData()
{
} // destructor

#ifdef _USE_CUDA
bool DepthMapsData::AllocateCudaPool(unsigned poolSize)
{
	ASSERT(pmCUDAPool.empty());
	if (poolSize == 0)
		poolSize = 1;
	// PatchMatch's ctor triggers CUDA::initDevices() on first construction;
	// build one to probe and check whether any device was actually picked up.
	auto probe = std::make_unique<MVS::CUDA::PatchMatch>();
	if (SEACAVE::CUDA::devices.IsEmpty())
		return false;
	probe->Init(false);
	pmCUDAPool.reserve(poolSize);
	pmCUDAPool.emplace_back(std::move(probe));
	for (unsigned k = 1; k < poolSize; ++k) {
		auto pm = std::make_unique<MVS::CUDA::PatchMatch>();
		pm->Init(false);
		pmCUDAPool.emplace_back(std::move(pm));
	}
	pmCUDANextIdx = (Thread::safe_t)-1;
	return true;
}

void DepthMapsData::ReinitCudaPoolForGeom()
{
	for (auto& pm : pmCUDAPool) {
		pm->Release();
		pm->Init(true);
	}
	pmCUDANextIdx = (Thread::safe_t)-1;
	Thread::safeInc(pmCUDAEpoch);
}
#endif // _USE_CUDA

#ifdef _USE_METAL
bool DepthMapsData::AllocateMetalPool(unsigned poolSize)
{
	ASSERT(pmMetalPool.empty());
	if (poolSize == 0)
		poolSize = 1;
	auto probe = std::make_unique<MVS::METAL::PatchMatch>();
	if (!probe->IsValid())
		return false;
	probe->Init(false);
	pmMetalPool.reserve(poolSize);
	pmMetalPool.emplace_back(std::move(probe));
	for (unsigned k = 1; k < poolSize; ++k) {
		auto pm = std::make_unique<MVS::METAL::PatchMatch>();
		// the probe proved the device + pipelines build; an additional instance
		// failing is unexpected (e.g. resource exhaustion), so stop growing rather
		// than add an invalid worker that would silently produce empty depth-maps
		if (!pm->IsValid())
			break;
		pm->Init(false);
		pmMetalPool.emplace_back(std::move(pm));
	}
	pmMetalNextIdx = (Thread::safe_t)-1;
	return true;
}

void DepthMapsData::ReinitMetalPoolForGeom()
{
	for (auto& pm : pmMetalPool) {
		pm->Release();
		pm->Init(true);
	}
	pmMetalNextIdx = (Thread::safe_t)-1;
	Thread::safeInc(pmMetalEpoch);
}
#endif // _USE_METAL
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
		!scene.SelectNeighborViews(idxImage, depthData.points, OPTDENSE::nMinViews, OPTDENSE::nMinViewsTrustPoint>1?OPTDENSE::nMinViewsTrustPoint:2, FD2R(OPTDENSE::fOptimAngle), OPTDENSE::fWeightPointInsideROI))
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
// if loadDepthMaps is 1, the depth-maps are loaded from disk (neighbors: depth only),
// if 2, same as 1 but neighbors' normal-map and confidence-map are ALSO loaded (Task 12: only used
// for the last geometric-consistency iteration when OPTDENSE::bEstimateConfidence is set, so the
// integrated DepthMapsData::AdjustConfidence(DepthData&) overload can read them from
// depthData.images[] -- no extra disk open, same neighbor "dmap" file already being read for depth),
// if 0, the reference depth-map is initialized from sparse point-cloud,
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
		if (VERBOSITY_LEVEL > 2) {
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
	for (IIndex i=1; i<depthData.images.size(); ) {
		DepthData::ViewData& view = depthData.images[i];
		if (loadDepthMaps > 0) {
			// load known depth-map;
			// Task 12: when loadDepthMaps==2 (last geometric-consistency iteration with
			// OPTDENSE::bEstimateConfidence), also decode this neighbor's normal-map and
			// confidence-map from the SAME file/read (ImportDepthDataRaw just skips those bytes
			// via fseek otherwise -- no extra disk open either way) directly into view.normalMap /
			// view.confMap, so the integrated AdjustConfidence(DepthData&) overload below can use
			// them without any additional neighbor load
			String imageFileName;
			IIndexArr IDs;
			cv::Size imageSize;
			Depth dMin, dMax;
			NormalMap normalMap;
			ConfidenceMap confMap;
			ViewsMap viewsMap;
			const bool bLoadNeighborConf(loadDepthMaps >= 2);
			const unsigned nLoadFlags(bLoadNeighborConf ?
				(HeaderDepthDataRaw::HAS_DEPTH|HeaderDepthDataRaw::HAS_NORMAL|HeaderDepthDataRaw::HAS_CONF) :
				HeaderDepthDataRaw::HAS_DEPTH);
			if (!ImportDepthDataRaw(ComposeDepthFilePath(view.GetID(), "dmap"),
				imageFileName, IDs, imageSize, view.cameraDepthMap.K, view.cameraDepthMap.R, view.cameraDepthMap.C,
				dMin, dMax, view.depthMap, normalMap, confMap, viewsMap, nLoadFlags))
			{
				// neighbor depth-maps are needed during geometric-consistency iterations;
				// some views may have failed depth estimation, so their depth-map is missing
				VERBOSE("warning: skipping neighbor view %u (%s): cannot load depth-map '%s'",
					view.GetID(), Util::getFileNameExt(view.pImageData->name).c_str(), ComposeDepthFilePath(view.GetID(), "dmap").c_str());
				view.depthMap.release();
				depthData.images.RemoveAtMove(i);
				continue;
			}
			ASSERT(viewRef.image.size() == view.depthMap.size());
			if (bLoadNeighborConf) {
				view.normalMap = std::move(normalMap);
				view.confMap = std::move(confMap);
			}
		}
		view.Init(viewRef.camera);
		++i;
	}
	if (depthData.images.size() < 2) {
		depthData.images.Release();
		return false;
 	}

	// initialize depth-map and normal-map for the reference image
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

// roughly estimate depth and normal maps by triangulating the sparse point-cloud
// and interpolating normal and depth for all pixels
bool DepthMapsData::InitDepthMap(DepthData& depthData)
{
	TD_TIMER_STARTD();

	ASSERT(depthData.images.GetSize() > 1 && !depthData.points.IsEmpty());
	const DepthData::ViewData& image(depthData.GetView());
	TriangulatePoints2DepthMap(image.camera, image.image.size(), scene.pointcloud, depthData.points,
		depthData.depthMap, depthData.normalMap, depthData.dMin, depthData.dMax,
		OPTDENSE::bAddCorners && image.pImageData->avgDepth > 0 ? image.pImageData->avgDepth : 0.f, OPTDENSE::bInitSparse);
	depthData.dMin *= 0.9f;
	depthData.dMax *= 1.1f;

	#if TD_VERBOSE != TD_VERBOSE_OFF
	// save rough depth map as image
	if (VERBOSITY_LEVEL > 4) {
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
		ASSERT(ISEQUAL(norm(normal), 1.f), "Norm = ", norm(normal));
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
	if (!pmCUDAPool.empty()) {
		// claim a pool slot for this worker thread; epoch invalidates the claim
		// across phase boundaries so re-used OS threads pick a fresh slot. also
		// re-claim when a thread reused across DepthMapsData instances holds a slot
		// now out of range for a smaller pool (epochs can collide at the value 0)
		static thread_local int s_slot = -1;
		static thread_local Thread::safe_t s_epoch = (Thread::safe_t)-1;
		if (!ISINSIDE(s_slot, 0, (int)pmCUDAPool.size()) || s_epoch != pmCUDAEpoch) {
			s_slot = (int)(Thread::safeInc(pmCUDANextIdx) % (Thread::safe_t)pmCUDAPool.size());
			s_epoch = pmCUDAEpoch;
		}
		pmCUDAPool[s_slot]->EstimateDepthMap(arrDepthData[idxImage]);
		return true;
	}
	#endif // _USE_CUDA

	#ifdef _USE_METAL
	if (!pmMetalPool.empty()) {
		// runs both photometric (nGeometricIter < 0) and geometric-consistency passes;
		// the pool's bGeomConsistency state (Init/ReinitMetalPoolForGeom) selects the mode
		static thread_local int s_slotM = -1;
		static thread_local Thread::safe_t s_epochM = (Thread::safe_t)-1;
		// re-claim a slot when uninitialized, after a phase boundary (epoch bump), or
		// when a thread reused across DepthMapsData instances holds a slot that is now
		// out of range for a smaller pool (epochs can collide at the initial value 0)
		if (!ISINSIDE(s_slotM, 0, (int)pmMetalPool.size()) || s_epochM != pmMetalEpoch) {
			s_slotM = (int)(Thread::safeInc(pmMetalNextIdx) % (Thread::safe_t)pmMetalPool.size());
			s_epochM = pmMetalEpoch;
		}
		pmMetalPool[s_slotM]->EstimateDepthMap(arrDepthData[idxImage]);
		return true;
	}
	#endif // _USE_METAL

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
			if (OPTDENSE::nIgnoreMaskLabel >= 0 && DepthEstimator::ImportIgnoreMask(*image.pImageData, depthData.depthMap.size(), (uint8_t)OPTDENSE::nIgnoreMaskLabel, mask))
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
			if (VERBOSITY_LEVEL > 4 && nGeometricIter < 0) {
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
			if (VERBOSITY_LEVEL > 4) {
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


// compute a cheap intra-map geometric prior in [0,1] from local depth (and normal) coherence;
// high where the local surface is smooth and the normals are consistent, computed once per map with no
// neighbor views; used as a Bayesian prior during confidence adjustment to keep coherent but weakly-confirmed
// estimates alive (recovering fusion's few-view false-negatives)
// bParallel: enable the inner "#pragma omp parallel for" -- false when called from a context that is
// already parallelized by some OTHER mechanism, per this codebase's no-per-view-threading policy (see
// GetIntraMapPrior's declaration comment in SceneDensify.h -- and its honest measured-effect note)
void DepthMapsData::ComputeIntraMapPrior(const DepthData& depthData, ConfidenceMap& priorMap, bool bParallel) const
{
	const DepthMap& depthMap = depthData.depthMap;
	const NormalMap& normalMap = depthData.normalMap;
	const bool bHasNormal(!normalMap.empty());
	const Matrix3x3f K(depthData.GetView().camera.K);
	const DepthGradientEstimator est(K, depthMap);
	const float band(OPTDENSE::fDepthDiffThreshold * 3.f);          // relative planarity band
	const float sigmaNormalSq(SQUARE(FD2R(OPTDENSE::fNormalDiffThreshold))); // angular-variance scale
	const float invKmin(1.f / 4.f);                                // soft planar quorum (~4 inliers)
	const bool bCoherence(OPTDENSE::bConfPriorNormalCoherence);
	priorMap.create(depthMap.size());
	priorMap.memset(0);
	#ifdef DENSE_USE_OPENMP
	#pragma omp parallel for if(bParallel)
	#endif
	for (int r=0; r<depthMap.rows; ++r) {
		for (int c=0; c<depthMap.cols; ++c) {
			const Depth w(depthMap(r,c));
			if (w <= 0)
				continue;
			// fit a slope-aware local depth plane using only depth-similar neighbors
			Point3f ws;
			if (!est.DepthGradient(ImageRef(c,r), ws))
				continue;                                          // not on a locally coherent surface
			const float wx(ws[1]), wy(ws[2]);
			// slope-aware planarity + inlier quorum over the 3x3 window:
			// count neighbors whose depth matches the fitted plane prediction (not just the center depth)
			int nInl(0);
			float sumE2(0);
			for (int y=-1; y<=1; ++y) {
				const int rr(r+y);
				if (rr<0 || rr>=depthMap.rows) continue;
				for (int x=-1; x<=1; ++x) {
					if (x==0 && y==0) continue;
					const int cc(c+x);
					if (cc<0 || cc>=depthMap.cols) continue;
					const Depth dN(depthMap(rr,cc));
					if (dN <= 0) continue;
					const float dpred(w + wx*x + wy*y);
					const float e(ABS(dN-dpred)/w);
					if (e < band) { ++nInl; sumE2 += SQUARE(e/band); }
				}
			}
			if (nInl < 3)
				continue;
			const float Pplane(EXP(-sumE2/nInl));
			const float gate(1.f - EXP(-(float)nInl*invKmin));
			// normal agreement: depth-gradient normal vs stored (photometric) normal = burst discriminator.
			// On a real surface the two coincide; on a textureless/repetitive burst the photometric normal
			// is unconstrained and disagrees with the geometry-implied gradient normal => Pnorm collapses.
			float Pnorm(1.f);
			if (bHasNormal) {
				const Normal nGrad(est.NormalFromGradient(c, r, w, wx, wy));
				Pnorm = MAXF(0.f, nGrad.dot(normalMap(r,c)));
				if (bCoherence) {
					// variant B: also require coherence of the stored normals across the window
					Point3f mean(0,0,0); int cnt(0);
					for (int y=-1; y<=1; ++y) { const int rr(r+y); if (rr<0||rr>=depthMap.rows) continue;
						for (int x=-1; x<=1; ++x) { const int cc(c+x); if (cc<0||cc>=depthMap.cols) continue;
							if (depthMap(rr,cc) <= 0) continue; mean += normalMap(rr,cc); ++cnt; } }
					const float nrm(norm(mean));
					if (cnt > 0 && nrm > 1e-6f) {
						mean /= nrm;
						float varAng(0); int cnt2(0);
						for (int y=-1; y<=1; ++y) { const int rr(r+y); if (rr<0||rr>=depthMap.rows) continue;
							for (int x=-1; x<=1; ++x) { const int cc(c+x); if (cc<0||cc>=depthMap.cols) continue;
								if (depthMap(rr,cc) <= 0) continue;
								const float dotN(CLAMP(mean.dot(normalMap(rr,cc)), -1.f, 1.f));
								varAng += SQUARE(ACOS(dotN)); ++cnt2; } }
						Pnorm *= EXP(-varAng/(cnt2*sigmaNormalSq));
					}
				}
			}
			priorMap(r,c) = CLAMP(Pplane*Pnorm*gate, 0.f, 1.f);
		}
	}
} // ComputeIntraMapPrior
/*----------------------------------------------------------------*/

// compute-if-absent accessor for the intra-map prior: shared by AdjustConfidence and
// DenseFuseDepthMaps so the O(pixels) prior computed for a given DepthData is not recomputed if it
// is touched a second time before being released. Caveat honestly documented here (not just claimed):
// this only helps within the lifetime of a single cached DepthData -- both the adjust phase's own
// DMapCache (ComputeDepthMaps) and the fusion phase's DMapCache call ClearCache()/Release() on every
// touched image at the END of their respective phase, which also clears priorMap (see the
// DepthData::priorMap comment in DepthMap.h for why that's the right lifetime). So in EVERY currently
// existing call pattern -- staged benchmark (separate processes) AND the default single-process
// estimate->adjust->fuse pipeline alike -- AdjustConfidence's prior for an image is gone by the time
// DenseFuseDepthMaps looks at that same image, and this accessor's cache is a guaranteed miss on
// first touch, same as calling ComputeIntraMapPrior directly. It is still the right shape (a single
// shared code path + a real cache slot) for any future pipeline restructuring that keeps DepthData
// resident across phases; the measurable win of this change is entirely the bParallel argument below.
const ConfidenceMap& DepthMapsData::GetIntraMapPrior(DepthData& depthData, bool bParallel) const
{
	if (depthData.priorMap.empty())
		ComputeIntraMapPrior(depthData, depthData.priorMap, bParallel);
	return depthData.priorMap;
}
/*----------------------------------------------------------------*/

// ----------------------------------------------------------------------------
// AdjustConfidence -- recalibrate the per-pixel confidence-map so that it predicts
// "will this depth survive DenseFuseDepthMaps" instead of mere photometric NCC.
//
// WHY: the confMap produced by depth estimation is photometric only (conf = 1 - score,
// score = 1 - NCC). NCC is high wherever a patch correlates -- including repetitive
// texture, specular highlights and occlusion edges where a WRONG depth still matches well
// -- and low on correct but textureless surfaces. So the raw confidence is a poor predictor
// of what actually matters downstream: whether a depth is kept by fusion. DenseFuseDepthMaps
// is the gold standard for that (a depth survives only if other views geometrically confirm
// its 3D point), but its global flood-fill cannot be cheaply evaluated per pixel. This routine
// reproduces fusion's keep/drop decision LOCALLY and cheaply, for every pixel.
//
// Two independent, complementary sources of evidence are combined per reference pixel:
//
//  (A) MULTI-VIEW confirmation count K  -- one-hop, O(neighbors), the inter-map evidence:
//      back-project the pixel to its 3D point X, project X into each neighbor depth-map and
//      test the neighbor's own estimate against the EXACT four DenseFuse gates --
//        G1 depth-similarity, G2 forward-backward reprojection, G3 normal agreement,
//        G4 neighbor min-confidence. Every neighbor passing all four is a genuine confirmation
//      (++K) and contributes its confidence to Pconf. K is thus a faithful per-pixel proxy for
//      "how many views would confirm this point during fusion".
//
//  (B) INTRA-MAP geometric prior pGeo (ComputeIntraMapPrior, once per map, O(pixels)):
//      how well the pixel fits the local surface defined by its own 3x3 neighborhood (small
//      relative depth differences + coherent normals). A pixel on a smooth, self-consistent
//      surface scores high even with NO neighbor confirmation; an isolated depth spike (the
//      typical outlier) breaks local depth/normal coherence and scores ~0.
//
// The two are merged into a calibrated confidence in [0,1] with NO hard cliff (a confirmed or
// locally-coherent pixel is never zeroed):
//      gate        = 1 - exp(-(K + kPrior*pGeo)/tau)   soft analogue of "nMinViewsFuse>=2";
//                                                       pGeo acts as a fractional virtual view
//      posterior   = (s*pGeo + Pconf)/(s + Pconf + lambda*V)  Beta posterior mean, prior = s
//                                                       pseudo-obs; V (Task 15, GT-recalibrated
//                                                       default lambda=2, Task 17) is a count of
//                                                       free-space-violation neighbors -- G1
//                                                       failures where the neighbor's OWN depth is
//                                                       well BEHIND ours, i.e. its ray passes
//                                                       through our point -- diluting the posterior
//                                                       (lambda=0 restores the pre-Task-15 exact no-op)
//      photoFactor = w0 + (1-w0)*confPhoto             retain a photometric floor
//      conf        = clamp(posterior * gate * photoFactor, 0, 1)
//      if K>=1:  conf = max(conf, fConfFloor*confPhoto) anti-cascade floor (see below)
//
// GT-RECALIBRATED SHIPPED DEFAULTS (Task 17, gt_bench/SWEEP_GT.md): the confirmation count K is
// the fractional Ksoft of the SOFT-gate path (bConfSoftGates default 1 -- continuous weights +
// edge-aware bilinear neighbor sampling, see the TWO MODES note below), the FSV term is active
// (lambda = fConfViolationWeight = 2, margin = fConfViolationMargin = 2), and the posterior shape is
// s=2 / tau=1.5 / w0(photoFloor)=0.7 / floor=0.03. Pooled real-GT ROC-AUC 0.9463 -> 0.9598.
//
// HOW THIS KEEPS INLIERS AND DROPS OUTLIERS:
//  * Inlier seen by MANY views: K large -> gate->1 and posterior->~1 -> high confidence.
//  * Inlier seen by FEW views (precisely fusion's false-negatives): even K=1 gives gate~=0.73,
//    and the anti-cascade floor guarantees conf >= 0.5*confPhoto, so it is NOT thresholded away.
//  * Inlier seen by NO view but lying on a coherent surface (K=0, pGeo high): gate~=0.24 keeps
//    it ALIVE just around the fusion gate (judged in the LENIENT regime) rather than zeroing it.
//    This is the "valid even without much neighbor evidence, because it continues the local
//    surface" case -- especially valuable on thin / grazing / scene-boundary geometry, and on
//    mono-like smooth surfaces that lack texture for strong NCC.
//  * Photometric floater (high NCC, geometrically wrong): no view confirms it (K=0) AND it does
//    not fit any local surface (pGeo~0), so BOTH gate and posterior collapse; the high NCC alone
//    (photoFactor) cannot rescue it -> confidence -> ~0. Geometry, not photometry, decides.
//
// ANTI-CASCADE: the subsequent REAL fusion also gates neighbors on min-confidence (its G4), so
// zeroing a confirmed pixel here would remove it as a confirming neighbor for other pixels and
// erode the cloud. The K>=1 floor keeps any genuinely confirmed pixel above the fusion gate.
//
// COST: O(pixels x neighbors), a single lookup per neighbor -- no full neighbor-map reprojection
// and no extra full-resolution maps in RAM (unlike the removed Merrell-style implementation).
//
// TWO MODES / WHICH TO USE (Task 13 A/B decision, gt_bench/AB_INTEGRATED.md):
//  * STANDALONE (--postprocess-dmaps 4): this sweep runs as its own phase over MINF(nMaxThreads,
//    nImages) CPU threads (up to 30 here). This is the DEFAULT / RECOMMENDED path for pipelines that
//    consume the adjusted confidence.
//  * INTEGRATED ("Estimate Confidence = 1", the AdjustConfidence(DepthData&) overload below): the
//    SAME sweep runs inline as an epilogue of the last geometric-consistency iteration, so it reuses
//    the neighbor depth/normal/conf already loaded for that iteration (no separate phase, no dmap
//    reload). Confidence QUALITY is equivalent (|dROC| <= ~0.003 vs standalone on 4 of 5 bench
//    scenes; up to -0.013 only on the hardest 15-view scene, where the estimation-time neighbor set
//    diverges from the fusion-time set). BUT it is sized for GPU dispatch: only nDenseWorkers (==
//    nPatchMatchCUDAInstances, default 4) CPU workers run the sweep, vs up to 30 standalone. Because
//    the sweep is the same expensive O(pixels x neighbors) work, squeezing it through 4 workers makes
//    integrated a TOTAL-time win only on small / few-image scenes (where folding it into estimation
//    amortizes the standalone phase's fixed per-phase overhead) and a net LOSS on large-image scenes
//    (courtyard/office/facade: +8..+64s). Raising nPatchMatchCUDAInstances does NOT fix this -- that
//    same knob also raises GPU-dispatch concurrency for every geometric iteration, and oversubscribing
//    the single GPU cancels the small sweep saving (measured). A safe fix (decouple the epilogue's CPU
//    pool from the GPU pool -- MORE per-view workers only for the last iteration, still NO threads
//    inside this sweep) is a dispatch restructuring, deferred to Task 14; even then its best case is a
//    TIE with standalone (identical sweep work, identical dmap I/O). Use integrated for confidence-only
//    consumers (e.g. TSDF) that want to skip a separate phase or avoid dmaps on disk, especially on
//    small scenes; otherwise prefer standalone.
// ----------------------------------------------------------------------------
// accumulates the pure AdjustConfidence compute time (intra-map prior + multi-view confirmation loop)
// across the multithreaded adjust phase; reset and reported by the dispatch in ComputeDepthMaps;
// the second counter isolates the ComputeIntraMapPrior share so the prior and the confirmation
// sweep can be tracked separately across the speed work (confirmation rewritten first, prior later)
static std::atomic<int64_t> g_confAdjustComputeNS(0);
static std::atomic<int64_t> g_confPriorComputeNS(0);

// phase-lifetime depth-map cache for the adjust-confidence phase (Task 10): the adjust phase runs
// one worker per reference image, each pulling up to 8 neighbors, so a naive per-reference
// IncRef/DecRef (the pre-Task-10 design) re-reads any shared neighbor from disk up to ~9x. A single
// DMapCache instance (mirroring FuseDepthMaps' usage) shared across the whole phase lets a neighbor
// loaded for one reference stay resident for the next. DMapCache::UseImage briefly releases its own
// internal lock while performing the (potentially slow) disk Load(), so two worker threads racing to
// cache the SAME not-yet-loaded image concurrently could both pass the "is it empty" check and both
// call Load() on the same DepthData. Closing that race with ONE global mutex around every UseImage()
// call was tried first and measured ~2x SLOWER on courtyard (30 threads for only 38 images means most
// UseImage() calls are cheap cache-HITS that don't need any extra locking beyond DMapCache's own
// internal mutex -- serializing those too just adds futex/scheduler overhead across many threads).
// Reusing each DepthData's own (already thread-safe, already unused now that IncRef/DecRef is gone
// from this phase) CriticalSection as a PER-IMAGE lock gives the same correctness -- only two threads
// racing to load the SAME still-empty image ever contend -- with none of the false contention between
// threads touching different images.
static DMapCache* g_pAdjustDMapCache(NULL);

// AdjustConfidence's multi-view confirmation loop projects every reference pixel into every selected
// neighbor depth-map (and back) using a FIXED pair of cameras (ref, neighbor) -- only the per-pixel
// depth changes. Precomputing the composed single-precision linear maps below ONCE PER NEIGHBOR (not
// per pixel) turns each gate into a single 3x3 float matrix-vector product, replacing the double-
// precision TransformPointI2W/TransformPointW2C/ProjectPointP round trips previously repeated for
// every (pixel, neighbor) pair. Derivation (K's third row is always (0,0,1), see Camera.h):
//   camX = Rn*(Xworld-Cn), Xworld = Rr^T*Kr^-1*(u*d,v*d,d)+Cr  =>  Kn*camX = A*(u*d,v*d,d) + b
//   (fwd-bwd) Xn = Rn^T*Kn^-1*(un*dn,vn*dn,dn)+Cn, ref-projected = Kr*Rr*(Xn-Cr) = Ai*(...) + bi
// reused verbatim by later confirmation-loop rewrites (fusion, inlier labeling).
struct NeighborProj {
	Matrix3x3f A;    // Kn*Rn*Rr^T*Kr^-1 : ref (u*d,v*d,d) h-coords -> nbr h-coords (q.z = nbr cam depth)
	Point3f    b;    // Kn*Rn*(Cr-Cn)
	Matrix3x3f Ai;   // Kr*Rr*Rn^T*Kn^-1 : nbr (u*d,v*d,d) h-coords -> ref h-coords (qr.z = ref cam depth)
	Point3f    bi;   // Kr*Rr*(Cn-Cr)
	Matrix3x3f Rrel; // Rn*Rr^T : rotates a ref-camera-space normal directly into the nbr camera space
	const DepthMap* depthMap;
	const ConfidenceMap* confMap;
	const NormalMap* normalMap;
};

// forward decl -- shared tail of both AdjustConfidence overloads below, defined after them (Task 12)
static bool AdjustConfidenceSweep(DepthMapsData& depthMapsData, DepthData& depthDataRef,
	CLISTDEF0(NeighborProj)& neighborProjs, size_t nRequestedNeighbors, bool bDeferSwap);

// Task 16 (opt-in, OPTDENSE::bConfSoftGates): validity-aware bilinear neighbor-depth sample used by
// AdjustConfidenceSweep's soft-gate path in place of the hard path's nearest-neighbor ROUND2INT
// lookup. Returns false (caller falls back to the nearest sample) when any of the 4 taps is
// outside the map / invalid, OR when the 4 taps are not mutually depth-similar -- i.e. NEVER
// interpolates across a depth discontinuity (a foreground/background edge), which would otherwise
// synthesize a fictitious in-between depth at exactly the pixels where soft gating matters most.
static inline bool SampleDepthBilinear(const DepthMap& dm, float px, float py,
                                       float thDepthDiff, Depth& d) {
	const int x0=FLOOR2INT(px), y0=FLOOR2INT(py);
	if (x0 < 0 || y0 < 0 || x0+1 >= dm.width() || y0+1 >= dm.height()) return false;
	const Depth d00=dm(y0,x0), d01=dm(y0,x0+1), d10=dm(y0+1,x0), d11=dm(y0+1,x0+1);
	if (d00<=0 || d01<=0 || d10<=0 || d11<=0) return false;   // caller falls back to nearest
	const Depth dmin(MINF(MINF(d00,d01),MINF(d10,d11))), dmax(MAXF(MAXF(d00,d01),MAXF(d10,d11)));
	if (!IsDepthSimilar(dmin, dmax, thDepthDiff)) return false; // never interpolate across an edge
	const float wx=px-(float)x0, wy=py-(float)y0;
	d = (d00*(1.f-wx)+d01*wx)*(1.f-wy) + (d10*(1.f-wx)+d11*wx)*wy;
	return true;
}

bool DepthMapsData::AdjustConfidence(DepthData& depthDataRef, const IIndexArr& idxNeighbors)
{
	ASSERT(depthDataRef.IsValid() && !depthDataRef.IsEmpty() && !idxNeighbors.empty());
	const Camera& cameraRef = depthDataRef.GetView().camera;

	// precompute the fused single-precision ref->neighbor (and back) projection for every valid
	// neighbor once per reference map (see NeighborProj); the double-precision composition (K/R/C)
	// happens here only, O(neighbors), not O(pixels x neighbors); neighbor data comes from the
	// shared arrDepthData[] (this is the standalone --postprocess-dmaps 4 phase's phase-lifetime
	// DMapCache -- see AdjustConfidenceSweep below for why that forces the deferred confMapAdjusted
	// swap instead of writing confMap directly)
	CLISTDEF0(NeighborProj) neighborProjs(0, idxNeighbors.size());
	{
		const Matrix3x3 invKr(cameraRef.GetInvK());
		for (IIndex idxN: idxNeighbors) {
			const DepthData& depthDataN = arrDepthData[idxN];
			if (depthDataN.IsEmpty())
				continue;
			const Camera& cameraN = depthDataN.GetView().camera;
			const Matrix3x3 invKn(cameraN.GetInvK());
			const Matrix3x3 Rrel(cameraN.R*cameraRef.R.t()); // ref-cam -> nbr-cam rotation
			NeighborProj& np = neighborProjs.AddEmpty();
			np.A = Matrix3x3f(cameraN.K*Rrel*invKr);
			np.b = Point3f(cameraN.K*cameraN.R*(cameraRef.C-cameraN.C));
			np.Ai = Matrix3x3f(cameraRef.K*Rrel.t()*invKn);
			np.bi = Point3f(cameraRef.K*cameraRef.R*(cameraN.C-cameraRef.C));
			np.Rrel = Matrix3x3f(Rrel);
			np.depthMap = &depthDataN.depthMap;
			np.confMap = &depthDataN.confMap;
			np.normalMap = &depthDataN.normalMap;
		}
	}
	return AdjustConfidenceSweep(*this, depthDataRef, neighborProjs, idxNeighbors.size(), /*bDeferSwap=*/true);
} // AdjustConfidence
/*----------------------------------------------------------------*/

// Task 12: integrated fusion-faithful confidence -- epilogue of the LAST geometric-consistency
// iteration (see the EVT_SAVEDEPTHMAP call site in DenseReconstructionEstimate), reusing THIS
// reference's own depthDataRef.images[] (index 0 is the reference itself, skipped below) instead of
// indexing the shared arrDepthData[] the standalone overload above uses. Each neighbor ViewData's
// normalMap/confMap was populated by InitViews ONLY when loadDepthMaps==2 (last geometric-
// consistency iteration + OPTDENSE::bEstimateConfidence, see InitViews and its call site) -- the
// SAME disk read already needed for this iteration's geometric-consistency depth scoring, just
// decoding a few more fields from it, so this costs no extra neighbor load. A neighbor whose
// depth-map failed to load this iteration (view.depthMap empty) is skipped, exactly like an
// IsEmpty() neighbor is skipped in the standalone overload.
//
// cameraDepthMap (not camera) is used for each neighbor's pose: camera is the (possibly rescaled)
// photometric-matching camera, while cameraDepthMap is the pose at the RESOLUTION the neighbor's
// depth/normal/conf arrays were actually loaded at (see ViewData::Init, which builds the
// geometric-consistency Tl/Tm/Tr/Tn transforms from cameraDepthMap for exactly this reason).
bool DepthMapsData::AdjustConfidence(DepthData& depthDataRef)
{
	ASSERT(depthDataRef.IsValid() && !depthDataRef.IsEmpty());
	const Camera& cameraRef = depthDataRef.GetView().camera;

	CLISTDEF0(NeighborProj) neighborProjs(0, depthDataRef.images.empty() ? 0 : depthDataRef.images.size()-1);
	{
		const Matrix3x3 invKr(cameraRef.GetInvK());
		for (IIndex i=1; i<depthDataRef.images.size(); ++i) {
			const DepthData::ViewData& viewN = depthDataRef.images[i];
			if (viewN.depthMap.empty())
				continue; // this neighbor's depth failed to load this iteration (see InitViews)
			const Camera& cameraN = viewN.cameraDepthMap;
			const Matrix3x3 invKn(cameraN.GetInvK());
			const Matrix3x3 Rrel(cameraN.R*cameraRef.R.t()); // ref-cam -> nbr-cam rotation
			NeighborProj& np = neighborProjs.AddEmpty();
			np.A = Matrix3x3f(cameraN.K*Rrel*invKr);
			np.b = Point3f(cameraN.K*cameraN.R*(cameraRef.C-cameraN.C));
			np.Ai = Matrix3x3f(cameraRef.K*Rrel.t()*invKn);
			np.bi = Point3f(cameraRef.K*cameraRef.R*(cameraN.C-cameraRef.C));
			np.Rrel = Matrix3x3f(Rrel);
			np.depthMap = &viewN.depthMap;
			np.confMap = &viewN.confMap;
			np.normalMap = &viewN.normalMap;
		}
	}
	// bDeferSwap=false: neighbor data above is a private, disk-snapshotted copy loaded just for
	// this reference's own iteration -- never the shared, live arrDepthData[] state another
	// concurrently-estimating view could be reading -- so confMap can be written immediately
	return AdjustConfidenceSweep(*this, depthDataRef, neighborProjs, neighborProjs.size(), /*bDeferSwap=*/false);
} // AdjustConfidence (integrated)
/*----------------------------------------------------------------*/

#ifdef _USE_CUDA
// Task 14: GPU counterpart of the integrated AdjustConfidence(DepthData&) above -- the SAME neighbor
// build (this reference's private depthDataRef.images[] loaded by InitViews' loadDepthMaps==2), but
// the per-pixel intra-map prior + one-hop confirmation sweep run on the GPU (ConfidenceCUDA.cu's
// RunConfidenceCUDA) instead of the CPU AdjustConfidenceSweep. The neighbor depth/normal/conf are
// already resident in host memory from this iteration's geometric-consistency scoring, so this costs
// no extra disk read; the launcher uploads them, runs the two kernels, and downloads the recalibrated
// confidence, written to depthDataRef.confMap in place (private copies -> no deferred swap). Returns
// false on any CUDA error (contiguity/allocation/launch) so the caller falls back to the CPU sweep.
bool DepthMapsData::AdjustConfidenceCUDA(DepthData& depthDataRef)
{
	ASSERT(depthDataRef.IsValid() && !depthDataRef.IsEmpty());
	const DepthMap& depthMapRef = depthDataRef.depthMap;
	const NormalMap& normalMapRef = depthDataRef.normalMap;
	const ConfidenceMap& confMapRef = depthDataRef.confMap;
	const int W(depthMapRef.cols), H(depthMapRef.rows);
	if (W <= 0 || H <= 0 || confMapRef.size() != depthMapRef.size())
		return false;
	// the launcher indexes maps as contiguous row-major float buffers; bail to the CPU sweep otherwise
	if (!depthMapRef.isContinuous() || !confMapRef.isContinuous() ||
		(!normalMapRef.empty() && !normalMapRef.isContinuous()))
		return false;
	const Camera& cameraRef = depthDataRef.GetView().camera;

	std::vector<MVS::CUDA::ConfNeighborHost> hn;
	hn.reserve(depthDataRef.images.empty() ? 0 : depthDataRef.images.size()-1);
	const Matrix3x3 invKr(cameraRef.GetInvK());
	for (IIndex i=1; i<depthDataRef.images.size(); ++i) {
		const DepthData::ViewData& viewN = depthDataRef.images[i];
		if (viewN.depthMap.empty())
			continue; // this neighbor's depth failed to load this iteration (see InitViews)
		if (!viewN.depthMap.isContinuous() ||
			(!viewN.confMap.empty() && !viewN.confMap.isContinuous()) ||
			(!viewN.normalMap.empty() && !viewN.normalMap.isContinuous()))
			return false;
		const Camera& cameraN = viewN.cameraDepthMap;
		const Matrix3x3 invKn(cameraN.GetInvK());
		const Matrix3x3 Rrel(cameraN.R*cameraRef.R.t()); // ref-cam -> nbr-cam rotation
		const Matrix3x3f A(cameraN.K*Rrel*invKr);
		const Point3f b(cameraN.K*cameraN.R*(cameraRef.C-cameraN.C));
		const Matrix3x3f Ai(cameraRef.K*Rrel.t()*invKn);
		const Point3f bi(cameraRef.K*cameraRef.R*(cameraN.C-cameraRef.C));
		const Matrix3x3f Rrelf(Rrel);
		MVS::CUDA::ConfNeighborHost d;
		for (int r=0;r<3;++r) for (int c=0;c<3;++c) { d.A[r*3+c]=A(r,c); d.Ai[r*3+c]=Ai(r,c); d.Rrel[r*3+c]=Rrelf(r,c); }
		d.b[0]=b.x; d.b[1]=b.y; d.b[2]=b.z;
		d.bi[0]=bi.x; d.bi[1]=bi.y; d.bi[2]=bi.z;
		d.depth = viewN.depthMap.ptr<float>();
		d.conf = viewN.confMap.empty() ? NULL : viewN.confMap.ptr<float>();
		d.normal = viewN.normalMap.empty() ? NULL : viewN.normalMap.ptr<float>();
		d.width = viewN.depthMap.cols; d.height = viewN.depthMap.rows;
		hn.push_back(d);
	}

	// single-precision parameter snapshot, identical to AdjustConfidenceSweep's setup
	ConfRefine::Params p;
	p.minConfidence = 1.f - OPTDENSE::fNCCThresholdKeep;
	p.thReproj = OPTDENSE::fDepthReprojectionErrorThreshold;
	p.thDepth = OPTDENSE::fDepthDiffThreshold;
	p.normalError = COS(FD2R(OPTDENSE::fNormalDiffThreshold));
	p.s = OPTDENSE::fConfPriorStrength;
	p.tau = MAXF(OPTDENSE::fConfConfirmTau, 0.01f);
	p.kPrior = OPTDENSE::fConfPriorGate;
	p.w0 = OPTDENSE::fConfPhotoFloor;
	p.confFloor = OPTDENSE::fConfFloor;
	p.lambdaViol = OPTDENSE::fConfViolationWeight;
	p.violMargin = OPTDENSE::fConfViolationMargin;
	p.epsConf = MAXF(0.5f*p.minConfidence, 1e-6f);

	const Matrix3x3f Kf(cameraRef.K);
	ConfidenceMap newConfMap(depthMapRef.size());
	const std::chrono::steady_clock::time_point t0(std::chrono::steady_clock::now());
	const bool ok(MVS::CUDA::RunConfidenceCUDA(W, H,
		depthMapRef.ptr<float>(), normalMapRef.empty() ? NULL : normalMapRef.ptr<float>(), confMapRef.ptr<float>(),
		Kf(0,0), Kf(1,1), Kf(0,2), Kf(1,2), OPTDENSE::fNormalDiffThreshold,
		hn.data(), (int)hn.size(), p, OPTDENSE::bConfSoftGates, OPTDENSE::bConfPriorNormalCoherence,
		newConfMap.ptr<float>()));
	g_confAdjustComputeNS.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now() - t0).count(), std::memory_order_relaxed);
	if (!ok)
		return false;
	depthDataRef.confMap = std::move(newConfMap);
	return true;
} // AdjustConfidenceCUDA
/*----------------------------------------------------------------*/
#endif // _USE_CUDA

// core confidence-recalibration sweep shared by both AdjustConfidence overloads above (Task 9-11
// standalone postprocess phase and Task 12 integrated last-geometric-iteration epilogue); identical
// math either way -- the callers differ only in how neighborProjs is built (see above) and in
// bDeferSwap:
//  - bDeferSwap=true (standalone): neighbor confMaps read by neighborProjs are the LIVE, shared
//    arrDepthData[] confMap of other references, which other concurrently-adjusting references may
//    still be reading as one of THEIR neighbors -- so the recalibrated map is parked in
//    confMapAdjusted and only swapped into confMap by the EVT_ADJUSTDEPTHMAP handler once the
//    whole-phase semaphore barrier confirms every reference has finished reading (see
//    DenseReconstructionFilter)
//  - bDeferSwap=false (integrated): neighbor data is a private, disk-snapshotted copy loaded just
//    for this reference's own geometric-consistency iteration -- it is never the shared, live
//    arrDepthData[] state another concurrently-estimating view could be reading, so there is
//    nothing to protect against and the recalibrated map is written into confMap immediately
static bool AdjustConfidenceSweep(DepthMapsData& depthMapsData, DepthData& depthDataRef,
	CLISTDEF0(NeighborProj)& neighborProjs, size_t nRequestedNeighbors, bool bDeferSwap)
{
	TD_TIMER_STARTD();

	ASSERT(depthDataRef.IsValid() && !depthDataRef.IsEmpty());
	ASSERT(depthDataRef.confMap.size() == depthDataRef.depthMap.size());
	const DepthData::ViewData& imageRef = depthDataRef.GetView();
	const DepthMap& depthMapRef = depthDataRef.depthMap;
	const NormalMap& normalMapRef = depthDataRef.normalMap;
	const ConfidenceMap& confMapRef = depthDataRef.confMap;
	const bool bHasRefNormal(!normalMapRef.empty());

	// confirmation gates reused verbatim from DenseFuseDepthMaps
	const float normalError(COS(FD2R(OPTDENSE::fNormalDiffThreshold)));
	const float minConfidence(1.f - OPTDENSE::fNCCThresholdKeep);
	const float thReproj(OPTDENSE::fDepthReprojectionErrorThreshold);
	const float maxReprojErrorSq(SQUARE(thReproj));
	const Depth thDepth(OPTDENSE::fDepthDiffThreshold);
	// Task 16 (opt-in): soft continuous-weight gates + bilinear neighbor-depth sampling in place of
	// the hard nearest-neighbor pass/fail path; default OFF keeps this function bit-identical to
	// Task 15 (see the branch in the neighbor sweep below)
	const bool bSoftGates(OPTDENSE::bConfSoftGates);
	// soft GATE-4 transition half-width: the soft path replaces the hard cN<minConfidence rejection
	// with a smoothstep centered on minConfidence, so a low-confidence neighbor down-weights BOTH
	// Ksoft and Pconf (a modest fraction of minConfidence; MAXF guards fNCCThresholdKeep==1 =>
	// minConfidence==0; Task 17 can sweep this)
	const float epsConf(MAXF(0.5f*minConfidence, 1e-6f));
	// posterior/gate shape parameters
	const float s(OPTDENSE::fConfPriorStrength);
	const float tau(MAXF(OPTDENSE::fConfConfirmTau, 0.01f));
	const float kPrior(OPTDENSE::fConfPriorGate);
	const float w0(OPTDENSE::fConfPhotoFloor);
	const float confFloor(OPTDENSE::fConfFloor);
	// Task 15: free-space-violation (FSV) negative evidence (opt-in, lambda==0 is an exact no-op)
	const float lambdaViol(OPTDENSE::fConfViolationWeight);
	const float violMargin(OPTDENSE::fConfViolationMargin);
	// single-precision parameter snapshot shared with the CUDA confidence kernel; the final
	// per-pixel posterior below is routed through ConfRefine::Posterior so CPU and GPU evaluate the
	// identical closed form (the CPU path stays byte-identical -- ConfRefine::CRexp uses double std::exp)
	ConfRefine::Params crp;
	crp.s = s; crp.tau = tau; crp.kPrior = kPrior; crp.w0 = w0; crp.confFloor = confFloor;
	crp.minConfidence = minConfidence; crp.thReproj = thReproj; crp.thDepth = (float)thDepth;
	crp.normalError = normalError; crp.lambdaViol = lambdaViol; crp.violMargin = violMargin;
	crp.epsConf = epsConf;

	// intra-map geometric prior (once per map); bParallel=false -- this call runs inside one of
	// nMaxThreads already-parallel pool-worker threads (see GetIntraMapPrior's declaration comment)
	const std::chrono::steady_clock::time_point timeAdjustStart(std::chrono::steady_clock::now());
	const ConfidenceMap& priorMap = depthMapsData.GetIntraMapPrior(depthDataRef, false);
	g_confPriorComputeNS.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now() - timeAdjustStart).count(), std::memory_order_relaxed);

	ConfidenceMap newConfMap(depthMapRef.size());
	// optional per-pixel feature export for offline parameter tuning (recompute newConf from these in Python)
	const bool bFeat(OPTDENSE::bExportConfFeatures);
	TImage<uint16_t> featK, featV;
	TImage<float> featPconf, featPrior, featPhoto;
	if (bFeat) {
		featK.create(depthMapRef.size()); featK.memset(0);
		featV.create(depthMapRef.size()); featV.memset(0);
		featPconf.create(depthMapRef.size()); featPconf.memset(0);
		featPrior.create(depthMapRef.size()); featPrior.memset(0);
		featPhoto.create(depthMapRef.size()); featPhoto.memset(0);
	}

	// one-hop multi-view confirmation, swept NEIGHBOR-OUTER / pixel-inner: the sweep is memory-
	// latency-bound (per confirmation it randomly samples the neighbor's depth/conf/normal maps),
	// so visiting one neighbor at a time keeps a single neighbor's maps hot in cache and lets the
	// smooth ref->neighbor warp drive the hardware prefetcher, instead of interleaving up to 8
	// neighbor working sets per pixel. Per pixel the neighbors are still accumulated in the exact
	// neighborProjs order (pass k adds neighbor k for every pixel), so K and the float Pconf sum
	// are BIT-IDENTICAL to the pixel-outer/neighbor-inner order.
	TImage<uint16_t> countMap(depthMapRef.size());
	countMap.memset(0);
	ConfidenceMap pconfMap(depthMapRef.size());
	pconfMap.memset(0);
	// Task 16 (opt-in): fractional confirmation-weight accumulator Ksoft, used INSTEAD OF countMap
	// when bSoftGates is set (allocated only then, so the hard/default path pays nothing extra)
	ConfidenceMap countMapSoft;
	if (bSoftGates) {
		countMapSoft.create(depthMapRef.size());
		countMapSoft.memset(0);
	}
	// Task 15: per-pixel free-space-violation count V, accumulated the same neighbor-outer/pixel-inner
	// way as countMap/pconfMap above (so it stays bit-identical regardless of neighbor visiting order)
	TImage<uint16_t> violMap(depthMapRef.size());
	violMap.memset(0);
	// per-row projection buffers shared by every neighbor pass (see stage A below)
	std::vector<int> xRow, yRow;
	std::vector<float> zRow;
	for (const NeighborProj& np: neighborProjs) {
		const bool bNormalGate(bHasRefNormal && !np.normalMap->empty());
		const bool bHasConf(!np.confMap->empty());
		const DepthMap& depthMapN(*np.depthMap);
		// unpack the fused transforms into scalar locals: the generic cv::Matx operator* spills
		// temporaries to the stack in this hot loop, while plain float locals stay enregistered;
		// the accumulation order below matches cv::Matx (left-to-right dot product) so the
		// results are bit-identical
		const float A00(np.A(0,0)), A01(np.A(0,1)), A02(np.A(0,2));
		const float A10(np.A(1,0)), A11(np.A(1,1)), A12(np.A(1,2));
		const float A20(np.A(2,0)), A21(np.A(2,1)), A22(np.A(2,2));
		const float b0(np.b.x), b1(np.b.y), b2(np.b.z);
		const float Ai00(np.Ai(0,0)), Ai01(np.Ai(0,1)), Ai02(np.Ai(0,2));
		const float Ai10(np.Ai(1,0)), Ai11(np.Ai(1,1)), Ai12(np.Ai(1,2));
		const float Ai20(np.Ai(2,0)), Ai21(np.Ai(2,1)), Ai22(np.Ai(2,2));
		const float bi0(np.bi.x), bi1(np.bi.y), bi2(np.bi.z);
		// the sweep is bound by the projection stage (transform+divide+round+bounds, executed for
		// EVERY pixel x neighbor candidate, hits and misses alike -- measured at ~80% of the sweep):
		// stage A projects a WHOLE ROW into flat buffers, 4 columns per iteration with SSE where
		// available (the ops are IEEE-identical per lane and in the same order as the scalar code,
		// and ROUND2INT(float) == (int)floor(x+.5f) is reproduced exactly by _mm_floor_ps, so the
		// buffered results are bit-identical to the scalar path), prefetching the neighbor-depth
		// (and normal) cache lines of surviving lanes; stage B then walks the row applying the
		// gates. zRow[c] <= 0 marks "skip column c" (invalid depth / behind camera / outside map).
		const int cols(depthMapRef.cols);
		xRow.resize(cols); yRow.resize(cols); zRow.resize(cols);
		for (int r=0; r<depthMapRef.rows; ++r) {
			const float rd((float)r);
			const Depth* const rowD(depthMapRef.ptr<const Depth>(r));
			// ---- stage A: project the row ----
			const auto Project = [&](int c) { // scalar fallback/tail
				float& z(zRow[c]);
				const Depth depthRef(rowD[c]);
				if (depthRef <= 0) {
					z = 0;
					return;
				}
				// ref pixel in homogeneous (u*d,v*d,d) form
				const float ud((float)c*depthRef), vd(rd*depthRef);
				const float qz(A20*ud + A21*vd + A22*depthRef + b2);
				if (qz <= 0) {
					z = 0; // point behind the neighbor camera (guard before homogeneous divide)
					return;
				}
				const float qx(A00*ud + A01*vd + A02*depthRef + b0);
				const float qy(A10*ud + A11*vd + A12*depthRef + b1);
				const ImageRef x(ROUND2INT(qx/qz), ROUND2INT(qy/qz));
				if (!depthMapN.isInside(x)) {
					z = 0;
					return;
				}
				#ifdef _USE_SSE
				_mm_prefetch((const char*)&depthMapN(x), _MM_HINT_T0);
				if (bNormalGate)
					_mm_prefetch((const char*)&(*np.normalMap)(x), _MM_HINT_T0);
				#endif
				xRow[c] = x.x; yRow[c] = x.y;
				z = qz;
			};
			int c = 0;
			#ifdef _USE_SSE
			{
			const __m128 vZero(_mm_setzero_ps()), vHalf(_mm_set1_ps(.5f)), vRd(_mm_set1_ps(rd));
			const __m128 vA00(_mm_set1_ps(A00)), vA01(_mm_set1_ps(A01)), vA02(_mm_set1_ps(A02)), vB0(_mm_set1_ps(b0));
			const __m128 vA10(_mm_set1_ps(A10)), vA11(_mm_set1_ps(A11)), vA12(_mm_set1_ps(A12)), vB1(_mm_set1_ps(b1));
			const __m128 vA20(_mm_set1_ps(A20)), vA21(_mm_set1_ps(A21)), vA22(_mm_set1_ps(A22)), vB2(_mm_set1_ps(b2));
			const __m128i vNegOne(_mm_set1_epi32(-1));
			const __m128i vW(_mm_set1_epi32(depthMapN.cols)), vH(_mm_set1_epi32(depthMapN.rows));
			const __m128 vRamp(_mm_setr_ps(0.f, 1.f, 2.f, 3.f));
			for (; c+4<=cols; c+=4) {
				const __m128 d4(_mm_loadu_ps(rowD+c));
				// (float)(c+k) computed as (float)c + k: exact for any image-sized c
				const __m128 c4(_mm_add_ps(_mm_set1_ps((float)c), vRamp));
				const __m128 ud(_mm_mul_ps(c4, d4)), vd(_mm_mul_ps(vRd, d4));
				// same evaluation tree as the scalar path: ((A*ud + A*vd) + A*d) + b
				const __m128 qz4(_mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(vA20, ud), _mm_mul_ps(vA21, vd)), _mm_mul_ps(vA22, d4)), vB2));
				const __m128 qx4(_mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(vA00, ud), _mm_mul_ps(vA01, vd)), _mm_mul_ps(vA02, d4)), vB0));
				const __m128 qy4(_mm_add_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(vA10, ud), _mm_mul_ps(vA11, vd)), _mm_mul_ps(vA12, d4)), vB1));
				// ROUND2INT(float) == (int)floor(x+.5f); garbage lanes (d<=0/qz<=0) produce
				// harmless garbage ints (cvtt of NaN/Inf -> INT_MIN) that the bounds test rejects
				const __m128i xi(_mm_cvttps_epi32(_mm_floor_ps(_mm_add_ps(_mm_div_ps(qx4, qz4), vHalf))));
				const __m128i yi(_mm_cvttps_epi32(_mm_floor_ps(_mm_add_ps(_mm_div_ps(qy4, qz4), vHalf))));
				const __m128i inX(_mm_and_si128(_mm_cmpgt_epi32(xi, vNegOne), _mm_cmpgt_epi32(vW, xi)));
				const __m128i inY(_mm_and_si128(_mm_cmpgt_epi32(yi, vNegOne), _mm_cmpgt_epi32(vH, yi)));
				const __m128 okF(_mm_and_ps(_mm_cmpgt_ps(d4, vZero), _mm_cmpgt_ps(qz4, vZero)));
				const __m128i ok(_mm_and_si128(_mm_castps_si128(okF), _mm_and_si128(inX, inY)));
				_mm_storeu_si128((__m128i*)&xRow[c], xi);
				_mm_storeu_si128((__m128i*)&yRow[c], yi);
				// masked-out lanes store +0.0 == the "skip" sentinel
				_mm_storeu_ps(&zRow[c], _mm_and_ps(qz4, _mm_castsi128_ps(ok)));
				const int m(_mm_movemask_ps(_mm_castsi128_ps(ok)));
				for (int k=0; k<4; ++k) { // prefetch the neighbor-map lines of surviving lanes
					if ((m&(1<<k)) == 0)
						continue;
					const ImageRef x(xRow[c+k], yRow[c+k]);
					_mm_prefetch((const char*)&depthMapN(x), _MM_HINT_T0);
					if (bNormalGate)
						_mm_prefetch((const char*)&(*np.normalMap)(x), _MM_HINT_T0);
				}
			}
			}
			#endif
			for (; c<cols; ++c)
				Project(c);
			// ---- stage B: gates ----
			// Task 16: branch ONCE per row on the knob (not per pixel) so the default/hard path below
			// is a byte-for-byte copy of Task 15's loop -- no float ever touches countMap/K on this path
			if (!bSoftGates) {
			for (int c=0; c<cols; ++c) {
				const float qz(zRow[c]);
				if (qz <= 0)
					continue; // invalid ref depth / behind camera / outside neighbor map
				const ImageRef x(xRow[c], yRow[c]);
				const Depth dN(depthMapN(x));
				if (dN <= 0)
					continue;
				// GATE 1: depth similarity (neighbor measured depth as denominator, as in DenseFuse)
				// GATE 2: forward-backward reprojection (back-project the neighbor pixel, reproject into reference)
				// both tests are evaluated before a SINGLE combined branch: their outcomes alternate
				// spatially (half-agreeing surfaces) and cost a mispredict each when tested separately,
				// while evaluating both back-to-back lets G2's matrix product overlap G1's divide; the
				// tests themselves and their values are unchanged, and the positive <=/&-forms reject
				// NaN/Inf exactly like the original early-out sequence did
				const bool gDepth(IsDepthSimilar(dN, (Depth)qz, thDepth));
				// Task 15: when GATE 1 fails, classify why -- the neighbor's own measured depth dN
				// lies on the SAME ray as our point's depth qz (both expressed in the neighbor's
				// camera frame), so the two cases are distinguishable:
				//  * dN >> qz (well BEHIND, past the margin): if our point were real, the neighbor's
				//    camera would have stopped its ray AT us, not at a surface further away -- so the
				//    neighbor's ray passes THROUGH our point out to a real surface behind it. This is
				//    negative evidence (a free-space violation) even though no positive gate fired.
				//  * dN << qz: the neighbor sees something much CLOSER than our point instead -- we
				//    are simply occluded in that view, which says nothing about our point: neutral.
				// only a violation well past the G1 similarity boundary counts, not a borderline miss.
				if (!gDepth && dN > qz*(1.f + violMargin*thDepth))
					++violMap(r,c);
				const float un((float)x.x*dN), vn((float)x.y*dN);
				const float qrz(Ai20*un + Ai21*vn + Ai22*dN + bi2);
				const float qrx(Ai00*un + Ai01*vn + Ai02*dN + bi0);
				const float qry(Ai10*un + Ai11*vn + Ai12*dN + bi1);
				const float du(qrx/qrz - (float)c), dv(qry/qrz - rd);
				const bool gReproj(qrz > 0 && du*du+dv*dv <= maxReprojErrorSq);
				if (!(gDepth & gReproj))
					continue;
				// GATE 3: normal agreement (only if both normal-maps are available); the ref normal is
				// rotated lazily into THIS neighbor's camera frame (np.Rrel) only when reached here,
				// the neighbor normal is used raw (already in its own camera space, no rotation needed)
				if (bNormalGate) {
					const Point3f nRefN(np.Rrel*normalMapRef(r,c));
					if (nRefN.dot((*np.normalMap)(x)) < normalError)
						continue;
				}
				// the neighbor's confidence, read only now that the geometric gates passed (the
				// map is immutable during the sweep, so the deferred load returns the same value
				// the original early load did -- but only gate-3 survivors pay for it)
				const float cN(bHasConf ? (*np.confMap)(x) : 1.f);
				// GATE 4: neighbor min-confidence
				if (cN < minConfidence)
					continue;
				// all gates passed: this neighbor confirms the reference depth
				++countMap(r,c);
				pconfMap(r,c) += cN;
			}
			} else {
			// ---- stage B (Task 16, opt-in): soft continuous-weight gates ----
			// Free-space-violation bookkeeping (Task 15) stays HARD and is evaluated on the nearest
			// sample exactly as the hard path above -- V is deliberately NOT softened. The confirmation
			// gates G1/G2/G3 are replaced by continuous weights wD/wR/wN in [0,1]; their product w
			// contributes a FRACTIONAL confirmation (Ksoft += w) and a w-weighted confidence
			// (Pconf += w*cN) instead of the hard ++countMap/+=cN. GATE 4 (neighbor min-confidence) is
			// intentionally not reapplied as a hard cutoff here (see task-16-report.md): Pconf is
			// already cN-weighted, so a low-confidence neighbor is automatically down-weighted there,
			// even though it can still add to Ksoft's purely-geometric confirmation count.
			for (int c=0; c<cols; ++c) {
				const float qz(zRow[c]);
				if (qz <= 0)
					continue; // invalid ref depth / behind camera / outside neighbor map
				const ImageRef x(xRow[c], yRow[c]);
				const Depth dNNearest(depthMapN(x));
				if (dNNearest <= 0)
					continue;
				// Task 15 free-space-violation logic, HARD, on the nearest sample -- unchanged
				const bool gDepthNearest(IsDepthSimilar(dNNearest, (Depth)qz, thDepth));
				if (!gDepthNearest && dNNearest > qz*(1.f + violMargin*thDepth))
					++violMap(r,c);
				// bilinear (edge-aware) neighbor-depth sample at the continuous projected location;
				// recompute the continuous (px,py) here -- stage A only kept the rounded xRow/yRow
				// used by the hard path -- cheap, and only paid on this opt-in branch
				const Depth depthRef(rowD[c]);
				const float ud((float)c*depthRef), vd(rd*depthRef);
				const float qx(A00*ud + A01*vd + A02*depthRef + b0);
				const float qy(A10*ud + A11*vd + A12*depthRef + b1);
				const float px(qx/qz), py(qy/qz);
				Depth dN;
				if (!SampleDepthBilinear(depthMapN, px, py, thDepth, dN))
					dN = dNNearest; // straddles a depth edge, or out of bounds: fall back to nearest
				// GATE 1 (soft): Gaussian depth agreement, same relative-depth convention as IsDepthSimilar
				const float wD(expf(-SQUARE((qz-dN)/(0.5f*thDepth*qz))));
				// GATE 2 (soft): forward-backward reprojection residual, reusing G2's exact formula
				// (same neighbor pixel location x) with the (possibly bilinear) dN swapped in
				const float un((float)x.x*dN), vn((float)x.y*dN);
				const float qrz(Ai20*un + Ai21*vn + Ai22*dN + bi2);
				float wR(0.f);
				if (qrz > 0) {
					const float qrx(Ai00*un + Ai01*vn + Ai02*dN + bi0);
					const float qry(Ai10*un + Ai11*vn + Ai12*dN + bi1);
					const float du(qrx/qrz - (float)c), dv(qry/qrz - rd);
					wR = expf(-(du*du+dv*dv)/SQUARE(0.5f*thReproj));
				}
				// GATE 3 (soft): same cosine as the hard G3; neutral (1) when no normal maps available
				float wN(1.f);
				if (bNormalGate) {
					const Point3f nRefN(np.Rrel*normalMapRef(r,c));
					wN = MAXF(0.f, nRefN.dot((*np.normalMap)(x)));
				}
				// GATE 4 (soft): smoothstep(cN; minConfidence-epsConf, minConfidence+epsConf) mirrors
				// the hard path's cN<minConfidence rejection. Folding wC into w makes a low-confidence
				// neighbor down-weight BOTH Ksoft and Pconf, so a geometrically-strong but low-conf
				// neighbor can no longer push Ksoft>=1 and trip the anti-cascade floor Task 15 reserves
				// for genuinely min-conf-passing pixels. cN==1 when no conf map => wC==1 (matches the
				// hard path treating a missing conf as passing G4).
				const float cN(bHasConf ? (*np.confMap)(x) : 1.f);
				const float tC(CLAMP((cN - (minConfidence - epsConf))*(0.5f/epsConf), 0.f, 1.f));
				const float wC(tC*tC*(3.f - 2.f*tC));
				const float w(wD*wR*wN*wC);
				if (w <= 0.05f)
					continue; // negligible joint agreement: does not contribute
				countMapSoft(r,c) += w;
				pconfMap(r,c) += w*cN;
			}
			}
		}
	}
	#if TD_VERBOSE != TD_VERBOSE_OFF
	unsigned nProcessed(0), nDiscarded(0);
	#endif
	for (int r=0; r<depthMapRef.rows; ++r) {
		for (int c=0; c<depthMapRef.cols; ++c) {
			const Depth depthRef(depthMapRef(r,c));
			if (depthRef <= 0) {
				newConfMap(r,c) = 0;
				continue;
			}
			#if TD_VERBOSE != TD_VERBOSE_OFF
			++nProcessed;
			#endif
			const float confPhoto(confMapRef(r,c));
			const float pGeo(priorMap(r,c));
			const unsigned K(countMap(r,c));
			// Task 16 (opt-in): the fractional soft-gate confirmation weight Ksoft is consumed in place
			// of the hard integer K everywhere below when bSoftGates is set; when off, Kf is bit-identical
			// to the previous (float)K cast (ternary short-circuits, so countMapSoft -- unallocated when
			// off -- is never touched), so the formulas and their outputs are unchanged (Task 15 parity)
			const float Kf(bSoftGates ? countMapSoft(r,c) : (float)K);
			const float Pconf(pconfMap(r,c));
			const unsigned V(violMap(r,c));
			// map (prior, confirmation count, photometric conf) to a calibrated [0,1] confidence with no
			// hard cliff -- the gate (soft nMinViews), Beta posterior mean with FSV dilution, photometric
			// floor, and anti-cascade floor now live in ConfRefine::Posterior (shared with the GPU kernel)
			const float conf(ConfRefine::Posterior(confPhoto, pGeo, Kf, Pconf, (float)V, crp));
			newConfMap(r,c) = conf;
			if (bFeat) {
				// Task 16: in soft mode cfeatK stores Ksoft SCALED by 1000 (fixed-point, ~3 decimal digits)
				// instead of the raw integer count, so Task 17's offline tuning can recover the fractional
				// value -- consumers must check bConfSoftGates and divide by 1000.f accordingly
				featK(r,c) = bSoftGates ? (uint16_t)MINF((unsigned)ROUND2INT(Kf*1000.f), 65535u)
				                        : (uint16_t)MINF(K, 65535u);
				featV(r,c) = (uint16_t)MINF(V, 65535u);
				featPconf(r,c) = Pconf;
				featPrior(r,c) = pGeo;
				featPhoto(r,c) = confPhoto;
			}
			#if TD_VERBOSE != TD_VERBOSE_OFF
			if (conf < minConfidence)
				++nDiscarded;
			#endif
		}
	}
	g_confAdjustComputeNS.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now() - timeAdjustStart).count(), std::memory_order_relaxed);
	if (bFeat) {
		const IIndex id(imageRef.GetID());
		SaveRawMap(ComposeDepthFilePath(id, "cfeatK"), featK, 2);
		SaveRawMap(ComposeDepthFilePath(id, "cfeatV"), featV, 2);
		SaveRawMap(ComposeDepthFilePath(id, "cfeatPconf"), featPconf, 4);
		SaveRawMap(ComposeDepthFilePath(id, "cfeatPrior"), featPrior, 4);
		SaveRawMap(ComposeDepthFilePath(id, "cfeatPhoto"), featPhoto, 4);
		// tuning/export mode: leave the source depth-map untouched (confidence is recomputed offline);
		// returning false skips the EVT_ADJUSTDEPTHMAP merge that would otherwise re-save the depth-map
		DEBUG("Confidence-map %3u features exported (%s)", id, TD_TIMER_GET_FMT().c_str());
		return false;
	}
	if (bDeferSwap) {
		// store the recalibrated confidence-map in memory; the EVT_ADJUSTDEPTHMAP handler swaps it
		// into confMap only after every reference using this image as a neighbor has finished
		// reading the PRE-adjustment confMap (guaranteed by the data.sem barrier -- see
		// DenseReconstructionFilter)
		depthDataRef.confMapAdjusted = std::move(newConfMap);
	} else {
		// integrated path: no concurrent reader of this reference's PRE-adjustment confMap to
		// protect (see the bDeferSwap comment above AdjustConfidenceSweep) -- swap in directly
		depthDataRef.confMap = std::move(newConfMap);
	}

	DEBUG("Confidence-map %3u adjusted using %u other images: %u/%u depths below fusion confidence (%s)",
		imageRef.GetID(), (unsigned)nRequestedNeighbors, nDiscarded, nProcessed, TD_TIMER_GET_FMT().c_str());
	return true;
} // AdjustConfidenceSweep
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


// fuse all depth-maps by simply projecting them in a 3D point-cloud
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
		DEBUG_ULTIMATE("Depth-map for reference image %3u merged using %u depth-maps: %u new points (%s)",
			idxImage, depthData.images.size()-1, pointcloud.points.size()-nNumPointsPrev, TD_TIMER_GET_FMT().c_str());
		progress.display(idxImage+1);
	}
	GET_LOGCONSOLE().Play();
	progress.close();

	DEBUG_EXTRA("Depth-maps merged: %u depth-maps, %u depths, %u points (%d%%) (%s)",
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

// fuse all valid depth-maps in the same 3D point-cloud;
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
				ASSERT(ISEQUAL(norm(normal), 1.f, 1e-2f), "Norm = ", norm(normal));
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
					const auto [pt, depthProjB] = imageDataB.camera.ProjectPointP(point);
					if (depthProjB <= 0)
						continue;
					const ImageRef xB(ROUND2INT(pt));
					DepthMap& depthMapB = depthDataB.depthMap;
					if (!depthMapB.isInside(xB))
						continue;
					Depth& depthB = depthMapB(xB);
					if (depthB == 0)
						continue;
					uint32_t& idxPointB = arrDepthIdx[idxImageB](xB);
					if (idxPointB != NO_ID)
						continue;
					if (IsDepthSimilar(depthProjB, depthB, OPTDENSE::fDepthDiffThreshold)) {
						// check if normals agree
						const PointCloud::Normal normalB(!depthData.normalMap.empty() ? Cast<Normal::Type>(imageDataB.camera.R.t() * Cast<REAL>(depthDataB.normalMap(xB))) : Normal(0, 0, -1));
						ASSERT(ISEQUAL(norm(normalB), 1.f, 1e-2f), "Norm = ", norm(normalB));
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
					if (depthProjB < depthB) {
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
		DEBUG_ULTIMATE("Depth-map for reference image %3u fused using %u depth-maps: %u new points, %u/%u cached images (%s)",
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

	DEBUG_EXTRA("Depth-maps fused and filtered: %u depth-maps, %u depths, %u points (%d%%), %.2f hits in %.2f cached (%s)",
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


// fuse all valid depth-maps in the same 3D point-cloud;
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
	// Task 18: free-space-violation (FSV) guard -- the set of DISTINCT view IDs that, for the point
	// currently being accumulated, were rejected by the join gate below BECAUSE their own measured
	// depth lies well behind the point (same classification as Task 15's AdjustConfidenceSweep
	// violMap). Deduplicated the same way fusedViews dedups observing views (InsertSortUnique), so V
	// counts "how many distinct views see behind this point" and is per-view-bounded like Task 15's
	// V -- the flood-fill can reach one neighbor view via several parent paths before its useMask is
	// set, so a plain per-probe counter would over-count a single view. Only consulted at the
	// keep-rule for points RESCUED by virtualSupport (see OPTDENSE::nFuseViolationMax); reset
	// alongside fusedViews et al.
	PointCloud::ViewArr fusedViolViews;
	// Task 19: opt-in second-chance fusion pass -- STEP 1 finding (read the rejection path first):
	// inside FusePointImpl, useMask.set(x) (see below) fires UNCONDITIONALLY for every pixel that
	// locally passes the per-pixel gates (bounds/depth/confidence, and -- when fuseDepth>0 -- the
	// depth/reprojection/normal join gate), BEFORE control ever returns to the keep-rule check at the
	// bottom of this (i,j) loop. So a seed's contributing pixels are marked consumed the instant they
	// join the flood-fill, regardless of whether the assembled group later passes or fails the
	// keep-rule; a discarded group's pixels are just as permanently "used" as a kept group's. The only
	// pixels that stay unmarked (useMask==0) are ones that were PROBED as a would-be join and FAILED
	// the depth/reprojection/normal test (the `return` before useMask.set at the join-gate checks) --
	// but by construction every valid (depth>0, confident) pixel is eventually visited either as a
	// successful join (marked) or as its own seed at fuseDepth==0 (which has no join gate and always
	// marks it), so by the end of the main fusion pass EVERY valid pixel belongs to exactly one
	// already-attempted group. There is no leftover, never-tried pixel pool for a later pass to pick
	// up. Consequently a second-chance pass CANNOT recover anything by re-running FusePoint on a
	// recorded seed pixel -- that pixel (and everything it successfully joined) is already useMask==1
	// from pass 1, so a literal replay is a guaranteed no-op. The only thing that can work -- and the
	// only thing this implementation does -- is to snapshot the ALREADY-ASSEMBLED group's point/views/
	// weights/normal/color at the moment pass 1 would have discarded it, then re-apply a relaxed
	// keep-rule to that frozen snapshot after the main pass. This touches no pixel a second time (no
	// double-fuse: the recorded group's pixels were never part of any kept point) and needs no further
	// useMask interaction at all.
	struct SecondChanceCandidate {
		PointCloud::Point point;
		PointCloud::ViewArr views;
		PointCloud::WeightArr weights;
		PointCloud::Normal normal;
		PointCloud::Color color;
		unsigned numPixels;
	};
	// CLISTDEF2IDX (useConstruct=2), NOT CLISTDEF0IDX: SecondChanceCandidate owns non-trivial members
	// (ViewArr/WeightArr, themselves heap-owning cLists), which need their constructor/destructor
	// actually invoked on every AddEmpty()/RemoveLast()/grow -- CLISTDEF0IDX skips construction
	// entirely (meant for POD types) and corrupts the nested lists' _vector/_size/_vectorSize.
	CLISTDEF2IDX(SecondChanceCandidate, IIndex) secondChanceCandidates;
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
				const auto [pt, depthProj] = image.camera.ProjectPointP(refPoint);
				// check if depth agrees with current depth
				ASSERT(depthProj > Depth(0) || !IsDepthSimilar(depth, depthProj, OPTDENSE::fDepthDiffThreshold));
				if (!IsDepthSimilar(depth, depthProj, OPTDENSE::fDepthDiffThreshold)) {
					// Task 18: classify why the join gate failed, SAME free-space-violation (FSV)
					// test as Task 15's AdjustConfidenceSweep (violMap): `depth` is this view's OWN
					// measured depth at x, `depthProj` is our accumulating point reprojected into
					// this view -- if this view's ray sees a surface well BEHIND our point instead
					// of agreeing with it, that is negative evidence the point is real (only
					// meaningful when depthProj>0, i.e. the point is actually in front of this view).
					// Record the DISTINCT view ID (InsertSortUnique) so V counts violating views, not
					// probes -- one view can be re-reached before its useMask is set.
					if (depthProj > Depth(0) && depth > depthProj * (1.f + OPTDENSE::fConfViolationMargin * OPTDENSE::fDepthDiffThreshold))
						fusedViolViews.InsertSortUnique(ID);
					return;
				}
				// check reprojection error of the reference point in the current view
				const Point2f diff(pt - Cast<float>(x));
				if (normSq(diff) > maxReprojErrorSq)
					return;
				// check if normals agree
				normal = image.camera.R.t() * Cast<REAL>(depthData.normalMap(x));
				ASSERT(ISEQUAL(norm(normal), 1.f, 1e-2f), "Norm = ", norm(normal));
				if (refNormal.dot(normal) < normalError)
					return;
			} else {
				normal = image.camera.R.t() * Cast<REAL>(depthData.normalMap(x));
				ASSERT(ISEQUAL(norm(normal), 1.f, 1e-2f), "Norm = ", norm(normal));
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
				const ImageRef nextx(ROUND2INT(std::get<0>(nextDepthData.GetCamera().ProjectPointP(X))));
				FusePointImpl(nextID, nextx, fuseDepth, FusePointImpl);
			}
		};
		lambda(ID, x, fuseDepth, lambda);
	};
	// optional intra-map geometric prior of the reference depth-map (recomputed per image): it grants
	// fractional "virtual" view/pixel support so that an inlier lying on a locally coherent surface but
	// confirmed by too few views/pixels is still kept (same prior used by AdjustConfidence); empty when off
	const bool bUsePrior(OPTDENSE::fFusePriorWeight > 0);
	// Task 19: the second-chance record gate also needs priorMap(i,j), independent of fFusePriorWeight
	// (bUsePrior), since a user may want the recovery pass without the w3 rescue's virtual support
	const bool bNeedPrior(bUsePrior || OPTDENSE::bFuseSecondChance);
	// loop over each depth-map
	IIndex numDMapsFused = 0;
	while (true) {
		TD_TIMER_STARTD();
		// find the best depth-map to fuse next as the one with the most neighbors already in cache
		const auto [idxImage, numImageNeighborsInCache, numImagesInCache] = FetchBestNextDMapIndex(arrDepthData, cacheDMaps, fusedDMaps);
		if (idxImage == NO_ID)
			break; // no more depth-maps to fuse (only invalid depth-maps left)
		totalNumImageNeighborsInCache += numImageNeighborsInCache;
		totalNumImagesInCache += numImagesInCache;
		++numDMapsFused;
		// fuse depth-map
		cacheDMaps.UseImage(idxImage);
		cacheDMaps.SkipMemoryCheckIdxImage(idxImage);
		DepthData& depthData(arrDepthData[idxImage]); // non-const: GetIntraMapPrior caches into depthData.priorMap
		ASSERT(depthData.GetView().GetLocalID(scene.images) == idxImage);
		ASSERT(!depthData.IsEmpty());
		if (bEstimateNormal && depthData.normalMap.empty())
			EstimateNormalMaps();
		if (bNeedPrior)
			GetIntraMapPrior(depthData, true); // depth+normal coherence of every seed pixel, O(pixels); bParallel=true (serial caller, idle cores)
		// make sure all neighbors are cached
		neighbors.Memset(0);
		neighbors[idxImage] = true;
		IIndex numNeighbors(0);
		ASSERT(!depthData.images.empty() && !depthData.neighbors.empty());
		#ifdef DENSE_USE_OPENMP
		bool bAbort(false);
		#pragma omp parallel for
		for (int64_t i=0; i<(int64_t)depthData.neighbors.size(); ++i) {
			#pragma omp flush (bAbort)
			if (bAbort)
				continue;
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
			UseMask& useMask = arrUseMask[neighbor.ID];
			if (!useMask.empty())
				continue;
			useMask.create(depthDataB.depthMap.size());
			useMask.memset(0);
			if (++numNeighbors >= OPTDENSE::nMaxViewsFuse) {
				#ifdef DENSE_USE_OPENMP
				bAbort = true;
				#pragma omp flush (bAbort)
				#else
				break;
				#endif
			}
		}
		ASSERT(!depthData.IsEmpty());
		MAYBEUNUSED const Image& imageData = *depthData.images.front().pImageData;
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
				// the intra-map prior of the seed pixel (j,i) contributes fractional virtual support: a seed
				// lying on a locally coherent surface partially satisfies the view and pixel minimums, so an
				// inlier that fusion would otherwise drop for want of cross-view confirmation is recovered.
				// the prior measures agreement with same-depth-map neighbors, which nMinPixelsFuse explicitly
				// counts; the !fusedViews.empty() guard ensures at least the real seed exists so the prior can
				// never fabricate a point out of nothing (when disabled, virtualSupport==0 => identical output)
				const float virtualSupport(bUsePrior ? OPTDENSE::fFusePriorWeight * depthData.priorMap(i,j) : 0.f);
				bool bKept(false);
				if (!fusedViews.empty() &&
					(float)fusedPoints[0].size() + virtualSupport >= (float)OPTDENSE::nMinPixelsFuse &&
					(float)fusedViews.size() + virtualSupport >= (float)nMinViewsFuse) {
					// Task 18: a point that passes ONLY thanks to virtualSupport (i.e. would have
					// FAILED the keep-rule at virtualSupport==0) is "rescued"; nFuseViolationMax
					// additionally requires such a point to be seen behind by at most that many
					// DISTINCT views (fusedViolViews, populated above by the join gate). A NON-rescued
					// point (already meets both thresholds on real support alone) is NEVER subject
					// to this guard. nFuseViolationMax<0 (default) disables the guard entirely, so
					// this whole check is skipped/inert and the output is byte-identical.
					const bool rescued = fusedPoints[0].size() < OPTDENSE::nMinPixelsFuse ||
										  fusedViews.size() < nMinViewsFuse;
					if (!rescued || OPTDENSE::nFuseViolationMax < 0 || fusedViolViews.size() <= (unsigned)OPTDENSE::nFuseViolationMax) {
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
							pointcloud.colors.emplace_back((fusedColor/static_cast<float>(fusedPoints[0].size())).cast<uint8_t>());
						bKept = true;
					}
				}
				// Task 19: this seed failed the keep-rule above (bKept==false). Every pixel it touched
				// (the seed itself, plus everything it successfully joined) is already permanently
				// marked used in useMask -- see the rejection-path note above the FusePoint lambda --
				// so nothing can be recovered by revisiting this pixel later. The only recoverable
				// thing is the already-assembled group itself: if it already has >=2 real observing
				// views and the seed sits on a well-supported intra-map prior surface, snapshot it now
				// (point/views/weights/normal/color are about to be cleared below) for the opt-in
				// second-chance pass after the main fusion loop. Gated on the knob first
				// (short-circuit) so priorMap is never touched, and nothing is recorded, when
				// bFuseSecondChance is off (default) -- keeps the no-op trivial.
				if (OPTDENSE::bFuseSecondChance && !bKept && fusedViews.size() >= 2 && depthData.priorMap(i,j) >= 0.5f) {
					SecondChanceCandidate& cand = secondChanceCandidates.AddEmpty();
					cand.point = PointCloud::Point(
						fusedPoints[0].GetMedian(),
						fusedPoints[1].GetMedian(),
						fusedPoints[2].GetMedian()
					);
					cand.views = fusedViews;
					ASSERT(fusedViews.size() == fusedWeights.size());
					for (float weight: fusedWeights)
						cand.weights.push_back(weight);
					cand.normal = bEstimateNormal ? Cast<float>(normalized(fusedNormal)) : PointCloud::Normal(0,0,0);
					cand.color = bEstimateColor ? (fusedColor/static_cast<float>(fusedPoints[0].size())).cast<uint8_t>() : PointCloud::Color(0,0,0);
					cand.numPixels = (unsigned)fusedPoints[0].size();
					// strict V==0 requirement (Task-18 counting, independent of nFuseViolationMax): a
					// candidate seen from behind by even one distinct view is never eligible, so drop
					// it here rather than carrying dead weight into the second-chance pass below
					if (!fusedViolViews.empty())
						secondChanceCandidates.RemoveLast();
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
					fusedViolViews.clear();
				}
			}
		}
		fusedDMaps[idxImage] = true;
		ASSERT(pointcloud.points.size() == pointcloud.pointViews.size() && pointcloud.points.size() == pointcloud.pointWeights.size());
		DEBUG_ULTIMATE("Depth-map for reference image %3u fused using %u depth-maps: %u new points, %u/%u cached images (%s)",
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
	// Task 19: opt-in second-chance pass. Re-test every snapshot recorded above (already-assembled,
	// prior-supported, violation-free groups that failed the main keep-rule) with a relaxed pixel-count
	// minimum. This does NOT revisit any pixel or touch useMask -- per the STEP-1 rejection-path note
	// above the FusePoint lambda, by now every pixel is already permanently consumed one way or
	// another, so there is nothing left to gate on; it only re-decides, on the frozen snapshot, whether
	// the group deserves to become a point after all. secondChanceCandidates is always empty when
	// bFuseSecondChance is off (default), so this is then a no-op and output stays byte-identical.
	//
	// MEASURED (Task 19, ETH3D courtyard/office/delivery_area, deterministic dmaps held fixed):
	// with the CURRENT default fFusePriorWeight=3.0 (w3 rescue on), this recovers exactly ZERO points
	// on every scene tested, and it PROVABLY always will: the record gate above requires priorMap>=0.5,
	// so any surviving (V==0) recorded candidate had virtualSupport=fFusePriorWeight*priorMap>=1.5 at
	// the moment pass 1 rejected it on pixel-count alone, i.e. its REAL pixel count was
	// <nMinPixelsFuse-1.5=3.5 (<=3) -- strictly below the relaxed floor of nMinPixelsFuse-1=4 computed
	// below. So under the shipped default, the w3 rescue's own virtual support already guarantees no
	// clean discarded candidate can reach the relaxed floor: this pass is fully redundant with it.
	// Confirmed the mechanism itself is NOT broken: with fFusePriorWeight=0 (w3 off), courtyard
	// recovered 558299/4206407 candidates (+25.9% points), completeness rose at every ETH3D tolerance
	// tier, gross_outlier_frac moved only 0.202%->0.207% (+0.005pp, well inside the +0.05pp gate) --
	// i.e. it works exactly as designed, it is just currently redundant given fFusePriorWeight=3.0's
	// default. Hence default stays OFF (see task-19-report.md for the full A/B).
	if (OPTDENSE::bFuseSecondChance) {
		const unsigned nMinPixelsFuseRelaxed(MAXF((int)OPTDENSE::nMinPixelsFuse - 1, 3));
		unsigned numSecondChanceRecovered = 0;
		for (const SecondChanceCandidate& cand: secondChanceCandidates) {
			// V==0 was already enforced at record time (dead-on-arrival candidates were dropped
			// immediately, since fusedViolViews cannot change after the seed's traversal completes);
			// only the relaxed pixel/view minimums remain to be checked here
			if (cand.numPixels < nMinPixelsFuseRelaxed || cand.views.size() < nMinViewsFuse)
				continue;
			pointcloud.points.emplace_back(cand.point);
			PointCloud::WeightArr& weights = pointcloud.pointWeights.AddEmpty();
			weights = cand.weights;
			pointcloud.pointViews.emplace_back(cand.views);
			if (bEstimateNormal)
				pointcloud.normals.emplace_back(cand.normal);
			if (bEstimateColor)
				pointcloud.colors.emplace_back(cand.color);
			++numSecondChanceRecovered;
		}
		DEBUG_EXTRA("Second-chance fusion: %u/%u recorded prior-supported seeds recovered",
			numSecondChanceRecovered, secondChanceCandidates.size());
	}
	if (!_bEstimateNormal)
		pointcloud.normals.Release();

	DEBUG_EXTRA("Depth-maps dense fused and filtered: %u depth-maps, %u depths, %u points (%d%%), %.2f hits in %.2f cached (%s)",
		numDMapsFused, nDepths, pointcloud.points.size(), ROUND2INT((100.f*pointcloud.points.size())/nDepths),
		static_cast<double>(totalNumImageNeighborsInCache) / numDMapsFused,
		static_cast<double>(totalNumImagesInCache) / numDMapsFused, TD_TIMER_GET_FMT().c_str());
} // DenseFuseDepthMaps
/*----------------------------------------------------------------*/



// label every depth estimate as inlier/outlier by replaying the DenseFuseDepthMaps flood-fill with the
// confidence gate DISABLED (so the labels are pure geometry, independent of the confidence-map being evaluated);
// for each kept cluster all contributing pixels (across views) get the cluster label and its view-count support;
// writes per-image .flabel (uint8 FusionLabel) and .fsupport (uint16 #views) beside the .dmap files
void DepthMapsData::LabelFusionInliers()
{
	TD_TIMER_STARTD();

	typedef SEACAVE::BitMatrix UseMask;
	typedef CLISTDEFIDX(UseMask,IIndex) UseMaskArr;
	typedef std::pair<IIndex,ImageRef> Member;

	const unsigned nMinViewsFuse(MINF(OPTDENSE::nMinViewsFuse, arrDepthData.size()));
	const float normalError(COS(FD2R(OPTDENSE::fNormalDiffThreshold)));
	const float maxReprojErrorSq(SQUARE(OPTDENSE::fDepthReprojectionErrorThreshold));
	const IIndex numDMapsReserveFusion(10);
	UseMaskArr arrUseMask(arrDepthData.size());
	// per-image label/support maps (std::vector so the cv::Mat elements are properly constructed/destructed)
	std::vector<TImage<uint8_t>> arrLabelMap(arrDepthData.size());
	std::vector<TImage<uint16_t>> arrSupportMap(arrDepthData.size());
	const unsigned depthDataLoadFlags(HeaderDepthDataRaw::HAS_DEPTH | HeaderDepthDataRaw::HAS_NORMAL | HeaderDepthDataRaw::HAS_CONF);
	Util::Progress progress(_T("Labeled depth-maps"), arrDepthData.size());
	GET_LOGCONSOLE().Pause();
	BoolArr fusedDMaps(arrDepthData.size());
	fusedDMaps.Memset(0);
	DMapCache cacheDMaps(arrDepthData, depthDataLoadFlags, GetAvailableMemory(arrDepthData, fusedDMaps, numDMapsReserveFusion));
	BoolArr neighbors(arrDepthData.size());
	PointCloud::Point refPoint;
	PointCloud::Normal refNormal;
	std::vector<Member> fusedMembers;
	PointCloud::ViewArr fusedViews;
	const auto EnsureMaps = [&](IIndex ID) -> void {
		if (!arrUseMask[ID].empty())
			return;
		const cv::Size size(arrDepthData[ID].depthMap.size());
		arrUseMask[ID].create(size);
		arrUseMask[ID].memset(0);
		arrLabelMap[ID].create(size); arrLabelMap[ID].memset(LABEL_INVALID);
		arrSupportMap[ID].create(size); arrSupportMap[ID].memset(0);
	};
	const auto FusePoint = [&](IIndex ID, const ImageRef& x, unsigned fuseDepth) -> void {
		const auto lambda = [&](IIndex ID, const ImageRef& x, unsigned fuseDepth, const auto& FusePointImpl) -> void {
			const DepthData& depthData = arrDepthData[ID];
			if (!Image8U::isInside(x, depthData.size))
				return;
			const Depth depth = depthData.depthMap(x);
			if (depth <= Depth(0))
				return;
			UseMask& useMask = arrUseMask[ID];
			if (useMask(x))
				return;
			// NOTE: the confidence gate of DenseFuseDepthMaps is intentionally DISABLED here so that the
			// labels stay independent of the confidence-map we want to evaluate
			const DepthData::ViewData& image = depthData.GetView();
			PointCloud::Normal normal;
			if (fuseDepth > 0) {
				const auto [pt, depthProj] = image.camera.ProjectPointP(refPoint);
				if (!IsDepthSimilar(depth, depthProj, OPTDENSE::fDepthDiffThreshold))
					return;
				const Point2f diff(pt - Cast<float>(x));
				if (normSq(diff) > maxReprojErrorSq)
					return;
				normal = image.camera.R.t() * Cast<REAL>(depthData.normalMap(x));
				if (refNormal.dot(normal) < normalError)
					return;
			} else {
				normal = image.camera.R.t() * Cast<REAL>(depthData.normalMap(x));
			}
			useMask.set(x);
			const PointCloud::Point X(image.camera.TransformPointI2W(Point3(REAL(x.x), REAL(x.y), REAL(depth))));
			fusedMembers.emplace_back(ID, x);
			fusedViews.InsertSortUnique(ID);
			if (fuseDepth == 0) {
				refPoint = X;
				refNormal = normal;
			}
			if (++fuseDepth >= OPTDENSE::nMaxFuseDepth || fusedMembers.size() >= OPTDENSE::nMaxPointsFuse)
				return;
			for (const ViewScore& neighbor : image.pImageData->neighbors) {
				const IIndex nextID(neighbor.ID);
				if (!neighbors[nextID])
					continue;
				const DepthData& nextDepthData = arrDepthData[nextID];
				const ImageRef nextx(ROUND2INT(std::get<0>(nextDepthData.GetCamera().ProjectPointP(X))));
				FusePointImpl(nextID, nextx, fuseDepth, FusePointImpl);
			}
		};
		lambda(ID, x, fuseDepth, lambda);
	};
	// loop over each depth-map (best connected first), exactly as DenseFuseDepthMaps
	IIndex numDMapsFused = 0;
	while (true) {
		const auto [idxImage, numImageNeighborsInCache, numImagesInCache] = FetchBestNextDMapIndex(arrDepthData, cacheDMaps, fusedDMaps);
		(void)numImageNeighborsInCache; (void)numImagesInCache;
		if (idxImage == NO_ID)
			break;
		++numDMapsFused;
		cacheDMaps.UseImage(idxImage);
		cacheDMaps.SkipMemoryCheckIdxImage(idxImage);
		const DepthData& depthData(arrDepthData[idxImage]);
		ASSERT(!depthData.IsEmpty());
		if (depthData.normalMap.empty())
			EstimateNormalMaps();
		// make sure all neighbors are cached and have label/use maps
		neighbors.Memset(0);
		neighbors[idxImage] = true;
		IIndex numNeighbors(0);
		for (const ViewScore& neighbor: depthData.neighbors) {
			const DepthData& depthDataB(arrDepthData[neighbor.ID]);
			if (!depthDataB.IsValid())
				continue;
			cacheDMaps.UseImage(neighbor.ID);
			if (depthDataB.IsEmpty())
				continue;
			neighbors[neighbor.ID] = true;
			EnsureMaps(neighbor.ID);
			if (++numNeighbors >= OPTDENSE::nMaxViewsFuse)
				break;
		}
		EnsureMaps(idxImage);
		// try to label each depth estimate
		for (int i=0; i<depthData.size.height; ++i) {
			for (int j=0; j<depthData.size.width; ++j) {
				FusePoint(idxImage, ImageRef(j,i), 0);
				if (fusedMembers.empty())
					continue;
				const unsigned V(fusedViews.size());
				const unsigned P((unsigned)fusedMembers.size());
				uint8_t label;
				if (V >= nMinViewsFuse && P >= OPTDENSE::nMinPixelsFuse) {
					label = LABEL_CONFIDENT_INLIER;
				} else if (V >= 2) {
					label = LABEL_WEAK_INLIER;
				} else {
					// singleton cluster: outlier if a neighbor covers the seed (seen but not confirmed), else ambiguous
					bool seen(false);
					for (const ViewScore& neighbor: depthData.neighbors) {
						if (!neighbors[neighbor.ID])
							continue;
						const DepthData& nd(arrDepthData[neighbor.ID]);
						const auto [pt, dproj] = nd.GetCamera().ProjectPointP(refPoint);
						if (dproj <= 0)
							continue;
						const ImageRef nx(ROUND2INT(pt));
						if (nd.depthMap.isInside(nx) && nd.depthMap(nx) > 0) {
							seen = true;
							break;
						}
					}
					label = seen ? LABEL_OUTLIER : LABEL_AMBIGUOUS;
				}
				const uint16_t support((uint16_t)MINF(V, 65535u));
				for (const Member& m: fusedMembers) {
					arrLabelMap[m.first](m.second) = label;
					arrSupportMap[m.first](m.second) = support;
				}
				fusedMembers.clear();
				fusedViews.clear();
			}
		}
		fusedDMaps[idxImage] = true;
		progress.display(numDMapsFused);
		cacheDMaps.SkipMemoryCheckIdxImage();
		if (numDMapsFused % numDMapsReserveFusion == 0)
			cacheDMaps.SetMaxMemory(GetAvailableMemory(arrDepthData, fusedDMaps, numDMapsReserveFusion, cacheDMaps.GetUsedMemory()));
	}
	GET_LOGCONSOLE().Play();
	progress.close();
	cacheDMaps.ClearCache();

	// write the per-image label/support maps beside the depth-maps
	unsigned nWritten(0);
	FOREACH(i, arrDepthData) {
		if (arrLabelMap[i].empty())
			continue;
		const IIndex viewID(arrDepthData[i].GetView().GetID());
		if (!SaveRawMap(ComposeDepthFilePath(viewID, "flabel"), arrLabelMap[i], 1) ||
			!SaveRawMap(ComposeDepthFilePath(viewID, "fsupport"), arrSupportMap[i], 2)) {
			DEBUG("error: failed to write fusion labels for image %u", viewID);
		} else {
			++nWritten;
		}
	}
	arrUseMask.Release();
	DEBUG_EXTRA("Fusion inlier/outlier labels exported for %u depth-maps (%s)", nWritten, TD_TIMER_GET_FMT().c_str());
} // LabelFusionInliers
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

DenseDepthMapData::DenseDepthMapData(Scene& _scene, int _nFusionMode, float _fSampleMeshNeighbors) :
	scene(_scene), depthMaps(_scene), idxImage(0), sem(1), nEstimationGeometricIter(-1),
	nFusionMode(_nFusionMode), fSampleMeshNeighbors(_fSampleMeshNeighbors), nClosing(0), nDenseWorkers(2u)
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

bool Scene::DenseReconstruction(int nFusionMode, bool bCrop2ROI, float fBorderROI, float fSampleMeshNeighbors)
{
	DenseDepthMapData data(*this, nFusionMode, fSampleMeshNeighbors);

	// estimate depth-maps
	if (!ComputeDepthMaps(data))
		return false;

	// optionally export per-pixel fusion inlier/outlier labels for confidence evaluation
	// (independent of fusion, so it also runs in export-only mode |nFusionMode|==1)
	if (OPTDENSE::bExportFusionLabels)
		data.depthMaps.LabelFusionInliers();

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
	if (VERBOSITY_LEVEL > 2) {
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
		SampleMeshWithVisibility(static_cast<REAL>(data.fSampleMeshNeighbors));
		mesh.Release();
	}

	// if no geometry available, estimate neighbor views based on image pairs baseline
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
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (OPTDENSE::fWeightPointInsideROI > 0 && IsBounded()) {
			VERBOSE("Select neighbor views by weighting inside ROI points with %.2f", OPTDENSE::fWeightPointInsideROI);
		}
		#endif
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

	#if defined(_USE_CUDA) || defined(_USE_METAL)
	// One PatchMatch instance per worker thread; host-side prep (image upload,
	// depth-prior packing, result unpack) parallelizes across the worker pool while
	// the backend serializes the kernel launches as needed (CUDA via the cudaEvent_t
	// chain). The GPU backend (CUDA on Windows/Linux, Metal on Apple) is selected by
	// the shared --gpu-device param (-1 GPU, -2/cpu/empty CPU).
	if (!SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs) && data.nFusionMode >= 0) {
		const unsigned poolSize = (nMaxThreads > 1)
			? CLAMP(OPTDENSE::nPatchMatchCUDAInstances, 1u, nMaxThreads)
			: 1u;
		#ifdef _USE_CUDA
		const bool bAllocatedPool = data.depthMaps.AllocateCudaPool(poolSize);
		#else
		const bool bAllocatedPool = data.depthMaps.AllocateMetalPool(poolSize);
		#endif
		if (bAllocatedPool) {
			// raise the in-flight semaphore so all pool workers can run
			// EstimateDepthMap concurrently
			data.sem.Clear(poolSize);
			data.nDenseWorkers = poolSize;
			#ifdef _USE_CUDA
			VERBOSE("Using CUDA compute backend for depth-map estimation (%u workers)", poolSize);
			#else
			VERBOSE("Using Metal compute backend for depth-map estimation (%u workers)", poolSize);
			#endif
		}
	}
	#endif // _USE_CUDA || _USE_METAL

	// initialize the queue of images to be processed
	const int nOptimize(OPTDENSE::nOptimize);
	if (OPTDENSE::nEstimationGeometricIters && data.nFusionMode >= 0)
		OPTDENSE::nOptimize = 0;
	data.idxImage = 0;
	data.nClosing = 0;
	ASSERT(data.events.IsEmpty());
	data.events.AddEvent(new EVTProcessImage(0));
	// start working threads
	data.progress = new Util::Progress("Estimated depth-maps", data.images.GetSize());
	GET_LOGCONSOLE().Pause();
	if (nMaxThreads > 1) {
		// data.nDenseWorkers is set to the CUDA pool size (or kept at the
		// constructor default of 2 for the CPU path) before we get here.
		cList<SEACAVE::Thread> threads(data.nDenseWorkers);
		FOREACHPTR(pThread, threads)
			pThread->start(DenseReconstructionEstimateTmp, (void*)&data);
		FOREACHPTR(pThread, threads)
			pThread->join();
	} else {
		// single-thread execution
		DenseReconstructionEstimate((void*)&data);
	}
	GET_LOGCONSOLE().Play();
	// the balanced shutdown leaves the queue empty on success; anything left is a
	// genuine worker failure (e.g. a propagated EVTFail)
	if (!data.events.IsEmpty())
		return false;
	data.progress.Release();

	// Task 12: 'Estimate Confidence' only has a hook to fire from -- the epilogue of the LAST
	// geometric-consistency iteration (EVT_SAVEDEPTHMAP) -- when PatchMatch geometric-consistency
	// iterations actually run; without them the knob is silently a no-op, so warn once up front
	if (OPTDENSE::bEstimateConfidence && (data.nFusionMode < 0 || OPTDENSE::nEstimationGeometricIters == 0))
		VERBOSE("warning: 'Estimate Confidence' has no effect: it requires PatchMatch geometric-"
			"consistency iterations (nFusionMode>=0 && 'Estimation Geometric Iters'>0)");

	if (data.nFusionMode >= 0) {
		#ifdef _USE_CUDA
		if (!data.depthMaps.pmCUDAPool.empty() && OPTDENSE::nEstimationGeometricIters)
			data.depthMaps.ReinitCudaPoolForGeom();
		#endif // _USE_CUDA
		#ifdef _USE_METAL
		if (!data.depthMaps.pmMetalPool.empty() && OPTDENSE::nEstimationGeometricIters)
			data.depthMaps.ReinitMetalPoolForGeom();
		#endif // _USE_METAL
		while (++data.nEstimationGeometricIter < (int)OPTDENSE::nEstimationGeometricIters) {
			// initialize the queue of images to be geometric processed
			if (data.nEstimationGeometricIter+1 == (int)OPTDENSE::nEstimationGeometricIters)
				OPTDENSE::nOptimize = nOptimize;
			data.idxImage = 0;
			data.nClosing = 0;
			ASSERT(data.events.IsEmpty());
			data.events.AddEvent(new EVTProcessImage(0));
			// start working threads
			data.progress = new Util::Progress("Geometric-consistent estimated depth-maps", data.images.GetSize());
			GET_LOGCONSOLE().Pause();
			if (nMaxThreads > 1) {
				// same worker count as the depth-map phase
				cList<SEACAVE::Thread> threads(data.nDenseWorkers);
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

	// Task 12 double-adjust guard: if the integrated per-view confidence estimation already ran
	// (as an epilogue of the last geometric-consistency iteration, see EVT_SAVEDEPTHMAP above),
	// running the standalone postprocess adjust phase too would recalibrate an already-recalibrated
	// confMap a second time (compounding the posterior/gate/floor formula on its own output) --
	// warn and skip it instead of silently double-adjusting.
	// skip the standalone postprocess adjust phase when the integrated per-view confidence already ran
	// as the last-geometric-iteration epilogue (GPU when CUDA estimation + bEstimateConfidenceCUDA, or
	// the legacy integrated-CPU opt-in bEstimateConfidence) -- running both would recalibrate an
	// already-recalibrated confMap a second time. When estimation is on the CPU with the GPU path
	// disabled, the epilogue does nothing and this standalone phase is the confidence path.
	const bool bIntegratedConfRan((OPTDENSE::nOptimize & OPTDENSE::ADJUST_CONFIDENCE) != 0 &&
		data.nFusionMode >= 0 && OPTDENSE::nEstimationGeometricIters > 0 &&
		(OPTDENSE::bEstimateConfidence
		#ifdef _USE_CUDA
		|| (!data.depthMaps.pmCUDAPool.empty() && OPTDENSE::bEstimateConfidenceCUDA)
		#endif
		));
	if (bIntegratedConfRan) {
		VERBOSE("skipping the postprocess confidence-adjust phase (--postprocess-dmaps 4): the adaptive "
			"confidence was already recalibrated during the last geometric-consistency iteration");
	} else
	if ((OPTDENSE::nOptimize & OPTDENSE::ADJUST_CONFIDENCE) != 0) {
		TD_TIMER_STARTD();
		g_confAdjustComputeNS.store(0);
		g_confPriorComputeNS.store(0);
		// initialize the queue of depth-maps to be filtered
		data.sem.Clear();
		data.idxImage = data.images.GetSize();
		ASSERT(data.events.IsEmpty());
		FOREACH(i, data.images)
			data.events.AddEvent(new EVTFilterDepthMap(i));
		// phase-lifetime depth-map cache (Task 10): every image is read from disk at most once for
		// the whole phase instead of once per reference that uses it as a neighbor. The budget is
		// deliberately UNLIMITED (maxMemory=0, no eviction ever): full-scene residency is this
		// phase's design premise, and -- crucially -- DMapCache::EjectOldest() Release()s the LRU
		// image with no pin/in-use awareness, while a concurrent worker's confirmation sweep holds
		// raw pointers into its neighbors' depth/normal/conf maps (NeighborProj in AdjustConfidence)
		// for the whole sweep; a bounded budget under memory pressure would therefore be a
		// use-after-free, not a graceful degradation. Unlimited turns memory exhaustion into an
		// honest allocation failure instead of a silent UAF. The cost is made visible: estimated
		// peak logged right below (incl. the per-image in-memory adjusted-confidence side buffers,
		// which live outside the cache's accounting), actual disk reads + resident cache bytes
		// logged at phase end. Measured worst-case (BlendedMVS 5b7a3890, 375 views @768x576):
		// 3.7GB peak cache / 6.2GB peak process RSS -- comfortably bounded in practice.
		// Full load flags (15/all) are used so the EVT_ADJUSTDEPTHMAP re-save below round-trips
		// depthMap/normalMap/confMap/viewsMap exactly like the old per-reference IncRef(fileName) did.
		DMapCache cacheDMaps(data.depthMaps.arrDepthData, 15u/*all*/, 0/*unlimited -- see above*/);
		g_pAdjustDMapCache = &cacheDMaps;
		#if TD_VERBOSE != TD_VERBOSE_OFF
		{
			// estimated peak memory: per pixel 4B depth + 12B normal + 4B conf + 4B views (upper
			// bound; normal/views may be absent) resident in the cache, plus 4B for the
			// confMapAdjusted side buffer and 4B for the cached intra-map priorMap (Task 11,
			// GetIntraMapPrior) -- at the semaphore barrier every image's side buffers are live
			// simultaneously, so both belong in the estimate even though the cache can't see them
			size_t estPeakMemory(0);
			for (const DepthData& depthData: data.depthMaps.arrDepthData)
				if (depthData.IsValid())
					estPeakMemory += (size_t)depthData.size.area() * ((1/*depth*/+3/*normal*/+1/*conf*/+1/*confMapAdjusted*/+1/*priorMap*/)*4 + 4/*views*/);
			VERBOSE("Adjust-confidence phase: caching all %u depth-maps in memory, estimated peak %lluMB (incl. in-memory adjusted confidence + prior)",
				data.images.size(), (unsigned long long)(estPeakMemory>>20));
		}
		#endif
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
		const uint32_t numDMapReads(cacheDMaps.GetNumImageReads());
		// with the unlimited budget nothing is ever ejected, so the final resident size IS the peak
		const size_t peakCacheMemory(cacheDMaps.GetUsedMemory());
		cacheDMaps.ClearCache();
		g_pAdjustDMapCache = NULL;
		if (!data.events.IsEmpty())
			return false;
		data.progress.Release();
		VERBOSE("Confidence-maps adjusted: %u depth-maps (%s; %.3gs prior+confirmation compute, %.2fms/map avg; %.3gs prior / %.3gs confirmation; %u dmap disk reads via cache, %lluMB peak cache memory)",
			data.images.GetSize(), TD_TIMER_GET_FMT().c_str(),
			g_confAdjustComputeNS.load()/1e9, g_confAdjustComputeNS.load()/1e6/(double)MAXF(data.images.GetSize(),1u),
			g_confPriorComputeNS.load()/1e9, (g_confAdjustComputeNS.load()-g_confPriorComputeNS.load())/1e9,
			numDMapReads, (unsigned long long)(peakCacheMemory>>20));
	}
	return true;
} // ComputeDepthMaps
/*----------------------------------------------------------------*/

void* DenseReconstructionEstimateTmp(void* arg) {
	const DenseDepthMapData& dataThreads = *((const DenseDepthMapData*)arg);
	dataThreads.scene.DenseReconstructionEstimate(arg);
	return NULL;
}

// initialize the dense reconstruction with the sparse point-cloud
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
					// Work is exhausted. More than one worker can reach this branch
					// (each pulls a distinct safeInc'd index past the end), so don't
					// let every one of them broadcast: the first worker here (latch
					// 0->1) enqueues exactly one EVT_CLOSE per worker -- itself
					// included -- and every worker, including the ones in this branch,
					// then exits by consuming exactly one. The counts stay balanced,
					// so no orphaned EVT_CLOSE is left behind and a non-empty queue
					// after the join remains a reliable failure signal.
					if (Thread::safeInc(data.nClosing) == 1)
						for (unsigned k = 0; k < data.nDenseWorkers; ++k)
							data.events.AddEvent(new EVTClose);
					break; // loop back to consume our own EVT_CLOSE
				}
				return;
			}
			// select views to reconstruct the depth-map for this image
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			// cached .dmap is only reusable if it was written at the current
			// image resolution; a .dmap from a previous run at a different
			// resolution-level would propagate its (stale) size through
			// InitViews and violate the image/depthMap size invariant that
			// PatchMatch::EstimateDepthMap relies on. Peek the header (flags=0
			// reads only the metadata) and treat a size mismatch as if the
			// cache were missing so the image gets re-estimated cleanly.
			const auto isCachedDmapUsable = [&](IIndex idxImg) {
				const String path(ComposeDepthFilePath(data.scene.images[idxImg].ID, "dmap"));
				if (!File::access(path))
					return false;
				String storedImageFileName;
				IIndexArr storedIDs;
				cv::Size storedImageSize;
				KMatrix K; RMatrix R; CMatrix C;
				Depth dMin, dMax;
				DepthMap _d; NormalMap _n; ConfidenceMap _c; ViewsMap _v;
				if (!ImportDepthDataRaw(path, storedImageFileName, storedIDs, storedImageSize,
						K, R, C, dMin, dMax, _d, _n, _c, _v, 0))
					return false;
				return data.scene.images[idxImg].image.size() == storedImageSize;
			};
			const bool depthmapComputed(data.nFusionMode < 0 || (data.nFusionMode >= 0 && data.nEstimationGeometricIter < 0 && isCachedDmapUsable(idx)));
			// Task 12: on the LAST geometric-consistency iteration, ask InitViews to also load
			// neighbors' normal-map and confidence-map (loadDepthMaps==2) alongside their depth-map,
			// so the integrated confidence adjustment (EVT_SAVEDEPTHMAP below) can run from data
			// that's already being read for geometric-consistency scoring -- no extra neighbor load
			const bool bLastGeometricIter(data.nEstimationGeometricIter >= 0 &&
				data.nEstimationGeometricIter+1 == (int)OPTDENSE::nEstimationGeometricIters);
			// load neighbor normal+conf (==2) on the last geometric iteration when the integrated
			// confidence recalibration will run there: GPU (CUDA estimation + bEstimateConfidenceCUDA)
			// or the legacy integrated-CPU opt-in (bEstimateConfidence). nOptimize is already restored
			// to its ADJUST_CONFIDENCE bit on the last iteration (see the geometric loop above).
			const bool bWillAdjustConf(bLastGeometricIter && (OPTDENSE::nOptimize & OPTDENSE::ADJUST_CONFIDENCE) &&
				(OPTDENSE::bEstimateConfidence
				#ifdef _USE_CUDA
				|| (!data.depthMaps.pmCUDAPool.empty() && OPTDENSE::bEstimateConfidenceCUDA)
				#endif
				));
			const int nLoadDepthMaps(depthmapComputed ? -1 : (data.nEstimationGeometricIter >= 0 ?
				(bWillAdjustConf ? 2 : 1) : 0));
			// initialize images pair: reference image and the best neighbor view
			ASSERT(data.neighborsMap.IsEmpty() || data.neighborsMap[evtImage.idxImage] != NO_ID);
			if (!data.depthMaps.InitViews(depthData, data.neighborsMap.IsEmpty()?NO_ID:data.neighborsMap[evtImage.idxImage], OPTDENSE::nNumViews, !depthmapComputed, nLoadDepthMaps)) {
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
			if (VERBOSITY_LEVEL > 3)
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
			TD_TIMER_STARTD();
			const EVTSaveDepthMap& evtImage = *((EVTSaveDepthMap*)(Event*)evt);
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			// Task 12: integrated fusion-faithful confidence -- epilogue of the LAST geometric-
			// consistency iteration, using neighbor depth/normal/conf already loaded into
			// depthData.images[] by InitViews (loadDepthMaps==2, see its call site above) for THIS
			// iteration's geometric-consistency scoring; depthData.images[] is still resident here
			// (ReleaseImages() below hasn't run yet), so this costs no extra neighbor load and
			// writes depthData.confMap in place before it is serialized to disk a few lines down.
			// See the DepthMapsData::AdjustConfidence(DepthData&) overload above for the full
			// rationale and why no deferred confMapAdjusted swap is needed here.
			if ((OPTDENSE::nOptimize & OPTDENSE::ADJUST_CONFIDENCE) && data.nEstimationGeometricIter >= 0 &&
				data.nEstimationGeometricIter+1 == (int)OPTDENSE::nEstimationGeometricIters &&
				!depthData.depthMap.empty()) {
				bool bDone(false);
				#ifdef _USE_CUDA
				// GPU is the default when CUDA did the estimation (bEstimateConfidenceCUDA); on any CUDA
				// error AdjustConfidenceCUDA returns false and we fall back to the CPU sweep below
				const bool bTryGPU(!data.depthMaps.pmCUDAPool.empty() && OPTDENSE::bEstimateConfidenceCUDA);
				if (bTryGPU)
					bDone = data.depthMaps.AdjustConfidenceCUDA(depthData);
				#else
				const bool bTryGPU(false);
				#endif
				// CPU integrated: as the GPU-error fallback, or the legacy 'Estimate Confidence' opt-in
				if (!bDone && (bTryGPU || OPTDENSE::bEstimateConfidence))
					data.depthMaps.AdjustConfidence(depthData);
			}
			#if TD_VERBOSE != TD_VERBOSE_OFF
			// save depth map as image
			if (VERBOSITY_LEVEL > 2) {
				ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "png"), depthData.depthMap);
				ExportConfidenceMap(ComposeDepthFilePath(depthData.GetView().GetID(), "conf.png"), depthData.confMap);
				ExportPointCloud(ComposeDepthFilePath(depthData.GetView().GetID(), "ply"), *depthData.images.First().pImageData, depthData.depthMap, depthData.normalMap);
				if (VERBOSITY_LEVEL > 4) {
					ExportNormalMap(ComposeDepthFilePath(depthData.GetView().GetID(), "normal.png"), depthData.normalMap);
					depthData.confMap.Save(ComposeDepthFilePath(depthData.GetView().GetID(), "conf.pfm"));
				}
			}
			#endif
			// capture identifiers before Release wipes depthData state
			const IIndex viewID = depthData.GetView().GetID();
			const int dmRows = depthData.depthMap.rows;
			const int dmCols = depthData.depthMap.cols;
			// save compute depth-map for this image
			if (!depthData.depthMap.empty() &&
				!depthData.Save(ComposeDepthFilePath(viewID, data.nEstimationGeometricIter < 0 ? "dmap" : "geo.dmap")))
				exit(EXIT_FAILURE);
			depthData.ReleaseImages();
			depthData.Release();
			data.progress->operator++();
			// per-image save timing (gated at -v 2 so the default-verbose run
			// is not dragged down by per-image logging when pool-size grows)
			DEBUG_ULTIMATE("Depth-map %3u saved: %dx%d (%s)", viewID,
				dmCols, dmRows, TD_TIMER_GET_FMT().c_str());
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
			// make sure this image and its neighbors are loaded, via the phase-lifetime cache so a
			// neighbor shared by several references is read from disk at most once for the phase
			// (was up to ~9x via per-reference IncRef/DecRef); each DepthData's own CriticalSection
			// (unused in this phase now that IncRef/DecRef is gone) serializes only same-image cache
			// accesses, not unrelated ones -- see the g_pAdjustDMapCache comment above
			ASSERT(g_pAdjustDMapCache != NULL);
			{
				Lock l(depthData.cs);
				g_pAdjustDMapCache->UseImage(idx);
			}
			const unsigned numMaxNeighbors(8);
			IIndexArr idxNeighbors(0, depthData.neighbors.GetSize());
			for (const ViewScore& neighbor: depthData.neighbors) {
				DepthData& depthDataPair = data.depthMaps.arrDepthData[neighbor.ID];
				if (!depthDataPair.IsValid())
					continue;
				{
					Lock l(depthDataPair.cs);
					g_pAdjustDMapCache->UseImage(neighbor.ID);
				}
				idxNeighbors.push_back(neighbor.ID);
				if (idxNeighbors.size() == numMaxNeighbors)
					break;
			}
			// filter the depth-map for this image
			if ((OPTDENSE::nOptimize & OPTDENSE::ADJUST_CONFIDENCE) != 0 && data.depthMaps.AdjustConfidence(depthData, idxNeighbors)) {
				// load the filtered map after all depth-maps were filtered
				data.events.AddEvent(new EVTAdjustDepthMap(evtImage.idxImage));
			}
			data.SignalCompleteDepthmapFilter();
			break; }

		case EVT_ADJUSTDEPTHMAP: {
			const EVTAdjustDepthMap& evtImage = *((EVTAdjustDepthMap*)(Event*)evt);
			const IIndex idx = data.images[evtImage.idxImage];
			DepthData& depthData(data.depthMaps.arrDepthData[idx]);
			ASSERT(depthData.IsValid());
			// blocks until every EVT_FILTERDEPTHMAP has finished (SignalCompleteDepthmapFilter only
			// signals sem once idxImage reaches 0 -- see ComputeDepthMaps); this is what makes the
			// deferred swap below safe: all neighbor reads of this image's PRE-adjustment confMap
			// (from other references' AdjustConfidence calls) have completed by the time we get here
			data.sem.Wait();
			// ensure the depth-map is resident before swapping in the recalibrated conf-map computed
			// in memory by AdjustConfidence (no more adjusted.cmap disk round-trip); with the
			// phase's unlimited cache budget nothing is ever ejected, so this is a guaranteed cache
			// hit -- kept for uniformity and as defense should the budget policy ever change
			ASSERT(g_pAdjustDMapCache != NULL);
			{
				Lock l(depthData.cs);
				g_pAdjustDMapCache->UseImage(idx);
			}
			ASSERT(!depthData.IsEmpty() && !depthData.confMapAdjusted.empty());
			depthData.confMap = std::move(depthData.confMapAdjusted);
			depthData.confMapAdjusted.release();
			#if TD_VERBOSE != TD_VERBOSE_OFF
			// save depth map as image
			if (VERBOSITY_LEVEL > 2) {
				DepthMap depthMap(depthData.depthMap.clone());
				NormalMap normalMap(depthData.normalMap.clone());
				FilterDepthMap(depthMap, normalMap, depthData.confMap);
				ExportDepthMap(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.png"), depthMap);
				ExportPointCloud(ComposeDepthFilePath(depthData.GetView().GetID(), "filtered.ply"), *depthData.images.First().pImageData, depthMap, normalMap);
			}
			#endif
			// save filtered depth-map for this image
			if (!depthData.Save(ComposeDepthFilePath(depthData.GetView().GetID(), "dmap")))
				exit(EXIT_FAILURE);
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
	if (VERBOSITY_LEVEL > 2) {
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

	DEBUG_EXTRA("Point-cloud filtered: %u/%u points (%d%%) (%s)", pointcloud.points.size(), numInitPoints, ROUND2INT((100.f*pointcloud.points.GetSize())/numInitPoints), TD_TIMER_GET_FMT().c_str());
} // PointCloudFilter
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
