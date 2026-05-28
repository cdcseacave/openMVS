/*
* SceneDensify.h
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

#ifndef _MVS_SCENEDENSIFY_H_
#define _MVS_SCENEDENSIFY_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "SemiGlobalMatcher.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Forward declarations
class MVS_API Scene;
#ifdef _USE_CUDA
namespace CUDA {
class PatchMatch;
} // namespace CUDA
#endif // _USE_CUDA

// structure used to compute all depth-maps
class MVS_API DepthMapsData
{
public:
	DepthMapsData(Scene& _scene);
	~DepthMapsData();

	// pmCUDAPool holds move-only std::unique_ptrs; explicitly forbid copy so the
	// MSVC dllexport instantiator does not try to synthesize a copy constructor.
	DepthMapsData(const DepthMapsData&) = delete;
	DepthMapsData& operator=(const DepthMapsData&) = delete;

	bool SelectViews(DepthData& depthData);
	bool InitViews(DepthData& depthData, IIndex idxNeighbor, IIndex numNeighbors, bool loadImages, int loadDepthMaps);
	bool InitDepthMap(DepthData& depthData);
	bool EstimateDepthMap(IIndex idxImage, int nGeometricIter);

	#ifdef _USE_CUDA
	// Construct poolSize PatchMatch instances ready for the depth-map phase.
	// First construction probes CUDA::initDevices(); returns false if the
	// device set is still empty afterwards (caller falls back to CPU).
	bool AllocateCudaPool(unsigned poolSize);
	// Tear down per-instance state and re-init for the geometric-consistency
	// phase, resetting the slot counter and bumping the epoch so worker threads
	// re-claim slots cleanly even if the OS reuses them across the boundary.
	void ReinitCudaPoolForGeom();
	#endif // _USE_CUDA

	bool RemoveSmallSegments(DepthData& depthData);
	bool GapInterpolation(DepthData& depthData);

	void EstimateNormalMaps();

	bool AdjustConfidenceFast(DepthData& depthData, const IIndexArr& idxNeighbors);
	bool AdjustConfidence(DepthData& depthDataRef, const IIndexArr& idxNeighbors);
	void MergeDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal);
	void FuseDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal);
	void DenseFuseDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal);

	static DepthData ScaleDepthData(const DepthData& inputDeptData, float scale);

protected:
	static void* STCALL ScoreDepthMapTmp(void*);
	static void* STCALL EstimateDepthMapTmp(void*);
	static void* STCALL EndDepthMapTmp(void*);

public:
	Scene& scene;

	DepthDataArr arrDepthData;

	// used internally to estimate the depth-maps
	Image8U::Size prevDepthMapSize; // remember the size of the last estimated depth-map
	Image8U::Size prevDepthMapSizeTrg; // ... same for target image
	DepthEstimator::MapRefArr coords; // map pixel index to zigzag matrix coordinates
	DepthEstimator::MapRefArr coordsTrg; // ... same for target image

	#ifdef _USE_CUDA
	// One PatchMatch instance per worker thread; each worker claims a slot via
	// thread-local index gated by pmCUDAEpoch. Lets the {UploadCameras + kernel
	// launch} window stay mutex-serialized via the global event chain while the
	// per-instance host prep (image upload, depth-prior packing, result unpack)
	// runs lock-free in parallel across the SceneDensify ThreadPool workers.
	std::vector<std::unique_ptr<MVS::CUDA::PatchMatch>> pmCUDAPool;
	mutable volatile Thread::safe_t pmCUDANextIdx;
	mutable volatile Thread::safe_t pmCUDAEpoch;
	#endif // _USE_CUDA
};
/*----------------------------------------------------------------*/

struct MVS_API DenseDepthMapData {
	Scene& scene;
	IIndexArr images;
	IIndexArr neighborsMap;
	DepthMapsData depthMaps;
	volatile Thread::safe_t idxImage;
	SEACAVE::EventQueue events; // internal events queue (processed by the working threads)
	Semaphore sem;
	CAutoPtr<Util::Progress> progress;
	int nEstimationGeometricIter;
	int nFusionMode;
	float fSampleMeshNeighbors;
	STEREO::SemiGlobalMatcher sgm;
	// number of workers in the dense-reconstruction ThreadPool; set by
	// DenseReconstruction once the CUDA pool size is known. Used by the worker
	// EVT_PROCESSIMAGE handler to broadcast EVT_CLOSE to all sibling workers
	// (the old single-EVT_CLOSE pattern hung when nWorkers > 2).
	unsigned nDenseWorkers;

	DenseDepthMapData(Scene& _scene, int _nFusionMode=0, float _fSampleMeshNeighbors=0);
	~DenseDepthMapData();

	void SignalCompleteDepthmapFilter();
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif
