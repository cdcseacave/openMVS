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
	
// structure used to compute all depth-maps
class MVS_API DepthMapsData
{
public:
	DepthMapsData(Scene& _scene);
	~DepthMapsData();

	bool SelectViews(IIndexArr& images, IIndexArr& imagesMap, IIndexArr& neighborsMap);
	bool SelectViews(DepthData& depthData);
	bool InitViews(DepthData& depthData, IIndex idxNeighbor, IIndex numNeighbors);
	bool InitDepthMap(DepthData& depthData);
	bool EstimateDepthMap(IIndex idxImage);

	bool RemoveSmallSegments(DepthData& depthData);
	bool GapInterpolation(DepthData& depthData);

	bool FilterDepthMap(DepthData& depthData, const IIndexArr& idxNeighbors, bool bAdjust=true);
	void FuseDepthMaps(PointCloud& pointcloud, bool bEstimateColor, bool bEstimateNormal);

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
	int nFusionMode;
	STEREO::SemiGlobalMatcher sgm;

	DenseDepthMapData(Scene& _scene, int _nFusionMode=0);
	~DenseDepthMapData();

	void SignalCompleteDepthmapFilter();
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif
