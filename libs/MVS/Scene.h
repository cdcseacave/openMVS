/*
* Scene.h
*
* Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
* Please read <http://foxel.ch/license> for more information.
*
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This file is part of the FOXEL project <http://foxel.ch>.
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
*
*      You are required to attribute the work as explained in the "Usage and
*      Attribution" section of <http://foxel.ch/license>.
*/

#ifndef _MVS_SCENE_H_
#define _MVS_SCENE_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"
#include "PointCloud.h"
#include "Mesh.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

typedef float Depth;
typedef Point3f Normal;
typedef TImage<Depth> DepthMap;
typedef TImage<Normal> NormalMap;
typedef TImage<float> ConfidenceMap;
typedef SEACAVE::cList<Depth, Depth, 0> DepthArr;
typedef SEACAVE::cList<DepthMap, const DepthMap&, 2> DepthMapArr;
typedef SEACAVE::cList<NormalMap, const NormalMap&, 2> NormalMapArr;
typedef SEACAVE::cList<ConfidenceMap, const ConfidenceMap&, 2> ConfidenceMapArr;
/*----------------------------------------------------------------*/

class Scene
{
public:
	PlatformArr platforms; // camera platforms, each containing the mounted cameras and all known poses
	ImageArr images; // images, each referencing a platform's camera pose
	PointCloud pointcloud; // point-cloud (sparse or dense), each containing the point position and the views seeing it
	Mesh mesh; // mesh, represented as vertices and triangles, constructed from the input point-cloud

	unsigned nCalibratedImages; // number of valid images

public:
	inline Scene() {}

	bool LoadInterface(const String& fileName);
	bool SaveInterface(const String& fileName) const;

	bool Load(const String& fileName);
	bool Save(const String& fileName, ARCHIVE_TYPE type=ARCHIVE_BINARY_ZIP) const;

	bool ReconstructMesh(float distInsert=2, bool bUseFreeSpaceSupport=true);

	bool RefineMesh(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews, float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nScales, float fScaleStep, float fRegularityWeight, unsigned nMaxFaceArea, float fConfidenceThreshold);

	bool TextureMesh(unsigned nResolutionLevel, unsigned nMinResolution, bool bGlobalSeamLeveling=true, bool bLocalSeamLeveling=true);

	bool SelectNeighborViews(uint32_t ID, IndexArr& points, unsigned nMinViews=3, unsigned nMinPointViews=2, float fOptimAngle=FD2R(12));
	static bool FilterNeighborViews(ViewScoreArr& neighbors, float fMinArea=0.12f, float fMinScale=0.2f, float fMaxScale=2.4f, float fMinAngle=FD2R(3), float fMaxAngle=FD2R(45), unsigned nMaxViews=12);

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & platforms;
		ar & images;
		ar & pointcloud;
		ar & mesh;
	}
	#endif
};
/*----------------------------------------------------------------*/

// Tools
bool ExportDepthMap(const String& fileName, const DepthMap& depthMap, Depth minDepth=FLT_MAX, Depth maxDepth=0);
bool ExportNormalMap(const String& fileName, const NormalMap& normalMap);
bool ExportConfidenceMap(const String& fileName, const ConfidenceMap& confMap);
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_SCENE_H_
