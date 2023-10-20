/*
* Scene.h
*
* Copyright (c) 2014-2022 SEACAVE
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

#ifndef _MVS_SCENE_H_
#define _MVS_SCENE_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "SceneDensify.h"
#include "Mesh.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Forward declarations
struct MVS_API DenseDepthMapData;

class MVS_API Scene
{
public:
	PlatformArr platforms; // camera platforms, each containing the mounted cameras and all known poses
	ImageArr images; // images, each referencing a platform's camera pose
	PointCloud pointcloud; // point-cloud (sparse or dense), each containing the point position and the views seeing it
	Mesh mesh; // mesh, represented as vertices and triangles, constructed from the input point-cloud
	OBB3f obb; // optional region-of-interest; oriented bounding box containing the entire scene

	unsigned nCalibratedImages; // number of valid images

	unsigned nMaxThreads; // maximum number of threads used to distribute the work load

public:
	inline Scene(unsigned _nMaxThreads=0)
		: obb(true), nMaxThreads(Thread::getMaxThreads(_nMaxThreads)) {}

	void Release();
	bool IsValid() const;
	bool IsEmpty() const;
	bool ImagesHaveNeighbors() const;
	bool IsBounded() const { return obb.IsValid(); }

	bool LoadInterface(const String& fileName);
	bool SaveInterface(const String& fileName, int version=-1) const;

	bool LoadDMAP(const String& fileName);
	bool LoadViewNeighbors(const String& fileName);
	bool SaveViewNeighbors(const String& fileName) const;
	bool Import(const String& fileName);

	enum SCENE_TYPE {
		SCENE_NA = 0,
		SCENE_INTERFACE = 1,
		SCENE_MVS = 2,
		SCENE_IMPORT = 3,
	};
	SCENE_TYPE Load(const String& fileName, bool bImport=false);
	bool Save(const String& fileName, ARCHIVE_TYPE type=ARCHIVE_DEFAULT) const;

	bool EstimateNeighborViewsPointCloud(unsigned maxResolution=16);
	void SampleMeshWithVisibility(unsigned maxResolution=320);
	bool ExportMeshToDepthMaps(const String& baseName);

	bool SelectNeighborViews(uint32_t ID, IndexArr& points, unsigned nMinViews = 3, unsigned nMinPointViews = 2, float fOptimAngle = FD2R(12), unsigned nInsideROI = 1);
	void SelectNeighborViews(unsigned nMinViews = 3, unsigned nMinPointViews = 2, float fOptimAngle = FD2R(12), unsigned nInsideROI = 1);
	static bool FilterNeighborViews(ViewScoreArr& neighbors, float fMinArea=0.1f, float fMinScale=0.2f, float fMaxScale=2.4f, float fMinAngle=FD2R(3), float fMaxAngle=FD2R(45), unsigned nMaxViews=12);

	bool ExportCamerasMLP(const String& fileName, const String& fileNameScene) const;
	static bool ExportLinesPLY(const String& fileName, const CLISTDEF0IDX(Line3f,uint32_t)& lines, const Pixel8U* colors=NULL, bool bBinary=true);

	// sub-scene split and save
	struct ImagesChunk {
		std::unordered_set<IIndex> images;
		AABB3f aabb;
	};
	typedef cList<ImagesChunk,const ImagesChunk&,2,16,uint32_t> ImagesChunkArr;
	unsigned Split(ImagesChunkArr& chunks, float maxArea, int depthMapStep=8) const;
	bool ExportChunks(const ImagesChunkArr& chunks, const String& path, ARCHIVE_TYPE type=ARCHIVE_DEFAULT) const;

	// Transform scene
	bool Center(const Point3* pCenter = NULL);
	bool Scale(const REAL* pScale = NULL);
	bool ScaleImages(unsigned nMaxResolution = 0, REAL scale = 0, const String& folderName = String());
	void Transform(const Matrix3x3& rotation, const Point3& translation, REAL scale);
	void Transform(const Matrix3x4& transform);
	bool AlignTo(const Scene&);
	REAL ComputeLeveledVolume(float planeThreshold=0, float sampleMesh=-100000, unsigned upAxis=2, bool verbose=true);

	// Estimate and set region-of-interest
	bool EstimateROI(int nEstimateROI=0, float scale=1.f);
	
	// Tower scene
	bool ComputeTowerCylinder(Point2f& centerPoint, float& fRadius, float& fROIRadius, float& zMin, float& zMax, float& minCamZ, const int towerMode);
	void InitTowerScene(const int towerMode);
	size_t DrawCircle(PointCloud& pc,PointCloud::PointArr& outCircle, const Point3f& circleCenter, const float circleRadius, const unsigned nTargetPoints, const float fStartAngle, const float fAngleBetweenPoints);
	PointCloud BuildTowerMesh(const PointCloud& origPointCloud, const Point2f& centerPoint, const float fRadius, const float fROIRadius, const float zMin, const float zMax, const float minCamZ, bool bFixRadius = false);
	
	// Dense reconstruction
	bool DenseReconstruction(int nFusionMode=0, bool bCrop2ROI=true, float fBorderROI=0);
	bool ComputeDepthMaps(DenseDepthMapData& data);
	void DenseReconstructionEstimate(void*);
	void DenseReconstructionFilter(void*);
	void PointCloudFilter(int thRemove=-1);

	// Mesh reconstruction
	bool ReconstructMesh(float distInsert=2, bool bUseFreeSpaceSupport=true, bool bUseOnlyROI=false, unsigned nItersFixNonManifold=4,
						 float kSigma=2.f, float kQual=1.f, float kb=4.f,
						 float kf=3.f, float kRel=0.1f/*max 0.3*/, float kAbs=1000.f/*min 500*/, float kOutl=400.f/*max 700.f*/,
						 float kInf=(float)(INT_MAX/8));

	// Mesh refinement
	bool RefineMesh(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews, float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize,
		unsigned nMaxFaceArea, unsigned nScales, float fScaleStep, unsigned nAlternatePair, float fRegularityWeight, float fRatioRigidityElasticity, float fGradientStep,
		float fThPlanarVertex=0.f, unsigned nReduceMemory=1);
	#ifdef _USE_CUDA
	bool RefineMeshCUDA(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews, float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize,
		unsigned nMaxFaceArea, unsigned nScales, float fScaleStep, unsigned nAlternatePair, float fRegularityWeight, float fRatioRigidityElasticity, float fGradientStep);
	#endif

	// Mesh texturing
	bool TextureMesh(unsigned nResolutionLevel, unsigned nMinResolution, unsigned minCommonCameras=0, float fOutlierThreshold=0.f, float fRatioDataSmoothness=0.3f,
		bool bGlobalSeamLeveling=true, bool bLocalSeamLeveling=true, unsigned nTextureSizeMultiple=0, unsigned nRectPackingHeuristic=3, Pixel8U colEmpty=Pixel8U(255,127,39),
		float fSharpnessWeight=0.5f, int ignoreMaskLabel=-1, int maxTextureSize=0, const IIndexArr& views=IIndexArr());

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & platforms;
		ar & images;
		ar & pointcloud;
		ar & mesh;
		ar & obb;
	}
	#endif
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_SCENE_H_
