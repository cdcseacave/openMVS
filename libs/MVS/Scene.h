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
	OBB3f obb; // region-of-interest represented as oriented bounding box containing the entire scene (optional)
	Matrix4x4 transform; // transformation used to convert from absolute to relative coordinate system (optional)

	unsigned nCalibratedImages; // number of valid images

	unsigned nMaxThreads; // maximum number of threads used to distribute the work load

public:
	inline Scene(unsigned _nMaxThreads=0)
		: obb(true), transform(Matrix4x4::IDENTITY), nMaxThreads(Thread::getMaxThreads(_nMaxThreads)) {}

	void Release();
	bool IsValid() const;
	bool IsEmpty() const;
	bool ImagesHaveNeighbors() const;
	bool IsBounded() const { return obb.IsValid(); }
	bool HasTransform() const { return transform != Matrix4x4::IDENTITY; }

	bool LoadInterface(const String& fileName);
	bool SaveInterface(const String& fileName, int version=-1) const;

	bool LoadROI(const String& fileName);
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

	bool EstimatePointCloudNormals(bool bRefine=true);
	bool EstimateSparseSurface(unsigned kNeighbors=16, float sizeScale=0.9f, float normalAngleMax=D2R(0.f));
	bool EstimateNeighborViewsPointCloud(unsigned maxResolution=16);
	void SampleMeshWithVisibility(REAL sampling=0, unsigned maxResolution=320);
	bool ExportMeshToDepthMaps(const String& baseName);

	bool SelectNeighborViews(uint32_t ID, IndexArr& points, unsigned nMinViews=3, unsigned nMinPointViews=2, float fOptimAngle=D2R(12.f), float fWeightPointInsideROI=0.7f);
	void SelectNeighborViews(unsigned nMinViews=3, unsigned nMinPointViews=2, float fOptimAngle=D2R(12.f), float fWeightPointInsideROI=0.7f);
	static bool FilterNeighborViews(ViewScoreArr& neighbors, float fMinArea=0.1f, float fMinScale=0.2f, float fMaxScale=2.4f, float fMinAngle=D2R(3.f), float fMaxAngle=D2R(45.f), unsigned nMaxViews=12);

	bool ExportCamerasMLP(const String& fileName, const String& fileNameScene) const;
	static bool ExportLinesPLY(const String& fileName, const CLISTDEF0IDX(Line3f,uint32_t)& lines, const Pixel8U* colors=NULL, bool bBinary=true);

	// Sub-scene split and save
	struct ImagesChunk {
		std::unordered_set<IIndex> images;
		AABB3f aabb;
	};
	typedef cList<ImagesChunk,const ImagesChunk&,2,16,uint32_t> ImagesChunkArr;
	unsigned Split(ImagesChunkArr& chunks, float maxArea, int depthMapStep=8) const;
	bool ExportChunks(const ImagesChunkArr& chunks, const String& path, ARCHIVE_TYPE type=ARCHIVE_DEFAULT) const;

	// Transform scene
	bool Center(const Point3* pCenter=NULL);
	bool Scale(const REAL* pScale=NULL);
	bool ScaleImages(unsigned nMaxResolution=0, REAL scale=0, const String& folderName={});
	Matrix4x4 ComputeNormalizationTransform(bool bScale = false) const;
	void Transform(const Matrix3x3& rotation, const Point3& translation, REAL scale);
	void Transform(const Matrix3x4& transform);
	bool AlignTo(const Scene&);
	REAL ComputeLeveledVolume(float planeThreshold=0, float sampleMesh=-100000, unsigned upAxis=2, bool verbose=true);
	void AddNoiseCameraPoses(float epsPosition, float epsRotation);
	Scene SubScene(const IIndexArr& idxImages) const;
	Scene& CropToROI(const OBB3f&, unsigned minNumPoints=3);
	bool EstimateROI(float scaleROI=1.1f, int upAxis=-1);
	bool EstimateGravityDirection(Point3f& up) const;
	FloatArr ROIPointWeights(const UnsignedArr& indices, float& medianNeighborDistance) const;
	float ComputeDistanceCameras2Scene(float depthPercentile=0.1f, bool bForceRecompute=false, bool bUseROI=true);

	// Tower scene
	bool ComputeCenterLine(Line3f &camCenterLine, const Point3f* up=NULL) const;
	bool ComputeTowerCylinder(Point2f& centerPoint, float& fRadius, float& fROIRadius, float& zMin, float& zMax, float& minCamZ, const int towerMode);
	void InitTowerScene(const int towerMode);
	size_t DrawCircle(PointCloud& pc,PointCloud::PointArr& outCircle, const Point3f& circleCenter, const float circleRadius, const unsigned nTargetPoints, const float fStartAngle, const float fAngleBetweenPoints);
	PointCloud BuildTowerMesh(const PointCloud& origPointCloud, const Point2f& centerPoint, const float fRadius, const float fROIRadius, const float zMin, const float zMax, const float minCamZ, bool bFixRadius = false);

	// Dense reconstruction
	bool DenseReconstruction(int nFusionMode=0, bool bCrop2ROI=true, float fBorderROI=0, float fSampleMeshNeighbors=0);
	bool ComputeDepthMaps(DenseDepthMapData& data);
	void DenseReconstructionEstimate(void*);
	void DenseReconstructionFilter(void*);
	void PointCloudFilter(int thRemove=-1);

	// Mesh reconstruction
	// hard-constraint capacity of the cells containing a camera; not exposed by the apps
	static constexpr float kInfCapacity = (float)(INT_MAX/8);
	// experiment switches; every member defaults to the shipped behavior, so an omitted
	// struct leaves the reconstruction bit-identical
	struct ReconstructMeshParams {
		// t-edge enforcement of the weakly supported surfaces classifier (ISRN-2014 Eq. 8):
		// PRODUCT multiplies the target cell's t-edge by epsAbs once per firing (vertex, view)
		// pair - the shipped behavior, a per-view product that diverges geometrically;
		// PAPER sums epsAbs over the firing pairs of a cell and multiplies once - the fidelity
		// restoration; ADD and MAX replace the multiply and so drop the alpha^2 unit defect
		// Eq. 8 itself carries - deliberate departures from the paper, not restorations
		enum WeakSurfEnforcement {
			WSE_PRODUCT = 0,
			WSE_PAPER,
			WSE_ADD,
			WSE_MAX
		};
		WeakSurfEnforcement weakSurfEnforcement = WSE_PRODUCT;
		// scale kQual by the mean confidence of the consumed point weights: weighting shrinks
		// every data-term capacity by that mean while the quality term keeps its unit-vote
		// calibration; no-op if the point-cloud carries no weights
		bool bQualityCoScale = false;
		// grazing-incidence down-weighting of the visibility votes deposited on the crossed
		// facets: the vote is scaled by max(grazingCosFloor, |cos(ray, facet normal)|^grazingCosExp),
		// so a ray skimming a facet is weaker free-space evidence than one crossing it frontally;
		// a floor of 1 makes the factor identically 1 and so disables the whole term
		float grazingCosFloor = 1.f;
		// exponent shaping the incidence cosine before the floor is applied (>0)
		float grazingCosExp = 1.f;
		// per-vertex sigma in the three roles where sigma stands for the point's own positional
		// uncertainty (soft-visibility exponent, end-cell offset, free-space-support windows):
		// sigma_v = kSigma * median incident finite-Delaunay-edge length of the vertex, clamped
		// to [0.25, 4] x the global sigma, so a locally denser sample gets a tighter uncertainty;
		// false = the single global sigma everywhere, the exact legacy behavior
		bool bAdaptiveSigma = false;
		// per-vertex sigma derived from the pixel footprint instead of the local edge length:
		// footprint_v = mean over the (point, view) pairs merged into the vertex of
		// ||X - C_view|| / f_view, the scene-unit size of one pixel of that view at that range -
		// the physical scale densification localized the point at. The field is consumed
		// relatively: it is calibrated by sigma / median(footprint_v) so the established global
		// sigma scale (and the working space of the canonical rescale) is preserved, then clamped
		// to the same [0.25, 4] x global sigma band. Competing base with bAdaptiveSigma - the app
		// rejects the combination and the library lets bAdaptiveSigma win; needs only the point
		// views, so it works on a cloud carrying no confidences
		bool bFootprintSigma = false;
		// shrink the per-vertex sigma towards the confident end of its range:
		// sigma_v *= 1 - sigmaConfShrink * conf_v, where conf_v in [0,1] is the mean of the
		// per-view confidences consumed for that vertex, the result clamped to the same
		// [0.25, 4] x global sigma band; the confidence reaches sigma only, the votes keep
		// their unit scale (see bConstantVotes); 0 disables, no-op without point weights
		float sigmaConfShrink = 0.f;
		// deposit a vote of 1 per (vertex, view) pair even when the point-cloud carries
		// per-view confidences, so a run can feed those confidences to sigma while the
		// energy stays in the uniform-weight regime its constants are tuned for
		bool bConstantVotes = false;
		// rescale the triangulation by a power of two so the median Delaunay edge lands near 1,
		// where the ray-walk orientation predicate is calibrated: that predicate tests an
		// unnormalized determinant growing as the cube of the edge length against a fixed
		// absolute epsilon, so a scene whose median edge sits far below one unit collapses to
		// COPLANAR at every walk step. The factor multiplies exactly in IEEE arithmetic and the
		// kernel's exact predicates are scale-invariant, so the triangulation is scaled in place
		// and the inverse applied at extraction; scenes whose median edge already falls inside
		// [2^-10, 2^10] are left untouched. This repairs the predicate only - geometry already
		// quantized away by the float storage of PointCloud::Point needs a load-time fix instead
		bool bCanonicalRescale = false;
	};
	// carveRaysFile: optional sidecar of confident depth pixels fusion dropped (see UnfusedPixel),
	// replayed as free-space rays that carve without inserting vertices (empty - disabled)
	bool ReconstructMesh(float distInsert=2, bool bUseFreeSpaceSupport=true, bool bUseOnlyROI=false,
						 float kSigma=2.f, float kQual=1.f, float kb=4.f,
						 float kf=3.f, float kRel=0.1f/*max 0.3*/, float kAbs=1000.f/*min 500*/, float kOutl=400.f/*max 700.f*/,
						 float kInf=kInfCapacity, const String& carveRaysFile=String(),
						 const ReconstructMeshParams& params=ReconstructMeshParams());

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
		bool bGlobalSeamLeveling=true, bool bLocalSeamLeveling=true, unsigned nTextureSizeMultiple=0, Pixel8U colEmpty=Pixel8U(255,127,39),
		float fSharpnessWeight=0.5f, int ignoreMaskLabel=-1, int maxTextureSize=0, const IIndexArr& views=IIndexArr());
	bool ComputeVertexColors(unsigned nResolutionLevel, unsigned nMinResolution, unsigned minCommonCameras=0,
		float fOutlierThreshold=0.f, float fRatioDataSmoothness=0.3f, Pixel8U colEmpty=Pixel8U(255,127,39),
		int ignoreMaskLabel=-1, const IIndexArr& views=IIndexArr());

	// Reconstruction quality assessment
	struct Score {
		float completeness{0}; // fraction of image covered by mesh [0,1]
		float ssim{0};         // SSIM in covered region [0,1]
		float psnr{0};         // PSNR in dB (diagnostic)
		float score() const { return 100.f * completeness * ssim; }
	};
	struct ImageScore : Score {
		IIndex idxImage;
	};
	struct ReconstructionQuality : Score {
		CLISTDEFIDX(ImageScore, IIndex) imageScores;
	};
	ReconstructionQuality ComputeReconstructionQuality(unsigned nMaxResolution = 0) const;

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & platforms;
		ar & images;
		ar & pointcloud;
		ar & mesh;
		ar & obb;
		ar & transform;
	}
	#endif
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_SCENE_H_
