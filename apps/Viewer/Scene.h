/*
 * Scene.h
 *
 * Copyright (c) 2014-2025 SEACAVE
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

#pragma once

#include "Window.h"

namespace VIEWER {

class Scene {
public:
	// Per-image pose uncertainty loaded from a CreateStructure pose-quality CSV report
	struct CameraUncertainty {
		enum State : uint8_t {
			NOT_COMPUTED = 0,
			COMPUTED,
			DATUM
		};
		Matrix3x3f posCov; // world-frame camera-center covariance (m^2 when geo-referenced)
		Point3f posSigma; // camera-center 1-sigma along the world axes (sqrt of posCov diagonal)
		Point3f rotSigma; // rotation 1-sigma about the camera x/y/z axes (deg)
		State state{NOT_COMPUTED};
		bool IsComputed() const { return state != NOT_COMPUTED; }
		float MaxPosSigma() const { return MAXF(MAXF(posSigma.x, posSigma.y), posSigma.z); }
	};
	typedef CLISTDEF0IDX(CameraUncertainty, uint32_t) CameraUncertaintyArr;

	struct Layer {
		uint32_t id{NO_ID};
		String label;
		String sceneName;
		String workingFolder;
		bool visible{true};
		bool dirty{false};
		bool compareRight{false}; // compare split view side: false = left (A), true = right (B)
		bool usePointSolidColor{false};
		bool useCameraJetColor{false};
		Point3f pointColor{1.f, 1.f, 1.f};
		Point3f cameraColor{1.f, 1.f, 0.f};
		MVS::Scene scene;
		ImageArr images; // valid scene photos for this layer
		CameraUncertaintyArr cameraUncertainty; // per viewer-image (indexed like images), empty when not loaded
		float cameraUncertaintyNorm{0.f}; // ellipsoid colormap normalization (robust max of MaxPosSigma)
		float cameraUncertaintyAutoScale{1.f}; // scene-fit radius scale (Window::uncertaintyEllipsoidScale multiplies it)
		AABB3f bounds{true};
		Point3f sceneSize{0, 0, 0};
		float sceneDistance{1.f};

		Layer() = default;
		Layer(const Layer&) = delete;
		Layer& operator=(const Layer&) = delete;
		Layer(Layer&&) noexcept = default;
		Layer& operator=(Layer&&) noexcept = default;

		bool IsOpen() const { return scene.IsValid() || !scene.IsEmpty(); }
	};
	using LayerArr = std::vector<Layer>;

	struct EstimateROIWorkflowOptions {
		float scaleROI{1.1f};
		int upAxis{-1}; // -1 = auto, 0=X,1=Y,2=Z
	};

	struct DensifyWorkflowOptions {
		unsigned resolutionLevel{1};
		unsigned maxResolution{2560};
		unsigned minResolution{640};
		unsigned subResolutionLevels{2};
		#ifdef _USE_CUDA
		unsigned numViews{8};
		#else
		unsigned numViews{5};
		#endif
		unsigned minViews{3};
		unsigned minViewsTrust{2};
		unsigned minViewsFuse{2};
		#ifdef _USE_CUDA
		unsigned estimationIters{4};
		#else
		unsigned estimationIters{3};
		#endif
		unsigned geometricIters{2};
		unsigned fuseFilter{2};
		bool estimateColors{true};
		bool estimateNormals{true};
		bool removeDepthMaps{false};
		bool postprocess{false};
		int fusionMode{0};
		float fDepthReprojectionErrorThreshold{1.0f};
		bool cropToROI{true};
		float borderROI{0.f};
		float sampleMeshNeighbors{0.f};
	};

	struct ReconstructMeshWorkflowOptions {
		float minPointDistance{1.5f};
		bool useFreeSpaceSupport{false};
		bool useOnlyROI{false};
		bool constantWeight{true};
		float thicknessFactor{1.f};
		float qualityFactor{1.f};
		float decimateMesh{1.f};
		unsigned targetFaceNum{0};
		float removeSpurious{20.f};
		bool removeSpikes{true};
		unsigned closeHoles{30};
		unsigned smoothSteps{10};
		float edgeLength{0.f};
		bool cropToROI{true};
	};

	struct RefineMeshWorkflowOptions {
		unsigned resolutionLevel{0};
		unsigned minResolution{640};
		unsigned maxViews{8};
		float decimateMesh{0.f};
		unsigned closeHoles{30};
		unsigned ensureEdgeSize{1};
		unsigned maxFaceArea{32};
		unsigned scales{2};
		float scaleStep{0.5f};
		unsigned alternatePair{0};
		float regularityWeight{0.2f};
		float rigidityElasticityRatio{0.9f};
		float gradientStep{45.05f};
		float planarVertexRatio{0.f};
	};

	struct TextureMeshWorkflowOptions {
		float decimateMesh{1.f};
		unsigned closeHoles{30};
		unsigned resolutionLevel{0};
		unsigned minResolution{640};
		unsigned minCommonCameras{0};
		float outlierThreshold{6e-2f};
		float ratioDataSmoothness{0.1f};
		bool globalSeamLeveling{true};
		bool localSeamLeveling{true};
		unsigned textureSizeMultiple{0};
		uint32_t emptyColor{0x00FF7F27};
		float sharpnessWeight{0.5f};
		int ignoreMaskLabel{-1};
		int maxTextureSize{8192};
	};

public:
	ARCHIVE_TYPE nArchiveType;
	String name;

	bool estimateSfMNormals;
	bool estimateSfMPatches;
	LayerArr layers;
	int activeLayerIndex;
	uint32_t nextLayerID;
	uint32_t workflowLayerID;
	Window window;

	// Track-based neighbor information with shared point indices
	struct ViewScoreWithPoints {
		MVS::ViewScore score;
		MVS::PointCloud::IndexArr sharedPoints; // indices of shared points in the pointcloud
	};
	typedef CLISTDEFIDX(ViewScoreWithPoints,uint32_t) ViewScoreWithPointsArr;
	CLISTDEFIDX(ViewScoreWithPointsArr,uint32_t) trackBasedNeighbors; // per-viewer image neighbors from shared tracks

	EstimateROIWorkflowOptions estimateROIOptions;
	DensifyWorkflowOptions densifyOptions;
	ReconstructMeshWorkflowOptions reconstructOptions;
	RefineMeshWorkflowOptions refineOptions;
	TextureMeshWorkflowOptions textureOptions;

	// multi-threading
	static SEACAVE::EventQueue events; // internal events queue (processed by the working threads)
	static SEACAVE::Thread thread; // worker thread

	// workflow state tracking
	enum WorkflowState {
		WF_STATE_IDLE = 0,
		WF_STATE_RUNNING,
		WF_STATE_COMPLETED,
		WF_STATE_FAILED
	};
	enum WorkflowType {
		WF_NONE = 0,
		WF_ESTIMATE_ROI,
		WF_DENSIFY,
		WF_RECONSTRUCT,
		WF_REFINE,
		WF_TEXTURE
	};
	enum ExportGeometry {
		EXPORT_ALL = 0,
		EXPORT_POINT_CLOUD,
		EXPORT_MESH
	};
	std::atomic<WorkflowState> workflowState;
	std::atomic<WorkflowType> currentWorkflowType;
	std::atomic<bool> geometryModified;
	std::atomic<unsigned> pendingImageLoads;
	double workflowStartTime;
	SEACAVE::CriticalSection workflowMutex;

	// Workflow history for stats display
	struct WorkflowHistoryEntry {
		WorkflowType type;
		double duration;
		bool success;
	};
	std::vector<WorkflowHistoryEntry> workflowHistory;

public:
	explicit Scene(ARCHIVE_TYPE _nArchiveType = ARCHIVE_MVS);
	~Scene();

	bool Initialize(const cv::Size& size, const String& windowName,
				   const String& fileName = String(), const String& geometryFileName = String());
	void Run();

	void Reset();
	void Release();

	inline bool IsValid() const { return window.IsValid(); }
	inline bool IsOpen() const { return IsValid() && !layers.empty(); }

	bool SetViewFromFile(const String& viewFileName);
	bool SetViewFromCamera(unsigned camIndex);

	// Scene management
	bool Open(const String& fileName, String geometryFileName = {});
	bool OpenFiles(const std::vector<String>& fileNames, bool replaceExisting = true);
	bool AddLayer(const String& fileName, String geometryFileName = {}, bool makeActive = true);
	bool RemoveLayer(size_t layerIndex);
	bool Save(const String& fileName = String(), bool bRescaleImages = false);
	bool SaveModifiedLayers();
	bool Export(const String& fileName, const String& exportType = String(), bool bViews = true, ExportGeometry geometry = EXPORT_ALL) const;
	bool ExportVisibleLayers(const String& fileName, const String& exportType = String(), ExportGeometry geometry = EXPORT_ALL) const;

	// Pose uncertainty display (per-image quality report produced by CreateStructure)
	bool LoadPoseUncertainty(const String& fileName);
	bool HasCameraUncertainty() const;

	// Workflows (async execution)
	bool RunEstimateROIWorkflow(const EstimateROIWorkflowOptions& options);
	bool RunDensifyWorkflow(const DensifyWorkflowOptions& options);
	bool RunReconstructMeshWorkflow(const ReconstructMeshWorkflowOptions& options);
	bool RunRefineMeshWorkflow(const RefineMeshWorkflowOptions& options);
	bool RunTextureMeshWorkflow(const TextureMeshWorkflowOptions& options);
	bool RunBatchWorkflow(const std::vector<WorkflowType>& workflowTypes);

	// Workflow state management
	bool IsWorkflowRunning() const { return workflowState.load() == WF_STATE_RUNNING; }
	bool HasPendingImageLoads() const { return pendingImageLoads.load() != 0; }
	bool HasBackgroundWork() const { return workflowState.load() != WF_STATE_IDLE || HasPendingImageLoads(); }
	WorkflowState GetWorkflowState() const { return workflowState.load(); }
	WorkflowType GetCurrentWorkflowType() const { return currentWorkflowType.load(); }
	static const char* GetWorkflowName(WorkflowType type, bool shortName = false);
	double GetWorkflowElapsedTime() const;
	void CheckWorkflowCompletion(); // Called from main loop to check if workflow completed
	bool IsGeometryModified() const { return geometryModified.load(); }
	void SetGeometryModified(bool modified = true);
	const std::vector<WorkflowHistoryEntry>& GetWorkflowHistory() const { return workflowHistory; }
	void ClearWorkflowHistory() { workflowHistory.clear(); }

	// Geometry operations
	void RemoveSelectedGeometry();
	void SetROIFromSelection(bool aabb = false);
	void ClearBoundingBox();
	void SetBoundingBox(const OBB3f& obb);
	MVS::Scene CropToPoints(const MVS::PointCloud::IndexArr& selectedPointIndices, unsigned minPoints = 20) const;

	// Accessors
	const CLISTDEFIDX(ViewScoreWithPointsArr,uint32_t)& GetTrackBasedNeighbors() const { return trackBasedNeighbors; }

	// Getters
	EstimateROIWorkflowOptions& GetEstimateROIWorkflowOptions() { return estimateROIOptions; }
	const EstimateROIWorkflowOptions& GetEstimateROIWorkflowOptions() const { return estimateROIOptions; }
	DensifyWorkflowOptions& GetDensifyWorkflowOptions() { return densifyOptions; }
	const DensifyWorkflowOptions& GetDensifyWorkflowOptions() const { return densifyOptions; }
	ReconstructMeshWorkflowOptions& GetReconstructMeshWorkflowOptions() { return reconstructOptions; }
	const ReconstructMeshWorkflowOptions& GetReconstructMeshWorkflowOptions() const { return reconstructOptions; }
	RefineMeshWorkflowOptions& GetRefineMeshWorkflowOptions() { return refineOptions; }
	const RefineMeshWorkflowOptions& GetRefineMeshWorkflowOptions() const { return refineOptions; }
	TextureMeshWorkflowOptions& GetTextureMeshWorkflowOptions() { return textureOptions; }
	const TextureMeshWorkflowOptions& GetTextureMeshWorkflowOptions() const { return textureOptions; }
	const LayerArr& GetLayers() const { return layers; }
	size_t GetLayerCount() const { return layers.size(); }
	int GetActiveLayerIndex() const { return activeLayerIndex; }
	bool HasVisibleLayers() const;
	Layer* GetLayer(size_t idx);
	const Layer* GetLayer(size_t idx) const;
	Layer* GetActiveLayer();
	const Layer* GetActiveLayer() const;
	Layer* GetLayerByID(uint32_t layerID);
	const Layer* GetLayerByID(uint32_t layerID) const;
	bool SetActiveLayer(size_t layerIndex, bool requestRedraw = true);
	bool SetActiveLayerByID(uint32_t layerID, bool requestRedraw = true);
	void SetLayerVisible(size_t layerIndex, bool visible);
	void SetAllLayersVisible(bool visible = true);
	void SoloLayer(size_t layerIndex);
	void ActivateNextLayer(int direction);
	// Compare view (swipe or split): when first enabled, assign the active layer to
	// side A and every other layer to side B
	void EnableCompareMode(Window::CompareMode mode);
	// Align every other layer to the active layer with a similarity transform estimated from
	// cameras matched by image name (fallback: preserved SFM image ID); returns true if any layer moved
	bool AlignLayersToActive();
	const MVS::Scene& GetScene() const
	{
		const Layer* layer = GetActiveLayer();
		ASSERT(layer != NULL);
		return layer->scene;
	}
	MVS::Scene& GetScene()
	{
		Layer* layer = GetActiveLayer();
		ASSERT(layer != NULL);
		return layer->scene;
	}
	const ImageArr& GetImages() const
	{
		const Layer* layer = GetActiveLayer();
		ASSERT(layer != NULL);
		return layer->images;
	}
	ImageArr& GetImages()
	{
		Layer* layer = GetActiveLayer();
		ASSERT(layer != NULL);
		return layer->images;
	}
	Window& GetWindow() { return window; }
	MVS::IIndex ImageIdxMVS2Viewer(MVS::IIndex idx) const;

	// Event handlers
	void OnCenterScene(const Point3f& center);
	void OnCastRay(const Point2f& screenPos, const Ray3d& ray, int button, int action, int mods);
	void OnSetCameraViewMode(MVS::IIndex camID);
	void OnSelectPointsByCamera(bool highlightCameraVisiblePoints);

private:
	bool LoadLayer(Layer& layer, const String& fileName, String geometryFileName = {});
	void RefreshLayerState(Layer& layer, bool rebuildImages = true);
	void UpdateWindowTitle();
	void UpdateWindowSceneBounds(bool resetView = true);
	void RefreshVisibleLayers();
	float ComputeVisibleSceneDistance() const;
	void UpdateGeometryModifiedFlag();
	bool BuildMergedVisiblePointCloud(MVS::PointCloud& pointcloud) const;
	bool BuildMergedVisibleMesh(MVS::Mesh& mesh) const;
	void ClearLayers();

	void CropToBounds();
	void TogleSceneBox();
	void PrecomputeTrackBasedNeighbors();
	bool StartWorkflow(WorkflowType type, Layer& layer, SEACAVE::Event* event);
	bool StartNextBatchWorkflow();
	bool batchWorkflowActive{false};
	std::deque<WorkflowType> batchWorkflowQueue;
	EstimateROIWorkflowOptions batchEstimateROIOptions;
	DensifyWorkflowOptions batchDensifyOptions;
	ReconstructMeshWorkflowOptions batchReconstructOptions;
	RefineMeshWorkflowOptions batchRefineOptions;
	TextureMeshWorkflowOptions batchTextureOptions;

	// Workflow finalization (called from main thread after workflow completes)
	void FinalizeWorkflow(bool success);

	static void* ThreadWorker(void*);
};

} // namespace VIEWER
