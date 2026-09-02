/*
 * Scene.cpp
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

#include "Common.h"
#include "Scene.h"
#include "../../libs/MVS/SceneRefineCommon.h"

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define VIEWER_USE_OPENMP
#endif

#define IMAGE_MAX_RESOLUTION 1024


// S T R U C T S ///////////////////////////////////////////////////

enum EVENT_TYPE {
	EVT_JOB = 0,
	EVT_CLOSE,
};

class EVTClose : public Event
{
public:
	EVTClose() : Event(EVT_CLOSE) {}
};

class EVTLoadImage : public Event
{
public:
	Scene* pScene;
	uint32_t layerID;
	MVS::IIndex idx;
	unsigned nMaxResolution;
	bool Run(void*) {
		const auto finish = [&](bool success) {
			pScene->pendingImageLoads.fetch_sub(1);
			return success;
		};
		Scene::Layer* layer = pScene->GetLayerByID(layerID);
		if (layer == NULL || idx >= layer->images.size())
			return finish(false);
		Image& image = layer->images[idx];
		ASSERT(image.idx != NO_ID);
		MVS::Image& imageData = layer->scene.images[image.idx];
		ASSERT(imageData.IsValid());
		if (imageData.image.empty() && !imageData.ReloadImage(nMaxResolution)) {
			image.CancelImageLoading();
			return finish(false);
		}
		imageData.UpdateCamera(layer->scene.platforms);
		image.AssignImage(imageData.image);
		imageData.ReleaseImage();
		glfwPostEmptyEvent();
		return finish(true);
	}
	EVTLoadImage(Scene* _pScene, uint32_t _layerID, MVS::IIndex _idx, unsigned _nMaxResolution = 0) : Event(EVT_JOB), pScene(_pScene), layerID(_layerID), idx(_idx), nMaxResolution(_nMaxResolution) {}
};

// Base class for workflow events
class EventWorkflow : public Event
{
public:
	Scene* pScene;
	uint32_t layerID;

	EventWorkflow(Scene* _pScene, uint32_t _layerID) : Event(EVT_JOB), pScene(_pScene), layerID(_layerID) {}

	virtual ~EventWorkflow() {}

	// Execute the workflow (must be implemented by derived classes)
	virtual bool Execute() = 0;

	// Run wrapper that handles state management
	bool Run(void*) final {
		const bool success = Execute();
		// Update workflow state
		pScene->workflowState.store(success ? Scene::WF_STATE_COMPLETED : Scene::WF_STATE_FAILED);
		// Signal completion
		glfwPostEmptyEvent();
		return success;
	}
};

// Workflow event classes
class EVTWorkflowEstimateROI : public EventWorkflow
{
public:
	const Scene::EstimateROIWorkflowOptions options;

	bool Execute() override {
		Scene::Layer* layer = pScene->GetLayerByID(layerID);
		if (layer == NULL)
			return false;
		return layer->scene.EstimateROI(options.scaleROI, options.upAxis);
	}
	EVTWorkflowEstimateROI(Scene* _pScene, uint32_t _layerID, const Scene::EstimateROIWorkflowOptions& _options)
		: EventWorkflow(_pScene, _layerID), options(_options) {}
};

class EVTWorkflowDensify : public EventWorkflow
{
public:
	const Scene::DensifyWorkflowOptions options;

	bool Execute() override {
		Scene::Layer* layer = pScene->GetLayerByID(layerID);
		if (layer == NULL)
			return false;
		// Set MVS options
		MVS::OPTDENSE::init();
		MVS::OPTDENSE::update();
		MVS::OPTDENSE::nResolutionLevel = options.resolutionLevel;
		MVS::OPTDENSE::nMaxResolution = options.maxResolution;
		MVS::OPTDENSE::nMinResolution = options.minResolution;
		MVS::OPTDENSE::nSubResolutionLevels = options.subResolutionLevels;
		MVS::OPTDENSE::nNumViews = options.numViews;
		MVS::OPTDENSE::nMinViews = MAXF(1u, options.minViews);
		MVS::OPTDENSE::nMinViewsTrustPoint = MAXF(1u, options.minViewsTrust);
		MVS::OPTDENSE::nMinViewsFuse = MAXF(1u, options.minViewsFuse);
		MVS::OPTDENSE::nEstimationIters = MAXF(1u, options.estimationIters);
		MVS::OPTDENSE::nEstimationGeometricIters = options.geometricIters;
		MVS::OPTDENSE::nFuseFilter = CLAMP(options.fuseFilter, 0u, (unsigned)MVS::OPTDENSE::FUSE_DENSEFILTER);
		MVS::OPTDENSE::fDepthReprojectionErrorThreshold = options.fDepthReprojectionErrorThreshold;
		MVS::OPTDENSE::nEstimateColors = options.estimateColors ? 2u : 0u;
		MVS::OPTDENSE::nEstimateNormals = options.estimateNormals ? 2u : 0u;
		MVS::OPTDENSE::bRemoveDmaps = options.removeDepthMaps;
		MVS::OPTDENSE::nOptimize = options.postprocess ? (unsigned)MVS::OPTDENSE::OPTIMIZE : 0u;

		return layer->scene.DenseReconstruction(options.fusionMode, options.cropToROI, options.borderROI, options.sampleMeshNeighbors);
	}
	EVTWorkflowDensify(Scene* _pScene, uint32_t _layerID, const Scene::DensifyWorkflowOptions& _options)
		: EventWorkflow(_pScene, _layerID), options(_options) {}
};

class EVTWorkflowReconstructMesh : public EventWorkflow
{
public:
	const Scene::ReconstructMeshWorkflowOptions options;

	bool Execute() override {
		Scene::Layer* layer = pScene->GetLayerByID(layerID);
		if (layer == NULL)
			return false;
		MVS::Scene& mvsScene = layer->scene;

		// Remove point weights if constant weight requested
		if (options.constantWeight)
			mvsScene.pointcloud.pointWeights.Release();

		// Reconstruct mesh
		MVS::Scene::ReconstructMeshParams params;
		params.distInsert = options.minPointDistance;
		params.bUseFreeSpaceSupport = options.useFreeSpaceSupport;
		params.bUseOnlyROI = options.useOnlyROI;
		params.kSigma = options.thicknessFactor;
		params.kQual = options.qualityFactor;
		if (!mvsScene.ReconstructMesh(params))
			return false;

		// Crop to ROI if requested
		if (options.cropToROI && mvsScene.IsBounded()) {
			const size_t numVertices = mvsScene.mesh.vertices.size();
			const size_t numFaces = mvsScene.mesh.faces.size();
			mvsScene.mesh.RemoveFacesOutside(mvsScene.obb);
			VERBOSE("Mesh trimmed to ROI: %u vertices and %u faces removed",
				(unsigned)(numVertices - mvsScene.mesh.vertices.size()),
				(unsigned)(numFaces - mvsScene.mesh.faces.size()));
		}

		// Decimate mesh
		float decimate = options.decimateMesh;
		if (options.targetFaceNum && !mvsScene.mesh.faces.empty())
			decimate = static_cast<float>(options.targetFaceNum) / mvsScene.mesh.faces.size();
		decimate = CLAMP(decimate, 0.f, 1.f);
		if (decimate <= 0.f)
			decimate = 1.f;

		// Clean mesh
		MVS::Mesh::CleanParams cleanParams;
		cleanParams.simplifyTarget = decimate;
		cleanParams.spuriousFactor = options.removeSpurious;
		cleanParams.removeSpikes = options.removeSpikes;
		cleanParams.maxHoleEdges = options.closeHoles;
		cleanParams.smoothIterations = (int)options.smoothSteps;
		cleanParams.edgeLength = options.edgeLength;
		mvsScene.mesh.Clean(cleanParams);
		return true;
	}
	EVTWorkflowReconstructMesh(Scene* _pScene, uint32_t _layerID, const Scene::ReconstructMeshWorkflowOptions& _options)
		: EventWorkflow(_pScene, _layerID), options(_options) {}
};

class EVTWorkflowRefineMesh : public EventWorkflow
{
public:
	const Scene::RefineMeshWorkflowOptions options;

	bool Execute() override {
		Scene::Layer* layer = pScene->GetLayerByID(layerID);
		if (layer == NULL)
			return false;
		// the refiner reads its own configuration space, which nothing else in the Viewer touches:
		// without this the declared defaults (the pixel rejection gates in particular) would stay
		// at the zero every DEFVAR_OPTION starts at
		MVS::OPTREFINE::init();
		MVS::OPTREFINE::update();
		return layer->scene.RefineMesh(options.resolutionLevel, options.minResolution, options.maxViews,
		                               options.decimateMesh, options.closeHoles, options.ensureEdgeSize, options.maxFaceArea,
		                               options.scales, options.scaleStep, options.alternatePair, options.regularityWeight,
		                               options.rigidityElasticityRatio, options.gradientStep, options.planarVertexRatio);
	}
	EVTWorkflowRefineMesh(Scene* _pScene, uint32_t _layerID, const Scene::RefineMeshWorkflowOptions& _options)
		: EventWorkflow(_pScene, _layerID), options(_options) {}
};

class EVTWorkflowTextureMesh : public EventWorkflow
{
public:
	const Scene::TextureMeshWorkflowOptions options;

	bool Execute() override {
		Scene::Layer* layer = pScene->GetLayerByID(layerID);
		if (layer == NULL)
			return false;
		MVS::Scene& mvsScene = layer->scene;

		// Clean and decimate mesh
		float decimate = CLAMP(options.decimateMesh, 0.f, 1.f);
		if (decimate <= 0.f)
			decimate = 1.f;
		MVS::Mesh::CleanParams cleanParams;
		cleanParams.simplifyTarget = decimate;
		cleanParams.maxHoleEdges = options.closeHoles;
		mvsScene.mesh.Clean(cleanParams);

		// Texture mesh
		return mvsScene.TextureMesh(options.resolutionLevel, options.minResolution, options.minCommonCameras,
			options.outlierThreshold, options.ratioDataSmoothness, options.globalSeamLeveling,
			options.localSeamLeveling, options.textureSizeMultiple,
			Pixel8U(options.emptyColor), options.sharpnessWeight, options.ignoreMaskLabel, options.maxTextureSize);
	}
	EVTWorkflowTextureMesh(Scene* _pScene, uint32_t _layerID, const Scene::TextureMeshWorkflowOptions& _options)
		: EventWorkflow(_pScene, _layerID), options(_options) {}
};

void* Scene::ThreadWorker(void*) {
	while (true) {
		CAutoPtr<Event> evt(events.GetEvent());
		switch (evt->GetID()) {
		case EVT_JOB:
			evt->Run();
			break;
		case EVT_CLOSE:
			return NULL;
		default:
			ASSERT("Should not happen!" == NULL);
		}
	}
	return NULL;
}
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

SEACAVE::EventQueue Scene::events;
SEACAVE::Thread Scene::thread;

namespace {
bool IsSceneProjectFile(const String& fileName)
{
	const String ext(Util::getFileExt(fileName).ToLower());
	return ext == _T(".mvs") || ext == _T(".sfm") || ext == _T(".dmap");
}

bool IsGeometryFile(const String& fileName)
{
	const String ext(Util::getFileExt(fileName).ToLower());
	return ext == _T(".ply") || ext == _T(".obj") || ext == _T(".gltf") || ext == _T(".glb");
}

String GetDefaultSaveFileName(const Scene::Layer& layer)
{
	if (IsSceneProjectFile(layer.sceneName))
		return Util::insertBeforeFileExt(layer.sceneName, _T("_new"));
	return Util::getFileFullName(layer.sceneName) + _T("_new.mvs");
}

void ActivateWorkingFolder(const String& folder)
{
	if (folder.empty())
		return;
	WORKING_FOLDER = folder;
	INIT_WORKING_FOLDER;
}

void ActivateWorkingFolder(const Scene::Layer& layer)
{
	ActivateWorkingFolder(layer.workingFolder);
}

String MakeUniqueLayerLabel(const Scene::LayerArr& layers, const String& requestedLabel, uint32_t ignoredLayerID = NO_ID)
{
	const String baseLabel(requestedLabel.empty() ? String(_T("Untitled")) : requestedLabel);
	String label(baseLabel);
	for (unsigned suffix = 2;; ++suffix) {
		bool duplicate = false;
		for (const Scene::Layer& layer : layers) {
			if (layer.id != ignoredLayerID && layer.label == label) {
				duplicate = true;
				break;
			}
		}
		if (!duplicate)
			return label;
		label = String::FormatString(_T("%s (%u)"), baseLabel.c_str(), suffix);
	}
}

unsigned UpdateCameraUncertaintyStatistics(Scene::Layer& layer)
{
	FloatArr sigmas;
	for (const Scene::CameraUncertainty& uncertainty : layer.cameraUncertainty)
		if (uncertainty.state == Scene::CameraUncertainty::COMPUTED)
			sigmas.push_back(uncertainty.MaxPosSigma());
	if (sigmas.empty()) {
		layer.cameraUncertaintyNorm = 0.f;
		layer.cameraUncertaintyAutoScale = 1.f;
		return 0;
	}
	sigmas.Sort();
	layer.cameraUncertaintyNorm = sigmas[(sigmas.size() - 1) * 95 / 100];
	if (layer.cameraUncertaintyNorm <= 0.f)
		layer.cameraUncertaintyNorm = MAXF(sigmas.Last(), 1.f);
	const float sceneExtent = norm(layer.sceneSize);
	const float medianSigma = sigmas[sigmas.size() / 2];
	layer.cameraUncertaintyAutoScale = (sceneExtent > 0.f && medianSigma > 0.f) ?
		MINF(MAXF(0.03f * sceneExtent / medianSigma, 1e-6f), 1e6f) : 1.f;
	return (unsigned)sigmas.size();
}

bool HasNonCollinearPoints(const Point3Arr& points)
{
	if (points.size() < 3)
		return false;
	Eigen::Vector3d mean(Eigen::Vector3d::Zero());
	for (const Point3& point : points)
		mean += static_cast<const Point3::CEVecMap>(point);
	mean /= (double)points.size();
	Eigen::Matrix3d covariance(Eigen::Matrix3d::Zero());
	for (const Point3& point : points) {
		const Eigen::Vector3d centered(static_cast<const Point3::CEVecMap>(point) - mean);
		covariance += centered * centered.transpose();
	}
	const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
	if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
		return false;
	const double largest = solver.eigenvalues()[2];
	return largest > std::numeric_limits<double>::epsilon() && solver.eigenvalues()[1] > largest * 1e-10;
}

template <typename Geometry>
AABB3f ComputeViewerBounds(const Geometry& geometry, size_t elementCount)
{
	// Percentile bounds keep reconstruction outliers from making the useful
	// geometry tiny. Small inputs need their full extent, because trimming only
	// a handful of samples can collapse an axis or discard most of the model.
	constexpr size_t MIN_ROBUST_BOUNDS_ELEMENTS = 100;
	AABB3f bounds(elementCount < MIN_ROBUST_BOUNDS_ELEMENTS
	                  ? geometry.GetAABB()
	                  : geometry.GetAABB(0.1f, 0.9f));
	if (bounds.IsEmpty())
		bounds = geometry.GetAABB();

	float maxExtent = 0.f;
	for (int axis = 0; axis < 3; ++axis) {
		if (!std::isfinite(bounds.ptMin[axis]) || !std::isfinite(bounds.ptMax[axis]))
			return AABB3f(true);
		maxExtent = MAXF(maxExtent, bounds.ptMax[axis] - bounds.ptMin[axis]);
	}
	const float padding = MAXF(maxExtent * 0.005f, 0.001f);
	for (int axis = 0; axis < 3; ++axis) {
		if (bounds.ptMin[axis] < bounds.ptMax[axis])
			continue;
		const float center = (bounds.ptMin[axis] + bounds.ptMax[axis]) * 0.5f;
		bounds.ptMin[axis] = center - padding;
		bounds.ptMax[axis] = center + padding;
	}
	return bounds;
}
} // unnamed namespace

Scene::Scene(ARCHIVE_TYPE _nArchiveType)
	: nArchiveType(_nArchiveType)
	, estimateSfMNormals(false)
	, estimateSfMPatches(false)
	, activeLayerIndex(-1)
	, nextLayerID(1)
	, workflowLayerID(NO_ID)
	, workflowState(WF_STATE_IDLE)
	, currentWorkflowType(WF_NONE)
	, geometryModified(false)
	, pendingImageLoads(0)
	, workflowStartTime(0.0)
{
}

Scene::~Scene() {
	Release();
}

void Scene::Reset()
{
	window.Reset();
	trackBasedNeighbors.Release();
	batchWorkflowActive = false;
	batchWorkflowQueue.clear();
	ClearLayers();
	geometryModified.store(false);
	UpdateWindowTitle();
}

void Scene::Release()
{
	if (window.IsValid())
		window.SetVisible(false);
	if (thread.isRunning()) {
		events.AddEvent(new EVTClose());
		thread.join();
	}
	Reset();
	window.Release();
	glfwTerminate();
}

bool Scene::Initialize(const cv::Size& size, const String& windowName, const String& fileName, const String& geometryFileName) {
	// initialize window
	if (!window.Initialize(size, windowName, *this)) {
		DEBUG("error: Failed to initialize window");
		return false;
	}
	VERBOSE("OpenGL: %s %s", glGetString(GL_RENDERER), glGetString(GL_VERSION));
	name = windowName;
	window.GetCamera().SetCameraViewModeCallback([this](MVS::IIndex camID) {
		OnSetCameraViewMode(camID);
	});

	// init working thread
	thread.start(ThreadWorker);

	// open scene or init empty scene
	if (!fileName.empty()) {
		if (!Open(fileName, geometryFileName))
			return false;
	} else {
		window.SetVisible(true);
	}
	return true;
}

void Scene::Run() {
	window.Run();
}

// Set the view camera from a transform file (12 or 16 whitespace-separated
// values, row-major; camera-to-world: columns are the camera X,Y,Z axes in
// world space, last column is the camera center). Returns false if the file
// is missing or malformed, leaving the current view unchanged.
bool Scene::SetViewFromFile(const String& viewFileName) {
	Matrix3x4 m;
	if (!Util::loadMatrix3x4(viewFileName, m)) {
		DEBUG("error: cannot load view transform from '%s' (expected 12 or 16 values)", viewFileName.c_str());
		return false;
	}
	window.GetCamera().SetCameraFromPose(m);
	DEBUG("View set from '%s'", Util::getFileNameExt(viewFileName).c_str());
	return true;
}

// Set the view camera to exactly match an active-layer scene camera's pose and FOV.
bool Scene::SetViewFromCamera(unsigned camIndex) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL || layer->images.empty()) {
		DEBUG("error: no scene cameras to set the view from");
		return false;
	}
	if (camIndex >= layer->images.size())
		camIndex = layer->images.size() / 2;
	const MVS::Image& imageData = layer->scene.images[layer->images[camIndex].idx];
	window.GetCamera().SetCameraFromSceneData(imageData);
	DEBUG("View set from scene camera %u", camIndex);
	return true;
}

void Scene::ClearLayers()
{
	ASSERT(!HasPendingImageLoads());
	layers.clear();
	activeLayerIndex = -1;
	nextLayerID = 1;
	workflowLayerID = NO_ID;
}

Scene::Layer* Scene::GetLayer(size_t idx)
{
	return idx < layers.size() ? &layers[idx] : NULL;
}

const Scene::Layer* Scene::GetLayer(size_t idx) const
{
	return idx < layers.size() ? &layers[idx] : NULL;
}

Scene::Layer* Scene::GetActiveLayer()
{
	return activeLayerIndex >= 0 && (size_t)activeLayerIndex < layers.size() ? &layers[activeLayerIndex] : NULL;
}

const Scene::Layer* Scene::GetActiveLayer() const
{
	return activeLayerIndex >= 0 && (size_t)activeLayerIndex < layers.size() ? &layers[activeLayerIndex] : NULL;
}

Scene::Layer* Scene::GetLayerByID(uint32_t layerID)
{
	for (Layer& layer : layers)
		if (layer.id == layerID)
			return &layer;
	return NULL;
}

const Scene::Layer* Scene::GetLayerByID(uint32_t layerID) const
{
	for (const Layer& layer : layers)
		if (layer.id == layerID)
			return &layer;
	return NULL;
}

bool Scene::HasVisibleLayers() const
{
	for (const Layer& layer : layers)
		if (layer.visible)
			return true;
	return false;
}

bool Scene::HasCameraUncertainty() const
{
	for (const Layer& layer : layers)
		if (!layer.cameraUncertainty.empty())
			return true;
	return false;
}

void Scene::RefreshLayerState(Layer& layer, bool rebuildImages)
{
	MVS::Scene& scene(layer.scene);
	AABB3f bounds(true);
	AABB3f imageBounds(true);

	if (scene.IsBounded()) {
		bounds = scene.obb.GetAABB();
	} else {
		if (!scene.pointcloud.IsEmpty())
			bounds = ComputeViewerBounds(scene.pointcloud, scene.pointcloud.points.size());
		if (!scene.mesh.IsEmpty()) {
			scene.mesh.ComputeNormalFaces();
			bounds.Insert(ComputeViewerBounds(scene.mesh, scene.mesh.vertices.size()));
		}
	}

	if (rebuildImages)
		layer.images.Release();
	if (rebuildImages || layer.images.empty()) {
		layer.images.Reserve(scene.images.size());
		FOREACH(idxImage, scene.images) {
			const MVS::Image& imageData = scene.images[idxImage];
			if (!imageData.IsValid())
				continue;
			layer.images.emplace_back(idxImage);
			imageBounds.InsertFull(Cast<float>(imageData.camera.C));
		}
	} else {
		for (const Image& image : layer.images)
			imageBounds.InsertFull(Cast<float>(scene.images[image.idx].camera.C));
	}
	if (bounds.IsEmpty() && !imageBounds.IsEmpty()) {
		imageBounds.Enlarge(0.5);
		bounds = imageBounds;
	}

	layer.bounds = bounds;
	if (bounds.IsEmpty())
		layer.sceneSize = Point3f(1, 1, 1);
	else
		layer.sceneSize = Point3f(bounds.GetSize().cast<float>());
	layer.sceneDistance = layer.images.empty() ? 1.f : scene.ComputeDistanceCameras2Scene(0.1f, true);
	if (layer.label.empty()) {
		layer.label = Util::getFileNameExt(layer.sceneName);
	}
}

float Scene::ComputeVisibleSceneDistance() const
{
	float sceneDistance = 1.f;
	bool foundVisibleLayer = false;
	for (const Layer& layer : layers) {
		if (!layer.visible)
			continue;
		sceneDistance = MAXF(sceneDistance, layer.sceneDistance);
		foundVisibleLayer = true;
	}
	if (!foundVisibleLayer) {
		const Layer* layer = GetActiveLayer();
		if (layer != NULL)
			sceneDistance = MAXF(sceneDistance, layer->sceneDistance);
	}
	return sceneDistance;
}

void Scene::UpdateWindowSceneBounds(bool resetView)
{
	if (!window.IsValid())
		return;
	AABB3f bounds(true);
	for (const Layer& layer : layers) {
		if (!layer.visible || layer.bounds.IsEmpty())
			continue;
		bounds.Insert(layer.bounds);
	}
	if (bounds.IsEmpty()) {
		const Layer* layer = GetActiveLayer();
		if (layer != NULL && !layer->bounds.IsEmpty())
			bounds = layer->bounds;
	}
	if (bounds.IsEmpty())
		return;
	if (resetView)
		window.SetSceneBounds(bounds.GetCenter(), bounds.GetSize().cast<float>());
}

void Scene::RefreshVisibleLayers()
{
	window.GetCamera().SetSceneDistance(ComputeVisibleSceneDistance());
	window.UploadRenderData();
}

void Scene::UpdateWindowTitle()
{
	if (!window.IsValid())
		return;
	if (!IsOpen()) {
		window.SetTitle(name);
		return;
	}
	const Layer* activeLayer(GetActiveLayer());
	ASSERT(activeLayer != NULL);
	window.SetTitle(String::FormatString((name + _T(": %s [%u layers]")).c_str(), activeLayer->label.c_str(), (unsigned)layers.size()));
}

void Scene::UpdateGeometryModifiedFlag()
{
	bool modified = false;
	for (const Layer& layer : layers) {
		if (layer.dirty) {
			modified = true;
			break;
		}
	}
	geometryModified.store(modified);
}

void Scene::SetGeometryModified(bool modified)
{
	Layer* layer = GetActiveLayer();
	if (layer != NULL)
		layer->dirty = modified;
	if (modified)
		geometryModified.store(true);
	else
		UpdateGeometryModifiedFlag();
}

bool Scene::SetActiveLayer(size_t layerIndex, bool requestRedraw)
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot change the active layer while background work is running");
		return false;
	}
	if (layerIndex >= layers.size())
		return false;
	if ((int)layerIndex == activeLayerIndex) {
		ActivateWorkingFolder(layers[layerIndex]);
		return true;
	}
	activeLayerIndex = (int)layerIndex;
	const Layer& layer(layers[layerIndex]);
	ActivateWorkingFolder(layer);
	PrecomputeTrackBasedNeighbors();
	window.GetCamera().SetMaxCamID(layer.images.size());
	window.GetCamera().SetSceneDistance(ComputeVisibleSceneDistance());
	window.GetCamera().DisableCameraViewMode();
	window.GetSelectionController().clearSelection();
	window.selectionType = Window::SEL_NA;
	window.ClearSelectionIds();
	window.selectedNeighborCamera = NO_ID;
	window.GetRenderer().UploadSelection(window);
	window.GetRenderer().UploadBounds(layer.scene);
	UpdateWindowTitle();
	if (requestRedraw)
		window.RequestRedraw();
	return true;
}

bool Scene::SetActiveLayerByID(uint32_t layerID, bool requestRedraw)
{
	for (size_t i = 0; i < layers.size(); ++i) {
		if (layers[i].id == layerID)
			return SetActiveLayer(i, requestRedraw);
	}
	return false;
}

void Scene::SetLayerVisible(size_t layerIndex, bool visible)
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot change layer visibility while background work is running");
		return;
	}
	Layer* layer = GetLayer(layerIndex);
	if (layer == NULL || layer->visible == visible)
		return;
	layer->visible = visible;
	RefreshVisibleLayers();
}

void Scene::SetAllLayersVisible(bool visible)
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot change layer visibility while background work is running");
		return;
	}
	bool visibilityChanged = false;
	for (Layer& layer : layers) {
		if (layer.visible != visible) {
			layer.visible = visible;
			visibilityChanged = true;
		}
	}
	if (!visibilityChanged)
		return;
	RefreshVisibleLayers();
}

void Scene::SoloLayer(size_t layerIndex)
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot solo a layer while background work is running");
		return;
	}
	if (layerIndex >= layers.size())
		return;
	bool alreadySolo = layers[layerIndex].visible;
	for (size_t i = 0; i < layers.size(); ++i) {
		if (i == layerIndex)
			continue;
		if (layers[i].visible) {
			alreadySolo = false;
			break;
		}
	}
	if (alreadySolo) {
		for (Layer& layer : layers)
			layer.visible = true;
	} else {
		for (size_t i = 0; i < layers.size(); ++i)
			layers[i].visible = (i == layerIndex);
	}
	if (layers.size() == 1 && alreadySolo)
		return;
	RefreshVisibleLayers();
}

void Scene::ActivateNextLayer(int direction)
{
	if (layers.empty())
		return;
	const int layerCount((int)layers.size());
	const int nextIndex((activeLayerIndex + direction + layerCount) % layerCount);
	SetActiveLayer((size_t)nextIndex);
}

void Scene::EnableCompareMode(Window::CompareMode mode)
{
	const bool wasEnabled = window.IsCompareEnabled();
	window.compareMode = layers.empty() ? Window::COMPARE_DISABLED : mode;
	if (window.IsCompareEnabled() && !wasEnabled) {
		// Default side assignment: active layer on the left (A), everything else on
		// the right (B); switching between swipe and split keeps the assignment.
		const Layer* activeLayer = GetActiveLayer();
		for (Layer& layer : layers)
			layer.compareRight = (&layer != activeLayer);
	}
	window.RequestRedraw();
}

bool Scene::AlignLayersToActive()
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot align layers while background work is running");
		return false;
	}
	const Layer* refLayer = GetActiveLayer();
	if (refLayer == NULL || layers.size() < 2)
		return false;
	// Reference camera centers keyed by image file name and by preserved SFM image ID.
	// Duplicate basenames/IDs are marked ambiguous instead of silently overwriting a
	// correspondence, which could otherwise produce a plausible but incorrect transform.
	struct CameraMatch {
		Point3 center;
		MVS::IIndex imageIdx{NO_ID};
	};
	std::unordered_map<std::string, CameraMatch> nameToCamera;
	std::unordered_map<uint32_t, CameraMatch> idToCamera;
	for (const Image& image : refLayer->images) {
		const MVS::Image& imageData = refLayer->scene.images[image.idx];
		const CameraMatch cameraMatch{Point3(imageData.camera.C), image.idx};
		const std::string imageName(Util::getFileNameExt(imageData.name).ToLower());
		const auto [nameIt, nameInserted] = nameToCamera.emplace(imageName, cameraMatch);
		if (!nameInserted)
			nameIt->second.imageIdx = NO_ID;
		if (imageData.ID != NO_ID) {
			const auto [idIt, idInserted] = idToCamera.emplace(imageData.ID, cameraMatch);
			if (!idInserted)
				idIt->second.imageIdx = NO_ID;
		}
	}
	unsigned alignedLayers = 0;
	for (Layer& layer : layers) {
		if (layer.id == refLayer->id)
			continue;
		Point3Arr points, pointsRef;
		std::unordered_set<MVS::IIndex> matchedRefImages;
		unsigned nameMatches = 0, idMatches = 0;
		for (const Image& image : layer.images) {
			const MVS::Image& imageData = layer.scene.images[image.idx];
			const CameraMatch* match = NULL;
			bool matchedByName = false;
			const auto nameIt = nameToCamera.find(Util::getFileNameExt(imageData.name).ToLower());
			if (nameIt != nameToCamera.end() && nameIt->second.imageIdx != NO_ID && !matchedRefImages.count(nameIt->second.imageIdx)) {
				match = &nameIt->second;
				matchedByName = true;
			}
			if (match == NULL && imageData.ID != NO_ID) {
				const auto idIt = idToCamera.find(imageData.ID);
				if (idIt != idToCamera.end() && idIt->second.imageIdx != NO_ID && !matchedRefImages.count(idIt->second.imageIdx))
					match = &idIt->second;
			}
			if (match == NULL)
				continue;
			points.emplace_back(imageData.camera.C);
			pointsRef.emplace_back(match->center);
			matchedRefImages.emplace(match->imageIdx);
			if (matchedByName)
				++nameMatches;
			else
				++idMatches;
		}
		if (points.size() < 3) {
			DEBUG("Layer '%s' not aligned: only %u camera(s) match the active layer", layer.label.c_str(), points.size());
			continue;
		}
		if (!HasNonCollinearPoints(points) || !HasNonCollinearPoints(pointsRef)) {
			DEBUG("Layer '%s' not aligned: the %u matched camera centers are coincident or collinear", layer.label.c_str(), points.size());
			continue;
		}
		const Matrix4x4 transform = SimilarityTransform(points, pointsRef);
		if (!static_cast<const Matrix4x4::CEMatMap>(transform).allFinite()) {
			DEBUG("Layer '%s' not aligned: similarity estimation produced a non-finite transform", layer.label.c_str());
			continue;
		}
		Matrix3x3 rotation; Point3 translation; REAL scale;
		DecomposeSimilarityTransform(transform, rotation, translation, scale);
		if (!std::isfinite(scale) || scale <= std::numeric_limits<REAL>::epsilon()) {
			DEBUG("Layer '%s' not aligned: invalid estimated scale %g", layer.label.c_str(), scale);
			continue;
		}
		layer.scene.Transform(rotation, translation, scale);
		RefreshLayerState(layer, false);
		if (!layer.cameraUncertainty.empty()) {
			const Eigen::Matrix3f R = static_cast<const Matrix3x3::CEMatMap>(rotation).cast<float>();
			for (CameraUncertainty& uncertainty : layer.cameraUncertainty) {
				if (uncertainty.state != CameraUncertainty::COMPUTED)
					continue;
				Eigen::Matrix3f covariance = static_cast<const Matrix3x3f::CEMatMap>(uncertainty.posCov);
				covariance = (float)SQUARE(scale) * R * covariance * R.transpose();
				covariance = (covariance + covariance.transpose()) * 0.5f;
				uncertainty.posCov = covariance;
				uncertainty.posSigma = Point3f(
					SQRT(MAXF(covariance(0, 0), 0.f)),
					SQRT(MAXF(covariance(1, 1), 0.f)),
					SQRT(MAXF(covariance(2, 2), 0.f)));
			}
			UpdateCameraUncertaintyStatistics(layer);
		}
		layer.dirty = true;
		++alignedLayers;
		DEBUG("Layer '%s' aligned to '%s' using %u matched cameras (%u by name, %u by ID; scale %g)",
			layer.label.c_str(), refLayer->label.c_str(), points.size(), nameMatches, idMatches, scale);
	}
	if (alignedLayers == 0)
		return false;
	UpdateGeometryModifiedFlag();
	// Refit to the unchanged reference layer. Loading an initially displaced layer
	// may have framed very large combined bounds, leaving the aligned result tiny or
	// off-center even though the transform itself succeeded.
	if (!refLayer->bounds.IsEmpty())
		window.SetSceneBounds(refLayer->bounds.GetCenter(), refLayer->bounds.GetSize().cast<float>());
	else
		UpdateWindowSceneBounds(true);
	window.GetCamera().SetSceneDistance(ComputeVisibleSceneDistance());
	window.UploadRenderData();
	return true;
}

bool Scene::LoadLayer(Layer& layer, const String& fileName, String geometryFileName)
{
	ASSERT(!fileName.empty());
	const String sceneFileName(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
	layer.sceneName = sceneFileName;
	layer.workingFolder = Util::getFilePath(sceneFileName);
	layer.label = Util::getFileNameExt(sceneFileName);
	ActivateWorkingFolder(layer);

	const MVS::Scene::SCENE_TYPE sceneType(layer.scene.Load(sceneFileName, true));
	if (sceneType == MVS::Scene::SCENE_NA) {
		DEBUG("error: can not open scene '%s'", sceneFileName.c_str());
		return false;
	}
	if (geometryFileName.empty() && sceneType == MVS::Scene::SCENE_INTERFACE) {
		const String defaultGeometryFileName(Util::getFileFullName(sceneFileName) + _T(".ply"));
		if (File::isFile(defaultGeometryFileName))
			geometryFileName = defaultGeometryFileName;
	}
	if (!geometryFileName.empty()) {
		const String geometryPath(MAKE_PATH_FULL(layer.workingFolder, geometryFileName));
		MVS::Mesh mesh;
		MVS::PointCloud pointcloud;
		if (mesh.Load(geometryPath)) {
			layer.scene.mesh.Swap(mesh);
		} else if (pointcloud.Load(geometryPath)) {
			layer.scene.pointcloud.Swap(pointcloud);
		}
	}
	if (!layer.scene.pointcloud.IsEmpty()) {
		layer.scene.pointcloud.PrintStatistics(layer.scene.images.data(), &layer.scene.obb);
		layer.usePointSolidColor = layer.scene.pointcloud.colors.empty();
		if (estimateSfMNormals && layer.scene.EstimatePointCloudNormals())
			if (estimateSfMPatches && layer.scene.mesh.IsEmpty())
				layer.scene.EstimateSparseSurface();
	}
	RefreshLayerState(layer, true);
	return true;
}

bool Scene::Open(const String& fileName, String geometryFileName)
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot open a scene while background work is running");
		return false;
	}
	Reset();
	return AddLayer(fileName, geometryFileName, true);
}

bool Scene::OpenFiles(const std::vector<String>& fileNames, bool replaceExisting)
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot open layers while background work is running");
		return false;
	}
	if (fileNames.empty())
		return false;
	if (replaceExisting)
		Reset();

	bool loadedAny = false;
	if (fileNames.size() == 2) {
		const String& firstFile(fileNames.front());
		const String& secondFile(fileNames.back());
		const bool firstIsScene(IsSceneProjectFile(firstFile));
		const bool secondIsScene(IsSceneProjectFile(secondFile));
		const bool firstIsGeometry(IsGeometryFile(firstFile));
		const bool secondIsGeometry(IsGeometryFile(secondFile));
		if (firstIsScene && secondIsGeometry && !secondIsScene) {
			loadedAny = AddLayer(firstFile, secondFile, true);
		} else if (secondIsScene && firstIsGeometry && !firstIsScene) {
			loadedAny = AddLayer(secondFile, firstFile, true);
		}
	}
	if (!loadedAny) {
		for (const String& fileName : fileNames) {
			if (!IsSceneProjectFile(fileName) && !IsGeometryFile(fileName))
				continue;
			loadedAny = AddLayer(fileName, String(), !loadedAny) || loadedAny;
		}
	}
	if (!loadedAny && replaceExisting)
		window.SetVisible(true);
	return loadedAny;
}

bool Scene::AddLayer(const String& fileName, String geometryFileName, bool makeActive)
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot add a layer while background work is running");
		return false;
	}
	ASSERT(!fileName.empty());
	window.SetVisible(false);
	DEBUG_EXTRA("Loading layer: '%s'", Util::getFileNameExt(fileName).c_str());

	Layer layer;
	layer.id = nextLayerID++;
	if (!LoadLayer(layer, fileName, geometryFileName)) {
		if (const Layer* activeLayer = GetActiveLayer())
			ActivateWorkingFolder(*activeLayer);
		window.SetVisible(true);
		return false;
	}
	layer.label = MakeUniqueLayerLabel(layers, layer.label);
	layers.emplace_back(std::move(layer));
	const bool activeLayerChanged(makeActive || activeLayerIndex < 0);
	if (activeLayerChanged)
		activeLayerIndex = (int)layers.size() - 1;
	const Layer& activeLayer(*GetActiveLayer());
	ActivateWorkingFolder(activeLayer);
	if (activeLayerChanged)
		PrecomputeTrackBasedNeighbors();
	window.GetCamera().SetMaxCamID(activeLayer.images.size());
	window.GetCamera().SetSceneDistance(ComputeVisibleSceneDistance());
	UpdateWindowSceneBounds(true);
	UpdateWindowTitle();
	window.UploadRenderData();
	window.SetVisible(true);
	return true;
}

bool Scene::RemoveLayer(size_t layerIndex)
{
	if (layerIndex >= layers.size())
		return false;
	if (HasBackgroundWork()) {
		DEBUG("Cannot remove a layer while background work is running");
		return false;
	}
	const Layer* previousActiveLayer = GetActiveLayer();
	const uint32_t previousActiveLayerID = previousActiveLayer != NULL ? previousActiveLayer->id : NO_ID;
	const uint32_t removedLayerID = layers[layerIndex].id;
	layers.erase(layers.begin() + layerIndex);
	if (layers.empty()) {
		Reset();
		window.SetVisible(true);
		return true;
	}
	size_t newActiveLayerIndex = MINF(layerIndex, layers.size() - 1);
	if (previousActiveLayerID != NO_ID && previousActiveLayerID != removedLayerID) {
		for (size_t i = 0; i < layers.size(); ++i) {
			if (layers[i].id == previousActiveLayerID) {
				newActiveLayerIndex = i;
				break;
			}
		}
	}
	activeLayerIndex = -1; // force SetActiveLayer() to rebuild dependent state
	SetActiveLayer(newActiveLayerIndex, false);
	UpdateGeometryModifiedFlag();
	UpdateWindowSceneBounds(true);
	window.UploadRenderData();
	return true;
}

bool Scene::Save(const String& _fileName, bool bRescaleImages) {
	if (HasBackgroundWork()) {
		DEBUG("Cannot save a scene while background work is running");
		return false;
	}
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return false;
	MVS::Scene& scene(layer->scene);
	if (!layer->IsOpen())
		return false;
	REAL imageScale = 0;
	if (bRescaleImages) {
		window.SetVisible(false);
		VERBOSE("Enter image resolution scale: ");
		String strScale;
		std::cin >> strScale;
		window.SetVisible(true);
		imageScale = strScale.From<REAL>(0);
	}
	const String requestedFileName(!_fileName.empty() ? _fileName : GetDefaultSaveFileName(*layer));
	const String fileName(MAKE_PATH_FULL(layer->workingFolder, requestedFileName));
	const String previousWorkingFolder(layer->workingFolder);
	const String saveWorkingFolder(Util::getFilePath(fileName));
	ActivateWorkingFolder(saveWorkingFolder);
	if (imageScale > 0 && imageScale < 1) {
		const String folderName(Util::getFilePath(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName)) + String::FormatString("images%d" PATH_SEPARATOR_STR, ROUND2INT(imageScale*100)));
		if (!scene.ScaleImages(0, imageScale, folderName)) {
			DEBUG("error: can not scale scene images to '%s'", folderName.c_str());
			ActivateWorkingFolder(previousWorkingFolder);
			return false;
		}
	}
	if (!scene.Save(fileName, nArchiveType)) {
		DEBUG("error: can not save scene to '%s'", fileName.c_str());
		ActivateWorkingFolder(previousWorkingFolder);
		return false;
	}
	layer->sceneName = fileName;
	layer->workingFolder = saveWorkingFolder;
	layer->label = MakeUniqueLayerLabel(layers, Util::getFileNameExt(fileName), layer->id);
	layer->dirty = false;
	UpdateGeometryModifiedFlag();
	UpdateWindowTitle();
	return true;
}

bool Scene::SaveModifiedLayers()
{
	if (!IsOpen() || HasBackgroundWork())
		return false;
	const int previousActiveLayer = activeLayerIndex;
	bool savedAny = false;
	bool success = true;
	for (size_t i = 0; i < layers.size(); ++i) {
		if (!layers[i].dirty)
			continue;
		SetActiveLayer(i, false);
		success = Save(String(), false) && success;
		savedAny = true;
	}
	if (previousActiveLayer >= 0 && previousActiveLayer < (int)layers.size())
		SetActiveLayer((size_t)previousActiveLayer, false);
	return success && savedAny;
}

bool Scene::Export(const String& _fileName, const String& exportType, bool bViews, ExportGeometry geometry) const {
	if (HasBackgroundWork()) {
		DEBUG("Cannot export a scene while background work is running");
		return false;
	}
	const Layer* layer = GetActiveLayer();
	if (layer == NULL || !layer->IsOpen())
		return false;
	const MVS::Scene& scene(layer->scene);
	ASSERT(!layer->sceneName.IsEmpty());
	String lastFileName;
	const String fileName(!_fileName.empty() ? _fileName : layer->sceneName);
	const String baseFileName(Util::getFileFullName(fileName));
	const bool exportPoints = geometry != EXPORT_MESH && !scene.pointcloud.IsEmpty();
	const bool exportMesh = geometry != EXPORT_POINT_CLOUD && !scene.mesh.IsEmpty() && !scene.mesh.faces.empty();
	const bool bPoints(exportPoints && scene.pointcloud.Save(lastFileName=(baseFileName+_T("_pointcloud")+(!exportType.empty()?exportType.c_str():(Util::getFileExt(fileName)==_T(".glb")?_T(".glb"):_T(".ply")))), nArchiveType==ARCHIVE_MVS && bViews));
	const bool bMesh(exportMesh && scene.mesh.Save(lastFileName=(baseFileName+_T("_mesh")+(!exportType.empty()?exportType.c_str():(Util::getFileExt(fileName)==_T(".obj")?_T(".obj"):_T(".ply")))), {}, true));
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2 && (bPoints || bMesh))
		scene.ExportCamerasMLP(Util::getFileFullName(lastFileName)+_T(".mlp"), lastFileName);
	#endif
	AABB3f aabb(true);
	if (scene.IsBounded()) {
		std::ofstream fs(baseFileName+_T("_roi.txt"));
		if (fs)
			fs << scene.obb;
		aabb = scene.obb.GetAABB();
	} else
	if (!scene.pointcloud.IsEmpty()) {
		aabb = scene.pointcloud.GetAABB();
	} else
	if (!scene.mesh.IsEmpty()) {
		aabb = scene.mesh.GetAABB();
	}
	if (!aabb.IsEmpty()) {
		std::ofstream fs(baseFileName+_T("_roi_box.txt"));
		if (fs)
			fs << aabb;
	}
	return bPoints || bMesh;
}

bool Scene::BuildMergedVisiblePointCloud(MVS::PointCloud& pointcloud) const
{
	pointcloud.Release();
	size_t totalPoints = 0;
	bool exportColors = false;
	bool exportNormals = false;
	for (const Layer& layer : layers) {
		if (!layer.visible || layer.scene.pointcloud.IsEmpty())
			continue;
		totalPoints += layer.scene.pointcloud.points.size();
		exportColors = exportColors || layer.scene.pointcloud.colors.size() == layer.scene.pointcloud.points.size();
		exportNormals = exportNormals || layer.scene.pointcloud.normals.size() == layer.scene.pointcloud.points.size();
	}
	if (totalPoints == 0)
		return false;

	pointcloud.points.Reserve(totalPoints);
	if (exportColors)
		pointcloud.colors.Reserve(totalPoints);
	if (exportNormals)
		pointcloud.normals.Reserve(totalPoints);

	for (const Layer& layer : layers) {
		if (!layer.visible || layer.scene.pointcloud.IsEmpty())
			continue;
		const MVS::PointCloud& layerPointCloud(layer.scene.pointcloud);
		pointcloud.points.Join(layerPointCloud.points);
		if (exportColors) {
			if (layerPointCloud.colors.size() == layerPointCloud.points.size())
				pointcloud.colors.Join(layerPointCloud.colors);
			else {
				FOREACH(i, layerPointCloud.points)
					pointcloud.colors.emplace_back(255, 255, 255);
			}
		}
		if (exportNormals) {
			if (layerPointCloud.normals.size() == layerPointCloud.points.size())
				pointcloud.normals.Join(layerPointCloud.normals);
			else {
				FOREACH(i, layerPointCloud.points)
					pointcloud.normals.emplace_back(0.f, 0.f, 0.f);
			}
		}
	}
	return !pointcloud.IsEmpty();
}

bool Scene::BuildMergedVisibleMesh(MVS::Mesh& mesh) const
{
	mesh.Release();
	bool hasMesh = false;
	for (const Layer& layer : layers) {
		if (!layer.visible || layer.scene.mesh.IsEmpty() || layer.scene.mesh.faces.empty())
			continue;
		MVS::Mesh layerMesh(layer.scene.mesh);
		if (layerMesh.HasTexture()) {
			layerMesh.faceTexcoords.Release();
			layerMesh.faceTexindices.Release();
			layerMesh.texturesDiffuse.Release();
		}
		layerMesh.vertexNormals.Release();
		layerMesh.faceNormals.Release();
		mesh.Join(layerMesh);
		hasMesh = true;
	}
	if (hasMesh)
		mesh.ComputeNormalVertices();
	return hasMesh && !mesh.IsEmpty();
}

bool Scene::ExportVisibleLayers(const String& _fileName, const String& exportType, ExportGeometry geometry) const
{
	if (HasBackgroundWork()) {
		DEBUG("Cannot export layers while background work is running");
		return false;
	}
	if (!HasVisibleLayers())
		return false;
	String lastFileName;
	const Layer* activeLayer(GetActiveLayer());
	const String fileName(!_fileName.empty() ? _fileName : (activeLayer != NULL ? activeLayer->sceneName : String()));
	const String baseFileName(Util::getFileFullName(fileName));

	MVS::PointCloud mergedPointCloud;
	MVS::Mesh mergedMesh;
	const bool hasPoints(geometry != EXPORT_MESH && BuildMergedVisiblePointCloud(mergedPointCloud));
	const bool hasMesh(geometry != EXPORT_POINT_CLOUD && BuildMergedVisibleMesh(mergedMesh));
	if (!hasPoints && !hasMesh)
		return false;

	const bool bPoints = hasPoints && mergedPointCloud.Save(lastFileName = (baseFileName + _T("_pointcloud") + (!exportType.empty() ? exportType.c_str() : (Util::getFileExt(fileName) == _T(".glb") ? _T(".glb") : _T(".ply")))), false);
	const bool bMesh = hasMesh && mergedMesh.Save(lastFileName = (baseFileName + _T("_mesh") + (!exportType.empty() ? exportType.c_str() : (Util::getFileExt(fileName) == _T(".obj") ? _T(".obj") : _T(".ply")))), {}, true);

	AABB3f aabb(true);
	if (bPoints)
		aabb = mergedPointCloud.GetAABB();
	if (bMesh)
		aabb.Insert(mergedMesh.GetAABB());
	if (!aabb.IsEmpty()) {
		std::ofstream fs(baseFileName + _T("_roi_box.txt"));
		if (fs)
			fs << aabb;
	}
	return bPoints || bMesh;
}

double Scene::GetWorkflowElapsedTime() const
{
	if (workflowState.load() != WF_STATE_RUNNING || workflowStartTime == 0.0)
		return 0.0;
	return glfwGetTime() - workflowStartTime;
}

const char* Scene::GetWorkflowName(WorkflowType type, bool shortName)
{
	switch (type) {
	case WF_ESTIMATE_ROI: return shortName ? "ROI" : "Estimate ROI";
	case WF_DENSIFY: return "Densify";
	case WF_RECONSTRUCT: return shortName ? "Reconstruct" : "Reconstruct Mesh";
	case WF_REFINE: return shortName ? "Refine" : "Refine Mesh";
	case WF_TEXTURE: return shortName ? "Texture" : "Texture Mesh";
	default: return shortName ? "?" : "Unknown";
	}
}

void Scene::CheckWorkflowCompletion()
{
	const WorkflowState state = workflowState.load();
	if (state == WF_STATE_COMPLETED || state == WF_STATE_FAILED) {
		// Workflow completed, finalize it on the main thread
		const bool success = (state == WF_STATE_COMPLETED);
		FinalizeWorkflow(success);
	}
}

void Scene::FinalizeWorkflow(bool success)
{
	SEACAVE::Lock lock(workflowMutex);

	// Check if we need to finalize (already done or not running)
	const WorkflowState state = workflowState.load();
	if (state != WF_STATE_COMPLETED && state != WF_STATE_FAILED)
		return;

	// Calculate duration directly (can't use GetWorkflowElapsedTime since state is no longer RUNNING)
	const double currentTime = glfwGetTime();
	const double duration = (workflowStartTime > 0.0) ? (currentTime - workflowStartTime) : 0.0;
	const WorkflowType type = currentWorkflowType.load();
	const char* workflowName(Scene::GetWorkflowName(type));

	// Add to workflow history
	workflowHistory.push_back({type, duration, success});

	if (success) {
		DEBUG("Workflow completed successfully: %s (%.2f seconds)", workflowName, duration);
	} else {
		DEBUG("Workflow failed: %s", workflowName);
	}

	// A failed workflow can still modify its input before reporting failure (for example,
	// mesh cleaning or point-weight removal), so always refresh and mark its layer dirty.
	Layer* layer = GetLayerByID(workflowLayerID);
	if (layer != NULL) {
		RefreshLayerState(*layer, false);
		if (layer == GetActiveLayer())
			PrecomputeTrackBasedNeighbors();
		layer->dirty = true;
		UpdateGeometryModifiedFlag();
		window.GetCamera().SetSceneDistance(ComputeVisibleSceneDistance());
		window.UploadRenderData();
		window.RequestRedraw();
	}

	// Reset workflow state
	workflowState.store(WF_STATE_IDLE);
	currentWorkflowType.store(WF_NONE);
	workflowStartTime = 0.0;
	workflowLayerID = NO_ID;
	if (!success) {
		batchWorkflowActive = false;
		batchWorkflowQueue.clear();
	} else if (!batchWorkflowQueue.empty()) {
		if (!StartNextBatchWorkflow()) {
			batchWorkflowActive = false;
			DEBUG("Batch workflow stopped because the next stage could not start");
		}
	} else if (batchWorkflowActive) {
		batchWorkflowActive = false;
		DEBUG_EXTRA("Workflow queue completed");
	}
}

// Load the per-image pose uncertainty from a CreateStructure pose-quality CSV report
// (--export-pose-quality) and enable the uncertainty-ellipsoids display.
// Rows are matched to the scene images by ID (ExportMVS preserves the SFM image ID).
bool Scene::LoadPoseUncertainty(const String& fileName) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL) {
		DEBUG("error: pose uncertainty requires an open scene");
		return false;
	}
	if (HasBackgroundWork()) {
		DEBUG("Cannot load pose uncertainty while background work is running");
		return false;
	}
	const ImageArr& images(layer->images);
	const MVS::Scene& scene(layer->scene);
	if (images.empty()) {
		DEBUG("error: pose uncertainty requires calibrated images in the active layer");
		return false;
	}
	std::ifstream is(fileName);
	if (!is.is_open()) {
		DEBUG("error: cannot open pose quality report '%s'", fileName.c_str());
		return false;
	}
	// parse the CSV rows: ID,name,valid,datum,sigmaPosX,sigmaPosY,sigmaPosZ,
	// covPosXY,covPosXZ,covPosYZ,sigmaRotX,sigmaRotY,sigmaRotZ[,...extra columns ignored]
	std::unordered_map<uint32_t, CameraUncertainty> mapUncertainty;
	std::string line;
	while (std::getline(is, line)) {
		if (line.empty() || line[0] == '#')
			continue;
		std::vector<std::string> fields;
		size_t start = 0;
		for (size_t pos; (pos = line.find(',', start)) != std::string::npos; start = pos + 1)
			fields.push_back(line.substr(start, pos - start));
		fields.push_back(line.substr(start));
		if (fields.size() < 13)
			continue;
		char* end;
		const unsigned long id = std::strtoul(fields[0].c_str(), &end, 10);
		if (end == fields[0].c_str() || *end != '\0')
			continue; // header or malformed line
		if (fields[2] == "0")
			continue; // image without computed uncertainty
		const Point3f sigmaPos((float)std::atof(fields[4].c_str()), (float)std::atof(fields[5].c_str()), (float)std::atof(fields[6].c_str()));
		if (sigmaPos.x < 0.f || sigmaPos.y < 0.f || sigmaPos.z < 0.f ||
			!std::isfinite(sigmaPos.x) || !std::isfinite(sigmaPos.y) || !std::isfinite(sigmaPos.z))
			continue;
		const Point3f covOff((float)std::atof(fields[7].c_str()), (float)std::atof(fields[8].c_str()), (float)std::atof(fields[9].c_str()));
		if (!std::isfinite(covOff.x) || !std::isfinite(covOff.y) || !std::isfinite(covOff.z))
			continue;
		CameraUncertainty& u = mapUncertainty[(uint32_t)id];
		u.posCov = Matrix3x3f(
			SQUARE(sigmaPos.x), covOff.x, covOff.y,
			covOff.x, SQUARE(sigmaPos.y), covOff.z,
			covOff.y, covOff.z, SQUARE(sigmaPos.z));
		u.posSigma = sigmaPos;
		u.rotSigma = Point3f((float)std::atof(fields[10].c_str()), (float)std::atof(fields[11].c_str()), (float)std::atof(fields[12].c_str()));
		u.state = fields[3] != "0" ? CameraUncertainty::DATUM : CameraUncertainty::COMPUTED;
	}
	if (mapUncertainty.empty()) {
		DEBUG("error: no valid pose uncertainty entries in '%s'", fileName.c_str());
		return false;
	}
	// match the entries to the scene images by ID
	CameraUncertaintyArr loadedUncertainty(images.size());
	unsigned matched = 0;
	FOREACH(i, images) {
		const MVS::Image& imageData = scene.images[images[i].idx];
		const auto it = mapUncertainty.find(imageData.ID);
		if (it == mapUncertainty.end())
			continue;
		loadedUncertainty[i] = it->second;
		++matched;
	}
	if (matched == 0) {
		DEBUG("error: no pose uncertainty entries in '%s' match the scene image IDs", fileName.c_str());
		return false;
	}
	// Auto-size the ellipsoids to the scene: raw 1-sigma radii are in world units and can be far
	// smaller (metric/GPS scenes) or far larger (datum-relative scenes, where the unanchored scale
	// mode saturates) than the scene itself, so a fixed scale renders them sub-pixel or scene-
	// spanning ("nothing visible"). Size against the MEDIAN sigma (not the 95th-pct color norm,
	// which the few worst-localized cameras inflate — that would shrink every typical ellipsoid):
	// scale so the median ellipsoid's largest axis is ~3% of the scene bounding-box diagonal
	// (poorly-localized cameras then stand out proportionally larger, while typical ones stay
	// small enough not to overlap their neighbours at the default x1 slider). This is kept SEPARATE
	// from the user-facing `Window::uncertaintyEllipsoidScale` (which multiplies it, defaulting to 1)
	// so the deferred ImGui-ini load of that persisted slider value cannot clobber the auto fit.
	layer->cameraUncertainty.Swap(loadedUncertainty);
	const unsigned drawable = UpdateCameraUncertaintyStatistics(*layer);
	window.showUncertaintyEllipsoids = true;
	window.GetRenderer().UploadUncertaintyEllipsoids(window);
	Window::RequestRedraw();
	// drawable = COMPUTED entries (datum entries have zero covariance and draw nothing)
	DEBUG("Pose uncertainty loaded from '%s': %u/%u images matched (%u drawable ellipsoids, %u datum), "
		"sigma norm %.3g, auto-fit ellipsoid scale %.3g (x%.3g slider)%s",
		Util::getFileNameExt(fileName).c_str(), matched, images.size(),
		drawable, matched - drawable, layer->cameraUncertaintyNorm, layer->cameraUncertaintyAutoScale,
		window.uncertaintyEllipsoidScale,
		drawable == 0 ? " -- WARNING: nothing to draw (all matched entries are gauge datum)" : "");
	return true;
}

// Estimate ROI workflow wrapper (async execution)
bool Scene::StartWorkflow(WorkflowType type, Layer& layer, SEACAVE::Event* event)
{
	ASSERT(event != NULL && workflowState.load() == WF_STATE_IDLE);
	ActivateWorkingFolder(layer);
	workflowState.store(WF_STATE_RUNNING);
	currentWorkflowType.store(type);
	workflowStartTime = glfwGetTime();
	workflowLayerID = layer.id;
	events.AddEvent(event);
	DEBUG("%s workflow started (async)", GetWorkflowName(type));
	return true;
}

bool Scene::RunEstimateROIWorkflow(const EstimateROIWorkflowOptions& options)
{
	Layer* layer = GetActiveLayer();
	if (layer == NULL || !layer->scene.pointcloud.IsValid() || HasBackgroundWork()) {
		DEBUG("Cannot start Estimate ROI: an active point-cloud layer is required and no background work can be running");
		return false;
	}
	estimateROIOptions = options;
	return StartWorkflow(WF_ESTIMATE_ROI, *layer, new EVTWorkflowEstimateROI(this, layer->id, options));
}

// Densify point-cloud workflow wrapper (async execution)
bool Scene::RunDensifyWorkflow(const DensifyWorkflowOptions& options) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL || !layer->scene.IsValid() || HasBackgroundWork()) {
		DEBUG("Cannot start Densify: an active calibrated-image layer is required and no background work can be running");
		return false;
	}
	densifyOptions = options;
	return StartWorkflow(WF_DENSIFY, *layer, new EVTWorkflowDensify(this, layer->id, options));
}

// Reconstruct mesh workflow wrapper (async execution)
bool Scene::RunReconstructMeshWorkflow(const ReconstructMeshWorkflowOptions& options) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL || !layer->scene.pointcloud.IsValid() || HasBackgroundWork()) {
		DEBUG("Cannot start Reconstruct Mesh: an active point-cloud layer is required and no background work can be running");
		return false;
	}
	reconstructOptions = options;
	return StartWorkflow(WF_RECONSTRUCT, *layer, new EVTWorkflowReconstructMesh(this, layer->id, options));
}

// Refine mesh workflow wrapper (async execution)
bool Scene::RunRefineMeshWorkflow(const RefineMeshWorkflowOptions& options) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL || !layer->scene.IsValid() || layer->scene.mesh.IsEmpty() || HasBackgroundWork()) {
		DEBUG("Cannot start Refine Mesh: an active mesh-and-image layer is required and no background work can be running");
		return false;
	}
	refineOptions = options;
	return StartWorkflow(WF_REFINE, *layer, new EVTWorkflowRefineMesh(this, layer->id, options));
}

// Texture mesh workflow wrapper (async execution)
bool Scene::RunTextureMeshWorkflow(const TextureMeshWorkflowOptions& options) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL || !layer->scene.IsValid() || layer->scene.mesh.IsEmpty() || HasBackgroundWork()) {
		DEBUG("Cannot start Texture Mesh: an active mesh-and-image layer is required and no background work can be running");
		return false;
	}
	textureOptions = options;
	return StartWorkflow(WF_TEXTURE, *layer, new EVTWorkflowTextureMesh(this, layer->id, options));
}

bool Scene::RunBatchWorkflow(const std::vector<WorkflowType>& workflowTypes)
{
	if (workflowTypes.empty() || HasBackgroundWork() || GetActiveLayer() == NULL)
		return false;
	for (WorkflowType type : workflowTypes) {
		if (type <= WF_NONE || type > WF_TEXTURE)
			return false;
	}
	batchWorkflowQueue.assign(workflowTypes.begin(), workflowTypes.end());
	batchWorkflowActive = true;
	batchEstimateROIOptions = estimateROIOptions;
	batchDensifyOptions = densifyOptions;
	batchReconstructOptions = reconstructOptions;
	batchRefineOptions = refineOptions;
	batchTextureOptions = textureOptions;
	if (StartNextBatchWorkflow())
		return true;
	batchWorkflowActive = false;
	batchWorkflowQueue.clear();
	return false;
}

bool Scene::StartNextBatchWorkflow()
{
	if (batchWorkflowQueue.empty())
		return false;
	const WorkflowType type(batchWorkflowQueue.front());
	batchWorkflowQueue.pop_front();
	bool started = false;
	switch (type) {
	case WF_ESTIMATE_ROI: started = RunEstimateROIWorkflow(batchEstimateROIOptions); break;
	case WF_DENSIFY: started = RunDensifyWorkflow(batchDensifyOptions); break;
	case WF_RECONSTRUCT: started = RunReconstructMeshWorkflow(batchReconstructOptions); break;
	case WF_REFINE: started = RunRefineMeshWorkflow(batchRefineOptions); break;
	case WF_TEXTURE: started = RunTextureMeshWorkflow(batchTextureOptions); break;
	default: break;
	}
	if (!started)
		batchWorkflowQueue.clear();
	return started;
}

MVS::IIndex Scene::ImageIdxMVS2Viewer(MVS::IIndex idx) const {
	const Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return NO_ID;
	const ImageArr& images(layer->images);
	// Convert MVS image index to viewer index
	// The list of images in the viewer is a subset of the MVS images,
	// more exactly only the valid images are stored in the viewer.
	// So we can use a small trick to search fast the index in the viewer:
	// start from the MVS index and search backwards
	MVS::IIndex i = MINF(idx+1, images.size());
	while (i-- > 0)
		if (images[i].idx == idx)
			return i;
	return NO_ID;
}

void Scene::PrecomputeTrackBasedNeighbors() {
	trackBasedNeighbors.clear();
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	const ImageArr& images(layer->images);
	const MVS::Scene& scene(layer->scene);
	trackBasedNeighbors.resize(images.size());
	if (!scene.IsValid() || !scene.pointcloud.IsValid() || images.empty())
		return;

	const MVS::PointCloud& pointcloud = scene.pointcloud;

	struct TrackNeighborStats {
		uint32_t points = 0;
		float scaleSum = 0.f;
		float angleSum = 0.f;
		uint32_t sumCount = 0;
		MVS::PointCloud::IndexArr sharedPoints;
	};

	#ifdef VIEWER_USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	for (int_t refViewerIdx = 0; refViewerIdx < (int_t)images.size(); ++refViewerIdx) {
	#else
	FOREACH(refViewerIdx, images) {
	#endif
		const MVS::IIndex refMVS = images[refViewerIdx].idx;
		if (refMVS == NO_ID)
			continue;
		const MVS::Image& refImage = scene.images[refMVS];
		if (!refImage.IsValid())
			continue;

		std::vector<TrackNeighborStats> stats(scene.images.size());
		FOREACH(p, pointcloud.points) {
			const MVS::PointCloud::ViewArr& views = pointcloud.pointViews[p];
			if (views.FindFirst(refMVS) == MVS::PointCloud::ViewArr::NO_INDEX)
				continue;
			const MVS::PointCloud::Point& point = pointcloud.points[p];
			const float refDepth = (float)refImage.camera.PointDepth(point);
			if (refDepth <= 0)
				continue;
			const Point3f V1 = refImage.camera.C - Cast<REAL>(point);
			const float footprint1 = refImage.camera.GetFootprintImage(refDepth);
			for (const MVS::PointCloud::View& view : views) {
				if (view == refMVS)
					continue;
				TrackNeighborStats& stat = stats[view];
				++stat.points;
				stat.sharedPoints.emplace_back(p);
				const MVS::Image& otherImage = scene.images[view];
				const float otherDepth = (float)otherImage.camera.PointDepth(point);
				if (otherDepth <= 0)
					continue;
				const Point3f V2(otherImage.camera.C - Cast<REAL>(point));
				stat.angleSum += ACOS(ComputeAngle(V1.ptr(), V2.ptr()));
				++stat.sumCount;
				const float footprint2 = otherImage.camera.GetFootprintImage(otherDepth);
				stat.scaleSum += footprint1 / footprint2;
			}
		}

		ViewScoreWithPointsArr& neighbors = trackBasedNeighbors[refViewerIdx];
		Point2fArr projs(0, 256);
		const Point2f boundsA(refImage.GetSize());
		FOREACH(view, scene.images) {
			const TrackNeighborStats& stat = stats[view];
			if (stat.points == 0)
				continue;
			const MVS::Image& otherImage = scene.images[view];
			if (!otherImage.IsValid())
				continue;

			float area = 0.f;
			if (!stat.sharedPoints.empty()) {
				const Point2f boundsB(otherImage.GetSize());
				projs.Empty();
				for (const auto pointIdx : stat.sharedPoints) {
					const MVS::PointCloud::Point& point = pointcloud.points[pointIdx];
					if (!otherImage.camera.IsInsideProjectionP(point, boundsB))
						continue;
					const auto [ptA, depth] = refImage.camera.ProjectPointP(point);
					if (depth > 0 && refImage.camera.IsInside(ptA, boundsA))
						projs.emplace_back(ptA);
				}
				if (!projs.empty())
					area = ComputeCoveredArea<float,2,16,false>((const float*)projs.data(), projs.size(), boundsA.ptr());
			}

			ViewScoreWithPoints& neighbor = neighbors.AddEmpty();
			neighbor.score.ID = (uint32_t)view;
			neighbor.score.points = stat.points;
			neighbor.score.scale = stat.sumCount > 0 ? stat.scaleSum / stat.sumCount : 1.f;
			neighbor.score.angle = stat.sumCount > 0 ? stat.angleSum / stat.sumCount : 0.f;
			neighbor.score.area = area;
			neighbor.score.score = (float)stat.points*MAXF(area,0.01f);
			neighbor.sharedPoints = stat.sharedPoints;
		}

		neighbors.Sort([](const ViewScoreWithPoints& a, const ViewScoreWithPoints& b) {
			return a.score.points > b.score.points;
		});
	}
}

void Scene::CropToBounds()
{
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	MVS::Scene& scene(layer->scene);
	if (!scene.IsBounded())
		return;
	const size_t numPoints = scene.pointcloud.points.size();
	const size_t numFaces = scene.mesh.faces.size();
	scene.pointcloud.RemovePointsOutside(scene.obb);
	scene.mesh.RemoveFacesOutside(scene.obb);
	// Mark as modified if anything was removed
	if (numPoints != scene.pointcloud.points.size() || numFaces != scene.mesh.faces.size())
		SetGeometryModified(true);
	RefreshLayerState(*layer, false);
	window.SetSceneBounds(scene.obb.GetCenter(), scene.obb.GetSize());
}

void Scene::TogleSceneBox()
{
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	MVS::Scene& scene(layer->scene);
	if (scene.IsBounded()) {
		ClearBoundingBox();
		return;
	}
	const auto EnlargeAABB = [](AABB3f aabb) {
		return aabb.Enlarge(aabb.GetSize().maxCoeff()*0.03f);
	};
	OBB3f newObb;
	if (!scene.mesh.IsEmpty())
		newObb.Set(EnlargeAABB(scene.mesh.GetAABB(0.1f, 0.9f)));
	else if (!scene.pointcloud.IsEmpty())
		newObb.Set(EnlargeAABB(scene.pointcloud.GetAABB(0.1f, 0.9f)));
	else
		return;
	SetBoundingBox(newObb);
}

void Scene::OnCenterScene(const Point3f& center) {
	if (!IsOpen())
		return;
	if (window.GetControlMode() != Window::CONTROL_ARCBALL)
		return; // Only allow centering in Arcball mode

	// Calculate direction from current target to new center
	const Eigen::Vector3d currentPos = window.GetCamera().GetPosition();
	const Eigen::Vector3d currentTarget = window.GetCamera().GetTarget();

	// Calculate current distance from camera to target
	const double currentDistance = (currentPos - currentTarget).norm();

	// Zoom in by reducing the distance by 25%
	const double zoomFactor = 0.75;
	const double newDistance = currentDistance * zoomFactor;

	// Calculate direction from new target to current camera position
	const Eigen::Vector3d newTarget = Cast<double>(center);
	Eigen::Vector3d direction = (currentPos - newTarget).normalized();

	// If the direction is too small (camera very close to target), use a default direction
	if (direction.norm() < 0.001)
		direction = Eigen::Vector3d(0, 0, 1); // Default to looking along Z axis

	// Calculate new camera position: newTarget + direction * newDistance
	const Eigen::Vector3d newPosition = newTarget + direction * newDistance;

	// Use ArcballControls animation instead of Camera animation
	window.GetArcballControls().animateTo(newPosition, newTarget, /*duration (s)*/ 0.5);
}

void Scene::OnCastRay(const Point2f& screenPos, const Ray3d& ray, int button, int action, int mods) {
	if (!IsOpen() || HasBackgroundWork())
		return;
	const double timeClick(0.2);
	const double timeDblClick(0.4);
	const double now(glfwGetTime());
	const int pickRadius = 3 * window.GetDevicePixelRatio().x(); // pick radius in pixels, adjusted for DPI scaling

	switch (action) {
	case GLFW_PRESS: {
		// remember when the click action started
		window.selectionTimeClick = now;
		break; }
	case GLFW_RELEASE: {
		if (now-window.selectionTimeClick > timeClick) {
			// this is a long click, ignore it
			break;
		}
		if (window.selectionType != Window::SEL_NA && now-window.selectionTime < timeDblClick) {
			// this is a double click, center scene at the selected element
			if (window.selectionType == Window::SEL_CAMERA && window.HasSelectionIds())
				window.GetCamera().SetCameraViewMode(static_cast<MVS::IIndex>(window.GetSelectionId()));
			else {
				window.GetCamera().DisableCameraViewMode();
				OnCenterScene(window.selectionPoints[3]);
			}
			window.selectionTime = now;
			break;
		}
		const Window::SELECTION prevSelectionType = window.selectionType;
		window.selectionType = Window::SEL_NA;
		Window::SELECTION newSelectionType = Window::SEL_NA;
		REAL minDist = REAL(FLT_MAX);
		IDX newSelectionIdx = NO_IDX;
		const Layer* previousActiveLayer = GetActiveLayer();
		const uint32_t previousActiveLayerID = previousActiveLayer != NULL ? previousActiveLayer->id : NO_ID;
		uint32_t newSelectionLayerID = previousActiveLayerID;
		Point3f newSelectionPoints[4]{};
		const Renderer::PickResult pickResult = window.GetRenderer().PickPrimitiveAt(screenPos, pickRadius, window);
		if (pickResult.IsValid()) {
			newSelectionLayerID = pickResult.layerID;
			if (pickResult.isPoint) {
				newSelectionType = Window::SEL_POINT;
				newSelectionIdx = pickResult.index;
				newSelectionPoints[0] = pickResult.points[0];
				minDist = norm(Point3f(ray.m_pOrig.cast<float>()) - pickResult.points[0]);
			} else {
				newSelectionType = Window::SEL_TRIANGLE;
				newSelectionIdx = pickResult.index;
				newSelectionPoints[0] = pickResult.points[0];
				newSelectionPoints[1] = pickResult.points[1];
				newSelectionPoints[2] = pickResult.points[2];
				const Ray3d::TRIANGLE tri(
					Cast<double>(newSelectionPoints[0]),
					Cast<double>(newSelectionPoints[1]),
					Cast<double>(newSelectionPoints[2]));
				if (!ray.Intersects<false>(tri, &minDist))
					minDist = norm(Point3f(ray.m_pOrig.cast<float>()) -
						(pickResult.points[0] + pickResult.points[1] + pickResult.points[2]) / 3.f);
			}
			newSelectionPoints[3] = ray.GetPoint(minDist).cast<float>();
		}
		// Check for camera intersection only when camera geometry is visible.
		if (window.showCameras) {
			const TCone<REAL, 3> cone(ray, D2R(REAL(0.5)));
			const TConeIntersect<REAL, 3> coneIntersect(cone);
			const bool pickCompareRight = screenPos.x >= (float)window.GetCompareSplitX();
			for (const Layer& layer : layers) {
				if (!layer.visible || (window.IsCompareEnabled() && layer.compareRight != pickCompareRight))
					continue;
				FOREACH(idx, layer.images) {
					const Image& image = layer.images[idx];
					const MVS::Image& imageData = layer.scene.images[image.idx];
					ASSERT(imageData.IsValid());
					REAL dist;
					if (coneIntersect.Classify(imageData.camera.C, dist) == VISIBLE && dist < minDist) {
						newSelectionType = Window::SEL_CAMERA;
						minDist = dist;
						newSelectionIdx = idx;
						newSelectionLayerID = layer.id;
						newSelectionPoints[0] = newSelectionPoints[3] = imageData.camera.C;
					}
				}
			}
		}
		// check if we have a new selection
		if (newSelectionType != Window::SEL_NA) {
			const bool selectionLayerChanged = previousActiveLayerID != newSelectionLayerID;
			SetActiveLayerByID(newSelectionLayerID, false);
			Layer* activeLayer = GetActiveLayer();
			ASSERT(activeLayer != NULL && activeLayer->id == newSelectionLayerID);
			const ImageArr& images(activeLayer->images);
			MVS::Scene& scene(activeLayer->scene);
			window.selectionType = newSelectionType;
			if (newSelectionType == Window::SEL_CAMERA && (mods & GLFW_MOD_ALT)) {
				// If alt is pressed, set view camera mode. Keep the previous selection only if it belongs to this layer.
				window.selectionType = selectionLayerChanged ? Window::SEL_NA : prevSelectionType;
				window.GetCamera().SetCameraViewMode(newSelectionIdx);
			} else if (newSelectionType == Window::SEL_CAMERA && (mods & GLFW_MOD_CONTROL)) {
				// If control is pressed, select a neighbor camera when a primary camera is already selected in this layer.
				const bool hasPrimaryCameraSelection = !selectionLayerChanged && prevSelectionType == Window::SEL_CAMERA && window.HasSelectionIds();
				if (!hasPrimaryCameraSelection) {
					window.SetSelectionId(newSelectionIdx);
					window.selectedNeighborCamera = NO_ID;
					window.selectionPoints[0] = newSelectionPoints[0];
					window.selectionPoints[1] = newSelectionPoints[1];
					window.selectionPoints[2] = newSelectionPoints[2];
					window.selectionPoints[3] = newSelectionPoints[3];
					window.selectionTime = now;
				} else {
					window.selectedNeighborCamera = newSelectionIdx;
				}
			} else {
				// Normal selection
				window.SetSelectionId(newSelectionIdx);
				window.selectedNeighborCamera = NO_ID;
				window.selectionPoints[0] = newSelectionPoints[0];
				window.selectionPoints[1] = newSelectionPoints[1];
				window.selectionPoints[2] = newSelectionPoints[2];
				window.selectionPoints[3] = newSelectionPoints[3];
				window.selectionTime = now;
			}
			switch (window.selectionType) {
			case Window::SEL_TRIANGLE: {
				const MVS::Mesh::Face& face(scene.mesh.faces[newSelectionIdx]);
				DEBUG("Face selected:\n\tindex: %u\n\tvertex 1: %u (%g, %g, %g)\n\tvertex 2: %u (%g, %g, %g)\n\tvertex 3: %u (%g, %g, %g)",
					newSelectionIdx,
					face[0], newSelectionPoints[0].x, newSelectionPoints[0].y, newSelectionPoints[0].z,
					face[1], newSelectionPoints[1].x, newSelectionPoints[1].y, newSelectionPoints[1].z,
					face[2], newSelectionPoints[2].x, newSelectionPoints[2].y, newSelectionPoints[2].z
				);
				break; }
			case Window::SEL_POINT: {
				DEBUG("Point selected:\n\tindex: %u (%g, %g, %g)%s",
					newSelectionIdx,
					newSelectionPoints[0].x, newSelectionPoints[0].y, newSelectionPoints[0].z,
					[&]() {
						if (scene.pointcloud.pointViews.empty())
							return String();
						const MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews[newSelectionIdx];
						ASSERT(!views.empty());
						String strViews(String::FormatString("\n\tviews: %u", views.size()));
						FOREACH(v, views) {
							const MVS::PointCloud::View idxImage = views[v];
							if (scene.images.empty()) {
								strViews += String::FormatString("\n\t\tview %u (no image data)", idxImage);
								continue;
							}
							const MVS::Image& imageData = scene.images[idxImage];
							const Point3 x(imageData.camera.TransformPointW2I3(Cast<REAL>(window.selectionPoints[0])));
							const float conf = scene.pointcloud.pointWeights.empty() ? 0.f : scene.pointcloud.pointWeights[newSelectionIdx][v];
							strViews += String::FormatString("\n\t\t%s (%.2f %.2f pixel, %.2f depth, %.2f conf)", Util::getFileNameExt(imageData.name).c_str(), x.x, x.y, x.z, conf);
						}
						return strViews;
					}().c_str()
				);
				break; }
			case Window::SEL_CAMERA: {
				if (!(mods & (GLFW_MOD_ALT | GLFW_MOD_CONTROL)))
					window.GetCamera().DisableCameraViewMode();
				const Image& image = images[newSelectionIdx];
				const MVS::Image& imageData = scene.images[image.idx];
				const MVS::Camera& camera = imageData.camera;
				Point3 eulerAngles;
				camera.R.GetRotationAnglesZYX(eulerAngles.x, eulerAngles.y, eulerAngles.z);
				DEBUG("Camera selected:\n\tindex: %u (ID: %u)\n\tname: %s (mask %s)\n\timage size: %ux%u"
					"\n\tintrinsics: fx %.2f, fy %.2f, cx %.2f, cy %.2f"
					"\n\tposition: %g, %g, %g\n\trotation (deg): %.2f, %.2f, %.2f"
					"\n\taverage depth: %.2g\n\tneighbors: %u",
					image.idx, imageData.ID, Util::getFileNameExt(imageData.name).c_str(),
					imageData.maskName.empty() ? "none" : Util::getFileNameExt(imageData.maskName).c_str(),
					imageData.width, imageData.height,
					camera.K(0, 0), camera.K(1, 1), camera.K(0, 2), camera.K(1, 2),
					camera.C.x, camera.C.y, camera.C.z,
					R2D(eulerAngles.x), R2D(eulerAngles.y), R2D(eulerAngles.z),
					imageData.avgDepth, imageData.neighbors.size()
				);
				break; }
			}
		}
		if (window.selectionType != Window::SEL_NA || prevSelectionType != Window::SEL_NA) {
			window.GetRenderer().UploadSelection(window);
			window.RequestRedraw();
		}
		break; }
	}
}

void Scene::OnSetCameraViewMode(MVS::IIndex camID) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL || camID >= layer->images.size())
		return;

	// Save current camera state if entering camera view mode for the first time
	if (!window.GetCamera().IsCameraViewMode())
		window.GetCamera().SaveCurrentState();
	window.GetCamera().SetCurrentCamID(camID);

	// Get the Image from images and then access the MVS::Image via its index
	Image& image = layer->images[camID];
	const MVS::Image& imageData = layer->scene.images[image.idx];

	// Load the image if not already loaded
	if (!image.IsValid() && !image.IsImageLoading()) {
		// Load image asynchronously
		image.SetImageLoading();
		pendingImageLoads.fetch_add(1);
		events.AddEvent(new EVTLoadImage(this, layer->id, camID, IMAGE_MAX_RESOLUTION));
	}

	// Update camera with the scene data and viewport
	window.GetCamera().SetCameraFromSceneData(imageData);
}

void Scene::OnSelectPointsByCamera(bool highlightCameraVisiblePoints) {
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	MVS::Scene& scene(layer->scene);
	ImageArr& images(layer->images);
	if (!scene.pointcloud.IsValid() || scene.images.empty())
		return;
	SelectionController& selectionController = window.GetSelectionController();
	// Prefer explicit selection of a camera, otherwise use camera-view-mode currentCamID
	MVS::IIndex camViewerIdx = NO_ID;
	if (window.selectionType == Window::SEL_CAMERA && window.HasSelectionIds())
		camViewerIdx = static_cast<MVS::IIndex>(window.GetSelectionId());
	else if (window.GetCamera().IsCameraViewMode())
		camViewerIdx = window.GetCamera().GetCurrentCamID();
	if (!highlightCameraVisiblePoints || camViewerIdx == NO_ID) {
		// Turn off: clear selection highlighting produced by this toggle
		selectionController.clearSelection();
		window.GetRenderer().UploadSelection(window);
		window.RequestRedraw();
		return;
	}
	// Highlight points visible in the current camera
	if (selectionController.getCurrentCameraIdxForHighlight() != camViewerIdx) {
		// Update current camera, recompute
		selectionController.setCurrentCameraIdxForHighlight(camViewerIdx);
		// Map viewer camera index to MVS image index
		const Image& img = images[camViewerIdx];
		// Build list of point indices visible in this image via pointViews
		MVS::PointCloud::IndexArr indices(0, 1024);
		FOREACH(p, scene.pointcloud.points) {
			const MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews[p];
			for (const auto v : views)
				if (v == img.idx) {
					indices.emplace_back(p);
					break;
				}
		}
		// Apply selection to highlight
		selectionController.setSelectedPoints(indices, scene.pointcloud.points.size());
		// Upload selection-related rendering state
		window.GetRenderer().UploadSelection(window);
		window.RequestRedraw();
	}
}
/*----------------------------------------------------------------*/

// Remove selected geometry (points and faces)
void Scene::RemoveSelectedGeometry() {
	if (HasBackgroundWork()) {
		DEBUG("Cannot remove geometry while background work is running");
		return;
	}
	if (!window.GetSelectionController().hasSelection())
		return;
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	MVS::Scene& scene(layer->scene);

	bool bDirtyScene = false;
	SelectionController& selectionController = window.GetSelectionController();

	// Classify geometry based on current selection
	if (!scene.pointcloud.IsEmpty()) {
		// Get selected point indices
		MVS::PointCloud::IndexArr selectedIndices = selectionController.getSelectedPointIndices();
		if (!selectedIndices.empty()) {
			// Remove selected points
			bDirtyScene = true;
			scene.pointcloud.RemovePoints(selectedIndices);
			VERBOSE("Removed %zu selected points", selectedIndices.size());
		}
	}

	if (!scene.mesh.IsEmpty()) {
		// Get selected face indices for removal
		MVS::Mesh::FaceIdxArr selectedIndices = selectionController.getSelectedFaceIndices();
		if (!selectedIndices.empty()) {
			// Remove selected faces
			bDirtyScene = true;
			scene.mesh.RemoveFaces(selectedIndices);
			VERBOSE("Removed %zu selected faces", selectedIndices.size());
		}
	}

	// If any geometry was modified, update the scene
	if (bDirtyScene) {
		SetGeometryModified(true);
		RefreshLayerState(*layer, false);
		window.UploadRenderData();
	}
}

// Set the ROI (region of interest) based on the current selection
//  - aabb: if true, use axis-aligned bounding box; if false, use oriented bounding box
void Scene::SetROIFromSelection(bool aabb) {
	if (HasBackgroundWork()) {
		DEBUG("Cannot set ROI while background work is running");
		return;
	}
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	MVS::Scene& scene(layer->scene);

	SelectionController& selectionController = window.GetSelectionController();
	if (!selectionController.hasSelection())
		return;

	// Collect all selected points for OBB fitting directly as Eigen vectors
	std::vector<OBB3f::POINT> selectedPoints;

	// Add selected point cloud points
	if (!scene.pointcloud.IsEmpty()) {
		MVS::PointCloud::IndexArr selectedIndices = selectionController.getSelectedPointIndices();
		selectedPoints.reserve(selectedPoints.size() + selectedIndices.size());
		for (MVS::PointCloud::Index idx : selectedIndices) {
			if (idx < scene.pointcloud.points.size()) {
				const Point3f& pt = scene.pointcloud.points[idx];
				selectedPoints.emplace_back(pt.x, pt.y, pt.z);
			}
		}
	}

	// Add vertices of selected mesh faces
	if (!scene.mesh.IsEmpty()) {
		MVS::Mesh::FaceIdxArr selectedIndices = selectionController.getSelectedFaceIndices();
		// Reserve space for up to 3 vertices per face (may have duplicates)
		selectedPoints.reserve(selectedPoints.size() + selectedIndices.size() * 3);
		for (uint32_t idx : selectedIndices) {
			if (idx < scene.mesh.faces.size()) {
				const MVS::Mesh::Face& face = scene.mesh.faces[idx];
				// Include all vertices of the selected face
				for (int j = 0; j < 3; ++j) {
					if (face[j] < scene.mesh.vertices.size()) {
						const Point3f& pt = scene.mesh.vertices[face[j]];
						selectedPoints.emplace_back(pt.x, pt.y, pt.z);
					}
				}
			}
		}
	}
	// Check if we found any selected geometry
	if (selectedPoints.empty())
		return;

	// Fit a new OBB to the selected points (aabb=true => axis-aligned fit)
	OBB3f newObb;
	if (aabb) {
		AABB3f aabbBounds;
		aabbBounds.Set(selectedPoints.data(), selectedPoints.size());
		newObb.Set(aabbBounds);
	} else {
		// Use OBB3f's built-in fitting to compute the optimal oriented bounding box
		newObb.Set(selectedPoints.data(), selectedPoints.size(), 32);
	}
	// Add a small margin
	const float margin = newObb.GetSize().maxCoeff() * 0.03f; // 3% margin
	newObb.Enlarge(margin);

	// Commit via the centralizing setter (handles UploadBounds + RequestRedraw)
	SetBoundingBox(newObb);
}

// Clear the scene bounding box, invalidating it so Scene::IsBounded() returns false.
// Centralizes the invariant: any code path that mutates scene.obb must refresh GPU
// buffers and request a redraw. Call this from menu actions, workflows, and controllers.
void Scene::ClearBoundingBox() {
	if (HasBackgroundWork()) {
		DEBUG("Cannot clear the bounding box while background work is running");
		return;
	}
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	MVS::Scene& scene(layer->scene);
	scene.obb = OBB3f(true); // zero-extent => IsValid() == false
	SetGeometryModified(true);
	window.GetRenderer().UploadBounds(scene);
	RefreshLayerState(*layer, false);
	window.RequestRedraw();
}

// Replace the scene bounding box and refresh GPU buffers.
// See ClearBoundingBox() for the rationale.
void Scene::SetBoundingBox(const OBB3f& obb) {
	if (HasBackgroundWork()) {
		DEBUG("Cannot change the bounding box while background work is running");
		return;
	}
	Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return;
	MVS::Scene& scene(layer->scene);
	scene.obb = obb;
	SetGeometryModified(true);
	window.GetRenderer().UploadBounds(scene);
	RefreshLayerState(*layer, false);
	window.RequestRedraw();
}

// Crop scene to only images that see at least minPoints of the selected points
MVS::Scene Scene::CropToPoints(const MVS::PointCloud::IndexArr& selectedPointIndices, unsigned minPoints) const {
	const Layer* layer = GetActiveLayer();
	if (layer == NULL)
		return MVS::Scene();
	const MVS::Scene& scene(layer->scene);
	if (!scene.IsValid() || !scene.pointcloud.IsValid())
		return MVS::Scene(); // Return empty scene

	// Count how many selected points each image sees
	std::unordered_map<MVS::IIndex, unsigned> imageCounts;
	for (MVS::PointCloud::Index pointIdx : selectedPointIndices) {
		const MVS::PointCloud::ViewArr& views = scene.pointcloud.pointViews[pointIdx];
		for (MVS::PointCloud::View imageIdx : views)
			imageCounts[imageIdx]++;
	}

	// Select images that see at least minPoints selected points
	MVS::IIndexArr selectedImageIndices;
	for (const auto& pair : imageCounts)
		if (pair.second >= minPoints)
			selectedImageIndices.emplace_back(pair.first);

	// Create sub-scene with selected images
	if (selectedImageIndices.size() < 2) {
		DEBUG("error: no images see %u or more points from %u selected", minPoints, scene.pointcloud.GetSize());
		return MVS::Scene(); // Return empty scene
	}
	if (selectedImageIndices.size() == scene.images.size()) {
		VERBOSE("Cropping scene: all %u images see at least %u points from %u selected; nothing to do",
			selectedImageIndices.size(), minPoints, scene.pointcloud.GetSize());
		return MVS::Scene(); // If all images are selected, return empty scene
	}
	VERBOSE("Cropping scene: found %u images that see at least %u points from %u selected",
		selectedImageIndices.size(), minPoints, scene.pointcloud.GetSize());
	return scene.SubScene(selectedImageIndices);
}
/*----------------------------------------------------------------*/
