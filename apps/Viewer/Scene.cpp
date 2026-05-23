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
	MVS::IIndex idx;
	unsigned nMaxResolution;
	bool Run(void*) {
		Image& image = pScene->images[idx];
		ASSERT(image.idx != NO_ID);
		MVS::Image& imageData = pScene->scene.images[image.idx];
		ASSERT(imageData.IsValid());
		if (imageData.image.empty() && !imageData.ReloadImage(nMaxResolution))
			return false;
		imageData.UpdateCamera(pScene->scene.platforms);
		image.AssignImage(imageData.image);
		imageData.ReleaseImage();
		glfwPostEmptyEvent();
		return true;
	}
	EVTLoadImage(Scene* _pScene, MVS::IIndex _idx, unsigned _nMaxResolution=0)
		: Event(EVT_JOB), pScene(_pScene), idx(_idx), nMaxResolution(_nMaxResolution) {}
};

// Base class for workflow events
class EventWorkflow : public Event
{
public:
	Scene* pScene;

	EventWorkflow(Scene* _pScene)
		: Event(EVT_JOB), pScene(_pScene) {}

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
	bool Execute() override {
		const auto& options = pScene->estimateROIOptions;
		return pScene->scene.EstimateROI(options.scaleROI, options.upAxis);
	}
	EVTWorkflowEstimateROI(Scene* _pScene) : EventWorkflow(_pScene) {}
};

class EVTWorkflowDensify : public EventWorkflow
{
public:
	bool Execute() override {
		const auto& options = pScene->densifyOptions;
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

		return pScene->scene.DenseReconstruction(options.fusionMode, options.cropToROI, options.borderROI, options.sampleMeshNeighbors);
	}
	EVTWorkflowDensify(Scene* _pScene) : EventWorkflow(_pScene) {}
};

class EVTWorkflowReconstructMesh : public EventWorkflow
{
public:
	bool Execute() override {
		const auto& options = pScene->reconstructOptions;
		MVS::Scene& mvsScene = pScene->scene;

		// Remove point weights if constant weight requested
		if (options.constantWeight)
			mvsScene.pointcloud.pointWeights.Release();

		// Reconstruct mesh
		if (!mvsScene.ReconstructMesh(options.minPointDistance, options.useFreeSpaceSupport, options.useOnlyROI,
			4, options.thicknessFactor, options.qualityFactor))
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
		mvsScene.mesh.Clean(decimate, options.removeSpurious, options.removeSpikes, options.closeHoles, options.smoothSteps, options.edgeLength);
		return true;
	}
	EVTWorkflowReconstructMesh(Scene* _pScene) : EventWorkflow(_pScene) {}
};

class EVTWorkflowRefineMesh : public EventWorkflow
{
public:
	bool Execute() override {
		const auto& options = pScene->refineOptions;
		return pScene->scene.RefineMesh(options.resolutionLevel, options.minResolution, options.maxViews,
			options.decimateMesh, options.closeHoles, options.ensureEdgeSize, options.maxFaceArea,
			options.scales, options.scaleStep, options.alternatePair, options.regularityWeight,
			options.rigidityElasticityRatio, options.gradientStep, options.planarVertexRatio,
			options.reduceMemory);
	}
	EVTWorkflowRefineMesh(Scene* _pScene) : EventWorkflow(_pScene) {}
};

class EVTWorkflowTextureMesh : public EventWorkflow
{
public:
	bool Execute() override {
		const auto& options = pScene->textureOptions;
		MVS::Scene& mvsScene = pScene->scene;

		// Clean and decimate mesh
		float decimate = CLAMP(options.decimateMesh, 0.f, 1.f);
		if (decimate <= 0.f)
			decimate = 1.f;
		mvsScene.mesh.Clean(decimate, 0.f, false, options.closeHoles, 0u, 0.f);

		// Texture mesh
		return mvsScene.TextureMesh(options.resolutionLevel, options.minResolution, options.minCommonCameras,
			options.outlierThreshold, options.ratioDataSmoothness, options.globalSeamLeveling,
			options.localSeamLeveling, options.textureSizeMultiple,
			Pixel8U(options.emptyColor), options.sharpnessWeight, options.ignoreMaskLabel, options.maxTextureSize);
	}
	EVTWorkflowTextureMesh(Scene* _pScene) : EventWorkflow(_pScene) {}
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

Scene::Scene(ARCHIVE_TYPE _nArchiveType)
	: nArchiveType(_nArchiveType)
	, geometryMesh(false)
	, estimateSfMNormals(false)
	, estimateSfMPatches(false)
	, workflowState(WF_STATE_IDLE)
	, currentWorkflowType(WF_NONE)
	, geometryModified(false)
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
	images.Release();
	scene.Release();
	sceneName.clear();
	geometryName.clear();
}
void Scene::Release()
{
	if (window.IsValid())
		window.SetVisible(false);
	if (!thread.isRunning()) {
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

	// init working thread
	thread.start(ThreadWorker);

	// open scene or init empty scene
	if (!fileName.empty())
		Open(fileName, geometryFileName);
	else
		window.SetVisible(true);
	return true;
}

void Scene::Run() {
	window.Run();
}

// Set the view camera from a transform file (12 or 16 whitespace-separated
// values, row-major; camera-to-world: columns are the camera X,Y,Z axes in
// world space, last column is the camera center). Returns false if the file
// is missing or malformed, leaving the current (auto-fit) view unchanged.
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

// Set the view camera to exactly match a scene camera's pose and FOV — what
// the photo would have framed (camIndex is clamped to the available valid
// cameras). Returns false if the scene has no cameras.
bool Scene::SetViewFromCamera(unsigned camIndex) {
	if (!IsOpen() || images.empty()) {
		DEBUG("error: no scene cameras to set the view from");
		return false;
	}
	if (camIndex >= images.size())
		camIndex = images.size() / 2; // sensible default: a central view
	const MVS::Image& imageData = scene.images[images[camIndex].idx];
	// pose + FOV exactly as captured (no camera-view-mode, so no image overlay)
	window.GetCamera().SetCameraFromSceneData(imageData);
	DEBUG("View set from scene camera %u", camIndex);
	return true;
}

double Scene::GetWorkflowElapsedTime() const {
	if (workflowState.load() != WF_STATE_RUNNING || workflowStartTime == 0.0)
		return 0.0;
	return glfwGetTime() - workflowStartTime;
}

void Scene::CheckWorkflowCompletion() {
	const WorkflowState state = workflowState.load();
	if (state == WF_STATE_COMPLETED || state == WF_STATE_FAILED) {
		// Workflow completed, finalize it on the main thread
		const bool success = (state == WF_STATE_COMPLETED);
		FinalizeWorkflow(success);
	}
}

void Scene::FinalizeWorkflow(bool success) {
	SEACAVE::Lock lock(workflowMutex);

	// Check if we need to finalize (already done or not running)
	const WorkflowState state = workflowState.load();
	if (state != WF_STATE_COMPLETED && state != WF_STATE_FAILED)
		return;

	// Calculate duration directly (can't use GetWorkflowElapsedTime since state is no longer RUNNING)
	const double currentTime = glfwGetTime();
	const double duration = (workflowStartTime > 0.0) ? (currentTime - workflowStartTime) : 0.0;
	const WorkflowType type = currentWorkflowType.load();

	// Add to workflow history
	workflowHistory.push_back({type, duration, success});

	if (success) {
		DEBUG("Workflow completed successfully: %s (%.2f seconds)",
			type == WF_ESTIMATE_ROI ? "Estimate ROI" :
			type == WF_DENSIFY ? "Densify" :
			type == WF_RECONSTRUCT ? "Reconstruct Mesh" :
			type == WF_REFINE ? "Refine Mesh" :
			type == WF_TEXTURE ? "Texture Mesh" : "Unknown",
			duration);

		// Re-upload geometry to GPU buffers
		window.UploadRenderData();

		// Mark geometry as modified
		geometryModified.store(true);

		// Request window redraw
		window.RequestRedraw();
	} else {
		DEBUG("Workflow failed: %s",
			type == WF_ESTIMATE_ROI ? "Estimate ROI" :
			type == WF_DENSIFY ? "Densify" :
			type == WF_RECONSTRUCT ? "Reconstruct Mesh" :
			type == WF_REFINE ? "Refine Mesh" :
			type == WF_TEXTURE ? "Texture Mesh" : "Unknown");
	}

	// Reset workflow state
	workflowState.store(WF_STATE_IDLE);
	currentWorkflowType.store(WF_NONE);
	workflowStartTime = 0.0;
}

bool Scene::Open(const String& fileName, String geometryFileName) {
	ASSERT(!fileName.empty());
	window.SetVisible(false);
	DEBUG_EXTRA("Loading: '%s'", Util::getFileNameExt(fileName).c_str());
	Reset();
	sceneName = fileName;

	// load the scene
	if (Util::isFullPath(fileName)) {
		WORKING_FOLDER = Util::getFilePath(fileName);
		INIT_WORKING_FOLDER;
	}
	const MVS::Scene::SCENE_TYPE sceneType(scene.Load(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName), true));
	if (sceneType == MVS::Scene::SCENE_NA) {
		DEBUG("error: can not open scene '%s'", fileName.c_str());
		window.SetVisible(true);
		return false;
	}
	if (geometryFileName.empty() && sceneType == MVS::Scene::SCENE_INTERFACE)
		geometryFileName = Util::getFileFullName(fileName) + _T(".ply");
	if (!geometryFileName.empty()) {
		// try to load given mesh
		MVS::Mesh mesh;
		MVS::PointCloud pointcloud;
		if (mesh.Load(MAKE_PATH_FULL(WORKING_FOLDER_FULL, geometryFileName))) {
			scene.mesh.Swap(mesh);
			geometryName = geometryFileName;
			geometryMesh = true;
		} else
		// try to load as a point-cloud
		if (pointcloud.Load(MAKE_PATH_FULL(WORKING_FOLDER_FULL, geometryFileName))) {
			scene.pointcloud.Swap(pointcloud);
			geometryName = geometryFileName;
			geometryMesh = false;
		}
	}
	if (!scene.pointcloud.IsEmpty()) {
		scene.pointcloud.PrintStatistics(scene.images.data(), &scene.obb);
		if (estimateSfMNormals && scene.EstimatePointCloudNormals())
			if (estimateSfMPatches && scene.mesh.IsEmpty())
				scene.EstimateSparseSurface();
	}

	// init scene
	AABB3f bounds(true);
	Point3f sceneCenter(0, 0, 0);
	if (scene.IsBounded()) {
		bounds = scene.obb.GetAABB();
		sceneCenter = bounds.GetCenter();
	} else {
		if (!scene.pointcloud.IsEmpty()) {
			bounds = scene.pointcloud.GetAABB(0.1f, 0.9f);
			sceneCenter = scene.pointcloud.GetCenter();
		}
		if (!scene.mesh.IsEmpty()) {
			scene.mesh.ComputeNormalFaces();
			bounds.Insert(scene.mesh.GetAABB(0.1f, 0.9f));
			sceneCenter = scene.mesh.GetCenter();
		}
	}

	// init images
	AABB3f imageBounds(true);
	images.Reserve(scene.images.size());
	FOREACH(idxImage, scene.images) {
		const MVS::Image& imageData = scene.images[idxImage];
		if (!imageData.IsValid())
			continue;
		images.emplace_back(idxImage);
		imageBounds.InsertFull(Cast<float>(imageData.camera.C));
	}
	if (bounds.IsEmpty() && !imageBounds.IsEmpty()) {
		// if no geometry is present, use image bounds
		imageBounds.Enlarge(0.5);
		bounds = imageBounds;
		sceneCenter = imageBounds.GetCenter();
	}

	// Precompute neighbors based on shared tracks
	PrecomputeTrackBasedNeighbors();

	// fit camera to scene
	if (!bounds.IsEmpty()) {
		const Point3f sceneSize = bounds.GetSize().cast<float>();
		window.SetSceneBounds(sceneCenter, sceneSize);
	}

	// Set images size for camera view mode
	if (!images.empty()) {
		window.GetCamera().SetMaxCamID(images.size());
		window.GetCamera().SetSceneDistance(scene.ComputeDistanceCameras2Scene(0.1f, true));
	}

	// Set up camera view mode callback
	window.GetCamera().SetCameraViewModeCallback([this](MVS::IIndex camID) {
		OnSetCameraViewMode(camID);
	});

	// set window title
	window.SetTitle(String::FormatString((name + _T(": %s")).c_str(), Util::getFileName(fileName).c_str()));

	// upload render data
	window.UploadRenderData();

	window.SetVisible(true);
	return true;
}

bool Scene::Save(const String& _fileName, bool bRescaleImages) {
	if (!IsOpen())
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
	const String fileName(!_fileName.empty() ? _fileName : Util::insertBeforeFileExt(sceneName, _T("_new")));
	MVS::Mesh mesh;
	if (!scene.mesh.IsEmpty() && !geometryName.empty() && geometryMesh)
		mesh.Swap(scene.mesh);
	MVS::PointCloud pointcloud;
	if (!scene.pointcloud.IsEmpty() && !geometryName.empty() && !geometryMesh)
		pointcloud.Swap(scene.pointcloud);
	if (imageScale > 0 && imageScale < 1) {
		// scale and save images
		const String folderName(Util::getFilePath(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName)) + String::FormatString("images%d" PATH_SEPARATOR_STR, ROUND2INT(imageScale*100)));
		if (!scene.ScaleImages(0, imageScale, folderName)) {
			DEBUG("error: can not scale scene images to '%s'", folderName.c_str());
			return false;
		}
	}
	if (!scene.Save(fileName, nArchiveType)) {
		DEBUG("error: can not save scene to '%s'", fileName.c_str());
		return false;
	}
	if (!mesh.IsEmpty())
		scene.mesh.Swap(mesh);
	if (!pointcloud.IsEmpty())
		scene.pointcloud.Swap(pointcloud);
	sceneName = fileName;
	return true;
}

bool Scene::Export(const String& _fileName, const String& exportType, bool bViews) const {
	if (!IsOpen())
		return false;
	ASSERT(!sceneName.IsEmpty());
	String lastFileName;
	const String fileName(!_fileName.empty() ? _fileName : sceneName);
	const String baseFileName(Util::getFileFullName(fileName));
	const bool bPoints(scene.pointcloud.Save(lastFileName=(baseFileName+_T("_pointcloud")+(!exportType.empty()?exportType.c_str():(Util::getFileExt(fileName)==_T(".glb")?_T(".glb"):_T(".ply")))), nArchiveType==ARCHIVE_MVS && bViews));
	const bool bMesh(scene.mesh.Save(lastFileName=(baseFileName+_T("_mesh")+(!exportType.empty()?exportType.c_str():(Util::getFileExt(fileName)==_T(".obj")?_T(".obj"):_T(".ply")))), {}, true));
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

// Estimate ROI workflow wrapper (async execution)
bool Scene::RunEstimateROIWorkflow(const EstimateROIWorkflowOptions& options) {
	ASSERT(IsOpen() && "No scene loaded");
	ASSERT(scene.pointcloud.IsValid() && "Point-cloud is empty");
	ASSERT(!IsWorkflowRunning() && "A workflow is already running");

	// Update options in scene
	estimateROIOptions = options;

	// Set workflow state
	workflowState.store(WF_STATE_RUNNING);
	currentWorkflowType.store(WF_ESTIMATE_ROI);
	workflowStartTime = glfwGetTime();

	// Submit workflow event to queue for async execution
	events.AddEvent(new EVTWorkflowEstimateROI(this));

	DEBUG("Estimate ROI workflow started (async)");
	return true;
}

// Densify point-cloud workflow wrapper (async execution)
bool Scene::RunDensifyWorkflow(const DensifyWorkflowOptions& options) {
	ASSERT(IsOpen() && "No scene loaded");
	ASSERT(!scene.images.empty() && "Scene has no images");
	ASSERT(!IsWorkflowRunning() && "A workflow is already running");

	// Update options in scene
	densifyOptions = options;

	// Set workflow state
	workflowState.store(WF_STATE_RUNNING);
	currentWorkflowType.store(WF_DENSIFY);
	workflowStartTime = glfwGetTime();

	DEBUG("Densify workflow started (async) at time %.3f", workflowStartTime);

	// Submit workflow event to queue for async execution
	events.AddEvent(new EVTWorkflowDensify(this));

	return true;
}

// Reconstruct mesh workflow wrapper (async execution)
bool Scene::RunReconstructMeshWorkflow(const ReconstructMeshWorkflowOptions& options) {
	ASSERT(IsOpen() && "No scene loaded");
	ASSERT(scene.pointcloud.IsValid() && "Point-cloud is empty");
	ASSERT(!IsWorkflowRunning() && "A workflow is already running");

	// Update options in scene
	reconstructOptions = options;

	// Set workflow state
	workflowState.store(WF_STATE_RUNNING);
	currentWorkflowType.store(WF_RECONSTRUCT);
	workflowStartTime = glfwGetTime();

	// Submit workflow event to queue for async execution
	events.AddEvent(new EVTWorkflowReconstructMesh(this));

	DEBUG("Reconstruct Mesh workflow started (async)");
	return true;
}

// Refine mesh workflow wrapper (async execution)
bool Scene::RunRefineMeshWorkflow(const RefineMeshWorkflowOptions& options) {
	ASSERT(IsOpen() && "No scene loaded");
	ASSERT(!scene.mesh.IsEmpty() && "Mesh is empty");
	ASSERT(!IsWorkflowRunning() && "A workflow is already running");

	// Update options in scene
	refineOptions = options;

	// Set workflow state
	workflowState.store(WF_STATE_RUNNING);
	currentWorkflowType.store(WF_REFINE);
	workflowStartTime = glfwGetTime();

	// Submit workflow event to queue for async execution
	events.AddEvent(new EVTWorkflowRefineMesh(this));

	DEBUG("Refine Mesh workflow started (async)");
	return true;
}

// Texture mesh workflow wrapper (async execution)
bool Scene::RunTextureMeshWorkflow(const TextureMeshWorkflowOptions& options) {
	ASSERT(IsOpen() && "No scene loaded");
	ASSERT(!scene.mesh.IsEmpty() && "Mesh is empty");
	ASSERT(!IsWorkflowRunning() && "A workflow is already running");

	// Update options in scene
	textureOptions = options;

	// Set workflow state
	workflowState.store(WF_STATE_RUNNING);
	currentWorkflowType.store(WF_TEXTURE);
	workflowStartTime = glfwGetTime();

	// Submit workflow event to queue for async execution
	events.AddEvent(new EVTWorkflowTextureMesh(this));

	DEBUG("Texture Mesh workflow started (async)");
	return true;
}

MVS::IIndex Scene::ImageIdxMVS2Viewer(MVS::IIndex idx) const {
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
				// Compute scale ratio (footprint1 / footprint2)
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
			neighbor.sharedPoints = stat.sharedPoints; // Store shared point indices
		}

		neighbors.Sort([](const ViewScoreWithPoints& a, const ViewScoreWithPoints& b) {
			return a.score.points > b.score.points;
		});
	}
}

void Scene::CropToBounds()
{
	if (!IsOpen())
		return;
	if (!scene.IsBounded())
		return;
	const size_t numPoints = scene.pointcloud.points.size();
	const size_t numFaces = scene.mesh.faces.size();
	scene.pointcloud.RemovePointsOutside(scene.obb);
	scene.mesh.RemoveFacesOutside(scene.obb);
	// Mark as modified if anything was removed
	if (numPoints != scene.pointcloud.points.size() || numFaces != scene.mesh.faces.size())
		geometryModified.store(true);
	window.SetSceneBounds(scene.obb.GetCenter(), scene.obb.GetSize());
}

void Scene::TogleSceneBox()
{
	if (!IsOpen())
		return;
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
	if (!IsOpen())
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
		REAL minDist = REAL(FLT_MAX);
		IDX newSelectionIdx = NO_IDX;
		Point3f newSelectionPoints[4];
		const Renderer::PickResult pickResult = window.GetRenderer().PickPrimitiveAt(screenPos, pickRadius, window);
		if (pickResult.IsValid()) {
			if (pickResult.isPoint) {
				window.selectionType = Window::SEL_POINT;
				newSelectionIdx = pickResult.index;
				newSelectionPoints[0] = pickResult.points[0];
				minDist = norm(Point3f(ray.m_pOrig.cast<float>()) - pickResult.points[0]);
			} else {
				window.selectionType = Window::SEL_TRIANGLE;
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
		// check for camera intersection
		const TCone<REAL, 3> cone(ray, D2R(REAL(0.5)));
		const TConeIntersect<REAL, 3> coneIntersect(cone);
		FOREACH(idx, images) {
			const Image& image = images[idx];
			const MVS::Image& imageData = scene.images[image.idx];
			ASSERT(imageData.IsValid());
			REAL dist;
			if (coneIntersect.Classify(imageData.camera.C, dist) == VISIBLE && dist < minDist) {
				window.selectionType = Window::SEL_CAMERA;
				minDist = dist;
				newSelectionIdx = idx;
				newSelectionPoints[0] = newSelectionPoints[3] = imageData.camera.C;
			}
		}
		// check if we have a new selection
		if (window.selectionType != Window::SEL_NA) {
			if (window.selectionType == Window::SEL_CAMERA && (mods & GLFW_MOD_ALT)) {
				// If alt is pressed, set view camera mode
				window.selectionType = prevSelectionType; // Restore previous selection type
				window.GetCamera().SetCameraViewMode(newSelectionIdx);
			} else if (window.selectionType == Window::SEL_CAMERA && (mods & GLFW_MOD_CONTROL)) {
				// If control is pressed, select neighbor camera
				window.selectedNeighborCamera = newSelectionIdx;
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
				MVS::Mesh::Face face(IsWorkflowRunning() ? MVS::Mesh::Face() :  scene.mesh.faces[newSelectionIdx]);
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
	if (!IsOpen() || camID >= images.size())
		return;

	// Save current camera state if entering camera view mode for the first time
	if (!window.GetCamera().IsCameraViewMode())
		window.GetCamera().SaveCurrentState();
	window.GetCamera().SetCurrentCamID(camID);

	// Get the Image from images and then access the MVS::Image via its index
	Image& image = images[camID];
	const MVS::Image& imageData = scene.images[image.idx];

	// Load the image if not already loaded
	if (!image.IsValid() && !image.IsImageLoading()) {
		// Load image asynchronously
		image.SetImageLoading();
		events.AddEvent(new EVTLoadImage(this, camID, IMAGE_MAX_RESOLUTION));
	}

	// Update camera with the scene data and viewport
	window.GetCamera().SetCameraFromSceneData(imageData);
}

void Scene::OnSelectPointsByCamera(bool highlightCameraVisiblePoints) {
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
	if (!window.GetSelectionController().hasSelection())
		return;

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
		geometryModified.store(true);
		window.UploadRenderData();
	}
}

// Set the ROI (region of interest) based on the current selection
//  - aabb: if true, use axis-aligned bounding box; if false, use oriented bounding box
void Scene::SetROIFromSelection(bool aabb) {
	if (!IsOpen())
		return;

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
	if (!IsOpen())
		return;
	scene.obb = OBB3f(true); // zero-extent => IsValid() == false
	window.GetRenderer().UploadBounds(scene);
	window.RequestRedraw();
}

// Replace the scene bounding box and refresh GPU buffers.
// See ClearBoundingBox() for the rationale.
void Scene::SetBoundingBox(const OBB3f& obb) {
	if (!IsOpen())
		return;
	scene.obb = obb;
	window.GetRenderer().UploadBounds(scene);
	window.RequestRedraw();
}

// Crop scene to only images that see at least minPoints of the selected points
MVS::Scene Scene::CropToPoints(const MVS::PointCloud::IndexArr& selectedPointIndices, unsigned minPoints) const {
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
