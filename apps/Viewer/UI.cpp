/*
 * UI.cpp
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
#include "UI.h"
#include "Scene.h"
#include "Camera.h"
#include "Window.h"
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <portable-file-dialogs.h>
#include "EmptySceneIcon.h"

using namespace VIEWER;


// D E F I N E S ///////////////////////////////////////////////////

constexpr float PAD = 10.f;
constexpr size_t MAX_UI_LOG_LINES = 9000;


// S T R U C T S ///////////////////////////////////////////////////

UI::UI()
	: showSceneInfo(false)
	, showCameraControls(false)
	, showSelectionControls(false)
	, showRenderSettings(false)
	, showBoundingBoxControls(false)
	, showConsoleOverlay(true)
	, showPerformanceOverlay(true)
	, showWorkflowOverlay(true)
	, showViewportOverlay(true)
	, showSelectionOverlay(true)
	, showAboutDialog(false)
	, showHelpDialog(false)
	, showExportDialog(false)
	, showCameraInfoDialog(false)
	, showSelectionDialog(false)
	, showSavePromptDialog(false)
	, useTrackBasedNeighbors(false)
	, showEstimateROIWorkflow(false)
	, showDensifyWorkflow(false)
	, showReconstructWorkflow(false)
	, showRefineWorkflow(false)
	, showTextureWorkflow(false)
	, showBatchWorkflow(false)
	, showMainMenu(false)
	, menuWasVisible(false)
	, menuTriggerHeight(50.f)
	, lastMenuInteraction(0.0)
	, menuFadeOutDelay(2.f)
	, deltaTime(0.0)
	, frameCount(0)
	, fps(0.f)
{
}

UI::~UI() {
	Release();
}


namespace {
const char* ImGuiGetClipboardText(void* glfwWindow) {
	if (glfwWindow == nullptr)
		return "";
	const char* clipboard = glfwGetClipboardString(static_cast<GLFWwindow*>(glfwWindow));
	return clipboard != nullptr ? clipboard : "";
}
void ImGuiSetClipboardText(void* glfwWindow, const char* text) {
	if (glfwWindow == nullptr)
		return;
	glfwSetClipboardString(static_cast<GLFWwindow*>(glfwWindow), text != nullptr ? text : "");
}
} // namespace

bool UI::Initialize(Window& window, const String& glslVersion) {
	// Setup Dear ImGui context
	#ifndef _RELEASE
	IMGUI_CHECKVERSION();
	#endif
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.GetClipboardTextFn = ImGuiGetClipboardText;
	io.SetClipboardTextFn = ImGuiSetClipboardText;
	io.ClipboardUserData = window.GetGLFWWindow();
	iniPath = Util::getApplicationFolder() + "Viewer.ini";
	io.IniFilename = iniPath.c_str();

	// Try to enable docking (available in ImGui 1.80+)
	#if defined(ImGuiConfigFlags_DockingEnable)
	try {
		// Check if the flag exists by testing if it's defined
		io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
		VERBOSE("Docking enabled");
	} catch (...) {
		VERBOSE("Docking feature not available");
	}
	#endif

	// Try to enable multi-viewport (available in ImGui 1.80+)
	#if defined(ImGuiConfigFlags_ViewportsEnable)
	try {
		io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
		VERBOSE("Multi-viewport enabled");
	} catch (...) {
		VERBOSE("Multi-viewport feature not available");
	}
	#endif

	// Setup Dear ImGui style
	SetupStyle();
	// Setup custom settings handler
	SetupCustomSettings(window);

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window.GetGLFWWindow(), true);
	ImGui_ImplOpenGL3_Init(glslVersion);
	ImGui::LoadIniSettingsFromDisk(io.IniFilename);

	// Register log listener to capture log messages for the in-app console
	GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &UI::RecordLog, this));

	return true;
}

void UI::Release() {
	// Unregister log listener
	GET_LOG().UnregisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &UI::RecordLog, this));

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	// Release embedded icon texture if loaded
	emptySceneIcon.Release();
}

void UI::NewFrame(Window& window) {
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// Handle global keyboard shortcuts
	HandleGlobalKeys(window);

	// Update menu visibility based on mouse position and usage
	UpdateMenuVisibility();
}

void UI::Render(Window& window) {
	ShowConsoleOverlay(window);
	ShowPerformanceOverlay(window);
	ShowWorkflowOverlay(window);
	ShowViewportOverlay(window);
	ShowEmptySceneOverlay(window);
	ShowSelectionOverlay(window);

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	// Update and render additional Platform Windows (if multi-viewport is enabled)
	#ifdef IMGUI_HAS_VIEWPORT
	ImGuiIO& io = ImGui::GetIO();
	if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
		GLFWwindow* backup_current_context = glfwGetCurrentContext();
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
		glfwMakeContextCurrent(backup_current_context);
	}
	#endif
}

void UI::ShowMainMenuBar(Window& window) {
	Scene& scene = window.GetScene();

	// Handle dialogs even when menu is hidden
	if (showAboutDialog)
		ShowAboutDialog();
	if (showHelpDialog)
		ShowHelpDialog();
	if (showExportDialog)
		ShowExportDialog(scene);
	if (showCameraInfoDialog)
		ShowCameraInfoDialog(window);
	if (showSelectionDialog)
		ShowSelectionDialog(window);
	if (showSavePromptDialog)
		ShowSavePromptDialog(window);

	// Only show menu bar if it should be visible
	if (!showMainMenu)
		return;

	if (ImGui::BeginMainMenuBar()) {
		// Update last interaction time when menu bar is actively being used
		if (ImGui::IsWindowHovered() || ImGui::IsAnyItemActive() || ImGui::IsAnyItemFocused())
			lastMenuInteraction = glfwGetTime();

		if (ImGui::BeginMenu("File")) {
			lastMenuInteraction = glfwGetTime(); // Update interaction time when menu is open
			#ifdef __APPLE__
			if (ImGui::MenuItem("Open Scene...", "Cmd+O")) {
			#else
			if (ImGui::MenuItem("Open Scene...", "Ctrl+O")) {
			#endif
				// Open file dialog and load scene if file selected
				window.SetVisible(false);
				String filename, geometryFilename;
				if (ShowOpenFileDialog(filename, geometryFilename))
					scene.Open(filename, geometryFilename);
				window.SetVisible(true);
			}
			#ifdef __APPLE__
			if (ImGui::MenuItem("Save Scene", "Cmd+S", false, scene.IsOpen())) {
			#else
			if (ImGui::MenuItem("Save Scene", "Ctrl+S", false, scene.IsOpen())) {
			#endif
				// Save scene to current file
				scene.Save();
			}
			#ifdef __APPLE__
			if (ImGui::MenuItem("Save Scene As...", "Cmd+Shift+S", false, scene.IsOpen())) {
			#else
			if (ImGui::MenuItem("Save Scene As...", "Ctrl+Shift+S", false, scene.IsOpen())) {
			#endif
				// Always prompt for save location
				window.SetVisible(false);
				String filename;
				if (ShowSaveFileDialog(filename))
					scene.Save(filename);
				window.SetVisible(true);
			}
			#ifdef __APPLE__
			if (ImGui::MenuItem("Save Screenshot...", "Cmd+Shift+P", false, window.IsValid())) {
			#else
			if (ImGui::MenuItem("Save Screenshot", "Ctrl+X", false, window.IsValid())) {
			#endif
				window.SetVisible(false);
				String filename = "screenshot.png";
				if (ShowSaveImageDialog(filename)) {
					if (Util::getFileExt(filename).empty())
						filename += ".png";
					window.RequestScreenshot(filename);
				}
				window.SetVisible(true);
			}
			#ifdef __APPLE__
			if (ImGui::MenuItem("Save Screenshot (with UI)...", "Cmd+Opt+Shift+P", false, window.IsValid())) {
			#else
			if (ImGui::MenuItem("Save Screenshot (with UI)", "Ctrl+Shift+X", false, window.IsValid())) {
			#endif
				window.SetVisible(false);
				String filename = "screenshot.png";
				if (ShowSaveImageDialog(filename)) {
					if (Util::getFileExt(filename).empty())
						filename += ".png";
					window.RequestScreenshot(filename, true);
				}
				window.SetVisible(true);
			}
			#ifdef __APPLE__
			if (ImGui::MenuItem("Close", "Cmd+W", false, scene.IsOpen())) {
			#else
			if (ImGui::MenuItem("Close", "Ctrl+W", false, scene.IsOpen())) {
			#endif
				// Release/reset the currently open scene (keeps the application/window running)
				scene.Reset();
				// Make sure renderer/UI reflect the cleared scene
				window.UploadRenderData();
			}
			ImGui::Separator();
			if (ImGui::MenuItem("Export...", nullptr, false, scene.IsOpen())) {
				// Show export dialog with export format options
				showExportDialog = true;
			}
			ImGui::Separator();
			#ifdef __APPLE__
			if (ImGui::MenuItem("Exit", "Cmd+Q")) {
			#else
			if (ImGui::MenuItem("Exit", "Alt+F4")) {
			#endif
				// Check if geometry was modified and show save prompt
				if (scene.IsGeometryModified()) {
					showSavePromptDialog = true;
				} else {
					glfwSetWindowShouldClose(window.GetGLFWWindow(), GLFW_TRUE);
				}
			}
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("View")) {
			lastMenuInteraction = glfwGetTime(); // Update interaction time when menu is open
			ImGui::MenuItem("Scene Info", "Shift+A", &showSceneInfo);
			ImGui::MenuItem("Camera Info", "Shift+Q", &showCameraInfoDialog);
			ImGui::MenuItem("Camera Controls", "Shift+C", &showCameraControls);
			ImGui::MenuItem("Selection Dialog", "Shift+S", &showSelectionDialog);
			ImGui::MenuItem("Render Settings", "Shift+R", &showRenderSettings);
			ImGui::MenuItem("Bounding Box", "Shift+B", &showBoundingBoxControls);
			ImGui::Separator();
			ImGui::MenuItem("Console", nullptr, &showConsoleOverlay);
			ImGui::MenuItem("Performance Overlay", nullptr, &showPerformanceOverlay);
			ImGui::MenuItem("Workflow Overlay", nullptr, &showWorkflowOverlay);
			ImGui::MenuItem("Viewport Overlay", nullptr, &showViewportOverlay);
			ImGui::MenuItem("Selection Overlay", nullptr, &showSelectionOverlay);
			ImGui::Separator();
			ImGui::MenuItem("Show Point Cloud", "P", &window.showPointCloud);
			ImGui::MenuItem("Show Mesh", "M", &window.showMesh);
			ImGui::MenuItem("Show Cameras", "C", &window.showCameras);
			if (window.showMesh) {
				ImGui::MenuItem("Wireframe", "W", &window.showMeshWireframe);
				ImGui::MenuItem("Textured", "T", &window.showMeshTextured);
			}
			if (ImGui::MenuItem("Show Bounding Box", "B", &window.showBounds))
				window.RequestRedraw();
			ImGui::Separator();
			if (ImGui::MenuItem("Reset Camera", "R"))
				window.ResetView();
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Workflow")) {
			lastMenuInteraction = glfwGetTime();
			const Scene& scene = window.GetScene();
			const bool hasScene = scene.IsOpen();
			const MVS::Scene& mvsScene = scene.GetScene();
			const bool hasImages = hasScene && mvsScene.IsValid();
			const bool hasPoints = hasImages && mvsScene.pointcloud.IsValid();
			const bool hasMesh = hasImages && !mvsScene.mesh.IsEmpty();
			const bool workflowRunning = scene.IsWorkflowRunning();
			const auto addWorkflowEntry = [&](const char* label, bool enabled, bool& toggleFlag, const char* tooltip) {
				// Disable if workflow is running or prerequisites not met
				const bool canRun = enabled && !workflowRunning;
				if (ImGui::MenuItem(label, nullptr, false, canRun))
					toggleFlag = true;
				else if (!canRun && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
					if (workflowRunning)
						ImGui::SetTooltip("A workflow is currently running");
					else
						ImGui::SetTooltip("%s", tooltip);
				}
			};
			addWorkflowEntry("Estimate ROI", hasPoints, showEstimateROIWorkflow, "Requires calibrated images and point-cloud.");
			if (ImGui::MenuItem("Recompute Bounding Box", "Ctrl+B", false, hasPoints && !workflowRunning))
				window.GetScene().RunEstimateROIWorkflow(window.GetScene().GetEstimateROIWorkflowOptions());
			else if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				ImGui::SetTooltip("Run the ROI estimation workflow with the current options (same as Ctrl+B)");
			if (ImGui::MenuItem("Set Bounding Box from Selection", nullptr, false, hasScene))
				window.GetScene().SetROIFromSelection(false);
			else if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				ImGui::SetTooltip("Fit an oriented bounding box to the currently selected points/faces");
			if (ImGui::MenuItem("Clear Bounding Box", nullptr, false, hasScene && mvsScene.IsBounded()))
				window.GetScene().ClearBoundingBox();
			else if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				ImGui::SetTooltip("Remove the scene's bounding box");
			ImGui::Separator();
			addWorkflowEntry("Densify Point Cloud", hasImages, showDensifyWorkflow, "Requires calibrated images.");
			addWorkflowEntry("Reconstruct Mesh", hasPoints, showReconstructWorkflow, "Requires a dense point-cloud.");
			addWorkflowEntry("Refine Mesh", hasMesh, showRefineWorkflow, "Requires an existing mesh.");
			addWorkflowEntry("Texture Mesh", hasMesh, showTextureWorkflow, "Requires a mesh and images.");
			ImGui::Separator();
			addWorkflowEntry("Batch Process", hasImages, showBatchWorkflow, "Requires calibrated images.");
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Help")) {
			lastMenuInteraction = glfwGetTime(); // Update interaction time when menu is open
			if (ImGui::MenuItem("Help", "F1"))
				showHelpDialog = true;
			ImGui::Separator();
			if (ImGui::MenuItem("About"))
				showAboutDialog = true;
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}
}

void UI::ShowSceneInfo(const Window& window) {
	if (!showSceneInfo) return;
	const MVS::Scene& scene = window.GetScene().GetScene();

	ImGui::SetNextWindowPos(ImVec2(10, 110), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(240, 410), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Scene Info", &showSceneInfo)) {
		ImGui::Text("Scene Statistics");
		ImGui::Separator();
		ImGui::Text("Images: %u valid (%u total)", scene.nCalibratedImages, scene.images.size());
		ImGui::Text("Platforms: %u", scene.platforms.size());
		ImGui::Text("OBB: %s", scene.obb.IsValid() ? "valid" : "NA");
		// Show full obb
		if (scene.obb.IsValid() && ImGui::CollapsingHeader("Oriented Bounding-Box")) {
			ImGui::Text("  rot1: [%.6f  %.6f  %.6f]", scene.obb.m_rot(0, 0), scene.obb.m_rot(0, 1), scene.obb.m_rot(0, 2));
			ImGui::Text("  rot2: [%.6f  %.6f  %.6f]", scene.obb.m_rot(1, 0), scene.obb.m_rot(1, 1), scene.obb.m_rot(1, 2));
			ImGui::Text("  rot3: [%.6f  %.6f  %.6f]", scene.obb.m_rot(2, 0), scene.obb.m_rot(2, 1), scene.obb.m_rot(2, 2));
			ImGui::Text("  pos : [%.6f  %.6f  %.6f]", scene.obb.m_pos.x(), scene.obb.m_pos.y(), scene.obb.m_pos.z());
			ImGui::Text("  ext : [%.6f  %.6f  %.6f]", scene.obb.m_ext.x(), scene.obb.m_ext.y(), scene.obb.m_ext.z());
		}
		ImGui::Text("Transform: %s", scene.HasTransform() ? "valid" : "NA");
		// Show full transform
		if (scene.HasTransform() && ImGui::CollapsingHeader("Transform")) {
			ImGui::Text("  [%.6f  %.6f  %.6f  %.6f]", scene.transform(0, 0), scene.transform(0, 1), scene.transform(0, 2), scene.transform(0, 3));
			ImGui::Text("  [%.6f  %.6f  %.6f  %.6f]", scene.transform(1, 0), scene.transform(1, 1), scene.transform(1, 2), scene.transform(1, 3));
			ImGui::Text("  [%.6f  %.6f  %.6f  %.6f]", scene.transform(2, 0), scene.transform(2, 1), scene.transform(2, 2), scene.transform(2, 3));
			ImGui::Text("  [%.6f  %.6f  %.6f  %.6f]", scene.transform(3, 0), scene.transform(3, 1), scene.transform(3, 2), scene.transform(3, 3));
		}

		if (!scene.pointcloud.IsEmpty()) {
			ImGui::Separator();
			ImGui::Text("Point Cloud Statistics");
			ImGui::Separator();
			ImGui::Text("Points: %zu", scene.pointcloud.points.size());
			ImGui::Text("Point Views: %zu", scene.pointcloud.pointViews.size());
			ImGui::Text("Point Weights: %zu", scene.pointcloud.pointWeights.size());
			ImGui::Text("Colors: %zu", scene.pointcloud.colors.size());
			ImGui::Text("Normals: %zu", scene.pointcloud.normals.size());
			AABB3f bounds = scene.pointcloud.GetAABB();
			ImGui::Text("Bounds:");
			ImGui::Text("  Min: (%.3f, %.3f, %.3f)", bounds.ptMin.x(), bounds.ptMin.y(), bounds.ptMin.z());
			ImGui::Text("  Max: (%.3f, %.3f, %.3f)", bounds.ptMax.x(), bounds.ptMax.y(), bounds.ptMax.z());
			Point3f size = bounds.GetSize();
			ImGui::Text("  Size: (%.3f, %.3f, %.3f)", size.x, size.y, size.z);
		}

		if (!scene.mesh.IsEmpty()) {
			ImGui::Separator();
			ImGui::Text("Mesh Statistics");
			ImGui::Separator();
			ImGui::Text("Vertices: %u", scene.mesh.vertices.size());
			ImGui::Text("Faces: %u", scene.mesh.faces.size());
			ImGui::Text("Textures: %u", scene.mesh.texturesDiffuse.size());
			// Show mesh bounds if available
			AABB3f meshBounds = scene.mesh.GetAABB();
			ImGui::Text("Mesh Bounds:");
			ImGui::Text("  Min: (%.3f, %.3f, %.3f)", meshBounds.ptMin.x(), meshBounds.ptMin.y(), meshBounds.ptMin.z());
			ImGui::Text("  Max: (%.3f, %.3f, %.3f)", meshBounds.ptMax.x(), meshBounds.ptMax.y(), meshBounds.ptMax.z());
			Point3f meshSize = meshBounds.GetSize();
			ImGui::Text("  Size: (%.3f, %.3f, %.3f)", meshSize.x, meshSize.y, meshSize.z);
		}

		// Estimate SfM normals and mesh patches if valid SfM scene
		ImGui::Separator();
		if (ImGui::Checkbox("Estimate SfM Normals", &window.GetScene().estimateSfMNormals))
			window.RequestRedraw();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Toggle SfM normals estimation; need to reopen the scene");
		if (ImGui::Checkbox("Estimate SfM Patches", &window.GetScene().estimateSfMPatches))
			window.RequestRedraw();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Toggle SfM patches estimation; need to reopen the scene");
	}
	ImGui::End();
}

void UI::ShowCameraControls(Window& window) {
	if (!showCameraControls) return;

	ImGui::SetNextWindowPos(ImVec2(1044, 100), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(224, 360), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Camera Controls", &showCameraControls)) {
		Camera& camera = window.GetCamera();

		// Navigation mode
		const char* navModes[] = { "Arcball", "First Person", "Selection" };
		int currentMode = (int)window.GetControlMode();
		if (ImGui::Combo("Navigation Mode", &currentMode, navModes, IM_ARRAYSIZE(navModes)))
			window.SetControlMode((Window::ControlMode)currentMode);

		// Projection mode
		bool ortho = camera.IsOrthographic();
		if (ImGui::Checkbox("Orthographic", &ortho))
			camera.SetOrthographic(ortho);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Toggle orthographic/perspective projection mode");

		// FOV slider
		float fov = (float)camera.GetFOV();
		if (ImGui::SliderFloat("FOV", &fov, 1.f, 179.f, "%.1f°"))
			camera.SetFOV(fov);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Field of View (FOV) angle");

		// Camera rendering checkbox
		if (ImGui::Checkbox("Show Cameras", &window.showCameras))
			window.RequestRedraw();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Toggle camera frustum display (C key)");
		if (ImGui::SliderFloat("Camera Size", &window.cameraSize, 0.005f, 0.5f, "%.4f")) {
			window.GetRenderer().UploadCameras(window);
			window.GetRenderer().UploadSelection(window);
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Adjust camera size");

		// Camera display color mode
		ImGui::TextUnformatted("Camera Display Color");
		int displayColor = (int)window.cameraDisplayColor;
		bool cameraDisplayChanged = false;
		if (ImGui::RadioButton("Solid##CameraDisplayColor", &displayColor, (int)Window::CAMERA_COLOR_SOLID))
			cameraDisplayChanged = true;
		ImGui::SameLine();
		if (ImGui::RadioButton("Jet##CameraDisplayColor", &displayColor, (int)Window::CAMERA_COLOR_JET))
			cameraDisplayChanged = true;

		// Camera display type
		ImGui::TextUnformatted("Camera Display Type");
		int displayType = (int)window.cameraDisplayType;
		if (ImGui::RadioButton("Frustum##CameraDisplayType", &displayType, (int)Window::CAMERA_DISPLAY_FRUSTUM))
			cameraDisplayChanged = true;
		ImGui::SameLine();
		if (ImGui::RadioButton("Dot##CameraDisplayType", &displayType, (int)Window::CAMERA_DISPLAY_DOT))
			cameraDisplayChanged = true;

		if (ImGui::Checkbox("Show Camera LookAt", &window.showCameraLookAt))
			cameraDisplayChanged = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Show look-at direction indicator for each camera");

		if (cameraDisplayChanged) {
			window.cameraDisplayColor = (Window::CameraDisplayColor)displayColor;
			window.cameraDisplayType = (Window::CameraDisplayType)displayType;
			window.GetRenderer().UploadCameras(window);
			window.GetRenderer().UploadSelection(window);
			window.RequestRedraw();
		}

		// Arcball sensitivity controls (only show when in arcball mode)
		if (window.GetControlMode() == Window::CONTROL_ARCBALL) {
			ImGui::Separator();
			ImGui::Text("Arcball Sensitivity");
			ArcballControls& arcballControls = window.GetArcballControls();
			// General Sensitivity input
			float sensitivity = (float)arcballControls.getSensitivity();
			if (ImGui::InputFloat("Sensitivity", &sensitivity, 0.1f, 5.f, "%.2f"))
				arcballControls.setSensitivity(MAXF(0.001f, sensitivity));
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Overall sensitivity multiplier");
			// Rotation Sensitivity slider
			float rotationSensitivity = (float)arcballControls.getRotationSensitivity();
			if (ImGui::SliderFloat("Rotation", &rotationSensitivity, 0.1f, 5.f, "%.2f"))
				arcballControls.setRotationSensitivity(rotationSensitivity);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Rotation sensitivity");
			// Zoom Sensitivity slider
			float zoomSensitivity = (float)arcballControls.getZoomSensitivity();
			if (ImGui::SliderFloat("Zoom", &zoomSensitivity, 0.1f, 5.f, "%.2f"))
				arcballControls.setZoomSensitivity(zoomSensitivity);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Zoom/scroll sensitivity");
			// Pan Sensitivity slider
			float panSensitivity = (float)arcballControls.getPanSensitivity();
			if (ImGui::SliderFloat("Pan", &panSensitivity, 0.1f, 5.f, "%.2f"))
				arcballControls.setPanSensitivity(panSensitivity);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Pan/translate sensitivity");
		}

		// First person sensitivity controls (only show when in first person mode)
		if (window.GetControlMode() == Window::CONTROL_FIRST_PERSON) {
			ImGui::Separator();
			ImGui::Text("First Person Sensitivity");
			FirstPersonControls& firstPersonControls = window.GetFirstPersonControls();
			// Movement Speed input
			float movementSpeed = (float)firstPersonControls.getMovementSpeed();
			if (ImGui::InputFloat("Speed", &movementSpeed, 0.1f, 1.f, "%.3f"))
				firstPersonControls.setMovementSpeed(MAXF(0.001f, movementSpeed));
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Movement speed multiplier");
			// Rotation Sensitivity slider
			float mouseSensitivity = (float)firstPersonControls.getMouseSensitivity();
			if (ImGui::SliderFloat("Sensitivity", &mouseSensitivity, 0.1f, 5.f, "%.2f"))
				firstPersonControls.setMouseSensitivity(mouseSensitivity);
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Mouse sensitivity");
		}

		// Camera view mode info
		if (camera.IsCameraViewMode()) {
			ImGui::Separator();
			ImGui::Text("Camera View Mode");
			ImGui::Text("Current Camera: %d", (int)camera.GetCurrentCamID());
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Use Left/Right arrows to switch cameras");
			// Show restore button if we have a saved state
			ImGui::SameLine();
			if (ImGui::SmallButton("Exit"))
				camera.DisableCameraViewMode();
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Exit camera view mode and restore previous position");
		} else {
			// Show save current state button when not in camera view mode
			ImGui::Separator();
			ImGui::Text("Camera State:");
			ImGui::SameLine();
			if (ImGui::SmallButton("Save"))
				camera.SaveCurrentState();
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Save current camera position and view direction");
			// Show status if state is saved
			if (camera.HasSavedState()) {
				ImGui::SameLine();
				if (ImGui::SmallButton("Restore"))
					camera.RestoreSavedState();
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("Restore previous camera position and view direction");
			}
		}

		// Camera info
		ImGui::Separator();
		Eigen::Vector3d pos = camera.GetPosition();
		ImGui::Text("Position: %.4g, %.4g, %.4g", pos.x(), pos.y(), pos.z());
		Eigen::Vector3d target = camera.GetTarget();
		ImGui::Text("Target: %.4g, %.4g, %.4g", target.x(), target.y(), target.z());

		// Highlight points visible by the current/selected camera
		ImGui::Separator();
		bool highlightCameraVisiblePoints(window.GetSelectionController().getCurrentCameraIdxForHighlight() != NO_ID);
		if (ImGui::Checkbox("Highlight points seen by camera", &highlightCameraVisiblePoints))
			window.GetScene().OnSelectPointsByCamera(highlightCameraVisiblePoints);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Select and highlight all points observed by the active camera");
		// Keep highlight in sync if camera selection changes while toggle is on
		if (highlightCameraVisiblePoints)
			window.GetScene().OnSelectPointsByCamera(true);

		// Reset button
		ImGui::Separator();
		if (ImGui::Button("Reset Camera"))
			window.ResetView();
	}
	ImGui::End();
}

void UI::ShowSelectionControls(Window& window) {
	// Auto-open selection controls when in selection mode
	if (window.GetControlMode() != Window::CONTROL_SELECTION)
		showSelectionControls = false;
	if (!showSelectionControls)
		return;

	ImGui::SetNextWindowPos(ImVec2(990, 210), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(280, 320), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Selection Controls", &showSelectionControls)) {
		// Only show controls if we have a selection controller
		if (window.GetControlMode() != Window::CONTROL_SELECTION) {
			ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.f), "Selection mode not active");
			ImGui::Text("Switch to Selection mode in Camera Controls");
			ImGui::Text("or press G to enable selection.");
			ImGui::End();
			return;
		}

		SelectionController& selectionController = window.GetSelectionController();

		// Selection tool selection
		ImGui::Text("Selection Tools");
		ImGui::Separator();
		const char* selectionModes[] = { "Box", "Lasso", "Circle" };
		int selMode = (int)selectionController.getSelectionMode();
		if (ImGui::Combo("Tool", &selMode, selectionModes, IM_ARRAYSIZE(selectionModes)))
			selectionController.setSelectionMode((SelectionController::SelectionMode)selMode);

		// Quick tool shortcuts
		ImGui::Text("Shortcuts: B = Box, L = Lasso, C = Circle");

		// Selection statistics
		ImGui::Separator();
		ImGui::Text("Selection Statistics");
		if (selectionController.hasSelection()) {
			ImGui::Text("Selected: %zu points, %zu faces",
				selectionController.getSelectedPointCount(),
				selectionController.getSelectedFaceCount());
		} else {
			ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.f), "No selection");
		}

		// Selection operations
		ImGui::Separator();
		ImGui::Text("Selection Operations");
		if (ImGui::Button("Clear Selection", ImVec2(-1, 0)))
			selectionController.clearSelection();

		if (selectionController.hasSelection()) {
			if (ImGui::Button("Invert Selection", ImVec2(-1, 0)))
				selectionController.invertSelection();

			// Geometry operations
			ImGui::Separator();
			ImGui::Text("Geometry Operations");
			if (ImGui::Button("Remove Selected", ImVec2(-1, 0)))
				ImGui::OpenPopup("Confirm Remove Selected");

			// ROI selection
			bool aabb = selectionController.isROIfromSelectionMode();
			if (ImGui::Checkbox("AABBox", &aabb))
				selectionController.setROIfromSelectionMode(aabb);
			ImGui::SameLine();
			if (ImGui::Button("Set ROI to Selection", ImVec2(-1, 0)))
				selectionController.runROICallback();

			static int minPoints = 150;
			if (selectionController.getSelectedPointCount() >= 3) {
				// Add parameter input for minimum views
				ImGui::InputInt("Min Points", &minPoints, 1, 10);
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("Minimum number of selected points an image must see to be included");

				if (ImGui::Button("Crop Scene to Selection", ImVec2(-1, 0)))
					ImGui::OpenPopup("Crop Scene to Selection");
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("Create a new scene containing only images that see the selected points");
			}

			// Crop Scene to Selection popup
			if (ImGui::BeginPopupModal("Crop Scene to Selection", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
				ImGui::Text("Create a new scene with images that see");
				ImGui::Text("at least %d selected points?", minPoints);
				ImGui::Separator();
				if (ImGui::Button("Crop Scene", ImVec2(120, 0))) {
					// Use the new Scene::CropToPoints function
					MVS::PointCloud::IndexArr selectedPointIndices = selectionController.getSelectedPointIndices();
					// Call the new CropToPoints function
					const Scene& scene = window.GetScene();
					MVS::Scene croppedScene = scene.CropToPoints(selectedPointIndices, static_cast<unsigned>(minPoints));
					// Check if we got a valid cropped scene
					if (!croppedScene.IsEmpty()) {
						// Show save dialog and export the new scene
						window.SetVisible(false);
						String filename;
						if (ShowSaveFileDialog(filename)) {
							// Ensure .mvs extension
							if (Util::getFileExt(filename).empty())
								filename += ".mvs";
							// Save the cropped scene directly
							if (!croppedScene.Save(filename, scene.nArchiveType))
								DEBUG("error: failed to save cropped scene to '%s'", filename.c_str());
						}
						window.SetVisible(true);
						ImGui::CloseCurrentPopup();
					} else {
						ImGui::TextColored(ImVec4(1.f, 0.6f, 0.6f, 1.f),
							"No images see %d or more selected points!", minPoints);
					}
				}
				ImGui::SameLine();
				if (ImGui::Button("Cancel", ImVec2(120, 0)))
					ImGui::CloseCurrentPopup();
				ImGui::EndPopup();
			}

			// Confirmation popups
			if (ImGui::BeginPopupModal("Confirm Remove Selected", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
				ImGui::Text("Remove %zu selected points/faces?",
					selectionController.getSelectedPointCount() + selectionController.getSelectedFaceCount());
				ImGui::TextColored(ImVec4(1.f, 0.6f, 0.6f, 1.f), "This operation cannot be undone!");
				ImGui::Separator();
				if (ImGui::Button("Remove", ImVec2(120, 0))) {
					selectionController.runDeleteCallback();
					ImGui::CloseCurrentPopup();
				}
				ImGui::SameLine();
				if (ImGui::Button("Cancel", ImVec2(120, 0))) {
					ImGui::CloseCurrentPopup();
				}
				ImGui::EndPopup();
			}
		}

		// Controls help
		ImGui::Separator();
		ImGui::Text("Controls");
		ImGui::Text("• G: Exit selection mode");
		ImGui::Text("• B/L/C: Switch selection tools");
		ImGui::Text("• Drag to select geometry");
		ImGui::Text("• Hold Shift: Add to selection");
		ImGui::Text("• Hold Ctrl: Remove from selection");
		ImGui::Text("• I: Invert selection");
		ImGui::Text("• R: Reset selection");
		ImGui::Text("• O: Set ROI from selection");
		ImGui::Text("• Delete: Delete selected elements");
	}
	ImGui::End();
}

void UI::ShowRenderSettings(Window& window) {
	if (!showRenderSettings) return;

	ImGui::SetNextWindowPos(ImVec2(10, 120), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(270, 320), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Render Settings", &showRenderSettings)) {
		ShowRenderingControls(window);
		ShowPointCloudControls(window);
		ShowMeshControls(window);
	}
	ImGui::End();
}

void UI::ShowBoundingBoxControls(Window& window) {
	if (!showBoundingBoxControls) return;

	Scene& scene = window.GetScene();
	MVS::Scene& mvsScene = scene.GetScene();

	ImGui::SetNextWindowPos(ImVec2(20, 130), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(340, 420), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Bounding Box", &showBoundingBoxControls)) {
		const bool hasScene = scene.IsOpen();
		const bool hasPoints = mvsScene.pointcloud.IsValid();
		const bool isBounded = mvsScene.IsBounded();
		const bool workflowRunning = scene.IsWorkflowRunning();

		// --- Visibility ---
		ImGui::TextUnformatted("Visibility");
		ImGui::Separator();
		if (ImGui::Checkbox("Show Bounding Box", &window.showBounds))
			window.RequestRedraw();
		ImGui::SameLine();
		if (ImGui::SmallButton("Toggle (B)")) {
			window.showBounds = !window.showBounds;
			window.RequestRedraw();
		}
		ImGui::Spacing();

		// --- Compute / recompute / clear ---
		ImGui::TextUnformatted("Compute");
		ImGui::Separator();
		{
			ImGui::BeginDisabled(!hasPoints || workflowRunning);
			if (ImGui::Button("Estimate ROI (run workflow)"))
				scene.RunEstimateROIWorkflow(scene.GetEstimateROIWorkflowOptions());
			ImGui::EndDisabled();
			if (!hasPoints && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				ImGui::SetTooltip("Requires a valid point-cloud");
			else if (workflowRunning && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				ImGui::SetTooltip("A workflow is currently running");

			Scene::EstimateROIWorkflowOptions& opts = scene.GetEstimateROIWorkflowOptions();
			ImGui::DragFloat("Scale ROI", &opts.scaleROI, 0.01f, 1.0f, 5.0f, "%.2f");
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Extra margin applied by EstimateROI (1.0 = no margin)");
			const char* upAxisLabels[4] = { "auto", "X", "Y", "Z" };
			int upAxisIdx = opts.upAxis < 0 ? 0 : (opts.upAxis + 1);
			if (ImGui::Combo("Up axis", &upAxisIdx, upAxisLabels, 4))
				opts.upAxis = upAxisIdx == 0 ? -1 : (upAxisIdx - 1);
		}
		ImGui::Spacing();
		{
			if (ImGui::Button("Select ROI with mouse...")) {
				// Switch to selection control so the user can draw a region.
				window.SetControlMode(Window::CONTROL_SELECTION);
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Enters selection mode (press 'G' to toggle). Draw a region,\n"
					"then press 'O' or use the panel buttons bellow to fit\n"
					"the bounding box to the selection.");
			ImGui::BeginDisabled(!hasScene);
			if (ImGui::Button("Set from Selection (OBB)"))
				scene.SetROIFromSelection(false);
			if (ImGui::Button("Set from Selection (AABB)"))
				scene.SetROIFromSelection(true);
			ImGui::EndDisabled();
		}
		ImGui::Spacing();
		{
			ImGui::BeginDisabled(!isBounded);
			if (ImGui::Button("Clear Bounding Box"))
				scene.ClearBoundingBox();
			ImGui::EndDisabled();
			if (!isBounded && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
				ImGui::SetTooltip("No bounding box to clear");
		}
		ImGui::Spacing();

		// --- Manual edit (numeric + 3D gizmo) ---
		ImGui::TextUnformatted("Manual Edit");
		ImGui::Separator();
		if (!isBounded) {
			ImGui::TextDisabled("(no bounding box - compute or set one first)");
		} else {
			// 3D gizmo edit mode toggle
			const bool inEditMode = window.GetControlMode() == Window::CONTROL_BBOX_EDIT;
			if (inEditMode) {
				if (ImGui::Button("Exit Edit Mode"))
					window.SetControlMode(Window::CONTROL_ARCBALL);
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("Return to camera controls. Press Esc in edit mode to cancel a drag.");
			} else {
				if (ImGui::Button("Enter Edit Mode (3D gizmo)"))
					window.SetControlMode(Window::CONTROL_BBOX_EDIT);
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("Show draggable corner, face and rotation handles on the bounding box.\n"
						"Left-drag to edit; Esc cancels the current drag.");
			}
			ImGui::Spacing();

			// Numeric edit - always available. Edit a local copy, then commit
			// through Scene::SetBoundingBox so invariants stay centralized.
			OBB3f edited = mvsScene.obb;
			if (BoxRotationWidget::EditOBB("##OBBEdit", edited)) {
				scene.SetBoundingBox(edited);
				// If the controller is active, refresh its working copy too.
				if (inEditMode)
					window.GetBBoxEditController().setOBB(edited);
			}
			ImGui::Spacing();
			if (ImGui::SmallButton("Re-run Estimate ROI")) {
				if (inEditMode)
					window.SetControlMode(Window::CONTROL_ARCBALL);
				scene.ClearBoundingBox();
				scene.RunEstimateROIWorkflow(scene.GetEstimateROIWorkflowOptions());
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Clear the current OBB and re-run Estimate ROI");
		}
	}
	ImGui::End();
}

void UI::ShowConsoleOverlay(Window& window)
{
	if (!showConsoleOverlay)
		return;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
								   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
								   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_HorizontalScrollbar;

	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImVec2 work_pos = vp->WorkPos;
	ImVec2 work_size = vp->WorkSize;
	ImVec2 window_pos, window_pos_pivot;

	// bottom-right corner
	window_pos.x = work_pos.x + work_size.x - PAD;
	window_pos.y = work_pos.y + work_size.y - PAD;
	window_pos_pivot.x = 1.f;
	window_pos_pivot.y = 1.f;

	ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
	ImGui::SetNextWindowBgAlpha(0.35f);
	ImGui::SetNextWindowSizeConstraints(ImVec2(400, 100), ImVec2(
		MINF(window.GetCamera().GetSize().width*0.8f, 800*window.userFontScale),
		MINF(window.GetCamera().GetSize().height*0.4f, 200*window.userFontScale)));

	if (ImGui::Begin("Console", &showConsoleOverlay, window_flags)) {
		// use the last item's rect (the child) in screen coordinates — this anchors the buttons
		// to the child's outer rectangle so they don't move with the child's scroll.
		ImVec2 child_min = ImGui::GetItemRectMin();
		ImVec2 child_max = ImGui::GetItemRectMax();

		// copy out-of-lock to avoid holding lock during ImGui calls
		std::vector<String> copyLines; {
			std::lock_guard<std::mutex> lock(logMutex);
			copyLines.assign(logBuffer.begin(), logBuffer.end());
		}
		// copy lines to ImGui
		for (const auto &line : copyLines)
			ImGui::TextUnformatted(line.c_str());
		// auto-scroll to bottom if already at bottom
		if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
			ImGui::SetScrollHereY(1.0f);
		ImGui::SetNextItemAllowOverlap();

		// render overlay buttons on top-right of the LogRegion (draw after child so they're on top)
		const auto CalcButtonWidth = []() {
			ImGuiStyle& style = ImGui::GetStyle();
			const char* btnLabels[2] = { "Clear", "Copy" };
			float totalButtonsWidth = 0.f;
			for (int i = 0; i < 2; ++i) {
				ImVec2 txtSize = ImGui::CalcTextSize(btnLabels[i]);
				float btnW = txtSize.x + style.FramePadding.x * 2.f;
				totalButtonsWidth += btnW;
			}
			totalButtonsWidth += style.ItemSpacing.x * 3.f; // spacing between 2 buttons
			ImVec2 btn_width;
			btn_width.x = -totalButtonsWidth;
			btn_width.y = style.ItemSpacing.y * 2.f;
			return btn_width;
		};
		static const ImVec2 btn_width = CalcButtonWidth();

		// move cursor to absolute screen position and render buttons (they will be drawn on top)
		ImVec2 btn_pos;
		btn_pos.x = child_max.x + btn_width.x;
		btn_pos.y = child_min.y + btn_width.y;
		ImGui::SetCursorScreenPos(btn_pos);
		if (ImGui::SmallButton("Clear")) {
			std::lock_guard<std::mutex> lock(logMutex);
			logBuffer.clear();
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("Copy")) {
			std::string all; {
				std::lock_guard<std::mutex> lock(logMutex);
				for (const auto &s : logBuffer)
					all += s.c_str();
			}
			ImGui::SetClipboardText(all.c_str());
		}
	}
	ImGui::End();
}

void UI::ShowPerformanceOverlay(Window& window) {
	if (!showPerformanceOverlay)
		return;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
								   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
								   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

	const ImGuiViewport* viewport = ImGui::GetMainViewport();
	ImVec2 work_pos = viewport->WorkPos;
	ImVec2 work_size = viewport->WorkSize;
	ImVec2 window_pos, window_pos_pivot;

	window_pos.x = work_pos.x + work_size.x - PAD;
	window_pos.y = work_pos.y + PAD;
	window_pos_pivot.x = 1.f;
	window_pos_pivot.y = 0.f;

	ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
	ImGui::SetNextWindowBgAlpha(0.35f);

	if (ImGui::Begin("Performance", &showPerformanceOverlay, window_flags)) {
		if (window.renderOnlyOnChange) {
			ImGui::Text("Frame Time: %.3f ms", deltaTime);
		} else {
			ImGui::Text("FPS: %.1f", fps);
			ImGui::Text("Frame Time: %.3f ms", 1000.f / fps);
		}
		ImGui::Separator();
		if (ImGui::IsMousePosValid())
			ImGui::Text("Mouse: %.f, %.f", ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y);
		else
			ImGui::Text("Mouse: <invalid>");
		if (window.GetControlMode() == Window::CONTROL_ARCBALL) {
			const auto& target = window.GetCamera().GetTarget();
			ImGui::Text("Target: %.4g, %.4g, %.4g", target.x(), target.y(), target.z());
		}
	}
	ImGui::End();
}

void UI::ShowWorkflowOverlay(Window& window) {
	const Scene& scene = window.GetScene();
	const bool workflowRunning = scene.IsWorkflowRunning();
	const auto& history = scene.GetWorkflowHistory();

	// Only show if there's an active workflow or history
	if (!showWorkflowOverlay || (!workflowRunning && history.empty()))
		return;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
								   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
								   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

	const ImGuiViewport* viewport = ImGui::GetMainViewport();
	ImVec2 work_pos = viewport->WorkPos;
	ImVec2 work_size = viewport->WorkSize;
	ImVec2 window_pos, window_pos_pivot;

	// Position below performance overlay on the right
	window_pos.x = work_pos.x + work_size.x - PAD;
	window_pos.y = work_pos.y + PAD + 100.f; // Below performance overlay
	window_pos_pivot.x = 1.f;
	window_pos_pivot.y = 0.f;

	ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
	ImGui::SetNextWindowBgAlpha(0.35f);

	if (ImGui::Begin("Workflow Status", &showWorkflowOverlay, window_flags)) {
		// Current workflow
		if (workflowRunning) {
			const Scene::WorkflowType type = scene.GetCurrentWorkflowType();
			const double elapsed = scene.GetWorkflowElapsedTime();
			const char* workflowName =
				type == Scene::WF_ESTIMATE_ROI ? "Estimate ROI" :
				type == Scene::WF_DENSIFY ? "Densify" :
				type == Scene::WF_RECONSTRUCT ? "Reconstruct Mesh" :
				type == Scene::WF_REFINE ? "Refine Mesh" :
				type == Scene::WF_TEXTURE ? "Texture Mesh" : "Unknown";

			ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "Running: %s", workflowName);
			ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1, 0));
			ImGui::Text("Elapsed: %.1f s", elapsed);
			ImGui::Separator();
		}

		// Workflow history stats
		if (!history.empty()) {
			ImGui::Text("Completed: %zu", history.size());

			// Show last few workflows
			const size_t maxShow = 5;
			const size_t start = history.size() > maxShow ? history.size() - maxShow : 0;
			for (size_t i = start; i < history.size(); ++i) {
				const auto& entry = history[i];
				const char* name =
					entry.type == Scene::WF_ESTIMATE_ROI ? "ROI" :
					entry.type == Scene::WF_DENSIFY ? "Densify" :
					entry.type == Scene::WF_RECONSTRUCT ? "Reconstruct" :
					entry.type == Scene::WF_REFINE ? "Refine" :
					entry.type == Scene::WF_TEXTURE ? "Texture" : "?";

				if (entry.success) {
					ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "%s: %.1f s", name, entry.duration);
				} else {
					ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "%s: FAILED", name);
				}
			}

			if (ImGui::SmallButton("Clear History")) {
				const_cast<Scene&>(scene).ClearWorkflowHistory();
			}
		}
	}
	ImGui::End();
}

void UI::ShowViewportOverlay(const Window& window) {
	if (!showViewportOverlay) return;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
								   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
								   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImVec2 work_pos = vp->WorkPos;
	ImVec2 window_pos;

	window_pos.x = work_pos.x + PAD;
	window_pos.y = work_pos.y + PAD;

	ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always);
	ImGui::SetNextWindowBgAlpha(0.35f);

	if (ImGui::Begin("Viewport Info", &showViewportOverlay, window_flags)) {
		const Camera& camera = window.GetCamera();
		ImGui::Text("Viewport: %dx%d", camera.GetSize().width, camera.GetSize().height);
		ImGui::Text("FOV: %.1f°", camera.GetFOV());
		ImGui::Text("Mode: %s", camera.IsOrthographic() ? "Orthographic" : "Perspective");
		// Navigation mode display
		const char* modeText(window.GetControlMode() == Window::CONTROL_ARCBALL ? "Arcball" : window.GetControlMode() == Window::CONTROL_FIRST_PERSON ? "First Person" : "Selection");
		ImGui::Text("Navigation: %s", modeText);
	}
	ImGui::End();
}

// Show a centered, headerless hint when there is no valid scene loaded
void UI::ShowEmptySceneOverlay(const Window& window) {
	Scene& scene = window.GetScene();
	if (scene.IsWorkflowRunning() || scene.IsOpen())
		return;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration |
								   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
								   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

	// Layout parameters
	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImVec2 center(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f);
	const float btn_w = 120.f, btn_h = 30.f;
	const float pad_x = 24.f, pad_y = 12.f;
	const float spacing_after_icon = 8.f;
	const float font_mult = 2.2f;
	const float font_size = ImGui::GetFontSize() * font_mult;
	const char* msg1 = "drag & drop";
	const char* msg2 = "a 3D scene";

	// Compute icon size based on viewport, preserving aspect ratio, clamp to 512
	const float vp_min = MINF(vp->WorkSize.x, vp->WorkSize.y);
	const float max_dim = MINF(512.f, vp_min * 0.25f); // at most 25% of smaller viewport dim
	const float icon_w = max_dim, icon_h = max_dim;

	// Compute text size using an explicit font size (so other UI isn't affected)
	ImVec2 text_size1 = ImGui::GetFont()->CalcTextSizeA(font_size, FLT_MAX, 0.f, msg1);
	ImVec2 text_size2 = ImGui::CalcTextSize(msg2);

	float content_w = MAXF3(icon_w, text_size1.x, btn_w);
	float win_w = content_w + pad_x * 2;
	float win_h = pad_y + icon_h + spacing_after_icon * 2 + text_size1.y + text_size2.y + 24.f + btn_h + pad_y;

	ImGui::SetNextWindowSize(ImVec2(win_w, win_h), ImGuiCond_Always);
	ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
	ImGui::SetNextWindowBgAlpha(0.1f);

	if (ImGui::Begin("EmptySceneHint", nullptr, window_flags)) {
		ImVec2 win_pos = ImGui::GetWindowPos();
		ImVec2 win_size = ImGui::GetWindowSize();

		// Lazy-load embedded PNG into GL texture via OpenCV
		if (!emptySceneIcon.IsValid()) {
			cv::Mat raw(1, (int)empty_scene_icon_png_len, CV_8UC1, (void*)empty_scene_icon_png);
			cv::Mat iconPreview = cv::imdecode(raw, cv::IMREAD_UNCHANGED);
			emptySceneIcon.Create(iconPreview, /*genMipmaps=*/true, /*srgb=*/false);
		}

		// Draw icon (texture)
		ASSERT(emptySceneIcon.IsValid());
		ImVec2 icon_pos(win_pos.x + (win_size.x - icon_w) * 0.5f, win_pos.y + pad_y);
		ImGui::SetCursorScreenPos(icon_pos);
		ImGui::Image((ImTextureID)(uintptr_t)emptySceneIcon.GetID(), ImVec2(icon_w, icon_h));

		// Message text below icon1
		ImVec2 text_pos1(win_pos.x + (win_size.x - text_size1.x) * 0.5f, icon_pos.y + icon_h + spacing_after_icon);
		ImGui::SetCursorScreenPos(text_pos1);
		ImGui::SetWindowFontScale(font_mult);
		ImGui::TextUnformatted(msg1);
		ImVec2 text_pos2(win_pos.x + (win_size.x - text_size2.x) * 0.5f, icon_pos.y + icon_h + spacing_after_icon*2 + text_size1.y);
		ImGui::SetCursorScreenPos(text_pos2);
		ImGui::SetWindowFontScale(1.f);
		ImGui::TextUnformatted(msg2);

		// Open button
		ImGui::Dummy(ImVec2(0,8));
		ImGui::SetCursorPosX((win_size.x - btn_w) * 0.5f);
		if (ImGui::Button("Open", ImVec2(btn_w, btn_h))) {
			String filename, geometryFilename;
			if (ShowOpenFileDialog(filename, geometryFilename))
				scene.Open(filename, geometryFilename);
		}
	}
	ImGui::End();
}

void UI::ShowAboutDialog() {
	if (!showAboutDialog)
		return;

	ImGui::OpenPopup("About");
	if (ImGui::BeginPopupModal("About", &showAboutDialog, ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::Text("OpenMVS Viewer " OpenMVS_VERSION);
		ImGui::Text("Author: SEACAVE");
		ImGui::Text("Website: https://cdcseacave.github.io");
		ImGui::Separator();
		ImGui::Text("Built with ImGui %s and", ImGui::GetVersion());
		ImGui::Text("OpenGL %s", glGetString(GL_VERSION));
		ImGui::Separator();
		if (ImGui::Button("Close")) {
			showAboutDialog = false;
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

void UI::ShowHelpDialog() {
	if (!showHelpDialog)
		return;

	ImGui::OpenPopup("Help");
	if (ImGui::BeginPopupModal("Help", &showHelpDialog, ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::Text("OpenMVS Viewer - Help & Controls");
		ImGui::Separator();

		// Detect macOS for platform-specific shortcuts
		#ifdef __APPLE__
		const bool isMacOS = true;
		#else
		const bool isMacOS = false;
		#endif

		// File Operations
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "File Operations:");
		if (isMacOS) {
			ImGui::Text("  Cmd+O         Open Scene");
			ImGui::Text("  Cmd+S         Save Scene");
			ImGui::Text("  Cmd+Shift+S   Save Scene As");
			ImGui::Text("  Cmd+X         Save Screenshot");
			ImGui::Text("  Cmd+Q         Exit");
		} else {
			ImGui::Text("  Ctrl+O        Open Scene");
			ImGui::Text("  Ctrl+S        Save Scene");
			ImGui::Text("  Ctrl+Shift+S  Save Scene As");
			ImGui::Text("  Ctrl+X        Save Screenshot");
			ImGui::Text("  Alt+F4        Exit");
		}
		ImGui::Separator();

		// Camera Navigation
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Camera Navigation:");
		ImGui::Text("  Tab           Switch navigation mode (Arcball/First Person)");
		ImGui::Text("  R             Reset camera");
		ImGui::Text("  F1            Show this help");
		ImGui::Text("  F11           Toggle fullscreen");
		ImGui::Separator();

		// Display Controls
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Display Controls:");
		ImGui::Text("  P             Toggle point cloud display");
		ImGui::Text("  M             Toggle mesh display");
		ImGui::Text("  C             Toggle camera frustum display");
		ImGui::Text("  W             Toggle wireframe mesh rendering");
		ImGui::Text("  T             Toggle textured mesh rendering");
		ImGui::Text("  B             Toggle bounding-box display");
		ImGui::Separator();

		// Bounding Box Controls
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Bounding Box:");
		if (isMacOS) {
			ImGui::Text("  Shift+B       Toggle Bounding Box panel");
			ImGui::Text("  Cmd+B         Recompute via Estimate ROI");
		} else {
			ImGui::Text("  Shift+B       Toggle Bounding Box panel");
			ImGui::Text("  Ctrl+B        Recompute via Estimate ROI");
		}
		ImGui::Text("  (From panel)  Enter 3D edit mode for draggable corner/face/rotation handles");
		ImGui::Text("  Escape        Cancel active handle drag while editing");
		ImGui::Separator();

		// Arcball Controls
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Arcball Mode:");
		if (isMacOS) {
			ImGui::Text("  Left click + drag   Rotate camera around target");
			ImGui::Text("  Right click + drag  Pan camera");
			ImGui::Text("  Two-finger drag     Pan camera (trackpad)");
			ImGui::Text("  Scroll/pinch        Zoom in/out");
			ImGui::Text("  Double-click        Focus on clicked point");
		} else {
			ImGui::Text("  Left click + drag   Rotate camera around target");
			ImGui::Text("  Right click + drag  Pan camera");
			ImGui::Text("  Middle click + drag Pan camera");
			ImGui::Text("  Scroll wheel        Zoom in/out");
			ImGui::Text("  Double-click        Focus on clicked point");
		}
		ImGui::Separator();

		// First Person Controls
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "First Person Mode:");
		ImGui::Text("  Mouse movement      Look around");
		ImGui::Text("  W, A, S, D          Move forward/left/backward/right");
		ImGui::Text("  Q, E                Move down/up");
		ImGui::Text("  Scroll wheel        Adjust movement speed");
		if (isMacOS) {
			ImGui::Text("  Shift (hold)        Move faster");
			ImGui::Text("  Cmd (hold)          Move slower");
		} else {
			ImGui::Text("  Shift (hold)        Move faster");
			ImGui::Text("  Ctrl (hold)         Move slower");
		}
		ImGui::Separator();

		// Camera View Mode
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Camera View Mode:");
		ImGui::Text("  Left/Right arrows   Switch between cameras");
		ImGui::Text("  Escape              Exit camera view mode");
		ImGui::Text("  Any camera movement Exit camera view mode");
		ImGui::Separator();

		// Selection & Interaction
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Selection & Interaction:");
		ImGui::Text("  Single click        Select point/face/camera");
		ImGui::Text("  Double-click        Focus on selection");
		ImGui::Text("                      (or enter camera view for cameras)");
		ImGui::Text("  Selection Dialog    Select point/face/camera by index");
		ImGui::Separator();

		// Selection Tools
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Selection Tools:");
		ImGui::Text("  G                   Toggle selection mode");
		ImGui::Text("  B                   Box selection mode");
		ImGui::Text("  L                   Lasso selection mode");
		ImGui::Text("  C                   Circle selection mode");
		ImGui::Text("  Left click + drag   Create selection area");
		if (isMacOS) {
			ImGui::Text("  Shift + drag        Add to selection");
			ImGui::Text("  Cmd + drag          Subtract from selection");
		} else {
			ImGui::Text("  Shift + drag        Add to selection");
			ImGui::Text("  Ctrl + drag         Subtract from selection");
		}
		ImGui::Text("  I                   Invert selection");
		ImGui::Text("  O                   Set ROI from selection");
		ImGui::Text("  Delete              Delete selected elements");
		ImGui::Text("  Escape              Clear selection");
		ImGui::Separator();

		// UI Controls
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "UI Controls:");
		ImGui::Text("  Mouse at top        Show/hide menu bar");
		ImGui::Text("  Escape              Close dialogs/windows");
		ImGui::Text("                      Clear focus/hide menu");
		ImGui::Separator();

		// File Formats
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Supported Formats:");
		ImGui::Text("  Scene files:        .mvs, .sfm, .dmap");
		ImGui::Text("  Geometry files:     .ply, .obj, .gltf, .glb");
		ImGui::Text("  Export formats:     .ply, .obj, .gltf, .glb");
		ImGui::Separator();

		// Tips
		ImGui::TextColored(ImVec4(1.f, 0.9f, 0.6f, 1.f), "Tips:");
		ImGui::Text("  • Use the View menu to toggle overlays and panels");
		ImGui::Text("  • Selection info appears in bottom-left corner");
		ImGui::Text("  • Viewport info appears in top-left corner");
		ImGui::Text("  • Performance stats appear in top-right corner");
		ImGui::Text("  • Double-click selections to focus/navigate to them");
		ImGui::Text("  • Selection tools work on both point clouds and meshes");
		ImGui::Text("  • Use modifier keys to combine multiple selections");
		if (isMacOS) {
			ImGui::Text("  • Use trackpad gestures for smooth navigation");
			ImGui::Text("  • Three-finger drag works as middle-click");
		}

		ImGui::Separator();
		if (ImGui::Button("Close", ImVec2(120, 0))) {
			showHelpDialog = false;
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

void UI::ShowExportDialog(Scene& scene) {
	if (!showExportDialog) return;

	// Show as a regular window instead of modal popup
	ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Export Scene", &showExportDialog, ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::Text("Export scene geometry to various formats");
		ImGui::Separator();

		static int exportFormat = 0;
		static bool bExportViews = true;
		const char* formatOptions[] = { "PLY Point Cloud", "GLTF Point Cloud", "PLY Mesh", "OBJ Mesh", "GLTF Mesh" };
		const char* formatExt[] = { ".ply", ".glb", ".ply", ".obj", ".glb" };
		ImGui::Combo("Export Format", &exportFormat, formatOptions, IM_ARRAYSIZE(formatOptions));

		ImGui::Separator();

		// Show what will be exported based on format and scene content
		const MVS::Scene& mvs_scene = scene.GetScene();
		bool hasPointCloud = !mvs_scene.pointcloud.IsEmpty();
		bool hasMesh = !mvs_scene.mesh.IsEmpty();

		switch (exportFormat) {
		case 0: // PLY Point Cloud
		case 1: // GLTF Point Cloud
			if (hasPointCloud) {
				ImGui::Text("✓ Point cloud: %zu points", mvs_scene.pointcloud.points.size());
				if (!mvs_scene.pointcloud.pointViews.empty()) {
					ImGui::Text("✓ Point views available");
					ImGui::SameLine();
					ImGui::Checkbox("Export", &bExportViews);
				}
				if (!mvs_scene.pointcloud.pointWeights.empty())
					ImGui::Text("✓ Point weights available");
				if (!mvs_scene.pointcloud.colors.empty())
					ImGui::Text("✓ Point colors available");
				if (!mvs_scene.pointcloud.normals.empty())
					ImGui::Text("✓ Point normals available");
			} else {
				ImGui::TextColored(ImVec4(1.f, 0.6f, 0.6f, 1.f), "⚠ No point cloud data to export");
			}
			break;
		case 2: // PLY Mesh
		case 3: // OBJ Mesh
		case 4: // GLTF Mesh
			if (hasMesh) {
				ImGui::Text("✓ Mesh: %u vertices, %u faces", mvs_scene.mesh.vertices.size(), mvs_scene.mesh.faces.size());
				if (!mvs_scene.mesh.faceTexcoords.empty() && !mvs_scene.mesh.texturesDiffuse.empty())
					ImGui::Text("✓ Texture coordinates and textures available");
				if (!mvs_scene.mesh.vertexNormals.empty())
					ImGui::Text("✓ Vertex normals available");
			} else {
				ImGui::TextColored(ImVec4(1.f, 0.6f, 0.6f, 1.f), "⚠ No mesh data to export");
			}
			break;
		}

		ImGui::Separator();

		bool canExport = ((exportFormat == 0 || exportFormat == 1) && hasPointCloud) ||
			((exportFormat == 2 || exportFormat == 3 || exportFormat == 4) && hasMesh);
		if (ImGui::Button("Export...", ImVec2(120, 0)) && canExport) {
			String filename;
			if (ShowSaveFileDialog(filename)) {
				// Ensure the filename has the correct extension
				String baseFileName = Util::getFileFullName(filename);
				String finalFileName = baseFileName + formatExt[exportFormat];
				scene.Export(finalFileName, formatExt[exportFormat], bExportViews);
			}
			showExportDialog = false;
		}
		if (!canExport) {
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.f), "(Export disabled - no compatible data)");
		}

		ImGui::SameLine();
		if (ImGui::Button("Cancel", ImVec2(120, 0)))
			showExportDialog = false;
	}
	ImGui::End();
}

void UI::ShowCameraInfoDialog(Window& window) {
	if (!showCameraInfoDialog) return;

	// Show as a regular window instead of modal popup
	ImGui::SetNextWindowPos(ImVec2(880, 100), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(390, 612), ImGuiCond_FirstUseEver);
	if (ImGui::Begin("Camera Information", &showCameraInfoDialog)) {
		const Scene& scene = window.GetScene();
		const ImageArr& images = scene.GetImages();
		const MVS::Scene& mvs_scene = scene.GetScene();

		const bool hasCameraSelection = window.selectionType == Window::SEL_CAMERA && window.HasSelectionIds();
		const IDX selectedCameraIdx = window.GetSelectionId();
		// Check if we have a selected camera
		if (hasCameraSelection && selectedCameraIdx < images.size()) {
			const Image& image = images[selectedCameraIdx];
			ASSERT(image.idx < mvs_scene.images.size());
			const MVS::Image& imageData = mvs_scene.images[image.idx];
			const MVS::Camera& camera = imageData.camera;
			Point3 eulerAngles;
			camera.R.GetRotationAnglesZYX(eulerAngles.x, eulerAngles.y, eulerAngles.z);

			// Basic image information
			ImGui::Text("Index: %u (ID: %u)", image.idx, imageData.ID);
			ImGui::Text("Name: %s", Util::getFileNameExt(imageData.name).c_str());
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Full Path: %s", imageData.name.c_str());
			if (!imageData.maskName.empty()) {
				ImGui::Text("Mask: %s", Util::getFileNameExt(imageData.maskName).c_str());
				ImGui::Text("Mask Path: %s", imageData.maskName.c_str());
			} else {
				ImGui::Text("Mask: None");
			}

			ImGui::Separator();

			// Image dimensions and properties
			ImGui::Text("Image Properties");
			ImGui::Text("  Size: %ux%u pixels", imageData.width, imageData.height);
			ImGui::Text("  Scale: %.3f", imageData.scale);
			ImGui::Text("  Average Depth: %.3g", imageData.avgDepth);

			// Additional image statistics if available
			if (ImGui::CollapsingHeader("Image Additional Information")) {
				// Check if image is loaded
				if (!imageData.image.empty()) {
					ImGui::Text("  Image Status: Loaded (%dx%dx%d)",
						imageData.image.cols, imageData.image.rows, imageData.image.channels());
				} else {
					ImGui::Text("  Image Status: Not loaded");
				}
				// Show if image is calibrated
				ASSERT(imageData.platformID != NO_ID);
				ImGui::Text("  Platform ID: %u", imageData.platformID);
				ImGui::Text("  Camera ID: %u (from %u)", imageData.cameraID, mvs_scene.platforms[imageData.cameraID].cameras.size());
				ImGui::Text("  Pose ID: %u", imageData.poseID);
			}

			ImGui::Separator();

			// Camera intrinsics
			ImGui::Text("Camera Intrinsics");
			ImGui::Text("  Focal Length: fx=%.2f, fy=%.2f", camera.K(0, 0), camera.K(1, 1));
			ImGui::Text("  Principal Point: cx=%.2f, cy=%.2f", camera.K(0, 2), camera.K(1, 2));

			// Show full intrinsic matrix
			if (ImGui::CollapsingHeader("Camera Additional Information")) {
				ImGui::Text("  FOV: x=%.2f, y=%.2f", R2D(imageData.ComputeFOV(0)), R2D(imageData.ComputeFOV(1)));
				ImGui::Text("  Intrinsic Matrix K:");
				ImGui::Text("    [%.2f  %.2f  %.2f]", camera.K(0, 0), camera.K(0, 1), camera.K(0, 2));
				ImGui::Text("    [%.2f  %.2f  %.2f]", camera.K(1, 0), camera.K(1, 1), camera.K(1, 2));
				ImGui::Text("    [%.2f  %.2f  %.2f]", camera.K(2, 0), camera.K(2, 1), camera.K(2, 2));
			}

			ImGui::Separator();

			// Camera extrinsics
			ImGui::Text("Camera Extrinsics");
			ImGui::Text("  Position: (%.6f, %.6f, %.6f)", camera.C.x, camera.C.y, camera.C.z);
			ImGui::Text("  Rotation (Euler XYZ): %.3f°, %.3f°, %.3f°",
				R2D(eulerAngles.x), R2D(eulerAngles.y), R2D(eulerAngles.z));

			// Show full rotation matrix
			if (ImGui::CollapsingHeader("Rotation Matrix R")) {
				ImGui::Text("  [%.6f  %.6f  %.6f]", camera.R(0, 0), camera.R(0, 1), camera.R(0, 2));
				ImGui::Text("  [%.6f  %.6f  %.6f]", camera.R(1, 0), camera.R(1, 1), camera.R(1, 2));
				ImGui::Text("  [%.6f  %.6f  %.6f]", camera.R(2, 0), camera.R(2, 1), camera.R(2, 2));
			}

			ImGui::Separator();

			// Neighbors information
			ImGui::Text("Neighbor Mode:");
			bool neighborModeChanged = false;
			if (ImGui::RadioButton("Densification", !useTrackBasedNeighbors)) {
				useTrackBasedNeighbors = false;
				neighborModeChanged = true;
			}
			ImGui::SameLine();
			if (ImGui::RadioButton("Track-based", useTrackBasedNeighbors)) {
				useTrackBasedNeighbors = true;
				neighborModeChanged = true;
			}
			if (neighborModeChanged && window.selectedNeighborCamera != NO_ID) {
				window.selectedNeighborCamera = NO_ID;
				window.RequestRedraw();
			}

			// Get neighbors (size only - we'll access elements individually due to different types)
			const size_t neighborCount = useTrackBasedNeighbors
				? scene.GetTrackBasedNeighbors()[selectedCameraIdx].size()
				: imageData.neighbors.size();
			ImGui::Text("Neighbor Images: %zu", neighborCount);
			ImGui::Text("Selected Neighbor Index: %s", window.selectedNeighborCamera == NO_ID ? "NA" : std::to_string(window.selectedNeighborCamera).c_str());
			ImGui::Text("Selected Neighbor Angle: %s",
				window.selectedNeighborCamera == NO_ID ? "NA" : String::FormatString("%.2f", R2D(ACOS(ComputeAngle(
					mvs_scene.images[images[selectedCameraIdx].idx].camera.Direction().ptr(),
					mvs_scene.images[images[window.selectedNeighborCamera].idx].camera.Direction().ptr())))).c_str());
			if (window.selectedNeighborCamera != NO_ID && window.selectionType == Window::SEL_CAMERA) {
				// Compute and display relative pose if a neighbor camera is selected
				const Image& mainView = images[selectedCameraIdx];
				const Image& neighView = images[window.selectedNeighborCamera];
				const MVS::Camera& camMain = mvs_scene.images[mainView.idx].camera;
				const MVS::Camera& camNeigh = mvs_scene.images[neighView.idx].camera;
				RMatrix poseR;
				CMatrix poseC;
				ComputeRelativePose(camMain.R, camMain.C, camNeigh.R, camNeigh.C, poseR, poseC);
				Point3 eulerAngles;
				poseR.GetRotationAnglesZYX(eulerAngles.x, eulerAngles.y, eulerAngles.z);
				ImGui::Separator();
				ImGui::Text("Relative Pose (Neighbor wrt Main)");
				ImGui::Text("  Position: %.3g, %.3g, %.3g (%.3g distance)",
					poseC.x, poseC.y, poseC.z, norm(poseC));
				ImGui::Text("  Rotation (ZYX): %.1f°, %.1f°, %.1f°",
					R2D(eulerAngles.x), R2D(eulerAngles.y), R2D(eulerAngles.z));
			}
			if (neighborCount > 0) {
				// Table headers
				constexpr int columnCount = 6;
				if (ImGui::BeginTable("NeighborsTable", columnCount, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_HighlightHoveredColumn)) {
					ImGui::TableSetupColumn("Index/ID", ImGuiTableColumnFlags_WidthFixed, 45.f);
					if (useTrackBasedNeighbors)
						ImGui::TableSetupColumn("Scale", ImGuiTableColumnFlags_WidthFixed, 50.f);
					else
						ImGui::TableSetupColumn("Score", ImGuiTableColumnFlags_WidthFixed, 50.f);
					ImGui::TableSetupColumn("Angle", ImGuiTableColumnFlags_WidthFixed, 33.f);
					ImGui::TableSetupColumn("Area", ImGuiTableColumnFlags_WidthFixed, 24.f);
					ImGui::TableSetupColumn(useTrackBasedNeighbors ? "Tracks" : "Points", ImGuiTableColumnFlags_WidthFixed, 39.f);
					ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
					ImGui::TableHeadersRow();
					// Show neighbors in table format
					for (size_t i = 0; i < neighborCount; ++i) {
						// Get neighbor ViewScore (handle different types)
						const MVS::ViewScore& neighbor = useTrackBasedNeighbors
							? scene.GetTrackBasedNeighbors()[selectedCameraIdx][i].score
							: imageData.neighbors[i];
						const MVS::Image& neighborImage = mvs_scene.images[neighbor.ID];
						ImGui::TableNextRow();
						ImGui::TableSetColumnIndex(0);
						// Highlight and make the entire row clickable by using Selectable
						const bool isSelected(window.selectedNeighborCamera == neighbor.ID);
						String rowLabel = String::FormatString("%u/%u##neighbor_%u", neighbor.ID, neighborImage.ID, neighbor.ID);
						bool rowClicked = ImGui::Selectable(rowLabel.c_str(), isSelected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap);
						// Handle row click
						if (rowClicked) {
							// Deselect if already selected, or select it otherwise
							const bool wasSelected = (window.selectedNeighborCamera == neighbor.ID);
							window.selectedNeighborCamera = wasSelected ? NO_ID : scene.ImageIdxMVS2Viewer(neighbor.ID);
							// Highlight shared points if using track-based neighbors and a neighbor is selected
							if (useTrackBasedNeighbors && !wasSelected && !mvs_scene.pointcloud.IsEmpty()) {
								// Get the shared points for this neighbor
								const auto& sharedPoints = scene.GetTrackBasedNeighbors()[selectedCameraIdx][i].sharedPoints;
								window.GetSelectionController().setSelectedPoints(sharedPoints, mvs_scene.pointcloud.points.size());
							} else if (wasSelected) {
								// Clear selection when deselecting
								window.GetSelectionController().clearSelection();
							}
							window.GetRenderer().UploadSelection(window);
							window.RequestRedraw();
						}
						// Handle double-click to focus on the neighbor camera
						if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
							// Select and focus on the neighbor camera
							window.selectionType = Window::SEL_CAMERA;
							const MVS::IIndex selectionIdx = scene.ImageIdxMVS2Viewer(neighbor.ID);
							window.SetSelectionId(selectionIdx);
							window.selectedNeighborCamera = NO_ID;
							// Clear point selection when switching camera
							if (useTrackBasedNeighbors)
								window.GetSelectionController().clearSelection();
							window.GetCamera().SetCameraViewMode(selectionIdx);
							window.GetRenderer().UploadSelection(window);
							ImGui::SetWindowFocus(nullptr); // Defocus dialog window
							window.RequestRedraw();
						}
						// Fill in the other columns with data
						ImGui::TableSetColumnIndex(1);
						if (useTrackBasedNeighbors) {
							ImGui::Text("%.2f", neighbor.scale);
							ImGui::TableSetColumnIndex(2);
						} else {
							ImGui::Text("%.2f", neighbor.score);
							ImGui::TableSetColumnIndex(2);
						}
						ImGui::Text("%.2f", FR2D(neighbor.angle));
						ImGui::TableSetColumnIndex(3);
						ImGui::Text("%d", ROUND2INT(neighbor.area*100));
						ImGui::TableSetColumnIndex(4);
						ImGui::Text("%u", neighbor.points);
						ImGui::TableSetColumnIndex(5);
						ImGui::Text("%s", Util::getFileNameExt(neighborImage.name).c_str());
					}
					ImGui::EndTable();
				}
			}
		} else {
			// No camera selected - clear neighbor selection and point highlights
			if (window.selectedNeighborCamera != NO_ID) {
				window.selectedNeighborCamera = NO_ID;
				window.GetSelectionController().clearSelection();
				window.GetRenderer().UploadSelection(window);
				window.RequestRedraw();
			}
			ImGui::Text("No camera/image selected");
			ImGui::Separator();
			ImGui::Text("Select a camera by clicking on it in the 3D view");
			ImGui::Text("or double-clicking to enter camera view mode.");
			ImGui::Spacing();
			ImGui::Text("Select a camera in 3D while pressing Ctrl in order");
			ImGui::Text("to select a neighbor camera, or select it in the");
			ImGui::Text("neighbors list.");
			ImGui::Separator();
			ImGui::Text("Total cameras in scene: %u", (unsigned)mvs_scene.images.size());
		}
	}
	ImGui::End();
}

void UI::ShowSelectionDialog(Window& window) {
	if (!showSelectionDialog) return;

	// Input buffers for the dialog
	static char selectionInputBuffer[4096] = "";
	static int selectionType = 0; // 0 Point, 1 Face, 2 Camera by Index, 3 Camera by Name

	// Set dialog properties
	ImGui::OpenPopup("Selection Dialog");
	if (ImGui::BeginPopupModal("Selection Dialog", &showSelectionDialog, ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::Text("Select an element by index or name:");
		ImGui::Separator();

		// Selection type radio buttons
		ImGui::RadioButton("Point by Index", &selectionType, 0);
		ImGui::SameLine();
		ImGui::RadioButton("Face by Index", &selectionType, 1);
		ImGui::RadioButton("Camera by Index", &selectionType, 2);
		ImGui::SameLine();
		ImGui::RadioButton("Camera by Name", &selectionType, 3);

		ImGui::Separator();

		// Input fields based on selection type
		const Scene& scene = window.GetScene();
		const MVS::Scene& mvs_scene = scene.GetScene();
		IDXArr selectionIndices;
		String selectionError;

		ImGui::InputText("##selectionInput", selectionInputBuffer, sizeof(selectionInputBuffer), ImGuiInputTextFlags_None);
		ImGui::SameLine();
		if (ImGui::Button("Paste")) {
			const char* clipboard = ImGui::GetClipboardText();
			if (clipboard && *clipboard) {
				std::strncpy(selectionInputBuffer, clipboard, sizeof(selectionInputBuffer) - 1);
				selectionInputBuffer[sizeof(selectionInputBuffer) - 1] = '\0';
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Clear"))
			selectionInputBuffer[0] = '\0';
		if (selectionType < 3)
			ImGui::TextDisabled("Use commas/spaces to separate IDs and '-' for ranges (e.g., 1 2 10-15).");

		if (strlen(selectionInputBuffer) > 0) {
			switch (selectionType) {
			case 0: { // Point by Index
				selectionError = Util::parseIndexRanges(selectionInputBuffer, mvs_scene.pointcloud.points.size(), selectionIndices, "point");
				} break;
			case 1: { // Face by Index
				selectionError = Util::parseIndexRanges(selectionInputBuffer, mvs_scene.mesh.faces.size(), selectionIndices, "face");
				} break;
			case 2: { // Camera by Index
				selectionError = Util::parseIndexRanges(selectionInputBuffer, mvs_scene.images.size(), selectionIndices, "camera");
				} break;
			case 3: { // Camera by Name
				int cameraIndex = -1;
				const ImageArr& images = scene.GetImages();
				FOREACH(i, images) {
					if (images[i].idx < mvs_scene.images.size()) {
						const MVS::Image& imageData = mvs_scene.images[images[i].idx];
						String fileName = Util::getFileNameExt(imageData.name);
						if (fileName.find(selectionInputBuffer) != String::npos) {
							cameraIndex = i;
							break;
						}
					}
				}
				if (cameraIndex != -1) {
					selectionIndices.assign(1, static_cast<IDX>(cameraIndex));
				} else {
					selectionError = "Camera name not found!";
				}
				} break;
			}
		}
		if (!selectionError.empty())
			ImGui::TextColored(ImVec4(1, 0, 0, 1), "%s", selectionError.c_str());

		ImGui::Separator();

		// Buttons
		if (ImGui::Button("Select", ImVec2(120, 0)) && selectionError.empty()) {
			// Perform the selection based on the type
			const IDX primarySelectionIdx = selectionIndices.front();
			switch (selectionType) {
			case 0: { // Point by Index
				window.selectionType = Window::SEL_POINT;
				window.SetSelectionIds(selectionIndices);
				window.selectionPoints[0] = mvs_scene.pointcloud.points[primarySelectionIdx];
			} break;

			case 1: { // Face by Index
				window.selectionType = Window::SEL_TRIANGLE;
				window.SetSelectionIds(selectionIndices);
				const MVS::Mesh::Face& face = mvs_scene.mesh.faces[primarySelectionIdx];
				window.selectionPoints[0] = mvs_scene.mesh.vertices[face[0]];
				window.selectionPoints[1] = mvs_scene.mesh.vertices[face[1]];
				window.selectionPoints[2] = mvs_scene.mesh.vertices[face[2]];
			} break;

			case 2:   // Camera by Index
			case 3: { // Camera by Name
				window.selectionType = Window::SEL_CAMERA;
				window.SetSelectionIds(selectionIndices);
				const MVS::Image& imageData = mvs_scene.images[scene.GetImages()[primarySelectionIdx].idx];
				window.selectionPoints[0] = imageData.camera.C;
			} break;
			}
			window.selectionTime = glfwGetTime();

			// Update renderer and request redraw
			window.GetRenderer().UploadSelection(window);
			window.RequestRedraw();

			// Close dialog
			showSelectionDialog = false;
			ImGui::CloseCurrentPopup();
		}

		ImGui::SameLine();
		if (ImGui::Button("Cancel", ImVec2(120, 0))) {
			showSelectionDialog = false;
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

void UI::ShowSavePromptDialog(Window& window) {
	if (!showSavePromptDialog) return;

	Scene& scene = window.GetScene();

	ImGui::OpenPopup("Save Changes?");
	ImVec2 center = ImGui::GetMainViewport()->GetCenter();
	ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

	if (ImGui::BeginPopupModal("Save Changes?", &showSavePromptDialog, ImGuiWindowFlags_AlwaysAutoResize)) {
		ImGui::Text("The geometry has been modified.");
		ImGui::Text("Do you want to save the changes before exiting?");
		ImGui::Separator();

		if (ImGui::Button("Save", ImVec2(120, 0))) {
			// Save the scene
			if (scene.Save()) {
				DEBUG("Scene saved successfully");
				scene.SetGeometryModified(false);
			}
			// Close the dialog and exit
			showSavePromptDialog = false;
			ImGui::CloseCurrentPopup();
			glfwSetWindowShouldClose(window.GetGLFWWindow(), GLFW_TRUE);
		}

		ImGui::SameLine();
		if (ImGui::Button("Don't Save", ImVec2(120, 0))) {
			// Exit without saving
			showSavePromptDialog = false;
			ImGui::CloseCurrentPopup();
			glfwSetWindowShouldClose(window.GetGLFWWindow(), GLFW_TRUE);
		}

		ImGui::SameLine();
		if (ImGui::Button("Cancel", ImVec2(120, 0))) {
			// Cancel exit
			showSavePromptDialog = false;
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

void UI::UpdateFrameStats(double frameDeltaTime) {
	constexpr float updateInterval = 0.5f; // Update every 500ms
	++frameCount;
	deltaTime += frameDeltaTime;
	if (deltaTime >= updateInterval) {
		fps = static_cast<double>(frameCount) / deltaTime;
		deltaTime = 0;
		frameCount = 0;
	}
}

void UI::SetupStyle() {
	ImGuiStyle& style = ImGui::GetStyle();

	// Color scheme
	ImVec4* colors = style.Colors;
	colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 0.9f);
	colors[ImGuiCol_MenuBarBg] = ImVec4(0.2f, 0.2f, 0.2f, 1.f);
	colors[ImGuiCol_Header] = ImVec4(0.3f, 0.3f, 0.3f, 1.f);
	colors[ImGuiCol_HeaderHovered] = ImVec4(0.4f, 0.4f, 0.4f, 1.f);
	colors[ImGuiCol_HeaderActive] = ImVec4(0.5f, 0.5f, 0.5f, 1.f);

	// Spacing
	style.WindowPadding = ImVec2(8, 8);
	style.ItemSpacing = ImVec2(6, 4);
	style.ItemInnerSpacing = ImVec2(4, 4);
	style.WindowRounding = 5.f;
	style.FrameRounding = 3.f;
}

void UI::SetUserFontScale(float scale) {
	float& currentScale = Window::GetCurrentWindow().userFontScale;
	const float ratio = scale / currentScale;
	if (ratio != 1.f)
		ImGui::GetStyle().ScaleAllSizes(ratio);
	ImGui::GetIO().FontGlobalScale = currentScale = scale;
	SetupStyle();
	Window::RequestRedraw();
}

void UI::ShowRenderingControls(Window& window) {
	ImGui::Text("Rendering");
	ImGui::Separator();

	// Font scale: user-controlled base scale
	float userFontScale = window.userFontScale;
	if (ImGui::InputFloat("Font Scale", &userFontScale, 0.1f, 0.5f, "%.2f"))
		SetUserFontScale(userFontScale);

	// Background color
	if (ImGui::ColorEdit3("Background", window.clearColor.data()))
		window.RequestRedraw();

	// Render-only-on-change optimization
	ImGui::Checkbox("Render Only on Change", &window.renderOnlyOnChange);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Optimize performance by rendering only when scene changes\n"
						  "Reduces CPU/GPU usage for static scenes");

	// Image overlay opacity (only show when in camera view mode)
	if (window.GetCamera().IsCameraViewMode()) {
		ImGui::Separator();
		ImGui::Text("Image Overlay");
		if (ImGui::SliderFloat("Opacity", &window.imageOverlayOpacity, 0.f, 1.f, "%.2f"))
			window.RequestRedraw();
		ImGui::Text("Camera ID: %d", (int)window.GetCamera().GetCurrentCamID());
	}

	// Arcball gizmo controls (only show when in arcball mode)
	if (window.GetControlMode() == Window::CONTROL_ARCBALL) {
		ImGui::Separator();
		ImGui::Text("Arcball Gizmos");
		bool enableGizmos = window.GetArcballControls().getEnableGizmos();
		if (ImGui::Checkbox("Show Gizmos", &enableGizmos)) {
			window.GetArcballControls().setEnableGizmos(enableGizmos);
			window.RequestRedraw();
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("Show arcball gizmos (replaces coordinate axes)");
		if (enableGizmos) {
			// Gizmo center control (indented under main gizmo control)
			ImGui::SameLine();
			bool enableGizmosCenter = window.GetArcballControls().getEnableGizmosCenter();
			if (ImGui::Checkbox("Show Center", &enableGizmosCenter)) {
				window.GetArcballControls().setEnableGizmosCenter(enableGizmosCenter);
				window.RequestRedraw();
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("Show small axes at the center of the trackball");
		}
	}
}

void UI::ShowPointCloudControls(Window& window) {
	ImGui::Text("Point Cloud");
	ImGui::Separator();

	if (ImGui::Checkbox("Show Point Cloud", &window.showPointCloud))
		window.RequestRedraw();
	if (window.showPointCloud) {
		ImGui::Indent();
		if (ImGui::SliderFloat("Point Size", &window.pointSize, 1.f, 10.f))
			window.RequestRedraw();
		// Check if normals are available
		const MVS::Scene& scene = window.GetScene().GetScene();
		if (!scene.pointcloud.normals.empty()) {
			if (ImGui::Checkbox("Show Normals", &window.showPointCloudNormals))
				window.RequestRedraw();
			if (window.showPointCloudNormals) {
				ImGui::Indent();
				if (ImGui::SliderFloat("Normal Length", &window.pointNormalLength, 0.001f, 0.1f, "%.3f")) {
					// Re-upload point cloud with new normal length
					window.GetRenderer().UploadPointCloud(scene.pointcloud, window.pointNormalLength);
					window.RequestRedraw();
				}
				ImGui::Unindent();
			}
		} else {
			// Disable the checkbox if no normals are available
			bool disabled = false;
			ImGui::BeginDisabled();
			ImGui::Checkbox("Show Normals (NA)", &disabled);
			ImGui::EndDisabled();
		}
		ImGui::Unindent();
	}
}

void UI::ShowMeshControls(Window& window) {
	ImGui::Text("Mesh");
	ImGui::Separator();

	if (ImGui::Checkbox("Show Mesh", &window.showMesh))
		window.RequestRedraw();
	if (window.showMesh) {
		ImGui::Indent();
		if (ImGui::Checkbox("Wireframe", &window.showMeshWireframe))
			window.RequestRedraw();
		if (ImGui::Checkbox("Textured", &window.showMeshTextured))
			window.RequestRedraw();

		// Sub-mesh controls
		if (!window.meshSubMeshVisible.empty()) {
			ImGui::Separator();
			ImGui::Text("Sub-meshes (%zu total)", window.meshSubMeshVisible.size());

			// All/None buttons for convenience
			ImGui::SameLine();
			if (ImGui::SmallButton("All")) {
				std::fill(window.meshSubMeshVisible.begin(), window.meshSubMeshVisible.end(), true);
				window.RequestRedraw();
			}
			ImGui::SameLine();
			if (ImGui::SmallButton("None")) {
				std::fill(window.meshSubMeshVisible.begin(), window.meshSubMeshVisible.end(), false);
				window.RequestRedraw();
			}

			// Individual sub-mesh checkboxes
			FOREACH(i, window.meshSubMeshVisible) {
				// Create a meaningful label for each sub-mesh
				String label = String::FormatString("Sub-mesh %zu", i);
				// Add texture info if available
				bool isVisible = window.meshSubMeshVisible[i];
				if (ImGui::Checkbox(label, &isVisible)) {
					window.meshSubMeshVisible[i] = isVisible;
					window.RequestRedraw();
				}
			}
		}
		ImGui::Unindent();
	}
}

void UI::ShowSelectionOverlay(const Window& window) {
	if (!showSelectionOverlay) return;

	// Only show if there's a valid selection
	if (window.selectionType == Window::SEL_NA) return;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
								   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
								   ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImVec2 work_pos = vp->WorkPos;
	ImVec2 work_size = vp->WorkSize;

	// Position in bottom left corner
	ImVec2 window_pos;
	window_pos.x = work_pos.x + PAD;
	window_pos.y = work_pos.y + work_size.y - PAD;

	ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, ImVec2(0.f, 1.f));
	ImGui::SetNextWindowBgAlpha(0.35f);

	if (ImGui::Begin("Selection Info", &showSelectionOverlay, window_flags)) {
		// Check for double-click on the selection overlay to open selection dialog
		if (ImGui::IsWindowHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
			showSelectionDialog = true;
		const Scene& scene = window.GetScene();
		const bool hasSelectionIds = window.HasSelectionIds();
		const IDX selectionIdx = window.GetSelectionId();
		switch (window.selectionType) {
		case Window::SEL_TRIANGLE: {
			const MVS::Scene& mvs_scene = scene.GetScene();
			ImGui::Text("Face selected:");
			if (hasSelectionIds)
				ImGui::Text("  index: %zu", selectionIdx);
			if (hasSelectionIds && !mvs_scene.mesh.IsEmpty() && selectionIdx < mvs_scene.mesh.faces.size()) {
				const MVS::Mesh::Face& face = mvs_scene.mesh.faces[selectionIdx];
				ImGui::Text("  vertex 1: %u (%.3f, %.3f, %.3f)", face[0],
					window.selectionPoints[0].x, window.selectionPoints[0].y, window.selectionPoints[0].z);
				ImGui::Text("  vertex 2: %u (%.3f, %.3f, %.3f)", face[1],
					window.selectionPoints[1].x, window.selectionPoints[1].y, window.selectionPoints[1].z);
				ImGui::Text("  vertex 3: %u (%.3f, %.3f, %.3f)", face[2],
					window.selectionPoints[2].x, window.selectionPoints[2].y, window.selectionPoints[2].z);
			}
			break; }
		case Window::SEL_POINT: {
			const MVS::Scene& mvs_scene = scene.GetScene();
			ImGui::Text("Point selected:");
			if (hasSelectionIds)
				ImGui::Text("  index: %zu (%.3f, %.3f, %.3f)",
					selectionIdx,
					window.selectionPoints[0].x, window.selectionPoints[0].y, window.selectionPoints[0].z);

			// Show view information if available
			if (hasSelectionIds && !mvs_scene.pointcloud.pointViews.empty() && selectionIdx < mvs_scene.pointcloud.pointViews.size()) {
				const MVS::PointCloud::ViewArr& views = mvs_scene.pointcloud.pointViews[selectionIdx];
				if (!views.empty()) {
					ImGui::Text("  views: %u", views.size());
					// Show first few views to avoid overwhelming the display
					const unsigned maxViewsToShow = MINF(8u, views.size());
					for (unsigned v = 0; v < maxViewsToShow; ++v) {
						const MVS::PointCloud::View idxImage = views[v];
						if (idxImage < mvs_scene.images.size()) {
							const MVS::Image& imageData = mvs_scene.images[idxImage];
							const Point2 x(imageData.camera.TransformPointW2I(Cast<REAL>(window.selectionPoints[0])));
							const float conf = mvs_scene.pointcloud.pointWeights.empty() ? 0.f :
								mvs_scene.pointcloud.pointWeights[selectionIdx][v];

							String fileName = Util::getFileNameExt(imageData.name);
							ImGui::Text("    %d (%s at %.1f %.1f px, %.2f conf)",
								idxImage, fileName.c_str(), x.x, x.y, conf);
						}
					}
					if (views.size() > maxViewsToShow && mvs_scene.IsValid())
						ImGui::Text("    ... and %u more", views.size() - maxViewsToShow);
				}
			}
			break; }
		case Window::SEL_CAMERA: {
			const ImageArr& images = scene.GetImages();
			const MVS::Scene& mvs_scene = scene.GetScene();
			if (hasSelectionIds && selectionIdx < images.size()) {
				const Image& image = images[selectionIdx];
				if (image.idx < mvs_scene.images.size()) {
					const MVS::Image& imageData = mvs_scene.images[image.idx];
					const MVS::Camera& camera = imageData.camera;
					Point3 eulerAngles;
					camera.R.GetRotationAnglesZYX(eulerAngles.x, eulerAngles.y, eulerAngles.z);

					ImGui::Text("Camera selected:");
					ImGui::Text("  index: %u (ID: %u)", image.idx, imageData.ID);
					ImGui::Text("  name: %s", Util::getFileNameExt(imageData.name).c_str());
					if (!imageData.maskName.empty()) {
						ImGui::Text("  mask: %s", Util::getFileNameExt(imageData.maskName).c_str());
					}
					ImGui::Text("  image size: %ux%u", imageData.width, imageData.height);
					ImGui::Text("  intrinsics: fx %.1f, fy %.1f", camera.K(0, 0), camera.K(1, 1));
					ImGui::Text("             cx %.1f, cy %.1f", camera.K(0, 2), camera.K(1, 2));
					ImGui::Text("  position: %.3g, %.3g, %.3g", camera.C.x, camera.C.y, camera.C.z);
					ImGui::Text("  rotation: %.1f°, %.1f°, %.1f°",
						R2D(eulerAngles.x), R2D(eulerAngles.y), R2D(eulerAngles.z));
					ImGui::Text("  avg depth: %.2g", imageData.avgDepth);
					ImGui::Text("  neighbors: %u", (unsigned)imageData.neighbors.size());
				}
			}
			break; }
		}
		if (window.GetCamera().IsCameraViewMode()) {
			// Show camera view information if in camera view mode
			const MVS::Scene& mvs_scene = scene.GetScene();
			const Image& image = scene.GetImages()[window.GetCamera().GetCurrentCamID()];
			ASSERT(image.idx < mvs_scene.images.size());
			const MVS::Image& imageData = mvs_scene.images[image.idx];
			ImGui::Separator();
			ImGui::Text("Camera View Mode:");
			ImGui::Text("  index: %u (ID: %u)", image.idx, imageData.ID);
			ImGui::Text("  Image: %s", Util::getFileNameExt(imageData.name).c_str());
		}
	}
	ImGui::End();
}

void UI::UpdateMenuVisibility() {
	bool mouseNearMenu = IsMouseNearMenuArea();
	bool menuInUse = IsMenuInUse();
	double currentTime = glfwGetTime();

	// Show menu if mouse is near top or menu is in use
	if (mouseNearMenu || menuInUse) {
		showMainMenu = true;
		lastMenuInteraction = currentTime;
	}
	// Hide menu if enough time has passed since last interaction
	else if (showMainMenu && (currentTime - lastMenuInteraction) > menuFadeOutDelay) {
		showMainMenu = false;
	}

	// Track if menu visibility changed
	menuWasVisible = showMainMenu;
}

bool UI::IsMouseNearMenuArea() const {
	ImGuiIO& io = ImGui::GetIO();

	// Check if mouse position is valid and near the top of the screen
	if (io.MousePos.x < 0 || io.MousePos.y < 0) return false;

	return io.MousePos.y <= menuTriggerHeight;
}

bool UI::IsMenuInUse() const {
	// Menu is in use if menu-related dialogs are open
	if (showAboutDialog || showHelpDialog || showExportDialog)
		return true;

	// Check if any menu-related popup is open (but not all popups)
	if (ImGui::IsPopupOpen("About", ImGuiPopupFlags_None) ||
		ImGui::IsPopupOpen("Help", ImGuiPopupFlags_None))
		return true;

	// Check if the main menu bar is currently active or being interacted with
	if (showMainMenu) {
		// Check if any menu item is active, focused, or hovered
		if (ImGui::IsAnyItemActive() || ImGui::IsAnyItemFocused() || ImGui::IsAnyItemHovered())
			return true;

		// Check if any menu popup is open (File, View, Help menus)
		if (ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopup))
			return true;
	}

	return false;
}

// Append log messages from the Log system (may be called from any thread)
void UI::RecordLog(const String& msg)
{
	// Thread-safe append to buffer. We don't touch ImGui state here.
	{
		std::lock_guard<std::mutex> lock(logMutex);
		logBuffer.push_back(msg);
		while (logBuffer.size() > MAX_UI_LOG_LINES)
			logBuffer.pop_front();
	}
	Window::RequestRedraw();
}

bool UI::WantCaptureMouse() const {
	return ImGui::GetIO().WantCaptureMouse;
}

bool UI::WantCaptureKeyboard() const {
	return ImGui::GetIO().WantCaptureKeyboard;
}

void UI::HandleGlobalKeys(Window& window) {
	// Handle UI-specific escape behavior for closing dialogs and defocusing
	if (ImGui::IsKeyReleased(ImGuiKey_Escape)) {
		// If camera in view mode, exit camera view
		if (window.GetCamera().IsCameraViewMode()) {
			window.GetCamera().DisableCameraViewMode();
			return;
		}
		// If any dialog is open, close it
		if (showAboutDialog) {
			showAboutDialog = false;
			return;
		}
		if (showHelpDialog) {
			showHelpDialog = false;
			return;
		}
		if (showExportDialog) {
			showExportDialog = false;
			return;
		}
		if (showSceneInfo) {
			showSceneInfo = false;
			return;
		}
		if (showCameraInfoDialog) {
			showCameraInfoDialog = false;
			return;
		}
		if (showCameraControls) {
			showCameraControls = false;
			return;
		}
		if (showSelectionDialog) {
			showSelectionDialog = false;
			return;
		}
		if (showRenderSettings) {
			showRenderSettings = false;
			return;
		}
		if (showBoundingBoxControls) {
			showBoundingBoxControls = false;
			return;
		}
		if (showDensifyWorkflow) {
			showDensifyWorkflow = false;
			return;
		}
		if (showReconstructWorkflow) {
			showReconstructWorkflow = false;
			return;
		}
		if (showRefineWorkflow) {
			showRefineWorkflow = false;
			return;
		}
		if (showTextureWorkflow) {
			showTextureWorkflow = false;
			return;
		}
		if (showBatchWorkflow) {
			showBatchWorkflow = false;
			return;
		}

		// If any popup is open, close it
		if (ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopup)) {
			ImGui::CloseCurrentPopup();
			return;
		}

		// Clear focus from any focused window or item
		ImGui::SetWindowFocus(nullptr);
		ImGui::ClearActiveID();

		// Also hide the main menu if it's visible
		if (showMainMenu)
			showMainMenu = false;
	}
}

void UI::ShowWorkflowWindows(Window& window) {
	ShowDensifyWorkflowWindow(window);
	ShowReconstructWorkflowWindow(window);
	ShowRefineWorkflowWindow(window);
	ShowTextureWorkflowWindow(window);
	ShowBatchWorkflowWindow(window);
	ShowEstimateROIWorkflowWindow(window);
}

void UI::ShowEstimateROIWorkflowWindow(Window& window) {
	if (!showEstimateROIWorkflow)
		return;

	Scene& scene = window.GetScene();
	const MVS::Scene& mvsScene = scene.GetScene();
	const bool hasPoints = mvsScene.IsValid() && mvsScene.pointcloud.IsValid();

	ImGui::SetNextWindowSize(ImVec2(360.f, 140.f), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Estimate ROI##workflow", &showEstimateROIWorkflow)) {
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted("Estimate Region-Of-Interest (ROI) from the scene point-cloud.");
	ImGui::Separator();

	Scene::EstimateROIWorkflowOptions& opts = scene.GetEstimateROIWorkflowOptions();
	ImGui::InputFloat("Scale (ROI multiplier)", &opts.scaleROI, 0.01f, 0.1f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Multiply computed ROI extents by this factor (default 1.1).");

	const char* axisLabels[] = { "Auto (-1)", "X (0)", "Y (1)", "Z (2)" };
	int axisIndex = (opts.upAxis < 0) ? 0 : (opts.upAxis + 1);
	if (ImGui::Combo("Up Axis", &axisIndex, axisLabels, IM_ARRAYSIZE(axisLabels))) {
		opts.upAxis = (axisIndex == 0) ? -1 : (axisIndex - 1);
	}

	ImGui::Separator();
	const bool canRun = scene.IsOpen() && hasPoints;
	ImGui::BeginDisabled(!canRun || scene.IsWorkflowRunning());
	if (ImGui::Button("Run")) {
		showEstimateROIWorkflow = false;
		scene.RunEstimateROIWorkflow(opts);
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button("Close"))
		showEstimateROIWorkflow = false;
	if (!canRun)
		ImGui::TextDisabled("Requires a loaded scene with a valid point-cloud.");

	ImGui::End();
}

void UI::ShowDensifyWorkflowWindow(Window& window) {
	if (!showDensifyWorkflow)
		return;

	Scene& scene = window.GetScene();
	Scene::DensifyWorkflowOptions& opts = scene.GetDensifyWorkflowOptions();
	const MVS::Scene& mvsScene = scene.GetScene();
	const bool hasImages = mvsScene.IsValid();
	ImGui::SetNextWindowSize(ImVec2(420.f, 0.f), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Densify Point Cloud##workflow", &showDensifyWorkflow)) {
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted("Generate a dense point-cloud from the current scene.");
	ImGui::Separator();

	int resolutionLevel = (int)opts.resolutionLevel;
	if (ImGui::SliderInt("Resolution Level", &resolutionLevel, 0, 6))
		opts.resolutionLevel = (unsigned)MAXF(resolutionLevel, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("How many times to scale down the images before dense reconstruction (0=original, 1=half, 2=quarter, etc.).\nHigher values process faster but produce less detail.");

	int maxResolution = (int)opts.maxResolution;
	if (ImGui::InputInt("Max Resolution", &maxResolution))
		opts.maxResolution = (unsigned)MAXF(maxResolution, 32);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Maximum image resolution in pixels. Images larger than this will be downscaled to this resolution.\nSet to 0 for no limit.");

	int minResolution = (int)opts.minResolution;
	if (ImGui::InputInt("Min Resolution", &minResolution)) {
		minResolution = MAXF(minResolution, 1);
		if (opts.maxResolution)
			minResolution = MINF(minResolution, (int)opts.maxResolution);
		opts.minResolution = (unsigned)minResolution;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum image resolution in pixels.\nImages can not be downscaled to a resolution smaller than this.");

	int subLevels = (int)opts.subResolutionLevels;
	if (ImGui::SliderInt("Sub-resolution Levels", &subLevels, 0, 4))
		opts.subResolutionLevels = (unsigned)MAXF(subLevels, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of additional lower resolution levels to process for better multi-scale depth estimation.\n0 means only process at the selected resolution level.");

	int numViews = (int)opts.numViews;
	if (ImGui::SliderInt("Number of Views", &numViews, 0, 32))
		opts.numViews = (unsigned)MAXF(numViews, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of neighbor images to use for depth estimation (0 to select valid views).\nMore views increase accuracy, but slow down processing.");

	int minViews = (int)opts.minViews;
	if (ImGui::SliderInt("Minimum Views Neighbors", &minViews, 1, 6))
		opts.minViews = (unsigned)MAXF(minViews, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum number of views in which a point must be visible to be considered during neighbor views estimation.\nHigher values produce more similar neighbor views, but may discard some valid points.");

	int minViewsTrust = (int)opts.minViewsTrust;
	if (ImGui::SliderInt("Trusted Views Initialization", &minViewsTrust, 1, 6))
		opts.minViewsTrust = (unsigned)MAXF(minViewsTrust, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum number of views for a point to be considered for approximating the depth-maps\nduring initialization (<2 - random initialization).");

	int minViewsFuse = (int)opts.minViewsFuse;
	if (ImGui::SliderInt("Views for Fusion", &minViewsFuse, 1, 12))
		opts.minViewsFuse = (unsigned)MAXF(minViewsFuse, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum number of views required to include a depth point in the final fused point cloud.\nHigher values produce cleaner results, but may lose coverage.");

	int estimationIters = (int)opts.estimationIters;
	if (ImGui::SliderInt("Estimation Iterations", &estimationIters, 1, 10))
		opts.estimationIters = (unsigned)MAXF(estimationIters, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of iterations for photometric refinement of each depth estimate.\nMore iterations improve accuracy, but increase computation time.");

	int geometricIters = (int)opts.geometricIters;
	if (ImGui::SliderInt("Geometric Iterations", &geometricIters, 0, 5))
		opts.geometricIters = (unsigned)MAXF(geometricIters, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of iterations for geometric consistency filtering (0 disabled).\nMore iterations may produce more accurate results, but increase computation time.");

	const char* fuseLabels[] = { "Merge only", "Fuse", "Dense fuse" };
	int fuseFilter = (int)opts.fuseFilter;
	if (ImGui::Combo("Fusion Filter", &fuseFilter, fuseLabels, IM_ARRAYSIZE(fuseLabels)))
		opts.fuseFilter = (unsigned)fuseFilter;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Fusion quality level:\n- Merge only: Fast, just merge all points\n- Fuse: Standard fusion with outlier removal\n- Dense fuse: Slower but produces the densest, highest quality result,\n.  exploiting neighbor pixel estimates");

	const char* fusionModeLabels[] = {
		"Depth + Fusion (0)",
		"Depth only (1)",
		"Export depth (-1)",
		"Fuse disparity (-2)"
	};
	const int fusionModeValues[] = { 0, 1, -1, -2 };
	int fusionIndex = 0;
	for (int i = 0; i < IM_ARRAYSIZE(fusionModeValues); ++i)
		if (fusionModeValues[i] == opts.fusionMode) { fusionIndex = i; break; }
	if (ImGui::Combo("Fusion Mode", &fusionIndex, fusionModeLabels, IM_ARRAYSIZE(fusionModeLabels)))
		opts.fusionMode = fusionModeValues[fusionIndex];
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Processing mode:\n- Depth + Fusion: Complete pipeline (compute depth maps and fuse into point cloud)\n- Depth only: Only generate depth maps\n- Export depth: Save depth maps to disk without fusion\n- Fuse disparity: Fuse existing disparity maps into point cloud");

	ImGui::Checkbox("Estimate Colors", &opts.estimateColors);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Estimate color for each point in the dense cloud based on the source images.\nDisable to skip color computation.");
	ImGui::Checkbox("Estimate Normals", &opts.estimateNormals);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Store estimated normals for each point.\nNormals are useful for surface reconstruction and visualization.");
	ImGui::Checkbox("Remove Depth Maps", &opts.removeDepthMaps);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Delete intermediate depth maps after fusion to save disk space.\nDisable to keep depth maps for later inspection or re-fusion.");
	ImGui::Checkbox("Post-process Depth Maps", &opts.postprocess);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Apply additional filtering and refinement to depth maps before fusion.\nImproves quality but increases processing time.");
	ImGui::DragFloat("Sample Mesh Neighbors", &opts.sampleMeshNeighbors, 0.25f, -10000.f, 10000.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of mesh samples to use for neighbor views estimation.\n- Sampling density per squared unit area (if >0)\n- Absolute number of points (if <0)\n- Use existing vertices as samples (if ==0)");
	ImGui::Checkbox("Crop to ROI", &opts.cropToROI);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Restrict processing to the Region of Interest (ROI) if defined.\nPoints outside ROI will be discarded.");
	ImGui::DragFloat("ROI Border (%)", &opts.borderROI, 0.1f, -100.f, 100.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Percentage to expand (positive) or shrink (negative) the ROI border.\nUseful to include context or tighten the bounds.");
	#if defined(_USE_CUDA) || defined(_USE_METAL)
	static char gpuDeviceBuf[64] = {};
	if (gpuDeviceBuf[0] == '\0')
		strncpy(gpuDeviceBuf, SEACAVE::CUDA::desiredDeviceIDs.c_str(), sizeof(gpuDeviceBuf)-1);
	if (ImGui::InputText("GPU Device(s)", gpuDeviceBuf, sizeof(gpuDeviceBuf)))
		SEACAVE::CUDA::desiredDeviceIDs = gpuDeviceBuf;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("GPU device(s) for depth-map estimation\n(comma-separated IDs, -1 for best GPU, empty for CPU)");
	#endif

	ImGui::Separator();
	const bool canRun = scene.IsOpen() && hasImages;
	ImGui::BeginDisabled(!canRun || scene.IsWorkflowRunning());
	if (ImGui::Button("Run")) {
		showDensifyWorkflow = false;
		scene.RunDensifyWorkflow(opts);
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button("Close"))
		showDensifyWorkflow = false;
	if (!canRun)
		ImGui::TextDisabled("Open a scene with calibrated images.");

	ImGui::End();
}

void UI::ShowReconstructWorkflowWindow(Window& window) {
	if (!showReconstructWorkflow)
		return;

	Scene& scene = window.GetScene();
	Scene::ReconstructMeshWorkflowOptions& opts = scene.GetReconstructMeshWorkflowOptions();
	const MVS::Scene& mvsScene = scene.GetScene();
	const bool hasPoints = mvsScene.IsValid() && mvsScene.pointcloud.IsValid();
	ImGui::SetNextWindowSize(ImVec2(420.f, 0.f), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Reconstruct Mesh##workflow", &showReconstructWorkflow)) {
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted("Build a surface from the dense point-cloud.");
	ImGui::Separator();
	ImGui::DragFloat("Min Point Distance", &opts.minPointDistance, 0.1f, 0.f, 20.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum distance in pixels between the projection of two 3D points to consider them different while triangulating (0 - disabled).\nIncrease for smoother, coarser meshes; decrease for finer detail.");
	ImGui::Checkbox("Use Free-space Support", &opts.useFreeSpaceSupport);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Use camera ray information to carve out empty space and improve surface reconstruction.\nRecommended for outdoor or complex scenes.");
	ImGui::Checkbox("Integrate Only ROI", &opts.useOnlyROI);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Process only points inside the Region of Interest.\nUseful to focus reconstruction on a specific area and reduce computation.");
	ImGui::Checkbox("Constant Weight", &opts.constantWeight);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Use uniform weighting for all points instead of confidence-based weighting.\nMay help with uniformly sampled point clouds, but can reduce quality.");

	ImGui::Separator();
	ImGui::DragFloat("Thickness Factor", &opts.thicknessFactor, 0.05f, 0.f, 10.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Multiplier adjusting the minimum thickness considered during visibility weighting.\nHigher values increase robustness to noise, but can create holes or remove thin surfaces.");
	ImGui::DragFloat("Quality Factor", &opts.qualityFactor, 0.05f, 0.f, 10.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Multiplier adjusting the quality weight considered during graph-cut.");
	ImGui::SliderFloat("Decimate Mesh", &opts.decimateMesh, 0.f, 1.f, "%.3f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Reduce mesh complexity after reconstruction (1 = no decimation).\nUseful to create lower-poly meshes for real-time rendering.");
	int targetFaces = (int)opts.targetFaceNum;
	if (ImGui::InputInt("Target Face Count", &targetFaces))
		opts.targetFaceNum = (unsigned)MAXF(targetFaces, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Target number of faces for the output mesh. Set to 0 to use the decimation ratio instead.\nUseful for creating meshes with specific polygon budgets.");
	ImGui::DragFloat("Remove Spurious", &opts.removeSpurious, 1.f, 0.f, 200.f, "%.1f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Remove spurious surfaces (isolated or floating geometry) with fewer than this many connected faces.\nHigher values remove more isolated pieces (0 - disabled)");
	ImGui::Checkbox("Remove Spikes", &opts.removeSpikes);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Automatically detect and remove spike artifacts (sharp, thin protrusions) from the mesh. Recommended for cleaner results.");
	int closeHoles = (int)opts.closeHoles;
	if (ImGui::InputInt("Close Holes", &closeHoles))
		opts.closeHoles = (unsigned)MAXF(closeHoles, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Maximum hole size (in edges) to automatically fill.\nLarger values close bigger holes (0 - disabled)");
	int smoothSteps = (int)opts.smoothSteps;
	if (ImGui::InputInt("Smooth Iterations", &smoothSteps))
		opts.smoothSteps = (unsigned)MAXF(smoothSteps, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of Laplacian smoothing iterations to apply.\nMore iterations create smoother surfaces, but may lose detail (0 - disabled)");
	ImGui::DragFloat("Edge Length", &opts.edgeLength, 0.01f, 0.f, 10.f, "%.3f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Target edge length for mesh faces (in scene units).\nControls mesh resolution and uniformity (0 - disabled)");
	ImGui::Checkbox("Crop to ROI", &opts.cropToROI);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Crop the final mesh to the Region of Interest bounds.\nVertices and faces outside the ROI will be removed.");

	ImGui::Separator();
	const bool canRun = scene.IsOpen() && hasPoints;
	ImGui::BeginDisabled(!canRun || scene.IsWorkflowRunning());
	if (ImGui::Button("Run")) {
		showReconstructWorkflow = false;
		scene.RunReconstructMeshWorkflow(opts);
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button("Close"))
		showReconstructWorkflow = false;
	if (!canRun)
		ImGui::TextDisabled("Requires a dense point-cloud.");

	ImGui::End();
}

void UI::ShowRefineWorkflowWindow(Window& window) {
	if (!showRefineWorkflow)
		return;

	Scene& scene = window.GetScene();
	Scene::RefineMeshWorkflowOptions& opts = scene.GetRefineMeshWorkflowOptions();
	ImGui::SetNextWindowSize(ImVec2(420.f, 0.f), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Refine Mesh##workflow", &showRefineWorkflow)) {
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted("Improve mesh quality using photo-consistency.");
	ImGui::Separator();
	int resolutionLevel = (int)opts.resolutionLevel;
	if (ImGui::SliderInt("Resolution Level", &resolutionLevel, 0, 6))
		opts.resolutionLevel = (unsigned)MAXF(resolutionLevel, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Image resolution scale for refinement (0=original, 1=half, etc.).\nHigher values are faster but less detailed.\nStart with lower resolution for coarse refinement.");
	int minResolution = (int)opts.minResolution;
	if (ImGui::InputInt("Min Resolution", &minResolution))
		opts.minResolution = (unsigned)MAXF(minResolution, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum image resolution in pixels.\nImages can not be downscaled to a resolution smaller than this.");
	int maxViews = (int)opts.maxViews;
	if (ImGui::SliderInt("Max Views", &maxViews, 1, 16))
		opts.maxViews = (unsigned)MAXF(maxViews, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Maximum number of view neighbors to use during refinement.\nMore views improve accuracy, but increase computation time and memory usage.");
	ImGui::SliderFloat("Decimate Input", &opts.decimateMesh, 0.f, 1.f, "%.3f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Simplify the input mesh before refinement (0 = no decimation, 1 = maximum).\nUseful for reducing computation on high-poly meshes.");
	int closeHoles = (int)opts.closeHoles;
	if (ImGui::InputInt("Close Holes", &closeHoles))
		opts.closeHoles = (unsigned)MAXF(closeHoles, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Maximum hole size (in edges) to fill before refinement.\nClosing holes prevents artifacts at boundaries (0 - disabled)");
	int ensureEdge = (int)opts.ensureEdgeSize;
	if (ImGui::SliderInt("Ensure Edge Size", &ensureEdge, 0, 2))
		opts.ensureEdgeSize = (unsigned)MAXF(ensureEdge, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Subdivide or collapse edges to ensure uniform size (0=no change, 1=moderate, 2=aggressive).\nHelps create more uniform mesh topology.");
	int maxFaceArea = (int)opts.maxFaceArea;
	if (ImGui::InputInt("Max Face Area", &maxFaceArea))
		opts.maxFaceArea = (unsigned)MAXF(maxFaceArea, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Maximum face area projected in any pair of images that is not subdivided (0 - disabled)");
	int scales = (int)opts.scales;
	if (ImGui::SliderInt("Scales", &scales, 1, 5))
		opts.scales = (unsigned)MAXF(scales, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of multi-scale refinement passes.\nMore scales improve convergence from coarse to fine detail.");
	ImGui::SliderFloat("Scale Step", &opts.scaleStep, 0.1f, 1.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Resolution scaling factor between successive refinement scales.\nLower values create more gradual transitions between scales.");
	const char* pairModes[] = { "Both references", "Alternate", "Left only", "Right only" };
	int alternatePair = (int)opts.alternatePair;
	if (ImGui::Combo("Reference Pair", &alternatePair, pairModes, IM_ARRAYSIZE(pairModes)))
		opts.alternatePair = (unsigned)alternatePair;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Which image pairs to use as reference during multi-view refinement:\n- Both references: Use all paired views (most accurate)\n- Alternate: Switch between left/right (balanced)\n- Left/Right only: Use only one reference (faster, less accurate)");
	ImGui::DragFloat("Regularity Weight", &opts.regularityWeight, 0.05f, 0.f, 10.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Weight for mesh regularity term.\nHigher values produce smoother surfaces, but may lose detail.\nLower values preserve sharp features, but can be noisy.");
	ImGui::DragFloat("Rigidity/Elasticity", &opts.rigidityElasticityRatio, 0.05f, 0.f, 1.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Balance between mesh rigidity and elasticity:\n- 0 = fully elastic (flexible deformation)\n- 1 = fully rigid (minimal deformation)\nAffects how much the mesh can deform.");
	float iters = FLOOR2INT(opts.gradientStep);
	float gstep = (opts.gradientStep-(float)iters)*10;
	ImGui::DragFloat("Gradient Iterations", &iters, 1.f, 0.f, 200.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Number of iterations of gradient descent optimization.");
	ImGui::DragFloat("Gradient Step", &gstep, 0.01f, 0.01f, 10.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Step size for gradient descent optimization.\nLarger values converge faster, but may be unstable.\nSmaller values are more stable, but slower.");
	opts.gradientStep = iters + gstep*0.1f;
	ImGui::DragFloat("Planar Vertex Ratio", &opts.planarVertexRatio, 0.01f, 0.f, 1.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Ratio of vertices to treat as planar (constrained to move along their normal).\nHigher values preserve flat surfaces better, but reduce flexibility.");
	int reduceMemory = (int)opts.reduceMemory;
	if (ImGui::SliderInt("Reduce Memory", &reduceMemory, 0, 3))
		opts.reduceMemory = (unsigned)MAXF(reduceMemory, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Memory reduction strategy:\n- 0 = no reduction (fastest, most memory)\n- 3 = maximum reduction (slowest, least memory)\nUse higher values for large scenes or limited RAM.");

	ImGui::Separator();
	const MVS::Scene& mvsScene = scene.GetScene();
	const bool canRun = scene.IsOpen() && mvsScene.IsValid() && !mvsScene.mesh.IsEmpty();
	ImGui::BeginDisabled(!canRun || scene.IsWorkflowRunning());
	if (ImGui::Button("Run")) {
		showRefineWorkflow = false;
		scene.RunRefineMeshWorkflow(opts);
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button("Close"))
		showRefineWorkflow = false;
	if (!canRun)
		ImGui::TextDisabled("Requires an existing mesh.");

	ImGui::End();
}

void UI::ShowTextureWorkflowWindow(Window& window) {
	if (!showTextureWorkflow)
		return;

	Scene& scene = window.GetScene();
	Scene::TextureMeshWorkflowOptions& opts = scene.GetTextureMeshWorkflowOptions();
	const MVS::Scene& mvsScene = scene.GetScene();
	const bool hasMesh = mvsScene.IsValid() && !mvsScene.mesh.IsEmpty();
	ImGui::SetNextWindowSize(ImVec2(420.f, 0.f), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Texture Mesh##workflow", &showTextureWorkflow)) {
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted("Bake textures onto the current mesh.");
	ImGui::Separator();
	ImGui::SliderFloat("Decimate Mesh", &opts.decimateMesh, 0.f, 1.f, "%.3f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Simplify the mesh before texturing (0 = no decimation, 1 = maximum).\nReduces polygon count to improve texture mapping efficiency.");
	int closeHoles = (int)opts.closeHoles;
	if (ImGui::InputInt("Close Holes", &closeHoles))
		opts.closeHoles = (unsigned)MAXF(closeHoles, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Maximum hole size (in edges) to fill before texturing.\nPrevents texture artifacts at mesh boundaries (0 - disabled)");
	int resolutionLevel = (int)opts.resolutionLevel;
	if (ImGui::SliderInt("Resolution Level", &resolutionLevel, 0, 6))
		opts.resolutionLevel = (unsigned)MAXF(resolutionLevel, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Image resolution scale for texture extraction (0=original, 1=half, etc.).\nHigher values are faster but produce lower quality textures.");
	int minResolution = (int)opts.minResolution;
	if (ImGui::InputInt("Min Resolution", &minResolution))
		opts.minResolution = (unsigned)MAXF(minResolution, 1);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum image resolution in pixels.\nImages can not be downscaled to a resolution smaller than this.");
	int minCommon = (int)opts.minCommonCameras;
	if (ImGui::InputInt("Min Common Cameras", &minCommon))
		opts.minCommonCameras = (unsigned)MAXF(minCommon, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Minimum number of cameras that must see a face for it to be textured.\nHigher values ensure better texture quality but may leave some faces untextured.");
	ImGui::DragFloat("Outlier Threshold", &opts.outlierThreshold, 0.005f, 0.f, 1.f, "%.3f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Threshold for rejecting outliers during views to face assignment.\nHigher values are more permissive.");
	ImGui::DragFloat("Cost Smoothness Ratio", &opts.ratioDataSmoothness, 0.01f, 0.f, 1.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Balance between data term and smoothness term:\n- 0 = prioritize photometric quality\n- 1 = prioritize seam smoothness");
	ImGui::Checkbox("Global Seam Leveling", &opts.globalSeamLeveling);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Apply global color adjustment to minimize exposure differences between texture patches.\nRecommended for better visual consistency across the entire model.");
	ImGui::Checkbox("Local Seam Leveling", &opts.localSeamLeveling);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Apply local color blending along texture seams.\nSmooths transitions between patches.\nWorks well with global seam leveling for best results.");
	int textureMultiple = (int)opts.textureSizeMultiple;
	if (ImGui::InputInt("Texture Size Multiple", &textureMultiple))
		opts.textureSizeMultiple = (unsigned)MAXF(textureMultiple, 0);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Texture dimensions will be multiples of this value (0 - power of two)");

	float color[3] = {
		((opts.emptyColor >> 16) & 0xFF) / 255.f,
		((opts.emptyColor >> 8) & 0xFF) / 255.f,
		(opts.emptyColor & 0xFF) / 255.f
	};
	if (ImGui::ColorEdit3("Empty Color", color, ImGuiColorEditFlags_NoAlpha)) {
		auto toChannel = [](float v) -> uint32_t {
			if (v < 0.f) v = 0.f;
			if (v > 1.f) v = 1.f;
			return (uint32_t)(v * 255.f + 0.5f);
		};
		opts.emptyColor = (toChannel(color[0]) << 16) | (toChannel(color[1]) << 8) | toChannel(color[2]);
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Color to use for unfilled texture regions (areas with no valid projection).\nMagenta is useful for debugging missing texture coverage.");
	ImGui::SliderFloat("Sharpness Weight", &opts.sharpnessWeight, 0.f, 2.f, "%.2f");
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Sharpness weight to be applied on the texture (0 - disabled, 0.5 - good value).");
	int ignoreLabel = opts.ignoreMaskLabel;
	if (ImGui::InputInt("Ignore Mask Label", &ignoreLabel))
		opts.ignoreMaskLabel = ignoreLabel;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Label value to ignore in the image mask, stored in the MVS scene or next to each image with '.mask.png' extension\n(-1 - auto estimate mask for lens distortion, -2 - disabled)");
	int maxTexture = opts.maxTextureSize;
	if (ImGui::InputInt("Max Texture Size", &maxTexture))
		opts.maxTextureSize = MAXF(0, maxTexture);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Maximum texture atlas size in pixels per dimension.\nMultiple textures are created if needed.\nLarger values allow higher resolution textures, but require more memory (0 - no limit)");

	ImGui::Separator();
	const bool canRun = scene.IsOpen() && hasMesh;
	ImGui::BeginDisabled(!canRun || scene.IsWorkflowRunning());
	if (ImGui::Button("Run")) {
		showTextureWorkflow = false;
		scene.RunTextureMeshWorkflow(opts);
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button("Close"))
		showTextureWorkflow = false;
	if (!canRun)
		ImGui::TextDisabled("Requires a mesh and images.");

	ImGui::End();
}

void UI::ShowBatchWorkflowWindow(Window& window) {
	if (!showBatchWorkflow)
		return;

	Scene& scene = window.GetScene();
	Scene::EstimateROIWorkflowOptions& estimateOpts = scene.GetEstimateROIWorkflowOptions();
	Scene::DensifyWorkflowOptions& densifyOpts = scene.GetDensifyWorkflowOptions();
	Scene::ReconstructMeshWorkflowOptions& reconstructOpts = scene.GetReconstructMeshWorkflowOptions();
	Scene::RefineMeshWorkflowOptions& refineOpts = scene.GetRefineMeshWorkflowOptions();
	Scene::TextureMeshWorkflowOptions& textureOpts = scene.GetTextureMeshWorkflowOptions();
	const MVS::Scene& mvsScene = scene.GetScene();
	const bool hasImages = mvsScene.IsValid();
	const bool hasPoints = hasImages && mvsScene.pointcloud.IsValid();
	const bool hasMesh = hasImages && !mvsScene.mesh.IsEmpty();
	ImGui::SetNextWindowSize(ImVec2(400.f, 184.f), ImGuiCond_FirstUseEver);
	if (!ImGui::Begin("Batch Process##workflow", &showBatchWorkflow)) {
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted("Select workflow modules to run sequentially.");
	ImGui::Separator();

	// Persistent selection and ordering
	static bool selectedModules[5] = { true, true, true, true, true }; // Estimate ROI, Densify, Reconstruct, Refine, Texture
	const char* labels[5] = { "Estimate ROI", "Densify Point Cloud", "Reconstruct Mesh", "Refine Mesh", "Texture Mesh" };
	const char* hints[5] = { "requires points", "requires images", "requires points with visibility", "requires mesh", "requires mesh" };
	for (int idx = 0; idx < 5; ++idx) {
		ImGui::PushID(idx);
		// determine if prerequisites are (or will be) met for this module
		bool prereqMet;
		switch (idx) {
		case 0: // Estimate ROI
			if (!(prereqMet = hasPoints))
				selectedModules[idx] = false;
			break;
		case 1: // Densify
			if (!(prereqMet = hasImages))
				selectedModules[idx] = false;
			break;
		case 2: // Reconstruct
			// Reconstruct requires points OR densify selected to produce points
			if (!(prereqMet = hasPoints || selectedModules[1]))
				selectedModules[idx] = false;
			break;
		case 3: // Refine
		case 4: // Texture
			// Refine/Texture require a mesh OR reconstruct selected to produce a mesh
			if (!(prereqMet = hasMesh || selectedModules[2]))
				selectedModules[idx] = false;
			break;
		}
		ImGui::BeginDisabled(!prereqMet);
		ImGui::Checkbox(labels[idx], &selectedModules[idx]);
		ImGui::EndDisabled();
		ImGui::SameLine();
		ImGui::TextDisabled("(%s)", hints[idx]);
		ImGui::PopID();
	}

	ImGui::Separator();
	// Build runnable list
	std::vector<int> runnable;
	for (int idx = 0; idx < 5; ++idx)
		if (selectedModules[idx])
			runnable.push_back(idx);
	const bool canRun = !runnable.empty();
	if (!canRun)
		ImGui::TextDisabled("No runnable modules selected or prerequisites missing.");

	if (ImGui::Button("Run") && canRun) {
		// Close window before running long tasks
		showBatchWorkflow = false;
		ImGui::End();
		// Execute selected modules in order
		FOREACH(i, runnable) {
			const int mod = runnable[i];
			switch (mod) {
			case 0:
				DEBUG("Batch: Running Estimate ROI...");
				scene.RunEstimateROIWorkflow(estimateOpts);
				break;
			case 1:
				DEBUG("Batch: Running Densify Point Cloud...");
				scene.RunDensifyWorkflow(densifyOpts);
				break;
			case 2:
				DEBUG("Batch: Running Reconstruct Mesh...");
				scene.RunReconstructMeshWorkflow(reconstructOpts);
				break;
			case 3:
				DEBUG("Batch: Running Refine Mesh...");
				scene.RunRefineMeshWorkflow(refineOpts);
				break;
			case 4:
				DEBUG("Batch: Running Texture Mesh...");
				scene.RunTextureMeshWorkflow(textureOpts);
				break;
			}
		}
		window.RequestRedraw();
		return; // already ended ImGui for this invocation
	}
	ImGui::SameLine();
	if (ImGui::Button("Close"))
		showBatchWorkflow = false;

	ImGui::End();
}

void* SettingsReadOpen(ImGuiContext*, ImGuiSettingsHandler* handler, const char* name) {
	if (strcmp(name, "Window") == 0)
		return handler->UserData;
	return nullptr;
}

void SettingsReadLine(ImGuiContext*, ImGuiSettingsHandler* handler, void* entry, const char* line) {
	Window& window = *reinterpret_cast<Window*>(handler->UserData);

	float x, y, z, w;
	int intVal;

	if (sscanf(line, "RenderOnlyOnChange=%d", &intVal) == 1) {
		window.renderOnlyOnChange = (intVal != 0);
	}
	else if (sscanf(line, "ClearColor=%f,%f,%f,%f", &x, &y, &z, &w) == 4) {
		window.clearColor = Eigen::Vector4f(x, y, z, w);
	}
	else if (sscanf(line, "CameraSize=%f", &x) == 1) {
		window.cameraSize = x;
	}
	else if (sscanf(line, "CameraDisplayColor=%d", &intVal) == 1) {
		window.cameraDisplayColor =
			intVal == (int)Window::CAMERA_COLOR_JET ? Window::CAMERA_COLOR_JET : Window::CAMERA_COLOR_SOLID;
	}
	else if (sscanf(line, "CameraDisplayType=%d", &intVal) == 1) {
		window.cameraDisplayType =
			intVal == (int)Window::CAMERA_DISPLAY_DOT ? Window::CAMERA_DISPLAY_DOT : Window::CAMERA_DISPLAY_FRUSTUM;
	}
	else if (sscanf(line, "ShowCameraLookAt=%d", &intVal) == 1) {
		window.showCameraLookAt = (intVal != 0);
	}
	else if (sscanf(line, "ShowCameraCoordinateSystem=%d", &intVal) == 1) {
		// Backward compatibility with older saved settings key.
		window.showCameraLookAt = (intVal != 0);
	}
	else if (sscanf(line, "PointSize=%f", &x) == 1) {
		window.pointSize = x;
	}
	else if (sscanf(line, "EstimateSfMNormals=%d", &intVal) == 1) {
		window.GetScene().estimateSfMNormals = (intVal != 0);
	}
	else if (sscanf(line, "EstimateSfMPatches=%d", &intVal) == 1) {
		window.GetScene().estimateSfMPatches = (intVal != 0);
	}
	else if (sscanf(line, "ShowCameras=%d", &intVal) == 1) {
		window.showCameras = (intVal != 0);
	}
	else if (sscanf(line, "ShowMeshWireframe=%d", &intVal) == 1) {
		window.showMeshWireframe = (intVal != 0);
	}
	else if (sscanf(line, "ShowMeshTextured=%d", &intVal) == 1) {
		window.showMeshTextured = (intVal != 0);
	}
	else if (sscanf(line, "ShowBounds=%d", &intVal) == 1) {
		window.showBounds = (intVal != 0);
	}
	else if (sscanf(line, "ImageOverlayOpacity=%f", &x) == 1) {
		window.imageOverlayOpacity = x;
	}
	else if (sscanf(line, "FontScale=%f", &x) == 1) {
		window.GetUI().SetUserFontScale(x);
	}
	else if (sscanf(line, "ArcballRenderGizmos=%d", &intVal) == 1) {
		window.GetArcballControls().setEnableGizmos(intVal != 0);
	}
	else if (sscanf(line, "ArcballRenderGizmosCenter=%d", &intVal) == 1) {
		window.GetArcballControls().setEnableGizmosCenter(intVal != 0);
	}
	else if (sscanf(line, "ArcballRotationSensitivity=%f", &x) == 1) {
		window.GetArcballControls().setRotationSensitivity(x);
	}
	else if (sscanf(line, "ArcballZoomSensitivity=%f", &x) == 1) {
		window.GetArcballControls().setZoomSensitivity(x);
	}
	else if (sscanf(line, "ArcballPanSensitivity=%f", &x) == 1) {
		window.GetArcballControls().setPanSensitivity(x);
	}
}

void SettingsWriteAll(ImGuiContext*, ImGuiSettingsHandler* handler, ImGuiTextBuffer* buf) {
	Window& window = *reinterpret_cast<Window*>(handler->UserData);
	buf->appendf("[%s][Window]\n", handler->TypeName);
	buf->appendf("RenderOnlyOnChange=%d\n", window.renderOnlyOnChange ? 1 : 0);
	buf->appendf("ClearColor=%f,%f,%f,%f\n",
		window.clearColor[0], window.clearColor[1],
		window.clearColor[2], window.clearColor[3]);
	buf->appendf("CameraSize=%f\n", window.cameraSize);
	buf->appendf("CameraDisplayColor=%d\n", (int)window.cameraDisplayColor);
	buf->appendf("CameraDisplayType=%d\n", (int)window.cameraDisplayType);
	buf->appendf("ShowCameraLookAt=%d\n", window.showCameraLookAt ? 1 : 0);
	buf->appendf("PointSize=%f\n", window.pointSize);
	buf->appendf("EstimateSfMNormals=%d\n",
		window.GetScene().estimateSfMNormals ? 1 : 0);
	buf->appendf("EstimateSfMPatches=%d\n",
		window.GetScene().estimateSfMPatches ? 1 : 0);
	buf->appendf("ShowCameras=%d\n", window.showCameras ? 1 : 0);
	buf->appendf("ShowMeshWireframe=%d\n", window.showMeshWireframe ? 1 : 0);
	buf->appendf("ShowMeshTextured=%d\n", window.showMeshTextured ? 1 : 0);
	buf->appendf("ShowBounds=%d\n", window.showBounds ? 1 : 0);
	buf->appendf("ImageOverlayOpacity=%f\n", window.imageOverlayOpacity);
	buf->appendf("FontScale=%f\n", window.userFontScale);
	buf->appendf("ArcballRenderGizmos=%d\n",
		window.GetArcballControls().getEnableGizmos() ? 1 : 0);
	buf->appendf("ArcballRenderGizmosCenter=%d\n",
		window.GetArcballControls().getEnableGizmosCenter() ? 1 : 0);
	buf->appendf("ArcballRotationSensitivity=%f\n",
		window.GetArcballControls().getRotationSensitivity());
	buf->appendf("ArcballZoomSensitivity=%f\n",
		window.GetArcballControls().getZoomSensitivity());
	buf->appendf("ArcballPanSensitivity=%f\n",
		window.GetArcballControls().getPanSensitivity());
}

// Custom settings implementation
bool UI::ShowOpenFileDialog(String& filename, String& geometryFilename) {
	// Use portable-file-dialogs for cross-platform file dialog
	try {
		auto dialog = pfd::open_file(
			"Open Scene File",                   // title
			WORKING_FOLDER_FULL,          // initial path (absolute)
			{
				"OpenMVS Scene Files", "*.mvs",
				"OpenMVS Depth Map Files", "*.dmap",
				"PLY Mesh / Point Cloud Files", "*.ply",
				"GLTF Mesh / Point Cloud Files", "*.gltf",
				"GLB Mesh / Point Cloud Files", "*.glb",
				"OBJ Mesh Files", "*.obj",
				"All Files", "*"
			},                                         // filters
			pfd::opt::multiselect             // options
		);

		// Get the result
		auto result = dialog.result();
		if (!result.empty()) {
			filename = result[0];
			if (result.size() > 1)
				geometryFilename = result[1];
			else
				geometryFilename.clear();
			return true;
		}
	} catch (const std::exception& e) {
		DEBUG("File dialog error: %s", e.what());
	}
	return false;
}

bool UI::ShowSaveFileDialog(String& filename) {
	// Use portable-file-dialogs for cross-platform save dialog
	try {
		auto dialog = pfd::save_file(
			"Save Scene File",                  // title
			WORKING_FOLDER_FULL,         // initial directory (like open dialog)
			{
				"OpenMVS Scene Files", "*.mvs",
				"PLY Mesh / Point Cloud Files", "*.ply",
				"GLTF Mesh / Point Cloud Files", "*.gltf",
				"GLB Mesh / Point Cloud Files", "*.glb",
				"OBJ Mesh Files", "*.obj",
				"All Files", "*"
			},                                        // filters
			pfd::opt::none                   // options
		);

		// Get the result - save_file returns a string directly, not a vector
		auto result = dialog.result();
		if (!result.empty()) {
			filename = result;
			return true;
		}
	} catch (const std::exception& e) {
		DEBUG("File dialog error: %s", e.what());
	}
	return false;
}

bool UI::ShowSaveImageDialog(String& filename) {
	try {
		auto dialog = pfd::save_file(
			"Save Screenshot",                 // title
			WORKING_FOLDER_FULL,        // initial directory (like open dialog)
			{
				"PNG Image", "*.png",
				"JPEG Image", "*.jpg",
				"JPEGXL Image", "*.jxl",
				"All Files", "*"
			},                                        // filters
			pfd::opt::none                   // options
		);

		auto result = dialog.result();
		if (!result.empty()) {
			filename = result;
			return true;
		}
	} catch (const std::exception& e) {
		DEBUG("File dialog error: %s", e.what());
	}
	return false;
}

void UI::SetupCustomSettings(Window& window) {
	// Register custom settings handler
	ImGuiContext& ctx = *ImGui::GetCurrentContext();
	ImGuiSettingsHandler handler;
	handler.TypeName = "ViewerSettings";
	handler.TypeHash = ImHashStr("ViewerSettings");
	handler.ReadOpenFn = SettingsReadOpen;
	handler.ReadLineFn = SettingsReadLine;
	handler.WriteAllFn = SettingsWriteAll;
	handler.UserData = &window; // Pass window pointer as user data
	ctx.SettingsHandlers.push_back(handler);
}
/*----------------------------------------------------------------*/
