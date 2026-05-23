/*
 * Window.cpp
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
#include "Window.h"
#include "ArcballControls.h"
#include "FirstPersonControls.h"
#include "SelectionController.h"
#include "Scene.h"
#ifdef _MSC_VER
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

#ifdef __APPLE__
extern "C" void OpenMVS_InstallFileHandler();
extern "C" void OpenMVS_ConsumePendingOpenFiles(std::vector<std::string>& out);
#endif

using namespace VIEWER;

Window::Window()
	: window(nullptr)
	#ifdef _MSC_VER
	, hIconBig(nullptr)
	, hIconSmall(nullptr)
	#endif
	, devicePixelRatio(1.0, 1.0)
	, currentControlMode(CONTROL_ARCBALL)
	, lastMousePos(0, 0)
	, lastFrame(0.0)
	, selectionType(SEL_NA)
	, selectedNeighborCamera(NO_ID)
	, clearColor(0.3f, 0.4f, 0.5f, 1.f)
	, minViews(2)
	, userFontScale(1.f)
	, cameraSize(0.1f)
	, cameraDisplayColor(CAMERA_COLOR_SOLID)
	, cameraDisplayType(CAMERA_DISPLAY_FRUSTUM)
	, showCameraLookAt(true)
	, pointSize(3.f)
	, pointNormalLength(0.02f)
	, imageOverlayOpacity(0.5f)
	, renderOnlyOnChange(true)
	, showCameras(true)
	, showPointCloud(true)
	, showPointCloudNormals(false)
	, showMesh(true)
	, showMeshWireframe(false)
	, showMeshTextured(true)
	, showBounds(true)
	, pendingScreenshotIncludeUI(false)
	, pendingScreenshotQuit(false)
{
}

Window::~Window() {
	Release();
}

bool Window::Initialize(const cv::Size& size, const String& windowTitle, Scene& scene) {
	title = windowTitle;

	#ifdef __APPLE__
	// Swizzle NSApplication's finishLaunching and install Apple Event handler
	// BEFORE glfwInit() — this ensures our application:openURLs: delegate
	// method is injected into GLFW's delegate before finishLaunching processes
	// the queued file-open event during cold start.
	OpenMVS_InstallFileHandler();
	#endif

	// Initialize GLFW
	if (!glfwInit()) {
		DEBUG("Failed to initialize GLFW");
		return false;
	}

	// Set GLFW window hints for OpenGL 3.3 Core Profile
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on Mac

	// Additional window hints
	glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
	glfwWindowHint(GLFW_DOUBLEBUFFER, GL_TRUE);
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // Create window initially hidden
	#if 0
	glfwWindowHint(GLFW_SAMPLES, 4); // 4x MSAA
	#endif

	// Create window
	window = glfwCreateWindow(size.width, size.height, title, nullptr, nullptr);
	if (!window) {
		DEBUG("Failed to create GLFW window");
		glfwTerminate();
		return false;
	}

	#ifdef _MSC_VER
	// Set application icon from resources for both window and taskbar.
	// Taskbar uses the class big icon; set both big/small and also the class icons.
	const HINSTANCE hInst = ::GetModuleHandle(NULL);
	const HWND hwnd = glfwGetWin32Window(window);
	// Load big and small icons from the same resource (101 added via CMake create_rc_files)
	hIconBig = (HICON)::LoadImage(hInst, MAKEINTRESOURCE(101), IMAGE_ICON, GetSystemMetrics(SM_CXICON), GetSystemMetrics(SM_CYICON), 0);
	hIconSmall = (HICON)::LoadImage(hInst, MAKEINTRESOURCE(101), IMAGE_ICON, GetSystemMetrics(SM_CXSMICON), GetSystemMetrics(SM_CYSMICON), 0);
	// Set window icons (affects title bar, alt-tab)
	::SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIconBig);
	::SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIconSmall);
	// Also set the class icons so the taskbar picks it up reliably
	::SetClassLongPtr(hwnd, GCLP_HICON, (LONG_PTR)hIconBig);
	::SetClassLongPtr(hwnd, GCLP_HICONSM, (LONG_PTR)hIconSmall);
	#endif

	// Make context current
	glfwMakeContextCurrent(window);

	// Load OpenGL functions with GLAD
	if (!gladLoadGL()) {
		DEBUG("Failed to initialize GLAD");
		glfwDestroyWindow(window);
		glfwTerminate();
		return false;
	}

	// Print OpenGL info
	VERBOSE("OpenGL Vendor: %s", glGetString(GL_VENDOR));
	VERBOSE("OpenGL Renderer: %s", glGetString(GL_RENDERER));
	VERBOSE("OpenGL Version: %s", glGetString(GL_VERSION));
	VERBOSE("GLSL Version: %s", glGetString(GL_SHADING_LANGUAGE_VERSION));

	// Enable/disable VSyns
	glfwSwapInterval(0);

	// Associate Scene with the window
	glfwSetWindowUserPointer(window, &scene);

	// Set GLFW callbacks
	glfwSetFramebufferSizeCallback(window, FramebufferSizeCallback);
	glfwSetCursorPosCallback(window, MouseCallback);
	glfwSetMouseButtonCallback(window, MouseButtonCallback);
	glfwSetScrollCallback(window, ScrollCallback);
	glfwSetKeyCallback(window, KeyCallback);
	glfwSetDropCallback(window, DropCallback);

	// Try to enable OpenGL debug output for automatic error checking
	GL_ENABLE_DEBUG_OUTPUT();

	// Initialize core systems
	arcballControls = std::make_unique<ArcballControls>(camera);
	firstPersonControls = std::make_unique<FirstPersonControls>(camera);
	selectionController = std::make_unique<SelectionController>(camera);
	bboxEditController = std::make_unique<BoundingBoxEditController>(camera);
	// Route every controller-driven OBB change through Scene::SetBoundingBox
	// so GPU buffer refresh and redraw stay centralized in one place.
	bboxEditController->setChangeCallback([this](const OBB3f& obb) {
		if (GetScene().IsOpen())
			GetScene().SetBoundingBox(obb);
	});
	renderer = std::make_unique<Renderer>();
	ui = std::make_unique<UI>();

	// Initialize renderer
	if (!renderer->Initialize()) {
		DEBUG("Failed to initialize renderer");
		return false;
	}

	// Initialize UI
	if (!ui->Initialize(*this, "#version 330")) {
		DEBUG("Failed to initialize UI");
		return false;
	}

	// Update device pixel ratio for accurate mouse coordinate conversion
	UpdateDevicePixelRatio();

	// Set up selection callback to automatically classify geometry when selection is completed
	selectionController->setChangeCallback([&scene, this]() {
		if (selectionController->hasSelectionPath()) {
			// Automatically classify geometry when selection is finished
			if (!scene.GetScene().pointcloud.IsEmpty() && showPointCloud)
				selectionController->classifyPointCloud(scene.GetScene().pointcloud, camera);
			if (!scene.GetScene().mesh.IsEmpty() && showMesh)
				selectionController->classifyMesh(scene.GetScene().mesh, camera);
			RequestRedraw();
		}
	});

	// Set up delete callback to remove selected geometry
	selectionController->setDeleteCallback([&scene, this]() {
		if (scene.IsWorkflowRunning()) {
			DEBUG("Cannot remove geometry while workflow is running");
			return;
		}
		scene.RemoveSelectedGeometry();
	});

	// Set up ROI callback to set region of interest from selection
	selectionController->setROICallback([&scene, this](bool aabb) {
		if (scene.IsWorkflowRunning()) {
			DEBUG("Cannot set ROI while workflow is running");
			return;
		}
		scene.SetROIFromSelection(aabb);
	});

	// Initialize timing
	lastFrame = glfwGetTime();

	return true;
}

void Window::Release() {
	if (window) {
		// Cleanup systems (in reverse order)
		ui.reset();
		renderer.reset();
		arcballControls.reset();
		firstPersonControls.reset();
		selectionController.reset();

		// Destroy window and terminate GLFW
		glfwDestroyWindow(window);
		window = nullptr;
	}
	#ifdef _MSC_VER
	// Destroy loaded icons if any
	if (hIconBig) { ::DestroyIcon(hIconBig); hIconBig = nullptr; }
	if (hIconSmall) { ::DestroyIcon(hIconSmall); hIconSmall = nullptr; }
	#endif
	glfwTerminate();
}

void Window::ResetView() {
	camera.Reset();
	currentControlMode = CONTROL_NONE;
	SetControlMode(CONTROL_ARCBALL);
	selectedNeighborCamera = NO_ID;
	selectionType = SEL_NA;
	selectionIdx.Release();
}

void Window::Reset() {
	ResetView();
	renderer->Reset();
	SetTitle(_T("(empty)"));
}

void Window::Run() {
	// Main loop
	while (!ShouldClose()) {
		// Update timing
		const double deltaTime = UpdateTiming();

		// Check for workflow completion
		GetScene().CheckWorkflowCompletion();

		// Update active control system
		switch (currentControlMode) {
		case CONTROL_ARCBALL:
			arcballControls->update(deltaTime);
			break;
		case CONTROL_FIRST_PERSON:
			firstPersonControls->update(deltaTime);
			break;
		case CONTROL_SELECTION:
			selectionController->update(deltaTime);
			break;
		case CONTROL_BBOX_EDIT:
			bboxEditController->update(deltaTime);
			break;
		}

		#ifdef __APPLE__
		// Check for files requested to open by Finder and open first
		{
			std::vector<std::string> pending;
			OpenMVS_ConsumePendingOpenFiles(pending);
			if (!pending.empty())
				GetScene().Open(pending.front());
		}
		#endif

		// Process events
		if (renderOnlyOnChange)
			glfwWaitEvents(); // wait for events
		else
			glfwPollEvents(); // poll events normally for continuous rendering

		// Render frame
		Render();

		// Swap buffers
		glfwSwapBuffers(window);

		// Update UI frame stats
		ui->UpdateFrameStats(deltaTime);
	}
}

bool Window::ShouldClose() const {
	return window ? glfwWindowShouldClose(window) : true;
}

void Window::UploadRenderData() {
	Scene& scene = GetScene();
	if (!scene.IsOpen())
		return;
	renderer->Reset();

	// Clear the selection since geometry has changed
	selectionController->clearSelection();
	selectionType = SEL_NA;
	selectionIdx.Release();

	// Upload point cloud data if needed
	if (!scene.GetScene().pointcloud.IsEmpty()) {
		renderer->UploadPointCloud(scene.GetScene().pointcloud, pointNormalLength);
		showPointCloud = true;
	}

	// Upload mesh data if needed
	meshSubMeshVisible.clear();
	if (!scene.GetScene().mesh.IsEmpty()) {
		showMesh = true;
		if (scene.GetScene().mesh.HasTexture())
			showMeshTextured = true;
		renderer->UploadMesh(scene.GetScene().mesh);
		meshSubMeshVisible.assign(renderer->GetMeshSubMeshCount(), true);
	}

	// Upload cameras if visible, and
	// upload image overlays for cameras with valid textures
	if (!scene.GetScene().images.empty())
		renderer->UploadCameras(*this);

	// Upload bounds if available
	renderer->UploadBounds(scene.GetScene());

	// Request a redraw
	RequestRedraw();
}

void Window::Render() {
	GL_DEBUG_SCOPE("Window::Render");

	// Enable depth testing
	GL_CHECK(glEnable(GL_DEPTH_TEST));
	GL_CHECK(glDepthFunc(GL_LESS));

	// Begin frame with UI's clear color
	renderer->BeginFrame(camera, clearColor);

	// Start UI frame
	ui->NewFrame(*this);

	Scene& scene = GetScene();
	if (scene.IsOpen()) {
		// Render the scene contents
		if (showPointCloud) {
			renderer->RenderPointCloud(*this);
			if (showPointCloudNormals)
				renderer->RenderPointCloudNormals(*this);
		}
		if (showMesh)
			renderer->RenderMesh(*this);

		// Flush pending screenshot if requested (without UI)
		if (!pendingScreenshotPath.empty() && !pendingScreenshotIncludeUI) {
			CaptureScreenshot(pendingScreenshotPath);
			pendingScreenshotPath.clear();
			if (pendingScreenshotQuit)
				glfwSetWindowShouldClose(window, GLFW_TRUE);
		}

		// Render cameras and selection highlights
		if (showCameras)
			renderer->RenderCameras(*this);
		renderer->RenderSelection(*this);
		renderer->RenderSelectedGeometry(*this);

		// Render bounds if visible and available
		if (showBounds)
			renderer->RenderBounds();

		// Render the interactive bounding-box edit gizmos while edit mode is active
		if (currentControlMode == CONTROL_BBOX_EDIT && GetScene().IsOpen()) {
			const OBB3f& editOBB = bboxEditController->getOBB();
			if (editOBB.IsValid()) {
				renderer->RenderBoundingBoxGizmos(
					editOBB,
					bboxEditController->getHoverCornerIdx(),
					bboxEditController->getHoverFaceIdx(),
					bboxEditController->getHoverAxisIdx());
			}
		}

		// Render image overlay when in camera view mode
		renderer->RenderImageOverlays(*this);

		// Render 2D selection overlay (after all 3D rendering, before UI)
		renderer->RenderSelectionOverlay(*this);

		// Show scene information
		ui->ShowSceneInfo(*this);

		// Show camera controls
		ui->ShowCameraControls(*this);

		// Show selection controls
		ui->ShowSelectionControls(*this);

		// Show render settings
		ui->ShowRenderSettings(*this);

		// Show bounding-box editor
		ui->ShowBoundingBoxControls(*this);
		ui->ShowWorkflowWindows(*this);
	}

	// Show UI
	ui->ShowMainMenuBar(*this);

	// Render gizmos or coordinate axes
	if (currentControlMode == CONTROL_ARCBALL && arcballControls && arcballControls->getEnableGizmos()) {
		// Render arcball gizmos instead of coordinate axes
		renderer->RenderArcballGizmos(camera, *arcballControls);
	} else {
		// Even when no scene is loaded, render coordinate axes as a visual indicator
		renderer->RenderCoordinateAxes(camera);
	}

	// Render UI
	ui->Render(*this);

	// Flush pending screenshot if requested (with UI)
	if (!pendingScreenshotPath.empty()) {
		CaptureScreenshot(pendingScreenshotPath);
		pendingScreenshotPath.clear();
		if (pendingScreenshotQuit)
			glfwSetWindowShouldClose(window, GLFW_TRUE);
	}

	// End frame
	renderer->EndFrame();

	#ifndef OPENGL_DEBUG_ENABLE
	// Manual error check as backup (this will be redundant if debug context is enabled)
	auto [error, errorString] = OPENGL_DEBUG::GetOpenGLError();
	if (error != GL_NO_ERROR)
		DEBUG("OpenGL Error in Render(): %s", errorString.c_str());
	#endif
}

void Window::SetTitle(const String& newTitle) {
	title = newTitle;
	if (window)
		glfwSetWindowTitle(window, title.c_str());
}

void Window::SetVisible(bool visible) {
	if (window) {
		if (visible)
			glfwShowWindow(window);
		else
			glfwHideWindow(window);
	}
}

void Window::RequestAttention() {
	if (window)
		glfwRequestWindowAttention(window);
}

void Window::Focus() {
	if (window)
		glfwFocusWindow(window);
}

void Window::SetSceneBounds(const Point3f& center, const Point3f& size) {
	camera.SetSceneBounds(center, size);
	arcballControls->setSensitivity(norm(size) * 0.1);
	firstPersonControls->setMovementSpeed(norm(size) * 0.1);
}

// Request an off-screen screenshot to be saved by the renderer on the next
// frame; if quitAfter is set the window closes once the image is written
void Window::RequestScreenshot(const String& filename, bool includeUI, bool quitAfter) {
	if (filename.empty())
		return;
	pendingScreenshotPath = filename;
	pendingScreenshotIncludeUI = includeUI;
	pendingScreenshotQuit = quitAfter;
	RequestRedraw();
}

GLFWwindow* Window::GetCurrentGLFWWindow()
{
	return glfwGetCurrentContext();
}

Window& Window::GetCurrentWindow()
{
	return GetCurrentScene().GetWindow();
}

void Window::SetControlMode(ControlMode mode) {
	if (currentControlMode == mode)
		return;
	// Leaving bounding-box edit mode mid-drag: auto-commit the working OBB so
	// the next mode change doesn't drop user edits. Esc during drag should
	// revert instead - see HandleKeyboard which forwards Esc to the controller.
	if (currentControlMode == CONTROL_BBOX_EDIT && bboxEditController) {
		if (bboxEditController->isDragging())
			bboxEditController->commit();
	}
	currentControlMode = mode;
	// Reset any control state when switching modes
	switch (currentControlMode) {
	case CONTROL_FIRST_PERSON:
		firstPersonControls->reset();
		break;
	case CONTROL_ARCBALL:
		arcballControls->reset();
		break;
	case CONTROL_SELECTION:
		// Auto-open selection controls when switching to selection mode
		ui->SetSelectionControls(true);
		// Don't reset selection when switching to selection mode
		// This preserves the active selection for inspection while navigating
		break;
	case CONTROL_BBOX_EDIT:
		// Seed the controller with the current scene OBB so its hover/drag
		// math operates on the live bounding box.
		if (GetScene().IsOpen())
			bboxEditController->setOBB(GetScene().GetScene().obb);
		RequestRedraw();
		break;
	}
}

// Static GLFW Callbacks
void Window::FramebufferSizeCallback(GLFWwindow* window, int width, int height) {
	// Update device pixel ratio for accurate mouse coordinate conversion
	GetScene(window).GetWindow().UpdateDevicePixelRatio();
}

void Window::MouseCallback(GLFWwindow* window, double xpos, double ypos) {
	GetScene(window).GetWindow().HandleMouseMove(xpos, ypos);
}

void Window::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
	GetScene(window).GetWindow().HandleMouseButton(button, action, mods);
}

void Window::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
	GetScene(window).GetWindow().HandleScroll(yoffset);
}

void Window::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	GetScene(window).GetWindow().HandleKeyboard(key, action, mods);
}

void Window::DropCallback(GLFWwindow* window, int count, const char** paths) {
	GetScene(window).GetWindow().HandleFileDrop(count, paths);
}

// Input Handling Methods
void Window::HandleMouseMove(double xpos, double ypos) {
	// Skip UI if it wants to capture mouse
	if (ui->WantCaptureMouse())
		return;

	// Normalize mouse position to [-1, 1] range
	Eigen::Vector2d normalizedPos = NormalizeMousePos(xpos, ypos);

	// Pass to active control system
	switch (currentControlMode) {
	case CONTROL_ARCBALL:
		arcballControls->handleMouseMove(normalizedPos);
		break;
	case CONTROL_FIRST_PERSON:
		firstPersonControls->handleMouseMove(normalizedPos);
		break;
	case CONTROL_SELECTION:
		selectionController->handleMouseMove(normalizedPos);
		break;
	case CONTROL_BBOX_EDIT:
		bboxEditController->handleMouseMove(normalizedPos);
		RequestRedraw(); // hover highlight needs a redraw to refresh
		break;
	}

	lastMousePos = Eigen::Vector2d(xpos, ypos);
}

void Window::HandleMouseButton(int button, int action, int mods) {
	// Skip UI if it wants to capture mouse
	if (ui->WantCaptureMouse())
		return;

	// Normalize current mouse position
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	Eigen::Vector2d normalizedPos = NormalizeMousePos(xpos, ypos);

	// Pass to active control system
	switch (currentControlMode) {
	case CONTROL_ARCBALL:
		arcballControls->handleMouseButton(button, action, normalizedPos);
		break;
	case CONTROL_FIRST_PERSON:
		firstPersonControls->handleMouseButton(button, action, normalizedPos);
		break;
	case CONTROL_SELECTION:
		selectionController->handleMouseButton(button, action, normalizedPos, mods);
		break;
	case CONTROL_BBOX_EDIT:
		bboxEditController->handleMouseButton(button, action, normalizedPos, mods);
		RequestRedraw();
		break;
	}

	// Handle raycast on click
	Ray3d ray = camera.GetPickingRay(normalizedPos);
	// Convert logical window cursor coords to framebuffer pixel coords using devicePixelRatio
	Point2f screenPos(
		static_cast<float>(xpos * devicePixelRatio.x()),
		static_cast<float>(ypos * devicePixelRatio.y()));
	GetScene().OnCastRay(screenPos, ray, button, action, mods);
}

void Window::HandleScroll(double yoffset) {
	// Skip UI if it wants to capture mouse
	if (ui->WantCaptureMouse())
		return;

	// Pass to active control system
	switch (currentControlMode) {
	case CONTROL_ARCBALL:
		arcballControls->handleScroll(yoffset);
		break;
	case CONTROL_FIRST_PERSON:
		firstPersonControls->handleScroll(yoffset);
		break;
	case CONTROL_SELECTION:
		selectionController->handleScroll(yoffset);
		break;
	}
}

void Window::HandleKeyboard(int key, int action, int mods) {
	const int disallowedMods = GLFW_MOD_CONTROL | GLFW_MOD_ALT | GLFW_MOD_SUPER;
	const bool shiftOnly = (mods & GLFW_MOD_SHIFT) && !(mods & disallowedMods);
	if (action == GLFW_PRESS && shiftOnly) {
		switch (key) {
		case GLFW_KEY_A:
			ui->ToggleSceneInfo();
			RequestRedraw();
			return;
		case GLFW_KEY_Q:
			ui->ToggleCameraInfoDialog();
			RequestRedraw();
			return;
		case GLFW_KEY_C:
			ui->ToggleCameraControls();
			RequestRedraw();
			return;
		case GLFW_KEY_S:
			ui->ToggleSelectionDialog();
			RequestRedraw();
			return;
		case GLFW_KEY_R:
			ui->ToggleRenderSettings();
			RequestRedraw();
			return;
		case GLFW_KEY_B:
			ui->ToggleBoundingBoxControls();
			RequestRedraw();
			return;
		default:
			break;
		}
	}

	// Skip UI if it wants to capture keyboard
	if (ui->WantCaptureKeyboard())
		return;

	// Handle special keys first
	if (action == GLFW_RELEASE) {
		switch (key) {
			case GLFW_KEY_ESCAPE:
				if (!camera.IsCameraViewMode() && currentControlMode != CONTROL_SELECTION) {
					// Close the window
					glfwSetWindowShouldClose(window, GLFW_TRUE);
				}
				return;

			case GLFW_KEY_F11:
				// Toggle fullscreen
				{
					static bool isFullscreen = false;
					static int windowedX, windowedY, windowedWidth, windowedHeight;

					if (!isFullscreen) {
						// Save windowed position and size
						glfwGetWindowPos(window, &windowedX, &windowedY);
						glfwGetWindowSize(window, &windowedWidth, &windowedHeight);

						// Get primary monitor
						GLFWmonitor* monitor = glfwGetPrimaryMonitor();
						const GLFWvidmode* mode = glfwGetVideoMode(monitor);

						// Switch to fullscreen
						glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
					} else {
						// Switch back to windowed
						glfwSetWindowMonitor(window, nullptr, windowedX, windowedY, windowedWidth, windowedHeight, GLFW_DONT_CARE);
					}

					isFullscreen = !isFullscreen;
				}
				return;

			case GLFW_KEY_TAB:
				// Tab key to switch between control modes
				if (currentControlMode == CONTROL_ARCBALL)
					SetControlMode(CONTROL_FIRST_PERSON);
				else
					SetControlMode(CONTROL_ARCBALL);
				return;

			case GLFW_KEY_O:
				#ifdef __APPLE__
				if (mods & GLFW_MOD_SUPER) {
				#else
				if (mods & GLFW_MOD_CONTROL) {
				#endif
					// Ctrl+O - Open file
					SetVisible(false);
					String filename, geometryFilename;
					if (ui->ShowOpenFileDialog(filename, geometryFilename))
						GetScene().Open(filename, geometryFilename);
					SetVisible(true);
				}
				break;

			case GLFW_KEY_S:
				#ifdef __APPLE__
				if (mods & GLFW_MOD_SUPER) {
				#else
				if (mods & GLFW_MOD_CONTROL) {
				#endif
					if (mods & GLFW_MOD_SHIFT) {
						// Ctrl+Shift+S - Save As
						SetVisible(false);
						String filename;
						if (ui->ShowSaveFileDialog(filename))
							GetScene().Save(filename, false);
						SetVisible(true);
					} else {
						// Ctrl+S - Save
						GetScene().Save("", false);
					}
				}
				break;

			case GLFW_KEY_B:
				#ifdef __APPLE__
				if (mods & GLFW_MOD_SUPER) {
				#else
				if (mods & GLFW_MOD_CONTROL) {
				#endif
					// Ctrl+B - Estimate ROI with default parameters
					GetScene().RunEstimateROIWorkflow(GetScene().GetEstimateROIWorkflowOptions());
				} else if (mods == 0 && currentControlMode != CONTROL_SELECTION) {
					// Plain B toggles bounding-box visibility, but we reserve B
					// for "Box selection mode" while the selection controller is active.
					showBounds = !showBounds;
					RequestRedraw();
				}
				break;

			case GLFW_KEY_LEFT:
				camera.PreviousCamera();
				break;
			case GLFW_KEY_RIGHT:
				camera.NextCamera();
				break;

			// Help dialog
			case GLFW_KEY_F1:
				ui->ToggleHelpDialog();
				return;

			// Screenshot
			case GLFW_KEY_X:
				#ifdef __APPLE__
				if ((mods & GLFW_MOD_SUPER)) {
				#else
				if ((mods & GLFW_MOD_CONTROL)) {
				#endif
					String filename;
					if (ui->ShowSaveImageDialog(filename)) {
						if (Util::getFileExt(filename).empty())
							filename += ".png";
						RequestScreenshot(filename, (mods & GLFW_MOD_SHIFT) != 0);
					}
					return;
				}
				return;

			// Rendering toggles
			case GLFW_KEY_P:
				showPointCloud = !showPointCloud;
				RequestRedraw();
				return;
			case GLFW_KEY_M:
				showMesh = !showMesh;
				RequestRedraw();
				return;
			case GLFW_KEY_C:
				// Toggle camera rendering (only if not in first person mode to avoid conflict with movement)
				if (currentControlMode != CONTROL_FIRST_PERSON) {
					showCameras = !showCameras;
					RequestRedraw();
				}
				return;
			case GLFW_KEY_W:
				if (currentControlMode == CONTROL_FIRST_PERSON)
					break;
				showMeshWireframe = !showMeshWireframe;
				RequestRedraw();
				return;
			case GLFW_KEY_T:
				showMeshTextured = !showMeshTextured;
				RequestRedraw();
				return;

			// Selection mode toggle
			case GLFW_KEY_G:
				if (currentControlMode == CONTROL_SELECTION) {
					// Exit selection mode to arcball
					SetControlMode(CONTROL_ARCBALL);
				} else {
					// Enter selection mode
					SetControlMode(CONTROL_SELECTION);
				}
				return;

			// Camera reset
			case GLFW_KEY_R:
				ResetView();
				return;
		}
	}

	// Pass to active control system
	if (currentControlMode == CONTROL_ARCBALL)
		arcballControls->handleKeyboard(key, action, mods);
	else if (currentControlMode == CONTROL_FIRST_PERSON)
		firstPersonControls->handleKeyboard(key, action, mods);
	else if (currentControlMode == CONTROL_SELECTION)
		selectionController->handleKeyboard(key, action, mods);
	else if (currentControlMode == CONTROL_BBOX_EDIT)
		bboxEditController->handleKeyboard(key, action, mods);
}

void Window::HandleFileDrop(int count, const char** paths) {
	if (count > 0) {
		// Handle first dropped file
		String filename(paths[0]);
		// Check file extension to determine if it's a scene or geometry file
		String ext = Util::getFileExt(filename).ToLower();
		if (ext == ".mvs" || ext == ".sfm" || ext == ".dmap") {
			// Scene file
			String geometryFilename;
			if (count > 1) {
				// Use second dropped file as geometry file if available
				geometryFilename = String(paths[1]);
			}
			GetScene().Open(filename, geometryFilename);
		} else if (ext == ".ply" || ext == ".obj" || ext == ".off" || ext == ".gltf" || ext == ".glb") {
			// Geometry file
			GetScene().Open(filename, "");
		} else {
			DEBUG("Unsupported file format: %s", ext.c_str());
		}
	}
}

bool Window::CaptureScreenshot(const String& filename) {
	const cv::Size& size = camera.GetSize();
	if (size.empty()) {
		DEBUG("error: invalid framebuffer size for screenshot");
		return false;
	}

	Image8U4 imgRGBA(size);
	GL_CHECK(glPixelStorei(GL_PACK_ALIGNMENT, 1));
	GL_CHECK(glReadBuffer(GL_BACK));
	GL_CHECK(glReadPixels(0, 0, size.width, size.height, GL_RGBA, GL_UNSIGNED_BYTE, imgRGBA.getData()));
	Image8U3 img(size);
	cv::cvtColor(imgRGBA, img, cv::COLOR_RGBA2BGR);
	cv::flip(img, img, 0);

	if (!img.Save(filename)) {
		DEBUG("error: failed to write screenshot to '%s'", filename.c_str());
		return false;
	}
	DEBUG("Screenshot saved to '%s'", filename.c_str());
	return true;
}

double Window::UpdateTiming() {
	double currentFrame = glfwGetTime();
	double deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;
	return deltaTime;
}

void Window::UpdateDevicePixelRatio() {
	if (!window) {
		devicePixelRatio = Eigen::Vector2d(1.0, 1.0);
		return;
	}

	// Get logical window size and framebuffer size
	cv::Size windowSize, size;
	glfwGetWindowSize(window, &windowSize.width, &windowSize.height);
	glfwGetFramebufferSize(window, &size.width, &size.height);

	// Calculate device pixel ratio (scale factor)
	devicePixelRatio.x() = (windowSize.width > 0 ? static_cast<double>(size.width) / static_cast<double>(windowSize.width) : 1.0);
	devicePixelRatio.y() = (windowSize.height > 0 ? static_cast<double>(size.height) / static_cast<double>(windowSize.height) : 1.0);

	// Set initial viewport to match framebuffer size
	GL_CHECK(glViewport(0, 0, size.width, size.height));

	// Set initial camera size
	camera.SetSize(size);

	DEBUG("Framebuffer size changed: %dx%d (window size: %dx%d)",
		size.width, size.height, windowSize.width, windowSize.height);
}

Eigen::Vector2d Window::NormalizeMousePos(double x, double y) const {
	// Convert mouse coordinates from logical window coordinates to framebuffer coordinates
	double framebufferX = x * devicePixelRatio.x();
	double framebufferY = y * devicePixelRatio.y();

	// Convert from framebuffer coordinates to normalized coordinates [-1, 1]
	double normalizedX = (2.0 * framebufferX) / GetSize().width - 1.0;
	double normalizedY = 1.0 - (2.0 * framebufferY) / GetSize().height;
	return Eigen::Vector2d(normalizedX, normalizedY);
}

// Hide/show mouse cursor (does not seem to work during remote desktop sessions)
void Window::SetCursorVisible(bool visible) {
	GLFWwindow* window = GetCurrentGLFWWindow();
	if (visible)
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	else
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
}

// Static method to get the associated Scene from the window
Scene& Window::GetScene(GLFWwindow* window) {
	return *reinterpret_cast<Scene*>(glfwGetWindowUserPointer(window));
}

Scene& Window::GetCurrentScene()
{
	return GetScene(GetCurrentGLFWWindow());
}

// Static method to request a redraw by posting a GLFW event
void Window::RequestRedraw() {
	glfwPostEmptyEvent();
}
/*----------------------------------------------------------------*/
