/*
 * Window.h
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

#include "Camera.h"
#include "ArcballControls.h"
#include "FirstPersonControls.h"
#include "SelectionController.h"
#include "BoundingBoxEdit.h"
#include "Renderer.h"
#include "UI.h"

namespace VIEWER {

// Forward declarations
class Scene;

class Window {
public:
	enum ControlMode {
		CONTROL_ARCBALL,
		CONTROL_FIRST_PERSON,
		CONTROL_SELECTION,
		CONTROL_BBOX_EDIT,
		CONTROL_NONE
	};
	enum CameraDisplayColor {
		CAMERA_COLOR_SOLID = 0,
		CAMERA_COLOR_JET
	};
	enum CameraDisplayType {
		CAMERA_DISPLAY_FRUSTUM = 0,
		CAMERA_DISPLAY_DOT
	};

private:
	GLFWwindow* window;
	String title;

	#ifdef _MSC_VER
	// Cached Windows icon handles
	HICON hIconBig;
	HICON hIconSmall;
	#endif

	// Device pixel ratio for Retina/high-DPI displays
	Eigen::Vector2d devicePixelRatio;

	// Core systems
	Camera camera;
	std::unique_ptr<ArcballControls> arcballControls;
	std::unique_ptr<FirstPersonControls> firstPersonControls;
	std::unique_ptr<SelectionController> selectionController;
	std::unique_ptr<BoundingBoxEditController> bboxEditController;
	std::unique_ptr<Renderer> renderer;
	std::unique_ptr<UI> ui;

	// Control mode
	ControlMode currentControlMode;

	// Input state
	Eigen::Vector2d lastMousePos;

	// Timing
	double lastFrame;

public:
	// Selection state
	enum SELECTION {
		SEL_NA = 0,
		SEL_POINT,
		SEL_TRIANGLE,
		SEL_CAMERA
	};
	SELECTION selectionType;
	Point3f selectionPoints[4];
	double selectionTimeClick, selectionTime;
	IDXArr selectionIdx; // indices of selected point/triangle/camera (empty if none) (if camera, the indices are in the Viewer scene images)
	MVS::IIndex selectedNeighborCamera; // index of neighbor camera to highlight (NO_ID if none) (index is in the Viewer scene images)

	// Settings
	Eigen::Vector4f clearColor;
	MVS::IIndex minViews;
	float userFontScale; // UI font scale
	float cameraSize;
	CameraDisplayColor cameraDisplayColor;
	CameraDisplayType cameraDisplayType;
	bool showCameraLookAt;
	float pointSize;
	float pointNormalLength;
	float imageOverlayOpacity;
	bool renderOnlyOnChange;
	bool showCameras;
	bool showPointCloud;
	bool showPointCloudNormals;
	bool showMesh;
	bool showMeshWireframe;
	bool showMeshTextured;
	bool showBounds; // draw the scene oriented bounding-box wireframe
	std::vector<bool> meshSubMeshVisible; // control visibility of individual sub-meshes (using unsigned char instead of bool for ImGui compatibility)
	String pendingScreenshotPath;
	bool pendingScreenshotIncludeUI;
	bool pendingScreenshotQuit; // close the window once the pending screenshot has been saved

public:
	Window();
	~Window();

	bool Initialize(const cv::Size& size, const String& title, Scene& scene);
	void Release();
	void ResetView();
	void Reset();
	inline bool IsValid() const { return window != NULL; }

	// Main loop
	void Run();
	bool ShouldClose() const;

	// Rendering
	void UploadRenderData();
	void Render();

	// Camera access
	Camera& GetCamera() { return camera; }
	const Camera& GetCamera() const { return camera; }

	// Control access
	void SetControlMode(ControlMode mode);
	ControlMode GetControlMode() const { return currentControlMode; }
	ArcballControls& GetArcballControls() const { return *arcballControls; }
	FirstPersonControls& GetFirstPersonControls() { return *firstPersonControls; }
	SelectionController& GetSelectionController() const { return *selectionController; }
	BoundingBoxEditController& GetBBoxEditController() const { return *bboxEditController; }

	// Selection helpers
	bool HasSelectionIds() const { return !selectionIdx.empty(); }
	size_t GetSelectionCount() const { return selectionIdx.size(); }
	IDX GetSelectionId(size_t index = 0) const { return index < selectionIdx.size() ? selectionIdx[index] : IDX(NO_IDX); }
	const IDXArr& GetSelectionIds() const { return selectionIdx; }
	void ClearSelectionIds() { selectionIdx.clear(); }
	void SetSelectionId(IDX idx) {
		if (idx == IDX(NO_IDX) || idx == IDX(NO_ID))
			selectionIdx.clear();
		else
			selectionIdx.assign(1, idx);
	}
	void SetSelectionIds(IDXArr& indices) { selectionIdx = std::move(indices); }

	// Renderer access
	Renderer& GetRenderer() { return *renderer; }
	const Renderer& GetRenderer() const { return *renderer; }

	// UI access
	UI& GetUI() { return *ui; }
	const UI& GetUI() const { return *ui; }

	// Utility
	void SetTitle(const String& title);
	void SetVisible(bool visible);
	void RequestAttention(); // request window attention (flash in taskbar)
	void Focus(); // bring window to front and give it focus
	const Eigen::Vector2d& GetDevicePixelRatio() const { return devicePixelRatio; }
	const cv::Size& GetSize() const { return camera.GetSize(); }
	void SetSceneBounds(const Point3f& center, const Point3f& size);
	void RequestScreenshot(const String& filename, bool includeUI = false, bool quitAfter = false);
	GLFWwindow* GetGLFWWindow() const { return window; }
	static GLFWwindow* GetCurrentGLFWWindow();
	static Window& GetCurrentWindow();
	Scene& GetScene() const { return GetScene(window); }
	static Scene& GetScene(GLFWwindow* window);
	static Scene& GetCurrentScene();
	static void RequestRedraw(); // post an event to trigger redraw

	// Cursor visibility helpers
	static void SetCursorVisible(bool visible);

private:
	// GLFW callbacks
	static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
	static void MouseCallback(GLFWwindow* window, double xpos, double ypos);
	static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
	static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void DropCallback(GLFWwindow* window, int count, const char** paths);

	void HandleMouseMove(double xpos, double ypos);
	void HandleMouseButton(int button, int action, int mods);
	void HandleScroll(double yoffset);
	void HandleKeyboard(int key, int action, int mods);
	void HandleFileDrop(int count, const char** paths);
	bool CaptureScreenshot(const String& filename);

	double UpdateTiming();
	void UpdateDevicePixelRatio();
	Eigen::Vector2d NormalizeMousePos(double x, double y) const;
};
/*----------------------------------------------------------------*/

} // namespace VIEWER
