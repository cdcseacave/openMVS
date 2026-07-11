/*
 * UI.h
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
#include "Texture.h"

namespace VIEWER {

// Forward declarations
class Scene;
class Window;

class UI {
private:
	String iniPath;

	bool showSceneInfo;
	bool showCameraControls;
	bool showSelectionControls;
	bool showRenderSettings;
	bool showBoundingBoxControls;
	bool showConsoleOverlay;
	bool showPerformanceOverlay;
	bool showWorkflowOverlay;
	bool showViewportOverlay;
	bool showSelectionOverlay;
	bool showAboutDialog;
	bool showHelpDialog;
	bool showExportDialog;
	bool showCameraInfoDialog;
	bool showSelectionDialog;
	bool showSavePromptDialog;
	bool useTrackBasedNeighbors;
	bool showEstimateROIWorkflow;
	bool showDensifyWorkflow;
	bool showReconstructWorkflow;
	bool showRefineWorkflow;
	bool showTextureWorkflow;
	bool showBatchWorkflow;

	// Auto-hiding menu state
	bool showMainMenu;
	bool menuWasVisible;
	float menuTriggerHeight;
	double lastMenuInteraction;
	float menuFadeOutDelay;

	// Embedded resources
	Texture emptySceneIcon;

	// Log console
	std::deque<String> logBuffer;
	std::mutex logMutex;

	// Statistics
	double deltaTime;
	uint32_t frameCount;
	float fps;

public:
	UI();
	~UI();

	bool Initialize(Window& window, const String& glslVersion = "#version 330");
	void Release();

	void NewFrame(Window& window);
	void Render(Window& window);

	// Main UI panels
	void ShowMainMenuBar(Window& window);
	void ShowSceneInfo(const Window& window);
	void ShowCameraControls(Window& window);
	void ShowSelectionControls(Window& window);
	void ShowRenderSettings(Window& window);
	void ShowBoundingBoxControls(Window& window);
	void ShowConsoleOverlay(Window& window);
	void ShowPerformanceOverlay(Window& window);
	void ShowWorkflowOverlay(Window& window);
	void ShowViewportOverlay(const Window& window);
	void ShowSelectionOverlay(const Window& window);
	void ShowEmptySceneOverlay(const Window& window);
	void ShowWorkflowWindows(Window& window);
	void ToggleHelpDialog() { showHelpDialog = !showHelpDialog; }
	void ToggleSceneInfo() { showSceneInfo = !showSceneInfo; }
	void ToggleCameraInfoDialog() { showCameraInfoDialog = !showCameraInfoDialog; }
	void ToggleCameraControls() { showCameraControls = !showCameraControls; }
	void ToggleSelectionDialog() { showSelectionDialog = !showSelectionDialog; }
	void ToggleRenderSettings() { showRenderSettings = !showRenderSettings; }
	void ToggleBoundingBoxControls() { showBoundingBoxControls = !showBoundingBoxControls; }
	void SetSelectionControls(bool v) { showSelectionControls = v; }
	void SetUserFontScale(float scale);

	// Dialogs
	void ShowAboutDialog();
	void ShowHelpDialog();
	void ShowExportDialog(Scene& scene);
	void ShowCameraInfoDialog(Window& window);
	void ShowSelectionDialog(Window& window);
	void ShowSavePromptDialog(Window& window);
	void PromptOpenPoseQualityReport(Window& window); // prompt for and load a pose-quality CSV onto the open scene
	static bool ShowOpenFileDialog(String& filename, String& geometryFilename);
	static bool ShowSaveFileDialog(String& filename);
	static bool ShowSaveImageDialog(String& filename);
	static bool ShowOpenPoseQualityDialog(String& filename);

	// Input handling
	void RecordLog(const String& msg);
	bool WantCaptureMouse() const;
	bool WantCaptureKeyboard() const;
	void HandleGlobalKeys(Window& window);

	void UpdateFrameStats(double deltaTime);

private:
	void SetupStyle();
	void SetupCustomSettings(Window& window);
	void ShowRenderingControls(Window& window);
	void ShowPointCloudControls(Window& window);
	void ShowMeshControls(Window& window);
	void ShowEstimateROIWorkflowWindow(Window& window);
	void ShowDensifyWorkflowWindow(Window& window);
	void ShowReconstructWorkflowWindow(Window& window);
	void ShowRefineWorkflowWindow(Window& window);
	void ShowTextureWorkflowWindow(Window& window);
	void ShowBatchWorkflowWindow(Window& window);

	// Auto-hiding menu helpers
	void UpdateMenuVisibility();
	bool IsMouseNearMenuArea() const;
	bool IsMenuInUse() const;

	String FormatFileSize(size_t bytes);
	String FormatDuration(double seconds);
};
/*----------------------------------------------------------------*/

} // namespace VIEWER
