/*
 * SelectionController.h
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

namespace VIEWER {

/**
 * SelectionController class implementing geometry selection functionality.
 * 
 * This class provides interactive 2D selection tools for selecting areas of 
 * geometry (point clouds and meshes) and performing operations on selected regions.
 * It operates independently of existing ray-cast selection functionality.
 * 
 * Key Features:
 * - Box selection: Rectangular region selection
 * - Lasso selection: Free-form polygon selection  
 * - Circle selection: Circular region selection
 * - Additive selection: Build complex selections from multiple areas
 * - Geometry operations: Remove selected or unselected geometry
 * - Visual feedback: Real-time preview of selections
 */
class SelectionController {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Selection modes
	enum SelectionMode {
		MODE_BOX,     // Rectangular selection
		MODE_LASSO,   // Free-form polygon
		MODE_CIRCLE   // Circular selection
	};

	// Selection operation types
	enum SelectionOperation {
		OP_REPLACE,    // Replace existing selection (default)
		OP_ADD,        // Add to existing selection (Shift)
		OP_SUBTRACT    // Subtract from existing selection (Ctrl)
	};

	// Selection state
	enum SelectionState {
		STATE_IDLE,       // Not selecting
		STATE_SELECTING,  // Currently drawing selection
		STATE_SELECTED    // Selection complete, ready for operations
	};

	// Constructor
	SelectionController(Camera& cam);
	~SelectionController();

	void reset();

	// Input handling - same interface as other controllers
	void handleMouseButton(int button, int action, const Eigen::Vector2d& pos, int mods = 0);
	void handleMouseMove(const Eigen::Vector2d& pos);
	void handleKeyboard(int key, int action, int mods);
	void handleScroll(double yOffset);

	// Update
	void update(double deltaTime);

	// Selection mode control
	void setSelectionMode(SelectionMode mode);
	SelectionMode getSelectionMode() const { return currentMode; }
	void setROIfromSelectionMode(bool aabb = false) { modeROIfromSelection = aabb; }
	bool isROIfromSelectionMode() const { return modeROIfromSelection; }

	// Selection state queries
	bool hasSelection() const;
	bool hasSelectionPath() const { return !selectionPath.empty(); }
	bool isSelecting() const { return currentState == STATE_SELECTING; }
	SelectionState getSelectionState() const { return currentState; }

	// Selection operations
	void clearSelection();
	void invertSelection();
	void finishCurrentSelection();

	// Programmatic selection control
	// Replace or augment the current point selection with the provided indices
	void setSelectedPoints(const MVS::PointCloud::IndexArr& indices, size_t totalPointCount, bool replace = true);

	// Geometry classification - called by Scene to classify points/faces
	// points hidden by the confidence filter are never selected
	void classifyPointCloud(const MVS::PointCloud& pointcloud, const Camera& camera, const PointConfidenceFilter& confFilter);
	void classifyMesh(const MVS::Mesh& mesh, const Camera& camera);

	// Selection results access
	const std::vector<bool>& getPointsSelected() const { return pointsSelected; }
	const std::vector<bool>& getFacesSelected() const { return facesSelected; }
	MVS::PointCloud::IndexArr getSelectedPointIndices() const;
	MVS::Mesh::FaceIdxArr getSelectedFaceIndices() const;

	// Selection geometry access (for rendering)
	const std::vector<Eigen::Vector2d>& getCurrentSelectionPath() const { return selectionPath; }
	Eigen::Vector2d getSelectionStart() const { return selectionStart; }
	Eigen::Vector2d getSelectionEnd() const { return selectionEnd; }
	float getCircleRadius() const { return circleRadius; }
	MVS::IIndex getCurrentCameraIdxForHighlight() const { return currentCameraIdxForHighlight; }
	void setCurrentCameraIdxForHighlight(MVS::IIndex idx) { currentCameraIdxForHighlight = idx; }

	// Statistics
	size_t getSelectedPointCount() const;
	size_t getSelectedFaceCount() const;

	// Callbacks
	void setChangeCallback(std::function<void()> callback) { changeCallback = callback; }
	void runChangeCallback() { if (changeCallback) changeCallback(); }
	void setDeleteCallback(std::function<void()> callback) { deleteCallback = callback; }
	void runDeleteCallback() { if (deleteCallback) deleteCallback(); }
	void setROICallback(std::function<void(bool)> callback) { roiCallback = callback; }
	void runROICallback() { if (roiCallback) roiCallback(modeROIfromSelection); }

private:
	// Camera reference
	Camera& camera;

	// Current selection mode and state
	SelectionMode currentMode;
	SelectionState currentState;

	// Current selection geometry (2D screen space)
	std::vector<Eigen::Vector2d> selectionPath;  // For lasso/box
	Eigen::Vector2d selectionStart, selectionEnd; // For box/circle
	float circleRadius; // For circle mode
	MVS::IIndex currentCameraIdxForHighlight; // cache last camera used to highlight seen points

	// Geometry classification results
	std::vector<bool> pointsSelected;   // Which points are selected
	std::vector<bool> facesSelected;    // Which faces are selected

	// Pending selection operation mode (for handling modifiers)
	bool pendingSelectionIsAdditive;
	bool pendingSelectionIsSubtractive;
	bool modeROIfromSelection;

	// Callback
	std::function<void()> changeCallback;
	std::function<void()> deleteCallback;
	std::function<void(bool)> roiCallback;

private:
	// Internal methods
	void startSelection(const Eigen::Vector2d& pos);
	void updateSelection(const Eigen::Vector2d& pos);
	void finishSelection(const Eigen::Vector2d& pos);

	// Method for specific selection areas
	bool isPointInSelection(const Point3f& worldPoint, 
	                       const std::vector<Eigen::Vector2d>& selectionPath,
	                       SelectionMode selectionMode,
	                       const Camera& camera) const;

	// 2D geometric tests
	bool isPointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) const;
	bool isPointInCircle(const Eigen::Vector2d& point, const Eigen::Vector2d& center, float radius, float aspectRatio = 1) const;
	bool isPointInBox(const Eigen::Vector2d& point, const Eigen::Vector2d& min, const Eigen::Vector2d& max) const;

	// Utility
	Eigen::Vector2d worldToScreen(const Point3f& worldPoint, const Camera& camera) const;

	// Helper method to generate circle vertices for rendering
	void generateCircleVertices(const Eigen::Vector2d& center, float radius);
};
/*----------------------------------------------------------------*/

} // namespace VIEWER
