/*
 * SelectionController.cpp
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
#include "SelectionController.h"
#include "Window.h"

using namespace VIEWER;

SelectionController::SelectionController(Camera& cam)
	: camera(cam)
	, currentMode(MODE_BOX)
	, currentState(STATE_IDLE)
	, selectionStart(0, 0)
	, selectionEnd(0, 0)
	, circleRadius(0.f)
	, currentCameraIdxForHighlight(NO_ID)
	, pendingSelectionIsAdditive(false)
	, pendingSelectionIsSubtractive(false)
	, modeROIfromSelection(false)
	, changeCallback(NULL)
	, deleteCallback(NULL)
	, roiCallback(NULL)
{
	reset();
}

SelectionController::~SelectionController() {
}

void SelectionController::reset() {
	currentState = STATE_IDLE;
	clearSelection();
}

void SelectionController::handleMouseButton(int button, int action, const Eigen::Vector2d& pos, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (action == GLFW_PRESS) {
			// Determine selection operation mode based on modifiers
			bool isAdditive = (mods & GLFW_MOD_SHIFT) != 0;
			bool isSubtractive = (mods & GLFW_MOD_CONTROL) != 0;

			// If we're starting a new selection
			if (currentState == STATE_IDLE || currentState == STATE_SELECTED) {
				// Clear existing selection if no modifiers are pressed
				if (!isAdditive && !isSubtractive && currentState == STATE_SELECTED) {
					clearSelection();
				}

				// Store the selection operation mode for when we finish
				pendingSelectionIsAdditive = isAdditive;
				pendingSelectionIsSubtractive = isSubtractive;

				startSelection(pos);
			}
		} else if (action == GLFW_RELEASE && currentState == STATE_SELECTING) {
			finishSelection(pos);
		}
	}
}

void SelectionController::handleMouseMove(const Eigen::Vector2d& pos) {
	if (currentState == STATE_SELECTING) {
		updateSelection(pos);
	}
}

void SelectionController::handleKeyboard(int key, int action, int mods) {
	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (key) {
		case GLFW_KEY_B:
			setSelectionMode(MODE_BOX);
			break;
		case GLFW_KEY_L:
			setSelectionMode(MODE_LASSO);
			break;
		case GLFW_KEY_C:
			setSelectionMode(MODE_CIRCLE);
			break;
		case GLFW_KEY_I:
			invertSelection();
			break;
		case GLFW_KEY_O:
			runROICallback();
			break;
		case GLFW_KEY_ESCAPE:
			clearSelection();
			currentState = STATE_IDLE;
			break;
		case GLFW_KEY_DELETE:
			runDeleteCallback();
			break;
		}
	}
}

void SelectionController::handleScroll(double yOffset) {
	// Not used for selection
}

void SelectionController::update(double deltaTime) {
	// No continuous updates needed for selection controller
}

void SelectionController::setSelectionMode(SelectionMode mode) {
	if (currentMode != mode) {
		currentMode = mode;
		// If we're currently selecting, restart with new mode
		if (currentState == STATE_SELECTING) {
			currentState = STATE_IDLE;
		}
	}
}

bool SelectionController::hasSelection() const {
	// Check if we have any selected points or faces
	for (bool selected : pointsSelected) {
		if (selected) return true;
	}
	for (bool selected : facesSelected) {
		if (selected) return true;
	}
	return false;
}

void SelectionController::clearSelection() {
	selectionPath.clear();
	pointsSelected.clear();
	facesSelected.clear();
	selectionStart = Eigen::Vector2d(0, 0);
	selectionEnd = Eigen::Vector2d(0, 0);
	circleRadius = 0.f;
	currentCameraIdxForHighlight = NO_ID;

	runChangeCallback();
}

void SelectionController::invertSelection() {
	// Invert all point selections
	for (size_t i = 0; i < pointsSelected.size(); ++i)
		pointsSelected[i] = !pointsSelected[i];
	// Invert all face selections
	for (size_t i = 0; i < facesSelected.size(); ++i)
		facesSelected[i] = !facesSelected[i];
	Window::RequestRedraw();
}

void SelectionController::finishCurrentSelection() {
	if (currentState == STATE_SELECTING && !selectionPath.empty()) {
		// Just change state - geometry classification happens via changeCallback
		currentState = STATE_SELECTED;

		runChangeCallback();
	}
}

void SelectionController::setSelectedPoints(const MVS::PointCloud::IndexArr& indices, size_t totalPointCount, bool replace) {
	// Ensure the selection buffer matches the total number of points
	if (pointsSelected.size() != totalPointCount)
		pointsSelected.assign(totalPointCount, false);

	// Mark provided indices as selected (ignore out-of-range safely)
	if (replace)
		std::fill(pointsSelected.begin(), pointsSelected.end(), false);
	for (const auto idx : indices)
		if (idx < pointsSelected.size())
			pointsSelected[idx] = true;

	// Mark state as having a selection
	currentState = STATE_SELECTED;

	// Notify listeners and request redraw
	selectionPath.clear();
	runChangeCallback();
	Window::RequestRedraw();
}

void SelectionController::startSelection(const Eigen::Vector2d& pos) {
	selectionStart = pos;
	selectionEnd = pos;
	selectionPath.clear();

	if (currentMode == MODE_LASSO) {
		selectionPath.push_back(pos);
	}

	currentState = STATE_SELECTING;
}

void SelectionController::updateSelection(const Eigen::Vector2d& pos) {
	selectionEnd = pos;

	switch (currentMode) {
	case MODE_BOX:
		// For box, we just track start and end points
		break;

	case MODE_LASSO:
		// Add point to the path
		selectionPath.push_back(pos);
		break;

	case MODE_CIRCLE:
		// Calculate radius from start to current position
		circleRadius = static_cast<float>((pos - selectionStart).norm());
		// Generate circle vertices for rendering
		generateCircleVertices(selectionStart, circleRadius);
		break;
	}
}

void SelectionController::finishSelection(const Eigen::Vector2d& pos) {
	updateSelection(pos);

	// Create the final selection path based on mode
	switch (currentMode) {
	case MODE_BOX:
		// Create rectangle path from start/end points
		selectionPath.push_back(selectionStart);
		selectionPath.push_back(selectionEnd);
		break;

	case MODE_CIRCLE:
		// Generate circle vertices for rendering (already done in updateSelection)
		break;

	case MODE_LASSO:
		// selectionPath is already populated from mouse moves
		break;
	}

	currentState = STATE_SELECTED;

	// Classification will happen in the changeCallback
	runChangeCallback();
	selectionPath.clear();

	// Reset pending operation flags
	pendingSelectionIsAdditive = false;
	pendingSelectionIsSubtractive = false;
}

MVS::PointCloud::IndexArr SelectionController::getSelectedPointIndices() const {
	MVS::PointCloud::IndexArr indices;
	for (size_t i = 0; i < pointsSelected.size(); ++i) {
		if (pointsSelected[i]) {
			indices.push_back(static_cast<MVS::PointCloud::Index>(i));
		}
	}
	return indices;
}

MVS::Mesh::FaceIdxArr SelectionController::getSelectedFaceIndices() const {
	MVS::Mesh::FaceIdxArr indices;
	for (size_t i = 0; i < facesSelected.size(); ++i) {
		if (facesSelected[i]) {
			indices.push_back(static_cast<MVS::Mesh::FIndex>(i));
		}
	}
	return indices;
}

size_t SelectionController::getSelectedPointCount() const {
	size_t count = 0;
	for (bool selected : pointsSelected) {
		if (selected) count++;
	}
	return count;
}

size_t SelectionController::getSelectedFaceCount() const {
	size_t count = 0;
	for (bool selected : facesSelected) {
		if (selected) count++;
	}
	return count;
}

Eigen::Vector2d SelectionController::worldToScreen(const Point3f& worldPoint, const Camera& camera) const {
	// Get the view and projection matrices
	Eigen::Matrix4d view = camera.GetViewMatrix();
	Eigen::Matrix4d proj = camera.GetProjectionMatrix();

	// Transform to clip space
	Eigen::Vector4d clipPos = proj * view * Eigen::Vector3d(Cast<double>(worldPoint)).homogeneous();

	// Perspective divide
	if (ABS(clipPos.w()) < 1e-6)
		return Eigen::Vector2d(-2, -2); // Point behind camera (outside NDC range)

	// Return NDC coordinates directly (matching selection coordinate space)
	Eigen::Vector3d ndc = clipPos.hnormalized();
	return ndc.head<2>();
}

bool SelectionController::isPointInBox(const Eigen::Vector2d& point, const Eigen::Vector2d& min, const Eigen::Vector2d& max) const {
	return point.x() >= min.x() && point.x() <= max.x() && 
	       point.y() >= min.y() && point.y() <= max.y();
}

bool SelectionController::isPointInCircle(const Eigen::Vector2d& point, const Eigen::Vector2d& center, float radius, float aspectRatio) const {
	Eigen::Vector2d diff = point - center;
	// Scale the X difference by the aspect ratio to create a circular selection
	diff.x() *= aspectRatio;
	return diff.norm() <= radius;
}

bool SelectionController::isPointInPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) const {
	if (polygon.size() < 3) return false;

	bool inside = false;
	size_t j = polygon.size() - 1;

	for (size_t i = 0; i < polygon.size(); ++i) {
		if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
		    (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + polygon[i].x())) {
			inside = !inside;
		}
		j = i;
	}

	return inside;
}

// Helper method to check if a point is in a specific selection area
bool SelectionController::isPointInSelection(const Point3f& worldPoint, 
                                           const std::vector<Eigen::Vector2d>& selectionPath,
                                           SelectionMode selectionMode,
                                           const Camera& camera) const {
	// Convert 3D point to NDC coordinates
	Eigen::Vector2d ndcPoint = worldToScreen(worldPoint, camera);

	// Check if point is visible (not behind camera and within NDC range)
	if (ndcPoint.x() < -1.0 || ndcPoint.x() > 1.0 || 
		ndcPoint.y() < -1.0 || ndcPoint.y() > 1.0) {
		return false;
	}

	// Check based on selection mode
	switch (selectionMode) {
		case MODE_BOX: {
			ASSERT(selectionPath.size() == 2);
			Eigen::Vector2d min = selectionPath[0];
			Eigen::Vector2d max = selectionPath[1];
			// Ensure min/max order
			if (min.x() > max.x()) std::swap(min.x(), max.x());
			if (min.y() > max.y()) std::swap(min.y(), max.y());
			return isPointInBox(ndcPoint, min, max);
		}
		case MODE_CIRCLE: {
			ASSERT(selectionPath.size() > 2);
			// Apply aspect ratio correction for circular selection
			float aspectRatio = static_cast<float>(camera.GetSize().width) / static_cast<float>(camera.GetSize().height);
			return isPointInCircle(ndcPoint, selectionStart, circleRadius, aspectRatio);
		}
		case MODE_LASSO: {
			if (selectionPath.size() < 3)
				break;
			return isPointInPolygon(ndcPoint, selectionPath);
		}
	}

	return false;
}

void SelectionController::classifyPointCloud(const MVS::PointCloud& pointcloud, const Camera& camera, const PointConfidenceFilter& confFilter) {
	// If no current selection path, nothing to classify
	if (selectionPath.empty())
		return;

	// Initialize selection buffer if needed
	if (pointsSelected.size() != pointcloud.points.size())
		pointsSelected.resize(pointcloud.points.size(), false);

	// Create temporary buffer for this selection
	std::vector<bool> currentSelection;
	currentSelection.reserve(pointcloud.points.size());

	// Classify points in current selection
	FOREACH(i, pointcloud.points)
		currentSelection.push_back(
			confFilter.IsShown(pointcloud, i) &&
			isPointInSelection(pointcloud.points[i], selectionPath, currentMode, camera));

	// Apply the operation based on modifier keys
	SelectionOperation operation = OP_REPLACE;
	if (pendingSelectionIsAdditive) {
		operation = OP_ADD;
	} else if (pendingSelectionIsSubtractive) {
		operation = OP_SUBTRACT;
	}

	// Update the final selection based on operation
	for (size_t i = 0; i < pointcloud.points.size(); ++i) {
		if (currentSelection[i]) {
			if (operation == OP_REPLACE || operation == OP_ADD) {
				pointsSelected[i] = true;
			} else if (operation == OP_SUBTRACT) {
				pointsSelected[i] = false;
			}
		} else if (operation == OP_REPLACE) {
			// For replace operation, clear points not in selection
			pointsSelected[i] = false;
		}
	}
}

void SelectionController::classifyMesh(const MVS::Mesh& mesh, const Camera& camera) {
	// If no current selection path, nothing to classify
	if (selectionPath.empty())
		return;

	// Initialize selection buffer if needed
	if (facesSelected.size() != mesh.faces.size())
		facesSelected.resize(mesh.faces.size(), false);

	// Cache vertex selection results for efficiency
	std::vector<bool> vertexInSelection(mesh.vertices.size());
	for (size_t i = 0; i < mesh.vertices.size(); ++i)
		vertexInSelection[i] = isPointInSelection(mesh.vertices[i], selectionPath, currentMode, camera);

	// Create temporary buffer for this selection
	std::vector<bool> currentSelection;
	currentSelection.reserve(mesh.faces.size());

	// Classify faces in current selection - a face is selected if any of its vertices are in selection
	for (const auto& face : mesh.faces) {
		bool faceInSelection = vertexInSelection[face[0]] || vertexInSelection[face[1]] || vertexInSelection[face[2]];
		currentSelection.push_back(faceInSelection);
	}

	// Apply the operation based on modifier keys
	SelectionOperation operation = OP_REPLACE;
	if (pendingSelectionIsAdditive) {
		operation = OP_ADD;
	} else if (pendingSelectionIsSubtractive) {
		operation = OP_SUBTRACT;
	}

	// Update the final selection based on operation
	for (size_t i = 0; i < mesh.faces.size(); ++i) {
		if (currentSelection[i]) {
			if (operation == OP_REPLACE || operation == OP_ADD) {
				facesSelected[i] = true;
			} else if (operation == OP_SUBTRACT) {
				facesSelected[i] = false;
			}
		} else if (operation == OP_REPLACE) {
			// For replace operation, clear faces not in selection
			facesSelected[i] = false;
		}
	}
}

void SelectionController::generateCircleVertices(const Eigen::Vector2d& center, float radius) {
	// Clear previous circle vertices
	selectionPath.clear();

	// Skip if radius is too small
	if (radius < 0.001f)
		return;

	// Calculate aspect ratio to make circle appear round
	const double aspectRatio = static_cast<double>(camera.GetSize().width) / static_cast<double>(camera.GetSize().height);

	// Generate circle vertices
	const int numSegments = 64; // Number of segments for the circle
	selectionPath.reserve(numSegments + 1);

	for (int i = 0; i <= numSegments; ++i) {
		double angle = TWO_PI * i / numSegments;
		// Adjust X coordinate by aspect ratio to maintain circular shape
		double x = center.x() + (radius / aspectRatio) * COS(angle);
		double y = center.y() + radius * SIN(angle);
		selectionPath.push_back(Eigen::Vector2d(x, y));
	}
}
/*----------------------------------------------------------------*/
