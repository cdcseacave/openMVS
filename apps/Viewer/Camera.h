/*
 * Camera.h
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

namespace VIEWER {

/**
 * Simple camera class for 3D rendering.
 *
 * This class provides basic camera functionality for view and projection matrices,
 * camera state management, and scene viewing. The camera supports both perspective
 * and orthographic projections.
 *
 * Navigation is handled by external control classes (e.g., ArcballControls) that
 * manipulate the camera's position, target, and orientation.
 */
class Camera {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	// Camera state
	Eigen::Vector3d position;
	Eigen::Vector3d target;
	Eigen::Vector3d up;

	// Scene bounds
	Eigen::Vector3f sceneCenter;
	Eigen::Vector3f sceneSize;
	float sceneDistance; // average distance from camera to scene

	// Projection parameters
	cv::Size size; // viewport size
	double fov, nearPlane, farPlane;
	bool orthographic;

	// Camera view mode as viewer camera ID
	MVS::IIndex prevCamID, currentCamID, maxCamID;

	// Saved camera state for restoring after camera view mode
	struct CameraState {
		Eigen::Vector3d position;
		Eigen::Vector3d target;
		Eigen::Vector3d up;
		cv::Size size;
		double fov;
		bool orthographic;
	};
	std::optional<CameraState> savedState;

	// Camera view mode callback
	std::function<void(MVS::IIndex)> cameraViewModeCallback;

public:
	Camera();

	// Core functionality
	void SetSize(const cv::Size& newSize) { size = newSize; }
	void SetFOV(double fov);
	void SetNearFar(double nearPlane, double farPlane);
	void SetOrthographic(bool ortho);

	// Matrix generation
	Eigen::Matrix3d GetRotationMatrix() const;
	Eigen::Matrix4d GetViewMatrix() const;
	Eigen::Matrix4d GetProjectionMatrix() const;

	// Scene setup
	void Reset();
	void SetSceneBounds(const Point3f& center, const Point3f& size);
	void SetSceneDistance(float distance) { sceneDistance = distance; }
	void SetTarget(const Point3f& newTarget);
	void SetLookAt(const Eigen::Vector3d& eye, const Eigen::Vector3d& target, const Eigen::Vector3d& up);

	// Ray casting
	Ray3d GetPickingRay(const Eigen::Vector2d& screenPos) const;

	// Getters
	const Eigen::Vector3d& GetPosition() const { return position; }
	const Eigen::Vector3d& GetTarget() const { return target; }
	const Eigen::Vector3d& GetUp() const { return up; }
	const Eigen::Vector3f& GetSceneCenter() const { return sceneCenter; }
	const Eigen::Vector3f& GetSceneSize() const { return sceneSize; }
	float GetSceneDistance() const { return sceneDistance; }
	double GetNearPlane() const { return nearPlane; }
	double GetFarPlane() const { return farPlane; }
	const cv::Size& GetSize() const { return size; }
	double GetFOV() const { return fov; }
	bool IsOrthographic() const { return orthographic; }

	// Camera view mode functionality
	bool IsCameraViewMode() const { return currentCamID != NO_ID; }
	void SetCameraViewMode(MVS::IIndex camID);
	void SetCameraFromSceneData(const MVS::Image& imageData);
	void SetCameraFromPose(const Matrix3x4& pose);
	void SetCameraFromPose(const MVS::Image& imageData);
	void DisableCameraViewMode();
	void SaveCurrentState();
	bool RestoreSavedState();
	bool HasSavedState() const { return savedState.has_value(); }
	MVS::IIndex GetCurrentCamID() const { return currentCamID; }
	void SetCurrentCamID(MVS::IIndex camID) {
		prevCamID = currentCamID;
		currentCamID = camID;
	}
	void SetMaxCamID(MVS::IIndex maxID) { maxCamID = maxID; }
	void SetCameraViewModeCallback(std::function<void(MVS::IIndex)> callback) {
		cameraViewModeCallback = callback;
	}

	// Camera navigation
	void NextCamera();
	void PreviousCamera();

private:
	// Static helper function for computing look-at matrix
	static Eigen::Matrix4d ComputeLookAtMatrix(const Eigen::Vector3d& eye, const Eigen::Vector3d& center, const Eigen::Vector3d& up);
};
/*----------------------------------------------------------------*/

} // namespace VIEWER
