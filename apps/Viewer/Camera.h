/*
 * Camera.h
 *
 * Copyright (c) 2014-2015 SEACAVE
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

#ifndef _VIEWER_CAMERA_H_
#define _VIEWER_CAMERA_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace VIEWER {

class Camera
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cv::Size size;
	AABB3d boxScene;
	Eigen::Vector3d centerScene;
	Eigen::Quaterniond rotation;
	Eigen::Vector3d center;
	double dist, radius;
	float fov, fovDef;
	float scaleF, scaleFDef;
	MVS::IIndex prevCamID, currentCamID, maxCamID;

public:
	Camera(const AABB3d& _box=AABB3d(true), const Point3d& _center=Point3d::ZERO, float _scaleF=1, float _fov=40);

	void Reset();
	void Resize(const cv::Size&);
	void SetFOV(float _fov);

	const cv::Size& GetSize() const { return size; }

	Eigen::Vector3d GetPosition() const;
	Eigen::Matrix3d GetRotation() const;
	Eigen::Matrix4d GetLookAt() const;

	void GetLookAt(Eigen::Vector3d& eye, Eigen::Vector3d& center, Eigen::Vector3d& up) const;
	void Rotate(const Eigen::Vector2d& pos, const Eigen::Vector2d& prevPos);
	void Translate(const Eigen::Vector2d& pos, const Eigen::Vector2d& prevPos);

	bool IsCameraViewMode() const { return prevCamID != currentCamID && currentCamID != NO_ID; }

protected:
	void ProjectOnSphere(double radius, Eigen::Vector3d& p) const;
};
/*----------------------------------------------------------------*/

} // namespace VIEWER

#endif // _VIEWER_CAMERA_H_
