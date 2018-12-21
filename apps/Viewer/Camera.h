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

	AABB3d box;
	int width, height;
	Eigen::Quaterniond rotation;
	Eigen::Vector3d center;
	double dist;
	double radius;
	double fov;
	float scaleF;
	MVS::IIndex prevCamID, currentCamID, maxCamID;

public:
	explicit Camera(const AABB3d& _box=AABB3d(true), double _fov=40);
	void CopyOf(const Camera&);

	void Init(const AABB3d&);
	void Reset();
	void Resize(int _width, int _height);
	void SetFOV(double _fov);

	Eigen::Vector3d GetPosition() const;
	Eigen::Matrix4d GetLookAt() const;
	void GetLookAt(Eigen::Vector3d& eye, Eigen::Vector3d& center, Eigen::Vector3d& up) const;
	void Rotate(const Eigen::Vector2d& pos, const Eigen::Vector2d& prevPos);

protected:
	void Project2Sphere(double radius, Eigen::Vector3d& p) const;
};
typedef CSharedPtr<Camera> CameraPtr;
/*----------------------------------------------------------------*/

} // namespace VIEWER

#endif // _VIEWER_CAMERA_H_
