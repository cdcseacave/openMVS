/*
* Camera.h
*
* Copyright (c) 2014-2024 SEACAVE
*
* Author(s):
*
*	  cDc <cdc.seacave@gmail.com>
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
*	  You are required to preserve legal notices and author attributions in
*	  that material or in the Appropriate Legal Notices displayed by works
*	  containing it.
*/

#ifndef _MVS_CAMERACUDA_H_
#define _MVS_CAMERACUDA_H_


// I N C L U D E S /////////////////////////////////////////////////

#include <string>
#include <vector>

#include "Maths.h"
#include "../../Math/Camera/CameraModels.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

namespace CUDA {

// the CameraModels.h models of the kernels
typedef SEACAVE::Pinhole<float> PinholeModel;
typedef SEACAVE::Spherical<float> SphericalModel;

// Camera pose
struct Pose {
	Matrix3 R; // rotation matrix
	Point3 C; // camera center

	__host__ __device__ Pose() {}
	__host__ __device__ Pose(const Matrix3& R, const Point3& C) :
		R(R), C(C) {}

	// transform a 3D point from world space to camera space
	__host__ __device__ inline Point3 TransformPointW2C(const Point3& X) const {
		return R * (X - C);
	}

	// transform a 3D point in camera space to world space
	__host__ __device__ inline Point3 TransformPointC2W(const Point3& X) const {
		return R.transpose() * X + C;
	}
};
/*----------------------------------------------------------------*/

// Camera view: the intrinsics in the fields of the CameraModels.h models (a pinhole K in pixels, or
// the angular K of an equirectangular camera: pixels per radian and the pixel of longitude and
// elevation 0), the pose and the image size; the camera model is not stored, the code reading a view
// knows it (a kernel's MODEL, PatchMatch::Params::sphericalViews) and builds it with Model<MODEL>()
struct Camera {
	float fx, fy, cx, cy;
	Pose pose;
	Point2i size;

	__host__ __device__ Camera() {}
	__host__ __device__ Camera(const Matrix3& K, const Matrix3& R, const Point3& C, int width, int height) :
		fx(K(0,0)), fy(K(1,1)), cx(K(0,2)), cy(K(1,2)), pose(R, C), size(width, height)
	{
		ASSERT(K(0,1) == 0 && K(1,0) == 0 && K(2,0) == 0 && K(2,1) == 0 && K(2,2) == 1);
		ASSERT(fx > 0 && fy > 0 && width > 0 && height > 0);
	}

	// the camera model of the view: PinholeModel or SphericalModel
	template <typename MODEL>
	__host__ __device__ inline MODEL Model() const {
		return MODEL{fx, fy, cx, cy};
	}
};
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS

#endif // _MVS_CAMERACUDA_H_
