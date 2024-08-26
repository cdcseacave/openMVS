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


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

namespace CUDA {

// Linear camera model
struct LinearCameraModel {
	Point2 f; // focal length
	Point2 p; // principal point

	__host__ __device__ LinearCameraModel() {}
	__host__ __device__ LinearCameraModel(float fx, float fy, float cx, float cy) :
		f(fx, fy), p(cx, cy) {}
	__host__ __device__ LinearCameraModel(const Matrix3& K) :
		f(K(0,0), K(1,1)), p(K(0,2), K(1,2)) { ASSERT(K(0,1) == 0); }

	__host__ __device__ inline Matrix3 K() const {
		Matrix3 M; M <<
			f.x(), 0, p.x(),
			0, f.y(), p.y(),
			0, 0, 1;
		return M;
	}

	// transform a point in image space to camera space
	__host__ __device__ inline Point2 NormalizePoint(const Point2& x) const {
		return Point2(
			(x.x() - p.x()) / f.x(),
			(x.y() - p.y()) / f.y());
	}

	// project a point in camera space to image space
	__host__ __device__ inline Point2 TransformPointC2I(const Point3& X) const {
		return Point2(
			f.x() * X.x() / X.z() + p.x(),
			f.y() * X.y() / X.z() + p.y());
	}

	// back-project a point in image space to camera space
	__host__ __device__ inline Point3 TransformPointI2C(const Point2& x, const float depth = 1.f) const {
		return Point3(
			depth * (x.x() - p.x()) / f.x(),
			depth * (x.y() - p.y()) / f.y(),
			depth);
	}

	// compute camera ray direction for the given pixel
	__host__ __device__ inline Point3 ViewDirection(const Point2i& x) const {
		return TransformPointI2C(x.cast<float>()).normalized();
	}
};
/*----------------------------------------------------------------*/

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

// Camera view
struct Camera {
	LinearCameraModel model;
	Pose pose;
	Point2i size;

	__host__ __device__ Camera() {}
	__host__ __device__ Camera(const LinearCameraModel& model, const Pose& pose, int width=0, int height=0) :
		model(model), pose(pose), size(width, height) {}
	__host__ __device__ Camera(const Matrix3& K, const Matrix3& R, const Point3& C, int width=0, int height=0) :
		model(K), pose(R, C), size(width, height) {}

	// project a 3D point in world space to image space
	__host__ __device__ inline Point2 TransformPointW2I(const Point3& X) const {
		return model.TransformPointC2I(pose.TransformPointW2C(X));
	}

	// back-project a point in image space to 3D point in world space
	__host__ __device__ inline Point3 TransformPointI2W(const Point2& x, const float depth = 1.f) const {
		return pose.TransformPointC2W(model.TransformPointI2C(x, depth));
	}
};
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS

#endif // _MVS_CAMERACUDA_H_
