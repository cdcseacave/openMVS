////////////////////////////////////////////////////////////////////
// CameraModels.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_CAMERAMODELS_H__
#define __SEACAVE_CAMERAMODELS_H__


// I N C L U D E S /////////////////////////////////////////////////

#include <cmath>
#include <cstdint>


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Camera projection models shared by host code, CUDA device code and Ceres (T = ceres::Jet).
// The including context provides Eigen, ASSERT and HOST_DEVICE (Common.h, or CUDA/Maths.h in .cu files).
// Conventions, common to all models:
//  - pixel centers lie at integer coordinates (OpenCV/MVS): pixel (0,0) covers [-0.5,0.5]^2;
//  - camera frame x-right, y-down, z-forward;
//  - Ray(x) is the camera-space direction of pixel x, Project(X, x) its inverse (returns
//    whether X is visible), and Depth(X) the distance along the ray that Ray(x) scales.

// model ids, stored in the scene files
enum class CameraModelType : uint8_t {
	PINHOLE = 0,
	BROWN = 1, // pinhole + BrownDistortion
	SPHERICAL = 2, // equirectangular, 360x180 degrees
};

// skew-free pinhole; Ray(x) lies on the z=1 plane, so the depth along it is z
template<typename T>
struct Pinhole {
	typedef Eigen::Matrix<T,2,1> Vec2;
	typedef Eigen::Matrix<T,3,1> Vec3;

	T fx, fy; // focal lengths in pixels
	T cx, cy; // principal point in pixels

	// pixel -> normalized image-plane coordinates (z=1)
	HOST_DEVICE Vec2 PixelToNormalized(const Vec2& x) const {
		ASSERT(fx > T(0) && fy > T(0));
		return Vec2((x.x() - cx) / fx, (x.y() - cy) / fy);
	}
	// normalized image-plane coordinates -> pixel
	HOST_DEVICE Vec2 NormalizedToPixel(const Vec2& n) const {
		return Vec2(fx * n.x() + cx, fy * n.y() + cy);
	}
	// camera-space point -> normalized image-plane coordinates
	HOST_DEVICE static Vec2 Perspective(const Vec3& X) {
		const T invZ(T(1) / X.z());
		return Vec2(X.x() * invZ, X.y() * invZ);
	}

	HOST_DEVICE Vec3 Ray(const Vec2& x) const {
		const Vec2 n(PixelToNormalized(x));
		return Vec3(n.x(), n.y(), T(1));
	}
	HOST_DEVICE bool Project(const Vec3& X, Vec2& x) const {
		x = NormalizedToPixel(Perspective(X));
		return X.z() > T(0);
	}
	HOST_DEVICE static T Depth(const Vec3& X) {
		return X.z();
	}
};
/*----------------------------------------------------------------*/


// Brown-Conrady lens distortion on normalized image-plane coordinates, OpenCV coefficient order
// and meaning: radial (1+k1*r2+k2*r4+k3*r6)/(1+k4*r2+k5*r4+k6*r6), tangential p1, p2;
// the rational part k4..k6 is zero for the plain polynomial model
template<typename T>
HOST_DEVICE inline Eigen::Matrix<T,2,1> DistortBrown(const Eigen::Matrix<T,2,1>& p,
	const T& k1, const T& k2, const T& p1, const T& p2, const T& k3, const T& k4, const T& k5, const T& k6)
{
	const T x2(p.x() * p.x());
	const T y2(p.y() * p.y());
	const T xy(p.x() * p.y());
	const T r2(x2 + y2);
	const T r4(r2 * r2);
	const T r6(r4 * r2);
	const T radial((T(1) + k1*r2 + k2*r4 + k3*r6) / (T(1) + k4*r2 + k5*r4 + k6*r6));
	return Eigen::Matrix<T,2,1>(
		p.x() * radial + (T(2)*p1*xy + p2*(r2 + T(2)*x2)),
		p.y() * radial + (p1*(r2 + T(2)*y2) + T(2)*p2*xy));
}

template<typename T>
struct BrownDistortion {
	typedef Eigen::Matrix<T,2,1> Vec2;

	T k1, k2, p1, p2, k3, k4, k5, k6;

	HOST_DEVICE bool IsZero() const {
		return k1 == T(0) && k2 == T(0) && p1 == T(0) && p2 == T(0) && k3 == T(0) && k4 == T(0) && k5 == T(0) && k6 == T(0);
	}

	// undistorted -> distorted normalized coordinates
	HOST_DEVICE Vec2 Distort(const Vec2& p) const {
		return DistortBrown(p, k1, k2, p1, p2, k3, k4, k5, k6);
	}
	// Jacobian of Distort at p
	HOST_DEVICE Eigen::Matrix<T,2,2> Jacobian(const Vec2& p) const {
		const T x2(p.x() * p.x());
		const T y2(p.y() * p.y());
		const T xy(p.x() * p.y());
		const T r2(x2 + y2);
		const T r4(r2 * r2);
		const T num(T(1) + k1*r2 + k2*r4 + k3*r4*r2);
		const T invDen(T(1) / (T(1) + k4*r2 + k5*r4 + k6*r4*r2));
		const T radial(num * invDen);
		// d(radial)/d(r2) * 2
		const T dRadial(T(2) * ((k1 + T(2)*k2*r2 + T(3)*k3*r4) - radial * (k4 + T(2)*k5*r2 + T(3)*k6*r4)) * invDen);
		Eigen::Matrix<T,2,2> J;
		J(0,0) = radial + dRadial*x2 + T(2)*p1*p.y() + T(6)*p2*p.x();
		J(0,1) = dRadial*xy + T(2)*p1*p.x() + T(2)*p2*p.y();
		J(1,0) = dRadial*xy + T(2)*p1*p.x() + T(2)*p2*p.y();
		J(1,1) = radial + dRadial*y2 + T(6)*p1*p.y() + T(2)*p2*p.x();
		return J;
	}
	// distorted -> undistorted normalized coordinates by Newton iterations from p, stopped when a step
	// moves both coordinates by less than 1e-12, or after 20 steps; the distortion must be
	// invertible between p and the solution (positive Jacobian determinant)
	HOST_DEVICE Vec2 Undistort(const Vec2& p) const {
		using std::abs;
		Vec2 u(p);
		for (int iter = 0; iter < 20; ++iter) {
			const Vec2 r(Distort(u) - p);
			const Eigen::Matrix<T,2,2> J(Jacobian(u));
			const T det(J(0,0)*J(1,1) - J(0,1)*J(1,0));
			ASSERT(det > T(0));
			const Vec2 step((J(1,1)*r.x() - J(0,1)*r.y()) / det, (J(0,0)*r.y() - J(1,0)*r.x()) / det);
			u -= step;
			if (abs(step.x()) < T(1e-12) && abs(step.y()) < T(1e-12))
				break;
		}
		return u;
	}
};
/*----------------------------------------------------------------*/


// equirectangular 360x180 degrees panorama: column = longitude, row = elevation;
// longitude atan2(x,z) grows with the column and is 0 at cx (forward +Z), elevation asin(-y/|X|)
// is pi/2 on the top row (up = -Y, as for an upright pinhole camera);
// Ray(x) is a unit vector, so the depth along it is |X|
template<typename T>
struct Spherical {
	typedef Eigen::Matrix<T,2,1> Vec2;
	typedef Eigen::Matrix<T,3,1> Vec3;

	T fx, fy; // pixels per radian: width/(2*pi), height/pi
	T cx, cy; // pixel of longitude 0 and elevation 0: (width-1)/2, (height-1)/2

	HOST_DEVICE static Spherical FromSize(int width, int height) {
		ASSERT(width > 0 && height > 0);
		return Spherical{
			T(width) / T(2*3.14159265358979323846), T(height) / T(3.14159265358979323846),
			T(width-1) / T(2), T(height-1) / T(2)};
	}

	// pixel -> (longitude, elevation) in radians
	HOST_DEVICE Vec2 Angles(const Vec2& x) const {
		ASSERT(fx > T(0) && fy > T(0));
		return Vec2((x.x() - cx) / fx, (cy - x.y()) / fy);
	}

	HOST_DEVICE Vec3 Ray(const Vec2& x) const {
		using std::cos; using std::sin;
		const Vec2 a(Angles(x));
		const T cosElevation(cos(a.y()));
		return Vec3(cosElevation * sin(a.x()), -sin(a.y()), cosElevation * cos(a.x()));
	}
	HOST_DEVICE bool Project(const Vec3& X, Vec2& x) const {
		using std::atan2; using std::sqrt;
		x = Vec2(
			fx * atan2(X.x(), X.z()) + cx,
			fy * atan2(X.y(), sqrt(X.x()*X.x() + X.z()*X.z())) + cy);
		return X.squaredNorm() > T(0);
	}
	HOST_DEVICE static T Depth(const Vec3& X) {
		return X.norm();
	}
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_CAMERAMODELS_H__
