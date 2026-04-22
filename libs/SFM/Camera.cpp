////////////////////////////////////////////////////////////////////
// Camera.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Camera.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

#ifdef _USE_BOOST
// Provide GUID + implementation for exported camera classes in this TU.
// Using BOOST_CLASS_EXPORT_GUID in one translation unit avoids ODR/template-specialization
// issues that occur when trying to place KEY macros in headers included by many TUs.
BOOST_CLASS_EXPORT_GUID(SFM::PinholeCamera, "SFM::PinholeCamera")
BOOST_CLASS_EXPORT_GUID(SFM::SphericalCamera, "SFM::SphericalCamera")
#endif

// PinholeCamera implementation
std::pair<Point2, bool> PinholeCamera::Project(const Point3& X) const
{
	ASSERT(IsValid());
	// Normalize by z coordinate
	const REAL invZ = REAL(1) / X.z;
	Point2 p(X.x * invZ, X.y * invZ);
	// Apply distortion if enabled
	if (HasDistortion())
		p = Distort(p);
	// Apply intrinsics
	return std::make_pair(
		Point2(fx * p.x + cx, fy * p.y + cy),
		X.z > REAL(1e-10) // valid if point is in front of camera
	);
}

Point3 PinholeCamera::Unproject(const Point2& x) const
{
	ASSERT(IsValid());
	// Remove intrinsics
	Point2 p(
		(x.x - cx) / fx,
		(x.y - cy) / fy
	);
	// Remove distortion if enabled
	if (HasDistortion())
		p = Undistort(p);
	// Return ray as a 3D point on the z=1 normalized plane
	return p.homogeneous();
}
Point3 PinholeCamera::UnprojectNormalized(const Point2& x) const
{
	// Return (normalized) ray
	return normalized(Unproject(x));
}

KMatrix PinholeCamera::GetK() const
{
	KMatrix K(KMatrix::IDENTITY);
	K(0, 0) = fx;
	K(1, 1) = fy;
	K(0, 2) = cx;
	K(1, 2) = cy;
	return K;
}
void PinholeCamera::SetK(const KMatrix& K)
{
	ASSERT(K(0, 1) == 0);
	fx = K(0, 0);
	fy = K(1, 1);
	cx = K(0, 2);
	cy = K(1, 2);
}

cv::Mat PinholeCamera::GetDistortionCoeffs() const
{
	cv::Mat distCoeffs(8, 1, CV_64F);
	distCoeffs.at<double>(0) = k1;
	distCoeffs.at<double>(1) = k2;
	distCoeffs.at<double>(2) = p1;
	distCoeffs.at<double>(3) = p2;
	distCoeffs.at<double>(4) = k3;
	distCoeffs.at<double>(5) = k4;
	distCoeffs.at<double>(6) = k5;
	distCoeffs.at<double>(7) = k6;
	return distCoeffs;
}

REAL PinholeCamera::PixelErrorToAngular(REAL pixelError) const
{
	ASSERT(pixelError >= 0);
	// Use average focal length for conversion
	const REAL avgFocal = (fx + fy) / REAL(2);
	// Angular error = atan(pixel_error / focal_length)
	return ATAN(pixelError / avgFocal);
}

REAL PinholeCamera::ComputeMaxDistortion(int sampleDensity) const
{
	ASSERT(IsValid());
	ASSERT(sampleDensity > 0);
	// If no distortion, return 0
	if (!HasDistortion())
		return REAL(0);

	// Sample points across the image in a grid pattern
	REAL maxDistortionSq = REAL(0);
	const REAL stepX = size.width / REAL(sampleDensity);
	const REAL stepY = size.height / REAL(sampleDensity);
	for (int i = 0; i <= sampleDensity; ++i) {
		for (int j = 0; j <= sampleDensity; ++j) {
			// Image coordinates
			const Point2 pixel(i * stepX, j * stepY);
			// Convert to normalized coordinates (undistorted)
			const Point2 p(
				(pixel.x - cx) / fx,
				(pixel.y - cy) / fy
			);
			// Apply distortion
			const Point2 pd = Distort(p);
			// Convert distortion to pixel units
			const Point2 distortion(
				fx * (pd.x - p.x),
				fy * (pd.y - p.y)
			);
			// Distortion magnitude in pixels
			const REAL distortionMagSq = normSq(distortion);
			if (maxDistortionSq < distortionMagSq)
				maxDistortionSq = distortionMagSq;
		}
	}
	return SQRT(maxDistortionSq);
}

// Helper function: apply distortion to normalized coordinates
Point2 PinholeCamera::Distort(const Point2& p) const
{
	const REAL x2 = p.x * p.x;
	const REAL y2 = p.y * p.y;
	const REAL xy = p.x * p.y;
	const REAL r2 = x2 + y2;
	const REAL r4 = r2 * r2;
	const REAL r6 = r4 * r2;

	// Radial distortion
	REAL radial;
	if (useAdditionalDistortion) {
		// Full rational model with additional distortion
		radial = (REAL(1) + k1*r2 + k2*r4 + k3*r6) /
		         (REAL(1) + k4*r2 + k5*r4 + k6*r6);
	} else {
		// Standard model without additional distortion
		radial = REAL(1) + k1*r2 + k2*r4 + k3*r6;
	}

	// Tangential distortion
	const Point2 d(
		REAL(2)*p1*xy + p2*(r2 + REAL(2)*x2),
		p1*(r2 + REAL(2)*y2) + REAL(2)*p2*xy
	);

	// Apply distortion
	return p * radial + d;
}

// Helper function: remove distortion from normalized coordinates (iterative)
Point2 PinholeCamera::Undistort(const Point2& p) const
{
	// Iterative undistortion: solve for u such that Distort(u) = p
	Point2 u(p);
	for (int iter = 0; iter < 20; ++iter) {
		const REAL x2 = u.x * u.x;
		const REAL y2 = u.y * u.y;
		const REAL xy = u.x * u.y;
		const REAL r2 = x2 + y2;
		const REAL r4 = r2 * r2;
		const REAL r6 = r4 * r2;

		REAL radial;
		if (useAdditionalDistortion) {
			// Full rational model
			radial = (REAL(1) + k1*r2 + k2*r4 + k3*r6) /
			         (REAL(1) + k4*r2 + k5*r4 + k6*r6);
		} else {
			// Standard model
			radial = REAL(1) + k1*r2 + k2*r4 + k3*r6;
		}

		const Point2 d(
			REAL(2)*p1*xy + p2*(r2 + REAL(2)*x2),
			p1*(r2 + REAL(2)*y2) + REAL(2)*p2*xy
		);

		// Apply inverse distortion model
		const Point2 nu = (p - d) / radial;

		// Check convergence
		const bool converged = ISZERO(nu - u);
		u = nu;
		if (converged)
			break;
	}
	return u;
}
/*----------------------------------------------------------------*/


// SphericalCamera implementation
std::pair<Point2, bool> SphericalCamera::Project(const Point3& X) const
{
	ASSERT(IsValid());

	// Convert 3D point to spherical coordinates
	const REAL r = norm(X);
	ASSERT(ISFINITE(r));
	if (r < REAL(1e-10))
		return std::make_pair(Point2(size.width, size.height)/2, false);

	// Longitude (theta) and latitude (phi)
	const REAL theta = ATAN2(X.x, X.z);  // azimuth angle [-pi, pi]
	const REAL phi = ASIN(CLAMP(X.y / r, REAL(-1), REAL(1)));  // elevation angle [-pi/2, pi/2]

	// Map to image coordinates
	// theta: -pi to pi -> 0 to width
	// phi: -pi/2 to pi/2 -> height to 0 (y-axis points down)
	return std::make_pair(Point2(
		(theta + REAL(M_PI)) / (REAL(2) * REAL(M_PI)) * size.width,
		(REAL(M_PI_2) - phi) / REAL(M_PI) * size.height
	), true);
}

Point2 SphericalCamera::MapImageToSpherical(const Point2& x) const
{
	// Map image coordinates to spherical angles
	ASSERT(IsValid());
	const REAL theta = (x.x / size.width) * REAL(2) * REAL(M_PI) - REAL(M_PI);
	const REAL phi = REAL(M_PI_2) - (x.y / size.height) * REAL(M_PI);
	return Point2(theta, phi);
}

Point3 SphericalCamera::Unproject(const Point2& x) const
{
	// Map image coordinates to spherical angles (x or theta = longitude, y or phi = latitude)
	const Point2 sph = MapImageToSpherical(x);

	// Convert spherical to Cartesian (not normalized ray);
	// Compute true unit bearing vector in camera space:
	//   d = (sin(theta)*cos(phi),  sin(phi),  cos(theta)*cos(phi))
	// Then scale so |d.z| = 1, preserving sign(d.z) = sign(cos(theta))
	// (since phi is in [-pi/2, pi/2], cos(phi) >= 0 always).
	//
	// For the FRONT hemisphere (cos(theta) > 0) this matches the pinhole
	// (X/Z, Y/Z, +1) form exactly, so pinhole callers see no behavior change.
	// For the BACK hemisphere (cos(theta) < 0) we return (-tan(theta),
	// -tan(phi)/cos(theta), -1); the (x, y) sign flip combined with z = -1
	// produces a 3D vector that points along the true back-facing bearing
	// (not the aliased front-hemisphere direction the old 2D form gave).
	//
	// Singular at the equator sides (cos(theta) = 0, pixel x = W/4 or 3W/4
	// on the vertical midline) and the poles (cos(phi) = 0, pixel y = 0 or H),
	// which are measure-zero subsets of the image. Callers needing a strictly
	// unit bearing vector should use UnprojectNormalized() instead — it has
	// no singularity on the sphere.
	const REAL cosTheta = COS(sph.x);
	const REAL sign = (cosTheta >= REAL(0)) ? REAL(1) : REAL(-1);
	return Point3(
		TAN(sph.x) * sign,
		TAN(sph.y) * sign / cosTheta,
		sign
	);
}
Point3 SphericalCamera::UnprojectNormalized(const Point2& x) const
{
	// Map image coordinates to spherical angles
	const Point2 sph = MapImageToSpherical(x);

	// Convert spherical to Cartesian (normalized ray)
	const REAL cosPhi = COS(sph.y);
	return Point3(
		cosPhi * SIN(sph.x),
		SIN(sph.y),
		cosPhi * COS(sph.x)
	);
}

REAL SphericalCamera::PixelErrorToAngular(REAL pixelError) const
{
	// For spherical/equirectangular projection:
	// 1 pixel ≈ (2π / width) radians in longitude direction
	const REAL pixelToRadian = REAL(2 * M_PI) / size.width;
	return pixelError * pixelToRadian;
}
/*----------------------------------------------------------------*/
