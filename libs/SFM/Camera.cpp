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
	Point2 p(Pinhole<REAL>::Perspective(X));
	if (HasDistortion())
		p = Distort(p);
	return std::make_pair(Point2(GetModel().NormalizedToPixel(p)), X.z > REAL(0));
}

Point3 PinholeCamera::Unproject(const Point2& x) const
{
	ASSERT(IsValid());
	Point2 p(GetModel().PixelToNormalized(x));
	if (HasDistortion())
		p = Undistort(p);
	// ray on the z=1 plane
	return p.homogeneous();
}
Point3 PinholeCamera::UnprojectNormalized(const Point2& x) const
{
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

/*----------------------------------------------------------------*/


// SphericalCamera implementation
std::pair<Point2, bool> SphericalCamera::Project(const Point3& X) const
{
	ASSERT(IsValid());
	Eigen::Vector2d x;
	const bool valid = GetModel().Project(X, x);
	return std::make_pair(Point2(x), valid);
}

Point3 SphericalCamera::Unproject(const Point2& x) const
{
	ASSERT(IsValid());
	// unit bearing scaled to |z|=1, keeping the sign of z, so the front hemisphere matches the
	// pinhole (X/Z, Y/Z, 1) form; singular where the bearing is orthogonal to the optical axis,
	// so callers needing a bearing everywhere use UnprojectNormalized()
	const Point3 d(UnprojectNormalized(x));
	return d * (REAL(1) / ABS(d.z));
}
Point3 SphericalCamera::UnprojectNormalized(const Point2& x) const
{
	ASSERT(IsValid());
	return Point3(GetModel().Ray(x));
}

REAL SphericalCamera::PixelErrorToAngular(REAL pixelError) const
{
	// For spherical/equirectangular projection:
	// 1 pixel ≈ (2π / width) radians in longitude direction
	const REAL pixelToRadian = REAL(2 * M_PI) / size.width;
	return pixelError * pixelToRadian;
}
/*----------------------------------------------------------------*/
