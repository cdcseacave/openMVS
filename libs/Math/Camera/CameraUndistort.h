////////////////////////////////////////////////////////////////////
// CameraUndistort.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_CAMERAUNDISTORT_H__
#define __SEACAVE_CAMERAUNDISTORT_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "CameraModels.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Resampling between camera models.

// remap tables from an undistorted pinhole image to its Brown-distorted source image
struct MATH_API UndistortMaps {
	cv::Mat map1, map2; // fixed-point remap tables (CV_16SC2 + CV_16UC1), sized as the undistorted image
	KMatrix K; // camera matrix of the undistorted image

	// tables for the undistorted image with camera matrix newK; K and size describe the source image
	void Init(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size, const KMatrix& newK);
	// same, newK chosen by cv::getOptimalNewCameraMatrix with free scaling alpha in [0,1]
	// (0: only valid pixels are kept, 1: all source pixels are kept)
	void Init(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size, double alpha);

	// resample an image, pixels mapped outside the source are set to 0
	void Remap(cv::InputArray src, cv::OutputArray dst, int interpolation) const;
	// resample a depth-map interpolating only depths similar to the nearest one (IsDepthSimilar);
	// pixels without a valid source depth are set to 0
	void RemapDepth(const Image32F& depthMap, Image32F& undistortedDepthMap) const;
};
/*----------------------------------------------------------------*/


// Equirectangular image -> N tangent pinhole views (cube, icosahedron, etc.) that pinhole
// pipelines consume uniformly. Supported face counts (polyhedron face-centred, except N=4 which
// is the equatorial subset of the cube):
//   4  - equatorial (+Z, -Z, +X, -X)
//   6  - cube (+Z, -Z, +X, -X, +Y, -Y) [default]
//   8  - octahedron face-centred
//   12 - dodecahedron face-centred
//   20 - icosahedron face-centred
// The geometry is built once per (numFaces, faceSize) and reused for every image:
//   const auto geom = SphereCubeMap::MakeTangentFacesGeometry(6, 1024);
//   for (...) {
//     auto faces = SphereCubeMap::SphericalToTangentialFaces(img, geom);
//     // use geom.rotations[k], geom.K, faces[k]
//   }
namespace SphereCubeMap {

// rotations mapping the panorama camera frame to each face camera frame (cached per n)
MATH_API std::vector<Matrix3x3> FaceRotations(int n);

// FOV per face in degrees, so that neighbouring faces overlap ~10-15%:
//   4, 6 -> 91; 8 -> 80; 12 -> 72; 20 -> 50
MATH_API REAL FaceFOVDegrees(int n);

// camera matrix of a square face: f = faceSize / (2 * tan(FOV / 2)), c = (faceSize - 1) / 2
MATH_API Matrix3x3 FaceIntrinsics(int faceSize, int n);

// per-face rotations + shared pinhole intrinsics for one (numFaces, faceSize) pair
struct MATH_API TangentFacesGeometry {
	std::vector<Matrix3x3> rotations; // N body->face rotations (== FaceRotations(numFaces))
	Matrix3x3              K;         // shared pinhole intrinsics (== FaceIntrinsics(faceSize, numFaces))
	int                    numFaces = 0;
	int                    faceSize = 0;
};

// geometry for the given (numFaces, faceSize); numFaces==0 if numFaces is not one of {4,6,8,12,20}
MATH_API TangentFacesGeometry MakeTangentFacesGeometry(int numFaces, int faceSize);

// render one equirectangular image into the N faces of the geometry (bilinear, wrapping across the
// seam); instantiated for Pixel8U and Pixel32F
template <typename TYPE>
MATH_API std::vector<TImage<TYPE>> SphericalToTangentialFaces(
	const TImage<TYPE>& sphericalImage,
	const TangentFacesGeometry& geometry);
/*----------------------------------------------------------------*/

} // namespace SphereCubeMap

} // namespace SEACAVE

#endif // __SEACAVE_CAMERAUNDISTORT_H__
