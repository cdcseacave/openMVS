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

// free scaling of the undistorted camera matrix (see UndistortedK) used wherever the undistorted
// images are not written to disk: the MVS loader and the depth-maps that go with its images
constexpr double UNDISTORT_ALPHA = 0.6;

// camera matrix of the undistorted pinhole image of a Brown-distorted camera, same image size:
// cv::getOptimalNewCameraMatrix with free scaling alpha in [0,1] (0: only valid pixels are kept,
// 1: all source pixels are kept); K and size describe the source image; the one rule shared by
// the SfM export, the MVS loader and the depth-map undistortion, deterministic for given inputs
MATH_API KMatrix UndistortedK(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size, double alpha=UNDISTORT_ALPHA);

// remap tables from an undistorted pinhole image to its Brown-distorted source image
struct MATH_API UndistortMaps {
	cv::Mat map1, map2; // fixed-point remap tables (CV_16SC2 + CV_16UC1), sized as the undistorted image
	KMatrix K; // camera matrix of the undistorted image

	// tables for the undistorted image with camera matrix newK; K and size describe the source image
	void Init(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size, const KMatrix& newK);
	// same, newK = UndistortedK(K, distortion, size, alpha)
	void Init(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size, double alpha);

	// resample an image, pixels mapped outside the source are set to 0
	void Remap(cv::InputArray src, cv::OutputArray dst, int interpolation) const;
	// resample a depth-map interpolating only depths similar to the nearest one (IsDepthSimilar);
	// pixels without a valid source depth are set to 0
	void RemapDepth(const Image32F& depthMap, Image32F& undistortedDepthMap) const;
};

// the undistortion of the images of one camera to UndistortedK(K, distortion, size), plus the mask
// of the undistorted pixels with a source: 255 where the bilinear footprint of the source position
// lies inside the source image, 0 elsewhere
struct MATH_API CameraUndistortMaps : UndistortMaps {
	Image8U mask;
};

// the CameraUndistortMaps of the camera (K, distortion) for source images of the given size, from a
// process-wide cache shared by every image of that camera; thread-safe: concurrent requests for
// the same camera wait for one computation; while the cached tables (~7 bytes per pixel) exceed
// a fixed memory budget, the least recently requested ones are dropped from the cache (whoever
// holds them keeps them alive)
MATH_API std::shared_ptr<const CameraUndistortMaps> GetCameraUndistortMaps(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size);
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

// whether numFaces is one of the supported face counts {4,6,8,12,20}
MATH_API bool IsSupportedNumFaces(int numFaces);

// geometry for the given (numFaces, faceSize); numFaces must be supported and faceSize positive
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
