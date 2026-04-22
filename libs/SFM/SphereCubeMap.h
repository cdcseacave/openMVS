////////////////////////////////////////////////////////////////////
// SphereCubeMap.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_SPHERECUBEMAP_H_
#define _SFM_SPHERECUBEMAP_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Sphere → tangent-pinhole-faces utility: given an equirectangular source
// image, render N tangent pinhole views (cube, icosahedron, etc.) that
// downstream pinhole pipelines can consume uniformly.
//
// Supported face counts (polyhedron face-centred, except N=4 which is an
// equatorial subset of the cube):
//   4  — equatorial (+Z, -Z, +X, -X)
//   6  — cube (+Z, -Z, +X, -X, +Y, -Y) [default]
//   8  — octahedron face-centred
//   12 — dodecahedron face-centred
//   20 — icosahedron face-centred
//
// This namespace contains only pure geometry + pixel rendering. MVS-export
// glue that consumes the types below lives in InterfaceMVS.cpp (as a
// continuation of this namespace scoped to that TU).
//
// The API is split into two steps so callers that process many spherical
// images with the same settings only pay the geometry cost once:
//
//   const auto geom  = SphereCubeMap::MakeTangentFacesGeometry(6, 1024);
//   for (...) {
//     auto faces = SphereCubeMap::SphericalToTangentialFaces(img, geom);
//     // use geom.rotations[k], geom.K, faces[k]
//   }
namespace SphereCubeMap {

// ---- Low-level geometric tables (also consumed directly by tests) ----

// Rotation matrices mapping rig body frame to face camera frame for the given
// face count. Returned by value; internally cached per N, so repeated calls
// reuse the same table. The 6-face table is bit-exact with prior cube-map
// implementations to preserve MVS-export rig contract.
std::vector<Matrix3x3> FaceRotations(int n);

// FOV per face in degrees. Chosen so neighbouring faces overlap ~10–15%:
//   4, 6   -> 91.0
//   8      -> 80.0
//   12     -> 72.0
//   20     -> 50.0
REAL FaceFOVDegrees(int n);

// Pinhole intrinsic matrix for a square face:
//   fx = fy = faceSize / (2 * tan(FaceFOVDegrees(n) / 2))
//   cx = cy = faceSize / 2
Matrix3x3 FaceIntrinsics(int faceSize, int n);
/*----------------------------------------------------------------*/


// ---- Split API: geometry once, images per-spherical-source ----

// Bundle of per-face rig geometry + shared pinhole intrinsics. Cheap to
// build and cheap to copy; intended to be computed ONCE for a given
// (numFaces, faceSize) pair and reused across every spherical image
// rendered with those settings.
struct SFM_API TangentFacesGeometry {
	std::vector<Matrix3x3> rotations;  // N body->face rotations (== FaceRotations(numFaces))
	Matrix3x3              K;          // shared pinhole intrinsics (== FaceIntrinsics(faceSize, numFaces))
	int                    numFaces = 0;
	int                    faceSize = 0;
};

// Construct a TangentFacesGeometry for the given (numFaces, faceSize).
// Returns a geometry with numFaces==0 if numFaces is not one of {4,6,8,12,20}.
TangentFacesGeometry MakeTangentFacesGeometry(int numFaces, int faceSize);


// Render ONE spherical image into N tangent-pinhole faces using a pre-built
// geometry. Returns a vector of N face images parallel to geometry.rotations.
// Each face image is (re)allocated to geometry.faceSize x geometry.faceSize.
// Pixel-type templated; explicit instantiations emitted for Pixel8U
// (Image8U3) and Pixel32F (Image32F3). Add new rows at the end of the .cpp
// if a caller needs other TImage pixel types.
template <typename TYPE>
std::vector<TImage<TYPE>> SphericalToTangentialFaces(
	const TImage<TYPE>& sphericalImage,
	const TangentFacesGeometry& geometry);
/*----------------------------------------------------------------*/

} // namespace SphereCubeMap
} // namespace SFM

#endif // _SFM_SPHERECUBEMAP_H_
