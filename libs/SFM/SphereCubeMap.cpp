////////////////////////////////////////////////////////////////////
// SphereCubeMap.cpp
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "SphereCubeMap.h"

using namespace SFM;
using namespace SFM::SphereCubeMap;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable save rendered faces to disk for visual inspection (debug only)
//#define SFM_DEBUG_SPHERICAL_FACES


// S T R U C T S ///////////////////////////////////////////////////

namespace {

// ------------------------------------------------------------------
// 6-face cube rotation table — bit-exact with the pre-generalization
// implementation so MVS-export outputs are unchanged.
//
// Face order: +Z forward, -Z back, +X right, -X left, +Y up (sky),
// -Y down (ground). All rotations are pure SO(3) elements (det=+1)
// that map the body Y-UP frame (SphericalCamera's convention — +Y is
// physical "up") to a face "Y-UP" frame whose +Z points along the
// named body axis.
//
// Because SphericalCamera uses Y-UP but MVS's pinhole formula assumes
// Y-DOWN, the rendered face image looks vertically inverted to a
// human viewer (sky at bottom, ground at top for face 0). This is
// intentional and self-consistent: MVS's K*cam.R*(X-C) chain produces
// pixel coordinates that exactly match where each sampled equirect
// direction was written, so dense reconstruction, meshing and
// texturing all behave correctly even though the raw face files
// look flipped.
// ------------------------------------------------------------------
std::array<Matrix3x3, 6> BuildFaceRotations6()
{
	std::array<Matrix3x3, 6> R;
	// Face 0: +Z forward → body +Z = face +Z. R = I.
	R[0] = Matrix3x3::IDENTITY;
	// Face 1: -Z back → body -Z = face +Z. R = Ry(180°) = diag(-1, 1, -1).
	R[1] = Matrix3x3::IDENTITY;
	R[1](0,0) = -1; R[1](2,2) = -1;
	// Face 2: +X right → body +X = face +Z. R = Ry(-90°).
	R[2] = Matrix3x3::ZERO;
	R[2](0,2) = -1; R[2](1,1) = 1; R[2](2,0) = 1;
	// Face 3: -X left → body -X = face +Z. R = Ry(+90°).
	R[3] = Matrix3x3::ZERO;
	R[3](0,2) = 1; R[3](1,1) = 1; R[3](2,0) = -1;
	// Face 4: +Y up (sky) → body +Y = face +Z. R = Rx(+90°).
	R[4] = Matrix3x3::ZERO;
	R[4](0,0) = 1; R[4](1,2) = -1; R[4](2,1) = 1;
	// Face 5: -Y down (ground) → body -Y = face +Z. R = Rx(-90°).
	R[5] = Matrix3x3::ZERO;
	R[5](0,0) = 1; R[5](1,2) = 1; R[5](2,1) = -1;
	return R;
}

const std::array<Matrix3x3, 6>& GetFaceRotations6()
{
	static const std::array<Matrix3x3, 6> kRotations = BuildFaceRotations6();
	return kRotations;
}

// Generic body→face rotation for an arbitrary unit "forward" direction n
// (where face +Z points, expressed in body frame). Uses Gram–Schmidt
// against body up-reference (+Y, or +X as fallback when n is parallel to +Y).
// R has rows [right^T; up_face^T; n^T] so R * n_body = (0,0,1), etc.
// The 6-face table does NOT go through this — it's kept bit-exact above —
// but the 8/12/20-face tables do.
Matrix3x3 RotationFromForward(const Point3& n_in)
{
	const REAL nlen = norm(n_in);
	ASSERT(nlen > REAL(0));
	const Point3 n = n_in * (REAL(1) / nlen);

	Point3 up_ref(REAL(0), REAL(1), REAL(0));
	if (ABS(n.y) > REAL(0.99))
		up_ref = Point3(REAL(1), REAL(0), REAL(0));

	Point3 right = up_ref.cross(n);
	const REAL rlen = norm(right);
	ASSERT(rlen > REAL(1e-6));
	right *= REAL(1) / rlen;

	const Point3 up_face = n.cross(right);

	Matrix3x3 R;
	R(0,0) = right.x;    R(0,1) = right.y;    R(0,2) = right.z;
	R(1,0) = up_face.x;  R(1,1) = up_face.y;  R(1,2) = up_face.z;
	R(2,0) = n.x;        R(2,1) = n.y;        R(2,2) = n.z;
	return R;
}

// 4-face: equatorial subset of the 6-face table (+Z, -Z, +X, -X).
std::vector<Matrix3x3> BuildFaceRotations4()
{
	const auto& R6 = GetFaceRotations6();
	return { R6[0], R6[1], R6[2], R6[3] };
}

// 8-face: octahedron face centres at (±1, ±1, ±1) / √3.
std::vector<Matrix3x3> BuildFaceRotations8()
{
	std::vector<Matrix3x3> R;
	R.reserve(8);
	const REAL s = REAL(1) / SQRT(REAL(3));
	for (int sx = +1; sx >= -1; sx -= 2)
		for (int sy = +1; sy >= -1; sy -= 2)
			for (int sz = +1; sz >= -1; sz -= 2)
				R.push_back(RotationFromForward(Point3(sx*s, sy*s, sz*s)));
	return R;
}

// 12-face (dodecahedron face centres = icosahedron vertices).
// Icosahedron vertices (normalised): (0, ±1, ±φ), (±1, ±φ, 0), (±φ, 0, ±1)
// each divided by √(1+φ²) = √(φ+2).
std::vector<Matrix3x3> BuildFaceRotations12()
{
	const REAL phi = (REAL(1) + SQRT(REAL(5))) * REAL(0.5);
	const REAL inv = REAL(1) / SQRT(REAL(1) + phi*phi);
	std::vector<Matrix3x3> R;
	R.reserve(12);
	for (int sy = +1; sy >= -1; sy -= 2)
		for (int sz = +1; sz >= -1; sz -= 2)
			R.push_back(RotationFromForward(Point3(REAL(0), sy*inv, sz*phi*inv)));
	for (int sx = +1; sx >= -1; sx -= 2)
		for (int sy = +1; sy >= -1; sy -= 2)
			R.push_back(RotationFromForward(Point3(sx*inv, sy*phi*inv, REAL(0))));
	for (int sx = +1; sx >= -1; sx -= 2)
		for (int sz = +1; sz >= -1; sz -= 2)
			R.push_back(RotationFromForward(Point3(sx*phi*inv, REAL(0), sz*inv)));
	return R;
}

// 20-face (icosahedron face centres = dodecahedron vertices).
// Dodecahedron vertices all have magnitude √3; normalise by /√3:
//   (±1, ±1, ±1), (0, ±1/φ, ±φ), (±1/φ, ±φ, 0), (±φ, 0, ±1/φ)
// (1/φ² + φ² = 3 exactly since φ² = φ+1 and 1/φ² = 2-φ.)
std::vector<Matrix3x3> BuildFaceRotations20()
{
	const REAL phi    = (REAL(1) + SQRT(REAL(5))) * REAL(0.5);
	const REAL invPhi = REAL(1) / phi;
	const REAL scale  = REAL(1) / SQRT(REAL(3));
	std::vector<Matrix3x3> R;
	R.reserve(20);
	for (int sx = +1; sx >= -1; sx -= 2)
		for (int sy = +1; sy >= -1; sy -= 2)
			for (int sz = +1; sz >= -1; sz -= 2)
				R.push_back(RotationFromForward(Point3(sx*scale, sy*scale, sz*scale)));
	for (int sy = +1; sy >= -1; sy -= 2)
		for (int sz = +1; sz >= -1; sz -= 2)
			R.push_back(RotationFromForward(Point3(REAL(0), sy*invPhi*scale, sz*phi*scale)));
	for (int sx = +1; sx >= -1; sx -= 2)
		for (int sy = +1; sy >= -1; sy -= 2)
			R.push_back(RotationFromForward(Point3(sx*invPhi*scale, sy*phi*scale, REAL(0))));
	for (int sx = +1; sx >= -1; sx -= 2)
		for (int sz = +1; sz >= -1; sz -= 2)
			R.push_back(RotationFromForward(Point3(sx*phi*scale, REAL(0), sz*invPhi*scale)));
	return R;
}

// ------------------------------------------------------------------
// Per-face render kernel — file-local. Equivalent of the old RenderFace
// but parameterised on the shared (f, cx, cy) instead of re-deriving
// them from the 90°-FOV cube convention.
// ------------------------------------------------------------------
template <typename TYPE>
void RenderFaceKernel(
	const TImage<TYPE>& sourceWrapped,
	int sourceWidth,
	const SphericalCamera& sphCam,
	const Matrix3x3& R_face,
	REAL f, REAL cx, REAL cy,
	int faceSize,
	TImage<TYPE>& dst)
{
	if (dst.cols != faceSize || dst.rows != faceSize)
		dst.create(faceSize, faceSize);

	const Matrix3x3 R_face_T = R_face.t();
	const int srcH = sourceWrapped.height();

	for (int v = 0; v < faceSize; ++v) {
		for (int u = 0; u < faceSize; ++u) {
			const REAL x_face = (REAL(u) - cx) / f;
			const REAL y_face = (REAL(v) - cy) / f;
			const Point3 b_body = R_face_T * Point3(x_face, y_face, REAL(1));

			const auto [proj, ok] = sphCam.Project(b_body);
			if (!ok) {
				dst(v, u) = TYPE::BLACK;
				continue;
			}

			REAL px = proj.x;
			REAL py = proj.y;
			while (px < REAL(0)) px += REAL(sourceWidth);
			while (px >= REAL(sourceWidth)) px -= REAL(sourceWidth);
			if (py < REAL(0)) py = REAL(0);
			if (py > REAL(srcH - 1)) py = REAL(srcH - 1);
			dst(v, u) = sourceWrapped.sampleSafe(Point2(px, py));
		}
	}
}

} // namespace


// ------------------------------------------------------------------
// Public API — pure geometric tables
// ------------------------------------------------------------------

std::vector<Matrix3x3> SFM::SphereCubeMap::FaceRotations(int n)
{
	switch (n) {
	case 4: {
		static const std::vector<Matrix3x3> kR = BuildFaceRotations4();
		return kR;
	}
	case 6: {
		const auto& R6 = GetFaceRotations6();
		return std::vector<Matrix3x3>(R6.begin(), R6.end());
	}
	case 8: {
		static const std::vector<Matrix3x3> kR = BuildFaceRotations8();
		return kR;
	}
	case 12: {
		static const std::vector<Matrix3x3> kR = BuildFaceRotations12();
		return kR;
	}
	case 20: {
		static const std::vector<Matrix3x3> kR = BuildFaceRotations20();
		return kR;
	}
	}
	ASSERT("SphereCubeMap::FaceRotations: unsupported numFaces (expected 4,6,8,12,20)" == NULL);
	return {};
}

REAL SFM::SphereCubeMap::FaceFOVDegrees(int n)
{
	switch (n) {
	case 4:
	case 6:  return REAL(91);
	case 8:  return REAL(80);
	case 12: return REAL(72);
	case 20: return REAL(50);
	}
	ASSERT("SphereCubeMap::FaceFOVDegrees: unsupported numFaces" == NULL);
	return REAL(90);
}

Matrix3x3 SFM::SphereCubeMap::FaceIntrinsics(int faceSize, int n)
{
	ASSERT(faceSize > 0);
	const REAL fovRad = D2R(FaceFOVDegrees(n));
	const REAL f = REAL(faceSize) * REAL(0.5) / TAN(fovRad * REAL(0.5));
	const REAL c = REAL(faceSize - 1) * REAL(0.5);
	Matrix3x3 K = Matrix3x3::IDENTITY;
	K(0,0) = f;
	K(1,1) = f;
	K(0,2) = c;
	K(1,2) = c;
	return K;
}
/*----------------------------------------------------------------*/


// ------------------------------------------------------------------
// Public API — split geometry / rendering
// ------------------------------------------------------------------

SphereCubeMap::TangentFacesGeometry SFM::SphereCubeMap::MakeTangentFacesGeometry(
	int numFaces, int faceSize)
{
	TangentFacesGeometry g;
	g.rotations = FaceRotations(numFaces);
	if (g.rotations.empty() || faceSize <= 0)
		return g; // invalid — leave numFaces=0 sentinel
	g.K        = FaceIntrinsics(faceSize, numFaces);
	g.numFaces = static_cast<int>(g.rotations.size());
	g.faceSize = faceSize;
	return g;
}


template <typename TYPE>
std::vector<TImage<TYPE>> SFM::SphereCubeMap::SphericalToTangentialFaces(
	const TImage<TYPE>& sphericalImage,
	const TangentFacesGeometry& geometry)
{
	ASSERT(geometry.numFaces > 0 && geometry.faceSize > 0);
	ASSERT((int)geometry.rotations.size() == geometry.numFaces);
	ASSERT(sphericalImage.width() > 0 && sphericalImage.height() > 0);

	const SphericalCamera sphCam(sphericalImage.size());
	const REAL f  = geometry.K(0,0);
	const REAL cx = geometry.K(0,2);
	const REAL cy = geometry.K(1,2);
	TImage<TYPE> sphericalImageWrapped;
	cv::copyMakeBorder(sphericalImage, sphericalImageWrapped, 0, 0, 0, 1, cv::BORDER_WRAP);

	std::vector<TImage<TYPE>> images(geometry.numFaces);
	for (int k = 0; k < geometry.numFaces; ++k)
		RenderFaceKernel(sphericalImageWrapped, sphericalImage.width(), sphCam, geometry.rotations[k],
		                 f, cx, cy, geometry.faceSize, images[k]);

	#ifdef SFM_DEBUG_SPHERICAL_FACES
	// Debug: save rendered faces to disk for visual inspection.
	// The output images look flipped vertically (sky at bottom, ground at top for face 0)
	// because SphericalCamera is Y-UP but MVS's pinhole formula assumes Y-DOWN;
	// this is intentional and self-consistent so that the raw pixel coordinates
	// match where each sampled equirect direction was written.
	for (int k = 0; k < geometry.numFaces; ++k) {
		String fileName = MAKE_PATH(String::FormatString("spherical_face_%02d.png", k));
		if (!images[k].Save(fileName))
			DEBUG("Warning: failed to save debug face image: %s", fileName.c_str());
	}
	#endif

	return images;
}


// Explicit instantiations — one line per supported pixel type. Add new
// rows here if a caller needs Pixel16U, Pixel64F, etc.
template std::vector<Image8U3>
	SFM::SphereCubeMap::SphericalToTangentialFaces<Pixel8U>(
	const Image8U3&, const TangentFacesGeometry&);
template std::vector<Image32F3>
	SFM::SphereCubeMap::SphericalToTangentialFaces<Pixel32F>(
	const Image32F3&, const TangentFacesGeometry&);
/*----------------------------------------------------------------*/
