////////////////////////////////////////////////////////////////////
// CameraUndistort.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "../Common.h"
#include "CameraUndistort.h"
#include <array>
#include <map>
#include <mutex>

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable save rendered faces to disk for visual inspection (debug only)
//#define SPHERECUBEMAP_DEBUG_FACES


// S T R U C T S ///////////////////////////////////////////////////

KMatrix SEACAVE::UndistortedK(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size, double alpha)
{
	ASSERT(size.width > 0 && size.height > 0);
	ASSERT(K(0,0) > 0 && K(1,1) > 0);
	ASSERT(alpha >= 0 && alpha <= 1);
	const cv::Vec<double,8> coeffs(distortion.k1, distortion.k2, distortion.p1, distortion.p2,
		distortion.k3, distortion.k4, distortion.k5, distortion.k6);
	return KMatrix(cv::getOptimalNewCameraMatrix(K, coeffs, size, alpha));
}

void UndistortMaps::Init(const KMatrix& _K, const BrownDistortion<REAL>& distortion, const cv::Size& size, const KMatrix& newK)
{
	ASSERT(size.width > 0 && size.height > 0);
	ASSERT(_K(0,0) > 0 && _K(1,1) > 0 && newK(0,0) > 0 && newK(1,1) > 0);
	const cv::Vec<double,8> coeffs(distortion.k1, distortion.k2, distortion.p1, distortion.p2,
		distortion.k3, distortion.k4, distortion.k5, distortion.k6);
	K = newK;
	cv::initUndistortRectifyMap(_K, coeffs, cv::noArray(), K, size, CV_16SC2, map1, map2);
}
void UndistortMaps::Init(const KMatrix& _K, const BrownDistortion<REAL>& distortion, const cv::Size& size, double alpha)
{
	Init(_K, distortion, size, UndistortedK(_K, distortion, size, alpha));
}

void UndistortMaps::Remap(cv::InputArray src, cv::OutputArray dst, int interpolation) const
{
	ASSERT(!map1.empty() && !src.empty());
	cv::remap(src, dst, map1, map2, interpolation, cv::BORDER_CONSTANT, cv::Scalar::all(0));
}

void UndistortMaps::RemapDepth(const Image32F& depthMap, Image32F& undistortedDepthMap) const
{
	ASSERT(!map1.empty() && !depthMap.empty());
	undistortedDepthMap.create(map1.size());
	undistortedDepthMap.setTo(cv::Scalar(0));
	// for each undistorted pixel, sample the source depth-map at the position given by the remap tables
	for (int y = 0; y < undistortedDepthMap.rows; ++y) {
		for (int x = 0; x < undistortedDepthMap.cols; ++x) {
			const Point2f srcPt = CoordinateRemap2Float(cv::Point2i(x, y), map1, map2);
			if (!depthMap.isInsideWithBorder(srcPt, 1))
				continue;
			// interpolate only between depths similar to the nearest one
			const float centerDepth = depthMap(ROUND2INT(srcPt));
			if (centerDepth <= 0.f)
				continue;
			const auto Sample = [centerDepth](const float& depth) {
				return IsDepthSimilar(centerDepth, depth);
			};
			float sampledDepth;
			if (!depthMap.sampleSafe(sampledDepth, srcPt, Sample))
				continue;
			undistortedDepthMap(y, x) = sampledDepth;
		}
	}
}

namespace {

// the cache behind GetCameraUndistortMaps
class CameraUndistortCache {
public:
	typedef std::array<REAL,15> Key; // K (fx, skew, cx, fy, cy), the 8 coefficients, the source size

	std::shared_ptr<const CameraUndistortMaps> Get(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size) {
		const Key key{K(0,0), K(0,1), K(0,2), K(1,1), K(1,2),
			distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3, distortion.k4, distortion.k5, distortion.k6,
			REAL(size.width), REAL(size.height)};
		std::shared_ptr<Entry> entry;
		{
			const std::lock_guard<std::mutex> lock(mtx);
			std::shared_ptr<Entry>& slot = entries[key];
			if (slot == NULL)
				slot = std::make_shared<Entry>();
			entry = slot;
			entry->lastUse = ++useClock;
		}
		std::call_once(entry->once, [&]() {
			const std::shared_ptr<CameraUndistortMaps> maps(std::make_shared<CameraUndistortMaps>());
			maps->Init(K, distortion, size, UNDISTORT_ALPHA);
			// a source footprint fully inside the source image interpolates 255 from the all-255 image
			Image8U source(size), remapped;
			source.setTo(cv::Scalar(255));
			maps->Remap(source, remapped, cv::INTER_LINEAR);
			cv::compare(remapped, cv::Scalar(255), maps->mask, cv::CMP_EQ);
			const std::lock_guard<std::mutex> lock(mtx);
			entry->bytes = maps->map1.total()*maps->map1.elemSize() + maps->map2.total()*maps->map2.elemSize() + maps->mask.total();
			entry->maps = maps;
		});
		const std::lock_guard<std::mutex> lock(mtx);
		ASSERT(entry->maps != NULL);
		size_t bytes(0);
		for (const auto& slot: entries)
			bytes += slot.second->bytes;
		while (bytes > budget) {
			auto itOldest = entries.end();
			for (auto it = entries.begin(); it != entries.end(); ++it)
				if (it->second != entry && it->second->maps != NULL &&
					(itOldest == entries.end() || it->second->lastUse < itOldest->second->lastUse))
					itOldest = it;
			if (itOldest == entries.end())
				break;
			bytes -= itOldest->second->bytes;
			entries.erase(itOldest);
		}
		return entry->maps;
	}

private:
	struct Entry {
		std::once_flag once;
		std::shared_ptr<const CameraUndistortMaps> maps; // set once by the first request
		size_t bytes = 0; // memory of the maps, 0 until computed
		uint64_t lastUse = 0;
	};
	static constexpr size_t budget = size_t(1) << 30;
	std::mutex mtx;
	std::map<Key, std::shared_ptr<Entry>> entries;
	uint64_t useClock = 0;
};

} // namespace

std::shared_ptr<const CameraUndistortMaps> SEACAVE::GetCameraUndistortMaps(const KMatrix& K, const BrownDistortion<REAL>& distortion, const cv::Size& size)
{
	static CameraUndistortCache cache;
	return cache.Get(K, distortion, size);
}
/*----------------------------------------------------------------*/


namespace {

// 6-face cube rotation table; face order: +Z forward, -Z back, +X right, -X left, +Y, -Y.
// All rotations are proper (det=+1) and map the panorama frame to a face frame whose +Z points
// along the named axis; the face frames keep the y-down convention of the Spherical model, so the
// rendered faces are upright (sky at top for face 0).
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
	// Face 4: +Y → body +Y = face +Z. R = Rx(+90°).
	R[4] = Matrix3x3::ZERO;
	R[4](0,0) = 1; R[4](1,2) = -1; R[4](2,1) = 1;
	// Face 5: -Y → body -Y = face +Z. R = Rx(-90°).
	R[5] = Matrix3x3::ZERO;
	R[5](0,0) = 1; R[5](1,2) = 1; R[5](2,1) = -1;
	return R;
}

const std::array<Matrix3x3, 6>& GetFaceRotations6()
{
	static const std::array<Matrix3x3, 6> kRotations = BuildFaceRotations6();
	return kRotations;
}

// body→face rotation for a unit "forward" direction n (where face +Z points, in the body frame);
// Gram–Schmidt against the body +Y (or +X when n is nearly parallel to +Y);
// R has rows [right^T; up_face^T; n^T] so R * n_body = (0,0,1);
// used by the 8/12/20-face tables, the 6-face one is the explicit table above
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

// render one face: each face pixel ray is projected into the panorama and sampled bilinearly;
// sourceWrapped carries one extra column (a copy of column 0) so samples across the seam interpolate
template <typename TYPE>
void RenderFaceKernel(
	const TImage<TYPE>& sourceWrapped,
	const Spherical<REAL>& sphere,
	const Matrix3x3& R_face,
	REAL f, REAL cx, REAL cy,
	int faceSize,
	TImage<TYPE>& dst)
{
	const int srcW = sourceWrapped.width() - 1;
	const int srcH = sourceWrapped.height();
	ASSERT(srcW > 0 && srcH > 0);
	if (dst.cols != faceSize || dst.rows != faceSize)
		dst.create(faceSize, faceSize);

	const Matrix3x3 R_face_T = R_face.t();
	for (int v = 0; v < faceSize; ++v) {
		for (int u = 0; u < faceSize; ++u) {
			const Point3 b_body = R_face_T * Point3((REAL(u) - cx) / f, (REAL(v) - cy) / f, REAL(1));
			Eigen::Vector2d x;
			if (!sphere.Project(b_body, x))
				ASSERT("face rays are never zero" == NULL);
			// the projection spans [-0.5, srcW-0.5] horizontally: wrap the left half-pixel onto
			// the extra column, and clamp vertically to the first/last row centers
			REAL px = x.x();
			if (px < REAL(0)) px += REAL(srcW);
			const REAL py = CLAMP(x.y(), REAL(0), REAL(srcH - 1));
			dst(v, u) = sourceWrapped.sampleSafe(Point2(px, py));
		}
	}
}

} // namespace


std::vector<Matrix3x3> SphereCubeMap::FaceRotations(int n)
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

REAL SphereCubeMap::FaceFOVDegrees(int n)
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

Matrix3x3 SphereCubeMap::FaceIntrinsics(int faceSize, int n)
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


bool SphereCubeMap::IsSupportedNumFaces(int numFaces)
{
	return numFaces == 4 || numFaces == 6 || numFaces == 8 || numFaces == 12 || numFaces == 20;
}

SphereCubeMap::TangentFacesGeometry SphereCubeMap::MakeTangentFacesGeometry(int numFaces, int faceSize)
{
	ASSERT(IsSupportedNumFaces(numFaces) && faceSize > 0);
	TangentFacesGeometry g;
	g.rotations = FaceRotations(numFaces);
	g.K        = FaceIntrinsics(faceSize, numFaces);
	g.numFaces = numFaces;
	g.faceSize = faceSize;
	ASSERT((int)g.rotations.size() == numFaces);
	return g;
}

template <typename TYPE>
std::vector<TImage<TYPE>> SphereCubeMap::SphericalToTangentialFaces(
	const TImage<TYPE>& sphericalImage,
	const TangentFacesGeometry& geometry)
{
	ASSERT(geometry.numFaces > 0 && geometry.faceSize > 0);
	ASSERT((int)geometry.rotations.size() == geometry.numFaces);
	ASSERT(sphericalImage.width() > 0 && sphericalImage.height() > 0);

	const Spherical<REAL> sphere(Spherical<REAL>::FromSize(sphericalImage.width(), sphericalImage.height()));
	const REAL f  = geometry.K(0,0);
	const REAL cx = geometry.K(0,2);
	const REAL cy = geometry.K(1,2);
	TImage<TYPE> sphericalImageWrapped;
	cv::copyMakeBorder(sphericalImage, sphericalImageWrapped, 0, 0, 0, 1, cv::BORDER_WRAP);

	std::vector<TImage<TYPE>> images(geometry.numFaces);
	for (int k = 0; k < geometry.numFaces; ++k)
		RenderFaceKernel(sphericalImageWrapped, sphere, geometry.rotations[k],
		                 f, cx, cy, geometry.faceSize, images[k]);

	#ifdef SPHERECUBEMAP_DEBUG_FACES
	for (int k = 0; k < geometry.numFaces; ++k) {
		String fileName = MAKE_PATH(String::FormatString("spherical_face_%02d.png", k));
		if (!images[k].Save(fileName))
			DEBUG("Warning: failed to save debug face image: %s", fileName.c_str());
	}
	#endif

	return images;
}

// explicit instantiations, exported from the library; add a row per pixel type a caller needs
template MATH_API std::vector<Image8U3>
	SphereCubeMap::SphericalToTangentialFaces<Pixel8U>(
	const Image8U3&, const SphereCubeMap::TangentFacesGeometry&);
template MATH_API std::vector<Image32F3>
	SphereCubeMap::SphericalToTangentialFaces<Pixel32F>(
	const Image32F3&, const SphereCubeMap::TangentFacesGeometry&);
/*----------------------------------------------------------------*/
