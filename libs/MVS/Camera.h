/*
* Camera.h
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
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
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#ifndef _MVS_CAMERA_H_
#define _MVS_CAMERA_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// a camera is represented as:
//   P = KR[I|-C]
// where R and C represent the camera orientation and position relative to the world coordinate system;
// R is expressed as the rotation from world to camera coordinates
// C is expressed as the explicit camera center in world coordinates
// (as opposite to standard form t which is negated and has the rotation of the camera already applied P = K[R|t]);
// the world and camera coordinates system is right handed,
// with x pointing right, y pointing down, and z pointing forward
// (see: R. Hartley, "Multiple View Geometry," 2004, pp. 156.);
// the projection in image coordinates uses the convention that the center of a pixel is defined at integer coordinates,
// i.e. the center is at (0, 0) and the top left corner is at (-0.5, -0.5);
// the projection model (Math/Camera/CameraModels.h) gives the meaning of K and of a depth:
//  - PINHOLE: K is the pinhole camera matrix, the depth of a point is its z in the camera frame
//  - BROWN: the pinhole camera of the undistorted image files (the lens is the platform camera's)
//  - SPHERICAL: an equirectangular panorama, K is the angular camera matrix (fx=W/2pi, fy=H/pi),
//    the depth of a point is its distance to the camera center and the columns wrap around
class MVS_API CameraIntern
{
public:
	KMatrix K; // the intrinsic camera parameters (3x3)
	RMatrix R; // rotation (3x3) and
	CMatrix C; // translation (3,1), the extrinsic camera parameters
	CameraModelType model; // the projection model

public:
	inline CameraIntern() : model(CameraModelType::PINHOLE) {}
	inline CameraIntern(const Matrix3x3& _R, const Point3& _C) : R(_R), C(_C), model(CameraModelType::PINHOLE) {}
	inline CameraIntern(const Matrix3x3& _K, const Matrix3x3& _R, const Point3& _C) : K(_K), R(_R), C(_C), model(CameraModelType::PINHOLE) {}

	// the camera is an equirectangular panorama (else a pinhole camera: Brown images are read undistorted)
	inline bool IsSpherical() const { return model == CameraModelType::SPHERICAL; }

	// the camera model of Math/Camera/CameraModels.h of the given type (Pinhole<T> or Spherical<T>),
	// parametrized by the focal lengths and the principal point of K; the type must be this camera's model
	template<typename MODEL>
	inline MODEL Model() const {
		typedef typename MODEL::Scalar T;
		ASSERT((std::is_same<MODEL, Spherical<T>>::value == IsSpherical()));
		ASSERT(ISZERO(K(0,1)));
		return MODEL{T(K(0,0)), T(K(1,1)), T(K(0,2)), T(K(1,2))};
	}
	// call f with the camera model in scalar T (Pinhole<T> or Spherical<T>) and return its result:
	// code generic over the model is written once, as a template on it, and the model is chosen once
	// per camera instead of once per pixel (as the GPU kernels do); both calls must return the same type
	template<typename T=REAL, typename FUNCTOR>
	inline decltype(auto) VisitModel(FUNCTOR&& f) const {
		if (IsSpherical())
			return f(Model<Spherical<T>>());
		return f(Model<Pinhole<T>>());
	}

	inline void SetT(const CMatrix& T) { C = R.t()*(-T); }
	inline CMatrix GetT() const { return R*(-C); }

	// returns the camera's view forward direction
	inline Point3 Direction() const { return R.row(2); /* equivalent to R.t() * Vec(0,0,1) */ }
	// returns the camera's view up direction
	inline Point3 UpDirection() const { return -R.row(1); /* equivalent to R.t() * Vec(0,-1,0) */ }

	// returns the focal length
	inline REAL GetFocalLength() const { return K(0,0); }
	// returns the focal length aspect ratio
	inline REAL GetFocalLengthRatio() const { return (K(1,1) / K(0,0)); }

	// returns the principal-point
	inline Point2 GetPrincipalPoint() const { return Point2(K(0,2), K(1,2)); }

	// update camera parameters given the delta
	inline void UpdateTranslation(const Point3& delta) {
		C += delta;
	}
	// update the camera rotation with the given delta (axis-angle)
	inline void UpdateRotation(const Point3& delta) {
		R.Apply((const Vec3&)delta);
	}
	inline void UpdateFocalLengthAbs(const REAL& f) {
		const REAL ratio = GetFocalLengthRatio();
		K(0,0) = f;
		K(1,1) = ratio * f;
	}
	inline void UpdateFocalLength(const REAL& df) {
		UpdateFocalLengthAbs(K(0,0)+df);
	}
	inline void UpdatePrincipalPoint(const Point2& delta) {
		K(0,2) += delta.x;
		K(1,2) += delta.y;
	}

	// returns the scale used to normalize the intrinsics
	static inline float GetNormalizationScale(uint32_t width, uint32_t height) {
		ASSERT(width>0 && height>0);
		return float(MAXF(width, height));
	}

	// create K with the supplied focal length and sensor center
	template<typename TYPE, typename TYPER>
	static inline TMatrix<TYPE,3,3> ComposeK(const TYPE& fX, const TYPE& fY, TYPER w=TYPER(1), TYPER h=TYPER(1)) {
		ASSERT(w>0 && h>0);
		TMatrix<TYPE,3,3> K(TMatrix<TYPE,3,3>::IDENTITY);
		K(0,0) = fX;
		K(1,1) = fY;
		K(0,2) = TYPE(0.5)*(w-1);
		K(1,2) = TYPE(0.5)*(h-1);
		return K;
	}
	template<typename TYPE, typename TYPER>
	static inline TMatrix<TYPE,3,3> ComposeInvK(const TYPE& fX, const TYPE& fY, TYPER w=TYPER(1), TYPER h=TYPER(1)) {
		ASSERT(w>0 && h>0);
		TMatrix<TYPE,3,3> invK(TMatrix<TYPE,3,3>::IDENTITY);
		invK(0,0) = TYPE(1)/fX;
		invK(1,1) = TYPE(1)/fY;
		invK(0,2) = TYPE(-0.5)*invK(0,0)*(w-1);
		invK(1,2) = TYPE(-0.5)*invK(1,1)*(h-1);
		return invK;
	}

	// scale image pixel coordinates with the given scale such that it accounts for
	// the convention that the center of a pixel is defined at integer coordinates
	template<typename TYPE>
	static inline TPoint2<TYPE> ScaleImagePixel(const TPoint2<TYPE>& x, TYPE s) {
		return TPoint2<TYPE>(
			(x.x+TYPE(0.5))*s-TYPE(0.5),
			(x.y+TYPE(0.5))*s-TYPE(0.5)
		);
	}

	// return scaled K (assuming standard K format)
	inline KMatrix GetScaledK(REAL s) const {
		return ScaleK(K, s);
	}
	// same as above, but for different scale on x and y;
	// in order to preserve the aspect ratio of the original size, scale both focal lengths by
	// the smaller of the scale factors, resulting in adding pixels in the dimension that's growing;
	inline KMatrix GetScaledK(const cv::Size& size, const cv::Size& newSize, bool keepAspect=false) const {
		return ScaleK(K, size, newSize, keepAspect);
	}

	// return K.inv() (assuming standard K format and no shear)
	template<typename TYPE>
	static inline TMatrix<TYPE,3,3> InvK(const TMatrix<TYPE,3,3>& K) {
		ASSERT(ISZERO(K(0,1)));
		TMatrix<TYPE,3,3> invK(TMatrix<TYPE,3,3>::IDENTITY);
		invK(0,0) = REAL(1)/K(0,0);
		invK(1,1) = REAL(1)/K(1,1);
		invK(0,2) = -K(0,2)*invK(0,0);
		invK(1,2) = -K(1,2)*invK(1,1);
		return invK;
	}
	inline KMatrix GetInvK() const {
		return InvK(K);
	}

	// return full K and the inverse of K (assuming standard K format);
	// the given resolution must be of the same aspect ratio as the normalized camera
	template<typename TYPE>
	inline TMatrix<TYPE,3,3> GetK(uint32_t width, uint32_t height) const {
		ASSERT(width>0 && height>0);
		const float scale(GetNormalizationScale(width, height));
		if (K(0,2) != 0 || K(1,2) != 0)
			return GetScaledK(scale);
		ASSERT(ISZERO(K(0,1)));
		return ComposeK(
			TYPE(K(0,0)*scale), TYPE(K(1,1)*scale),
			width, height );
	}
	template<typename TYPE>
	inline TMatrix<TYPE,3,3> GetInvK(uint32_t width, uint32_t height) const {
		ASSERT(width>0 && height>0);
		const float scale(GetNormalizationScale(width, height));
		if (K(0,2) != 0 || K(1,2) != 0)
			return InvK(GetScaledK(scale));
		ASSERT(ISZERO(K(0,1)));
		return ComposeInvK(
			TYPE(K(0,0)*scale), TYPE(K(1,1)*scale),
			width, height );
	}

	// return the OpenGL projection matrix corresponding to K:
	// - flip: if true, flip the y axis to match OpenGL image convention
	template<typename TYPE>
	static inline TMatrix<TYPE,4,4> ProjectionMatrixOpenGL(const TMatrix<TYPE,3,3>& K, const cv::Size& size, TYPE nearZ, TYPE farZ, bool flip = true) {
		// based on https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
		const TYPE fx(K(0,0)), fy(K(1,1));
		const TYPE cx(K(0,2)+0.5f), cy(K(1,2)+0.5f);
		const TYPE skew(K(0,1));
		const TYPE ihw(TYPE(2)/size.width), ihh(TYPE(2)/size.height);
		const TYPE iy(flip ? TYPE(-1) : TYPE(1));
		const TYPE ilen(TYPE(1)/(farZ-nearZ));
		return TMatrix<TYPE,4,4>(
			fx*ihw, skew*ihw, cx*ihw-TYPE(1), 0,
			0, iy*fy*ihh, iy*(cy*ihh-TYPE(1)), 0,
			0, 0, (farZ+nearZ)*ilen, -TYPE(2)*farZ*nearZ*ilen,
			0, 0, 1, 0);
	}
	inline Matrix4x4 GetProjectionMatrixOpenGL(const cv::Size& size, REAL nearZ, REAL farZ, bool flipY = true) const {
		ASSERT(!IsSpherical());
		return ProjectionMatrixOpenGL(K, size, nearZ, farZ, flipY);
	}

	// normalize inhomogeneous 2D point by the given camera intrinsics K
	// K is assumed to be the [3,3] triangular matrix with: fx, fy, s, cx, cy and scale 1
	template <typename TYPE>
	inline TPoint2<TYPE> NormalizeProjection(const TPoint2<TYPE>& proj) const {
		ASSERT(!IsSpherical());
		TPoint2<TYPE> pt;
		NormalizeProjectionInv(GetInvK<TYPE>(), proj.ptr(), pt.ptr());
		return pt;
	}

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & K;
		ar & R;
		ar & C;
		uint8_t modelID(static_cast<uint8_t>(model));
		ar & modelID;
		model = static_cast<CameraModelType>(modelID);
	}
	#endif
};
/*----------------------------------------------------------------*/


// same as above, plus caching the projection matrix
class MVS_API Camera : public CameraIntern
{
public:
	PMatrix P; // the composed projection matrix (3x4)

public:
	static const Camera IDENTITY;

public:
	inline Camera() {}
	inline Camera(const CameraIntern& camera) : CameraIntern(camera) {}
	Camera(const Matrix3x4& _P, bool bUpdate=true);
	Camera(const Matrix3x3& _R, const Point3& _C, bool bUpdate=true);
	Camera(const Matrix3x3& _K, const Matrix3x3& _R, const Point3& _C, bool bUpdate=true);

	Camera& operator= (const CameraIntern& camera);

	Camera GetScaled(REAL s) const; // return a camera scaled by the given factor
	Camera GetScaled(const cv::Size& size, const cv::Size& newSize) const; // return a camera scaled to the given resolution

	Matrix4x4 GetP() const; // the composed projection matrix (4x4) assuming valid P
	Matrix4x4 GetRC() const; // the composed transform matrix (4x4)
	void ComposeP_RC(); // compose P from R and C only
	void ComposeP(); // compose P from K, R and C
	void DecomposeP_RC(); // decompose P in R and C, keep K unchanged
	void DecomposeP(); // decompose P in K, R and C
	void Transform(const Matrix3x3& R, const Point3& t, const REAL& s); // transform camera by the given similarity transform

	REAL PointDepth(const Point3& X) const; // computes the depth of the given 3D point seen from this camera
	bool IsInFront(const Point3& X) const; // test if the given 3D point is in front of the camera
	REAL DistanceSq(const Point3& X) const; // compute the distance from the camera to the given 3D point
	inline REAL Distance(const Point3& X) const { return SQRT(DistanceSq(X)); }

	static REAL StereoRectify(const cv::Size& size1, const Camera& camera1, const cv::Size& size2, const Camera& camera2, Matrix3x3& R1, Matrix3x3& R2, Matrix3x3& K1, Matrix3x3& K2);
	static REAL StereoRectifyFusiello(const cv::Size& size1, const Camera& camera1, const cv::Size& size2, const Camera& camera2, Matrix3x3& R1, Matrix3x3& R2, Matrix3x3& K1, Matrix3x3& K2);
	static void SetStereoRectificationROI(const Point3fArr& points1, cv::Size& size1, const Camera& camera1, const Point3fArr& points2, cv::Size& size2, const Camera& camera2, const Matrix3x3& R1, const Matrix3x3& R2, Matrix3x3& K1, Matrix3x3& K2);

	// project 3D point by the camera;
	// the functions below follow the projection model: a pixel is the projection of the camera model, a
	// depth the distance along its ray (z for a pinhole, the range for a spherical camera), and the
	// functions of the pinhole image plane (z=1, the P matrix, the view plane) assert a pinhole camera
	template <typename TYPE>
	inline TPoint3<TYPE> ProjectPointRT3(const TPoint3<TYPE>& X) const {
		return R * (X - C);
	}
	template <typename TYPE>
	inline TPoint2<TYPE> ProjectPointRT(const TPoint3<TYPE>& X) const {
		ASSERT(!IsSpherical());
		const TPoint3<TYPE> q(R * (X - C));
		const TYPE invZ(INVERT(q.z));
		return TPoint2<TYPE>(q.x*invZ, q.y*invZ);
	}
	// pixel and range of a world point seen by a spherical camera
	template <typename TYPE>
	inline std::tuple<TPoint2<TYPE>,TYPE> ProjectPointSpherical(const TPoint3<TYPE>& X) const {
		ASSERT(IsSpherical());
		const auto [x, depth] = ProjectPointC(TransformPointW2C(Cast<REAL>(X)));
		return {TPoint2<TYPE>(x), TYPE(depth)};
	}
	// pixel and depth of a world point; the depth is not positive if the point is behind a pinhole camera
	template <typename TYPE>
	inline std::tuple<TPoint2<TYPE>,TYPE> ProjectPoint(const TPoint3<TYPE>& X) const {
		if (IsSpherical())
			return ProjectPointSpherical(X);
		const TPoint3<TYPE> q(K * (R * (X - C)));
		const TYPE invZ(INVERT(q.z));
		return {TPoint2<TYPE>(q.x*invZ, q.y*invZ), q.z};
	}
	template <typename TYPE>
	inline TPoint3<TYPE> ProjectPointP3(const TPoint3<TYPE>& X) const {
		ASSERT(!IsSpherical());
		const REAL* const p(P.val);
		return TPoint3<TYPE>(
			(TYPE)(p[0*4+0]*X.x + p[0*4+1]*X.y + p[0*4+2]*X.z + p[0*4+3]),
			(TYPE)(p[1*4+0]*X.x + p[1*4+1]*X.y + p[1*4+2]*X.z + p[1*4+3]),
			(TYPE)(p[2*4+0]*X.x + p[2*4+1]*X.y + p[2*4+2]*X.z + p[2*4+3]));
	}
	// the same as ProjectPoint, through the P matrix for a pinhole camera
	template <typename TYPE>
	inline std::tuple<TPoint2<TYPE>,TYPE> ProjectPointP(const TPoint3<TYPE>& X) const {
		if (IsSpherical())
			return ProjectPointSpherical(X);
		const TPoint3<TYPE> q(ProjectPointP3(X));
		const TYPE invZ(INVERT(q.z));
		return {TPoint2<TYPE>(q.x*invZ, q.y*invZ), q.z};
	}
	// pixel and depth of a camera-frame point; the depth is not positive if the point is behind a
	// pinhole camera or at the center of a spherical one
	inline std::tuple<Point2,REAL> ProjectPointC(const Point3& Xc) const {
		if (!IsSpherical())
			return {TransformPointC2I(Point2(Xc.x/Xc.z, Xc.y/Xc.z)), Xc.z};
		const Eigen::Vector3d X(Xc);
		Eigen::Vector2d x;
		Model<Spherical<REAL>>().Project(X, x);
		return {Point2(x), Spherical<REAL>::Depth(X)};
	}

	// transform from image pixel coords to view plane coords
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointI2V(const TPoint2<TYPE>& x) const {
		ASSERT(!IsSpherical());
		return TPoint2<TYPE>(
			TYPE(x.x-K(0,2)),
			TYPE(x.y-K(1,2)) );
	}
	// un-project from image pixel coords to the camera space z=1 plane
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointI2C(const TPoint2<TYPE>& x) const {
		ASSERT(!IsSpherical());
		return TPoint3<TYPE>(
			TYPE((x.x-K(0,2))/K(0,0)),
			TYPE((x.y-K(1,2))/K(1,1)),
			TYPE(1) );
	}
	// un-project from image pixel coords (x,y) and depth z to the camera space
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointI2C(const TPoint3<TYPE>& X) const {
		if (IsSpherical()) {
			ASSERT(X.z > 0);
			return Cast<TYPE>(Point3(Model<Spherical<REAL>>().Ray(Point2(X.x, X.y))*REAL(X.z)));
		}
		return TPoint3<TYPE>(
			TYPE((X.x-K(0,2))*X.z/K(0,0)),
			TYPE((X.y-K(1,2))*X.z/K(1,1)),
			X.z );
	}
	// camera-frame point of pixel x at the given depth
	inline Point3 TransformPointI2C(const Point2& x, REAL depth) const {
		return TransformPointI2C(Point3(x.x, x.y, depth));
	}
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointC2W(const TPoint3<TYPE>& X) const {
		return R.t() * X + C;
	}
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointI2W(const TPoint2<TYPE>& x) const {
		return TransformPointC2W(TransformPointI2C(x));
	}
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointI2W(const TPoint3<TYPE>& X) const {
		return TransformPointC2W(TransformPointI2C(X));
	}
	// world point of pixel x at the given depth
	inline Point3 TransformPointI2W(const Point2& x, REAL depth) const {
		return TransformPointC2W(TransformPointI2C(x, depth));
	}
	// world-frame direction of the ray of pixel x (not normalized)
	template <typename TYPE>
	inline TPoint3<TYPE> RayPoint(const TPoint2<TYPE>& x) const {
		if (IsSpherical())
			return R.t() * TransformPointI2C(TPoint3<TYPE>(x.x, x.y, TYPE(1)));
		return R.t() * TransformPointI2C(x);
	}
	template <typename TYPE>
	inline TPoint3<TYPE> RayPointP(const TPoint2<TYPE>& x) const {
		ASSERT(!IsSpherical());
		TPoint3<TYPE> ray;
		RayPoint_3x4_2_3(P.val, x.ptr(), ray.ptr());
		return ray;
	}

	// project from the camera z=1 plane to image pixels
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointC2I(const TPoint2<TYPE>& x) const {
		ASSERT(!IsSpherical());
		return TPoint2<TYPE>(
			TYPE(K(0,2)+K(0,0)*x.x),
			TYPE(K(1,2)+K(1,1)*x.y) );
	}
	// project from the camera space to view plane
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointC2V(const TPoint3<TYPE>& X) const {
		ASSERT(!IsSpherical());
		return TPoint2<TYPE>(
			TYPE(K(0,0)*X.x/X.z),
			TYPE(K(1,1)*X.y/X.z) );
	}
	// project from the camera space to image pixels
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointC2I(const TPoint3<TYPE>& X) const {
		if (IsSpherical())
			return TPoint2<TYPE>(std::get<0>(ProjectPointC(Cast<REAL>(X))));
		return TransformPointC2I(TPoint2<TYPE>(X.x/X.z, X.y/X.z));
	}
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointW2C(const TPoint3<TYPE>& X) const {
		return R * (X - C);
	}
	// pixel of a world point in front of the camera
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointW2I(const TPoint3<TYPE>& X) const {
		return TransformPointC2I(TransformPointW2C(X));
	}
	// pixel and depth of a world point
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointW2I3(const TPoint3<TYPE>& X) const {
		if (IsSpherical()) {
			const auto [x, depth] = ProjectPointSpherical(X);
			return TPoint3<TYPE>(x.x, x.y, depth);
		}
		const TPoint3<TYPE> camX(TransformPointW2C(X));
		return TPoint3<TYPE>(TransformPointC2I(camX), camX.z);
	}

	// check if the given point (or its projection) is inside the camera view
	template <typename TYPE1, typename TYPE2>
	inline bool IsInside(const TPoint2<TYPE1>& pt, const TPoint2<TYPE2>& size) const {
		return pt.x>=0 && pt.y>=0 && pt.x<size.x && pt.y<size.y;
	}
	template <typename TYPE1, typename TYPE2>
	inline bool IsInsideProjection(const TPoint3<TYPE1>& X, const TPoint2<TYPE2>& size) const {
		const auto [x, depth] = ProjectPoint(X);
		if (depth <= 0)
			return false;
		return IsInside(x, size);
	}
	template <typename TYPE1, typename TYPE2>
	inline bool IsInsideProjectionP(const TPoint3<TYPE1>& X, const TPoint2<TYPE2>& size) const {
		const auto [x, depth] = ProjectPointP(X);
		if (depth <= 0)
			return false;
		return IsInside(x, size);
	}
	// the position of a world point inside an image of the given size, in [0,size): false if it falls
	// outside or behind a pinhole camera; a spherical camera sees every direction, but its projection
	// spans [-0.5,size-0.5), so the point takes its pixel there, the column wrapped
	template <typename TYPE>
	inline bool ProjectPointInside(const TPoint3<TYPE>& X, const cv::Size& size, Point2f& x) const {
		const auto [pt, depth] = ProjectPointP(X);
		if (!IsSpherical()) {
			x = Point2f(pt);
			return depth > 0 && IsInside(x, Point2f(size));
		}
		ImageRef ir;
		if (depth <= 0 || !GetPixel(pt, size, ir))
			return false;
		x = Point2f((float)ir.x, (float)ir.y);
		return true;
	}
	// pixel ir of an image of the given size, its column wrapped for a spherical camera (ir.x within one
	// width of the image); false if outside the image
	inline bool WrapPixel(ImageRef& ir, const cv::Size& size) const {
		if (IsSpherical())
			ir.x = WrapColumn(ir.x, size.width);
		return Image8U::isInside(ir, size);
	}
	// the pixel of an image of the given size closest to x, its column wrapped for a spherical camera;
	// false if outside the image
	template <typename TYPE>
	inline bool GetPixel(const TPoint2<TYPE>& x, const cv::Size& size, ImageRef& ir) const {
		ir = ROUND2INT(x);
		return WrapPixel(ir, size);
	}
	// the column offset a-b, the shortest one around the panorama for a spherical camera
	template <typename TYPE>
	inline TYPE ColumnOffset(TYPE a, TYPE b, int width) const {
		const TYPE d(a - b);
		if (!IsSpherical())
			return d;
		return d > TYPE(width)/2 ? d - TYPE(width) : (d < -TYPE(width)/2 ? d + TYPE(width) : d);
	}

	// same as above, but for ortho-projection
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointOrthoI2C(const TPoint3<TYPE>& x) const {
		return TPoint3<TYPE>(
			TYPE((x.x-K(0,2))/K(0,0)),
			TYPE((x.y-K(1,2))/K(1,1)),
			x.z );
	}
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointOrthoI2W(const TPoint3<TYPE>& x) const {
		return TransformPointC2W(TransformPointOrthoI2C(x));
	}
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointOrthoC2I(const TPoint3<TYPE>& X) const {
		return TransformPointC2I(TPoint2<TYPE>(X.x, X.y));
	}
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointOrthoW2I(const TPoint3<TYPE>& X) const {
		return TransformPointOrthoC2I(TransformPointW2C(X));
	}

	// compute the projection scale in this camera of the given world point
	template <typename TYPE>
	inline TYPE GetFootprintImage(TYPE depth) const {
		return static_cast<TYPE>(GetFocalLength() / depth);
	}
	template <typename TYPE>
	inline TYPE GetFootprintImage(const TPoint3<TYPE>& X) const {
		return GetFootprintImage(PointDepth(X));
	}
	// compute the surface the projected pixel covers at the given depth
	template <typename TYPE>
	inline TYPE GetFootprintWorld(TYPE depth) const {
		return static_cast<TYPE>(depth / GetFocalLength());
	}
	// same as above, but the 3D point is given
	template <typename TYPE>
	inline TYPE GetFootprintWorld(const TPoint3<TYPE>& X) const {
		return GetFootprintWorld(PointDepth(X));
	}

	// create the 4 image points corresponding to the image corners
	Point2Arr GetImageCorners(const cv::Size& size) const {
		const int maxX = size.width - 1;
		const int maxY = size.height - 1;
		return Point2Arr{
			Point2(0, 0),
			Point2(0, maxY),
			Point2(maxX, maxY),
			Point2(maxX, 0)
		};
	}
	// compute the normalized rays in camera space corresponding to the image corners
	Point3Arr GetCameraCornerRays(const cv::Size& size, bool bNormalize=true) const {
		const Point2Arr corners(GetImageCorners(size));
		Point3Arr result(4);
		for (int i = 0; i < 4; ++i) {
			result[i] = RayPoint(corners[i]);
			if (bNormalize)
				normalize(result[i]);
		}
		return result;
	}

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const {
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MVS::CameraIntern);
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/) {
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MVS::CameraIntern);
		ComposeP();
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
};
typedef CLISTDEF0IDX(Camera,uint32_t) CameraArr;
/*----------------------------------------------------------------*/

MVS_API Point3 ComputeCamerasFocusPoint(const CameraArr& cameras, const Point3* pInitialFocus=NULL);
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_CAMERA_H_
