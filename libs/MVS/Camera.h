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
// i.e. the center is at (0, 0) and the top left corner is at (-0.5, -0.5)
class MVS_API CameraIntern
{
public:
	KMatrix K; // the intrinsic camera parameters (3x3)
	RMatrix R; // rotation (3x3) and
	CMatrix C; // translation (3,1), the extrinsic camera parameters

public:
	inline CameraIntern() {}
	inline CameraIntern(const Matrix3x3& _R, const Point3& _C) : R(_R), C(_C) {}
	inline CameraIntern(const Matrix3x3& _K, const Matrix3x3& _R, const Point3& _C) : K(_K), R(_R), C(_C) {}

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
	template<typename TYPE>
	static inline TMatrix<TYPE,3,3> ScaleK(const TMatrix<TYPE,3,3>& K, TYPE s) {
		return TMatrix<TYPE,3,3>(
			K(0,0)*s, K(0,1)*s, (K(0,2)+TYPE(0.5))*s-TYPE(0.5),
			TYPE(0),  K(1,1)*s, (K(1,2)+TYPE(0.5))*s-TYPE(0.5),
			TYPE(0), TYPE(0), TYPE(1)
		);
	}
	inline KMatrix GetScaledK(REAL s) const {
		return ScaleK(K, s);
	}
	// same as above, but for different scale on x and y;
	// in order to preserve the aspect ratio of the original size, scale both focal lengths by
	// the smaller of the scale factors, resulting in adding pixels in the dimension that's growing;
	template<typename TYPE>
	static inline TMatrix<TYPE,3,3> ScaleK(const TMatrix<TYPE,3,3>& K, const cv::Size& size, const cv::Size& newSize, bool keepAspect=false) {
		ASSERT(size.area() && newSize.area());
		cv::Point_<TYPE> s(cv::Point_<TYPE>(newSize) / cv::Point_<TYPE>(size));
		if (keepAspect)
			s.x = s.y = MINF(s.x, s.y);
		return TMatrix<TYPE,3,3>(
			K(0,0)*s.x, K(0,1)*s.x, (K(0,2)+TYPE(0.5))*s.x-TYPE(0.5),
			TYPE(0),    K(1,1)*s.y, (K(1,2)+TYPE(0.5))*s.y-TYPE(0.5),
			TYPE(0),    TYPE(0),    TYPE(1)
		);
	}
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

	// returns full K and the inverse of K (assuming standard K format)
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
		return ProjectionMatrixOpenGL(K, size, nearZ, farZ, flipY);
	}

	// normalize inhomogeneous 2D point by the given camera intrinsics K
	// K is assumed to be the [3,3] triangular matrix with: fx, fy, s, cx, cy and scale 1
	template <typename TYPE>
	inline TPoint2<TYPE> NormalizeProjection(const TPoint2<TYPE>& proj) const {
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

	// project 3D point by the camera
	template <typename TYPE>
	inline TPoint3<TYPE> ProjectPointRT3(const TPoint3<TYPE>& X) const {
		return R * (X - C);
	}
	template <typename TYPE>
	inline TPoint2<TYPE> ProjectPointRT(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> q(R * (X - C));
		const TYPE invZ(INVERT(q.z));
		return TPoint2<TYPE>(q.x*invZ, q.y*invZ);
	}
	template <typename TYPE>
	inline TPoint2<TYPE> ProjectPoint(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> q(K * (R * (X - C)));
		const TYPE invZ(INVERT(q.z));
		return TPoint2<TYPE>(q.x*invZ, q.y*invZ);
	}
	template <typename TYPE>
	inline TPoint3<TYPE> ProjectPointP3(const TPoint3<TYPE>& X) const {
		const REAL* const p(P.val);
		return TPoint3<TYPE>(
			(TYPE)(p[0*4+0]*X.x + p[0*4+1]*X.y + p[0*4+2]*X.z + p[0*4+3]),
			(TYPE)(p[1*4+0]*X.x + p[1*4+1]*X.y + p[1*4+2]*X.z + p[1*4+3]),
			(TYPE)(p[2*4+0]*X.x + p[2*4+1]*X.y + p[2*4+2]*X.z + p[2*4+3]));
	}
	template <typename TYPE>
	inline TPoint2<TYPE> ProjectPointP(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> q(ProjectPointP3(X));
		const TYPE invZ(INVERT(q.z));
		return TPoint2<TYPE>(q.x*invZ, q.y*invZ);
	}

	// transform from image pixel coords to view plane coords
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointI2V(const TPoint2<TYPE>& x) const {
		return TPoint2<TYPE>(
			TYPE(x.x-K(0,2)),
			TYPE(x.y-K(1,2)) );
	}
	// un-project from image pixel coords to the camera space (z=1 plane by default)
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointI2C(const TPoint2<TYPE>& x) const {
		return TPoint3<TYPE>(
			TYPE((x.x-K(0,2))/K(0,0)),
			TYPE((x.y-K(1,2))/K(1,1)),
			TYPE(1) );
	}
	// un-project from image pixel coords to the camera space
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointI2C(const TPoint3<TYPE>& X) const {
		return TPoint3<TYPE>(
			TYPE((X.x-K(0,2))*X.z/K(0,0)),
			TYPE((X.y-K(1,2))*X.z/K(1,1)),
			X.z );
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
	template <typename TYPE>
	inline TPoint3<TYPE> RayPoint(const TPoint2<TYPE>& x) const {
		return R.t() * TransformPointI2C(x);
	}
	template <typename TYPE>
	inline TPoint3<TYPE> RayPointP(const TPoint2<TYPE>& x) const {
		TPoint3<TYPE> ray;
		RayPoint_3x4_2_3(P.val, x.ptr(), ray.ptr());
		return ray;
	}

	// project from the camera z=1 plane to image pixels
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointC2I(const TPoint2<TYPE>& x) const {
		return TPoint2<TYPE>(
			TYPE(K(0,2)+K(0,0)*x.x),
			TYPE(K(1,2)+K(1,1)*x.y) );
	}
	// project from the camera space to view plane
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointC2V(const TPoint3<TYPE>& X) const {
		return TPoint2<TYPE>(
			TYPE(K(0,0)*X.x/X.z),
			TYPE(K(1,1)*X.y/X.z) );
	}
	// project from the camera space to image pixels
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointC2I(const TPoint3<TYPE>& X) const {
		return TransformPointC2I(TPoint2<TYPE>(X.x/X.z, X.y/X.z));
	}
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointW2C(const TPoint3<TYPE>& X) const {
		return R * (X - C);
	}
	template <typename TYPE>
	inline TPoint2<TYPE> TransformPointW2I(const TPoint3<TYPE>& X) const {
		return TransformPointC2I(TransformPointW2C(X));
	}
	template <typename TYPE>
	inline TPoint3<TYPE> TransformPointW2I3(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> camX(TransformPointW2C(X));
		return TPoint3<TYPE>(TransformPointC2I(camX), camX.z);
	}

	// check if the given point (or its projection) is inside the camera view
	template <typename TYPE>
	inline bool IsInside(const TPoint2<TYPE>& pt, const TPoint2<TYPE>& size) const {
		return pt.x>=0 && pt.y>=0 && pt.x<size.x && pt.y<size.y;
	}
	template <typename TYPE>
	inline bool IsInsideProjection(const TPoint3<TYPE>& X, const TPoint2<TYPE>& size) const {
		return IsInside(ProjectPoint(X), size);
	}
	template <typename TYPE>
	inline bool IsInsideProjectionP(const TPoint3<TYPE>& X, const TPoint2<TYPE>& size) const {
		return IsInside(ProjectPointP(X), size);
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
	inline TYPE GetFootprintImage(const TPoint3<TYPE>& X) const {
		#if 0
		const TYPE fSphereRadius(1);
		const TPoint3<TYPE> camX(TransformPointW2C(X));
		return norm(TransformPointC2I(TPoint3<TYPE>(camX.x+fSphereRadius,camX.y,camX.z))-TransformPointC2I(camX));
		#else
		return static_cast<TYPE>(GetFocalLength() / PointDepth(X));
		#endif
	}
	// compute the surface the projected pixel covers at the given depth
	template <typename TYPE>
	inline TYPE GetFootprintWorldSq(const TPoint2<TYPE>& x, TYPE depth) const {
		#if 0
		return SQUARE(GetFocalLength());
		#else
		// improved version of the above
		return SQUARE(depth) / (SQUARE(GetFocalLength()) + normSq(TransformPointI2V(x)));
		#endif
	}
	template <typename TYPE>
	inline TYPE GetFootprintWorld(const TPoint2<TYPE>& x, TYPE depth) const {
		return depth / SQRT(SQUARE(GetFocalLength()) + normSq(TransformPointI2V(x)));
	}
	// same as above, but the 3D point is given
	template <typename TYPE>
	inline TYPE GetFootprintWorldSq(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> camX(TransformPointW2C(X));
		return GetFootprintWorldSq(TPoint2<TYPE>(camX.x/camX.z,camX.y/camX.z), camX.z);
	}
	template <typename TYPE>
	inline TYPE GetFootprintWorld(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> camX(TransformPointW2C(X));
		return GetFootprintWorld(TPoint2<TYPE>(camX.x/camX.z,camX.y/camX.z), camX.z);
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

MVS_API void DecomposeProjectionMatrix(const PMatrix& P, KMatrix& K, RMatrix& R, CMatrix& C);
MVS_API void DecomposeProjectionMatrix(const PMatrix& P, RMatrix& R, CMatrix& C);
MVS_API void AssembleProjectionMatrix(const KMatrix& K, const RMatrix& R, const CMatrix& C, PMatrix& P);
MVS_API void AssembleProjectionMatrix(const RMatrix& R, const CMatrix& C, PMatrix& P);
MVS_API Point3 ComputeCamerasFocusPoint(const CameraArr& cameras, const Point3* pInitialFocus=NULL);
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_CAMERA_H_
