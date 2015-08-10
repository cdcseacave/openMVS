/*
* Camera.cpp
*
* Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
* Please read <http://foxel.ch/license> for more information.
*
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This file is part of the FOXEL project <http://foxel.ch>.
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
*
*      You are required to attribute the work as explained in the "Usage and
*      Attribution" section of <http://foxel.ch/license>.
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
// C is expressed as the explicit camera center (as opposite to standard form which is negated and has the rotation of the camera already applied P = K[R|t])
class CameraIntern
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

	// returns the camera's view direction
	inline Point3 Direction() const { return R.row(2); }

	// returns the focal length
	inline REAL GetFocalLength() const { return K(0,0); }
	// returns the focal length aspect ratio
	inline REAL GetFocalLengthRatio() const { return (K(1,1) / K(0,0)); }

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
		return float(MAXF(width, height));
	}

	// create K with the supplied focal length and sensor center
	template<typename TYPE, typename TYPER>
	static inline TMatrix<TYPE,3,3> ComposeK(const TYPE& fX, const TYPE& fY, TYPER w=TYPER(1), TYPER h=TYPER(1)) {
		TMatrix<TYPE,3,3> K(TMatrix<TYPE,3,3>::IDENTITY);
		K(0,0) = fX;
		K(1,1) = fY;
		K(0,2) = TYPE(0.5)*(w-1);
		K(1,2) = TYPE(0.5)*(h-1);
		return K;
	}
	template<typename TYPE, typename TYPER>
	static inline TMatrix<TYPE,3,3> ComposeInvK(const TYPE& fX, const TYPE& fY, TYPER w=TYPER(1), TYPER h=TYPER(1)) {
		TMatrix<TYPE,3,3> invK(TMatrix<TYPE,3,3>::IDENTITY);
		invK(0,0) = TYPE(1)/fX;
		invK(1,1) = TYPE(1)/fY;
		invK(0,2) = TYPE(-0.5)*invK(0,0)*(w-1);
		invK(1,2) = TYPE(-0.5)*invK(1,1)*(h-1);
		return invK;
	}

	// returns full K and the inverse of K (assuming standard K format)
	template<typename TYPE>
	inline TMatrix<TYPE,3,3> GetK(uint32_t width, uint32_t height) const {
		const float fScale(GetNormalizationScale(width, height));
		if (K(0,2)==0 && K(1,2)==0)
			return ComposeK(
				TYPE(K(0,0)*fScale), TYPE(K(1,1)*fScale),
				width, height );
		TMatrix<TYPE,3,3> fullK(TMatrix<TYPE,3,3>::IDENTITY);
		fullK(0,0) = K(0,0)*fScale;
		fullK(1,1) = K(1,1)*fScale;
		fullK(0,2) = K(0,2)*fScale;
		fullK(1,2) = K(1,2)*fScale;
		return fullK;
	}
	template<typename TYPE>
	inline TMatrix<TYPE,3,3> GetInvK(uint32_t width, uint32_t height) const {
		const float fScale(GetNormalizationScale(width, height));
		if (K(0,2)==0 && K(1,2)==0)
			return ComposeInvK(
				TYPE(K(0,0)*fScale), TYPE(K(1,1)*fScale),
				width, height );
		TMatrix<TYPE,3,3> fullInvK(TMatrix<TYPE,3,3>::IDENTITY);
		fullInvK(0,0) = TYPE(1)/(K(0,0)*fScale);
		fullInvK(1,1) = TYPE(1)/(K(1,1)*fScale);
		fullInvK(0,2) = -K(0,2)*fScale*fullInvK(0,0);
		fullInvK(1,2) = -K(1,2)*fScale*fullInvK(1,1);
		return fullInvK;
	}

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int version) {
		ar & K;
		ar & R;
		ar & C;
	}
	#endif
};
/*----------------------------------------------------------------*/


// same as above, plus caching the projection matrix
class Camera : public CameraIntern
{
public:
	PMatrix P; // the composed projection matrix (3x4)

public:
	static const Camera IDENTITY;

public:
	inline Camera() {}
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

	// project 3D point by the camera
	template <typename TYPE>
	inline TPoint3<TYPE> ProjectPointRT3(const TPoint3<TYPE>& X) const {
		return R * (X - C);
	}
	template <typename TYPE>
	inline TPoint2<TYPE> ProjectPointRT(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> q(R * (X - C));
		return ((const TPoint2<TYPE>&)q)*INVERT(q.z);
	}
	template <typename TYPE>
	inline TPoint2<TYPE> ProjectPoint(const TPoint3<TYPE>& X) const {
		const TPoint3<TYPE> q(K * (R * (X - C)));
		return ((const TPoint2<TYPE>&)q)*INVERT(q.z);
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
		return ((const TPoint2<TYPE>&)q)*INVERT(q.z);
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

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void save(Archive& ar, const unsigned int version) const {
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MVS::CameraIntern);
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MVS::CameraIntern);
		ComposeP();
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
};
typedef SEACAVE::cList<Camera, const Camera&, 0> CameraArr;
/*----------------------------------------------------------------*/

void DecomposeProjectionMatrix(const PMatrix& P, KMatrix& K, RMatrix& R, CMatrix& C);
void DecomposeProjectionMatrix(const PMatrix& P, RMatrix& R, CMatrix& C);
void AssembleProjectionMatrix(const KMatrix& K, const RMatrix& R, const CMatrix& C, PMatrix& P);
void AssembleProjectionMatrix(const RMatrix& R, const CMatrix& C, PMatrix& P);
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_CAMERA_H_
