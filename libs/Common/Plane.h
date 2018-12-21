////////////////////////////////////////////////////////////////////
// Plane.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_PLANE_H__
#define __SEACAVE_PLANE_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Basic hyper-plane class
// (plane represented in Hessian Normal Form: n.x+d=0 <=> ax+by+cz+d=0)
template <typename TYPE, int DIMS=3>
class TPlane
{
	STATIC_ASSERT(DIMS > 0 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS,1> VECTOR;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB;
	typedef SEACAVE::TRay<TYPE,DIMS> RAY;

	VECTOR	m_vN;	// plane normal vector
	TYPE	m_fD;	// distance to origin

	//---------------------------------------

	inline TPlane() {}
	inline TPlane(const VECTOR&, TYPE);
	inline TPlane(const VECTOR&, const POINT&);
	inline TPlane(const POINT&, const POINT&, const POINT&);
	inline TPlane(const TYPE p[DIMS+1]);

	inline void Set(const VECTOR&, TYPE);
	inline void Set(const VECTOR&, const POINT&);
	inline void Set(const POINT&, const POINT&, const POINT&);
	inline void Set(const TYPE p[DIMS+1]);
	inline void Negate();

	inline TYPE Distance(const TPlane&) const;
	inline TYPE Distance(const POINT&) const;
	inline TYPE DistanceAbs(const POINT&) const;

	inline GCLASS Classify(const POINT&) const;
	inline GCLASS Classify(const AABB&) const;

	bool Clip(const RAY&, TYPE, RAY*, RAY*) const;

	bool Intersects(const POINT& p0, const POINT& p1, const POINT& p2) const;
	bool Intersects(const TPlane& plane, RAY& ray) const;
	bool Intersects(const AABB& aabb) const;

	inline TYPE& operator [] (BYTE i) { ASSERT(i<=DIMS); return m_vN.data()[i]; }
	inline TYPE operator [] (BYTE i) const { ASSERT(i<=DIMS); return m_vN.data()[i]; }
}; // class TPlane
/*----------------------------------------------------------------*/


// Basic frustum class
// (represented as 6 planes oriented toward outside the frustum volume)
template <typename TYPE, int DIMS=6>
class TFrustum
{
	STATIC_ASSERT(DIMS > 0 && DIMS <= 6);

public:
	typedef Eigen::Matrix<TYPE,4,4,Eigen::RowMajor> MATRIX4x4;
	typedef Eigen::Matrix<TYPE,3,4,Eigen::RowMajor> MATRIX3x4;
	typedef Eigen::Matrix<TYPE,4,1> VECTOR4;
	typedef Eigen::Matrix<TYPE,3,1> VECTOR;
	typedef Eigen::Matrix<TYPE,3,1> POINT;
	typedef SEACAVE::TPlane<TYPE,3> PLANE;
	typedef SEACAVE::TSphere<TYPE,3> SPHERE;
	typedef SEACAVE::TAABB<TYPE,3> AABB;

	PLANE	m_planes[DIMS];	// left, right, top, bottom, near and far planes

	//---------------------------------------

	inline TFrustum() {}
	inline TFrustum(const MATRIX4x4&);
	inline TFrustum(const MATRIX3x4&);
	inline TFrustum(const MATRIX4x4&, TYPE width, TYPE height, TYPE near=TYPE(0.0001), TYPE far=TYPE(1000));
	inline TFrustum(const MATRIX3x4&, TYPE width, TYPE height, TYPE near=TYPE(0.0001), TYPE far=TYPE(1000));

	template <int MODE> void Set(const MATRIX4x4&);
	template <int MODE> void Set(const MATRIX3x4&);
	void Set(const MATRIX4x4&, TYPE width, TYPE height, TYPE near=TYPE(0.0001), TYPE far=TYPE(1000));
	void Set(const MATRIX3x4&, TYPE width, TYPE height, TYPE near=TYPE(0.0001), TYPE far=TYPE(1000));

	GCLASS Classify(const POINT&) const;
	GCLASS Classify(const SPHERE&) const;
	GCLASS Classify(const AABB&) const;

	inline TYPE& operator [] (BYTE i) { ASSERT(i<DIMS); return m_planes[i]; }
	inline TYPE operator [] (BYTE i) const { ASSERT(i<DIMS); return m_planes[i]; }
}; // class TPlane
/*----------------------------------------------------------------*/


#include "Plane.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_PLANE_H__
