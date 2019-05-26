////////////////////////////////////////////////////////////////////
// Ray.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_RAY_H__
#define __SEACAVE_RAY_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Basic triangle class
template <typename TYPE, int DIMS>
class TTriangle
{
	STATIC_ASSERT(DIMS > 1 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS,1> VECTOR;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB;
	typedef SEACAVE::TPlane<TYPE,DIMS> PLANE;
	enum { numScalar = (3*DIMS) };

	POINT	a, b, c;	// triangle vertices

	//---------------------------------------

	inline TTriangle() {}
	inline TTriangle(const POINT&, const POINT&, const POINT&);

	inline void Set(const POINT&, const POINT&, const POINT&);

	inline AABB GetAABB() const;
	inline PLANE GetPlane() const;

	inline const POINT& operator [] (BYTE i) const { ASSERT(i<3); return (&a)[i]; }
	inline POINT& operator [] (BYTE i) { ASSERT(i<3); return (&a)[i]; }
}; // class TTriangle
/*----------------------------------------------------------------*/


// Basic ray class
template <typename TYPE, int DIMS>
class TRay
{
	STATIC_ASSERT(DIMS > 1 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS+1,DIMS+1,Eigen::RowMajor> MATRIX;
	typedef Eigen::Matrix<TYPE,DIMS,1> VECTOR;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef SEACAVE::TTriangle<TYPE,DIMS> TRIANGLE;
	typedef SEACAVE::TSphere<TYPE,DIMS> SPHERE;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB;
	typedef SEACAVE::TOBB<TYPE,DIMS> OBB;
	typedef SEACAVE::TPlane<TYPE,DIMS> PLANE;
	enum { numScalar = (2*DIMS) };

	VECTOR	m_vDir;		// ray direction (normalized)
	POINT	m_pOrig;	// ray origin

	//---------------------------------------

	inline TRay() {}
	inline TRay(const POINT& pOrig, const VECTOR& vDir);
	inline TRay(const POINT& pt0, const POINT& pt1, bool bPoints);

	inline void Set(const POINT& pOrig, const VECTOR& vDir);
	inline void SetFromPoints(const POINT& pt0, const POINT& pt1);
	inline TYPE SetFromPointsLen(const POINT& pt0, const POINT& pt1);
	inline void DeTransform(const MATRIX&);			// move to matrix space

	template <bool bCull>
	bool Intersects(const TRIANGLE&, TYPE *t) const;
	template <bool bCull>
	bool Intersects(const TRIANGLE&, TYPE fL, TYPE *t) const;
	bool Intersects(const POINT&, const POINT&, const POINT&,
					bool bCull, TYPE *t) const;
	bool Intersects(const POINT&, const POINT&, const POINT&,
					bool bCull, TYPE fL, TYPE *t) const;
	bool Intersects(const PLANE& plane, bool bCull,
					TYPE *t, POINT* pPtHit) const;
	inline TYPE IntersectsDist(const PLANE& plane) const;
	inline POINT Intersects(const PLANE& plane) const;
	bool Intersects(const PLANE& plane, bool bCull,
					TYPE fL, TYPE *t, POINT* pPtHit) const;
	bool Intersects(const SPHERE& sphere) const;
	bool Intersects(const SPHERE& sphere, TYPE& t) const;
	bool Intersects(const AABB& aabb) const;
	bool Intersects(const AABB& aabb, TYPE& t) const;
	bool Intersects(const AABB& aabb, TYPE fL, TYPE *t) const;
	bool Intersects(const OBB& obb) const;
	bool Intersects(const OBB& obb, TYPE &t) const;
	bool Intersects(const OBB& obb, TYPE fL, TYPE *t) const;
	bool Intersects(const TRay& ray, TYPE& s) const;
	bool Intersects(const TRay& ray, POINT& p) const;
	bool Intersects(const TRay& ray, TYPE& s1, TYPE& s2) const;
	bool IntersectsAprox(const TRay& ray, POINT& p) const;
	bool IntersectsAprox2(const TRay& ray, POINT& p) const;

	inline TYPE CosAngle(const TRay&) const;
	inline bool Coplanar(const TRay&) const;
	inline bool Parallel(const TRay&) const;

	inline TYPE Classify(const POINT&) const;
	inline POINT ProjectPoint(const POINT&) const;
	bool DistanceSq(const POINT&, TYPE&) const;
	bool Distance(const POINT&, TYPE&) const;
	TYPE DistanceSq(const POINT&) const;
	TYPE Distance(const POINT&) const;

	TRay operator * (const MATRIX&) const;			// matrix multiplication
	inline TRay& operator *= (const MATRIX&);		// matrix multiplication

	inline TYPE& operator [] (BYTE i) { ASSERT(i<6); return m_vDir.data()[i]; }
	inline TYPE operator [] (BYTE i) const { ASSERT(i<6); return m_vDir.data()[i]; }
}; // class TRay
/*----------------------------------------------------------------*/


// Basic cylinder class
template <typename TYPE, int DIMS>
class TCylinder
{
	STATIC_ASSERT(DIMS > 1 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS,1> VECTOR;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef SEACAVE::TRay<TYPE,DIMS> RAY;
	typedef SEACAVE::TSphere<TYPE,DIMS> SPHERE;
	enum { numScalar = (2*DIMS+2) };

	RAY ray;            // central ray defining starting the point and direction
	TYPE radius;        // cylinder radius
	TYPE minHeight;     // cylinder heights satisfying:
	TYPE maxHeight;     // std::numeric_limits<TYPE>::lowest() <= minHeight <= 0 < maxHeight <= std::numeric_limits<TYPE>::max()

	//---------------------------------------

	inline TCylinder() {}

	inline TCylinder(const RAY& _ray, TYPE _radius, TYPE _minHeight=TYPE(0), TYPE _maxHeight=std::numeric_limits<TYPE>::max())
		: ray(_ray), radius(_radius), minHeight(_minHeight), maxHeight(_maxHeight) { ASSERT(TYPE(0) >= minHeight && minHeight < maxHeight); }

	inline bool IsFinite() const { return maxHeight < std::numeric_limits<TYPE>::max() && minHeight > std::numeric_limits<TYPE>::lowest(); }
	inline TYPE GetLength() const { return maxHeight - minHeight; }

	bool Intersects(const SPHERE&) const;

	inline GCLASS Classify(const POINT&, TYPE& t) const;
}; // class TCylinder
/*----------------------------------------------------------------*/


// Basic cone class
template <typename TYPE, int DIMS>
class TCone
{
	STATIC_ASSERT(DIMS > 1 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS,1> VECTOR;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef SEACAVE::TRay<TYPE,DIMS> RAY;
	typedef SEACAVE::TSphere<TYPE,DIMS> SPHERE;
	enum { numScalar = (2*DIMS+3) };

	RAY ray;            // ray origin is the cone vertex and ray direction is the cone axis direction
	TYPE angle;         // cone angle, in radians, must be inside [0, pi/2]
	TYPE minHeight;     // heights satisfying:
	TYPE maxHeight;     // 0 <= minHeight < maxHeight <= std::numeric_limits<TYPE>::max()

	//---------------------------------------

	inline TCone() {}
	inline TCone(const RAY& _ray, TYPE _angle, TYPE _minHeight=TYPE(0), TYPE _maxHeight=std::numeric_limits<TYPE>::max())
		: ray(_ray), angle(_angle), minHeight(_minHeight), maxHeight(_maxHeight) { ASSERT(TYPE(0) <= minHeight && minHeight < maxHeight); }
}; // class TCone

// Structure used to compute the intersection between a cone and the given sphere/point
template <typename TYPE, int DIMS>
class TConeIntersect
{
	STATIC_ASSERT(DIMS > 1 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS,1> VECTOR;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef SEACAVE::TCone<TYPE,DIMS> CONE;
	typedef SEACAVE::TSphere<TYPE,DIMS> SPHERE;

	const CONE& cone;

	// cache angle derivatives, to avoid calling trigonometric functions in geometric queries
	const TYPE cosAngle, sinAngle, sinAngleSq, cosAngleSq, invSinAngle;

	//---------------------------------------

	inline TConeIntersect(const CONE& _cone)
		: cone(_cone), cosAngle(COS(cone.angle)), sinAngle(SIN(cone.angle)),
		sinAngleSq(SQUARE(sinAngle)), cosAngleSq(SQUARE(cosAngle)),
		invSinAngle(TYPE(1)/sinAngle) {}

	bool operator()(const SPHERE&) const;

	GCLASS Classify(const POINT&, TYPE& t) const;
}; // class TConeIntersect
/*----------------------------------------------------------------*/


#include "Ray.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_RAY_H__
