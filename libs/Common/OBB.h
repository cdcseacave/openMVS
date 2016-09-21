////////////////////////////////////////////////////////////////////
// OBB.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_OBB_H__
#define __SEACAVE_OBB_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
class TAABB;

template <typename TYPE, int DIMS>
class TRay;

// Basic oriented bounding-box class
template <typename TYPE, int DIMS>
class TOBB
{
	STATIC_ASSERT(DIMS > 0 && DIMS <= 3);

public:
	typedef TYPE Type;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef Eigen::Matrix<TYPE,DIMS,DIMS,Eigen::RowMajor> MATRIX;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB;
	typedef SEACAVE::TRay<TYPE,DIMS> RAY;
	typedef unsigned ITYPE;
	typedef Eigen::Matrix<ITYPE,DIMS,1> TRIANGLE;
	enum { numCorners = (DIMS==1 ? 2 : (DIMS==2 ? 4 : 8)) }; // 2^DIMS
	enum { numScalar = (5*DIMS) };

	MATRIX m_rot;	// rotation matrix of the transformation (orthonormal axes)
	POINT m_pos;	// translation of the transformation (center-point)
	POINT m_ext;	// bounding box extents (half axis length)

	//---------------------------------------

	inline TOBB() {}
	inline TOBB(const POINT* pts, size_t n);
	inline TOBB(const POINT* pts, size_t n, const TRIANGLE* tris, size_t s);

	inline void Set(const POINT* pts, size_t n); // build from points
	inline void Set(const POINT* pts, size_t n, const TRIANGLE* tris, size_t s); // build from triangles
	inline void Set(const MATRIX& C, const POINT* pts, size_t n); // build from covariance matrix
	inline void SetRotation(const MATRIX& C); // build rotation only from covariance matrix
	inline void SetBounds(const POINT* pts, size_t n); // build size and center only from given points

	inline void BuildBegin(); // start online build for computing the rotation
	inline void BuildAdd(const POINT&); // add a new point to the online build
	inline void BuildEnd(); // end online build for computing the rotation

	inline void Enlarge(TYPE);
	inline void EnlargePercent(TYPE);

	inline void Translate(const POINT&);
	inline void Transform(const MATRIX&);

	inline POINT GetCenter() const;
	inline void GetCenter(POINT&) const;

	inline POINT GetSize() const;
	inline void GetSize(POINT&) const;

	inline void GetCorners(POINT pts[numCorners]) const;
	inline AABB GetAABB() const;

	inline TYPE GetVolume() const;

	inline TYPE& operator [] (BYTE i) { ASSERT(i<numScalar); return m_rot.data()[i]; }
	inline TYPE operator [] (BYTE i) const { ASSERT(i<numScalar); return m_rot.data()[i]; }
}; // class TOBB
/*----------------------------------------------------------------*/


#include "OBB.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_OBB_H__
