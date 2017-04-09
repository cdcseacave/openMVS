////////////////////////////////////////////////////////////////////
// AABB.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_AABB_H__
#define __SEACAVE_AABB_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Basic axis-aligned bounding-box class
template <typename TYPE, int DIMS>
class TAABB
{
	STATIC_ASSERT(DIMS > 0 && DIMS <= 3);

public:
	typedef TYPE Type;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef Eigen::Matrix<TYPE,DIMS,DIMS,Eigen::RowMajor> MATRIX;
	enum { numChildren = (2<<(DIMS-1)) };
	enum { numCorners = (DIMS==1 ? 2 : (DIMS==2 ? 4 : 8)) }; // 2^DIMS
	enum { numScalar = (2*DIMS) };

	POINT ptMin, ptMax;	// box extreme points

	//---------------------------------------

	inline TAABB() {}
	inline TAABB(bool);
	inline TAABB(const POINT& _pt);
	inline TAABB(const POINT& _ptMin, const POINT& _ptMax);
	inline TAABB(const POINT& center, const TYPE& radius);
	template <typename TPoint>
	inline TAABB(const TPoint* pts, size_t n);
	template <typename CTYPE>
	inline TAABB(const TAABB<CTYPE, DIMS>&);

	inline void Reset();
	inline void Set(const POINT& _pt);
	inline void Set(const POINT& _ptMin, const POINT& _ptMax);
	inline void Set(const POINT& center, const TYPE& radius);
	template <typename TPoint>
	inline void Set(const TPoint* pts, size_t n);

	inline bool IsEmpty() const;

	inline void Enlarge(TYPE);
	inline void EnlargePercent(TYPE);

		   void InsertFull(const POINT&);
		   void Insert(const POINT&);
		   void Insert(const TAABB&);

	inline void Translate(const POINT&);
	inline void Transform(const MATRIX&);

	inline POINT GetCenter() const;
	inline void GetCenter(POINT&) const;

	inline POINT GetSize() const;
	inline void GetSize(POINT&) const;

	inline void GetCorner(BYTE i, POINT&) const;
	inline POINT GetCorner(BYTE i) const;
	inline void GetCorners(POINT pts[numCorners]) const;

	bool Intersects(const TAABB&) const;
	bool IntersectsComplete(const TAABB&, TYPE) const;
	bool IntersectsComplete(const TAABB&) const;
	bool Intersects(const POINT&) const;
	bool IntersectsComplete(const POINT&, TYPE) const;
	bool IntersectsComplete(const POINT&) const;

	unsigned SplitBy(TYPE, int, TAABB [2]) const;
	unsigned SplitBy(const POINT&, TAABB [numChildren], unsigned&) const;

	inline TYPE& operator [] (BYTE i) { ASSERT(i<numScalar); return ptMin.data()[i]; }
	inline TYPE operator [] (BYTE i) const { ASSERT(i<numScalar); return ptMin.data()[i]; }
}; // class TAABB
/*----------------------------------------------------------------*/


#include "AABB.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_AABB_H__
