////////////////////////////////////////////////////////////////////
// Sphere.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_SPHERE_H__
#define __SEACAVE_SPHERE_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Basic sphere class
template <typename TYPE, int DIMS>
class TSphere
{
	STATIC_ASSERT(DIMS > 1 && DIMS <= 3);

public:
	typedef TYPE Type;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;

	POINT center;		// sphere center point
	TYPE radius;		// sphere radius

	//---------------------------------------

	inline TSphere() {}
	inline TSphere(const POINT& c, TYPE r) : center(c), radius(r) {}
	inline TSphere(const POINT& p1, const POINT& p2, const POINT& p3);

	inline void Set(const POINT& c, TYPE r);
	inline void Set(const POINT& p1, const POINT& p2, const POINT& p3);

	inline void Enlarge(TYPE);
	inline void EnlargePercent(TYPE);

	inline GCLASS Classify(const POINT&) const;
}; // class TSphere
/*----------------------------------------------------------------*/

#include "Sphere.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_SPHERE_H__
