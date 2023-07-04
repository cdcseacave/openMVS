////////////////////////////////////////////////////////////////////
// Line.h
//
// Copyright 2023 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_LINE_H__
#define __SEACAVE_LINE_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Generic line class represented as two points
template <typename TYPE, int DIMS>
class TLine
{
	STATIC_ASSERT(DIMS > 1 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS,1> VECTOR;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB;
	typedef SEACAVE::TRay<TYPE,DIMS> RAY;
	enum { numScalar = (2*DIMS) };
	enum { numParams = numScalar-1 };

	POINT	pt1, pt2;	// line description

	//---------------------------------------

	inline TLine() {}
	inline TLine(const POINT& pt1, const POINT& pt2);
	template <typename CTYPE>
	inline TLine(const TLine<CTYPE,DIMS>&);

	inline void Set(const POINT& pt1, const POINT& pt2);

	int Optimize(const POINT*, size_t, int maxIters=100);
	template <typename RobustNormFunctor>
	int Optimize(const POINT*, size_t, const RobustNormFunctor& robust, int maxIters=100);

	inline TYPE GetLength() const;
	inline TYPE GetLengthSq() const;
	inline POINT GetCenter() const;
	inline VECTOR GetDir() const;
	inline VECTOR GetNormDir() const;
	inline RAY GetRay() const;

	inline bool IsSame(const TLine&, TYPE th) const;

	bool Intersects(const AABB& aabb) const;
	bool Intersects(const AABB& aabb, TYPE& t) const;

	inline TYPE DistanceSq(const POINT&) const;
	inline TYPE Distance(const POINT&) const;

	inline TYPE Classify(const POINT&) const;
	inline POINT ProjectPoint(const POINT&) const;

	inline TYPE& operator [] (BYTE i) { ASSERT(i<numScalar); return pt1.data()[i]; }
	inline TYPE operator [] (BYTE i) const { ASSERT(i<numScalar); return pt1.data()[i]; }

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & pt1;
		ar & pt2;
	}
	#endif
}; // class TLine
/*----------------------------------------------------------------*/

template <typename TYPE, typename TYPEW=TYPE>
struct FitLineOnline : FitPlaneOnline<TYPE,TYPEW,true> {
	template <typename TYPEE> TPoint3<TYPEE> GetLine(TLine<TYPEE,3>& line) const;
};
/*----------------------------------------------------------------*/


#include "Line.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_LINE_H__
