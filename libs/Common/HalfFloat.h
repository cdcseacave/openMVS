////////////////////////////////////////////////////////////////////
// HalfFoat.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_HALFFLOAT_H__
#define __SEACAVE_HALFFLOAT_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// hfloat - a 16-bit floating point number class
// 
// Can represent positive and negative numbers whose magnitude is between 
// roughly 6.1e-5 and 6.5e+4 with a relative error of 9.8e-4; numbers 
// smaller than 6.1e-5 can be represented with an absolute error of 
// 6.0e-8. All integers from -2048 to +2048 can be represented exactly.
// 
// Only conversions from half to float are lossless.
class GENERAL_API hfloat
{
public:
	typedef short Type;

	inline hfloat() {}
	inline hfloat(float fv) : val(fromFloat(fv)) {}

	inline operator float() const {
		return toFloat(val);
	}

	inline hfloat& operator =(const hfloat& v) {
		val = v.val; return *this;
	}
	inline hfloat& operator +=(const hfloat& v) {
		return *this = *this + v;
	}
	inline hfloat& operator -=(const hfloat& v) {
		return *this = *this - v;
	}
	inline hfloat& operator *=(const hfloat& v) {
		return *this = *this * v;
	}
	inline hfloat& operator /=(const hfloat& v) {
		return *this = *this / v;
	}
	inline hfloat operator -() const {
		return hfloat(-(float)*this);
	}

	static inline short fromFloat(float fv) {
		const int& fltInt32((int&)fv);
		short fltInt16  = ((fltInt32 & 0x7fffffff) >> 13) - (0x38000000 >> 13);
		      fltInt16 |= ((fltInt32 & 0x80000000) >> 16);
		return fltInt16;
	}
	static inline float toFloat(short fltInt16) {
		int fltInt32  = ((fltInt16 & 0x8000) << 16);
		    fltInt32 |= ((fltInt16 & 0x7fff) << 13) + 0x38000000;
		return (float&)fltInt32;
	}

protected:
	short val;
};
/*----------------------------------------------------------------*/


inline hfloat operator + (const hfloat& v1, const hfloat& v2) {
	return hfloat((float)v1 + (float)v2);
}
inline hfloat operator - (const hfloat& v1, const hfloat& v2) {
	return hfloat((float)v1 - (float)v2);
}
inline hfloat operator * (const hfloat& v1, const hfloat& v2) {
	return hfloat((float)v1 * (float)v2);
}
inline hfloat operator / (const hfloat& v1, const hfloat& v2) {
	return hfloat((float)v1 / (float)v2);
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_HALFFLOAT_H__
