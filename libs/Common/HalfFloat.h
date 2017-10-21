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
// Supports Denormals-as-zero (DAZ), but does not support infinities or NaN.
//
// Only conversions from half to float are lossless.
class GENERAL_API hfloat
{
public:
	typedef short Type;

	inline hfloat() : val(fromFloat(float())) {}
	explicit inline hfloat(float v) : val(fromFloat(v)) {}
	template <typename T>
	inline hfloat(T v) : val(fromFloat(static_cast<float>(v))) {}

	inline hfloat& operator = (hfloat v) {
		val = v.val;
		return *this;
	}
	inline hfloat& operator = (float v) {
		val = fromFloat(v);
		return *this;
	}
	template <typename T>
	inline hfloat& operator = (T v) {
		val = fromFloat(static_cast<float>(v));
		return *this;
	}

	inline operator float() const {
		return toFloat(val);
	}
	inline hfloat operator - () const {
		return fromFloat2HFloat(-toFloat(val));
	}
	inline hfloat operator + (hfloat v) {
		return fromFloat2HFloat(toFloat(val) + toFloat(v.val));
	}
	inline hfloat operator + (float v) {
		return fromFloat2HFloat(toFloat(val) + v);
	}
	inline hfloat& operator += (hfloat v) {
		return *this = *this + v;
	}
	inline hfloat& operator += (float v) {
		return *this = *this + v;
	}
	inline hfloat operator - (hfloat v) {
		return fromFloat2HFloat(toFloat(val) - toFloat(v.val));
	}
	inline hfloat operator - (float v) {
		return fromFloat2HFloat(toFloat(val) - v);
	}
	inline hfloat& operator -= (hfloat v) {
		return *this = *this - v;
	}
	inline hfloat& operator -= (float v) {
		return *this = *this - v;
	}
	inline hfloat operator * (hfloat v) {
		return fromFloat2HFloat(toFloat(val) * toFloat(v.val));
	}
	inline hfloat operator * (float v) {
		return fromFloat2HFloat(toFloat(val) * v);
	}
	inline hfloat& operator *= (hfloat v) {
		return *this = *this * v;
	}
	inline hfloat& operator *= (float v) {
		return *this = *this * v;
	}
	inline hfloat operator / (hfloat v) {
		return fromFloat2HFloat(toFloat(val) / toFloat(v.val));
	}
	inline hfloat operator / (float v) {
		return fromFloat2HFloat(toFloat(val) / v);
	}
	inline hfloat& operator /= (hfloat v) {
		return *this = *this / v;
	}
	inline hfloat& operator /= (float v) {
		return *this = *this / v;
	}

	static float min() { return 6.1e-5f; }
	static float max() { return 6.5e+4f; }

	static inline short fromFloat(float f) {
		ASSERT(ISFINITE(f) && (ABS(f) == 0.f || ABS(f) >= min()) && ABS(f) <= max());
		// ~8 clock cycles on modern x86-64
		const CastF2I ufi(f);
		int32_t t1 = ufi.i & 0x7fffffff;      // Non-sign bits
		int32_t t2 = ufi.i & 0x80000000;      // Sign bit
		int32_t t3 = ufi.i & 0x7f800000;      // Exponent
		t1 >>= 13;                            // Align mantissa on MSB
		t2 >>= 16;                            // Shift sign bit into position
		t1 -= 0x1c000;                        // Adjust bias
		t1 = (t3 < 0x38800000) ? 0 : t1;      // Flush-to-zero
		t1 = (t3 > 0x47000000) ? 0x7bff : t1; // Clamp-to-max
		t1 = (t3 == 0 ? 0 : t1);              // Denormals-as-zero
		return (short)(t1 | t2);              // Re-insert sign bit
	}
	static inline float toFloat(short h) {
		// ~6 clock cycles on modern x86-64
		CastF2I ufi;
		int32_t t1 = h & 0x7fff; // Non-sign bits
		int32_t t2 = h & 0x8000; // Sign bit
		int32_t t3 = h & 0x7c00; // Exponent
		t1 <<= 13;               // Align mantissa on MSB
		t2 <<= 16;               // Shift sign bit into position
		t1 += 0x38000000;        // Adjust bias
		t1 = (t3 == 0 ? 0 : t1); // Denormals-as-zero
		ufi.i = t1 | t2;         // Re-insert sign bit
		return ufi.f;
	}

	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & val;
	}
	#endif

protected:
	static inline hfloat fromFloat2HFloat(float f) {
		return TAliasCast<short,hfloat>(fromFloat(f)).i;
	}

protected:
	Type val;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_HALFFLOAT_H__
