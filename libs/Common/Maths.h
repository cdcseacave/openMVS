////////////////////////////////////////////////////////////////////
// Maths.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_MATHS_H__
#define __SEACAVE_MATHS_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Config.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <type_traits>

#ifdef _USE_SSE
#include <xmmintrin.h>
#include <emmintrin.h>
#endif

#ifdef _USE_EIGEN
#if defined(_MSC_VER)
#pragma warning (push)
#pragma warning (disable : 4244) // 'argument': conversion from '__int64' to 'int', possible loss of data
#endif
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/LU>
#if defined(_MSC_VER)
#pragma warning (pop)
#endif
#include <opencv2/core/eigen.hpp>
#endif


// D E F I N E S ///////////////////////////////////////////////////

//
// Type defines

#ifndef _MSC_VER
typedef unsigned char		BYTE;
typedef unsigned short		WORD;
typedef unsigned int		DWORD;
typedef uint64_t	        QWORD;
#endif //_MSC_VER

#define DECLARE_NO_INDEX(...) std::numeric_limits<__VA_ARGS__>::max()

#ifndef MAKEWORD
#define MAKEWORD(a, b)		((WORD)(((BYTE)(((DWORD)(a)) & 0xff)) | ((WORD)((BYTE)(((DWORD)(b)) & 0xff))) << 8))
#endif
#ifndef MAKELONG
#define MAKELONG(a, b)		((DWORD)(((WORD)(((DWORD)(a)) & 0xffff)) | ((DWORD)((WORD)(((DWORD)(b)) & 0xffff))) << 16))
#endif
#ifndef LOWORD
#define LOWORD(l)			((WORD)(((DWORD)(l)) & 0xffff))
#endif
#ifndef HIWORD
#define HIWORD(l)			((WORD)((((DWORD)(l)) >> 16) & 0xffff))
#endif
#ifndef LOBYTE
#define LOBYTE(w)			((BYTE)(((WORD)(w)) & 0xff))
#endif
#ifndef HIBYTE
#define HIBYTE(w)			((BYTE)((((WORD)(w)) >> 8) & 0xff))
#endif

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#ifndef MINF
#define MINF                std::min
#endif
#ifndef MAXF
#define MAXF                std::max
#endif

#ifndef RAND
#define RAND			    std::rand
#endif


#define RGBA(r, g, b, a)	((DWORD)(((a) << 24) | ((r) << 16) | ((g) << 8) | (b)))
#define RGBC(clr)			(RGBA((BYTE)((clr).fR*255), (BYTE)((clr).fG*255), (BYTE)((clr).fB*255), (BYTE)((clr).fA*255)))
#define RGB24TO8(r,g,b)		((BYTE)((((WORD)r)*30+((WORD)g)*59+((WORD)b)*11)/100))
#define RGB24TO16(r,g,b)	((((WORD)(((BYTE)(r))>>3))<<11) | (((WORD)(((BYTE)(g))>>2))<<5) | ((WORD)(((BYTE)(b))>>3)))
#define RGB16TOR(rgb)		(((BYTE)(((WORD)(rgb))>>11))<<3)
#define RGB16TOG(rgb)		(((BYTE)((((WORD)(rgb))&0x07E0)>>5))<<2)
#define RGB16TOB(rgb)		(((BYTE)(((WORD)(rgb))&0x001F))<<3)


#ifndef _USE_MATH_DEFINES
/** e */
#ifndef M_E
#define M_E			2.7182818284590452353602874713527
#endif
/** ln(2) */
#ifndef M_LN2
#define M_LN2		0.69314718055994530941723212145818
#endif
/** ln(10) */
#ifndef M_LN10
#define M_LN10		2.3025850929940456840179914546844
#endif
/** pi */
#ifndef M_PI
#define M_PI		3.1415926535897932384626433832795
#endif
/** pi/2 */
#ifndef M_PI_2
#define M_PI_2		1.5707963267948966192313216916398
#endif
/** 1/pi */
#ifndef M_1_PI
#define M_1_PI		0.31830988618379067153776752674503
#endif
/** 2/pi */
#ifndef M_2_PI
#define M_2_PI		0.63661977236758134307553505349006
#endif
/** 2*sqrt(pi) */
#ifndef M_2_SQRTPI
#define M_2_SQRTPI	1.1283791670955125738961589031216
#endif
/** sqrt(2) */
#ifndef M_SQRT2
#define M_SQRT2		1.4142135623730950488016887242097
#endif
/** sqrt(1/2) */
#ifndef M_SQRT1_2
#define M_SQRT1_2	0.70710678118654752440084436210485
#endif
#endif

// constants
#define TWO_PI			6.283185307179586476925286766559
#define PI				3.1415926535897932384626433832795
#define HALF_PI			1.5707963267948966192313216916398
#define SQRT_2PI		2.506628274631000502415765284811
#define INV_TWO_PI		0.15915494309189533576888376337251
#define INV_PI			0.31830988618379067153776752674503
#define INV_HALF_PI		0.63661977236758134307553505349006
#define INV_SQRT_2PI	0.39894228040143267793994605993439
#define D2R(d)			((d)*(PI/180.0)) // degree to radian
#define R2D(r)			((r)*(180.0/PI)) // radian to degree
#define SQRT_2			1.4142135623730950488016887242097
#define SQRT_3			1.7320508075688772935274463415059
#define LOG_2			0.30102999566398119521373889472449
#define LN_2			0.69314718055994530941723212145818
#define ZERO_TOLERANCE	(1e-7)
#define INV_ZERO		(1e+14)

// float constants
#define FTWO_PI			((float)TWO_PI)
#define FPI				((float)PI)
#define FHALF_PI		((float)HALF_PI)
#define FSQRT_2PI		((float)SQRT_2PI)
#define FINV_TWO_PI		((float)INV_TWO_PI)
#define FINV_PI			((float)INV_PI)
#define FINV_HALF_PI	((float)INV_HALF_PI)
#define FINV_SQRT_2PI	((float)INV_SQRT_2PI)
#define FD2R(d)			((d)*(FPI/180.f)) // degree to radian
#define FR2D(r)			((r)*(180.f/FPI)) // radian to degree
#define FSQRT_2			((float)SQRT_2)
#define FSQRT_3			((float)SQRT_3)
#define FLOG_2			((float)LOG_2)
#define FLN_2			((float)LN_2)
#define FZERO_TOLERANCE	0.0001f
#define FINV_ZERO		1000000.f

#define GCLASS			unsigned
#define FRONT			0
#define BACK			1
#define PLANAR			2
#define CLIPPED			3
#define CULLED			4
#define VISIBLE			5


// M A C R O S /////////////////////////////////////////////////////

#define FLOOR			SEACAVE::Floor2Int
#define FLOOR2INT		SEACAVE::Floor2Int
#define CEIL			SEACAVE::Ceil2Int
#define CEIL2INT		SEACAVE::Ceil2Int
#define ROUND			SEACAVE::Round2Int
#define ROUND2INT		SEACAVE::Round2Int
#define SIN				std::sin
#define ASIN			std::asin
#define COS				std::cos
#define ACOS			std::acos
#define TAN				std::tan
#define ATAN			std::atan
#define ATAN2			std::atan2
#define POW				std::pow
#define POWI			SEACAVE::powi
#define LOG2I			SEACAVE::log2i


namespace SEACAVE {

// T Y P E   D E F I N E S /////////////////////////////////////////

// signed and unsigned types of the size of the architecture
// (32 or 64 bit for x86 and respectively x64)
#ifdef _ENVIRONMENT64
typedef int64_t             int_t;
typedef uint64_t            uint_t;
#else
typedef int32_t             int_t;
typedef uint32_t            uint_t;
#endif

// type used for the size of the files
typedef int64_t     	    size_f_t;

// type used as the default floating number precision
typedef double              REAL;

// invalid index
constexpr uint32_t NO_ID = DECLARE_NO_INDEX(uint32_t);

template<typename TYPE, typename REALTYPE=REAL>
struct RealType { typedef typename std::conditional<std::is_floating_point<TYPE>::value, TYPE, REALTYPE>::type type; };


// F U N C T I O N S ///////////////////////////////////////////////

template<typename T>
inline T MINF3(const T& x1, const T& x2, const T& x3) {
	return MINF(MINF(x1, x2), x3);
}
template<typename T>
inline T MAXF3(const T& x1, const T& x2, const T& x3) {
	return MAXF(MAXF(x1, x2), x3);
}

template<typename T>
FORCEINLINE T RANDOM() { return T(RAND())/T(RAND_MAX); }

template<typename T1, typename T2>
union TAliasCast {
	T1 f;
	T2 i;
	inline TAliasCast() {}
	inline TAliasCast(T1 v) : f(v) {}
	inline TAliasCast(T2 v) : i(v) {}
	inline TAliasCast& operator =(T1 v) { f = v; return *this; }
	inline TAliasCast& operator =(T2 v) { i = v; return *this; }
	inline operator T1 () const { return f; }
};
typedef TAliasCast<float,int32_t> CastF2I;
typedef TAliasCast<double,int32_t> CastD2I;

template<typename T>
struct MakeIdentity { using type = T; };
template<typename T>
using MakeSigned = typename std::conditional<std::is_integral<T>::value,std::make_signed<T>,SEACAVE::MakeIdentity<T>>::type;

template<typename T1, typename T2>
constexpr T1 Cast(const T2& v) {
	return static_cast<T1>(v);
}

template<typename T>
constexpr T& NEGATE(T& a) {
	return (a = -a);
}
template<typename T>
constexpr T SQUARE(const T& a) {
	return a * a;
}
template<typename T>
constexpr T CUBE(const T& a) {
	return a * a * a;
}
template<typename T>
inline T SQRT(const T& a) {
	return T(std::sqrt(a));
}
template<typename T>
inline T EXP(const T& a) {
	return T(std::exp(a));
}
template<typename T>
inline T LOGN(const T& a) {
	return T(std::log(a));
}
template<typename T>
inline T LOG10(const T& a) {
	return T(std::log10(a));
}
template<typename T>
constexpr T powi(T base, unsigned exp) {
	T result(1);
	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}
	return result;
}
constexpr int log2i(unsigned val) {
	int ret = -1;
	while (val) {
		val >>= 1;
		++ret;
	}
	return ret;
}
template <unsigned N> constexpr inline int log2i() { return 1+log2i<(N>>1)>(); }
template <>   constexpr inline int log2i<0>() { return -1; }
template <>   constexpr inline int log2i<1>() { return 0; }
template <>   constexpr inline int log2i<2>() { return 1; }

template<typename T>
inline T arithmeticSeries(T n, T a1=1, T d=1) {
	return (n*(a1*2+(n-1)*d))/2;
}
template<typename T>
constexpr T factorial(T n) {
	T ret = 1;
	while (n > 1)
		ret *= n--;
	return ret;
}
template<typename T>
constexpr T combinations(const T& n, const T& k) {
	SIMPLE_ASSERT(n >= k);
	#if 1
	T num = n;
	const T den = factorial(k);
	for (T i=n-k+1; i<n; ++i)
		num *= i;
	SIMPLE_ASSERT(num%den == 0);
	return num/den;
	#else
	return factorial(n) / (factorial(k)*factorial(n-k));
	#endif
}

// adapted from https://github.com/whackashoe/fastapprox.git
// (set bSafe to true if the values might be smaller than -126)
template<bool bSafe>
inline float FPOW2(float p) {
	if (bSafe && p < -126.f) {
		return 0.f;
	} else {
		ASSERT(p >= -126.f);
		CastF2I v;
		v.i = static_cast<int32_t>((1 << 23) * (p + 126.94269504f));
		return v.f;
	}
}
template<bool bSafe>
inline float FEXP(float v) {
	return FPOW2<bSafe>(1.44269504f * v);
}

// Inverse of the square root
// Compute a fast 1 / sqrtf(v) approximation
inline float RSQRT(float v) {
	#ifdef _FAST_INVSQRT
	// This code supposedly originates from Id-software
	const float halfV = v * 0.5f;
	(int32_t&)v = 0x5f3759df - (((int32_t&)v) >> 1);
	// Iterations of the Newton's method
	v = v * (1.5f - halfV * v * v);
	v = v * (1.5f - halfV * v * v);
	return v * (1.5f - halfV * v * v);
	#else
	return 1.f / SQRT(v);
	#endif
}
inline double RSQRT(const double& x) {
	#ifdef _FAST_INVSQRT
	double v = x;
	const double halfV = v * 0.5;
	(int64_t&)v = 0x5fe6ec85e7de30daLL - (((int64_t&)v) >> 1);
	// Iterations of the Newton's method
	v = v * (1.5 - halfV * v * v);
	v = v * (1.5 - halfV * v * v);
	v = v * (1.5 - halfV * v * v);
	return v * (1.5 - halfV * v * v);
	#else
	return 1.0 / SQRT(x);
	#endif
}

// approximate tanh
template <typename T>
inline T TANH(const T& x) {
	const T x2 = x*x;
	#if 0
	// Taylor series expansion (very inaccurate)
	return x*(1.0 + x2*(-T(1)/T(3) + x2*(T(2)/T(15) + x2*(-T(17)/T(315) + x2*(T(62)/T(2835) - x2*(T(1382)/T(155925)))))));
	#else
	// Lambert's continued fraction
	const T den = (((x2+T(378))*x2+T(17325))*x2+T(135135))*x;
	const T div = ((x2*T(28)+T(3150))*x2+T(62370))*x2+T(135135);
	return den/div;
	#endif
}
/*----------------------------------------------------------------*/


// Cubic root functions
// cube root approximation using bit hack for 32-bit float (5 decimals)
// (exploits the properties of IEEE 754 floating point numbers
// by leveraging the fact that their binary representation is close to a log2 representation)
inline float cbrt5(float x) {
	#if 0
	CastF2I c(x);
	c.i = ((c.i-(127<<23))/3+(127<<23));
	#else
	TAliasCast<float,uint32_t> c(x);
	c.i = c.i/3 + 709921077u;
	#endif
	return c.f;
}
// cube root approximation using bit hack for 64-bit float
// adapted from Kahan's cbrt (5 decimals)
inline double cbrt5(double x) {
	TAliasCast<double,uint32_t[2]> c(0.0), d(x);
	c.i[1] = d.i[1]/3 + 715094163u;
	return c.f;
}
// iterative cube root approximation using Halley's method
// faster convergence than Newton's method: (R/(a*a)+a*2)/3
template<typename T>
FORCEINLINE T cbrt_halley(const T& a, const T& R) {
	const T a3 = a*a*a;
	const T a3R = a3+R;
	return a * (a3R + R) / (a3 + a3R);
}
// fast cubic root (variable precision)
template<typename T, int N>
FORCEINLINE T fast_cbrt(const T& x) {
	return cbrt_halley(fast_cbrt<T,N-1>(x), x);
}
template<>
FORCEINLINE double fast_cbrt<double,1>(const double& x) {
	return cbrt_halley((double)cbrt5((float)x), x);
}
template<>
FORCEINLINE float fast_cbrt<float,1>(const float& x) {
	return cbrt_halley(cbrt5(x), x);
}
// default cubic root function
FORCEINLINE float CBRT(float x) {
	#ifdef _FAST_CBRT
	return fast_cbrt<float,1>(x);
	#else
	return POW(x, 1.0f/3.0f);
	#endif
}
FORCEINLINE double CBRT(const double& x) {
	#ifdef _FAST_CBRT
	return fast_cbrt<double,2>(x);
	#else
	return POW(x, 1.0/3.0);
	#endif
}
/*----------------------------------------------------------------*/


#if defined(__GNUC__)

FORCEINLINE int PopCnt(uint32_t bb) {
	return __builtin_popcount(bb);
}
FORCEINLINE int PopCnt(uint64_t bb) {
	return __builtin_popcountll(bb);
}
FORCEINLINE int PopCnt15(uint64_t bb) {
	return __builtin_popcountll(bb);
}
FORCEINLINE int PopCntSparse(uint64_t bb) {
	return __builtin_popcountll(bb);
}

#elif defined(_USE_SSE) && defined(_M_AMD64) // 64 bit windows

FORCEINLINE int PopCnt(uint32_t bb) {
	return (int)_mm_popcnt_u32(bb);
}
FORCEINLINE int PopCnt(uint64_t bb) {
	return (int)_mm_popcnt_u64(bb);
}
FORCEINLINE int PopCnt15(uint64_t bb) {
	return (int)_mm_popcnt_u64(bb);
}
FORCEINLINE int PopCntSparse(uint64_t bb) {
	return (int)_mm_popcnt_u64(bb);
}

#else

// general purpose population count
template<typename T>
constexpr int PopCnt(T bb)
{
	STATIC_ASSERT(std::is_integral<T>::value && std::is_unsigned<T>::value);
	return std::bitset<sizeof(T)*8>(bb).count();
}
template<>
inline int PopCnt(uint64_t bb) {
	const uint64_t k1 = (uint64_t)0x5555555555555555;
	const uint64_t k2 = (uint64_t)0x3333333333333333;
	const uint64_t k3 = (uint64_t)0x0F0F0F0F0F0F0F0F;
	const uint64_t k4 = (uint64_t)0x0101010101010101;
	bb -= (bb >> 1) & k1;
	bb = (bb & k2) + ((bb >> 2) & k2);
	bb = (bb + (bb >> 4)) & k3;
	return (bb * k4) >> 56;
}
// faster version assuming not more than 15 bits set, used in mobility
// eval, posted on CCC forum by Marco Costalba of Stockfish team
inline int PopCnt15(uint64_t bb) {
	unsigned w = unsigned(bb >> 32), v = unsigned(bb);
	v -= (v >> 1) & 0x55555555; // 0-2 in 2 bits
	w -= (w >> 1) & 0x55555555;
	v = ((v >> 2) & 0x33333333) + (v & 0x33333333); // 0-4 in 4 bits
	w = ((w >> 2) & 0x33333333) + (w & 0x33333333);
	v += w; // 0-8 in 4 bits
	v *= 0x11111111;
	return int(v >> 28);
}
// version faster on sparsely populated bitboards
inline int PopCntSparse(uint64_t bb) {
	int count = 0;
	while (bb) {
		count++;
		bb &= bb - 1;
	}
	return count;
}

#endif
/*----------------------------------------------------------------*/


#ifdef _FAST_FLOAT2INT
// fast float to int conversion
// (xs routines at stereopsis: http://www.stereopsis.com/sree/fpu2006.html by Sree Kotay)
const double _float2int_doublemagic         = 6755399441055744.0; //2^52 * 1.5, uses limited precision to floor
const double _float2int_doublemagicdelta    = (1.5e-8);
const double _float2int_doublemagicroundeps = (.5f-_float2int_doublemagicdelta); //almost .5f = .5f - 1e^(number of exp bit)
FORCEINLINE int CRound2Int(const double& x) {
	const CastD2I c(x + _float2int_doublemagic);
	ASSERT(int32_t(floor(x+.5)) == c.i);
	return c.i;
}
#endif
template <typename INTTYPE=int>
FORCEINLINE INTTYPE Floor2Int(float x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x)-_float2int_doublemagicroundeps);
	#else
	return static_cast<INTTYPE>(floor(x));
	#endif
}
template <typename INTTYPE=int>
FORCEINLINE INTTYPE Floor2Int(double x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(x-_float2int_doublemagicroundeps);
	#else
	return static_cast<INTTYPE>(floor(x));
	#endif
}
template <typename INTTYPE=int>
FORCEINLINE INTTYPE Ceil2Int(float x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x)+_float2int_doublemagicroundeps);
	#else
	return static_cast<INTTYPE>(ceil(x));
	#endif
}
template <typename INTTYPE=int>
FORCEINLINE INTTYPE Ceil2Int(double x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(x+_float2int_doublemagicroundeps);
	#else
	return static_cast<INTTYPE>(ceil(x));
	#endif
}
template <typename INTTYPE=int>
FORCEINLINE INTTYPE Round2Int(float x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x)+_float2int_doublemagicdelta);
	#else
	return static_cast<INTTYPE>(floor(x+.5f));
	#endif
}
template <typename INTTYPE=int>
FORCEINLINE INTTYPE Round2Int(double x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(x+_float2int_doublemagicdelta);
	#else
	return static_cast<INTTYPE>(floor(x+.5));
	#endif
}
/*----------------------------------------------------------------*/


// INTERPOLATION

// Linear interpolation
template<typename Type>
inline Type lerp(const Type& u, const Type& v, float x)
{
	return u + (v - u) * x;
}

// Cubic interpolation
template<typename Type>
inline Type cerp(const Type& u0, const Type& u1, const Type& u2, const Type& u3, float x)
{
	const Type p((u3 - u2) - (u0 - u1));
	const Type q((u0 - u1) - p);
	const Type r(u2 - u0);
	return x * (x * (x * p + q) + r) + u1;
}
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

#ifdef _USE_SSE

// define utile functions to deal with SSE operations

struct ALIGN(16) sse_vec4f {
	union {
		float v[4];
		struct {
			float x;
			float y;
			float z;
			float w;
		};
	};
	inline sse_vec4f() {}
	inline sse_vec4f(const float* p) : x(p[0]), y(p[1]), z(p[2]), w(p[3]) {}
	inline sse_vec4f(float f0, float f1, float f2, float f3) : x(f0), y(f1), z(f2), w(f3) {}
	inline operator const float*() const {return v;}
	inline operator float*() {return v;}
};

struct ALIGN(16) sse_vec2d {
	union {
		double v[2];
		struct {
			double x;
			double y;
		};
	};
	inline sse_vec2d() {}
	inline sse_vec2d(const double* p) : x(p[0]), y(p[1]) {}
	inline sse_vec2d(const double& f0, const double& f1) : x(f0), y(f1) {}
	inline operator const double*() const {return v;}
	inline operator double*() {return v;}
};

struct sse_f_t {
	typedef __m128 sse_t;
	typedef const sse_t& arg_sse_t;
	typedef float real_t;
	inline sse_f_t() {}
	inline sse_f_t(const sse_t& p) : v(p)        {}
	inline sse_f_t(real_t p) : v(load1(p))       {}
	inline sse_f_t(const real_t* p) : v(load(p)) {}
	inline sse_f_t(real_t f0, real_t f1, real_t f2, real_t f3) : v(set(f0,f1,f2,f3)) {}
	inline operator sse_t() const                {return v;}
	inline operator sse_t&()                     {return v;}
	inline sse_t operator ==(sse_t s) const      {return cmpeq(v,s);}
	inline sse_t operator =(sse_t s)             {return v=s;}
	inline sse_t operator +(sse_t s) const       {return add(v,s);}
	inline sse_t operator +=(sse_t s)            {return v=add(v,s);}
	inline sse_t operator -(sse_t s) const       {return sub(v,s);}
	inline sse_t operator -=(sse_t s)            {return v=sub(v,s);}
	inline sse_t operator *(sse_t s) const       {return mul(v,s);}
	inline sse_t operator *=(sse_t s)            {return v=mul(v,s);}
	inline sse_t operator /(sse_t s) const       {return div(v,s);}
	inline sse_t operator /=(sse_t s)            {return v=div(v,s);}
	inline void get(real_t* p) const             {store(p,v);}
	static inline sse_t zero()                   {return _mm_setzero_ps();}
	static inline sse_t load1(real_t p)          {return _mm_load1_ps(&p);}
	static inline sse_t load(const real_t* p)    {return _mm_load_ps(p);}
	static inline sse_t loadu(const real_t* p)   {return _mm_loadu_ps(p);}
	static inline sse_t set(real_t f0, real_t f1, real_t f2, real_t f3) {return _mm_set_ps(f0,f1,f2,f3);}
	static inline void  store(real_t *p, sse_t s){_mm_store_ps(p,s);}
	static inline void  storeu(real_t *p, sse_t s){_mm_storeu_ps(p,s);}
	static inline sse_t add(sse_t s1, sse_t s2)  {return _mm_add_ps(s1,s2);}
	static inline sse_t sub(sse_t s1, sse_t s2)  {return _mm_sub_ps(s1,s2);}
	static inline sse_t mul(sse_t s1, sse_t s2)  {return _mm_mul_ps(s1,s2);}
	static inline sse_t div(sse_t s1, sse_t s2)  {return _mm_div_ps(s1,s2);}
	static inline sse_t min(sse_t s1, sse_t s2)  {return _mm_min_ps(s1,s2);}
	static inline sse_t max(sse_t s1, sse_t s2)  {return _mm_max_ps(s1,s2);}
	static inline sse_t cmpeq(sse_t s1, sse_t s2){return _mm_cmpeq_ps(s1,s2);}
	static inline sse_t sqrt(sse_t s)            {return _mm_sqrt_ps(s);}
	static inline sse_t rsqrt(sse_t s)           {return _mm_rsqrt_ps(s);}
	static inline int floor2int(real_t f)        {return _mm_cvtt_ss2si(_mm_load_ss(&f));}
	#ifdef _WIN32
	static inline real_t sum(sse_t s)            {return (s.m128_f32[0]+s.m128_f32[2])+(s.m128_f32[1]+s.m128_f32[3]);}
	static inline real_t sum3(sse_t s)           {return (s.m128_f32[0]+s.m128_f32[2])+s.m128_f32[1];}
	#else
	static inline real_t sum(sse_t s)            {real_t *f = (real_t*)(&s); return (f[0]+f[2])+(f[1]+f[3]);}
	static inline real_t sum3(sse_t s)           {real_t *f = (real_t*)(&s); return (f[0]+f[2])+f[1];}
	#endif
	/*
	static inline real_t dot(sse_t s1, sse_t s2) {
		sse_t temp = _mm_dp_ps(s1, s2, 0xF1);
		real_t* f = (real_t*)(&temp); return f[0];
	}
	*/
	static real_t dot(const real_t* a, const real_t* b, size_t size) {
		const real_t* const end = a+size;
		const size_t iters = (size>>2);
		real_t fres = 0.f;
		if (iters) {
			const real_t* const e = a+(iters<<2);
			sse_t mres = zero();
			do {
				mres = _mm_add_ps(mres, _mm_mul_ps(_mm_loadu_ps(a), _mm_loadu_ps(b)));
				a += 4; b += 4;
			} while (a < e);
			fres = sum(mres);
		}
		while (a<end)
			fres += (*a++) * (*b++);
		return fres;
	}
	sse_t v;
};

class sse_d_t {
public:
	typedef __m128d sse_t;
	typedef double real_t;
	inline sse_d_t() {}
	inline sse_d_t(const sse_t& p) : v(p)        {}
	inline sse_d_t(const real_t& p) : v(load1(p)){}
	inline sse_d_t(const real_t* p) : v(load(p)) {}
	inline sse_d_t(const real_t& f0, const real_t& f1) : v(set(f0,f1)) {}
	inline operator sse_t() const                {return v;}
	inline operator sse_t&()                     {return v;}
	inline sse_t operator ==(sse_t s) const      {return cmpeq(v,s);}
	inline sse_t operator =(sse_t s)             {return v=s;}
	inline sse_t operator +(sse_t s) const       {return add(v,s);}
	inline sse_t operator +=(sse_t s)            {return v=add(v,s);}
	inline sse_t operator -(sse_t s) const       {return sub(v,s);}
	inline sse_t operator -=(sse_t s)            {return v=sub(v,s);}
	inline sse_t operator *(sse_t s) const       {return mul(v,s);}
	inline sse_t operator *=(sse_t s)            {return v=mul(v,s);}
	inline sse_t operator /(sse_t s) const       {return div(v,s);}
	inline sse_t operator /=(sse_t s)            {return v=div(v,s);}
	inline void get(real_t* p) const             {store(p,v);}
	static inline sse_t zero()                   {return _mm_setzero_pd();}
	static inline sse_t load1(const real_t& p)   {return _mm_load1_pd(&p);}
	static inline sse_t load(const real_t* p)    {return _mm_load_pd(p);}
	static inline sse_t loadu(const real_t* p)   {return _mm_loadu_pd(p);}
	static inline sse_t set(const real_t& f0, const real_t& f1) {return _mm_set_pd(f0,f1);}
	static inline void  store(real_t *p, sse_t s){_mm_store_pd(p,s);}
	static inline void  storeu(real_t *p, sse_t s){_mm_storeu_pd(p,s);}
	static inline sse_t add(sse_t s1, sse_t s2)  {return _mm_add_pd(s1,s2);}
	static inline sse_t sub(sse_t s1, sse_t s2)  {return _mm_sub_pd(s1,s2);}
	static inline sse_t mul(sse_t s1, sse_t s2)  {return _mm_mul_pd(s1,s2);}
	static inline sse_t div(sse_t s1, sse_t s2)  {return _mm_div_pd(s1,s2);}
	static inline sse_t min(sse_t s1, sse_t s2)  {return _mm_min_pd(s1,s2);}
	static inline sse_t max(sse_t s1, sse_t s2)  {return _mm_max_pd(s1,s2);}
	static inline sse_t cmpeq(sse_t s1, sse_t s2){return _mm_cmpeq_pd(s1,s2);}
	static inline sse_t sqrt(sse_t s)            {return _mm_sqrt_pd(s);}
	static inline int floor2int(const real_t& f) {return _mm_cvttsd_si32(_mm_load_sd(&f));}
	#ifdef _WIN32
	static inline real_t sum(sse_t s)            {return s.m128d_f64[0]+s.m128d_f64[1];}
	#else
	static inline real_t sum(sse_t s)            {real_t *d = (real_t*)(&s); return d[0]+d[1];}
	#endif
	/*
	static inline real_t dot(sse_t s1, sse_t s2) {
		sse_t temp = _mm_dp_pd(s1, s2, 0x31);
		real_t* f = (real_t*)(&temp); return f[0] ;
	}
	*/
	static real_t dot(const real_t* a, const real_t* b, size_t size) {
		const real_t* const end = a+size;
		const size_t iters = (size>>1);
		real_t fres = 0.0;
		if (iters) {
			const real_t* const e = a+(iters<<1);
			sse_t mres = zero();
			do {
				mres = _mm_add_pd(mres, _mm_mul_pd(_mm_loadu_pd(a), _mm_loadu_pd(b)));
				a += 2; b += 2;
			} while (a < e);
			fres = sum(mres);
		}
		while (a<end)
			fres += (*a++) * (*b++);
		return fres;
	}
	sse_t v;
};

inline void sse_prefetch(const void* p) {_mm_prefetch((const char*)p, _MM_HINT_NTA);}

#endif


// C L A S S E S ///////////////////////////////////////////////////

inline bool   ISINFORNAN(float x)				{ return std::isinf(x) || std::isnan(x); }
inline bool   ISINFORNAN(double x)				{ return std::isinf(x) || std::isnan(x); }
inline bool   ISFINITE(float x)					{ return std::isfinite(x); }
inline bool   ISFINITE(double x)				{ return std::isfinite(x); }
template<typename _Tp>
inline bool   ISFINITE(const _Tp* x, size_t n)	{ for (size_t i=0; i<n; ++i) if (ISINFORNAN(x[i])) return false; return true; }

template<typename _Tp>
constexpr bool  ISINSIDE(_Tp v,_Tp l0,_Tp l1)	{ SIMPLE_ASSERT(l0<l1); return l0 <= v && v < l1; }
template<typename _Tp>
constexpr bool  ISINSIDES(_Tp v,_Tp l0,_Tp l1)	{ return l0 < l1 ? ISINSIDE(v, l0, l1) : ISINSIDE(v, l1, l0); }

template<typename _Tp>
inline _Tp    CLAMP(_Tp v, _Tp l0, _Tp l1)	{
	ASSERT(l0<=l1);
	#ifdef _SUPPORT_CPP17
	return std::clamp(v, l0, l1);
	#else
	return MINF(MAXF(v, l0), l1);
	#endif
}
template<typename _Tp>
constexpr _Tp   CLAMPS(_Tp v, _Tp l0, _Tp l1)	{ return l0 <= l1 ? CLAMP(v, l0, l1) : CLAMP(v, l1, l0); }

template<typename _Tp>
constexpr _Tp   SIGN(_Tp x)						{ if (x > _Tp(0)) return _Tp(1); if (x < _Tp(0)) return _Tp(-1); return _Tp(0); }

template<typename _Tp>
constexpr _Tp   ABS(_Tp x)						{ return std::abs(x); }

// mod, which is always positive, instead of remainder provided by %, which can be positive or negative
template <typename _Tp>
constexpr _Tp   MOD(_Tp a, _Tp b)				{ return (a % b < 0) ? (a % b + b) : (a % b); }
template <typename _Tp>
constexpr _Tp   FMOD(_Tp a, _Tp b)				{ return (fmod(a, b) < 0) ? (fmod(a, b) + b) : fmod(a, b); }

template<typename _Tp>
constexpr _Tp    ZEROTOLERANCE()				{ return _Tp(0); }
template<>
constexpr float  ZEROTOLERANCE()				{ return FZERO_TOLERANCE; }
template<>
constexpr double ZEROTOLERANCE()				{ return ZERO_TOLERANCE; }

template<typename _Tp>
constexpr _Tp    EPSILONTOLERANCE()				{ return std::numeric_limits<_Tp>::epsilon(); }
template<>
constexpr float  EPSILONTOLERANCE()				{ return 0.00001f; }
template<>
constexpr double EPSILONTOLERANCE()				{ return 1e-10; }

constexpr bool   ISZERO(float  x)				{ return ABS(x) < FZERO_TOLERANCE; }
constexpr bool   ISZERO(double x)				{ return ABS(x) < ZERO_TOLERANCE; }

constexpr bool   ISEQUAL(float  x, float  v)	{ return ABS(x-v) < FZERO_TOLERANCE; }
constexpr bool   ISEQUAL(double x, double v)	{ return ABS(x-v) < ZERO_TOLERANCE; }

constexpr bool   ISEQUAL(float  x, float  v, float e)	{ return ABS(x-v) < e; }
constexpr bool   ISEQUAL(double x, double v, double e)	{ return ABS(x-v) < e; }

constexpr float  INVZERO(float)					{ return FINV_ZERO; }
constexpr double INVZERO(double)				{ return INV_ZERO; }
template<typename _Tp>
constexpr _Tp    INVZERO(_Tp)					{ return std::numeric_limits<_Tp>::max(); }

template<typename _Tp>
constexpr _Tp    INVERT(_Tp x)					{ return (x==_Tp(0) ? INVZERO(x) : _Tp(1)/x); }
template<typename _Tp>
constexpr _Tp    SAFEDIVIDE(_Tp x, _Tp y)		{ return (y==_Tp(0) ? INVZERO(y) : x/y); }
/*----------------------------------------------------------------*/

} // namespace SEACAVE


#ifdef _USE_EIGEN

namespace Eigen {

// columns vectors
template <class T>
using Vector1 = Matrix<T, 1, 1>;
template <class T>
using Vector2 = Matrix<T, 2, 1>;
template <class T>
using Vector3 = Matrix<T, 3, 1>;
template <class T>
using Vector4 = Matrix<T, 4, 1>;
template <class T>
using Vector5 = Matrix<T, 5, 1>;
template <class T>
using Vector6 = Matrix<T, 6, 1>;
template <class T>
using Vector7 = Matrix<T, 7, 1>;
template <class T>
using Vector8 = Matrix<T, 8, 1>;
template <class T>
using Vector9 = Matrix<T, 9, 1>;

using Vector1i = Vector1<int>;
using Vector1f = Vector1<float>;
using Vector1d = Vector1<double>;
using Vector5i = Vector5<int>;
using Vector5f = Vector5<float>;
using Vector5d = Vector5<double>;
using Vector6i = Vector6<int>;
using Vector6f = Vector6<float>;
using Vector6d = Vector6<double>;
using Vector7i = Vector7<int>;
using Vector7f = Vector7<float>;
using Vector7d = Vector7<double>;
using Vector8i = Vector8<int>;
using Vector8f = Vector8<float>;
using Vector8d = Vector8<double>;
using Vector9i = Vector9<int>;
using Vector9f = Vector9<float>;
using Vector9d = Vector9<double>;

// row vectors
template <class T>
using RowVector1 = Matrix<T, 1, 1>;
template <class T>
using RowVector2 = Matrix<T, 1, 2>;
template <class T>
using RowVector3 = Matrix<T, 1, 3>;
template <class T>
using RowVector4 = Matrix<T, 1, 4>;
template <class T>
using RowVector5 = Matrix<T, 1, 5>;
template <class T>
using RowVector6 = Matrix<T, 1, 6>;
template <class T>
using RowVector7 = Matrix<T, 1, 7>;
template <class T>
using RowVector8 = Matrix<T, 1, 8>;
template <class T>
using RowVector9 = Matrix<T, 1, 9>;

// column arrays
using Array3u = Array<uint8_t, 3, 1>;
using Array4u = Array<uint8_t, 4, 1>;

// square matrices
template <class T>
using Matrix1 = Matrix<T, 1, 1>;
template <class T>
using Matrix2 = Matrix<T, 2, 2>;
template <class T>
using Matrix3 = Matrix<T, 3, 3>;
template <class T>
using Matrix4 = Matrix<T, 4, 4>;
template <class T>
using Matrix5 = Matrix<T, 5, 5>;
template <class T>
using Matrix6 = Matrix<T, 6, 6>;
template <class T>
using Matrix7 = Matrix<T, 7, 7>;
template <class T>
using Matrix8 = Matrix<T, 8, 8>;
template <class T>
using Matrix9 = Matrix<T, 9, 9>;

// non-square matrices
using Matrix23d = Matrix<double, 2, 3>;
using Matrix34d = Matrix<double, 3, 4>;

// dynamics arrays/vectors/matrices
using ArrayXu = Array<uint8_t, Dynamic, 1>;
using VectorXu = Matrix<uint8_t, Dynamic, 1>;
using MatrixXu = Matrix<uint8_t, Dynamic, Dynamic>;

// geometry structures
template <class T>
using Line2 = ParametrizedLine<T, 2>;
using Line2f = Line2<float>;
using Line2d = Line2<double>;

template <class T>
using Line3 = ParametrizedLine<T, 3>;
using Line3f = Line3<float>;
using Line3d = Line3<double>;

template <class T>
using Affine3 = Transform<T, 3, Affine>;
using Affine3f = Affine3<float>;
using Affine3d = Affine3<double>;

template <class T>
using Projective3 = Transform<T, 3, Projective>;
using Projective3f = Projective3<float>;
using Projective3d = Projective3<double>;

template <class T>
using Plane3 = Hyperplane<T, 3>;
using Plane3f = Plane3<float>;
using Plane3d = Plane3<double>;
/*----------------------------------------------------------------*/


// util functions
inline Array3i Mod(const Array3i& a, const Array3i& b) {
	using namespace SEACAVE;
	return Array3i(MOD(a[0], b[0]), MOD(a[1], b[1]), MOD(a[2], b[2]));
}

// read a matrix from a stream (writing only implemented in Eigen/Core/src/IO.h)
template <typename Derived>
inline std::istream& operator >>(std::istream& st, MatrixBase<Derived>& m) {
	for (int i = 0; i < m.rows(); ++i)
		for (int j = 0; j < m.cols(); ++j)
			st >> m(i, j);
	return st;
}


// C L A S S  //////////////////////////////////////////////////////

// Implement SO3 and SO2 lie groups
// inspired by TooN library: https://github.com/edrosten/TooN
// Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk)

/// Class to represent a three-dimensional rotation matrix. Three-dimensional rotation
/// matrices are members of the Special Orthogonal Lie group SO3. This group can be parameterized
/// three numbers (a vector in the space of the Lie Algebra). In this class, the three parameters are the
/// finite rotation vector, i.e. a three-dimensional vector whose direction is the axis of rotation
/// and whose length is the angle of rotation in radians. Exponentiating this vector gives the matrix,
/// and the logarithm of the matrix gives this vector.
template <typename Precision>
class SO3
{
public:
	template <typename P>
	friend std::istream& operator>>(std::istream& is, SO3<P>& rhs);

	typedef Matrix<Precision,3,3,Eigen::RowMajor> Mat3;
	typedef Matrix<Precision,3,1> Vec3;

	/// Default constructor. Initializes the matrix to the identity (no rotation)
	inline SO3() : mat(Mat3::Identity()) {}

	/// Construct from a rotation matrix.
	inline SO3(const Mat3& rhs) : mat(rhs) {}

	/// Construct from the axis of rotation (and angle given by the magnitude).
	inline SO3(const Vec3& v) { exp(v); }

	/// creates an SO3 as a rotation that takes Vector a into the direction of Vector b
	/// with the rotation axis along a ^ b. If |a ^ b| == 0, it creates the identity rotation.
	/// An assertion will fail if Vector a and Vector b are in exactly opposite directions. 
	/// @param a source Vector
	/// @param b target Vector
	SO3(const Vec3& a, const Vec3& b) {
		ASSERT(a.size() == 3);
		ASSERT(b.size() == 3);
		Vec3 n(a.cross(b));
		const Precision nrmSq(n.squaredNorm());
		if (nrmSq == Precision(0)) {
			// check that the vectors are in the same direction if cross product is 0; if not,
			// this means that the rotation is 180 degrees, which leads to an ambiguity in the rotation axis
			ASSERT(a.dot(b) >= Precision(0));
			mat = Mat3::Identity();
			return;
		}
		n *= Precision(1)/SEACAVE::SQRT(nrmSq);
		Mat3 R1;
		R1.col(0) = a.normalized();
		R1.col(1) = n;
		R1.col(2) = R1.col(0).cross(n);
		mat.col(0) = b.normalized();
		mat.col(1) = n;
		mat.col(2) = mat.col(0).cross(n);
		mat = mat * R1.transpose();
	}

	/// Assignment operator from a general matrix. This also calls coerce()
	/// to make sure that the matrix is a valid rotation matrix.
	inline SO3& operator=(const Mat3& rhs) {
		mat = rhs;
		coerce();
		return *this;
	}

	/// Modifies the matrix to make sure it is a valid rotation matrix.
	void coerce() {
		mat.row(0).normalize();
		const Precision d01(mat.row(0).dot(mat.row(1)));
		mat.row(1) -= mat.row(0) * d01;
		mat.row(1).normalize();
		const Precision d02(mat.row(0).dot(mat.row(2)));
		mat.row(2) -= mat.row(0) * d02;
		const Precision d12(mat.row(1).dot(mat.row(2)));
		mat.row(2) -= mat.row(1) * d12;
		mat.row(2).normalize();
		// check for positive determinant <=> right handed coordinate system of row vectors
		ASSERT(mat.row(0).cross(mat.row(1)).dot(mat.row(2)) > 0); 
	}

	/// Exponentiate a vector in the Lie algebra to generate a new SO3.
	/// See the Detailed Description for details of this vector.
	inline SO3& exp(const Vec3& vect);

	/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
	/// See the Detailed Description for details of this vector.
	inline Vec3 ln() const;

	/// Right-multiply by another rotation matrix
	template <typename P>
	inline SO3& operator *=(const SO3<P>& rhs) {
		*this = *this * rhs;
		return *this;
	}

	/// Right-multiply by another rotation matrix
	inline SO3 operator *(const SO3& rhs) const { return SO3(*this, rhs); }

	/// Returns the SO3 as a Matrix<3>
	inline const Mat3& get_matrix() const { return mat; }

	/// Returns the i-th generator.  The generators of a Lie group are the basis
	/// for the space of the Lie algebra.  For %SO3, the generators are three
	/// \f$3\times3\f$ matrices representing the three possible (linearized)
	/// rotations.
	inline static Mat3 generator(int i) {
		Mat3 result(Mat3::Zero());
		result((i+1)%3,(i+2)%3) = Precision(-1);
		result((i+2)%3,(i+1)%3) = Precision( 1);
		return result;
	}

	/// Returns the i-th generator times pos
	inline static Vec3 generator_field(int i, const Vec3& pos) {
		Vec3 result;
		result(i) = Precision(0);
		result((i+1)%3) = -pos((i+2)%3);
		result((i+2)%3) =  pos((i+1)%3);
		return result;
	}

	template <typename PA, typename PB>
	inline SO3(const SO3<PA>& a, const SO3<PB>& b) : mat(a.get_matrix()*b.get_matrix()) {}

protected:
	Mat3 mat;
};
/*----------------------------------------------------------------*/


/// Class to represent a two-dimensional rotation matrix. Two-dimensional rotation
/// matrices are members of the Special Orthogonal Lie group SO2. This group can be parameterized
/// with one number (the rotation angle).
template<typename Precision>
class SO2
{
public:
	template <typename P>
	friend std::istream& operator>>(std::istream&, SO2<P>&);

	typedef Matrix<Precision,2,2,Eigen::RowMajor> Mat2;

	/// Default constructor. Initializes the matrix to the identity (no rotation)
	inline SO2() : mat(Mat2::Identity()) {}

	/// Construct from a rotation matrix.
	inline SO2(const Mat2& rhs) : mat(rhs) {}

	/// Construct from an angle.
	inline SO2(const Precision l) { exp(l); }

	/// Assignment operator from a general matrix. This also calls coerce()
	/// to make sure that the matrix is a valid rotation matrix.
	inline SO2& operator=(const Mat2& rhs) {
		mat = rhs;
		coerce();
		return *this;
	}

	/// Modifies the matrix to make sure it is a valid rotation matrix.
	inline void coerce() {
		mat.row(0).normalize();
		mat.row(1) = (mat.row(1) - mat.row(0) * (mat.row(0).dot(mat.row(1)))).normalized();
	}

	/// Exponentiate an angle in the Lie algebra to generate a new SO2.
	inline SO2& exp(const Precision& d);

	/// extracts the rotation angle from the SO2
	inline Precision ln() const;

	/// Self right-multiply by another rotation matrix
	inline SO2& operator *=(const SO2& rhs) {
		mat = mat*rhs.get_matrix();
		return *this;
	}

	/// Right-multiply by another rotation matrix
	inline SO2 operator *(const SO2& rhs) const { return SO2(*this, rhs); }

	/// Returns the SO2 as a Matrix<2>
	inline const Mat2& get_matrix() const { return mat; }

	/// returns generator matrix
	inline static Mat2 generator() {
		Mat2 result;
		result(0,0) = Precision(0); result(0,1) = Precision(-1);
		result(1,0) = Precision(1); result(1,1) = Precision(0);
		return result;
	}

protected:
	Mat2 mat;
};
/*----------------------------------------------------------------*/


///Compute a rotation exponential using the Rodrigues Formula.
///The rotation axis is given by \f$\vec{w}\f$, and the rotation angle must
///be computed using \f$ \theta = |\vec{w}|\f$. This is provided as a separate
///function primarily to allow fast and rough matrix exponentials using fast
///and rough approximations to \e A and \e B.
///
///@param w Vector about which to rotate.
///@param A \f$\frac{\sin \theta}{\theta}\f$
///@param B \f$\frac{1 - \cos \theta}{\theta^2}\f$
///@param R Matrix to hold the return value.
///@relates SO3
template <typename Precision>
inline void SO3_exp(const typename SO3<Precision>::Vec3& w, typename SO3<Precision>::Mat3& R) {
	static const Precision one_6th(1.0/6.0);
	static const Precision one_20th(1.0/20.0);
	//Use a Taylor series expansion near zero. This is required for
	//accuracy, since sin t / t and (1-cos t)/t^2 are both 0/0.
	Precision A, B;
	const Precision theta_sq(w.squaredNorm());
	if (theta_sq < Precision(1e-8)) {
		A = Precision(1) - one_6th * theta_sq;
		B = Precision(0.5);
	} else {
		if (theta_sq < Precision(1e-6)) {
			B = Precision(0.5) - Precision(0.25) * one_6th * theta_sq;
			A = Precision(1) - theta_sq * one_6th*(Precision(1) - one_20th * theta_sq);
		} else {
			const Precision theta(SEACAVE::SQRT(theta_sq));
			const Precision inv_theta(Precision(1)/theta);
			A = SIN(theta) * inv_theta;
			B = (Precision(1) - COS(theta)) * (inv_theta * inv_theta);
		}
	}
	{
	const Precision wx2(w(0)*w(0));
	const Precision wy2(w(1)*w(1));
	const Precision wz2(w(2)*w(2));
	R(0,0) = Precision(1) - B*(wy2 + wz2);
	R(1,1) = Precision(1) - B*(wx2 + wz2);
	R(2,2) = Precision(1) - B*(wx2 + wy2);
	}
	{
	const Precision a(A*w[2]);
	const Precision b(B*(w[0]*w[1]));
	R(0,1) = b - a;
	R(1,0) = b + a;
	}
	{
	const Precision a(A*w[1]);
	const Precision b(B*(w[0]*w[2]));
	R(0,2) = b + a;
	R(2,0) = b - a;
	}
	{
	const Precision a(A*w[0]);
	const Precision b(B*(w[1]*w[2]));
	R(1,2) = b - a;
	R(2,1) = b + a;
	}
}
template <typename Precision>
inline SO3<Precision>& SO3<Precision>::exp(const Vec3& w) {
	SO3_exp<Precision>(w, mat);
	return *this;
}

/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
/// See the Detailed Description for details of this vector.
template <typename Precision>
inline void SO3_ln(const typename SO3<Precision>::Mat3& R, typename SO3<Precision>::Vec3& w) {
	const Precision cos_angle((R(0,0) + R(1,1) + R(2,2) - Precision(1)) * Precision(0.5));
	w(0) = (R(2,1)-R(1,2))*Precision(0.5);
	w(1) = (R(0,2)-R(2,0))*Precision(0.5);
	w(2) = (R(1,0)-R(0,1))*Precision(0.5);

	const Precision sin_angle_abs(w.norm());
	if (cos_angle > Precision(M_SQRT1_2)) {           // [0 - Pi/4] use asin
		if (sin_angle_abs > Precision(0))
			w *= ASIN(sin_angle_abs) / sin_angle_abs;
	} else if (cos_angle > Precision(-M_SQRT1_2)) {   // [Pi/4 - 3Pi/4] use acos, but antisymmetric part
		if (sin_angle_abs > Precision(0))
			w *= ACOS(cos_angle) / sin_angle_abs;
	} else {                                       // rest use symmetric part
		// antisymmetric part vanishes, but still large rotation, need information from symmetric part
		const Precision angle(Precision(M_PI) - ASIN(sin_angle_abs));
		const Precision d0(R(0,0) - cos_angle);
		const Precision d1(R(1,1) - cos_angle);
		const Precision d2(R(2,2) - cos_angle);
		typename SO3<Precision>::Vec3 r2;
		if (d0*d0 > d1*d1 && d0*d0 > d2*d2) {      // first is largest, fill with first column
			r2(0) = d0;
			r2(1) = (R(1,0)+R(0,1))*Precision(0.5);
			r2(2) = (R(0,2)+R(2,0))*Precision(0.5);
		} else if (d1*d1 > d2*d2) {                // second is largest, fill with second column
			r2(0) = (R(1,0)+R(0,1))*Precision(0.5);
			r2(1) = d1;
			r2(2) = (R(2,1)+R(1,2))*Precision(0.5);
		} else {                                   // third is largest, fill with third column
			r2(0) = (R(0,2)+R(2,0))*Precision(0.5);
			r2(1) = (R(2,1)+R(1,2))*Precision(0.5);
			r2(2) = d2;
		}
		// flip, if we point in the wrong direction!
		if (r2.dot(w) < Precision(0))
			r2 *= Precision(-1);
		w = r2 * (angle/r2.norm());
	}
}
template <typename Precision>
inline typename SO3<Precision>::Vec3 SO3<Precision>::ln() const {
	Vec3 result;
	SO3_ln<Precision>(mat, result);
	return result;
}

/// Write/read a SO3 to a stream
/// @relates SO3
template <typename Precision>
inline std::ostream& operator <<(std::ostream& os, const SO3<Precision>& rhs) {
	return os << rhs.get_matrix();
}
template <typename Precision>
inline std::istream& operator >>(std::istream& is, SO3<Precision>& rhs) {
	is >> rhs.mat;
	rhs.coerce();
	return is;
}

/// Right-multiply by a Vector
/// @relates SO3
template <typename P, int O>
inline Matrix<P,3,1,O> operator *(const SO3<P>& lhs, const Matrix<P,3,1,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Vector
/// @relates SO3
template <typename P, int O>
inline Matrix<P,3,1,O> operator *(const Matrix<P,3,1,O>& lhs, const SO3<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/// Right-multiply by a matrix
/// @relates SO3
template <typename P, int C, int O>
inline Matrix<P,3,C,O> operator *(const SO3<P>& lhs, const Matrix<P,3,C,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a matrix
/// @relates SO3
template <typename P, int R, int O>
inline Matrix<P,R,3,O> operator *(const Matrix<P,R,3,O>& lhs, const SO3<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/*----------------------------------------------------------------*/


/// Exponentiate an angle in the Lie algebra to generate a new SO2.
template <typename Precision>
inline void SO2_exp(const Precision& d, typename SO2<Precision>::Mat2& R) {
	R(0,0) = R(1,1) = COS(d);
	R(1,0) = SIN(d);
	R(0,1) = -R(1,0);
}
template <typename Precision>
inline SO2<Precision>& SO2<Precision>::exp(const Precision& d) {
	SO2_exp<Precision>(d, mat);
	return *this;
}

/// Extracts the rotation angle from the SO2
template <typename Precision>
inline void SO2_ln(const typename SO2<Precision>::Mat2& R, Precision& d) {
	d = ATAN2(R(1,0), R(0,0));
}
template <typename Precision>
inline Precision SO2<Precision>::ln() const {
	Precision d;
	SO2_ln<Precision>(mat, d);
	return d;
}

/// Write/read a SO2 to a stream
/// @relates SO2
template <typename Precision>
inline std::ostream& operator <<(std::ostream& os, const SO2<Precision> & rhs) {
	return os << rhs.get_matrix();
}
template <typename Precision>
inline std::istream& operator >>(std::istream& is, SO2<Precision>& rhs) {
	is >> rhs.mat;
	rhs.coerce();
	return is;
}

/// Right-multiply by a Vector
/// @relates SO2
template <typename P, int O>
inline Matrix<P,2,1,O> operator *(const SO2<P>& lhs, const Matrix<P,2,1,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Vector
/// @relates SO2
template <typename P, int O>
inline Matrix<P,2,1,O> operator *(const Matrix<P,2,1,O>& lhs, const SO2<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/// Right-multiply by a Matrix
/// @relates SO2
template <typename P, int C, int O>
inline Matrix<P,2,C,O> operator *(const SO2<P>& lhs, const Matrix<P,2,C,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Matrix
/// @relates SO2
template <typename P, int R, int O>
inline Matrix<P,R,2,O> operator *(const Matrix<P,R,2,O>& lhs, const SO2<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/*----------------------------------------------------------------*/

} // namespace Eigen

#endif // _USE_EIGEN

#endif // __SEACAVE_MATHS_H__
