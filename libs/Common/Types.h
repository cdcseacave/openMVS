////////////////////////////////////////////////////////////////////
// Types.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_TYPES_H__
#define __SEACAVE_TYPES_H__


// I N C L U D E S /////////////////////////////////////////////////

#ifdef _MSC_VER
#include <windows.h>
#include <tchar.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#endif
#ifdef _SUPPORT_CPP11
#ifdef __clang__
#include <stdint.h>
#else
#include <cstdint>
#endif
#include <cstddef>
#include <type_traits>
#include <initializer_list>
#else
#include <stdint.h>
#endif
#include <new>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <functional>
#include <algorithm>
#include <numeric>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <list>
#include <queue>
#include <deque>
#include <iterator>
#include <cmath>
#include <ctime>
#include <random>
#ifdef _USE_OPENMP
#include <omp.h>
#endif

// Function delegate functionality
#ifdef _SUPPORT_CPP11
#include "FastDelegateCPP11.h"
#define DELEGATE fastdelegate::delegate
#define DELEGATEBIND(DLGT, FNC) DLGT::from< FNC >()
#define DELEGATEBINDCLASS(DLGT, FNC, OBJ) DLGT::from(*OBJ, FNC)
#else
#include "FastDelegate.h"
#include "FastDelegateBind.h"
#define DELEGATE fastdelegate::FastDelegate
#define DELEGATEBIND(DLGT, FNC) fastdelegate::bind(FNC)
#define DELEGATEBINDCLASS(DLGT, FNC, OBJ) fastdelegate::bind(FNC, OBJ)
#endif

// include usual boost libraries
#ifdef _USE_BOOST
#if 1
// disable exception support
#define BOOST_EXCEPTION_DISABLE
#define BOOST_NO_EXCEPTIONS
#endif
#ifdef BOOST_NO_EXCEPTIONS
namespace boost { void throw_exception(std::exception const&); }
#endif
#define BOOST_NO_UNREACHABLE_RETURN_DETECTION
// include headers that implement serialization support
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#if (BOOST_VERSION / 100000) > 1 || (BOOST_VERSION / 100 % 1000) > 55
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>
#endif
#include <boost/serialization/nvp.hpp>
// include headers that define an input and output archive
#include <boost/archive/detail/common_oarchive.hpp>
#include <boost/archive/detail/common_iarchive.hpp>
// include headers that define memory pools
#include <boost/pool/object_pool.hpp>
#include <boost/pool/singleton_pool.hpp>
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
#endif

#pragma push_macro("free")
#undef free
#pragma push_macro("DEBUG")
#undef DEBUG
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION > 2 || CV_MINOR_VERSION > 3
#include <opencv2/opencv_modules.hpp>
#endif
#include <opencv2/opencv.hpp>
#ifdef HAVE_OPENCV_GPU
#if CV_MAJOR_VERSION > 2
#include <opencv2/cudaarithm.hpp>
namespace cv { namespace gpu = cuda; }
#else
#include <opencv2/gpu/gpu.hpp>
#endif
#endif
#pragma pop_macro("DEBUG")
#pragma pop_macro("free")

#ifdef _USE_SSE
#include <xmmintrin.h>
#include <emmintrin.h>
#endif


// D E F I N E S ///////////////////////////////////////////////////

// In order to create a singleton class
// add this define in your class;
#define DECLARE_SINGLETON(SClass) \
	protected: SClass(); SClass(const SClass&); \
	public: static inline SClass& GetInstance() { static SClass instance; return instance; }
// or add this define and declare GetInstance() method in the .cpp
// so that the singleton is unique even across modules;
#define DEFINE_SINGLETON(SClass) \
	protected: SClass(); SClass(const SClass&); \
	public: static SClass& GetInstance();
// or add this declare ms_pInstance;
#define DECLARE_SINGLETONPTR(SClass) \
	SClass* SClass::ms_pInstance(NULL); \
	SClass*& SClass::GetInstanceRef() { return ms_pInstance; } \
	SClass* SClass::GetInstancePtr() { ASSERT(ms_pInstance); return ms_pInstance; } \
	SClass& SClass::GetInstance() { ASSERT(ms_pInstance); return *ms_pInstance; }
// and this define in the class definition.
#define DEFINE_SINGLETONPTR(SClass) \
	protected: SClass(); SClass(const SClass&); static SClass* ms_pInstance; \
	public: static SClass*& GetInstanceRef(); \
	public: static SClass* GetInstancePtr(); \
	public: static SClass& GetInstance();

#ifndef CALL_MEMBER_FN
#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember))
#endif

#ifndef __PROCESS__
# ifdef _MSC_VER
#  define __PROCESS__ ((unsigned)GetCurrentProcessId())
# else
#  define __PROCESS__ ((unsigned)getpid())
# endif
#endif

#ifndef __THREAD__
# ifdef _MSC_VER
#  define __THREAD__ ((unsigned)GetCurrentThreadId())
# elif defined(__APPLE__)
#  include <pthread.h>
inline pid_t GetCurrentThreadId() { uint64_t tid64; pthread_threadid_np(NULL, &tid64); return (pid_t)tid64; }
#  define __THREAD__ ((unsigned)GetCurrentThreadId())
# else
#  include <sys/syscall.h>
#  define __THREAD__ ((unsigned)((pid_t)syscall(SYS_gettid)))
# endif
#endif

#ifndef STCALL
# if defined(_MSC_VER)
#  define STCALL __cdecl
# elif defined(__OS2__)
#  if defined (__GNUC__) && __GNUC__ < 4
#   define STCALL _cdecl
#  else
#   /* On other compilers on OS/2, we use the _System calling convention */
#   /* to be compatible with every compiler */
#   define STCALL _System
#  endif
# elif defined(__GNUC__)
#   define STCALL __attribute__((__cdecl__))
# else
#  define STCALL
# endif
#endif // STCALL


// T Y P E   D E F I N E S /////////////////////////////////////////

//
// Type defines

#ifndef _MSC_VER
typedef int32_t				HRESULT;

typedef unsigned char		BYTE;
typedef unsigned short		WORD;
typedef unsigned int		DWORD;
typedef uint64_t	        QWORD;

typedef char				CHAR;
typedef CHAR*				LPSTR;
typedef const CHAR*			LPCSTR;
typedef CHAR				TCHAR;
typedef LPSTR				LPTSTR;
typedef LPCSTR				LPCTSTR;

#define _tcslen         	strlen
#define _tcscpy         	strcpy
#define _tcsncpy         	strncpy
#define _tcschr         	strchr
#define _tcsrchr         	strrchr
#define _tcscmp         	strcmp
#define _tcsncmp         	strncmp
#define _tcsicmp         	strcasecmp
#define _tcsnicmp         	strncasecmp
#define _tcsncicmp(s1,s2,s) strncasecmp(s1, s2, (s)*sizeof(TCHAR))
#define _stprintf           sprintf
#define _sntprintf          snprintf
#define _vsntprintf         vsnprintf
#define _vsctprintf         _vscprintf

int _vscprintf(LPCSTR format, va_list pargs);

#define _T(s)               s
#endif //_MSC_VER

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
typedef int64_t     		size_f_t;

#define DECLARE_NO_INDEX(...) std::numeric_limits<__VA_ARGS__>::max()
#define NO_ID				DECLARE_NO_INDEX(uint32_t)

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

#ifndef MAX_PATH
#define MAX_PATH			260
#endif

#ifndef NULL
#define NULL				0
#endif

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

#ifndef MINF
#define MINF			std::min
#endif
#ifndef MAXF
#define MAXF			std::max
#endif

namespace SEACAVE {

template<typename T>
inline T MINF3(const T& x1, const T& x2, const T& x3) {
	return MINF(MINF(x1, x2), x3);
}
template<typename T>
inline T MAXF3(const T& x1, const T& x2, const T& x3) {
	return MAXF(MAXF(x1, x2), x3);
}

#ifndef RAND
#define RAND			std::rand
#endif
template<typename T>
FORCEINLINE T RANDOM() { return (T(1)/RAND_MAX)*RAND(); }

template<typename T1, typename T2>
union TAliasCast
{
	T1 f;
	T2 i;
	inline TAliasCast() {}
	inline TAliasCast(T1 v) : f(v) {}
	inline TAliasCast(T2 v) : i(v) {}
	inline TAliasCast& operator = (T1 v) { f = v; return *this; }
	inline TAliasCast& operator = (T2 v) { i = v; return *this; }
	inline operator T1 () const { return f; }
};
typedef TAliasCast<float,int32_t> CastF2I;
typedef TAliasCast<double,int32_t> CastD2I;

} // namespace SEACAVE

#if defined(_MSC_VER)
# define __LITTLE_ENDIAN 0
# define __BIG_ENDIAN 1
# define __PDP_ENDIAN 2
# define __BYTE_ORDER __LITTLE_ENDIAN
#elif defined(__APPLE__)
# include <machine/endian.h>
#elif defined(__GNUC__)
# include <endian.h>
#endif


// I N C L U D E S /////////////////////////////////////////////////

#include "Strings.h"
#include "AutoPtr.h"
#include "Thread.h"
#include "SharedPtr.h"
#include "List.h"
#include "Queue.h"
#include "Hash.h"
#include "Timer.h"
#include "CriticalSection.h"
#include "Semaphore.h"
#include "Util.h"
#include "File.h"
#include "MemFile.h"
#include "LinkLib.h"

namespace SEACAVE {

typedef class GENERAL_API CSharedPtr<File>				FilePtr;

typedef class GENERAL_API CSharedPtr<ISTREAM>			ISTREAMPTR;
typedef ISTREAM*										LPISTREAM;

typedef class GENERAL_API CSharedPtr<OSTREAM>			OSTREAMPTR;
typedef OSTREAM*										LPOSTREAM;

typedef class GENERAL_API CSharedPtr<IOSTREAM>			IOSTREAMPTR;
typedef IOSTREAM*										LPIOSTREAM;

typedef class GENERAL_API cList<void*, void*, 0>        VoidArr;
typedef class GENERAL_API cList<LPCTSTR, LPCTSTR, 0>	LPCTSTRArr;
typedef class GENERAL_API cList<String>                 StringArr;
typedef class GENERAL_API cList<IDX, IDX, 0>			IDXArr;
typedef class GENERAL_API cList<uint8_t, uint8_t, 0>    Unsigned8Arr;
typedef class GENERAL_API cList<unsigned, unsigned, 0>  UnsignedArr;
typedef class GENERAL_API cList<uint32_t, uint32_t, 0>  Unsigned32Arr;
typedef class GENERAL_API cList<uint64_t, uint64_t, 0>  Unsigned64Arr;
typedef class GENERAL_API cList<size_t, size_t, 0>      SizeArr;
typedef class GENERAL_API cList<int, int, 0>            IntArr;
typedef class GENERAL_API cList<bool, bool, 0>          BoolArr;
typedef class GENERAL_API cList<float, float, 0>        FloatArr;
typedef class GENERAL_API cList<double, double, 0>      DoubleArr;

} // namespace SEACAVE

#include "Log.h"
#include "EventQueue.h"
#include "SML.h"
#include "ConfigTable.h"
#include "HTMLDoc.h"


// D E F I N E S ///////////////////////////////////////////////////

//
// Constant defines

// everything went smooth
#define _OK					((HRESULT)0L)

// just reports no errors
#define _CANCEL				0x82000000

// general error message
#define _FAIL				0x82000001

// specific error messages
#define _CREATEAPI			0x82000002
#define _CREATEDEVICE		0x82000003
#define _CREATEBUFFER		0x82000004
#define _INVALIDPARAM		0x82000005
#define _INVALIDID			0x82000006
#define _BUFFERSIZE			0x82000007
#define _BUFFERLOCK			0x82000008
#define _NOTCOMPATIBLE		0x82000009
#define _OUTOFMEMORY		0x8200000a
#define _FILENOTFOUND		0x8200000b
#define _INVALIDFILE		0x8200000c
#define _NOSHADERSUPPORT	0x8200000d
#define _NOSERVERFOUND		0x8200000e
#define _WOULDBLOCK			0x8200000f

#ifndef SUCCEEDED
#define SUCCEEDED(hr)       (((HRESULT)(hr)) >= 0)
#endif
#ifndef FAILED
#define FAILED(hr)          (((HRESULT)(hr)) < 0)
#endif


// D E F I N E S ///////////////////////////////////////////////////

#define RGBA(r, g, b, a)	((DWORD)(((a) << 24) | ((r) << 16) | ((g) << 8) | (b)))
#define RGBC(clr)			(RGBA((BYTE)((clr).fR*255), (BYTE)((clr).fG*255), (BYTE)((clr).fB*255), (BYTE)((clr).fA*255)))
#define RGB24TO8(r,g,b)		((BYTE)((((WORD)r)*30+((WORD)g)*59+((WORD)b)*11)/100))
#define RGB24TO16(r,g,b)	((((WORD)(((BYTE)(r))>>3))<<11) | (((WORD)(((BYTE)(g))>>2))<<5) | ((WORD)(((BYTE)(b))>>3)))
#define RGB16TOR(rgb)		(((BYTE)(((WORD)(rgb))>>11))<<3)
#define RGB16TOG(rgb)		(((BYTE)((((WORD)(rgb))&0x07E0)>>5))<<2)
#define RGB16TOB(rgb)		(((BYTE)(((WORD)(rgb))&0x001F))<<3)

#define TIMER_START()		SEACAVE::Timer::SysType timerStart = SEACAVE::Timer::GetSysTime()
#define TIMER_UPDATE()		timerStart = SEACAVE::Timer::GetSysTime()
#define TIMER_GET()			SEACAVE::Timer::SysTime2TimeMs(SEACAVE::Timer::GetSysTime() - timerStart)
#define TIMER_GET_INT()		((SEACAVE::Timer::SysType)TIMER_GET())
#define TIMER_GET_FORMAT()	SEACAVE::Util::formatTime(TIMER_GET_INT())


// D E F I N E S ///////////////////////////////////////////////////

#ifndef CHECK
#define CHECK(exp)			{ if (!(exp)) { VERBOSE("Check failed: " #exp); abort(); } }
#endif
#ifndef ABORT
#define ABORT(msg)			{ VERBOSE("error: " #msg); exit(-1); }
#endif

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

// F U N C T I O N S ///////////////////////////////////////////////

template<typename T>
inline T& NEGATE(T& a) {
	return (a = -a);
}
template<typename T>
inline T SQUARE(const T& a) {
	return (a * a);
}
template<typename T>
inline T CUBE(const T& a) {
	return (a * a * a);
}
template<typename T>
inline T SQRT(const T& a) {
	return T(sqrt(a));
}
template<typename T>
inline T EXP(const T& a) {
	return T(exp(a));
}
template<typename T>
inline T LOGN(const T& a) {
	return T(log(a));
}
template<typename T>
inline T LOG10(const T& a) {
	return T(log10(a));
}
template<typename T>
inline T powi(T base, int exp) {
	T result(1);
	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}
	return result;
}
inline int log2i(int val) {
	int ret = -1;
	while (val > 0) {
		val >>= 1;
		ret++;
	}
	return ret;
}
template <int N> constexpr inline int log2i() { return 1+log2i<(N>>1)>(); }
template <>   constexpr inline int log2i<0>() { return -1; }
template <>   constexpr inline int log2i<1>() { return 0; }
template <>   constexpr inline int log2i<2>() { return 1; }

template<typename T>
inline T arithmeticSeries(T n, T a1=1, T d=1) {
	return (n*(a1*2+(n-1)*d))/2;
}
template<typename T>
inline T factorial(T n) {
	T ret = 1;
	while (n > 1)
		ret *= n--;
	return ret;
}
template<typename T>
inline T combinations(const T& n, const T& k) {
	ASSERT(n >= k);
	#if 1
	T num = n;
	const T den = factorial(k);
	for (T i=n-k+1; i<n; ++i)
		num *= i;
	ASSERT(num%den == 0);
	return num/den;
	#else
	return factorial(n) / (factorial(k)*factorial(n-k));
	#endif
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
FORCEINLINE int Floor2Int(float x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x)-_float2int_doublemagicroundeps);
	#else
	return int(floor(x));
	#endif
}
FORCEINLINE int Floor2Int(double x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(x-_float2int_doublemagicroundeps);
	#else
	return int(floor(x));
	#endif
}
FORCEINLINE int Ceil2Int(float x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x)+_float2int_doublemagicroundeps);
	#else
	return int(ceil(x));
	#endif
}
FORCEINLINE int Ceil2Int(double x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(x+_float2int_doublemagicroundeps);
	#else
	return int(ceil(x));
	#endif
}
FORCEINLINE int Round2Int(float x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(double(x)+_float2int_doublemagicdelta);
	#else
	return int(floor(x+.5f));
	#endif
}
FORCEINLINE int Round2Int(double x) {
	#ifdef _FAST_FLOAT2INT
	return CRound2Int(x+_float2int_doublemagicdelta);
	#else
	return int(floor(x+.5));
	#endif
}
/*----------------------------------------------------------------*/


// Random number generation
// uniform random number generation
FORCEINLINE float random() {
	return RANDOM<float>();
}
FORCEINLINE double randomd() {
	return RANDOM<double>();
}
template<typename T>
FORCEINLINE T randomRange(T nMin, T nMax) {
	return nMin + ((nMax - nMin) * RAND())/RAND_MAX;
}
template<>
FORCEINLINE float randomRange<float>(float fMin, float fMax) {
	return fMin + (fMax - fMin) * random();
}
template<>
FORCEINLINE double randomRange<double>(double fMin, double fMax) {
	return fMin + (fMax - fMin) * randomd();
}
template<typename T>
FORCEINLINE T randomMeanRange(T mean, T delta/*=(max-min)/2*/) {
	return mean-delta + (delta*T(2) * RAND())/RAND_MAX;
}
template<>
FORCEINLINE float randomMeanRange<float>(float mean, float delta/*=(max-min)/2*/) {
	return mean + delta * (2.f * random() - 1.f);
}
template<>
FORCEINLINE double randomMeanRange<double>(double mean, double delta/*=(max-min)/2*/) {
	return mean + delta * (2.0 * randomd() - 1.0);
}
// gaussian random number generation
template<typename T>
FORCEINLINE T gaussian(T val, T sigma) {
	return EXP(-SQUARE(val/sigma)/2)/(SQRT(T(M_PI*2))*sigma);
}
template<typename T>
FORCEINLINE T randomGaussian(T mean, T sigma) {
	T x, y, r2;
	do {
		x = T(-1) + T(2) * RANDOM<T>();
		y = T(-1) + T(2) * RANDOM<T>();
		r2 = x * x + y * y;
	} while (r2 > T(1) || r2 == T(0));
	return mean + sigma * y * SQRT(T(-2) * LOGN(r2) / r2);
}
template<typename T>
FORCEINLINE T randomGaussian(T sigma) {
	return randomGaussian(T(0), sigma);
}
FORCEINLINE float randomGaussian() {
	return randomGaussian(0.f, 1.f);
}
FORCEINLINE double randomGaussiand() {
	return randomGaussian(0.0, 1.0);
}
/*----------------------------------------------------------------*/


// INTERPOLATION

// Linear interpolation
inline float lerp(float u, float v, float x)
{
	return u + (v - u) * x;
}
template<typename Type>
inline Type lerp(const Type& u, const Type& v, float x)
{
	return u + (v - u) * x;
}

// Cubic interpolation
inline float cerp(float u0, float u1, float u2, float u3, float x)
{
	const float p((u3 - u2) - (u0 - u1));
	const float q((u0 - u1) - p);
	const float r(u2 - u0);
	return x * (x * (x * p + q) + r) + u1;
}
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

inline bool   ISINFORNAN(float x)			{ return (std::isinf(x) || std::isnan(x)); }
inline bool   ISINFORNAN(double x)			{ return (std::isinf(x) || std::isnan(x)); }
inline bool   ISFINITE(float x)				{ return (!std::isinf(x) && !std::isnan(x)); }
inline bool   ISFINITE(double x)			{ return (!std::isinf(x) && !std::isnan(x)); }
template<typename _Tp>
inline bool   ISFINITE(const _Tp* x, size_t n)	{ for (size_t i=0; i<n; ++i) if (ISINFORNAN(x[i])) return false; return true; }

template<typename _Tp>
inline bool   ISINSIDE(_Tp v,_Tp l0,_Tp l1)	{ return (l0 <= v && v < l1); }

template<typename _Tp>
inline _Tp    CLAMP(_Tp v, _Tp c0, _Tp c1)	{ ASSERT(c0<=c1); return MINF(MAXF(v, c0), c1); }
template<typename _Tp>
inline _Tp    CLAMPS(_Tp v, _Tp c0, _Tp c1)	{ if (c0 <= c1) return CLAMP(v, c0, c1); return CLAMP(v, c1, c0); }

template<typename _Tp>
inline _Tp    SIGN(_Tp x)					{ if (x > _Tp(0)) return _Tp(1); if (x < _Tp(0)) return _Tp(-1); return _Tp(0); }

template<typename _Tp>
inline _Tp    ABS(_Tp    x)					{ return std::abs(x); }

template<typename _Tp>
inline _Tp    ZEROTOLERANCE()				{ return _Tp(0); }
template<>
inline float  ZEROTOLERANCE()				{ return FZERO_TOLERANCE; }
template<>
inline double ZEROTOLERANCE()				{ return ZERO_TOLERANCE; }

template<typename _Tp>
inline _Tp    EPSILONTOLERANCE()			{ return std::numeric_limits<_Tp>::epsilon(); }
template<>
inline float  EPSILONTOLERANCE()			{ return 0.00001f; }
template<>
inline double EPSILONTOLERANCE()			{ return 1e-10; }

inline bool   ISZERO(float  x)				{ return ABS(x) < FZERO_TOLERANCE; }
inline bool   ISZERO(double x)				{ return ABS(x) < ZERO_TOLERANCE; }

inline bool   ISEQUAL(float  x, float  v)	{ return ABS(x-v) < FZERO_TOLERANCE; }
inline bool   ISEQUAL(double x, double v)	{ return ABS(x-v) < ZERO_TOLERANCE; }

inline float  INVZERO(float)				{ return FINV_ZERO; }
inline double INVZERO(double)				{ return INV_ZERO; }
template<typename _Tp>
inline _Tp    INVZERO(_Tp)					{ return std::numeric_limits<_Tp>::max(); }

template<typename _Tp>
inline _Tp    INVERT(_Tp    x)				{ return (x==_Tp(0) ? INVZERO(x) : _Tp(1)/x); }

template<typename _Tp>
inline _Tp    SAFEDIVIDE(_Tp   x, _Tp   y)	{ return (y==_Tp(0) ? INVZERO(y) : x/y); }
/*----------------------------------------------------------------*/

} // namespace SEACAVE


#include "HalfFloat.h"


namespace SEACAVE {

// P R O T O T Y P E S /////////////////////////////////////////////

typedef double REAL;

template <typename TYPE, int m, int n> class TMatrix;
template <typename TYPE, int DIMS> class TAABB;
template <typename TYPE, int DIMS> class TRay;
template <typename TYPE, int DIMS> class TPlane;

// 2D point struct
template <typename TYPE>
class TPoint2 : public cv::Point_<TYPE>
{
public:
	typedef TYPE Type;
	typedef cv::Point_<TYPE> Base;
	typedef cv::Size Size;
	typedef cv::Vec<TYPE,2> cvVec;
	typedef cv::Matx<TYPE,2,1> Vec;
	typedef cv::Matx<TYPE,1,2> VecT;
	#ifdef _USE_EIGEN
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(TYPE,2)
	typedef Eigen::Matrix<TYPE,2,1> EVec;
	typedef Eigen::Map<EVec> EVecMap;
	#endif

	using Base::x;
	using Base::y;

	static const TPoint2 ZERO;
	static const TPoint2 INF;

public:
	inline TPoint2() {}
	inline TPoint2(const Size& rhs) : Base(rhs) {}
	template <typename T> inline TPoint2(const cv::Point_<T>& rhs) : Base((Type)rhs.x,(Type)rhs.y) {}
	template <typename T> inline TPoint2(const cv::Matx<T,2,1>& rhs) : Base(rhs(0),rhs(1)) {}
	template <typename T> inline TPoint2(const cv::Matx<T,1,2>& rhs) : Base(rhs(0),rhs(1)) {}
	#ifdef _USE_EIGEN
	inline TPoint2(const EVec& rhs) { operator EVec& () = rhs; }
	#endif
	explicit inline TPoint2(const TYPE& _x) : Base(_x,_x) {}
	inline TPoint2(const TYPE& _x, const TYPE& _y) : Base(_x,_y) {}
	explicit inline TPoint2(const cv::Point3_<TYPE>& pt) : Base(pt.x/pt.z,pt.y/pt.z) {}

	template <typename T> inline TPoint2& operator = (const cv::Point_<T>& rhs) { Base::operator = (rhs); return *this; }
	template <typename T> inline TPoint2& operator = (const cv::Matx<T,2,1>& rhs) { operator Vec& () = rhs; return *this; }
	template <typename T> inline TPoint2& operator = (const cv::Matx<T,1,2>& rhs) { operator VecT& () = rhs; return *this; }
	#ifdef _USE_EIGEN
	inline TPoint2& operator = (const EVec& rhs) { operator EVec& () = rhs; return *this; }
	#endif

	// conversion to another data type
	template <typename T> inline operator TPoint2<T> () const { return TPoint2<T>(x,y); }

	// pointer to the first element access
	inline const TYPE* ptr() const { return &x; }
	inline TYPE* ptr() { return &x; }

	// 1D element access
	inline const TYPE& operator [](size_t i) const { ASSERT(i>=0 && i<2); return ptr()[i]; }
	inline TYPE& operator [](size_t i) { ASSERT(i>=0 && i<2); return ptr()[i]; }

	// Access point as Size equivalent
	inline operator const Size& () const { return *((const Size*)this); }
	inline operator Size& () { return *((Size*)this); }

	// Access point as vector equivalent
	inline operator const Vec& () const { return *((const Vec*)this); }
	inline operator Vec& () { return *((Vec*)this); }

	// Access point as transposed vector equivalent
	inline operator const VecT& () const { return *((const VecT*)this); }
	inline operator VecT& () { return *((VecT*)this); }

	#ifdef _USE_EIGEN
	// Access point as Eigen equivalent
	inline operator const EVec& () const { return *((const EVec*)this); }
	inline operator EVec& () { return *((EVec*)this); }
	// Access point as Eigen::Map equivalent
	inline operator const EVecMap () const { return EVecMap((TYPE*)this); }
	inline operator EVecMap () { return EVecMap((TYPE*)this); }
	#endif

	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Base>(*this);
	}
	#endif
};
template <typename TYPE> const TPoint2<TYPE> TPoint2<TYPE>::ZERO(0,0);
template <typename TYPE> const TPoint2<TYPE> TPoint2<TYPE>::INF(std::numeric_limits<TYPE>::infinity(),std::numeric_limits<TYPE>::infinity());
/*----------------------------------------------------------------*/
typedef TPoint2<int> Point2i;
typedef TPoint2<hfloat> Point2hf;
typedef TPoint2<float> Point2f;
typedef TPoint2<double> Point2d;
/*----------------------------------------------------------------*/


// 3D point struct
template <typename TYPE>
class TPoint3 : public cv::Point3_<TYPE>
{
public:
	typedef TYPE Type;
	typedef cv::Point3_<TYPE> Base;
	typedef cv::Vec<TYPE,3> cvVec;
	typedef cv::Matx<TYPE,3,1> Vec;
	typedef cv::Matx<TYPE,1,3> VecT;
	#ifdef _USE_EIGEN
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(TYPE,3)
	typedef Eigen::Matrix<TYPE,3,1> EVec;
	typedef Eigen::Map<EVec> EVecMap;
	#endif

	using Base::x;
	using Base::y;
	using Base::z;

	static const TPoint3 ZERO;
	static const TPoint3 INF;

public:
	inline TPoint3() {}
	template <typename T> inline TPoint3(const cv::Point3_<T>& rhs) : Base((Type)rhs.x,(Type)rhs.y,(Type)rhs.z) {}
	template <typename T> inline TPoint3(const cv::Matx<T,3,1>& rhs) : Base(rhs(0),rhs(1),rhs(2)) {}
	template <typename T> inline TPoint3(const cv::Matx<T,1,3>& rhs) : Base(rhs(0),rhs(1),rhs(2)) {}
	#ifdef _USE_EIGEN
	inline TPoint3(const EVec& rhs) { operator EVec& () = rhs; }
	#endif
	explicit inline TPoint3(const TYPE& _x) : Base(_x,_x,_x) {}
	inline TPoint3(const TYPE& _x, const TYPE& _y, const TYPE& _z) : Base(_x,_y,_z) {}
	template <typename T> inline TPoint3(const cv::Point_<T>& pt, const T& _z=T(1)) : Base(pt.x,pt.y,_z) {}
	template <typename T1, typename T2> inline TPoint3(const cv::Point_<T1>& pt, const T2& _z) : Base(pt.x,pt.y,_z) {}

	template <typename T> inline TPoint3& operator = (const cv::Point3_<T>& rhs)  { Base::operator = (rhs); return *this; }
	template <typename T> inline TPoint3& operator = (const cv::Matx<T,3,1>& rhs)  { operator Vec& () = rhs; return *this; }
	template <typename T> inline TPoint3& operator = (const cv::Matx<T,1,3>& rhs)  { operator VecT& () = rhs; return *this; }
	#ifdef _USE_EIGEN
	inline TPoint3& operator = (const EVec& rhs) { operator EVec& () = rhs; return *this; }
	#endif

	// conversion to another data type
	template <typename T> inline operator TPoint3<T> () const { return TPoint3<T>(x,y,z); }

	// pointer to the first element access
	inline const TYPE* ptr() const { return &x; }
	inline TYPE* ptr() { return &x; }

	// 1D element access
	inline const TYPE& operator [](size_t i) const { ASSERT(i>=0 && i<3); return ptr()[i]; }
	inline TYPE& operator [](size_t i) { ASSERT(i>=0 && i<3); return ptr()[i]; }

	// Access point as vector equivalent
	inline operator const Vec& () const { return *((const Vec*)this); }
	inline operator Vec& () { return *((Vec*)this); }

	// Access point as transposed vector equivalent
	inline operator const VecT& () const { return *((const VecT*)this); }
	inline operator VecT& () { return *((VecT*)this); }

	#ifdef _USE_EIGEN
	// Access point as Eigen equivalent
	inline operator const EVec& () const { return *((const EVec*)this); }
	inline operator EVec& () { return *((EVec*)this); }
	// Access point as Eigen::Map equivalent
	inline operator const EVecMap () const { return EVecMap((TYPE*)this); }
	inline operator EVecMap () { return EVecMap((TYPE*)this); }
	#endif

	// rotate point using the given parametrized rotation (axis-angle)
	inline void RotateAngleAxis(const TPoint3& rot) { return (*this) = RotateAngleAxis((*this), rot); }
	static TPoint3 RotateAngleAxis(const TPoint3& X, const TPoint3& rot);

	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Base>(*this);
	}
	#endif
};
template <typename TYPE> const TPoint3<TYPE> TPoint3<TYPE>::ZERO(0,0,0);
template <typename TYPE> const TPoint3<TYPE> TPoint3<TYPE>::INF(std::numeric_limits<TYPE>::infinity(),std::numeric_limits<TYPE>::infinity(),std::numeric_limits<TYPE>::infinity());
/*----------------------------------------------------------------*/
typedef TPoint3<int> Point3i;
typedef TPoint3<hfloat> Point3hf;
typedef TPoint3<float> Point3f;
typedef TPoint3<double> Point3d;
/*----------------------------------------------------------------*/


// matrix struct
template <typename TYPE, int m, int n>
class TMatrix : public cv::Matx<TYPE,m,n>
{
public:
	typedef TYPE Type;
	typedef cv::Matx<TYPE,m,n> Base;
	typedef cv::Vec<TYPE,m> Vec;
	#ifdef _USE_EIGEN
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(TYPE,m*n)
	typedef Eigen::Matrix<TYPE,m,n,(n>1?Eigen::RowMajor:Eigen::Default)> EMat;
	typedef Eigen::Map<const EMat> CEMatMap;
	typedef Eigen::Map<EMat> EMatMap;
	#endif

	using Base::val;

	enum { elems = m*n };

	static const TMatrix ZERO;
	static const TMatrix IDENTITY;
	static const TMatrix INF;

public:
	inline TMatrix() {}
	template <typename T> inline TMatrix(const cv::Matx<T,m,n>& rhs) : Base(rhs) {}
	template <typename T> inline TMatrix(const cv::Point_<T>& rhs) : Base(rhs.x, rhs.y) {}
	template <typename T> inline TMatrix(const cv::Point3_<T>& rhs) : Base(rhs.x, rhs.y, rhs.z) {}
	inline TMatrix(const cv::Mat& rhs) : Base(rhs) {}
	#ifdef _USE_EIGEN
	inline TMatrix(const EMat& rhs) { operator EMat& () = rhs; }
	#endif

	template <typename T> inline TMatrix& operator = (const cv::Matx<T,m,n>& rhs) { Base::operator = (rhs); return *this; }
	inline TMatrix& operator = (const cv::Mat& rhs) { Base::operator = (rhs); return *this; }
	#ifdef _USE_EIGEN
	inline TMatrix& operator = (const EMat& rhs) { operator EMat& () = rhs; return *this; }
	#endif

	inline bool IsEqual(const Base&) const;
	inline bool IsEqual(const Base&, TYPE eps) const;

	// 1D element access
	inline const TYPE& operator [](size_t i) const { ASSERT(i<elems); return val[i]; }
	inline TYPE& operator [](size_t i) { ASSERT(i<elems); return val[i]; }

	// Access point as vector equivalent
	inline operator const Vec& () const { return *((const Vec*)this); }
	inline operator Vec& () { return *((Vec*)this); }

	#ifdef _USE_EIGEN
	// Access point as Eigen equivalent
	inline operator const EMat& () const { return *((const EMat*)this); }
	inline operator EMat& () { return *((EMat*)this); }
	// Access point as Eigen::Map equivalent
	inline operator CEMatMap() const { return CEMatMap((const TYPE*)val); }
	inline operator EMatMap () { return EMatMap((TYPE*)val); }
	#endif

	// calculate right null-space of this matrix ([n,n-m])
	inline TMatrix<TYPE,n,n-m> RightNullSpace(int flags = 0) const;
	// calculate right/left null-vector of this matrix ([n/m,1])
	inline TMatrix<TYPE,n,1> RightNullVector(int flags = 0) const;
	inline TMatrix<TYPE,m,1> LeftNullVector(int flags = 0) const;

	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Base>(*this);
	}
	#endif
};
template <typename TYPE, int m, int n> const TMatrix<TYPE,m,n> TMatrix<TYPE,m,n>::ZERO(TMatrix::zeros());
template <typename TYPE, int m, int n> const TMatrix<TYPE,m,n> TMatrix<TYPE,m,n>::IDENTITY(TMatrix::eye());
template <typename TYPE, int m, int n> const TMatrix<TYPE,m,n> TMatrix<TYPE,m,n>::INF(TMatrix::all(std::numeric_limits<TYPE>::infinity()));
/*----------------------------------------------------------------*/


// generic matrix struct
template <typename TYPE>
class TDMatrix : public cv::Mat_<TYPE>
{
public:
	typedef TYPE Type;
	typedef cv::Mat_<TYPE> Base;
	typedef cv::Size Size;
	#ifdef _USE_EIGEN
	typedef Eigen::Matrix<TYPE,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> EMat;
	typedef Eigen::Map<EMat> EMatMap;
	#endif

	using Base::rows;
	using Base::cols;
	using Base::data;
	using Base::step;
	using Base::dims;

public:
	inline TDMatrix() {}
	inline TDMatrix(const Base& rhs) : Base(rhs) {}
	inline TDMatrix(const cv::Mat& rhs) : Base(rhs) {}
	inline TDMatrix(const cv::MatExpr& rhs) : Base(rhs) {}
	inline TDMatrix(int _rows, int _cols) : Base(_rows, _cols) {}
	inline TDMatrix(int _rows, int _cols, TYPE* _data, size_t _step=Base::AUTO_STEP) : Base(_rows, _cols, _data, _step) {}
	inline TDMatrix(const Size& sz) : Base(sz) {}
	inline TDMatrix(const Size& sz, const TYPE& v) : Base(sz, v) {}
	inline TDMatrix(const Size& sz, TYPE* _data, size_t _step=Base::AUTO_STEP) : Base(sz.height, sz.width, _data, _step) {}
	#ifdef _USE_EIGEN
	inline TDMatrix(const EMat& rhs) { operator EMatMap () = rhs; }
	#endif

	inline TDMatrix& operator = (const Base& rhs) { Base::operator=(rhs); return *this; }
	inline TDMatrix& operator = (const cv::MatExpr& rhs) { Base::operator=(rhs); return *this; }
	#ifdef _USE_EIGEN
	inline TDMatrix& operator = (const EMat& rhs) { operator EMatMap () = rhs; return *this; }
	#endif

	/// Construct the 2D matrix with the desired size and init its elements
	inline void construct(int _rows, int _cols) {
		Base::create(_rows, _cols);
		for (int i=0; i<rows; ++i)
			for (int j=0; j<cols; ++j)
				new(cv::Mat::ptr<TYPE>(i,j)) TYPE;
	}
	inline void construct(const Size& sz) { construct(sz.height, sz.width); }
	inline void destroy() {
		for (int i=0; i<rows; ++i)
			for (int j=0; j<cols; ++j)
				cv::Mat::ptr<TYPE>(i,j)->~TYPE();
	}

	/// Set all elements to the given value
	inline void memset(uint8_t v) { ASSERT(dims == 2 && cv::Mat::isContinuous()); ::memset(data, v, row_stride()*rows); }
	inline void fill(const TYPE& v) { ASSERT(dims == 2); for (int i=0; i<rows; ++i) for (int j=0; j<cols; ++j) cv::Mat::at<TYPE>(i,j) = v; }

	/// What is the row stride of the matrix?
	inline size_t row_stride() const { ASSERT(dims == 2); return step[0]; }
	/// What is the elem stride of the matrix?
	inline size_t elem_stride() const { ASSERT(dims == 2 && step[1] == sizeof(TYPE)); return step[1]; }
	/// Compute the area of the 2D matrix
	inline int area() const { ASSERT(dims == 2); return cols*rows; }

	/// Is this coordinate inside the 2D matrix?
	inline bool isInside(const Size& pt) const {
		return pt.width>=0 && pt.height>=0 && pt.width<Base::size().width && pt.height<Base::size().height;
	}
	template <typename T>
	inline bool isInside(const cv::Point_<T>& pt) const {
		return int(pt.x)>=0 && int(pt.y)>=0 && int(pt.x)<Base::size().width && int(pt.y)<Base::size().height;
	}

	/// Is this coordinate inside the 2D matrix, and not too close to the edges?
	/// @param border: the size of the border
	inline bool isInsideWithBorder(const Size& pt, int border) const {
		return pt.width>=border && pt.height>=border && pt.width<Base::size().width-border && pt.height<Base::size().height-border;
	}
	template <typename T>
	inline bool isInsideWithBorder(const cv::Point_<T>& pt, int border) const {
		return int(pt.x)>=border && int(pt.y)>=border && int(pt.x)<Base::size().width-border && int(pt.y)<Base::size().height-border;
	}
	template <typename T, int border>
	inline bool isInsideWithBorder(const cv::Point_<T>& pt) const {
		return int(pt.x)>=border && int(pt.y)>=border && int(pt.x)<Base::size().width-border && int(pt.y)<Base::size().height-border;
	}

	/// Remove the given element from the vector
	inline void remove(int idx) {
		// replace the removed element by the last one and decrease the size
		ASSERT(rows == 1 || cols == 1);
		const int last = area()-1;
		Base::operator()(idx) = Base::operator()(last);
		Base::resize((size_t)last);
	}

	/** @author koeser
		@warning very slow, generic implementation (order "n!"), better to use
		matrix decomposition (see BIAS/MathAlgo/Lapack.hh) */
	inline TYPE getDetSquare() const;

	/** @brief computes the adjoint matrix
		@author Christian Beder */
	inline TDMatrix getAdjoint() const;

	/** @brief compute square system matrix dest = A^T * A
		@param dest holds result of Transpose * this

		If you want to solve A * x = b, where A has more rows than columns,
		a common technique is to solve x = (A^T * A)^-1 * A^T * b.
		This function provides a fast way to compute A^T*A from A.
		@author grest/koeser */
	inline void getSystemMatrix(TDMatrix& dest) const;

	/** @brief componentwise: this = 0.5(this + this^T) yields symmetric matrix
		only allowed for square shaped matrices
		@author koeser 01/2007 */
	inline void makeSymmetric();

	/** Return the L1 norm: |a| + |b| + |c| + ...
		@author Ingo Thomsen
		@date 04/11/2002
		@status untested    **/
	inline TYPE getNormL1() const;

	/** Return the L2 norm: a^2 + b^2 + c^2 + ...
		@author woelk 07/2004 */
	inline double getNormL2() const;

	/** Kronecker-product with matrix, result in dest */
	void Kronecker(const TDMatrix& B, TDMatrix& dest) const;

	/** @brief swaps two rows
		@author woelk 05/2008 www.vision-n.de */
	void SwapRows(int i, int r);

	/** @brief use the Gauss Jordan Algorithm to transform the matrix to
		reduced row echelon form.
		@author woelk 05/2008 www.vision-n.de */
	void GaussJordan();

	//! more convenient forms of row and element access operators
	const TYPE& operator [](size_t i) const { return cv::Mat::at<TYPE>((int)i); }
	TYPE& operator [](size_t i) { return cv::Mat::at<TYPE>((int)i); }

	/// Access an element from the matrix. Bounds checking is only performed in debug mode.
	inline const TYPE& operator [] (const Size& pos) const {
		ASSERT(isInside(pos) && elem_stride() == sizeof(TYPE));
		return ((const TYPE*)data)[pos.height*row_stride() + pos.width];
	}
	inline TYPE& operator [] (const Size& pos) {
		ASSERT(isInside(pos) && elem_stride() == sizeof(TYPE));
		return ((TYPE*)data)[pos.height*row_stride() + pos.width];
	}

	/// pointer to the beginning of the matrix data
	inline const TYPE* getData() const { ASSERT(cv::Mat::isContinuous()); return (const TYPE*)data; }
	inline TYPE* getData() { ASSERT(cv::Mat::isContinuous()); return (TYPE*)data; }

	#ifdef _USE_EIGEN
	// Access point as Eigen::Map equivalent
	inline operator const EMatMap () const { return EMatMap(getData(), rows, cols); }
	inline operator EMatMap () { return EMatMap(getData(), rows, cols); }
	#endif

	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Base>(*this);
	}
	#endif
};
/*----------------------------------------------------------------*/
typedef TDMatrix<REAL> DMatrix;
typedef TDMatrix<int8_t> DMatrix8S;
typedef TDMatrix<uint8_t> DMatrix8U;
typedef TDMatrix<int32_t> DMatrix32S;
typedef TDMatrix<uint32_t> DMatrix32U;
typedef TDMatrix<float> DMatrix32F;
typedef TDMatrix<double> DMatrix64F;
typedef SEACAVE::cList<DMatrix, const DMatrix&, 2> DMatrixArr;
typedef SEACAVE::cList<cv::Mat, const cv::Mat&, 2> MatArr;
/*----------------------------------------------------------------*/


// generic vector struct
template <typename TYPE>
class TDVector : public TDMatrix<TYPE>
{
public:
	typedef TYPE Type;
	typedef TDMatrix<TYPE> Base;
	typedef typename Base::Base BaseBase;
	typedef cv::Size Size;

	using Base::rows;
	using Base::cols;
	using Base::data;

public:
	inline TDVector() {}
	inline TDVector(const Base& rhs) : Base(rhs) {}
	inline TDVector(const cv::Mat& rhs) : Base(rhs) {}
	inline TDVector(const cv::MatExpr& rhs) : Base(rhs) {}
	inline TDVector(int _rows) : Base(_rows, 1) {}
	inline TDVector(int _rows, const TYPE& v) : Base(Size(_rows,1), v) {}
	inline TDVector(int _rows, TYPE* _data, size_t _step=Base::AUTO_STEP) : Base(_rows, 1, _data, _step) {}

	inline TDVector& operator = (const Base& rhs) { BaseBase::operator=(rhs); return *this; }
	inline TDVector& operator = (const BaseBase& rhs) { BaseBase::operator=(rhs); return *this; }
	inline TDVector& operator = (const cv::MatExpr& rhs) { BaseBase::operator=(rhs); return *this; }

	//! equivalent to Mat::create(_rows, 1, DataType<_Tp>::type)
	inline void create(int _rows) { BaseBase::create(_rows, 1); }

	/** outer product, constructs a matrix.
		Often written as v * v^T for col vectors
		@author Daniel Grest, Oct 2002
		@status tested */
	TDMatrix<TYPE> getOuterProduct(const TDVector<TYPE>& v) const;

	/** kronecker product
		@author woelk 08/2004 */
	void getKroneckerProduct(const TDVector<TYPE>& arg, TDVector<TYPE>& dst) const;
};
/*----------------------------------------------------------------*/
typedef TDVector<REAL> DVector;
typedef TDVector<int8_t> DVector8S;
typedef TDVector<uint8_t> DVector8U;
typedef TDVector<int32_t> DVector32S;
typedef TDVector<uint32_t> DVector32U;
typedef TDVector<float> DVector32F;
typedef TDVector<double> DVector64F;
typedef SEACAVE::cList<DVector, const DVector&, 2> DVectorArr;
/*----------------------------------------------------------------*/


// generic color type
// Here the color order means the order the data is stored on this machine.
// For example BGR means that the blue byte is stored first and the red byte last,
// which correspond to the RGB format on a little-endian machine.
#define _COLORMODE_BGR 1 // little-endian
#define _COLORMODE_RGB 2 // big-endian
#ifndef _COLORMODE
#define _COLORMODE _COLORMODE_BGR
#endif

template<typename TYPE> class ColorType
{
public:
	typedef TYPE value_type;
	typedef value_type alt_type;
	static const value_type ONE;
	static const alt_type ALTONE;
};
template<> class ColorType<uint8_t>
{
public:
	typedef uint8_t value_type;
	typedef float alt_type;
	static const value_type ONE;
	static const alt_type ALTONE;
};
template<> class ColorType<uint32_t>
{
public:
	typedef uint32_t value_type;
	typedef float alt_type;
	static const value_type ONE;
	static const alt_type ALTONE;
};
template<> class ColorType<float>
{
public:
	typedef float value_type;
	typedef uint8_t alt_type;
	static const value_type ONE;
	static const alt_type ALTONE;
};
template<> class ColorType<double>
{
public:
	typedef double value_type;
	typedef uint8_t alt_type;
	static const value_type ONE;
	static const alt_type ALTONE;
};
/*----------------------------------------------------------------*/

template <typename TYPE>
struct TPixel {
	union {
		struct {
			#if _COLORMODE == _COLORMODE_BGR
			TYPE b;
			TYPE g;
			TYPE r;
			#endif
			#if _COLORMODE == _COLORMODE_RGB
			TYPE r;
			TYPE g;
			TYPE b;
			#endif
		};
		struct {
			TYPE c0;
			TYPE c1;
			TYPE c2;
		};
		TYPE c[3];
	};
	typedef typename ColorType<TYPE>::alt_type ALT;
	typedef TYPE Type;
	typedef TPoint3<TYPE> Pnt;
	static const TPixel BLACK;
	static const TPixel WHITE;
	static const TPixel GRAY;
	static const TPixel RED;
	static const TPixel GREEN;
	static const TPixel BLUE;
	static const TPixel YELLOW;
	static const TPixel MAGENTA;
	static const TPixel CYAN;
	// init
	inline TPixel() {}
	template <typename T> inline TPixel(const TPixel<T>& p)
		#if _COLORMODE == _COLORMODE_BGR
		: b(TYPE(p.b)), g(TYPE(p.g)), r(TYPE(p.r)) {}
		#endif
		#if _COLORMODE == _COLORMODE_RGB
		: r(TYPE(p.r)), g(TYPE(p.g)), b(TYPE(p.b)) {}
		#endif
	inline TPixel(TYPE _r, TYPE _g, TYPE _b)
		#if _COLORMODE == _COLORMODE_BGR
		: b(_b), g(_g), r(_r) {}
		#endif
		#if _COLORMODE == _COLORMODE_RGB
		: r(_r), g(_g), b(_b) {}
		#endif
	inline TPixel(const Pnt& col) : c0(col.x),  c1(col.y),  c2(col.z) {}
	explicit inline TPixel(uint32_t col)
		#if _COLORMODE == _COLORMODE_BGR
		: b(TYPE(col&0xFF)), g(TYPE((col>>8)&0xFF)), r(TYPE((col>>16)&0xFF)) {}
		#endif
		#if _COLORMODE == _COLORMODE_RGB
		: r(TYPE((col>>16)&0xFF)), g(TYPE((col>>8)&0xFF)), b(TYPE(col&0xFF)) {}
		#endif
	// set/get from default type
	inline void set(TYPE _r, TYPE _g, TYPE _b) { r = _r; g = _g; b = _b; }
	inline void set(const TYPE* clr) { c[0] = clr[0]; c[1] = clr[1]; c[2] = clr[2]; }
	inline void get(TYPE& _r, TYPE& _g, TYPE& _b) const { _r = r; _g = g; _b = b; }
	inline void get(TYPE* clr) const { clr[0] = c[0]; clr[1] = c[1]; clr[2] = c[2]; }
	// set/get from alternative type
	inline void set(ALT _r, ALT _g, ALT _b) { r = TYPE(_r); g = TYPE(_g); b = TYPE(_b); }
	inline void set(const ALT* clr) { c[0] = TYPE(clr[0]); c[1] = TYPE(clr[1]); c[2] = TYPE(clr[2]); }
	inline void get(ALT& _r, ALT& _g, ALT& _b) const { _r = ALT(r); _g = ALT(g); _b = ALT(b); }
	inline void get(ALT* clr) const { clr[0] = ALT(c[0]); clr[1] = ALT(c[1]); clr[2] = ALT(c[2]); }
	// set/get as vector
	inline const TYPE& operator[](size_t i) const { ASSERT(i<3); return c[i]; }
	inline TYPE& operator[](size_t i) { ASSERT(i<3); return c[i]; }
	// access as point equivalent
	template<typename T> inline operator TPoint3<T>() const { return TPoint3<T>(T(c[0]), T(c[1]), T(c[2])); }
	// access as vector equivalent
	inline operator const Pnt& () const { return *((const Pnt*)this); }
	inline operator Pnt& () { return *((Pnt*)this); }
	// access as cv::Scalar equivalent
	inline operator cv::Scalar () const { return cv::Scalar(c[0], c[1], c[2], TYPE(0)); }
	// compare
	inline bool operator==(const TPixel& col) const { return (memcmp(c, col.c, sizeof(TPixel)) == 0); }
	inline bool operator!=(const TPixel& col) const { return (memcmp(c, col.c, sizeof(TPixel)) != 0); }
	// operators
	inline TPixel operator*(const TPixel& v) const { return TPixel(r*v.r, g*v.g, b*v.b); }
	template<typename T> inline TPixel operator*(T v) const { return TPixel((TYPE)(v*r), (TYPE)(v*g), (TYPE)(v*b)); }
	template<typename T> inline TPixel& operator*=(T v) { return (*this = operator*(v)); }
	inline TPixel operator/(const TPixel& v) const { return TPixel(r/v.r, g/v.g, b/v.b); }
	template<typename T> inline TPixel operator/(T v) const { return operator*(T(1)/v); }
	template<typename T> inline TPixel& operator/=(T v) { return (*this = operator/(v)); }
	inline TPixel operator+(const TPixel& v) const { return TPixel(r+v.r, g+v.g, b+v.b); }
	template<typename T> inline TPixel operator+(T v) const { return TPixel((TYPE)(r+v), (TYPE)(g+v), (TYPE)(b+v)); }
	template<typename T> inline TPixel& operator+=(T v) { return (*this = operator+(v)); }
	inline TPixel operator-(const TPixel& v) const { return TPixel(r-v.r, g-v.g, b-v.b); }
	template<typename T> inline TPixel operator-(T v) const { return TPixel((TYPE)(r-v), (TYPE)(g-v), (TYPE)(b-v)); }
	template<typename T> inline TPixel& operator-=(T v) { return (*this = operator-(v)); }
	inline uint32_t toDWORD() const { return RGBA((uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)0); }
	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & c;
	}
	#endif
};
template <> inline void TPixel<float>::set(uint8_t _r, uint8_t _g, uint8_t _b) { r = float(_r)/255; g = float(_g)/255; b = float(_b)/255; }
template <> inline void TPixel<float>::get(uint8_t& _r, uint8_t& _g, uint8_t& _b) const { _r = uint8_t(r*255); _g = uint8_t(g*255); _b = uint8_t(b*255); }
template <> inline void TPixel<uint8_t>::set(float _r, float _g, float _b) { r = uint8_t(_r*255); g = uint8_t(_g*255); b = uint8_t(_b*255); }
template <> inline void TPixel<uint8_t>::get(float& _r, float& _g, float& _b) const { _r = float(r)/255; _g = float(g)/255; _b = float(b)/255; }
/*----------------------------------------------------------------*/
typedef TPixel<uint8_t> Pixel8U;
typedef TPixel<float> Pixel32F;
typedef TPixel<double> Pixel64F;
/*----------------------------------------------------------------*/

template <typename TYPE>
struct TColor {
	union {
		struct {
			#if _COLORMODE == _COLORMODE_BGR
			TYPE b;
			TYPE g;
			TYPE r;
			TYPE a;
			#endif
			#if _COLORMODE == _COLORMODE_RGB
			TYPE a;
			TYPE r;
			TYPE g;
			TYPE b;
			#endif
		};
		struct {
			TYPE c0;
			TYPE c1;
			TYPE c2;
			TYPE c3;
		};
		TYPE c[4];
	};
	typedef typename ColorType<TYPE>::alt_type ALT;
	typedef TYPE Type;
	typedef TPixel<TYPE> Pxl;
	typedef TPoint3<TYPE> Pnt;
	static const TColor BLACK;
	static const TColor WHITE;
	static const TColor GRAY;
	static const TColor RED;
	static const TColor GREEN;
	static const TColor BLUE;
	static const TColor YELLOW;
	static const TColor MAGENTA;
	static const TColor CYAN;
	// init
	inline TColor() {}
	template <typename T> inline TColor(const TColor<T>& p)
		: r(TYPE(p.r)), g(TYPE(p.g)), b(TYPE(p.b)), a(TYPE(p.a)) {}
	inline TColor(TYPE _r, TYPE _g, TYPE _b, TYPE _a=ColorType<TYPE>::ONE)
		: r(_r), g(_g), b(_b), a(_a) {}
	inline TColor(const Pxl& col, TYPE _a=ColorType<TYPE>::ONE)
		: r(col.r),  g(col.g),  b(col.b), a(_a) {}
	#if _COLORMODE == _COLORMODE_BGR
	inline TColor(const Pnt& col, TYPE _a=ColorType<TYPE>::ONE)
		: b(col.x),  g(col.y),  r(col.z),  a(_a) {}
	#endif
	#if _COLORMODE == _COLORMODE_RGB
	inline TColor(const Pnt& col, TYPE _a=ColorType<TYPE>::ONE)
		: r(col.x),  g(col.y),  b(col.z),  a(_a) {}
	#endif
	explicit inline TColor(uint32_t col)
		: r(TYPE((col>>16)&0xFF)), g(TYPE((col>>8)&0xFF)), b(TYPE(col&0xFF)), a(TYPE((col>>24)&0xFF)) {}
	// set/get from default type
	inline void set(TYPE _r, TYPE _g, TYPE _b, TYPE _a=ColorType<TYPE>::ONE) { r = _r; g = _g; b = _b; a = _a; }
	inline void set(const TYPE* clr) { c[0] = clr[0]; c[1] = clr[1]; c[2] = clr[2]; c[3] = clr[3]; }
	inline void get(TYPE& _r, TYPE& _g, TYPE& _b, TYPE& _a) const { _r = r; _g = g; _b = b; _a = a; }
	inline void get(TYPE* clr) const { clr[0] = c[0]; clr[1] = c[1]; clr[2] = c[2]; clr[3] = c[3]; }
	// set/get from alternative type
	inline void set(ALT _r, ALT _g, ALT _b, ALT _a=ColorType<TYPE>::ALTONE) { r = TYPE(_r); g = TYPE(_g); b = TYPE(_b); a = TYPE(_a); }
	inline void set(const ALT* clr) { c[0] = TYPE(clr[0]); c[1] = TYPE(clr[1]); c[2] = TYPE(clr[2]); c[3] = TYPE(clr[3]); }
	inline void get(ALT& _r, ALT& _g, ALT& _b, ALT& _a) const { _r = ALT(r); _g = ALT(g); _b = ALT(b); _a = ALT(a); }
	inline void get(ALT* clr) const { clr[0] = ALT(c[0]); clr[1] = ALT(c[1]); clr[2] = ALT(c[2]); clr[3] = ALT(c[3]); }
	// set/get as vector
	inline const TYPE& operator[](size_t i) const { ASSERT(i<4); return c[i]; }
	inline TYPE& operator[](size_t i) { ASSERT(i<4); return c[i]; }
	// access as pixel equivalent
	inline operator const Pxl& () const { return *((const Pxl*)this); }
	inline operator Pxl& () { return *((Pxl*)this); }
	// access as point equivalent
	inline operator const Pnt& () const { return *((const Pnt*)this); }
	inline operator Pnt& () { return *((Pnt*)this); }
	// access as cv::Scalar equivalent
	inline operator cv::Scalar () const { return cv::Scalar(c[0], c[1], c[2], c[3]); }
	// compare
	inline bool operator==(const TColor& col) const { return (memcmp(c, col.c, sizeof(TColor)) == 0); }
	inline bool operator!=(const TColor& col) const { return (memcmp(c, col.c, sizeof(TColor)) != 0); }
	// operators
	inline TColor operator*(const TColor& v) const { return TColor(r*v.r, g*v.g, b*v.b, a*v.a); }
	template<typename T> inline TColor operator*(T v) const { return TColor((TYPE)(v*r), (TYPE)(v*g), (TYPE)(v*b), (TYPE)(v*a)); }
	template<typename T> inline TColor& operator*=(T v) { return (*this = operator*(v)); }
	inline TColor operator/(const TColor& v) const { return TColor(r/v.r, g/v.g, b/v.b, a/v.a); }
	template<typename T> inline TColor operator/(T v) const { return operator*(T(1)/v); }
	template<typename T> inline TColor& operator/=(T v) { return (*this = operator/(v)); }
	inline TColor operator+(const TColor& v) const { return TColor(r+v.r, g+v.g, b+v.b, a+v.a); }
	template<typename T> inline TColor operator+(T v) const { return TColor((TYPE)(r+v), (TYPE)(g+v), (TYPE)(b+v), (TYPE)(a+v)); }
	template<typename T> inline TColor& operator+=(T v) { return (*this = operator+(v)); }
	inline TColor operator-(const TColor& v) const { return TColor(r-v.r, g-v.g, b-v.b, a-v.a); }
	template<typename T> inline TColor operator-(T v) const { return TColor((TYPE)(r-v), (TYPE)(g-v), (TYPE)(b-v), (TYPE)(a-v)); }
	template<typename T> inline TColor& operator-=(T v) { return (*this = operator-(v)); }
	inline uint32_t toDWORD() const { return RGBA((uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)a); }
	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & c;
	}
	#endif
};
template <> inline void TColor<float>::set(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a) { r = float(_r)/255; g = float(_g)/255; b = float(_b)/255; a = float(_a)/255; }
template <> inline void TColor<float>::get(uint8_t& _r, uint8_t& _g, uint8_t& _b, uint8_t& _a) const { _r = uint8_t(r*255); _g = uint8_t(g*255); _b = uint8_t(b*255); _a = uint8_t(a*255); }
template <> inline void TColor<uint8_t>::set(float _r, float _g, float _b, float _a) { r = uint8_t(_r*255); g = uint8_t(_g*255); b = uint8_t(_b*255); a = uint8_t(_a*255); }
template <> inline void TColor<uint8_t>::get(float& _r, float& _g, float& _b, float& _a) const { _r = float(r)/255; _g = float(g)/255; _b = float(b)/255; _a = float(a)/255; }
template <> inline bool TColor<uint8_t>::operator==(const TColor& col) const { return (*((const uint32_t*)c) == *((const uint32_t*)col.c)); }
template <> inline bool TColor<uint8_t>::operator!=(const TColor& col) const { return (*((const uint32_t*)c) != *((const uint32_t*)col.c)); }
template <> inline uint32_t TColor<uint8_t>::toDWORD() const { return *((const uint32_t*)c); }
/*----------------------------------------------------------------*/
typedef TColor<uint8_t> Color8U;
typedef TColor<float> Color32F;
typedef TColor<double> Color64F;
/*----------------------------------------------------------------*/


// structure containing the image pixels
typedef Point2i ImageRef;
template <typename TYPE>
class TImage : public TDMatrix<TYPE>
{
public:
	typedef TYPE Type;
	typedef TDMatrix<TYPE> Base;
	typedef cv::Size Size;
	typedef typename Base::Base BaseBase;

	using Base::rows;
	using Base::cols;
	using Base::data;

public:
	inline TImage() {}
	inline TImage(const Base& rhs) : Base(rhs) {}
	inline TImage(const cv::Mat& rhs) : Base(rhs) {}
	inline TImage(const cv::MatExpr& rhs) : Base(rhs) {}
	inline TImage(int _rows, int _cols) : Base(_rows, _cols) {}
	inline TImage(const Size& sz) : Base(sz) {}
	inline TImage(const Size& sz, const TYPE& v) : Base(sz, v) {}
	inline TImage(const Size& sz, TYPE* _data, size_t _step=Base::AUTO_STEP) : Base(sz.height, sz.width, _data, _step) {}

	inline TImage& operator = (const Base& rhs) { BaseBase::operator=(rhs); return *this; }
	inline TImage& operator = (const BaseBase& rhs) { BaseBase::operator=(rhs); return *this; }
	inline TImage& operator = (const cv::MatExpr& rhs) { BaseBase::operator=(rhs); return *this; }

	/// What is the image size?
	inline int width() const { return cols; }
	inline int height() const { return rows; }

	/// What is the pixel stride of the image?
	inline size_t pixel_stride() const { return Base::elem_stride(); }

	inline const TYPE& getPixel(int y, int x) const;

	template <typename T>
	TYPE sample(const TPoint2<T>& pt) const;
	template <typename T>
	TYPE sampleSafe(const TPoint2<T>& pt) const;

	template <typename T>
	bool sample(TYPE& v, const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&)) const;
	template <typename T>
	bool sampleSafe(TYPE& v, const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&)) const;
	template <typename T>
	TYPE sample(const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&), const TYPE& dv) const;
	template <typename T>
	TYPE sampleSafe(const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&), const TYPE& dv) const;

	template <typename SAMPLER, typename INTERTYPE>
	INTERTYPE sample(const SAMPLER& sampler, const TPoint2<typename SAMPLER::Type>& pt) const;

	template <typename T>
	void toGray(TImage<T>& out, int code, bool bNormalize=false) const;

	unsigned computeMaxResolution(unsigned& level, unsigned minImageSize=320, unsigned maxImageSize=INT_MAX) const;
	static unsigned computeMaxResolution(unsigned width, unsigned height, unsigned& level, unsigned minImageSize=320, unsigned maxImageSize=INT_MAX);

	template <typename T, typename PARSER>
	static void RasterizeTriangle(const TPoint2<T>& v1, const TPoint2<T>& v2, const TPoint2<T>& v3, PARSER& parser);
	template <typename T, typename PARSER>
	static void RasterizeTriangleDepth(TPoint3<T> p1, TPoint3<T> p2, TPoint3<T> p3, PARSER& parser);

	template <typename T, typename PARSER>
	static void DrawLine(const TPoint2<T>& p1, const TPoint2<T>& p2, PARSER& parser);

	typedef void (STCALL *FncDrawPointAntialias) (const ImageRef&, const ImageRef&, float, float, void*);
	static bool DrawLineAntialias(Point2f x1, Point2f x2, FncDrawPointAntialias fncDrawPoint, void* pData=NULL);
	static bool DrawLineAntialias(const ImageRef& x1, const ImageRef& x2, FncDrawPointAntialias fncDrawPoint, void* pData=NULL);

	template <int HalfSize>
	void DilateMean(TImage<TYPE>& out, const TYPE& invalid) const;

	bool Load(const String&);
	bool Save(const String&) const;
	void Show(const String& winname, int delay=0, bool bDestroy=true) const;

	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Base>(*this);
	}
	#endif
};
/*----------------------------------------------------------------*/
typedef TImage<uint8_t> Image8U;
typedef TImage<hfloat> Image16F;
typedef TImage<float> Image32F;
typedef TImage<double> Image64F;
typedef TImage<Pixel8U> Image8U3;
typedef TImage<Color8U> Image8U4;
typedef TImage<Pixel32F> Image32F3;
typedef TImage<Color32F> Image32F4;
/*----------------------------------------------------------------*/


template <typename TYPE>
class TBitMatrix
{
public:
	typedef TYPE Type;
	typedef cv::Size Size;
	struct Index {
		size_t idx;
		Type flag;
		inline Index() {}
		inline Index(size_t _idx, Type _flag) : idx(_idx), flag(_flag) {}
	};

	enum { numBitsPerCell = sizeof(Type)*8 };
	enum { numBitsShift = LOG2I<TBitMatrix::numBitsPerCell>() };

public:
	inline TBitMatrix() : data(NULL) {}
	inline TBitMatrix(int _rows, int _cols=1) : rows(_rows), cols(_cols), data(rows && cols ? new Type[computeLength(rows, cols)] : NULL) {}
	inline TBitMatrix(int _rows, int _cols, uint8_t v) : rows(_rows), cols(_cols), data(rows && cols ? new Type[computeLength(rows, cols)] : NULL) { if (!empty()) memset(v); }
	inline TBitMatrix(const Size& sz) : rows(sz.height), cols(sz.width), data(rows && cols ? new Type[computeLength(sz)] : NULL) {}
	inline TBitMatrix(const Size& sz, uint8_t v) : rows(sz.height), cols(sz.width), data(rows && cols ? new Type[computeLength(sz)] : NULL) { if (!empty()) memset(v); }
	inline ~TBitMatrix() { delete[] data; }

	inline void create(int _rows, int _cols=1) {
		if (!empty() && rows == _rows && cols == _cols)
			return;
		rows = _rows; cols = _cols;
		if (rows <=0 || cols <= 0) {
			release();
			return;
		}
		delete[] data;
		data = new Type[length()];
	}
	inline void create(const Size& sz) { create(sz.height, sz.width); }
	inline void release() { delete[] data; data = NULL; }
	inline void memset(uint8_t v) { ASSERT(!empty()); ::memset(data, v, sizeof(Type)*length()); }
	inline void swap(TBitMatrix& m) {
		union {int i; Type* d;} tmp;
		tmp.i = rows; rows = m.rows; m.rows = tmp.i;
		tmp.i = cols; cols = m.cols; m.cols = tmp.i;
		tmp.d = data; data = m.data; m.data = tmp.d;
	}

	inline void And(const TBitMatrix& m) {
		ASSERT(rows == m.rows && cols == m.cols);
		int i = length();
		while (i-- > 0)
			data[i] &= m.data[i];
	}
	inline void Or(const TBitMatrix& m) {
		ASSERT(rows == m.rows && cols == m.cols);
		int i = length();
		while (i-- > 0)
			data[i] |= m.data[i];
	}
	inline void XOr(const TBitMatrix& m) {
		ASSERT(rows == m.rows && cols == m.cols);
		int i = length();
		while (i-- > 0)
			data[i] ^= m.data[i];
	}

	inline bool empty() const { return data == NULL; }
	inline Size size() const { return Size(cols, rows); }
	inline int area() const { return cols*rows; }
	inline int length() const { return computeLength(rows, cols); }

	inline bool operator() (int i) const { return isSet(i); }
	inline bool operator() (int i, int j) const { return isSet(i,j); }
	inline bool operator() (const ImageRef& ir) const { return isSet(ir); }

	inline bool isSet(int i) const { ASSERT(!empty() && i<area()); const Index idx(computeIndex(i)); return (data[idx.idx] & idx.flag) != 0; }
	inline bool isSet(int i, int j) const { ASSERT(!empty() && i<rows && j<cols); const Index idx(computeIndex(i, j, cols)); return (data[idx.idx] & idx.flag) != 0; }
	inline bool isSet(const ImageRef& ir) const { ASSERT(!empty() && ir.y<rows && ir.x<cols); const Index idx(computeIndex(ir, cols)); return (data[idx.idx] & idx.flag) != 0; }

	inline void set(int i, bool v) { if (v) set(i); else unset(i); }
	inline void set(int i, int j, bool v) { if (v) set(i, j); else unset(i, j); }
	inline void set(const ImageRef& ir, bool v) { if (v) set(ir); else unset(ir); }

	inline void set(int i) { ASSERT(!empty() && i<area()); const Index idx(computeIndex(i)); data[idx.idx] |= idx.flag; }
	inline void set(int i, int j) { ASSERT(!empty() && i<rows && j<cols); const Index idx(computeIndex(i, j, cols)); data[idx.idx] |= idx.flag; }
	inline void set(const ImageRef& ir) { ASSERT(!empty() && ir.y<rows && ir.x<cols); const Index idx(computeIndex(ir, cols)); data[idx.idx] |= idx.flag; }

	inline void unset(int i) { ASSERT(!empty() && i<area()); const Index idx(computeIndex(i)); data[idx.idx] &= ~idx.flag; }
	inline void unset(int i, int j) { ASSERT(!empty() && i<rows && j<cols); const Index idx(computeIndex(i, j, cols)); data[idx.idx] &= ~idx.flag; }
	inline void unset(const ImageRef& ir) { ASSERT(!empty() && ir.y<rows && ir.x<cols); const Index idx(computeIndex(ir, cols)); data[idx.idx] &= ~idx.flag; }

	inline void flip(int i) { ASSERT(!empty() && i<area()); const Index idx(computeIndex(i)); data[idx.idx] ^= idx.flag; }
	inline void flip(int i, int j) { ASSERT(!empty() && i<rows && j<cols); const Index idx(computeIndex(i, j, cols)); data[idx.idx] ^= idx.flag; }
	inline void flip(const ImageRef& ir) { ASSERT(!empty() && ir.y<rows && ir.x<cols); const Index idx(computeIndex(ir, cols)); data[idx.idx] ^= idx.flag; }

	/// Is this coordinate inside the 2D matrix?
	inline bool isInside(const Size& pt) const {
		return pt.width>=0 && pt.height>=0 && pt.width<size().width && pt.height<size().height;
	}
	template <typename T>
	inline bool isInside(const cv::Point_<T>& pt) const {
		return int(pt.x)>=0 && int(pt.y)>=0 && int(pt.x)<size().width && int(pt.y)<size().height;
	}

	/// Is this coordinate inside the 2D matrix, and not too close to the edges?
	/// @param border: the size of the border
	inline bool isInsideWithBorder(const Size& pt, int border) const {
		return pt.width>=border && pt.height>=border && pt.width<size().width-border && pt.height<size().height-border;
	}
	template <typename T>
	inline bool isInsideWithBorder(const cv::Point_<T>& pt, int border) const {
		return int(pt.x)>=border && int(pt.y)>=border && int(pt.x)<size().width-border && int(pt.y)<size().height-border;
	}
	template <typename T, int border>
	inline bool isInsideWithBorder(const cv::Point_<T>& pt) const {
		return int(pt.x)>=border && int(pt.y)>=border && int(pt.x)<size().width-border && int(pt.y)<size().height-border;
	}

	static inline int computeLength(int size) { return (size+numBitsPerCell-1)>>numBitsShift; }
	static inline int computeLength(int _rows, int _cols) { return computeLength(_rows*_cols); }
	static inline int computeLength(const Size& sz) { return computeLength(sz.area()); }

	static inline Index computeIndex(int i) { return Index(i>>numBitsShift, Type(1)<<(i&(numBitsPerCell-1))); }
	static inline Index computeIndex(int i, int j, int stride) { return computeIndex(i*stride+j); }
	static inline Index computeIndex(const ImageRef& ir, int stride) { return computeIndex(ir.y*stride+ir.x); }

	//! the number of rows and columns
	int rows, cols;
	//! pointer to the data
	Type* data;

#ifdef _USE_BOOST
protected:
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const {
		if (empty()) {
			const int sz(0);
			ar & sz;
			return;
		}
		ar & cols;
		ar & rows;
		ar & boost::serialization::make_array(data, length());
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/) {
		release();
		ar & cols;
		if (cols == 0)
			return;
		ar & rows;
		create(rows, cols);
		ar & boost::serialization::make_array(data, length());
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
#endif
};
/*----------------------------------------------------------------*/
typedef TBitMatrix<size_t> BitMatrix;
/*----------------------------------------------------------------*/


// weighted accumulator class that operates on arbitrary types
template <typename TYPE, typename ACCUMTYPE=TYPE, typename WEIGHTTYPE=float>
struct TAccumulator {
	typedef TYPE Type;
	typedef ACCUMTYPE AccumType;
	typedef WEIGHTTYPE WeightType;

	AccumType value;
	WeightType weight;

	inline TAccumulator() : value(0), weight(0) {}
	inline TAccumulator(const Type& v, const WeightType& w) : value(v), weight(w) {}
	// adds the given weighted value to the internal value
	inline void Add(const Type& v, const WeightType& w) {
		value += v*w;
		weight += w;
	}
	inline TAccumulator& operator +=(const TAccumulator& accum) {
		value += accum.value;
		weight += accum.weight;
		return *this;
	}
	inline TAccumulator operator +(const TAccumulator& accum) const {
		return TAccumulator(
			value + accum.value,
			weight + accum.weight
		);
	}
	// subtracts the given weighted value to the internal value
	inline void Sub(const Type& v, const WeightType& w) {
		value -= v*w;
		weight -= w;
	}
	inline TAccumulator& operator -=(const TAccumulator& accum) {
		value -= accum.value;
		weight -= accum.weight;
		return *this;
	}
	inline TAccumulator operator -(const TAccumulator& accum) const {
		return TAccumulator(
			value - accum.value,
			weight - accum.weight
		);
	}
	// returns the normalized version of the internal value
	inline AccumType NormalizedFull() const {
		return value / weight;
	}
	inline Type Normalized() const {
		return Type(NormalizedFull());
	}
	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & value;
		ar & weight;
	}
	#endif
};
/*----------------------------------------------------------------*/


// structure used for sorting some indices by their score (decreasing by default)
template <typename IndexType, typename ScoreType>
struct TIndexScore {
	IndexType idx;
	ScoreType score;
	inline TIndexScore() {}
	inline TIndexScore(IndexType _idx, ScoreType _score) : idx(_idx), score(_score) {}
	// compare by index (increasing)
	inline bool operator<(IndexType i) const { return (idx < i); }
	inline bool operator==(IndexType i) const { return (idx == i); }
	// compare by score (decreasing)
	inline bool operator<(const TIndexScore& r) const { return (score > r.score); }
	inline bool operator==(const TIndexScore& r) const { return (score == r.score); }
	static bool STCALL CompareByIndex(const TIndexScore& l, const TIndexScore& r) { return (l.idx < r.idx); }
	static bool STCALL CompareByScore(const TIndexScore& l, const TIndexScore& r) { return (r.score < l.score); }
	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & idx;
		ar & score;
	}
	#endif
};
/*----------------------------------------------------------------*/
typedef TIndexScore<uint32_t, float> IndexScore;
typedef SEACAVE::cList<IndexScore, const IndexScore&, 0> IndexScoreArr;
/*----------------------------------------------------------------*/


// structure describing a pair of indices as one long index
struct PairIdx {
	typedef uint64_t PairIndex;
	typedef uint32_t Index;
	union {
		PairIndex idx;
		Index indices[2];
		struct {
			#if __BYTE_ORDER == __LITTLE_ENDIAN
			Index j;
			Index i;
			#else
			Index i;
			Index j;
			#endif
		};
	};
	inline PairIdx() {}
	inline PairIdx(PairIndex _idx) : idx(_idx) {}
	inline PairIdx(Index _i, Index _j)
		#if __BYTE_ORDER == __LITTLE_ENDIAN
		: j(_j), i(_i) {}
		#else
		: i(_i), j(_j) {}
		#endif
	// get index
	inline operator PairIndex () const { return idx; }
	inline Index operator[](unsigned n) const {
		ASSERT(n < 2);
		#if __BYTE_ORDER == __LITTLE_ENDIAN
		return indices[(n+1)%2];
		#else
		return indices[n];
		#endif
	}
	inline Index& operator[](unsigned n) {
		ASSERT(n < 2);
		#if __BYTE_ORDER == __LITTLE_ENDIAN
		return indices[(n+1)%2];
		#else
		return indices[n];
		#endif
	}
	// compare by index (increasing)
	inline bool operator<(const PairIdx& r) const { return (idx < r.idx); }
	inline bool operator==(const PairIdx& r) const { return (idx == r.idx); }
};
/*----------------------------------------------------------------*/
typedef SEACAVE::cList<PairIdx, const PairIdx&, 0> PairIdxArr;
inline PairIdx MakePairIdx(uint32_t idxImageA, uint32_t idxImageB) {
	return (idxImageA<idxImageB ? PairIdx(idxImageA, idxImageB) : PairIdx(idxImageB, idxImageA));
}
/*----------------------------------------------------------------*/


struct cuint32_t {
	typedef uint32_t Type;
	union {
		uint32_t idx;
		uint8_t i;
	};
	inline cuint32_t() {}
	inline cuint32_t(uint32_t _idx) : idx(_idx) {}
	inline operator uint32_t () const { return idx; }
	inline operator uint32_t& () { return idx; }
};
/*----------------------------------------------------------------*/


// tools
String cvMat2String(const cv::Mat&, LPCSTR format="% 10.4f ");
template<typename TYPE, int m, int n> inline String cvMat2String(const TMatrix<TYPE,m,n>& mat, LPCSTR format="% 10.4f ") { return cvMat2String(cv::Mat(mat), format); }
template<typename TYPE> inline String cvMat2String(const TPoint3<TYPE>& pt, LPCSTR format="% 10.4f ") { return cvMat2String(cv::Mat(pt), format); }
/*----------------------------------------------------------------*/

} // namespace SEACAVE


#ifdef _USE_EIGEN

// Implement SO3 and SO2 lie groups
// as in TooN library: https://github.com/edrosten/TooN
// Copyright (C) 2005,2009 Tom Drummond (twd20@cam.ac.uk)

namespace Eigen {

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
	inline SO3(const Vec3& v) : mat(exp(v)) {}

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
		n *= Precision(1)/sqrt(nrmSq);
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
	inline Mat3 exp(const Vec3& vect) const;

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

	#ifdef _USE_BOOST
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const
	{
		Vec3 comp(ln());
		ar & comp;
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/)
	{
		Vec3 comp;
		ar & comp;
		mat = exp(comp);
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
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
	inline SO2(const Precision l) : mat(exp(l)) {}

	/// Assigment operator from a general matrix. This also calls coerce()
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
	inline Mat2 exp(const Precision& d) const;

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

	#ifdef _USE_BOOST
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const
	{
		Precision comp(ln());
		ar & comp;
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/)
	{
		Precision comp;
		ar & comp;
		mat = exp(comp);
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
};
/*----------------------------------------------------------------*/

} // namespace Eigen

#endif // _USE_EIGEN

#include "Types.inl"
#include "Rotation.h"
#include "Sphere.h"
#include "AABB.h"
#include "OBB.h"
#include "Plane.h"
#include "Ray.h"
#include "Octree.h"
#include "Util.inl"
#include "CUDA.h"

#endif // __SEACAVE_TYPES_H__
