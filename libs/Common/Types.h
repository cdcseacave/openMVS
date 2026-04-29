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
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#endif
#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <initializer_list>
#ifdef _SUPPORT_CPP17
#if !defined(__GNUC__) || (__GNUC__ > 7)
#include <filesystem>
#endif
#endif
#include <new>
#include <memory>
#include <string>
#include <codecvt>
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
#include <optional>
#include <vector>
#include <list>
#include <queue>
#include <deque>
#include <iterator>
#include <chrono>
#include <cmath>
#include <ctime>
#include <random>
#include <stdexcept>
#include <thread>
#ifdef _USE_OPENMP
#include <omp.h>
#endif

// Function delegate functionality
#include "FastDelegate.h"
#define DELEGATE fastdelegate::delegate
#define DELEGATEBIND(DLGT, FNC) DLGT::from< FNC >()
#define DELEGATEBINDCLASS(DLGT, FNC, OBJ) DLGT::from(*OBJ, FNC)

// include usual boost libraries
#ifdef _USE_BOOST
// In static builds we suppress boost's exception-unwinding machinery to keep
// archive size down and provide a user-defined `boost::throw_exception` in
// libs/Common/Common.cpp. In shared builds that approach is fragile across
// DLL boundaries (forward decls in boost headers don't carry dllexport, so
// SFM/MVS can't link the user-defined version), and there is no archive-size
// concern, so let boost use its native exception path. The OPENMVS_SHARED
// macro is defined globally via CMake when BUILD_SHARED_LIBS=ON.
#ifndef OPENMVS_SHARED
#define BOOST_NO_UNREACHABLE_RETURN_DETECTION
#define BOOST_EXCEPTION_DISABLE
#define BOOST_NO_EXCEPTIONS
#endif
#ifdef BOOST_NO_EXCEPTIONS
#include <boost/throw_exception.hpp>
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

#pragma push_macro("malloc")
#undef malloc
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
#pragma pop_macro("malloc")

#pragma push_macro("malloc")
#undef malloc
#pragma push_macro("free")
#undef free
#include <nanoflann.hpp>
#pragma pop_macro("free")
#pragma pop_macro("malloc")

#if defined(_MSC_VER)
#define __LITTLE_ENDIAN 0
#define __BIG_ENDIAN 1
#define __PDP_ENDIAN 2
#define __BYTE_ORDER __LITTLE_ENDIAN
#elif defined(__APPLE__)
#include <machine/endian.h>
#elif defined(__GNUC__)
#include <endian.h>
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
// define string related types
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

GENERAL_API int _vscprintf(LPCSTR format, va_list pargs);

#define _T(s)               s
#endif // _MSC_VER

#ifndef MAX_PATH
#define MAX_PATH			260
#endif

#ifndef NULL
#define NULL				0
#endif


// functions simplifying the task of printing messages
namespace SEACAVE {
// print the given message composed of any number of arguments to the given stream
template<typename... Args>
std::ostringstream& PrintMessageToStream(std::ostringstream& oss, Args&&... args) {
	// fold expression to insert all arguments into the stream
	(oss << ... << args);
	return oss;
}
// print the given message composed of any number of arguments to a string
template<typename... Args>
std::string PrintMessageToString(Args&&... args) {
	std::ostringstream oss;
	(oss << ... << args);
	return oss.str();
}
} // namespace SEACAVE


// I N C L U D E S /////////////////////////////////////////////////

#include "Maths.h"
#include "Strings.h"
#include "AutoPtr.h"
#include "List.h"
#include "Thread.h"
#include "SharedPtr.h"
#include "Queue.h"
#include "Hash.h"
#include "Timer.h"
#include "CriticalSection.h"
#include "Semaphore.h"
#include "RunningAverage.h"
#include "Util.h"
#include "File.h"
#include "MemFile.h"
#include "LinkLib.h"

namespace SEACAVE {

// CSharedPtr is an inline-only template; each consumer TU instantiates locally
// rather than importing a pre-instantiated copy from Common.dll. Tagging these
// typedefs with GENERAL_API would mark the template as dllimport in consumers
// without Common.dll actually exporting any out-of-line symbol for it.
typedef CSharedPtr<File>								FilePtr;

typedef CSharedPtr<ISTREAM>								ISTREAMPTR;
typedef ISTREAM*										LPISTREAM;

typedef CSharedPtr<OSTREAM>								OSTREAMPTR;
typedef OSTREAM*										LPOSTREAM;

typedef CSharedPtr<IOSTREAM>							IOSTREAMPTR;
typedef IOSTREAM*										LPIOSTREAM;

// cList<T> is a header-only template; each consumer TU instantiates locally.
// Tagging these typedefs with GENERAL_API would mark them dllimport in
// consumers without Common.dll providing matching exports for the inline-only
// member functions, producing LNK2001s.
typedef cList<void*, void*, 0>          VoidArr;
typedef cList<LPCTSTR, LPCTSTR, 0>		LPCTSTRArr;
typedef cList<String>                   StringArr;
typedef cList<IDX, IDX, 0>				IDXArr;
typedef cList<uint8_t, uint8_t, 0>      Unsigned8Arr;
typedef cList<unsigned, unsigned, 0>    UnsignedArr;
typedef cList<uint32_t, uint32_t, 0>    Unsigned32Arr;
typedef cList<uint64_t, uint64_t, 0>    Unsigned64Arr;
typedef cList<size_t, size_t, 0>        SizeArr;
typedef cList<int, int, 0>              IntArr;
typedef cList<bool, bool, 0>            BoolArr;
typedef cList<float, float, 0>          FloatArr;
typedef cList<double, double, 0>        DoubleArr;

} // namespace SEACAVE

#include "Log.h"
#include "EventQueue.h"
#include "SML.h"
#include "ConfigTable.h"


// D E F I N E S ///////////////////////////////////////////////////

//
// Constant defines

#define TIMER_START()		SEACAVE::Timer::SysType timerStart = SEACAVE::Timer::GetSysTime()
#define TIMER_UPDATE(name)	SEACAVE::Timer::Type time##name = SEACAVE::Timer::GetTimeElapsedMsUpdate(timerStart)
#define TIMER_GET()			SEACAVE::Timer::SysTime2TimeMs(SEACAVE::Timer::GetSysTime() - timerStart)
#define TIMER_GET_INT()		((SEACAVE::Timer::SysType)TIMER_GET())
#define TIMER_GET_FORMAT()	SEACAVE::Util::formatTime(TIMER_GET_INT())

#ifndef CHECK
#define CHECK(exp)			{ if (!(exp)) { VERBOSE("Check failed: " #exp); abort(); } }
#endif
#ifndef ABORT
#define ABORT(msg)			{ VERBOSE("error: " #msg); exit(-1); }
#endif


// I N C L U D E S /////////////////////////////////////////////////

#include "Random.h"
#include "HalfFloat.h"


namespace SEACAVE {

// P R O T O T Y P E S /////////////////////////////////////////////

template <typename TYPE, int m, int n> class TMatrix;
template <typename TYPE, int DIMS> class TAABB;
template <typename TYPE, int DIMS> class TRay;
template <typename TYPE, int DIMS> class TPlane;
template <typename TYPE> class TPoint3;

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
	typedef Eigen::Map<const EVec> CEVecMap;
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
	inline TPoint2(const EVec& rhs) { operator EVecMap () = rhs; }
	#endif
	explicit inline TPoint2(const TYPE& _x) : Base(_x,_x) {}
	inline TPoint2(const TYPE& _x, const TYPE& _y) : Base(_x,_y) {}
	explicit inline TPoint2(const cv::Point3_<TYPE>& pt) : Base(pt.x/pt.z,pt.y/pt.z) {}

	template <typename T> inline TPoint2& operator = (const cv::Point_<T>& rhs) { Base::operator = (rhs); return *this; }
	template <typename T> inline TPoint2& operator = (const cv::Matx<T,2,1>& rhs) { operator Vec& () = rhs; return *this; }
	template <typename T> inline TPoint2& operator = (const cv::Matx<T,1,2>& rhs) { operator VecT& () = rhs; return *this; }
	#ifdef _USE_EIGEN
	inline TPoint2& operator = (const EVec& rhs) { operator EVecMap () = rhs; return *this; }
	#endif

	// conversion to another data type
	template <typename T> inline operator TPoint2<T> () const { return TPoint2<T>((T)x,(T)y); }

	// pointer to the first element access
	inline const TYPE* ptr() const { return &x; }
	inline TYPE* ptr() { return &x; }

	// iterator base access to enable range-based for loops
	inline const TYPE* begin() const { return &x; }
	inline const TYPE* end() const { return &x+2; }

	// get homogeneous coordinates
	inline TPoint3<TYPE> homogeneous() const { return TPoint3<TYPE>(x, y, TYPE(1)); }

	// 1D element access
	inline const TYPE& operator ()(int i) const { ASSERT(i>=0 && i<2); return ptr()[i]; }
	inline TYPE& operator ()(int i) { ASSERT(i>=0 && i<2); return ptr()[i]; }
	inline const TYPE& operator [](int i) const { ASSERT(i>=0 && i<2); return ptr()[i]; }
	inline TYPE& operator [](int i) { ASSERT(i>=0 && i<2); return ptr()[i]; }

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
	inline operator EVec () const { return CEVecMap(ptr()); }
	// Access point as Eigen::Map equivalent
	inline operator CEVecMap () const { return CEVecMap(ptr()); }
	inline operator EVecMap () { return EVecMap(ptr()); }
	#endif

	// cross product
	inline TYPE cross(const Base& v) const { return x * v.y - y * v.x; }
	inline TYPE cross(const Vec& v) const { return x * v(1) - y * v(0); }

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
	typedef Eigen::Map<const EVec> CEVecMap;
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
	inline TPoint3(const EVec& rhs) { operator EVecMap () = rhs; }
	#endif
	explicit inline TPoint3(const TYPE& _x) : Base(_x,_x,_x) {}
	inline TPoint3(const TYPE& _x, const TYPE& _y, const TYPE& _z) : Base(_x,_y,_z) {}
	template <typename T> inline TPoint3(const cv::Point_<T>& pt, const T& _z=T(1)) : Base(pt.x,pt.y,_z) {}
	template <typename T1, typename T2> inline TPoint3(const cv::Point_<T1>& pt, const T2& _z) : Base(pt.x,pt.y,_z) {}

	template <typename T> inline TPoint3& operator = (const cv::Point3_<T>& rhs)  { Base::operator = (rhs); return *this; }
	template <typename T> inline TPoint3& operator = (const cv::Matx<T,3,1>& rhs)  { operator Vec& () = rhs; return *this; }
	template <typename T> inline TPoint3& operator = (const cv::Matx<T,1,3>& rhs)  { operator VecT& () = rhs; return *this; }
	#ifdef _USE_EIGEN
	inline TPoint3& operator = (const EVec& rhs) { operator EVecMap () = rhs; return *this; }
	#endif

	// conversion to another data type
	template <typename T> inline operator TPoint3<T> () const { return TPoint3<T>((T)x,(T)y,(T)z); }

	// pointer to the first element access
	inline const TYPE* ptr() const { return &x; }
	inline TYPE* ptr() { return &x; }

	// iterator base access to enable range-based for loops
	inline const TYPE* begin() const { return &x; }
	inline const TYPE* end() const { return &x+3; }

	// 1D element access
	inline const TYPE& operator ()(int i) const { ASSERT(i>=0 && i<3); return ptr()[i]; }
	inline TYPE& operator ()(int i) { ASSERT(i>=0 && i<3); return ptr()[i]; }
	inline const TYPE& operator [](int i) const { ASSERT(i>=0 && i<3); return ptr()[i]; }
	inline TYPE& operator [](int i) { ASSERT(i>=0 && i<3); return ptr()[i]; }

	// Access point as vector equivalent
	inline operator const Vec& () const { return *reinterpret_cast<const Vec*>(this); }
	inline operator Vec& () { return *reinterpret_cast<Vec*>(this); }

	// Access point as transposed vector equivalent
	inline operator const VecT& () const { return *reinterpret_cast<const VecT*>(this); }
	inline operator VecT& () { return *reinterpret_cast<VecT*>(this); }

	#ifdef _USE_EIGEN
	// Access point as Eigen equivalent
	inline operator EVec () const { return CEVecMap(ptr()); }
	// Access point as Eigen::Map equivalent
	inline operator CEVecMap () const { return CEVecMap(ptr()); }
	inline operator EVecMap () { return EVecMap(ptr()); }
	#endif

	// rotate point using the given parametrized rotation (axis-angle)
	inline void RotateAngleAxis(const TPoint3& rot) { return (*this) = RotateAngleAxis((*this), rot); }
	static TPoint3 RotateAngleAxis(const TPoint3& X, const TPoint3& rot);

	// dot/cross product
	inline TYPE dot(const Base& v) const { return x*v.x + y*v.y + z*v.z; }
	inline TYPE dot(const Vec& v) const { return x*v(0) + y*v(1) + z*v(2); }
	inline TPoint3 cross(const Base& v) const { return TPoint3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
	inline TPoint3 cross(const Vec& v) const { return TPoint3(y*v(2)-z*v(1), z*v(0)-x*v(2), x*v(1)-y*v(0)); }

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
	using Base::channels;

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
	template <typename Derived, typename std::enable_if<(
		Derived::RowsAtCompileTime == m && Derived::ColsAtCompileTime == n
	), int>::type = 0>
	inline TMatrix(const Eigen::MatrixBase<Derived>& rhs) {
		operator EMatMap () = rhs;
	}
	#endif

	TMatrix(TYPE v0); //!< 1x1 matrix
	TMatrix(TYPE v0, TYPE v1); //!< 1x2 or 2x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2); //!< 1x3 or 3x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3); //!< 1x4, 2x2 or 4x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4); //!< 1x5 or 5x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5); //!< 1x6, 2x3, 3x2 or 6x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6); //!< 1x7 or 7x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7); //!< 1x8, 2x4, 4x2 or 8x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7, TYPE v8); //!< 1x9, 3x3 or 9x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7, TYPE v8, TYPE v9); //!< 1x10, 2x5 or 5x2 or 10x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3,
		TYPE v4, TYPE v5, TYPE v6, TYPE v7,
		TYPE v8, TYPE v9, TYPE v10, TYPE v11); //!< 1x12, 2x6, 3x4, 4x3, 6x2 or 12x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3,
		TYPE v4, TYPE v5, TYPE v6, TYPE v7,
		TYPE v8, TYPE v9, TYPE v10, TYPE v11,
		TYPE v12, TYPE v13); //!< 1x14, 2x7, 7x2 or 14x1 matrix
	TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3,
		TYPE v4, TYPE v5, TYPE v6, TYPE v7,
		TYPE v8, TYPE v9, TYPE v10, TYPE v11,
		TYPE v12, TYPE v13, TYPE v14, TYPE v15); //!< 1x16, 4x4 or 16x1 matrix
	explicit TMatrix(const TYPE* vals); //!< initialize from a plain array

	TMatrix(const TMatrix<TYPE,m,n>& a, const TMatrix<TYPE,m,n>& b, cv::Matx_AddOp) : Base(a, b, cv::Matx_AddOp()) {}
	TMatrix(const TMatrix<TYPE,m,n>& a, const TMatrix<TYPE,m,n>& b, cv::Matx_SubOp) : Base(a, b, cv::Matx_SubOp()) {}
	template<typename TYPE2> TMatrix(const TMatrix<TYPE,m,n>& a, TYPE2 alpha, cv::Matx_ScaleOp) : Base(a, alpha, cv::Matx_ScaleOp()) {}
	TMatrix(const TMatrix<TYPE,m,n>& a, const TMatrix<TYPE,m,n>& b, cv::Matx_MulOp) : Base(a, b, cv::Matx_MulOp()) {}
	TMatrix(const TMatrix<TYPE,m,n>& a, const TMatrix<TYPE,m,n>& b, cv::Matx_DivOp) : Base(a, b, cv::Matx_DivOp()) {}
	template<int l> TMatrix(const TMatrix<TYPE,m,l>& a, const TMatrix<TYPE,l,n>& b, cv::Matx_MatMulOp) : Base(a, b, cv::Matx_MatMulOp()) {}
	TMatrix(const TMatrix<TYPE,n,m>& a, cv::Matx_TOp) : Base(a, cv::Matx_TOp()) {}

	template <typename T> inline TMatrix& operator = (const cv::Matx<T,m,n>& rhs) { Base::operator = (rhs); return *this; }
	inline TMatrix& operator = (const cv::Mat& rhs) { Base::operator = (rhs); return *this; }
	#ifdef _USE_EIGEN
	template <typename Derived, typename std::enable_if<(
		Derived::RowsAtCompileTime == m && Derived::ColsAtCompileTime == n
	), int>::type = 0>
	inline TMatrix& operator = (const Eigen::MatrixBase<Derived>& rhs) {
		operator EMatMap () = rhs;
		return *this;
	}
	#endif

	inline bool IsEqual(const Base&) const;
	inline bool IsEqual(const Base&, TYPE eps) const;

	// 1D element access
	inline const TYPE& operator [](size_t i) const { ASSERT(i<elems); return val[i]; }
	inline TYPE& operator [](size_t i) { ASSERT(i<elems); return val[i]; }

	// Access point as vector equivalent
	inline operator const Vec& () const { return *reinterpret_cast<const Vec*>(this); }
	inline operator Vec& () { return *reinterpret_cast<Vec*>(this); }

	#ifdef _USE_EIGEN
	// Access point as Eigen equivalent
	inline operator EMat () const { return CEMatMap((const TYPE*)val); }
	template <int N = n>
	inline operator typename std::enable_if<(N>1), Eigen::Matrix<TYPE,m,n> >::type () const { return CEMatMap((const TYPE*)val); }
	// Access point as Eigen::Map equivalent
	inline operator CEMatMap() const { return CEMatMap((const TYPE*)val); }
	inline operator EMatMap () { return EMatMap((TYPE*)val); }
	#endif

	// compute right null-space of this matrix ([n,n-m])
	inline TMatrix<TYPE,n,n-m> RightNullSpace(int flags = 0) const;
	// compute right/left null-vector of this matrix ([n/m,1])
	inline TMatrix<TYPE,n,1> RightNullVector(int flags = 0) const;
	inline TMatrix<TYPE,m,1> LeftNullVector(int flags = 0) const;

	// set byte value to all memory size of this matrix
	inline void memset(uint8_t v) { ::memset(val, v, sizeof(TYPE) * m * n); }
	// compute the memory size of this matrix (in bytes)
	inline size_t memory_size() const { return sizeof(TMatrix) + sizeof(TYPE) * m * n; }

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
	typedef Eigen::Map<const EMat> CEMatMap;
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
	inline int area() const { ASSERT(dims == 0 || dims == 2); return cols*rows; }
	/// Compute the memory size of this matrix (in bytes)
	inline size_t memory_size() const { return sizeof(TDMatrix) + cv::Mat::total() * cv::Mat::elemSize(); }

	/// Is this coordinate inside the 2D matrix?
	template <typename T>
	static inline typename std::enable_if<std::is_integral<T>::value,bool>::type isInside(const cv::Point_<T>& pt, const cv::Size& size) {
		return pt.x>=T(0) && pt.y>=T(0) && pt.x<T(size.width) && pt.y<T(size.height);
	}
	template <typename T>
	static inline typename std::enable_if<std::is_floating_point<T>::value,bool>::type isInside(const cv::Point_<T>& pt, const cv::Size& size) {
		return pt.x>=T(0) && pt.y>=T(0) && pt.x<=T(size.width) && pt.y<=T(size.height);
	}
	template <typename T>
	inline bool isInside(const cv::Point_<T>& pt) const {
		return isInside<T>(pt, Base::size());
	}

	/// Is this coordinate inside the 2D matrix, and not too close to the edges?
	/// @param border: the size of the border
	inline bool isInsideWithBorder(const Size& pt, int border) const {
		return pt.width>=border && pt.height>=border && pt.width<Base::size().width-border && pt.height<Base::size().height-border;
	}
	template <typename T>
	inline typename std::enable_if<std::is_integral<T>::value,bool>::type isInsideWithBorder(const cv::Point_<T>& pt, int border) const {
		return pt.x>=border && pt.y>=border && pt.x<Base::size().width-border && pt.y<Base::size().height-border;
	}
	template <typename T>
	inline typename std::enable_if<std::is_floating_point<T>::value,bool>::type isInsideWithBorder(const cv::Point_<T>& pt, int border) const {
		return pt.x>=T(border) && pt.y>=T(border) && pt.x<=T(Base::size().width-(border+1)) && pt.y<=T(Base::size().height-(border+1));
	}
	template <typename T, int border>
	inline typename std::enable_if<std::is_integral<T>::value,bool>::type isInsideWithBorder(const cv::Point_<T>& pt) const {
		return pt.x>=border && pt.y>=border && pt.x<Base::size().width-border && pt.y<Base::size().height-border;
	}
	template <typename T, int border>
	inline typename std::enable_if<std::is_floating_point<T>::value,bool>::type isInsideWithBorder(const cv::Point_<T>& pt) const {
		return pt.x>=T(border) && pt.y>=T(border) && pt.x<=T(Base::size().width-(border+1)) && pt.y<=T(Base::size().height-(border+1));
	}

	template <typename T, int border=0>
	static inline void clip(TPoint2<T>& ptMin, TPoint2<T>& ptMax, const cv::Size& size) {
		if (ptMin.x < T(border))
			ptMin.x = T(border);
		if (ptMin.y < T(border))
			ptMin.y = T(border);
		if (ptMax.x >= T(size.width-border))
			ptMax.x = T(size.width-(border+1));
		if (ptMax.y >= T(size.height-border))
			ptMax.y = T(size.height-(border+1));
	}
	template <typename T, int border=0>
	inline void clip(TPoint2<T>& ptMin, TPoint2<T>& ptMax) const {
		clip<T,border>(ptMin, ptMax, Base::size());
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
	inline const TYPE* getData() const { ASSERT(cv::Mat::empty() || cv::Mat::isContinuous()); return (const TYPE*)data; }
	inline TYPE* getData() { ASSERT(cv::Mat::empty() || cv::Mat::isContinuous()); return (TYPE*)data; }

	#ifdef _USE_EIGEN
	// Access point as Eigen equivalent
	inline operator EMat () const { return CEMatMap(getData(), rows, cols); }
	inline operator Eigen::Matrix<TYPE,Eigen::Dynamic,Eigen::Dynamic> () const { return CEMatMap(getData(), rows, cols); }
	// Access point as Eigen::Map equivalent
	inline operator const CEMatMap () const { return CEMatMap(getData(), rows, cols); }
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
typedef CLISTDEF2(DMatrix) DMatrixArr;
typedef CLISTDEF2(cv::Mat) MatArr;
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
typedef CLISTDEF2(DVector) DVectorArr;
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

template<typename TYPE> struct ColorType {
	typedef TYPE value_type;
	typedef value_type alt_type;
	typedef value_type work_type;
	static const value_type ONE;
	static const alt_type ALTONE;
};
// Static-const members of explicit template specializations are awkward to
// dllexport on MSVC; switching to `inline static constexpr` keeps the value
// in the header (no DLL crossing) and works uniformly across static + shared
// builds. Requires C++17 (already enabled project-wide).
template<> struct ColorType<uint8_t> {
	typedef uint8_t value_type;
	typedef float alt_type;
	typedef float work_type;
	inline static constexpr value_type ONE{255};
	inline static constexpr alt_type ALTONE{1.f};
};
template<> struct ColorType<uint32_t> {
	typedef uint32_t value_type;
	typedef float alt_type;
	typedef float work_type;
	inline static constexpr value_type ONE{255};
	inline static constexpr alt_type ALTONE{1.f};
};
template<> struct ColorType<float> {
	typedef float value_type;
	typedef uint8_t alt_type;
	typedef float work_type;
	inline static constexpr value_type ONE{1.f};
	inline static constexpr alt_type ALTONE{255};
};
template<> struct ColorType<double> {
	typedef double value_type;
	typedef uint8_t alt_type;
	typedef float work_type;
	inline static constexpr value_type ONE{1.0};
	inline static constexpr alt_type ALTONE{255};
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
	typedef typename ColorType<TYPE>::work_type WT;
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
	inline TPixel& set(TYPE _r, TYPE _g, TYPE _b) { r = _r; g = _g; b = _b; return *this; }
	inline TPixel& set(const TYPE* clr) { c[0] = clr[0]; c[1] = clr[1]; c[2] = clr[2]; return *this; }
	inline TPixel& set(TYPE _g) { r = _g; g = _g; b = _g; return *this; }
	inline void get(TYPE& _r, TYPE& _g, TYPE& _b) const { _r = r; _g = g; _b = b; }
	inline void get(TYPE* clr) const { clr[0] = c[0]; clr[1] = c[1]; clr[2] = c[2]; }
	// set/get from alternative type
	inline TPixel& set(ALT _r, ALT _g, ALT _b) { r = TYPE(_r); g = TYPE(_g); b = TYPE(_b); return *this; }
	inline TPixel& set(const ALT* clr) { c[0] = TYPE(clr[0]); c[1] = TYPE(clr[1]); c[2] = TYPE(clr[2]); return *this; }
	inline void get(ALT& _r, ALT& _g, ALT& _b) const { _r = ALT(r); _g = ALT(g); _b = ALT(b); }
	inline void get(ALT* clr) const { clr[0] = ALT(c[0]); clr[1] = ALT(c[1]); clr[2] = ALT(c[2]); }
	template<typename T> inline TPixel<typename std::enable_if<!std::is_floating_point<TYPE>::value || !std::is_same<T,uint8_t>::value,T>::type> cast() const { return TPixel<T>(T(r), T(g), T(b)); }
	template<typename T> inline TPixel<typename std::enable_if<std::is_floating_point<TYPE>::value && std::is_same<T,uint8_t>::value,T>::type> cast() const {
		return TPixel<uint8_t>(
			(uint8_t)CLAMP(ROUND2INT(r), 0, 255),
			(uint8_t)CLAMP(ROUND2INT(g), 0, 255),
			(uint8_t)CLAMP(ROUND2INT(b), 0, 255)
		);
	}
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
	// tools
	static TPixel colorRamp(WT v, WT vmin, WT vmax);
	static TPixel gray2color(WT v);
	static TPixel random();
	#ifdef _USE_BOOST
	// serialize
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & c;
	}
	#endif
};
template <> inline TPixel<float>& TPixel<float>::set(uint8_t _r, uint8_t _g, uint8_t _b) { r = float(_r)/255; g = float(_g)/255; b = float(_b)/255; return *this; }
template <> inline void TPixel<float>::get(uint8_t& _r, uint8_t& _g, uint8_t& _b) const { _r = (uint8_t)CLAMP(ROUND2INT(r*255), 0, 255); _g = (uint8_t)CLAMP(ROUND2INT(g*255), 0, 255); _b = (uint8_t)CLAMP(ROUND2INT(b*255), 0, 255); }
template <> inline TPixel<uint8_t>& TPixel<uint8_t>::set(float _r, float _g, float _b) { r = (uint8_t)CLAMP(ROUND2INT(_r*255), 0, 255); g = (uint8_t)CLAMP(ROUND2INT(_g*255), 0, 255); b = (uint8_t)CLAMP(ROUND2INT(_b*255), 0, 255); return *this; }
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
	inline TColor& set(TYPE _r, TYPE _g, TYPE _b, TYPE _a=ColorType<TYPE>::ONE) { r = _r; g = _g; b = _b; a = _a; return *this; }
	inline TColor& set(const TYPE* clr) { c[0] = clr[0]; c[1] = clr[1]; c[2] = clr[2]; c[3] = clr[3]; return *this; }
	inline TColor& set(TYPE _g) { r = _g; g = _g; b = _g; return *this; }
	inline void get(TYPE& _r, TYPE& _g, TYPE& _b, TYPE& _a) const { _r = r; _g = g; _b = b; _a = a; }
	inline void get(TYPE* clr) const { clr[0] = c[0]; clr[1] = c[1]; clr[2] = c[2]; clr[3] = c[3]; }
	// set/get from alternative type
	inline TColor& set(ALT _r, ALT _g, ALT _b, ALT _a=ColorType<TYPE>::ALTONE) { r = TYPE(_r); g = TYPE(_g); b = TYPE(_b); a = TYPE(_a); return *this; }
	inline TColor& set(const ALT* clr) { c[0] = TYPE(clr[0]); c[1] = TYPE(clr[1]); c[2] = TYPE(clr[2]); c[3] = TYPE(clr[3]); return *this; }
	inline void get(ALT& _r, ALT& _g, ALT& _b, ALT& _a) const { _r = ALT(r); _g = ALT(g); _b = ALT(b); _a = ALT(a); }
	inline void get(ALT* clr) const { clr[0] = ALT(c[0]); clr[1] = ALT(c[1]); clr[2] = ALT(c[2]); clr[3] = ALT(c[3]); }
	template<typename T> inline TColor<typename std::enable_if<!std::is_floating_point<TYPE>::value || !std::is_same<T,uint8_t>::value,T>::type> cast() const { return TColor<T>(T(r), T(g), T(b), T(a)); }
	template<typename T> inline TColor<typename std::enable_if<std::is_floating_point<TYPE>::value && std::is_same<T,uint8_t>::value,T>::type> cast() const {
		return TColor<uint8_t>(
			(uint8_t)CLAMP(ROUND2INT(r), 0, 255),
			(uint8_t)CLAMP(ROUND2INT(g), 0, 255),
			(uint8_t)CLAMP(ROUND2INT(b), 0, 255),
			(uint8_t)CLAMP(ROUND2INT(a), 0, 255)
		);
	}
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
template <> inline TColor<float>& TColor<float>::set(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a) { r = float(_r)/255; g = float(_g)/255; b = float(_b)/255; a = float(_a)/255; return *this; }
template <> inline void TColor<float>::get(uint8_t& _r, uint8_t& _g, uint8_t& _b, uint8_t& _a) const { _r = uint8_t(r*255); _g = uint8_t(g*255); _b = uint8_t(b*255); _a = uint8_t(a*255); }
template <> inline TColor<uint8_t>& TColor<uint8_t>::set(float _r, float _g, float _b, float _a) { r = uint8_t(_r*255); g = uint8_t(_g*255); b = uint8_t(_b*255); a = uint8_t(_a*255); return *this; }
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
	#ifdef _SUPPORT_CPP11
	inline TImage(cv::Mat&& rhs) : Base(std::forward<cv::Mat>(rhs)) {}

	inline TImage& operator = (cv::Mat&& rhs) { BaseBase::operator=(std::forward<cv::Mat>(rhs)); return *this; }
	#endif
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

	template <typename T, typename TV, typename Functor>
	bool sample(TV& v, const TPoint2<T>& pt, const Functor& functor) const;
	template <typename T, typename TV, typename Functor>
	bool sampleSafe(TV& v, const TPoint2<T>& pt, const Functor& functor) const;
	template <typename T, typename Functor>
	TYPE sample(const TPoint2<T>& pt, const Functor& functor, const TYPE& dv) const;
	template <typename T, typename Functor>
	TYPE sampleSafe(const TPoint2<T>& pt, const Functor& functor, const TYPE& dv) const;

	template <typename SAMPLER, typename INTERTYPE>
	INTERTYPE sample(const SAMPLER& sampler, const TPoint2<typename SAMPLER::Type>& pt) const;

	template <typename T>
	void toGray(TImage<T>& out, int code, bool bNormalize=false, bool bSRGB=false) const;

	static cv::Size computeResize(const cv::Size& size, REAL scale);
	static cv::Size computeResize(const cv::Size& size, REAL scale, unsigned resizes);
	unsigned computeMaxResolution(unsigned& level, unsigned minImageSize=320, unsigned maxImageSize=INT_MAX) const;
	static unsigned computeMaxResolution(unsigned width, unsigned height, unsigned& level, unsigned minImageSize=320, unsigned maxImageSize=INT_MAX);

	template <typename T, typename PARSER>
	static void RasterizeTriangle(const TPoint2<T>& v1, const TPoint2<T>& v2, const TPoint2<T>& v3, PARSER& parser);
	template <typename T, typename PARSER, bool CULL=true>
	static void RasterizeTriangleBary(const TPoint2<T>& v1, const TPoint2<T>& v2, const TPoint2<T>& v3, PARSER& parser);
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

	#ifndef _RELEASE
	void Show(const String& winname, int delay=0, bool bDestroy=true) const;
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
typedef TImage<uint8_t> Image8U;
typedef TImage<uint16_t> Image16U;
typedef TImage<hfloat> Image16F;
typedef TImage<float> Image32F;
typedef TImage<double> Image64F;
typedef TImage<Pixel8U> Image8U3;
typedef TImage<Color8U> Image8U4;
typedef TImage<Point2f> Image32F2;
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
		const int len(length());
		rows = _rows; cols = _cols;
		const int newLen = length();
		if (!empty() && len == newLen)
			return;
		if (rows <=0 || cols <= 0) {
			release();
			return;
		}
		delete[] data;
		data = new Type[newLen];
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
	inline void copyTo(cv::OutputArray _dst) const {
		_dst.create(rows, cols, CV_8U);
		cv::Mat dst(_dst.getMat());
		for (int i=0; i<rows; ++i)
			for (int j=0; j<cols; ++j)
				dst.at<uint8_t>(i,j) = (isSet(i,j) ? uint8_t(255) : uint8_t(0));
	}

	inline TBitMatrix& operator = (const TBitMatrix& rhs) {
		create(rhs.rows, rhs.cols);
		if (!empty())
			memcpy(data, rhs.data, sizeof(Type)*length());
		return *this;
	}
	inline TBitMatrix& operator = (cv::InputArray _rhs) {
		if (_rhs.dims() == 2 && _rhs.channels() == 1) {
			const cv::Mat rhs(_rhs.getMat());
			ASSERT(rhs.depth() == CV_8U);
			create(rhs.rows, rhs.cols);
			for (int i=0; i<rows; ++i)
				for (int j=0; j<cols; ++j)
					set(i,j, rhs.at<uint8_t>(i,j)!=0);
		} else {
			release();
			ASSERT("TBitMatrix: invalid cv::InputArray type!" == NULL);
		}
		return *this;
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
	unsigned count;

	inline TAccumulator();
	inline TAccumulator(const Type& v, const WeightType& w) : value(v*w), weight(w), count(1) {}
	inline bool IsEmpty() const { ASSERT((weight > 0 && count > 0) || (weight <= 0 && count == 0)); return weight <= 0; }
	// adds the given weighted value to the internal value
	inline void Add(const Type& v, const WeightType& w) {
		value += v*w;
		weight += w;
		++count;
	}
	inline TAccumulator& operator +=(const TAccumulator& accum) {
		value += accum.value;
		weight += accum.weight;
		count += accum.count;
		return *this;
	}
	inline TAccumulator operator +(const TAccumulator& accum) const {
		return TAccumulator(
			value + accum.value,
			weight + accum.weight,
			count + accum.count
		);
	}
	// subtracts the given weighted value to the internal value
	inline void Sub(const Type& v, const WeightType& w) {
		value -= v*w;
		weight -= w;
		--count;
	}
	inline TAccumulator& operator -=(const TAccumulator& accum) {
		value -= accum.value;
		weight -= accum.weight;
		count -= accum.count;
		return *this;
	}
	inline TAccumulator operator -(const TAccumulator& accum) const {
		return TAccumulator(
			value - accum.value,
			weight - accum.weight,
			count - accum.count
		);
	}
	// returns the normalized version of the internal value
	inline AccumType NormalizedFull() const {
		return value / weight;
	}
	inline Type Normalized() const {
		return Type(NormalizedFull());
	}
	inline WeightType NormalizedWeight() const {
		return weight / count;
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
typedef CLISTDEF0(IndexScore) IndexScoreArr;
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
typedef CLISTDEF0(PairIdx) PairIdxArr;
inline PairIdx MakePairIdx(uint32_t idxImageA, uint32_t idxImageB) {
	ASSERT(idxImageA != idxImageB);
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
GENERAL_API String cvMat2String(const cv::Mat&, LPCSTR format="% 10.4f ");
template<typename TYPE, int m, int n> inline String cvMat2String(const TMatrix<TYPE,m,n>& mat, LPCSTR format="% 10.4f ") { return cvMat2String(cv::Mat(mat), format); }
template<typename TYPE> inline String cvMat2String(const TPoint3<TYPE>& pt, LPCSTR format="% 10.4f ") { return cvMat2String(cv::Mat(pt), format); }
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#include "../Math/LMFit/lmmin.h"
#include "Types.inl"
#include "Util.inl"
#include "Rotation.h"
#include "Sphere.h"
#include "AABB.h"
#include "OBB.h"
#include "Plane.h"
#include "Ray.h"
#include "Line.h"
#include "Octree.h"
#include "OctreeLOD.h"
#include "UtilCUDA.h"

#endif // __SEACAVE_TYPES_H__
