////////////////////////////////////////////////////////////////////
// Common.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _COMMON_COMMON_H_
#define _COMMON_COMMON_H_


// D E F I N E S ///////////////////////////////////////////////////

#include "Config.h"

// macors controling the verbosity
#define TD_VERBOSE_OFF		0
#define TD_VERBOSE_ON		1
#define TD_VERBOSE_DEBUG	2
#ifndef TD_VERBOSE
#ifdef _RELEASE
#define TD_VERBOSE			TD_VERBOSE_ON
#else
#define TD_VERBOSE			TD_VERBOSE_DEBUG
#endif
#endif

#if TD_VERBOSE == TD_VERBOSE_OFF
#define VERBOSE LOG
#define DEBUG_LEVEL(n,...)
#else
#ifndef VERBOSITY_LEVEL
namespace SEACAVE { extern int g_nVerbosityLevel; }
#define VERBOSITY_LEVEL g_nVerbosityLevel
#endif
#define VERBOSE LOG
#define DEBUG_LEVEL(n,...)	{ if (n < VERBOSITY_LEVEL) VERBOSE(__VA_ARGS__); }
#endif
#define DEBUG(...)			DEBUG_LEVEL(0, __VA_ARGS__)
#define DEBUG_EXTRA(...)	DEBUG_LEVEL(1, __VA_ARGS__)
#define DEBUG_ULTIMATE(...)	DEBUG_LEVEL(2, __VA_ARGS__)


// macros that simplify timing tasks
#define TD_TIMER_OFF		0
#define TD_TIMER_ON			1
#ifndef TD_TIMER
#define TD_TIMER			TD_TIMER_ON
#endif

#if TD_TIMER == TD_TIMER_OFF
#define TD_TIMER_START()
#define TD_TIMER_UPDATE()
#define TD_TIMER_GET() 0
#define TD_TIMER_GET_INT() 0
#define TD_TIMER_GET_FMT() String()
#define TD_TIMER_STARTD()
#define TD_TIMER_UPDATED()
#endif
#if TD_TIMER == TD_TIMER_ON
#define TD_TIMER_START()	TIMER_START()
#define TD_TIMER_UPDATE()	TIMER_UPDATE()
#define TD_TIMER_GET()		TIMER_GET()
#define TD_TIMER_GET_INT()	TIMER_GET_INT()
#define TD_TIMER_GET_FMT()	TIMER_GET_FORMAT()
#if TD_VERBOSE == TD_VERBOSE_OFF
#define TD_TIMER_STARTD()
#define TD_TIMER_UPDATED()
#else
#define TD_TIMER_STARTD()	TIMER_START()
#define TD_TIMER_UPDATED()	TIMER_UPDATE()
#endif
#endif


// macros redirecting standard streams to the log
#define LOG_OUT() GET_LOG() //or std::cout
#define LOG_ERR() GET_LOG() //or std::cerr


// macros simplifying the task of composing file paths;
// WORKING_FOLDER and WORKING_FOLDER_FULL must be defined as strings
// containing the relative/full path to the working folder
#ifndef WORKING_FOLDER
namespace SEACAVE {
class String;
extern String g_strWorkingFolder; // empty by default (current folder)
extern String g_strWorkingFolderFull; // full path to current folder
}
#define WORKING_FOLDER		g_strWorkingFolder // empty by default (current folder)
#define WORKING_FOLDER_FULL	g_strWorkingFolderFull // full path to current folder
#endif
#define INIT_WORKING_FOLDER	{SEACAVE::Util::ensureValidFolderPath(WORKING_FOLDER); WORKING_FOLDER_FULL = SEACAVE::Util::getFullPath(WORKING_FOLDER);} // initialize working folders
#define MAKE_PATH(str)		SEACAVE::Util::getSimplifiedPath(WORKING_FOLDER+(str)) // add working directory to the given file name
#define MAKE_PATH_SAFE(str)	(SEACAVE::Util::isFullPath(str) ? SEACAVE::String(str) : MAKE_PATH(str)) // add working directory to the given file name only if not full path already
#define MAKE_PATH_FULL(p,s) (SEACAVE::Util::isFullPath((s).c_str()) ? SEACAVE::String(s) : SEACAVE::Util::getSimplifiedPath((p)+(s))) // add the given path to the given file name
#define MAKE_PATH_REL(p,s)	((s).compare(0,(p).size(),p) ? SEACAVE::String(s) : SEACAVE::String(SEACAVE::String((s).substr((p).size())))) // remove the given path from the given file name
#define GET_PATH_FULL(str)	(SEACAVE::Util::isFullPath((str).c_str()) ? SEACAVE::Util::getFilePath(str) : SEACAVE::Util::getSimplifiedPath(WORKING_FOLDER_FULL+SEACAVE::Util::getFilePath(str))) // retrieve the full path to the given file


// macros simplifying the task of managing options
#define DECOPT_SPACE(SPACE) namespace SPACE { \
	void init(); \
	void update(); \
	extern SEACAVE::VoidArr arrFncOpt; \
	extern SEACAVE::CConfigTable oConfig; \
}
#define DEFOPT_SPACE(SPACE, name) namespace SPACE { \
	SEACAVE::CConfigTable oConfig(name); \
	typedef LPCTSTR (*FNCINDEX)(); \
	typedef void (*FNCINIT)(SEACAVE::IDX); \
	typedef void (*FNCUPDATE)(); \
	VoidArr arrFncOpt; \
	void init() { \
		FOREACH(i, arrFncOpt) \
			((FNCINIT)arrFncOpt[i])(i); \
	} \
	void update() { \
		FOREACH(i, arrFncOpt) \
			((FNCUPDATE)arrFncOpt[i])(); \
	} \
}

#define DEFVAR_OPTION(SPACE, flags, type, name, title, desc, ...) namespace SPACE { \
	type name; \
	LPCTSTR defval_##name(NULL); \
	void update_##name() { \
		SEACAVE::String::FromString(oConfig[title].val, name); \
	} \
	void init_##name(SEACAVE::IDX idx) { \
		LPCTSTR const vals[] = {__VA_ARGS__}; \
		arrFncOpt[idx] = (void*)update_##name; \
		SMLVALUE& val = oConfig[title]; \
		CFGITEM& opt = *((CFGITEM*)val.data); \
		opt.state = flags; \
		val.val = opt.defval = (defval_##name != NULL ? defval_##name : vals[0]); \
		opt.vals.Insert(opt.defval); \
		for (size_t i=1; i<sizeof(vals)/sizeof(LPCTSTR); ++i) \
			opt.vals.Insert(vals[i]); \
		SEACAVE::String::FromString(opt.defval, name); \
	} \
	LPCTSTR index_##name() { \
		arrFncOpt.Insert((void*)init_##name); \
		return title; \
	} \
	LPCTSTR const name_##name(index_##name()); \
}

#define FDEFVAR_string(SPACE, flags, name, title, desc, ...)  DEFVAR_OPTION(SPACE, flags, String, name, title, desc, __VA_ARGS__)
#define FDEFVAR_bool(SPACE, flags, name, title, desc, ...)    DEFVAR_OPTION(SPACE, flags, bool, name, title, desc, __VA_ARGS__)
#define FDEFVAR_int32(SPACE, flags, name, title, desc, ...)   DEFVAR_OPTION(SPACE, flags, int32_t, name, title, desc, __VA_ARGS__)
#define FDEFVAR_uint32(SPACE, flags, name, title, desc, ...)  DEFVAR_OPTION(SPACE, flags, uint32_t, name, title, desc, __VA_ARGS__)
#define FDEFVAR_flags(SPACE, flags, name, title, desc, ...)   DEFVAR_OPTION(SPACE, flags, Flags, name, title, desc, __VA_ARGS__)
#define FDEFVAR_float(SPACE, flags, name, title, desc, ...)   DEFVAR_OPTION(SPACE, flags, float, name, title, desc, __VA_ARGS__)
#define FDEFVAR_double(SPACE, flags, name, title, desc, ...)  DEFVAR_OPTION(SPACE, flags, double, name, title, desc, __VA_ARGS__)

#define DEFVAR_string(SPACE, name, title, desc, ...)  DEFVAR_OPTION(SPACE, CFGITEM::NA, String, name, title, desc, __VA_ARGS__)
#define DEFVAR_bool(SPACE, name, title, desc, ...)    DEFVAR_OPTION(SPACE, CFGITEM::NA, bool, name, title, desc, __VA_ARGS__)
#define DEFVAR_int32(SPACE, name, title, desc, ...)   DEFVAR_OPTION(SPACE, CFGITEM::NA, int32_t, name, title, desc, __VA_ARGS__)
#define DEFVAR_uint32(SPACE, name, title, desc, ...)  DEFVAR_OPTION(SPACE, CFGITEM::NA, uint32_t, name, title, desc, __VA_ARGS__)
#define DEFVAR_flags(SPACE, name, title, desc, ...)   DEFVAR_OPTION(SPACE, CFGITEM::NA, Flags, name, title, desc, __VA_ARGS__)
#define DEFVAR_float(SPACE, name, title, desc, ...)   DEFVAR_OPTION(SPACE, CFGITEM::NA, float, name, title, desc, __VA_ARGS__)
#define DEFVAR_double(SPACE, name, title, desc, ...)  DEFVAR_OPTION(SPACE, CFGITEM::NA, double, name, title, desc, __VA_ARGS__)

#define TDEFVAR_string(SPACE, name, title, desc, ...) DEFVAR_OPTION(SPACE, CFGITEM::TEMP, String, name, title, desc, __VA_ARGS__)
#define TDEFVAR_bool(SPACE, name, title, desc, ...)   DEFVAR_OPTION(SPACE, CFGITEM::TEMP, bool, name, title, desc, __VA_ARGS__)
#define TDEFVAR_int32(SPACE, name, title, desc, ...)  DEFVAR_OPTION(SPACE, CFGITEM::TEMP, int32_t, name, title, desc, __VA_ARGS__)
#define TDEFVAR_uint32(SPACE, name, title, desc, ...) DEFVAR_OPTION(SPACE, CFGITEM::TEMP, uint32_t, name, title, desc, __VA_ARGS__)
#define TDEFVAR_flags(SPACE, name, title, desc, ...)  DEFVAR_OPTION(SPACE, CFGITEM::TEMP, Flags, name, title, desc, __VA_ARGS__)
#define TDEFVAR_float(SPACE, name, title, desc, ...)  DEFVAR_OPTION(SPACE, CFGITEM::TEMP, float, name, title, desc, __VA_ARGS__)
#define TDEFVAR_double(SPACE, name, title, desc, ...) DEFVAR_OPTION(SPACE, CFGITEM::TEMP, double, name, title, desc, __VA_ARGS__)


// I N C L U D E S /////////////////////////////////////////////////

#include "Types.h"


// P R O T O T Y P E S /////////////////////////////////////////////

namespace SEACAVE {

typedef TSphere<float, 2> Sphere2f;
typedef TSphere<float, 3> Sphere3f;
typedef TAABB<float, 2> AABB2f;
typedef TAABB<float, 3> AABB3f;
typedef TOBB<float, 2> OBB2f;
typedef TOBB<float, 3> OBB3f;
typedef TRay<float, 2> Ray2f;
typedef TRay<float, 3> Ray3f;
typedef TTriangle<float, 2> Triangle2f;
typedef TTriangle<float, 3> Triangle3f;
typedef TPlane<float> Planef;
typedef TPoint2<float> Point2f;
typedef TPoint3<float> Point3f;
typedef TMatrix<float,2,1> Vec2f;
typedef TMatrix<float,3,1> Vec3f;
typedef TMatrix<float,4,1> Vec4f;
typedef TMatrix<float,2,2> Matrix2x2f;
typedef TMatrix<float,3,3> Matrix3x3f;
typedef TMatrix<float,3,4> Matrix3x4f;
typedef TMatrix<float,4,4> Matrix4x4f;

typedef TSphere<double, 2> Sphere2d;
typedef TSphere<double, 3> Sphere3d;
typedef TAABB<double, 2> AABB2d;
typedef TAABB<double, 3> AABB3d;
typedef TOBB<double, 2> OBB2d;
typedef TOBB<double, 3> OBB3d;
typedef TRay<double, 2> Ray2d;
typedef TRay<double, 3> Ray3d;
typedef TTriangle<double, 2> Triangle2d;
typedef TTriangle<double, 3> Triangle3d;
typedef TPlane<double> Planed;
typedef TPoint2<double> Point2d;
typedef TPoint3<double> Point3d;
typedef TMatrix<double,2,1> Vec2d;
typedef TMatrix<double,3,1> Vec3d;
typedef TMatrix<double,4,1> Vec4d;
typedef TMatrix<double,2,2> Matrix2x2d;
typedef TMatrix<double,3,3> Matrix3x3d;
typedef TMatrix<double,3,4> Matrix3x4d;
typedef TMatrix<double,4,4> Matrix4x4d;

typedef TSphere<REAL, 2> Sphere2;
typedef TSphere<REAL, 3> Sphere3;
typedef TAABB<REAL, 2> AABB2;
typedef TAABB<REAL, 3> AABB3;
typedef TOBB<REAL, 2> OBB2;
typedef TOBB<REAL, 3> OBB3;
typedef TRay<REAL, 2> Ray2;
typedef TRay<REAL, 3> Ray3;
typedef TTriangle<REAL, 2> Triangle2;
typedef TTriangle<REAL, 3> Triangle3;
typedef TPlane<REAL> Plane;
typedef TPoint2<REAL> Point2;
typedef TPoint3<REAL> Point3;
typedef TMatrix<REAL,2,1> Vec2;
typedef TMatrix<REAL,3,1> Vec3;
typedef TMatrix<REAL,4,1> Vec4;
typedef TMatrix<REAL,2,2> Matrix2x2;
typedef TMatrix<REAL,3,3> Matrix3x3;
typedef TMatrix<REAL,3,4> Matrix3x4;
typedef TMatrix<REAL,4,4> Matrix4x4;

typedef TQuaternion<REAL> Quaternion;
typedef TRMatrixBase<REAL> RMatrixBase;

// camera matrix types
typedef Point3      CMatrix;
typedef RMatrixBase RMatrix;
typedef Matrix3x3   KMatrix;
typedef Matrix3x4   PMatrix;

// reconstructed 3D point type
typedef Vec3 X3D;
typedef SEACAVE::cList<X3D, const X3D&, 0, 8192> X3DArr;

typedef SEACAVE::cList<uint32_t, uint32_t, 0> IndexArr;

typedef CLISTDEF0(REAL) REALArr;
typedef CLISTDEF0(Point2f) Point2fArr;
typedef CLISTDEF0(Point3f) Point3fArr;
typedef CLISTDEF0(Point2d) Point2dArr;
typedef CLISTDEF0(Point3d) Point3dArr;
typedef CLISTDEF0(Point2) Point2Arr;
typedef CLISTDEF0(Point3) Point3Arr;
typedef CLISTDEF0(Vec4) Vec4Arr;
typedef CLISTDEF0(Matrix3x3) Matrix3x3Arr;
typedef CLISTDEF0(Matrix3x4) Matrix3x4Arr;
typedef CLISTDEF0(CMatrix) CMatrixArr;
typedef CLISTDEF0(RMatrix) RMatrixArr;
typedef CLISTDEF0(KMatrix) KMatrixArr;
typedef CLISTDEF0(PMatrix) PMatrixArr;
typedef CLISTDEF0(ImageRef) ImageRefArr;
typedef CLISTDEF0(Pixel8U) Pixel8UArr;
typedef CLISTDEF0(Pixel32F) Pixel32FArr;
typedef CLISTDEF0(Color8U) Color8UArr;
typedef CLISTDEF0(Color32F) Color32FArr;
/*----------------------------------------------------------------*/

} // namespace SEACAVE

// define specialized cv:DataType<>
DEFINE_CVDATATYPE(SEACAVE::Vec2f)
DEFINE_CVDATATYPE(SEACAVE::Vec3f)
DEFINE_CVDATATYPE(SEACAVE::Vec4f)
DEFINE_CVDATATYPE(SEACAVE::Matrix2x2f)
DEFINE_CVDATATYPE(SEACAVE::Matrix3x3f)
DEFINE_CVDATATYPE(SEACAVE::Matrix3x4f)
DEFINE_CVDATATYPE(SEACAVE::Matrix4x4f)

DEFINE_CVDATATYPE(SEACAVE::Vec2d)
DEFINE_CVDATATYPE(SEACAVE::Vec3d)
DEFINE_CVDATATYPE(SEACAVE::Vec4d)
DEFINE_CVDATATYPE(SEACAVE::Matrix2x2d)
DEFINE_CVDATATYPE(SEACAVE::Matrix3x3d)
DEFINE_CVDATATYPE(SEACAVE::Matrix3x4d)
DEFINE_CVDATATYPE(SEACAVE::Matrix4x4d)
/*----------------------------------------------------------------*/

#endif // _COMMON_COMMON_H_
