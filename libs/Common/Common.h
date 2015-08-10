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
#define INIT_WORKING_FOLDER	{SEACAVE::Util::ensureValidDirectoryPath(WORKING_FOLDER); WORKING_FOLDER_FULL = SEACAVE::Util::getFullPath(WORKING_FOLDER);} // initialize working folders
#define MAKE_PATH(str)		(WORKING_FOLDER+(str)) // add working directory to the given file name
#define MAKE_PATH_SAFE(str)	(SEACAVE::Util::isFullPath((str).c_str()) ? String(str) : String(WORKING_FOLDER+(str))) // add working directory to the given file name only if not full path already
#define MAKE_PATH_FULL(p,s) (SEACAVE::Util::isFullPath((s).c_str()) ? String(s) : String((p)+(s))) // add the given path to the given file name
#define MAKE_PATH_REL(p,s)	((s).compare(0,(p).size(),p) ? String(s) : String(SEACAVE::String((s).substr((p).size())))) // remove the given path from the given file name
#define GET_PATH_FULL(str)	(SEACAVE::Util::isFullPath((str).c_str()) ? SEACAVE::Util::getFilePath(str) : String(WORKING_FOLDER_FULL+SEACAVE::Util::getFilePath(str)) // retrieve the full path to the given file


// I N C L U D E S /////////////////////////////////////////////////

#include "Types.h"


// P R O T O T Y P E S /////////////////////////////////////////////

namespace SEACAVE {

typedef TAABB<float, 2> AABB2f;
typedef TAABB<float, 3> AABB3f;
typedef TOBB<float, 2> OBB2f;
typedef TOBB<float, 3> OBB3f;
typedef TRay<float, 2> Ray2f;
typedef TRay<float, 3> Ray3f;
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

typedef TAABB<double, 2> AABB2d;
typedef TAABB<double, 3> AABB3d;
typedef TOBB<double, 2> OBB2d;
typedef TOBB<double, 3> OBB3d;
typedef TRay<double, 2> Ray2d;
typedef TRay<double, 3> Ray3d;
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

typedef TAABB<REAL, 2> AABB2;
typedef TAABB<REAL, 3> AABB3;
typedef TOBB<REAL, 2> OBB2;
typedef TOBB<REAL, 3> OBB3;
typedef TRay<REAL, 2> Ray2;
typedef TRay<REAL, 3> Ray3;
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

DEFINE_CVDATATYPE(SEACAVE::cuint32_t)
/*----------------------------------------------------------------*/

#endif // _COMMON_COMMON_H_
