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
#endif
#if TD_VERBOSE == TD_VERBOSE_ON
#ifndef VERBOSITY_LEVEL
#define VERBOSITY_LEVEL 2
#endif
#define VERBOSE LOG
#define DEBUG_LEVEL(n,...)	{ if (VERBOSITY_LEVEL > n) VERBOSE(__VA_ARGS__); }
#endif
#if TD_VERBOSE == TD_VERBOSE_DEBUG
#ifndef VERBOSITY_LEVEL
#define VERBOSITY_LEVEL 3
#endif
#define VERBOSE LOG
#define DEBUG_LEVEL(n,...)	{ if (VERBOSITY_LEVEL > n) VERBOSE(__VA_ARGS__); }
#endif
#define DEBUG(...)			DEBUG_LEVEL(0, __VA_ARGS__)
#define DEBUG_EXTRA(...)	DEBUG_LEVEL(1, __VA_ARGS__)
#define DEBUG_ULTIMATE(...)	DEBUG_LEVEL(2, __VA_ARGS__)

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

#define LOG_OUT() GET_LOG() //or std::cout
#define LOG_ERR() GET_LOG() //or std::cerr


// I N C L U D E S /////////////////////////////////////////////////

#include "Types.h"


// P R O T O T Y P E S /////////////////////////////////////////////

namespace SEACAVE {

typedef TPoint2<float> Point2f;
typedef TPoint3<float> Point3f;
typedef TMatrix<float,2,1> Vec2f;
typedef TMatrix<float,3,1> Vec3f;
typedef TMatrix<float,4,1> Vec4f;
typedef TMatrix<float,2,2> Matrix2x2f;
typedef TMatrix<float,3,3> Matrix3x3f;
typedef TMatrix<float,3,4> Matrix3x4f;
typedef TMatrix<float,4,4> Matrix4x4f;

typedef TPoint2<double> Point2d;
typedef TPoint3<double> Point3d;
typedef TMatrix<double,2,1> Vec2d;
typedef TMatrix<double,3,1> Vec3d;
typedef TMatrix<double,4,1> Vec4d;
typedef TMatrix<double,2,2> Matrix2x2d;
typedef TMatrix<double,3,3> Matrix3x3d;
typedef TMatrix<double,3,4> Matrix3x4d;
typedef TMatrix<double,4,4> Matrix4x4d;

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

typedef SEACAVE::CLISTDEF0(REAL) REALArr;
typedef SEACAVE::CLISTDEF0(Point2f) Point2fArr;
typedef SEACAVE::CLISTDEF0(Point3f) Point3fArr;
typedef SEACAVE::CLISTDEF0(Point2d) Point2dArr;
typedef SEACAVE::CLISTDEF0(Point3d) Point3dArr;
typedef SEACAVE::CLISTDEF0(Point2) Point2Arr;
typedef SEACAVE::CLISTDEF0(Point3) Point3Arr;
typedef SEACAVE::CLISTDEF0(Vec4) Vec4Arr;
typedef SEACAVE::CLISTDEF0(Matrix3x3) Matrix3x3Arr;
typedef SEACAVE::CLISTDEF0(Matrix3x4) Matrix3x4Arr;
typedef SEACAVE::CLISTDEF0(CMatrix) CMatrixArr;
typedef SEACAVE::CLISTDEF0(RMatrix) RMatrixArr;
typedef SEACAVE::CLISTDEF0(KMatrix) KMatrixArr;
typedef SEACAVE::CLISTDEF0(PMatrix) PMatrixArr;
typedef SEACAVE::CLISTDEF0(ImageRef) ImageRefArr;
typedef SEACAVE::CLISTDEF0(Pixel8U) Pixel8UArr;
typedef SEACAVE::CLISTDEF0(Pixel32F) Pixel32FArr;
typedef SEACAVE::CLISTDEF0(Color8U) Color8UArr;
typedef SEACAVE::CLISTDEF0(Color32F) Color32FArr;
/*----------------------------------------------------------------*/

// used for sorting some indices by their score (decreasing)
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
	void serialize(Archive& ar, const unsigned int version) {
		ar & idx;
		ar & score;
	}
	#endif
};
typedef TIndexScore<uint32_t,float> IndexScore;
typedef cList<IndexScore, const IndexScore&, 0> IndexScoreArr;
/*----------------------------------------------------------------*/

struct PairIdx {
	union {
		uint64_t idx;
		struct {
			#if __BYTE_ORDER == __LITTLE_ENDIAN
			uint32_t j;
			uint32_t i;
			#else
			uint32_t i;
			uint32_t j;
			#endif
		};
	};
	inline PairIdx() {}
	inline PairIdx(uint64_t _idx) : idx(_idx) {}
	inline PairIdx(uint32_t _i, uint32_t _j) : i(_i), j(_j) {}
	// get index
	inline operator uint64_t () const { return idx; }
	// compare by index (increasing)
	inline bool operator<(const PairIdx& r) const { return (idx < r.idx); }
	inline bool operator==(const PairIdx& r) const { return (idx == r.idx); }
};
typedef cList<PairIdx, const PairIdx&, 0> PairIdxArr;
inline PairIdx MakePairIdx(uint32_t idxImageA, uint32_t idxImageB) {
	return (idxImageA<idxImageB ? PairIdx(idxImageA,idxImageB) : PairIdx(idxImageB,idxImageA));
}
/*----------------------------------------------------------------*/

} // SEACAVE

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

typedef SEACAVE::cList<cv::Mat, const cv::Mat&, 2> MatArr;
typedef SEACAVE::cList<cv::Matx34f> CameraMatArr;
typedef SEACAVE::cList<uint32_t, uint32_t, 0> IndexArr;
/*----------------------------------------------------------------*/

#endif //_COMMON_COMMON_H_
