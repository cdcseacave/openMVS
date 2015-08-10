////////////////////////////////////////////////////////////////////
// Types.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////


// G L O B A L S ///////////////////////////////////////////////////

template <typename TYPE> const typename ColorType<TYPE>::value_type ColorType<TYPE>::ONE(1);
template <typename TYPE> const typename ColorType<TYPE>::alt_type ColorType<TYPE>::ALTONE(1);

template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::BLACK		(0.f, 0.f, 0.f);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::WHITE		(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::GRAY		(0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::RED		(ColorType<TYPE>::ONE, 0.f, 0.f);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::GREEN		(0.f, ColorType<TYPE>::ONE, 0.f);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::BLUE		(0.f, 0.f, ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::YELLOW	(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, 0.f);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::MAGENTA	(ColorType<TYPE>::ONE, 0.f, ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::CYAN		(0.f, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);

template <typename TYPE> const TColor<TYPE> TColor<TYPE>::BLACK		(0.f, 0.f, 0.f, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::WHITE		(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::GRAY		(0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::RED		(ColorType<TYPE>::ONE, 0.f, 0.f, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::GREEN		(0.f, ColorType<TYPE>::ONE, 0.f, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::BLUE		(0.f, 0.f, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::YELLOW	(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, 0.f, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::MAGENTA	(ColorType<TYPE>::ONE, 0.f, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::CYAN		(0.f, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);


// C L A S S  //////////////////////////////////////////////////////

// square
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> SQUARE(const SEACAVE::TPoint2<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(SQUARE(v.x), SQUARE(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> SQUARE(const SEACAVE::TPoint3<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(SQUARE(v.x), SQUARE(v.y), SQUARE(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> SQUARE(const SEACAVE::TMatrix<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = SQUARE(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> SQUARE(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(SQUARE(v.x), SQUARE(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> SQUARE(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(SQUARE(v.x), SQUARE(v.y), SQUARE(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> SQUARE(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = SQUARE(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPixel<TYPE> SQUARE(const SEACAVE::TPixel<TYPE>& v)
{
	return SEACAVE::TPixel<TYPE>(SQUARE(v.r), SQUARE(v.g), SQUARE(v.b));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TColor<TYPE> SQUARE(const SEACAVE::TColor<TYPE>& v)
{
	return SEACAVE::TColor<TYPE>(SQUARE(v.r), SQUARE(v.g), SQUARE(v.b), SQUARE(v.a));
}

// sqrt
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> SQRT(const SEACAVE::TPoint2<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(SQRT(v.x), SQRT(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> SQRT(const SEACAVE::TPoint3<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(SQRT(v.x), SQRT(v.y), SQRT(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> SQRT(const SEACAVE::TMatrix<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = SQRT(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> SQRT(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(SQRT(v.x), SQRT(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> SQRT(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(SQRT(v.x), SQRT(v.y), SQRT(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> SQRT(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = SQRT(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPixel<TYPE> SQRT(const SEACAVE::TPixel<TYPE>& v)
{
	return SEACAVE::TPixel<TYPE>(SQRT(v.r), SQRT(v.g), SQRT(v.b));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TColor<TYPE> SQRT(const SEACAVE::TColor<TYPE>& v)
{
	return SEACAVE::TColor<TYPE>(SQRT(v.r), SQRT(v.g), SQRT(v.b), SQRT(v.a));
}

// exp
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> EXP(const SEACAVE::TPoint2<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(EXP(v.x), EXP(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> EXP(const SEACAVE::TPoint3<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(EXP(v.x), EXP(v.y), EXP(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> EXP(const SEACAVE::TMatrix<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = EXP(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> EXP(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(EXP(v.x), EXP(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> EXP(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(EXP(v.x), EXP(v.y), EXP(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> EXP(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = EXP(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPixel<TYPE> EXP(const SEACAVE::TPixel<TYPE>& v)
{
	return SEACAVE::TPixel<TYPE>(EXP(v.r), EXP(v.g), EXP(v.b));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TColor<TYPE> EXP(const SEACAVE::TColor<TYPE>& v)
{
	return SEACAVE::TColor<TYPE>(EXP(v.r), EXP(v.g), EXP(v.b), EXP(v.a));
}

// loge
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> LOGN(const SEACAVE::TPoint2<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(LOGN(v.x), LOGN(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> LOGN(const SEACAVE::TPoint3<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(LOGN(v.x), LOGN(v.y), LOGN(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> LOGN(const SEACAVE::TMatrix<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = LOGN(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> LOGN(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(LOGN(v.x), LOGN(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> LOGN(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(LOGN(v.x), LOGN(v.y), LOGN(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> LOGN(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = LOGN(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPixel<TYPE> LOGN(const SEACAVE::TPixel<TYPE>& v)
{
	return SEACAVE::TPixel<TYPE>(LOGN(v.r), LOGN(v.g), LOGN(v.b));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TColor<TYPE> LOGN(const SEACAVE::TColor<TYPE>& v)
{
	return SEACAVE::TColor<TYPE>(LOGN(v.r), LOGN(v.g), LOGN(v.b), LOGN(v.a));
}

// abs
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> ABS(const SEACAVE::TPoint2<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(ABS(v.x), ABS(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> ABS(const SEACAVE::TPoint3<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(ABS(v.x), ABS(v.y), ABS(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> ABS(const SEACAVE::TMatrix<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = ABS(v.val[i]);
	return nv;
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<TYPE> ABS(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<TYPE>(ABS(v.x), ABS(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<TYPE> ABS(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<TYPE>(ABS(v.x), ABS(v.y), ABS(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> ABS(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv(i) = ABS(v.val[i]);
	return nv;
}

// ISZERO
template <typename TYPE>
FORCEINLINE bool ISZERO(const cv::Point_<TYPE>& v)
{
	return (ISZERO(v.x) && ISZERO(v.y));
}
template <typename TYPE>
FORCEINLINE bool ISZERO(const cv::Point3_<TYPE>& v)
{
	return (ISZERO(v.x) && ISZERO(v.y) && ISZERO(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE bool ISZERO(const cv::Matx<TYPE,m,n>& v)
{
	for (register int i=0; i<m*n; ++i)
		if (!ISZERO(v.val[i]))
			return false;
	return true;
}

// ISEQUAL
template <typename TYPE>
FORCEINLINE bool ISEQUAL(const cv::Point_<TYPE>& v1, const cv::Point_<TYPE>& v2)
{
	return (ISEQUAL(v1.x,v2.x) && ISEQUAL(v1.y,v2.y));
}
template <typename TYPE>
FORCEINLINE bool ISEQUAL(const cv::Point3_<TYPE>& v1, const cv::Point3_<TYPE>& v2)
{
	return (ISEQUAL(v1.x,v2.x) && ISEQUAL(v1.y,v2.y) && ISEQUAL(v1.z,v2.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE bool ISEQUAL(const cv::Matx<TYPE,m,n>& v1, const cv::Matx<TYPE,m,n>& v2)
{
	for (register int i=0; i<m*n; ++i)
		if (!ISEQUAL(v1.val[i], v2.val[i]))
			return false;
	return true;
}

// round
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<int32_t> Floor2Int(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<int32_t>(FLOOR2INT(v.x), FLOOR2INT(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<int32_t> Ceil2Int(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<int32_t>(CEIL2INT(v.x), CEIL2INT(v.y));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint2<int32_t> Round2Int(const cv::Point_<TYPE>& v)
{
	return SEACAVE::TPoint2<int32_t>(ROUND2INT(v.x), ROUND2INT(v.y));
}

template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<int32_t> Floor2Int(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<int32_t>(FLOOR2INT(v.x), FLOOR2INT(v.y), FLOOR2INT(v.z));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<int32_t> Ceil2Int(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<int32_t>(CEIL2INT(v.x), CEIL2INT(v.y), CEIL2INT(v.z));
}
template <typename TYPE>
FORCEINLINE SEACAVE::TPoint3<int32_t> Round2Int(const cv::Point3_<TYPE>& v)
{
	return SEACAVE::TPoint3<int32_t>(ROUND2INT(v.x), ROUND2INT(v.y), ROUND2INT(v.z));
}

template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> Floor2Int(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv.val[i] = Floor2Int(v.val[i]);
	return nv;
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> Ceil2Int(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv.val[i] = Ceil2Int(v.val[i]);
	return nv;
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> Round2Int(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (register int i=0; i<m*n; ++i)
		nv.val[i] = Round2Int(v.val[i]);
	return nv;
}

// ISFINITE
template <typename TYPE>
FORCEINLINE bool ISFINITE(const SEACAVE::TPoint2<TYPE>& v)
{
	return (ISFINITE(v.x) && ISFINITE(v.y));
}
template <typename TYPE>
FORCEINLINE bool ISFINITE(const SEACAVE::TPoint3<TYPE>& v)
{
	return (ISFINITE(v.x) && ISFINITE(v.y) && ISFINITE(v.z));
}
template <typename TYPE>
FORCEINLINE bool ISFINITE(const SEACAVE::TDMatrix<TYPE>& v)
{
	ASSERT(v.isContinuous());
	return ISFINITE(v.val, v.area());
}
template <typename TYPE>
FORCEINLINE bool ISFINITE(const cv::Point_<TYPE>& v)
{
	return (ISFINITE(v.x) && ISFINITE(v.y));
}
template <typename TYPE>
FORCEINLINE bool ISFINITE(const cv::Point3_<TYPE>& v)
{
	return (ISFINITE(v.x) && ISFINITE(v.y) && ISFINITE(v.z));
}
template <typename TYPE, int m, int n>
FORCEINLINE bool ISFINITE(const cv::Matx<TYPE,m,n>& v)
{
	ASSERT(v.isContinuous());
	return ISFINITE(v.val, m*n);
}
template <typename TYPE, int R, int C>
FORCEINLINE bool ISFINITE(const Eigen::Matrix<TYPE,R,C>& m)
{
	return ISFINITE(m.data(), m.size());
}
/*----------------------------------------------------------------*/

template <typename TYPE>
inline IDX MinIndex(const TYPE* data, IDX size) {
	ASSERT(size>0);
	IDX idx = 0;
	for (IDX i=1; i<size; ++i)
		if (data[idx] > data[i])
			idx = i;
	return idx;
}
template <typename TYPE>
inline IDX MaxIndex(const TYPE* data, IDX size) {
	ASSERT(size>0);
	IDX idx = 0;
	for (IDX i=1; i<size; ++i)
		if (data[idx] < data[i])
			idx = i;
	return idx;
}
/*----------------------------------------------------------------*/


// geometry
template <typename TYPE, typename ACCTYPE>
inline ACCTYPE dot(const TYPE* a, const TYPE* b, size_t n) {
	ACCTYPE s(0);
	const TYPE* const last = a+n;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		s += ACCTYPE(a[0])*ACCTYPE(b[0]) + ACCTYPE(a[1])*ACCTYPE(b[1]) + ACCTYPE(a[2])*ACCTYPE(b[2]) + ACCTYPE(a[3])*ACCTYPE(b[3]);
		a += 4; b += 4;
	}
	#endif
	while (a < last)
		s += ACCTYPE(*a++) * ACCTYPE(*b++);
	return s;
}
template <typename TYPE, typename ACCTYPE, int N>
inline ACCTYPE dot(const TYPE* a, const TYPE* b) {
	ACCTYPE s(0);
	const TYPE* const last = a+N;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		s += ACCTYPE(a[0])*ACCTYPE(b[0]) + ACCTYPE(a[1])*ACCTYPE(b[1]) + ACCTYPE(a[2])*ACCTYPE(b[2]) + ACCTYPE(a[3])*ACCTYPE(b[3]);
		a += 4; b += 4;
	}
	#endif
	while (a < last)
		s += ACCTYPE(*a++) * ACCTYPE(*b++);
	return s;
}
#ifdef _USE_SSE
template <>
inline float dot<float,float>(const float* l, const float* r, size_t size) {
	return sse_f_t::dot(l, r, size);
}
template <>
inline double dot<double,double>(const double* l, const double* r, size_t size) {
	return sse_d_t::dot(l, r, size);
}
#endif

template <typename TYPE, int m, int n>
inline TYPE dot(const TMatrix<TYPE,m,n>& l, const TMatrix<TYPE,m,n>& r) {
	return ((const cv::Vec<TYPE,m*n>&)l).dot((const cv::Vec<TYPE,m*n>&)r);
}
template <typename TYPE, int m, int n>
inline TYPE dot(const cv::Matx<TYPE,m,n>& l, const cv::Matx<TYPE,m,n>& r) {
	return ((const cv::Vec<TYPE,m*n>&)l).dot((const cv::Vec<TYPE,m*n>&)r);
}

inline float dot(float v1, float v2) {
	return v1 * v2;
}
inline double dot(double v1, double v2) {
	return v1 * v2;
}
template <typename TYPE>
inline TYPE dot(const cv::Point_<TYPE>* v1, const cv::Point_<TYPE>* v2, size_t size) {
	return dot((const TYPE*)v1, (const TYPE*)v2, size*2);
}
template <typename TYPE>
inline TYPE dot(const cv::Point3_<TYPE>* v1, const cv::Point3_<TYPE>* v2, size_t size) {
	return dot((const TYPE*)v1, (const TYPE*)v2, size*3);
}
template <typename TYPE, int m>
inline TYPE dot(const cv::Vec<TYPE,m>* v1, const cv::Vec<TYPE,m>* v2, size_t size) {
	return dot((const TYPE*)v1, (const TYPE*)v2, size*m);
}

template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> cross(const TMatrix<TYPE,m,n>& l, const TMatrix<TYPE,m,n>& r) {
	return static_cast<cv::Matx<TYPE,m,n>&>(((const typename TMatrix<TYPE,m*n,1>::Vec&)l).cross((const typename TMatrix<TYPE,m*n,1>::Vec&)r));
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> cross(const cv::Matx<TYPE,m,n>& l, const cv::Matx<TYPE,m,n>& r) {
	return static_cast<cv::Matx<TYPE,m,n>&>(((const typename TMatrix<TYPE,m*n,1>::Vec&)l).cross((const typename TMatrix<TYPE,m*n,1>::Vec&)r));
}

template <typename TYPE>
struct RealType { typedef REAL type; };
template <>
struct RealType<float> { typedef float type; };
template <>
struct RealType<double> { typedef double type; };

template <typename TYPE>
inline typename RealType<TYPE>::type normSq(const cv::Point_<TYPE>& v) {
	typedef typename RealType<TYPE>::type real;
	return SQUARE((real)v.x)+SQUARE((real)v.y);
}
template <typename TYPE>
inline typename RealType<TYPE>::type normSq(const cv::Point3_<TYPE>& v) {
	typedef typename RealType<TYPE>::type real;
	return SQUARE((real)v.x)+SQUARE((real)v.y)+SQUARE((real)v.z);
}
template <typename TYPE, int m, int n>
inline typename RealType<TYPE>::type normSq(const TMatrix<TYPE,m,n>& v) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>(v.val, m*n);
}
template <typename TYPE, int m, int n>
inline typename RealType<TYPE>::type normSq(const cv::Matx<TYPE,m,n>& v) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>(v.val, m*n);
}
template <typename TYPE>
inline typename RealType<TYPE>::type normSq(const TDMatrix<TYPE>& v) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>(v.cv::Mat::ptr<const TYPE>(), v.area());
}
template <typename TYPE>
inline typename RealType<TYPE>::type normSq(const cv::Mat_<TYPE>& v) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>(v.cv::Mat::ptr<const TYPE>(), v.cols*v.rows);
}

inline REAL normSq(int v) {
	return SQUARE((REAL)v);
}
inline REAL normSq(unsigned v) {
	return SQUARE((REAL)v);
}
inline float normSq(float v) {
	return SQUARE(v);
}
inline double normSq(double v) {
	return SQUARE(v);
}
template <typename TYPE>
inline typename RealType<TYPE>::type normSq(const cv::Point_<TYPE>* v, size_t size) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>((const TYPE*)v, size*2);
}
template <typename TYPE>
inline typename RealType<TYPE>::type normSq(const cv::Point3_<TYPE>* v, size_t size) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>((const TYPE*)v, size*3);
}
template <typename TYPE, int m>
inline typename RealType<TYPE>::type normSq(const cv::Vec<TYPE,m>* v, size_t size) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>((const TYPE*)v, size*m);
}

template <typename TYPE, typename ACCTYPE>
inline ACCTYPE normSq(const TYPE* a, size_t n) {
	ACCTYPE s(0);
	const TYPE* const last = a+n;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		s += SQUARE(ACCTYPE(a[0])) + SQUARE(ACCTYPE(a[1])) + SQUARE(ACCTYPE(a[2])) + SQUARE(ACCTYPE(a[3]));
		a += 4;
	}
	#endif
	while (a < last)
		s += SQUARE(ACCTYPE(*a++));
	return s;
}
template <typename TYPE, typename ACCTYPE, int N>
inline ACCTYPE normSq(const TYPE* a) {
	ACCTYPE s(0);
	const TYPE* const last = a+N;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		s += SQUARE(ACCTYPE(a[0])) + SQUARE(ACCTYPE(a[1])) + SQUARE(ACCTYPE(a[2])) + SQUARE(ACCTYPE(a[3]));
		a += 4;
	}
	#endif
	while (a < last)
		s += SQUARE(ACCTYPE(*a++));
	return s;
}

template <typename TYPE, typename INTTYPE, typename ACCTYPE>
inline ACCTYPE normSq(const TYPE* a, const TYPE* b, size_t n) {
	ACCTYPE s(0);
	const TYPE* const last = a+n;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		const INTTYPE v0(a[0] - b[0]), v1(a[1] - b[1]), v2(a[2] - b[2]), v3(a[3] - b[3]);
		s += ACCTYPE(v0*v0 + v1*v1 + v2*v2 + v3*v3);
		a += 4; b += 4;
	}
	#endif
	while (a < last) {
		const INTTYPE v(*a++ - *b++);
		s += ACCTYPE(v*v);
	}
	return s;
}
template <typename TYPE, typename INTTYPE, typename ACCTYPE, int N>
inline ACCTYPE normSq(const TYPE* a, const TYPE* b) {
	ACCTYPE s(0);
	const TYPE* const last = a+N;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		const INTTYPE v0(a[0] - b[0]), v1(a[1] - b[1]), v2(a[2] - b[2]), v3(a[3] - b[3]);
		s += ACCTYPE(v0*v0 + v1*v1 + v2*v2 + v3*v3);
		a += 4; b += 4;
	}
	#endif
	while (a < last) {
		const INTTYPE v(*a++ - *b++);
		s += ACCTYPE(v*v);
	}
	return s;
}

template <typename TYPE, typename ACCTYPE>
inline ACCTYPE normSqDelta(TYPE* a, size_t n, const ACCTYPE avg) {
	ACCTYPE s(0);
	const TYPE* const last = a+n;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		const ACCTYPE v0(a[0]-=avg), v1(a[1]-=avg), v2(a[2]-=avg), v3(a[3]-=avg);
		s += v0*v0 + v1*v1 + v2*v2 + v3*v3;
		a += 4;
	}
	#endif
	while (a < last) {
		const ACCTYPE v(*a++ -= avg);
		s += v*v;
	}
	return s;
}
template <typename TYPE, typename ACCTYPE, int N>
inline ACCTYPE normSqDelta(TYPE* a, const ACCTYPE avg) {
	ACCTYPE s(0);
	const TYPE* const last = a+N;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		const ACCTYPE v0(a[0]-=avg), v1(a[1]-=avg), v2(a[2]-=avg), v3(a[3]-=avg);
		s += v0*v0 + v1*v1 + v2*v2 + v3*v3;
		a += 4;
	}
	#endif
	while (a < last) {
		const ACCTYPE v(*a++ -= avg);
		s += v*v;
	}
	return s;
}

template <typename TYPE, typename ACCTYPE>
inline ACCTYPE normSqZeroMean(TYPE* v, size_t n) {
	TYPE* a = v;
	ACCTYPE avg(0);
	const TYPE* const last = a+n;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		avg += ACCTYPE(a[0]) + ACCTYPE(a[1]) + ACCTYPE(a[2]) + ACCTYPE(a[3]);
		a += 4;
	}
	#endif
	while (a < last)
		avg += ACCTYPE(*a++);
	avg /= (ACCTYPE)n;
	
	a = v;
	ACCTYPE s(0);
	#if 1 // unrolled
	while (a < lastgroup) {
		const ACCTYPE v0(a[0]-=avg), v1(a[1]-=avg), v2(a[2]-=avg), v3(a[3]-=avg);
		s += v0*v0 + v1*v1 + v2*v2 + v3*v3;
		a += 4;
	}
	#endif
	while (a < last) {
		const ACCTYPE v(*a++ -= avg);
		s += v*v;
	}
	return s;
}
template <typename TYPE, typename ACCTYPE, int N>
inline ACCTYPE normSqZeroMean(TYPE* v) {
	TYPE* a = v;
	ACCTYPE avg(0);
	const TYPE* const last = a+N;
	#if 1 // unrolled
	const TYPE* const lastgroup = last-3;
	while (a < lastgroup) {
		avg += ACCTYPE(a[0]) + ACCTYPE(a[1]) + ACCTYPE(a[2]) + ACCTYPE(a[3]);
		a += 4;
	}
	#endif
	while (a < last)
		avg += ACCTYPE(*a++);
	avg /= (ACCTYPE)N;
	
	a = v;
	ACCTYPE s(0);
	#if 1 // unrolled
	while (a < lastgroup) {
		const ACCTYPE v0(a[0]-=avg), v1(a[1]-=avg), v2(a[2]-=avg), v3(a[3]-=avg);
		s += v0*v0 + v1*v1 + v2*v2 + v3*v3;
		a += 4;
	}
	#endif
	while (a < last) {
		const ACCTYPE v(*a++ -= avg);
		s += v*v;
	}
	return s;
}

template <typename TYPE>
inline typename RealType<TYPE>::type norm(const TPoint2<TYPE>& v) {
	return SQRT(normSq(v));
}
template <typename TYPE>
inline typename RealType<TYPE>::type norm(const TPoint3<TYPE>& v) {
	return SQRT(normSq(v));
}
template <typename TYPE, int m, int n>
inline typename RealType<TYPE>::type norm(const TMatrix<TYPE,m,n>& v) {
	typedef typename RealType<TYPE>::type real;
	return SQRT(cv::normL2Sqr<TYPE,real>(v.val, m*n));
}
template <typename TYPE>
inline typename RealType<TYPE>::type norm(const TDMatrix<TYPE>& v) {
	typedef typename RealType<TYPE>::type real;
	return SQRT(cv::normL2Sqr<TYPE,real>(v.cv::Mat::ptr<const TYPE>(), v.area()));
}

template <typename TYPE>
inline TPoint2<TYPE> normalized(const TPoint2<TYPE>& v) {
	return cv::normalize((const typename TPoint2<TYPE>::cvVec&)v);
}
template <typename TYPE>
inline TPoint2<TYPE> normalized(const cv::Point_<TYPE>& v) {
	return cv::normalize((const typename TPoint2<TYPE>::cvVec&)v);
}
template <typename TYPE>
inline TPoint3<TYPE> normalized(const TPoint3<TYPE>& v) {
	return cv::normalize((const typename TPoint3<TYPE>::cvVec&)v);
}
template <typename TYPE>
inline TPoint3<TYPE> normalized(const cv::Point3_<TYPE>& v) {
	return cv::normalize((const typename TPoint3<TYPE>::cvVec&)v);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> normalized(const TMatrix<TYPE,m,n>& v) {
	return static_cast<cv::Matx<TYPE,m,n>&>(cv::normalize((const typename TMatrix<TYPE,m*n,1>::Vec&)v));
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> normalized(const cv::Matx<TYPE,m,n>& v) {
	return static_cast<cv::Matx<TYPE,m,n>&>(cv::normalize((const typename TMatrix<TYPE,m*n,1>::Vec&)v));
}

template <typename TYPE>
inline void normalize(TPoint2<TYPE>& v) {
	(typename TPoint2<TYPE>::cvVec&)v = cv::normalize((const typename TPoint2<TYPE>::cvVec&)v);
}
template <typename TYPE>
inline void normalize(cv::Point_<TYPE>& v) {
	(typename TPoint2<TYPE>::cvVec&)v = cv::normalize((const typename TPoint2<TYPE>::cvVec&)v);
}
template <typename TYPE>
inline void normalize(TPoint3<TYPE>& v) {
	(typename TPoint3<TYPE>::cvVec&)v = cv::normalize((const typename TPoint3<TYPE>::cvVec&)v);
}
template <typename TYPE>
inline void normalize(cv::Point3_<TYPE>& v) {
	(typename TPoint3<TYPE>::cvVec&)v = cv::normalize((const typename TPoint3<TYPE>::cvVec&)v);
}
template <typename TYPE, int m, int n>
inline void normalize(TMatrix<TYPE,m,n>& v) {
	(typename TMatrix<TYPE,m*n,1>::Vec&)v = cv::normalize((const typename TMatrix<TYPE,m*n,1>::Vec&)v);
}
template <typename TYPE, int m, int n>
inline void normalize(cv::Matx<TYPE,m,n>& v) {
	(typename TMatrix<TYPE,m*n,1>::Vec&)v = cv::normalize((const typename TMatrix<TYPE,m*n,1>::Vec&)v);
}

template <typename TYPE>
inline TYPE area(const TPoint2<TYPE>& v1, const TPoint2<TYPE>& v2) {
	return ABS(v1.x * v2.y - v1.y * v2.x);
}
template <typename TYPE>
inline TYPE area(const cv::Point_<TYPE>& v1, const cv::Point_<TYPE>& v2) {
	return ABS(v1.x * v2.y - v1.y * v2.x);
}
template <typename TYPE>
inline TYPE area(const TPoint3<TYPE>& v1, const TPoint3<TYPE>& v2) {
	return norm(cross(v1,v2));
}
template <typename TYPE>
inline TYPE area(const cv::Point3_<TYPE>& v1, const cv::Point3_<TYPE>& v2) {
	return norm(cross(v1,v2));
}
/*----------------------------------------------------------------*/


/** @brief comparison function for floating point values
	See http://www.boost.org/libs/test/doc/components/test_tools/floating_point_comparison.html */
template<typename _Tp> // left == right
inline bool equal(const _Tp& left, const _Tp& right, const _Tp& eps = std::numeric_limits<_Tp>::epsilon())
{ return (left==right); }
template<>
inline bool equal<double>(const double& left, const double& right, const double& eps) {
	// the code below does not work when one value is zero
	if (left==0. || right==0.)
		return (ABS(left-right) <= eps);
	// compare two non-zero values
	return (ABS(left-right) <= eps*MAXF(ABS(right),ABS(left)));
}
template<>
inline bool equal<float>(const float& left, const float& right, const float& eps) {
	// the code below does not work when one value is zero
	if (left==0. || right==0.)
		return (ABS(left-right) <= eps);
	// compare two non-zero values
	return (ABS(left-right) <= eps*MAXF(ABS(right),ABS(left)));
}

/** @brief comparison function for floating point values */
template<typename _Tp> // left < right
inline bool less(const _Tp& left, const _Tp& right, const _Tp& eps = std::numeric_limits<_Tp>::epsilon())
{ if (left==0. || right==0.) return (right-left) > eps;
	return ((right-left) > eps * MINF(right,left)); }

/** @brief comparison function for floating point values */
template<typename _Tp>  // left <= right
inline bool lessEqual(const _Tp& left, const _Tp& right, const _Tp& eps = std::numeric_limits<_Tp>::epsilon())
{ return (less(left, right, eps) || equal(left, right, eps)); }

/** @brief comparison function for floating point values */
template<typename _Tp> // left > right
inline bool greater(const _Tp& left, const _Tp& right, const _Tp& eps = std::numeric_limits<_Tp>::epsilon())
{ if (left==0. || right==0.) return (left-right) > eps;
	return ((left-right) > eps * MINF(right,left)); }

/** @brief comparison function for floating point values */
template<typename _Tp> // left >= right
inline bool greaterEqual(const _Tp& left, const _Tp& right, const _Tp& eps = std::numeric_limits<_Tp>::epsilon())
{ return (greater(left,right) || equal(left, right, eps)); }
/*----------------------------------------------------------------*/


/** @brief determine the number of decimals to store, w.g. to file
	@returns the number of decimals between
	lower bound guaranteed precision (digits10)
	and the upper bound max. precision (nr. of binary digits) */
#ifdef WIN32
#  pragma warning(push, 2)
#endif // WIN32
template<typename _Tp>
inline unsigned int decimalsToStore()
{
	if (std::numeric_limits<_Tp>::is_integer){
		// integer types can display exactly one decimal more than their guaranteed precision digits
		return std::numeric_limits<_Tp>::digits10 +1;
	} else {
		//return std::numeric_limits<_Tp>::digits10; // lower bound
		return std::numeric_limits<_Tp>::digits;   // upper bound
	}
}
#ifdef WIN32
#  pragma warning(pop)
#endif // WIN32
/*----------------------------------------------------------------*/


// operators

// TPoint2 operators
#if CV_MAJOR_VERSION > 2
template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE> operator/(const TPoint2<TYPE>& pt, TYPEM m) {
	return TPoint2<TYPE>(pt.x/m, pt.y/m);
}
template <typename TYPEM>
inline TPoint2<float> operator/(const TPoint2<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	return TPoint2<float>(invm*pt.x, invm*pt.y);
}
template <typename TYPEM>
inline TPoint2<double> operator/(const TPoint2<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	return TPoint2<double>(invm*pt.x, invm*pt.y);
}

template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE>& operator/=(TPoint2<TYPE>& pt, TYPEM m) {
	pt.x /= m; pt.y /= m;
	return pt;
}
template <typename TYPEM>
inline TPoint2<float>& operator/=(TPoint2<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	pt.x *= invm; pt.y *= invm;
	return pt;
}
template <typename TYPEM>
inline TPoint2<double>& operator/=(TPoint2<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	pt.x *= invm; pt.y *= invm;
	return pt;
}
#else
template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE> operator/(const cv::Point_<TYPE>& pt, TYPEM m) {
	return TPoint2<TYPE>(pt.x/m, pt.y/m);
}
template <typename TYPEM>
inline TPoint2<float> operator/(const cv::Point_<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	return TPoint2<float>(invm*pt.x, invm*pt.y);
}
template <typename TYPEM>
inline TPoint2<double> operator/(const cv::Point_<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	return TPoint2<double>(invm*pt.x, invm*pt.y);
}

template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE>& operator/=(cv::Point_<TYPE>& pt, TYPEM m) {
	pt.x /= m; pt.y /= m;
	return (TPoint2<TYPE>&)pt;
}
template <typename TYPEM>
inline TPoint2<float>& operator/=(cv::Point_<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	pt.x *= invm; pt.y *= invm;
	return (TPoint2<float>&)pt;
}
template <typename TYPEM>
inline TPoint2<double>& operator/=(cv::Point_<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	pt.x *= invm; pt.y *= invm;
	return (TPoint2<double>&)pt;
}
#endif

template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE> operator/(const cv::Point_<TYPE>& pt0, const cv::Point_<TYPEM>& pt1) {
	return TPoint2<TYPE>(pt0.x/pt1.x, pt0.y/pt1.y);
}
template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE>& operator/=(cv::Point_<TYPE>& pt0, const cv::Point_<TYPEM>& pt1) {
	pt0.x/=pt1.x; pt0.y/=pt1.y;
	return (TPoint2<TYPE>&)pt0;
}

template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE> operator*(const cv::Point_<TYPE>& pt0, const cv::Point_<TYPEM>& pt1) {
	return TPoint2<TYPE>(pt0.x*pt1.x, pt0.y*pt1.y);
}
template <typename TYPE, typename TYPEM>
inline TPoint2<TYPE>& operator*=(cv::Point_<TYPE>& pt0, const cv::Point_<TYPEM>& pt1) {
	pt0.x*=pt1.x; pt0.y*=pt1.y;
	return (TPoint2<TYPE>&)pt0;
}

// TPoint3 operators
#if CV_MAJOR_VERSION > 2
template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE> operator/(const TPoint3<TYPE>& pt, TYPEM m) {
	return TPoint3<TYPE>(pt.x/m, pt.y/m, pt.z/m);
}
template <typename TYPEM>
inline TPoint3<float> operator/(const TPoint3<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	return TPoint3<float>(invm*pt.x, invm*pt.y, invm*pt.z);
}
template <typename TYPEM>
inline TPoint3<double> operator/(const TPoint3<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	return TPoint3<double>(invm*pt.x, invm*pt.y, invm*pt.z);
}

template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE>& operator/=(TPoint3<TYPE>& pt, TYPEM m) {
	pt.x /= m; pt.y /= m; pt.z /= m;
	return pt;
}
template <typename TYPEM>
inline TPoint3<float>& operator/=(TPoint3<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	pt.x *= invm; pt.y *= invm; pt.z *= invm;
	return pt;
}
template <typename TYPEM>
inline TPoint3<double>& operator/=(TPoint3<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	pt.x *= invm; pt.y *= invm; pt.z *= invm;
	return pt;
}
#else
template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE> operator/(const cv::Point3_<TYPE>& pt, TYPEM m) {
	return TPoint3<TYPE>(pt.x/m, pt.y/m, pt.z/m);
}
template <typename TYPEM>
inline TPoint3<float> operator/(const cv::Point3_<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	return TPoint3<float>(invm*pt.x, invm*pt.y, invm*pt.z);
}
template <typename TYPEM>
inline TPoint3<double> operator/(const cv::Point3_<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	return TPoint3<double>(invm*pt.x, invm*pt.y, invm*pt.z);
}

template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE>& operator/=(cv::Point3_<TYPE>& pt, TYPEM m) {
	pt.x /= m; pt.y /= m; pt.z /= m;
	return (TPoint3<TYPE>&)pt;
}
template <typename TYPEM>
inline TPoint3<float>& operator/=(cv::Point3_<float>& pt, TYPEM m) {
	const float invm(INVERT(float(m)));
	pt.x *= invm; pt.y *= invm; pt.z *= invm;
	return (TPoint3<float>&)pt;
}
template <typename TYPEM>
inline TPoint3<double>& operator/=(cv::Point3_<double>& pt, TYPEM m) {
	const double invm(INVERT(double(m)));
	pt.x *= invm; pt.y *= invm; pt.z *= invm;
	return (TPoint3<double>&)pt;
}
#endif

template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE> operator/(const cv::Point3_<TYPE>& pt0, const cv::Point3_<TYPEM>& pt1) {
	return TPoint3<TYPE>(pt0.x/pt1.x, pt0.y/pt1.y, pt0.z/pt1.z);
}
template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE>& operator/=(cv::Point3_<TYPE>& pt0, const cv::Point3_<TYPEM>& pt1) {
	pt0.x/=pt1.x; pt0.y/=pt1.y; pt0.z/=pt1.z;
	return (TPoint3<TYPE>&)pt0;
}

template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE> operator*(const cv::Point3_<TYPE>& pt0, const cv::Point3_<TYPEM>& pt1) {
	return TPoint3<TYPE>(pt0.x*pt1.x, pt0.y*pt1.y, pt0.z*pt1.z);
}
template <typename TYPE, typename TYPEM>
inline TPoint3<TYPE>& operator*=(cv::Point3_<TYPE>& pt0, const cv::Point3_<TYPEM>& pt1) {
	pt0.x*=pt1.x; pt0.y*=pt1.y; pt0.z*=pt1.z;
	return (TPoint3<TYPE>&)pt0;
}

// TPixel operators
template <typename TYPE, typename TYPEM>
inline TPixel<TYPE> operator/(const TPixel<TYPE>& pt, TYPEM m) {
	const TYPEM invm(INVERT(m));
	return TPixel<TYPE>(invm*pt.r, invm*pt.g, invm*pt.b);
}
template <typename TYPE, typename TYPEM>
inline TPixel<TYPE>& operator/=(TPixel<TYPE>& pt, TYPEM m) {
	const TYPEM invm(INVERT(m));
	pt.r *= invm; pt.g *= invm; pt.b *= invm;
	return pt;
}
template <typename TYPE>
inline TPixel<TYPE> operator/(const TPixel<TYPE>& pt0, const TPixel<TYPE>& pt1) {
	return TPixel<TYPE>(pt0.r/pt1.r, pt0.g/pt1.g, pt0.b/pt1.b);
}
template <typename TYPE>
inline TPixel<TYPE>& operator/=(TPixel<TYPE>& pt0, const TPixel<TYPE>& pt1) {
	pt0.r/=pt1.r; pt0.g/=pt1.g; pt0.b/=pt1.b;
	return pt0;
}
template <typename TYPE>
inline TPixel<TYPE> operator*(const TPixel<TYPE>& pt0, const TPixel<TYPE>& pt1) {
	return TPixel<TYPE>(pt0.r*pt1.r, pt0.g*pt1.g, pt0.b*pt1.b);
}
template <typename TYPE>
inline TPixel<TYPE>& operator*=(TPixel<TYPE>& pt0, const TPixel<TYPE>& pt1) {
	pt0.r*=pt1.r; pt0.g*=pt1.g; pt0.b*=pt1.b;
	return pt0;
}

// TColor operators
template <typename TYPE, typename TYPEM>
inline TColor<TYPE> operator/(const TColor<TYPE>& pt, TYPEM m) {
	const TYPEM invm(INVERT(m));
	return TColor<TYPE>(invm*pt.r, invm*pt.g, invm*pt.b, invm*pt.a);
}
template <typename TYPE, typename TYPEM>
inline TColor<TYPE>& operator/=(TColor<TYPE>& pt, TYPEM m) {
	const TYPEM invm(INVERT(m));
	pt.r *= invm; pt.g *= invm; pt.b *= invm; pt.a *= invm;
	return pt;
}
template <typename TYPE>
inline TColor<TYPE> operator/(const TColor<TYPE>& pt0, const TColor<TYPE>& pt1) {
	return TColor<TYPE>(pt0.r/pt1.r, pt0.g/pt1.g, pt0.b/pt1.b, pt0.a/pt1.a);
}
template <typename TYPE>
inline TColor<TYPE>& operator/=(TColor<TYPE>& pt0, const TColor<TYPE>& pt1) {
	pt0.r/=pt1.r; pt0.g/=pt1.g; pt0.b/=pt1.b; pt0.a/=pt1.a;
	return pt0;
}
template <typename TYPE>
inline TColor<TYPE> operator*(const TColor<TYPE>& pt0, const TColor<TYPE>& pt1) {
	return TColor<TYPE>(pt0.r*pt1.r, pt0.g*pt1.g, pt0.b*pt1.b, pt0.a*pt1.a);
}
template <typename TYPE>
inline TColor<TYPE>& operator*=(TColor<TYPE>& pt0, const TColor<TYPE>& pt1) {
	pt0.r*=pt1.r; pt0.g*=pt1.g; pt0.b*=pt1.b; pt0.a*=pt1.a;
	return pt0;
}

// operator /
template <typename TYPE, typename TYPEM>
FORCEINLINE SEACAVE::TPoint2<TYPE> operator/(TYPEM n, const cv::Point_<TYPE>& d) {
	return SEACAVE::TPoint2<TYPE>(n/d.x, n/d.y);
}
template <typename TYPE, typename TYPEM>
FORCEINLINE SEACAVE::TPoint3<TYPE> operator/(TYPEM n, const cv::Point3_<TYPE>& d) {
	return SEACAVE::TPoint3<TYPE>(n/d.x, n/d.y, n/d.z);
}
template <typename TYPE, typename TYPEM>
FORCEINLINE SEACAVE::TPixel<TYPE> operator/(TYPEM n, const SEACAVE::TPixel<TYPE>& d) {
	return SEACAVE::TPixel<TYPE>(n/d.r, n/d.g, n/d.b);
}
template <typename TYPE, typename TYPEM>
FORCEINLINE SEACAVE::TColor<TYPE> operator/(TYPEM n, const SEACAVE::TColor<TYPE>& d) {
	return SEACAVE::TColor<TYPE>(n/d.r, n/d.g, n/d.b, n/d.a);
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE


namespace cv {

#if CV_MAJOR_VERSION > 2
template<> class DataType<unsigned>
{
public:
	typedef unsigned    value_type;
	typedef value_type  work_type;
	typedef value_type  channel_type;
	typedef value_type  vec_type;
	enum { generic_type = 0,
		depth        = CV_32S,
		channels     = 1,
		fmt          = (int)'i',
		type         = CV_MAKETYPE(depth, channels)
	};
};

template <typename REAL_TYPE, typename INT_TYPE>
INT_TYPE cvRANSACUpdateNumIters(REAL_TYPE p, REAL_TYPE ep, INT_TYPE modelPoints, INT_TYPE maxIters)
{
	ASSERT(p>=0 && p<=1);
	ASSERT(ep>=0 && ep<=1);

	// avoid inf's & nan's
	REAL_TYPE num = MAXF(REAL_TYPE(1)-p, DBL_MIN);
	REAL_TYPE denom = REAL_TYPE(1)-powi(REAL_TYPE(1)-ep, modelPoints);
	if (denom < REAL_TYPE(DBL_MIN))
		return 0;

	num = log(num);
	denom = log(denom);

	return (denom >= 0 || -num >= (-denom)*maxIters ? maxIters : ROUND2INT(num/denom));
}
#else
// Convenience creation functions. In the far future, there may be variadic templates here.
template<typename T>
Ptr<T> makePtr()
{
	return Ptr<T>(new T());
}
template<typename T, typename A1>
Ptr<T> makePtr(const A1& a1)
{
	return Ptr<T>(new T(a1));
}
template<typename T, typename A1, typename A2>
Ptr<T> makePtr(const A1& a1, const A2& a2)
{
	return Ptr<T>(new T(a1, a2));
}
template<typename T, typename A1, typename A2, typename A3>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3)
{
	return Ptr<T>(new T(a1, a2, a3));
}
template<typename T, typename A1, typename A2, typename A3, typename A4>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3, const A4& a4)
{
	return Ptr<T>(new T(a1, a2, a3, a4));
}
template<typename T, typename A1, typename A2, typename A3, typename A4, typename A5>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3, const A4& a4, const A5& a5)
{
	return Ptr<T>(new T(a1, a2, a3, a4, a5));
}
template<typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3, const A4& a4, const A5& a5, const A6& a6)
{
	return Ptr<T>(new T(a1, a2, a3, a4, a5, a6));
}
template<typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3, const A4& a4, const A5& a5, const A6& a6, const A7& a7)
{
	return Ptr<T>(new T(a1, a2, a3, a4, a5, a6, a7));
}
template<typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3, const A4& a4, const A5& a5, const A6& a6, const A7& a7, const A8& a8)
{
	return Ptr<T>(new T(a1, a2, a3, a4, a5, a6, a7, a8));
}
template<typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3, const A4& a4, const A5& a5, const A6& a6, const A7& a7, const A8& a8, const A9& a9)
{
	return Ptr<T>(new T(a1, a2, a3, a4, a5, a6, a7, a8, a9));
}
template<typename T, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9, typename A10>
Ptr<T> makePtr(const A1& a1, const A2& a2, const A3& a3, const A4& a4, const A5& a5, const A6& a6, const A7& a7, const A8& a8, const A9& a9, const A10& a10)
{
	return Ptr<T>(new T(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10));
}

// property implementation macros
#define CV_IMPL_PROPERTY_RO(type, name, member) \
	inline type get##name() const { return member; }

#define CV_HELP_IMPL_PROPERTY(r_type, w_type, name, member) \
	CV_IMPL_PROPERTY_RO(r_type, name, member) \
	inline void set##name(w_type val) { member = val; }

#define CV_HELP_WRAP_PROPERTY(r_type, w_type, name, internal_name, internal_obj) \
	r_type get##name() const { return internal_obj.get##internal_name(); } \
	void set##name(w_type val) { internal_obj.set##internal_name(val); }

#define CV_IMPL_PROPERTY(type, name, member) CV_HELP_IMPL_PROPERTY(type, type, name, member)
#define CV_IMPL_PROPERTY_S(type, name, member) CV_HELP_IMPL_PROPERTY(type, const type &, name, member)

#define CV_WRAP_PROPERTY(type, name, internal_name, internal_obj)  CV_HELP_WRAP_PROPERTY(type, type, name, internal_name, internal_obj)
#define CV_WRAP_PROPERTY_S(type, name, internal_name, internal_obj) CV_HELP_WRAP_PROPERTY(type, const type &, name, internal_name, internal_obj)

#define CV_WRAP_SAME_PROPERTY(type, name, internal_obj) CV_WRAP_PROPERTY(type, name, name, internal_obj)
#define CV_WRAP_SAME_PROPERTY_S(type, name, internal_obj) CV_WRAP_PROPERTY_S(type, name, name, internal_obj)
#endif

//! copy every second source image element to the destination image
inline void downsample2x(InputArray _src, OutputArray _dst)
{
	Mat src(_src.getMat());
	#if 1
	_dst.create((src.rows+1)/2, (src.cols+1)/2, src.type());
	#else
	if (_dst.empty()) {
		// create a new matrix
		_dst.create(src.rows/2, src.cols/2, src.type());
	} else {
		// overwrite elements in the existing matrix
		ASSERT(src.rows > 0 && (unsigned)(src.rows-_dst.size().height*2) <= 1);
		ASSERT(src.cols > 0 && (unsigned)(src.cols-_dst.size().width*2) <= 1);
		ASSERT(src.type() == _dst.type());
	}
	#endif
	Mat dst(_dst.getMat());
	ASSERT(src.elemSize() == dst.elemSize());
	switch (src.elemSize()) {
	case 1:
		for (int i=0; i<dst.rows; ++i)
			for (int j=0; j<dst.cols; ++j)
				dst.at<uint8_t>(i,j) = src.at<uint8_t>(2*i,2*j);
		break;
	case 2:
		for (int i=0; i<dst.rows; ++i)
			for (int j=0; j<dst.cols; ++j)
				dst.at<uint16_t>(i,j) = src.at<uint16_t>(2*i,2*j);
		break;
	case 4:
		for (int i=0; i<dst.rows; ++i)
			for (int j=0; j<dst.cols; ++j)
				dst.at<float>(i,j) = src.at<float>(2*i,2*j);
		break;
	case 8:
		for (int i=0; i<dst.rows; ++i)
			for (int j=0; j<dst.cols; ++j)
				dst.at<double>(i,j) = src.at<double>(2*i,2*j);
		break;
	default:
		for (int i=0; i<dst.rows; ++i)
			for (int j=0; j<dst.cols; ++j)
				memcpy(dst.ptr(i,j), src.ptr(2*i,2*j), src.elemSize());
	}
}
//! copy elements of source image to the 2 x position in the destination image
inline void upsample2x(InputArray _src, OutputArray _dst)
{
	Mat src(_src.getMat());
	if (_dst.empty()) {
		// create a new matrix
		_dst.create(src.rows*2, src.cols*2, src.type());
	} else {
		// overwrite only new elements in the existing matrix
		ASSERT(src.rows > 0 && src.rows*2-1 <= _dst.size().height);
		ASSERT(src.cols > 0 && src.cols*2-1 <= _dst.size().width);
		ASSERT(src.type() == _dst.type());
	}
	Mat dst(_dst.getMat());
	ASSERT(src.elemSize() == dst.elemSize());
	switch (src.elemSize()) {
	case 1:
		for (int i=0; i<src.rows; ++i)
			for (int j=0; j<src.cols; ++j)
				dst.at<uint8_t>(2*i,2*j) = src.at<uint8_t>(i,j);
		break;
	case 2:
		for (int i=0; i<src.rows; ++i)
			for (int j=0; j<src.cols; ++j)
				dst.at<uint16_t>(2*i,2*j) = src.at<uint16_t>(i,j);
		break;
	case 4:
		for (int i=0; i<src.rows; ++i)
			for (int j=0; j<src.cols; ++j)
				dst.at<float>(2*i,2*j) = src.at<float>(i,j);
		break;
	case 8:
		for (int i=0; i<src.rows; ++i)
			for (int j=0; j<src.cols; ++j)
				dst.at<double>(2*i,2*j) = src.at<double>(i,j);
		break;
	default:
		for (int i=0; i<src.rows; ++i)
			for (int j=0; j<src.cols; ++j)
				memcpy(dst.ptr(2*i,2*j), src.ptr(i,j), src.elemSize());
	}
}

} // namespace cv

// Informative template class for OpenCV "scalars"
#define DEFINE_GENERIC_CVDATATYPE(tp,ctp) namespace cv { \
template<> class DataType<tp> { \
public: \
	typedef tp value_type; \
	typedef value_type work_type; \
	typedef ctp channel_type; \
	typedef value_type vec_type; \
	enum { generic_type = 0, \
		depth = DataDepth<channel_type>::value, \
		channels = sizeof(value_type)/sizeof(channel_type), \
		fmt=DataDepth<channel_type>::fmt, \
		type = CV_MAKETYPE(depth, channels) }; \
}; }
#define DEFINE_CVDATATYPE(tp) DEFINE_GENERIC_CVDATATYPE(tp,tp::Type)

// define specialized cv:DataType<>
DEFINE_CVDATATYPE(SEACAVE::Point2i)
DEFINE_CVDATATYPE(SEACAVE::Point2f)
DEFINE_CVDATATYPE(SEACAVE::Point2d)
/*----------------------------------------------------------------*/
DEFINE_CVDATATYPE(SEACAVE::Point3i)
DEFINE_CVDATATYPE(SEACAVE::Point3f)
DEFINE_CVDATATYPE(SEACAVE::Point3d)
/*----------------------------------------------------------------*/
DEFINE_CVDATATYPE(SEACAVE::Pixel8U)
DEFINE_CVDATATYPE(SEACAVE::Pixel32F)
DEFINE_CVDATATYPE(SEACAVE::Pixel64F)
/*----------------------------------------------------------------*/
DEFINE_CVDATATYPE(SEACAVE::Color8U)
DEFINE_CVDATATYPE(SEACAVE::Color32F)
DEFINE_CVDATATYPE(SEACAVE::Color64F)
/*----------------------------------------------------------------*/
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DMatrix8S, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DMatrix8U, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DMatrix32S, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DMatrix32U, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DMatrix32F, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DMatrix64F, uint8_t)
/*----------------------------------------------------------------*/
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DVector8S, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DVector8U, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DVector32S, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DVector32U, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DVector32F, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::DVector64F, uint8_t)
/*----------------------------------------------------------------*/
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image8U, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image32F, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image64F, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image8U3, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image8U4, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image32F3, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image32F4, uint8_t)
/*----------------------------------------------------------------*/


namespace SEACAVE {

// C L A S S  //////////////////////////////////////////////////////

inline TPoint2<REAL> CastReal(const cv::Point2f& ptFloat) {
	return TPoint2<REAL>(ptFloat);
}
inline TPoint2<float> CastFloat(const cv::Point_<REAL>& ptReal) {
	return TPoint2<float>(ptReal);
}
inline TPoint2<double> CastDouble(const cv::Point2f& ptFloat) {
	return TPoint2<double>(ptFloat);
}
inline TPoint3<REAL> CastReal(const cv::Point3f& ptFloat) {
	return TPoint3<REAL>(ptFloat);
}
inline TPoint3<float> CastFloat(const cv::Point3_<REAL>& ptReal) {
	return TPoint3<float>(ptReal);
}
inline TPoint3<double> CastDouble(const cv::Point3f& ptFloat) {
	return TPoint3<double>(ptFloat);
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

template <typename TYPE>
TPoint3<TYPE> TPoint3<TYPE>::RotateAngleAxis(const TPoint3& pt, const TPoint3& angle_axis)
{
	TPoint3 result;
	const TYPE theta2(normSq(angle_axis));
	if (theta2 > TYPE(1e-12)) {
		// Away from zero, use the Rodriguez formula
		//
		//   result = pt costheta +
		//            (w x pt) * sintheta +
		//            w (w . pt) (1 - costheta)
		//
		// We want to be careful to only evaluate the square root if the
		// norm of the angle_axis vector is greater than zero. Otherwise
		// we get a division by zero.
		//
		const TYPE theta(SQRT(theta2));
		const TPoint3 w(angle_axis*(TYPE(1)/theta));
		const TYPE costheta(COS(theta));
		const TYPE sintheta(SIN(theta));
		const TPoint3 w_cross_pt(w.cross(pt));
		const TYPE w_dot_pt(w.dot(pt));
		result = pt*costheta + w_cross_pt*sintheta + w*(TYPE(1.0)-costheta)*w_dot_pt;
	} else {
		// Near zero, the first order Taylor approximation of the rotation
		// matrix R corresponding to a vector w and angle w is
		//
		//   R = I + hat(w) * sin(theta)
		//
		// But sintheta ~ theta and theta * w = angle_axis, which gives us
		//
		//  R = I + hat(w)
		//
		// and actually performing multiplication with the point pt, gives us
		// R * pt = pt + w x pt.
		//
		// Switching to the Taylor expansion at zero helps avoid all sorts
		// of numerical nastiness.
		const TPoint3 w_cross_pt(angle_axis.cross(pt));
		result = pt + w_cross_pt;
	}
	return result;
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

template <typename TYPE, int m, int n>
inline bool TMatrix<TYPE,m,n>::IsEqual(const Base& rhs) const
{
	for (register int i=0; i<elems; ++i)
		if (!ISEQUAL(val[i], rhs.val[i]))
			return false;
	return true;
}
template <typename TYPE, int m, int n>
inline bool TMatrix<TYPE,m,n>::IsEqual(const Base& rhs, TYPE eps) const
{
	for (register int i=0; i<elems; ++i)
		if (!equal(val[i], rhs.val[i], eps))
			return false;
	return true;
}

// calculate right null-vector of matrix A ([n,1])
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,n,1> TMatrix<TYPE,m,n>::RightNullVector(int flags /*= 0*/) const
{
	const cv::SVD svd(*this, (m >= n ? flags : flags|cv::SVD::FULL_UV));
	// the singular values could be checked for numerical stability
	// return result (last row transposed)
	return *svd.vt.ptr< const TMatrix<TYPE,n,1> >(m-1);
}
// calculate left null-vector of matrix A ([n,1])
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,n,1> TMatrix<TYPE,m,n>::LeftNullVector(int flags /*= 0*/) const
{
	return TMatrix<TYPE,m,n>(t()).RightNullVector(flags);
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

template <typename TYPE>
inline TYPE TDMatrix<TYPE>::getDetSquare() const
{
	ASSERT(cols == rows);
	const int dim = rows;
	const int subdim = dim-1;
	// end of recursion in 1x1:
	if (subdim==0)
		return Base::operator()(0,0);
	// end of recursion in 2x2:
	if (subdim==1)
		return (Base::operator()(0,0)*Base::operator()(1,1)-
				Base::operator()(1,0)*Base::operator()(0,1));
	TYPE d = 0;
	TDMatrix<TYPE> SubMatrix(subdim, subdim);
	for (register int sub=0; sub<dim; sub++) {
		// first column is used,
		// set up the n-1xn-1 submatrices from the right matrix part
		// this is done n times (loop counter="sub")
		for (register int i=0; i<subdim; i++) {
			// construct the sub matrix under inspection,
			// skip the first column
			for (register int j=0; j<subdim; j++) {
				SubMatrix(j,i) = Base::operator()((sub+j+1)%dim,i+1);
			}
		}
		// add value of subdeterminant to overall sum
		d += SubMatrix.getDetSquare() * Base::operator()(sub,0);
	}
	return d;
}

template <typename TYPE>
inline TDMatrix<TYPE> TDMatrix<TYPE>::getAdjoint() const
{
	ASSERT(rows==cols);
	TDMatrix<TYPE> Out(cols,rows);
	TDMatrix<TYPE> SubMatrix(rows-1,cols-1);
	for (int i=0; i<rows; i++) {
		for (int j=0; j<cols; j++) {
			for (int k=0; k<rows-1; k++) {
				const int kk = k<i?k:k+1;
				for (int r=0; r<rows-1; r++) {
					const int rr = r<j?r:r+1;
					SubMatrix(k,r) = Base::operator()(kk,rr);
				}
			}
			Out(j,i)= (((i+j)%2==0)?1:-1) * SubMatrix.getDetSquare();
		}
	}
	return Out;
}

template <typename TYPE>
inline void TDMatrix<TYPE>::getSystemMatrix(TDMatrix<TYPE>& dest) const
{
	// resize result to square matrix
	dest = TDMatrix::zeros(cols, cols);
	/// Hessian is symmetric! so first diagonal then lower left
	// diagonal
	for (int col = 0; col<cols; col++) {
		for (int k = 0; k<rows; k++) {
			dest(col,col)+=SQUARE(Base::operator()(k,col));
		}
	}
	// lower left (which is equal to transposed upper right)
	for (int r=1; r<cols; r++) {
		for (int c=0; c<r; c++) {
			for (int k = 0; k<rows; k++) {
				const TYPE* pJ_k_ = Base::operator[](k);
				dest(r,c) += pJ_k_[r] * pJ_k_[c];
			}
			dest(c,r)=dest(r,c); // symmetry of Hessian!
		}
	}
}


template <typename TYPE>
inline void TDMatrix<TYPE>::makeSymmetric()
{
	ASSERT(cols == rows);
	const int num = cols;
	for (int r=0; r<num; r++) {
		for (int c=r; c<num; c++) {
			Base::operator()(c,r) = Base::operator()(r,c) = TYPE((Base::operator()(c,r) + Base::operator()(r,c)) * 0.5);
		}
	}
}

template <typename TYPE>
inline TYPE TDMatrix<TYPE>::getNormL1() const
{
	TYPE result = 0;
	for (register const TYPE* dataP = getData()+Base::total()-1; dataP >= getData(); dataP--)
		result += ABS(*dataP);
	return result;
}


template <typename TYPE>
inline double TDMatrix<TYPE>::getNormL2() const
{
	double result = 0;
	for (register const TYPE* dataP = getData()+Base::total()-1; dataP >= getData(); dataP--)
		result += (double)((*dataP) * (*dataP));
	return SQRT(result);
}

/** Kronecker-product with matrix B, result in dest */
template <typename TYPE>
void TDMatrix<TYPE>::Kronecker(const TDMatrix& B, TDMatrix& dest) const
{
	const int A_rows = rows;
	const int A_cols = cols;
	const int B_rows = B.rows;
	const int B_cols = B.cols;

	dest.newsize(A_rows*B_rows,A_cols*B_cols);
	for (int i=0; i<A_rows; i++) {
		for (int j=0; j<A_cols; j++) {
			for (int r=0; r<B_rows; r++) {
				for (int s=0; s<B_cols; s++) {
					dest(i*B_rows+r,j*B_cols+s) = Base::operator()(i,j) * B(r,s);
				}
			}
		}
	}
}

template <typename TYPE>
void TDMatrix<TYPE>::SwapRows(int i, int r)
{
	TYPE* const rowi = Base::operator[](i);
	TYPE* const rowr = Base::operator[](r);
	for (int c=0; c<cols; c++)
		std::swap(rowi[c], rowr[c]);
}

//#define MDOUT(arg) if (verbose) cout << arg;
#define MDOUT(arg) {}

template <typename TYPE>
void TDMatrix<TYPE>::GaussJordan()
{
	//const bool verbose = false;
	const int numr = rows, numc = cols;
	const int max = MINF(numr, numc);
	///<< offset to right from diagonal, the working column is given by r+offs
	int offs = 0;
	for (int r=0; r<max; r++) { ///<< r is the working row
		MDOUT("working row: "<<r<<endl);
		// search for the next column which has non zero elements in or
		// below working row
		for (; r+offs<numc && Base::operator()(r, r+offs)==0.; offs++) {
			// search for the next row with non-zero element in current column
			MDOUT(*this<<"\nsearching for the next row with non-zero element "
				  <<"in current column "<<r+offs<<endl);
			int rr;
			for (rr=r+1; rr<numr && Base::operator()(rr, r+offs)==0.; rr++) {}
			if (rr!=numr) {
				MDOUT("found next non-zero element in column "<<r+offs<<" and row "<<rr
					  <<", swapping rows"<<endl);
				SwapRows(r, rr);
				break;
			}
			MDOUT("only zeros below ["<<r<<"]["<<r+offs<<"], increasing offset\n");
		}
		if (r+offs==numc) {
			// only zeros below and right of [rr][r+offs]
			MDOUT("no further non-zero elements below and right\n");
			break;
		}
		MDOUT("working column: "<<r+offs<<endl);

		/// this is guaranteed by the above
		ASSERT(Base::operator()(r,r+offs) != 0.0);

		//	divide working row by value of working position to get a 1 on the
		//	diagonal
		const TYPE tempval=Base::operator()(r,r+offs);
		MDOUT((*this)<<"\nscaling working row by "<<tempval<<endl);
		for (int col=0; col<numc; col++) {
			Base::operator()(r,col)=Base::operator()(r,col)/tempval;
		}

		//	eliminate values below working position by subtracting a multiple of
		//	the current row
		MDOUT("eliminitaing entries in working column ("<<r+offs
			  <<") below working row ("<<r<<")\n");
		for (long row=r+1; row<numr; ++row) {
			const TYPE wval=Base::operator()(row,r+offs);
			for (int col=0; col<numc; col++) {
				Base::operator()(row,col)-=wval*Base::operator()(r,col);
			}
		}
		MDOUT("finished manipulation of row "<<r<<"\n"<<*this<<endl<<"---------\n");
	}

	// zero the elements above the leading element in each row
	MDOUT("zero the elements above the leading element in each row\n");
	for (int r=numr-1; r>=0; --r) {
		// search for leading element
		int c;
		for (c=r; c<numc && Base::operator()(r,c)==0.; c++) {}
		//	eliminate value above working position by subtracting a multiple of
		//	the current row
		for (int row=r-1; row>=0; --row) {
			const TYPE wval=Base::operator()(row,c);
			for (int col=0; col<numc; ++col) {
				Base::operator()(row,col)-=wval*Base::operator()(r,col);
			}
		}
	}
	MDOUT("finished computation of reduced row echelon form: "<<*this<<endl);
}
#undef MDOUT
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

template <typename TYPE>
TDMatrix<TYPE> TDVector<TYPE>::getOuterProduct(const TDVector<TYPE>& v) const {
	TDMatrix<TYPE> mat(rows,v.rows);
	TYPE rVal;
	for (int row=0; row<rows; row++) {
		rVal = (*this)[row];
		for (int col=0; col<v.rows; col++)
			mat(row,col) = rVal * v[col];
	}
	return mat;
}

template <typename TYPE>
void TDVector<TYPE>::getKroneckerProduct(const TDVector<TYPE>& arg, TDVector<TYPE>& dst) const
{
	const int s1=rows, s2=arg.rows;
	int l=0;
	dst.newsize(s1*s2);
	for (register int i=0; i<s1; i++) {
		for (register int k=0; k<s2; k++) {
			dst[l]=(*this)[i]*arg[k];
			l++;
		}
	}
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

// Find a pixel inside the image
template <typename TYPE>
inline const TYPE& TImage<TYPE>::getPixel(int y, int x) const
{
	if (x < 0)
		x = 0;
	else if (x >= cols)
		x = cols-1;
	if (y < 0)
		y = 0;
	else if (y >= rows)
		y = rows-1;
	return BaseBase::operator()(y,x);
}
/*----------------------------------------------------------------*/


// sample by bilinear interpolation
template <typename TYPE>
template <typename T>
TYPE TImage<TYPE>::sample(const TPoint2<T>& pt) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	return y1*(x1*BaseBase::operator()( ly, lx) + x*BaseBase::operator()( ly, lx+1)) +
			y*(x1*BaseBase::operator()(ly+1,lx) + x*BaseBase::operator()(ly+1,lx+1));
}
template <typename TYPE>
template <typename T>
TYPE TImage<TYPE>::sampleSafe(const TPoint2<T>& pt) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	return y1*(x1*getPixel( ly, lx) + x*getPixel( ly, lx+1)) +
			y*(x1*getPixel(ly+1,lx) + x*getPixel(ly+1,lx+1));
}
/*----------------------------------------------------------------*/


// sample by bilinear interpolation, using only pixels that meet the user condition
template <typename TYPE>
template <typename T>
bool TImage<TYPE>::sample(TYPE& v, const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&)) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(BaseBase::operator()(ly  , lx  )); const bool b00(fncCond(x0y0));
	const TYPE& x1y0(BaseBase::operator()(ly  , lx+1)); const bool b10(fncCond(x1y0));
	const TYPE& x0y1(BaseBase::operator()(ly+1, lx  )); const bool b01(fncCond(x0y1));
	const TYPE& x1y1(BaseBase::operator()(ly+1, lx+1)); const bool b11(fncCond(x1y1));
	if (!b00 && !b10 && !b01 && !b11)
		return false;
	v = y1*(x1*(b00 ? x0y0 : (b10 ? x1y0 : (b01 ? x0y1 : x1y1))) + x*(b10 ? x1y0 : (b00 ? x0y0 : (b11 ? x1y1 : x0y1)))) +
		y *(x1*(b01 ? x0y1 : (b11 ? x1y1 : (b00 ? x0y0 : x1y0))) + x*(b11 ? x1y1 : (b01 ? x0y1 : (b10 ? x1y0 : x0y0))));
	return true;
}
template <typename TYPE>
template <typename T>
bool TImage<TYPE>::sampleSafe(TYPE& v, const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&)) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(getPixel(ly  , lx  )); const bool b00(fncCond(x0y0));
	const TYPE& x1y0(getPixel(ly  , lx+1)); const bool b10(fncCond(x1y0));
	const TYPE& x0y1(getPixel(ly+1, lx  )); const bool b01(fncCond(x0y1));
	const TYPE& x1y1(getPixel(ly+1, lx+1)); const bool b11(fncCond(x1y1));
	if (!b00 && !b10 && !b01 && !b11)
		return false;
	v = y1*(x1*(b00 ? x0y0 : (b10 ? x1y0 : (b01 ? x0y1 : x1y1))) + x*(b10 ? x1y0 : (b00 ? x0y0 : (b11 ? x1y1 : x0y1)))) +
		y *(x1*(b01 ? x0y1 : (b11 ? x1y1 : (b00 ? x0y0 : x1y0))) + x*(b11 ? x1y1 : (b01 ? x0y1 : (b10 ? x1y0 : x0y0))));
	return true;
}
// same as above, but using default value if the condition is not met
template <typename TYPE>
template <typename T>
TYPE TImage<TYPE>::sample(const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&), const TYPE& dv) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(BaseBase::operator()(ly  , lx  )); const bool b00(fncCond(x0y0));
	const TYPE& x1y0(BaseBase::operator()(ly  , lx+1)); const bool b10(fncCond(x1y0));
	const TYPE& x0y1(BaseBase::operator()(ly+1, lx  )); const bool b01(fncCond(x0y1));
	const TYPE& x1y1(BaseBase::operator()(ly+1, lx+1)); const bool b11(fncCond(x1y1));
	return y1*(x1*(b00 ? x0y0 : dv) + x*(b10 ? x1y0 : dv)) +
		y*(x1*(b01 ? x0y1 : dv) + x*(b11 ? x1y1 : dv));
}
template <typename TYPE>
template <typename T>
TYPE TImage<TYPE>::sampleSafe(const TPoint2<T>& pt, bool (STCALL *fncCond)(const TYPE&), const TYPE& dv) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(getPixel(ly  , lx  )); const bool b00(fncCond(x0y0));
	const TYPE& x1y0(getPixel(ly  , lx+1)); const bool b10(fncCond(x1y0));
	const TYPE& x0y1(getPixel(ly+1, lx  )); const bool b01(fncCond(x0y1));
	const TYPE& x1y1(getPixel(ly+1, lx+1)); const bool b11(fncCond(x1y1));
	return y1*(x1*(b00 ? x0y0 : dv) + x*(b10 ? x1y0 : dv)) +
			y*(x1*(b01 ? x0y1 : dv) + x*(b11 ? x1y1 : dv));
}
/*----------------------------------------------------------------*/


// Sampling functors
// These functors computes weight associated to each pixels
// Copyright (c) 2012, 2013, 2015 Pierre MOULON.
// Copyright (c) 2015 Romuald Perrot.
//
// For a (relative) sampling position x (in [0,1]) between two (consecutive) points :
//
//   A  .... x ......... B
//
//   w[0] is the weight associated to A
//   w[1] is the weight associated to B
//
// Note: The following functors generalize the sampling to more than two neighbors
// They all contains the width variable that specify the number of neighbors used for sampling
//
// All contain the operator () with the following definition:
//
//   @brief Computes weight associated to neighboring pixels
//   @author Romuald Perrot <perrot.romuald_AT_gmail.com>
//   @param x Sampling position
//   @param[out] weigth Sampling factors associated to the neighboring
//   @note weight must be at least width length
namespace Sampler {
// Linear sampling (ie: linear interpolation between two pixels)
template <typename TYPE>
struct Linear {
	typedef TYPE Type;

	static const int halfWidth = 1;
	static const int width = halfWidth*2;

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = TYPE(1) - x;
		weigth[1] = x;
	}
};
// Cubic interpolation between 4 pixels
//
// Interpolation weight is for A,B,C and D pixels given a x position as illustrated as follow :
//
// A      B    x C      D
//
// Cubic Convolution Interpolation for Digital Image Processing , R. Keys, eq(4)
//
// The cubic filter family with the two free parameters usually named B and C:
// if |x|<1
//   ((12-9B-6C)|x|^3 + (-18+12B+6C)|x|^2 + (6-2B))/6
// if 1<=|x|<2
//   ((-B-6C)|x|^3 + (6B+30C)|x|^2 + (-12B-48C)|x| + (8B+24C))/6
template <typename TYPE>
struct Cubic {
	typedef TYPE Type;

	static const int halfWidth = 2;
	static const int width = halfWidth*2;

	// Sharpness coefficient used to control sharpness of the cubic curve (between 0.5 to 0.75)
	// cubic(0,1/2): 0.5 gives better mathematically result (ie: approximation at 3 order precision - Catmull-Rom)
	const TYPE sharpness;
	inline Cubic(const TYPE& _sharpness = 0.5) : sharpness(_sharpness) {}

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		// remember :
		// A      B    x  C       D

		// weigth[0] -> weight for A
		// weight[1] -> weight for B
		// weight[2] -> weight for C
		// weight[3] -> weigth for D

		weigth[0] = CubicInter12(x + TYPE(1));
		weigth[1] = CubicInter01(x);
		weigth[2] = CubicInter01(TYPE(1) - x);
		weigth[3] = CubicInter12(TYPE(2) - x);
	}

	// Cubic interpolation for x (in [0,1])
	inline TYPE CubicInter01(const TYPE x) const {
		// A = -sharpness
		// f(x) = (A + 2) * x^3 - (A + 3) * x^2 + 1
		return ((TYPE(2)-sharpness)*x - (TYPE(3)-sharpness))*x*x + TYPE(1);
	}
	// Cubic interpolation for x (in [1,2])
	inline TYPE CubicInter12(const TYPE x) const {
		// A = -sharpness
		// f(x) = A * x^3 - 5 * A * x^2 + 8 * A * x - 4 * A
		return (((TYPE(5)-x)*x - TYPE(8))*x + TYPE(4))*sharpness;
	}
};
// Sampler spline16 -> Interpolation on 4 points used for 2D ressampling (16 = 4x4 sampling)
// Cubic interpolation with 0-derivative at edges (ie at A and D points)
// See Helmut Dersch for more details
//
// Some refs :
//  -   http://forum.doom9.org/archive/index.php/t-147117.html
//  -   http://avisynth.nl/index.php/Resampling
//  -   http://www.ipol.im/pub/art/2011/g_lmii/
//
// The idea is to consider 3 cubic splines (f1,f2,f3) in the sampling interval :
//
// A   f1    B    f2    C     f3     D
//
// with curves defined as follow :
// f1(x) = a1 x^3 + b1 x^2 + c1 x + d1
// f2(x) = a2 x^3 + b2 x^2 + c2 x + d2
// f3(x) = a3 x^3 + b2 x^2 + c3 x + d3
//
// We want to compute spline coefs for A,B,C,D assuming that:
// y0 = coef[A] = f1(-1)
// y1 = coef[B] = f1(0) = f2(0)
// y2 = coef[C] = f2(1) = f3(1)
// y3 = coef[D] = f3(2)
//
// coef are computed using the following constraints :
// Curve is continuous, ie:
// f1(0)  = f2(0)
// f2(1)  = f3(1)
// First derivative are equals, ie:
// f1'(0) = f2'(0)
// f2'(1) = f3'(1)
// Second derivative are equals, ie:
// f1''(0) = f2''(0)
// f2''(1) = f3''(0)
// Curve is, at boundary, with second derivative set to zero (it's a constraint introduced by Dersch), ie:
// f1''(-1) = 0
// f3''(2) = 0
//
// Then, you can solve for (a1,a2,a3,b1,b2,b3,c1,c2,c3,d1,d2,d3)
//
// for ex, for curve f2 you find :
//
// d2 = y1                                      // easy since y1 = f2(0)
// c2 = - 7/15 y0 - 1/5 y1 + 4/5 y2 - 2/15 y3
// b2 = 4/5 y0 - 9/5 y1 + 6/5 y2 - 1/5 y3
// a2 = - 1/3 y0 + y1 - y2 + 1/3 y3
//
//
// When you have coefs, you just have to express your curve as a linear combination of the control points, fort ex
// with f2 :
//
//
// f2(x) = w0(x) * y0 + w1(x) + y1 + w2(x) * y2 + w3(x) * y3
//
// with :
//
// w0(x) = - 1/3 * x^3 + 4/5 * x^2 - 7/15 * x
// w1(x) = x^3 - 9/5 * x^2 - 1/5 * x + 1
// w2(x) = -x^3 + 6/5 * x^2 + 4/5 * x
// w3(x) = 1/3 * x^3 - 1/5 * x^2 - 2/15 * x
//
// substituting boundary conditions gives the correct coefficients for y0,y1,y2,y3 giving the final sampling scheme
template <typename TYPE>
struct Spline16 {
	typedef TYPE Type;

	static const int halfWidth = 2;
	static const int width = halfWidth*2;

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = ((TYPE(-1) / TYPE(3) * x + TYPE(4) / TYPE(5)) * x - TYPE(7) / TYPE(15)) * x;
		weigth[1] = ((x - TYPE(9) / TYPE(5)) * x - TYPE(1) / TYPE(5)) * x + TYPE(1);
		weigth[2] = ((TYPE(6) / TYPE(5) - x) * x + TYPE(4) / TYPE(5)) * x;
		weigth[3] = ((TYPE(1) / TYPE(3) * x - TYPE(1) / TYPE(5)) * x - TYPE(2) / TYPE(15)) * x;
	}
};
// Sampler spline 36
// Same as spline 16 but on 6 neighbors (used for 6x6 frame)
template <typename TYPE>
struct Spline36 {
	typedef TYPE Type;

	static const int halfWidth = 3;
	static const int width = halfWidth*2;

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = ((TYPE(1) / TYPE(11) * x - TYPE(45) / TYPE(209)) * x + TYPE(26) / TYPE(209)) * x;
		weigth[1] = ((TYPE(-6) / TYPE(11) * x + TYPE(270) / TYPE(209)) * x - TYPE(156) / TYPE(209)) * x;
		weigth[2] = ((TYPE(13) / TYPE(11) * x - TYPE(453) / TYPE(209)) * x - TYPE(3) / TYPE(209)) * x + TYPE(1);
		weigth[3] = ((TYPE(-13) / TYPE(11) * x + TYPE(288) / TYPE(209)) * x + TYPE(168) / TYPE(209)) * x;
		weigth[4] = ((TYPE(6) / TYPE(11) * x - TYPE(72) / TYPE(209)) * x - TYPE(42) / TYPE(209)) * x;
		weigth[5] = ((TYPE(-1) / TYPE(11) * x + TYPE(12) / TYPE(209)) * x + TYPE(7) / TYPE(209)) * x;
	}
};
// Sampler spline 64
// Same as spline 16 but on 8 neighbors (used for 8x8 frame)
template <typename TYPE>
struct Spline64 {
	typedef TYPE Type;

	static const int halfWidth = 4;
	static const int width = halfWidth*2;

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = ((TYPE(-1) / TYPE(41) * x + TYPE(168) / TYPE(2911)) * x - TYPE(97) / TYPE(2911)) * x;
		weigth[1] = ((TYPE(6) / TYPE(41) * x - TYPE(1008) / TYPE(2911)) * x +  TYPE(582) / TYPE(2911)) * x;
		weigth[2] = ((TYPE(-24) / TYPE(41) * x + TYPE(4032) / TYPE(2911)) * x - TYPE(2328) / TYPE(2911)) * x;
		weigth[3] = ((TYPE(49) / TYPE(41) * x - TYPE(6387) / TYPE(2911)) * x - TYPE(3) / TYPE(2911)) * x + TYPE(1);
		weigth[4] = ((TYPE(-49) / TYPE(41) * x + TYPE(4050) / TYPE(2911)) * x + TYPE(2340) / TYPE(2911)) * x;
		weigth[5] = ((TYPE(24) / TYPE(41) * x - TYPE(1080) / TYPE(2911)) * x - TYPE(624) / TYPE(2911)) * x;
		weigth[6] = ((TYPE(-6) / TYPE(41) * x + TYPE(270) / TYPE(2911)) * x + TYPE(156) / TYPE(2911)) * x;
		weigth[7] = ((TYPE(1) / TYPE(41) * x - TYPE(45) / TYPE(2911)) * x - TYPE(26) / TYPE(2911)) * x;
	}
};
} // namespace Sampler
// Sample image at a specified position
// @param sampler used to make the sampling
// @param pt X and Y-coordinate of sampling
// @return Sampled value
template <typename TYPE>
template <typename SAMPLER, typename INTERTYPE>
INTERTYPE TImage<TYPE>::sample(const SAMPLER& sampler, const TPoint2<typename SAMPLER::Type>& pt) const
{
	typedef typename SAMPLER::Type T;

	const int im_width(width());
	const int im_height(height());

	// integer position of sample (x,y)
	const int grid_x(FLOOR2INT(pt.x));
	const int grid_y(FLOOR2INT(pt.y));

	// compute difference between exact pixel location and sample
	const T dx(pt.x-(T)grid_x);
	const T dy(pt.y-(T)grid_y);

	// get sampler weights
	T coefs_x[SAMPLER::width];
	sampler(dx, coefs_x);
	T coefs_y[SAMPLER::width];
	sampler(dy, coefs_y);

	// Sample a grid around specified grid point
	INTERTYPE res(0);
	T total_weight(0);
	for (int i = 0; i < SAMPLER::width; ++i) {
		// get current i value
		// +1 for correct scheme (draw it to be convinced)
		const int cur_i(grid_y + 1 + i - SAMPLER::halfWidth);
		// handle out of range
		if (cur_i < 0 || cur_i >= im_height)
			continue;
		for (int j = 0; j < SAMPLER::width; ++j) {
			// get current j value
			// +1 for the same reason
			const int cur_j(grid_x + 1 + j - SAMPLER::halfWidth);
			// handle out of range
			if (cur_j < 0 || cur_j >= im_width)
				continue;
			// sample input image and weight according to sampler
			const T w(coefs_x[j] * coefs_y[i]);
			res += INTERTYPE(BaseBase::operator()(cur_i, cur_j)) * w;
			total_weight += w;
		}
	}

	// if value too small, it should be instable, so return the sampled value
	if (total_weight <= T(0.2))
		return INTERTYPE();
	if (total_weight != T(1))
		return res/total_weight;
	return res;
}
/*----------------------------------------------------------------*/


// convert color image to gray
template <typename TYPE>
template <typename T>
void TImage<TYPE>::toGray(TImage<T>& out, int code, bool bNormalize) const
{
	#if 1
	ASSERT(code==cv::COLOR_RGB2GRAY || code==cv::COLOR_RGBA2GRAY || code==cv::COLOR_BGR2GRAY || code==cv::COLOR_BGRA2GRAY);
	static const T coeffsRGB[] = {T(0.299), T(0.587), T(0.114)};
	static const T coeffsBGR[] = {T(0.114), T(0.587), T(0.299)};
	static const T coeffsRGBn[] = {T(0.299/255), T(0.587/255), T(0.114/255)};
	static const T coeffsBGRn[] = {T(0.114/255), T(0.587/255), T(0.299/255)};
	const float* coeffs;
	switch (code) {
	case cv::COLOR_BGR2GRAY:
	case cv::COLOR_BGRA2GRAY:
		coeffs = (bNormalize ? coeffsBGRn : coeffsBGR);
		break;
	case cv::COLOR_RGB2GRAY:
	case cv::COLOR_RGBA2GRAY:
		coeffs = (bNormalize ? coeffsRGBn : coeffsRGB);
		break;
	default:
		ASSERT("Unsupported image format" == NULL);
	}
	const T &cb(coeffs[0]), &cg(coeffs[1]), &cr(coeffs[2]);
	if (out.rows!=rows || out.cols!=cols)
		out.create(rows, cols);
	ASSERT(cv::Mat::isContinuous());
	ASSERT(out.cv::Mat::isContinuous());
	const int scn(channels());
	T* dst = out.cv::Mat::ptr<T>();
	T* const dstEnd = dst + out.area();
	typedef typename cv::DataType<TYPE>::channel_type ST;
	for (const ST* src=cv::Mat::ptr<ST>(); dst!=dstEnd; src+=scn)
		*dst++ = cb*T(src[0]) + cg*T(src[1]) + cr*T(src[2]);
	#else
	cv::Mat cimg;
	convertTo(cimg, cv::DataType<real>::type);
	cv::cvtColor(cimg, out, code);
	if (bNormalize)
		out *= T(1)/T(255);
	#endif
}
/*----------------------------------------------------------------*/


// compute image scale for a given max and min resolution
template <typename TYPE>
unsigned TImage<TYPE>::computeMaxResolution(unsigned maxImageSize, unsigned& level, unsigned minImageSize)
{
	// if the max level it's used, return original image size
	if (level == 0)
		return maxImageSize;
	// compute the resolution corresponding to the desired level
	unsigned imageSize = (maxImageSize >> level);
	// if the image is too small
	if (imageSize < minImageSize) {
		// start from the max level
		level = 0;
		while ((maxImageSize>>(level+1)) > minImageSize)
			++level;
		imageSize = (maxImageSize >> level);
	}
	return imageSize;
}
template <typename TYPE>
unsigned TImage<TYPE>::computeMaxResolution(unsigned& level, unsigned minImageSize) const
{
	const unsigned maxImageSize = (unsigned)MAXF(width(), height());
	return computeMaxResolution(maxImageSize, level, minImageSize);
}
/*----------------------------------------------------------------*/


// Raster the given triangle and output the position of each pixel of the triangle;
// based on "Advanced Rasterization" by Nick (Nicolas Capens)
// http://devmaster.net/forums/topic/1145-advanced-rasterization/
template <typename TYPE>
template <typename T, typename PARSER>
void TImage<TYPE>::RasterizeTriangle(const TPoint2<T>& v1, const TPoint2<T>& v2, const TPoint2<T>& v3, PARSER& parser)
{
	// 28.4 fixed-point coordinates
	const int_t Y1 = ROUND2INT(T(16) * v1.y);
	const int_t Y2 = ROUND2INT(T(16) * v2.y);
	const int_t Y3 = ROUND2INT(T(16) * v3.y);

	const int_t X1 = ROUND2INT(T(16) * v1.x);
	const int_t X2 = ROUND2INT(T(16) * v2.x);
	const int_t X3 = ROUND2INT(T(16) * v3.x);

	// Deltas
	const int_t DX12 = X1 - X2;
	const int_t DX23 = X2 - X3;
	const int_t DX31 = X3 - X1;

	const int_t DY12 = Y1 - Y2;
	const int_t DY23 = Y2 - Y3;
	const int_t DY31 = Y3 - Y1;

	// Fixed-point deltas
	const int_t FDX12 = DX12 << 4;
	const int_t FDX23 = DX23 << 4;
	const int_t FDX31 = DX31 << 4;

	const int_t FDY12 = DY12 << 4;
	const int_t FDY23 = DY23 << 4;
	const int_t FDY31 = DY31 << 4;

	// Bounding rectangle
	int minx = (int)((MINF3(X1, X2, X3) + 0xF) >> 4);
	int maxx = (int)((MAXF3(X1, X2, X3) + 0xF) >> 4);
	int miny = (int)((MINF3(Y1, Y2, Y3) + 0xF) >> 4);
	int maxy = (int)((MAXF3(Y1, Y2, Y3) + 0xF) >> 4);

	// Block size, standard 8x8 (must be power of two)
	const int q = 8;

	// Start in corner of 8x8 block
	minx &= ~(q - 1);
	miny &= ~(q - 1);

	// Half-edge constants
	int_t C1 = DY12 * X1 - DX12 * Y1;
	int_t C2 = DY23 * X2 - DX23 * Y2;
	int_t C3 = DY31 * X3 - DX31 * Y3;

	// Correct for fill convention
	if (DY12 < 0 || (DY12 == 0 && DX12 > 0)) C1++;
	if (DY23 < 0 || (DY23 == 0 && DX23 > 0)) C2++;
	if (DY31 < 0 || (DY31 == 0 && DX31 > 0)) C3++;

	// Loop through blocks
	int pixy = miny;
	for (int y = miny; y < maxy; y += q)
	{
		for (int x = minx; x < maxx; x += q)
		{
			// Corners of block
			int_t x0 = int_t(x) << 4;
			int_t x1 = int_t(x + q - 1) << 4;
			int_t y0 = int_t(y) << 4;
			int_t y1 = int_t(y + q - 1) << 4;

			// Evaluate half-space functions
			bool a00 = C1 + DX12 * y0 - DY12 * x0 > 0;
			bool a10 = C1 + DX12 * y0 - DY12 * x1 > 0;
			bool a01 = C1 + DX12 * y1 - DY12 * x0 > 0;
			bool a11 = C1 + DX12 * y1 - DY12 * x1 > 0;
			int a = (a00 << 0) | (a10 << 1) | (a01 << 2) | (a11 << 3);

			bool b00 = C2 + DX23 * y0 - DY23 * x0 > 0;
			bool b10 = C2 + DX23 * y0 - DY23 * x1 > 0;
			bool b01 = C2 + DX23 * y1 - DY23 * x0 > 0;
			bool b11 = C2 + DX23 * y1 - DY23 * x1 > 0;
			int b = (b00 << 0) | (b10 << 1) | (b01 << 2) | (b11 << 3);

			bool c00 = C3 + DX31 * y0 - DY31 * x0 > 0;
			bool c10 = C3 + DX31 * y0 - DY31 * x1 > 0;
			bool c01 = C3 + DX31 * y1 - DY31 * x0 > 0;
			bool c11 = C3 + DX31 * y1 - DY31 * x1 > 0;
			int c = (c00 << 0) | (c10 << 1) | (c01 << 2) | (c11 << 3);

			// Skip block when outside an edge
			if (a == 0x0 || b == 0x0 || c == 0x0) continue;

			int nowpixy = pixy;

			// Accept whole block when totally covered
			if (a == 0xF && b == 0xF && c == 0xF)
			{
				for (int iy = 0; iy < q; iy++)
				{
					for (int ix = x; ix < x + q; ix++)
						parser(ImageRef(ix,nowpixy));

					++nowpixy;
				}
			}
			else // Partially covered block
			{
				int_t CY1 = C1 + DX12 * y0 - DY12 * x0;
				int_t CY2 = C2 + DX23 * y0 - DY23 * x0;
				int_t CY3 = C3 + DX31 * y0 - DY31 * x0;

				for (int iy = y; iy < y + q; iy++)
				{
					int_t CX1 = CY1;
					int_t CX2 = CY2;
					int_t CX3 = CY3;

					for (int ix = x; ix < x + q; ix++)
					{
						if (CX1 > 0 && CX2 > 0 && CX3 > 0)
							parser(ImageRef(ix,nowpixy));

						CX1 -= FDY12;
						CX2 -= FDY23;
						CX3 -= FDY31;
					}

					CY1 += FDX12;
					CY2 += FDX23;
					CY3 += FDX31;

					++nowpixy;
				}
			}
		}

		pixy += q;
	}
}

// drawing line between 2 points from left to right
// papb -> pcpd
// pa, pb, pc, pd must then be sorted before
template <typename T, typename PARSER>
inline void _ProcessScanLine(int y, const TPoint3<T>& pa, const TPoint3<T>& pb, const TPoint3<T>& pc, const TPoint3<T>& pd, PARSER& parser) {
	// Thanks to current Y, we can compute the gradient to compute others values like
	// the starting X (sx) and ending X (ex) to draw between
	// if pa.y == pb.y or pc.y == pd.y, gradient is forced to 1
	const T gradient1 = CLAMP(pa.y != pb.y ? (T(y) - pa.y) / (pb.y - pa.y) : T(1), T(0), T(1));
	const T gradient2 = CLAMP(pc.y != pd.y ? (T(y) - pc.y) / (pd.y - pc.y) : T(1), T(0), T(1));

	const int sx = FLOOR2INT(lerp(pa.x, pb.x, gradient1));
	const int ex = FLOOR2INT(lerp(pc.x, pd.x, gradient2));

	// starting Z & ending Z
	const T z1 = lerp(pa.z, pb.z, gradient1);
	const T z2 = lerp(pc.z, pd.z, gradient2);

	// drawing a line from left (sx) to right (ex)
	const T invd = T(1) / (T)(ex - sx);
	for (int x = sx; x < ex; ++x) {
		const T gradient = T(x - sx) * invd;
		ASSERT(gradient >= T(0) && gradient <= T(1));
		const T z = lerp(z1, z2, gradient);
		parser(ImageRef(x,y), z);
	}
}
// Raster the given triangle and output the position and depth of each pixel of the triangle;
// based on "Learning how to write a 3D software engine � Rasterization & Z-Buffering" by Nick (David Rousset)
// http://blogs.msdn.com/b/davrous/archive/2013/06/21/tutorial-part-4-learning-how-to-write-a-3d-software-engine-in-c-ts-or-js-rasterization-amp-z-buffering.aspx
template <typename TYPE>
template <typename T, typename PARSER>
void TImage<TYPE>::RasterizeTriangleDepth(TPoint3<T> p1, TPoint3<T> p2, TPoint3<T> p3, PARSER& parser)
{
	// sorting the points in order to always have this order on screen p1, p2 & p3
	// with p1 always up (thus having the Y the lowest possible to be near the top screen)
	// then p2 between p1 & p3
	if (p1.y > p2.y)
		std::swap(p1, p2);
	if (p2.y > p3.y)
		std::swap(p2, p3);
	if (p1.y > p2.y)
		std::swap(p1, p2);

	// computing lines' directions (slopes)
	// http://en.wikipedia.org/wiki/Slope
	const T dP1P2 = (p2.y - p1.y > 0 ? (p2.x - p1.x) / (p2.y - p1.y) : T(0));
	const T dP1P3 = (p3.y - p1.y > 0 ? (p3.x - p1.x) / (p3.y - p1.y) : T(0));

	// first case where triangles are like that:
	// P1
	// -
	// --
	// - -
	// -  -
	// -   - P2
	// -  -
	// - -
	// -
	// P3
	if (dP1P2 > dP1P3) {
		for (int y = (int)p1.y; y <= (int)p3.y; ++y) {
			if (y < p2.y)
				_ProcessScanLine(y, p1, p3, p1, p2, parser);
			else
				_ProcessScanLine(y, p1, p3, p2, p3, parser);
		}
	}
	// second case where triangles are like that:
	//       P1
	//        -
	//       -- 
	//      - -
	//     -  -
	// P2 -   - 
	//     -  -
	//      - -
	//        -
	//       P3
	else {
		for (int y = (int)p1.y; y <= (int)p3.y; ++y) {
			if (y < p2.y)
				_ProcessScanLine(y, p1, p2, p1, p3, parser);
			else
				_ProcessScanLine(y, p2, p3, p1, p3, parser);
		}
	}
}


template <typename TYPE>
void SEACAVE::TImage<TYPE>::DrawLine(const ImageRef& x1, const ImageRef& x2,
								 FncDrawPoint fncDrawPoint, void* pData)
{
	const ImageRef d = ABS(x2 - x1);
	int sx = x1.x<x2.x ? 1 : -1;
	int sy = x1.y<x2.y ? 1 : -1;
	int err = (d.x>d.y ? d.x : -d.y)/2, e2;

	ImageRef p = x1;
	while (true) {
		fncDrawPoint(p, pData);
		if (p == x2) break;
		e2 = err;
		if (e2 >-d.x) { err -= d.y; p.x += sx; }
		if (e2 < d.y) { err += d.x; p.y += sy; }
	}
}

#define ipart_(X) FLOOR2INT(X)
#define round_(X) ROUND2INT(X)
#define fpart_(X) ((X)-(float)ipart_(X))
#define rfpart_(X) (1.f-fpart_(X))
template <typename TYPE>
bool TImage<TYPE>::DrawLineAntialias(Point2f x1, Point2f x2,
									 FncDrawPointAntialias fncDrawPoint, void* pData)
{
	bool revert = false;
	const Point2f dx = x2 - x1;
	if (ABS(dx.x) > ABS(dx.y)) {
		const ImageRef delta(0,1);
		if (x2.x < x1.x) {
			std::swap(x1, x2);
			revert = true;
		}

		const float gradient = dx.y / dx.x;
		int xpxl1 = round_(x1.x);
		float yend = x1.y + gradient*(float(xpxl1) - x1.x);
		int ypxl1 = ipart_(yend);
		float xgap = rfpart_(x1.x + 0.5);
		float tmp = fpart_(yend)*xgap;
		fncDrawPoint(ImageRef(xpxl1, ypxl1), delta, xgap-tmp, tmp, pData);

		float intery = yend + gradient;
		int xpxl2 = round_(x2.x);
		yend = x2.y + gradient*(float(xpxl2) - x2.x);
		int ypxl2 = ipart_(yend);
		for (int x=xpxl1+1; x<xpxl2; ++x) {
			tmp = fpart_(intery);
			fncDrawPoint(ImageRef(x, ipart_(intery)), delta, 1.f-tmp, tmp, pData);
			intery += gradient;
		}

		xgap = fpart_(x2.x+0.5);
		tmp = fpart_(yend)*xgap;
		fncDrawPoint(ImageRef(xpxl2, ypxl2), delta, xgap-tmp, tmp, pData);
	} else {
		const ImageRef delta(1,0);
		if (x2.y < x1.y) {
			std::swap(x1, x2);
			revert = true;
		}

		const float gradient = dx.x / dx.y;
		int ypxl1 = round_(x1.y);
		float xend = x1.x + gradient*(float(ypxl1) - x1.y);
		float ygap = rfpart_(x1.y + 0.5);
		int xpxl1 = ipart_(xend);
		float tmp = fpart_(xend)*ygap;
		fncDrawPoint(ImageRef(xpxl1, ypxl1), delta, ygap-tmp, tmp, pData);

		float interx = xend + gradient;
		int ypxl2 = round_(x2.y);
		xend = x2.x + gradient*(float(ypxl2) - x2.y);
		int xpxl2 = ipart_(xend);

		for (int y=ypxl1+1; y<ypxl2; ++y) {
			tmp = fpart_(interx);
			fncDrawPoint(ImageRef(ipart_(interx), y), delta, 1.f-tmp, tmp, pData);
			interx += gradient;
		}

		ygap = fpart_(x2.y+0.5);
		tmp = fpart_(xend)*ygap;
		fncDrawPoint(ImageRef(xpxl2, ypxl2), delta, ygap-tmp, tmp, pData);
	}
	return revert;
}
#undef ipart_

#define ipart_(X) ((int)(X))
template <typename TYPE>
bool TImage<TYPE>::DrawLineAntialias(const ImageRef& x1, const ImageRef& x2,
									 FncDrawPointAntialias fncDrawPoint, void* pData)
{
	const ImageRef dx = x2 - x1;
	if (ABS(dx.x) > ABS(dx.y)) {
		const ImageRef delta(0,1);
		const float gradient = (float)dx.y / dx.x;
		float intery = float(x1.y) + gradient;
		if (x2.x < x1.x) {
			for (int x=x1.x-1; x>x2.x; --x) {
				const float tmp = fpart_(intery);
				fncDrawPoint(ImageRef(x, ipart_(intery)), delta, 1.f-tmp, tmp, pData);
				intery += gradient;
			}
			return true;
		}
		else {
			for (int x=x1.x+1; x<x2.x; ++x) {
				const float tmp = fpart_(intery);
				fncDrawPoint(ImageRef(x, ipart_(intery)), delta, 1.f-tmp, tmp, pData);
				intery += gradient;
			}
			return false;
		}
	} else {
		const ImageRef delta(1,0);
		const float gradient = (float)dx.x / dx.y;
		float interx = float(x1.x) + gradient;
		if (x2.y < x1.y) {
			for (int y=x1.y-1; y>x2.y; --y) {
				const float tmp = fpart_(interx);
				fncDrawPoint(ImageRef(ipart_(interx), y), delta, 1.f-tmp, tmp, pData);
				interx += gradient;
			}
			return true;
		}
		else {
			for (int y=x1.y+1; y<x2.y; ++y) {
				const float tmp = fpart_(interx);
				fncDrawPoint(ImageRef(ipart_(interx), y), delta, 1.f-tmp, tmp, pData);
				interx += gradient;
			}
			return false;
		}
	}
}
#undef ipart_
#undef fpart_
#undef round_
#undef rfpart_
/*----------------------------------------------------------------*/


template <typename TYPE>
bool TImage<TYPE>::Load(const String& fileName)
{
	cv::Mat img(cv::imread(fileName, CV_LOAD_IMAGE_UNCHANGED));
	if (img.empty()) {
		VERBOSE("error: loading image '%s'", fileName.c_str());
		return false;
	}
	if (img.channels() != Base::channels()) {
		if (img.channels() == 3 && Base::channels() == 1)
			cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
		else if (img.channels() == 1 && Base::channels() == 3)
			cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
		else if (img.channels() == 4 && Base::channels() == 1)
			cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
		else if (img.channels() == 1 && Base::channels() == 4)
			cv::cvtColor(img, img, cv::COLOR_GRAY2BGRA);
	}
	if (img.type() == Base::type())
		cv::swap(img, *this);
	else
		img.convertTo(*this, Base::type());
	return true;
}
/*----------------------------------------------------------------*/

template <typename TYPE>
bool TImage<TYPE>::Save(const String& fileName) const
{
	std::vector<int> compression_params;
	const String ext(Util::getFileExt(fileName).ToLower());
	if (ext == ".png") {
		compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(6);
	} else
	if (ext == ".jpg") {
		compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
		compression_params.push_back(80);
	} else
	if (ext == ".pfm") {
		if (Base::depth() != CV_32F)
			return false;
		Util::ensureDirectory(fileName);
		File fImage(fileName, File::WRITE, File::CREATE | File::TRUNCATE);
		if (!fImage.isOpen())
			return false;
		ASSERT(sizeof(float) == 4);
		static const uint8_t b[4] = { 255, 0, 0, 0 };
		static const bool is_little_endian = (*((float*)b) < 1.f);
		static const double scale = (is_little_endian ? -1.0 : 1.0);
		fImage.print("Pf\n%d %d\n%lf\n", width(), height(), scale*Base::channels());
		ASSERT(sizeof(float)*Base::channels() == Base::step.p[1]);
		const size_t rowbytes = (size_t)Base::size.p[1]*Base::step.p[1];
		for (int i=0; i<rows; ++i)
			fImage.write(cv::Mat::ptr<const float>(i), rowbytes);
		return true;
	}

	try {
		if (!cv::imwrite(fileName, *this, compression_params)) {
			VERBOSE("error: saving image '%s'", fileName.c_str());
			return false;
		}
	}
	catch (std::runtime_error& ex) {
		VERBOSE("error: saving image '%s' (exception: %s)", fileName.c_str(), ex.what());
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

template <typename TYPE>
void TImage<TYPE>::Show(const String& winname, int delay=0, bool bDestroy=true) const
{
	cv::imshow(winname, *this);
	cv::waitKey(delay);
	if (bDestroy)
		cv::destroyWindow(winname);
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

// Create a derivative kernel, such that you can take the
// derivative of an image by convolving with the kernel horizontally and vertically
inline const TMatrix<double,3,3>& CreateDerivativeKernel3x3() {
	static const double kernel[3*3] = {
		-1.0/8.0, 0.0, 1.0/8.0,
		-2.0/8.0, 0.0, 2.0/8.0,
		-1.0/8.0, 0.0, 1.0/8.0
	};
	return *((const TMatrix<double,3,3>*)kernel);
}
inline const TMatrix<double,3,3>& CreateDerivativeKernel3x3xx() {
	static const double kernel[3*3] = {
		1.0/6.0, -2.0/6.0, 1.0/6.0,
		4.0/6.0, -8.0/6.0, 4.0/6.0,
		1.0/6.0, -2.0/6.0, 1.0/6.0
	};
	return *((const TMatrix<double,3,3>*)kernel);
}
inline const TMatrix<double,3,3>& CreateDerivativeKernel3x3xy() {
	static const double kernel[3*3] = {
		 1.0/4.0, 0.0, -1.0/4.0,
		   0.0,   0.0,    0.0,
		-1.0/4.0, 0.0,  1.0/4.0
	};
	return *((const TMatrix<double,3,3>*)kernel);
}
// same as above, but using central differences like approach: Noise Robust Gradient Operators
// (see: http://www.holoborodko.com/pavel/image-processing/edge-detection)
inline const TMatrix<double,3,5>& CreateDerivativeKernel3x5() {
	static const double kernel[3*5] = {
		-1.0/32.0, -2.0/32.0, 0.0, 2.0/32.0, 1.0/32.0,
		-2.0/32.0, -4.0/32.0, 0.0, 4.0/32.0, 2.0/32.0,
		-1.0/32.0, -2.0/32.0, 0.0, 2.0/32.0, 1.0/32.0
	};
	return *((const TMatrix<double,3,5>*)kernel);
}
inline const TMatrix<double,5,7>& CreateDerivativeKernel5x7() {
	static const double kernel[5*7] = {
		-1.0/512.0,  -4.0/512.0, -5.0/512.0, 0.0,  5.0/512.0,  4.0/512.0, 1.0/512.0,
		-4.0/512.0, -16.0/512.0, 20.0/512.0, 0.0, 20.0/512.0, 16.0/512.0, 4.0/512.0,
		-6.0/512.0, -24.0/512.0, 30.0/512.0, 0.0, 30.0/512.0, 24.0/512.0, 6.0/512.0,
		-4.0/512.0, -16.0/512.0, 20.0/512.0, 0.0, 20.0/512.0, 16.0/512.0, 4.0/512.0,
		-1.0/512.0,  -4.0/512.0, -5.0/512.0, 0.0,  5.0/512.0,  4.0/512.0, 1.0/512.0
	};
	return *((const TMatrix<double,5,7>*)kernel);
}

// Zero mean Gaussian.
inline double Gaussian(double x, double sigma) {
	return 1/sqrt(2*M_PI*sigma*sigma) * exp(-(x*x/2.0/sigma/sigma));
}
// 2D gaussian (zero mean)
// (see: (9) in http://mathworld.wolfram.com/GaussianFunction.html)
inline double Gaussian2D(double x, double y, double sigma) {
	return 1.0/(2.0*M_PI*sigma*sigma) * exp( -(x*x+y*y)/(2.0*sigma*sigma));
}
inline double GaussianDerivative(double x, double sigma) {
	return -x / sigma / sigma * Gaussian(x, sigma);
}
// Solve the inverse of the Gaussian for positive x.
inline double GaussianInversePositive(double y, double sigma) {
	return sqrt(-2.0 * sigma * sigma * log(y * sigma * sqrt(2.0*M_PI)));
}

// Compute the Gaussian kernel width corresponding to the given sigma.
inline int ComputeGaussianKernelWidth(double sigma) {
	ASSERT(sigma >= 0.0);
	// 0.004 implies a 3 pixel kernel with 1 pixel sigma.
	const double truncation_factor(0.004);
	// Calculate the kernel size based on sigma such that it is odd.
	const double precisehalfwidth(GaussianInversePositive(truncation_factor, sigma));
	int width = ROUND2INT(2*precisehalfwidth);
	if (width % 2 == 0)
		width++;
	return width;
}
// Compute a Gaussian kernel, such that you can compute the blurred image
// by convolving with the kernel horizontally then vertically.
inline void ComputeGaussianKernel(double sigma, DVector64F& kernel) {
	const int width(ComputeGaussianKernelWidth(sigma));
	// Calculate the gaussian kernel and its derivative.
	kernel.create(width);
	kernel.memset(0);
	const int halfwidth(width / 2);
	for (int i = -halfwidth; i <= halfwidth; ++i)
		kernel(i + halfwidth) = Gaussian(i, sigma);
	// Since images should not get brighter or darker, normalize.
	cv::normalize(kernel, kernel, 1.0, 0.0, cv::NORM_L1);
}
// Compute a Gaussian kernel and derivative, such that you can take the
// derivative of an image by convolving with the kernel horizontally then the
// derivative vertically to get (eg) the y derivative.
inline void ComputeGaussianKernel(double sigma, DVector64F& kernel, DVector64F& derivative) {
	const int width(ComputeGaussianKernelWidth(sigma));
	// Calculate the gaussian kernel and its derivative.
	kernel.create(width);
	derivative.create(width);
	kernel.memset(0);
	derivative.memset(0);
	const int halfwidth(width / 2);
	for (int i = -halfwidth; i <= halfwidth; ++i) {
		kernel(i + halfwidth) = Gaussian(i, sigma);
		derivative(i + halfwidth) = GaussianDerivative(i, sigma);
	}
	// Since images should not get brighter or darker, normalize.
	cv::normalize(kernel, kernel, 1.0, 0.0, cv::NORM_L1);
	// Normalize the derivative differently. See
	// www.cs.duke.edu/courses/spring03/cps296.1/handouts/Image%20Processing.pdf
	double factor(0);
	for (int i = -halfwidth; i <= halfwidth; ++i)  {
		factor -= derivative(i+halfwidth)*i;
	}
	derivative /= factor;
}

template <typename Type, int size, bool vertical>
void FastConvolve(const DVector64F& kernel, int width, int height,
				  const Type* src, int src_stride, int src_line_stride,
				  Type* dst, int dst_stride) {
	double coefficients[2 * size + 1];
	for (int k = 0; k < 2 * size + 1; ++k) {
		coefficients[k] = kernel(2 * size - k);
	}
	// Fast path: if the kernel has a certain size, use the constant sized loops.
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			double sum(0);
			for (int k = -size; k <= size; ++k) {
				if (vertical) {
					if (y + k >= 0 && y + k < height) {
						sum += double(src[k * src_line_stride]) * coefficients[k + size];
					}
				} else {
					if (x + k >= 0 && x + k < width) {
						sum += double(src[k * src_stride]) * coefficients[k + size];
					}
				}
			}
			dst[0] = static_cast<Type>(sum);
			src += src_stride;
			dst += dst_stride;
		}
	}
}
template <typename Type, bool vertical>
void Convolve(const TImage<Type>& in,
			  const DVector64F& kernel,
			  TImage<Type>& out) {
	ASSERT(kernel.rows % 2 == 1);
	ASSERT(&in != &out);

	const int width = in.width();
	const int height = in.height();
	out.create(in.size());

	const int src_line_stride = in.step1(0);
	const int src_stride = in.step1(1);
	const int dst_stride = out.step1(1);
	const Type* src = in.cv::Mat::ptr<Type>();
	Type* dst = out.cv::Mat::ptr<Type>();

	// Use a dispatch table to make most convolutions used in practice use the
	// fast path.
	const int half_width(kernel.rows / 2);
	switch (half_width) {
	#define static_convolution(size) case size: \
		FastConvolve<Type,size,vertical>(kernel, width, height, src, src_stride, \
			src_line_stride, dst, dst_stride); break;
	static_convolution(1)
	static_convolution(2)
	static_convolution(3)
	static_convolution(4)
	static_convolution(5)
	static_convolution(6)
	static_convolution(7)
	#undef static_convolution
	default:
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				double sum = 0;
				// Slow path: this loop cannot be unrolled.
				for (int k = -half_width; k <= half_width; ++k) {
					if (vertical) {
						if (y + k >= 0 && y + k < height) {
							sum += double(src[k * src_line_stride]) * kernel(2 * half_width - (k + half_width));
						}
					} else {
						if (x + k >= 0 && x + k < width) {
							sum += double(src[k * src_stride]) * kernel(2 * half_width - (k + half_width));
						}
					}
				}
				dst[0] = static_cast<Type>(sum);
				src += src_stride;
				dst += dst_stride;
			}
		}
	}
}
template <typename Type>
inline void ConvolveHorizontal(const TImage<Type>& in, const DVector64F& kernel, TImage<Type>& out) {
	Convolve<Type,false>(in, kernel, out);
}
template <typename Type>
inline void ConvolveVertical(const TImage<Type>& in, const DVector64F& kernel, TImage<Type>& out) {
	Convolve<Type,true>(in, kernel, out);
}

// Compute the gaussian blur of an image, and store the results in one channel.
template <typename Type>
void BlurredImage(const TImage<Type>& in, TImage<Type>& blurred, double sigma=0.32) {
	ASSERT(in.channels() == 1);
	DVector64F kernel;
	ComputeGaussianKernel(sigma, kernel);
	TImage<Type> tmp;
	// Compute convolved image.
	blurred.create(in.size());
	ConvolveVertical(in, kernel, tmp);
	ConvolveHorizontal(tmp, kernel, blurred);
}
// Compute the gaussian blur of an image and the derivatives of the blurred
// image, and store the results in three channels.
template <typename Type>
void BlurredImageAndDerivatives(const TImage<Type>& in, TImage<Type>& blurred, TImage<Type> grad[2], double sigma=0.32) {
	ASSERT(in.channels() == 1);
	DVector64F kernel, derivative;
	ComputeGaussianKernel(sigma, kernel, derivative);
	TImage<Type> tmp, dir[2];
	// Compute convolved image.
	blurred.create(in.size());
	ConvolveVertical(in, kernel, tmp);
	ConvolveHorizontal(tmp, kernel, blurred);
	// Compute first derivative in x.
	ConvolveHorizontal(tmp, derivative, grad[0]);
	// Compute first derivative in y.
	ConvolveHorizontal(in, kernel, tmp);
	ConvolveVertical(tmp, derivative, grad[1]);
}
template <typename Type>
void BlurredImageAndDerivatives(const TImage<Type>& in, TImage<Type>& blurred, TImage< TPoint2<Type> >& grad, double sigma=0.32) {
	TImage<Type> dir[2];
	BlurredImageAndDerivatives(in, blurred, grad, sigma);
	cv::merge(dir, 2, grad);
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

template <typename TYPE>
const int TBitMatrix<TYPE>::numBitsShift = log2i<TBitMatrix::numBitsPerCell>();
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

// Wavelet Constructor
// Supports both Separable wavelets and Red-Black wavelets
//
// Input: 
// dist_func - 0 for exp(-(I-J)^2/sigma^2) and 1 for 1/(|I-J|^sigma+eps)
// sigma - sets the range scale used in the pixel-range distance function
template <typename TYPE, int TABLE_LEN>
TWavelets<TYPE,TABLE_LEN>::TWavelets(DistanceType distType, TYPE alpha, TYPE eps) {
	// initiates distance table
	switch (distType) {
	case DT_EXP:
		// initiates distance table (exp(-d^2))
		for (int i = 0; i < TABLE_LEN; ++i) {
			const TYPE v = TYPE(4) * (((TYPE)i+TYPE(0.5)) / TABLE_LEN - TYPE(0.5));
			dist_table[i] = exp(-(SQUARE(v*alpha)));
		}
		break;
	case DT_INV:
		// initiates distance table (1/d)
		for (int i = 0; i < TABLE_LEN; ++i) {
			const TYPE v = TYPE(4) * (((TYPE)i+TYPE(0.5)) / TABLE_LEN - TYPE(0.5));
			dist_table[i] = pow(fabs(v) + eps, -alpha);
		}
		break;
	default:
		ASSERT("Unsupported distance type!" == NULL);
	}
}

template <typename TYPE, int TABLE_LEN>
inline TYPE TWavelets<TYPE,TABLE_LEN>::dist(TYPE v) const {
	if (v > TYPE(2) || v < TYPE(-2))
		return dist_table[0];
	const int i = (int)((v + TYPE(2)) * TYPE(0.25) * TABLE_LEN);
	return dist_table[i];
}

// Forward Wavelet Transform
//
// Input: 
// I - 2D image (must have its values between 0 and 1)
// nlevels - number of transformation levels (each is full octave)
// 
// Output: 
// A - (cell array) approximation and detail coefficients (approx. not really needed)
// W - (cell array) wavelets weights
template <typename TYPE, int TABLE_LEN>
template <class WAVELET_TYPE, typename PIXEL_TYPE>
void TWavelets<TYPE,TABLE_LEN>::ForwardTransform(const TImage<PIXEL_TYPE>& image, TDMatrix< TImage<typename WAVELET_TYPE::Type> >& A, TDMatrix< TImage<typename WAVELET_TYPE::Type> >& W, WAVELET_TYPE& wavelet, size_t nlevels)
{
	typedef TImage<TYPE> IMAGE_TYPE;
	const int nc = cv::DataType<PIXEL_TYPE>::channels;
	A.construct(nlevels+1, nc);
	W.construct(nlevels, nc);
	// extract image channels
	MatArr channels(nc);
	cv::split(image, channels.Begin());
	if (cv::DataType<typename cv::DataType<PIXEL_TYPE>::channel_type>::type < CV_32F) {
		// cast image channels to the working format
		FOREACHPTR(pImage, channels)
			pImage->convertTo(*pImage, cv::DataType<TYPE>::type, TYPE(1)/TYPE(255));
	}
	// decompose wavelet levels
	for (int c=0; c<nc; ++c) {
		cv::Mat& J = channels[c];
		for (int i=0; i<nlevels; ++i) {
			IMAGE_TYPE OA(J.size(), J.ptr<TYPE>());
			wavelet.Decompose(OA, A(i,c), W(i,c));
			cv::downsample2x(A(i,c), J);
		}
		J.copyTo(A(nlevels,c));
	}
}

// Backward Wavelet Transform
//
// Input:  
// A - (cell array) approximation and detail coefficients
// W - (cell array) wavelets weights
//
// Output:
// I - 2D image
template <typename TYPE, int TABLE_LEN>
template <class WAVELET_TYPE, typename PIXEL_TYPE>
void TWavelets<TYPE,TABLE_LEN>::BackwardTransform(const TDMatrix< TImage<typename WAVELET_TYPE::Type> >& A, const TDMatrix< TImage<typename WAVELET_TYPE::Type> >& W, TImage<PIXEL_TYPE>& image, WAVELET_TYPE& wavelet)
{
	typedef TImage<TYPE> IMAGE_TYPE;
	const int nc = A.cols;
	const int nlevels = W.rows;
	// compose wavelet levels
	MatArr channels(nc);
	for (int c=0; c<nc; ++c) {
		cv::Mat& J = channels[c];
		J = A(nlevels,c);
		for (int i=nlevels-1; i>=0; --i) {
			cv::Mat nextJ(A(i,c));
			cv::upsample2x(J, nextJ);
			IMAGE_TYPE A(nextJ.size(), nextJ.ptr<TYPE>());
			J.create(nextJ.size(), cv::DataType<TYPE>::type);
			IMAGE_TYPE OA(J.size(), J.ptr<TYPE>());
			wavelet.Compose(A, W(i,c), OA);
		}
	}
	if (cv::DataType<typename cv::DataType<PIXEL_TYPE>::channel_type>::type < CV_32F) {
		// cast image channels to the desired format
		FOREACHPTR(pImage, channels) {
			cv::Mat& channel = *pImage;
			//cv::min(cv::max(channel, 0), 1, channel);
			channel.convertTo(channel, cv::DataType<PIXEL_TYPE>::type, TYPE(255));
		}
	}
	// compose image channels
	image.create(channels[0].size());
	cv::merge(channels.Begin(), nc, cv::Mat(image.rows, image.cols, cv::DataType<PIXEL_TYPE>::type, (void*)image.data));
}
/*----------------------------------------------------------------*/

#define UPDT TYPE(0.5)

/***************************************************/
/*             Separable Wavelets                  */
/***************************************************/

template <typename TYPE, int TABLE_LEN>
void TSeparableWavelets<TYPE,TABLE_LEN>::Decompose(const TImage<TYPE>& OA, TImage<TYPE>& A, TImage<TYPE>& W) const {
	const int nx = OA.rows;
	const int ny = OA.cols;

	const int nxm = 2 * ((nx-1)/2);
	const int nym = 2 * ((ny-1)/2);

	TYPE w1, w2, w3, w4, sw;
	TYPE A1, A2, A3, A4;
	TYPE Axy;

	OA.copyTo(A);
	W.create(nx*ny*3,1);

	int i = 0;
	for (int y = 0; y < nym; y+=2) {
		for (int x = 0; x < nxm; x+=2) {
			Axy = A(x+1,y);

			A1 = A(x,y);
			A2 = A(x+2,y);

			w1 = dist(A1-Axy);
			w2 = dist(A2-Axy);
			sw = TYPE(1) / (w1 + w2);

			w1 *= sw;
			w2 *= sw;
			W[i++] = w1;
			W[i++] = w2;

			A(x+1,y) = Axy - (w1 * A1 + w2 * A2);

			Axy = A(x,y+1);
			// A1 = A(x,y);
			A2 = A(x,y+2);

			w1 = dist(A1-Axy);
			w2 = dist(A2-Axy);
			sw = TYPE(1) / (w1 + w2);

			w1 *= sw;
			w2 *= sw;
			W[i++] = w1;
			W[i++] = w2;

			A(x,y+1) = Axy - (w1 * A1 + w2 * A2);

			Axy = A(x+1,y+1);
			// A1 = A(x,y);
			// A2 = A(x,y+2);
			A3 = A(x+2,y);
			A4 = A(x+2,y+2);

			w1 = dist(A1-Axy);
			w2 = dist(A2-Axy);
			w3 = dist(A3-Axy);
			w4 = dist(A4-Axy);

			sw = TYPE(1) / (w1 + w2 + w3 + w4);

			w1 *= sw;
			w2 *= sw;
			w3 *= sw;
			w4 *= sw;

			W[i++] = w1;
			W[i++] = w2;
			W[i++] = w3;
			W[i++] = w4;

			A(x+1,y+1) = Axy - (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
		}

		Axy = A(nxm,y);
		A(nxm,y+1) = A(nxm,y+1) - Axy;

		if (nxm + 1 < nx) {
			A(nxm+1,y) = A(nxm+1,y) - Axy;
			A(nxm+1,y+1) = A(nxm+1,y+1) - Axy;
		}
	}

	for (int x = 0; x < nxm+1; x+=2) {
		Axy = A(x,nym);

		if (x + 1 < nx)
			A(x+1,nym) = A(x+1,nym) - Axy;

		if (nym + 1 < ny) {
			A(x,nym+1) = A(x,nym+1) - Axy;

			if (x + 1 < nx)
				A(x+1,nym+1) = A(x+1,nym+1) - Axy;
		}
	}

	i = nx * ny * 2;

	nxm = nx - 1;
	nym = ny - 1;

	for (int y = 2; y < nym; y+=2)
		for (int x = 2; x < nxm; x+=2) {
			Axy = OA(x,y);

			w1 = dist(OA(x+1,y)-Axy);
			w2 = dist(OA(x-1,y)-Axy);
			w3 = dist(OA(x,y+1)-Axy);
			w4 = dist(OA(x,y-1)-Axy);

			sw = UPDT / (w1 + w2 + w3 + w4);

			w1 *= sw;
			w2 *= sw;
			w3 *= sw;
			w4 *= sw;

			W[i++] = w1;
			W[i++] = w2;
			W[i++] = w3;
			W[i++] = w4;

			A(x,y) += (w1 * A(x+1,y) + w2 * A(x-1,y) + w3 * A(x,y+1) + w4 * A(x,y-1));
		}
}

template <typename TYPE, int TABLE_LEN>
void TSeparableWavelets<TYPE,TABLE_LEN>::Compose(const TImage<TYPE>& OA, const TImage<TYPE>& W, TImage<TYPE>& A) const {
	const int nx = OA.rows;
	const int ny = OA.cols;

	const int nxm = nx - 1;
	const int nym = ny - 1;

	TYPE w1, w2, w3, w4;
	TYPE A1, A2, A3, A4;
	TYPE Axy, t;

	OA.copyTo(A);

	int i = nx * ny * 2;
	for (int y = 2; y < nym; y+=2)
		for (int x = 2; x < nxm; x+=2) {
			t = W[i++] * A(x+1,y);
			t += W[i++] * A(x-1,y);
			t += W[i++] * A(x,y+1);
			t += W[i++] * A(x,y-1);

			A(x,y) += t;
		}

		i = 0;

		nxm = 2 * ((nx-1)/2);
		nym = 2 * ((ny-1)/2);

		for (int y = 0; y < nym; y+=2) {
			for (int x = 0; x < nxm; x+=2) {
				Axy = A(x+1,y);
				A1 = A(x,y);
				A2 = A(x+2,y);

				w1 = W[i++];
				w2 = W[i++];

				A(x+1,y) = Axy + (w1 * A1 + w2 * A2);

				Axy = A(x,y+1);
				// A1 = A(x,y);
				A2 = A(x,y+2);

				w1 = W[i++];
				w2 = W[i++];

				A(x,y+1) = Axy + (w1 * A1 + w2 * A2);

				Axy = A(x+1,y+1);
				// A1 = A(x,y);
				// A2 = A(x,y+2);
				A3 = A(x+2,y);
				A4 = A(x+2,y+2);

				w1 = W[i++];
				w2 = W[i++];
				w3 = W[i++];
				w4 = W[i++];

				A(x+1,y+1) = Axy + (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			}

			Axy = A(nxm,y);
			A(nxm,y+1) = A(nxm,y+1) + Axy;

			if (nxm + 1 < nx) {
				A(nxm+1,y) = A(nxm+1,y) + Axy;
				A(nxm+1,y+1) = A(nxm+1,y+1) + Axy;
			}
		}

		for (int x = 0; x < nxm+1; x+=2) {
			Axy = A(x,nym);

			if (x + 1 < nx)
				A(x+1,nym) = A(x+1,nym) + Axy;

			if (nym + 1 < ny) {
				A(x,nym+1) = A(x,nym+1) + Axy;

				if (x + 1 < nx)
					A(x+1,nym+1) = A(x+1,nym+1) + Axy;
			}
		}
}
/*----------------------------------------------------------------*/


/***************************************************/
/*             Red-Black Wavelets                  */
/***************************************************/

template <typename TYPE, int TABLE_LEN>
void TRedBlackWavelets<TYPE,TABLE_LEN>::Decompose(const TImage<TYPE>& OA, TImage<TYPE>& A, TImage<TYPE>& W) const {
	const int nx = OA.rows;
	const int ny = OA.cols;

	const int nxm = nx - 1;
	const int nym = ny - 1;

	TYPE w1, w2, w3, w4, sw;
	TYPE A1=0, A2=0, A3=0, A4=0;
	TYPE Axy;

	OA.copyTo(A);
	W.create(nx*ny*8,1);

	// PREDICT I
	int i = 0;
	for (int y = 0; y < ny; y++) {
		for (int x = 0; x < nx; x++) {
			if ((x+y) % 2 == 0)
				continue;

			if (x > 0 && y > 0 && x < nxm && y < nym) {
				Axy = A(x,y);
				A1 = A(x+1,y);
				A2 = A(x-1,y);
				A3 = A(x,y+1);
				A4 = A(x,y-1);

				w1 = dist(A1-Axy);
				w2 = dist(A2-Axy);
				w3 = dist(A3-Axy);
				w4 = dist(A4-Axy);

				sw = TYPE(1) / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				W[i++] = w1;
				W[i++] = w2;
				W[i++] = w3;
				W[i++] = w4;

				A(x,y) -= (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				Axy = A(x,y);

				if (x + 1 < nx) {
					A1 = A(x+1,y);
					w1 = dist(A1-Axy);
				}
				else
					w1 = 0;

				if (x - 1 >= 0) {
					A2 = A(x-1,y);
					w2 = dist(A2-Axy);
				}
				else
					w2 = 0;

				if (y + 1 < ny) {
					A3 = A(x,y+1);
					w3 = dist(A3-Axy);
				}
				else
					w3 = 0;

				if (y - 1 >= 0) {
					A4 = A(x,y-1);
					w4 = dist(A4-Axy);
				}
				else
					w4 = 0;

				sw = TYPE(1) / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				if (x + 1 < nx) {
					W[i++] = w1;
					A(x,y) -= w1 * A1;
				}

				if (x - 1 >= 0) {
					W[i++] = w2;
					A(x,y) -= w2 * A2;
				}

				if (y + 1 < ny) {
					W[i++] = w3;
					A(x,y) -= w3 * A3;
				}

				if (y - 1 >= 0) {
					W[i++] = w4;
					A(x,y) -= w4 * A4;
				}
			}
		}
	}

	i = 2 * nx * ny;


	// UPDATE I
	for (int y = 0; y < ny; y++) {
		for (int x = 0; x < nx; x++) {
			if ((x+y) % 2 == 1)
				continue;

			if (x > 0 && y > 0 && x < nxm && y < nym) {
				Axy = OA(x,y); // needed for weights
				A1 = OA(x+1,y);
				A2 = OA(x-1,y);
				A3 = OA(x,y+1);
				A4 = OA(x,y-1);

				w1 = dist(A1-Axy);
				w2 = dist(A2-Axy);
				w3 = dist(A3-Axy);
				w4 = dist(A4-Axy);

				sw = UPDT / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				W[i++] = w1;
				W[i++] = w2;
				W[i++] = w3;
				W[i++] = w4;

				A1 = A(x+1,y);
				A2 = A(x-1,y);
				A3 = A(x,y+1);
				A4 = A(x,y-1);

				A(x,y) += (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				Axy = OA(x,y);

				if (x + 1 < nx) {
					A1 = OA(x+1,y);
					w1 = dist(A1-Axy);
				}
				else
					w1 = 0;

				if (x - 1 >= 0) {
					A2 = OA(x-1,y);
					w2 = dist(A2-Axy);
				}
				else
					w2 = 0;

				if (y + 1 < ny) {
					A3 = OA(x,y+1);
					w3 = dist(A3-Axy);
				}
				else
					w3 = 0;

				if (y - 1 >= 0) {
					A4 = OA(x,y-1);
					w4 = dist(A4-Axy);
				}
				else
					w4 = 0;

				sw = UPDT / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				if (x + 1 < nx) {
					W[i++] = w1;
					A(x,y) += w1 * A(x+1,y);
				}

				if (x - 1 >= 0) {
					W[i++] = w2;
					A(x,y) += w2 * A(x-1,y);
				}

				if (y + 1 < ny) {
					W[i++] = w3;
					A(x,y) += w3 * A(x,y+1);
				}

				if (y - 1 >= 0) {
					W[i++] = w4;
					A(x,y) += w4 * A(x,y-1);
				}
			}
		}
	}

	i = 4 * nx * ny;

	// PREDICT II
	for (int y = 1; y < ny; y+=2) {
		for (int x = 1; x < nx; x+=2) {
			if (x > 0 && y > 0 && x < nxm && y < nym) {
				Axy = A(x,y);
				A1 = A(x-1,y+1);
				A2 = A(x-1,y-1);
				A3 = A(x+1,y+1);
				A4 = A(x+1,y-1);

				w1 = dist(A1-Axy);
				w2 = dist(A2-Axy);
				w3 = dist(A3-Axy);
				w4 = dist(A4-Axy);

				sw = TYPE(1) / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				W[i++] = w1;
				W[i++] = w2;
				W[i++] = w3;
				W[i++] = w4;

				A(x,y) -= (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				Axy = A(x,y);

				if (x - 1 >=0 && y + 1 < ny)
					w1 = dist(A(x-1,y+1)-Axy);
				else
					w1 = 0;

				if (x - 1 >=0 && y - 1 >= 0)
					w2 = dist(A(x-1,y-1)-Axy);
				else
					w2 = 0;

				if (x + 1 < nx && y + 1 < ny)
					w3 = dist(A(x+1,y+1)-Axy);
				else
					w3 = 0;

				if (x + 1 < nx && y - 1 >= 0)
					w4 = dist(A(x+1,y-1)-Axy);
				else
					w4 = 0;

				sw = TYPE(1) / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				if (x - 1 >=0 && y + 1 < ny) {
					W[i++] = w1;
					A(x,y) -= w1 * A(x-1,y+1);
				}

				if (x - 1 >=0 && y - 1 >= 0) {
					W[i++] = w2;
					A(x,y) -= w2 * A(x-1,y-1);
				}

				if (x + 1 < nx && y + 1 < ny) {
					W[i++] = w3;
					A(x,y) -= w3 * A(x+1,y+1);
				}

				if (x + 1 < nx && y - 1 >= 0) {
					W[i++] = w4;
					A(x,y) -= w4 * A(x+1,y-1);
				}
			}
		}
	}

	i = 6 * nx * ny;

	// UPDATE II
	for (int y = 0; y < ny; y+=2) {
		for (int x = 0; x < nx; x+=2) {
			if (x > 0 && y > 0 && x < nxm && y < nym) {
				Axy = OA(x,y);
				A1 = OA(x-1,y+1);
				A2 = OA(x-1,y-1);
				A3 = OA(x+1,y+1);
				A4 = OA(x+1,y-1);

				w1 = dist(A1-Axy);
				w2 = dist(A2-Axy);
				w3 = dist(A3-Axy);
				w4 = dist(A4-Axy);

				sw = UPDT / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				W[i++] = w1;
				W[i++] = w2;
				W[i++] = w3;
				W[i++] = w4;

				A1 = A(x-1,y+1);
				A2 = A(x-1,y-1);
				A3 = A(x+1,y+1);
				A4 = A(x+1,y-1);

				A(x,y) += (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				Axy = OA(x,y);

				if (x - 1 >=0 && y + 1 < ny)
					w1 = dist(OA(x-1,y+1)-Axy);
				else
					w1 = 0;

				if (x - 1 >=0 && y - 1 >= 0)
					w2 = dist(OA(x-1,y-1)-Axy);
				else
					w2 = 0;

				if (x + 1 < nx && y + 1 < ny)
					w3 = dist(OA(x+1,y+1)-Axy);
				else
					w3 = 0;

				if (x + 1 < nx && y - 1 >= 0)
					w4 = dist(OA(x+1,y-1)-Axy);
				else
					w4 = 0;

				sw = UPDT / (w1 + w2 + w3 + w4);

				w1 *= sw;
				w2 *= sw;
				w3 *= sw;
				w4 *= sw;

				if (x - 1 >=0 && y + 1 < ny) {
					W[i++] = w1;
					A(x,y) += w1 * A(x-1,y+1);
				}

				if (x - 1 >=0 && y - 1 >= 0) {
					W[i++] = w2;
					A(x,y) += w2 * A(x-1,y-1);
				}

				if (x + 1 < nx && y + 1 < ny) {
					W[i++] = w3;
					A(x,y) += w3 * A(x+1,y+1);
				}

				if (x + 1 < nx && y - 1 >= 0) {
					W[i++] = w4;
					A(x,y) += w4 * A(x+1,y-1);
				}
			}
		}
	}
}

template <typename TYPE, int TABLE_LEN>
void TRedBlackWavelets<TYPE,TABLE_LEN>::Compose(const TImage<TYPE>& OA, const TImage<TYPE>& W, TImage<TYPE>& A) const {
	const int nx = OA.rows;
	const int ny = OA.cols;

	const int nxm = nx - 1;
	const int nym = ny - 1;

	TYPE w1, w2, w3, w4;
	TYPE A1=0, A2=0, A3=0, A4=0;

	OA.copyTo(A);

	int i = 6 * nx * ny;

	// UPDATE II
	for (int y = 0; y < ny; y+=2) {
		for (int x = 0; x < nx; x+=2) {
			if (x > 0 && y > 0 && x < nxm && y < nym) {
				w1 = W[i++];
				w2 = W[i++];
				w3 = W[i++];
				w4 = W[i++];

				A1 = A(x-1,y+1);
				A2 = A(x-1,y-1);
				A3 = A(x+1,y+1);
				A4 = A(x+1,y-1);

				A(x,y) -= (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				if (x - 1 >=0 && y + 1 < ny) {
					w1 = W[i++];
					A(x,y) -= w1 * A(x-1,y+1);
				}

				if (x - 1 >=0 && y - 1 >= 0) {
					w2 = W[i++];
					A(x,y) -= w2 * A(x-1,y-1);
				}

				if (x + 1 < nx && y + 1 < ny) {
					w3 = W[i++];
					A(x,y) -= w3 * A(x+1,y+1);
				}

				if (x + 1 < nx && y - 1 >= 0) {
					w4 = W[i++];
					A(x,y) -= w4 * A(x+1,y-1);
				}
			}
		}
	}

	i = 4 * nx * ny;

	// PREDICT II
	for (int y = 1; y < ny; y+=2) {
		for (int x = 1; x < nx; x+=2) {
			if (x > 0 && y > 0 && x < nxm && y < nym) {
				w1 = W[i++];
				w2 = W[i++];
				w3 = W[i++];
				w4 = W[i++];

				A1 = A(x-1,y+1);
				A2 = A(x-1,y-1);
				A3 = A(x+1,y+1);
				A4 = A(x+1,y-1);

				A(x,y) += (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				if (x - 1 >=0 && y + 1 < ny) {
					w1 = W[i++];
					A(x,y) += w1 * A(x-1,y+1);
				}

				if (x - 1 >=0 && y - 1 >= 0) {
					w2 = W[i++];
					A(x,y) += w2 * A(x-1,y-1);
				}

				if (x + 1 < nx && y + 1 < ny) {
					w3 = W[i++];
					A(x,y) += w3 * A(x+1,y+1);
				}

				if (x + 1 < nx && y - 1 >= 0) {
					w4 = W[i++];
					A(x,y) += w4 * A(x+1,y-1);
				}
			}
		}
	}

	i = 2 * nx * ny;

	// UPDATE I
	for (int y = 0; y < ny; y++) {
		for (int x = 0; x < nx; x++) {
			if ((x+y) % 2 == 1)
				continue;

			if (x > 0 && y > 0 && x < nxm && y < nym) {
				w1 = W[i++];
				w2 = W[i++];
				w3 = W[i++];
				w4 = W[i++];

				A1 = A(x+1,y);
				A2 = A(x-1,y);
				A3 = A(x,y+1);
				A4 = A(x,y-1);

				A(x,y) -= (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				if (x + 1 < nx) {
					w1 = W[i++];
					A(x,y) -= w1 * A(x+1,y);
				}

				if (x - 1 >= 0) {
					w2 = W[i++];
					A(x,y) -= w2 * A(x-1,y);
				}

				if (y + 1 < ny) {
					w3 = W[i++];
					A(x,y) -= w3 * A(x,y+1);
				}

				if (y - 1 >= 0) {
					w4 = W[i++];
					A(x,y) -= w4 * A(x,y-1);
				}
			}
		}
	}

	// PREDICT I
	i = 0;
	for (int y = 0; y < ny; y++) {
		for (int x = 0; x < nx; x++) {
			if ((x+y) % 2 == 0)
				continue;

			if (x > 0 && y > 0 && x < nxm && y < nym) {
				A1 = A(x+1,y);
				A2 = A(x-1,y);
				A3 = A(x,y+1);
				A4 = A(x,y-1);

				w1 = W[i++];
				w2 = W[i++];
				w3 = W[i++];
				w4 = W[i++];

				A(x,y) += (w1 * A1 + w2 * A2 + w3 * A3 + w4 * A4);
			} else {
				if (x + 1 < nx) {
					w1 = W[i++];
					A(x,y) += w1 * A(x+1,y);
				}

				if (x - 1 >= 0) {
					w2 = W[i++];
					A(x,y) += w2 * A(x-1,y);
				}

				if (y + 1 < ny) {
					w3 = W[i++];
					A(x,y) += w3 * A(x,y+1);
				}

				if (y - 1 >= 0) {
					w4 = W[i++];
					A(x,y) += w4 * A(x,y-1);
				}
			}
		}
	}
}
/*----------------------------------------------------------------*/

#undef UPDT


// returns determinant, note that if it's zero no inversion done;
// use Mi == M to store result into M
template<typename TYPE>
TYPE InvertMatrix3x3(const TYPE* m, TYPE* mi) {
	const TYPE d(m[0]*(m[4]*m[8]-m[5]*m[7])+m[1]*(m[5]*m[6]-m[3]*m[8])+m[2]*(m[3]*m[7]-m[4]*m[6]));
	if (d == TYPE(0))
		return TYPE(0);
	const TYPE invd = TYPE(1)/d;

	TYPE mc[9];
	const TYPE* mm = m;
	if (mi == m) {
		memcpy(mc, m, sizeof(TYPE)*9);
		mm = mc;
	}

	mi[0] = (mm[4]*mm[8]-mm[5]*mm[7])*invd;
	mi[1] = (mm[2]*mm[7]-mm[1]*mm[8])*invd;
	mi[2] = (mm[1]*mm[5]-mm[2]*mm[4])*invd;
	mi[3] = (mm[5]*mm[6]-mm[3]*mm[8])*invd;
	mi[4] = (mm[0]*mm[8]-mm[2]*mm[6])*invd;
	mi[5] = (mm[2]*mm[3]-mm[0]*mm[5])*invd;
	mi[6] = (mm[3]*mm[7]-mm[4]*mm[6])*invd;
	mi[7] = (mm[1]*mm[6]-mm[0]*mm[7])*invd;
	mi[8] = (mm[0]*mm[4]-mm[1]*mm[3])*invd;

	return d;
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE


// C L A S S  //////////////////////////////////////////////////////

#ifdef _USE_EIGEN

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
inline void eigen_so3_exp(const Eigen::Matrix<Precision,3,1>& w, Eigen::Matrix<Precision,3,3>& R) {
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
			const Precision theta(sqrt(theta_sq));
			const Precision inv_theta(Precision(1)/theta);
			A = sin(theta) * inv_theta;
			B = (Precision(1) - cos(theta)) * (inv_theta * inv_theta);
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
inline Eigen::SO3<Precision> Eigen::SO3<Precision>::exp(const Vec3& w) {
	SO3<Precision> result;
	eigen_so3_exp(w, result.my_matrix);
	return result;
}

/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
/// See the Detailed Description for details of this vector.
template <typename Precision>
inline void eigen_so3_ln(const Eigen::Matrix<Precision,3,3>& R, Eigen::Matrix<Precision,3,1>& w) {
	const Precision cos_angle((R(0,0) + R(1,1) + R(2,2) - Precision(1)) * Precision(0.5));
	w(0) = (R(2,1)-R(1,2))*Precision(0.5);
	w(1) = (R(0,2)-R(2,0))*Precision(0.5);
	w(2) = (R(1,0)-R(0,1))*Precision(0.5);

	const Precision sin_angle_abs(sqrt(w.squaredNorm()));
	if (cos_angle > Precision(M_SQRT1_2)) {           // [0 - Pi/4] use asin
		if (sin_angle_abs > Precision(0))
			w *= asin(sin_angle_abs) / sin_angle_abs;
	} else if (cos_angle > Precision(-M_SQRT1_2)) {   // [Pi/4 - 3Pi/4] use acos, but antisymmetric part
		if (sin_angle_abs > Precision(0))
			w *= acos(cos_angle) / sin_angle_abs;        
	} else {                                       // rest use symmetric part
		// antisymmetric part vanishes, but still large rotation, need information from symmetric part
		const Precision angle(Precision(M_PI) - asin(sin_angle_abs));
		const Precision d0(R(0,0) - cos_angle);
		const Precision d1(R(1,1) - cos_angle);
		const Precision d2(R(2,2) - cos_angle);
		Eigen::Matrix<Precision,3,1> r2;
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
inline Eigen::Matrix<Precision,3,1> Eigen::SO3<Precision>::ln() const {
	Vec3 result;
	eigen_so3_ln(my_matrix, result);
	return result;
}

/// Write an SO3 to a stream 
/// @relates SO3
template <typename Precision>
inline std::ostream& operator<<(std::ostream& os, const Eigen::SO3<Precision>& rhs) {
	return os << rhs.get_matrix();
}
/// Read from SO3 to a stream 
/// @relates SO3
template <typename Precision>
inline std::istream& operator>>(std::istream& is, Eigen::SO3<Precision>& rhs) {
	is >> rhs.my_matrix;
	rhs.coerce();
	return is;
}

/// Right-multiply by a Vector
/// @relates SO3
template<typename P>
inline Eigen::Matrix<P,3,1> operator*(const Eigen::SO3<P>& lhs, const Eigen::Matrix<P,3,1>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Vector
/// @relates SO3
template<typename P>
inline Eigen::Matrix<P,3,1> operator*(const Eigen::Matrix<P,3,1>& lhs, const Eigen::SO3<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/// Right-multiply by a matrix
/// @relates SO3
template<typename P, int C>
inline Eigen::Matrix<P,3,C> operator*(const Eigen::SO3<P>& lhs, const Eigen::Matrix<P,3,C>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a matrix
/// @relates SO3
template<typename P, int R>
inline Eigen::Matrix<P,R,3> operator*(const Eigen::Matrix<P,R,3>& lhs, const Eigen::SO3<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/*----------------------------------------------------------------*/


/// Exponentiate an angle in the Lie algebra to generate a new SO2.
template <typename Precision>
inline void eigen_so2_exp(const Precision& d, Eigen::Matrix<Precision,2,2>& R) {
	R(0,0) = R(1,1) = cos(d);
	R(1,0) = sin(d);
	R(0,1) = -R(1,0);
}
template <typename Precision>
inline Eigen::SO2<Precision> Eigen::SO2<Precision>::exp(const Precision& d) {
	SO2<Precision> result;
	eigen_so2_exp(d, result.my_matrix);
	return result;
}

/// Extracts the rotation angle from the SO2
template <typename Precision>
inline void eigen_so2_ln(const Eigen::Matrix<Precision,2,2>& R, Precision& d) {
	d = atan2(R(1,0), R(0,0));
}
template <typename Precision>
Precision Eigen::SO2<Precision>::ln() const {
	Precision d;
	eigen_so2_ln(my_matrix, d);
	return d;
}

/// Write an SO2 to a stream 
/// @relates SO2
template <typename Precision>
inline std::ostream& operator<<(std::ostream& os, const Eigen::SO2<Precision> & rhs) {
	return os << rhs.get_matrix();
}
/// Read from SO2 to a stream 
/// @relates SO2
template <typename Precision>
inline std::istream& operator>>(std::istream& is, Eigen::SO2<Precision>& rhs) {
	is >> rhs.my_matrix;
	rhs.coerce();
	return is;
}

/// Right-multiply by a Vector
/// @relates SO2
template<typename P>
inline Eigen::Matrix<P,2,1> operator*(const Eigen::SO2<P>& lhs, const Eigen::Matrix<P,2,1>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Vector
/// @relates SO2
template<typename P>
inline Eigen::Matrix<P,2,1> operator*(const Eigen::Matrix<P,2,1>& lhs, const Eigen::SO2<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/// Right-multiply by a Matrix
/// @relates SO2
template <typename P, int C> 
inline Eigen::Matrix<P,2,C> operator*(const Eigen::SO2<P>& lhs, const Eigen::Matrix<P,2,C>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Matrix
/// @relates SO2
template <typename P, int R>
inline Eigen::Matrix<P,R,2> operator*(const Eigen::Matrix<P,R,2>& lhs, const Eigen::SO2<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/*----------------------------------------------------------------*/

#endif // _USE_EIGEN


// C L A S S  //////////////////////////////////////////////////////

#ifdef _USE_BOOST

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)

namespace boost {
	namespace serialization {

		// Serialization support for cv::Mat
		template<class Archive>
		void save(Archive& ar, const cv::Mat& m, const unsigned int /*version*/)
		{
			const int elem_type = m.type();
			const size_t elem_size = m.elemSize();

			ar & m.cols;
			ar & m.rows;
			ar & elem_type;
			ar & elem_size;

			const size_t data_size = elem_size * m.cols * m.rows;
			if (m.isContinuous()) {
				ar & boost::serialization::make_array(m.ptr(), data_size);
			} else {
				cv::Mat m_cont;
				m.copyTo(m_cont);
				ar & boost::serialization::make_array(m_cont.ptr(), data_size);
			}
		}
		template<class Archive>
		void load(Archive& ar, cv::Mat& m, const unsigned int /*version*/)
		{
			int cols, rows, elem_type;
			size_t elem_size;

			ar & cols;
			ar & rows;
			ar & elem_type;
			ar & elem_size;

			m.create(rows, cols, elem_type);

			const size_t data_size = elem_size * m.cols * m.rows;
			ar & boost::serialization::make_array(m.ptr(), data_size);
		}

		// Serialization support for cv::Mat_
		template<class Archive, typename _Tp>
		void save(Archive& ar, const cv::Mat_<_Tp>& m, const unsigned int /*version*/)
		{
			ar & m.cols;
			ar & m.rows;

			const size_t data_size = m.cols * m.rows;
			if (m.isContinuous()) {
				ar & boost::serialization::make_array((const _Tp*)m.ptr(), data_size);
			} else {
				cv::Mat_<_Tp> m_cont;
				m.copyTo(m_cont);
				ar & boost::serialization::make_array((const _Tp*)m_cont.ptr(), data_size);
			}
		}
		template<class Archive, typename _Tp>
		void load(Archive& ar, cv::Mat_<_Tp>& m, const unsigned int /*version*/)
		{
			int cols, rows;
			ar & cols;
			ar & rows;

			m.create(rows, cols);

			const size_t data_size = m.cols * m.rows;
			for (size_t n=0; n<data_size; ++n)
				new((_Tp*)m.data+n) _Tp;
			ar & boost::serialization::make_array((_Tp*)m.ptr(), data_size);
		}
		template<class Archive, typename _Tp>
		inline void serialize(Archive& ar, cv::Mat_<_Tp>& t, const unsigned int version) {
			split_free(ar, t, version);
		}

		// Serialization support for cv::Matx
		template<class Archive, typename _Tp, int m, int n>
		void serialize(Archive& ar, cv::Matx<_Tp, m, n>& _m, const unsigned int /*version*/) {
			ar & _m.val;
		}

		// Serialization support for cv::Vec
		template<class Archive, typename _Tp, int cn>
		void serialize(Archive& ar, cv::Vec<_Tp, cn>& v, const unsigned int /*version*/) {
			ar & boost::serialization::base_object<cv::Matx<_Tp, cn, 1> >(v);
		}

		// Serialization support for cv::Point_
		template<class Archive, typename _Tp>
		void serialize(Archive& ar, cv::Point_<_Tp>& pt, const unsigned int /*version*/) {
			ar & pt.x & pt.y;
		}

		// Serialization support for cv::Point3_
		template<class Archive, typename _Tp>
		void serialize(Archive& ar, cv::Point3_<_Tp>& pt, const unsigned int /*version*/) {
			ar & pt.x & pt.y & pt.z;
		}

		// Serialization support for cv::Size_
		template<class Archive, typename _Tp>
		void serialize(Archive& ar, cv::Size_<_Tp>& sz, const unsigned int /*version*/) {
			ar & sz.width & sz.height;
		}

		// Serialization support for cv::Rect_
		template<class Archive, typename _Tp>
		void serialize(Archive& ar, cv::Rect_<_Tp>& rc, const unsigned int /*version*/) {
			ar & rc.x & rc.y & rc.width & rc.height;
		}

		// Serialization support for cv::KeyPoint
		template<class Archive>
		void serialize(Archive& ar, cv::KeyPoint& k, const unsigned int /*version*/) {
			ar & k.pt;
			ar & k.size;
			ar & k.angle;
			ar & k.response;
			ar & k.octave;
			ar & k.class_id;
		}

		// Serialization support for cv::DMatch
		template<class Archive>
		void serialize(Archive& ar, cv::DMatch& m, const unsigned int /*version*/) {
			ar & m.queryIdx;
			ar & m.trainIdx;
			ar & m.imgIdx;
			ar & m.distance;
		}

	} // namespace serialization
} // namespace boost
/*----------------------------------------------------------------*/

// include headers that implement a archive in simple text and binary format or XML format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
//#include <boost/archive/xml_oarchive.hpp>
//#include <boost/archive/xml_iarchive.hpp>
// include headers that implement compressed serialization support
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>

enum ARCHIVE_TYPE {
	ARCHIVE_TEXT = 0,
	ARCHIVE_BINARY,
	ARCHIVE_BINARY_ZIP,
	ARCHIVE_LAST
};

// export the current state of the given reconstruction object
template <typename TYPE>
bool SerializeSave(const TYPE& obj, std::ofstream& fs, ARCHIVE_TYPE type, unsigned flags=0)
{
	// serialize out the current state
	switch (type) {
	case ARCHIVE_TEXT: {
		boost::archive::text_oarchive ar(fs, flags);
		ar << obj;
		break; }
	case ARCHIVE_BINARY: {
		boost::archive::binary_oarchive ar(fs, flags);
		ar << obj;
		break; }
	case ARCHIVE_BINARY_ZIP: {
		namespace io = boost::iostreams;
		io::filtering_streambuf<io::output> ffs;
		ffs.push(io::zlib_compressor(io::zlib::best_speed));
		ffs.push(fs);
		boost::archive::binary_oarchive ar(ffs, flags);
		ar << obj;
		break; }
	default:
		VERBOSE("error: Can not save the object, invalid archive type");
		return false;
	}
	return true;
} // SerializeSave
template <typename TYPE>
bool SerializeSave(const TYPE& obj, const SEACAVE::String& fileName, ARCHIVE_TYPE type, unsigned flags=0)
{
	// open the output stream
	std::ofstream fs(fileName, std::ios::out | std::ios::binary);
	if (!fs.is_open())
		return false;
	// serialize out the current state
	return SerializeSave(obj, fs, type, flags);
} // SerializeSave

// import the state to the given reconstruction object
template <typename TYPE>
bool SerializeLoad(TYPE& obj, std::ifstream& fs, ARCHIVE_TYPE type, unsigned flags=0)
{
	try {
		// serialize in the saved state
		switch (type) {
		case ARCHIVE_TEXT: {
			boost::archive::text_iarchive ar(fs, flags);
			ar >> obj;
			break; }
		case ARCHIVE_BINARY: {
			boost::archive::binary_iarchive ar(fs, flags);
			ar >> obj;
			break; }
		case ARCHIVE_BINARY_ZIP: {
			namespace io = boost::iostreams;
			io::filtering_streambuf<io::input> ffs;
			ffs.push(io::zlib_decompressor());
			ffs.push(fs);
			boost::archive::binary_iarchive ar(ffs, flags);
			ar >> obj;
			break; }
		default:
			VERBOSE("error: Can not load the object, invalid archive type");
			return false;
		}
	}
	catch (const std::exception& e) {
		VERBOSE("error: invalid stream (%s)", e.what());
		return false;
	}
	return true;
} // SerializeLoad
template <typename TYPE>
bool SerializeLoad(TYPE& obj, const SEACAVE::String& fileName, ARCHIVE_TYPE type, unsigned flags=0)
{
	// open the input stream
	std::ifstream fs(fileName, std::ios::in | std::ios::binary);
	if (!fs.is_open())
		return false;
	// serialize in the saved state
	return SerializeLoad(obj, fs, type, flags);
} // SerializeLoad
/*----------------------------------------------------------------*/

#endif // _USE_BOOST


#ifdef _USE_BREAKPAD

#include <breakpad/client/windows/crash_generation/client_info.h>
#include <breakpad/client/windows/crash_generation/crash_generation_server.h>
#include <breakpad/client/windows/handler/exception_handler.h>
#include <breakpad/client/windows/common/ipc_protocol.h>

class MiniDumper {
public:
	static inline void Create(const SEACAVE::String& strAppNameANSI, SEACAVE::String strDumpPathANSI, MINIDUMP_TYPE dumpType=MiniDumpNormal, bool bOutOfProcess=true) {
		static MiniDumper oMiniDumper(strAppNameANSI, strDumpPathANSI, dumpType, bOutOfProcess);
	}
	static inline void Create(const SEACAVE::String& strAppNameANSI, SEACAVE::String strDumpPathANSI, int verbosity, bool bOutOfProcess=true) {
		MINIDUMP_TYPE dumpType;
		switch (verbosity) {
		case 1:
			dumpType = MiniDumpWithDataSegs;
			break;
		case 2:
			dumpType = MiniDumpWithFullMemory;
			break;
		default:
			dumpType = MiniDumpNormal;
		}
		Create(strAppNameANSI, strDumpPathANSI, dumpType, bOutOfProcess);
	}
	MiniDumper(const SEACAVE::String& strAppNameANSI, SEACAVE::String strDumpPathANSI, MINIDUMP_TYPE dumpType=MiniDumpNormal, bool bOutOfProcess=true)
		:
		crash_server(NULL),
		handler(NULL)
	{
		if (strDumpPathANSI.IsEmpty())
			strDumpPathANSI = SEACAVE::Util::getCurrentDirectory();
		const std::wstring strDumpPath(strDumpPathANSI.begin(), strDumpPathANSI.end());
		const std::wstring strAppName(strAppNameANSI.begin(), strAppNameANSI.end());
		std::wstring strPipeName;
		if (bOutOfProcess) {
			// try to create an out-of-process minidumper
			const SEACAVE::String strUUIDANSI(SEACAVE::Util::getUniqueName(0));
			const std::wstring strUUID(strUUIDANSI.begin(), strUUIDANSI.end());
			strPipeName = L"\\\\.\\pipe\\BreakpadCrashServices\\"+strAppName+L"-"+strUUID;
			crash_server = new google_breakpad::CrashGenerationServer(
				strPipeName,
				NULL,
				NULL,
				NULL,
				NULL,
				NULL,
				NULL,
				NULL,
				NULL,
				NULL,
				true,
				&strDumpPath);
			if (!crash_server->Start()) {
				VERBOSE("error: unable to start minidump server");
				delete crash_server;
				crash_server = NULL;
			}
		}
		// create the minidumper
		const int kCustomInfoCount = 2;
		const google_breakpad::CustomInfoEntry kCustomInfoEntries[] = {
			google_breakpad::CustomInfoEntry(L"prod", strAppName.c_str()),
			google_breakpad::CustomInfoEntry(L"ver", L"1.0"),
		};
		google_breakpad::CustomClientInfo custom_info = {kCustomInfoEntries, kCustomInfoCount};
		handler = new google_breakpad::ExceptionHandler(
			strDumpPath,
			NULL,
			NULL,
			NULL,
			google_breakpad::ExceptionHandler::HANDLER_ALL,
			dumpType,
			(crash_server ? strPipeName.c_str() : NULL),
			&custom_info);
	}
	~MiniDumper() {
		delete handler;
		delete crash_server;
	}
	google_breakpad::CrashGenerationServer* crash_server;
	google_breakpad::ExceptionHandler* handler;
};

#endif
