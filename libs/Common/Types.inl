////////////////////////////////////////////////////////////////////
// Types.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace std {

//namespace tr1 {
// Specializations for unordered containers
template <> struct hash<SEACAVE::ImageRef>
{
	typedef SEACAVE::ImageRef argument_type;
	typedef size_t result_type;
	result_type operator()(const argument_type& v) const {
		return std::hash<uint64_t>()((const uint64_t&)v);
	}
};
//} // namespace tr1

} // namespace std


#if CV_MAJOR_VERSION > 3
template <typename REAL_TYPE, typename INT_TYPE>
INT_TYPE cvRANSACUpdateNumIters(REAL_TYPE p, REAL_TYPE ep, INT_TYPE modelPoints, INT_TYPE maxIters)
{
	ASSERT(p>=0 && p<=1);
	ASSERT(ep>=0 && ep<=1);
	// avoid inf's & nan's
	REAL_TYPE num = MAXF(REAL_TYPE(1)-p, SEACAVE::EPSILONTOLERANCE<REAL_TYPE>());
	REAL_TYPE denom = REAL_TYPE(1)-POWI(REAL_TYPE(1)-ep, modelPoints);
	if (denom < SEACAVE::EPSILONTOLERANCE<REAL_TYPE>())
		return 0;
	num = SEACAVE::LOGN(num);
	denom = SEACAVE::LOGN(denom);
	return (denom >= 0 || -num >= (-denom)*maxIters ? maxIters : (INT_TYPE)ROUND2INT(num/denom));
}
#endif

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
#else
#if CV_MINOR_VERSION < 4
template<typename _Tp, typename _AccTp> static inline
_AccTp normL2Sqr(const _Tp* a, int n)
{
	_AccTp s = 0;
	int i=0;
	#if CV_ENABLE_UNROLLED
	for (; i <= n - 4; i += 4) {
		_AccTp v0 = a[i], v1 = a[i+1], v2 = a[i+2], v3 = a[i+3];
		s += v0*v0 + v1*v1 + v2*v2 + v3*v3;
	}
	#endif
	for (; i < n; i++) {
		_AccTp v = a[i];
		s += v*v;
	}
	return s;
}
#endif

#if CV_MINOR_VERSION < 4 || CV_SUBMINOR_VERSION < 11
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
#endif

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


namespace SEACAVE {

// G L O B A L S ///////////////////////////////////////////////////

template <typename TYPE> const typename ColorType<TYPE>::value_type ColorType<TYPE>::ONE(1);
template <typename TYPE> const typename ColorType<TYPE>::alt_type ColorType<TYPE>::ALTONE(1);

template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::BLACK		(0, 0, 0);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::WHITE		(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::GRAY		(0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::RED		(ColorType<TYPE>::ONE, 0, 0);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::GREEN		(0, ColorType<TYPE>::ONE, 0);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::BLUE		(0, 0, ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::YELLOW	(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, 0);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::MAGENTA	(ColorType<TYPE>::ONE, 0, ColorType<TYPE>::ONE);
template <typename TYPE> const TPixel<TYPE> TPixel<TYPE>::CYAN		(0, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);

template <typename TYPE> const TColor<TYPE> TColor<TYPE>::BLACK		(0, 0, 0, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::WHITE		(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::GRAY		(0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE, 0.8f*ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::RED		(ColorType<TYPE>::ONE, 0, 0, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::GREEN		(0, ColorType<TYPE>::ONE, 0, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::BLUE		(0, 0, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::YELLOW	(ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, 0, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::MAGENTA	(ColorType<TYPE>::ONE, 0, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);
template <typename TYPE> const TColor<TYPE> TColor<TYPE>::CYAN		(0, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE, ColorType<TYPE>::ONE);

#ifdef _SUPPORT_CPP11
template <typename A>
inline typename std::enable_if<std::is_array<A>::value, size_t>::type
SizeOfArray(const A&) {
	return std::extent<A>::value;
}
#else
template <typename T, size_t N>
inline size_t
SizeOfArray(const T(&)[N]) {
	return N;
}
#endif


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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
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
	for (int i=0; i<m*n; ++i)
		nv.val[i] = Floor2Int(v.val[i]);
	return nv;
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> Ceil2Int(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (int i=0; i<m*n; ++i)
		nv.val[i] = Ceil2Int(v.val[i]);
	return nv;
}
template <typename TYPE, int m, int n>
FORCEINLINE SEACAVE::TMatrix<TYPE,m,n> Round2Int(const cv::Matx<TYPE,m,n>& v)
{
	SEACAVE::TMatrix<TYPE,m,n> nv;
	for (int i=0; i<m*n; ++i)
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
	return ((const typename TMatrix<TYPE,m*n,1>::Vec&)l).cross((const typename TMatrix<TYPE,m*n,1>::Vec&)r);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> cross(const cv::Matx<TYPE,m,n>& l, const cv::Matx<TYPE,m,n>& r) {
	return ((const typename TMatrix<TYPE,m*n,1>::Vec&)l).cross((const typename TMatrix<TYPE,m*n,1>::Vec&)r);
}

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
	return cv::normL2Sqr<TYPE,real>(v.cv::Mat::template ptr<const TYPE>(), v.area());
}
template <typename TYPE>
inline typename RealType<TYPE>::type normSq(const cv::Mat_<TYPE>& v) {
	typedef typename RealType<TYPE>::type real;
	return cv::normL2Sqr<TYPE,real>(v.cv::Mat::template ptr<const TYPE>(), v.cols*v.rows);
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
inline ACCTYPE normSqZeroMean(TYPE* ptr, size_t n) {
	TYPE* a = ptr;
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

	a = ptr;
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
inline ACCTYPE normSqZeroMean(TYPE* ptr) {
	TYPE* a = ptr;
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

	a = ptr;
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
	return SQRT(cv::normL2Sqr<TYPE,real>(v.cv::Mat::template ptr<const TYPE>(), v.area()));
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
	return cv::normalize((const typename TMatrix<TYPE,m*n,1>::Vec&)v);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> normalized(const cv::Matx<TYPE,m,n>& v) {
	return cv::normalize((const typename TMatrix<TYPE,m*n,1>::Vec&)v);
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
inline bool equal(const _Tp& left, const _Tp& right, const _Tp& = std::numeric_limits<_Tp>::epsilon())
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
inline unsigned decimalsToStore() {
	if (std::numeric_limits<_Tp>::is_integer){
		// integer types can display exactly one decimal more than their guaranteed precision digits
		return std::numeric_limits<_Tp>::digits10+1;
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

template <typename TFROM, typename TTO>
inline TPoint2<TTO> cvtPoint2(const TPoint2<TFROM>& p) {
	return TPoint2<TTO>(TTO(p.x), TTO(p.y));
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

template <typename TFROM, typename TTO>
inline TPoint3<TTO> cvtPoint3(const TPoint3<TFROM>& p) {
	return TPoint3<TTO>(TTO(p.x), TTO(p.y), TTO(p.z));
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

// TMatrix operators
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> operator + (const TMatrix<TYPE,m,n>& m1, const TMatrix<TYPE,m,n>& m2) {
	return TMatrix<TYPE,m,n>(m1, m2, cv::Matx_AddOp());
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> operator + (const TMatrix<TYPE,m,n>& m1, const cv::Matx<TYPE,m,n>& m2) {
	return cv::Matx<TYPE,m,n>(m1, m2, cv::Matx_AddOp());
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> operator + (const cv::Matx<TYPE,m,n>& m1, const TMatrix<TYPE,m,n>& m2) {
	return TMatrix<TYPE,m,n>(m1, m2, cv::Matx_AddOp());
}

template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> operator - (const TMatrix<TYPE,m,n>& m1, const TMatrix<TYPE,m,n>& m2) {
	return TMatrix<TYPE,m,n>(m1, m2, cv::Matx_SubOp());
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> operator - (const TMatrix<TYPE,m,n>& m1, const cv::Matx<TYPE,m,n>& m2) {
	return TMatrix<TYPE,m,n>(m1, m2, cv::Matx_SubOp());
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> operator - (const cv::Matx<TYPE,m,n>& m1, const TMatrix<TYPE,m,n>& m2) {
	return TMatrix<TYPE,m,n>(m1, m2, cv::Matx_SubOp());
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n> operator - (const TMatrix<TYPE,m,n>& M) {
	return TMatrix<TYPE,m,n>(M, TYPE(-1), cv::Matx_ScaleOp());
}

template <typename TYPE, int m, int l, int n>
inline TMatrix<TYPE,m,n> operator * (const TMatrix<TYPE,m,l>& m1, const TMatrix<TYPE,l,n>& m2) {
	return TMatrix<TYPE,m,n>(m1, m2, cv::Matx_MatMulOp());
}

template <typename TYPE, int m, int n, typename TYPE2>
inline TMatrix<TYPE,m,n> operator / (const TMatrix<TYPE,m,n>& mat, TYPE2 v) {
	typedef typename std::conditional<std::is_floating_point<TYPE2>::value,TYPE2,REAL>::type real_t;
	return TMatrix<TYPE,m,n>(mat, real_t(1)/v, cv::Matx_ScaleOp());
}
template <typename TYPE, int m, int n, typename TYPE2>
inline TMatrix<TYPE,m,n>& operator /= (TMatrix<TYPE,m,n>& mat, TYPE2 v) {
	return mat = mat/v;
}

// TImage operators
template <typename TFROM, typename TTO>
inline TImage<TTO> cvtImage(const TImage<TFROM>& image) {
	TImage<TTO> img(image.size());
	for (int r=0; r<image.rows; ++r)
		for (int c=0; c<image.cols; ++c)
			img(r,c) = image(r,c);
	return img;
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


// Informative template class for OpenCV "scalars"
#define DEFINE_GENERIC_CVDATATYPE(tp,ctp) namespace cv { \
template<> class DataType< tp > { \
public: \
	typedef tp value_type; \
	typedef value_type work_type; \
	typedef ctp channel_type; \
	typedef value_type vec_type; \
	enum { \
		generic_type = 0, \
		depth = DataDepth<channel_type>::value, \
		channels = sizeof(value_type)/sizeof(channel_type), \
		fmt = DataDepth<channel_type>::fmt, \
		type = CV_MAKETYPE(depth, channels) \
	}; \
}; }
#define DEFINE_CVDATATYPE(tp) DEFINE_GENERIC_CVDATATYPE(tp,tp::Type)

#define DEFINE_GENERIC_CVDATADEPTH(tp,ctp) namespace cv { \
template<> class DataDepth< tp > { \
public: \
	enum { \
		value = DataDepth< ctp >::value, \
		fmt = DataDepth< ctp >::fmt \
	}; \
}; }
#define DEFINE_CVDATADEPTH(tp) DEFINE_GENERIC_CVDATADEPTH(tp,tp::Type)

// define specialized cv:DataType<>
DEFINE_CVDATADEPTH(SEACAVE::hfloat)
DEFINE_CVDATADEPTH(SEACAVE::cuint32_t)

// define specialized cv:DataType<>
DEFINE_CVDATATYPE(SEACAVE::hfloat)
DEFINE_CVDATATYPE(SEACAVE::cuint32_t)
DEFINE_GENERIC_CVDATATYPE(uint64_t,double)
/*----------------------------------------------------------------*/
DEFINE_CVDATATYPE(SEACAVE::Point2i)
DEFINE_CVDATATYPE(SEACAVE::Point2hf)
DEFINE_CVDATATYPE(SEACAVE::Point2f)
DEFINE_CVDATATYPE(SEACAVE::Point2d)
/*----------------------------------------------------------------*/
DEFINE_CVDATATYPE(SEACAVE::Point3i)
DEFINE_CVDATATYPE(SEACAVE::Point3hf)
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
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image16F, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image32F, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image64F, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image8U3, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image8U4, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image32F3, uint8_t)
DEFINE_GENERIC_CVDATATYPE(SEACAVE::Image32F4, uint8_t)
/*----------------------------------------------------------------*/


namespace SEACAVE {

namespace CONVERT {

// convert sRGB to/from linear value
// (see http://en.wikipedia.org/wiki/SRGB)
template <typename T>
constexpr T sRGB2RGB(T x) {
	return x <= T(0.04045) ? x * (T(1)/T(12.92)) : POW((x + T(0.055)) * (T(1)/T(1.055)), T(2.4));
}
template <typename T>
constexpr T RGB2sRGB(T x) {
	return x <= T(0.0031308) ? T(12.92) * x : T(1.055) * POW(x, T(1)/T(2.4)) - T(0.055);
}
static const CAutoPtrArr<float> g_ptrsRGB82RGBf([]() {
	float* const buffer = new float[256];
	for (int i=0; i<256; ++i)
		buffer[i] = sRGB2RGB(float(i)/255.f);
	return buffer;
}());

// color conversion helper structures
template <typename TI, typename TO>
struct NormRGB_t {
	const TO v;
	inline NormRGB_t(TI _v) : v(TO(_v)*(TO(1)/TO(255))) {}
	inline operator TO () const { return v; }
};
template <typename TI, typename TO>
struct RGBUnNorm_t {
	const TO v;
	inline RGBUnNorm_t(TI _v) : v(TO(_v)*TO(255)) {}
	inline operator TO () const { return v; }
};
template <typename TI=float, typename TO=uint8_t>
struct RoundF2U_t {
	const TO v;
	inline RoundF2U_t(TI _v) : v(ROUND2INT<TO>(_v)) {}
	inline operator TO () const { return v; }
};
template <typename TI, typename TO>
struct sRGB2RGB_t {
	const TO v;
	inline sRGB2RGB_t(TI _v) : v(sRGB2RGB(TO(_v))) {}
	inline operator TO () const { return v; }
};
template <typename TI, typename TO>
struct NormsRGB2RGB_t {
	const TO v;
	inline NormsRGB2RGB_t(TI _v) : v(sRGB2RGB(TO(_v)*(TO(1)/TO(255)))) {}
	inline operator TO () const { return v; }
};
template <>
struct NormsRGB2RGB_t<uint8_t,float> {
	const float v;
	inline NormsRGB2RGB_t(uint8_t _v) : v(g_ptrsRGB82RGBf[_v]) {}
	inline operator float () const { return v; }
};
template <typename TI, typename TO>
struct NormsRGB2RGBUnNorm_t {
	const TO v;
	inline NormsRGB2RGBUnNorm_t(TI _v) : v(sRGB2RGB(TO(_v)*(TO(1)/TO(255)))*TO(255)) {}
	inline operator TO () const { return v; }
};
template <>
struct NormsRGB2RGBUnNorm_t<uint8_t,float> {
	const float v;
	inline NormsRGB2RGBUnNorm_t(uint8_t _v) : v(g_ptrsRGB82RGBf[_v]*255.f) {}
	inline operator float () const { return v; }
};

} // namespace CONVERT
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

template <typename FLT>
struct OppositeType {
	typedef FLT Type;
};
template <>
struct OppositeType<float> {
	typedef double Type;
};
template <>
struct OppositeType<double> {
	typedef float Type;
};
// Point2
template <typename FLT1, typename FLT2>
inline cv::Point_<FLT1> Cast(const cv::Point_<FLT2>& pt) {
	return pt;
}
template <typename FLT1, typename FLT2>
inline TPoint2<FLT1> Cast(const TPoint2<FLT2>& pt) {
	return pt;
}
// Point3
template <typename FLT1, typename FLT2>
inline cv::Point3_<FLT1> Cast(const cv::Point3_<FLT2>& pt) {
	return pt;
}
template <typename FLT1, typename FLT2>
inline TPoint3<FLT1> Cast(const TPoint3<FLT2>& pt) {
	return pt;
}
// Pixel
template <typename FLT1, typename FLT2>
inline TPixel<FLT1> Cast(const TPixel<FLT2>& pt) {
	return pt;
}
// Color
template <typename FLT1, typename FLT2>
inline TColor<FLT1> Cast(const TColor<FLT2>& pt) {
	return pt;
}
// Matrix
template <typename FLT1, typename FLT2, int m, int n>
inline TMatrix<FLT1,m,n> Cast(const TMatrix<FLT2,m,n>& v) {
	return v;
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
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0)
{
	STATIC_ASSERT(channels >= 1);
	val[0] = v0;
	for (int i = 1; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1)
{
	STATIC_ASSERT(channels >= 2);
	val[0] = v0; val[1] = v1;
	for (int i = 2; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2)
{
	STATIC_ASSERT(channels >= 3);
	val[0] = v0; val[1] = v1; val[2] = v2;
	for (int i = 3; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3)
{
	STATIC_ASSERT(channels >= 4);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	for (int i = 4; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4)
{
	STATIC_ASSERT(channels >= 5);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3; val[4] = v4;
	for (int i = 5; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5)
{
	STATIC_ASSERT(channels >= 6);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5;
	for (int i = 6; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6)
{
	STATIC_ASSERT(channels >= 7);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6;
	for (int i = 7; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7)
{
	STATIC_ASSERT(channels >= 8);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
	for (int i = 8; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7, TYPE v8)
{
	STATIC_ASSERT(channels >= 9);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
	val[8] = v8;
	for (int i = 9; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7, TYPE v8, TYPE v9)
{
	STATIC_ASSERT(channels >= 10);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
	val[8] = v8; val[9] = v9;
	for (int i = 10; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7, TYPE v8, TYPE v9, TYPE v10, TYPE v11)
{
	STATIC_ASSERT(channels >= 12);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
	val[8] = v8; val[9] = v9; val[10] = v10; val[11] = v11;
	for (int i = 12; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7, TYPE v8, TYPE v9, TYPE v10, TYPE v11, TYPE v12, TYPE v13)
{
	STATIC_ASSERT(channels == 14);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
	val[8] = v8; val[9] = v9; val[10] = v10; val[11] = v11;
	val[12] = v12; val[13] = v13;
	for (int i = 14; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(TYPE v0, TYPE v1, TYPE v2, TYPE v3, TYPE v4, TYPE v5, TYPE v6, TYPE v7, TYPE v8, TYPE v9, TYPE v10, TYPE v11, TYPE v12, TYPE v13, TYPE v14, TYPE v15)
{
	STATIC_ASSERT(channels >= 16);
	val[0] = v0; val[1] = v1; val[2] = v2; val[3] = v3;
	val[4] = v4; val[5] = v5; val[6] = v6; val[7] = v7;
	val[8] = v8; val[9] = v9; val[10] = v10; val[11] = v11;
	val[12] = v12; val[13] = v13; val[14] = v14; val[15] = v15;
	for (int i = 16; i < channels; i++)
		val[i] = TYPE(0);
}
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,n>::TMatrix(const TYPE* values)
{
	for (int i = 0; i < channels; i++)
		val[i] = values[i];
}


template <typename TYPE, int m, int n>
inline bool TMatrix<TYPE,m,n>::IsEqual(const Base& rhs) const
{
	for (int i=0; i<elems; ++i)
		if (!ISEQUAL(val[i], rhs.val[i]))
			return false;
	return true;
}
template <typename TYPE, int m, int n>
inline bool TMatrix<TYPE,m,n>::IsEqual(const Base& rhs, TYPE eps) const
{
	for (int i=0; i<elems; ++i)
		if (!equal(val[i], rhs.val[i], eps))
			return false;
	return true;
}

// calculate right null-Space of the matrix ([n,1])
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,n,n-m> TMatrix<TYPE,m,n>::RightNullSpace(int flags /*= 0*/) const
{
	STATIC_ASSERT(n > m);
	const cv::SVD svd(*this, flags|cv::SVD::FULL_UV);
	// the orthonormal basis of the null space is formed by the columns
	// of svd.vt such that the corresponding singular values are 0 (n - m or svd.vt.cols - svd.w.rows)
	ASSERT(svd.vt.rows == n && svd.vt.rows == svd.vt.cols && svd.w.rows == m);
	// return result (last rows transposed)
	return svd.vt.rowRange(m,n).t();
}
// calculate right null-vector of the matrix ([n,1])
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,n,1> TMatrix<TYPE,m,n>::RightNullVector(int flags /*= 0*/) const
{
	const cv::SVD svd(*this, (m >= n ? flags : flags|cv::SVD::FULL_UV));
	// the singular values could be checked for numerical stability
	// return result (last row transposed)
	ASSERT(svd.vt.rows == n);
	return *svd.vt.ptr< const TMatrix<TYPE,n,1> >(n-1);
}
// calculate left null-vector of the matrix ([m,1])
template <typename TYPE, int m, int n>
inline TMatrix<TYPE,m,1> TMatrix<TYPE,m,n>::LeftNullVector(int flags /*= 0*/) const
{
	return TMatrix<TYPE,m,n>(Base::t()).RightNullVector(flags);
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
	for (int sub=0; sub<dim; sub++) {
		// first column is used,
		// set up the n-1xn-1 submatrices from the right matrix part
		// this is done n times (loop counter="sub")
		for (int i=0; i<subdim; i++) {
			// construct the sub matrix under inspection,
			// skip the first column
			for (int j=0; j<subdim; j++) {
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
	for (const TYPE* dataP = getData()+Base::total()-1; dataP >= getData(); dataP--)
		result += ABS(*dataP);
	return result;
}


template <typename TYPE>
inline double TDMatrix<TYPE>::getNormL2() const
{
	double result = 0;
	for (const TYPE* dataP = getData()+Base::total()-1; dataP >= getData(); dataP--)
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
	for (int i=0; i<s1; i++) {
		for (int k=0; k<s2; k++) {
			dst[l]=(*this)[i]*arg[k];
			l++;
		}
	}
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

// Color ramp code from "Colour Ramping for Data Visualisation"
// (see http://paulbourke.net/texture_colour/colourspace)
template <typename TYPE/*PixelType*/>
template <typename VT/*ValueType*/>
TPixel<TYPE> TPixel<TYPE>::colorRamp(VT v, VT vmin, VT vmax)
{
	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	const TYPE dv((TYPE)(vmax - vmin));
	TPixel<TYPE> c(1,1,1); // white
	if (v < vmin + (VT)(TYPE(0.25) * dv)) {
		c.r = TYPE(0);
		c.g = TYPE(4) * (v - vmin) / dv;
	} else if (v < vmin + (VT)(TYPE(0.5) * dv)) {
		c.r = TYPE(0);
		c.b = TYPE(1) + TYPE(4) * (vmin + TYPE(0.25) * dv - v) / dv;
	} else if (v < vmin + (VT)(TYPE(0.75) * dv)) {
		c.r = TYPE(4) * (v - vmin - TYPE(0.5) * dv) / dv;
		c.b = TYPE(0);
	} else {
		c.g = TYPE(1) + TYPE(4) * (vmin + TYPE(0.75) * dv - v) / dv;
		c.b = TYPE(0);
	}
	return c;
}

// Gray values are expected in the range [0, 1] and converted to RGB values.
template <typename TYPE>
TPixel<TYPE> TPixel<TYPE>::gray2color(ALT gray)
{
	ASSERT(ALT(0) <= gray && gray <= ALT(1));
	// Jet colormap inspired by Matlab.
	auto const Interpolate = [](ALT val, ALT y0, ALT x0, ALT y1, ALT x1) -> ALT {
		return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
	};
	auto const  Base = [&Interpolate](ALT val) -> ALT {
		if (val <= ALT(0.125)) {
			return ALT(0);
		} else if (val <= ALT(0.375)) {
			return Interpolate(ALT(2) * val - ALT(1), ALT(0), ALT(-0.75), ALT(1), ALT(-0.25));
		} else if (val <= ALT(0.625)) {
			return ALT(1);
		} else if (val <= ALT(0.87)) {
			return Interpolate(ALT(2) * val - ALT(1), ALT(1), ALT(0.25), ALT(0), ALT(0.75));
		} else {
			return ALT(0);
		}
	};
	return TPixel<ALT>(
		Base(gray + ALT(0.25)),
		Base(gray),
		Base(gray - ALT(0.25))
	).template cast<TYPE>();
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
	return (BaseBase::operator()( ly, lx)*x1 + BaseBase::operator()( ly, lx+1)*x)*y1 +
		   (BaseBase::operator()(ly+1,lx)*x1 + BaseBase::operator()(ly+1,lx+1)*x)*y;
}
template <typename TYPE>
template <typename T>
TYPE TImage<TYPE>::sampleSafe(const TPoint2<T>& pt) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	return (getPixel( ly, lx)*x1 + getPixel( ly, lx+1)*x)*y1 +
		   (getPixel(ly+1,lx)*x1 + getPixel(ly+1,lx+1)*x)*y;
}
/*----------------------------------------------------------------*/


// sample by bilinear interpolation, using only pixels that meet the user condition
template <typename TYPE>
template <typename T, typename TV, typename Functor>
bool TImage<TYPE>::sample(TV& v, const TPoint2<T>& pt, const Functor& functor) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(BaseBase::operator()(ly  , lx  )); const bool b00(functor(x0y0));
	const TYPE& x1y0(BaseBase::operator()(ly  , lx+1)); const bool b10(functor(x1y0));
	const TYPE& x0y1(BaseBase::operator()(ly+1, lx  )); const bool b01(functor(x0y1));
	const TYPE& x1y1(BaseBase::operator()(ly+1, lx+1)); const bool b11(functor(x1y1));
	if (!b00 && !b10 && !b01 && !b11)
		return false;
	v = TV(y1*(x1*Cast<T>(b00 ? x0y0 : (b10 ? x1y0 : (b01 ? x0y1 : x1y1))) + x*Cast<T>(b10 ? x1y0 : (b00 ? x0y0 : (b11 ? x1y1 : x0y1)))) +
		   y *(x1*Cast<T>(b01 ? x0y1 : (b11 ? x1y1 : (b00 ? x0y0 : x1y0))) + x*Cast<T>(b11 ? x1y1 : (b01 ? x0y1 : (b10 ? x1y0 : x0y0)))));
	return true;
}
template <typename TYPE>
template <typename T, typename TV, typename Functor>
bool TImage<TYPE>::sampleSafe(TV& v, const TPoint2<T>& pt, const Functor& functor) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(getPixel(ly  , lx  )); const bool b00(functor(x0y0));
	const TYPE& x1y0(getPixel(ly  , lx+1)); const bool b10(functor(x1y0));
	const TYPE& x0y1(getPixel(ly+1, lx  )); const bool b01(functor(x0y1));
	const TYPE& x1y1(getPixel(ly+1, lx+1)); const bool b11(functor(x1y1));
	if (!b00 && !b10 && !b01 && !b11)
		return false;
	v = TV(y1*(x1*Cast<T>(b00 ? x0y0 : (b10 ? x1y0 : (b01 ? x0y1 : x1y1))) + x*Cast<T>(b10 ? x1y0 : (b00 ? x0y0 : (b11 ? x1y1 : x0y1)))) +
		   y *(x1*Cast<T>(b01 ? x0y1 : (b11 ? x1y1 : (b00 ? x0y0 : x1y0))) + x*Cast<T>(b11 ? x1y1 : (b01 ? x0y1 : (b10 ? x1y0 : x0y0)))));
	return true;
}
// same as above, but using default value if the condition is not met
template <typename TYPE>
template <typename T, typename Functor>
TYPE TImage<TYPE>::sample(const TPoint2<T>& pt, const Functor& functor, const TYPE& dv) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(BaseBase::operator()(ly  , lx  )); const bool b00(functor(x0y0));
	const TYPE& x1y0(BaseBase::operator()(ly  , lx+1)); const bool b10(functor(x1y0));
	const TYPE& x0y1(BaseBase::operator()(ly+1, lx  )); const bool b01(functor(x0y1));
	const TYPE& x1y1(BaseBase::operator()(ly+1, lx+1)); const bool b11(functor(x1y1));
	return TYPE(y1*(x1*Cast<T>(b00 ? x0y0 : dv) + x*Cast<T>(b10 ? x1y0 : dv)) +
				y *(x1*Cast<T>(b01 ? x0y1 : dv) + x*Cast<T>(b11 ? x1y1 : dv)));
}
template <typename TYPE>
template <typename T, typename Functor>
TYPE TImage<TYPE>::sampleSafe(const TPoint2<T>& pt, const Functor& functor, const TYPE& dv) const
{
	const int lx((int)pt.x);
	const int ly((int)pt.y);
	const T x(pt.x-lx), x1(T(1)-x);
	const T y(pt.y-ly), y1(T(1)-y);
	const TYPE& x0y0(getPixel(ly  , lx  )); const bool b00(functor(x0y0));
	const TYPE& x1y0(getPixel(ly  , lx+1)); const bool b10(functor(x1y0));
	const TYPE& x0y1(getPixel(ly+1, lx  )); const bool b01(functor(x0y1));
	const TYPE& x1y1(getPixel(ly+1, lx+1)); const bool b11(functor(x1y1));
	return TYPE(y1*(x1*Cast<T>(b00 ? x0y0 : dv) + x*Cast<T>(b10 ? x1y0 : dv)) +
				y *(x1*Cast<T>(b01 ? x0y1 : dv) + x*Cast<T>(b11 ? x1y1 : dv)));
}

// same as above, sample image at a specified position, but using the given sampler
#include "Sampler.inl"
template <typename TYPE>
template <typename SAMPLER, typename INTERTYPE>
INTERTYPE TImage<TYPE>::sample(const SAMPLER& sampler, const TPoint2<typename SAMPLER::Type>& pt) const
{
	return Sampler::Sample< TImage<TYPE>, SAMPLER, TPoint2<typename SAMPLER::Type>, INTERTYPE >(*this, sampler, pt);
}

// convert color image to gray
template <typename TYPE>
template <typename T>
void TImage<TYPE>::toGray(TImage<T>& out, int code, bool bNormalize, bool bSRGB) const
{
	#if 1
	typedef typename RealType<T,float>::type Real;
	ASSERT(code==cv::COLOR_RGB2GRAY || code==cv::COLOR_RGBA2GRAY || code==cv::COLOR_BGR2GRAY || code==cv::COLOR_BGRA2GRAY);
	static const Real coeffsRGB[] = {Real(0.299), Real(0.587), Real(0.114)};
	static const Real coeffsBGR[] = {Real(0.114), Real(0.587), Real(0.299)};
	const Real* coeffs;
	switch (code) {
	case cv::COLOR_BGR2GRAY:
	case cv::COLOR_BGRA2GRAY:
		coeffs = coeffsBGR;
		break;
	case cv::COLOR_RGB2GRAY:
	case cv::COLOR_RGBA2GRAY:
		coeffs = coeffsRGB;
		break;
	default:
		ASSERT("Unsupported image format" == NULL);
	}
	const Real &cb(coeffs[0]), &cg(coeffs[1]), &cr(coeffs[2]);
	if (out.rows!=rows || out.cols!=cols)
		out.create(rows, cols);
	ASSERT(cv::Mat::isContinuous());
	ASSERT(out.cv::Mat::isContinuous());
	const int scn(this->cv::Mat::channels());
	T* dst = out.cv::Mat::template ptr<T>();
	T* const dstEnd = dst + out.area();
	typedef typename cv::DataType<TYPE>::channel_type ST;
	if (bSRGB) {
		if (bNormalize) {
			typedef typename CONVERT::NormsRGB2RGB_t<ST,Real> ColConv;
			for (const ST* src=cv::Mat::template ptr<ST>(); dst!=dstEnd; src+=scn)
				*dst++ = T(cb*ColConv(src[0]) + cg*ColConv(src[1]) + cr*ColConv(src[2]));
		} else {
			typedef typename CONVERT::NormsRGB2RGBUnNorm_t<ST,Real> ColConv;
			for (const ST* src=cv::Mat::template ptr<ST>(); dst!=dstEnd; src+=scn)
				*dst++ = T(cb*ColConv(src[0]) + cg*ColConv(src[1]) + cr*ColConv(src[2]));
		}
	} else {
		if (bNormalize) {
			typedef typename CONVERT::NormRGB_t<ST,Real> ColConv;
			for (const ST* src=cv::Mat::template ptr<ST>(); dst!=dstEnd; src+=scn)
				*dst++ = T(cb*ColConv(src[0]) + cg*ColConv(src[1]) + cr*ColConv(src[2]));
		} else {
			for (const ST* src=cv::Mat::template ptr<ST>(); dst!=dstEnd; src+=scn)
				*dst++ = T(cb*src[0] + cg*src[1] + cr*src[2]);
		}
	}
	#else
	cv::Mat cimg;
	convertTo(cimg, cv::DataType<real>::type);
	cv::cvtColor(cimg, out, code);
	if (bNormalize)
		out *= T(1)/T(255);
	#endif
}
/*----------------------------------------------------------------*/


// compute scaled size such that the biggest dimension is scaled as desired
// and the smaller one maintains the aspect ratio as best as it can
template <typename TYPE>
cv::Size TImage<TYPE>::computeResize(const cv::Size& size, REAL scale)
{
	cv::Size scaledSize;
	if (size.width > size.height) {
		scaledSize.width = ROUND2INT(REAL(size.width)*scale);
		scaledSize.height = ROUND2INT(REAL(size.height)*scaledSize.width/REAL(size.width));
	} else {
		scaledSize.height = ROUND2INT(REAL(size.height)*scale);
		scaledSize.width = ROUND2INT(REAL(size.width)*scaledSize.height/REAL(size.height));
	}
	return scaledSize;
}
// compute the final scaled size by performing successive resizes
// with the given scale value
template <typename TYPE>
cv::Size TImage<TYPE>::computeResize(const cv::Size& size, REAL scale, unsigned resizes)
{
	cv::Size scaledSize(size);
	while (resizes-- > 0)
		scaledSize = computeResize(scaledSize, scale);
	return scaledSize;
}

// compute image scale for a given max and min resolution
template <typename TYPE>
unsigned TImage<TYPE>::computeMaxResolution(unsigned width, unsigned height, unsigned& level, unsigned minImageSize, unsigned maxImageSize)
{
	// consider the native resolution the max(width,height)
	const unsigned imageSize = MAXF(width, height);
	// if the max level it's used, return original image size
	if (level == 0)
		return MINF(imageSize, maxImageSize);
	// compute the resolution corresponding to the desired level
	unsigned size = (imageSize >> level);
	// if the image is too small
	if (size < minImageSize) {
		// start from the max level
		level = 0;
		while ((imageSize>>(level+1)) >= minImageSize)
			++level;
		size = (imageSize>>level);
	}
	return MINF(size, maxImageSize);
}
template <typename TYPE>
unsigned TImage<TYPE>::computeMaxResolution(unsigned& level, unsigned minImageSize, unsigned maxImageSize) const
{
	return computeMaxResolution((unsigned)width(), (unsigned)height(), level, minImageSize, maxImageSize);
}
/*----------------------------------------------------------------*/


// Raster the given triangle and output the position of each pixel of the triangle;
// based on "Advanced Rasterization" by Nick (Nicolas Capens)
// http://devmaster.net/forums/topic/1145-advanced-rasterization
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
			const int_t x0 = int_t(x) << 4;
			const int_t x1 = int_t(x + q - 1) << 4;
			const int_t y0 = int_t(y) << 4;
			const int_t y1 = int_t(y + q - 1) << 4;

			// Evaluate half-space functions
			const bool a00 = C1 + DX12 * y0 - DY12 * x0 > 0;
			const bool a10 = C1 + DX12 * y0 - DY12 * x1 > 0;
			const bool a01 = C1 + DX12 * y1 - DY12 * x0 > 0;
			const bool a11 = C1 + DX12 * y1 - DY12 * x1 > 0;
			const int a = (a00 << 0) | (a10 << 1) | (a01 << 2) | (a11 << 3);

			const bool b00 = C2 + DX23 * y0 - DY23 * x0 > 0;
			const bool b10 = C2 + DX23 * y0 - DY23 * x1 > 0;
			const bool b01 = C2 + DX23 * y1 - DY23 * x0 > 0;
			const bool b11 = C2 + DX23 * y1 - DY23 * x1 > 0;
			const int b = (b00 << 0) | (b10 << 1) | (b01 << 2) | (b11 << 3);

			const bool c00 = C3 + DX31 * y0 - DY31 * x0 > 0;
			const bool c10 = C3 + DX31 * y0 - DY31 * x1 > 0;
			const bool c01 = C3 + DX31 * y1 - DY31 * x0 > 0;
			const bool c11 = C3 + DX31 * y1 - DY31 * x1 > 0;
			const int c = (c00 << 0) | (c10 << 1) | (c01 << 2) | (c11 << 3);

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
template <typename T, typename PARSER>
void TImage<TYPE>::DrawLine(const TPoint2<T>& p1, const TPoint2<T>& p2, PARSER& parser)
{
	#if 0
	const TPoint2<T> d(ABS(p2 - p1));
	const int sx(p1.x<p2.x ? 1 : -1);
	const int sy(p1.y<p2.y ? 1 : -1);
	T err((d.x>d.y ? d.x : -d.y)/2), e2;

	const ImageRef x2(p2);
	ImageRef p(p1);
	while (true) {
		parser(p);
		if (p == x2) break;
		e2 = err;
		if (e2 >-d.x) { err -= d.y; p.x += sx; }
		if (e2 < d.y) { err += d.x; p.y += sy; }
	}
	#else
	// Bresenham's line algorithm
	// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	T x1(p1.x), y1(p1.y), x2(p2.x), y2(p2.y);
	const bool steep(ABS(y2 - y1) > ABS(x2 - x1));
	if (steep) {
		std::swap(x1, y1);
		std::swap(x2, y2);
	}
	if (x1 > x2) {
		std::swap(x1, x2);
		std::swap(y1, y2);
	}
	const T dx(x2 - x1);
	const T dy(ABS(y2 - y1));

	T error(dx / 2.f);
	int y(ROUND2INT(y1));
	const int ystep((y1 < y2) ? 1 : -1);
	const int maxX(ROUND2INT(x2));
	for (int x=ROUND2INT(x1); x<maxX; ++x) {
		parser(steep ? ImageRef(y,x) : ImageRef(x,y));
		error -= dy;
		if (error < 0) {
			y += ystep;
			error += dx;
		}
	}
	#endif
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


// set all invalid pixels to the average of the neighbors
template <typename TYPE>
template <int HalfSize>
void TImage<TYPE>::DilateMean(TImage<TYPE>& dst, const TYPE& invalid) const
{
	TImage<TYPE> out;
	Base::copyTo(out);
	const int RowsEnd(rows-HalfSize);
	const int ColsEnd(cols-HalfSize);
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			const TYPE& v = Base::operator()(r,c);
			if (v != invalid)
				continue;
			TYPE vo(0);
			int n(0);
			for (int i=-HalfSize; i<=HalfSize; ++i) {
				const int rw(r+i);
				for (int j=-HalfSize; j<=HalfSize; ++j) {
					if (i==0 && j==0)
						continue;
					const int cw(c+j);
					const TYPE& vi = Base::operator()(rw,cw);
					if (vi == invalid)
						continue;
					vo += vi;
					++n;
				}
			}
			if (n > 0)
				out(r,c) = (n > 1 ? vo/n : vo);
		}
	}
	dst = out;
}
/*----------------------------------------------------------------*/


template <typename TYPE>
bool TImage<TYPE>::Load(const String& fileName)
{
	if (Util::getFileExt(fileName).ToLower() == ".pfm") {
		if (Base::depth() != CV_32F)
			return false;
		File fImage(fileName, File::READ, File::OPEN);
		if (!fImage.isOpen())
			return false;
		ASSERT(sizeof(float) == 4);
		int i;
		char buffer[128];
		// check header
		for (i=0; i<4; ++i) {
			if (fImage.read(buffer+i, 1) != 1)
				return false;
			if (buffer[i] == '\n')
				break;
		}
		if (buffer[0] != 'P' || buffer[1] != 'f' || buffer[i] != '\n')
			return false;
		// read resolution
		int w, h;
		for (i=0; i<127; ++i) {
			if (fImage.read(buffer+i, 1) != 1)
				return false;
			if (buffer[i] == '\n')
				break;
		}
		buffer[i] = 0;
		if (sscanf(buffer, "%d %d", &w, &h) != 2)
			return false;
		// read number of channels
		double sc;
		for (i=0; i<127; ++i) {
			if (fImage.read(buffer+i, 1) != 1)
				return false;
			if (buffer[i] == '\n')
				break;
		}
		buffer[i] = 0;
		if (sscanf(buffer, "%lf", &sc) != 1)
			return false;
		const bool bLittleEndian(sc < 0);
		#if __BYTE_ORDER == __LITTLE_ENDIAN
		ASSERT(bLittleEndian);
		#else
		ASSERT(!bLittleEndian);
		#endif
		const int nChannels(bLittleEndian ? -((int)sc) : (int)sc);
		if (nChannels != Base::channels())
			return false;
		Base::create(h, w);
		ASSERT(sizeof(float)*Base::channels() == Base::step.p[1]);
		const size_t rowbytes((size_t)Base::size.p[1]*Base::step.p[1]);
		for (int i=0; i<rows; ++i)
			if (fImage.read(cv::Mat::template ptr<float>(i), rowbytes) != rowbytes)
				return false;
		return true;
	}
	cv::Mat img(cv::imread(fileName, cv::IMREAD_UNCHANGED));
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
		Util::ensureFolder(fileName);
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
			fImage.write(cv::Mat::template ptr<const float>(i), rowbytes);
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
void TImage<TYPE>::Show(const String& winname, int delay, bool bDestroy) const
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
	const Type* src = in.cv::Mat::template ptr<Type>();
	Type* dst = out.cv::Mat::template ptr<Type>();

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
inline void eigen_SO3_exp(const typename Eigen::SO3<Precision>::Vec3& w, typename Eigen::SO3<Precision>::Mat3& R) {
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
inline typename Eigen::SO3<Precision>::Mat3 Eigen::SO3<Precision>::exp(const Vec3& w) const {
	Mat3 result;
	eigen_SO3_exp<Precision>(w, result);
	return result;
}

/// Take the logarithm of the matrix, generating the corresponding vector in the Lie Algebra.
/// See the Detailed Description for details of this vector.
template <typename Precision>
inline void eigen_SO3_ln(const typename Eigen::SO3<Precision>::Mat3& R, typename Eigen::SO3<Precision>::Vec3& w) {
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
		typename Eigen::SO3<Precision>::Vec3 r2;
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
inline typename Eigen::SO3<Precision>::Vec3 Eigen::SO3<Precision>::ln() const {
	Vec3 result;
	eigen_SO3_ln<Precision>(mat, result);
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
	is >> rhs.mat;
	rhs.coerce();
	return is;
}

/// Right-multiply by a Vector
/// @relates SO3
template <typename P, int O>
inline Eigen::Matrix<P,3,1,O> operator*(const Eigen::SO3<P>& lhs, const Eigen::Matrix<P,3,1,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Vector
/// @relates SO3
template <typename P, int O>
inline Eigen::Matrix<P,3,1,O> operator*(const Eigen::Matrix<P,3,1,O>& lhs, const Eigen::SO3<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/// Right-multiply by a matrix
/// @relates SO3
template <typename P, int C, int O>
inline Eigen::Matrix<P,3,C,O> operator*(const Eigen::SO3<P>& lhs, const Eigen::Matrix<P,3,C,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a matrix
/// @relates SO3
template <typename P, int R, int O>
inline Eigen::Matrix<P,R,3,O> operator*(const Eigen::Matrix<P,R,3,O>& lhs, const Eigen::SO3<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/*----------------------------------------------------------------*/


/// Exponentiate an angle in the Lie algebra to generate a new SO2.
template <typename Precision>
inline void eigen_SO2_exp(const Precision& d, typename Eigen::SO2<Precision>::Mat2& R) {
	R(0,0) = R(1,1) = cos(d);
	R(1,0) = sin(d);
	R(0,1) = -R(1,0);
}
template <typename Precision>
inline typename Eigen::SO2<Precision>::Mat2 Eigen::SO2<Precision>::exp(const Precision& d) const {
	Mat2 result;
	eigen_SO2_exp<Precision>(d, result);
	return result;
}

/// Extracts the rotation angle from the SO2
template <typename Precision>
inline void eigen_SO2_ln(const typename Eigen::SO2<Precision>::Mat2& R, Precision& d) {
	d = atan2(R(1,0), R(0,0));
}
template <typename Precision>
inline Precision Eigen::SO2<Precision>::ln() const {
	Precision d;
	eigen_SO2_ln<Precision>(mat, d);
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
	is >> rhs.mat;
	rhs.coerce();
	return is;
}

/// Right-multiply by a Vector
/// @relates SO2
template <typename P, int O>
inline Eigen::Matrix<P,2,1,O> operator*(const Eigen::SO2<P>& lhs, const Eigen::Matrix<P,2,1,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Vector
/// @relates SO2
template <typename P, int O>
inline Eigen::Matrix<P,2,1,O> operator*(const Eigen::Matrix<P,2,1,O>& lhs, const Eigen::SO2<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/// Right-multiply by a Matrix
/// @relates SO2
template <typename P, int C, int O>
inline Eigen::Matrix<P,2,C,O> operator*(const Eigen::SO2<P>& lhs, const Eigen::Matrix<P,2,C,O>& rhs) {
	return lhs.get_matrix() * rhs;
}
/// Left-multiply by a Matrix
/// @relates SO2
template <typename P, int R, int O>
inline Eigen::Matrix<P,R,2,O> operator*(const Eigen::Matrix<P,R,2,O>& lhs, const Eigen::SO2<P>& rhs) {
	return lhs * rhs.get_matrix();
}
/*----------------------------------------------------------------*/

#endif // _USE_EIGEN


// C L A S S  //////////////////////////////////////////////////////

#ifdef _USE_BOOST

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
		template<class Archive>
		inline void serialize(Archive& ar, cv::Mat& m, const unsigned int version) {
			split_free(ar, m, version);
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
		inline void serialize(Archive& ar, cv::Mat_<_Tp>& m, const unsigned int version) {
			split_free(ar, m, version);
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
#if defined(_MSC_VER)
#pragma warning (push)
#pragma warning (disable : 4715) // not all control paths return a value
#endif
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
//#include <boost/archive/xml_oarchive.hpp>
//#include <boost/archive/xml_iarchive.hpp>
// include headers that implement compressed serialization support
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#if defined(_MSC_VER)
#pragma warning (pop)
#endif

enum ARCHIVE_TYPE {
	ARCHIVE_MVS = -1,
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
			strDumpPathANSI = SEACAVE::Util::getCurrentFolder();
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
