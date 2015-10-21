////////////////////////////////////////////////////////////////////
// Types.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Types.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////


// G L O B A L S ///////////////////////////////////////////////////

#ifndef _MSC_VER
int _vscprintf(LPCSTR format, va_list pargs) {
	va_list argcopy;
	va_copy(argcopy, pargs);
	const int retval(vsnprintf(NULL, 0, format, argcopy));
	va_end(argcopy);
	return retval;
}
#endif
/*----------------------------------------------------------------*/


namespace SEACAVE {

const ColorType<uint8_t>::value_type ColorType<uint8_t>::ONE(255);
const ColorType<uint8_t>::alt_type ColorType<uint8_t>::ALTONE(1.f);

const ColorType<uint32_t>::value_type ColorType<uint32_t>::ONE(255);
const ColorType<uint32_t>::alt_type ColorType<uint32_t>::ALTONE(1.f);

const ColorType<float>::value_type ColorType<float>::ONE(1.f);
const ColorType<float>::alt_type ColorType<float>::ALTONE(255);

const ColorType<double>::value_type ColorType<double>::ONE(1.0);
const ColorType<double>::alt_type ColorType<double>::ALTONE(255);
/*----------------------------------------------------------------*/


// print matrix
template<typename TYPE>
String cvMat2String(const TYPE* M, uint32_t rows, uint32_t cols, uint32_t step, LPCSTR format) {
	String str;
	char buf[32];
	if (step == 0)
		step = rows;
	for (uint32_t i=0; i<rows; ++i) {
		const TYPE* Mi = M+i*step;
		for (uint32_t j=0; j<cols; ++j) {
			_stprintf(buf, format, Mi[j]);
			str += buf;
		}
		str += _T("\n");
	}
	return str;
}
String cvMat2String(const cv::Mat& M, LPCSTR format) {
	switch (M.type()) {
	case CV_32F: return cvMat2String(M.ptr<float>(), (uint32_t)M.rows, (uint32_t)M.cols, (uint32_t)M.step1(), format);
	case CV_64F: return cvMat2String(M.ptr<double>(), (uint32_t)M.rows, (uint32_t)M.cols, (uint32_t)M.step1(), format);
	}
	return String();
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE


// C L A S S  //////////////////////////////////////////////////////

