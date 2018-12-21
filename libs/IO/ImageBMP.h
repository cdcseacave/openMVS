////////////////////////////////////////////////////////////////////
// ImageBMP.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGEBMP_H__
#define __SEACAVE_IMAGEBMP_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class IO_API CImageBMP : public CImage
{
public:
	CImageBMP();
	virtual ~CImageBMP();

	HRESULT		ReadHeader();
	HRESULT		ReadData(void*, PIXELFORMAT, Size nStride, Size lineWidth);
	HRESULT		WriteHeader(PIXELFORMAT, Size width, Size height, BYTE numLevels);
	HRESULT		WriteData(void*, PIXELFORMAT, Size nStride, Size lineWidth);
}; // class CImageBMP
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGEBMP_H__
