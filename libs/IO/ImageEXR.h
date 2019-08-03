////////////////////////////////////////////////////////////////////
// ImageEXR.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGEEXR_H__
#define __SEACAVE_IMAGEEXR_H__


// D E F I N E S ///////////////////////////////////////////////////


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class IO_API CImageEXR : public CImage
{
public:
	CImageEXR();
	virtual ~CImageEXR();

	void		Close();

	HRESULT		ReadHeader();
	HRESULT		ReadData(void*, PIXELFORMAT, Size nStride, Size lineWidth);
	HRESULT		WriteHeader(PIXELFORMAT, Size width, Size height, BYTE numLevels);
	HRESULT		WriteData(void*, PIXELFORMAT, Size nStride, Size lineWidth);

protected:

}; // class CImageEXR
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGEEXR_H__
