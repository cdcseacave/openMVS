////////////////////////////////////////////////////////////////////
// ImageSCI.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGESCI_H__
#define __SEACAVE_IMAGESCI_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class IO_API CImageSCI : public CImage
{
public:
	CImageSCI();
	virtual ~CImageSCI();

	HRESULT		ReadHeader();
	HRESULT		WriteHeader(PIXELFORMAT, Size width, Size height, BYTE numLevels);
}; // class CImageSCI
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGESCI_H__
