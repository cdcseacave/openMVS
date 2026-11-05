////////////////////////////////////////////////////////////////////
// ImageTIFF.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGETIFF_H__
#define __SEACAVE_IMAGETIFF_H__


// D E F I N E S ///////////////////////////////////////////////////


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class IO_API CImageTIFF : public CImage
{
public:
	CImageTIFF();
	~CImageTIFF() override;

	void		Close() override;

	bool		ReadHeader() override;
	bool		ReadData(void*, PIXELFORMAT, Size nStride, Size lineWidth) override;
	bool		WriteHeader(PIXELFORMAT, Size width, Size height, BYTE numLevels) override;
	bool		WriteData(void*, PIXELFORMAT, Size nStride, Size lineWidth) override;

	#ifdef _USE_TESTS
	static bool	Test(const String& folder);
	#endif

protected:
	void*		m_state;
}; // class CImageTIFF
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGETIFF_H__
