////////////////////////////////////////////////////////////////////
// ImageEXIF.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGEEXIV_H__
#define __SEACAVE_IMAGEEXIV_H__


// D E F I N E S ///////////////////////////////////////////////////

struct Exiv2Struct;


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class IO_API CImageEXIF : public CImage
{
public:
	CImageEXIF();
	virtual ~CImageEXIF();

	void		Close();

	HRESULT		ReadHeader();
	HRESULT		ReadData(void*, PIXELFORMAT, Size nStride, Size lineWidth);
	HRESULT		WriteHeader(PIXELFORMAT, Size width, Size height, BYTE numLevels);
	HRESULT		WriteData(void*, PIXELFORMAT, Size nStride, Size lineWidth);

	bool		HasEXIF() const;
	bool		HasIPTC() const;
	bool		HasXMP() const;

	String		ReadKeyEXIF(const String& name, bool bInterpret=true) const;

	void		DumpAll();

protected:
	CAutoPtr<Exiv2Struct>	m_state;
}; // class CImageEXIF
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGEEXIV_H__
