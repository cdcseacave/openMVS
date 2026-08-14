////////////////////////////////////////////////////////////////////
// ImageHEIF.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGEHEIF_H__
#define __SEACAVE_IMAGEHEIF_H__


// D E F I N E S ///////////////////////////////////////////////////


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class IO_API CImageHEIF : public CImage
{
public:
	CImageHEIF();
	virtual ~CImageHEIF();

	void		Close();

	bool		ReadHeader();
	bool		ReadData(void*, PIXELFORMAT, Size nStride, Size lineWidth);
	bool		WriteHeader(PIXELFORMAT, Size width, Size height, BYTE numLevels);
	bool		WriteData(void*, PIXELFORMAT, Size nStride, Size lineWidth);

	virtual bool	GetMetadataEXIF(std::vector<uint8_t>& blob) const override;

	#ifdef _USE_TESTS
	// Self-test of the reader, living next to the code it covers: decoded dimensions
	// (container irot applied), absolute channel order, alpha decoded only on demand,
	// the EXIF blob bridge and the write refusal.
	// `folder` holds the fixtures (apps/Tests/data/images/heif).
	static bool	Test(const String& folder);
	#endif

protected:
	void*		m_state; // placeholder for libheif context/handle state
}; // class CImageHEIF
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGEHEIF_H__
