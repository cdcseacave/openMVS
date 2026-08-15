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
	~CImageHEIF() override;

	void		Close() override;

	bool		ReadHeader() override;
	bool		ReadData(void*, PIXELFORMAT, Size nStride, Size lineWidth) override;
	bool		WriteHeader(PIXELFORMAT, Size width, Size height, BYTE numLevels) override;
	bool		WriteData(void*, PIXELFORMAT, Size nStride, Size lineWidth) override;

	virtual bool	GetMetadataEXIF(std::vector<uint8_t>& blob) const override;

	#ifdef _USE_TESTS
	// Self-test of the reader, living next to the code it covers: decoded dimensions
	// (container irot applied), absolute channel order, alpha decoded only on demand,
	// the EXIF blob bridge and the write refusal.
	// `folder` holds the test images (apps/Tests/data/images).
	static bool	Test(const String& folder);
	#endif

protected:
	void*		m_state; // opaque HeifState (file buffer + libheif context/handle), so that
						 // no libheif header is needed to include this one
}; // class CImageHEIF
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGEHEIF_H__
