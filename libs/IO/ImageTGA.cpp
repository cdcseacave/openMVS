////////////////////////////////////////////////////////////////////
// ImageTGA.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_TGA
#include "ImageTGA.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

#pragma pack(push, 1)
typedef struct TGAINFOHEADER_TYPE {
	BYTE	byIDLength;				// ID length
	BYTE	byCMType;				// Color map type
	BYTE	byType;					// Image type
	struct {						// Color map specifications
		uint16_t	shCMIdx;		// First entry index, offset into the color map table
		uint16_t	shCMLength;		// Color map length, number of entries
		BYTE	byCMBPP;			// Color map entry size, number of bits per pixel
	};
	struct {						// Image specifications
		uint16_t	shXOrigin;		// X-origin, absolute coordinate of lower-left corner for displays where origin is at the lower left
		uint16_t	shYOrigin;		// Y-origin, as for X-origin
		uint16_t	shWidth;		// Image width, width in pixels
		uint16_t	shHeight;		// Image height, width in pixels
		BYTE	byBPP;				// Pixel depth, bits per pixel
		union {
			BYTE	byAlphaInfo;	// Image descriptor, bits 3-0 give the alpha channel depth, bits 4-5 give direction
			struct {
				BYTE byAlphaBPP	: 3;// alpha channel depth
				BYTE bMirrored	: 1;// alpha channel depth
				BYTE bFlipped	: 1;// alpha channel depth
				BYTE			: 0;// Force alignment to next boundary.
			};
		};
	};
} TGAINFOHEADER;
#pragma pack(pop)



// S T R U C T S ///////////////////////////////////////////////////

CImageTGA::CImageTGA()
{
} // Constructor

CImageTGA::~CImageTGA()
{
} // Destructor
/*----------------------------------------------------------------*/


HRESULT CImageTGA::ReadHeader()
{
	// read header
	((ISTREAM*)m_pStream)->setPos(0);
	TGAINFOHEADER tgaInfo;
	m_pStream->read(&tgaInfo, sizeof(TGAINFOHEADER));
	if (tgaInfo.byCMType != 0) {	// palletted images not supported
		LOG(LT_IMAGE, "error: invalid TGA image");
		return _INVALIDFILE;
	}

	m_dataWidth = m_width = tgaInfo.shWidth;
	m_dataHeight = m_height = tgaInfo.shHeight;
	m_numLevels = 0;
	m_level = 0;

	// get the format
	m_stride = (tgaInfo.byBPP>>3);
	switch (tgaInfo.byType) 
	{
	case 2:	//uncompressed, true-color image
	case 3:	//uncompressed, black-and-white image
		m_format = (m_stride == 3 ? PF_R8G8B8 : PF_R8G8B8A8);
		m_bRLE = false;
		break;
	//case 10://run-length encoded, true-color image and
	//	m_format = IF_3DC;
	//	m_bRLE = true;
	//	break;
	default:
		ASSERT(0);
		LOG(LT_IMAGE, "error: unsupported TGA image");
		return _INVALIDFILE;
	}

	m_lineWidth = m_width * m_stride;

	// read the id stuff, not using seeking to be able to use non seekable sources
	if (tgaInfo.byIDLength) {
		CAutoPtrArr<uint8_t> const buffer(new uint8_t[tgaInfo.byIDLength]);
		m_pStream->read(buffer, tgaInfo.byIDLength);
	}

	return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/


HRESULT CImageTGA::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	// read data
	if (dataFormat == m_format && nStride == m_stride) {
		// read image directly to the data buffer
		(BYTE*&)pData += (m_height-1)*lineWidth;
		for (Size j=0; j<m_height; ++j, (uint8_t*&)pData-=lineWidth)
			if (m_lineWidth != m_pStream->read(pData, m_lineWidth))
				return _INVALIDFILE;
	} else {
		// read image to a buffer and convert it
		CAutoPtrArr<uint8_t> const buffer(new uint8_t[m_lineWidth]);
		for (Size j=0; j<m_height; ++j) {
			if (m_lineWidth != m_pStream->read(buffer, m_lineWidth))
				return _INVALIDFILE;
			if (!FilterFormat((BYTE*)pData+(m_height-j-1)*lineWidth, dataFormat, nStride, buffer, m_format, m_stride, m_width))
				return _FAIL;
		}
	}
	return _OK;
} // ReadData
/*----------------------------------------------------------------*/


HRESULT CImageTGA::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	return _FAIL;
} // WriteHeader
/*----------------------------------------------------------------*/


HRESULT CImageTGA::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	return _FAIL;
} // WriteData
/*----------------------------------------------------------------*/

#endif // _IMAGE_TGA
