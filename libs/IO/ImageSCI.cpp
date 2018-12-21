////////////////////////////////////////////////////////////////////
// ImageSCI.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ImageSCI.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

#define IMAGE_SCI_TYPE		0x494353	//"SCI" = 3 bytes
#define IMAGE_SCI_VERSION	1			//1 byte
#define IMAGE_SCI_HEADER	((DWORD)((((DWORD)IMAGE_SCI_VERSION)<<24)|(DWORD)IMAGE_SCI_TYPE))	//"SCI"+ver = 4 bytes


// S T R U C T S ///////////////////////////////////////////////////

typedef struct SCIINFOHEADER_TYPE {
	uint32_t dwHeader;
	uint16_t shWidth;
	uint16_t shHeight;
	uint8_t  byFormat;
	uint8_t  byLevels;
	uint8_t  byReserved1;
	uint8_t  byReserved2;
} SCIINFOHEADER;


CImageSCI::CImageSCI()
{
} // Constructor

CImageSCI::~CImageSCI()
{
} // Destructor
/*----------------------------------------------------------------*/


HRESULT CImageSCI::ReadHeader()
{
	// read header
	((ISTREAM*)m_pStream)->setPos(0);
	SCIINFOHEADER sciInfo;
	m_pStream->read(&sciInfo, sizeof(SCIINFOHEADER));
	if (sciInfo.dwHeader != IMAGE_SCI_HEADER) {
		LOG(LT_IMAGE, "error: invalid SCI image");
		return _INVALIDFILE;
	}
	m_width = sciInfo.shWidth;
	m_height = sciInfo.shHeight;
	m_numLevels = sciInfo.byLevels;
	m_level = 0;
	m_format = (PIXELFORMAT)sciInfo.byFormat;
	m_stride = GetStride(m_format);
	m_lineWidth = GetDataSizes(0, m_dataWidth, m_dataHeight);
	return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/


HRESULT CImageSCI::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	// write header
	CImage::WriteHeader(imageFormat, width, height, numLevels);
	((OSTREAM*)m_pStream)->setPos(0);
	const SCIINFOHEADER sciInfo = {IMAGE_SCI_HEADER, (uint16_t)m_width, (uint16_t)m_height, (uint8_t)m_format, m_numLevels, 0, 0};
	if (sizeof(SCIINFOHEADER) != m_pStream->write(&sciInfo, sizeof(SCIINFOHEADER))) {
		LOG(LT_IMAGE, "error: failed writing the SCI image");
		return _INVALIDFILE;
	}
	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/
