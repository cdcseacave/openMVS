////////////////////////////////////////////////////////////////////
// ImageBMP.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_BMP
#include "ImageBMP.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

#ifndef _MSC_VER
typedef long LONG;
typedef struct tagBITMAPFILEHEADER {
	WORD    bfType;
	DWORD   bfSize;
	WORD    bfReserved1;
	WORD    bfReserved2;
	DWORD   bfOffBits;
} BITMAPFILEHEADER;
typedef struct tagBITMAPINFOHEADER {
	DWORD      biSize;
	LONG       biWidth;
	LONG       biHeight;
	WORD       biPlanes;
	WORD       biBitCount;
	DWORD      biCompression;
	DWORD      biSizeImage;
	LONG       biXPelsPerMeter;
	LONG       biYPelsPerMeter;
	DWORD      biClrUsed;
	DWORD      biClrImportant;
} BITMAPINFOHEADER;
/* constants for the biCompression field */
#define BI_RGB        0L
#define BI_RLE8       1L
#define BI_RLE4       2L
#define BI_BITFIELDS  3L
#define BI_JPEG       4L
#define BI_PNG        5L
#endif


CImageBMP::CImageBMP()
{
} // Constructor

CImageBMP::~CImageBMP()
{
} // Destructor
/*----------------------------------------------------------------*/


HRESULT CImageBMP::ReadHeader()
{
	// Jump to the beginning of the file
	((ISTREAM*)m_pStream)->setPos(0);

	// Read the BITMAPFILEHEADER
	// and check the type field to make sure we have a .bmp file
	// bfType should contain the letters "BM"
	BITMAPFILEHEADER bmp_fileheader;
	if (sizeof(BITMAPFILEHEADER) != m_pStream->read(&bmp_fileheader, sizeof(BITMAPFILEHEADER)) ||
		memcmp(&bmp_fileheader.bfType, "BM", 2))
	{
		LOG(LT_IMAGE, _T("error: invalid BMP image"));
		return _INVALIDFILE;
	}

	// Read the BITMAPINFOHEADER.
	// and double Check to make sure we got a correct BITMAPINFOHEADER
	BITMAPINFOHEADER bmp_infoheader;
	if (sizeof(BITMAPINFOHEADER) != m_pStream->read(&bmp_infoheader, sizeof(BITMAPINFOHEADER)) ||
		bmp_infoheader.biSize != sizeof(bmp_infoheader))
	{
		LOG(LT_IMAGE, _T("error: invalid BMP image"));
		return _INVALIDFILE;
	}

	// Check for unsupported format: biPlanes MUST equal 1
	// and we can only handle uncompressed .bmps
	if (bmp_infoheader.biPlanes != 1 ||
		(bmp_infoheader.biCompression != BI_RGB && bmp_infoheader.biCompression != BI_BITFIELDS))
	{
		LOG(LT_IMAGE, "error: unsupported BMP image");
		return _INVALIDFILE;
	}

	// Initililize our width, height and format as the .bmp we are loading
	m_dataWidth	= m_width	= bmp_infoheader.biWidth;
	m_dataHeight= m_height	= bmp_infoheader.biHeight;
	m_numLevels	= 0;
	m_level		= 0;
	m_lineWidth	= (bmp_fileheader.bfSize-bmp_fileheader.bfOffBits)/bmp_infoheader.biHeight; //bmp_infoheader.biSizeImage may be set to zero for BI_RGB bitmaps
	m_stride	= bmp_infoheader.biBitCount>>3;
	switch (m_stride)
	{
	case 1:
		m_format = PF_GRAY8;
		break;
	case 2:
		m_format = PF_R5G6B5;
		break;
	case 3:
		m_format = PF_R8G8B8;
		break;
	case 4:
		m_format = PF_R8G8B8A8;
		break;
	default:
		LOG(LT_IMAGE, "error: unsupported BMP image");
		return _INVALIDFILE;
	}
	// Ensure m_lineWidth is DWORD aligned
	while ((m_lineWidth%4) != 0) ++m_lineWidth;

	// Jump to the location where the bitmap data is stored
	((ISTREAM*)m_pStream)->setPos(bmp_fileheader.bfOffBits);

	return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/


HRESULT CImageBMP::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	// read data
	const size_t nSize = m_width*m_stride;
	const size_t nPad = m_lineWidth-nSize;
	CAutoPtrArr<uint8_t> const bufferPad(nPad ? new uint8_t[nPad] : NULL);
	if (dataFormat == m_format && nStride == m_stride) {
		// read image directly to the data buffer
		(BYTE*&)pData += (m_height-1)*lineWidth;
		for (Size j=0; j<m_height; ++j,(uint8_t*&)pData-=lineWidth)
			if (nSize != m_pStream->read(pData, nSize) ||
				(nPad && nPad != m_pStream->read(bufferPad, nPad)))
				return _INVALIDFILE;
	} else {
		// read image to a buffer and convert it
		CAutoPtrArr<uint8_t> const buffer(new uint8_t[m_lineWidth]);
		for (Size j=0; j<m_height; ++j) {
			if (m_lineWidth != m_pStream->read(buffer, m_lineWidth))
				return _INVALIDFILE;
			if (!FilterFormat((uint8_t*)pData+(m_height-j-1)*lineWidth, dataFormat, nStride, buffer, m_format, m_stride, m_width))
				return _FAIL;
		}
	}
	return _OK;
} // ReadData
/*----------------------------------------------------------------*/


HRESULT CImageBMP::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE /*numLevels*/)
{
	// write header
	m_numLevels = 0;
	m_level = 0;
	DWORD dwCompression;
	switch (imageFormat)
	{
	case PF_A8:
	case PF_GRAY8:
		m_stride = 1;
		dwCompression = BI_RGB;
		m_format = PF_GRAY8;
		break;
	case PF_R5G6B5:
		m_stride = 2;
		dwCompression = BI_BITFIELDS;
		m_format = PF_R5G6B5;
		break;
	case PF_B8G8R8:
	case PF_R8G8B8:
		m_stride = 3;
		dwCompression = BI_RGB;
		m_format = PF_R8G8B8;
		break;
	case PF_R8G8B8A8:
	case PF_A8R8G8B8:
	case PF_B8G8R8A8:
	case PF_A8B8G8R8:
		m_stride = 4;
		dwCompression = BI_BITFIELDS;
		m_format = PF_R8G8B8A8;
		break;
	default:
		LOG(LT_IMAGE, "error: unsupported BMP image format");
		return _INVALIDFILE;
	}
	m_dataWidth	= m_width = width;
	m_dataHeight= m_height = height;
	m_lineWidth	= m_width*m_stride;
	// Ensure m_lineWidth is DWORD aligned
	while ((m_lineWidth%4) != 0) ++m_lineWidth;
	((OSTREAM*)m_pStream)->setPos(0);
	// Write the BITMAPFILEHEADER and the BITMAPINFOHEADER.
	const BITMAPFILEHEADER bmp_fileheader = {
		0x4d42, //BM
		(DWORD)sizeof(BITMAPFILEHEADER)+(DWORD)sizeof(BITMAPINFOHEADER)+m_lineWidth*m_height, //total file size
		0, 0, //reserved
		(DWORD)sizeof(BITMAPFILEHEADER)+(DWORD)sizeof(BITMAPINFOHEADER) //offset to the bitmap bits
	};
	const BITMAPINFOHEADER bmp_infoheader = {
		(DWORD)sizeof(BITMAPINFOHEADER), //size of this structure
		(LONG)m_width, (LONG)m_height, //dim
		(WORD)1, //number of color planes being used (must be set to 1)
		(WORD)(m_stride<<3), //number of bits per pixel
		dwCompression, //compression method being used
		m_lineWidth*m_height, //raw image size
		0, 0, //pixels-per-meter
		0, //number of colors in the color palette
		0 //number of important colors used
	};
	if (sizeof(BITMAPFILEHEADER) != m_pStream->write(&bmp_fileheader, sizeof(BITMAPFILEHEADER)) ||
		sizeof(BITMAPINFOHEADER) != m_pStream->write(&bmp_infoheader, sizeof(BITMAPINFOHEADER)))
	{
		LOG(LT_IMAGE, "error: failed writing the BMP image");
		return _INVALIDFILE;
	}
	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/


HRESULT CImageBMP::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	// write data
	const size_t nSize = m_width*m_stride;
	const size_t nPad = m_lineWidth-nSize;
	CAutoPtrArr<uint8_t> const bufferPad(nPad ? new uint8_t[nPad] : NULL);
	if (nPad) memset(bufferPad, 0, nPad);
	if (dataFormat == m_format && nStride == m_stride) {
		// write data buffer directly to the image
		for (Size j=0; j<m_height; ++j)
			if (nSize != m_pStream->write((uint8_t*)pData+(m_height-j-1)*lineWidth, nSize) ||
				(nPad && nPad != m_pStream->write(bufferPad, nPad)))
				return _INVALIDFILE;
	} else {
		// convert data to a buffer and write it
		CAutoPtrArr<uint8_t> const buffer(new uint8_t[m_lineWidth]);
		for (Size j=0; j<m_height; ++j) {
			if (!FilterFormat(buffer, m_format, m_stride, (uint8_t*)pData+(m_height-j-1)*lineWidth, dataFormat, nStride, m_width))
				return _FAIL;
			if (m_lineWidth != m_pStream->write(buffer, m_lineWidth))
				return _INVALIDFILE;
		}
	}
	return _OK;
} // WriteData
/*----------------------------------------------------------------*/

#endif // _IMAGE_BMP
