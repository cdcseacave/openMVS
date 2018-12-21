////////////////////////////////////////////////////////////////////
// Image.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "Image.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG(CImage, _T("IO      "));

// Set the image details;
// if image's data is not NULL, but its size is too small,
// data's buffer is not allocated and _BUFFERSIZE is returned.
HRESULT CImage::Reset(Size width, Size height, PIXELFORMAT pixFormat, Size levels, bool bAllocate)
{
	// reinitialize image with given params
	const size_t oldDataSize = GetDataSize();
	m_dataWidth	= m_width	= width;
	m_dataHeight= m_height	= height;
	m_format	= pixFormat;
	m_stride	= GetStride(pixFormat);
	m_lineWidth	= m_width * m_stride;
	m_numLevels	= levels;
	m_level		= 0;
	if (bAllocate) {
		if (m_data != NULL) {
			if (oldDataSize < GetDataSize())
				return _BUFFERSIZE;
		} else {
			m_data	= new uint8_t[GetDataSize()];
		}
	}
	return _OK;
} // Reset
/*----------------------------------------------------------------*/

HRESULT CImage::Reset(LPCTSTR szFileName, IMCREATE mode)
{
	// open the new image stream
	m_fileName = szFileName;
	// try to directly access the file
	File* f;
	if (mode == READ) {
		f = new File(szFileName, File::READ, File::OPEN);
	} else {
		Util::ensureFolder(szFileName);
		f = new File(szFileName, File::WRITE, File::CREATE | File::TRUNCATE);
	}
	if (!f->isOpen()) {
		delete f;
		return _INVALIDFILE;
	}
	if ((m_pStream = f) == NULL) {
		LOG(LT_IMAGE, _T("error: failed opening image '%s'"), szFileName);
		return _INVALIDFILE;
	}
	return _OK;
} // Reset
/*----------------------------------------------------------------*/

HRESULT CImage::Reset(IOSTREAMPTR& pStream)
{
	// use the already opened image stream
	m_fileName.clear();
	m_pStream = pStream;
	return _OK;
} // Reset
/*----------------------------------------------------------------*/

void CImage::Close()
{
	// close the image stream
	m_pStream.Release();
	m_data.Release();
} // Close
/*----------------------------------------------------------------*/


HRESULT CImage::ReadHeader()
{
	return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/

HRESULT CImage::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	// read data
	if (dataFormat == m_format && nStride == m_stride) {
		// read image directly to the data buffer
		if (lineWidth == m_lineWidth) {
			const size_t nSize = m_dataHeight*m_lineWidth;
			if (nSize != m_pStream->read(pData, nSize))
				return _INVALIDFILE;
		} else {
			for (Size j=0; j<m_dataHeight; ++j, (uint8_t*&)pData+=lineWidth)
				if (m_lineWidth != m_pStream->read(pData, m_lineWidth))
					return _INVALIDFILE;
		}
	} else {
		// read image to a buffer and convert it
		CAutoPtrArr<uint8_t> const buffer(new uint8_t[m_lineWidth]);
		for (Size j=0; j<m_dataHeight; ++j, (uint8_t*&)pData+=lineWidth) {
			if (m_lineWidth != m_pStream->read(buffer, m_lineWidth))
				return _INVALIDFILE;
			if (!FilterFormat(pData, dataFormat, nStride, buffer, m_format, m_stride, m_dataWidth))
				return _FAIL;
		}
	}
	// prepare next level
	if (m_level+1 < m_numLevels)
		m_lineWidth = GetDataSizes(++m_level, m_dataWidth, m_dataHeight);
	return _OK;
} // ReadData
/*----------------------------------------------------------------*/


HRESULT CImage::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	// write header
	m_numLevels = numLevels;
	m_level = 0;
	m_format = imageFormat;
	m_stride = GetStride(m_format);
	m_width = width;
	m_height = height;
	m_lineWidth = GetDataSizes(0, m_dataWidth, m_dataHeight);
	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/

HRESULT CImage::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	// write data
	if (dataFormat == m_format && nStride == m_stride) {
		// write data buffer directly to the image
		if (lineWidth == m_lineWidth) {
			const size_t nSize = m_dataHeight*m_lineWidth;
			if (nSize != m_pStream->write(pData, nSize))
				return _INVALIDFILE;
		} else {
			for (Size j=0; j<m_dataHeight; ++j, (uint8_t*&)pData+=lineWidth)
				if (m_lineWidth != m_pStream->write(pData, m_lineWidth))
					return _INVALIDFILE;
		}
	} else {
		// convert data to a buffer and write it
		CAutoPtrArr<uint8_t> const buffer(new uint8_t[m_lineWidth]);
		for (Size j=0; j<m_dataHeight; ++j, (uint8_t*&)pData+=lineWidth) {
			if (!FilterFormat(buffer, m_format, m_stride, pData, dataFormat, nStride, m_dataWidth))
				return _FAIL;
			if (m_lineWidth != m_pStream->write(buffer, m_lineWidth))
				return _INVALIDFILE;
		}
	}
	// prepare next level
	if (m_level+1 < m_numLevels)
		m_lineWidth = GetDataSizes(++m_level, m_dataWidth, m_dataHeight);
	return _OK;
} // WriteData
/*----------------------------------------------------------------*/


CImage::Size CImage::GetDataSizes(Size mipLevel, Size& width, Size& height) const
{
	// Bump by the mip level.
	height = MAXF((Size)1, m_height >> mipLevel);
	width  = MAXF((Size)1, m_width >> mipLevel);

	// if compressed, divide dims by 4
	if (m_format >= PF_DXT1)
	{
		width = (width+3)>>2;
		height = (height+3)>>2;
	}

	return width * m_stride;
} // GetDataSizes
/*----------------------------------------------------------------*/


CImage::Size CImage::GetStride(PIXELFORMAT pixFormat)
{
	switch (pixFormat)
	{
	case PF_A8:
	case PF_GRAY8:
		return 1;
	case PF_R5G6B5:
		return 2;
	case PF_B8G8R8:
	case PF_R8G8B8:
		return 3;
	case PF_R8G8B8A8:
	case PF_A8R8G8B8:
	case PF_B8G8R8A8:
	case PF_A8B8G8R8:
		return 4;
	case PF_DXT1:
		return 8;
	case PF_DXT2:
	case PF_DXT3:
	case PF_DXT4:
	case PF_DXT5:
	case PF_3DC:
		return 16;
	default:
		LOG(LT_IMAGE, "error: unsupported SCI pixel format");
	}
	return 0;
} // GetStride
/*----------------------------------------------------------------*/


// If the given image format has alpha channel, returns true.
bool CImage::FormatHasAlpha(PIXELFORMAT format)
{
	switch (format)
	{
	case PF_A8:
	case PF_R8G8B8A8:
	case PF_A8R8G8B8:
	case PF_B8G8R8A8:
	case PF_A8B8G8R8:
	case PF_DXT2:
	case PF_DXT3:
	case PF_DXT4:
	case PF_DXT5:
		return true;
	case PF_GRAY8:
	case PF_R5G6B5:
	case PF_B8G8R8:
	case PF_R8G8B8:
	case PF_DXT1:
	case PF_3DC:
		return false;
	default:
		ASSERT("Unknown format" == NULL);
	}
	return false;
} // FormatHasAlpha
/*----------------------------------------------------------------*/


// Convert from one format to another.
// nSzize is the number of pixels to process.
bool CImage::FilterFormat(void* pDst, PIXELFORMAT formatDst, Size strideDst, const void* pSrc, PIXELFORMAT formatSrc, Size strideSrc, Size nSzize)
{
	//ASSERT(formatDst != formatSrc || strideDst != strideSrc)
	switch (formatDst)
	{
	case PF_A8:
	case PF_GRAY8:
		switch (formatSrc)
		{
		case PF_R8G8B8A8:
		case PF_A8R8G8B8:
		case PF_B8G8R8A8:
		case PF_A8B8G8R8:
			// from PF_R8G8B8A8 to PF_A8 (just copy the alpha channel)
			(uint8_t*&)pSrc += 3; //skip the first RGB values
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_A8 (just copy)
			ASSERT(strideDst != strideSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[0];
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_A8 (16bits to gray)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				const uint16_t clr = *((uint16_t*)pSrc);
				((uint8_t*)pDst)[0] = RGB24TO8(RGB16TOR(clr), RGB16TOG(clr), RGB16TOB(clr));
			}
			return true;

		case PF_B8G8R8:
		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_A8 (24bits to gray)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint8_t*)pDst)[0] = RGB24TO8(((uint8_t*)pSrc)[2], ((uint8_t*)pSrc)[1], ((uint8_t*)pSrc)[0]);
			return true;
		}
		break;

	case PF_R5G6B5:
		switch (formatSrc)
		{
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_R5G6B5 (doesn't make sense)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				*((uint16_t*)pDst) = RGB24TO16(((uint8_t*)pSrc)[0], ((uint8_t*)pSrc)[0], ((uint8_t*)pSrc)[0]);
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_R5G6B5 (just copy)
			ASSERT(strideDst != strideSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				*((uint16_t*)pDst) = *((uint16_t*)pSrc);
			return true;

		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_R5G6B5 (24bits to 16bits)
		case PF_R8G8B8A8:
			// from PF_R8G8B8A8 to PF_R5G6B5 (24bits to 16bits)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				*((uint16_t*)pDst) = RGB24TO16(((uint8_t*)pSrc)[2], ((uint8_t*)pSrc)[1], ((uint8_t*)pSrc)[0]);
			return true;

		case PF_A8R8G8B8:
			// from PF_A8R8G8B8 to PF_R5G6B5 (24bits to 16bits)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				*((uint16_t*)pDst) = RGB24TO16(((uint8_t*)pSrc)[3], ((uint8_t*)pSrc)[2], ((uint8_t*)pSrc)[1]);
			return true;

		case PF_B8G8R8:
			// from PF_B8G8R8 to PF_R5G6B5 (24bits to 16bits)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				*((uint16_t*)pDst) = RGB24TO16(((uint8_t*)pSrc)[0], ((uint8_t*)pSrc)[1], ((uint8_t*)pSrc)[2]);
			return true;

		case PF_B8G8R8A8:
			// from PF_B8G8R8A8 to PF_R5G6B5 (24bits to 16bits)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				*((uint16_t*)pDst) = RGB24TO16(((uint8_t*)pSrc)[0], ((uint8_t*)pSrc)[1], ((uint8_t*)pSrc)[2]);
			return true;

		case PF_A8B8G8R8:
			// from PF_A8B8G8R8 to PF_R5G6B5 (24bits to 16bits)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				*((uint16_t*)pDst) = RGB24TO16(((uint8_t*)pSrc)[1], ((uint8_t*)pSrc)[2], ((uint8_t*)pSrc)[3]);
			return true;
		}
		break;

	case PF_R8G8B8:
		switch (formatSrc)
		{
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_R8G8B8 (doesn't make sense)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint8_t*)pDst)[0] = ((uint8_t*)pDst)[1] = ((uint8_t*)pDst)[2] = *((uint8_t*)pSrc);
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_R8G8B8 (16bits to 24bits))
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				const uint16_t clr = *((uint16_t*)pSrc);
				((uint8_t*)pDst)[0] = RGB16TOB(clr);
				((uint8_t*)pDst)[1] = RGB16TOG(clr);
				((uint8_t*)pDst)[2] = RGB16TOR(clr);
			}
			return true;

		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_R8G8B8 (just copy)
			ASSERT(strideDst != strideSrc);
		case PF_R8G8B8A8:
			// from PF_R8G8B8A8 to PF_R8G8B8 (just copy)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint16_t*)pDst)[0] = ((uint16_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[2];
			}
			return true;

		case PF_A8R8G8B8:
			// from PF_A8R8G8B8 to PF_R8G8B8 (just copy)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[3];
			}
			return true;

		case PF_B8G8R8:
			// from PF_B8G8R8 to PF_R8G8B8 (flip)
		case PF_B8G8R8A8:
			// from PF_B8G8R8A8 to PF_R8G8B8 (flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_A8B8G8R8:
			// from PF_A8B8G8R8 to PF_R8G8B8 (flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
			}
			return true;
		}
		break;

	case PF_R8G8B8A8:
		switch (formatSrc)
		{
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_R8G8B8A8 (doesn't make sense)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pDst)[1] = ((uint8_t*)pDst)[2] = *((uint8_t*)pSrc);
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_R8G8B8A8 (16bits to 24bits))
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				const uint16_t clr = *((uint16_t*)pSrc);
				((uint8_t*)pDst)[0] = RGB16TOB(clr);
				((uint8_t*)pDst)[1] = RGB16TOG(clr);
				((uint8_t*)pDst)[2] = RGB16TOR(clr);
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_R8G8B8A8 (just copy the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint16_t*)pDst)[0] = ((uint16_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_R8G8B8A8:
			// from PF_R8G8B8A8 to PF_R8G8B8A8 (just copy)
			ASSERT(strideDst != strideSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint32_t*)pDst)[0] = ((uint32_t*)pSrc)[0];
			return true;

		case PF_A8R8G8B8:
			// from PF_A8R8G8B8 to PF_R8G8B8A8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_B8G8R8:
			// from PF_B8G8R8 to PF_R8G8B8A8 (copy and flip the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_B8G8R8A8:
			// from PF_B8G8R8A8 to PF_R8G8B8A8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[3];
			}
			return true;

		case PF_A8B8G8R8:
			// from PF_A8B8G8R8 to PF_R8G8B8A8 (flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;
		}
		break;

	case PF_A8R8G8B8:
		switch (formatSrc)
		{
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_A8R8G8B8 (doesn't make sense)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = ((uint8_t*)pDst)[2] = ((uint8_t*)pDst)[3] = *((uint8_t*)pSrc);
			}
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_A8R8G8B8 (16bits to 24bits))
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				const uint16_t clr = *((uint16_t*)pSrc);
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = RGB16TOB(clr);
				((uint8_t*)pDst)[2] = RGB16TOG(clr);
				((uint8_t*)pDst)[3] = RGB16TOR(clr);
			}
			return true;

		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_A8R8G8B8 (just copy the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[2];
			}
			return true;

		case PF_R8G8B8A8:
			// from PF_R8G8B8A8 to PF_A8R8G8B8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[2];
			}
			return true;

		case PF_A8R8G8B8:
			// from PF_A8R8G8B8 to PF_A8R8G8B8 (just copy)
			ASSERT(strideDst != strideSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint32_t*)pDst)[0] = ((uint32_t*)pSrc)[0];
			return true;

		case PF_B8G8R8:
			// from PF_B8G8R8 to PF_A8R8G8B8 (copy and flip the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_B8G8R8A8:
			// from PF_B8G8R8A8 to PF_A8R8G8B8 (flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_A8B8G8R8:
			// from PF_A8B8G8R8 to PF_A8R8G8B8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[1];
			}
			return true;
		}
		break;

	case PF_B8G8R8:
		switch (formatSrc)
		{
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_B8G8R8 (doesn't make sense)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint8_t*)pDst)[0] = ((uint8_t*)pDst)[1] = ((uint8_t*)pDst)[2] = *((uint8_t*)pSrc);
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_B8G8R8 (16bits to 24bits))
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				const uint16_t clr = *((uint16_t*)pSrc);
				((uint8_t*)pDst)[0] = RGB16TOR(clr);
				((uint8_t*)pDst)[1] = RGB16TOG(clr);
				((uint8_t*)pDst)[2] = RGB16TOB(clr);
			}
			return true;

		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_B8G8R8 (flip)
		case PF_R8G8B8A8:
			// from PF_R8G8B8A8 to PF_B8G8R8 (flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_A8R8G8B8:
			// from PF_A8R8G8B8 to PF_B8G8R8 (flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
			}
			return true;

		case PF_B8G8R8:
			// from PF_B8G8R8 to PF_B8G8R8 (just copy)
			ASSERT(strideDst != strideSrc);
		case PF_B8G8R8A8:
			// from PF_B8G8R8A8 to PF_B8G8R8 (just copy)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint16_t*)pDst)[0] = ((uint16_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[2];
			}
			return true;

		case PF_A8B8G8R8:
			// from PF_A8B8G8R8 to PF_B8G8R8 (just copy)
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[3];
			}
			return true;
		}
		break;

	case PF_B8G8R8A8:
		switch (formatSrc)
		{
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_B8G8R8A8 (doesn't make sense)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pDst)[1] = ((uint8_t*)pDst)[2] = *((uint8_t*)pSrc);
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_B8G8R8A8 (16bits to 24bits))
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				const uint16_t clr = *((uint16_t*)pSrc);
				((uint8_t*)pDst)[0] = RGB16TOR(clr);
				((uint8_t*)pDst)[1] = RGB16TOG(clr);
				((uint8_t*)pDst)[2] = RGB16TOB(clr);
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_B8G8R8A8 (copy and flip the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_R8G8B8A8:
			// from PF_R8G8B8A8 to PF_B8G8R8A8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[3];
			}
			return true;

		case PF_A8R8G8B8:
			// from PF_A8R8G8B8 to PF_B8G8R8A8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_B8G8R8:
			// from PF_B8G8R8 to PF_B8G8R8A8 (just copy the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint16_t*)pDst)[0] = ((uint16_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[3] = 0xFF;
			}
			return true;

		case PF_B8G8R8A8:
			// from PF_B8G8R8A8 to PF_B8G8R8A8 (just copy)
			ASSERT(strideDst != strideSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint32_t*)pDst)[0] = ((uint32_t*)pSrc)[0];
			return true;

		case PF_A8B8G8R8:
			// from PF_A8B8G8R8 to PF_B8G8R8A8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;
		}
		break;

	case PF_A8B8G8R8:
		switch (formatSrc)
		{
		case PF_A8:
		case PF_GRAY8:
			// from PF_A8 to PF_A8B8G8R8 (doesn't make sense)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = ((uint8_t*)pDst)[2] = ((uint8_t*)pDst)[3] = *((uint8_t*)pSrc);
			}
			return true;

		case PF_R5G6B5:
			// from PF_R5G6B5 to PF_A8B8G8R8 (16bits to 24bits))
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				const uint16_t clr = *((uint16_t*)pSrc);
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = RGB16TOR(clr);
				((uint8_t*)pDst)[2] = RGB16TOG(clr);
				((uint8_t*)pDst)[3] = RGB16TOB(clr);
			}
			return true;

		case PF_R8G8B8:
			// from PF_R8G8B8 to PF_A8B8G8R8 (copy and flip the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_R8G8B8A8:
			// from PF_R8G8B8A8 to PF_A8B8G8R8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_A8R8G8B8:
			// from PF_A8R8G8B8 to PF_A8B8G8R8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[2];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[0];
			}
			return true;

		case PF_B8G8R8:
			// from PF_B8G8R8 to PF_A8B8G8R8 (just copy the RGB and set A to 255)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = 0xFF;
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[2];
			}
			return true;

		case PF_B8G8R8A8:
			// from PF_B8G8R8A8 to PF_A8B8G8R8 (copy and flip)
			ASSERT(pDst != pSrc);
			for (Size i=0; i<nSzize; ++i, (uint8_t*&)pDst+=strideDst, (uint8_t*&)pSrc+=strideSrc) {
				((uint8_t*)pDst)[0] = ((uint8_t*)pSrc)[3];
				((uint8_t*)pDst)[1] = ((uint8_t*)pSrc)[0];
				((uint8_t*)pDst)[2] = ((uint8_t*)pSrc)[1];
				((uint8_t*)pDst)[3] = ((uint8_t*)pSrc)[2];
			}
			return true;

		case PF_A8B8G8R8:
			// from PF_A8B8G8R8 to PF_A8B8G8R8 (just copy)
			ASSERT(strideDst != strideSrc);
			for (Size i=0; i<nSzize; ++i,(uint8_t*&)pDst+=strideDst,(uint8_t*&)pSrc+=strideSrc)
				((uint32_t*)pDst)[0] = ((uint32_t*)pSrc)[0];
			return true;
		}
		break;
	}
	return false;
} // FilterFormat
/*----------------------------------------------------------------*/


// Flip R and B in a 24bit pixel color.
// size is the number of pixels to process.
// stride is the number of bytes used for one pixel.
void CImage::FlipRB24(uint8_t* data, Size size, Size stride)
{
	for (Size i=0; i<size; ++i, data+=stride)
	{
		const uint8_t tmp = data[0];
		data[0] = data[2];
		data[2] = tmp;
	}
} // FlipRB24
/*----------------------------------------------------------------*/


// Copy and flip R and B in a 24bit pixel color.
// size is the number of pixels to process.
// stride is the number of bytes used for one pixel for dest and source respectively.
void CImage::CopyFlipRB24(uint8_t* pDst, const uint8_t* pSrc, Size size, Size strideDst, Size strideSrc)
{
	for (Size i=0; i<size; ++i, pDst+=strideDst, pSrc+=strideSrc)
	{
		pDst[0] = pSrc[2];
		pDst[1] = pSrc[1];
		pDst[2] = pSrc[0];
	}
} // CopyFlipRB24
/*----------------------------------------------------------------*/


CImage* CImage::Create(LPCTSTR szName, IMCREATE mode)
{
	// check image file type
	CImage* pImage;
	LPCTSTR const fext = _tcsrchr(szName, '.');
	if (fext == NULL)
		goto UNKNOWN_FORMAT;
	if (_tcsncicmp(fext, _T(".sci"), 4) == 0)
		pImage = new CImageSCI();
	#ifdef _IMAGE_BMP
	else if (_tcsncicmp(fext, _T(".bmp"), 4) == 0)
		pImage = new CImageBMP();
	#endif
	#ifdef _IMAGE_TGA
	else if (_tcsncicmp(fext, _T(".tga"), 4) == 0)
		pImage = new CImageTGA();
	#endif
	#ifdef _IMAGE_DDS
	else if (_tcsncicmp(fext, _T(".dds"), 4) == 0)
		pImage = new CImageDDS();
	#endif
	#ifdef _IMAGE_PNG
	else if (_tcsncicmp(fext, _T(".png"), 4) == 0)
		pImage = new CImagePNG();
	#endif
	#ifdef _IMAGE_JPG
	else if (_tcsncicmp(fext, _T(".jpg"), 4) == 0)
		pImage = new CImageJPG();
	#endif
	#ifdef _IMAGE_TIFF
	else if (_tcsncicmp(fext, _T(".tif"), 4) == 0 || _tcsncicmp(fext, _T(".tiff"), 5) == 0)
		pImage = new CImageTIFF();
	#endif
	else
		goto UNKNOWN_FORMAT;

	// open the image stream
	if (FAILED(pImage->Reset(szName, mode))) {
		delete pImage;
		return NULL;
	}

	return pImage;

UNKNOWN_FORMAT:
	// Unknown format
	LOG(LT_IMAGE, "error: unknown image format '%s'", szName);
	return NULL;
} // Create
/*----------------------------------------------------------------*/


#ifndef _RELEASE

// Save image as raw data.
void CImage::Dump(LPCTSTR szFileName)
{
	const String strName = Util::getFileName(szFileName)+String::ToString(m_width)+_T("x")+String::ToString(m_height)+_T("x")+String::ToString(m_stride)+Util::getFileExt(szFileName);
	File f(strName, File::WRITE, File::CREATE | File::TRUNCATE);
	if (!f.isOpen())
		return;
	f.write(GetData(), GetDataSize());
} // FlipRB24
/*----------------------------------------------------------------*/

#endif
