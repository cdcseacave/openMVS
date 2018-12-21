////////////////////////////////////////////////////////////////////
// ImagePNG.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_PNG
#include "ImagePNG.h"
#include <png.h>
#include <zlib.h>

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// F U N C T I O N S ///////////////////////////////////////////////

void custom_png_read_file(png_structp png_ptr, png_bytep buffer, png_size_t size)
{
	png_voidp const ptrIO = png_get_io_ptr(png_ptr);
	((InputStream*)ptrIO)->read(buffer, size);
}

void custom_png_write_file(png_structp png_ptr, png_bytep buffer, png_size_t size)
{
	png_voidp const ptrIO = png_get_io_ptr(png_ptr);
	((IOStream*)ptrIO)->write(buffer, size);
}

void custom_png_flush_file(png_structp png_ptr)
{
	png_voidp const ptrIO = png_get_io_ptr(png_ptr);
	((IOStream*)ptrIO)->flush();
}



// S T R U C T S ///////////////////////////////////////////////////

CImagePNG::CImagePNG() : m_png_ptr(NULL), m_info_ptr(NULL)
{
} // Constructor

CImagePNG::~CImagePNG()
{
	Close();
} // Destructor
/*----------------------------------------------------------------*/

void CImagePNG::Close()
{
	if (m_png_ptr) {
		png_structp png_ptr = (png_structp)m_png_ptr;
		png_infop info_ptr = (png_infop)m_info_ptr;
		if (bRead)
			png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
		else
			png_destroy_write_struct(&png_ptr, &info_ptr);
		m_png_ptr = m_info_ptr = NULL;
	}
	CImage::Close();
}
/*----------------------------------------------------------------*/


HRESULT CImagePNG::ReadHeader()
{
	// initialize stuff
	ASSERT(m_png_ptr == NULL);
	m_png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!m_png_ptr) {
		LOG(LT_IMAGE, "error: invalid PNG image (png_create_read_struct)");
		return _INVALIDFILE;
	}
	bRead = true;
	png_structp png_ptr = (png_structp)m_png_ptr;

	m_info_ptr = png_create_info_struct(png_ptr);
	if (!m_info_ptr) {
		LOG(LT_IMAGE, "error: invalid PNG image (png_create_info_struct)");
		return _INVALIDFILE;
	}
	png_infop info_ptr = (png_infop)m_info_ptr;

	if (setjmp(png_jmpbuf(png_ptr)))
		return _INVALIDFILE;

	((ISTREAM*)m_pStream)->setPos(0);
	png_set_read_fn(png_ptr, m_pStream, custom_png_read_file);

	png_read_info(png_ptr, info_ptr);

	png_uint_32 width, height; int bitdepth, colortype;
	png_get_IHDR(png_ptr, info_ptr, &width, &height, &bitdepth, &colortype, NULL, NULL, NULL);

	if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) {
		png_set_tRNS_to_alpha(png_ptr);
		colortype = PNG_COLOR_TYPE_RGB_ALPHA;
	}

	#if 0
	// if it doesn't have a file gamma, don't do any correction ("do no harm")
	if (png_get_valid(png_ptr, info_ptr, PNG_INFO_gAMA)) {
		double gamma = 0;
		const double screen_gamma = 2.2;
		if (png_get_gAMA(png_ptr, info_ptr, &gamma))
			png_set_gamma(png_ptr, screen_gamma, gamma);
	}
	#endif

	switch (colortype)
	{
	case PNG_COLOR_TYPE_GRAY:
		if (bitdepth < 8)
			png_set_expand_gray_1_2_4_to_8(png_ptr);
		m_format = PF_GRAY8;
		m_stride = 1;
		break;
	case PNG_COLOR_TYPE_PALETTE:
		png_set_palette_to_rgb(png_ptr);
		m_format = PF_B8G8R8;
		m_stride = 3;
		break;
	case PNG_COLOR_TYPE_GRAY_ALPHA:
		png_set_gray_to_rgb(png_ptr);
		m_format = PF_B8G8R8;
		m_stride = 3;
		break;
	case PNG_COLOR_TYPE_RGB:
		m_format = PF_B8G8R8;
		m_stride = 3;
		break;
	case PNG_COLOR_TYPE_RGB_ALPHA:
		m_format = PF_B8G8R8A8;
		m_stride = 4;
		break;
	default:
		LOG(LT_IMAGE, "error: unsupported PNG image");
		return _INVALIDFILE;
	}

	if (bitdepth == 16)
		png_set_strip_16(png_ptr);

	png_set_interlace_handling(png_ptr);
	png_read_update_info(png_ptr, info_ptr);

	m_dataWidth	= m_width	= (Size)width;
	m_dataHeight= m_height	= (Size)height;
	m_numLevels	= 0;
	m_level		= 0;
	m_lineWidth	= (Size)png_get_rowbytes(png_ptr, info_ptr);

	return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/


HRESULT CImagePNG::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	png_structp png_ptr = (png_structp)m_png_ptr;

	// read data
	if (nStride == m_stride && (m_format != PF_B8G8R8A8 || (dataFormat != PF_A8R8G8B8 && dataFormat != PF_A8B8G8R8))) {
		if ((m_format == PF_B8G8R8 && dataFormat == PF_R8G8B8) ||
			(m_format == PF_B8G8R8A8 && dataFormat == PF_R8G8B8A8)) {
			// flip R and B from the PNG library
			png_set_bgr(png_ptr);
		} else {
			ASSERT(m_format == dataFormat);
		}
		// read image directly to the data buffer
		for (Size j=0; j<m_height; ++j, (BYTE*&)pData+=lineWidth)
			png_read_row(png_ptr, (png_bytep)pData, NULL);
	} else {
		// read image to a buffer and convert it
		CAutoPtrArr<png_byte> const buffer(new png_byte[m_lineWidth]);
		for (Size j=0; j<m_height; ++j, pData=(uint8_t*)pData+lineWidth) {
			png_read_row(png_ptr, buffer, NULL);
			if (!FilterFormat(pData, dataFormat, nStride, buffer, m_format, m_stride, m_width))
				return _FAIL;
		}
	}

	return _OK;
} // Read
/*----------------------------------------------------------------*/


HRESULT CImagePNG::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE /*numLevels*/)
{
	// initialize stuff
	ASSERT(m_png_ptr == NULL);
	m_png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!m_png_ptr) {
		LOG(LT_IMAGE, "error: can not create PNG image (png_create_write_struct)");
		return _INVALIDFILE;
	}
	bRead = false;
	png_structp png_ptr = (png_structp)m_png_ptr;

	m_info_ptr = png_create_info_struct(png_ptr);
	if (!m_info_ptr) {
		LOG(LT_IMAGE, "error: invalid PNG image (png_create_info_struct)");
		return _INVALIDFILE;
	}
	png_infop info_ptr = (png_infop)m_info_ptr;

	if (setjmp(png_jmpbuf(png_ptr)))
		return _INVALIDFILE;

	// write header
	switch (imageFormat)
	{
	case PF_A8:
	case PF_GRAY8:
		m_stride = 1;
		m_format = PF_GRAY8;
		break;
	case PF_B8G8R8:
	case PF_R8G8B8:
		m_stride = 3;
		m_format = PF_B8G8R8;
		break;
	case PF_R8G8B8A8:
	case PF_A8R8G8B8:
	case PF_B8G8R8A8:
	case PF_A8B8G8R8:
		m_stride = 4;
		m_format = PF_B8G8R8A8;
		break;
	default:
		LOG(LT_IMAGE, "error: unsupported PNG image format");
		return _INVALIDFILE;
	}
	m_numLevels = 1;
	m_level = 0;
	m_width = width;
	m_height = height;
	m_lineWidth = GetDataSizes(0, m_dataWidth, m_dataHeight);

	png_set_write_fn(png_ptr, m_pStream, custom_png_write_file, custom_png_flush_file);

	int compression_level = 0;
	int compression_strategy = Z_RLE;

	if (compression_level > 0) {
		png_set_compression_mem_level(png_ptr, compression_level);
	}
	else {
		// tune parameters for speed
		// (see http://wiki.linuxquestions.org/wiki/Libpng)
		png_set_filter(png_ptr, PNG_FILTER_TYPE_BASE, PNG_FILTER_SUB);
		png_set_compression_level(png_ptr, Z_BEST_SPEED);
	}
	png_set_compression_strategy(png_ptr, compression_strategy);

	png_set_IHDR(png_ptr, info_ptr, m_width, m_height, 8/*depth*/,
		m_stride == 1 ? PNG_COLOR_TYPE_GRAY :
		m_stride == 3 ? PNG_COLOR_TYPE_RGB : PNG_COLOR_TYPE_RGBA,
		PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
		PNG_FILTER_TYPE_DEFAULT);

	png_write_info(png_ptr, info_ptr);

	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/


HRESULT CImagePNG::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	png_structp png_ptr = (png_structp)m_png_ptr;

	// write data
	if (nStride == m_stride && (m_format != PF_B8G8R8A8 || (dataFormat != PF_A8R8G8B8 && dataFormat != PF_A8B8G8R8))) {
		if ((m_format == PF_B8G8R8 && dataFormat == PF_R8G8B8) ||
			(m_format == PF_B8G8R8A8 && dataFormat == PF_R8G8B8A8)) {
			// flip R and B from the PNG library
			png_set_bgr(png_ptr);
		} else {
			ASSERT(m_format == dataFormat);
		}
		// write image directly to the data buffer
		for (Size j=0; j<m_height; ++j, (BYTE*&)pData+=lineWidth)
			png_write_row(png_ptr, (png_bytep)pData);
	} else {
		// read image to a buffer and convert it
		CAutoPtrArr<png_byte> const buffer(new png_byte[m_lineWidth]);
		for (Size j=0; j<m_height; ++j, pData=(uint8_t*)pData+lineWidth) {
			if (!FilterFormat(buffer, m_format, m_stride, pData, dataFormat, nStride, m_width))
				return _FAIL;
			png_write_row(png_ptr, buffer);
		}
	}
	png_write_end(png_ptr, NULL);

	return _OK;
} // WriteData
/*----------------------------------------------------------------*/

#endif // _IMAGE_PNG
