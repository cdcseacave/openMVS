////////////////////////////////////////////////////////////////////
// ImageJPG.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_JPG
#include "ImageJPG.h"

#ifdef _MSC_VER
#define XMD_H // prevent redefinition of INT32
#undef FAR  // prevent FAR redefinition
#endif

extern "C" {
#include <jpeglib.h>
}
#include <setjmp.h>

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

#define JPG_BUFFER_SIZE	(16*1024)

struct JpegErrorMgr
{
	struct jpeg_error_mgr pub;
	jmp_buf setjmp_buffer;
};

struct JpegSource
{
	struct jpeg_source_mgr pub;
	ISTREAM* pStream;
	JOCTET* buffer;
};

struct JpegState
{
	jpeg_decompress_struct cinfo; // IJG JPEG codec structure
	JpegErrorMgr jerr;// error processing manager state
	JpegSource source;// memory buffer source
};


// F U N C T I O N S ///////////////////////////////////////////////

METHODDEF(void)
stub(j_decompress_ptr cinfo)
{
	JpegSource* source = (JpegSource*)cinfo->src;
	source->pStream->setPos(0);
}

METHODDEF(boolean)
fill_input_buffer(j_decompress_ptr cinfo)
{
	JpegSource* source = (JpegSource*)cinfo->src;
	const size_t size = source->pStream->read(source->buffer, JPG_BUFFER_SIZE);
	if (size == STREAM_ERROR || size == 0)
	return FALSE;
	source->pub.next_input_byte = source->buffer;
	source->pub.bytes_in_buffer = size;
	return TRUE;
}

METHODDEF(void)
skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
	JpegSource* source = (JpegSource*)cinfo->src;

	if (num_bytes > (long)source->pub.bytes_in_buffer)
	{
		// We need to skip more data than we have in the buffer.
		source->pStream->setPos(source->pStream->getPos() + (num_bytes - source->pub.bytes_in_buffer));
		source->pub.next_input_byte += source->pub.bytes_in_buffer;
		source->pub.bytes_in_buffer = 0;
	}
	else
	{
		// Skip portion of the buffer
		source->pub.bytes_in_buffer -= num_bytes;
		source->pub.next_input_byte += num_bytes;
	}
}

METHODDEF(void)
error_exit(j_common_ptr cinfo)
{
	JpegErrorMgr* err_mgr = (JpegErrorMgr*)(cinfo->err);

	/* Return control to the setjmp point */
	longjmp( err_mgr->setjmp_buffer, 1 );
}


// S T R U C T S ///////////////////////////////////////////////////

CImageJPG::CImageJPG() : m_state(NULL)
{
} // Constructor

CImageJPG::~CImageJPG()
{
	//clean up
	Close();
} // Destructor
/*----------------------------------------------------------------*/

void CImageJPG::Close()
{
	if (m_state)
	{
		JpegState* state = (JpegState*)m_state;
		jpeg_destroy_decompress( &state->cinfo );
		delete state;
		m_state = NULL;
	}
	m_width = m_height = 0;
	CImage::Close();
}
/*----------------------------------------------------------------*/

HRESULT CImageJPG::ReadHeader()
{
	JpegState* state = new JpegState;
	m_state = state;
	state->cinfo.err = jpeg_std_error(&state->jerr.pub);
	state->jerr.pub.error_exit = error_exit;

	if (setjmp(state->jerr.setjmp_buffer ) == 0)
	{
		jpeg_create_decompress( &state->cinfo );

		// Prepare for suspending reader
		state->source.pub.init_source = stub;
		state->source.pub.fill_input_buffer = fill_input_buffer;
		state->source.pub.skip_input_data = skip_input_data;
		state->source.pub.resync_to_restart = jpeg_resync_to_restart;
		state->source.pub.term_source = stub;
		state->source.pub.bytes_in_buffer = 0;// forces fill_input_buffer on first read
		state->source.pub.next_input_byte = NULL;
		state->source.pStream = (IOSTREAM*)m_pStream;
		state->source.buffer = (JOCTET*)(*state->cinfo.mem->alloc_small)((j_common_ptr)&state->cinfo, JPOOL_PERMANENT, JPG_BUFFER_SIZE * sizeof(JOCTET));
		state->cinfo.src = &state->source.pub;

		jpeg_read_header(&state->cinfo, TRUE);

		m_dataWidth = m_width = state->cinfo.image_width;
		m_dataHeight= m_height = state->cinfo.image_height;
		m_numLevels = 0;
		m_level     = 0;
		m_stride    = state->cinfo.num_components;
		m_lineWidth = m_width * m_stride;
		switch (m_stride)
		{
		case 1:
			m_format = PF_GRAY8;
			state->cinfo.out_color_space = JCS_GRAYSCALE;
			state->cinfo.out_color_components = 1;
			break;
		case 3:
			m_format = PF_B8G8R8;
			state->cinfo.out_color_space = JCS_RGB;
			state->cinfo.out_color_components = 3;
			break;
		default:
			LOG(LT_IMAGE, "error: unsupported JPG image");
			return _INVALIDFILE;
		}
		return _OK;
	}

	Close();
	return _FAIL;
} // ReadHeader
/*----------------------------------------------------------------*/

HRESULT CImageJPG::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	JpegState* state = (JpegState*)m_state;

	if (state && m_width && m_height)
	{
		jpeg_decompress_struct* cinfo = &state->cinfo;
		JpegErrorMgr* jerr = &state->jerr;

		if (setjmp(jerr->setjmp_buffer) == 0)
		{
			jpeg_start_decompress(cinfo);

			// read data
			if (dataFormat == m_format && nStride == m_stride) {
				// read image directly to the data buffer
				JSAMPLE* buffer[1] = {(JSAMPLE*)pData};
				uint8_t*& data = (uint8_t*&)buffer[0];
				for (Size j=0; j<m_height; ++j, data+=lineWidth)
					jpeg_read_scanlines(cinfo, buffer, 1);
			} else {
				// read image to a buffer and convert it
				JSAMPARRAY buffer = (*cinfo->mem->alloc_sarray)((j_common_ptr)cinfo, JPOOL_IMAGE, m_lineWidth, 1);
				uint8_t* dst = (uint8_t*)pData;
				uint8_t* src = (uint8_t*)buffer[0];
				for (Size j=0; j<m_height; ++j, dst+=lineWidth) {
					jpeg_read_scanlines(cinfo, buffer, 1);
					if (!FilterFormat(dst, dataFormat, nStride, src, m_format, m_stride, m_width))
						return _FAIL;
				}
			}

			jpeg_finish_decompress(cinfo);
			return _OK;
		}
	}

	Close();
	return _FAIL;
} // Read
/*----------------------------------------------------------------*/

HRESULT CImageJPG::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	//TODO: to implement the JPG encoder
	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/

HRESULT CImageJPG::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	//TODO: to implement the JPG encoder
	//const int quality = 100;
	//struct jpeg_compress_struct cinfo;
	//struct jpeg_error_mgr jerr;
	///* More stuff */
	//FILE * outfile;		/* target file */
	//JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */
	//int row_stride;		/* physical row width in image buffer */

	//cinfo.err = jpeg_std_error(&jerr);
	//jpeg_create_compress(&cinfo);

	//if ((outfile = fopen(filename.c_str(), "wb")) == NULL) {
	//	fprintf(stderr, "can't open %s\n", filename.c_str());
	//	exit(1);
	//}
	//jpeg_stdio_dest(&cinfo, outfile);

	//cinfo.image_width = width;
	//cinfo.image_height = height;
	//cinfo.input_components = 3;
	//cinfo.in_color_space = JCS_RGB;
	//jpeg_set_defaults(&cinfo);
	//jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

	//jpeg_start_compress(&cinfo, TRUE);

	//row_stride = width * 3;	/* JSAMPLEs per row in image_buffer */

	//while (cinfo.next_scanline < cinfo.image_height) {
	//	if (flip)
	//		row_pointer[0] = (JSAMPROW)& buffer[(cinfo.image_height - 1 - cinfo.next_scanline) * row_stride];
	//	else
	//		row_pointer[0] = (JSAMPROW)& buffer[cinfo.next_scanline * row_stride];
	//	(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);

	//}

	//jpeg_finish_compress(&cinfo);
	//fclose(outfile);

	//jpeg_destroy_compress(&cinfo);
	return _OK;
} // WriteData
/*----------------------------------------------------------------*/

#endif // _IMAGE_JPG
