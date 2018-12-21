////////////////////////////////////////////////////////////////////
// ImageTIFF.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_TIFF
#include "ImageTIFF.h"

extern "C" {
#if !defined(_MSC_VER) && !defined(__BORLANDC__)
#include <tiffconf.h>
#undef TIFF_INT64_T
#define TIFF_INT64_T int64_t
#undef TIFF_UINT64_T
#define TIFF_UINT64_T uint64_t
#endif
#include <tiffio.h>
}

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

/*
  ISO C++ uses a 'std::streamsize' type to define counts.  This makes
  it similar to, (but perhaps not the same as) size_t.

  The std::ios::pos_type is used to represent stream positions as used
  by tellg(), tellp(), seekg(), and seekp().  This makes it similar to
  (but perhaps not the same as) 'off_t'.  The std::ios::streampos type
  is used for character streams, but is documented to not be an
  integral type anymore, so it should *not* be assigned to an integral
  type.

  The std::ios::off_type is used to specify relative offsets needed by
  the variants of seekg() and seekp() which accept a relative offset
  argument.

  Useful prototype knowledge:

  Obtain read position
    ios::pos_type basic_istream::tellg()

  Set read position
    basic_istream& basic_istream::seekg(ios::pos_type)
    basic_istream& basic_istream::seekg(ios::off_type, ios_base::seekdir)

  Read data
    basic_istream& istream::read(char *str, streamsize count)

  Number of characters read in last unformatted read
    streamsize istream::gcount();

  Obtain write position
    ios::pos_type basic_ostream::tellp()

  Set write position
    basic_ostream& basic_ostream::seekp(ios::pos_type)
    basic_ostream& basic_ostream::seekp(ios::off_type, ios_base::seekdir)

  Write data
    basic_ostream& ostream::write(const char *str, streamsize count)
*/

struct tiffis_data;
struct tiffos_data;

extern "C" {

	static tmsize_t _tiffosReadProc(thandle_t, void*, tmsize_t);
	static tmsize_t _tiffisReadProc(thandle_t fd, void* buf, tmsize_t size);
	static tmsize_t _tiffosWriteProc(thandle_t fd, void* buf, tmsize_t size);
	static tmsize_t _tiffisWriteProc(thandle_t, void*, tmsize_t);
	static uint64   _tiffosSeekProc(thandle_t fd, uint64 off, int whence);
	static uint64   _tiffisSeekProc(thandle_t fd, uint64 off, int whence);
	static uint64   _tiffosSizeProc(thandle_t fd);
	static uint64   _tiffisSizeProc(thandle_t fd);
	static int      _tiffosCloseProc(thandle_t fd);
	static int      _tiffisCloseProc(thandle_t fd);
	static int 	    _tiffDummyMapProc(thandle_t, void** base, toff_t* size);
	static void     _tiffDummyUnmapProc(thandle_t, void* base, toff_t size);
	static TIFF*    _tiffStreamOpen(const char* name, const char* mode, void *fd);

	struct tiffis_data
	{
		ISTREAM* stream;
		size_f_t start_pos;
	};

	struct tiffos_data
	{
		OSTREAM* stream;
		size_f_t start_pos;
	};

	static tmsize_t _tiffosReadProc(thandle_t, void*, tmsize_t)
	{
		return 0;
	}

	static tmsize_t _tiffisReadProc(thandle_t fd, void* buf, tmsize_t size)
	{
		tiffis_data	*data = reinterpret_cast<tiffis_data *>(fd);

		// Verify that type does not overflow.
		size_t request_size = size;
		if (static_cast<tmsize_t>(request_size) != size)
			return static_cast<tmsize_t>(-1);

		return static_cast<tmsize_t>(data->stream->read(buf, request_size));
	}

	static tmsize_t _tiffosWriteProc(thandle_t fd, void* buf, tmsize_t size)
	{
		tiffos_data	*data = reinterpret_cast<tiffos_data *>(fd);

		// Verify that type does not overflow.
		size_t request_size = size;
		if (static_cast<tmsize_t>(request_size) != size)
			return static_cast<tmsize_t>(-1);

		return static_cast<tmsize_t>(data->stream->write(buf, request_size));
	}

	static tmsize_t _tiffisWriteProc(thandle_t, void*, tmsize_t)
	{
		return 0;
	}

	static uint64 _tiffosSeekProc(thandle_t fd, uint64 off, int whence)
	{
		tiffos_data	*data = reinterpret_cast<tiffos_data *>(fd);
		OSTREAM* os = data->stream;

		// if the stream has already failed, don't do anything
		if (os == NULL)
			return static_cast<uint64>(-1);

		bool bSucceeded(true);
		switch (whence) {
		case SEEK_SET:
		{
			// Compute 64-bit offset
			uint64 new_offset = static_cast<uint64>(data->start_pos) + off;

			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(new_offset);
			if (static_cast<uint64>(offset) != new_offset)
				return static_cast<uint64>(-1);

			bSucceeded = os->setPos(offset);
			break;
		}
		case SEEK_CUR:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64>(offset) != off)
				return static_cast<uint64>(-1);

			bSucceeded = os->setPos(os->getPos()+offset);
			break;
		}
		case SEEK_END:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64>(offset) != off)
				return static_cast<uint64>(-1);

			bSucceeded = os->setPos(os->getSize()-offset);
			break;
		}
		}

		// Attempt to workaround problems with seeking past the end of the
		// stream.  ofstream doesn't have a problem with this but
		// ostrstream/ostringstream does. In that situation, add intermediate
		// '\0' characters.
		if (!bSucceeded) {
			size_f_t origin;
			switch (whence) {
			case SEEK_SET:
			default:
				origin = data->start_pos;
				break;
			case SEEK_CUR:
				origin = os->getPos();
				break;
			case SEEK_END:
				os->setPos(os->getSize());
				origin = os->getPos();
				break;
			}

			// only do something if desired seek position is valid
			if ((static_cast<uint64>(origin) + off) > static_cast<uint64>(data->start_pos)) {
				uint64	num_fill;
				// extend the stream to the expected size
				os->setPos(os->getSize());
				num_fill = (static_cast<uint64>(origin)) + off - os->getPos();
				const char dummy = '\0';
				for (uint64 i = 0; i < num_fill; i++)
					os->write(&dummy, 1);
				// retry the seek
				os->setPos(static_cast<size_f_t>(static_cast<uint64>(origin) + off));
			}
		}

		return static_cast<uint64>(os->getPos());
	}

	static uint64 _tiffisSeekProc(thandle_t fd, uint64 off, int whence)
	{
		tiffis_data	*data = reinterpret_cast<tiffis_data *>(fd);
		ISTREAM* is = data->stream;

		switch (whence) {
		case SEEK_SET:
		{
			// Compute 64-bit offset
			uint64 new_offset = static_cast<uint64>(data->start_pos) + off;

			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(new_offset);
			if (static_cast<uint64>(offset) != new_offset)
				return static_cast<uint64>(-1);

			is->setPos(offset);
			break;
		}
		case SEEK_CUR:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64>(offset) != off)
				return static_cast<uint64>(-1);

			is->setPos(is->getPos()+offset);
			break;
		}
		case SEEK_END:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64>(offset) != off)
				return static_cast<uint64>(-1);

			is->setPos(is->getSize()-offset);
			break;
		}
		}

		return (uint64)(is->getPos() - data->start_pos);
	}

	static uint64 _tiffosSizeProc(thandle_t fd)
	{
		tiffos_data	*data = reinterpret_cast<tiffos_data *>(fd);
		return (uint64)data->stream->getSize();
	}

	static uint64 _tiffisSizeProc(thandle_t fd)
	{
		tiffis_data	*data = reinterpret_cast<tiffis_data *>(fd);
		return (uint64)data->stream->getSize();
	}

	static int _tiffosCloseProc(thandle_t fd)
	{
		// Our stream was not allocated by us, so it shouldn't be closed by us.
		delete reinterpret_cast<tiffos_data *>(fd);
		return 0;
	}

	static int _tiffisCloseProc(thandle_t fd)
	{
		// Our stream was not allocated by us, so it shouldn't be closed by us.
		delete reinterpret_cast<tiffis_data *>(fd);
		return 0;
	}

	static int _tiffDummyMapProc(thandle_t, void** /*base*/, toff_t* /*size*/)
	{
		return (0);
	}

	static void _tiffDummyUnmapProc(thandle_t, void* /*base*/, toff_t /*size*/)
	{
	}

	/*
	 * Open a TIFF file descriptor for read/writing.
	 */
	static TIFF* _tiffStreamOpen(const char* name, const char* mode, void *fd)
	{
		TIFF*	tif;

		if (strchr(mode, 'w')) {
			tiffos_data	*data = new tiffos_data;
			data->stream = reinterpret_cast<OSTREAM*>(fd);
			data->start_pos = data->stream->getPos();

			// Open for writing.
			tif = TIFFClientOpen(name, mode,
								 reinterpret_cast<thandle_t>(data),
								 _tiffosReadProc,
								 _tiffosWriteProc,
								 _tiffosSeekProc,
								 _tiffosCloseProc,
								 _tiffosSizeProc,
								 _tiffDummyMapProc,
								 _tiffDummyUnmapProc);
		} else {
			tiffis_data	*data = new tiffis_data;
			data->stream = reinterpret_cast<ISTREAM*>(fd);
			data->start_pos = data->stream->getPos();
			// Open for reading.
			tif = TIFFClientOpen(name, mode,
								 reinterpret_cast<thandle_t>(data),
								 _tiffisReadProc,
								 _tiffisWriteProc,
								 _tiffisSeekProc,
								 _tiffisCloseProc,
								 _tiffisSizeProc,
								 _tiffDummyMapProc,
								 _tiffDummyUnmapProc);
		}

		return (tif);
	}

} /* extern "C" */

// TIFFOpen() mode flags are different to fopen().  A 'b' in mode "rb" has no effect when reading.
// http://www.remotesensing.org/libtiff/man/TIFFOpen.3tiff.html
// NB: We don't support mapped files with streams so add 'm'
TIFF* TIFFStreamOpen(const char* name, OSTREAM* os)
{
	return _tiffStreamOpen(name, "wm", os);
}
TIFF* TIFFStreamOpen(const char* name, ISTREAM* is)
{
	return _tiffStreamOpen(name, "rm", is);
}


// S T R U C T S ///////////////////////////////////////////////////

CImageTIFF::CImageTIFF() : m_state(NULL)
{
} // Constructor

CImageTIFF::~CImageTIFF()
{
	//clean up
	Close();
} // Destructor
/*----------------------------------------------------------------*/

void CImageTIFF::Close()
{
	if (m_state)
	{
		TIFF* tif = static_cast<TIFF*>(m_state);
		TIFFClose(tif);
		m_state = NULL;
	}
	m_width = m_height = 0;
	CImage::Close();
}
/*----------------------------------------------------------------*/

HRESULT CImageTIFF::ReadHeader()
{
	TIFF* tif = static_cast<TIFF*>(m_state);
	if (!tif) {
		tif = TIFFStreamOpen("ReadTIFF", (ISTREAM*)m_pStream);
		if (!tif) {
			LOG(LT_IMAGE, "error: unsupported TIFF image");
			return _INVALIDFILE;
		}
	}
	m_state = tif;

	uint16 photometric = 0;
	if (TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &m_width) &&
		TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &m_height) &&
		TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &photometric))
	{
		uint16 bpp=8, ncn = photometric > 1 ? 3 : 1;
		TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bpp);
		TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &ncn);

		m_dataWidth = m_width;
		m_dataHeight= m_height;
		m_numLevels = 0;
		m_level     = 0;

		if ((bpp == 32 && ncn == 3) || photometric == PHOTOMETRIC_LOGLUV) {
			// this is HDR format with 3 floats per pixel
			//TODO: implement
			ASSERT("error: not implemented" == NULL);
			Close();
			return _FAIL;
		}
		if (bpp > 8 &&
			((photometric != 2 && photometric != 1) ||
				(ncn != 1 && ncn != 3 && ncn != 4)))
			bpp = 8;
		switch (bpp) {
		case 8:
			m_stride = 4;
			m_format = PF_B8G8R8A8;
			break;
		//case 16:
		//	m_type = CV_MAKETYPE(CV_16U, photometric > 1 ? 3 : 1);
		//	break;
		//case 32:
		//	m_type = CV_MAKETYPE(CV_32F, photometric > 1 ? 3 : 1);
		//	break;
		//case 64:
		//	m_type = CV_MAKETYPE(CV_64F, photometric > 1 ? 3 : 1);
		//	break;
		default:
			//TODO: implement
			ASSERT("error: not implemented" == NULL);
			LOG(LT_IMAGE, "error: unsupported TIFF image");
			Close();
			return _INVALIDFILE;
		}
		m_lineWidth = m_width * m_stride;

		return _OK;
	}

	Close();
	return _FAIL;
} // ReadHeader
/*----------------------------------------------------------------*/

HRESULT CImageTIFF::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	if (m_state && m_width && m_height) {
		TIFF* tif = (TIFF*)m_state;
		uint32 tile_width0 = m_width, tile_height0 = 0;
		int is_tiled = TIFFIsTiled(tif);
		uint16 photometric;
		TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &photometric);
		uint16 bpp = 8, ncn = photometric > 1 ? 3 : 1;
		TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bpp);
		TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &ncn);
		const int bitsPerByte = 8;
		int dst_bpp = (int)(1 * bitsPerByte);
		if (dst_bpp == 8) {
			char errmsg[1024];
			if (!TIFFRGBAImageOK(tif, errmsg)) {
				Close();
				return _INVALIDFILE;
			}
		}

		if ((!is_tiled) ||
			(is_tiled &&
			 TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tile_width0) &&
			 TIFFGetField(tif, TIFFTAG_TILELENGTH, &tile_height0)))
		{
			if (!is_tiled)
				TIFFGetField(tif, TIFFTAG_ROWSPERSTRIP, &tile_height0);

			if (tile_width0 <= 0)
				tile_width0 = m_width;

			if (tile_height0 <= 0 ||
				(!is_tiled && tile_height0 == std::numeric_limits<uint32>::max()))
				tile_height0 = m_height;

			uint8_t* data = (uint8_t*)pData;
			if (!is_tiled && tile_height0 == 1 && dataFormat == m_format && nStride == m_stride) {
				// read image directly to the data buffer
				for (Size j=0; j<m_height; ++j, data+=lineWidth)
					if (!TIFFReadRGBAStrip(tif, j, (uint32*)data)) {
						Close();
						return _INVALIDFILE;
					}
			} else {
				// read image to a buffer and convert it
				const size_t buffer_size = 4 * tile_height0 * tile_width0;
				CLISTDEF0(uint8_t) _buffer(buffer_size);
				uint8_t* buffer = _buffer.Begin();

				for (uint32 y = 0; y < m_height; y += tile_height0, data += lineWidth*tile_height0) {
					uint32 tile_height = tile_height0;
					if (y + tile_height > m_height)
						tile_height = m_height - y;

					for (uint32 x = 0; x < m_width; x += tile_width0) {
						uint32 tile_width = tile_width0;
						if (x + tile_width > m_width)
							tile_width = m_width - x;

						int ok;
						switch (dst_bpp) {
						case 8:
						{
							uint8_t* bstart = buffer;
							if (!is_tiled)
								ok = TIFFReadRGBAStrip(tif, y, (uint32*)buffer);
							else {
								ok = TIFFReadRGBATile(tif, x, y, (uint32*)buffer);
								//Tiles fill the buffer from the bottom up
								bstart += (tile_height0 - tile_height) * tile_width0 * 4;
							}
							if (!ok) {
								Close();
								return _INVALIDFILE;
							}

							for (uint32 i = 0; i < tile_height; ++i) {
								uint8_t* dst = data + x*3 + lineWidth*(tile_height - i - 1);
								uint8_t* src = bstart + i*tile_width0*4;
								if (!FilterFormat(dst, dataFormat, nStride, src, m_format, m_stride, tile_width)) {
									Close();
									return _FAIL;
								}
							}
							break;
						}
						default:
						{
							Close();
							return _INVALIDFILE;
						}
						}
					}
				}
			}

			return _OK;
		}
	}

	Close();
	return _FAIL;
} // Read
/*----------------------------------------------------------------*/

HRESULT CImageTIFF::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	//TODO: to implement the TIFF encoder
	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/

HRESULT CImageTIFF::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	//TODO: to implement the TIFF encoder
	return _OK;
} // WriteData
/*----------------------------------------------------------------*/

#endif // _IMAGE_TIFF
