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

// https://www.awaresystems.be/imaging/tiff/tifftags/sampleformat.html
enum TIFF_SAMPLEFORMAT_TYPE {
    TIFF_SAMPLEFORMAT_UINT = 1,
    TIFF_SAMPLEFORMAT_INT = 2,
    TIFF_SAMPLEFORMAT_IEEEFP = 3
};

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
		uint16 bpp=8, ncn = photometric > 1 ? 3 : 1, sampleFormat = 1;
		TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bpp);
		TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &ncn);
		TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &sampleFormat);

		m_dataWidth = m_width;
		m_dataHeight= m_height;
		m_numLevels = 0;
		m_level     = 0;
		m_stride    = ncn;

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

		bool implemented = true;

		switch (bpp){
		case 8:
			if (ncn >= 3){
				m_format = PF_B8G8R8A8;
				m_stride = 4;
			}else if (ncn == 1){
				m_format = PF_GRAY8;
				m_stride = 1;
			}else{
				implemented = false;
			}
			break;
		case 16:
			m_format = PF_GRAYU16;
			m_stride = 2;
			if (ncn != 1 || sampleFormat != TIFF_SAMPLEFORMAT_UINT) implemented = false;
			break;
		case 32:
			m_format = PF_GRAYF32;
			m_stride = 4;
			if (ncn != 1 || sampleFormat != TIFF_SAMPLEFORMAT_IEEEFP) implemented = false;
			break;
		default:
			// TODO: implement support for more
			implemented = false;
		}

		if (!implemented){
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
		uint16 photometric;
		TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &photometric);
		uint16 bpp = 8, ncn = photometric > 1 ? 3 : 1;
		TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bpp);
		TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &ncn);

		const size_t buffer_size = m_stride * m_width * m_height;
		CLISTDEF0(uint8_t) _buffer(buffer_size);
		uint8_t* buffer = _buffer.Begin();

		if (m_format == PF_B8G8R8A8 || m_format == PF_GRAY8){
			char errmsg[1024];
			if (!TIFFRGBAImageOK(tif, errmsg)) {
				Close();
				return _INVALIDFILE;
			}

			// Simplified
			if (!TIFFReadRGBAImageOriented(tif, m_width, m_height, (uint32*)buffer, ORIENTATION_TOPLEFT, 0)){
				Close();
				return _INVALIDFILE;
			}
		}else if (m_format == PF_GRAYU16 || m_format == PF_GRAYF32){
			for (uint32 y = 0; y < m_height; y++, buffer += m_lineWidth){
				if (!TIFFReadScanline(tif, buffer, y, 0)){
					Close();
					return _INVALIDFILE;
				}
			}

			buffer = _buffer.Begin();
		}

		// Data not in the format we need?
		if (dataFormat != m_format || nStride != m_stride){
			if (!FilterFormat(pData, dataFormat, nStride, buffer, m_format, m_stride, m_width * m_height)) {
				Close();
				return _FAIL;
			}
		}

		return _OK;
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
