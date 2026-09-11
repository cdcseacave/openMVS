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
	static uint64_t _tiffosSeekProc(thandle_t fd, uint64_t off, int whence);
	static uint64_t _tiffisSeekProc(thandle_t fd, uint64_t off, int whence);
	static uint64_t _tiffosSizeProc(thandle_t fd);
	static uint64_t _tiffisSizeProc(thandle_t fd);
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

	static uint64_t _tiffosSeekProc(thandle_t fd, uint64_t off, int whence)
	{
		tiffos_data	*data = reinterpret_cast<tiffos_data *>(fd);
		OSTREAM* os = data->stream;

		// if the stream has already failed, don't do anything
		if (os == NULL)
			return static_cast<uint64_t>(-1);

		bool bSucceeded(true);
		switch (whence) {
		case SEEK_SET:
		{
			// Compute 64-bit offset
			uint64_t new_offset = static_cast<uint64_t>(data->start_pos) + off;

			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(new_offset);
			if (static_cast<uint64_t>(offset) != new_offset)
				return static_cast<uint64_t>(-1);

			bSucceeded = os->setPos(offset);
			break;
		}
		case SEEK_CUR:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64_t>(offset) != off)
				return static_cast<uint64_t>(-1);

			bSucceeded = os->setPos(os->getPos()+offset);
			break;
		}
		case SEEK_END:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64_t>(offset) != off)
				return static_cast<uint64_t>(-1);

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
			if ((static_cast<uint64_t>(origin) + off) > static_cast<uint64_t>(data->start_pos)) {
				uint64_t	num_fill;
				// extend the stream to the expected size
				os->setPos(os->getSize());
				num_fill = (static_cast<uint64_t>(origin)) + off - os->getPos();
				const char dummy = '\0';
				for (uint64_t i = 0; i < num_fill; i++)
					os->write(&dummy, 1);
				// retry the seek
				os->setPos(static_cast<size_f_t>(static_cast<uint64_t>(origin) + off));
			}
		}

		return static_cast<uint64_t>(os->getPos() - data->start_pos);
	}

	static uint64_t _tiffisSeekProc(thandle_t fd, uint64_t off, int whence)
	{
		tiffis_data	*data = reinterpret_cast<tiffis_data *>(fd);
		ISTREAM* is = data->stream;

		switch (whence) {
		case SEEK_SET:
		{
			// Compute 64-bit offset
			uint64_t new_offset = static_cast<uint64_t>(data->start_pos) + off;

			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(new_offset);
			if (static_cast<uint64_t>(offset) != new_offset)
				return static_cast<uint64_t>(-1);

			is->setPos(offset);
			break;
		}
		case SEEK_CUR:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64_t>(offset) != off)
				return static_cast<uint64_t>(-1);

			is->setPos(is->getPos()+offset);
			break;
		}
		case SEEK_END:
		{
			// Verify that value does not overflow
			size_f_t offset = static_cast<size_f_t>(off);
			if (static_cast<uint64_t>(offset) != off)
				return static_cast<uint64_t>(-1);

			is->setPos(is->getSize()-offset);
			break;
		}
		}

		return (uint64_t)(is->getPos() - data->start_pos);
	}

	static uint64_t _tiffosSizeProc(thandle_t fd)
	{
		tiffos_data	*data = reinterpret_cast<tiffos_data *>(fd);
		return (uint64_t)data->stream->getSize();
	}

	static uint64_t _tiffisSizeProc(thandle_t fd)
	{
		tiffis_data	*data = reinterpret_cast<tiffis_data *>(fd);
		return (uint64_t)data->stream->getSize();
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
			if (!tif)
				delete data;
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
			if (!tif)
				delete data;
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

bool CImageTIFF::ReadHeader()
{
	TIFF* tif = static_cast<TIFF*>(m_state);
	if (!tif) {
		tif = TIFFStreamOpen("ReadTIFF", (ISTREAM*)m_pStream);
		if (!tif) {
			LOG(LT_IMAGE, "error: unsupported TIFF image");
			return false;
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
			return false;
		}
		if (bpp > 8 &&
			((photometric != 2 && photometric != 1) ||
				(ncn != 1 && ncn != 3 && ncn != 4)))
			bpp = 8;
		switch (bpp) {
		case 8:
			if (photometric == PHOTOMETRIC_PALETTE) {
				m_stride = 3;
				m_format = PF_B8G8R8;
			} else {
				switch (ncn) {
				case 1:
					m_stride = 1;
					m_format = PF_GRAY8;
					break;
				case 3:
					m_stride = 3;
					m_format = PF_B8G8R8;
					break;
				default:
					m_stride = 4;
					m_format = PF_B8G8R8A8;
					break;
				}
			}
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
			return false;
		}
		m_lineWidth = m_width * m_stride;

		return true;
	}

	Close();
	return false;
} // ReadHeader
/*----------------------------------------------------------------*/

bool CImageTIFF::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	if (m_state && m_width && m_height) {
		TIFF* tif = (TIFF*)m_state;
		uint32_t tile_width0 = m_width, tile_height0 = 0;
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
				return false;
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
				(!is_tiled && tile_height0 == std::numeric_limits<uint32_t>::max()))
				tile_height0 = m_height;

			uint8_t* data = (uint8_t*)pData;
			if (!is_tiled && tile_height0 == 1 && dataFormat == PF_B8G8R8A8 && nStride == 4) {
				// read image directly to the data buffer
				for (Size j=0; j<m_height; ++j, data+=lineWidth)
					if (!TIFFReadRGBAStrip(tif, j, (uint32_t*)data)) {
						Close();
						return false;
					}
			} else {
				// read image to a buffer and convert it
				const size_t buffer_size = 4 * tile_height0 * tile_width0;
				CLISTDEF0(uint8_t) _buffer(buffer_size);
				uint8_t* buffer = _buffer.Begin();

				for (uint32_t y = 0; y < m_height; y += tile_height0, data += lineWidth*tile_height0) {
					uint32_t tile_height = tile_height0;
					if (y + tile_height > m_height)
						tile_height = m_height - y;

					for (uint32_t x = 0; x < m_width; x += tile_width0) {
						uint32_t tile_width = tile_width0;
						if (x + tile_width > m_width)
							tile_width = m_width - x;

						int ok;
						switch (dst_bpp) {
						case 8:
						{
							uint8_t* bstart = buffer;
							if (!is_tiled)
								ok = TIFFReadRGBAStrip(tif, y, (uint32_t*)buffer);
							else {
								ok = TIFFReadRGBATile(tif, x, y, (uint32_t*)buffer);
								//Tiles fill the buffer from the bottom up
								bstart += (tile_height0 - tile_height) * tile_width0 * 4;
							}
							if (!ok) {
								Close();
								return false;
							}

							for (uint32_t i = 0; i < tile_height; ++i) {
								uint8_t* dst = data + x*nStride + lineWidth*(tile_height - i - 1);
								uint8_t* src = bstart + i*tile_width0*4;
								if (!FilterFormat(dst, dataFormat, nStride, src, PF_B8G8R8A8, 4, tile_width)) {
									Close();
									return false;
								}
							}
							break;
						}
						default:
						{
							Close();
							return false;
						}
						}
					}
				}
			}

			return true;
		}
	}

	Close();
	return false;
} // Read
/*----------------------------------------------------------------*/

bool CImageTIFF::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	ASSERT(m_pStream != NULL);
	ASSERT(width > 0 && height > 0);

	TIFF* tif = static_cast<TIFF*>(m_state);
	if (!tif) {
		tif = TIFFStreamOpen("WriteTIFF", (OSTREAM*)m_pStream);
		if (!tif) {
			LOG(LT_IMAGE, "error: unsupported TIFF image");
			return false;
		}
	}
	m_state = tif;

	uint16 samplesPerPixel = 0;
	switch (imageFormat) {
	case PF_A8:
	case PF_GRAY8:
		m_stride = 1;
		m_format = PF_GRAY8;
		samplesPerPixel = 1;
		break;
	case PF_B8G8R8:
	case PF_R8G8B8:
		m_stride = 3;
		m_format = PF_B8G8R8;
		samplesPerPixel = 3;
		break;
	case PF_R8G8B8A8:
	case PF_A8R8G8B8:
	case PF_B8G8R8A8:
	case PF_A8B8G8R8:
		m_stride = 4;
		m_format = PF_B8G8R8A8;
		samplesPerPixel = 4;
		break;
	default:
		LOG(LT_IMAGE, "error: unsupported TIFF image format");
		Close();
		return false;
	}

	m_dataWidth = m_width = width;
	m_dataHeight = m_height = height;
	m_numLevels = numLevels;
	m_level = 0;
	m_lineWidth = m_width * m_stride;

	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, (uint32)m_width);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, (uint32)m_height);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, samplesPerPixel);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 8);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, samplesPerPixel > 1 ? PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK);
	TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(tif, 0));
	if (samplesPerPixel == 4) {
		const uint16 extraSamples[1] = { EXTRASAMPLE_ASSOCALPHA };
		TIFFSetField(tif, TIFFTAG_EXTRASAMPLES, 1, extraSamples);
	}

	return true;
} // WriteHeader
/*----------------------------------------------------------------*/

bool CImageTIFF::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	ASSERT(pData != NULL);
	ASSERT(m_width > 0 && m_height > 0);
	ASSERT(nStride > 0 && lineWidth >= m_width * nStride);

	TIFF* tif = static_cast<TIFF*>(m_state);
	if (!tif)
		return false;

	const uint8_t* pRow = static_cast<const uint8_t*>(pData);
	if (dataFormat == m_format && nStride == m_stride) {
		for (Size j = 0; j < m_height; ++j, pRow += lineWidth) {
			if (TIFFWriteScanline(tif, const_cast<uint8_t*>(pRow), (uint32)j) < 0) {
				Close();
				return false;
			}
		}
	} else {
		CAutoPtrArr<uint8_t> const buffer(new uint8_t[m_lineWidth]);
		for (Size j = 0; j < m_height; ++j, pRow += lineWidth) {
			if (!FilterFormat(buffer, m_format, m_stride, pRow, dataFormat, nStride, m_width)) {
				Close();
				return false;
			}
			if (TIFFWriteScanline(tif, buffer, (uint32)j) < 0) {
				Close();
				return false;
			}
		}
	}

	TIFFFlush(tif);
	return true;
} // WriteData
/*----------------------------------------------------------------*/

#ifdef _USE_TESTS

bool CImageTIFF::Test(const String& folder)
{
	String dir(folder);
	Util::ensureValidFolderPath(dir);

	// 1) Write and read-back an RGB TIFF with distinct channel values (catching red/blue swaps)
	{
		const String fileName(dir + "test_rgb.tif");
		const Size width = 32, height = 24;
		const Size stride = 3;
		std::vector<uint8_t> writeBuffer(width * height * stride);
		for (Size y = 0; y < height; ++y) {
			for (Size x = 0; x < width; ++x) {
				const size_t idx = (y * width + x) * stride;
				// In OpenCV BGR format (PF_R8G8B8): byte 0 = B, byte 1 = G, byte 2 = R
				writeBuffer[idx + 0] = uint8_t((200 + x) % 256); // B
				writeBuffer[idx + 1] = uint8_t((50 + y) % 256);  // G
				writeBuffer[idx + 2] = uint8_t((10 + x * y) % 256); // R
			}
		}
		// Write using PF_R8G8B8 (OpenCV BGR layout)
		{
			CAutoPtr<CImage> pImage(CImage::Create(fileName, CImage::WRITE));
			if (pImage == NULL ||
				!pImage->WriteHeader(PF_R8G8B8, width, height, 1) ||
				!pImage->WriteData(writeBuffer.data(), PF_R8G8B8, stride, width * stride))
			{
				VERBOSE("error: CImageTIFF::Test: failed writing RGB TIFF '%s'", fileName.c_str());
				return false;
			}
		}
		// Read back header and check metadata
		{
			CAutoPtr<CImage> pImage(CImage::Create(fileName, CImage::READ));
			if (pImage == NULL || !pImage->ReadHeader()) {
				VERBOSE("error: CImageTIFF::Test: failed reading header of '%s'", fileName.c_str());
				return false;
			}
			if (pImage->GetWidth() != width || pImage->GetHeight() != height ||
				pImage->GetStride() != stride || pImage->GetFormat() != PF_B8G8R8)
			{
				VERBOSE("error: CImageTIFF::Test: header mismatch for '%s' (got %ux%u stride %u format %u, expected %ux%u stride %u format %u)",
					fileName.c_str(), pImage->GetWidth(), pImage->GetHeight(), pImage->GetStride(), (unsigned)pImage->GetFormat(),
					width, height, stride, (unsigned)PF_B8G8R8);
				return false;
			}
			// Read back data into PF_R8G8B8 (OpenCV BGR layout)
			std::vector<uint8_t> readBuffer(width * height * stride);
			if (!pImage->ReadData(readBuffer.data(), PF_R8G8B8, stride, width * stride)) {
				VERBOSE("error: CImageTIFF::Test: failed reading data of '%s'", fileName.c_str());
				return false;
			}
			for (size_t i = 0; i < writeBuffer.size(); ++i) {
				if (readBuffer[i] != writeBuffer[i]) {
					VERBOSE("error: CImageTIFF::Test: pixel mismatch at byte %zu (read %u != written %u); possible channel swap!",
						i, (unsigned)readBuffer[i], (unsigned)writeBuffer[i]);
					return false;
				}
			}
		}
		// Cross-verify with OpenCV cv::imread
		{
			cv::Mat cvImg = cv::imread(fileName.c_str(), cv::IMREAD_COLOR);
			if (cvImg.empty() || cvImg.cols != (int)width || cvImg.rows != (int)height) {
				VERBOSE("error: CImageTIFF::Test: OpenCV failed reading TIFF '%s'", fileName.c_str());
				return false;
			}
			for (int y = 0; y < (int)height; ++y) {
				const uint8_t* row = cvImg.ptr<uint8_t>(y);
				for (int x = 0; x < (int)width; ++x) {
					const size_t idx = (y * width + x) * stride;
					if (row[x * 3 + 0] != writeBuffer[idx + 0] ||
						row[x * 3 + 1] != writeBuffer[idx + 1] ||
						row[x * 3 + 2] != writeBuffer[idx + 2])
					{
						VERBOSE("error: CImageTIFF::Test: OpenCV reading mismatch at (%d,%d)", x, y);
						return false;
					}
				}
			}
		}
		File::deleteFile(fileName);
	}

	// 2) Write and read-back a Grayscale TIFF
	{
		const String fileName(dir + "test_gray.tif");
		const Size width = 32, height = 24;
		const Size stride = 1;
		std::vector<uint8_t> writeBuffer(width * height);
		for (size_t i = 0; i < writeBuffer.size(); ++i)
			writeBuffer[i] = uint8_t((i * 7) % 256);
		{
			CAutoPtr<CImage> pImage(CImage::Create(fileName, CImage::WRITE));
			if (pImage == NULL ||
				!pImage->WriteHeader(PF_GRAY8, width, height, 1) ||
				!pImage->WriteData(writeBuffer.data(), PF_GRAY8, stride, width * stride))
			{
				VERBOSE("error: CImageTIFF::Test: failed writing Gray TIFF '%s'", fileName.c_str());
				return false;
			}
		}
		{
			CAutoPtr<CImage> pImage(CImage::Create(fileName, CImage::READ));
			if (pImage == NULL || !pImage->ReadHeader()) {
				VERBOSE("error: CImageTIFF::Test: failed reading Gray header of '%s'", fileName.c_str());
				return false;
			}
			if (pImage->GetWidth() != width || pImage->GetHeight() != height ||
				pImage->GetStride() != 1 || pImage->GetFormat() != PF_GRAY8)
			{
				VERBOSE("error: CImageTIFF::Test: Gray header mismatch for '%s'", fileName.c_str());
				return false;
			}
			std::vector<uint8_t> readBuffer(width * height);
			if (!pImage->ReadData(readBuffer.data(), PF_GRAY8, 1, width)) {
				VERBOSE("error: CImageTIFF::Test: failed reading Gray data of '%s'", fileName.c_str());
				return false;
			}
			for (size_t i = 0; i < writeBuffer.size(); ++i) {
				if (readBuffer[i] != writeBuffer[i]) {
					VERBOSE("error: CImageTIFF::Test: Gray pixel mismatch at byte %zu", i);
					return false;
				}
			}
		}
		File::deleteFile(fileName);
	}

	// 3) Write and read-back an RGBA TIFF
	{
		const String fileName(dir + "test_rgba.tif");
		const Size width = 32, height = 24;
		const Size stride = 4;
		std::vector<uint8_t> writeBuffer(width * height * stride);
		for (Size y = 0; y < height; ++y) {
			for (Size x = 0; x < width; ++x) {
				const size_t idx = (y * width + x) * stride;
				// In OpenCV BGRA format (PF_R8G8B8A8): byte 0 = B, 1 = G, 2 = R, 3 = A
				writeBuffer[idx + 0] = uint8_t((180 + x) % 256); // B
				writeBuffer[idx + 1] = uint8_t((90 + y) % 256);  // G
				writeBuffer[idx + 2] = uint8_t((30 + x * y) % 256); // R
				writeBuffer[idx + 3] = uint8_t((240 - x) % 256); // A
			}
		}
		{
			CAutoPtr<CImage> pImage(CImage::Create(fileName, CImage::WRITE));
			if (pImage == NULL ||
				!pImage->WriteHeader(PF_R8G8B8A8, width, height, 1) ||
				!pImage->WriteData(writeBuffer.data(), PF_R8G8B8A8, stride, width * stride))
			{
				VERBOSE("error: CImageTIFF::Test: failed writing RGBA TIFF '%s'", fileName.c_str());
				return false;
			}
		}
		{
			CAutoPtr<CImage> pImage(CImage::Create(fileName, CImage::READ));
			if (pImage == NULL || !pImage->ReadHeader()) {
				VERBOSE("error: CImageTIFF::Test: failed reading RGBA header of '%s'", fileName.c_str());
				return false;
			}
			if (pImage->GetWidth() != width || pImage->GetHeight() != height ||
				pImage->GetStride() != 4 || pImage->GetFormat() != PF_B8G8R8A8 ||
				!pImage->FormatHasAlpha())
			{
				VERBOSE("error: CImageTIFF::Test: RGBA header mismatch for '%s'", fileName.c_str());
				return false;
			}
			std::vector<uint8_t> readBuffer(width * height * stride);
			if (!pImage->ReadData(readBuffer.data(), PF_R8G8B8A8, stride, width * stride)) {
				VERBOSE("error: CImageTIFF::Test: failed reading RGBA data of '%s'", fileName.c_str());
				return false;
			}
			for (size_t i = 0; i < writeBuffer.size(); ++i) {
				if (readBuffer[i] != writeBuffer[i]) {
					VERBOSE("error: CImageTIFF::Test: RGBA pixel mismatch at byte %zu (read %u != written %u)",
						i, (unsigned)readBuffer[i], (unsigned)writeBuffer[i]);
					return false;
				}
			}
		}
		File::deleteFile(fileName);
	}

	return true;
} // Test

#endif // _USE_TESTS
/*----------------------------------------------------------------*/

#endif // _IMAGE_TIFF
