////////////////////////////////////////////////////////////////////
// ImageDDS.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_DDS
#include "ImageDDS.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

// These were copied from the DX9 docs. The names are changed
// from the "real" defines since not all platforms have them.

#ifndef MakeFourCC
#define MakeFourCC(ch0, ch1, ch2, ch3) \
	((uint32_t)(uint8_t)(ch0) | ((uint32_t)(uint8_t)(ch1) << 8) | \
	((uint32_t)(uint8_t)(ch2) << 16) | ((uint32_t)(uint8_t)(ch3) << 24 ))
#endif

#define IMAGE_DDS_TYPE	MakeFourCC('D','D','S',' ')	//"DDS " = 4 bytes

#define FOURCC_DXT1		MakeFourCC('D','X','T','1')
#define FOURCC_DXT2		MakeFourCC('D','X','T','2')
#define FOURCC_DXT3		MakeFourCC('D','X','T','3')
#define FOURCC_DXT4		MakeFourCC('D','X','T','4')
#define FOURCC_DXT5		MakeFourCC('D','X','T','5')
#define FOURCC_3DC		MakeFourCC('A','T','I','2')

enum DDSSurfaceDescFlags
{
   DDSDCaps			= 0x00000001l,
   DDSDHeight		= 0x00000002l,
   DDSDWidth		= 0x00000004l,
   DDSDPitch		= 0x00000008l,
   DDSDPixelFormat	= 0x00001000l,
   DDSDMipMapCount	= 0x00020000l,
   DDSDLinearSize	= 0x00080000l,
   DDSDDepth		= 0x00800000l,
};

enum DDSPixelFormatFlags
{
   DDPFAlphaPixels	= 0x00000001,
   DDPFFourCC		= 0x00000004,
   DDPFRGB			= 0x00000040,
};

enum DDSCapFlags
{
   DDSCAPSComplex	= 0x00000008,
   DDSCAPSTexture	= 0x00001000,
   DDSCAPSMipMap	= 0x00400000,

   DDSCAPS2Cubemap	= 0x00000200,
   DDSCAPS2Cubemap_POSITIVEX = 0x00000400,
   DDSCAPS2Cubemap_NEGATIVEX = 0x00000800,
   DDSCAPS2Cubemap_POSITIVEY = 0x00001000,
   DDSCAPS2Cubemap_NEGATIVEY = 0x00002000,
   DDSCAPS2Cubemap_POSITIVEZ = 0x00004000,
   DDSCAPS2Cubemap_NEGATIVEZ = 0x00008000,
   DDSCAPS2Volume	= 0x00200000,
};

typedef struct DDPIXELFORMAT_TYPE
{
    DWORD       dwSize;                 // size of structure
    DWORD       dwFlags;                // pixel format flags
    DWORD       dwFourCC;               // (FOURCC code)
    union
    {
        DWORD   dwRGBBitCount;          // how many bits per pixel
        DWORD   dwYUVBitCount;          // how many bits per pixel
        DWORD   dwZBufferBitDepth;      // how many total bits/pixel in z buffer (including any stencil bits)
        DWORD   dwAlphaBitDepth;        // how many bits for alpha channels
        DWORD   dwLuminanceBitCount;    // how many bits per pixel
        DWORD   dwBumpBitCount;         // how many bits per "buxel", total
        DWORD   dwPrivateFormatBitCount;// Bits per pixel of private driver formats. Only valid in texture
                                        // format list and if DDPF_D3DFORMAT is set
    };
    union
    {
        DWORD   dwRBitMask;             // mask for red bit
        DWORD   dwYBitMask;             // mask for Y bits
        DWORD   dwStencilBitDepth;      // how many stencil bits (note: dwZBufferBitDepth-dwStencilBitDepth is total Z-only bits)
        DWORD   dwLuminanceBitMask;     // mask for luminance bits
        DWORD   dwBumpDuBitMask;        // mask for bump map U delta bits
        DWORD   dwOperations;           // DDPF_D3DFORMAT Operations
    };
    union
    {
        DWORD   dwGBitMask;             // mask for green bits
        DWORD   dwUBitMask;             // mask for U bits
        DWORD   dwZBitMask;             // mask for Z bits
        DWORD   dwBumpDvBitMask;        // mask for bump map V delta bits
        struct
        {
            WORD    wFlipMSTypes;       // Multisample methods supported via flip for this D3DFORMAT
            WORD    wBltMSTypes;        // Multisample methods supported via blt for this D3DFORMAT
        };

    };
    union
    {
        DWORD   dwBBitMask;             // mask for blue bits
        DWORD   dwVBitMask;             // mask for V bits
        DWORD   dwStencilBitMask;       // mask for stencil bits
        DWORD   dwBumpLuminanceBitMask; // mask for luminance in bump map
    };
    union
    {
        DWORD   dwRGBAlphaBitMask;      // mask for alpha channel
        DWORD   dwYUVAlphaBitMask;      // mask for alpha channel
        DWORD   dwLuminanceAlphaBitMask;// mask for alpha channel
        DWORD   dwRGBZBitMask;          // mask for Z channel
        DWORD   dwYUVZBitMask;          // mask for Z channel
    };
} DDSPF;

typedef struct DDSCAPS_TYPE
{
    DWORD       dwCaps;         // capabilities of surface wanted
    DWORD       dwCaps2;
    DWORD       dwCaps3;
    union
    {
        DWORD       dwCaps4;
        DWORD       dwVolumeDepth;
    };
} DDSCAPS;

typedef struct DDSINFOHEADER_TYPE {
	DWORD	dwHeader;				// magic number "DDS "
	DWORD	dwSize;					// size of the DDSURFACEDESC structure
	DWORD	dwFlags;				// determines what fields are valid
	DWORD	dwHeight;				// height of surface to be created
	DWORD	dwWidth;				// width of input surface
	DWORD	dwPitchOrLinearSize;	// distance to start of next line or formless late-allocated optimized surface size
	DWORD	dwDepth;				// the depth if this is a volume texture 
	DWORD	dwMipMapCount;			// number of mip-map levels requestde
	DWORD	dwReserved1[11];
	DDSPF	ddsPixelFormat;			// pixel format description of the surface
	DDSCAPS	ddsCaps;				// direct draw surface capabilities
	DWORD	dwReserved2;
} DDSINFOHEADER;

#define IMAGE_DDS_DDSINFOHEADERSIZE	(sizeof(DDSINFOHEADER)-sizeof(DWORD)/*dwHeader*/)	// should be 124

#define ISBITMASK(r,g,b,a)			(ddsInfo.ddsPixelFormat.dwRBitMask == r && ddsInfo.ddsPixelFormat.dwGBitMask == g && ddsInfo.ddsPixelFormat.dwBBitMask == b && ddsInfo.ddsPixelFormat.dwRGBAlphaBitMask == a )



// S T R U C T S ///////////////////////////////////////////////////

CImageDDS::CImageDDS()
{
} // Constructor

CImageDDS::~CImageDDS()
{
} // Destructor
/*----------------------------------------------------------------*/


HRESULT CImageDDS::ReadHeader()
{
	// read header
	((ISTREAM*)m_pStream)->setPos(0);
	DDSINFOHEADER ddsInfo;
	m_pStream->read(&ddsInfo, sizeof(DDSINFOHEADER));
	if (ddsInfo.dwHeader != IMAGE_DDS_TYPE &&
		ddsInfo.dwSize != IMAGE_DDS_DDSINFOHEADERSIZE &&
		!(ddsInfo.dwFlags & (DDSDCaps | DDSDPixelFormat | DDSDWidth | DDSDHeight))) {	// always include DDSD_CAPS, DDSD_PIXELFORMAT, DDSD_WIDTH, DDSD_HEIGHT
		LOG(LT_IMAGE, "error: invalid DDS image");
		return _INVALIDFILE;
	}

	m_width = ddsInfo.dwWidth;
	m_height = ddsInfo.dwHeight;
	m_numLevels = ddsInfo.dwFlags & DDSDMipMapCount ? (BYTE)ddsInfo.dwMipMapCount : 0;
	m_level = 0;

	// get the format
	m_stride = ddsInfo.ddsPixelFormat.dwRGBBitCount>>3;
	m_format = PF_UNKNOWN;
	if (ddsInfo.ddsPixelFormat.dwFlags & DDPFFourCC)
	{
		switch (ddsInfo.ddsPixelFormat.dwFourCC) 
		{
		case FOURCC_DXT1:
			m_format = PF_DXT1;
			m_stride = 8;
			break;
		case FOURCC_DXT2:
			m_format = PF_DXT2;
			m_stride = 16;
			break;
		case FOURCC_DXT3:
			m_format = PF_DXT3;
			m_stride = 16;
			break;
		case FOURCC_DXT4:
			m_format = PF_DXT4;
			m_stride = 16;
			break;
		case FOURCC_DXT5:
			m_format = PF_DXT5;
			m_stride = 16;
			break;
		case FOURCC_3DC:
			m_format = PF_3DC;
			m_stride = 16;
			break;
		//case 0x74:
		//	m_format=FORMAT_R32G32B32A32F;
		//	break;
		//case 0x71:
		//	m_format=FORMAT_R16G16B16A16F;
		//	break;
		//case 0x70:
		//	m_format=FORMAT_G16R16F;
		//	break;
		//case 0x73:
		//	m_format=FORMAT_G32R32F;
		//	break;
		//case 0x6F:
		//	m_format=FORMAT_R16F;
		//	break;
		//case 0x72:
		//	m_format=FORMAT_R32F;
		//	break;
		}
	} 
	else if (ddsInfo.ddsPixelFormat.dwFlags & DDPFRGB)
	{
		if (ddsInfo.ddsPixelFormat.dwFlags & DDPFAlphaPixels)
		{
			if (ISBITMASK(0xff, 0xff00, 0xff0000, 0xff000000))
				m_format = PF_B8G8R8A8;
			else if (ISBITMASK(0xff0000, 0xff00, 0xff, 0xff000000))
				m_format = PF_R8G8B8A8;
			//else if (ISBITMASK(0xff000000, 0xff0000, 0xff00, 0xff))
			//	m_format = FORMAT_ABGR;
			//else if (ISBITMASK(0x7C00, 0x3E0, 0x1F, 0x8000))
			//	m_format = FORMAT_A1R5G5B5;
		}
		else //no DDPFAlphaPixels
		{
			if (ISBITMASK(0xff, 0xff00, 0xff0000, 0x00))
				m_format = PF_B8G8R8;
			else if (ISBITMASK(0xff0000, 0xff00, 0xff, 0x00))
				m_format = PF_R8G8B8;
			else if (ISBITMASK(0xF800, 0x7E0, 0x1F, 0x00))
				m_format = PF_R5G6B5;
			//else if (ISBITMASK(0xffFF, 0xffFF0000, 0x00, 0x00))
			//	m_format = FORMAT_G16R16;
			//else if (ISBITMASK(0x7C00, 0x3E0, 0x1F, 0x00))
			//	m_format = FORMAT_X1R5G5B5;
		}
	}
	else //no DDPFRGB
	{
		if (ISBITMASK(0x00, 0x00, 0x00, 0xff))
			m_format = PF_A8;
		else if (ISBITMASK(0xff, 0x00, 0x00, 0x00))
			m_format = PF_GRAY8;
		//else if (ISBITMASK(0xffff, 0x00, 0x00, 0x00))
		//	m_format = FORMAT_L16;
		//else if (ISBITMASK(0xff, 0x00, 0x00, 0xff00))
		//	m_format = FORMAT_A8L8;
		//else if (ISBITMASK(0xff, 0xff00, 0x00, 0x00))
		//	m_format = FORMAT_V8U8;
		//else if (ISBITMASK(0xFF, 0xFF00, 0xFF0000, 0xFF000000))
		//	m_format = FORMAT_Q8W8V8U8;
		//else if (ISBITMASK(0xFFFF, 0xFFFF0000, 0x00, 0x00))
		//	m_format = FORMAT_V16U16;
	}
	ASSERT(m_format != PF_UNKNOWN);
	if (m_format == PF_UNKNOWN)
	{
		LOG(LT_IMAGE, "error: unsupported DDS image");
		return _INVALIDFILE;
	}

	m_lineWidth = GetDataSizes(0, m_dataWidth, m_dataHeight);
	return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/


HRESULT CImageDDS::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
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
	m_lineWidth = GetDataSizes(++m_level, m_dataWidth, m_dataHeight);
	return _OK;
} // ReadData
/*----------------------------------------------------------------*/


HRESULT CImageDDS::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	return _FAIL;
} // WriteHeader
/*----------------------------------------------------------------*/


HRESULT CImageDDS::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	return _FAIL;
} // WriteData
/*----------------------------------------------------------------*/

#endif // _IMAGE_DDS