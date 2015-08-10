////////////////////////////////////////////////////////////////////
// Image.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGE_H__
#define __SEACAVE_IMAGE_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

#define LT_IMAGE		CImage::ms_nLogType


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// All formats are listed from left to right, most-significant bit to least-significant bit.
// For example, PF_A8R8G8B8 is ordered from the most-significant bit channel A (alpha),
// to the least-significant bit channel B (blue). When traversing image data, the data is
// stored in memory on a little-endian machine from least-significant bit to most-significant bit,
// which means that the channel order in memory is from least-significant bit (blue) to
// most-significant bit (alpha).
typedef enum PIXELFORMAT_TYPE {
	PF_UNKNOWN = 0,
	// gray
	PF_A8,
	PF_GRAY8,
	// uncompressed RGB
	PF_R5G6B5,
	PF_R8G8B8,
	PF_R8G8B8A8,
	PF_A8R8G8B8,
	// uncompressed BGR
	PF_B8G8R8,
	PF_B8G8R8A8,
	PF_A8B8G8R8,
	// compressed
	PF_DXT1 = 128,
	PF_DXT2,
	PF_DXT3,
	PF_DXT4,
	PF_DXT5,
	PF_3DC,
} PIXELFORMAT;

class GENERAL_API CImage
{
	DECLARE_LOG();

public:
	typedef enum IMCREATE_TYPE {
		READ,
		WRITE
	} IMCREATE;

	CImage()			{}
	virtual ~CImage()	{}

	virtual HRESULT		Reset(UINT width, UINT height, PIXELFORMAT pixFormat, UINT levels = 1, bool bAllocate = false);
	virtual HRESULT		Reset(LPCTSTR szFileName, IMCREATE mode);
	virtual HRESULT		Reset(IOSTREAMPTR& pStream);
	virtual void		Close();

	virtual HRESULT		ReadHeader();
	virtual HRESULT		ReadData(void*, PIXELFORMAT, UINT nStride, UINT lineWidth);

	virtual HRESULT		WriteHeader(PIXELFORMAT, UINT width, UINT height, BYTE numLevels);
	virtual HRESULT		WriteData(void*, PIXELFORMAT, UINT nStride, UINT lineWidth);

	const IOSTREAMPTR&	GetStream() const		{ return m_pStream; }
	IOSTREAMPTR&		GetStream()				{ return m_pStream; }
	BYTE*				GetData() const			{ return m_data; }
	BYTE*&				GetData()				{ return m_data; }
	size_t				GetDataSize() const		{ return m_lineWidth * m_dataHeight; }
	UINT				GetWidth() const		{ return m_width; }
	UINT				GetHeight() const		{ return m_height; }
	UINT				GetDataWidth() const	{ return m_dataWidth; }
	UINT				GetDataHeight() const	{ return m_dataHeight; }
	UINT				GetStride() const		{ return m_stride; }
	UINT				GetLineWidth() const	{ return m_lineWidth; }
	BYTE				GetNumLevels() const	{ return m_numLevels; }
	PIXELFORMAT			GetFormat() const		{ return m_format; }
	bool				FormatHasAlpha() const	{ return FormatHasAlpha(m_format); }
	const String&		GetFileName() const		{ return m_fileName; }
	String&				GetFileName()			{ return m_fileName; }

	UINT				GetDataSizes(UINT mipLevel, UINT& width, UINT& height) const;

	static UINT			GetStride(PIXELFORMAT); //in bits
	static bool			FormatHasAlpha(PIXELFORMAT);
	static bool			FilterFormat(void*, PIXELFORMAT, UINT, const void*, PIXELFORMAT, UINT, UINT nSzize);
	static void			FlipRB24(uint8_t* data, UINT size, UINT stride);
	static void			CopyFlipRB24(uint8_t* pDst, const uint8_t* pSrc, UINT size, UINT strideDst, UINT strideSrc);

	static CImage*		Create(LPCTSTR szName, IMCREATE mode);

	#ifndef _RELEASE
	void				Dump(LPCTSTR szFileName);
	#endif

protected:
	IOSTREAMPTR			m_pStream;		// stream used to read/write the image data
	CAutoPtrArr<BYTE>	m_data;			// image's data buffer
	UINT				m_width;		// image width in pixels
	UINT				m_height;		// image height in pixels
	UINT				m_dataWidth;	// image's data width including mipmaps
	UINT				m_dataHeight;	// image's data height
	UINT				m_stride;		// bytes per pixel
	UINT				m_lineWidth;	// image canvas width in bytes
	PIXELFORMAT			m_format;		// image format (pixel type)
	BYTE				m_numLevels;	// number of mipmap levels (0 = auto-generate)
	BYTE				m_level;		// index of the mipmap level currently reading
	String				m_fileName;		// image's file name
}; // class CImage
typedef CSharedPtr<CImage> IMAGEPTR;
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGE_H__
