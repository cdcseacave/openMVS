////////////////////////////////////////////////////////////////////
// ImageEXIF.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_EXIF
#include "ImageEXIF.h"

#if !defined(EXV_HAVE_STDINT_H) && _MSC_VER < 1600
#define EXV_HAVE_STDINT_H
#endif
#include <exiv2/image.hpp>
#include <exiv2/exif.hpp>

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

class Exiv2IO : public Exiv2::BasicIo {
public:
	Exiv2IO(ISTREAM& f) : m_stream(f) {}
	virtual ~Exiv2IO() {}
	virtual int open() { return 0; }
	virtual int close() { return 0; }
	virtual long write(const Exiv2::byte* data, long wcount) { return 0; /*m_stream.write(data, wcount);*/ }
	virtual long write(Exiv2::BasicIo& src) { return 0; }
	virtual int putb(Exiv2::byte data) { return 0; }
	virtual Exiv2::DataBuf read(long rcount)
	{
		Exiv2::DataBuf buf(rcount);
		buf.size_ = (long)m_stream.read(buf.pData_, buf.size_);
		if (buf.size_ == STREAM_ERROR)
			buf.size_ = 0;
		return buf;
	}
	virtual long read(Exiv2::byte* buf, long rcount)
	{
		const size_t ret = m_stream.read(buf, rcount);
		if (ret == STREAM_ERROR)
			return 0;
		return (long)ret;
	}
	virtual int getb()
	{
		int ret = 0;
		if (STREAM_ERROR == m_stream.read(&ret, 1))
			return EOF;
		return ret;
	}
	virtual void transfer(Exiv2::BasicIo& src) {}
	#ifdef _MSC_VER
	virtual int seek(uint64_t offset, Position pos)
	#else
	virtual int seek(long offset, Position pos)
	#endif
	{
		bool ret;
		switch (pos) {
		case beg:
			ret = m_stream.setPos(offset);
			break;
		case cur:
			ret = m_stream.setPos(m_stream.getPos()+offset);
			break;
		case end:
			ret = m_stream.setPos(m_stream.getSize()-offset);
			break;
		}
		if (ret == true)
			return 0;
		return -1;
	}
	virtual Exiv2::byte* mmap(bool isWriteable) { return NULL; }
	virtual int munmap() { return 0; }
	virtual long tell() const { return (long)m_stream.getPos(); }
	virtual long size() const { return (long)m_stream.getSize(); }
	virtual bool isopen() const { return true; }
	virtual int error() const { return 0; }
	virtual bool eof() const { return (m_stream.getPos() == m_stream.getSize()); }
	virtual std::string path() const { return std::string(); }
	#ifdef EXV_UNICODE_PATH
	virtual std::wstring wpath() const { return std::wstring(); }
	#endif
	virtual Exiv2IO::AutoPtr temporary() const { return Exiv2IO::AutoPtr(); }

protected:
	ISTREAM&	m_stream;
}; // class Exiv2IO

struct Exiv2Struct {
	Exiv2::Image::AutoPtr pImage;
};



// S T R U C T S ///////////////////////////////////////////////////

CImageEXIF::CImageEXIF()
{
} // Constructor

CImageEXIF::~CImageEXIF()
{
	//clean up
	Close();
} // Destructor
/*----------------------------------------------------------------*/

void CImageEXIF::Close()
{
}
/*----------------------------------------------------------------*/

HRESULT CImageEXIF::ReadHeader()
{
	m_state = new Exiv2Struct;
	Exiv2Struct& state = *m_state;
	state.pImage = Exiv2::ImageFactory::open(Exiv2::BasicIo::AutoPtr(new Exiv2IO(*((ISTREAM*)m_pStream))));
	if (state.pImage.get() == NULL) {
		m_state.Release();
		return _FAIL;
	}
	state.pImage->readMetadata();
	return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/

HRESULT CImageEXIF::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	return _FAIL;
} // ReadData
/*----------------------------------------------------------------*/

HRESULT CImageEXIF::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	//TODO: to implement the EXIF encoder
	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/

HRESULT CImageEXIF::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	//TODO: to implement the EXIF encoder
	return _OK;
} // WriteData
/*----------------------------------------------------------------*/

bool CImageEXIF::HasEXIF() const
{
	if (m_state == NULL)
		return false;
	return !m_state->pImage->exifData().empty();
} // HasEXIF
bool CImageEXIF::HasIPTC() const
{
	if (m_state == NULL)
		return false;
	return !m_state->pImage->iptcData().empty();
} // HasIPTC
bool CImageEXIF::HasXMP() const
{
	if (m_state == NULL)
		return false;
	return !m_state->pImage->xmpData().empty();
} // HasXMP
/*----------------------------------------------------------------*/

String CImageEXIF::ReadKeyEXIF(const String& name, bool bInterpret) const
{
	const Exiv2Struct& state = *m_state;
	const Exiv2::ExifData& exifData = state.pImage->exifData();
	ASSERT(!exifData.empty());
	const Exiv2::ExifKey key("Exif."+name);
	Exiv2::ExifData::const_iterator it = exifData.findKey(key);
	if (it == exifData.end())
		return String();
	if (bInterpret)
		return it->print();
	return it->value().toString();
} // ReadKeyEXIF
/*----------------------------------------------------------------*/

void CImageEXIF::DumpAll()
{
	const Exiv2Struct& state = *m_state;
	// print EXIF info
	const Exiv2::ExifData& exifData = state.pImage->exifData();
	if (exifData.empty()) {
		LOG(LT_IMAGE, _T("Info: Image constains no EXIF data"));
	} else {
		Exiv2::ExifData::const_iterator end = exifData.end();
		for (Exiv2::ExifData::const_iterator i = exifData.begin(); i != end; ++i)
			LOG(LT_IMAGE, _T("%s %d %s %d %s"), i->key().c_str(), i->tag(), i->typeName(), i->count(), i->print().c_str());
	}
	// print IPTC info
	const Exiv2::IptcData& iptcData = state.pImage->iptcData();
	if (iptcData.empty()) {
		LOG(LT_IMAGE, _T("Info: Image constains no IPTC data"));
	} else {
		Exiv2::IptcData::const_iterator end = iptcData.end();
		for (Exiv2::IptcData::const_iterator i = iptcData.begin(); i != end; ++i)
			LOG(LT_IMAGE, _T("%s %d %s %d %s"), i->key().c_str(), i->tag(), i->typeName(), i->count(), i->print().c_str());
	}
	// print XMP info
	const Exiv2::XmpData& xmpData = state.pImage->xmpData();
	if (xmpData.empty()) {
		LOG(LT_IMAGE, _T("Info: Image constains no XMP data"));
	} else {
		Exiv2::XmpData::const_iterator end = xmpData.end();
		for (Exiv2::XmpData::const_iterator i = xmpData.begin(); i != end; ++i)
			LOG(LT_IMAGE, _T("%s %d %s %d %s"), i->key().c_str(), i->tag(), i->typeName(), i->count(), i->print().c_str());
	}
} // DumpAll
/*----------------------------------------------------------------*/

#endif // _IMAGE_EXIF
