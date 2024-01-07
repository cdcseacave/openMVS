////////////////////////////////////////////////////////////////////
// Streams.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_STREAMS_H__
#define __SEACAVE_STREAMS_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "AutoPtr.h"


// D E F I N E S ///////////////////////////////////////////////////

#define SIZE_NA					((size_f_t)-1)
#define STREAM_ERROR			((size_t)-1)


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class InputStream;
typedef	InputStream ISTREAM;

class OutputStream;
typedef OutputStream OSTREAM;

class IOStream;
typedef IOStream IOSTREAM;

class GENERAL_API NOINITVTABLE Stream {
public:
	virtual ~Stream() { }

	// Identify stream type
	virtual InputStream* getInputStream(int) { return NULL; }
	virtual OutputStream* getOutputStream(int) { return NULL; }
	virtual IOStream* getIOStream(int, int) { return NULL; }

	// Get the length of the stream.
	//         SIZE_NA if there were errors.
	virtual size_f_t	getSize() const = 0;
	// Position the stream at the given offset from the beginning.
	virtual bool		setPos(size_f_t pos) = 0;
	// Get the current position in the stream (offset from the beginning).
	//         SIZE_NA if there were errors.
	virtual size_f_t	getPos() const = 0;
};

class GENERAL_API NOINITVTABLE InputStream : public Stream {
public:
	virtual ~InputStream() { }
	/**
	 * Call this function until it returns 0 to get all bytes.
	 * @return The number of bytes read. len reflects the number of bytes
	 *		   actually read from the stream source in this call.
	 *         STREAM_ERROR if there were errors.
	 */
	virtual size_t	read(void* buf, size_t len) = 0;

	enum { LAYER_ID_IN=0 };
	InputStream* getInputStream(int typ=LAYER_ID_IN) override { return (typ == LAYER_ID_IN ? this : (InputStream*)NULL); }
};

class GENERAL_API NOINITVTABLE OutputStream : public Stream {
public:
	virtual ~OutputStream() { }
	
	/**
	 * @return The actual number of bytes written. len bytes will always be
	 *         consumed, but fewer or more bytes may actually be written,
	 *         for example if the stream is being compressed.
	 *         STREAM_ERROR if there were errors.
	 */
	virtual size_t	write(const void* buf, size_t len) = 0;
	inline  size_t	print(LPCTSTR szFormat, ...) {
		va_list args;
		va_start(args, szFormat);
		TCHAR szBuffer[2048];
		const size_t len((size_t)_vsntprintf(szBuffer, 2048, szFormat, args));
		if (len > 2048) {
			const size_t count((size_t)_vsctprintf(szFormat, args));
			ASSERT(count != (size_t)-1);
			CAutoPtrArr<TCHAR> szBufferDyn(new TCHAR[count+1]);
			_vsntprintf(szBufferDyn, count+1, szFormat, args);
			va_end(args);
			return write(szBufferDyn, count);
		}
		va_end(args);
		return write(szBuffer, len);
	}
	inline  size_t	print(const std::string& str) {
		return write(str.c_str(), str.length());
	}
	template<class T>
	inline  OutputStream& operator<<(const T& val) {
		std::ostringstream ostr;
		ostr << val;
		print(ostr.str());
		return *this;
	}
	/**
	 * This must be called before destroying the object to make sure all data
	 * is properly written. Note that some implementations
	 * might not need it...
	 *
	 * @return The actual number of bytes written.
	 *         STREAM_ERROR if there were errors.
	 */
	virtual size_t	flush() = 0;

	enum { LAYER_ID_OUT=0 };
	OutputStream* getOutputStream(int typ=LAYER_ID_OUT) override { return (typ == LAYER_ID_OUT ? this : (OutputStream*)NULL); }
};

class GENERAL_API NOINITVTABLE IOStream : public InputStream, public OutputStream {
public:
	InputStream* getInputStream(int typ=LAYER_ID_IN) override { return InputStream::getInputStream(typ); }
	OutputStream* getOutputStream(int typ=LAYER_ID_OUT) override { return OutputStream::getOutputStream(typ); }
	IOStream* getIOStream(int typIn=LAYER_ID_IN, int typOut=LAYER_ID_OUT) override {
		return ((InputStream*)this)->getInputStream(typIn) != NULL &&
			((OutputStream*)this)->getOutputStream(typOut) != NULL ? this : (IOStream*)NULL;
	}
};
/*----------------------------------------------------------------*/



template<bool managed=true>
class LayerInputStream : public InputStream {
public:
	LayerInputStream(InputStream* aStream) : s(aStream) { ASSERT(s != NULL); }
	virtual ~LayerInputStream() noexcept { if (managed) delete s; }

	InputStream* getInputStream(int typ=InputStream::LAYER_ID_IN) override { return s->getInputStream(typ); }

	size_t		read(void* wbuf, size_t len) override {
		return s->read(wbuf, len);
	}

	size_f_t	getSize() const override {
		return s->getSize();
	}

	bool		setPos(size_f_t wpos) override {
		return s->setPos(wpos);
	}

	size_f_t	getPos() const override {
		return s->getPos();
	}

protected:
	InputStream* const s;
};

template<bool managed=true>
class LayerOutputStream : public OutputStream {
public:
	LayerOutputStream(OutputStream* aStream) : s(aStream) {}
	virtual ~LayerOutputStream() noexcept { if (managed) delete s; }

	OutputStream* getOutputStream(int typ=OutputStream::LAYER_ID_OUT) override { return s->getOutputStream(typ); }

	size_t		write(const void* buf, size_t len) override {
		return s->write(buf, len);
	}

	size_t		flush() override {
		return s->flush();
	}

	size_f_t	getSize() const override {
		return s->getSize();
	}

	bool		setPos(size_f_t wpos) override {
		return s->setPos(wpos);
	}

	size_f_t	getPos() const override {
		return s->getPos();
	}

protected:
	OutputStream* const s;
};
/*----------------------------------------------------------------*/



template<bool managed=true>
class LayerIOStream : public IOStream {
public:
	LayerIOStream(InputStream* aStream) : s(aStream), sIn(aStream), sOut(NULL) { ASSERT(aStream != NULL); }
	LayerIOStream(OutputStream* aStream) : s(aStream), sIn(NULL), sOut(aStream) { ASSERT(aStream != NULL); }
	LayerIOStream(InputStream* streamIn, OutputStream* streamOut) : s(NULL), sIn(streamIn), sOut(streamOut) { ASSERT(sIn != NULL || sOut != NULL); }
	virtual ~LayerIOStream() noexcept { if (managed) { delete sIn; delete sOut; } }

	InputStream* getInputStream(int typ=InputStream::LAYER_ID_IN) override { return (sIn ? sIn->getInputStream(typ) : NULL); }
	OutputStream* getOutputStream(int typ=OutputStream::LAYER_ID_OUT) override { return (sOut ? sOut->getOutputStream(typ) : NULL); }

	size_f_t	getSize() const override {
		return s->getSize();
	}

	bool		setPos(size_f_t wpos) override {
		return s->setPos(wpos);
	}

	size_f_t	getPos() const override {
		return s->getPos();
	}

	size_t		read(void* wbuf, size_t len) override {
		return sIn->read(wbuf, len);
	}

	size_t		write(const void* buf, size_t len) override {
		return sOut->write(buf, len);
	}

	size_t		flush() override {
		return sOut->flush();
	}

	operator InputStream* () const { return sIn; }
	operator OutputStream* () const { return sOut; }

protected:
	Stream* const s;
	InputStream* const sIn;
	OutputStream* const sOut;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_STREAMS_H__
