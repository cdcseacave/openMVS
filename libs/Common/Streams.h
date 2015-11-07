////////////////////////////////////////////////////////////////////
// Streams.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_STREAMS_H__
#define __SEACAVE_STREAMS_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

#define SIZE_NA					((size_f_t)-1)
#define STREAM_ERROR			((size_t)-1)

#define FILE_WRITE_MINBUF_SIZE	(128*1024)						// The min size the file write buffer should allocate.
#define FILE_WRITE_MAXBUF_SIZE	(100*1024*1024)					// The max size the file write buffer should allocate.
#define FILE_READ_MINBUF_SIZE	(512*1024)						// The min size the file read buffer should allocate.
#define FILE_READ_MAXBUF_SIZE	(200*1024*1024)					// The max size the file read buffer should allocate.

#define LAYER_BASE				0


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
	// Get the length of the stream.
	//         SIZE_NA if there were errors.
	virtual size_f_t	getSize() const = 0;
	// Position the stream at the given offset from the beginning.
	virtual bool		setPos(size_f_t pos) = 0;
	// Get the current position in the stream (offset from the beginning).
	//         SIZE_NA if there were errors.
	virtual size_f_t	getPos() const = 0;
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
			TCHAR* const buffer(new TCHAR[count]);
			_vsntprintf(buffer, count, szFormat, args);
			va_end(args);
			const size_t size(write(buffer, count));
			delete[] buffer;
			return size;
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

	virtual OutputStream* getOutputStream(int typ) { return (typ == LAYER_BASE ? this : NULL); }
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

	virtual InputStream* getInputStream(int typ) { return (typ == LAYER_BASE ? this : NULL); }
};

class GENERAL_API NOINITVTABLE IOStream : public InputStream, public OutputStream {
public:
	static LPCSTR getClassType() { return "IOStream"; }
	virtual LPCSTR getClassName() const { return IOStream::getClassType(); }
};

template<bool managed>
class LayerOutputStream : public OutputStream {
public:
	LayerOutputStream(OutputStream* aStream) : s(aStream) {}
	virtual ~LayerOutputStream() { if (managed) delete s; }

	OutputStream* getOutputStream(int typ) { return s->getOutputStream(typ); }

	size_f_t	getSize() const {
		return s->getSize();
	}

	bool		setPos(size_f_t wpos) {
		return s->setPos(wpos);
	}

	size_f_t	getPos() const {
		return s->getPos();
	}

protected:
	OutputStream* const s;
};

template<bool managed>
class LayerInputStream : public InputStream {
public:
	LayerInputStream(InputStream* aStream) : s(aStream) { ASSERT(s != NULL); }
	virtual ~LayerInputStream() { if (managed) delete s; }

	InputStream* getInputStream(int typ) { return s->getInputStream(typ); }

	size_f_t	getSize() const {
		return s->getSize();
	}

	bool		setPos(size_f_t wpos) {
		return s->setPos(wpos);
	}

	size_f_t	getPos() const {
		return s->getPos();
	}

protected:
	InputStream* const s;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_STREAMS_H__
