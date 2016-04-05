////////////////////////////////////////////////////////////////////
// Filters.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_FILTERS_H__
#define __SEACAVE_FILTERS_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Streams.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

#define LAYER_MASK	1

template<bool managed>
class MaskInputStream : public LayerInputStream<managed> {
public:
	MaskInputStream(InputStream* aStream, size_f_t nPos, size_f_t nSize) : LayerInputStream<managed>(aStream), startPos(nPos), size(nSize), pos(0) { }
	virtual ~MaskInputStream() { }

	size_t read(void* wbuf, size_t len) {
		if (pos >= size)
			return 0;
		if (!this->s->setPos(startPos + pos))
			return STREAM_ERROR;
		if (pos+(size_f_t)len > size)
			len = (size_t)(size - pos);
		const size_t r = this->s->read(wbuf, len);
		if (r == STREAM_ERROR)
			return STREAM_ERROR;
		pos += r;
		return r;
	}

	size_f_t	getSize() const {
		return size;
	}

	bool		setPos(size_f_t wpos) {
		if (wpos > size)
			pos = size;
		else
			pos = wpos;
		return true;
	}

	size_f_t	getPos() const {
		return pos;
	}

	InputStream* getInputStream(int typ) { return (typ == LAYER_MASK ? this : this->s->getInputStream(typ)); }

private:
	const size_f_t size;
	const size_f_t startPos;
	size_f_t pos;
};


#define LAYER_BUFFER	2

template<bool managed>
class BufferedOutputStream : public LayerOutputStream<managed> {
public:
	BufferedOutputStream(OutputStream* aStream, size_t aBufSize) : LayerOutputStream<managed>(aStream), buf(new uint8_t[aBufSize]), bufSize(aBufSize), pos(0) { }
	virtual ~BufferedOutputStream() {
		flush();
		delete[] buf;
	}

	size_t flush() {
		size_t ret = 0;
		if (pos > 0) {
			ret = this->s->write(buf, pos);
			pos = 0;
			if (ret == STREAM_ERROR)
				return STREAM_ERROR;
		}
		return ret + this->s->flush();
	}

	size_t write(const void* wbuf, size_t len) {
		if (len < bufSize - pos) {
			memcpy(buf + pos, (const uint8_t*)wbuf, len);
			pos += len;
			return len;
		}
		uint8_t* b = (uint8_t*)wbuf;
		const size_t l2 = len;
		do {
			if (pos == bufSize) {
				if (this->s->write(buf, bufSize) == STREAM_ERROR)
					return STREAM_ERROR;
				pos = 0;
				if (len >= bufSize) {
					if (this->s->write(b, len) == STREAM_ERROR)
						return STREAM_ERROR;
					break;
				}
			}
			const size_t n = MINF(bufSize - pos, len);
			memcpy(buf + pos, b, n);
			b += n;
			pos += n;
			len -= n;
		} while (len > 0);
		return l2;
	}

	OutputStream* getOutputStream(int typ) { return (typ == LAYER_BUFFER ? this : this->s->getOutputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t bufSize;
	size_t pos;
};


template<bool managed>
class BufferedInputStream : public LayerInputStream<managed> {
public:
	BufferedInputStream(InputStream* aStream, size_t aBufSize) : LayerInputStream<managed>(aStream), buf(new uint8_t[aBufSize]), bufSize(aBufSize), maxCache(0), caching(0), pos(0) { }
	virtual ~BufferedInputStream() {
		delete[] buf;
	}

	size_t read(void* wbuf, size_t len) {
		uint8_t* b = (uint8_t*)wbuf;
		const size_t l2 = len;
		do {
			if (caching < maxCache) {
				size_t sliceLen = bufSize-caching;
				if (sliceLen > FILE_READ_MINBUF_SIZE+(FILE_READ_MINBUF_SIZE/2))
					sliceLen = FILE_READ_MINBUF_SIZE;
				const size_t r = this->s->read(buf+caching, sliceLen);
				if (r == STREAM_ERROR)
					return STREAM_ERROR;
				caching += r;
				if (r != sliceLen) {
					maxCache = caching;
				}
			}
			else if (pos == maxCache) {
				if (len >= bufSize) {
					const size_t r = this->s->read(b, len);
					if (r == STREAM_ERROR)
						return STREAM_ERROR;
					return l2 - len + r;
				}
				pos = 0;
				switch (caching = this->s->read(buf, FILE_READ_MINBUF_SIZE)) {
					case 0:
						maxCache = 0;
						return l2 - len;
					case FILE_READ_MINBUF_SIZE:
						maxCache = bufSize;
						break;
					case STREAM_ERROR:
						return STREAM_ERROR;
					default: // < FILE_READ_MINBUF_SIZE
						maxCache = caching;
				}
			}
			const size_t n = MINF(caching - pos, len);
			memcpy(b, buf + pos, n);
			b += n;
			pos += n;
			len -= n;
		} while (len > 0);
		return l2;
	}

	bool		setPos(size_f_t wpos) {
		pos = caching = maxCache = 0;
		return this->s->setPos(wpos);
	}

	size_f_t	getPos() const {
		const size_f_t r = this->s->getPos();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r-(caching-pos);
	}

	InputStream* getInputStream(int typ) { return (typ == LAYER_BUFFER ? this : this->s->getInputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t bufSize;
	size_t maxCache;
	size_t caching;
	size_t pos;
};


#define LAYER_ZIP	3

template<class Filter, bool managed>
class FilteredOutputStream : public LayerOutputStream<managed> {
public:
	using OutputStream::write;

	FilteredOutputStream(OutputStream* aFile, size_t aBufSize, void* pData) : LayerOutputStream<managed>(aFile), buf(new uint8_t[aBufSize]), bufSize(aBufSize), filter(pData), flushed(false) {}
	virtual ~FilteredOutputStream() {
		delete[] buf;
	}

	size_t flush() {
		if (flushed)
			return this->s->flush();

		flushed = true;
		size_t written = 0;

		for (;;) {
			size_t n = bufSize;
			size_t zero = 0;
			bool more = filter(NULL, zero, buf, n);

			written += this->s->write(buf, n);

			if (!more)
				break;
		}
		const size_t r = this->s->flush();
		if (r == STREAM_ERROR)
			return STREAM_ERROR;
		return written + r;
	}

	size_t write(const void* wbuf, size_t len) {
		if (flushed)
			return STREAM_ERROR;

		const uint8_t* wb = (const uint8_t*)wbuf;
		size_t written = 0;
		while (len > 0) {
			size_t n = bufSize;
			size_t m = len;

			const bool more = filter(wb, m, buf, n);
			wb += m;
			len -= m;

			const size_t r = this->s->write(buf, n);
			if (r == STREAM_ERROR)
				return STREAM_ERROR;
			written += r;

			if (!more) {
				if (len > 0)
					return STREAM_ERROR;
				flushed = true;
				return written;
			}
		}
		return written;
	}

	OutputStream* getOutputStream(int typ) { return (typ == LAYER_ZIP ? this : this->s->getOutputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t bufSize;
	Filter filter;
	bool flushed;
};

template<class Filter, bool managed>
class FilteredInputStream : public LayerInputStream<managed> {
public:
	FilteredInputStream(InputStream* pFile, size_t nFilteredSize, size_t nBufSize, void* pData) : LayerInputStream<managed>(pFile), buf(new uint8_t[nBufSize]), filteredSize(nFilteredSize), bufSize(nBufSize), valid(0), pos(0), filter(pData), more(true) {}
	virtual ~FilteredInputStream() {
		delete[] buf;
	}

	/**
	* Read data through filter, keep calling until len returns 0.
	* @param rbuf Data buffer
	* @param len Buffer size on entry, bytes actually read on exit
	* @return Length of data in buffer
	*/
	size_t read(void* rbuf, size_t len) {
		uint8_t* rb = (uint8_t*)rbuf;

		const size_t l2 = len;
		while (more && len) {
			#if ZIP_MODE == BZIP2_MODE
			size_t n = len;
			filter.flush(rb, n);
			rb += n;
			len -= n;
			#endif

			if (pos == valid) {
				valid = this->s->read(buf, bufSize);
				if (valid == STREAM_ERROR)
					return STREAM_ERROR;
				pos = 0;
			}

			size_t m = valid - pos;
			#if ZIP_MODE == BZIP2_MODE
			n = len;
			#else
			size_t n = len;
			#endif
			more = filter(buf + pos, m, rb, n);
			pos += m;
			rb += n;
			len -= n;
		}
		return l2-len;
	}

	size_f_t	getSize() const {
		return filteredSize;
	}

	bool		setPos(size_f_t wpos) {
		valid = pos = 0;
		return this->s->setPos(wpos);
	}

	size_f_t	getPos() const {
		const size_f_t r = this->s->getPos();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r-(valid-pos);
	}

	InputStream* getInputStream(int typ) { return (typ == LAYER_ZIP ? this : this->s->getInputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t filteredSize;
	const size_t bufSize;
	size_t valid;
	size_t pos;
	Filter filter;
	bool more;
};


#define LAYER_TOKEN	4

#ifdef _UNICODE
#define TOKEN_SIZE		2 //in bytes
#else
#define TOKEN_SIZE		1 //in bytes
#endif
#define TOKEN_MAXBUF	(2048*TOKEN_SIZE) //in bytes
#define TOKEN_MAXIGN	32 //in TCHARs
template<bool managed>
class TokenInputStream : public LayerInputStream<managed> {
public:
	typedef LayerInputStream<managed> Base;

	TokenInputStream(InputStream* aStream, TCHAR aToken=_T('\n')) : LayerInputStream<managed>(aStream), pos(TOKEN_MAXBUF), eos(false), token(aToken) { arrIgnore[0] = _T('\0'); }
	virtual ~TokenInputStream() {
	}

	void		emptyBuffer() {
		setPos(getPos());
	}

	TCHAR		getToken() const {
		return token;
	}

	void		setToken(TCHAR aToken) {
		token = aToken;
	}

	LPCTSTR		getTrimTokens() const {
		return arrIgnore;
	}

	size_t		setTrimTokens(LPCTSTR szIgnore) {
		const size_t len = _tcslen(szIgnore);
		if (len >= TOKEN_MAXIGN)
			return 0;
		_tcscpy(arrIgnore, szIgnore);
		return len;
	}

	LPTSTR		trimFrontLine(LPTSTR line) {
		while (line[0] != _T('\0')) {
			if (_tcschr(arrIgnore, line[0]) == NULL)
				return line;
			++line;
		}
		return line;
	}

	LPTSTR		trimFrontLine(MemFile& memFile) {
		memFile.ensureSize(1);
		LPTSTR line = (LPTSTR)memFile.getData();
		line[memFile.getSizeLeft()] = _T('\0');
		LPTSTR newLine = trimFrontLine(line);
		memFile.movePos(newLine-line);
		return newLine;
	}

	size_t		trimBackLine(LPTSTR line) {
		const size_t len = _tcslen(line);
		size_t i = len;
		while (i > 0 && line[--i] != _T('\0')) {
			if (_tcschr(arrIgnore, line[i]) == NULL)
				return len-(i+1);
			line[i] = _T('\0');
		}
		return len;
	}

	size_t		trimBackLine(MemFile& memFile) {
		memFile.ensureSize(1);
		LPTSTR line = (LPTSTR)memFile.getData();
		line[memFile.getSizeLeft()] = _T('\0');
		const size_t trimedSize = trimBackLine(line);
		memFile.moveSize(-((size_f_t)trimedSize));
		return trimedSize;
	}

	// revert the deleted token during the last readLine()
	void		restoreToken() {
		LPTSTR line = (LPTSTR)buf;
		ASSERT(pos > 0);
		line[--pos] = token;
	}

	size_t		readLine(LPTSTR wbuf, size_t len) {
		if (eos)
			return 0;
		uint8_t* b = (uint8_t*)wbuf;
		const size_t l2 = len;
		// process chunks of TOKEN_MAXBUF bytes
		while (len >= TOKEN_MAXBUF / TOKEN_SIZE) {
			const size_t n = read(b, TOKEN_MAXBUF);
			if (n == STREAM_ERROR)
				return STREAM_ERROR;
			*((TCHAR*)(b+n)) = _T('\0');
			LPTSTR t = _tcschr((TCHAR*)b, token);
			// if token found...
			if (t != NULL) {
				// ... set the end of line and return it
				t[0] = _T('\0');
				if (n == TOKEN_SIZE && len != 1) {
					eos = true;
					return l2-len;
				}
				++t;
				const size_t bytesParsed = (uint8_t*)t-b;
				const size_t bytesNotParsed = n - bytesParsed;
				pos = TOKEN_MAXBUF - bytesNotParsed;
				// store the unprocessed data
				memcpy(buf+pos, (uint8_t*)t, bytesNotParsed);
				len -= (bytesParsed - TOKEN_SIZE) / TOKEN_SIZE;
				return l2-len;
			}
			len -= n / TOKEN_SIZE;
			// if end of stream return
			if (n < TOKEN_MAXBUF) {
				eos = true;
				return l2-len;
			}
			b += n;
			// if we reached the end of the buffer, signal it
			if (len == 0)
				return l2+1;
		}
		// process the last sub-chunk part
		const size_t n = read(b, len*TOKEN_SIZE);
		if (n == STREAM_ERROR)
			return STREAM_ERROR;
		*((TCHAR*)(b+n)) = _T('\0');
		LPTSTR t = _tcschr((TCHAR*)b, token);
		// if token found...
		if (t != NULL) {
			// ... set the end of line and return it
			t[0] = _T('\0');
			if (n == TOKEN_SIZE && len != 1) {
				eos = true;
				return l2-len;
			}
			++t;
			const size_t bytesParsed = (uint8_t*)t-b;
			const size_t bytesNotParsed = n - bytesParsed;
			pos = TOKEN_MAXBUF - bytesNotParsed;
			// store the unprocessed data
			memcpy(buf+pos, (uint8_t*)t, bytesNotParsed);
			len -= (bytesParsed - TOKEN_SIZE) / TOKEN_SIZE;
			return l2-len;
		}
		// if end of stream return
		if (n < len / TOKEN_SIZE) {
			eos = true;
			return l2-len;
		}
		// we reached the end of the buffer, signal it
		return l2+1;
	}

	size_t		readLine(MemFile& memFile)
	{
		if (eos)
			return 0;
		// make sure we read one full line
		const size_f_t oldSize = memFile.getSize();
		while (true) {
			memFile.ensureSize((4096+1)*TOKEN_SIZE);
			const size_t ret = readLine((LPTSTR)(memFile.getBuffer()+memFile.getSize()), 4096);
			if (ret == STREAM_ERROR)
				return STREAM_ERROR;
			if (ret <= 4096) {
				memFile.growSize(ret*TOKEN_SIZE);
				break;
			}
			memFile.growSize(4096*TOKEN_SIZE);
		}
		return size_t(memFile.getSize()-oldSize);
	}

	// read all to the memfile
	size_t		read(MemFile& memFile)
	{
		if (eos)
			return 0;
		const size_t len = (size_t)(Base::getSize() - getPos());
		memFile.ensureSize(len);
		const size_t ret = read(memFile.getBuffer()+memFile.getSize(), len);
		if (ret == STREAM_ERROR)
			return STREAM_ERROR;
		memFile.growSize(ret);
		return ret;
	}

	size_t		read(void* wbuf, size_t len) {
		size_t n = 0;
		if (pos < TOKEN_MAXBUF) {
			n = TOKEN_MAXBUF-pos;
			if (n > len)
				n = len;
			memcpy(wbuf, buf+pos, n);
			len -= n;
			pos += n;
		}
		if (len == 0)
			return n;
		const size_t r = this->s->read((uint8_t*)wbuf+n, len);
		if (r == STREAM_ERROR)
			return STREAM_ERROR;
		return n+r;
	}

	bool		setPos(size_f_t wpos) {
		eos = false;
		pos = TOKEN_MAXBUF;
		return this->s->setPos(wpos);
	}

	size_f_t	getPos() const {
		const size_f_t r = this->s->getPos();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r-(TOKEN_MAXBUF-pos);
	}

	bool		isEOS() const {
		return eos;
	}

	InputStream* getInputStream(int typ) { return (typ == LAYER_TOKEN ? this : this->s->getInputStream(typ)); }

private:
	uint8_t buf[TOKEN_MAXBUF+TOKEN_SIZE];
	size_t pos;
	bool eos;
	TCHAR token;
	TCHAR arrIgnore[TOKEN_MAXIGN];
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_FILTERS_H__
