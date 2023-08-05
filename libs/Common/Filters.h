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

template<bool managed=true>
class BufferedInputStream : public LayerInputStream<managed> {
public:
	typedef LayerInputStream<managed> Base;

	BufferedInputStream(InputStream* aStream, size_t aBufSize)
		: Base(aStream), buf(new uint8_t[aBufSize]), bufSize(aBufSize), cache(0), pos(0) { ASSERT(aBufSize > 0); }
	virtual ~BufferedInputStream() {
		delete[] buf;
	}

	size_t		read(void* wbuf, size_t len) override {
		uint8_t* b = (uint8_t*)wbuf;
		const size_t l2 = len;
		do {
			ASSERT(pos <= cache);
			if (pos == cache) {
				if (len >= bufSize) {
					const size_t r = Base::read(b, len);
					if (r == STREAM_ERROR)
						return STREAM_ERROR;
					return l2 - len + r;
				}
				pos = 0;
				switch (cache = Base::read(buf, bufSize)) {
					case 0:
						return l2 - len;
					case STREAM_ERROR:
						return STREAM_ERROR;
				}
			}
			const size_t n = MINF(cache - pos, len);
			memcpy(b, buf + pos, n);
			b += n;
			pos += n;
			len -= n;
		} while (len > 0);
		return l2;
	}

	bool		setPos(size_f_t wpos) override {
		pos = cache = 0;
		return Base::setPos(wpos);
	}

	size_f_t	getPos() const override {
		const size_f_t r = Base::getPos();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r-(cache-pos);
	}

	enum { LAYER_ID_IN=1 };
	InputStream* getInputStream(int typ=InputStream::LAYER_ID_IN) override { return (typ == LAYER_ID_IN ? static_cast<InputStream*>(this) : Base::getInputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t bufSize;
	size_t cache;
	size_t pos;
};


template<bool managed=true>
class BufferedOutputStream : public LayerOutputStream<managed> {
public:
	typedef LayerOutputStream<managed> Base;

	BufferedOutputStream(OutputStream* aStream, size_t aBufSize)
		: Base(aStream), buf(new uint8_t[aBufSize]), bufSize(aBufSize), pos(0) { }
	virtual ~BufferedOutputStream() {
		flush();
		delete[] buf;
	}

	size_t		write(const void* wbuf, size_t len) override {
		if (len < bufSize - pos) {
			memcpy(buf + pos, (const uint8_t*)wbuf, len);
			pos += len;
			return len;
		}
		uint8_t* b = (uint8_t*)wbuf;
		const size_t l2 = len;
		do {
			if (pos == bufSize) {
				if (Base::write(buf, bufSize) == STREAM_ERROR)
					return STREAM_ERROR;
				pos = 0;
				if (len < bufSize) {
					if (Base::write(b, len) == STREAM_ERROR)
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

	size_f_t	getSize() const override {
		const size_f_t r = Base::getSize();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r + pos;
	}

	bool		setPos(size_f_t wpos) override {
		if (pos > 0) {
			const size_t ret = Base::write(buf, pos);
			pos = 0;
			if (ret == STREAM_ERROR)
				return false;
		}
		return Base::setPos(wpos);
	}

	size_f_t	getPos() const override {
		const size_f_t r = Base::getPos();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r + pos;
	}

	size_t		flush() override {
		size_t ret = 0;
		if (pos > 0) {
			ret = Base::write(buf, pos);
			pos = 0;
			if (ret == STREAM_ERROR)
				return STREAM_ERROR;
		}
		return ret + Base::flush();
	}

	enum { LAYER_ID_OUT=1 };
	OutputStream* getOutputStream(int typ=OutputStream::LAYER_ID_OUT) override { return (typ == LAYER_ID_OUT ? static_cast<OutputStream*>(this) : Base::getOutputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t bufSize;
	size_t pos;
};
/*----------------------------------------------------------------*/



template<class Filter, bool managed=true>
class FilteredInputStream : public LayerInputStream<managed> {
public:
	typedef LayerInputStream<managed> Base;

	FilteredInputStream(InputStream* pFile, size_t nFilteredSize, size_t nBufSize, void* pData)
		: Base(pFile), buf(new uint8_t[nBufSize]), filteredSize(nFilteredSize), bufSize(nBufSize), valid(0), pos(0), filter(pData), more(true) {}
	virtual ~FilteredInputStream() {
		delete[] buf;
	}

	/**
	* Read data through filter, keep calling until len returns 0.
	* @param rbuf Data buffer
	* @param len Buffer size on entry, bytes actually read on exit
	* @return Length of data in buffer
	*/
	size_t		read(void* rbuf, size_t len) override {
		uint8_t* rb = (uint8_t*)rbuf;

		const size_t l2 = len;
		while (more && len) {
			if (pos == valid) {
				valid = Base::read(buf, bufSize);
				if (valid == STREAM_ERROR)
					return STREAM_ERROR;
				pos = 0;
			}
			size_t m = valid - pos;
			size_t n = len;
			more = filter(buf + pos, m, rb, n);
			pos += m;
			rb += n;
			len -= n;
		}
		return l2-len;
	}

	size_f_t	getSize() const override {
		return filteredSize;
	}

	bool		setPos(size_f_t wpos) override {
		valid = pos = 0;
		return Base::setPos(wpos);
	}

	size_f_t	getPos() const override {
		const size_f_t r = Base::getPos();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r-(valid-pos);
	}

	enum { LAYER_ID_IN=2 };
	InputStream* getInputStream(int typ=InputStream::LAYER_ID_IN) override { return (typ == LAYER_ID_IN ? static_cast<InputStream*>(this) : Base::getInputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t filteredSize;
	const size_t bufSize;
	size_t valid;
	size_t pos;
	Filter filter;
	bool more;
};


template<class Filter, bool managed=true>
class FilteredOutputStream : public LayerOutputStream<managed> {
public:
	typedef LayerOutputStream<managed> Base;

	FilteredOutputStream(OutputStream* aFile, size_t aBufSize, void* pData)
		: Base(aFile), buf(new uint8_t[aBufSize]), bufSize(aBufSize), filter(pData), flushed(false) {}
	virtual ~FilteredOutputStream() {
		flush();
		delete[] buf;
	}

	size_t		write(const void* wbuf, size_t len) override {
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

			const size_t r = Base::write(buf, n);
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

	size_t		flush() override {
		if (flushed)
			return Base::flush();

		flushed = true;
		size_t written = 0;

		while (true) {
			size_t n = bufSize;
			size_t zero = 0;
			bool more = filter(NULL, zero, buf, n);

			written += Base::write(buf, n);

			if (!more)
				break;
		}
		const size_t r = Base::flush();
		if (r == STREAM_ERROR)
			return STREAM_ERROR;
		return written + r;
	}

	enum { LAYER_ID_OUT=2 };
	OutputStream* getOutputStream(int typ=OutputStream::LAYER_ID_OUT) override { return (typ == LAYER_ID_OUT ? static_cast<OutputStream*>(this) : Base::getOutputStream(typ)); }

private:
	uint8_t* const buf;
	const size_t bufSize;
	Filter filter;
	bool flushed;
};
/*----------------------------------------------------------------*/



#ifdef _UNICODE
#define TOKEN_SIZE		2 //in bytes
#else
#define TOKEN_SIZE		1 //in bytes
#endif
#define TOKEN_MAXBUF	(2048*TOKEN_SIZE) //in bytes
#define TOKEN_MAXIGN	32 //in TCHARs

template<bool managed=true>
class TokenInputStream : public LayerInputStream<managed> {
public:
	typedef LayerInputStream<managed> Base;

	TokenInputStream(InputStream* aStream, TCHAR aToken=_T('\n'))
		: Base(aStream), pos(TOKEN_MAXBUF), eos(false), token(aToken) { arrIgnore[0] = _T('\0'); }
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
		if (n < len * TOKEN_SIZE) {
			eos = true;
			return n / TOKEN_SIZE;
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

	size_t		read(void* wbuf, size_t len) override {
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
		const size_t r = Base::read((uint8_t*)wbuf+n, len);
		if (r == STREAM_ERROR)
			return STREAM_ERROR;
		return n+r;
	}

	bool		setPos(size_f_t wpos) override {
		eos = false;
		pos = TOKEN_MAXBUF;
		return Base::setPos(wpos);
	}

	size_f_t	getPos() const override {
		const size_f_t r = Base::getPos();
		if (r == SIZE_NA)
			return SIZE_NA;
		return r-(TOKEN_MAXBUF-pos);
	}

	bool		isEOS() const {
		return eos;
	}

	enum { LAYER_ID_IN=5 };
	InputStream* getInputStream(int typ=InputStream::LAYER_ID_IN) override { return (typ == LAYER_ID_IN ? static_cast<InputStream*>(this) : Base::getInputStream(typ)); }

private:
	uint8_t buf[TOKEN_MAXBUF+TOKEN_SIZE];
	size_t pos;
	bool eos;
	TCHAR token;
	TCHAR arrIgnore[TOKEN_MAXIGN];
};
/*----------------------------------------------------------------*/



template<bool managed=true>
class MaskInputStream : public LayerInputStream<managed> {
public:
	typedef LayerInputStream<managed> Base;

	MaskInputStream(InputStream* aStream, size_f_t nPos, size_f_t nSize)
		: Base(aStream), startPos(nPos), size(nSize), pos(0) { }
	virtual ~MaskInputStream() { }

	size_t		read(void* wbuf, size_t len) override {
		if (pos >= size)
			return 0;
		if (!Base::setPos(startPos + pos))
			return STREAM_ERROR;
		if (pos+(size_f_t)len > size)
			len = (size_t)(size - pos);
		const size_t r = Base::read(wbuf, len);
		if (r == STREAM_ERROR)
			return STREAM_ERROR;
		pos += r;
		return r;
	}

	size_f_t	getSize() const override {
		return size;
	}

	bool		setPos(size_f_t wpos) override {
		if (wpos > size)
			pos = size;
		else
			pos = wpos;
		return true;
	}

	size_f_t	getPos() const override {
		return pos;
	}

	enum { LAYER_ID_IN=6 };
	InputStream* getInputStream(int typ=InputStream::LAYER_ID_IN) override { return (typ == LAYER_ID_IN ? static_cast<InputStream*>(this) : Base::getInputStream(typ)); }

private:
	const size_f_t size;
	const size_f_t startPos;
	size_f_t pos;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_FILTERS_H__
