////////////////////////////////////////////////////////////////////
// MemFile.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_MEMFILE_H__
#define __SEACAVE_MEMFILE_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Streams.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class GENERAL_API MemFile : public IOStream {
public:
	MemFile() : m_buffer(NULL), m_sizeBuffer(0), m_size(0), m_pos(0) {
	}
	MemFile(size_t initialSize) : m_buffer(new BYTE[initialSize]), m_sizeBuffer(initialSize), m_size(0), m_pos(0) {
	}
	MemFile(BYTE* data, size_t size) : m_buffer(data), m_sizeBuffer(0), m_size(size), m_pos(0) {
	}
	virtual ~MemFile() {
		close();
	}

	static LPCSTR getClassType() { return "MemFile"; }
	virtual LPCSTR getClassName() const { return MemFile::getClassType(); }

	bool isOpen() const { return m_buffer != NULL; }

	virtual void close() {
		if (!isOpen())
			return;
		if (m_sizeBuffer)
			delete[] m_buffer;
		m_sizeBuffer = 0;
		m_buffer = NULL;
		m_size = 0;
		m_pos = 0;
	}

	virtual size_f_t getSizeBuffer() const {
		return m_sizeBuffer;
	}

	virtual size_f_t getSizeLeft() const {
		return m_size - m_pos;
	}

	virtual size_f_t getSize() const {
		return m_size;
	}

	virtual bool setSize(size_f_t newSize) {
		ASSERT(newSize >= 0);
		if (newSize > m_sizeBuffer)
			setMaxSize(newSize);
		m_size = newSize;
		if (m_pos > m_size)
			m_pos = m_size;
		return true;
	}

	virtual bool growSize(size_f_t deltaSize) {
		return setSize(m_size+deltaSize);
	}

	virtual bool moveSize(size_f_t delataSize) {
		return setSize(m_size + delataSize);
	}

	virtual bool setMaxSize(size_f_t newSize) {
		ASSERT(newSize > m_sizeBuffer);
		// grow by 50% or at least to minNewVectorSize
		const size_f_t expoSize(m_sizeBuffer + (m_sizeBuffer>>1));
		if (newSize < expoSize)
			newSize = expoSize;
		// allocate a larger chunk of memory, copy the data and delete the old chunk
		BYTE* const tmp(m_buffer);
		m_buffer = new BYTE[(size_t)newSize];
		if (!m_buffer) {
			m_buffer = tmp;
			return false;
		}
		if (m_size > newSize) {
			m_size = newSize;
			if (m_pos > m_size)
				m_pos = m_size;
		}
		memcpy(m_buffer, tmp, (size_t)m_size);
		if (m_sizeBuffer)
			delete[] tmp;
		m_sizeBuffer = newSize;
		return true;
	}

	virtual bool growMaxSize(size_f_t deltaSize) {
		return setMaxSize(m_sizeBuffer+deltaSize);
	}

	virtual bool ensureSize(size_f_t extraSize) {
		const size_f_t newSize = m_size + extraSize;
		if (newSize > m_sizeBuffer)
			return setMaxSize(newSize);
		return true;
	}

	virtual size_f_t getPos() const {
		return m_pos;
	}		

	virtual bool setPos(size_f_t pos) {
		if (pos > m_size && !setSize(pos))
			return false;
		m_pos = pos;
		return true;
	}

	virtual bool movePos(size_f_t delataPos) {
		return setPos(m_pos + delataPos);
	}

	virtual bool isEOF() {
		return (m_pos == m_size);
	}

	virtual size_t read(void* buf, size_t len) {
		if (m_pos >= m_size)
			return 0;
		if (m_pos+(size_f_t)len > m_size)
			len = (size_t)(m_size-m_pos);
		memcpy(buf, m_buffer+m_pos, len);
		m_pos += len;
		return len;
	}

	virtual size_t write(const void* buf, size_t len) {
		const size_f_t endSize = m_pos+len;
		if (endSize > m_size && !setSize(endSize))
			return 0;
		memcpy(m_buffer+m_pos, buf,  len);
		m_pos += len;
		return len;
	}

	virtual BYTE* getData() {
		return m_buffer + m_pos;
	}

	virtual BYTE* getBuffer() {
		return m_buffer;
	}

	virtual size_t flush() {
		return 0;
	}

protected:
	BYTE*		m_buffer;		//buffer where we will store the data
	size_f_t	m_sizeBuffer;	//size of the whole buffer, 0 if no buffer or not allocated here
	size_f_t	m_size;			//size of the stored data
	size_f_t	m_pos;			//current position in the stored data

private:
	MemFile(const MemFile&);
	MemFile& operator=(const MemFile&);
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_MEMFILE_H__
