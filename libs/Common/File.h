////////////////////////////////////////////////////////////////////
// File.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_FILE_H__
#define __SEACAVE_FILE_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Streams.h"

#ifdef _MSC_VER
#include <io.h>
#else
#include <unistd.h>
#define _taccess access
#endif


// D E F I N E S ///////////////////////////////////////////////////

/* size of the stored file size variable */
#ifdef LARGEFILESIZE
#define FILESIZE	size_f_t
#else
#define FILESIZE	size_t
#endif


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class GENERAL_API File : public IOStream {
public:
	typedef struct FILEINFO_TYPE {
		String path;
		FILESIZE size;
		DWORD attrib;
	} FILEINFO;
	typedef cList<FILEINFO> FileInfoArr;

	typedef enum FMCREATE_TYPE {
		OPEN = 0x01,
		CREATE = 0x02,
		TRUNCATE = 0x04
	} FMCREATE;

	typedef enum FMFALGS_TYPE {
		NOBUFFER = 0x01,
		RANDOM = 0x02,
		SEQUENTIAL = 0x03
	} FMFALGS;

#ifdef _MSC_VER
	typedef enum FMACCESS_TYPE {
		READ = GENERIC_READ,
		WRITE = GENERIC_WRITE,
		RW = READ | WRITE
	} FMACCESS;

	typedef enum FMCHECKACCESS_TYPE {
		CA_EXIST	= 0, // existence
		CA_WRITE	= 2, // write
		CA_READ		= 4, // read
		CA_RW		= CA_READ | CA_WRITE
	} FMCHECKACCESS;

	File() : h(INVALID_HANDLE_VALUE) {
		#ifndef _RELEASE
		breakRead = -1;
		breakWrite = -1;
		#endif
	}

	File(LPCTSTR aFileName, int access, int mode, int flags=0) : h(INVALID_HANDLE_VALUE) {
		#ifndef _RELEASE
		breakRead = -1;
		breakWrite = -1;
		#endif
		open(aFileName, access, mode, flags);
	}

	/**
	 * Open the file specified.
	 * If there are errors, h is set to INVALID_HANDLE_VALUE.
	 * Use isOpen() to check.
	 */
	virtual void open(LPCTSTR aFileName, int access, int mode, int flags=0) {
		ASSERT(access == WRITE || access == READ || access == (READ | WRITE));

		close();

		DWORD m = 0;
		if (mode & OPEN) {
			if (mode & CREATE) {
				m = (mode & TRUNCATE) ? CREATE_ALWAYS : OPEN_ALWAYS;
			} else {
				m = (mode & TRUNCATE) ? TRUNCATE_EXISTING : OPEN_EXISTING;
			}
		} else {
			ASSERT(mode & CREATE);
			m = (mode & TRUNCATE) ? CREATE_ALWAYS : CREATE_NEW;
		}

		DWORD f = 0;
		if (flags & NOBUFFER)
			f |= FILE_FLAG_NO_BUFFERING;
		if (flags & RANDOM)
			f |= FILE_FLAG_RANDOM_ACCESS;
		if (flags & SEQUENTIAL)
			f |= FILE_FLAG_SEQUENTIAL_SCAN;

		h = ::CreateFile(aFileName, access, FILE_SHARE_READ, NULL, m, f, NULL);
	}

	bool isOpen() { return h != INVALID_HANDLE_VALUE; };

	virtual void close() {
		if (isOpen()) {
			FlushFileBuffers(h);
			CloseHandle(h);
			h = INVALID_HANDLE_VALUE;
		}
	}

	uint32_t getLastModified() {
		FILETIME f = {0};
		::GetFileTime(h, NULL, NULL, &f);
		return convertTime(&f);
	}

	static uint32_t convertTime(FILETIME* f) {
		SYSTEMTIME s = { 1970, 1, 0, 1, 0, 0, 0, 0 };
		FILETIME f2 = {0};
		if (::SystemTimeToFileTime(&s, &f2)) {
			uint64_t* a = (uint64_t*)f;
			uint64_t* b = (uint64_t*)&f2;
			*a -= *b;
			*a /= (1000LL*1000LL*1000LL/100LL);		// 100ns > s
			return (uint32_t)*a;
		}
		return 0;
	}

	virtual size_f_t getSize() const {
		DWORD x;
		DWORD l = ::GetFileSize(h, &x);
		if ((l == INVALID_FILE_SIZE) && (GetLastError() != NO_ERROR))
			return SIZE_NA;
		return (size_f_t)l | ((size_f_t)x)<<32;
	}

	virtual bool setSize(size_f_t newSize) {
		const size_f_t pos = getPos();
		if (pos == SIZE_NA)
			return false;
		if (!setPos(newSize))
			return false;
		if (!setEOF())
			return false;
		if (!setPos(pos))
			return false;
		return true;
	}

	virtual size_f_t getPos() const {
		LONG x = 0;
		const DWORD l = ::SetFilePointer(h, 0, &x, FILE_CURRENT);
		if (l == INVALID_SET_FILE_POINTER)
			return SIZE_NA;
		return (size_f_t)l | ((size_f_t)x)<<32;
	}

	virtual bool setPos(size_f_t pos) {
		LONG x = (LONG) (pos>>32);
		return (::SetFilePointer(h, (DWORD)(pos & 0xffffffff), &x, FILE_BEGIN) != INVALID_SET_FILE_POINTER);
	}

	virtual bool setEndPos(size_f_t pos) {
		LONG x = (LONG) (pos>>32);
		return (::SetFilePointer(h, (DWORD)(pos & 0xffffffff), &x, FILE_END) != INVALID_SET_FILE_POINTER);
	}

	virtual bool movePos(size_f_t pos) {
		LONG x = (LONG) (pos>>32);
		return (::SetFilePointer(h, (DWORD)(pos & 0xffffffff), &x, FILE_CURRENT) != INVALID_SET_FILE_POINTER);
	}

	virtual size_t read(void* buf, size_t len) {
		#ifndef _RELEASE
		if (breakRead != (size_t)(-1)) {
			if (breakRead <= len) {
				ASSERT("FILE::read() break" == NULL);
				breakRead = -1;
			} else {
				breakRead -= len;
			}
		}
		#endif
		DWORD x;
		if (!::ReadFile(h, buf, (DWORD)len, &x, NULL))
			return STREAM_ERROR;
		return x;
	}

	virtual size_t write(const void* buf, size_t len) {
		#ifndef _RELEASE
		if (breakWrite != (size_t)(-1)) {
			if (breakWrite <= len) {
				ASSERT("FILE::write() break" == NULL);
				breakWrite = -1;
			} else {
				breakWrite -= len;
			}
		}
		#endif
		DWORD x;
		if (!::WriteFile(h, buf, (DWORD)len, &x, NULL))
			return STREAM_ERROR;
		ASSERT(x == len);
		return x;
	}
	virtual bool setEOF() {
		ASSERT(isOpen());
		return (SetEndOfFile(h) != FALSE);
	}

	virtual size_t flush() {
		if (isOpen())
			return 0;
		if (!FlushFileBuffers(h))
			return STREAM_ERROR;
		return 0;
	}

	virtual bool getInfo(BY_HANDLE_FILE_INFORMATION* fileInfo) {
		return (GetFileInformationByHandle(h, fileInfo) != FALSE);
	}

	static uint32_t getAttrib(LPCTSTR aFileName) {
		return GetFileAttributes(aFileName);
	}

	static bool setAttrib(LPCTSTR aFileName, uint32_t attribs) {
		return (SetFileAttributes(aFileName, attribs) != FALSE);
	}

	static void deleteFile(LPCTSTR aFileName) { ::DeleteFile(aFileName); }
	static bool renameFile(LPCTSTR source, LPCTSTR target) {
		if (!::MoveFile(source, target)) {
			// Can't move, try copy/delete...
			if (!::CopyFile(source, target, FALSE))
				return false;
			deleteFile(source);
		}
		return true;
	}
	static bool copyFile(LPCTSTR source, LPCTSTR target) { return ::CopyFile(source, target, FALSE) == TRUE; }

	static size_f_t getSize(LPCTSTR aFileName) {
		const HANDLE fh = ::CreateFile(aFileName, FILE_READ_ATTRIBUTES, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_FLAG_NO_BUFFERING, NULL);
		if (fh == INVALID_HANDLE_VALUE)
			return SIZE_NA;
		DWORD x;
		DWORD l = ::GetFileSize(fh, &x);
		CloseHandle(fh);
		if ((l == INVALID_FILE_SIZE) && (GetLastError() != NO_ERROR))
			return SIZE_NA;
		return (((size_f_t)l) | (((size_f_t)x)<<32));
	}


	static size_f_t findFiles(const String& _strPath, const String& strMask, bool bProcessSubdir, FileInfoArr& arrFiles)
	{	// List all the files.
		WIN32_FIND_DATA fd;
		HANDLE hFind;
		size_f_t totalSize = 0;
		String strPath(_strPath);
		Util::ensureFolderSlash(strPath);
		//Find all the files in this folder.
		hFind = FindFirstFile((strPath + strMask).c_str(), &fd);
		if (hFind != INVALID_HANDLE_VALUE)
		{
			do {
				// this is a file that can be used
				if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
					continue;
				// Store the file name.
				FILEINFO& fileInfo = arrFiles.AddEmpty();
				fileInfo.path = strPath + fd.cFileName;
				#ifdef LARGEFILESIZE
				fileInfo.size = (((size_f_t)fd.nFileSizeLow) | (((size_f_t)fd.nFileSizeHigh)<<32));
				#else
				fileInfo.size = fd.nFileSizeLow;
				#endif
				fileInfo.attrib = fd.dwFileAttributes;
				totalSize += fileInfo.size;
			}
			while (FindNextFile(hFind, &fd));
			FindClose(hFind);
		}
		//Process the subfolders also...
		if (!bProcessSubdir)
			return totalSize;
		hFind = FindFirstFile((strPath + '*').c_str(), &fd);
		if (hFind != INVALID_HANDLE_VALUE)
		{
			do {
				// if SUBDIR then process that too
				if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
					continue;
				if (!_tcscmp(fd.cFileName, _T(".")))
					continue;
				if (!_tcscmp(fd.cFileName, _T("..")))
					continue;
				// Processe all subfolders recursively
				totalSize += findFiles(strPath + fd.cFileName + PATH_SEPARATOR, strMask, true, arrFiles);
			}
			while (FindNextFile(hFind, &fd));
			FindClose(hFind);
		}
		return totalSize;
	}

#else // _MSC_VER

	typedef enum FMACCESS_TYPE {
		READ = 0x01,
		WRITE = 0x02,
		RW = READ | WRITE,
	} FMACCESS;

	typedef enum FMCHECKACCESS_TYPE {
		CA_EXIST	= F_OK, // existence
		CA_WRITE	= W_OK, // write
		CA_READ		= R_OK, // read
		CA_RW		= R_OK | W_OK,
		CA_EXEC		= X_OK, // execute
	} FMCHECKACCESS;

	File() : h(-1) {
		#ifndef _RELEASE
		breakRead = -1;
		breakWrite = -1;
		#endif
	}

	File(LPCTSTR aFileName, int access, int mode, int flags=0) : h(-1) {
		#ifndef _RELEASE
		breakRead = -1;
		breakWrite = -1;
		#endif
		open(aFileName, access, mode, flags);
	}

	/**
	 * Open the file specified.
	 * If there are errors, h is set to -1.
	 * Use isOpen() to check.
	 */
	virtual void open(LPCTSTR aFileName, int access, int mode, int flags=0) {
		ASSERT(access == WRITE || access == READ || access == (READ | WRITE));

		close();

		int m = 0;
		if (access == READ)
			m |= O_RDONLY;
		else if (access == WRITE)
			m |= O_WRONLY;
		else
			m |= O_RDWR;

		if (mode & CREATE)
			m |= O_CREAT;
		if (mode & TRUNCATE)
			m |= O_TRUNC;

		#ifndef __APPLE__
		if (flags & NOBUFFER)
			m |= O_DIRECT;
		#endif
		h = ::open(aFileName, m, S_IRUSR | S_IWUSR);
	}

	bool isOpen() { return h != -1; };

	virtual void close() {
		if (h != -1) {
			::close(h);
			h = -1;
		}
	}

	uint32_t getLastModified() {
		struct stat s;
		if (::fstat(h, &s) == -1)
			return 0;

		return (uint32_t)s.st_mtime;
	}

	virtual size_f_t getSize() const {
		struct stat s;
		if (::fstat(h, &s) == -1)
			return SIZE_NA;

		return (size_f_t)s.st_size;
	}

	virtual size_f_t getPos() const {
		return (size_f_t) lseek(h, 0, SEEK_CUR);
	}

	virtual bool setPos(size_f_t pos) { return lseek(h, (off_t)pos, SEEK_SET) != (off_t)-1; };
	virtual void setEndPos(size_f_t pos) { lseek(h, (off_t)pos, SEEK_END); };
	virtual void movePos(size_f_t pos) { lseek(h, (off_t)pos, SEEK_CUR); };

	virtual size_t read(void* buf, size_t len) {
		#ifndef _RELEASE
		if (breakRead != (size_t)(-1)) {
			if (breakRead <= len) {
				ASSERT("FILE::read() break" == NULL);
				breakRead = -1;
			} else {
				breakRead -= len;
			}
		}
		#endif
		ssize_t x = ::read(h, buf, len);
		if (x == -1)
			return STREAM_ERROR;
		return (size_t)x;
	}

	virtual size_t write(const void* buf, size_t len) {
		#ifndef _RELEASE
		if (breakWrite != (size_t)(-1)) {
			if (breakWrite <= len) {
				ASSERT("FILE::write() break" == NULL);
				breakWrite = -1;
			} else {
				breakWrite -= len;
			}
		}
		#endif
		ssize_t x = ::write(h, buf, len);
		if (x == -1)
			return STREAM_ERROR;
		if (x < (ssize_t)len)
			return STREAM_ERROR;
		return x;
	}

	virtual bool setEOF() {
		return (ftruncate(h, (off_t)getPos()) != -1);
	}
	virtual bool setSize(size_f_t newSize) {
		return (ftruncate(h, (off_t)newSize) != -1);
	}

	virtual size_t flush() {
		return fsync(h);
	}

	static void deleteFile(LPCTSTR aFileName) { ::remove(aFileName); }
	static bool renameFile(LPCTSTR source, LPCTSTR target) { return ::rename(source, target) == 0; }
	static bool copyFile(LPCTSTR source, LPCTSTR target) {
		std::ifstream src(source, std::ios::binary);
		if (!src.is_open())
			return false;
		std::ofstream dst(target, std::ios::binary);
		if (!dst.is_open())
			return false;
		dst << src.rdbuf();
		return true;
	}

	static size_f_t getSize(LPCTSTR aFileName) {
		struct stat s;
		if (stat(aFileName, &s) == -1)
			return SIZE_NA;
		return s.st_size;
	}

#endif // _MSC_VER

	static bool access(LPCTSTR aFileName, int mode=CA_EXIST) { return ::_taccess(aFileName, mode) == 0; }

	template <class VECTOR>
	inline size_t write(const VECTOR& arr) {
		const typename VECTOR::IDX nSize(arr.GetSize());
		size_t nBytes(write(&nSize, sizeof(typename VECTOR::IDX)));
		nBytes += write(arr.GetData(), arr.GetDataSize());
		return nBytes;
	}

	template <class VECTOR>
	inline size_t read(VECTOR& arr) {
		typename VECTOR::IDX nSize;
		size_t nBytes(read(&nSize, sizeof(typename VECTOR::IDX)));
		arr.Resize(nSize);
		nBytes += read(arr.GetData(), arr.GetDataSize());
		return nBytes;
	}

	virtual ~File() {
		File::close();
	}

	static LPCSTR getClassType() { return "File"; }
	virtual LPCSTR getClassName() const { return File::getClassType(); }

protected:
	#ifdef _MSC_VER
	HANDLE h;
	#else
	int h;
	#endif

public:
	#ifndef _RELEASE
	size_t breakRead;
	size_t breakWrite;
	#endif

private:
	File(const File&);
	File& operator=(const File&);
};
typedef File* LPFILE;
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_FILE_H__
