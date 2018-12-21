////////////////////////////////////////////////////////////////////
// Util.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_UTIL_H__
#define __SEACAVE_UTIL_H__


// I N C L U D E S /////////////////////////////////////////////////

#ifdef _MSC_VER
#include <io.h>
#else
#include <unistd.h>
#include <dirent.h>
#endif


// D E F I N E S ///////////////////////////////////////////////////

#define PATH_SEPARATOR _T('/')
#define PATH_SEPARATOR_STR _T("/")
#define REVERSE_PATH_SEPARATOR _T('\\')

#ifdef _MSC_VER
#define LINE_SEPARATOR_STR _T("\r\n")
#define LINE_SEPARATOR_LEN 2
#else
#define LINE_SEPARATOR_STR _T("\n")
#define LINE_SEPARATOR_LEN 1
#endif

#ifdef _MSC_VER
#define SETTINGS_PATH _T("%APPDATA%")
#else
#define SETTINGS_PATH _T("~/.config")
#endif

#define ensureUnifySlashWin		ensureUnifyReverseSlash
#define ensureUnifySlashUnix	ensureUnifySlash

#define GET_TICK()		Util::getTick()
#define GET_TIME()		Util::getTime()
#define SIMD_ENABLED	Util::ms_CPUFNC


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Manage setting/removing bit flags
template <typename TYPE>
class TFlags
{
public:
	typedef TYPE Type;

public:
	inline TFlags() : flags(0)							{ }
	inline TFlags(const TFlags& rhs) : flags(rhs.flags)	{ }
	inline TFlags(Type f) : flags(f)					{ }
	inline bool isSet(Type aFlag) const					{ return (flags & aFlag) == aFlag; }
	inline bool isSet(Type aFlag, Type nF) const		{ return (flags & (aFlag|nF)) == aFlag; }
	inline bool isSetExclusive(Type aFlag) const		{ return flags == aFlag; }
	inline bool isAnySet(Type aFlag) const				{ return (flags & aFlag) != 0; }
	inline bool isAnySet(Type aFlag, Type nF) const		{ const Type m(flags & (aFlag|nF)); return m != 0 && (m & nF) == 0; }
	inline bool isAnySetExclusive(Type aFlag) const		{ return (flags & aFlag) != 0 && (flags & ~aFlag) == 0; }
	inline void set(Type aFlag, bool bSet)				{ if (bSet) set(aFlag); else unset(aFlag); }
	inline void set(Type aFlag)							{ flags |= aFlag; }
	inline void unset(Type aFlag)						{ flags &= ~aFlag; }
	inline void flip(Type aFlag)						{ flags ^= aFlag; }
	inline void operator=(TFlags rhs)					{ flags = rhs.flags; }
	inline operator Type() const						{ return flags; }
	inline operator Type&()								{ return flags; }
protected:
	Type flags;
	#ifdef _USE_BOOST
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & flags;
	}
	#endif
};
typedef class GENERAL_API TFlags<uint32_t> Flags;
/*----------------------------------------------------------------*/


// A histogram class, that computes the distribution function (df)
// of unique sended value or iterable data inside the provided range.
// The Histogram object can keep a tally of values within a range,
// the range is arranged into some number of bins specified during
// construction.
// Jansson Consulting
// 2009-06-30, updated 2011-06-17 and 2011-08-03
// 2011-12-17 Modified by Pierre Moulon
//  - use vector array to avoid memory management
//  - add value by sequence with iterator
// 2015-04-04 Modified by cDc
//  - rewrite
//  - add GetApproximatePermille()
// Dedicated to the public domain.
template <typename TYPE>
class THistogram
{
public:
	typedef TYPE Type;

public:
	// Construct a histogram that can count within a range of values.
	// All bins of the histogram are set to zero.
	THistogram(const std::pair<Type,Type>& range, size_t bins=10) :
		Start(range.first),
		End(range.second),
		BinInterval(Type(bins)/(End-Start)),
		Freq(bins, 0),
		Overflow(0),
		Underflow(0) {}

	// Construct a histogram from a sequence of data
	template <typename DataInputIterator>
	void Add(DataInputIterator begin, DataInputIterator end) {
		for (DataInputIterator iter = begin; iter != end; ++iter)
			Add(static_cast<Type>(*iter));
	}
	// Increase the count for the bin that holds a value that is in range
	// for this histogram or the under-/overflow count if it is not in range
	void Add(const Type& x) {
		if (x < Start) {
			++Underflow;
		} else {
			const size_t i(static_cast<size_t>((x-Start)*BinInterval));
			if (i < Freq.size()) ++Freq[i];
			else ++Overflow;
		}
	}

	// Get the sum of all counts in the histogram
	inline size_t GetTotalCount() const { return std::accumulate(Freq.begin(), Freq.end(), 0); }
	// Get the overflow count
	inline size_t GetOverflow() const { return Overflow; }
	// Get the underflow count
	inline size_t GetUnderflow() const { return Underflow; }
	// Get frequencies
	inline const std::vector<size_t>& GetHist() const { return Freq; }
	// Get XbinsValue
	std::vector<Type> GetXbinsValue() const {
		const size_t NBins(Freq.size());
		std::vector<Type> vec_XbinValue(NBins);
		const Type val((End-Start)/static_cast<Type>(NBins-1));
		for (size_t i = 0; i < NBins; ++i)
			vec_XbinValue[i] = (val*static_cast<Type>(i) + Start);
		return vec_XbinValue;
	}
	// Get start
	inline Type GetStart() const { return Start; }
	// Get End
	inline Type GetEnd() const { return End; }

	// Returns the approximate permille
	Type GetApproximatePermille(float permille) const {
		ASSERT(permille >= 0.f && permille <= 1.f);
		size_t NumValues(0);
		for (size_t n: Freq)
			NumValues += n;
		size_t Num(0);
		Type UpperBound(Start);
		for (size_t i = 0; i < Freq.size(); ++i) {
			if (static_cast<float>(Num)/NumValues > permille)
				return UpperBound;
			Num += Freq[i];
			UpperBound = (static_cast<Type>(i)*End)/(Freq.size()-1)+Start;
		}
		return End;
	}

	// Text display of the histogram
	std::string ToString(const std::string& sTitle = "") const {
		std::ostringstream os;
		os.precision(3);
		os << sTitle << "\n";
		const size_t n(Freq.size());
		for (size_t i = 0; i < n; ++i)
			os << static_cast<float>(End-Start)/n*static_cast<float>(i) << "\t|\t" << Freq[i] << "\n";
		os << End << "\n";
		return os.str();
	}

protected:
	const Type Start, End, BinInterval; // min/max/step of values
	std::vector<size_t> Freq; // histogram
	size_t Overflow, Underflow; // count under/over flow
};
typedef class GENERAL_API THistogram<float> Histogram32F;
typedef class GENERAL_API THistogram<double> Histogram64F;
/*----------------------------------------------------------------*/


class GENERAL_API Util
{
public:
	static String getAppName() {
		#ifdef _MSC_VER
		TCHAR buf[MAX_PATH+1];
		GetModuleFileName(NULL, buf, MAX_PATH);
		return ensureUnifySlash(String(buf));
		#else // _MSC_VER
		LPTSTR home = getenv("HOME");
		if (home == NULL)
			return String();
		String name(String(home) + "/app");
		return ensureUnifySlash(name);
		#endif // _MSC_VER
	}

	// generate a unique name based on process ID and time
	static String getUniqueName(TCHAR dash='-')
	{
		TCHAR szDate[256];
		#ifdef _MSC_VER
		SYSTEMTIME st;
		GetLocalTime(&st);
		LPTSTR szTime = szDate+
		GetDateFormat(LOCALE_USER_DEFAULT,0,&st,_T("yy''MM''dd"),szDate,80);
		GetTimeFormat(LOCALE_USER_DEFAULT,0,&st,_T("HH''mm''ss"),szTime,80);
		#else // _MSC_VER
		const time_t t = time(NULL);
		const struct tm *tmp = localtime(&t);
		LPTSTR szTime = szDate+1+
		strftime(szDate, 80, "%y%m%d", tmp);
		strftime(szTime, 80, "%H%M%S", tmp);
		#endif // _MSC_VER
		const uint32_t ID((uint32_t(__PROCESS__) + RAND())&0x00FFFFFF);
		if (dash)
			return String::FormatString("%s%c%s%c%06X", szDate, dash, szTime, dash, ID);
		return String::FormatString("%s%s%06X", szDate, szTime, ID);
	}

	static String translateError(int aError) {
		#ifdef _MSC_VER
		LPVOID lpMsgBuf;
		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER |
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			aError,
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
			(LPTSTR) &lpMsgBuf,
			0,
			NULL
			);
		String tmp((LPCTSTR)lpMsgBuf);
		// Free the buffer.
		LocalFree(lpMsgBuf);
		String::size_type i;

		while ((i = tmp.find_last_of(LINE_SEPARATOR_STR)) != String::npos)
			tmp.erase(i, LINE_SEPARATOR_LEN);
		return tmp;
		#else // _MSC_VER
		return strerror(aError);
		#endif // _MSC_VER
	}

	static String& trimUnifySlash(String& path)
	{
		String::size_type start = 1;
		while ((start = path.find(PATH_SEPARATOR, start)) != String::npos)
			if (path[start-1] == PATH_SEPARATOR)
				path.erase(start, 1);
			else
				++start;
		return path;
	}
	static String& ensureUnifySlash(String& path)
	{
		String::size_type start = 0;
		while ((start = path.find(REVERSE_PATH_SEPARATOR, start)) != String::npos)
			path[start] = PATH_SEPARATOR;
		return trimUnifySlash(path);
	}
	static String& ensureUnifyReverseSlash(String& path)
	{
		String::size_type start = 0;
		while ((start = path.find(PATH_SEPARATOR, start)) != String::npos)
			path[start] = REVERSE_PATH_SEPARATOR;
		return path;
	}

	static String& ensureFolderSlash(String& path)
	{
		if (path.empty())
			return path;
		String::size_type nEnd = path.size()-1;
		if (path[nEnd] != PATH_SEPARATOR)
			path += PATH_SEPARATOR;
		return path;
	}

	static void ensureFolder(const String& path)
	{
		String::size_type start = 0;
		while ((start = path.find(PATH_SEPARATOR, start)) != String::npos)
			#ifdef _MSC_VER
			CreateDirectory(path.substr(0, ++start).c_str(), NULL);
			#else
			mkdir(path.substr(0, ++start).c_str(), 0755);
			#endif
	}

	static String& ensureValidPath(String& path) {
		return simplifyPath(ensureUnifySlash(strTrim(path, _T("\""))));
	}
	static String& ensureValidFolderPath(String& path) {
		return simplifyPath(ensureFolderSlash(ensureUnifySlash(strTrim(path, _T("\"")))));
	}

	static inline bool isFullPath(LPCTSTR path) {
		// returns true if local drive full path or network path
		return (path && (
			#ifdef _MSC_VER
			(path[1]==_T(':') && path[0]!=_T('\0')) ||
			#else // _MSC_VER
			path[0]==_T('/') ||
			#endif // _MSC_VER
			#ifdef UNICODE
			*reinterpret_cast<const DWORD*>(path)==0x5C005C00/*"\\\\"*/));
			#else
			*reinterpret_cast<const WORD*>(path)==0x5C5C/*"\\\\"*/));
			#endif // UNICODE
	}
	static String getFullPath(const String& str) {
		if (isFullPath(str))
			return str;
		return getCurrentFolder()+str;
	}

	static inline bool isParentFolder(LPCTSTR path, int off=0) {
		// returns true if the folder starting at the given position in path is the parent folder ".."
		if (off < 0 || path[off] != _T('.'))
			return false;
		if (off > 0 && path[off-1] != PATH_SEPARATOR)
			return false;
		if (path[off+1] != _T('.'))
			return false;
		return path[off+2] == _T('\0') || path[off+2] == PATH_SEPARATOR;
	}

	static String getHomeFolder();
	static String getApplicationFolder();
	static String getCurrentFolder();
	static String getProcessFolder() {
		return getFilePath(getAppName());
	}

	static String ensureUnitPath(const String& path)
	{
		if (path.find(_T(" ")) == String::npos)
			return path;
		return String(_T("\"")+path+_T("\""));
	}

	static String& simplifyPath(String& path) {
		// compress path by removing all "./" and "folder/../" occurrences
		// (if path only, it should end in path-separator)
		{
		// removes all "./" occurrences
		String::size_type i(0);
		while ((i = path.find(_T(".") PATH_SEPARATOR_STR, i)) != String::npos) {
			if (i > 0 && path[i-1] != PATH_SEPARATOR)
				i += 2;
			else
				path.erase(i, 2);
		}}
		{
		// removes all "folder/../" occurrences
		String::size_type i(0);
		while ((i = path.find(_T("..") PATH_SEPARATOR_STR, i)) != String::npos) {
			if (i > 1 && path[i-1] == PATH_SEPARATOR) {
				String::size_type prev = path.rfind(PATH_SEPARATOR, i-2);
				if (prev == String::npos) prev = 0; else ++prev;
				if (!isParentFolder(path, (int)prev)) {
					path.erase(prev, i+3-prev);
					i = prev;
					continue;
				}
			}
			i += 3;
		}}
		return path;
	}
	static String getSimplifiedPath(String path) {
		return simplifyPath(path);
	}

	static String getRelativePath(const String& currentPath, const String& targetPath) {
		// returns the path to the target relative to the current path;
		// both current and target paths must be full paths;
		// current path is assumed to be a folder, while target path can be also a file
		CLISTDEF2(String) currentPathValues, targetPathValues;
		Util::strSplit(currentPath, PATH_SEPARATOR, currentPathValues);
		if (currentPathValues.back().empty())
			currentPathValues.pop_back();
		Util::strSplit(targetPath, PATH_SEPARATOR, targetPathValues);
		size_t idxCurrentPath(0), idxTargetPath(0);
		while (
			idxCurrentPath < currentPathValues.size() &&
			idxTargetPath < targetPathValues.size() &&
			#ifdef _MSC_VER
			_tcsicmp(currentPathValues[idxCurrentPath], targetPathValues[idxTargetPath]) == 0
			#else
			_tcscmp(currentPathValues[idxCurrentPath], targetPathValues[idxTargetPath]) == 0
			#endif
		)
			++idxCurrentPath, ++idxTargetPath;
		if (idxCurrentPath == 0)
			return targetPath;
		String relativePath;
		relativePath.reserve(targetPath.size());
		while (idxCurrentPath < currentPathValues.size()) {
			relativePath += _T("..") PATH_SEPARATOR_STR;
			++idxCurrentPath;
		}
		const size_t idxFirstTarget(idxTargetPath);
		while (idxTargetPath < targetPathValues.size()) {
			if (idxTargetPath > idxFirstTarget)
				relativePath += PATH_SEPARATOR;
			relativePath += targetPathValues[idxTargetPath++];
		}
		return relativePath;
	}

	static String& getCommonPath(String& commonPath, const String& path) {
		// returns the path shared by the given paths
		#ifdef _MSC_VER
		while (_tcsnicmp(commonPath, path, commonPath.length()) != 0) {
		#else
		while (_tcsncmp(commonPath, path, commonPath.length()) != 0) {
		#endif
			commonPath.pop_back();
			commonPath = getFilePath(commonPath);
		}
		return commonPath;
	}
	static String getCommonPath(const String* arrPaths, size_t numPaths) {
		// returns the path shared by all given paths
		ASSERT(numPaths > 0);
		String commonPath(arrPaths[0]);
		for (size_t i=1; !commonPath.empty() && i<numPaths; ++i)
			getCommonPath(commonPath, arrPaths[i]);
		return commonPath;
	}

	static String getFilePath(const String& path) {
		const String::size_type i = path.rfind(PATH_SEPARATOR);
		return (i != String::npos) ? path.substr(0, i+1) : String();
	}
	static String getFileFullName(const String& path) {
		const String::size_type i = path.rfind('.');
		return (i != String::npos) ? String(path.substr(0, i)) : path;
	}
	static String getFileNameExt(const String& path) {
		const String::size_type i = path.rfind(PATH_SEPARATOR);
		if (i != String::npos)
			return path.substr(i+1);
		return path;
	}
	static String getFileName(const String& path) {
		String::size_type i = path.rfind(PATH_SEPARATOR);
		if (i == String::npos) i = 0; else ++i;
		String::size_type j = path.rfind('.');
		if (j == String::npos) j = path.length();
		return path.substr(i, j-i);
	}
	static String getFileExt(const String& path) {
		const String::size_type i = path.rfind('.');
		return (i != String::npos) ? path.substr(i) : String();
	}
	static String getLastDir(const String& path) {
		const String::size_type i = path.rfind(PATH_SEPARATOR);
		if (i == String::npos) return String();
		const String::size_type j = path.rfind(PATH_SEPARATOR, i-1);
		if (j != String::npos)
			return path.substr(j+1, j-i-1);
		return path;
	}
	static String insertBeforeFileExt(const String& path, const String& extra) {
		const String::size_type i = path.rfind('.');
		return (i != String::npos) ? String(path).insert(i, extra) : path+extra;
	}
	static String insertAfterFilePath(const String& path, const String& extra) {
		const String::size_type i = path.rfind(PATH_SEPARATOR);
		return (i != String::npos) ? String(path).insert(i+1, extra) : extra+path;
	}
	static int compareFileName(const String& path1, const String& path2) {
		#ifdef _MSC_VER
		return _tcsicmp(path1, path2);
		#else // _MSC_VER
		return _tcscmp(path1, path2);
		#endif // _MSC_VER
	}

	static String getValue(const String& str, const String& label) {
		String::size_type pos = str.find(label);
		if (pos == String::npos)
			return String();
		pos = str.find(_T("="), pos);
		if (pos == String::npos)
			return String();
		String::size_type end = str.find_first_of(_T(LINE_SEPARATOR_STR), ++pos);
		return str.substr(pos, end-pos);
	}

	// remove all instances of a given sequence of characters from the beginning and end of the given string
	static String& strTrim(String& str, const String& strTrim) {
		if (str.empty())
			return str;
		while (str.substr(0, strTrim.size()) == strTrim)
			str.erase(0, strTrim.size());
		while (str.substr(str.size()-strTrim.size(), strTrim.size()) == strTrim)
			str.erase(str.size()-strTrim.size(), strTrim.size());
		return str;
	}
	// split an input string with a delimiter and fill a string vector
	static void strSplit(const String& str, TCHAR delim, CLISTDEF2(String)& values) {
		values.Empty();
		String::size_type start(0), end(0);
		while (end != String::npos) {
			end = str.find(delim, start);
			values.AddConstruct(str.substr(start, end-start));
			start = end + 1;
		}
	}
	static void strSplit(const String& str, const String& delim, CLISTDEF2(String)& values) {
		ASSERT(!delim.empty());
		values.Empty();
		String::size_type start(0), end(0);
		while (end != String::npos) {
			end = str.find(delim, start);
			values.AddConstruct(str.substr(start, end-start));
			start = end + delim.size();
		}
	}

	static String getShortTimeString() {
		char buf[8];
		time_t _tt = time(NULL);
		tm* _tm = localtime(&_tt);
		if (_tm == NULL) {
			strcpy(buf, "xx:xx");
		} else {
			strftime(buf, 8, "%H:%M", _tm);
		}
		return buf;
	}

	static String getTimeString() {
		char buf[64];
		time_t _tt;
		time(&_tt);
		tm* _tm = localtime(&_tt);
		if (_tm == NULL) {
			strcpy(buf, "xx:xx:xx");
		} else {
			strftime(buf, 64, "%X", _tm);
		}
		return buf;
	}

	static String formatBytes(int64_t aBytes) {
		if (aBytes < (int64_t)1024) {
			return String::FormatString("%dB", (uint32_t)aBytes&0xffffffff);
		} else if (aBytes < (int64_t)1024*1024) {
			return String::FormatString("%.02fKB", (double)aBytes/(1024.0));
		} else if (aBytes < (int64_t)1024*1024*1024) {
			return String::FormatString("%.02fMB", (double)aBytes/(1024.0*1024.0));
		} else if (aBytes < (int64_t)1024*1024*1024*1024) {
			return String::FormatString("%.02fGB", (double)aBytes/(1024.0*1024.0*1024.0));
		} else {
			return String::FormatString("%.02fTB", (double)aBytes/(1024.0*1024.0*1024.0*1024.0));
		}
	}

	// format given time in milliseconds to higher units
	static String formatTime(int64_t sTime, uint32_t nAproximate = 0) {
		char buf[128];
		uint32_t len = 0;
		uint32_t nrNumbers = 0;

		uint32_t rez = (uint32_t)(sTime / ((int64_t)24*3600*1000));
		if (rez) {
			++nrNumbers;
			len += _stprintf(buf+len, "%ud", rez);
		}
		if (nAproximate > 3 && nrNumbers > 0)
			return buf;
		rez = (uint32_t)((sTime%((int64_t)24*3600*1000)) / (3600*1000));
		if (rez) {
			++nrNumbers;
			len += _stprintf(buf+len, "%uh", rez);
		}
		if (nAproximate > 2 && nrNumbers > 0)
			return buf;
		rez = (uint32_t)((sTime%((int64_t)3600*1000)) / (60*1000));
		if (rez) {
			++nrNumbers;
			len += _stprintf(buf+len, "%um", rez);
		}
		if (nAproximate > 1 && nrNumbers > 0)
			return buf;
		rez = (uint32_t)((sTime%((int64_t)60*1000)) / (1*1000));
		if (rez) {
			++nrNumbers;
			len += _stprintf(buf+len, "%us", rez);
		}
		if (nAproximate > 0 && nrNumbers > 0)
			return buf;
		rez = (uint32_t)(sTime%((int64_t)1*1000));
		if (rez || !nrNumbers)
			len += _stprintf(buf+len, "%ums", rez);

		return String(buf, len);
	}

	static String toString(const wchar_t* wsz) {
		if (wsz == NULL)
			return String();
		#if 1
		const std::wstring ws(wsz);
		return std::string(ws.cbegin(), ws.cend());
		#elif 1
		std::mbstate_t state = std::mbstate_t();
		const size_t len(std::wcsrtombs(NULL, &wsz, 0, &state));
		if (len == static_cast<std::size_t>(-1))
			return String();
		std::vector<char> mbstr(len+1);
		if (std::wcsrtombs(&mbstr[0], &wsz, mbstr.size(), &state) == static_cast<std::size_t>(-1))
			return String();
		return String(&mbstr[0]);
		#elif 1
		const std::wstring ws(wsz);
		const std::locale locale("");
		typedef std::codecvt<wchar_t, char, std::mbstate_t> converter_type;
		const converter_type& converter = std::use_facet<converter_type>(locale);
		std::vector<char> to(ws.length() * converter.max_length());
		std::mbstate_t state;
		const wchar_t* from_next;
		char* to_next;
		if (converter.out(state, ws.data(), ws.data() + ws.length(), from_next, &to[0], &to[0] + to.size(), to_next) != converter_type::ok)
			return String();
		return std::string(&to[0], to_next);
		#else
		typedef std::codecvt_utf8<wchar_t> convert_typeX;
		std::wstring_convert<convert_typeX, wchar_t> converterX;
		return converterX.to_bytes(wstr);
		#endif
	}

	static int64_t toInt64(LPCTSTR aString) {
		#ifdef _MSC_VER
		return _atoi64(aString);
		#else
		return atoll(aString);
		#endif
	}
	static int toInt(LPCTSTR aString) {
		return atoi(aString);
	}
	static uint32_t toUInt32Hex(LPCTSTR aString) {
		uint32_t val;
		sscanf(aString, "%x", &val);
		return val;
	}
	static uint32_t toUInt32(LPCTSTR aString) {
		return (uint32_t)atoi(aString);
	}
	static double toDouble(LPCTSTR aString) {
		return atof(aString);
	}
	static float toFloat(LPCTSTR aString) {
		return (float)atof(aString);
	}

	static time_t getTime() {
		return (time_t)time(NULL);
	}

	static uint32_t getTick() {
		#ifdef _MSC_VER
		return GetTickCount();
		#else
		timeval tv;
		gettimeofday(&tv, NULL);
		return (uint32_t)(tv.tv_sec * 1000 ) + (tv.tv_usec / 1000);
		#endif
	}


	/**
	 * IPRT - CRC64.
	 *
	 * The method to compute the CRC64 is referred to as CRC-64-ISO:
	 *     http://en.wikipedia.org/wiki/Cyclic_redundancy_check
	 * The generator polynomial is x^64 + x^4 + x^3 + x + 1.
	 *     Reverse polynom: 0xd800000000000000ULL
	 *     
	 * As in: http://www.virtualbox.org/svn/vbox/trunk/src/VBox/Runtime/common/checksum/crc64.cpp
	 */

	/**
	 * Calculate CRC64 for a memory block.
	 *
	 * @returns CRC64 for the memory block.
	 * @param   pv      Pointer to the memory block.
	 * @param   cb      Size of the memory block in bytes.
	 */
	static uint64_t CRC64(const void *pv, size_t cb);

	/**
	 * Start a multiblock CRC64 calculation.
	 *
	 * @returns Start CRC64.
	 */
	static uint64_t CRC64Start() {
		return 0ULL;
	}
	/**
	 * Processes a multiblock of a CRC64 calculation.
	 *
	 * @returns Intermediate CRC64 value.
	 * @param   uCRC64  Current CRC64 intermediate value.
	 * @param   pv      The data block to process.
	 * @param   cb      The size of the data block in bytes.
	 */
	static uint64_t CRC64Process(uint64_t uCRC64, const void *pv, size_t cb);
	/**
	 * Complete a multiblock CRC64 calculation.
	 *
	 * @returns CRC64 value.
	 * @param   uCRC64  Current CRC64 intermediate value.
	 */
	static uint64_t CRC64Finish(uint64_t uCRC64) {
		return uCRC64;
	}


	static String	GetCPUInfo();
	static String	GetRAMInfo();
	static String	GetOSInfo();
	enum CPUFNC {NA=0, SSE, AVX};
	static const Flags ms_CPUFNC;

	static void		LogBuild();
	static void		LogMemoryInfo();

	static LPSTR* CommandLineToArgvA(LPCSTR CmdLine, size_t& _argc);
	static String CommandLineToString(size_t argc, LPCTSTR* argv) {
		String strCmdLine;
		for (size_t i=1; i<argc; ++i)
			strCmdLine += _T(" ") + String(argv[i]);
		return strCmdLine;
	}

	struct Progress {
		const String msg; // custom message header (ex: "Processed images")
		const size_t total; // total number of jobs to be processed
		const Timer::Type slp; // how often to display the progress (in ms)
		const Timer::SysType start; // time when the work started
		Timer::Type lastElapsed; // time when the last progress was displayed
		size_t lastMsgLen; // how many characters had the last message
		volatile size_t processed; // number of jobs already processed
		CriticalSection cs; // multi-threading safety only for the incremental operator

		Progress(const String& _msg, size_t _total, Timer::Type _slp=100/*ms*/)
			: msg(_msg), total(_total), slp(_slp), start(Timer::GetSysTime()), lastElapsed(0), lastMsgLen(0), processed(0) {}
		~Progress() { if (processed) close(); }

		void operator++ () {
			Thread::safeInc((Thread::safe_t&)processed);
			if (cs.TryEnter()) {
				process();
				cs.Leave();
			}
		}
		void display(size_t done) {
			processed = done;
			process();
		}
		void displayRemaining(size_t remaining) {
			processed = total-remaining;
			process();
		}
		void process() {
			// make sure we don't print the progress too often
			const Timer::Type elapsed(Timer::SysTime2TimeMs(Timer::GetSysTime()-start));
			if (elapsed-lastElapsed < slp)
				return;
			lastElapsed = elapsed;
			// compute percentage, elapsed and ETA
			const size_t done(processed);
			const float percentage((float)done/(float)total);
			const Timer::Type remaining(percentage<0.01f && (done<10 || elapsed<10*1000) ? Timer::Type(0) : elapsed/percentage - elapsed);
			// display progress
			print(String::FormatString(_T("%s %u (%.2f%%, %s, ETA %s)..."), msg.c_str(), done, percentage*100.f, formatTime((int64_t)elapsed,1).c_str(), formatTime((int64_t)remaining,2).c_str()));
		}
		void close() {
			// make sure we print the complete progress
			const Timer::Type elapsed(Timer::SysTime2TimeMs(Timer::GetSysTime()-start));
			// display progress
			print(String::FormatString(_T("%s %u (100%%, %s)"), msg.c_str(), total, formatTime((int64_t)elapsed).c_str()));
			std::cout << _T("\n");
			processed = 0;
		}
		void print(const String& line) {
			// print given line and make sure the last line is erased
			const size_t msgLen = line.length();
			std::cout << _T("\r") << line;
			if (lastMsgLen > msgLen)
				std::cout << String(lastMsgLen-msgLen, _T(' '));
			std::cout << std::flush;
			lastMsgLen = msgLen;
		}
	};
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_UTIL_H__
