////////////////////////////////////////////////////////////////////
// Strings.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_STRING_H__
#define __SEACAVE_STRING_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Streams.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

/// String class: enhanced std::string
class GENERAL_API String : public std::string
{
public:
	inline String() {}
	inline String(LPCTSTR sz) : std::string(sz) {}
	inline String(const std::string& str) : std::string(str) {}
	inline String(size_t n, value_type v) : std::string(n, v) {}
	inline String(LPCTSTR sz, size_t count) : std::string(sz, count) {}
	inline String(LPCTSTR sz, size_t offset, size_t count) : std::string(sz, offset, count) {}

	inline void Release() { return clear(); }
	inline bool IsEmpty() const { return empty(); }

	inline operator LPCTSTR() const { return c_str(); }

	inline void Format(LPCTSTR szFormat, ...) {
		va_list args;
		va_start(args, szFormat);
		TCHAR szBuffer[2048];
		if (_vsntprintf(szBuffer, 2047, szFormat, args) == -1) {
			*this = FormatStringSafe(szFormat, args);
			va_end(args);
		} else {
			va_end(args);
			szBuffer[2047] = 0;
			*this = szBuffer;
		}
	}
	static String FormatString(LPCTSTR szFormat, ...) {
		va_list args;
		va_start(args, szFormat);
		TCHAR szBuffer[2048];
		if (_vsntprintf(szBuffer, 2047, szFormat, args) == -1) {
			const String str = FormatStringSafe(szFormat, args);
			va_end(args);
			return str;
		}
		va_end(args);
		szBuffer[2047] = 0;
		return szBuffer;
	}
	static String FormatStringSafe(LPCTSTR szFormat, va_list args) {
		#ifdef _MSC_VER
		const size_t count = (size_t)_vsctprintf(szFormat, args)+1;
		#else
		const size_t count = (size_t)_vsntprintf(NULL, 0, szFormat, args)+1;
		#endif
		TCHAR* szBuffer = new TCHAR[count];
		_vsntprintf(szBuffer, count, szFormat, args);
		String str(szBuffer);
		delete[] szBuffer;
		return str;
	}

	inline void ToUpper(String& out) const {
		out.resize(size());
		std::transform(begin(), end(), out.begin(), ::towupper);
	}
	inline String ToUpper() const {
		String str;
		ToUpper(str);
		return str;
	}

	inline void ToLower(String& out) const {
		out.resize(size());
		std::transform(begin(), end(), out.begin(), ::towlower);
	}
	inline String ToLower() const {
		String str;
		ToLower(str);
		return str;
	}

	inline void	Save(OSTREAM& oStream) const {
		const WORD nSize = (WORD)size();
		oStream.write(&nSize, sizeof(WORD));
		oStream.write(c_str(), nSize);
	}
	inline void	Load(ISTREAM& oStream) {
		WORD nSize;
		oStream.read(&nSize, sizeof(WORD));
		if (nSize == 0) {
			clear();
			return;
		}
		char* pBuffer = new char[nSize];
		oStream.read(pBuffer, nSize);
		assign(pBuffer, nSize);
		delete[] pBuffer;
	}

	template <class T>
	static String ToString(const T& val) {
		std::ostringstream os;
		os << val;
		return os.str();
	}
	template <class T>
	static String ToStringHex(const T& val) {
		std::ostringstream os;
		os << std::hex << val;
		return os.str();
	}

	template <class T>
	static void FromString(const String& str, T& val) {
		std::istringstream is(str);
		is >> val;
	}
	template <class T>
	static T FromString(const String& str, const T& def) {
		T val(def);
		FromString(str, val);
		return val;
	}
	template <class T>
	static T FromString(const String& str) {
		T val;
		FromString(str, val);
		return val;
	}
	template <class T>
	inline void From(T& val) const {
		FromString(*this, val);
	}
	template <class T>
	inline T From(const T& def) const {
		T val(def);
		From(val);
		return val;
	}
	template <class T>
	inline T From() const {
		T val;
		From(val);
		return val;
	}

	static int CompareAlphabetically(const void* elem, const void* key) { return _tcscmp(((const String*)elem)->c_str(), ((const String*)key)->c_str()); }
	static int CompareAlphabeticallyInv(const void* elem, const void* key) { return _tcscmp(((const String*)key)->c_str(), ((const String*)elem)->c_str()); }

#ifdef _USE_BOOST
protected:
	// implement BOOST serialization
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		ar & boost::serialization::base_object<std::string>(*this);
	}
#endif
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE


namespace std {
//namespace tr1 {
// Specializations for unordered containers
template <> struct hash<SEACAVE::String> : public hash<string>{};
//} // namespace tr1
template <> struct equal_to<SEACAVE::String> : public equal_to<string>{};
} // namespace std

#endif // __SEACAVE_STRING_H__
