////////////////////////////////////////////////////////////////////
// LinkLib.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_LINKLIB_H__
#define __SEACAVE_LINKLIB_H__


// I N C L U D E S /////////////////////////////////////////////////

#ifdef _MSC_VER
#else
#include <dlfcn.h>
#endif


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////


/**************************************************************************************
 * CLinkLib
 * ---------------
 * manage dynamic linked libraries
 **************************************************************************************/

class GENERAL_API CLinkLib
{
public:
	#ifdef _MSC_VER
	typedef HINSTANCE LibID;
	#else
	typedef void* LibID;
	#endif

	CLinkLib() : m_hLib(NULL)
	{
	}

	CLinkLib(const String& dllName) : m_hLib(NULL)
	{
		Load(dllName);
	}

	CLinkLib(const CLinkLib& lib) : m_hLib(lib.m_hLib)
	{
		((CLinkLib&)lib).m_hLib = NULL;
	}

	~CLinkLib()
	{
		Free();
	}

	bool	Load(const String& dllName)
	{
		Free();
		#ifdef _MSC_VER
		m_hLib = LoadLibrary(dllName);
		#else
		m_hLib = dlopen(dllName, RTLD_NOW);
		#endif
		return (m_hLib != NULL);
	}

	void	Free()
	{
		if (!IsLoaded())
			return;
		#ifdef _MSC_VER
		FreeLibrary(m_hLib);
		#else
		dlclose(m_hLib);
		#endif
		m_hLib = NULL;
	}

	void*	GetFunction(const String& funcName) const
	{
		#ifdef _MSC_VER
		return GetProcAddress(m_hLib, funcName);
		#else
		return dlsym(m_hLib, funcName);
		#endif
	}

	bool	IsLoaded() const
	{
		return (m_hLib != NULL);
	}

	LibID	GetLibID() const
	{
		return m_hLib;
	}

protected:
	LibID	m_hLib;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_LINKLIB_H__
