////////////////////////////////////////////////////////////////////
// Common.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

// Source file that includes just the standard includes
// Common.pch will be the pre-compiled header
// Common.obj will contain the pre-compiled type information

#include "Common.h"

namespace SEACAVE {
// Tagged GENERAL_API to match the `extern GENERAL_API` declarations in
// Common.h so MSVC actually emits these as exported entries in Common.dll.
#if TD_VERBOSE == TD_VERBOSE_ON
GENERAL_API int g_nVerbosityLevel(2);
#endif
#if TD_VERBOSE == TD_VERBOSE_DEBUG
GENERAL_API int g_nVerbosityLevel(3);
#endif

GENERAL_API String g_strWorkingFolder;
GENERAL_API String g_strWorkingFolderFull;
} // namespace SEACAVE

#ifdef _USE_BOOST
#ifdef BOOST_NO_EXCEPTIONS
#if (BOOST_VERSION / 100000) > 1 || (BOOST_VERSION / 100 % 1000) > 72
#include <boost/assert/source_location.hpp>
#endif
namespace boost {
	void throw_exception(std::exception const & e) {
		VERBOSE("exception thrown: %s", e.what());
		ASSERT("boost exception thrown" == NULL);
		exit(EXIT_FAILURE);
	}
	#if (BOOST_VERSION / 100000) > 1 || (BOOST_VERSION / 100 % 1000) > 72
	void throw_exception(std::exception const & e, boost::source_location const & loc) {
		std::ostringstream ostr; ostr << loc;
		VERBOSE("exception thrown at %s: %s", ostr.str().c_str(), e.what());
		ASSERT("boost exception thrown" == NULL);
		exit(EXIT_FAILURE);
	}
	#endif
} // namespace boost
#endif
#endif

void SEACAVE::Initialize(LPCTSTR appname, unsigned nMaxThreads, int nProcessPriority) {
	// initialize thread options
	Process::setCurrentProcessPriority((Process::Priority)nProcessPriority);
	#ifdef _USE_OPENMP
	if (nMaxThreads != 0)
		omp_set_num_threads(nMaxThreads);
	#endif

	#ifdef _USE_BREAKPAD
	// initialize crash memory dumper
	MiniDumper::Create(appname, WORKING_FOLDER);
	#endif

	// initialize random number generator
	Util::Init();
}

void SEACAVE::Finalize() {
	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print memory statistics
	Util::LogMemoryInfo();
	#endif
}
/*----------------------------------------------------------------*/
