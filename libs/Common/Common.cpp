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
#if TD_VERBOSE == TD_VERBOSE_ON
int g_nVerbosityLevel(2);
#endif
#if TD_VERBOSE == TD_VERBOSE_DEBUG
int g_nVerbosityLevel(3);
#endif

String g_strWorkingFolder;
String g_strWorkingFolderFull;
} // namespace SEACAVE

#ifdef _USE_BOOST
#ifdef BOOST_NO_EXCEPTIONS
namespace boost {
	void throw_exception(std::exception const & e) {
		VERBOSE("exception thrown: %s", e.what());
		ASSERT("boost exception thrown" == NULL);
		exit(EXIT_FAILURE);
	}
} // namespace boost
#endif
#endif
