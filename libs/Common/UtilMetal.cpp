////////////////////////////////////////////////////////////////////
// UtilMetal.cpp
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "UtilMetal.h"

#ifndef _USE_METAL

namespace SEACAVE {
namespace METAL {

bool isRuntimeAvailable()
{
	return false;
}

} // namespace METAL
} // namespace SEACAVE

#endif // _USE_METAL