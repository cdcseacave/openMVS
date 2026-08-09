////////////////////////////////////////////////////////////////////
// UtilMetal.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef  __SEACAVE_METAL_H__
#define  __SEACAVE_METAL_H__


namespace SEACAVE {
namespace METAL {

// Returns true if the Metal runtime can compile and complete a trivial compute
// dispatch within a bounded interval; returns false in builds without Metal.
GENERAL_API bool isRuntimeAvailable();

} // namespace METAL
} // namespace SEACAVE

#endif // __SEACAVE_METAL_H__