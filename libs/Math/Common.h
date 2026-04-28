////////////////////////////////////////////////////////////////////
// Common.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __MATH_COMMON_H__
#define __MATH_COMMON_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "../Common/Common.h"

// Per-library export macro: keyed only on Math_EXPORTS (auto-defined by CMake
// for the Math target) so Math symbols are exported while building Math.dll
// and imported elsewhere, without affecting the export state of Common symbols.
#ifndef MATH_API
  #ifdef _MSC_VER
    #if defined(_USRDLL)
      #ifdef Math_EXPORTS
        #define MATH_API EXPORT_API
      #else
        #define MATH_API IMPORT_API
      #endif
    #elif defined(OPENMVS_SHARED)
      #define MATH_API IMPORT_API
    #else
      #define MATH_API
    #endif
  #else
    #ifdef Math_EXPORTS
      #define MATH_API EXPORT_API
    #else
      #define MATH_API
    #endif
  #endif
#endif
#ifndef MATH_TPL
  #ifdef Math_EXPORTS
    #define MATH_TPL
  #else
    #define MATH_TPL extern
  #endif
#endif

#include "LMFit/lmmin.h"
#include "DisjointSet.h"
#include "RobustNorms.h"
#include "SimilarityTransform.h"


// D E F I N E S ///////////////////////////////////////////////////


// P R O T O T Y P E S /////////////////////////////////////////////

using namespace SEACAVE;

#endif // __MATH_COMMON_H__
