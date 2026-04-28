////////////////////////////////////////////////////////////////////
// Common.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_COMMON_H_
#define _SFM_COMMON_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "../Common/Common.h"
#include "../Math/Common.h"
#include "../IO/Common.h"
#include "../Common/BS_thread_pool.hpp"

// Per-library export macro: keyed only on SFM_EXPORTS so SFM symbols are
// exported while building SFM.dll and imported elsewhere, without affecting
// the export state of symbols owned by Common/Math/IO (which use their own macros).
#ifndef SFM_API
  #ifdef _MSC_VER
    #if defined(_USRDLL)
      #ifdef SFM_EXPORTS
        #define SFM_API EXPORT_API
      #else
        #define SFM_API IMPORT_API
      #endif
    #elif defined(OPENMVS_SHARED)
      #define SFM_API IMPORT_API
    #else
      #define SFM_API
    #endif
  #else
    #ifdef SFM_EXPORTS
      #define SFM_API EXPORT_API
    #else
      #define SFM_API
    #endif
  #endif
#endif
#ifndef SFM_TPL
  #ifdef SFM_EXPORTS
    #define SFM_TPL
  #else
    #define SFM_TPL extern
  #endif
#endif


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////


#endif // _SFM_COMMON_H_

