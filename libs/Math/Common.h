////////////////////////////////////////////////////////////////////
// Common.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __MATH_COMMON_H__
#define __MATH_COMMON_H__


// I N C L U D E S /////////////////////////////////////////////////

#if defined(Math_EXPORTS) && !defined(Common_EXPORTS)
#define Common_EXPORTS
#endif

#include "../Common/Common.h"

#ifndef MATH_API
#define MATH_API GENERAL_API
#endif
#ifndef MATH_TPL
#define MATH_TPL GENERAL_TPL
#endif
/*----------------------------------------------------------------*/

#endif // __MATH_COMMON_H__
