////////////////////////////////////////////////////////////////////
// Common.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __IO_COMMON_H__
#define __IO_COMMON_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "../Common/Common.h"

// Per-library export macro: keyed only on IO_EXPORTS so IO symbols are
// exported while building IO.dll and imported elsewhere, without affecting
// the export state of symbols owned by Common.
#ifndef IO_API
  #ifdef _MSC_VER
    #if defined(_USRDLL)
      #ifdef IO_EXPORTS
        #define IO_API EXPORT_API
      #else
        #define IO_API IMPORT_API
      #endif
    #elif defined(OPENMVS_SHARED)
      #define IO_API IMPORT_API
    #else
      #define IO_API
    #endif
  #else
    #ifdef IO_EXPORTS
      #define IO_API EXPORT_API
    #else
      #define IO_API
    #endif
  #endif
#endif
#ifndef IO_TPL
  #ifdef IO_EXPORTS
    #define IO_TPL
  #else
    #define IO_TPL extern
  #endif
#endif

#define _IMAGE_BMP		// add BMP support
#define _IMAGE_TGA		// add TGA support
#define _IMAGE_DDS		// add DDS support
#ifdef _USE_PNG
#define _IMAGE_PNG		// add PNG support
#endif
#ifdef _USE_JPG
#define _IMAGE_JPG		// add JPG support
#endif
#ifdef _USE_JXL
#define _IMAGE_JXL		// add JpegXL support
#endif
#ifdef _USE_TIFF
#define _IMAGE_TIFF		// add TIFF support
#endif

#include "ImageSCI.h"
#ifdef _IMAGE_BMP
#include "ImageBMP.h"
#endif
#ifdef _IMAGE_TGA
#include "ImageTGA.h"
#endif
#ifdef _IMAGE_DDS
#include "ImageDDS.h"
#endif
#ifdef _IMAGE_PNG
#include "ImagePNG.h"
#endif
#ifdef _IMAGE_JPG
#include "ImageJPG.h"
#endif
#ifdef _IMAGE_TIFF
#include "ImageTIFF.h"
#endif
#ifdef _IMAGE_JXL
#include "ImageJXL.h"
#endif
#include "PLY.h"
#include "OBJ.h"
/*----------------------------------------------------------------*/

#endif // __IO_COMMON_H__
