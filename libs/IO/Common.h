////////////////////////////////////////////////////////////////////
// Common.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __IO_COMMON_H__
#define __IO_COMMON_H__


// I N C L U D E S /////////////////////////////////////////////////

#if defined(IO_EXPORTS) && !defined(Common_EXPORTS)
#define Common_EXPORTS
#endif

#include "../Common/Common.h"

#ifndef IO_API
#define IO_API GENERAL_API
#endif
#ifndef IO_TPL
#define IO_TPL GENERAL_TPL
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
#include "PLY.h"
#include "OBJ.h"
/*----------------------------------------------------------------*/

#endif // __IO_COMMON_H__
