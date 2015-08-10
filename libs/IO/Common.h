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
#ifdef _USE_EXIV2
#define _IMAGE_EXIF		// complete EXIF info support based on Exiv2
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
#ifdef _IMAGE_EXIF
#include "ImageEXIF.h"
#endif
#include "EXIF.h"
#include "PLY.h"
#include "TinyXML2.h"
/*----------------------------------------------------------------*/

#endif // __IO_COMMON_H__
