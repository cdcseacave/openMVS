////////////////////////////////////////////////////////////////////
// ImageJPG.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_IMAGEJPG_H__
#define __SEACAVE_IMAGEJPG_H__


// D E F I N E S ///////////////////////////////////////////////////


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

class IO_API CImageJPG : public CImage
{
public:
	CImageJPG();
	virtual ~CImageJPG();

	void		Close();

	HRESULT		ReadHeader();
	HRESULT		ReadData(void*, PIXELFORMAT, UINT nStride, UINT lineWidth);
	HRESULT		WriteHeader(PIXELFORMAT, UINT width, UINT height, BYTE numLevels);
	HRESULT		WriteData(void*, PIXELFORMAT, UINT nStride, UINT lineWidth);

protected:
	void*		m_state;
}; // class CImageJPG
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_IMAGEJPG_H__
