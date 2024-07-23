////////////////////////////////////////////////////////////////////
// ImageJPG.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#ifdef _IMAGE_PPM
#include "ImagePPM.h"

#ifdef _MSC_VER
#define XMD_H // prevent redefinition of INT32
#undef FAR  // prevent FAR redefinition
#endif

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

// F U N C T I O N S ///////////////////////////////////////////////

// S T R U C T S ///////////////////////////////////////////////////

CImagePPM::CImagePPM()
{

} // Constructor

CImagePPM::~CImagePPM()
{
    if (!m_data)
        delete m_data;
} // Destructor
/*----------------------------------------------------------------*/

void CImagePPM::Close()
{

}
/*----------------------------------------------------------------*/

HRESULT CImagePPM::ReadHeader()
{
    const size_t nSize = ((ISTREAM*)m_pStream)->getSize();

    ((ISTREAM*)m_pStream)->setPos(0);
    CAutoPtrArr<uint8_t> pData(new uint8_t[nSize]);
    m_pStream->read(pData, nSize);

    std::ifstream file((char *) &pData, nSize);

    // Read the magic number (P6).
    char magic_number[3];
    file.read(magic_number, 2);
    if (std::string(magic_number) != "P6") {
        std::cerr << "Error: Invalid image format." << std::endl;
        return _FAIL;
    }

    // Read the whitespace character.
    char whitespace;
    file.get(whitespace);
    if (whitespace != ' ') {
        std::cerr << "Error: Invalid image format." << std::endl;
        return _FAIL;
    }

    // Read the width.
    int width;
    file >> width;

    // Read the whitespace character.
    file.get(whitespace);
    if (whitespace != ' ') {
        std::cerr << "Error: Invalid image format." << std::endl;
        return _FAIL;
    }

    // Read the height.
    int height;
    file >> height;

    // Read the whitespace character.
    file.get(whitespace);
    if (whitespace != ' ') {
        std::cerr << "Error: Invalid image format." << std::endl;
        return _FAIL;
    }

    // Read the maximum value.
    int max_value;
    file >> max_value;

    m_dataWidth	= m_width = width;
	m_dataHeight= m_height = height;
	m_numLevels	= 0;
	m_level = 0;
	m_stride = 3;
	m_format = PF_R8G8B8;
	m_lineWidth = m_dataWidth * m_stride;

    size_t offset = nSize - (m_dataHeight * m_lineWidth);
    memcpy(m_data, pData + offset, m_dataHeight * m_lineWidth);

    return _OK;
} // ReadHeader
/*----------------------------------------------------------------*/

HRESULT CImagePPM::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
    memcpy(pData, m_data, m_height * lineWidth);
	return _OK;
} // Read
/*----------------------------------------------------------------*/

HRESULT CImagePPM::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE numLevels)
{
	//TODO: to implement the PPM encoder
	return _FAIL;
} // WriteHeader
/*----------------------------------------------------------------*/

HRESULT CImagePPM::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	//TODO: to implement the PPM encoder
	return _FAIL;
} // WriteData
/*----------------------------------------------------------------*/

#endif // _IMAGE_PPM
