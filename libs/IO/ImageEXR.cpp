////////////////////////////////////////////////////////////////////
// ImageEXR.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"

#include "ImageEXR.h"

#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfChannelList.h>
#include <OpenEXR/ImfArray.h>
#include "OpenEXR/Iex.h"

#include <OpenEXR/ImfNamespace.h>

namespace IMF = OPENEXR_IMF_NAMESPACE;

using namespace std;
using namespace IMF;
using namespace SEACAVE;
using namespace IMATH_NAMESPACE;



// D E F I N E S ///////////////////////////////////////////////////
typedef struct tagRGBF {
	float red;
	float green;
	float blue;
} RGBF;

typedef struct tagRGBI {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} RGBI;


// F U N C T I O N S ///////////////////////////////////////////////
int
pixelType (PixelType pt)
{
	int val=0;
    switch (pt)
    {
        case UINT:
            //cout << "32-bit unsigned integer";
						break;
        case HALF:
            //cout << "16-bit floating-point";
						val = 2;
						break;
        case FLOAT:
            //cout << "32-bit floating-point";
						val = 4;
						break;
        default:
            //cout << "check exr type --> " << int (pt);
						break;
    }
		return val;
}

// S T R U C T S ///////////////////////////////////////////////////
class C_IStream: public IStream
{
  public:

    C_IStream (FILE *file, const char fileName[]):
	IStream (fileName), _file (file) {}

    virtual bool	read (char c[/*n*/], int n);
    virtual Int64	tellg ();
    virtual void	seekg (Int64 pos);
    virtual void	clear ();

  private:

    FILE *		_file;
};

bool
C_IStream::read (char c[/*n*/], int n)
{
    if (n != static_cast<int>(fread (c, 1, n, _file)))
    {
        //
        // fread() failed, but the return value does not distinguish
        // between I/O errors and end of file, so we call ferror() to
        // determine what happened.
        //

        if (ferror (_file))
            IEX_NAMESPACE::throwErrnoExc();
        else
            throw IEX_NAMESPACE::InputExc ("Unexpected end of file.");
    }

    return feof (_file);
}


Int64
C_IStream::tellg ()
{
    return ftell (_file);
}


void
C_IStream::seekg (Int64 pos)
{
    clearerr (_file);
    fseek (_file, pos, SEEK_SET);
}

void
C_IStream::clear ()
{
    clearerr (_file);
}


CImageEXR::CImageEXR()
{
} // Constructor

CImageEXR::~CImageEXR()
{
	Close();
} // Destructor
/*----------------------------------------------------------------*/

void CImageEXR::Close()
{
	m_width = m_height = 0;
	CImage::Close();
}
/*----------------------------------------------------------------*/


HRESULT CImageEXR::ReadHeader()
{
	int byte=0,channelsNum=0;
	string colortype="";
	// reading image
  {

		FILE * cfile = fopen (m_fileName, "rb");
		if (!cfile) {
				LOG(LT_IMAGE, "error: unsupported EXR image");
				return _INVALIDFILE;
		}

    if (cfile == 0)
    {
      LOG(LT_IMAGE, "Cannot open EXR file '%s'", m_fileName);
    }
    else
    {
      try
      {
				C_IStream istr (cfile, "EXR_FILE");
				RgbaInputFile exr (istr);
				Box2i dw = exr.dataWindow();

				m_dataWidth =m_width=dw.max.x - dw.min.x + 1;
				m_dataHeight=m_height=dw.max.y - dw.min.y + 1;

				if(exr.isComplete())
				{
					m_numLevels = 0;
					m_level     = 0;

					const ChannelList &channels = exr.header().channels();
			    for (ChannelList::ConstIterator i = channels.begin(); i != channels.end(); ++i)
			         {
							  colortype+=i.name();
								if(strcmp(i.name(), "R")||strcmp(i.name(), "G")||strcmp(i.name(), "B")||strcmp(i.name(), "A") == 0){
									byte=pixelType(i.channel().type);
								}
			          channelsNum++;
			         }

				 // check if 32bit image
				 if(byte==4)
				 {
					 // channel number
					 switch (channelsNum)
					 {
						 case 1:
						 	if(colortype == "A"){
								m_format = PF_A32;
							}else{
								m_format = PF_GRAY32;
							}
						 		m_stride = channelsNum*byte;
						 		break;
						 case 3:
						 		m_format = PF_R32G32B32;
						 		m_stride = channelsNum*byte;
						 		break;
						 case 4:
						 		m_format = PF_R32G32B32A32;
						 		m_stride = channelsNum*byte;
								// consider RGBA as RGB ??
								// m_format = PF_R32G32B32;
						 		// m_stride = (channelsNum-1)*byte;
						 		break;
						 default:
						 		LOG(LT_IMAGE, "error: unsupported EXR image");
								Close();
						 		return _INVALIDFILE;
					 }
					 m_lineWidth = m_width * m_stride;

				 }else{
					 Close();
					 LOG(LT_IMAGE, "error: not a 32bit float EXR image");
					 return _FAIL;
				 }

			 }else{
				 Close();
				 LOG(LT_IMAGE, "error: incomplete EXR image");
				 return _FAIL;
			 }
      }
      catch (...)
      {
				LOG(LT_IMAGE, "error: can't process EXR image");
				Close();
        throw;
				return _FAIL;
      }
			// ending as expected
			return _OK;
    }
  }

	Close();
	return _FAIL;
} // ReadHeader
/*----------------------------------------------------------------*/


HRESULT CImageEXR::ReadData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{
	FILE *cfile = fopen (m_fileName, "rb");
	if (!cfile) {
			LOG(LT_IMAGE, "error: unsupported EXR image");
			return _INVALIDFILE;
	}
	if (cfile == 0)
	{
		LOG(LT_IMAGE, "Cannot open EXR file '%s'", m_fileName);
	}
	else
	{
		try
		{
			if (m_width && m_height)
			{
				// use the low level interface
				C_IStream istr (cfile, "EXR_FILE");

				if (dataFormat == m_format && nStride == m_stride)
				{
					const BYTE *bits = (BYTE*&)pData;
					InputFile exr (istr);
					// build a frame buffer (i.e. what we want on output)
					FrameBuffer frameBuffer;

					Array2D<float> rPixels(m_height, m_width);
					const Box2i &dw = exr.header().dataWindow();
					const size_t bytespp = sizeof (rPixels[0][0]) * 1;
					const unsigned pitch = sizeof (rPixels[0][0]) * m_width;  // could be lineWidth ??
					// allow dataWindow with minimal bounds different form zero
					size_t offset = - dw.min.x * bytespp - dw.min.y * pitch;

					const char *channel_name[3] = { "R", "G", "B" };
					for(int c = 0; c < 3; c++)
					{
						frameBuffer.insert (
							channel_name[c],															// name
							Slice (FLOAT,																	// type
								(char*)(bits + c * sizeof(float) + offset), // base
								bytespp,																		// xStride sizeof (rPixels[0][0]) * 1,
								pitch,																			// yStride sizeof (rPixels[0][0]) * width,
								1, 1,																				// x/y sampling
								0.0));																			// fillValue
					}

					// read the file
					exr.setFrameBuffer(frameBuffer);
					exr.readPixels(dw.min.y, dw.max.y);

				}else{
						// read image to a buffer and convert it
						const BYTE *bits = (BYTE*&)pData;
						BYTE *data = (BYTE*)bits;
						RgbaInputFile exr (istr);
						// read the file in lines
						Box2i dw = exr.dataWindow();

						Array2D<Rgba> line(1, m_width);

						while (dw.min.y <= dw.max.y) {
							// read a line
							exr.setFrameBuffer (&line[0][0] - dw.min.x - dw.min.y * m_width, 1, m_width);
							exr.readPixels (dw.min.y, min(dw.min.y , dw.max.y));

							RGBI *pixel = (RGBI*)data;
							for(int x = 0; x < m_width; x++) {
								pixel[x].red = F32TO8(line[0][x].b);
								pixel[x].green = F32TO8(line[0][x].g);
								pixel[x].blue = F32TO8(line[0][x].r);
							}
							// next line
							data += lineWidth;
							dw.min.y += 1;
						}
				}
				return _OK;
			}
		}
		catch (...)
		{
			LOG(LT_IMAGE, "error: can't process EXR image");
			Close();
			throw;
			return _FAIL;
		}
	}
	Close();
	return _FAIL;
} // Read
/*----------------------------------------------------------------*/


HRESULT CImageEXR::WriteHeader(PIXELFORMAT imageFormat, Size width, Size height, BYTE /*numLevels*/)
{
	// TODO: to implement the EXR encoder

	return _OK;
} // WriteHeader
/*----------------------------------------------------------------*/


HRESULT CImageEXR::WriteData(void* pData, PIXELFORMAT dataFormat, Size nStride, Size lineWidth)
{

	// TODO: to implement the EXR encoder

	return _OK;
} // WriteData
/*----------------------------------------------------------------*/

//#endif // _IMAGE_EXR
