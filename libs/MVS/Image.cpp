/*
* Image.cpp
*
* Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
* Please read <http://foxel.ch/license> for more information.
*
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This file is part of the FOXEL project <http://foxel.ch>.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*
*      You are required to attribute the work as explained in the "Usage and
*      Attribution" section of <http://foxel.ch/license>.
*/

#include "Common.h"
#include "Image.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

IMAGEPTR Image::OpenImage(const String& fileName)
{
	#if 0
	if (Util::isFullPath(fileName))
		return IMAGEPTR(CImage::Create(fileName, CImage::READ));
	return IMAGEPTR(CImage::Create((Util::getCurrentDirectory()+fileName).c_str(), CImage::READ));
	#else
	return IMAGEPTR(CImage::Create(fileName, CImage::READ));
	#endif
} // OpenImage
/*----------------------------------------------------------------*/

IMAGEPTR Image::ReadImageHeader(const String& fileName)
{
	IMAGEPTR pImage(OpenImage(fileName));
	if (pImage == NULL || FAILED(pImage->ReadHeader())) {
		LOG("error: failed loading image header");
		pImage.Release();
	}
	return pImage;
} // ReadImageHeader
/*----------------------------------------------------------------*/

IMAGEPTR Image::ReadImage(const String& fileName, Image8U3& image)
{
	IMAGEPTR pImage(OpenImage(fileName));
	if (pImage != NULL && !ReadImage(pImage, image))
		pImage.Release();
	return pImage;
} // ReadImage
/*----------------------------------------------------------------*/

bool Image::ReadImage(IMAGEPTR pImage, Image8U3& image)
{
	if (FAILED(pImage->ReadHeader())) {
		LOG("error: failed loading image header");
		return false;
	}
	image.create(pImage->GetHeight(), pImage->GetWidth());
	if (FAILED(pImage->ReadData(image.data, PF_R8G8B8, 3, (UINT)image.step))) {
		LOG("error: failed loading image data");
		return false;
	}
	return true;
} // ReadImage
/*----------------------------------------------------------------*/


bool Image::LoadImage(const String& fileName, unsigned nMaxResolution)
{
	name = fileName;
	// open image file
	IMAGEPTR pImage(OpenImage(fileName));
	if (pImage == NULL) {
		LOG("error: failed opening input image '%s'", name.c_str());
		return false;
	}
	// create and fill image data
	if (!ReadImage(pImage, image)) {
		LOG("error: failed loading image '%s'", name.c_str());
		return false;
	}
	// resize image if needed
	scale = ResizeImage(nMaxResolution);
	return true;
} // LoadImage
/*----------------------------------------------------------------*/

// open the stored image file name and read again the image data
bool Image::ReloadImage(unsigned nMaxResolution, bool bLoadPixels)
{
	IMAGEPTR pImage(bLoadPixels ? ReadImage(name, image) : ReadImageHeader(name));
	if (pImage == NULL) {
		LOG("error: failed reloading image '%s'", name.c_str());
		return false;
	}
	if (!bLoadPixels) {
		// init image size
		width = pImage->GetWidth();
		height = pImage->GetHeight();
	}
	// resize image if needed
	scale = ResizeImage(nMaxResolution);
	return true;
} // ReloadImage
/*----------------------------------------------------------------*/

// free the image data
void Image::ReleaseImage()
{
	image.release();
} // ReleaseImage
/*----------------------------------------------------------------*/

// resize image if needed
// return scale
float Image::ResizeImage(unsigned nMaxResolution)
{
	if (!image.empty()) {
		width = image.width();
		height = image.height();
	}
	if (nMaxResolution == 0 || (width <= nMaxResolution && height <= nMaxResolution))
		return 1.f;
	float scale;
	if (width > height) {
		scale = (float)nMaxResolution/width;
		height = height*nMaxResolution/width;
		width = nMaxResolution;
	} else {
		scale = (float)nMaxResolution/height;
		width = width*nMaxResolution/height;
		height = nMaxResolution;
	}
	if (!image.empty())
		cv::resize(image, image, cv::Size((int)width, (int)height), 0, 0, cv::INTER_LINEAR);
	return scale;
} // ResizeImage
/*----------------------------------------------------------------*/

// compute image scale for a given max and min resolution
unsigned Image::ComputeMaxResolution(unsigned& level, unsigned minImageSize) const
{
	const unsigned maxImageSize = MAXF(width, height);
	return Image8U3::computeMaxResolution(maxImageSize, level, minImageSize);
} // ComputeMaxResolution
/*----------------------------------------------------------------*/

// compute image scale for a given max and min resolution, using the current image file data
unsigned Image::RecomputeMaxResolution(unsigned& level, unsigned minImageSize) const
{
	IMAGEPTR pImage(ReadImageHeader(name));
	unsigned maxImageSize;
	if (pImage == NULL) {
		// something went wrong, use the current known size (however it will most probably fail later)
		maxImageSize = MAXF(width, height);
	} else {
		// re-compute max image size
		maxImageSize = MAXF(pImage->GetWidth(), pImage->GetHeight());
	}
	return Image8U3::computeMaxResolution(maxImageSize, level, minImageSize);
} // RecomputeMaxResolution
/*----------------------------------------------------------------*/


// compute the camera extrinsics from the platform pose and the relative camera pose to the platform
void Image::UpdateCamera(const PlatformArr& platforms)
{
	ASSERT(platformID != NO_ID);
	ASSERT(cameraID != NO_ID);
	ASSERT(poseID != NO_ID);

	// compute the normalized absolute camera pose
	const Platform& platform = platforms[platformID];
	camera = platform.GetCamera(cameraID, poseID);

	// compute the unnormalized camera
	camera.K = camera.GetK<REAL>(width, height);
	camera.ComposeP();
} // UpdateCamera
/*----------------------------------------------------------------*/
