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

bool Image::LoadImage(const String& fileName, unsigned nMaxResolution)
{
	name = fileName;
	if (!image.Load(fileName)) {
		LOG("error: failed opening input image '%s'", name.c_str());
		return false;
	}
	// resize image if needed
	ResizeImage(nMaxResolution);
	return true;
} // LoadImage
/*----------------------------------------------------------------*/

// open the stored image file name and read again the image data
bool Image::ReloadImage(unsigned nMaxResolution, bool bLoadPixels)
{
	if (!image.Load(name)) {
		LOG("error: failed reloading image '%s'", name.c_str());
		return false;
	}
	// resize image if needed
	ResizeImage(nMaxResolution);
	// release image pixels if not needed
	if (!bLoadPixels)
		ReleaseImage();
	return true;
} // ReloadImage
/*----------------------------------------------------------------*/

// free the image data
void Image::ReleaseImage()
{
	image.release();
	imageGray.release();
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
