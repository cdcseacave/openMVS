/*
* Image.h
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

#ifndef _MVS_VIEW_H_
#define _MVS_VIEW_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Platform.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// a view instance seeing the scene
class Image
{
public:
	uint32_t platformID; // ID of the associated platform
	uint32_t cameraID; // ID of the associated camera on the associated platform
	uint32_t poseID; // ID of the pose of the associated platform
	String name; // image file name
	uint32_t width, height; // image size
	Image8U3 image; // image color pixels
	Image32F imageGray; // image gray pixels
	Camera camera; // view's pose

public:
	inline Image() {}

	inline bool IsValid() const { return poseID != NO_ID; }

	// read image data from the file
	bool LoadImage(const String& fileName, unsigned nMaxResolution=0);
	bool ReloadImage(unsigned nMaxResolution=0, bool bLoadPixels=true);
	void ReleaseImage();
	float ResizeImage(unsigned nMaxResolution=0);
	unsigned ComputeMaxResolution(unsigned& level, unsigned minImageSize) const;

	void UpdateCamera(const PlatformArr& platforms);
};
typedef SEACAVE::cList<Image, const Image&, 2> ImageArr;
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_VIEW_H_
