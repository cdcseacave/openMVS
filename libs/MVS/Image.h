/*
* Image.h
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
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
*/

#ifndef _MVS_VIEW_H_
#define _MVS_VIEW_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Platform.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

typedef uint32_t IIndex;
typedef cList<IIndex, IIndex, 0, 16, IIndex> IIndexArr;

struct MVS_API ViewInfo {
	IIndex ID; // image ID
	uint32_t points; // number of 3D points shared with the reference image
	float scale; // image scale relative to the reference image
	float angle; // image angle relative to the reference image (radians)
	float area; // common image area relative to the reference image (ratio)

	#ifdef _USE_BOOST
				// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & ID;
		ar & points;
		ar & scale;
		ar & angle;
		ar & area;
	}
	#endif
};
typedef MVS_API TIndexScore<ViewInfo, float> ViewScore;
typedef MVS_API CLISTDEF0IDX(ViewScore, IIndex) ViewScoreArr;
/*----------------------------------------------------------------*/

// a view instance seeing the scene
class MVS_API Image
{
public:
	uint32_t platformID; // ID of the associated platform
	uint32_t cameraID; // ID of the associated camera on the associated platform
	uint32_t poseID; // ID of the pose of the associated platform
	String name; // image file name (relative path)
	Camera camera; // view's pose
	uint32_t width, height; // image size
	Image8U3 image; // image color pixels
	ViewScoreArr neighbors; // score&store the neighbor images
	float scale; // image scale relative to the original size
	float avgDepth; // average depth of the points seen by this camera

public:
	inline Image() {
		#ifndef _RELEASE
		width = height = 0;
		#endif
	}

	inline bool IsValid() const { return poseID != NO_ID; }
	inline Image8U::Size GetSize() const { return Image8U::Size(width, height); }

	// read image data from the file
	static IMAGEPTR OpenImage(const String& fileName);
	static IMAGEPTR ReadImageHeader(const String& fileName);
	static IMAGEPTR ReadImage(const String& fileName, Image8U3& image);
	static bool ReadImage(IMAGEPTR pImage, Image8U3& image);
	bool LoadImage(const String& fileName, unsigned nMaxResolution=0);
	bool ReloadImage(unsigned nMaxResolution=0, bool bLoadPixels=true);
	void ReleaseImage();
	float ResizeImage(unsigned nMaxResolution=0);
	unsigned RecomputeMaxResolution(unsigned& level, unsigned minImageSize) const;

	Camera GetCamera(const PlatformArr& platforms, const Image8U::Size& resolution) const;
	void UpdateCamera(const PlatformArr& platforms);

	float GetNormalizationScale() const {
		ASSERT(width > 0 && height > 0);
		return camera.GetNormalizationScale(width, height);
	}

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const {
		ar & platformID;
		ar & cameraID;
		ar & poseID;
		const String relName(MAKE_PATH_REL(WORKING_FOLDER_FULL, name));
		ar & relName;
		ar & width & height;
		ar & neighbors;
		ar & avgDepth;
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/) {
		ar & platformID;
		ar & cameraID;
		ar & poseID;
		ar & name;
		name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, name);
		ar & width & height;
		ar & neighbors;
		ar & avgDepth;
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
};
typedef MVS_API SEACAVE::cList<Image, const Image&, 2, 16, IIndex> ImageArr;
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_VIEW_H_
