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
#include "../Math/Camera/CameraUndistort.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

typedef uint32_t IIndex;
typedef SEACAVE::cList<IIndex, IIndex, 0, 16, IIndex> IIndexArr;
typedef _INTERFACE_NAMESPACE::Interface::Image::ViewScore ViewScore;
typedef CLISTDEF0IDX(ViewScore, IIndex) ViewScoreArr;
/*----------------------------------------------------------------*/

// a view instance seeing the scene
class MVS_API Image
{
public:
	uint32_t platformID; // ID of the associated platform
	uint32_t cameraID; // ID of the associated camera on the associated platform
	uint32_t poseID; // ID of the pose of the associated platform
	uint32_t ID; // global ID of the image (the ID given outside the current scene - ex. the index in the full list of image files)
	String name; // image file name (relative path)
	String maskName; // segmentation file name (optional)
	Camera camera; // view's pose, intrinsics and projection model, from its platform camera (UpdateCamera)
	uint32_t width, height; // image size
	Image8U3 image; // image color pixels
	Image8U mask; // image 8-bit segmentation mask, max 256 labels
	ViewScoreArr neighbors; // scored neighbor images (image indices ordered by score)
	float scale; // image scale relative to the original size
	float avgDepth; // average depth of the points seen by this camera

public:
	inline Image() : platformID(NO_ID), cameraID(NO_ID), poseID(NO_ID), ID(NO_ID), width(0), height(0), scale(1), avgDepth(0) {}

	inline bool IsValid() const { return poseID != NO_ID; }
	// the pixels of the image file are undistorted when read (BROWN), which leaves the
	// undistorted pixels without a source invalid (see GetUndistortMaps()->mask)
	inline bool IsUndistortedOnRead() const { return camera.model == CameraModelType::BROWN; }
	inline bool IsSpherical() const { return camera.IsSpherical(); }
	// the platform camera of the image, describing its file (the lens of a Brown camera)
	inline const Platform::Camera& GetPlatformCamera(const PlatformArr& platforms) const {
		ASSERT(platformID < platforms.size() && cameraID < platforms[platformID].cameras.size());
		return platforms[platformID].cameras[cameraID];
	}
	// the lens of the image file; IsUndistortedOnRead() only
	inline const CameraLens& GetLens(const PlatformArr& platforms) const {
		ASSERT(IsUndistortedOnRead());
		const Platform::Camera& platformCamera = GetPlatformCamera(platforms);
		ASSERT(platformCamera.model == CameraModelType::BROWN);
		return platformCamera.lens;
	}
	inline String GetMaskFileName() const { return maskName.empty() ? Util::getFileFullName(name)+".mask.png" : maskName; }
	inline bool HasResolution() const { return width > 0 && height > 0; }
	inline cv::Size GetSize() const { return cv::Size(width, height); }
	REAL GetSizeScale(unsigned nMaxResolution) const;
	cv::Size GetSize(unsigned nMaxResolution) const;
	// the size of a raster of the given size scaled by the given factor, as TImage::computeResize; an
	// equirectangular raster keeps an even width, which the seam-wrapped red/black propagation of its
	// depth-map needs at every level: the width is rounded to an even number and the height follows
	// from that uniform factor, so a 2:1 panorama stays 2:1 (its angular K keeps fx=W/2pi, fy=H/pi)
	static cv::Size ComputeResize(const cv::Size& size, REAL scale, bool bSpherical);

	// read image data from the file
	static IMAGEPTR OpenImage(const String& fileName);
	static IMAGEPTR ReadImageHeader(const String& fileName);
	static IMAGEPTR ReadImage(const String& fileName, Image8U3& image);
	static bool ReadImage(IMAGEPTR pImage, Image8U3& image);
	// read the pixels of the image file at its resolution, as the camera sees them: undistorted if
	// IsUndistortedOnRead(), through the lens of its platform camera; every reader of the image pixels
	// goes through it
	IMAGEPTR ReadPixels(const PlatformArr& platforms, Image8U3& image) const;
	// read the segmentation mask file (GetMaskFileName()) as the camera sees it, at the image file
	// resolution: undistorted with nearest-neighbor sampling if IsUndistortedOnRead()
	bool LoadMask(const PlatformArr& platforms, Image8U& mask) const;
	// the undistortion maps of the image file, shared by the images of the camera; IsUndistortedOnRead() only
	std::shared_ptr<const CameraUndistortMaps> GetUndistortMaps(const PlatformArr& platforms) const;
	bool LoadImage(const PlatformArr& platforms, const String& fileName, unsigned nMaxResolution=0);
	bool ReloadImage(const PlatformArr& platforms, unsigned nMaxResolution=0, bool bLoadPixels=true);
	// reload the pixels at the resolution the scene was prepared at: width/height
	// are left at that resolution when the pixels are released, so the scale
	// recomputation reproduces them exactly
	bool ReloadImageAtPreparedResolution(const PlatformArr& platforms, bool bLoadPixels=true) { return ReloadImage(platforms, MAXF(width, height), bLoadPixels); }
	void ReleaseImage();
	float ResizeImage(unsigned nMaxResolution=0);
	// resize a raster on an image grid to its uniform size (see TImage::computeResize) with one factor on
	// both axes, so the pixel mapping matches the K GetCamera() returns for that size; resizing
	// straight to the size maps each axis by its own rounded ratio, up to half a pixel off
	static void ResizeUniform(cv::InputArray src, cv::OutputArray dst, const cv::Size& size, int interpolation);
	unsigned RecomputeMaxResolution(unsigned& level, unsigned minImageSize, unsigned maxImageSize=INT_MAX) const;

	Image GetImage(const PlatformArr& platforms, double scale, bool bUseImage=true) const;
	Camera GetCamera(const PlatformArr& platforms, const cv::Size& resolution, bool forceAspect=false) const;
	void UpdateCamera(const PlatformArr& platforms);
	// the field of view along the width (0), the height (1) or, for a pinhole camera, the diagonal (2)
	REAL ComputeFOV(int dir=0) const;

	static bool StereoRectifyImages(const Image& image1, const Image& image2, const Point3fArr& points1, const Point3fArr& points2, Image8U3& rectifiedImage1, Image8U3& rectifiedImage2, Image8U& mask1, Image8U& mask2, Matrix3x3& H, Matrix4x4& Q);
	static void ScaleStereoRectification(Matrix3x3& H, Matrix4x4& Q, REAL scale);
	static float Disparity2Depth(const Matrix4x4& Q, const ImageRef& u, float d);
	static float Disparity2Depth(const Matrix4x4& Q, const Point2f& u, float d);
	static float Disparity2Depth(const Matrix4x4& Q, const ImageRef& u, float d, Point2f& pt);
	static float Disparity2Depth(const Matrix4x4& Q, const Point2f& u, float d, Point2f& pt);
	static float Disparity2Distance(const Matrix3x3& K, const Matrix4x4& Q, const Point2f& u, float d);
	static bool Depth2Disparity(const Matrix4x4& Q, const Point2f& u, float d, float& disparity);

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
		ar & ID;
		const String relName(MAKE_PATH_REL(WORKING_FOLDER_FULL, name));
		ar & relName;
		const String relMaskName(maskName.empty() ? String() : MAKE_PATH_REL(WORKING_FOLDER_FULL, maskName));
		ar & relMaskName;
		ar & width & height;
		ar & neighbors;
		ar & avgDepth;
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/) {
		ar & platformID;
		ar & cameraID;
		ar & poseID;
		ar & ID;
		ar & name;
		name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, name);
		ar & maskName;
		maskName = maskName.empty() ? String() : MAKE_PATH_FULL(WORKING_FOLDER_FULL, maskName);
		ar & width & height;
		ar & neighbors;
		ar & avgDepth;
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
};
typedef CLISTDEF2IDX(Image,IIndex) ImageArr;

// maps the feature observations of an image between its file and the image as the camera sees it,
// both at the image file resolution: the identity unless the image is undistorted on read
class ObservationMap {
public:
	ObservationMap(const Image& imageData, const PlatformArr& platforms) : bUndistort(imageData.IsUndistortedOnRead()) {
		if (!bUndistort)
			return;
		const CameraLens& lens = imageData.GetLens(platforms);
		const KMatrix undistortedK(UndistortedK(lens.K, lens.distortion, lens.size));
		ASSERT(ISZERO(lens.K(0,1)) && ISZERO(undistortedK(0,1)));
		file = Pinhole<REAL>{lens.K(0,0), lens.K(1,1), lens.K(0,2), lens.K(1,2)};
		undistorted = Pinhole<REAL>{undistortedK(0,0), undistortedK(1,1), undistortedK(0,2), undistortedK(1,2)};
		distortion = lens.distortion;
	}
	Point2f FileToCamera(const Point2f& x) const {
		if (!bUndistort)
			return x;
		const Eigen::Vector2d u(undistorted.NormalizedToPixel(distortion.Undistort(file.PixelToNormalized(Point2(x)))));
		return Point2f(Point2(u));
	}
	Point2f CameraToFile(const Point2f& x) const {
		if (!bUndistort)
			return x;
		const Eigen::Vector2d d(file.NormalizedToPixel(distortion.Distort(undistorted.PixelToNormalized(Point2(x)))));
		return Point2f(Point2(d));
	}
	// one map per image
	static std::vector<ObservationMap> MakeMaps(const ImageArr& images, const PlatformArr& platforms) {
		std::vector<ObservationMap> maps;
		maps.reserve(images.size());
		for (const Image& imageData: images)
			maps.emplace_back(imageData, platforms);
		return maps;
	}
private:
	bool bUndistort;
	Pinhole<REAL> file{}, undistorted{}; // cameras of the image file and of the undistorted image
	BrownDistortion<REAL> distortion{};
};
typedef std::vector<ObservationMap> ObservationMapArr;

static inline IIndex ImageID2Index(const ImageArr& images, IIndex ID) {
	return images.FindFunc([&ID](const Image& image) { return image.ID == ID; });
}
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_VIEW_H_
