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

// a camera seen through the projection model of its view: the depth of a pixel is the distance along
// the model's ray (z for a pinhole, the range for an equirectangular panorama, whose columns wrap);
// the geometry outside the PatchMatch estimator (fusion, depth-map filters, point colors and normals)
// goes through it, so pinhole and spherical views mix; the pinhole calls are the camera's own
class ModelCamera
{
public:
	const Camera& camera;
	const bool bSpherical;

	inline ModelCamera(const Camera& _camera, CameraModelType model) : camera(_camera), bSpherical(model == CameraModelType::SPHERICAL) {}

	// the equirectangular model of a spherical camera, from its angular K
	inline Spherical<REAL> GetSpherical() const {
		ASSERT(bSpherical && ISZERO(camera.K(0,1)));
		return Spherical<REAL>{camera.K(0,0), camera.K(1,1), camera.K(0,2), camera.K(1,2)};
	}

	// camera-frame point of pixel x at the given depth
	inline Point3 TransformPointI2C(const Point2& x, REAL depth) const {
		if (!bSpherical)
			return camera.TransformPointI2C(Point3(x.x, x.y, depth));
		ASSERT(depth > 0);
		const Eigen::Vector3d ray(GetSpherical().Ray(Eigen::Vector2d(x.x, x.y)));
		return Point3(ray.x()*depth, ray.y()*depth, ray.z()*depth);
	}
	// world point of pixel x at the given depth
	inline Point3 TransformPointI2W(const Point2& x, REAL depth) const {
		return camera.TransformPointC2W(TransformPointI2C(x, depth));
	}
	// world-frame direction of the ray of pixel x (not normalized)
	inline Point3 RayPoint(const Point2& x) const {
		if (!bSpherical)
			return camera.RayPoint(x);
		return camera.R.t() * TransformPointI2C(x, REAL(1));
	}
	// pixel and depth of a world point; the depth is not positive if the point is behind a pinhole camera
	template <typename TYPE>
	inline std::tuple<TPoint2<TYPE>,TYPE> ProjectPointP(const TPoint3<TYPE>& X) const {
		if (!bSpherical)
			return camera.ProjectPointP(X);
		const Point3 Xc(camera.TransformPointW2C(Cast<REAL>(X)));
		Eigen::Vector2d x;
		GetSpherical().Project(Eigen::Vector3d(Xc.x, Xc.y, Xc.z), x);
		return {TPoint2<TYPE>(TYPE(x.x()), TYPE(x.y())), TYPE(norm(Xc))};
	}
	// pixel of a world point in front of the camera
	inline Point2 TransformPointW2I(const Point3& X) const {
		if (!bSpherical)
			return camera.TransformPointW2I(X);
		return std::get<0>(ProjectPointP(X));
	}
	// the pixel of an image of the given size closest to x, its column wrapped for a spherical camera;
	// false if outside the image
	template <typename TYPE>
	inline bool GetPixel(const TPoint2<TYPE>& x, const cv::Size& size, ImageRef& ir) const {
		ir = ROUND2INT(x);
		if (bSpherical)
			ir.x = WrapColumn(ir.x, size.width);
		return Image8U::isInside(ir, size);
	}
	// a column at most one width off the image, wrapped onto it (equirectangular images)
	static inline int WrapColumn(int c, int width) {
		ASSERT(c >= -width && c < 2*width);
		return c < 0 ? c + width : (c >= width ? c - width : c);
	}
	// the column offset a-b, the shortest one around the panorama for a spherical camera
	template <typename TYPE>
	inline TYPE ColumnOffset(TYPE a, TYPE b, int width) const {
		const TYPE d(a - b);
		if (!bSpherical)
			return d;
		return d > TYPE(width)/2 ? d - TYPE(width) : (d < -TYPE(width)/2 ? d + TYPE(width) : d);
	}
};
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
	Camera camera; // view's pose
	CameraModelType model; // projection model of the image file, from its platform camera (UpdateCamera)
	CameraLens lens; // BROWN: the lens of the image file, removed when the pixels are read
	uint32_t width, height; // image size
	Image8U3 image; // image color pixels
	Image8U mask; // image 8-bit segmentation mask, max 256 labels
	ViewScoreArr neighbors; // scored neighbor images (image indices ordered by score)
	float scale; // image scale relative to the original size
	float avgDepth; // average depth of the points seen by this camera

public:
	inline Image() : poseID(NO_ID), model(CameraModelType::PINHOLE), width(0), height(0), avgDepth(0) {}

	inline bool IsValid() const { return poseID != NO_ID; }
	// the pixels of the image file are undistorted when read (BROWN), which leaves the
	// undistorted pixels without a source invalid (see GetUndistortMaps()->mask)
	inline bool IsUndistortedOnRead() const { return model == CameraModelType::BROWN; }
	// the depth of a world point along the ray of the camera model: z, or the range for a spherical image
	inline REAL PointDepth(const Point3& X) const { return model == CameraModelType::SPHERICAL ? camera.Distance(X) : camera.PointDepth(X); }
	inline bool IsSpherical() const { return model == CameraModelType::SPHERICAL; }
	inline ModelCamera GetModelCamera() const { return ModelCamera(camera, model); }
	inline String GetMaskFileName() const { return maskName.empty() ? Util::getFileFullName(name)+".mask.png" : maskName; }
	inline bool HasResolution() const { return width > 0 && height > 0; }
	inline cv::Size GetSize() const { return cv::Size(width, height); }
	REAL GetSizeScale(unsigned nMaxResolution) const;
	cv::Size GetSize(unsigned nMaxResolution) const;

	// read image data from the file
	static IMAGEPTR OpenImage(const String& fileName);
	static IMAGEPTR ReadImageHeader(const String& fileName);
	static IMAGEPTR ReadImage(const String& fileName, Image8U3& image);
	static bool ReadImage(IMAGEPTR pImage, Image8U3& image);
	// read the pixels of the image file at its resolution, as the camera sees them: undistorted if
	// IsUndistortedOnRead(); every reader of the image pixels goes through it
	IMAGEPTR ReadPixels(Image8U3& image) const;
	// read the segmentation mask file (GetMaskFileName()) as the camera sees it, at the image file
	// resolution: undistorted with nearest-neighbor sampling if IsUndistortedOnRead()
	bool LoadMask(Image8U& mask) const;
	// the undistortion maps of the image file, shared by the images of the camera; IsUndistortedOnRead() only
	std::shared_ptr<const CameraUndistortMaps> GetUndistortMaps() const;
	bool LoadImage(const String& fileName, unsigned nMaxResolution=0);
	bool ReloadImage(unsigned nMaxResolution=0, bool bLoadPixels=true);
	// reload the pixels at the resolution the scene was prepared at: width/height
	// are left at that resolution when the pixels are released, so the scale
	// recomputation reproduces them exactly
	bool ReloadImageAtPreparedResolution(bool bLoadPixels=true) { return ReloadImage(MAXF(width, height), bLoadPixels); }
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

static inline IIndex ImageID2Index(const ImageArr& images, IIndex ID) {
	return images.FindFunc([&ID](const Image& image) { return image.ID == ID; });
}
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_VIEW_H_
