/*
* Image.cpp
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
	return IMAGEPTR(CImage::Create((Util::getCurrentFolder()+fileName).c_str(), CImage::READ));
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
	if (FAILED(pImage->ReadData(image.data, PF_R8G8B8, 3, (CImage::Size)image.step))) {
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
	if (nMaxResolution == 0 || MAXF(width,height) <= nMaxResolution)
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
		cv::resize(image, image, cv::Size((int)width, (int)height), 0, 0, cv::INTER_AREA);
	return scale;
} // ResizeImage
/*----------------------------------------------------------------*/

// compute image scale for a given max and min resolution, using the current image file data
unsigned Image::RecomputeMaxResolution(unsigned& level, unsigned minImageSize, unsigned maxImageSize) const
{
	IMAGEPTR pImage(ReadImageHeader(name));
	if (pImage == NULL) {
		// something went wrong, use the current known size (however it will most probably fail later)
		return Image8U3::computeMaxResolution(width, height, level, minImageSize, maxImageSize);
	}
	// re-compute max image size
	return Image8U3::computeMaxResolution(pImage->GetWidth(), pImage->GetHeight(), level, minImageSize, maxImageSize);
} // RecomputeMaxResolution
/*----------------------------------------------------------------*/


// resize image at desired scale
Image Image::GetImage(const PlatformArr& platforms, double scale, bool bUseImage) const
{
	Image scaledImage(*this);
	const cv::Size scaledSize(Image8U::computeResize(GetSize(), scale));
	scaledImage.width = (unsigned)scaledSize.width;
	scaledImage.height = (unsigned)scaledSize.height;
	scaledImage.camera = GetCamera(platforms, scaledImage.GetSize());
	if (!image.empty()) {
		if (bUseImage)
			cv::resize(image, scaledImage.image, scaledSize, 0, 0, scale>1?cv::INTER_CUBIC:cv::INTER_AREA);
		else
			scaledImage.image.release();
	}
	return scaledImage;
} // GetImage
// compute the camera extrinsics from the platform pose and the relative camera pose to the platform
Camera Image::GetCamera(const PlatformArr& platforms, const Image8U::Size& resolution) const
{
	ASSERT(platformID != NO_ID);
	ASSERT(cameraID != NO_ID);
	ASSERT(poseID != NO_ID);

	// compute the normalized absolute camera pose
	const Platform& platform = platforms[platformID];
	Camera camera(platform.GetCamera(cameraID, poseID));

	// compute the unnormalized camera
	camera.K = camera.GetK<REAL>(resolution.width, resolution.height);
	camera.ComposeP();

	return camera;
} // GetCamera
void Image::UpdateCamera(const PlatformArr& platforms)
{
	camera = GetCamera(platforms, Image8U::Size(width, height));
} // UpdateCamera
// computes camera's field of view for the given direction
REAL Image::ComputeFOV(int dir) const
{
	switch (dir) {
	case 0: // width
		return 2*ATAN(REAL(width)/(camera.K(0,0)*2));
	case 1: // height
		return 2*ATAN(REAL(height)/(camera.K(1,1)*2));
	case 2: // diagonal
		return 2*ATAN(SQRT(REAL(SQUARE(width)+SQUARE(height)))/(camera.K(0,0)+camera.K(1,1)));
	}
	return 0;
} // ComputeFOV
/*----------------------------------------------------------------*/


// stereo-rectify the image pair
//  - input leftPoints and rightPoints contains the pairs of corresponding image projections and their depth (optional)
//  - H converts a pixel from original to rectified left-image
//  - Q converts [x' y' disparity 1] in rectified coordinates to [x*z y*z z 1]*w in original image coordinates
//    where disparity=x1-x2
bool Image::StereoRectifyImages(const Image& image1, const Image& image2, const Point3fArr& points1, const Point3fArr& points2, Image8U3& rectifiedImage1, Image8U3& rectifiedImage2, Image8U& mask1, Image8U& mask2, Matrix3x3& H, Matrix4x4& Q)
{
	ASSERT(image1.IsValid() && image2.IsValid());
	ASSERT(image1.GetSize() == image1.image.size() && image2.GetSize() == image2.image.size());
	ASSERT(points1.size() && points2.size());

	#if 0
	{	// display projection pairs
		std::vector<Point2f> matches1, matches2;
		FOREACH(i, points1) {
			matches1.emplace_back(reinterpret_cast<const Point2f&>(points1[i]));
			matches2.emplace_back(reinterpret_cast<const Point2f&>(points2[i]));
		}
		RECTIFY::DrawMatches(const_cast<Image8U3&>(image1.image), const_cast<Image8U3&>(image2.image), matches1, matches2);
	}
	#endif

	// compute rectification
	Matrix3x3 K1, K2, R1, R2;
	#if 0
	const REAL t(Camera::StereoRectify(image1.GetSize(), image1.camera, image2.GetSize(), image2.camera, R1, R2, K1, K2));
	#elif 1
	const REAL t(Camera::StereoRectifyFusiello(image1.GetSize(), image1.camera, image2.GetSize(), image2.camera, R1, R2, K1, K2));
	#else
	Pose pose;
	ComputeRelativePose(image1.camera.R, image1.camera.C, image2.camera.R, image2.camera.C, pose.R, pose.C);
	cv::Mat P1, P2;
	cv::stereoRectify(image1.camera.K, cv::noArray(), image2.camera.K, cv::noArray(), image1.GetSize(), pose.R, Vec3(pose.GetTranslation()), R1, R2, P1, P2, Q, 0/*cv::CALIB_ZERO_DISPARITY*/, -1);
	K1 = P1(cv::Rect(0,0,3,3));
	K2 = P2(cv::Rect(0,0,3,3));
	const Point3 _t(R2 * pose.GetTranslation());
	ASSERT((ISZERO(_t.x) || ISZERO(_t.y)) && ISZERO(_t.z));
	const REAL t(ISZERO(_t.x)?_t.y:_t.x);
	#if 0
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(image1.camera.K, cv::noArray(), R1, K1, image1.GetSize(), CV_16SC2, map1, map2);
	cv::remap(image1.image, rectifiedImage1, map1, map2, cv::INTER_CUBIC);
	cv::initUndistortRectifyMap(image2.camera.K, cv::noArray(), R2, K2, image1.GetSize(), CV_16SC2, map1, map2);
	cv::remap(image2.image, rectifiedImage2, map1, map2, cv::INTER_CUBIC);
	return;
	#endif
	#endif
	if (ISZERO(t))
		return false;

	// adjust rectified camera matrices such that the entire area common to both source images is contained in the rectified images
	cv::Size size1(image1.GetSize()), size2(image2.GetSize());
	if (!points1.empty())
		Camera::SetStereoRectificationROI(points1, size1, image1.camera, points2, size2, image2.camera, R1, R2, K1, K2);
	ASSERT(size1 == size2);

	// compute rectification homography (from original to rectified image)
	const Matrix3x3 H1(K1 * R1 * image1.camera.GetInvK()); H = H1;
	const Matrix3x3 H2(K2 * R2 * image2.camera.GetInvK());

	#if 0
	{	// display epipolar lines before and after rectification
		Pose pose;
		ComputeRelativePose(image1.camera.R, image1.camera.C, image2.camera.R, image2.camera.C, pose.R, pose.C);
		const Matrix3x3 F(CreateF(pose.R, pose.C, image1.camera.K, image2.camera.K));
		std::vector<Point2f> matches1, matches2;
		#if 1
		FOREACH(i, points1) {
			matches1.emplace_back(reinterpret_cast<const Point2f&>(points1[i]));
			matches2.emplace_back(reinterpret_cast<const Point2f&>(points2[i]));
		}
		#endif
		RECTIFY::DrawRectifiedImages(image1.image.clone(), image2.image.clone(), F, H1, H2, matches1, matches2);
	}
	#endif

	// rectify images (apply homographies)
	rectifiedImage1.create(size1);
	cv::warpPerspective(image1.image, rectifiedImage1, H1, rectifiedImage1.size());
	rectifiedImage2.create(size2);
	cv::warpPerspective(image2.image, rectifiedImage2, H2, rectifiedImage2.size());

	// mark valid regions covered by the rectified images
	struct Compute {
		static void Mask(Image8U& mask, const cv::Size& sizeh, const cv::Size& size, const Matrix3x3& H) {
			mask.create(sizeh);
			mask.memset(0);
			std::vector<Point2f> corners(4);
			corners[0] = Point2f(0,0);
			corners[1] = Point2f((float)size.width,0);
			corners[2] = Point2f((float)size.width,(float)size.height);
			corners[3] = Point2f(0,(float)size.height);
			cv::perspectiveTransform(corners, corners, H);
			std::vector<std::vector<Point2i>> contours(1);
			for (int i=0; i<4; ++i)
				contours.front().emplace_back(ROUND2INT(corners[i]));
			cv::drawContours(mask, contours, 0, cv::Scalar(255), cv::FILLED);
		}
	};
	Compute::Mask(mask1, size1, image1.GetSize(), H1);
	Compute::Mask(mask2, size2, image2.GetSize(), H2);

	// from the formula that relates disparity to depth as z=B*f/d where B=-t and d=x_l-x_r
	// and the formula that converts the image projection from right to left x_r=K1*K2.inv()*x_l
	// compute the inverse projection matrix that transforms image coordinates in image 1 and its
	// corresponding disparity value to the 3D point in camera 1 coordinates as:
	ASSERT(ISEQUAL(K1(1,1),K2(1,1)));
	Q = Matrix4x4::ZERO;
	//   Q * [x, y, disparity, 1] = [X, Y, Z, 1] * w
	ASSERT(ISEQUAL(K1(0,0),K2(0,0)) && ISZERO(K1(0,1)) && ISZERO(K2(0,1)));
	Q(0,0) = Q(1,1) = REAL(1);
	Q(0,3) = -K1(0,2);
	Q(1,3) = -K1(1,2);
	Q(2,3) =  K1(0,0);
	Q(3,2) = -REAL(1)/t;
	Q(3,3) =  (K1(0,2)-K2(0,2))/t;

	// compute Q that converts disparity from rectified to depth in original image
	Matrix4x4 P(Matrix4x4::IDENTITY);
	cv::Mat(image1.camera.K*R1.t()).copyTo(cv::Mat(4,4,cv::DataType<Matrix4x4::Type>::type,P.val)(cv::Rect(0,0,3,3)));
	Q = P*Q;
	return true;
}

// adjust H and Q such that they correspond to rectified images scaled by the given factor
void Image::ScaleStereoRectification(Matrix3x3& H, Matrix4x4& Q, REAL scale)
{
	{
		Matrix3x3 S(Matrix3x3::IDENTITY);
		S(0,0) = S(1,1) = scale;
		H = S*H*S.inv();
	}
	{
		Matrix4x4 Sinv(Matrix4x4::IDENTITY);
		Sinv(0,0) = Sinv(1,1) = Sinv(2,2) = REAL(1)/scale;
		Matrix4x4 S(Matrix4x4::IDENTITY);
		S(0,0) = S(1,1) = scale*scale;
		S(2,2) = S(3,3) = scale;
		Q = S*Q*Sinv;
	}
}


// note: disparity has to be inverted as the formula is z=B*f/d where d=x_l-x_r
//       while we store d as the value that fulfills x_l+d=x_r
//
// converts the given disparity at the rectified image coordinates to
// the depth corresponding to the un-rectified image coordinates
template <typename TYPE>
float TDisparity2Depth(const Matrix4x4& Q, const TPoint2<TYPE>& u, float d)
{
	const REAL w(Q(3,0)*u.x + Q(3,1)*u.y - Q(3,2)*d + Q(3,3));
	if (ISZERO(w))
		return 0;
	const REAL z(Q(2,0)*u.x + Q(2,1)*u.y - Q(2,2)*d + Q(2,3));
	const float depth((float)(z/w));
	return depth < ZEROTOLERANCE<float>() ? float(0) : depth;
}
float Image::Disparity2Depth(const Matrix4x4& Q, const ImageRef& u, float d)
{
	return TDisparity2Depth(Q, u, d);
}
float Image::Disparity2Depth(const Matrix4x4& Q, const Point2f& u, float d)
{
	return TDisparity2Depth(Q, u, d);
}
// same as above, but converts also the coordinates from rectified to original image
template <typename TYPE>
float TDisparity2Depth(const Matrix4x4& Q, const TPoint2<TYPE>& u, float d, Point2f& pt)
{
	const REAL w(Q(3,0)*u.x + Q(3,1)*u.y - Q(3,2)*d + Q(3,3));
	if (ISZERO(w))
		return 0;
	const REAL z((Q(2,0)*u.x + Q(2,1)*u.y - Q(2,2)*d + Q(2,3))/w);
	if (z < ZEROTOLERANCE<REAL>())
		return 0;
	const REAL nrm(REAL(1)/(w*z));
	pt.x = (float)((Q(0,0)*u.x + Q(0,1)*u.y - Q(0,2)*d + Q(0,3))*nrm);
	pt.y = (float)((Q(1,0)*u.x + Q(1,1)*u.y - Q(1,2)*d + Q(1,3))*nrm);
	return (float)z;
}
float Image::Disparity2Depth(const Matrix4x4& Q, const ImageRef& u, float d, Point2f& pt)
{
	return TDisparity2Depth(Q, u, d, pt);
}
float Image::Disparity2Depth(const Matrix4x4& Q, const Point2f& u, float d, Point2f& pt)
{
	return TDisparity2Depth(Q, u, d, pt);
}
// converts the given disparity at the rectified image coordinates into
// the distance from the camera position to the corresponding 3D point;
// note: the distance is the same in both rectified and un-rectified cameras
//  - K is the camera matrix of the un-rectified image
float Image::Disparity2Distance(const Matrix3x3& K, const Matrix4x4& Q, const Point2f& u, float d)
{
	Point2f pt;
	const float depth(Disparity2Depth(Q, u, d, pt));
	return (float)(SQRT(SQUARE(pt.x-K(0,2))+SQUARE(pt.y-K(1,2))+SQUARE(K(0,0)))*depth/K(0,0));
}
// converts the given depth at the un-rectified image coordinates to
// the disparity corresponding to the rectified image coordinates
bool Image::Depth2Disparity(const Matrix4x4& Q, const Point2f& u, float d, float& disparity)
{
	const REAL w((Q(3,0)*u.x + Q(3,1)*u.y + Q(3,2))*d + Q(3,3));
	if (ISZERO(w))
		return false;
	const REAL z((Q(2,0)*u.x + Q(2,1)*u.y + Q(2,2))*d + Q(2,3));
	disparity = -(float)(z/w);
	return true;
}
/*----------------------------------------------------------------*/
