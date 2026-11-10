/*
* Platform.h
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

#ifndef _MVS_PLATFORM_H_
#define _MVS_PLATFORM_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// the Brown lens of the image files of a camera: their camera matrix at their resolution and the
// distortion coefficients; the pixels are undistorted when they are read (Image::ReadPixels) to
// the pinhole camera UndistortedK(K, distortion, size), the one every consumer sees
struct MVS_API CameraLens {
	KMatrix K; // camera matrix of the distorted image files, at their resolution
	BrownDistortion<REAL> distortion{}; // lens distortion of the image files
	cv::Size size; // resolution of the image files
};

// a mobile platform with cameras attached to it
class MVS_API Platform
{
public:
	// structure describing a normalized camera mounted on a platform;
	// K is the camera matrix of the images as every consumer sees them:
	//  - PINHOLE: the image files as they are
	//  - BROWN: the undistorted images; lens describes the image files
	//  - SPHERICAL: equirectangular images, K is the angular camera matrix (Interface::Platform::Camera)
	struct MVS_API Camera : CameraIntern {
		CameraModelType model; // projection model of the image files
		CameraLens lens; // BROWN only

		inline Camera() : model(CameraModelType::PINHOLE) {}
		inline Camera(const Matrix3x3& _K, const Matrix3x3& _R, const Point3& _C) : CameraIntern(_K, _R, _C), model(CameraModelType::PINHOLE) {}

		// make it a BROWN camera of the image files of the given resolution and pixel camera matrix:
		// K becomes the normalized UndistortedK, the camera every consumer sees
		void SetBrownLens(const KMatrix& fileK, const BrownDistortion<REAL>& distortion, const cv::Size& size);

		#ifdef _USE_BOOST
		// implement BOOST serialization; the fields of CameraIntern are written here, not through its
		// base object, so the archive of a version 0 camera (a CameraIntern) reads as one
		template <class Archive>
		void serialize(Archive& ar, const unsigned int version) {
			ar & K;
			ar & R;
			ar & C;
			if (version > 0) {
				uint8_t modelID(static_cast<uint8_t>(model));
				ar & modelID;
				model = static_cast<CameraModelType>(modelID);
				if (model == CameraModelType::BROWN) {
					ar & lens.K;
					ar & lens.distortion.k1 & lens.distortion.k2 & lens.distortion.p1 & lens.distortion.p2;
					ar & lens.distortion.k3 & lens.distortion.k4 & lens.distortion.k5 & lens.distortion.k6;
					ar & lens.size.width & lens.size.height;
				}
			}
		}
		#endif
	};
	typedef CLISTDEFIDX(Camera,uint32_t) CameraArr;

	// structure describing a pose along the trajectory of a platform
	struct Pose {
		RMatrix R; // platform's rotation matrix
		CMatrix C; // platform's translation vector in the global coordinate system
		#ifdef _USE_BOOST
		template <class Archive>
		void serialize(Archive& ar, const unsigned int /*version*/) {
			ar & R;
			ar & C;
		}
		#endif
	};
	typedef CLISTDEF0IDX(Pose,uint32_t) PoseArr;

public:
	String name; // platform's name
	CameraArr cameras; // cameras mounted on the platform
	PoseArr poses; // trajectory of the platform

public:
	inline Platform() {}

	CameraIntern GetCamera(uint32_t cameraID, uint32_t poseID) const;

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & name;
		ar & cameras;
		ar & poses;
	}
	#endif
};
typedef CLISTDEFIDX(Platform,uint32_t) PlatformArr;
/*----------------------------------------------------------------*/

} // namespace MVS

#ifdef _USE_BOOST
// version 1: model and lens
BOOST_CLASS_VERSION(MVS::Platform::Camera, 1)
#endif

#endif // _MVS_PLATFORM_H_
