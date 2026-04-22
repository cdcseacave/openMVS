////////////////////////////////////////////////////////////////////
// View.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_VIEW_H_
#define _SFM_VIEW_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Pose.h"
#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// View represents a specific viewpoint (pose) with an associated camera
// It inherits from Pose3D and adds a reference to a Camera object
// Cameras can be shared between views if taken with the same physical camera
class SFM_API View : public Pose3D
{
public:
	IIndex cameraID;  // ID of the associated camera
	CameraPtr pCamera;  // pointer to the camera (can be shared)

	// Optional metadata associated with this view (pose + geolocation + orientation)
	struct Metadata {
		REAL positionAccuracy{0};   // horizontal accuracy (m)
		REAL positionAccuracyZ{0};  // vertical accuracy (m)
		REAL rotationAccuracy{0};   // rotation accuracy (deg)
		REAL latitude{0};           // WGS84 latitude (deg)
		REAL longitude{0};          // WGS84 longitude (deg)
		REAL altitude{0};           // altitude above sea level (m)
		REAL yawDeg{0};             // yaw/heading (deg)
		REAL pitchDeg{0};           // pitch (deg)
		REAL rollDeg{0};            // roll (deg)

		bool rotated{false};        // true when image was rotated 90deg clockwise on load

		inline bool HasGPS() const { return latitude != 0 || longitude != 0 || altitude != 0; }
	};
	Metadata metadata;

public:
	View()
		: Pose3D(RMatrix::IDENTITY, CMatrix::INF), cameraID(NO_ID), pCamera(NULL) {}

	View(IIndex _cameraID, CameraPtr _pCamera)
		: Pose3D(RMatrix::IDENTITY, CMatrix::INF), cameraID(_cameraID), pCamera(_pCamera) {}

	View(const Pose3D& pose, IIndex _cameraID, CameraPtr _pCamera)
		: Pose3D(pose), cameraID(_cameraID), pCamera(_pCamera) {}

	~View() { InvalidateCamera(); }

	// Check if view has a valid camera
	inline bool HasCamera() const {
		return pCamera != NULL && pCamera->IsValid();
	}
	inline void InvalidateCamera() {
		if (cameraID == NO_ID && pCamera)
			delete pCamera;
		else
			cameraID = NO_ID;
		pCamera = NULL;
	}

	// Check if view is valid (has valid pose and camera)
	inline bool IsValid() const {
		return HasCamera() && HasPose();
	}

	// Check if view has a pose
	inline bool HasPose() const {
		return Pose3D::C != CMatrix::INF;
	}
	inline void InvalidatePose() {
		Pose3D::C = CMatrix::INF;
	}

	// Get image dimensions
	inline int GetWidth() const { ASSERT(HasCamera()); return pCamera->GetWidth(); }
	inline int GetHeight() const { ASSERT(HasCamera()); return pCamera->GetHeight(); }
	inline cv::Size GetSize() const { return cv::Size(GetWidth(), GetHeight()); }
	inline float GetAspectRatio() const { ASSERT(HasCamera()); return pCamera->GetAspectRatio(); }
	inline float GetNormalizationScale() const { ASSERT(HasCamera()); return pCamera->GetNormalizationScale(); }

	// Image orientation helpers
	inline bool IsRotated() const { return metadata.rotated; }
	// Original on-disk resolution (portrait/landscape as stored)
	inline cv::Size GetOriginalSize() const { return IsRotated() ? cv::Size(GetHeight(), GetWidth()) : pCamera->GetSize(); }
	// Rotate a matrix to the working orientation (landscape if portrait)
	cv::Mat& ToWorkingOrientation(cv::Mat& mat) const;
	// Rotate a matrix back to the original on-disk orientation (no-op if not rotated)
	cv::Mat& ToOriginalOrientation(cv::Mat& mat) const;
	// Rotate point coordinates 90° CCW from working (landscape) to original (portrait) orientation
	Point2f ToOriginalOrientation(const Point2f& pw) const;
	// Rotate point coordinates 90° CW from original (portrait) to working (landscape) orientation
	Point2f ToWorkingOrientation(const Point2f& po) const;
	// Apply orientation correction to intrinsics (and optionally pose); returns original size
	cv::Size RevertRotation(Matrix3x3::Base* pK, Matrix3x3::Base* pR = NULL) const;

	// Get camera type
	CameraType GetCameraType() const {
		ASSERT(HasCamera());
		return pCamera->GetType();
	}

	// Get the intrinsic matrix K (if applicable)
	KMatrix GetK() const {
		ASSERT(HasCamera());
		return pCamera->GetK();
	}
	bool TrustIntrinsics() const {
		ASSERT(HasCamera());
		return pCamera->TrustIntrinsics();
	}

	// Compose/decompose projection matrix P = K*R*[I|-C]
	Matrix4x4 GetP4() const; // the composed projection matrix (4x4) assuming valid P
	PMatrix GetP() const; // compose P from K, R and C
	void DecomposeP(const PMatrix&); // decompose P in K, R and C

	// Project a 3D point in world coordinates to 2D image coordinates
	inline std::pair<Point2, bool> ProjectPoint(const Point3& X) const {
		ASSERT(HasCamera());
		// Transform from world to camera coordinates, then project
		const Point3 Xc = TransformPointW2C(X);
		return pCamera->Project(Xc);
	}
	inline std::pair<Point2, bool> ProjectPoint(const Point3& X, REAL& d) const {
		ASSERT(HasCamera());
		// Transform from world to camera coordinates, then project
		const Point3 Xc = TransformPointW2C(X);
		d = Xc.z;
		return pCamera->Project(Xc);
	}

	// Unproject a 2D image point and depth to a 3D point in world coordinates
	inline Point3 UnprojectPoint(const Point2& x, REAL d = REAL(1)) const {
		ASSERT(HasCamera());
		return TransformPointC2W(pCamera->Unproject(x) * d);
	}

	// Returns the ray from camera center through the given 2D point, in world coordinates
	inline Point3 Ray(const Point2& x) const {
		ASSERT(HasCamera());
		return RayCameraToWorld(pCamera->Unproject(x));
	}
	inline Point3 RayNormalized(const Point2& x) const {
		ASSERT(HasCamera());
		return RayCameraToWorld(pCamera->UnprojectNormalized(x));
	}

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Pose3D>(*this);
		ar & cameraID;
		ar & pCamera;
		ar & metadata.positionAccuracy;
		ar & metadata.positionAccuracyZ;
		ar & metadata.rotationAccuracy;
		ar & metadata.latitude;
		ar & metadata.longitude;
		ar & metadata.altitude;
		ar & metadata.yawDeg;
		ar & metadata.pitchDeg;
		ar & metadata.rollDeg;
		ar & metadata.rotated;
	}
	#endif
};

typedef SEACAVE::cList<View, const View&, 0, 16, uint32_t> ViewArr;
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_VIEW_H_

