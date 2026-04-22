////////////////////////////////////////////////////////////////////
// Camera.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_CAMERA_H_
#define _SFM_CAMERA_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

typedef uint32_t IIndex;
typedef CLISTDEF0IDX(IIndex, IIndex) IIndexArr;

class SFM_API Camera;

enum class CameraType : uint8_t {
	UNDEFINED = 0,
	PINHOLE = 1,
	SPHERICAL = 2
};
// Convert between CameraType and string
inline CameraType CameraTypeFromString(const String& str) {
	if (str == "Pinhole") return CameraType::PINHOLE;
	if (str == "Spherical") return CameraType::SPHERICAL;
	return CameraType::UNDEFINED;
} // FromString
inline String CameraTypeToString(const CameraType type) {
	switch (type) {
	case CameraType::PINHOLE: return "Pinhole";
	case CameraType::SPHERICAL: return "Spherical";
	default: return "Undefined";
	}
} // ToString
/*----------------------------------------------------------------*/


// Base camera class defining the interface for all camera types
// Following MVS convention: the world and camera coordinate system is right handed,
// with x pointing right, y pointing down, and z pointing forward
class SFM_API Camera
{
public:
	// Image resolution
	cv::Size size;

	// Optional metadata
	struct Metadata {
		String name;           // camera name/identifier
		String model;          // camera model
		REAL sensorWidth{0};   // sensor width in mm (0 if unknown)
		REAL sensorHeight{0};  // sensor height in mm (0 if unknown)
	};
	Metadata metadata;

public:
	Camera() : size(0, 0) {}
	Camera(const cv::Size& _size) : size(_size) {
		ASSERT(_size.width > 0 && _size.height > 0);
	}
	virtual ~Camera() {}

	// Metadata setters
	inline void SetName(const String& n) { metadata.name = n; }
	inline void SetModel(const String& m) { metadata.model = m; }
	inline void SetSensorSize(REAL w_mm, REAL h_mm) { metadata.sensorWidth = w_mm; metadata.sensorHeight = h_mm; }

	// Pure virtual methods that must be implemented by derived classes

	// Clone the camera (for polymorphic copying)
	virtual Camera* Clone() const = 0;

	// Project a 3D point in camera coordinates to 2D image coordinates;
	// returns projected point and a bool indicating if point is valid (ex. in front of camera for pinhole)
	virtual std::pair<Point2, bool> Project(const Point3& X) const = 0;

	// Unproject a 2D image point to a 3D point on the bearing ray in camera coordinates (w/ & w/o normalization)
	virtual Point3 Unproject(const Point2& x) const = 0;
	virtual Point3 UnprojectNormalized(const Point2& x) const = 0;

	// Get camera type
	virtual CameraType GetType() const = 0;

	// Get the intrinsics, like K matrix, focal-length and principal-point (if applicable)
	virtual KMatrix GetK() const = 0;
	virtual REAL GetFocalLength() const {
		KMatrix K = GetK();
		return (K(0, 0) + K(1, 1)) * REAL(0.5);
	}
	virtual Point2 GetPrincipalPoint() const {
		KMatrix K = GetK();
		return Point2(K(0, 2), K(1, 2));
	}

	// Trust intrinsics validity
	virtual bool TrustIntrinsics() const = 0;

	// Check if camera has valid parameters
	virtual bool IsValid() const { return !size.empty(); }

	// Check if camera supports distortion and has valid distortion parameters
	virtual bool HasDistortion() const { return false; }

	// Get image size
	inline const cv::Size& GetSize() const { return size; }
	inline int GetWidth() const { return size.width; }
	inline int GetHeight() const { return size.height; }
	inline float GetAspectRatio() const {
		return size.width > size.height ?
			(float)size.width / (float)size.height :
			(float)size.height / (float)size.width;
	}
	inline float GetNormalizationScale() const {
		ASSERT(size.width > 0 && size.height > 0);
		return float(MAXF(size.width, size.height));
	}

	// Format intrinsic parameters as a human-readable string (for logging)
	virtual String GetIntrinsicsString() const = 0;

	// Accumulate intrinsic parameters from another camera of the same type
	virtual void AccumulateIntrinsics(const Camera& other) = 0;

	// Scale all intrinsic parameters by a factor (used to finalize averaging)
	virtual void ScaleIntrinsics(REAL factor) = 0;

	// Reset all intrinsic parameters to zero (used to start accumulation)
	virtual void ResetIntrinsics() = 0;

	// Convert pixel-based error threshold to angular threshold (radians)
	// This accounts for image resolution and (for pinhole) focal length
	virtual REAL PixelErrorToAngular(REAL pixelError) const = 0;

	// Relative feature localization noise compared to a baseline pinhole camera.
	// Returns a multiplier on pixel-based reprojection thresholds that callers
	// (PnP RANSAC, match filters, etc.) should apply when a single global pixel
	// threshold is tuned for pinhole but the camera model produces noisier
	// feature positions. Default 1 (pinhole baseline); overridden per model.
	virtual REAL GetFeatureNoiseScale() const { return REAL(1); }

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & size.width & size.height;
		ar & metadata.name & metadata.model;
		ar & metadata.sensorWidth & metadata.sensorHeight;
	}
	#endif
};

typedef Camera* CameraPtr;
typedef SEACAVE::cList<CameraPtr, CameraPtr, 0, 4, IIndex> CameraPtrArr;
/*----------------------------------------------------------------*/


// Pinhole camera model with radial and tangential distortion
class SFM_API PinholeCamera : public Camera
{
public:
	// Intrinsic parameters
	REAL fx, fy;  // focal length
	REAL cx, cy;  // principal point

	// Distortion parameters (Brown-Conrady model)
	REAL k1, k2, k3;  // radial distortion
	REAL p1, p2;      // tangential distortion
	REAL k4, k5, k6;  // additional radial distortion (optional, use only if enabled)

	// Control flag for additional distortion
	bool useAdditionalDistortion;  // enable k4, k5, k6 (default: false)

	// Trust intrinsics validity
	bool trustIntrinsics;

public:
	PinholeCamera()
		: fx(0), fy(0), cx(0), cy(0),
		  k1(0), k2(0), k3(0), p1(0), p2(0), k4(0), k5(0), k6(0),
		  useAdditionalDistortion(false) {}

	PinholeCamera(const cv::Size& _size)
		: Camera(_size), fx(0), fy(0), cx(0), cy(0),
		  k1(0), k2(0), k3(0), p1(0), p2(0), k4(0), k5(0), k6(0),
		  useAdditionalDistortion(false) {}

	PinholeCamera(const cv::Size& _size, REAL _fx, REAL _fy, REAL _cx, REAL _cy)
		: Camera(_size), fx(_fx), fy(_fy), cx(_cx), cy(_cy),
		  k1(0), k2(0), k3(0), p1(0), p2(0), k4(0), k5(0), k6(0),
		  useAdditionalDistortion(false) {}

	virtual ~PinholeCamera() {}

	// Clone the camera
	virtual Camera* Clone() const override {
		return new PinholeCamera(*this);
	}

	// Project 3D point to 2D with distortion
	virtual std::pair<Point2, bool> Project(const Point3& X) const override;

	// Unproject 2D point to 3D ray (inverse of undistorted projection, z=1)
	virtual Point3 Unproject(const Point2& x) const override;
	virtual Point3 UnprojectNormalized(const Point2& x) const override;

	virtual CameraType GetType() const override { return CameraType::PINHOLE; }

	// Get/set intrinsic matrix K
	virtual KMatrix GetK() const override;
	void SetK(const KMatrix& K);

	// Trust intrinsics validity
	virtual bool TrustIntrinsics() const override {
		return trustIntrinsics;
	}

	// Check if camera has valid parameters
	virtual bool IsValid() const override {
		return Camera::IsValid() && fx > 0 && fy > 0;
	}

	// Check if distortion is valid
	virtual bool HasDistortion() const override {
		return k1 != 0 || k2 != 0 || k3 != 0 || p1 != 0 || p2 != 0;
	}
	inline bool HasAdditionalDistortion() const {
		return useAdditionalDistortion && (k4 != 0 || k5 != 0 || k6 != 0);
	}

	// Intrinsics setter (optional helper)
	inline void SetIntrinsics(REAL _fx, REAL _fy, REAL _cx, REAL _cy) { fx = _fx; fy = _fy; cx = _cx; cy = _cy; }

	// Distortion setter (optional helper)
	inline void SetDistortion(REAL _k1, REAL _k2, REAL _p1, REAL _p2, REAL _k3 = 0) {
		k1 = _k1; k2 = _k2; p1 = _p1; p2 = _p2; k3 = _k3;
	}

	// Get distortion coefficients as OpenCV format
	cv::Mat GetDistortionCoeffs() const;

	// Format intrinsic parameters as a human-readable string
	virtual String GetIntrinsicsString() const override {
		String str = String::FormatString("fx %.2f, fy %.2f, cx %.2f, cy %.2f", fx, fy, cx, cy);
		if (HasDistortion())
			str += String::FormatString(", k1 %.4g, k2 %.4g, k3 %.4g, p1 %.4g, p2 %.4g", k1, k2, k3, p1, p2);
		if (HasAdditionalDistortion())
			str += String::FormatString(", k4 %.4g, k5 %.4g, k6 %.4g", k4, k5, k6);
		return str;
	}

	// Accumulate intrinsic parameters from another PinholeCamera
	virtual void AccumulateIntrinsics(const Camera& other) override {
		const PinholeCamera& o = static_cast<const PinholeCamera&>(other);
		fx += o.fx; fy += o.fy; cx += o.cx; cy += o.cy;
		k1 += o.k1; k2 += o.k2; k3 += o.k3;
		p1 += o.p1; p2 += o.p2;
		k4 += o.k4; k5 += o.k5; k6 += o.k6;
	}

	// Scale all intrinsic parameters
	virtual void ScaleIntrinsics(REAL factor) override {
		fx *= factor; fy *= factor; cx *= factor; cy *= factor;
		k1 *= factor; k2 *= factor; k3 *= factor;
		p1 *= factor; p2 *= factor;
		k4 *= factor; k5 *= factor; k6 *= factor;
	}

	// Reset all intrinsic parameters to zero
	virtual void ResetIntrinsics() override {
		fx = fy = cx = cy = 0;
		k1 = k2 = k3 = p1 = p2 = k4 = k5 = k6 = 0;
	}

	// Convert pixel error to angular error (radians)
	virtual REAL PixelErrorToAngular(REAL pixelError) const override;

	// Compute maximum pixel distortion magnitude across the image
	// Returns the maximum distance (in pixels) between distorted and undistorted positions
	// sampleDensity controls sampling grid resolution (default: 20x20)
	REAL ComputeMaxDistortion(int sampleDensity = 20) const;

	// Helper function: apply distortion to normalized coordinates
	// Input: undistorted normalized coordinates (x, y)
	// Output: distorted normalized coordinates
	Point2 Distort(const Point2& p) const;

	// Helper function: remove distortion from normalized coordinates (iterative)
	// Input: distorted normalized coordinates (x, y)
	// Output: undistorted normalized coordinates
	Point2 Undistort(const Point2& p) const;

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Camera>(*this);
		ar & fx & fy & cx & cy;
		ar & k1 & k2 & k3 & p1 & p2 & k4 & k5 & k6;
		ar & useAdditionalDistortion;
		ar & trustIntrinsics;
	}
	#endif
};
/*----------------------------------------------------------------*/


// Spherical camera model for equirectangular (360 degree) images
class SFM_API SphericalCamera : public Camera
{
public:
	SphericalCamera() {}

	SphericalCamera(const cv::Size& _size) : Camera(_size) {
		// Equirectangular images must cover 360x180 degrees, so width = 2 * height.
		ASSERT(_size.width > 0 && _size.width == 2 * _size.height);
	}

	virtual ~SphericalCamera() {}

	// Clone the camera
	virtual Camera* Clone() const override {
		return new SphericalCamera(*this);
	}

	// Project 3D point to 2D using equirectangular projection
	virtual std::pair<Point2, bool> Project(const Point3& X) const override;

	// Unproject 2D point to 3D ray using equirectangular projection;
	// Unproject returns a Point3 scaled so that |z|=1
	Point2 MapImageToSpherical(const Point2& x) const;
	virtual Point3 Unproject(const Point2& x) const override;
	virtual Point3 UnprojectNormalized(const Point2& x) const override;

	virtual CameraType GetType() const override { return CameraType::SPHERICAL; }

	// Spherical cameras don't have a traditional K matrix
	virtual KMatrix GetK() const override { return KMatrix::IDENTITY; }

	// Trust intrinsics validity
	virtual bool TrustIntrinsics() const override { return true; }

	// Format intrinsic parameters as a human-readable string
	virtual String GetIntrinsicsString() const override {
		return String::FormatString("spherical %dx%d", size.width, size.height);
	}

	// SphericalCamera has no intrinsic parameters to accumulate
	virtual void AccumulateIntrinsics(const Camera& /*other*/) override {}
	virtual void ScaleIntrinsics(REAL /*factor*/) override {}
	virtual void ResetIntrinsics() override {}

	// Convert pixel error to angular error (radians)
	virtual REAL PixelErrorToAngular(REAL pixelError) const override;

	// Cube-face SIFT extraction produces features with ~2x the pixel-space
	// localization noise of a pinhole pipeline (face-seam sampling plus
	// off-center descriptor warping), so pixel-calibrated thresholds widen
	// by this factor for the equirectangular model.
	virtual REAL GetFeatureNoiseScale() const override { return REAL(2); }

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & boost::serialization::base_object<Camera>(*this);
	}
	#endif
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_CAMERA_H_
