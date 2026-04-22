////////////////////////////////////////////////////////////////////
// Image.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_IMAGE_H_
#define _SFM_IMAGE_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "View.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Stores metrics describing how well the reference view is connected to a neighbor view
struct SFM_API ViewScore {
	uint32_t ID;       // image ID of the neighbor view
	uint32_t points;   // number of shared tracks between views
	float angle;       // average angle between viewing rays (radians)
	float area;        // overlap area ratio (fraction of reference image covered by shared points) [0-1]
};
typedef CLISTDEF0IDX(ViewScore, uint32_t) ViewScoreArr;
/*----------------------------------------------------------------*/


// Image class manages per-image data including pixels, features, descriptors, and view
class SFM_API Image : public View
{
public:
	IIndex ID;             // unique image ID
	String fileName;       // image file path (relative or absolute)
	double timestamp;      // timestamp in seconds (from video or capture time)

	// Pixel data (loaded on demand)
	cv::Mat pixels;        // image pixels (can be empty if not loaded)

	// Feature data
	std::vector<cv::KeyPoint> keypoints;  // detected keypoints
	cv::Mat descriptors;                  // feature descriptors (one row per keypoint)

	// Additional metadata
	struct Metadata {
		String name;               // optional descriptive name
		String dateTimeOriginal;   // capture datetime (if available)
		double exposureTime{0};    // exposure time in seconds (0 if unknown)
		uint16_t ISO{0};           // ISO sensitivity (0 if unknown)
		uint16_t orientation{1};   // EXIF orientation tag (default 1)
	};
	Metadata metadata;

public:
	Image() : ID(NO_ID), timestamp(0) {}

	Image(IIndex _ID, const String& _fileName, double _timestamp = 0)
		: ID(_ID), fileName(_fileName), timestamp(_timestamp) {}

	Image(IIndex _ID, const String& _fileName, const Pose3D& pose, IIndex _cameraID, CameraPtr _pCamera, double _timestamp = 0)
		: View(pose, _cameraID, _pCamera), ID(_ID), fileName(_fileName), timestamp(_timestamp) {}

	// Check if image has loaded pixels
	inline bool HasPixels() const { return !pixels.empty(); }

	// Check if image has features
	inline bool HasFeatures() const { return !keypoints.empty(); }

	// Check if image has descriptors
	inline bool HasDescriptors() const { return !descriptors.empty(); }

	// Load EXIF metadata and initialize view camera (does not decode pixels)
	//  - defaultFocalRatio: default focal length to image width ratio if EXIF data is missing
	bool LoadMetadata(float defaultFocalRatio = 1.2f);

	// Load image pixels from file
	//  - gray: load image as grayscale if true, otherwise as color
	bool LoadPixels(bool gray = false);

	// Save image pixels to stored file (uses format from fileName extension or JXL if no extension)
	bool SavePixels() const;

	// Release image pixels to free memory
	void ReleasePixels() { pixels.release(); }

	// Return a BGR/8U view of the image pixels suitable for pipeline stages
	// that expect CV_8UC3. If pixels are already BGR/8U the returned Image8U3
	// shares OpenCV's ref-counted buffer (no copy); otherwise grayscale/BGRA/
	// floating-point inputs are converted into a freshly-allocated buffer.
	Image8U3 GetImage8U3() const;

	// Select top keypoints/descriptors using grid-based spatial distribution and keypoint response
	//  - maxKeypoints: maximum number of keypoints to select
	// Returns vector of indices into keypoints/descriptors arrays
	UnsignedArr SelectTopKeypoints(unsigned maxKeypoints) const;

	// Scoring Strategy: Weighted Stability (Response + Size)
	// Incorporates feature size alongside response to improve SfM geometric stability.
	// - Large Features (>20px): Prioritized as they represent major structural elements
	// (e.g., window corners) that survive downsampling and large viewpoint changes.
	// - Small Features (2-3px): Penalized even if response is high, as they often match
	// transient high-contrast noise or textures (e.g., leaves) that disappear when the camera moves.
	// This scoring strategy enhances the selection of robust features for SfM tasks.
	//  - kp: input keypoint
	//  - minResponse: minimum response threshold (below which weight=0)
	// Returns computed keypoint weight, in range [0,1.4]
	static float ComputeKeypointWeight(const cv::KeyPoint& kp, float minResponse = 0);
	// Scoring Strategy: Precision Estimation
	// Estimates the precision (inverse variance) of a keypoint based on its response and size.
	// This metric helps prioritize features that are both reliable (high response)
	// and precise (small size) for accurate geometric computations in SfM.
	//  - kp: input keypoint
	//  - minResponse: minimum response threshold (below which precision=0)
	// Returns estimated keypoint precision, in range [0,1]
	static float ComputeKeypointPrecision(const cv::KeyPoint& kp, float minResponse = 0);

    #ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const {
		ar & ID;
		const String relFileName = MAKE_PATH_REL(WORKING_FOLDER_FULL, fileName);
		ar & relFileName;
		ar & timestamp;
		ar & boost::serialization::base_object<View>(*this);
		ar & metadata.name;
		ar & metadata.dateTimeOriginal;
		ar & metadata.exposureTime;
		ar & metadata.ISO;
		ar & metadata.orientation;
		ar & keypoints;
		ar & descriptors;
	}
	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/) {
		ar & ID;
		ar & fileName;
		fileName = MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName);
		ar & timestamp;
		ar & boost::serialization::base_object<View>(*this);
		ar & metadata.name;
		ar & metadata.dateTimeOriginal;
		ar & metadata.exposureTime;
		ar & metadata.ISO;
		ar & metadata.orientation;
		ar & keypoints;
		ar & descriptors;
	}
	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
};

typedef CLISTDEF2IDX(Image, IIndex) ImageArr;
/*----------------------------------------------------------------*/


// Helper: convert vector of KeyPoints to vector of Point2f
inline std::vector<Point2f> ConvertToPoints(const std::vector<cv::KeyPoint>& keypoints)
{
	std::vector<Point2f> points;
	points.reserve(keypoints.size());
	for (const auto& kp : keypoints)
		points.emplace_back(kp.pt.x, kp.pt.y);
	return points;
}
// Helper: convert vector of Point2f to vector of KeyPoints
inline std::vector<cv::KeyPoint> ConvertToKeypoints(const std::vector<Point2f>& points)
{
	std::vector<cv::KeyPoint> keypoints;
	keypoints.reserve(points.size());
	for (const auto& pt : points)
		keypoints.emplace_back(pt.x, pt.y, 1.f);
	return keypoints;
}

// Export/Import all image poses to/from a human-readable CSV.
// Schema per row: filename,fx,fy,cx,cy,qx,qy,qz,qw,Cx,Cy,Cz,score
//  - fileName: poses CSV file name for export/import
//  - images: array of images to export/import poses
//  - mode: flags for importing camera poses from CSV: 0=all, 1=extrinsics only, 2=positions only
// returns number of valid image poses exported/imported
SFM_API unsigned ExportPosesCSV(const String& fileName, const ImageArr& images);
SFM_API unsigned ImportPosesCSV(const String& fileName, ImageArr& images, unsigned mode = 0);

// Estimates image blur using a robust multi-scale variance-of-Laplacian focus measure.
// The function internally converts to grayscale, normalizes intensity to [0,1],
// evaluates Laplacian energy over an image pyramid (3 levels or until <64px),
// returns sharpness (smaller = blurrier)
SFM_API float EstimateImageSharpness(const cv::Mat& pixels);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_IMAGE_H_

