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
	// keypoints holds the described keypoints first -- the detector's own detections, one
	// descriptor row each -- and, past that prefix, the dense keypoints: ROMAv2 warp samples
	// appended by the dense supplementation pass, which carry no descriptor. So
	// keypoints.size() >= NumDescribedKeypoints() == descriptors.rows, and there is no separate
	// keypoint structure: a keypoint index alone says which kind it is (IsDenseKeypoint).
	std::vector<cv::KeyPoint> keypoints;  // described keypoints, then the appended dense ones
	cv::Mat descriptors;                  // feature descriptors (one row per described keypoint)
	// Where the described prefix ends; NO_ID until a dense keypoint is appended, which is the
	// same statement as "every keypoint is described".
	// Stored and serialized rather than derived from descriptors.rows, because the descriptors are
	// released before PairsMatcher::FilterRedundantKeypoints runs (that filter cannot remap
	// descriptor rows, which is why it only runs there) and long before bundle adjustment: a
	// boundary derived from them would report every keypoint as dense in exactly the two places
	// that need it. Read it through NumDescribedKeypoints()/IsDenseKeypoint(), never directly.
	uint32_t numDescribedKeypoints = NO_ID;

	// Optional 1xD CV_32F L2-normalized global retrieval descriptor of the whole image
	// (GeM-pooled DINOv3 features from the ROMAv2 describe graph); empty unless the
	// ROMAv2 retrieval pass ran, in which case it ranks the candidate pairs
	cv::Mat globalDescriptor;

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

	// Number of leading keypoints that carry a descriptor (see numDescribedKeypoints)
	inline uint32_t NumDescribedKeypoints() const {
		const uint32_t numDescribed = numDescribedKeypoints == NO_ID ? (uint32_t)keypoints.size() : numDescribedKeypoints;
		// while the descriptors are still alive they are the boundary, so the stored count must
		// agree with them; once they are released this is the only record of it left
		ASSERT(!HasDescriptors() || (uint32_t)descriptors.rows == numDescribed);
		ASSERT(numDescribed <= keypoints.size());
		return numDescribed;
	}
	// Check if any dense (descriptor-less) keypoint was appended past the described prefix
	inline bool HasDenseKeypoints() const { return numDescribedKeypoints != NO_ID; }
	// Number of dense (descriptor-less) keypoints appended past the described prefix
	inline uint32_t NumDenseKeypoints() const { return (uint32_t)keypoints.size() - NumDescribedKeypoints(); }
	// Check if the given keypoint index is a dense (descriptor-less) keypoint
	inline bool IsDenseKeypoint(uint32_t idx) const {
		ASSERT(idx < keypoints.size());
		return numDescribedKeypoints != NO_ID && idx >= numDescribedKeypoints;
	}
	// Close the described prefix at the current keypoint count, before dense keypoints are appended
	// past it. Idempotent: a second supplemented pair appending to the same image must not move a
	// boundary the first one already set.
	inline void CloseDescribedKeypoints() {
		if (numDescribedKeypoints == NO_ID)
			numDescribedKeypoints = (uint32_t)keypoints.size();
	}
	// Move the described prefix explicitly, after keypoints were removed from inside it
	// (PairsMatcher::FilterRedundantKeypoints' remap); only meaningful on an image that already
	// carries dense keypoints, since the boundary of one without them is its keypoint count
	inline void SetNumDescribedKeypoints(uint32_t numDescribed) {
		ASSERT(HasDenseKeypoints() && numDescribed <= keypoints.size());
		numDescribedKeypoints = numDescribed;
	}
	// Drop every feature, boundary included, so a re-extraction or re-import starts from a state
	// where every keypoint it produces is described
	inline void ReleaseFeatures() {
		keypoints.clear();
		descriptors.release();
		numDescribedKeypoints = NO_ID;
	}
	// Take over src's features, leaving it with none (the sub-scene extract/merge hand-off).
	// The three travel together and must: keypoints arriving without their boundary would report
	// every dense keypoint as described, and a boundary left behind on an image whose keypoints
	// are gone is a count larger than the array it indexes.
	inline void MoveFeaturesFrom(Image& src) {
		keypoints = std::move(src.keypoints);
		descriptors = std::move(src.descriptors);
		numDescribedKeypoints = src.numDescribedKeypoints;
		src.ReleaseFeatures();
	}
	// Copy src's features, leaving it untouched (a borrowed scratch view over the same keypoints).
	// Exists for the same reason as MoveFeaturesFrom: the boundary travels with the array it
	// indexes, so no caller has to remember to carry it by hand.
	inline void CopyFeaturesFrom(const Image& src) {
		keypoints = src.keypoints;
		descriptors = src.descriptors;
		numDescribedKeypoints = src.numDescribedKeypoints;
	}

	// Check if image has a global retrieval descriptor
	inline bool HasGlobalDescriptor() const { return !globalDescriptor.empty(); }

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
	// Returns vector of indices into keypoints/descriptors arrays; restricted to the described
	// prefix, since every caller pairs the returned index with a descriptor row
	UnsignedArr SelectTopKeypoints(unsigned maxKeypoints) const;

	// Build the keypoint standing for one dense (ROMAv2 warp) correspondence sample.
	// The response/size convention the weighting functions below then see:
	//  - response = the warp confidence at that sample, which is the only per-sample quality the
	//    warp offers; it lands in [ROMA2Config::minConfidence, 1], well above the response of a
	//    weak detector keypoint, so a dense point is never mistaken for a low-confidence one
	//  - size = warpCellSize, the pixel footprint of one warp cell in this image, i.e. the scale
	//    the position was actually sampled at. This is the honest number: it makes
	//    ComputeKeypointPrecision's 1/size^2 report a dense point as the less precise measurement
	//    it is, which is the same statement the bundle-adjustment down-weighting makes -- so only
	//    ONE of the two may apply to a residual, and SelectReprojectionLoss drops
	//    BAConfig::denseObservationWeight whenever the confidence term is on.
	// It deliberately does NOT try to make dense points lose the duplicate filter's response*size
	// ranking: that would depend on the detector's response range and break silently when it
	// shifts, so described-wins is an explicit rule there instead (FilterRedundantKeypoints).
	static cv::KeyPoint MakeDenseKeypoint(const Point2f& pt, float confidence, float warpCellSize) {
		return cv::KeyPoint(pt.x, pt.y, warpCellSize, -1.f, confidence);
	}

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
		ar & numDescribedKeypoints;
		ar & globalDescriptor;
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
		ar & numDescribedKeypoints;
		ar & globalDescriptor;
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

// Estimates image blur using a robust multi-scale variance-of-Laplacian focus measure.
// The function internally converts to grayscale, normalizes intensity to [0,1],
// evaluates Laplacian energy over an image pyramid (3 levels or until <64px),
// returns sharpness (smaller = blurrier)
SFM_API float EstimateImageSharpness(const cv::Mat& pixels);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_IMAGE_H_
