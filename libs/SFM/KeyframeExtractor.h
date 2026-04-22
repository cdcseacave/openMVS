////////////////////////////////////////////////////////////////////
// KeyframeExtractor.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_KEYFRAMEEXTRACTOR_H_
#define _SFM_KEYFRAMEEXTRACTOR_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Scene.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API PairsMatcher;

// Configuration for keyframe extraction
struct SFM_API KeyframeConfig
{
	// Camera model type
	CameraType cameraType { CameraType::PINHOLE };

	// Feature detector type: "AKAZE", "ORB", or "SIFT"
	FeatureType detectorType { FeatureType::AKAZE };

	// Overlap threshold (0-1): keyframe is selected when overlap drops below this
	float overlapThreshold { 0.85f };

	// Known camera intrinsics (optional)
	// If focalLength > 0, use this as the initial focal length estimate instead of max(width, height)
	// If focalLength > 0, focal calibration from fundamental matrices will refine this estimate
	// If focalLength <= 0, use max(width, height) as initial guess and rely on auto-calibration
	float focalLength { 0.f };

	// Principal point offsets from image center (optional)
	// If both are 0.0, principal point is assumed at image center (width/2, height/2)
	// Otherwise, principal point is (width/2 + ppOffsetX, height/2 + ppOffsetY)
	float ppOffsetX { 0.f };
	float ppOffsetY { 0.f };

	// Maximum features per grid cell (3x3 grid)
	unsigned maxFeaturesPerCell { 3000 };

	// Minimum features per cell (adjust sensitivity if below)
	unsigned minFeaturesPerCell { 500 };

	// Number of tangent-pinhole faces used when extracting features from
	// spherical frames (ignored for pinhole). Valid values: 4, 6, 8, 12, 20.
	unsigned cubemapFaces { 6 };

	// Optional Gaussian blur kernel size applied to images passed to optical flow
	// 0 = disabled (default). When >0, images used for calcOpticalFlowPyrLK
	// will be blurred with this kernel size to improve tracking in noisy/video-compressed frames.
	unsigned blurSize { 0 };

	// Use CUDA for SiftGPU if available (otherwise OpenGL)
	bool useCUDA = true;

	// Output directory for keyframe images
	String outputDirectory { "keyframes" };

	enum RefineCalibrationType {
		// No refinement of intrinsics
		NONE = 0,
		// Enable refinement of calibration (focal & radial distortion) on matched keyframe pairs.
		// When true (default) and intrinsics are not yet trusted, the system
		// attempts to refine focal length and distortion.
		TWO_VIEW = 1,
		// Run star-initializer + bundle-adjustment calibration over
		// subsampled consecutive triplets of keyframes at the end of extraction.
		// The middle keyframe acts as the reference (highest connectivity).
		// Collects (f,k1,k2) estimates per triplet and aggregates them.
		THREE_VIEW = 2,
		// Run view graph calibration using Fetzer focal length estimation.
		// Uses the entire graph of image pairs and their fundamental matrices
		// to globally optimize focal length. More robust than sequential methods.
		VIEW_GRAPH = 3
	};
	RefineCalibrationType refineCalibration { VIEW_GRAPH };
};
/*----------------------------------------------------------------*/


// KeyframeExtractor extracts keyframes from video using hybrid tracking
// (features + optical flow) and ensures minimum overlap between consecutive keyframes
namespace KeyframeExtractor {

// Extract keyframes from a video file
// Returns a Scene object containing the extracted keyframes and their relationships
SFM_API bool ExtractFromVideo(const String& videoPath, const KeyframeConfig& config, Scene& scene);

// Helper function: Compute overlap between two sets of tracked features
SFM_API float ComputeFeatureOverlap(
	const std::vector<Point2f>& prevPoints,
	const std::vector<Point2f>& currPoints,
	const std::vector<uchar>& status,
	const cv::Size& imageSize);

// Helper function: Compute overlap area using homography (pinhole) or angular displacement (spherical)
SFM_API float ComputeHomographyOverlap(
	const std::vector<Point2f>& prevPoints,
	const std::vector<Point2f>& currPoints,
	const std::vector<uchar>& status,
	const cv::Size& imageSize,
	const Camera& camera);
/*----------------------------------------------------------------*/

} // namespace KeyframeExtractor

} // namespace SFM

#endif // _SFM_KEYFRAMEEXTRACTOR_H_

