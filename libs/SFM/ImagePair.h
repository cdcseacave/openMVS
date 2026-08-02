////////////////////////////////////////////////////////////////////
// ImagePair.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_IMAGEPAIR_H_
#define _SFM_IMAGEPAIR_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"
#include "Pose.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API Image;

// Simple match structure (similar to cv::DMatch)
struct SFM_API DMatch
{
	uint32_t queryIdx; // query feature/descriptor index
	uint32_t trainIdx; // train feature/descriptor index

	DMatch()
		: queryIdx(0), trainIdx(0) {}
	DMatch(uint32_t _queryIdx, uint32_t _trainIdx)
		: queryIdx(_queryIdx), trainIdx(_trainIdx) {}
	DMatch(const cv::DMatch& m)
		: queryIdx((uint32_t)m.queryIdx), trainIdx((uint32_t)m.trainIdx) {}

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & queryIdx;
		ar & trainIdx;
	}
	#endif
};

// ImagePair stores data for two images: matches, relative pose, etc.
class SFM_API ImagePair
{
public:
	IIndex ID1; // ID of first image
	IIndex ID2; // ID of second image (always > ID1)

	// Feature matches between the two images
	std::vector<DMatch> matches; // inliers (after geometric verification and filtering)
	std::vector<DMatch> outlierMatches; // outlier (split from initial matches)
	int numFilteredInliers; // number of inliers after filtering (cheirality, angle, epipole), as the first N of `matches`

	// Relative pose from image1 to image2 (optional)
	std::optional<Pose3D> relativePose;

	// Fundamental matrix (optional)
	std::optional<Matrix3x3> F;

	// Essential matrix (optional)
	std::optional<Matrix3x3> E;

	// Homography matrix (optional) - useful for overlap computation
	std::optional<Matrix3x3> H;

	// Overlap metrics
	float overlapRatio;       // ratio of tracked/matched features
	float overlapArea;        // overlap area computed from homography (0-1)
	float meanRayAngle;       // median angle between viewing rays of inlier matches in radians (pseudo-baseline)

	// Composite weighting scores
	float weightSpatial;      // Intrinsic: geometric spread/conditioning (0-1)
	float weightConnectivity; // Extrinsic: local connectivity strength (0-1)
	float weightTriplet;      // Extrinsic: cycle consistency support (0-1)

public:
	ImagePair()
		: ID1(NO_ID), ID2(NO_ID), numFilteredInliers(-1),
		  overlapRatio(0.f), overlapArea(0.f), meanRayAngle(0.f),
		  weightSpatial(0.f), weightConnectivity(0.f), weightTriplet(0.f) {}

	ImagePair(IIndex _ID1, IIndex _ID2)
		: ID1(_ID1), ID2(_ID2), numFilteredInliers(-1),
		  overlapRatio(0.f), overlapArea(0.f), meanRayAngle(0.f),
		  weightSpatial(0.f), weightConnectivity(0.f), weightTriplet(0.f)
	{
		if (ID1 > ID2)
			std::swap(ID1, ID2);
	}

	// Clear all data
	void Reset() {
		ResetMatches();
		ResetGeometry();
	}
	// Reset all matches
	void ResetMatches() {
		matches = std::vector<DMatch>();
		outlierMatches = std::vector<DMatch>();
		numFilteredInliers = -1;
	}
	// Reset inlier matches by merging all matches back
	void ResetInlierMatches() {
		matches.insert(matches.end(), outlierMatches.begin(), outlierMatches.end());
		outlierMatches = std::vector<DMatch>();
		numFilteredInliers = -1;
	}
	// Reset geometric data
	void ResetGeometry() {
		relativePose.reset();
		F.reset();
		E.reset();
		H.reset();
		overlapRatio = 0.f;
		overlapArea = 0.f;
		meanRayAngle = 0.f;
		weightSpatial = 0.f;
		weightConnectivity = 0.f;
		weightTriplet = 0.f;
	}

	// Invalidate pair matches setting them all as outliers
	void InvalidateMatches() {
		numFilteredInliers = -1;
		if (matches.empty())
			return;
		outlierMatches.insert(outlierMatches.end(), matches.begin(), matches.end());
		matches = std::vector<DMatch>();
	}
	// Check if pair has matches
	inline bool HasMatches() const { return !matches.empty(); }
	// Check if pair has geometric verification
	inline bool HasGeometricVerification() const {
		return relativePose.has_value() || F.has_value() || E.has_value() || H.has_value();
	}

	// Get number of matches/inliers
	unsigned GetNumMatches() const { return (unsigned)matches.size() + (unsigned)outlierMatches.size(); }
	unsigned GetNumInliers() const { return (unsigned)matches.size(); }
	unsigned GetNumFilteredInliers() const { return numFilteredInliers >= 0 ? (unsigned)numFilteredInliers : GetNumInliers(); }

	// Compute composite weight from components:
	// W = numInliers * cbrt(weightSpatial * weightConnectivity * (0.5 + weightTriplet))
	// The quality factors are combined by geometric mean instead of a raw product: each lives
	// in [0,1] and a raw product spans many orders of magnitude, so one weak (or noisy) factor
	// annihilates a pair with hundreds of verified inliers and disconnects valid sub-blocks
	// from the track graph; the geometric mean preserves the ordering while keeping the weight
	// commensurate with the inlier evidence.
	inline float GetCompositeWeight() const {
		const unsigned nCappedInliers = MINF(GetNumFilteredInliers(), 1000u); // cap inliers to avoid excessive weight
		const float wQuality = weightSpatial * weightConnectivity * (0.5f + weightTriplet);
		return nCappedInliers * CBRT(wQuality);
	}
	inline bool HasValidWeight() const {
		return GetCompositeWeight() > 0.f;
	}
	inline void InvalidateWeight() {
		weightSpatial = 0.f;
	}

	// Partition current matches by an inlier mask (true=inlier),
	// storing inliers in `matches` and outliers in `outlierMatches`.
	// Preserve existing outlier matches, it adds to them.
	//  - if numInliers<0, it counts inliers from the mask
	//  - if reorderOnly=true, it only reorders matches without splitting, placing the inliers first
	// Returns the number of inliers.
	unsigned PartitionMatchesByMask(const std::vector<char>& mask, int numInliers = -1, bool reorderOnly = false);

	// Return all matched points (either inliers only or all matches)
	//  - allInliers: if true, returns both filtered and inlier matched points
	//  - allMatches: if true, returns all matched points (inliers + outliers)
	std::pair<std::vector<Point2f>, std::vector<Point2f>> GetMatchedPoints(
		const Image& img1, const Image& img2, bool allInliers = false, bool allMatches = false) const;

	// Filter matches using cheirality, triangulation angle, and epipole distance constraints
	// minAngle: minimum triangulation angle in degrees
	// epipoleThresh: minimum distance to epipole in pixels (if > 0)
	// reprojThreshold: maximum reprojection error in pixels (if > 0)
	unsigned FilterMatches(
		const Image& img1,
		const Image& img2,
		float minAngle = 2.f,
		float reprojThreshold = 6.f,
		float epipoleThresh = 0.f);

	// Check inliers based on epipolar constraint; returns number of inliers
	//  - threshold: inlier distance threshold in pixels (for fundamental/essential) or symmetric transfer error (for homography)
	//  - forceEpipolarType: -1=auto, 0=relativePose, 1=E, 2=F, 3=H
	unsigned CheckEpipolarInliers(const Image& img1, const Image& img2, float threshold = 3.f, int forceEpipolarType = -1,
		cv::InputOutputArray inlierMask = cv::noArray()) const;

	// static functions for composing the essential matrix from relative pose and vice-versa
	static Matrix3x3 ComposeEssentialMatrix(const Pose3D& pose);
	static Pose3D DecomposeEssentialMatrix(const Matrix3x3& E);
	// static function to compute epipole from essential matrix (in homogeneous coordinates)
	static Point3 EpipoleFromEssentialMatrix(const Matrix3x3& E, bool leftImage);

	// static functions for composing the fundamental matrix from essential + camera matrices and vice-versa
	static Matrix3x3 ComposeFundamentalMatrix(const Matrix3x3& E, const Matrix3x3& K1, const Matrix3x3& K2, bool normalize = false);
	static Matrix3x3 DecomposeFundamentalMatrix(const Matrix3x3& F, const Matrix3x3& K1, const Matrix3x3& K2, bool normalize = false);

	// Recover the unique relative pose from essential matrix and matched points using cheirality check
	static unsigned RecoverPose(
		const Matrix3x3& E,
		const std::vector<Point2f>& points1,
		const std::vector<Point2f>& points2,
		const Matrix3x3& K,
		Pose3D& pose,
		cv::InputOutputArray inliers = cv::noArray());

	// The Mathematics of Angle Baseline Weighting
	// The average ray angle θ_avg provides:
	//   - Small θ_avg (< 1.5°): Poor baseline - noisy depth estimation
	//   - Medium θ_avg (6-24°): Good baseline - reliable triangulation (optimal at 15°)
	//   - Large θ_avg (> 24°): Wide baseline - harder matching but potentially better if matches exist
	// Returns a weight in [0, 1] with minimal penalty near optimal angle, slow degradation within ±9°,
	// and rapid falloff for very small (< 1.5°) or very large (> 24°) angles
	static float ComputeAngleBaselineWeight(float meanAngleDegrees);
	inline float ComputeAngleBaselineWeight() const { return ComputeAngleBaselineWeight(R2D(meanRayAngle)); }

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void save(Archive& ar, const unsigned int /*version*/) const {
		ar & ID1 & ID2;
		ar & matches & outlierMatches;
		ar & numFilteredInliers;
		ar & overlapRatio & overlapArea & meanRayAngle;
		ar & weightSpatial & weightConnectivity & weightTriplet;

		// Serialize std::optional fields
		const bool hasRelativePose = relativePose.has_value();
		ar & hasRelativePose;
		if (hasRelativePose)
			ar & relativePose.value();

		const bool hasFundamental = F.has_value();
		ar & hasFundamental;
		if (hasFundamental)
			ar & F.value();

		const bool hasEssential = E.has_value();
		ar & hasEssential;
		if (hasEssential)
			ar & E.value();

		const bool hasHomography = H.has_value();
		ar & hasHomography;
		if (hasHomography)
			ar & H.value();
	}

	template<class Archive>
	void load(Archive& ar, const unsigned int /*version*/) {
		ar & ID1 & ID2;
		ar & matches & outlierMatches;
		ar & numFilteredInliers;
		ar & overlapRatio & overlapArea & meanRayAngle;
		ar & weightSpatial & weightConnectivity & weightTriplet;

		// Deserialize std::optional fields
		bool hasRelativePose;
		ar & hasRelativePose;
		if (hasRelativePose) {
			Pose3D pose;
			ar & pose;
			relativePose = pose;
		}

		bool hasFundamental;
		ar & hasFundamental;
		if (hasFundamental) {
			Matrix3x3 mat;
			ar & mat;
			F = mat;
		}

		bool hasEssential;
		ar & hasEssential;
		if (hasEssential) {
			Matrix3x3 mat;
			ar & mat;
			E = mat;
		}

		bool hasHomography;
		ar & hasHomography;
		if (hasHomography) {
			Matrix3x3 mat;
			ar & mat;
			H = mat;
		}
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER()
	#endif
};

typedef CLISTDEF2IDX(ImagePair, uint32_t) ImagePairArr;
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_IMAGEPAIR_H_
