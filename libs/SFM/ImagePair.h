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
	// `matches` carries a three-way partition, and each segment means something different:
	//   [0, numFilteredInliers)                                   sparse (descriptor-matched)
	//                                                             correspondences that passed the
	//                                                             strict filter -- track-forming,
	//                                                             and the pair's DESCRIPTOR EVIDENCE
	//   [numFilteredInliers, numFilteredInliers+numDenseInliers)   the dense (ROMAv2 warp) supplement
	//                                                             -- track-forming, but no evidence
	//                                                             of the pair's authority
	//   [numFilteredInliers+numDenseInliers, matches.size())       RANSAC inliers the strict filter
	//                                                             rejected -- not track-forming
	// GetNumFilteredInliers() is the first segment only: every view-graph weight and gate reads it as
	// "how much geometrically verified correspondence evidence does this pair have", and the dense
	// supplement is a coverage-maximising draw that must not re-rank the graph. What forms tracks is
	// the union of the first two, GetNumTrackFormingMatches().
	int numFilteredInliers; // number of inliers after filtering (cheirality, angle, epipole), as the first N of `matches`
	int numDenseInliers;    // number of dense supplement matches, stored right after those

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
	float meanRayAngle;       // median angle between viewing rays of the SPARSE inlier matches in radians
	                          // (pseudo-baseline); the dense supplement is excluded because this feeds
	                          // ComputeIntrinsicWeight, i.e. it is part of the pair's authority

	// Composite weighting scores
	float weightSpatial;      // Intrinsic: geometric spread/conditioning (0-1)
	float weightConnectivity; // Extrinsic: local connectivity strength (0-1)
	float weightTriplet;      // Extrinsic: cycle consistency support (0-1)

public:
	ImagePair()
		: ID1(NO_ID), ID2(NO_ID), numFilteredInliers(-1), numDenseInliers(0),
		  overlapRatio(0.f), overlapArea(0.f), meanRayAngle(0.f),
		  weightSpatial(0.f), weightConnectivity(0.f), weightTriplet(0.f) {}

	ImagePair(IIndex _ID1, IIndex _ID2)
		: ID1(_ID1), ID2(_ID2), numFilteredInliers(-1), numDenseInliers(0),
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
		numDenseInliers = 0;
	}
	// Reset inlier matches by merging all matches back
	void ResetInlierMatches() {
		matches.insert(matches.end(), outlierMatches.begin(), outlierMatches.end());
		outlierMatches = std::vector<DMatch>();
		numFilteredInliers = -1;
		numDenseInliers = 0;
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
		numDenseInliers = 0;
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
	// The pair's descriptor evidence: geometrically verified SPARSE correspondences only, the first
	// segment of `matches`. This is what every view-graph weight, gate and diagnostic reads, and it
	// deliberately does not count the dense supplement -- see the partition comment on the members.
	unsigned GetNumFilteredInliers() const {
		// a dense segment only ever exists past a materialised sparse count (AppendDenseMatches
		// closes it before inserting), so the -1 "no strict filter ran" case cannot carry one
		ASSERT(numFilteredInliers >= 0 || numDenseInliers == 0);
		return numFilteredInliers >= 0 ? (unsigned)numFilteredInliers : GetNumInliers();
	}
	// Number of dense (ROMAv2 warp) supplement matches, the second segment of `matches`
	unsigned GetNumDenseInliers() const {
		// The partition must describe ranges of `matches` that exist. Asserted at the READER rather
		// than only at the writers that maintain it, because the four union consumers (BuildTracks
		// steps 2 and 4, both GlobalAlignment union-finds) evaluate `matches[i]` for
		// i < GetNumTrackFormingMatches() and only look at the DMatch's contents on the next line:
		// an over-long dense count is an out-of-bounds read that happens before any of their own
		// bounds tests can see it. There are eight writers of the two counts today, and this catches
		// a ninth that grows one or shrinks `matches` without pairing the two.
		ASSERT(numDenseInliers >= 0 && (numFilteredInliers < 0 ? numDenseInliers == 0 :
			(size_t)numFilteredInliers + (size_t)numDenseInliers <= matches.size()));
		return (unsigned)numDenseInliers;
	}
	// The TRACK-FORMING set: the sparse inliers plus the dense supplement, i.e. the leading
	// `matches` prefix BuildTracks unions. Everything past it are matches the strict filter
	// deliberately rejected and must never form a track. Inherits both accessors' assertions.
	unsigned GetNumTrackFormingMatches() const { return GetNumFilteredInliers() + GetNumDenseInliers(); }

	// Debug-only invariant check on the `matches` partition: no match in the sparse segment
	// [0, GetNumFilteredInliers()) may have a dense (ROMAv2 warp) endpoint, since a descriptor match
	// is described at both ends on every path that produces one and the keypoint filter's
	// described-wins rule never moves a described keypoint onto a dense one. Compiled out of release
	// builds. Deliberately ONE-DIRECTIONAL: the converse is false by design, because
	// FilterRedundantKeypoints' step 3 maintains the dense segment by position and leaves a
	// supplement match whose *both* endpoints collapsed onto described survivors inside it, while
	// FilterMatches classifies that same shape as sparse.
	void CheckSparseSegmentIsDescribed(const Image& img1, const Image& img2) const;

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
	//  - default: only the GetNumFilteredInliers() prefix, i.e. the pair's sparse descriptor
	//    evidence -- neither the dense supplement nor the strict filter's rejects
	//  - allInliers: if true, returns every point of `matches` (rejects and dense included), which
	//    is what a caller iterating `matches` element by element needs
	//  - allMatches: if true, returns all matched points (inliers + outliers)
	std::pair<std::vector<Point2f>, std::vector<Point2f>> GetMatchedPoints(
		const Image& img1, const Image& img2, bool allInliers = false, bool allMatches = false) const;

	// Filter matches using cheirality, triangulation angle, and epipole distance constraints
	// minAngle: minimum triangulation angle in degrees
	// epipoleThresh: minimum distance to epipole in pixels (if > 0)
	// reprojThreshold: maximum reprojection error in pixels (if > 0)
	// Rebuilds the whole partition and returns the SPARSE count, i.e. what GetNumFilteredInliers()
	// reports afterwards -- callers compare it against a minimum-matches bar, which is a
	// descriptor-evidence bar. The surviving dense supplement is re-derived into its own segment.
	// (Or matches.size(), unchanged, on the early return when there is no relative pose to filter
	// against; such a pair carries no dense segment either.)
	// meanRayAngle is updated from the SPARSE matches only, for the same reason the count is sparse:
	// it feeds ComputeIntrinsicWeight through ComputeAngleBaselineWeight.
	unsigned FilterMatches(
		const Image& img1,
		const Image& img2,
		float minAngle = 2.f,
		float reprojThreshold = 6.f,
		float epipoleThresh = 0.f);

	// Check inliers based on epipolar constraint; returns number of inliers
	//  - threshold: inlier distance threshold in pixels (for fundamental/essential) or symmetric transfer error (for homography)
	//  - forceEpipolarType: -1=auto, 0=relativePose, 1=E, 2=F, 3=H
	// The count and inlierMask cover ALL of `matches` -- the dense supplement and the strict filter's
	// rejects included -- which is what the mask's matches.size() length always promised; on a
	// partitioned pair this used to be truncated to the sparse segment.
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
		ar & numFilteredInliers & numDenseInliers;
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
		ar & numFilteredInliers & numDenseInliers;
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
