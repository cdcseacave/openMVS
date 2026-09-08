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

// How much one DENSE (ROMAv2 warp sampled, descriptor-less) match is worth as EVIDENCE that two
// images see the same thing, relative to the 1.0 a descriptor match carries: the view graph's own
// discount, read into PairsWeightingConfig::denseObservationWeight and consumed everywhere through
// ImagePair::GetNumWeightedInliers (ComputePairsWeights is the one pass that writes it).
//
// Bundle adjustment borrows this same constant, but only as EstimateDenseObservationWeight's
// fallback -- a scene with no dense keypoints at all, a population too small to give a sigma, or a
// sigma of exactly zero: its real per-observation weight is MEASURED at the head of every solve, off
// the scene's own dense-vs-described reprojection sigmas (BundleAdjustment.cpp), because that is a
// different question with a different answer. A warp correspondence localizes a point several times
// less precisely than a descriptor one -- which is exactly what bundle adjustment charges it for --
// but it says nearly as much about whether the two images overlap. Charging the precision penalty
// twice would demote exactly the pairs that carry a capture the descriptor matcher cannot match at
// all: for a textureless interior capture, those dense-only pairs can be the difference between
// images registering at all and not registering.
//
// So this stays a fixed constant deliberately: nothing has asked the view graph's number to be
// measured, and coupling it to bundle adjustment's moving one would re-couple two answers that must
// stay independent.
constexpr double DENSE_OBSERVATION_WEIGHT = 0.25;

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
	// GetNumFilteredInliers() is the first segment only: the pair's DESCRIPTOR evidence, which is what
	// the estimation bars (minMatches, the strict filter's own return) and the diagnostics that report
	// "how many correspondences did the descriptor matcher verify" mean. What the view graph ranks on
	// is GetNumWeightedInliers(), sparse + w * dense: the supplement is real evidence about the pair,
	// discounted for the precision of a warp-sampled position rather than ignored. What forms tracks
	// is the union of the first two segments, GetNumTrackFormingMatches().
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
	float meanRayAngle;       // median angle between viewing rays of the TRACK-FORMING matches in
	                          // radians (pseudo-baseline): the dense supplement is included, because a
	                          // ray angle is a geometric quantity a warp-sampled position measures
	                          // just as well, and this feeds ComputeIntrinsicWeight -- the one term
	                          // that can demote a degenerate baseline must be available on a pair
	                          // whose evidence is dense. 0 = never measured (no relative pose)

	// Composite weighting scores
	float weightSpatial;      // Intrinsic: geometric spread/conditioning (0-1)
	float weightConnectivity; // Extrinsic: local connectivity strength (0-1)
	float weightTriplet;      // Extrinsic: cycle consistency support (0-1)
	// The pair's INLIER EVIDENCE as everything that ranks the view graph reads it (through
	// GetNumWeightedInliers): its sparse inliers plus its dense supplement discounted by the dense
	// observation weight, sparse + w * dense. Written by ComputePairsWeights, the one pass that
	// holds the view graph's own dense discount (PairsWeightingConfig::denseObservationWeight,
	// DENSE_OBSERVATION_WEIGHT above); -1 until it has run, and the accessor then answers with the
	// sparse count -- which is the pre-supplement answer, and is what every consumer running before
	// the weighting pass (the matcher's own replace and skip tests) has always used. Cleared by every
	// writer that changes the partition it summarises -- the four reset paths below,
	// AppendDenseMatches, and FilterRedundantKeypoints' recount -- so a stale value can never be
	// read as a fresh one.
	// Stored rather than computed on the fly because GetCompositeWeight() and its ~15 callers have
	// no access to a configuration, and a second hard-coded copy of the weight would be a second
	// answer to a question that must have one.
	float weightedInliers;

public:
	ImagePair()
		: ID1(NO_ID), ID2(NO_ID), numFilteredInliers(-1), numDenseInliers(0),
		  overlapRatio(0.f), overlapArea(0.f), meanRayAngle(0.f),
		  weightSpatial(0.f), weightConnectivity(0.f), weightTriplet(0.f), weightedInliers(-1.f) {}

	ImagePair(IIndex _ID1, IIndex _ID2)
		: ID1(_ID1), ID2(_ID2), numFilteredInliers(-1), numDenseInliers(0),
		  overlapRatio(0.f), overlapArea(0.f), meanRayAngle(0.f),
		  weightSpatial(0.f), weightConnectivity(0.f), weightTriplet(0.f), weightedInliers(-1.f)
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
		weightedInliers = -1.f; // it describes matches that no longer exist
	}
	// Reset inlier matches by merging all matches back
	void ResetInlierMatches() {
		matches.insert(matches.end(), outlierMatches.begin(), outlierMatches.end());
		outlierMatches = std::vector<DMatch>();
		numFilteredInliers = -1;
		numDenseInliers = 0;
		weightedInliers = -1.f; // the partition it counted is gone
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
		weightedInliers = -1.f;
	}

	// Invalidate pair matches setting them all as outliers
	void InvalidateMatches() {
		numFilteredInliers = -1;
		numDenseInliers = 0;
		weightedInliers = -1.f;
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
	// THE PAIR'S INLIER EVIDENCE, and the one notion of it: sparse + w * dense, rounded, with w the
	// dense observation weight (see `weightedInliers`). Everything that ranks or gates the view graph
	// on "how much correspondence evidence does this pair have" reads this -- the composite weight,
	// the connectivity normalisation, the triplet edge strength, the star initializer's degree and
	// candidate ranking, the calibrator's trust bars, the matcher's own replace and skip tests -- so
	// that a dense-only pair (no sparse inliers at all) is a first-class member of the graph rather
	// than a zero. The sparse count itself stays available for the two things that genuinely mean
	// "descriptor evidence": the diagnostics that report it, and the estimation bars that predate any
	// supplement.
	// Before the weighting pass has run it answers with the sparse count, which is what it always was.
	unsigned GetNumWeightedInliers() const {
		return weightedInliers >= 0.f ? ROUND2INT<unsigned>(weightedInliers) : GetNumFilteredInliers();
	}

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
		const unsigned nCappedInliers = MINF(GetNumWeightedInliers(), 1000u); // cap inliers to avoid excessive weight
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

	// The points of the TRACK-FORMING prefix: the sparse inliers plus the dense supplement, i.e.
	// exactly the matches BuildTracks unions, and nothing the strict filter rejected. What a
	// question about the pair's spatial COVERAGE has to be asked over -- a dense supplement covers
	// the frame it was drawn over whether or not it counts as descriptor evidence.
	std::pair<std::vector<Point2f>, std::vector<Point2f>> GetTrackFormingPoints(
		const Image& img1, const Image& img2) const;

	// The median triangulation angle over that same TRACK-FORMING prefix, in radians: what
	// meanRayAngle holds, measured rather than filtered. FilterMatches computes it as a by-product
	// of its accept loop; this is for the caller that CHANGED the prefix without re-filtering the
	// pair -- AppendDenseMatches, which must not put the dense supplement through the strict
	// geometric filter (that is the whole design of the supplement) but must not leave the pair
	// claiming a baseline measured on matches it no longer only has either.
	// 0 when the pair has no relative pose to triangulate against, i.e. no measurable baseline.
	float ComputeMeanRayAngle(const Image& img1, const Image& img2) const;

	// Filter matches using cheirality, triangulation angle, and epipole distance constraints
	// minAngle: minimum triangulation angle in degrees
	// epipoleThresh: minimum distance to epipole in pixels (if > 0)
	// reprojThreshold: maximum reprojection error in pixels (if > 0)
	// Rebuilds the whole partition and returns the SPARSE count, i.e. what GetNumFilteredInliers()
	// reports afterwards -- callers compare it against a minimum-matches bar, which is a
	// descriptor-evidence bar. The surviving dense supplement is re-derived into its own segment.
	// (Or matches.size(), unchanged, on the early return when there is no relative pose to filter
	// against; such a pair carries no dense segment either.)
	// meanRayAngle is updated over the whole accepted set, sparse and dense together, unlike the
	// SPARSE count this function returns: it feeds ComputeIntrinsicWeight through
	// ComputeAngleBaselineWeight, which scores a geometric quantity a warp-sampled position measures
	// just as well as a described one.
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
		ar & weightSpatial & weightConnectivity & weightTriplet & weightedInliers;

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
		// weightedInliers travels with the three weights it belongs to: the reconstruction path that
		// loads an already-matched scene does not re-run ComputePairsWeights, so a derived value left
		// out of the stream would come back as "never computed" and silently drop every pair's dense
		// evidence from the view graph
		ar & weightSpatial & weightConnectivity & weightTriplet & weightedInliers;

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
