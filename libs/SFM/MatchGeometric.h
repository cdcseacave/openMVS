////////////////////////////////////////////////////////////////////
// MatchGeometric.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_MATCHGEOMETRIC_H_
#define _SFM_MATCHGEOMETRIC_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "View.h"
#include "PairsMatcher.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Where the guided match of one described keypoint of imgA looks for its match in imgB
struct SFM_API GuidedSearch {
	float length = 0.f;              // half-length of the band along the epipolar line, imgB pixels; the radius of the disc searched without a line
	float halfWidth = 0.f;           // half-width of the band across the line, imgB pixels; <= 0 searches the disc
	std::optional<Matrix3x3> F;      // x_B^T F x_A = 0 in pixels, the line's source; absent searches the disc
	float sameFeatureDistance = 3.f; // a rival this close to the winner is the same feature described twice, not a rival
	// Which winners must also beat the closest described keypoint OUTSIDE the region by the ratio
	enum OutsideReference : uint8_t {
		OUTSIDE_NONE = 0, // none: a winner alone in its region stands, one with a rival answers to the rival alone
		OUTSIDE_LONE = 1, // a winner alone in its region, which has no rival to answer to
		OUTSIDE_ALL  = 2  // every winner, on top of the ratio against its rival
	};
	uint8_t outsideReference = OUTSIDE_ALL;
};

// The adaptive half-width of the band (ROMA2Config::guidedBandResidualFactor): `factor` times the
// median of the given epipolar residuals -- the verdict's inlier cells under the pair's geometry, in
// pixels -- held between minHalfWidth (the matcher's epipolar bar) and maxHalfWidth (the band's
// length); no residual at all gives the floor. The vector is taken by value because the median is
// found in place.
SFM_API float GuidedBandHalfWidth(std::vector<float> residuals, float factor, float minHalfWidth, float maxHalfWidth);

/**
 * @brief Guided sparse matching of one admitted image pair, under the warp that admitted it.
 *
 * The ROMAv2 one-pass matcher's descriptor step (MatchPairsROMA2): the pair already passed the dense
 * verdict, which is where its geometry comes from, so this function only has to say which described
 * keypoints of the two images are the same point.
 *
 * For each described keypoint i of imgA the warp tracked (trackStatus[i] == 1, prediction
 * trackedB[i] in imgB's pixels), the candidates are imgB's described keypoints inside the query's
 * search region (GuidedSearch): a band along the keypoint's epipolar line under search.F, centred on
 * the prediction, search.length either way along the line and search.halfWidth either way across it
 * -- or the disc of radius search.length around the prediction when there is no line to follow (no F,
 * a half-width of 0, or a prediction within two lengths of imgB's epipole, where every direction is
 * equally wrong). The winner is the candidate with the smallest descriptor distance. It is accepted
 * iff it beats its closest rival inside the region by the matcher's ratio (MatchConfig::matchRatio),
 * a rival being any other candidate farther than search.sameFeatureDistance from it: a scale or
 * orientation duplicate of the winner, sitting on top of it, is the same feature described twice and
 * counts for nothing, while a second keypoint along the line as close in appearance as the winner
 * makes the match ambiguous and refuses it -- the epipolar test downstream cannot see that neighbour,
 * so this is where it is caught. Keypoints outside the region take no part: the geometry says the
 * match is not there.
 *
 * The reference OUTSIDE the region (search.outsideReference): a winner must also beat the closest
 * described keypoint of imgB outside its region by the same ratio -- every winner (OUTSIDE_ALL), the
 * reference every winner answered to before the band; or only a winner alone in its region, which
 * has no rival to answer to (OUTSIDE_LONE); or none (OUTSIDE_NONE), a lone winner standing as it is.
 * The outside reference is what refuses an impostor where the true keypoint was never detected in
 * imgB: the winner is then a random keypoint of the region, one of a handful, and a handful's second
 * best is beaten by the ratio a third of the time, where the best of imgB's thousands is not (on
 * alameda, the region's ratio alone let through 33M matches SIFT does not have, at a median Sampson
 * error twice that of the common ones, against 4.3M with the outside reference as well). The outside
 * distance comes from the
 * thread's own descriptor matcher: the K_NN = 8 nearest neighbours of the query over all of imgB's
 * described descriptors (approximate when that matcher is a FLANN index, which is the default), the
 * first of them not in the region is "best outside"; when all K_NN are in the region the K_NN-th
 * distance stands in -- every keypoint outside is at least that far, so the test can only get
 * stricter -- unless those K_NN are the whole of imgB, in which case there is no outside and the
 * winner stands. A keypoint with no candidate, or whose winner fails its test, yields no match.
 *
 * The band is centred on the WARP's prediction and takes only its direction from the geometry: the
 * prediction lives in imgB's own pixels, so an imprecise focal or principal point, or lens distortion
 * the fitted geometry does not model, turns the band by a few degrees and moves it not at all; the
 * half-width covers the warp's own across-line error (2.3 px median on a 2789 px frame with the
 * 160-cell grid, against 5.8 px in 2D), the length its along-line one.
 *
 * No geometry is estimated or applied here, no train-side collision is resolved and there is no
 * descriptor-only fallback: geometry is the verdict's business (JudgePairROMA2) and the final fit's
 * (AssemblePairROMA2), and a pair the warp rejected never reaches this function.
 *
 * Only the DESCRIBED prefix of either image takes part -- the query scan and the candidate search
 * both run over NumDescribedKeypoints() -- because every match this function selects is decided by a
 * descriptor row, which a dense keypoint appended by an earlier pair does not have.
 *
 * @param pairsMatcher  PairsMatcher holding the matching configuration (matchRatio,
 *                      descriptorsAreBinary) and the per-thread descriptor matchers;
 *                      MatchConfig::crossCheck must be off (a cross-checking matcher refuses k > 1).
 * @param imgA          Query image; its described keypoints and descriptors are the queries.
 * @param imgB          Train image; its described keypoints and descriptors are the candidates.
 * @param trackedB      Predicted position in imgB of every described keypoint of imgA, in the same
 *                      order and size as that prefix (TrackKeypointsByWarp's own convention).
 * @param trackStatus   Status per prediction (1 = valid, 0 = invalid); an untracked keypoint has no
 *                      disc to search and is skipped.
 * @param search        The search region (GuidedSearch): the band's length and half-width in imgB's
 *                      pixels, the geometry its line comes from, the same-feature distance and which
 *                      winners answer to the outside reference.
 * @param threadIdx     Index of the calling thread, selecting its private descriptor matcher inside
 *                      pairsMatcher; must be in [0, PairsMatcher::GetNumMatchers()), and concurrent
 *                      calls must pass distinct indices.
 * @param matches       Out: the selected matches (queryIdx = i, trainIdx = j), cleared first and
 *                      filled in increasing queryIdx; a tie inside a region goes to the smaller
 *                      trainIdx, so the result is decided by the inputs alone and not by the order
 *                      the candidates happened to be collected in.
 * @return the number of selected matches.
 */
SFM_API size_t MatchFeaturesGuided(
	PairsMatcher& pairsMatcher,
	const Image& imgA,
	const Image& imgB,
	const std::vector<Point2f>& trackedB,
	const std::vector<uchar>& trackStatus,
	const GuidedSearch& search,
	unsigned threadIdx,
	std::vector<DMatch>& matches);
/*----------------------------------------------------------------*/

/**
 * @brief Match the features of two consecutive video keyframes along the geometry their optical-flow
 * tracks imply.
 *
 * The video keyframe path (KeyframeExtractor): two frames close in time, already linked by the
 * tracker that decided the second one is a keyframe, and by nothing else -- no warp, no dense
 * verdict, no geometry a caller could supply. So the geometry is estimated here, from the tracked
 * points themselves (Step 1, PairsMatcher::GeometricFilter), and the descriptor matches are then
 * selected in the epipolar band it defines (Step 2), each keypoint of img1 restricted to a spatial
 * neighborhood of its tracked position in img2 rather than to the whole epipolar line. The winner of
 * a band must pass the plain ratio test against the second best of that same band.
 *
 * When the tracks are too few to fit a geometry, or the fit fails, the pair falls back to plain
 * descriptor-only matching (PairsMatcher::MatchFeatures) and the function reports false: the pair is
 * still worth matching, it just has no guidance left to match with.
 *
 * Configuration is taken from pairsMatcher.GetConfig(): maxEpipolarError is the RANSAC threshold of
 * the estimation, and minTriangulationAngle, reprojThreshold and epipoleFilterThreshold the filters
 * applied to the selected matches at the end.
 *
 * Only the DESCRIBED prefix of either image takes part: both the query scan and the candidate search
 * (octree and brute-force fallback) run over NumDescribedKeypoints(), because every match this
 * function selects is decided by a descriptor row, which a dense keypoint appended by another pass
 * does not have.
 *
 * @param pairsMatcher       PairsMatcher instance with config and descriptor matching.
 * @param img1               Image 1 (provides keypoints, descriptors, camera).
 * @param img2               Image 2 (provides keypoints, descriptors, camera).
 * @param trackedPoints1     Tracked pixel positions in image 1 (same order and size as img1's
 *                           described keypoint prefix -- the tracker's own convention).
 * @param trackedPoints2     Expected pixel positions in image 2 (same order as trackedPoints1).
 * @param trackStatus        Status per tracked point (1 = valid, 0 = invalid). Also gates the
 *                           "insufficient tracked points" floor: the tracked points are both the
 *                           estimation's input and Step 2's spatial-disc centres.
 * @param pair               ImagePair for both input tracked matches and output geometry + matches.
 * @param epipolarThreshold  Maximum distance to epipolar line for geometric match acceptance (pixels).
 * @param threadIdx          Index of the calling thread, selecting its private descriptor matcher
 *                           inside pairsMatcher; must be in [0, PairsMatcher::GetNumMatchers()),
 *                           and concurrent calls must pass distinct indices.
 * @return true if a geometry was estimated; false if the descriptor-only fallback was used.
 */
SFM_API bool MatchFeaturesGeometric(
	PairsMatcher& pairsMatcher,
	const Image& img1,
	const Image& img2,
	const std::vector<Point2f>& trackedPoints1,
	const std::vector<Point2f>& trackedPoints2,
	const std::vector<uchar>& trackStatus,
	ImagePair& pair,
	float epipolarThreshold = 2.f,
	unsigned threadIdx = 0);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_MATCHGEOMETRIC_H_
