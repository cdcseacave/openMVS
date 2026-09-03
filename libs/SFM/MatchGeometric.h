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

/**
 * @brief Guided sparse matching of one admitted image pair, under the warp that admitted it.
 *
 * The ROMAv2 one-pass matcher's descriptor step (MatchPairsROMA2): the pair already passed the dense
 * verdict, which is where its geometry comes from, so this function only has to say which described
 * keypoints of the two images are the same point.
 *
 * For each described keypoint i of imgA the warp tracked (trackStatus[i] == 1, prediction
 * trackedB[i] in imgB's pixels), the candidates are imgB's described keypoints within discRadius of
 * the prediction; the winner is the candidate with the smallest descriptor distance, accepted iff it
 * beats the descriptor distance of the best keypoint of imgB OUTSIDE the disc by the matcher's ratio
 * (MatchConfig::matchRatio). The outside distance comes from the thread's own descriptor matcher:
 * the K_NN = 8 nearest neighbours of the query over all of imgB's described descriptors (approximate
 * when that matcher is a FLANN index, which is the default), the first of them not in the disc is
 * "best outside"; when all K_NN are in the disc the K_NN-th distance stands in -- every keypoint
 * outside the disc is at least that far, so the test can only get stricter -- unless those K_NN are
 * the whole of imgB, in which case there is no outside and the winner stands. A keypoint with no
 * candidate, or whose winner fails the test, yields no match.
 *
 * The ratio is taken against the best keypoint OUTSIDE the disc, and not against the second best
 * overall, because the warp already says where the match is. A scale or orientation duplicate of the
 * true match sitting on top of it -- the classic reason a correct match fails the plain ratio test,
 * which sees two near-identical distances and rejects both -- is inside the disc and no longer
 * defeats it, while a lookalike anywhere else in imgB still does: appearance must agree with the
 * warp, not merely exist somewhere in the other image.
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
 * @param discRadius    Radius of the search disc around a prediction, in imgB's pixels: two warp
 *                      cells, 2 * MAXF(imgB.width, imgB.height) / warpSize.
 * @param threadIdx     Index of the calling thread, selecting its private descriptor matcher inside
 *                      pairsMatcher; must be in [0, PairsMatcher::GetNumMatchers()), and concurrent
 *                      calls must pass distinct indices.
 * @param matches       Out: the selected matches (queryIdx = i, trainIdx = j), cleared first and
 *                      filled in increasing queryIdx; a tie inside a disc goes to the smaller
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
	float discRadius,
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
