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
 * @brief Match features using tracked correspondences to guide epipolar search.
 *
 * Uses tracked points to estimate relative pose or F via GeometricFilter, unless the caller
 * already has a checked geometry for this pair (validatedGeometry), in which case the estimation
 * is skipped entirely and that geometry is used instead. Either way, Step 2 then filters
 * descriptor matches within an epipolar/spatial band. When no geometry is available -- neither
 * supplied nor successfully estimated -- falls back to descriptor-only matching.
 *
 * Configuration is taken from pairsMatcher.GetConfig():
 * - maxEpipolarError: RANSAC threshold and epipolar constraint threshold (estimating path only).
 * - Other config settings (minTriangulationAngle, reprojThreshold, epipoleFilterThreshold)
 *   are applied during geometric verification.
 *
 * Only the DESCRIBED prefix of either image takes part: both the query scan and the candidate
 * search (octree and brute-force fallback) run over NumDescribedKeypoints(), because every match
 * this function selects is decided by a descriptor row, which a dense keypoint appended by an
 * earlier supplemented pair does not have.
 *
 * @param pairsMatcher        PairsMatcher instance with config and descriptor matching.
 * @param img1               Image 1 (provides keypoints, descriptors, camera).
 * @param img2               Image 2 (provides keypoints, descriptors, camera).
 * @param trackedPoints1     Tracked pixel positions in image 1 (same order and size as img1's
 *                           described keypoint prefix -- TrackKeypointsByWarp's own convention).
 * @param trackedPoints2     Expected pixel positions in image 2 (same order as trackedPoints1).
 * @param trackStatus        Status per tracked point (1 = valid, 0 = invalid). Also gates the
 *                           "insufficient tracked points" floor below, whether or not a geometry
 *                           is supplied: the tracked points are Step 2's spatial-disc centres, so
 *                           a pair that tracked almost nothing has no guidance to give whatever
 *                           geometry it was handed.
 * @param pair               ImagePair for both input tracked matches and output geometry + matches.
 * @param validatedGeometry  Geometry a caller already fitted and RANSAC-checked for this pair
 *                           (e.g. the dense two-view gate, ValidatePairsROMA2), or NULL to estimate
 *                           it here from trackedPoints1/trackedPoints2 as before. When supplied,
 *                           GeometricFilter and its fallbacks are skipped -- that estimation is the
 *                           per-pair cost this parameter exists to remove.
 * @param epipolarThreshold  Maximum distance to epipolar line for geometric match acceptance (pixels).
 * @param threadIdx          Index of the calling thread, selecting its private descriptor matcher
 *                           inside pairsMatcher; must be in [0, Scene::nMaxThreads), which is how
 *                           many matchers PairsMatcher creates (PairsMatcher::GetNumMatchers()),
 *                           and concurrent calls must pass distinct indices.
 * @param crossCheck         Drop the forward matches that lose a train-side collision. The check is
 *                           restricted to the forward candidate sets - a match (i->j) survives only
 *                           if, among all keypoints of A whose selected candidate is j, i has the
 *                           smallest descriptor distance; ties keep the smaller queryIdx. No reverse
 *                           epipolar pass is run. Only with the check on does the single-candidate
 *                           case get its descriptor distance computed (it would otherwise stay 0 and
 *                           win every collision); off, the forward selection is bit-identical to
 *                           what it always was.
 * @param numSharedTrain     Optional out: how many forward matches share their trainIdx with another
 *                           match - the number the cross-check removed when it is on, the number of
 *                           matches sitting on a contested trainIdx when it is off. Diagnostic only
 *                           (ROMA2's per-pair DEBUG_ULTIMATE line); NULL skips the count.
 * @return true if geometry was available (estimated here, or supplied by the caller); false if
 *         fallback was used.
 */
SFM_API bool MatchFeaturesGeometric(
	PairsMatcher& pairsMatcher,
	const Image& img1,
	const Image& img2,
	const std::vector<Point2f>& trackedPoints1,
	const std::vector<Point2f>& trackedPoints2,
	const std::vector<uchar>& trackStatus,
	ImagePair& pair,
	const PairsMatcher::ValidatedGeometry* validatedGeometry,
	float epipolarThreshold = 2.f,
	unsigned threadIdx = 0,
	bool crossCheck = false,
	unsigned* numSharedTrain = NULL);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_MATCHGEOMETRIC_H_
