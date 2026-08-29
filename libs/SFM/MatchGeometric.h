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
 * Uses tracked points to estimate relative pose or F via GeometricFilter,
 * then filters descriptor matches within an epipolar/spatial band.
 * When geometry estimation fails, falls back to descriptor-only matching.
 *
 * Configuration is taken from pairsMatcher.GetConfig():
 * - maxEpipolarError: RANSAC threshold and epipolar constraint threshold.
 * - Other config settings (minTriangulationAngle, reprojThreshold, epipoleFilterThreshold) 
 *   are applied during geometric verification.
 *
 * @param pairsMatcher        PairsMatcher instance with config and descriptor matching.
 * @param img1               Image 1 (provides keypoints, descriptors, camera).
 * @param img2               Image 2 (provides keypoints, descriptors, camera).
 * @param trackedPoints1     Tracked pixel positions in image 1 (same order as keypoints1).
 * @param trackedPoints2     Expected pixel positions in image 2 (same order as keypoints1).
 * @param trackStatus        Status per tracked point (1 = valid, 0 = invalid).
 * @param pair               ImagePair for both input tracked matches and output geometry + matches.
 * @param epipolarThreshold  Maximum distance to epipolar line for geometric match acceptance (pixels).
 * @param threadIdx          Index of the calling thread, selecting its private descriptor matcher
 *                           inside pairsMatcher; must be in [0, Scene::nMaxThreads), which is how
 *                           many matchers PairsMatcher creates (PairsMatcher::GetNumMatchers()),
 *                           and concurrent calls must pass distinct indices.
 * @return true if geometry was estimated (pair.E/F/relativePose); false if fallback was used.
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
