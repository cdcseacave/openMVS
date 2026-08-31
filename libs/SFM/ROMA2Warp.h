/*
 * ROMA2Warp.h
 *
 * Copyright (c) 2014-2026 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _SFM_ROMA2WARP_H_
#define _SFM_ROMA2WARP_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"
#include "ImagePair.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;

// Cells per image side of the coarse grid the spread of a warp sample is measured on
// (SampleWarpByCoverage): fine enough that a sample clustered in one part of the overlap cannot
// reach a high fraction, coarse enough that a legitimately sparse but spread-out sample does
constexpr unsigned DENSE_COVERAGE_GRID = 16;

// Dense correspondence maps of an image pair as produced by the ROMAv2 coarse matcher:
// one cell per warp grid position of the first image, holding where that cell lands in
// the second image and how confident the model is that both images see it
struct SFM_API WarpMaps {
	Image32F2 warp;   // normalized (align_corners=false) target coordinates, in [-1,1]
	Image32F overlap; // matching confidence, in [0,1]

	inline bool IsValid() const {
		return !warp.empty() && warp.size() == overlap.size();
	}
};
/*----------------------------------------------------------------*/

// One candidate image pair as the dense two-view gate saw it (ValidatePairsROMA2): the geometry a
// single model fitted to the coverage-maximising warp sample, the subset of that sample the
// geometry explains, and the spread the verdict rests on. Written for every candidate the gate
// warped, accepted or not, so the gate's threshold can be swept offline from a single run; on a
// rejected pair the point arrays are released again and only the counts below survive.
// This is the gate's hand-off interface to whatever consumes dense-validated pairs.
struct SFM_API DensePairValidation {
	IIndex ID1 = NO_ID, ID2 = NO_ID;         // the pair, ID1 < ID2 (indices into Scene::images)
	std::vector<Point2f> pointsA, pointsB;   // the dense sample: pixels of the working orientation of each image, index-parallel
	std::vector<uint32_t> inliers;           // ascending indices into pointsA/pointsB the fitted geometry explains
	std::optional<Pose3D> relativePose;      // relative pose ID1 -> ID2, when both cameras trust their intrinsics
	std::optional<Matrix3x3> E;              // essential matrix, set together with relativePose
	std::optional<Matrix3x3> F;              // fundamental matrix, when both cameras are pinhole
	float coverageA = 0.f, coverageB = 0.f;  // fraction of a DENSE_COVERAGE_GRID^2 grid the sample occupies in each image
	unsigned numSampled = 0;                 // size of the drawn sample (pointsA.size() while it is kept)
	unsigned numInliers = 0;                 // size of the inlier set (inliers.size() while it is kept)
	float inlierRatio = 0.f;                 // numInliers/numSampled, the quantity the gate thresholds
	bool bValidated = false;                 // the verdict: inlierRatio >= ROMA2Config::minDenseInlierRatio
};
typedef std::vector<DensePairValidation> DensePairValidationArr;
/*----------------------------------------------------------------*/

// Map a pixel coordinate from the resolution of A to the resolution of B (align_corners=true)
SFM_API Point2f CoordFromTo(const Point2f& coord, const cv::Size& sizeA, const cv::Size& sizeB);

// Map a normalized warp coordinate to the pixel coordinates of an image (align_corners=false)
SFM_API Point2f DenormCoord(const Point2f& normCoord, const cv::Size& size);

// Erode confidence map if requested (helps remove outliers near edges)
SFM_API void ErodeConfidenceMap(Image32F& imgConfidence, int erodeBorder, float minConfidence, float minErodeConfidence);

// Track the keypoints of imgA into imgB through the given warp and overlap maps;
// trackedA/trackedB/trackStatus are resized to the number of keypoints of imgA and
// trackedB is only written where trackStatus is 1.
// Returns the number of tracked keypoints.
SFM_API size_t TrackKeypointsByWarp(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& overlap,
	float minConfidence,
	std::vector<Point2f>& trackedA,
	std::vector<Point2f>& trackedB,
	std::vector<uchar>& trackStatus);

// Draw a coverage-maximising sample of confident correspondences out of an (already eroded) warp:
// the warp grid is stratified into floor(sqrt(maxSamples))^2 buckets, the most confident cell of
// every bucket is taken first -- so the sample's core is spread over the whole confident overlap
// rather than over its best-scoring corner -- and the rest of the budget is filled by descending
// confidence. Only cells at least minConfidence confident whose warped point lands inside imgB are
// eligible; the bucket count is derived from the budget so the winners alone can never exceed it.
// sampledA/sampledB come out in warp-grid raster order and are index-parallel, in the pixels of the
// working orientation of imgA/imgB (the pixels the keypoints live in, TrackKeypointsByWarp's
// convention). coverageA/coverageB receive the fraction of a DENSE_COVERAGE_GRID^2 grid over each
// image that the sample occupies -- a sample covering only part of the overlap must be visible as
// such, since a two-view verdict drawn from it rests on the spread.
// Returns the number of sampled correspondences.
SFM_API size_t SampleWarpByCoverage(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& overlap,
	float minConfidence,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	float& coverageA,
	float& coverageB);

// Store the given guided pair in the scene, either creating it or replacing the existing one:
// pairIndexMap maps the pair key to its index in scene.pairs and is updated accordingly,
// maxReplaceInliers is the inlier ceiling above which an existing pair is never replaced
// (0 = no ceiling), and bCreated tells whether a new pair was appended. A created pair carries
// no overlap of its own (overlapRatio/overlapArea stay 0), so ComputePairsWeights weights it
// from the same proxy it uses for every other pair.
// Returns true if the scene was modified (pair created or replaced).
SFM_API bool ApplyROMA2Pair(
	Scene& scene,
	std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap,
	ImagePair&& pair,
	unsigned maxReplaceInliers,
	bool& bCreated);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_ROMA2WARP_H_
