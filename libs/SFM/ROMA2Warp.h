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

// Quantiles of the per-pair epipolar residual the dense two-view gate records, as probabilities.
// The threshold cannot be swept offline the way a score can -- RANSAC's chosen model depends on it,
// so every value costs its own run -- but the shape of the residual distribution under the model
// that was fitted is visible from a single run, and it is what says whether right and wrong pairs
// differ in precision at all
constexpr unsigned DENSE_RESIDUAL_QUANTILES = 5;
constexpr float DENSE_RESIDUAL_PROBABILITIES[DENSE_RESIDUAL_QUANTILES] = {0.50f, 0.75f, 0.90f, 0.95f, 0.99f};

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
	// The dense sample and its inlier subset. WARNING for consumers: these three arrays are
	// released again on a pair the gate rejected (it is dropped, so nothing downstream can read
	// them) -- never infer a size from them, always read numSampled/numInliers below, which
	// survive. On a validated pair pointsA.size() == numSampled and inliers.size() == numInliers.
	std::vector<Point2f> pointsA, pointsB;   // the dense sample: pixels of the working orientation of each image, index-parallel
	std::vector<uint32_t> inliers;           // ascending indices into pointsA/pointsB the fitted geometry explains
	std::optional<Pose3D> relativePose;      // relative pose ID1 -> ID2 (set by the ESSENTIAL branch, and by a decomposed FUNDAMENTAL fit)
	std::optional<Matrix3x3> E;              // essential matrix, set together with relativePose
	std::optional<Matrix3x3> F;              // fundamental matrix, when both cameras are pinhole
	// Spread, on the same DENSE_COVERAGE_GRID^2 grid over each image: first of the whole drawn
	// sample, then of its inlier subset alone. The pair is the diagnostic -- a sample spread over
	// the overlap whose *inliers* huddle in one corner is a wrong pair that a ratio cannot see.
	float coverageA = 0.f, coverageB = 0.f;
	float coverageInlierA = 0.f, coverageInlierB = 0.f;
	unsigned numSampled = 0;                 // size of the drawn sample
	// Two inlier counts, because the estimator produces two and they differ per geometry branch:
	// numInliers is the RANSAC inlier set of the fitted geometry (ImagePair::GetNumInliers), and is
	// what inlierRatio and the gate's verdict are computed from, on every branch. numFiltered is
	// the subset of those that also survive the cheirality / triangulation-angle / reprojection
	// filter (ImagePair::GetNumFilteredInliers), which only runs where a relative pose exists -- so
	// on a plain FUNDAMENTAL fit the two are equal, and on an ESSENTIAL fit numFiltered <= numInliers.
	unsigned numInliers = 0;
	unsigned numFilteredInliers = 0;
	float inlierRatio = 0.f;                 // numInliers/numSampled, the quantity the gate thresholds
	float filteredInlierRatio = 0.f;         // numFilteredInliers/numSampled, recorded only
	uint8_t geometryBranch = 0;              // PairsMatcher::GeometryBranch the estimator actually took
	// The epipolar threshold this pair was actually fitted with, in both frames it can be read in:
	// the configured warp-native value (the model's own square input frame, RoMa2Onnx::ImageSize)
	// and what that became in this pair's full-resolution pixels. Both are recorded per pair because
	// the conversion depends on the images' resolution, so the same native setting is a different
	// number of target pixels on every dataset -- exactly the confound that makes a bare pixel
	// threshold unreadable across scenes.
	float epipolarNativePx = 0.f;
	float epipolarFullResPx = 0.f;
	// Epipolar (Sampson) residual of the whole drawn sample under the fitted geometry, as quantiles
	// in warp-native pixels, at DENSE_RESIDUAL_PROBABILITIES. Negative where there is nothing to
	// measure: no geometry was fitted, or the fit produced no fundamental matrix (a spherical or
	// mixed pair, where the residual is an angle rather than a pixel distance).
	float residualNative[DENSE_RESIDUAL_QUANTILES] = {-1.f, -1.f, -1.f, -1.f, -1.f};
	bool bValidated = false;                 // the verdict: inlierRatio >= ROMA2Config::minDenseInlierRatio
	// whether min(coverageInlierA, coverageInlierB) >= ROMA2Config::minInlierCoverage. Recorded
	// only: this round the gate's accept/reject is the ratio alone, and no complementary rejection
	// rule is pre-registered
	bool bMeetsInlierCoverage = false;
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

// Draw a spatially uniform sample of confident correspondences out of an (already eroded) warp.
// A cell is eligible when it is at least minConfidence confident and its warped point lands inside
// imgB; let E be how many of the grid's T cells are. If E <= maxSamples the whole confident overlap
// is taken. Otherwise the warp grid is stratified into n x n buckets over the WHOLE image, with
//    n = min(warpSize, ceil(sqrt(maxSamples * T / E))),
// and the most confident cell of each bucket is taken -- one per bucket, and nothing else.
//
// The T/E factor is what makes the draw uniform under partial overlap, and it is the whole point of
// the design. A pair never overlaps completely, so with a fixed sqrt(maxSamples) grid most buckets
// hold no eligible cell at all, and topping the budget up by descending confidence -- which an
// earlier version of this function did -- could only draw those extra points from inside the
// overlap, since that is the only place eligible cells exist. At 40% overlap that turned a
// nominally stratified draw into ~1226 of 2000 points crammed into the same 40% of the frame, and a
// sample crammed into a sub-area is locally smooth, which one geometry then fits almost exactly.
// Over-binning instead of topping up keeps the winners spread across the whole frame; the count of
// occupied buckets lands near maxSamples on its own, which is why there is no fill-up step.
//
// So maxSamples is a TARGET, not a cap: the returned sample is smaller when the confident overlap is
// small (which is the informative outcome -- the sample size now tracks the overlap instead of
// sitting at the budget regardless), and may exceed it slightly when the eligible cells are spread
// more evenly than a random scatter. Callers must therefore read the returned count, and must cope
// with a sample too small for whatever they do next.
//
// sampledA/sampledB come out in warp-grid raster order and are index-parallel, in the pixels of the
// working orientation of imgA/imgB (the pixels the keypoints live in, TrackKeypointsByWarp's
// convention). coverageA/coverageB receive the fraction of a DENSE_COVERAGE_GRID^2 grid over each
// image that the sample occupies; because the draw is now uniform over the whole frame rather than
// weighted toward wherever confidence is highest, that fraction is a fair proxy for the true
// overlap area, and a pair whose overlap is a corner of the frame must read as such.
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

// Fraction of a DENSE_COVERAGE_GRID^2 grid over each image that a warp sample occupies:
// coverageA over sampledA in an image of sizeA, coverageB over sampledB in an image of sizeB.
// `indices`, when non-empty, restricts the measurement to that subset of the sample (the gate
// measures the whole sample and its inlier subset with the same call, so the two numbers are
// comparable by construction). NOTE that an empty `indices` therefore means *the whole sample*,
// not the empty subset: the coverage of an empty subset is 0 and needs no call.
SFM_API void ComputeSampleCoverage(
	const std::vector<Point2f>& sampledA,
	const std::vector<Point2f>& sampledB,
	const cv::Size& sizeA,
	const cv::Size& sizeB,
	const std::vector<uint32_t>& indices,
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
