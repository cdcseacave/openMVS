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

// Map a pixel coordinate from the resolution of A to the resolution of B (align_corners=true)
SFM_API Point2f CoordFromTo(const Point2f& coord, const cv::Size& sizeA, const cv::Size& sizeB);

// Map a normalized warp coordinate to the pixel coordinates of an image (align_corners=false)
SFM_API Point2f DenormCoord(const Point2f& normCoord, const cv::Size& size);

// Erode confidence map if requested (helps remove outliers near edges)
SFM_API void ErodeConfidenceMap(Image32F& imgConfidence, int erodeBorder, float minConfidence, float minErodeConfidence);

// Track the DESCRIBED keypoints of imgA into imgB through the given warp and overlap maps;
// trackedA/trackedB/trackStatus are resized to imgA.NumDescribedKeypoints() -- the dense keypoints
// an earlier supplemented pair appended carry no descriptor, so there is nothing for a guided
// descriptor re-match to do with them -- and trackedB is only written where trackStatus is 1.
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
// sitting at the budget regardless), and the count of occupied buckets -- the actual sample size --
// is bounded above by min(E, n^2), not by maxSamples itself. That bound is tight: it is reached
// when the eligible cells are scattered one per bucket rather than packed into a contiguous overlap,
// which drives n up to cover the whole grid without shrinking the number of occupied buckets, and
// it works out to roughly 3.6x maxSamples for a typical warp grid and budget. A contiguous overlap --
// the normal case -- gives back about the budget, since neighbouring eligible cells then share
// buckets instead of each claiming one. Either way the tail is bounded and cheap for whatever runs
// on the sample next (RANSAC's cost grows with the sample, not with the square of it), so it is left
// alone rather than capped here. Callers must therefore read the returned count, and must cope with
// a sample too small -- or, less often, a few times larger than the budget -- for whatever they do
// next.
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
