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

// Dense correspondence maps of an image pair in ONE direction, as produced by the ROMAv2 coarse
// matcher: one cell per warp grid position of the source image, holding where that cell lands in
// the target image and how confident the model is that both images see it
struct SFM_API WarpMaps {
	Image32F2 warp;      // normalized (align_corners=false) target coordinates, in [-1,1]
	Image32F confidence; // matching confidence, in [0,1]

	inline bool IsValid() const {
		return !warp.empty() && warp.size() == confidence.size();
	}
};
/*----------------------------------------------------------------*/

// Both directions of one (A,B) pair. They come out of a single coarse-match call -- the model
// produces the B->A field from the same pair-ViT pass as the A->B one -- so they are one statement
// about the pair and travel together: the verdict measures its inlier area on A's cells through
// `ab` and on B's cells through `ba`, and only the min of the two admits the pair (MatchROMA2.h,
// JudgePairROMA2).
struct SFM_API PairWarps {
	WarpMaps ab, ba;

	inline bool IsValid() const {
		return ab.IsValid() && ba.IsValid() && ab.warp.size() == ba.warp.size();
	}
};
/*----------------------------------------------------------------*/

// Map a pixel coordinate from the resolution of A to the resolution of B (align_corners=true)
SFM_API Point2f CoordFromTo(const Point2f& coord, const cv::Size& sizeA, const cv::Size& sizeB);

// Map a normalized warp coordinate to the pixel coordinates of an image (align_corners=false)
SFM_API Point2f DenormCoord(const Point2f& normCoord, const cv::Size& size);

// Half a warp cell in image pixels: the accuracy a coarse-warp correspondence can claim, and the
// epipolar tolerance every test on warp cells uses -- the verdict's Sampson test on both sides of
// the pair (JudgePairROMA2) and the dense segment's classification (AssemblePairROMA2). A function
// of the two image sizes and the warp grid rather than a setting, because it is not a precision the
// user chooses but the one the grid has: measured at 2 px on the 640/160 grid.
inline float WarpTolerance(const cv::Size& sizeA, const cv::Size& sizeB, int warpSize) {
	return 0.5f * (float)MAXF(MAXF(sizeA.width, sizeA.height), MAXF(sizeB.width, sizeB.height)) / (float)warpSize;
}
/*----------------------------------------------------------------*/

// Track the DESCRIBED keypoints of imgA into imgB through the given warp and confidence maps;
// trackedA/trackedB/trackStatus are resized to imgA.NumDescribedKeypoints() -- the dense keypoints
// an earlier pair appended carry no descriptor, so there is nothing for a guided descriptor match
// to do with them -- and trackedB is only written where trackStatus is 1.
// Returns the number of tracked keypoints.
SFM_API size_t TrackKeypointsByWarp(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	std::vector<Point2f>& trackedA,
	std::vector<Point2f>& trackedB,
	std::vector<uchar>& trackStatus);

// Draw a spatially uniform sample of confident correspondences out of a warp.
// A cell is eligible when it is at least minConfidence confident and its warped point lands inside
// imgB; let E be how many of the grid's T cells are. If E <= maxSamples the whole confident overlap
// is taken. Otherwise the warp grid is stratified into n x n buckets over the WHOLE image, with
//    n = min(warpSize, ceil(sqrt(maxSamples * T / E))),
// and ONE cell of each bucket is taken -- one per bucket, and nothing else. Which one is the shared
// winner rule of every warp draw (ROMA2Warp.cpp, WarpCellLatticePriority): the eligible cell with
// the highest pair-independent lattice priority, i.e. the one whose grid coordinates are both
// divisible by the largest power of two, ties going to raster order. Confidence is the eligibility
// test, not the ranking -- a rule that reads this pair's confidences would pick a different cell of
// the same region for the next pair, and the cross-pair keypoint identity the dense fill's tracks
// are built on needs the two to agree (see WarpCellLatticePriority for the full argument).
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
// image that the sample occupies; because the draw is uniform over the whole frame rather than
// weighted toward wherever confidence is highest, that fraction is a fair proxy for the true
// overlap area, and a pair whose overlap is a corner of the frame must read as such.
// Returns the number of sampled correspondences.
SFM_API size_t SampleWarpByCoverage(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	float& coverageA,
	float& coverageB);

// Thin an index-parallel warp sample down to at most maxSamples correspondences by an even stride
// through its order, in place. Never by confidence: a confidence sort would re-cluster the survivors
// onto the warp's most certain region, which is the textured region the sparse matcher already
// covered, and undo the stratification the draw exists for. A sample already within the budget is
// left exactly as it is, and a zero budget empties it.
SFM_API void ThinSampleEvenly(
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences,
	unsigned maxSamples);

// Draw a sample of confident correspondences out of a warp that COMPLEMENTS correspondences the
// caller already has: the dense fill of an admitted pair, drawn so that the union of the pair's
// guided sparse matches and this sample is spread as evenly as the warp allows. Same eligibility
// rule, same n x n bucket stratification and the same in-bucket winner rule as SampleWarpByCoverage,
// with n sized from maxSamples -- this draw's budget -- and with two differences:
//
//  - `occupiedA` are positions in imgA's pixels (the pair's guided matches) whose buckets are
//    struck out of the draw entirely. An occupied bucket yields NO dense point: "up to" a budget
//    is a ceiling, not a quota to fill, and a bucket the sparse matcher already covered is exactly
//    where a dense point adds nothing. Occupancy is read in A's frame alone -- the frame the warp
//    grid lives in, and the only one where a keypoint position and a warp cell are directly
//    comparable; on a genuine pair the warp carries that density over to B.
//  - maxSamples is a real CAP here, not only the target it is for the verdict: a pair has a match
//    budget, so a draw whose unoccupied-bucket count still runs over it is thinned by an even stride
//    through the raster order (ThinSampleEvenly).
//
// sampledA/sampledB/confidences are index-parallel and in warp-grid raster order (the order
// AppendDenseMatches needs to hand out reproducible keypoint indices), sampledA/sampledB in the
// pixels of the working orientation of imgA/imgB, and confidences carrying each winning cell's own
// confidence value -- the value it was selected on -- for Image::MakeDenseKeypoint.
// The draw is a pure function of its inputs: same inputs, same output, on any thread.
// Returns the number of sampled correspondences (<= maxSamples).
SFM_API size_t SampleWarpComplementary(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	unsigned maxSamples,
	const std::vector<Point2f>& occupiedA,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences);

// Append one pair's dense (ROMAv2 warp) correspondences to the scene, alongside the sparse matches
// the pair already carries. Each correspondence becomes one keypoint at the end of each image's
// keypoint array -- past its described prefix, which is closed here if it was still open -- and one
// match between the two. pointsA/pointsB/confidences are index-parallel and in the pixels of the
// working orientation of each image (SampleWarpByCoverage's convention); warpSize is the warp grid
// the sample was drawn on, which fixes the size of the appended keypoints (Image::MakeDenseKeypoint).
//
// The matches become the pair's dense segment, `[numFilteredInliers, +numDenseInliers)` of `matches`
// (the partition is documented on ImagePair's members): inside the track-forming prefix, because
// BuildTracks reads only GetNumTrackFormingMatches() matches and a match appended past that prefix
// would form no track at all, and outside GetNumFilteredInliers(), because that count is the pair's
// descriptor evidence and a coverage-maximising draw must not re-rank the view graph with it.
//
// Callers must invoke this serially and in a fixed pair order: the keypoint indices it hands out
// depend on how many keypoints the two images already carry, so a parallel or completion-ordered
// append would give a different labelling run to run.
// Returns the number of matches added.
SFM_API unsigned AppendDenseMatches(
	Scene& scene,
	ImagePair& pair,
	const std::vector<Point2f>& pointsA,
	const std::vector<Point2f>& pointsB,
	const std::vector<float>& confidences,
	const cv::Size& warpSize);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_ROMA2WARP_H_
