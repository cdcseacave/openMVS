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
// and ONE cell of each bucket is taken -- one per bucket, and nothing else. Which one is the shared
// winner rule of every warp draw (ROMA2Warp.cpp, WarpCellLatticePriority): the eligible cell with
// the highest pair-independent lattice priority, i.e. the one whose grid coordinates are both
// divisible by the largest power of two, ties going to raster order. Confidence is the eligibility
// test, not the ranking -- a rule that reads this pair's confidences would pick a different cell of
// the same region for the next pair, and the cross-pair keypoint identity the supplement's tracks
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

// Bucket census of one complementary draw: how much of the pair's VALID DISPARITY AREA -- the part
// of the warp the confidence map (after the erosion) admits and the warp sends inside the second
// image, which is exactly the area the dense two-view gate judged the pair on -- the correspondences
// the caller already has do cover.
// Measured on the draw's own n x n bucket grid, the fine one, so a region the sparse matcher covered
// sparsely still reads as largely uncovered: `numConfidentBuckets` are the buckets holding at least
// one eligible warp cell, `numOccupiedBuckets` those of them an `occupiedA` position also lands in.
// Coverage() is their ratio, and 1 - Coverage() is the share of the valid area still to be filled --
// what the caller sizes the dense budget from (MatchROMA2.cpp, DrawDenseSupplement).
struct SFM_API WarpDrawCoverage {
	unsigned numConfidentBuckets = 0; // buckets holding at least one eligible (confident, in-frame) warp cell
	unsigned numOccupiedBuckets = 0;  // ...of those, the ones an already-held correspondence occupies
	int numBuckets = 0;               // side n of the n x n grid the census was taken on
	// Occupied share of the valid disparity area, in [0,1]. A warp with no eligible cell at all has
	// no valid area to cover, and reads as 0 (uncovered) rather than as fully covered: the draw
	// returns nothing on it either way, so the two differ only in what the caller logs.
	inline float Coverage() const {
		return numConfidentBuckets > 0 ? (float)numOccupiedBuckets/(float)numConfidentBuckets : 0.f;
	}
};

// Thin an index-parallel warp sample down to at most maxSamples correspondences by an even stride
// through its order, in place. Never by confidence: a confidence sort would re-cluster the survivors
// onto the warp's most certain region, which is the textured region the sparse matcher already
// covered, and undo the stratification the draw exists for. A sample already within the budget is
// left exactly as it is, and a zero budget empties it.
// Shared by SampleWarpComplementary (which caps its own draw with it) and by the caller that thins a
// full draw to a budget only the finished draw's coverage could tell it (DrawDenseSupplement), so
// the two can never thin differently.
SFM_API void ThinSampleEvenly(
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences,
	unsigned maxSamples);

// Draw a sample of confident correspondences out of an (already eroded) warp that COMPLEMENTS
// correspondences the caller already has: the dense supplement of a weak pair, drawn so that the
// union of the pair's verified sparse matches and this sample is spread as evenly as the warp
// allows. Same eligibility rule, same n x n bucket stratification and the same in-bucket winner rule
// as SampleWarpByCoverage, with n sized from maxSamples -- this draw's budget, i.e. the total the
// supplemented pair is drawn against -- and with two differences:
//
//  - `occupiedA` are positions in imgA's pixels (its verified sparse inliers) whose buckets are
//    struck out of the draw entirely. An occupied bucket yields NO dense point: "up to" a budget
//    is a ceiling, not a quota to fill, and a bucket the sparse matcher already covered is exactly
//    where a dense point adds nothing. Occupancy is read in A's frame alone -- the frame the warp
//    grid lives in, and the only one where a keypoint position and a warp cell are directly
//    comparable; on a genuine pair the warp carries that density over to B.
//  - maxSamples is a real CAP here, not only the target it is for the gate: a pair has a match
//    budget, so a draw whose unoccupied-bucket count still runs over it is thinned by an even stride
//    through the raster order (ThinSampleEvenly).
//
// sampledA/sampledB/confidences are index-parallel and in warp-grid raster order (the order
// AppendDenseMatches needs to hand out reproducible keypoint indices), sampledA/sampledB in the
// pixels of the working orientation of imgA/imgB, and confidences carrying each winning cell's own
// overlap value -- the value it was selected on -- for Image::MakeDenseKeypoint.
// `coverage`, when given, receives the bucket census of THIS draw (see WarpDrawCoverage): the
// caller's own share of the valid disparity area, measured on the grid the draw just used and
// before any thinning, which is the only point where both numbers exist.
// The draw is a pure function of its inputs: same inputs, same output, on any thread.
// Returns the number of sampled correspondences (<= maxSamples).
SFM_API size_t SampleWarpComplementary(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& overlap,
	float minConfidence,
	unsigned maxSamples,
	const std::vector<Point2f>& occupiedA,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences,
	WarpDrawCoverage* coverage = NULL);

// Same draw, taking the occupied positions from `pair` itself: the image-A keypoint positions of
// its verified SPARSE inliers. This is the form a supplemented pair actually uses, and it exists as
// its own entry point because the gather is where a silent frame or segment mistake would live --
// three choices, each with a plausible-looking wrong answer that still compiles and still returns a
// supplement, only one that complements anything:
//   - the SPARSE segment `matches[0, GetNumFilteredInliers())`, which is the pair's descriptor
//     evidence -- not GetNumTrackFormingMatches(), whose second half is an earlier supplement, and
//     not the whole of `matches`, whose tail is what the strict filter REJECTED and which must
//     therefore stay open for the dense draw;
//   - `queryIdx`, the imgA index of a match (AppendDenseMatches emits `(baseA+i, baseB+i)`);
//   - imgA's keypoints, because the warp grid lives in imgA's frame (see the note above).
// `pair` must be the verified pair of imgA/imgB in that order (pair.ID1 -> imgA), and must carry no
// dense segment yet. A pair with no sparse inliers at all (the dense-only pair of a validated pair
// whose guided SIFT pass failed) occupies nothing, so its census reads coverage 0 and its draw is
// the whole valid area -- which is the point.
SFM_API size_t SampleWarpComplementary(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& overlap,
	float minConfidence,
	unsigned maxSamples,
	const ImagePair& pair,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences,
	WarpDrawCoverage* coverage = NULL);

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
