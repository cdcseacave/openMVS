/*
 * ROMA2Warp.cpp
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

#include "Common.h"
#include "ROMA2Warp.h"
#include "Scene.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ROMA2   "));


Point2f SFM::CoordFromTo(const Point2f& coord, const cv::Size& sizeA, const cv::Size& sizeB) {
	return Point2f(
		coord.x * (float)(sizeB.width  - 1) / (float)(sizeA.width  - 1),
		coord.y * (float)(sizeB.height - 1) / (float)(sizeA.height - 1)
	);
}

Point2f SFM::DenormCoord(const Point2f& normCoord, const cv::Size& size) {
	// adjust for align_corners=False mapping (default PyTorch grid_sample),
	// which adds the 0.5-pixel offset (different from OpenMVS integer = pixel center convention)
	return Point2f(
		0.5f * (normCoord.x + 1.f) * (float)size.width - 0.5f,
		0.5f * (normCoord.y + 1.f) * (float)size.height - 0.5f
	);
}
/*----------------------------------------------------------------*/


// Erode confidence map if requested (helps remove outliers near edges)
void SFM::ErodeConfidenceMap(Image32F& imgConfidence, int erodeBorder, float minConfidence, float minErodeConfidence)
{
	ASSERT(erodeBorder > 0);
	// Create binary mask: 0 for invalid pixels (0.f values), 1 for valid
	Image8U mask(imgConfidence >= minConfidence);
	// Compute distance from each pixel to nearest 0 pixel
	Image32F distMap;
	cv::distanceTransform(mask, distMap, cv::DIST_L2, cv::DIST_MASK_PRECISE);
	// Zero out pixels closer than erodeBorder to invalid pixels, if confidence is below threshold
	for (int y = 0; y < imgConfidence.rows; ++y)
		for (int x = 0; x < imgConfidence.cols; ++x)
			if (distMap(y, x) < erodeBorder && imgConfidence(y, x) < minErodeConfidence)
				imgConfidence(y, x) = 0.f;
}
/*----------------------------------------------------------------*/


size_t SFM::TrackKeypointsByWarp(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& overlap,
	float minConfidence,
	std::vector<Point2f>& trackedA,
	std::vector<Point2f>& trackedB,
	std::vector<uchar>& trackStatus)
{
	ASSERT(!warp.empty() && warp.size() == overlap.size());
	// Track keypoints from A to B using warp and overlap maps.
	// The described prefix only: the tracked points guide a descriptor re-match, and a dense
	// keypoint appended by an earlier supplemented pair has no descriptor to match with. Tracking
	// it would also break the index-parallel contract MatchFeaturesGeometric asserts, since it
	// walks the same prefix.
	const size_t numKp = imgA.NumDescribedKeypoints();
	trackedA.resize(numKp);
	trackedB.resize(numKp);
	trackStatus.resize(numKp);
	size_t numTracked = 0;
	for (size_t i = 0; i < numKp; ++i) {
		const cv::Point2f& kpA = imgA.keypoints[i].pt;
		trackedA[i] = kpA;
		const Point2f wkpA = CoordFromTo(kpA, imgA.GetSize(), warp.size());
		// sampleSafe, not sample: a keypoint on the right/bottom border of A maps exactly onto the
		// last warp column/row, where the bilinear interpolation of sample() reads the (zero-weighted)
		// neighbour one past the end of the grid. Clamping that neighbour to the last cell leaves
		// every interior sample bit-identical and only makes the border reads legal.
		const float ckpB = overlap.sampleSafe(wkpA);
		if (ckpB < minConfidence) {
			trackStatus[i] = 0;
			continue;
		}
		const Point2f nwkpB = warp.sampleSafe(wkpA);
		const Point2f kpB = DenormCoord(nwkpB, imgB.GetSize());
		if (!Image8U::isInside(kpB, imgB.GetSize())) {
			trackStatus[i] = 0;
			continue;
		}
		trackedB[i] = kpB;
		trackStatus[i] = 1;
		++numTracked;
	}
	return numTracked;
}
/*----------------------------------------------------------------*/


namespace {

// One warp cell a draw may pick: a correspondence both the confidence map and the warp admit.
// `cell` is what keeps a sample in warp-grid raster order, which is in turn what makes the
// keypoint indices an append hands out reproducible (AppendDenseMatches).
struct WarpCandidate {
	float confidence;
	int cell;        // y*cols + x of the warp grid, so the sample keeps raster order
	int priority;    // lattice priority of the cell, the bucket winner rule (see WarpCellLatticePriority)
	Point2f ptA, ptB;
};

// PAIR-INDEPENDENT priority of a warp cell: the largest k for which both cell coordinates are
// multiples of 2^k, so cells on a coarse dyadic lattice outrank cells on a finer one and (0,0)
// outranks everything. Depends on the cell alone -- not on this pair's confidences, not on its
// bucket grid -- which is the whole point:
//   The bucket grid is sized per draw (WarpBucketGridSide), so two pairs sharing image A stratify
//   the SAME region of A on grids of different pitch and phase. Picking the most confident cell in
//   each bucket then puts the two draws' A-side points a few cells apart, and the exact-position
//   dedup downstream (FilterRedundantKeypoints, 0.1 px) sees two distinct keypoints where one
//   surface point was sampled twice. A lattice priority makes any two buckets that cover a common
//   region agree on which cell of it to take whenever both find it eligible, so those samples land
//   on the same pixel of A and dedup chains them into one track across pairs.
//   Chaining is through the A SIDE ONLY: the B-side position is whatever the warp maps that cell to,
//   a float that two pairs have no reason to agree on. So a chain grows along the images that play
//   the A role of their pairs, and stops naturally wherever the confident overlaps stop coinciding.
// Confidence remains the ELIGIBILITY test (the eroded minConfidence bar in CollectWarpCandidates),
// it is simply no longer the ranking: every candidate cell is one the warp is confident about, and
// among those the choice may as well be the one that two pairs can both make.
inline int WarpCellLatticePriority(int x, int y)
{
	// trailing zeros of x, with 0 divisible by every power of two (so it never loses a comparison);
	// the warp grid is a few hundred cells wide, so the loop runs at most ~9 times
	const auto TrailingZeros = [](int v) {
		if (v == 0)
			return 31;
		int k = 0;
		while ((v & 1) == 0) { v >>= 1; ++k; }
		return k;
	};
	return MINF(TrailingZeros(x), TrailingZeros(y));
}

// The ONE bucket winner rule, shared by every warp draw: higher lattice priority wins, and a tie
// keeps the candidate seen first. Both draws walk their candidates in warp-grid raster order, so
// "first" is raster order, and the rule is a pure function of the cell coordinates.
inline bool WarpCandidateBeats(const WarpCandidate& candidate, const WarpCandidate& best)
{
	return candidate.priority > best.priority;
}

// Pass 1 of every warp draw: every eligible cell, in raster order. Their count is what the bucket
// grid is sized from, so it has to be known before a single bucket exists.
void CollectWarpCandidates(const cv::Size& sizeA, const cv::Size& sizeB, const Image32F2& warp,
	const Image32F& overlap, float minConfidence, std::vector<WarpCandidate>& candidates)
{
	candidates.clear();
	candidates.reserve((size_t)overlap.rows*overlap.cols/4);
	for (int y = 0; y < overlap.rows; ++y) {
		for (int x = 0; x < overlap.cols; ++x) {
			const float confidence = overlap(y, x);
			if (confidence < minConfidence)
				continue;
			const Point2f ptB(DenormCoord(warp(y, x), sizeB));
			if (!Image8U::isInside(ptB, sizeB))
				continue; // the warp sends this cell outside the second image
			candidates.push_back(WarpCandidate{confidence, y*overlap.cols + x,
				WarpCellLatticePriority(x, y),
				CoordFromTo(Point2f((float)x, (float)y), overlap.size(), sizeA), ptB});
		}
	}
}

// Side n of the n x n bucket grid a warp draw stratifies on, scaled up by the inverse overlap
// fraction (see SampleWarpByCoverage's header for why) so that the buckets which do hold an
// eligible cell number about maxSamples. Integer arithmetic throughout -- n is the smallest value
// with n^2 * E >= maxSamples * T, which is ceil(sqrt(maxSamples*T/E)) without a libm square root
// that could land a hair off a perfect square and shift the whole grid on a different platform.
// Capped at the warp side: beyond one bucket per cell there is nothing left to gain.
int WarpBucketGridSide(const Image32F& overlap, size_t numCandidates, unsigned maxSamples)
{
	ASSERT(numCandidates > 0);
	const uint64_t target = (uint64_t)maxSamples*(uint64_t)overlap.rows*(uint64_t)overlap.cols;
	int numBuckets = 1;
	while (numBuckets < overlap.cols &&
		(uint64_t)numBuckets*(uint64_t)numBuckets*(uint64_t)numCandidates < target)
		++numBuckets;
	return numBuckets;
}

// The bucket a warp grid cell falls in, on a numBuckets x numBuckets grid over the WHOLE warp
inline size_t WarpCellBucket(int cell, const Image32F& overlap, int numBuckets)
{
	const int x = cell % overlap.cols, y = cell / overlap.cols;
	return (size_t)(y*numBuckets/overlap.rows)*numBuckets + x*numBuckets/overlap.cols;
}

} // namespace


size_t SFM::SampleWarpByCoverage(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& overlap,
	float minConfidence,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	float& coverageA,
	float& coverageB)
{
	ASSERT(!warp.empty() && warp.size() == overlap.size());
	sampledA.clear();
	sampledB.clear();
	coverageA = coverageB = 0.f;
	if (maxSamples == 0)
		return 0;
	const cv::Size sizeA(imgA.GetSize()), sizeB(imgB.GetSize());
	std::vector<WarpCandidate> candidates;
	CollectWarpCandidates(sizeA, sizeB, warp, overlap, minConfidence, candidates);
	if (candidates.empty())
		return 0;

	std::vector<int> chosen;
	if (candidates.size() <= maxSamples) {
		// the confident overlap already fits in the budget: there is nothing to select, and any
		// stratification would only be able to throw points away
		chosen.resize(candidates.size());
		std::iota(chosen.begin(), chosen.end(), 0);
	} else {
		// pass 2: one winner per bucket of the n x n grid over the whole warp
		const int numBuckets = WarpBucketGridSide(overlap, candidates.size(), maxSamples);
		std::vector<int> bucketBest((size_t)numBuckets*numBuckets, -1);
		FOREACH(i, candidates) {
			const WarpCandidate& candidate = candidates[i];
			int& best = bucketBest[WarpCellBucket(candidate.cell, overlap, numBuckets)];
			if (best < 0 || WarpCandidateBeats(candidate, candidates[best]))
				best = (int)i; // the one winner rule: lattice priority, ties to raster order
		}
		chosen.reserve(MINF(bucketBest.size(), candidates.size()));
		for (const int idxCandidate : bucketBest)
			if (idxCandidate >= 0)
				chosen.push_back(idxCandidate);
		// hand the sample back in raster order, independent of the bucket traversal above
		std::sort(chosen.begin(), chosen.end());
	}
	sampledA.reserve(chosen.size());
	sampledB.reserve(chosen.size());
	for (const int idxCandidate : chosen) {
		const WarpCandidate& candidate = candidates[idxCandidate];
		sampledA.push_back(candidate.ptA);
		sampledB.push_back(candidate.ptB);
	}
	ComputeSampleCoverage(sampledA, sampledB, sizeA, sizeB, std::vector<uint32_t>(), coverageA, coverageB);
	return sampledA.size();
}
/*----------------------------------------------------------------*/


void SFM::ThinSampleEvenly(
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences,
	unsigned maxSamples)
{
	ASSERT(sampledA.size() == sampledB.size() && sampledA.size() == confidences.size());
	const size_t numSamples = sampledA.size();
	if (numSamples <= maxSamples)
		return;
	// an even stride through the sample's own order, which is the warp raster order every draw
	// hands back: strictly increasing (numSamples > maxSamples), so the survivors keep that order
	// and the keypoint indices an append hands out stay reproducible
	for (size_t i = 0; i < maxSamples; ++i) {
		const size_t src = i*numSamples/maxSamples;
		sampledA[i] = sampledA[src];
		sampledB[i] = sampledB[src];
		confidences[i] = confidences[src];
	}
	sampledA.resize(maxSamples);
	sampledB.resize(maxSamples);
	confidences.resize(maxSamples);
}
/*----------------------------------------------------------------*/


size_t SFM::SampleWarpComplementary(
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
	WarpDrawCoverage* coverage)
{
	ASSERT(!warp.empty() && warp.size() == overlap.size());
	// cleared before any early return: the draw is a pure function of its inputs, so a caller
	// reusing its buffers must get exactly what a caller passing empty ones gets
	sampledA.clear();
	sampledB.clear();
	confidences.clear();
	if (coverage)
		*coverage = WarpDrawCoverage();
	if (maxSamples == 0)
		return 0;
	const cv::Size sizeA(imgA.GetSize()), sizeB(imgB.GetSize());
	std::vector<WarpCandidate> candidates;
	CollectWarpCandidates(sizeA, sizeB, warp, overlap, minConfidence, candidates);
	if (candidates.empty())
		return 0;
	// the grid is sized for THIS draw's budget, which is what makes the sample complementary in
	// scale as well as in position: a pair whose sparse matches already fill most of its budget
	// asks for few dense points and gets a coarse grid, one that has almost none asks for many and
	// gets a fine one
	const int numBuckets = WarpBucketGridSide(overlap, candidates.size(), maxSamples);
	// every bucket an already-verified correspondence sits in is out of the draw. Occupancy is
	// measured in image A only, deliberately: the warp grid lives in A's frame, so that is the one
	// frame where a sparse keypoint position and a warp cell are directly comparable. On a genuine
	// pair -- the only kind that reaches here, the gate having validated it -- the warp is close
	// enough to a diffeomorphism that the B-side density mirrors A's through it, so a second grid
	// in B (which would also need a B->A back-map the coarse warp does not carry) would mark the
	// same buckets.
	std::vector<bool> occupied((size_t)numBuckets*numBuckets, false);
	for (const Point2f& pt : occupiedA) {
		// inverse of the cell -> imgA pixel map the draw itself uses: CoordFromTo is linear and
		// carries no half-pixel term, so its inverse carries none either. Rounded to the nearest
		// cell, and clamped because a keypoint may sit on the very border of A.
		const Point2f cell(CoordFromTo(pt, sizeA, overlap.size()));
		const int x = MINF(MAXF(ROUND2INT(cell.x), 0), overlap.cols-1);
		const int y = MINF(MAXF(ROUND2INT(cell.y), 0), overlap.rows-1);
		occupied[WarpCellBucket(y*overlap.cols + x, overlap, numBuckets)] = true;
	}
	// one winner per UNOCCUPIED bucket. An occupied bucket contributes nothing rather than a
	// reduced quota: "up to" a budget is a ceiling, not a target that has to be filled, and the
	// point of the draw is the part of the frame the sparse matches left empty.
	// The same pass censuses the buckets (WarpDrawCoverage): which of them hold an eligible cell at
	// all -- the pair's valid disparity area on this grid -- and how many of THOSE the caller's own
	// correspondences already occupy. Counted here rather than by a second sweep because this loop
	// is the only place both facts are known per bucket, and counted over the eligible cells alone
	// so that an occupied position outside the valid area (possible: occupancy is rounded to the
	// nearest cell and a sparse inlier may sit where the warp is not confident) cannot report more
	// covered area than there is.
	std::vector<bool> hasCandidate((size_t)numBuckets*numBuckets, false);
	unsigned numConfidentBuckets = 0, numOccupiedBuckets = 0;
	std::vector<int> bucketBest((size_t)numBuckets*numBuckets, -1);
	FOREACH(i, candidates) {
		const WarpCandidate& candidate = candidates[i];
		const size_t bucket = WarpCellBucket(candidate.cell, overlap, numBuckets);
		if (!hasCandidate[bucket]) {
			hasCandidate[bucket] = true;
			++numConfidentBuckets;
			if (occupied[bucket])
				++numOccupiedBuckets;
		}
		if (occupied[bucket])
			continue;
		int& best = bucketBest[bucket];
		if (best < 0 || WarpCandidateBeats(candidate, candidates[best]))
			best = (int)i; // the one winner rule: lattice priority, ties to raster order
	}
	if (coverage)
		*coverage = WarpDrawCoverage{numConfidentBuckets, numOccupiedBuckets, numBuckets};
	std::vector<int> chosen;
	chosen.reserve(MINF(bucketBest.size(), candidates.size()));
	for (const int idxCandidate : bucketBest)
		if (idxCandidate >= 0)
			chosen.push_back(idxCandidate);
	std::sort(chosen.begin(), chosen.end()); // raster order, independent of the bucket traversal above
	sampledA.reserve(chosen.size());
	sampledB.reserve(chosen.size());
	confidences.reserve(chosen.size());
	for (const int idxCandidate : chosen) {
		const WarpCandidate& candidate = candidates[idxCandidate];
		sampledA.push_back(candidate.ptA);
		sampledB.push_back(candidate.ptB);
		// the cell's own confidence, not a bilinear read-back of the map at the point it produced:
		// the two differ by rounding, and this one is what the winner was actually chosen on
		confidences.push_back(candidate.confidence);
	}
	// the unoccupied-bucket count is bounded by min(E, n^2) and not by maxSamples (see
	// SampleWarpByCoverage's header), so a draw can still come out over its budget
	ThinSampleEvenly(sampledA, sampledB, confidences, maxSamples);
	return sampledA.size();
}
/*----------------------------------------------------------------*/


size_t SFM::SampleWarpComplementary(
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
	WarpDrawCoverage* coverage)
{
	// the pair's sparse evidence, in imgA's pixels -- see the header for why each of the three
	// choices made here (the segment, the index side, the image) is the one that makes the draw
	// complement anything
	ASSERT(pair.GetNumDenseInliers() == 0); // a pair is supplemented once, before it has a dense segment
	const unsigned numSparse = pair.GetNumFilteredInliers();
	std::vector<Point2f> occupiedA;
	occupiedA.reserve(numSparse);
	for (unsigned m = 0; m < numSparse; ++m) {
		// the sparse segment is described at both ends on every path that produces it
		// (ImagePair::CheckSparseSegmentIsDescribed), so this index is inside imgA's described prefix
		ASSERT((size_t)pair.matches[m].queryIdx < imgA.NumDescribedKeypoints());
		occupiedA.push_back(imgA.keypoints[pair.matches[m].queryIdx].pt);
	}
	return SampleWarpComplementary(imgA, imgB, warp, overlap, minConfidence, maxSamples,
		occupiedA, sampledA, sampledB, confidences, coverage);
}
/*----------------------------------------------------------------*/


void SFM::ComputeSampleCoverage(
	const std::vector<Point2f>& sampledA,
	const std::vector<Point2f>& sampledB,
	const cv::Size& sizeA,
	const cv::Size& sizeB,
	const std::vector<uint32_t>& indices,
	float& coverageA,
	float& coverageB)
{
	ASSERT(sampledA.size() == sampledB.size());
	std::vector<bool> gridA((size_t)DENSE_COVERAGE_GRID*DENSE_COVERAGE_GRID, false), gridB(gridA);
	const auto MarkCell = [](std::vector<bool>& grid, const Point2f& pt, const cv::Size& size) {
		const unsigned cx = MINF((unsigned)MAXF(0.f, (float)DENSE_COVERAGE_GRID*pt.x/(float)size.width), DENSE_COVERAGE_GRID-1);
		const unsigned cy = MINF((unsigned)MAXF(0.f, (float)DENSE_COVERAGE_GRID*pt.y/(float)size.height), DENSE_COVERAGE_GRID-1);
		grid[(size_t)cy*DENSE_COVERAGE_GRID + cx] = true;
	};
	const auto MarkOne = [&](size_t i) {
		MarkCell(gridA, sampledA[i], sizeA);
		MarkCell(gridB, sampledB[i], sizeB);
	};
	if (indices.empty()) {
		for (size_t i = 0; i < sampledA.size(); ++i)
			MarkOne(i);
	} else {
		for (const uint32_t i : indices) {
			ASSERT(i < sampledA.size());
			MarkOne(i);
		}
	}
	coverageA = (float)std::count(gridA.begin(), gridA.end(), true)/(float)gridA.size();
	coverageB = (float)std::count(gridB.begin(), gridB.end(), true)/(float)gridB.size();
}
/*----------------------------------------------------------------*/


unsigned SFM::AppendDenseMatches(Scene& scene, ImagePair& pair,
	const std::vector<Point2f>& pointsA, const std::vector<Point2f>& pointsB,
	const std::vector<float>& confidences, const cv::Size& warpSize)
{
	ASSERT(pointsA.size() == pointsB.size() && pointsA.size() == confidences.size());
	ASSERT(warpSize.width > 0 && warpSize.height > 0);
	if (pointsA.empty())
		return 0;
	Image& imgA = scene.images[pair.ID1];
	Image& imgB = scene.images[pair.ID2];
	// the scale the positions were sampled at: the pixel footprint of one warp cell in each image
	const auto WarpCellSize = [&warpSize](const Image& img) {
		return MAXF((float)img.GetWidth()/(float)warpSize.width, (float)img.GetHeight()/(float)warpSize.height);
	};
	const float cellSizeA = WarpCellSize(imgA);
	const float cellSizeB = WarpCellSize(imgB);
	// the described prefix of each image ends where it stands now; a second supplemented pair on
	// the same image finds the boundary already closed and appends past the dense keypoints the
	// first one left, which is what lets FilterRedundantKeypoints reuse a coinciding one
	imgA.CloseDescribedKeypoints();
	imgB.CloseDescribedKeypoints();
	const uint32_t baseA = (uint32_t)imgA.keypoints.size();
	const uint32_t baseB = (uint32_t)imgB.keypoints.size();
	std::vector<DMatch> dense;
	dense.reserve(pointsA.size());
	for (uint32_t i = 0; i < (uint32_t)pointsA.size(); ++i) {
		imgA.keypoints.push_back(Image::MakeDenseKeypoint(pointsA[i], confidences[i], cellSizeA));
		imgB.keypoints.push_back(Image::MakeDenseKeypoint(pointsB[i], confidences[i], cellSizeB));
		dense.emplace_back(baseA + i, baseB + i);
	}
	// The supplement becomes the middle segment of `matches` (see the partition comment in
	// ImagePair.h): after the sparse inliers, which stay the pair's descriptor evidence and are what
	// GetNumFilteredInliers() counts, and before the RANSAC inliers the strict filter rejected,
	// which stay outside the track-forming prefix where they belong. Inside the track-forming prefix
	// rather than at the end because BuildTracks reads only that prefix -- appended past it the
	// whole supplement would be inert -- and outside the sparse count because a coverage-maximising
	// draw must not re-rank the view graph.
	if (pair.numFilteredInliers < 0) {
		// no strict filter ran on this pair, which is the same statement as "every match is an
		// inlier"; materialise that so the sparse count stays a count of sparse matches once the
		// dense ones are in the array
		pair.numFilteredInliers = (int)pair.matches.size();
	}
	const size_t at = (size_t)pair.numFilteredInliers + (size_t)pair.numDenseInliers;
	ASSERT(at <= pair.matches.size());
	pair.matches.insert(pair.matches.begin() + at, dense.begin(), dense.end());
	pair.numDenseInliers += (int)dense.size();
	// the two per-pair statistics derived from the partition this just changed. The discounted
	// inlier count is invalidated rather than recomputed, because the weight it is discounted by
	// lives in the weighting pass and not here; the ray angle IS recomputed, because nothing else
	// will -- a pair infused here is not re-filtered afterwards (deliberately: the supplement must
	// not go through the strict geometric filter), and a pair left claiming a baseline measured on
	// its sparse matches alone -- or, on a dense-only pair, none at all -- would keep the one weight
	// term that can demote a degenerate baseline unavailable on exactly this population
	pair.weightedInliers = -1.f;
	if (pair.relativePose.has_value())
		pair.meanRayAngle = pair.ComputeMeanRayAngle(imgA, imgB);
	return (unsigned)dense.size();
}
/*----------------------------------------------------------------*/


bool SFM::ApplyROMA2Pair(Scene& scene, std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap, ImagePair&& pair, unsigned maxReplaceInliers, bool& bCreated, bool bCreateOnly)
{
	ASSERT(pair.ID1 < pair.ID2 && (!pair.matches.empty() || bCreateOnly));
	const PairIdx::PairIndex key = PairIdx(pair.ID1, pair.ID2).idx;
	const auto it = pairIndexMap.find(key);
	if (it != pairIndexMap.end()) {
		ImagePair& scenePair = scene.pairs[it->second];
		const unsigned existingInliers = scenePair.GetNumWeightedInliers();
		if (bCreateOnly) {
			// a dense-only candidate: the existing pair keeps its place, whatever the two counts say
			DEBUG_ULTIMATE("ROMA2 pair (% 4u, % 4u) kept: %u existing inliers vs a dense-only candidate", pair.ID1, pair.ID2, existingInliers);
			return false;
		}
		// polycpp ShouldReplaceROMA2Pair (import_roma2.hpp:39-45): strictly more inliers, and the existing pair below the ceiling
		// both sides through the same accessor: comparing the candidate's descriptor evidence against
		// the incumbent's dense-inclusive evidence would be two different questions in one test
		if (pair.GetNumWeightedInliers() <= existingInliers || (maxReplaceInliers > 0 && existingInliers >= maxReplaceInliers)) {
			DEBUG_ULTIMATE("ROMA2 pair (% 4u, % 4u) kept: %u existing vs %u guided inliers", pair.ID1, pair.ID2, existingInliers, pair.GetNumWeightedInliers());
			return false;
		}
		scenePair = std::move(pair);
		bCreated = false;
		return true;
	}
	// overlapRatio/overlapArea stay at their reset value (0): a created pair is weighted exactly
	// like any other pair, ComputePairsWeights computing its own overlap proxy for it. Stamping a
	// full 1/1 overlap here (what the old NPZ import did) would survive PairsMatcher::Match --
	// nothing else writes overlapRatio, and the weighting only fills in a still-zero overlapArea --
	// and hand every dense-created pair a best-possible overlap score it was never measured to have
	pairIndexMap.emplace(key, (IIndex)scene.pairs.size());
	scene.pairs.emplace_back(std::move(pair));
	bCreated = true;
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
