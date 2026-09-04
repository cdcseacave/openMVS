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

Point2f SFM::NormCoord(const Point2f& coord, const cv::Size& size) {
	// the exact inverse of DenormCoord above, and it stays next to it so the align_corners=False
	// half-pixel convention is written down once
	return Point2f(
		2.f * (coord.x + 0.5f) / (float)size.width - 1.f,
		2.f * (coord.y + 0.5f) / (float)size.height - 1.f
	);
}
/*----------------------------------------------------------------*/


size_t SFM::TrackKeypointsByWarp(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	std::vector<Point2f>& trackedA,
	std::vector<Point2f>& trackedB,
	std::vector<uchar>& trackStatus)
{
	ASSERT(!warp.empty() && warp.size() == confidence.size());
	// Track keypoints from A to B using the warp and confidence maps.
	// The described prefix only: the tracked points guide a descriptor match, and a dense keypoint
	// appended by an earlier pair has no descriptor to match with. Tracking it would also break the
	// index-parallel contract MatchFeaturesGuided asserts, since it walks the same prefix.
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
		const float ckpB = confidence.sampleSafe(wkpA);
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
// Confidence remains the ELIGIBILITY test (the minConfidence bar in CollectWarpCandidates), it is
// simply no longer the ranking: every candidate cell is one the warp is confident about, and among
// those the choice may as well be the one that two pairs can both make.
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

// A PAIR-INDEPENDENT SCRAMBLE of a warp cell index -- an integer finalizer (multiply-xorshift), not
// randomness. It orders the cells that share a lattice level against each other, which is what the
// thinning below needs to break its ties, and the thinning is only correct because this is a pure
// function of the cell: two pairs that reach the same cell scramble it to the same value and so
// agree on whether it survives. Anything that varied per pair -- a random draw, a running counter,
// the pair's own confidences -- would put the two back to keeping different subsets of the same
// winners, which is the very thing the fixed pitch exists to prevent.
// Raster order would be pair-independent too, and is not usable: a prefix of it is the top band of
// the overlap, so a thinning that took one would delete the bottom of every draw it binds on. The
// scramble spreads a prefix over the whole overlap instead.
inline uint32_t WarpCellScramble(int cell)
{
	uint32_t h = (uint32_t)cell;
	h ^= h >> 16; h *= 0x7feb352dU;
	h ^= h >> 15; h *= 0x846ca68bU;
	h ^= h >> 16;
	return h;
}

// The ONE bucket winner rule, shared by every warp draw: higher lattice priority wins, and a tie
// keeps the candidate seen first. Both draws walk their candidates in warp-grid raster order, so
// "first" is raster order, and the rule is a pure function of the cell coordinates.
inline bool WarpCandidateBeats(const WarpCandidate& candidate, const WarpCandidate& best)
{
	return candidate.priority > best.priority;
}

// Pass 1 of every warp draw: every eligible cell, in raster order -- the population the bucket grid
// stratifies. SampleWarpByCoverage still sizes its grid from their count (WarpBucketGridSide);
// SampleWarpComplementary's grid is the caller's pitch instead, but needs this same pass to know
// which cells there are to put in it.
void CollectWarpCandidates(const cv::Size& sizeA, const cv::Size& sizeB, const Image32F2& warp,
	const Image32F& confidence, float minConfidence, std::vector<WarpCandidate>& candidates)
{
	candidates.clear();
	candidates.reserve((size_t)confidence.rows*confidence.cols/4);
	for (int y = 0; y < confidence.rows; ++y) {
		for (int x = 0; x < confidence.cols; ++x) {
			const float conf = confidence(y, x);
			Point2f ptB;
			if (!IsWarpCellEligible(conf, warp(y, x), minConfidence, sizeB, ptB))
				continue;
			candidates.push_back(WarpCandidate{conf, y*confidence.cols + x,
				WarpCellLatticePriority(x, y),
				CoordFromTo(Point2f((float)x, (float)y), confidence.size(), sizeA), ptB});
		}
	}
}

// Side n of the n x n bucket grid a warp draw stratifies on, scaled up by the inverse overlap
// fraction (see SampleWarpByCoverage's header for why) so that the buckets which do hold an
// eligible cell number about maxSamples. Integer arithmetic throughout -- n is the smallest value
// with n^2 * E >= maxSamples * T, which is ceil(sqrt(maxSamples*T/E)) without a libm square root
// that could land a hair off a perfect square and shift the whole grid on a different platform.
// Capped at the warp side: beyond one bucket per cell there is nothing left to gain.
int WarpBucketGridSide(const Image32F& confidence, size_t numCandidates, unsigned maxSamples)
{
	ASSERT(numCandidates > 0);
	const uint64_t target = (uint64_t)maxSamples*(uint64_t)confidence.rows*(uint64_t)confidence.cols;
	int numBuckets = 1;
	while (numBuckets < confidence.cols &&
		(uint64_t)numBuckets*(uint64_t)numBuckets*(uint64_t)numCandidates < target)
		++numBuckets;
	return numBuckets;
}

// The bucket a warp grid cell falls in, on a numBuckets x numBuckets grid over the WHOLE warp
inline size_t WarpCellBucket(int cell, const Image32F& confidence, int numBuckets)
{
	const int x = cell % confidence.cols, y = cell / confidence.cols;
	return (size_t)(y*numBuckets/confidence.rows)*numBuckets + x*numBuckets/confidence.cols;
}

} // namespace


size_t SFM::SampleWarpByCoverage(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB)
{
	ASSERT(!warp.empty() && warp.size() == confidence.size());
	sampledA.clear();
	sampledB.clear();
	if (maxSamples == 0)
		return 0;
	const cv::Size sizeA(imgA.GetSize()), sizeB(imgB.GetSize());
	std::vector<WarpCandidate> candidates;
	CollectWarpCandidates(sizeA, sizeB, warp, confidence, minConfidence, candidates);
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
		const int numBuckets = WarpBucketGridSide(confidence, candidates.size(), maxSamples);
		std::vector<int> bucketBest((size_t)numBuckets*numBuckets, -1);
		FOREACH(i, candidates) {
			const WarpCandidate& candidate = candidates[i];
			int& best = bucketBest[WarpCellBucket(candidate.cell, confidence, numBuckets)];
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
	return sampledA.size();
}
/*----------------------------------------------------------------*/


void SFM::ThinSampleByLatticePriority(
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences,
	std::vector<int>& cells,
	int warpCols,
	unsigned maxSamples)
{
	ASSERT(sampledA.size() == sampledB.size() && sampledA.size() == confidences.size() &&
		sampledA.size() == cells.size());
	ASSERT(warpCols > 0);
	const size_t numSamples = sampledA.size();
	if (numSamples <= maxSamples)
		return;
	// Rank by the SAME pair-independent key the bucket winners were picked on, then keep a prefix
	// of it: lattice priority first, the cell's scramble to break the level it stops inside. Two
	// pairs sharing image A therefore keep NESTED prefixes of one order over the cells rather than
	// two subsets that happen not to meet -- the smaller budget's survivors are the larger's, cell
	// for cell, so everything they still share is a keypoint at the same pixel of A and chains.
	// (A stride through this pair's own list, which is what this used to be, has survivors that
	// depend on the pair's sample count and the pair's ceiling, so two pairs that agreed on every
	// winner kept different subsets of them and the chaining fell to the product of the two
	// thinning ratios.) The priority is dyadic -- a level-k cell lies on a 2^k lattice -- so the
	// prefix that keeps whole levels is spatially uniform whatever level it stops at, and the
	// scramble makes the part-level remainder an unbiased subset of that level.
	std::vector<unsigned> order(numSamples);
	std::iota(order.begin(), order.end(), 0u);
	const auto Priority = [&cells, warpCols](unsigned i) {
		return WarpCellLatticePriority(cells[i]%warpCols, cells[i]/warpCols);
	};
	std::sort(order.begin(), order.end(), [&](unsigned l, unsigned r) {
		const int priorityL = Priority(l), priorityR = Priority(r);
		if (priorityL != priorityR)
			return priorityL > priorityR;
		const uint32_t scrambleL = WarpCellScramble(cells[l]), scrambleR = WarpCellScramble(cells[r]);
		if (scrambleL != scrambleR)
			return scrambleL < scrambleR;
		return cells[l] < cells[r]; // a total order even where the scramble collides
	});
	order.resize(maxSamples);
	// and back to the order the sample came in -- the warp-grid raster order every draw hands back,
	// which is what makes the keypoint indices an append hands out reproducible (AppendDenseMatches).
	// Sorted ascending, so no survivor is ever written over a source still to be read.
	std::sort(order.begin(), order.end());
	for (size_t i = 0; i < maxSamples; ++i) {
		const unsigned src = order[i];
		ASSERT(src >= i);
		sampledA[i] = sampledA[src];
		sampledB[i] = sampledB[src];
		confidences[i] = confidences[src];
		cells[i] = cells[src];
	}
	sampledA.resize(maxSamples);
	sampledB.resize(maxSamples);
	confidences.resize(maxSamples);
	cells.resize(maxSamples);
}
/*----------------------------------------------------------------*/


size_t SFM::SampleWarpComplementary(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	int bucketGridSide,
	unsigned maxSamples,
	const std::vector<Point2f>& occupiedA,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences)
{
	ASSERT(!warp.empty() && warp.size() == confidence.size());
	// cleared before any early return: the draw is a pure function of its inputs, so a caller
	// reusing its buffers must get exactly what a caller passing empty ones gets
	sampledA.clear();
	sampledB.clear();
	confidences.clear();
	if (maxSamples == 0)
		return 0;
	const cv::Size sizeA(imgA.GetSize()), sizeB(imgB.GetSize());
	std::vector<WarpCandidate> candidates;
	CollectWarpCandidates(sizeA, sizeB, warp, confidence, minConfidence, candidates);
	if (candidates.empty())
		return 0;
	// the caller's pitch, the same for every pair (DenseFillGridSide): the draw is a density over
	// the overlap, so the number of buckets a pair's overlap holds is what varies, not the pitch
	ASSERT(bucketGridSide > 0);
	const int numBuckets = MINF(bucketGridSide, confidence.cols);
	// every bucket an already-held correspondence sits in is out of the draw. Occupancy is measured
	// in image A only, deliberately: the warp grid lives in A's frame, so that is the one frame
	// where a sparse keypoint position and a warp cell are directly comparable. On a genuine pair --
	// the only kind that reaches here, the verdict having admitted it -- the warp is close enough to
	// a diffeomorphism that the B-side density mirrors A's through it, so a second grid in B (which
	// would also need a B->A back-map of this very draw) would mark the same buckets.
	std::vector<bool> occupied((size_t)numBuckets*numBuckets, false);
	for (const Point2f& pt : occupiedA) {
		// inverse of the cell -> imgA pixel map the draw itself uses: CoordFromTo is linear and
		// carries no half-pixel term, so its inverse carries none either. Rounded to the nearest
		// cell, and clamped because a keypoint may sit on the very border of A.
		const Point2f cell(CoordFromTo(pt, sizeA, confidence.size()));
		const int x = MINF(MAXF(ROUND2INT(cell.x), 0), confidence.cols-1);
		const int y = MINF(MAXF(ROUND2INT(cell.y), 0), confidence.rows-1);
		occupied[WarpCellBucket(y*confidence.cols + x, confidence, numBuckets)] = true;
	}
	// one winner per UNOCCUPIED bucket. An occupied bucket contributes nothing rather than a
	// reduced quota: "up to" a budget is a ceiling, not a target that has to be filled, and the
	// point of the draw is the part of the frame the sparse matches left empty.
	std::vector<int> bucketBest((size_t)numBuckets*numBuckets, -1);
	FOREACH(i, candidates) {
		const WarpCandidate& candidate = candidates[i];
		const size_t bucket = WarpCellBucket(candidate.cell, confidence, numBuckets);
		if (occupied[bucket])
			continue;
		int& best = bucketBest[bucket];
		if (best < 0 || WarpCandidateBeats(candidate, candidates[best]))
			best = (int)i; // the one winner rule: lattice priority, ties to raster order
	}
	std::vector<int> chosen;
	chosen.reserve(MINF(bucketBest.size(), candidates.size()));
	for (const int idxCandidate : bucketBest)
		if (idxCandidate >= 0)
			chosen.push_back(idxCandidate);
	std::sort(chosen.begin(), chosen.end()); // raster order, independent of the bucket traversal above
	sampledA.reserve(chosen.size());
	sampledB.reserve(chosen.size());
	confidences.reserve(chosen.size());
	// the winners' cells, kept alongside the sample only so the thinning below can rank it on the
	// same pair-independent key the winners themselves were picked on
	std::vector<int> cells;
	cells.reserve(chosen.size());
	for (const int idxCandidate : chosen) {
		const WarpCandidate& candidate = candidates[idxCandidate];
		sampledA.push_back(candidate.ptA);
		sampledB.push_back(candidate.ptB);
		// the cell's own confidence, not a bilinear read-back of the map at the point it produced:
		// the two differ by rounding, and this one is what the winner was actually chosen on
		confidences.push_back(candidate.confidence);
		cells.push_back(candidate.cell);
	}
	// the unoccupied-bucket count is bounded by min(E, n^2) and not by maxSamples (see
	// SampleWarpByCoverage's header), so a draw can still come out over its budget
	ThinSampleByLatticePriority(sampledA, sampledB, confidences, cells, confidence.cols, maxSamples);
	return sampledA.size();
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
	// the described prefix of each image ends where it stands now; a second pair on the same image
	// finds the boundary already closed and appends past the dense keypoints the first one left,
	// which is what lets FilterRedundantKeypoints reuse a coinciding one
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
	// The dense fill becomes the middle segment of `matches` (see the partition comment in
	// ImagePair.h): after the sparse inliers, which stay the pair's descriptor evidence and are what
	// GetNumFilteredInliers() counts, and before the RANSAC inliers the strict filter rejected,
	// which stay outside the track-forming prefix where they belong. Inside the track-forming prefix
	// rather than at the end because BuildTracks reads only that prefix -- appended past it the
	// whole fill would be inert -- and outside the sparse count because a coverage-maximising draw
	// must not re-rank the view graph.
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
	// will -- a pair filled here is not re-filtered afterwards (deliberately: the dense segment must
	// not go through the strict geometric filter), and a pair left claiming a baseline measured on
	// its sparse matches alone -- or, on a dense-only pair, none at all -- would keep the one weight
	// term that can demote a degenerate baseline unavailable on exactly this population
	pair.weightedInliers = -1.f;
	if (pair.relativePose.has_value())
		pair.meanRayAngle = pair.ComputeMeanRayAngle(imgA, imgB);
	return (unsigned)dense.size();
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
