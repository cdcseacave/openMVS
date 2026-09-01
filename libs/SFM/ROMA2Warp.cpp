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
	struct Candidate {
		float confidence;
		int cell;        // y*cols + x of the warp grid, so the sample keeps raster order
		Point2f ptA, ptB;
	};
	// pass 1: every eligible cell. Their count is what the bucket grid below is sized from, so it
	// has to be known before a single bucket exists
	std::vector<Candidate> candidates;
	candidates.reserve((size_t)overlap.rows*overlap.cols/4);
	for (int y = 0; y < overlap.rows; ++y) {
		for (int x = 0; x < overlap.cols; ++x) {
			const float confidence = overlap(y, x);
			if (confidence < minConfidence)
				continue;
			const Point2f ptB(DenormCoord(warp(y, x), sizeB));
			if (!Image8U::isInside(ptB, sizeB))
				continue; // the warp sends this cell outside the second image
			candidates.push_back(Candidate{confidence, y*overlap.cols + x,
				CoordFromTo(Point2f((float)x, (float)y), overlap.size(), sizeA), ptB});
		}
	}
	if (candidates.empty())
		return 0;

	std::vector<int> chosen;
	if (candidates.size() <= maxSamples) {
		// the confident overlap already fits in the budget: there is nothing to select, and any
		// stratification would only be able to throw points away
		chosen.resize(candidates.size());
		std::iota(chosen.begin(), chosen.end(), 0);
	} else {
		// pass 2: one winner per bucket of an n x n grid over the whole warp, with n scaled up by the
		// inverse overlap fraction (see the header) so that the buckets which do hold an eligible cell
		// number about maxSamples. Integer arithmetic throughout -- n is the smallest value with
		// n^2 * E >= maxSamples * T, which is ceil(sqrt(maxSamples*T/E)) without a libm square root
		// that could land a hair off a perfect square and shift the whole grid on a different platform.
		// Capped at the warp side: beyond one bucket per cell there is nothing left to gain.
		const uint64_t target = (uint64_t)maxSamples*(uint64_t)overlap.rows*(uint64_t)overlap.cols;
		int numBuckets = 1;
		while (numBuckets < overlap.cols &&
			(uint64_t)numBuckets*(uint64_t)numBuckets*(uint64_t)candidates.size() < target)
			++numBuckets;
		std::vector<int> bucketBest((size_t)numBuckets*numBuckets, -1);
		FOREACH(i, candidates) {
			const Candidate& candidate = candidates[i];
			const int x = candidate.cell % overlap.cols, y = candidate.cell / overlap.cols;
			int& best = bucketBest[(size_t)(y*numBuckets/overlap.rows)*numBuckets + x*numBuckets/overlap.cols];
			if (best < 0 || candidate.confidence > candidates[best].confidence)
				best = (int)i; // strictly more confident wins, so a tie keeps the first cell in raster order
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
		const Candidate& candidate = candidates[idxCandidate];
		sampledA.push_back(candidate.ptA);
		sampledB.push_back(candidate.ptB);
	}
	ComputeSampleCoverage(sampledA, sampledB, sizeA, sizeB, std::vector<uint32_t>(), coverageA, coverageB);
	return sampledA.size();
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


bool SFM::ApplyROMA2Pair(Scene& scene, std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap, ImagePair&& pair, unsigned maxReplaceInliers, bool& bCreated)
{
	ASSERT(pair.ID1 < pair.ID2 && !pair.matches.empty());
	const PairIdx::PairIndex key = PairIdx(pair.ID1, pair.ID2).idx;
	const auto it = pairIndexMap.find(key);
	if (it != pairIndexMap.end()) {
		ImagePair& scenePair = scene.pairs[it->second];
		const unsigned existingInliers = scenePair.GetNumFilteredInliers();
		// polycpp ShouldReplaceROMA2Pair (import_roma2.hpp:39-45): strictly more inliers, and the existing pair below the ceiling
		if (pair.GetNumFilteredInliers() <= existingInliers || (maxReplaceInliers > 0 && existingInliers >= maxReplaceInliers)) {
			DEBUG_ULTIMATE("ROMA2 pair (% 4u, % 4u) kept: %u existing vs %u guided inliers", pair.ID1, pair.ID2, existingInliers, pair.GetNumFilteredInliers());
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
