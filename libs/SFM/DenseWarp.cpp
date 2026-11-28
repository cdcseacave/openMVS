/*
 * DenseWarp.cpp
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
#include "DenseWarp.h"
#include "Scene.h"

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

Point2f SFM::PixelToCell(const Point2f& pixel, const cv::Size& sizeImage, const cv::Size& sizeGrid)
{
	return Point2f(
		(pixel.x + 0.5f) * (float)sizeGrid.width  / (float)sizeImage.width  - 0.5f,
		(pixel.y + 0.5f) * (float)sizeGrid.height / (float)sizeImage.height - 0.5f
	);
}

Point2f SFM::CellToPixel(const Point2f& cell, const cv::Size& sizeGrid, const cv::Size& sizeImage)
{
	return PixelToCell(cell, sizeGrid, sizeImage);
}

Point2f SFM::DenormCoord(const Point2f& normCoord, const cv::Size& size)
{
	return Point2f(
		0.5f * (normCoord.x + 1.f) * (float)size.width  - 0.5f,
		0.5f * (normCoord.y + 1.f) * (float)size.height - 0.5f
	);
}

Point2f SFM::NormCoord(const Point2f& coord, const cv::Size& size)
{
	return Point2f(
		2.f * (coord.x + 0.5f) / (float)size.width  - 1.f,
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
	std::vector<Point2f>& trackedB,
	std::vector<uchar>& trackStatus)
{
	ASSERT(!warp.empty() && warp.size() == confidence.size());
	const size_t numKp = imgA.NumDescribedKeypoints();
	trackedB.resize(numKp);
	trackStatus.resize(numKp);
	size_t numTracked = 0;
	for (size_t i = 0; i < numKp; ++i) {
		const Point2f cell(PixelToCell(imgA.keypoints[i].pt, imgA.GetSize(), warp.size()));
		const Point2f wkpA(CLAMP(cell.x, 0.f, (float)(warp.cols - 1)), CLAMP(cell.y, 0.f, (float)(warp.rows - 1)));
		if (confidence.sampleSafe(wkpA) < minConfidence) {
			trackStatus[i] = 0;
			continue;
		}
		const Point2f kpB = DenormCoord(warp.sampleSafe(wkpA), imgB.GetSize());
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


size_t SFM::CollectEligibleWarpCells(
	const WarpMaps& maps,
	const Image& imgSrc,
	const Image& imgDst,
	float minConfidence,
	float sphericalBottomBand,
	std::vector<Point2f>& ptsSrc,
	std::vector<Point2f>& ptsDst,
	std::vector<float>& confidences,
	std::vector<int>& cells)
{
	ASSERT(maps.IsValid());
	ptsSrc.clear();
	ptsDst.clear();
	confidences.clear();
	cells.clear();
	const size_t reserveHint = (size_t)maps.confidence.rows * maps.confidence.cols / 4;
	ptsSrc.reserve(reserveHint);
	ptsDst.reserve(reserveHint);
	confidences.reserve(reserveHint);
	cells.reserve(reserveHint);
	for (int y = 0; y < maps.confidence.rows; ++y) {
		for (int x = 0; x < maps.confidence.cols; ++x) {
			const float conf = maps.confidence(y, x);
			const Point2f ptSrc(CellToPixel(Point2f((float)x, (float)y), maps.confidence.size(), imgSrc.GetSize()));
			Point2f ptDst;
			if (!IsWarpCellEligible(conf, maps.warp(y, x), minConfidence, imgDst, ptSrc, imgSrc, sphericalBottomBand, ptDst))
				continue;
			ptsSrc.push_back(ptSrc);
			ptsDst.push_back(ptDst);
			confidences.push_back(conf);
			cells.push_back(y * maps.confidence.cols + x);
		}
	}
	return ptsSrc.size();
}
/*----------------------------------------------------------------*/


namespace {

// Pair-independent dyadic lattice priority: largest k such that 2^k divides both x and y (0 -> 31).
// Guarantees two pairs covering the same region pick the same cell so keypoints coincide across pairs.
inline int WarpCellLatticePriority(int x, int y)
{
	int v = x | y;
	if (v == 0)
		return 31;
	int k = 0;
	while ((v & 1) == 0) { v >>= 1; ++k; }
	return k;
}

// Deterministic multiply-xorshift scramble of a cell index for pair-independent tie-breaking.
inline uint32_t WarpCellScramble(int cell)
{
	uint32_t h = (uint32_t)cell;
	h ^= h >> 16; h *= 0x7feb352dU;
	h ^= h >> 15; h *= 0x846ca68bU;
	h ^= h >> 16;
	return h;
}

// Side n of the n x n bucket grid: smallest n with n^2 * E >= maxSamples * T (integer arithmetic).
int WarpBucketGridSide(const cv::Size& gridSize, size_t numCandidates, unsigned maxSamples)
{
	ASSERT(numCandidates > 0 && gridSize.width > 0 && gridSize.height > 0);
	const int maxSide = MINF(gridSize.width, gridSize.height);
	const uint64_t target = (uint64_t)maxSamples * (uint64_t)gridSize.height * (uint64_t)gridSize.width;
	int numBuckets = 1;
	while (numBuckets < maxSide &&
		(uint64_t)numBuckets * (uint64_t)numBuckets * (uint64_t)numCandidates < target)
		++numBuckets;
	return numBuckets;
}

inline size_t WarpCellBucket(int x, int y, const cv::Size& gridSize, int numBuckets)
{
	return (size_t)(y * numBuckets / gridSize.height) * numBuckets + x * numBuckets / gridSize.width;
}

// Select one winner per (unoccupied) bucket by dyadic lattice priority (ties to raster order),
// returning winning candidate indices sorted in raster order.
void SelectBucketWinners(
	const std::vector<int>& cells,
	const cv::Size& gridSize,
	int numBuckets,
	const std::vector<uint8_t>& occupied,
	std::vector<int>& chosen)
{
	const size_t totalBuckets = (size_t)numBuckets * numBuckets;
	std::vector<int> bucketBest(totalBuckets, -1);
	std::vector<int> bucketPriority(totalBuckets, -1);
	FOREACH(i, cells) {
		const int x = cells[i] % gridSize.width;
		const int y = cells[i] / gridSize.width;
		const size_t bucket = WarpCellBucket(x, y, gridSize, numBuckets);
		if (!occupied.empty() && occupied[bucket])
			continue;
		const int prio = WarpCellLatticePriority(x, y);
		if (bucketBest[bucket] < 0 || prio > bucketPriority[bucket]) {
			bucketBest[bucket] = (int)i;
			bucketPriority[bucket] = prio;
		}
	}
	chosen.clear();
	chosen.reserve(MINF(totalBuckets, cells.size()));
	for (const int idx : bucketBest)
		if (idx >= 0)
			chosen.push_back(idx);
	std::sort(chosen.begin(), chosen.end());
}

} // namespace


size_t SFM::SampleWarpByCoverage(
	const std::vector<Point2f>& ptsA,
	const std::vector<Point2f>& ptsB,
	const std::vector<int>& cells,
	const cv::Size& gridSize,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB)
{
	ASSERT(ptsA.size() == ptsB.size() && ptsA.size() == cells.size());
	sampledA.clear();
	sampledB.clear();
	if (maxSamples == 0 || ptsA.empty())
		return 0;
	if (ptsA.size() <= maxSamples) {
		sampledA = ptsA;
		sampledB = ptsB;
		return sampledA.size();
	}
	const int numBuckets = WarpBucketGridSide(gridSize, cells.size(), maxSamples);
	std::vector<int> chosen;
	SelectBucketWinners(cells, gridSize, numBuckets, {}, chosen);
	sampledA.reserve(chosen.size());
	sampledB.reserve(chosen.size());
	for (const int idx : chosen) {
		sampledA.push_back(ptsA[idx]);
		sampledB.push_back(ptsB[idx]);
	}
	return sampledA.size();
}

size_t SFM::SampleWarpByCoverage(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	float sphericalBottomBand,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB)
{
	ASSERT(!warp.empty() && warp.size() == confidence.size());
	if (maxSamples == 0) {
		sampledA.clear();
		sampledB.clear();
		return 0;
	}
	std::vector<Point2f> ptsA, ptsB;
	std::vector<float> confidences;
	std::vector<int> cells;
	CollectEligibleWarpCells(WarpMaps{warp, confidence}, imgA, imgB, minConfidence, sphericalBottomBand, ptsA, ptsB, confidences, cells);
	return SampleWarpByCoverage(ptsA, ptsB, cells, confidence.size(), maxSamples, sampledA, sampledB);
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
	if (maxSamples == 0) {
		sampledA.clear();
		sampledB.clear();
		confidences.clear();
		cells.clear();
		return;
	}
	// Precompute packed ranking key: smaller key wins (higher lattice priority, then smaller scramble)
	struct RankedSample {
		uint64_t key;
		int cell;
		unsigned idx;
	};
	std::vector<RankedSample> ranked(numSamples);
	for (size_t i = 0; i < numSamples; ++i) {
		const int c = cells[i];
		const int prio = WarpCellLatticePriority(c % warpCols, c / warpCols);
		ranked[i] = { ((uint64_t)(31 - prio) << 32) | WarpCellScramble(c), c, (unsigned)i };
	}
	std::nth_element(ranked.begin(), ranked.begin() + maxSamples, ranked.end(),
		[](const RankedSample& l, const RankedSample& r) {
			return l.key != r.key ? l.key < r.key : l.cell < r.cell;
		});
	ranked.resize(maxSamples);
	// Restore original raster order (ascending idx guarantees safe in-place compaction)
	std::sort(ranked.begin(), ranked.end(),
		[](const RankedSample& l, const RankedSample& r) { return l.idx < r.idx; });
	for (size_t i = 0; i < maxSamples; ++i) {
		const unsigned src = ranked[i].idx;
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
	const std::vector<Point2f>& ptsA,
	const std::vector<Point2f>& ptsB,
	const std::vector<float>& inConfidences,
	const std::vector<int>& inCells,
	const cv::Size& sizeA,
	const cv::Size& gridSize,
	int bucketGridSide,
	unsigned maxSamples,
	const std::vector<Point2f>& occupiedA,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences)
{
	ASSERT(ptsA.size() == ptsB.size() && ptsA.size() == inConfidences.size() && ptsA.size() == inCells.size());
	ASSERT(gridSize.width > 0 && gridSize.height > 0 && bucketGridSide > 0);
	sampledA.clear();
	sampledB.clear();
	confidences.clear();
	if (maxSamples == 0 || ptsA.empty())
		return 0;
	const int numBuckets = MINF(bucketGridSide, MINF(gridSize.width, gridSize.height));
	std::vector<uint8_t> occupied((size_t)numBuckets * numBuckets, 0);
	for (const Point2f& pt : occupiedA) {
		const Point2f cell(PixelToCell(pt, sizeA, gridSize));
		const int x = CLAMP(ROUND2INT(cell.x), 0, gridSize.width - 1);
		const int y = CLAMP(ROUND2INT(cell.y), 0, gridSize.height - 1);
		occupied[WarpCellBucket(x, y, gridSize, numBuckets)] = 1;
	}
	std::vector<int> chosen;
	SelectBucketWinners(inCells, gridSize, numBuckets, occupied, chosen);
	sampledA.reserve(chosen.size());
	sampledB.reserve(chosen.size());
	confidences.reserve(chosen.size());
	std::vector<int> outCells;
	outCells.reserve(chosen.size());
	for (const int idx : chosen) {
		sampledA.push_back(ptsA[idx]);
		sampledB.push_back(ptsB[idx]);
		confidences.push_back(inConfidences[idx]);
		outCells.push_back(inCells[idx]);
	}
	ThinSampleByLatticePriority(sampledA, sampledB, confidences, outCells, gridSize.width, maxSamples);
	return sampledA.size();
}

size_t SFM::SampleWarpComplementary(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	float sphericalBottomBand,
	int bucketGridSide,
	unsigned maxSamples,
	const std::vector<Point2f>& occupiedA,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences)
{
	ASSERT(!warp.empty() && warp.size() == confidence.size());
	if (maxSamples == 0) {
		sampledA.clear();
		sampledB.clear();
		confidences.clear();
		return 0;
	}
	std::vector<Point2f> ptsA, ptsB;
	std::vector<float> inConfidences;
	std::vector<int> inCells;
	CollectEligibleWarpCells(WarpMaps{warp, confidence}, imgA, imgB, minConfidence, sphericalBottomBand, ptsA, ptsB, inConfidences, inCells);
	return SampleWarpComplementary(ptsA, ptsB, inConfidences, inCells, imgA.GetSize(), confidence.size(),
		bucketGridSide, maxSamples, occupiedA, sampledA, sampledB, confidences);
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
	const auto WarpCellSize = [&warpSize](const Image& img) {
		return MAXF((float)img.GetWidth() / (float)warpSize.width, (float)img.GetHeight() / (float)warpSize.height);
	};
	const float cellSizeA = WarpCellSize(imgA);
	const float cellSizeB = WarpCellSize(imgB);
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
	if (pair.numFilteredInliers < 0)
		pair.numFilteredInliers = (int)pair.matches.size();
	const size_t at = (size_t)pair.numFilteredInliers + (size_t)pair.numDenseInliers;
	ASSERT(at <= pair.matches.size());
	pair.matches.insert(pair.matches.begin() + at, dense.begin(), dense.end());
	pair.numDenseInliers += (int)dense.size();
	pair.weightedInliers = -1.f;
	if (pair.relativePose.has_value())
		pair.meanRayAngle = pair.ComputeMeanRayAngle(imgA, imgB);
	return (unsigned)dense.size();
}
/*----------------------------------------------------------------*/
