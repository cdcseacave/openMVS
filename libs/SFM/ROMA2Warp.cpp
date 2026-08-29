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
	// Track keypoints from A to B using warp and overlap maps
	const size_t numKp = imgA.keypoints.size();
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
	pair.overlapRatio = pair.overlapArea = 1.f; // a dense-matched pair covers the frame (the old NPZ import set the same)
	pairIndexMap.emplace(key, (IIndex)scene.pairs.size());
	scene.pairs.emplace_back(std::move(pair));
	bCreated = true;
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
