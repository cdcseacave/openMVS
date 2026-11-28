/*
 * DenseWarp.h
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

#ifndef _SFM_DENSEWARP_H_
#define _SFM_DENSEWARP_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"
#include "ImagePair.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;

// Dense correspondence maps of an image pair in one direction (source grid -> target image).
struct SFM_API WarpMaps {
	Image32F2 warp;      // normalized (align_corners=false) target coordinates in [-1,1]
	Image32F confidence; // matching confidence in [0,1]

	inline bool IsValid() const {
		return !warp.empty() && warp.size() == confidence.size();
	}
};
/*----------------------------------------------------------------*/

// Both directions of one (A,B) pair from a single dense-matcher pass.
struct SFM_API PairWarps {
	WarpMaps ab, ba;

	inline bool IsValid() const {
		return ab.IsValid() && ba.IsValid() && ab.warp.size() == ba.warp.size();
	}
};
/*----------------------------------------------------------------*/

// Pixel <-> warp-grid cell maps (align_corners=false: cell (i,j) sits at (i+0.5)*W/C - 0.5).
SFM_API Point2f PixelToCell(const Point2f& pixel, const cv::Size& sizeImage, const cv::Size& sizeGrid);
SFM_API Point2f CellToPixel(const Point2f& cell, const cv::Size& sizeGrid, const cv::Size& sizeImage);

// Normalized [-1,1] warp coordinate <-> pixel coordinate maps (align_corners=false).
SFM_API Point2f DenormCoord(const Point2f& normCoord, const cv::Size& size);
SFM_API Point2f NormCoord(const Point2f& coord, const cv::Size& size);

// Eligibility predicate shared by every warp scan: confidence >= minConfidence, warped point inside
// imgDst, and neither endpoint inside the spherical bottom band (0 = disabled).
inline bool IsWarpCellEligible(float conf, const Point2f& normCoord, float minConfidence,
	const Image& imgDst, const Point2f& ptSrc, const Image& imgSrc, float sphericalBottomBand, Point2f& ptDst)
{
	if (conf < minConfidence)
		return false;
	ptDst = DenormCoord(normCoord, imgDst.GetSize());
	if (!Image8U::isInside(ptDst, imgDst.GetSize()))
		return false;
	return !(sphericalBottomBand > 0.f &&
		((imgSrc.pCamera->GetType() == CameraType::SPHERICAL && imgSrc.IsInSphericalBottomBand(ptSrc, sphericalBottomBand)) ||
		 (imgDst.pCamera->GetType() == CameraType::SPHERICAL && imgDst.IsInSphericalBottomBand(ptDst, sphericalBottomBand))));
}

// Half a warp cell in image pixels (epipolar tolerance for coarse-warp correspondences).
inline float WarpTolerance(const cv::Size& sizeA, const cv::Size& sizeB, int warpSize) {
	ASSERT(warpSize > 0);
	return 0.5f * (float)MAXF(MAXF(sizeA.width, sizeA.height), MAXF(sizeB.width, sizeB.height)) / (float)warpSize;
}

// Side of the fixed bucket grid used by the dense fill to achieve uniform density and cross-pair chaining.
inline int DenseFillGridSide(unsigned denseMatchesPerFrame, int warpSide) {
	ASSERT(warpSide > 0);
	return MINF(warpSide, MAXF(1, (int)std::ceil(std::sqrt((double)denseMatchesPerFrame))));
}
/*----------------------------------------------------------------*/

// Track described keypoints of imgA into imgB through warp and confidence maps.
// trackedB and trackStatus are resized to imgA.NumDescribedKeypoints(); trackedB is written where trackStatus == 1.
// Returns the number of tracked keypoints.
SFM_API size_t TrackKeypointsByWarp(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	std::vector<Point2f>& trackedB,
	std::vector<uchar>& trackStatus);

// Collect every eligible warp cell of one direction in raster order.
// Outputs are index-parallel: source cell centres, warped target pixels, confidences, and cell indices (y*cols + x).
SFM_API size_t CollectEligibleWarpCells(
	const WarpMaps& maps,
	const Image& imgSrc,
	const Image& imgDst,
	float minConfidence,
	float sphericalBottomBand,
	std::vector<Point2f>& ptsSrc,
	std::vector<Point2f>& ptsDst,
	std::vector<float>& confidences,
	std::vector<int>& cells);

// Draw a spatially uniform sample of confident correspondences from a warp (maxSamples is a target;
// stratified by inverse overlap fraction with pair-independent dyadic lattice priority).
// Outputs are index-parallel in warp-grid raster order. Returns the number of sampled correspondences.
SFM_API size_t SampleWarpByCoverage(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& confidence,
	float minConfidence,
	float sphericalBottomBand,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB);

// Overload operating directly on pre-collected eligible cells (avoids re-scanning the warp grid).
SFM_API size_t SampleWarpByCoverage(
	const std::vector<Point2f>& ptsA,
	const std::vector<Point2f>& ptsB,
	const std::vector<int>& cells,
	const cv::Size& gridSize,
	unsigned maxSamples,
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB);

// Thin an index-parallel warp sample in place to at most maxSamples using pair-independent
// dyadic lattice priority + deterministic cell scramble, preserving warp-grid raster order.
SFM_API void ThinSampleByLatticePriority(
	std::vector<Point2f>& sampledA,
	std::vector<Point2f>& sampledB,
	std::vector<float>& confidences,
	std::vector<int>& cells,
	int warpCols,
	unsigned maxSamples);

// Draw up to maxSamples confident correspondences complementing occupiedA on a fixed bucketGridSide^2 grid,
// thinning any excess by ThinSampleByLatticePriority. Pure function; outputs in warp-grid raster order.
SFM_API size_t SampleWarpComplementary(
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
	std::vector<float>& confidences);

// Overload operating directly on pre-collected eligible cells (avoids rebuilding a temporary warp map).
SFM_API size_t SampleWarpComplementary(
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
	std::vector<float>& confidences);

// Append dense correspondences to scene images (past described prefixes) and insert them into
// pair.matches[numFilteredInliers, +numDenseInliers). Must be called serially in deterministic pair order.
SFM_API unsigned AppendDenseMatches(
	Scene& scene,
	ImagePair& pair,
	const std::vector<Point2f>& pointsA,
	const std::vector<Point2f>& pointsB,
	const std::vector<float>& confidences,
	const cv::Size& warpSize);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_DENSEWARP_H_
