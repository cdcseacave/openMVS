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

// Store the given guided pair in the scene, either creating it or replacing the existing one:
// pairIndexMap maps the pair key to its index in scene.pairs and is updated accordingly,
// maxReplaceInliers is the inlier ceiling above which an existing pair is never replaced
// (0 = no ceiling), and bCreated tells whether a new pair was appended.
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
