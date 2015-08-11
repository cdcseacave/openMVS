/*
* RectsBinPack.h
*
* Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
* Please read <http://foxel.ch/license> for more information.
*
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This file is part of the FOXEL project <http://foxel.ch>.
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
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*
*      You are required to attribute the work as explained in the "Usage and
*      Attribution" section of <http://foxel.ch/license>.
*/

#ifndef _MVS_RECTSBINPACK_H_
#define _MVS_RECTSBINPACK_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {


// implements the MAXRECTS data structure and different bin packing algorithms that use this structure
class MaxRectsBinPack
{
public:
	typedef cv::Rect Rect;
	typedef CLISTDEF0(Rect) RectArr;

	/// Instantiates a bin of size (0,0). Call Init to create a new bin.
	MaxRectsBinPack();

	/// Instantiates a bin of the given size.
	MaxRectsBinPack(int width, int height);

	/// (Re)initializes the packer to an empty bin of width x height units. Call whenever
	/// you need to restart with a new bin.
	void Init(int width, int height);

	/// Specifies the different heuristic rules that can be used when deciding where to place a new rectangle.
	enum FreeRectChoiceHeuristic {
		RectBestShortSideFit, ///< -BSSF: Positions the rectangle against the short side of a free rectangle into which it fits the best.
		RectBestLongSideFit, ///< -BLSF: Positions the rectangle against the long side of a free rectangle into which it fits the best.
		RectBestAreaFit, ///< -BAF: Positions the rectangle into the smallest free rect into which it fits.
		RectBottomLeftRule, ///< -BL: Does the Tetris placement.
		RectContactPointRule ///< -CP: Chooses the placement where the rectangle touches other rects as much as possible.
	};

	/// Inserts the given list of rectangles in an offline/batch mode, possibly rotated.
	/// @param rects [IN/OUT] The list of rectangles to insert; the rectangles will be modified with the new coordinates in the process.
	/// @param method The rectangle placement rule to use when packing.
	/// returns true if all rectangles were inserted
	bool Insert(RectArr& rects, FreeRectChoiceHeuristic method=RectBestShortSideFit);

	/// Inserts a single rectangle into the bin, possibly rotated.
	Rect Insert(int width, int height, FreeRectChoiceHeuristic method=RectBestShortSideFit);

	/// Computes the ratio of used surface area to the total bin area.
	float Occupancy() const;

	/// Computes an approximate texture atlas size.
	static int ComputeTextureSize(const RectArr& rects);

private:
	int binWidth;
	int binHeight;

	RectArr usedRectangles;
	RectArr freeRectangles;

	/// Computes the placement score for placing the given rectangle with the given method.
	/// @param score1 [out] The primary placement score will be outputted here.
	/// @param score2 [out] The secondary placement score will be outputted here. This is used to break ties.
	/// @return This struct identifies where the rectangle would be placed if it were placed.
	Rect ScoreRect(int width, int height, FreeRectChoiceHeuristic method, int &score1, int &score2) const;

	/// Places the given rectangle into the bin.
	void PlaceRect(const Rect &node);

	/// Computes the placement score for the -CP variant.
	int ContactPointScoreNode(int x, int y, int width, int height) const;

	Rect FindPositionForNewNodeBottomLeft(int width, int height, int &bestY, int &bestX) const;
	Rect FindPositionForNewNodeBestShortSideFit(int width, int height, int &bestShortSideFit, int &bestLongSideFit) const;
	Rect FindPositionForNewNodeBestLongSideFit(int width, int height, int &bestShortSideFit, int &bestLongSideFit) const;
	Rect FindPositionForNewNodeBestAreaFit(int width, int height, int &bestAreaFit, int &bestShortSideFit) const;
	Rect FindPositionForNewNodeContactPoint(int width, int height, int &contactScore) const;

	/// @return True if the free node was split.
	bool SplitFreeNode(Rect freeNode, const Rect &usedNode);

	/// Goes through the free rectangle list and removes any redundant entries.
	void PruneFreeList();
};
typedef MaxRectsBinPack RectsBinPack;
/*----------------------------------------------------------------*/


/// Returns true if a is contained/on the border in b.
inline bool IsContainedIn(const RectsBinPack::Rect& a, const RectsBinPack::Rect& b) {
	return a.x >= b.x && a.y >= b.y
		&& a.x+a.width <= b.x+b.width
		&& a.y+a.height <= b.y+b.height;
}
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_RECTSBINPACK_H_
