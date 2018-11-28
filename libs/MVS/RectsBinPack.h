/*
* RectsBinPack.h
*
* Copyright (c) 2014-2015 SEACAVE
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
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

/**
Initial version created by:

@author Jukka Jylänki

@brief Implements different bin packer algorithms that use the MAXRECTS, SKYLINE and GUILLOTINE data structures.

This work is released to Public Domain, do whatever you want with it.
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
		RectContactPointRule, ///< -CP: Chooses the placement where the rectangle touches other rects as much as possible.
		RectLast
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
	static int ComputeTextureSize(const RectArr& rects, int mult=0);

	/// Returns true if a is contained/on the border in b.
	static inline bool IsContainedIn(const Rect& a, const Rect& b) {
		return a.x >= b.x && a.y >= b.y
			&& a.x+a.width <= b.x+b.width
			&& a.y+a.height <= b.y+b.height;
	}

protected:
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
/*----------------------------------------------------------------*/


#ifndef _RELEASE
class DisjointRectCollection
{
public:
	typedef cv::Rect Rect;
	std::vector<Rect> rects;

	bool Add(const Rect &r)
	{
		// Degenerate rectangles are ignored.
		if (r.width == 0 || r.height == 0)
			return true;

		if (!Disjoint(r))
			return false;
		rects.push_back(r);
		return true;
	}

	void Clear()
	{
		rects.clear();
	}

	bool Disjoint(const Rect &r) const
	{
		// Degenerate rectangles are ignored.
		if (r.width == 0 || r.height == 0)
			return true;

		for (size_t i = 0; i < rects.size(); ++i)
			if (!Disjoint(rects[i], r))
				return false;
		return true;
	}

	static bool Disjoint(const Rect &a, const Rect &b)
	{
		if (a.x + a.width <= b.x ||
			b.x + b.width <= a.x ||
			a.y + a.height <= b.y ||
			b.y + b.height <= a.y)
			return true;
		return false;
	}
};
#endif


// implements different variants of bin packer algorithms that use the GUILLOTINE data structure
// to keep track of the free space of the bin where rectangles may be placed
class GuillotineBinPack
{
public:
	typedef cv::Rect Rect;
	typedef CLISTDEF0(Rect) RectArr;

	/// The initial bin size will be (0,0). Call Init to set the bin size.
	GuillotineBinPack();

	/// Initializes a new bin of the given size.
	GuillotineBinPack(int width, int height);

	/// (Re)initializes the packer to an empty bin of width x height units. Call whenever
	/// you need to restart with a new bin.
	void Init(int width, int height);

	/// Specifies the different choice heuristics that can be used when deciding which of the free subrectangles
	/// to place the to-be-packed rectangle into.
	enum FreeRectChoiceHeuristic
	{
		RectBestAreaFit, ///< -BAF
		RectBestShortSideFit, ///< -BSSF
		RectBestLongSideFit, ///< -BLSF
		RectWorstAreaFit, ///< -WAF
		RectWorstShortSideFit, ///< -WSSF
		RectWorstLongSideFit, ///< -WLSF
		RectLast
	};

	/// Specifies the different choice heuristics that can be used when the packer needs to decide whether to
	/// subdivide the remaining free space in horizontal or vertical direction.
	enum GuillotineSplitHeuristic
	{
		SplitShorterLeftoverAxis, ///< -SLAS
		SplitLongerLeftoverAxis, ///< -LLAS
		SplitMinimizeArea, ///< -MINAS, Try to make a single big rectangle at the expense of making the other small.
		SplitMaximizeArea, ///< -MAXAS, Try to make both remaining rectangles as even-sized as possible.
		SplitShorterAxis, ///< -SAS
		SplitLongerAxis, ///< -LAS
		SplitLast
	};

	/// Inserts a single rectangle into the bin. The packer might rotate the rectangle, in which case the returned
	/// struct will have the width and height values swapped.
	/// @param merge If true, performs free Rectangle Merge procedure after packing the new rectangle. This procedure
	///		tries to defragment the list of disjoint free rectangles to improve packing performance, but also takes up 
	///		some extra time.
	/// @param rectChoice The free rectangle choice heuristic rule to use.
	/// @param splitMethod The free rectangle split heuristic rule to use.
	Rect Insert(int width, int height, bool merge, FreeRectChoiceHeuristic rectChoice, GuillotineSplitHeuristic splitMethod);

	/// Inserts a list of rectangles into the bin.
	/// @param rects The list of rectangles to add. This list will be destroyed in the packing process.
	/// @param merge If true, performs Rectangle Merge operations during the packing process.
	/// @param rectChoice The free rectangle choice heuristic rule to use.
	/// @param splitMethod The free rectangle split heuristic rule to use.
	bool Insert(RectArr& rects, bool merge,
				FreeRectChoiceHeuristic rectChoice, GuillotineSplitHeuristic splitMethod);

	/// Computes the ratio of used/total surface area. 0.00 means no space is yet used, 1.00 means the whole bin is used.
	float Occupancy() const;

	/// Returns the internal list of disjoint rectangles that track the free area of the bin. You may alter this vector
	/// any way desired, as long as the end result still is a list of disjoint rectangles.
	std::vector<Rect> &GetFreeRectangles() { return freeRectangles; }

	/// Returns the list of packed rectangles. You may alter this vector at will, for example, you can move a Rect from
	/// this list to the Free Rectangles list to free up space on-the-fly, but notice that this causes fragmentation.
	std::vector<Rect> &GetUsedRectangles() { return usedRectangles; }

	/// Performs a Rectangle Merge operation. This procedure looks for adjacent free rectangles and merges them if they
	/// can be represented with a single rectangle. Takes up Theta(|freeRectangles|^2) time.
	void MergeFreeList();

protected:
	int binWidth;
	int binHeight;

	/// Stores a list of all the rectangles that we have packed so far. This is used only to compute the Occupancy ratio,
	/// so if you want to have the packer consume less memory, this can be removed.
	std::vector<Rect> usedRectangles;

	/// Stores a list of rectangles that represents the free area of the bin. This rectangles in this list are disjoint.
	std::vector<Rect> freeRectangles;

	#ifndef _RELEASE
	/// Used to track that the packer produces proper packings.
	DisjointRectCollection disjointRects;
	#endif

	/// Goes through the list of free rectangles and finds the best one to place a rectangle of given size into.
	/// Running time is Theta(|freeRectangles|).
	/// @param nodeIndex [out] The index of the free rectangle in the freeRectangles array into which the new
	///		rect was placed.
	/// @return A Rect structure that represents the placement of the new rect into the best free rectangle.
	Rect FindPositionForNewNode(int width, int height, FreeRectChoiceHeuristic rectChoice, size_t& nodeIndex);

	static int ScoreByHeuristic(int width, int height, const Rect &freeRect, FreeRectChoiceHeuristic rectChoice);
	// The following functions compute (penalty) score values if a rect of the given size was placed into the 
	// given free rectangle. In these score values, smaller is better.

	static int ScoreBestAreaFit(int width, int height, const Rect &freeRect);
	static int ScoreBestShortSideFit(int width, int height, const Rect &freeRect);
	static int ScoreBestLongSideFit(int width, int height, const Rect &freeRect);

	static int ScoreWorstAreaFit(int width, int height, const Rect &freeRect);
	static int ScoreWorstShortSideFit(int width, int height, const Rect &freeRect);
	static int ScoreWorstLongSideFit(int width, int height, const Rect &freeRect);

	/// Splits the given L-shaped free rectangle into two new free rectangles after placedRect has been placed into it.
	/// Determines the split axis by using the given heuristic.
	void SplitFreeRectByHeuristic(const Rect &freeRect, const Rect &placedRect, GuillotineSplitHeuristic method);

	/// Splits the given L-shaped free rectangle into two new free rectangles along the given fixed split axis.
	void SplitFreeRectAlongAxis(const Rect &freeRect, const Rect &placedRect, bool splitHorizontal);
};
/*----------------------------------------------------------------*/


// implements bin packing algorithms that use the SKYLINE data structure to store the bin contents;
// uses GuillotineBinPack as the waste map
class SkylineBinPack
{
public:
	typedef cv::Rect Rect;
	typedef CLISTDEF0(Rect) RectArr;

	/// Instantiates a bin of size (0,0). Call Init to create a new bin.
	SkylineBinPack();

	/// Instantiates a bin of the given size.
	SkylineBinPack(int binWidth, int binHeight, bool useWasteMap);

	/// (Re)initializes the packer to an empty bin of width x height units. Call whenever
	/// you need to restart with a new bin.
	void Init(int binWidth, int binHeight, bool useWasteMap);

	/// Defines the different heuristic rules that can be used to decide how to make the rectangle placements.
	enum LevelChoiceHeuristic
	{
		LevelBottomLeft,
		LevelMinWasteFit,
		LevelLast
	};

	/// Inserts the given list of rectangles in an offline/batch mode, possibly rotated.
	/// @param rects [in/out] The list of rectangles to insert. This vector will be update in the process.
	/// @param method The rectangle placement rule to use when packing.
	bool Insert(RectArr& rects, LevelChoiceHeuristic method);

	/// Inserts a single rectangle into the bin, possibly rotated.
	Rect Insert(int width, int height, LevelChoiceHeuristic method);

	Rect ScoreRect(int width, int height, LevelChoiceHeuristic method, int &score1, int &score2, int &index) const;

	/// Computes the ratio of used surface area to the total bin area.
	float Occupancy() const;

protected:
	int binWidth;
	int binHeight;

	#ifndef _RELEASE
	DisjointRectCollection disjointRects;
	#endif

	/// Represents a single level (a horizontal line) of the skyline/horizon/envelope.
	struct SkylineNode
	{
		/// The starting x-coordinate (leftmost).
		int x;

		/// The y-coordinate of the skyline level line.
		int y;

		/// The line width. The ending coordinate (inclusive) will be x+width-1.
		int width;
	};

	std::vector<SkylineNode> skyLine;

	unsigned long usedSurfaceArea;

	/// If true, we use the GuillotineBinPack structure to recover wasted areas into a waste map.
	bool useWasteMap;
	GuillotineBinPack wasteMap;

	Rect InsertBottomLeft(int width, int height);
	Rect InsertMinWaste(int width, int height);

	Rect FindPositionForNewNodeMinWaste(int width, int height, int &bestHeight, int &bestWastedArea, int &bestIndex) const;
	Rect FindPositionForNewNodeBottomLeft(int width, int height, int &bestHeight, int &bestWidth, int &bestIndex) const;

	bool RectangleFits(int skylineNodeIndex, int width, int height, int &y) const;
	bool RectangleFits(int skylineNodeIndex, int width, int height, int &y, int &wastedArea) const;
	int ComputeWastedArea(int skylineNodeIndex, int width, int height, int y) const;

	void AddWasteMapArea(int skylineNodeIndex, int width, int height, int y);

	void AddSkylineLevel(int skylineNodeIndex, const Rect &rect);

	/// Merges all skyline nodes that are at the same level.
	void MergeSkylines();
};
/*----------------------------------------------------------------*/


typedef MaxRectsBinPack RectsBinPack;
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_RECTSBINPACK_H_
