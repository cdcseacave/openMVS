/*
* RectsBinPack.cpp
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

#include "Common.h"
#include "RectsBinPack.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

/**
Initial version created by:

@author Jukka Jylänki

@brief Implements different bin packer algorithms that use the MAXRECTS data structure.

This work is released to Public Domain, do whatever you want with it.
*/

MaxRectsBinPack::MaxRectsBinPack()
	:
	binWidth(0),
	binHeight(0)
{
}

MaxRectsBinPack::MaxRectsBinPack(int width, int height)
{
	Init(width, height);
}

void MaxRectsBinPack::Init(int width, int height)
{
	binWidth = width;
	binHeight = height;

	usedRectangles.Empty();

	freeRectangles.Empty();
	freeRectangles.Insert(Rect(0,0, width,height));
}

MaxRectsBinPack::Rect MaxRectsBinPack::Insert(int width, int height, FreeRectChoiceHeuristic method)
{
	Rect newNode;
	// Unused in this function. We don't need to know the score after finding the position.
	int score1 = std::numeric_limits<int>::max();
	int score2 = std::numeric_limits<int>::max();
	switch (method) {
	case RectBestShortSideFit: newNode = FindPositionForNewNodeBestShortSideFit(width, height, score1, score2); break;
	case RectBottomLeftRule: newNode = FindPositionForNewNodeBottomLeft(width, height, score1, score2); break;
	case RectContactPointRule: newNode = FindPositionForNewNodeContactPoint(width, height, score1); break;
	case RectBestLongSideFit: newNode = FindPositionForNewNodeBestLongSideFit(width, height, score2, score1); break;
	case RectBestAreaFit: newNode = FindPositionForNewNodeBestAreaFit(width, height, score1, score2); break;
	}

	if (newNode.height == 0)
		return newNode;

	PlaceRect(newNode);
	return newNode;
}

bool MaxRectsBinPack::Insert(RectArr& rects, FreeRectChoiceHeuristic method)
{
	cList<IDX, IDX, 0> indices(rects.GetSize());
	std::iota(indices.Begin(), indices.End(), 0);
	RectArr newRects(rects.GetSize());
	while (!rects.IsEmpty()) {
		int bestScore1 = std::numeric_limits<int>::max();
		int bestScore2 = std::numeric_limits<int>::max();
		IDX bestRectIndex = NO_IDX;
		Rect bestNode;

		// find the best place to store this rectangle
		FOREACH(i, rects) {
			int score1, score2;
			Rect newNode(ScoreRect(rects[i].width, rects[i].height, method, score1, score2));
			if (score1 < bestScore1 || (score1 == bestScore1 && score2 < bestScore2)) {
				bestScore1 = score1;
				bestScore2 = score2;
				bestNode = newNode;
				bestRectIndex = i;
			}
		}

		// if no place found...
		if (bestRectIndex == NO_IDX) {
			// restore the original rectangles
			FOREACH(j, rects)
				newRects[indices[j]] = rects[j];
			rects.Swap(newRects);
			return false;

		}

		// store rectangle
		PlaceRect(bestNode);
		newRects[indices[bestRectIndex]] = bestNode;
		rects.RemoveAt(bestRectIndex);
		indices.RemoveAt(bestRectIndex);
	}
	rects.Swap(newRects);
	return true;
}

void MaxRectsBinPack::PlaceRect(const Rect &node)
{
	for (size_t i = 0; i < freeRectangles.GetSize(); ++i) {
		if (SplitFreeNode(freeRectangles[i], node))
			freeRectangles.RemoveAtMove(i--);
	}

	PruneFreeList();

	usedRectangles.Insert(node);
}

MaxRectsBinPack::Rect MaxRectsBinPack::ScoreRect(int width, int height, FreeRectChoiceHeuristic method, int &score1, int &score2) const
{
	Rect newNode;
	score1 = std::numeric_limits<int>::max();
	score2 = std::numeric_limits<int>::max();
	switch (method) {
	case RectBestShortSideFit: newNode = FindPositionForNewNodeBestShortSideFit(width, height, score1, score2); break;
	case RectBottomLeftRule: newNode = FindPositionForNewNodeBottomLeft(width, height, score1, score2); break;
	case RectContactPointRule: newNode = FindPositionForNewNodeContactPoint(width, height, score1);
		score1 = -score1; // Reverse since we are minimizing, but for contact point score bigger is better.
		break;
	case RectBestLongSideFit: newNode = FindPositionForNewNodeBestLongSideFit(width, height, score2, score1); break;
	case RectBestAreaFit: newNode = FindPositionForNewNodeBestAreaFit(width, height, score1, score2); break;
	}

	// Cannot fit the current rectangle.
	if (newNode.height == 0) {
		score1 = std::numeric_limits<int>::max();
		score2 = std::numeric_limits<int>::max();
	}

	return newNode;
}

/// Computes the ratio of used surface area.
float MaxRectsBinPack::Occupancy() const
{
	unsigned long usedSurfaceArea = 0;
	for (size_t i = 0; i < usedRectangles.GetSize(); ++i)
		usedSurfaceArea += usedRectangles[i].width * usedRectangles[i].height;

	return (float)usedSurfaceArea / (binWidth * binHeight);
}

MaxRectsBinPack::Rect MaxRectsBinPack::FindPositionForNewNodeBottomLeft(int width, int height, int &bestY, int &bestX) const
{
	Rect bestNode;
	memset(&bestNode, 0, sizeof(Rect));

	bestY = std::numeric_limits<int>::max();
	bestX = std::numeric_limits<int>::max();

	for (size_t i = 0; i < freeRectangles.GetSize(); ++i) {
		// Try to place the rectangle in upright (non-flipped) orientation.
		if (freeRectangles[i].width >= width && freeRectangles[i].height >= height) {
			int topSideY = freeRectangles[i].y + height;
			if (topSideY < bestY || (topSideY == bestY && freeRectangles[i].x < bestX)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = width;
				bestNode.height = height;
				bestY = topSideY;
				bestX = freeRectangles[i].x;
			}
		}
		if (freeRectangles[i].width >= height && freeRectangles[i].height >= width) {
			int topSideY = freeRectangles[i].y + width;
			if (topSideY < bestY || (topSideY == bestY && freeRectangles[i].x < bestX)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = height;
				bestNode.height = width;
				bestY = topSideY;
				bestX = freeRectangles[i].x;
			}
		}
	}
	return bestNode;
}

MaxRectsBinPack::Rect MaxRectsBinPack::FindPositionForNewNodeBestShortSideFit(int width, int height, int &bestShortSideFit, int &bestLongSideFit) const
{
	Rect bestNode;
	memset(&bestNode, 0, sizeof(Rect));

	bestShortSideFit = std::numeric_limits<int>::max();
	bestLongSideFit = std::numeric_limits<int>::max();

	for (size_t i = 0; i < freeRectangles.GetSize(); ++i) {
		// Try to place the rectangle in upright (non-flipped) orientation.
		if (freeRectangles[i].width >= width && freeRectangles[i].height >= height) {
			int leftoverHoriz = ABS(freeRectangles[i].width - width);
			int leftoverVert = ABS(freeRectangles[i].height - height);
			int shortSideFit = MINF(leftoverHoriz, leftoverVert);
			int longSideFit = MAXF(leftoverHoriz, leftoverVert);

			if (shortSideFit < bestShortSideFit || (shortSideFit == bestShortSideFit && longSideFit < bestLongSideFit)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = width;
				bestNode.height = height;
				bestShortSideFit = shortSideFit;
				bestLongSideFit = longSideFit;
			}
		}

		if (freeRectangles[i].width >= height && freeRectangles[i].height >= width) {
			int flippedLeftoverHoriz = ABS(freeRectangles[i].width - height);
			int flippedLeftoverVert = ABS(freeRectangles[i].height - width);
			int flippedShortSideFit = MINF(flippedLeftoverHoriz, flippedLeftoverVert);
			int flippedLongSideFit = MAXF(flippedLeftoverHoriz, flippedLeftoverVert);

			if (flippedShortSideFit < bestShortSideFit || (flippedShortSideFit == bestShortSideFit && flippedLongSideFit < bestLongSideFit)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = height;
				bestNode.height = width;
				bestShortSideFit = flippedShortSideFit;
				bestLongSideFit = flippedLongSideFit;
			}
		}
	}
	return bestNode;
}

MaxRectsBinPack::Rect MaxRectsBinPack::FindPositionForNewNodeBestLongSideFit(int width, int height, int &bestShortSideFit, int &bestLongSideFit) const
{
	Rect bestNode;
	memset(&bestNode, 0, sizeof(Rect));

	bestShortSideFit = std::numeric_limits<int>::max();
	bestLongSideFit = std::numeric_limits<int>::max();

	for (size_t i = 0; i < freeRectangles.GetSize(); ++i) {
		// Try to place the rectangle in upright (non-flipped) orientation.
		if (freeRectangles[i].width >= width && freeRectangles[i].height >= height) {
			int leftoverHoriz = ABS(freeRectangles[i].width - width);
			int leftoverVert = ABS(freeRectangles[i].height - height);
			int shortSideFit = MINF(leftoverHoriz, leftoverVert);
			int longSideFit = MAXF(leftoverHoriz, leftoverVert);

			if (longSideFit < bestLongSideFit || (longSideFit == bestLongSideFit && shortSideFit < bestShortSideFit)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = width;
				bestNode.height = height;
				bestShortSideFit = shortSideFit;
				bestLongSideFit = longSideFit;
			}
		}

		if (freeRectangles[i].width >= height && freeRectangles[i].height >= width) {
			int leftoverHoriz = ABS(freeRectangles[i].width - height);
			int leftoverVert = ABS(freeRectangles[i].height - width);
			int shortSideFit = MINF(leftoverHoriz, leftoverVert);
			int longSideFit = MAXF(leftoverHoriz, leftoverVert);

			if (longSideFit < bestLongSideFit || (longSideFit == bestLongSideFit && shortSideFit < bestShortSideFit)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = height;
				bestNode.height = width;
				bestShortSideFit = shortSideFit;
				bestLongSideFit = longSideFit;
			}
		}
	}
	return bestNode;
}

MaxRectsBinPack::Rect MaxRectsBinPack::FindPositionForNewNodeBestAreaFit(int width, int height, int &bestAreaFit, int &bestShortSideFit) const
{
	Rect bestNode;
	memset(&bestNode, 0, sizeof(Rect));

	bestAreaFit = std::numeric_limits<int>::max();
	bestShortSideFit = std::numeric_limits<int>::max();

	for (size_t i = 0; i < freeRectangles.GetSize(); ++i) {
		int areaFit = freeRectangles[i].width * freeRectangles[i].height - width * height;

		// Try to place the rectangle in upright (non-flipped) orientation.
		if (freeRectangles[i].width >= width && freeRectangles[i].height >= height) {
			int leftoverHoriz = ABS(freeRectangles[i].width - width);
			int leftoverVert = ABS(freeRectangles[i].height - height);
			int shortSideFit = MINF(leftoverHoriz, leftoverVert);

			if (areaFit < bestAreaFit || (areaFit == bestAreaFit && shortSideFit < bestShortSideFit)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = width;
				bestNode.height = height;
				bestShortSideFit = shortSideFit;
				bestAreaFit = areaFit;
			}
		}

		if (freeRectangles[i].width >= height && freeRectangles[i].height >= width) {
			int leftoverHoriz = ABS(freeRectangles[i].width - height);
			int leftoverVert = ABS(freeRectangles[i].height - width);
			int shortSideFit = MINF(leftoverHoriz, leftoverVert);

			if (areaFit < bestAreaFit || (areaFit == bestAreaFit && shortSideFit < bestShortSideFit)) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = height;
				bestNode.height = width;
				bestShortSideFit = shortSideFit;
				bestAreaFit = areaFit;
			}
		}
	}
	return bestNode;
}

/// Returns 0 if the two intervals i1 and i2 are disjoint, or the length of their overlap otherwise.
int CommonIntervalLength(int i1start, int i1end, int i2start, int i2end)
{
	if (i1end < i2start || i2end < i1start)
		return 0;
	return MINF(i1end, i2end) - MAXF(i1start, i2start);
}

int MaxRectsBinPack::ContactPointScoreNode(int x, int y, int width, int height) const
{
	int score = 0;

	if (x == 0 || x + width == binWidth)
		score += height;
	if (y == 0 || y + height == binHeight)
		score += width;

	for (size_t i = 0; i < usedRectangles.GetSize(); ++i) {
		if (usedRectangles[i].x == x + width || usedRectangles[i].x + usedRectangles[i].width == x)
			score += CommonIntervalLength(usedRectangles[i].y, usedRectangles[i].y + usedRectangles[i].height, y, y + height);
		if (usedRectangles[i].y == y + height || usedRectangles[i].y + usedRectangles[i].height == y)
			score += CommonIntervalLength(usedRectangles[i].x, usedRectangles[i].x + usedRectangles[i].width, x, x + width);
	}
	return score;
}

MaxRectsBinPack::Rect MaxRectsBinPack::FindPositionForNewNodeContactPoint(int width, int height, int &bestContactScore) const
{
	Rect bestNode;
	memset(&bestNode, 0, sizeof(Rect));

	bestContactScore = -1;

	for (size_t i = 0; i < freeRectangles.GetSize(); ++i) {
		// Try to place the rectangle in upright (non-flipped) orientation.
		if (freeRectangles[i].width >= width && freeRectangles[i].height >= height) {
			int score = ContactPointScoreNode(freeRectangles[i].x, freeRectangles[i].y, width, height);
			if (score > bestContactScore) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = width;
				bestNode.height = height;
				bestContactScore = score;
			}
		}
		if (freeRectangles[i].width >= height && freeRectangles[i].height >= width) {
			int score = ContactPointScoreNode(freeRectangles[i].x, freeRectangles[i].y, height, width);
			if (score > bestContactScore) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = height;
				bestNode.height = width;
				bestContactScore = score;
			}
		}
	}
	return bestNode;
}

bool MaxRectsBinPack::SplitFreeNode(Rect freeNode, const Rect &usedNode)
{
	// Test with SAT if the rectangles even intersect.
	if (usedNode.x >= freeNode.x + freeNode.width || usedNode.x + usedNode.width <= freeNode.x ||
		usedNode.y >= freeNode.y + freeNode.height || usedNode.y + usedNode.height <= freeNode.y)
		return false;

	if (usedNode.x < freeNode.x + freeNode.width && usedNode.x + usedNode.width > freeNode.x) {
		// New node at the top side of the used node.
		if (usedNode.y > freeNode.y && usedNode.y < freeNode.y + freeNode.height) {
			Rect newNode = freeNode;
			newNode.height = usedNode.y - newNode.y;
			freeRectangles.Insert(newNode);
		}

		// New node at the bottom side of the used node.
		if (usedNode.y + usedNode.height < freeNode.y + freeNode.height) {
			Rect newNode = freeNode;
			newNode.y = usedNode.y + usedNode.height;
			newNode.height = freeNode.y + freeNode.height - (usedNode.y + usedNode.height);
			freeRectangles.Insert(newNode);
		}
	}

	if (usedNode.y < freeNode.y + freeNode.height && usedNode.y + usedNode.height > freeNode.y) {
		// New node at the left side of the used node.
		if (usedNode.x > freeNode.x && usedNode.x < freeNode.x + freeNode.width) {
			Rect newNode = freeNode;
			newNode.width = usedNode.x - newNode.x;
			freeRectangles.Insert(newNode);
		}

		// New node at the right side of the used node.
		if (usedNode.x + usedNode.width < freeNode.x + freeNode.width) {
			Rect newNode = freeNode;
			newNode.x = usedNode.x + usedNode.width;
			newNode.width = freeNode.x + freeNode.width - (usedNode.x + usedNode.width);
			freeRectangles.Insert(newNode);
		}
	}

	return true;
}

void MaxRectsBinPack::PruneFreeList()
{
	/*
	///  Would be nice to do something like this, to avoid a Theta(n^2) loop through each pair.
	///  But unfortunately it doesn't quite cut it, since we also want to detect containment.
	///  Perhaps there's another way to do this faster than Theta(n^2).
	if (freeRectangles.size() > 0) {
		clb::sort::QuickSort(freeRectangles.Begin(), freeRectangles.GetSize(), NodeSortCmp);
		for (size_t i = 0; i < freeRectangles.GetSize()-1; ++i)
			if (freeRectangles[i].x == freeRectangles[i+1].x &&
				freeRectangles[i].y == freeRectangles[i+1].y &&
				freeRectangles[i].width == freeRectangles[i+1].width &&
				freeRectangles[i].height == freeRectangles[i+1].height)
			{
				freeRectangles.RemoveAtMove(i--);
			}
	}
	*/

	/// Go through each pair and remove any rectangle that is redundant.
	for (size_t i = 0; i < freeRectangles.GetSize(); ++i)
		for (size_t j = i+1; j < freeRectangles.GetSize(); ++j) {
			if (IsContainedIn(freeRectangles[i], freeRectangles[j])) {
				freeRectangles.RemoveAtMove(i--);
				break;
			}
			if (IsContainedIn(freeRectangles[j], freeRectangles[i])) {
				freeRectangles.RemoveAtMove(j--);
			}
		}
}


/// Compute the appropriate texture atlas size
/// (an approximation since the packing is a heuristic)
int MaxRectsBinPack::ComputeTextureSize(const RectArr& rects)
{
	int area(0), maxSizePatch(0);
	FOREACHPTR(pRect, rects) {
		const Rect& rect = *pRect;
		area += rect.area();
		const int sizePatch(MAXF(rect.width, rect.height));
		if (maxSizePatch < sizePatch)
			maxSizePatch = sizePatch;
	}
	// compute the approximate area
	// considering the best case scenario for the packing algorithm: 0.9 fill
	area = CEIL2INT((1.f/0.9f)*(float)area);
	// compute texture size as power of two
	int sizeTex(MAXF(CEIL2INT(SQRT((float)area)), maxSizePatch));
	return POWI((int)2, CEIL2INT(LOGN((float)sizeTex) / LOGN(2.f)));
}
/*----------------------------------------------------------------*/
