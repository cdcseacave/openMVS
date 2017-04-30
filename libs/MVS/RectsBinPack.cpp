/*
* RectsBinPack.cpp
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

#include "Common.h"
#include "RectsBinPack.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define RECTPACK_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

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
	int score1, score2;
	switch (method) {
	case RectBestShortSideFit: newNode = FindPositionForNewNodeBestShortSideFit(width, height, score1, score2); break;
	case RectBestLongSideFit: newNode = FindPositionForNewNodeBestLongSideFit(width, height, score2, score1); break;
	case RectBestAreaFit: newNode = FindPositionForNewNodeBestAreaFit(width, height, score1, score2); break;
	case RectBottomLeftRule: newNode = FindPositionForNewNodeBottomLeft(width, height, score1, score2); break;
	case RectContactPointRule: newNode = FindPositionForNewNodeContactPoint(width, height, score1); break;
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
		#ifdef RECTPACK_USE_OPENMP
		#pragma omp parallel
		{
			int privBestScore1 = std::numeric_limits<int>::max();
			int privBestScore2 = std::numeric_limits<int>::max();
			IDX privBestRectIndex = NO_IDX;
			Rect privBestNode;
			#pragma omp for nowait
			for (int_t i=0; i<(int_t)rects.GetSize(); ++i) {
				int score1, score2;
				Rect newNode(ScoreRect(rects[i].width, rects[i].height, method, score1, score2));
				if (score1 < privBestScore1 || (score1 == privBestScore1 && score2 < privBestScore2)) {
					privBestScore1 = score1;
					privBestScore2 = score2;
					privBestNode = newNode;
					privBestRectIndex = i;
				}
			}
			#pragma omp critical
			{
				if (privBestScore1 < bestScore1 || (privBestScore1 == bestScore1 && privBestScore2 < bestScore2)) {
					bestScore1 = privBestScore1;
					bestScore2 = privBestScore2;
					bestNode = privBestNode;
					bestRectIndex = privBestRectIndex;
				}
			}
		}
		#else
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
		#endif

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
	switch (method) {
	case RectBestShortSideFit: return FindPositionForNewNodeBestShortSideFit(width, height, score1, score2);
	case RectBestLongSideFit: return FindPositionForNewNodeBestLongSideFit(width, height, score2, score1);
	case RectBestAreaFit: return FindPositionForNewNodeBestAreaFit(width, height, score1, score2);
	case RectBottomLeftRule: return FindPositionForNewNodeBottomLeft(width, height, score1, score2);
	case RectContactPointRule: { Rect newNode = FindPositionForNewNodeContactPoint(width, height, score1);
		score1 = -score1; // Reverse since we are minimizing, but for contact point score bigger is better.
		return newNode; }
	default: ASSERT("unknown method" == NULL); return Rect();
	}
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


// Compute the appropriate texture atlas size
// (an approximation since the packing is a heuristic)
// (if mult > 0, the returned size is a multiple of that value, otherwise is a power of two)
int MaxRectsBinPack::ComputeTextureSize(const RectArr& rects, int mult)
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
	// compute texture size...
	const int sizeTex(MAXF(CEIL2INT(SQRT((float)area)), maxSizePatch));
	if (mult > 0) {
		// ... as multiple of mult
		return ((sizeTex+mult-1)/mult)*mult;
	}
	// ... as power of two
	return POWI((int)2, CEIL2INT(LOGN((float)sizeTex) / LOGN(2.f)));
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

GuillotineBinPack::GuillotineBinPack()
	: binWidth(0), binHeight(0)
{
}

GuillotineBinPack::GuillotineBinPack(int width, int height)
{
	Init(width, height);
}

void GuillotineBinPack::Init(int width, int height)
{
	binWidth = width;
	binHeight = height;

	#ifndef _RELEASE
	disjointRects.Clear();
	#endif

	// Clear any memory of previously packed rectangles.
	usedRectangles.clear();

	// We start with a single big free rectangle that spans the whole bin.
	Rect n;
	n.x = 0;
	n.y = 0;
	n.width = width;
	n.height = height;

	freeRectangles.clear();
	freeRectangles.push_back(n);
}

bool GuillotineBinPack::Insert(RectArr& rects, bool merge,
							   FreeRectChoiceHeuristic rectChoice, GuillotineSplitHeuristic splitMethod)
{
	// Remember variables about the best packing choice we have made so far during the iteration process.
	size_t bestFreeRect = 0;
	size_t bestRect = 0;
	bool bestFlipped = false;

	// Pack rectangles one at a time until we have cleared the rects array of all rectangles.
	// rects will get destroyed in the process.
	cList<IDX, IDX, 0> indices(rects.GetSize());
	std::iota(indices.Begin(), indices.End(), 0);
	RectArr newRects(rects.GetSize());
	while (!rects.IsEmpty()) {
		// Stores the penalty score of the best rectangle placement - bigger=worse, smaller=better.
		int bestScore = std::numeric_limits<int>::max();

		for (size_t i = 0; i < freeRectangles.size(); ++i) {
			for (size_t j = 0; j < rects.GetSize(); ++j) {
				// If this rectangle is a perfect match, we pick it instantly.
				if (rects[j].width == freeRectangles[i].width && rects[j].height == freeRectangles[i].height) {
					bestFreeRect = i;
					bestRect = j;
					bestFlipped = false;
					bestScore = std::numeric_limits<int>::min();
					i = freeRectangles.size(); // Force a jump out of the outer loop as well - we got an instant fit.
					break;
				}
				// If flipping this rectangle is a perfect match, pick that then.
				else if (rects[j].height == freeRectangles[i].width && rects[j].width == freeRectangles[i].height) {
					bestFreeRect = i;
					bestRect = j;
					bestFlipped = true;
					bestScore = std::numeric_limits<int>::min();
					i = freeRectangles.size(); // Force a jump out of the outer loop as well - we got an instant fit.
					break;
				}
				// Try if we can fit the rectangle upright.
				else if (rects[j].width <= freeRectangles[i].width && rects[j].height <= freeRectangles[i].height) {
					int score = ScoreByHeuristic(rects[j].width, rects[j].height, freeRectangles[i], rectChoice);
					if (score < bestScore) {
						bestFreeRect = i;
						bestRect = j;
						bestFlipped = false;
						bestScore = score;
					}
				}
				// If not, then perhaps flipping sideways will make it fit?
				else if (rects[j].height <= freeRectangles[i].width && rects[j].width <= freeRectangles[i].height) {
					int score = ScoreByHeuristic(rects[j].height, rects[j].width, freeRectangles[i], rectChoice);
					if (score < bestScore) {
						bestFreeRect = i;
						bestRect = j;
						bestFlipped = true;
						bestScore = score;
					}
				}
			}
		}

		// If we didn't manage to find any rectangle to pack, abort.
		if (bestScore == std::numeric_limits<int>::max()) {
			// restore the original rectangles
			FOREACH(j, rects)
				newRects[indices[j]] = rects[j];
			rects.Swap(newRects);
			return false;
		}

		// Otherwise, we're good to go and do the actual packing.
		Rect newNode;
		newNode.x = freeRectangles[bestFreeRect].x;
		newNode.y = freeRectangles[bestFreeRect].y;
		newNode.width = rects[bestRect].width;
		newNode.height = rects[bestRect].height;

		if (bestFlipped)
			std::swap(newNode.width, newNode.height);

		// Remove the free space we lost in the bin.
		SplitFreeRectByHeuristic(freeRectangles[bestFreeRect], newNode, splitMethod);
		freeRectangles.erase(freeRectangles.begin() + bestFreeRect);

		// Remove the rectangle we just packed from the input list.
		newRects[indices[bestRect]] = newNode;
		rects.RemoveAt(bestRect);
		indices.RemoveAt(bestRect);

		// Perform a Rectangle Merge step if desired.
		if (merge)
			MergeFreeList();

		// Remember the new used rectangle.
		usedRectangles.push_back(newNode);

		// Check that we're really producing correct packings here.
		ASSERT(disjointRects.Add(newNode) == true);
	}
	return true;
}

GuillotineBinPack::Rect GuillotineBinPack::Insert(int width, int height, bool merge, FreeRectChoiceHeuristic rectChoice, GuillotineSplitHeuristic splitMethod)
{
	// Find where to put the new rectangle.
	size_t freeNodeIndex = 0;
	Rect newRect = FindPositionForNewNode(width, height, rectChoice, freeNodeIndex);

	// Abort if we didn't have enough space in the bin.
	if (newRect.height == 0)
		return newRect;

	// Remove the space that was just consumed by the new rectangle.
	SplitFreeRectByHeuristic(freeRectangles[freeNodeIndex], newRect, splitMethod);
	freeRectangles.erase(freeRectangles.begin() + freeNodeIndex);

	// Perform a Rectangle Merge step if desired.
	if (merge)
		MergeFreeList();

	// Remember the new used rectangle.
	usedRectangles.push_back(newRect);

	// Check that we're really producing correct packings here.
	ASSERT(disjointRects.Add(newRect) == true);

	return newRect;
}

/// Computes the ratio of used surface area to the total bin area.
float GuillotineBinPack::Occupancy() const
{
	///\todo The occupancy rate could be cached/tracked incrementally instead
	///      of looping through the list of packed rectangles here.
	unsigned long usedSurfaceArea = 0;
	for (size_t i = 0; i < usedRectangles.size(); ++i)
		usedSurfaceArea += usedRectangles[i].width * usedRectangles[i].height;

	return (float)usedSurfaceArea / (binWidth * binHeight);
}

/// Returns the heuristic score value for placing a rectangle of size width*height into freeRect. Does not try to rotate.
int GuillotineBinPack::ScoreByHeuristic(int width, int height, const Rect &freeRect, FreeRectChoiceHeuristic rectChoice)
{
	switch (rectChoice) {
	case RectBestAreaFit: return ScoreBestAreaFit(width, height, freeRect);
	case RectBestShortSideFit: return ScoreBestShortSideFit(width, height, freeRect);
	case RectBestLongSideFit: return ScoreBestLongSideFit(width, height, freeRect);
	case RectWorstAreaFit: return ScoreWorstAreaFit(width, height, freeRect);
	case RectWorstShortSideFit: return ScoreWorstShortSideFit(width, height, freeRect);
	case RectWorstLongSideFit: return ScoreWorstLongSideFit(width, height, freeRect);
	default: ASSERT(false); return std::numeric_limits<int>::max();
	}
}

int GuillotineBinPack::ScoreBestAreaFit(int width, int height, const Rect &freeRect)
{
	return freeRect.width * freeRect.height - width * height;
}

int GuillotineBinPack::ScoreBestShortSideFit(int width, int height, const Rect &freeRect)
{
	int leftoverHoriz = abs(freeRect.width - width);
	int leftoverVert = abs(freeRect.height - height);
	int leftover = MINF(leftoverHoriz, leftoverVert);
	return leftover;
}

int GuillotineBinPack::ScoreBestLongSideFit(int width, int height, const Rect &freeRect)
{
	int leftoverHoriz = abs(freeRect.width - width);
	int leftoverVert = abs(freeRect.height - height);
	int leftover = MAXF(leftoverHoriz, leftoverVert);
	return leftover;
}

int GuillotineBinPack::ScoreWorstAreaFit(int width, int height, const Rect &freeRect)
{
	return -ScoreBestAreaFit(width, height, freeRect);
}

int GuillotineBinPack::ScoreWorstShortSideFit(int width, int height, const Rect &freeRect)
{
	return -ScoreBestShortSideFit(width, height, freeRect);
}

int GuillotineBinPack::ScoreWorstLongSideFit(int width, int height, const Rect &freeRect)
{
	return -ScoreBestLongSideFit(width, height, freeRect);
}

GuillotineBinPack::Rect GuillotineBinPack::FindPositionForNewNode(int width, int height, FreeRectChoiceHeuristic rectChoice, size_t& nodeIndex)
{
	Rect bestNode;

	int bestScore = std::numeric_limits<int>::max();

	/// Try each free rectangle to find the best one for placement.
	for (size_t i = 0; i < freeRectangles.size(); ++i) {
		// If this is a perfect fit upright, choose it immediately.
		if (width == freeRectangles[i].width && height == freeRectangles[i].height) {
			bestNode.x = freeRectangles[i].x;
			bestNode.y = freeRectangles[i].y;
			bestNode.width = width;
			bestNode.height = height;
			bestScore = std::numeric_limits<int>::min();
			nodeIndex = i;
			ASSERT(disjointRects.Disjoint(bestNode));
			break;
		}
		// If this is a perfect fit sideways, choose it.
		else if (height == freeRectangles[i].width && width == freeRectangles[i].height) {
			bestNode.x = freeRectangles[i].x;
			bestNode.y = freeRectangles[i].y;
			bestNode.width = height;
			bestNode.height = width;
			bestScore = std::numeric_limits<int>::min();
			nodeIndex = i;
			ASSERT(disjointRects.Disjoint(bestNode));
			break;
		}
		// Does the rectangle fit upright?
		else if (width <= freeRectangles[i].width && height <= freeRectangles[i].height) {
			int score = ScoreByHeuristic(width, height, freeRectangles[i], rectChoice);

			if (score < bestScore) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = width;
				bestNode.height = height;
				bestScore = score;
				nodeIndex = i;
				ASSERT(disjointRects.Disjoint(bestNode));
			}
		}
		// Does the rectangle fit sideways?
		else if (height <= freeRectangles[i].width && width <= freeRectangles[i].height) {
			int score = ScoreByHeuristic(height, width, freeRectangles[i], rectChoice);

			if (score < bestScore) {
				bestNode.x = freeRectangles[i].x;
				bestNode.y = freeRectangles[i].y;
				bestNode.width = height;
				bestNode.height = width;
				bestScore = score;
				nodeIndex = i;
				ASSERT(disjointRects.Disjoint(bestNode));
			}
		}
	}
	return bestNode;
}

void GuillotineBinPack::SplitFreeRectByHeuristic(const Rect &freeRect, const Rect &placedRect, GuillotineSplitHeuristic method)
{
	// Compute the lengths of the leftover area.
	const int w = freeRect.width - placedRect.width;
	const int h = freeRect.height - placedRect.height;

	// Placing placedRect into freeRect results in an L-shaped free area, which must be split into
	// two disjoint rectangles. This can be achieved with by splitting the L-shape using a single line.
	// We have two choices: horizontal or vertical.	

	// Use the given heuristic to decide which choice to make.

	bool splitHorizontal;
	switch (method) {
	case SplitShorterLeftoverAxis:
		// Split along the shorter leftover axis.
		splitHorizontal = (w <= h);
		break;
	case SplitLongerLeftoverAxis:
		// Split along the longer leftover axis.
		splitHorizontal = (w > h);
		break;
	case SplitMinimizeArea:
		// Maximize the larger area == minimize the smaller area.
		// Tries to make the single bigger rectangle.
		splitHorizontal = (placedRect.width * h > w * placedRect.height);
		break;
	case SplitMaximizeArea:
		// Maximize the smaller area == minimize the larger area.
		// Tries to make the rectangles more even-sized.
		splitHorizontal = (placedRect.width * h <= w * placedRect.height);
		break;
	case SplitShorterAxis:
		// Split along the shorter total axis.
		splitHorizontal = (freeRect.width <= freeRect.height);
		break;
	case SplitLongerAxis:
		// Split along the longer total axis.
		splitHorizontal = (freeRect.width > freeRect.height);
		break;
	default:
		splitHorizontal = true;
		ASSERT(false);
	}

	// Perform the actual split.
	SplitFreeRectAlongAxis(freeRect, placedRect, splitHorizontal);
}

/// This function will add the two generated rectangles into the freeRectangles array. The caller is expected to
/// remove the original rectangle from the freeRectangles array after that.
void GuillotineBinPack::SplitFreeRectAlongAxis(const Rect &freeRect, const Rect &placedRect, bool splitHorizontal)
{
	// Form the two new rectangles.
	Rect bottom;
	bottom.x = freeRect.x;
	bottom.y = freeRect.y + placedRect.height;
	bottom.height = freeRect.height - placedRect.height;

	Rect right;
	right.x = freeRect.x + placedRect.width;
	right.y = freeRect.y;
	right.width = freeRect.width - placedRect.width;

	if (splitHorizontal) {
		bottom.width = freeRect.width;
		right.height = placedRect.height;
	} else // Split vertically
	{
		bottom.width = placedRect.width;
		right.height = freeRect.height;
	}

	// Add the new rectangles into the free rectangle pool if they weren't degenerate.
	if (bottom.width > 0 && bottom.height > 0)
		freeRectangles.push_back(bottom);
	if (right.width > 0 && right.height > 0)
		freeRectangles.push_back(right);

	ASSERT(disjointRects.Disjoint(bottom));
	ASSERT(disjointRects.Disjoint(right));
}

void GuillotineBinPack::MergeFreeList()
{
	#ifndef _RELEASE
	DisjointRectCollection test;
	for (size_t i = 0; i < freeRectangles.size(); ++i)
		ASSERT(test.Add(freeRectangles[i]) == true);
	#endif

	// Do a Theta(n^2) loop to see if any pair of free rectangles could me merged into one.
	// Note that we miss any opportunities to merge three rectangles into one. (should call this function again to detect that)
	for (size_t i = 0; i < freeRectangles.size(); ++i)
		for (size_t j = i+1; j < freeRectangles.size(); ++j) {
			if (freeRectangles[i].width == freeRectangles[j].width && freeRectangles[i].x == freeRectangles[j].x) {
				if (freeRectangles[i].y == freeRectangles[j].y + freeRectangles[j].height) {
					freeRectangles[i].y -= freeRectangles[j].height;
					freeRectangles[i].height += freeRectangles[j].height;
					freeRectangles.erase(freeRectangles.begin() + j);
					--j;
				} else if (freeRectangles[i].y + freeRectangles[i].height == freeRectangles[j].y) {
					freeRectangles[i].height += freeRectangles[j].height;
					freeRectangles.erase(freeRectangles.begin() + j);
					--j;
				}
			} else if (freeRectangles[i].height == freeRectangles[j].height && freeRectangles[i].y == freeRectangles[j].y) {
				if (freeRectangles[i].x == freeRectangles[j].x + freeRectangles[j].width) {
					freeRectangles[i].x -= freeRectangles[j].width;
					freeRectangles[i].width += freeRectangles[j].width;
					freeRectangles.erase(freeRectangles.begin() + j);
					--j;
				} else if (freeRectangles[i].x + freeRectangles[i].width == freeRectangles[j].x) {
					freeRectangles[i].width += freeRectangles[j].width;
					freeRectangles.erase(freeRectangles.begin() + j);
					--j;
				}
			}
		}

	#ifndef _RELEASE
	test.Clear();
	for (size_t i = 0; i < freeRectangles.size(); ++i)
		ASSERT(test.Add(freeRectangles[i]) == true);
	#endif
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////


SkylineBinPack::SkylineBinPack()
	:binWidth(0),
	binHeight(0)
{
}

SkylineBinPack::SkylineBinPack(int width, int height, bool useWasteMap)
{
	Init(width, height, useWasteMap);
}

void SkylineBinPack::Init(int width, int height, bool useWasteMap_)
{
	binWidth = width;
	binHeight = height;

	useWasteMap = useWasteMap_;

	#ifndef _RELEASE
	disjointRects.Clear();
	#endif

	usedSurfaceArea = 0;
	skyLine.clear();
	SkylineNode node;
	node.x = 0;
	node.y = 0;
	node.width = binWidth;
	skyLine.push_back(node);

	if (useWasteMap) {
		wasteMap.Init(width, height);
		wasteMap.GetFreeRectangles().clear();
	}
}

bool SkylineBinPack::Insert(RectArr& rects, LevelChoiceHeuristic method)
{
	cList<IDX, IDX, 0> indices(rects.GetSize());
	std::iota(indices.Begin(), indices.End(), 0);
	RectArr newRects(rects.GetSize());
	while (!rects.IsEmpty()) {
		int bestScore1 = std::numeric_limits<int>::max();
		int bestScore2 = std::numeric_limits<int>::max();
		int bestSkylineIndex = -1;
		IDX bestRectIndex = NO_IDX;
		Rect bestNode;

		#ifdef RECTPACK_USE_OPENMP
		#pragma omp parallel
		{
			int privBestScore1 = std::numeric_limits<int>::max();
			int privBestScore2 = std::numeric_limits<int>::max();
			int privBestSkylineIndex = -1;
			IDX privBestRectIndex = NO_IDX;
			Rect privBestNode;
			#pragma omp for nowait
			for (int_t i=0; i<(int_t)rects.GetSize(); ++i) {
				int score1, score2, index;
				Rect newNode(ScoreRect(rects[i].width, rects[i].height, method, score1, score2, index));
				if (score1 < privBestScore1 || (score1 == privBestScore1 && score2 < privBestScore2)) {
					privBestScore1 = score1;
					privBestScore2 = score2;
					privBestNode = newNode;
					privBestSkylineIndex = index;
					privBestRectIndex = i;
				}
			}
			#pragma omp critical
			{
				if (privBestScore1 < bestScore1 || (privBestScore1 == bestScore1 && privBestScore2 < bestScore2)) {
					bestScore1 = privBestScore1;
					bestScore2 = privBestScore2;
					bestNode = privBestNode;
					bestSkylineIndex = privBestSkylineIndex;
					bestRectIndex = privBestRectIndex;
				}
			}
		}
		#else
		FOREACH(i, rects) {
			int score1, score2, index;
			Rect newNode(ScoreRect(rects[i].width, rects[i].height, method, score1, score2, index));
			if (score1 < bestScore1 || (score1 == bestScore1 && score2 < bestScore2)) {
				bestNode = newNode;
				bestScore1 = score1;
				bestScore2 = score2;
				bestSkylineIndex = index;
				bestRectIndex = i;
			}
		}
		#endif

		// if no place found...
		if (bestRectIndex == NO_IDX) {
			// restore the original rectangles
			FOREACH(j, rects)
				newRects[indices[j]] = rects[j];
			rects.Swap(newRects);
			return false;
		}

		// Perform the actual packing.
		#ifndef _RELEASE
		ASSERT(disjointRects.Disjoint(bestNode));
		disjointRects.Add(bestNode);
		#endif
		AddSkylineLevel(bestSkylineIndex, bestNode);
		usedSurfaceArea += rects[bestRectIndex].area();

		newRects[indices[bestRectIndex]] = bestNode;
		rects.RemoveAt(bestRectIndex);
		indices.RemoveAt(bestRectIndex);
	}
	rects.Swap(newRects);
	return true;
}

SkylineBinPack::Rect SkylineBinPack::Insert(int width, int height, LevelChoiceHeuristic method)
{
	if (useWasteMap) {
		// First try to pack this rectangle into the waste map, if it fits.
		Rect node = wasteMap.Insert(width, height, true, GuillotineBinPack::RectBestShortSideFit,
									GuillotineBinPack::SplitMaximizeArea);
		ASSERT(disjointRects.Disjoint(node));
		if (node.height != 0) {
			Rect newNode;
			newNode.x = node.x;
			newNode.y = node.y;
			newNode.width = node.width;
			newNode.height = node.height;
			usedSurfaceArea += width * height;
			#ifndef _RELEASE
			ASSERT(disjointRects.Disjoint(newNode));
			disjointRects.Add(newNode);
			#endif
			return newNode;
		}
	}
	switch (method) {
	case LevelBottomLeft: return InsertBottomLeft(width, height);
	case LevelMinWasteFit: return InsertMinWaste(width, height);
	default: ASSERT(false); return Rect();
	}
}

SkylineBinPack::Rect SkylineBinPack::ScoreRect(int width, int height, LevelChoiceHeuristic method, int &score1, int &score2, int &index) const
{
	Rect newNode;
	switch (method) {
	case LevelBottomLeft: newNode = FindPositionForNewNodeBottomLeft(width, height, score1, score2, index); break;
	case LevelMinWasteFit: newNode = FindPositionForNewNodeMinWaste(width, height, score2, score1, index); break;
	default: ASSERT(false);
	}
	ASSERT(disjointRects.Disjoint(newNode));
	return newNode;
}

bool SkylineBinPack::RectangleFits(int skylineNodeIndex, int width, int height, int &y) const
{
	int x = skyLine[skylineNodeIndex].x;
	if (x + width > binWidth)
		return false;
	int widthLeft = width;
	int i = skylineNodeIndex;
	y = skyLine[skylineNodeIndex].y;
	while (widthLeft > 0) {
		y = MAXF(y, skyLine[i].y);
		if (y + height > binHeight)
			return false;
		widthLeft -= skyLine[i].width;
		++i;
		ASSERT(i < (int)skyLine.size() || widthLeft <= 0);
	}
	return true;
}

int SkylineBinPack::ComputeWastedArea(int skylineNodeIndex, int width, int height, int y) const
{
	int wastedArea = 0;
	const int rectLeft = skyLine[skylineNodeIndex].x;
	const int rectRight = rectLeft + width;
	for (; skylineNodeIndex < (int)skyLine.size() && skyLine[skylineNodeIndex].x < rectRight; ++skylineNodeIndex) {
		if (skyLine[skylineNodeIndex].x >= rectRight || skyLine[skylineNodeIndex].x + skyLine[skylineNodeIndex].width <= rectLeft)
			break;

		int leftSide = skyLine[skylineNodeIndex].x;
		int rightSide = MINF(rectRight, leftSide + skyLine[skylineNodeIndex].width);
		ASSERT(y >= skyLine[skylineNodeIndex].y);
		wastedArea += (rightSide - leftSide) * (y - skyLine[skylineNodeIndex].y);
	}
	return wastedArea;
}

bool SkylineBinPack::RectangleFits(int skylineNodeIndex, int width, int height, int &y, int &wastedArea) const
{
	bool fits = RectangleFits(skylineNodeIndex, width, height, y);
	if (fits)
		wastedArea = ComputeWastedArea(skylineNodeIndex, width, height, y);

	return fits;
}

void SkylineBinPack::AddWasteMapArea(int skylineNodeIndex, int width, int height, int y)
{
	// int wastedArea = 0; // unused
	const int rectLeft = skyLine[skylineNodeIndex].x;
	const int rectRight = rectLeft + width;
	for (; skylineNodeIndex < (int)skyLine.size() && skyLine[skylineNodeIndex].x < rectRight; ++skylineNodeIndex) {
		if (skyLine[skylineNodeIndex].x >= rectRight || skyLine[skylineNodeIndex].x + skyLine[skylineNodeIndex].width <= rectLeft)
			break;

		int leftSide = skyLine[skylineNodeIndex].x;
		int rightSide = MINF(rectRight, leftSide + skyLine[skylineNodeIndex].width);
		ASSERT(y >= skyLine[skylineNodeIndex].y);

		Rect waste;
		waste.x = leftSide;
		waste.y = skyLine[skylineNodeIndex].y;
		waste.width = rightSide - leftSide;
		waste.height = y - skyLine[skylineNodeIndex].y;

		ASSERT(disjointRects.Disjoint(waste));
		wasteMap.GetFreeRectangles().push_back(waste);
	}
}

void SkylineBinPack::AddSkylineLevel(int skylineNodeIndex, const Rect &rect)
{
	// First track all wasted areas and mark them into the waste map if we're using one.
	if (useWasteMap)
		AddWasteMapArea(skylineNodeIndex, rect.width, rect.height, rect.y);

	SkylineNode newNode;
	newNode.x = rect.x;
	newNode.y = rect.y + rect.height;
	newNode.width = rect.width;
	skyLine.insert(skyLine.begin() + skylineNodeIndex, newNode);

	ASSERT(newNode.x + newNode.width <= binWidth);
	ASSERT(newNode.y <= binHeight);

	for (size_t i = skylineNodeIndex+1; i < skyLine.size(); ++i) {
		ASSERT(skyLine[i-1].x <= skyLine[i].x);

		if (skyLine[i].x < skyLine[i-1].x + skyLine[i-1].width) {
			int shrink = skyLine[i-1].x + skyLine[i-1].width - skyLine[i].x;

			skyLine[i].x += shrink;
			skyLine[i].width -= shrink;

			if (skyLine[i].width <= 0) {
				skyLine.erase(skyLine.begin() + i);
				--i;
			} else
				break;
		} else
			break;
	}
	MergeSkylines();
}

void SkylineBinPack::MergeSkylines()
{
	for (size_t i = 0; i < skyLine.size()-1; ++i)
		if (skyLine[i].y == skyLine[i+1].y) {
			skyLine[i].width += skyLine[i+1].width;
			skyLine.erase(skyLine.begin() + (i+1));
			--i;
		}
}

SkylineBinPack::Rect SkylineBinPack::InsertBottomLeft(int width, int height)
{
	int bestHeight;
	int bestWidth;
	int bestIndex;
	Rect newNode = FindPositionForNewNodeBottomLeft(width, height, bestHeight, bestWidth, bestIndex);

	if (bestIndex != -1) {
		ASSERT(disjointRects.Disjoint(newNode));

		// Perform the actual packing.
		AddSkylineLevel(bestIndex, newNode);

		usedSurfaceArea += width * height;
		#ifndef _RELEASE
		disjointRects.Add(newNode);
		#endif
	}

	return newNode;
}

SkylineBinPack::Rect SkylineBinPack::FindPositionForNewNodeBottomLeft(int width, int height, int &bestHeight, int &bestWidth, int &bestIndex) const
{
	bestHeight = std::numeric_limits<int>::max();
	bestIndex = -1;
	// Used to break ties if there are nodes at the same level. Then pick the narrowest one.
	bestWidth = std::numeric_limits<int>::max();
	Rect newNode;
	for (int i = 0; i < (int)skyLine.size(); ++i) {
		int y;
		if (RectangleFits(i, width, height, y)) {
			if (y + height < bestHeight || (y + height == bestHeight && skyLine[i].width < bestWidth)) {
				bestHeight = y + height;
				bestIndex = i;
				bestWidth = skyLine[i].width;
				newNode.x = skyLine[i].x;
				newNode.y = y;
				newNode.width = width;
				newNode.height = height;
				ASSERT(disjointRects.Disjoint(newNode));
			}
		}
		if (RectangleFits(i, height, width, y)) {
			if (y + width < bestHeight || (y + width == bestHeight && skyLine[i].width < bestWidth)) {
				bestHeight = y + width;
				bestIndex = i;
				bestWidth = skyLine[i].width;
				newNode.x = skyLine[i].x;
				newNode.y = y;
				newNode.width = height;
				newNode.height = width;
				ASSERT(disjointRects.Disjoint(newNode));
			}
		}
	}

	return newNode;
}

SkylineBinPack::Rect SkylineBinPack::InsertMinWaste(int width, int height)
{
	int bestHeight;
	int bestWastedArea;
	int bestIndex;
	Rect newNode = FindPositionForNewNodeMinWaste(width, height, bestHeight, bestWastedArea, bestIndex);

	if (bestIndex != -1) {
		ASSERT(disjointRects.Disjoint(newNode));

		// Perform the actual packing.
		AddSkylineLevel(bestIndex, newNode);

		usedSurfaceArea += width * height;
		#ifndef _RELEASE
		disjointRects.Add(newNode);
		#endif
	}

	return newNode;
}

SkylineBinPack::Rect SkylineBinPack::FindPositionForNewNodeMinWaste(int width, int height, int &bestHeight, int &bestWastedArea, int &bestIndex) const
{
	bestHeight = std::numeric_limits<int>::max();
	bestWastedArea = std::numeric_limits<int>::max();
	bestIndex = -1;
	Rect newNode;
	for (int i = 0; i < (int)skyLine.size(); ++i) {
		int y;
		int wastedArea;

		if (RectangleFits(i, width, height, y, wastedArea)) {
			if (wastedArea < bestWastedArea || (wastedArea == bestWastedArea && y + height < bestHeight)) {
				bestHeight = y + height;
				bestWastedArea = wastedArea;
				bestIndex = i;
				newNode.x = skyLine[i].x;
				newNode.y = y;
				newNode.width = width;
				newNode.height = height;
				ASSERT(disjointRects.Disjoint(newNode));
			}
		}
		if (RectangleFits(i, height, width, y, wastedArea)) {
			if (wastedArea < bestWastedArea || (wastedArea == bestWastedArea && y + width < bestHeight)) {
				bestHeight = y + width;
				bestWastedArea = wastedArea;
				bestIndex = i;
				newNode.x = skyLine[i].x;
				newNode.y = y;
				newNode.width = height;
				newNode.height = width;
				ASSERT(disjointRects.Disjoint(newNode));
			}
		}
	}

	return newNode;
}

/// Computes the ratio of used surface area.
float SkylineBinPack::Occupancy() const
{
	return (float)usedSurfaceArea / (binWidth * binHeight);
}
/*----------------------------------------------------------------*/
