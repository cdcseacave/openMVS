/*
* DMapCache.h
*
* Copyright (c) 2014-2024 SEACAVE
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

#pragma once
#ifndef _MVS_DMAPCACHE_H_
#define _MVS_DMAPCACHE_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "DepthMap.h"
#include "../Common/ListFIFO.h"

#include <unordered_map>


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Caches depth-maps to disk.
//
// Fusion samples the colors of a point from the images of the depth-maps it was
// fused from, which are exactly the ones this cache holds, so it can manage the
// pixels of those images as well: pass the scene images to decode them alongside
// their depth-data and release them together with it, keeping both under the same
// memory budget instead of requiring every image to stay resident.
class DMapCache {
public:
	explicit DMapCache(DepthDataArr& arrDepthData, unsigned loadFlags, size_t max_memory_bytes, ImageArr* pImages = NULL);
	// reports how well the cache did
	~DMapCache();

	// check if the list is empty
	bool IsEmpty() const { ASSERT((usedMemory == 0) == fifo.IsEmpty()); return fifo.IsEmpty(); }

	// set the maximum memory usage (in bytes)
	void SetMaxMemory(size_t max_memory_bytes = 0/*unlimited*/);

	// enable/disable memory usage
	void DisableMemoryCheck() { disabledMaxMemory = maxMemory; maxMemory = 0; }
	void EnableMemoryCheck() { if (disabledMaxMemory) { maxMemory = disabledMaxMemory; disabledMaxMemory = 0; Eject(); } }

	// skip memory check if this image index is to be ejected
	void SkipMemoryCheckIdxImage(IIndex idxImage = NO_ID) { skipMemoryCheckIdxImage = idxImage; }

	// ensure the depth-data is loaded and mark it as recently used:
	// return true if the image was loaded from disk
	bool UseImage(IIndex idxImage) const;

	// get the image indices loaded in cache.
	IIndexArr GetCachedImageIndices(bool ordered = false) const;

	// return true if the key is in the cache
	bool IsImageCached(IIndex idxImage) const;

	// eject all images from the cache
	void ClearCache();

	// get the current memory usage (in bytes)
	size_t GetUsedMemory() const { return usedMemory; }

	// counters tracking how well the cache served its uses so far
	// (numMisses = depth-maps fetched from disk)
	const CacheHitStats& GetHitStats() const { return hitStats; }

private:
	// eject the least recently used images if the cache size is above max-limit
	bool Eject() const;
	// eject the least recently used image
	bool EjectOldest() const;
	// bytes the depth-data of the given image occupies, plus its color pixels
	// when this cache manages them too
	size_t GetMemorySize(IIndex idxImage) const;

private:
	unsigned loadFlags;
	DepthDataArr& arrDepthData;
	// scene images whose pixels are cached along the depth-data, NULL if the
	// caller keeps them resident itself
	ImageArr* pImages;

	// maximum and used memory (in bytes)
	size_t maxMemory, disabledMaxMemory;
	mutable size_t usedMemory;

	// bytes accounted into usedMemory when each image was cached; ejection subtracts exactly
	// this snapshot, so maps grown AFTER caching (a normal-map estimated in place, the fusion
	// phase's priorMap, an adjusted confidence side buffer) can never underflow the counter
	mutable std::unordered_map<IIndex,size_t> accountedMemory;

	// index of the image to skip memory check
	IIndex skipMemoryCheckIdxImage;

	// guard access to variables that are dynamically loaded from disk
	mutable std::mutex mutex;

	// track which images are last accessed
	mutable ListFIFO<IIndex> fifo;

	// depth-maps loaded from disk (misses) and served from the cache (hits)
	mutable CacheHitStats hitStats;
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif
