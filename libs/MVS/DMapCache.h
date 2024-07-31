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


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Caches depth-maps to disk.
class DMapCache {
public:
	explicit DMapCache(DepthDataArr& arrDepthData, unsigned loadFlags, size_t max_memory_bytes);

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

	// get the number of times images were read from disk
	uint32_t GetNumImageReads() const { return numImageRead; }

	// eject all images from the cache
	void ClearCache();

	// get the current memory usage (in bytes)
	size_t GetUsedMemory() const { return usedMemory; }
	size_t ComputeUsedMemory() const;

private:
	// eject the least recently used images if the cache size is above max-limit
	bool Eject() const;
	// eject the least recently used image
	bool EjectOldest() const;

private:
	unsigned loadFlags;
	DepthDataArr& arrDepthData;

	// maximum and used memory (in bytes)
	size_t maxMemory, disabledMaxMemory;
	mutable size_t usedMemory;

	// index of the image to skip memory check
	IIndex skipMemoryCheckIdxImage;

	// guard access to variables that are dynamically loaded from disk
	mutable std::mutex mutex;

	// track which images are last accessed
	mutable ListFIFO<IIndex> fifo;

	// number of times images were read from disk (debug only)
	mutable uint32_t numImageRead;
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif
