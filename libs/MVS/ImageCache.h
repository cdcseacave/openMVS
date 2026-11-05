/*
* ImageCache.h
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
#ifndef _MVS_IMAGECACHE_H_
#define _MVS_IMAGECACHE_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"
#include "../Common/ListFIFO.h"

// the STL containers and the rest of the standard library this header uses come with
// the Common.h precompiled header; only what the PCH does not provide is included here
#include <condition_variable>
#include <functional>
#include <mutex>


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Caches decoded images, decoding them on demand and evicting the least recently used
// ones once the memory budget is reached; safe to use from any number of threads.
//
// What an entry holds is up to the decoder the cache is built with: the depth-map
// estimation keeps the float intensities it consumes (a hit then also skips the color
// conversion), the Viewer keeps the full resolution color pixels it crops observation
// patches from. Decoding every image up front does not scale: a scene of ten thousand
// views needs tens of gigabytes of pixels resident before the first one is used.
class MVS_API ImageCache {
public:
	// decode the image with the given index; called outside the cache lock, from any thread
	typedef std::function<bool(IIndex, cv::Mat&)> FncDecode;
	// bytes the given image will hold once decoded
	typedef std::function<size_t(IIndex)> FncMemorySize;

	explicit ImageCache(FncDecode decode, size_t max_memory_bytes = 0/*no caching*/);
	~ImageCache();

	// return the given image, decoding it if not cached;
	// the returned image shares its memory with the cache and must not be modified
	bool UseImage(IIndex idxImage, cv::Mat& image);
	template <typename TYPE>
	bool UseImage(IIndex idxImage, TImage<TYPE>& image) {
		cv::Mat decoded;
		if (!UseImage(idxImage, decoded))
			return false;
		ASSERT(decoded.type() == cv::DataType<TYPE>::type);
		image = decoded;
		return true;
	}

	// decode as many of the given images as the budget holds, in the order given,
	// which is also the order the caller consumes them in; filling the cache
	// while all the cores are still idle keeps the workers from having to decode
	// their way through the first pass one image at a time
	bool Prefetch(const IIndexArr& idxImages, const FncMemorySize& imageMemorySize);

	// drop everything cached and set the maximum memory usage (in bytes);
	// pass 0 to disable caching, decoding each image every time it is used;
	// reports how well the cache did before forgetting it
	void Reset(size_t max_memory_bytes = 0/*no caching*/);

	size_t GetMaxMemory() const { return maxMemory; }
	// bytes the cache may still grow by, which is what the rest of the caller's
	// work has to leave alone on top of what the cache already holds
	size_t GetFreeMemory() const { return maxMemory > usedMemory ? maxMemory - usedMemory : 0; }
	// number of images decoded from disk
	uint32_t GetNumImageReads() const { return hitStats.numMisses; }

protected:
	static size_t ComputeImageMemorySize(const cv::Mat& image) {
		return image.total() * image.elemSize();
	}
	// eject the least recently used images while above the memory limit
	void Eject();

protected:
	const FncDecode decode;

	// maximum and used memory (in bytes); a worker holding an image that gets
	// ejected keeps it alive through the reference count, so the used memory is
	// a lower bound on what the cached images actually occupy
	size_t maxMemory, usedMemory;

	// cached images; an entry holding an empty image is a placeholder marking
	// a decode another thread has already started, so the threads finding it wait
	// for that decode instead of repeating it
	std::unordered_map<IIndex, cv::Mat> images;
	std::condition_variable decoded;

	// track which images are last accessed; holds only the entries that finished
	// decoding, the placeholders enter it once they do
	ListFIFO<IIndex> fifo;

	// guard access to the images that are dynamically decoded from disk
	std::mutex mutex;

	// images decoded from disk (misses) and served from the cache (hits)
	CacheHitStats hitStats;
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif
