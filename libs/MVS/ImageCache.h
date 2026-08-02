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

#include <condition_variable>
#include <mutex>
#include <unordered_map>


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// Caches the image intensities used to estimate depth-maps, decoding them on demand.
//
// Decoding every image up front does not scale: a scene of ten thousand views needs
// tens of gigabytes of pixels resident before the first depth-map is estimated, which
// is what keeps the depth-map phase from running on large datasets. Instead the images
// are opened only for their resolution and their pixels are decoded the first time a
// view asks for them, evicting the least recently used ones once the memory budget is
// reached.
//
// Entries store the image already converted to the float intensities the estimator
// consumes, which is also what makes a hit cheaper than the eager path it replaces:
// within a pass a view is used once as the reference image and once more for every
// image listing it as a neighbor, and each of those uses used to redo the color
// conversion of the whole image.
class ImageCache {
public:
	explicit ImageCache(const ImageArr& images, size_t max_memory_bytes = 0/*no caching*/);
	~ImageCache();

	// return the intensities of the given image, decoding them if not cached;
	// the returned image shares its memory with the cache and must not be modified
	bool UseImage(IIndex idxImage, Image32F& imageGray);

	// decode as many of the given images as the budget holds, in the order given,
	// which is also the order the estimation consumes them in; filling the cache
	// while all the cores are still idle keeps the workers from having to decode
	// their way through the first pass one image at a time
	bool Prefetch(const IIndexArr& idxImages);

	// drop everything cached and set the maximum memory usage (in bytes);
	// pass 0 to disable caching, decoding each image every time it is used;
	// reports how well the cache did before forgetting it
	void Reset(size_t max_memory_bytes = 0/*no caching*/);

	// bytes needed to hold the intensities of all the given images
	static size_t ComputeMemorySize(const ImageArr& images, const IIndexArr& idxImages);

	size_t GetMaxMemory() const { return maxMemory; }
	// bytes the cache may still grow by, which is what the rest of the estimation
	// has to leave alone on top of what the cache already holds
	size_t GetFreeMemory() const { return maxMemory > usedMemory ? maxMemory - usedMemory : 0; }
	// number of images decoded from disk
	uint32_t GetNumImageReads() const { return hitStats.numMisses; }

protected:
	// decode the image file and convert it to the intensities the estimator consumes
	static bool DecodeImage(const Image& imageData, Image32F& imageGray);
	static size_t ComputeImageMemorySize(const Image32F& imageGray) {
		return (size_t)imageGray.size().area() * sizeof(float);
	}
	// eject the least recently used images while above the memory limit
	void Eject();

protected:
	const ImageArr& images;

	// maximum and used memory (in bytes); a worker holding an image that gets
	// ejected keeps it alive through the reference count, so the used memory is
	// a lower bound on what the cached images actually occupy
	size_t maxMemory, usedMemory;

	// cached intensities; an entry holding an empty image is a placeholder marking
	// a decode another thread has already started, so the threads finding it wait
	// for that decode instead of repeating it
	std::unordered_map<IIndex, Image32F> imagesGray;
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
