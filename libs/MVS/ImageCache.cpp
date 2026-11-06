/*
* ImageCache.cpp
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

#include "Common.h"
#include "ImageCache.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ImgCache"));

ImageCache::ImageCache(FncDecode _decode, size_t _max_memory_bytes)
	:
	decode(std::move(_decode)),
	maxMemory(_max_memory_bytes), usedMemory(0)
{
	ASSERT(decode);
}

ImageCache::~ImageCache()
{
	Reset();
}

bool ImageCache::Prefetch(const IIndexArr& idxImages, const FncMemorySize& imageMemorySize) {
	if (maxMemory == 0)
		return true;
	// count up front how many fit, so the parallel loop below has no dependency
	// between its iterations and cannot race on the budget
	IIndex numImages(0);
	size_t memory(0);
	while (numImages < idxImages.size()) {
		const size_t size(imageMemorySize(idxImages[numImages]));
		if (memory + size > maxMemory)
			break;
		memory += size;
		++numImages;
	}
	bool bSuccess(true);
	#ifdef _USE_OPENMP
	#pragma omp parallel for shared(bSuccess)
	#endif
	for (int64_t i=0; i<(int64_t)numImages; ++i) {
		cv::Mat image;
		if (!UseImage(idxImages[(IIndex)i], image))
			bSuccess = false;
	}
	return bSuccess;
}

void ImageCache::Reset(size_t max_memory_bytes) {
	std::lock_guard<std::mutex> guard(mutex);
	REPORT_CACHE_HIT_STATS(hitStats, "Image");
	images.clear();
	fifo.Clear();
	usedMemory = 0;
	maxMemory = max_memory_bytes;
	hitStats.Reset();
}

bool ImageCache::UseImage(IIndex idxImage, cv::Mat& image) {
	if (maxMemory == 0) {
		// caching disabled: there is not enough memory to keep even a few images,
		// so decode it for this use only
		return decode(idxImage, image);
	}
	std::unique_lock<std::mutex> lock(mutex);
	while (true) {
		const auto it = images.find(idxImage);
		if (it == images.end()) {
			// claim the decode, so the threads asking for the same image wait for
			// this one instead of decoding it again
			images.emplace(idxImage, cv::Mat());
			break;
		}
		if (!it->second.empty()) {
			hitStats.Hit();
			image = it->second;
			fifo.Put(idxImage);
			return true;
		}
		decoded.wait(lock);
	}
	lock.unlock();
	// decode outside the lock so the workers do not serialize on the misses
	cv::Mat imageDecoded;
	const bool bDecoded(decode(idxImage, imageDecoded));
	lock.lock();
	images.erase(idxImage);
	if (bDecoded) {
		ASSERT(!imageDecoded.empty());
		hitStats.Miss();
		image = imageDecoded;
		const size_t size(ComputeImageMemorySize(imageDecoded));
		if (size <= maxMemory) {
			images[idxImage] = imageDecoded;
			fifo.Put(idxImage);
			usedMemory += size;
			Eject();
		}
	}
	decoded.notify_all();
	return bDecoded;
}

void ImageCache::Eject() {
	while (usedMemory > maxMemory) {
		const IIndex idxOldest(fifo.Pop());
		usedMemory -= ComputeImageMemorySize(images[idxOldest]);
		images.erase(idxOldest);
	}
}
/*----------------------------------------------------------------*/
