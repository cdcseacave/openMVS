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

ImageCache::ImageCache(const ImageArr& _images, size_t _max_memory_bytes)
	:
	images(_images),
	maxMemory(_max_memory_bytes), usedMemory(0)
{
}

ImageCache::~ImageCache()
{
	Reset();
}

bool ImageCache::Prefetch(const IIndexArr& idxImages) {
	if (maxMemory == 0)
		return true;
	// count up front how many fit, so the parallel loop below has no dependency
	// between its iterations and cannot race on the budget
	IIndex numImages(0);
	size_t memory(0);
	while (numImages < idxImages.size()) {
		const size_t size((size_t)images[idxImages[numImages]].GetSize().area() * sizeof(float));
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
		Image32F imageGray;
		if (!UseImage(idxImages[(IIndex)i], imageGray))
			bSuccess = false;
	}
	return bSuccess;
}

void ImageCache::Reset(size_t max_memory_bytes) {
	std::lock_guard<std::mutex> guard(mutex);
	REPORT_CACHE_HIT_STATS(hitStats, "Image");
	imagesGray.clear();
	fifo.Clear();
	usedMemory = 0;
	maxMemory = max_memory_bytes;
	hitStats.Reset();
}

size_t ImageCache::ComputeMemorySize(const ImageArr& images, const IIndexArr& idxImages) {
	size_t memory(0);
	for (IIndex idxImage: idxImages) {
		ASSERT(images[idxImage].HasResolution());
		memory += (size_t)images[idxImage].GetSize().area() * sizeof(float);
	}
	return memory;
}

bool ImageCache::DecodeImage(const Image& imageData, Image32F& imageGray) {
	ASSERT(imageData.HasResolution());
	if (!imageData.image.empty()) {
		// the color pixels are resident, as the SGM fusion modes keep them
		ASSERT(imageData.image.size() == imageData.GetSize());
		imageData.image.toGray(imageGray, cv::COLOR_BGR2GRAY, true);
		return true;
	}
	Image8U3 imageColor;
	const auto pImage(Image::ReadImage(imageData.name, imageColor));
	if (pImage == NULL) {
		LOG("error: failed decoding image '%s'", imageData.name.c_str());
		return false;
	}
	// bring the image to the resolution the depth-maps are estimated at;
	// Image::GetSize() is the size ReloadImage() resolved when it read the header
	if (imageColor.size() != imageData.GetSize())
		cv::resize(imageColor, imageColor, imageData.GetSize(), 0, 0, cv::INTER_AREA);
	imageColor.toGray(imageGray, cv::COLOR_BGR2GRAY, true);
	return true;
}

bool ImageCache::UseImage(IIndex idxImage, Image32F& imageGray) {
	ASSERT(idxImage < images.size());
	if (maxMemory == 0) {
		// caching disabled: there is not enough memory to keep even a few images,
		// so decode it for this use only
		return DecodeImage(images[idxImage], imageGray);
	}
	std::unique_lock<std::mutex> lock(mutex);
	while (true) {
		const auto it = imagesGray.find(idxImage);
		if (it == imagesGray.end()) {
			// claim the decode, so the threads asking for the same image wait for
			// this one instead of decoding it again
			imagesGray.emplace(idxImage, Image32F());
			break;
		}
		if (!it->second.empty()) {
			hitStats.Hit();
			imageGray = it->second;
			fifo.Put(idxImage);
			return true;
		}
		decoded.wait(lock);
	}
	lock.unlock();
	// decode outside the lock so the workers do not serialize on the misses
	Image32F imageDecoded;
	const bool bDecoded(DecodeImage(images[idxImage], imageDecoded));
	lock.lock();
	imagesGray.erase(idxImage);
	if (bDecoded) {
		hitStats.Miss();
		imageGray = imageDecoded;
		const size_t size(ComputeImageMemorySize(imageDecoded));
		if (size <= maxMemory) {
			imagesGray[idxImage] = imageDecoded;
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
		usedMemory -= ComputeImageMemorySize(imagesGray[idxOldest]);
		imagesGray.erase(idxOldest);
	}
}
/*----------------------------------------------------------------*/
