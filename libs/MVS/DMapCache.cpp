/*
* DMapCache.cpp
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
#include "DMapCache.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("DMapCach"));

DMapCache::DMapCache(DepthDataArr& _arrDepthData, unsigned _loadFlags, size_t _max_memory_bytes, ImageArr* _pImages)
	:
	loadFlags(_loadFlags), arrDepthData(_arrDepthData), pImages(_pImages),
	maxMemory(_max_memory_bytes), disabledMaxMemory(0), usedMemory(0),
	skipMemoryCheckIdxImage(NO_ID)
{
}

DMapCache::~DMapCache()
{
	REPORT_CACHE_HIT_STATS(hitStats, "Depth-map");
}

size_t DMapCache::GetMemorySize(IIndex idxImage) const {
	size_t memory(arrDepthData[idxImage].GetMemorySize());
	if (pImages) {
		const Image8U3& image = (*pImages)[idxImage].image;
		memory += image.total() * image.elemSize();
	}
	return memory;
}

void DMapCache::SetMaxMemory(size_t max_memory_bytes) {
	maxMemory = max_memory_bytes;
	ASSERT(skipMemoryCheckIdxImage == NO_ID);
	Eject();
}

bool DMapCache::UseImage(IIndex idxImage) const {
	ASSERT(idxImage < arrDepthData.size());
	std::lock_guard<std::mutex> guard(mutex);
	ASSERT(arrDepthData[idxImage].IsValid());
	if (!arrDepthData[idxImage].IsEmpty()) {
		hitStats.Hit();
		fifo.Put(idxImage);
		return false;
	}
	mutex.unlock();
	const String fileName(ComposeDepthFilePath(arrDepthData[idxImage].GetView().GetID(), "dmap"));
	while (!std::filesystem::is_regular_file(static_cast<const std::string&>(fileName)))
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	arrDepthData[idxImage].Load(fileName, loadFlags);
	ASSERT(!arrDepthData[idxImage].IsEmpty());
	if (pImages) {
		// decode the image at the resolution its depth-map was estimated at, which
		// is the one Image::width/height were left at when the scene was prepared;
		// on failure the image stays empty and the fusion skips its colors
		Image& imageData = (*pImages)[idxImage];
		if (imageData.image.empty() && !imageData.ReloadImageAtPreparedResolution())
			VERBOSE("warning: image %u could not be decoded; the points it sees stay uncolored", imageData.ID);
	}
	mutex.lock();
	hitStats.Miss();
	usedMemory += (accountedMemory[idxImage] = GetMemorySize(idxImage));
	fifo.Put(idxImage);
	Eject();
	return true;
}

IIndexArr DMapCache::GetCachedImageIndices(bool ordered) const {
	std::lock_guard<std::mutex> guard(mutex);
	IIndexArr cachedImageIndices;
	FOREACH(idxImage, arrDepthData)
		if (!arrDepthData[idxImage].IsEmpty())
			cachedImageIndices.push_back(idxImage);
	if (ordered)
		cachedImageIndices.Sort();
	return cachedImageIndices;
}

bool DMapCache::IsImageCached(IIndex idxImage) const {
	return fifo.Contains(idxImage);
}

void DMapCache::ClearCache() {
	std::lock_guard<std::mutex> guard(mutex);
	skipMemoryCheckIdxImage = NO_ID;
	while (!IsEmpty())
		EjectOldest();
}

bool DMapCache::Eject() const {
	if (maxMemory == 0)
		return true;
	while (usedMemory > maxMemory) {
		if (!EjectOldest())
			return false;
	}
	return true;
}

bool DMapCache::EjectOldest() const {
	ASSERT(!fifo.IsEmpty());
	if (fifo.Back() == skipMemoryCheckIdxImage)
		return false;
	const IIndex idxImage = fifo.Pop();
	// subtract the bytes accounted at load time, NOT the current size: maps grown since
	// (see accountedMemory) would otherwise underflow the counter
	const auto itAccounted(accountedMemory.find(idxImage));
	ASSERT(itAccounted != accountedMemory.end());
	usedMemory -= itAccounted->second;
	accountedMemory.erase(itAccounted);
	// release the depth-data; no need to save the depth-data to disk as it is already saved
	arrDepthData[idxImage].Release();
	if (pImages)
		(*pImages)[idxImage].ReleaseImage();
	return true;
}
/*----------------------------------------------------------------*/
