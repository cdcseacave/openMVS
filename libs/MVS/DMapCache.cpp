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


// S T R U C T S ///////////////////////////////////////////////////

DMapCache::DMapCache(DepthDataArr& _arrDepthData, unsigned _loadFlags, size_t _max_memory_bytes)
	:
	loadFlags(_loadFlags), arrDepthData(_arrDepthData),
	maxMemory(_max_memory_bytes), disabledMaxMemory(0), usedMemory(0),
	skipMemoryCheckIdxImage(NO_ID), numImageRead(0)
{
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
		fifo.Put(idxImage);
		return false;
	}
	mutex.unlock();
	const String fileName(ComposeDepthFilePath(arrDepthData[idxImage].GetView().GetID(), "dmap"));
	while (!std::filesystem::is_regular_file(static_cast<const std::string&>(fileName)))
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	arrDepthData[idxImage].Load(fileName, loadFlags);
	ASSERT(!arrDepthData[idxImage].IsEmpty());
	mutex.lock();
	++numImageRead;
	usedMemory += arrDepthData[idxImage].GetMemorySize();
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

size_t DMapCache::ComputeUsedMemory() const {
	std::lock_guard<std::mutex> guard(mutex);
	size_t computedUsedMemory = 0;
	for (const auto& depthData : arrDepthData)
		if (!depthData.IsEmpty())
			computedUsedMemory += depthData.GetMemorySize();
	ASSERT(computedUsedMemory == usedMemory);
	return computedUsedMemory;
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
	usedMemory -= arrDepthData[idxImage].GetMemorySize();
	// release the depth-data; no need to save the depth-data to disk as it is already saved
	arrDepthData[idxImage].Release();
	return true;
}
/*----------------------------------------------------------------*/
