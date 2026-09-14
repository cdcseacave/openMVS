/*
* SceneSplit.cpp
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

// Sub-scene extraction: Split (image chunks by area and resolution), ExportChunks, SubScene.

#include "Common.h"
#include "Scene.h"


using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));


// split the scene in sub-scenes such that each sub-scene surface does not exceed the given
// maximum sampling area; the area is composed of overlapping samples from different cameras
// taking into account the footprint of each sample (pixels/unit-length, GSD inverse),
// including overlapping samples;
// the indirect goals this method tries to achieve are:
//  - limit the maximum number of images in each sub-scene such that the depth-map fusion
//    can load all sub-scene's depth-maps into memory at once
//  - limit in the same time maximum accumulated images resolution (total number of pixels)
//    per sub-scene in order to allow all images to be loaded and processed during mesh refinement
unsigned Scene::Split(ImagesChunkArr& chunks, float maxArea, int depthMapStep) const
{
	TD_TIMER_STARTD();
	ASSERT(depthMapStep > 0);
	// gather samples from all depth-maps
	const float areaScale(0.01f);
	typedef cList<Point3f::EVec,const Point3f::EVec&,0,4096,uint32_t> Samples;
	typedef TOctree<Samples,float,3> Octree;
	Octree octree;
	FloatArr areas(0, images.size()*4192);
	IIndexArr visibility(0, (IIndex)areas.capacity());
	Unsigned32Arr imageAreas(images.size()); {
		Samples samples(0, (uint32_t)areas.capacity());
		unsigned numDepthMapsMissing(0);
		FOREACH(idxImage, images) {
			const Image& imageData = images[idxImage];
			if (!imageData.IsValid())
				continue;
			// an image can lack a depth-map (e.g. featureless views): count it instead of logging each one
			const String fileName(ComposeDepthFilePath(imageData.ID, "dmap"));
			DepthData depthData;
			if (!File::access(fileName) || !depthData.Load(fileName, 1) || depthData.IsEmpty()) {
				++numDepthMapsMissing;
				continue;
			}
			const IIndex numPointsBegin(visibility.size());
			const Camera camera(imageData.GetCamera(platforms, depthData.depthMap.size()));
			for (int r=(depthData.depthMap.rows%depthMapStep)/2; r<depthData.depthMap.rows; r+=depthMapStep) {
				for (int c=(depthData.depthMap.cols%depthMapStep)/2; c<depthData.depthMap.cols; c+=depthMapStep) {
					const Depth depth = depthData.depthMap(r,c);
					if (depth <= 0)
						continue;
					const Point3f X(Cast<float>(camera.TransformPointI2W(Point3(c,r,depth))));
					if (IsBounded() && !obb.Intersects(X))
						continue;
					areas.emplace_back(camera.GetFootprintImage(X)*areaScale);
					visibility.emplace_back(idxImage);
					samples.emplace_back(X);
				}
			}
			imageAreas[idxImage] = visibility.size()-numPointsBegin;
		}
		if (numDepthMapsMissing > 0)
			VERBOSE("warning: %u depth-maps missing or empty, ignored by the scene split", numDepthMapsMissing);
		if (samples.empty()) {
			VERBOSE("error: no depth-map samples to split the scene by");
			return 0;
		}
		const AABB3f aabb(IsBounded() ? obb.GetAABB() : [&samples]() {
			#if 0
			return AABB3f(samples.data(), samples.size());
			#else
			// try to find a dominant plane, and set the bounding-box center on the plane bottom
			OBB3f obbSamples(samples.data(), samples.size());
			obbSamples.m_ext(0) *= 2;
			#if 0 || defined(_DEBUG)
			// dump box for visualization
			OBB3f::POINT pts[8];
			obbSamples.GetCorners(pts);
			PointCloud pc;
			for (int i=0; i<8; ++i)
				pc.points.emplace_back(pts[i]);
			pc.Save(MAKE_PATH("scene_obb.ply"));
			#endif
			return obbSamples.GetAABB();
			#endif
		}());
		octree.Insert(samples, aabb, [](Octree::IDX_TYPE size, Octree::Type /*radius*/) {
			return size > 128;
		});
		#if 0 && !defined(_RELEASE)
		Octree::DEBUGINFO_TYPE info;
		octree.GetDebugInfo(&info);
		Octree::LogDebugInfo(info);
		#endif
		octree.ResetItems();
	}
	struct AreaInserter {
		const FloatArr& areas;
		float area;
		inline void operator() (const Octree::IDX_TYPE* indices, Octree::SIZE_TYPE size) {
			FOREACHRAWPTR(pIdx, indices, size)
				area += areas[*pIdx];
		}
		inline float PopArea() {
			const float a(area);
			area = 0;
			return a;
		}
	} areaEstimator{areas, 0.f};
	struct ChunkInserter {
		const IIndex numImages;
		const Octree& octree;
		const IIndexArr& visibility;
		ImagesChunkArr& chunks;
		CLISTDEF2(Unsigned32Arr) imagesAreas;
		void operator() (const Octree::CELL_TYPE& parentCell, Octree::Type parentRadius, const UnsignedArr& children) {
			ASSERT(!children.empty());
			ImagesChunk& chunk = chunks.AddEmpty();
			Unsigned32Arr& imageAreas = imagesAreas.AddEmpty();
			imageAreas.resize(numImages);
			imageAreas.Memset(0);
			struct Inserter {
				const IIndexArr& visibility;
				std::unordered_set<IIndex>& images;
				Unsigned32Arr& imageAreas;
				inline void operator() (const Octree::IDX_TYPE* indices, Octree::SIZE_TYPE size) {
					FOREACHRAWPTR(pIdx, indices, size) {
						const IIndex idxImage(visibility[*pIdx]);
						images.emplace(idxImage);
						++imageAreas[idxImage];
					}
				}
			} inserter{visibility, chunk.images, imageAreas};
			if (children.size() == 1) {
				octree.CollectCells(parentCell.GetChild(children.front()), inserter);
				chunk.aabb = parentCell.GetChildAabb(children.front(), parentRadius);
			} else {
				chunk.aabb.Reset();
				for (unsigned c: children) {
					octree.CollectCells(parentCell.GetChild(c), inserter);
					chunk.aabb.Insert(parentCell.GetChildAabb(c, parentRadius));
				}
			}
			if (chunk.images.empty()) {
				chunks.RemoveLast();
				imagesAreas.RemoveLast();
			}
		}
	} chunkInserter{images.size(), octree, visibility, chunks};
	octree.SplitVolume(maxArea, areaEstimator, chunkInserter);
	if (chunks.size() < 2)
		return 0;
	// remove images with very little contribution
	const float minImageContributionRatio(0.3f);
	FOREACH(c, chunks) {
		ImagesChunk& chunk = chunks[c];
		const Unsigned32Arr& chunkImageAreas = chunkInserter.imagesAreas[c];
		float maxAreaRatio = 0;
		for (const IIndex idxImage : chunk.images) {
			const float areaRatio(static_cast<float>(chunkImageAreas[idxImage])/static_cast<float>(imageAreas[idxImage]));
			if (maxAreaRatio < areaRatio)
				maxAreaRatio = areaRatio;
		}
		const float minImageContributionRatioChunk(maxAreaRatio * minImageContributionRatio);
		for (auto it = chunk.images.begin(); it != chunk.images.end(); ) {
			const IIndex idxImage(*it);
			if (static_cast<float>(chunkImageAreas[idxImage])/static_cast<float>(imageAreas[idxImage]) < minImageContributionRatioChunk)
				it = chunk.images.erase(it);
			else
				++it;
		}
	}
	#if 1
	// remove images already completely contained by a larger chunk
	const float minImageContributionRatioLargerChunk(0.9f);
	FOREACH(cSmall, chunks) {
		ImagesChunk& chunkSmall = chunks[cSmall];
		const Unsigned32Arr& chunkSmallImageAreas = chunkInserter.imagesAreas[cSmall];
		FOREACH(cLarge, chunks) {
			const ImagesChunk& chunkLarge = chunks[cLarge];
			if (chunkLarge.images.size() <= chunkSmall.images.size())
				continue;
			const Unsigned32Arr& chunkLargeImageAreas = chunkInserter.imagesAreas[cLarge];
			for (auto it = chunkSmall.images.begin(); it != chunkSmall.images.end(); ) {
				const IIndex idxImage(*it);
				if (chunkSmallImageAreas[idxImage] < chunkLargeImageAreas[idxImage] &&
					static_cast<float>(chunkLargeImageAreas[idxImage])/static_cast<float>(imageAreas[idxImage]) > minImageContributionRatioLargerChunk)
					it = chunkSmall.images.erase(it);
				else
					++it;
			}
		}
	}
	#endif
	// drop the chunks left without images by the pruning above
	unsigned numEmptyChunks(0);
	RFOREACH(c, chunks) {
		if (!chunks[c].images.empty())
			continue;
		chunks.RemoveAt(c);
		chunkInserter.imagesAreas.RemoveAt(c);
		++numEmptyChunks;
	}
	#if 1
	// merge small chunks into larger chunk neighbors
	// TODO: better manage the bounding-box merge
	const unsigned minNumImagesPerChunk(4);
	RFOREACH(cSmall, chunks) {
		ImagesChunk& chunkSmall = chunks[cSmall];
		if (chunkSmall.images.size() > minNumImagesPerChunk)
			continue;
		// find the chunk having the most images in common
		IIndex idxBestChunk;
		unsigned numLargestCommonImages(0);
		FOREACH(cLarge, chunks) {
			if (cSmall == cLarge)
				continue;
			const ImagesChunk& chunkLarge = chunks[cLarge];
			unsigned numCommonImages(0);
			for (const IIndex idxImage: chunkSmall.images)
				if (chunkLarge.images.find(idxImage) != chunkLarge.images.end())
					++numCommonImages;
			if (numCommonImages == 0)
				continue;
			if (numLargestCommonImages < numCommonImages ||
				(numLargestCommonImages == numCommonImages && chunks[idxBestChunk].images.size() < chunkLarge.images.size()))
			{
				numLargestCommonImages = numCommonImages;
				idxBestChunk = cLarge;
			}
		}
		if (numLargestCommonImages == 0) {
			DEBUG_ULTIMATE("warning: small chunk can not be merged (%u chunk, %u images)",
				cSmall, chunkSmall.images.size());
			continue;
		}
		// merge the small chunk and remove it
		ImagesChunk& chunkLarge = chunks[idxBestChunk];
		DEBUG_ULTIMATE("Small chunk merged: %u chunk (%u images) -> %u chunk (%u images)",
			cSmall, chunkSmall.images.size(), idxBestChunk, chunkLarge.images.size());
		chunkLarge.aabb.Insert(chunkSmall.aabb);
		chunkLarge.images.insert(chunkSmall.images.begin(), chunkSmall.images.end());
		chunks.RemoveAt(cSmall);
	}
	#endif
	if (IsBounded()) {
		// make sure the chunks bounding box do not exceed the scene bounding box
		const AABB3f aabb(obb.GetAABB());
		RFOREACH(c, chunks) {
			ImagesChunk& chunk = chunks[c];
			chunk.aabb.BoundBy(aabb);
			if (chunk.aabb.IsEmpty()) {
				DEBUG_ULTIMATE("warning: chunk bounding box is empty");
				chunks.RemoveAt(c);
				++numEmptyChunks;
			}
		}
	}
	if (numEmptyChunks > 0)
		VERBOSE("warning: %u empty chunks removed by the scene split", numEmptyChunks);
	DEBUG_EXTRA("Scene split (%g max-area): %u chunks (%s)", maxArea, chunks.size(), TD_TIMER_GET_FMT().c_str());
	#if 0 || defined(_DEBUG)
	// dump chunks for visualization
	FOREACH(c, chunks) {
		const ImagesChunk& chunk = chunks[c];
		PointCloud pc = pointcloud;
		pc.RemovePointsOutside(OBB3f(OBB3f::MATRIX::Identity(), chunk.aabb.ptMin, chunk.aabb.ptMax));
		pc.Save(String::FormatString(MAKE_PATH("scene_%04u.ply"), c));
	}
	#endif
	return chunks.size();
} // Split

// split the scene in sub-scenes according to the given chunks array, and save them to disk
bool Scene::ExportChunks(const ImagesChunkArr& chunks, const String& path, ARCHIVE_TYPE type) const
{
	FOREACH(chunkID, chunks) {
		const ImagesChunk& chunk = chunks[chunkID];
		IIndexArr idxImages(chunk.images.begin(), chunk.images.end(), true);
		Scene subset = SubScene(idxImages);
		// set scene ROI and keep only the mesh inside it
		subset.obb.Set(OBB3f::MATRIX::Identity(), chunk.aabb.ptMin, chunk.aabb.ptMax);
		if (!subset.mesh.IsEmpty())
			subset.mesh.RemoveFacesOutside(subset.obb);
		// serialize out the current state
		if (!subset.Save(String::FormatString("%s" PATH_SEPARATOR_STR "scene_%04u.mvs", path.c_str(), chunkID), type))
			return false;
	}
	return true;
} // ExportChunks
/*----------------------------------------------------------------*/


// fetch sub-scene composed of the given image indices
Scene Scene::SubScene(const IIndexArr& idxImages) const
{
	ASSERT(!idxImages.empty());
	// nothing to drop if every calibrated image is kept
	const auto isValid([](const Image& image) { return image.IsValid(); });
	if (std::count_if(images.begin(), images.end(), isValid) ==
		std::count_if(idxImages.begin(), idxImages.end(), [&](IIndex idxImage) { return isValid(images[idxImage]); }))
		return *this;
	Scene subScene(nMaxThreads);
	subScene.obb = obb;
	subScene.transform = transform;
	subScene.nCalibratedImages = 0;
	// export images and poses
	std::unordered_map<IIndex,IIndex> mapImages;
	std::unordered_map<uint32_t,uint32_t> mapPlatforms;
	std::unordered_map<PairIdx,PairIdx> mapPlatformCamera;
	for (IIndex idxImage: idxImages) {
		const Image& image = images[idxImage];
		if (!image.IsValid())
			continue;
		const Platform& platform = platforms[image.platformID];
		const Platform::Camera& camera = platform.cameras[image.cameraID];
		const auto platformIt(mapPlatforms.emplace(image.platformID, (uint32_t)mapPlatforms.size()));
		const uint32_t platformID(platformIt.first->second);
		if (platformIt.second) {
			// create new platform
			Platform& subPlatform = subScene.platforms.AddEmpty();
			subPlatform.name = platform.name;
		}
		Platform& subPlatform = subScene.platforms[platformID];
		const auto platformCameraIt(mapPlatformCamera.emplace(PairIdx(image.platformID,image.cameraID), PairIdx(platformID,subPlatform.cameras.size())));
		if (platformCameraIt.second) {
			// create new camera
			subPlatform.cameras.emplace_back(camera);
		}
		mapImages.emplace(idxImage, subScene.images.size());
		Image& subImage = subScene.images.emplace_back(image);
		if (subImage.ID == NO_ID)
			subImage.ID = idxImage;
		subImage.platformID = platformCameraIt.first->second.i;
		subImage.cameraID = platformCameraIt.first->second.j;
		subImage.poseID = subPlatform.poses.size();
		subPlatform.poses.emplace_back(platform.poses[image.poseID]);
		++subScene.nCalibratedImages;
	}
	ASSERT(!mapImages.empty());
	// remap image neighbors
	for (Image& image: subScene.images) {
		ASSERT(image.IsValid());
		RFOREACH(idxN, image.neighbors) {
			ViewScore& neighbor = image.neighbors[idxN];
			const auto itImage(mapImages.find(neighbor.ID));
			if (itImage == mapImages.end()) {
				image.neighbors.RemoveAtMove(idxN);
				continue;
			}
			ASSERT(itImage->second < subScene.images.size());
			neighbor.ID = itImage->second;
		}
	}
	// export points
	FOREACH(idxPoint, pointcloud.points) {
		PointCloud::ViewArr subPointViews;
		PointCloud::WeightArr subPointWeights;
		const PointCloud::ViewArr& views = pointcloud.pointViews[idxPoint];
		FOREACH(idxView, views) {
			const PointCloud::View idxImage = views[idxView];
			const auto it(mapImages.find(idxImage));
			if (it == mapImages.end())
				continue;
			subPointViews.push_back(it->second);
			if (!pointcloud.pointWeights.empty())
				subPointWeights.push_back(pointcloud.pointWeights[idxPoint][idxView]);
		}
		if (subPointViews.size() < 2)
			continue;
		subScene.pointcloud.points.emplace_back(pointcloud.points[idxPoint]);
		subScene.pointcloud.pointViews.emplace_back(std::move(subPointViews));
		if (!subPointWeights.empty())
			subScene.pointcloud.pointWeights.emplace_back(std::move(subPointWeights));
		if (!pointcloud.normals.empty())
			subScene.pointcloud.normals.emplace_back(pointcloud.normals[idxPoint]);
		if (!pointcloud.colors.empty())
			subScene.pointcloud.colors.emplace_back(pointcloud.colors[idxPoint]);
	}
	subScene.mesh = mesh;
	return subScene;
}

#pragma pop_macro("VERBOSE")
