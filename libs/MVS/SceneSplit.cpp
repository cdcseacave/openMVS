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

// Sub-scene extraction: Split (image chunks fitting a memory budget), ExportChunks, SubScene.

#include "Common.h"
#include "Scene.h"


using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Scene   "));


// Memory a sub-scene needs to be processed, predicted from what each stage holds; the constants are
// fitted on measured peak commit (fusion-only DensifyPointCloud and ReconstructMesh runs on T&T and
// EPFL scenes, median relative error 14% and 4%; checked on the sub-scenes of a 12 GB split of T&T
// Church: fusion predicted -6..+12% of the measured 10.4-12.0 GB, mesh 3 GB):
//  - the depth-map fusion holds the depth-maps of the sub-scene's images (every valid pixel of their
//    maps, not only the ones in its box) and the fused points;
//  - the Delaunay reconstruction holds the tetrahedralization of the points left by the 1.5 px merge.
// A sampled depth-map pixel becomes SplitPointsPerDepth fused points (0.034-0.076 measured on T&T
// scenes and sub-scenes), and a fused point SplitVerticesPerPoint Delaunay vertices (0.24-0.37).
namespace {
constexpr float SplitPointsPerDepth = 0.05f;
constexpr float SplitVerticesPerPoint = 0.35f;
struct SplitCost {
	static constexpr float FixedFusion = 3.8f, PerDepthFusion = 0.026f, PerPointFusion = 0.18f; // GB, GB per million
	static constexpr float FixedMesh = 1.7f, PerVertexMesh = 1.0f; // GB, GB per million vertices
};
} // namespace

// the predicted peak memory, in GB, of the depth-map fusion and the mesh reconstruction of a
// (sub-)scene whose images' depth-maps hold numDepths valid pixels and whose fusion gives numPoints
// points: the larger of the two stages
float Scene::PredictSubsceneMemory(double numDepths, double numPoints)
{
	ASSERT(numDepths >= 0 && numPoints >= 0);
	const float mdepths(float(numDepths*1e-6)), mpoints(float(numPoints*1e-6));
	return MAXF(SplitCost::FixedFusion + SplitCost::PerDepthFusion*mdepths + SplitCost::PerPointFusion*mpoints,
		SplitCost::FixedMesh + SplitCost::PerVertexMesh*SplitVerticesPerPoint*mpoints);
}

// split the scene in sub-scenes such that the depth-map fusion and the mesh reconstruction of each
// fit in maxMemory GB: the depth-map samples (every depthMapStep-th pixel of every depth-map) are cut
// recursively into K parts of equal size, a box allotted K parts being split on its longest axis at
// the sample quantile floor(K/2)/K, and K is the smallest count whose largest sub-scene is predicted
// to fit (the depth-maps a sub-scene's images bring do not shrink with its box, so the count is
// searched, starting from a lower bound). Every sub-scene so ends with nearly the
// same predicted memory, just under the budget (the octree area split used before produced
// sub-scenes from nearly empty to over the limit, with no relation between its area and the memory a
// stage needs). The images of a sub-scene are those whose samples fall in its box, minus the ones
// contributing little to it. Returns the predicted peak memory of the largest sub-scene, of the whole
// scene if chunks stays empty (it fits, or no split lowers it), or 0 if the budget is below a
// sub-scene's fixed cost or no depth-map gives a sample.
float Scene::Split(ImagesChunkArr& chunks, float maxMemory, int depthMapStep) const
{
	TD_TIMER_STARTD();
	ASSERT(depthMapStep > 0 && maxMemory > 0 && chunks.empty());
	const float fixedCost(PredictSubsceneMemory(0, 0));
	if (maxMemory <= fixedCost) {
		VERBOSE("error: a sub-scene needs more than %gGB, the budget is %gGB", fixedCost, maxMemory);
		return 0;
	}
	// gather samples from all depth-maps: the ones in the scene ROI place the image, all of them
	// measure its depth-map, loaded whole by the fusion
	typedef uint32_t SIdx;
	std::vector<Point3f> samples;
	IIndexArr visibility;
	Unsigned32Arr imageSamples(images.size()), imageDepths(images.size());
	imageSamples.Memset(0);
	imageDepths.Memset(0);
	{
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
			const Camera camera(imageData.GetCamera(platforms, depthData.depthMap.size()));
			for (int r=(depthData.depthMap.rows%depthMapStep)/2; r<depthData.depthMap.rows; r+=depthMapStep) {
				for (int c=(depthData.depthMap.cols%depthMapStep)/2; c<depthData.depthMap.cols; c+=depthMapStep) {
					const Depth depth = depthData.depthMap(r,c);
					if (depth <= 0)
						continue;
					++imageDepths[idxImage];
					const Point3f X(Cast<float>(camera.TransformPointI2W(Point3(c,r,depth))));
					if (IsBounded() && !obb.Intersects(X))
						continue;
					visibility.emplace_back(idxImage);
					samples.emplace_back(X);
					++imageSamples[idxImage];
				}
			}
		}
		if (numDepthMapsMissing > 0)
			VERBOSE("warning: %u depth-maps missing or empty, ignored by the scene split", numDepthMapsMissing);
		if (samples.empty()) {
			VERBOSE("error: no depth-map samples to split the scene by");
			return 0;
		}
	}
	const double pointsPerSample(SQUARE(depthMapStep)*SplitPointsPerDepth);
	// the images of a box: every image with samples in it, minus the ones whose share of their own
	// samples falling in the box is below 30% of the best share an image has there
	constexpr float minImageContributionRatio(0.3f);
	Unsigned32Arr counts(images.size());
	const auto BoxImages = [&](const std::vector<SIdx>& idxs, IIndexArr& chunkImages) {
		counts.Memset(0);
		for (const SIdx i: idxs)
			++counts[visibility[i]];
		float maxRatio(0);
		FOREACH(idxImage, counts)
			if (counts[idxImage])
				maxRatio = MAXF(maxRatio, (float)counts[idxImage]/(float)imageSamples[idxImage]);
		chunkImages.clear();
		FOREACH(idxImage, counts)
			if (counts[idxImage] && (float)counts[idxImage]/(float)imageSamples[idxImage] >= minImageContributionRatio*maxRatio)
				chunkImages.push_back(idxImage);
	};
	// the predicted memory of a box: its images' whole depth-maps and its points
	const double depthsPerSample(SQUARE(depthMapStep));
	const auto BoxCost = [&](const std::vector<SIdx>& idxs, IIndexArr& chunkImages) -> float {
		BoxImages(idxs, chunkImages);
		double depths(0);
		for (const IIndex idxImage: chunkImages)
			depths += imageDepths[idxImage]*depthsPerSample;
		return PredictSubsceneMemory(depths, idxs.size()*pointsPerSample);
	};
	// balanced recursive bisection into numParts boxes
	struct Leaf { std::vector<SIdx> idxs; AABB3f aabb; };
	AABB3f rootBox(true);
	if (IsBounded()) {
		rootBox = obb.GetAABB();
	} else {
		for (const Point3f& X: samples)
			rootBox.InsertFull(X);
	}
	const auto Partition = [&](int numParts) {
		struct Node { std::vector<SIdx> idxs; AABB3f aabb; int numParts{0}; };
		std::vector<Leaf> leaves;
		std::vector<Node> stack(1);
		stack.front().idxs.resize(samples.size());
		std::iota(stack.front().idxs.begin(), stack.front().idxs.end(), SIdx(0));
		stack.front().aabb = rootBox;
		stack.front().numParts = numParts;
		while (!stack.empty()) {
			Node node(std::move(stack.back()));
			stack.pop_back();
			ASSERT(node.numParts >= 1 && !node.idxs.empty());
			if (node.numParts == 1 || node.idxs.size() < 2) {
				leaves.push_back(Leaf{std::move(node.idxs), node.aabb});
				continue;
			}
			// cut the longest axis of the samples' extent at the quantile giving each side its share
			AABB3f extent(true);
			for (const SIdx i: node.idxs)
				extent.InsertFull(samples[i]);
			int axis(0);
			const Point3f size(extent.GetSize());
			if (size[1] > size[axis]) axis = 1;
			if (size[2] > size[axis]) axis = 2;
			const int numPartsLeft(node.numParts/2);
			const size_t numLeft(MAXF(size_t(1), node.idxs.size()*(size_t)numPartsLeft/(size_t)node.numParts));
			std::nth_element(node.idxs.begin(), node.idxs.begin()+numLeft, node.idxs.end(), [&](SIdx a, SIdx b) {
				return samples[a][axis] < samples[b][axis];
			});
			const float cutAt(samples[node.idxs[numLeft]][axis]);
			Node left, right;
			left.idxs.assign(node.idxs.begin(), node.idxs.begin()+numLeft);
			right.idxs.assign(node.idxs.begin()+numLeft, node.idxs.end());
			left.aabb = right.aabb = node.aabb;
			left.aabb.ptMax[axis] = right.aabb.ptMin[axis] = cutAt;
			left.numParts = numPartsLeft;
			right.numParts = node.numParts-numPartsLeft;
			node.idxs = {};
			stack.emplace_back(std::move(right));
			stack.emplace_back(std::move(left));
		}
		return leaves;
	};
	// the smallest part count whose largest sub-scene fits, searched from a lower bound: the leaves
	// share the points equally, and both stages' cost grows with them (the depth-maps only add)
	const double mpoints(double(samples.size())*pointsPerSample*1e-6);
	int numParts(MAXF(1, MAXF(
		CEIL2INT(SplitCost::PerPointFusion*mpoints/(maxMemory-SplitCost::FixedFusion)),
		CEIL2INT(SplitCost::PerVertexMesh*SplitVerticesPerPoint*mpoints/(maxMemory-SplitCost::FixedMesh)))));
	std::vector<Leaf> leaves;
	std::vector<float> costs;
	float bestMax(FLT_MAX);
	int bestParts(numParts), numStalled(0);
	IIndexArr chunkImages;
	while (true) {
		leaves = Partition(numParts);
		costs.resize(leaves.size());
		float maxCost(0);
		for (size_t l=0; l<leaves.size(); ++l)
			maxCost = MAXF(maxCost, costs[l] = BoxCost(leaves[l].idxs, chunkImages));
		if (maxCost <= maxMemory)
			break;
		// the image depth-maps can put a floor under the sub-scene size: stop when more parts
		// stopped lowering the largest one
		if (maxCost < bestMax*0.99f) {
			bestMax = maxCost;
			bestParts = numParts;
			numStalled = 0;
		} else if (++numStalled >= 4) {
			VERBOSE("warning: no split fits %gGB, the smallest sub-scene peak reached is %.1fGB (%d sub-scenes)", maxMemory, bestMax, bestParts);
			leaves = Partition(numParts = bestParts);
			for (size_t l=0; l<leaves.size(); ++l)
				costs[l] = BoxCost(leaves[l].idxs, chunkImages);
			break;
		}
		++numParts;
	}
	for (size_t l=0; l<leaves.size(); ++l) {
		BoxImages(leaves[l].idxs, chunkImages);
		if (chunkImages.empty())
			continue;
		ImagesChunk& chunk = chunks.AddEmpty();
		chunk.images.insert(chunkImages.begin(), chunkImages.end());
		chunk.aabb = leaves[l].aabb;
		DEBUG_EXTRA("\tsub-scene %u: %u images, %u samples, predicted %.1fGB", chunks.size()-1,
			(unsigned)chunkImages.size(), (unsigned)leaves[l].idxs.size(), costs[l]);
	}
	const float peakCost(*std::max_element(costs.begin(), costs.end()));
	if (chunks.size() < 2) {
		// one box: the whole scene fits, or no split lowers it (warned above)
		chunks.Empty();
		if (peakCost <= maxMemory)
			VERBOSE("Scene fits in %.1fGB (predicted %.1fGB): not split", maxMemory, peakCost);
		return peakCost;
	}
	// remove images already completely contained by a larger chunk, unless that empties a chunk:
	// the budget is met without this, and a chunk dropped would leave its box unreconstructed
	{
		std::vector<Unsigned32Arr> chunkCounts(chunks.size());
		// per chunk, how many of each image's samples fall in its box
		FOREACH(c, chunks) {
			Unsigned32Arr& cc = chunkCounts[c];
			cc.resize(images.size());
			cc.Memset(0);
		}
		for (size_t i=0; i<samples.size(); ++i)
			FOREACH(c, chunks)
				if (chunks[c].aabb.Intersects(samples[i]))
					++chunkCounts[c][visibility[i]];
		const float minImageContributionRatioLargerChunk(0.9f);
		FOREACH(cSmall, chunks) {
			std::unordered_set<IIndex> keptImages(chunks[cSmall].images);
			FOREACH(cLarge, chunks) {
				const ImagesChunk& chunkLarge = chunks[cLarge];
				if (chunkLarge.images.size() <= keptImages.size())
					continue;
				for (auto it = keptImages.begin(); it != keptImages.end(); ) {
					const IIndex idxImage(*it);
					if (chunkCounts[cSmall][idxImage] < chunkCounts[cLarge][idxImage] &&
						static_cast<float>(chunkCounts[cLarge][idxImage])/static_cast<float>(imageSamples[idxImage]) > minImageContributionRatioLargerChunk)
						it = keptImages.erase(it);
					else
						++it;
				}
			}
			if (!keptImages.empty())
				chunks[cSmall].images.swap(keptImages);
		}
	}
	ASSERT(std::none_of(chunks.begin(), chunks.end(), [](const ImagesChunk& chunk) { return chunk.images.empty(); }));
	DEBUG_EXTRA("Scene split (%gGB max-memory): %u chunks (%s)", maxMemory, chunks.size(), TD_TIMER_GET_FMT().c_str());
	return peakCost;
} // Split

// save the sub-scenes of a split scene (Split gave chunks) as scene_XXXX.mvs in the folder path,
// created here: a scene that is not split gets no folder
bool Scene::ExportChunks(const ImagesChunkArr& chunks, const String& path, ARCHIVE_TYPE type) const
{
	ASSERT(!chunks.empty());
	String folder(path);
	Util::ensureFolder(Util::ensureFolderSlash(folder));
	FOREACH(chunkID, chunks) {
		const ImagesChunk& chunk = chunks[chunkID];
		IIndexArr idxImages(chunk.images.begin(), chunk.images.end(), true);
		Scene subset = SubScene(idxImages);
		// set scene ROI and keep only the mesh inside it
		subset.obb.Set(OBB3f::MATRIX::Identity(), chunk.aabb.ptMin, chunk.aabb.ptMax);
		if (!subset.mesh.IsEmpty())
			subset.mesh.RemoveFacesOutside(subset.obb);
		// serialize out the current state
		if (!subset.Save(folder + String::FormatString("scene_%04u.mvs", chunkID), type))
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
