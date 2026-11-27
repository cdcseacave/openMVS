/*
 * SceneBlocks.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "SceneBlocks.h"
#include "Link2D3D.h"
#include "PoseLink.h"
#include "Triangulation.h"
#include "GlobalAlignment.h"

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

namespace {
// Append src to images without copying its features: an Image has no move constructor (its View
// declares a destructor), so a plain move would copy the keypoints; they are moved by hand instead
Image& AppendImage(ImageArr& images, Image& src)
{
	ASSERT(images.size() < images.capacity()); // no reallocation, which would copy every image
	Image features;
	features.MoveFeaturesFrom(src);
	Image& img = images.emplace_back(src);
	img.MoveFeaturesFrom(features);
	return img;
}
} // namespace

SceneBlocks::SceneBlocks(Scene& _scene)
	: scene(_scene)
{
}

void SceneBlocks::TakeWhole()
{
	ASSERT(blocks.empty());
	blocks.emplace_back(std::move(scene));
}

void SceneBlocks::Extract(const ScenePartition& partition)
{
	ASSERT(blocks.empty());
	blockOf.resize(scene.images.size());
	blockOf.MemsetValue(NO_ID);
	#ifndef _RELEASE
	// the clusters and the pool hold every image once
	std::vector<unsigned> count(scene.images.size(), 0);
	for (const IIndexArr& cluster : partition.clusters)
		for (IIndex g : cluster)
			++count[g];
	for (IIndex g : partition.pool)
		++count[g];
	for (unsigned c : count)
		ASSERT(c == 1);
	#endif

	// the budget is shared by the blocks: a partition yielding one real cluster and a crowd of
	// pool images must not leave the one reconstruction that runs with a single thread
	const unsigned nClusters = (unsigned)partition.clusters.size();
	const unsigned nThreadsPerCluster = MAXF(1u, scene.nMaxThreads / MAXF(nClusters, 1u));
	DEBUG_EXTRA("Allocating %u threads per sub-scene (%u clusters, %u parent threads)",
		nThreadsPerCluster, nClusters, scene.nMaxThreads);

	blocks.reserve(nClusters);
	localToGlobal.reserve(nClusters);
	IIndexArr globalToLocal(scene.images.size());
	globalToLocal.MemsetValue(NO_ID);
	for (const IIndexArr& cluster : partition.clusters) {
		ASSERT(!cluster.empty() && std::is_sorted(cluster.begin(), cluster.end()));
		const IIndex blockIdx = (IIndex)blocks.size();
		FOREACH(localID, cluster) {
			globalToLocal[cluster[localID]] = localID;
			blockOf[cluster[localID]] = blockIdx;
		}
		blocks.emplace_back(ExtractBlock(cluster, globalToLocal, nThreadsPerCluster));
		localToGlobal.emplace_back(cluster);
	}
	// Partition intra-block pairs in a single O(|pairs|) pass, preserving pair order
	ImagePairArr crossPairs;
	crossPairs.reserve(scene.pairs.size());
	for (ImagePair& pair : scene.pairs) {
		const IIndex b1 = blockOf[pair.ID1];
		if (b1 != NO_ID && b1 == blockOf[pair.ID2]) {
			pair.ID1 = globalToLocal[pair.ID1];
			pair.ID2 = globalToLocal[pair.ID2];
			ASSERT(pair.ID1 < pair.ID2);
			blocks[b1].pairs.emplace_back(std::move(pair));
		} else {
			crossPairs.emplace_back(std::move(pair));
		}
	}
	scene.pairs = std::move(crossPairs);
	FOREACH(b, blocks)
		VERBOSE("Sub-scene %u: %u images, %u pairs, %u tracks",
			b, blocks[b].images.size(), blocks[b].pairs.size(), blocks[b].tracks.size());
	DEBUG("Clustering: split into %u sub-scenes and %u skipped views, %u cross-sub-scene pairs remain",
		(unsigned)blocks.size(), (unsigned)partition.pool.size(), scene.pairs.size());
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		FOREACH(b, localToGlobal) {
			const IIndexArr& cluster = localToGlobal[b];
			String membership;
			FOREACH(i, cluster) {
				if (i > 0 && cluster[i] == cluster[i-1] + 1)
					continue;
				IIndex last = cluster[i];
				for (IIndex j = i + 1; j < cluster.size() && cluster[j] == last + 1; ++j)
					last = cluster[j];
				if (!membership.empty())
					membership += ", ";
				membership += last == cluster[i] ?
					String::FormatString("%u", cluster[i]) :
					String::FormatString("%u-%u", cluster[i], last);
			}
			DEBUG_ULTIMATE("Clustering: sub-scene %u: %s (%u images)", b, membership.c_str(), (unsigned)cluster.size());
		}
		if (!blocks.empty() && !blocks[0].images.empty() && blocks[0].images[0].View::metadata.HasGPS())
			SceneCluster::ExportClusterPositions(blocks, MAKE_PATH(String("clusters_gps.ply")));
	}
	#endif
}

Scene SceneBlocks::ExtractBlock(
	const IIndexArr& viewIndices,
	const IIndexArr& globalToLocal,
	unsigned nThreads)
{
	Scene subScene(nThreads);
	std::unordered_map<IIndex, IIndex> cameraMap;

	// Move image features from global scene to sub-scene without copying keypoints/descriptors
	subScene.images.reserve(viewIndices.size());
	for (IIndex globalID : viewIndices) {
		Image& img = scene.images[globalID];
		const IIndex globalCamID = img.cameraID;
		auto ret = cameraMap.emplace(globalCamID, subScene.cameras.size());
		if (ret.second)
			subScene.cameras.emplace_back(scene.cameras[globalCamID]->Clone());
		const IIndex localCamID = ret.first->second;
		const IIndex localImgID = subScene.images.size();
		Image& subImg = AppendImage(subScene.images, img);
		subImg.ID = localImgID;
		subImg.cameraID = localCamID;
		subImg.pCamera = subScene.cameras[localCamID];
	}

	// Copy tracks with >= 2 observations in this cluster
	const IIndex blockIdx = viewIndices.empty() ? NO_ID : blockOf[viewIndices[0]];
	for (const Track& srcTrack : scene.tracks) {
		Track dstTrack;
		dstTrack.position = srcTrack.position;
		FOREACH(k, srcTrack.observations) {
			const Observation& obs = srcTrack.observations[k];
			if (blockOf[obs.imageID] != blockIdx)
				continue;
			dstTrack.observations.emplace_back(globalToLocal[obs.imageID], obs.featureID);
			if (k < srcTrack.numInliers)
				++dstTrack.numInliers;
		}
		if (dstTrack.GetNumObservations() >= 2)
			subScene.tracks.emplace_back(std::move(dstTrack));
	}
	return subScene;
}
/*----------------------------------------------------------------*/

unsigned SceneBlocks::MergeInRounds(const ResectionConfig& resectionCfg, const ReconstructionConfig& config)
{
	ASSERT(IsSplit() && !blocks.empty());
	// each round places the blocks and merges one model -- the one carrying the most images in round 1, the one
	// holding block 0 (the merged model read again) after it -- then adjusts it at the loose bar; a block the round
	// does not place stays whole. While blocks are left, the pool step (the whole-scene resection of the images in
	// no block, the final pass's own function) runs on the merged model and the next round reads the blocks left
	// against it. At the fixed point -- the pool step entered no image and the last round placed no block (the
	// second round always reads the blocks left once) -- one more round releases them, merged without their
	// poses, their images left to the final pass's resection. Every round that reads blocks for their poses
	// first runs the weak-image filter on all of them, the merged model among them (FilterWeakImages)
	MergeReport report;
	bool release = false;
	unsigned round = 1;
	// the images the pool steps registered on the merged model, over the rounds: they support no reading of
	// the next merges (GlobalAlignment::ScoreSeam)
	std::unordered_set<IIndex> poolImages;
	// the images the filter took from the merged model, over the rounds: one the pool step registers again is no
	// image entered, or every image the two trade would hold the rounds open
	std::unordered_set<IIndex> filteredImages;
	for (; ; ++round) {
		if (!release) {
			for (const IIndex g : FilterWeakImages())
				filteredImages.insert(g);
			if (blocks.empty()) {
				VERBOSE("warning: the weak-image filter left no block a posed image; every image is left to the resection");
				scene.RecomputeCalibratedImages();
				return round - 1;
			}
		}
		const bool model = hasModel; // the regrouping below makes block 0 the model of the next round
		GlobalAlignment globalAlign(scene, config.globalAlignmentCfg, poolImages);
		globalAlign.MergeScenes(blocks, localToGlobal, report, release, model ? 0u : NO_ID);
		ASSERT(!model || report.poses[0].state == BlockPose::ADMITTED);
		VERBOSE("Hierarchical merge round %u: %u/%u blocks placed, %u images placed, %u images of unplaced blocks %s",
			round, report.numPlaced, report.numBlocks, report.numImagesPlaced, report.numImagesUnplaced,
			release ? "released to the resection" : "kept whole");
		AdjustMergedScene(scene, config);
		[[maybe_unused]] const uint32_t numModelImages = scene.RecomputeCalibratedImages();
		if (release || report.numImagesUnplaced == 0)
			break;
		// the pool step on the merged model: the images in no block (the kept blocks' images hold no features in
		// the global scene, so they are no candidates), closing only when it registered one
		std::vector<bool> posedBefore(scene.images.size());
		FOREACH(g, scene.images)
			posedBefore[g] = scene.images[g].HasPose();
		{
			Resection resection(scene, resectionCfg);
			if (resection.RegisterImages(false))
				FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
		}
		unsigned numEntered = 0;
		FOREACH(g, scene.images)
			if (!posedBefore[g] && scene.images[g].HasPose()) {
				poolImages.insert((IIndex)g);
				numEntered += filteredImages.count((IIndex)g) ? 0u : 1u;
			}
		[[maybe_unused]] const uint32_t numAfterPool = scene.RecomputeCalibratedImages();
		ASSERT(numAfterPool >= numModelImages + numEntered);
		const unsigned numBlocksPlaced = model ? report.numPlaced - 1u : 1u; // block 0, when the merged model, is always placed
		release = numEntered == 0 && numBlocksPlaced == 0;
		std::vector<bool> placed(report.poses.size());
		FOREACH(b, report.poses)
			placed[b] = report.poses[b].state == BlockPose::ADMITTED;
		RegroupAfterMerge(placed);
		VERBOSE("Hierarchical merge round %u: the pool step entered %u images; %u blocks left are read against the merged model%s",
			round + 1, numEntered, (unsigned)blocks.size() - 1u, release ? " and released (nothing changed)" : "");
	}
	return round;
}
/*----------------------------------------------------------------*/

IIndexArr SceneBlocks::FilterWeakImages()
{
	TD_TIMER_START();
	ASSERT(IsSplit() && !blocks.empty());
	// the blocks in parallel, the pairs of the ones left without a posed image appended in block order
	std::vector<IIndexArr> removed(blocks.size()); // global IDs, written by the blocks in parallel
	std::vector<ImagePairArr> released(blocks.size());
	scene.threadPool.detach_loop(IIndex(0), (IIndex)blocks.size(), [&](IIndex b) {
		for (const IIndex l : FilterWeaklyConnectedImages(blocks[b]).all)
			removed[b].push_back(localToGlobal[b][l]);
		if (UnregisteredImages(b).size() == blocks[b].images.size())
			released[b] = DetachAll(b);
	});
	scene.threadPool.wait();
	unsigned numImages = 0, numBlocks = 0;
	FOREACH(b, blocks) {
		numImages += (unsigned)removed[b].size();
		numBlocks += removed[b].empty() ? 0u : 1u;
		for (ImagePair& pair : released[b])
			scene.pairs.emplace_back(std::move(pair));
	}
	// the merged model's images hold its poses in the global scene too
	IIndexArr removedFromModel;
	if (hasModel) {
		removedFromModel.swap(removed[0]);
		for (const IIndex g : removedFromModel) {
			ASSERT(scene.images[g].HasPose());
			scene.images[g].InvalidatePose();
		}
	}
	const IIndexArr newIndex(RemoveEmptyBlocks());
	VERBOSE("Weak-image filter before the merge: %u images removed from %u of %u blocks (%u from the merged model), %u blocks left without a posed image handed to the pool (%s)",
		numImages, numBlocks, (unsigned)newIndex.size(), (unsigned)removedFromModel.size(), (unsigned)std::count(newIndex.begin(), newIndex.end(), NO_ID), TD_TIMER_GET_FMT().c_str());
	return removedFromModel;
}

IIndexArr SceneBlocks::RemoveEmptyBlocks()
{
	IIndexArr newIndex(blocks.size());
	IIndex numKept = 0;
	FOREACH(b, blocks) {
		if (blocks[b].images.empty()) {
			newIndex[b] = NO_ID;
			continue;
		}
		newIndex[b] = numKept;
		if (numKept != b) {
			blocks[numKept] = std::move(blocks[b]);
			localToGlobal[numKept] = std::move(localToGlobal[b]);
		}
		++numKept;
	}
	blocks.erase(blocks.begin() + numKept, blocks.end());
	localToGlobal.erase(localToGlobal.begin() + numKept, localToGlobal.end());
	for (IIndex& b : blockOf)
		if (b != NO_ID)
			b = newIndex[b];
	hasModel = hasModel && newIndex[0] != NO_ID;
	return newIndex;
}
/*----------------------------------------------------------------*/

void SceneBlocks::RegroupAfterMerge(const std::vector<bool>& placed)
{
	ASSERT(IsSplit() && placed.size() == blocks.size());
	std::vector<Scene> kept;
	std::vector<IIndexArr> keptLocalToGlobal;
	FOREACH(b, blocks)
		if (!placed[b]) {
			kept.emplace_back(std::move(blocks[b]));
			keptLocalToGlobal.emplace_back(std::move(localToGlobal[b]));
		}
	blocks.clear();
	localToGlobal.clear();
	blockOf.MemsetValue(NO_ID);
	// block 0: the merged model, every posed image of the global scene, extracted as a cluster is
	IIndexArr model;
	FOREACH(g, scene.images)
		if (scene.images[g].HasPose())
			model.push_back(g);
	ASSERT(!model.empty());
	IIndexArr globalToLocal(scene.images.size());
	globalToLocal.MemsetValue(NO_ID);
	FOREACH(l, model) {
		globalToLocal[model[l]] = l;
		blockOf[model[l]] = 0;
	}
	blocks.emplace_back(ExtractBlock(model, globalToLocal, scene.nMaxThreads));
	ImagePairArr remainingPairs;
	remainingPairs.reserve(scene.pairs.size());
	for (ImagePair& pair : scene.pairs) {
		const IIndex l1 = globalToLocal[pair.ID1], l2 = globalToLocal[pair.ID2];
		if (l1 != NO_ID && l2 != NO_ID) {
			ASSERT(l1 < l2);
			pair.ID1 = l1;
			pair.ID2 = l2;
			blocks.back().pairs.emplace_back(std::move(pair));
		} else {
			remainingPairs.emplace_back(std::move(pair));
		}
	}
	scene.pairs = std::move(remainingPairs);
	blocks.back().RecomputeCalibratedImages();
	hasModel = true;
	// the merged model's tracks now live in block 0; a pool image's observations join them again at the
	// next merge through its pairs (connecting pairs), as at the first
	scene.tracks.Release();
	scene.colors.Release();
	scene.status.nTracks = 0;
	localToGlobal.emplace_back(std::move(model));
	// then the blocks the merge kept whole, as they were
	FOREACH(k, kept) {
		for (const IIndex g : keptLocalToGlobal[k]) {
			ASSERT(blockOf[g] == NO_ID && !scene.images[g].HasPose());
			blockOf[g] = (IIndex)blocks.size();
		}
		blocks.emplace_back(std::move(kept[k]));
		localToGlobal.emplace_back(std::move(keptLocalToGlobal[k]));
	}
}
/*----------------------------------------------------------------*/

IIndex SceneBlocks::LocalOf(IIndex block, IIndex globalID) const
{
	const IIndexArr& l2g = localToGlobal[block];
	const auto it = std::lower_bound(l2g.begin(), l2g.end(), globalID);
	ASSERT(it != l2g.end() && *it == globalID);
	return (IIndex)(it - l2g.begin());
}

IIndexArr SceneBlocks::UnregisteredImages(IIndex block) const
{
	IIndexArr images;
	FOREACH(l, blocks[block].images)
		if (!blocks[block].images[l].HasPose())
			images.push_back(localToGlobal[block][l]);
	return images;
}

ImagePairArr SceneBlocks::Remap(IIndex block, const IIndexArr& newLocalToGlobal, std::unordered_map<IIndex, Image>& inserted)
{
	Scene& blk = blocks[block];
	IIndexArr& l2g = localToGlobal[block];
	// nothing else is indexed by image or track in a block at this point
	ASSERT(blk.colors.empty() && blk.poseUncertainty.empty() && blk.priorPoses.empty());

	// the images: kept ones at their new place, inserted ones at theirs, the others gone
	IIndexArr oldToNew(l2g.size());
	oldToNew.MemsetValue(NO_ID);
	ImageArr images;
	images.reserve(newLocalToGlobal.size());
	IIndex o = 0;
	FOREACH(n, newLocalToGlobal) {
		const IIndex g = newLocalToGlobal[n];
		ASSERT(n == 0 || newLocalToGlobal[n-1] < g);
		while (o < l2g.size() && l2g[o] < g)
			++o;
		if (o < l2g.size() && l2g[o] == g) {
			oldToNew[o] = n;
			AppendImage(images, blk.images[o]).ID = n;
			++o;
		} else {
			const auto it = inserted.find(g);
			ASSERT(it != inserted.end() && blockOf[g] == NO_ID);
			AppendImage(images, it->second).ID = n;
			blockOf[g] = block;
		}
	}
	FOREACH(i, l2g) {
		if (oldToNew[i] != NO_ID)
			continue;
		ASSERT(blockOf[l2g[i]] == block);
		blockOf[l2g[i]] = NO_ID;
	}
	blk.images = std::move(images);

	// the pairs: kept ones renumbered in place, the ones that lost an image handed back in global IDs
	ImagePairArr pairs, removed;
	pairs.reserve(blk.pairs.size());
	for (ImagePair& pair : blk.pairs) {
		const IIndex n1 = oldToNew[pair.ID1], n2 = oldToNew[pair.ID2];
		if (n1 == NO_ID || n2 == NO_ID) {
			pair.ID1 = l2g[pair.ID1];
			pair.ID2 = l2g[pair.ID2];
			ASSERT(pair.ID1 < pair.ID2);
			removed.emplace_back(std::move(pair));
			continue;
		}
		ASSERT(n1 < n2);
		pair.ID1 = n1;
		pair.ID2 = n2;
		pairs.emplace_back(std::move(pair));
	}
	blk.pairs = std::move(pairs);

	// the tracks: observations renumbered in place, the removed images' ones taken out in order (an
	// unposed image is never in an inlier prefix, so no inlier count changes), a track left with fewer
	// than two observations dropped (never an inlier track then)
	TrackArr tracks;
	tracks.reserve(blk.tracks.size());
	for (Track& track : blk.tracks) {
		IIndex numKept = 0;
		FOREACH(k, track.observations) {
			Observation& obs = track.observations[k];
			const IIndex n = oldToNew[obs.imageID];
			if (n == NO_ID) {
				ASSERT(k >= track.numInliers);
				continue;
			}
			obs.imageID = n;
			if (numKept != k)
				track.observations[numKept] = obs;
			++numKept;
		}
		track.observations.resize(numKept);
		if (!track.IsValid()) {
			ASSERT(!track.IsInlier());
			continue;
		}
		tracks.emplace_back(std::move(track));
	}
	blk.tracks = std::move(tracks);
	l2g = newLocalToGlobal;

	// the cameras no image uses any more
	IIndexArr cameraIDs(blk.cameras.size());
	cameraIDs.MemsetValue(NO_ID);
	for (const Image& img : blk.images)
		cameraIDs[img.cameraID] = 0;
	CameraPtrArr cameras;
	FOREACH(c, blk.cameras) {
		if (cameraIDs[c] == NO_ID) {
			delete blk.cameras[c];
			continue;
		}
		cameraIDs[c] = cameras.size();
		cameras.push_back(blk.cameras[c]);
	}
	blk.cameras = std::move(cameras);
	for (Image& img : blk.images) {
		img.cameraID = cameraIDs[img.cameraID];
		ASSERT(img.pCamera == blk.cameras[img.cameraID]);
	}
	return removed;
}

ImagePairArr SceneBlocks::Detach(IIndex block, const IIndexArr& images)
{
	ASSERT(!images.empty() && std::is_sorted(images.begin(), images.end()));
	Scene& blk = blocks[block];
	for (IIndex g : images) {
		Image& img = blk.images[LocalOf(block, g)];
		ASSERT(!img.HasPose());
		scene.images[g].MoveFeaturesFrom(img);
	}
	IIndexArr newLocalToGlobal(0, localToGlobal[block].size());
	std::set_difference(localToGlobal[block].begin(), localToGlobal[block].end(), images.begin(), images.end(),
		std::back_inserter(newLocalToGlobal));
	ASSERT(newLocalToGlobal.size() + images.size() == localToGlobal[block].size());
	std::unordered_map<IIndex, Image> none;
	return Remap(block, newLocalToGlobal, none);
}

ImagePairArr SceneBlocks::DetachAll(IIndex block)
{
	Scene& blk = blocks[block];
	const IIndexArr& l2g = localToGlobal[block];
	FOREACH(l, blk.images) {
		scene.images[l2g[l]].MoveFeaturesFrom(blk.images[l]);
		ASSERT(blockOf[l2g[l]] == block);
		blockOf[l2g[l]] = NO_ID;
	}
	ImagePairArr pairs(std::move(blk.pairs));
	for (ImagePair& pair : pairs) {
		pair.ID1 = l2g[pair.ID1];
		pair.ID2 = l2g[pair.ID2];
		ASSERT(pair.ID1 < pair.ID2);
	}
	blk.Release();
	localToGlobal[block].Release();
	return pairs;
}

void SceneBlocks::Release(IIndex block, const IIndexArr& images)
{
	for (ImagePair& pair : Detach(block, images))
		scene.pairs.emplace_back(std::move(pair));
}

void SceneBlocks::Insert(IIndex block, const IIndexArr& images, const ReconstructionConfig& config, const ResectionConfig& resectionCfg)
{
	ASSERT(!images.empty() && std::is_sorted(images.begin(), images.end()));
	Scene& blk = blocks[block];

	// the images: the global shell, its features moved in, the camera of the block serving its global
	// camera (a clone of the global one when none does)
	std::unordered_map<IIndex, IIndex> cameraOf; // global camera -> local camera
	FOREACH(l, blk.images)
		cameraOf.emplace(scene.images[localToGlobal[block][l]].cameraID, blk.images[l].cameraID);
	std::unordered_map<IIndex, Image> inserted;
	for (IIndex g : images) {
		ASSERT(blockOf[g] == NO_ID);
		Image& shell = scene.images[g];
		ASSERT(shell.HasFeatures());
		Image features;
		features.MoveFeaturesFrom(shell);
		Image& img = inserted.emplace(g, shell).first->second;
		img.MoveFeaturesFrom(features);
		const auto [it, isNew] = cameraOf.emplace(shell.cameraID, blk.cameras.size());
		if (isNew)
			blk.cameras.emplace_back(scene.cameras[shell.cameraID]->Clone());
		img.cameraID = it->second;
		img.pCamera = blk.cameras[it->second];
	}
	IIndexArr newLocalToGlobal(0, localToGlobal[block].size() + images.size());
	std::merge(localToGlobal[block].begin(), localToGlobal[block].end(), images.begin(), images.end(),
		std::back_inserter(newLocalToGlobal));
	[[maybe_unused]] const ImagePairArr removed(Remap(block, newLocalToGlobal, inserted));
	ASSERT(removed.empty());

	// the pairs whose two images the block now holds, out of the global scene in local IDs
	[[maybe_unused]] const uint32_t firstNew = blk.pairs.size();
	ImagePairArr kept;
	kept.reserve(scene.pairs.size());
	for (ImagePair& pair : scene.pairs) {
		if (blockOf[pair.ID1] != block || blockOf[pair.ID2] != block) {
			kept.emplace_back(std::move(pair));
			continue;
		}
		pair.ID1 = LocalOf(block, pair.ID1);
		pair.ID2 = LocalOf(block, pair.ID2);
		ASSERT(pair.ID1 < pair.ID2);
		blk.pairs.emplace_back(std::move(pair));
	}
	scene.pairs = std::move(kept);

	DEBUG_EXTRA("Block %u takes %u pool images: %u pairs", block, images.size(), blk.pairs.size() - firstNew);
	RebuildTracks(block, config, resectionCfg);
}

namespace {
// a track's observations as a key independent of their order (FilterTracks reorders them)
std::string ObservationSetKey(const Track& track)
{
	std::vector<uint64_t> keys(track.observations.size());
	FOREACH(k, track.observations)
		keys[k] = PairIdx(track.observations[k].imageID, track.observations[k].featureID).idx;
	std::sort(keys.begin(), keys.end());
	return std::string(reinterpret_cast<const char*>(keys.data()), keys.size() * sizeof(uint64_t));
}
} // namespace

void SceneBlocks::RebuildTracks(IIndex block, const ReconstructionConfig& config, const ResectionConfig& resectionCfg)
{
	Scene& blk = blocks[block];
	// the tracks as they are, by their observation sets
	TrackArr before(std::move(blk.tracks));
	std::unordered_map<std::string, uint32_t> trackOf;
	trackOf.reserve(before.size());
	FOREACH(t, before)
		trackOf.emplace(ObservationSetKey(before[t]), (uint32_t)t);

	// the block's own track building over its pairs as they are now, read in decreasing composite weight
	// (the order ComputePairsWeights leaves every pair array in, which the block's own build read them in,
	// a pair of equal weight in its place); the block's pairs are put back in their order afterwards
	std::vector<uint32_t> order(blk.pairs.size());
	std::iota(order.begin(), order.end(), 0u);
	std::stable_sort(order.begin(), order.end(), [&blk](uint32_t p, uint32_t q) {
		return blk.pairs[p].GetCompositeWeight() > blk.pairs[q].GetCompositeWeight();
	});
	const auto Permute = [&blk](const std::vector<uint32_t>& from) {
		ImagePairArr pairs;
		pairs.reserve(blk.pairs.size());
		for (uint32_t p : from)
			pairs.emplace_back(std::move(blk.pairs[p]));
		blk.pairs.swap(pairs);
	};
	Permute(order);
	BuildTracks(blk, config.minPairWeight, config.trackConflictCfg);
	std::vector<uint32_t> back(order.size());
	FOREACH(k, order)
		back[order[k]] = (uint32_t)k;
	Permute(back);

	// a track holding the same observations as before is the same track: its position, inlier state and
	// observation order stay; every other one is triangulated and filtered as the resection does after a
	// registration, the unchanged ones put back as they were afterwards
	std::vector<std::pair<uint32_t, uint32_t>> unchanged; // (new index, old index)
	FOREACH(t, blk.tracks) {
		const auto it = trackOf.find(ObservationSetKey(blk.tracks[t]));
		if (it == trackOf.end())
			continue;
		blk.tracks[t] = before[it->second];
		unchanged.emplace_back((uint32_t)t, it->second);
	}
	const unsigned numChanged = (unsigned)(blk.tracks.size() - unchanged.size());
	TriangulateTracks(blk, true, resectionCfg.maxReprojError, resectionCfg.minAngleThreshold);
	FilterTracks(blk, resectionCfg.maxReprojError, resectionCfg.minAngleThreshold, resectionCfg.multDepthNear, resectionCfg.multDepthFar);
	for (const auto& [t, o] : unchanged)
		blk.tracks[t] = before[o];
	DEBUG_EXTRA("Block %u tracks rebuilt: %u tracks (%u before), %u unchanged, %u new or changed",
		block, blk.tracks.size(), before.size(), (unsigned)unchanged.size(), numChanged);
}

void SceneBlocks::ResectPool(const ResectionConfig& resectionCfg, const ReconstructionConfig& config)
{
	TD_TIMER_START();
	ASSERT(IsSplit());
	const IIndexArr origin(blockOf); // where each image was before: its block, or NO_ID for the partition's pool
	const auto NumPool = [this]() {
		return (unsigned)std::count(blockOf.begin(), blockOf.end(), NO_ID);
	};
	const unsigned numPartitionPool = NumPool();

	// every block hands its unregistered images to the pool, a block whose initialization failed all of
	// them; the blocks in parallel, their pairs appended in block order
	std::vector<ImagePairArr> released(blocks.size());
	std::vector<uint8_t> failed(blocks.size(), 0); // written by the blocks in parallel
	scene.threadPool.detach_loop(IIndex(0), (IIndex)blocks.size(), [&](IIndex b) {
		if (!blocks[b].status.nState.isSet(Scene::Status::STATE::CALIBRATED)) {
			failed[b] = 1;
			released[b] = DetachAll(b);
			return;
		}
		const IIndexArr images(UnregisteredImages(b));
		if (!images.empty())
			released[b] = Detach(b, images);
	});
	scene.threadPool.wait();
	for (ImagePairArr& pairs : released)
		for (ImagePair& pair : pairs)
			scene.pairs.emplace_back(std::move(pair));
	released.clear();
	VERBOSE("Pool resection: %u images in the pool, %u left unregistered by their blocks and %u by the partition; %u blocks failed to initialize",
		NumPool(), NumPool() - numPartitionPool, numPartitionPool, (unsigned)std::count(failed.begin(), failed.end(), 1));

	// rounds of passes: each pool image to the block its evidence weighs most on among the ones it has
	// not tried this round, the blocks' resections in parallel, the images they refuse back to the pool
	std::map<std::pair<IIndex, IIndex>, unsigned> moves; // (origin block, NO_ID for the partition's pool; target block) -> images
	unsigned numMoved = 0;
	for (unsigned round = 1; ; ++round) {
		std::unordered_map<IIndex, std::unordered_set<IIndex>> tried; // pool image -> the blocks it tried this round
		unsigned numMovedRound = 0, numTriedRound = 0, numPasses = 0;
		for (;;) {
			// the evidence each block holds of each pool image: the weight of its links to the block's registered images
			std::vector<uint8_t> registered(scene.images.size(), 0);
			FOREACH(b, blocks)
				FOREACH(l, blocks[b].images)
					if (blocks[b].images[l].HasPose())
						registered[localToGlobal[b][l]] = 1;
			std::unordered_map<IIndex, std::unordered_map<IIndex, float>> weights; // pool image -> block -> weight
			ForEachLink(scene.pairs,
				[&registered](IIndex g) { return registered[g] != 0; },
				[&](IIndex g, IIndex h, const ImagePair& pair) {
					if (blockOf[g] == NO_ID)
						weights[g][blockOf[h]] += pair.GetCompositeWeight();
				});
			IIndexArr candidates(0, (IIndex)weights.size());
			for (const auto& it : weights)
				candidates.push_back(it.first);
			candidates.Sort();
			std::map<IIndex, IIndexArr> batches; // block -> the pool images it tries in this pass (sorted)
			for (IIndex g : candidates) {
				const auto itTried = tried.find(g);
				IIndex best = NO_ID;
				float bestWeight = 0.f;
				for (const auto& [b, weight] : weights[g]) {
					if (itTried != tried.end() && itTried->second.count(b))
						continue;
					if (weight > bestWeight || (weight == bestWeight && b < best)) {
						best = b;
						bestWeight = weight;
					}
				}
				if (best != NO_ID)
					batches[best].push_back(g);
			}
			if (batches.empty())
				break; // no pool image has an untried block it has evidence in: the round is over
			++numPasses;

			// each batch into its block, then every block with a batch tries it with its own resection; the
			// batch is all the block holds unposed, so the resection tries exactly it
			std::vector<IIndex> targets;
			std::unordered_map<IIndex, unsigned> numCorrInserted; // pool image -> its observations on the block's inlier tracks
			for (const auto& [b, batch] : batches) {
				Insert(b, batch, config, resectionCfg);
				const std::vector<TrackObservationCount> counts(CountTrackObservations(blocks[b]));
				FOREACH(l, blocks[b].images)
					if (!blocks[b].images[l].HasPose())
						numCorrInserted[localToGlobal[b][l]] = counts[l].observations;
				ASSERT(UnregisteredImages(b).size() == batch.size());
				targets.push_back(b);
			}
			const unsigned nThreads = MAXF(1u, scene.nMaxThreads / (unsigned)targets.size());
			scene.threadPool.detach_loop(IIndex(0), (IIndex)targets.size(), [&](IIndex i) {
				Scene& blk = blocks[targets[i]];
				blk.nMaxThreads = nThreads;
				Resection resection(blk, resectionCfg);
				if (resection.RegisterImages(false))
					FilterTracks(blk, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
			});
			scene.threadPool.wait();

			// what each block registered stays there, the rest returns to the pool
			for (const auto& [b, batch] : batches) {
				const Scene& blk = blocks[b];
				const std::vector<TrackObservationCount> counts(CountTrackObservations(blk)); // at the end of the resection
				IIndexArr refused;
				for (IIndex g : batch) {
					const IIndex l = LocalOf(b, g);
					const bool registeredHere = blk.images[l].HasPose();
					DEBUG("Pool resection round %u: image %u (from block %d) -> block %u: %s (%u correspondences when inserted, %u at the end, %u inliers)",
						round, g, origin[g] == NO_ID ? -1 : (int)origin[g], b, registeredHere ? "registered" : "refused",
						numCorrInserted[g], counts[l].observations, counts[l].inliers);
					tried[g].insert(b);
					++numTriedRound;
					if (registeredHere) {
						++moves[std::make_pair(origin[g], b)];
						++numMovedRound;
					} else {
						refused.push_back(g);
					}
				}
				// a refused image leaves with its observations; what it joined of the block's tracks stays, the
				// points it let two of the block's views share with it
				if (!refused.empty())
					Release(b, refused);
			}
		}
		numMoved += numMovedRound;
		VERBOSE("Pool resection round %u: %u passes, %u attempts, %u images registered, %u left in the pool (%s)",
			round, numPasses, numTriedRound, numMovedRound, NumPool(), TD_TIMER_GET_FMT().c_str());
		if (numMovedRound == 0 || NumPool() == 0)
			break;
	}

	// the blocks left empty (their initialization failed) are removed, the others renumbered in order
	const IIndexArr newIndex(RemoveEmptyBlocks());
	String renumbered;
	unsigned numDropped = 0;
	FOREACH(b, newIndex) {
		if (newIndex[b] == NO_ID) {
			ASSERT(failed[b]);
			++numDropped;
		} else if (newIndex[b] != b)
			renumbered += String::FormatString(" %u->%u", b, newIndex[b]);
	}

	String perMove;
	for (const auto& [move, n] : moves)
		perMove += String::FormatString(" %d->%u:%u", move.first == NO_ID ? -1 : (int)move.first, move.second, n);
	VERBOSE("Pool resection: %u images registered into blocks (from->to:images, -1 the partition's pool:%s), %u left in the pool, %u blocks dropped%s%s (%s)",
		numMoved, perMove.c_str(), NumPool(), numDropped, renumbered.empty() ? "" : ", renumbered", renumbered.c_str(), TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/
