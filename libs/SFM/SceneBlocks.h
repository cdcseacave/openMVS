/*
 * SceneBlocks.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_SCENEBLOCKS_H_
#define _SFM_SCENEBLOCKS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Scene.h"
#include "Resection.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

/*
 * Hierarchical SfM -- the blocks and the pool
 * ===========================================
 *
 * Owns the state between the partition (SceneCluster) and the merge (GlobalAlignment): one Scene
 * per block, each with its own cameras, images, pairs and tracks in local image IDs, and the pool,
 * the images no block holds, which stay in the global scene.
 *
 * Invariants:
 * - a block's local IDs are ordered as the global ones (localToGlobal strictly increasing), so a
 *   pair keeps ID1 < ID2 through every remap between local and global IDs;
 * - every image is in exactly one block or in the pool (blockOf);
 * - features exist in one place only: a block image holds its keypoints and descriptors, its
 *   global shell holds none; a pool image keeps them in the global scene;
 * - a pair lives in the block holding both its images, else in the global scene with global IDs
 *   (a pair with a pool image or joining two blocks).
 *
 * Between the blocks' own reconstruction and the merge, the pool resection (ResectPool) moves images
 * between blocks through the pool: every block hands back the images it could not register, and each
 * pool image is offered to the block its verified pairs to registered images weigh most on, where
 * that block's ordinary resection tries it. Release and Insert are the two moves; both renumber the
 * block through one remap and keep every invariant above.
 */
class SFM_API SceneBlocks
{
public:
	std::vector<Scene> blocks;            // one scene per block, local image IDs ordered as the global ones
	std::vector<IIndexArr> localToGlobal; // per block, local image ID -> global image ID (strictly increasing)
	IIndexArr blockOf;                    // per global image, the block holding it (NO_ID: the pool); empty when the whole scene is one block

public:
	explicit SceneBlocks(Scene& scene);

	// whether the scene was split into blocks (and a pool); false when the whole scene is the one block
	bool IsSplit() const { return !blockOf.empty(); }

	/**
	 * @brief The whole scene as the one block, moved in with its IDs unchanged (no partition, no pool)
	 */
	void TakeWhole();

	/**
	 * @brief One block per cluster of the partition: cameras cloned, images copied with their
	 * features MOVED in, every pair of two images of the block MOVED in (remapped to local IDs);
	 * the pool's images keep their features in the global scene, and every pair with a pool image
	 * or joining two blocks stays there. The scene's threads are split over the blocks.
	 */
	void Extract(const ScenePartition& partition);

	/**
	 * @brief Hand images of a block back to the pool: their features return to the global scene, their
	 * pairs too (in global IDs), their track observations are removed (a track left with fewer than two
	 * observations is dropped), and the block is renumbered
	 * @param images global IDs, sorted, held by the block, none of them posed (never in an inlier prefix)
	 */
	void Release(IIndex block, const IIndexArr& images);

	/**
	 * @brief Move pool images into a block: their features, their pairs to the block's images and to each
	 * other (in local IDs, the block's pairs kept in decreasing composite weight), then the block's tracks
	 * rebuilt by its own track building (RebuildTracks), as if the images had been in the block from the start
	 * @param images global IDs, sorted, all in the pool
	 */
	void Insert(IIndex block, const IIndexArr& images, const ReconstructionConfig& config, const ResectionConfig& resectionCfg);

	/**
	 * @brief The pool resection, between the blocks' reconstruction and the merge: every block hands its
	 * unregistered images to the pool (a block whose initialization failed hands all of them and is
	 * dropped); then, pass after pass, each pool image goes to the block not yet tried this round that its
	 * verified pairs to registered images weigh most on (ForEachRegisteredPair), the blocks with
	 * images to try run their ordinary resection on them in parallel, and what a block does not register
	 * returns to the pool with its observations (Release); what it joined of the block's tracks stays. A
	 * round ends when no pool image has an untried block it has evidence in; rounds repeat until one moves
	 * nothing. A block that registers images is settled as the blocks'
	 * reconstruction leaves one (FilterTracks at config's bars). Blocks left empty are removed.
	 */
	void ResectPool(const ResectionConfig& resectionCfg, const ReconstructionConfig& config);

private:
	// one block from a sorted cluster of global image IDs (globalToLocal maps them to the block)
	Scene ExtractBlock(const IIndexArr& viewIndices, const IIndexArr& globalToLocal, unsigned nThreads);

	// The block's tracks built again from its pairs by BuildTracks (the block's weight bar and conflict
	// settling): a track holding the same observations as before stays as it was (position, inlier state,
	// observation order); every new or changed one is triangulated and filtered as the resection does after
	// a registration (TriangulateTracks and FilterTracks at the resection's bars)
	void RebuildTracks(IIndex block, const ReconstructionConfig& config, const ResectionConfig& resectionCfg);

	// Renumber a block to the sorted membership newLocalToGlobal: kept images move to their new local
	// index, the images of `inserted` (by global ID) take theirs, the others are removed with their track
	// observations (a track left with fewer than two is dropped) and the cameras no image uses any more;
	// blockOf follows. Returns the pairs that lost an image, in global IDs.
	ImagePairArr Remap(IIndex block, const IIndexArr& newLocalToGlobal, std::unordered_map<IIndex, Image>& inserted);

	// Release without handing the pairs to the global scene: returns them (in global IDs), so blocks can
	// release in parallel and the pairs be appended in block order
	ImagePairArr Detach(IIndex block, const IIndexArr& images);

	// Every image of a block back to the pool, its pairs returned in global IDs: the block's model is
	// discarded (a block whose initialization failed), leaving the block empty
	ImagePairArr DetachAll(IIndex block);

	// the block's images without a pose, in global IDs (sorted)
	IIndexArr UnregisteredImages(IIndex block) const;

	// the local ID of a global image in its block
	IIndex LocalOf(IIndex block, IIndex globalID) const;

private:
	Scene& scene; // the global scene: every image's shell, the pool's features, the pairs no block holds
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_SCENEBLOCKS_H_
