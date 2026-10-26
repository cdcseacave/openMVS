/*
 * GlobalAlignment.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_GLOBALALIGNMENT_H_
#define _SFM_GLOBALALIGNMENT_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"
#include "Pose.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// forward declarations to avoid circular includes
class SFM_API Scene;

/*
 * Hierarchical SfM — Global Alignment and Merge
 * ===============================================
 *
 * After SceneCluster splits the scene, each sub-scene ("block" below) is reconstructed
 * independently and lives in its own arbitrary coordinate system. The merge places the blocks as
 * the view-graph filter places images: the blocks are the nodes of a small graph, every block pair
 * joined by cross pairs an edge holding the readings of its Sim(3), and the block poses are what
 * one robust averaging of the kept readings says, every reading the poses contradict cut. See
 * docs/design/HierarchicalSFM.md for the full design; this is the short version.
 *
 * Readings (EstimateSeamPair): every block pair joined by cross pairs is measured in whichever
 * mode GlobalAlignmentConfig::alignment selects. ALIGN_CAMERAS reads each direction once: one
 * block's cameras as a generalized rig against the other's inlier tracks (generalized PnP with
 * scale), refined over its inliers at the reprojection bar; a direction whose rig is one camera (a
 * door) is measured by that camera's own pose. The two directions stay two readings. ALIGN_POINTS
 * fits one 3D-3D similarity to the matches lying on an inlier track in both blocks. A reading is
 * scored on every correspondence of the pair -- the cameras that support it on its weaker side --
 * and read against the verified image pairs across it.
 *
 * Selection (SelectSeamGraph): every component of the kept readings (all of them at first) is
 * averaged (AverageBlockPoses: rotations by GlobalRotationEstimator, log-scales and translations
 * reweighted by the inverse of each reading's error); every reading is judged against the poses of
 * its component and kept within the graph bars; repeated, re-entry allowed, until the kept set settles. A bridge (a block
 * pair no cycle of the other block pairs tests) keeps a reading only with minSeamWeight cameras on
 * its weaker side and its verified pairs not against it. Each component of the readings left is a
 * model, its block poses the averaging's.
 *
 * Merge (MergeScenes): transform every block of the model holding the most images, average the
 * intrinsics of a camera shared across blocks, move keypoints, descriptors and pairs back from
 * every block (reversing SceneCluster::ExtractSubScene), and merge tracks with a union-find over
 * global feature IDs: every correspondence of every placed block pair that the final block poses
 * explain within the loose bar joins by construction, cross-block pairs join under a
 * duplicate-image and a 3D-proximity guard, and intra-block pairs are skipped -- their tracks were
 * already correctly formed. AdjustMergedScene (Scene.h) then filters the merged scene at that loose
 * bar and adjusts it, so a block bent within the bars keeps the matches of its far end until the
 * adjustment has straightened it. Every block outside the merged model is merged without a pose,
 * for the post-merge resection to recover.
 */

// One cross-block correspondence: a feature of image A matched to a feature of image B (global image IDs)
struct SFM_API SeamCorrespondence
{
	IIndex imageA, imageB;
	uint32_t featureA, featureB;
};

// One reprojection constraint of a seam: a 3D point of one block observed by a camera of the other
struct SFM_API SeamObservation
{
	Point3 X;            // the point, in the point block's local frame
	RMatrix R;           // observing camera rotation (world-to-camera), in its own block's local frame
	Point3 C;            // observing camera centre, in its own block's local frame
	Point3 bearing;      // observed unit bearing in the camera frame
	REAL pixelPerRadian; // converts the angular chord to the camera's pixels
	bool forward;        // true: point in A observed by a camera of B (p_B = T*p_A); false: the reverse
	IIndex rigImage;     // global ID of the observing camera
	IIndex pointImage;   // global ID of the camera whose feature the point came from
};

// What the evidence says about one transform: the inliers over the observations it was scored on,
// and per side (0 = block A, 1 = block B) the cameras that support it
struct SFM_API SeamScore
{
	unsigned inliers{0};
	unsigned support[2]{0, 0}; // cameras holding at least minCameraInliers inlier correspondences
	// the supporting cameras of the weaker side: a side one camera holds weighs one, however many
	// correspondences that camera carries
	unsigned Weight() const { return MINF(support[0], support[1]); }
};

// What the verified image pairs across a seam say about one of its readings: the inlier weight of
// the pairs it reproduces and of the ones it does not
struct SFM_API PairAgreement
{
	unsigned numPairs{0};
	float agree{0.f}, disagree{0.f};
	// Does the weight of the pairs the reading reproduces carry the weight of the ones it does not?
	// Nothing read, nothing held against it. Weight, not a count of pairs, so two thin pairs do not
	// outvote twenty-four heavy ones.
	bool Holds() const { return disagree <= agree; }
	// The share of the pairs' inlier weight the reading reproduces: 1 where nothing was read
	float Share() const { return agree + disagree > 0.f ? agree / (agree + disagree) : 1.f; }
};

// One reading of the Sim(3) between two blocks, with the evidence behind it
struct SFM_API SeamCandidate
{
	enum Source : uint8_t { RIG_B_ON_A, RIG_A_ON_B, POINTS };
	uint32_t sceneA{NO_ID}, sceneB{NO_ID}; // sceneA < sceneB
	Transform T;                           // p_B = T * p_A
	Source source{POINTS};
	bool scaleObservable{true};
	SeamScore score;                       // T scored on every correspondence of the pair, side 0 = A, side 1 = B
	// what the averaging weighs T by: for a reading of the camera alignment the supporting cameras of
	// its weaker side (at least one) times the share of the verified pairs' inlier weight it
	// reproduces; for the reading of the 3D-3D alignment the supporting cameras of its weaker side
	float weight{0.f};
	PairAgreement pairs;                   // the verified pairs across the seam, read against T
	// what the averaged poses of its component make of T: rotation (degrees), scale ratio >= 1 and
	// translation (a fraction of the footprint the two blocks share); an infinite rotation when the
	// averaging did not place both its blocks in one component
	float residualRotation{0.f}, residualScale{1.f}, residualTranslation{0.f};
};

// The camera footprint of a block in its own frame: the diagonal of the box its cameras span, the
// unit the translations of its seams are read in, and its cameras' centroid, the point they are read
// at -- a block's frame has an arbitrary origin, and a discrepancy read there would charge a
// rotation by the distance to it
struct SFM_API BlockFootprint
{
	REAL extent{0};
	Point3 centre{Point3::ZERO};
};

// Where one block sits in a model frame, and why it does not sit anywhere
struct SFM_API BlockPose
{
	enum State : uint8_t { UNPLACED, ADMITTED, UNPLACEABLE };
	Transform T;             // local -> model frame (valid when state == ADMITTED)
	uint32_t model{NO_ID};   // model index
	State state{UNPLACED};
	String reason;           // for the log, when not admitted
};

// What the merge did with the blocks it was given: how many were placed and where, which images
// were left unregistered, and the evidence the verdict rests on
struct SFM_API MergeReport
{
	unsigned numBlocks{0}, numPlaced{0}, numModels{0};
	unsigned numImagesPlaced{0}, numImagesUnplaced{0};
	IIndexArr unplacedImages;              // global IDs of the images of blocks that were not placed
	std::vector<SeamCandidate> candidates; // every reading of the block graph, with its residuals (moved out of the merge)
	std::vector<uint32_t> modelSeams;      // indices into candidates: the readings the merged model rests on
	std::vector<BlockPose> poses;          // final pose and state of every block
};

/**
 * @brief Configuration for global alignment
 */
struct SFM_API GlobalAlignmentConfig
{
	// How the relative Sim(3) of two sub-scenes is measured (see EstimateSeamPair):
	enum Alignment : unsigned {
		ALIGN_POINTS = 0,  // similarity from the 3D-3D correspondences of matches lying on an inlier track in BOTH sub-scenes
		ALIGN_CAMERAS = 1, // generalized-camera PnP with scale of one sub-scene's cameras against the other's tracks, both directions
	};
	unsigned alignment{ALIGN_CAMERAS};
	unsigned minCommonTracks{25};      // weighted inliers a cross pair links two blocks with, and inliers a reading needs
	bool mergeTrackInliersOnly{true};  // seed union-find with only inlier observations (true) or all observations (false)
	float maxReprojError{4.f};         // pixels; the camera alignment's reprojection threshold, the resection's own
	// A camera supports a reading with this many inlier correspondences, and a reading standing alone
	// in the block graph (a bridge) needs this many supporting cameras on its weaker side: a frame
	// misplaced inside its own block that carries a whole seam on its side, and a door of one camera,
	// weigh one, and stand only where a cycle corroborates them
	unsigned minCameraInliers{10};
	unsigned minSeamWeight{2};
	float minRigSpreadRatio{0.03f};    // rig spread over median depth for an observable scale
	// The verified image pairs across a seam are read against each of its readings: a pair of at
	// least minPairInliers weighted inliers agrees when the relative rotation and the baseline
	// direction the reading gives its two images are within maxPairRotationResidual of what it
	// measured. The share of the inlier weight that agrees weighs a reading of the camera alignment,
	// and a bridge keeps a reading only where that weight does not hold against it. 0 disables it.
	unsigned minPairInliers{30};
	float maxPairRotationResidual{5.f};     // degrees
	// The graph bars: a reading stays in the block graph while the averaged poses of its component
	// sit within these of it (a reading that observes no scale on its rotation alone). The right
	// readings of the two measured block sets reach 2.8 deg / 1.093 / 0.131 and 3.3 deg / 1.156 /
	// 0.205 at their 99th percentile, and the copies of a repeated structure sit past 0.27 of the
	// footprint
	float maxGraphRotationResidual{5.f};    // degrees
	float maxGraphScaleResidual{1.20f};
	float maxGraphTranslationResidual{0.20f}; // fraction of the pair's shared camera-bbox diagonal
	// constant: the loose bar, in times maxReprojError, the correspondences of the merged model's
	// block pairs join their tracks at and the merged scene is first filtered at: a model never closes
	// its cycles exactly, and a block bent within the graph bars is off by about this at its far end
	static constexpr float looseSeamFactor = 3.f;
	// 3D-3D estimator (ALIGN_POINTS):
	double simInlierThresholdFactor{0.01};  // RANSAC inlier distance as a fraction of the destination bbox diagonal
	double minSimInlierRatio{0.3};          // minimum RANSAC inlier ratio required to accept a sub-scene pair
	unsigned simRansacMaxIters{10000};      // RANSAC iteration budget; needed to find low-inlier-ratio models
};

class SFM_API GlobalAlignment
{
public:
	/**
	 * @brief Constructor - initializes with scene and config
	 * @param scene Global reference scene to align to (modified in-place)
	 * @param config Global alignment configuration
	 */
	GlobalAlignment(Scene& scene, const GlobalAlignmentConfig& config);

	/**
	 * @brief Merge the blocks into the global scene
	 *
	 * On return the scene holds the merged model — the poses of the placed blocks and the
	 * observations of every block, those of the unplaced ones without poses so the post-merge
	 * resection can recover them — and the report says what was left out and why.
	 * @param subScenes the blocks to place and merge (consumed)
	 * @param localToGlobals per block, its local image index -> global image ID
	 * @return true when at least one block was placed, which leaves the scene holding a
	 *         reconstruction whatever the seams said
	 */
	bool MergeScenes(
		std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		MergeReport& report);

	/**
	 * @brief Fill globalToLocal, blockPairLinks and blockPointMaps from the blocks
	 *
	 * EstimateSeamCandidates calls it first, and a caller reaching EstimateSeamPair or
	 * CollectPairObservations directly must call it first too.
	 */
	void PrepareSeamEvidence(
		const std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals);

	/**
	 * @brief The readings of every block pair joined by cross pairs (EstimateSeamPair)
	 */
	bool EstimateSeamCandidates(
		const std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		std::vector<SeamCandidate>& candidates);

	/**
	 * @brief The readings of one block pair (a < b), each scored on every correspondence of the
	 * pair and read against the verified pairs across it: one per direction measured (the camera
	 * alignment), or the one 3D-3D similarity (the point alignment); appends the ones that enter the
	 * block graph
	 */
	void EstimateSeamPair(
		const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
		std::vector<SeamCandidate>& candidates) const;

	/**
	 * @brief Both directions' correspondences of one block pair, collected without estimation;
	 * the observations' forward flags tell the two directions apart
	 */
	void CollectPairObservations(
		const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
		std::vector<SeamObservation>& observations,
		std::vector<SeamCorrespondence>& correspondences) const;

	/**
	 * @brief The one decision over the block graph: which readings stay, and where the blocks of
	 * every model sit
	 *
	 * Every component of the kept readings is averaged; every reading is judged against the poses
	 * of its component and kept within the graph bars; repeated, re-entry allowed, until the kept
	 * set settles (kRobustRounds at most). A bridge then keeps a reading only with minSeamWeight
	 * cameras on its weaker side and its verified pairs not against it, until none changes.
	 * @param footprints per block, its camera footprint in its own frame; its size is the number of blocks
	 * @param candidates in/out: every reading, the residuals the last averaging leaves it written in
	 * @param poses out: per block, state ADMITTED, its pose and model: the component of the readings
	 * left when the averaging placed it there, otherwise a model of its own at the identity, with
	 * the reason it has no other
	 * @param edgeSeams out: indices into candidates, the readings left
	 */
	void SelectSeamGraph(
		const std::vector<BlockFootprint>& footprints,
		std::vector<SeamCandidate>& candidates,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& edgeSeams) const;

private:
	/**
	 * @brief Robust rotation, scale and translation averaging of the given readings — the one call
	 * site of the three global solvers
	 *
	 * @param edges indices into candidates of the readings of ONE connected component; the blocks
	 * they touch are the nodes, and a reading weighs its candidate's weight. The rotations by
	 * GlobalRotationEstimator (its maximum spanning tree, L1, its own reweighting), run again over
	 * whatever a run leaves in another frame; the log-scales and the translations, between the
	 * blocks' camera centroids (BlockFootprint), so they do not depend on where the arbitrary origin
	 * of a block's frame lies, every reading reweighted by the inverse of its error as the
	 * estimators' row scale (their normal equations weigh its square, which settles on one reading
	 * rather than the L1 median). A reading past the rotation bar or of weight 0 takes no part in the
	 * scales, and one past the scale bar none in the translations
	 * @param fixedBlock the gauge: the frame every pose comes out in, when the readings reach it
	 * @param footprints per block, its camera footprint; its size is the number of blocks
	 * @param poses out: one entry per block; every block the rotation, scale and translation
	 * averaging reaches from the gauge gets its local -> gauge frame transform and model 0, the
	 * rest keep model NO_ID and state UNPLACED (all of them when the component's readings observe
	 * no scale at all)
	 */
	void AverageBlockPoses(
		const std::vector<SeamCandidate>& candidates,
		const std::vector<uint32_t>& edges,
		const std::vector<BlockFootprint>& footprints,
		uint32_t fixedBlock,
		std::vector<BlockPose>& poses) const;

	// The cross-block image pairs grouped by block pair, the pair ordered a < b
	void MapBlockPairLinks();

	/**
	 * @brief The score of T on the given observations: the inliers at the bar and, per side
	 * (sideOf(image) -> 0 or 1), the cameras holding minCameraInliers inlier correspondences
	 */
	void ScoreSeam(
		const std::vector<SeamObservation>& observations,
		const std::vector<SeamCorrespondence>& correspondences,
		const Transform& T,
		const std::function<int(IIndex)>& sideOf,
		SeamScore& score) const;

	/**
	 * @brief What the verified pairs between blocks a and b say of the reading T (A -> B): each
	 * pair's relative pose against the one T gives its two images (PoseLink.h
	 * MeasurePairDisagreement); nothing is read when maxPairRotationResidual is 0
	 */
	PairAgreement MeasureSeamPairs(const std::vector<Scene>& subScenes, uint32_t a, uint32_t b, const Transform& T) const;

	/**
	 * @brief Build and validate global image -> (sub-scene, local image) mapping
	 *
	 * Enforces one-to-one ownership: a global image can belong to at most one sub-scene.
	 */
	void BuildGlobalToLocalMap(const std::vector<IIndexArr>& localToGlobals);

	/**
	 * @brief Transform the blocks the merged model admitted by their poses and merge every block in
	 *
	 * A block the merged model admitted comes in with its poses and its 3D points; every other
	 * block comes in without them, so its images stay unregistered for the post-merge resection.
	 * @param seamInliers the correspondences of the merged model's block pairs its block poses
	 * explain, passed down to MergeTracksWithCrossSubScenePairs so their two ends join tracks by
	 * construction
	 */
	bool MergeTransformedScenes(
		std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		const std::vector<BlockPose>& poses,
		const std::vector<SeamCorrespondence>& seamInliers);

	/**
	 * @brief Merge a single scene into the global scene
	 *
	 * Moves keypoints/descriptors back from sub-scene images to the global scene
	 * (they were moved to sub-scenes during SceneCluster::ExtractSubScene to save memory).
	 * Also moves image pairs back and remaps track observation IDs.
	 * When not placed, camera poses are not copied and all merged tracks are marked
	 * non-inlier (numInliers=0): the observations survive for later triangulation, but no
	 * pose or 3D position from the unplaced sub-scene can influence the reconstruction.
	 */
	void MergeSingleScene(Scene& subScene, const IIndexArr& localToGlobal, bool placed);

	/**
	 * @brief Merge tracks from sub-scenes and connect them via cross-sub-scene pairs
	 *
	 * Uses a union-find over global feature IDs (same pattern as BuildTracks) to:
	 * 1. Initialize each sub-scene's tracks as independent sets
	 * 2. Union every correspondence of the merged model's block pairs its block poses explain:
	 *    its two ends join by construction, on the duplicate-image guard alone
	 * 3. Process cross-sub-scene pairs to connect tracks across boundaries,
	 *    using 3D proximity as validation when both sides have triangulated positions
	 * 4. Assemble final tracks, triangulating any new tracks without 3D positions
	 *
	 * Tracks of unplaced sub-scenes (all observations in unregistered images) are seeded with
	 * all their observations but no 3D position, so their structure survives for the
	 * post-merge resection to re-triangulate.
	 * @param unplacedImages per-global-image flags marking images of blocks that were not placed
	 * @param seamInliers the correspondences of the merged model's block pairs its block poses explain
	 */
	void MergeTracksWithCrossSubScenePairs(const std::vector<bool>& unplacedImages, const std::vector<SeamCorrespondence>& seamInliers);

	// Global image ID -> (sub-scene index, local image index)
	std::unordered_map<IIndex, std::pair<uint32_t, IIndex>> globalToLocal;

	// Block pair (a < b) -> indices into scene.pairs of its cross pairs
	std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> blockPairLinks;
	// Per block: (local image, feature) -> the index of the inlier track holding that observation,
	// what the correspondence collection looks each match up in
	std::vector<std::unordered_map<PairIdx, uint32_t>> blockPointMaps;

	Scene& scene; // Reference to input scene
	const GlobalAlignmentConfig& config; // Global alignment configuration
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBALALIGNMENT_H_
