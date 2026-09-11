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
 * Hierarchical SfM — Global Alignment and Merge Phase
 * ====================================================
 *
 * After SceneCluster splits the scene and each sub-scene is independently
 * reconstructed (tracks built, star-initialized, images resected, bundle-
 * adjusted), the sub-scenes live in their own arbitrary coordinate systems.
 * This file implements the MERGE phase: estimating the similarity transforms
 * that bring all sub-scenes into a single consistent coordinate system,
 * applying those transforms, and merging everything back into the original
 * global scene.
 *
 * ── Pipeline overview (merge) ──────────────────────────────────────────────
 *
 * STAGE 1: EVIDENCE (PrepareSeamEvidence)
 *   Which block every image belongs to, which image pairs cross a block boundary, and where
 *   each block triangulated the observations those pairs match — the raw material every later
 *   stage reads.
 *
 * STAGE 2: SEAM CANDIDATES (EstimateSeamCandidates)
 *   The relative Sim(3) of every adjacent block pair, measured in whichever mode
 *   GlobalAlignmentConfig::alignment selects. ALIGN_POINTS fits the similarity by RANSAC over
 *   the matches lying on an inlier track in BOTH blocks. ALIGN_CAMERAS treats one block's
 *   cameras as a rig and the other's inlier tracks as the points, so a match contributes when
 *   ONE endpoint lies on a track and a seam stays measurable from either side alone; both
 *   directions are estimated and both are scored on the union of their correspondences, so the
 *   two opinions answer to the same evidence. Every camera with enough correspondences votes,
 *   and a candidate must explain enough of the union, carry the votes of both sides and leave
 *   the two blocks' cameras unmixed. Directions that agree become one seam refined over their
 *   inliers; ones that disagree are settled by the camera votes, or both are kept for the graph.
 *
 * STAGE 3: SEAM GRAPH (ClassifySeamGraph)
 *   A seam is measured from two reconstructions that know nothing of each other, so a wrong one
 *   cannot be recognized on its own evidence: only the cycles it sits in can indict it. Every
 *   component is averaged robustly and each candidate keeps the residual of the consensus
 *   against it — ROBUST when the consensus confirms it, VERIFIED when only its own evidence
 *   stands behind it, UNDECIDED when the graph cannot tell, REJECTED when a stronger consistent
 *   path contradicts it.
 *
 * STAGE 4: INITIAL BLOCK POSES (ComputeInitialBlockPoses)
 *   The trusted seams alone carry the blocks: each of their components is averaged about its
 *   best connected block and becomes one model, numbered by the trusted weight it holds. These
 *   poses are a starting point, not a verdict.
 *
 * STAGE 5: PLACEMENT (PlaceBlocks, PlaceGroup, AdmitGroup)
 *   Each model is grown one block at a time, from the block the graph is most sure of. The next
 *   block tried is the one the admitted blocks support most, and PlaceGroup decides it: it pools
 *   every correspondence between the block and the model, estimates the pose from that pool both
 *   ways (the block's cameras on the model's points, the model's cameras on the block's points),
 *   scores each hypothesis over the whole pool and holds it to four gates — enough of the pool
 *   explained, the cameras of both sides behind it, the admitted neighbours' own seams agreeing
 *   with what it implies, and the two sides' cameras left unmixed. A block that passes is
 *   admitted, the seams that agree with its placement become model seams — a trusted one that
 *   disagrees comes in carrying its loop discrepancy — and every admitted block is then refined
 *   jointly over them; a block that fails is deferred and tried again once the model has grown.
 *   A block is a group of one, so the same routine places a whole model.
 *
 *   A block whose neighbours are behind it and whose cameras are unmixed, refused only by what it
 *   explains or by the votes, is the block a cycle runs through: it faces, alone, the error the
 *   model accumulated while it grew the long way round, and no single pose of it can satisfy both
 *   ends. Such a block is taken in on trust, the model is averaged over its seams so the cycle's
 *   discrepancy is spread over them, and the block is judged again at the pose that averaging gives
 *   it — against a model that has absorbed the loop. It is admitted only if it passes there, and
 *   the model is restored exactly if it does not.
 *
 * STAGE 6: THE MERGED MODEL (PlaceRemainingBlocks, SplitFoldedBlock, RevalidateBlocks)
 *   A block no model took in is a reconstruction of its own: the blocks left over form models on
 *   the trusted seams among them — a lone block is a model — and every model is then placed against
 *   the one carrying the most images, as a single group through the same routine that places a
 *   block. A model that goes in brings the seams it rested on with it, one similarity having moved
 *   all of its blocks at once.
 *
 *   A block whose own cameras split in two over a placement — some behind it, as many against it —
 *   is not a block that cannot be placed but two blocks the reconstruction folded into one, its two
 *   halves having never seen each other. Such a block is cut along the thin seam its own covisibility
 *   leaves between the two sides, its parts measure their seams like any other block, and each part
 *   is placed on its own.
 *
 *   Every block the model admitted is finally judged again at the pose it ended up with, against a
 *   model that has since grown around it: one its cameras then contradict is let go, and one whose
 *   cameras split is the fold that only one admitted neighbour hid.
 *
 *   The model holding the most images is the one merged with poses. The blocks of every other
 *   model, and those no model could place, are merged without poses so the post-merge resection
 *   re-registers their images against the consensus — the same process that would have placed
 *   them had the cluster boundary not severed their strongest pairs. The report names every one of
 *   them and why it stayed out.
 *
 * STAGE 7: MERGE TRANSFORMED SUB-SCENES
 *   Apply the pose of every placed block to its cameras and 3D points, then
 *   merge into the global scene:
 *
 *   a) Transform: apply Scene::Transform() to each sub-scene.
 *
 *   b) Intrinsics averaging: accumulate camera intrinsics (focal length,
 *      principal point, distortion coefficients) from all sub-scenes that
 *      share a global camera, then average. Uses the polymorphic
 *      Camera::AccumulateIntrinsics/ScaleIntrinsics interface so each camera
 *      type (PinholeCamera, SphericalCamera) handles its own parameters.
 *
 *   c) MergeSingleScene: for each sub-scene, move keypoints, descriptors,
 *      and image pairs back from the sub-scene to the global scene (reversing
 *      the moves done by SceneCluster::ExtractSubScene). Copy camera poses
 *      from sub-scene images to global images. Remap and append tracks.
 *
 *   d) MergeTracksWithCrossSubScenePairs: the critical step that creates
 *      cross-sub-scene track connectivity. Uses a union-find (disjoint set)
 *      over global feature IDs — the same data structure as BuildTracks:
 *
 *      Phase 1 — Initialize: seed the union-find with each sub-scene's
 *        track observations as pre-formed sets, storing the 3D position and
 *        inlier count at each set's root. By default only INLIER observations
 *        are included (config.mergeTrackInliersOnly = true); setting it to
 *        false includes all observations (outliers may add connectivity but
 *        also noise).
 *
 *      Phase 2 — Connect: iterate ONLY cross-sub-scene pairs — pairs whose
 *        two images belong to different sub-scenes, identified via the
 *        globalToLocal map. Intra-sub-scene pairs are deliberately skipped:
 *        their tracks were already correctly formed by BuildTracks during
 *        independent sub-scene reconstruction, and re-processing them here
 *        would over-merge tracks (outlier observations removed during
 *        reconstruction can lift the duplicate-image guard that originally
 *        kept them separate), bloating image sets and blocking legitimate
 *        cross-sub-scene connections.
 *        For each inlier match in a connecting pair, attempt to union the
 *        two features' sets. A guarded union rejects the merge if:
 *        - It would create duplicate observations (same image in one track),
 *          which would be geometrically invalid.
 *        - Both sides have triangulated 3D positions that are too far apart
 *          (exceeding a fraction of the scene bounding box diagonal), which
 *          indicates a false feature match.
 *        When the merge succeeds, the 3D positions are averaged weighted by
 *        inlier count, and the inlier count is accumulated.
 *
 *      Phase 3 — Assemble: iterate all features, group by union-find root,
 *        and construct the final track array. Tracks with pre-existing 3D
 *        positions (from merged sub-scene tracks) use the accumulated
 *        position and inlier count. New tracks (from cross-sub-scene pair
 *        features not in any original track) are triangulated via
 *        TriangulateSkewLLS; if triangulation fails, they are kept with
 *        numInliers=0 (excluded from BA until re-triangulation).
 *
 * ── Why this design ────────────────────────────────────────────────────────
 *
 * Averaging places every block at once and therefore believes every seam at once: one wrong
 * seam moves the blocks around it and the error is spread over the ones that were right. So the
 * averaged poses are only where the placement starts from, and each block is then admitted on
 * its own, against a model of blocks already admitted — evidence pooled over every seam to that
 * model, judged by the cameras of both sides, and answerable to the neighbours it closes a cycle
 * with. A block whose evidence does not carry it stays out and is merged without poses rather
 * than dragging the model with it.
 *
 * The averaging itself keeps the decoupled rotation → scale → translation order, because each
 * subproblem is then convex (or nearly so): rotation averaging on SO(3) has well-studied convex
 * relaxations, scale averaging in log-space is linear least-squares, and translation averaging
 * given rotations and scales is linear.
 *
 * The union-find track merging reuses the proven BuildTracks pattern but adds
 * 3D-aware guards: since sub-scene tracks already have triangulated positions,
 * the proximity test catches false feature matches that the standard duplicate-
 * image guard alone would miss (two tracks from non-overlapping sub-scenes can
 * never fail the duplicate-image test, so 3D proximity is the only defense).
 *
 * ── Memory protocol ────────────────────────────────────────────────────────
 *
 * MergeSingleScene reverses the moves done by SceneCluster::ExtractSubScene:
 * - Keypoints and descriptors are MOVED back from sub-scene images to global
 *   images (restoring the global scene's per-image feature data).
 * - Image pairs are MOVED back from sub-scenes to the global scene (restoring
 *   the full pair set, now with both intra-cluster and cross-cluster pairs).
 * - Colors (scene.colors) are released during track reassembly (Phase 3 of
 *   MergeTracksWithCrossSubScenePairs) since track indices change; they must
 *   be rebuilt downstream if needed.
 * - After all sub-scenes are merged, the sub-scene objects can be destroyed
 *   (their data has been moved out).
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

// The vote of one image on a candidate: its correspondences, the inliers among them, the share of
// the image grid its inlier keypoints occupy, and the verdict
struct SFM_API CameraVote
{
	IIndex image{NO_ID};
	unsigned correspondences{0}, inliers{0};
	float coverage{0.f};
	int8_t vote{0}; // +1 support, -1 contradiction, 0 abstain
};

// What the evidence says about one transform: the inlier mask over the observations it was scored
// on, the vote of every image with enough correspondences, the vote counts per side (0 = A, or the
// group being placed; 1 = B, or the model), the distinct supporting centres, and the share of the
// moving cameras whose nearest neighbour is one of their own after the transform. Filled by
// ScoreSeam only; held by every candidate and every placement hypothesis
struct SFM_API SeamScore
{
	unsigned inliers{0};
	unsigned support[2]{0, 0}, contra[2]{0, 0};
	unsigned centres{0};
	float ownNeighbourFraction{1.f};
	std::vector<bool> inlierMask;  // parallel to the scored observations
	std::vector<CameraVote> votes; // one per image with >= minVoteCorrespondences correspondences
	float Weight(float maxVoteWeight) const { return MINF(float(support[0] + support[1]), maxVoteWeight); }
};

// A measured Sim(3) between two blocks with the evidence that supports it
struct SFM_API SeamCandidate
{
	enum Source : uint8_t { RIG_B_ON_A, RIG_A_ON_B, UNION, POINTS };
	enum Class : uint8_t { UNCLASSIFIED, ROBUST, VERIFIED, UNDECIDED, REJECTED };
	uint32_t sceneA{NO_ID}, sceneB{NO_ID}; // sceneA < sceneB
	Transform T;                           // p_B = T * p_A
	Source source{UNION};
	bool oneDirection{false};              // the pair yielded only this direction
	bool scaleObservable{true};
	SeamScore score;                       // T scored on `observations`, side 0 = A, side 1 = B
	float weight{0.f};                     // score.Weight(maxVoteWeight); floor 1 for a weak closing seam
	Class cls{UNCLASSIFIED};
	float residualRotation{0.f}, residualScale{1.f}, residualTranslation{0.f};
	std::vector<SeamObservation> observations;       // all correspondences of the pair, both directions
	std::vector<SeamCorrespondence> correspondences; // parallel to observations
	unsigned NumInliers(int forward = -1) const;     // over all observations, or those with the given forward flag
	unsigned NumObservations(int forward = -1) const;
	bool IsTrusted() const { return cls == ROBUST || cls == VERIFIED; }
};

// Where one block sits in a model frame, and why it does not sit anywhere
struct SFM_API BlockPose
{
	enum State : uint8_t { UNPLACED, ADMITTED, DEFERRED, SPLIT, UNPLACEABLE };
	Transform T;             // local -> model frame (valid when state == ADMITTED)
	uint32_t model{NO_ID};   // model index
	State state{UNPLACED};
	String reason;           // for the log, when not admitted
};

// Blocks placed together: one block (its local frame is the group frame, frames = {identity}) or
// a model (each block's pose in that model's frame)
struct SFM_API BlockGroup
{
	std::vector<uint32_t> blocks;
	std::vector<Transform> frames; // block local -> group frame, parallel to blocks
};

// A hypothesis for the pose of a group in the model frame, scored on a pool of observations
struct SFM_API PlacementHypothesis
{
	enum Source : uint8_t { RIG_ON_MODEL, MODEL_ON_GROUP, INITIAL };
	Transform T;              // group frame -> model
	Source source{INITIAL};
	SeamScore score;          // side 0 = the group, side 1 = the model
	// the admitted neighbours that agree with what it implies, that disagree over a cycle their
	// own trusted seam closes, and that contradict it
	unsigned neighbourSupport{0}, neighbourLoop{0}, neighbourContra{0};
	String failedGate;        // the failed gates, comma-separated; empty when passed
	bool Passed() const { return failedGate.empty(); }
};

// Pool of observations between a group and the admitted blocks: the group side in the group frame,
// the model side in the model frame
struct SFM_API PlacementPool
{
	BlockGroup group;
	std::vector<uint32_t> candidateIdx;              // candidates contributing (group <-> admitted)
	std::vector<SeamObservation> observations;       // forward == true: point in the group, camera in the model; false: the reverse
	std::vector<SeamCorrespondence> correspondences; // parallel to observations
	std::vector<uint32_t> observationCandidate;      // parallel: index into candidateIdx (NO_ID for a raw pair without a candidate)
	std::vector<Point3> groupCentres;                // centres of every group camera in the group frame
	std::vector<Point3> modelCentres;                // centres of every admitted camera in the model frame
};

// What the merge did with the blocks it was given: how many were placed and where, which images
// were left unregistered, and the evidence the verdict rests on
struct SFM_API MergeReport
{
	unsigned numBlocks{0}, numPlaced{0}, numModels{0};
	unsigned numImagesPlaced{0}, numImagesUnplaced{0};
	unsigned numRobust{0}, numVerified{0}, numUndecided{0}, numRejected{0}; // candidate classes after the merge
	IIndexArr unplacedImages;              // global IDs of the images of blocks that were not placed
	std::vector<SeamCandidate> candidates; // every candidate with its final class (moved out of the merge)
	std::vector<uint32_t> modelSeams;      // indices into candidates: the seams admitted into the merged model
	std::vector<BlockPose> poses;          // final pose and state of every block
	bool camerasRelaxed{false};            // the camera-level relaxation ran
	float seamErrorBeforeRelax{0.f}, seamErrorAfterRelax{0.f}; // largest model-seam median error in pixels
};

/**
 * @brief Configuration for global alignment
 */
struct SFM_API GlobalAlignmentConfig
{
	// How the relative Sim(3) of two sub-scenes is measured (see EstimateSeamCandidates):
	enum Alignment : unsigned {
		ALIGN_POINTS = 0,  // similarity from the 3D-3D correspondences of matches lying on an inlier track in BOTH sub-scenes
		ALIGN_CAMERAS = 1, // generalized-camera PnP with scale of one sub-scene's cameras against the other's tracks, both directions
	};
	unsigned alignment{ALIGN_CAMERAS};
	unsigned minCommonTracks{25};      // minimum tracks to connect sub-scenes
	bool mergeTrackInliersOnly{true};  // seed union-find with only inlier observations (true) or all observations (false)
	float maxReprojError{4.f};         // pixels; the camera alignment's reprojection threshold, the resection's own
	// A camera votes on a seam candidate only with this many correspondences, supports it only with
	// this many inliers spread over this share of its image, and contradicts it when that many
	// correspondences leave almost no inlier. The three bars are read from the per-camera statistics
	// the merge logs, which is why every vote is written out at the ultimate verbosity level.
	unsigned minVoteCorrespondences{10};
	unsigned minVoteInliers{30};
	float minVoteCoverage{0.25f};
	float minCameraVoteRatio{2.f};          // supporters over contradictors per side
	float minCameraVoteRatioVerified{3.f};  // the same, when the model was built on verified seams only
	unsigned minSupportingCentres{3};       // distinct supporting rig centres, both sides summed
	float minCrossSupportRatio{0.5f};       // share of a side's own best candidate a candidate must explain
	float minRigSpreadRatio{0.03f};         // rig spread over median depth for an observable scale
	float minOwnNeighbourFraction{0.8f};    // a transform that interleaves the two blocks' cameras is vetoed
	// The scale and rotation limits gate the two directions of a seam against each other and the
	// neighbour check of a placement; the translation limit is the graph's own bar.
	float maxSimRotationError{3.f};         // degrees
	float maxSimScaleRatio{1.1f};
	float maxSimTranslationError{0.05f};    // fraction of the pair's shared camera-bbox diagonal
	float voteMargin{1.5f};                 // margin by which one candidate beats another on camera votes
	float maxVoteWeight{30.f};              // cap of a candidate's weight
	// Seam graph consensus: a candidate the averaged consensus contradicts by more than these is
	// inconsistent, and is rejected when a consistent alternative path at least this strong exists.
	float maxGraphRotationResidual{5.f};    // degrees
	float maxGraphScaleResidual{1.05f};
	float rejectWeightMargin{1.5f};
	float maxFoldCutRatio{0.2f};            // a block splits at a cut this thin relative to its own weight
	unsigned minFoldPartViews{10};          // neither part of a fold split may be smaller
	float relaxSeamResidualFactor{2.f};     // seam residual past this multiple of the bar relaxes the cameras
	float intraBlockEdgeWeight{10.f};       // weight of a block's internal edges during that relaxation
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
	 * @param subScenes the blocks to place and merge (consumed); a block cut along a fold appends
	 *        its two parts here, and is left holding nothing
	 * @param localToGlobals per block, its local image index -> global image ID; grows with subScenes
	 * @return true when at least one block was placed, which leaves the scene holding a
	 *         reconstruction whatever the seams said
	 */
	bool MergeScenes(
		std::vector<Scene>& subScenes,
		std::vector<IIndexArr>& localToGlobals,
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
	 * @brief Stage 2: measure every adjacent block pair in both directions, in whichever of the
	 * two modes GlobalAlignmentConfig::alignment selects; one or two candidates per pair. Scale
	 * is recovered directly by both modes, so no separate pairwise scale estimation is needed.
	 */
	bool EstimateSeamCandidates(
		const std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		std::vector<SeamCandidate>& candidates);

	/**
	 * @brief The routine of one block pair (a < b): both directions estimated, scored, gated and
	 * combined; appends 0, 1 or 2 candidates, and records the pair when the gates refuse both
	 */
	void EstimateSeamPair(
		const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
		std::vector<SeamCandidate>& candidates);

	/**
	 * @brief Both directions' correspondences of one block pair, collected without estimation
	 *
	 * The raw evidence of a pair, whether or not it has a candidate; the observations' forward
	 * flags tell the two directions apart.
	 */
	void CollectPairObservations(
		const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
		std::vector<SeamObservation>& observations,
		std::vector<SeamCorrespondence>& correspondences) const;

	/**
	 * @brief The scoring core, the one place camera votes and the interleaving fraction are computed
	 *
	 * Fills the inlier mask of `observations` under T (an observation's `forward` flag says which
	 * side holds the point; p_side1 = T * p_side0), the vote of every image with enough
	 * correspondences, the counts per side (sideOf(image) -> 0 or 1), the distinct supporting
	 * centres, and the share of `movingCentres`, mapped by T, whose nearest centre among the mapped
	 * moving centres and `fixedCentres` (side 1's frame) is a moving one.
	 */
	void ScoreSeam(
		const std::vector<Scene>& subScenes,
		const std::vector<SeamObservation>& observations,
		const std::vector<SeamCorrespondence>& correspondences,
		const Transform& T,
		const std::function<int(IIndex)>& sideOf,
		const std::vector<Point3>& movingCentres,
		const std::vector<Point3>& fixedCentres,
		SeamScore& score) const;

	/**
	 * @brief The gates on a score, all of them evaluated: union support (inliers >=
	 * minCommonTracks and, when bestOther > 0, inliersOther >= minCrossSupportRatio * bestOther),
	 * camera votes (support >= voteRatio * contra on each side, and centres >=
	 * minSupportingCentres) and interleaving (ownNeighbourFraction >= minOwnNeighbourFraction)
	 * @return the failed gates, comma-separated ("" when the score passed)
	 */
	String FailedGates(const SeamScore& score, unsigned inliersOther, unsigned bestOther, float voteRatio) const;

	/**
	 * @brief ScoreSeam of c.T on the candidate's own observations: sides A and B, block A's
	 * camera centres moved by T among block B's
	 */
	void ScoreCandidate(const std::vector<Scene>& subScenes, SeamCandidate& c) const;

	/**
	 * @brief Whether one direction of a scored candidate can observe a scale (forward = true: the
	 * rig of block B on the points of block A): spread of the supporting rig centres over the
	 * median inlier depth
	 */
	bool IsScaleObservable(const std::vector<Scene>& subScenes, const SeamCandidate& c, bool forward) const;

	/**
	 * @brief Give a seam the scale its own direction could not observe
	 *
	 * Only the scale changes: the rig centre and the point it already sat at across the seam stay
	 * paired, so the rig keeps its place and everything around it is rescaled about it.
	 * @param rigCentre centre of the rig whose direction cannot observe a scale, in its own block's frame
	 * @param rigIsB true when that rig is block B's, false when it is block A's
	 */
	static void RescaleSeamAboutRig(const Point3& rigCentre, bool rigIsB, REAL scale, Transform& T);

	/**
	 * @brief Stage 3: what the whole seam graph says about each of its candidates
	 *
	 * A seam is measured from two reconstructions that know nothing of each other, so a wrong one
	 * cannot be recognized on its own evidence; only the cycles it sits in can indict it. Every
	 * component of the graph is averaged robustly, each candidate keeps the residual of the
	 * consensus against it, and the class says how far it can be trusted: ROBUST when the consensus
	 * confirms it, VERIFIED when nothing corroborates it but its own evidence stands alone,
	 * UNDECIDED when the graph cannot tell, REJECTED when a stronger consistent path contradicts it.
	 * @param blockExtents blockExtents[i] = diagonal of block i's camera bounding box, in block i's
	 * own units; its size is the number of blocks
	 */
	void ClassifySeamGraph(const std::vector<REAL>& blockExtents, std::vector<SeamCandidate>& candidates) const;

	/**
	 * @brief Robust rotation, scale and translation averaging of the given seams — the one call site
	 * of the three global solvers
	 *
	 * The graph classes, the initial poses and the block pose graph all place their blocks through
	 * this one routine, so they all fit the same edges the same way.
	 * @param edges indices into candidates of the seams to average; the blocks they touch are the
	 * nodes, and an edge weighs its candidate's weight, halved when the candidate is only VERIFIED
	 * @param fixedBlock the gauge: the frame every pose comes out in, when the seams reach it
	 * @param poses out: numBlocks entries; every block the rotation, scale and translation
	 * consensus reaches from the gauge gets its local -> gauge frame transform and model 0, the
	 * rest keep model NO_ID and state UNPLACED
	 * @param residuals out: one per edge, the consensus against it (rotation degrees, scale ratio
	 * >= 1, translation as a fraction of the footprint the two blocks share); an edge the
	 * rotation averaging left with its two ends in different frames, and one measuring a scale
	 * between blocks the metric consensus did not both place, gets an infinite rotation. A seam
	 * that cannot observe a scale is judged on its rotation alone, and keeps a residual of 1 in
	 * scale and 0 in translation.
	 * @return false when no block could be placed, which a component whose seams observe no scale
	 * at all always reports even though its rotation residuals are filled in
	 */
	bool AverageBlockPoses(
		const std::vector<SeamCandidate>& candidates,
		const std::vector<uint32_t>& edges,
		const std::vector<REAL>& blockExtents,
		uint32_t numBlocks, uint32_t fixedBlock,
		std::vector<BlockPose>& poses,
		std::vector<Point3>& residuals) const;

	/**
	 * @brief Stage 4: the pose of every block of every trusted component, each component in its own
	 * model frame
	 *
	 * The trusted seams alone carry the blocks: each of their components is averaged about its
	 * best connected block and becomes one model, numbered by how much trusted weight it holds.
	 * Blocks no trusted seam reaches keep model NO_ID. The poses are only a starting point — the
	 * placement decides which of them are admitted.
	 */
	bool ComputeInitialBlockPoses(
		const std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t numBlocks,
		std::vector<BlockPose>& poses) const;

	/**
	 * @brief The one refinement of block poses: the admitted blocks fitted jointly to the inlier
	 * observations of the given model seams
	 *
	 * Seven parameters per block — a unit quaternion on its own manifold, a translation and a log
	 * scale — and one residual per inlier observation: the point is carried from its own block into
	 * the model frame and back into the block of the camera that saw it, and what that camera then
	 * predicts is charged against what it observed, as the chord between the two bearings in the
	 * camera's own pixels under a Huber loss of maxReprojError. Solving every block at once is what
	 * lets a seam's two ends move apart: a chain refined pair by pair can only pass its error on.
	 * @param modelSeams indices into candidates of the seams the model rests on; one whose two
	 * blocks are not both admitted into this model carries nothing and is skipped
	 * @param model only the blocks admitted into this model are fitted; another model's blocks
	 * are in another frame and have nothing to say here
	 * @param gaugeBlock the block held fixed, whose frame the poses are therefore expressed in
	 * @param poses in/out: the pose of every block, refined where the seams reach it
	 */
	void RefineBlockPoses(
		const std::vector<SeamCandidate>& candidates,
		const std::vector<uint32_t>& modelSeams,
		uint32_t model, uint32_t gaugeBlock,
		std::vector<BlockPose>& poses) const;

	/**
	 * @brief Every correspondence between a group of blocks and the admitted ones, in the two
	 * frames a placement is judged in
	 *
	 * One function for a block and for a model: a block is a group of one. Every candidate of a
	 * group-to-model pair contributes its evidence, whatever its class — a class discounts a
	 * candidate's transform, not what its cameras saw, and the pool is judged afresh; this is also
	 * what lets a folded block's two halves contradict each other. A pair no candidate covers at
	 * all still carries correspondences, and those are collected raw.
	 * @param model the model the group is being placed in: only the blocks admitted into it are
	 * the model side, another model's blocks sitting in a frame this one knows nothing about
	 */
	void BuildPlacementPool(
		const std::vector<Scene>& subScenes,
		const std::vector<SeamCandidate>& candidates,
		const std::vector<BlockPose>& poses,
		uint32_t model,
		const BlockGroup& group,
		PlacementPool& pool) const;

	/**
	 * @brief What the pool says about one placement: ScoreSeam of its transform over every pooled
	 * observation, then the gates on what came out
	 * @param bestOwnInliers what the best hypothesis of this group explains, so a placement that
	 * covers far less of the same evidence is refused
	 * @param voteRatio the camera-vote margin the model is held to
	 */
	void ScoreHypothesis(
		const std::vector<Scene>& subScenes,
		const PlacementPool& pool,
		unsigned bestOwnInliers,
		float voteRatio,
		PlacementHypothesis& h) const;

	/**
	 * @brief Stage 5: admit the blocks of one model, one at a time, against the ones already in
	 *
	 * The model starts at the block the seam graph is most sure of and grows by the block the
	 * admitted ones support most. A block that cannot be placed is deferred and tried again as
	 * soon as the model has changed, since what the model could not confirm then it may now.
	 * A block whose own cameras split over its best placement is cut along the fold instead of being
	 * deferred, which appends its two parts to the blocks and queues them here.
	 * @param model the blocks with poses[b].model == model are the ones this run may admit; a model
	 * that already holds blocks keeps them and the seams it rests on, and grows from there
	 * @param poses in/out: the stage 4 poses coming in, the admitted blocks' own frames going out
	 * @param modelSeams out: the seams the model rests on, indices into candidates
	 * @return the number of blocks the model holds, at least one whenever it holds a block
	 */
	unsigned PlaceBlocks(
		std::vector<Scene>& subScenes,
		std::vector<IIndexArr>& localToGlobals,
		std::vector<SeamCandidate>& candidates,
		std::vector<REAL>& blockExtents,
		uint32_t model,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& modelSeams);

	/**
	 * @brief Stage 6: the blocks no model took in, and the models themselves, against the largest
	 *
	 * What every model's own placement left over is a reconstruction of its own: the trusted seams
	 * among the blocks outside every model carry them into models of their own — a block none of
	 * them reaches is a model of one — and each of those grows like any other. Every model is then
	 * placed against the one carrying the most images, largest first, as a single group: the blocks
	 * of a model are rigid against each other, so the pose that carries one of them carries all.
	 * A model that cannot be placed keeps its own frame, and its blocks are left to the resection.
	 * @param poses in/out: the blocks of the merged model admitted into it, the rest unplaceable
	 * @param modelSeams in/out: per model, the seams it rests on; a model that goes in hands its
	 * own over to the model it was placed on
	 */
	void PlaceRemainingBlocks(
		std::vector<Scene>& subScenes,
		std::vector<IIndexArr>& localToGlobals,
		std::vector<SeamCandidate>& candidates,
		std::vector<REAL>& blockExtents,
		std::vector<BlockPose>& poses,
		std::vector<std::vector<uint32_t>>& modelSeams);

private:
	/**
	 * @brief One model placed on another, as a single group
	 *
	 * The blocks of `src` with their poses in its frame are the group, so both models' cameras vote
	 * on the one similarity that carries all of them, and the admission re-bases every block of
	 * `src` and promotes the seams that agree. No placement code of its own: the same pool, the same
	 * hypotheses, the same gates and the same admission a single block answers to.
	 * @param modelSeams in/out: the seams `dst` rests on, the seams `src` rested on among them
	 * @param reason out: why the group could not be placed, empty on success
	 */
	bool PlaceModel(
		const std::vector<Scene>& subScenes,
		std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t src, uint32_t dst,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& modelSeams,
		String& reason) const;

	/**
	 * @brief The fold test: a block whose own cameras split over a placement cut in two
	 *
	 * Cameras that saw nothing of each other can be reconstructed into one block at any pose
	 * relative to one another, and no single placement of such a block can answer to both halves.
	 * The two sides of the vote are the two halves, the cameras that could not vote go with the side
	 * their own covisibility ties them to, and the block comes apart only where its own pairs are
	 * already almost cut: each side connected on its own, the cut between them thin against what
	 * holds either side together, and neither side smaller than a block needs to be. The parts are
	 * appended to the blocks, their pairs measured like any other block's, and the seams of the
	 * block they came from are disowned.
	 * @param best the hypothesis whose votes split, the one the placement refused
	 * @param parts out: the indices of the two new blocks
	 * @return true when the block was cut, which leaves it holding nothing
	 */
	bool SplitFoldedBlock(
		std::vector<Scene>& subScenes,
		std::vector<IIndexArr>& localToGlobals,
		std::vector<REAL>& blockExtents,
		std::vector<SeamCandidate>& candidates,
		uint32_t block,
		const PlacementHypothesis& best,
		std::pair<uint32_t, uint32_t>& parts);

	/**
	 * @brief Stage 6: every block of a model judged again, at the pose the model left it at
	 *
	 * A block is admitted against the model as it stood then; the model has grown since, and what
	 * it has taken in since may contradict it. Each admitted block is therefore read once more
	 * against all the others, at its own pose: one whose cameras split is the fold that a single
	 * admitted neighbour hid at the time, and is cut; one the cameras contradict is let go, with the
	 * seams the model rested on through it.
	 * @return the number of blocks let go or cut, which is what tells the caller to grow the model
	 * once more over their parts
	 */
	unsigned RevalidateBlocks(
		std::vector<Scene>& subScenes,
		std::vector<IIndexArr>& localToGlobals,
		std::vector<REAL>& blockExtents,
		std::vector<SeamCandidate>& candidates,
		uint32_t model,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& modelSeams);

	/**
	 * @brief The trusted components the given blocks span, each averaged into a model of its own
	 *
	 * The one way a set of blocks becomes models: the components of the trusted seams among them,
	 * each averaged about the block carrying most of their weight and numbered from `firstModel` in
	 * component order. The stage 4 poses and the models the leftover blocks form both come out of it.
	 * @param eligible per block, whether it may take part; a seam with an end outside is not a seam
	 * of this graph
	 * @param poses in/out: the blocks of every component get their pose and their model
	 * @return the number of components, i.e. of model numbers consumed
	 */
	unsigned AverageTrustedComponents(
		const std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		const std::vector<bool>& eligible,
		uint32_t firstModel,
		std::vector<BlockPose>& poses) const;

	/**
	 * @brief The evidence of the blocks appended past `firstBlock`
	 *
	 * What PrepareSeamEvidence builds, for blocks that appeared after it ran: the images they took
	 * over answer to them now, they get the point map of what they triangulated, and every cross
	 * pair is read again so a pair that ran to the block they were cut from runs to the part that
	 * holds its images.
	 */
	void ExtendSeamEvidence(
		const std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		uint32_t firstBlock);

	// The cross-block image pairs grouped by block pair, the pair ordered a < b, as globalToLocal
	// currently reads them
	void MapBlockPairLinks();

	/**
	 * @brief The one placement routine: one attempt to place `group` against the admitted blocks
	 * of `model`
	 *
	 * Every correspondence between the two sides is pooled, the pose is estimated from that pool
	 * in both directions and taken from stage 4 as a third opinion, and each hypothesis answers
	 * to the same four gates: how much of the pool it explains, what the cameras of both sides
	 * vote, whether the admitted neighbours' own seams agree with what it implies, and whether it
	 * leaves the two sides' cameras unmixed. A block is a group of one, and a whole model is a
	 * group too.
	 * @param winner out: the hypothesis that won, or the best one when none did
	 * @param reason out: why nothing could be placed, empty on success
	 * @return true when a hypothesis carried the group
	 */
	bool PlaceGroup(
		const std::vector<Scene>& subScenes,
		std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t model,
		const BlockGroup& group,
		const std::vector<BlockPose>& poses,
		PlacementHypothesis& winner,
		String& reason) const;

	/**
	 * @brief Take the group into the model at the winning hypothesis
	 *
	 * Every block of the group is moved to where the hypothesis puts it, and every candidate
	 * between the group and the admitted blocks is held against what the placement implies: one
	 * that agrees becomes a seam the model rests on — an undecided candidate is verified by it —
	 * and a trusted one that disagrees becomes a model seam carrying its loop discrepancy, so the
	 * refinement answers for it. An undecided candidate that disagrees is left where it was.
	 * @return true when a block of the group now has two or more admitted neighbours joined to it
	 * by model seams, i.e. the admission closed a cycle
	 */
	bool AdmitGroup(
		const std::vector<Scene>& subScenes,
		std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t model,
		const BlockGroup& group,
		const PlacementHypothesis& winner,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& modelSeams) const;

	/**
	 * @brief The blocks of one model where the consensus of the given seams puts them
	 *
	 * The averaging alone, gauged at the seed and anchored back at the pose the model already has
	 * there, so the model keeps its frame and a block the consensus could not reach stays in the
	 * same frame as the blocks it moved. The one step both the relaxation and the cycle closure
	 * take.
	 * @param seams the seams to average over, indices into candidates
	 * @param poses in/out: the blocks admitted into this model that the consensus reaches
	 * @param residuals out: what the consensus makes of each of those seams, parallel to `seams`
	 * @return false when the consensus placed nothing, or could not place the seed to anchor it,
	 * which leaves every pose as it was
	 */
	bool AverageModelPoses(
		const std::vector<SeamCandidate>& candidates,
		const std::vector<uint32_t>& seams,
		const std::vector<REAL>& blockExtents,
		uint32_t model, uint32_t seed,
		std::vector<BlockPose>& poses,
		std::vector<Point3>& residuals) const;

	/**
	 * @brief The one loop closure at admission: a block the votes refuse, judged again once the
	 * cycle it closes has been taken into the model
	 *
	 * A group whose admitted neighbours are behind it and whose cameras the model leaves unmixed,
	 * refused only by what it explains or by the cameras, and joined to two or more admitted blocks
	 * by trusted seams, is the group a cycle runs through: it faces the error the model accumulated
	 * growing the long way round, and no pose of it can answer to both ends at once. It is taken in
	 * at its best hypothesis, the model is averaged over its seams and the group's own — which
	 * spreads that error over the cycle and moves the group to where its seams, not its hypothesis,
	 * put it — and the pool is read again there and held to the same gates. What the cameras then
	 * say is what they make of a model that has absorbed the loop.
	 * @param best in: the hypothesis the placement refused; out: the hypothesis the relaxed model
	 * carried, when it did
	 * @param poses, modelSeams in/out: left exactly as they were unless the group is admitted
	 * @param closedCycle out: what the admission made of the group's seams, as `AdmitGroup` reports it
	 * @param reason out: the gates the relaxed hypothesis failed, when it was refused
	 * @return true when the relaxed model carried the group, which `AdmitGroup` has then taken in
	 */
	bool CloseCycleThrough(
		const std::vector<Scene>& subScenes,
		std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t model, uint32_t seed,
		const BlockGroup& group,
		PlacementHypothesis& best,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& modelSeams,
		bool& closedCycle,
		String& reason) const;

	/**
	 * @brief The blocks of one model averaged over the seams it rests on, then refined jointly
	 *
	 * A model grown one block at a time carries the error of every seam it grew through, and that
	 * error ends at the seam closing the cycle. Averaging the admitted blocks over the model's own
	 * seams spreads it over the whole cycle, and the joint refinement then reads the observations
	 * behind those seams again from where the consensus put the blocks.
	 * @param seed the gauge: the block whose frame the model is expressed in
	 * @param modelSeams the seams the model rests on, indices into candidates
	 * @param poses in/out: the admitted blocks of this model, moved to where the consensus puts them
	 * @return the largest rotation discrepancy (degrees) the model's seams carried before, and carry
	 * after
	 */
	std::pair<float, float> RelaxBlockPoses(
		const std::vector<Scene>& subScenes,
		std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t model, uint32_t seed,
		const std::vector<uint32_t>& modelSeams,
		std::vector<BlockPose>& poses) const;

	/**
	 * @brief The seams the pairs of a model could not measure on their own but the model predicts
	 *
	 * Every pair of admitted blocks that carries correspondences but no seam the model rests on is
	 * read again from where the model puts its two blocks: a pair too thin for its own cameras to
	 * vote on is still evidence once something else says where it should lie, and the seam it yields
	 * closes the cycle its blocks sit in. It enters the model at the weight its cameras carry,
	 * floored so that a seam no camera could vote on still holds the two blocks together.
	 * @param modelSeams in/out: the verified seams are appended to the ones the model rests on
	 * @return how many seams were added
	 */
	unsigned VerifyWeakSeams(
		const std::vector<Scene>& subScenes,
		std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t model,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& modelSeams) const;

	/**
	 * @brief The one way a predicted transform becomes a candidate: the pair's own correspondences
	 * read under it
	 *
	 * No estimator runs here — the prediction is the hypothesis. The pair's raw correspondences are
	 * collected, those it explains at kLooseSeamFactor times the reprojection bar are what it is
	 * refined over, and the refined seam is then scored at that bar like any other. It answers to
	 * the union support and the interleaving veto, but not to its own cameras' votes: what stands
	 * behind it is the consistency of the model that predicted it, not the pair's ability to measure
	 * itself.
	 * @param T the predicted similarity, A -> B
	 * @param c out: the candidate, filled only when it is returned true
	 * @return true when the pair still explains enough of itself under the prediction
	 */
	bool CandidateFromPrediction(
		const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
		const Transform& T, SeamCandidate& c) const;

	/**
	 * @brief The tail of a placement: the model relaxed over its seams, the pairs it can now predict
	 * verified, and the model relaxed again when one of them became a seam
	 */
	void CloseModel(
		const std::vector<Scene>& subScenes,
		std::vector<SeamCandidate>& candidates,
		const std::vector<REAL>& blockExtents,
		uint32_t model, uint32_t seed,
		std::vector<BlockPose>& poses,
		std::vector<uint32_t>& modelSeams) const;

	/**
	 * @brief Refine one A -> B similarity against every reprojection the given observations carry
	 *
	 * The two-block case of RefineBlockPoses: block A gauges the model at the identity, so the
	 * seam travels in and out through block B's pose.
	 */
	void RefineSeamTransform(
		const std::vector<SeamObservation>& observations, float maxReprojError, Transform& T) const;

	/**
	 * @brief Build and validate global image -> (sub-scene, local image) mapping
	 *
	 * Enforces one-to-one ownership: a global image can belong to at most one sub-scene.
	 */
	void BuildGlobalToLocalMap(const std::vector<IIndexArr>& localToGlobals);

	/**
	 * @brief The routine of one block pair in point mode: the single candidate the 3D-3D
	 * similarity of the matches lying on an inlier track in both blocks yields, scored on the
	 * same correspondences the camera alignment collects so the votes mean the same thing
	 * @param pairIndices indices into scene.pairs of the pair's cross pairs
	 */
	void EstimatePointSeamPair(
		const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
		const std::vector<uint32_t>& pairIndices,
		std::vector<SeamCandidate>& candidates);

	/**
	 * @brief Stage 7: transform the placed blocks by their poses and merge every block in
	 *
	 * A block the placement admitted into the merged model comes in with its poses and its 3D
	 * points; every other block comes in without them, so its images stay unregistered for the
	 * post-merge resection to recover.
	 */
	bool MergeTransformedScenes(
		std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		const std::vector<BlockPose>& poses);

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
	 * 2. Process cross-sub-scene pairs to connect tracks across boundaries,
	 *    using 3D proximity as validation when both sides have triangulated positions
	 * 3. Assemble final tracks, triangulating any new tracks without 3D positions
	 *
	 * Tracks of unplaced sub-scenes (all observations in unregistered images) are seeded with
	 * all their observations but no 3D position, so their structure survives for the
	 * post-merge resection to re-triangulate.
	 * @param unplacedImages per-global-image flags marking images of blocks that were not placed
	 */
	void MergeTracksWithCrossSubScenePairs(const std::vector<bool>& unplacedImages);

	// Global image ID -> (sub-scene index, local image index)
	std::unordered_map<IIndex, std::pair<uint32_t, IIndex>> globalToLocal;

	// Block pair (a < b) -> indices into scene.pairs of its cross pairs
	std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> blockPairLinks;
	// The block pairs whose seam was measured and refused by the gates: their correspondences are
	// evidence that has already been judged, so a placement must not take them for raw evidence
	// and re-litigate a verdict the whole pair was weighed for
	std::set<std::pair<uint32_t, uint32_t>> refusedSeamPairs;
	// Per block: (local image, feature) -> the position of the inlier track holding that
	// observation, what the correspondence collection looks each match up in
	std::vector<std::unordered_map<PairIdx, Point3>> blockPointMaps;

	Scene& scene; // Reference to input scene
	const GlobalAlignmentConfig& config; // Global alignment configuration
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBALALIGNMENT_H_
