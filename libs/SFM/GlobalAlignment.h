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
 * STAGE 1: ESTIMATE RELATIVE SIMILARITIES
 *   For every pair of sub-scenes that share images connected by cross-sub-scene
 *   pairs (pairs left in the global scene after splitting), estimate a 7-DOF
 *   similarity transform (Sim(3): rotation, translation, scale) and store it as
 *   a ScenePair with its inlier count. Both modes start from the same per-sub-scene
 *   observation cache mapping (localImage, feature) to the 3D position of the inlier
 *   track holding that observation, and both walk the same cross-sub-scene matches;
 *   they differ in what they ask of a match. GlobalAlignmentConfig::alignment selects.
 *
 *   ALIGN_POINTS — similarity from 3D-3D correspondences:
 *   - A match contributes when BOTH its endpoints hit the caches, giving one 3D point
 *     per sub-scene, each in its own local frame.
 *   - EstimateSimilarityTransform fits the Sim(3) by RANSAC, with the inlier distance
 *     a fraction of the destination point cloud's bounding-box diagonal so the
 *     criterion is invariant to each sub-scene's arbitrary units.
 *   - This needs a match whose two endpoints both lie on a track. Dense (warp-sampled)
 *     keypoints are laid out per pair on the target side, so they rarely join a track
 *     there and such a seam can end up with no correspondence at all.
 *
 *   ALIGN_CAMERAS — generalized-camera PnP with scale:
 *   - One sub-scene's cameras are a rig whose internal poses are known in that
 *     sub-scene's frame and at its scale; the other sub-scene's inlier tracks are the
 *     3D points; the cross-sub-scene matches are the rig's observations of them. A
 *     match contributes when ONE endpoint hits the cache — the other endpoint only has
 *     to be a keypoint — so a seam is measurable from either side alone.
 *   - PoseLib's generalized absolute pose with scale (LO-RANSAC over gp4ps, then a
 *     scale-aware refinement) solves the rig pose and the rig-to-points scale together,
 *     from bearing vectors, so any central camera model is handled.
 *   - Both directions are estimated and both are scored on the union of the two directions'
 *     correspondences, so the two opinions are compared on the same evidence. Every camera of
 *     either sub-scene with enough correspondences then votes on each candidate — support,
 *     contradiction or abstention, from its inlier share and how far its inliers spread over
 *     its image — and a candidate must carry the votes of both sides, explain what the other
 *     direction saw, and leave the two sub-scenes' cameras unmixed. Survivors that agree in
 *     rotation and scale become one seam refined over the union of their inliers; survivors
 *     that disagree are settled by the camera votes when one carries clearly more of them, and
 *     otherwise both are kept for the seam graph to decide. A direction whose rig is too
 *     shallow to observe a scale takes the other direction's.
 *   Pairs with too few inliers or low inlier ratio are discarded, in both modes.
 *
 * STAGE 2: ROTATION AVERAGING
 *   Extract relative rotations R_ij from each ScenePair and solve for global
 *   rotations R_i using GlobalRotationEstimator (L1-ADMM initialization
 *   followed by IRLS refinement). The rotations are represented as angle-axis
 *   vectors in so(3) and solved via a sparse linear system. This decouples
 *   rotation from scale and translation, which is standard practice because
 *   SO(3) averaging is better conditioned than joint Sim(3) estimation.
 *
 * STAGE 3: SCALE AVERAGING
 *   Pairwise scale ratios come directly from each ScenePair's relativeTransform
 *   (no more median-depth computation). Solve the overdetermined system
 *   log(s_j) - log(s_i) = log(s_ij) via least-squares in log-space using
 *   GlobalScaleEstimator. Working in log-space converts the multiplicative
 *   scale group (R+) into an additive linear problem. The gauge freedom is
 *   fixed by setting the first sub-scene's scale to 1.0.
 *
 * STAGE 4: TRANSLATION AVERAGING
 *   For each ScenePair, rotate and scale the relative translation t_ij by the
 *   corresponding global rotation and scale to align it to the global frame.
 *   Solve the linear system t_j - t_i = t_ij for all pairs via least-squares
 *   using GlobalTranslationEstimator. The gauge freedom is fixed by pinning
 *   the best-connected sub-scene at the origin.
 *
 * VALIDATION (between stages 4 and 5)
 *   Each sub-scene pair's measured Sim(3) is composed with the averaged global transforms
 *   of its two end-points; the residual is identity when the edge agrees with the
 *   consensus. The seam the consensus contradicts most is dropped and the averaging redone,
 *   until no residual is past its limit; dropping a seam that bridges the graph also demotes
 *   the smaller side, which has then lost its only link. A sub-scene still dominated by
 *   conflicting incident edge weight is demoted as a last resort: it is merged without its
 *   poses so the post-merge resection re-registers its images against the trusted consensus.
 *   The remaining sub-scenes are then re-averaged and re-validated until the verdict is stable.
 *   A seam graph with no cycle has nothing to validate: the averaging fits every edge exactly.
 *
 *   This validates the sub-scenes against EACH OTHER; whether a single sub-scene is itself
 *   internally sound is not re-litigated here. A cluster holding two blocks joined by a
 *   seam too sparse to observe their relative scale reconstructs at two scales, and that is
 *   a clustering fault: SceneCluster refuses such an interface when merging clusters
 *   (ClusterConfig::minClusterCoupling), splits any cluster that ends up with one anyway
 *   (RefineClustersSplitThinWaist), and reports the spectral-cut coupling of every finished
 *   cluster. If the defect ever reappears, that is where it is detected and fixed — the
 *   merge stage must not compensate for it. Comparing each sub-scene's own two-view
 *   geometry against its own poses was tried here and abandoned: on a 7-scene benchmark it
 *   flagged every scene (11-72% violated pair weight), including ones that registered every
 *   image, because two-view relative poses are unreliable on the low-parallax and
 *   homography-degenerate pairs such captures are full of.
 *
 * STAGE 5: MERGE TRANSFORMED SUB-SCENES
 *   Apply the estimated similarity transforms (s_i * R_i, t_i) to each
 *   sub-scene's cameras and 3D points, then merge into the global scene:
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
 * The decoupled rotation → scale → translation estimation is more robust than
 * joint Sim(3) averaging because each subproblem is convex (or nearly so):
 * - Rotation averaging on SO(3) has well-studied convex relaxations (Weiszfeld
 *   on the angular manifold).
 * - Scale averaging in log-space is a linear least-squares problem.
 * - Translation averaging given known rotations and scales is linear.
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

/**
 * @brief Scene pair connection with relative 7-DOF similarity transform
 *
 * relativeTransform maps points from sub-scene A's local frame to sub-scene B's:
 *     p_B = relativeTransform * p_A = scale * R * p_A + t
 * Its scale field is therefore s_A / s_B (source scale divided by destination scale).
 */
struct SFM_API ScenePair
{
	uint32_t sceneA;              // First sub-scene index
	uint32_t sceneB;              // Second sub-scene index
	Transform relativeTransform;  // 7-DOF Sim(3) mapping p_A -> p_B
	unsigned numInliers;          // RANSAC inlier count (used as averaging weight)

	ScenePair() : sceneA(NO_ID), sceneB(NO_ID), numInliers(0) {}
};

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
	float maxSimTranslationError{0.05f};    // fraction of the smaller block's local camera-bbox diagonal
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
	 * @brief Align and merge sub-scenes into the global scene
	 * @param subScenes Vector of sub-scenes to align and merge (modified in-place)
	 * @param localToGlobals Vector of ID mappings from sub-scenes to global scene (parallel to subScenes)
	 * @return true if all sub-scenes were aligned and merged; false if alignment could not
	 *         complete, in which case the global scene is populated with the largest intact
	 *         sub-scene so a good partial reconstruction is never discarded (never left empty)
	 *
	 * Combines all sub-scenes, handling duplicate cameras/points.
	 */
	bool MergeScenes(std::vector<Scene>& subScenes, const std::vector<IIndexArr>& localToGlobals);

	/**
	 * @brief Measure the relative similarity of every connected sub-scene pair, exactly as
	 * MergeScenes does before averaging, without merging anything
	 *
	 * The first stage on its own: it neither consumes the sub-scenes nor touches the global
	 * scene, so the alignment can be measured against a known answer.
	 */
	bool EstimateSubScenePairs(
		const std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		std::vector<ScenePair>& scenePairs);

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
	 * @brief Stage 1: measure every adjacent block pair in both directions, in whichever of the
	 * two modes GlobalAlignmentConfig::alignment selects; one or two candidates per pair. Scale
	 * is recovered directly by both modes, so no separate pairwise scale estimation is needed.
	 */
	bool EstimateSeamCandidates(
		const std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		std::vector<SeamCandidate>& candidates);

	/**
	 * @brief The routine of one block pair (a < b): both directions estimated, scored, gated and
	 * combined; appends 0, 1 or 2 candidates
	 */
	void EstimateSeamPair(
		const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
		std::vector<SeamCandidate>& candidates) const;

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

private:
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
		std::vector<SeamCandidate>& candidates) const;

	/**
	 * @brief One ScenePair per candidate its pair verdict settled: an agreement, a winner on the
	 * camera votes, or a lone survivor. A pair still holding two candidates emits none, so the
	 * averaging below sees it exactly as it sees a pair whose seam was refused.
	 */
	static void CandidatesToScenePairs(
		const std::vector<SeamCandidate>& candidates,
		std::vector<ScenePair>& scenePairs);

	/**
	 * @brief Stage 2: Estimate global rotations from pairwise rotations.
	 * Robustly rejects rotation-inconsistent pairs and PRUNES them from scenePairs in place, so
	 * the downstream scale/translation averaging only use rotation-consistent links. Sub-scenes
	 * the estimator cannot place are left with an INF rotation (caller selects by finiteness).
	 * @param scenePairs in/out: pruned to the rotation-consistent subset.
	 */
	bool EstimateGlobalRotations(
		std::vector<ScenePair>& scenePairs,
		const uint32_t numSubScenes,
		std::vector<Point3d>& globalRotations);

	/**
	 * @brief Stage 3: Estimate global scales from pairwise scale ratios
	 * extracted from each ScenePair::relativeTransform.
	 */
	bool EstimateGlobalScales(
		const std::vector<ScenePair>& scenePairs,
		const uint32_t numSubScenes,
		std::vector<REAL>& globalScales);

	/**
	 * @brief Stage 4: Estimate global translations from pairwise translations
	 */
	bool EstimateGlobalTranslations(
		const std::vector<ScenePair>& scenePairs,
		const std::vector<Point3d>& globalRotations,
		const std::vector<REAL>& globalScales,
		const uint32_t numSubScenes,
		std::vector<Point3>& globalTranslations);

	/**
	 * @brief Stage 5: Merge transformed sub-scenes into global scene
	 * @param demoted per-sub-scene flags (from ValidateAlignment): merge without poses when true
	 */
	bool MergeTransformedScenes(
		std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		const std::vector<Point3d>& globalRotations,
		const std::vector<REAL>& globalScales,
		const std::vector<Point3>& globalTranslations,
		const std::vector<bool>& demoted);

	/**
	 * @brief Drop the seams the averaged consensus contradicts, re-averaging after each one
	 *
	 * A seam is measured from two reconstructions that know nothing of each other, so a wrong one
	 * cannot be recognized on its own evidence; only the cycles of the seam graph can indict it.
	 * Each round scores every surviving seam against the averaged global transforms, takes the one
	 * whose residual exceeds its limit by the largest factor and drops it, then re-averages scale
	 * and translation over what is left (rotation averaging is already robust, so its result is
	 * kept). A seam whose removal disconnects the graph is dropped too, but its smaller side has
	 * then lost its only link to the consensus and is demoted to be rebuilt by resection. The loop
	 * stops when no seam is past its limit, and cannot run longer than there are seams.
	 *
	 * A seam graph with no cycle carries no such evidence at all: the averaging reproduces every
	 * edge exactly and every residual is zero, whatever the seams claim.
	 *
	 * @param scenePairs in/out: pruned to the seams the consensus does not contradict
	 * @param demoted in/out: gains the sub-scenes a dropped bridge left unlinked
	 * @return false if re-averaging failed, in which case the alignment cannot complete
	 */
	bool PruneConflictingSeams(
		const std::vector<Scene>& subScenes,
		std::vector<ScenePair>& scenePairs,
		const std::vector<Point3d>& globalRotations,
		std::vector<REAL>& globalScales,
		std::vector<Point3>& globalTranslations,
		std::vector<bool>& demoted);

	/**
	 * @brief Validate the averaged alignment via Sim(3) cycle consistency and decide which
	 * sub-scenes cannot be trusted with their poses.
	 *
	 * Every surviving ScenePair carries a relative Sim(3) measured from 3D-3D correspondences
	 * between two reconstructions; composing it with the averaged global transforms of its two
	 * end-points yields a residual that is identity when the edge agrees with the consensus.
	 * Edges whose residual is too large in scale, rotation or translation are conflicting, and
	 * the sub-scene most dominated by conflicting incident edge weight is demoted, iteratively
	 * (a node with a single incident edge is satisfied exactly by the averaging, so it carries
	 * no cycle evidence and can never be flagged). Sub-scenes left unplaced by rotation
	 * averaging (mergeMask false) are demoted as well.
	 *
	 * Demoted sub-scenes are merged WITHOUT their poses and 3D positions (features, image
	 * pairs and track observations only), leaving their images unregistered so the post-merge
	 * resection re-registers them incrementally against the trusted consensus — the same
	 * process that would have placed them correctly had the cluster boundary not severed
	 * their strongest pairs.
	 *
	 * @return per-sub-scene demotion flags (true = merge without poses)
	 */
	std::vector<bool> ValidateAlignment(
		const std::vector<Scene>& subScenes,
		const std::vector<bool>& mergeMask,
		const std::vector<ScenePair>& scenePairs,
		const std::vector<Point3d>& globalRotations,
		const std::vector<REAL>& globalScales,
		const std::vector<Point3>& globalTranslations) const;

	/**
	 * @brief Re-average the demoted-free sub-set until the validation verdict is stable
	 *
	 * Demoting a sub-scene removes its edges, so the scale and translation consensus must be
	 * recomputed over the survivors and re-validated; rotation averaging is already robust, so
	 * its result is kept. Demoting can also disconnect the pair graph, while scale/translation
	 * averaging pin a single gauge node, so every sub-scene outside the largest surviving
	 * component is demoted too. Each iteration demotes at least one more sub-scene, bounding
	 * the loop by their number.
	 *
	 * @return false if re-averaging failed, in which case the alignment cannot complete
	 */
	bool RefineDemotedAlignment(
		const std::vector<Scene>& subScenes,
		std::vector<ScenePair>& scenePairs,
		const std::vector<Point3d>& globalRotations,
		std::vector<REAL>& globalScales,
		std::vector<Point3>& globalTranslations,
		std::vector<bool>& demoted);

	/**
	 * @brief Merge a single scene into the global scene
	 *
	 * Moves keypoints/descriptors back from sub-scene images to the global scene
	 * (they were moved to sub-scenes during SceneCluster::ExtractSubScene to save memory).
	 * Also moves image pairs back and remaps track observation IDs.
	 * When not trusted, camera poses are not copied and all merged tracks are marked
	 * non-inlier (numInliers=0): the observations survive for later triangulation, but no
	 * pose or 3D position from the demoted sub-scene can influence the reconstruction.
	 */
	void MergeSingleScene(Scene& subScene, const IIndexArr& localToGlobal, bool trusted);

	/**
	 * @brief Merge tracks from sub-scenes and connect them via cross-sub-scene pairs
	 *
	 * Uses a union-find over global feature IDs (same pattern as BuildTracks) to:
	 * 1. Initialize each sub-scene's tracks as independent sets
	 * 2. Process cross-sub-scene pairs to connect tracks across boundaries,
	 *    using 3D proximity as validation when both sides have triangulated positions
	 * 3. Assemble final tracks, triangulating any new tracks without 3D positions
	 *
	 * Tracks of demoted sub-scenes (all observations in untrusted images) are seeded with
	 * all their observations but no 3D position, so their structure survives for the
	 * post-merge resection to re-triangulate.
	 * @param untrustedImages per-global-image flags marking images of demoted sub-scenes
	 */
	void MergeTracksWithCrossSubScenePairs(const std::vector<bool>& untrustedImages);

	// Global image ID -> (sub-scene index, local image index)
	std::unordered_map<IIndex, std::pair<uint32_t, IIndex>> globalToLocal;

	// Block pair (a < b) -> indices into scene.pairs of its cross pairs
	std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> blockPairLinks;
	// Per block: (local image, feature) -> the position of the inlier track holding that
	// observation, what the correspondence collection looks each match up in
	std::vector<std::unordered_map<PairIdx, Point3>> blockPointMaps;

	Scene& scene; // Reference to input scene
	const GlobalAlignmentConfig& config; // Global alignment configuration
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBALALIGNMENT_H_
