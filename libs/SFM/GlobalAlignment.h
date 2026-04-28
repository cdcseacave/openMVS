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
 *   similarity transform (Sim(3): rotation, translation, scale) directly from
 *   3D-3D point correspondences using RANSAC:
 *   - Build per-sub-scene observation caches mapping (localImage, feature) to
 *     the 3D position of the inlier track that contains that observation.
 *   - For each cross-sub-scene image pair and each inlier 2D match, look up
 *     both endpoints in the corresponding caches; when both hit, this yields a
 *     3D-3D point correspondence between sub-scene A and sub-scene B (both
 *     expressed in their own local frames).
 *   - Call EstimateSimilarityTransform on the collected correspondences. The
 *     threshold is chosen per pair as a fraction of the source point cloud's
 *     bounding-box diagonal so the criterion is invariant to each sub-scene's
 *     arbitrary units.
 *   - Store the result as a ScenePair with the full Sim(3) relativeTransform
 *     and the RANSAC inlier count.
 *   Pairs with too few inliers or low inlier ratio are discarded.
 *
 *   Rationale: the previous implementation used PoseLib's generalized relative
 *   pose solver on 2D-2D correspondences, which assumes both sub-scene "rigs"
 *   share the same metric scale — an assumption that does not hold after
 *   independent hierarchical reconstruction. Estimating Sim(3) from the 3D
 *   points the sub-scenes already triangulated avoids that bias entirely.
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

/**
 * @brief Configuration for global alignment
 */
struct SFM_API GlobalAlignmentConfig
{
	float minPairWeight = 3.f;         // minimum composite weight for image pairs to be included during matches import
	unsigned minCommonTracks{25};      // minimum tracks to connect sub-scenes
	bool mergeTrackInliersOnly{true};  // Phase 1: seed union-find with only inlier observations (true) or all observations (false)
};

/**
 * @brief Pairwise similarity transform between two sub-scenes
 */
struct SFM_API PairwiseSimilarity
{
	uint32_t sceneA;      // First sub-scene index
	uint32_t sceneB;      // Second sub-scene index
	Transform transform;  // Transform from A to B
	float weight;         // Confidence weight (# correspondences)

	PairwiseSimilarity() : sceneA(NO_ID), sceneB(NO_ID), weight(0.f) {}
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
	 * @brief Merge aligned sub-scenes into single scene
	 * @param subScenes Vector of sub-scenes to  aligned and merge (modified in-place)
	 * @param localToGlobals Vector of ID mappings from sub-scenes to global scene (parallel to subScenes)
	 * @return true if merge successful
	 *
	 * Combines all sub-scenes, handling duplicate cameras/points.
	 */
	bool MergeScenes(std::vector<Scene>& subScenes, const std::vector<IIndexArr>& localToGlobals);

private:
	/**
	 * @brief Build and validate global image -> (sub-scene, local image) mapping
	 *
	 * Enforces one-to-one ownership: a global image can belong to at most one sub-scene.
	 */
	void BuildGlobalToLocalMap(const std::vector<IIndexArr>& localToGlobals);

	/**
	 * @brief Stage 1: Estimate relative 7-DOF similarity transforms between
	 * connected sub-scenes via RANSAC over 3D-3D point correspondences
	 * (EstimateSimilarityTransform). Scale is recovered directly, so no
	 * separate pairwise scale estimation is needed downstream.
	 */
	bool EstimateRelativePoses(
		const std::vector<Scene>& subScenes,
		std::vector<ScenePair>& scenePairs);

	/**
	 * @brief Stage 2: Estimate global rotations from pairwise rotations
	 */
	bool EstimateGlobalRotations(
		const std::vector<ScenePair>& scenePairs,
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
	 */
	bool MergeTransformedScenes(
		std::vector<Scene>& subScenes,
		const std::vector<IIndexArr>& localToGlobals,
		const std::vector<Point3d>& globalRotations,
		const std::vector<REAL>& globalScales,
		const std::vector<Point3>& globalTranslations);

	/**
	 * @brief Merge a single scene into the global scene
	 *
	 * Moves keypoints/descriptors back from sub-scene images to the global scene
	 * (they were moved to sub-scenes during SceneCluster::ExtractSubScene to save memory).
	 * Also moves image pairs back and remaps track observation IDs.
	 */
	void MergeSingleScene(Scene& subScene, const IIndexArr& localToGlobal);

	/**
	 * @brief Merge tracks from sub-scenes and connect them via cross-sub-scene pairs
	 *
	 * Uses a union-find over global feature IDs (same pattern as BuildTracks) to:
	 * 1. Initialize each sub-scene's tracks as independent sets
	 * 2. Process cross-sub-scene pairs to connect tracks across boundaries,
	 *    using 3D proximity as validation when both sides have triangulated positions
	 * 3. Assemble final tracks, triangulating any new tracks without 3D positions
	 */
	void MergeTracksWithCrossSubScenePairs();

	// Global image ID -> (sub-scene index, local image index)
	std::unordered_map<IIndex, std::pair<uint32_t, IIndex>> globalToLocal;

	Scene& scene; // Reference to input scene
	const GlobalAlignmentConfig& config; // Global alignment configuration
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBALALIGNMENT_H_
