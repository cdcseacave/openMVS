/*
 * MatchROMA2.h
 *
 * Copyright (c) 2014-2026 SEACAVE
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
 */

#ifndef _SFM_MATCHROMA2_H_
#define _SFM_MATCHROMA2_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "ImagePair.h" // the pair the infusion decision is taken on, and Pose3D through it


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;
class SFM_API Image;
class SFM_API RoMa2Onnx;
class SFM_API PairsMatcher;

// Configuration of the in-process ROMAv2 (ONNX Runtime) retrieval and dense matching
struct SFM_API ROMA2Config {
	bool enabled = false;                  // enable the in-process ROMAv2 model (explicit opt-in)
	String modelPath;                      // folder holding the exported ROMAv2 ONNX graphs (empty = $OPENMVS_ROMA2_MODEL_PATH)
	String setting = "base";               // model preset to load: turbo|fast|base
	String provider = "auto";              // execution provider: auto|cuda|coreml|dml|cpu
	bool useRetrieval = true;              // rank the candidate image pairs with the ROMAv2 global descriptors
	// guide the sparse feature matching with the ROMAv2 dense warps. EXPERIMENTAL, hence off by
	// default: end-to-end validation showed it supplies far more inliers and pairs, but degrades
	// pose accuracy when the intrinsics are self-calibrated (see docs/design/ROMA2InProcess.md,
	// Limitations). Enable with --roma2-match true, preferably together with
	// --roma2-skip-healthy 100 --roma2-max-replace 15, or with imported intrinsics
	bool useMatching = false;
	float minConfidence = 0.3f;            // minimum warp confidence for a keypoint to be tracked
	float minErodeConfidence = 0.9f;       // confidence above which a cell survives the erosion of the confidence map
	int erodeBorder = 8;                   // border size (in warp cells) to erode the confidence map (0 = disabled)
	float epipolarThreshold = 2.f;         // maximum distance to epipolar line when filtering candidates
	unsigned slotBudget = 64;              // maximum number of image descriptors kept resident on the device
	unsigned maxReplaceInliers = 0;        // only replace pairs below this inlier count (0 = replace any weaker pair)
	unsigned skipHealthyInliers = 0;       // skip pairs already having at least this many inliers (0 = warp every pair)
	unsigned feedbackMaxReplaceInliers = 15;  // maxReplaceInliers of the verification-feedback round
	unsigned feedbackSkipHealthyInliers = 100; // skipHealthyInliers of the verification-feedback round
	// minimum fraction of the warp's cells whose confidence is at least minConfidence, after the
	// erosion, for a pair the descriptor matcher did not verify to be created out of the warp alone
	// (0 = off). Pairs that already exist are never gated. On a repetitive scene the created pairs
	// are what fragments the view graph, and this gate does not fix it: measured alone at 0.05 it
	// left 32265651 on 124 of 377 registered images, against the 250 the follow-up campaign
	// pre-registered as the bar for making it a default (docs/design/ROMA2InProcess.md, Limitations)
	float minCreatedOverlap = 0.f;
	// drop the guided matches that lose a train-side collision (MatchFeaturesGeometric). Off by the
	// same ruling: measured alone it costs no verified inliers (median per pair 126 vs 126 and 158
	// vs 157 on the two captures), but it also registers far fewer images (183 of 345 against 309 on
	// f7dbf861), and on the only statistic comparable across two arms that registered different sets
	// -- the alignment-free relative rotation -- it is worse there (+18.1% against the SIFT baseline,
	// where the plain dense arm is +9.5%); see docs/design/ROMA2InProcess.md, Limitations
	bool guidedCrossCheck = false;
	// dense two-view pair validation gate, running before any descriptor matching on the pairs the
	// match mode selected: each candidate is warped, a coverage-maximising sample of the warp is
	// drawn, one geometry is fitted to that whole sample without any epipolar pre-selection, and
	// the pair is kept only if the fit's inlier subset still covers enough of both images
	// (minInlierCoverage). A rejected pair is dropped, not demoted -- it does not fall through to
	// descriptor matching. Opt-in and independent of useMatching: the gate judges pairs, the dense
	// matcher re-matches them
	bool useValidation = false;
	unsigned denseSampleSize = 2000;       // budget of the coverage-maximising warp sample (SampleWarpByCoverage)
	// the gate's whole accept/reject rule: keep the pair when min(coverageInlierA, coverageInlierB)
	// reaches this fraction of the coarse coverage grid. It replaced the RANSAC inlier ratio the
	// gate first shipped with, rather than complementing it. Measured over four Truck arms plus
	// Meetingroom and Courthouse, the ratio separated true from false pairs barely above chance
	// (restricted-population AUC 0.55-0.70) and cost recall outright -- at 0.5 warp-native px it
	// rejected 89% of the true pairs it was shown -- while coverage alone leaks 0.00-0.15% of
	// normal false pairs at 90-99% recall on all three scenes. Pairs that see different instances
	// of a repeated structure are NOT this gate's job; the triplet view-graph filter handles those.
	// See docs/design/ROMA2InProcess.md. 0 disables rejection: every pair passes
	float minInlierCoverage = 0.25f;
	// dense supplementation: on a pair the gate validated and the guided pass then verified, add
	// dense correspondences drawn from that pair's own warp ALONGSIDE its sparse matches, as
	// keypoints appended past each image's described prefix. It exists for the weakly-textured
	// pairs a descriptor matcher can only find a handful of correspondences on: they contribute
	// structure instead of dropping out. Needs both useValidation and useMatching -- the gate is
	// what makes "validated" mean anything, and the guided pass is where the warp already is, so
	// no third warp pass exists. Opt-in, like every other pass here
	bool useSupplement = false;
	// what counts as a pair weak enough to infuse, decided right after the guided pass and its
	// geometric filter: the SIFT pass failed that filter, OR it left fewer verified inliers than
	// supplementMaxInliers, OR its coverage of the pair's valid disparity area is below
	// supplementMinCoverage. Any one is enough; they catch different failures -- no descriptor
	// evidence at all, too little of it, and enough of it but all in one textured corner of an
	// overlap whose remainder is exactly what the dense draw is for.
	// Coverage is measured on the complementary draw's own (fine) bucket grid: the buckets holding a
	// confident warp cell are the valid disparity area the gate judged, and the ones a verified SIFT
	// inlier lands in are what the sparse matcher covered (WarpDrawCoverage). A failed SIFT pass
	// covers nothing, so its coverage is 0.
	unsigned supplementMaxInliers = 500;
	float supplementMinCoverage = 0.3f;
	// TOTAL correspondences a supplemented pair is drawn against, sparse and dense together. The
	// dense budget is round(supplementTotalMatches * (1 - coverage)): the share of the total
	// proportional to the part of the valid disparity area the sparse matches left empty, which is
	// the part the draw can fill. This is also what bounds the scene-wide keypoint growth a wide arm
	// has to pay for, since a dense match costs a keypoint in EACH of the two images plus one track.
	// 0 = no total budget, in which case the draw is bounded by denseSampleSize alone
	unsigned supplementTotalMatches = 2000;
	// Relative-pose choice on an infused pair that has both a SIFT pose (fitted on its verified
	// sparse inliers) and the gate's dense pose (fitted on its spread warp sample): the pair keeps
	// the SIFT pose -- the more accurate one when the two agree, sub-pixel even without coverage --
	// unless the two differ substantially, and then it takes the dense one. "Substantially" is
	// either angle over its threshold: the rotation angle of R_sift * R_dense^T, or the angle
	// between the two unit translation directions.
	// EDUCATED FIRST GUESSES, to be fitted on pseudo-GT: every infused pair emits both poses and
	// both angles at -v 3 ("ROMA2 pose check"), so the two can be swept offline against COLMAP
	// relative poses without re-running the matcher.
	float supplementPoseMaxRotationDeg = 2.f;
	float supplementPoseMaxTranslationDeg = 10.f;
	// Instead of choosing between the two poses, re-estimate one on all of the pair's
	// correspondences, sparse and dense together, with the matcher's own estimator (same branch,
	// same threshold). Off: it is a hypothesis the pseudo-GT sweep has to confirm before it can be
	// a default, and a refit on a draw that came from one of the two poses is not independent
	// evidence about it.
	bool supplementRefitPose = false;
	bool useGPU = true;                    // allow the GPU execution providers (false forces the CPU provider)

	// Return the folder holding the exported models: the explicit setting if given,
	// else the OPENMVS_ROMA2_MODEL_PATH environment variable, else empty
	inline String ResolveModelPath() const {
		if (!modelPath.empty())
			return modelPath;
		const char* const envModelPath = getenv("OPENMVS_ROMA2_MODEL_PATH");
		return envModelPath ? String(envModelPath) : String();
	}

	// Return true if the in-process ROMAv2 model is enabled, used by at least one pass, and locatable
	inline bool IsInProcessEnabled() const {
		return enabled && (useRetrieval || useMatching || useValidation) && !ResolveModelPath().empty();
	}

	// Return true if a pass needs the coarse-match graph itself (the warps), and not merely the
	// global descriptors: the ONNX sessions have to be loaded for those, descriptors alone may
	// already be stored in the scene
	inline bool NeedsWarps() const {
		return useMatching || useValidation;
	}
};
/*----------------------------------------------------------------*/

// Run the ROMAv2 describe pass over every image of the scene: pipelines each image's load
// and preprocessing on the thread pool while roma2.Describe() (which may only be driven from
// one thread) runs on the calling thread, then stores the descriptor graph's own on-device
// retrieval pooling of every successfully described image in Image::globalDescriptor. roma2
// must already be loaded (RoMa2Onnx::Load); this pass never touches the coarse-match graph.
// Returns the number of images successfully described; a per-image load/describe failure is
// logged individually and leaves that image's globalDescriptor empty, so a return value below
// scene.images.size() is the caller's cue to treat the whole pass as failed.
SFM_API unsigned ComputeGlobalDescriptorsROMA2(Scene& scene, RoMa2Onnx& roma2);
/*----------------------------------------------------------------*/

// One pair's dense infusion: the correspondences drawn from that pair's own warp, to be appended
// alongside whatever sparse matches it has. Index-parallel, in the pixels of the working orientation
// of each image (SampleWarpByCoverage's convention). Empty on a pair that was not infused.
struct SFM_API DenseSupplement {
	std::vector<Point2f> pointsA, pointsB;
	std::vector<float> confidences;
	// what the decision was taken on, for the per-pair record: the pair's SIFT coverage of its valid
	// disparity area (0 on a pair whose guided SIFT pass failed) and the budget that coverage bought
	float coverage = 0.f;
	unsigned budget = 0;
};

// Decide whether to infuse dense correspondences into one pair, and draw them if so. See the
// implementation for the rule; in short, a gate-validated pair is infused when its guided SIFT pass
// failed, or left fewer than config.supplementMaxInliers verified inliers, or covered less than
// config.supplementMinCoverage of the pair's valid disparity area, and the budget is the share of
// config.supplementTotalMatches proportional to the uncovered remainder.
// `guided` is the pair as the SIFT pass left it -- its verified sparse inliers, or no matches at all
// on a pair kept dense-only -- and must carry no dense segment yet.
// Returns true when the pair is infused, and then `supplement` holds the draw.
SFM_API bool DrawDenseSupplement(
	const Image& imgA,
	const Image& imgB,
	const Image32F2& warp,
	const Image32F& overlap,
	const ROMA2Config& config,
	const ImagePair& guided,
	DenseSupplement& supplement);

// Re-estimate ONE geometry for an infused pair on all of its correspondences, sparse and dense
// together (ROMA2Config::supplementRefitPose), through the matcher's own estimator -- same branch,
// same threshold -- on temporary Image copies whose keypoints are those correspondences. The copies
// carry no pose, for the same reason the gate's do not: a scene that happens to hold a ground-truth
// solution must not be able to leak it into a fitted geometry. The dense keypoints do not exist in
// the images yet (they are appended serially, after the pass that calls this), which is why the
// refit runs off the drawn positions in `supplement` rather than off the stored pair.
// On success `pair` receives the refitted relative pose AND the E and F composed from it, so the
// three describe one geometry -- a pair whose F came from the previous fit would hand
// ViewGraphCalibrator and the epipolar band a constraint the pose no longer agrees with. On failure
// the pair is left exactly as it was.
// Returns true if the refit landed.
SFM_API bool RefitInfusedPose(
	PairsMatcher& pairsMatcher,
	const Image& imgA,
	const Image& imgB,
	const DenseSupplement& supplement,
	ImagePair& pair);

// The rotation and unit-translation angles between two relative poses, in degrees: the angle of
// R1 * R2^T, and the angle between the two translation directions, both in ImagePair::relativePose's
// own convention.
SFM_API void RelativePoseDifference(const Pose3D& pose1, const Pose3D& pose2, float& rotationDeg, float& translationDeg);

// Which relative pose an infused pair ends up carrying (MatchPairsROMA2, and the "ROMA2 pose check"
// record it emits per infused pair).
enum class InfusedPoseChoice : uint8_t {
	NONE = 0,   // neither fit produced a pose
	SPARSE,     // the pose fitted on the pair's verified sparse inliers, which agrees with the gate's
	DENSE,      // the gate's pose, fitted on its spread warp sample: the two disagreed substantially
	DENSE_ONLY, // the pair has no sparse inliers, so it is the gate's pose or nothing
	REFIT,      // one pose re-estimated on sparse and dense together (ROMA2Config::supplementRefitPose)
};
SFM_API LPCTSTR InfusedPoseChoiceName(InfusedPoseChoice choice);

// The pose choice of an infused pair, as a rule with no side effects: the SIFT pose is the more
// accurate one WHEN THE TWO AGREE (sub-pixel correspondences, even with poor coverage), so the pair
// keeps it unless the two differ substantially -- either angle over its threshold
// (config.supplementPoseMaxRotationDeg, config.supplementPoseMaxTranslationDeg) -- and then it takes
// the dense pose, which rests on evidence spread over the whole overlap. `rotationDeg` and
// `translationDeg` receive the two angles, or NaN when there is no pair of poses to compare.
// REFIT is never returned here: it is the caller's own decision, taken when the refit succeeds.
SFM_API InfusedPoseChoice SelectInfusedPose(
	const std::optional<Pose3D>& sparsePose,
	const std::optional<Pose3D>& densePose,
	unsigned numSparse,
	const ROMA2Config& config,
	float& rotationDeg,
	float& translationDeg);
/*----------------------------------------------------------------*/

// Run the ROMAv2 dense matching pass over the given candidate pairs of an already
// descriptor-matched scene: plans which image descriptors stay resident on the device (Belady
// over the candidates in (ID1,ID2) order, at most config.slotBudget slots), then, on the
// calling thread, loads and describes those slots and runs the coarse-match graph pair by
// pair, while the thread pool turns each warp into a guided sparse re-match of that pair
// (ErodeConfidenceMap, TrackKeypointsByWarp, MatchFeaturesGeometric). Guided results are
// stored into the scene serially in (ID1,ID2) order (ApplyROMA2Pair), so what a pair replaces
// never depends on the order the pool happened to finish in (design decision 11).
// A pair the descriptor matcher did not verify is only created when the eroded confidence map
// covers at least config.minCreatedOverlap of the warp grid (0 = off); existing pairs are never
// gated. config.guidedCrossCheck selects the train-side cross-check of the guided re-match.
// bFeedbackRound selects the per-round replace policy: the first round warps every candidate
// and replaces whenever the guided set is larger, the verification-feedback round skips pairs
// that are already healthy and only replaces the weakest ones (design decision 6).
// With config.useSupplement, the pass decides right after the guided matching of a gate-validated
// pair and its geometric filter -- while that pair's warp and its own complementary draw are still
// at hand -- whether to infuse dense correspondences into it (see ROMA2Config::supplementMaxInliers
// for the rule and the budget). Infused matches are dense keypoints appended past each image's
// described prefix, drawn only where the pair's verified sparse matches are NOT
// (SampleWarpComplementary, AppendDenseMatches). Drawn on the pool but appended in the same serial
// (ID1,ID2) pass as the results, since the keypoint indices an append hands out depend on what the
// two images already carry.
// A validated pair whose guided SIFT pass FAILED is kept as a dense-only pair -- zero sparse
// inliers, the gate's F/E and relative pose as its geometry, the whole draw -- instead of being
// dropped. Such a result may only CREATE a pair: an existing pair carries sparse evidence a
// dense-only one does not, and never loses to it.
// roma2 must already be loaded (RoMa2Onnx::Load); a pair whose image could not be loaded,
// described, or matched is dropped with a message, never matched against a stale slot.
// Returns the number of scene pairs created plus replaced.
SFM_API unsigned MatchPairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, const PairIdxArr& candidatePairs, const ROMA2Config& config, bool bFeedbackRound);
/*----------------------------------------------------------------*/

// Dense two-view pair validation gate: run the ROMAv2 warp over the given candidate pairs -- the
// ones the match mode selected, before any descriptor matching -- and keep only those a single
// geometry explains. Per pair: erode the confidence map, draw a coverage-maximising sample of the
// warp (SampleWarpByCoverage, config.denseSampleSize points), then fit one geometry to that whole
// sample through PairsMatcher::GeometricFilter on temporary Image copies whose keypoints are the
// dense points (the MatchFeaturesGeometric precedent, so no second estimator exists). Nothing is
// pre-selected along the epipolar lines of a geometry the warp itself supplied, which is what makes
// the verdict independent of the warp's own claim; the pair passes when the fit's inliers still
// cover config.minInlierCoverage of both images.
// The fit is the matcher's own: GeometricFilter is called with the PairsMatcher's configuration, so
// which geometry runs is PairsMatcher::SelectGeometryBranch's decision from what the two images
// actually carry (calibrated bearings where both trust their intrinsics, F otherwise) and the
// epipolar threshold is MatchConfig::maxEpipolarError, the same precision the descriptor path
// demands. The gate holds no threshold and no branch choice of its own.
// The device slots, prefetch pipeline and warp order are exactly MatchPairsROMA2's, so the two
// passes cost the same per pair; unlike it, this pass needs no descriptors, only cameras.
// `pairs` is filtered in place to the pairs that passed -- a rejected pair is dropped, never
// demoted to ordinary descriptor matching. A candidate whose image could not be described, or whose
// warp the graph could not produce, was never judged at all, which is a different fact from being
// rejected, and the summary line counts the two separately.
// roma2 must already be loaded (RoMa2Onnx::Load); a pair whose image could not be loaded,
// described or coarse-matched is dropped with a message, never judged against a stale slot.
// Returns the number of pairs that passed the gate (== pairs.size() on return).
SFM_API unsigned ValidatePairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, PairIdxArr& pairs, const ROMA2Config& config);

} // namespace SFM

#endif // _SFM_MATCHROMA2_H_
