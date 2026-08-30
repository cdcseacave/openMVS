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

#include "Common.h" // SFM_API, String


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;
class SFM_API RoMa2Onnx;
class SFM_API PairsMatcher;

// Recipe used to pool the per-image ROMAv2 descriptor graph outputs into one global
// retrieval descriptor: FACETS pools the attention value facets (2048-d, default),
// LAYERS pools the shipped `layers` output as the reference implementation does (1024-d)
enum class RetrievalRecipe : uint8_t {
	FACETS = 0,
	LAYERS = 1
};

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
	RetrievalRecipe retrievalRecipe = RetrievalRecipe::FACETS; // how to pool the global retrieval descriptor
	float retrievalPower = 0.f;            // exponent of the signed power normalization of the retrieval descriptor (0 = the manifest's retrieval_recipes.facets.power)
	float minConfidence = 0.3f;            // minimum warp confidence for a keypoint to be tracked
	float minErodeConfidence = 0.9f;       // confidence above which a cell survives the erosion of the confidence map
	int erodeBorder = 8;                   // border size (in warp cells) to erode the confidence map (0 = disabled)
	float epipolarThreshold = 2.f;         // maximum distance to epipolar line when filtering candidates
	unsigned slotBudget = 64;              // maximum number of image descriptors kept resident on the device
	unsigned maxReplaceInliers = 0;        // only replace pairs below this inlier count (0 = replace any weaker pair)
	unsigned skipHealthyInliers = 0;       // skip pairs already having at least this many inliers (0 = warp every pair)
	unsigned feedbackMaxReplaceInliers = 15;  // maxReplaceInliers of the verification-feedback round
	unsigned feedbackSkipHealthyInliers = 100; // skipHealthyInliers of the verification-feedback round
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
		return enabled && (useRetrieval || useMatching) && !ResolveModelPath().empty();
	}
};
/*----------------------------------------------------------------*/

// Run the ROMAv2 describe pass over every image of the scene: pipelines each image's load
// and preprocessing on the thread pool while roma2.Describe() (which may only be driven from
// one thread) runs on the calling thread, then pools the descriptor-graph output of every
// successfully described image into its global retrieval descriptor (PoolRetrievalDescriptor,
// config.retrievalRecipe) and stores it in Image::globalDescriptor. roma2 must already be
// loaded (RoMa2Onnx::Load); this pass never touches the coarse-match graph.
// Returns the number of images successfully described; a per-image load/describe failure is
// logged individually and leaves that image's globalDescriptor empty, so a return value below
// scene.images.size() is the caller's cue to treat the whole pass as failed.
SFM_API unsigned ComputeGlobalDescriptorsROMA2(Scene& scene, RoMa2Onnx& roma2, const ROMA2Config& config);
/*----------------------------------------------------------------*/

// Run the ROMAv2 dense matching pass over the given candidate pairs of an already
// descriptor-matched scene: plans which image descriptors stay resident on the device (Belady
// over the candidates in (ID1,ID2) order, at most config.slotBudget slots), then, on the
// calling thread, loads and describes those slots and runs the coarse-match graph pair by
// pair, while the thread pool turns each warp into a guided sparse re-match of that pair
// (ErodeConfidenceMap, TrackKeypointsByWarp, MatchFeaturesGeometric). Guided results are
// stored into the scene serially in (ID1,ID2) order (ApplyROMA2Pair), so what a pair replaces
// never depends on the order the pool happened to finish in (design decision 11).
// bFeedbackRound selects the per-round replace policy: the first round warps every candidate
// and replaces whenever the guided set is larger, the verification-feedback round skips pairs
// that are already healthy and only replaces the weakest ones (design decision 6).
// roma2 must already be loaded (RoMa2Onnx::Load); a pair whose image could not be loaded,
// described, or matched is dropped with a message, never matched against a stale slot.
// Returns the number of scene pairs created plus replaced.
SFM_API unsigned MatchPairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, const PairIdxArr& candidatePairs, const ROMA2Config& config, bool bFeedbackRound);

} // namespace SFM

#endif // _SFM_MATCHROMA2_H_
