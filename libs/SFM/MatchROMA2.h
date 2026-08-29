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
	bool useMatching = true;               // guide the sparse feature matching with the ROMAv2 dense warps
	RetrievalRecipe retrievalRecipe = RetrievalRecipe::FACETS; // how to pool the global retrieval descriptor
	float retrievalPower = 0.3f;           // exponent of the signed power normalization of the retrieval descriptor
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

} // namespace SFM

#endif // _SFM_MATCHROMA2_H_
