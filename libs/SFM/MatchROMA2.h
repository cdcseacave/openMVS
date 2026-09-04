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

#include "ImagePair.h" // the pair the verdict fits and the assembly fills, and Pose3D through it


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;
class SFM_API Image;
class SFM_API RoMa2Onnx;
class SFM_API PairsMatcher;
struct SFM_API PairWarps;

// Configuration of the in-process ROMAv2 (ONNX Runtime) retrieval and dense matching
struct SFM_API ROMA2Config {
	bool enabled = false;          // enable the in-process ROMAv2 model (explicit opt-in)
	String modelPath;              // folder of the exported graphs (empty = $OPENMVS_ROMA2_MODEL_PATH)
	String setting = "base";       // preset: turbo|fast|base
	String provider = "auto";      // execution provider: auto|cuda|coreml|dml|cpu
	bool useRetrieval = true;      // rank candidate pairs with the ROMAv2 global descriptors
	bool useMatching = false;      // one-pass dense pair matching (verdict, guided, fill, store)
	float minConfidence = 0.1f;    // a warp cell takes part (verdict, tracking, fill) at this confidence or above
	float minOverlap = 0.10f;      // verdict: min(inlier area A, inlier area B) >= minOverlap
	// Dense correspondences the fill may draw per FULL FRAME of overlap: a density, not a count per
	// pair. A pair's draw is that density over the part of its overlap its guided matches did not
	// already cover (DenseFillGridSide sets the pitch that makes it so), capped by DenseFillCeiling.
	unsigned denseMatchesPerFrame = 2000;
	unsigned slotBudget = 64;      // image descriptors kept resident on the device
	bool useGPU = true;            // allow the GPU execution providers

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

	// Return true if a pass needs the coarse-match graph itself (the warps), and not merely the
	// global descriptors: the ONNX sessions have to be loaded for those, descriptors alone may
	// already be stored in the scene
	inline bool NeedsWarps() const { return useMatching; }
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

// What the verdict knows about one pair once it ran
struct SFM_API PairVerdict {
	bool admitted = false;
	float confidentAreaA = 0.f, confidentAreaB = 0.f; // share of each warp grid at conf >= minConfidence and landing in-frame
	float inlierAreaA = 0.f, inlierAreaB = 0.f;       // share of each warp grid the fitted geometry explains
	// A's inlier cells, raster order, index-parallel, in the pixels of the working orientation of
	// each image (SampleWarpByCoverage's convention), with each cell's confidence: the population the
	// dense fill draws from
	std::vector<Point2f> inliersA, inliersB;
	std::vector<float> confidences;
};

// The most dense correspondences one pair may keep: the configured density over the SMALLER of the
// two inlier areas the verdict measured. The fill's bucket grid lives in image A's frame, so it
// bounds the dense keypoint density there and nowhere else -- but every dense correspondence costs a
// keypoint in B as well, and a warp that puts a large part of A onto a small part of B would pile
// them up in it. This is the term that stops that, and it binds only when B is the constraining
// frame: with inlierAreaA <= inlierAreaB it sits above the draw the pitch produces anyway, so it
// never charges the sparse matches' coverage a second time.
inline unsigned DenseFillCeiling(const ROMA2Config& config, const PairVerdict& verdict) {
	return (unsigned)ROUND2INT((float)config.denseMatchesPerFrame *
		MINF(verdict.inlierAreaA, verdict.inlierAreaB));
}

// Judge one candidate pair from its bidirectional warp alone. Fits one geometry on a coverage-uniform
// sample (SampleWarpByCoverage, target VERDICT_SAMPLE = 4000) of A's cells at conf >= config.minConfidence
// through pairsMatcher.GeometricFilter(tmpA, tmpB, pair, WarpTolerance(...)) on temporary Image copies
// carrying the sample as keypoints and no pose (a scene holding a ground-truth solution must not leak
// it into the fit); scores ALL of A's eligible cells against pair.F (Sampson distance <=
// WarpTolerance) -> inlierAreaA; scores all of B's eligible cells, each as the correspondence
// (DenormCoord(ba.warp[cell]) in A, cell centre in B), against the same pair.F -> inlierAreaB;
// admits iff min(inlierAreaA, inlierAreaB) >= config.minOverlap.
// On admission `pair` carries the fit's F/E/relativePose (whichever the branch produced) and no
// matches; verdict.inliersA/B are A's inlier cells. On rejection pair is Reset() and verdict says why
// (areas filled as far as they were computed). A sample under 8 cells is rejected without a fit.
// Pure function of its inputs; runs on the pool (GeometricFilter is const and thread-safe).
SFM_API void JudgePairROMA2(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairWarps& warps, const ROMA2Config& config, ImagePair& pair, PairVerdict& verdict);
/*----------------------------------------------------------------*/

// One pair's dense segment before it is appended
struct SFM_API DenseMatches {
	std::vector<Point2f> pointsA, pointsB;
	std::vector<float> confidences;
};

// Turn an admitted pair's evidence into the pair the scene stores: draw the dense fill from the
// verdict's inlier cells where the guided candidates are not (SampleWarpComplementary with occupiedA =
// the A positions of `guided`, at config.denseMatchesPerFrame's density, capped by DenseFillCeiling);
// fit ONE geometry on guided u dense through pairsMatcher.GeometricFilter at the matcher's own
// maxEpipolarError on temporary Image copies whose keypoints are those correspondences; then classify
// against pair.F: the guided matches within the
// matcher's maxEpipolarError are the sparse segment (`pair.matches[0, numFilteredInliers)`, the pair's
// descriptor evidence), the dense correspondences within WarpTolerance are the dense segment (returned
// in `dense`, appended by StorePairROMA2 after the pair exists). When the union fit fails (too few
// inliers at the sparse tolerance, or the branch's strict filters), the verdict's geometry stands:
// the classification runs against it instead, so an admitted pair is always stored -- a pair with
// zero sparse inliers is a dense-only pair, its dense segment its whole evidence.
// `pair` must arrive carrying the verdict's geometry and no matches, and `guided` in increasing
// queryIdx (MatchFeaturesGuided's own order), which is the order the sparse segment keeps.
// Returns true when the pair carries at least one correspondence of either kind.
SFM_API bool AssemblePairROMA2(const PairsMatcher& pairsMatcher, const Image& imgA, const Image& imgB,
	const PairVerdict& verdict, const std::vector<DMatch>& guided, const ROMA2Config& config, int warpSize,
	ImagePair& pair, DenseMatches& dense);

// Store one assembled pair: create it, or replace the same-key pair a previous Match() left, then
// append its dense segment (AppendDenseMatches). Serial, in (ID1,ID2) order -- the keypoint indices
// the append hands out depend on what the two images already carry.
SFM_API void StorePairROMA2(Scene& scene, std::unordered_map<PairIdx::PairIndex, IIndex>& pairIndexMap,
	ImagePair&& pair, const DenseMatches& dense, int warpSize);
/*----------------------------------------------------------------*/

// One-pass dense pair matching of the given candidate pairs (each unordered pair once, i < j): sorts
// them in (ID1,ID2) order, plans the device slots (Belady, config.slotBudget), and on the calling
// thread describes the slots and runs the bidirectional coarse-match graph pair by pair while the pool
// judges each pair (JudgePairROMA2), guides its sparse matching (TrackKeypointsByWarp,
// MatchFeaturesGuided) and assembles it (AssemblePairROMA2); assembled pairs are stored serially in
// (ID1,ID2) order (StorePairROMA2). A candidate already in scene.pairs is skipped (the feedback round
// proposes only new pairs, and a re-run must not double-store). A pair whose image could not be
// described or whose graph call failed is dropped with a message.
// The summary line reports candidates, judged, admitted, stored, dense-only, the slot plan's
// loads/reloads (the cache cost of the order), and the two skip counts apart: the candidates the
// scene already held (ordinary) and the ones an image without a camera or descriptors cost (a
// failure of an earlier stage).
// Fills `numStored` with the number of pairs stored and returns true, a pass whose verdict rejected
// every candidate storing none and still succeeding; returns false only when the pass could not run
// at all (the device slot pool could not be allocated), which the caller must treat as a failed
// matching round rather than as an empty one.
SFM_API bool MatchPairsROMA2(PairsMatcher& pairsMatcher, RoMa2Onnx& roma2, const PairIdxArr& candidatePairs,
	const ROMA2Config& config, unsigned& numStored);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_MATCHROMA2_H_
