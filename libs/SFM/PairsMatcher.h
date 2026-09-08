/*
 * PairsMatcher.h
 *
 * Copyright (c) 2014-2025 SEACAVE
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

#ifndef _SFM_PAIRSMATCHER_H_
#define _SFM_PAIRSMATCHER_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"
#include "PairsWeighting.h"
#include "MatchROMA2.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Image;
class SFM_API ImagePair;
struct SFM_API DMatch;
class SFM_API Scene;
class SFM_API VocabularyTree;
class SFM_API GlobalDescriptors;
enum class FeatureType : uint8_t;

/**
 * @brief Configuration for image pair matching
 */
struct SFM_API MatchConfig
{
	enum MatchMode {
		SKIP = -1,
		EXHAUSTIVE = 0,  // Match all O(N²) pairs (small scenes only)
		VOCABULARY = 1,  // Use vocabulary tree retrieval over the local descriptors (recommended)
		SEQUENTIAL = 2,  // Match consecutive images only (ordered sequences)
		KNOWN_POSES = 3, // Select pairs from already-known camera poses
		RETRIEVAL = 4    // Use retrieval over the DINOv3+GeM global descriptors instead: no
		                 // vocabulary tree is ever built, and an image missing its descriptor
		                 // is an error, not a fallback (design decision 10). Appended rather
		                 // than inserted so existing serialized/CLI mode values stay unchanged.
	};

	MatchMode mode = VOCABULARY;
	unsigned maxDescriptorsPerImage = 2000; // Max descriptors per image for vocabulary tree
	unsigned maxPairsPerImage = 50;     // Target pairs per image (VOCABULARY/KNOWN_POSES/RETRIEVAL mode)
	bool verificationFeedback = true;   // Two-round matching: hold back part of the pair budget and re-invest it in pairs suggested by the geometrically verified matches (VOCABULARY/KNOWN_POSES/RETRIEVAL mode)
	unsigned matchSequenceOverlap = 3;  // Consecutive images each image is matched with: the whole candidate set of SEQUENTIAL mode, and a prior added to the VOCABULARY/RETRIEVAL candidates (0 = no prior)
	unsigned preMatchThreshold = 0;     // Minimum number of matches in pre-matching step to keep the pair (0 = disabled)
	float minFeatureDistance = 0.f;     // Minimum distance between matched features in pixels (0 = disabled)
	float matchDistance = 100.f;        // Absolute distance test threshold (100 - AKAZE 486bit, 64 - ORB 256bit, FLT_MAX - SIFT)
	// Lowe's ratio test threshold (0.9 - AKAZE/ORB, 0.8 - SIFT). The two matching paths range it over
	// different quantities, so the same constant is not the same strictness: PairsMatcher::MatchFeatures
	// applies it to the distances FLANN reports, which for its L2 index are SQUARED, hence a true-norm
	// ratio of sqrt(matchRatio); MatchFeaturesGuided (MatchGeometric.cpp) recomputes both sides as true
	// norms and applies it to those, hence matchRatio itself. At the SIFT default that is 0.894 for the
	// descriptor path against 0.8 for the guided one, which is therefore the stricter of the two
	float matchRatio = 0.9f;
	bool crossCheck = false;            // Enable cross-check consistency
	bool useFlannMatcher = true;        // Use FLANN (LSH/KDTree) for faster matching; set false to force BFMatcher
	unsigned minMatches = 50;           // Minimum inlier matches to accept pair (50 - AKAZE/ORB, 15 - SIFT)
	float maxEpipolarError = 4.f;       // Enable RANSAC E/F/H verification using this maximum epipolar error in pixels (0 = disabled)
	float minTriangulationAngle = 0.5f; // Minimum triangulation angle in degrees (0 = disabled)
	float reprojThreshold = 6.f;        // Maximum reprojection error (pixels, 0 = disabled)
	float epipoleFilterThreshold = 0.f; // Filter matches close to epipoles (pixels, 0 = disabled)
	bool releaseDescriptors = true;     // Release descriptors after matching to save memory
	bool forceFundamental = false;      // Force F-matrix estimation instead of E-matrix even if camera intrinsics are trusted
	bool forceFundamentalWithFocal = false; // Force F-matrix estimation with focal extraction (when both images share same camera)
	bool forceFundamentalDecomposition = false; // Force F-matrix decomposition into essential and relative pose even if trusted intrinsics are not available

	// Descriptor kind for vocabulary tree and retrieval scoring
	// Both binary and quantized floats are stored as CV_8U; this flag selects
	// Hamming (true) vs L2 on quantized bytes (false).
	bool descriptorsAreBinary = true;

	bool viewGraphCalibrationEnabled = true;  // Enable view graph calibration
	bool useCUDA = true; // use CUDA for SiftMatchGPU if available (otherwise OpenGL)

	// Pairs weighting parameters
	PairsWeightingConfig weightingCfg;

	inline bool IsMatchesFilterOn() const {
		return minTriangulationAngle > 0.f || reprojThreshold > 0.f || epipoleFilterThreshold > 0.f;
	}

	MatchConfig& DefaultsForFeatureType(FeatureType type);
};

/**
 * @brief Feature matching between image pairs
 *
 * Stateful matcher that reuses matchers and vocabulary trees for efficiency.
 * Supports multi-threading with per-thread matcher instances.
 */
class SFM_API PairsMatcher
{
public:
	/**
	 * @brief Construct pair matcher for a scene
	 * @param scene Scene with images and features to match
	 * @param config Matching configuration
	 */
	PairsMatcher(Scene& scene, const MatchConfig& config);
	~PairsMatcher();

	// Access scene
	const Scene& GetScene() const { return scene; }
	Scene& GetScene() { return scene; }

	// Access configuration
	const MatchConfig& GetConfig() const { return config; }

	// Number of private descriptor matchers, one per thread of the scene thread pool:
	// the valid range of every threadIdx argument taken by the matching entry points
	unsigned GetNumMatchers() const { return (unsigned)matchers.size(); }

	// The calling thread's own descriptor matcher, for the guided pass's k-NN query
	cv::DescriptorMatcher& GetMatcher(unsigned threadIdx) { ASSERT(threadIdx < matchers.size()); return *matchers[threadIdx]; }

	// Pre-match pairs using vocabulary tree top descriptors (filters weak pairs)
	void PreMatch(PairIdxArr& pairsToMatch);

	// Match all image pairs according to strategy.
	// Checks existing pairs and only matches new or incomplete pairs.
	// Existing pairs with geometric data (non-empty inliers) are preserved.
	// bFatal is set when a matching round exited on an error instead of finishing its candidates,
	// which the returned count cannot express: a round legitimately stores nothing (every candidate
	// already matched, or every pair rejected), and the pairs an earlier round stored hide the
	// failure of a later one from the caller's own empty-view-graph check. A scene too small to
	// have any pair is not fatal -- it processes none legitimately.
	// Return number of valid image pairs created
	unsigned Match(bool& bFatal);

	// Match features between two images
	bool MatchPair(
		const Image& img1,
		const Image& img2,
		ImagePair& pair);

	// Feature matching with ratio test and cross-check
	void MatchFeatures(
		const cv::Mat& desc1,
		const cv::Mat& desc2,
		std::vector<DMatch>& matches,
		unsigned threadIdx = 0);

	// Which geometry GeometricFilter estimates for a pair. Kept as one named decision so that a
	// caller recording the branch and the estimator choosing it can never disagree.
	enum class GeometryBranch : uint8_t {
		SHARED_FOCAL = 0, // F with focal extraction: forceFundamentalWithFocal, one shared pinhole camera
		ESSENTIAL    = 1, // 5-DoF calibrated bearings + cheirality: both cameras trust their intrinsics
		FUNDAMENTAL  = 2, // 7-DoF F: anything else, including forceFundamental over trusted intrinsics
	};
	static GeometryBranch SelectGeometryBranch(const MatchConfig& cfg, const Image& img1, const Image& img2);
	static LPCTSTR GeometryBranchName(GeometryBranch branch);

	// Geometric verification with RANSAC
	// If both cameras trust intrinsics, estimates calibrated relative pose
	// and initializes pair.relativePose, pair.E and pair.F.
	// Otherwise estimates fundamental matrix and sets pair.F.
	bool GeometricFilter(
		const Image& img1,
		const Image& img2,
		ImagePair& pair) const;

	// The same estimator at a caller-chosen epipolar tolerance, in pixels: the ROMA2 verdict fits
	// warp cells, which can only claim half a warp cell of accuracy, and demanding the descriptor
	// path's sub-pixel precision of them would reject the true pairs along with the false ones
	// (ROMA2Warp.h, WarpTolerance). The branch choice (SelectGeometryBranch), the strict filters and
	// minMatches stay the estimator's own; the three-argument form forwards config.maxEpipolarError.
	bool GeometricFilter(
		const Image& img1,
		const Image& img2,
		ImagePair& pair,
		float maxEpipolarError) const;

	// Decompose F into E and relative-pose
	// note: if intrinsics are not accurate, the decomposition will result in very few filtered inliers
	bool DecomposeFundamentalToPose(
		const Image& img1,
		const Image& img2,
		ImagePair& pair
	) const;

	// Recompute relative-pose for all image pairs, or only for those marked as needing update.
	//  - updatedCameras: if non-empty, only pairs involving these cameras are updated.
	//  - onlyTrustedIntrinsics: if true, only updates pairs where both cameras have trusted intrinsics.
	//  - onlyComputeIfMissing: if true, only computes relative pose for pairs missing it.
	// Returns number of pairs updated.
	unsigned ComputeRelativePoses(bool onlyTrustedIntrinsics = true, bool onlyComputeIfMissing = true, const std::unordered_set<CameraPtr>& updatedCameras = {});

	// Attach the in-process ROMAv2 model and its configuration for the dense one-pass matching
	// path (MatchPairsROMA2); model may be NULL if roma2Cfg leaves that path disabled.
	void SetROMA2(RoMa2Onnx* model, const ROMA2Config& cfg);

	// Build the global-descriptor index on demand, unconditionally (RETRIEVAL mode): never
	// builds nor falls back to the vocabulary tree — the backend is the DINOv3+GeM descriptors
	// already stored on every Image, however they were produced (CPU pooling or the v2 ONNX
	// graph; this call does not know or care which).
	// A scene missing a descriptor on any image is a hard error naming that image (logged by
	// GlobalDescriptors::Build), matching design decision 10: a requested-but-unavailable
	// backend never silently degrades into a different one.
	// Returns false if the index could not be built.
	bool EnsureGlobalDescriptorsIndex();

	// Query the ranked list of the images most similar to the given one (as an index in the
	// scene image array) from whichever retrieval backend its caller already built;
	// the vocabulary tree includes the query image itself in its results, the
	// global-descriptor index does not, so callers must skip self-matches.
	// Returns an empty list if no backend is ready.
	std::vector<std::pair<uint32_t, float>> QueryRetrieval(IIndex idx, unsigned maxResults) const;

	// Build vocabulary tree on demand (lazy initialization)
	void EnsureVocabularyTree();

	// Build candidate pairs from the vocabulary-tree retrieval: re-rank the per-image ranked
	// lists with symmetric reciprocal-rank fusion, keep the pairs present in the fused top-K
	// lists of both endpoints, and bridge any remaining connected components with the
	// best-scoring cross-component pairs; topK is the per-image candidate-list length
	// (see Match for how it maps to the configured pairs-per-image target).
	// Returns an empty array if the vocabulary tree cannot be built.
	PairIdxArr CollectVocabularyPairs(unsigned topK);

	// Build candidate pairs from the global-descriptor retrieval alone (RETRIEVAL mode): the
	// same symmetric reciprocal-rank fusion and connectivity bridging as CollectVocabularyPairs
	// (the fusion and the pair budget are properties of the ranking, not of the backend), but
	// the index is built by EnsureGlobalDescriptorsIndex, so no vocabulary tree is ever built
	// and a missing descriptor is a hard error rather than a fallback. topK is the per-image
	// candidate-list length (see Match for how it maps to the configured pairs-per-image target).
	// Returns an empty array if the global-descriptor index cannot be built.
	PairIdxArr CollectRetrievalPairs(unsigned topK);

	// Build candidate pairs from the known camera poses: reject the pairs whose optical axes
	// diverge too much, score the remaining ones by baseline and viewing-direction agreement,
	// and keep the pairs present in the candidate lists of both endpoints; every image also
	// keeps its nearest cameras regardless of the view-angle gate (occlusion safeguard), and
	// any remaining connected components are bridged with the best-scoring cross pairs;
	// images without a pose receive vocabulary-retrieved pairs so they can be resected later;
	// topK is the per-image candidate-list length (see Match for how it maps to the
	// configured pairs-per-image target).
	// Returns an empty array if less than two images are posed or the poses are degenerate.
	PairIdxArr CollectKnownPosePairs(unsigned topK);

	// Build additional candidate pairs from the geometrically verified pairs of the previous
	// matching round (verification feedback), investing the part of the pair budget the first
	// round did not spend: KNOWN_POSES closes the triangles of the verified pair graph
	// (two images sharing verified neighbors likely overlap too), while VOCABULARY and
	// RETRIEVAL both propagate each verified pair to the top retrieval candidates of its
	// endpoints (feedback is orthogonal to which backend round 1 ranked with); the images with the
	// weakest verified connectivity then refill the remaining budget from their next
	// best-ranked candidates. attemptedPairs lists the already-matched candidates; only new
	// pairs are returned, at most as many as left in the total budget maxPairsPerImage*N/2.
	PairIdxArr CollectVerificationFeedbackPairs(const PairIdxArr& attemptedPairs);

	// Sequential prior for the retrieval-based match modes: images are imported in sorted
	// numeric-stem order (Scene::Import), so for a video or hand-held capture the image index
	// order is the capture order and a consecutive pair verifies almost every time it is
	// attempted -- exactly the pairs a visual ranking (vocabulary tree or global-descriptor
	// retrieval) is most likely to leave out on a repetitive or textureless run. Appends the
	// missing (i, i+k) pairs for k = 1..overlap (no wrap-around) to pairs, skipping any already
	// present. overlap 0 is the prior switched off (no-op). Returns the number of pairs added.
	static unsigned AddSequentialPairs(const Scene& scene, unsigned overlap, PairIdxArr& pairs);

	// Reorder pairs to minimize GPU descriptor transfers by grouping pairs sharing the same first image,
	// with secondary ordering by descriptor cost (descending) for better thread pool load balancing
	void OptimizePairsOrder(PairIdxArr& pairsToMatch);

	// Filter redundant keypoints (same position, within 0.1 px) and remap matches.
	// Runs over dense keypoints too, which is where a dense point appended by two different pairs
	// of the same image becomes one reused point and its track can exceed two views. Duplicate
	// resolution is described-wins: a dense keypoint coinciding with a described one collapses onto
	// the described one, never the reverse, so the described prefix survives structurally; between
	// two dense points the higher warp confidence wins. The surviving keypoints of an image
	// carrying dense ones keep their original relative order and its stored described-keypoint
	// count moves through the same remap, since a removal inside the prefix shrinks the boundary.
	// Only callable once the descriptors are released: it cannot remap descriptor rows.
	void FilterRedundantKeypoints();

	// Export image pairs to a CSV file
	static bool ExportPairsCSV(const Scene& scene, const String& fileName, float minWeight, float minYield, int gridSize);

private:
	// Counters accumulated by the matching rounds
	struct MatchStats {
		unsigned newPairs = 0;
		unsigned updatedPairs = 0;
		unsigned densePairs = 0; // pairs the one-pass dense matching stored (MatchPairsROMA2)
		size_t numMatches = 0;
		size_t numInliers = 0;
		size_t numFilteredInliers = 0;
	};

	// Match and geometrically verify the given candidate pairs in parallel, storing the
	// valid ones in the scene and accumulating the counters into stats.
	// Returns false only on fatal initialization errors (e.g. GPU matcher setup).
	bool MatchPairsBatch(const PairIdxArr& pairsToMatch, LPCTSTR progressCaption, MatchStats& stats);

	// Shared core behind CollectVocabularyPairs and CollectRetrievalPairs: the symmetric
	// reciprocal-rank fusion, mutual top-K agreement and connectivity bridging over the backend
	// its caller built (QueryRetrieval picks it up automatically); the fusion and the pair
	// budget are properties of the ranking, not of the backend, so this is the only place
	// either mode implements them. Its DEBUG summary names the backend from config.mode, the
	// one thing that actually decides it, not from which index happens to be built.
	PairIdxArr CollectFusedRetrievalPairs(unsigned topK);

	// Mechanical construction behind EnsureGlobalDescriptorsIndex: allocates globalDescriptors
	// and builds it from the scene, releasing it again on failure.
	// The caller keeps its own already-built check and its own error message; this only
	// reports whether the build succeeded.
	bool BuildGlobalDescriptorsIndex();

	Scene& scene;
	const MatchConfig config;

	// Per-thread matchers for efficient parallel processing
	std::vector<cv::Ptr<cv::DescriptorMatcher>> matchers;

	// Vocabulary tree for image retrieval (lazy initialization)
	std::unique_ptr<VocabularyTree> vocabularyTree;

	// Global-descriptor retrieval index: the sole ranking backend for RETRIEVAL mode, never
	// built for VOCABULARY (lazy initialization)
	std::unique_ptr<GlobalDescriptors> globalDescriptors;

	// In-process ROMAv2 model and its configuration (NULL/defaults unless SetROMA2 was called)
	RoMa2Onnx* roma2 = NULL;
	ROMA2Config roma2Cfg;

	// Symmetric fused retrieval score of every pair retrieved by the last
	// CollectVocabularyPairs or CollectRetrievalPairs call, kept for
	// CollectVerificationFeedbackPairs (released by Match once the matching rounds complete)
	std::unordered_map<PairIdx::PairIndex, float> fusedRetrievalScores;
};

/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_PAIRSMATCHER_H_
