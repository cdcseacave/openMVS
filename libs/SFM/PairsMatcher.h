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
class SFM_API RoMa2Onnx;
enum class FeatureType : uint8_t;

/**
 * @brief Configuration for image pair matching
 */
struct SFM_API MatchConfig
{
	enum MatchMode {
		SKIP = -1,
		EXHAUSTIVE = 0,  // Match all O(N²) pairs (small scenes only)
		VOCABULARY = 1,  // Use vocabulary tree retrieval (recommended)
		SEQUENTIAL = 2,  // Match consecutive images only (ordered sequences)
		KNOWN_POSES = 3  // Select pairs from already-known camera poses
	};

	MatchMode mode = VOCABULARY;
	unsigned maxDescriptorsPerImage = 2000; // Max descriptors per image for vocabulary tree
	unsigned maxPairsPerImage = 50;     // Target pairs per image (VOCABULARY/KNOWN_POSES mode)
	bool verificationFeedback = true;   // Two-round matching: hold back part of the pair budget and re-invest it in pairs suggested by the geometrically verified matches (VOCABULARY/KNOWN_POSES mode)
	unsigned matchSequenceOverlap = 3;  // Number of subsequent images to match in SEQUENTIAL mode
	unsigned preMatchThreshold = 0;     // Minimum number of matches in pre-matching step to keep the pair (0 = disabled)
	float minFeatureDistance = 0.f;     // Minimum distance between matched features in pixels (0 = disabled)
	float matchDistance = 100.f;        // Absolute distance test threshold (100 - AKAZE 486bit, 64 - ORB 256bit, FLT_MAX - SIFT)
	float matchRatio = 0.9f;            // Lowe's ratio test threshold (0.9 - AKAZE/ORB, 0.8 - SIFT)
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

	// Pre-match pairs using vocabulary tree top descriptors (filters weak pairs)
	void PreMatch(PairIdxArr& pairsToMatch);

	// Match all image pairs according to strategy.
	// Checks existing pairs and only matches new or incomplete pairs.
	// Existing pairs with geometric data (non-empty inliers) are preserved.
	// Return number of valid image pairs created
	unsigned Match();

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

	// Geometric verification with RANSAC
	// If both cameras trust intrinsics, estimates calibrated relative pose
	// and initializes pair.relativePose, pair.E and pair.F.
	// Otherwise estimates fundamental matrix and sets pair.F.
	bool GeometricFilter(
		const Image& img1,
		const Image& img2,
		ImagePair& pair) const;

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

	// Attach the in-process ROMAv2 model and its configuration; model may be NULL, which
	// still enables the global-descriptor retrieval backend if the scene carries the
	// descriptors (the model is only needed to compute them, not to rank with them)
	void SetROMA2(RoMa2Onnx* model, const ROMA2Config& cfg);

	// Return true if the candidate pairs are ranked by the ROMAv2 global descriptors
	// instead of the vocabulary tree: retrieval enabled, the scene marked as described,
	// and every image carrying a descriptor
	bool UseGlobalDescriptors() const;

	// Build the retrieval backend on demand (lazy initialization): the global-descriptor
	// index when UseGlobalDescriptors(), the vocabulary tree otherwise.
	// Returns false if neither backend could be built.
	bool EnsureRetrievalIndex();

	// Query the ranked list of the images most similar to the given one (as an index in the
	// scene image array) from whichever retrieval backend EnsureRetrievalIndex built;
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
	// (two images sharing verified neighbors likely overlap too), while VOCABULARY propagates
	// each verified pair to the top retrieval candidates of its endpoints; the images with the
	// weakest verified connectivity then refill the remaining budget from their next
	// best-ranked candidates. attemptedPairs lists the already-matched candidates; only new
	// pairs are returned, at most as many as left in the total budget maxPairsPerImage*N/2.
	PairIdxArr CollectVerificationFeedbackPairs(const PairIdxArr& attemptedPairs);

	// Reorder pairs to minimize GPU descriptor transfers by grouping pairs sharing the same first image,
	// with secondary ordering by descriptor cost (descending) for better thread pool load balancing
	void OptimizePairsOrder(PairIdxArr& pairsToMatch);

	// Filter redundant keypoints (same position) and remap matches
	void FilterRedundantKeypoints();

	// Export image pairs to a CSV file
	static bool ExportPairsCSV(const Scene& scene, const String& fileName, float minWeight = 0.f);

private:
	// Counters accumulated by MatchPairsBatch across matching rounds
	struct MatchStats {
		unsigned newPairs = 0;
		unsigned updatedPairs = 0;
		size_t numMatches = 0;
		size_t numInliers = 0;
		size_t numFilteredInliers = 0;
	};

	// Match and geometrically verify the given candidate pairs in parallel, storing the
	// valid ones in the scene and accumulating the counters into stats.
	// Returns false only on fatal initialization errors (e.g. GPU matcher setup).
	bool MatchPairsBatch(const PairIdxArr& pairsToMatch, LPCTSTR progressCaption, MatchStats& stats);

	Scene& scene;
	const MatchConfig config;

	// Per-thread matchers for efficient parallel processing
	std::vector<cv::Ptr<cv::DescriptorMatcher>> matchers;

	// Vocabulary tree for image retrieval (lazy initialization)
	std::unique_ptr<VocabularyTree> vocabularyTree;

	// Global-descriptor retrieval index, replacing the vocabulary tree as the ranking
	// backend when the scene carries the ROMAv2 descriptors (lazy initialization)
	std::unique_ptr<GlobalDescriptors> globalDescriptors;

	// In-process ROMAv2 model and its configuration (NULL/defaults unless SetROMA2 was called)
	RoMa2Onnx* roma2 = NULL;
	ROMA2Config roma2Cfg;

	// Symmetric fused retrieval score of every pair retrieved by the last
	// CollectVocabularyPairs call, kept for CollectVerificationFeedbackPairs
	// (released by Match once the matching rounds complete)
	std::unordered_map<PairIdx::PairIndex, float> fusedRetrievalScores;
};

/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_PAIRSMATCHER_H_
