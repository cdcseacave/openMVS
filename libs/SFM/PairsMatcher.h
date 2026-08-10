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


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Image;
class SFM_API ImagePair;
struct SFM_API DMatch;
class SFM_API Scene;
class SFM_API VocabularyTree;
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
	unsigned maxPairsPerImage = 50;     // Max pairs per image (VOCABULARY/KNOWN_POSES mode)
	unsigned expandPairsTopK = 5;       // Top-K neighbors per endpoint to expand base vocab pairs (0 = no expansion)
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

	// Build vocabulary tree on demand (lazy initialization)
	void EnsureVocabularyTree();

	// Build top-N vocab pairs for all images and expand them via top-K co-neighbors.
	// Returns a single vector of candidate pairs where the first numBasePairs are
	// the base vocabulary pairs and the rest are expanded pairs. Existing pairs
	// (as per PairExists) are excluded. If topK == 0, no expanded pairs are added.
	PairIdxArr CollectVocabularyPairs(unsigned* ptrNumBasePairs = NULL);

	// Build candidate pairs from the known camera poses: reject the pairs whose optical axes
	// diverge too much, score the remaining ones by baseline and viewing-direction agreement,
	// and keep the top-K per image (symmetric union, deduplicated).
	// Returns an empty array if less than two images are posed or the poses are degenerate.
	PairIdxArr CollectKnownPosePairs();

	// Reorder pairs to minimize GPU descriptor transfers by grouping pairs sharing the same first image,
	// with secondary ordering by descriptor cost (descending) for better thread pool load balancing
	void OptimizePairsOrder(PairIdxArr& pairsToMatch);

	// Filter redundant keypoints (same position) and remap matches
	void FilterRedundantKeypoints();

	// Export image pairs to a CSV file
	static bool ExportPairsCSV(const Scene& scene, const String& fileName, float minWeight = 0.f);

private:
	Scene& scene;
	const MatchConfig config;

	// Per-thread matchers for efficient parallel processing
	std::vector<cv::Ptr<cv::DescriptorMatcher>> matchers;

	// Vocabulary tree for image retrieval (lazy initialization)
	std::unique_ptr<VocabularyTree> vocabularyTree;
};

/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_PAIRSMATCHER_H_
