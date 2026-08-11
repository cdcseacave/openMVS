/*
 * PairsMatcher.cpp
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

#include "Common.h"
#include "PairsMatcher.h"
#include "Scene.h"
#include "FeaturesExtractor.h"
#include "PairsWeighting.h"
#include "VocabularyTree.h"
#include <PoseLib/poselib.h>

#ifdef _USE_SIFTGPU
#include <siftgpu/SiftGPU.h>
#endif

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("PairMtch"));


#ifdef _USE_SIFTGPU
/**
 * @brief Coordinates SiftGPU matching using thread pool
 *
 * Implements producer-consumer pattern:
 * - Main thread: Executes GPU operations (SiftMatchGPU::GetSiftMatch)
 * - Worker threads: Geometric verification in parallel
 */
class SiftGPUMatchCoordinator
{
public:
	SiftGPUMatchCoordinator(PairsMatcher& _pairsMatcher)
		: pairsMatcher(_pairsMatcher) {}

	// Initialize SiftMatchGPU context (returns false on failure)
	bool Initialize() {
		const int maxNumMatches = 32768;
		gpu.reset(CreateNewSiftMatchGPU(maxNumMatches));
		if (!gpu) {
			VERBOSE("error: failed to create SiftMatchGPU");
			return false;
		}
		// Set language
		int lang = SiftMatchGPU::SIFTMATCH_GLSL;
		#ifdef SIFTGPU_CUDA
		if (pairsMatcher.GetConfig().useCUDA)
			lang = SiftMatchGPU::SIFTMATCH_CUDA;
		#endif
		gpu->SetLanguage(lang);
		// Create/verify the OpenGL/CUDA context
		if (!gpu->CreateContextGL()) {
			VERBOSE("error: SiftMatchGPU failed to create OpenGL/CUDA context");
			return false;
		}
		// Allocate GPU memory for matching
		if (!gpu->Allocate(maxNumMatches, pairsMatcher.GetConfig().crossCheck ? 1 : 0)) {
			VERBOSE("error: not enough GPU memory to match %d features", maxNumMatches);
			return false;
		}
		if (gpu->GetLanguage() == SiftMatchGPU::SIFTMATCH_GLSL && gpu->GetMaxSift() < maxNumMatches) {
      		VERBOSE("warning: OpenGL version of SiftGPU only supports a maximum of %d matches; try switching to CUDA to avoid this limitation",
				gpu->GetMaxSift());
		}
		DEBUG_EXTRA("SiftGPU matcher initialized: %s mode", gpu->GetLanguage() == SiftMatchGPU::SIFTMATCH_CUDA ? "CUDA" : "GLSL");
		return true;
	}

	// Process all pairs with batching
	void ProcessPairs(
		const PairIdxArr& pairsToMatch,
		const std::unordered_map<uint64_t, IIndex>& existingPairMap,
		Util::Progress& progress,
		unsigned& newPairs,
		unsigned& updatedPairs,
		size_t& numMatches,
		size_t& numInliers,
		size_t& numFilteredInliers)
	{
		Scene& scene = pairsMatcher.GetScene();
		std::mutex sceneMutex;
		std::atomic<unsigned> atomicNewPairs{0}, atomicUpdatedPairs{0};
		std::atomic<size_t> atomicNumMatches{0}, atomicNumInliers{0}, atomicNumFilteredInliers{0};
		IIndex prevImageID1 = NO_ID, prevImageID2 = NO_ID;
		const int batchSize = 1000;
		for (size_t batchStart = 0; batchStart < pairsToMatch.size(); batchStart += batchSize) {
			const size_t batchEnd = std::min(batchStart + batchSize, pairsToMatch.size());
			// Main thread: GPU matching
			for (size_t _idx = batchStart; _idx < batchEnd; ++_idx) {
				++progress;
				const PairIdx pairIDs = pairsToMatch[_idx];

				// Check existing pair
				bool existingFound = false;
				ImagePair existingPair(NO_ID, NO_ID);
				IIndex existingIdx = NO_ID;

				auto it = existingPairMap.find(pairIDs.idx);
				if (it != existingPairMap.end()) {
					// Lock: the worker tasks below append new pairs concurrently, which can
					// reallocate the pairs array from under this indexed access
					std::lock_guard<std::mutex> lock(sceneMutex);
					ImagePair& p = scene.pairs[it->second];
					if (!p.HasMatches() || (pairsMatcher.GetConfig().maxEpipolarError > 0 && !p.HasGeometricVerification())) {
						// Keep the stored pair intact until the asynchronous rematch succeeds.
						existingPair = p;
						existingIdx = it->second;
						existingFound = true;
					} else {
						// Already done
						continue;
					}
				}

				// Setup pair
				ImagePair pair(pairIDs.i, pairIDs.j);
				if (existingFound) pair = std::move(existingPair);
				else { pair.ID1 = pairIDs.i; pair.ID2 = pairIDs.j; }
				pair.matches.clear();

				const Image& img1 = scene.images[pair.ID1];
				const Image& img2 = scene.images[pair.ID2];

				// GPU matching (main thread, blocking)
				if (gpu->GetMaxSift() < img1.descriptors.rows || gpu->GetMaxSift() < img2.descriptors.rows) {
					VERBOSE("error: not enough GPU memory to match %d and %d features; increase SiftMatchGPU max_sift parameter",
						img1.descriptors.rows, img2.descriptors.rows);
				}
				if (prevImageID1 != pair.ID1) {
					gpu->SetDescriptors(0, img1.descriptors.rows, img1.descriptors.ptr<unsigned char>());
					prevImageID1 = pair.ID1;
				}
				if (prevImageID2 != pair.ID2) {
					gpu->SetDescriptors(1, img2.descriptors.rows, img2.descriptors.ptr<unsigned char>());
					prevImageID2 = pair.ID2;
				}

				const int numMaxMatches = MINF(img1.descriptors.rows, img2.descriptors.rows);
				CLISTDEF0(DMatch) matchBuffer(numMaxMatches);
				int numMatchesFound = gpu->GetSiftMatch(numMaxMatches, reinterpret_cast<uint32_t (*)[2]>(matchBuffer.data()),
					0.7f, pairsMatcher.GetConfig().matchRatio, 1);
				if (numMatchesFound == 0)
					continue;
				matchBuffer.resize((size_t)numMatchesFound);

				// Submit post-processing task (capture by value to avoid dangling references)
				scene.threadPool.detach_task([this, pair = std::move(pair), matchBuffer = std::move(matchBuffer), existingIdx, &img1, &img2, &sceneMutex, &atomicNewPairs, &atomicUpdatedPairs, &atomicNumMatches, &atomicNumInliers, &atomicNumFilteredInliers]() mutable {
					// Copy matches to pair
					pair.matches.resize(matchBuffer.size());
					memcpy(pair.matches.data(), matchBuffer.data(), matchBuffer.size() * sizeof(DMatch));
					atomicNumMatches.fetch_add(pair.GetNumMatches(), std::memory_order_relaxed);
					if (pair.GetNumMatches() < pairsMatcher.GetConfig().minMatches) {
						pair.InvalidateMatches();
						return;
					}
					// Run geometric verification
					if (pairsMatcher.GetConfig().maxEpipolarError > 0) {
						if (!pairsMatcher.GeometricFilter(img1, img2, pair))
							return;
						DEBUG_ULTIMATE("Matched pair (% 4u, % 4u): % 5u matches, %u inliers",
							img1.ID, img2.ID, pair.GetNumMatches(), pair.GetNumFilteredInliers());
					} else {
						// No geometric verification - all matches are "inliers"
						DEBUG_ULTIMATE("Matched pair (% 4u, % 4u): % 5u matches",
						    img1.ID, img2.ID, pair.GetNumMatches());
					}
					atomicNumInliers.fetch_add(pair.GetNumInliers(), std::memory_order_relaxed);
					atomicNumFilteredInliers.fetch_add(pair.GetNumFilteredInliers(), std::memory_order_relaxed);
					// Store pair into scene; the lock also covers the indexed write-back,
					// as appending a new pair can reallocate the pairs array
					std::lock_guard<std::mutex> lock(sceneMutex);
					if (existingIdx != NO_ID) {
						pairsMatcher.GetScene().pairs[existingIdx] = std::move(pair);
						atomicUpdatedPairs.fetch_add(1, std::memory_order_relaxed);
					} else {
						pairsMatcher.GetScene().pairs.emplace_back(std::move(pair));
						atomicNewPairs.fetch_add(1, std::memory_order_relaxed);
					}
				});
			}
			// Wait for batch completion
			scene.threadPool.wait();
		}

		newPairs = atomicNewPairs.load(std::memory_order_relaxed);
		updatedPairs = atomicUpdatedPairs.load(std::memory_order_relaxed);
		numMatches = atomicNumMatches.load(std::memory_order_relaxed);
		numInliers = atomicNumInliers.load(std::memory_order_relaxed);
		numFilteredInliers = atomicNumFilteredInliers.load(std::memory_order_relaxed);
	}

private:
	PairsMatcher& pairsMatcher;
	std::unique_ptr<SiftMatchGPU> gpu;
};
#endif // _USE_SIFTGPU


MatchConfig& MatchConfig::DefaultsForFeatureType(FeatureType type) {
	switch (type) {
	case FeatureType::AKAZE:
		descriptorsAreBinary = true;
		maxDescriptorsPerImage = 2000;
		matchDistance = 100.f;
		matchRatio = 0.9f;
		minMatches = 50;
		weightingCfg.sigmaInlierPerMatches = 0.6f; // AKAZE typically have around 70% inliers among matches
		break;
	case FeatureType::ORB:
		descriptorsAreBinary = true;
		maxDescriptorsPerImage = 2000;
		matchDistance = 64.f;
		matchRatio = 0.9f;
		minMatches = 50;
		weightingCfg.sigmaInlierPerMatches = 0.6f; // ORB typically have around 70% inliers among matches
		break;
	case FeatureType::SIFT:
	case FeatureType::SIFTGPU:
		descriptorsAreBinary = false;
		maxDescriptorsPerImage = 1000; // SIFT descriptors are larger and more descriptive, need less per image
		matchDistance = FLT_MAX; // disable distance test for SIFT
		matchRatio = 0.8f;
		minMatches = 15;
		weightingCfg.sigmaInlierPerMatches = 0.77f; // SIFT typically have around 90% inliers among matches
		break;
	default:
		ASSERT("Unknown feature type for match config defaults" == NULL);
		break;
	}
	return *this;
}
/*----------------------------------------------------------------*/


PairsMatcher::PairsMatcher(Scene& _scene, const MatchConfig& _config)
	: scene(_scene), config(_config)
{
	// Determine norm type from descriptor kind (binary vs quantized float)
	// Both are stored as CV_8U, but binary uses Hamming distance, quantized uses L2
	const int normType = config.descriptorsAreBinary ? cv::NORM_HAMMING : cv::NORM_L2;
	const bool useFlannMatcher = config.useFlannMatcher && !config.crossCheck;
	// Create one matcher per thread
	const unsigned nThreads = scene.nMaxThreads;
	matchers.resize(nThreads);
	if (useFlannMatcher) {
		if (config.descriptorsAreBinary) {
			auto indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
			auto searchParams = cv::makePtr<cv::flann::SearchParams>(50);
			for (unsigned i = 0; i < nThreads; ++i)
				matchers[i] = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
		} else {
			auto indexParams = cv::makePtr<cv::flann::KDTreeIndexParams>(4);
			auto searchParams = cv::makePtr<cv::flann::SearchParams>(50);
			for (unsigned i = 0; i < nThreads; ++i)
				matchers[i] = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
		}
	} else {
		for (unsigned i = 0; i < nThreads; ++i)
			matchers[i] = cv::BFMatcher::create(normType, config.crossCheck);
	}
	if (config.useFlannMatcher && !useFlannMatcher)
		DEBUG_EXTRA("Cross-check enabled; forcing BFMatcher instead of FLANN");
	DEBUG("PairsMatcher initialized with %u threads, descriptor type: %s, matcher: %s",
	    nThreads, config.descriptorsAreBinary ? "binary" : "quantized",
	    useFlannMatcher ? "FLANN" : "BF");
}
PairsMatcher::~PairsMatcher() = default;


void PairsMatcher::MatchFeatures(
	const cv::Mat& desc1,
	const cv::Mat& desc2,
	std::vector<DMatch>& matches,
	unsigned threadIdx)
{
	matches.clear();
	if (desc1.rows < (int)config.minMatches || desc2.rows < (int)config.minMatches)
		return;
	// Reuse pre-initialized matcher for this thread
	ASSERT(threadIdx < matchers.size());
	cv::Ptr<cv::DescriptorMatcher>& matcher = matchers[threadIdx];
	if (config.crossCheck) {
		// BF matching (supports cross-check)
		std::vector<cv::DMatch> bfMatches;
		matcher->match(desc1, desc2, bfMatches);
		matches.reserve(bfMatches.size());
		for (const auto& m : bfMatches)
			matches.emplace_back(m);
	} else {
		// BF/FLANN KNN matching with Lowe's ratio test
		std::vector<std::vector<cv::DMatch>> knnMatches;
		if (config.descriptorsAreBinary) {
			matcher->knnMatch(desc1, desc2, knnMatches, 2);
		} else {
			cv::Mat desc1f, desc2f;
			desc1.convertTo(desc1f, CV_32F);
			desc2.convertTo(desc2f, CV_32F);
			matcher->knnMatch(desc1f, desc2f, knnMatches, 2);
		}
		for (const auto& m : knnMatches)
			if (m.size() == 2 && m[0].distance < config.matchDistance && m[0].distance < config.matchRatio * m[1].distance)
				matches.push_back(m[0]);
	}
}

bool PairsMatcher::GeometricFilter(
	const Image& img1,
	const Image& img2,
	ImagePair& pair) const
{
	// Start with no outliers; we'll partition after RANSAC
	pair.ResetInlierMatches();
	pair.ResetGeometry();
	if (pair.matches.size() < 8) {
		pair.InvalidateMatches();
		return false;
	}
	ASSERT(img1.HasCamera() && img2.HasCamera());

	// Configure RANSAC options.
	// PoseLib master: RelativePoseOptions wraps RansacOptions + BundleOptions and
	// holds the inlier threshold (max_error) directly. The estimate_* free functions
	// no longer take a separate BundleOptions parameter.
	poselib::RelativePoseOptions opt;
	opt.max_error = config.maxEpipolarError; // reprojection error threshold
	opt.ransac.min_iterations = 100; // min iterations
	opt.ransac.max_iterations = 10000; // max iterations
	std::vector<char> inliers;

	// Lambda to fetch matched points from the pair
	const auto FetchPoints = [&]() {
		const float minFeatureDistanceSq = SQUARE(config.minFeatureDistance);
		std::vector<poselib::Point2D> pts1, pts2;
		pts1.reserve(pair.matches.size());
		pts2.reserve(pair.matches.size());
		for (const auto& m : pair.matches) {
			const cv::Point2f& pt1 = img1.keypoints[m.queryIdx].pt;
			const cv::Point2f& pt2 = img2.keypoints[m.trainIdx].pt;
			if (minFeatureDistanceSq > 0 && normSq(pt1 - pt2) < minFeatureDistanceSq)
				continue; // skip matches that are too close
			pts1.emplace_back(pt1.x, pt1.y);
			pts2.emplace_back(pt2.x, pt2.y);
		}
		return std::make_pair(pts1, pts2);
	};

	// Common finalize helper: partition inliers, fill pose, compose E/F, apply strict filtering
	const auto FinalizeRelative = [&](
		const poselib::CameraPose& pl_pose,
		const KMatrix* pK1,
		const KMatrix* pK2,
		const std::vector<char>& inliersMask,
		size_t numInliers) -> bool
	{
		ASSERT(numInliers >= config.minMatches);
		// Partition matches into inliers and outliers
		pair.PartitionMatchesByMask(inliersMask, (int)numInliers);
		// Fill relative pose
		Pose3D& rel = pair.relativePose.emplace();
		rel.R = pl_pose.R();
		rel.SetT(pl_pose.t);
		// Compose E matrix from relative pose
		pair.E = ImagePair::ComposeEssentialMatrix(rel);
		if (pK1 && pK2 && !pair.F.has_value()) {
			// Compose F matrix from E and K matrices
			pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), *pK1, *pK2);
		}
		if (config.IsMatchesFilterOn()) {
			// Further filter matches based on triangulation angle, reprojection error, epipole proximity
			const unsigned numFilteredInliers = pair.FilterMatches(img1, img2, config.minTriangulationAngle, config.reprojThreshold, config.epipoleFilterThreshold);
			if (numFilteredInliers < config.minMatches) {
				pair.InvalidateMatches();
				return false;
			}
		}
		return true;
	};

	// Check if we should estimate focal length using shared-focal estimator
	// This is for uncalibrated scenarios where both images use the same (unknown) focal length
	if (config.forceFundamentalWithFocal && img1.pCamera == img2.pCamera && img1.pCamera->GetType() == CameraType::PINHOLE) {
		const Camera& cam = *img1.pCamera;
		KMatrix K = cam.GetK();
		const Point2 pp(K(0,2), K(1,2)); // Principal point
		const auto [pts1, pts2] = FetchPoints();

		// Use shared-focal relative pose estimator
		poselib::ImagePair plImagePair;
		opt.real_focal_check = true;
		poselib::RansacStats stats = poselib::estimate_shared_focal_relative_pose(
			pts1, pts2, pp, opt, &plImagePair, &inliers);
		if (stats.num_inliers < config.minMatches) {
			pair.InvalidateMatches();
			return false;
		}

		// Extract estimated focal length
		const double estimatedFocal = plImagePair.camera1.focal();
		DEBUG_ULTIMATE("GeometricFilter: shared-focal estimator succeeded with %zu inliers, f=%.2f",
			stats.num_inliers, estimatedFocal);

		// Build K matrix with estimated focal
		K(0,0) = K(1,1) = estimatedFocal;
		// Finalize with shared-focal pose
		return FinalizeRelative(plImagePair.pose, &K, &K, inliers, stats.num_inliers);
	}

	// Calibrated branch: if both cameras trust intrinsics, estimate relative pose
	if (!config.forceFundamental && img1.TrustIntrinsics() && img2.TrustIntrinsics()) {
		const Camera& cam1 = *img1.pCamera;
		const Camera& cam2 = *img2.pCamera;
		const KMatrix K1 = cam1.GetK();
		const KMatrix K2 = cam2.GetK();

		// Unified bearing-vector relative pose path: works for any central camera
		// model. Convert the pixel-space Sampson threshold to an angular threshold
		// by averaging the per-camera angular equivalents (the Sampson-on-sphere
		// residual is measured in radians, so the average is a valid combined
		// threshold for a stereo pair with possibly different pixel resolutions):
		//   angle_k = cam_k.PixelErrorToAngular(pixel_threshold)
		//   angle   = 0.5 * (angle_1 + angle_2)
		// For pinhole this reduces to 0.5/focal_1 + 0.5/focal_2 in the small-angle
		// limit (previous hand-rolled scaling). The bearing estimator interprets
		// opt.max_error as an angle in radians (converted internally to sin(angle),
		// the unit of its unit-norm symmetric Sampson residual).
		const REAL pxErr = opt.max_error;
		const REAL angle1 = cam1.PixelErrorToAngular(pxErr);
		const REAL angle2 = cam2.PixelErrorToAngular(pxErr);
		opt.max_error = 0.5 * (angle1 + angle2);

		// Extract matched keypoints and convert directly to 3D unit bearing vectors
		const float minFeatureDistanceSq = SQUARE(config.minFeatureDistance);
		std::vector<poselib::Point3D> bearings1, bearings2;
		bearings1.reserve(pair.matches.size());
		bearings2.reserve(pair.matches.size());
		for (const auto& m : pair.matches) {
			const cv::Point2f& pt1 = img1.keypoints[m.queryIdx].pt;
			const cv::Point2f& pt2 = img2.keypoints[m.trainIdx].pt;
			if (minFeatureDistanceSq > 0 && normSq(pt1 - pt2) < minFeatureDistanceSq)
				continue; // skip matches that are too close
			bearings1.emplace_back(cam1.UnprojectNormalized(Cast<REAL>(pt1)));
			bearings2.emplace_back(cam2.UnprojectNormalized(Cast<REAL>(pt2)));
		}

		poselib::CameraPose plPose;
		if (pair.relativePose.has_value()) {
			// Initialize with existing pose (from PreMatch) to guide RANSAC
			plPose = poselib::CameraPose(pair.relativePose->R, pair.relativePose->GetT());
			opt.ransac.score_initial_model = true;
		}
		// Cheirality check stays at its default (enabled) — it's bearing-native and
		// works for both pinhole and spherical back-hemisphere features. Without it,
		// the four (R, ±t), (R', ±t) decompositions of the essential matrix all have
		// identical Sampson scores and RANSAC picks whichever one the 5-point solver
		// returned first.
		poselib::RansacStats stats = poselib::estimate_relative_pose_bearings(
			bearings1, bearings2,
			opt,
			&plPose,
			&inliers);
		if (stats.num_inliers < config.minMatches) {
			pair.InvalidateMatches();
			return false;
		}
		// F is only geometrically meaningful when BOTH cameras are pinhole —
		// SphericalCamera::GetK() returns IDENTITY, so composing F from a mixed
		// pair yields garbage. Pass null Ks unless both sides are pinhole; the
		// downstream consumers that rely on F (e.g. MatchGeometric descriptor
		// filtering, ViewGraphCalibrator) check pair.F.has_value() and take
		// the bearing/E path when F is absent.
		const bool bothPinhole(cam1.GetType() == CameraType::PINHOLE && cam2.GetType() == CameraType::PINHOLE);
		const KMatrix *pK1 = bothPinhole ? &K1 : nullptr;
		const KMatrix *pK2 = bothPinhole ? &K2 : nullptr;
		return FinalizeRelative(plPose, pK1, pK2, inliers, stats.num_inliers);
	}

	// Uncalibrated branch: estimate fundamental matrix
	const auto [pts1, pts2] = FetchPoints();
	Eigen::Matrix3d F;
	if (pair.F.has_value()) {
		// Initialize with existing fundamental matrix (from PreMatch) to guide RANSAC
		F = pair.F.value();
		opt.ransac.score_initial_model = true;
	}
	poselib::RansacStats stats = poselib::estimate_fundamental(pts1, pts2, opt, &F, &inliers);
	if (stats.num_inliers < config.minMatches) {
		pair.InvalidateMatches();
		return false;
	}
	pair.PartitionMatchesByMask(inliers, (int)stats.num_inliers);

	// Update F on the pair
	pair.F = F.cast<REAL>();

	// Decompose F into E and relative pose if intrinsics are trusted
	// note: if intrinsics are not accurate, the decomposition will result in very few filtered inliers
	if (config.forceFundamentalDecomposition || (img1.TrustIntrinsics() && img2.TrustIntrinsics()))
		return DecomposeFundamentalToPose(img1, img2, pair);
	return true;
}


bool PairsMatcher::DecomposeFundamentalToPose(const Image& img1, const Image& img2, ImagePair& pair) const
{
	ASSERT(pair.ID1 == img1.ID);
	ASSERT(pair.ID2 == img2.ID);
	ASSERT(img1.HasCamera() && img2.HasCamera());
	ASSERT(pair.F.has_value());
	const KMatrix K1 = img1.GetK();
	const KMatrix K2 = img2.GetK();
	// Decompose F into E
	pair.E = ImagePair::DecomposeFundamentalMatrix(pair.F.value(), K1, K2);
	// Decompose E into relative pose using cheirality check
	// (only if same image size, a limitation of cv::recoverPose which assumes same K)
	if (img1.GetWidth() == img2.GetWidth() && img1.GetHeight() == img2.GetHeight()) {
		const auto [points1, points2] = pair.GetMatchedPoints(img1, img2);
		const unsigned numInliers = ImagePair::RecoverPose(
			pair.E.value(),
			points1, points2,
			K1,
			pair.relativePose.emplace());
		if (numInliers < config.minMatches) {
			// Failed to recover a valid pose
			pair.relativePose.reset();
			pair.InvalidateMatches();
			return false;
		}
		if (config.IsMatchesFilterOn()) {
			// Apply strict outlier filtering (cheirality, angle, epipole)
			const unsigned numFilteredInliers = pair.FilterMatches(img1, img2, config.minTriangulationAngle, config.reprojThreshold, config.epipoleFilterThreshold);
			if (numFilteredInliers < config.minMatches) {
				pair.InvalidateMatches();
				return false;
			}
		}
	}
	return true;
}

unsigned PairsMatcher::ComputeRelativePoses(bool onlyTrustedIntrinsics, bool onlyComputeIfMissing, const std::unordered_set<CameraPtr>& updatedCameras)
{
	TD_TIMER_STARTD();
	std::atomic<unsigned> numPairsUpdated{0};
	std::atomic<size_t> numMatches{0};
	std::atomic<size_t> numInliers{0};
	std::atomic<size_t> numFilteredInliers{0};

	cv::setNumThreads(1); // temporary turn off multi-threading for OpenCV functions
	scene.threadPool.detach_loop(0u, scene.pairs.size(), [&](unsigned i) {
		ImagePair& pair = scene.pairs[i];
		if (onlyComputeIfMissing && pair.relativePose.has_value())
			return;
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		if (onlyTrustedIntrinsics && (!img1.TrustIntrinsics() || !img2.TrustIntrinsics()))
			return;
		if (!updatedCameras.empty() && updatedCameras.count(img1.pCamera) == 0 && updatedCameras.count(img2.pCamera) == 0)
			return;
		// Recompute relative pose with the new intrinsics
		GeometricFilter(img1, img2, pair);
		numMatches += pair.GetNumMatches();
		numInliers += pair.GetNumInliers();
		numFilteredInliers += pair.GetNumFilteredInliers();
		++numPairsUpdated;
	});
	scene.threadPool.wait();
	cv::setNumThreads(scene.nMaxThreads);

	DEBUG("Relative pose updated for %u/%u pairs: %zu matches, %zu inliers, %zu filtered inliers (%s)",
		numPairsUpdated.load(), scene.pairs.size(), numMatches.load(), numInliers.load(), numFilteredInliers.load(), TD_TIMER_GET_FMT().c_str());
	if (numPairsUpdated.load()) {
		// Recompute pair weights
		ComputePairsWeights(scene, config.weightingCfg);
	}
	return numPairsUpdated.load();
}


bool PairsMatcher::MatchPair(
	const Image& img1,
	const Image& img2,
	ImagePair& pair)
{
	// Match features if not already matched
	if (!pair.HasMatches()) {
		// Initialize pair
		ASSERT(img1.ID < img2.ID);
		pair.ID1 = img1.ID;
		pair.ID2 = img2.ID;
		// Match features using thread-local matcher
		const static thread_local unsigned threadIdx = std::hash<std::thread::id>{}(std::this_thread::get_id()) % matchers.size();
		ASSERT(img1.descriptors.rows == (int)img1.keypoints.size());
		ASSERT(img2.descriptors.rows == (int)img2.keypoints.size());
		MatchFeatures(img1.descriptors, img2.descriptors, pair.matches, threadIdx);
		if (pair.GetNumMatches() < config.minMatches) {
			pair.InvalidateMatches();
			return false;
		}
	}

	// Geometric verification
	if (config.maxEpipolarError > 0) {
		if (!GeometricFilter(img1, img2, pair))
			return false;
		ASSERT(pair.GetNumFilteredInliers() >= config.minMatches);
		DEBUG_ULTIMATE("Matched pair (% 4u, % 4u): % 5u matches, %u inliers",
			img1.ID, img2.ID, pair.GetNumMatches(), pair.GetNumFilteredInliers());
	} else {
		// No geometric verification - all matches are "inliers"
		DEBUG_ULTIMATE("Matched pair (% 4u, % 4u): % 5u matches",
		    img1.ID, img2.ID, pair.GetNumMatches());
	}
	return true;
}

void PairsMatcher::EnsureVocabularyTree()
{
	if (vocabularyTree)
		return;
	TD_TIMER_STARTD();
	// Build vocabulary tree
	vocabularyTree = std::make_unique<VocabularyTree>();
	VocabularyTree::Config vcfg;
	vcfg.descriptorsAreBinary = config.descriptorsAreBinary;
	vcfg.maxDescriptorsPerImage = config.maxDescriptorsPerImage;
	// Keep defaults for K/L/iters/seed from VocabularyTree
	if (!vocabularyTree->Build(scene, vcfg)) {
		VERBOSE("error: failed to build vocabulary tree");
		vocabularyTree.reset();
	}
	DEBUG("Vocabulary tree built from %u images in %s",
	      scene.images.size(), TD_TIMER_GET_FMT().c_str());
}

PairIdxArr PairsMatcher::CollectVocabularyPairs(unsigned topK)
{
	PairIdxArr result;
	// Ensure vocabulary tree is ready
	EnsureVocabularyTree();
	if (!vocabularyTree || !vocabularyTree->IsValid())
		return result;

	TD_TIMER_STARTD();

	const IIndex nImages = scene.images.size();
	topK = (unsigned)MINF(topK, nImages - 1);
	const unsigned queryDepth = (unsigned)MINF(MAXF(topK*4u, 100u), nImages - 1);
	const float rrfK0 = 10.f; // reciprocal-rank-fusion damping constant

	// 1) Query the ranked similar-image list for each image (without matching);
	//    the list is queried deeper than top-K so the fused rank of a pair can be
	//    recovered even when only one endpoint retrieves the other early
	std::unordered_map<IIndex, IIndex> idxFromID;
	idxFromID.reserve(nImages);
	FOREACH(idx, scene.images)
		idxFromID.emplace(scene.images[idx].ID, idx);
	std::vector<IIndexArr> rankedPerImage(nImages); // candidate image indices, best first
	scene.threadPool.detach_loop(0u, nImages, [&](IIndex i) {
		const Image& img = scene.images[i];
		const auto candidates = vocabularyTree->Query(img, queryDepth + 1);
		IIndexArr& ranked = rankedPerImage[i];
		ranked.reserve((IIndex)candidates.size());
		for (const auto& kv : candidates) {
			if (kv.first == img.ID)
				continue; // skip self-match
			const auto it = idxFromID.find(kv.first);
			if (it != idxFromID.end())
				ranked.push_back(it->second);
		}
	});
	scene.threadPool.wait();

	// 2) Score every retrieved pair by symmetric reciprocal-rank fusion: each direction
	//    contributes 1/(k0+rank), so a pair both images retrieve early outranks a pair
	//    only one image scores high; being rank-based, the fusion is immune to the
	//    per-query score-scale drift of raw TF-IDF similarities
	std::unordered_map<PairIdx::PairIndex, float> pairScores;
	pairScores.reserve((size_t)nImages * MINF(queryDepth, 16u));
	for (IIndex i = 0; i < nImages; ++i) {
		const IIndexArr& ranked = rankedPerImage[i];
		FOREACH(r, ranked)
			pairScores[MakePairIdx(scene.images[i].ID, scene.images[ranked[r]].ID).idx] += 1.f / (rrfK0 + (float)r);
	}

	// 3) Keep the pairs ranked in the fused top-K lists of BOTH endpoints: requiring
	//    mutual agreement suppresses the one-sided (mostly false) tail of each list,
	//    which is what wastes most of the matching budget at small top-K settings
	struct ScoredPair {
		PairIdx::PairIndex idx;
		float score;
	};
	std::vector<CLISTDEF0(ScoredPair)> topPerImage(nImages);
	for (const auto& [pairIndex, score] : pairScores) {
		const PairIdx pair(pairIndex);
		topPerImage[idxFromID[pair.i]].push_back({pairIndex, score});
		topPerImage[idxFromID[pair.j]].push_back({pairIndex, score});
	}
	std::unordered_map<PairIdx::PairIndex, uint8_t> numEndpointVotes;
	numEndpointVotes.reserve((size_t)nImages * MINF(topK, 16u));
	for (IIndex i = 0; i < nImages; ++i) {
		CLISTDEF0(ScoredPair)& scoredPairs = topPerImage[i];
		if (scoredPairs.size() > topK) {
			scoredPairs.Sort([](const ScoredPair& a, const ScoredPair& b) {
				return a.score > b.score;
			});
			scoredPairs.Resize(topK);
		}
		for (const ScoredPair& sp : scoredPairs)
			++numEndpointVotes[sp.idx];
	}
	std::unordered_set<PairIdx::PairIndex> setPairs;
	setPairs.reserve(numEndpointVotes.size() / 2);
	for (const auto& [pairIndex, votes] : numEndpointVotes) {
		ASSERT(votes <= 2);
		if (votes == 2 && setPairs.emplace(pairIndex).second)
			result.emplace_back(PairIdx(pairIndex));
	}
	const unsigned numMutualPairs = (unsigned)result.size();

	// 4) Connectivity backbone: add the maximum-similarity edges bridging the connected
	//    components of the selected pair graph (Kruskal over all retrieved pairs, with
	//    the union-find seeded by the mutual pairs), so a sparse selection cannot
	//    silently split the view graph
	IIndexArr ufParent(nImages);
	std::iota(ufParent.begin(), ufParent.end(), 0u);
	const auto Find = [&ufParent](IIndex x) {
		while (ufParent[x] != x)
			x = ufParent[x] = ufParent[ufParent[x]];
		return x;
	};
	for (const PairIdx& p : result)
		ufParent[Find(idxFromID[p.i])] = Find(idxFromID[p.j]);
	CLISTDEF0(ScoredPair) allPairs(0, (IIndex)pairScores.size());
	for (const auto& [pairIndex, score] : pairScores)
		allPairs.push_back({pairIndex, score});
	allPairs.Sort([](const ScoredPair& a, const ScoredPair& b) {
		return a.score > b.score;
	});
	for (const ScoredPair& sp : allPairs) {
		const PairIdx pair(sp.idx);
		const IIndex rootA = Find(idxFromID[pair.i]);
		const IIndex rootB = Find(idxFromID[pair.j]);
		if (rootA != rootB) {
			ufParent[rootA] = rootB;
			if (setPairs.emplace(sp.idx).second)
				result.emplace_back(pair);
		}
	}
	DEBUG("Vocabulary-based matching: %u candidate pairs, %u mutual top-%u and %u connectivity bridges (%.2f/%u pairs/image) in %s",
		result.size(), numMutualPairs, topK, result.size() - numMutualPairs,
		(float)result.size() / nImages, config.maxPairsPerImage, TD_TIMER_GET_FMT().c_str());
	// keep the fused retrieval scores for the verification-feedback round
	fusedRetrievalScores = std::move(pairScores);
	return result;
}

namespace {

// Collect the images having a known pose (as indices in the scene image array)
IIndexArr CollectPosedImages(const Scene& scene)
{
	IIndexArr posedImages(0, scene.images.size());
	FOREACH(i, scene.images)
		if (scene.images[i].HasPose())
			posedImages.push_back(i);
	return posedImages;
}

// Estimate the scene scale as the median nearest-neighbor camera-center distance
// (0 if all camera centers coincide)
REAL EstimateSceneScale(Scene& scene, const IIndexArr& posedImages)
{
	const IIndex nPosed = posedImages.size();
	REALArr nearestDistances(nPosed);
	scene.threadPool.detach_loop(0u, nPosed, [&](IIndex a) {
		const CMatrix& C = scene.images[posedImages[a]].C;
		REAL nearestDistance = std::numeric_limits<REAL>::max();
		for (IIndex b = 0; b < nPosed; ++b) {
			if (b == a)
				continue;
			const REAL distance = norm(C - scene.images[posedImages[b]].C);
			if (nearestDistance > distance)
				nearestDistance = distance;
		}
		nearestDistances[a] = nearestDistance;
	});
	scene.threadPool.wait();
	return nearestDistances.GetMedian();
}

// Pose-guided pair scoring, shared by the candidate selection, the connectivity bridging
// and the verification-feedback round.
// The pairs whose optical axes diverge too much are rejected, as they can not observe the
// same surface; note: the two cameras observe the scene, not each other, so no camera-center
// cheirality test is done: in an orbit capture the neighboring camera centers lie tangentially
// to the view direction and in a nadir aerial capture perpendicularly to it, so the strongest
// overlapping pairs fail such a test.
// The baseline term peaks at optBaseline and decays symmetrically in log-scale towards
// no-parallax (baseline -> 0, no triangulation) and no-overlap (baseline -> infinity),
// while the view term linearly penalizes the diverging optical axes;
// note: the baseline is only a ranking preference and not a rejection criterion, as the
//  scene depth is unknown and the distance at which two views still overlap varies by
//  orders of magnitude in scene-scale units between close-range and aerial captures
struct PosePairScorer {
	const REAL sceneScale;                   // median nearest-neighbor camera-center distance
	const REAL maxViewAngle = D2R(REAL(75)); // maximum angle between the two optical axes (cameras facing away)
	const REAL optBaseline = 2;              // best scoring baseline, in scene-scale units

	struct Score {
		REAL distance;  // camera-center distance
		REAL viewAngle; // angle between the two optical axes
		REAL score;     // pair score, the higher the better; negative if the view-angle gate rejects the pair
		bool IsGated() const { return score < 0; }
	};

	// directionA must be imgA.Direction(), passed in so the caller can hoist it out of its loop
	Score operator()(const Image& imgA, const Point3& directionA, const Image& imgB) const {
		Score s;
		s.distance = norm(imgA.C - imgB.C);
		s.viewAngle = ACOS(CLAMP(directionA.dot(imgB.Direction()), REAL(-1), REAL(1)));
		s.score = s.viewAngle > maxViewAngle ?
			REAL(-1) : ScoreBaseline(s.distance) * (REAL(1) - s.viewAngle/maxViewAngle);
		return s;
	}

	REAL ScoreBaseline(REAL distance) const {
		const REAL baseline(distance / sceneScale);
		return REAL(2)*baseline*optBaseline / (SQUARE(baseline) + SQUARE(optBaseline));
	}

	// two-tier score for connectivity bridging: any pair passing the view-angle gate outranks
	// every rejected one, and the rejected (fallback) tier prefers the nearest cameras
	REAL BridgeScore(const Score& s) const {
		return s.IsGated() ?
			REAL(1) / (REAL(1) + s.distance/sceneScale) : // fallback tier, ordered by distance
			REAL(1) + s.score;                            // admissible tier, ordered by the pair score
	}
};

} // namespace

PairIdxArr PairsMatcher::CollectKnownPosePairs(unsigned topK)
{
	PairIdxArr result;
	const IIndex nImages = scene.images.size();
	const unsigned requestedTopK = topK;
	const IIndexArr posedImages = CollectPosedImages(scene);
	const IIndex nPosed = posedImages.size();
	if (nPosed < 2) {
		DEBUG("Pose-guided matching: only %u images have a known pose", nPosed);
		return result;
	}

	TD_TIMER_STARTD();

	// 1) Estimate the scene scale and initialize the shared pose-pair scoring
	const REAL sceneScale = EstimateSceneScale(scene, posedImages);
	if (sceneScale <= 0) {
		DEBUG("Pose-guided matching: degenerate camera configuration, all %u camera centers coincide", nPosed);
		return result;
	}
	const PosePairScorer scorer{sceneScale};

	// 2) Score every posed pair and keep the best candidates for each image
	topK = (unsigned)MINF(topK, nPosed - 1);
	const unsigned floorNN = MINF(2u, nPosed - 1); // per-image nearest cameras kept regardless of the view-angle gate
	struct ScoredImage {
		IIndex idx;  // index in the posed images array
		REAL score;  // pair score, the higher the better
	};
	std::vector<CLISTDEF0(ScoredImage)> topPerImage(nPosed);
	std::vector<CLISTDEF0(ScoredImage)> nearestPerImage(nPosed); // per-image nearest cameras by center distance, ungated
	std::atomic<unsigned> numRejectedByViewAngle{0};
	scene.threadPool.detach_loop(0u, nPosed, [&](IIndex a) {
		const Image& imgA = scene.images[posedImages[a]];
		const Point3 directionA(imgA.Direction());
		CLISTDEF0(ScoredImage)& scoredImages = topPerImage[a];
		scoredImages.reserve(nPosed - 1);
		CLISTDEF0(ScoredImage)& nearestImages = nearestPerImage[a];
		for (IIndex b = 0; b < nPosed; ++b) {
			if (b == a)
				continue;
			const Image& imgB = scene.images[posedImages[b]];
			const PosePairScorer::Score s = scorer(imgA, directionA, imgB);
			// track the nearest cameras with no gating: under occlusion (e.g. an indoor camera
			// turning back at the end of a corridor) ALL top covisible partners can exceed the
			// view-angle gate, and dropping them can split the view graph
			if (nearestImages.size() < floorNN) {
				nearestImages.emplace_back(ScoredImage{b, -s.distance});
				if (nearestImages.size() == floorNN)
					nearestImages.Sort([](const ScoredImage& i, const ScoredImage& j) {
						return i.score > j.score;
					});
			} else if (-s.distance > nearestImages.Last().score) {
				nearestImages.Last() = ScoredImage{b, -s.distance};
				for (IIndex n = nearestImages.size() - 1; n > 0 && nearestImages[n].score > nearestImages[n-1].score; --n)
					std::swap(nearestImages[n], nearestImages[n-1]);
			}
			if (s.IsGated()) {
				++numRejectedByViewAngle;
				continue;
			}
			scoredImages.emplace_back(ScoredImage{b, s.score});
		}
		if (scoredImages.size() > topK) {
			scoredImages.Sort([](const ScoredImage& i, const ScoredImage& j) {
				return i.score > j.score;
			});
			scoredImages.Resize(topK);
		}
	});
	scene.threadPool.wait();

	// 3) Keep the pairs present in the candidate lists of BOTH endpoints: requiring mutual
	//    agreement suppresses the one-sided tail of each list (candidates kept only because
	//    the other image sits in a denser part of the trajectory), which is what wastes most
	//    of the matching budget at small top-K settings
	std::unordered_map<PairIdx::PairIndex, uint8_t> numEndpointVotes;
	numEndpointVotes.reserve((size_t)nPosed * MINF(topK, 16u));
	for (IIndex a = 0; a < nPosed; ++a)
		for (const ScoredImage& scoredImage : topPerImage[a])
			++numEndpointVotes[MakePairIdx(a, scoredImage.idx).idx];
	std::unordered_set<PairIdx::PairIndex> setPairs; // pairs of indices in the posed images array
	setPairs.reserve(numEndpointVotes.size() / 2);
	for (const auto& [pairIndex, votes] : numEndpointVotes) {
		ASSERT(votes <= 2);
		if (votes == 2)
			setPairs.emplace(pairIndex);
	}
	const unsigned numMutualPairs = (unsigned)setPairs.size();

	// 4) Ungated nearest-camera floor: every image keeps its nearest cameras
	for (IIndex a = 0; a < nPosed; ++a)
		for (const ScoredImage& nearestImage : nearestPerImage[a])
			setPairs.emplace(MakePairIdx(a, nearestImage.idx).idx);
	const unsigned numFloorPairs = (unsigned)setPairs.size() - numMutualPairs;

	// 5) Connectivity backbone: join the connected components of the selected pair graph
	//    (Boruvka rounds over the two-tier bridge score: any admissible score outranks every
	//    rejected one, and rejected bridges prefer the nearest cameras), so a sparse or
	//    gate-fragmented selection cannot silently split the view graph
	IIndexArr ufParent(nPosed);
	std::iota(ufParent.begin(), ufParent.end(), 0u);
	const auto Find = [&ufParent](IIndex x) {
		while (ufParent[x] != x)
			x = ufParent[x] = ufParent[ufParent[x]];
		return x;
	};
	unsigned numComponents = nPosed;
	for (const PairIdx::PairIndex pairIndex : setPairs) {
		const PairIdx pair(pairIndex);
		const IIndex rootA = Find(pair.i), rootB = Find(pair.j);
		if (rootA != rootB) {
			ufParent[rootA] = rootB;
			--numComponents;
		}
	}
	struct Bridge {
		PairIdx::PairIndex pairIndex;
		REAL score;
	};
	while (numComponents > 1) {
		// one Boruvka round: find the best outgoing edge of each component and merge
		std::unordered_map<IIndex, Bridge> bestBridge; // component root -> best cross edge
		for (IIndex a = 0; a < nPosed; ++a) {
			const Image& imgA = scene.images[posedImages[a]];
			const Point3 directionA(imgA.Direction());
			const IIndex rootA = Find(a);
			for (IIndex b = a + 1; b < nPosed; ++b) {
				if (rootA == Find(b))
					continue;
				const Image& imgB = scene.images[posedImages[b]];
				const REAL bridgeScore = scorer.BridgeScore(scorer(imgA, directionA, imgB));
				const auto [it, inserted] = bestBridge.try_emplace(rootA, Bridge{MakePairIdx(a, b).idx, bridgeScore});
				if (!inserted && it->second.score < bridgeScore)
					it->second = Bridge{MakePairIdx(a, b).idx, bridgeScore};
			}
		}
		if (bestBridge.empty())
			break;
		for (const auto& [root, bridge] : bestBridge) {
			const PairIdx pair(bridge.pairIndex);
			const IIndex rootA = Find(pair.i), rootB = Find(pair.j);
			if (rootA != rootB) {
				ufParent[rootA] = rootB;
				--numComponents;
				setPairs.emplace(pair.idx);
			}
		}
	}

	// 6) Convert the selected posed-index pairs to image-ID pairs.
	// If the pose file covers only part of the image set, add vocabulary candidates touching
	// unposed images; otherwise those images would have no matches and the reconstruction
	// tail could never resect them. Fall back to those images' exhaustive pairs only if visual
	// retrieval is unavailable.
	result.reserve((IIndex)setPairs.size());
	std::unordered_set<PairIdx::PairIndex> selectedPairIDs;
	selectedPairIDs.reserve(setPairs.size());
	for (const PairIdx::PairIndex pairIndex : setPairs) {
		const PairIdx pair(pairIndex);
		const PairIdx imagePair(MakePairIdx(scene.images[posedImages[pair.i]].ID, scene.images[posedImages[pair.j]].ID));
		selectedPairIDs.emplace(imagePair.idx);
		result.emplace_back(imagePair);
	}
	unsigned numUnposedPairs = 0;
	if (nPosed < nImages) {
		std::unordered_set<IIndex> posedImageIDs;
		posedImageIDs.reserve(nPosed);
		for (const IIndex imageIdx : posedImages)
			posedImageIDs.emplace(scene.images[imageIdx].ID);
		PairIdxArr unposedCandidates = CollectVocabularyPairs(requestedTopK);
		if (unposedCandidates.empty()) {
			VERBOSE("warning: visual retrieval for the %u images without poses failed; matching every pair touching them",
				nImages - nPosed);
			for (IIndex i = 0; i < nImages; ++i)
				for (IIndex j = i + 1; j < nImages; ++j)
					if (!scene.images[i].HasPose() || !scene.images[j].HasPose())
						unposedCandidates.emplace_back(MakePairIdx(scene.images[i].ID, scene.images[j].ID));
		}
		for (const PairIdx& pair : unposedCandidates) {
			if (posedImageIDs.count(pair.i) && posedImageIDs.count(pair.j))
				continue;
			if (selectedPairIDs.emplace(pair.idx).second) {
				result.emplace_back(pair);
				++numUnposedPairs;
			}
		}
	}
	const unsigned numExhaustivePairs((nImages - 1) * nImages / 2);
	DEBUG("Pose-guided matching: %u candidate pairs from %u posed images (%u mutual top-%u, %u nearest-camera floor, %u connectivity bridges, %u visual pairs covering %u unposed images; %.2f/%u pairs/image, %.1f%% of the %u exhaustive pairs, %u rejected by the %.0fdeg view-angle test) in %s",
		(unsigned)result.size(), nPosed, numMutualPairs, topK, numFloorPairs, (unsigned)setPairs.size() - numMutualPairs - numFloorPairs,
		numUnposedPairs, nImages - nPosed, (float)result.size() / nImages, config.maxPairsPerImage,
		100.f * result.size() / numExhaustivePairs, numExhaustivePairs,
		numRejectedByViewAngle.load() / 2, R2D(scorer.maxViewAngle), TD_TIMER_GET_FMT().c_str());
	return result;
}

PairIdxArr PairsMatcher::CollectVerificationFeedbackPairs(const PairIdxArr& attemptedPairs)
{
	PairIdxArr result;
	ASSERT(config.mode == MatchConfig::VOCABULARY || config.mode == MatchConfig::KNOWN_POSES);
	const bool poseGuided(config.mode == MatchConfig::KNOWN_POSES);
	const IIndex nImages = scene.images.size();

	TD_TIMER_STARTD();

	// 1) Compute the pair budget left to invest: the total budget targets
	//    maxPairsPerImage*N/2 pairs over the full image set and the first round spent one
	//    attempt per candidate. In pose-guided mode, nEligible below still limits feedback
	//    proposals to posed images, but visual candidates for unposed images consume the same
	//    scene-wide budget.
	const IIndexArr posedImages = poseGuided ? CollectPosedImages(scene) : IIndexArr();
	const IIndex nEligible = poseGuided ? posedImages.size() : nImages;
	if (nEligible < 3)
		return result; // no pair outside the exhaustive set of a 2-image scene
	const size_t numTotalPairsBudget = MINF(
		(size_t)config.maxPairsPerImage*nImages/2, (size_t)nImages*(nImages - 1)/2);
	std::unordered_set<PairIdx::PairIndex> attempted;
	attempted.reserve(attemptedPairs.size() + scene.pairs.size());
	for (const PairIdx& pair : attemptedPairs)
		attempted.emplace(pair.idx);
	for (const ImagePair& pair : scene.pairs)
		attempted.emplace(MakePairIdx(pair.ID1, pair.ID2).idx);
	if (attempted.size() >= numTotalPairsBudget) {
		DEBUG("Verification-feedback matching: no pair budget left (%u pairs attempted of %u budgeted)",
			(unsigned)attempted.size(), (unsigned)numTotalPairsBudget);
		return result;
	}
	const size_t budget = numTotalPairsBudget - attempted.size();

	// 2) Build the geometrically verified pair graph of the previous matching round
	PairIdxArr verifiedPairs(0, scene.pairs.size());
	std::vector<IIndexArr> verifiedNeighbors(nImages);
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.HasMatches() || (config.maxEpipolarError > 0 && !pair.HasGeometricVerification()))
			continue;
		ASSERT(pair.ID1 < nImages && pair.ID2 < nImages);
		verifiedPairs.emplace_back(MakePairIdx(pair.ID1, pair.ID2));
		verifiedNeighbors[pair.ID1].push_back(pair.ID2);
		verifiedNeighbors[pair.ID2].push_back(pair.ID1);
	}
	if (verifiedPairs.empty()) {
		DEBUG("Verification-feedback matching: no verified pairs to build on");
		return result;
	}

	// 3) Propose and score new pairs from the verified graph
	struct Proposal {
		PairIdx::PairIndex pairIndex;
		uint32_t votes;  // verification-feedback strength (e.g. common verified neighbors)
		float score;     // first-round candidate score, breaking the vote ties
	};
	CLISTDEF0(Proposal) proposals;
	struct ScoredCandidate {
		IIndex idx;   // candidate image index
		float score;  // first-round candidate score, the higher the better
	};
	std::vector<CLISTDEF0(ScoredCandidate)> rankedPerImage(nImages); // refill candidates, best first
	const auto SortCandidates = [](CLISTDEF0(ScoredCandidate)& candidates) {
		candidates.Sort([](const ScoredCandidate& a, const ScoredCandidate& b) {
			return a.score > b.score || (a.score == b.score && a.idx < b.idx);
		});
	};
	if (poseGuided) {
		// close the triangles of the verified graph: two images sharing verified neighbors
		// most likely overlap too, no matter how the pose-based score ranks them (the strongest
		// signal available: it recovers the true pairs the view-angle gate or the baseline
		// preference mis-ranked in the first round)
		const REAL sceneScale = EstimateSceneScale(scene, posedImages);
		if (sceneScale <= 0)
			return result;
		const PosePairScorer scorer{sceneScale};
		std::unordered_map<PairIdx::PairIndex, uint32_t> numCommonNeighbors;
		for (IIndex i = 0; i < nImages; ++i) {
			const IIndexArr& neighbors = verifiedNeighbors[i];
			FOREACH(x, neighbors)
				for (IIndex y = x + 1; y < neighbors.size(); ++y) {
					const PairIdx::PairIndex pairIndex = MakePairIdx(neighbors[x], neighbors[y]).idx;
					if (attempted.find(pairIndex) == attempted.end())
						++numCommonNeighbors[pairIndex];
				}
		}
		proposals.reserve((IIndex)numCommonNeighbors.size());
		for (const auto& [pairIndex, votes] : numCommonNeighbors) {
			const PairIdx pair(pairIndex);
			const Image& imgA = scene.images[pair.i];
			const Image& imgB = scene.images[pair.j];
			if (!imgA.HasPose() || !imgB.HasPose())
				continue; // verified pairs may predate this matching round and involve unposed images
			const float score((float)scorer(imgA, imgA.Direction(), imgB).score);
			proposals.push_back({pairIndex, votes, MAXF(score, 0.f)});
		}
		// rank the refill candidates by the pose-based score (gated pairs excluded), deep
		// enough past the first-round candidate lists to always offer unattempted pairs
		if (proposals.size() < budget) {
			const IIndex refillDepth = MINF(config.maxPairsPerImage*4/3 + 8, nEligible - 1);
			scene.threadPool.detach_loop(0u, nEligible, [&](IIndex a) {
				const Image& imgA = scene.images[posedImages[a]];
				const Point3 directionA(imgA.Direction());
				CLISTDEF0(ScoredCandidate)& candidates = rankedPerImage[posedImages[a]];
				candidates.reserve(nEligible - 1);
				for (IIndex b = 0; b < nEligible; ++b) {
					if (b == a)
						continue;
					const PosePairScorer::Score s = scorer(imgA, directionA, scene.images[posedImages[b]]);
					if (!s.IsGated())
						candidates.push_back({posedImages[b], (float)s.score});
				}
				SortCandidates(candidates);
				if (candidates.size() > refillDepth)
					candidates.Resize(refillDepth);
			});
			scene.threadPool.wait();
		}
	} else {
		// propagate each verified pair to the top retrieval candidates of its endpoints:
		// a candidate retrieved high by one endpoint of a verified pair most likely overlaps
		// the other endpoint too (the retrieval analogue of closing verified triangles)
		if (fusedRetrievalScores.empty()) {
			DEBUG("Verification-feedback matching: no retrieval scores kept from the vocabulary round");
			return result;
		}
		for (const auto& [pairIndex, score] : fusedRetrievalScores) {
			const PairIdx pair(pairIndex);
			ASSERT(pair.i < nImages && pair.j < nImages);
			rankedPerImage[pair.i].push_back({pair.j, score});
			rankedPerImage[pair.j].push_back({pair.i, score});
		}
		for (CLISTDEF0(ScoredCandidate)& candidates : rankedPerImage)
			SortCandidates(candidates);
		const auto FusedScore = [this](IIndex a, IIndex b) {
			const auto it = fusedRetrievalScores.find(MakePairIdx(a, b).idx);
			return it != fusedRetrievalScores.end() ? it->second : 0.f;
		};
		const unsigned propagateK = 5; // retrieval candidates each verified endpoint propagates to
		std::unordered_map<PairIdx::PairIndex, float> propagatedScores;
		const auto Propagate = [&](IIndex a, IIndex b) {
			// propose the pairs (a, c) for the top retrieval candidates c of its verified partner b
			const CLISTDEF0(ScoredCandidate)& ranked = rankedPerImage[b];
			const IIndex topK = MINF((IIndex)propagateK, (IIndex)ranked.size());
			for (IIndex r = 0; r < topK; ++r) {
				const IIndex c = ranked[r].idx;
				if (c == a)
					continue;
				const PairIdx::PairIndex pairIndex = MakePairIdx(a, c).idx;
				if (attempted.find(pairIndex) != attempted.end())
					continue;
				const float score = FusedScore(a, c) + FusedScore(b, c);
				const auto [it, inserted] = propagatedScores.try_emplace(pairIndex, score);
				if (!inserted && it->second < score)
					it->second = score;
			}
		};
		for (const PairIdx& pair : verifiedPairs) {
			Propagate(pair.i, pair.j);
			Propagate(pair.j, pair.i);
		}
		proposals.reserve((IIndex)propagatedScores.size());
		for (const auto& [pairIndex, score] : propagatedScores)
			proposals.push_back({pairIndex, 1u, score});
	}

	// 4) Take the best proposals until the budget is spent
	proposals.Sort([](const Proposal& a, const Proposal& b) {
		if (a.votes != b.votes)
			return a.votes > b.votes;
		if (a.score != b.score)
			return a.score > b.score;
		return a.pairIndex < b.pairIndex;
	});
	for (const Proposal& proposal : proposals) {
		if (result.size() >= budget)
			break;
		attempted.emplace(proposal.pairIndex);
		result.emplace_back(PairIdx(proposal.pairIndex));
	}
	const unsigned numProposedPairs = (unsigned)result.size();

	// 5) Refill: the images with the weakest verified connectivity spend the remaining
	//    budget on their next best-ranked candidates from the first-round scoring
	if (result.size() < budget) {
		const unsigned maxRefillsPerImage = 2;
		IIndexArr weakestFirst(nEligible);
		FOREACH(i, weakestFirst)
			weakestFirst[i] = poseGuided ? posedImages[i] : i;
		weakestFirst.Sort([&verifiedNeighbors](IIndex a, IIndex b) {
			const IIndex degA = verifiedNeighbors[a].size(), degB = verifiedNeighbors[b].size();
			return degA < degB || (degA == degB && a < b);
		});
		for (const IIndex a : weakestFirst) {
			if (result.size() >= budget)
				break;
			unsigned numAdded = 0;
			for (const ScoredCandidate& candidate : rankedPerImage[a]) {
				const PairIdx::PairIndex pairIndex = MakePairIdx(a, candidate.idx).idx;
				if (!attempted.emplace(pairIndex).second)
					continue;
				result.emplace_back(PairIdx(pairIndex));
				if (++numAdded >= maxRefillsPerImage || result.size() >= budget)
					break;
			}
		}
	}

	DEBUG("Verification-feedback matching: %u candidate pairs (%u proposed by the %u verified pairs, %u refilled by the weakest-connected images; %u attempted of the %u pair budget) in %s",
		(unsigned)result.size(), numProposedPairs, verifiedPairs.size(), (unsigned)result.size() - numProposedPairs,
		(unsigned)(attempted.size() - result.size()), (unsigned)numTotalPairsBudget, TD_TIMER_GET_FMT().c_str());
	return result;
}

void PairsMatcher::OptimizePairsOrder(PairIdxArr& pairsToMatch)
{
	if (pairsToMatch.empty())
		return;
	// Build extended pair info with costs for sorting
	struct PairCostInfo {
		PairIdx pair;
		size_t cost;
	};
	CLISTDEF0(PairCostInfo) pairInfos;
	pairInfos.reserve(pairsToMatch.size());
	for (const PairIdx& pairIdx : pairsToMatch) {
		const Image& img1 = scene.images[pairIdx.i];
		const Image& img2 = scene.images[pairIdx.j];
		const size_t cost = (size_t)img1.descriptors.rows * img2.descriptors.rows;
		pairInfos.push_back({pairIdx, cost});
	}
	// Sort pairs primarily by image IDs to maximize GPU cache hits,
	// secondarily by cost (descending) for load balancing
	pairInfos.Sort([](const PairCostInfo& a, const PairCostInfo& b) {
		// Primary: group by first image ID (maximizes slot 0 GPU cache reuse)
		if (a.pair.i != b.pair.i)
			return a.pair.i < b.pair.i;
		// Secondary: within same first image, sort by cost descending
		return a.cost > b.cost;
	});
	// Reorder pairsToMatch according to optimized sort
	FOREACH(i, pairsToMatch)
		pairsToMatch[i] = pairInfos[i].pair;
}

void PairsMatcher::FilterRedundantKeypoints()
{
	TD_TIMER_STARTD();
	const IIndex numImages = scene.images.size();
	std::vector<Unsigned32Arr> remaps(numImages);
	const auto IsDuplicate = [](const cv::KeyPoint& ka, const cv::KeyPoint& kb) {
		constexpr float maxDistSq = 1e-2f; // 0.1 pixel distance squared
		return normSq(ka.pt - kb.pt) < maxDistSq;
	};

	// 1. Identify redundant keypoints per image
	std::atomic<size_t> atomicNumRemoved{0};
	std::atomic<size_t> atomicNumTotal{0};
	scene.threadPool.detach_loop(0u, numImages, [&](IIndex i) {
		Image& img = scene.images[i];
		const size_t numKPs = img.keypoints.size();
		if (numKPs == 0)
			return;
		// Sort keypoints by position -> response -> size
		std::vector<uint32_t> idxs(numKPs);
		std::iota(idxs.begin(), idxs.end(), 0);
		std::sort(idxs.begin(), idxs.end(), [&](uint32_t a, uint32_t b) {
			const cv::KeyPoint& ka = img.keypoints[a];
			const cv::KeyPoint& kb = img.keypoints[b];
			if (!IsDuplicate(ka, kb)) return ka.pt.x < kb.pt.x || (ka.pt.x == kb.pt.x && ka.pt.y < kb.pt.y);
			return ka.response*ka.size > kb.response*kb.size; // larger response*size first
		});
		// Identify duplicates
		Unsigned32Arr& remap = remaps[i];
		std::vector<cv::KeyPoint> newKeypoints;
		newKeypoints.reserve(numKPs);
		atomicNumTotal += numKPs;
		// We need to map old indices to new indices
		// Initialize remap with invalid value
		remap.assign(numKPs, NO_ID);
		for (size_t j = 0; j < numKPs; ) {
			const uint32_t bestIdx = idxs[j];
			const cv::KeyPoint& bestKP = img.keypoints[bestIdx];
			// This keypoint is kept
			const uint32_t newIdx = (uint32_t)newKeypoints.size();
			newKeypoints.push_back(bestKP);
			remap[bestIdx] = newIdx;
			// Skip all duplicates (they appear immediately after because of sort)
			while (++j < numKPs) {
				const uint32_t otherIdx = idxs[j];
				const cv::KeyPoint& otherKP = img.keypoints[otherIdx];
				// Check equality
				if (!IsDuplicate(otherKP, bestKP))
					break;
				// Map duplicate to the kept keypoint
				remap[otherIdx] = newIdx;
			}
		}
		if (newKeypoints.size() == numKPs) {
			// Clear remap to indicate no changes needed for this image
			remap.clear();
			return;
		}
		// Update keypoints
		ASSERT(img.descriptors.empty());
		img.keypoints = std::move(newKeypoints);
		atomicNumRemoved += (numKPs - img.keypoints.size());
	});
	scene.threadPool.wait();

	const size_t numRemoved = atomicNumRemoved.load();
	const size_t numTotal = atomicNumTotal.load();
	if (numRemoved == 0)
		return;

	// 2. Remap matches
	scene.threadPool.detach_loop(0u, scene.pairs.size(), [&](unsigned i) {
		ImagePair& pair = scene.pairs[i];
		const Unsigned32Arr& remap1 = remaps[pair.ID1];
		const Unsigned32Arr& remap2 = remaps[pair.ID2];
		if (!remap1.empty()) {
			for (auto& m : pair.matches) {
				ASSERT(static_cast<size_t>(m.queryIdx) < remap1.size());
				m.queryIdx = remap1[m.queryIdx];
				ASSERT(static_cast<size_t>(m.queryIdx) < scene.images[pair.ID1].keypoints.size());
			}
			for (auto& m : pair.outlierMatches) {
				ASSERT(static_cast<size_t>(m.queryIdx) < remap1.size());
				m.queryIdx = remap1[m.queryIdx];
				ASSERT(static_cast<size_t>(m.queryIdx) < scene.images[pair.ID1].keypoints.size());
			}
		}
		if (!remap2.empty()) {
			for (auto& m : pair.matches) {
				ASSERT(static_cast<size_t>(m.trainIdx) < remap2.size());
				m.trainIdx = remap2[m.trainIdx];
				ASSERT(static_cast<size_t>(m.trainIdx) < scene.images[pair.ID2].keypoints.size());
			}
			for (auto& m : pair.outlierMatches) {
				ASSERT(static_cast<size_t>(m.trainIdx) < remap2.size());
				m.trainIdx = remap2[m.trainIdx];
				ASSERT(static_cast<size_t>(m.trainIdx) < scene.images[pair.ID2].keypoints.size());
			}
		}
	});
	scene.threadPool.wait();

	// 3. Filter duplicate matches that arose from remapping
	// When multiple features at the same location matched different features in another image,
	// after remapping they become duplicate matches. Keep only the match with highest combined weight.
	std::atomic<size_t> atomicNumDuplicateMatches{0};
	scene.threadPool.detach_loop(0u, scene.pairs.size(), [&](unsigned i) {
		ImagePair& pair = scene.pairs[i];
		const Unsigned32Arr& remap1 = remaps[pair.ID1];
		const Unsigned32Arr& remap2 = remaps[pair.ID2];
		// Skip pairs where no remapping occurred
		if (remap1.empty() && remap2.empty())
			return;
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];

		// Helper to filter duplicates in a match vector (bidirectional check)
		const auto FilterDuplicates = [&](std::vector<DMatch>& matches) {
			unsigned numDuplicateMatches = 0;
			if (matches.empty())
				return numDuplicateMatches;

			// Lambda to remove duplicates by a specific index (queryIdx or trainIdx)
			const auto RemoveDuplicatesByIndex = [&](
				std::vector<DMatch>& data,
				auto getIndex) -> unsigned
			{
				unsigned numRemoved = 0;
				std::sort(data.begin(), data.end(), [&](const DMatch& a, const DMatch& b) {
					return getIndex(a) < getIndex(b);
				});

				std::vector<DMatch> filtered;
				filtered.reserve(data.size());
				for (size_t j = 0; j < data.size(); ) {
					const DMatch& first = data[j];
					const size_t start = j;
					const auto firstIndex = getIndex(first);
					while (++j < data.size() && getIndex(data[j]) == firstIndex);

					if (j - start == 1) {
						filtered.push_back(first);
					} else {
						// Multiple matches with same index - keep best by combined weight
						size_t bestIdx = start;
						float bestWeight = Image::ComputeKeypointWeight(img1.keypoints[first.queryIdx]) *
						                   Image::ComputeKeypointWeight(img2.keypoints[first.trainIdx]);
						for (size_t m = start + 1; m < j; ++m) {
							const float weight = Image::ComputeKeypointWeight(img1.keypoints[data[m].queryIdx]) *
							                     Image::ComputeKeypointWeight(img2.keypoints[data[m].trainIdx]);
							if (weight > bestWeight) {
								bestWeight = weight;
								bestIdx = m;
							}
						}
						filtered.push_back(data[bestIdx]);
						numRemoved += j - start - 1;
					}
				}
				data = std::move(filtered);
				return numRemoved;
			};

			// First pass: remove duplicates by queryIdx
			numDuplicateMatches += RemoveDuplicatesByIndex(matches, [](const DMatch& m) { return m.queryIdx; });

			// Second pass: remove duplicates by trainIdx
			numDuplicateMatches += RemoveDuplicatesByIndex(matches, [](const DMatch& m) { return m.trainIdx; });

			return numDuplicateMatches;
		};

		const unsigned numDuplicateMatches = FilterDuplicates(pair.matches);
		atomicNumDuplicateMatches += numDuplicateMatches + FilterDuplicates(pair.outlierMatches);
		if (pair.matches.size() < config.minMatches) {
			pair.InvalidateMatches();
		} else if (pair.numFilteredInliers > 0) {
			// Update the number of filtered inliers if applicable
			pair.numFilteredInliers -= (int)numDuplicateMatches;
			if (pair.numFilteredInliers < 0)
				pair.numFilteredInliers = pair.matches.size();
		}
	});
	scene.threadPool.wait();

	const size_t numDuplicateMatches = atomicNumDuplicateMatches.load();
	VERBOSE("Filtered %u redundant keypoints from %u total keypoints, removed %u duplicate matches (%s)",
		numRemoved, numTotal, numDuplicateMatches, TD_TIMER_GET_FMT().c_str());
}

void PairsMatcher::PreMatch(PairIdxArr& pairsToMatch)
{
	if (vocabularyTree == nullptr || !vocabularyTree->IsValid()) {
		VERBOSE("error: vocabulary tree not initialized for pre-matching");
		return;
	}
	TD_TIMER_STARTD();

	// Filter out already verified pairs from the list
	{
		size_t numAlreadyVerified = 0;
		RFOREACH(i, pairsToMatch) {
			const ImagePair* pPair = scene.FindPair(pairsToMatch[i].i, pairsToMatch[i].j);
			// Removing pair if it exists, has verification, and sufficient matches
			if (pPair && pPair->HasGeometricVerification() && pPair->GetNumFilteredInliers() >= config.minMatches) {
				pairsToMatch.RemoveAt(i);
				++numAlreadyVerified;
			}
		}
		if (numAlreadyVerified > 0)
			DEBUG("Pre-match: skipped %u pairs already geometrically verified", numAlreadyVerified);
	}
	DEBUG("Pre-matching %u pairs using top %u descriptors with threshold %u matches...",
		pairsToMatch.size(), vocabularyTree->GetMaxDescriptors(), config.preMatchThreshold);

	// Pre-match pairs
	cv::setNumThreads(1); // temporary turn off multi-threading for OpenCV functions
	size_t numVerifiedStored = 0;
	std::atomic<size_t> atomicNumRemoved{0};
	std::mutex pairsMutex;
	scene.threadPool.detach_loop(0u, pairsToMatch.size(), [&](unsigned idx) {
		PairIdx& p = pairsToMatch[idx];
		const Image& img1 = scene.images[p.i];
		const Image& img2 = scene.images[p.j];
		// Use cached top descriptors
		const cv::Mat& desc1 = vocabularyTree->GetTopDescriptors(img1);
		const cv::Mat& desc2 = vocabularyTree->GetTopDescriptors(img2);
		if (desc1.empty() || desc2.empty()) {
			p = PairIdx(NO_ID, NO_ID);
			++atomicNumRemoved;
			return;
		}
		const static thread_local unsigned threadIdx = std::hash<std::thread::id>{}(std::this_thread::get_id()) % matchers.size();
		std::vector<DMatch> matches;
		MatchFeatures(desc1, desc2, matches, threadIdx);
		if (matches.size() < config.preMatchThreshold) {
			p = PairIdx(NO_ID, NO_ID);
			++atomicNumRemoved;
			return;
		}
		// Geometric verification for PreMatch
		if (config.maxEpipolarError > 0) {
			ImagePair pair(p.i, p.j);
			pair.matches = std::move(matches);
			if (!GeometricFilter(img1, img2, pair)) {
				// Failed geometric verification
				p = PairIdx(NO_ID, NO_ID);
				++atomicNumRemoved;
				return;
			}
			// Succeeded: Store pair with geometry, but clear matches to force rematch
			pair.ResetMatches();
			{
				std::lock_guard<std::mutex> lock(pairsMutex);
				scene.pairs.emplace_back(std::move(pair));
				++numVerifiedStored;
			}
		}
	});
	scene.threadPool.wait();
	cv::setNumThreads(scene.nMaxThreads); // restore OpenCV threading

	const size_t numRemoved = atomicNumRemoved.load();
	// Prune invalid pairs
	if (numRemoved > 0) {
		PairIdxArr kept;
		kept.reserve(pairsToMatch.size() - numRemoved);
		for (const auto& p : pairsToMatch)
			if (p.i != NO_ID)
				kept.push_back(p);
		pairsToMatch = std::move(kept);
	}
	DEBUG("Pre-matching completed: %u pairs validated (%u verified & stored), %u removed (%s)",
		pairsToMatch.size(), numVerifiedStored, numRemoved, TD_TIMER_GET_FMT().c_str());
}

bool PairsMatcher::MatchPairsBatch(const PairIdxArr& pairsToMatch, LPCTSTR progressCaption, MatchStats& stats)
{
	if (pairsToMatch.empty())
		return true;
	// Hash map to quickly find existing pairs (or created by PreMatch)
	std::unordered_map<uint64_t, IIndex> existingPairMap;
	existingPairMap.reserve(scene.pairs.size());
	FOREACH(i, scene.pairs) {
		const ImagePair& p = scene.pairs[i];
		existingPairMap[PairIdx(p.ID1, p.ID2).idx] = i;
	}
	// Match all collected pairs in parallel
	cv::setNumThreads(1); // temporary turn off multi-threading for OpenCV functions
	Util::Progress progress(progressCaption, pairsToMatch.size());
	GET_LOGCONSOLE().Pause();
	unsigned newPairs = 0, updatedPairs = 0;
	size_t numMatches = 0,  numInliers = 0, numFilteredInliers = 0;

	#if defined(_USE_SIFTGPU) && defined(_USE_CUDA)
	// SiftGPU branch (only if using CUDA, even if SiftGPU can use GLSL for matching, is slower than modern CPU)
	if (scene.status.nFeaturesType == FeatureType::SIFTGPU) {
		SiftGPUMatchCoordinator coordinator(*this);
		if (!coordinator.Initialize()) {
			VERBOSE("error: SiftMatchGPU coordinator initialization failed");
			GET_LOGCONSOLE().Play();
			progress.close();
			cv::setNumThreads(scene.nMaxThreads);
			return false;
		}
		coordinator.ProcessPairs(pairsToMatch, existingPairMap, progress, newPairs, updatedPairs, numMatches, numInliers, numFilteredInliers);
	} else
	#endif // _USE_SIFTGPU
	{
		// Standard CPU / other descriptor matching
		std::atomic<unsigned> atomicNewPairs{0};
		std::atomic<unsigned> atomicUpdatedPairs{0};
		std::atomic<size_t> atomicNumMatches{0};
		std::atomic<size_t> atomicNumInliers{0};
		std::atomic<size_t> atomicNumFilteredInliers{0};
		std::mutex pairsMutex;
		scene.threadPool.detach_loop<size_t>(0, pairsToMatch.size(), [&](size_t idxPair) {
			const PairIdx pairIDs = pairsToMatch[idxPair];
			// Skip if pair already exists with geometric data AND matches
			ImagePair pair(pairIDs.i, pairIDs.j);
			IIndex existingIdx = NO_ID; {
				// Try to find existing pair (e.g. from PreMatch) to reuse geometry
				auto it = existingPairMap.find(pairIDs.idx);
				if (it != existingPairMap.end()) {
					std::lock_guard<std::mutex> lock(pairsMutex);
					// Check if geometric verification was done
					ImagePair& existingPair = scene.pairs[it->second];
					if (!existingPair.HasMatches() || (config.maxEpipolarError > 0 && !existingPair.HasGeometricVerification())) {
						// Preserve the stored pair until rematching succeeds; a failed rematch must not
						// leave the scene entry moved-from.
						pair = existingPair;
						existingIdx = it->second;
					}
					if (existingIdx == NO_ID) {
						++progress;
						return;
					}
				}
			}
			// Match pair and perform geometric verification
			if (MatchPair(scene.images[pair.ID1], scene.images[pair.ID2], pair)) {
				atomicNumMatches += pair.GetNumMatches();
				atomicNumInliers += pair.GetNumInliers();
				atomicNumFilteredInliers += pair.GetNumFilteredInliers();
				std::lock_guard<std::mutex> lock(pairsMutex);
				if (existingIdx != NO_ID) {
					// Update existing pair in place
					scene.pairs[existingIdx] = std::move(pair);
					atomicUpdatedPairs++;
				} else {
					// Add new pair
					scene.pairs.emplace_back(std::move(pair));
					atomicNewPairs++;
				}
			}
			++progress;
		});
		scene.threadPool.wait();

		newPairs = atomicNewPairs.load();
		updatedPairs = atomicUpdatedPairs.load();
		numMatches = atomicNumMatches.load();
		numInliers = atomicNumInliers.load();
		numFilteredInliers = atomicNumFilteredInliers.load();
	}
	GET_LOGCONSOLE().Play();
	progress.close();
	cv::setNumThreads(scene.nMaxThreads); // restore OpenCV threading

	stats.newPairs += newPairs;
	stats.updatedPairs += updatedPairs;
	stats.numMatches += numMatches;
	stats.numInliers += numInliers;
	stats.numFilteredInliers += numFilteredInliers;
	return true;
}

unsigned PairsMatcher::Match()
{
	const IIndex nImages = scene.images.size();
	if (nImages < 2) {
		VERBOSE("error: need at least 2 images for matching");
		return 0;
	}

	TD_TIMER_STARTD();
	const MatchConfig::MatchMode matchMode(
		config.mode == MatchConfig::VOCABULARY && nImages < config.maxPairsPerImage*6/5 ?
		MatchConfig::EXHAUSTIVE : config.mode);

	// Two-round verification feedback: hold back part of the pair budget in the first round
	// and re-invest it in the pairs suggested by the geometrically verified matches; engaged
	// only for the selective modes, and only when the budget is large enough for the
	// first-round verified graph to carry a useful signal
	bool verificationFeedback = config.verificationFeedback && config.maxPairsPerImage >= 10 &&
		(matchMode == MatchConfig::VOCABULARY || matchMode == MatchConfig::KNOWN_POSES);
	// Per-image candidate-list length for the selective modes: in single-round matching the
	// list is inflated because the mutual-agreement rule is stricter than a one-sided top-K
	// union (at the configured setting it then selects about the configured volume of pairs,
	// just distributed by two-sided preference); with verification feedback the first round
	// instead uses a deflated list (80% of the target, uninflated) and the second round
	// fills the rest of the maxPairsPerImage*N/2 pair budget guided by the verified matches
	const unsigned vocabularyTopK = verificationFeedback ?
		config.maxPairsPerImage*4/5 : config.maxPairsPerImage*8/5;
	const unsigned knownPosesTopK = verificationFeedback ?
		config.maxPairsPerImage*4/5 : config.maxPairsPerImage*4/3;

	// Collect pairs to match based on selected mode
	PairIdxArr pairsToMatch;
	const unsigned numExhaustivePairs((nImages - 1) * nImages / 2);
	const auto CollectExhaustivePairs = [&]() {
		VERBOSE("Exhaustive matching %u images...", nImages);
		pairsToMatch.reserve(numExhaustivePairs);
		for (IIndex i = 0; i < nImages; ++i)
			for (IIndex j = i + 1; j < nImages; ++j)
				pairsToMatch.emplace_back(MakePairIdx(scene.images[i].ID, scene.images[j].ID));
	};
	switch (matchMode) {
	case MatchConfig::EXHAUSTIVE: {
		CollectExhaustivePairs();
		break;
	}
	case MatchConfig::VOCABULARY: {
		// Build vocabulary candidates from the tree retrieval ranking
		pairsToMatch = CollectVocabularyPairs(vocabularyTopK);
		if (pairsToMatch.empty()) {
			VERBOSE("error: vocabulary produced no new candidate pairs");
			return 0;
		}
		break;
	}
	case MatchConfig::KNOWN_POSES: {
		// Select the candidate pairs using the already-known camera poses
		pairsToMatch = CollectKnownPosePairs(knownPosesTopK);
		if (pairsToMatch.empty()) {
			VERBOSE("warning: known poses produced no candidate pairs (less than two posed images or degenerate poses); falling back to exhaustive matching");
			CollectExhaustivePairs();
			verificationFeedback = false;
		}
		break;
	}
	case MatchConfig::SEQUENTIAL: {
		// Sequential matching: consecutive images only
		// Close the loop: include pairs that wrap from the end to the front;
		// makes sense only when nImages >= 2*overlap, and to avoid duplicates
		VERBOSE("Sequential matching %u images (overlap: %u)...", nImages, config.matchSequenceOverlap);
		pairsToMatch.reserve(nImages * config.matchSequenceOverlap);
		if (nImages >= 2 * config.matchSequenceOverlap) {
			for (IIndex i = 0; i < nImages; ++i)
				for (unsigned k = 1; k <= config.matchSequenceOverlap; ++k) {
					if (2*k == nImages && i >= nImages/2)
						continue; // diametrically-opposite pair, already emitted from the other endpoint
					pairsToMatch.emplace_back(MakePairIdx(scene.images[i].ID, scene.images[(i + k) % nImages].ID));
				}
		} else {
			for (IIndex i = 0; i < nImages; ++i)
				for (unsigned k = 1; k <= config.matchSequenceOverlap; ++k)
					if (i + k < nImages)
						pairsToMatch.emplace_back(MakePairIdx(scene.images[i].ID, scene.images[i + k].ID));
		}
		break;
	}
	default:
		ASSERT("Invalid match mode" == NULL);
		return 0;
	}
	ASSERT(!pairsToMatch.empty());

	// Run a matching round: pre-match filter if requested, GPU-friendly ordering, then
	// parallel feature matching and geometric verification of all the candidate pairs
	MatchStats stats;
	const auto MatchRound = [&](PairIdxArr& pairs, LPCTSTR progressCaption) {
		// Pre-match the pairs if requested
		// Pre-matching needs the vocabulary-tree descriptors, so it runs only when candidate
		// collection built the tree (VOCABULARY, or KNOWN_POSES with unposed images).
		if (vocabularyTree) {
			if (config.preMatchThreshold > 0)
				PreMatch(pairs);
			// Clear descriptors cache
			vocabularyTree->ClearDescriptorsCache();
		}
		// Reorder pairs to minimize GPU transfers and improve load balancing
		OptimizePairsOrder(pairs);
		return MatchPairsBatch(pairs, progressCaption, stats);
	};
	if (!MatchRound(pairsToMatch, _T("Match image pairs")))
		return 0;

	// Second round: spend the held-back pair budget on the pairs suggested by the
	// geometrically verified matches of the first round
	if (verificationFeedback) {
		PairIdxArr feedbackPairs = CollectVerificationFeedbackPairs(pairsToMatch);
		if (!feedbackPairs.empty() && !MatchRound(feedbackPairs, _T("Match feedback pairs")))
			return 0;
	}
	fusedRetrievalScores.clear(); // only kept for the verification-feedback round

	const unsigned numProcessedPairs = stats.newPairs + stats.updatedPairs;
	DEBUG("Images matched: created %u/%u new/updated pairs (%u total from %u exhaustive),\n%u/%u/%u matches (%.2f/%.2f/%.2f per pair) in %s",
		stats.newPairs, stats.updatedPairs, scene.pairs.size(), numExhaustivePairs, stats.numFilteredInliers, stats.numInliers, stats.numMatches,
		numProcessedPairs ? static_cast<double>(stats.numFilteredInliers) / numProcessedPairs : 0.0,
		numProcessedPairs ? static_cast<double>(stats.numInliers) / numProcessedPairs : 0.0,
		numProcessedPairs ? static_cast<double>(stats.numMatches) / numProcessedPairs : 0.0,
		TD_TIMER_GET_FMT().c_str());

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		// Log pairs statistics:
		//  - number of valid pairs per image
		//  - number of matches and inliers per pair
		MeanStdMinMax<unsigned,REAL> pairsPerImage;
		MeanStdMinMax<unsigned,REAL> matchesPerPair;
		MeanStdMinMax<unsigned,REAL> inliersPerPair;
		MeanStdMinMax<unsigned,REAL> filteredInliersPerPair;
		IIndexArr imagePairCounts(scene.images.size());
		imagePairCounts.Memset(0);
		unsigned nMatches = 0, nInliers = 0, nFilteredInliers = 0;
		for (const ImagePair& pair : scene.pairs) {
			if (!pair.HasMatches())
				continue;
			++imagePairCounts[pair.ID1];
			++imagePairCounts[pair.ID2];
			matchesPerPair.Update(pair.GetNumMatches());
			inliersPerPair.Update(pair.GetNumInliers());
			filteredInliersPerPair.Update(pair.GetNumFilteredInliers());
			nMatches += pair.GetNumMatches();
			nInliers += pair.GetNumInliers();
			nFilteredInliers += pair.GetNumFilteredInliers();
		}
		pairsPerImage.Compute(imagePairCounts.data(), imagePairCounts.size());
		VERBOSE("Pairs per image: mean %.2f, std %.2f, range [%u, %u]",
				pairsPerImage.GetMean(), pairsPerImage.GetStdDev(),
				pairsPerImage.GetMin(), pairsPerImage.GetMax());
		VERBOSE("Matches per pair: mean %.2f, std %.2f, range [%u, %u], total %u",
				matchesPerPair.GetMean(), matchesPerPair.GetStdDev(),
				matchesPerPair.GetMin(), matchesPerPair.GetMax(), nMatches);
		if (nInliers > 0) {
			VERBOSE("Inliers per pair: mean %.2f, std %.2f, range [%u, %u], total %u",
					inliersPerPair.GetMean(), inliersPerPair.GetStdDev(),
					inliersPerPair.GetMin(), inliersPerPair.GetMax(), nInliers);
		}
		if (nFilteredInliers > 0) {
			VERBOSE("Filtered inliers per pair: mean %.2f, std %.2f, range [%u, %u], total %u",
					filteredInliersPerPair.GetMean(), filteredInliersPerPair.GetStdDev(),
					filteredInliersPerPair.GetMin(), filteredInliersPerPair.GetMax(), nFilteredInliers);
		}
	}
	#endif

	if (config.releaseDescriptors) {
		// Release descriptors to save memory
		for (Image& img : scene.images)
			img.descriptors.release();

		// Filter redundant keypoints to avoid artificial track breaks
		// (only if descriptors are released as the filter can not process descriptors)
		FilterRedundantKeypoints();
	}

	// Compute pair weights
	ComputePairsWeights(scene, config.weightingCfg);

	return scene.pairs.size();
}


bool PairsMatcher::ExportPairsCSV(const Scene& scene, const String& fileName, float minWeight)
{
	std::ofstream ofs(fileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", fileName.c_str());
		return false;
	}
	const String basePath = MAKE_PATH_FULL(WORKING_FOLDER_FULL, Util::getFilePath(fileName));
	ofs << "ImageA,ImageB,NumMatches,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle\n";
	for (const ImagePair& pair : scene.pairs) {
		const String relImageNameA = MAKE_PATH_REL(basePath, scene.images[pair.ID1].fileName);
		const String relImageNameB = MAKE_PATH_REL(basePath, scene.images[pair.ID2].fileName);
		ofs << relImageNameA << "," << relImageNameB << ","
		    << pair.GetNumFilteredInliers() << ","
			<< pair.GetCompositeWeight() << ","
			<< pair.weightSpatial << ","
			<< pair.weightConnectivity << ","
			<< pair.weightTriplet << ","
			<< R2D(pair.meanRayAngle) << "\n";
	}
	ofs.close();
	VERBOSE("Exported %u pairs to '%s'",
		(unsigned)scene.pairs.size(), fileName.c_str());
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
