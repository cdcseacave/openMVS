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
					ImagePair& p = scene.pairs[it->second];
					if (!p.HasMatches() || (pairsMatcher.GetConfig().maxEpipolarError > 0 && !p.HasGeometricVerification())) {
						existingPair = std::move(p);
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
					// Store pair into scene
					if (existingIdx != NO_ID) {
						pairsMatcher.GetScene().pairs[existingIdx] = std::move(pair);
						atomicUpdatedPairs.fetch_add(1, std::memory_order_relaxed);
					} else {
						std::lock_guard<std::mutex> lock(sceneMutex);
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
		// limit (previous hand-rolled scaling). RelativePoseOptions stores this
		// angle directly in opt.max_error (Sampson residual is radians-scaled).
		const REAL pxErr = opt.max_error;
		const REAL angle1 = cam1.PixelErrorToAngular(pxErr);
		const REAL angle2 = cam2.PixelErrorToAngular(pxErr);
		opt.SetMaxErrorFromAngle(0.5 * (angle1 + angle2));

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

PairIdxArr PairsMatcher::CollectVocabularyPairs(unsigned* ptrNumBasePairs)
{
	const unsigned topN = config.maxPairsPerImage;
	const unsigned topK = config.expandPairsTopK;
	unsigned numBasePairs = 0;
	PairIdxArr result;
	// Ensure vocabulary tree is ready
	EnsureVocabularyTree();
	if (!vocabularyTree || !vocabularyTree->IsValid())
		return result;

	TD_TIMER_STARTD();

	// 1) Query top-N similar images for each image (without matching)
	const IIndex nImages = scene.images.size();
	std::vector<IIndexArr> topPerImage(nImages);
	scene.threadPool.detach_loop(0u, nImages, [&](IIndex i) {
		const Image& img = scene.images[i];
		const auto candidates = vocabularyTree->Query(img, topN);
		topPerImage[i].reserve(candidates.size());
		for (const auto& kv : candidates)
			if (kv.first != img.ID) // skip self-match
				topPerImage[i].push_back(kv.first);
	});
	scene.threadPool.wait();

	// 2) Build base set from union of all top-N lists (for exclusion),
	//    and base vector of pairs to actually match (skip existing pairs).
	std::unordered_set<PairIdx::PairIndex> setPairs; // union of all retrieved pairs
	setPairs.reserve((size_t)nImages * MINF(topN, 8u));
	for (IIndex i = 0; i < nImages; ++i) {
		const IIndex idA = scene.images[i].ID;
		for (const IIndex idB : topPerImage[i]) {
			ASSERT(idB != idA);
			const PairIdx p = MakePairIdx(idA, idB);
			// For base result vector, add on first encounter
			if (setPairs.emplace(p.idx).second)
				result.emplace_back(p);
		}
	}
	numBasePairs = (unsigned)result.size();
	if (ptrNumBasePairs)
		*ptrNumBasePairs = numBasePairs;
	if (topK == 0 || numBasePairs == 0) {
		DEBUG("Vocabulary-based matching: %u base candidate pairs (%.2f/%u pairs/image) in %s",
			numBasePairs, (float)numBasePairs / nImages, config.maxPairsPerImage, TD_TIMER_GET_FMT().c_str());
		return result;
	}

	// 3) Expand using top-K neighbors per endpoint for each base pair, if requested
	for (IIndex i = 0; i < nImages; ++i) {
		const IIndex idA = scene.images[i].ID;
		const auto& listA = topPerImage[idA];
		const unsigned kA = MINF(topK, (unsigned)listA.size());
		for (unsigned a = 0; a < kA; ++a) {
			const IIndex idB = listA[a];
			const auto& listB = topPerImage[idB];
			const unsigned kB = MINF(topK, (unsigned)listB.size());
			// Combine endpoints with each other's top-K
			for (unsigned b = 0; b < kB; ++b) {
				const IIndex idN = listB[b];
				if (idA == idN)
					continue; // skip self-match;
				const PairIdx q = MakePairIdx(idA, idN);
				// Skip if pair already exists
				if (setPairs.emplace(q.idx).second)
					result.emplace_back(q);
			}
		}
	}
	DEBUG("Vocabulary-based matching: %u base candidate pairs, with %u expanded candidates (%.2f/%u pairs/image) in %s",
		numBasePairs, result.size() - numBasePairs, (float)result.size() / nImages, config.maxPairsPerImage, TD_TIMER_GET_FMT().c_str());
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

	// Collect pairs to match based on selected mode
	PairIdxArr pairsToMatch;
	const unsigned numExhaustivePairs((nImages - 1) * nImages / 2);
	switch (matchMode) {
	case MatchConfig::EXHAUSTIVE: {
		VERBOSE("Exhaustive matching %u images...", nImages);
		pairsToMatch.reserve(numExhaustivePairs);
		for (IIndex i = 0; i < nImages; ++i)
			for (IIndex j = i + 1; j < nImages; ++j)
				pairsToMatch.emplace_back(MakePairIdx(scene.images[i].ID, scene.images[j].ID));
		break;
	}
	case MatchConfig::VOCABULARY: {
		// Build vocabulary candidates (base + optional expanded pairs)
		pairsToMatch = CollectVocabularyPairs();
		if (pairsToMatch.empty()) {
			VERBOSE("error: vocabulary produced no new candidate pairs");
			return 0;
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
				for (unsigned k = 1; k <= config.matchSequenceOverlap; ++k)
					pairsToMatch.emplace_back(MakePairIdx(scene.images[i].ID, scene.images[(i + k) % nImages].ID));
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

	// Make pre-matching if requested
	if (vocabularyTree) {
		if (config.preMatchThreshold > 0)
			PreMatch(pairsToMatch);
		// Clear descriptors cache
		vocabularyTree->ClearDescriptorsCache();
	}

	// Reorder pairs to minimize GPU transfers and improve load balancing
	OptimizePairsOrder(pairsToMatch);

	// Hash map to quickly find existing pairs (or created by PreMatch)
	std::unordered_map<uint64_t, IIndex> existingPairMap;
	existingPairMap.reserve(scene.pairs.size());
	FOREACH(i, scene.pairs) {
		const ImagePair& p = scene.pairs[i];
		existingPairMap[PairIdx(p.ID1, p.ID2).idx] = i;
	}
	// Match all collected pairs in parallel
	cv::setNumThreads(1); // temporary turn off multi-threading for OpenCV functions
	Util::Progress progress(_T("Match image pairs"), pairsToMatch.size());
	GET_LOGCONSOLE().Pause();
	unsigned newPairs = 0, updatedPairs = 0;
	size_t numMatches = 0,  numInliers = 0, numFilteredInliers = 0;

	#if defined(_USE_SIFTGPU) && defined(_USE_CUDA)
	// SiftGPU branch (only if using CUDA, even if SiftGPU can use GLSL for matching, is slower than modern CPU)
	if (scene.status.nFeaturesType == FeatureType::SIFTGPU) {
		SiftGPUMatchCoordinator coordinator(*this);
		if (!coordinator.Initialize()) {
			VERBOSE("error: SiftMatchGPU coordinator initialization failed");
			return 0;
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
						// Move existing pair data (Pose/F) but keep matches clean/empty if they were cleared
						pair = std::move(existingPair);
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

	DEBUG("Images matched: created %u/%u new/updated pairs (%u total from %u exhaustive),\n%u/%u/%u matches (%.2f/%.2f/%.2f per pair) in %s",
		newPairs, updatedPairs, scene.pairs.size(), numExhaustivePairs, numFilteredInliers, numInliers, numMatches,
		static_cast<double>(numFilteredInliers) / (newPairs + updatedPairs), static_cast<double>(numInliers) / (newPairs + updatedPairs), static_cast<double>(numMatches) / (newPairs + updatedPairs),
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
	ofs << "ImageA,ImageB,NumMatches,Weight\n";
	for (const ImagePair& pair : scene.pairs) {
		const String relImageNameA = MAKE_PATH_REL(basePath, scene.images[pair.ID1].fileName);
		const String relImageNameB = MAKE_PATH_REL(basePath, scene.images[pair.ID2].fileName);
		ofs << relImageNameA << "," << relImageNameB << ","
		    << pair.GetNumFilteredInliers() << ","
			<< pair.GetCompositeWeight() << "\n";
	}
	ofs.close();
	VERBOSE("Exported %u pairs to '%s'",
		(unsigned)scene.pairs.size(), fileName.c_str());
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
