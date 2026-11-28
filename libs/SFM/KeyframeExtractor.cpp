////////////////////////////////////////////////////////////////////
// KeyframeExtractor.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "KeyframeExtractor.h"
#include "BundleAdjustment.h"
#include "FeaturesExtractor.h"
#include "MatchGeometric.h"
#include "PairsWeighting.h"
#include "RelativePoseRefine.h"
#include "StarInitializer.h"
#include "ViewGraphCalibrator.h"
#include <opencv2/video/tracking.hpp>

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM::KeyframeExtractor {

// Compute overlap ratio based on tracked features
float ComputeFeatureOverlap(
	const std::vector<Point2f>& prevPoints,
	const std::vector<Point2f>& currPoints,
	const std::vector<uchar>& status,
	const cv::Size& imageSize)
{
	ASSERT(!prevPoints.empty() && prevPoints.size() == currPoints.size() && prevPoints.size() == status.size());
	// Count successfully tracked features
	int trackedCount = 0;
	FOREACH(i, status) {
		if (!status[i])
			continue;
		// Check if point is still within image bounds
		const cv::Point2f& pt = currPoints[i];
		if (pt.x >= 0 && pt.x < imageSize.width &&
			pt.y >= 0 && pt.y < imageSize.height)
			++trackedCount;
	}
	return static_cast<float>(trackedCount) / prevPoints.size();
}

// Compute overlap area using homography (pinhole) or angular displacement (spherical)
float ComputeHomographyOverlap(
	const std::vector<Point2f>& prevPoints,
	const std::vector<Point2f>& currPoints,
	const std::vector<uchar>& status,
	const cv::Size& imageSize,
	const Camera& camera)
{
	if (camera.GetType() == CameraType::SPHERICAL) {
		// Spherical path: compute median angular displacement on the unit sphere.
		// Bearings are obtained from equirectangular pixel coords via Unproject.
		// Result mapped to [0,1] via exp(-medianAngle / (pi/4)) so the existing
		// overlapThreshold (0.85) has compatible semantics.
		REALArr angles(0, status.size());
		FOREACH(i, status) {
			if (!status[i])
				continue;
			const Point3 b1 = camera.UnprojectNormalized(Cast<REAL>(prevPoints[i]));
			const Point3 b2 = camera.UnprojectNormalized(Cast<REAL>(currPoints[i]));
			angles.push_back(CLAMP(b1.dot(b2), REAL(-1), REAL(1)));
		}
		if (angles.empty())
			return 0.f;
		const REAL medianAngle = ACOS(angles.GetMedian());
		return static_cast<float>(EXP(-medianAngle / (REAL(M_PI) / 4)));
	}

	// Pinhole path: homography-based overlap area
	std::vector<Point2f> prevInliers, currInliers;
	FOREACH(i, status) {
		if (status[i]) {
			prevInliers.push_back(prevPoints[i]);
			currInliers.push_back(currPoints[i]);
		}
	}
	if (prevInliers.size() < 4)
		return 0.f;

	// Estimate homography using RANSAC
	cv::Mat H = cv::findHomography(prevInliers, currInliers, cv::RANSAC, 3.0, cv::noArray(), 2000, 0.999);
	if (H.empty())
		return 0.f;

	// Define corners of the previous image
	std::vector<Point2f> corners(4);
	corners[0] = cv::Point2f(0, 0);
	corners[1] = cv::Point2f((float)imageSize.width-1, 0);
	corners[2] = cv::Point2f((float)imageSize.width-1, (float)imageSize.height-1);
	corners[3] = cv::Point2f(0, (float)imageSize.height-1);

	// Transform corners using homography
	std::vector<Point2f> transformedCorners(4);
	cv::perspectiveTransform(corners, transformedCorners, H);

	// Compute actual polygon intersection area
	const float intersectionArea = (float)cv::intersectConvexConvex(transformedCorners, corners, cv::noArray());
	const float imageArea = static_cast<float>(imageSize.area());
	return intersectionArea / imageArea;
}

// Small cache entry to keep recent frames and their tracked positions/status
struct CachedFrame {
	cv::Mat frame; // color
	cv::Mat gray;  // grayscale
	std::vector<Point2f> trackedPoints;
	std::vector<uchar> status;
	uint32_t frameIdx = 0; // index of the frame in decoding order (0-based)
	double timestamp = 0; // presentation time of the frame in the video (seconds)
	float overlapRatio = 1.f;
	float overlapArea = 1.f;
	float sharpness = 0.f;
};



// Keyframe post-processing event: save image and run matching (runs in background worker)
class KeyframePostProcessEvent : public SEACAVE::Event {
public:
	IIndex prevID;
	IIndex currID;
	Scene& scene;
	PairsMatcher& pairsMatcher;
	const Camera& cam;
	bool focalKnown;
	std::vector<Point2f> trackedPrevPts;
	std::vector<Point2f> trackedCurrPts;
	std::vector<uchar> trackedStatus;
	float overlapRatio;
	float overlapArea;
	SEACAVE::CriticalSection& pcs;
	DoubleArr& focalEstimates;
	DoubleArr& k1Estimates;
	DoubleArr& k2Estimates;
	bool refineTwoViewCalibration;

	KeyframePostProcessEvent(IIndex aPrev, IIndex aCurr, Scene& s, PairsMatcher& pm, const Camera& camRef, bool fk,
							 const std::vector<Point2f>& tp, const std::vector<Point2f>& tc,
							 const std::vector<uchar>& st, float oratio, float oarea,
							 SEACAVE::CriticalSection& _pcs, DoubleArr& fe, DoubleArr& k1e, DoubleArr& k2e, bool refine)
		: SEACAVE::Event(1), prevID(aPrev), currID(aCurr), scene(s), pairsMatcher(pm), cam(camRef), focalKnown(fk),
		  trackedPrevPts(tp), trackedCurrPts(tc), trackedStatus(st), overlapRatio(oratio), overlapArea(oarea),
		  pcs(_pcs), focalEstimates(fe), k1Estimates(k1e), k2Estimates(k2e), refineTwoViewCalibration(refine) {}

	bool Run(void* = NULL) override {
		pcs.Enter();
		// Save current image to disk (use copy so we don't hold lock during IO)
		Image& currImg = scene.images[currID];
		currImg.SavePixels();
		if (prevID == NO_ID) {
			// No previous keyframe to match against
			pcs.Leave();
			return true;
		}

		// Run matching
		Image& prevImg = scene.images[prevID];
		ImagePair& pair = scene.pairs.emplace_back(prevID, currID);
		[[maybe_unused]] const bool geometryEstimated = MatchFeaturesGeometric(
			pairsMatcher,
			prevImg,
			currImg,
			trackedPrevPts, trackedCurrPts, trackedStatus,
			pair);
		pair.overlapRatio = overlapRatio;
		pair.overlapArea = overlapArea;
		if (!pair.matches.empty()) {
			ASSERT(geometryEstimated);
			// Refine calibration if we don't trust intrinsics yet and distortion not set
			if (refineTwoViewCalibration && cam.GetType() == CameraType::PINHOLE && !static_cast<const PinholeCamera&>(cam).HasDistortion() &&
				(pair.relativePose.has_value() || pairsMatcher.DecomposeFundamentalToPose(prevImg, currImg, pair))) {
				RelativePoseRefine::Config calibCfg;
				calibCfg.refineFocalLength = !focalKnown;
				RelativePoseRefine::Result calibRes;
				std::unique_ptr<PinholeCamera> pCamCopy(static_cast<PinholeCamera*>(cam.Clone()));
				if (RelativePoseRefine::RefineTwoViewCalibration(
						prevImg.keypoints, currImg.keypoints, pair.matches,
						*pCamCopy, pair.relativePose.value(), calibCfg, &calibRes))
				{
					focalEstimates.push_back(pCamCopy->fx);
					k1Estimates.push_back(pCamCopy->k1);
					k2Estimates.push_back(pCamCopy->k2);
					DEBUG_ULTIMATE("Refined calibration pair %d-%d: f=%.2f k1=%.6f k2=%.6f (cost %.2f->%.2f)",
						prevID, currID, pCamCopy->fx, pCamCopy->k1, pCamCopy->k2, calibRes.initialCost, calibRes.finalCost);
				}
			}

			DEBUG_EXTRA("Created pair between keyframes %d and %d with %u matches",
				prevID, currID, pair.GetNumFilteredInliers());
		}


		// Release pixels in shared scene storage to free memory
		currImg.ReleasePixels();

		pcs.Leave();
		return true;
	}
};

// Average intrinsics estimates helper
static REAL AverageEstimates(DoubleArr& estimates)
{
	// Compute statistics
	const double median = estimates.GetMedian();
	const MeanStdMinMax<double> stats(estimates.data(), estimates.size());
	// Mean of the focal estimates excluding top/bottom 10% percentiles
	const REAL mean(estimates.GetTrimmedMean<REAL>(0.1f, 0.1f));
	DEBUG("Focal estimates: median %.2f mean %.2f stddev %.2f range [%.2f,%.2f] n %u",
			median, stats.GetMean(), stats.GetStdDev(), stats.GetMin(), stats.GetMax(), estimates.size());
	return mean;
}

// Triplet star-calibration helper
static void RunThreeViewStarCalibration(
    Scene& scene,
    DoubleArr& focalEstimates,
    DoubleArr& k1Estimates,
    DoubleArr& k2Estimates)
{
	if (scene.images.size() < 3)
		return;
	ASSERT(scene.cameras[0]->GetType() == CameraType::PINHOLE);
	// Subsample stride for speed
	const IIndex N = scene.images.size();
	const IIndex stride = (N > 60 ? 3 : (N > 30 ? 2 : 1));
	focalEstimates.clear();
	k1Estimates.clear();
	k2Estimates.clear();
	for (IIndex center = 1; center + 1 < N; center += stride) {
		const IIndex prevID = center - 1;
		const IIndex nextID = center + 1;
		const ImagePair* pPrev = scene.FindPair(prevID, center);
		const ImagePair* pNext = scene.FindPair(center, nextID);
		if (!pPrev || !pNext)
			continue;
		if (!pPrev->relativePose.has_value() || !pNext->relativePose.has_value())
			continue; // relative pose must be available
		if (pPrev->matches.size() < 15 || pNext->matches.size() < 15)
			continue; // insufficient matches for meaningful tracks
		// Build sub-scene with three images
		Scene sub;
		// Clone camera (shared intrinsics initial values)
		sub.cameras.emplace_back(scene.cameras[0]->Clone());
		// Copy images
		auto CopyImage = [&](uint32_t srcID, uint32_t dstID) {
			Image& dst = sub.images.emplace_back(scene.images[srcID]);
			dst.ID = dstID;
			ASSERT(dst.cameraID == 0);
			dst.pCamera = sub.cameras[0];
		};
		CopyImage(prevID, 0);
		CopyImage(center, 1);
		CopyImage(nextID, 2);
		// Copy pairs (prev-center) and (center-next)
		auto AddPair = [&](const ImagePair* srcPair, uint32_t aNew, uint32_t bNew) {
			ImagePair& np = sub.pairs.emplace_back(aNew, bNew);
			np.matches = srcPair->matches; // inlier matches only
			np.relativePose = srcPair->relativePose; // already estimated
		};
		AddPair(pPrev, 0, 1);
		AddPair(pNext, 1, 2);
		// Build tracks in sub-scene
		BuildTracks(sub);
		if (sub.tracks.empty())
			continue;
		// Star initialization (reference will be center with connectivity 2)
		StarInitConfig initCfg; // defaults
		initCfg.minViews = 3;
		if (!StarInitializer::Initialize(sub, initCfg))
			continue;
		// Collect refined intrinsics (camera index 0)
		PinholeCamera& refinedCam = *static_cast<PinholeCamera*>(sub.cameras[0]);
		if (refinedCam.fx > 0)
			focalEstimates.push_back(refinedCam.fx);
		if (ABS(refinedCam.k1) < 0.5)
			k1Estimates.push_back(refinedCam.k1);
		if (ABS(refinedCam.k2) < 0.5)
			k2Estimates.push_back(refinedCam.k2);
		DEBUG_EXTRA("Triplet calibration center %u: f=%.2f k1=%.6f k2=%.6f (tracks %u)",
		            center, refinedCam.fx, refinedCam.k1, refinedCam.k2, (unsigned)sub.tracks.size());
	}
}

// Main keyframe extraction function
bool ExtractFromVideo(const String& videoPath, const KeyframeConfig& config, Scene& scene)
{
	ASSERT(config.imageFormat == "jxl" || config.imageFormat == "jpg" || config.imageFormat == "png");
	// Clear the scene
	scene.Release();

	// Open video file - try multiple backends
	cv::VideoCapture video;
	const int backends[] = { cv::CAP_ANY, cv::CAP_FFMPEG, cv::CAP_GSTREAMER };
	const char* backendNames[] = { "AUTO", "FFMPEG", "GSTREAMER" };
	for (size_t i = 0; i < 3; ++i) {
		video.open(videoPath.c_str(), backends[i]);
		if (video.isOpened()) {
			DEBUG_EXTRA("Opened video using backend: %s", backendNames[i]);
			goto ProcessVideo;
		}
	}
	VERBOSE("KeyframeExtractor: failed to open video '%s'", videoPath.c_str());
	return false;
	ProcessVideo:

	// Get video properties
	const int frameWidth = (int)video.get(cv::CAP_PROP_FRAME_WIDTH);
	const int frameHeight = (int)video.get(cv::CAP_PROP_FRAME_HEIGHT);
	const double fps = video.get(cv::CAP_PROP_FPS);
	const unsigned totalFrames = (unsigned)video.get(cv::CAP_PROP_FRAME_COUNT);
	DEBUG("Video: %dx%d @ %.2f fps, %d frames, %d s",
		frameWidth, frameHeight, fps, totalFrames, ROUND2INT(totalFrames / fps));
	if (config.cameraType == CameraType::SPHERICAL && frameWidth != 2 * frameHeight) {
		VERBOSE("error: video '%s' is declared spherical but has %dx%d; equirectangular input requires width == 2 * height",
			videoPath.c_str(), frameWidth, frameHeight);
		return false;
	}

	// Ensure output directory exists
	ASSERT(Util::isFullPath(config.outputDirectory));
	Util::ensureFolder(config.outputDirectory);

	// Create a camera for this video (shared by all frames)
	Camera* pCamera = nullptr;
	switch (config.cameraType) {
	case CameraType::PINHOLE: {
		// Use provided focal length or default to max(width, height)
		const double focalLength = (config.focalLength > 0) ? config.focalLength : MAXF(frameWidth, frameHeight);
		// Compute principal point: center + optional offsets
		const double ppX = frameWidth / 2.0 + config.ppOffsetX;
		const double ppY = frameHeight / 2.0 + config.ppOffsetY;
		PinholeCamera* pPinholeCamera = new PinholeCamera(cv::Size(frameWidth, frameHeight),
		                                                  focalLength, focalLength,
		                                                  ppX, ppY);
		// only a user-provided focal is trusted; the guessed one stays untrusted (default)
		// so that the two-view/triplet auto-calibration below is allowed to refine it
		if (config.focalLength > 0)
			pPinholeCamera->trustIntrinsics = true;
		DEBUG("Camera intrinsics: f %.2f, pp (%.2f, %.2f) %s",
		      focalLength, ppX, ppY,
		      (config.focalLength > 0) ? "[user-provided]" : "[auto-estimated]");
		pCamera = pPinholeCamera;
	} break;
	case CameraType::SPHERICAL: {
		pCamera = new SphericalCamera(cv::Size(frameWidth, frameHeight));
	} break;
	default:
		VERBOSE("KeyframeExtractor: unsupported camera type %d", (int)config.cameraType);
		return false;
	}
	const IIndex cameraID = scene.cameras.size();
	scene.cameras.emplace_back(pCamera);
	scene.nMaxThreads = 1; // force single-threaded feature extraction and matching

	// Configure FeaturesExtractor for feature extraction
	FeatureExtractionConfig extractConfig;
	extractConfig.detectorType = config.detectorType;
	extractConfig.maxFeaturesPerCell = config.maxFeaturesPerCell;
	extractConfig.minFeaturesPerCell = config.minFeaturesPerCell;
	extractConfig.releaseImagePixels = false; // we will manage image pixel release ourselves
	extractConfig.useCUDA = config.useCUDA;
	extractConfig.cubemapFaces = (int)config.cubemapFaces;
	FeaturesExtractor extractor(scene, extractConfig);

	// Configure PairsMatcher for feature matching
	MatchConfig matchConfig;
	matchConfig.DefaultsForFeatureType(config.detectorType);
	matchConfig.maxEpipolarError = 5.f; // pixels
	matchConfig.forceFundamentalWithFocal = !pCamera->TrustIntrinsics() && config.refineCalibration == KeyframeConfig::TWO_VIEW; // if two-view focal estimation is requested, enable shared-focal F estimation
	matchConfig.useCUDA = config.useCUDA;
	PairsMatcher pairsMatcher(scene, matchConfig);

	// Create a background worker to offload heavy tasks (saving images, matching)
	SEACAVE::EventQueue workerQueue;
	SEACAVE::CriticalSection sceneCs; // protect access to scene and focalEstimates from worker/main thread
	// Worker thread: will fetch events from workerQueue and execute them
	class WorkerThread : public SEACAVE::Thread {
	public:
		SEACAVE::EventQueue& q;
		WorkerThread(SEACAVE::EventQueue& _q) : q(_q) {}
	protected:
		void run() override {
			while (true) {
				SEACAVE::Event* evt = q.GetEvent();
				if (!evt) continue;
				// id==0 will be the shutdown event
				if (evt->GetID() == 0) {
					delete evt;
					break;
				}
				evt->Run();
				delete evt;
			}
		}
	} worker(workerQueue);
	worker.start();

	// Variables for tracking
	cv::Ptr<cv::Feature2D> detector;
	cv::Mat currFrame, keyframeGray, currGray;
	std::vector<Point2f> keyframePoints;  // Fixed points from last keyframe (constant until new keyframe)
	std::vector<Point2f> currPoints;      // Current tracked positions (updated each frame, used as initial guess)
	std::vector<uchar> status;
	std::vector<float> err;
	float overlapRatio = 1.f;
	float overlapArea = 1.f;
	unsigned frameIdx = 0;
	bool bExtractionFailed = false;
	DoubleArr focalEstimates; // focal length estimates derived from fundamental matrices
	DoubleArr k1Estimates; // radial distortion k1 estimates
	DoubleArr k2Estimates; // radial distortion k2 estimates

	// Small rolling cache of recent frames to pick the sharpest frame when selecting a keyframe
	std::deque<CachedFrame> frameCache;
	constexpr size_t FRAME_CACHE_SIZE = 5;
	const auto EnqueueFrame2Cache = [&]() {
		// Add current frame to cache
		if (frameCache.size() == FRAME_CACHE_SIZE)
			frameCache.pop_front();
		CachedFrame cf;
		cf.frame = currFrame.clone();
		cf.gray = currGray.clone();
		cf.trackedPoints = currPoints;
		cf.status = status;
		cf.frameIdx = frameIdx;
		// the backend's time stamp of the frame just read: unlike frameIdx/fps, it does not drift
		// on variable frame rate video (the container's fps is only an average or a nominal rate);
		// exact with FFmpeg and AVFoundation, within a fraction of a frame with Media Foundation
		cf.timestamp = video.get(cv::CAP_PROP_POS_MSEC) * 1e-3;
		cf.overlapRatio = overlapRatio;
		cf.overlapArea = overlapArea;
		cf.sharpness = EstimateImageSharpness(currGray);
		frameCache.push_back(std::move(cf));
	};
	const auto SelectBestFrame = [&frameCache]() -> CachedFrame& {
		ASSERT(!frameCache.empty());
		// Start selecting a frame from the most recent (back) towards the front
		// so that when sharpness is similar we prefer the most recent frame.
		// Only pick an older frame if it is considerably sharper than the
		// most recent one (relative improvement threshold).
		constexpr float IMPROVEMENT_RATIO = 1.05f; // require >5% sharper to switch
		float bestSharp = frameCache.back().sharpness;
		IIndex bestIdx = (IIndex)frameCache.size() - 1;
		for (IIndex k = bestIdx; k-- > 0; ) {
			const float s = frameCache[k].sharpness;
			if (s / bestSharp > IMPROVEMENT_RATIO) {
				bestSharp = s;
				bestIdx = k;
			}
		}
		return frameCache[bestIdx];
	};

	// Process video frames
	Util::Progress progress(_T("Processing video frames"), totalFrames);
	GET_LOGCONSOLE().Pause();
	while (video.read(currFrame)) {
		// Convert to grayscale for tracking
		cv::cvtColor(currFrame, currGray, cv::COLOR_BGR2GRAY);

		// Optionally blur image used for optical flow to improve tracking in noisy/compressed frames
		if (config.blurSize > 0)
			cv::GaussianBlur(currGray, currGray, cv::Size((int)config.blurSize, (int)config.blurSize), 0.0);

		bool isKeyframe = false;
		if (scene.images.empty() || keyframePoints.empty() || currPoints.empty()) {
			// First frame or no features to track, force keyframe
			isKeyframe = true;
			EnqueueFrame2Cache();
		} else {
			// Track features using optical flow with incremental updates
			// Track from previous frame to current frame (frame-to-frame)
			cv::calcOpticalFlowPyrLK(
				keyframeGray, currGray,
				keyframePoints, currPoints,
				status, err,
				cv::Size(21, 21), 3,
				cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01),
				cv::OPTFLOW_USE_INITIAL_FLOW);

			// Compute overlap metrics (always relative to fixed keyframe positions)
			overlapRatio = ComputeFeatureOverlap(keyframePoints, currPoints, status, currFrame.size());
			overlapArea = ComputeHomographyOverlap(keyframePoints, currPoints, status, currFrame.size(), *pCamera);

			// Compute sharpness for this frame and push into small rolling cache
			EnqueueFrame2Cache();

			// Check if we need a new keyframe:
			// if overlap drops below threshold or if we are at the last frame
			if (overlapRatio < config.overlapThreshold ||
				overlapArea < config.overlapThreshold ||
				(frameIdx+1 == totalFrames && (overlapRatio < 0.95f || overlapArea < 0.95f)))
				isKeyframe = true;
			DEBUG_ULTIMATE("\tFrame %d: overlap ratio %.3f, area %.3f, sharpness %.3f",
				frameIdx, overlapRatio, overlapArea, frameCache.back().sharpness);
		}

		if (isKeyframe) {
			// Choose the sharpest frame from the cache (prefer recent sharp frames)
			CachedFrame& chosen = SelectBestFrame();
			const IIndex keyframeID = scene.images.size();
			DEBUG("Selecting keyframe %d at frame %d (time %.3fs)", keyframeID, chosen.frameIdx, chosen.timestamp);

			// Create new image (protected) using chosen cached frame
			sceneCs.Enter();
			Image& image = scene.images.emplace_back();
			image.ID = keyframeID;
			image.timestamp = chosen.timestamp;
			image.pixels = std::move(chosen.frame);

			// Set view
			image.cameraID = cameraID;
			image.pCamera = pCamera;

			// Generate filename
			image.fileName = config.outputDirectory + String::FormatString("keyframe_%05d.", keyframeID) + config.imageFormat;
			sceneCs.Leave();

			// Extract features (need keypoints)
			if (!extractor.ExtractImage(image, detector)) {
				VERBOSE("error: failed to extract features from keyframe %u", keyframeID);
				bExtractionFailed = true;
				break;
			}
			if (image.keypoints.empty()) {
				// a frame without features (e.g. a black frame) can not be matched, so it is useless:
				// drop it and select the next keyframe among the following frames
				DEBUG("Skipping frame %u (time %.3fs): no features", chosen.frameIdx, chosen.timestamp);
				sceneCs.Enter();
				scene.images.RemoveLast();
				sceneCs.Leave();
			} else {
				// Enqueue a single post-processing event (save + matching)
				workerQueue.AddEvent(new
					KeyframePostProcessEvent(keyframeID-1, keyframeID,
						scene, pairsMatcher, *pCamera, config.focalLength > 0,
						keyframePoints, chosen.trackedPoints, chosen.status, chosen.overlapRatio, chosen.overlapArea,
						sceneCs, focalEstimates, k1Estimates, k2Estimates, config.refineCalibration == KeyframeConfig::TWO_VIEW));

				// Convert keypoints to points for tracking
				// keyframePoints: fixed reference positions (constant until next keyframe)
				keyframePoints.clear();
				for (const auto& kp : image.keypoints)
					keyframePoints.push_back(kp.pt);

				// Initialize tracked positions to keyframe positions
				// (will be updated incrementally as we track across frames)
				currPoints = keyframePoints;

				// Update keyframe gray to the chosen frame's gray
				keyframeGray = std::move(chosen.gray);
			}

			// Clear cache so we start accumulating frames for the next keyframe
			frameCache.clear();
		}

		++frameIdx;
		++progress;
	}

	// Shutdown worker: post sentinel event with id==0 and wait for thread to finish
	video.release();
	workerQueue.AddEvent(new SEACAVE::Event(0));
	worker.join();
	GET_LOGCONSOLE().Play();
	progress.close();
	if (bExtractionFailed)
		return false;

	// Finalize scene status
	scene.status.nFeaturesType = config.detectorType;
	scene.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);

	// Compute pair weights
	ComputePairsWeights(scene, matchConfig.weightingCfg);

	// Auto-calibrate intrinsics (if enabled and sufficient estimates)
	// Only update if user didn't provide known intrinsics (config.focalLength <= 0)
	if (pCamera->GetType() == CameraType::PINHOLE) {
		PinholeCamera* pPinholeCamera = dynamic_cast<PinholeCamera*>(pCamera);

		// Optional calibration refinement strategies
		if (config.refineCalibration == KeyframeConfig::VIEW_GRAPH && config.focalLength <= 0) {
			// Use view graph calibration (global optimization over all pairs)
			ViewGraphCalibratorConfig vgConfig;
			vgConfig.maxTwoViewError = 0; // disable two-view filtering
			ViewGraphCalibrator calibrator(vgConfig);
			if (calibrator.Solve(scene)) {
				// Recompute relative-pose for all pairs with updated image cameras
				if (!calibrator.GetUpdatedCameras().empty())
					pairsMatcher.ComputeRelativePoses(true, false, calibrator.GetUpdatedCameras());
			} else {
				DEBUG("warning: ViewGraph calibration failed");
			}
		} else if (config.refineCalibration == KeyframeConfig::THREE_VIEW) {
			RunThreeViewStarCalibration(scene, focalEstimates, k1Estimates, k2Estimates);
		} else {
			const unsigned totalPairs(scene.images.empty() ? 0u : scene.images.size()-1);
			DEBUG("Focal estimation succeeded for %u/%u pairs (%.2f%%)",
				focalEstimates.size(), totalPairs,
				totalPairs > 0 ? (focalEstimates.size() * 100.0 / totalPairs) : 0.0);
		}

		// Auto-calibrate focal length from per-pair / triplet fundamental matrices
		// Only update if user didn't provide known intrinsics and view graph didn't handle it
		if (focalEstimates.size() > 16 && !pPinholeCamera->trustIntrinsics) {
			// Mean of the focal estimates excluding top/bottom 10% percentiles
			const REAL meanF = AverageEstimates(focalEstimates);
			if (config.focalLength <= 0) {
				// Update camera intrinsics
				pPinholeCamera->fx = pPinholeCamera->fy = meanF;
				pPinholeCamera->trustIntrinsics = true;
				DEBUG("\tAuto-calibrated focal length: %.2f pixels", pPinholeCamera->fx);
			} else {
				// User provided intrinsics: report statistics but don't override
				DEBUG("\tKeeping user-provided focal %.2f", pPinholeCamera->fx);
			}
		} else if (config.focalLength <= 0) {
			DEBUG("Insufficient focal estimates (%d < 16), keeping initial guess f=%.2f",
				(int)focalEstimates.size(), pPinholeCamera->fx);
		}
		// Auto-calibrate distortion coefficients from refined estimates
		if (k1Estimates.size() > 16 && k2Estimates.size() > 16) {
			// Mean of k1/k2 estimates excluding top/bottom 10% percentiles
			const REAL meanK1 = AverageEstimates(k1Estimates);
			const REAL meanK2 = AverageEstimates(k2Estimates);
			if (!pPinholeCamera->HasDistortion()) {
				// Update camera distortion
				pPinholeCamera->k1 = meanK1;
				pPinholeCamera->k2 = meanK2;
				DEBUG("\tAuto-calibrated distortion: k1=%.6f k2=%.6f", pPinholeCamera->k1, pPinholeCamera->k2);
			} else {
				DEBUG("\tDistortion estimates collected but not applied (focal known or distortion already set)");
			}
		} else {
			DEBUG("Insufficient distortion estimates (k1:%d k2:%d < 16), keeping initial values",
				(int)k1Estimates.size(), (int)k2Estimates.size());
		}
	}

	DEBUG("Extracted %u keyframes from %d frames", scene.images.size(), totalFrames);
	return !scene.images.empty();
}

} // namespace SFM::KeyframeExtractor
