/*
 * Resection.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "Resection.h"
#include "Scene.h"
#include "Track.h"
#include "Triangulation.h"
#include <PoseLib/poselib.h>

using namespace SFM;

// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

Resection::Resection(Scene& _scene, const ResectionConfig& _config)
	: scene(_scene), config(_config)
{}

IIndexArr Resection::SelectNextImages(IIndexScores& unregistered) const
{
	ASSERT(!unregistered.empty());

	// Score accumulation
	for (auto& it : unregistered)
		it.second = 0;
	for (uint32_t trackID = 0; trackID < scene.tracks.size(); ++trackID) {
		const Track& track = scene.tracks[trackID];
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track.observations) {
			auto it = unregistered.find(obs.imageID);
			if (it != unregistered.end())
				++it->second;
		}
	}

	// Fetch image IDs and order by score
	IIndexArr nextIDs;
	for (const auto& it : unregistered)
		if (it.second >= config.minCorrespondences)
			nextIDs.push_back(it.first);
	if (nextIDs.empty()) {
		VERBOSE("warning: no next images with sufficient correspondences");
		return nextIDs;
	}
	nextIDs.Sort([&unregistered](IIndex a, IIndex b) {
		return unregistered.at(a) > unregistered.at(b);
	});

	// Select top images with sufficient visible points
	const unsigned thScore = config.ratioCorrespondences * unregistered.at(nextIDs[0]);
	for (unsigned i = 1; i < nextIDs.size(); ++i) {
		if (unregistered.at(nextIDs[i]) < thScore) {
			nextIDs.resize(i);
			break;
		}
	}
	VERBOSE("Selected %u images with %u best visible points", nextIDs.size(), unregistered.at(nextIDs[0]));
	return nextIDs;
}

 std::pair<unsigned, unsigned> Resection::RegisterImage(IIndex imageID)
{
	Image& img = scene.images[imageID];
	ASSERT(img.HasCamera() && !img.HasPose());

	// Unified bearing-vector PnP path: works for any central camera model
	// (pinhole, spherical / equirectangular, fisheye). The bearings come from
	// Camera::UnprojectNormalized which already returns unit vectors carrying
	// hemisphere information (sign(z)) for spherical cameras.
	std::vector<poselib::Point3D> bearings;
	std::vector<poselib::Point3D> points3D;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track.observations) {
			if (obs.imageID == imageID) {
				const Point2 kp = img.keypoints[obs.featureID].pt;
				bearings.emplace_back(img.pCamera->UnprojectNormalized(kp));
				points3D.push_back(track.position);
				break;
			}
		}
	}
	const unsigned n = (unsigned)bearings.size();
	if (n < config.minInliers)
		return {0, n};

	// Convert the pixel-space reprojection threshold to an angular threshold
	// on the unit sphere via the camera's PixelErrorToAngular helper, then
	// let AbsolutePoseOptions convert to the chord-distance metric its
	// scoring function uses internally. The per-camera noise scale widens
	// the pinhole-tuned threshold for models (e.g. spherical cube-face SIFT)
	// whose feature positions have higher pixel-space uncertainty.
	poselib::AbsolutePoseOptions opt;
	opt.ransac.max_iterations = config.ransac.max_iterations;
	opt.ransac.min_iterations = config.ransac.min_iterations;
	opt.ransac.success_prob = config.ransac.confidence;
	opt.SetMaxErrorFromAngle(img.pCamera->PixelErrorToAngular(
		config.ransac.threshold * img.pCamera->GetFeatureNoiseScale()));

	std::vector<char> inliers;
	poselib::CameraPose camPose;
	poselib::RansacStats stats = poselib::estimate_absolute_pose_bearings(
		bearings, points3D, opt, &camPose, &inliers);

	const unsigned numInliers = (unsigned)stats.num_inliers;
	if (numInliers < config.minInliers)
		return {0, n};

	img.R = camPose.R();
	img.SetT(camPose.t);
	return {numInliers, n};
}

IIndexArr Resection::BuildLocalWindow(const IIndexArr& imageIDs) const
{
	const std::unordered_set<IIndex> uniqueIDs(imageIDs.begin(), imageIDs.end());
	std::unordered_map<IIndex, uint32_t> counts;
	counts.reserve(64);
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		bool observedByTarget = false;
		for (const Observation& obs : track) {
			if (uniqueIDs.count(obs.imageID) > 0) {
				observedByTarget = true;
				break;
			}
		}
		if (!observedByTarget)
			continue;
		for (const Observation& obs : track) {
			ASSERT(scene.images[obs.imageID].IsValid());
			if (uniqueIDs.count(obs.imageID) == 0)
				++counts[obs.imageID];
		}
	}
	if (counts.empty())
		return {};

	using IIndexPoints = TIndexScore<IIndex,unsigned>;
	CLISTDEF0IDX(IIndexPoints, unsigned) ranked(0u, counts.size());
	for (const auto& entry : counts)
		ranked.emplace_back(entry.first, entry.second);
	ranked.Sort([](const auto& a, const auto& b) {
		return a.score > b.score;
	});
	const unsigned maxNeighbors(config.maxLocalWindow == 0 ? (unsigned)ranked.size() : MINF((unsigned)ranked.size(), config.maxLocalWindow - imageIDs.size()));
	IIndexArr fixedViewIDs(0u, maxNeighbors);
	for (unsigned i = 0; i < maxNeighbors; ++i)
		fixedViewIDs.push_back(ranked[i].idx);
	return fixedViewIDs;
}

bool Resection::RegisterImages()
{
	TD_TIMER_STARTD();

	// Collect unregistered images
	IIndexScores unregistered;
	unregistered.reserve(scene.images.size() * 2 / 3);
	for (const Image& img : scene.images)
		if (!img.HasPose())
			unregistered.emplace(img.ID, 0u);
	if (unregistered.empty()) {
		VERBOSE("warning: no unregistered images");
		return true;
	}

	// Resection loop
	unsigned nBA = 0;
	unsigned registeredCount = 0;
	unsigned sinceFullBA = 0;
	IIndexArr lastRegistered;
	TRunningAverage<float, 10> avgInliersRatio;
	while (!unregistered.empty()) {
		IIndexArr nextIDs = SelectNextImages(unregistered);
		if (nextIDs.empty()) {
			VERBOSE("warning: no more connected images to register, %u images remain", unregistered.size());
			break;
		}
		const unsigned startRegisteredCount = registeredCount;
		for (IIndex n = 0; n < nextIDs.size(); ) {
			// Attempt to register next image
			const IIndex nextID = nextIDs[n];
			const auto [numInliers, numPoints] = RegisterImage(nextID);
			if (numPoints > 0)
				avgInliersRatio += numInliers / (float)numPoints;
			if (numInliers == 0) {
				DEBUG("warning: failed to register image %u (%u/%u correspondences), retrying later", nextID, numInliers, numPoints);
				nextIDs.RemoveAtMove(n);
				continue; // n now points to the shifted element, do not increment
			}
			lastRegistered.push_back(nextID);
			unregistered.erase(nextID);
			++registeredCount;
			++sinceFullBA;
			++n;
			DEBUG_EXTRA("\tImage %u registered: %u/%u correspondences (%u/%u images, %.2f%% avg inliers ratio)",
				nextID, numInliers, numPoints, scene.status.nCalibratedImages+registeredCount, scene.images.size(), avgInliersRatio.GetAverage() * 100.f);
			if ((config.fullBAEvery[nBA] > 0 && sinceFullBA >= config.fullBAEvery[nBA]) || (config.avgInliersRatioForceBA > 0.f && avgInliersRatio.GetAverage() < config.avgInliersRatioForceBA)) {
				// Full BA every N registered images
				TriangulateTracks(scene, false, config.maxReprojError, config.minAngleThreshold);
				if (config.minRefineExtIntrs > 0 && scene.status.nCalibratedImages + registeredCount >= config.minRefineExtIntrs)
					config.fullBAConfig.RefineExtendedIntrinsics();
				BundleAdjustment::Adjust(scene, config.fullBAConfig);
				FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
				lastRegistered.clear();
				avgInliersRatio.Clear();
				sinceFullBA = 0;
				if (nBA + 1 < config.fullBAEvery.size())
					++nBA;
				break; // restart selection of next images
			} else if (config.localBAEvery > 0 && lastRegistered.size() >= config.localBAEvery) {
				// Local BA every N registered images
				TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
				const IIndexArr fixedViewIDs = BuildLocalWindow(lastRegistered);
				ASSERT(!fixedViewIDs.empty());
				BundleAdjustment::AdjustLocal(scene, lastRegistered, fixedViewIDs, config.localBAConfig);
				FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
				lastRegistered.clear();
				break; // restart selection of next images
			} else if (n+1 == nextIDs.size() || (config.triangulateEvery > 0 && (lastRegistered.size() % config.triangulateEvery) == 0)) {
				// Update scene with new points every N registered images
				TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
				FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
				break; // restart selection of next images
			}
		}
		if (registeredCount == startRegisteredCount) {
			VERBOSE("warning: no images were registered in last iteration, stopping resection, %u images remain", unregistered.size());
			break;
		}
	}

	// Full BA after all images are registered
	TriangulateTracks(scene, false, config.maxReprojError, config.minAngleThreshold);
	config.fullBAConfig.maxIterations = 100;
	config.fullBAConfig.RefineExtendedIntrinsics();
	BundleAdjustment::Adjust(scene, config.fullBAConfig);
	FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);

	// Update scene status
	scene.status.nCalibratedImages += registeredCount;
	scene.status.nState.set(Scene::Status::STATE::CALIBRATED);
	DEBUG("Resection registered %u new images, total %u/%u images (%s)",
		registeredCount, scene.status.nCalibratedImages, scene.images.size(), TD_TIMER_GET_FMT().c_str());
	return registeredCount > 0;
}
/*----------------------------------------------------------------*/
