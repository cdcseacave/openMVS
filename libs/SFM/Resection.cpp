/*
 * Resection.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "Resection.h"
#include "PoseLink.h"
#include "Scene.h"
#include "Track.h"
#include "Triangulation.h"
#include <PoseLib/poselib.h>

using namespace SFM;

// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace {

// The absolute pose of an image, as the quorum reads its neighbors
inline auto NeighborPose(const Scene& scene)
{
	return [&scene](IIndex imageID) -> const Pose3D& { return scene.images[imageID]; };
}

// Every link the given image has to an already registered image, strongest first
void CollectPoseLinks(const Scene& scene, IIndex imageID, std::vector<PoseLink>& links)
{
	links.clear();
	for (const ImagePair& pair : scene.pairs) {
		if (!IsPoseLinkPair(pair))
			continue;
		IIndex neighborID;
		if (pair.ID1 == imageID)
			neighborID = pair.ID2;
		else if (pair.ID2 == imageID)
			neighborID = pair.ID1;
		else
			continue;
		if (scene.images[neighborID].HasPose())
			links.push_back(MakePoseLink(pair, neighborID));
	}
	std::sort(links.begin(), links.end(), IsStrongerLink);
}

// Polish an absolute pose over its inliers with each dense correspondence at denseWeight against a
// described one's 1, through the estimator's own final refinement (estimate_absolute_pose_bearings:
// the same loss at the same chord-distance tolerance). Uniform weights would leave that refinement's
// pose where it is, so an inlier set of a single kind is left alone.
void PolishAbsolutePose(const std::vector<poselib::Point3D>& bearings, const std::vector<poselib::Point3D>& points3D,
	const std::vector<char>& dense, const std::vector<char>& inliers, const poselib::AbsolutePoseOptions& opt,
	float denseWeight, poselib::CameraPose& pose)
{
	ASSERT(denseWeight >= 0.f && denseWeight <= 1.f);
	ASSERT(bearings.size() == points3D.size() && bearings.size() == dense.size() && bearings.size() == inliers.size());
	std::vector<poselib::Point3D> bearingsInliers, pointsInliers;
	std::vector<double> weights;
	size_t numDense = 0;
	FOREACH(k, bearings) {
		if (!inliers[k])
			continue;
		bearingsInliers.push_back(bearings[k]);
		pointsInliers.push_back(points3D[k]);
		weights.push_back(dense[k] ? (double)denseWeight : 1.0);
		if (dense[k])
			++numDense;
	}
	if (numDense == 0 || numDense == weights.size())
		return;
	poselib::BundleOptions bundle = opt.bundle;
	bundle.loss_scale = 2.0 * std::sin(0.5 * opt.max_error);
	poselib::bundle_adjust_bearing(bearingsInliers, pointsInliers, &pose, bundle, weights);
}

} // namespace

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

 std::tuple<unsigned, unsigned, unsigned> Resection::RegisterImage(IIndex imageID)
{
	Image& img = scene.images[imageID];
	ASSERT(img.HasCamera() && !img.HasPose());

	// Unified bearing-vector PnP path: works for any central camera model
	// (pinhole, spherical / equirectangular, fisheye). The bearings come from
	// Camera::UnprojectNormalized which already returns unit vectors carrying
	// hemisphere information (sign(z)) for spherical cameras.
	std::vector<poselib::Point3D> bearings;
	std::vector<poselib::Point3D> points3D;
	std::vector<char> dense;
	unsigned numDescribed = 0;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track.observations) {
			if (obs.imageID == imageID) {
				const Point2 kp = img.keypoints[obs.featureID].pt;
				bearings.emplace_back(img.pCamera->UnprojectNormalized(kp));
				points3D.push_back(track.position);
				dense.push_back(img.IsDenseKeypoint(obs.featureID));
				if (!dense.back())
					++numDescribed;
				break;
			}
		}
	}
	const unsigned n = (unsigned)bearings.size();
	if (n < config.minInliers)
		return {0, n, numDescribed};

	// Convert the pixel-space reprojection threshold to an angular threshold
	// on the unit sphere via the camera's PixelErrorToAngular helper and hand
	// it to the bearing estimator as opt.max_error (radians); the estimator
	// converts internally to the chord-distance metric its scoring function
	// uses. The per-camera noise scale widens the pinhole-tuned threshold for
	// models (e.g. spherical cube-face SIFT) whose feature positions have
	// higher pixel-space uncertainty.
	poselib::AbsolutePoseOptions opt;
	opt.ransac.max_iterations = config.ransac.max_iterations;
	opt.ransac.min_iterations = config.ransac.min_iterations;
	opt.ransac.success_prob = config.ransac.confidence;
	opt.max_error = img.pCamera->PixelErrorToAngular(
		config.ransac.threshold * img.pCamera->GetFeatureNoiseScale());

	std::vector<char> inliers;
	poselib::CameraPose camPose;
	poselib::RansacStats stats = poselib::estimate_absolute_pose_bearings(
		bearings, points3D, opt, &camPose, &inliers);

	const unsigned numInliers = (unsigned)stats.num_inliers;
	if (numInliers < config.minInliers)
		return {0, n, numDescribed};

	// A pose is accepted only when its support is credible. A small consensus inside a large set of
	// correspondences can agree on a pose the image never had, so the inliers must be a large enough
	// share of the correspondences, unless there are so many of them that the count vouches for the
	// pose whatever the share.
	const float inlierRatio = numInliers / (float)n;
	if (config.minInlierRatio > 0.f && inlierRatio < config.minInlierRatio && numInliers < config.minInliersAbsolute) {
		DEBUG("warning: rejected the pose of image %u: %u/%u inliers (%.1f%%) below the %.1f%% minimum share",
			imageID, numInliers, n, inlierRatio * 100.f, config.minInlierRatio * 100.f);
		return {0, n, numDescribed};
	}

	// the polish that closes the fit, again over its inliers but with each dense correspondence at
	// ResectionConfig::denseFitWeight: the robust fit over every correspondence found the pose, and the
	// described ones, sub-pixel where a dense one is a coarse warp sample whose error its neighbours
	// share, set it wherever they are many
	if (config.denseFitWeight != 1.f)
		PolishAbsolutePose(bearings, points3D, dense, inliers, opt, config.denseFitWeight, camPose);

	// the pose the estimator found, in the model's own frame: the checks below weigh it against the
	// image's links before the image is given it
	Pose3D pose;
	pose.R = camPose.R();
	pose.SetT(camPose.t);

	// Every pose is cross-checked against the image's verified pairs to already registered images,
	// however large its inlier share: on alameda a 54% share (87/162) put a pose 1.9 degrees off,
	// taking the 15 images it seeded with it, and every right registration passes the three checks
	// (quorum within 7.7 of the 15-degree bar; one direction refusal of 1538). The witness is the
	// quorum of those pairs -- the largest group agreeing on a rotation -- not the strongest one of
	// them, which speaks falsely whenever that single neighbor is itself misplaced. When the pairs
	// contradict one another they are no witness at all, so the pose is left to the other
	// acceptance rules and the disagreement is reported.
	if (config.minRegisteredEvidence > 0.f || config.maxRelativeRotationError > 0.f || config.maxLinkDirectionError > 0.f) {
		// registered where the evidence is: an image most of whose pair weight lies in images the
		// model does not hold yet is registered by the few pairs it does hold, and its community then
		// hangs off the model through them
		if (config.minRegisteredEvidence > 0.f) {
			const float share = RegisteredEvidenceShare(imageID);
			const float bar = config.minRegisteredEvidence * RegisteredImagesFraction();
			if (share < bar) {
				DEBUG("warning: deferred the pose of image %u: %u/%u inliers (%.1f%%) and only %.1f%% of its pair weight lies in "
					"registered images, under the %.1f%% the registered share of the scene asks", imageID, numInliers, n,
					inlierRatio * 100.f, share * 100.f, bar * 100.f);
				// a deferral reports no correspondences: it says nothing about how well the model fits this
				// image, only that its evidence is elsewhere for now
				return {0, 0, numDescribed};
			}
		}
		std::vector<PoseLink> links;
		CollectPoseLinks(scene, imageID, links);
		const PoseLinkQuorum quorum = ComputePoseLinkQuorum(links, config.maxRelativeRotationError, NeighborPose(scene));
		if (quorum.IsContested()) {
			DEBUG("the pairs of image %u to its registered neighbours disagree by up to %.1f degrees",
				imageID, quorum.maxDisagreement);
		} else if (!quorum.links.empty()) {
			// the rotation the quorum composes
			if (config.maxRelativeRotationError > 0.f) {
				const double angle = R2D(ACOS(ComputeAngle(pose.R, quorum.R)));
				if (angle > config.maxRelativeRotationError) {
					DEBUG("warning: rejected the pose of image %u: %u/%u inliers (%.1f%%) and %.1f degrees away from the "
						"rotation its quorum of %u of %u pairs composes (strongest to image %u)", imageID, numInliers, n,
						inlierRatio * 100.f, angle, (unsigned)quorum.links.size(), (unsigned)links.size(),
						quorum.links.front().neighborID);
					return {0, n, numDescribed};
				}
			}
			// the baseline direction each quorum link predicts: each is held to the tolerance its own
			// ray angle allows, and the pose is refused only when the links that contradict it
			// outweigh the ones that agree, so one misplaced neighbour cannot refuse a right pose
			// that other links vouch for
			if (config.maxLinkDirectionError > 0.f) {
				unsigned agreeing = 0, disagreeing = 0;
				for (const PoseLink& link : quorum.links) {
					const PairDisagreement d = link.Measure(pose, scene.images[link.neighborID]);
					if (!d.HasDirection())
						continue;
					(d.direction <= d.DirectionTolerance(config.maxLinkDirectionError) ? agreeing : disagreeing) += link.numInliers;
				}
				if (disagreeing > agreeing) {
					DEBUG("warning: rejected the pose of image %u: %u/%u inliers (%.1f%%) and its centre lies in a direction "
						"the pairs carrying %u of the %u weighted inliers of the quorum links that fix one contradict "
						"(strongest to image %u)",
						imageID, numInliers, n, inlierRatio * 100.f, disagreeing, agreeing + disagreeing,
						quorum.links.front().neighborID);
					return {0, n, numDescribed};
				}
			}
		}
	}

	static_cast<Pose3D&>(img) = pose;
	return {numInliers, n, numDescribed};
}

float Resection::RegisteredEvidenceShare(IIndex imageID) const
{
	float registered = 0.f, total = 0.f;
	for (const ImagePair& pair : scene.pairs) {
		if (pair.ID1 != imageID && pair.ID2 != imageID)
			continue;
		const float weight = pair.GetCompositeWeight();
		if (weight <= 0.f)
			continue;
		total += weight;
		if (scene.images[pair.ID1 == imageID ? pair.ID2 : pair.ID1].HasPose())
			registered += weight;
	}
	return total > 0.f ? registered / total : 1.f;
}

float Resection::RegisteredImagesFraction() const
{
	unsigned numImages = 0, numRegistered = 0;
	for (const Image& img : scene.images) {
		if (!img.HasCamera())
			continue;
		++numImages;
		if (img.HasPose())
			++numRegistered;
	}
	return numImages > 0 ? numRegistered / (float)numImages : 0.f;
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

void Resection::ExcludeImages(const IIndexArr& imageIDs)
{
	excludedImages.insert(imageIDs.begin(), imageIDs.end());
}

unsigned SFM::AdjustRepeated(Scene& scene, const BAConfig& baConfig, const ResectionConfig& config)
{
	ASSERT(config.maxReprojError > 0.f);
	const float inputErrorPixels = config.maxReprojError * ResectionConfig::fullBAInputErrorFactor;
	const auto Filter = [&](float bar) {
		FilterTracks(scene, bar, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	};
	// per image and keypoint: bit 1 = in the input of the solve that ran, bit 2 = in the input of the next one
	std::vector<std::vector<uint8_t>> input(config.fullBARepeats > 0 ? scene.images.size() : 0);
	FOREACH(i, input)
		if (scene.images[i].IsValid())
			input[i].assign(scene.images[i].keypoints.size(), 0);
	const auto Mark = [&](uint8_t bit) {
		for (const Track& track : scene.tracks) {
			if (!track.IsInlier())
				continue;
			for (const Observation& obs : track) {
				ASSERT(obs.featureID < input[obs.imageID].size()); // an inlier prefix holds valid images only
				input[obs.imageID][obs.featureID] |= bit;
			}
		}
	};
	Filter(inputErrorPixels);
	unsigned solves = 0;
	double change = 0;
	for (;;) {
		if (!input.empty())
			Mark(1);
		BundleAdjustment::Adjust(scene, baConfig);
		++solves;
		Filter(config.maxReprojError);
		if (solves > config.fullBARepeats)
			break;
		// the tracks left without a point triangulated again, and what the next solve would fit from the model
		TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
		Filter(inputErrorPixels);
		Mark(2);
		// the largest share of one camera's input that the next solve's input adds or removes
		change = 0;
		for (std::vector<uint8_t>& bits : input) {
			unsigned before = 0, changed = 0;
			for (uint8_t& b : bits) {
				before += b & 1;
				changed += b == 1 || b == 2;
				b = 0;
			}
			if (before >= ResectionConfig::fullBARepeatMinObservations)
				change = MAXF(change, (double)changed / before);
		}
		if (change < ResectionConfig::fullBARepeatMinChange) {
			Filter(config.maxReprojError);
			break;
		}
	}
	DEBUG("Full adjustment ran %u solve(s), the last input change %.1f%% of a camera's input", solves, change * 100);
	return solves;
}

bool Resection::RegisterImages()
{
	TD_TIMER_STARTD();

	// Collect unregistered images
	IIndexScores unregistered;
	unregistered.reserve(scene.images.size() * 2 / 3);
	for (const Image& img : scene.images)
		if (!img.HasPose() && !excludedImages.count(img.ID))
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
	// Run the bundle adjustment the registration count has come due for, if any; true when one ran,
	// in which case the selection of the next images has to start over
	const auto AdjustIfScheduled = [&]() {
		if ((config.fullBAEvery[nBA] > 0 && sinceFullBA >= config.fullBAEvery[nBA]) ||
			(config.avgInliersRatioForceBA > 0.f && sinceFullBA >= config.minImagesForceBA &&
			 avgInliersRatio.GetAverage() < config.avgInliersRatioForceBA)) {
			// Full BA every N registered images;
			// filter first so the observations of the images registered since the last filtering enter the
			// inlier prefix the BA iterates, then re-triangulate only the tracks left without a valid
			// position (real outliers and never-triangulated tracks): a track that merely gained a newly
			// valid view keeps the position the previous BA refined instead of being reset to a linear solve. The
			// adjustment fits a wider bar than the model is held to and repeats while what it fits keeps changing
			FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
			TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
			const bool refineExtended = config.minRefineExtIntrs > 0 &&
				scene.status.nCalibratedImages + registeredCount >= config.minRefineExtIntrs;
			AdjustRepeated(scene, refineExtended ? config.extendedBAConfig : config.fullBAConfig, config);
			lastRegistered.clear();
			avgInliersRatio.Clear();
			sinceFullBA = 0;
			if (nBA + 1 < config.fullBAEvery.size())
				++nBA;
			return true;
		}
		if (config.localBAEvery > 0 && lastRegistered.size() >= config.localBAEvery) {
			// Local BA every N registered images
			TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
			const IIndexArr fixedViewIDs = BuildLocalWindow(lastRegistered);
			ASSERT(!fixedViewIDs.empty());
			BundleAdjustment::AdjustLocal(scene, lastRegistered, fixedViewIDs, config.localBAConfig);
			FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
			lastRegistered.clear();
			return true;
		}
		return false;
	};
	while (!unregistered.empty()) {
		IIndexArr nextIDs = SelectNextImages(unregistered);
		if (nextIDs.empty()) {
			VERBOSE("warning: no more connected images to register, %u images remain", unregistered.size());
			for (const auto& it : unregistered) {
				DEBUG_EXTRA("\timage %u ('%s'): %u 2D-3D correspondences (min %u)", it.first,
					Util::getFileName(scene.images[it.first].fileName).c_str(), it.second, config.minCorrespondences);
			}
			break;
		}
		const unsigned startRegisteredCount = registeredCount;
		for (IIndex n = 0; n < nextIDs.size(); ) {
			// Attempt to register next image
			const IIndex nextID = nextIDs[n];
			const auto [numInliers, numPoints, numDescribed] = RegisterImage(nextID);
			if (numInliers == 0) {
				DEBUG("warning: failed to register image %u (%u/%u correspondences), retrying later", nextID, numInliers, numPoints);
				nextIDs.RemoveAtMove(n);
				continue; // n now points to the shifted element, do not increment
			}
			// The average the full bundle adjustment is forced from reads the registered images' inlier
			// ratios alone, weighed against what a registration with this same mix of described and
			// dense correspondences reaches on a healthy model, so it reads drift and not how much of
			// the image matched densely: an image whose attempt failed or was refused is not in the
			// model, so its zero says nothing about how well the model fits
			const float describedShare = numDescribed / (float)numPoints;
			const float expectedRatio = describedShare + (1.f - describedShare) * config.denseInlierRatioFactor;
			avgInliersRatio += (numInliers / (float)numPoints) / expectedRatio;
			lastRegistered.push_back(nextID);
			unregistered.erase(nextID);
			++registeredCount;
			++sinceFullBA;
			++n;
			DEBUG_EXTRA("\tImage %u registered: %u/%u correspondences (%u/%u images, %.2f%% avg normalized inliers ratio)",
				nextID, numInliers, numPoints, scene.status.nCalibratedImages+registeredCount, scene.images.size(), avgInliersRatio.GetAverage() * 100.f);
			if (AdjustIfScheduled()) {
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

	// Full BA to close the resection, unless the last one scheduled inside the loop already
	// covers every image registered since (nothing left for a further adjustment to refine)
	if (sinceFullBA > 0) {
		TriangulateTracks(scene, false, config.maxReprojError, config.minAngleThreshold);
		config.extendedBAConfig.maxIterations = 100;
		AdjustRepeated(scene, config.extendedBAConfig, config);
	}

	// Update scene status
	scene.status.nCalibratedImages += registeredCount;
	scene.status.nState.set(Scene::Status::STATE::CALIBRATED);
	DEBUG("Resection registered %u new images, total %u/%u images (%s)",
		registeredCount, scene.status.nCalibratedImages, scene.images.size(), TD_TIMER_GET_FMT().c_str());
	return registeredCount > 0;
}
/*----------------------------------------------------------------*/
