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

namespace {

// A verified pair (valid weight and a known relative pose) joining an image to an already
// registered one, with the pair's relative pose turned around so that
//   pose(image) = relPose * pose(neighbor)
// holds whichever side of the pair the neighbor is on. ImagePair stores the relative pose as the
// transform from ID1 to ID2, so it is used as it is when the neighbor is ID1 and inverted when the
// neighbor is ID2 -- the same turn-around the star initializer applies around its reference view.
struct PoseLink
{
	IIndex neighborID;   // registered image on the other side of the pair
	unsigned numInliers; // weighted inliers of the pair
	Pose3D relPose;      // transform from the neighbor's frame to the image's frame
};

inline PoseLink MakePoseLink(const ImagePair& pair, IIndex neighborID)
{
	ASSERT(pair.relativePose.has_value() && (pair.ID1 == neighborID || pair.ID2 == neighborID));
	return PoseLink{ neighborID, pair.GetNumWeightedInliers(),
		pair.ID1 == neighborID ? pair.relativePose.value() : pair.relativePose->Inverse() };
}

// Order links by pair strength, ties by the lower neighbor ID so the choice never depends on the
// order the pairs happen to be stored in
inline bool IsStrongerLink(const PoseLink& a, const PoseLink& b)
{
	return a.numInliers > b.numInliers || (a.numInliers == b.numInliers && a.neighborID < b.neighborID);
}

// Does this pair connect the two images with a usable relative pose?
inline bool IsPoseLinkPair(const ImagePair& pair)
{
	return pair.relativePose.has_value() && pair.HasValidWeight();
}

// Strongest link of the given image to an already registered image; false if it has none
bool FindStrongestPoseLink(const Scene& scene, IIndex imageID, PoseLink& strongest)
{
	bool found = false;
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
		if (!scene.images[neighborID].HasPose())
			continue;
		const PoseLink link = MakePoseLink(pair, neighborID);
		if (!found || IsStrongerLink(link, strongest)) {
			strongest = link;
			found = true;
		}
	}
	return found;
}

// Median of the given values (the upper one of the two middle values for an even count)
inline REAL MedianValue(std::vector<REAL>& values)
{
	ASSERT(!values.empty());
	const size_t mid = values.size() / 2;
	std::nth_element(values.begin(), values.begin() + mid, values.end());
	return values[mid];
}

// Median distance between the given registered image's center and the centers of the registered
// images its verified pairs join it to; falls back to the median over every registered pair in the
// scene when it has no other registered neighbor. Returns 0 when the scene has no baseline at all.
REAL MedianBaseline(const Scene& scene, IIndex imageID)
{
	const Image& img = scene.images[imageID];
	std::vector<REAL> baselines;
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
		const Image& neighbor = scene.images[neighborID];
		if (neighbor.HasPose())
			baselines.push_back(norm(img.C - neighbor.C));
	}
	if (baselines.empty()) {
		for (const ImagePair& pair : scene.pairs) {
			if (!IsPoseLinkPair(pair))
				continue;
			const Image& img1 = scene.images[pair.ID1];
			const Image& img2 = scene.images[pair.ID2];
			if (img1.HasPose() && img2.HasPose())
				baselines.push_back(norm(img1.C - img2.C));
		}
	}
	if (baselines.empty())
		return 0;
	return MedianValue(baselines);
}

// One ray along which a registered image places the center of an unregistered one
struct CenterRay
{
	IIndex neighborID; // the registered image the ray starts from
	Point3 origin;     // the registered image's center, in world coordinates
	Point3 direction;  // unit direction towards the unregistered image's center, in world coordinates
};

// Least-squares point closest to every ray: minimizing the sum of the squared distances to the
// rays gives A*X = b with A = sum(I - d*d') and b = sum((I - d*d')*C).
bool ClosestPointToRays(const std::vector<CenterRay>& rays, Point3& X)
{
	ASSERT(rays.size() >= 2);
	Eigen::Matrix3d A(Eigen::Matrix3d::Zero());
	Eigen::Vector3d b(Eigen::Vector3d::Zero());
	for (const CenterRay& ray : rays) {
		const Eigen::Vector3d d(ray.direction.x, ray.direction.y, ray.direction.z);
		const Eigen::Vector3d c(ray.origin.x, ray.origin.y, ray.origin.z);
		const Eigen::Matrix3d P(Eigen::Matrix3d::Identity() - d * d.transpose());
		A += P;
		b += P * c;
	}
	const Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
	if (!lu.isInvertible())
		return false;
	const Eigen::Vector3d x = lu.solve(b);
	if (!x.allFinite())
		return false;
	X = Point3(x(0), x(1), x(2));
	return true;
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
		return {0, n};

	// A pose is accepted only when its support is credible. A small consensus inside a large set of
	// correspondences can agree on a pose the image never had, so the inliers must be a large enough
	// share of the correspondences, unless there are so many of them that the count vouches for the
	// pose whatever the share.
	const float inlierRatio = numInliers / (float)n;
	if (config.minInlierRatio > 0.f && inlierRatio < config.minInlierRatio && numInliers < config.minInliersAbsolute) {
		DEBUG("warning: rejected the pose of image %u: %u/%u inliers (%.1f%%) below the %.1f%% minimum share",
			imageID, numInliers, n, inlierRatio * 100.f, config.minInlierRatio * 100.f);
		return {0, n};
	}

	// Cross-check the estimated rotation against the one the strongest verified pair to an already
	// registered image composes: a weakly supported pose that contradicts its own pair is a
	// misregistration, while a well supported one is trusted over a pair that may itself be wrong.
	if (config.maxRelativeRotationError > 0.f && inlierRatio < 0.5f) {
		PoseLink link;
		if (FindStrongestPoseLink(scene, imageID, link)) {
			const RMatrix pairR(link.relPose.R * scene.images[link.neighborID].R);
			RMatrix poseR;
			poseR = camPose.R();
			const double angle = R2D(ACOS(ComputeAngle(poseR, pairR)));
			if (angle > config.maxRelativeRotationError) {
				DEBUG("warning: rejected the pose of image %u: %u/%u inliers (%.1f%%) and %.1f degrees away from the "
					"rotation of its strongest pair to image %u", imageID, numInliers, n, inlierRatio * 100.f,
					angle, link.neighborID);
				return {0, n};
			}
		}
	}

	img.R = camPose.R();
	img.SetT(camPose.t);
	return {numInliers, n};
}

IIndex Resection::RegisterFromRelativePoses(const IIndexScores& unregistered)
{
	// Gather, in one pass over the pairs, the links every unregistered image has to a registered one
	std::unordered_map<IIndex, std::vector<PoseLink>> links;
	for (const ImagePair& pair : scene.pairs) {
		if (!IsPoseLinkPair(pair))
			continue;
		const bool registered1 = scene.images[pair.ID1].HasPose();
		const bool registered2 = scene.images[pair.ID2].HasPose();
		if (registered1 == registered2)
			continue; // both registered, or neither: no link to add
		const IIndex imageID = registered1 ? pair.ID2 : pair.ID1;
		if (unregistered.find(imageID) == unregistered.end())
			continue;
		links[imageID].push_back(MakePoseLink(pair, registered1 ? pair.ID1 : pair.ID2));
	}

	// Candidates, ranked by the total inliers their links carry, ties to the lower image ID
	std::vector<IIndex> candidates;
	candidates.reserve(links.size());
	std::unordered_map<IIndex, unsigned> scores;
	scores.reserve(links.size());
	for (const auto& it : links) {
		unsigned score = 0;
		for (const PoseLink& link : it.second)
			score += link.numInliers;
		scores.emplace(it.first, score);
		candidates.push_back(it.first);
	}
	std::sort(candidates.begin(), candidates.end(), [&scores](IIndex a, IIndex b) {
		return scores.at(a) > scores.at(b) || (scores.at(a) == scores.at(b) && a < b);
	});
	// A candidate none of whose links can place a center is no reason to end the resection: try the
	// next one down the ranking instead, up to this many of them
	constexpr unsigned maxCandidates = 3;
	if (candidates.size() > maxCandidates)
		candidates.resize(maxCandidates);

	for (const IIndex imageID : candidates) {
		std::vector<PoseLink>& imageLinks = links[imageID];
		std::sort(imageLinks.begin(), imageLinks.end(), IsStrongerLink);

		// The rotation comes from the strongest link, composed with its neighbor's absolute pose
		const PoseLink& strongest = imageLinks.front();
		const RMatrix R(strongest.relPose.R * scene.images[strongest.neighborID].R);

		// Every link casts a ray from its neighbor's center along the direction in which the pair
		// places the image's center; the relative pose fixes that direction but not how far along it
		// the center lies, so the length is the only unknown left
		std::vector<CenterRay> rays;
		rays.reserve(imageLinks.size());
		for (const PoseLink& link : imageLinks) {
			const Image& neighbor = scene.images[link.neighborID];
			const Point3 direction(neighbor.R.t() * link.relPose.C);
			const REAL length = norm(direction);
			if (length > ZEROTOLERANCE<REAL>())
				rays.emplace_back(CenterRay{link.neighborID, neighbor.C, direction / length});
		}
		if (rays.empty()) {
			DEBUG("warning: cannot register image %u from its relative poses: none of its pairs has a baseline", imageID);
			continue;
		}

		CMatrix C;
		bool solved = false; // the center came out of the rays themselves, not out of the prior below
		if (rays.size() >= 2) {
			// Two or more rays fix the center where they come closest, provided they are far enough
			// from parallel for that point to be defined at all (the angle between the lines they
			// span, so that opposite directions count as parallel too) and that it does not land
			// behind one of the neighbors
			REAL minCosAngle = 1;
			for (size_t i = 0; i + 1 < rays.size(); ++i)
				for (size_t j = i + 1; j < rays.size(); ++j)
					minCosAngle = MINF(minCosAngle, ABS(rays[i].direction.dot(rays[j].direction)));
			const REAL angle = R2D(ACOS(minCosAngle));
			if (angle < 1) {
				DEBUG("warning: image %u has %u rays only %.3f degrees apart, placing its center on the strongest one alone",
					imageID, (unsigned)rays.size(), angle);
			} else if (!ClosestPointToRays(rays, C)) {
				DEBUG("warning: the %u rays of image %u do not meet, placing its center on the strongest one alone",
					imageID, (unsigned)rays.size());
			} else {
				bool ahead = true;
				for (const CenterRay& ray : rays) {
					if ((C - ray.origin).dot(ray.direction) <= 0) {
						DEBUG("warning: the center the %u rays of image %u meet at falls behind a neighbor, placing it on "
							"the strongest ray alone", (unsigned)rays.size(), imageID);
						ahead = false;
						break;
					}
				}
				solved = ahead;
			}
		}
		const char* path = solved ? "two rays" : (rays.size() >= 2 ? "one ray after a degenerate solve" : "one ray");
		if (!solved) {
			// One usable ray, or several the solve above could not use: the distance stays open, so
			// take the median distance the strongest ray's neighbor already has to its own registered
			// neighbors, which is what a capture moving steadily between views suggests
			const REAL baseline = MedianBaseline(scene, rays.front().neighborID);
			if (baseline <= 0) {
				DEBUG("warning: cannot register image %u from its relative poses: no baseline to scale its ray with", imageID);
				continue;
			}
			C = rays.front().origin + rays.front().direction * baseline;
		}

		Image& img = scene.images[imageID];
		static_cast<Pose3D&>(img) = Pose3D(R, C);
		// Its two-view tracks with the registered images can now be triangulated, which is what gives
		// the next selection the 2D-3D correspondences it was missing
		TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
		DEBUG("Image %u registered from the relative pose to image %u (%u neighbours, %s, baseline %s)",
			imageID, strongest.neighborID, (unsigned)imageLinks.size(), path,
			String::FormatString("%g", norm(C - scene.images[strongest.neighborID].C)).c_str());
		return imageID;
	}
	return NO_ID;
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
	unsigned relativePoseCount = 0;
	unsigned sinceFullBA = 0;
	IIndexArr lastRegistered;
	TRunningAverage<float, 10> avgInliersRatio;
	// Run the bundle adjustment the registration count has come due for, if any; true when one ran,
	// in which case the selection of the next images has to start over
	const auto AdjustIfScheduled = [&]() {
		if ((config.fullBAEvery[nBA] > 0 && sinceFullBA >= config.fullBAEvery[nBA]) ||
			(config.avgInliersRatioForceBA > 0.f && sinceFullBA >= config.minImagesForceBA &&
			 // an image registered from a relative pose has no inlier ratio to contribute, so the
			 // average may hold no measurement at all and must not be read as a low one
			 avgInliersRatio.GetCount() > 0 && avgInliersRatio.GetAverage() < config.avgInliersRatioForceBA)) {
			// Full BA every N registered images;
			// filter first so the observations of the images registered since the last filtering enter the
			// inlier prefix the BA iterates, then re-triangulate only the tracks left without a valid
			// position (real outliers and never-triangulated tracks): a track that merely gained a newly
			// valid view keeps the position the previous BA refined instead of being reset to a linear solve
			FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
			TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
			if (config.minRefineExtIntrs > 0 && scene.status.nCalibratedImages + registeredCount >= config.minRefineExtIntrs)
				config.fullBAConfig.RefineExtendedIntrinsics();
			BundleAdjustment::Adjust(scene, config.fullBAConfig);
			FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
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
	// When the resection stalls -- no image reaches the correspondence minimum, or every candidate
	// that did failed to register -- images joined to the model by a verified pair still have a
	// relative pose: register one of them from it so its two-view tracks can be triangulated, and
	// let the next selection pick up from there; false when no image can be registered that way
	const auto RegisterOneFromRelativePoses = [&]() {
		const IIndex imageID = config.relativePoseFallback ? RegisterFromRelativePoses(unregistered) : NO_ID;
		if (imageID == NO_ID)
			return false;
		lastRegistered.push_back(imageID);
		unregistered.erase(imageID);
		++registeredCount;
		++relativePoseCount;
		++sinceFullBA;
		AdjustIfScheduled();
		return true;
	};
	while (!unregistered.empty()) {
		IIndexArr nextIDs = SelectNextImages(unregistered);
		if (nextIDs.empty()) {
			if (RegisterOneFromRelativePoses())
				continue;
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
			if (AdjustIfScheduled()) {
				break; // restart selection of next images
			} else if (n+1 == nextIDs.size() || (config.triangulateEvery > 0 && (lastRegistered.size() % config.triangulateEvery) == 0)) {
				// Update scene with new points every N registered images
				TriangulateTracks(scene, true, config.maxReprojError, config.minAngleThreshold);
				FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
				break; // restart selection of next images
			}
		}
		if (registeredCount == startRegisteredCount && !RegisterOneFromRelativePoses()) {
			VERBOSE("warning: no images were registered in last iteration, stopping resection, %u images remain", unregistered.size());
			break;
		}
	}

	// Full BA after all images are registered (nothing changed if none were)
	if (registeredCount > 0) {
		TriangulateTracks(scene, false, config.maxReprojError, config.minAngleThreshold);
		config.fullBAConfig.maxIterations = 100;
		config.fullBAConfig.RefineExtendedIntrinsics();
		BundleAdjustment::Adjust(scene, config.fullBAConfig);
		FilterTracks(scene, config.maxReprojError, config.minAngleThreshold, config.multDepthNear, config.multDepthFar);
	}

	// Update scene status
	scene.status.nCalibratedImages += registeredCount;
	scene.status.nState.set(Scene::Status::STATE::CALIBRATED);
	DEBUG("Resection registered %u new images (%u from relative poses), total %u/%u images (%s)",
		registeredCount, relativePoseCount, scene.status.nCalibratedImages, scene.images.size(), TD_TIMER_GET_FMT().c_str());
	return registeredCount > 0;
}
/*----------------------------------------------------------------*/
