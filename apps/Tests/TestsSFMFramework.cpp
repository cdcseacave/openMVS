/*
 * TestsSFMFramework.cpp
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
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "TestsSFMFramework.h"
#include "../../libs/SFM/Triangulation.h"
#include "../../libs/SFM/SceneCluster.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSFM "));

namespace SFM {

TestScope::TestScope(const char* _name)
	:
	name(_name),
	start(Timer::GetSysTime())
{
	ASSERT(name != NULL && *name != 0);
}

bool TestScope::Pass(LPCTSTR fmt, ...) const
{
	va_list args;
	va_start(args, fmt);
	const String message(String::FormatStringSafe(fmt, args));
	va_end(args);
	VERBOSE("%s PASSED: %s (%s)", name, message.c_str(),
		Util::formatTime((Timer::SysType)Timer::SysTime2TimeMs(Timer::GetSysTime() - start)).c_str());
	return true;
}

bool TestScope::Fail(const char* file, int line, const char* condition, LPCTSTR fmt, ...) const
{
	va_list args;
	va_start(args, fmt);
	const String message(String::FormatStringSafe(fmt, args));
	va_end(args);
	const char* fileName = file;
	for (const char* c = file; *c != 0; ++c)
		if (*c == '/' || *c == '\\')
			fileName = c + 1;
	VERBOSE("%s FAILED: %s:%d: %s: %s", name, fileName, line, condition, message.c_str());
	return false;
}
/*----------------------------------------------------------------*/


// ===============================================================================
// Helper: Generate Test Scene with configurable cameras, images, and points
// ===============================================================================
// Helper: Generate random rotation
Matrix3x3 GenerateRandomRotation(std::mt19937& rng, REAL angleRng) {
	std::uniform_real_distribution<REAL> angleDist(-angleRng, angleRng);
	Eigen::Vector3d axis(angleDist(rng), angleDist(rng), angleDist(rng));
	axis.normalize();
	REAL angle = angleDist(rng);
	Eigen::AngleAxisd aa(angle, axis);
	Matrix3x3 R_rel = aa.toRotationMatrix();
	return R_rel;
}
// Helper: Generate random translation
static Point3 GenerateRandomTranslation(std::mt19937& rng, REAL transRng = 1.0) {
	std::uniform_real_distribution<REAL> transDist(-transRng, transRng);
	Point3 t_rel;
	do
		t_rel = Point3(transDist(rng), transDist(rng), transDist(rng));
	while (norm(t_rel) < 0.1);
	return t_rel;
}
// Helper: Generate random pose
static Pose3D GenerateRandomPose(std::mt19937& rng, REAL angleRng = 0.3, REAL transRng = 1.0) {
	Matrix3x3 R_rel = GenerateRandomRotation(rng, angleRng);
	Point3 t_rel = GenerateRandomTranslation(rng, transRng);
	Pose3D pose;
	pose.R = R_rel;
	pose.SetT(t_rel);
	return pose;
}

// GenerateTestScene: Returns ground truth scene, optionally perturbed scene
void GenerateTestScene(Scene& scene, const SceneConfig& cfg, Scene* scenePerturbed) {
	std::mt19937 rng(cfg.randomSeed);

	// Create cameras
	for (const auto& camSpec : cfg.cameras) {
		Camera* cam = nullptr;
		if (camSpec.type == SceneConfig::PINHOLE) {
			PinholeCamera* pinhole = new PinholeCamera(cv::Size(camSpec.width, camSpec.height),
			                        camSpec.focal, camSpec.focal,
			                        camSpec.cx, camSpec.cy);
			pinhole->k1 = camSpec.k1;
			pinhole->k2 = camSpec.k2;
			// intrinsics come straight from the ground-truth spec, so they are trusted:
			// tests exercising the calibrated (essential-matrix) matching branch rely on it;
			// tests exercising focal refinement clear the flag explicitly
			pinhole->trustIntrinsics = true;
			cam = pinhole;
		} else {
			cam = new SphericalCamera(cv::Size(camSpec.width, camSpec.height));
		}
		scene.cameras.emplace_back(cam);
	}

	// Generate poses based on mode
	std::vector<Pose3D> poses;
	if (cfg.poseMode == SceneConfig::RANDOM_POSES) {
		Pose3D pose = GenerateRandomPose(rng);
		poses.push_back(pose);
		for (unsigned i = 1; i < cfg.numImages; ++i) {
			pose = GenerateRandomPose(rng) * pose;
			poses.push_back(pose);
		}
	} else if (cfg.poseMode == SceneConfig::CIRCULAR_ARRANGEMENT) {
		for (unsigned i = 0; i < cfg.numImages; ++i) {
			const REAL angle = D2R(i * cfg.rotationAngleStep);
			Pose3D pose;
			pose.C.x = cfg.circularRadius * COS(angle);
			pose.C.y = 0;
			pose.C.z = cfg.circularRadius * SIN(angle);
			// Camera looks at origin, Up is (0,1,0)
			pose.R.LookAt(pose.C, Point3(0,0,0), Point3(0,1,0));
			poses.push_back(pose);
		}
	} else { // SIMPLE_TRANSLATION
		for (unsigned i = 0; i < cfg.numImages; ++i) {
			Pose3D pose;
			if (cfg.addPoseRotations) {
				const REAL angle = D2R(i * cfg.rotationAngleStep);
				pose.R = Matrix3x3(
					COS(angle), 0, SIN(angle),
					0, 1, 0,
					-SIN(angle), 0, COS(angle)
				);
			} else {
				pose.R = Matrix3x3::IDENTITY;
			}
			pose.C.x = i * cfg.cameraSeparation;
			pose.C.y = (i % 2) * cfg.cameraSeparation * 0.5;
			pose.C.z = 0;
			poses.push_back(pose);
		}
	}

	// Create images
	for (unsigned i = 0; i < cfg.numImages; ++i) {
		const IIndex camID = static_cast<IIndex>(i % cfg.cameras.size());
		Camera* cam = scene.cameras[camID];
		scene.images.emplace_back(static_cast<IIndex>(i), "", poses[i], camID, cam);
	}
	scene.status.nCalibratedImages = scene.images.size();

	// Generate 3D points and project to all images
	if (cfg.numPoints > 0) {
		Image& img0 = scene.images[0];
		std::uniform_int_distribution<int> pixelWidthDist(10, img0.GetWidth() - 10);
		std::uniform_int_distribution<int> pixelHeightDist(10, img0.GetHeight() - 10);
		std::uniform_real_distribution<REAL> depthDist(1, 10);
		for (unsigned p = 0; p < cfg.numPoints; ++p) {
			Point2 pixel(pixelWidthDist(rng), pixelHeightDist(rng));
			REAL depth = depthDist(rng);
			Point3 X = img0.UnprojectPoint(pixel, depth);
			Track track(X);
			for (unsigned v = 0; v < cfg.numImages; ++v) {
				Image& img = scene.images[v];
				const auto [proj, valid] = img.ProjectPoint(X);
				if (!valid || !Image8U::isInside(proj, img.GetSize()))
					continue;
				const uint32_t featID = static_cast<uint32_t>(img.keypoints.size());
				img.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
				track.observations.emplace_back(img.ID, featID);
			}
			if (track.observations.size() < 2) {
				// Regenerate track if not enough observations
				// and remove added keypoints
				for (const auto& obs : track.observations)
					scene.images[obs.imageID].keypoints.pop_back();
				--p;
				continue;
			}
			track.numInliers = static_cast<uint8_t>(track.observations.size());
			scene.tracks.emplace_back(std::move(track));
		}
		scene.status.nTracks = scene.tracks.size();
		scene.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);
	}

	// Generate descriptors if requested
	if (cfg.generateDescriptors) {
		// First pass: generate random descriptors for all keypoints
		std::uniform_int_distribution<int> byteDist(0, 255);
		for (Image& img : scene.images) {
			const size_t numKeypoints = img.keypoints.size();
			img.descriptors.create((int)numKeypoints, cfg.descriptorDim, CV_8U);
			for (size_t k = 0; k < numKeypoints; ++k) {
				uint8_t* desc = img.descriptors.ptr<uint8_t>((int)k);
				for (int d = 0; d < cfg.descriptorDim; ++d)
					desc[d] = (uint8_t)byteDist(rng);
			}
		}
		// Second pass: make corresponding keypoints have similar descriptors
		std::normal_distribution<float> noise(0.f, cfg.binaryDescriptors ? 2.f : 8.f);
		for (const Track& track : scene.tracks) {
			// Use the first observation's descriptor as the base
			ASSERT(!track.observations.empty());
			const auto& firstObs = track.observations[0];
			const Image& firstImg = scene.images[firstObs.imageID];
			const uint8_t* baseDesc = firstImg.descriptors.ptr<uint8_t>((int)firstObs.featureID);
			// Regenerate descriptors for remaining observations as noisy versions
			for (size_t i = 1; i < track.observations.size(); ++i) {
				const auto& obs = track.observations[i];
				Image& img = scene.images[obs.imageID];
				uint8_t* desc = img.descriptors.ptr<uint8_t>((int)obs.featureID);
				for (int d = 0; d < cfg.descriptorDim; ++d) {
					int v = ROUND2INT((float)baseDesc[d] + noise(rng));
					desc[d] = (uint8_t)CLAMP(v, 0, 255);
				}
			}
		}
		scene.status.nFeaturesType = (cfg.binaryDescriptors ? FeatureType::AKAZE : FeatureType::SIFT);
	}

	// Generate image pairs with matches if requested
	if (cfg.generatePairs) {
		const unsigned nImages = static_cast<unsigned>(scene.images.size());
		// Create pairs for all image combinations
		for (unsigned i = 0; i + 1 < nImages; ++i) {
			for (unsigned j = i + 1; j < nImages; ++j) {
				ImagePair& pair = scene.pairs.emplace_back(i, j);
				// Build matches from track observations
				for (const Track& track : scene.tracks) {
					uint32_t obs_i = NO_ID, obs_j = NO_ID;
					for (const auto& obs : track.observations) {
						if (obs.imageID == i) obs_i = obs.featureID;
						else if (obs.imageID == j) obs_j = obs.featureID;
					}
					if (obs_i != NO_ID && obs_j != NO_ID)
						pair.matches.emplace_back(obs_i, obs_j);
				}
				// Compute ground truth relative pose
				pair.relativePose = scene.images[j] / scene.images[i];
				pair.E = ImagePair::ComposeEssentialMatrix(pair.relativePose.value());
				pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[i].GetK(), scene.images[j].GetK());
			}
		}
		scene.status.nState.set(Scene::Status::STATE::MATCHED);
	}

	// Generate GPS metadata if requested
	if (cfg.generateGPS) {
		// Base GPS location: Mountain View, CA (Googleplex)
		const double base_latitude = 37.3861;  // degrees North
		const double base_longitude = -122.0839; // degrees West
		const double base_altitude = 30.0;      // meters (approximate)
		// GPS conversion factors (approximate)
		const double metersPerDegLat = 111132.0; // meters per degree latitude
		for (Image& img : scene.images) {
			// Convert camera position (meters) to GPS offset
			// Camera coordinate system: X=East, Y=North, Z=Up (assumed)
			const double lat_offset_deg = img.C.y / metersPerDegLat;
			const double lat = base_latitude + lat_offset_deg;
			// Longitude offset depends on latitude (cosine correction)
			const double metersPerDegLon = metersPerDegLat * COS(D2R(lat));
			const double lon_offset_deg = img.C.x / metersPerDegLon;
			const double lon = base_longitude + lon_offset_deg;
			const double alt = base_altitude + img.C.z;
			// Set GPS metadata via View cast
			View::Metadata& meta = static_cast<View&>(img).metadata;
			meta.latitude = lat;
			meta.longitude = lon;
			meta.altitude = alt;
			meta.positionAccuracy = 0.1;  // 10cm horizontal accuracy
			meta.positionAccuracyZ = 0.5; // 50cm vertical accuracy
		}
		// Align to GPS first
		scene.AlignToGPS();
	}

	// Create perturbed copy if requested
	if (scenePerturbed) {
		*scenePerturbed = scene;
		std::uniform_real_distribution<REAL> perturbDist(-0.01, 0.01);
		// Optionally perturb intrinsics
		if (cfg.perturbOptions & SceneConfig::PERTURB_INTRINSICS) {
			for (Camera* cam : scenePerturbed->cameras)
				if (auto* pinhole = dynamic_cast<PinholeCamera*>(cam))
					pinhole->fy = pinhole->fx += pinhole->fx * perturbDist(rng) * 3.0; // 3% noise
		}
		// Optionally perturb poses
		if (cfg.perturbOptions & SceneConfig::PERTURB_POSES) {
			for (Image& img : scenePerturbed->images) {
				img.C.x += perturbDist(rng) * 0.01; // 1cm translation noise
				img.C.y += perturbDist(rng) * 0.01;
				img.C.z += perturbDist(rng) * 0.01;
				img.R = img.R * GenerateRandomRotation(rng, 0.01); // 0.1 rad rotation noise
			}
		}
		// Optionally perturb pairwise poses
		if (cfg.perturbOptions & SceneConfig::PERTURB_PAIR_POSES) {
			for (ImagePair& pair : scenePerturbed->pairs) {
				pair.relativePose->C.x += perturbDist(rng) * 0.01; // 1cm translation noise
				pair.relativePose->C.y += perturbDist(rng) * 0.01;
				pair.relativePose->C.z += perturbDist(rng) * 0.01;
				pair.relativePose->R = pair.relativePose->R * GenerateRandomRotation(rng, 0.01); // 0.1 rad rotation noise
			}
		}
		// Optionally perturb keypoints
		if (cfg.perturbOptions & SceneConfig::PERTURB_KEYPOINTS) {
			for (Image& img : scenePerturbed->images) {
				for (auto& kp : img.keypoints) {
					kp.pt.x += static_cast<float>(perturbDist(rng) * 10.0); // 0.1 pixel noise
					kp.pt.y += static_cast<float>(perturbDist(rng) * 10.0);
				}
			}
		}
		// Optionally perturb tracks
		if (cfg.perturbOptions & SceneConfig::PERTURB_POINTS) {
			for (Track& track : scenePerturbed->tracks) {
				track.position.x += perturbDist(rng) * 0.5; // 0.5cm noise
				track.position.y += perturbDist(rng) * 0.5;
				track.position.z += perturbDist(rng) * 0.5;
			}
		}
	}
}
/*----------------------------------------------------------------*/


// ===============================================================================
// Helper: Generate a scene with known 2-cluster structure for clustering tests
// ===============================================================================
void GenerateTwoClusterScene(
	Scene& scene,
	unsigned clusterSizeA,
	unsigned clusterSizeB,
	unsigned numCrossPairs,
	unsigned matchesPerCrossPair,
	unsigned numPoints,
	uint32_t seed,
	bool hubBridge)
{
	const unsigned totalImages = clusterSizeA + clusterSizeB;
	SceneConfig cfg;
	cfg.randomSeed = seed;
	cfg.numImages = totalImages;
	cfg.numPoints = numPoints;
	cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	cfg.rotationAngleStep = 360.0 / totalImages;
	cfg.generateDescriptors = true;
	cfg.generatePairs = true;
	GenerateTestScene(scene, cfg);

	// Choose the cross-cluster pairs to keep before touching the array, preferring the ones that
	// bring in an image neither side has used yet: a bridge spread over distinct cameras is what a
	// seam looks like, and it is what an alignment registering one side's cameras needs.
	// With hubBridge the choice is the opposite one: every cross pair runs through a single image
	// of cluster A, so that side of the seam has one camera to register with -- too few to observe
	// a scale -- while the other side keeps one camera per pair.
	std::vector<bool> keepCross(scene.pairs.size(), false);
	if (hubBridge) {
		const IIndex hub = clusterSizeA - 1;
		unsigned kept = 0;
		FOREACH(i, scene.pairs) {
			if (kept >= numCrossPairs)
				break;
			const ImagePair& pair = scene.pairs[i];
			if ((pair.ID1 < clusterSizeA) == (pair.ID2 < clusterSizeA) || (pair.ID1 != hub && pair.ID2 != hub))
				continue;
			keepCross[i] = true;
			++kept;
		}
	} else {
		std::set<IIndex> usedA, usedB;
		unsigned kept = 0;
		for (unsigned pass = 0; pass < 2 && kept < numCrossPairs; ++pass) {
			FOREACH(i, scene.pairs) {
				if (kept >= numCrossPairs)
					break;
				const ImagePair& pair = scene.pairs[i];
				if (keepCross[i] || (pair.ID1 < clusterSizeA) == (pair.ID2 < clusterSizeA))
					continue;
				const IIndex idA = pair.ID1 < clusterSizeA ? pair.ID1 : pair.ID2;
				const IIndex idB = pair.ID1 < clusterSizeA ? pair.ID2 : pair.ID1;
				if (pass == 0 && (usedA.count(idA) || usedB.count(idB)))
					continue;
				usedA.insert(idA);
				usedB.insert(idB);
				keepCross[i] = true;
				++kept;
			}
		}
	}

	// Weight intra-cluster pairs high, cross-cluster pairs low or remove them
	// Cluster A: images [0, clusterSizeA), Cluster B: images [clusterSizeA, totalImages)
	RFOREACH(i, scene.pairs) {
		ImagePair& pair = scene.pairs[i];
		const bool inA = pair.ID1 < clusterSizeA && pair.ID2 < clusterSizeA;
		const bool inB = pair.ID1 >= clusterSizeA && pair.ID2 >= clusterSizeA;
		if (inA || inB) {
			// Intra-cluster: keep all matches, boost weight
			pair.weightSpatial = 10.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 10.f;
			pair.weightTriplet = 10.f;
		} else if (keepCross[i]) {
			// Cross-cluster pair: keep but with fewer matches
			if (pair.matches.size() > matchesPerCrossPair)
				pair.matches.resize(matchesPerCrossPair);
			pair.weightSpatial = 1.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 1.f;
			pair.weightTriplet = 0.f;
		} else {
			scene.pairs.RemoveAtMove(i);
		}
	}
}
/*----------------------------------------------------------------*/


ArcScene::ArcScene(unsigned numImages, uint32_t seed)
	:
	rng(seed)
{
	ASSERT(numImages > 0);
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), REAL(500), REAL(500), REAL(320), REAL(240)));
	for (unsigned i = 0; i < numImages; ++i) {
		truth.push_back(Pose(i * STEP));
		scene.images.emplace_back(IIndex(i), String::FormatString("%u.jpg", i), truth.back(), IIndex(0), scene.cameras[0]);
	}
	scene.status.nCalibratedImages = numImages;
}

Pose3D ArcScene::Pose(REAL angleDeg)
{
	const REAL angle = D2R(angleDeg);
	Pose3D pose;
	pose.C = Point3(RADIUS * COS(angle), 0, RADIUS * SIN(angle));
	pose.R.LookAt(pose.C, Point3(0, 0, 0), Point3(0, 1, 0));
	return pose;
}

void ArcScene::Place(IIndex imageID, REAL angleDeg)
{
	ASSERT(imageID < scene.images.size() && scene.tracks.empty());
	truth[imageID] = Pose(angleDeg);
	static_cast<Pose3D&>(scene.images[imageID]) = truth[imageID];
}

void ArcScene::Tracks(const IIndexArr& imageIDs, unsigned numTracks)
{
	ASSERT(!imageIDs.empty());
	std::uniform_real_distribution<REAL> coord(-1, 1);
	for (unsigned n = 0; n < numTracks; ) {
		const Point3 X(coord(rng), coord(rng), coord(rng));
		std::vector<Point2> projections;
		for (IIndex imageID : imageIDs) {
			const Image& img = scene.images[imageID];
			const auto [proj, valid] = img.ProjectPoint(X);
			if (!valid || !Image8U::isInside(proj, img.GetSize()))
				break;
			projections.push_back(proj);
		}
		if (projections.size() != imageIDs.size())
			continue;
		Track track(X);
		FOREACH(i, imageIDs) {
			Image& img = scene.images[imageIDs[i]];
			track.observations.emplace_back(imageIDs[i], (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(cv::Point2f((float)projections[i].x, (float)projections[i].y), 1.f);
		}
		scene.tracks.emplace_back(std::move(track));
		++n;
	}
}

void ArcScene::Correspondences(IIndex imageID, unsigned numTrue, unsigned numNoise, const Pose3D* projecting)
{
	ASSERT(numTrue + numNoise <= scene.tracks.size());
	Image& img = scene.images[imageID];
	const Pose3D own(img);
	if (projecting != NULL)
		static_cast<Pose3D&>(img) = *projecting;
	std::uniform_real_distribution<float> pixelX(10.f, 630.f), pixelY(10.f, 470.f);
	for (unsigned i = 0; i < numTrue + numNoise; ++i) {
		Track& track = scene.tracks[i];
		cv::Point2f pt(pixelX(rng), pixelY(rng));
		if (i < numTrue) {
			const auto [proj, valid] = img.ProjectPoint(track.position);
			ASSERT(valid && Image8U::isInside(proj, img.GetSize()));
			pt = cv::Point2f((float)proj.x, (float)proj.y);
		}
		track.observations.emplace_back(imageID, (uint32_t)img.keypoints.size());
		img.keypoints.emplace_back(pt, 1.f);
	}
	static_cast<Pose3D&>(img) = own;
}

ImagePair& ArcScene::Pair(IIndex idA, IIndex idB, unsigned numInliers, const ArcLinkSpec& spec)
{
	ASSERT(idA != idB && MAXF(idA, idB) < scene.images.size());
	ImagePair& pair = scene.pairs.emplace_back(MINF(idA, idB), MAXF(idA, idB));
	pair.relativePose = scene.images[pair.ID2] / scene.images[pair.ID1];
	pair.weightedInliers = (float)numInliers;
	pair.weightSpatial = 1.f;
	pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
	pair.weightConnectivity = 1.f;
	pair.meanRayAngle = (float)D2R(spec.rayAngleDeg);
	if (spec.turnTranslationDeg != 0) {
		// a unit vector perpendicular to the translation mixes into it with the sine of the angle
		Point3& C = pair.relativePose->C;
		const REAL length = norm(C);
		ASSERT(length > 0);
		const Point3 dir(C / length);
		Point3 perp(dir.cross(Point3(0, 1, 0)));
		if (norm(perp) < 1e-6)
			perp = dir.cross(Point3(1, 0, 0));
		perp /= norm(perp);
		const REAL turn = D2R(spec.turnTranslationDeg);
		C = (dir * COS(turn) + perp * SIN(turn)) * length;
	}
	if (spec.turnRotationDeg != 0)
		pair.relativePose->R = RMatrix(Point3(0, D2R(spec.turnRotationDeg), 0)) * pair.relativePose->R;
	return pair;
}

void ArcScene::Unregister(const IIndexArr& imageIDs)
{
	for (IIndex imageID : imageIDs) {
		ASSERT(scene.images[imageID].HasPose());
		scene.images[imageID].InvalidatePose();
	}
	scene.status.nCalibratedImages = (unsigned)(scene.images.size() - imageIDs.size());
	TriangulateTracks(scene, false, 4.f, 1.f);
}
/*----------------------------------------------------------------*/


// ===============================================================================
// The ring of the merge tests
// ===============================================================================
// Two images belong to a pair only when this many tracks join them, the bar a real matcher's
// geometric verification leaves behind
static constexpr unsigned kRingMinSharedTracks = 20;
// How far a ring camera may be tilted off the radial direction
static constexpr REAL kRingTiltDeg = 10;

// One camera of the ring: its centre on the circle at the given angle, moved along the tangent by
// `tangentOffset`, looking radially outward and tilted up or down by a random angle. The tilt turns
// the camera about its own right axis and leaves it pointing outward, so how far around the wall a
// camera reaches -- which decides what two cameras share and therefore where the blocks meet -- is
// the same for every camera of the ring.
static Pose3D RingCamera(REAL angle, REAL radius, REAL tangentOffset, std::mt19937& rng)
{
	const REAL c = COS(angle), s = SIN(angle);
	Pose3D pose;
	pose.C = Point3(radius * c - tangentOffset * s, radius * s + tangentOffset * c, 0);
	std::uniform_real_distribution<REAL> tiltDist(-kRingTiltDeg, kRingTiltDeg);
	RMatrix tilt;
	tilt.SetRotationAxisAngle(Point3(D2R(tiltDist(rng)), 0, 0));
	// the camera axes as the rows of the world-to-camera rotation: right along the tangent, down
	// along the ring axis, forward radially outward
	pose.R = RMatrix(tilt * Matrix3x3(
		 s, -c,  0,
		 0,  0, -1,
		 c,  s,  0));
	return pose;
}

void GenerateRingScene(const RingSceneConfig& cfg, Scene& scene, std::vector<IIndexArr>& blocks, std::vector<Pose3D>& gtPoses)
{
	std::mt19937 rng(cfg.seed);
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), REAL(400), REAL(400), REAL(320), REAL(240)));

	// the cameras, block by block, so every block owns a contiguous arc and contiguous global IDs
	const REAL slotAngle = REAL(2) * REAL(M_PI) / REAL(cfg.numBlocks * cfg.camsPerBlock);
	blocks.assign(cfg.numBlocks, IIndexArr());
	const auto AddCamera = [&](uint32_t block, const Pose3D& pose) {
		blocks[block].push_back((IIndex)scene.images.size());
		scene.images.emplace_back((IIndex)scene.images.size(), String(), pose, 0, scene.cameras[0]);
	};
	for (uint32_t b = 0; b < cfg.numBlocks; ++b) {
		if (cfg.tinyBlocks.count(b) > 0) {
			// two cameras a millimetre apart at the centre of the block's arc: a rig too short to
			// observe a scale of its own
			const REAL angle = slotAngle * (REAL(b * cfg.camsPerBlock) + REAL(cfg.camsPerBlock - 1) / 2);
			AddCamera(b, RingCamera(angle, cfg.cameraRadius, REAL(-0.0005), rng));
			AddCamera(b, RingCamera(angle, cfg.cameraRadius, REAL(0.0005), rng));
			continue;
		}
		for (unsigned k = 0; k < cfg.camsPerBlock; ++k)
			AddCamera(b, RingCamera(slotAngle * REAL(b * cfg.camsPerBlock + k), cfg.cameraRadius, 0, rng));
	}
	scene.status.nCalibratedImages = scene.images.size();
	gtPoses.assign(scene.images.size(), Pose3D());
	FOREACH(i, scene.images) {
		gtPoses[i].R = scene.images[i].R;
		gtPoses[i].C = scene.images[i].C;
	}

	// the wall the ring looks at, and what every camera sees of it
	std::uniform_real_distribution<REAL> wallAngleDist(0, REAL(2) * REAL(M_PI));
	std::uniform_real_distribution<REAL> wallHeightDist(-cfg.wallHeight / 2, cfg.wallHeight / 2);
	std::normal_distribution<REAL> keypointNoise(0, cfg.noisePx);
	for (unsigned p = 0; p < cfg.numPoints; ++p) {
		const REAL a = wallAngleDist(rng);
		const Point3 X(cfg.wallRadius * COS(a), cfg.wallRadius * SIN(a), wallHeightDist(rng));
		std::vector<std::pair<IIndex, Point2>> projections;
		FOREACH(i, scene.images) {
			const auto [proj, valid] = scene.images[i].ProjectPoint(X);
			if (valid && Image8U::isInside(proj, scene.images[i].GetSize()))
				projections.emplace_back(i, proj);
		}
		if (projections.size() < 2)
			continue;
		Track track(X);
		for (const auto& [image, proj] : projections) {
			Image& img = scene.images[image];
			track.observations.emplace_back(image, (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(Point2(proj.x + keypointNoise(rng), proj.y + keypointNoise(rng)), 0.f, 0.f, 10.f);
		}
		track.numInliers = (uint8_t)MINF(track.observations.size(), (IIndex)UINT8_MAX);
		scene.tracks.emplace_back(std::move(track));
	}

	// a pair per image couple the tracks join often enough, matched through those very tracks
	std::unordered_map<PairIdx::PairIndex, unsigned> numShared;
	const auto ForEachTrackPair = [&scene](const std::function<void(const Observation&, const Observation&)>& visit) {
		for (const Track& track : scene.tracks)
			FOREACH(i, track.observations)
				for (IIndex j = i + 1; j < track.observations.size(); ++j)
					visit(track.observations[i], track.observations[j]);
	};
	ForEachTrackPair([&numShared](const Observation& first, const Observation& second) {
		++numShared[PairIdx(first.imageID, second.imageID).idx];
	});
	std::unordered_map<PairIdx::PairIndex, uint32_t> pairOf;
	for (const auto& [key, num] : numShared) {
		if (num < kRingMinSharedTracks)
			continue;
		const PairIdx images(key);
		pairOf.emplace(key, (uint32_t)scene.pairs.size());
		scene.pairs.emplace_back(images.i, images.j);
	}
	ForEachTrackPair([&scene, &pairOf](const Observation& first, const Observation& second) {
		const auto it = pairOf.find(PairIdx(first.imageID, second.imageID).idx);
		if (it != pairOf.end())
			scene.pairs[it->second].matches.emplace_back(first.featureID, second.featureID);
	});
	for (ImagePair& pair : scene.pairs) {
		pair.relativePose = scene.images[pair.ID2] / scene.images[pair.ID1];
		pair.E = ImagePair::ComposeEssentialMatrix(pair.relativePose.value());
		pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), scene.images[pair.ID1].GetK(), scene.images[pair.ID2].GetK());
	}
	scene.status.nTracks = scene.tracks.size();
	scene.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);
	scene.status.nState.set(Scene::Status::STATE::MATCHED);
	ComputePairsWeights(scene);
}
/*----------------------------------------------------------------*/


std::vector<SEACAVE::Transform> SplitOnTruth(Scene& scene, SceneBlocks& sceneBlocks, const std::vector<IIndexArr>& blocks,
	const std::vector<Pose3D>& truth, uint32_t seed, const std::function<void(uint32_t block, Scene& sub)>& distort)
{
	ASSERT(!blocks.empty() && truth.size() == scene.images.size() && sceneBlocks.blocks.empty());
	// the given blocks, each sorted; an image in none of them is in the pool
	ScenePartition partition;
	partition.clusters = blocks;
	std::vector<bool> inBlock(scene.images.size(), false);
	for (IIndexArr& cluster : partition.clusters) {
		cluster.Sort();
		for (IIndex g : cluster)
			inBlock[g] = true;
	}
	FOREACH(g, scene.images)
		if (!inBlock[g])
			partition.pool.push_back(g);
	sceneBlocks.Extract(partition);

	// the reconstruction hands the merge a scene without tracks; the blocks carry them
	scene.tracks.Release();
	scene.status.nTracks = 0;

	// the blocks carry the poses from here on: the global scene is the uncalibrated one a merge
	// writes its answer into, exactly as the reconstruction hands it over
	FOREACH(i, scene.images)
		scene.images[i].InvalidatePose();
	scene.status.nCalibratedImages = 0;

	std::mt19937 rng(seed);
	std::vector<SEACAVE::Transform> applied(sceneBlocks.blocks.size());
	FOREACH(b, sceneBlocks.blocks) {
		Scene& sub = sceneBlocks.blocks[b];
		FOREACH(localID, sub.images) {
			sub.images[localID].R = truth[sceneBlocks.localToGlobal[b][localID]].R;
			sub.images[localID].C = truth[sceneBlocks.localToGlobal[b][localID]].C;
		}
		if (distort)
			distort((uint32_t)b, sub);
		sub.RecomputeCalibratedImages();
		// the block's own points, from the cameras it ended up with; a rig a millimetre wide has
		// nothing left once the angle bar is applied, so it is lifted here
		for (Track& track : sub.tracks)
			if (track.observations.size() >= 2)
				TriangulateSkewLLS(track, sub.images, 4.f, 0.f);
		applied[b] = SEACAVE::Transform::Random(rng, 180, 10, 0.5);
		sub.Transform(applied[b]);
	}
	return applied;
}
/*----------------------------------------------------------------*/


RingFixture::RingFixture(const RingSceneConfig& _cfg)
	:
	cfg(_cfg),
	split(scene)
{
	GenerateRingScene(cfg, scene, blocks, truth);
}

void RingFixture::Split()
{
	applied = SplitOnTruth(scene, split, blocks, truth, cfg.seed + 1);
}

MergeReport RingFixture::Merge(bool releaseUnplaced, uint32_t anchorBlock, const std::unordered_set<IIndex>& poolImages,
	const GlobalAlignmentConfig& alignCfg)
{
	ASSERT(!split.blocks.empty());
	MergeReport report;
	GlobalAlignment alignment(scene, alignCfg, poolImages);
	alignment.MergeScenes(split.blocks, split.localToGlobal, report, releaseUnplaced, anchorBlock);
	return report;
}
/*----------------------------------------------------------------*/


ReconstructionConfig BundledImagesConfig()
{
	ReconstructionConfig cfg;
	cfg.featuresCfg.detectorType = FeatureType::AKAZE;
	cfg.featuresCfg.maxFeaturesPerCell = 900;
	cfg.featuresCfg.minFeaturesPerCell = 400;
	cfg.matchCfg.mode = MatchConfig::EXHAUSTIVE;
	cfg.matchCfg.DefaultsForFeatureType(cfg.featuresCfg.detectorType);
	return cfg;
}

bool ImportBundledImages(Scene& scene, const ImportConfig& importCfg)
{
	ASSERT(scene.images.empty());
	if (!scene.Import(MAKE_PATH("images"), importCfg)) {
		VERBOSE("error: the bundled images were not imported");
		return false;
	}
	if (scene.images.size() != 4) {
		#ifdef _IMAGE_HEIF
		constexpr const char* reason = "";
		#else
		constexpr const char* reason = " (built without libheif, so the two HEIC images were skipped)";
		#endif
		VERBOSE("error: %u of the 4 bundled images imported%s", (unsigned)scene.images.size(), reason);
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

} // namespace SFM
