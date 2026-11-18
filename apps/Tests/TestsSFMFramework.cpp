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


SceneConfig RingConfig(unsigned numPoints)
{
	ASSERT(numPoints > 0);
	SceneConfig cfg;
	cfg.numImages = 16;
	cfg.numPoints = numPoints;
	cfg.rotationAngleStep = 16.0;
	cfg.generatePairs = true;
	return cfg;
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


PairGraph::PairGraph(Scene& _scene, IIndex numImages)
	:
	scene(_scene)
{
	ASSERT(scene.images.empty() && numImages >= 2);
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), REAL(600), REAL(600), REAL(320), REAL(240)));
	for (IIndex i = 0; i < numImages; ++i) {
		Image& img = scene.images.emplace_back(i, String::FormatString("%u.jpg", i));
		img.cameraID = 0;
		img.pCamera = scene.cameras[0];
	}
}

ImagePair& PairGraph::Pair(IIndex a, IIndex b, float weight)
{
	ASSERT(a != b && MAXF(a, b) < scene.images.size() && weight > 0.f);
	ImagePair& pair = scene.pairs.emplace_back(a, b);
	pair.weightedInliers = weight;
	pair.weightSpatial = 1.f;
	pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
	pair.weightConnectivity = 1.f;
	pair.weightTriplet = 0.75f;
	return pair;
}

ImagePair& PairGraph::Matched(IIndex a, IIndex b, unsigned numMatches, bool verified, unsigned cellsA, unsigned cellsB)
{
	ASSERT(MAXF(a, b) < scene.images.size() && cellsA >= 1 && cellsA <= 100 && cellsB >= 1 && cellsB <= 100);
	ImagePair& pair = scene.pairs.emplace_back(a, b);
	// the constructor orders ID1 < ID2: the keypoints are added to the images the matches will be read against
	Image& imageA = scene.images[pair.ID1];
	Image& imageB = scene.images[pair.ID2];
	const auto CellCentre = [](unsigned cell) {
		return cv::Point2f((float)(cell % 10) * 64.f + 32.f, (float)(cell / 10) * 48.f + 24.f);
	};
	for (unsigned m = 0; m < numMatches; ++m) {
		imageA.keypoints.emplace_back(CellCentre(m % cellsA), 1.f);
		imageB.keypoints.emplace_back(CellCentre(m % cellsB), 1.f);
		pair.matches.emplace_back((uint32_t)imageA.keypoints.size() - 1, (uint32_t)imageB.keypoints.size() - 1);
	}
	pair.numFilteredInliers = numMatches;
	if (verified)
		pair.F = Matrix3x3::IDENTITY;
	return pair;
}

void PairGraph::Hub(IIndex hub, IIndex last, float weight)
{
	ASSERT(hub < last);
	for (IIndex k = hub + 1; k <= last; ++k)
		Pair(hub, k, weight);
}

void PairGraph::Chain(IIndex first, IIndex last, float weight)
{
	ASSERT(first < last);
	for (IIndex k = first; k < last; ++k)
		Pair(k, k + 1, weight);
}

void PairGraph::Clique(IIndex first, IIndex last, float weight)
{
	ASSERT(first < last);
	for (IIndex a = first; a <= last; ++a)
		for (IIndex b = a + 1; b <= last; ++b)
			Pair(a, b, weight);
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

bool ArcScene::TrackAt(const Point3& X, const IIndexArr& imageIDs)
{
	ASSERT(!imageIDs.empty());
	std::vector<Point2> projections;
	for (IIndex imageID : imageIDs) {
		const Image& img = scene.images[imageID];
		const auto [proj, valid] = img.ProjectPoint(X);
		if (!valid || !Image8U::isInside(proj, img.GetSize()))
			return false;
		projections.push_back(proj);
	}
	Track track(X);
	FOREACH(i, imageIDs) {
		Image& img = scene.images[imageIDs[i]];
		track.observations.emplace_back(imageIDs[i], (uint32_t)img.keypoints.size());
		img.keypoints.emplace_back(cv::Point2f((float)projections[i].x, (float)projections[i].y), 1.f);
	}
	scene.tracks.emplace_back(std::move(track));
	return true;
}

void ArcScene::Tracks(const IIndexArr& imageIDs, unsigned numTracks, REAL spreadY)
{
	ASSERT(spreadY >= 0 && spreadY <= 1);
	std::uniform_real_distribution<REAL> coord(-1, 1);
	for (unsigned n = 0; n < numTracks; )
		if (TrackAt(Point3(coord(rng), spreadY*coord(rng), coord(rng)), imageIDs))
			++n;
}

void ArcScene::Displace(IIndex imageID, unsigned firstTrack, unsigned numTracks, const Point2f& delta)
{
	ASSERT(firstTrack + numTracks <= scene.tracks.size());
	for (unsigned t = firstTrack; t < firstTrack + numTracks; ++t) {
		const Track& track = scene.tracks[t];
		const auto it = std::find_if(track.observations.begin(), track.observations.end(),
			[imageID](const Observation& obs) { return obs.imageID == imageID; });
		ASSERT(it != track.observations.end());
		scene.images[imageID].keypoints[it->featureID].pt += cv::Point2f(delta.x, delta.y);
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
	if (spec.withMatches) {
		pair.matches.resize(numInliers);
		pair.numFilteredInliers = numInliers;
	}
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
	Triangulate();
}

void ArcScene::Triangulate()
{
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
		if (seed == NO_ID)
			continue;
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

bool BundledImagesFeatures(Scene& scene, const ImportConfig& importCfg)
{
	const ImportConfig defaults;
	ASSERT(scene.images.empty() && importCfg.imageIndicesStr.empty() && importCfg.importPosesFile.empty() &&
		importCfg.cameraType == defaults.cameraType && importCfg.importPosesMode == defaults.importPosesMode &&
		importCfg.defaultFocalRatio == defaults.defaultFocalRatio);
	// the scenes featured so far in this process, by forced focal, k1 and k2
	struct Featured { float focal, k1, k2; Scene scene; };
	static std::list<Featured> featured;
	auto it = std::find_if(featured.begin(), featured.end(), [&importCfg](const Featured& f) {
		return f.focal == importCfg.focalLength && f.k1 == importCfg.k1 && f.k2 == importCfg.k2;
	});
	if (it == featured.end()) {
		Scene source(2);
		if (!source.Import(MAKE_PATH("images"), importCfg)) {
			VERBOSE("error: the bundled images were not imported");
			return false;
		}
		if (source.images.size() != 4) {
			#ifdef _IMAGE_HEIF
			constexpr const char* reason = "";
			#else
			constexpr const char* reason = " (built without libheif, so the two HEIC images were skipped)";
			#endif
			VERBOSE("error: %u of the 4 bundled images imported%s", (unsigned)source.images.size(), reason);
			return false;
		}
		if (!source.ExtractFeatures(BundledImagesConfig().featuresCfg)) {
			VERBOSE("error: no features extracted from the bundled images");
			return false;
		}
		it = featured.insert(featured.end(), Featured{importCfg.focalLength, importCfg.k1, importCfg.k2, std::move(source)});
	}
	scene = it->scene;
	// the copy shares the descriptors' memory with the cache: a caller writing them would change every later copy
	for (Image& img : scene.images)
		img.descriptors = img.descriptors.clone();
	return true;
}
/*----------------------------------------------------------------*/

Scene MakeRig(cv::Size size, REAL focal, const std::vector<Pose3D>& poses, REAL k1)
{
	ASSERT(size.area() > 0 && focal > 0 && !poses.empty());
	Scene scene;
	PinholeCamera* camera = new PinholeCamera(size, focal, focal, REAL(size.width) / 2, REAL(size.height) / 2);
	camera->k1 = k1;
	camera->trustIntrinsics = true;
	scene.cameras.emplace_back(camera);
	FOREACH(i, poses)
		scene.images.emplace_back((IIndex)i, String::FormatString("%u.jpg", (unsigned)i), poses[i], 0, scene.cameras[0]);
	scene.status.nCalibratedImages = scene.images.size();
	return scene;
}

void ArmPair(ImagePair& pair, const Scene& scene)
{
	ASSERT(pair.ID1 < pair.ID2 && pair.ID2 < scene.images.size());
	const Image& imgA = scene.images[pair.ID1];
	const Image& imgB = scene.images[pair.ID2];
	ASSERT(imgA.IsValid() && imgB.IsValid());
	pair.relativePose = imgB / imgA;
	pair.E = ImagePair::ComposeEssentialMatrix(pair.relativePose.value());
	pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), imgA.GetK(), imgB.GetK());
}
/*----------------------------------------------------------------*/


Scene MakeSphereScene(const std::vector<Point3>& centres, unsigned numPoints, uint32_t seed, cv::Size size)
{
	ASSERT(!centres.empty() && size.width == 2 * size.height);
	Scene scene;
	scene.cameras.emplace_back(new SphericalCamera(size));
	for (const Point3& centre : centres) {
		Pose3D pose;
		pose.R = Matrix3x3::IDENTITY;
		pose.C = centre;
		scene.images.emplace_back((IIndex)scene.images.size(), String(), pose, 0, scene.cameras[0]);
	}
	scene.status.nCalibratedImages = scene.images.size();
	if (numPoints == 0)
		return scene;
	std::mt19937 rng(seed);
	std::uniform_real_distribution<REAL> cosThetaDist(REAL(-1), REAL(1));
	std::uniform_real_distribution<REAL> phiDist(REAL(-M_PI), REAL(M_PI));
	std::uniform_real_distribution<REAL> radiusDist(REAL(4.5), REAL(5.5));
	for (unsigned p = 0; p < numPoints; ++p) {
		const REAL r = radiusDist(rng);
		const REAL ct = cosThetaDist(rng);
		const REAL st = SQRT(REAL(1) - ct*ct);
		const REAL ph = phiDist(rng);
		const Point3 X(r * st * COS(ph), r * ct, r * st * SIN(ph));
		Track track(X);
		for (Image& img : scene.images) {
			const auto [proj, valid] = img.ProjectPoint(X);
			if (!valid || !Image8U::isInside(proj, img.GetSize()))
				continue;
			track.observations.emplace_back(img.ID, (uint32_t)img.keypoints.size());
			img.keypoints.emplace_back(Cast<float>(proj), 0.f, 0.f, 10.f);
		}
		if (track.observations.size() < 2) {
			for (const Observation& obs : track.observations)
				scene.images[obs.imageID].keypoints.pop_back();
			continue;
		}
		track.numInliers = (uint8_t)track.observations.size();
		scene.tracks.emplace_back(std::move(track));
	}
	scene.status.nTracks = scene.tracks.size();
	scene.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);
	return scene;
}
/*----------------------------------------------------------------*/


PoseError PoseErrors(const Scene& scene, const std::vector<Pose3D>& truth, const IIndexArr& subset)
{
	ASSERT(truth.size() == scene.images.size() && truth.size() >= 2);
	IIndexArr measured;
	Point3Arr centres, trueCentres;
	const auto Measure = [&](IIndex i) {
		ASSERT(i < scene.images.size());
		if (!scene.images[i].IsValid())
			return;
		measured.push_back(i);
		centres.push_back(scene.images[i].C);
		trueCentres.push_back(truth[i].C);
	};
	if (subset.empty())
		FOREACH(i, scene.images)
			Measure((IIndex)i);
	else
		for (IIndex i : subset)
			Measure(i);
	PoseError e;
	e.count = (unsigned)measured.size();
	if (e.count < 3)
		return e;
	REAL extent = 0;
	FOREACH(i, truth)
		for (IIndex j = i + 1; j < truth.size(); ++j)
			extent = MAXF(extent, norm(truth[i].C - truth[j].C));
	ASSERT(extent > 0);
	const SEACAVE::Transform T(EstimateSimilarityTransform(centres, trueCentres));
	std::vector<REAL> rotations, positions;
	FOREACH(k, measured) {
		rotations.push_back(RotationErrorDeg(RMatrix(scene.images[measured[k]].R * T.R.t()), truth[measured[k]].R));
		positions.push_back(norm(T * centres[k] - trueCentres[k]) / extent);
	}
	e.rotationMax = *std::max_element(rotations.begin(), rotations.end());
	e.positionMax = *std::max_element(positions.begin(), positions.end());
	e.rotationMedian = Median(rotations);
	e.positionMedian = Median(positions);
	return e;
}
/*----------------------------------------------------------------*/


// ===============================================================================
// Helpers that two or more topic files use (moved unchanged from the single TestsSFM.cpp)
// ===============================================================================
// Whether a pair's sparse segment is all described keypoints and its dense segment all dense ones
bool HasDensePartition(const Scene& scene, const ImagePair& pair)
{
	const Image& imgA = scene.images[pair.ID1];
	const Image& imgB = scene.images[pair.ID2];
	const unsigned numSparse = pair.GetNumFilteredInliers(), numDense = pair.GetNumDenseInliers();
	for (unsigned k = 0; k < numSparse + numDense; ++k) {
		const DMatch& m = pair.matches[k];
		const bool dense = k >= numSparse;
		if (imgA.IsDenseKeypoint(m.queryIdx) != dense || imgB.IsDenseKeypoint(m.trainIdx) != dense)
			return false;
	}
	return true;
}

// ===============================================================================
// Helper: Build a synthetic full-sphere scene for PairsMatcher / Resection tests.
// Two (or more) spherical cameras are placed in a cluster near the origin; 3D
// points are distributed uniformly on a sphere of radius 5 around origin so
// every camera sees ~50% back-hemisphere observations. Keypoints and pair
// matches are populated from the ground-truth projections.
// ===============================================================================
void BuildSphericalTwoViewScene(Scene& scene, Pose3D& poseRel)
{
	const int width = 2048, height = 1024;
	scene.cameras.emplace_back(new SphericalCamera(cv::Size(width, height)));

	// Two spherical cameras in a tight cluster — baseline chosen so the
	// relative pose has a well-defined translation direction but both
	// cameras still see essentially the whole sphere.
	const Point3 camCenters[2] = {
		Point3(-0.5, 0.0, 0.0),
		Point3( 0.5, 0.0, 0.2),
	};
	for (unsigned i = 0; i < 2; ++i) {
		Pose3D pose;
		pose.C = camCenters[i];
		pose.R = Matrix3x3::IDENTITY;
		scene.images.emplace_back(static_cast<IIndex>(i), String(), pose, 0, scene.cameras[0]);
	}
	scene.status.nCalibratedImages = scene.images.size();

	// Relative pose from img0 to img1 (ground truth)
	poseRel = scene.images[1] / scene.images[0];

	// Uniform sphere sampling: 120 points on a sphere of radius ~5 around origin.
	const unsigned numPoints = 120;
	std::mt19937 rng(2027);
	std::uniform_real_distribution<REAL> cosThetaDist(REAL(-1), REAL(1));
	std::uniform_real_distribution<REAL> phiDist(REAL(-M_PI), REAL(M_PI));
	std::uniform_real_distribution<REAL> radiusDist(REAL(4.5), REAL(5.5));
	for (unsigned p = 0; p < numPoints; ++p) {
		const REAL r = radiusDist(rng);
		const REAL ct = cosThetaDist(rng);
		const REAL st = SQRT(REAL(1) - ct*ct);
		const REAL ph = phiDist(rng);
		const Point3 X(r * st * COS(ph), r * ct, r * st * SIN(ph));

		Track track(X);
		for (unsigned v = 0; v < scene.images.size(); ++v) {
			Image& img = scene.images[v];
			const auto [proj, valid] = img.ProjectPoint(X);
			if (!valid || !Image8U::isInside(proj, img.GetSize()))
				continue;
			const uint32_t featID = static_cast<uint32_t>(img.keypoints.size());
			img.keypoints.emplace_back(Cast<float>(proj), 0.f, 0.f, 10.f);
			track.observations.emplace_back(img.ID, featID);
		}
		if (track.observations.size() < 2) {
			for (const auto& obs : track.observations)
				scene.images[obs.imageID].keypoints.pop_back();
			continue;
		}
		track.numInliers = static_cast<uint8_t>(track.observations.size());
		scene.tracks.emplace_back(std::move(track));
	}
	scene.status.nTracks = scene.tracks.size();
	scene.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);
}

// Reconstruction stage of RealImagesPipelineTest: build the tracks of an already matched scene of the
// four bundled images, initialize it, and check what that produced -- all images calibrated, the
// forced intrinsics refined back to the truth, and a plausible number of inlier tracks.
// Shared with ROMA2ReconstructTest, which runs it on the same scene matched through the ROMAv2
// dense warps, so that both hold a matched scene to exactly the same expectations.
// testName prefixes the messages; minTracks/maxTracks bound the expected inlier track count and
// maxDistortion the residual lens distortion the refined intrinsics may still carry (in pixels).
bool ReconstructMatchedScene(Scene& scene, const char* testName, unsigned minTracks, unsigned maxTracks, REAL maxDistortion)
{
	// Build tracks
	BuildTracks(scene);
	VERBOSE("%s: Built %u tracks", testName, (unsigned)scene.tracks.size());

	// Initialize with star initializer
	StarInitConfig initCfg;
	initCfg.minViews = 3;
	if (!StarInitializer::Initialize(scene, initCfg)) {
		VERBOSE("%s: StarInitializer::Initialize failed", testName);
		return false;
	}
	VERBOSE("%s: Initialized scene with %u calibrated images", testName, (unsigned)scene.status.nCalibratedImages);

	// The star refines only the focal length (StarInitConfig::refineFocalLength); reproduce the
	// distortion-refining bundle adjustment the pipeline's resection runs right after it
	// (ResectionConfig::DeriveBAConfigs) so this helper still exercises the calibration recovery
	// its callers below check for
	BAConfig fullBaConfig;
	fullBaConfig.RefineMainIntrinsics();
	if (!BundleAdjustment::Adjust(scene, fullBaConfig)) {
		VERBOSE("%s: post-star bundle adjustment with intrinsics refinement failed", testName);
		return false;
	}

	// Sample colors for tracks
	if (!scene.SampleColors() || scene.colors.size() != scene.tracks.size()) {
		VERBOSE("%s: SampleColors failed", testName);
		return false;
	}

	// Test 1: All 4 images should be valid
	unsigned numValidImages = 0;
	for (const Image& img : scene.images) {
		if (img.IsValid())
			++numValidImages;
	}
	if (numValidImages != 4 || scene.status.nCalibratedImages != 4) {
		VERBOSE("%s: Expected 4 valid images, got %u (%u)", testName, numValidImages, scene.status.nCalibratedImages);
		return false;
	}

	// Test 2: Intrinsics should be close to f=700, k1=0, k2=0
	if (scene.cameras.empty()) {
		VERBOSE("%s: No cameras found", testName);
		return false;
	}
	const PinholeCamera* cam = dynamic_cast<PinholeCamera*>(scene.cameras[0]);
	if (!cam) {
		VERBOSE("%s: Camera is not PinholeCamera", testName);
		return false;
	}
	const REAL focal_error = ABS(cam->fx - 700.f);
	const REAL k1_error = ABS(cam->k1 - 0.f);
	const REAL k2_error = ABS(cam->k2 - 0.f);
	const REAL max_distortion = cam->ComputeMaxDistortion();
	VERBOSE("%s: Refined intrinsics: f=%.2f (err=%.2f), k1=%.4g (err=%.4g), k2=%.4g (err=%.4g), max_distortion=%.4g",
	    testName, cam->fx, focal_error, cam->k1, k1_error, cam->k2, k2_error, max_distortion);
	if (focal_error > 100.f) { // Allow 100 pixels error
		VERBOSE("%s: focal length error too large (%.2f)", testName, focal_error);
		return false;
	}
	if (max_distortion > maxDistortion) {
		VERBOSE("%s: distortion error too large (k1_err=%.4g, k2_err=%.4g, max_distortion=%.4g > %.4g)",
			testName, k1_error, k2_error, max_distortion, maxDistortion);
		return false;
	}

	// Test 3: the inlier track count, in the range the caller's matching produces
	VERBOSE("%s: Found %u inlier tracks (expected [%u, %u])", testName, scene.status.nTracks, minTracks, maxTracks);
	if (scene.status.nTracks < minTracks || scene.status.nTracks > maxTracks) {
		VERBOSE("%s: number of inlier tracks out of range [%u, %u]", testName, minTracks, maxTracks);
		return false;
	}
	return true;
}

void AddTripletImages(Scene& scene, IIndex numImages)
{
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), REAL(600), REAL(600), REAL(320), REAL(240)));
	for (IIndex i = 0; i < numImages; ++i) {
		scene.images.emplace_back(i, String::FormatString("%u.jpg", i));
		scene.images[i].cameraID = 0;
		scene.images[i].pCamera = scene.cameras[0];
	}
}

// A pair of numInliers track-forming matches whose keypoints occupy the first cellsA (cellsB)
// cells of a 10x10 grid over each 640x480 image, walked row by row from the top left; 100 cells
// is full coverage, which leaves the pair's triplet strength equal to its inlier count -- what
// every scene below relies on unless it says otherwise.
void AddTripletPair(Scene& scene, IIndex idA, IIndex idB, unsigned numInliers, bool verified,
	unsigned cellsA, unsigned cellsB)
{
	ASSERT(cellsA >= 1 && cellsA <= 100 && cellsB >= 1 && cellsB <= 100);
	ImagePair pair(idA, idB);
	// ImagePair's constructor orders ID1 < ID2, swapping when idA > idB: every later reader
	// (ComputePairCoverage included) looks up images by pair.ID1/pair.ID2, so the match indices
	// recorded below must be built against those, not against idA/idB directly, or a swapped call
	// (idA > idB) would store queryIdx/trainIdx into the wrong image's keypoints.
	Image& imageA = scene.images[pair.ID1];
	Image& imageB = scene.images[pair.ID2];
	const auto cellCentre = [](unsigned cell) {
		return cv::Point2f((float)(cell % 10) * 64.f + 32.f, (float)(cell / 10) * 48.f + 24.f);
	};
	for (unsigned m = 0; m < numInliers; ++m) {
		imageA.keypoints.emplace_back(cellCentre(m % cellsA), 1.f);
		imageB.keypoints.emplace_back(cellCentre(m % cellsB), 1.f);
		pair.matches.emplace_back((uint32_t)imageA.keypoints.size() - 1, (uint32_t)imageB.keypoints.size() - 1);
	}
	pair.numFilteredInliers = numInliers;
	if (verified)
		pair.F = Matrix3x3::IDENTITY; // stands in for the geometric verification
	scene.pairs.emplace_back(std::move(pair));
}

Pose3D ResectionArcPose(REAL angleDegrees)
{
	const REAL angle = D2R(angleDegrees);
	Pose3D pose;
	pose.C = Point3(RESECTION_ARC_RADIUS * COS(angle), 0, RESECTION_ARC_RADIUS * SIN(angle));
	pose.R.LookAt(pose.C, Point3(0, 0, 0), Point3(0, 1, 0));
	return pose;
}

// The arc's images, all posed, sharing one pinhole camera
void BuildResectionArc(Scene& scene, unsigned numImages)
{
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(640, 480), REAL(500), REAL(500), REAL(320), REAL(240)));
	for (unsigned i = 0; i < numImages; ++i) {
		scene.images.emplace_back(IIndex(i), String::FormatString("%u.jpg", i),
			ResectionArcPose(i * RESECTION_ARC_STEP), IIndex(0), scene.cameras[0]);
	}
	scene.status.nCalibratedImages = numImages;
}

// One track at the given point, observed by the given images at its exact projection; false (and
// nothing added) when the point does not project inside all of them
bool AddResectionTrack(Scene& scene, const Point3& X, const std::vector<IIndex>& imageIDs)
{
	std::vector<Point2> projections;
	for (IIndex imageID : imageIDs) {
		const Image& img = scene.images[imageID];
		const auto [proj, valid] = img.ProjectPoint(X);
		if (!valid || !Image8U::isInside(proj, img.GetSize()))
			return false;
		projections.push_back(proj);
	}
	Track track(X);
	FOREACH(i, imageIDs) {
		Image& img = scene.images[imageIDs[i]];
		track.observations.emplace_back(imageIDs[i], (uint32_t)img.keypoints.size());
		img.keypoints.emplace_back(cv::Point2f((float)projections[i].x, (float)projections[i].y), 1.f);
	}
	scene.tracks.emplace_back(std::move(track));
	return true;
}

// numTracks tracks observed by the given images, at points drawn from a 2-unit box around the origin
void AddResectionTracks(Scene& scene, const std::vector<IIndex>& imageIDs, unsigned numTracks, std::mt19937& rng)
{
	std::uniform_real_distribution<REAL> coord(-1, 1);
	for (unsigned i = 0; i < numTracks; ) {
		if (AddResectionTrack(scene, Point3(coord(rng), coord(rng), coord(rng)), imageIDs))
			++i;
	}
}

// A verified pair carrying the ground-truth relative pose of its two images and a chosen strength.
// The resection reads only the relative pose and the weighted inliers off a pair, so no matches are
// needed; the two weights are what makes the pair's composite weight non-zero, i.e. valid.
void AddResectionPair(Scene& scene, IIndex idA, IIndex idB, unsigned numInliers)
{
	ImagePair pair(idA, idB);
	pair.relativePose = scene.images[pair.ID2] / scene.images[pair.ID1];
	pair.weightedInliers = (float)numInliers;
	pair.weightSpatial = 1.f;
	pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
	pair.weightConnectivity = 1.f;
	scene.pairs.emplace_back(std::move(pair));
}

// ===============================================================================
// Phase 1: Scene Clustering Tests
// ===============================================================================

// The blocks of a partition as the reconstruction extracts them (SceneBlocks::Extract), handed out
// as the sub-scenes and their local-to-global maps the merge reads
std::vector<Scene> ExtractBlocks(Scene& scene, const ScenePartition& partition, std::vector<IIndexArr>* localToGlobals)
{
	SceneBlocks sceneBlocks(scene);
	sceneBlocks.Extract(partition);
	if (localToGlobals)
		*localToGlobals = std::move(sceneBlocks.localToGlobal);
	return std::move(sceneBlocks.blocks);
}

// The middle value of a sample, its own numbers left in place
REAL Median(std::vector<REAL>& values)
{
	ASSERT(!values.empty());
	std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
	return values[values.size() / 2];
}

} // namespace SFM
