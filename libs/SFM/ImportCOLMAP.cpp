////////////////////////////////////////////////////////////////////
// ImportCOLMAP.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ImportCOLMAP.h"
#include "PairsWeighting.h"
#include "Scene.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// COLMAP binary file import helpers
namespace {
	template <typename T>
	void ReadBinary(std::ifstream& file, T& value) {
		file.read(reinterpret_cast<char*>(&value), sizeof(T));
	}
	template <>
	void ReadBinary(std::ifstream& file, DMatch& value) {
		file.read(reinterpret_cast<char*>(&value.queryIdx), sizeof(value.queryIdx));
		file.read(reinterpret_cast<char*>(&value.trainIdx), sizeof(value.trainIdx));
	}
	void ReadString(std::ifstream& file, std::string& str) {
		uint64_t length = 0;
		ReadBinary(file, length);
		str.resize(length);
		file.read(&str[0], length);
	}
	void ReadMatrix3d(std::ifstream& file, Eigen::Matrix3d& mat) {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				ReadBinary(file, mat(i, j));
			}
		}
	}
	void ReadVector2d(std::ifstream& file, Eigen::Vector2d& vec) {
		ReadBinary(file, vec(0));
		ReadBinary(file, vec(1));
	}
	void ReadVector3d(std::ifstream& file, Eigen::Vector3d& vec) {
		ReadBinary(file, vec(0));
		ReadBinary(file, vec(1));
		ReadBinary(file, vec(2));
	}
	void ReadRigid3d(std::ifstream& file, Eigen::Quaterniond& quat, Eigen::Vector3d& trans) {
		// Read quaternion (w, x, y, z)
		double w, x, y, z;
		ReadBinary(file, w);
		ReadBinary(file, x);
		ReadBinary(file, y);
		ReadBinary(file, z);
		quat = Eigen::Quaterniond(w, x, y, z);
		// Read translation
		ReadVector3d(file, trans);
	}
	void ReadMatrixXi(std::ifstream& file, Eigen::MatrixXi& mat) {
		int64_t rows = 0, cols = 0;
		ReadBinary(file, rows);
		ReadBinary(file, cols);
		mat.resize(rows, cols);
		for (int64_t i = 0; i < rows; ++i) {
			for (int64_t j = 0; j < cols; ++j) {
				ReadBinary(file, mat(i, j));
			}
		}
	}
}

bool SFM::ImportCOLMAP(const String& fileName, Scene& scene,
	bool importCameras, bool importRelativePoses, bool importPoses, bool importTracks)
{
	TD_TIMER_STARTD();
	VERBOSE("Importing COLMAP scene from '%s'", fileName.c_str());

	std::ifstream file(fileName.c_str(), std::ios::binary);
	if (!file.is_open()) {
		VERBOSE("error: failed to open COLMAP file '%s'", fileName.c_str());
		return false;
	}

	// ===== 1. Read and validate header =====
	uint32_t magic_number = 0;
	ReadBinary(file, magic_number);
	if (magic_number != 0x474C4D50) { // "GLMP" in hex
		VERBOSE("error: invalid COLMAP magic number (expected 0x474C4D50, got 0x%08X)", magic_number);
		return false;
	}

	uint32_t version = 0;
	ReadBinary(file, version);
	if (version != 1) {
		VERBOSE("error: unsupported COLMAP version %u", version);
		return false;
	}

	// ===== 2. Build image lookup map by filename stem =====
	std::unordered_map<String, IIndex> imageByNameStem;
	imageByNameStem.reserve(scene.images.size());
	FOREACH(i, scene.images) {
		const String stem = Util::getFileName(scene.images[i].fileName);
		imageByNameStem.emplace(stem, i);
		if (importCameras)
			scene.images[i].InvalidateCamera();
	}

	// ===== 3. Import cameras =====
	uint64_t num_cameras = 0;
	ReadBinary(file, num_cameras);
	VERBOSE("Importing %u cameras", (unsigned)num_cameras);

	std::unordered_map<uint32_t, IIndex> cameraIDMap; // maps COLMAP camera_id to OpenMVS camera index
	cameraIDMap.reserve(num_cameras);
	for (uint64_t c = 0; c < num_cameras; ++c) {
		uint32_t camera_id = 0;
		ReadBinary(file, camera_id);
		int width = 0, height = 0;
		ReadBinary(file, width);
		ReadBinary(file, height);
		bool priorFocal = false;
		ReadBinary(file, priorFocal);
		Eigen::Matrix3d K;
		ReadMatrix3d(file, K);

		cameraIDMap.emplace(camera_id, c);
		if (!importCameras)
			continue;

		// Extract intrinsics from K matrix
		PinholeCamera* pc = static_cast<PinholeCamera*>(scene.cameras[c]);
		pc->fx = K(0, 0);
		pc->fy = K(1, 1);
		pc->cx = K(0, 2);
		pc->cy = K(1, 2);
		if (pc->GetWidth() != width || pc->GetHeight() != height) {
			VERBOSE("warning: COLMAP camera %u resolution (%ux%u) does not match existing camera resolution (%ux%u), updating",
				camera_id, width, height, pc->GetWidth(), pc->GetHeight());
		}
		pc->trustIntrinsics = priorFocal;
	}

	// ===== 4. Import images (features only, match by name stem) =====
	uint64_t num_images = 0;
	ReadBinary(file, num_images);
	VERBOSE("Importing %u image entries", (unsigned)num_images);
	std::unordered_map<uint32_t, IIndex> imageIDMap; // maps COLMAP image_id to OpenMVS image index
	imageIDMap.reserve(num_images);
	uint32_t matched_images = 0;
	Matrix3x3Arr srcRots, dstRots;
	for (uint64_t img_idx = 0; img_idx < num_images; ++img_idx) {
		uint32_t img_t = 0;
		uint32_t image_id = 0;
		std::string file_name;
		uint32_t camera_id = 0;
		uint64_t num_features = 0;

		ReadBinary(file, img_t);
		ReadBinary(file, image_id);
		ReadString(file, file_name);
		ReadBinary(file, camera_id);
		ReadBinary(file, num_features);

		// Read features (keypoints)
		std::vector<cv::KeyPoint> keypoints;
		keypoints.reserve(num_features);
		for (uint64_t f = 0; f < num_features; ++f) {
			Eigen::Vector2d pt;
			ReadVector2d(file, pt);
			keypoints.emplace_back(cv::Point2f(pt(0), pt(1)), 1.0f);
		}

		// Read pose
	    bool has_pose = false;
		ReadBinary(file, has_pose);
		Eigen::Quaterniond quat;
		Eigen::Vector3d trans;
		if (has_pose)
			ReadRigid3d(file, quat, trans);

		// Match image by filename stem
		const String stem = Util::getFileName(file_name);
		auto it = imageByNameStem.find(stem);
		if (it == imageByNameStem.end()) {
			VERBOSE("warning: COLMAP image '%s' (stem '%s') not found in existing images, skipping",
				file_name.c_str(), stem.c_str());
			continue;
		}

		const IIndex local_img_idx = it->second;
		imageIDMap.emplace(image_id, local_img_idx);

		// Update image with COLMAP data
		Image& img = scene.images[local_img_idx];

		// Validate and assign camera
		if (importCameras) {
			const IIndex cam_idx = cameraIDMap.at(camera_id);
			img.cameraID = cam_idx;
			img.pCamera = scene.cameras[cam_idx];
		}

		// Store keypoints (overwrite existing if necessary), dropping the descriptors and the
		// described-keypoint boundary with them: both describe the array being replaced, and a
		// boundary outliving its keypoints is a count larger than the array it indexes
		if (num_features > 0) {
			img.ReleaseFeatures();
			img.keypoints = std::move(keypoints);
		}

		// Store pose if available
		if (importPoses && has_pose) {
			const Matrix3x3 R = quat.toRotationMatrix();
			if (img.HasPose()) {
				srcRots.push_back(img.R);
				dstRots.push_back(R);
			}
			img.R = R;
			if (trans.hasNaN()) {
				img.C = Point3::ZERO;
			} else {
				img.SetT(trans);
			}
		}

		matched_images++;
	}
	VERBOSE("Matched %u COLMAP images to existing images", matched_images);
	if (!srcRots.empty()) {
		Matrix3x3 alignR;
		if (!EstimateRotationAlignment(srcRots, dstRots, alignR)) {
			VERBOSE("error: rotation alignment estimation failed (%zu matches)", srcRots.size());
			return false;
		}
		MeanStdMinMax<double> rotationErrors;
		constexpr double rotErrorThresholdDeg = 10.0;
		constexpr double rotErrorLargeThresholdDeg = 30.0;
		unsigned numLargeRotErrors = 0, numVeryLargeRotErrors = 0;
		FOREACH(k, srcRots) {
			const Matrix3x3 R_rel_scene(srcRots[k] * alignR);
			const Matrix3x3& R_gt = dstRots[k];
			const double ang = R2D(ACOS(ComputeAngle(R_rel_scene, R_gt)));
			if (ang > rotErrorLargeThresholdDeg)
				++numVeryLargeRotErrors;
			else if (ang > rotErrorThresholdDeg)
				++numLargeRotErrors;
			rotationErrors.Update(ang);
		}
		VERBOSE("COLMAP imported image poses rotation error: num %u, mean %.3f, std %.3f, max %.3f, large %u, very-large %u",
			rotationErrors.size, rotationErrors.GetMean(), rotationErrors.GetStdDev(), rotationErrors.GetMax(), numLargeRotErrors, numVeryLargeRotErrors);
	}

	// ===== 5. Import tracks =====
	if (importTracks)
		scene.tracks.clear();
	uint64_t num_tracks = 0;
	ReadBinary(file, num_tracks);
	VERBOSE("Importing %u tracks", (unsigned)num_tracks);
	for (uint64_t t = 0; t < num_tracks; ++t) {
		uint64_t track_id = 0;
		ReadBinary(file, track_id);
		uint32_t num_observations = 0;
		ReadBinary(file, num_observations);

		// Create track with default position (0, 0, 0)
		Track track(Point3(0, 0, 0));
		track.observations.reserve(num_observations);

		bool valid_track = true;
		for (uint32_t obs_idx = 0; obs_idx < num_observations; ++obs_idx) {
			uint32_t obs_image_id = 0, obs_feature_id = 0;
			ReadBinary(file, obs_image_id);
			ReadBinary(file, obs_feature_id);

			// Map COLMAP image_id to local image index
			auto itImg = imageIDMap.find(obs_image_id);
			if (itImg == imageIDMap.end()) {
				VERBOSE("warning: COLMAP track observation references unknown image_id %u", obs_image_id);
				valid_track = false;
				continue;
			}

			const IIndex local_img_idx = itImg->second;
			track.observations.emplace_back(local_img_idx, obs_feature_id);
		}
		if (!valid_track || track.observations.empty()) {
			VERBOSE("warning: COLMAP track %u has no valid observations, skipping", track_id);
			continue;
		}

		// Set numInliers based on observation count
		track.numInliers = (uint8_t)track.observations.size();

		if (track.IsValid() && importTracks)
			scene.tracks.emplace_back(std::move(track));
	}
	VERBOSE("Imported %u valid tracks", (unsigned)scene.tracks.size());

	// ===== 6. Import view graph (image pairs) =====
	scene.pairs.clear();
	uint64_t numPairs = 0, numValidPairs = 0;
	ReadBinary(file, numPairs);
	VERBOSE("Importing %zu image pairs", numPairs);
	size_t numMatches = 0,  numInliers = 0;
	unsigned numCamCalibrated = 0, numCamUncalibrated = 0, numCamMixed = 0;
	for (uint64_t pair_idx = 0; pair_idx < numPairs; ++pair_idx) {
		uint32_t image_id1 = 0, image_id2 = 0;
		uint64_t pair_id = 0;
		int config = 0;
		float tri_angle = 0.f;
		bool is_valid = false;

		ReadBinary(file, pair_id);
		ReadBinary(file, image_id1);
		ReadBinary(file, image_id2);
		ReadBinary(file, config);
		ReadBinary(file, tri_angle);
		ReadBinary(file, is_valid);

		// Read matrices
		Eigen::Matrix3d E, F, H;
		ReadMatrix3d(file, E);
		ReadMatrix3d(file, F);
		ReadMatrix3d(file, H);

		// Read relative pose
		Eigen::Quaterniond quat;
		Eigen::Vector3d trans;
		ReadRigid3d(file, quat, trans);

		// Read matches
		Eigen::MatrixXi matches;
		ReadMatrixXi(file, matches);

		// Read inliers
		uint64_t num_inliers = 0;
		ReadBinary(file, num_inliers);
		std::vector<DMatch> inliers;
		inliers.reserve(num_inliers);
		for (uint64_t inlier_idx = 0; inlier_idx < num_inliers; ++inlier_idx) {
			DMatch inlier_val;
			ReadBinary(file, inlier_val);
			inliers.push_back(inlier_val);
		}

		// Map COLMAP image IDs to local image indices
		auto itImg1 = imageIDMap.find(image_id1);
		auto itImg2 = imageIDMap.find(image_id2);
		if (itImg1 == imageIDMap.end() || itImg2 == imageIDMap.end()) {
			VERBOSE("warning: COLMAP pair references unknown images %u-%u, skipping", image_id1, image_id2);
			continue;
		}

		const IIndex local_img1 = itImg1->second;
		const IIndex local_img2 = itImg2->second;
		if (is_valid == false)
			continue;
		if (config == 3) {
			numCamUncalibrated++;
		} else if (config == 2) {
			numCamCalibrated++;
		} else {
			numCamMixed++;
		}
		numMatches += matches.rows();
		numInliers += inliers.size();

		// Ensure ID1 < ID2 for consistency
		const IIndex ID1 = std::min(local_img1, local_img2);
		const IIndex ID2 = std::max(local_img1, local_img2);

		// Create image pair
		ImagePair& pair = scene.pairs.emplace_back(ID1, ID2);

		if (inliers.empty() || !importRelativePoses) {
			// Populate matches as DMatch objects
			// matches matrix: rows = num matches, cols = 2 (queryIdx, trainIdx)
			pair.matches.reserve(matches.rows());
			for (int m = 0; m < matches.rows(); ++m) {
				DMatch match;
				match.queryIdx = matches(m, 0);
				match.trainIdx = matches(m, 1);
				if (local_img1 > local_img2) {
					// If image order was swapped, swap match indices
					std::swap(match.queryIdx, match.trainIdx);
				}
				pair.matches.push_back(match);
			}
		} else {
			// Store inliers
			pair.numFilteredInliers = (int)inliers.size();
			pair.matches = std::move(inliers);
		}

		// Store geometric models
		if (importRelativePoses) {
			// Convert Eigen matrices to OpenMVS types
			if (E != Eigen::Matrix3d::Zero())
				pair.E = E;
			if (F != Eigen::Matrix3d::Zero())
				pair.F = F;
			if (H != Eigen::Matrix3d::Zero())
				pair.H = H;

			// Convert quaternion to rotation matrix
			Matrix3x3 R_mvs(quat.toRotationMatrix());
			Point3 t_mvs(trans(0), trans(1), trans(2));

			// Swap R and t if image order was reversed
			if (local_img1 > local_img2) {
				R_mvs = R_mvs.t();
				t_mvs = -R_mvs * t_mvs;
			}
			if (R_mvs != Matrix3x3::IDENTITY)
				pair.relativePose = Pose3D(R_mvs, t_mvs);

			pair.meanRayAngle = tri_angle;
		}
		++numValidPairs;
	}
	VERBOSE("Imported %u/%u image pairs: %u calibrated, %u uncalibrated, %u mixed; %zu matches, %zu inliers",
		(unsigned)numValidPairs, (unsigned)numPairs, numCamCalibrated, numCamUncalibrated, numCamMixed, numMatches, numInliers);

	if (importRelativePoses && !scene.pairs.empty()) {
		// Compute pair weights
		ComputePairsWeights(scene);
	}
	file.close();

	// ===== 7. Set status and finalize =====
	scene.status.nState.set(Scene::Status::STATE::FEATURES_EXTRACTED);
	scene.status.nState.set(Scene::Status::STATE::MATCHED);
	scene.status.nTracks = (uint32_t)std::count_if(scene.tracks.begin(), scene.tracks.end(),
		[](const Track& t) { return t.IsInlier(); });

	DEBUG("COLMAP scene imported (%s): %u cameras, %u images (matched %u), %u pairs, %u tracks",
		TD_TIMER_GET_FMT().c_str(),
		scene.cameras.size(), (unsigned)num_images, matched_images, (unsigned)scene.pairs.size(), (unsigned)scene.tracks.size());
	return true;
}
/*----------------------------------------------------------------*/
