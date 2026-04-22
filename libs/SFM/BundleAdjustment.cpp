/*
 * BundleAdjustment.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

// Include Eigen before OpenCV to avoid header ordering issues
#include "Common.h"
#include "BundleAdjustment.h"
#include "Scene.h"
#include "../Math/GeodeticTransforms.h"
#include "BundleAdjustmentCostFunctions.h"

using namespace SFM;

// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// Convert OpenMVS pose to/from Ceres quaternion parameterization [qw, qx, qy, qz, Cx, Cy, Cz]
void SFM::Pose3DToQuaternionAndCenter(const Pose3D& pose, double* params) {
	ceres::RotationMatrixToQuaternion(ceres::RowMajorAdapter3x3(pose.R.val), params);
	Eigen::Map<Point3d::EVec>(params + 4) = (const Point3d::EVec)pose.C;
}
void SFM::QuaternionAndCenterToPose3D(const double* params, Pose3D& pose) {
	ceres::QuaternionToRotation(params, ceres::RowMajorAdapter3x3(pose.R.val));
	pose.C = Eigen::Map<const Point3d::EVec>(params + 4);
}

// Convert OpenMVS pose to/from Ceres angle-axis parameterization [ax, ay, az, Cx, Cy, Cz]
void SFM::Pose3DToAngleAxisAndCenter(const Pose3D& pose, double* params) {
	ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3(pose.R.val), params);
	Eigen::Map<Point3d::EVec>(params + 3) = (const Point3d::EVec)pose.C;
}
void SFM::AngleAxisAndCenterToPose3D(const double* params, Pose3D& pose) {
	ceres::AngleAxisToRotationMatrix(params, ceres::RowMajorAdapter3x3(pose.R.val));
	pose.C = Eigen::Map<const Point3d::EVec>(params + 3);
}
/*----------------------------------------------------------------*/


bool BundleAdjustment::Adjust(Scene& scene, const BAConfig& config)
{
	TD_TIMER_STARTD();

	// Count registered images (those with valid poses)
	IIndex nRegisteredImages = 0;
	for (const Image& img : scene.images)
		if (img.IsValid())
			++nRegisteredImages;
	const uint32_t nInlierTracks(scene.status.nTracks > 1000 ? scene.status.nTracks : scene.tracks.size());
	if (nRegisteredImages < 2 || nInlierTracks < 50) {
		VERBOSE("error: insufficient data for bundle adjustment");
		return false;
	}
	DEBUG_EXTRA("Bundle adjustment with %u cameras, %u images, %u tracks",
		scene.cameras.size(), nRegisteredImages, nInlierTracks);

	// Pose parameters: [qw, qx, qy, qz, Cx, Cy, Cz] x nImages
	std::vector<double> poseParams(scene.images.size() * 7);
	FOREACH(i, scene.images)
		if (scene.images[i].IsValid())
			Pose3DToQuaternionAndCenter(scene.images[i], poseParams.data() + i * 7);

	// Intrinsic parameters: map unique cameras to parameter blocks
	// Each pinhole camera has 12 params: [fx, fy, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6]
	std::unordered_map<const Camera*, DoubleArr> intrinsicParams;
	for (const Image& img : scene.images) {
		if (!img.IsValid())
			continue;
		CameraType model = img.GetCameraType();
		if (model != CameraType::PINHOLE)
			continue;
		const auto it = intrinsicParams.emplace(img.pCamera, DoubleArr());
		if (!it.second)
			continue; // already processed
		const PinholeCamera* pinholeCamera = static_cast<const PinholeCamera*>(img.pCamera);
		it.first->second.resize(12);
		double* intr = it.first->second.data();
		intr[0] = pinholeCamera->fx;
		intr[1] = pinholeCamera->fy / pinholeCamera->fx;
		intr[2] = pinholeCamera->cx;
		intr[3] = pinholeCamera->cy;
		intr[4] = pinholeCamera->k1;
		intr[5] = pinholeCamera->k2;
		intr[6] = pinholeCamera->k3;
		intr[7] = pinholeCamera->p1;
		intr[8] = pinholeCamera->p2;
		intr[9] = pinholeCamera->k4;
		intr[10] = pinholeCamera->k5;
		intr[11] = pinholeCamera->k6;
	}

	// Build Ceres problem
	ceres::Problem problem;
	// Use standard Huber loss (threshold in pixels)
	ceres::LossFunction* loss_function = config.robustThreshold > 0.f ?
		new ceres::HuberLoss(config.robustThreshold) : nullptr;

	// Set quaternion manifold for all pose blocks
	#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
	// Ceres 2.1+: Use ProductManifold to combine QuaternionManifold (4 params) + EuclideanManifold (3 params)
	// This represents SE(3): rotation (quaternion, 3 DOF tangent space) + translation (Euclidean, 3 DOF)
	auto* se3_manifold = new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>{
		ceres::QuaternionManifold{}, ceres::EuclideanManifold<3>{}};
	FOREACH(i, scene.images) {
		if (!scene.images[i].IsValid())
			continue;
		problem.AddParameterBlock(poseParams.data() + i * 7, 7, se3_manifold);
	}
	#else
	// Ceres 2.0: Use parameterizations
	auto* quaternion_param = new ceres::QuaternionParameterization;
	auto* identity_param = new ceres::IdentityParameterization(3);
	auto* pose_param = new ceres::ProductParameterization(quaternion_param, identity_param);
	FOREACH(i, scene.images) {
		if (!scene.images[i].IsValid())
			continue;
		problem.AddParameterBlock(poseParams.data() + i * 7, 7);
		problem.SetParameterization(poseParams.data() + i * 7, pose_param);
	}
	#endif

	// Add reprojection residuals
	uint32_t numReprojResiduals = 0;
	uint32_t numSkippedLowConfidence = 0;
	UnsignedArr numReprojResidualsPerImage(scene.images.size());
	numReprojResidualsPerImage.Memset(0);
	for (Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const auto& obs : track) {
			const IIndex imgID = obs.imageID;
			const Image& img = scene.images[imgID];
			if (!img.IsValid())
				continue;
			ASSERT(obs.featureID < img.keypoints.size());
			const cv::KeyPoint& kp = img.keypoints[obs.featureID];
			// Compute weight from keypoint response / size (if enabled)
			ceres::LossFunction* residual_loss_function = loss_function;
			if (config.useKeypointConfidence) {
				double weight = Image::ComputeKeypointPrecision(kp, config.minKeypointResponse);
				if (weight <= 0.0) {
					++numSkippedLowConfidence;
					continue; // skip low-confidence keypoints
				}
				if (weight != 1.0)
					residual_loss_function = new ceres::ScaledLoss(loss_function, weight, ceres::DO_NOT_TAKE_OWNERSHIP);
			}

			// Create cost function based on camera model
			switch (img.GetCameraType()) {
			case CameraType::PINHOLE: {
				DoubleArr& intr = intrinsicParams.at(img.pCamera);
				ceres::CostFunction* cost_function =
					#if 0
					new PinholeReprojectionErrorAnalytic(kp.pt.x, kp.pt.y);
					#else
					PinholeReprojectionError::Create(kp.pt.x, kp.pt.y);
					#endif
				problem.AddResidualBlock(
					cost_function,
					residual_loss_function,
					poseParams.data() + imgID * 7,  // Pose params
					intr.data(),                    // Intrinsic params
					track.position.ptr()            // Point params
				);
			} break;
			case CameraType::SPHERICAL: {
				// Spherical error is already scaled to pixels and weighted inside the functor
				ceres::CostFunction* cost_function = SphericalAngularReprojectionError::Create(
					kp.pt.x, kp.pt.y, img.pCamera->GetWidth(), img.pCamera->GetHeight());
				problem.AddResidualBlock(
					cost_function,
					residual_loss_function,
					poseParams.data() + imgID * 7,  // Pose params
					track.position.ptr()            // Point params
				);
			} break;
			}
			++numReprojResidualsPerImage[imgID];
			++numReprojResiduals;
		}
	}
	if (config.useKeypointConfidence) {
		DEBUG_EXTRA("Created %u reprojection residuals (%u skipped low-confidence)",
		    numReprojResiduals, numSkippedLowConfidence);
	} else {
		DEBUG_EXTRA("Created %u reprojection residuals", numReprojResiduals);
	}

	// Set intrinsic parameter constraints (if refining intrinsics)
	if (config.IsRefiningIntrinsics() && !intrinsicParams.empty()) {
		// Build subset manifold for each camera based on refinement flags
		// Intrinsic layout: [fx, fy/fx, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6]
		std::vector<int> constantParams;
		constantParams.reserve(12);
		if (!config.refineFocalLength) {
			constantParams.push_back(0);  // fx
			constantParams.push_back(1);  // fy/fx
		} else if (!config.refineFocalLengthAspectRatio) {
			constantParams.push_back(1);  // fy/fx
		}
		if (!config.refinePrincipalPoint) {
			constantParams.push_back(2);  // cx
			constantParams.push_back(3);  // cy
		}
		if (!config.refineRadialDistortion123) {
			constantParams.push_back(4);  // k1
			constantParams.push_back(5);  // k2
			constantParams.push_back(6);  // k3
		}
		if (!config.refineTangentialDistortion) {
			constantParams.push_back(7);  // p1
			constantParams.push_back(8);  // p2
		}
		if (!config.refineRadialDistortion456) {
			constantParams.push_back(9);   // k4
			constantParams.push_back(10);  // k5
			constantParams.push_back(11);  // k6
		}
		std::vector<int> internConstantParams(constantParams);
		if (config.refineRadialDistortion456) {
			internConstantParams.push_back(9);   // k4
			internConstantParams.push_back(10);  // k5
			internConstantParams.push_back(11);  // k6
		}

		#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
		auto* intrinsicManifold = new ceres::SubsetManifold(12, constantParams);
		auto* internIntrinsicManifold = new ceres::SubsetManifold(12, internConstantParams);
		#else
		auto* intrinsicManifold = new ceres::SubsetParameterization(12, constantParams);
		auto* internIntrinsicManifold = new ceres::SubsetParameterization(12, internConstantParams);
		#endif
		bool bIntrinsicManifoldUsed = false;
		bool bInternIntrinsicManifoldUsed = false;
		for (auto& pair : intrinsicParams) {
			ASSERT(!pair.second.empty());
			auto intrManifold = (pair.first->GetType() == CameraType::PINHOLE && !static_cast<const PinholeCamera*>(pair.first)->useAdditionalDistortion ?
				internIntrinsicManifold : intrinsicManifold);
			if (intrManifold == intrinsicManifold)
				bIntrinsicManifoldUsed = true;
			else
				bInternIntrinsicManifoldUsed = true;
			#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
			problem.SetManifold(pair.second.data(), intrManifold);
			#else
			problem.SetParameterization(pair.second.data(), intrManifold);
			#endif
		}
		if (!bIntrinsicManifoldUsed)
			delete intrinsicManifold;
		if (!bInternIntrinsicManifoldUsed)
			delete internIntrinsicManifold;
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (internConstantParams.empty() || !bInternIntrinsicManifoldUsed) {
			DEBUG("Intrinsic parameters refined");
		} else {
			std::string paramStr;
			FOREACH(i, internConstantParams) {
				if (i > 0) paramStr += ", ";
				paramStr += std::to_string(internConstantParams[i]);
			}
			DEBUG("Fixed intrinsic parameters: %s", paramStr.c_str());
		}
		#endif
	} else if (!intrinsicParams.empty()) {
		// Not refining intrinsics: set all intrinsic blocks constant
		for (auto& pair : intrinsicParams) {
			ASSERT(!pair.second.empty());
			problem.SetParameterBlockConstant(pair.second.data());
		}
		DEBUG("Fixed all intrinsic parameters");
	}

	// Add GPS position constraints (if enabled)
	uint32_t nGPSResiduals = 0;
	if ((config.gpsPositionWeight > 0.0 || config.gpsPositionWeightZ > 0.0) && scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN)) {
		// Estimate median distance from tracks
		DoubleArr distances;
		distances.reserve(scene.tracks.size());
		for (const Track& track : scene.tracks) {
			if (!track.IsInlier())
				continue;
			for (const auto& obs : track) {
				ASSERT(obs.imageID < scene.images.size());
				const Image& img = scene.images[obs.imageID];
				ASSERT(img.IsValid());
				double dist = norm(track.position - img.C);
				if (dist > 0.1) // filter out degenerate points
					distances.push_back(dist);
				break; // only need one observation per track
			}
		}
		// Compute scene scale for unit-aware weighting
		ASSERT(!distances.empty());
		const double median_depth = distances.GetMedian();
		// Estimate median focal length
		DoubleArr focals;
		for (const Image& img : scene.images) {
			if (img.IsValid() && img.GetCameraType() == CameraType::PINHOLE) {
				const PinholeCamera* pc = dynamic_cast<const PinholeCamera*>(img.pCamera);
				focals.push_back((pc->fx + pc->fy) / 2.0);
			}
		}
		double median_focal = 1.0; // default fallback
		if (!focals.empty())
			median_focal = focals.GetMedian();
		// Compute pixel-to-meter scale
		const double pixel_scale = median_depth / median_focal;
		const double weight_h_scaled = SQRT(config.gpsPositionWeight * config.gpsWeightScaleFactor * pixel_scale);
		const double weight_v_scaled = SQRT(config.gpsPositionWeightZ * config.gpsWeightScaleFactor * pixel_scale);
		DEBUG_EXTRA("GPS weight scaling: median_depth %.2f m, median_focal %.1f px, pixel_scale %.4f m/px",
		    median_depth, median_focal, pixel_scale);
		DEBUG_EXTRA("Effective GPS weights: horizontal %.4f, vertical %.4f", weight_h_scaled, weight_v_scaled);
		// Collect GPS observations and create GPS residuals
		const Point3d centerECEF = scene.GetCenterECEF();
		double lat0, lon0, alt0;
		ECEFToWGS84(centerECEF.x, centerECEF.y, centerECEF.z, lat0, lon0, alt0);
		FOREACH(i, scene.images) {
			const Image& img = scene.images[i];
			if (!img.IsValid())
				continue;
			const View::Metadata& meta = img.View::metadata;
			if (!meta.HasGPS())
				continue;
			double enu_east, enu_north, enu_up;
			WGS84ToENU(meta.latitude, meta.longitude, meta.altitude,
						lat0, lon0, alt0,
						enu_east, enu_north, enu_up);
			// Create GPS residual
			// Camera coordinate convention: X=East, Y=North, Z=Up (adjust if scene uses different convention)
			ceres::CostFunction* gps_cost = GPSPositionError::Create(
				enu_east, enu_north, enu_up,
				meta.positionAccuracy, meta.positionAccuracyZ,
				weight_h_scaled, weight_v_scaled
			);
			problem.AddResidualBlock(
				gps_cost,
				nullptr, // No robust loss for GPS (already weighted by accuracy)
				poseParams.data() + i * 7
			);
			++nGPSResiduals;
		}
		DEBUG("Added %u GPS position constraints (origin: lat=%.6f°, lon=%.6f°, alt=%.1fm)",
			nGPSResiduals, lat0, lon0, alt0);
	}

	// Fix best connected camera (gauge freedom) - unless we have GPS constraints
	if (nGPSResiduals == 0) {
		IIndex bestImgID = NO_ID;
		FOREACH(i, scene.images) {
			if (!scene.images[i].IsValid())
				continue;
			if (bestImgID == NO_ID || numReprojResidualsPerImage[bestImgID] < numReprojResidualsPerImage[i])
				bestImgID = i;
		}
		if (bestImgID != NO_ID) {
			problem.SetParameterBlockConstant(poseParams.data() + bestImgID * 7);
			DEBUG("Fixed view %u (reference, no GPS)", bestImgID);
		}
	}

	// Optionally disable pose/point refinement
	if (!config.IsRefiningPoses()) {
		// Disable all pose refinement
		FOREACH(i, scene.images)
			if (scene.images[i].IsValid())
				problem.SetParameterBlockConstant(poseParams.data() + i * 7);
		DEBUG("Views poses: FIXED");
	} else if (!config.refinePosesRotation || !config.refinePosesPosition) {
		// Selectively disable rotation and/or position refinement
		// Pose format: [qw, qx, qy, qz, Cx, Cy, Cz]
		std::vector<int> constantParams;
		FOREACH(i, scene.images) {
			if (!scene.images[i].IsValid())
				continue;
			double* pose = poseParams.data() + i * 7;
			constantParams.clear();
			if (!config.refinePosesRotation) {
				// Fix rotation (indices 0-3)
				constantParams.push_back(0);
				constantParams.push_back(1);
				constantParams.push_back(2);
				constantParams.push_back(3);
			}
			if (!config.refinePosesPosition) {
				// Fix position (indices 4-6)
				constantParams.push_back(4);
				constantParams.push_back(5);
				constantParams.push_back(6);
			}
			#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
			// Ceres 2.1+: Use ProductManifold with SubsetManifold
			auto* subset = new ceres::SubsetManifold(7, constantParams);
			problem.SetManifold(pose, subset);
			#else
			// Ceres 2.0: Use parameterizations with SubsetParameterization
			auto* subset = new ceres::SubsetParameterization(7, constantParams);
			problem.SetParameterization(pose, subset);
			#endif
		}
		DEBUG("Views poses: rotation=%s, position=%s",
		      config.refinePosesRotation ? "OPTIMIZED" : "FIXED",
		      config.refinePosesPosition ? "OPTIMIZED" : "FIXED");
	}
	if (!config.refinePoints) {
		// Disable all point refinement
		for (Track& track : scene.tracks)
			problem.SetParameterBlockConstant(track.position.ptr());
		DEBUG("3D points: FIXED");
	}

	// Configure solver
	ceres::Solver::Options options;
	if (numReprojResiduals < 500000) {
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.preconditioner_type = ceres::IDENTITY; // Not used with DENSE_SCHUR
	} else {
		// For large problems, use SPARSE_SCHUR or ITERATIVE_SCHUR
		// Use ITERATIVE_SCHUR for better numerical stability, especially on macOS Apple Accelerate
		// SPARSE_SCHUR can fail with "Numeric factorisation failed" on poorly conditioned problems
		options.linear_solver_type = ceres::ITERATIVE_SCHUR;
		options.preconditioner_type = ceres::SCHUR_JACOBI; // Robust preconditioner
		options.use_inner_iterations = true; // Improves convergence
	}
	#ifndef _RELEASE
	options.minimizer_progress_to_stdout = true;
	#else
	options.minimizer_progress_to_stdout = false;
	#endif
	options.max_num_iterations = config.maxIterations;
	options.num_threads = config.numThreads > 0 ? config.numThreads : std::thread::hardware_concurrency();
	options.function_tolerance = config.functionTolerance;

	// Solve
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	DEBUG("BA Summary: %s", summary.BriefReport().c_str());
	if (!summary.IsSolutionUsable()) {
		VERBOSE("error: bundle adjustment failed");
		return false;
	}

	// Update scene with optimized parameters
	FOREACH(i, scene.images)
		if (scene.images[i].IsValid())
			QuaternionAndCenterToPose3D(poseParams.data() + i * 7, scene.images[i]);

	// Update camera intrinsics if refined
	if (config.IsRefiningIntrinsics() && !intrinsicParams.empty()) {
		for (auto& pair : intrinsicParams) {
			Camera* cam = const_cast<Camera*>(pair.first);
			PinholeCamera* pinholeCamera = dynamic_cast<PinholeCamera*>(cam);
			if (!pinholeCamera)
				continue;
			const double* intr = pair.second.data();
			pinholeCamera->fx = static_cast<REAL>(intr[0]);
			pinholeCamera->fy = pinholeCamera->fx * static_cast<REAL>(intr[1]);
			pinholeCamera->cx = static_cast<REAL>(intr[2]);
			pinholeCamera->cy = static_cast<REAL>(intr[3]);
			pinholeCamera->k1 = static_cast<REAL>(intr[4]);
			pinholeCamera->k2 = static_cast<REAL>(intr[5]);
			pinholeCamera->k3 = static_cast<REAL>(intr[6]);
			pinholeCamera->p1 = static_cast<REAL>(intr[7]);
			pinholeCamera->p2 = static_cast<REAL>(intr[8]);
			pinholeCamera->k4 = static_cast<REAL>(intr[9]);
			pinholeCamera->k5 = static_cast<REAL>(intr[10]);
			pinholeCamera->k6 = static_cast<REAL>(intr[11]);
			DEBUG_EXTRA("Camera intrinsics updated: %s", pinholeCamera->GetIntrinsicsString().c_str());
		}
		DEBUG("Updated intrinsics for %u cameras", (unsigned)intrinsicParams.size());
	}

	DEBUG("Bundle adjustment complete: %u reprojection residuals, %u GPS residuals, %.4g -> %.4g cost (%s)",
	    numReprojResiduals, nGPSResiduals, summary.initial_cost, summary.final_cost, TD_TIMER_GET_FMT().c_str());

	// Report average reprojection errors
	ComputeTracksMeanReprojectionError(scene);
	return true;
}

bool BundleAdjustment::AdjustLocal(
	Scene& scene,
	const IIndexArr& viewIDs,
	const IIndexArr& fixedViewIDs,
	const BAConfig& config)
{
	TD_TIMER_STARTD();

	// 1. Set local window
	ASSERT(!viewIDs.empty());
	const std::unordered_set<IIndex> localImages(viewIDs.begin(), viewIDs.end());
	const std::unordered_set<IIndex> fixedImages(fixedViewIDs.begin(), fixedViewIDs.end());
	const IIndexArr allImages(viewIDs + fixedViewIDs);

	// 2. Collect relevant points (observed by at least one local image)
	std::vector<uint32_t> activePoints;
	activePoints.reserve(scene.tracks.size() / 10); // heuristic
	FOREACH(i, scene.tracks) {
		const Track& track = scene.tracks[i];
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track) {
			if (localImages.find(obs.imageID) != localImages.end()) {
				activePoints.push_back(i);
				break;
			}
		}
	}
	if (activePoints.empty()) {
		VERBOSE("warning: no points in local window");
		return true;
	}
	DEBUG_EXTRA("Local bundle adjustment with %u cameras, %u (%u local, %u fixed) images, %u tracks",
		scene.cameras.size(), allImages.size(), viewIDs.size(), fixedViewIDs.size(), (unsigned)activePoints.size());

	// 3. Build Ceres problem (subset of global problem)
	ceres::Problem problem;
	// Use standard Huber loss (threshold in pixels)
	ceres::LossFunction* loss_function = config.robustThreshold > 0.f ?
		new ceres::HuberLoss(config.robustThreshold) : nullptr;

	// Initialize parameters for local and fixed images
	std::unordered_map<IIndex, DoubleArr> poseParams;
	for (IIndex imgID : allImages) {
		ASSERT(scene.images[imgID].IsValid());
		poseParams[imgID].resize(7);
		Pose3DToQuaternionAndCenter(scene.images[imgID], poseParams[imgID].data());
	}

	// Intrinsic parameters: always fixed in local BA (not refined)
	// Each pinhole camera has 12 params: [fx, fy, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6]
	std::unordered_map<const Camera*, DoubleArr> intrinsicParams;
	for (IIndex imgID : allImages) {
		const Image& img = scene.images[imgID];
		if (!img.IsValid())
			continue;
		CameraType model = img.GetCameraType();
		if (model != CameraType::PINHOLE)
			continue;
		const auto it = intrinsicParams.emplace(img.pCamera, DoubleArr());
		if (!it.second)
			continue; // already processed
		const PinholeCamera* pinholeCamera = static_cast<const PinholeCamera*>(img.pCamera);
		it.first->second.resize(12);
		double* intr = it.first->second.data();
		intr[0] = pinholeCamera->fx;
		intr[1] = pinholeCamera->fy / pinholeCamera->fx;
		intr[2] = pinholeCamera->cx;
		intr[3] = pinholeCamera->cy;
		intr[4] = pinholeCamera->k1;
		intr[5] = pinholeCamera->k2;
		intr[6] = pinholeCamera->k3;
		intr[7] = pinholeCamera->p1;
		intr[8] = pinholeCamera->p2;
		intr[9] = pinholeCamera->k4;
		intr[10] = pinholeCamera->k5;
		intr[11] = pinholeCamera->k6;
	}

	// Add residuals
	uint32_t numReprojResiduals = 0;
	for (const IIndex pointID : activePoints) {
		Track& track = scene.tracks[pointID];
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track) {
			const IIndex imgID = obs.imageID;
			// Only consider observations in local or fixed images
			auto pose_it = poseParams.find(imgID);
			if (pose_it == poseParams.end())
				continue;
			const Image& img = scene.images[imgID];
			ASSERT(obs.featureID < img.keypoints.size());
			const cv::KeyPoint& kp = img.keypoints[obs.featureID];
			// Compute weight from keypoint response / size (if enabled)
			ceres::LossFunction* residual_loss_function = loss_function;
			if (config.useKeypointConfidence) {
				double weight = Image::ComputeKeypointPrecision(kp, config.minKeypointResponse);
				if (weight <= 0.0)
					continue; // skip low-confidence keypoints
				if (weight != 1.0)
					residual_loss_function = new ceres::ScaledLoss(loss_function, weight, ceres::DO_NOT_TAKE_OWNERSHIP);
			}

			switch (img.GetCameraType()) {
			case CameraType::PINHOLE: {
				DoubleArr& intr = intrinsicParams.at(img.pCamera);
				ceres::CostFunction* cost_function =
					#if 0
					new PinholeReprojectionErrorAnalytic(kp.pt.x, kp.pt.y);
					#else
					PinholeReprojectionError::Create(kp.pt.x, kp.pt.y);
					#endif
				problem.AddResidualBlock(
					cost_function,
					residual_loss_function,
					pose_it->second.data(),      // Pose params
					intr.data(),      // Intrinsic params (will be fixed)
					track.position.ptr()     // Point params
				);
			} break;
			case CameraType::SPHERICAL: {
				// Spherical error is already scaled to pixels and weighted inside the functor
				ceres::CostFunction* cost_function = SphericalAngularReprojectionError::Create(
					kp.pt.x, kp.pt.y, img.pCamera->GetWidth(), img.pCamera->GetHeight());
				problem.AddResidualBlock(
					cost_function,
					residual_loss_function,
					pose_it->second.data(),      // Pose params
					track.position.ptr()     // Point params
				);
			} break;
			}
			++numReprojResiduals;
		}
	}

	// Set quaternion manifold for all pose blocks
	#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
	// Ceres 2.1+: Use ProductManifold for SE(3) poses
	auto* se3_manifold = new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>{
		ceres::QuaternionManifold{}, ceres::EuclideanManifold<3>{}};
	for (auto& pair : poseParams)
		problem.SetManifold(pair.second.data(), se3_manifold);
	#else
	// Ceres 2.0: Use parameterizations
	auto* quaternion_param = new ceres::QuaternionParameterization;
	auto* identity_param = new ceres::IdentityParameterization(3);
	auto* pose_param = new ceres::ProductParameterization(quaternion_param, identity_param);
	for (auto& pair : poseParams)
		problem.SetParameterization(pair.second.data(), pose_param);
	#endif

	// 4. Set fixed parameters
	if (!intrinsicParams.empty()) {
		for (auto& pair : intrinsicParams) {
			ASSERT(!pair.second.empty());
			problem.SetParameterBlockConstant(pair.second.data());
		}
		DEBUG("Fixed all intrinsic parameters");
	}

	// Fixed images
	for (uint32_t imgID : fixedImages) {
		auto it = poseParams.find(imgID);
		if (it != poseParams.end())
			problem.SetParameterBlockConstant(it->second.data());
	}

	// Fix first camera if no fixed images (gauge freedom)
	if (fixedImages.empty() && !localImages.empty()) {
		IIndex refID = *localImages.begin();
		auto it = poseParams.find(refID);
		if (it != poseParams.end()) {
			problem.SetParameterBlockConstant(it->second.data());
			VERBOSE("Fixed reference camera %u", refID);
		}
	}

	// Optionally disable pose/point refinement
	if (!config.IsRefiningPoses()) {
		for (auto& pair : poseParams)
			problem.SetParameterBlockConstant(pair.second.data());
		DEBUG("Camera poses (local BA): FIXED");
	} else if (!config.refinePosesRotation || !config.refinePosesPosition) {
		// Selectively disable rotation and/or position refinement
		// Pose format: [qw, qx, qy, qz, Cx, Cy, Cz]
		std::vector<int> constantParams;
		for (auto& pair : poseParams) {
			constantParams.clear();
			if (!config.refinePosesRotation) {
				// Fix rotation (indices 0-3)
				constantParams.push_back(0);
				constantParams.push_back(1);
				constantParams.push_back(2);
				constantParams.push_back(3);
			}
			if (!config.refinePosesPosition) {
				// Fix position (indices 4-6)
				constantParams.push_back(4);
				constantParams.push_back(5);
				constantParams.push_back(6);
			}
			#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
			// Ceres 2.1+: Use ProductManifold with SubsetManifold
			auto* subset = new ceres::SubsetManifold(7, constantParams);
			problem.SetManifold(pair.second.data(), subset);
			#else
			// Ceres 2.0: Use parameterizations with SubsetParameterization
			auto* subset = new ceres::SubsetParameterization(7, constantParams);
			problem.SetParameterization(pair.second.data(), subset);
			#endif
		}
		DEBUG("Camera poses (local BA): rotation=%s, position=%s",
		      config.refinePosesRotation ? "OPTIMIZED" : "FIXED",
		      config.refinePosesPosition ? "OPTIMIZED" : "FIXED");
	}
	if (!config.refinePoints) {
		// Fix all active points
		for (uint32_t pointID : activePoints)
			problem.SetParameterBlockConstant(scene.tracks[pointID].position.ptr());
		DEBUG("3D points (local BA): FIXED");
	}

	// Solve
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	#ifndef _RELEASE
	options.minimizer_progress_to_stdout = true;
	#else
	options.minimizer_progress_to_stdout = false;
	#endif
	options.max_num_iterations = config.maxIterations;
	options.num_threads = config.numThreads > 0 ? config.numThreads : std::thread::hardware_concurrency();

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	VERBOSE("BA Summary: %s", summary.BriefReport().c_str());
	if (!summary.IsSolutionUsable()) {
		VERBOSE("error: bundle adjustment failed");
		return false;
	}

	// 5. Update scene (only local images)
	for (IIndex imgID : localImages) {
		auto it = poseParams.find(imgID);
		if (it != poseParams.end())
			QuaternionAndCenterToPose3D(it->second.data(), scene.images[imgID]);
	}

	DEBUG("Local bundle adjustment complete: %u reprojection residuals, %.4g -> %.4g cost (%s)",
	    numReprojResiduals, summary.initial_cost, summary.final_cost, TD_TIMER_GET_FMT().c_str());

	// Report average reprojection errors for local window
	ComputeTracksMeanReprojectionError(scene);
	return true;
}
/*----------------------------------------------------------------*/


bool SFM::PinholeReprojectionJacobianTest()
{
	TD_TIMER_START();
	VERBOSE("\n--- Testing PinholeReprojectionErrorAnalytic Jacobians ---");

	// Create synthetic test data
	const double observed_x = 320.5;
	const double observed_y = 240.7;

	// Test parameters
	double pose[7] = {0.1, 0.2, 0.05, 0.97, 1.0, 0.5, 3.0}; // quat + center
	double intrinsics[12] = {500.0, 1.0, 320.0, 240.0, 0.1, -0.05, 0.01, 0.001, -0.001, 0.0, 0.0, 0.0};
	double point[3] = {2.0, 1.5, 5.0};

	// Normalize quaternion
	Eigen::Map<Eigen::Vector4d>(pose).normalize();

	// Evaluate analytic cost function
	PinholeReprojectionErrorAnalytic analytic_cost(observed_x, observed_y);
	double analytic_residuals[2];
	double* analytic_jacobians[3];
	double analytic_J_pose[2*7];
	double analytic_J_intrinsics[2*12];
	double analytic_J_point[2*3];
	analytic_jacobians[0] = analytic_J_pose;
	analytic_jacobians[1] = analytic_J_intrinsics;
	analytic_jacobians[2] = analytic_J_point;
	const double* params[3] = {pose, intrinsics, point};
	const double* const* params_const = params;
	if (!analytic_cost.Evaluate(params_const, analytic_residuals, analytic_jacobians)) {
		VERBOSE("FAILED: Analytic cost evaluation failed");
		return false;
	}

	// Evaluate auto-diff cost function for comparison
	std::unique_ptr<ceres::CostFunction> autodiff_cost(PinholeReprojectionError::Create(observed_x, observed_y));
	double autodiff_residuals[2];
	double* autodiff_jacobians[3];
	double autodiff_J_pose[2*7];
	double autodiff_J_intrinsics[2*12];
	double autodiff_J_point[2*3];
	autodiff_jacobians[0] = autodiff_J_pose;
	autodiff_jacobians[1] = autodiff_J_intrinsics;
	autodiff_jacobians[2] = autodiff_J_point;
	if (!autodiff_cost->Evaluate(params_const, autodiff_residuals, autodiff_jacobians)) {
		VERBOSE("FAILED: Auto-diff cost evaluation failed");
		return false;
	}

	// Compute numeric Jacobians using finite differences
	const double epsilon = 1e-8;
	double numeric_J_pose[2*7];
	double numeric_J_intrinsics[2*12];
	double numeric_J_point[2*3];

	// Jacobian w.r.t. pose (7 params)
	for (int i = 0; i < 7; ++i) {
		double pose_plus[7], pose_minus[7];
		std::memcpy(pose_plus, pose, 7 * sizeof(double));
		std::memcpy(pose_minus, pose, 7 * sizeof(double));
		pose_plus[i] += epsilon;
		pose_minus[i] -= epsilon;

		// Renormalize quaternion if perturbing quaternion components
		if (i < 4) {
			Eigen::Map<Eigen::Vector4d>(pose_plus).normalize();
			Eigen::Map<Eigen::Vector4d>(pose_minus).normalize();
		}

		double res_plus[2], res_minus[2];
		const double* params_plus[3] = {pose_plus, intrinsics, point};
		const double* params_minus[3] = {pose_minus, intrinsics, point};
		analytic_cost.Evaluate(params_plus, res_plus, nullptr);
		analytic_cost.Evaluate(params_minus, res_minus, nullptr);

		numeric_J_pose[0*7 + i] = (res_plus[0] - res_minus[0]) / (2.0 * epsilon);
		numeric_J_pose[1*7 + i] = (res_plus[1] - res_minus[1]) / (2.0 * epsilon);
	}

	// Jacobian w.r.t. intrinsics (12 params)
	for (int i = 0; i < 12; ++i) {
		double intr_plus[12], intr_minus[12];
		std::memcpy(intr_plus, intrinsics, 12 * sizeof(double));
		std::memcpy(intr_minus, intrinsics, 12 * sizeof(double));
		intr_plus[i] += epsilon;
		intr_minus[i] -= epsilon;

		double res_plus[2], res_minus[2];
		const double* params_plus[3] = {pose, intr_plus, point};
		const double* params_minus[3] = {pose, intr_minus, point};
		analytic_cost.Evaluate(params_plus, res_plus, nullptr);
		analytic_cost.Evaluate(params_minus, res_minus, nullptr);

		numeric_J_intrinsics[0*12 + i] = (res_plus[0] - res_minus[0]) / (2.0 * epsilon);
		numeric_J_intrinsics[1*12 + i] = (res_plus[1] - res_minus[1]) / (2.0 * epsilon);
	}

	// Jacobian w.r.t. point (3 params)
	for (int i = 0; i < 3; ++i) {
		double point_plus[3], point_minus[3];
		std::memcpy(point_plus, point, 3 * sizeof(double));
		std::memcpy(point_minus, point, 3 * sizeof(double));
		point_plus[i] += epsilon;
		point_minus[i] -= epsilon;

		double res_plus[2], res_minus[2];
		const double* params_plus[3] = {pose, intrinsics, point_plus};
		const double* params_minus[3] = {pose, intrinsics, point_minus};
		analytic_cost.Evaluate(params_plus, res_plus, nullptr);
		analytic_cost.Evaluate(params_minus, res_minus, nullptr);

		numeric_J_point[0*3 + i] = (res_plus[0] - res_minus[0]) / (2.0 * epsilon);
		numeric_J_point[1*3 + i] = (res_plus[1] - res_minus[1]) / (2.0 * epsilon);
	}

	// Compare Jacobians (analytic vs numeric vs auto-diff)
	const double jacobian_tol = 2.2e-5; // Tolerance for manifold-aware numerical differentiation
	double max_diff_numeric = 0.0;
	double max_diff_autodiff = 0.0;

	// Check pose Jacobian (2x7)
	for (int i = 0; i < 2; ++i) {
		// Project quaternion part of autodiff Jacobian [i*7, i*7+4) onto tangent space
		// to match manifold-aware derivatives (numeric/analytic)
		double dot = 0.0;
		for (int k = 0; k < 4; ++k) dot += autodiff_J_pose[i*7 + k] * pose[k];
		for (int k = 0; k < 4; ++k) autodiff_J_pose[i*7 + k] -= dot * pose[k];

		for (int j = 0; j < 7; ++j) {
			const int idx = i*7 + j;
			const double diff_numeric = ABS(analytic_J_pose[idx] - numeric_J_pose[idx]);
			const double diff_autodiff = ABS(analytic_J_pose[idx] - autodiff_J_pose[idx]);
			max_diff_numeric = MAX(max_diff_numeric, diff_numeric);
			max_diff_autodiff = MAX(max_diff_autodiff, diff_autodiff);
			if (diff_numeric > jacobian_tol) {
				VERBOSE("FAILED: Pose Jacobian[%d] mismatch (numeric): analytic=%.6e, numeric=%.6e, diff=%.6e",
				        idx, analytic_J_pose[idx], numeric_J_pose[idx], diff_numeric);
				return false;
			}
			if (diff_autodiff > jacobian_tol) {
				VERBOSE("FAILED: Pose Jacobian[%d] mismatch (auto-diff): analytic=%.6e, autodiff=%.6e, diff=%.6e",
				        idx, analytic_J_pose[idx], autodiff_J_pose[idx], diff_autodiff);
				return false;
			}
		}
	}

	// Check intrinsics Jacobian (2x12)
	for (int i = 0; i < 2*12; ++i) {
		const double diff_numeric = ABS(analytic_J_intrinsics[i] - numeric_J_intrinsics[i]);
		const double diff_autodiff = ABS(analytic_J_intrinsics[i] - autodiff_J_intrinsics[i]);
		max_diff_numeric = MAX(max_diff_numeric, diff_numeric);
		max_diff_autodiff = MAX(max_diff_autodiff, diff_autodiff);
		if (diff_numeric > jacobian_tol) {
			VERBOSE("FAILED: Intrinsics Jacobian[%d] mismatch (numeric): analytic=%.6e, numeric=%.6e, diff=%.6e",
			        i, analytic_J_intrinsics[i], numeric_J_intrinsics[i], diff_numeric);
			return false;
		}
		if (diff_autodiff > jacobian_tol) {
			VERBOSE("FAILED: Intrinsics Jacobian[%d] mismatch (auto-diff): analytic=%.6e, autodiff=%.6e, diff=%.6e",
			        i, analytic_J_intrinsics[i], autodiff_J_intrinsics[i], diff_autodiff);
			return false;
		}
	}

	// Check point Jacobian (2x3)
	for (int i = 0; i < 2*3; ++i) {
		const double diff_numeric = ABS(analytic_J_point[i] - numeric_J_point[i]);
		const double diff_autodiff = ABS(analytic_J_point[i] - autodiff_J_point[i]);
		max_diff_numeric = MAX(max_diff_numeric, diff_numeric);
		max_diff_autodiff = MAX(max_diff_autodiff, diff_autodiff);
		if (diff_numeric > jacobian_tol) {
			VERBOSE("FAILED: Point Jacobian[%d] mismatch (numeric): analytic=%.6e, numeric=%.6e, diff=%.6e",
			        i, analytic_J_point[i], numeric_J_point[i], diff_numeric);
			return false;
		}
		if (diff_autodiff > jacobian_tol) {
			VERBOSE("FAILED: Point Jacobian[%d] mismatch (auto-diff): analytic=%.6e, autodiff=%.6e, diff=%.6e",
			        i, analytic_J_point[i], autodiff_J_point[i], diff_autodiff);
			return false;
		}
	}

	VERBOSE("PASSED: All Jacobians match within tolerance (numeric max diff=%.2e, auto-diff max diff=%.2e) %s",
	        max_diff_numeric, max_diff_autodiff, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/
