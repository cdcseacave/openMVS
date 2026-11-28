////////////////////////////////////////////////////////////////////
// RelativePoseRefine.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "RelativePoseRefine.h"
#include "BundleAdjustment.h"
#include "ImagePair.h"

#ifdef SFM_USE_CERES

#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#pragma pop_macro("VERBOSE")
#pragma pop_macro("LOG")

using namespace SFM;

// Cost functor with pose parameterized as quaternion (world->cam2) and camera center C2 in world.
// Camera1: R1=I, C1=(0,0,0)
// intr: [f, k1, k2, cx, cy]
struct TwoViewReprojectionError {
	TwoViewReprojectionError(double x1, double y1, double x2, double y2)
		: x1_(x1), y1_(y1), x2_(x2), y2_(y2) {}

	template <typename T>
	bool operator()(const T* const intr, const T* const pose, T* residuals) const {
		typedef Eigen::Matrix<T,2,1> Vector2;
		typedef Eigen::Matrix<T,3,1> Vector3;
		typedef Eigen::Matrix<T,4,1> Vector4;

		const T f = intr[0];
		const T k1 = intr[1];
		const T k2 = intr[2];
		const T cx = intr[3];
		const T cy = intr[4];
		const T* quat = pose;      // quaternion [qw, qx, qy, qz] for R (world->cam2)
		const T* C2 = pose + 4;    // camera2 center in world

		// Project pixel (with distortion)
		const auto ProjectPixel = [&](const Vector3& hp) -> Vector2 {
			const Vector2 x = hp.hnormalized();
			const T r2 = x.squaredNorm();
			const T r4 = r2*r2;
			const T radial = 1.0 + k1*r2 + k2*r4;
			return x * (radial * f) + Vector2(cx, cy);
		};
		// Undistort to get ray directions (unit) in camera coords
		const auto UndistortRay = [&](double xd, double yd) -> Vector3 {
			Vector2 u((xd - cx) / f, (yd - cy) / f);
			const Vector2 cu = u;
			for (int i=0;i<3;++i) {
				T r2 = u.squaredNorm();
				T r4 = r2*r2;
				T radial = 1.0 + k1*r2 + k2*r4;
				u = cu / radial;
			}
			T n = ceres::sqrt(u.squaredNorm() + 1.0);
			return u.homogeneous() / n;
		};
		Vector3 d1_cam = UndistortRay(x1_, y1_); // camera1 direction
		Vector3 d2_cam2 = UndistortRay(x2_, y2_); // camera2 direction in cam2 frame

		// Convert d2 to world: d2_world = R^T * d2_cam2 (use inverse quaternion rotation)
		// Inverse quaternion: [qw, qx, qy, qz] -> [qw, -qx, -qy, -qz]
		Vector4 inv_quat{quat[0], -quat[1], -quat[2], -quat[3]};
		Vector3 d2_world;
		ceres::UnitQuaternionRotatePoint(inv_quat.data(), d2_cam2.data(), d2_world.data());

		// Ray1: origin O1=(0,0,0), direction d1_world = d1_cam (since R1=I)
		const Vector3& d1_world = d1_cam;
		// Ray2: origin O2=C2, direction d2_world
		const Vector3 O2 = Eigen::Map<const Vector3>(C2);

		// Solve closest points between rays: minimize ||s*d1 - (O2 + t*d2)||^2
		// Ray1: P1 = s*d1 (origin at 0)
		// Ray2: P2 = O2 + t*d2
		// Standard solution for closest approach between two 3D lines
		const T d1d1 = d1_world.dot(d1_world);
		const T d1d2 = d1_world.dot(d2_world);
		const T d2d2 = d2_world.dot(d2_world);
		const T O2d1 = O2.dot(d1_world);
		const T O2d2 = O2.dot(d2_world);
		const T denom = d1d1*d2d2 - d1d2*d1d2;
		if (ceres::abs(denom) < 1e-12) {
			residuals[0]=residuals[1]=residuals[2]=residuals[3]=T(0);
			return true;
		}
		const T s = (O2d1*d2d2 - O2d2*d1d2) / denom;
		const T t = (O2d1*d1d2 - O2d2*d1d1) / denom;
		const Vector3 P1 = s * d1_world;
		const Vector3 P2 = O2 + t * d2_world;
		const Vector3 Xw = (P1 + P2) * 0.5; // midpoint in world

		// Project to camera1 (R1=I, C1=0)
		const Vector3& Xc1 = Xw; // since C1=0
		if (Xc1(2) <= 0.0) {
			residuals[0]=residuals[1]=residuals[2]=residuals[3]=T(0);
			return true;
		}
		const Vector2 px1 = ProjectPixel(Xc1);

		// Project to camera2: Xc2 = R*(Xw - C2)
		const Vector3 rel = Xw - O2;
		Vector3 Xc2;
		ceres::UnitQuaternionRotatePoint(quat, rel.data(), Xc2.data());
		if (Xc2(2) <= 0.0) {
			residuals[0]=residuals[1]=residuals[2]=residuals[3]=T(0);
			return true;
		}
		const Vector2 px2 = ProjectPixel(Xc2);

		// Compute residuals
		residuals[0] = px1(0) - x1_;
		residuals[1] = px1(1) - y1_;
		residuals[2] = px2(0) - x2_;
		residuals[3] = px2(1) - y2_;
		return true;
	}

	static ceres::CostFunction* Create(const Point2f& pt1, const Point2f& pt2) {
		return new ceres::AutoDiffCostFunction<TwoViewReprojectionError,4/*residuals*/,5/*intrinsics*/,7/*pose*/>(
			new TwoViewReprojectionError(pt1.x, pt1.y, pt2.x, pt2.y));
	}

	double x1_, y1_, x2_, y2_;
};

bool RelativePoseRefine::RefineTwoViewCalibration(
	const std::vector<cv::KeyPoint>& keypoints1,
	const std::vector<cv::KeyPoint>& keypoints2,
	const std::vector<DMatch>& matches,
	PinholeCamera& camera,
	Pose3D& relativePose,
	const Config& config,
	Result* result)
{
	if (result)
		*result = Result();
	if (matches.size() < 15)
		return false;

	std::array<double,5> intr { camera.fx, camera.k1, camera.k2, camera.cx, camera.cy };
	std::array<double,7> pose; // quaternion[4] + center[3]
	Pose3DToQuaternionAndCenter(relativePose, pose.data());

	// Subsample for speed
	std::vector<size_t> indices(matches.size());
	std::iota(indices.begin(), indices.end(), 0);
	if (matches.size() > config.maxMatches) {
		std::random_device rd; std::mt19937 g(rd());
		std::shuffle(indices.begin(), indices.end(), g);
		indices.resize(config.maxMatches);
	}
	ceres::Problem problem;
	ceres::LossFunction* loss = new ceres::HuberLoss(config.robustThreshold);
	for (size_t idx : indices) {
		const DMatch& m = matches[idx];
		ceres::CostFunction* cost = TwoViewReprojectionError::Create(keypoints1[m.queryIdx].pt, keypoints2[m.trainIdx].pt);
		problem.AddResidualBlock(cost, loss, intr.data(), pose.data());
	}

	// Bounds for variable intrinsics (f,k1,k2)
	std::vector<int> constantIndices;
	problem.SetParameterLowerBound(intr.data(),1,-0.5);
	problem.SetParameterUpperBound(intr.data(),1, 0.5);
	problem.SetParameterLowerBound(intr.data(),2,-0.5);
	problem.SetParameterUpperBound(intr.data(),2, 0.5);
	if (config.refineFocalLength) {
		problem.SetParameterLowerBound(intr.data(),0,camera.fx*0.5);
		problem.SetParameterUpperBound(intr.data(),0,camera.fx*2.0);
	} else {
		constantIndices.push_back(0); // fix f
	}
	// Fix cx,cy by marking them as constant (indices 3 and 4 in the intrinsics array)
	constantIndices.push_back(3);
	constantIndices.push_back(4);

	#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
	ceres::SubsetManifold* subsetManifold = new ceres::SubsetManifold(5, constantIndices);
	problem.SetManifold(intr.data(), subsetManifold);
	// Set quaternion manifold for pose using ProductManifold
	auto* se3Manifold = new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>{
		ceres::QuaternionManifold{}, ceres::EuclideanManifold<3>{}};
	problem.SetManifold(pose.data(), se3Manifold);
	#else
	ceres::SubsetParameterization* subsetParam = new ceres::SubsetParameterization(5, constantIndices);
	problem.SetParameterization(intr.data(), subsetParam);
	// Set quaternion parameterization for pose
	auto* quaternionParam = new ceres::QuaternionParameterization;
	auto* identityParam = new ceres::IdentityParameterization(3);
	auto* poseParam = new ceres::ProductParameterization(quaternionParam, identityParam);
	problem.SetParameterization(pose.data(), poseParam);
	#endif

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.max_num_iterations = config.maxIterations;
	options.minimizer_progress_to_stdout = config.verbose;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	DEBUG_ULTIMATE(summary.FullReport().c_str());
	if (result) {
		result->success = summary.IsSolutionUsable();
		result->initialCost=summary.initial_cost;
		result->finalCost=summary.final_cost;
	}
	if (!summary.IsSolutionUsable())
		return false;

	camera.fx = camera.fy = intr[0];
	camera.k1 = intr[1]; camera.k2 = intr[2];
	// cx,cy unchanged
	QuaternionAndCenterToPose3D(pose.data(), relativePose);
	return true;
}

#else // SFM_USE_CERES not defined

using namespace SFM;
bool RelativePoseRefine::RefineTwoViewCalibration(const std::vector<cv::KeyPoint>&, const std::vector<cv::KeyPoint>&, const std::vector<DMatch>&, PinholeCamera&, Pose3D&, const Config&, Result*) {
	VERBOSE("RelativePoseRefine: Ceres disabled");
	return false;
}

#endif // SFM_USE_CERES
/*----------------------------------------------------------------*/
