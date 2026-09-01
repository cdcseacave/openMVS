////////////////////////////////////////////////////////////////////
// ViewGraphCalibrator.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ViewGraphCalibrator.h"
#include "Scene.h"

#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#pragma push_macro("CHECK")
#undef CHECK
#pragma push_macro("DEBUG_EXTRA")
#undef DEBUG_EXTRA
#include <ceres/ceres.h>
#pragma pop_macro("DEBUG_EXTRA")
#pragma pop_macro("CHECK")
#pragma pop_macro("LOG")
#pragma pop_macro("VERBOSE")

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ViewGhCa"));

namespace SFM {

// ----------------------------------------
// Fetzer Focal Length Cost Functions
// ----------------------------------------
// Based on "Direct Focal Length Calibration from Two Views" by Fetzer et al.
// These helper functions compute intermediate values for the Fetzer cost function.
inline Eigen::Vector4d FetzerFocalLengthCostHelper(
	const Eigen::Vector3d& ai, const Eigen::Vector3d& bi,
	const Eigen::Vector3d& aj, const Eigen::Vector3d& bj,
	const int u, const int v)
{
	return Eigen::Vector4d{
		ai(u) * aj(v) - ai(v) * aj(u),
		ai(u) * bj(v) - ai(v) * bj(u),
		bi(u) * aj(v) - bi(v) * aj(u),
		bi(u) * bj(v) - bi(v) * bj(u)
	};
}

inline std::array<Eigen::Vector4d, 2> FetzerFocalLengthCostHelper(const Eigen::Matrix3d& i1_G_i0) {
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(i1_G_i0, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Vector3d s = svd.singularValues();
	Eigen::Vector3d v_0 = svd.matrixV().col(0);
	Eigen::Vector3d v_1 = svd.matrixV().col(1);
	Eigen::Vector3d u_0 = svd.matrixU().col(0);
	Eigen::Vector3d u_1 = svd.matrixU().col(1);
	// Compute ai, aj, bi, bj components as in equation 11 of the paper
	// ai components: based on V matrix (right singular vectors) and singular values
	// ai = (σ₁²(v₁₁² + v₁₂²), σ₁σ₂(v₁₁v₂₁ + v₁₂v₂₂), σ₂²(v₂₁² + v₂₂²))
	Eigen::Vector3d ai(s(0) * s(0) * (SQUARE(v_0(0)) + SQUARE(v_0(1))),
	                   s(0) * s(1) * (v_0(0) * v_1(0) + v_0(1) * v_1(1)),
	                   s(1) * s(1) * (SQUARE(v_1(0)) + SQUARE(v_1(1))));
	// aj components: based on U matrix (left singular vectors)
	// aj = (u₂₁² + u₂₂², -(u₁₁u₂₁ + u₁₂u₂₂), u₁₁² + u₁₂²)
	// Note: the ORDER is reversed - starts with second column u₂, ends with first column u₁
	Eigen::Vector3d aj(SQUARE(u_1(0)) + SQUARE(u_1(1)),
	                   -(u_0(0) * u_1(0) + u_0(1) * u_1(1)),
	                   SQUARE(u_0(0)) + SQUARE(u_0(1)));
	// bi components: based on principal point projection onto V and singular values
	// bi = (σ₁²(cₚᵢᵀv₁)², σ₁σ₂(cₚᵢᵀv₁)(cₚᵢᵀv₂), σ₂²(cₚᵢᵀv₂)²)
	// When cₚᵢ normalized to [0,0,1]ᵀ, cₚᵢᵀvₖ = vₖ₃ (third component)
	Eigen::Vector3d bi(s(0) * s(0) * SQUARE(v_0(2)),
	                   s(0) * s(1) * v_0(2) * v_1(2),
	                   s(1) * s(1) * SQUARE(v_1(2)));
	// bj components: based on principal point projection onto U
	// bj = ((cₚⱼᵀu₂)², -(cₚⱼᵀu₁)(cₚⱼᵀu₂), (cₚⱼᵀu₁)²)
	// When cₚⱼ normalized to [0,0,1]ᵀ, cₚⱼᵀuₖ = uₖ₃ (third component)
	// Note: the ORDER is reversed - starts with u₂, ends with u₁
	Eigen::Vector3d bj(SQUARE(u_1(2)),
	                   -(u_0(2) * u_1(2)),
	                   SQUARE(u_0(2)));
	// Compute d_01 and d_12 vectors as in equations 12 of the paper
	Eigen::Vector4d d_01 = FetzerFocalLengthCostHelper(ai, bi, aj, bj, 1, 0);
	Eigen::Vector4d d_12 = FetzerFocalLengthCostHelper(ai, bi, aj, bj, 2, 1);
	return std::array<Eigen::Vector4d, 2>{d_01, d_12};
}

// Fetzer focal length cost function for two different cameras:
// estimates focal lengths fi and fj from the fundamental matrix F and principal points of both cameras.
class FetzerFocalLengthCostFunctor {
public:
	FetzerFocalLengthCostFunctor(const Matrix3x3d& i1_F_i0,
		const Point2d& principalPoint0, const Point2d& principalPoint1)
	{
		Matrix3x3d K0 = Matrix3x3d::IDENTITY;
		K0(0, 2) = principalPoint0.x;
		K0(1, 2) = principalPoint0.y;

		Matrix3x3d K1 = Matrix3x3d::IDENTITY;
		K1(0, 2) = principalPoint1.x;
		K1(1, 2) = principalPoint1.y;

		const Matrix3x3d i1_G_i0 = ImagePair::DecomposeFundamentalMatrix(i1_F_i0, K0, K1);

		const std::array<Eigen::Vector4d, 2> ds = FetzerFocalLengthCostHelper(i1_G_i0);
		d_01 = ds[0];
		d_12 = ds[1];
	}

	static ceres::CostFunction* Create(const Matrix3x3d& i1_F_i0,
		const Point2d& principalPoint0, const Point2d& principalPoint1)
	{
		return (new ceres::AutoDiffCostFunction<FetzerFocalLengthCostFunctor, 2, 1, 1>(
			new FetzerFocalLengthCostFunctor(i1_F_i0, principalPoint0, principalPoint1)));
	}

	template <typename T>
	bool operator()(const T* const fi_, const T* const fj_, T* residuals) const
	{
		const Eigen::Vector<T, 4> d_01_ = d_01.cast<T>();
		const Eigen::Vector<T, 4> d_12_ = d_12.cast<T>();
		const T fi2 = SQUARE(fi_[0]);
		const T fj2 = SQUARE(fj_[0]);
		// Compute residual based on eq. 13 in the paper
		T di(fj2 * d_01_(0) + d_01_(1));
		if (di == 0.0) di = T(1e-6);
		const T K0_01 = (fj2 * d_01_(2) + d_01_(3)) / di;
		residuals[0] = (fi2 + K0_01) / fi2;
		// Compute residual based on eq. 14 in the paper
		T dj(fi2 * d_12_(0) + d_12_(2));
		if (dj == 0.0) dj = T(1e-6);
		const T K1_12 = (fi2 * d_12_(1) + d_12_(3)) / dj;
		residuals[1] = (fj2 + K1_12) / fj2;
		return true;
	}

private:
	Eigen::Vector4d d_01;
	Eigen::Vector4d d_12;
};


// Fetzer focal length cost function for the same camera:
// estimates focal length f from the fundamental matrix F when both
// images share the same camera (same principal point).
class FetzerFocalLengthSameCameraCostFunctor {
public:
	FetzerFocalLengthSameCameraCostFunctor(const Matrix3x3d& i1_F_i0, const Point2d& principalPoint)
	{
		Matrix3x3d K = Matrix3x3d::IDENTITY;
		K(0, 2) = principalPoint.x;
		K(1, 2) = principalPoint.y;

		const Matrix3x3d i1_G_i0 = ImagePair::DecomposeFundamentalMatrix(i1_F_i0, K, K);

		const std::array<Eigen::Vector4d, 2> ds = FetzerFocalLengthCostHelper(i1_G_i0);
		d_01 = ds[0];
		d_12 = ds[1];
	}

	static ceres::CostFunction* Create(const Matrix3x3d& i1_F_i0, const Point2d& principalPoint)
	{
		return (new ceres::AutoDiffCostFunction<FetzerFocalLengthSameCameraCostFunctor, 2, 1>(
			new FetzerFocalLengthSameCameraCostFunctor(i1_F_i0, principalPoint)));
	}

	template <typename T>
	bool operator()(const T* const f_, T* residuals) const
	{
		const Eigen::Vector<T, 4> d_01_ = d_01.cast<T>();
		const Eigen::Vector<T, 4> d_12_ = d_12.cast<T>();
		const T f2 = SQUARE(f_[0]);
		// Compute residual based on eq. 13 in the paper
		T di(f2 * d_01_(0) + d_01_(1));
		if (di == 0.0) di = T(1e-6);
		const T K0_01 = (f2 * d_01_(2) + d_01_(3)) / di;
		residuals[0] = (f2 + K0_01) / f2;
		// Compute residual based on eq. 14 in the paper
		T dj(f2 * d_12_(0) + d_12_(2));
		if (dj == 0.0) dj = T(1e-6);
		const T K1_12 = (f2 * d_12_(1) + d_12_(3)) / dj;
		residuals[1] = (f2 + K1_12) / f2;
		return true;
	}

private:
	Eigen::Vector4d d_01;
	Eigen::Vector4d d_12;
};
/*----------------------------------------------------------------*/


// ViewGraphCalibrator
ViewGraphCalibrator::ViewGraphCalibrator(const ViewGraphCalibratorConfig& config)
	: config_(config) {}
ViewGraphCalibrator::~ViewGraphCalibrator() = default;

bool ViewGraphCalibrator::Solve(Scene& scene) {
	// Reset the problem
	Reset(scene);

	// Set solver options based on problem size
	ceres::Solver::Options solverOptions;
	if (focals_.size() < 50)
		solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
	else
		solverOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	solverOptions.max_num_iterations = config_.maxIterations;
	solverOptions.num_threads = config_.numThreads > 0 ? config_.numThreads : std::thread::hardware_concurrency();
	#if TD_VERBOSE != TD_VERBOSE_OFF
	solverOptions.minimizer_progress_to_stdout = VERBOSITY_LEVEL > 2;
	#endif

	// Add image pairs to the problem
	AddImagePairsToProblem(scene);
	if (problem_->NumResiduals() <= 0) {
		DEBUG("warning: no valid image pairs with fundamental matrices");
		return true;
	}

	// Parameterize cameras (mark trusted ones as constant)
	const size_t numCamerasToOptimize = ParameterizeCameras(scene);
	if (numCamerasToOptimize == 0) {
		DEBUG("warning: no cameras to optimize (all trusted)");
		return true;
	}

	// Solve the problem
	ceres::Solver::Summary summary;
	ceres::Solve(solverOptions, problem_.get(), &summary);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 1) {
		VERBOSE("Summary: %s", summary.FullReport().c_str());
	} else {
		DEBUG("Summary: %s", summary.BriefReport().c_str());
	}
	#endif
	if (!summary.IsSolutionUsable()) {
		VERBOSE("error: optimization failed");
		return false;
	}

	// Copy results back to cameras
	CopyBackResults(scene);

	// Filter invalid pairs
	if (config_.maxTwoViewError > 0)
		FilterImagePairs(scene);
	return true;
}

void ViewGraphCalibrator::Reset(const Scene& scene) {
	// Initialize focal length parameters from cameras
	focals_.clear();
	for (const CameraPtr pCamera : scene.cameras) {
		if (pCamera->GetType() != CameraType::PINHOLE)
			continue;
		const PinholeCamera* pPinholeCamera = static_cast<const PinholeCamera*>(pCamera);
		// Use average of fx and fy as initial focal length
		focals_[pCamera] = (pPinholeCamera->fx + pPinholeCamera->fy) * 0.5;
	}

	// Set up Ceres problem
	ceres::Problem::Options problemOptions;
	problemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_ = std::make_unique<ceres::Problem>(problemOptions);
	lossFunction_ = std::make_shared<ceres::ArctanLoss>(config_.lossThreshold);
	updatedCameras_.clear();
}

void ViewGraphCalibrator::AddImagePairsToProblem(const Scene& scene) {
	for (const ImagePair& pair : scene.pairs) {
		// Skip pairs without fundamental matrix
		if (!pair.F.has_value())
			continue;
		// Skip pairs with insufficient matches (the pair's evidence, dense supplement included:
		// the same quantity the composite weight just below is built on)
		if (pair.GetNumWeightedInliers() < 15)
			continue;
		// Skip pairs with small weight
		if (pair.GetCompositeWeight() < config_.minPairWeight)
			continue;

		ASSERT(pair.ID1 < scene.images.size());
		ASSERT(pair.ID2 < scene.images.size());
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];

		// Both cameras must be pinhole
		if (img1.GetCameraType() != CameraType::PINHOLE || img2.GetCameraType() != CameraType::PINHOLE)
			continue;
		ASSERT(focals_.count(img1.pCamera) > 0);
		ASSERT(focals_.count(img2.pCamera) > 0);

		// Get principal points
		const PinholeCamera* pPinholeCamera1 = static_cast<const PinholeCamera*>(img1.pCamera);
		Point2d pp1(pPinholeCamera1->cx, pPinholeCamera1->cy);

		// Add residual block
		if (img1.pCamera == img2.pCamera) {
			// Same camera: use single-camera cost function
			problem_->AddResidualBlock(
				FetzerFocalLengthSameCameraCostFunctor::Create(pair.F.value(), pp1),
				lossFunction_.get(),
				&(focals_[img1.pCamera]));
		} else {
			// Different cameras: use two-camera cost function
			const PinholeCamera* pPinholeCamera2 = static_cast<const PinholeCamera*>(img2.pCamera);
			Point2d pp2(pPinholeCamera2->cx, pPinholeCamera2->cy);
			problem_->AddResidualBlock(
				FetzerFocalLengthCostFunctor::Create(pair.F.value(), pp1, pp2),
				lossFunction_.get(),
				&(focals_[img1.pCamera]),
				&(focals_[img2.pCamera]));
		}
	}
}

unsigned ViewGraphCalibrator::ParameterizeCameras(const Scene& scene) {
	unsigned numCamerasToOptimize = 0;
	for (const CameraPtr pCamera : scene.cameras) {
		if (!problem_->HasParameterBlock(&(focals_[pCamera])))
			continue;
		// Set lower bound to avoid negative focal lengths
		problem_->SetParameterLowerBound(&(focals_[pCamera]), 0, 1e-3);
		// If camera has trusted intrinsics, keep it constant
		if (pCamera->TrustIntrinsics() && config_.trustIntrinsics)
			problem_->SetParameterBlockConstant(&(focals_[pCamera]));
		else
			++numCamerasToOptimize;
	}
	return numCamerasToOptimize;
}

unsigned ViewGraphCalibrator::CopyBackResults(Scene& scene) {
	unsigned numRejected = 0;
	updatedCameras_.reserve(focals_.size());
	FOREACH(i, scene.cameras) {
		CameraPtr pCamera = scene.cameras[i];
		if (pCamera->GetType() != CameraType::PINHOLE)
			continue;
		if (!problem_->HasParameterBlock(&(focals_[pCamera])))
			continue;

		PinholeCamera* pPinholeCamera = static_cast<PinholeCamera*>(pCamera);
		const double originalFocal = (pPinholeCamera->fx + pPinholeCamera->fy) * 0.5;
		const double estimatedFocal = focals_[pCamera];

		// Check if estimated focal is reasonable
		const double ratio = estimatedFocal / originalFocal;
		if (ratio < config_.minFocalRatio || ratio > config_.maxFocalRatio) {
			VERBOSE("warning: rejecting degenerate focal estimate %.2f (original: %.2f, ratio: %.2f)",
				estimatedFocal, originalFocal, ratio);
			++numRejected;
			continue;
		}

		// Update camera focal length
		pPinholeCamera->fx = pPinholeCamera->fy = static_cast<REAL>(estimatedFocal);
		pPinholeCamera->trustIntrinsics = true;
		updatedCameras_.insert(pCamera);
		DEBUG_EXTRA("View-Graph calibrator updated camera %u focal length: %.2f -> %.2f",
			i, originalFocal, estimatedFocal);
	}

	if (numRejected > 0) {
		DEBUG("View-Graph calibrator rejected %u degenerate focal estimates", numRejected);
	}
	return numRejected;
}

unsigned ViewGraphCalibrator::FilterImagePairs(Scene& scene) const {
	// Evaluate residuals for all image pairs
	ceres::Problem::EvaluateOptions evalOptions;
	evalOptions.num_threads = config_.numThreads > 0 ? config_.numThreads : std::thread::hardware_concurrency();
	evalOptions.apply_loss_function = false;

	std::vector<double> residuals;
	problem_->Evaluate(evalOptions, nullptr, &residuals, nullptr, nullptr);

	// Mark pairs with high residuals as invalid
	size_t residualIdx = 0;
	unsigned numInvalidPairs = 0;
	const double maxErrorSq = SQUARE(config_.maxTwoViewError);
	for (ImagePair& pair : scene.pairs) {
		// Skip pairs that weren't added to the problem
		if (!pair.F.has_value() || pair.GetNumWeightedInliers() < 15 || pair.GetCompositeWeight() < config_.minPairWeight)
			continue;
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		if (img1.GetCameraType() != CameraType::PINHOLE || img2.GetCameraType() != CameraType::PINHOLE)
			continue;
		// Check residual (2 residuals per pair)
		ASSERT(residualIdx + 1 < residuals.size());
		const Point2d error(residuals[residualIdx], residuals[residualIdx + 1]);
		if (normSq(error) > maxErrorSq) {
			// Mark pair as having invalid calibration (optional: could add flag to ImagePair)
			pair.InvalidateWeight();
			++numInvalidPairs;
			DEBUG_ULTIMATE("Filtered pair (% 4u, % 4u): %.3g residual, %.2f weight",
				pair.ID1, pair.ID2, norm(error), pair.GetCompositeWeight());
		}
		residualIdx += 2;
	}

	DEBUG("View-Graph calibrator marked %u pairs as invalid (high residual)", numInvalidPairs);
	return numInvalidPairs;
}
/*----------------------------------------------------------------*/

} // namespace SFM

#pragma pop_macro("VERBOSE")
