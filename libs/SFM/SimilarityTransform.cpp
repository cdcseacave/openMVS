/*
 * SimilarityTransform.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "SimilarityTransform.h"
#include "BundleAdjustment.h"
#include "Pose.h"
#include "../Common/AutoEstimator.h"
#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#pragma pop_macro("VERBOSE")
#pragma pop_macro("LOG")

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

// Kernel for RANSAC similarity transform estimation
class SimilarityTransformKernel
{
public:
	typedef Transform Model;
	typedef std::vector<Model> Models;
	enum { MINIMUM_SAMPLES = 3 };
	enum { MAX_MODELS = 1 };

	SimilarityTransformKernel(const Point3Arr& src, const Point3Arr& dst)
		: src_(src), dst_(dst) {}

	size_t NumSamples() const { return src_.size(); }

	bool Fit(const std::vector<size_t>& samples, Models& models) const {
		Point3Arr srcSubset, dstSubset;
		srcSubset.reserve(samples.size());
		dstSubset.reserve(samples.size());
		for (size_t idx : samples) {
			srcSubset.push_back(src_[idx]);
			dstSubset.push_back(dst_[idx]);
		}
		// Use the core estimation
		const Transform t = EstimateSimilarityTransform(srcSubset, dstSubset);
		if (t.scale <= 0)
			return false;
		models.push_back(t);
		return true;
	}

	void EvaluateModel(const Model& model) {
		model_ = model;
	}

	double Error(size_t index) const {
		const Point3 p_transformed = model_ * src_[index];
		return normSq(p_transformed - dst_[index]);
	}

private:
	const Point3Arr& src_;
	const Point3Arr& dst_;
	Model model_;
};

// Cost functor for refining similarity transform with Ceres
struct SimilarityResidual {
	SimilarityResidual(const Point3& src, const Point3& dst)
		: src_(src), dst_(dst) {}

	template <typename T>
	bool operator()(const T* const transform, T* residuals) const {
		// Transform: [quaternion[4], t[3], scale]
		const T* quaternion = transform;
		const T* translation = transform + 4;
		const T scale = transform[7];
		// Apply rotation
		T p_rot[3];
		ceres::UnitQuaternionRotatePoint(quaternion, Cast<T>(src_).ptr(), p_rot);
		// Apply scale and translation: dst = s * R * src + t
		residuals[0] = scale * p_rot[0] + translation[0] - dst_(0);
		residuals[1] = scale * p_rot[1] + translation[1] - dst_(1);
		residuals[2] = scale * p_rot[2] + translation[2] - dst_(2);
		return true;
	}

	static ceres::CostFunction* Create(const Point3& src, const Point3& dst) {
		return new ceres::AutoDiffCostFunction<SimilarityResidual, 3, 8>(
			new SimilarityResidual(src, dst));
	}

private:
	const Point3 src_, dst_;
};

unsigned SFM::EstimateSimilarityTransform(
	const Point3Arr& srcPoints,
	const Point3Arr& dstPoints,
	Transform& transform,
	double threshold,
	bool refine,
	size_t maxIters,
	double confidence)
{
	const size_t n = srcPoints.size();
	if (n != dstPoints.size() || n < 3) {
		VERBOSE("error: invalid correspondences (src: %u, dst: %u)",
			(unsigned)srcPoints.size(), (unsigned)dstPoints.size());
		return 0;
	}

	if (threshold > 0.0) {
		// RANSAC estimation
		SimilarityTransformKernel kernel(srcPoints, dstPoints);
		UniformSampler sampler;
		std::vector<size_t> inliers;
		#ifdef ACRANSAC_SAMPLE_INLIERS
		RANSAC(kernel, sampler, inliers, transform, threshold, maxIters);
		#else
		RANSAC(kernel, sampler, inliers, transform, threshold, confidence, maxIters);
		#endif
		if (inliers.size() < SimilarityTransformKernel::MINIMUM_SAMPLES) {
			VERBOSE("error: Similarity-transform RANSAC failed to find enough inliers");
			return 0;
		}
		DEBUG_EXTRA("Similarity-transform RANSAC found %u inliers (%.2f%%)", (unsigned)inliers.size(), 100.0f * inliers.size() / n);

		// Refine using inliers (disable RANSAC to avoid recursion);
		// pass nullptr so the recursive call does not overwrite our inlier count.
		Point3Arr srcInliers, dstInliers;
		srcInliers.reserve(inliers.size());
		dstInliers.reserve(inliers.size());
		for (size_t idx : inliers) {
			srcInliers.push_back(srcPoints[idx]);
			dstInliers.push_back(dstPoints[idx]);
		}
		return EstimateSimilarityTransform(srcInliers, dstInliers, transform, 0.0, refine && inliers.size() < n);
	}

	transform = EstimateSimilarityTransform(srcPoints, dstPoints);
	if (!ISFINITE(transform.scale) || transform.scale <= 0) {
		VERBOSE("error: degenerate similarity transform (scale %.3g), points likely coincident or collinear", transform.scale);
		return 0;
	}
	DEBUG_EXTRA("Estimated transform: scale %.3g, translation %.3g, rotation %.3g",
		transform.scale, norm(transform.t), FrobeniusNorm(transform.R));

	// Optional: refine with Ceres when we have enough correspondences
	if (refine && n >= 10) {
		// Parameters: [quaternion[4], t[3], scale]
		Pose3D pose(transform.R, transform.t);
		double params[8];
		Pose3DToQuaternionAndCenter(pose, params);
		params[7] = transform.scale;
		// Set quaternion manifold for all pose blocks
		ceres::Problem problem;
		#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
		// Ceres 2.1+: Use ProductManifold to combine QuaternionManifold (4 params) + EuclideanManifold (3 params) + Scale (1 param)
		// This represents SE(3): rotation (quaternion, 3 DOF tangent space) + translation (Euclidean, 3 DOF) + scale (1 DOF)
		auto* se3_manifold = new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<4>>{
			ceres::QuaternionManifold{}, ceres::EuclideanManifold<4>{} };
		problem.AddParameterBlock(params, 8, se3_manifold);
		#else
		// Ceres 2.0: Use parameterizations
		auto* quaternion_param = new ceres::QuaternionParameterization;
		auto* identity_param = new ceres::IdentityParameterization(4);
		auto* pose_param = new ceres::ProductParameterization(quaternion_param, identity_param);
		problem.AddParameterBlock(params, 8);
		problem.SetParameterization(params, pose_param);
		#endif
		// Build Ceres problem
		FOREACH(i, srcPoints) {
			problem.AddResidualBlock(
				SimilarityResidual::Create(srcPoints[i], dstPoints[i]),
				nullptr,
				params
			);
		}
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		#ifndef _RELEASE
		options.minimizer_progress_to_stdout = true;
		#else
		options.minimizer_progress_to_stdout = false;
		#endif
		options.max_num_iterations = 100;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		DEBUG("BA Summary: %s", summary.BriefReport().c_str());
		if (!summary.IsSolutionUsable() || !ISFINITE(params[7]) || params[7] <= 0) {
			VERBOSE("error: similarity transform refinement failed");
			return 0;
		}
		// Extract refined parameters
		QuaternionAndCenterToPose3D(params, pose);
		transform.R = pose.R;
		transform.t = pose.C;
		transform.scale = params[7];
		DEBUG_EXTRA("Refined transform: scale %.3g, translation %.3g, rotation %.3g, cost %.4g -> %.4g",
			transform.scale, norm(transform.t), FrobeniusNorm(transform.R), summary.initial_cost, summary.final_cost);
	}
	return n;
}
/*----------------------------------------------------------------*/


unsigned SFM::EstimateSimilarityTransformWithRotations(
	const Point3Arr& srcCenters,
	const Point3Arr& dstCenters,
	const Matrix3x3Arr& srcRots,
	const Matrix3x3Arr& dstRots,
	Transform& transform,
	double threshold)
{
	ASSERT(srcCenters.size() == dstCenters.size() &&
		srcCenters.size() == srcRots.size() && srcRots.size() == dstRots.size());
	// detect a (nearly) collinear destination-center set: with the second principal spread
	// below the threshold, the roll about the common axis is unconstrained by the centers
	bool wellSpread = true;
	if (threshold > 0 && dstCenters.size() >= 3) {
		Point3 mean(Point3::ZERO);
		for (const Point3& C: dstCenters)
			mean += C;
		mean /= (REAL)dstCenters.size();
		Eigen::Matrix3d cov(Eigen::Matrix3d::Zero());
		for (const Point3& C: dstCenters) {
			const Eigen::Vector3d d(Point3d(C-mean));
			cov += d * d.transpose();
		}
		cov /= (double)dstCenters.size();
		// standard deviation along each principal axis, in increasing order
		const Eigen::Vector3d spread(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(cov, Eigen::EigenvaluesOnly).eigenvalues().cwiseMax(0.).cwiseSqrt());
		wellSpread = spread(1) >= threshold;
		if (!wellSpread)
			DEBUG("Centers are nearly collinear (spread %gx%gx%g, threshold %g): "
				"estimating the alignment rotation from the paired rotations",
				spread(2), spread(1), spread(0), threshold);
	}
	if (wellSpread)
		return EstimateSimilarityTransform(srcCenters, dstCenters, transform, threshold);

	// rotation from robust rotation averaging (dstR ~= srcR * alignR, and poses transform
	// as R_new = R * T.R^t, so T.R = alignR^t)
	Matrix3x3 alignR;
	if (!EstimateRotationAlignment(srcRots, dstRots, alignR)) {
		DEBUG("Rotation alignment of the collinear center set failed");
		return 0;
	}
	transform.R = RMatrix(Matrix3x3(alignR.t()));
	// scale and translation by least squares with the rotation fixed
	const Eigen::Matrix3d R = transform.R;
	Eigen::Vector3d meanSrc(Eigen::Vector3d::Zero()), meanDst(Eigen::Vector3d::Zero());
	FOREACH(k, srcCenters) {
		meanSrc += Eigen::Vector3d(Point3d(srcCenters[k]));
		meanDst += Eigen::Vector3d(Point3d(dstCenters[k]));
	}
	meanSrc /= (double)srcCenters.size();
	meanDst /= (double)dstCenters.size();
	double num = 0, den = 0;
	FOREACH(k, srcCenters) {
		const Eigen::Vector3d dr(R * (Eigen::Vector3d(Point3d(srcCenters[k])) - meanSrc));
		const Eigen::Vector3d dp(Eigen::Vector3d(Point3d(dstCenters[k])) - meanDst);
		num += dp.dot(dr);
		den += dr.squaredNorm();
	}
	if (den <= 0 || num <= 0) {
		DEBUG("Source centers are degenerate, cannot recover the scale of the collinear alignment");
		return 0;
	}
	transform.scale = num / den;
	const Eigen::Vector3d t(meanDst - transform.scale * (R * meanSrc));
	transform.t = Point3(t.x(), t.y(), t.z());
	// the rotation came from the rotations alone, so validate it against the centers:
	// the aligned centers must land within the threshold of their counterparts
	DoubleArr residuals(srcCenters.size());
	unsigned numInliers = 0;
	FOREACH(k, srcCenters) {
		residuals[k] = (double)norm(Point3(transform * srcCenters[k] - dstCenters[k]));
		if (residuals[k] <= threshold)
			++numInliers;
	}
	const double medianResidual = residuals.GetMedian();
	if (medianResidual > threshold) {
		DEBUG("Aligning the collinear center set is too inaccurate (median residual %g, threshold %g)",
			medianResidual, threshold);
		return 0;
	}
	return numInliers;
}
/*----------------------------------------------------------------*/


// Similarity transform unit test
bool SFM::TestSimilarityTransform()
{
	#ifndef _RELEASE
	std::mt19937 rng(123);
	#else
	std::random_device rd;
	std::mt19937 rng(rd());
	#endif

	// Define a known similarity: scale * R * x + t
	const double scale = 1.5;
	Eigen::AngleAxisd aa(M_PI / 13.0, Eigen::Vector3d(0.3, 0.5, 0.2).normalized());
	Matrix3x3 R = aa.toRotationMatrix();
	Point3 t(0.7, -0.3, 0.2);

	// Create a small set of 3D points
	Point3Arr src, dst;
	std::uniform_real_distribution<REAL> transDist(-100, 100);
	for (int i = 0; i < 6; ++i) {
		src.emplace_back(transDist(rng), transDist(rng), transDist(rng));
		dst.emplace_back(scale * R * src[i] + t);
	}

	Transform tr;
	if (EstimateSimilarityTransform(src, dst, tr) == 0) {
		VERBOSE("SimilarityTransformUnitTest: EstimateSimilarityTransform failed");
		return false;
	}

	// Check scale within tolerance
	const double sErr = ABS(tr.scale - scale);
	const double tol = 1e-2;
	VERBOSE("SimilarityTransformUnitTest: scale=%f (expected %f)", tr.scale, scale);
	if (sErr >= tol)
		return false;

	// Validate transform by applying it to source points and comparing to destination
	double maxErr = 0.0;
	for (size_t i = 0; i < src.size(); ++i) {
		const Point3 mapped = tr.scale * tr.R * src[i] + tr.t;
		double err = norm(mapped - dst[i]);
		maxErr = MAXF(maxErr, err);
	}
	VERBOSE("SimilarityTransformUnitTest: All tests passed (max residual = %.6g)", maxErr);
	return maxErr < 1e-6;
}
/*----------------------------------------------------------------*/