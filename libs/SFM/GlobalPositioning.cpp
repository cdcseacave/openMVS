/*
 * GlobalPositioning.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalPositioning.h"
#include "Scene.h"

#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#pragma push_macro("DEBUG_EXTRA")
#undef DEBUG_EXTRA
#include <ceres/ceres.h>
#pragma pop_macro("DEBUG_EXTRA")
#pragma pop_macro("LOG")
#pragma pop_macro("VERBOSE")

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("GlbPos  "));

namespace {

// Helper function to generate random 3D point
Point3 RandPoint3(std::mt19937& randomGenerator, REAL low, REAL high)
{
	std::uniform_real_distribution<REAL> distribution(low, high);
	return Point3(
		distribution(randomGenerator),
		distribution(randomGenerator),
		distribution(randomGenerator));
}

// Computes the error between a translation direction and the direction formed from
// two positions such that: t_ij - scale * (p_j - p_i) is minimized.
// The positions can either be two camera centers or one camera center and one 3D point.
struct BATAPairwiseDirectionCostFunctor
{
	BATAPairwiseDirectionCostFunctor(const Eigen::Vector3d& translationObs) :
		translationObs(translationObs) {}

	template <typename T>
	bool operator()(const T* position1,
					const T* position2,
					const T* scale,
					T* residuals) const
	{
		typedef Eigen::Matrix<T, 3, 1> Vector3;
		Eigen::Map<Vector3>{residuals} =
			translationObs.cast<T>() - scale[0] * (Eigen::Map<const Vector3>(position2) - Eigen::Map<const Vector3>(position1));
		return true;
	}

	static ceres::CostFunction* Create(const Eigen::Vector3d& translationObs)
	{
		return new ceres::AutoDiffCostFunction<BATAPairwiseDirectionCostFunctor, 3, 3, 3, 1>(
			new BATAPairwiseDirectionCostFunctor(translationObs));
	}

	const Eigen::Vector3d translationObs;
};

// Analytic version of the cost function.
// This removes the overhead of AutoDiff and ensures Ceres uses
// fixed-size memory blocks (3,1,3) instead of dynamic ones (d,d,d).
struct BATAPairwiseDirectionCostAnalytic : public ceres::SizedCostFunction<3, 3, 3, 1> {
    BATAPairwiseDirectionCostAnalytic(const Eigen::Vector3d& translationObs)
        : translationObs(translationObs) {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override {
        // Map parameters
        Eigen::Map<const Eigen::Vector3d> p1(parameters[0]); // Position 1
        Eigen::Map<const Eigen::Vector3d> p2(parameters[1]); // Position 2
        const double s = parameters[2][0]; // Scale

        // Compute Residual: r = t - s * (p2 - p1)
        Eigen::Vector3d diff = p2 - p1;
        Eigen::Map<Eigen::Vector3d> res(residuals);
        res = translationObs - s * diff;

        // Compute Jacobians if requested
        if (jacobians) {
			using Matrix3dRowMajor = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
            // Jacobian w.r.t Position 1 (3x3): dr/dp1 = s * I
            if (jacobians[0]) {
                Eigen::Map<Matrix3dRowMajor> J1(jacobians[0]);
                J1.setIdentity();
                J1 *= s;
            }
            // Jacobian w.r.t Position 2 (3x3): dr/dp2 = -s * I
            if (jacobians[1]) {
                Eigen::Map<Matrix3dRowMajor> J2(jacobians[1]);
                J2.setIdentity();
                J2 *= -s;
            }
            // Jacobian w.r.t Scale (3x1): dr/ds = -(p2 - p1)
            if (jacobians[2]) {
                Eigen::Map<Eigen::Vector3d> J3(jacobians[2]);
                J3 = -diff;
            }
        }
        return true;
    }

    const Eigen::Vector3d translationObs;
};

} // namespace
/*----------------------------------------------------------------*/


GlobalPositioner::GlobalPositioner(const GlobalPositionerOptions& optionsIn)
	: options(optionsIn)
{
	randomGenerator.seed(options.seed);
}
GlobalPositioner::~GlobalPositioner() = default;


bool GlobalPositioner::Solve(Scene& scene)
{
	if (scene.images.empty())
		return false;
	if (scene.pairs.empty() && options.constraintType != GlobalPositionerOptions::ONLY_POINTS)
		return false;
	if (scene.tracks.empty() && options.constraintType != GlobalPositionerOptions::ONLY_CAMERAS)
		return false;
	TD_TIMER_STARTD();

	// Setup the problem
	ceres::Problem::Options problem_options;
	problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem = std::make_unique<ceres::Problem>(problem_options);
	lossFunction = std::make_shared<ceres::HuberLoss>(options.thLossFunction);
	ceres::Solver::Options solverOptions;
	solverOptions.max_num_iterations = options.maxNumIterations;
	solverOptions.num_threads = options.numThreads;
	solverOptions.function_tolerance = options.functionTolerance;
	#ifndef _RELEASE
	solverOptions.minimizer_progress_to_stdout = true;
	#else
	solverOptions.minimizer_progress_to_stdout = false;
	#endif
	solverOptionsPtr = &solverOptions;

	// Allocate enough memory for the scales (very important to avoid reallocations)
	scales.clear();
	size_t numPtToCam = 0;
	for (const Track& track : scene.tracks) {
		if (track.observations.size() < options.minNumViewPerTrack)
			continue;
		for (const Observation& obs : track.observations)
			if (scene.images[obs.imageID].IsValid())
				++numPtToCam;
	}
	scales.reserve(scene.pairs.size() + numPtToCam);

	// Generate random positions for constrained images.
	// An image is considered constrained if it appears in at least one valid image pair;
	// or if it observes at least one valid track. However, we do not need to explicitly
	// collect these images here, as they were already marked, by initializing the pose,
	// during rotation averaging.
	unsigned numValidImages = 0;
	for (Image& image : scene.images) {
		if (image.IsValid()) {
			image.C = RandPoint3(randomGenerator, -100, 100);
			++numValidImages;
		}
	}

	// Add the camera to camera constraints to the problem
	unsigned numValidPairs = 0;
	if (options. constraintType != GlobalPositionerOptions::ONLY_POINTS)
		numValidPairs = AddCameraToCameraConstraints(scene);

	// Add the point to camera constraints to the problem
	unsigned numValidTracks = 0;
	if (options.constraintType != GlobalPositionerOptions::ONLY_CAMERAS)
		numValidTracks = AddPointToCameraConstraints(scene, numPtToCam);

	// Set the parameter groups and parameterize the variables
	ConfigureProblem(scene);

	ceres::Solver::Summary summary;
	ceres::Solve(solverOptions, problem.get(), &summary);
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 1) {
		VERBOSE("Summary: %s", summary.FullReport().c_str());
	} else {
		DEBUG("Summary: %s", summary.BriefReport().c_str());
	}
	#endif
	if (!summary.IsSolutionUsable()) {
		VERBOSE("error: bundle adjustment failed");
		return false;
	}
	DEBUG("Global positioner completed: %u images, %u pairs, %u tracks (%s)",
		numValidImages, numValidPairs, numValidTracks, TD_TIMER_GET_FMT().c_str());
	return true;
}

unsigned GlobalPositioner::AddCameraToCameraConstraints(Scene& scene)
{
	// Add constraints from relative poses between image pairs
	unsigned numValidPairs = 0;
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue; // skip invalid pairs
		ASSERT(scene.images[pair.ID1].HasCamera() && scene.images[pair.ID2].HasCamera());
		if (!scene.images[pair.ID1].IsValid() || !scene.images[pair.ID2].IsValid())
			continue; // skip pairs with unposed images (not initialized in rotation averaging)

		const IIndex imageId1 = pair.ID1;
		const IIndex imageId2 = pair.ID2;
		ASSERT(imageId1 < scene.images.size() && imageId2 < scene.images.size());

		Image& image1 = scene.images[imageId1];
		Image& image2 = scene.images[imageId2];
		ASSERT(image1.HasCamera() && image2.HasCamera());

		ASSERT(scales.capacity() > scales.size());
		double& scale = scales.emplace_back(1);

		// Convert relative pose translation to world coordinates
		// Rotate to world frame using image2's rotation (transpose for inverse)
		const Point3 translation = -(image2.R.t() * pair.relativePose->GetT());

		ceres::CostFunction* costFunction = new BATAPairwiseDirectionCostAnalytic(translation);

		// Optimize camera centers directly
		problem->AddResidualBlock(
			costFunction,
			lossFunction.get(),
			image1.C.ptr(), // camera center data
			image2.C.ptr(),
			&scale);

		problem->SetParameterLowerBound(&scale, 0, 1e-5);
		++numValidPairs;
	}
	return numValidPairs;
}

unsigned GlobalPositioner::AddPointToCameraConstraints(Scene& scene, size_t numPtToCam)
{
	// The number of camera-to-camera constraints coming from the relative poses
	const size_t numCamToCam = problem->NumResidualBlocks();
	double weightScalePt = 1.0;
	// Set the relative weight of the point to camera constraints based on
	// the number of camera to camera constraints.
	if (numCamToCam > 0 && options.constraintType == GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED) {
		weightScalePt = options.constraintReweightScale * static_cast<double>(numCamToCam) / static_cast<double>(numPtToCam);
	}

	if (lossFunctionCamUncalibrated == nullptr) {
		lossFunctionCamUncalibrated = std::make_shared<ceres::ScaledLoss>(
			lossFunction.get(), 0.5 * weightScalePt, ceres::DO_NOT_TAKE_OWNERSHIP);
	}

	if (options.constraintType == GlobalPositionerOptions::POINTS_AND_CAMERAS_BALANCED) {
		lossFunctionCamCalibrated = std::make_shared<ceres::ScaledLoss>(
			lossFunction.get(), weightScalePt, ceres::DO_NOT_TAKE_OWNERSHIP);
	} else {
		lossFunctionCamCalibrated = lossFunction;
	}

	// Add point to camera constraints
	unsigned numValidTracks = 0;
	for (Track& track : scene.tracks) {
		if (track.observations.size() < options.minNumViewPerTrack)
			continue;
		// Only set the points to be random if they are needed to be optimized
		if (options.optimizePoints && options.generateRandomPoints)
			track.position = RandPoint3(randomGenerator, -100, 100);
		// For each observation in the track add the point to camera correspondences.
		bool hasValidObservation = false;
		for (const Observation& obs : track.observations) {
			ASSERT(obs.imageID < scene.images.size());
			Image& image = scene.images[obs.imageID];
			if (!image.IsValid())
				continue;
			// Unproject and rotate to get normalized ray direction (undistorted) in world coordinates
			ASSERT(obs.featureID < image.keypoints.size());
			const cv::KeyPoint& kp = image.keypoints[obs.featureID];
			const Point3 translation = normalized(image.Ray(Cast<REAL>(kp.pt)));

			ASSERT(scales.capacity() > scales.size());
			double& scale = scales.emplace_back(1);
			if (!options.generateScales && track.IsInlier()) {
				// Initialize scale from existing triangulated position
				const Point3d trans_calc = track.position - image.C;
				scale = MAXF(1e-5, translation.dot(trans_calc) / normSq(trans_calc));
			}

			// Select loss function based on camera calibration
			// Down-weight uncalibrated cameras (TrustIntrinsics = false)
			ceres::LossFunction* lossFunction = image.HasCamera() && image.TrustIntrinsics()
					? lossFunctionCamCalibrated.get()
					: lossFunctionCamUncalibrated.get();

			ceres::CostFunction* costFunction = BATAPairwiseDirectionCostFunctor::Create(translation);

			problem->AddResidualBlock(
				costFunction,
				lossFunction,
				image.C.ptr(),
				track.position.ptr(),
				&scale);

			problem->SetParameterLowerBound(&scale, 0, 1e-5);
			hasValidObservation = true;
		}
		if (hasValidObservation)
			++numValidTracks;
	}
	return numValidTracks;
}

void GlobalPositioner::ConfigureProblem(Scene& scene)
{
	ceres::Solver::Options& solverOptions = *(static_cast<ceres::Solver::Options*>(solverOptionsPtr));

	// Add cameras and points to parameter groups:
	// Create a custom ordering for Schur-based problems.
	ceres::ParameterBlockOrdering* parameterOrdering = new ceres::ParameterBlockOrdering;
	// Add scale parameters to group 0 (large and independent)
	for (double& scale : scales)
		parameterOrdering->AddElementToGroup(&scale, 0);
	// Add point parameters to group 1
	int group = 1;
	if (!scene.tracks.empty()) {
		for (Track& track : scene.tracks)
			if (problem->HasParameterBlock(track.position.ptr()))
				parameterOrdering->AddElementToGroup(track.position.ptr(), 1);
		++group;
	}
	// Add camera centers to the next group
	for (Image& image : scene.images) {
		if (!image.IsValid())
			continue;
		if (problem->HasParameterBlock(image.C.ptr()))
			parameterOrdering->AddElementToGroup(image.C.ptr(), group);
	}
	solverOptions.linear_solver_ordering.reset(parameterOrdering);
	solverOptions.visibility_clustering_type = ceres::CANONICAL_VIEWS;

	// Parameterize the variables:
	// If do not optimize the positions, set the camera positions to be constant
	if (!options.optimizePositions) {
		for (Image& image : scene.images) {
			if (!image.IsValid())
				continue;
			if (problem->HasParameterBlock(image.C.ptr()))
				problem->SetParameterBlockConstant(image.C.ptr());
		}
	}
	// If do not optimize the points, set the track positions to be constant
	if (!options.optimizePoints) {
		for (Track& track : scene.tracks)
			if (problem->HasParameterBlock(track.position.ptr()))
				problem->SetParameterBlockConstant(track.position.ptr());
	}
	// If do not optimize the scales, set the scales to be constant
	if (!options.optimizeScales) {
		for (double& scale : scales)
			if (problem->HasParameterBlock(&scale))
				problem->SetParameterBlockConstant(&scale);
	} else {
		// Set the first scale to be constant to remove the gauge ambiguity.
		for (double& scale : scales) {
			if (problem->HasParameterBlock(&scale)) {
				problem->SetParameterBlockConstant(&scale);
				break;
			}
		}
	}

	// Set up the options for the solver
	if (!scene.tracks.empty()) {
		solverOptions.linear_solver_type = ceres::SPARSE_SCHUR;
		solverOptions.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
	} else {
		solverOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
		solverOptions.preconditioner_type = ceres::JACOBI;
	}

	// Configure GPU/CUDA acceleration if available and requested; applied after the solver
	// type is chosen so the sparse backend is only switched on a path cuDSS can handle
	#ifdef _USE_CUDA
	if (options.useGpu && scene.images.size() >= options.minNumImagesGpuSolver) {
		#if (CERES_VERSION_MAJOR >= 3 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2))
		// Offload dense direct solves to the GPU via cuSOLVER when available.
		if (ceres::IsDenseLinearAlgebraLibraryTypeAvailable(ceres::CUDA))
			solverOptions.dense_linear_algebra_library_type = ceres::CUDA;
		else
			VERBOSE("warning: GPU dense solver requested but Ceres was built without CUDA; using CPU direct solvers instead.");
		// cuDSS (CUDA_SPARSE) currently backs only SPARSE_NORMAL_CHOLESKY, so enable it only on
		// that path; the runtime check auto-activates it once the linked Ceres provides cuDSS.
		if (solverOptions.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY) {
			if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CUDA_SPARSE))
				solverOptions.sparse_linear_algebra_library_type = ceres::CUDA_SPARSE;
			else
				VERBOSE("warning: GPU sparse solver requested but Ceres was built without cuDSS; using CPU sparse solvers instead.");
		}
		#else
		VERBOSE("warning: GPU solver requested but Ceres (version < 2.2) was built without CUDA; using CPU solvers instead.");
		#endif
	}
	#endif // _USE_CUDA
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
