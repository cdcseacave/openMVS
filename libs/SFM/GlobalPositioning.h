/*
 * GlobalPositioning.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_GLOBAL_POSITIONING_H_
#define _SFM_GLOBAL_POSITIONING_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace ceres {
	class Problem;
	class LossFunction;
} // namespace ceres

namespace SFM {

// Forward declarations
class SFM_API Scene;
class SFM_API Image;
class SFM_API ImagePair;
class SFM_API Track;

// Options struct for Global Positioning
struct SFM_API GlobalPositionerOptions
{
	// ONLY_POINTS is recommended
	enum ConstraintType {
		// only include camera to point constraints
		ONLY_POINTS,
		// only include camera to camera constraints
		ONLY_CAMERAS,
		// the points and cameras are reweighted to have similar total contribution
		POINTS_AND_CAMERAS_BALANCED,
		// treat each contribution from camera to point and camera to camera equally
		POINTS_AND_CAMERAS,
	};

	// Threshold for the loss function (difference in vectors)
	double thLossFunction = 1e-1;

	// Options for the solver
	int numThreads = 1;
	int maxNumIterations = 200;
	double functionTolerance = 1e-5;

	// GPU/CUDA options
	bool useGpu = true;
	unsigned minNumImagesGpuSolver = 50;


	// Constrain the minimum number of views per track
	unsigned minNumViewPerTrack = 3;

	// Random seed
	unsigned seed = 123;

	// Type of global positioning
	ConstraintType constraintType = ONLY_POINTS;
	double constraintReweightScale = 1.0; // only relevant for POINTS_AND_CAMERAS_BALANCED

	GlobalPositionerOptions() : numThreads(std::thread::hardware_concurrency()) {}
};

class SFM_API GlobalPositioner
{
public:
	GlobalPositioner(const GlobalPositionerOptions& optionsIn);
	~GlobalPositioner();

	// Returns true if the optimization was successfull
	bool Solve(Scene& scene);

	GlobalPositionerOptions& GetOptions() { return options; }

protected:
	// Creates camera to camera constraints from relative translations
	unsigned AddCameraToCameraConstraints(Scene& scene);

	// Add tracks to the problem
	unsigned AddPointToCameraConstraints(Scene& scene, size_t numPtToCam);

	// Set the parameter groups and parameterize the variables
	void ConfigureProblem(Scene& scene);

protected:
	GlobalPositionerOptions options;

	std::mt19937 randomGenerator;
	std::unique_ptr<ceres::Problem> problem;
	void* solverOptionsPtr; // ceres::Solver::Options*

	// Loss functions for reweighted terms
	std::shared_ptr<ceres::LossFunction> lossFunction;
	std::shared_ptr<ceres::LossFunction> lossFunctionCamUncalibrated;
	std::shared_ptr<ceres::LossFunction> lossFunctionCamCalibrated;

	// Auxiliary scale variables.
	std::vector<double> scales;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBAL_POSITIONING_H_
