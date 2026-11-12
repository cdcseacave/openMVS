////////////////////////////////////////////////////////////////////
// ViewGraphCalibrator.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_VIEWGRAPHCALIBRATOR_H_
#define _SFM_VIEWGRAPHCALIBRATOR_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace ceres {
	class Problem;
	class LossFunction;
} // namespace ceres

namespace SFM {

// Forward declaration
class SFM_API Scene;

/**
 * @brief Configuration for view graph calibration
 * 
 * Estimates camera focal lengths from the entire graph of image pairs
 * and their fundamental matrices using global optimization.
 */
struct SFM_API ViewGraphCalibratorConfig
{
	// Focal length ratio bounds (reject if estimated/prior is outside this range)
	double minFocalRatio = 0.1;   // Minimum allowed focal_estimated / focal_prior
	double maxFocalRatio = 10.0;  // Maximum allowed focal_estimated / focal_prior
	bool trustIntrinsics = true;  // If true, cameras with known intrinsics are not modified

	// Image pair filtering
	float maxTwoViewError = 2.f;  // Two-view error threshold (reject pairs with residual above this)
	float minPairWeight = 0.9f;   // Minimum composite weight for a pair to give a focal residual (docs/design/PairWeight.md)

	// Solver options
	double lossThreshold = 1e-1;  // Loss function threshold for robust estimation
	unsigned maxIterations = 100; // Maximum number of solver iterations
	unsigned numThreads = 0;      // 0 = auto-detect
};
/*----------------------------------------------------------------*/


/**
 * @brief View graph calibrator for estimating camera focal lengths
 * 
 * Uses the Fetzer focal length estimation method to refine camera intrinsics
 * from fundamental matrices across all image pairs. This provides more robust
 * estimates than per-pair or triplet-based methods by leveraging the entire
 * connectivity graph.
 * 
 * Reference: Fetzer et al. "Direct Focal Length Calibration from Two Views"
 */
class SFM_API ViewGraphCalibrator
{
public:
	ViewGraphCalibrator(const ViewGraphCalibratorConfig& config = ViewGraphCalibratorConfig());
	~ViewGraphCalibrator();

	/**
	 * @brief Calibrate cameras using view graph optimization
	 * @param scene Scene with images, cameras, and image pairs (with F matrices)
	 * @return true if calibration succeeded
	 * 
	 * Optimizes focal lengths for all pinhole cameras in the scene that don't
	 * have trustIntrinsics set. Updates camera focal lengths in-place.
	 * Also marks invalid pairs (high residual) for potential exclusion.
	 */
	bool Solve(Scene& scene);

	// Get set of cameras that were updated
	const std::unordered_set<CameraPtr>& GetUpdatedCameras() const {
		return updatedCameras_;
	}

private:
	// Reset the optimization problem
	void Reset(const Scene& scene);

	// Add image pairs to the optimization problem
	void AddImagePairsToProblem(const Scene& scene);

	// Parameterize cameras (set constant if trustIntrinsics)
	unsigned ParameterizeCameras(const Scene& scene);

	// Copy optimized results back to cameras
	unsigned CopyBackResults(Scene& scene);

	// Filter invalid image pairs based on residuals
	unsigned FilterImagePairs(Scene& scene) const;

	ViewGraphCalibratorConfig config_;
	std::unique_ptr<ceres::Problem> problem_;
	std::unordered_map<CameraPtr, double> focals_;  // Maps camera ptr -> focal length parameter
	std::shared_ptr<ceres::LossFunction> lossFunction_;
	std::unordered_set<CameraPtr> updatedCameras_;  // Cameras that were updated
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_VIEWGRAPHCALIBRATOR_H_
