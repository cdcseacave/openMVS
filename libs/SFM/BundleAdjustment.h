/*
 * BundleAdjustment.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_BUNDLEADJUSTMENT_H_
#define _SFM_BUNDLEADJUSTMENT_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// forward declarations to avoid circular includes
class SFM_API Pose3D;
class SFM_API Scene;

/**
 * @brief Configuration for bundle adjustment
 */
struct SFM_API BAConfig
{
	// Pose and point refinement
	bool refinePosesRotation = true;  // Optimize camera rotation (part of pose)
	bool refinePosesPosition = true;  // Optimize camera position (part of pose)
	bool refinePoints = true;         // Optimize 3D points

	// Intrinsic refinement (global BA only, not local BA)
	bool refineFocalLength = false;            // Refine fx, fy
	bool refineFocalLengthAspectRatio = false; // Refine fx, fy while keeping aspect ratio constant
	bool refinePrincipalPoint = false;         // Refine cx, cy
	bool refineRadialDistortion123 = false;    // Refine k1, k2, k3
	bool refineTangentialDistortion = false;   // Refine p1, p2
	bool refineRadialDistortion456 = false;    // Refine k4, k5, k6

	// GPS position constraints (weight = 0 disables)
	double gpsPositionWeight = 0.0;     // Horizontal GPS constraint weight
	double gpsPositionWeightZ = 0.0;    // Vertical GPS constraint weight
	double gpsWeightScaleFactor = 1.0;  // Manual scaling override for GPS weights

	// Angular reprojection error with keypoint confidence weighting
	bool useKeypointConfidence = false; // Weight observations by keypoint response and size
	float minKeypointResponse = 0.001f; // Minimum keypoint response to include in BA (0 = include all)

	// Solver parameters
	unsigned maxIterations = 100;    // Maximum solver iterations
	float robustThreshold = 2.f;     // Huber loss threshold (pixels, 0 = disabled)
	unsigned numThreads = 0;         // Number of threads (0 = auto)
	double functionTolerance = 1e-6; // Convergence tolerance

	// Helper: Enable all intrinsic refinement flags
	void RefineMainIntrinsics() {
		refineFocalLength = true;
		refineRadialDistortion123 = true;
	}
	void RefineExtendedIntrinsics() {
		RefineMainIntrinsics();
		refinePrincipalPoint = true;
		refineTangentialDistortion = true;
	}
	void RefineAllIntrinsics() {
		RefineExtendedIntrinsics();
		refineFocalLengthAspectRatio = true;
		refineRadialDistortion456 = true;
	}

	// Helper: Check if any intrinsic refinement is enabled
	bool IsRefiningIntrinsics() const {
		return refineFocalLength || refinePrincipalPoint ||
		       refineRadialDistortion123 || refineTangentialDistortion ||
		       refineRadialDistortion456;
	}

	// Helper: Check if any pose component is being refined
	bool IsRefiningPoses() const {
		return refinePosesRotation || refinePosesPosition;
	}
};
/*----------------------------------------------------------------*/

/**
 * @brief Non-linear bundle adjustment using Ceres Solver
 *
 * Refines camera intrinsics, poses, and 3D points by minimizing
 * reprojection error across all observations.
 */
class SFM_API BundleAdjustment
{
public:
	/**
	 * @brief Perform global bundle adjustment
	 * @param scene Scene with cameras, poses, and points
	 * @param config BA configuration
	 * @return true if optimization successful
	 */
	static bool Adjust(Scene& scene, const BAConfig& config);

	/**
	 * @brief Perform local bundle adjustment
	 *
	 * Optimizes subset of views and points observed by those views.
	 * Other parameters held constant.
	 *
	 * @param scene Scene with reconstruction
	 * @param viewIDs Views to optimize (+ observed points)
	 * @param fixedViewIDs Views to keep fixed
	 * @param config BA configuration
	 * @return true if optimization successful
	 */
	static bool AdjustLocal(
		Scene& scene,
		const IIndexArr& viewIDs,
		const IIndexArr& fixedViewIDs,
		const BAConfig& config);
};
/*----------------------------------------------------------------*/


// Convert OpenMVS pose to/from Ceres quaternion parameterization
// params[7] = { qw, qx, qy, qz, Cx, Cy, Cz }
SFM_API void Pose3DToQuaternionAndCenter(const Pose3D& pose, double* params);
SFM_API void QuaternionAndCenterToPose3D(const double* params, Pose3D& pose);

// Convert OpenMVS pose to/from Ceres angle-axis parameterization
// params[6] = { ax, ay, az, Cx, Cy, Cz }
SFM_API void Pose3DToAngleAxisAndCenter(const Pose3D& pose, double* params);
SFM_API void AngleAxisAndCenterToPose3D(const double* params, Pose3D& pose);
/*----------------------------------------------------------------*/


// Test PinholeReprojectionErrorAnalytic Jacobians using Auto-diff
SFM_API bool PinholeReprojectionJacobianTest();
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_BUNDLEADJUSTMENT_H_
