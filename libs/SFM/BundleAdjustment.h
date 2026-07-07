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

namespace ceres { class Problem; }

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
 * @brief Per-image pose uncertainty estimated from the bundle-adjustment covariance
 *
 * Per-axis variances read off the diagonal of the pose's 3x3 marginal-covariance blocks:
 * rotation about the camera x/y/z axes (rad^2, SE(3) tangent space) and camera-center
 * position along the world X/Y/Z axes (world-units^2; East/North/Up on a geo-aligned
 * scene). Lower = better localized; the reference (gauge) image is exactly 0; negative
 * means not computed (unregistered image, or pose block absent/partially fixed).
 */
struct SFM_API PoseUncertainty
{
	Point3f rotVar; // rotation variance about the camera x/y/z axes (rad^2)
	Point3f posVar; // camera-center variance along the world X/Y/Z axes (world-units^2)

	bool IsValid() const { return posVar.x >= 0.f; }

	// Collapse a per-axis variance triplet into a single scalar trust value: the largest
	// per-axis variance (conservative and direction-independent; it lower-bounds the top
	// eigenvalue of the full 3x3 covariance block).
	static float MaxVariance(const Point3f& var) { return MAXF(MAXF(var.x, var.y), var.z); }
	float MaxRotationVariance() const { return MaxVariance(rotVar); }
	float MaxPositionVariance() const { return MaxVariance(posVar); }
};
typedef CLISTDEF0IDX(PoseUncertainty, IIndex) PoseUncertaintyArr;
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
	BundleAdjustment(Scene& scene, const BAConfig& config);
	~BundleAdjustment();

	/**
	 * @brief Perform global bundle adjustment
	 *
	 * On success the solved Ceres problem is kept alive by this instance, so
	 * ComputePoseUncertainty() can be called afterwards.
	 * @return true if optimization successful
	 */
	bool Adjust();

	/**
	 * @brief Perform local bundle adjustment
	 *
	 * Optimizes the views in viewIDs together with the 3D points they observe; the views in
	 * fixedViewIDs contribute observations but stay constant, and every other view and point
	 * is held out of the problem. Intrinsics are never refined in local BA.
	 * On success the solved Ceres problem is kept alive by this instance, so
	 * ComputePoseUncertainty() can be called afterwards.
	 * @param viewIDs Views to optimize (+ the points they observe)
	 * @param fixedViewIDs Views kept fixed (contribute observations only)
	 * @return true if optimization successful
	 */
	bool AdjustLocal(
		const IIndexArr& viewIDs,
		const IIndexArr& fixedViewIDs);

	/**
	 * @brief Estimate per-image pose uncertainty from the last Adjust() run
	 *
	 * Marginal pose covariance of the Gauss-Newton Hessian at the solution: the 3D points
	 * are eliminated by their block-diagonal Schur complement and the per-pose blocks are
	 * read off the sparse selected inverse of the reduced system (intrinsics held fixed,
	 * so the result is conditioned on them — adequate as a relative trust signal).
	 * @return one entry per image (invalid where not computed), or empty on failure
	 */
	PoseUncertaintyArr ComputePoseUncertainty();

	/**
	 * @brief One-shot global bundle adjustment
	 * @param scene Scene with cameras, poses, and points
	 * @param config BA configuration
	 * @return true if optimization successful
	 */
	static bool Adjust(Scene& scene, const BAConfig& config) {
		return BundleAdjustment(scene, config).Adjust();
	}

	/**
	 * @brief One-shot local bundle adjustment
	 * @param scene Scene with reconstruction
	 * @param viewIDs Views to optimize (+ the points they observe)
	 * @param fixedViewIDs Views kept fixed (contribute observations only)
	 * @param config BA configuration
	 * @return true if optimization successful
	 */
	static bool AdjustLocal(
		Scene& scene,
		const IIndexArr& viewIDs,
		const IIndexArr& fixedViewIDs,
		const BAConfig& config) {
		return BundleAdjustment(scene, config).AdjustLocal(viewIDs, fixedViewIDs);
	}

private:
	Scene& scene;
	const BAConfig config;
	std::unique_ptr<ceres::Problem> problem; // solved problem, alive after a successful Adjust()
	std::vector<double> poseParams;          // 7 doubles per image [qw,qx,qy,qz,Cx,Cy,Cz], indexed by image ID
	UnsignedArr numReprojResidualsPerImage;  // per-image reprojection-residual count (gauge/datum selection)
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
