/*
 * GlobalRotationAveraging.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 *
 * Global rotation averaging implementation adapted from GLOMAP
 * Reference: "GLOMAP: Global Structure-from-Motion Revisited" (arXiv:2407.20219)
 */

#ifndef _SFM_GLOBAL_ROTATION_AVERAGING_H_
#define _SFM_GLOBAL_ROTATION_AVERAGING_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;
class SFM_API Image;

// Pairwise relative rotation
struct SFM_API RotationPair
{
	uint32_t idxA;            // First index
	uint32_t idxB;            // Second index (first always smaller than second)
	Matrix3x3 relativeRotation; // R_B * R_A^T (rotation from A to B)
	float weight;             // confidence weight (e.g., number of inliers)

	RotationPair() : idxA(NO_ID), idxB(NO_ID), relativeRotation(Matrix3x3::IDENTITY), weight(0.f) {}
	RotationPair(uint32_t a, uint32_t b, const Matrix3x3& R, float w)
		: idxA(a), idxB(b), relativeRotation(R), weight(w) {}
};

struct SFM_API GlobalRotationEstimatorOptions
{
	// Maximum number of times to run L1 minimization.
	int maxNumL1Iterations = 5;

	// Average step size threshold to terminate the L1 minimization
	double l1StepConvergenceThreshold = 0.001;

	// The number of iterative reweighted least squares iterations to perform.
	int maxNumIrlsIterations = 100;

	// Average step size threshold to terminate the IRLS minimization
	double irlsStepConvergenceThreshold = 0.001;

	// This is the point where the Huber-like cost function switches from L1 to L2.
	double irlsLossParameterSigma = 5.0; // in degrees

	enum WeightType {
		// For Geman-McClure weight, refer to "Efficient and robust
		// large-scale rotation averaging" (Chatterjee et al., 2013)
		GEMAN_MCCLURE,
		// For Half Norm, refer to "Robust Relative Rotation Averaging"
		// (Chatterjee et al., 2017)
		HALF_NORM,
	} weightType = GEMAN_MCCLURE;

	// Flag to use maximum spanning tree for initialization
	bool skipInitialization = false;

	// Flag to use weighting for rotation averaging;
	// if false, all relative rotations are weighted equally during optimization, while
	// weighted by number of inlier matches only during maximum spanning tree initialization
	bool useWeight = true;

	// Maximum angle (in degrees) between relative rotation and computed rotation from global estimates (0 = disabled)
	double maxRelativeRotationAngle = 12.0;
};

class SFM_API GlobalRotationEstimator
{
public:
	explicit GlobalRotationEstimator(const GlobalRotationEstimatorOptions& optionsIn) :
    options(optionsIn) {}

	// Estimates the global orientations of all images based on relative poses.
	//  - pNumFilteredPairs: if not NULL, returns the number of filtered pairs
	// Returns true on successful estimation and false otherwise.
	bool EstimateRotations(Scene& scene, unsigned* pNumFilteredPairs = NULL);

	// Estimate global rotations from pairwise relative rotations
	//  - pairwiseRotations: Vector of pairwise rotation constraints
	//  - numNodes: Total number of nodes (ex. number of images)
	//  - globalRotations: Output vector of global rotations in angle-axis format (one per node);
	//    either empty or pre-filled with initial estimates (which skips MST initialization)
	//    INF value indicates invalid/unestimated rotation
	// Returns true if estimation successful
	bool EstimateRotations(
		const std::vector<RotationPair>& pairwiseRotations,
		const uint32_t numNodes,
		std::vector<Point3>& globalRotations);

	// The one maximum spanning tree of a set of relative rotations, and the global rotations it
	// carries out from its best connected node at the identity: the initialization this estimator
	// starts from, exposed so a caller that wants to see the tree, or to keep one pair out of it,
	// reads the same tree this does.
	//  - excluded: a node pair the tree may not run between (NO_ID for none)
	//  - rotations: out, one per node, INF for every node outside the largest component the tree
	//    reaches; left untouched when nothing could be initialized
	//  - treeEdges: if not NULL, out: the node pairs that component's tree is built from
	// Returns the root the rotations are gauged at, NO_ID when nothing could be initialized
	static uint32_t RotationsFromMaximumSpanningTree(
		uint32_t numNodes,
		const std::vector<RotationPair>& pairwiseRotations,
		const std::pair<uint32_t, uint32_t>& excluded,
		std::vector<Point3>& rotations,
		std::vector<std::pair<uint32_t, uint32_t>>* treeEdges = NULL);

	// Filter relative rotations that are inconsistent with current global estimates
	//  - maxRelativeAngle: maximum allowed angle (degrees) between stored and computed relative rotation
	// Returns the number of filtered pairs
	static unsigned FilterRelativeRotations(Scene& scene, REAL maxRelativeAngle = 10);
	static unsigned FilterRelativeRotations(const std::vector<Point3>& globalRotations, std::vector<RotationPair>& pairwiseRotations, REAL maxRelativeAngle = 10);

protected:
	// Initialize the rotation from the maximum spanning tree
	// Number of inliers serve as weights
	void InitializeFromMaximumSpanningTree(uint32_t numNodes, const std::vector<RotationPair>& pairwiseRotations);

	// Sets up the sparse linear system such that dR_ij = dR_j - dR_i. This is the
	// first-order approximation of the angle-axis rotations. This should only be
	// called once.
	bool SetupLinearSystem(const std::vector<RotationPair>& pairwiseRotations);

	// Performs the L1 robust loss minimization.
	bool SolveL1Regression(const std::vector<RotationPair>& pairwiseRotations);

	// Performs the iteratively reweighted least squares.
	bool SolveIRLS(const std::vector<RotationPair>& pairwiseRotations);

	// Updates the global rotations based on the current rotation change.
	void UpdateGlobalRotations();

	// Computes the relative rotation (tangent space) residuals based on the
	// current global orientation estimates.
	void ComputeResiduals(const std::vector<RotationPair>& pairwiseRotations);

	// Computes the average size of the most recent step of the algorithm.
	double ComputeAverageStepSize() const;

private:
	// Options for the solver.
	const GlobalRotationEstimatorOptions& options;

	// The sparse matrix used to maintain the linear system. This is matrix A in Ax = b.
	Eigen::SparseMatrix<double> sparseMatrix;

	// x in the linear system Ax = b.
	Eigen::VectorXd tangentSpaceStep;

	// b in the linear system Ax = b.
	Eigen::VectorXd tangentSpaceResidual;

	// The weights for the edges
	Eigen::ArrayXd weights;

	// Rotation estimates in angle-axis representation
	Point3dArr estimatedRotations;

	// Variables for intermediate results
	std::unordered_map<uint32_t, uint32_t> nodeIdToIdx; // map node ID to the position in the rotation estimates vector
	std::unordered_map<uint32_t, uint32_t> pairIdToInfo; // map valid pair ID to the position of relative pose in the residual vector

	// The fixed node id. This is used to remove the gauge freedom.
	uint32_t fixedNodeId;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBAL_ROTATION_AVERAGING_H_
