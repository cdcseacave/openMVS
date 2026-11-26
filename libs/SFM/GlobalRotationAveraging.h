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
class SFM_API ImagePair;
struct SFM_API ScalePair;
struct SFM_API PairsWeightingConfig;
class SFM_API SequenceNeighbours;

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

	// The median the pair weights are scaled to inside the estimator (> 0): the weights count only
	// relative to each other, so the result does not depend on their absolute scale. It sets where
	// the L1 step's fixed-iteration ADMM sits between a weighted least-squares fit (large values) and
	// an L1 fit (small values); the reconstructions are flat from 1 to 1000 on the captures measured
	// (docs/design/ContradictionCut.md).
	double weightMedian = 10.0;

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
	uint32_t fixedNodeId = NO_ID;
};
/*----------------------------------------------------------------*/

// The contradiction cut (FilterPairsByContradiction, below). The declarations down to SolveScaleComponents are what
// its stages share with the chain of an image sequence (SequenceChain.h).

// The state of a scene pair in the cut
enum class PairState : uint8_t {
	KEPT,   // kept
	LINK,   // kept, and a link of the sequence chain with a solved scale: the translation stage's verdict leaves it
	CUT,    // removed by the rotation or the translation stage: what the repair is offered
	VETOED  // removed by the sequence chain: never repaired, joins no spot, vouches for no pair
};
inline bool IsRemoved(PairState state) { return state == PairState::CUT || state == PairState::VETOED; }

// The baseline of every scene pair in the world frame, at its pose's own length: RMatrix(rotations[ID1]).t() *
// relativePose->C. Point3::INF for a pair without a pose or whose two images do not lie in one rotation component
// (componentOf: per image, NO_ID where not rotated): no stage after the rotations reads such a pair.
SFM_API std::vector<Point3> ComputeWorldBaselines(const Scene& scene, const std::vector<Point3>& rotations,
	const std::vector<uint32_t>& componentOf);
// What the stages ask of a pair's world baseline: is there one; has it a length; has it a direction (a length, and a
// mean ray angle of at least kNearDuplicateRayAngle: under it the direction is noise)
bool HasBaseline(const Point3& baseline);
bool HasBaselineLength(const Point3& baseline);
bool HasBaselineDirection(const ImagePair& pair, const Point3& baseline);

// One solved component of the pair-scale graph (SolveScaleComponents)
struct ScaleComponentSolve {
	std::vector<uint32_t> pairs;       // its scene pairs, ascending
	std::vector<REAL> scales;          // per scene pair: C_2 - C_1 = scale * baseline (valid for `pairs`)
	std::vector<Point3> centres;       // per image: its camera centre (valid where `placed`)
	std::vector<REAL> centreResiduals; // per pair of `pairs`: the distance of its scaled baseline from the centres' offset
	std::vector<bool> placed;          // per image: an image of one of `pairs`
};
// The scale + centres solve of the translation stage, which the chain of a sequence runs on its links: the components
// of the graph of the given pair-scale equations (ComputePairScales), largest first (by pairs, ties to the lower
// pair), and per component of at least 10 pairs the scales by RobustAverage (Geman-McClure) around
// GlobalScaleEstimator, then the centres from C_2 - C_1 = scale * baseline (baselines and weights: per scene pair) by
// GlobalTranslationEstimator reweighted towards the sum of norms (convex, so no start decides). Returns the solved
// components in that order (one whose scale or centres solve fails is left out); numComponents: all the components
// of the graph.
std::vector<ScaleComponentSolve> SolveScaleComponents(const Scene& scene, const std::vector<ScalePair>& equations,
	const std::vector<Point3>& baselines, const std::vector<float>& weights, unsigned& numComponents);

// What the chain of an image sequence did in the contradiction cut (FilterPairsByContradiction given the neighbours)
struct SFM_API SequenceFilterStats {
	unsigned numSegments{0};  // segments of the chain
	unsigned numLinks{0};     // links of the chain
	unsigned numJudged{0};    // pairs the veto judged against the chain
	unsigned numVetoed{0};    // of them, the pairs it removed
	unsigned numProtected{0}; // links the translation stage would have removed
};

// The contradiction cut: remove every pair whose relative rotation is more than the estimator's
// maxRelativeRotationAngle from the global rotations a robust averaging of the usable pairs
// gives. The usable pairs are those holding a relative pose and a composite weight above
// minPairWeight (the ones the reconstruction reads); each is weighted by its composite weight
// times its triplet score (scores: per scene pair, TripletScores::unscored, negative, for an
// unscored pair, which weighs 1), and the averaging starts from the maximum spanning tree of those
// weights. A component of the usable graph is averaged on its own (the largest first); every scene
// pair with a pose whose two images one solve reached is judged, usable or not. A pair the graph
// holds alone, a bridge, is fitted exactly and never removed.
//  - maxDirectionResidual: after the rotation stage, a translation stage on the pairs it kept: the
//    scales of the pairs (StarInitializer's equations, solved robustly), the camera centres they
//    place (towards the sum of norms), and every pair whose baseline direction is more than this
//    many degrees from the centres removed; a pair under kNearDuplicateRayAngle of ray angle is
//    never judged. In [0,180]; 0 skips the translation stage, the rotation stage alone.
//  - repair: repair instead of remove (needs maxDirectionResidual above 0): a removed pair comes back
//    when, with its rotation fixed to the averaged one, its matches agree on a baseline direction that
//    the camera centres of the translation stage confirm and a neighbouring spot vouches for (the
//    guard is in GlobalRotationAveraging.cpp, RepairContradictedPairs); it then carries that relative
//    pose and its agreeing matches, and is not counted as removed.
//  - neighbours (nullptr: off; acts only with maxDirectionResidual above 0): the images are frames of a
//    sequence. Between the two stages the chain of the kept neighbour pairs is solved and the pairs that
//    contradict it are removed (ApplySequenceChain, SequenceChain.h); they are never offered to the
//    repair. The translation stage then removes no link of the chain.
//  - stats (optional): what the chain did; all zero without it.
// The pair weights are recomputed when anything was removed or repaired. Returns the number of removed
// pairs, the vetoed ones among them.
unsigned SFM_API FilterPairsByContradiction(Scene& scene, const std::vector<float>& scores,
	float minPairWeight, float maxDirectionResidual, const PairsWeightingConfig& weightingCfg, bool repair = false,
	const SequenceNeighbours* neighbours = nullptr, SequenceFilterStats* stats = nullptr);

} // namespace SFM

#endif // _SFM_GLOBAL_ROTATION_AVERAGING_H_
