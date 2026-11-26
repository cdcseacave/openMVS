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

// What the chain of an image sequence did in the contradiction cut (FilterPairsByContradiction given the neighbours)
struct SFM_API SequenceFilterStats {
	unsigned numSegments{0};  // segments of the chain
	unsigned numLinks{0};     // chain links (of positive scale)
	unsigned numJudged{0};    // pairs the veto judged against the chain
	unsigned numVetoed{0};    // of them, the pairs it removed
	unsigned numProtected{0}; // chain links the translation stage would have removed
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
//  - neighbours (needs maxDirectionResidual above 0; nullptr: none of this): the images are frames of a sequence
//    and the chain of its verified neighbour pairs is trusted (SolveSequenceChain below, on the pairs the rotation
//    stage kept, at the covisibility bar minLinkCovisibility; a link joins two images of one component of the
//    averaging, whose rotations share a frame). Before the translation stage, the veto: a pair that is no link,
//    with a pose and a world baseline of some length, whose two images one segment of the chain places and that
//    shares a pair-scale equation (ComputePairScales) with a link of that segment, is judged, a pair under
//    kNearDuplicateRayAngle of ray angle like any other (the veto reads a pair's length, which needs no baseline
//    direction). Its scale is the median, over those equations, of the link's scale carried
//    through the equation; the scale times its baseline is its offset in the segment's frame, and it is removed
//    when ChainVetoes says so: far from where the chain puts its two images and short against their distance, the
//    look-alike of a repetitive place, which claims that two distant frames stand close. A vetoed pair is not
//    offered to the repair. Then the protection: the translation stage solves as it does without the chain, but
//    removes no link (a pair of positive SequenceChain::linkScale).
//  - stats: what the chain did; all zero without it.
// The pair weights are recomputed when anything was removed or repaired. Returns the number of removed
// pairs, the vetoed ones among them.
unsigned SFM_API FilterPairsByContradiction(Scene& scene, const std::vector<float>& scores,
	float minPairWeight, float maxDirectionResidual, const PairsWeightingConfig& weightingCfg, bool repair = false,
	const SequenceNeighbours* neighbours = nullptr, float minLinkCovisibility = 0.f, SequenceFilterStats* stats = nullptr);

// The rule of the chain's veto, every length in the frame of the pair's segment: discrepancy is the distance between
// the pair's offset and the chain's (the offset of the centres of its two images), pairLength the length of the pair's
// baseline, chainDistance the distance of the two centres, localStep the segment's median step and path the length of
// the chain between the two images. The pair is vetoed when it is off the chain by more than 8 local steps and more
// than 0.2 of the path (the chain drifts with its length, so a loop closure off by a share of the path is no
// contradiction), and shorter than a third of the chain distance (a look-alike claims that its images stand close; a
// pair as long as the chain says differs from it in direction, which is the translation stage's to judge).
SFM_API bool ChainVetoes(REAL discrepancy, REAL pairLength, REAL chainDistance, REAL localStep, REAL path);

// The chain of an image sequence: the camera centres along the sequence as the verified neighbour pairs alone place
// them, segment by segment. The vectors are indexed by image (segmentOf, centres, pathLength), by scene pair
// (linkScale) and by segment (localStep).
struct SFM_API SequenceChain {
	std::vector<uint32_t> segmentOf; // per image: its chain segment, NO_ID where the chain placed nothing
	std::vector<Point3> centres;     // per image: its centre in its segment's frame (valid where segmentOf != NO_ID)
	// per image: the path along its segment from the segment's first image, through the images its links place, in
	// order; an image placed by standing still has the path of the image it stands at
	std::vector<REAL> pathLength;
	std::vector<REAL> localStep;     // per segment: the median distance between adjacent images its links place (0 for a segment of one image)
	// per scene pair: a chain link's solved scale, the multiplier of its relative pose's baseline as the translation
	// stage uses a pair's scale, so the link's offset in the segment's frame is its world baseline
	// (RMatrix(rotations[ID1]).t() * relativePose->C, at the pose's own length) times this scale; 0 for every other
	// pair. A positive scale means both images of the pair lie in one segment and the scale is in that segment's
	// frame: a neighbour pair joining two segments, or solved by a component other than its images' segment, has 0
	std::vector<REAL> linkScale;
	unsigned numSegments{0}, numLinks{0};
	unsigned numStill{0}; // the images no link places, placed by standing still where a placed neighbour stands
};
// The chain of a sequence: the scale + centres solve of the translation stage on the chain links alone. A link is a
// scene pair with a relative pose whose images are neighbours, both rotated (rotations != Point3::INF), not `removed`,
// with a positive composite weight (the centres solve skips a weightless pair, which would place its images nowhere),
// a world baseline of some length, meanRayAngle >= kNearDuplicateRayAngle, and whose covisibility, when it carries
// one, is >= minCovisibility (0 disables that test). Weights: the composite weight. One segment per solved
// component of the links' pair-scale graph (components under minScaleComponent links: unplaced); an image two
// components place belongs to the larger one (a component whose every image a larger one holds is no segment);
// numLinks counts the links of positive scale. The segments are numbered in the order of their first image, so they
// follow the sequence.
// Then the frames standing still. A still pair is a pair that is no link only for having no baseline direction: its
// meanRayAngle is under kNearDuplicateRayAngle or its world baseline has no length, every other condition of a link
// holding (neighbours, a pose, both images rotated, not `removed`, a positive weight, the covisibility bar). An image
// no segment placed that a still pair joins to a placed image takes that image's segment, centre and path length,
// pass by pass until a pass places nothing; a pass reads the images placed before it, and of several placed images
// the nearest in position (image index) is taken, the lower one on a tie, so the order of the scene's pairs decides
// nothing. numStill counts those images. A still pair stays no link (linkScale 0, not in numLinks), the local step
// and the numbering of the segments are those of the links alone, and with no segment solved nothing is placed.
SFM_API SequenceChain SolveSequenceChain(const Scene& scene, const std::vector<Point3>& rotations,
	const SequenceNeighbours& neighbours, float minCovisibility, const std::vector<bool>* removed = nullptr);

} // namespace SFM

#endif // _SFM_GLOBAL_ROTATION_AVERAGING_H_
