/*
 * GlobalRotationAveraging.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 *
 * Global rotation averaging implementation adapted from GLOMAP
 * Reference: "GLOMAP: Global Structure-from-Motion Revisited" (arXiv:2407.20219)
 */

#include "Common.h"
#include "GlobalRotationAveraging.h"
#include "Scene.h"
#include "PairsWeighting.h"
#include "GlobalAlignment.h"
#include "GlobalScaleAveraging.h"
#include "GlobalTranslationAveraging.h"
#include "PoseLink.h"
#include "RobustAveraging.h"
#include "StarInitializer.h"
#include "../Math/DisjointSet.h"
#include "../Math/LeastAbsoluteDeviationSolver.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#ifdef _USE_SUITESPARSE
#include <Eigen/CholmodSupport>
#else
#include <Eigen/SparseCholesky>
#endif

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("GlbRotAg"));

namespace {

// Compute relative angle error with normalization
double RelAngleError(double angle12, double angle1, double angle2)
{
    double est = (angle2 - angle1) - angle12;
	while (est >= M_PI)
		est -= TWO_PI;
	while (est < -M_PI)
		est += TWO_PI;

	// Inject random noise if the angle is too close to the boundary to break
	// possible balance at local minima
	if (est > M_PI - 0.01 || est < -M_PI + 0.01) {
		const double noise = (rand() % 1000) / 1000.0 * 0.01;
		if (est < 0)
			est += noise;
		else
			est -= noise;
	}
	return est;
}

} // namespace
/*----------------------------------------------------------------*/


// Estimate rotations for all images in the scene
bool GlobalRotationEstimator::EstimateRotations(Scene& scene, unsigned* pNumFilteredPairs)
{
	TD_TIMER_STARTD();

	// Convert ImagePairs to RotationPairs
	std::vector<RotationPair> rotationPairs;
	rotationPairs.reserve(scene.pairs.size());
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		const float weight = options.useWeight ? pair.GetCompositeWeight() : (float)pair.GetNumWeightedInliers();
		rotationPairs.emplace_back(pair.ID1, pair.ID2, pair.relativePose->R, weight);
	}
	if (rotationPairs.empty()) {
		VERBOSE("error: no valid image pairs for rotation averaging");
		return false;
	}

	// Initialize rotations from scene if requested
	std::vector<Point3> globalRotations;
	if (options.skipInitialization) {
		globalRotations.reserve(scene.images.size());
		for (const Image& img : scene.images)
			globalRotations.emplace_back(img.IsValid() ? Point3(img.R.GetRotationAxisAngle()) : Point3::INF);
	}

	// Solve using common rotation averaging solver
	if (!EstimateRotations(rotationPairs, scene.images.size(), globalRotations))
		return false;

	// Convert results back to scene
	for (Image& image : scene.images) {
		if (globalRotations[image.ID] != Point3::INF) {
			image.R.SetRotationAxisAngle(globalRotations[image.ID]);
			if (!image.IsValid())
				image.C = Point3::ZERO; // validate position
		}
	}

	// Filter relative rotations if requested
	unsigned numFilteredPairs = 0;
	if (options.maxRelativeRotationAngle > 0)
		numFilteredPairs = FilterRelativeRotations(scene, options.maxRelativeRotationAngle);
	if (pNumFilteredPairs)
		*pNumFilteredPairs = numFilteredPairs;
	return true;
}

bool GlobalRotationEstimator::EstimateRotations(
	const std::vector<RotationPair>& pairwiseRotations,
	const uint32_t numNodes,
	std::vector<Point3>& globalRotations)
{
	TD_TIMER_STARTD();

	// Initialize rotations from input if provided or from maximum spanning tree
	fixedNodeId = NO_ID;
	if (!globalRotations.empty()) {
		ASSERT(globalRotations.size() == numNodes);
		estimatedRotations = globalRotations;
	} else
		InitializeFromMaximumSpanningTree(numNodes, pairwiseRotations);

	// Set up the linear system
	if (!SetupLinearSystem(pairwiseRotations))
		return false;

	// Solve the linear system for L1 norm optimization
	if (options.maxNumL1Iterations > 0 && !SolveL1Regression(pairwiseRotations))
		return false;

	// Solve the linear system for IRLS optimization
	if (options.maxNumIrlsIterations > 0 && !SolveIRLS(pairwiseRotations))
		return false;

	// Copy results to output
	globalRotations.assign(numNodes, Point3::INF);
	for (const auto& [nodeId, idx] : nodeIdToIdx)
		globalRotations[nodeId] = estimatedRotations[idx];

	DEBUG("Global rotation averaging completed: %u nodes, %u pairs (%s)",
	      (unsigned)nodeIdToIdx.size(), (unsigned)pairIdToInfo.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

void GlobalRotationEstimator::InitializeFromMaximumSpanningTree(uint32_t numNodes, const std::vector<RotationPair>& pairwiseRotations)
{
	std::vector<Point3> rotations;
	const uint32_t root = RotationsFromMaximumSpanningTree(
		numNodes, pairwiseRotations, std::make_pair(NO_ID, NO_ID), rotations, NULL);
	// nothing to initialize from leaves the estimates exactly as they were
	if (root == NO_ID)
		return;
	estimatedRotations = rotations;
	if (fixedNodeId == NO_ID)
		fixedNodeId = root;
}

uint32_t GlobalRotationEstimator::RotationsFromMaximumSpanningTree(
	uint32_t numNodes, const std::vector<RotationPair>& pairwiseRotations,
	const std::pair<uint32_t, uint32_t>& excluded,
	std::vector<Point3>& rotations,
	std::vector<std::pair<uint32_t, uint32_t>>* treeEdges)
{
	if (treeEdges != NULL)
		treeEdges->clear();

	// Build an undirected weighted graph with all nodes as vertices
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
		boost::no_property, boost::property<boost::edge_weight_t, double>> Graph;
	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<Graph>::edge_descriptor Edge;

	Graph g(numNodes);

	// Add weighted edges from the valid pairs, skipping the excluded node pair
	FOREACH(pairIdx, pairwiseRotations) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		if (pair.weight <= 0)
			continue;
		if (MINF(pair.idxA, pair.idxB) == MINF(excluded.first, excluded.second) &&
			MAXF(pair.idxA, pair.idxB) == MAXF(excluded.first, excluded.second))
			continue;
		// Maximum Spanning Tree is needed, but Kruskal finds Minimum, so negate weights
		boost::add_edge(pair.idxA, pair.idxB, -pair.weight, g);
	}

	// Extract the largest connected component of nodes
	std::vector<unsigned> component(num_vertices(g));
	const unsigned numComponents = boost::connected_components(g, &component[0]);
	if (numComponents == 0)
		return NO_ID;
	std::vector<unsigned> componentSize(numComponents, 0);
	FOREACH(i, component)
		++componentSize[component[i]];
	unsigned largestComponent = 0;
	for (unsigned i = 1; i < numComponents; ++i)
		if (componentSize[i] > componentSize[largestComponent])
			largestComponent = i;
	if (componentSize[largestComponent] < 2)
		return NO_ID;

	// Find the maximum spanning tree
	// Note: Kruskal finds the MST forest
	std::vector<Edge> mstEdges;
	boost::kruskal_minimum_spanning_tree(g, std::back_inserter(mstEdges));

	// Build the tree as an adjacency list for BFS traversal within the LCC
	std::vector<IIndexArr> adj(numNodes);
	for (const Edge& e : mstEdges) {
		const Vertex u = boost::source(e, g);
		const Vertex v = boost::target(e, g);
		if (component[u] != largestComponent)
			continue;
		adj[u].push_back((uint32_t)v);
		adj[v].push_back((uint32_t)u);
		if (treeEdges != NULL)
			treeEdges->emplace_back((uint32_t)u, (uint32_t)v);
	}

	// Find the root as the node with most connections in the MST
	uint32_t root = NO_ID;
	FOREACH(i, adj)
		if (root == NO_ID || adj[i].size() > adj[root].size())
			root = i;
	ASSERT(root != NO_ID);

	// Initialize rotation estimates
	rotations.assign(numNodes, Point3::INF);

	// Use the tree to initialize the global rotations, starting from the root as identity
	std::queue<uint32_t> q;
	std::vector<bool> visited(numNodes, false);
	q.push(root);
	visited[root] = true;
	rotations[root] = Point3::ZERO; // identity rotation in angle-axis
	while (!q.empty()) {
		const uint32_t curr = q.front();
		q.pop();
		for (const uint32_t child : adj[curr]) {
			if (visited[child])
				continue;
			visited[child] = true;
			// Find the pair; a caller is free to give it either way round, the direction being
			// read off the pair itself below
			const RotationPair* pPair = nullptr;
			for (const RotationPair& pair : pairwiseRotations) {
				if ((pair.idxA == curr && pair.idxB == child) ||
					(pair.idxA == child && pair.idxB == curr)) {
					pPair = &pair;
					break;
				}
			}
			ASSERT(pPair != nullptr);
			if (pPair == nullptr)
				continue;
			const Matrix3x3& relR = pPair->relativeRotation;
			RMatrix Rcurr(rotations[curr]);
			if (pPair->idxA == curr) {
				// R_child = R_rel * R_curr
				RMatrix Rchild = relR * Rcurr;
				rotations[child] = Rchild.GetRotationAxisAngle();
			} else {
				// R_child = R_rel^T * R_curr
				RMatrix Rchild = relR.t() * Rcurr;
				rotations[child] = Rchild.GetRotationAxisAngle();
			}
			q.push(child);
		}
	}

	DEBUG("Initialized rotations for %d nodes using MST (root: %u)", componentSize[largestComponent], root);
	return root;
}

// Set up the linear system for rotation averaging
bool GlobalRotationEstimator::SetupLinearSystem(const std::vector<RotationPair>& pairwiseRotations)
{
	// Clear all structures
	sparseMatrix.resize(0, 0);
	tangentSpaceStep.resize(0);
	tangentSpaceResidual.resize(0);
	weights.resize(0);
	nodeIdToIdx.clear();
	pairIdToInfo.clear();

	// Map nodes to degrees of freedom, only those with valid rotations,
	// initialize rotations and find best connected image
	const uint32_t numInitialNodes = (uint32_t)estimatedRotations.size();
	ASSERT(fixedNodeId == NO_ID ||
		(fixedNodeId < numInitialNodes && estimatedRotations[fixedNodeId] != Point3::INF),
		"fixed node is invalid for rotation averaging");
	Point3d fixedNodeRotation;
	for (uint32_t nodeId = 0; nodeId < numInitialNodes; ++nodeId) {
		const Point3d rotation = estimatedRotations[nodeId];
		if (rotation == Point3::INF)
			continue; // avoid zero-column in the linear system
		if (fixedNodeId == NO_ID)
			fixedNodeId = nodeId; // fix the first valid node if no fixed node specified
		if (nodeId == fixedNodeId) {
			fixedNodeRotation = rotation;
			continue; // store the fixed node rotation and skip it in the linear system
		}
		const uint32_t idx = (uint32_t)nodeIdToIdx.size();
		estimatedRotations[idx] = rotation;
		nodeIdToIdx.emplace(nodeId, idx);
	}
	if (nodeIdToIdx.empty()) {
		VERBOSE("error: no connected nodes for rotation averaging");
		return false;
	}
	ASSERT(fixedNodeId != NO_ID);
	const uint32_t numFreeNodes = (uint32_t)nodeIdToIdx.size();
	estimatedRotations[numFreeNodes] = fixedNodeRotation; // add fixed node at the end
	nodeIdToIdx.emplace(fixedNodeId, numFreeNodes);
	estimatedRotations.resize(nodeIdToIdx.size());

	// Prepare relative information from rotation pairs
	std::vector<Eigen::Triplet<double>> vecCoeffs;
	std::vector<double> vecWeights;
	vecCoeffs.reserve(pairwiseRotations.size() * 6);
	vecWeights.reserve(pairwiseRotations.size() * 3);
	unsigned currPos = 0;
	FOREACH(pairIdx, pairwiseRotations) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		if (pair.weight <= 0)
			continue; // skip invalid pairs
		// Check if both nodes are in the estimation set
		auto itA = nodeIdToIdx.find(pair.idxA);
		auto itB = nodeIdToIdx.find(pair.idxB);
		if (itA == nodeIdToIdx.end() || itB == nodeIdToIdx.end())
			continue;
		// Map pair to relative rotation
		pairIdToInfo[pairIdx] = currPos;
		// Get weight
		const double weight = options.useWeight ? (double)pair.weight : 1.0;
		// Set up linear system: R_rel = R_j * R_i^T => dR_rel = dR_j - dR_i
		const uint32_t idx1 = itA->second * 3;
		const uint32_t idx2 = itB->second * 3;
		for (int i = 0; i < 3; ++i) {
			if (pair.idxA != fixedNodeId)
				vecCoeffs.emplace_back(currPos + i, idx1 + i, -1.0);
			if (pair.idxB != fixedNodeId)
				vecCoeffs.emplace_back(currPos + i, idx2 + i, 1.0);
			vecWeights.push_back(weight);
		}
		currPos += 3;
	}

	// Build sparse matrix
	const unsigned numDof = numFreeNodes * 3; // 3 DOF per rotation (angle-axis), fixed node is included
	sparseMatrix.resize(currPos, numDof);
	sparseMatrix.setFromTriplets(vecCoeffs.begin(), vecCoeffs.end());

	// Set up weights
	weights = Eigen::Map<Eigen::ArrayXd>(vecWeights.data(), vecWeights.size());

	// Initialize solution vectors
	tangentSpaceStep.resize(numDof);
	tangentSpaceResidual.resize(currPos);
	return true;
}

// Solve using L1 regression with ADMM
bool GlobalRotationEstimator::SolveL1Regression(const std::vector<RotationPair>& pairwiseRotations)
{
	const Eigen::SparseMatrix<double> A = weights.matrix().asDiagonal() * sparseMatrix;
	LeastAbsoluteDeviationSolver::Options l1SolverOptions;
	l1SolverOptions.max_num_iterations = 10;
	#ifdef _USE_SUITESPARSE
	l1SolverOptions.solver_type = LeastAbsoluteDeviationSolver::Options::SolverType::SupernodalCholmodLLT;
	#else
	l1SolverOptions.solver_type = LeastAbsoluteDeviationSolver::Options::SolverType::SimplicialLLT;
	#endif
	LeastAbsoluteDeviationSolver l1Solver(l1SolverOptions, A);

	ComputeResiduals(pairwiseRotations);
	DEBUG_EXTRA("L1 regression: initial residual computed");

	double currNorm = 0;
	int iteration = 0;
	for (iteration = 0; iteration < options.maxNumL1Iterations; ) {
		++iteration;

		// Use the current residual as b (Ax - b)
		tangentSpaceStep.setZero();
		if (!l1Solver.Solve(weights.matrix().asDiagonal() * tangentSpaceResidual, &tangentSpaceStep)) {
			DEBUG("L1 solver failed");
			return false;
		}
		if (tangentSpaceStep.array().isNaN().any()) {
			DEBUG("L1 solver produced NaN");
			return false;
		}

		const double lastNorm = currNorm;
		currNorm = tangentSpaceStep.norm();
		UpdateGlobalRotations();
		ComputeResiduals(pairwiseRotations);

		// Check the residual. If it is small, stop
		const double avgStep = ComputeAverageStepSize();
		DEBUG_ULTIMATE("L1 ADMM: iteration %d, avg-step %.6g, residual %.6g",
			iteration, avgStep, (sparseMatrix * tangentSpaceStep - tangentSpaceResidual).array().abs().sum());
		if (avgStep < options.l1StepConvergenceThreshold || ABS(lastNorm - currNorm) < 1e-10) {
			DEBUG_EXTRA("L1 ADMM converged after %d iterations (avg-step %.6g, norm-diff %.6g)",
				iteration, avgStep, ABS(lastNorm - currNorm));
			break;
		}
	}

	DEBUG("L1 ADMM total iterations: %d", iteration);
	return true;
}

// Solve using iteratively reweighted least squares
bool GlobalRotationEstimator::SolveIRLS(const std::vector<RotationPair>& pairwiseRotations)
{
	#ifdef _USE_SUITESPARSE
	Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> llt;
	#else
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> llt;
	#endif
	llt.analyzePattern(sparseMatrix.transpose() * sparseMatrix);

	const double sigma = D2R(options.irlsLossParameterSigma);
	Eigen::ArrayXd weightsIrls(sparseMatrix.rows());

	ComputeResiduals(pairwiseRotations);

	int iteration = 0;
	for (iteration = 0; iteration < options.maxNumIrlsIterations; ) {
		++iteration;

		// Compute robust weights based on residuals
		for (const auto& [pairIdx, pos] : pairIdToInfo) {
			const double residualSq = tangentSpaceResidual.segment<3>(pos).squaredNorm();
			double weight;
			if (options.weightType == GlobalRotationEstimatorOptions::GEMAN_MCCLURE) {
				// Geman-McClure: w = σ² / (σ² + ε²)²
				const double sigmaSq = SQUARE(sigma);
				const double denomSq = SQUARE(sigmaSq + residualSq);
				weight = sigmaSq / denomSq;
			} else { // HALF_NORM
				// Half-Norm: w = 1 / sqrt(ε²) = 1 / |ε|
				weight = 1.0 / MAXF(SQRT(residualSq), sigma);
			}
			weightsIrls.segment<3>(pos) = weight * weights.segment<3>(pos);
		}

		// Form the system: A^T W A dx = A^T W b where W = diag(weights_irls)
		Eigen::SparseMatrix<double> ATWeight = sparseMatrix.transpose() * weightsIrls.matrix().asDiagonal();

		// Factorize and solve
		llt.factorize(ATWeight * sparseMatrix);
		if (llt.info() != Eigen::Success) {
			DEBUG("IRLS factorization failed at iteration %d", iteration);
			return false;
		}
		tangentSpaceStep = llt.solve(ATWeight * tangentSpaceResidual);
		if (tangentSpaceStep.array().isNaN().any()) {
			DEBUG("IRLS solver produced NaN at iteration %d", iteration);
			return false;
		}

		UpdateGlobalRotations();
		ComputeResiduals(pairwiseRotations);

		// Check convergence
		const double avgStep = ComputeAverageStepSize();
		DEBUG_ULTIMATE("IRLS: iteration %d, avg_step %.6g", iteration, avgStep);
		if (avgStep < options.irlsStepConvergenceThreshold) {
			DEBUG_EXTRA("IRLS converged after %d iterations (avg_step %.6g)", iteration, avgStep);
			break;
		}
	}

	DEBUG("IRLS total iterations: %d", iteration);
	return true;
}

// Update global rotations based on computed step
void GlobalRotationEstimator::UpdateGlobalRotations()
{
	ASSERT(estimatedRotations.size() >= 2);
	const uint32_t numFreeNodes = (uint32_t)estimatedRotations.size() - 1; // fixed node is skipped
	for (uint32_t idx = 0; idx < numFreeNodes; ++idx) {
		RMatrix currentR(estimatedRotations[idx]);
		Point3 aaUpdate(-tangentSpaceStep.segment<3>(idx * 3));
		RMatrix updateR(aaUpdate);
		RMatrix newR = currentR * updateR;
		estimatedRotations[idx] = newR.GetRotationAxisAngle();
	}
}

// Compute residuals for current rotation estimates
void GlobalRotationEstimator::ComputeResiduals(const std::vector<RotationPair>& pairwiseRotations)
{
	for (const auto& [pairIdx, pos] : pairIdToInfo) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		// Get current rotations in matrix form
		const RMatrix R1(estimatedRotations[nodeIdToIdx.at(pair.idxA)]);
		const RMatrix R2(estimatedRotations[nodeIdToIdx.at(pair.idxB)]);
		// Compute residual: -log(R2^T * R_rel * R1)
		// This is the error rotation in the tangent space
		RMatrix errorR = R2.t() * pair.relativeRotation * R1;
		Eigen::Vector3d residual = errorR.GetRotationAxisAngle();
		tangentSpaceResidual.segment<3>(pos) = -residual;
	}
}

// Compute average step size
double GlobalRotationEstimator::ComputeAverageStepSize() const
{
	ASSERT(estimatedRotations.size() >= 2);
	const uint32_t numFreeNodes = (uint32_t)estimatedRotations.size() - 1; // fixed node is skipped
	double totalUpdate = 0;
	for (uint32_t idx = 0; idx < numFreeNodes; ++idx)
		totalUpdate += tangentSpaceStep.segment<3>(idx * 3).norm();
	return totalUpdate / numFreeNodes;
}

// Filter relative rotations that are inconsistent with current global estimates
unsigned GlobalRotationEstimator::FilterRelativeRotations(Scene& scene, REAL maxRelativeAngle)
{
	const REAL minCosAngle = COS(D2R(maxRelativeAngle));
	unsigned numPairs = 0, numInvalidPairs = 0;
	float maxInvalidWeight = 0.f;
	for (ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		Image& image1 = scene.images[pair.ID1];
		Image& image2 = scene.images[pair.ID2];
		ASSERT(image1.HasCamera() && image2.HasCamera());
		if (!image1.IsValid() || !image2.IsValid())
			continue;
		// Compute relative rotation from current global estimates:
		// R_rel_calc = R2 * R1^T
		const Matrix3x3 relCalcR = image2.R * image1.R.t();
		// Get stored relative rotation
		const Matrix3x3& relStoredR = pair.relativePose->R;
		// Compute rotation difference: errorR = relStoredR * relCalcR^T
		const REAL cosAngle = ComputeAngle(relStoredR, relCalcR);
		if (cosAngle < minCosAngle) {
			// Invalidate the pair by setting weight to 0
			DEBUG_ULTIMATE("Filtered pair (% 4u, % 4u): %u/%u matches, relative rotation angle %.2f degrees, %.2f weight",
				pair.ID1, pair.ID2, pair.GetNumFilteredInliers(), pair.GetNumMatches(), R2D(ACOS(cosAngle)), pair.GetCompositeWeight());
			if (pair.GetCompositeWeight() > maxInvalidWeight)
				maxInvalidWeight = pair.GetCompositeWeight();
			pair.InvalidateWeight();
			++numInvalidPairs;
		}
		++numPairs;
	}
	DEBUG("Filtered %u/%u relative rotations with angle > %.2f degrees (max weight %.2f)",
		numInvalidPairs, numPairs, maxRelativeAngle, maxInvalidWeight);
	return numInvalidPairs;
}
unsigned GlobalRotationEstimator::FilterRelativeRotations(const std::vector<Point3>& globalRotations, std::vector<RotationPair>& pairwiseRotations, REAL maxRelativeAngle)
{
	const REAL minCosAngle = COS(D2R(maxRelativeAngle));
	unsigned numPairs = 0, numInvalidPairs = 0;
	float maxInvalidWeight = 0.f;
	for (RotationPair& pair : pairwiseRotations) {
		if (pair.weight <= 0)
			continue;
		// Compute relative rotation from current global estimates:
		// R_rel_calc = R2 * R1^T
		const Matrix3x3 relCalcR = RMatrix(globalRotations[pair.idxB]) * RMatrix(globalRotations[pair.idxA]).t();
		// Get stored relative rotation
		const Matrix3x3& relStoredR = pair.relativeRotation;
		// Compute rotation difference: errorR = relStoredR * relCalcR^T
		const REAL cosAngle = ComputeAngle(relStoredR, relCalcR);
		if (cosAngle < minCosAngle) {
			// Invalidate the pair by setting weight to 0
			DEBUG_ULTIMATE("Filtered pair (% 4u, % 4u): relative rotation angle %.2f degrees, %.2f weight",
				pair.idxA, pair.idxB, R2D(ACOS(cosAngle)), pair.weight);
			if (pair.weight > maxInvalidWeight)
				maxInvalidWeight = pair.weight;
			pair.weight = 0;
			++numInvalidPairs;
		}
		++numPairs;
	}
	DEBUG("Filtered %u/%u relative rotations with angle > %.2f degrees (max weight %.2f)",
		numInvalidPairs, numPairs, maxRelativeAngle, maxInvalidWeight);
	return numInvalidPairs;
}
/*----------------------------------------------------------------*/

namespace {

// The translation stage of the contradiction cut: on the pairs the rotation stage kept, the scales
// of the pairs, then the camera centres, then every pair whose baseline direction is more than
// maxDirectionResidual degrees from the centres. The solves read the usable pairs with a measurable
// baseline (a ray angle past kNearDuplicateRayAngle), weighted by composite weight times triplet
// score; the scale equations are StarInitializer's (ComputePairScales), solved per component of the
// pair-scale graph by RobustAverage (Geman-McClure at the merge graph's bar) around the sparse
// GlobalScaleEstimator; the centres per component by GlobalTranslationEstimator reweighted towards
// the sum of norms (convex, so no start decides). A pair of a component is judged by its own solve,
// any other pair with a pose and a measurable baseline by the largest solve that places both its
// images. Marks the removed pairs in `removed`; returns their number.
constexpr unsigned minScaleComponent = 10;  // pairs a component of the pair-scale graph needs to be solved
constexpr unsigned numL1Rounds = 40;        // reweighting rounds of the centres
constexpr REAL l1Floor = REAL(1e-3);        // the L1 weight's error floor, in median pair lengths

unsigned RemoveTranslationContradictions(const Scene& scene, const std::vector<Point3>& rotations,
	const std::vector<uint32_t>& componentOf, const std::vector<float>& scores, float minPairWeight,
	float maxDirectionResidual, std::vector<bool>& removed)
{
	TD_TIMER_STARTD();
	const uint32_t numPairs = (uint32_t)scene.pairs.size();
	const uint32_t numImages = (uint32_t)scene.images.size();
	// the pairs judged, with their world-frame baseline (length: the pair's own, usually arbitrary)
	std::vector<Point3> baselines(numPairs);
	IIndexArr judged, solved;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (removed[idxPair] || !pair.relativePose.has_value() || !(pair.meanRayAngle >= D2R(kNearDuplicateRayAngle)) ||
			componentOf[pair.ID1] == NO_ID || componentOf[pair.ID1] != componentOf[pair.ID2])
			continue;
		baselines[idxPair] = RMatrix(rotations[pair.ID1]).t() * pair.relativePose->C;
		if (norm(baselines[idxPair]) <= ZEROTOLERANCE<REAL>())
			continue;
		judged.push_back((IIndex)idxPair);
		if (pair.GetCompositeWeight() > minPairWeight && scores[idxPair] != 0.f)
			solved.push_back((IIndex)idxPair);
	}
	const auto weightOf = [&](IIndex idxPair) {
		return scene.pairs[idxPair].GetCompositeWeight() * (scores[idxPair] < 0.f ? 1.f : scores[idxPair]);
	};

	// the components of the pair-scale graph, largest first (by pairs, ties to the lower pair)
	const std::vector<ScalePair> equations = ComputePairScales(scene, solved);
	DisjointSet<uint32_t> sets(numPairs);
	for (const ScalePair& equation : equations)
		sets.Union(equation.idxA, equation.idxB);
	struct Component { uint32_t root; std::vector<uint32_t> pairs; std::vector<ScalePair> equations; };
	std::unordered_map<uint32_t, Component> componentOfRoot;
	for (const ScalePair& equation : equations) {
		Component& component = componentOfRoot[sets.Find(equation.idxA)];
		component.root = sets.Find(equation.idxA);
		component.equations.push_back(equation);
	}
	std::vector<bool> inEquation(numPairs, false);
	for (const ScalePair& equation : equations)
		inEquation[equation.idxA] = inEquation[equation.idxB] = true;
	for (uint32_t idxPair = 0; idxPair < numPairs; ++idxPair)
		if (inEquation[idxPair])
			componentOfRoot[sets.Find(idxPair)].pairs.push_back(idxPair);
	std::vector<Component*> components;
	for (auto& [root, component] : componentOfRoot)
		components.push_back(&component);
	std::sort(components.begin(), components.end(), [](const Component* a, const Component* b) {
		return a->pairs.size() != b->pairs.size() ? a->pairs.size() > b->pairs.size() : a->root < b->root;
	});

	// one solve per component: the pair's angle against the centres, NaN where no solve judged it
	std::vector<REAL> angles(numPairs, std::numeric_limits<REAL>::quiet_NaN());
	std::vector<bool> scaled(numPairs, false);
	const REAL maxLogScaleResidual = LOGN((REAL)GlobalAlignmentConfig().maxGraphScaleResidual);
	unsigned numComponents = 0;
	std::vector<REAL> centresMedianResidual;
	for (const Component* pComponent : components) {
		if (pComponent->pairs.size() < minScaleComponent)
			break;
		// the scales of the component's pairs
		GlobalScaleEstimator scaleEstimator;
		std::vector<REAL> scales, scaleResiduals;
		if (!RobustAverage(pComponent->equations, maxLogScaleResidual, kRobustRounds,
			[&](const std::vector<ScalePair>& active, std::vector<REAL>& s) { return scaleEstimator.EstimateScales(active, numPairs, NO_ID, s, true); },
			[&](size_t i, const std::vector<REAL>& s) {
				const ScalePair& p = pComponent->equations[i];
				return LOGN(p.scaleRatio) - (LOGN(s[p.idxB]) - LOGN(s[p.idxA]));
			},
			scales, scaleResiduals))
			continue;
		// the centres: C_2 - C_1 = scale * baseline of every pair, towards the sum of norms
		std::vector<TranslationPair> offsets;
		std::vector<REAL> lengths;
		for (const uint32_t idxPair : pComponent->pairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			offsets.emplace_back(pair.ID1, pair.ID2, baselines[idxPair] * scales[idxPair], weightOf(idxPair));
			lengths.push_back(norm(offsets.back().relativeTranslation));
		}
		std::sort(lengths.begin(), lengths.end());
		const REAL floorLength = l1Floor * lengths[lengths.size() / 2];
		GlobalTranslationEstimator translationEstimator;
		std::vector<Point3> centres;
		std::vector<REAL> centreResiduals;
		if (!RobustAverageWith(offsets, numL1Rounds,
			[&](const std::vector<TranslationPair>& active, std::vector<Point3>& t) {
				// the estimator's rows are scaled by the weight, the least squares' weight is its square
				std::vector<TranslationPair> rows(active);
				for (TranslationPair& row : rows)
					row.weight = SQRT(row.weight);
				return translationEstimator.EstimateTranslations(rows, numImages, NO_ID, t);
			},
			[&](size_t i, const std::vector<Point3>& t) {
				const TranslationPair& p = offsets[i];
				return norm(t[p.idxB] - t[p.idxA] - p.relativeTranslation);
			},
			[&](REAL r) { return (float)(REAL(1) / MAXF(r, floorLength)); },
			centres, centreResiduals))
			continue;
		++numComponents;
		std::vector<bool> placed(numImages, false);
		for (const TranslationPair& p : offsets)
			placed[p.idxA] = placed[p.idxB] = true;
		for (const uint32_t idxPair : pComponent->pairs)
			scaled[idxPair] = true;
		// the angles: the component's own pairs by this solve, any other judged pair no solve has judged
		// yet when both its images are placed
		std::vector<REAL> residualsSorted(centreResiduals);
		std::sort(residualsSorted.begin(), residualsSorted.end());
		centresMedianResidual.push_back(residualsSorted[residualsSorted.size() / 2]);
		for (const IIndex idxPair : judged) {
			const ImagePair& pair = scene.pairs[idxPair];
			const bool own = sets.Find(idxPair) == pComponent->root && inEquation[idxPair];
			if (!own && (scaled[idxPair] || !std::isnan(angles[idxPair]) || !placed[pair.ID1] || !placed[pair.ID2]))
				continue;
			const Point3 centresOffset = centres[pair.ID2] - centres[pair.ID1];
			const REAL length = norm(centresOffset);
			if (length <= ZEROTOLERANCE<REAL>())
				continue;
			const REAL cosAngle = baselines[idxPair].dot(centresOffset) / (norm(baselines[idxPair]) * length);
			angles[idxPair] = R2D(ACOS(MINF(MAXF(cosAngle, REAL(-1)), REAL(1))));
		}
	}
	unsigned numJudged = 0, numRemoved = 0;
	for (const IIndex idxPair : judged) {
		if (std::isnan(angles[idxPair]))
			continue;
		++numJudged;
		if (angles[idxPair] <= maxDirectionResidual)
			continue;
		removed[idxPair] = true;
		++numRemoved;
	}
	VERBOSE("Translation cut: removed %u/%u pairs whose baseline direction is more than %g degrees from the centres of %u of %u scale components "
		"(%u pairs scaled by %u equations, median residual of the largest %.3g) (%s)",
		numRemoved, numJudged, maxDirectionResidual, numComponents, (unsigned)components.size(), (unsigned)solved.size(), (unsigned)equations.size(),
		centresMedianResidual.empty() ? REAL(0) : centresMedianResidual.front(), TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}

} // namespace
/*----------------------------------------------------------------*/

unsigned SFM::FilterPairsByContradiction(Scene& scene, const std::vector<float>& scores,
	float minPairWeight, float maxDirectionResidual, const PairsWeightingConfig& weightingCfg)
{
	ASSERT(scores.size() == scene.pairs.size());
	ASSERT(maxDirectionResidual >= 0 && maxDirectionResidual <= 180);
	TD_TIMER_STARTD();
	// the pairs the averaging is built from: the usable ones, weighted by the composite weight and the score
	std::vector<RotationPair> rotationPairs;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.relativePose.has_value() || pair.GetCompositeWeight() <= minPairWeight)
			continue;
		const float weight = pair.GetCompositeWeight() * (scores[idxPair] < 0.f ? 1.f : scores[idxPair]);
		if (weight > 0.f)
			rotationPairs.emplace_back(pair.ID1, pair.ID2, pair.relativePose->R, weight);
	}
	// each component of the pairs is averaged on its own, the largest first: its images get their
	// rotations and the component's number, the next solve takes the pairs left
	const GlobalRotationEstimatorOptions options;
	const uint32_t numImages = (uint32_t)scene.images.size();
	std::vector<Point3> rotations(numImages, Point3::INF);
	std::vector<uint32_t> componentOf(numImages, NO_ID);
	unsigned numComponents = 0;
	while (!rotationPairs.empty()) {
		std::vector<Point3> solved;
		GlobalRotationEstimator estimator(options);
		if (!estimator.EstimateRotations(rotationPairs, numImages, solved))
			break;
		for (uint32_t i = 0; i < numImages; ++i) {
			if (solved[i] == Point3::INF)
				continue;
			rotations[i] = solved[i];
			componentOf[i] = numComponents;
		}
		std::vector<RotationPair> leftPairs;
		for (const RotationPair& pair : rotationPairs) {
			// a pair joins the two images of one component, so both or neither are reached
			ASSERT((solved[pair.idxA] == Point3::INF) == (solved[pair.idxB] == Point3::INF));
			if (solved[pair.idxA] == Point3::INF)
				leftPairs.push_back(pair);
		}
		rotationPairs.swap(leftPairs);
		++numComponents;
	}
	// judge every pair with a pose whose two images one solve reached
	const REAL minCosAngle = COS(D2R(options.maxRelativeRotationAngle));
	std::vector<bool> removed(scene.pairs.size(), false);
	unsigned numJudged = 0, numRemoved = 0;
	float maxRemovedWeight = 0.f;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.relativePose.has_value() || componentOf[pair.ID1] == NO_ID || componentOf[pair.ID1] != componentOf[pair.ID2])
			continue;
		++numJudged;
		const Matrix3x3 solvedR = RMatrix(rotations[pair.ID2]) * RMatrix(rotations[pair.ID1]).t();
		if (ComputeAngle(pair.relativePose->R, solvedR) >= minCosAngle)
			continue;
		removed[idxPair] = true;
		maxRemovedWeight = MAXF(maxRemovedWeight, pair.GetCompositeWeight());
		++numRemoved;
	}
	VERBOSE("Contradiction cut: removed %u/%u pairs more than %g degrees from the rotations of the weighted averaging of %u usable components (heaviest %.1f)",
		numRemoved, numJudged, options.maxRelativeRotationAngle, numComponents, maxRemovedWeight);
	if (maxDirectionResidual > 0)
		numRemoved += RemoveTranslationContradictions(scene, rotations, componentOf, scores, minPairWeight, maxDirectionResidual, removed);
	if (numRemoved == 0)
		return 0;
	// remove the scene pairs, compacting in one pass
	const unsigned numPairs = scene.pairs.size();
	unsigned numKept = 0;
	for (unsigned idxPair = 0; idxPair < numPairs; ++idxPair) {
		if (removed[idxPair])
			continue;
		if (numKept != idxPair)
			scene.pairs[numKept] = std::move(scene.pairs[idxPair]);
		++numKept;
	}
	ASSERT(numPairs - numKept == numRemoved);
	scene.pairs.RemoveLast(numRemoved);
	ComputePairsWeights(scene, weightingCfg);
	DEBUG("Filtered the view graph by contradiction with its %s averaging: %u pairs removed (%s)",
		maxDirectionResidual > 0 ? "rotation and translation" : "rotation", numRemoved, TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
