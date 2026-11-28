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
#include "PoseLink.h"
#include "Scene.h"
#include "../Math/DisjointSet.h"
#include "../Math/LeastAbsoluteDeviationSolver.h"
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


// Estimate rotations for all images in the scene
bool GlobalRotationEstimator::EstimateRotations(Scene& scene, unsigned* pNumFilteredPairs)
{
	TD_TIMER_STARTD();

	std::vector<RotationPair> rotationPairs;
	rotationPairs.reserve(scene.pairs.size());
	for (const ImagePair& pair : scene.pairs) {
		if (!IsPoseLinkPair(pair))
			continue;
		const float weight = options.useWeight ? pair.GetCompositeWeight() : (float)pair.GetNumWeightedInliers();
		rotationPairs.emplace_back(pair.ID1, pair.ID2, pair.relativePose->R, weight);
	}
	if (rotationPairs.empty()) {
		VERBOSE("error: no valid image pairs for rotation averaging");
		return false;
	}

	std::vector<Point3> globalRotations;
	if (options.skipInitialization) {
		globalRotations.reserve(scene.images.size());
		for (const Image& img : scene.images)
			globalRotations.emplace_back(img.IsValid() ? Point3(img.R.GetRotationAxisAngle()) : Point3::INF);
	}

	if (!EstimateRotations(rotationPairs, scene.images.size(), globalRotations))
		return false;

	for (Image& image : scene.images) {
		if (globalRotations[image.ID] != Point3::INF) {
			image.R.SetRotationAxisAngle(globalRotations[image.ID]);
			if (!image.IsValid())
				image.C = Point3::ZERO;
		}
	}

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

	fixedNodeId = NO_ID;
	if (!globalRotations.empty()) {
		ASSERT(globalRotations.size() == numNodes);
		estimatedRotations = globalRotations;
	} else
		InitializeFromMaximumSpanningTree(numNodes, pairwiseRotations);

	if (!SetupLinearSystem(pairwiseRotations))
		return false;

	if (options.maxNumL1Iterations > 0 && !SolveL1Regression(pairwiseRotations))
		return false;

	if (options.maxNumIrlsIterations > 0 && !SolveIRLS(pairwiseRotations))
		return false;

	globalRotations.assign(numNodes, Point3::INF);
	for (const auto& [nodeId, idx] : nodeIdToIdx)
		globalRotations[nodeId] = estimatedRotations[idx];

	DEBUG("Global rotation averaging completed: %u nodes, %u pairs (%s)",
	      (unsigned)nodeIdToIdx.size(), (unsigned)pairIdToInfo.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

unsigned GlobalRotationEstimator::EstimateAllComponents(
	const std::vector<RotationPair>& pairwiseRotations,
	const uint32_t numNodes,
	std::vector<Point3>& rotations,
	std::vector<uint32_t>& componentOf) const
{
	rotations.assign(numNodes, Point3::INF);
	componentOf.assign(numNodes, NO_ID);
	std::vector<RotationPair> remaining(pairwiseRotations);
	unsigned numComponents = 0;
	while (!remaining.empty()) {
		std::vector<Point3> solved;
		GlobalRotationEstimator estimator(options);
		if (!estimator.EstimateRotations(remaining, numNodes, solved))
			break;
		// a node is placed once, by the first run that solves it; a run placing nothing ends the loop
		unsigned numPlaced = 0;
		for (uint32_t i = 0; i < numNodes; ++i) {
			if (solved[i] == Point3::INF || componentOf[i] != NO_ID)
				continue;
			rotations[i] = solved[i];
			componentOf[i] = numComponents;
			++numPlaced;
		}
		if (numPlaced == 0)
			break;
		std::vector<RotationPair> next;
		for (const RotationPair& pair : remaining) {
			ASSERT((solved[pair.idxA] == Point3::INF) == (solved[pair.idxB] == Point3::INF));
			if (componentOf[pair.idxA] == NO_ID && componentOf[pair.idxB] == NO_ID)
				next.push_back(pair);
		}
		remaining.swap(next);
		++numComponents;
	}
	return numComponents;
}

void GlobalRotationEstimator::InitializeFromMaximumSpanningTree(uint32_t numNodes, const std::vector<RotationPair>& pairwiseRotations)
{
	std::vector<Point3> rotations;
	const uint32_t root = RotationsFromMaximumSpanningTree(numNodes, pairwiseRotations, rotations);
	if (root == NO_ID)
		return;
	estimatedRotations = rotations;
	if (fixedNodeId == NO_ID)
		fixedNodeId = root;
}

uint32_t GlobalRotationEstimator::RotationsFromMaximumSpanningTree(
	uint32_t numNodes, const std::vector<RotationPair>& pairwiseRotations,
	std::vector<Point3>& rotations)
{
	if (numNodes < 2 || pairwiseRotations.empty())
		return NO_ID;

	// Sort valid pair indices descending by weight for Kruskal's maximum spanning forest
	std::vector<uint32_t> order;
	order.reserve(pairwiseRotations.size());
	FOREACH(pairIdx, pairwiseRotations) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		ASSERT(pair.idxA < numNodes && pair.idxB < numNodes);
		if (pair.weight > 0.f && pair.idxA != pair.idxB)
			order.push_back((uint32_t)pairIdx);
	}
	if (order.empty())
		return NO_ID;
	std::stable_sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
		return pairwiseRotations[a].weight > pairwiseRotations[b].weight;
	});

	// Build the maximum spanning forest and find the largest connected component
	DisjointSet<uint32_t> dsu(numNodes);
	std::vector<uint32_t> mstPairs;
	mstPairs.reserve(numNodes - 1);
	for (const uint32_t pairIdx : order) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		if (dsu.UnionMin(pair.idxA, pair.idxB))
			mstPairs.push_back(pairIdx);
	}
	if (mstPairs.empty())
		return NO_ID;

	dsu.CompressAllPaths();
	std::vector<uint32_t> compOfNode;
	const unsigned numComponents = dsu.GetComponents(compOfNode);
	std::vector<unsigned> compSize(numComponents, 0);
	for (const uint32_t c : compOfNode)
		++compSize[c];
	unsigned largestComp = 0;
	for (unsigned c = 1; c < numComponents; ++c)
		if (compSize[c] > compSize[largestComp])
			largestComp = c;
	if (compSize[largestComp] < 2)
		return NO_ID;

	// Adjacency list carrying the exact chosen pair index (handles parallel edges accurately in O(1))
	struct TreeEdge { uint32_t neighbor; uint32_t pairIdx; };
	std::vector<std::vector<TreeEdge>> adj(numNodes);
	for (const uint32_t pairIdx : mstPairs) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		if (compOfNode[pair.idxA] != largestComp)
			continue;
		adj[pair.idxA].push_back({ pair.idxB, pairIdx });
		adj[pair.idxB].push_back({ pair.idxA, pairIdx });
	}

	uint32_t root = NO_ID;
	for (uint32_t i = 0; i < numNodes; ++i)
		if (!adj[i].empty() && (root == NO_ID || adj[i].size() > adj[root].size()))
			root = i;
	ASSERT(root != NO_ID);

	// Propagate rotations over the MST starting from root at identity
	rotations.assign(numNodes, Point3::INF);
	std::queue<uint32_t> q;
	std::vector<uint8_t> visited(numNodes, 0);
	q.push(root);
	visited[root] = 1;
	rotations[root] = Point3::ZERO;
	while (!q.empty()) {
		const uint32_t curr = q.front();
		q.pop();
		const RMatrix Rcurr(rotations[curr]);
		for (const TreeEdge& edge : adj[curr]) {
			if (visited[edge.neighbor])
				continue;
			visited[edge.neighbor] = 1;
			const RotationPair& pair = pairwiseRotations[edge.pairIdx];
			const RMatrix Rchild = (pair.idxA == curr) ? RMatrix(pair.relativeRotation * Rcurr)
			                                           : RMatrix(pair.relativeRotation.t() * Rcurr);
			rotations[edge.neighbor] = Rchild.GetRotationAxisAngle();
			q.push(edge.neighbor);
		}
	}

	DEBUG("Initialized rotations for %u nodes using MST (root: %u)", compSize[largestComp], root);
	return root;
}

bool GlobalRotationEstimator::SetupLinearSystem(const std::vector<RotationPair>& pairwiseRotations)
{
	sparseMatrix.resize(0, 0);
	tangentSpaceStep.resize(0);
	tangentSpaceResidual.resize(0);
	weights.resize(0);
	nodeIdToIdx.clear();
	pairIdToInfo.clear();

	const uint32_t numInitialNodes = (uint32_t)estimatedRotations.size();
	ASSERT(fixedNodeId == NO_ID ||
		(fixedNodeId < numInitialNodes && estimatedRotations[fixedNodeId] != Point3::INF),
		"fixed node is invalid for rotation averaging");
	Point3d fixedNodeRotation;
	for (uint32_t nodeId = 0; nodeId < numInitialNodes; ++nodeId) {
		const Point3d rotation = estimatedRotations[nodeId];
		if (rotation == Point3::INF)
			continue;
		if (fixedNodeId == NO_ID)
			fixedNodeId = nodeId;
		if (nodeId == fixedNodeId) {
			fixedNodeRotation = rotation;
			continue;
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
	estimatedRotations[numFreeNodes] = fixedNodeRotation;
	nodeIdToIdx.emplace(fixedNodeId, numFreeNodes);
	estimatedRotations.resize(nodeIdToIdx.size());

	std::vector<Eigen::Triplet<double>> vecCoeffs;
	std::vector<double> pairWeights;
	vecCoeffs.reserve(pairwiseRotations.size() * 6);
	pairWeights.reserve(pairwiseRotations.size());
	unsigned currPos = 0;
	FOREACH(pairIdx, pairwiseRotations) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		if (pair.weight <= 0)
			continue;
		auto itA = nodeIdToIdx.find(pair.idxA);
		auto itB = nodeIdToIdx.find(pair.idxB);
		if (itA == nodeIdToIdx.end() || itB == nodeIdToIdx.end())
			continue;
		pairIdToInfo[pairIdx] = currPos;
		const double weight = options.useWeight ? (double)pair.weight : 1.0;
		const uint32_t idx1 = itA->second * 3;
		const uint32_t idx2 = itB->second * 3;
		for (int i = 0; i < 3; ++i) {
			if (pair.idxA != fixedNodeId)
				vecCoeffs.emplace_back(currPos + i, idx1 + i, -1.0);
			if (pair.idxB != fixedNodeId)
				vecCoeffs.emplace_back(currPos + i, idx2 + i, 1.0);
		}
		pairWeights.push_back(weight);
		currPos += 3;
	}

	// Normalize pair weights to median = options.weightMedian so L1 ADMM is scale-invariant
	ASSERT(options.weightMedian > 0 && !pairWeights.empty());
	std::vector<double> sortedWeights(pairWeights);
	std::nth_element(sortedWeights.begin(), sortedWeights.begin() + sortedWeights.size() / 2, sortedWeights.end());
	const double medianWeight = sortedWeights[sortedWeights.size() / 2];
	ASSERT(medianWeight > 0);
	const double scale = options.weightMedian / medianWeight;

	const unsigned numDof = numFreeNodes * 3;
	sparseMatrix.resize(currPos, numDof);
	sparseMatrix.setFromTriplets(vecCoeffs.begin(), vecCoeffs.end());

	weights.resize(currPos);
	FOREACH(k, pairWeights)
		weights.segment<3>((Eigen::Index)k * 3).setConstant(pairWeights[k] * scale);

	tangentSpaceStep.resize(numDof);
	tangentSpaceResidual.resize(currPos);
	return true;
}

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
		for (const auto& [pairIdx, pos] : pairIdToInfo) {
			const double residualSq = tangentSpaceResidual.segment<3>(pos).squaredNorm();
			double weight;
			if (options.weightType == GlobalRotationEstimatorOptions::GEMAN_MCCLURE) {
				const double sigmaSq = SQUARE(sigma);
				weight = sigmaSq / SQUARE(sigmaSq + residualSq);
			} else {
				weight = 1.0 / MAXF(SQRT(residualSq), sigma);
			}
			weightsIrls.segment<3>(pos) = weight * weights.segment<3>(pos);
		}

		const Eigen::SparseMatrix<double> ATWeight = sparseMatrix.transpose() * weightsIrls.matrix().asDiagonal();
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

void GlobalRotationEstimator::UpdateGlobalRotations()
{
	ASSERT(estimatedRotations.size() >= 2);
	const uint32_t numFreeNodes = (uint32_t)estimatedRotations.size() - 1;
	for (uint32_t idx = 0; idx < numFreeNodes; ++idx) {
		const RMatrix currentR(estimatedRotations[idx]);
		const RMatrix updateR(Point3(-tangentSpaceStep.segment<3>(idx * 3)));
		estimatedRotations[idx] = RMatrix(currentR * updateR).GetRotationAxisAngle();
	}
}

void GlobalRotationEstimator::ComputeResiduals(const std::vector<RotationPair>& pairwiseRotations)
{
	for (const auto& [pairIdx, pos] : pairIdToInfo) {
		const RotationPair& pair = pairwiseRotations[pairIdx];
		const RMatrix R1(estimatedRotations[nodeIdToIdx.at(pair.idxA)]);
		const RMatrix R2(estimatedRotations[nodeIdToIdx.at(pair.idxB)]);
		const RMatrix errorR = R2.t() * pair.relativeRotation * R1;
		tangentSpaceResidual.segment<3>(pos) = -Eigen::Vector3d(errorR.GetRotationAxisAngle());
	}
}

double GlobalRotationEstimator::ComputeAverageStepSize() const
{
	ASSERT(estimatedRotations.size() >= 2);
	const uint32_t numFreeNodes = (uint32_t)estimatedRotations.size() - 1;
	double totalUpdate = 0;
	for (uint32_t idx = 0; idx < numFreeNodes; ++idx)
		totalUpdate += tangentSpaceStep.segment<3>(idx * 3).norm();
	return totalUpdate / numFreeNodes;
}

unsigned GlobalRotationEstimator::FilterRelativeRotations(Scene& scene, REAL maxRelativeAngle)
{
	const REAL minCosAngle = COS(D2R(maxRelativeAngle));
	unsigned numPairs = 0, numInvalidPairs = 0;
	float maxInvalidWeight = 0.f;
	for (ImagePair& pair : scene.pairs) {
		if (!IsPoseLinkPair(pair))
			continue;
		const Image& image1 = scene.images[pair.ID1];
		const Image& image2 = scene.images[pair.ID2];
		ASSERT(image1.HasCamera() && image2.HasCamera());
		if (!image1.IsValid() || !image2.IsValid())
			continue;
		const REAL cosAngle = ComputeAngle(pair.relativePose->R, Matrix3x3(image2.R * image1.R.t()));
		if (cosAngle < minCosAngle) {
			DEBUG_ULTIMATE("Filtered pair (% 4u, % 4u): %u/%u matches, relative rotation angle %.2f degrees, %.2f weight",
				pair.ID1, pair.ID2, pair.GetNumFilteredInliers(), pair.GetNumMatches(), R2D(ACOS(cosAngle)), pair.GetCompositeWeight());
			maxInvalidWeight = MAXF(maxInvalidWeight, pair.GetCompositeWeight());
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
		const REAL cosAngle = ComputeAngle(pair.relativeRotation, Matrix3x3(RMatrix(globalRotations[pair.idxB]) * RMatrix(globalRotations[pair.idxA]).t()));
		if (cosAngle < minCosAngle) {
			DEBUG_ULTIMATE("Filtered pair (% 4u, % 4u): relative rotation angle %.2f degrees, %.2f weight",
				pair.idxA, pair.idxB, R2D(ACOS(cosAngle)), pair.weight);
			maxInvalidWeight = MAXF(maxInvalidWeight, pair.weight);
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

#pragma pop_macro("VERBOSE")
