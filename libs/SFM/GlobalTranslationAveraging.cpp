/*
 * GlobalTranslationAveraging.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalTranslationAveraging.h"
#include <Eigen/SparseCholesky>
#include "../Math/DisjointSet.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("GlbTrsAg"));


bool GlobalTranslationEstimator::EstimateTranslations(
	const std::vector<TranslationPair>& pairwiseTranslations,
	const uint32_t numIndices,
	const uint32_t fixedIdx,
	std::vector<Point3>& outTranslations)
{
	if (pairwiseTranslations.empty() || numIndices == 0) {
		VERBOSE("warning: no pairwise translations provided");
		return false;
	}

	// Collect unique node indices
	std::unordered_set<uint32_t> idxSet;
	idxSet.reserve(pairwiseTranslations.size());
	for (const auto& transPair : pairwiseTranslations) {
		idxSet.emplace(transPair.idxA);
		idxSet.emplace(transPair.idxB);
	}

	const int N = (int)idxSet.size(); // number of nodes with translation unknowns
	const int M = (int)pairwiseTranslations.size(); // number of equations
	if (N < 2) {
		VERBOSE("warning: insufficient indices for translation estimation");
		return false;
	}

	// Find the gauge index: use the fixed index if provided, otherwise find the best-connected node
	uint32_t gaugeIdx = fixedIdx;
	if (gaugeIdx == NO_ID) {
		std::unordered_map<uint32_t, float> nodeWeights;
		nodeWeights.reserve(N);
		float maxWeight = 0.f;
		for (const auto& transPair : pairwiseTranslations) {
			nodeWeights[transPair.idxA] += transPair.weight;
			nodeWeights[transPair.idxB] += transPair.weight;
		}
		for (const auto& [idx, weight] : nodeWeights) {
			if (weight > maxWeight) {
				maxWeight = weight;
				gaugeIdx = idx;
			}
		}
	}
	ASSERT(gaugeIdx != NO_ID);

	// A component of the pairs that does not hold the gauge node floats free: its lowest node is
	// pinned at the origin too, which makes the normal equations below positive definite
	DisjointSet<uint32_t> components(numIndices);
	for (const auto& transPair : pairwiseTranslations) {
		ASSERT(transPair.idxA < numIndices && transPair.idxB < numIndices);
		components.Union(transPair.idxA, transPair.idxB);
	}
	std::unordered_map<uint32_t, uint32_t> pinOfComponent;
	if (idxSet.count(gaugeIdx))
		pinOfComponent.emplace(components.Find(gaugeIdx), gaugeIdx);
	for (const uint32_t idx : idxSet) {
		const auto [it, inserted] = pinOfComponent.emplace(components.Find(idx), idx);
		if (!inserted && it->second != gaugeIdx)
			it->second = MINF(it->second, idx);
	}

	// Build final map excluding the pinned nodes; this enforces t_pinned = 0 exactly.
	std::unordered_map<uint32_t, int> idxToVar;
	idxToVar.reserve((size_t)N - 1);
	for (const uint32_t idx : idxSet) {
		if (pinOfComponent.at(components.Find(idx)) == idx)
			continue;
		idxToVar.emplace(idx, (int)idxToVar.size());
	}
	const int V = (int)idxToVar.size();

	// Build sparse linear system for each coordinate (X, Y, Z separately)
	// System: A * t = b
	// Equations: t_j - t_i = relative_translation_ij
	typedef Eigen::Triplet<double> T;
	std::vector<T> triplets;
	triplets.reserve((size_t)M * 2);

	// Add pairwise constraints
	Eigen::VectorXd bX = Eigen::VectorXd::Zero(M);
	Eigen::VectorXd bY = Eigen::VectorXd::Zero(M);
	Eigen::VectorXd bZ = Eigen::VectorXd::Zero(M);
	for (int i = 0; i < M; ++i) {
		const TranslationPair& transPair = pairwiseTranslations[i];
		const double weight = (double)transPair.weight;

		// Equation: t_B - t_A = relative_translation, with t_gauge fixed to zero.
		auto itA = idxToVar.find(transPair.idxA);
		auto itB = idxToVar.find(transPair.idxB);
		if (itA != idxToVar.end())
			triplets.emplace_back(i, itA->second, -weight);
		if (itB != idxToVar.end())
			triplets.emplace_back(i, itB->second, weight);

		bX(i) = weight * (double)transPair.relativeTranslation.x;
		bY(i) = weight * (double)transPair.relativeTranslation.y;
		bZ(i) = weight * (double)transPair.relativeTranslation.z;
	}

	// Build sparse matrix
	Eigen::SparseMatrix<double> A(M, V);
	A.setFromTriplets(triplets.begin(), triplets.end());

	// Solve the normal equations (a weighted graph Laplacian) by a sparse Cholesky: the same least
	// squares as the QR of the incidence matrix, a thousand times faster on a view graph's
	const Eigen::SparseMatrix<double> At = A.transpose();
	normal.matrix = At * A;
	normal.nodeOfRow.assign((size_t)V, NO_ID);
	for (const auto& [idx, varIdx] : idxToVar)
		normal.nodeOfRow[varIdx] = idx;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(normal.matrix);
	if (solver.info() != Eigen::Success) {
		VERBOSE("error: solver decomposition failed");
		return false;
	}
	DEBUG_EXTRA("Global translation normal equations: %d unknowns, %zu non-zeros, factor %zu (incidence %d x %d)",
		V, (size_t)normal.matrix.nonZeros(), (size_t)solver.matrixL().nestedExpression().nonZeros(), M, V);
	// one step of refinement against the incidence matrix itself recovers the digits the squared condition number takes
	const auto solve = [&](const Eigen::VectorXd& b) {
		Eigen::VectorXd x = solver.solve(At * b);
		x += solver.solve(At * (b - A * x));
		return x;
	};
	const Eigen::VectorXd xX = solve(bX);
	const Eigen::VectorXd xY = solve(bY);
	const Eigen::VectorXd xZ = solve(bZ);
	if (solver.info() != Eigen::Success) {
		VERBOSE("error: solver failed");
		return false;
	}

	// Convert results to output translations
	outTranslations.resize(numIndices, Point3::ZERO);
	outTranslations[gaugeIdx] = Point3::ZERO;
	for (const auto& [idx, varIdx] : idxToVar) {
		ASSERT(idx < numIndices);
		outTranslations[idx] = Point3(
			(REAL)xX(varIdx),
			(REAL)xY(varIdx),
			(REAL)xZ(varIdx));
	}

	DEBUG("Global translation averaging completed: %u indices, %u pairs (gauge idx %u fixed at origin)", N, M, gaugeIdx);
	return true;
}
/*----------------------------------------------------------------*/
