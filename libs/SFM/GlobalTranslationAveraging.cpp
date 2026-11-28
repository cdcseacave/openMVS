/*
 * GlobalTranslationAveraging.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalTranslationAveraging.h"
#include "RobustAveraging.h"
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
	ASSERT(fixedIdx == NO_ID || fixedIdx < numIndices);

	// Collect unique node indices in deterministic first-appearance order
	std::unordered_set<uint32_t> idxSet;
	std::vector<uint32_t> orderedNodes;
	idxSet.reserve(pairwiseTranslations.size());
	for (const TranslationPair& transPair : pairwiseTranslations) {
		ASSERT(transPair.idxA < numIndices && transPair.idxB < numIndices);
		ASSERT(transPair.idxA != transPair.idxB);
		ASSERT(transPair.weight > 0.f && ISFINITE(transPair.weight));
		if (idxSet.insert(transPair.idxA).second)
			orderedNodes.push_back(transPair.idxA);
		if (idxSet.insert(transPair.idxB).second)
			orderedNodes.push_back(transPair.idxB);
	}

	const int N = (int)orderedNodes.size();
	const int M = (int)pairwiseTranslations.size();
	if (N < 2) {
		VERBOSE("warning: insufficient indices for translation estimation");
		return false;
	}

	const uint32_t gaugeIdx = SelectGaugeNode(pairwiseTranslations, fixedIdx);
	ASSERT(gaugeIdx != NO_ID && gaugeIdx < numIndices);

	// Pin gaugeIdx in its component and the lowest node in every other component
	DisjointSet<uint32_t> components(numIndices);
	for (const TranslationPair& transPair : pairwiseTranslations)
		components.UnionMin(transPair.idxA, transPair.idxB);
	std::unordered_map<uint32_t, uint32_t> pinOfComponent;
	if (idxSet.count(gaugeIdx))
		pinOfComponent.emplace(components.Find(gaugeIdx), gaugeIdx);
	for (const uint32_t idx : orderedNodes)
		pinOfComponent.emplace(components.Find(idx), components.Find(idx));

	// Map free (unpinned) nodes to column indices in deterministic order
	std::unordered_map<uint32_t, int> idxToVar;
	idxToVar.reserve((size_t)N - 1);
	normal.nodeOfRow.clear();
	normal.nodeOfRow.reserve((size_t)N - 1);
	for (const uint32_t idx : orderedNodes) {
		if (pinOfComponent.at(components.Find(idx)) == idx)
			continue;
		idxToVar.emplace(idx, (int)normal.nodeOfRow.size());
		normal.nodeOfRow.push_back(idx);
	}
	const int V = (int)normal.nodeOfRow.size();

	// Build incidence matrix A and 3-column RHS B for t_B - t_A = relativeTranslation
	std::vector<Eigen::Triplet<double>> triplets;
	triplets.reserve((size_t)M * 2);
	Eigen::MatrixX3d B(M, 3);
	for (int i = 0; i < M; ++i) {
		const TranslationPair& transPair = pairwiseTranslations[i];
		const double weight = (double)transPair.weight;
		auto itA = idxToVar.find(transPair.idxA);
		auto itB = idxToVar.find(transPair.idxB);
		if (itA != idxToVar.end())
			triplets.emplace_back(i, itA->second, -weight);
		if (itB != idxToVar.end())
			triplets.emplace_back(i, itB->second, weight);
		B(i, 0) = weight * (double)transPair.relativeTranslation.x;
		B(i, 1) = weight * (double)transPair.relativeTranslation.y;
		B(i, 2) = weight * (double)transPair.relativeTranslation.z;
	}

	Eigen::SparseMatrix<double> A(M, V);
	A.setFromTriplets(triplets.begin(), triplets.end());

	// Solve the weighted graph Laplacian via sparse Cholesky with one refinement step
	const Eigen::SparseMatrix<double> At = A.transpose();
	normal.matrix = At * A;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(normal.matrix);
	if (solver.info() != Eigen::Success) {
		VERBOSE("error: solver decomposition failed");
		return false;
	}
	DEBUG_EXTRA("Global translation normal equations: %d unknowns, %zu non-zeros, factor %zu (incidence %d x %d)",
		V, (size_t)normal.matrix.nonZeros(), (size_t)solver.matrixL().nestedExpression().nonZeros(), M, V);
	Eigen::MatrixX3d X = solver.solve(At * B);
	X += solver.solve(At * (B - A * X));
	if (solver.info() != Eigen::Success || !X.array().isFinite().all()) {
		VERBOSE("error: solver failed");
		return false;
	}

	outTranslations.assign(numIndices, Point3::ZERO);
	for (int varIdx = 0; varIdx < V; ++varIdx)
		outTranslations[normal.nodeOfRow[varIdx]] = Point3((REAL)X(varIdx, 0), (REAL)X(varIdx, 1), (REAL)X(varIdx, 2));

	DEBUG("Global translation averaging completed: %u indices, %u pairs (gauge idx %u fixed at origin)", N, M, gaugeIdx);
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
