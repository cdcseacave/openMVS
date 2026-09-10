/*
 * GlobalTranslationAveraging.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalTranslationAveraging.h"
#include <Eigen/SparseQR>

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

	// Build final map excluding the gauge node; this enforces t_gauge = 0 exactly.
	std::unordered_map<uint32_t, int> idxToVar;
	idxToVar.reserve((size_t)N - 1);
	for (const uint32_t idx : idxSet) {
		if (idx == gaugeIdx)
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

	// Solve using Sparse QR
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
	solver.compute(A);
	if (solver.info() != Eigen::Success) {
		VERBOSE("error: solver decomposition failed");
		return false;
	}

	Eigen::VectorXd xX = solver.solve(bX);
	Eigen::VectorXd xY = solver.solve(bY);
	Eigen::VectorXd xZ = solver.solve(bZ);
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
