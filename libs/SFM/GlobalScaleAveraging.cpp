/*
 * GlobalScaleAveraging.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalScaleAveraging.h"
#include <Eigen/IterativeLinearSolvers>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("GlbSclAg"));

namespace {

// The sparse route's conjugate gradients on the normal equations A^T A x = A^T b. The ridge, relative
// to the mean diagonal, keeps solvable a component the fixed index does not reach (its scales are
// free); the tolerance is tight because at 1e-10 an equation on the robust weight floor crosses the
// bar in another round and the cut's removed pairs change; the correction of the refinement step
// needs only its first digits.
constexpr double kSparseRidge = 1e-12;
constexpr double kSparseTolerance = 1e-14;
constexpr int kSparseMaxIterations = 20000;
constexpr double kRefinementTolerance = 1e-6;

bool SelectFixedNode(
	const std::vector<ScalePair>& pairwiseScales,
	const uint32_t fixedIdx,
	const std::unordered_map<uint32_t, int>& idxToVar,
	uint32_t& fixedNode)
{
	fixedNode = fixedIdx;
	if (fixedNode == NO_ID) {
		std::unordered_map<uint32_t, float> connectionWeights;
		for (const ScalePair& scalePair : pairwiseScales) {
			connectionWeights[scalePair.idxA] += scalePair.weight;
			connectionWeights[scalePair.idxB] += scalePair.weight;
		}
		float maxWeight = 0.f;
		for (const auto& [idx, weight] : connectionWeights) {
			if (weight > maxWeight) {
				maxWeight = weight;
				fixedNode = idx;
			}
		}
	}
	if (fixedNode == NO_ID || idxToVar.find(fixedNode) == idxToVar.end()) {
		VERBOSE("warning: invalid fixed index for scale estimation");
		return false;
	}
	return true;
}

} // namespace


bool GlobalScaleEstimator::EstimateScales(
	const std::vector<ScalePair>& pairwiseScales,
	const uint32_t numIndices,
	const uint32_t fixedIdx,
	std::vector<REAL>& outScales,
	bool sparse)
{
	if (pairwiseScales.empty() || numIndices == 0) {
		VERBOSE("warning: no pairwise scales provided");
		return false;
	}

	// Map index to variable position in the linear system
	std::unordered_map<uint32_t, int> idxToVar;
	for (const ScalePair& scalePair : pairwiseScales) {
		ASSERT(scalePair.scaleRatio > REAL(0));
		ASSERT(ISFINITE(scalePair.scaleRatio));
		ASSERT(scalePair.weight > 0.f);
		ASSERT(ISFINITE(scalePair.weight));
		const auto [itA, insertedA] = idxToVar.emplace(scalePair.idxA, (int)idxToVar.size());
		(void)itA;
		(void)insertedA;
		const auto [itB, insertedB] = idxToVar.emplace(scalePair.idxB, (int)idxToVar.size());
		(void)itB;
		(void)insertedB;
	}

	const int N = (int)idxToVar.size(); // number of variables (scales)
	const int M = (int)pairwiseScales.size(); // number of equations
	if (N < 2) {
		VERBOSE("warning: insufficient indices for scale estimation");
		return false;
	}

	uint32_t fixedNode;
	if (!SelectFixedNode(pairwiseScales, fixedIdx, idxToVar, fixedNode))
		return false;

	// Eliminate the fixed variable for exact gauge enforcement.
	std::unordered_map<uint32_t, int> idxToTmpVar;
	idxToTmpVar.reserve((size_t)N - 1);
	for (const auto& [idx, varIdx] : idxToVar) {
		(void)varIdx;
		if (idx == fixedNode)
			continue;
		idxToTmpVar.emplace(idx, (int)idxToTmpVar.size());
	}
	const int Ntmp = (int)idxToTmpVar.size();
	if (Ntmp == 0) {
		outScales.resize(numIndices, REAL(1));
		return true;
	}

	Eigen::VectorXd b = Eigen::VectorXd::Zero(M);
	FOREACH(i, pairwiseScales) {
		const ScalePair& scalePair = pairwiseScales[i];
		b(i) = (double)scalePair.weight * LOGN((double)scalePair.scaleRatio);
	}
	Eigen::VectorXd x;
	if (sparse) {
		std::vector<Eigen::Triplet<double>> triplets;
		triplets.reserve((size_t)M * 2);
		FOREACH(i, pairwiseScales) {
			const ScalePair& scalePair = pairwiseScales[i];
			const double weight = (double)scalePair.weight;
			const auto itA = idxToTmpVar.find(scalePair.idxA);
			const auto itB = idxToTmpVar.find(scalePair.idxB);
			if (itA != idxToTmpVar.end())
				triplets.emplace_back((int)i, itA->second, -weight);
			if (itB != idxToTmpVar.end())
				triplets.emplace_back((int)i, itB->second, weight);
		}
		Eigen::SparseMatrix<double> A(M, Ntmp);
		A.setFromTriplets(triplets.begin(), triplets.end());
		Eigen::SparseMatrix<double> normal = A.transpose() * A;
		const double ridge = kSparseRidge * normal.diagonal().mean();
		for (int i = 0; i < Ntmp; ++i)
			normal.coeffRef(i, i) += ridge;
		// the start: the previous sparse solve's scales taken relative to this solve's fixed index, when it
		// solved as many indices (a robust round's change of the equations moves the answer a little)
		Eigen::VectorXd x0 = Eigen::VectorXd::Zero(Ntmp);
		if (previousScales.size() == numIndices && fixedNode < numIndices) {
			const double logFixed = LOGN((double)previousScales[fixedNode]);
			for (const auto& [idx, tmpVarIdx] : idxToTmpVar) {
				ASSERT(idx < numIndices);
				x0(tmpVarIdx) = LOGN((double)previousScales[idx]) - logFixed;
			}
		}
		// conjugate gradients with the Jacobi preconditioner: the normal matrix of a view graph's pair
		// scales is as dense as an expander and its Cholesky factor many times denser, while a product
		// with it is cheap
		Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
		solver.setTolerance(kSparseTolerance);
		solver.setMaxIterations(kSparseMaxIterations);
		solver.compute(normal);
		x = solver.solveWithGuess(A.transpose() * b, x0);
		solve.iterations = (unsigned)solver.iterations();
		solve.converged = solver.info() == Eigen::Success;
		if (x.array().isNaN().any()) {
			VERBOSE("warning: sparse scale solve failed");
			return false;
		}
		// one step of refinement against the incidence matrix itself, as the translation solve does: the
		// normal matrix holds the squared condition number's rounding (a weak equation between two groups
		// of strong ones moved the answer by 3e-7), the residual of A and b does not
		solver.setTolerance(kRefinementTolerance);
		x += solver.solve(A.transpose() * (b - A * x) - ridge * x);
		solve.refinementIterations = (unsigned)solver.iterations();
		solve.converged = solve.converged && solver.info() == Eigen::Success;
		DEBUG_EXTRA("Global scale conjugate gradients: %u unknowns, %u iterations and %u of refinement, %s", (unsigned)Ntmp,
			solve.iterations, solve.refinementIterations, solve.converged ? "converged" : "stopped at the iteration bound");
	} else {
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(M, Ntmp);
		FOREACH(i, pairwiseScales) {
			const ScalePair& scalePair = pairwiseScales[i];
			const double weight = (double)scalePair.weight;
			const auto itA = idxToTmpVar.find(scalePair.idxA);
			const auto itB = idxToTmpVar.find(scalePair.idxB);
			if (itA != idxToTmpVar.end())
				A(i, itA->second) = -weight;
			if (itB != idxToTmpVar.end())
				A(i, itB->second) = weight;
		}
		x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	}

	outScales.resize(numIndices, REAL(1));
	for (const auto& [idx, tmpVarIdx] : idxToTmpVar) {
		ASSERT(idx < numIndices);
		outScales[idx] = EXP(x(tmpVarIdx));
	}
	if (fixedNode < numIndices)
		outScales[fixedNode] = REAL(1);
	if (sparse)
		previousScales = outScales;

	DEBUG("Global scale averaging completed: %u indices, %u pairs (exact gauge, fixed %u)",
		N, M, fixedNode);
	return true;
}
/*----------------------------------------------------------------*/
