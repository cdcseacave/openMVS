/*
 * GlobalScaleAveraging.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalScaleAveraging.h"
#include <Eigen/IterativeLinearSolvers>
#include "../Math/DisjointSet.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("GlbSclAg"));

namespace {

// The conjugate gradients on the normal equations A^T A x = A^T b of a component pinned at one of its
// indices: the pinned system's normal matrix is positive definite, so it needs no ridge. The tolerance is
// tight because at 1e-10 an equation on the robust weight floor crosses the bar in another round and the
// cut's removed pairs change; the correction of the refinement step needs only its first digits.
constexpr double kTolerance = 1e-14;
constexpr int kMaxIterations = 20000;
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
	std::vector<REAL>& outScales)
{
	if (pairwiseScales.empty() || numIndices == 0) {
		VERBOSE("warning: no pairwise scales provided");
		return false;
	}

	// The unknowns are the indices the equations name, numbered in order of first appearance
	std::unordered_map<uint32_t, int> idxToVar;
	std::vector<uint32_t> varToIdx;
	for (const ScalePair& scalePair : pairwiseScales) {
		ASSERT(scalePair.scaleRatio > REAL(0));
		ASSERT(ISFINITE(scalePair.scaleRatio));
		ASSERT(scalePair.weight > 0.f);
		ASSERT(ISFINITE(scalePair.weight));
		ASSERT(scalePair.idxA != scalePair.idxB);
		ASSERT(scalePair.idxA < numIndices && scalePair.idxB < numIndices);
		if (idxToVar.emplace(scalePair.idxA, (int)varToIdx.size()).second)
			varToIdx.push_back(scalePair.idxA);
		if (idxToVar.emplace(scalePair.idxB, (int)varToIdx.size()).second)
			varToIdx.push_back(scalePair.idxB);
	}
	const int N = (int)varToIdx.size(); // number of variables (scales)
	const int M = (int)pairwiseScales.size(); // number of equations
	ASSERT(N >= 2);

	uint32_t fixedNode;
	if (!SelectFixedNode(pairwiseScales, fixedIdx, idxToVar, fixedNode))
		return false;

	// The equations split into connected components, which share no unknown: each is solved on its own
	DisjointSet<uint32_t> sets((size_t)N);
	for (const ScalePair& scalePair : pairwiseScales)
		sets.Union((uint32_t)idxToVar.at(scalePair.idxA), (uint32_t)idxToVar.at(scalePair.idxB));
	sets.CompressAllPaths();
	std::vector<uint32_t> componentOfVar;
	const unsigned numComponents = sets.GetComponents(componentOfVar);
	struct Component {
		std::vector<int> vars; // in order of first appearance
		std::vector<int> equations;
	};
	std::vector<Component> components(numComponents);
	for (int v = 0; v < N; ++v)
		components[componentOfVar[v]].vars.push_back(v);
	FOREACH(i, pairwiseScales)
		components[componentOfVar[idxToVar.at(pairwiseScales[i].idxA)]].equations.push_back((int)i);
	const int fixedVar = idxToVar.at(fixedNode);

	// the start of every solve: the previous solve's scales, taken relative to the component's pin, when it
	// had as many indices (a robust round's change of the equations moves the answer a little)
	const bool warmStart = previousScales.size() == numIndices;
	std::vector<REAL> scales(numIndices, REAL(1));
	std::vector<int> unknownOfVar(N, -1);
	Solve stats;
	stats.converged = true;
	for (const Component& component : components) {
		// the pin takes scale 1: the fixed index in the component that holds it, the first named index in any
		// other, which is then shifted to mean-zero log scales (it has no scale to be tied to, and the minimum-norm
		// least squares of the whole system is the mean-zero one)
		const bool holdsFixed = componentOfVar[component.vars.front()] == componentOfVar[fixedVar];
		const int pin = holdsFixed ? fixedVar : component.vars.front();
		int numUnknowns = 0;
		for (const int v : component.vars)
			unknownOfVar[v] = (v == pin ? -1 : numUnknowns++);
		ASSERT(numUnknowns > 0);
		const int numEquations = (int)component.equations.size();
		Eigen::VectorXd b(numEquations);
		std::vector<Eigen::Triplet<double>> triplets;
		triplets.reserve((size_t)numEquations * 2);
		for (int row = 0; row < numEquations; ++row) {
			const ScalePair& scalePair = pairwiseScales[component.equations[row]];
			const double weight = (double)scalePair.weight;
			const int unknownA = unknownOfVar[idxToVar.at(scalePair.idxA)];
			const int unknownB = unknownOfVar[idxToVar.at(scalePair.idxB)];
			if (unknownA >= 0)
				triplets.emplace_back(row, unknownA, -weight);
			if (unknownB >= 0)
				triplets.emplace_back(row, unknownB, weight);
			b(row) = weight * LOGN((double)scalePair.scaleRatio);
		}
		Eigen::SparseMatrix<double> A(numEquations, numUnknowns);
		A.setFromTriplets(triplets.begin(), triplets.end());
		const Eigen::SparseMatrix<double> normal = A.transpose() * A;
		Eigen::VectorXd x0 = Eigen::VectorXd::Zero(numUnknowns);
		if (warmStart) {
			const double logPin = LOGN((double)previousScales[varToIdx[pin]]);
			for (const int v : component.vars)
				if (unknownOfVar[v] >= 0)
					x0(unknownOfVar[v]) = LOGN((double)previousScales[varToIdx[v]]) - logPin;
		}
		// conjugate gradients with the Jacobi preconditioner: the normal matrix of a view graph's pair
		// scales is as dense as an expander and its Cholesky factor many times denser, while a product
		// with it is cheap
		Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
		solver.setTolerance(kTolerance);
		solver.setMaxIterations(kMaxIterations);
		solver.compute(normal);
		Eigen::VectorXd x = solver.solveWithGuess(A.transpose() * b, x0);
		stats.iterations += (unsigned)solver.iterations();
		stats.converged = stats.converged && solver.info() == Eigen::Success;
		if (x.array().isNaN().any()) {
			VERBOSE("warning: scale solve failed");
			return false;
		}
		// one step of refinement against the incidence matrix itself, as the translation solve does: the
		// normal matrix holds the squared condition number's rounding (a weak equation between two groups
		// of strong ones moved the answer by 3e-7), the residual of A and b does not
		solver.setTolerance(kRefinementTolerance);
		x += solver.solve(A.transpose() * (b - A * x));
		stats.refinementIterations += (unsigned)solver.iterations();
		stats.converged = stats.converged && solver.info() == Eigen::Success;
		const double shift = holdsFixed ? 0.0 : x.sum() / (double)component.vars.size();
		for (const int v : component.vars)
			scales[varToIdx[v]] = EXP((v == pin ? 0.0 : x(unknownOfVar[v])) - shift);
	}
	DEBUG_EXTRA("Global scale conjugate gradients: %u unknowns, %u iterations and %u of refinement, %s", (unsigned)(N - (int)numComponents),
		stats.iterations, stats.refinementIterations, stats.converged ? "converged" : "stopped at the iteration bound");
	solve = stats;
	outScales = std::move(scales);
	previousScales = outScales;

	DEBUG("Global scale averaging completed: %u indices, %u pairs (exact gauge, fixed %u)",
		N, M, fixedNode);
	return true;
}
/*----------------------------------------------------------------*/
