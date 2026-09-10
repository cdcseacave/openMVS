/*
 * RobustAveraging.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_ROBUST_AVERAGING_H_
#define _SFM_ROBUST_AVERAGING_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Iteratively reweighted averaging around any weighted least-squares solve over pairs that carry a
// float `weight`:
//  solve(activePairs, solution) -> bool   the plain solver over the pairs with weight > 0
//  residual(i, solution) -> REAL          pair i's signed error, in the units of sigma
// Geman-McClure weights w = (sigma^2 / (sigma^2 + r^2))^2 are recomputed every round from each
// pair's initial weight and the current solution; a pair under 1e-3 of its initial weight sits the
// next round out, never permanently. Stops after maxRounds or when no weight moved by more than
// 1e-3 of its initial value. Returns false when the first solve fails; a later failure keeps the
// previous solution. residuals[i] = |residual(i, solution)| against the returned solution.
template <typename Pair, typename Solution, typename SolveFn, typename ResidualFn>
bool RobustAverage(const std::vector<Pair>& pairs, REAL sigma, unsigned maxRounds, SolveFn solve, ResidualFn residual, Solution& solution, std::vector<REAL>& residuals)
{
	if (pairs.empty())
		return false;
	std::vector<Pair> weighted(pairs);
	residuals.assign(pairs.size(), REAL(0));
	const REAL sigma2 = sigma * sigma;
	for (unsigned round = 0; ; ++round) {
		std::vector<Pair> active;
		active.reserve(weighted.size());
		for (const Pair& p : weighted)
			if (p.weight > 0)
				active.push_back(p);
		Solution current;
		if (active.empty() || !solve(active, current)) {
			if (round == 0)
				return false;
			break; // keep the previous round's solution and residuals
		}
		solution = std::move(current);
		// every pair's weight for the next round comes from its initial weight and this solution
		REAL maxChange = 0;
		for (size_t i = 0; i < pairs.size(); ++i) {
			const REAL r = residuals[i] = ABS(residual(i, solution));
			const float w = pairs[i].weight * (float)SQUARE(sigma2 / (sigma2 + r * r));
			maxChange = MAXF(maxChange, REAL(ABS(w - weighted[i].weight)) / pairs[i].weight);
			weighted[i].weight = (w < 1e-3f * pairs[i].weight ? 0.f : w);
		}
		if (round + 1 >= maxRounds || maxChange < REAL(1e-3))
			break;
	}
	return true;
}
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_ROBUST_AVERAGING_H_
