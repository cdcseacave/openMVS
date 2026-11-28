/*
 * GlobalScaleAveraging.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_GLOBAL_SCALE_AVERAGING_H_
#define _SFM_GLOBAL_SCALE_AVERAGING_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

/**
 * @brief Pairwise scale ratio between two sub-scenes or images
 */
struct SFM_API ScalePair
{
	uint32_t idxA;      // First index (sub-scene or image)
	uint32_t idxB;      // Second index (sub-scene or image)
	REAL scaleRatio;    // scale_B / scale_A
	float weight;       // confidence weight (e.g., number of correspondences)

	ScalePair() : idxA(NO_ID), idxB(NO_ID), scaleRatio(REAL(1)), weight(0.f) {}
	ScalePair(uint32_t a, uint32_t b, REAL ratio, float w)
		: idxA(a), idxB(b), scaleRatio(ratio), weight(w) {}
};

/**
 * @brief Global scale estimation from pairwise scale ratios
 *
 * Solves log(s_j) - log(s_i) ~= log(scale_ratio_ij) per connected component using
 * Jacobi-preconditioned conjugate gradients on the normal equations with one step of
 * iterative refinement. Pins fixedIdx (or the best-connected node) to scale 1 in its
 * component, shifts unpinned components to zero-mean log scale, and leaves unreferenced
 * indices at scale 1. Warm-starts from the previous solve when numIndices matches.
 */
class SFM_API GlobalScaleEstimator
{
public:
	/**
	 * @brief Estimate global scales from pairwise ratios
	 * @param pairwiseScales Vector of pairwise scale ratios
	 * @param numIndices Total number of indices (scenes/images)
	 * @param fixedIdx Optional fixed index to set exact scale=1 (NO_ID = auto-select best-connected index)
	 * @param outScales Output vector of global scales (indexed by scene/image ID), replaced, not
	 *        resized: scale 1 at every index no equation names
	 * @return true if estimation successful
	 */
	bool EstimateScales(
		const std::vector<ScalePair>& pairwiseScales,
		const uint32_t numIndices,
		const uint32_t fixedIdx,
		std::vector<REAL>& outScales);

	bool EstimateScales(
		const std::vector<ScalePair>& pairwiseScales,
		const uint32_t numIndices,
		std::vector<REAL>& outScales) {
		return EstimateScales(pairwiseScales, numIndices, NO_ID, outScales);
	}

	/// the last solve: the conjugate gradients iterations it took from their start, summed over the components
	/// (and those of the refinement of their answers), and whether all reached the tolerance, not the iteration bound
	struct Solve {
		unsigned iterations = 0;
		unsigned refinementIterations = 0;
		bool converged = false;
	};
	const Solve& GetSolve() const { return solve; }

private:
	Solve solve;
	std::vector<REAL> previousScales; // the scales of the last solve, the start of the next
};

// Geman-McClure iteratively reweighted global scale averaging on log-scale residuals
SFM_API bool RobustAverageScales(
	const std::vector<ScalePair>& pairs,
	uint32_t numIndices,
	uint32_t fixedIdx,
	REAL maxLogResidual,
	unsigned maxRounds,
	std::vector<REAL>& outScales,
	std::vector<REAL>& outResiduals);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBAL_SCALE_AVERAGING_H_
