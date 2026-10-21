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
 * Estimates global scales by solving a weighted least-squares system:
 *   log(s_j) - log(s_i) ≈ log(scale_ratio_ij)
 *
 * This is solved via SVD decomposition with exact gauge enforcement by
 * eliminating one fixed-scale variable (scale = 1.0). Systems of many indices and
 * equations (the scales of every pair of a view graph) take the sparse route instead:
 * conjugate gradients on the normal equations, started from the previous sparse solve
 * of the same estimator.
 *
 * Generalized from StarInitializer::EstimateGlobalScale() to work with
 * arbitrary indices (sub-scenes, images, etc.) instead of image pairs.
 */
class SFM_API GlobalScaleEstimator
{
public:
	/**
	 * @brief Estimate global scales from pairwise ratios
	 * @param pairwiseScales Vector of pairwise scale ratios
	 * @param numIndices Total number of indices (scenes/images)
	 * @param fixedIdx Optional fixed index to set exact scale=1 (NO_ID = auto-select best-connected index)
	 * @param outScales Output vector of global scales (indexed by scene/image ID)
	 * @param sparse Solve by conjugate gradients on the normal equations (with one step of refinement
	 *        against the equations themselves) instead of the dense SVD, started from the scales of this
	 *        estimator's previous sparse solve when it had as many indices (GetSolve() reports the
	 *        iterations); the dense route stays the default. The same least-squares solution when the
	 *        pairs join all indices in one component; a component the fixed index does not reach is
	 *        solved up to a ridge, not as the SVD's minimum-norm answer.
	 * @return true if estimation successful
	 */
	bool EstimateScales(
		const std::vector<ScalePair>& pairwiseScales,
		const uint32_t numIndices,
		const uint32_t fixedIdx,
		std::vector<REAL>& outScales,
		bool sparse = false);

	bool EstimateScales(
		const std::vector<ScalePair>& pairwiseScales,
		const uint32_t numIndices,
		std::vector<REAL>& outScales) {
		return EstimateScales(pairwiseScales, numIndices, NO_ID, outScales);
	}

	/// the last sparse solve: the conjugate gradients iterations it took from its start (and those of the
	/// refinement of its answer), and whether both reached the tolerance, not the iteration bound
	struct Solve {
		unsigned iterations = 0;
		unsigned refinementIterations = 0;
		bool converged = false;
	};
	const Solve& GetSolve() const { return solve; }

private:
	Solve solve;
	std::vector<REAL> previousScales; // the scales of the last sparse solve, the start of the next
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBAL_SCALE_AVERAGING_H_
