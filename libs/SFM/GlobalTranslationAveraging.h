/*
 * GlobalTranslationAveraging.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_GLOBAL_TRANSLATION_AVERAGING_H_
#define _SFM_GLOBAL_TRANSLATION_AVERAGING_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

/**
 * @brief Pairwise relative translation between two sub-scenes
 */
struct SFM_API TranslationPair
{
	uint32_t idxA;            // First index (sub-scene or image)
	uint32_t idxB;            // Second index (sub-scene or image)
	Point3 relativeTranslation; // t_B - t_A (after rotation/scale alignment)
	float weight;             // confidence weight (e.g., number of inliers)

	TranslationPair() : idxA(NO_ID), idxB(NO_ID), relativeTranslation(Point3::ZERO), weight(0.f) {}
	TranslationPair(uint32_t a, uint32_t b, const Point3& t, float w)
		: idxA(a), idxB(b), relativeTranslation(t), weight(w) {}
};

/**
 * @brief Global translation estimation from pairwise relative translations
 *
 * Estimates global translations by solving a weighted linear least-squares system:
 *   t_j - t_i = relative_translation_ij
 *
 * The system is solved using Eigen's sparse linear solvers (QR or LU decomposition).
 * Gauge freedom is resolved by pinning the node the caller names, and the best-connected
 * node of the pairs when it names none.
 */
class SFM_API GlobalTranslationEstimator
{
public:
	/**
	 * @brief Estimate global translations from pairwise relative translations
	 * @param pairwiseTranslations Vector of pairwise translation constraints
	 * @param numIndices Total number of indices (scenes/images)
	 * @param fixedIdx Fixed gauge index (NO_ID = auto-select best-connected index)
	 * @param outTranslations Output vector of global translations (indexed by scene/image ID)
	 * @return true if estimation successful
	 */
	bool EstimateTranslations(
		const std::vector<TranslationPair>& pairwiseTranslations,
		const uint32_t numIndices,
		const uint32_t fixedIdx,
		std::vector<Point3>& outTranslations);

	bool EstimateTranslations(
		const std::vector<TranslationPair>& pairwiseTranslations,
		const uint32_t numIndices,
		std::vector<Point3>& outTranslations) {
		return EstimateTranslations(pairwiseTranslations, numIndices, NO_ID, outTranslations);
	}
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_GLOBAL_TRANSLATION_AVERAGING_H_
