////////////////////////////////////////////////////////////////////
// ContradictionCut.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_CONTRADICTION_CUT_H_
#define _SFM_CONTRADICTION_CUT_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API Scene;
class SFM_API ImagePair;
struct SFM_API ScalePair;

// The internals of the contradiction cut (FilterPairsByContradiction, GlobalRotationAveraging.h): what its stages
// share with the chain of an image sequence (SequenceChain.h). Not part of the library's interface (SFM.h).

// The state of a scene pair in the cut
enum class PairState : uint8_t {
	KEPT,   // kept
	LINK,   // kept, and a link of the sequence chain (SequenceChain.h): the translation stage's verdict leaves it
	CUT,    // removed by the rotation or the translation stage: what the repair is offered
	VETOED  // removed by the sequence chain: never repaired, joins no spot, vouches for no pair
};
inline bool IsRemoved(PairState state) { return state == PairState::CUT || state == PairState::VETOED; }

// The baseline of every scene pair in the world frame, at its pose's own length: RMatrix(rotations[ID1]).t() *
// relativePose->C. Point3::INF for a pair without a pose or whose two images do not lie in one rotation component
// (componentOf: per image, NO_ID where not rotated): no stage after the rotations reads such a pair.
SFM_API std::vector<Point3> ComputeWorldBaselines(const Scene& scene, const std::vector<Point3>& rotations,
	const std::vector<uint32_t>& componentOf);
// What the stages ask of a pair's world baseline: is there one; has it a length; has it a direction (a length, and a
// mean ray angle of at least kNearDuplicateRayAngle: under it the direction is noise)
bool HasBaseline(const Point3& baseline);
bool HasBaselineLength(const Point3& baseline);
bool HasBaselineDirection(const ImagePair& pair, const Point3& baseline);

// One solved component of the pair-scale graph (SolveScaleComponents)
struct ScaleComponentSolve {
	std::vector<uint32_t> pairs;       // its scene pairs, ascending
	std::vector<REAL> scales;          // per pair of `pairs`: C_2 - C_1 = scale * baseline
	std::vector<Point3> centres;       // per image: its camera centre (valid where `placed`)
	std::vector<REAL> centreResiduals; // per pair of `pairs`: the distance of its scaled baseline from the centres' offset
	std::vector<bool> placed;          // per image: an image of one of `pairs`
};
// The scale + centres solve of the translation stage, which the chain of a sequence runs on its candidate links: the
// components of the graph of the given pair-scale equations (ComputePairScales), largest first (by pairs, ties to the
// lower pair), and per component of at least minScaleComponent pairs (ContradictionCut.cpp) the scales by
// RobustAverage (Geman-McClure) around GlobalScaleEstimator, then the centres from C_2 - C_1 = scale * baseline
// (baselines and weights: per scene pair; a pair of an equation has a baseline with a length) by
// GlobalTranslationEstimator reweighted towards the sum of norms (convex, so no start decides). Returns the solved
// components in that order (one whose scale or centres solve fails is left out); numComponents: all the components
// of the graph.
std::vector<ScaleComponentSolve> SolveScaleComponents(const Scene& scene, const std::vector<ScalePair>& equations,
	const std::vector<Point3>& baselines, const std::vector<float>& weights, unsigned& numComponents);

} // namespace SFM

#endif // _SFM_CONTRADICTION_CUT_H_
