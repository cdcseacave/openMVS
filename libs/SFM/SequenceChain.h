////////////////////////////////////////////////////////////////////
// SequenceChain.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_SEQUENCE_CHAIN_H_
#define _SFM_SEQUENCE_CHAIN_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "ContradictionCut.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API SequenceNeighbours;
struct SFM_API SequenceFilterStats;

// The chain of an image sequence in the contradiction cut (docs/design/ContradictionCut.md). Terms:
//  - candidate link: a scene pair of neighbours (SequenceNeighbours) that is kept, has a positive composite weight
//    and a baseline with a direction (HasBaselineDirection): the pairs the chain is solved from;
//  - link: a candidate link the chain solved, that is with a scale in the segment both of its images lie in. Only
//    links are protected from the translation stage, and only equations with a link give a judged pair its scale. A
//    candidate link the chain did not solve is an ordinary kept pair: judged by the veto, not protected;
//  - still pair: a pair that meets every condition of a candidate link but the direction of its baseline (a baseline
//    it has: a pose, and both its images in one rotation component): the camera stands still;
//  - segment: one solved scale component of the candidate links, with its own frame: the orientation of the
//    rotations and a scale of its own.

// The camera path of an image sequence as its links place it (SolveSequenceChain).
struct SFM_API SequenceChain {
	std::vector<uint32_t> segmentOf; // per image: its segment, NO_ID where the chain places nothing
	std::vector<Point3> centres;     // per image: its centre in its segment's frame (valid where placed)
	std::vector<REAL> pathLength;    // per image: the path from its segment's first image through the placed images, in order
	std::vector<REAL> localStep;     // per segment: the median distance between consecutive images its links place
	// per scene pair: above 0 exactly for a link; its offset in its segment's frame is linkScale times its world
	// baseline (ComputeWorldBaselines)
	std::vector<REAL> linkScale;
	unsigned numSegments{0};
	unsigned numLinks{0}; // the links
	unsigned numStill{0}; // the still images: placed by a still pair, where the placed image it joins them to stands

	unsigned NumPlaced() const { return (unsigned)(segmentOf.size() - std::count(segmentOf.begin(), segmentOf.end(), NO_ID)); }
};

// Solves the chain: the translation stage's scale and centres solve (SolveScaleComponents) on the candidate links
// alone, weighted by composite weight. `baselines` are ComputeWorldBaselines', `states` the cut's (a candidate link is
// a KEPT pair). Each solved scale component is a segment; an image two components place belongs to the larger, and
// the segments are numbered by their first image.
// Then the still images: an image no segment places, joined by a still pair to a placed image, copies that image's
// segment, centre and path length (the nearest in position, the lower on a tie), pass by pass until a pass places
// none. Still pairs get no scale and change neither the segments, numLinks nor localStep.
SFM_API SequenceChain SolveSequenceChain(const Scene& scene, const std::vector<Point3>& baselines,
	const SequenceNeighbours& neighbours, const std::vector<PairState>& states);

// The veto rule; every length in the frame of the pair's segment. discrepancy: the distance between the pair's offset
// and the chain's (the offset of the chain centres of its two images); pairLength: the length of the pair's baseline;
// chainDistance: the distance of the two chain centres; localStep: the segment's; path: the chain's path between the
// two images. True when discrepancy > max(8 localStep, 0.2 path) and pairLength < chainDistance / 3: a short pair
// claiming that two distant frames stand close. The path term tolerates the chain's drift; a pair as long as the
// chain distance that differs from it in direction is the translation stage's to judge.
SFM_API bool ChainVetoes(REAL discrepancy, REAL pairLength, REAL chainDistance, REAL localStep, REAL path);

// The chain's part of the contradiction cut, between its rotation and translation stages: solves the chain of the
// KEPT pairs, marks its links LINK, and marks VETOED every pair ChainVetoes condemns among the judged ones: the KEPT
// pairs with a baseline of some length whose two images lie in one segment and that share a pair-scale equation
// (ComputePairScales) with a link. A judged pair's scale is the median of the links' scales carried through those
// equations. Fills the segments, links, judged and vetoed pairs of `stats` (numProtected is the translation stage's).
void ApplySequenceChain(const Scene& scene, const std::vector<Point3>& baselines, const SequenceNeighbours& neighbours,
	std::vector<PairState>& states, SequenceFilterStats& stats);

} // namespace SFM

#endif // _SFM_SEQUENCE_CHAIN_H_
