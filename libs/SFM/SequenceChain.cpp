////////////////////////////////////////////////////////////////////
// SequenceChain.cpp
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "SequenceChain.h"
#include "Scene.h"
#include "SequenceNeighbours.h"
#include "GlobalRotationAveraging.h"
#include "GlobalScaleAveraging.h"
#include "StarInitializer.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("SeqChain"));

namespace {

// The bars of the veto (ChainVetoes)
constexpr REAL kVetoLocalSteps = 8;    // local steps a pair may be off the chain
constexpr REAL kVetoPathShare = 0.2;   // share of the chain's path between its images a pair may be off the chain
constexpr REAL kVetoShortDivisor = 3;  // a pair under the chain distance over this is short

// Numbers the segments by their first image, so they follow the sequence
void NumberSegmentsByFirstImage(SequenceChain& chain)
{
	std::vector<uint32_t> numberOf(chain.numSegments, NO_ID);
	uint32_t numNumbered = 0;
	for (uint32_t& segment : chain.segmentOf) {
		if (segment == NO_ID)
			continue;
		if (numberOf[segment] == NO_ID)
			numberOf[segment] = numNumbered++;
		segment = numberOf[segment];
	}
	ASSERT(numNumbered == chain.numSegments); // a segment owns an image
}

// Computes the path along each segment through its placed images in order, and its local step (the upper median of
// the path's steps)
void ComputeChainPaths(SequenceChain& chain)
{
	std::vector<uint32_t> lastOf(chain.numSegments, NO_ID);
	std::vector<std::vector<REAL>> stepsOf(chain.numSegments);
	FOREACH(i, chain.segmentOf) {
		const uint32_t segment = chain.segmentOf[i];
		if (segment == NO_ID)
			continue;
		const uint32_t last = lastOf[segment];
		if (last != NO_ID) {
			stepsOf[segment].push_back(norm(chain.centres[i] - chain.centres[last]));
			chain.pathLength[i] = chain.pathLength[last] + stepsOf[segment].back();
		}
		lastOf[segment] = (uint32_t)i;
	}
	chain.localStep.assign(chain.numSegments, REAL(0));
	FOREACH(segment, stepsOf) {
		std::vector<REAL>& steps = stepsOf[segment];
		if (steps.empty())
			continue;
		std::nth_element(steps.begin(), steps.begin() + steps.size() / 2, steps.end());
		chain.localStep[segment] = steps[steps.size() / 2];
	}
}

// Places the still images (SolveSequenceChain). A pass reads only the images placed before it, and an image takes
// the nearest placed image its still pairs join it to, so the order of the scene's pairs decides nothing
void PlaceStillImages(const Scene& scene, const IIndexArr& stillPairs, SequenceChain& chain)
{
	std::vector<uint32_t> sourceOf(chain.segmentOf.size(), NO_ID); // per still image: the image it stands at
	std::vector<uint32_t> placedNow;
	do {
		placedNow.clear();
		for (const IIndex idxPair : stillPairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			const bool placed1 = chain.segmentOf[pair.ID1] != NO_ID;
			if (placed1 == (chain.segmentOf[pair.ID2] != NO_ID))
				continue;
			const uint32_t image = placed1 ? pair.ID2 : pair.ID1, source = placed1 ? pair.ID1 : pair.ID2;
			const auto distanceTo = [image](uint32_t other) { return other > image ? other - image : image - other; };
			uint32_t& best = sourceOf[image];
			if (best == NO_ID)
				placedNow.push_back(image);
			else if (distanceTo(source) > distanceTo(best) || (distanceTo(source) == distanceTo(best) && source > best))
				continue;
			best = source;
		}
		for (const uint32_t image : placedNow) {
			const uint32_t source = sourceOf[image];
			chain.segmentOf[image] = chain.segmentOf[source];
			chain.centres[image] = chain.centres[source];
			chain.pathLength[image] = chain.pathLength[source];
		}
		chain.numStill += (unsigned)placedNow.size();
	} while (!placedNow.empty());
}

// the name of an image in the log line of a vetoed pair, which is read token by token: its file name without
// directory and extension, a white space in it replaced, and the image's index when it has no name
String ChainImageName(const Scene& scene, IIndex idxImage)
{
	String name = Util::getFileName(scene.images[idxImage].fileName);
	if (name.empty())
		return String::FormatString("%u", idxImage);
	std::replace_if(name.begin(), name.end(), [](char c) { return isspace((unsigned char)c) != 0; }, '_');
	return name;
}

// The veto (ApplySequenceChain): judges against the chain the pairs that share an equation with a link, marks the
// vetoed ones and counts both
void VetoPairsByChain(const Scene& scene, const std::vector<Point3>& baselines, const SequenceChain& chain,
	std::vector<PairState>& states, SequenceFilterStats& stats)
{
	TD_TIMER_START();
	const size_t numPairs = scene.pairs.size();
	ASSERT(baselines.size() == numPairs && states.size() == numPairs && chain.linkScale.size() == numPairs);
	ASSERT(chain.segmentOf.size() == scene.images.size());
	// the links and the pairs to judge, in the order of the scene's pairs
	std::vector<bool> isLink(numPairs, false);
	IIndexArr pairIndices;
	bool anyToJudge = false;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		isLink[idxPair] = states[idxPair] == PairState::LINK;
		// the rule reads a pair's length, not its direction: a pair under the near-duplicate ray angle is judged too
		const bool toJudge = states[idxPair] == PairState::KEPT && HasBaselineLength(baselines[idxPair]) &&
			chain.segmentOf[pair.ID1] != NO_ID && chain.segmentOf[pair.ID1] == chain.segmentOf[pair.ID2];
		if (isLink[idxPair] || toJudge)
			pairIndices.push_back((IIndex)idxPair);
		anyToJudge = anyToJudge || toJudge;
	}
	// the scale each equation with a link gives its other pair, sorted by pair, then by scale. An equation is the
	// relation the scale solve fits (SolveScaleComponents): s[idxB] = s[idxA] * scaleRatio
	std::vector<std::pair<uint32_t, REAL>> scales;
	if (anyToJudge) {
		for (const ScalePair& equation : ComputePairScales(scene, pairIndices, &isLink)) {
			const bool linkFirst = isLink[equation.idxA];
			const uint32_t idxLink = linkFirst ? equation.idxA : equation.idxB;
			const uint32_t idxJudged = linkFirst ? equation.idxB : equation.idxA;
			// an equation joins a link, which has a scale, and a pair that is none; the two share an image, and a
			// link lies in the segment of its images
			ASSERT(isLink[equation.idxA] != isLink[equation.idxB] && chain.linkScale[idxLink] > 0 && equation.scaleRatio > 0);
			ASSERT(chain.segmentOf[scene.pairs[idxLink].ID1] == chain.segmentOf[scene.pairs[idxJudged].ID1]);
			scales.emplace_back(idxJudged, linkFirst ? chain.linkScale[idxLink] * equation.scaleRatio : chain.linkScale[idxLink] / equation.scaleRatio);
		}
		std::sort(scales.begin(), scales.end());
	}
	for (size_t begin = 0, end = 0; begin < scales.size(); begin = end) {
		const uint32_t idxPair = scales[begin].first;
		while (end < scales.size() && scales[end].first == idxPair)
			++end;
		// the pair's scale is the median over its equations: its length and offset in the segment's frame
		const REAL scale = scales[begin + (end - begin) / 2].second;
		const ImagePair& pair = scene.pairs[idxPair];
		const Point3 chainOffset = chain.centres[pair.ID2] - chain.centres[pair.ID1];
		const REAL discrepancy = norm(baselines[idxPair] * scale - chainOffset);
		const REAL pairLength = scale * norm(baselines[idxPair]);
		const REAL chainDistance = norm(chainOffset);
		const REAL path = ABS(chain.pathLength[pair.ID2] - chain.pathLength[pair.ID1]);
		++stats.numJudged;
		if (!ChainVetoes(discrepancy, pairLength, chainDistance, chain.localStep[chain.segmentOf[pair.ID1]], path))
			continue;
		// one line per vetoed pair only above verbosity 3: each line syncs the log to disk
		DEBUG_LEVEL(3, "Vetoed pair: %s %s length %.9g chain %.9g path %.9g discrepancy %.9g equations %u",
			ChainImageName(scene, pair.ID1).c_str(), ChainImageName(scene, pair.ID2).c_str(), pairLength, chainDistance, path, discrepancy,
			(unsigned)(end - begin));
		states[idxPair] = PairState::VETOED;
		++stats.numVetoed;
	}
	VERBOSE("Sequence chain veto: removed %u of %u judged pairs (%s)", stats.numVetoed, stats.numJudged, TD_TIMER_GET_FMT().c_str());
}

} // namespace
/*----------------------------------------------------------------*/

SequenceChain SFM::SolveSequenceChain(const Scene& scene, const std::vector<Point3>& baselines,
	const SequenceNeighbours& neighbours, const std::vector<PairState>& states)
{
	TD_TIMER_START();
	const uint32_t numPairs = (uint32_t)scene.pairs.size();
	const uint32_t numImages = (uint32_t)scene.images.size();
	ASSERT(baselines.size() == numPairs && states.size() == numPairs);
	SequenceChain chain;
	chain.segmentOf.assign(numImages, NO_ID);
	chain.centres.assign(numImages, Point3::ZERO);
	chain.pathLength.assign(numImages, REAL(0));
	chain.linkScale.assign(numPairs, REAL(0));
	// the candidate links, with their weight, and the still pairs
	IIndexArr candidateLinks, stillPairs;
	std::vector<float> weights(numPairs, 0.f);
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (states[idxPair] != PairState::KEPT || !HasBaseline(baselines[idxPair]) || !pair.HasValidWeight() ||
			!neighbours.AreNeighbours(pair.ID1, pair.ID2))
			continue;
		if (!HasBaselineDirection(pair, baselines[idxPair])) {
			stillPairs.push_back((IIndex)idxPair);
			continue;
		}
		candidateLinks.push_back((IIndex)idxPair);
		weights[idxPair] = pair.GetCompositeWeight();
	}
	if (candidateLinks.empty()) {
		VERBOSE("Sequence chain solve: no link among the neighbour pairs (%s)", TD_TIMER_GET_FMT().c_str());
		return chain;
	}

	// one segment per solved scale component, the largest first: an image belongs to the first that places it, and a
	// candidate link is a link, with its solved scale, only when both its images belong to the component that solved it
	const std::vector<ScalePair> equations = ComputePairScales(scene, candidateLinks);
	unsigned numComponents;
	for (const ScaleComponentSolve& solve : SolveScaleComponents(scene, equations, baselines, weights, numComponents)) {
		const uint32_t segment = chain.numSegments;
		bool owns = false;
		for (uint32_t i = 0; i < numImages; ++i) {
			if (!solve.placed[i] || chain.segmentOf[i] != NO_ID)
				continue;
			chain.segmentOf[i] = segment;
			chain.centres[i] = solve.centres[i];
			owns = true;
		}
		if (!owns)
			continue; // every image it places belongs to a larger component: no segment
		++chain.numSegments;
		FOREACH(k, solve.pairs) {
			const uint32_t idxPair = solve.pairs[k];
			const ImagePair& pair = scene.pairs[idxPair];
			if (chain.segmentOf[pair.ID1] != segment || chain.segmentOf[pair.ID2] != segment)
				continue;
			chain.linkScale[idxPair] = solve.scales[k];
			++chain.numLinks;
		}
	}
	NumberSegmentsByFirstImage(chain);
	ComputeChainPaths(chain);
	const unsigned numPlacedByLinks = chain.NumPlaced();
	PlaceStillImages(scene, stillPairs, chain);
	VERBOSE("Sequence chain solve: %u segment(s) placing %u images by %u links of %u candidates (%u scale components, %u equations), %u still image(s) (%s)",
		chain.numSegments, numPlacedByLinks, chain.numLinks, (unsigned)candidateLinks.size(), numComponents, (unsigned)equations.size(),
		chain.numStill, TD_TIMER_GET_FMT().c_str());
	return chain;
}
/*----------------------------------------------------------------*/

bool SFM::ChainVetoes(REAL discrepancy, REAL pairLength, REAL chainDistance, REAL localStep, REAL path)
{
	return discrepancy > MAXF(kVetoLocalSteps * localStep, kVetoPathShare * path) && pairLength < chainDistance / kVetoShortDivisor;
}

void SFM::ApplySequenceChain(const Scene& scene, const std::vector<Point3>& baselines, const SequenceNeighbours& neighbours,
	std::vector<PairState>& states, SequenceFilterStats& stats)
{
	const SequenceChain chain = SolveSequenceChain(scene, baselines, neighbours, states);
	stats.numSegments = chain.numSegments;
	stats.numLinks = chain.numLinks;
	FOREACH(idxPair, scene.pairs)
		if (chain.linkScale[idxPair] > 0)
			states[idxPair] = PairState::LINK;
	VetoPairsByChain(scene, baselines, chain, states, stats);
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
