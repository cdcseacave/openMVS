////////////////////////////////////////////////////////////////////
// SequenceNeighbours.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_SEQUENCE_NEIGHBOURS_H_
#define _SFM_SEQUENCE_NEIGHBOURS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API Scene;
struct SFM_API MatchConfig;

// Where an image sequence is interrupted, from its file names in sequence order. Returns n-1 flags (none for n < 2):
// breaks[k] is true when the sequence breaks between the names k and k+1.
// A name (directory and stem; the extension is ignored) splits into a number, its trailing digits (at most 19, else
// none), and a prefix, the rest. Between two names that both have a number there is a break when the prefixes
// differ, when the number does not rise, or, if maxGap > 0, when it rises by more than maxGap times the median rise
// of its run (the names between two prefix changes). A name without a number never breaks.
SFM_API std::vector<bool> ComputeNameBreaks(const std::vector<String>& fileNames, unsigned maxGap);

// Which images of a sequence are neighbours: positions (indices of scene.images) at most `overlap` apart with no
// name break (ComputeNameBreaks) between them. The one definition the matcher and the contradiction cut share.
class SFM_API SequenceNeighbours {
public:
	// breaks[k]: a break between the positions k and k+1 (the positions past the flags have none)
	SequenceNeighbours(const std::vector<bool>& breaks, unsigned overlap);
	// The neighbours of a run that reads the scene's images as a sequence (config.IsImageSequence()): the breaks of
	// their file names at config.sequenceMaxGap, config.matchSequenceOverlap apart. nullopt for any other run.
	static std::optional<SequenceNeighbours> FromConfig(const Scene& scene, const MatchConfig& config);

	// a != b, |a - b| <= overlap, and no break between them
	bool AreNeighbours(IIndex a, IIndex b) const;
	unsigned Overlap() const { return overlap; }
	unsigned NumBreaks() const { return breaksBefore.back(); }

private:
	std::vector<unsigned> breaksBefore; // breaksBefore[k]: the number of breaks between the positions 0 and k
	unsigned overlap;
};

} // namespace SFM

#endif // _SFM_SEQUENCE_NEIGHBOURS_H_
