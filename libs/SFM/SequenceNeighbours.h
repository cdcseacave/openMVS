////////////////////////////////////////////////////////////////////
// SequenceNeighbours.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _SFM_SEQUENCE_NEIGHBOURS_H_
#define _SFM_SEQUENCE_NEIGHBOURS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "Image.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

class SFM_API Scene;

// The name breaks of an image sequence: breaks[k] is true when a break lies between the positions k and k+1
// (the size is max(0, n-1)). A name is the directory and the stem of a file name (the extension is ignored);
// its number is the trailing decimal digits of the stem (none, or more than 19 digits: no number) and its
// prefix is the rest of the name. A break lies between two names that both have a number when the prefixes
// differ, or the number does not rise, or it rises by more than maxGap times the median rise of its run: a run
// is the names between two prefix changes (a number that does not rise breaks but keeps the run), its median
// over its adjacent pairs that share a prefix and whose number rises (with no such pair the run has no gap
// rule). A maxGap of 0 gives no break by the rises (names that are clock times, IMG_20261125_103059, rise by
// 41 or more at every minute rollover against a median of 1; the prefix and not-rising rules still apply).
// A name without a number makes no break with either neighbour.
SFM_API std::vector<bool> ComputeNameBreaks(const std::vector<String>& fileNames, unsigned maxGap);

// The positions that are neighbours in a sequence of images: at most overlap positions apart with no name break
// between them. Everything that reads a capture as a sequence (the matcher's candidates, the view graph
// filter) shares this definition.
class SFM_API SequenceNeighbours {
public:
	// positions are the indices of scene.images, their names the images' fileName
	SequenceNeighbours(const Scene& scene, unsigned overlap, unsigned maxGap);
	// explicit breaks (tests)
	SequenceNeighbours(std::vector<bool> breaks, unsigned overlap);

	// a != b, |a - b| <= overlap, and no break between them
	bool AreNeighbours(IIndex a, IIndex b) const;
	unsigned Overlap() const { return overlap; }
	const std::vector<bool>& Breaks() const { return breaks; }

private:
	std::vector<bool> breaks;
	std::vector<unsigned> breaksBefore; // breaksBefore[k]: the number of breaks between positions 0 and k
	unsigned overlap;
};

} // namespace SFM

#endif // _SFM_SEQUENCE_NEIGHBOURS_H_
