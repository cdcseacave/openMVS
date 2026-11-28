////////////////////////////////////////////////////////////////////
// SequenceNeighbours.cpp
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "SequenceNeighbours.h"
#include "Scene.h"
#include <algorithm>

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

namespace {

// 19 digits always fit the 64 bits number
constexpr size_t MAX_NUMBER_DIGITS = 19;

struct ParsedName {
	std::string prefix; // the directory and the stem without its trailing digits
	uint64_t number = 0;
	bool hasNumber = false;
};

ParsedName ParseName(const String& name)
{
	const size_t dirEnd = name.find_last_of("/\\");
	const size_t nameBegin = (dirEnd == String::npos ? 0 : dirEnd + 1);
	const size_t dot = name.find_last_of('.');
	// a leading dot is part of the stem (a hidden file), not an extension
	const size_t stemEnd = (dot == String::npos || dot <= nameBegin ? name.size() : dot);
	size_t digitsBegin = stemEnd;
	while (digitsBegin > nameBegin && name[digitsBegin-1] >= '0' && name[digitsBegin-1] <= '9')
		--digitsBegin;
	ParsedName parsed;
	parsed.prefix = name.substr(0, digitsBegin);
	const size_t numDigits = stemEnd - digitsBegin;
	if (numDigits > 0 && numDigits <= MAX_NUMBER_DIGITS) {
		parsed.hasNumber = true;
		for (size_t i = digitsBegin; i < stemEnd; ++i)
			parsed.number = parsed.number*10 + uint64_t(name[i]-'0');
	}
	return parsed;
}

} // namespace


// C L A S S ///////////////////////////////////////////////////////

std::vector<bool> SFM::ComputeNameBreaks(const std::vector<String>& fileNames, unsigned maxGap)
{
	const size_t numNames = fileNames.size();
	if (numNames < 2)
		return {};
	std::vector<ParsedName> names;
	names.reserve(numNames);
	for (const String& fileName : fileNames)
		names.push_back(ParseName(fileName));
	const auto numbered = [&names](size_t k) { return names[k].hasNumber && names[k+1].hasNumber; };

	// one run at a time: the names begin..end, up to the next prefix change. Its median rise is the usual step of its
	// numbering, which tells a missing stretch from a numbering that counts by tens
	std::vector<bool> breaks(numNames-1, false);
	for (size_t begin = 0, end; begin+1 < numNames; begin = end+1) {
		DoubleArr rises;
		for (end = begin; end+1 < numNames && !(numbered(end) && names[end].prefix != names[end+1].prefix); ++end)
			if (numbered(end) && names[end+1].number > names[end].number)
				rises.push_back(double(names[end+1].number - names[end].number));
		if (end+1 < numNames)
			breaks[end] = true; // the prefix change
		const double maxRise = (maxGap > 0 && !rises.empty() ? double(maxGap) * rises.GetMedian() : std::numeric_limits<double>::infinity());
		for (size_t k = begin; k < end; ++k)
			if (numbered(k) && (names[k+1].number <= names[k].number || double(names[k+1].number - names[k].number) > maxRise))
				breaks[k] = true;
	}
	return breaks;
}


SequenceNeighbours::SequenceNeighbours(const std::vector<bool>& breaks, unsigned _overlap)
	: breaksBefore(breaks.size()+1, 0), overlap(_overlap)
{
	for (size_t k = 0; k < breaks.size(); ++k)
		breaksBefore[k+1] = breaksBefore[k] + (breaks[k] ? 1u : 0u);
}

std::optional<SequenceNeighbours> SequenceNeighbours::FromConfig(const Scene& scene, const MatchConfig& config)
{
	if (!config.IsImageSequence())
		return std::nullopt;
	std::vector<String> fileNames;
	fileNames.reserve(scene.images.size());
	for (const Image& image : scene.images)
		fileNames.push_back(image.fileName);
	return SequenceNeighbours(ComputeNameBreaks(fileNames, config.sequenceMaxGap), config.matchSequenceOverlap);
}

bool SequenceNeighbours::AreNeighbours(IIndex a, IIndex b) const
{
	if (a == b)
		return false;
	const IIndex lo = std::min(a, b), hi = std::max(a, b);
	if (hi-lo > overlap)
		return false;
	// a break lies between them when one of the breaks lo .. hi-1 is set
	const size_t numBreaks = breaksBefore.size()-1;
	return breaksBefore[std::min<size_t>(hi, numBreaks)] == breaksBefore[std::min<size_t>(lo, numBreaks)];
}
