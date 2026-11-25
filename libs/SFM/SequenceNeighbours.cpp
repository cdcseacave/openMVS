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

// 19 digits always fit the 64 bits number (nanosecond-epoch stems are 19 digits)
constexpr size_t MAX_NUMBER_DIGITS = 19;

struct ParsedName {
	std::string prefix; // the directory and the stem without its trailing digits
	uint64_t number = 0;
	bool hasNumber = false;
};

ParsedName ParseName(const String& fileName)
{
	const std::string name(fileName.c_str(), fileName.size());
	const size_t dirEnd = name.find_last_of("/\\");
	const size_t nameBegin = (dirEnd == std::string::npos ? 0 : dirEnd + 1);
	const size_t dot = name.find_last_of('.');
	// a leading dot is part of the stem (a hidden file), not an extension
	const size_t stemEnd = (dot == std::string::npos || dot <= nameBegin ? name.size() : dot);
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

	// the usual step of the numbering, to tell a missing stretch from a numbering that just counts by tens: the median rise
	// of each run of one prefix (consecutive names between two prefix changes; a number that does not rise is a break but
	// keeps the run), so clips numbered at different steps each have their own
	std::vector<bool> breaks(numNames-1, false);
	std::vector<size_t> runOf(numNames, 0);
	size_t numRuns = 1;
	for (size_t k = 0; k+1 < numNames; ++k) {
		const ParsedName& a = names[k];
		const ParsedName& b = names[k+1];
		if (a.hasNumber && b.hasNumber && a.prefix != b.prefix) {
			breaks[k] = true;
			++numRuns;
		}
		runOf[k+1] = numRuns-1;
	}
	std::vector<std::vector<double>> risesOfRun(numRuns);
	for (size_t k = 0; k+1 < numNames; ++k) {
		const ParsedName& a = names[k];
		const ParsedName& b = names[k+1];
		if (a.hasNumber && b.hasNumber && a.prefix == b.prefix && b.number > a.number)
			risesOfRun[runOf[k]].push_back(double(b.number-a.number));
	}
	std::vector<double> medianRise(numRuns, 0.0);
	for (size_t run = 0; run < numRuns; ++run) {
		std::vector<double>& rises = risesOfRun[run];
		if (rises.empty())
			continue;
		std::sort(rises.begin(), rises.end());
		const size_t mid = rises.size()/2;
		medianRise[run] = (rises.size()%2 ? rises[mid] : 0.5*(rises[mid-1]+rises[mid]));
	}

	for (size_t k = 0; k+1 < numNames; ++k) {
		const ParsedName& a = names[k];
		const ParsedName& b = names[k+1];
		if (!a.hasNumber || !b.hasNumber || breaks[k])
			continue;
		if (b.number <= a.number)
			breaks[k] = true;
		else if (maxGap > 0 && !risesOfRun[runOf[k]].empty() && double(b.number-a.number) > double(maxGap)*medianRise[runOf[k]])
			breaks[k] = true; // a gap of 0 gives no break by the rises (names that are clock times)
	}
	return breaks;
}


SequenceNeighbours::SequenceNeighbours(const Scene& scene, unsigned _overlap, unsigned maxGap)
	: SequenceNeighbours([&scene, maxGap]() {
		std::vector<String> fileNames;
		fileNames.reserve(scene.images.size());
		for (const Image& image : scene.images)
			fileNames.push_back(image.fileName);
		return ComputeNameBreaks(fileNames, maxGap);
	}(), _overlap)
{
}

SequenceNeighbours::SequenceNeighbours(std::vector<bool> _breaks, unsigned _overlap)
	: breaks(std::move(_breaks)), overlap(_overlap)
{
	breaksBefore.assign(breaks.size()+1, 0);
	for (size_t k = 0; k < breaks.size(); ++k)
		breaksBefore[k+1] = breaksBefore[k] + (breaks[k] ? 1u : 0u);
}

bool SequenceNeighbours::AreNeighbours(IIndex a, IIndex b) const
{
	if (a == b)
		return false;
	const IIndex lo = std::min(a, b), hi = std::max(a, b);
	if (hi-lo > overlap)
		return false;
	// a break lies between them when one of the breaks lo .. hi-1 is set (positions past the breaks have none)
	const size_t first = std::min<size_t>(lo, breaks.size()), last = std::min<size_t>(hi, breaks.size());
	return breaksBefore[last] == breaksBefore[first];
}
