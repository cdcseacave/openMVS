/*
 * DenseMatcher.h
 *
 * Copyright (c) 2014-2026 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#ifndef _SFM_DENSEMATCHER_H_
#define _SFM_DENSEMATCHER_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "OnnxRuntime.h"
#include "ROMA2Warp.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

struct SFM_API ROMA2Config;

// What a dense matcher is told about the two images of a pair besides their descriptors: their pixel sizes (no model
// reads a camera: RoMa2 has no camera input and FlexGlue's pose head H1 reads none)
struct SFM_API PairInput {
	cv::Size sizeA, sizeB;
};

// What a dense matcher returns for one pair. The warps (both directions, on the matcher's grid) are the one
// output every matcher has, the common currency the verdict, the guided matching and the dense fill read; the rest
// is absent (std::optional) where a matcher does not produce it: the covisibility score in [0,1] and a pose prior.
struct SFM_API PairOutput : PairWarps {
	std::optional<float> covisibility;
	std::optional<PosePrior> pose;
};
/*----------------------------------------------------------------*/

// One dense matcher model (RoMa2, FlexGlue), loaded once and driven from a single thread: every image is described once
// into a descriptor tensor (`code`, resident on the device where the provider keeps tensors there) and a global
// retrieval descriptor, then two codes are matched into a PairOutput. The passes that describe the images, plan the
// descriptor slots, prefetch the images and feed the pairs to the thread pool (MatchROMA2.cpp) only need this interface.
class SFM_API DenseMatcher
{
public:
	virtual ~DenseMatcher() = default;

	// Whether this build can load a dense matcher (it has ONNX Runtime support)
	static bool IsAvailable();
	// A new, not yet loaded matcher of the kind config.matcher names ("roma2" or "flexglue"); needs IsAvailable()
	static std::unique_ptr<DenseMatcher> Create(const ROMA2Config& config);
	// The size of the global retrieval descriptors the matcher config names will produce, known before it is loaded
	// (FlexGlue's is fixed, RoMa2's is its manifest's); 0 where it cannot be told (no model folder or no manifest in it)
	static unsigned ExpectedRetrievalSize(const ROMA2Config& config);

	// Load the model from its folder ("setting" is the model's preset where it has several) on the provider
	// ("auto"|"cuda"|"coreml"|"dml"|"cpu"); false with a message on any failure
	virtual bool Load(const String& modelDir, const String& setting, const String& provider) = 0;
	virtual bool IsLoaded() const = 0;
	virtual String ProviderName() const = 0;

	// Side of the square the images are squashed to, and the planar RGB floats Preprocess makes of one image
	virtual int ImageSize() const = 0;
	virtual void Preprocess(const Image8U3& bgr, std::vector<float>& planarRgb) const = 0;
	// Cells per side of the warp grid a MatchPair returns
	virtual int GridSize() const = 0;
	// Size of the global retrieval descriptor DescribeImage returns
	virtual unsigned RetrievalSize() const = 0;

	// One image's descriptor tensor
	virtual OrtTensor MakeCode() = 0;
	// Describe a preprocessed image into `code`; `retrieval` receives the image's unit global retrieval descriptor
	virtual bool DescribeImage(const float* planarRgb, OrtTensor& code, std::vector<float>& retrieval) = 0;
	// Match the pair of codes (A, B)
	virtual bool MatchPair(const OrtTensor& codeA, const OrtTensor& codeB, const PairInput& input, PairOutput& out) = 0;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_DENSEMATCHER_H_
