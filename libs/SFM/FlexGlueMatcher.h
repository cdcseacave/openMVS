/*
 * FlexGlueMatcher.h
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



#ifndef _SFM_FLEXGLUEMATCHER_H_
#define _SFM_FLEXGLUEMATCHER_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "OnnxRuntime.h"
#include "DenseMatcher.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// The manifest of a FlexGlue model bundle, flexglue_<setting>.json (docs/DenseMatcherModels.md): the two graph files and the sizes the
// graphs are built for, checked against the shapes this build reads
struct SFM_API FlexGlueManifest
{
	String setting;          // preset name (one today: base)
	int imageSize = 0;       // S: the descriptor graph takes S x S images
	int warpSize = 0;        // the match graph's warp grid side
	unsigned retrievalDim = 0; // the descriptor graph's retrieval output
	String descriptorFile;   // descriptor graph: image -> features, retrieval
	String matchFile;        // match graph: two feature maps -> warps, confidences, covisibility, the H1 pose

	// Parse the manifest; false with a message naming the file and the key on a missing file, another format version or model,
	// a missing or ill-typed key, or a bundle without the pose head
	bool Load(const String& fileName);
};
/*----------------------------------------------------------------*/

// FlexGlue (a DINOv3 ConvNeXt-B backbone and a transformer matcher) as a DenseMatcher, from the model bundle the export script writes
// (docs/design/DenseMatching.md): the manifest flexglue_<setting>.json and its two graphs, the descriptor graph (the image squashed to
// 512 x 512 -> the stride-8 feature map, 256 x 64 x 64, and the unit 256-D retrieval descriptor, that map GeM-pooled) and the match graph
// (two feature maps -> both warps on a 128 x 128 grid with their confidences gated by FlexGlue's own checks, the covisibility score
// "do the two images show the same place", and the H1 pose head's coarse relative rotation and baseline direction, read from the
// pair's layer-9 features in the same pass, with no camera).
// Both sessions run on the provider the descriptor got. Same threading and lifetime rules as RoMa2Onnx.
#ifdef _USE_ONNXRUNTIME
class SFM_API FlexGlueMatcher : public DenseMatcher
{
public:
	static constexpr int IMAGE_SIZE = 512;           // the descriptor's input side
	static constexpr int FEATURE_CHANNELS = 256;     // feature map: FEATURE_CHANNELS x GRID x GRID
	static constexpr int GRID = IMAGE_SIZE / 8;
	static constexpr int WARP_GRID = IMAGE_SIZE / 4; // the match graph's grid (coarse stride 4)
	static constexpr int RETRIEVAL_SIZE = 256;       // the unit global descriptor: the feature map GeM-pooled (p = 3)

	// Load the bundle of modelDir (its manifest flexglue_<setting>.json and the graphs it names) on the provider
	// ("auto"|"cuda"|"coreml"|"dml"|"cpu")
	bool Load(const String& modelDir, const String& setting, const String& provider) override;
	bool IsLoaded() const override { return descriptor.IsLoaded() && match.IsLoaded(); }
	String ProviderName() const override;

	int ImageSize() const override { return IMAGE_SIZE; }
	// float pixels in [0,1] squashed to IMAGE_SIZE x IMAGE_SIZE, bilinear (half-pixel centres, no antialiasing) as FlexGlue does
	void Preprocess(const Image8U3& bgr, std::vector<float>& planarRgb) const override;
	int GridSize() const override { return WARP_GRID; }
	unsigned RetrievalSize() const override { return RETRIEVAL_SIZE; }

	OrtTensor MakeCode() override;
	bool DescribeImage(const float* planarRgb, OrtTensor& features, std::vector<float>& retrieval) override;
	bool MatchPair(const OrtTensor& featuresA, const OrtTensor& featuresB, const PairInput& input, PairOutput& out) override;

private:
	OnnxModel descriptor, match;
	// host inputs and outputs of the graphs, allocated once by Load (one thread uses them)
	OrtTensor image, retrieval;
	OrtTensor imageSize[2], warp[2], confidence[2], covisibility, rotation, direction;
};
#endif // _USE_ONNXRUNTIME
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_FLEXGLUEMATCHER_H_
