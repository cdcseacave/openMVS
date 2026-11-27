/*
 * RoMa2Matcher.h
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

#ifndef _SFM_ROMA2MATCHER_H_
#define _SFM_ROMA2MATCHER_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "OnnxRuntime.h"
#include "DenseMatcher.h"

#include <cstdint>
#include <vector>


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

#ifdef _USE_ONNXRUNTIME

// Resample an 8-bit BGR image to size x size planar RGB in [0,1] matching PyTorch's
// F.interpolate(mode="bicubic", align_corners=False, antialias=True). Thread-safe.
SFM_API void PreprocessImageRoMa2(const Image8U3& bgr, int size, std::vector<float>& planarRgb);

#endif // _USE_ONNXRUNTIME
/*----------------------------------------------------------------*/

// Exported RoMa v2 preset described by roma_<setting>.json: graph filenames and static I/O shapes.
struct SFM_API RoMa2Manifest
{
	String setting;                        // preset name: turbo|fast|base
	int imageSize = 0;                     // S: input image side
	int patch = 16;                        // backbone patch size (G = S/patch)
	std::vector<int64_t> layersShape;      // [1,2,G,G,1024]
	std::vector<int64_t> facetsShape;      // [1,2,G,G,1024]
	std::vector<int64_t> retrievalShape;   // [1,facetsDim]
	int warpSize = 0;                      // C: coarse warp grid side
	int confidenceChannels = 1;
	unsigned facetsDim = 2048;             // pooled retrieval descriptor dimension
	String descriptorFile, descriptorData;
	String matchFile, matchData;

	// Parse and validate the manifest against schema version 1 and self-consistent I/O shapes.
	bool Load(const String& fileName);
};
/*----------------------------------------------------------------*/

#ifdef _USE_ONNXRUNTIME

// RoMa v2 dense matcher over ONNX Runtime: descriptor session plus lazily loaded coarse-match session
// sharing the same execution provider. Single-threaded per instance; destroy MakeLayers() tensors
// before this object or before reloading.
class SFM_API RoMa2Onnx : public DenseMatcher
{
public:
	RoMa2Onnx() = default;
	RoMa2Onnx(const RoMa2Onnx&) = delete;
	RoMa2Onnx& operator=(const RoMa2Onnx&) = delete;
	~RoMa2Onnx() override = default;

	bool Load(const String& modelDir, const String& setting = "base", const String& provider = "auto") override;
	bool IsLoaded() const override { return descriptor.IsLoaded(); }
	const RoMa2Manifest& Manifest() const { return manifest; }
	String ProviderName() const override;

	inline int ImageSize() const override { return manifest.imageSize; }
	inline int GridSize() const override { return manifest.warpSize; }
	inline unsigned RetrievalSize() const override { return manifest.facetsDim; }
	void Preprocess(const Image8U3& bgr, std::vector<float>& planarRgb) const override;
	OrtTensor MakeCode() override { return MakeLayers(); }
	bool DescribeImage(const float* planarRgb, OrtTensor& layers, std::vector<float>& retrieval) override { return Describe(planarRgb, layers, NULL, retrieval); }
	bool MatchPair(const OrtTensor& layersA, const OrtTensor& layersB, const PairInput&, PairOutput& out) override;

	inline int WarpSize() const { return manifest.warpSize; }
	inline const std::vector<int64_t>& LayersShape() const { return manifest.layersShape; }
	unsigned NumPatches() const;

	OrtTensor MakeLayers();
	bool Describe(const float* planarRgb, OrtTensor& layersOut, std::vector<float>* facetsOut, std::vector<float>& retrievalOut);
	bool MatchCoarse(const OrtTensor& layersA, const OrtTensor& layersB,
		Image32F2& warpAB, Image32F& confidenceAB, Image32F2& warpBA, Image32F& confidenceBA);

private:
	void Reset();
	bool EnsureMatch();

	// Sessions declared first so tensors allocated from their arenas are destroyed before them
	OnnxModel descriptor;
	OnnxModel match;
	OrtTensor image;
	OrtTensor facetsScratch;
	OrtTensor facetsHost;
	OrtTensor retrievalHost;
	OrtTensor dummyImage;
	OrtTensor warpHost, confidenceHost;
	OrtTensor warpBAHost, confidenceBAHost;
	String modelDir;
	RoMa2Manifest manifest;
	bool bMatchFailed = false;
};

#endif // _USE_ONNXRUNTIME
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_ROMA2MATCHER_H_
