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

#include "Common.h" // SFM_API, Image8U3, Image32F, Image32F2
#include "OnnxRuntime.h" // OrtTensor (an inert stand-in in builds without ONNX Runtime)

#include <cstdint>
#include <memory>
#include <vector>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

#ifdef _USE_ONNXRUNTIME

// Resample an 8-bit BGR image to size x size planar RGB in [0,1], reproducing torch's
// F.interpolate(mode="bicubic", align_corners=False, antialias=True) exactly (separable Keys
// cubic with A=-0.5, antialiasing support scaled by the downsampling ratio) -- the
// preprocessing the exported RoMa2 descriptor graph expects on its `image` input.
// Pure CPU, thread-safe (no shared state).
SFM_API void PreprocessImageRoMa2(const Image8U3& bgr, int size, std::vector<float>& planarRgb);

#endif // _USE_ONNXRUNTIME
/*----------------------------------------------------------------*/

// One exported ROMAv2 preset, as described by its `roma_<setting>.json` manifest: which
// graphs to load, the static shapes of their inputs/outputs, and the recipes the host-side
// retrieval pooling follows. Everything the C++ needs to consume an export it did not produce.
struct SFM_API RoMa2Manifest
{
	String setting;                    // preset name: turbo|fast|base
	int imageSize = 0;                 // S: the graphs take S x S images
	int patch = 16;                    // backbone patch size (S/patch = G, the descriptor grid)
	std::vector<int> layers;           // backbone blocks the `layers` output stacks
	std::vector<int> valueFacetBlocks; // backbone blocks the `value_facets` output taps
	std::vector<int64_t> layersShape;  // [1,2,G,G,1024], the `layers` output shape
	std::vector<int64_t> facetsShape;  // [1,2,G,G,1024], the `value_facets` output shape
	int warpSize = 0;                  // C: the coarse matcher's C x C warp grid
	int confidenceChannels = 1;        // channels of the `confidence` output
	unsigned facetsDim = 2048;         // dimension of the FACETS retrieval descriptor
	unsigned layersDim = 1024;         // dimension of the LAYERS retrieval descriptor
	float facetsPower = 0.3f;          // exponent of the FACETS signed power normalization
	int opset = 0;                     // ONNX opset the graphs were traced with
	String descriptorFile, descriptorData; // descriptor graph and its external-data file
	String matchFile, matchData;           // coarse-match graph and its external-data file

	// Parse the manifest, rejecting a file of another schema version, with a missing or
	// ill-typed key, or whose declared graph I/O disagrees with the shapes derived from
	// `image_size`/`warp_size`; every rejection names the offending key
	bool Load(const String& fileName);
};
/*----------------------------------------------------------------*/

// One ROMAv2 model: the descriptor graph and, loaded lazily on the first pair, the coarse-match
// graph, both on the same ONNX Runtime execution provider so a descriptor tensor left on the
// device can be fed straight to the matcher. Owns the sessions and their scratch tensors and
// nothing else; the slot pool and the scheduling live above it.
// All calls on one instance must come from a single thread (ONNX Runtime sessions are used
// through one IoBinding each). Available only in builds with ONNX Runtime: elsewhere
// IsAvailable() is false and Load() fails with a message.
class SFM_API RoMa2Onnx
{
public:
	RoMa2Onnx();
	RoMa2Onnx(const RoMa2Onnx&) = delete;
	RoMa2Onnx& operator=(const RoMa2Onnx&) = delete;
	~RoMa2Onnx();

	// True if this build can run the exported graphs at all
	static bool IsAvailable();

	// Load the manifest `roma_<setting>.json` of modelDir and its descriptor graph on the
	// requested execution provider ("auto"|"cuda"|"coreml"|"dml"|"cpu"); the coarse-match
	// graph is loaded on the first MatchCoarse call, on the provider the descriptor got.
	// Returns false, with a message, on any manifest, file, or session problem.
	bool Load(const String& modelDir, const String& setting = "base", const String& provider = "auto");

	bool IsLoaded() const;
	const RoMa2Manifest& Manifest() const;
	String ProviderName() const; // execution provider of the descriptor session

	inline int ImageSize() const { return Manifest().imageSize; }
	inline int WarpSize() const { return Manifest().warpSize; }
	inline const std::vector<int64_t>& LayersShape() const { return Manifest().layersShape; }
	unsigned NumPatches() const; // G*G, the descriptor grid cells one image is described by

	// One image's descriptor tensor, on the session device when the provider has device
	// memory (a host tensor otherwise); invalid if the model is not loaded
	OrtTensor MakeLayers();

	// Run the descriptor graph on one preprocessed image (3*S*S floats, PreprocessImageRoMa2),
	// writing the `layers` output into layersOut (a MakeLayers()-shaped tensor). When facetsOut
	// is not NULL the raw `value_facets` tensor is read back into it as 2*G*G*1024 host floats;
	// only the retrieval pass asks for it, the matching pass leaves it on the device.
	bool Describe(const float* planarRgb, OrtTensor& layersOut, std::vector<float>* facetsOut);

	// Run the coarse-match graph on two descriptor tensors, returning the C x C normalized warp
	// (align_corners=false) into image B and the overlap probability (the graph's logit through a sigmoid)
	bool MatchCoarse(const OrtTensor& layersA, const OrtTensor& layersB, Image32F2& warp, Image32F& overlap);

private:
	struct Impl;
	std::unique_ptr<Impl> impl; // pimpl: the header stays free of the ONNX Runtime session types
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_ROMA2MATCHER_H_
