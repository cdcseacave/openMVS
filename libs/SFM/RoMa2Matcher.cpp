/*
 * RoMa2Matcher.cpp
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

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "RoMa2Matcher.h"
#include "../IO/json.hpp"

#include <fstream>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ROMA2   "));

namespace {

// Fetch a required manifest member, naming the file and the missing key on failure
const nlohmann::json* JsonMember(const nlohmann::json& node, const char* key, const String& fileName)
{
	const auto it = node.find(key);
	if (it == node.end()) {
		VERBOSE("error: RoMa2 manifest '%s' has no '%s' key", fileName.c_str(), key);
		return NULL;
	}
	return &(*it);
}

// Read a required manifest member of the given type, naming a key that is missing or ill-typed
template <typename TYPE>
bool ReadJson(const nlohmann::json& node, const char* key, const String& fileName, TYPE& value)
{
	const nlohmann::json* const member = JsonMember(node, key, fileName);
	if (member == NULL)
		return false;
	try {
		value = member->get<TYPE>();
	} catch (const nlohmann::json::exception& e) {
		VERBOSE("error: RoMa2 manifest '%s' key '%s' has the wrong type (%s)", fileName.c_str(), key, e.what());
		return false;
	}
	return true;
}

// Read a required manifest string member (SEACAVE::String is not a type nlohmann converts to)
bool ReadJsonString(const nlohmann::json& node, const char* key, const String& fileName, String& value)
{
	std::string text;
	if (!ReadJson(node, key, fileName, text))
		return false;
	value = text;
	return true;
}

// Check a declared graph I/O shape against the one derived from the manifest's own sizes:
// a disagreement means the manifest does not describe the graphs sitting next to it
bool ExpectManifestShape(const nlohmann::json& node, const char* key, const String& fileName, const std::vector<int64_t>& expected)
{
	std::vector<int64_t> shape;
	if (!ReadJson(node, key, fileName, shape))
		return false;
	if (shape != expected) {
		VERBOSE("error: RoMa2 manifest '%s' declares a shape for '%s' that disagrees with its own sizes", fileName.c_str(), key);
		return false;
	}
	return true;
}

} // namespace

bool RoMa2Manifest::Load(const String& fileName)
{
	std::ifstream stream(fileName.c_str());
	if (!stream.is_open()) {
		VERBOSE("error: failed to open RoMa2 manifest '%s'", fileName.c_str());
		return false;
	}
	const nlohmann::json data = nlohmann::json::parse(stream, nullptr, false);
	if (data.is_discarded() || !data.is_object()) {
		VERBOSE("error: failed to parse RoMa2 manifest '%s'", fileName.c_str());
		return false;
	}
	int formatVersion = 0;
	String model;
	if (!ReadJson(data, "format_version", fileName, formatVersion) ||
		!ReadJsonString(data, "model", fileName, model))
		return false;
	if (formatVersion != 1 || model != "roma2") {
		VERBOSE("error: RoMa2 manifest '%s' is version %d of model '%s', expected version 1 of 'roma2'",
			fileName.c_str(), formatVersion, model.c_str());
		return false;
	}
	if (!ReadJsonString(data, "setting", fileName, setting) ||
		!ReadJson(data, "image_size", fileName, imageSize) ||
		!ReadJson(data, "patch", fileName, patch) ||
		!ReadJson(data, "layers", fileName, layers) ||
		!ReadJson(data, "descriptor_layers_shape", fileName, layersShape) ||
		!ReadJson(data, "warp_size", fileName, warpSize) ||
		!ReadJson(data, "confidence_channels", fileName, confidenceChannels) ||
		!ReadJson(data, "value_facet_blocks", fileName, valueFacetBlocks) ||
		!ReadJson(data, "value_facets_shape", fileName, facetsShape) ||
		!ReadJson(data, "opset", fileName, opset))
		return false;
	if (imageSize <= 0 || patch <= 0 || warpSize <= 0 || confidenceChannels <= 0) {
		VERBOSE("error: RoMa2 manifest '%s' has a non-positive image_size/patch/warp_size/confidence_channels", fileName.c_str());
		return false;
	}
	// the retrieval recipes: only the GeM p=3 pooling of PoolRetrievalDescriptor is implemented,
	// with LAYERS pooling the deepest of the two descriptor slices
	const nlohmann::json* const recipes = JsonMember(data, "retrieval_recipes", fileName);
	const nlohmann::json* const facetsRecipe = (recipes != NULL ? JsonMember(*recipes, "facets", fileName) : NULL);
	const nlohmann::json* const layersRecipe = (recipes != NULL ? JsonMember(*recipes, "layers", fileName) : NULL);
	if (facetsRecipe == NULL || layersRecipe == NULL)
		return false;
	int facetsGemP = 0, layersGemP = 0, layersSlice = 0;
	if (!ReadJson(*facetsRecipe, "dim", fileName, facetsDim) ||
		!ReadJson(*facetsRecipe, "gem_p", fileName, facetsGemP) ||
		!ReadJson(*facetsRecipe, "power", fileName, facetsPower) ||
		!ReadJson(*layersRecipe, "dim", fileName, layersDim) ||
		!ReadJson(*layersRecipe, "gem_p", fileName, layersGemP) ||
		!ReadJson(*layersRecipe, "slice", fileName, layersSlice))
		return false;
	if (facetsGemP != 3 || layersGemP != 3 || layersSlice != 1) {
		VERBOSE("error: RoMa2 manifest '%s' asks for retrieval recipes this build does not implement "
			"(GeM p=%d/%d, layers slice %d; expected p=3 and slice 1)",
			fileName.c_str(), facetsGemP, layersGemP, layersSlice);
		return false;
	}
	const nlohmann::json* const files = JsonMember(data, "files", fileName);
	if (files == NULL ||
		!ReadJsonString(*files, "descriptor", fileName, descriptorFile) ||
		!ReadJsonString(*files, "descriptor_data", fileName, descriptorData) ||
		!ReadJsonString(*files, "match_coarse", fileName, matchFile) ||
		!ReadJsonString(*files, "match_coarse_data", fileName, matchData))
		return false;
	// the declared graph I/O must agree with the shapes derived from image_size and warp_size:
	// the C++ loader checks the graphs against these same derived shapes
	const int64_t S(imageSize), cells(warpSize);
	const std::vector<int64_t> imageShape{1, 3, S, S};
	const nlohmann::json* const io = JsonMember(data, "io", fileName);
	const nlohmann::json* const ioDescriptor = (io != NULL ? JsonMember(*io, "descriptor", fileName) : NULL);
	const nlohmann::json* const ioMatch = (io != NULL ? JsonMember(*io, "match_coarse", fileName) : NULL);
	if (ioDescriptor == NULL || ioMatch == NULL)
		return false;
	const nlohmann::json* const descriptorInputs = JsonMember(*ioDescriptor, "inputs", fileName);
	const nlohmann::json* const descriptorOutputs = JsonMember(*ioDescriptor, "outputs", fileName);
	const nlohmann::json* const matchInputs = JsonMember(*ioMatch, "inputs", fileName);
	const nlohmann::json* const matchOutputs = JsonMember(*ioMatch, "outputs", fileName);
	if (descriptorInputs == NULL || descriptorOutputs == NULL || matchInputs == NULL || matchOutputs == NULL)
		return false;
	if (!ExpectManifestShape(*descriptorInputs, "image", fileName, imageShape) ||
		!ExpectManifestShape(*descriptorOutputs, "layers", fileName, layersShape) ||
		!ExpectManifestShape(*descriptorOutputs, "value_facets", fileName, facetsShape) ||
		!ExpectManifestShape(*matchInputs, "descriptors_A", fileName, layersShape) ||
		!ExpectManifestShape(*matchInputs, "descriptors_B", fileName, layersShape) ||
		!ExpectManifestShape(*matchInputs, "img_A", fileName, imageShape) ||
		!ExpectManifestShape(*matchInputs, "img_B", fileName, imageShape) ||
		!ExpectManifestShape(*matchOutputs, "warp", fileName, {1, cells, cells, 2}) ||
		!ExpectManifestShape(*matchOutputs, "confidence", fileName, {1, cells, cells, (int64_t)confidenceChannels}))
		return false;
	return true;
}
/*----------------------------------------------------------------*/

#ifdef _USE_ONNXRUNTIME

namespace {

// Keys cubic convolution kernel, A=-0.5: torch's ANTIALIASED bicubic coefficient (not the
// -0.75 of its plain bicubic path, which resamples ~2e-3 differently)
inline float CubicFilter(float x)
{
	constexpr float kA = -0.5f;
	x = ABS(x);
	if (x < 1.f)
		return ((kA + 2.f) * x - (kA + 3.f)) * x * x + 1.f;
	if (x < 2.f)
		return ((kA * x - 5.f * kA) * x + 8.f * kA) * x - 4.f * kA;
	return 0.f;
}

// Per-output-sample resampling taps for one axis: the source range each output sample reads
// and the normalized cubic weights over it. Mirrors torch's antialiased weight computation
// (aten/src/ATen/native/UpSample.h): the filter support widens with the downsampling ratio,
// which is what antialias=true buys over a plain cubic.
struct AxisWeights {
	std::vector<int> start, count; // per output sample: first source index read, tap count
	std::vector<float> weights;    // maxTaps-strided, count[i] valid entries per output sample
	int maxTaps;                   // weights stride: the widest tap count over all output samples
};

AxisWeights ComputeAxisWeights(int in, int out)
{
	AxisWeights w;
	w.start.resize(out);
	w.count.resize(out);
	const float scale = (float)in / (float)out;
	const float support = scale >= 1.f ? 2.f * scale : 2.f;
	const float invScale = scale >= 1.f ? 1.f / scale : 1.f;
	w.maxTaps = MINF((int)std::ceil(support) * 2 + 1, in);
	w.weights.assign((size_t)out * w.maxTaps, 0.f);
	for (int i = 0; i < out; ++i) {
		const float center = scale * (i + 0.5f);
		const int start = MAXF((int)(center - support + 0.5f), 0);
		const int count = MINF((int)(center + support + 0.5f), in) - start;
		float* taps = &w.weights[(size_t)i * w.maxTaps];
		float sum = 0.f;
		for (int t = 0; t < count; ++t)
			sum += (taps[t] = CubicFilter((t + start - center + 0.5f) * invScale));
		if (sum != 0.f)
			for (int t = 0; t < count; ++t)
				taps[t] /= sum;
		w.start[i] = start;
		w.count[i] = count;
	}
	return w;
}

} // namespace

void SFM::PreprocessImageRoMa2(const Image8U3& bgr, int size, std::vector<float>& planarRgb)
{
	ASSERT(!bgr.empty() && size > 0);
	const AxisWeights wx = ComputeAxisWeights(bgr.cols, size), wy = ComputeAxisWeights(bgr.rows, size);
	// horizontal pass: rows x size x 3 (RGB), scaled to [0,1]; NO clamping (bicubic overshoot is in the reference too)
	std::vector<float> tmp((size_t)bgr.rows * size * 3);
	for (int y = 0; y < bgr.rows; ++y) {
		const Pixel8U* row = bgr.ptr<Pixel8U>(y);
		float* dst = &tmp[(size_t)y * size * 3];
		for (int x = 0; x < size; ++x, dst += 3) {
			float r = 0, g = 0, b = 0;
			const float* taps = &wx.weights[(size_t)x * wx.maxTaps];
			for (int t = 0; t < wx.count[x]; ++t) {
				const Pixel8U& p = row[wx.start[x] + t];
				r += taps[t] * p.r; g += taps[t] * p.g; b += taps[t] * p.b;
			}
			dst[0] = r / 255.f; dst[1] = g / 255.f; dst[2] = b / 255.f;
		}
	}
	// vertical pass straight into planar R, G, B
	planarRgb.assign((size_t)3 * size * size, 0.f);
	for (int y = 0; y < size; ++y) {
		const float* taps = &wy.weights[(size_t)y * wy.maxTaps];
		for (int t = 0; t < wy.count[y]; ++t) {
			const float* src = &tmp[(size_t)(wy.start[y] + t) * size * 3];
			const float wt = taps[t];
			for (int x = 0; x < size; ++x, src += 3)
				for (int c = 0; c < 3; ++c)
					planarRgb[((size_t)c * size + y) * size + x] += wt * src[c];
		}
	}
}
/*----------------------------------------------------------------*/

namespace {

// The subset of manifests this build can run: the descriptor grid, the pooled retrieval
// dimensions and the confidence layout the C++ derives from the shapes must all agree with
// what the manifest declares, or the tensors allocated here would not describe the graphs
bool IsSupportedManifest(const RoMa2Manifest& manifest, const String& setting)
{
	return manifest.setting == setting && manifest.patch == 16 && manifest.confidenceChannels == 1 &&
		manifest.layersShape.size() == 5 && manifest.facetsShape == manifest.layersShape &&
		manifest.layersShape[0] == 1 && manifest.layersShape[1] == 2 &&
		manifest.layersShape[2] == manifest.imageSize/manifest.patch &&
		manifest.layersShape[3] == manifest.layersShape[2] &&
		manifest.layersDim == (unsigned)manifest.layersShape[4] &&
		manifest.facetsDim == (unsigned)(manifest.layersShape[1]*manifest.layersShape[4]);
}

} // namespace

// The two ONNX Runtime sessions are declared first and every OrtTensor after them: members
// are destroyed in the reverse of their declaration order, so the tensors -- some of which
// are allocated from a session's device arena -- are released before the session that owns
// that arena, which is the lifetime rule OrtTensor documents.
struct RoMa2Onnx::Impl
{
	OnnxModel descriptor;      // image -> (layers, value_facets)
	OnnxModel match;           // (descriptors_A, descriptors_B, img_A, img_B) -> (warp, confidence)
	OrtTensor image;           // host input of the descriptor graph, copied H2D by ORT inside Run
	OrtTensor facetsScratch;   // matching pass: value_facets stays on the device, never read back
	OrtTensor facetsHost;      // retrieval pass: bound instead of the scratch so ORT copies it out
	OrtTensor dummyImage;      // the coarse graph's dead img_A/img_B input, shared by both
	OrtTensor warpHost;        // host output: ORT copies the warp D2H inside Run
	OrtTensor confidenceHost;  // host output: the raw overlap logit
	String modelDir;
	RoMa2Manifest manifest;
	bool bMatchFailed = false; // a failed match-graph load is remembered, not retried per pair

	// Load the coarse-match graph on the first pair, on the provider the descriptor session got
	bool EnsureMatch();
};

bool RoMa2Onnx::Impl::EnsureMatch()
{
	if (bMatchFailed)
		return false;
	if (match.IsLoaded())
		return true;
	// the same provider (and device ID) as the descriptor session, so a descriptor tensor left
	// in its device arena is bindable as an input here
	OnnxModel::Options options;
	options.provider = descriptor.Provider();
	const int64_t cells(manifest.warpSize);
	if (!match.Load(modelDir + manifest.matchFile, options) ||
		!match.ExpectInputShape("descriptors_A", manifest.layersShape) ||
		!match.ExpectInputShape("descriptors_B", manifest.layersShape) ||
		!match.ExpectOutputShape("warp", {1, cells, cells, 2}) ||
		!match.ExpectOutputShape("confidence", {1, cells, cells, 1}) ||
		match.Provider() != descriptor.Provider()) {
		VERBOSE("error: can not load the RoMa2 coarse-match graph '%s'", manifest.matchFile.c_str());
		bMatchFailed = true;
		return false;
	}
	// img_A/img_B are in the graph contract (polyml MatchWrap) but dead on the coarse graph
	// (dpt.py:127-138 ignores them), so one shared image tensor is bound to both; its contents
	// never reach an output, which is why the device arena's uninitialized memory is fine here
	if (match.InputShape("img_A") != NULL) {
		const int64_t S(manifest.imageSize);
		dummyImage = match.MakeDeviceTensor({1, 3, S, S});
	}
	warpHost = OrtTensor::Host({1, cells, cells, 2});
	confidenceHost = OrtTensor::Host({1, cells, cells, 1});
	if (!warpHost.IsValid() || !confidenceHost.IsValid() ||
		(match.InputShape("img_A") != NULL && !dummyImage.IsValid())) {
		VERBOSE("error: can not allocate the RoMa2 coarse-match tensors");
		bMatchFailed = true;
		return false;
	}
	return true;
}

bool RoMa2Onnx::IsAvailable()
{
	return true;
}

bool RoMa2Onnx::Load(const String& modelDir, const String& setting, const String& providerName)
{
	impl.reset(new Impl); // a reload starts clean, releasing the tensors before their sessions
	impl->modelDir = modelDir;
	Util::ensureFolderSlash(impl->modelDir);
	RoMa2Manifest& manifest = impl->manifest;
	if (!manifest.Load(impl->modelDir + "roma_" + setting + ".json"))
		return false;
	if (!IsSupportedManifest(manifest, setting)) {
		VERBOSE("error: unsupported RoMa2 manifest for setting '%s'", setting.c_str());
		return false;
	}
	for (const String& file : {manifest.descriptorFile, manifest.descriptorData, manifest.matchFile, manifest.matchData})
		if (!file.empty() && !File::isFile(impl->modelDir + file)) {
			VERBOSE("error: RoMa2 model file '%s' missing", file.c_str());
			return false;
		}
	OnnxModel::Options options;
	options.provider = OnnxProviderFromString(providerName);
	if (!impl->descriptor.Load(impl->modelDir + manifest.descriptorFile, options))
		return false;
	if (impl->descriptor.Provider() == OnnxProvider::CPU)
		VERBOSE("warning: RoMa2 running on the CPU execution provider (seconds per image)");
	const int64_t S(manifest.imageSize);
	if (!impl->descriptor.ExpectInputShape("image", {1, 3, S, S}) ||
		!impl->descriptor.ExpectOutputShape("layers", manifest.layersShape) ||
		!impl->descriptor.ExpectOutputShape("value_facets", manifest.facetsShape))
		return false;
	impl->image = OrtTensor::Host({1, 3, S, S});
	impl->facetsScratch = impl->descriptor.MakeDeviceTensor(manifest.facetsShape);
	impl->facetsHost = OrtTensor::Host(manifest.facetsShape);
	if (!impl->image.IsValid() || !impl->facetsScratch.IsValid() || !impl->facetsHost.IsValid()) {
		VERBOSE("error: can not allocate the RoMa2 descriptor tensors");
		return false;
	}
	DEBUG("RoMa2 model '%s' ready on %s: %dx%d images, %ux%u descriptor grid, %dx%d warp cells",
		setting.c_str(), ProviderName().c_str(), manifest.imageSize, manifest.imageSize,
		(unsigned)manifest.layersShape[2], (unsigned)manifest.layersShape[3], manifest.warpSize, manifest.warpSize);
	return true;
}

bool RoMa2Onnx::IsLoaded() const
{
	return impl->descriptor.IsLoaded();
}

String RoMa2Onnx::ProviderName() const
{
	return OnnxProviderName(impl->descriptor.Provider());
}

OrtTensor RoMa2Onnx::MakeLayers()
{
	ASSERT(IsLoaded());
	return impl->descriptor.MakeDeviceTensor(impl->manifest.layersShape);
}

bool RoMa2Onnx::Describe(const float* planarRgb, OrtTensor& layersOut, std::vector<float>* facetsOut)
{
	ASSERT(IsLoaded() && layersOut.Shape() == impl->manifest.layersShape);
	memcpy(impl->image.HostData(), planarRgb, sizeof(float)*3*(size_t)ImageSize()*ImageSize());
	impl->descriptor.ClearBindings();
	if (!impl->descriptor.BindInput("image", impl->image) ||
		!impl->descriptor.BindOutput("layers", layersOut) ||
		!impl->descriptor.BindOutput("value_facets", facetsOut != NULL ? impl->facetsHost : impl->facetsScratch))
		return false;
	if (!impl->descriptor.Run())
		return false;
	if (facetsOut != NULL)
		facetsOut->assign(impl->facetsHost.HostData(), impl->facetsHost.HostData()+impl->facetsHost.NumElements());
	return true;
}

bool RoMa2Onnx::MatchCoarse(const OrtTensor& layersA, const OrtTensor& layersB, Image32F2& warp, Image32F& overlap)
{
	ASSERT(IsLoaded());
	if (!impl->EnsureMatch())
		return false;
	const int cells(WarpSize());
	impl->match.ClearBindings();
	if (!impl->match.BindInput("descriptors_A", layersA) ||
		!impl->match.BindInput("descriptors_B", layersB) ||
		!impl->match.BindOutput("warp", impl->warpHost) ||           // host outputs: ORT copies D2H in Run()
		!impl->match.BindOutput("confidence", impl->confidenceHost))
		return false;
	if (impl->dummyImage.IsValid() &&
		(!impl->match.BindInput("img_A", impl->dummyImage) || !impl->match.BindInput("img_B", impl->dummyImage)))
		return false;
	if (!impl->match.Run())
		return false;
	warp.create(cells, cells);
	overlap.create(cells, cells);
	memcpy(warp.data, impl->warpHost.HostData(), (size_t)cells*cells*sizeof(Point2f)); // Point2f{x,y} == the graph's [...,0:2]
	// the graph emits the raw overlap logit; the sigmoid is applied here (polycpp matcher.cpp:484-494 GatherOverlap)
	const float* pLogit(impl->confidenceHost.HostData());
	float* pOverlap(overlap.ptr<float>());
	for (int i = 0, n = cells*cells; i < n; ++i)
		pOverlap[i] = 1.f/(1.f + std::exp(-pLogit[i]));
	return true;
}

#else // _USE_ONNXRUNTIME

// Inert stand-in for builds without ONNX Runtime: no graph can ever be loaded, so the manifest
// stays empty and the two run calls are unreachable
struct RoMa2Onnx::Impl
{
	RoMa2Manifest manifest;
};

bool RoMa2Onnx::IsAvailable()
{
	return false;
}

bool RoMa2Onnx::Load(const String&, const String&, const String&)
{
	VERBOSE("error: ROMA2 model requested but this build has no ONNX Runtime support");
	return false;
}

bool RoMa2Onnx::IsLoaded() const
{
	return false;
}

String RoMa2Onnx::ProviderName() const
{
	return String();
}

OrtTensor RoMa2Onnx::MakeLayers()
{
	return OrtTensor();
}

bool RoMa2Onnx::Describe(const float*, OrtTensor&, std::vector<float>*)
{
	ASSERT(false); // unreachable: Load() always fails, so no caller ever gets a loaded model
	return false;
}

bool RoMa2Onnx::MatchCoarse(const OrtTensor&, const OrtTensor&, Image32F2&, Image32F&)
{
	ASSERT(false); // unreachable: Load() always fails, so no caller ever gets a loaded model
	return false;
}

#endif // _USE_ONNXRUNTIME

// the accessors that only read the manifest, shared by both builds (the out-of-line destructor
// is what lets the header hold a unique_ptr to the incomplete Impl)
RoMa2Onnx::RoMa2Onnx()
	: impl(new Impl)
{
}

RoMa2Onnx::~RoMa2Onnx()
{
}

const RoMa2Manifest& RoMa2Onnx::Manifest() const
{
	return impl->manifest;
}

unsigned RoMa2Onnx::NumPatches() const
{
	const std::vector<int64_t>& shape = LayersShape();
	return (shape.size() == 5 ? (unsigned)(shape[2]*shape[3]) : 0u);
}

#pragma pop_macro("VERBOSE")
/*----------------------------------------------------------------*/
