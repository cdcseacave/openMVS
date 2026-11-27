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
#include <nlohmann/json.hpp>

#include <fstream>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ROMA2   "));

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
	const char* key = "";
	try {
		key = "format_version"; const int formatVersion = data.at(key).get<int>();
		key = "model"; const std::string model = data.at(key).get<std::string>();
		if (formatVersion != 1 || model != "roma2") {
			VERBOSE("error: RoMa2 manifest '%s' is version %d of model '%s', expected version 1 of 'roma2'",
				fileName.c_str(), formatVersion, model.c_str());
			return false;
		}
		key = "setting"; setting = data.at(key).get<std::string>();
		key = "image_size"; imageSize = data.at(key).get<int>();
		key = "patch"; patch = data.at(key).get<int>();
		key = "layers"; (void)data.at(key).get<std::vector<int>>();
		key = "descriptor_layers_shape"; layersShape = data.at(key).get<std::vector<int64_t>>();
		key = "warp_size"; warpSize = data.at(key).get<int>();
		key = "confidence_channels"; confidenceChannels = data.at(key).get<int>();
		key = "value_facet_blocks"; (void)data.at(key).get<std::vector<int>>();
		key = "value_facets_shape"; facetsShape = data.at(key).get<std::vector<int64_t>>();
		key = "opset"; (void)data.at(key).get<int>();
		if (imageSize <= 0 || patch <= 0 || warpSize <= 0 || confidenceChannels <= 0) {
			VERBOSE("error: RoMa2 manifest '%s' has a non-positive image_size/patch/warp_size/confidence_channels", fileName.c_str());
			return false;
		}
		key = "retrieval_recipes"; facetsDim = data.at(key).at("facets").at("dim").get<unsigned>();
		key = "files";
		const nlohmann::json& files = data.at(key);
		descriptorFile = files.at("descriptor").get<std::string>();
		descriptorData = files.at("descriptor_data").get<std::string>();
		matchFile = files.at("match_coarse").get<std::string>();
		matchData = files.at("match_coarse_data").get<std::string>();

		const int64_t S(imageSize), cells(warpSize);
		const std::vector<int64_t> imageShape{1, 3, S, S};
		key = "io";
		const nlohmann::json& io = data.at(key);
		const nlohmann::json& descIn = io.at("descriptor").at("inputs");
		const nlohmann::json& descOut = io.at("descriptor").at("outputs");
		const nlohmann::json& matchIn = io.at("match_coarse").at("inputs");
		const nlohmann::json& matchOut = io.at("match_coarse").at("outputs");
		const auto CheckShape = [&](const nlohmann::json& node, const char* name, const std::vector<int64_t>& expected) {
			key = name;
			if (node.at(name).get<std::vector<int64_t>>() != expected) {
				VERBOSE("error: RoMa2 manifest '%s' declares a shape for '%s' that disagrees with its own sizes", fileName.c_str(), name);
				return false;
			}
			return true;
		};
		if (!CheckShape(descIn, "image", imageShape) ||
			!CheckShape(descOut, "layers", layersShape) ||
			!CheckShape(descOut, "value_facets", facetsShape) ||
			!CheckShape(matchIn, "descriptors_A", layersShape) ||
			!CheckShape(matchIn, "descriptors_B", layersShape) ||
			!CheckShape(matchOut, "warp", {1, cells, cells, 2}) ||
			!CheckShape(matchOut, "confidence", {1, cells, cells, (int64_t)confidenceChannels}) ||
			!CheckShape(matchOut, "warp_BA", {1, cells, cells, 2}) ||
			!CheckShape(matchOut, "confidence_BA", {1, cells, cells, (int64_t)confidenceChannels}))
			return false;
		if (matchIn.contains("img_A") && !CheckShape(matchIn, "img_A", imageShape))
			return false;
		if (matchIn.contains("img_B") && !CheckShape(matchIn, "img_B", imageShape))
			return false;
		key = "retrieval";
		retrievalShape = descOut.at(key).get<std::vector<int64_t>>();
		if (retrievalShape != std::vector<int64_t>{1, (int64_t)facetsDim}) {
			VERBOSE("error: RoMa2 manifest '%s' declares a shape for 'retrieval' that disagrees with its own sizes", fileName.c_str());
			return false;
		}
	} catch (const nlohmann::json::exception& e) {
		VERBOSE("error: RoMa2 manifest '%s' key '%s' missing or invalid (%s)", fileName.c_str(), key, e.what());
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

#ifdef _USE_ONNXRUNTIME

namespace {

// Keys cubic convolution kernel (A = -0.5, matching PyTorch antialiased bicubic)
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

struct AxisWeights {
	std::vector<int> start, count;
	std::vector<float> weights;
	int maxTaps;
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
		if (sum != 0.f) {
			const float invSum = 1.f / sum;
			for (int t = 0; t < count; ++t)
				taps[t] *= invSum;
		}
		w.start[i] = start;
		w.count[i] = count;
	}
	return w;
}

bool IsSupportedManifest(const RoMa2Manifest& manifest, const String& setting)
{
	return manifest.setting == setting && manifest.patch == 16 && manifest.confidenceChannels == 1 &&
		manifest.layersShape.size() == 5 && manifest.facetsShape == manifest.layersShape &&
		manifest.layersShape[0] == 1 && manifest.layersShape[1] == 2 &&
		manifest.layersShape[2] == manifest.imageSize / manifest.patch &&
		manifest.layersShape[3] == manifest.layersShape[2] &&
		manifest.facetsDim == (unsigned)(manifest.layersShape[1] * manifest.layersShape[4]);
}

void SigmoidInto(const OrtTensor& logitHost, Image32F& confidence)
{
	const float* pLogit(logitHost.HostData());
	float* pConfidence(confidence.ptr<float>());
	for (int i = 0, n = confidence.rows * confidence.cols; i < n; ++i)
		pConfidence[i] = 1.f / (1.f + std::exp(-pLogit[i]));
}

} // namespace

void SFM::PreprocessImageRoMa2(const Image8U3& bgr, int size, std::vector<float>& planarRgb)
{
	ASSERT(!bgr.empty() && size > 0);
	constexpr float kInv255 = 1.f / 255.f;
	const AxisWeights wx = ComputeAxisWeights(bgr.cols, size), wy = ComputeAxisWeights(bgr.rows, size);
	// Horizontal pass: rows x size x 3 (RGB), scaled to [0,1] without clamping
	std::vector<float> tmp((size_t)bgr.rows * size * 3);
	for (int y = 0; y < bgr.rows; ++y) {
		const Pixel8U* row = bgr.ptr<Pixel8U>(y);
		float* dst = &tmp[(size_t)y * size * 3];
		for (int x = 0; x < size; ++x, dst += 3) {
			float r = 0, g = 0, b = 0;
			const float* taps = &wx.weights[(size_t)x * wx.maxTaps];
			const Pixel8U* src = row + wx.start[x];
			for (int t = 0; t < wx.count[x]; ++t) {
				const float wt = taps[t];
				r += wt * src[t].r;
				g += wt * src[t].g;
				b += wt * src[t].b;
			}
			dst[0] = r * kInv255;
			dst[1] = g * kInv255;
			dst[2] = b * kInv255;
		}
	}
	// Vertical pass directly into planar R, G, B
	const size_t plane = (size_t)size * size;
	planarRgb.assign(3 * plane, 0.f);
	for (int y = 0; y < size; ++y) {
		float* dstR = &planarRgb[(size_t)y * size];
		float* dstG = dstR + plane;
		float* dstB = dstG + plane;
		const float* taps = &wy.weights[(size_t)y * wy.maxTaps];
		for (int t = 0; t < wy.count[y]; ++t) {
			const float* src = &tmp[(size_t)(wy.start[y] + t) * size * 3];
			const float wt = taps[t];
			for (int x = 0; x < size; ++x, src += 3) {
				dstR[x] += wt * src[0];
				dstG[x] += wt * src[1];
				dstB[x] += wt * src[2];
			}
		}
	}
}
/*----------------------------------------------------------------*/

void RoMa2Onnx::Reset()
{
	// Release tensors before sessions that own their device arenas
	image = OrtTensor();
	facetsScratch = OrtTensor();
	facetsHost = OrtTensor();
	retrievalHost = OrtTensor();
	dummyImage = OrtTensor();
	warpHost = OrtTensor();
	confidenceHost = OrtTensor();
	warpBAHost = OrtTensor();
	confidenceBAHost = OrtTensor();
	match.Unload();
	descriptor.Unload();
	modelDir.clear();
	manifest = RoMa2Manifest();
	bMatchFailed = false;
}

bool RoMa2Onnx::EnsureMatch()
{
	if (bMatchFailed)
		return false;
	if (match.IsLoaded())
		return true;
	OnnxModel::Options options;
	options.provider = descriptor.Provider();
	const int64_t cells(manifest.warpSize);
	if (!match.Load(modelDir + manifest.matchFile, options) ||
		!match.ExpectInputShape("descriptors_A", manifest.layersShape) ||
		!match.ExpectInputShape("descriptors_B", manifest.layersShape) ||
		!match.ExpectOutputShape("warp", {1, cells, cells, 2}) ||
		!match.ExpectOutputShape("confidence", {1, cells, cells, 1}) ||
		!match.ExpectOutputShape("warp_BA", {1, cells, cells, 2}) ||
		!match.ExpectOutputShape("confidence_BA", {1, cells, cells, 1})) {
		VERBOSE("error: can not load the RoMa2 coarse-match graph '%s'", manifest.matchFile.c_str());
		match.Unload();
		bMatchFailed = true;
		return false;
	}
	ASSERT(match.Provider() == descriptor.Provider());
	if (match.InputShape("img_A") != NULL) {
		const int64_t S(manifest.imageSize);
		dummyImage = OrtTensor::Host({1, 3, S, S});
	}
	warpHost = OrtTensor::Host({1, cells, cells, 2});
	confidenceHost = OrtTensor::Host({1, cells, cells, 1});
	warpBAHost = OrtTensor::Host({1, cells, cells, 2});
	confidenceBAHost = OrtTensor::Host({1, cells, cells, 1});
	if (!warpHost.IsValid() || !confidenceHost.IsValid() || !warpBAHost.IsValid() || !confidenceBAHost.IsValid() ||
		(match.InputShape("img_A") != NULL && !dummyImage.IsValid())) {
		VERBOSE("error: can not allocate the RoMa2 coarse-match tensors");
		match.Unload();
		bMatchFailed = true;
		return false;
	}
	return true;
}

bool RoMa2Onnx::Load(const String& _modelDir, const String& setting, const String& providerName)
{
	Reset();
	modelDir = _modelDir;
	Util::ensureFolderSlash(modelDir);
	if (!manifest.Load(modelDir + "roma_" + setting + ".json")) {
		Reset();
		return false;
	}
	if (!IsSupportedManifest(manifest, setting)) {
		VERBOSE("error: unsupported RoMa2 manifest for setting '%s'", setting.c_str());
		Reset();
		return false;
	}
	for (const String& file : {manifest.descriptorFile, manifest.descriptorData, manifest.matchFile, manifest.matchData}) {
		if (!file.empty() && !File::isFile(modelDir + file)) {
			VERBOSE("error: RoMa2 model file '%s' missing", file.c_str());
			Reset();
			return false;
		}
	}
	OnnxModel::Options options;
	options.provider = OnnxProviderFromString(providerName);
	if (!descriptor.Load(modelDir + manifest.descriptorFile, options)) {
		Reset();
		return false;
	}
	if (descriptor.Provider() == OnnxProvider::CPU)
		VERBOSE("warning: RoMa2 running on the CPU execution provider (seconds per image)");
	const int64_t S(manifest.imageSize);
	if (!descriptor.ExpectInputShape("image", {1, 3, S, S}) ||
		!descriptor.ExpectOutputShape("layers", manifest.layersShape) ||
		!descriptor.ExpectOutputShape("value_facets", manifest.facetsShape) ||
		!descriptor.ExpectOutputShape("retrieval", manifest.retrievalShape)) {
		Reset();
		return false;
	}
	image = OrtTensor::Host({1, 3, S, S});
	facetsScratch = descriptor.MakeDeviceTensor(manifest.facetsShape);
	retrievalHost = OrtTensor::Host(manifest.retrievalShape);
	if (!image.IsValid() || !facetsScratch.IsValid() || !retrievalHost.IsValid()) {
		VERBOSE("error: can not allocate the RoMa2 descriptor tensors");
		Reset();
		return false;
	}
	DEBUG("RoMa2 model '%s' ready on %s: %dx%d images, %ux%u descriptor grid, %dx%d warp cells",
		setting.c_str(), OnnxProviderName(descriptor.Provider()), manifest.imageSize, manifest.imageSize,
		(unsigned)manifest.layersShape[2], (unsigned)manifest.layersShape[3], manifest.warpSize, manifest.warpSize);
	return true;
}

String RoMa2Onnx::ProviderName() const
{
	return OnnxProviderName(descriptor.Provider());
}

unsigned RoMa2Onnx::NumPatches() const
{
	return manifest.layersShape.size() == 5 ? (unsigned)(manifest.layersShape[2] * manifest.layersShape[3]) : 0u;
}

OrtTensor RoMa2Onnx::MakeLayers()
{
	if (!IsLoaded())
		return OrtTensor();
	return descriptor.MakeDeviceTensor(manifest.layersShape);
}

bool RoMa2Onnx::Describe(const float* planarRgb, OrtTensor& layersOut, std::vector<float>* facetsOut, std::vector<float>& retrievalOut)
{
	ASSERT(IsLoaded() && planarRgb != NULL && layersOut.IsValid() && layersOut.Shape() == manifest.layersShape);
	if (facetsOut != NULL && !facetsHost.IsValid()) {
		facetsHost = OrtTensor::Host(manifest.facetsShape);
		if (!facetsHost.IsValid())
			return false;
	}
	memcpy(image.HostData(), planarRgb, sizeof(float) * 3 * (size_t)ImageSize() * ImageSize());
	descriptor.ClearBindings();
	if (!descriptor.BindInput("image", image) ||
		!descriptor.BindOutput("layers", layersOut) ||
		!descriptor.BindOutput("value_facets", facetsOut != NULL ? facetsHost : facetsScratch) ||
		!descriptor.BindOutput("retrieval", retrievalHost) ||
		!descriptor.Run())
		return false;
	if (facetsOut != NULL)
		facetsOut->assign(facetsHost.HostData(), facetsHost.HostData() + facetsHost.NumElements());
	retrievalOut.assign(retrievalHost.HostData(), retrievalHost.HostData() + retrievalHost.NumElements());
	return true;
}

bool RoMa2Onnx::MatchCoarse(const OrtTensor& layersA, const OrtTensor& layersB,
	Image32F2& warpAB, Image32F& confidenceAB, Image32F2& warpBA, Image32F& confidenceBA)
{
	ASSERT(IsLoaded() && layersA.IsValid() && layersB.IsValid());
	if (!EnsureMatch())
		return false;
	const int cells(WarpSize());
	match.ClearBindings();
	if (!match.BindInput("descriptors_A", layersA) ||
		!match.BindInput("descriptors_B", layersB) ||
		!match.BindOutput("warp", warpHost) ||
		!match.BindOutput("confidence", confidenceHost) ||
		!match.BindOutput("warp_BA", warpBAHost) ||
		!match.BindOutput("confidence_BA", confidenceBAHost))
		return false;
	if (dummyImage.IsValid() &&
		(!match.BindInput("img_A", dummyImage) || !match.BindInput("img_B", dummyImage)))
		return false;
	if (!match.Run())
		return false;
	warpAB.create(cells, cells);
	confidenceAB.create(cells, cells);
	warpBA.create(cells, cells);
	confidenceBA.create(cells, cells);
	memcpy(warpAB.data, warpHost.HostData(), (size_t)cells * cells * sizeof(Point2f));
	memcpy(warpBA.data, warpBAHost.HostData(), (size_t)cells * cells * sizeof(Point2f));
	SigmoidInto(confidenceHost, confidenceAB);
	SigmoidInto(confidenceBAHost, confidenceBA);
	return true;
}

void RoMa2Onnx::Preprocess(const Image8U3& bgr, std::vector<float>& planarRgb) const
{
	PreprocessImageRoMa2(bgr, ImageSize(), planarRgb);
}

bool RoMa2Onnx::MatchPair(const OrtTensor& layersA, const OrtTensor& layersB, const PairInput&, PairOutput& out)
{
	out.covisibility.reset();
	out.pose.reset();
	return MatchCoarse(layersA, layersB, out.ab.warp, out.ab.confidence, out.ba.warp, out.ba.confidence);
}

#endif // _USE_ONNXRUNTIME

#pragma pop_macro("VERBOSE")
/*----------------------------------------------------------------*/
