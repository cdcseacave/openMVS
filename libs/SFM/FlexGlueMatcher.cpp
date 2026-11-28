/*
 * FlexGlueMatcher.cpp
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
#include "FlexGlueMatcher.h"
#include <nlohmann/json.hpp>
#include <fstream>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("FlexGlue"));

bool FlexGlueManifest::Load(const String& fileName)
{
	std::ifstream stream(fileName.c_str());
	if (!stream.is_open()) {
		VERBOSE("error: failed to open the FlexGlue manifest '%s'", fileName.c_str());
		return false;
	}
	const nlohmann::json data = nlohmann::json::parse(stream, nullptr, false);
	if (data.is_discarded() || !data.is_object()) {
		VERBOSE("error: failed to parse the FlexGlue manifest '%s'", fileName.c_str());
		return false;
	}
	const char* key = "";
	try {
		key = "format_version"; const int formatVersion = data.at(key).get<int>();
		key = "model"; const std::string model = data.at(key).get<std::string>();
		// format_version 1: one match graph returning both warps, the covisibility score and the H1 pose (R10, t10) from one Run
		if (formatVersion != 1 || model != "flexglue") {
			VERBOSE("error: the FlexGlue manifest '%s' is version %d of model '%s', expected version 1 of 'flexglue'", fileName.c_str(), formatVersion, model.c_str());
			return false;
		}
		key = "setting"; setting = data.at(key).get<std::string>();
		key = "image_size"; imageSize = data.at(key).get<int>();
		key = "warp_size"; warpSize = data.at(key).get<int>();
		key = "retrieval_recipes"; retrievalDim = data.at(key).at("features").at("dim").get<unsigned>();
		key = "files"; descriptorFile = data.at(key).at("descriptor").get<std::string>(); matchFile = data.at(key).at("match").get<std::string>();
		key = "heads"; const std::string poseHead = data.at(key).at("pose").at("name").get<std::string>();
		if (poseHead != "H1") {
			VERBOSE("error: the FlexGlue manifest '%s' carries the pose head '%s', this build reads H1", fileName.c_str(), poseHead.c_str());
			return false;
		}
	} catch (const nlohmann::json::exception& e) {
		VERBOSE("error: the FlexGlue manifest '%s' lacks the key '%s' or it has the wrong type (%s)", fileName.c_str(), key, e.what());
		return false;
	}
	return true;
}

#ifdef _USE_ONNXRUNTIME

// only read by an ASSERT (which MSVC Code Analysis also reads in a release build)
[[maybe_unused]] static bool ConfidenceInRange(const Image32F& confidence)
{
	double minValue, maxValue;
	cv::minMaxLoc(confidence, &minValue, &maxValue);
	return minValue >= 0. && maxValue <= 1.;
}

bool FlexGlueMatcher::Load(const String& _modelDir, const String& setting, const String& providerName)
{
	ASSERT(!_modelDir.empty() && !setting.empty() && !IsLoaded());
	String modelDir(_modelDir);
	Util::ensureFolderSlash(modelDir);
	FlexGlueManifest manifest;
	if (!manifest.Load(modelDir + "flexglue_" + setting + ".json"))
		return false;
	if (manifest.setting != setting || manifest.imageSize != IMAGE_SIZE || manifest.warpSize != WARP_GRID || manifest.retrievalDim != RETRIEVAL_SIZE) {
		VERBOSE("error: the FlexGlue manifest of '%s' describes setting '%s' with %d px images, a %d-cell warp grid and %u-D retrieval; this build reads '%s', %d px, %d cells, %d-D",
			modelDir.c_str(), manifest.setting.c_str(), manifest.imageSize, manifest.warpSize, manifest.retrievalDim, setting.c_str(), IMAGE_SIZE, WARP_GRID, RETRIEVAL_SIZE);
		return false;
	}
	OnnxModel::Options options;
	options.provider = OnnxProviderFromString(providerName);
	if (!descriptor.Load(modelDir + manifest.descriptorFile, options))
		return false;
	if (descriptor.Provider() == OnnxProvider::CPU)
		VERBOSE("warning: FlexGlue running on the CPU execution provider (seconds per image)");
	options.provider = descriptor.Provider();
	if (!match.Load(modelDir + manifest.matchFile, options)) {
		descriptor.Unload();
		return false;
	}
	const std::vector<int64_t> featuresShape{1, FEATURE_CHANNELS, GRID, GRID};
	const std::vector<int64_t> warpShape{1, WARP_GRID, WARP_GRID, 2};
	const std::vector<int64_t> confidenceShape{1, WARP_GRID, WARP_GRID};
	if (!descriptor.ExpectInputShape("image", {1, 3, IMAGE_SIZE, IMAGE_SIZE}) ||
		!descriptor.ExpectOutputShape("features", featuresShape) ||
		!descriptor.ExpectOutputShape("retrieval", {1, RETRIEVAL_SIZE}) ||
		!match.ExpectInputShape("features0", featuresShape) || !match.ExpectInputShape("features1", featuresShape) ||
		!match.ExpectInputShape("image_size0", {1, 2}) || !match.ExpectInputShape("image_size1", {1, 2}) ||
		!match.ExpectOutputShape("warp0", warpShape) || !match.ExpectOutputShape("warp1", warpShape) ||
		!match.ExpectOutputShape("confidence0", confidenceShape) || !match.ExpectOutputShape("confidence1", confidenceShape) ||
		!match.ExpectOutputShape("covisibility", {1}) ||
		!match.ExpectOutputShape("R10", {1, 3, 3}) || !match.ExpectOutputShape("t10", {1, 3})) {
		VERBOSE("error: the FlexGlue graphs of '%s' do not have the inputs and outputs of the bundle format this build reads", modelDir.c_str());
		descriptor.Unload(); match.Unload();
		return false;
	}
	image = OrtTensor::Host({1, 3, IMAGE_SIZE, IMAGE_SIZE});
	retrieval = OrtTensor::Host({1, RETRIEVAL_SIZE});
	covisibility = OrtTensor::Host({1});
	for (int i = 0; i < 2; ++i) {
		imageSize[i] = OrtTensor::Host({1, 2});
		warp[i] = OrtTensor::Host(warpShape);
		confidence[i] = OrtTensor::Host(confidenceShape);
	}
	rotation = OrtTensor::Host({1, 3, 3});
	direction = OrtTensor::Host({1, 3});
	if (!image.IsValid() || !retrieval.IsValid() || !covisibility.IsValid() ||
		!imageSize[0].IsValid() || !imageSize[1].IsValid() ||
		!warp[0].IsValid() || !warp[1].IsValid() ||
		!confidence[0].IsValid() || !confidence[1].IsValid() ||
		!rotation.IsValid() || !direction.IsValid()) {
		VERBOSE("error: can not allocate FlexGlue scratch tensors");
		descriptor.Unload(); match.Unload();
		return false;
	}
	DEBUG("FlexGlue model '%s' ready on %s: %dx%d images, %dx%d feature grid, %dx%d warp cells",
		setting.c_str(), OnnxProviderName(descriptor.Provider()), IMAGE_SIZE, IMAGE_SIZE, GRID, GRID, WARP_GRID, WARP_GRID);
	return true;
}

String FlexGlueMatcher::ProviderName() const
{
	return OnnxProviderName(descriptor.Provider());
}

OrtTensor FlexGlueMatcher::MakeCode()
{
	ASSERT(IsLoaded());
	return descriptor.MakeDeviceTensor({1, FEATURE_CHANNELS, GRID, GRID});
}

void FlexGlueMatcher::Preprocess(const Image8U3& bgr, std::vector<float>& planarRgb) const
{
	ASSERT(!bgr.empty());
	Image32F3 f32, resized;
	bgr.convertTo(f32, CV_32FC3, 1.0 / 255.0);
	cv::resize(f32, resized, cv::Size(IMAGE_SIZE, IMAGE_SIZE), 0, 0, cv::INTER_LINEAR);
	const size_t plane = (size_t)IMAGE_SIZE * IMAGE_SIZE;
	planarRgb.resize(3 * plane);
	float* dstR = planarRgb.data();
	float* dstG = dstR + plane;
	float* dstB = dstG + plane;
	for (int y = 0; y < IMAGE_SIZE; ++y) {
		const Pixel32F* row = resized.ptr<Pixel32F>(y);
		const size_t rowOffset = (size_t)y * IMAGE_SIZE;
		for (int x = 0; x < IMAGE_SIZE; ++x) {
			dstR[rowOffset + x] = row[x].r;
			dstG[rowOffset + x] = row[x].g;
			dstB[rowOffset + x] = row[x].b;
		}
	}
}

bool FlexGlueMatcher::DescribeImage(const float* planarRgb, OrtTensor& features, std::vector<float>& retrievalOut)
{
	static const std::vector<int64_t> kFeaturesShape{1, FEATURE_CHANNELS, GRID, GRID};
	ASSERT(IsLoaded() && planarRgb != NULL && features.IsValid() && features.Shape() == kFeaturesShape);
	memcpy(image.HostData(), planarRgb, image.NumElements() * sizeof(float));
	descriptor.ClearBindings();
	if (!(descriptor.BindInput("image", image) && descriptor.BindOutput("features", features) &&
		descriptor.BindOutput("retrieval", retrieval) && descriptor.Run()))
		return false;
	retrievalOut.assign(retrieval.HostData(), retrieval.HostData() + RETRIEVAL_SIZE);
	return true;
}

bool FlexGlueMatcher::MatchPair(const OrtTensor& featuresA, const OrtTensor& featuresB, const PairInput& input, PairOutput& out)
{
	ASSERT(IsLoaded() && featuresA.IsValid() && featuresB.IsValid() && input.sizeA.area() > 0 && input.sizeB.area() > 0);
	imageSize[0].HostData()[0] = (float)input.sizeA.width; imageSize[0].HostData()[1] = (float)input.sizeA.height;
	imageSize[1].HostData()[0] = (float)input.sizeB.width; imageSize[1].HostData()[1] = (float)input.sizeB.height;
	match.ClearBindings();
	if (!(match.BindInput("features0", featuresA) && match.BindInput("features1", featuresB) &&
		match.BindInput("image_size0", imageSize[0]) && match.BindInput("image_size1", imageSize[1]) &&
		match.BindOutput("warp0", warp[0]) && match.BindOutput("confidence0", confidence[0]) &&
		match.BindOutput("warp1", warp[1]) && match.BindOutput("confidence1", confidence[1]) &&
		match.BindOutput("covisibility", covisibility) &&
		match.BindOutput("R10", rotation) && match.BindOutput("t10", direction) && match.Run()))
		return false;
	const size_t cells = (size_t)WARP_GRID * WARP_GRID;
	out.ab.warp.create(WARP_GRID, WARP_GRID); out.ab.confidence.create(WARP_GRID, WARP_GRID);
	out.ba.warp.create(WARP_GRID, WARP_GRID); out.ba.confidence.create(WARP_GRID, WARP_GRID);
	memcpy(out.ab.warp.data, warp[0].HostData(), cells * sizeof(Point2f));
	memcpy(out.ba.warp.data, warp[1].HostData(), cells * sizeof(Point2f));
	memcpy(out.ab.confidence.data, confidence[0].HostData(), cells * sizeof(float));
	memcpy(out.ba.confidence.data, confidence[1].HostData(), cells * sizeof(float));
	out.covisibility = covisibility.HostData()[0];
	ASSERT(*out.covisibility >= 0.f && *out.covisibility <= 1.f);
	ASSERT(ConfidenceInRange(out.ab.confidence) && ConfidenceInRange(out.ba.confidence));
	// the H1 head's pose, in the cameras of the two images as given: x_B = R x_A + s t, s > 0 unknown
	PosePrior prior;
	memcpy(prior.R.val, rotation.HostData(), 9 * sizeof(float));
	prior.t = Point3f(direction.HostData()[0], direction.HostData()[1], direction.HostData()[2]);
	// the graph builds both by construction (Gram-Schmidt and cross products of normalised vectors, a normalised direction), in float32:
	// their error is around 1e-6, so 1e-4 only leaves room for the arithmetic of the provider and catches a broken graph
	ASSERT(cv::norm(prior.R.t() * prior.R - Matrix3x3f::IDENTITY) < 1e-4 && ABS(cv::determinant(prior.R) - 1.0) < 1e-4 && ABS(cv::norm(prior.t) - 1.0) < 1e-4);
	out.pose = prior;
	return true;
}

#endif // _USE_ONNXRUNTIME

#pragma pop_macro("VERBOSE")
