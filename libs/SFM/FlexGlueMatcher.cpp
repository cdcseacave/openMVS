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

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("FlexGlue"));

#ifdef _USE_ONNXRUNTIME

// Every cell of a confidence map is a probability
static bool ConfidenceInRange(const Image32F& confidence)
{
	double minValue, maxValue;
	cv::minMaxLoc(confidence, &minValue, &maxValue);
	return minValue >= 0. && maxValue <= 1.;
}

bool FlexGlueMatcher::Load(const String& _modelDir, const String&, const String& providerName)
{
	ASSERT(!_modelDir.empty() && !IsLoaded());
	String modelDir(_modelDir);
	Util::ensureFolderSlash(modelDir);
	OnnxModel::Options options;
	options.provider = OnnxProviderFromString(providerName);
	if (!descriptor.Load(modelDir + "flexglue_descriptor.onnx", options))
		return false;
	options.provider = descriptor.Provider(); // every graph runs where the descriptor does
	if (!dense.Load(modelDir + "flexglue_dense.onnx", options) || !pose.Load(modelDir + "flexglue_pose_h0.onnx", options))
		return false;
	const std::vector<int64_t> featuresShape{1, FEATURE_CHANNELS, GRID, GRID};
	const std::vector<int64_t> pooledShape{1, POOLED_CHANNELS, POOLED_GRID, POOLED_GRID};
	const std::vector<int64_t> warpShape{1, WARP_GRID, WARP_GRID, 2};
	const std::vector<int64_t> confidenceShape{1, WARP_GRID, WARP_GRID};
	if (!descriptor.ExpectInputShape("image", {1, 3, IMAGE_SIZE, IMAGE_SIZE}) ||
		!descriptor.ExpectOutputShape("features", featuresShape) ||
		!descriptor.ExpectOutputShape("retrieval", {1, RETRIEVAL_SIZE}) ||
		!dense.ExpectInputShape("features0", featuresShape) || !dense.ExpectInputShape("features1", featuresShape) ||
		!dense.ExpectInputShape("image_size0", {1, 2}) || !dense.ExpectInputShape("image_size1", {1, 2}) ||
		!dense.ExpectOutputShape("warp0", warpShape) || !dense.ExpectOutputShape("warp1", warpShape) ||
		!dense.ExpectOutputShape("confidence0", confidenceShape) || !dense.ExpectOutputShape("confidence1", confidenceShape) ||
		!dense.ExpectOutputShape("covisibility", {1}) ||
		!dense.ExpectOutputShape("pooled0", pooledShape) || !dense.ExpectOutputShape("pooled1", pooledShape) ||
		!pose.ExpectInputShape("pooled0", pooledShape) || !pose.ExpectInputShape("pooled1", pooledShape) ||
		!pose.ExpectInputShape("intrinsics0", {1, 4}) || !pose.ExpectInputShape("intrinsics1", {1, 4}) ||
		!pose.ExpectOutputShape("R10", {1, 3, 3}) || !pose.ExpectOutputShape("t10", {1, 3})) {
		VERBOSE("error: can not load the FlexGlue graphs from '%s'", modelDir.c_str());
		descriptor.Unload(); dense.Unload(); pose.Unload();
		return false;
	}
	image = OrtTensor::Host({1, 3, IMAGE_SIZE, IMAGE_SIZE});
	retrieval = OrtTensor::Host({1, RETRIEVAL_SIZE});
	covisibility = OrtTensor::Host({1});
	for (int i = 0; i < 2; ++i) {
		imageSize[i] = OrtTensor::Host({1, 2});
		warp[i] = OrtTensor::Host(warpShape);
		confidence[i] = OrtTensor::Host(confidenceShape);
		pooled[i] = dense.MakeDeviceTensor(pooledShape);
	}
	for (int i = 0; i < 2; ++i)
		intrinsics[i] = OrtTensor::Host({1, 4});
	rotation = OrtTensor::Host({1, 3, 3});
	direction = OrtTensor::Host({1, 3});
	// a failed allocation (device memory) is the outside world's failure, not a broken invariant
	if (!pooled[0].IsValid() || !pooled[1].IsValid()) {
		VERBOSE("error: can not allocate the FlexGlue pooled feature tensors on the %s provider", ProviderName().c_str());
		return false;
	}
	ASSERT(image.IsValid() && retrieval.IsValid() && covisibility.IsValid() &&
		imageSize[0].IsValid() && imageSize[1].IsValid() && warp[0].IsValid() && warp[1].IsValid() && confidence[0].IsValid() && confidence[1].IsValid() &&
		intrinsics[0].IsValid() && intrinsics[1].IsValid() && rotation.IsValid() && direction.IsValid());
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
	cv::Mat rgb, resized;
	cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
	rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
	cv::resize(rgb, resized, cv::Size(IMAGE_SIZE, IMAGE_SIZE), 0, 0, cv::INTER_LINEAR);
	const size_t plane = (size_t)IMAGE_SIZE * IMAGE_SIZE;
	planarRgb.resize(3 * plane);
	for (int y = 0; y < IMAGE_SIZE; ++y) {
		const cv::Vec3f* row = resized.ptr<cv::Vec3f>(y);
		for (int x = 0; x < IMAGE_SIZE; ++x) {
			const size_t i = (size_t)y * IMAGE_SIZE + x;
			planarRgb[i] = row[x][0];
			planarRgb[plane + i] = row[x][1];
			planarRgb[2 * plane + i] = row[x][2];
		}
	}
}

bool FlexGlueMatcher::DescribeImage(const float* planarRgb, OrtTensor& features, std::vector<float>& retrievalOut)
{
	ASSERT(IsLoaded() && planarRgb != NULL && features.Shape() == std::vector<int64_t>({1, FEATURE_CHANNELS, GRID, GRID}));
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
	dense.ClearBindings();
	if (!(dense.BindInput("features0", featuresA) && dense.BindInput("features1", featuresB) &&
		dense.BindInput("image_size0", imageSize[0]) && dense.BindInput("image_size1", imageSize[1]) &&
		dense.BindOutput("warp0", warp[0]) && dense.BindOutput("confidence0", confidence[0]) &&
		dense.BindOutput("warp1", warp[1]) && dense.BindOutput("confidence1", confidence[1]) &&
		dense.BindOutput("covisibility", covisibility) &&
		dense.BindOutput("pooled0", pooled[0]) && dense.BindOutput("pooled1", pooled[1]) && dense.Run()))
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
	// the head reads the cameras in the 512 x 512 frame the features were made in
	const PinholeIntrinsics* intr[2] = {&input.intrA, &input.intrB};
	const cv::Size* size[2] = {&input.sizeA, &input.sizeB};
	for (int i = 0; i < 2; ++i) {
		const float sx = (float)IMAGE_SIZE / size[i]->width, sy = (float)IMAGE_SIZE / size[i]->height;
		ASSERT(intr[i]->fx > 0.f && intr[i]->fy > 0.f);
		float* k = intrinsics[i].HostData();
		k[0] = intr[i]->fx * sx; k[1] = intr[i]->fy * sy; k[2] = intr[i]->cx * sx; k[3] = intr[i]->cy * sy;
	}
	pose.ClearBindings();
	if (!(pose.BindInput("pooled0", pooled[0]) && pose.BindInput("pooled1", pooled[1]) &&
		pose.BindInput("intrinsics0", intrinsics[0]) && pose.BindInput("intrinsics1", intrinsics[1]) &&
		pose.BindOutput("R10", rotation) && pose.BindOutput("t10", direction) && pose.Run()))
		return false;
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
