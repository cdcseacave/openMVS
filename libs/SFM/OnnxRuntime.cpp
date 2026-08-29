/*
 * OnnxRuntime.cpp
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
#include "OnnxRuntime.h"

#ifdef _USE_ONNXRUNTIME

using namespace SFM;

// optional execution-provider headers: present only on the OSs that ship them (macOS for
// CoreML, some Windows ORT packages for DirectML); both branches compile out on Linux
#if __has_include(<coreml_provider_factory.h>)
#include <coreml_provider_factory.h>
#define _ORT_HAS_COREML_HEADER
#endif
#if __has_include(<dml_provider_factory.h>)
#include <dml_provider_factory.h>
#define _ORT_HAS_DML_HEADER
#endif


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ONNXRT  "));

namespace {

// Human-readable name of a provider, for log messages only
const char* ProviderName(OnnxProvider provider)
{
	switch (provider) {
	case OnnxProvider::CUDA: return "CUDA";
	case OnnxProvider::COREML: return "CoreML";
	case OnnxProvider::DML: return "DirectML";
	case OnnxProvider::CPU: return "CPU";
	default: return "AUTO";
	}
}

// Format a shape like "[1,3,320,320]", for mismatch log messages only
std::string ShapeToString(const std::vector<int64_t>& shape)
{
	std::string str("[");
	for (size_t i = 0; i < shape.size(); ++i) {
		if (i) str += ',';
		str += std::to_string(shape[i]);
	}
	str += ']';
	return str;
}

// Route ONNX Runtime's own log messages through openMVS logging
void ORT_API_CALL OnnxLogCallback(void*, OrtLoggingLevel severity, const char* category, const char*, const char* location, const char* message)
{
	switch (severity) {
	case ORT_LOGGING_LEVEL_VERBOSE:
	case ORT_LOGGING_LEVEL_INFO:
		DEBUG_EXTRA("ONNX Runtime [%s]: %s", category, message);
		break;
	case ORT_LOGGING_LEVEL_WARNING:
		DEBUG("ONNX Runtime warning [%s]: %s", category, message);
		break;
	default:
		VERBOSE("ONNX Runtime error [%s]: %s (%s)", category, message, location);
		break;
	}
}

// ORT takes the model path as ORTCHAR_T*: wchar_t on Windows, char elsewhere (openMVS
// equates "Windows" with "built by MSVC", Types.h:12-14, so _MSC_VER is the right guard here)
std::basic_string<ORTCHAR_T> ToOrtPath(const String& path)
{
	#ifdef _MSC_VER
	return Util::toWString(path); // UTF-8 -> UTF-16
	#else
	return std::basic_string<ORTCHAR_T>(path.begin(), path.end());
	#endif
}

// Providers this binary was linked with, in preference order, filtered by the request;
// a specifically requested (non-AUTO) provider that is not available gets a warning before
// silently falling through to CPU
std::vector<OnnxProvider> ProviderCandidates(OnnxProvider requested)
{
	const std::vector<std::string> available(Ort::GetAvailableProviders());
	const auto has = [&](const char* name) { return std::find(available.begin(), available.end(), name) != available.end(); };
	std::vector<OnnxProvider> candidates;
	if (has("CUDAExecutionProvider")) {
		if (requested == OnnxProvider::AUTO || requested == OnnxProvider::CUDA)
			candidates.push_back(OnnxProvider::CUDA);
	} else if (requested == OnnxProvider::CUDA)
		VERBOSE("warning: requested execution provider CUDA is not available in this build");
	if (has("CoreMLExecutionProvider")) {
		if (requested == OnnxProvider::AUTO || requested == OnnxProvider::COREML)
			candidates.push_back(OnnxProvider::COREML);
	} else if (requested == OnnxProvider::COREML)
		VERBOSE("warning: requested execution provider CoreML is not available in this build");
	if (has("DmlExecutionProvider")) {
		if (requested == OnnxProvider::AUTO || requested == OnnxProvider::DML)
			candidates.push_back(OnnxProvider::DML);
	} else if (requested == OnnxProvider::DML)
		VERBOSE("warning: requested execution provider DirectML is not available in this build");
	if (requested == OnnxProvider::AUTO || requested == OnnxProvider::CPU || candidates.empty())
		candidates.push_back(OnnxProvider::CPU);
	return candidates;
}

// Register one execution provider on the session options; throws Ort::Exception on failure
// (caught by the retry loop in OnnxModel::Load)
void AppendProvider(Ort::SessionOptions& so, OnnxProvider provider, const OnnxModel::Options& options)
{
	const OrtApi& api(Ort::GetApi());
	switch (provider) {
	case OnnxProvider::CUDA: {
		OrtCUDAProviderOptionsV2* p = NULL;
		Ort::ThrowOnError(api.CreateCUDAProviderOptions(&p));
		std::unique_ptr<OrtCUDAProviderOptionsV2, void(ORT_API_CALL*)(OrtCUDAProviderOptionsV2*)> cudaOptions(p, api.ReleaseCUDAProviderOptions);
		const std::string deviceID(std::to_string(options.deviceID)), memLimit(std::to_string(options.gpuMemLimit ? options.gpuMemLimit : (size_t)SIZE_MAX));
		const char* keys[] = {"device_id", "gpu_mem_limit", "arena_extend_strategy", "cudnn_conv_algo_search", "do_copy_in_default_stream", "use_tf32"};
		const char* values[] = {deviceID.c_str(), memLimit.c_str(), "kSameAsRequested", "EXHAUSTIVE", "1", options.bTF32 ? "1" : "0"};
		Ort::ThrowOnError(api.UpdateCUDAProviderOptions(cudaOptions.get(), keys, values, 6));
		so.AppendExecutionProvider_CUDA_V2(*cudaOptions);
		break; }
	case OnnxProvider::COREML: {
		// generic registration API (ORT >= 1.20): MLProgram format, all compute units;
		// option key names come from the CoreML EP header when it's available (macOS), else
		// the (identical) string literals it documents for the generic AppendExecutionProvider
		#ifdef _ORT_HAS_COREML_HEADER
		std::unordered_map<std::string, std::string> coreml{{kCoremlProviderOption_ModelFormat, "MLProgram"}, {kCoremlProviderOption_MLComputeUnits, "ALL"}};
		#else
		std::unordered_map<std::string, std::string> coreml{{"ModelFormat", "MLProgram"}, {"MLComputeUnits", "ALL"}};
		#endif
		so.AppendExecutionProvider("CoreML", coreml);
		break; }
	case OnnxProvider::DML: {
		#ifdef _ORT_HAS_DML_HEADER
		Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_DML(so, options.deviceID));
		so.DisableMemPattern(); so.SetExecutionMode(ORT_SEQUENTIAL); // DML requirements
		#else
		throw Ort::Exception("DirectML provider header not available in this build", ORT_FAIL);
		#endif
		break; }
	case OnnxProvider::CPU:
	default:
		break;
	}
}

// Look name up in io, VERBOSE a mismatch listing both shapes; false if not found
bool ExpectShape(const std::vector<std::pair<std::string, std::vector<int64_t>>>& io, const String& name, const std::vector<int64_t>& shape, const char* kind)
{
	for (const auto& entry : io) {
		if (entry.first != name.c_str())
			continue;
		if (entry.second == shape)
			return true;
		VERBOSE("error: ONNX %s '%s' shape mismatch: expected %s, got %s",
			kind, name.c_str(), ShapeToString(entry.second).c_str(), ShapeToString(shape).c_str());
		return false;
	}
	VERBOSE("error: ONNX %s '%s' not found in the model", kind, name.c_str());
	return false;
}

} // namespace

OnnxProvider SFM::OnnxProviderFromString(const String& name)
{
	if (name == "auto") return OnnxProvider::AUTO;
	if (name == "cuda") return OnnxProvider::CUDA;
	if (name == "coreml") return OnnxProvider::COREML;
	if (name == "dml") return OnnxProvider::DML;
	if (name == "cpu") return OnnxProvider::CPU;
	VERBOSE("warning: unrecognized ONNX Runtime execution provider '%s', using auto", name.c_str());
	return OnnxProvider::AUTO;
}

Ort::Env& SFM::OnnxEnv()
{
	static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "OpenMVS", OnnxLogCallback, NULL);
	return env;
}

size_t OrtTensor::NumElements() const
{
	if (shape.empty())
		return 0;
	size_t n = 1;
	for (const int64_t d : shape)
		n *= (size_t)d;
	return n;
}

OrtTensor OrtTensor::Host(const std::vector<int64_t>& shape)
{
	ASSERT(!shape.empty());
	OrtTensor t;
	t.shape = shape;
	try {
		t.host.assign(t.NumElements(), 0.f);
		t.memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
		t.value = Ort::Value::CreateTensor<float>(t.memInfo, t.host.data(), t.host.size(), shape.data(), shape.size()); // non-owning over host
	} catch (const std::exception& e) { // Ort::Exception or std::bad_alloc from the host vector
		VERBOSE("error: can not allocate a host tensor of %s (%s)",
			Util::formatBytes((int64_t)(t.NumElements() * sizeof(float))).c_str(), e.what());
		return OrtTensor();
	}
	return t;
}

bool OnnxModel::Load(const String& fileName, const Options& _options)
{
	options = _options;
	#ifdef _USE_CUDA
	if (options.provider == OnnxProvider::AUTO && SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs))
		options.provider = OnnxProvider::CPU; // --gpu-device -2
	if (!SEACAVE::CUDA::devices.IsEmpty())
		options.deviceID = SEACAVE::CUDA::devices[0].ID; // the device initDevices() selected (UtilCUDA.cpp:125-160)
	#endif
	if (!File::isFile(fileName)) {
		VERBOSE("error: ONNX model '%s' not found", fileName.c_str());
		return false;
	}
	const std::basic_string<ORTCHAR_T> ortPath(ToOrtPath(fileName));
	for (const OnnxProvider candidate : ProviderCandidates(options.provider)) {
		try {
			sessionOptions = Ort::SessionOptions();
			sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
			if (options.numIntraOpThreads > 0)
				sessionOptions.SetIntraOpNumThreads(options.numIntraOpThreads);
			AppendProvider(sessionOptions, candidate, options);
			session = Ort::Session(OnnxEnv(), ortPath.c_str(), sessionOptions);
			provider = candidate;
			break;
		} catch (const Ort::Exception& e) {
			VERBOSE("warning: ONNX Runtime provider %s unavailable for '%s' (%s)",
				ProviderName(candidate), Util::getFileNameExt(fileName).c_str(), e.what());
		}
	}
	if (session == nullptr) {
		VERBOSE("error: can not load ONNX model '%s' on any execution provider", fileName.c_str());
		return false;
	}
	// static I/O metadata; reject dynamic dims (static graphs only, like polycpp matcher.cpp:408-414);
	// every ORT call below can throw (unlikely, but Load() is contracted to log and return
	// false rather than let an exception escape), so the whole block is guarded and any
	// failure resets the model back to the not-loaded state
	try {
		Ort::AllocatorWithDefaultOptions allocator;
		inputs.clear(); outputs.clear();
		for (size_t i = 0; i < session.GetInputCount(); ++i)
			inputs.emplace_back(session.GetInputNameAllocated(i, allocator).get(),
				session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
		for (size_t i = 0; i < session.GetOutputCount(); ++i)
			outputs.emplace_back(session.GetOutputNameAllocated(i, allocator).get(),
				session.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
		for (const auto& io : inputs)
			for (const int64_t d : io.second)
				if (d < 0) {
					VERBOSE("error: dynamic dimension in input '%s' of '%s'", io.first.c_str(), fileName.c_str());
					Reset();
					return false;
				}
		for (const auto& io : outputs)
			for (const int64_t d : io.second)
				if (d < 0) {
					VERBOSE("error: dynamic dimension in output '%s' of '%s'", io.first.c_str(), fileName.c_str());
					Reset();
					return false;
				}
		if (IsOnDevice())
			deviceAllocator = std::make_unique<Ort::Allocator>(session, Ort::MemoryInfo("Cuda", OrtArenaAllocator, options.deviceID, OrtMemTypeDefault));
		binding = Ort::IoBinding(session);
	} catch (const Ort::Exception& e) {
		VERBOSE("error: can not query ONNX model '%s' (%s)", fileName.c_str(), e.what());
		Reset();
		return false;
	}
	DEBUG("ONNX model '%s' loaded on %s", Util::getFileNameExt(fileName).c_str(), ProviderName(provider));
	return true;
}

void OnnxModel::Reset()
{
	session = Ort::Session(nullptr);
	binding = Ort::IoBinding(nullptr);
	deviceAllocator.reset();
	inputs.clear();
	outputs.clear();
	provider = OnnxProvider::CPU; // the class's own not-loaded default
}

bool OnnxModel::IsOnDevice() const
{
	return provider == OnnxProvider::CUDA; // CoreML/DML take host tensors through IoBinding
}

OrtTensor OnnxModel::MakeDeviceTensor(const std::vector<int64_t>& shape)
{
	if (!IsOnDevice())
		return OrtTensor::Host(shape);
	OrtTensor t;
	t.shape = shape;
	t.bOnDevice = true;
	try {
		t.value = Ort::Value::CreateTensor(*deviceAllocator, shape.data(), shape.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT); // owned by the session's CUDA arena
	} catch (const Ort::Exception& e) {
		VERBOSE("error: can not allocate a device tensor of %s (%s)",
			Util::formatBytes((int64_t)(t.NumElements() * sizeof(float))).c_str(), e.what());
		return OrtTensor();
	}
	return t;
}

bool OnnxModel::ExpectInputShape(const String& name, const std::vector<int64_t>& shape) const
{
	return ExpectShape(inputs, name, shape, "input");
}

bool OnnxModel::ExpectOutputShape(const String& name, const std::vector<int64_t>& shape) const
{
	return ExpectShape(outputs, name, shape, "output");
}

bool OnnxModel::BindInput(const String& name, const OrtTensor& t)
{
	if (!ExpectInputShape(name, t.Shape())) // VERBOSEs the mismatch itself
		return false;
	try {
		binding.BindInput(name.c_str(), t.Value()); // host inputs are copied by ORT
	} catch (const Ort::Exception& e) {
		VERBOSE("error: can not bind input '%s' (%s)", name.c_str(), e.what());
		return false;
	}
	return true;
}

bool OnnxModel::BindOutput(const String& name, OrtTensor& t)
{
	if (!ExpectOutputShape(name, t.Shape())) // VERBOSEs the mismatch itself
		return false;
	try {
		binding.BindOutput(name.c_str(), t.Value()); // host output => ORT copies D2H in Run()
	} catch (const Ort::Exception& e) {
		VERBOSE("error: can not bind output '%s' (%s)", name.c_str(), e.what());
		return false;
	}
	return true;
}

bool OnnxModel::Run()
{
	try {
		Ort::RunOptions runOptions;
		session.Run(runOptions, binding);
		binding.SynchronizeOutputs();
		return true;
	} catch (const Ort::Exception& e) {
		VERBOSE("error: ONNX Runtime run failed: %s", e.what());
		return false;
	}
}

void OnnxModel::ClearBindings()
{
	binding.ClearBoundInputs();
	binding.ClearBoundOutputs();
}

#pragma pop_macro("VERBOSE")

#endif // _USE_ONNXRUNTIME
/*----------------------------------------------------------------*/
