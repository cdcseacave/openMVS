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

// Convert UTF-8 path to ORTCHAR_T string (wchar_t on Windows, char elsewhere)
std::basic_string<ORTCHAR_T> ToOrtPath(const String& path)
{
	#ifdef _MSC_VER
	return Util::toWString(path);
	#else
	return std::basic_string<ORTCHAR_T>(path.begin(), path.end());
	#endif
}

// Available providers in preference order, filtered by requested provider.
std::vector<OnnxProvider> ProviderCandidates(OnnxProvider requested)
{
	const std::vector<std::string> available(Ort::GetAvailableProviders());
	const auto has = [&](const char* name) { return std::find(available.begin(), available.end(), name) != available.end(); };
	static constexpr std::pair<OnnxProvider, const char*> kProviders[] = {
		{ OnnxProvider::CUDA,   "CUDAExecutionProvider" },
		{ OnnxProvider::COREML, "CoreMLExecutionProvider" },
		{ OnnxProvider::DML,    "DmlExecutionProvider" },
	};
	std::vector<OnnxProvider> candidates;
	for (const auto& [prov, epName] : kProviders) {
		if (has(epName)) {
			if (requested == OnnxProvider::AUTO || requested == prov)
				candidates.push_back(prov);
		} else if (requested == prov) {
			VERBOSE("error: requested execution provider %s is not available in this build", OnnxProviderName(prov));
		}
	}
	if (requested == OnnxProvider::AUTO || requested == OnnxProvider::CPU)
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
		VERBOSE("error: ONNX %s '%s' shape mismatch: model declares %s, caller expects %s",
			kind, name.c_str(), ShapeToString(entry.second).c_str(), ShapeToString(shape).c_str());
		return false;
	}
	VERBOSE("error: ONNX %s '%s' not found in the model", kind, name.c_str());
	return false;
}

} // namespace

const char* SFM::OnnxProviderName(OnnxProvider provider)
{
	switch (provider) {
	case OnnxProvider::CUDA: return "CUDA";
	case OnnxProvider::COREML: return "CoreML";
	case OnnxProvider::DML: return "DirectML";
	case OnnxProvider::CPU: return "CPU";
	default: return "AUTO";
	}
}

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
	ASSERT(!shape.empty() && std::all_of(shape.begin(), shape.end(), [](int64_t d) { return d > 0; }));
	OrtTensor t;
	t.shape = shape;
	try {
		t.host.assign(t.NumElements(), 0.f);
		t.memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
		t.value = Ort::Value::CreateTensor<float>(t.memInfo, t.host.data(), t.host.size(), shape.data(), shape.size());
	} catch (const std::exception& e) {
		VERBOSE("error: can not allocate a host tensor of %s (%s)",
			Util::formatBytes((int64_t)(t.NumElements() * sizeof(float))).c_str(), e.what());
		return OrtTensor();
	}
	return t;
}

bool OnnxModel::Load(const String& fileName, const Options& _options)
{
	Options options = _options;
	Unload();
	#ifdef _USE_CUDA
	if (options.provider == OnnxProvider::AUTO && SEACAVE::CUDA::isCpuRequested(SEACAVE::CUDA::desiredDeviceIDs))
		options.provider = OnnxProvider::CPU;
	if (options.deviceID < 0 && !SEACAVE::CUDA::devices.IsEmpty())
		options.deviceID = SEACAVE::CUDA::devices[0].ID;
	#endif
	if (options.deviceID < 0)
		options.deviceID = 0;
	if (!File::isFile(fileName)) {
		VERBOSE("error: ONNX model '%s' not found", fileName.c_str());
		return false;
	}
	const std::basic_string<ORTCHAR_T> ortPath(ToOrtPath(fileName));
	for (const OnnxProvider candidate : ProviderCandidates(options.provider)) {
		try {
			Ort::SessionOptions sessionOptions;
			sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
			if (options.numIntraOpThreads > 0)
				sessionOptions.SetIntraOpNumThreads(options.numIntraOpThreads);
			AppendProvider(sessionOptions, candidate, options);
			session = Ort::Session(OnnxEnv(), ortPath.c_str(), sessionOptions);
			provider = candidate;
			break;
		} catch (const Ort::Exception& e) {
			VERBOSE("warning: ONNX Runtime provider %s unavailable for '%s' (%s)",
				OnnxProviderName(candidate), Util::getFileNameExt(fileName).c_str(), e.what());
		}
	}
	if (session == nullptr) {
		if (options.provider != OnnxProvider::AUTO)
			VERBOSE("error: can not load ONNX model '%s' on the requested %s execution provider",
				fileName.c_str(), OnnxProviderName(options.provider));
		else
			VERBOSE("error: can not load ONNX model '%s' on any execution provider", fileName.c_str());
		return false;
	}
	// Static I/O metadata; reject dynamic dimensions
	try {
		Ort::AllocatorWithDefaultOptions allocator;
		inputs.clear(); outputs.clear();
		for (size_t i = 0; i < session.GetInputCount(); ++i)
			inputs.emplace_back(session.GetInputNameAllocated(i, allocator).get(),
				session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
		for (size_t i = 0; i < session.GetOutputCount(); ++i)
			outputs.emplace_back(session.GetOutputNameAllocated(i, allocator).get(),
				session.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
		const auto CheckStatic = [&](const auto& ioList, const char* kind) -> bool {
			for (const auto& io : ioList)
				for (const int64_t d : io.second)
					if (d < 0) {
						VERBOSE("error: dynamic dimension in %s '%s' of '%s'", kind, io.first.c_str(), fileName.c_str());
						return false;
					}
			return true;
		};
		if (!CheckStatic(inputs, "input") || !CheckStatic(outputs, "output")) {
			Unload();
			return false;
		}
		if (IsOnDevice())
			deviceAllocator = std::make_unique<Ort::Allocator>(session, Ort::MemoryInfo("Cuda", OrtArenaAllocator, options.deviceID, OrtMemTypeDefault));
		binding = Ort::IoBinding(session);
	} catch (const std::exception& e) {
		VERBOSE("error: can not query ONNX model '%s' (%s)", fileName.c_str(), e.what());
		Unload();
		return false;
	}
	DEBUG("ONNX model '%s' loaded on %s", Util::getFileNameExt(fileName).c_str(), OnnxProviderName(provider));
	return true;
}

void OnnxModel::Unload()
{
	// tear down in the reverse of the member declaration order (session, binding,
	// deviceAllocator): the device allocator was created from the session and must be
	// released before it, and the binding was created from the session too
	deviceAllocator.reset();
	binding = Ort::IoBinding(nullptr);
	session = Ort::Session(nullptr);
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
	ASSERT(IsLoaded() && !shape.empty());
	if (!IsOnDevice())
		return OrtTensor::Host(shape);
	OrtTensor t;
	t.shape = shape;
	t.bOnDevice = true;
	try {
		t.value = Ort::Value::CreateTensor(*deviceAllocator, shape.data(), shape.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT);
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

const std::vector<int64_t>* OnnxModel::InputShape(const String& name) const
{
	for (const auto& entry : inputs)
		if (entry.first == name.c_str())
			return &entry.second;
	return NULL;
}

bool OnnxModel::BindInput(const String& name, const OrtTensor& t)
{
	ASSERT(IsLoaded() && t.IsValid());
	if (!ExpectInputShape(name, t.Shape()))
		return false;
	try {
		binding.BindInput(name.c_str(), t.Value());
	} catch (const Ort::Exception& e) {
		VERBOSE("error: can not bind input '%s' (%s)", name.c_str(), e.what());
		return false;
	}
	return true;
}

bool OnnxModel::BindOutput(const String& name, OrtTensor& t)
{
	ASSERT(IsLoaded() && t.IsValid());
	if (!ExpectOutputShape(name, t.Shape()))
		return false;
	try {
		binding.BindOutput(name.c_str(), t.Value());
	} catch (const Ort::Exception& e) {
		VERBOSE("error: can not bind output '%s' (%s)", name.c_str(), e.what());
		return false;
	}
	return true;
}

bool OnnxModel::Run()
{
	ASSERT(IsLoaded());
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
	ASSERT(IsLoaded());
	binding.ClearBoundInputs();
	binding.ClearBoundOutputs();
}

#pragma pop_macro("VERBOSE")

#endif // _USE_ONNXRUNTIME
/*----------------------------------------------------------------*/
