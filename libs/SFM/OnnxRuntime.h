/*
 * OnnxRuntime.h
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

#ifndef _SFM_ONNXRUNTIME_H_
#define _SFM_ONNXRUNTIME_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h" // SFM_API, String

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef _USE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

#ifdef _USE_ONNXRUNTIME

// Execution provider requested for/selected by an OnnxModel: AUTO tries CUDA, then CoreML,
// then DirectML, then falls back to CPU, in that order, keeping the first one that loads
enum class OnnxProvider : uint8_t {
	AUTO = 0,
	CUDA = 1,
	COREML = 2,
	DML = 3,
	CPU = 4
};

// Parse the --roma2-provider option ("auto"|"cuda"|"coreml"|"dml"|"cpu"); anything else maps to AUTO
SFM_API OnnxProvider OnnxProviderFromString(const String& name);

// Human-readable name of a provider ("CUDA"|"CoreML"|"DirectML"|"CPU"|"AUTO"), for log messages only
SFM_API const char* OnnxProviderName(OnnxProvider provider);

// A single fp32, static-shape ONNX Runtime tensor: either host-owned (a std::vector<float>
// wrapped in a non-owning CPU Ort::Value) or allocated from an OnnxModel's device arena
// (Ort::Allocator over the session's execution provider) when that provider keeps tensors
// resident on the device (CUDA only; CoreML/DirectML/CPU always take host tensors through
// IoBinding, so MakeDeviceTensor silently hands back a host tensor there).
// Move-only. Lifetime: a tensor made by OnnxModel::MakeDeviceTensor must be destroyed
// before the OnnxModel it came from (it may reference that session's device allocator).
class SFM_API OrtTensor
{
public:
	OrtTensor() = default;
	OrtTensor(const OrtTensor&) = delete;
	OrtTensor& operator=(const OrtTensor&) = delete;
	OrtTensor(OrtTensor&&) = default;
	OrtTensor& operator=(OrtTensor&&) = default;
	~OrtTensor() = default;

	// Host-owned tensor of the given shape, zero-initialized; always available regardless
	// of the active execution provider
	static OrtTensor Host(const std::vector<int64_t>& shape);

	inline bool IsValid() const { return value != nullptr; }
	inline bool IsOnDevice() const { return bOnDevice; }
	inline const std::vector<int64_t>& Shape() const { return shape; }
	size_t NumElements() const;
	inline float* HostData() { return host.empty() ? NULL : host.data(); }
	inline const float* HostData() const { return host.empty() ? NULL : host.data(); }
	inline const Ort::Value& Value() const { return value; }

private:
	friend class OnnxModel;
	std::vector<int64_t> shape;
	std::vector<float> host;          // host-owned backing buffer (empty for device tensors)
	Ort::MemoryInfo memInfo{nullptr}; // only set for host tensors (CreateCpu)
	Ort::Value value{nullptr};
	bool bOnDevice = false;
};

// Portable wrapper around one ONNX Runtime session: picks the first available execution
// provider (or the one forced through Options), exposes its static input/output shapes,
// and runs it through IoBinding so callers can mix host and device tensors freely (ORT
// copies host inputs to the device inside Run, and copies a device output back to the
// host there too when the bound output tensor is host-owned).
class SFM_API OnnxModel
{
public:
	struct Options {
		OnnxProvider provider = OnnxProvider::AUTO;
		int deviceID = 0;
		size_t gpuMemLimit = 0; // 0 = no limit (SIZE_MAX)
		bool bTF32 = true;      // allow TF32 on the CUDA provider
		int numIntraOpThreads = 0; // 0 = ORT default
	};

	OnnxModel() = default;
	OnnxModel(const OnnxModel&) = delete;
	OnnxModel& operator=(const OnnxModel&) = delete;
	OnnxModel(OnnxModel&&) = default;
	OnnxModel& operator=(OnnxModel&&) = default;
	~OnnxModel() = default;

	// Load the model, trying providers in preference order (or just the forced one from
	// options.provider), logging the execution provider it ends up on; rejects a model with
	// any dynamic (negative) input/output dimension. Lifetime: any OrtTensor made by
	// MakeDeviceTensor must be destroyed before this OnnxModel is (see OrtTensor).
	// (Two overloads rather than a defaulted `= Options()` argument: GCC rejects a nested
	// class's default member initializers used as a default argument of a sibling member
	// function before the enclosing class is complete.)
	bool Load(const String& fileName, const Options& options);
	inline bool Load(const String& fileName) { return Load(fileName, Options()); }

	inline bool IsLoaded() const { return session != nullptr; }
	inline OnnxProvider Provider() const { return provider; }
	bool IsOnDevice() const; // true only for CUDA (device-resident tensors possible)

	// Allocate a tensor from this session's device arena; a host tensor when !IsOnDevice()
	OrtTensor MakeDeviceTensor(const std::vector<int64_t>& shape);

	// Look the name up in the model's static input/output metadata and VERBOSE a mismatch
	// listing both shapes; false if the name is not one of the model's inputs/outputs
	bool ExpectInputShape(const String& name, const std::vector<int64_t>& shape) const;
	bool ExpectOutputShape(const String& name, const std::vector<int64_t>& shape) const;

	// Static shape of the named graph input, or NULL when the model has no such input;
	// for callers that adapt to an optional input instead of requiring it
	const std::vector<int64_t>* InputShape(const String& name) const;

	void ClearBindings();
	// false + VERBOSE on a shape mismatch (name unknown, or shape disagrees) or an ORT
	// binding failure; callers may ignore the result (Run() still fails safely on a bad
	// binding, this just gives an earlier, more specific diagnostic)
	bool BindInput(const String& name, const OrtTensor& tensor);  // host tensors are copied to the device by ORT
	bool BindOutput(const String& name, OrtTensor& tensor);       // host tensor => ORT copies device->host in Run()
	bool Run();

	// Release the session and everything derived from it, returning to the not-loaded state.
	// Used on any failure path once the session itself has already been constructed, so
	// IsLoaded() reliably reports false after a failed Load(). Callers that need to drop a
	// loaded model must call this rather than move-assign an empty OnnxModel over it: member-wise
	// assignment would release the session before the binding and the device allocator that were
	// created from it, while this tears them down in the required reverse order.
	void Unload();

private:
	Options options;
	OnnxProvider provider = OnnxProvider::CPU;
	Ort::SessionOptions sessionOptions{nullptr};
	Ort::Session session{nullptr};
	Ort::IoBinding binding{nullptr};
	std::unique_ptr<Ort::Allocator> deviceAllocator; // set only when IsOnDevice()
	std::vector<std::pair<std::string, std::vector<int64_t>>> inputs;
	std::vector<std::pair<std::string, std::vector<int64_t>>> outputs;
};

// Process-wide ONNX Runtime environment (one OrtEnv per process, as ORT requires), routing
// its log messages through openMVS logging
SFM_API Ort::Env& OnnxEnv();

#else // _USE_ONNXRUNTIME

// Inert stand-in so code written against OrtTensor (e.g. MatchROMA2.cpp) compiles unchanged
// in builds without ONNX Runtime: every accessor reports "no tensor", nothing is ever valid.
class SFM_API OrtTensor
{
public:
	OrtTensor() = default;

	static OrtTensor Host(const std::vector<int64_t>&) { return OrtTensor(); }

	inline bool IsValid() const { return false; }
	inline bool IsOnDevice() const { return false; }
	inline const std::vector<int64_t>& Shape() const { static const std::vector<int64_t> kEmpty; return kEmpty; }
	inline size_t NumElements() const { return 0; }
	inline float* HostData() { return NULL; }
	inline const float* HostData() const { return NULL; }
};

#endif // _USE_ONNXRUNTIME

/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_ONNXRUNTIME_H_
