////////////////////////////////////////////////////////////////////
// UtilCUDA.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "UtilCUDA.h"

// GPU device-selection string + CPU-sentinel check, shared by all backends and
// available even when no GPU backend is compiled (UtilCUDA.cpp is always built).
namespace SEACAVE {
namespace CUDA {
GENERAL_API String desiredDeviceIDs("-1");
// returns true for any of: empty, "-2", "cpu"/case-insensitive, "none"
bool isCpuRequested(const String& deviceIDs)
{
	if (deviceIDs.empty() || deviceIDs == "-2")
		return true;
	const String lower(deviceIDs.ToLower());
	return lower == "cpu" || lower == "none";
}
} // namespace CUDA
} // namespace SEACAVE

#ifdef _USE_CUDA


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

namespace CUDA {

// S T R U C T S ///////////////////////////////////////////////////

GENERAL_API Devices devices;

// log the failed CUDA runtime call and abort
void abortCudaCall(cudaError_t error, const char* call, const char* file, int line)
{
	VERBOSE("CUDA error at %s:%d: %s (%s (code %d) - %s)", file, line, call, cudaGetErrorName(error), static_cast<int>(error), cudaGetErrorString(error));
	ASSERT("CudaError" == NULL);
	exit(EXIT_FAILURE);
}
/*----------------------------------------------------------------*/


// the compute capabilities (major*10+minor) the CUDA code is compiled for,
// see CMAKE_CUDA_ARCHITECTURES: native code (SASS), architecture-specific
// native code and the lowest PTX (0 if none)
#ifndef _USE_HIP
static constexpr int cudaArchsSASS[] = {0, _CUDA_ARCHS_SASS};
static constexpr int cudaArchsSASSExact[] = {0, _CUDA_ARCHS_SASS_EXACT};
static constexpr int cudaArchPTX(_CUDA_ARCH_PTX);
#endif

// the CUDA code runs on a device if it holds native code of the same major
// and an equal or lower minor version, architecture-specific native code
// (ex. sm_90a) of the exact version, or PTX of an equal or lower version
// that the driver can compile for it
static bool _isComputeCapabilitySupported(int major, int minor)
{
	#ifdef _USE_HIP
	// HIP enumerates only the devices its runtime can drive, and the kernels are built for the
	// gfx targets of CMAKE_HIP_ARCHITECTURES: the NVIDIA compute-capability check does not apply
	return true;
	#else
	ASSERT(major > 0 && minor >= 0 && minor < 10);
	const int cc(major*10+minor);
	for (int arch: cudaArchsSASS)
		if (arch > 0 && arch/10 == major && arch%10 <= minor)
			return true;
	for (int arch: cudaArchsSASSExact)
		if (arch == cc)
			return true;
	return cudaArchPTX > 0 && cudaArchPTX <= cc;
	#endif
}

#ifndef _USE_HIP
static String _formatComputeCapabilities()
{
	String archs;
	for (int arch: cudaArchsSASS)
		if (arch > 0)
			archs += String::FormatString("sm_%d ", arch);
	for (int arch: cudaArchsSASSExact)
		if (arch > 0)
			archs += String::FormatString("sm_%da ", arch);
	if (cudaArchPTX > 0)
		archs += String::FormatString("compute_%d (PTX) ", cudaArchPTX);
	ASSERT(!archs.empty());
	archs.pop_back();
	return archs;
}
#endif

// fill the Device struct if the device can run the CUDA code
static bool _initDevice(int devID, Device& device)
{
	cudaDeviceProp props;
	int computeMode;
	if (cudaGetDeviceProperties(&props, devID) != cudaSuccess ||
		cudaDeviceGetAttribute(&computeMode, cudaDevAttrComputeMode, devID) != cudaSuccess) {
		VERBOSE("CUDA device %d: failed to query its properties", devID);
		return false;
	}
	if (computeMode == cudaComputeModeProhibited) {
		VERBOSE("CUDA device %d (%s) is running in compute mode prohibited", devID, props.name);
		return false;
	}
	if (!_isComputeCapabilitySupported(props.major, props.minor)) {
		#ifndef _USE_HIP
		VERBOSE("CUDA device %d (%s) has compute capability %d.%d, but this build contains CUDA code only for %s; "
			"rebuild with -DCMAKE_CUDA_ARCHITECTURES=%d%d (or all-major) to use it",
			devID, props.name, props.major, props.minor, _formatComputeCapabilities().c_str(), props.major, props.minor);
		#endif
		return false;
	}
	device.ID = devID;
	device.major = props.major;
	device.minor = props.minor;
	return true;
}

// initialize CUDA devices from a comma-separated list of device IDs
CUresult initDevices(const String& deviceIDs)
{
	ASSERT(devices.IsEmpty());
	if (isCpuRequested(deviceIDs))
		return CUDA_ERROR_INVALID_DEVICE;

	// no driver, no device or a driver too old for the CUDA runtime: expected on
	// machines without an NVIDIA GPU, so not an error
	int deviceCount;
	if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) {
		VERBOSE("No CUDA device available");
		return CUDA_ERROR_NO_DEVICE;
	}

	if (deviceIDs == "-1") {
		// auto-select the supported device with the highest compute capability, then performance
		size_t maxPerf(0);
		Device bestDevice;
		for (int devID = 0; devID < deviceCount; ++devID) {
			Device device;
			if (!_initDevice(devID, device))
				continue;
			int multiProcessorCount, clockRate;
			if (cudaDeviceGetAttribute(&multiProcessorCount, cudaDevAttrMultiProcessorCount, devID) != cudaSuccess ||
				cudaDeviceGetAttribute(&clockRate, cudaDevAttrClockRate, devID) != cudaSuccess)
				continue;
			const size_t perf((size_t)multiProcessorCount * (size_t)clockRate);
			if (bestDevice.ID < 0 ||
				device.major > bestDevice.major ||
				(device.major == bestDevice.major && device.minor > bestDevice.minor) ||
				(device.major == bestDevice.major && device.minor == bestDevice.minor && perf > maxPerf))
			{
				bestDevice = device;
				maxPerf = perf;
			}
		}
		if (bestDevice.ID >= 0)
			devices.Insert(bestDevice);
	} else {
		// parse comma-separated device IDs
		CLISTDEF2(String) tokens;
		Util::strSplit(deviceIDs, _T(','), tokens);
		for (const String& token: tokens) {
			if (token.empty())
				continue;
			const int devID = std::atoi(token.c_str());
			if (devID < 0 || devID >= deviceCount) {
				VERBOSE("CUDA device %d does not exist (%d detected)", devID, deviceCount);
				continue;
			}
			Device device;
			if (_initDevice(devID, device))
				devices.Insert(device);
		}
	}
	if (devices.IsEmpty()) {
		VERBOSE("No usable CUDA device");
		return CUDA_ERROR_NO_DEVICE;
	}

	// set the first device as active
	if (cudaSetDevice(devices[0].ID) != cudaSuccess) {
		VERBOSE("CUDA device %d: failed to activate it", devices[0].ID);
		devices.Release();
		return CUDA_ERROR_INVALID_DEVICE;
	}

	#if TD_VERBOSE != TD_VERBOSE_OFF
	for (const Device& device: devices) {
		cudaDeviceProp props;
		cudaGetDeviceProperties(&props, device.ID);
		DEBUG("CUDA device %d initialized: %s (compute capability %d.%d; memory %s)",
			device.ID, props.name, device.major, device.minor,
			Util::formatBytes(props.totalGlobalMem).c_str());
	}
	#endif
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/


void MemDevice::Release() {
	if (pData) {
		reportCudaError(cuMemFree(pData));
		pData = 0;
	}
}
CUresult MemDevice::Reset(size_t size) {
	if (pData) {
		if (nSize == size)
			return CUDA_SUCCESS;
		Release();
	}
	if (cuMemAlloc(&pData, size) != CUDA_SUCCESS) {
		pData = 0;
		return CUDA_ERROR_OUT_OF_MEMORY;
	}
	nSize = size;
	return CUDA_SUCCESS;
}
CUresult MemDevice::Reset(const void* pDataHost, size_t size) {
	if (pData && nSize != size)
		Release();
	if (!pData && cuMemAlloc(&pData, size) != CUDA_SUCCESS) {
		pData = 0;
		return CUDA_ERROR_OUT_OF_MEMORY;
	}
	nSize = size;
	return cuMemcpyHtoD(pData, pDataHost, size);
}

CUresult MemDevice::SetData(const void* pDataHost, size_t size) {
	ASSERT(IsValid());
	return cuMemcpyHtoD(pData, pDataHost, size);
}

CUresult MemDevice::GetData(void* pDataHost, size_t size) const {
	ASSERT(IsValid());
	return cuMemcpyDtoH(pDataHost, pData, size);
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace SEACAVE

#endif // _USE_CUDA
