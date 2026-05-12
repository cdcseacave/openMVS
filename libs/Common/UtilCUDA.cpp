////////////////////////////////////////////////////////////////////
// UtilCUDA.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "UtilCUDA.h"

#ifdef _USE_CUDA


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

namespace CUDA {

// S T R U C T S ///////////////////////////////////////////////////

GENERAL_API String desiredDeviceIDs("-1");
GENERAL_API Devices devices;

// validate a CUDA device and fill the Device struct
static CUresult _validateDevice(int devID, Device& device)
{
	int device_count;
	if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
		VERBOSE("CUDA error: no devices supporting CUDA");
		return CUDA_ERROR_NO_DEVICE;
	}
	if (devID < 0 || devID >= device_count) {
		VERBOSE("CUDA error: device [%d] is not a valid GPU device (%d detected)", devID, device_count);
		return CUDA_ERROR_INVALID_DEVICE;
	}
	int computeMode;
	if (cudaDeviceGetAttribute(&computeMode, cudaDevAttrComputeMode, devID) != cudaSuccess) {
		VERBOSE("CUDA error: failed to get compute mode for device %d", devID);
		return CUDA_ERROR_INVALID_DEVICE;
	}
	if (computeMode == cudaComputeModeProhibited) {
		VERBOSE("CUDA error: device %d is running in Compute Mode Prohibited", devID);
		return CUDA_ERROR_INVALID_DEVICE;
	}
	int major, minor;
	if (cudaDeviceGetAttribute(&major, cudaDevAttrComputeCapabilityMajor, devID) != cudaSuccess ||
		cudaDeviceGetAttribute(&minor, cudaDevAttrComputeCapabilityMinor, devID) != cudaSuccess) {
		VERBOSE("CUDA error: failed to get compute capability for device %d", devID);
		return CUDA_ERROR_INVALID_DEVICE;
	}
	if (major < 3) {
		VERBOSE("CUDA error: device %d compute capability %d.%d < 3.0", devID, major, minor);
		return CUDA_ERROR_INVALID_DEVICE;
	}
	device.ID = devID;
	device.major = major;
	device.minor = minor;
	device.computeMode = computeMode;
	return CUDA_SUCCESS;
}

// select the best available CUDA device by compute capability and performance
static CUresult _selectBestDevice(Device& bestDevice)
{
	int device_count = 0;
	if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
		VERBOSE("CUDA error: no devices supporting CUDA");
		return CUDA_ERROR_NO_DEVICE;
	}
	size_t max_perf = 0;
	bool found = false;
	for (int i = 0; i < device_count; ++i) {
		int computeMode;
		if (cudaDeviceGetAttribute(&computeMode, cudaDevAttrComputeMode, i) != cudaSuccess)
			continue;
		int major, minor;
		if (cudaDeviceGetAttribute(&major, cudaDevAttrComputeCapabilityMajor, i) != cudaSuccess ||
			cudaDeviceGetAttribute(&minor, cudaDevAttrComputeCapabilityMinor, i) != cudaSuccess)
			continue;
		if (computeMode == cudaComputeModeProhibited || major < 3)
			continue;
		int multiProcessorCount, clockRate;
		if (cudaDeviceGetAttribute(&multiProcessorCount, cudaDevAttrMultiProcessorCount, i) != cudaSuccess ||
			cudaDeviceGetAttribute(&clockRate, cudaDevAttrClockRate, i) != cudaSuccess)
			continue;
		const size_t perf = (size_t)multiProcessorCount * (size_t)clockRate;
		if (!found ||
			major > bestDevice.major ||
			(major == bestDevice.major && minor > bestDevice.minor) ||
			(major == bestDevice.major && minor == bestDevice.minor && perf > max_perf))
		{
			bestDevice.ID = i;
			bestDevice.major = major;
			bestDevice.minor = minor;
			bestDevice.computeMode = computeMode;
			max_perf = perf;
			found = true;
		}
	}
	if (!found) {
		VERBOSE("CUDA error: no suitable CUDA device found");
		return CUDA_ERROR_NO_DEVICE;
	}
	return CUDA_SUCCESS;
}

// returns true for any of: empty, "-2", "cpu"/"CPU"/case-insensitive, "none"
bool isCpuRequested(const String& deviceIDs)
{
	if (deviceIDs.empty() || deviceIDs == "-2")
		return true;
	const String lower(deviceIDs.ToLower());
	return lower == "cpu" || lower == "none";
}

// initialize CUDA devices from a comma-separated list of device IDs
CUresult initDevices(const String& deviceIDs)
{
	if (isCpuRequested(deviceIDs))
		return CUDA_ERROR_INVALID_DEVICE;

	// cuInit needed because MemDevice uses Driver API (cuMemAlloc, etc.)
	checkCudaError(cuInit(0));

	if (deviceIDs == "-1") {
		// auto-select best device
		Device device;
		const CUresult ret = _selectBestDevice(device);
		if (ret != CUDA_SUCCESS)
			return ret;
		if (cudaSetDevice(device.ID) != cudaSuccess) {
			VERBOSE("CUDA error: failed to set device %d", device.ID);
			return CUDA_ERROR_INVALID_DEVICE;
		}
		devices.Insert(device);
	} else {
		// parse comma-separated device IDs
		CLISTDEF2(String) tokens;
		Util::strSplit(deviceIDs, _T(','), tokens);
		for (const String& token: tokens) {
			if (token.empty())
				continue;
			const int devID = std::atoi(token.c_str());
			Device device;
			if (_validateDevice(devID, device) != CUDA_SUCCESS) {
				VERBOSE("CUDA warning: skipping invalid device ID %d", devID);
				continue;
			}
			devices.Insert(device);
		}
	}
	if (devices.IsEmpty())
		return CUDA_ERROR_NO_DEVICE;

	// set the first device as active
	cudaSetDevice(devices[0].ID);

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
