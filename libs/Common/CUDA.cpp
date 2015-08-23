////////////////////////////////////////////////////////////////////
// CUDA.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "CUDA.h"

#ifdef _USE_CUDA


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

namespace CUDA {

// S T R U C T S ///////////////////////////////////////////////////

Devices devices;

// GPU Architecture definitions
int convertSMVer2Cores(int major, int minor)
{
	// Defines for GPU Architecture types (using the SM version to determine the # of cores per SM
	struct sSMtoCores {
		int SM; // 0xMm (hexidecimal notation), M = SM Major version, and m = SM minor version
		int Cores;
	};
	const sSMtoCores nGpuArchCoresPerSM[] = {
		{0x20, 32}, // Fermi Generation (SM 2.0) GF100 class
		{0x21, 48}, // Fermi Generation (SM 2.1) GF10x class
		{0x30, 192}, // Kepler Generation (SM 3.0) GK10x class
		{0x32, 192}, // Kepler Generation (SM 3.2) GK10x class
		{0x35, 192}, // Kepler Generation (SM 3.5) GK11x class
		{0x37, 192}, // Kepler Generation (SM 3.7) GK21x class
		{0x50, 128}, // Maxwell Generation (SM 5.0) GM10x class
		{0x52, 128}, // Maxwell Generation (SM 5.2) GM20x class
		{-1, -1}
	};

	int index(0);
	while (nGpuArchCoresPerSM[index].SM != -1) {
		if (nGpuArchCoresPerSM[index].SM == ((major << 4) + minor))
			return nGpuArchCoresPerSM[index].Cores;
		index++;
	}

	// If we don't find the values, we default use the previous one to run properly
	VERBOSE("MapSMtoCores for SM %d.%d is undefined; default to use %d cores/SM", major, minor, nGpuArchCoresPerSM[index-1].Cores);
	return nGpuArchCoresPerSM[index-1].Cores;
}

#ifdef __CUDA_RUNTIME_H__
// checks that the given device ID is valid;
// return the device ID if successful
int gpuCheckDeviceId(int devID)
{
	int device_count;
	checkCudaErrors(cudaGetDeviceCount(&device_count));
	if (device_count == 0) {
		VERBOSE("CUDA error: no devices supporting CUDA");
		return -1;
	}
	if (devID < 0)
		devID = 0;
	if (devID > device_count-1) {
		VERBOSE("CUDA error: device [%d] is not a valid GPU device (%d CUDA capable GPU device(s) detected)", devID, device_count);
		return -1;
	}
	cudaDeviceProp deviceProp;
	checkCudaErrors(cudaGetDeviceProperties(&deviceProp, devID));
	if (deviceProp.computeMode == cudaComputeModeProhibited) {
		VERBOSE("CUDA error: device is running in <Compute Mode Prohibited>, no threads can use ::cudaSetDevice()");
		return -1;
	}
	if (deviceProp.major < 1) {
		VERBOSE("CUDA error: GPU device does not support CUDA");
		return -1;
	}
	checkCudaErrors(cudaSetDevice(devID));
	return devID;
}

// finds the best GPU (with maximum GFLOPS);
// return the device ID if successful
int gpuGetMaxGflopsDeviceId()
{
	int current_device     = 0, sm_per_multiproc  = 0;
	int max_perf_device    = 0;
	int device_count       = 0, best_SM_arch      = 0;
	int devices_prohibited = 0;

	unsigned long long max_compute_perf = 0;
	cudaDeviceProp deviceProp;
	cudaGetDeviceCount(&device_count);

	checkCudaErrors(cudaGetDeviceCount(&device_count));
	if (device_count == 0) {
		VERBOSE("CUDA error: no devices supporting CUDA");
		return -1;
	}

	// Find the best major SM Architecture GPU device
	while (current_device < device_count) {
		cudaGetDeviceProperties(&deviceProp, current_device);

		// If this GPU is not running on Compute Mode prohibited, then we can add it to the list
		if (deviceProp.computeMode != cudaComputeModeProhibited) {
			if (deviceProp.major > 0 && deviceProp.major < 9999) {
				best_SM_arch = MAX(best_SM_arch, deviceProp.major);
			}
		} else {
			devices_prohibited++;
		}

		current_device++;
	}

	if (devices_prohibited == device_count) {
		VERBOSE("CUDA error: all devices have compute mode prohibited");
		return -1;
	}

	// Find the best CUDA capable GPU device
	current_device = 0;

	while (current_device < device_count) {
		cudaGetDeviceProperties(&deviceProp, current_device);

		// If this GPU is not running on Compute Mode prohibited, then we can add it to the list
		if (deviceProp.computeMode != cudaComputeModeProhibited) {
			if (deviceProp.major == 9999 && deviceProp.minor == 9999) {
				sm_per_multiproc = 1;
			} else {
				sm_per_multiproc = convertSMVer2Cores(deviceProp.major, deviceProp.minor);
			}

			unsigned long long compute_perf  = (unsigned long long) deviceProp.multiProcessorCount * sm_per_multiproc * deviceProp.clockRate;

			if (compute_perf  > max_compute_perf) {
				// If we find GPU with SM major > 2, search only these
				if (best_SM_arch > 2) {
					// If our device==dest_SM_arch, choose this, or else pass
					if (deviceProp.major == best_SM_arch) {
						max_compute_perf  = compute_perf;
						max_perf_device   = current_device;
					}
				} else {
					max_compute_perf  = compute_perf;
					max_perf_device   = current_device;
				}
			}
		}

		++current_device;
	}

	return max_perf_device;
}

// initialize the given CUDA device and add it to the array of initialized devices;
// if the given device is -1, the best available device is selected
CUresult initDevice(int deviceID)
{
	Device device;

	if (deviceID >= 0) {
		device.ID = gpuCheckDeviceId(deviceID);
	} else {
		// Otherwise pick the device with the highest Gflops/s
		device.ID = gpuGetMaxGflopsDeviceId();
	}
	if (device.ID < 0) {
		VERBOSE("error: no CUDA capable devices found");
		return CUDA_ERROR_NO_DEVICE;
	}
	checkCudaErrors(cudaSetDevice(device.ID));

	checkCudaErrors(cudaGetDeviceProperties(&device.prop, device.ID));
	if (device.prop.major < 2) {
		VERBOSE("CUDA error: compute capability 2.0 or greater required (available %d.%d for device[%d])", device.ID, device.prop.major, device.prop.minor);
		return CUDA_ERROR_INVALID_DEVICE;
	}
	devices.Insert(device);
	DEBUG("CUDA device[%d] initialized: %s (Compute Capability %d.%d)", device.ID, device.prop.name, device.prop.major, device.prop.minor);

	#if 1
	// dummy memory allocation to work around a bug inside CUDA
	// (this seems to initialize some more things)
	void* cpDummy;
	cudaMalloc(&cpDummy, sizeof(int));
	cudaFree(cpDummy);
	#endif
	return CUDA_SUCCESS;
}

// load/read kernel from 'program' file/string, compile and return the requested function
CUresult ptxJIT(const char* program, const char* functionName, CUmodule *phModule, CUfunction *phKernel, CUlinkState *lState, bool bFromFile)
{
	CUjit_option options[6];
	void *optionVals[6];
	float walltime(0);
	const unsigned logSize(8192);
	char error_log[logSize], info_log[logSize];
	void *cuOut;
	size_t outSize;

	// Setup linker options
	// Return walltime from JIT compilation
	options[0] = CU_JIT_WALL_TIME;
	optionVals[0] = (void*)&walltime;
	// Pass a buffer for info messages
	options[1] = CU_JIT_INFO_LOG_BUFFER;
	optionVals[1] = (void*)info_log;
	// Pass the size of the info buffer
	options[2] = CU_JIT_INFO_LOG_BUFFER_SIZE_BYTES;
	optionVals[2] = (void*)(long)logSize;
	// Pass a buffer for error message
	options[3] = CU_JIT_ERROR_LOG_BUFFER;
	optionVals[3] = (void*)error_log;
	// Pass the size of the error buffer
	options[4] = CU_JIT_ERROR_LOG_BUFFER_SIZE_BYTES;
	optionVals[4] = (void*)(long)logSize;
	// Make the linker verbose
	options[5] = CU_JIT_LOG_VERBOSE;
	optionVals[5] = (void*)1;

	// Create a pending linker invocation
	checkCudaErrors(cuLinkCreate(6, options, optionVals, lState));

	DEBUG("Loading '%s' program", functionName);
	CUresult myErr;
	if (bFromFile) {
		// Load the PTX from the file (64-bit)
		myErr = cuLinkAddFile(*lState, CU_JIT_INPUT_PTX, program, 0, 0, 0);
	} else {
		// Load the PTX from the string myPtx (64-bit)
		myErr = cuLinkAddData(*lState, CU_JIT_INPUT_PTX, (void*)program, strlen(program)+1, 0, 0, 0, 0);
	}
	if (myErr != CUDA_SUCCESS) {
		// Errors will be put in error_log, per CU_JIT_ERROR_LOG_BUFFER option above.
		VERBOSE("PTX Linker Error: %s", error_log);
		return myErr;
	}

	// Complete the linker step
	checkCudaErrors(cuLinkComplete(*lState, &cuOut, &outSize));

	// Linker walltime and info_log were requested in options above.
	DEBUG("CUDA link completed (%gms):\n%s", walltime, info_log);

	// Load resulting cuBin into module
	checkCudaErrors(cuModuleLoadData(phModule, cuOut));

	// Locate the kernel entry point
	checkCudaErrors(cuModuleGetFunction(phKernel, *phModule, functionName));

	// Destroy the linker invocation
	checkCudaErrors(cuLinkDestroy(*lState));
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/



#ifdef _SUPPORT_CPP11
void KernelRT::Release() {
	Reset();
	if (hModule) {
		cuModuleUnload(hModule);
		hModule = NULL;
	}
}
void KernelRT::Reset() {
	paramOffset = 0;
	FOREACHPTR(ptrData, inDatas)
		cudaFree(*ptrData);
	inDatas.Empty();
	FOREACHPTR(ptrData, outDatas)
		cudaFree(*ptrData);
	outDatas.Empty();
}
CUresult KernelRT::Reset(LPCSTR program, LPCSTR functionName, bool bFromFile) {
	// JIT Compile the Kernel from PTX and get the Handles (Driver API)
	CUresult result(ptxJIT(program, functionName, &hModule, &hKernel, &lState, bFromFile));
	if (result != CUDA_SUCCESS)
		hModule = NULL;
	return result;
}


// append a generic input parameter (allocate&copy input buffer)
CUresult KernelRT::_AddParam(const InputParam& param) {
	void*& data = inDatas.AddEmpty();
	if (cudaMalloc(&data, param.size) != cudaSuccess)
		return CUDA_ERROR_OUT_OF_MEMORY;
	if (cudaMemcpy(data, param.data, param.size, cudaMemcpyHostToDevice) != cudaSuccess)
		return CUDA_ERROR_INVALID_VALUE;
	return addKernelParam(hKernel, paramOffset, data);
}
// append a generic output parameter (allocate output buffer)
CUresult KernelRT::_AddParam(const OutputParam& param) {
	void*& data = outDatas.AddEmpty();
	if (cudaMalloc(&data, param.size) != cudaSuccess)
		return CUDA_ERROR_OUT_OF_MEMORY;
	return addKernelParam(hKernel, paramOffset, data);
}


// copy result from the given output parameter index back to the host
CUresult KernelRT::GetResult(int idx, const ReturnParam& param) {
	return (cudaMemcpy(param.data, outDatas[idx], param.size, cudaMemcpyDeviceToHost) == cudaSuccess) ?
		CUDA_SUCCESS : CUDA_ERROR_INVALID_VALUE;
}
// read from the device the variadic output parameters
CUresult KernelRT::GetResult(const std::initializer_list<ReturnParam>& params) {
	IDX idx(0);
	for (auto param : params)
		if (cudaMemcpy(param.data, outDatas[idx++], param.size, cudaMemcpyDeviceToHost) != cudaSuccess)
			return CUDA_ERROR_INVALID_VALUE;
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/
#endif // _SUPPORT_CPP11

#endif // __CUDA_RUNTIME_H__

} // namespace CUDA

} // namespace SEACAVE

#endif // _USE_CUDA
