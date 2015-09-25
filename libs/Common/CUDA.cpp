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
	if (reportCudaError(cudaGetDeviceCount(&device_count)) != CUDA_SUCCESS)
		return -1;
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
	if (reportCudaError(cudaGetDeviceProperties(&deviceProp, devID)) != CUDA_SUCCESS)
		return -1;
	if (deviceProp.computeMode == cudaComputeModeProhibited) {
		VERBOSE("CUDA error: device is running in <Compute Mode Prohibited>, no threads can use ::cudaSetDevice()");
		return -1;
	}
	if (deviceProp.major < 1) {
		VERBOSE("CUDA error: GPU device does not support CUDA");
		return -1;
	}
	if (reportCudaError(cudaSetDevice(devID)) != CUDA_SUCCESS)
		return -1;
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

	if (reportCudaError(cudaGetDeviceCount(&device_count)) != CUDA_SUCCESS)
		return -1;
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
	if (device.ID == NO_ID) {
		VERBOSE("error: no CUDA capable devices found");
		return CUDA_ERROR_NO_DEVICE;
	}
	checkCudaError(cudaSetDevice(device.ID));

	checkCudaError(cudaGetDeviceProperties(&device.prop, device.ID));
	if (device.prop.major < 2) {
		VERBOSE("CUDA error: compute capability 2.0 or greater required (available %d.%d for device[%d])", device.ID, device.prop.major, device.prop.minor);
		return CUDA_ERROR_INVALID_DEVICE;
	}
	devices.Insert(device);

	#if 1
	// dummy memory allocation to work around a bug inside CUDA
	// (this seems to initialize some more things)
	{
	void* cpDummy;
	while (cudaMalloc(&cpDummy, sizeof(float)*256) != cudaSuccess);
	cudaFree(cpDummy);
	}
	#endif

	#if TD_VERBOSE != TD_VERBOSE_OFF
	size_t memFree, memSize;
	checkCudaError(cudaMemGetInfo(&memFree, &memSize));
	DEBUG("CUDA device[%d] initialized: %s (compute capability %d.%d; memory %s)", device.ID, device.prop.name, device.prop.major, device.prop.minor, Util::formatBytes(memSize).c_str());
	#endif
	return CUDA_SUCCESS;
}

// load/read module (program) from file/string and compile it
CUresult ptxJIT(LPCSTR program, CUmodule& hModule, int mode)
{
	CUlinkState lState;
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
	checkCudaError(cuLinkCreate(6, options, optionVals, &lState));

	const size_t programLen(strlen(program));
	CUresult myErr;
	if (mode == JIT::FILE || (mode == JIT::AUTO && programLen < 256)) {
		// Load the PTX from the file
		myErr = cuLinkAddFile(lState, CU_JIT_INPUT_PTX, program, 0, 0, 0);
	} else {
		// Load the PTX from the string
		myErr = cuLinkAddData(lState, CU_JIT_INPUT_PTX, (void*)program, programLen+1, 0, 0, 0, 0);
	}
	if (myErr != CUDA_SUCCESS) {
		// Errors will be put in error_log, per CU_JIT_ERROR_LOG_BUFFER option above
		VERBOSE("PTX Linker Error: %s", error_log);
		return myErr;
	}

	// Complete the linker step
	checkCudaError(cuLinkComplete(lState, &cuOut, &outSize));

	// Linker walltime and info_log were requested in options above
	DEBUG_LEVEL(3, "CUDA link completed (%gms):\n%s", walltime, info_log);

	// Load resulting cuBin into module
	checkCudaError(cuModuleLoadData(&hModule, cuOut));

	// Destroy the linker invocation
	return reportCudaError(cuLinkDestroy(lState));
}

// requested function (kernel) from module (program)
CUresult ptxGetFunc(const CUmodule& hModule, LPCSTR functionName, CUfunction& hKernel)
{
	// Locate the kernel entry point
	checkCudaError(cuModuleGetFunction(&hKernel, hModule, functionName));
	DEBUG_ULTIMATE("Kernel '%s' loaded", functionName);
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/


void MemDevice::Release() {
	if (pData) {
		reportCudaError(cudaFree(pData));
		pData = NULL;
	}
}
CUresult MemDevice::Reset(size_t size) {
	if (pData) {
		if (nSize == size)
			return CUDA_SUCCESS;
		Release();
	}
	if (cudaMalloc(&pData, size) != cudaSuccess) {
		pData = NULL;
		return CUDA_ERROR_OUT_OF_MEMORY;
	}
	nSize = size;
	return CUDA_SUCCESS;
}
CUresult MemDevice::Reset(const void* pDataHost, size_t size) {
	if (pData && nSize != size)
		Release();
	if (!pData && cudaMalloc(&pData, size) != cudaSuccess) {
		pData = NULL;
		return CUDA_ERROR_OUT_OF_MEMORY;
	}
	nSize = size;
	if (cudaMemcpy(pData, pDataHost, size, cudaMemcpyHostToDevice) != cudaSuccess)
		return CUDA_ERROR_INVALID_VALUE;
	return CUDA_SUCCESS;
}

CUresult MemDevice::SetData(const void* pDataHost, size_t size) {
	ASSERT(IsValid());
	if (cudaMemcpy(pData, pDataHost, size, cudaMemcpyHostToDevice) != cudaSuccess)
		return CUDA_ERROR_INVALID_VALUE;
	return CUDA_SUCCESS;
}

CUresult MemDevice::GetData(void* pDataHost, size_t size) const {
	ASSERT(IsValid());
	if (cudaMemcpy(pDataHost, pData, size, cudaMemcpyDeviceToHost) != cudaSuccess)
		return CUDA_ERROR_INVALID_VALUE;
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/


void EventRT::Release() {
	if (hEvent) {
		reportCudaError(cuEventDestroy(hEvent));
		hEvent = NULL;
	}
}
CUresult EventRT::Reset(unsigned flags) {
	CUresult ret(cuEventCreate(&hEvent, flags));
	if (ret != CUDA_SUCCESS)
		hEvent = NULL;
	return ret;
}
/*----------------------------------------------------------------*/


void StreamRT::Release() {
	if (hStream) {
		reportCudaError(cuStreamDestroy(hStream));
		hStream = NULL;
	}
}
CUresult StreamRT::Reset(unsigned flags) {
	CUresult ret(cuStreamCreate(&hStream, flags));
	if (ret != CUDA_SUCCESS)
		hStream = NULL;
	return ret;
}

CUresult StreamRT::Wait(CUevent hEvent) {
	ASSERT(IsValid());
	return cuStreamWaitEvent(hStream, hEvent, 0);
}
/*----------------------------------------------------------------*/


void ModuleRT::Release() {
	if (hModule) {
		reportCudaError(cuModuleUnload(hModule));
		hModule = NULL;
	}
}
CUresult ModuleRT::Reset(LPCSTR program, int mode) {
	// compile the module (program) from PTX and get its handle (Driver API)
	CUresult result(ptxJIT(program, hModule, mode));
	if (result != CUDA_SUCCESS)
		hModule = NULL;
	return result;
}
/*----------------------------------------------------------------*/


void KernelRT::Release() {
	inDatas.Release();
	outDatas.Release();
	ptrModule.Release();
	hKernel = NULL;
}
void KernelRT::Reset() {
	paramOffset = 0;
	inDatas.Empty();
	outDatas.Empty();
}
CUresult KernelRT::Reset(LPCSTR functionName) {
	// get the function handle (Driver API)
	ASSERT(ptrModule != NULL && ptrModule->IsValid());
	CUresult result(ptxGetFunc(*ptrModule, functionName, hKernel));
	if (result != CUDA_SUCCESS) {
		ptrModule.Release();
		hKernel = NULL;
	}
	return result;
}
CUresult KernelRT::Reset(const ModuleRTPtr& _ptrModule, LPCSTR functionName) {
	// set module
	ptrModule = _ptrModule;
	// set function
	return Reset(functionName);
}
CUresult KernelRT::Reset(LPCSTR program, LPCSTR functionName, int mode) {
	// compile the module (program) from PTX and get the function handle (Driver API)
	ptrModule = new ModuleRT(program, mode);
	if (!ptrModule->IsValid()) {
		ptrModule.Release();
		hKernel = NULL;
		return CUDA_ERROR_INVALID_HANDLE;
	}
	return Reset(functionName);
}


// append a generic input parameter (allocate&copy input buffer)
CUresult KernelRT::_AddParam(const InputParam& param) {
	MemDevice& data = inDatas.AddEmpty();
	if (data.Reset(param.data, param.size) != CUDA_SUCCESS)
		return CUDA_ERROR_OUT_OF_MEMORY;
	return addKernelParam(hKernel, paramOffset, (void*)data);
}
// append a generic output parameter (allocate output buffer)
CUresult KernelRT::_AddParam(const OutputParam& param) {
	MemDevice& data = outDatas.AddEmpty();
	if (data.Reset(param.size) != CUDA_SUCCESS)
		return CUDA_ERROR_OUT_OF_MEMORY;
	return addKernelParam(hKernel, paramOffset, (void*)data);
}


// copy result from the given output parameter index back to the host
CUresult KernelRT::GetResult(void* data, const ReturnParam& param) const {
	return (cudaMemcpy(param.data, data, param.size, cudaMemcpyDeviceToHost) == cudaSuccess) ?
		CUDA_SUCCESS : CUDA_ERROR_INVALID_VALUE;
}
// read from the device the variadic output parameters
CUresult KernelRT::GetResult(const std::initializer_list<ReturnParam>& params) const {
	MemDeviceArr::IDX idx(0);
	for (auto param : params)
		if (outDatas[idx++].GetData(param.data, param.size) != CUDA_SUCCESS)
			return CUDA_ERROR_INVALID_VALUE;
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/

#endif // __CUDA_RUNTIME_H__

} // namespace CUDA

} // namespace SEACAVE

#endif // _USE_CUDA
