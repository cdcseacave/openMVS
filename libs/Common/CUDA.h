////////////////////////////////////////////////////////////////////
// CUDA.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef  __SEACAVE_CUDA_H__
#define  __SEACAVE_CUDA_H__

#ifdef _USE_CUDA


// I N C L U D E S /////////////////////////////////////////////////

// CUDA driver & runtime
#include <cuda.h>
#include <cuda_runtime.h>


// D E F I N E S ///////////////////////////////////////////////////

#ifdef _MSC_VER
//#define TIMER_OLDSUPPORT
#endif
#define FIX_FPS


namespace SEACAVE {

namespace CUDA {

// S T R U C T S ///////////////////////////////////////////////////

#ifdef __DRIVER_TYPES_H__
#ifndef DEVICE_RESET
#define DEVICE_RESET cudaDeviceReset();
#endif
inline void __getLastCudaError(const char *errorMessage)
{
	cudaError_t err(cudaGetLastError());
	if (err == cudaSuccess)
		return;
	LOG("CUDA error at %s:%d: %s (code %d)", __FILE__, __LINE__, errorMessage, static_cast<unsigned>(err));
	ASSERT("CudaLastError" == NULL);
	DEVICE_RESET;
	exit(EXIT_FAILURE);
}
// This will output the proper error string when calling cudaGetLastError
#define getLastCudaError(msg) CUDA::__getLastCudaError(msg)
#else
#ifndef DEVICE_RESET
#define DEVICE_RESET
#endif
#define getLastCudaError(msg)
#endif // __DRIVER_TYPES_H__

template <typename T>
inline void __checkCudaError(T result, char const *const msg)
{
	if (result == CUDA_SUCCESS)
		return;
	LOG("CUDA error at %s:%d: %s (code %d)", __FILE__, __LINE__, msg, static_cast<unsigned>(result));
	ASSERT("CudaError" == NULL);
	DEVICE_RESET;
	exit(EXIT_FAILURE);
}
// This will output the proper CUDA error code in the event that a CUDA host call returns an error
#define checkCudaErrors(val) CUDA::__checkCudaError(val, #val)

// GPU Architecture definitions
inline int convertSMVer2Cores(int major, int minor);

#ifdef __CUDA_RUNTIME_H__
// checks that the given device ID is valid;
// return the device ID if successful
int gpuCheckDeviceId(int devID);

// finds the best GPU (with maximum GFLOPS);
// return the device ID if successful
int gpuGetMaxGflopsDeviceId();

// global list of initialized devices
struct Device {
	unsigned ID;
	cudaDeviceProp prop;
};
typedef CLISTDEF0(Device) Devices;
extern Devices devices;

// initialize the given CUDA device and add it to the array of initialized devices;
// if the given device is -1, the best available device is selected
CUresult initDevice(int deviceID=-1);

// load/read kernel from 'program' file/string, compile and return the requested function
CUresult ptxJIT(const char* program, const char* functionName, CUmodule *phModule, CUfunction *phKernel, CUlinkState *lState, bool bFromFile=false);

// add a new parameter to the given kernel
template <typename T>
inline CUresult addKernelParam(CUfunction& hKernel, int& paramOffset, T param)
{
	CUresult myErr = cuParamSetv(hKernel, paramOffset, (void*)&param, sizeof(T));
	if (myErr != CUDA_SUCCESS)
		return myErr;
	paramOffset += sizeof(T);
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/



#ifdef _SUPPORT_CPP11
class KernelRT
{
public:
	CUmodule hModule;
	CUfunction hKernel;
	CUlinkState lState;
	CLISTDEF0(void*) inDatas; // array of pointers to the allocated memory read by the program
	CLISTDEF0(void*) outDatas; // array of pointers to the allocated memory written by the program
	int paramOffset; // used during parameter insertion to remember current parameter position

public:
	inline KernelRT() : hModule(NULL) {}
	inline KernelRT(LPCSTR program, LPCSTR functionName, bool bFromFile=false) { Reset(program, functionName, bFromFile); }
	inline ~KernelRT() { Release(); }

	inline bool IsValid() const {
		return (hModule != NULL);
	}
	void Release();
	CUresult Reset(LPCSTR program, LPCSTR functionName, bool bFromFile=false);

	struct InputParam {
		const void* data; // pointer to host data to be allocated and copied to the CUDA device
		size_t size; // size in bytes of the data
		inline InputParam() {}
		inline InputParam(const void* _data, size_t _size) : data(_data), size(_size) {}
	};
	struct OutputParam {
		size_t size; // size in bytes of the data to be allocated on the CUDA device
		inline OutputParam() {}
		inline OutputParam(size_t _size) : size(_size) {}
	};
	// lunch the program with the given parameters;
	//  numThreads - total number of threads to run
	//  args - variadic parameters to be passed to the kernel
	template <typename... Args>
	CUresult operator()(int numThreads, Args&&... args) {
		Reset();
		CUresult result;
		// set the kernel parameters (Driver API)
		if ((result=AddParam(std::forward<Args>(args)...)) != CUDA_SUCCESS)
			return result;
		if ((result=cuParamSetSize(hKernel, paramOffset)) != CUDA_SUCCESS)
			return result;
		// launch the kernel (Driver API)
		const cudaDeviceProp& deviceProp = CUDA::devices.Last().prop;
		const int numBlockThreads(MINF(numThreads, deviceProp.maxThreadsPerBlock));
		const int nBlocks(MAXF((numThreads+numBlockThreads-1)/numBlockThreads, 1));
		if ((result=cuFuncSetBlockShape(hKernel, numBlockThreads, 1, 1)) != CUDA_SUCCESS)
			return result;
		return cuLaunchGrid(hKernel, nBlocks, 1);
	}

	struct ReturnParam {
		void* data; // pointer to host data to be written with the output data from the CUDA device
		size_t size; // size in bytes of the data
		inline ReturnParam() {}
		inline ReturnParam(void* _data, size_t _size) : data(_data), size(_size) {}
	};
	// read from the device the variadic output parameters
	CUresult GetResult(const std::initializer_list<ReturnParam>& params);

private:
	void Reset();

	template <typename T>
	inline CUresult _AddParam(T param) {
		return addKernelParam(hKernel, paramOffset, param);
	}
	CUresult _AddParam(const InputParam& param);
	CUresult _AddParam(const OutputParam& param);
	template <typename T>
	inline CUresult AddParam(T&& param) {
		return _AddParam(std::forward<T>(param));
	}
	template <typename T, typename... Args>
	inline CUresult AddParam(T&& param, Args&&... args) {
		CUresult result(AddParam(std::forward<T>(param)));
		if (result != CUDA_SUCCESS)
			return result;
		if ((result=AddParam(std::forward<Args>(args)...)) != CUDA_SUCCESS)
			return result;
		return CUDA_SUCCESS;
	}
};
/*----------------------------------------------------------------*/
#endif // _SUPPORT_CPP11

#endif // __CUDA_RUNTIME_H__

} // namespace CUDA

} // namespace SEACAVE

#endif // _USE_CUDA

#endif // __SEACAVE_CUDA_H__
