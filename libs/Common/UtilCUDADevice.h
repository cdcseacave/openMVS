////////////////////////////////////////////////////////////////////
// UtilCUDADevice.h
//
// Copyright 2024 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef  __SEACAVE_CUDA_DEVICE_H__
#define  __SEACAVE_CUDA_DEVICE_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "Config.h"

// CUDA driver
#include <cuda.h>

// CUDA toolkit
#include <cuda_runtime.h>

#include <memory>


// D E F I N E S ///////////////////////////////////////////////////

#if __CUDA_ARCH__ > 0
#define __CDC__CUDA__ARCH__ 1
#else
#undef __CDC__CUDA__ARCH__
#endif

#ifndef VERBOSE
#define DEFINE_VERBOSE 1
#define VERBOSE(...) fprintf(stderr, __VA_ARGS__)
#endif

// check for CUDA errors following a CUDA call
#define CUDA_CHECK(condition) SEACAVE::CUDA::checkCudaCall(condition)

// check cudaGetLastError() for success
#define CUDA_CHECK_LAST_ERROR CUDA_CHECK(cudaGetLastError());


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

namespace CUDA {

// active device's compute capability, as returned by getActiveDeviceCC();
// device == -1 means no active device or the query failed
struct DeviceComputeCapability {
	int device;
	int major;
	int minor;
};

// query the currently active CUDA device's compute capability
FORCEINLINE DeviceComputeCapability getActiveDeviceCC() {
	DeviceComputeCapability cc = { -1, 0, 0 };
	if (cudaGetDevice(&cc.device) != cudaSuccess || cc.device < 0) {
		cc.device = -1;
		return cc;
	}
	cudaDeviceGetAttribute(&cc.major, cudaDevAttrComputeCapabilityMajor, cc.device);
	cudaDeviceGetAttribute(&cc.minor, cudaDevAttrComputeCapabilityMinor, cc.device);
	return cc;
}

FORCEINLINE void checkCudaCall(const cudaError_t error) {
	if (error == cudaSuccess)
		return;
	VERBOSE("CUDA error at %s:%d: %s (code %d)", __FILE__, __LINE__, cudaGetErrorString(error), error);
	// these three errors all map to "device cannot run this kernel/symbol":
	//   500 cudaErrorNotFound             - named symbol not found on the active device
	//   209 cudaErrorNoKernelImageForDevice - no SASS/PTX path for this device's compute capability
	//    98 cudaErrorInvalidDeviceFunction - kernel was not registered for this device
	// the typical cause is a CUDA_ARCHITECTURES list that omits the device's compute
	// capability and ships no PTX-virtual fallback; point the user at the fix
	// cudaErrorNotFound (500) was removed in CUDA 12+; compare against the literal value
	if (error == static_cast<cudaError_t>(500) || error == cudaErrorNoKernelImageForDevice || error == cudaErrorInvalidDeviceFunction) {
		const DeviceComputeCapability cc = getActiveDeviceCC();
		if (cc.device >= 0)
			VERBOSE("CUDA hint: the active device is compute capability %d.%d (sm_%d%d); "
				"rebuild OpenMVS with -DCMAKE_CUDA_ARCHITECTURES=%d%d "
				"(or a list that includes this arch, optionally with PTX virtual fallback) "
				"and ensure the CUDA Toolkit supports it.",
				cc.major, cc.minor, cc.major, cc.minor, cc.major, cc.minor);
	}
	ASSERT("CudaError" == NULL);
	exit(EXIT_FAILURE);
}

// define smart pointers for CUDA stream
struct CudaStreamDestructor {
	void operator()(cudaStream_t s) {
		if (s)
			CUDA_CHECK(cudaStreamDestroy(s));
	}
};

typedef std::unique_ptr<std::remove_pointer<cudaStream_t>::type, CudaStreamDestructor> CudaStreamPtr;
inline CudaStreamPtr CreateStream() {
	cudaStream_t stream;
	CUDA_CHECK(cudaStreamCreate(&stream));
	return CudaStreamPtr(stream, CudaStreamDestructor());
}

typedef std::shared_ptr<std::remove_pointer<cudaStream_t>::type> CudaStreamSharedPtr;
inline CudaStreamSharedPtr CreateSharedStream() {
	cudaStream_t stream;
	CUDA_CHECK(cudaStreamCreate(&stream));
	return CudaStreamSharedPtr(stream, CudaStreamDestructor());
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace SEACAVE

#ifdef DEFINE_VERBOSE
#undef DEFINE_VERBOSE
#undef VERBOSE
#endif

#endif // __SEACAVE_CUDA_DEVICE_H__
