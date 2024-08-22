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

inline void checkCudaCall(const cudaError_t error) {
	if (error == cudaSuccess)
		return;
	VERBOSE("CUDA error at %s:%d: %s (code %d)", __FILE__, __LINE__, cudaGetErrorString(error), error);
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
