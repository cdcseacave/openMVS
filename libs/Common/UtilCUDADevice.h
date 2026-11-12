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

#include "UtilCUDACompat.h"


// D E F I N E S ///////////////////////////////////////////////////

// check for CUDA errors following a CUDA call
#define CUDA_CHECK(condition) SEACAVE::CUDA::checkCudaCall(condition, #condition, __FILE__, __LINE__)

// check cudaGetLastError() for success
#define CUDA_CHECK_LAST_ERROR CUDA_CHECK(cudaGetLastError());


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

namespace CUDA {

// log the failed CUDA call (in the log file, also from .cu translation units) and abort
[[noreturn]] GENERAL_API void abortCudaCall(cudaError_t error, const char* call, const char* file, int line);

FORCEINLINE void checkCudaCall(cudaError_t error, const char* call, const char* file, int line) {
	if (error != cudaSuccess)
		abortCudaCall(error, call, file, line);
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace SEACAVE

#endif // __SEACAVE_CUDA_DEVICE_H__
