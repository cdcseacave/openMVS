////////////////////////////////////////////////////////////////////
// UtilCUDACompat.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef  __SEACAVE_CUDA_COMPAT_H__
#define  __SEACAVE_CUDA_COMPAT_H__

// Maps the CUDA runtime, driver and cuRAND names used by the GPU code to their
// HIP equivalents when building for AMD GPUs (_USE_HIP, compiled by hipcc).
// The same .cu sources then build unchanged for CUDA, which is the default.
// Include this header instead of the CUDA headers.

#ifdef _USE_HIP

// hipcc does not define __CUDACC__ / __CUDA_ARCH__: code testing them must also test
// __HIP__ (host and device passes) and __HIP_DEVICE_COMPILE__ (device pass), see Config.h
#include <hip/hip_runtime.h>
#include <hip/hip_runtime_api.h>
#include <hiprand/hiprand_kernel.h>

// no CUDA fast-math intrinsics in HIP: plain (more accurate) equivalents
#define __expf expf
#define __fdividef(a, b) ((a) / (b))

// runtime API
#define cudaError_t hipError_t
#define cudaSuccess hipSuccess
#define cudaStream_t hipStream_t
#define cudaEvent_t hipEvent_t
#define cudaGraph_t hipGraph_t
#define cudaGraphExec_t hipGraphExec_t
#define cudaArray_t hipArray_t
#define cudaTextureObject_t hipTextureObject_t
#define cudaSurfaceObject_t hipSurfaceObject_t
#define cudaTextureDesc hipTextureDesc
#define cudaResourceDesc hipResourceDesc
#define cudaResourceTypeArray hipResourceTypeArray
#define cudaChannelFormatDesc hipChannelFormatDesc
#define cudaChannelFormatKindFloat hipChannelFormatKindFloat
#define cudaCreateChannelDesc hipCreateChannelDesc
#define cudaFilterModeLinear hipFilterModeLinear
#define cudaAddressModeClamp hipAddressModeClamp
#define cudaAddressModeWrap hipAddressModeWrap
#define cudaReadModeElementType hipReadModeElementType
#define cudaDeviceProp hipDeviceProp_t
#define cudaComputeModeProhibited hipComputeModeProhibited
#define cudaDevAttrComputeMode hipDeviceAttributeComputeMode
#define cudaDevAttrClockRate hipDeviceAttributeClockRate
#define cudaDevAttrMultiProcessorCount hipDeviceAttributeMultiprocessorCount
#define cudaMemcpyHostToDevice hipMemcpyHostToDevice
#define cudaMemcpyDeviceToHost hipMemcpyDeviceToHost
#define cudaStreamCaptureModeThreadLocal hipStreamCaptureModeThreadLocal
#define cudaHostAllocDefault hipHostMallocDefault
#define cudaEventDisableTiming hipEventDisableTiming
#define cudaGetDeviceCount hipGetDeviceCount
#define cudaGetDeviceProperties hipGetDeviceProperties
#define cudaDeviceGetAttribute hipDeviceGetAttribute
#define cudaSetDevice hipSetDevice
#define cudaGetLastError hipGetLastError
#define cudaGetErrorName hipGetErrorName
#define cudaGetErrorString hipGetErrorString
#define cudaMalloc hipMalloc
#define cudaMallocAsync hipMallocAsync
#define cudaMallocArray hipMallocArray
#define cudaFree hipFree
#define cudaFreeAsync hipFreeAsync
#define cudaFreeArray hipFreeArray
#define cudaHostAlloc hipHostAlloc
#define cudaFreeHost hipHostFree
#define cudaMemGetInfo hipMemGetInfo
#define cudaMemcpyAsync hipMemcpyAsync
#define cudaMemcpy2DToArrayAsync hipMemcpy2DToArrayAsync
#define cudaMemcpyToSymbolAsync hipMemcpyToSymbolAsync
#define cudaStreamCreate hipStreamCreate
#define cudaStreamDestroy hipStreamDestroy
#define cudaStreamSynchronize hipStreamSynchronize
#define cudaStreamWaitEvent hipStreamWaitEvent
#define cudaStreamBeginCapture hipStreamBeginCapture
#define cudaStreamEndCapture hipStreamEndCapture
// CUDA's 3-argument form; HIP's takes an error-log buffer as well, which is unused here
#define cudaGraphInstantiate(exec, graph, flags) hipGraphInstantiate(exec, graph, nullptr, nullptr, 0)
#define cudaGraphLaunch hipGraphLaunch
#define cudaGraphExecDestroy hipGraphExecDestroy
#define cudaGraphDestroy hipGraphDestroy
#define cudaEventCreateWithFlags hipEventCreateWithFlags
#define cudaEventRecord hipEventRecord
#define cudaCreateTextureObject hipCreateTextureObject
#define cudaDestroyTextureObject hipDestroyTextureObject
#define cudaCreateSurfaceObject hipCreateSurfaceObject
#define cudaDestroySurfaceObject hipDestroySurfaceObject

// driver API
#define CUresult hipError_t
#define CUDA_SUCCESS hipSuccess
#define CUdeviceptr hipDeviceptr_t
#define CUarray hipArray_t
#define CUarray_format hipArray_Format
#define CUDA_ARRAY3D_DESCRIPTOR HIP_ARRAY3D_DESCRIPTOR
#define CUDA_MEMCPY2D hip_Memcpy2D
#define CU_AD_FORMAT_UNSIGNED_INT8 HIP_AD_FORMAT_UNSIGNED_INT8
#define CU_AD_FORMAT_UNSIGNED_INT16 HIP_AD_FORMAT_UNSIGNED_INT16
#define CU_AD_FORMAT_UNSIGNED_INT32 HIP_AD_FORMAT_UNSIGNED_INT32
#define CU_AD_FORMAT_SIGNED_INT8 HIP_AD_FORMAT_SIGNED_INT8
#define CU_AD_FORMAT_SIGNED_INT16 HIP_AD_FORMAT_SIGNED_INT16
#define CU_AD_FORMAT_SIGNED_INT32 HIP_AD_FORMAT_SIGNED_INT32
#define CU_AD_FORMAT_HALF HIP_AD_FORMAT_HALF
#define CU_AD_FORMAT_FLOAT HIP_AD_FORMAT_FLOAT
#define CU_MEMORYTYPE_HOST hipMemoryTypeHost
#define CU_MEMORYTYPE_ARRAY hipMemoryTypeArray
#define CUDA_ERROR_OUT_OF_MEMORY hipErrorOutOfMemory
#define CUDA_ERROR_NO_DEVICE hipErrorNoDevice
#define CUDA_ERROR_INVALID_DEVICE hipErrorInvalidDevice
#define cuMemAlloc hipMalloc
#define cuMemAllocHost hipHostMalloc
#define cuMemFree hipFree
#define cuMemFreeHost hipHostFree
#define cuMemcpyHtoD hipMemcpyHtoD
#define cuMemcpyDtoH hipMemcpyDtoH
#define cuMemcpy2D hipMemcpyParam2D
#define cuMemsetD16 hipMemsetD16
#define cuMemsetD32 hipMemsetD32
#define cuMemsetD32Async hipMemsetD32Async
#define cuArray3DCreate hipArray3DCreate
#define cuArrayDestroy hipArrayDestroy
// ROCm has no CUDA_ARRAY3D_SURFACE_LDST flag: surface load/store is always available on HIP arrays
#define CUDA_ARRAY3D_SURFACE_LDST 0u
#define cuCtxSynchronize hipCtxSynchronize
#define cuGetErrorName hipDrvGetErrorName
#define cuGetErrorString hipDrvGetErrorString

// cuRAND
#define curandState hiprandState
#define curand_init hiprand_init
#define curand_uniform hiprand_uniform

// warp-wide sync mask: CUDA takes a 32-bit mask; HIP's *_sync intrinsics take a 64-bit
// mask and keep only the lanes of the active wavefront (32 lanes on wave32 devices)
#define OPENMVS_WARP_FULL_MASK 0xffffffffull

#else // CUDA toolkit

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <curand_kernel.h>
#include <math_constants.h>
#include <vector_types.h>

#define OPENMVS_WARP_FULL_MASK 0xffffffffu

#endif // _USE_HIP

#endif // __SEACAVE_CUDA_COMPAT_H__
