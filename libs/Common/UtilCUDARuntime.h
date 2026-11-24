////////////////////////////////////////////////////////////////////
// UtilCUDARuntime.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef  __SEACAVE_CUDA_RUNTIME_H__
#define  __SEACAVE_CUDA_RUNTIME_H__


// The GPU runtime the CUDA code is written against; include it instead of the CUDA headers.
// A CUDA build includes the CUDA toolkit. A HIP build (_USE_HIP: AMD GPUs through ROCm, the same
// .cu sources compiled by hipcc) includes HIP and declares the CUDA names the code uses as typed
// aliases of their HIP counterparts: types as `using`, enumerators as constants, functions as
// inline forwarders. No macros: nothing rewrites the tokens of the translation units including
// this header, and every forwarder is type-checked against the HIP signature it calls.


// I N C L U D E S /////////////////////////////////////////////////

#include "Config.h"

#ifndef _USE_HIP

// CUDA driver
#include <cuda.h>

// CUDA toolkit
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <curand_kernel.h>
#include <math_constants.h>
#include <vector_types.h>

#else // _USE_HIP

#include <hip/hip_runtime.h>
#include <hiprand/hiprand_kernel.h>


// S T R U C T S ///////////////////////////////////////////////////

// runtime API types
using cudaError_t = hipError_t;
using cudaStream_t = hipStream_t;
using cudaEvent_t = hipEvent_t;
using cudaGraph_t = hipGraph_t;
using cudaGraphExec_t = hipGraphExec_t;
using cudaArray_t = hipArray_t;
using cudaTextureObject_t = hipTextureObject_t;
using cudaSurfaceObject_t = hipSurfaceObject_t;
using cudaTextureDesc = hipTextureDesc;
using cudaResourceDesc = hipResourceDesc;
using cudaChannelFormatDesc = hipChannelFormatDesc;
using cudaDeviceProp = hipDeviceProp_t;

// runtime API constants
constexpr hipError_t cudaSuccess = hipSuccess;
constexpr hipResourceType cudaResourceTypeArray = hipResourceTypeArray;
constexpr hipChannelFormatKind cudaChannelFormatKindFloat = hipChannelFormatKindFloat;
constexpr hipTextureFilterMode cudaFilterModeLinear = hipFilterModeLinear;
constexpr hipTextureAddressMode cudaAddressModeClamp = hipAddressModeClamp;
constexpr hipTextureAddressMode cudaAddressModeWrap = hipAddressModeWrap;
constexpr hipTextureReadMode cudaReadModeElementType = hipReadModeElementType;
constexpr hipComputeMode cudaComputeModeProhibited = hipComputeModeProhibited;
constexpr hipDeviceAttribute_t cudaDevAttrComputeMode = hipDeviceAttributeComputeMode;
constexpr hipDeviceAttribute_t cudaDevAttrClockRate = hipDeviceAttributeClockRate;
constexpr hipDeviceAttribute_t cudaDevAttrMultiProcessorCount = hipDeviceAttributeMultiprocessorCount;
constexpr hipDeviceAttribute_t cudaDevAttrWarpSize = hipDeviceAttributeWarpSize;
constexpr hipMemcpyKind cudaMemcpyHostToDevice = hipMemcpyHostToDevice;
constexpr hipMemcpyKind cudaMemcpyDeviceToHost = hipMemcpyDeviceToHost;
constexpr hipStreamCaptureMode cudaStreamCaptureModeThreadLocal = hipStreamCaptureModeThreadLocal;
constexpr unsigned cudaHostAllocDefault = hipHostMallocDefault;
constexpr unsigned cudaEventDisableTiming = hipEventDisableTiming;

// runtime API functions
inline hipError_t cudaGetDeviceCount(int* count) { return hipGetDeviceCount(count); }
inline hipError_t cudaGetDeviceProperties(hipDeviceProp_t* prop, int device) { return hipGetDeviceProperties(prop, device); }
inline hipError_t cudaDeviceGetAttribute(int* value, hipDeviceAttribute_t attr, int device) { return hipDeviceGetAttribute(value, attr, device); }
inline hipError_t cudaSetDevice(int device) { return hipSetDevice(device); }
inline hipError_t cudaGetLastError() { return hipGetLastError(); }
inline const char* cudaGetErrorName(hipError_t error) { return hipGetErrorName(error); }
inline const char* cudaGetErrorString(hipError_t error) { return hipGetErrorString(error); }
inline hipError_t cudaMemGetInfo(size_t* free, size_t* total) { return hipMemGetInfo(free, total); }
template <typename T>
inline hipError_t cudaMalloc(T** ptr, size_t size) { return hipMalloc((void**)ptr, size); }
template <typename T>
inline hipError_t cudaMallocAsync(T** ptr, size_t size, hipStream_t stream) { return hipMallocAsync((void**)ptr, size, stream); }
inline hipError_t cudaMallocArray(hipArray_t* array, const hipChannelFormatDesc* desc, size_t width, size_t height, unsigned flags = 0) { return hipMallocArray(array, desc, width, height, flags); }
template <typename T>
inline hipError_t cudaHostAlloc(T** ptr, size_t size, unsigned flags) { return hipHostMalloc((void**)ptr, size, flags); }
inline hipError_t cudaFree(void* ptr) { return hipFree(ptr); }
inline hipError_t cudaFreeAsync(void* ptr, hipStream_t stream) { return hipFreeAsync(ptr, stream); }
inline hipError_t cudaFreeArray(hipArray_t array) { return hipFreeArray(array); }
inline hipError_t cudaFreeHost(void* ptr) { return hipHostFree(ptr); }
inline hipError_t cudaMemcpyAsync(void* dst, const void* src, size_t size, hipMemcpyKind kind, hipStream_t stream = 0) { return hipMemcpyAsync(dst, src, size, kind, stream); }
inline hipError_t cudaMemcpy2DToArrayAsync(hipArray_t dst, size_t wOffset, size_t hOffset, const void* src, size_t spitch, size_t width, size_t height, hipMemcpyKind kind, hipStream_t stream = 0) { return hipMemcpy2DToArrayAsync(dst, wOffset, hOffset, src, spitch, width, height, kind, stream); }
template <typename T>
inline hipError_t cudaMemcpyToSymbolAsync(const T& symbol, const void* src, size_t size, size_t offset, hipMemcpyKind kind, hipStream_t stream = 0) { return hipMemcpyToSymbolAsync(HIP_SYMBOL(symbol), src, size, offset, kind, stream); }
inline hipError_t cudaStreamCreate(hipStream_t* stream) { return hipStreamCreate(stream); }
inline hipError_t cudaStreamDestroy(hipStream_t stream) { return hipStreamDestroy(stream); }
inline hipError_t cudaStreamSynchronize(hipStream_t stream) { return hipStreamSynchronize(stream); }
inline hipError_t cudaStreamWaitEvent(hipStream_t stream, hipEvent_t event, unsigned flags = 0) { return hipStreamWaitEvent(stream, event, flags); }
inline hipError_t cudaStreamBeginCapture(hipStream_t stream, hipStreamCaptureMode mode) { return hipStreamBeginCapture(stream, mode); }
inline hipError_t cudaStreamEndCapture(hipStream_t stream, hipGraph_t* graph) { return hipStreamEndCapture(stream, graph); }
// CUDA's (exec, graph, flags) form is HIP's hipGraphInstantiateWithFlags; hipGraphInstantiate takes an error-log buffer instead of the flags
inline hipError_t cudaGraphInstantiate(hipGraphExec_t* exec, hipGraph_t graph, unsigned long long flags = 0) { return hipGraphInstantiateWithFlags(exec, graph, flags); }
inline hipError_t cudaGraphLaunch(hipGraphExec_t exec, hipStream_t stream) { return hipGraphLaunch(exec, stream); }
inline hipError_t cudaGraphExecDestroy(hipGraphExec_t exec) { return hipGraphExecDestroy(exec); }
inline hipError_t cudaGraphDestroy(hipGraph_t graph) { return hipGraphDestroy(graph); }
inline hipError_t cudaEventCreateWithFlags(hipEvent_t* event, unsigned flags) { return hipEventCreateWithFlags(event, flags); }
inline hipError_t cudaEventRecord(hipEvent_t event, hipStream_t stream = 0) { return hipEventRecord(event, stream); }
inline hipChannelFormatDesc cudaCreateChannelDesc(int x, int y, int z, int w, hipChannelFormatKind kind) { return hipCreateChannelDesc(x, y, z, w, kind); }
inline hipError_t cudaCreateTextureObject(hipTextureObject_t* tex, const hipResourceDesc* resDesc, const hipTextureDesc* texDesc, const hipResourceViewDesc* viewDesc) { return hipCreateTextureObject(tex, resDesc, texDesc, viewDesc); }
inline hipError_t cudaDestroyTextureObject(hipTextureObject_t tex) { return hipDestroyTextureObject(tex); }
inline hipError_t cudaCreateSurfaceObject(hipSurfaceObject_t* surf, const hipResourceDesc* resDesc) { return hipCreateSurfaceObject(surf, resDesc); }
inline hipError_t cudaDestroySurfaceObject(hipSurfaceObject_t surf) { return hipDestroySurfaceObject(surf); }

// driver API types; a device address is an integer as in CUDA (the code offsets it in bytes),
// where HIP's hipDeviceptr_t is a void*: the forwarders below convert it
using CUresult = hipError_t;
using CUdeviceptr = unsigned long long;
static_assert(sizeof(CUdeviceptr) == sizeof(hipDeviceptr_t), "a device address must fit CUdeviceptr");
using CUarray = hipArray_t;
using CUarray_format = hipArray_Format;
using CUDA_ARRAY3D_DESCRIPTOR = HIP_ARRAY3D_DESCRIPTOR;
using CUDA_MEMCPY2D = hip_Memcpy2D;

// driver API constants
#ifndef CUDA_SUCCESS // the HIP runtime header defines it itself when hipcc compiles
constexpr hipError_t CUDA_SUCCESS = hipSuccess;
#endif
constexpr hipError_t CUDA_ERROR_OUT_OF_MEMORY = hipErrorOutOfMemory;
constexpr hipError_t CUDA_ERROR_NO_DEVICE = hipErrorNoDevice;
constexpr hipError_t CUDA_ERROR_INVALID_DEVICE = hipErrorInvalidDevice;
constexpr hipArray_Format CU_AD_FORMAT_UNSIGNED_INT8 = HIP_AD_FORMAT_UNSIGNED_INT8;
constexpr hipArray_Format CU_AD_FORMAT_UNSIGNED_INT16 = HIP_AD_FORMAT_UNSIGNED_INT16;
constexpr hipArray_Format CU_AD_FORMAT_UNSIGNED_INT32 = HIP_AD_FORMAT_UNSIGNED_INT32;
constexpr hipArray_Format CU_AD_FORMAT_SIGNED_INT8 = HIP_AD_FORMAT_SIGNED_INT8;
constexpr hipArray_Format CU_AD_FORMAT_SIGNED_INT16 = HIP_AD_FORMAT_SIGNED_INT16;
constexpr hipArray_Format CU_AD_FORMAT_SIGNED_INT32 = HIP_AD_FORMAT_SIGNED_INT32;
constexpr hipArray_Format CU_AD_FORMAT_HALF = HIP_AD_FORMAT_HALF;
constexpr hipArray_Format CU_AD_FORMAT_FLOAT = HIP_AD_FORMAT_FLOAT;
constexpr hipMemoryType CU_MEMORYTYPE_HOST = hipMemoryTypeHost;
constexpr hipMemoryType CU_MEMORYTYPE_ARRAY = hipMemoryTypeArray;
constexpr unsigned CUDA_ARRAY3D_SURFACE_LDST = hipArraySurfaceLoadStore;

// driver API functions
inline hipError_t cuGetErrorName(hipError_t error, const char** name) { return hipDrvGetErrorName(error, name); }
inline hipError_t cuGetErrorString(hipError_t error, const char** str) { return hipDrvGetErrorString(error, str); }
inline hipError_t cuCtxSynchronize() { return hipDeviceSynchronize(); }
inline hipError_t cuMemAlloc(CUdeviceptr* ptr, size_t size) { void* p; const hipError_t ret(hipMalloc(&p, size)); *ptr = (CUdeviceptr)p; return ret; }
inline hipError_t cuMemFree(CUdeviceptr ptr) { return hipFree((void*)ptr); }
inline hipError_t cuMemAllocHost(void** ptr, size_t size) { return hipHostMalloc(ptr, size, hipHostMallocDefault); }
inline hipError_t cuMemFreeHost(void* ptr) { return hipHostFree(ptr); }
inline hipError_t cuMemcpyHtoD(CUdeviceptr dst, const void* src, size_t size) { return hipMemcpyHtoD((hipDeviceptr_t)dst, src, size); }
inline hipError_t cuMemcpyDtoH(void* dst, CUdeviceptr src, size_t size) { return hipMemcpyDtoH(dst, (hipDeviceptr_t)src, size); }
inline hipError_t cuMemcpy2D(const hip_Memcpy2D* copy) { return hipMemcpyParam2D(copy); }
inline hipError_t cuMemsetD16(CUdeviceptr dst, unsigned short value, size_t count) { return hipMemsetD16((hipDeviceptr_t)dst, value, count); }
inline hipError_t cuMemsetD32(CUdeviceptr dst, unsigned value, size_t count) { return hipMemsetD32((hipDeviceptr_t)dst, (int)value, count); }
inline hipError_t cuMemsetD32Async(CUdeviceptr dst, unsigned value, size_t count, hipStream_t stream) { return hipMemsetD32Async((hipDeviceptr_t)dst, (int)value, count, stream); }
inline hipError_t cuArray3DCreate(hipArray_t* array, const HIP_ARRAY3D_DESCRIPTOR* desc) { return hipArray3DCreate(array, desc); }
inline hipError_t cuArrayDestroy(hipArray_t array) { return hipArrayDestroy(array); }

// cuRAND
using curandState = hiprandState;
#ifdef __HIP__
__device__ inline void curand_init(unsigned long long seed, unsigned long long subsequence, unsigned long long offset, hiprandState* state) { hiprand_init(seed, subsequence, offset, state); }
__device__ inline float curand_uniform(hiprandState* state) { return hiprand_uniform(state); }
#endif

#endif // _USE_HIP

#endif // __SEACAVE_CUDA_RUNTIME_H__
