////////////////////////////////////////////////////////////////////
// UtilCUDA.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef  __SEACAVE_CUDA_H__
#define  __SEACAVE_CUDA_H__


// GPU device-selection string shared by all backends (CUDA and Metal), available
// even when no GPU backend is compiled. Set via --gpu-device:
// "-1" best GPU, "-2"/"cpu"/"none"/empty CPU, >=0 comma-separated device IDs.
namespace SEACAVE {
namespace CUDA {
extern GENERAL_API String desiredDeviceIDs;
// returns true if deviceIDs is a CPU sentinel (empty / "-2" / "cpu" / "none")
GENERAL_API bool isCpuRequested(const String& deviceIDs);
} // namespace CUDA
} // namespace SEACAVE


#ifdef _USE_CUDA


// I N C L U D E S /////////////////////////////////////////////////

#include "UtilCUDACompat.h"

#include "UtilCUDADevice.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

namespace CUDA {

// global list of initialized devices
struct Device {
	int ID;
	int major, minor;

	inline Device() : ID(-1) {}
};
typedef CLISTDEF0(Device) Devices;
extern GENERAL_API Devices devices;

// logs the failed CUDA driver call; returns its result
inline CUresult __reportCudaError(CUresult result, LPCSTR call, LPCSTR file, int line) {
	if (result == CUDA_SUCCESS)
		return CUDA_SUCCESS;
	// both leave their output untouched for an unrecognized code
	LPCSTR szName("unrecognized error code");
	cuGetErrorName(result, &szName);
	LPCSTR szError("");
	cuGetErrorString(result, &szError);
	VERBOSE("CUDA error at %s:%d: %s (%s (code %d) - %s)", file, line, call, szName, static_cast<int>(result), szError);
	ASSERT("CudaError" == NULL);
	return result;
}
#define reportCudaError(val) SEACAVE::CUDA::__reportCudaError(val, #val, __FILE__, __LINE__)
#define checkCudaError(val) { const CUresult ret(reportCudaError(val)); if (ret != CUDA_SUCCESS) return ret; }
/*----------------------------------------------------------------*/

// initialize CUDA devices from a comma-separated list of device IDs;
// if deviceIDs is "-1", the best available device is selected;
// if deviceIDs is empty, "-2", "cpu", or "none", CUDA is disabled (CPU only)
GENERAL_API CUresult initDevices(const String& deviceIDs = String());
inline bool isEnabled() { return !devices.empty(); }
/*----------------------------------------------------------------*/


class GENERAL_API MemDevice
{
protected:
	CUdeviceptr pData;
	size_t nSize;

public:
	inline MemDevice() : pData(0) {}
	inline ~MemDevice() { Release(); }

	MemDevice(MemDevice&& rhs) : pData(rhs.pData) { rhs.pData = 0; }
	MemDevice& operator=(MemDevice& rhs) { pData = rhs.pData; rhs.pData = 0; return *this; }

	inline bool IsValid() const { return (pData != 0); }
	void Release();
	CUresult Reset(size_t size);
	CUresult Reset(const void* pDataHost, size_t size);
	template <typename TYPE>
	inline CUresult Reset(const TImage<TYPE>& param) {
		ASSERT(!param.empty() && param.isContinuous());
		return Reset(param.getData(), sizeof(TYPE)*param.area());
	}
	template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
	inline CUresult Reset(const cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>& param) {
		ASSERT(!param.IsEmpty());
		return Reset(param.GetData(), param.GetDataSize());
	}

	CUresult SetData(const void* pDataHost, size_t size);
	template <typename TYPE>
	inline CUresult SetData(const TImage<TYPE>& param) {
		ASSERT(!param.empty() && param.isContinuous());
		return SetData(param.getData(), sizeof(TYPE)*param.area());
	}
	template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
	inline CUresult SetData(const cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>& param) {
		ASSERT(!param.IsEmpty());
		return SetData(param.GetData(), param.GetDataSize());
	}

	CUresult GetData(void* pDataHost, size_t size) const;
	template <typename TYPE>
	inline CUresult GetData(TImage<TYPE>& param) const {
		ASSERT(!param.empty() && param.isContinuous());
		return GetData(param.getData(), sizeof(TYPE)*param.area());
	}
	template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
	inline CUresult GetData(cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>& param) const {
		ASSERT(!param.IsEmpty());
		return GetData(param.GetData(), param.GetDataSize());
	}

	inline operator CUdeviceptr() const {
		return pData;
	}
};
/*----------------------------------------------------------------*/


namespace ARRAY {
template <typename T> struct traits { static const CUarray_format format; };
template<> struct traits<uint8_t> { static const CUarray_format format = CU_AD_FORMAT_UNSIGNED_INT8; };
template<> struct traits<uint16_t> { static const CUarray_format format = CU_AD_FORMAT_UNSIGNED_INT16; };
template<> struct traits<uint32_t> { static const CUarray_format format = CU_AD_FORMAT_UNSIGNED_INT32; };
template<> struct traits<int8_t> { static const CUarray_format format = CU_AD_FORMAT_SIGNED_INT8; };
template<> struct traits<int16_t> { static const CUarray_format format = CU_AD_FORMAT_SIGNED_INT16; };
template<> struct traits<int32_t> { static const CUarray_format format = CU_AD_FORMAT_SIGNED_INT32; };
template<> struct traits<hfloat> { static const CUarray_format format = CU_AD_FORMAT_HALF; };
template<> struct traits<float> { static const CUarray_format format = CU_AD_FORMAT_FLOAT; };
} // namespace ARRAY

template <typename TYPE>
class TArrayRT
{
public:
	typedef TYPE Type;
	typedef TImage<TYPE> ImageType;

protected:
	CUarray hArray;

public:
	inline TArrayRT() : hArray(NULL) {}
	inline ~TArrayRT() { Release(); }

	TArrayRT(TArrayRT&& rhs) : hArray(rhs.hArray) { rhs.hArray = NULL; }
	TArrayRT& operator=(TArrayRT& rhs) {
		hArray = rhs.hArray;
		rhs.hArray = NULL;
		return *this;
	}

	inline bool IsValid() const {
		return (hArray != NULL);
	}
	void Release() {
		if (hArray) {
			reportCudaError(cuArrayDestroy(hArray));
			hArray = NULL;
		}
	}
	inline CUresult Reset(const cv::Size& size, unsigned flags=0) {
		return Reset((unsigned)size.width, (unsigned)size.height, 0, flags);
	}
	CUresult Reset(unsigned width, unsigned height, unsigned depth=0, unsigned flags=0) {
		Release();
		CUDA_ARRAY3D_DESCRIPTOR prop;
		prop.Width = width;
		prop.Height = height;
		prop.Depth = depth;
		prop.Format = ARRAY::traits<Type>::format;
		prop.NumChannels = cv::DataType<Type>::channels;
		prop.Flags = flags;
		CUresult ret(cuArray3DCreate(&hArray, &prop));
		if (ret != CUDA_SUCCESS)
			hArray = NULL;
		return ret;
	}

	operator CUarray() const {
		return hArray;
	}
	// copy some data from host memory to device memory
	CUresult SetData(const ImageType& image) {
		ASSERT(IsValid() && !image.empty());
		CUDA_MEMCPY2D param;
		memset(&param, 0, sizeof(CUDA_MEMCPY2D));
		param.dstMemoryType = CU_MEMORYTYPE_ARRAY;
		param.dstArray = hArray;
		param.srcMemoryType = CU_MEMORYTYPE_HOST;
		param.srcHost = image.getData();
		param.srcPitch = image.row_stride();
		param.WidthInBytes = image.row_stride();
		param.Height = image.height();
		return cuMemcpy2D(&param);
	}
};
typedef TArrayRT<float> ArrayRT32F;
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace SEACAVE

#endif // _USE_CUDA

#endif // __SEACAVE_CUDA_H__
