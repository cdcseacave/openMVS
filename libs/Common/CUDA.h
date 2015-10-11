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

// CUDA driver
#include <cuda.h>


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

namespace CUDA {

// S T R U C T S ///////////////////////////////////////////////////

// global list of initialized devices
struct Device {
	CUdevice ID;
	int major, minor;
	int computeMode;
	CUdevprop prop;
	CUcontext ctx;

	inline Device() : ctx(NULL) {}
	inline ~Device() { if (ctx != NULL) cuCtxDestroy(ctx); }
};
typedef CLISTDEF0(Device) Devices;
extern Devices devices;

// outputs the proper CUDA error code in the event that a CUDA host call returns an error
inline CUresult __reportCudaError(CUresult result, LPCSTR errorMessage) {
	if (result == CUDA_SUCCESS)
		return CUDA_SUCCESS;
	LPCSTR szName;
	cuGetErrorName(result, &szName);
	LPCSTR szError;
	cuGetErrorString(result, &szError);
	#ifdef _DEBUG
	VERBOSE("CUDA error at %s:%d: %s (%s (code %d) - %s)", __FILE__, __LINE__, errorMessage, szName, static_cast<unsigned>(result), szError);
	#else
	DEBUG("CUDA error: %s (%s (code %d) - %s)", errorMessage, szName, static_cast<unsigned>(result), szError);
	#endif
	ASSERT("CudaError" == NULL);
	return result;
}
#define reportCudaError(val) CUDA::__reportCudaError(val, #val)

#define checkCudaError(val) { const CUresult ret(CUDA::__reportCudaError(val, #val)); if (ret != CUDA_SUCCESS) return ret; }

// outputs the proper CUDA error code and abort in the event that a CUDA host call returns an error
inline void __ensureCudaResult(CUresult result, LPCSTR errorMessage) {
	if (__reportCudaError(result, errorMessage))
		return;
	ASSERT("CudaAbort" == NULL);
	exit(EXIT_FAILURE);
}
#define ensureCudaResult(val) CUDA::__ensureCudaResult(val, #val)

// rounds up addr to the align boundary
template <typename T>
inline T align(T o, T a) {
	a -= T(1);
	return (o + a)&~a;
}

// initialize the given CUDA device and add it to the array of initialized devices;
// if the given device is -1, the best available device is selected
CUresult initDevice(int deviceID=-1);

// load/read module (program) from file/string and compile it
enum JIT { AUTO=0, STRING=1, FILE=2 };
CUresult ptxJIT(LPCSTR program, CUmodule& hModule, int mode=JIT::AUTO);

// requested function (kernel) from module (program)
CUresult ptxGetFunc(const CUmodule& hModule, LPCSTR functionName, CUfunction& hKernel);

// add a new parameter to the given kernel
template <typename T>
inline CUresult addKernelParam(CUfunction& hKernel, int& paramOffset, const T& param) {
	paramOffset = align(paramOffset, (int)alignof(T));
	const CUresult result(cuParamSetv(hKernel, paramOffset, (void*)&param, sizeof(T)));
	paramOffset += sizeof(T);
	return result;
}

// allocate on the CUDA device a chunk of memory of the given size
inline CUresult allocMemDevice(size_t size, CUdeviceptr& dataDevice) {
	return cuMemAlloc(&dataDevice, size);
}
// copy on the CUDA device the given chunk of memory
inline CUresult copyMemDevice(const void* data, size_t size, CUdeviceptr dataDevice) {
	return cuMemcpyHtoD(dataDevice, data, size);
}
// allocate and copy on the CUDA device the given chunk of memory
inline CUresult createReplicaDevice(const void* data, size_t size, CUdeviceptr& dataDevice) {
	if (cuMemAlloc(&dataDevice, size) != CUDA_SUCCESS)
		return CUDA_ERROR_OUT_OF_MEMORY;
	return cuMemcpyHtoD(dataDevice, data, size);
}
// copy from the CUDA device the given chunk of memory
inline CUresult fetchMemDevice(void* data, size_t size, const CUdeviceptr dataDevice) {
	return cuMemcpyDtoH(data, dataDevice, size);
}
// free the given memory on the CUDA device
inline CUresult freeMemDevice(CUdeviceptr& dataDevice) {
	if (cuMemFree(dataDevice) != CUDA_SUCCESS)
		return CUDA_ERROR_NOT_INITIALIZED;
	dataDevice = 0;
	return CUDA_SUCCESS;
}
/*----------------------------------------------------------------*/


class MemDevice
{
protected:
	CUdeviceptr pData;
	size_t nSize;

public:
	inline MemDevice() : pData(0) {}
	inline MemDevice(size_t size) : pData(0) { reportCudaError(Reset(size)); }
	inline MemDevice(const void* pDataHost, size_t size) : pData(0) { reportCudaError(Reset(pDataHost, size)); }
	template <typename TYPE>
	inline MemDevice(const TImage<TYPE>& param) : pData(0) { reportCudaError(Reset(param)); }
	template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
	inline MemDevice(const cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>& param) : pData(0) { reportCudaError(Reset(param)); }
	inline ~MemDevice() { Release(); }

	MemDevice(MemDevice& rhs) : pData(rhs.pData) { rhs.pData = 0; }
	MemDevice& operator=(MemDevice& rhs) { pData = rhs.pData; rhs.pData = 0; return *this; }

	inline bool IsValid() const {
		return (pData != 0);
	}
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
typedef CSharedPtr<MemDevice> MemDevicePtr;
typedef CLISTDEFIDX(MemDevice,int) MemDeviceArr;
/*----------------------------------------------------------------*/


class EventRT
{
protected:
	CUevent hEvent;

protected:
	EventRT(const EventRT&);
	EventRT& operator=(const EventRT&);

public:
	inline EventRT(unsigned flags = CU_EVENT_DEFAULT) { reportCudaError(Reset(flags)); }
	inline ~EventRT() { Release(); }

	inline bool IsValid() const {
		return (hEvent != NULL);
	}
	void Release();
	CUresult Reset(unsigned flags = CU_EVENT_DEFAULT);
		
	inline operator CUevent() const {
		return hEvent;
	}
};
typedef CSharedPtr<EventRT> EventRTPtr;
/*----------------------------------------------------------------*/


class StreamRT
{
protected:
	CUstream hStream;

protected:
	StreamRT(const StreamRT&);
	StreamRT& operator=(const StreamRT&);

public:
	inline StreamRT(unsigned flags = CU_STREAM_DEFAULT) { reportCudaError(Reset(flags)); }
	inline ~StreamRT() { Release(); }

	inline bool IsValid() const {
		return (hStream != NULL);
	}
	void Release();
	CUresult Reset(unsigned flags = CU_STREAM_DEFAULT);

	inline operator CUstream() const {
		return hStream;
	}

	CUresult Wait(CUevent hEvent);
};
typedef CSharedPtr<StreamRT> StreamRTPtr;
/*----------------------------------------------------------------*/


class ModuleRT
{
protected:
	CUmodule hModule;

protected:
	ModuleRT(const ModuleRT&);
	ModuleRT& operator=(const ModuleRT&);

public:
	inline ModuleRT() : hModule(NULL) {}
	inline ModuleRT(LPCSTR program, int mode=JIT::AUTO) { Reset(program, mode); }
	inline ~ModuleRT() { Release(); }

	inline bool IsValid() const {
		return (hModule != NULL);
	}
	void Release();
	CUresult Reset(LPCSTR program, int mode=JIT::AUTO);

	inline operator CUmodule() const {
		return hModule;
	}
};
typedef CSharedPtr<ModuleRT> ModuleRTPtr;
/*----------------------------------------------------------------*/


class KernelRT
{
public:
	ModuleRTPtr ptrModule;
	StreamRTPtr ptrStream;
	CUfunction hKernel;
	MemDeviceArr inDatas; // array of pointers to the allocated memory read by the program
	MemDeviceArr outDatas; // array of pointers to the allocated memory written by the program
	int paramOffset; // used during parameter insertion to remember current parameter position

protected:
	KernelRT(const KernelRT&);
	KernelRT& operator=(const KernelRT&);

public:
	inline KernelRT() : hKernel(NULL) {}
	inline KernelRT(const ModuleRTPtr& _ptrModule, LPCSTR functionName) : ptrModule(_ptrModule) { Reset(functionName); }
	inline KernelRT(LPCSTR program, LPCSTR functionName, int mode=JIT::AUTO) { Reset(program, functionName, mode); }
	inline ~KernelRT() { Release(); }

	inline bool IsValid() const {
		ASSERT(hKernel == NULL || (ptrModule != NULL && ptrModule->IsValid()));
		return (hKernel != NULL);
	}
	void Release();
	void Reset();
	CUresult Reset(LPCSTR functionName);
	CUresult Reset(const ModuleRTPtr& _ptrModule, LPCSTR functionName);
	CUresult Reset(LPCSTR program, LPCSTR functionName, int mode=JIT::AUTO);

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
	#ifdef _SUPPORT_CPP11
	template <typename... Args>
	CUresult operator()(int numThreads, Args&&... args) {
		ASSERT(IsValid());
		Reset();
		CUresult result;
		// set the kernel parameters (Driver API)
		if ((result=AddParam(std::forward<Args>(args)...)) != CUDA_SUCCESS)
			return result;
		if ((result=cuParamSetSize(hKernel, paramOffset)) != CUDA_SUCCESS)
			return result;
		// launch the kernel (Driver API)
		const CUdevprop& deviceProp = CUDA::devices.Last().prop;
		const int numBlockThreads(MINF(numThreads, deviceProp.maxThreadsPerBlock));
		const int nBlocks(MAXF((numThreads+numBlockThreads-1)/numBlockThreads, 1));
		if ((result=cuFuncSetBlockShape(hKernel, numBlockThreads, 1, 1)) != CUDA_SUCCESS)
			return result;
		if (ptrStream != NULL)
			return cuLaunchGridAsync(hKernel, nBlocks, 1, *ptrStream);
		return cuLaunchGrid(hKernel, nBlocks, 1);
	}
	// same for 2D data
	template <typename... Args>
	CUresult operator()(const TPoint2<int>& numThreads, Args&&... args) {
		ASSERT(IsValid());
		Reset();
		CUresult result;
		// set the kernel parameters (Driver API)
		if ((result=AddParam(std::forward<Args>(args)...)) != CUDA_SUCCESS)
			return result;
		if ((result=cuParamSetSize(hKernel, paramOffset)) != CUDA_SUCCESS)
			return result;
		// launch the kernel (Driver API)
		const CUdevprop& deviceProp = CUDA::devices.Last().prop;
		const REAL scale(MINF(REAL(1), SQRT((REAL)deviceProp.maxThreadsPerBlock/(REAL)(numThreads.x*numThreads.y))));
		const SEACAVE::TPoint2<int> numBlockThreads(FLOOR2INT(SEACAVE::TPoint2<REAL>(numThreads)*scale));
		const TPoint2<int> nBlocks(
			MAXF((numThreads.x+numBlockThreads.x-1)/numBlockThreads.x, 1),
			MAXF((numThreads.y+numBlockThreads.y-1)/numBlockThreads.y, 1));
		if ((result=cuFuncSetBlockShape(hKernel, numBlockThreads.x, numBlockThreads.y, 1)) != CUDA_SUCCESS)
			return result;
		if (ptrStream != NULL)
			return cuLaunchGridAsync(hKernel, nBlocks.x, nBlocks.y, *ptrStream);
		return cuLaunchGrid(hKernel, nBlocks.x, nBlocks.y);
	}
	#endif // _SUPPORT_CPP11

	struct ReturnParam {
		void* data; // pointer to host data to be written with the output data from the CUDA device
		size_t size; // size in bytes of the data
		inline ReturnParam() {}
		inline ReturnParam(void* _data, size_t _size) : data(_data), size(_size) {}
	};
	CUresult GetResult(const CUdeviceptr data, const ReturnParam& param) const;
	inline CUresult GetResult(const MemDevice& memDev, const ReturnParam& param) const {
		return memDev.GetData(param.data, param.size);
	}
	inline CUresult GetResult(int idx, const ReturnParam& param) const {
		return GetResult(outDatas[idx], param);
	}
	template <typename TYPE>
	inline CUresult GetResult(int idx, const TImage<TYPE>& param) const {
		ASSERT(!param.empty() && param.isContinuous());
		return GetResult(idx, ReturnParam(param.getData(), sizeof(TYPE)*param.area()));
	}
	template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
	inline CUresult GetResult(int idx, const cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>& param) const {
		ASSERT(!param.IsEmpty());
		return GetResult(idx, ReturnParam(param.GetData(), param.GetDataSize()));
	}
	CUresult GetResult(const std::initializer_list<ReturnParam>& params) const;

protected:
	CUresult _AddParam(const InputParam& param);
	CUresult _AddParam(const OutputParam& param);
	template <typename T>
	inline CUresult _AddParam(const T& param) {
		return addKernelParam(hKernel, paramOffset, param);
	}
	inline CUresult _AddParam(const MemDevice& param) {
		ASSERT(param.IsValid());
		return addKernelParam(hKernel, paramOffset, (CUdeviceptr)param);
	}
	template <typename TYPE>
	inline CUresult _AddParam(const TImage<TYPE>& param) {
		ASSERT(!param.empty() && param.isContinuous());
		return _AddParam(InputParam(param.getData(), sizeof(TYPE)*param.area()));
	}
	template <typename TYPE, typename ARG_TYPE, int useConstruct, int grow, typename IDX_TYPE>
	inline CUresult _AddParam(const cList<TYPE,ARG_TYPE,useConstruct,grow,IDX_TYPE>& param) {
		ASSERT(!param.IsEmpty());
		return _AddParam(InputParam(param.GetData(), param.GetDataSize()));
	}
	#ifdef _SUPPORT_CPP11
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
	#endif // _SUPPORT_CPP11
};
typedef CSharedPtr<KernelRT> KernelRTPtr;
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
	inline TArrayRT(const Image8U::Size& size, unsigned flags=0) : hArray(NULL) { reportCudaError(Reset(size, flags)); }
	inline TArrayRT(unsigned width, unsigned height, unsigned depth=0, unsigned flags=0) : hArray(NULL) { reportCudaError(Reset(width, height, depth, flags)); }
	inline ~TArrayRT() { Release(); }

	TArrayRT(TArrayRT& rhs) : hArray(rhs.hArray) { rhs.hArray = NULL; }
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
	inline CUresult Reset(const Image8U::Size& size, unsigned flags=0) {
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
	operator CUarray&() {
		return hArray;
	}
	CUDA_ARRAY3D_DESCRIPTOR GetDescriptor() const {
		CUDA_ARRAY3D_DESCRIPTOR prop;
		cuArray3DGetDescriptor(&prop, hArray);
		return prop;
	}
	unsigned Width() const {
		return (unsigned)GetDescriptor().Width;
	}
	unsigned Height() const {
		return (unsigned)GetDescriptor().Height;
	}
	unsigned Depth() const {
		return (unsigned)GetDescriptor().Depth;
	}
	unsigned NumChannels() const {
		return GetDescriptor().NumChannels;
	}
	CUarray_format Format() const {
		return GetDescriptor().Format;
	}
	unsigned Flags() const {
		return GetDescriptor().Flags;
	}
	size_t Size() const {
		return sizeof(Type)*Width()*Height()*(Depth()>0?Depth():1)*NumChannels();
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

	// copy data from device memory to host memory
	CUresult GetData(ImageType& image) const {
		ASSERT(IsValid() && !image.empty());
		CUDA_MEMCPY2D param;
		memset(&param, 0, sizeof(CUDA_MEMCPY2D));
		param.dstMemoryType = CU_MEMORYTYPE_HOST;
		param.dstHost = image.getData();
		param.dstPitch = image.row_stride();
		param.srcMemoryType = CU_MEMORYTYPE_ARRAY;
		param.srcArray = hArray;
		param.WidthInBytes = image.row_stride();
		param.Height = image.height();
		return cuMemcpy2D(&param);
	}
};
typedef TArrayRT<uint8_t> ArrayRT8U;
typedef TArrayRT<uint32_t> ArrayRT32U;
typedef TArrayRT<hfloat> ArrayRT16F;
typedef TArrayRT<float> ArrayRT32F;
/*----------------------------------------------------------------*/


template <class TYPE>
class TTextureRT
{
public:
	typedef TArrayRT<TYPE> ArrayType;
	typedef typename ArrayType::Type Type;
	typedef typename ArrayType::ImageType ImageType;

public:
	ModuleRTPtr ptrModule;
	CUtexref hTexref;

public:
	inline TTextureRT() : hTexref(NULL) {}
	inline TTextureRT(const ModuleRTPtr& _ptrModule, LPCSTR texrefName, CUfilter_mode filtermode=CU_TR_FILTER_MODE_POINT, CUaddress_mode addrmode=CU_TR_ADDRESS_MODE_CLAMP, bool bNormalizedCoords=false) : ptrModule(_ptrModule) { Reset(texrefName, filtermode, addrmode, bNormalizedCoords); }
	inline ~TTextureRT() { Release(); }

	inline bool IsValid() const {
		ASSERT(hTexref == NULL || (ptrModule != NULL && ptrModule->IsValid()));
		return (hTexref != NULL);
	}
	void Release() {
		ptrModule.Release();
		hTexref = NULL;
	}
	CUresult Reset(LPCSTR texrefName, CUfilter_mode filtermode=CU_TR_FILTER_MODE_POINT, CUaddress_mode addrmode=CU_TR_ADDRESS_MODE_CLAMP, bool bNormalizedCoords=false) {
		// get the texture-reference handle (Driver API)
		ASSERT(ptrModule != NULL && ptrModule->IsValid());
		CUresult result(cuModuleGetTexRef(&hTexref, *ptrModule, texrefName));
		if (result != CUDA_SUCCESS)
			Release();
		// set texture parameters
		checkCudaError(cuTexRefSetFilterMode(hTexref, filtermode));
		if (bNormalizedCoords) {
			checkCudaError(cuTexRefSetFlags(hTexref, CU_TRSF_NORMALIZED_COORDINATES));
			for (int i=0; i<2; ++i)
				checkCudaError(cuTexRefSetAddressMode(hTexref, i, addrmode));
		} else {
			for (int i=0; i<2; ++i)
				checkCudaError(cuTexRefSetAddressMode(hTexref, i, CU_TR_ADDRESS_MODE_CLAMP));
		}
		cuTexRefSetFormat(hTexref, ARRAY::traits<Type>::format, cv::DataType<Type>::channels);
		return result;
	}
	inline CUresult Reset(const ModuleRTPtr& _ptrModule, LPCSTR texrefName, CUfilter_mode filtermode=CU_TR_FILTER_MODE_POINT, CUaddress_mode addrmode=CU_TR_ADDRESS_MODE_CLAMP, bool bNormalizedCoords=false) {
		// set module
		ptrModule = _ptrModule;
		// set texture
		return Reset(texrefName, filtermode, addrmode, bNormalizedCoords);
	}

	// bind the given array to the texture
	CUresult Bind(ArrayType& array) {
		return cuTexRefSetArray(hTexref, array, CU_TRSA_OVERRIDE_FORMAT);
	}

	// fetch the array bind to the texture
	CUresult Fetch(ArrayType& array) {
		return cuTexRefGetArray(hTexref, array);
	}
};
typedef TTextureRT<uint8_t> TextureRT8U;
typedef TTextureRT<uint32_t> TextureRT32U;
typedef TTextureRT<hfloat> TextureRT16F;
typedef TTextureRT<float> TextureRT32F;
/*----------------------------------------------------------------*/


template <class TYPE>
class TSurfaceRT
{
public:
	typedef TArrayRT<TYPE> ArrayType;
	typedef typename ArrayType::Type Type;
	typedef typename ArrayType::ImageType ImageType;

public:
	ModuleRTPtr ptrModule;
	CUsurfref hSurfref;

public:
	inline TSurfaceRT() : hSurfref(NULL) {}
	inline TSurfaceRT(const ModuleRTPtr& _ptrModule, LPCSTR surfrefName) : ptrModule(_ptrModule) { Reset(surfrefName); }
	inline ~TSurfaceRT() { Release(); }

	inline bool IsValid() const {
		ASSERT(hSurfref == NULL || (ptrModule != NULL && ptrModule->IsValid()));
		return (hSurfref != NULL);
	}
	void Release() {
		ptrModule.Release();
		hSurfref = NULL;
	}
	CUresult Reset(LPCSTR surfrefName) {
		// get the surface-reference handle (Driver API)
		ASSERT(ptrModule != NULL && ptrModule->IsValid());
		const CUresult result(cuModuleGetSurfRef(&hSurfref, *ptrModule, surfrefName));
		if (result != CUDA_SUCCESS)
			Release();
		return result;
	}
	inline CUresult Reset(const ModuleRTPtr& _ptrModule, LPCSTR texrefName) {
		// set module
		ptrModule = _ptrModule;
		// set texture
		return Reset(texrefName);
	}

	// bind the given array to the surface
	CUresult Bind(const ArrayType& array) {
		return cuSurfRefSetArray(hSurfref, array, 0);
	}

	// fetch the array bind to the surface
	CUresult Fetch(const ArrayType& array) const {
		return cuSurfRefGetArray(hSurfref, array);
	}
};
typedef TSurfaceRT<uint8_t> SurfaceRT8U;
typedef TSurfaceRT<uint32_t> SurfaceRT32U;
typedef TSurfaceRT<hfloat> SurfaceRT16F;
typedef TSurfaceRT<float> SurfaceRT32F;
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace SEACAVE

#endif // _USE_CUDA

#endif // __SEACAVE_CUDA_H__
