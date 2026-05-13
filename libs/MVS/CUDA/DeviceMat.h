#pragma once

#include <memory>
#include <cuda_runtime.h>
#include <opencv2/core.hpp>
#include "DeviceBufferBase.h"
#include "DeviceBuffer.h"
#include "DeviceImage.h"

namespace MVS {

namespace CUDA {

// Base class for a mat accessible from the GPU
template <typename T>
class TDeviceMatBase
{
public:
	virtual ~TDeviceMatBase() {}

	// returns true if this type of CUDA device mat has an associated stream
	virtual bool HasAssociatedStream() const = 0;

	Point2i CudaImageSize() const {
		return Point2i(cols, rows);
	}
	int CudaStride() const {
		return cudaStride;
	}
	int CudaChannels() const {
		return cudaChannels;
	}

	TDeviceImage<const T> DeviceImage() const {
		return TDeviceImage<const T>(GetDeviceData(), CudaImageSize(), CudaStride(), CudaChannels());
	}
	TDeviceImage<T> MutableDeviceImage() {
		return TDeviceImage<T>(GetDeviceData(), CudaImageSize(), CudaStride(), CudaChannels());
	}

	size_t SizeInBytes() const {
		return DeviceBuffer().SizeInBytes();
	}

	// accessors to the raw pointer
	virtual const T* GetDeviceData() const {
		return DeviceBuffer().GetDeviceData();
	}
	virtual T* GetDeviceData() {
		return DeviceBuffer().GetDeviceData();
	}

	// resize mat; this might reallocate memory if required, but no new memory will be allocated,
	// if is resized to a smaller size:
	//  - stride: in units of T, not bytes (like OpenCV), for getting a pointer to the beginning of a row
	//            T* ptrRowY = ptrRow0 + y * stride
	virtual void Resize(const cv::Size& size, int numChannels, int stride = 0) {
		if (stride <= 0)
			stride = size.width * numChannels;
		DeviceBuffer().ReallocateToAtLeastSize(size.height * stride);
		rows = size.height;
		cols = size.width;
		cudaStride = stride;
		cudaChannels = numChannels;
	}

	virtual bool Upload(const cv::Mat& mat, cudaStream_t stream) {
		if (mat.empty())
			return false;
		ASSERT(mat.step[1] == sizeof(T) * cudaChannels);
		CUDA_CHECK(cudaMemcpy2DAsync(GetDeviceData(), cudaStride * sizeof(T), mat.data, mat.step, cols * mat.elemSize(), rows, cudaMemcpyHostToDevice, stream));
		return true;
	}
	virtual bool Download(cv::Mat& mat, cudaStream_t stream) const {
		if (mat.empty()) {
			constexpr int depth = cv::DataDepth<T>::value;
			mat.create(cv::Size(cols,rows), CV_MAKETYPE(depth, cudaChannels));
		}
		ASSERT(mat.step[1] == sizeof(T) * cudaChannels);
		CUDA_CHECK(cudaMemcpy2DAsync(mat.data, mat.step, GetDeviceData(), cudaStride * sizeof(T), cols * cudaChannels * sizeof(T), rows, cudaMemcpyDeviceToHost, stream));
		return true;
	}

protected:
	TDeviceMatBase() = default;
	TDeviceMatBase(const cv::Size& _size, int _cudaStride, int _cudaChannels)
		: rows(_size.height), cols(_size.width), cudaStride(_cudaStride), cudaChannels(_cudaChannels) {}

	TDeviceMatBase(TDeviceMatBase<T>&& other) = default;
	TDeviceMatBase<T>& operator=(TDeviceMatBase<T>&& other) = default;

	virtual TDeviceBufferBase<T>& DeviceBuffer() = 0;
	virtual const TDeviceBufferBase<T>& DeviceBuffer() const = 0;

	int rows{0};
	int cols{0};
	int cudaStride{0};
	int cudaChannels{0};
};
/*----------------------------------------------------------------*/


// Allocate CUDA device-only memory and as a GPU image for OpenCV
template <typename T>
class TDeviceMat : public TDeviceMatBase<T>
{
public:
	TDeviceMat() {}

	// stride is in units of T, not bytes
	explicit TDeviceMat(const cv::Size& size, int numChannels = 1, int stride = 0) {
		this->Resize(size, numChannels, stride);
	}

	// no copy constructor or assignment
	TDeviceMat(const TDeviceMat<T>& other) = delete;
	void operator=(const TDeviceMat<T>& other) = delete;

	TDeviceMat(TDeviceMat<T>&& other) = default;
	TDeviceMat<T>& operator=(TDeviceMat<T>&& other) = default;

	bool HasAssociatedStream() const override {
		return false;
	}

protected:
	TDeviceBufferBase<T>& DeviceBuffer() override {
		return deviceBuffer;
	}

	const TDeviceBufferBase<T>& DeviceBuffer() const override {
		return deviceBuffer;
	}

private:
	TDeviceBuffer<T> deviceBuffer;
};
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS
