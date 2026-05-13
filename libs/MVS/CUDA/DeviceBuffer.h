#pragma once

#include <array>
#include <memory>
#include <vector>

#include "DeviceBufferBase.h"
#include "Maths.h"


namespace MVS {

namespace CUDA {

// Storage for memory that is only accessible to CUDA kernels.
// The memory is allocated with cudaMalloc.
template <typename DataType>
class TDeviceBuffer : public TDeviceBufferBase<DataType>
{
public:
	TDeviceBuffer() : data(NULL) {}
	explicit TDeviceBuffer(size_t _numElements)
		: TDeviceBufferBase<DataType>(_numElements), data(NULL) {
		AllocateFromNumElements();
	}

	~TDeviceBuffer() {
		Free();
	}

	// move constructor avoids double free
	TDeviceBuffer(TDeviceBuffer&& other)
		: TDeviceBufferBase<DataType>(std::move(other)), data(other.data) {
		other.data = NULL;
	}

	// prevent l-value copying
	TDeviceBuffer(const TDeviceBuffer&) = delete;
	TDeviceBuffer& operator=(const TDeviceBuffer&) = delete;

	// Move assignment avoids double free
	TDeviceBuffer& operator=(TDeviceBuffer&& other) {
		if (this == &other)
			return *this;
		Free();
		TDeviceBufferBase<DataType>::operator=(std::move(other));
		data = other.data;
		other.data = NULL;
		return *this;
	}

	void CopyFrom(const TDeviceBuffer& other, cudaStream_t stream) {
		// Match size exactly?  Could also use ReallocateToAtLeastSize, but that sounds potentially
		// confusing.
		ReallocateIfNotSize(other.numElements);
		if (this->numElements > 0)
			CUDA_CHECK(cudaMemcpyAsync(data, other.data, other.SizeInBytes(), cudaMemcpyDeviceToDevice, stream));
	}

	void UploadFromHost(const DataType* ptrHost, size_t _numElements, cudaStream_t stream) {
		ASSERT(ptrHost != NULL);
		ReallocateIfNotSize(_numElements);
		CUDA_CHECK(cudaMemcpyAsync(data, ptrHost, this->SizeInBytes(), cudaMemcpyHostToDevice, stream));
	}

	template <size_t N>
	void UploadFromHost(const std::array<DataType, N>& hostArray, cudaStream_t stream) {
		ReallocateIfNotSize(hostArray.size());
		CUDA_CHECK(cudaMemcpyAsync(data, hostArray.data(), this->SizeInBytes(), cudaMemcpyHostToDevice, stream));
	}

	template <typename Alloc>
	void UploadFromHost(const std::vector<DataType, Alloc>& hostVector, cudaStream_t stream) {
		ReallocateIfNotSize(hostVector.size());
		CUDA_CHECK(cudaMemcpyAsync(data, hostVector.data(), this->SizeInBytes(), cudaMemcpyHostToDevice, stream));
	}

	void DownloadFromDevice(DataType* ptrHost, cudaStream_t stream) const {
		ASSERT(ptrHost != NULL);
		CUDA_CHECK(cudaMemcpyAsync(ptrHost, data, this->SizeInBytes(), cudaMemcpyDeviceToHost, stream));
	}

	template <size_t N>
	void DownloadFromDevice(std::array<DataType, N>& hostArray, cudaStream_t stream) const {
		ASSERT(hostArray.size() == this->numElements);
		CUDA_CHECK(cudaMemcpyAsync(hostArray.data(), data, this->SizeInBytes(), cudaMemcpyDeviceToHost, stream));
	}

	template <typename Alloc>
	void DownloadFromDevice(std::vector<DataType, Alloc>& hostVector, cudaStream_t stream) const {
		if (hostVector.size() != this->numElements)
			hostVector.resize(this->numElements);
		CUDA_CHECK(cudaMemcpyAsync(hostVector.data(), data, this->SizeInBytes(), cudaMemcpyDeviceToHost, stream));
	}

	void ReallocateToAtLeastSize(size_t _numElements) override {
		if (_numElements > this->numElements) {
			Free();
			this->numElements = _numElements;
			AllocateFromNumElements();
		}
	}

	void ReallocateIfNotSize(size_t _numElements) {
		if (_numElements != this->numElements) {
			Free();
			this->numElements = _numElements;
			AllocateFromNumElements();
		}
	}

	bool IsValid() const override {
		return data != NULL;
	}

	const DataType* GetDeviceData() const override {
		return data;
	}
	DataType* GetDeviceData() override {
		return data;
	}

private:
	void Free() {
		if (data != NULL) {
			CUDA_CHECK(cudaFree(data));
			data = NULL;
		}
		this->numElements = 0;
	}

	void AllocateFromNumElements() {
		ASSERT(data == NULL);
		if (this->numElements > 0)
			CUDA_CHECK(cudaMalloc(&data, this->SizeInBytes()));
	}

	DataType* data{NULL};
};
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS
