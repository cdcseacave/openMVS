#pragma once

#include <memory>


namespace MVS {

namespace CUDA {

// Represents an abstract CUDA memory buffer that is accessible from CUDA kernel code.
template <typename DataType>
class TDeviceBufferBase
{
public:
	TDeviceBufferBase() : numElements(0) {}
	explicit TDeviceBufferBase(size_t _numElements) : numElements(_numElements) {
		ASSERT(numElements > 0);
	}

	virtual ~TDeviceBufferBase() {}

	explicit TDeviceBufferBase(TDeviceBufferBase<DataType>&& other) = default;
	TDeviceBufferBase<DataType>& operator=(TDeviceBufferBase<DataType>&& other) = default;

	virtual void ReallocateToAtLeastSize(size_t _numElements) = 0;

	virtual bool IsValid() const = 0;

	virtual const DataType* GetDeviceData() const = 0;
	virtual DataType* GetDeviceData() = 0;

	size_t NumElements() const {
		return numElements;
	}

	size_t SizeInBytes() const {
		return numElements * sizeof(DataType);
	}

protected:
	size_t numElements{0};
};
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS
