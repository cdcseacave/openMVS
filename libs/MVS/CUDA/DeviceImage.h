#pragma once

#include "Maths.h"


namespace MVS {

namespace CUDA {

// Structure defining a device image
template <typename T>
struct TDeviceImage
{
	__host__ __device__ TDeviceImage()
		: data(NULL), imageSize(0, 0), stride(0), channels(0) {}

	TDeviceImage(T* data_, const MVS::CUDA::Point2i& _imageSize, int _stride, int _channels)
		: data(data_), imageSize(_imageSize), stride(_stride), channels(_channels) {}

	T* __restrict__ data;
	Point2i imageSize;
	int stride;
	int channels;
};
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS
