/*
* Maths.h
*
* Copyright (c) 2014-2024 SEACAVE
*
* Author(s):
*
*	  cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*	  You are required to preserve legal notices and author attributions in
*	  that material or in the Appropriate Legal Notices displayed by works
*	  containing it.
*/

#pragma once

#ifndef _MVS_MATHSCUDA_H_
#define _MVS_MATHSCUDA_H_


// I N C L U D E S /////////////////////////////////////////////////

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <float.h>

// CUDA toolkit
#include <cuda_runtime.h>
#include <math_constants.h>  // next to cuda_runtime.h
#include <cuda_runtime_api.h>
#include <curand_kernel.h>
#include <vector_types.h>

// Eigen
#ifdef __CUDACC__
#pragma push_macro("EIGEN_DEFAULT_DENSE_INDEX_TYPE")
#undef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE int
#endif
#include <Eigen/Dense>
#include <Eigen/Geometry>
#ifdef __CUDACC__
#pragma pop_macro("EIGEN_DEFAULT_DENSE_INDEX_TYPE")
#endif

#include "../../Common/UtilCUDADevice.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

#ifndef __CUDACC__
// host implementations of CUDA functions
constexpr int max(int a, int b) {
	return a > b ? a : b;
}
constexpr int min(int a, int b) {
	return a < b ? a : b;
}

inline float rsqrtf(float x) {
	return 1.f / sqrtf(x);
}
#endif

// CUDA helper math functions
__host__ __device__ inline float2 make_float2(float s) {
	return make_float2(s, s);
}
__host__ __device__ inline float2 make_float2(int2 a) {
	return make_float2(float(a.x), float(a.y));
}
__host__ __device__ inline float3 make_float3(int3 a) {
	return make_float3(float(a.x), float(a.y), float(a.z));
}
inline __device__ __host__ int clamp(int f, int a, int b) {
	return max(a, min(f, b));
}
__host__ __device__ inline float2 operator-(float2& a) {
	return make_float2(-a.x, -a.y);
}
__host__ __device__ inline int2 operator-(int2& a) {
	return make_int2(-a.x, -a.y);
}
__host__ __device__ inline float3 operator-(float3& a) {
	return make_float3(-a.x, -a.y, -a.z);
}
__host__ __device__ inline int3 operator-(int3& a) {
	return make_int3(-a.x, -a.y, -a.z);
}
__host__ __device__ inline int3 operator-(int3 a, int b) {
	return make_int3(a.x - b, a.y - b, a.z - b);
}
__host__ __device__ inline float2 operator-(float2 a, float2 b) {
	return make_float2(a.x - b.x, a.y - b.y);
}
__host__ __device__ inline float3 operator-(float3 a, float b) {
	return make_float3(a.x - b, a.y - b, a.z - b);
}
__host__ __device__ inline float3 operator-(float3 a, float3 b) {
	return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
__host__ __device__ inline float2 operator+(float2 a, float b) {
	return make_float2(a.x + b, a.y + b);
}
__host__ __device__ inline float3 operator+(float3 a, float b) {
	return make_float3(a.x + b, a.y + b, a.z + b);
}
__host__ __device__ inline int2 operator+(int2 a, int2 b) {
	return make_int2(a.x + b.x, a.y + b.y);
}
__host__ __device__ inline int3 operator+(int3 a, int3 b) {
	return make_int3(a.x + b.x, a.y + b.y, a.z + b.z);
}
__host__ __device__ inline float3 operator+(float3 a, float3 b) {
	return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
__host__ __device__ inline float2 operator*(float2 a, float b) {
	return make_float2(a.x * b, a.y * b);
}
__host__ __device__ inline float3 operator*(float3 a, float b) {
	return make_float3(a.x * b, a.y * b, a.z * b);
}
__host__ __device__ inline float2 operator/(float2 a, float b) {
	return make_float2(a.x / b, a.y / b);
}
__host__ __device__ inline float3 operator/(float3 a, float b) {
	return make_float3(a.x / b, a.y / b, a.z / b);
}
__host__ __device__ inline float dot(float2 a, float2 b) {
	return a.x * b.x + a.y * b.y;
}
__host__ __device__ inline float dot(float3 a, float3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
__host__ __device__ inline float length(float2 v) {
	return sqrtf(dot(v, v));
}
__host__ __device__ inline float length(float3 v) {
	return sqrtf(dot(v, v));
}
__host__ __device__ inline float2 normalize(float2 v) {
	return v * rsqrtf(dot(v, v));
}
__host__ __device__ inline float3 normalize(float3 v) {
	return v * rsqrtf(dot(v, v));
}
__host__ __device__ inline float3 cross(float3 a, float3 b) {
	return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
/*----------------------------------------------------------------*/


namespace MVS {

namespace CUDA {

// common point and matrix types
typedef Eigen::Matrix<int32_t, 2, 1> Point2i;
typedef Eigen::Matrix<int32_t, 3, 1> Point3i;
typedef Eigen::Matrix<uint32_t, 2, 1> Point2u;
typedef Eigen::Matrix<uint32_t, 3, 1> Point3u;
typedef Eigen::Matrix<float, 2, 1> Point2;
typedef Eigen::Matrix<float, 3, 1> Point3;
typedef Eigen::Matrix<float, 4, 1> Point4;
typedef Eigen::Matrix<float, 3, 3> Matrix3;

// convert between float2 and Point2
__host__ __device__ inline float2 Convert(const MVS::CUDA::Point2 m) {
	return make_float2(m[0], m[1]);
}
__host__ __device__ inline MVS::CUDA::Point2 Convert(const float2 m) {
	return MVS::CUDA::Point2(m.x, m.y);
}
// convert between float3 and Point3
__host__ __device__ inline float3 Convert(const MVS::CUDA::Point3 m) {
	return make_float3(m[0], m[1], m[2]);
}
__host__ __device__ inline MVS::CUDA::Point3 Convert(const float3 m) {
	return MVS::CUDA::Point3(m.x, m.y, m.z);
}
// convert between int2 and Point2i
__host__ __device__ inline int2 Convert(const MVS::CUDA::Point2i m) {
	return make_int2(m[0], m[1]);
}
__host__ __device__ inline MVS::CUDA::Point2i Convert(const int2 m) {
	return MVS::CUDA::Point2i(m.x, m.y);
}
// convert between int3 and Point3i
__host__ __device__ inline int3 Convert(const MVS::CUDA::Point3i m) {
	return make_int3(m[0], m[1], m[2]);
}
__host__ __device__ inline MVS::CUDA::Point3i Convert(const int3 m) {
	return MVS::CUDA::Point3i(m.x, m.y, m.z);
}
/*----------------------------------------------------------------*/


#ifdef __CUDACC__
// round and clamp to uint8
__device__ inline uint8_t RoundAndClampToUint8(const float x) {
	return clamp(__float2int_rn(x), 0, 255);
}

// round and floor to int
__device__ inline Point2i RoundToInt2(const Point2& v) {
	return Point2i(__float2int_rn(v.x()), __float2int_rn(v.y()));
}
__device__ inline Point2i FloorToInt2(const Point2& v) {
	return Point2i(__float2int_rd(v.x()), __float2int_rd(v.y()));
}

__device__ inline Point3i RoundToInt3(const Point3& v) {
	return Point3i(__float2int_rn(v.x()), __float2int_rn(v.y()), __float2int_rn(v.z()));
}
__device__ inline Point3i FloorToInt3(const Point3& v) {
	return Point3i(__float2int_rd(v.x()), __float2int_rd(v.y()), __float2int_rd(v.z()));
}
#endif


template <typename T>
__host__ __device__ constexpr T Square(const T x) {
	return x * x;
}

template <typename T>
__host__ __device__ constexpr void Swap(T& a, T& b) {
	const T c(a);
	a = b;
	b = c;
}

// linear interpolation
__host__ __device__ inline float lerp(float a, float b, float t) {
	return a + (b-a) * t;
}
__host__ __device__ inline Point3 lerp(Point3 a, Point3 b, float t) {
	return a + (b-a) * t;
}

#ifdef __CUDACC__
// thread index
inline __device__ int GetThreadIndex() {
	return blockIdx.x * blockDim.x + threadIdx.x;
}

inline __device__ Point2i GetThreadIndex2() {
	return Point2i(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y);
}

inline __device__ Point3i GetThreadIndex3() {
	return Point3i(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y, blockIdx.z * blockDim.z + threadIdx.z);
}
#endif

// ensure the total threads per block equal the given threadsPerBlock
inline dim3& EnsureThreadsPerBlock(unsigned threadsPerBlock, dim3& blockSize) {
	while (true) {
		const unsigned productCheck = blockSize.x * blockSize.y * blockSize.z;
		ASSERT(productCheck >= threadsPerBlock);
		if (productCheck == threadsPerBlock)
			break;
		if (blockSize.z > 1)
			--blockSize.z;
		else if (blockSize.y > 1)
			--blockSize.y;
		else
			--blockSize.x;
	}
	return blockSize;
}
// get the number of threads per block such that the threads are equal in each of the first two dimensions
inline dim3 GetThreadsPerBlock2(unsigned threadsPerBlock) {
	unsigned threadsPerBlockX = static_cast<unsigned>(ceil(sqrt(threadsPerBlock)));
	dim3 blockSize(threadsPerBlockX, threadsPerBlockX, 1);
	return EnsureThreadsPerBlock(threadsPerBlock, blockSize);
}
// get the number of blocks, each of blockSize, such that the blocks cover all elements
inline unsigned GetBlockCount(unsigned blockSize, unsigned elements) {
	return (elements + blockSize - 1) / blockSize;
}
// default threads per block
inline unsigned DefaultThreadsPerBlock() {
	return 512;
}
inline std::pair<unsigned,unsigned> DefaultBlockCount(unsigned elements) {
	const unsigned threadsPerBlock = DefaultThreadsPerBlock();
	const unsigned gridSize = GetBlockCount(threadsPerBlock, elements);
	return {gridSize, threadsPerBlock};
}

// convert 2D to 1D coordinates and back
__host__ __device__ inline int Point2Idx(const Point2i& p, int width) {
	return p.y() * width + p.x();
}
__host__ __device__ inline int Point2Idx(int stride, const Point2i& pixel, int numChannels = 1) {
	return pixel.y() * stride + pixel.x() * numChannels;
}
__host__ __device__ inline int Point2Idx(int width, int channels, const Point2i& pixel) {
	return pixel.y() * (width * channels) + pixel.x() * channels;
}
__host__ __device__ inline Point2i Idx2Point(int idx, int width) {
	return Point2i(idx % width, idx / width);
}

// check if a pixel is inside the image
__host__ __device__ inline bool IsInImage(const int x, const int y, const int width, const int height) {
	return x >= 0 && y >= 0 && x < width && y < height;
}
__host__ __device__ inline bool IsInImage(const Point2i& pixel, const Point2i& size) {
	return pixel.x() >= 0 && pixel.y() >= 0 && pixel.x() < size.x() && pixel.y() < size.y();
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS

#endif // _MVS_MATHSCUDA_H_
