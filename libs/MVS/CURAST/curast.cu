#include <thrust/execution_policy.h>
#include <thrust/system/cuda/execution_policy.h>
#include <cuda_runtime.h>
#include <device_atomic_functions.h>  
#include <cuda_fp16.h>
#include <thrust/device_ptr.h>
#include <thrust/fill.h>
#include <thrust/transform_reduce.h>
#include <thrust/pair.h>
#include <thrust/device_vector.h>
#include <thrust/transform.h>
#include <thrust/sort.h>
#include <thrust/universal_vector.h>
#include <thrust/host_vector.h>
#include <thrust/count.h>
#include <thrust/unique.h>
#include <thrust/extrema.h>
#include <thrust/iterator/constant_iterator.h>
#include "curast.h"
namespace MVS {
namespace CUDA {
// gaussian kernels

__constant__ int kernel9[9] = {
	 1, 2, 1,
	 2, 4, 2,
	 1, 2, 1
};
__constant__ int kernel25[25] = {
	 1, 4, 6, 4, 1,
	 4,16,24,16,4,
	 6,24,36,24,6,
	 4,16,24,16,4,
	 1, 4, 6, 4, 1
};
__constant__ int kernel81[81] = {
	1,  1,  2,  2,  2,  2,  2,  1,  1,
	1,  2,  4,  5,  5,  5,  4,  2,  1,
	2,  4,  8, 10, 10, 10,  8,  4,  2,
	2,  5, 10,16, 18, 16, 10,  5,  2,
	2,  5, 10,18, 24, 18, 10,  5,  2,
	2,  5, 10,16, 18, 16, 10,  5,  2,
	2,  4,  8,10, 10, 10,  8,  4,  2,
	1,  2,  4, 5,   5,  5,  4,  2,  1,
	1,  1,  2, 2,   2,  2,  2,  1,  1
};
__constant__ int kernel169[169] = {
	1,1,2,2,4,4,5,4,4,2,2,1,1,
	1,2,3,4,6,7,8,7,6,4,3,2,1,
	2,3,6,7,10,12,14,12,10,7,6,3,2,
	2,4,7, 10,14,18,20,18,14,10,7,4,2,
	4,6,10,14,20,26,30,26,20,14,10,6,4,
	4,7,12,18,26,36,40,36,26,18,12,7,4,
	5,8,14,20,30,40,48,40,30,20,14,8,5,
	4,7,12,18,26,36,40,36,26,18,12,7,4,
	4,6,10,14,20,26,30,26,20,14,10,6,4,
	2,4,7,10,14,18,20,18,14,10, 7,4,2,
	2,3,6,7,10,12,14,12,10,7,6,3,2,
	1,2,3,4,6,7,8,7,6,4,3,2,1,
	1,1,2,2,4,4,5,4,4,2,2,1,1
};
__constant__ int kernel289[289] = {
	1,  1,  2,  2,  4,  4,  5,  6,  6,  6,  5,  4,  4,  2,  2,  1,  1,
	1,  2,  3,  4,  6,  7,  8,  9, 10,  9,  8,  7,  6,  4,  3,  2,  1,
	2,  3,  6,  7, 10, 12, 14, 16, 18, 16, 14, 12, 10,  7,  6,  3,  2,
	2,  4,  7, 10, 14, 18, 20, 23, 26, 23, 20, 18, 14, 10,  7,  4,  2,
	4,  6, 10, 14, 20, 26, 30, 34, 38, 34, 30, 26, 20, 14, 10,  6,  4,
	4,  7, 12, 18, 26, 36, 40, 46, 52, 46, 40, 36, 26, 18, 12,  7,  4,
	5,  8, 14, 20, 30, 40, 48, 56, 64, 56, 48, 40, 30, 20, 14,  8,  5,
	6,  9, 16, 23, 34, 46, 56, 66, 76, 66, 56, 46, 34, 23, 16,  9,  6,
	6, 10, 18, 26, 38, 52, 64, 76, 88, 76, 64, 52, 38, 26, 18, 10,  6,
	6,  9, 16, 23, 34, 46, 56, 66, 76, 66, 56, 46, 34, 23, 16,  9,  6,
	5,  8, 14, 20, 30, 40, 48, 56, 64, 56, 48, 40, 30, 20, 14,  8,  5,
	4,  7, 12, 18, 26, 36, 40, 46, 52, 46, 40, 36, 26, 18, 12,  7,  4,
	4,  6, 10, 14, 20, 26, 30, 34, 38, 34, 30, 26, 20, 14, 10,  6,  4,
	2,  4,  7, 10, 14, 18, 20, 23, 26, 23, 20, 18, 14, 10,  7,  4,  2,
	2,  3,  6,  7, 10, 12, 14, 16, 18, 16, 14, 12, 10,  7,  6,  3,  2,
	1,  2,  3,  4,  6,  7,  8,  9, 10,  9,  8,  7,  6,  4,  3,  2,  1,
	1,  1,  2,  2,  4,  4,  5,  6,  6,  6,  5,  4,  4,  2,  2,  1,  1
};

struct DiffCount {
	float diff;
	int count;
	__host__ __device__ DiffCount(float d = 0.f, int c = 0) : diff(d), count(c) {}
};

struct DiffCountPlus {
	__host__ __device__ DiffCount operator()(const DiffCount& a, const DiffCount& b) const {
		return DiffCount(a.diff + b.diff, a.count + b.count);
	}
};

;__device__ inline Point2 worldToTilePixel(
	const Point3& P,
	const Point2& tileOrigin,
	const Point2& tileSize,
	const Point2i& tileRes)
{
	const float u = (P.x() - tileOrigin.x()) / tileSize.x();
	const float v = (P.y() - tileOrigin.y()) / tileSize.y();

	const float px = u * float(tileRes.x() - 1);
	const float py =(1 - v) * float(tileRes.y() - 1);

	return Point2{px, py};
}
 
__device__ bool inverse3x3(const Matrix3& A, Matrix3& invA)
{
    float det =
        A(0)*(A(4)*A(8)-A(5)*A(7)) -
        A(1)*(A(3)*A(8)-A(5)*A(6)) +
        A(2)*(A(3)*A(7)-A(4)*A(6));

    if (fabs(det) < 1e-8f)
        return false;

    float invDet = 1.0f/det;

    invA(0) = (A(4)*A(8)-A(5)*A(7))*invDet;
    invA(1) = (A(2)*A(7)-A(1)*A(8))*invDet;
    invA(2) = (A(1)*A(5)-A(2)*A(4))*invDet;

    invA(3) = (A(5)*A(6)-A(3)*A(8))*invDet;
    invA(4) = (A(0)*A(8)-A(2)*A(6))*invDet;
    invA(5) = (A(2)*A(3)-A(0)*A(5))*invDet;

    invA(6) = (A(3)*A(7)-A(4)*A(6))*invDet;
    invA(7) = (A(1)*A(6)-A(0)*A(7))*invDet;
    invA(8) = (A(0)*A(4)-A(1)*A(3))*invDet;

    return true;
}

__device__ inline float edgeFn(const Point2& a, const Point2& b, const Point2& p) {
	return (p.x() - a.x())*(b.y() - a.y()) - (p.y() - a.y())*(b.x() - a.x());
}
__device__ inline float edgeFn(const Point2 pts[3]) {
	return edgeFn(pts[0], pts[1], pts[2]);
}

// bilinear sampling in normalized coords (u,v in [0,1]) for 3-channel float image
__device__ inline Point3 samplePhotoBilinear3C(
	const float* photo, const Point2i size, float u, float v) {
	const uint W = size.x();
	const uint H = size.y();

	// clamp to [0,1]
	u = fminf(fmaxf(u, 0.f), 1.f);
	v = fminf(fmaxf(v, 0.f), 1.f);

	// convert to pixel space (center sampling)
	float x = u * (W - 1);
	float y = v * (H - 1);

	int x0 = (int)floorf(x);
	int x1 = min(x0 + 1, W - 1);
	int y0 = (int)floorf(y);
	int y1 = min(y0 + 1, H - 1);

	float tx = x - x0;
	float ty = y - y0;

	auto fetch = [&](int ix, int iy) {
		int idx = (iy * W + ix) * 3;
		return  Point3{
				photo[idx],
				photo[idx+1],
				photo[idx+2]
		};
	};

	Point3 c00 = fetch(x0, y0);
	Point3 c10 = fetch(x1, y0);
	Point3 c01 = fetch(x0, y1);
	Point3 c11 = fetch(x1, y1);
	Point3 cx0, cx1;
	cx0 = {c00.x() + (c10.x() - c00.x())*tx,
			c00.y() + (c10.y() - c00.y())*tx,
			c00.z() + (c10.z() - c00.z())*tx};
	cx1 = {c01.x() + (c11.x() - c01.x())*tx,
			c01.y() + (c11.y() - c01.y())*tx,
			c01.z() + (c11.z() - c01.z())*tx};

	return Point3{
		cx0.x() + (cx1.x() - cx0.x())*ty,
		cx0.y() + (cx1.y() - cx0.y())*ty,
		cx0.z() + (cx1.z() - cx0.z())*ty
	};
}

namespace CURAST {

// fill device buffer using thrust
template <typename T>
void Fill(T* data, T value, uint nElement, cudaStream_t stream) {
	thrust::device_ptr<T> dev_ptr(data);
	thrust::fill(thrust::cuda::par.on(stream), dev_ptr, dev_ptr + nElement, value);
}
template void MVS::CUDA::CURAST::Fill<int>(int*, int, uint, cudaStream_t);
template void MVS::CUDA::CURAST::Fill<float>(float*, float, uint, cudaStream_t);
/*----------------------------------------------------------------*/

// jump flood related kernels and functions
__device__ float dist2(int x1, int y1, int x2, int y2) {
	float dx = float(x1 - x2);
	float dy = float(y1 - y2);
	return dx * dx + dy * dy;
}
__global__ void initSeeds(const float* edgeMap, Seed* seedMap, int width, int height) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;
	int idx = y * width + x;
	if (edgeMap[idx] == 1.f) {
		seedMap[idx].x = x;
		seedMap[idx].y = y;
	} else {
		seedMap[idx].x = -1;
		seedMap[idx].y = -1;
	}
}
__global__ void initSeedsWithATag(const int* __restrict__ tagMap, Seed* __restrict__ seedMap, Point2i size, uint tag) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= size.x() || y >= size.y()) return;
	int idx = y * size.x() + x;
	if (tagMap[idx] == tag) {
		seedMap[idx].x = x;
		seedMap[idx].y = y;
	} else {
		seedMap[idx].x = -1;
		seedMap[idx].y = -1;
	}
}
__global__ void initSeeds(const uint* variationMap, Seed* seedMap, int width, int height) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;
	int idx = y * width + x;
	if (variationMap[idx] == 1) {
		seedMap[idx].x = x;
		seedMap[idx].y = y;
	} else {
		seedMap[idx].x = -1;
		seedMap[idx].y = -1;
	}
}
__global__ void jumpFloodPass(const Seed* seedMap_in, Seed* seedMap_out, int width, int height, int jump) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;
	int idx = y * width + x;
	Seed bestSeed = seedMap_in[idx];
	float bestDist = (bestSeed.x == -1) ? 1e20f : dist2(x, y, bestSeed.x, bestSeed.y);
	// check neighbors at distance 'jump'
	for (int oy = -jump; oy <= jump; oy += jump) {
		for (int ox = -jump; ox <= jump; ox += jump) {
			if (ox == 0 && oy == 0) continue;
			int nx = x + ox;
			int ny = y + oy;
			if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
			int nidx = ny * width + nx;
			Seed s = seedMap_in[nidx];
			if (s.x != -1) {
				float d = dist2(x, y, s.x, s.y);
				if (d < bestDist) {
					bestDist = d;
					bestSeed = s;
				}
			}
		}
	}
	seedMap_out[idx] = bestSeed;
}
__global__ void computeDistance(const Seed* seedMap, float* distMap, int width, int height, float maxDist = 1.) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	int idx = y * width + x;
	Seed s = seedMap[idx];
	float dx = float(x - s.x);
	float dy = float(y - s.y);
	float dist = sqrtf(dx * dx + dy * dy);
	distMap[idx] = dist/maxDist;
}
__global__ void computeTruncDistance(const Seed* seedMap, float* distMap, int width, int height, float maxDist = 1.) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	int idx = y * width + x;
	Seed s = seedMap[idx];
	float dx = float(x - s.x);
	float dy = float(y - s.y);
	float dist = sqrtf(dx * dx + dy * dy);
	distMap[idx] = min(1.f,dist/maxDist);
}
__global__ void computeDistanceCustom(const Seed* seedMap, float* distMap, int width, int height) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height) return;

	int idx = y * width + x;
	Seed s = seedMap[idx];
	int maxDim = max(width, height);
	// distance to image borders 
	float minCoord = min(float(x), float(y));
	distMap[idx] = 1;
	float distBorder = 1;
	float distEdge = 1;
	distBorder = sqrt((min(minCoord*100.f/(maxDim*5.f), 1.f)+0.f)/1.f);
	// distance to nearest edge
	if (s.x != -1) {
		float dx = float(x - s.x);
		float dy = float(y - s.y);
		float dist = sqrtf(dx * dx + dy * dy);
		distEdge = sqrt((min(dist*100.f/(maxDim*0.20f), 1.f)+0.f)/1.f);
	}
	distMap[idx] = distBorder*distEdge;
}

/*----------------------------------------------------------------*/

// extract a single image from the nth slot of a per-pixel stack
template<typename T>
__global__ void ExtractImgFromStackKernel(const T* __restrict__ d_stack, T* __restrict__ d_output,
	Point2i imgSize,
	uint stackSize,
	int chanels,
	uint nStack) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= imgSize.x() || y >= imgSize.y()) return;
	int idx = y * imgSize.x() + x;
	for (int i = 0; i < chanels; i++)
		d_output[idx * chanels + i] = d_stack[idx * stackSize * chanels + nStack * chanels + i];
}

template<typename T>
void ExtractImgFromStack(const T* d_stack, T* d_output,
	Point2i imgSize, // size of the image
	uint stackSize, // size of the stack
	int chanels, // number of chanels
	uint nStack, // which stack to extract
	cudaStream_t stream) {
	dim3 blockSize = dim3(16, 16);
	dim3 gridSize((imgSize.x() + blockSize.x - 1) / blockSize.x, (imgSize.y() + blockSize.y - 1) / blockSize.y);

	ExtractImgFromStackKernel<<<gridSize, blockSize, 0, stream>>>(
		d_stack, d_output, imgSize, stackSize, chanels, nStack);
}
template void MVS::CUDA::CURAST::ExtractImgFromStack<float>(const float* d_stack, float* d_output, Point2i imgSize, uint stackSize, int chanels, uint nStack, cudaStream_t stream);
template void MVS::CUDA::CURAST::ExtractImgFromStack<int>(const int* d_stack, int* d_output, Point2i imgSize, uint stackSize, int chanels, uint nStack, cudaStream_t stream);

/*----------------------------------------------------------------*/

// atomic float minimum using compare-and-swap on the integer representation
__device__ float atomicMinFloat(float* addr, float value) {
	int* addr_as_int = (int*)addr;
	int old = *addr_as_int, assumed;
	do {
		assumed = old;
		if (__int_as_float(assumed) <= value) break;
		old = atomicCAS(addr_as_int, assumed, __float_as_int(value));
	} while (assumed != old);
	return __int_as_float(old);
}
// atomic float maximum using compare-and-swap on the integer representation
__device__ float atomicMaxFloat(float* addr, float value) {
	int* addr_as_int = (int*)addr;
	int old = *addr_as_int, assumed;
	do {
		assumed = old;
		if (__int_as_float(assumed) >= value) break;
		old = atomicCAS(addr_as_int, assumed, __float_as_int(value));
	} while (assumed != old);
	return __int_as_float(old);
}
// project mesh onto image plane 
// paralellization is done per triangle and per pixel in the bounding box of the triangle
__global__ void ProjectMeshKernel(float* depthMap, int* faceMap, float* normalMap, const Point3* positions, const uint numFaces, const CUDA::Camera camera, const Point2i imgSize) {
	int triId = blockIdx.x; // one block per triangle
	if (triId >= numFaces) return;

	Point3 vPositions[3] = {
		positions[triId*3],
		positions[triId*3+1],
		positions[triId*3+2]
	};

	// get vertices projections on image plane
	Point2  pImg[3];
	bool isBehind = false;
	for (int i = 0; i < 3; i++) {
		Point3 P_cam = camera.R * (vPositions[i] - camera.C);
		Point3 P_img = camera.K * P_cam;
		pImg[i] = Point2{P_img.x() / P_img.z(), P_img.y() / P_img.z()};
		if (P_img.z() <= 0) {
			isBehind = true;
		}
	}
	if (isBehind) return; // triangle is behind camera
	// check if triangle is in image bounds
	if ((pImg[0].x() < 0 && pImg[1].x() < 0 && pImg[2].x() < 0) ||
		(pImg[0].x() >= imgSize.x() && pImg[1].x() >= imgSize.x() && pImg[2].x() >= imgSize.x()) ||
		(pImg[0].y() < 0 && pImg[1].y() < 0 && pImg[2].y() < 0) ||
		(pImg[0].y() >= imgSize.y() && pImg[1].y() >= imgSize.y() && pImg[2].y() >= imgSize.y())) {
		return;
	}

	// get bounding square for the image triangle
	uint minX = (uint)floor(fminf(fminf(pImg[0].x(), pImg[1].x()), pImg[2].x()));
	uint minY = (uint)floor(fminf(fminf(pImg[0].y(), pImg[1].y()), pImg[2].y()));
	uint maxX = (uint)ceil(fmaxf(fmaxf(pImg[0].x(), pImg[1].x()), pImg[2].x()));
	uint maxY = (uint)ceil(fmaxf(fmaxf(pImg[0].y(), pImg[1].y()), pImg[2].y()));

	// clamp to image size
	minX = MAX(minX, 0.f);
	minY = MAX(minY, 0.f);
	maxX = MIN(maxX, imgSize.x() - 1.f);
	maxY = MIN(maxY, imgSize.y() - 1.f);

	float area = edgeFn(pImg);
	if (area < 1e-10f) return; // degenerate triangle

	// rasterize the triangle
	uint width = maxX - minX + 1;
	uint height = maxY - minY + 1;

	for (int i = threadIdx.x; i < width*height; i += blockDim.x) {
		int x = i % width + minX;
		int y = i / width + minY;
		Point2 P = {x, y};
		// barycentric coordinates
		float w0 = edgeFn(pImg[1], pImg[2], P);
		float w1 = edgeFn(pImg[2], pImg[0], P);
		float w2 = edgeFn(pImg[0], pImg[1], P);
		// if P is inside the triangle
		if (!((w0 >= 0 && w1 >= 0 && w2 >= 0 && area > 0) ||
			(w0 <= 0 && w1 <= 0 && w2 <= 0 && area < 0))) {
			continue;
		}
		// normalize barycentric coordinates
		float invArea = 1.0f / area;
		w0 *= invArea;
		w1 *= invArea;
		w2 *= invArea;
		// on mesh corrdinates
		Point3 Pw = vPositions[0] * w0 + vPositions[1] * w1 + vPositions[2] * w2;
		// camera space corrdinates
		Point3 Pc = camera.R * (Pw - camera.C);
		float depth = Pc.z();
		if (depth <= 0) continue; // behind the camera
		// compute normal and view direction
		Point3 normalW = (vPositions[1] - vPositions[0]).cross(vPositions[2] - vPositions[0]);
		normalW.normalize();
		Point3 normalC = camera.R * normalW;
		Point3 viewDirC = (-Pc).normalized();
		float cosTheta = normalC.dot(viewDirC);

		// atomic depth buffer test and update
		int pixIdx = y * imgSize.x() + x;
		float oldDepth = atomicMinFloat(&depthMap[pixIdx], depth);
		if (depth < oldDepth  && oldDepth > 0) {
			faceMap[pixIdx] = triId;
			normalMap[pixIdx] = abs(cosTheta);
		}
	}
}
// orthographic mesh projection kernel
// paralellization is done per triangle and per pixel in the bounding box of the triangle
__global__ void ProjectMeshKernelOrtho(float* depthMap, int* faceMap, float* normalMap, const Point3* positions, const uint numFaces, const CUDA::Camera camera, const Point2i imgSize) {
	int triId = blockIdx.x; // one block per triangle
	if (triId >= numFaces) return;

	Point3 vPositions[3] = {
		positions[triId*3],
		positions[triId*3+1],
		positions[triId*3+2]
	};

	// get vertices projections on image plane
	Point2  pImg[3];
	bool isBehind = false;
	for (int i = 0; i < 3; i++) {
		Point3 P_cam = camera.R * (vPositions[i] - camera.C);
		Point3 P_img = camera.K * P_cam;
		pImg[i] = Point2{P_img.x() / P_img.z(), P_img.y() / P_img.z()};
		if (P_img.z() <= 0) {
			isBehind = true;
		}
	}
	if (isBehind) return; // triangle is behind camera
	// check if triangle is in image bounds
	if ((pImg[0].x() < 0 && pImg[1].x() < 0 && pImg[2].x() < 0) ||
		(pImg[0].x() >= imgSize.x() && pImg[1].x() >= imgSize.x() && pImg[2].x() >= imgSize.x()) ||
		(pImg[0].y() < 0 && pImg[1].y() < 0 && pImg[2].y() < 0) ||
		(pImg[0].y() >= imgSize.y() && pImg[1].y() >= imgSize.y() && pImg[2].y() >= imgSize.y())) {
		return;
	}

	// get bounding square for the image triangle
	uint minX = (uint)floor(fminf(fminf(pImg[0].x(), pImg[1].x()), pImg[2].x()));
	uint minY = (uint)floor(fminf(fminf(pImg[0].y(), pImg[1].y()), pImg[2].y()));
	uint maxX = (uint)ceil(fmaxf(fmaxf(pImg[0].x(), pImg[1].x()), pImg[2].x()));
	uint maxY = (uint)ceil(fmaxf(fmaxf(pImg[0].y(), pImg[1].y()), pImg[2].y()));

	// clamp to image size
	minX = MAX(minX, 0.f);
	minY = MAX(minY, 0.f);
	maxX = MIN(maxX, imgSize.x() - 1.f);
	maxY = MIN(maxY, imgSize.y() - 1.f);

	float area = edgeFn(pImg);
	if (area < 1e-10f) return; // degenerate triangle

	// rasterize the triangle
	uint width = maxX - minX + 1;
	uint height = maxY - minY + 1;

	for (int i = threadIdx.x; i < width*height; i += blockDim.x) {
		int x = i % width + minX;
		int y = i / width + minY;
		Point2 P = {x, y};
		// barycentric coordinates
		float w0 = edgeFn(pImg[1], pImg[2], P);
		float w1 = edgeFn(pImg[2], pImg[0], P);
		float w2 = edgeFn(pImg[0], pImg[1], P);
		// if P is inside the triangle
		if (!((w0 >= 0 && w1 >= 0 && w2 >= 0 && area > 0) ||
			(w0 <= 0 && w1 <= 0 && w2 <= 0 && area < 0))) {
			continue;
		}
		// normalize barycentric coordinates
		float invArea = 1.0f / area;
		w0 *= invArea;
		w1 *= invArea;
		w2 *= invArea;
		// on mesh corrdinates
		Point3 Pw = vPositions[0] * w0 + vPositions[1] * w1 + vPositions[2] * w2;
		// camera space corrdinates
		Point3 camToPoint = camera.C - Pw;
		Point3 Pc = - camera.R * camToPoint;
		float depth = Pc.z();
		if (depth <= 0) continue; // behind the camera
		// compute normal and view direction
		Point3 normalW = (vPositions[1] - vPositions[0]).cross(vPositions[2] - vPositions[0]);
		normalW.normalize();
		float norm = camToPoint.norm();
		camToPoint.normalize();
		float t2 = abs(camToPoint.z())/norm;
		float t1 = camToPoint.dot(normalW);
		//if (t1 < 0.3f) continue;
		if (t1 < 0 || t2 < 0 ) continue;
		float cosTheta = t2;
		// atomic depth buffer test and update
		int pixIdx = y * imgSize.x() + x;
		float oldDepth = atomicMinFloat(&depthMap[pixIdx], depth);
		if (depth < oldDepth  && oldDepth > 0) {
			faceMap[pixIdx] = triId;
			normalMap[pixIdx] = cosTheta;
		}
	}
}

// project the mesh onto the image plane to create depth map and face map, and normal map
void ProjectMesh(float* depthMap, int* faceMap, float* normalMap, Point3* positions, uint numTriangles, const CUDA::Camera camera, const Point2i imgSize, cudaStream_t stream) {
	const uint blockSize = 64;
	const uint gridSize = numTriangles;
	// reset faceMap, depthMap and normalMap on the caller's stream so subsequent
	// kernel launches on the same stream see the cleared buffers (without
	// thrust::cuda::par.on(stream) these would dispatch on Thrust's default stream
	// and race with the kernel below)
	thrust::device_ptr<int> facePtr(faceMap);
	thrust::device_ptr<float> depthPtr(depthMap);
	thrust::device_ptr<float> normalPtr(normalMap);
	const auto policy = thrust::cuda::par.on(stream);
	thrust::fill(policy, facePtr, facePtr + imgSize.x() * imgSize.y(), -1);
	thrust::fill(policy, depthPtr, depthPtr + imgSize.x() * imgSize.y(), FLT_MAX);
	thrust::fill(policy, normalPtr, normalPtr + imgSize.x() * imgSize.y(), 0.f);
	ProjectMeshKernel<<<gridSize, blockSize, 0, stream>>>(depthMap, faceMap, normalMap, positions, numTriangles, camera, imgSize);
	CUDA_CHECK_LAST_ERROR;
}
// ortho projection of the mesh onto the image plane to create depth map and face map, and normal map
void ProjectMeshOrtho(float* depthMap, int* faceMap, float* normalMap, Point3* positions, uint numTriangles, const CUDA::Camera camera, const Point2i imgSize, cudaStream_t stream) {
	const uint blockSize = 64;
	const uint gridSize = numTriangles;
	// reset faceMap, depthMap and normalMap
	thrust::device_ptr<int> facePtr(faceMap);
	thrust::device_ptr<float> depthPtr(depthMap);
	thrust::device_ptr<float> normalPtr(normalMap);
	thrust::fill(facePtr, facePtr + imgSize.x() * imgSize.y(), -1);
	thrust::fill(depthPtr, depthPtr + imgSize.x() * imgSize.y(), FLT_MAX);
	thrust::fill(normalPtr, normalPtr + imgSize.x() * imgSize.y(), 0.f);
	CUDA_CHECK_LAST_ERROR;
	ProjectMeshKernelOrtho<<<gridSize, blockSize, 0, stream>>>(depthMap, faceMap, normalMap, positions, numTriangles, camera, imgSize);
	CUDA_CHECK_LAST_ERROR;
}

// rasterize the orthographic normal map for a tile.
// projects mesh triangles into the tile's XY plane (using worldToTilePixel) and writes
// cos(inclination) = |normalW.z| at each covered pixel, keeping only the topmost surface
// (atomicMax on a depth buffer based on world Z). Backface triangles (normalW.z < 0) are culled.
__global__ void ProjectNormalOrthoMap(float* depthMap, float* normalMap, const Point3* positions, const uint numFaces,
	const Point2 tileSize, const Point2i tileRes, const Point2 origine) {
	int triId = blockIdx.x; // one block per triangle
	if (triId >= numFaces) return;
	
	Point3 vPositions[3] = {
		positions[triId*3],
		positions[triId*3+1],
		positions[triId*3+2]
	};
	
	// get vertices projections on image plane
	Point2  pImg[3];
	for (int i = 0; i < 3; i++) {
		pImg[i] = worldToTilePixel(vPositions[i], origine, tileSize, tileRes);
	}
	// check if triangle is in image bounds
	if ((pImg[0].x() < 0 && pImg[1].x() < 0 && pImg[2].x() < 0) ||
		(pImg[0].x() >= tileRes.x() && pImg[1].x() >= tileRes.x() && pImg[2].x() >= tileRes.x()) ||
		(pImg[0].y() < 0 && pImg[1].y() < 0 && pImg[2].y() < 0) ||
		(pImg[0].y() >= tileRes.y() && pImg[1].y() >= tileRes.y() && pImg[2].y() >= tileRes.y())) {
		return;
	}
	
	// get bounding square for the image triangle
	uint minX = (uint)floor(fminf(fminf(pImg[0].x(), pImg[1].x()), pImg[2].x()));
	uint minY = (uint)floor(fminf(fminf(pImg[0].y(), pImg[1].y()), pImg[2].y()));
	uint maxX = (uint)ceil(fmaxf(fmaxf(pImg[0].x(), pImg[1].x()), pImg[2].x()));
	uint maxY = (uint)ceil(fmaxf(fmaxf(pImg[0].y(), pImg[1].y()), pImg[2].y()));
	
	// clamp to image size
	minX = MAX(minX, 0.f);
	minY = MAX(minY, 0.f);
	maxX = MIN(maxX, tileRes.x() - 1.f);
	maxY = MIN(maxY, tileRes.y() - 1.f);
	
	float area = edgeFn(pImg);
	if (area < 1e-10f && area > -1e-10f) return; // degenerate triangle

	// rasterize the triangle
	uint width = maxX - minX + 1;
	uint height = maxY - minY + 1;

	Point3 normalW = (vPositions[1] - vPositions[0]).cross(vPositions[2] - vPositions[0]);
	normalW.normalize();

	if (normalW.z() < 0) return;
	for (int i = threadIdx.x; i < width*height; i += blockDim.x) {
		int x = i % width + minX;
		int y = i / width + minY;
		Point2 P = {x, y};
		// barycentric coordinates
		float w0 = edgeFn(pImg[1], pImg[2], P);
		float w1 = edgeFn(pImg[2], pImg[0], P);
		float w2 = edgeFn(pImg[0], pImg[1], P);

		// if P is inside the triangle
		if (!((w0 >= 0 && w1 >= 0 && w2 >= 0 && area > 0) ||
			(w0 <= 0 && w1 <= 0 && w2 <= 0 && area < 0))) {
			continue;
		}
		// normalize barycentric coordinates
		float invArea = 1.0f / area;
		w0 *= invArea;
		w1 *= invArea;
		w2 *= invArea;
		// world coordinates
		Point3 Pw = vPositions[0] * w0 + vPositions[1] * w1 + vPositions[2] * w2;
		// relative to camera corrdinates
		float depth = Pw.z();
		float cosTheta = normalW.z();
		// atomic depth buffer test and update
		int pixIdx = y * tileRes.x() + x;
		float oldDepth = atomicMaxFloat(&depthMap[pixIdx], depth);
		if (depth > oldDepth ) {
			normalMap[pixIdx] = fabsf(cosTheta);
		}
	}
}

void GetNormalOrthoMap(float* normalMap, Point3* positions, uint numTriangles, const Point2 tileSize, const Point2i tileRes, cudaStream_t stream, Point2 origine){
	const uint blockSize = 64;
	const uint gridSize = numTriangles;
	// stream-ordered allocation; free is queued on the same stream below so no host sync is needed
	float* depthMap;
	const size_t numPixels = (size_t)tileRes.x() * tileRes.y();
	cudaMallocAsync(&depthMap, numPixels * sizeof(float), stream);
	thrust::device_ptr<float> depthPtr(depthMap);
	thrust::device_ptr<float> normalPtr(normalMap);
	thrust::fill(thrust::cuda::par.on(stream), depthPtr, depthPtr + numPixels, -FLT_MAX);
	thrust::fill(thrust::cuda::par.on(stream), normalPtr, normalPtr + numPixels, 0.f);
	CUDA_CHECK_LAST_ERROR;
	ProjectNormalOrthoMap<<<gridSize, blockSize, 0, stream>>>(depthMap, normalMap, positions, numTriangles, tileSize, tileRes, origine);
	CUDA_CHECK_LAST_ERROR;
	cudaFreeAsync(depthMap, stream);
}
/*----------------------------------------------------------------*/

// custom image pyramid construction kernels and functions
__global__ void GaussianBlurKernel(float* inputImage, float* outputImage, Point2i imgSize, uint nKernel) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= imgSize.x() || y >= imgSize.y())
		return;
	
	// select kernel size
	int halfKernel;
	if (nKernel == 0)
		halfKernel = 2;
	else if (nKernel == 1)
		halfKernel = 4;
	else if (nKernel == 2)
		halfKernel = 8;

	for (int c = 0; c < 3; c++) {
		float sum = 0;
		float normalizer = 0;
		for (int ky = -halfKernel; ky <= halfKernel; ky++) {
			for (int kx = -halfKernel; kx <= halfKernel; kx++) {
				int sampleX = min(max(x + kx, 0), imgSize.x() - 1);
				int sampleY = min(max(y + ky, 0), imgSize.y() - 1);
				uint idx = (sampleY * imgSize.x() + sampleX) * 3 + c;
				int kIdx = (ky + halfKernel) * (2 * halfKernel + 1) + (kx + halfKernel);

				if (nKernel == 0) {
					sum += inputImage[idx] * kernel25[kIdx];
					normalizer += kernel25[kIdx];
				} else if (nKernel == 1) {
					sum += inputImage[idx] * kernel81[kIdx];
					normalizer += kernel81[kIdx];
				} else if (nKernel == 2) {
					sum += inputImage[idx] * kernel289[kIdx];
					normalizer += kernel289[kIdx];
				}
			}
		}
		uint outIdx = (y * imgSize.x() + x) * 3 + c;
		outputImage[outIdx] = sum / normalizer;
	}
}

// compute difference between upImage and blurredImage and store result in upImage
__global__ void DifferenceKernel(float* upImage, float* blurredImage, Point2i imgSize) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= imgSize.x() || y >= imgSize.y())
		return;
	for (int c = 0; c < 3; c++) {
		uint idx = (y * imgSize.x() + x) * 3 + c;
		upImage[idx] = upImage[idx] - blurredImage[idx];
	}
}

// downsample upImage into downImage
__global__ void DownSampleKernel(float* upImage, float* downImage, Point2i downSize, Point2i previousSize) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= downSize.x() || y >= downSize.y())
		return;
	uint ratio = previousSize.x() / downSize.x();
	int srcX = x * ratio;
	int srcY = y * ratio;
	for (int c = 0; c < 3; c++) {
		uint downIdx = (y * downSize.x() + x) * 3 + c;
		uint upIdx = (min(srcY, downSize.y()*ratio - 1) * downSize.x()*ratio + min(srcX, downSize.x()*ratio - 1)) *3 + c;
		downImage[downIdx] = upImage[upIdx];
	}
}
// construct custom image pyramid, with strong downsampling at the last level
void MakeCustomPyramid(std::vector<TDeviceMat<float>>& pyramidImages, const Point2i imgSize, const uint nLevels, cudaStream_t stream) {
	Point2i currentSize = imgSize;
	for (uint l = 1; l < nLevels; l++) {
		float* tmpImg;
		cudaMalloc(&tmpImg, sizeof(float) * currentSize.x() * currentSize.y() * 3);
		dim3 blockSize(16, 16);
		dim3 gridSize((currentSize.x() + blockSize.x - 1) / blockSize.x,
					  (currentSize.y() + blockSize.y - 1) / blockSize.y);
		if (l == nLevels -1)
			GaussianBlurKernel<<<gridSize, blockSize, 0, stream>>>(pyramidImages[l - 1].GetDeviceData(), tmpImg, currentSize, 2);
		else
			GaussianBlurKernel<<<gridSize, blockSize, 0, stream>>>(pyramidImages[l - 1].GetDeviceData(), tmpImg, currentSize, 0);
		
		CUDA_CHECK_LAST_ERROR;
		DifferenceKernel<<<gridSize, blockSize, 0, stream>>>(pyramidImages[l - 1].GetDeviceData(), tmpImg, currentSize);
		CUDA_CHECK_LAST_ERROR;
		Point2i previousSize = currentSize;
		if (l == nLevels-1) {
			currentSize.x() = currentSize.x() >> 3;
			currentSize.y() = currentSize.y() >> 3;
		} else {
			currentSize.x() = currentSize.x() >> 1;
			currentSize.y() = currentSize.y() >> 1;
		}
		pyramidImages[l].Resize(cv::Size(currentSize.x(), currentSize.y()), 3);
		dim3 gridSizeDS(
			(currentSize.x() + blockSize.x - 1) / blockSize.x,
			(currentSize.y() + blockSize.y - 1) / blockSize.y
		);
		DownSampleKernel<<<gridSizeDS, blockSize, 0, stream>>>(tmpImg, pyramidImages[l].GetDeviceData(), currentSize, previousSize);
		CUDA_CHECK_LAST_ERROR;
		cudaFree(tmpImg);
	}
}
/*----------------------------------------------------------------*/
// function and kernel related to view scoring per face
__global__ void CountPixelPerFace(int* faceMap, int* counts, float* visibilityMap, float* scoreFace, Point2i imgSize) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= imgSize.x() || y >= imgSize.y())
		return;
	uint idx = y * imgSize.x() + x;
	float visibility = visibilityMap[idx];
	int faceID = faceMap[idx];
	if (faceID < 0)
		return;
	atomicAdd(&scoreFace[faceID], visibility);
	atomicAdd(&counts[faceID], 1);
}

__global__ void NormalizeScorePerFace(float* scoreFace, int* counts, uint numFaces) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if (x >= numFaces)
		return;
	if (counts[x] > 0)
		scoreFace[x] = scoreFace[x] / (float)counts[x];
	else
		scoreFace[x] = 0.f;
}
__global__ void UpdateBestViewsKernel(float* bestViewsScore, int* bestViewsIdx, float* scoreFace, uint* resFace, int* pixelsFace, uint numFaces, uint nViews, uint viewIdx) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if (x >= numFaces)
		return;

	float nPixels = pixelsFace[x];
	float fScore = scoreFace[x];
	if (fScore == 0)
		return;
	uint* res = &resFace[x*nViews];
	int* idx = &bestViewsIdx[x*nViews];
	float* score = &bestViewsScore[x*nViews];
	// iterate throught scores stack and replace keeping the scored ordered
	for (int i = 0; i < nViews; i++) {
		if (idx[i] == -1) {
			// empty slot
			score[i] = fScore;
			res[i] = nPixels;
			idx[i] = viewIdx;
			break;
		}
		if (fScore> score[i]) {
			for (int j = nViews - 1; j > i; j--) {
				score[j] = score[j - 1];
				res[j] = res[j - 1];
				idx[j] = idx[j - 1];
			}
			// insert
			score[i] = fScore;
			res[i] = nPixels;
			idx[i] = viewIdx;
			break;
		}
	}

}

// smart visibility average over horizontal direction with depth and angle constraints to limit sum to pixel on a same surface
__global__ void VisibilityAverageOverH(const float* in, float* out, const float* visibility, float* depthMap,
	Point2i imgSize, int radius, float meanD, float fx, bool divide = false) {
	int height = imgSize.y();
	int width = imgSize.x();

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;
	if (visibility[y * width + x] == 0.f) {
		out[y * width + x] = 0.f;
		return;
	}
	float depth0 = depthMap[y * width + x];
	if (depth0 == FLT_MAX || depth0 <= 0.f) {
		out[y * width + x] = 0.f;
		return;
	}

	// build a max depth difference by pixel to consider them belonging to the same surface
	float cosTheta = visibility[y * width + x];
	float tanTh = sqrt(1.0f - cosTheta * cosTheta) / cosTheta;
	float dTh = 1.*depth0*fabsf(tanTh) / fx;

	// widow size based on relative depth and image size
	int R = (int)ceilf((imgSize.x()/10.f) * (meanD/depth0));
	R = min(R, width - 1);

	// sum over horizontal direction
	float sum = 0.0f;
	int   count = 0;
	for (int k = -R; k <= R; ++k) {
		int sx = x + k;
		if (sx >= 0 && sx < width) {
			float depth = depthMap[y * width + sx];
			float cosThetaN = visibility[y * width + sx];
			if (depth == FLT_MAX || depth <= 0.f)
				continue;
			if (fabsf(depth0 - depth) > fabsf(k) * dTh) // threshold based on depth difference
				continue;
			if (fabsf(acos(cosThetaN)-acos(cosTheta)) > 0.04*3.14) // threshold based on angle 
				continue;

			if (divide) {
				depth = ceilf(ceilf(depth * meanD/30.) * 30 / meanD); // quantization of the depth to reduce noise
				sum += in[y * width + sx]/(depth);
			} else
				sum += in[y * width + sx];
			++count;
		}
	}
	out[y * width + x] = sum / (float)count;
}

// smart visibility average over vertical direction with depth and angle constraints to limit sum to pixel on a same surface
__global__ void VisibilityAverageOverV(const float* in, float* out, const float* visibility, float* depthMap,
	Point2i imgSize, int radius, float meanD, float fy, bool divide = false) {
	int height = imgSize.y();
	int width = imgSize.x();
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;
	if (visibility[y * width + x] == 0.f) {
		out[y * width + x] = 0.f;
		return;
	}
	float depth0 = depthMap[y * width + x];
	if (depth0 == FLT_MAX || depth0 <= 0.f) {
		out[y * width + x] = 0.f;
		return;
	}

	// build a max depth difference by pixel to consider them belonging to the same surface
	float cosTheta = visibility[y * width + x];
	float tanTh = sqrt(1.0f - cosTheta * cosTheta) / cosTheta;
	float limit = 1. * depth0 * fabsf(tanTh) / fy;

	// widow size based on relative depth and image size
	int R = (int)ceilf((imgSize.y()/10.f) * (meanD/depth0));
	R = min(R, height - 1);
	
	// sum over vertical direction
	float sum = 0.0f;
	int   count = 0;
	for (int k = -R; k <= R; ++k) {
		int sy = y + k;
		if (sy >= 0 && sy < height) {
			float depth = depthMap[sy * width + x];
			float cosThetaN = visibility[sy * width + x];
			if (depth == FLT_MAX || depth <= 0.f)
				continue;
			if (fabsf(depth0 - depth) > fabsf(k) * limit) // threshold based on depth difference
				continue;
			if (fabsf(acos(cosThetaN)-acos(cosTheta)) > 0.04*3.14) // threshold based on angle 
				continue;
			if (divide) {
				depth = ceilf(roundf(depth * meanD/30) * 30 / meanD); // quantization of the depth to reduce noise
				sum += in[sy * width + x]/(depth);
			} else
				sum += in[sy * width + x];
			++count;
		}
	}
	out[y * width + x] = sum / (float)count;
}

// conditional angle average over horizontal direction with depth and angle constraints, also aply quantization on angle
__global__ void AverageThetaH(const float* in, float* out, const float* visibility, float* depthMap,
	Point2i imgSize, float meanD) {
	int height = imgSize.y();
	int width = imgSize.x();
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;
	if (visibility[y * width + x] == 0.f) {
		out[y * width + x] = 0.f;
		return;
	}
	float depth0 = depthMap[y * width + x];
	if (depth0 == FLT_MAX || depth0 <= 0.f) {
		out[y * width + x] = 0.f;
		return;
	}
	// window size based on relative depth and image size
	int rx = (int)ceilf((imgSize.x()/200.f) * (meanD/depth0));
	float cosTheta = visibility[y * width + x];
	rx = min(rx, width - 1);
	// sum over horizontal direction
	float sum = 0.0f;
	int   count = 0;
	for (int l = -rx; l <= rx; ++l) {
		int sx = x + l;
		if (sx >= 0 && sx < width && y >=0 && y < height) {
			float depthN = depthMap[y * width + sx];
			if (depthN == FLT_MAX || depthN <= 0.f)
				continue;
			float cosThetaN = visibility[y * width + sx];
			float thThresh = 0.02 *(max(30.f-float(l), 0.f))/30.f + 0.04; // threshold lower for closer pixels
			if (fabsf(acos(cosThetaN)- acos(cosTheta)) > thThresh*3.14) // threshold based on angle 
				continue;
			float th = acos(in[y * width + sx]);
			th = roundf(th * 20.) / 20.; // reduce precision to avoid noise
			sum += cos(th);
			++count;
		}
	}
	out[y * width + x] = sum / (float)count;
}

// conditional angle average over vertical direction with depth and angle constraints, also aply quantization on angle
__global__ void AverageThetaV(const float* in, float* out, const float* visibility, float* depthMap,
	Point2i imgSize, float meanD) {
	int height = imgSize.y();
	int width = imgSize.x();
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;
	if (visibility[y * width + x] == 0.f) {
		out[y * width + x] = 0.f;
		return;
	}
	float depth0 = depthMap[y * width + x];
	if (depth0 == FLT_MAX || depth0 <= 0.f) {
		out[y * width + x] = 0.f;
		return;
	}
	// window size based on relative depth and image size
	int ry = (int)ceilf((imgSize.y()/200.f) * (meanD/depth0));
	float cosTheta = visibility[y * width + x];
	ry = min(ry, height - 1);
	// sum over vertical direction
	float sum = 0.0f;
	int   count = 0;
	for (int k = -ry; k <= ry; ++k) {
		int sy = y + k;
		if (sy >= 0 && sy < height && x >=0 && x < width) {
			float depthN = depthMap[sy * width + x];
			if (depthN == FLT_MAX || depthN <= 0.f)
				continue;
			float cosThetaN = visibility[sy * width + x];
			float thThresh = 0.02 *(max(30.f-float(k), 0.f))/30.f + 0.04; // threshold lower for closer pixels
			if (fabsf(acos(cosThetaN)- acos(cosTheta)) > thThresh*3.14) // threshold based on angle 
				continue;
			float th = acos(in[sy * width + x]);
			th = roundf(th * 20) / 20.; // reduce precision to avoid noise
			sum += cos(th);
			++count;
		}
	}
	out[y * width + x] = sum / (float)count;
}

// create edge map from depth map
__global__ void MakeEdgeMapKernel(float* depthMap, float* edgeMap, int imgW, int imgH, float dDepth) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	float depthThreshold = 0.006f*dDepth; // threshold proportional to depth range

	if (pixX >= imgW || pixY >= imgH) return;
	int pix = pixY * imgW + pixX;
	float centerDepth = depthMap[pix];
	if (centerDepth >= 1e10f) {
		edgeMap[pix] = 1.f; // mark as edge if no depth
		return;
	}
	bool isEdge = false;
	// check 4-neighbors
	int offsets[4][2] = {{1,0}, {-1,0}, {0,1}, {0,-1}};
	for (int i = 0; i < 4; i++) {
		int nx = pixX + offsets[i][0];
		int ny = pixY + offsets[i][1];
		if (nx < 0 || nx >= imgW || ny < 0 || ny >= imgH)
			continue;
		int nPix = ny * imgW + nx;
		float nDepth = depthMap[nPix];
		if (nDepth >= 1e10f) {
			isEdge = true;
			break;
		}
		if (fabsf(nDepth - centerDepth)> depthThreshold) {
			isEdge = true;
			break;
		}
	}
	edgeMap[pix] = isEdge ? 1.f : 0.f;
}

// does A = A * B
__global__ void MultiplyKernel(float* A, float* B, Point2i imgSize) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= imgSize.x() || y >= imgSize.y())
		return;
	uint idx = y * imgSize.x() + x;
	A[idx] = A[idx] * B[idx];
}

// compute visibility score per face and update best views per face
void UpdateBestViews(int* faceMap, float* visibilityMap, float* depthMap, float* bestViewsScore, uint* resFace, int* bestViewsIdx, const uint numFaces, const Point2i imgSize, uint nViews, uint viewIdx, float fx, float fy, cudaStream_t stream) {
	dim3 blockSize(16, 16);
	dim3 gridSize(
		(imgSize.x() + blockSize.x - 1) / blockSize.x,
		(imgSize.y() + blockSize.y - 1) / blockSize.y
	);

	// get min / max depth excluding FLT_MAX
	thrust::device_ptr<float> depthPtr(depthMap);
	using MinMax = thrust::pair<float, float>;
	auto result = thrust::transform_reduce(thrust::cuda::par.on(stream), depthPtr, depthPtr + imgSize.x()*imgSize.y(),
		[] __host__ __device__(float v) {
		return (v == FLT_MAX) ? MinMax{FLT_MAX, -FLT_MAX} : MinMax{v, v};
		},
		MinMax{FLT_MAX, -FLT_MAX},
		[] __host__ __device__(	MinMax a, MinMax b) {
		return MinMax{ fminf(a.first,  b.first), fmaxf(a.second, b.second) };
		});
	float minDepth = result.first;
	float maxDepth = result.second;
	float meanD = (maxDepth + minDepth)/2.f;
	if (meanD == 0.f)
		return;

	// compute visibility score map for the view
	// first we average the angle map 
	// then visibility is computed by averaging the visibility in large sliding windows
	// angle and depth are quantized to avoid best view to localy change due to little variations
	// the averaging is constrained to pixels that are likely to belong to the same surface based on depth and angle similarity
	// because of the large windows used, the averaging is approximated in multiple passes of horizontal and vertical averages (this is not exact because of the conditions)
	float* averageMap;
	float* averageTheta;
	float* tmp;
	cudaMalloc(&averageMap, sizeof(float) * imgSize.x() * imgSize.y());
	cudaMalloc(&averageTheta, sizeof(float)* imgSize.x()* imgSize.y());
	cudaMalloc(&tmp, sizeof(float)* imgSize.x()* imgSize.y());
	cudaMemset(tmp, 0, sizeof(float)* imgSize.x()* imgSize.y());
	cudaMemcpy(averageTheta, visibilityMap, sizeof(float)* imgSize.x()* imgSize.y(), cudaMemcpyDeviceToDevice);
	cudaMemcpy(averageMap, visibilityMap, sizeof(float)* imgSize.x()* imgSize.y(), cudaMemcpyDeviceToDevice);

	int rx = (int)ceilf(imgSize.x()/20.f);
	int ry = (int)ceilf(imgSize.y()/20.f);
	// conditional average on angle map
	for (int i = 0; i < 4; i++) {
	AverageThetaH<<<gridSize, blockSize, 0, stream>>>(averageTheta, tmp, visibilityMap, depthMap, imgSize, meanD);
	CUDA_CHECK_LAST_ERROR;
	AverageThetaV<<<gridSize, blockSize, 0, stream>>>(tmp, averageTheta, visibilityMap, depthMap, imgSize, meanD);
	CUDA_CHECK_LAST_ERROR;
	}

	// smart score average on visibility map 
	VisibilityAverageOverH<<<gridSize, blockSize, 0, stream>>>(averageMap, tmp, averageTheta, depthMap, imgSize, rx, meanD, fx, true);
	CUDA_CHECK_LAST_ERROR;
	for (int i = 0; i < 6; i++) {
		VisibilityAverageOverV<<<gridSize, blockSize, 0, stream>>>(tmp, averageMap, averageTheta, depthMap, imgSize, ry, meanD, fy);
		CUDA_CHECK_LAST_ERROR;
		VisibilityAverageOverH<<<gridSize, blockSize, 0, stream>>>(averageMap, tmp, averageTheta, depthMap, imgSize, rx, meanD, fx);
		CUDA_CHECK_LAST_ERROR; 
	}
	VisibilityAverageOverV<<<gridSize, blockSize, 0, stream>>>(tmp, averageMap, averageTheta, depthMap, imgSize, ry, meanD, fy);
	CUDA_CHECK_LAST_ERROR;

	cudaMemcpy(visibilityMap, averageMap, sizeof(float)* imgSize.x()* imgSize.y(), cudaMemcpyDeviceToDevice);
	cudaFree(averageMap);
	cudaFree(averageTheta);
	cudaFree(tmp);

	float* edgeMap;
	cudaMalloc((void**)&edgeMap, imgSize.x() * imgSize.y() * sizeof(float));
	cudaMemset(edgeMap, 0, imgSize.x() * imgSize.y() * sizeof(float));
	size_t imgW = imgSize.x();
	size_t imgH = imgSize.y();
	// make distance map to depth edges
	{// make edge map from depth map
		dim3 blockSize(16, 16);
		dim3 gridSize((imgW + blockSize.x - 1) / blockSize.x, (imgH + blockSize.y - 1) / blockSize.y);
		MakeEdgeMapKernel<<<gridSize, blockSize, 0, stream>>>(depthMap, edgeMap, imgW, imgH, maxDepth - minDepth);
		CUDA_CHECK_LAST_ERROR
	}
	{// make distance map using jump flooding
		Seed* seedMap1_d, * seedMap2_d;
		cudaMalloc((void**)&seedMap1_d, imgW * imgH * sizeof(Seed));
		cudaMalloc((void**)&seedMap2_d, imgW * imgH * sizeof(Seed));
		dim3 blockSize(16, 16);
		dim3 gridSize((imgW + blockSize.x - 1) / blockSize.x, (imgH + blockSize.y - 1) / blockSize.y);

		initSeeds<<<gridSize, blockSize, 0, stream>>>(edgeMap, seedMap1_d, imgW, imgH);
		CUDA_CHECK_LAST_ERROR;
		int maxJump = max(imgW, imgH);
		for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
			jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, imgW, imgH, jump);
			CUDA_CHECK_LAST_ERROR
				// Swap seed maps
				Seed* temp = seedMap1_d;
			seedMap1_d = seedMap2_d;
			seedMap2_d = temp;
		}
		computeDistanceCustom<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, edgeMap, imgW, imgH);
		cudaFree(seedMap1_d);
		cudaFree(seedMap2_d);
		CUDA_CHECK_LAST_ERROR;

		// multiply pixel wise visibility with distance to edge
		MultiplyKernel<<<gridSize, blockSize, 0, stream>>>(visibilityMap, edgeMap, imgSize);
		CUDA_CHECK_LAST_ERROR;
	}
	cudaFree(edgeMap);
	
	float* vScorePerFace;
	cudaMalloc(&vScorePerFace, sizeof(float) * numFaces);
	cudaMemset(vScorePerFace, 0, sizeof(float) * numFaces);
	int* countPixByFace;
	cudaMalloc(&countPixByFace, sizeof(int)* numFaces);
	cudaMemset(countPixByFace, 0, sizeof(int)* numFaces);
	// compute resolution score and visibility score for each face
	// the resolution score is the number of pixels that the face project into the image
	// the visibility score is the average of the visibility score on the pixels covered by the face
	CountPixelPerFace<<<gridSize, blockSize, 0, stream>>>(faceMap, countPixByFace, visibilityMap, vScorePerFace, imgSize);
	CUDA_CHECK_LAST_ERROR;
	auto [gridSize2, blockSize2] = DefaultBlockCount(numFaces);
	NormalizeScorePerFace<<<gridSize2, blockSize2, 0, stream>>>(vScorePerFace, countPixByFace, numFaces);
	CUDA_CHECK_LAST_ERROR;
	// update best views per face vector keeping the order by score
	UpdateBestViewsKernel<<<gridSize2, blockSize2, 0, stream>>>(bestViewsScore, bestViewsIdx, vScorePerFace, resFace, countPixByFace, numFaces, nViews, viewIdx);
	CUDA_CHECK_LAST_ERROR;
	cudaFree(vScorePerFace);
	cudaFree(countPixByFace);
}


void UpdateBestViewsOrtho(int* faceMap, float* visibilityMap, float* depthMap, float* bestViewsScore, uint* resFace, int* bestViewsIdx, const uint numFaces, const Point2i imgSize, uint nViews, uint viewIdx, float fx, float fy, cudaStream_t stream) {
	dim3 blockSize(16, 16);
	dim3 gridSize(
		(imgSize.x() + blockSize.x - 1) / blockSize.x,
		(imgSize.y() + blockSize.y - 1) / blockSize.y
	);

	// get min / max depth excluding FLT_MAX
	thrust::device_ptr<float> depthPtr(depthMap);
	using MinMax = thrust::pair<float, float>;
	auto result = thrust::transform_reduce(thrust::cuda::par.on(stream), depthPtr, depthPtr + imgSize.x()*imgSize.y(),
		[] __host__ __device__(float v) {
		return (v == FLT_MAX) ? MinMax{FLT_MAX, -FLT_MAX} : MinMax{v, v};
		},
		MinMax{FLT_MAX, -FLT_MAX},
		[] __host__ __device__(	MinMax a, MinMax b) {
		return MinMax{ fminf(a.first,  b.first), fmaxf(a.second, b.second) };
		});
	float minDepth = result.first;
	float maxDepth = result.second;
	float meanD = (maxDepth + minDepth)/2.f;
	if (meanD == 0.f)
		return;

	// compute visibility score map for the view from normal map
	float* edgeMap;
	cudaMalloc((void**)&edgeMap, imgSize.x() * imgSize.y() * sizeof(float));
	cudaMemset(edgeMap, 0, imgSize.x() * imgSize.y() * sizeof(float));
	size_t imgW = imgSize.x();
	size_t imgH = imgSize.y();
	// make distance map to depth edges
	{// make edge map from depth map
		dim3 blockSize(16, 16);
		dim3 gridSize((imgW + blockSize.x - 1) / blockSize.x, (imgH + blockSize.y - 1) / blockSize.y);
		MakeEdgeMapKernel<<<gridSize, blockSize, 0, stream>>>(depthMap, edgeMap, imgW, imgH, maxDepth - minDepth);
		CUDA_CHECK_LAST_ERROR
	}
	{// make distance map using jump flooding
		Seed* seedMap1_d, * seedMap2_d;
		cudaMalloc((void**)&seedMap1_d, imgW * imgH * sizeof(Seed));
		cudaMalloc((void**)&seedMap2_d, imgW * imgH * sizeof(Seed));
		dim3 blockSize(16, 16);
		dim3 gridSize((imgW + blockSize.x - 1) / blockSize.x, (imgH + blockSize.y - 1) / blockSize.y);

		initSeeds<<<gridSize, blockSize, 0, stream>>>(edgeMap, seedMap1_d, imgW, imgH);
		CUDA_CHECK_LAST_ERROR;
		int maxJump = max(imgW, imgH);
		for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
			jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, imgW, imgH, jump);
			CUDA_CHECK_LAST_ERROR
				// Swap seed maps
				Seed* temp = seedMap1_d;
			seedMap1_d = seedMap2_d;
			seedMap2_d = temp;
		}
		computeDistanceCustom<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, edgeMap, imgW, imgH);
		cudaFree(seedMap1_d);
		cudaFree(seedMap2_d);
		CUDA_CHECK_LAST_ERROR;

	}
	// bake distance-to-edge into the visibility map (matches UpdateBestViews):
	// faces near depth edges should score lower to penalize discontinuous regions
	MultiplyKernel<<<gridSize, blockSize, 0, stream>>>(visibilityMap, edgeMap, imgSize);
	CUDA_CHECK_LAST_ERROR;
	cudaFree(edgeMap);
	float* vScorePerFace;
	cudaMalloc(&vScorePerFace, sizeof(float) * numFaces);
	cudaMemset(vScorePerFace, 0, sizeof(float) * numFaces);
	int* countPixByFace;
	cudaMalloc(&countPixByFace, sizeof(int)* numFaces);
	cudaMemset(countPixByFace, 0, sizeof(int)* numFaces);
	// compute resolution score and visibility score for each face
	// the resolution score is the number of pixels that the face project into the image
	// the visibility score is the average of the visibility score on the pixels covered by the face
	CountPixelPerFace<<<gridSize, blockSize, 0, stream>>>(faceMap, countPixByFace, visibilityMap, vScorePerFace, imgSize);
	CUDA_CHECK_LAST_ERROR;
	auto [gridSize2, blockSize2] = DefaultBlockCount(numFaces);
	NormalizeScorePerFace<<<gridSize2, blockSize2, 0, stream>>>(vScorePerFace, countPixByFace, numFaces);
	CUDA_CHECK_LAST_ERROR;
	// update best views per face vector keeping the order by score
	UpdateBestViewsKernel<<<gridSize2, blockSize2, 0, stream>>>(bestViewsScore, bestViewsIdx, vScorePerFace, resFace, countPixByFace, numFaces, nViews, viewIdx);
	CUDA_CHECK_LAST_ERROR;
	cudaFree(vScorePerFace);
	cudaFree(countPixByFace);
}

/*----------------------------------------------------------------*/

// functions and kernel related to texture stacks rasterization
__device__ int GetNextEmpty(const float* scores, const uint stackSize) {
	for (int i = 0; i < stackSize; i++) {
		if (scores[i] <= 0.f) {
			return i;
		}
	}
	return -1;
}

__device__ void MeanAndMoment2(const float* pixStack, uint stackSize, Point3& mean, Point3& square, uint& count) {
	for (uint i = 0; i < stackSize; i++) {
		mean.x() += pixStack[i*3];
		mean.y() += pixStack[i*3+1];
		mean.z() += pixStack[i*3+2];
		square.x() += pixStack[i*3] * pixStack[i*3];
		square.y() += pixStack[i*3+1] * pixStack[i*3+1];
		square.z() += pixStack[i*3+2] * pixStack[i*3+2];
		count++;
	}
	if (count == 0) return;
	mean /= (float)count;
	square /= (float)count;
}

// find the index of the color in the stack that if replaced by newPix would minimize the variance/diversity of the stack
// the improvement must be above threshold to be considered, that improves medium scale consistency insteed of optimizing visibility score
__device__ uint GetReplaceIdx(const float* __restrict__ pixStack, const uint stackSize, const Point3 newPix, const float threshold) {
	int replaceIdx = -1;
	Point3 mean(0.f, 0.f, 0.f);
	Point3 moment2(0.f, 0.f, 0.f);
	uint count = 0;
	MeanAndMoment2(pixStack, stackSize, mean, moment2, count);
	// we use Lab space L scale is not the same than ab, it should be taken into account in the div computation
	// but we also ajust the weights to give more importance to tonality consistency over luminance consistency
	Point3 inititalDiv = Point3(sqrtf((moment2.x() - mean.x() * mean.x())*1/100.f/1.f),
						sqrtf((moment2.y() - mean.y() * mean.y())*1/255.f/1.f),
						sqrtf((moment2.z() - mean.z() * mean.z())*1/255.f/1.f)
					   );
	Point3 div = inititalDiv;
	float minDiv = inititalDiv.norm();
	// adding new color
	mean = (mean * (float)count + newPix) / (float)(count + 1);
	moment2 = (moment2 * (float)count + Point3(newPix.x()*newPix.x(), newPix.y()*newPix.y(), newPix.z()*newPix.z())) / (float)(count + 1);

	for (int s = 0; s < stackSize; ++s) {
		// simulate replacing color at s
		Point3 repPix(pixStack[s * 3], pixStack[s * 3 + 1], pixStack[s * 3 + 2]);
		Point3 repMean = (mean * (float)(count + 1) - repPix) / (float)count;
		Point3 repMoment2 = (moment2 * (float)(count + 1) - Point3(repPix.x()*repPix.x(), repPix.y()*repPix.y(), repPix.z()*repPix.z())) / (float)count;
		Point3 repDiv= Point3(sqrtf((repMoment2.x() - repMean.x() * repMean.x())*1/100.f/1.f),
						sqrtf((repMoment2.y() - repMean.y() * repMean.y())*1/255.f/1.f),
						sqrtf((repMoment2.z() - repMean.z() * repMean.z())*1/255.f/1.f)
					   );
		float d = repDiv.norm();
		if (d < minDiv) {
			minDiv = d;
			replaceIdx = s;
			div = repDiv;
		}
	}
	Point3 vFinalDiv = inititalDiv - div;
	if (vFinalDiv.x() <= threshold && vFinalDiv.y() <= threshold && vFinalDiv.z() <= threshold)
		replaceIdx = -1;
	return replaceIdx;
}
__global__ void TextureRasterizeKernel(float** __restrict__ texStacks, const uint stackSize, float* __restrict__ texAverage, float* __restrict__ bufferCount, const Point2i texSize, float* __restrict__ visibilityScores,
	const float** __restrict__ pyramidImage, const Point2i* imgSize, const uint levels, const float* __restrict__ facesScore, const int* __restrict__ isBestView, int* __restrict__ idxStacks, 
	const Point3* __restrict__ positions, const Point2* __restrict__ texCoords, const uint numViewFaces,
	const CUDA::Camera camera, uint viewIdx) {
	if (blockIdx.x >= numViewFaces) return;
	const uint maxLevels = 10;
	uint triId = blockIdx.x; 
	int isBest = isBestView[triId];
	float score = facesScore[triId];

	if (score <= 0.f) return;
	Point3 pos[3];
	Point2 uvs[3];
	for (uint i = 0; i < 3; i++) {
		uint v = triId * 3 + i;
		pos[i] = positions[v];
		uvs[i] = texCoords[v];
	}
	// compute normal 
	Point3 N = (pos[1] - pos[0]).cross(pos[2] - pos[0]);
	N = N / N.norm();
	// triangle bounding box in texel space
	float minX = floorf(fminf(uvs[0].x(), fminf(uvs[1].x(), uvs[2].x())));
	float minY = floorf(fminf(uvs[0].y(), fminf(uvs[1].y(), uvs[2].y())));
	float maxX = ceilf(fmaxf(uvs[0].x(), fmaxf(uvs[1].x(), uvs[2].x())));
	float maxY = ceilf(fmaxf(uvs[0].y(), fmaxf(uvs[1].y(), uvs[2].y())));
	// clamp to texture size
	minX = max(0, (int)minX);
	minY = max(0, (int)minY);
	maxX = min(texSize.x()-1, (int)maxX);
	maxY = min(texSize.y()-1, (int)maxY);
	float area = edgeFn(uvs);
	if (fabsf(area)< 1e-12f) return;
	int width = maxX - minX + 1;
	int height = maxY - minY + 1;
	for (int i = threadIdx.x; i < width*height; i += blockDim.x) {
		int x = i % width + minX;
		int y = i / width + minY;
		Point2 P = {x , y};
		// barycentric coordinates
		float w0 = edgeFn(uvs[1], uvs[2], P);
		float w1 = edgeFn(uvs[2], uvs[0], P);
		float w2 = edgeFn(uvs[0], uvs[1], P);
		// if P is inside the triangle
		if (!((w0 >= 0 && w1 >= 0 && w2 >= 0 && area > 0) ||
			(w0 <= 0 && w1 <= 0 && w2 <= 0 && area < 0))) {
			continue;
		}
		float invArea = 1.0f / area;
		w0 *= invArea;
		w1 *= invArea;
		w2 *= invArea;
		// interpolate 3D position in world space
		Point3 pW = {
			pos[0].x()* w0 + pos[1].x() * w1 + pos[2].x() * w2,
			pos[0].y()* w0 + pos[1].y() * w1 + pos[2].y() * w2,
			pos[0].z()* w0 + pos[1].z() * w1 + pos[2].z() * w2
		};
		Point2 uv[maxLevels];
		Point3 pCam = camera.R*(pW-camera.C);
		float cosTheta = fabsf(N.dot(pCam)) / pCam.norm();
		float z = pCam.z();

		// project to each pyramid stage
		for (int l = 0; l < levels; l++) {
			Matrix3 K = camera.K;
			float ratio = (float)imgSize[l].x() /(float)imgSize[0].x();

			K(0, 0) = K(0, 0) * ratio;
			K(1, 1) = K(1, 1) * ratio;
			K(0, 2) = K(0, 2) * ratio;
			K(1, 2) = K(1, 2) * ratio;
			Point3  pImg = K * pCam;
			uv[l] = {
				pImg.x() / (pImg.z()*(imgSize[l].x()-1)),
				pImg.y() / (pImg.z()*(imgSize[l].y()-1))
			};
		}
		uint refLevel = levels - 1; // reference level for the outlier detection is the color level (lowerest resolution)
		float u = uv[refLevel].x(); // color base u
		float v = uv[refLevel].y(); // color base v
		if (u < 0.f || u > 1.f || v < 0.f || v > 1.f) continue;

		Point2i baseSize = imgSize[refLevel]; // base size
		int pix = y * texSize.x() + x;
		int baseIdx = (int)floorf(u * (baseSize.x()-1)) + (int)floorf(v*(baseSize.y()-1)) * baseSize.x(); // color base index
		const float* __restrict__ colorBase = pyramidImage[refLevel];
		Point3 C = samplePhotoBilinear3C(colorBase, imgSize[refLevel], u, v); // base color pixel
		int scoreIdx = pix * stackSize;	// index for the score stack
		int texIdx = pix * 3 *stackSize; // index for the texture stack
		int nextIdx = -1;
		float* texStackColor = &texStacks[refLevel][texIdx];
		if (isBest == 1) { // sample can integrate the stack only if the view is one of the best for the face
			// try to find an empty slot first
			nextIdx = GetNextEmpty(&visibilityScores[scoreIdx], stackSize);
			if (nextIdx < 0) { // if no empty slot, find best to replace based on variance reduction
				nextIdx = GetReplaceIdx(texStackColor, stackSize, C, 0.1f);
			} 
		}

		// update the average of the color background
		Point3 dist = Point3(C.x() - texAverage[pix * 3 + 0], C.y() - texAverage[pix * 3 + 1], C.z() - texAverage[pix * 3 + 2]);
		float tmpScore = bufferCount[pix];
		float d = 0.3; // how much to reduce the contribution of the color not included in the stack or removed from it
		texAverage[pix * 3 + 0] = (texAverage[pix * 3 + 0]*tmpScore + C.x()*score)/(tmpScore + score);
		texAverage[pix * 3 + 1] = (texAverage[pix * 3 + 1]*tmpScore + C.y()*score)/(tmpScore + score);
		texAverage[pix * 3 + 2] = (texAverage[pix * 3 + 2]*tmpScore + C.z()*score)/(tmpScore + score);
		bufferCount[pix] += score;
		if (nextIdx == -1) {
			Point3 toRemove = C;
			tmpScore += score;
			float scrToRmv = score*d;
			texAverage[pix * 3 + 0] = (texAverage[pix * 3 + 0]*tmpScore - toRemove.x()*scrToRmv)/(tmpScore - scrToRmv);
			texAverage[pix * 3 + 1] = (texAverage[pix * 3 + 1]*tmpScore - toRemove.y()*scrToRmv)/(tmpScore - scrToRmv);
			texAverage[pix * 3 + 2] = (texAverage[pix * 3 + 2]*tmpScore - toRemove.z()*scrToRmv)/(tmpScore - scrToRmv);
			bufferCount[pix] -= scrToRmv;
			continue;
		} else if (nextIdx != -1 && visibilityScores[scoreIdx + nextIdx] != 0) { // remove half of the contribution of the replaced color
			Point3 toRemove = Point3(texStackColor[nextIdx * 3 + 0], texStackColor[nextIdx * 3 + 1], texStackColor[nextIdx * 3 + 2]);
			tmpScore += score;
			float scrToRmv = visibilityScores[scoreIdx + nextIdx]*d;
			texAverage[pix * 3 + 0] = (texAverage[pix * 3 + 0]*tmpScore - toRemove.x()*scrToRmv)/(tmpScore - scrToRmv);
			texAverage[pix * 3 + 1] = (texAverage[pix * 3 + 1]*tmpScore - toRemove.y()*scrToRmv)/(tmpScore - scrToRmv);
			texAverage[pix * 3 + 2] = (texAverage[pix * 3 + 2]*tmpScore - toRemove.z()*scrToRmv)/(tmpScore - scrToRmv);
			bufferCount[pix] -= scrToRmv;
		}
		// insert the new sample in to the stack of each pyramid stage
		texStackColor[nextIdx * 3 + 0] = C.x();
		texStackColor[nextIdx * 3 + 1] = C.y();
		texStackColor[nextIdx * 3 + 2] = C.z();
		for (int l = 0; l < levels-1; l++) {
			if (l == refLevel) continue;
			float* __restrict__ texStack = texStacks[l];
			Point3 pixel = samplePhotoBilinear3C(pyramidImage[l], imgSize[l], uv[l].x(), uv[l].y());
			texStack[texIdx+nextIdx*3+0] = pixel.x();
			texStack[texIdx+nextIdx*3+1] = pixel.y();
			texStack[texIdx+nextIdx*3+2] = pixel.z();
		}
		visibilityScores[scoreIdx+nextIdx] = (float)score;
		idxStacks[scoreIdx+nextIdx] = viewIdx;
	}
}

void TextureRasterize(std::vector<CUDA::TDeviceMat<float>>& texStacks, float* texAverage, float* bufferCount, const uint stackSize, float* visibilityScores, const float* faceScores, 
	const int* isBestView, const std::vector<CUDA::TDeviceMat<float>>& pyramidImage, int* idxStacks, const Point3* positions, const Point2* texCoords, 
	const uint numViewFaces, const uint nFace, const CUDA::Camera camera, uint viewIdx, cudaStream_t stream) {
	uint blockSize = 64;
	uint gridSize = numViewFaces;
	Point2i texSize = texStacks[0].CudaImageSize();
	uint levels = pyramidImage.size();
	// prepare ptr arrays
	const float** d_pyramidImgPtr;
	float** d_texStacksPtr;
	Point2i* d_imgSizes;
	cudaMalloc(&d_imgSizes, sizeof(Point2i) * levels);
	cudaMalloc(&d_pyramidImgPtr, sizeof(float*) * levels);
	cudaMalloc(&d_texStacksPtr, sizeof(float*) * levels);
	std::vector<const float*> h_pyramidImgPtr;
	std::vector<float*> h_texStacksPtr;
	std::vector<Point2i> h_imgSizes;
	for (int i = 0; i<levels; i++) {
		h_imgSizes.push_back(pyramidImage[i].CudaImageSize());
		h_pyramidImgPtr.push_back(pyramidImage[i].GetDeviceData());
		h_texStacksPtr.push_back(texStacks[i].GetDeviceData());
	}
	cudaMemcpy(d_imgSizes, h_imgSizes.data(), sizeof(Point2i) * levels, cudaMemcpyHostToDevice);
	cudaMemcpy(d_pyramidImgPtr, h_pyramidImgPtr.data(), sizeof(float*) * levels, cudaMemcpyHostToDevice);
	cudaMemcpy(d_texStacksPtr, h_texStacksPtr.data(), sizeof(float*) * levels, cudaMemcpyHostToDevice);

	TextureRasterizeKernel<<<gridSize, blockSize, 0, stream>>>(d_texStacksPtr, stackSize, texAverage, bufferCount, texSize, visibilityScores,
		d_pyramidImgPtr, d_imgSizes, levels, faceScores, isBestView, idxStacks, positions, texCoords, numViewFaces, camera, viewIdx);
	CUDA_CHECK_LAST_ERROR;
	cudaFree(d_imgSizes);
	cudaFree(d_pyramidImgPtr);
	cudaFree(d_texStacksPtr);
}

// fills per-pixel stacks for one view of a tile
__global__ void OrthoRasterizeKernelBis(float** __restrict__ tileStacks, const uint stackSize, const Point2i tileRes, float* __restrict__ visibilityScores, const float** __restrict__ pyramidImage, const Point2i* imgSize,
	const uint levels, const float* __restrict__ facesScore, const Point3* __restrict__ positions, const uint numViewFaces, const CUDA::Camera camera, const uint viewIdx, const Point2 tileOrigin, const Point2 tileSize, float* __restrict__ depthMap) {
	if (blockIdx.x >= numViewFaces) return;
	const uint maxLevels = 10;
	uint triId = blockIdx.x;
	float score = facesScore[triId];
	if (score <= 0.f) return;
	Point3 pos[3];
	Point2 tileCoord[3];
	for (uint i = 0; i < 3; i++) {
		uint v = triId * 3 + i;
		pos[i] = positions[v];
		tileCoord[i] = worldToTilePixel(pos[i], tileOrigin, tileSize, tileRes);
	}
	// triangle bounding box in tile space
	float minX = floorf(fminf(tileCoord[0].x(), fminf(tileCoord[1].x(), tileCoord[2].x())));
	float minY = floorf(fminf(tileCoord[0].y(), fminf(tileCoord[1].y(), tileCoord[2].y())));
	float maxX = ceilf(fmaxf(tileCoord[0].x(), fmaxf(tileCoord[1].x(),	tileCoord[2].x())));
	float maxY = ceilf(fmaxf(tileCoord[0].y(), fmaxf(tileCoord[1].y(), tileCoord[2].y())));
	// clamp to texture size
	minX = max(0, (int)minX);
	minY = max(0, (int)minY);
	maxX = min(tileRes.x()-1, (int)maxX);
	maxY = min(tileRes.y()-1, (int)maxY);
	float area = edgeFn(tileCoord);

	if (fabsf(area)< 1e-12f && fabsf(area) > -1e-12f) return;
	int width = maxX - minX + 1;
	int height = maxY - minY + 1;
	for (int i = threadIdx.x; i < width*height; i += blockDim.x) {
		int x = i % width + minX;
		int y = i / width + minY;
		Point2 P = {x , y};
		// barycentric coordinates
		float w0 = edgeFn(tileCoord[1], tileCoord[2], P);
		float w1 = edgeFn(tileCoord[2], tileCoord[0], P);
		float w2 = edgeFn(tileCoord[0], tileCoord[1], P);
				// if P is inside the triangle
		if (!((w0 >= 0 && w1 >= 0 && w2 >= 0 && area > 0) ||
			(w0 <= 0 && w1 <= 0 && w2 <= 0 && area < 0))) {
			continue;
		}

		float invArea = 1.0f / area;
		w0 *= invArea;
		w1 *= invArea;
		w2 *= invArea;
		// interpolate 3D position in world space
		Point3 pW = {
			pos[0].x()* w0 + pos[1].x() * w1 + pos[2].x() * w2,
			pos[0].y()* w0 + pos[1].y() * w1 + pos[2].y() * w2,
			pos[0].z()* w0 + pos[1].z() * w1 + pos[2].z() * w2
		};
		Point2 uv[maxLevels]; // position on every stage of the image pyramid
		Point3 pCam = camera.R*(pW-camera.C);
		float z = pCam.z();

		// project to each pyramid stage
		for (int l = 0; l < levels; l++) {
			Matrix3 K = camera.K;
			float ratio = (float)imgSize[l].x() /(float)imgSize[0].x();
			K(0, 0) = K(0, 0) * ratio;
			K(1, 1) = K(1, 1) * ratio;
			K(0, 2) = K(0, 2) * ratio;
			K(1, 2) = K(1, 2) * ratio;
			Point3  pImg = K * pCam;
			uv[l] = {
				pImg.x() / (pImg.z()*(imgSize[l].x()-1)),
				pImg.y() / (pImg.z()*(imgSize[l].y()-1))
			};
		}
		uint refLevel = levels - 1; // reference level for the outlier detection is the color level (lowerest resolution)
		float u = uv[refLevel].x(); // color base u
		float v = uv[refLevel].y(); // color base v
		if (u < 0.f || u > 1.f || v < 0.f || v > 1.f) continue;

		Point2i baseSize = imgSize[refLevel]; // base size
		int pix = y * tileRes.x() + x;
		// per-view depth test: keep only the topmost (largest world-Z) surface so far for this tile pixel,
		// matching the orthographic top-down convention used by ProjectNormalOrthoMap.
		// atomicMaxFloat ensures the depth buffer monotonically rises and avoids the worst-case where a
		// far triangle silently overwrites a near one; same best-effort pattern as ProjectNormalOrthoMap
		// (a contender that already lost the depth race skips its sample writes entirely)
		const float oldDepth = atomicMaxFloat(&depthMap[pix], (float)pW.z());
		if ((float)pW.z() <= oldDepth) continue;
		int baseIdx = (int)floorf(u * (baseSize.x()-1)) + (int)floorf(v*(baseSize.y()-1)) * baseSize.x(); // color base index
		const float* __restrict__ colorBase = pyramidImage[refLevel];
		Point3 C = samplePhotoBilinear3C(colorBase, imgSize[refLevel], u, v); // base color pixel
		int scoreIdx = pix * stackSize;	// index for the score stack
		int texIdx = pix * 3 *stackSize; // index for the texture stack
		int nextIdx = viewIdx;
		float* texStackColor = &tileStacks[refLevel][texIdx];
		
		// insert the new sample in to the stack of each pyramid stage
		texStackColor[nextIdx * 3 + 0] = C.x();
		texStackColor[nextIdx * 3 + 1] = C.y();
		texStackColor[nextIdx * 3 + 2] = C.z();
	
		for (int l = 0; l < levels-1; l++) {
			if (l == refLevel) continue;
			float* __restrict__ texStack = tileStacks[l];
			Point3 pixel = samplePhotoBilinear3C(pyramidImage[l], imgSize[l], uv[l].x(), uv[l].y());
			texStack[texIdx+nextIdx*3+0] = pixel.x();
			texStack[texIdx+nextIdx*3+1] = pixel.y();
			texStack[texIdx+nextIdx*3+2] = pixel.z();
		}
		visibilityScores[scoreIdx+nextIdx] = (float)score;
	}
}

void OrthoRasterize(std::vector<CUDA::TDeviceMat<float>>& texStacks, const uint stackSize, float* visibilityScores, const float* faceScores, 
	const std::vector<CUDA::TDeviceMat<float>>& pyramidImage, const Point3* positions,  const uint numViewFaces, 
	const uint numTriangles, const CUDA::Camera camera, uint viewIdx, Point2 tileOrigin, Point2 tileSize, cudaStream_t stream) {
	uint blockSize = 64;
	uint gridSize = numViewFaces;
	Point2i texSize = texStacks[0].CudaImageSize();
	uint levels = pyramidImage.size();

	// stream-ordered pointer-table allocations; H2D copies stay synchronous from the host (so the host
	// vectors below can be safely destroyed at scope exit), but the frees are queued on the stream
	// so the device side does not require an explicit cudaDeviceSynchronize.
	const float** d_pyramidImgPtr;
	float** d_texStacksPtr;
	Point2i* d_imgSizes;
	cudaMallocAsync(&d_imgSizes, sizeof(Point2i) * levels, stream);
	cudaMallocAsync(&d_pyramidImgPtr, sizeof(float*) * levels, stream);
	cudaMallocAsync(&d_texStacksPtr, sizeof(float*) * levels, stream);
	std::vector<const float*> h_pyramidImgPtr;
	std::vector<float*> h_texStacksPtr;
	std::vector<Point2i> h_imgSizes;
	for (int i = 0; i<levels; i++) {
		Point2i imgSize = pyramidImage[i].CudaImageSize();
		h_imgSizes.push_back(pyramidImage[i].CudaImageSize());
		h_pyramidImgPtr.push_back(pyramidImage[i].GetDeviceData());
		h_texStacksPtr.push_back(texStacks[i].GetDeviceData());
	}
	cudaMemcpy(d_imgSizes, h_imgSizes.data(), sizeof(Point2i) * levels, cudaMemcpyHostToDevice);
	cudaMemcpy(d_pyramidImgPtr, h_pyramidImgPtr.data(), sizeof(float*) * levels, cudaMemcpyHostToDevice);
	cudaMemcpy(d_texStacksPtr, h_texStacksPtr.data(), sizeof(float*) * levels, cudaMemcpyHostToDevice);
	CUDA_CHECK_LAST_ERROR;

	// per-tile depth buffer used by the kernel to keep only the topmost surface contribution per pixel
	// (one float per pixel of the tile; fresh per OrthoRasterize call so it's view-local)
	float* d_depthMap;
	const size_t numTilePixels = (size_t)texSize.x() * texSize.y();
	cudaMallocAsync(&d_depthMap, numTilePixels * sizeof(float), stream);
	thrust::fill(thrust::cuda::par.on(stream), thrust::device_ptr<float>(d_depthMap), thrust::device_ptr<float>(d_depthMap) + numTilePixels, -FLT_MAX);
	CUDA_CHECK_LAST_ERROR;

	OrthoRasterizeKernelBis<<<gridSize, blockSize, 0, stream>>>(d_texStacksPtr, stackSize, texSize, visibilityScores,
	d_pyramidImgPtr, d_imgSizes, levels, faceScores, positions, numViewFaces, camera, viewIdx, tileOrigin, tileSize, d_depthMap);
	CUDA_CHECK_LAST_ERROR;
	cudaFreeAsync(d_imgSizes, stream);
	cudaFreeAsync(d_pyramidImgPtr, stream);
	cudaFreeAsync(d_texStacksPtr, stream);
	cudaFreeAsync(d_depthMap, stream);
}

/*----------------------------------------------------------------*/
// kernels and function relative to pyramid recombination 
// recombine a lower-frequency pyramid level (texIn, slot 0) into the next finer level (texOut, slot 0).
// if blur > 0, texIn is spatially blurred before adding
__global__ void CombineKernel(float* __restrict__ texIn, float* __restrict__ texOut, Point2i texSize, uint stackSize, int blur = 0, float factor = 1.f) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= texSize.x()|| pixY >= texSize.x()) return;

	int idx = (pixY * texSize.y() + pixX) * 3 * stackSize;
	int countIdx = (pixY * texSize.y() + pixX) * stackSize;
	int kernelSum = 0;
	Point3 blurred = {0.f, 0.f, 0.f};
	if (blur>0) {
		int kernelRadius;
		if (blur == 1)
			kernelRadius = 1;
		else if (blur == 2)
			kernelRadius = 2;
		else if (blur == 3)
			kernelRadius = 4;
		else if (blur == 4)
			kernelRadius = 8;

		for (int ky = -kernelRadius; ky <= kernelRadius; ky++) {
			for (int kx = -kernelRadius; kx <= kernelRadius; kx++) {
				if (pixX + kx < 0 || pixX + kx >= texSize.x() || pixY + ky < 0 || pixY + ky >= texSize.y())
					continue; // skip out of bound
				int tmpIdx = ((pixY + ky) * texSize.x() + pixX + kx) * 3 * stackSize;
				Point3 sample = Point3(texIn[tmpIdx], texIn[tmpIdx+1], texIn[tmpIdx+2]);
				if (sample.x() == 0.f && sample.y() == 0.f && sample.z() == 0.f)
					continue; // skip empty pixel
				int kIdx = (ky + kernelRadius) * (2 * kernelRadius + 1) + (kx + kernelRadius);
				int kval;
				if (blur == 1)
					kval = kernel9[kIdx];
				else if (blur == 2)
					kval = kernel25[kIdx];
				else if (blur == 3)
					kval = kernel81[kIdx];
				else if (blur == 4)
					kval = kernel289[kIdx];
				kernelSum += kval;
				blurred.x() += sample.x() * kval;
				blurred.y() += sample.y() * kval;
				blurred.z() += sample.z() * kval;
			}
		}
		if (kernelSum == 0)
			return;
		blurred.x() /= (float)kernelSum;
		blurred.y() /= (float)kernelSum;
		blurred.z() /= (float)kernelSum;
		texOut[idx] = blurred.x() + factor*texOut[idx];
		texOut[idx + 1] = blurred.y() + factor*texOut[idx + 1];
		texOut[idx + 2] = blurred.z() + factor*texOut[idx + 2];
	} else {
		texOut[idx] = texIn[idx] + factor*texOut[idx];
		texOut[idx + 1] = texIn[idx+1] + factor*texOut[idx + 1];
		texOut[idx + 2] = texIn[idx+2] + factor*texOut[idx + 2];
	}
}

// apply an S-curve contrast correction to the CIE L* channel of the color stored in slot,
// the curve boosts mid-tone contrast while compressing near-white and near-black regions,
// contrastFactor controls the strength of the effect (0 = identity)
__global__ void CorrectContrastKernel(float* texStack, Point2i texSize, uint stackSize, float contrastFactor) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= texSize.x() || pixY >= texSize.y()) return;
	int idx = (pixY * texSize.y() + pixX) * 3 * stackSize;
	float value = texStack[idx + 0];
	if (value == 0.f)
		return;
	value = (value - 50.f)/100.f;
	if (value >= .5f)
		value = 0.5f;
	else if (value <= -0.5f)
		value = -0.5f;
	if (value >= 0)
		value = 2*((1-pow(0.5f - value, 1+contrastFactor*value)) - 0.5);
	else
		value = 2*(pow(value  + 0.5f, 1-contrastFactor*value)-0.5);
	value = 50.f + value * 50.f;
	texStack[idx + 0] = value;
}

// multiplicative saturation boost: scales CIE a* and b* channels by saturationFactor
__global__ void CorrectSaturationKernel(float* texStack, Point2i texSize, uint stackSize, float saturationFactor) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= texSize.x() || pixY >= texSize.y()) return;
	int idx = (pixY * texSize.y() + pixX) * 3 * stackSize;
	texStack[idx + 1] = saturationFactor * texStack[idx + 1];
	texStack[idx + 2] = saturationFactor * texStack[idx + 2];
}

// recombine pyramid levels into a single texture
// apply details enhancement by adding a stronger weight to high frequency levels
// apply contrast and saturation correction after recombination
void CollapsePyramid(std::vector<CUDA::TDeviceMat<float>>& pyramidTexStack, Point2i texSize,
	unsigned levels, uint stackSize, cudaStream_t stream, float correctContrast, float correctSaturation) {
	dim3 blockSize(16, 16);
	dim3 gridSize((texSize.x() + blockSize.x - 1) / blockSize.x, (texSize.y() + blockSize.y - 1) / blockSize.y);
	float multiplier[2] = {2.f, 1.3f};
	int blur = 0;
	for (int i = levels-1; i>0; i--) {
		CombineKernel<<<gridSize, blockSize, 0, stream>>>(pyramidTexStack[i].GetDeviceData(), pyramidTexStack[i-1].GetDeviceData(), texSize, stackSize, blur, multiplier[i - 1]);
		CUDA_CHECK_LAST_ERROR;
	}
	if (correctContrast > 0.f) { 
		CorrectContrastKernel<<<gridSize, blockSize, 0, stream>>>(pyramidTexStack[0].GetDeviceData(), texSize, stackSize, correctContrast);
		CUDA_CHECK_LAST_ERROR;
	}
	if (correctSaturation > 0.f) {
		CorrectSaturationKernel<<<gridSize, blockSize, 0, stream>>>(pyramidTexStack[0].GetDeviceData(), texSize, stackSize, correctSaturation);
		CUDA_CHECK_LAST_ERROR;
	}
}
/*----------------------------------------------------------------*/

// kernels and functions relative to final value to calcuating for each pyramid stage from stacked data

// perceptually-weighted inner product in CIE Lab space.
// Used as a color distance metric for outlier detection and blending decisions
__device__ float LabNorm(Point3 a, Point3 b, float mod = 2.f) {
    return sqrtf((a.x() * b.x() / 100.f / mod) + ((a.y() * b.y() / 255.f )+ (a.z() * b.z() / 255.f))*(3.f - (1.f / mod))/2);
}
__device__ float LabNorm(float3 a, float3 b, float mod = 2.f) {
    return sqrtf((a.x * b.x / 100.f / mod) + ((a.y * b.y / 255.f )+ (a.z * b.z / 255.f))*(3.f - (1.f / mod))/2);
}
__device__ float LabNormColor(Point3 a, Point3 b) {
    return sqrtf((a.y() * b.y() / 255.f )+ (a.z() * b.z() / 255.f));
}

// find the stack sample that minimizes the mean absolute deviation to all other samples
__device__ float GetConsensus(float* __restrict__ pixStack, float* __restrict__ visibilityScore, int& consensusIdx, uint channel, uint stackSize) {
	float bestConsensus = -1;
	consensusIdx = -1;
	for (int i = 0; i < stackSize; i++) {
		if (visibilityScore[i] <= 0)
			continue;
		float value = pixStack[i*3 + channel];
		float meanDist = 0.f;
		for (int j = 0; j < stackSize; j++) {
			if (i == j) continue;
			if (visibilityScore[j] <= 0)
				continue;
			float otherValue = pixStack[j*3 + channel];
			meanDist += fabsf(value - otherValue);
		}
		meanDist /= float(stackSize - 1);
		if (meanDist < bestConsensus || bestConsensus < 0) {
			bestConsensus = meanDist;
			consensusIdx = i;
		}
	}
	return pixStack[consensusIdx*3 + channel];
}

// overload of GetConsensus operating on a subset of stack slots specified by sampleIdxs.
__device__ float GetConsensus(float* __restrict__ pixStack, float* __restrict__ visibilityScore, int* sampleIdxs, int& consensusIdx, uint channel, uint sampleSize) {
	float bestConsensus = -1;
	consensusIdx = -1;
	for (int i = 0; i < sampleSize; i++) {
		int idx = sampleIdxs[i];
		if (idx == -1) continue;
		if (visibilityScore[idx] <= 0)
			continue;
		float value = pixStack[idx*3 + channel];
		float meanDist = 0.f;
		for (int j = 0; j < sampleSize; j++) {
			int cIdx = sampleIdxs[j];
			if (cIdx == -1) continue;
			if (idx == cIdx) continue;
			if (visibilityScore[cIdx] <= 0)
				continue;
			float otherValue = pixStack[cIdx*3 + channel];
			meanDist += fabsf(value - otherValue);
		}
		meanDist /= float(sampleSize - 1);
		if (meanDist < bestConsensus || bestConsensus < 0) {
			bestConsensus = meanDist;
			consensusIdx = idx;
		}
	}
	if(consensusIdx == -1) 
		return 0.f;
	else
		return pixStack[consensusIdx*3 + channel];
}


// build a binary variation map where variationMap[pix] = 1 if the best-view index at 'stack' slot
// differs from any 8-connected neighbor, indicating a best-view boundary
__global__ void MakeIdxVariationMap(int* __restrict__ idxStacks, uint* __restrict__  variationMap, Point2i texSize, uint stackSize, uint stack) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= texSize.x() || pixY >= texSize.y()) return;
	int pix = pixY * texSize.x() + pixX;
	int idxIdx = pix * stackSize;
	bool found = false;
	int centerIdx = idxStacks[idxIdx + stack];
	if (centerIdx == -1)
		return;
	for (int dy = -1; dy <= 1; dy++) {
		for (int dx = -1; dx <= 1; dx++) {
			int nx = pixX + dx;
			int ny = pixY + dy;
			if (nx < 0 || nx >= texSize.x() || ny < 0 || ny >= texSize.y())
				continue;
			int nPix = ny * texSize.x() + nx;
			int nIdxIdx = nPix * stackSize;
			int vidx = idxStacks[nIdxIdx + stack];
			if (vidx == -1)
				continue;
			if (vidx !=  centerIdx) {
				variationMap[pix] = 1;
				found = true;
			}
			break;
		}
		if (found)
			break;
	}
}
// per-pixel stack collapse kernel for the perspective TextureCURAST pipeline,
// algorithm:
//   1. Find the best-scoring sample in the visibility-score stack,
//   2. Set the coarsest (color) level to the running weighted average (texAverage),
//   3. For each detail (Laplacian) level: blend between the consensus sample (robust to outliers)
//      and the best sample, weighted by distance to the nearest best-view boundary,
//      far from boundaries: use the best sample for sharpness.
//      near boundaries: use consensus to reduce seam artifacts.
__global__ void CollapseStacksKernel(float** __restrict__ texStacks, float* __restrict__ texAverage, float* bufferCount, float* __restrict__ visibilityScores, float* __restrict__ distMap, Point2i texSize, uint stackSize, uint levels) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= texSize.x() || pixY >= texSize.y()) return;
	int pix = pixY * texSize.x() + pixX;
	int countsIdx = pix * stackSize;
	int baseIdx = pix * 3 * stackSize;
	float bestScore = 0.f;
	int bestIdx = -1;
	// get best sample of the stack
	Point3 mean = Point3::Zero();
	for (int i = 0; i < stackSize; i++) {
		float c = visibilityScores[countsIdx + i];
		uint dIndex = texSize.x()*texSize.y() * i + pix;
		if (c > bestScore) {
			bestScore = c;
			bestIdx = i;
		}
	}
	if (bestIdx == -1)
		return;
	float dist = distMap[pix];
	{ // set color stage to average
		float* texStack = &texStacks[levels-1][baseIdx];
		texStack[0] = texAverage[pix*3];
		texStack[1] = texAverage[pix*3 + 1];
		texStack[2] = texAverage[pix*3 + 2];
	}

	// set differential stages either to best sample or mixture of best sample and consensus depending on distance to a best view change
	float tD = MAX(1.f,1.f * texSize.x()/512.f);
	for (int i = 0; i < levels-1; i++) {
		float dist2 = min(1.f, dist / ((i+1)*tD));
		float* texStack = &texStacks[i][baseIdx];
		float consensus[3];
		int consensusIdx;
		consensus[0] = GetConsensus(texStack, &visibilityScores[countsIdx], consensusIdx, 0, stackSize);
		consensus[1] = GetConsensus(texStack, &visibilityScores[countsIdx], consensusIdx, 1, stackSize);
		consensus[2] = GetConsensus(texStack, &visibilityScores[countsIdx], consensusIdx, 2, stackSize);
		if (dist2 != 1.f && consensusIdx != -1) {
			texStack[0] = consensus[0] * (1.f - dist2) + texStack[bestIdx*3] * dist2;
			texStack[1] = consensus[1] * (1.f - dist2) + texStack[bestIdx*3 + 1] * dist2;
			texStack[2] = consensus[2] * (1.f - dist2) + texStack[bestIdx*3 + 2] * dist2;
		} else {
			texStack[0] = texStack[bestIdx*3];
			texStack[1] = texStack[bestIdx*3 + 1];
			texStack[2] = texStack[bestIdx*3 + 2];
		}
	}
}


// remove pixels from finalPyr that lack corroborating samples in the stack
// a pixel is cleared (set to zero) if fewer than 2 stack samples are within Lab distance
// 'threshold' from the current finalPyr color, AND the tile normal is nearly vertical (>=0.9).
__global__ void Corroborate(float** __restrict__ texStack, PyrPix* __restrict__ finalPyr , float* __restrict__ normalMap, Point2i texSize, uint stackSize, uint levels, float threshold) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;	
	if (pixX >= texSize.x() || pixY >= texSize.y()) return;
	int pix = pixY * texSize.x() + pixX;
	int stackIdx = pix * 3 * stackSize;
	float normal = normalMap[pix];
	if (normal < 0.9f)
		return;
	Point3 C = finalPyr[pix].GetColor();
	uint cmpt = 0;
	for (int i = 0; i < stackSize; i++) {
		Point3 nC =Point3(0.f,0.f,0.f);
		for (int j = 0; j < levels; j++) {
			if (j != levels - 1) continue;
			float* cStack = &texStack[j][stackIdx];
			nC += Point3(cStack[i * 3], cStack[i * 3 + 1],cStack[i * 3 + 2]);
		}
		float dist = LabNorm(C-nC,C-nC);
		if ( dist < threshold)  
			cmpt ++;
		if (cmpt > 1)
			return;
	}	
	finalPyr[pix] = PyrPix();
}

__global__ void SetVisibilityKernel(float** __restrict__ texStack, float* __restrict__ visibilityScores, Point2i texSize, uint stackSize, uint levels) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;	
	if (pixX >= texSize.x() || pixY >= texSize.y()) return;
	int pix = pixY * texSize.x() + pixX;
	int texIdx = pix * 3 * stackSize;
	int scoreIdx = pix * stackSize;
	float* stack = &texStack[levels-1][texIdx];
	if (stack[0] == 0.f && stack[1] == 0.f && stack[2] == 0.f)
		visibilityScores[scoreIdx] = 0.f;
	else
		visibilityScores[scoreIdx] = 1.f;
}

__global__ void SetBestViewIdx(int* __restrict__ idxStacks, float* __restrict__ visibilityScores, Point2i texSize, uint stackSize) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= texSize.x() || pixY >= texSize.y()) return;
	int pix = pixY * texSize.x() + pixX;
	int idx = pix * stackSize;
	float bestScore = -1.f;
	int bestIdx = -1;
	for (int i = 0; i < stackSize; i++) {
		float c = visibilityScores[idx + i];
		if (c > bestScore) {
			bestScore = c;
			bestIdx = idxStacks[idx+i];
		}
	}
	idxStacks[idx] = bestIdx;
}


// initialize the connected-component tag map for empty regions,
// sets replaceArea[pix] = pix (pixel's own linear index) for pixels where finalPyr color is black
__global__ void InitTagArea(int* __restrict__ replaceArea, PyrPix* pyrMap, Point2i size) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= size.x() || pixY >= size.y()) return;
	int pixIdx = pixY * size.x() + pixX;
	Point3 c = pyrMap[pixIdx].GetColor();
	if (c.x() == 0.f && c.y() == 0.f && c.z() == 0.f)
		replaceArea[pixIdx] = pixIdx;
}

// single pass of connected-component label propagation (Union-Find expansion),
// each empty pixel (replaceArea != 0) checks its 8 neighbors and adopts the maximum
// tag value found — this merges components by propagating the largest seed index,
// repeat until no changes occur to converge to a stable labeling
__global__ void PropagateTag(int* __restrict__ replaceArea, Point2i size) {
	int pixX = blockIdx.x * blockDim.x + threadIdx.x;
	int pixY = blockIdx.y * blockDim.y + threadIdx.y;
	if (pixX >= size.x() || pixY >= size.y()) return;
	int pixIdx = pixY * size.x() + pixX;
	int value = replaceArea[pixIdx];
	if (value == 0) return;
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++ ) {
			if (i == 0 && j == 0) continue;
			if (pixY + j < 0 || pixX + i < 0 || pixY + j >= size.y() || pixX + i >= size.x()) continue;
			int nIdx = (pixY + j) * size.x() + pixX + i;
			int nValue = replaceArea[nIdx];
			if (nValue < 0) continue;
			if (value < nValue) {
				replaceArea[pixIdx] = nValue;
			}
		}
	}
}

__device__ inline Point3 ComputeLocalMeanColor(
	float** __restrict__ texStacks,
	Point2i texSize,
	uint stackSize,
	uint level,
	uint sampleIdx,
	uint pixIdx,
	int radius)
{
	const int x = int(pixIdx % texSize.x());
	const int y = int(pixIdx / texSize.x());
	Point3 sum(0.f, 0.f, 0.f);
	uint count = 0;
	float* levelStack = texStacks[level];
	for (int dy = -radius; dy <= radius; ++dy) {
		const int ny = y + dy;
		if (ny < 0 || ny >= texSize.y())
			continue;

		for (int dx = -radius; dx <= radius; ++dx) {
			const int nx = x + dx;
			if (nx < 0 || nx >= texSize.x())
				continue;

			const uint nPix = uint(ny * texSize.x() + nx);
			const uint base = nPix * stackSize * 3 + sampleIdx * 3;

			const Point3 c(levelStack[base], levelStack[base + 1], levelStack[base + 2]);
			if (c.x() == 0.f && c.y() == 0.f && c.z() == 0.f)
				continue;

			sum += c;
			++count;
		}
	}
	return (count > 0) ? (sum / float(count)) : Point3(0.f, 0.f, 0.f);
}

__device__ Point3 LabNormalize( Point3 a) {
	return Point3(a.x() / 100.f, (a.y() + 128.f) / 255.f, (a.z() + 128.f) / 255.f);
} 

// collapseStacksOrtho — collapse multi-view Laplacian pyramid stacks into a single pyramid output.
// best-view map          - for each pixel, pick the stack slot with the highest visibility
//                          score × colour magnitude and store it in bestViewIdxMap.
// view frequency sort    - count how many pixels name each view as their best view; sort views
//                          descending by count (mostFrequentOrder) so the widest-covering view
//                          is applied first.
// first composite pass   - iterate views in frequency order; for each view run two JFA passes:
//                          (a) distMap     = normalised distance from holes in final pyramid
//                          (blend-boundary weight, seeds = empty pixels in finalPyr),
//                          (b) distanceMap = normalised distance from holes in the view's own
//                          stack slot (view-coverage weight, seeds = empty pixels in view);
//                          blend view colour into finalPyr with soft boundaries using distMap and
//                          distanceMap; save result to savedResult.  
// remove outlier in	  - pixels with fewer than 2 corroborating samples in the stack (Lab distance < threshold) 
// finalPyr				    are cleared to black;
// restore little         — build binary mask1/mask2 from finalPyr (filled=1, empty=0);
// cluster                  morphological retract then expand to remove isolated outlier pixels or little cluster;
//                          restore savedResult where the mask changed. Connected component of empty pixels are tagged
// 						    and small are also filled back from savedResult to avoid over-removal.
// outlier removal        — 10 iterations of Mahalanobis: compute per-pixel Lab colour mean and
// in the stack             covariance over all stack slots; zero out slots whose Mahalanobis
//                          distance exceeds the threshold; morphological retract and expand is used again
// 							to clean up too small clusters of outliers		
// base color             — connected-component of empty pixels in finalPyr are re-tagged
// level fill               for each empty area, run JFA to build distMap (float
//                          distance to the area edge) for the boundary-band colour comparison;
//                          pick the best N candidate views minimising L1 colour difference on
//                          that band; fill the area with blended patches using distMap/patchDist.
// differential           — fill Laplacian detail levels adding best views one by one, blending done
// level fill               using 2 dist map : from holes in finalPyr and holes in the integrated view
//							from the stack
// final composite pass   — repeat the first composite pass
// Residual fill          — last filling using finalPyr using distMap as the interpolation weight.
// copy result            — copy each level of finalPyr into slot 0 of the corresponding pyramid
//                          stack
void CollapseStacksOrtho(std::vector<CUDA::TDeviceMat<float>>& pyramidTex, float* visibilityScores, float* normalMap, Point2i texSize, unsigned levelCount, uint stackSize, ::MVS::CollapseWorkspace& ws, cudaStream_t stream) {
	dim3 blockSize(16, 16);
	dim3 gridSize((texSize.x() + blockSize.x - 1) / blockSize.x, (texSize.y() + blockSize.y - 1) / blockSize.y);

	float maxDist1 = max(10.f * texSize.x() / 512.f, 3.01f);
	float maxDist2 = maxDist1;
	float maxDist3 = max(maxDist1 / 6.f, 3.01f);
	float a = 0.05;
	float b = 0.3;

	std::vector<float*> h_texStacks(levelCount);
	for (int i = 0; i < levelCount; i++) {
		h_texStacks[i] = pyramidTex[i].GetDeviceData();
	}
	ws.texStacks.ReallocateToAtLeastSize(levelCount); 	
	float** d_texStacks = ws.texStacks.GetDeviceData();
	cudaMemcpyAsync(d_texStacks, h_texStacks.data(), sizeof(float*) * levelCount, cudaMemcpyHostToDevice, stream);
	uint nPixels = texSize.x() * texSize.y();
	auto first = thrust::make_counting_iterator<uint>(0);
	auto last = first + nPixels;
	auto policy = thrust::cuda::par.on(stream);
	PyrPix* finalPyr = ws.finalPyr.GetDeviceData();
	cudaMemsetAsync(finalPyr, 0, nPixels * sizeof(PyrPix),stream);

	int* bestViewIdxMap = ws.bestViewIdxMap.GetDeviceData();
	thrust::fill(policy, bestViewIdxMap, bestViewIdxMap + nPixels, -1);
	float* distMap = ws.distMap.GetDeviceData();  
	char* mask1 = ws.mask1.GetDeviceData();        
	char* mask2 = ws.mask2.GetDeviceData();        
	char* mask3 = ws.mask3.GetDeviceData();
	int* tagsMap = ws.tagsMap.GetDeviceData();
	int* tmp = ws.tmpInt.GetDeviceData();

	Seed* seedMap1_d = ws.seedMap1.GetDeviceData();
	Seed* seedMap2_d = ws.seedMap2.GetDeviceData();

	// best-view map: for each pixel pick the stack slot with the highest
	// visibility score and not empty color and store its index in bestViewIdxMap.
	#if 1
	thrust::fill(policy, bestViewIdxMap, bestViewIdxMap + nPixels, -1);
	thrust::transform(policy, first, last, bestViewIdxMap, [ d_texStacks, visibilityScores, levelCount, stackSize] __device__(uint idx) {
		int bestView = -1;
		float bestScore = -1.f;
		float* pixStack = &d_texStacks[levelCount - 1][idx * stackSize * 3];
		for (uint i = 0; i < stackSize; i++) {
			Point3 color = Point3(pixStack[i * 3], pixStack[i * 3 + 1], pixStack[i * 3 + 2]);
			if (color.x() == 0.f && color.y() == 0.f && color.z() == 0.f)
				continue;
			float score = visibilityScores[idx * stackSize + i];
			if (score > bestScore) {
				bestScore = score;
				bestView = i;
			}
		}
		return bestView;
	});
	#endif

	// view frequency sort: count how many pixels each view is "best" for,
	// then sort descending so the most-covering view is applied first (mostFrequentOrder).
	std::vector<int> mostFrequentOrder;
	int nUnique = 0;
	#if 1
	{
		thrust::device_vector<int> dSorted(bestViewIdxMap, bestViewIdxMap + nPixels);
		thrust::sort(thrust::device, dSorted.begin(), dSorted.end());
		thrust::device_vector<int> dUniqueVals(nPixels);
		thrust::device_vector<int> dCounts(nPixels);
		auto new_end = thrust::reduce_by_key(
			thrust::device,
			dSorted.begin(), dSorted.end(),
			thrust::make_constant_iterator(1),
			dUniqueVals.begin(),
			dCounts.begin()
		);

		nUnique = static_cast<int>(new_end.first - dUniqueVals.begin());

		std::vector<int> hUniqueVals(nUnique);
		std::vector<int> hCounts(nUnique);
		mostFrequentOrder.resize(nUnique);

		thrust::copy(dUniqueVals.begin(), dUniqueVals.begin() + nUnique, hUniqueVals.begin());
		thrust::copy(dCounts.begin(), dCounts.begin() + nUnique, hCounts.begin());

		std::vector<std::pair<int, int>> valCountPairs(nUnique);
		for (int i = 0; i < nUnique; i++) {
			valCountPairs[i] = { hUniqueVals[i], hCounts[i] };
		}

		std::sort(valCountPairs.begin(), valCountPairs.end(),
			[](const std::pair<int, int>& a, const std::pair<int, int>& b) {
				return a.second > b.second;
			});
		bool hasMinusOne = false;
		for (int i = 0; i < nUnique; i++) {
			if ( valCountPairs[i].first == -1) {
				hasMinusOne = true;
				continue;
			}
			mostFrequentOrder[i] = valCountPairs[i].first;
		}
		if (hasMinusOne) {
			nUnique = hasMinusOne ? nUnique - 1 : nUnique;
			mostFrequentOrder.resize(nUnique);
		}
	}
	cudaDeviceSynchronize();
	#endif

	// iterate views in frequency order and blend each view's colour into finalPyr 
	// using two JFA-derived distance maps as blending weights, result is also 
	// copied to savedResult for later recovery.
	PyrPix* savedResult = ws.savedResult.GetDeviceData();
	cudaMemsetAsync(savedResult, 0, nPixels * sizeof(PyrPix),stream);
	#if 1
	{
		float* distanceMap = ws.distanceMap.GetDeviceData();
		cudaMemsetAsync(distanceMap, 0, nPixels * sizeof(float),stream);
		cudaMemsetAsync(finalPyr, 0, nPixels * sizeof(PyrPix),stream);
		cudaMemsetAsync(distMap, 0, nPixels * sizeof(float),stream);

		int viewIdx = mostFrequentOrder[0];
		for (int  i = 0; i < nUnique; i++) {
			viewIdx = mostFrequentOrder[i];
			// JFA pass: seeds = pixels where finalPyr is empty (holes in the current composite),
			// output: distMap = normalised distance from each pixel to the nearest hole boundary
			float blendDist = max(10.f * texSize.x() / 512.f, 8.f);
			{
				cudaMemsetAsync(seedMap1_d, 0, nPixels * sizeof(Seed),stream);
				cudaMemsetAsync(seedMap2_d, 0, nPixels * sizeof(Seed),stream);
				// init seeds
				thrust::transform(policy, first, last, seedMap1_d, [finalPyr, viewIdx, bestViewIdxMap, texSize, levelCount] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Point3 fin = finalPyr[idx].p[levelCount - 1];
					if (fin.x() == 0.f)
						return Seed(x, y);
					return Seed(-1, -1);	
				});
				int maxJump = max(texSize.x(), texSize.y());
				for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
					jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
					// Swap seed maps
					Seed* temp = seedMap1_d;
					seedMap1_d = seedMap2_d;
					seedMap2_d = temp;
				}
				thrust::transform(policy, first, last, distMap, [seedMap1_d, blendDist, texSize] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Seed s = seedMap1_d[idx];
					int sIdx = s.y * texSize.x() + s.x;
					float dx = float(x - s.x);
					float dy = float(y - s.y);
					
					float dist = min(sqrt(dx * dx + dy * dy)/blendDist,1.f);
					return dist;
				});
			}
			// JFA pass: seeds = pixels where this view's stack slot is empty,
			// output: distanceMap = normalised distance from each pixel to the view's nearest
			// uncovered pixel		
			float blendDist2 = max(10.f*texSize.x() / 512.f, 8.f);
			{
				cudaMemsetAsync(seedMap1_d, 0, nPixels * sizeof(Seed),stream);
				cudaMemsetAsync(seedMap2_d, 0, nPixels * sizeof(Seed),stream);
				// init seeds
				thrust::transform(policy, first, last, seedMap1_d, [viewIdx, d_texStacks, texSize, stackSize, levelCount] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					float* pixStack = &d_texStacks[levelCount - 1][idx * stackSize * 3];
					if (pixStack[viewIdx * 3] == 0)
						return Seed(x, y);
					return Seed(-1, -1);	
				});
				int maxJump = max(texSize.x(), texSize.y());
				for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
					jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
					// swap seed maps
					Seed* temp = seedMap1_d;
					seedMap1_d = seedMap2_d;
					seedMap2_d = temp;
				}
				thrust::transform(policy, first, last, distanceMap, [seedMap1_d, blendDist2, texSize] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Seed s = seedMap1_d[idx];
					int sIdx = s.y * texSize.x() + s.x;
					float dx = float(x - s.x);
					float dy = float(y - s.y);
					
					float dist = min(sqrt(dx * dx + dy * dy)/blendDist2,1.f);
					return dist;
				});
			}

			// blend view colour into finalPyr using JFA-derived weights:
			//   w  (distMap value)     = distance from nearest hole in finalPyr; p			
			//   pw (distanceMap value) = distance from the view's own uncovered pixels;			
			// Effective blend factor is max(w, 1-pw) for a smooth soft-boundary transition.
			thrust::for_each(policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), 
				[distMap, viewIdx, d_texStacks, finalPyr, bestViewIdxMap, blendDist, blendDist2, distanceMap, stackSize, texSize, levelCount, a, b] __device__(uint idx) {
				int x = idx % texSize.x();
				int y = idx / texSize.x();
				float dis = distMap[idx];
				float dis2 = distanceMap[idx];
				PyrPix fin = finalPyr[idx];
				if (dis >= 1)
					return;
				PyrPix P = PyrPix();
				for (uint l = 0; l < levelCount; l++) {
					float* pixStack = &d_texStacks[l][idx * stackSize * 3 + viewIdx * 3];
					P.p[l].x() += pixStack[0];
					P.p[l].y() += pixStack[1];
					P.p[l].z() += pixStack[2];
					}
				if (P.p[0].x() == 0)
					return;
				PyrPix res;
				for (uint l = 0; l < levelCount; l++) {
						float d = dis; 
						float pd = dis2;
					    // if not on the coarsest level, the blending is applied on a shorter distance
						if (l != levelCount - 1 && dis > 0) {
							d = min(3 * d/(float)(l + 1),1.f);
						    pd = min(3 * pd / (float)(l + 1), 1.f);
						}
						float w;
						if ( d < a) 
							w = d;
						else if ( d < b)
							if (d < 1 - pd) 
								w = (d * (b - d) + (1 - pd) * (d - a)) / (b - a) ;
							else
								w = d;
						else
							w = max(d, 1 - pd);
						if (fin.p[levelCount - 1].x() != 0)
							res.p[l] = fin.p[l] * w + P.p[l] * (1 - w);
						else 
							res.p[l] = P.p[l];
					}
				finalPyr[idx] = res;

				});
		}
		cudaMemcpyAsync(savedResult, finalPyr, nPixels * sizeof(PyrPix), cudaMemcpyDeviceToDevice, stream);
	}
	#endif

	// outlier removal from finalPyr: zero out pixels in finalPyr that lack corroborating samples in the stack (fewer than 2 samples within Lab distance threshold), 
	// and have near-vertical normals (<0.9) (removal is done only on near horizontal surfaces)
	#if 1
	Corroborate<<<gridSize, blockSize, 0, stream>>>(d_texStacks,finalPyr,normalMap,texSize,stackSize,levelCount,0.3f);

	// little cluster restoration: build binary mask1/mask2 from finalPyr (1=filled, 0=empty), then
	// apply a morphological retract followed by an expand to detect and remove spurious little isolated cluster
	// also tag connected components of empty pixels and restore small ones before a last expend passes
	#if 1
	{
		// initialise mask1 (char) as filled/empty map of finalPyr; save original into mask2
		thrust::transform(policy, first, last, mask1, [finalPyr, levelCount] __device__(uint idx) -> char {	Point3 C = finalPyr[idx].p[levelCount - 1];	return (C.x() == 0.f && C.y() == 0.f && C.z() == 0.f) ? (char)0 : (char)1;});
		cudaMemcpyAsync(mask2, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice,stream);
		// retract: expand empty regions into filled neighbours (erodes filled islands)
		#if 1
		uint iter = max(texSize.x() / 512 / 4, 1);
		for (uint j = 0; j < iter; j++) {
			cudaMemcpyAsync(mask3, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice, stream);
			thrust::transform(policy, first, last, mask1, [mask3, levelCount, texSize] __device__(uint idx) -> char {
				int x = idx % texSize.x();
				int y = idx / texSize.x();
				if (mask3[idx] == 1)
					return (char)1;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (i == 0 && j == 0) continue;
						int nX = x + i;
						int nY = y + j;
						if (nX >= texSize.x() || nY >= texSize.y() || nX < 0 || nY < 0) continue;
						int nIdx = nY * texSize.x() + nX;
						if (mask3[nIdx] == 1) {
							return (char)1;
						}
					}
				}
				return (char)0;
			});
		}
		#endif
			
	#if 1
		#if 1
		// expand: re-fills eroded originaly empty areas if any empty pixel persists after retract
		iter = max(2*texSize.x() / 512 / 4, 1);
		for (uint j = 0; j < iter; j++) {
			cudaMemcpyAsync(mask3, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice,stream);
			// expand empty area
			thrust::transform(policy, first, last, mask1, [mask2, mask3, levelCount, texSize] __device__(uint idx) -> char {
				int x = idx % texSize.x();
				int y = idx / texSize.x();
				if (mask3[idx] == 0)
					return (char)0;
				if(mask2[idx] == 1)
					return (char)1;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (i == 0 && j == 0) continue;
						int nX = x + i;
						int nY = y + j;
						if (nX >= texSize.x() || nY >= texSize.y() || nX < 0 || nY < 0) continue;
						int nIdx = nY * texSize.x() + nX;
						if (mask3[nIdx] == 0) {
							return (char)0;
						}
					}
				}
				return (char)1;
			});
		}
		#endif 


		// remove by tag area size
		// tag empty areas
		#if 1
		{
			{
				cudaMemsetAsync(tmp, 0, nPixels * sizeof(int),stream);
				cudaMemsetAsync(tagsMap, 0, nPixels * sizeof(int),stream);
				int changes = 1;
				// init tags
				thrust::transform(policy, first, last, tagsMap, [mask1, texSize] __device__(int idx) {
					if (mask1[idx] == 0)
						return idx + 1;
					return 0;
				});
				CUDA_CHECK_LAST_ERROR;
				auto policy = thrust::cuda::par.on(stream);
				while (changes != 0) {
					PropagateTag<<<gridSize, blockSize, 0, stream>>>(tagsMap, texSize);
					auto iterIf = thrust::make_transform_iterator(thrust::make_counting_iterator<uint>(0), [tmp, tagsMap] __host__ __device__(uint idx) -> uint {
						return (tmp[idx] != tagsMap[idx]) ? 1u : 0u;
					});
					changes = thrust::reduce(policy, iterIf, iterIf + nPixels, (uint)0);
					cudaMemcpyAsync(tmp, tagsMap, nPixels * sizeof(int), cudaMemcpyDeviceToDevice,stream);
				}
			}

			// first pass to remove small areas
			thrust::device_vector<int> dKeysP(nPixels);
			thrust::transform(policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), dKeysP.begin(),
					[tagsMap] __device__ (uint p) {
						const int t = tagsMap[p];
						return (t > 0) ? t : 0;
					});

			thrust::device_vector<uint> dPermP(nPixels);
			thrust::copy(policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), dPermP.begin());
			thrust::sort_by_key(policy, dKeysP.begin(), dKeysP.end(), dPermP.begin());
			thrust::universal_vector<int> tagAreaP(nPixels);		
			thrust::device_vector<int> vKeys;
			int nKeys;
			// get area size for each tag
			{	
				int* keys = ws.keys.GetDeviceData();
				auto ones = thrust::make_constant_iterator<int>(1);
				auto end_cnt = thrust::reduce_by_key(policy, dKeysP.begin(), dKeysP.end(), ones, keys, tagAreaP.begin());
				nKeys = (int)(end_cnt.first - keys); // number of unique keys
				vKeys.resize(nKeys);
				cudaMemcpyAsync(vKeys.data().get(), keys, nKeys * sizeof(int), cudaMemcpyDeviceToDevice, stream);
				cudaStreamSynchronize(stream);	
			}
			// remove small areas
			{	
				auto* pTagAreaP = thrust::raw_pointer_cast(tagAreaP.data());
				auto* pVKeys    = thrust::raw_pointer_cast(vKeys.data());
				uint minArea = (uint)ceil(nPixels/60000.f);
				thrust::transform(policy,
				tagsMap, tagsMap + nPixels, tagsMap, 
				[pTagAreaP, pVKeys, nKeys, minArea] __device__(int t) { 
					for (int i = 0; i < nKeys; i++) {
						if (abs(t) == pVKeys[i]) {
							return (pTagAreaP[i] > minArea) ? t : 0;
						}
					}	
					return t;
				});				
				thrust::transform(policy,
				thrust::make_counting_iterator<int>(0), thrust::make_counting_iterator<int>(nPixels), mask1, 
				[tagsMap, mask1] __device__ (uint p) {
				const int t = tagsMap[p];
				if (mask1[p] == 1)
					return (char)1;
				if (t == 0 && mask1[p] == 0) {
					return (char)1;
				}
				return (char)0;
				});
			}
		}
		#endif
		#if 1
		// increase the size of the remaining empty areas 		
		iter = max(10*texSize.x() / 512 / 4, 1);
		for (uint j = 0; j <iter; j++) {
			cudaMemcpyAsync(mask3, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice,stream);
			// expand empty area
			thrust::transform(policy, first, last, mask1, [mask3, levelCount, texSize] __device__(uint idx) -> char {
				int x = idx % texSize.x();
				int y = idx / texSize.x();
				if (mask3[idx] == 0)
					return (char)0;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (i == 0 && j == 0) continue;
						int nX = x + i;
						int nY = y + j;
						if (nX >= texSize.x() || nY >= texSize.y() || nX < 0 || nY < 0) continue;
						int nIdx = nY * texSize.x() + nX;
						if (mask3[nIdx] == 0) {
							return (char)0;
						}
					}
				}
				return (char)1;
			});
		}
	#endif
		// apply change to finalyPyr
		thrust::transform(policy, first, last, finalPyr, [mask1, savedResult, levelCount] __device__(uint idx) {
			if (mask1[idx] == 0)
				return PyrPix();
			return savedResult[idx];
		});
		#endif
	}
    #endif
	#endif
	
	#if 1
	// outlier removal from stack (10 Mahalanobis iterations): compute per-pixel Lab colour mean
	// and covariance over all stack slots; zero out slots whose Mahalanobis distance exceeds the
	// threshold,  expend and retracted is used again to clean up too small clusters of outliers.
	#if 1
	Point3* average = ws.average.GetDeviceData();
	Matrix3* covariance = ws.covariance.GetDeviceData();
	PyrPix* saveTmp = ws.saveTmp.GetDeviceData();
	uint maxIter = min(10, stackSize/5);
	for (uint m = 0; m < 10; m++)
	{
		float threshold = 2.5; // threshold for Mahalanobis distance
		 //compute average color for each pixel
		thrust::transform(policy, first, last, average, [d_texStacks, levelCount, stackSize] __device__(uint idx) {
			Point3 avg(0.f, 0.f, 0.f);
			uint cmpt = 0;
			for (uint i = 0; i < stackSize; i++) {
				Point3 C = Point3(0.f, 0.f, 0.f);
				for (uint l = 0; l < levelCount; l++) {
					float* cStack = &d_texStacks[l][idx * stackSize * 3];
					C += Point3(cStack[i * 3], cStack[i * 3 + 1], cStack[i * 3 + 2]);
				}
				if (C.x() == 0.f && C.y() == 0.f && C.z() == 0.f)
					continue;
				avg += C;
				cmpt++;
			}
			if (cmpt > 0) {
				avg /= cmpt;
			}
			return avg;
		});
		// compute covariance matrix for each pixel
		thrust::transform(policy, first, last, covariance,[d_texStacks, average, levelCount, stackSize] __device__(uint idx) {
			Matrix3 cov = Matrix3::Zero();
			Point3 mean = LabNormalize(average[idx]);
			uint cmpt = 0;
			for (uint i = 0; i < stackSize; i++) {
				Point3 C = Point3(0.f, 0.f, 0.f);
				for (uint l = 0; l < levelCount; l++) {
					float* cStack = &d_texStacks[l][idx * stackSize * 3];
					C += Point3(cStack[i * 3], cStack[i * 3 + 1], cStack[i * 3 + 2]);
				}
				if (C.x() == 0.f && C.y() == 0.f && C.z() == 0.f)
					continue;
				C = LabNormalize(C);
				Point3 diff = C - mean;
				cov += diff * diff.transpose();
				cmpt++;
			}
		 if (cmpt > 1) {
			 cov /= (cmpt - 1);
		 }
		 return cov;
		});
		// compute Mahalanobis distance for each sample in the stack and remove outliers
		{
			for (uint i = 0; i < stackSize; i++) {
				// save the pyramid for the current sample to recover later little clusters of removed samples
				thrust::transform(policy, first, last, saveTmp, [d_texStacks, i, levelCount, stackSize] __device__(uint idx) {
					PyrPix p;
					for (uint l = 0; l < levelCount; l++) {
						float* cStack = &d_texStacks[l][idx * stackSize * 3];
						p.p[l].x() = cStack[i * 3];
						p.p[l].y() = cStack[i * 3 + 1];
						p.p[l].z() = cStack[i * 3 + 2];
					}
					return p;
				});

				// compute Mahalanobis distance and remove outliers
				thrust::for_each(policy, first, last, [d_texStacks, average, covariance, visibilityScores, normalMap, levelCount, stackSize, threshold, i] __device__(uint idx) {
					if (normalMap[idx] < 0.9) 
						return;
					
					Matrix3 cov = covariance[idx];
					Point3 mean = LabNormalize(average[idx]);
					
					Matrix3 invCov = cov.inverse();
					Point3 C = Point3(0.f, 0.f, 0.f);
					for (uint l = 0; l < levelCount; l++) {
						float* cStack = &d_texStacks[l][idx * stackSize * 3];
						C += Point3(cStack[i * 3], cStack[i * 3 + 1], cStack[i * 3 + 2]);
					}
					if (C.x() == 0.f && C.y() == 0.f && C.z() == 0.f)
						return;
					C = LabNormalize(C);
					Point3 diff = C - mean;
					float mahalanobisDist = sqrt(diff.transpose() * invCov * diff);
					if (mahalanobisDist > threshold) {
						for (uint l = 0; l < levelCount; l++) {
							float* cStack = &d_texStacks[l][idx * stackSize * 3];
							cStack[i * 3] = 0.f;
							cStack[i * 3 + 1] = 0.f;
							cStack[i * 3 + 2] = 0.f;
						}
						visibilityScores[idx * stackSize + i] = 0.f;
					}
				});
				
				// recover isolated removals using morphological operations similar to the one applied on finalPyr: first retract  then expand
				// build binary mask1 from the current sample (1=filled, 0=empty)
				thrust::transform(policy, first, last, mask1,[d_texStacks, i, levelCount, stackSize] __device__(uint idx) {
					float* cStack = &d_texStacks[levelCount - 1][idx * stackSize * 3];
					Point3 C = Point3(cStack[i * 3], cStack[i * 3 + 1], cStack[i * 3 + 2]);
					return (C.x() == 0.f && C.y() == 0.f && C.z() == 0.f) ? (char)0 : (char)1;
				});
				cudaMemcpyAsync(mask2, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice, stream);
				// retract: retract empty regions into filled neighbours 				
				uint iter = max(texSize.x()/512/2,1);				
				for (uint it = 0; it < iter; it++) {
					cudaMemcpyAsync(mask3, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice, stream);
					thrust::for_each(policy, first, last,[i, mask1,  mask3, levelCount, stackSize, texSize] __device__(uint idx) {
						int x = idx % texSize.x();
						int y = idx / texSize.x();
						if (mask3[idx] == 1)
							return;
						// loop over 8 neighbours
						bool full = true;
						for (int j = -1; j <= 1; j++) {
							for (int k = -1; k <= 1; k++) {
								if ( j == 0 && k == 0) continue;
								int nX = x + j;
								int nY = y + k;
								if (nX >= texSize.x() || nY >= texSize.y() || nX < 0 || nY < 0) continue;
								uint nIdx = nY * texSize.x() + nX;
								char nMask = mask3[nIdx];
								if ( nMask == 1) {
									/*cmpt++;
									if(cmpt >= 4) {*/
										full = false;
										/*break;
									}*/
								} 
							}
							if (!full)
								break;
						}
						if (!full) {
							mask1[idx] = 1;
						}
					});
				}
				// expand empty regions if they survived the retract		
				iter = max(10 * texSize.x()/512/2,1);
				for (uint it = 0; it < iter; it++) {
					cudaMemcpyAsync(mask3, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice, stream);
					thrust::for_each(policy, first, last,[i, mask1, mask2, mask3, levelCount, stackSize, texSize] __device__(uint idx) {
						int x = idx % texSize.x();
						int y = idx / texSize.x();
						if (mask2[idx] == 1 || mask3[idx] == 0)
							return;
						// loop over 8 neighbours
						bool nearEmpty = false;
						for (int j = -1; j <= 1; j++) {
							for (int k = -1; k <= 1; k++) {
								if ( j == 0 && k == 0) continue;
								int nX = x + j;
								int nY = y + k;
								if (nX >= texSize.x() || nY >= texSize.y() || nX < 0 || nY < 0) continue;
								uint nIdx = nY * texSize.x() + nX;
								char nMask = mask3[nIdx];
								if (nMask == 0) {
									/*cmpt++;
									if(cmpt >= 4) {*/
										nearEmpty = true;
										/*break;
									}*/
								} 
							}
							if (nearEmpty)
								break;
						}
						if (nearEmpty) {
							mask1[idx] = 0;
						}
					});
				}
				// expand further the empty regions
				iter = max(texSize.x()/1025,1);				
				for (uint it = 0; it < iter; it++) {
					cudaMemcpyAsync(mask3, mask1, nPixels * sizeof(char), cudaMemcpyDeviceToDevice, stream);
					thrust::for_each(policy, first, last,[i, mask1, mask3, levelCount, stackSize, texSize] __device__(uint idx) {
						int x = idx % texSize.x();
						int y = idx / texSize.x();
						if (mask3[idx] == 0)
							return;
						// loop over 8 neighbours
						bool nearEmpty = false;
						for (int j = -1; j <= 1; j++) {
							for (int k = -1; k <= 1; k++) {
								if ( j == 0 && k == 0) continue;
								int nX = x + j;
								int nY = y + k;
								if (nX >= texSize.x() || nY >= texSize.y() || nX < 0 || nY < 0) continue;
								uint nIdx = nY * texSize.x() + nX;
								char nMask = mask3[nIdx];
								if (nMask == 0) {
										nearEmpty = true;
									} 
							}
							if (nearEmpty)
								break;
						}
						if (nearEmpty) {
							mask1[idx] = 0;
						}
					});
				}
				// apply changes to the view in the stack
				thrust::for_each(policy, first, last,[i, mask1, mask2, saveTmp, d_texStacks, levelCount, stackSize] __device__(uint idx) {
					if (mask1[idx] == mask2[idx])
						return;
					PyrPix p = saveTmp[idx];
					for (uint l = 0; l < levelCount; l++) {
						float* cStack = &d_texStacks[l][idx * stackSize * 3];
						if (mask1[idx] == 0) {
							cStack[i * 3] = 0;
							cStack[i * 3 + 1] = 0;
							cStack[i * 3 + 2] = 0;
						} else {
							cStack[i * 3] = p.p[l].x();
							cStack[i * 3 + 1] = p.p[l].y();
							cStack[i * 3 + 2] = p.p[l].z();
						}
					}
				});

			}
		}
	}
	#endif

	// save the empty area before color level filling to use it for differential levels filling
	#if 1
	{
		cudaMemsetAsync(mask1, 0, nPixels * sizeof(char),stream);
		thrust::transform(policy, first, last, mask1, [finalPyr, levelCount] __device__(uint idx) -> char {
			Point3 C = finalPyr[idx].p[levelCount - 1];
			return (C.x() == 0.f && C.y() == 0.f && C.z() == 0.f) ? (char)0 : (char)1;
		});
	}
	#endif
	thrust::universal_vector<int> tagAreaP(nPixels);
	thrust::device_vector<int> dKeysP(nPixels);
	thrust::device_vector<uint> dPermP(nPixels);
	thrust::device_vector<int> vKeys;
	int nKeys;

	// color level filling: for each remaining empty area, run JFA to build distMap
	// for the boundary-band colour comparison, select the best N candidate views 
	// minimising L1 colour difference on that band, and fill the area with blended patches.
	#if 1
	uint nIter = 2;
	for (int it = 0; it < nIter; it++) {
		// retag empty areas
		{
			cudaMemsetAsync(tmp, 0, nPixels * sizeof(int),stream);
			cudaMemsetAsync(tagsMap, 0, nPixels * sizeof(int),stream);
			int changes = 1;
			InitTagArea<<<gridSize, blockSize, 0, stream>>>(tagsMap, finalPyr, texSize);
			CUDA_CHECK_LAST_ERROR;
			auto policy = thrust::cuda::par.on(stream);
			while (changes != 0) {
				PropagateTag<<<gridSize, blockSize, 0, stream>>>(tagsMap, texSize);
				auto iterIf = thrust::make_transform_iterator(thrust::make_counting_iterator<uint>(0), [tmp, tagsMap] __host__ __device__(uint idx) -> uint {
					return (tmp[idx] != tagsMap[idx]) ? 1u : 0u;
				});
				changes = thrust::reduce(policy, iterIf, iterIf + nPixels, (uint)0);
				cudaMemcpyAsync(tmp, tagsMap, nPixels * sizeof(int), cudaMemcpyDeviceToDevice);
			}
		}
		{
			// build key/perm for sort by key
			thrust::transform( policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), dKeysP.begin(),
				[tagsMap] __device__ (uint p) {
					const int t = tagsMap[p];
					return (t > 0) ? t : 0;
				});
			thrust::copy( policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), dPermP.begin());
			thrust::sort_by_key(policy, dKeysP.begin(), dKeysP.end(), dPermP.begin());
			// get area size for each tag
			{
				int* keys = ws.keys.GetDeviceData();
				auto ones = thrust::make_constant_iterator<int>(1);
				auto end_cnt = thrust::reduce_by_key(policy, dKeysP.begin(), dKeysP.end(), ones, keys, tagAreaP.begin());
				nKeys = (int)(end_cnt.first - keys); // number of unique keys
				vKeys.resize(nKeys);
				cudaMemcpyAsync(vKeys.data().get(), keys, nKeys * sizeof(int), cudaMemcpyDeviceToDevice, stream);
				cudaStreamSynchronize(stream);	
			}
			thrust::host_vector<int> hKeys(nKeys);
			thrust::host_vector<int> hTagAreaP(nKeys);
			cudaMemcpyAsync(hKeys.data(), vKeys.data().get(), nKeys * sizeof(int), cudaMemcpyDeviceToHost, stream);
			cudaMemcpyAsync(hTagAreaP.data(), tagAreaP.data().get(), nKeys * sizeof(int), cudaMemcpyDeviceToHost, stream);
			cudaStreamSynchronize(stream);
			#if 1
			const uint nBestViews = ::MVS::CollapseWorkspace::kMaxBestViews;
			float* patchDist = ws.patchDist.GetDeviceData();
			thrust::device_vector<int> d_outCounts(nPixels);
			// for each tag, build distMap (float) = JFA distance from the area boundary,
			// then select the N best views by measuring L1 colour diff in the boundary band
			// (pixels where 0 < distMap < 1 are the band), and fill the area with those views.
			for (int t = 0; t < nKeys; t++) {
				int tag = hKeys[t];
				float maxDist = max(2 * texSize.x() / 512.f, 4.f);
				// JFA: seeds = pixels belonging to this tag; distMap = truncated distance to the
				// area edge normalised to [0,1] — values in (0,1) form the boundary band used
				// for colour difference scoring below.
				{
					cudaMemsetAsync(distMap, 0, nPixels * sizeof(float),stream);
					initSeedsWithATag<<<gridSize, blockSize, 0, stream>>>(tagsMap, seedMap1_d, texSize, tag);
					CUDA_CHECK_LAST_ERROR;
					int maxJump = max(texSize.x(), texSize.y());
					for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
						jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(
							seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
						Seed* temp = seedMap1_d;
						seedMap1_d = seedMap2_d;
						seedMap2_d = temp;
					}
				}
				computeTruncDistance<<<gridSize, blockSize, 0, stream>>>(
					seedMap1_d, distMap, texSize.x(), texSize.y(), maxDist);
				CUDA_CHECK_LAST_ERROR;
				// select n-best views minimizing mean diff on neighborhood band (0 < dist < 1)
				std::vector<float> bestScores(nBestViews, FLT_MAX);
				std::vector<int> bestViews(nBestViews, -1);
				// get diff for each view
				for (uint v = 0; v < stackSize; ++v) {
					DiffCount dc = thrust::transform_reduce(
						policy,
						thrust::make_counting_iterator<uint>(0),
						thrust::make_counting_iterator<uint>(nPixels),
						[distMap, finalPyr, d_texStacks, stackSize, levelCount, v] __device__(uint p) -> DiffCount {
							const float d = distMap[p];
							if (d <= 0.f || d >= 1.f)
								return DiffCount(0.f, 0);
							const uint base = p * stackSize * 3 + v * 3;
							Point3 C(0.f, 0.f, 0.f);
							for (uint l = 0; l < levelCount; ++l) {
								if (l != levelCount -1) continue;
								const float* s = &d_texStacks[l][base];
								C.x() += s[0];
								C.y() += s[1];
								C.z() += s[2];
							}
							if (C.x() == 0.f && C.y() == 0.f && C.z() == 0.f)
								return DiffCount(0.f, 0);

							const Point3 ref = finalPyr[p].p[levelCount - 1];
							if (ref.x() == 0.f && ref.y() == 0.f && ref.z() == 0.f)
								return DiffCount(0.f, 1);

							const Point3 R = ref - C;
							//const float lDiff = fabsf(ref.x() - C.x());
							const float lDiff = LabNorm(R,R);
							return DiffCount(lDiff,1);
						},
						DiffCount(0.f, 0),
						DiffCountPlus());
					// get number of pixel non null on per group 
					int countP = thrust::count_if(
						policy,
						thrust::counting_iterator<uint>(0),
						thrust::counting_iterator<uint>(nPixels),
						[tagsMap, d_texStacks, tag, visibilityScores, stackSize, v, levelCount] __device__ (uint p) {
							if (tagsMap[p] != tag)
								return false;
							const uint idx = p * stackSize * 3 + v * 3;
							const Point3 c(
								d_texStacks[levelCount - 1][idx + 0],
								d_texStacks[levelCount - 1][idx + 1],
								d_texStacks[levelCount - 1][idx + 2]
							);
							return ( !(c.x() == 0.f && c.y() == 0.f && c.z() == 0.f) && visibilityScores[p * stackSize + v] != 0);
						});
					// if the support is too low or the mean diff is too high, skip this view
					const int minSupport = max(5, hTagAreaP[t] / 10);
					if (dc.count < minSupport || countP < hTagAreaP[t] / 10)
						continue;
					// insert the view in the n-best if better
					const float score = dc.diff / (float)dc.count;
					for (uint i = 0; i < nBestViews; ++i) {
						if (score < bestScores[i]) {
							for (uint k = nBestViews - 1; k > i; --k) {
								bestScores[k] = bestScores[k - 1];
								bestViews[k] = bestViews[k - 1];
							}
							bestScores[i] = score;
							bestViews[i] = (int)v;
							break;
						}
					}
				}
				// apply patch from the best views one by one
				{
					cudaMemsetAsync(patchDist, 0, nPixels * sizeof(float),stream);
					maxDist = maxDist1;
					for (uint i = 0; i < nBestViews; ++i) {
						int bIdx = bestViews[i];
						if (bIdx == -1)
							continue;
						// compute distance to empty in finalPyr and update distMap for the given tag
						{
							cudaMemsetAsync(distMap, 0, nPixels * sizeof(float),stream);
							thrust::transform(policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), seedMap1_d, 
								[finalPyr, tagsMap, tag, levelCount, texSize] __device__(uint idx) {
									  int x = idx % texSize.x();
									  int y = idx / texSize.x();
									  if (tagsMap[idx] == tag && finalPyr[idx].p[levelCount - 1].x() == 0.f)
										return Seed(x, y);
									  else
										return Seed(-1, -1);
								});
							int maxJump = max(texSize.x(), texSize.y());
							for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
								jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(
									seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
								Seed* temp = seedMap1_d;
								seedMap1_d = seedMap2_d;
								seedMap2_d = temp;
							}
							computeTruncDistance<<<gridSize, blockSize, 0, stream>>>(
								seedMap1_d, distMap, texSize.x(), texSize.y(), maxDist);
							CUDA_CHECK_LAST_ERROR;
						}
						// compute distance to empty in the given img
						{	
							float maxPatchDist = maxDist2;
							cudaMemsetAsync(patchDist, 0, nPixels * sizeof(float),stream);
							thrust::transform(policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), seedMap1_d, 
								[d_texStacks, distMap, bIdx, levelCount, stackSize, texSize] __device__(uint idx) {
									  int x = idx % texSize.x();
									  int y = idx / texSize.x();
									  if (distMap[idx] >= 1.f) 
										  return Seed(-1, -1);
									  const uint base = idx * stackSize * 3 + bIdx * 3;
									  if (d_texStacks[levelCount - 1][base] == 0.f)
										return Seed(x, y);
									  else
										return Seed(-1, -1);
								});
							int maxJump = max(texSize.x(), texSize.y());
							for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
								jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(
									seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
								Seed* temp = seedMap1_d;
								seedMap1_d = seedMap2_d;
								seedMap2_d = temp;
							}
							computeTruncDistance<<<gridSize, blockSize, 0, stream>>>(
								seedMap1_d, patchDist, texSize.x(), texSize.y(), maxPatchDist);
							CUDA_CHECK_LAST_ERROR;
						}
						// appply the patch using distMap as blending factor
						thrust::transform(policy,
							thrust::make_counting_iterator<uint>(0),
							thrust::make_counting_iterator<uint>(nPixels),
							finalPyr,
							[distMap, patchDist, finalPyr, d_texStacks, stackSize, levelCount, bIdx, a, b] __device__(uint p) {
								PyrPix fin = finalPyr[p];
								if (distMap[p] >= 1)
									return fin;
								PyrPix res = PyrPix();
								const float d = distMap[p];
								const float pd = patchDist[p];
								const uint base = p * stackSize * 3 + bIdx * 3;
								for (uint l = 0; l < levelCount; ++l) {
									if (l != levelCount - 1) continue;
									const float* s = &d_texStacks[l][base];
									res.p[l].x() = s[0];
									res.p[l].y() = s[1];
									res.p[l].z() = s[2];
								}
								if (res.p[levelCount - 1].x() == 0.f) {
								    res.p[levelCount - 1] = Point3(70, - 20, -20);
									//return res;
									return fin;
								   }
								if (fin.p[levelCount - 1].x() == 0.f) {
									return res;
								}
								for (uint l = 0; l < levelCount; l++) {
									float w;
									if ( d < a) 
										w = d;
									else if ( d < b)
									    if (d < 1 - pd) 
											w = (d * (b - d) + (1 - pd) * (d - a)) / (b - a) ;
										else
											w = d;
								    else
										w = max(d, 1 - pd);
									//float w = max(d, 1-pd) + d*(1-pd) + d;
									//w = 1 - pd;
									//float w = d;
									if (l != levelCount -1 ) w = 1;
									res.p[l] = fin.p[l] * w + res.p[l] * (1 - w);
								}
								return res;
							}); 
						}
					}
				}
			#endif
		}
	} 
	#endif 	

	// fill remaining holes in color level using best views

	#if 1
	#if 1
	// remake bestViewIdxMap after remmoval in the stack 
	// get the best view index for each pixel
	thrust::fill(policy, bestViewIdxMap, bestViewIdxMap + nPixels, -1);
	thrust::transform(policy, first, last, bestViewIdxMap, [ d_texStacks, visibilityScores, levelCount, stackSize] __device__(uint idx) {
		int bestView = -1;
		float bestScore = -1.f;
		float* pixStack = &d_texStacks[levelCount - 1][idx * stackSize * 3];
		for (uint i = 0; i < stackSize; i++) {
			Point3 color = Point3(pixStack[i * 3], pixStack[i * 3 + 1], pixStack[i * 3 + 2]);
			if (color.x() == 0.f && color.y() == 0.f && color.z() == 0.f)
				continue;
			float score = visibilityScores[idx * stackSize + i];
			if (score > bestScore) {
				bestScore = score;
				bestView = i;
			}
		}
		return bestView;
	});
	#endif

	// get the most frequent best view index
	#if 1
	mostFrequentOrder.clear();
	nUnique = 0;
	{
		thrust::device_vector<int> dSorted(bestViewIdxMap, bestViewIdxMap + nPixels);
		thrust::sort(thrust::device, dSorted.begin(), dSorted.end());
		thrust::device_vector<int> dUniqueVals(nPixels);
		thrust::device_vector<int> dCounts(nPixels);
		auto new_end = thrust::reduce_by_key(
			thrust::device,
			dSorted.begin(), dSorted.end(),
			thrust::make_constant_iterator(1),
			dUniqueVals.begin(),
			dCounts.begin()
		);

		nUnique = static_cast<int>(new_end.first - dUniqueVals.begin());
		std::vector<int> hUniqueVals(nUnique);
		std::vector<int> hCounts(nUnique);
		mostFrequentOrder.resize(nUnique);

		thrust::copy(dUniqueVals.begin(), dUniqueVals.begin() + nUnique, hUniqueVals.begin());
		thrust::copy(dCounts.begin(), dCounts.begin() + nUnique, hCounts.begin());
		std::vector<std::pair<int, int>> valCountPairs(nUnique);
		for (int i = 0; i < nUnique; i++) {
			valCountPairs[i] = { hUniqueVals[i], hCounts[i] };
		}
		std::sort(valCountPairs.begin(), valCountPairs.end(),
			[](const std::pair<int, int>& a, const std::pair<int, int>& b) {
				return a.second > b.second;
			});
		bool hasMinusOne = false;
		for (int i = 0; i < nUnique; i++) {
			if ( valCountPairs[i].first == -1) {
				hasMinusOne = true;
				continue;
			}
			mostFrequentOrder[i] = valCountPairs[i].first;
		}
		if (hasMinusOne) {
			nUnique = hasMinusOne ? nUnique - 1 : nUnique;
			mostFrequentOrder.resize(nUnique);
		}
	}
	#endif
	// save the best view pyramid
	#if 1
	PyrPix* bestViewPyr = ws.bestViewPyr.GetDeviceData();
	cudaMemsetAsync(bestViewPyr, 0, nPixels * sizeof(PyrPix),stream);
	thrust::transform(policy, first, last, bestViewPyr, [d_texStacks, bestViewIdxMap, levelCount, stackSize] __device__(uint idx) {
		int bestView = bestViewIdxMap[idx];
		if (bestView == -1) return PyrPix();
		PyrPix p;
		for (uint l = 0; l < levelCount; l++) {
			float* pixStack = &d_texStacks[l][idx * stackSize * 3 + (uint)bestView * 3];
			p.p[l].x() = pixStack[0];
			p.p[l].y() = pixStack[1];
			p.p[l].z() = pixStack[2];
		}
		return p;
	});
	#endif
	// repeat the frequency-ordered view blending to cover holes in the color level
	#if 1
	{
		float* distanceMap = ws.distanceMap.GetDeviceData();
		cudaMemsetAsync(distanceMap, 0, nPixels * sizeof(float),stream);
		cudaMemsetAsync(distMap, 0, nPixels * sizeof(float),stream);
		for (int  i = 0; i < nUnique; i++) {
			int viewIdx = mostFrequentOrder[i];
			// compute distance to empty in finalPyr
			float blendDist = maxDist1;
			// JFA pass (a): seeds = empty pixels in finalPyr that are the best view for this slot.
			{
				cudaMemsetAsync(seedMap1_d, 0, nPixels * sizeof(Seed),stream);
				cudaMemsetAsync(seedMap2_d, 0, nPixels * sizeof(Seed),stream);
				// init seeds
				thrust::transform(policy, first, last, seedMap1_d, [finalPyr, bestViewIdxMap, mask1, viewIdx, texSize, levelCount] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Point3 C = finalPyr[idx].p[levelCount - 1];
					if (C.x() == 0 && bestViewIdxMap[idx] == viewIdx)
						return Seed(x, y);
					return Seed(-1, -1);	
				});
				int maxJump = max(texSize.x(), texSize.y());
				for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
					jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
					// swap seed maps
					Seed* temp = seedMap1_d;
					seedMap1_d = seedMap2_d;
					seedMap2_d = temp;
				}
				thrust::transform(policy, first, last, distMap, [seedMap1_d, blendDist, texSize] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Seed s = seedMap1_d[idx];
					int sIdx = s.y * texSize.x() + s.x;
					float dx = float(x - s.x);
					float dy = float(y - s.y);
					
					float dist = min(sqrt(dx * dx + dy * dy)/blendDist,1.f);
					return dist;
				});
			}
			// JFA pass (b): seeds = pixels where this view's slot is empty.
			float blendDist2 = maxDist2;
			{
				cudaMemsetAsync(seedMap1_d, 0, nPixels * sizeof(Seed),stream);
				cudaMemsetAsync(seedMap2_d, 0, nPixels * sizeof(Seed),stream);
				// init seeds
				thrust::transform(policy, first, last, seedMap1_d, [viewIdx, d_texStacks, distMap, texSize, stackSize, levelCount] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					float* pixStack = &d_texStacks[levelCount - 1][idx * stackSize * 3];
					if (pixStack[viewIdx * 3] == 0)
						return Seed(x, y);
					return Seed(-1, -1);	
				});
				int maxJump = max(texSize.x(), texSize.y());
				for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
					jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
					// swap seed maps
					Seed* temp = seedMap1_d;
					seedMap1_d = seedMap2_d;
					seedMap2_d = temp;
				}
				thrust::transform(policy, first, last, distanceMap, [seedMap1_d, blendDist2, texSize] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Seed s = seedMap1_d[idx];
					int sIdx = s.y * texSize.x() + s.x;
					float dx = float(x - s.x);
					float dy = float(y - s.y);
					float dist = min(sqrt(dx * dx + dy * dy)/blendDist2,1.f);
					return dist;
				});
			}

			// blend view colour into finalPyr (coarsest level only) using JFA-derived weights:
			//   w  (distMap value)     = finalyPyr hole distance; 
			//   pw (distanceMap value) = view hole distance; 			
			// effective blend factor = max(w, 1-pw)
			thrust::for_each(policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), 
				[distMap, viewIdx, d_texStacks, finalPyr, bestViewIdxMap, blendDist, blendDist2, distanceMap, stackSize, texSize, levelCount, a, b] __device__(uint idx) {
				int x = idx % texSize.x();
				int y = idx / texSize.x();
				float d = distMap[idx];
				float pd = distanceMap[idx];
				PyrPix fin = finalPyr[idx];
				if (d >= 1)
					return;
				Point3 P;
				float* pixStack = &d_texStacks[levelCount - 1][idx * stackSize * 3 + viewIdx * 3];
				P.x() = pixStack[0];
				P.y() = pixStack[1];
				P.z() = pixStack[2];
				if (P.x() == 0)
					return;
				if (fin.p[levelCount - 1].x() == 0) {
					finalPyr[idx].p[levelCount - 1] = P;
					return;
				}
				Point3 res;
				float w;
				if ( d < a) 
					w = d;
				else if ( d < b)
					if (d < 1 - pd) 
						w = (d * (b - d) + (1 - pd) * (d - a)) / (b - a) ;
					else
						w = d;
				else
					w = max(d, 1 - pd);
				res = fin.p[levelCount - 1] * w + P * (1 - w);
				finalPyr[idx].p[levelCount - 1] = res;
			});
		}
	}
	#endif
	#endif

	// differential level fill: fill detail Laplacian levels (0..levelCount-2) using
	// the same frequency-ordered view iteration.  For each view, two JFA passes produce:
	//   distMap     (float) = blend-boundary weight seeded from mask1==0 (empty differential levels) pixels that are best
	//                         for this view .
	//   distanceMap (float) = view-coverage weight seeded from the view's own empty pixels.
	#if 1
	{	
		float* distanceMap = ws.distanceMap.GetDeviceData();
		cudaMemsetAsync(distanceMap, 0, nPixels * sizeof(float),stream);
		uint nEmpty = nPixels;
		for (int  i = 0; i < nUnique; i++) {
			int viewIdx = mostFrequentOrder[i];
			if (nEmpty == 0)
				break;
			// JFA pass (a): seeds = pixels where mask1==0 (empty differntial levels) and this view is best.
			float blendDist = maxDist3;
			{
				cudaMemsetAsync(distMap, 0, nPixels * sizeof(float),stream);
				cudaMemsetAsync(seedMap1_d, 0, nPixels * sizeof(Seed),stream);
				cudaMemsetAsync(seedMap2_d, 0, nPixels * sizeof(Seed),stream);
				// init seeds
				thrust::transform(policy, first, last, seedMap1_d, [mask1, viewIdx, bestViewIdxMap, texSize] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					if (mask1[idx] == 0 && bestViewIdxMap[idx] == viewIdx)
						return Seed(x, y);
					return Seed(-1, -1);	
				});
				int maxJump = max(texSize.x(), texSize.y());
				for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
					jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
					// swap seed maps
					Seed* temp = seedMap1_d;
					seedMap1_d = seedMap2_d;
					seedMap2_d = temp;
				}
				thrust::transform(policy, first, last, distMap, [seedMap1_d, blendDist, texSize] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Seed s = seedMap1_d[idx];
					int sIdx = s.y * texSize.x() + s.x;
					float dx = float(x - s.x);
					float dy = float(y - s.y);
					
					float dist = min(sqrt(dx * dx + dy * dy)/blendDist,1.f);
					return dist;
				});
			}

			// JFA pass (b): seeds = pixels where this view's coarsest slot is empty.
			// Output: distanceMap = float view-coverage weight [0,1].
			float blendDist2 = maxDist3;
			{
				cudaMemsetAsync(seedMap1_d, 0, nPixels * sizeof(Seed),stream);
				cudaMemsetAsync(seedMap2_d, 0, nPixels * sizeof(Seed),stream);
				// init seeds
				thrust::transform(policy, first, last, seedMap1_d, [viewIdx, d_texStacks, texSize, stackSize, levelCount] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					float* pixStack = &d_texStacks[levelCount - 1][idx * stackSize * 3];
					if (pixStack[viewIdx * 3] == 0)
						return Seed(x, y);
					return Seed(-1, -1);	
				});
				int maxJump = max(texSize.x(), texSize.y());
				for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
					jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
					// swap seed maps
					Seed* temp = seedMap1_d;
					seedMap1_d = seedMap2_d;
					seedMap2_d = temp;
				}
				thrust::transform(policy, first, last, distanceMap, [seedMap1_d, blendDist2, texSize] __device__(uint idx) {
					int x = idx % texSize.x();
					int y = idx / texSize.x();
					Seed s = seedMap1_d[idx];
					int sIdx = s.y * texSize.x() + s.x;
					float dx = float(x - s.x);
					float dy = float(y - s.y);
					float dist = min(sqrt(dx * dx + dy * dy)/blendDist2,1.f);
					return dist;
				});
			}
			// blend view colour into detail levels (0..levelCount-2) using JFA weights:
			//   w  (distMap)      = blend-boundary weight from empty detail pixels for this view.
			//   pw (distanceMap)  = view-coverage weight from the view's own uncovered pixels.
			thrust::for_each(policy, thrust::make_counting_iterator<uint>(0), thrust::make_counting_iterator<uint>(nPixels), 
				[mask1, distMap, viewIdx, d_texStacks, finalPyr, bestViewIdxMap, blendDist, blendDist2, distanceMap, stackSize, texSize, levelCount] __device__(uint idx) {
				int x = idx % texSize.x();
				int y = idx / texSize.x();
				float d = distMap[idx];
				float pd = distanceMap[idx];
				PyrPix fin = finalPyr[idx];	
				if (d >= 1)
					return;
					PyrPix P = PyrPix();
				for (uint l = 0; l < levelCount; l++) {
					float* pixStack = &d_texStacks[l][idx * stackSize * 3 + viewIdx * 3];
					P.p[l].x() += pixStack[0];
					P.p[l].y() += pixStack[1];
					P.p[l].z() += pixStack[2];
					}
				if (P.p[levelCount - 1].x() == 0)
					return;
				PyrPix res;

				for (uint l = 0; l < levelCount - 1; l++) {
						float w;
						w = max(d, 1 - pd);
						if (mask1[idx] != 0) {
							res.p[l] = fin.p[l] * w + P.p[l] * (1 - w);
						}
						else 
							res.p[l] = P.p[l];
				}
				res.p[levelCount - 1] = fin.p[levelCount - 1];
				finalPyr[idx] = res;
				mask1[idx] = 1;
			});
		}
	}
	#endif
	// residual fill: compute distMap = JFA distance to remaining empty pixels
	// in finalPyr, then blend finalPyr with savedResult using distMap as the weight so that
	// any pixels that are still empty (distMap≈0) are filled from the saved first-pass result.
	#if 1
	// JFA: seeds = pixels where finalPyr is still empty; output distMap = truncated distance.
	{
		float maxDist = max(maxDist1/2.5f,3.f);
		cudaMemsetAsync(distMap, 0, nPixels * sizeof(float),stream);
		thrust::transform(policy, first, last, seedMap1_d, [finalPyr, texSize, levelCount] __device__(uint idx) {
			int x = idx % texSize.x();
			int y = idx / texSize.x();
			Point3 C = finalPyr[idx].p[levelCount - 1];
			if(C.x() == 0 && C.y() == 0 && C.z() == 0)
				return Seed(x, y);
			return Seed(-1, -1);	
		});
		CUDA_CHECK_LAST_ERROR;
		int maxJump = max(texSize.x(), texSize.y());
		for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
			jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(
				seedMap1_d, seedMap2_d, texSize.x(), texSize.y(), jump);
			Seed* temp = seedMap1_d;
			seedMap1_d = seedMap2_d;
			seedMap2_d = temp;
		}
		computeTruncDistance<<<gridSize, blockSize, 0, stream>>>(
			seedMap1_d, distMap, texSize.x(), texSize.y(), maxDist);
		CUDA_CHECK_LAST_ERROR;
	}
	// blend finalPyr with savedResult using distMap as weight	
	thrust::transform(policy, first, last, finalPyr, [distMap, savedResult, finalPyr, stackSize, levelCount] __device__(uint p) {
		PyrPix fin = finalPyr[p];
		const float w = fminf(fmaxf(distMap[p], 0.f), 1.f);
		if (w >= 1.f)
			return fin;
		PyrPix best = savedResult[p];
		if (w == 0.f) 
			return best;
		PyrPix res;
		for (uint l = 0; l < levelCount; l++) {
			float d = w;
			if(l != levelCount - 1)
				d = min(3 * d / (float)l, 1.f);
			res.p[l] = fin.p[l] * d + best.p[l] * (1 - d);
		}
		return res;
	});
	#endif
	#endif
	// write finalPyr into slot 0 of each pyramid level stack 	
	PyrPix* srcImage = finalPyr;
	for (uint l = 0; l < levelCount; ++l) {
		float* dstStack = h_texStacks[l];
		thrust::for_each(
			policy,
			thrust::make_counting_iterator<uint>(0),
			thrust::make_counting_iterator<uint>(nPixels),
			[dstStack, srcImage, stackSize, l] __device__ (uint p) {
				const uint srcIdx = p;
				const uint dstIdx = p * stackSize * 3; // slot 0 of the stack
				Point3 C = srcImage[srcIdx].p[l];
				dstStack[dstIdx] = C.x();
				dstStack[dstIdx + 1] = C.y();
				dstStack[dstIdx + 2] = C.z();					
			});
	}	
}	

void CollapseStacks(std::vector<CUDA::TDeviceMat<float>>& pyramidTex, float* texAverage, float* bufferCount, float* visibilityScores, int* idxStacks, Point2i texSize, unsigned levelCount, uint stackSize, cudaStream_t stream) {
	dim3 blockSize(16, 16);
	dim3 gridSize((texSize.x() + blockSize.x - 1) / blockSize.x, (texSize.y() + blockSize.y - 1) / blockSize.y);
	// prepare stack pointers
	float** d_texStacks;
	std::vector<float*> h_texStacks(levelCount);
	for (int i = 0; i < levelCount; i++) {
		h_texStacks[i] = pyramidTex[i].GetDeviceData();
	}
	cudaMalloc((void**)&d_texStacks, sizeof(float*) * levelCount);
	cudaMemcpy(d_texStacks, h_texStacks.data(), sizeof(float*) * levelCount, cudaMemcpyHostToDevice);

	// allocate distance map
	float* distanceMap;
	uint texW = texSize.x();
	uint texH = texSize.y();
	uint nPixels = texW * texH;
	cudaMallocAsync((void**)&distanceMap, nPixels*sizeof(float),stream);
	cudaMemsetAsync(distanceMap, 0, nPixels*sizeof(float),stream);
	// make distance map to best view change using jump flooding
	{
		// set ground level of the idxStacks to best view idx
		SetBestViewIdx<<<gridSize, blockSize, 0, stream>>>(idxStacks, visibilityScores, texSize, stackSize);
		uint* d_variationMap;
		cudaMalloc((void**)&d_variationMap, nPixels * sizeof(uint));
		cudaMemsetAsync(d_variationMap, 0, nPixels * sizeof(uint), stream);
		MakeIdxVariationMap<<<gridSize, blockSize, 0, stream>>>(idxStacks, d_variationMap, texSize, stackSize, 0);
		Seed* seedMap1_d, * seedMap2_d;
		cudaMalloc((void**)&seedMap1_d, nPixels * sizeof(Seed));
		cudaMalloc((void**)&seedMap2_d, nPixels * sizeof(Seed));
		dim3 blockSize(16, 16);
		dim3 gridSize((texSize.x() + blockSize.x - 1) / blockSize.x, (texSize.y() + blockSize.y - 1) / blockSize.y);
		initSeeds<<<gridSize, blockSize, 0, stream>>>(d_variationMap, seedMap1_d, texW, texH);
		int maxJump = max(texSize.x(), texSize.y());
		for (int jump = maxJump / 2; jump >= 1; jump /= 2) {
			jumpFloodPass<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, seedMap2_d, texW, texH, jump);
			// Swap seed maps
			Seed* temp = seedMap1_d;
			seedMap1_d = seedMap2_d;
			seedMap2_d = temp;
		}
		computeDistance<<<gridSize, blockSize, 0, stream>>>(seedMap1_d, distanceMap, texW, texH);
		CUDA_CHECK_LAST_ERROR;
		cudaFree(seedMap1_d);
		cudaFree(seedMap2_d);
		cudaFree(d_variationMap);
	}

	// compose final value for each pyramid stage using stacked samples
	CollapseStacksKernel<<<gridSize, blockSize, 0, stream>>>(d_texStacks, texAverage, bufferCount, visibilityScores, distanceMap, texSize, stackSize, levelCount);
	CUDA_CHECK_LAST_ERROR;
	// set visibility texture map to zero where color is (0,0,0) to use it as mask later
	SetVisibilityKernel<<<gridSize, blockSize, 0, stream>>>(d_texStacks, visibilityScores, texSize, stackSize, levelCount);
	CUDA_CHECK_LAST_ERROR;
	cudaFree(d_texStacks);
	// distanceMap was allocated with cudaMallocAsync — release with the matching async free
	cudaFreeAsync(distanceMap, stream);
}
/*----------------------------------------------------------------*/

}// CUDA::curast
}// CUDA
}// MVS
