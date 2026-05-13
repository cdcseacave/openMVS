#include <thrust/execution_policy.h>
#include <thrust/system/cuda/execution_policy.h>
#include <cuda_runtime.h>
#include <device_atomic_functions.h>  
#include <cuda_fp16.h>
#include <thrust/device_ptr.h>
#include <thrust/fill.h>
#include <thrust/transform_reduce.h>
#include <thrust/pair.h>
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

// edge function
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
struct Seed
{
	int x;
	int y;
};
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
