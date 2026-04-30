/*
* PatchMatchCUDA.cu
*
* Copyright (c) 2014-2021 SEACAVE
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

#include "PatchMatchCUDA.inl"

// static max supported views
#define MAX_VIEWS 32

// samples used to perform views selection
#define NUM_SAMPLES 32

// patch window radius
#define nSizeHalfWindow 4

// patch stepping
#define nSizeStep 2

// F2: launch-bounds tuning. The default (PATCHMATCHCUDA_LB_256_2 = 1)
// uses 256 threads/block with 2 resident blocks/SM, which on RTX-class
// GPUs lets the warp scheduler interleave warps across blocks while
// one is stalled on tex2D latency. Set to 0 to fall back to the
// historical 512 threads/block, 1 block/SM configuration.
#ifndef PATCHMATCHCUDA_LB_256_2
#define PATCHMATCHCUDA_LB_256_2 1
#endif

#if PATCHMATCHCUDA_LB_256_2
#define PATCHMATCHCUDA_BLOCK_H_DIV 4   // BLOCK_H = BLOCK_W / 4 = 8
#define PATCHMATCHCUDA_LAUNCH_BOUNDS __launch_bounds__(256, 2)
#else
#define PATCHMATCHCUDA_BLOCK_H_DIV 2   // BLOCK_H = BLOCK_W / 2 = 16
#define PATCHMATCHCUDA_LAUNCH_BOUNDS __launch_bounds__(512, 1)
#endif


namespace MVS {

namespace CUDA {

#define ImagePixels cudaTextureObject_t
#define RandState curandState

// A2: cameras live in __constant__ memory. CUDA::Camera is 72 B; with
// MAX_VIEWS=32 source + 1 reference = 33 entries, the array is ~2.4 KB --
// well within the 64 KB constant-memory budget. Constant memory has a
// dedicated 8 KB per-SM cache that broadcasts to all threads in a warp on
// a single read, eliminating the per-pixel L1/global-memory loads through
// the previous `cameras` pointer parameter.
__constant__ Camera g_cameras[MAX_VIEWS + 1];

// set/check a bit
__device__ constexpr void SetBit(unsigned& input, unsigned i) {
	input |= (1u << i);
}
__device__ constexpr int IsBitSet(unsigned input, unsigned i) {
	return (input >> i) & 1u;
}

// C2: read-only-cache loaders for the planes[] array. Within the
// black/red checkerboard pass every neighbour offset in `dirs` and in
// `neighborPositions` has odd Manhattan parity from the current pixel,
// so the cells we read are guaranteed not to be written in this kernel
// launch -- the __ldg() read-only contract holds and L1 bandwidth is
// freed up for the texture work.
__device__ __forceinline__ Point4 LoadPlaneLDG(const Point4* p) {
	const float* f = reinterpret_cast<const float*>(p);
	Point4 r;
	r.x() = __ldg(f + 0);
	r.y() = __ldg(f + 1);
	r.z() = __ldg(f + 2);
	r.w() = __ldg(f + 3);
	return r;
}
__device__ __forceinline__ float LoadPlaneWLDG(const Point4* p) {
	return __ldg(reinterpret_cast<const float*>(p) + 3);
}

// sort the given values array using bubble sort algorithm
__device__ inline void Sort(const float* values, float* sortedValues, int n) {
	for (int i = 0; i < n; ++i)
		sortedValues[i] = values[i];
	do {
		int newn = 0;
		for (int i = 1; i < n; ++i) {
			if (sortedValues[i-1] > sortedValues[i]) {
				Swap(sortedValues[i-1], sortedValues[i]);
				newn = i;
			}
		}
		n = newn;
	} while(n);
}

// find the index of the minimum value in the given values array
__device__ inline int FindMinIndex(const float* values, const int n) {
	float minValue = values[0];
	int minValueIdx = 0;
	for (int i = 1; i < n; ++i) {
		if (minValue > values[i]) {
			minValue = values[i];
			minValueIdx = i;
		}
	}
	return minValueIdx;
}

// convert Probability Density Function (PDF) to Cumulative Distribution Function (CDF)
__device__ inline void PDF2CDF(float* probs, const int numProbs) {
	float probSum = 0.f;
	for (int i = 0; i < numProbs; ++i)
		probSum += probs[i];
	const float invProbSum = 1.f / probSum;
	float sumProb = 0.f;
	for (int i = 0; i < numProbs-1; ++i) {
		sumProb += probs[i] * invProbSum;
		probs[i] = sumProb;
	}
	probs[numProbs-1] = 1.f;
}
/*----------------------------------------------------------------*/


// generate a random normal (Marsaglia's method on the unit sphere).
// Algebraically unit-length: |n|^2 = 4q1^2(1-s) + 4q2^2(1-s) + (1-2s)^2 = 1.
// Float-32 round-off contributes <=1 ULP from the single sqrtf(1-s); no
// .normalized() defensive call is required.
__device__ inline Point3 GenerateRandomNormal(const CUDA::Camera& camera, const Point2i& p, RandState* randState)
{
	float q1, q2, s;
	do {
		q1 = 2.f * curand_uniform(randState) - 1.f;
		q2 = 2.f * curand_uniform(randState) - 1.f;
		s = q1 * q1 + q2 * q2;
	} while (s >= 1.f);
	const float sq = sqrtf(1.f - s);
	const Point3 normal(
		2.f * q1 * sq,
		2.f * q2 * sq,
		1.f - 2.f * s);

	const Point3 viewDirection = camera.model.ViewDirection(p);
	return normal.dot(viewDirection) > 0.f ? Point3(-normal) : normal;
}

// randomly perturb a normal
// Algorithmically unit-preserving: Rodrigues rotation around a Marsaglia-unit
// axis by a random small angle. For unit |axis|=1 and unit |normal|=1, the
// rotation v' = v cosθ + (axis x v) sinθ + axis (axis . v)(1 - cosθ) preserves
// |v'|=1 exactly in math; float32 round-off contributes ~2-3 ULP, the same
// noise floor as the input -- so no .normalized() defensive call is required.
// Also halves the trig cost (1 sincosf vs 3 sin + 3 cos) and replaces the
// 9-element non-orthogonal matrix product with one cross + one dot.
__device__ inline Point3 GeneratePerturbedNormal(const CUDA::Camera& camera, const Point2i& p, const Point3& normal, RandState* randState, const float perturbation)
{
	// Random unit-length axis on the sphere (Marsaglia's method, exact in math).
	float q1, q2, s;
	do {
		q1 = 2.f * curand_uniform(randState) - 1.f;
		q2 = 2.f * curand_uniform(randState) - 1.f;
		s = q1 * q1 + q2 * q2;
	} while (s >= 1.f);
	const float sq = sqrtf(1.f - s);
	const Point3 axis(2.f * q1 * sq, 2.f * q2 * sq, 1.f - 2.f * s);

	// Random angle in [-perturbation/2, +perturbation/2].
	const float theta = (curand_uniform(randState) - 0.5f) * perturbation;
	float sinT, cosT;
	__sincosf(theta, &sinT, &cosT);

	// Rodrigues' rotation formula.
	const float aDotN = axis.dot(normal);
	const Point3 axCrossN = axis.cross(normal);
	const Point3 normalPerturbed = normal * cosT + axCrossN * sinT + axis * (aDotN * (1.f - cosT));

	// Keep the perturbed normal in the camera-facing half-space.
	const Point3 viewDirection = camera.model.ViewDirection(p);
	if (normalPerturbed.dot(viewDirection) >= 0.f)
		return normal;
	return normalPerturbed;
}

// randomly perturb a normal
__device__ inline float GeneratePerturbedDepth(float depth, RandState* randState, const float perturbation, const PatchMatch::Params& params)
{
	const float depthMinPerturbed = (1.f - perturbation) * depth;
	const float depthMaxPerturbed = (1.f + perturbation) * depth;
	float depthPerturbed;
	do {
		depthPerturbed = curand_uniform(randState) * (depthMaxPerturbed - depthMinPerturbed) + depthMinPerturbed;
	} while (depthPerturbed < params.fDepthMin && depthPerturbed > params.fDepthMax);
	return depthPerturbed;
}

// interpolate given pixel's estimate to the current position
__device__ inline float InterpolatePixel(const CUDA::Camera& camera, const Point2i& p, const Point2i& np, float depth, const Point3& normal, const PatchMatch::Params& params)
{
	float depthNew;
	if (p.x() == np.x()) {
		const float nx1 = (p.y() - camera.model.p.y()) / camera.model.f.y();
		const float denom = normal.z() + nx1 * normal.y();
		if (abs(denom) < FLT_EPSILON)
			return depth;
		const float x1 = (np.y() - camera.model.p.y()) / camera.model.f.y();
		const float nom = depth * (normal.z() + x1 * normal.y());
		depthNew = nom / denom;
	} else if (p.y() == np.y()) {
		const float nx1 = (p.x() - camera.model.p.x()) / camera.model.f.x();
		const float denom = normal.z() + nx1 * normal.x();
		if (abs(denom) < FLT_EPSILON)
			return depth;
		const float x1 = (np.x() - camera.model.p.x()) / camera.model.f.x();
		const float nom = depth * (normal.z() + x1 * normal.x());
		depthNew = nom / denom;
	} else {
		const float planeD = normal.dot(camera.model.TransformPointI2C(np.cast<float>(), depth));
		depthNew = planeD / normal.dot(camera.model.TransformPointI2C(p.cast<float>()));
	}
	return (depthNew >= params.fDepthMin && depthNew <= params.fDepthMax) ? depthNew : depth;
}

// compute normal to the surface given the 4 neighbors
__device__ inline Point3 ComputeDepthGradient(const LinearCameraModel& model, float depth, const Point2i& pos, const Point4& ndepth) {
	constexpr float2 nposg[4] = {{0,-1}, {0,1}, {-1,0}, {1,0}};
	Point2 dg(0,0);
	// add neighbor depths at the gradient locations
	for (int i=0; i<4; ++i)
		dg += Point2(nposg[i].x,nposg[i].y) * (ndepth[i] - depth);
	// compute depth gradient
	const Point2 d = dg*0.5f;
	// compute normal from depth gradient
	// NOTE: must remain unit-length: InterpolatePixel and ComputeHomography
	// have scale-dependent denom guards (FLT_EPSILON, 1e-6) that fire
	// spuriously on unnormalized normals whose magnitude tracks |depth|.
	return Point3(
		model.f.x()*d.x(),
		model.f.y()*d.y(),
		(model.p.x()-pos.x())*d.x()+(model.p.y()-pos.y())*d.y()-depth).normalized();
}

// compose tho homography matrix that transforms a point from reference to source camera through the given plane
__device__ inline Matrix3 ComputeHomography(const CUDA::Camera& refCamera, const CUDA::Camera& trgCamera, const Point2& p, const Point4& plane)
{
	const Point3 X = refCamera.model.TransformPointI2C(p, plane.w());
	const Point3 normal = plane.topLeftCorner<3,1>();
	// Guard against plane passing through (or near) the reference camera center:
	// normal.dot(X) -> 0 makes t infinite and the resulting H NaN, which would
	// silently corrupt the patch walk (NaN comparisons fail-open in the bounds check).
	const float denom = normal.dot(X);
	const float safeDenom = fabsf(denom) < 1e-6f ? copysignf(1e-6f, denom) : denom;
	const Point3 t = (refCamera.pose.C - trgCamera.pose.C) / safeDenom;
	const Matrix3 H = trgCamera.pose.R * (refCamera.pose.R.transpose() + t*normal.transpose());
	return trgCamera.model.K() * H * refCamera.model.K().inverse();
}

// weight a neighbor texel based on color similarity and distance to the center texel
__device__ inline float ComputeBilateralWeight(int xDist, int yDist, float pix, float centerPix)
{
	// Spatial Gaussian for the 5x5 patch (xDist,yDist in {-4,-2,0,2,4}; sigmaSpatial = -1/18)
	// precomputed: exp(-(dx*dx + dy*dy) / 18); indexed by (xDist+4)/2 and (yDist+4)/2.
	static constexpr float spatialLUT[5][5] = {
		{ 0.169013f, 0.329193f, 0.411112f, 0.329193f, 0.169013f },
		{ 0.329193f, 0.641180f, 0.800737f, 0.641180f, 0.329193f },
		{ 0.411112f, 0.800737f, 1.000000f, 0.800737f, 0.411112f },
		{ 0.329193f, 0.641180f, 0.800737f, 0.641180f, 0.329193f },
		{ 0.169013f, 0.329193f, 0.411112f, 0.329193f, 0.169013f },
	};
	constexpr float sigmaColor = -1.f / (2.f * 25.f/255.f*25.f/255.f);
	const float spatialW = spatialLUT[(yDist + nSizeHalfWindow) / nSizeStep][(xDist + nSizeHalfWindow) / nSizeStep];
	const float colorDistSq = Square(pix - centerPix);
	return spatialW * __expf(colorDistSq * sigmaColor);
}

// compute the geometric consistency weight
__device__ inline float GeometricConsistencyWeight(const ImagePixels depthImage, const CUDA::Camera& refCamera, const CUDA::Camera& trgCamera, const Point4& plane, const Point2i& p)
{
	if (depthImage == NULL)
		return 0.f;
	constexpr float maxDist = 4.f;
	const Point3 forwardPoint = refCamera.TransformPointI2W(p.cast<float>(), plane.w());
	const Point2 trgPt = trgCamera.TransformPointW2I(forwardPoint);
	const float trgDepth = tex2D<float>(depthImage, trgPt.x() + 0.5f, trgPt.y() + 0.5f);
	if (trgDepth == 0.f)
		return maxDist;
	const Point3 trgX = trgCamera.TransformPointI2W(trgPt, trgDepth);
	const Point2 backwardPoint = refCamera.TransformPointW2I(trgX);
	const Point2 diff = p.cast<float>() - backwardPoint;
	const float dist = diff.norm();
	return min(maxDist, sqrt(dist*(dist+2.f)));
}

// number of samples in the (2*halfWin/step + 1)^2 reference patch
#define N_PATCH_SAMPLES ((2 * nSizeHalfWindow / nSizeStep + 1) * (2 * nSizeHalfWindow / nSizeStep + 1))

// Per-pixel reference-patch state. Depends only on the reference image at p,
// so it is invariant across source views and plane hypotheses. Compute once
// at the top of ProcessPixel / InitializePixelScore and reuse for every
// ScorePlane call (eliminates ~13*nNumViews redundant ref tex2D fetches and
// 25*13*nNumViews bilateral-weight evaluations per pixel).
struct RefPatchCache {
	float weight[N_PATCH_SAMPLES];        // bilateral weight per patch sample
	float weightRefPix[N_PATCH_SAMPLES];  // weight * refPix per sample
	float sumRef;                          // Σ weight * refPix
	float bilateralWeightSum;              // Σ weight
	float varRef;                          // sumRefRef*Σw - sumRef^2
};

__device__ inline void ComputeRefPatchCache(const ImagePixels refImage, const Point2i& p, RefPatchCache& cache)
{
	const float refCenterPix = tex2D<float>(refImage, p.x() + 0.5f, p.y() + 0.5f);
	float sumRef = 0.f, sumRefRef = 0.f, bws = 0.f;
	int idx = 0;
	for (int i = -nSizeHalfWindow; i <= nSizeHalfWindow; i += nSizeStep) {
		for (int j = -nSizeHalfWindow; j <= nSizeHalfWindow; j += nSizeStep) {
			const float refPix = tex2D<float>(refImage, p.x() + j + 0.5f, p.y() + i + 0.5f);
			const float w = ComputeBilateralWeight(j, i, refPix, refCenterPix);
			const float wRef = w * refPix;
			cache.weight[idx] = w;
			cache.weightRefPix[idx] = wRef;
			sumRef += wRef;
			sumRefRef += wRef * refPix;
			bws += w;
			++idx;
		}
	}
	cache.sumRef = sumRef;
	cache.bilateralWeightSum = bws;
	cache.varRef = sumRefRef * bws - sumRef * sumRef;
}

// compute photometric score using weighted ZNCC; uses precomputed reference cache
__device__ float ScorePlane(const RefPatchCache& cache, const CUDA::Camera& refCamera, const ImagePixels trgImage, const CUDA::Camera& trgCamera, const Point2i& p, const Point4& plane, const float lowDepth, const PatchMatch::Params& params)
{
	constexpr float maxCost = 1.2f;

	Matrix3 H = ComputeHomography(refCamera, trgCamera, p.cast<float>(), plane);
	// F3 (#1): inline hnormalized() with __fdividef so each projection is
	// one RCP + two FMAs instead of two IEEE-compliant divisions. The +0.5
	// tex2D pixel-center bias rides into the FMA. Saves ~25 divisions per
	// ScorePlane call (the hottest inner loop in the kernel).
	{
		const Point3 ptH = H * p.cast<float>().homogeneous();
		const float invZ = __fdividef(1.f, ptH.z());
		const float ptX = ptH.x() * invZ;
		const float ptY = ptH.y() * invZ;
		if (ptX >= trgCamera.size.x() || ptX < 0.f || ptY >= trgCamera.size.y() || ptY < 0.f)
			return maxCost;
	}
	Point3 X = H * Point2(p.x()-nSizeHalfWindow, p.y()-nSizeHalfWindow).homogeneous();
	Point3 baseX(X);
	H *= float(nSizeStep);

	float sumTrg = 0.f, sumTrgTrg = 0.f, sumRefTrg = 0.f;
	int idx = 0;
	for (int i = -nSizeHalfWindow; i <= nSizeHalfWindow; i += nSizeStep) {
		for (int j = -nSizeHalfWindow; j <= nSizeHalfWindow; j += nSizeStep) {
			const float invZ = __fdividef(1.f, X.z());
			const float trgPx = X.x() * invZ + 0.5f;
			const float trgPy = X.y() * invZ + 0.5f;
			const float trgPix = tex2D<float>(trgImage, trgPx, trgPy);
			const float w = cache.weight[idx];
			const float wTrg = w * trgPix;
			sumTrg += wTrg;
			sumTrgTrg += wTrg * trgPix;
			sumRefTrg += cache.weightRefPix[idx] * trgPix;
			++idx;
			X += H.col(0);
		}
		baseX += H.col(1);
		X = baseX;
	}

	if (lowDepth <= 0 && cache.varRef < 1e-8f)
		return maxCost;
	const float varTrg = sumTrgTrg * cache.bilateralWeightSum - sumTrg * sumTrg;
	const float varRefTrg = cache.varRef * varTrg;
	if (varRefTrg < 1e-16f)
		return maxCost;
	const float covarTrgRef = sumRefTrg * cache.bilateralWeightSum - cache.sumRef * sumTrg;
	float ncc = 1.f - covarTrgRef * rsqrtf(varRefTrg);

	// apply depth prior weight based on patch textureless
	if (lowDepth > 0) {
		const float depth(plane.w());
		const float deltaDepth(min((abs(lowDepth-depth) / lowDepth), 0.5f));
		constexpr float smoothSigmaDepth(-1.f / (1.f * 0.02f)); // 0.12: patch texture variance below 0.02 (0.12^2) is considered texture-less
		const float factorDeltaDepth(__expf(cache.varRef * smoothSigmaDepth));
		ncc = (1.f-factorDeltaDepth)*ncc + factorDeltaDepth*deltaDepth;
	}
	return max(0.f, min(2.f, ncc));
}

// compute photometric score for all neighbor images
__device__ inline void MultiViewScorePlane(const RefPatchCache& cache, const ImagePixels* images, const ImagePixels* depthImages, const Point2i& p, const Point4& plane, const float lowDepth, float* costVector, const PatchMatch::Params& params)
{
	for (int imgId = 1; imgId <= params.nNumViews; ++imgId)
		costVector[imgId-1] = ScorePlane(cache, g_cameras[0], images[imgId], g_cameras[imgId], p, plane, lowDepth, params);
	if (params.bGeomConsistency)
		for (int imgId = 0; imgId < params.nNumViews; ++imgId)
			costVector[imgId] += 0.1f * GeometricConsistencyWeight(depthImages[imgId], g_cameras[0], g_cameras[imgId+1], plane, p);
}
// same as above, but interpolate the plane to current pixel position
__device__ inline float MultiViewScoreNeighborPlane(const RefPatchCache& cache, const ImagePixels* images, const ImagePixels* depthImages, const Point2i& p, const Point2i& np, Point4 plane, const float lowDepth, float* costVector, const PatchMatch::Params& params)
{
	plane.w() = InterpolatePixel(g_cameras[0], p, np, plane.w(), plane.topLeftCorner<3,1>(), params);
	MultiViewScorePlane(cache, images, depthImages, p, plane, lowDepth, costVector, params);
	return plane.w();
}

// aggregate photometric score from all images
__device__ inline float AggregateMultiViewScores(const unsigned* viewWeights, const float* costVector, int numViews)
{
	float cost = 0;
	unsigned wsum = 0;
	for (int imgId = 0; imgId < numViews; ++imgId)
		if (viewWeights[imgId]) {
			cost += viewWeights[imgId] * costVector[imgId];
			wsum += viewWeights[imgId];
		}
	return wsum ? cost / float(wsum) : 1.2f;
}

// propagate and refine the plane estimate for the current pixel employing the asymmetric approach described in:
// "Multi-View Stereo with Asymmetric Checkerboard Propagation and Multi-Hypothesis Joint View Selection", 2018
__device__ void ProcessPixel(const ImagePixels* images, const ImagePixels* depthImages, Point4* planes, const float* lowDepths, float* costs, RandState* randStates, unsigned* selectedViews, const Point2i& p, const PatchMatch::Params& params, const int iter)
{
	const int width = g_cameras[0].size.x();
	const int height = g_cameras[0].size.y();
	if (p.x() >= width || p.y() >= height)
		return;
	const int idx = Point2Idx(p, width);
	RandState* randState = &randStates[idx];
	float lowDepth = 0;
	if (params.bLowResProcessed)
		lowDepth = lowDepths[idx];
	// reference-patch state is invariant across views and hypotheses; cache once
	RefPatchCache refCache;
	ComputeRefPatchCache(images[0], p, refCache);

	// adaptive sampling: 0 up-near, 1 down-near, 2 left-near, 3 right-near, 4 up-far, 5 down-far, 6 left-far, 7 right-far
	static constexpr int2 dirs[8][11] = {
		{{ 0,-1},{-1,-2},{ 1,-2},{-2,-3},{ 2,-3},{-3,-4},{ 3,-4}},
		{{ 0, 1},{-1, 2},{ 1, 2},{-2, 3},{ 2, 3},{-3, 4},{ 3, 4}},
		{{-1, 0},{-2,-1},{-2, 1},{-3,-2},{-3, 2},{-4,-3},{-4, 3}},
		{{ 1, 0},{ 2,-1},{ 2, 1},{ 3,-2},{ 3, 2},{ 4,-3},{ 4, 3}},
		{{0,-3},{0,-5},{0,-7},{0,-9},{0,-11},{0,-13},{0,-15},{0,-17},{0,-19},{0,-21},{0,-23}},
		{{0, 3},{0, 5},{0, 7},{0, 9},{0, 11},{0, 13},{0, 15},{0, 17},{0, 19},{0, 21},{0, 23}},
		{{-3,0},{-5,0},{-7,0},{-9,0},{-11,0},{-13,0},{-15,0},{-17,0},{-19,0},{-21,0},{-23,0}},
		{{ 3,0},{ 5,0},{ 7,0},{ 9,0},{ 11,0},{ 13,0},{ 15,0},{ 17,0},{ 19,0},{ 21,0},{ 23,0}}
	};
	static constexpr int numDirs[8] = {7, 7, 7, 7, 11, 11, 11, 11};
	const int neighborPositions[4] = {
		idx - width,
		idx + width,
		idx - 1,
		idx + 1,
	};
	bool valid[8] = {false, false, false, false, false, false, false, false};
	int positions[8];
	float neighborDepths[8];
	float costArray[8][MAX_VIEWS];

	for (int posId=0; posId<8; ++posId) {
		const int2* samples = dirs[posId];
		Point2i bestNx; float bestConf(FLT_MAX);
		for (int dirId=0; dirId<numDirs[posId]; ++dirId) {
			const int2& offset = samples[dirId];
			const Point2i np(p.x()+offset.x, p.y()+offset.y);
			if (!(np.x()>=0 && np.y()>=0 && np.x()<width && np.y()<height))
				continue;
			const int nidx = Point2Idx(np, width);
			const float nconf = costs[nidx];
			if (bestConf > nconf) {
				bestNx = np;
				bestConf = nconf;
			}
		}
		if (bestConf < FLT_MAX) {
			valid[posId] = true;
			positions[posId] = Point2Idx(bestNx, width);
			neighborDepths[posId] = MultiViewScoreNeighborPlane(refCache, images, depthImages, p, bestNx, LoadPlaneLDG(&planes[positions[posId]]), lowDepth, costArray[posId], params);
		}
	}

	// multi-hypothesis view selection
	float viewSelectionPriors[MAX_VIEWS] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
	for (int posId = 0; posId < 4; ++posId) {
		if (valid[posId]) {
			const unsigned selectedView = selectedViews[neighborPositions[posId]];
			for (int j = 0; j < params.nNumViews; ++j)
				viewSelectionPriors[j] += (IsBitSet(selectedView, j) ? 0.9f : 0.1f);
		}
	}
	float samplingProbs[MAX_VIEWS];
	constexpr float thCostBad = 1.2f;
	const float thCost = 0.8f * exp(Square((float)iter) / (-2.f * 4.f*4.f));
	for (int imgId = 0; imgId < params.nNumViews; ++imgId) {
		float sumW = 0;
		unsigned count = 0;
		unsigned countBad = 0;
		for (int posId = 0; posId < 8; posId++) {
			if (valid[posId]) {
				if (costArray[posId][imgId] < thCost) {
					sumW += exp(Square(costArray[posId][imgId]) / (-2.f * 0.3f*0.3f));
					++count;
				} else if (costArray[posId][imgId] > thCostBad) {
					++countBad;
				}
			}
		}
		if (count > 2 && countBad < 3) {
			samplingProbs[imgId] = viewSelectionPriors[imgId] * sumW / count;
		} else if (countBad < 3) {
			samplingProbs[imgId] = viewSelectionPriors[imgId] * exp(Square(thCost) / (-2.f * 0.4f*0.4f));
		} else {
			samplingProbs[imgId] = 0.f;
		}
	}
	PDF2CDF(samplingProbs, params.nNumViews);
	unsigned viewWeights[MAX_VIEWS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	for (int sample = 0; sample < NUM_SAMPLES; ++sample) {
		const float randProb = curand_uniform(randState);
		for (int imgId = 0; imgId < params.nNumViews; ++imgId) {
			if (samplingProbs[imgId] > randProb) {
				++viewWeights[imgId];
				break;
			}
		}
	}

	// propagate best neighbor plane
	Point4& plane = planes[idx];
	float& cost = costs[idx];
	unsigned newSelectedViews = 0;
	for (int imgId = 0; imgId < params.nNumViews; ++imgId)
		if (viewWeights[imgId])
			SetBit(newSelectedViews, imgId);
	float finalCosts[8];
	for (int posId = 0; posId < 8; ++posId)
		finalCosts[posId] = AggregateMultiViewScores(viewWeights, costArray[posId], params.nNumViews);
	const int minCostIdx = FindMinIndex(finalCosts, 8);
	float costVector[MAX_VIEWS];
	MultiViewScorePlane(refCache, images, depthImages, p, plane, lowDepth, costVector, params);
	cost = AggregateMultiViewScores(viewWeights, costVector, params.nNumViews);
	if (finalCosts[minCostIdx] < cost && valid[minCostIdx]) {
		plane = LoadPlaneLDG(&planes[positions[minCostIdx]]);
		plane.w() = neighborDepths[minCostIdx];
		cost = finalCosts[minCostIdx];
		selectedViews[idx] = newSelectedViews;
	}
	const float depth = plane.w();

	// refine estimate
	constexpr float perturbationDepth = 0.005f;
	constexpr float perturbationNormal = 0.01f * (float)M_PI;
	const float depthPerturbed = GeneratePerturbedDepth(depth, randState, perturbationDepth, params);
	const Point3 perturbedNormal = GeneratePerturbedNormal(g_cameras[0], p, plane.topLeftCorner<3,1>(), randState, perturbationNormal);
	const Point3 normalRand = GenerateRandomNormal(g_cameras[0], p, randState);
	int numValidPlanes = 3;
	Point3 surfaceNormal;
	if (valid[0] && valid[1] && valid[2] && valid[3]) {
		// estimate normal from surrounding surface
		const Point4 ndepths(
			LoadPlaneWLDG(&planes[neighborPositions[0]]),
			LoadPlaneWLDG(&planes[neighborPositions[1]]),
			LoadPlaneWLDG(&planes[neighborPositions[2]]),
			LoadPlaneWLDG(&planes[neighborPositions[3]])
		);
		surfaceNormal = ComputeDepthGradient(g_cameras[0].model, depth, p, ndepths);
		numValidPlanes = 4;
	}
	constexpr int numPlanes = 4;
	const float depths[numPlanes] = {depthPerturbed, depth, depth, depth};
	const Point3 normals[numPlanes] = {plane.topLeftCorner<3,1>(), perturbedNormal, normalRand, surfaceNormal};
	for (int i = 0; i < numValidPlanes; ++i) {
		Point4 newPlane;
		newPlane.topLeftCorner<3,1>() = normals[i];
		newPlane.w() = depths[i];
		MultiViewScorePlane(refCache, images, depthImages, p, newPlane, lowDepth, costVector, params);
		const float costPlane = AggregateMultiViewScores(viewWeights, costVector, params.nNumViews);
		if (cost > costPlane) {
			cost = costPlane;
			plane = newPlane;
		}
	}
}

// compute the score of the current plane estimate
__device__ void InitializePixelScore(const ImagePixels *images, const ImagePixels* depthImages, Point4* planes, const float* lowDepths, float* costs, RandState* randStates, unsigned* selectedViews, const Point2i& p, const PatchMatch::Params params)
{
	const int width = g_cameras[0].size.x();
	const int height = g_cameras[0].size.y();
	if (p.x() >= width || p.y() >= height)
		return;
	const int idx = Point2Idx(p, width);
	float lowDepth = 0;
	if (params.bLowResProcessed)
		lowDepth = lowDepths[idx];
	// reference-patch state is invariant across views and hypotheses; cache once
	RefPatchCache refCache;
	ComputeRefPatchCache(images[0], p, refCache);
	// initialize estimate randomly if not set
	RandState* randState = &randStates[idx];
	curand_init(1234/*threadIdx.x*/, p.y(), p.x(), randState);
	Point4& plane = planes[idx];
	float depth = plane.w();
	if (depth <= 0.f) {
		// generate random plane
		plane.topLeftCorner<3,1>() = GenerateRandomNormal(g_cameras[0], p, randState);
		plane.w() = curand_uniform(randState) * (params.fDepthMax - params.fDepthMin) + params.fDepthMin;
	} else if (plane.topLeftCorner<3,1>().dot(g_cameras[0].model.ViewDirection(p)) >= 0.f) {
		// generate random normal
		plane.topLeftCorner<3,1>() = GenerateRandomNormal(g_cameras[0], p, randState);
	}
	// compute costs
	float costVector[MAX_VIEWS];
	MultiViewScorePlane(refCache, images, depthImages, p, plane, lowDepth, costVector, params);
	// select best views
	float costVectorSorted[MAX_VIEWS];
	Sort(costVector, costVectorSorted, params.nNumViews);
	float cost = 0.f;
	for (int i = 0; i < params.nInitTopK; ++i)
		cost += costVectorSorted[i];
	const float costThreshold = costVectorSorted[params.nInitTopK - 1];
	unsigned& selectedView = selectedViews[idx];
	selectedView = 0;
	for (int imgId = 0; imgId < params.nNumViews; ++imgId)
		if (costVector[imgId] <= costThreshold)
			SetBit(selectedView, imgId);
	costs[idx] = cost / params.nInitTopK;
}
__global__ PATCHMATCHCUDA_LAUNCH_BOUNDS void InitializeScore(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, Point4* planes, const float* lowDepths, float* costs, curandState* randStates, unsigned* selectedViews, const PatchMatch::Params params)
{
	const Point2i p = GetThreadIndex2();
	InitializePixelScore((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p, params);
}

// traverse image in a back/red checkerboard pattern
__global__ PATCHMATCHCUDA_LAUNCH_BOUNDS void BlackPixelProcess(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, Point4* planes, const float* lowDepths, float* costs, curandState* randStates, unsigned* selectedViews, const PatchMatch::Params params, const int iter)
{
	Point2i p = GetThreadIndex2();
	p.y() = p.y() * 2 + (threadIdx.x % 2 == 0 ? 0 : 1);
	ProcessPixel((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p, params, iter);
}
__global__ PATCHMATCHCUDA_LAUNCH_BOUNDS void RedPixelProcess(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, Point4* planes, const float* lowDepths, float* costs, curandState* randStates, unsigned* selectedViews, const PatchMatch::Params params, const int iter)
{
	Point2i p = GetThreadIndex2();
	p.y() = p.y() * 2 + (threadIdx.x % 2 == 0 ? 1 : 0);
	ProcessPixel((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, planes, lowDepths, costs, (RandState*)randStates, selectedViews, p, params, iter);
}

// filter depth/normals
__global__ void FilterPlanes(Point4* planes, float* costs, unsigned* selectedViews, int width, int height, const PatchMatch::Params params)
{
	const Point2i p = GetThreadIndex2();
	if (p.x() >= width || p.y() >= height)
		return;
	const int idx = Point2Idx(p, width);
	// filter estimates if the score is not good enough
	Point4& plane = planes[idx];
	float conf = costs[idx];
	if (plane.w() <= 0 || conf >= params.fThresholdKeepCost) {
		conf = 0;
		plane = Point4::Zero();
		selectedViews[idx] = 0;
	}
}
/*----------------------------------------------------------------*/


// host helper: upload camera array into the device __constant__ symbol.
// B3: queued on the per-instance stream so it serializes with the rest of
// the H->D setup without fencing the device.
__host__ void PatchMatch::UploadCameras()
{
	const size_t n = cameras.size();
	ASSERT(n <= MAX_VIEWS + 1);
	CUDA_CHECK(cudaMemcpyToSymbolAsync(g_cameras, cameras.data(), sizeof(Camera) * n, 0, cudaMemcpyHostToDevice, cudaStream));
}

__host__ void PatchMatch::RunCUDA(float* ptrCostMap, uint32_t* ptrViewsMap)
{
	const unsigned width = cameras[0].size.x();
	const unsigned height = cameras[0].size.y();

	constexpr unsigned BLOCK_W = 32;
	// BLOCK_H is selected by PATCHMATCHCUDA_LB_256_2 (build-time toggle): the
	// default mode uses BLOCK_W/4 (=8) so threads/block matches the
	// __launch_bounds__(256, 2) annotation; the legacy mode uses BLOCK_W/2
	// (=16) for the historical 512 threads/block, 1 block/SM config.
	constexpr unsigned BLOCK_H = (BLOCK_W / PATCHMATCHCUDA_BLOCK_H_DIV);

	const dim3 blockSize(BLOCK_W, BLOCK_H, 1);
	// gridSizeFull: block covers BLOCK_W in x. The previous formula divided
	// width by BLOCK_H, launching ~2x extra blocks of which most threads
	// early-returned. Dividing by BLOCK_W matches actual block coverage.
	const dim3 gridSizeFull((width + BLOCK_W - 1) / BLOCK_W, (height + BLOCK_H - 1) / BLOCK_H, 1);
	const dim3 gridSizeCheckerboard((width + BLOCK_W - 1) / BLOCK_W, ((height / 2) + BLOCK_H - 1) / BLOCK_H, 1);

	InitializeScore<<<gridSizeFull, blockSize, 0, cudaStream>>>(cudaTextureImages, cudaTextureDepths, cudaDepthNormalEstimates, cudaLowDepths, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, params);
	cudaStreamSynchronize(cudaStream);

	for (int iter = 0; iter < params.nEstimationIters; ++iter) {
		BlackPixelProcess<<<gridSizeCheckerboard, blockSize, 0, cudaStream>>>(cudaTextureImages, cudaTextureDepths, cudaDepthNormalEstimates, cudaLowDepths, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, params, iter);
		cudaStreamSynchronize(cudaStream);
		RedPixelProcess<<<gridSizeCheckerboard, blockSize, 0, cudaStream>>>(cudaTextureImages, cudaTextureDepths, cudaDepthNormalEstimates, cudaLowDepths, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, params, iter);
		cudaStreamSynchronize(cudaStream);
	}

	if (params.fThresholdKeepCost > 0)
		FilterPlanes<<<gridSizeFull, blockSize, 0, cudaStream>>>(cudaDepthNormalEstimates, cudaDepthNormalCosts, cudaSelectedViews, width, height, params);

	cudaMemcpyAsync(depthNormalEstimates, cudaDepthNormalEstimates, sizeof(Point4) * width * height, cudaMemcpyDeviceToHost, cudaStream);
	if (ptrCostMap)
		cudaMemcpyAsync(ptrCostMap, cudaDepthNormalCosts, sizeof(float) * width * height, cudaMemcpyDeviceToHost, cudaStream);
	if (ptrViewsMap)
		cudaMemcpyAsync(ptrViewsMap, cudaSelectedViews, sizeof(uint32_t) * width * height, cudaMemcpyDeviceToHost, cudaStream);

	cudaStreamSynchronize(cudaStream);
}
/*----------------------------------------------------------------*/

} // namespace CUDA

} // namespace MVS
