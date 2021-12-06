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

using namespace MVS;

typedef Eigen::Matrix<int,2,1> Point2i;
typedef Eigen::Matrix<float,2,1> Point2;
typedef Eigen::Matrix<float,3,1> Point3;
typedef Eigen::Matrix<float,4,1> Point4;
typedef Eigen::Matrix<float,3,3> Matrix3;

#define ImagePixels cudaTextureObject_t
#define RandState curandState

// square the given value
__device__ inline constexpr float Square(float v) {
	return v * v;
}

// set/check a bit
__device__ inline constexpr void SetBit(unsigned& input, unsigned i) {
	input |= (1u << i);
}
__device__ inline constexpr int IsBitSet(unsigned input, unsigned i) {
	return (input >> i) & 1u;
}

// swap the given values
__device__ inline constexpr void Swap(float& v0, float& v1) {
	const float tmp = v0;
	v0 = v1;
	v1 = tmp;
}

// convert 2d to 1d coordinates and back
__device__ inline int Point2Idx(const Point2i& p, int width) {
	return p.y() * width + p.x();
}
__device__ inline Point2i Idx2Point(int idx, int width) {
	return Point2i(idx % width, idx / width);
}

// project and back-project a 3D point
__device__ inline Point2 ProjectPoint(const PatchMatchCUDA::Camera& camera, const Point3& X) {
	const Point3 x = camera.K * camera.R * (X - camera.C);
	return x.hnormalized();
}
__device__ inline Point3 BackProjectPointCamera(const PatchMatchCUDA::Camera& camera, const Point2& p, const float depth = 1.f) {
	return Point3(
		depth * (p.x() - camera.K(0,2)) / camera.K(0,0),
		depth * (p.y() - camera.K(1,2)) / camera.K(1,1),
		depth);
}
__device__ inline Point3 BackProjectPoint(const PatchMatchCUDA::Camera& camera, const Point2& p, const float depth) {
	const Point3 camX = BackProjectPointCamera(camera, p, depth);
	return camera.R.transpose() * camX + camera.C;
}

// compute camera ray direction for the given pixel
__device__ inline Point3 ViewDirection(const PatchMatchCUDA::Camera& camera, const Point2i& p) {
	return BackProjectPointCamera(camera, p.cast<float>()).normalized();
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


// generate a random normal
__device__ inline Point3 GenerateRandomNormal(const PatchMatchCUDA::Camera& camera, const Point2i& p, RandState* randState)
{
	float q1, q2, s;
	do {
		q1 = 2.f * curand_uniform(randState) - 1.f;
		q2 = 2.f * curand_uniform(randState) - 1.f;
		s = q1 * q1 + q2 * q2;
	} while (s >= 1.f);
	const float sq = sqrt(1.f - s);
	Point3 normal(
		2.f * q1 * sq,
		2.f * q2 * sq,
		1.f - 2.f * s);

	const Point3 viewDirection = ViewDirection(camera, p);
	if (normal.dot(viewDirection) > 0.f)
		normal = -normal;
	return normal.normalized();
}

// randomly perturb a normal
__device__ inline Point3 GeneratePerturbedNormal(const PatchMatchCUDA::Camera& camera, const Point2i& p, const Point3& normal, RandState* randState, const float perturbation)
{
	const Point3 viewDirection = ViewDirection(camera, p);

	const float a1 = (curand_uniform(randState) - 0.5f) * perturbation;
	const float a2 = (curand_uniform(randState) - 0.5f) * perturbation;
	const float a3 = (curand_uniform(randState) - 0.5f) * perturbation;

	const float sinA1 = sin(a1);
	const float sinA2 = sin(a2);
	const float sinA3 = sin(a3);
	const float cosA1 = cos(a1);
	const float cosA2 = cos(a2);
	const float cosA3 = cos(a3);

	Matrix3 perturb; perturb <<
		cosA2 * cosA3,
		cosA3 * sinA1 * sinA2 - cosA1 * sinA3,
		sinA1 * sinA3 + cosA1 * cosA3 * sinA2,
		cosA2 * sinA3,
		cosA1 * cosA3 + sinA1 * sinA2 * sinA3,
		cosA1 * sinA2 * sinA3 - cosA3 * sinA1,
		-sinA2,
		cosA2 * sinA1,
		cosA1 * cosA2;

	Point3 normalPerturbed = perturb * normal.topLeftCorner<3,1>();
	if (normalPerturbed.dot(viewDirection) >= 0.f)
		return normal;
	return normalPerturbed.normalized();
}

// interpolate given pixel's estimate to the current position
__device__ inline float InterpolatePixel(const PatchMatchCUDA::Camera& camera, const Point2i& p, const Point2i& np, float depth, const Point3& normal, const PatchMatchCUDA::Params& params)
{
	float depthNew;
	if (p.x() == np.x()) {
		const float nx1 = (p.y() - camera.K(1,2)) / camera.K(1,1);
		const float denom = normal.z() + nx1 * normal.y();
		if (abs(denom) < FLT_EPSILON)
			return depth;
		const float x1 = (np.y() - camera.K(1,2)) / camera.K(1,1);
		const float nom = depth * (normal.z() + x1 * normal.y());
		depthNew = nom / denom;
	} else if (p.y() == np.y()) {
		const float nx1 = (p.x() - camera.K(0,2)) / camera.K(0,0);
		const float denom = normal.z() + nx1 * normal.x();
		if (abs(denom) < FLT_EPSILON)
			return depth;
		const float x1 = (np.x() - camera.K(0,2)) / camera.K(0,0);
		const float nom = depth * (normal.z() + x1 * normal.x());
		depthNew = nom / denom;
	} else {
		const float planeD = normal.dot(BackProjectPointCamera(camera, np.cast<float>(), depth));
		depthNew = planeD / normal.dot(BackProjectPointCamera(camera, p.cast<float>()));
	}
	return (depthNew >= params.fDepthMin && depthNew <= params.fDepthMax) ? depthNew : depth;
}

// compute normal to the surface given the 4 neighbors
__device__ inline Point3 ComputeDepthGradient(const Matrix3& K, float depth, const Point2i& pos, const Point4& ndepth) {
	constexpr float2 nposg[4] = {{0,-1}, {0,1}, {-1,0}, {1,0}};
	Point2 dg(0,0);
	// add neighbor depths at the gradient locations
	for (int i=0; i<4; ++i)
		dg += Point2(nposg[i].x,nposg[i].y) * (ndepth[i] - depth);
	// compute depth gradient
	const Point2 d = dg*0.5f;
	// compute normal from depth gradient
	return Point3(
		K(0,0)*d.x(),
		K(1,1)*d.y(),
		(K(0,2)-pos.x())*d.x()+(K(1,2)-pos.y())*d.y()-depth).normalized();
}

// compose tho homography matrix that transforms a point from reference to source camera through the given plane
__device__ inline Matrix3 ComputeHomography(const PatchMatchCUDA::Camera& refCamera, const PatchMatchCUDA::Camera& trgCamera, const Point2& p, const Point4& plane)
{
	const Point3 X = BackProjectPointCamera(refCamera, p, plane.w());
	const Point3 normal = plane.topLeftCorner<3,1>();
	const Point3 t = (refCamera.C - trgCamera.C) / (normal.dot(X));
	const Matrix3 H = trgCamera.R * (refCamera.R.transpose() + t*normal.transpose());
	return trgCamera.K * H * refCamera.K.inverse();
}

// weight a neighbor texel based on color similarity and distance to the center texel
__device__ inline float ComputeBilateralWeight(int xDist, int yDist, float pix, float centerPix)
{
	constexpr float sigmaSpatial = -1.f / (2.f * (nSizeHalfWindow-1)*(nSizeHalfWindow-1));
	constexpr float sigmaColor = -1.f / (2.f * 25.f/255.f*25.f/255.f);
	const float spatialDistSq = float(xDist * xDist + yDist * yDist);
	const float colorDistSq = Square(pix - centerPix);
	return exp(spatialDistSq * sigmaSpatial + colorDistSq * sigmaColor);
}

// compute the geometric consistency weight
__device__ inline float GeometricConsistencyWeight(const ImagePixels depthImage, const PatchMatchCUDA::Camera& refCamera, const PatchMatchCUDA::Camera& trgCamera, const Point4& plane, const Point2i& p)
{
	if (depthImage == NULL)
		return 0.f;
	constexpr float maxDist = 4.f;
	const Point3 forwardPoint = BackProjectPoint(refCamera, p.cast<float>(), plane.w());
	const Point2 trgPt = ProjectPoint(trgCamera, forwardPoint);
	const float trgDepth = tex2D<float>(depthImage, trgPt.x() + 0.5f, trgPt.y() + 0.5f);
	if (trgDepth == 0.f)
		return maxDist;
	const Point3 trgX = BackProjectPoint(trgCamera, trgPt, trgDepth);
	const Point2 backwardPoint = ProjectPoint(refCamera, trgX);
	const Point2 diff = p.cast<float>() - backwardPoint;
	const float dist = diff.norm();
	return min(maxDist, sqrt(dist*(dist+2.f)));
}

// compute photometric score using weighted ZNCC
__device__ float ScorePlane(const ImagePixels refImage, const PatchMatchCUDA::Camera& refCamera, const ImagePixels trgImage, const PatchMatchCUDA::Camera& trgCamera, const Point2i& p, const Point4& plane, const PatchMatchCUDA::Params& params)
{
	constexpr float maxCost = 1.2f;

	Matrix3 H = ComputeHomography(refCamera, trgCamera, p.cast<float>(), plane);
	const Point2 pt = (H * p.cast<float>().homogeneous()).hnormalized();
	if (pt.x() >= trgCamera.width || pt.x() < 0.f || pt.y() >= trgCamera.height || pt.y() < 0.f)
		return maxCost;
	Point3 X = H * Point2(p.x()-nSizeHalfWindow, p.y()-nSizeHalfWindow).homogeneous();
	Point3 baseX(X);
	H *= float(nSizeStep);

	float sumRef = 0.f;
	float sumRefRef = 0.f;
	float sumTrg = 0.f;
	float sumTrgTrg = 0.f;
	float sumRefTrg = 0.f;
	float bilateralWeightSum = 0.f;
	const float refCenterPix = tex2D<float>(refImage, p.x() + 0.5f, p.y() + 0.5f);
	for (int i = -nSizeHalfWindow; i <= nSizeHalfWindow; i += nSizeStep) {
		for (int j = -nSizeHalfWindow; j <= nSizeHalfWindow; j += nSizeStep) {
			const Point2i refPt = Point2i(p.x() + j, p.y() + i);
			const Point2 trgPt = X.hnormalized();
			const float refPix = tex2D<float>(refImage, refPt.x() + 0.5f, refPt.y() + 0.5f);
			const float trgPix = tex2D<float>(trgImage, trgPt.x() + 0.5f, trgPt.y() + 0.5f);
			const float weight = ComputeBilateralWeight(j, i, refPix, refCenterPix);
			const float weightRefPix = weight * refPix;
			const float weightTrgPix = weight * trgPix;
			sumRef += weightRefPix;
			sumTrg += weightTrgPix;
			sumRefRef += weightRefPix * refPix;
			sumTrgTrg += weightTrgPix * trgPix;
			sumRefTrg += weightRefPix * trgPix;
			bilateralWeightSum += weight;
			X += H.col(0);
		}
		baseX += H.col(1);
		X = baseX;
	}

	const float varRef = sumRefRef * bilateralWeightSum - sumRef * sumRef;
	const float varTrg = sumTrgTrg * bilateralWeightSum - sumTrg * sumTrg;

	constexpr float kMinVar = 1e-5f;
	if (varRef < kMinVar || varTrg < kMinVar)
		return maxCost;
	const float covarTrgRef = sumRefTrg * bilateralWeightSum - sumRef * sumTrg;
	const float varRefTrg = sqrt(varRef * varTrg);
	return max(0.f, min(maxCost, 1.f - covarTrgRef / varRefTrg));
}

// compute photometric score for all neighbor images
__device__ inline void MultiViewScorePlane(const ImagePixels *images, const ImagePixels* depthImages, const PatchMatchCUDA::Camera* cameras, const Point2i& p, const Point4& plane, float* costVector, const PatchMatchCUDA::Params& params)
{
	for (int imgId = 1; imgId <= params.nNumViews; ++imgId)
		costVector[imgId-1] = ScorePlane(images[0], cameras[0], images[imgId], cameras[imgId], p, plane, params);
	if (params.bGeomConsistency)
		for (int imgId = 0; imgId < params.nNumViews; ++imgId)
			costVector[imgId] += 0.1f * GeometricConsistencyWeight(depthImages[imgId], cameras[0], cameras[imgId+1], plane, p);
}
// same as above, but interpolate the plane to current pixel position
__device__ inline float MultiViewScoreNeighborPlane(const ImagePixels* images, const ImagePixels* depthImages, const PatchMatchCUDA::Camera* cameras, const Point2i& p, const Point2i& np, Point4 plane, float* costVector, const PatchMatchCUDA::Params& params)
{
	plane.w() = InterpolatePixel(cameras[0], p, np, plane.w(), plane.topLeftCorner<3,1>(), params);
	MultiViewScorePlane(images, depthImages, cameras, p, plane, costVector, params);
	return plane.w();
}

// aggregate photometric score from all images
__device__ inline float AggregateMultiViewScores(const unsigned* viewWeights, const float* costVector, int numViews)
{
	float cost = 0;
	for (int imgId = 0; imgId < numViews; ++imgId)
		if (viewWeights[imgId])
			cost += viewWeights[imgId] * costVector[imgId];
	return cost;
}

// propagate and refine the plane estimate for the current pixel employing the asymmetric approach described in:
// "Multi-View Stereo with Asymmetric Checkerboard Propagation and Multi-Hypothesis Joint View Selection", 2018
__device__ void ProcessPixel(const ImagePixels* images, const ImagePixels* depthImages, const PatchMatchCUDA::Camera* cameras, Point4* planes, float* costs, RandState* randStates, unsigned* selectedViews, const Point2i& p, const PatchMatchCUDA::Params& params, const int iter)
{
	int width = cameras[0].width;
	int height = cameras[0].height;
	if (p.x() >= width || p.y() >= height)
		return;
	const int idx = Point2Idx(p, width);
	RandState* randState = &randStates[idx];
	Point4& plane = planes[idx];

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
			neighborDepths[posId] = MultiViewScoreNeighborPlane(images, depthImages, cameras, p, bestNx, planes[positions[posId]], costArray[posId], params);
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
	unsigned newSelectedViews = 0;
	for (int imgId = 0; imgId < params.nNumViews; ++imgId)
		if (viewWeights[imgId])
			SetBit(newSelectedViews, imgId);
	float finalCosts[8];
	for (int posId = 0; posId < 8; ++posId)
		finalCosts[posId] = AggregateMultiViewScores(viewWeights, costArray[posId], params.nNumViews) / NUM_SAMPLES;
	const int minCostIdx = FindMinIndex(finalCosts, 8);
	float costVector[MAX_VIEWS];
	MultiViewScorePlane(images, depthImages, cameras, p, plane, costVector, params);
	float cost = AggregateMultiViewScores(viewWeights, costVector, params.nNumViews) / NUM_SAMPLES;
	costs[idx] = cost;
	if (finalCosts[minCostIdx] < cost && valid[minCostIdx]) {
		plane = planes[positions[minCostIdx]];
		plane.w() = neighborDepths[minCostIdx];
		costs[idx] = finalCosts[minCostIdx];
		selectedViews[idx] = newSelectedViews;
	}
	const float depth = plane.w();

	// refine estimate
	constexpr float perturbation = 0.01f;
	const float depthMinPerturbed = (1.f - perturbation) * depth;
	const float depthMaxPerturbed = (1.f + perturbation) * depth;
	float depthPerturbed;
	do {
		depthPerturbed = curand_uniform(randState) * (depthMaxPerturbed - depthMinPerturbed) + depthMinPerturbed;
	} while (depthPerturbed < params.fDepthMin && depthPerturbed > params.fDepthMax);
	const Point3 perturbedNormal = GeneratePerturbedNormal(cameras[0], p, plane.topLeftCorner<3,1>(), randState, perturbation * (float)M_PI);
	const Point3 normalRand = GenerateRandomNormal(cameras[0], p, randState);
	int numValidPlanes = 3;
	Point3 surfaceNormal;
	if (valid[0] && valid[1] && valid[2] && valid[3]) {
		// estimate normal from surrounding surface
		const Point4 ndepths(
			planes[neighborPositions[0]].w(),
			planes[neighborPositions[1]].w(),
			planes[neighborPositions[2]].w(),
			planes[neighborPositions[3]].w()
		);
		surfaceNormal = ComputeDepthGradient(cameras[0].K, depth, p, ndepths);
		numValidPlanes = 4;
	}
	const int numPlanes = 4;
	const float depths[numPlanes] = {depthPerturbed, depth, depth, depth};
	const Point3 normals[numPlanes] = {plane.topLeftCorner<3,1>(), perturbedNormal, normalRand, surfaceNormal};
	for (int i = 0; i < numValidPlanes; ++i) {
		Point4 newPlane;
		newPlane.topLeftCorner<3,1>() = normals[i];
		newPlane.w() = depths[i];
		MultiViewScorePlane(images, depthImages, cameras, p, newPlane, costVector, params);
		const float costPlane = AggregateMultiViewScores(viewWeights, costVector, params.nNumViews) / NUM_SAMPLES;
		if (cost > costPlane) {
			cost = costPlane;
			plane = newPlane;
		}
	}
}

// compute the score of the current plane estimate
__global__ void InitializeScore(const cudaTextureObject_t* images, const cudaTextureObject_t* depthImages, PatchMatchCUDA::Camera* cameras, Point4* planes, float* costs, curandState* randStates, unsigned* selectedViews, const PatchMatchCUDA::Params params)
{
	const Point2i p = Point2i(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y);
	const int width = cameras[0].width;
	const int height = cameras[0].height;
	if (p.x() >= width || p.y() >= height)
		return;
	const int idx = Point2Idx(p, width);
	// initialize estimate randomly if not set
	RandState* randState = &randStates[idx];
	curand_init(1234/*threadIdx.x*/, p.y(), p.x(), randState);
	Point4& plane = planes[idx];
	float depth = plane.w();
	if (depth <= 0.f) {
		// generate random plane
		plane.topLeftCorner<3,1>() = GenerateRandomNormal(cameras[0], p, randState);
		plane.w() = curand_uniform(randState) * (params.fDepthMax - params.fDepthMin) + params.fDepthMin;
	} else if (plane.topLeftCorner<3,1>().dot(ViewDirection(cameras[0], p)) >= 0.f){
		// generate random normal
		plane.topLeftCorner<3,1>() = GenerateRandomNormal(cameras[0], p, randState);
	}
	// compute costs
	float costVector[MAX_VIEWS];
	MultiViewScorePlane((const ImagePixels*)images, (const ImagePixels*)depthImages, cameras, p, plane, costVector, params);
	// select best views
	float costVectorSorted[MAX_VIEWS];
	Sort(costVector, costVectorSorted, params.nNumViews);
	float cost = 0.f;
	for (int i = 0; i < params.nInitTopK; ++i)
		cost += costVectorSorted[i];
	const float costThreshold = costVectorSorted[params.nInitTopK - 1];
	unsigned selectedView = selectedViews[idx];
	selectedView = 0;
	for (int imgId = 0; imgId < params.nNumViews; ++imgId)
		if (costVector[imgId] <= costThreshold)
			SetBit(selectedView, imgId);
	costs[idx] = cost / params.nInitTopK;
}

// traverse image in a back/red checkerboard pattern
__global__ void BlackPixelProcess(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, PatchMatchCUDA::Camera* cameras, Point4* planes, float* costs, curandState* randStates, unsigned* selectedViews, const PatchMatchCUDA::Params params, const int iter)
{
	Point2i p = Point2i(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y);
	p.y() = p.y() * 2 + (threadIdx.x % 2 == 0 ? 0 : 1);
	ProcessPixel((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, cameras, planes, costs, (RandState*)randStates, selectedViews, p, params, iter);
}
__global__ void RedPixelProcess(const cudaTextureObject_t* textureImages, const cudaTextureObject_t* textureDepths, PatchMatchCUDA::Camera* cameras, Point4* planes, float* costs, curandState* randStates, unsigned* selectedViews, const PatchMatchCUDA::Params params, const int iter)
{
	Point2i p = Point2i(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y);
	p.y() = p.y() * 2 + (threadIdx.x % 2 == 0 ? 1 : 0);
	ProcessPixel((const ImagePixels*)textureImages, (const ImagePixels*)textureDepths, cameras, planes, costs, (RandState*)randStates, selectedViews, p, params, iter);
}
/*----------------------------------------------------------------*/


__host__ void PatchMatchCUDA::RunCUDA()
{
	constexpr unsigned BLOCK_W = 32;
	constexpr unsigned BLOCK_H = (BLOCK_W / 2);

	const unsigned width = cameras[0].width;
	const unsigned height = cameras[0].height;

	const dim3 blockSize(BLOCK_W, BLOCK_H, 1);
	const dim3 gridSizeInitialize((width + BLOCK_H - 1) / BLOCK_H, (height + BLOCK_H - 1) / BLOCK_H, 1);
	const dim3 gridSizeCheckerboard((width + BLOCK_W - 1) / BLOCK_W, ((height / 2) + BLOCK_H - 1) / BLOCK_H, 1);

	InitializeScore<<<gridSizeInitialize, blockSize>>>(cudaTextureImages, cudaTextureDepths, cudaCameras, cudaDepthNormalEstimates, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, params);
	cudaDeviceSynchronize();

	for (int i = 0; i < params.nEstimationIters; ++i) {
		BlackPixelProcess<<<gridSizeCheckerboard, blockSize>>>(cudaTextureImages, cudaTextureDepths, cudaCameras, cudaDepthNormalEstimates, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, params, i);
		cudaDeviceSynchronize();
		RedPixelProcess<<<gridSizeCheckerboard, blockSize>>>(cudaTextureImages, cudaTextureDepths, cudaCameras, cudaDepthNormalEstimates, cudaDepthNormalCosts, cudaRandStates, cudaSelectedViews, params, i);
		cudaDeviceSynchronize();
	}

	cudaMemcpy(depthNormalEstimates, cudaDepthNormalEstimates, sizeof(Point4) * width * height, cudaMemcpyDeviceToHost);
	cudaMemcpy(depthNormalCosts, cudaDepthNormalCosts, sizeof(float) * width * height, cudaMemcpyDeviceToHost);
	cudaDeviceSynchronize();
}
/*----------------------------------------------------------------*/
