/*
* PatchMatchMetal.metal
*
* Copyright (c) 2014-2026 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
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
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

// Metal backend contributed by leNeo.
// Full MSL port of OpenMVS PatchMatchCUDA.cu (ACMH / AMHMVS patch-match stereo).
// Faithful translation: 4 kernels (InitializeScore, Black/RedPixelProcess, FilterPlanes)
// + all device functions. Eigen -> float3/float3x3, curand -> PCG, __constant__ -> buffers,
// <bool GEOM> template -> function constant.
#include <metal_stdlib>
using namespace metal;

constant bool GEOM [[function_constant(0)]];

#define MAX_VIEWS 32         // matches CUDA cap
#define NUM_SAMPLES 32
#define fBadCost 1.2f
#define HALF 4               // nSizeHalfWindow
#define STEP 2               // nSizeStep
#define NSAMP 25             // (2*HALF/STEP+1)^2

struct Camera {
    float2   f;
    float2   pp;
    float3x3 R;
    float3   C;
    int2     size;
};

struct Params {
    float fDepthMin;
    float fDepthMax;
    float fThresholdKeepCost;
    int   nNumViews;
    int   nEstimationIters;
    int   nInitTopK;
    int   bLowResProcessed;
    int   width;
    int   height;
};

// ---------------- small helpers ----------------
static inline void SetBit(thread uint& v, uint i) { v |= (1u << i); }
static inline int  IsBitSet(uint v, uint i) { return (v >> i) & 1u; }
static inline float Square(float x) { return x * x; }

static inline void Sort(thread const float* values, thread float* sorted, int n) {
    for (int i = 0; i < n; ++i) sorted[i] = values[i];
    do {
        int newn = 0;
        for (int i = 1; i < n; ++i)
            if (sorted[i-1] > sorted[i]) {
                float t = sorted[i-1]; sorted[i-1] = sorted[i]; sorted[i] = t;
                newn = i;
            }
        n = newn;
    } while (n);
}
static inline int FindMinIndex(thread const float* values, int n) {
    float mv = values[0]; int mi = 0;
    for (int i = 1; i < n; ++i) if (mv > values[i]) { mv = values[i]; mi = i; }
    return mi;
}
static inline void PDF2CDF(thread float* probs, int n) {
    float sum = 0; for (int i = 0; i < n; ++i) sum += probs[i];
    const float inv = 1.0f / sum;
    float acc = 0;
    for (int i = 0; i < n-1; ++i) { acc += probs[i] * inv; probs[i] = acc; }
    probs[n-1] = 1.0f;
}

// ---------------- RNG (curand replacement: PCG counter-based) ----------------
static inline uint pcg(thread uint& s) {
    s = s * 747796405u + 2891336453u;
    uint w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
    return (w >> 22u) ^ w;
}
static inline float curand_uniform(thread uint& s) {
    return float(pcg(s) & 0x00FFFFFFu) / float(0x01000000);
}

// ---------------- camera transforms (port of CUDA/Camera.h) ----------------
static inline float3 I2C(constant Camera& c, float2 x, float depth) {
    return float3(depth*(x.x-c.pp.x)/c.f.x, depth*(x.y-c.pp.y)/c.f.y, depth);
}
static inline float2 C2I(constant Camera& c, float3 X) {
    return float2(c.f.x*X.x/X.z + c.pp.x, c.f.y*X.y/X.z + c.pp.y);
}
static inline float3 W2C(constant Camera& c, float3 X) { return c.R * (X - c.C); }
static inline float3 C2W(constant Camera& c, float3 X) { return transpose(c.R) * X + c.C; }
static inline float2 W2I(constant Camera& c, float3 X) { return C2I(c, W2C(c, X)); }
static inline float3 I2W(constant Camera& c, float2 x, float depth) { return C2W(c, I2C(c, x, depth)); }
static inline float3 viewDir(constant Camera& c, float2 x) { return normalize(I2C(c, x, 1.0f)); }
static inline float3x3 Kmat(constant Camera& c) {
    return float3x3(float3(c.f.x,0,0), float3(0,c.f.y,0), float3(c.pp.x,c.pp.y,1));
}
static inline float3x3 Kinv(constant Camera& c) {
    return float3x3(float3(1.0f/c.f.x,0,0), float3(0,1.0f/c.f.y,0), float3(-c.pp.x/c.f.x,-c.pp.y/c.f.y,1));
}
static inline float3x3 outer(float3 t, float3 n) { return float3x3(t*n.x, t*n.y, t*n.z); }

// ---------------- random normals / perturbations ----------------
static inline float3 GenerateRandomUnitVector(thread uint& s) {
    float q1, q2, ss;
    do { q1 = 2.0f*curand_uniform(s)-1.0f; q2 = 2.0f*curand_uniform(s)-1.0f; ss = q1*q1+q2*q2; } while (ss >= 1.0f);
    const float sq = sqrt(1.0f - ss);
    return float3(2.0f*q1*sq, 2.0f*q2*sq, 1.0f - 2.0f*ss);
}
static inline float3 GenerateRandomNormal(constant Camera& cam, float2 p, thread uint& s) {
    const float3 n = GenerateRandomUnitVector(s);
    return dot(n, viewDir(cam, p)) > 0.0f ? -n : n;
}
static inline float3 GeneratePerturbedNormal(constant Camera& cam, float2 p, float3 normal, thread uint& s, float perturbation) {
    const float theta = (curand_uniform(s) - 0.5f) * perturbation;
    const float sinT = sin(theta), cosT = cos(theta);
    const float3 axis = GenerateRandomUnitVector(s);
    const float aDotN = dot(axis, normal);
    const float3 axCrossN = cross(axis, normal);
    const float3 np = normal*cosT + axCrossN*sinT + axis*(aDotN*(1.0f-cosT));
    return dot(np, viewDir(cam, p)) >= 0.0f ? normal : np;
}
static inline float GeneratePerturbedDepth(float depth, thread uint& s, float perturbation, constant Params& prm) {
    const float lo = max((1.0f-perturbation)*depth, prm.fDepthMin);
    const float hi = min((1.0f+perturbation)*depth, prm.fDepthMax);
    return lo + curand_uniform(s) * (hi - lo);
}

// interpolate neighbor plane's depth to current pixel (port of InterpolatePixel)
static inline float InterpolatePixel(constant Camera& cam, int2 p, int2 np, float depth, float3 normal, constant Params& prm) {
    float depthNew;
    if (p.x == np.x) {
        const float nx1 = (p.y - cam.pp.y) / cam.f.y;
        const float denom = normal.z + nx1*normal.y;
        if (fabs(denom) < FLT_EPSILON) return depth;
        const float x1 = (np.y - cam.pp.y) / cam.f.y;
        depthNew = depth*(normal.z + x1*normal.y) / denom;
    } else if (p.y == np.y) {
        const float nx1 = (p.x - cam.pp.x) / cam.f.x;
        const float denom = normal.z + nx1*normal.x;
        if (fabs(denom) < FLT_EPSILON) return depth;
        const float x1 = (np.x - cam.pp.x) / cam.f.x;
        depthNew = depth*(normal.z + x1*normal.x) / denom;
    } else {
        const float planeD = dot(normal, I2C(cam, float2(np), depth));
        depthNew = planeD / dot(normal, I2C(cam, float2(p), 1.0f));
    }
    return (depthNew >= prm.fDepthMin && depthNew <= prm.fDepthMax) ? depthNew : depth;
}

// surface normal from 4-neighborhood depths (port of ComputeDepthGradient)
static inline float3 ComputeDepthGradient(constant Camera& cam, float depth, int2 pos, float4 ndepth) {
    const float2 nposg[4] = { float2(0,-1), float2(0,1), float2(-1,0), float2(1,0) };
    float2 dg = float2(0,0);
    for (int i = 0; i < 4; ++i) dg += nposg[i] * (ndepth[i] - depth);
    const float2 d = dg * 0.5f;
    return normalize(float3(cam.f.x*d.x, cam.f.y*d.y,
                            (cam.pp.x-pos.x)*d.x + (cam.pp.y-pos.y)*d.y - depth));
}

static inline float3x3 ComputeHomography(constant Camera& ref, constant Camera& trg, float2 p, float4 plane) {
    const float3 X = I2C(ref, p, plane.w);
    const float3 normal = plane.xyz;
    const float denom = dot(normal, X);
    const float safeDenom = (fabs(denom) < FLT_EPSILON) ? copysign(FLT_EPSILON, denom) : denom;
    const float3 t = (ref.C - trg.C) / safeDenom;
    const float3x3 H = trg.R * (transpose(ref.R) + outer(t, normal));
    return Kmat(trg) * H * Kinv(ref);
}

// ---------------- bilateral ZNCC ----------------
static inline float bilateral(int xDist, int yDist, float pix, float center) {
    const float sigmaSpatial = -1.0f / (2.0f*(HALF-1)*(HALF-1));
    const float sigmaColor   = -1.0f / (2.0f*(25.0f/255.0f)*(25.0f/255.0f));
    return exp(float(xDist*xDist+yDist*yDist)*sigmaSpatial + Square(pix-center)*sigmaColor);
}
constexpr sampler texSampler(coord::pixel, filter::linear, address::clamp_to_edge);

struct RefCache {
    float w[NSAMP];
    float wRef[NSAMP];
    float sumRef;
    float bws;
    float varRef;
};
static inline void ComputeRefCache(texture2d<float> refImg, float2 p, thread RefCache& c) {
    const float center = refImg.sample(texSampler, float2(p.x+0.5f, p.y+0.5f)).r;
    float sumRef=0, sumRefRef=0, bws=0; int idx=0;
    for (int i=-HALF; i<=HALF; i+=STEP)
        for (int j=-HALF; j<=HALF; j+=STEP) {
            const float refPix = refImg.sample(texSampler, float2(p.x+j+0.5f, p.y+i+0.5f)).r;
            const float w = bilateral(j, i, refPix, center);
            const float wRef = w*refPix;
            c.w[idx]=w; c.wRef[idx]=wRef;
            sumRef+=wRef; sumRefRef+=wRef*refPix; bws+=w; ++idx;
        }
    c.sumRef=sumRef; c.bws=bws; c.varRef=sumRefRef*bws - sumRef*sumRef;
}

static inline float ScorePlane(thread const RefCache& cache, constant Camera& ref,
                               texture2d<float> trgImg, constant Camera& trg,
                               float2 p, float4 plane, float lowDepth) {
    float3x3 H = ComputeHomography(ref, trg, p, plane);
    {
        const float3 ptH = H * float3(p, 1.0f);
        const float invZ = 1.0f/ptH.z;
        const float ptX = ptH.x*invZ, ptY = ptH.y*invZ;
        if (ptX >= (float)trg.size.x || ptX < 0.0f || ptY >= (float)trg.size.y || ptY < 0.0f)
            return fBadCost;
    }
    float3 X = H * float3(p.x-HALF, p.y-HALF, 1.0f);
    float3 baseX = X;
    H = H * float(STEP);
    float sumTrg=0, sumTrgTrg=0, sumRefTrg=0; int idx=0;
    for (int i=-HALF; i<=HALF; i+=STEP) {
        for (int j=-HALF; j<=HALF; j+=STEP) {
            const float invZ = 1.0f/X.z;
            const float trgPix = trgImg.sample(texSampler, float2(X.x*invZ+0.5f, X.y*invZ+0.5f)).r;
            const float w = cache.w[idx];
            const float wTrg = w*trgPix;
            sumTrg += wTrg; sumTrgTrg += wTrg*trgPix; sumRefTrg += cache.wRef[idx]*trgPix;
            ++idx; X += H[0];
        }
        baseX += H[1]; X = baseX;
    }
    if (lowDepth <= 0 && cache.varRef < 1e-8f) return fBadCost;
    const float varTrg = sumTrgTrg*cache.bws - sumTrg*sumTrg;
    const float varRefTrg = cache.varRef*varTrg;
    if (varRefTrg < 1e-16f) return fBadCost;
    const float covar = sumRefTrg*cache.bws - cache.sumRef*sumTrg;
    float ncc = 1.0f - covar*rsqrt(varRefTrg);
    if (lowDepth > 0 && cache.varRef < 0.0025f) {
        const float depth = plane.w;
        const float deltaDepth = min(fabs(lowDepth-depth)/lowDepth, 0.5f);
        const float smoothSigmaDepth = -1.0f/(1.0f*0.02f);
        const float factor = exp(cache.varRef*smoothSigmaDepth);
        ncc = (1.0f-factor)*ncc + factor*deltaDepth;
    }
    return max(0.0f, min(2.0f, ncc));
}

static inline float GeometricConsistencyWeight(texture2d<float> depthImg, constant Camera& ref,
                                               constant Camera& trg, float4 plane, int2 p) {
    const float maxDist = 4.0f;
    // No depth-map for this neighbor: the driver binds the 1x1 dummy texture as a
    // stand-in. Mirror CUDA's `if (depthImage == NULL) return 0.f;` and skip the
    // geometric term, instead of charging the full maxDist penalty the zero-depth
    // branch below would — otherwise missing neighbors make Metal far more
    // conservative than CUDA (fewer fused points / lower recall).
    if (depthImg.get_width() <= 1) return 0.0f;
    const float3 fwd = I2W(ref, float2(p), plane.w);
    const float2 trgPt = W2I(trg, fwd);
    const float trgDepth = depthImg.sample(texSampler, float2(trgPt.x+0.5f, trgPt.y+0.5f)).r;
    if (trgDepth == 0.0f) return maxDist;
    const float3 trgX = I2W(trg, trgPt, trgDepth);
    const float2 back = W2I(ref, trgX);
    const float distSq = distance_squared(float2(p), back);
    return min(maxDist, sqrt(distSq + sqrt(distSq)*2.0f));
}

// multi-view score: fills costVector[0..nNumViews-1]
static inline void MultiViewScorePlane(thread const RefCache& cache,
                                       const array<texture2d<float>, MAX_VIEWS> images,
                                       const array<texture2d<float>, MAX_VIEWS> depthImages,
                                       constant Camera* cams, int2 p, float4 plane, float lowDepth,
                                       constant Params& prm, thread float* costVector) {
    const int nNumViews = prm.nNumViews;
    for (int imgId = 1; imgId <= nNumViews; ++imgId)
        costVector[imgId-1] = ScorePlane(cache, cams[0], images[imgId], cams[imgId], float2(p), plane, lowDepth);
    if (GEOM)
        for (int imgId = 0; imgId < nNumViews; ++imgId)
            costVector[imgId] += 0.1f * GeometricConsistencyWeight(depthImages[imgId], cams[0], cams[imgId+1], plane, p);
}
static inline float MultiViewScoreNeighborPlane(thread const RefCache& cache,
                                                const array<texture2d<float>, MAX_VIEWS> images,
                                                const array<texture2d<float>, MAX_VIEWS> depthImages,
                                                constant Camera* cams, int2 p, int2 np, float4 plane,
                                                float lowDepth, constant Params& prm, thread float* costVector) {
    plane.w = InterpolatePixel(cams[0], p, np, plane.w, plane.xyz, prm);
    MultiViewScorePlane(cache, images, depthImages, cams, p, plane, lowDepth, prm, costVector);
    return plane.w;
}
static inline float AggregateMultiViewScores(thread const uint* viewWeights, thread const float* costVector, int n) {
    float cost = 0;
    for (int imgId = 0; imgId < n; ++imgId) if (viewWeights[imgId]) cost += viewWeights[imgId]*costVector[imgId];
    return cost / float(NUM_SAMPLES);
}

// ---------------- per-pixel processing ----------------
static inline void ProcessPixel(const array<texture2d<float>, MAX_VIEWS> images,
                                const array<texture2d<float>, MAX_VIEWS> depthImages,
                                constant Camera* cams,
                                device float4* planes, device const float* lowDepths,
                                device float* costs, device uint* rngStates, device uint* selectedViews,
                                int2 p, int iter, constant Params& prm) {
    const int width = prm.width, height = prm.height;
    if (p.x >= width || p.y >= height) return;
    const int idx = p.y*width + p.x;
    uint state = rngStates[idx];
    float lowDepth = prm.bLowResProcessed ? lowDepths[idx] : 0.0f;
    RefCache refCache; ComputeRefCache(images[0], float2(p), refCache);

    const int2 dirs[8][11] = {
        {int2(0,-1),int2(-1,-2),int2(1,-2),int2(-2,-3),int2(2,-3),int2(-3,-4),int2(3,-4),int2(0,0),int2(0,0),int2(0,0),int2(0,0)},
        {int2(0,1),int2(-1,2),int2(1,2),int2(-2,3),int2(2,3),int2(-3,4),int2(3,4),int2(0,0),int2(0,0),int2(0,0),int2(0,0)},
        {int2(-1,0),int2(-2,-1),int2(-2,1),int2(-3,-2),int2(-3,2),int2(-4,-3),int2(-4,3),int2(0,0),int2(0,0),int2(0,0),int2(0,0)},
        {int2(1,0),int2(2,-1),int2(2,1),int2(3,-2),int2(3,2),int2(4,-3),int2(4,3),int2(0,0),int2(0,0),int2(0,0),int2(0,0)},
        {int2(0,-3),int2(0,-5),int2(0,-7),int2(0,-9),int2(0,-11),int2(0,-13),int2(0,-15),int2(0,-17),int2(0,-19),int2(0,-21),int2(0,-23)},
        {int2(0,3),int2(0,5),int2(0,7),int2(0,9),int2(0,11),int2(0,13),int2(0,15),int2(0,17),int2(0,19),int2(0,21),int2(0,23)},
        {int2(-3,0),int2(-5,0),int2(-7,0),int2(-9,0),int2(-11,0),int2(-13,0),int2(-15,0),int2(-17,0),int2(-19,0),int2(-21,0),int2(-23,0)},
        {int2(3,0),int2(5,0),int2(7,0),int2(9,0),int2(11,0),int2(13,0),int2(15,0),int2(17,0),int2(19,0),int2(21,0),int2(23,0)}
    };
    const int numDirs[8] = {7,7,7,7,11,11,11,11};
    const int neighborPositions[4] = { idx-width, idx+width, idx-1, idx+1 };
    bool valid[8] = {false,false,false,false,false,false,false,false};
    int positions[8];
    float neighborDepths[8];
    float costArray[8][MAX_VIEWS];

    for (int posId = 0; posId < 8; ++posId) {
        int2 bestNx = int2(0,0); float bestConf = FLT_MAX;
        for (int dirId = 0; dirId < numDirs[posId]; ++dirId) {
            const int2 np = int2(p.x+dirs[posId][dirId].x, p.y+dirs[posId][dirId].y);
            if (!(np.x>=0 && np.y>=0 && np.x<width && np.y<height)) continue;
            const float nconf = costs[np.y*width+np.x];
            if (bestConf > nconf) { bestConf = nconf; bestNx = np; }
        }
        if (bestConf < FLT_MAX) {
            valid[posId] = true;
            positions[posId] = bestNx.y*width+bestNx.x;
            neighborDepths[posId] = MultiViewScoreNeighborPlane(refCache, images, depthImages, cams, p, bestNx,
                                        planes[positions[posId]], lowDepth, prm, costArray[posId]);
        }
    }

    float viewSelectionPriors[MAX_VIEWS] = {};
    const int nNumViews = prm.nNumViews;
    for (int posId = 0; posId < 4; ++posId)
        if (valid[posId]) {
            const uint sv = selectedViews[neighborPositions[posId]];
            for (int j = 0; j < nNumViews; ++j) viewSelectionPriors[j] += IsBitSet(sv, j) ? 0.9f : 0.1f;
        }
    float samplingProbs[MAX_VIEWS];
    const float thCost = 0.8f * exp(Square((float)iter) / (-2.0f*4.0f*4.0f));
    for (int imgId = 0; imgId < nNumViews; ++imgId) {
        float sumW = 0; uint count = 0, countBad = 0;
        for (int posId = 0; posId < 8; ++posId)
            if (valid[posId]) {
                if (costArray[posId][imgId] < thCost) { sumW += exp(Square(costArray[posId][imgId])/(-2.0f*0.3f*0.3f)); ++count; }
                else if (costArray[posId][imgId] >= fBadCost) ++countBad;
            }
        if (count > 2 && countBad < 3) samplingProbs[imgId] = viewSelectionPriors[imgId]*sumW/count;
        else if (countBad < 3) samplingProbs[imgId] = viewSelectionPriors[imgId]*exp(Square(thCost)/(-2.0f*0.4f*0.4f));
        else samplingProbs[imgId] = 0.0f;
    }
    PDF2CDF(samplingProbs, nNumViews);
    uint viewWeights[MAX_VIEWS] = {};
    for (int sample = 0; sample < NUM_SAMPLES; ++sample) {
        const float r = curand_uniform(state);
        for (int imgId = 0; imgId < nNumViews; ++imgId) if (samplingProbs[imgId] > r) { ++viewWeights[imgId]; break; }
    }

    float4 plane = planes[idx];
    float cost = costs[idx];
    uint newSelectedViews = 0;
    for (int imgId = 0; imgId < nNumViews; ++imgId) if (viewWeights[imgId]) SetBit(newSelectedViews, (uint)imgId);
    float finalCosts[8];
    for (int posId = 0; posId < 8; ++posId) finalCosts[posId] = AggregateMultiViewScores(viewWeights, costArray[posId], nNumViews);
    const int minCostIdx = FindMinIndex(finalCosts, 8);
    float costVector[MAX_VIEWS];
    MultiViewScorePlane(refCache, images, depthImages, cams, p, plane, lowDepth, prm, costVector);
    cost = AggregateMultiViewScores(viewWeights, costVector, nNumViews);
    if (finalCosts[minCostIdx] < cost && valid[minCostIdx]) {
        plane = planes[positions[minCostIdx]];
        plane.w = neighborDepths[minCostIdx];
        cost = finalCosts[minCostIdx];
        selectedViews[idx] = newSelectedViews;
    }
    const float depth = plane.w;

    // refine
    const float perturbationDepth = 0.005f;
    const float perturbationNormal = 0.01f * M_PI_F;
    const float depthPerturbed = GeneratePerturbedDepth(depth, state, perturbationDepth, prm);
    const float3 perturbedNormal = GeneratePerturbedNormal(cams[0], float2(p), plane.xyz, state, perturbationNormal);
    const float3 normalRand = GenerateRandomNormal(cams[0], float2(p), state);
    int numValidPlanes = 3;
    float3 surfaceNormal = float3(0,0,0);
    if (valid[0] && valid[1] && valid[2] && valid[3]) {
        const float4 ndepths = float4(planes[neighborPositions[0]].w, planes[neighborPositions[1]].w,
                                      planes[neighborPositions[2]].w, planes[neighborPositions[3]].w);
        surfaceNormal = ComputeDepthGradient(cams[0], depth, p, ndepths);
        numValidPlanes = 4;
    }
    const float depths[4] = { depthPerturbed, depth, depth, depth };
    const float3 normals[4] = { plane.xyz, perturbedNormal, normalRand, surfaceNormal };
    for (int i = 0; i < numValidPlanes; ++i) {
        float4 newPlane = float4(normals[i], depths[i]);
        MultiViewScorePlane(refCache, images, depthImages, cams, p, newPlane, lowDepth, prm, costVector);
        const float costPlane = AggregateMultiViewScores(viewWeights, costVector, nNumViews);
        if (cost > costPlane) { cost = costPlane; plane = newPlane; }
    }

    planes[idx] = plane;
    costs[idx] = cost;
    rngStates[idx] = state;
}

static inline void InitializePixelScore(const array<texture2d<float>, MAX_VIEWS> images,
                                        const array<texture2d<float>, MAX_VIEWS> depthImages,
                                        constant Camera* cams,
                                        device float4* planes, device const float* lowDepths,
                                        device float* costs, device uint* rngStates, device uint* selectedViews,
                                        int2 p, constant Params& prm) {
    const int width = prm.width, height = prm.height;
    if (p.x >= width || p.y >= height) return;
    const int idx = p.y*width + p.x;
    float lowDepth = prm.bLowResProcessed ? lowDepths[idx] : 0.0f;
    RefCache refCache; ComputeRefCache(images[0], float2(p), refCache);

    uint state = (uint)(p.x*1973u + p.y*9277u + 1234u); // analogue of curand_init(1234, y, x)
    float4 plane = planes[idx];
    if (plane.w <= 0.0f) {
        plane.xyz = GenerateRandomNormal(cams[0], float2(p), state);
        plane.w = curand_uniform(state)*(prm.fDepthMax - prm.fDepthMin) + prm.fDepthMin;
    } else if (dot(plane.xyz, viewDir(cams[0], float2(p))) >= 0.0f) {
        plane.xyz = GenerateRandomNormal(cams[0], float2(p), state);
    }
    const int nNumViews = prm.nNumViews;
    const int nInitTopK = prm.nInitTopK;
    float costVector[MAX_VIEWS];
    MultiViewScorePlane(refCache, images, depthImages, cams, p, plane, lowDepth, prm, costVector);
    float sorted[MAX_VIEWS];
    Sort(costVector, sorted, nNumViews);
    float cost = 0;
    for (int i = 0; i < nInitTopK; ++i) cost += sorted[i];
    const float costThreshold = sorted[nInitTopK-1];
    uint sv = 0;
    for (int imgId = 0; imgId < nNumViews; ++imgId) if (costVector[imgId] <= costThreshold) SetBit(sv, (uint)imgId);
    selectedViews[idx] = sv;
    planes[idx] = plane;
    costs[idx] = cost / nInitTopK;
    rngStates[idx] = state;
}

// ---------------- kernels ----------------
kernel void InitializeScore(array<texture2d<float>, MAX_VIEWS> images [[texture(0)]],
                            array<texture2d<float>, MAX_VIEWS> depthImages [[texture(MAX_VIEWS)]],
                            constant Camera* cams [[buffer(0)]],
                            device float4* planes [[buffer(1)]],
                            device const float* lowDepths [[buffer(2)]],
                            device float* costs [[buffer(3)]],
                            device uint* rngStates [[buffer(4)]],
                            device uint* selectedViews [[buffer(5)]],
                            constant Params& prm [[buffer(6)]],
                            uint2 gid [[thread_position_in_grid]]) {
    InitializePixelScore(images, depthImages, cams, planes, lowDepths, costs, rngStates, selectedViews, int2(gid), prm);
}

kernel void BlackPixelProcess(array<texture2d<float>, MAX_VIEWS> images [[texture(0)]],
                              array<texture2d<float>, MAX_VIEWS> depthImages [[texture(MAX_VIEWS)]],
                              constant Camera* cams [[buffer(0)]],
                              device float4* planes [[buffer(1)]],
                              device const float* lowDepths [[buffer(2)]],
                              device float* costs [[buffer(3)]],
                              device uint* rngStates [[buffer(4)]],
                              device uint* selectedViews [[buffer(5)]],
                              constant Params& prm [[buffer(6)]],
                              constant int& iter [[buffer(7)]],
                              uint2 gid [[thread_position_in_grid]],
                              uint2 lid [[thread_position_in_threadgroup]]) {
    int2 p = int2(gid.x, gid.y*2 + ((lid.x % 2 == 0) ? 0 : 1));
    ProcessPixel(images, depthImages, cams, planes, lowDepths, costs, rngStates, selectedViews, p, iter, prm);
}

kernel void RedPixelProcess(array<texture2d<float>, MAX_VIEWS> images [[texture(0)]],
                            array<texture2d<float>, MAX_VIEWS> depthImages [[texture(MAX_VIEWS)]],
                            constant Camera* cams [[buffer(0)]],
                            device float4* planes [[buffer(1)]],
                            device const float* lowDepths [[buffer(2)]],
                            device float* costs [[buffer(3)]],
                            device uint* rngStates [[buffer(4)]],
                            device uint* selectedViews [[buffer(5)]],
                            constant Params& prm [[buffer(6)]],
                            constant int& iter [[buffer(7)]],
                            uint2 gid [[thread_position_in_grid]],
                            uint2 lid [[thread_position_in_threadgroup]]) {
    int2 p = int2(gid.x, gid.y*2 + ((lid.x % 2 == 0) ? 1 : 0));
    ProcessPixel(images, depthImages, cams, planes, lowDepths, costs, rngStates, selectedViews, p, iter, prm);
}

kernel void FilterPlanes(device float4* planes [[buffer(1)]],
                         device float* costs [[buffer(3)]],
                         device uint* selectedViews [[buffer(5)]],
                         constant Params& prm [[buffer(6)]],
                         uint2 gid [[thread_position_in_grid]]) {
    const int width = prm.width, height = prm.height;
    if ((int)gid.x >= width || (int)gid.y >= height) return;
    const int idx = gid.y*width + gid.x;
    if (planes[idx].w <= 0 || costs[idx] >= prm.fThresholdKeepCost) {
        costs[idx] = 0; planes[idx] = float4(0); selectedViews[idx] = 0;
    }
}
