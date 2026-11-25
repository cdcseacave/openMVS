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
// the camera models a kernel serves (the MODEL template argument of the CUDA kernels): every view
// pinhole, or a pinhole or an equirectangular reference whose neighbor views each take their own model
// (resolved once per pixel, hypothesis and view)
constant int MODEL [[function_constant(1)]];
#define MODEL_PINHOLE 0
#define MODEL_PINHOLE_REF 1
#define MODEL_SPHERICAL_REF 2

#define MAX_VIEWS 32         // matches CUDA cap
#define NUM_SAMPLES 32
#define fBadCost 1.2f
#define HALF 4               // nSizeHalfWindow
#define STEP 2               // nSizeStep
#define NSAMP 25             // (2*HALF/STEP+1)^2

// a view: its intrinsics in the fields of the CameraModels.h models (a pinhole K in pixels, or the
// angular K of an equirectangular camera) and its image size; the model is the kernel's to know
// (AsPinhole, AsSpherical), the pose is the view's ModelView
struct Camera {
    float fx, fy, cx, cy;
    int2  size;
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
    uint  sphericalViews;         // bit i set: view i (0 the reference) is equirectangular
    float fSinSqWarpMaxElevation; // squared sine of the target elevation above which the exact warp replaces the tangent-plane one
    float cosMinViewAngle;        // cosine of the triangulation angle below which a view scores as unseen (spherical views)
};

// a view as the kernels see it: its pose relative to the reference (a reference camera-frame
// point X is R*X+t in the view's frame; identity for the reference), its center in the reference
// camera frame and the scale from texel-centered pixel coordinates to its texture coordinates
// (normalized for an equirectangular view, else 1)
struct ModelView {
    float3x3 R;
    float3   t;
    float3   C;
    float2   texScale;
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

static inline float3x3 outer(float3 t, float3 n) { return float3x3(t*n.x, t*n.y, t*n.z); }

// ---------------- random normals / perturbations ----------------
static inline float3 GenerateRandomUnitVector(thread uint& s) {
    float q1, q2, ss;
    do { q1 = 2.0f*curand_uniform(s)-1.0f; q2 = 2.0f*curand_uniform(s)-1.0f; ss = q1*q1+q2*q2; } while (ss >= 1.0f);
    const float sq = sqrt(1.0f - ss);
    return float3(2.0f*q1*sq, 2.0f*q2*sq, 1.0f - 2.0f*ss);
}
static inline float GeneratePerturbedDepth(float depth, thread uint& s, float perturbation, constant Params& prm) {
    const float lo = max((1.0f-perturbation)*depth, prm.fDepthMin);
    const float hi = min((1.0f+perturbation)*depth, prm.fDepthMax);
    return lo + curand_uniform(s) * (hi - lo);
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
static inline float AggregateMultiViewScores(thread const uint* viewWeights, thread const float* costVector, int n) {
    float cost = 0;
    for (int imgId = 0; imgId < n; ++imgId) if (viewWeights[imgId]) cost += viewWeights[imgId]*costVector[imgId];
    return cost / float(NUM_SAMPLES);
}

// ================================================================================================
// camera models: hand mirror of libs/Math/Camera/CameraModels.h and libs/Common/DepthGeometry.h
// (MSL has no Eigen), function by function; keep it in step with them. Matrices are float3x3 built
// from their columns, where the Eigen comma initializers there list the rows. EvaluateModelMath, at
// the end of the block, evaluates it for the unit test against the C++ models (TestModelMath kernel).

// CameraModels.h FastAtan2: atan2 by the minimax polynomial of atan on [0,1], the same coefficients
static inline float FastAtan2(float y, float x) {
    const float ax = x < 0.f ? -x : x, ay = y < 0.f ? -y : y;
    const float mx = ax > ay ? ax : ay;
    if (mx == 0.f)
        return 0.f;
    const float a = (ax < ay ? ax : ay)/mx, s = a*a;
    float r = (((((((-0.0040540580f*s + 0.0218612288f)*s - 0.0559098861f)*s + 0.0964200441f)*s - 0.1390853351f)*s + 0.1994653599f)*s - 0.3332985605f)*s + 0.9999993329f)*a;
    if (ay > ax)
        r = M_PI_2_F - r;
    if (x < 0.f)
        r = M_PI_F - r;
    return y < 0.f ? -r : r;
}

// CameraModels.h Pinhole: focal lengths and principal point in pixels
struct PinholeModel {
    float fx, fy, cx, cy;
};
// Pinhole::Ray: PixelToNormalized, on the z=1 plane
static inline float3 PinholeRay(PinholeModel m, float2 x) {
    return float3((x.x - m.cx)/m.fx, (x.y - m.cy)/m.fy, 1.f);
}
// Pinhole::Project: Perspective, then NormalizedToPixel (Pinhole::ProjectFast is the same)
static inline bool PinholeProject(PinholeModel m, float3 X, thread float2& x) {
    const float invZ = 1.f/X.z;
    x = float2(m.fx*(X.x*invZ) + m.cx, m.fy*(X.y*invZ) + m.cy);
    return X.z > 0.f;
}
// Pinhole::Depth is X.z
// Pinhole::RayFrame: rows (1/fx,0,r.x), (0,1/fy,r.y), (0,0,1)
static inline float3x3 PinholeRayFrame(PinholeModel m, float2 x) {
    return float3x3(float3(1.f/m.fx, 0.f, 0.f), float3(0.f, 1.f/m.fy, 0.f), PinholeRay(m, x));
}
// Pinhole::PixelHomography: K
static inline float3x3 PinholePixelHomography(PinholeModel m) {
    return float3x3(float3(m.fx, 0.f, 0.f), float3(0.f, m.fy, 0.f), float3(m.cx, m.cy, 1.f));
}

// CameraModels.h Spherical: pixels per radian and the pixel of longitude 0 and elevation 0
struct SphericalModel {
    float fx, fy, cx, cy;
};
// Spherical::Ray from the sines and cosines of the pixel's longitude and elevation
static inline float3 SphericalRay(float sinLon, float cosLon, float sinEl, float cosEl) {
    return float3(cosEl*sinLon, -sinEl, cosEl*cosLon);
}
// Spherical::ProjectFast, given rho = sqrt(X.x^2+X.z^2)
static inline bool SphericalProjectFast(SphericalModel m, float3 X, thread float2& x, float rho) {
    x = float2(m.fx*FastAtan2(X.x, X.z) + m.cx, m.fy*FastAtan2(X.y, rho) + m.cy);
    return rho > 0.f || X.y != 0.f;
}
static inline bool SphericalProjectFast(SphericalModel m, float3 X, thread float2& x) {
    return SphericalProjectFast(m, X, x, sqrt(X.x*X.x + X.z*X.z));
}
// Spherical::Depth
static inline float SphericalDepth(float3 X) {
    return length(X);
}
// Spherical::RayFrame from the sines and cosines of the pixel's longitude and elevation: rows
// (cosEl*cosLon/fx, sinEl*sinLon/fy, cosEl*sinLon), (0, cosEl/fy, -sinEl), (-cosEl*sinLon/fx, sinEl*cosLon/fy, cosEl*cosLon)
static inline float3x3 SphericalRayFrame(SphericalModel m, float sinLon, float cosLon, float sinEl, float cosEl) {
    return float3x3(
        float3(cosEl*cosLon/m.fx, 0.f, -cosEl*sinLon/m.fx),
        float3(sinEl*sinLon/m.fy, cosEl/m.fy, sinEl*cosLon/m.fy),
        SphericalRay(sinLon, cosLon, sinEl, cosEl));
}
// Spherical::PixelHomography(ray, x, rho), x the pixel of the ray and rho = sqrt(ray.x^2+ray.z^2): rows
// (sx*ray.z + x.x*ray.x, x.x*ray.y, -sx*ray.x + x.x*ray.z), (-sy*ray.x + x.y*ray.x, fy*rho + x.y*ray.y, -sy*ray.z + x.y*ray.z), ray
static inline float3x3 SphericalPixelHomography(SphericalModel m, float3 ray, float2 x, float rho) {
    const float invRho = 1.f/rho;
    const float sx = m.fx*(rho*rho + ray.y*ray.y)*invRho*invRho, sy = m.fy*ray.y*invRho;
    return float3x3(
        float3(sx*ray.z + x.x*ray.x, -sy*ray.x + x.y*ray.x, ray.x),
        float3(x.x*ray.y, m.fy*rho + x.y*ray.y, ray.y),
        float3(-sx*ray.x + x.x*ray.z, -sy*ray.z + x.y*ray.z, ray.z));
}

// DepthGeometry.h InterpolatePlaneDepth(ray, nRay, nDepth, normal, dMin, dMax): the depth along ray of the
// plane through the point at nDepth along nRay; nDepth if the plane is parallel to ray or the new
// depth is outside [dMin,dMax]
static inline float InterpolatePlaneDepth(float3 ray, float3 nRay, float nDepth, float3 normal, float dMin, float dMax) {
    const float denom = dot(normal, ray);
    if (fabs(denom) < FLT_EPSILON)
        return nDepth;
    const float depth = nDepth*dot(normal, nRay)/denom;
    return depth >= dMin && depth <= dMax ? depth : nDepth;
}
// DepthGeometry.h DepthPlaneFit::Gradient over the 4-neighbors up, down, left, right (offsets (0,-1),
// (0,1), (-1,0), (1,0): hxx = hyy = 2, hxy = 0), given their depths in that order
static inline float2 DepthGradient4(float depth, float4 ndepth) {
    return float2((ndepth.w - depth) - (ndepth.z - depth), (ndepth.y - depth) - (ndepth.x - depth)) * 0.5f;
}
// DepthGeometry.h NormalFromDepthGradient (any camera model), given the camera's RayFrame F at the pixel
static inline float3 NormalFromDepthGradient(float3x3 F, float depth, float2 gradient) {
    const float3 tx = gradient.x*F[2] + depth*F[0];
    const float3 ty = gradient.y*F[2] + depth*F[1];
    return normalize(cross(ty, tx));
}
// DepthGeometry.h NormalFromDepthGradient(Pinhole, x, depth, gradient): the same normal in closed form
static inline float3 PinholeNormalFromDepthGradient(PinholeModel m, int2 x, float depth, float2 gradient) {
    return normalize(float3(m.fx*gradient.x, m.fy*gradient.y,
                            (m.cx - float(x.x))*gradient.x + (m.cy - float(x.y))*gradient.y - depth));
}
// DepthGeometry.h InterpolatePlaneDepth(Pinhole, x, nx, nDepth, normal, dMin, dMax): a neighbor nx on the
// same column or row is intersected in that slice, any other one along the ray through x
static inline float PinholeInterpolatePlaneDepth(PinholeModel m, int2 x, int2 nx, float nDepth, float3 normal, float dMin, float dMax) {
    float num, denom;
    if (x.x == nx.x) {
        const float rayY = (float(x.y) - m.cy)/m.fy;
        const float nRayY = (float(nx.y) - m.cy)/m.fy;
        denom = normal.z + rayY*normal.y;
        num = nDepth*(normal.z + nRayY*normal.y);
    } else if (x.y == nx.y) {
        const float rayX = (float(x.x) - m.cx)/m.fx;
        const float nRayX = (float(nx.x) - m.cx)/m.fx;
        denom = normal.z + rayX*normal.x;
        num = nDepth*(normal.z + nRayX*normal.x);
    } else {
        num = dot(normal, float3(nDepth*(float(nx.x) - m.cx)/m.fx, nDepth*(float(nx.y) - m.cy)/m.fy, nDepth));
        denom = dot(normal, float3((float(x.x) - m.cx)/m.fx, (float(x.y) - m.cy)/m.fy, 1.f));
    }
    if (fabs(denom) < FLT_EPSILON)
        return nDepth;
    const float depth = num/denom;
    return depth >= dMin && depth <= dMax ? depth : nDepth;
}

// the inputs and outputs of EvaluateModelMath; PatchMatchMetal.h holds the same struct
struct ModelMathSample {
    // inputs
    float spherical[4];      // Spherical fx, fy, cx, cy
    float pinhole[4];        // Pinhole fx, fy, cx, cy
    float X[3];              // a camera-frame point
    float trig[4];           // sin and cos of a longitude, then of an elevation
    float pixel[2];          // a pinhole pixel
    float ray[3], nRay[3];   // InterpolatePlaneDepth rays
    float normal[3];         // InterpolatePlaneDepth plane normal
    float nDepth, dMin, dMax;
    float depth, gradient[2];// NormalFromDepthGradient at the ray frame of trig
    int   ipixel[2], npixel[2];// a pinhole pixel and a neighbor of it, for the pinhole closed forms
    // outputs
    float fastAtan2;         // FastAtan2(X.y, X.x)
    float projected[2];      // Spherical::ProjectFast(X)
    float visible;           // its return value (0 or 1)
    float range;             // Spherical::Depth(X)
    float sphRay[3];         // Spherical::Ray(trig)
    float sphFrame[9];       // Spherical::RayFrame(trig), row-major
    float homography[9];     // Spherical::PixelHomography(X, projected, rho), row-major
    float pinFrame[9];       // Pinhole::RayFrame(pixel), row-major
    float pinProjected[2];   // Pinhole::Project(X)
    float pinVisible;        // its return value (0 or 1)
    float interpolated;      // InterpolatePlaneDepth(ray, nRay, nDepth, normal, dMin, dMax)
    float normalOut[3];      // NormalFromDepthGradient(sphFrame, depth, gradient)
    float pinNormal[3];      // NormalFromDepthGradient(pinhole, ipixel, depth, gradient)
    float pinInterpolated;   // InterpolatePlaneDepth(pinhole, ipixel, npixel, nDepth, normal, dMin, dMax)
};
static inline void StoreRowMajor(device float* out, float3x3 M) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[r*3+c] = M[c][r];
}
static inline void EvaluateModelMath(device ModelMathSample& s) {
    const SphericalModel sph = {s.spherical[0], s.spherical[1], s.spherical[2], s.spherical[3]};
    const PinholeModel pin = {s.pinhole[0], s.pinhole[1], s.pinhole[2], s.pinhole[3]};
    const float3 X = float3(s.X[0], s.X[1], s.X[2]);
    s.fastAtan2 = FastAtan2(X.y, X.x);
    const float rho = sqrt(X.x*X.x + X.z*X.z);
    float2 x;
    s.visible = SphericalProjectFast(sph, X, x, rho) ? 1.f : 0.f;
    s.projected[0] = x.x; s.projected[1] = x.y;
    s.range = SphericalDepth(X);
    const float3 r = SphericalRay(s.trig[0], s.trig[1], s.trig[2], s.trig[3]);
    s.sphRay[0] = r.x; s.sphRay[1] = r.y; s.sphRay[2] = r.z;
    const float3x3 F = SphericalRayFrame(sph, s.trig[0], s.trig[1], s.trig[2], s.trig[3]);
    StoreRowMajor(s.sphFrame, F);
    StoreRowMajor(s.homography, SphericalPixelHomography(sph, X, x, rho));
    StoreRowMajor(s.pinFrame, PinholeRayFrame(pin, float2(s.pixel[0], s.pixel[1])));
    float2 xp;
    s.pinVisible = PinholeProject(pin, X, xp) ? 1.f : 0.f;
    s.pinProjected[0] = xp.x; s.pinProjected[1] = xp.y;
    s.interpolated = InterpolatePlaneDepth(float3(s.ray[0], s.ray[1], s.ray[2]), float3(s.nRay[0], s.nRay[1], s.nRay[2]),
        s.nDepth, float3(s.normal[0], s.normal[1], s.normal[2]), s.dMin, s.dMax);
    const float3 n = NormalFromDepthGradient(F, s.depth, float2(s.gradient[0], s.gradient[1]));
    s.normalOut[0] = n.x; s.normalOut[1] = n.y; s.normalOut[2] = n.z;
    const int2 ix = int2(s.ipixel[0], s.ipixel[1]), nx = int2(s.npixel[0], s.npixel[1]);
    const float3 pn = PinholeNormalFromDepthGradient(pin, ix, s.depth, float2(s.gradient[0], s.gradient[1]));
    s.pinNormal[0] = pn.x; s.pinNormal[1] = pn.y; s.pinNormal[2] = pn.z;
    s.pinInterpolated = PinholeInterpolatePlaneDepth(pin, ix, nx, s.nDepth, float3(s.normal[0], s.normal[1], s.normal[2]), s.dMin, s.dMax);
}
// ======================================== end of the hand mirror ================================

// ================================================================================================
// the kernels of every MODEL, as in PatchMatchCUDA.cu

// an equirectangular view's image and depth textures take normalized coordinates, wrapping the
// columns across the seam and clamping the rows
constexpr sampler sphSampler(coord::normalized, filter::linear, s_address::repeat, t_address::clamp_to_edge);

// the camera model of a view (CUDA/Camera.h Camera::Model)
static inline PinholeModel AsPinhole(constant Camera& c) {
    return PinholeModel{c.fx, c.fy, c.cx, c.cy};
}
static inline SphericalModel AsSpherical(constant Camera& c) {
    return SphericalModel{c.fx, c.fy, c.cx, c.cy};
}

// a texel of a view's image or depth texture at its texture coordinates
static inline float SampleView(texture2d<float> tex, bool bSpherical, float2 coords) {
    return bSpherical ? tex.sample(sphSampler, coords).r : tex.sample(texSampler, coords).r;
}

// per-pixel reference state: the RayFrame F of the reference model at the pixel (the ray derivatives
// along x and y, then the ray); an equirectangular reference builds it, and the rays of the patch
// texels, from the sines and cosines of the column's longitude and the row's elevation (refTrig: W
// columns, then H rows), as the CPU estimator does
struct RefPixel {
    float3x3 F;
};
static inline RefPixel MakeRefPixel(constant Camera& ref, device const float2* refTrig, int2 p) {
    RefPixel r;
    if (MODEL == MODEL_SPHERICAL_REF) {
        const float2 lon = refTrig[p.x], el = refTrig[ref.size.x + p.y];
        r.F = SphericalRayFrame(AsSpherical(ref), lon.x, lon.y, el.x, el.y);
    } else {
        r.F = PinholeRayFrame(AsPinhole(ref), float2(p));
    }
    return r;
}
// the ray of the reference pixel at offset (j,i) of p, inside the image but for the wrapping columns
static inline float3 RefTexelRay(constant Camera& ref, device const float2* refTrig, int2 p, int j, int i) {
    if (MODEL == MODEL_SPHERICAL_REF) {
        const int width = ref.size.x;
        int c = p.x + j;
        c = c < 0 ? c + width : (c >= width ? c - width : c);
        const float2 lon = refTrig[c], el = refTrig[width + p.y + i];
        return SphericalRay(lon.x, lon.y, el.x, el.y);
    }
    return PinholeRay(AsPinhole(ref), float2(float(p.x + j), float(p.y + i)));
}

// reference image texel at pixel (x,y): an equirectangular reference texture takes normalized
// coordinates, wrapping the columns across the seam
static inline float FetchRefTexel(texture2d<float> refImg, constant ModelView* views, int x, int y) {
    if (MODEL == MODEL_SPHERICAL_REF)
        return refImg.sample(sphSampler, float2((x+0.5f)*views[0].texScale.x, (y+0.5f)*views[0].texScale.y)).r;
    return refImg.sample(texSampler, float2(x+0.5f, y+0.5f)).r;
}
static inline void ComputeRefCache(texture2d<float> refImg, constant ModelView* views, int2 p, thread RefCache& c) {
    const float center = FetchRefTexel(refImg, views, p.x, p.y);
    float sumRef=0, sumRefRef=0, bws=0; int idx=0;
    for (int i=-HALF; i<=HALF; i+=STEP)
        for (int j=-HALF; j<=HALF; j+=STEP) {
            const float refPix = FetchRefTexel(refImg, views, p.x+j, p.y+i);
            const float w = bilateral(j, i, refPix, center);
            const float wRef = w*refPix;
            c.w[idx]=w; c.wRef[idx]=wRef;
            sumRef+=wRef; sumRefRef+=wRef*refPix; bws+=w; ++idx;
        }
    c.sumRef=sumRef; c.bws=bws; c.varRef=sumRefRef*bws - sumRef*sumRef;
}

// target texels of the patch against the reference ones: the patch warp gives the homogeneous target
// texture coordinates of the top-left texel (X) and their steps along a patch row (dx) and down a patch
// column (dy); offset moves the dehomogenized coordinates onto the texel centers
struct PatchSums {
    float sumTrg, sumTrgTrg, sumRefTrg;
};
static inline void SampleWarpedPatch(thread const RefCache& cache, texture2d<float> trgImg, bool bSpherical,
                                     float3 X, float3 dx, float3 dy, float2 offset, thread PatchSums& sums) {
    float3 baseX = X;
    int idx = 0;
    for (int i=-HALF; i<=HALF; i+=STEP) {
        for (int j=-HALF; j<=HALF; j+=STEP) {
            const float invZ = 1.0f/X.z;
            const float trgPix = SampleView(trgImg, bSpherical, float2(X.x*invZ + offset.x, X.y*invZ + offset.y));
            const float w = cache.w[idx];
            const float wTrg = w*trgPix;
            sums.sumTrg += wTrg; sums.sumTrgTrg += wTrg*trgPix; sums.sumRefTrg += cache.wRef[idx]*trgPix;
            ++idx; X += dx;
        }
        baseX += dy; X = baseX;
    }
}
// weighted ZNCC cost of the sampled patch, blended with the depth prior in texture-less patches (the
// tail of ScorePlane)
static inline float ScoreNCC(thread const RefCache& cache, thread const PatchSums& sums, float depth, float lowDepth) {
    if (lowDepth <= 0 && cache.varRef < 1e-8f) return fBadCost;
    const float varTrg = sums.sumTrgTrg*cache.bws - sums.sumTrg*sums.sumTrg;
    const float varRefTrg = cache.varRef*varTrg;
    if (varRefTrg < 1e-16f) return fBadCost;
    const float covar = sums.sumRefTrg*cache.bws - cache.sumRef*sums.sumTrg;
    float ncc = 1.0f - covar*rsqrt(varRefTrg);
    if (lowDepth > 0 && cache.varRef < 0.0025f) {
        const float deltaDepth = min(fabs(lowDepth-depth)/lowDepth, 0.5f);
        const float smoothSigmaDepth = -1.0f/(1.0f*0.02f);
        const float factor = exp(cache.varRef*smoothSigmaDepth);
        ncc = (1.0f-factor)*ncc + factor*deltaDepth;
    }
    return max(0.0f, min(2.0f, ncc));
}

// Method A: each texel ray of the reference patch intersected with the plane and projected exactly
// into the equirectangular target; false if a texel ray meets the plane behind the reference camera
static inline bool SampleExactPatch(thread const RefCache& cache, constant Camera& refCam, device const float2* refTrig,
                                    texture2d<float> trgImg, SphericalModel trg, constant ModelView& view,
                                    float3 normal, float nX, int2 p, thread PatchSums& sums) {
    int idx = 0;
    for (int i=-HALF; i<=HALF; i+=STEP) {
        for (int j=-HALF; j<=HALF; j+=STEP, ++idx) {
            const float3 ray = RefTexelRay(refCam, refTrig, p, j, i);
            const float den = dot(normal, ray);
            if (den >= 0.f)
                return false;
            float2 x;
            SphericalProjectFast(trg, view.R * ray * (nX / den) + view.t, x);
            const float trgPix = trgImg.sample(sphSampler, float2((x.x + 0.5f) * view.texScale.x, (x.y + 0.5f) * view.texScale.y)).r;
            const float w = cache.w[idx];
            const float wTrg = w*trgPix;
            sums.sumTrg += wTrg; sums.sumTrgTrg += wTrg*trgPix; sums.sumRefTrg += cache.wRef[idx]*trgPix;
        }
    }
    return true;
}

// photometric score of the plane against a neighbor view of any camera model: the patch warp
// M = trg.PixelHomography(H*r0)*H*F (DepthGeometry.h PlanePatchWarp), F the reference ray frame at p
// and H*F = R*F + t*(n^T*F)/(n.X); into an equirectangular target the tangent-plane warp (Method B) up
// to the elevation threshold, the exact per-texel warp (Method A) above it, as the CPU estimator
static inline float ScorePlane(thread const RefCache& cache, thread const RefPixel& ref, texture2d<float> trgImg,
                                    constant Camera* cams, constant ModelView* views, device const float2* refTrig,
                                    int imgId, int2 p, float4 plane, float lowDepth, constant Params& prm) {
    constant ModelView& view = views[imgId];
    const float3 normal = plane.xyz;
    const float nX = plane.w * dot(normal, ref.F[2]); // n.X of the plane point at p
    if (nX >= 0.f)
        return fBadCost; // a propagated plane not facing the reference camera at p
    const float3x3 HF = view.R * ref.F + outer(view.t, float3(dot(normal, ref.F[0]), dot(normal, ref.F[1]), dot(normal, ref.F[2])) / nX);
    const float3 q0 = HF[2]; // the target ray of the patch center
    constant Camera& trgCam = cams[imgId];
    PatchSums sums = {0.f, 0.f, 0.f};
    if (MODEL != MODEL_PINHOLE && IsBitSet(prm.sphericalViews, (uint)imgId)) {
        const SphericalModel trg = AsSpherical(trgCam);
        if (Square(q0.y) > prm.fSinSqWarpMaxElevation * length_squared(q0)) {
            if (!SampleExactPatch(cache, cams[0], refTrig, trgImg, trg, view, normal, nX, p, sums))
                return fBadCost;
        } else {
            const float rho = sqrt(q0.x*q0.x + q0.z*q0.z);
            float2 xc;
            SphericalProjectFast(trg, q0, xc, rho);
            float3x3 M = SphericalPixelHomography(trg, q0, xc, rho) * HF;
            // the texture-coordinate scale folded into rows 0 and 1
            for (int k = 0; k < 3; ++k)
                M[k] = float3(M[k].x * view.texScale.x, M[k].y * view.texScale.y, M[k].z);
            const float3 X0 = M * float3(float(-HALF), float(-HALF), 1.f);
            const float3 dx = M[0] * float(STEP), dy = M[1] * float(STEP);
            // the homogeneous coordinate is affine over the patch: every texel is in front of the
            // tangent plane (its ray less than 90 degrees off the center) if the corners are
            const float steps = float(2 * HALF / STEP);
            if (X0.z <= 0.f || X0.z + dx.z * steps <= 0.f || X0.z + dy.z * steps <= 0.f || X0.z + (dx.z + dy.z) * steps <= 0.f)
                return fBadCost;
            // the texture wraps the columns across the seam and clamps the rows
            SampleWarpedPatch(cache, trgImg, true, X0, dx, dy, 0.5f * view.texScale, sums);
        }
    } else {
        // the patch center in front of a pinhole target and inside its image
        if (q0.z <= 0.f)
            return fBadCost;
        const float3x3 M = PinholePixelHomography(AsPinhole(trgCam)) * HF;
        const float3 center = M[2];
        const float invZ = 1.0f/center.z;
        const float ptX = center.x*invZ, ptY = center.y*invZ;
        if (ptX >= (float)trgCam.size.x || ptX < 0.0f || ptY >= (float)trgCam.size.y || ptY < 0.0f)
            return fBadCost;
        const float3 X0 = M * float3(float(-HALF), float(-HALF), 1.f);
        SampleWarpedPatch(cache, trgImg, false, X0, M[0] * float(STEP), M[1] * float(STEP), float2(0.5f, 0.5f), sums);
    }
    return ScoreNCC(cache, sums, plane.w, lowDepth);
}

// geometric consistency against a neighbor depth-map of any camera model: the reference point moved
// into the neighbor's frame and projected, the neighbor depth sampled there, the neighbor point at that
// depth along the same ray projected back; the reprojection distance (columns wrapped for an
// equirectangular reference) mapped to [0,4], 4 where the neighbor holds no depth
static inline float GeometricConsistencyWeight(texture2d<float> depthImg, thread const RefPixel& ref,
                                                    constant Camera* cams, constant ModelView* views,
                                                    int imgId, float4 plane, int2 p, constant Params& prm) {
    // no depth-map for this neighbor: the driver binds the 1x1 dummy texture, which skips the geometric
    // term as CUDA does for a missing depth-map (the maxDist penalty of a zero depth would make Metal far
    // more conservative than CUDA)
    if (depthImg.get_width() <= 1) return 0.0f;
    const float maxDist = 4.0f;
    constant ModelView& view = views[imgId];
    const float3 X1 = view.R * (ref.F[2] * plane.w) + view.t;
    const bool bSpherical = MODEL != MODEL_PINHOLE && IsBitSet(prm.sphericalViews, (uint)imgId);
    float2 x1;
    float depth1Expected;
    if (bSpherical) {
        SphericalProjectFast(AsSpherical(cams[imgId]), X1, x1);
        depth1Expected = SphericalDepth(X1);
    } else {
        if (!PinholeProject(AsPinhole(cams[imgId]), X1, x1))
            return maxDist;
        depth1Expected = X1.z;
    }
    const float depth1 = SampleView(depthImg, bSpherical, float2((x1.x + 0.5f) * view.texScale.x, (x1.y + 0.5f) * view.texScale.y));
    if (depth1 == 0.0f) return maxDist;
    const float3 Xb = transpose(view.R) * (X1 * (depth1 / depth1Expected) - view.t);
    float2 xb;
    if (MODEL == MODEL_SPHERICAL_REF) {
        if (!SphericalProjectFast(AsSpherical(cams[0]), Xb, xb))
            return maxDist;
    } else {
        if (!PinholeProject(AsPinhole(cams[0]), Xb, xb))
            return maxDist;
    }
    float dx = float(p.x) - xb.x;
    if (MODEL == MODEL_SPHERICAL_REF) {
        const float width = float(cams[0].size.x);
        dx -= width * rint(dx / width);
    }
    const float distSq = Square(dx) + Square(float(p.y) - xb.y);
    return min(maxDist, sqrt(distSq + sqrt(distSq)*2.0f));
}

// multi-view score: fills costVector[0..nNumViews-1]. With a spherical view, the view gate of the CPU
// estimator (DepthEstimatorT::ScorePixel), at the hypothesis depth: a view seeing the hypothesis point at
// a triangulation angle below fMinAngle does not constrain its depth and scores as a view that does not
// see the patch, unless every view does: then none is gated, else next to the epipoles, where the true
// depth gates every view, the closer hypotheses that some view sees at a larger angle would win for
// that alone; pinhole scenes run without the gate
static inline void MultiViewScorePlane(thread const RefCache& cache, thread const RefPixel& ref,
                                       const array<texture2d<float>, MAX_VIEWS> images,
                                       const array<texture2d<float>, MAX_VIEWS> depthImages,
                                       constant Camera* cams, constant ModelView* views, device const float2* refTrig,
                                       int2 p, float4 plane, float lowDepth,
                                       constant Params& prm, thread float* costVector) {
    const int nNumViews = prm.nNumViews;
    uint gatedViews = 0; // bit imgId-1: view imgId
    if (MODEL != MODEL_PINHOLE) {
        const float3 X = ref.F[2] * plane.w; // the hypothesis point in the reference frame
        for (int imgId = 1; imgId <= nNumViews; ++imgId) {
            const float3 D = X - views[imgId].C;
            if (dot(X, D) > prm.cosMinViewAngle * sqrt(length_squared(X) * length_squared(D)))
                SetBit(gatedViews, (uint)(imgId-1));
        }
        if (popcount(gatedViews) == (uint)nNumViews)
            gatedViews = 0;
    }
    for (int imgId = 1; imgId <= nNumViews; ++imgId)
        costVector[imgId-1] = IsBitSet(gatedViews, (uint)(imgId-1)) ? fBadCost :
            ScorePlane(cache, ref, images[imgId], cams, views, refTrig, imgId, p, plane, lowDepth, prm);
    if (GEOM)
        for (int imgId = 0; imgId < nNumViews; ++imgId)
            costVector[imgId] += 0.1f * GeometricConsistencyWeight(depthImages[imgId], ref, cams, views, imgId+1, plane, p, prm);
}
// the depth at p of the neighbor plane at np: DepthGeometry.h InterpolatePlaneDepth of the reference model
static inline float InterpolateNeighborDepth(constant Camera* cams, device const float2* refTrig,
                                             int2 p, int2 np, float depth, float3 normal, constant Params& prm) {
    if (MODEL == MODEL_SPHERICAL_REF)
        return InterpolatePlaneDepth(RefTexelRay(cams[0], refTrig, p, 0, 0), RefTexelRay(cams[0], refTrig, np, 0, 0),
                                     depth, normal, prm.fDepthMin, prm.fDepthMax);
    return PinholeInterpolatePlaneDepth(AsPinhole(cams[0]), p, np, depth, normal, prm.fDepthMin, prm.fDepthMax);
}
static inline float MultiViewScoreNeighborPlane(thread const RefCache& cache, thread const RefPixel& ref,
                                                const array<texture2d<float>, MAX_VIEWS> images,
                                                const array<texture2d<float>, MAX_VIEWS> depthImages,
                                                constant Camera* cams, constant ModelView* views, device const float2* refTrig,
                                                int2 p, int2 np, float4 plane, float lowDepth,
                                                constant Params& prm, thread float* costVector) {
    plane.w = InterpolateNeighborDepth(cams, refTrig, p, np, plane.w, plane.xyz, prm);
    MultiViewScorePlane(cache, ref, images, depthImages, cams, views, refTrig, p, plane, lowDepth, prm, costVector);
    return plane.w;
}
// the surface normal at p from its 4-neighbors' depths (up, down, left, right): DepthGeometry.h
// NormalFromDepthGradient of the reference model
static inline float3 SurfaceNormal(constant Camera& refCam, thread const RefPixel& ref, float depth, int2 p, float4 ndepths) {
    const float2 gradient = DepthGradient4(depth, ndepths);
    if (MODEL == MODEL_SPHERICAL_REF)
        return NormalFromDepthGradient(ref.F, depth, gradient);
    return PinholeNormalFromDepthGradient(AsPinhole(refCam), p, depth, gradient);
}

// random normals facing the camera along the pixel's viewing ray (any length): the distribution is
// isotropic, so it is the same in the ray's local frame for every model
static inline float3 GenerateRandomNormal(float3 viewDirection, thread uint& s) {
    const float3 n = GenerateRandomUnitVector(s);
    return dot(n, viewDirection) > 0.0f ? -n : n;
}
static inline float3 GeneratePerturbedNormal(float3 viewDirection, float3 normal, thread uint& s, float perturbation) {
    const float theta = (curand_uniform(s) - 0.5f) * perturbation;
    const float sinT = sin(theta), cosT = cos(theta);
    const float3 axis = GenerateRandomUnitVector(s);
    const float aDotN = dot(axis, normal);
    const float3 axCrossN = cross(axis, normal);
    const float3 np = normal*cosT + axCrossN*sinT + axis*(aDotN*(1.0f-cosT));
    return dot(np, viewDirection) >= 0.0f ? normal : np;
}

// smoothness coupling of the plane hypotheses (PatchMatchCUDA.cu): ProcessPixel fits per pixel a smoothed
// plane to the current estimates of its 4 adjacent neighbors (of the other checkerboard color, so not
// rewritten by the pass) and scales the aggregated cost of every hypothesis, the stored one included, by
// SmoothnessFactor
constant float smoothBonus = 0.2f; // the cost reduction of a hypothesis on the smoothed plane
constant float smoothSigmaDepth = -1.0f/(2.0f*0.02f*0.02f); // distance to the smoothed plane relative to the depth, sigma 0.02
#define SMOOTH_ANGLE_NORMAL (13.0f*M_PI_F/180.0f) // normal sigma 13 deg
constant float smoothSigmaNormal = 2.0f*(-1.0f/(2.0f*SMOOTH_ANGLE_NORMAL*SMOOTH_ANGLE_NORMAL)); // applied to 1-cos: acos(c)^2 ~ 2*(1-c)
#undef SMOOTH_ANGLE_NORMAL
constant float smoothSigmaColor = -1.0f/(2.0f*25.0f/255.0f*25.0f/255.0f); // neighbor weight: gray similarity, as the patch bilateral weight
constant float smoothSigmaConf = -1.0f/(2.0f*0.3f*0.3f); // neighbor weight: confidence from its cost, as the view-selection weight

// the factor scaling the aggregated cost of a plane hypothesis (normal, depth along the pixel ray) by its
// agreement with the smoothed plane of the pixel: its point Xs on the pixel ray and its normal ns
static inline float SmoothnessFactor(float3 ray, float3 Xs, float3 ns, float4 plane) {
    const float depth = plane.w;
    const float d = dot(ns, ray*depth - Xs) / depth;
    return 1.0f - smoothBonus * exp(Square(d)*smoothSigmaDepth + (1.0f - dot(ns, plane.xyz))*smoothSigmaNormal);
}

// per-pixel processing (PatchMatchCUDA.cu ProcessPixel). Every aggregated cost of the propagation and of the
// refinement, the stored one included, is scaled by the SmoothnessFactor of the hypothesis against the
// pixel's smoothed plane, fitted to its 4 adjacent neighbors, when the pixel has one.
// An equirectangular reference wraps the neighbor columns across
// the seam, which keeps the red/black parity as its width is even, and does not cross the poles (over
// a pole the column moves by half the width, which flips the parity when the width is twice the
// height); its rows next to the poles are not estimated, as on the CPU.
static inline void ProcessPixel(const array<texture2d<float>, MAX_VIEWS> images,
                                     const array<texture2d<float>, MAX_VIEWS> depthImages,
                                     constant Camera* cams, constant ModelView* views, device const float2* refTrig,
                                     device float4* planes, device const float* lowDepths,
                                     device float* costs, device uint* rngStates, device uint* selectedViews,
                                     int2 p, int iter, constant Params& prm) {
    const int width = prm.width, height = prm.height;
    if (p.x >= width || p.y >= height) return;
    if (MODEL == MODEL_SPHERICAL_REF && (p.y < HALF || p.y >= height - HALF)) return;
    const int idx = p.y*width + p.x;
    uint state = rngStates[idx];
    float lowDepth = prm.bLowResProcessed ? lowDepths[idx] : 0.0f;
    RefCache refCache; ComputeRefCache(images[0], views, p, refCache);
    const RefPixel ref = MakeRefPixel(cams[0], refTrig, p);

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
    const bool bWrapCols = MODEL == MODEL_SPHERICAL_REF;
    const int neighborPositions[4] = {
        idx-width,
        idx+width,
        bWrapCols && p.x == 0 ? idx-1+width : idx-1,
        bWrapCols && p.x == width-1 ? idx+1-width : idx+1
    };
    bool valid[8] = {false,false,false,false,false,false,false,false};
    int positions[8];
    float neighborDepths[8];
    float costArray[8][MAX_VIEWS];

    for (int posId = 0; posId < 8; ++posId) {
        int2 bestNx = int2(0,0); float bestConf = FLT_MAX;
        for (int dirId = 0; dirId < numDirs[posId]; ++dirId) {
            int2 np = int2(p.x+dirs[posId][dirId].x, p.y+dirs[posId][dirId].y);
            if (bWrapCols) {
                if (!(np.y>=0 && np.y<height)) continue;
                np.x = np.x < 0 ? np.x + width : (np.x >= width ? np.x - width : np.x);
            } else {
                if (!(np.x>=0 && np.y>=0 && np.x<width && np.y<height)) continue;
            }
            const float nconf = costs[np.y*width+np.x];
            if (bestConf > nconf) { bestConf = nconf; bestNx = np; }
        }
        if (bestConf < FLT_MAX) {
            valid[posId] = true;
            positions[posId] = bestNx.y*width+bestNx.x;
            neighborDepths[posId] = MultiViewScoreNeighborPlane(refCache, ref, images, depthImages, cams, views, refTrig,
                                        p, bestNx, planes[positions[posId]], lowDepth, prm, costArray[posId]);
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

    // the smoothed plane of the pixel, if it has one: the estimates of its 4 adjacent neighbors (of the other
    // color, not rewritten by this pass) weighted by their gray similarity to the pixel times their confidence
    // give the weighted mean normal through the weighted mean point; its point on the pixel ray and its normal
    const float3 pixelRay = ref.F[2];
    bool bSmooth = false;
    float3 smoothX = float3(0), smoothNormal = float3(0);
    {
        const int2 offsets[4] = {int2(0,-1), int2(0,1), int2(-1,0), int2(1,0)};
        const float refPix = FetchRefTexel(images[0], views, p.x, p.y);
        float3 sumN = float3(0), sumX = float3(0);
        float sumW = 0;
        for (int posId = 0; posId < 4; ++posId) {
            if (!valid[posId]) continue;
            const float4 nbPlane = planes[neighborPositions[posId]];
            if (nbPlane.w <= 0) continue;
            const int nr = p.y + offsets[posId].y;
            int nc = p.x + offsets[posId].x;
            if (bWrapCols) nc = nc < 0 ? nc + width : (nc >= width ? nc - width : nc);
            const float w = exp(Square(FetchRefTexel(images[0], views, nc, nr) - refPix)*smoothSigmaColor + Square(costs[neighborPositions[posId]])*smoothSigmaConf);
            sumW += w;
            sumN += nbPlane.xyz * w;
            sumX += RefTexelRay(cams[0], refTrig, p, offsets[posId].x, offsets[posId].y) * (nbPlane.w * w);
        }
        const float normN = length(sumN);
        if (sumW > 1e-4f && normN > 1e-3f * sumW) {
            smoothNormal = sumN / normN;
            const float nRay = dot(smoothNormal, pixelRay);
            if (nRay < -1e-2f) {
                const float depthSmooth = dot(smoothNormal, sumX / sumW) / nRay;
                smoothX = pixelRay * depthSmooth;
                bSmooth = depthSmooth > 0;
            }
        }
    }

    float4 plane = planes[idx];
    float cost = costs[idx];
    uint newSelectedViews = 0;
    for (int imgId = 0; imgId < nNumViews; ++imgId) if (viewWeights[imgId]) SetBit(newSelectedViews, (uint)imgId);
    float finalCosts[8];
    for (int posId = 0; posId < 8; ++posId)
        finalCosts[posId] = valid[posId] ? AggregateMultiViewScores(viewWeights, costArray[posId], nNumViews) : FLT_MAX;
    if (bSmooth)
        for (int posId = 0; posId < 8; ++posId)
            if (valid[posId]) {
                float4 nbPlane = planes[positions[posId]];
                nbPlane.w = neighborDepths[posId];
                finalCosts[posId] *= SmoothnessFactor(pixelRay, smoothX, smoothNormal, nbPlane);
            }
    const int minCostIdx = FindMinIndex(finalCosts, 8);
    float costVector[MAX_VIEWS];
    MultiViewScorePlane(refCache, ref, images, depthImages, cams, views, refTrig, p, plane, lowDepth, prm, costVector);
    cost = AggregateMultiViewScores(viewWeights, costVector, nNumViews);
    if (bSmooth)
        cost *= SmoothnessFactor(pixelRay, smoothX, smoothNormal, plane);
    if (finalCosts[minCostIdx] < cost) {
        plane = planes[positions[minCostIdx]];
        plane.w = neighborDepths[minCostIdx];
        cost = finalCosts[minCostIdx];
        selectedViews[idx] = newSelectedViews;
    }
    const float depth = plane.w;

    // refine
    const float perturbationDepth = 0.005f;
    const float perturbationNormal = 0.01f * M_PI_F;
    const float3 viewDirection = pixelRay;
    const float depthPerturbed = GeneratePerturbedDepth(depth, state, perturbationDepth, prm);
    const float3 perturbedNormal = GeneratePerturbedNormal(viewDirection, plane.xyz, state, perturbationNormal);
    const float3 normalRand = GenerateRandomNormal(viewDirection, state);
    int numValidPlanes = 3;
    float3 surfaceNormal = float3(0,0,0);
    if (valid[0] && valid[1] && valid[2] && valid[3]) {
        const float4 ndepths = float4(planes[neighborPositions[0]].w, planes[neighborPositions[1]].w,
                                      planes[neighborPositions[2]].w, planes[neighborPositions[3]].w);
        surfaceNormal = SurfaceNormal(cams[0], ref, depth, p, ndepths);
        numValidPlanes = 4;
    }
    const float depths[4] = { depthPerturbed, depth, depth, depth };
    const float3 normals[4] = { plane.xyz, perturbedNormal, normalRand, surfaceNormal };
    for (int i = 0; i < numValidPlanes; ++i) {
        float4 newPlane = float4(normals[i], depths[i]);
        MultiViewScorePlane(refCache, ref, images, depthImages, cams, views, refTrig, p, newPlane, lowDepth, prm, costVector);
        float costPlane = AggregateMultiViewScores(viewWeights, costVector, nNumViews);
        if (bSmooth)
            costPlane *= SmoothnessFactor(pixelRay, smoothX, smoothNormal, newPlane);
        if (cost > costPlane) { cost = costPlane; plane = newPlane; }
    }

    planes[idx] = plane;
    costs[idx] = cost;
    rngStates[idx] = state;
}

// the score of the current plane estimate (PatchMatchCUDA.cu InitializePixelScore)
static inline void InitializePixelScore(const array<texture2d<float>, MAX_VIEWS> images,
                                             const array<texture2d<float>, MAX_VIEWS> depthImages,
                                             constant Camera* cams, constant ModelView* views, device const float2* refTrig,
                                             device float4* planes, device const float* lowDepths,
                                             device float* costs, device uint* rngStates, device uint* selectedViews,
                                             int2 p, constant Params& prm) {
    const int width = prm.width, height = prm.height;
    if (p.x >= width || p.y >= height) return;
    const int idx = p.y*width + p.x;
    if (MODEL == MODEL_SPHERICAL_REF && (p.y < HALF || p.y >= height - HALF)) {
        // the rows next to the poles are not estimated (ProcessPixel): no depth, never a neighbor
        planes[idx] = float4(0);
        costs[idx] = FLT_MAX;
        selectedViews[idx] = 0;
        return;
    }
    float lowDepth = prm.bLowResProcessed ? lowDepths[idx] : 0.0f;
    RefCache refCache; ComputeRefCache(images[0], views, p, refCache);
    const RefPixel ref = MakeRefPixel(cams[0], refTrig, p);
    const float3 viewDirection = ref.F[2];

    uint state = (uint)(p.x*1973u + p.y*9277u + 1234u); // analogue of curand_init(1234, y, x)
    float4 plane = planes[idx];
    if (plane.w <= 0.0f) {
        plane.xyz = GenerateRandomNormal(viewDirection, state);
        plane.w = curand_uniform(state)*(prm.fDepthMax - prm.fDepthMin) + prm.fDepthMin;
    } else if (dot(plane.xyz, viewDirection) >= 0.0f) {
        plane.xyz = GenerateRandomNormal(viewDirection, state);
    }
    const int nNumViews = prm.nNumViews;
    const int nInitTopK = prm.nInitTopK;
    float costVector[MAX_VIEWS];
    MultiViewScorePlane(refCache, ref, images, depthImages, cams, views, refTrig, p, plane, lowDepth, prm, costVector);
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
                            constant ModelView* views [[buffer(8)]],
                            device const float2* refTrig [[buffer(9)]],
                            uint2 gid [[thread_position_in_grid]]) {
    InitializePixelScore(images, depthImages, cams, views, refTrig, planes, lowDepths, costs, rngStates, selectedViews, int2(gid), prm);
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
                              constant ModelView* views [[buffer(8)]],
                              device const float2* refTrig [[buffer(9)]],
                              uint2 gid [[thread_position_in_grid]],
                              uint2 lid [[thread_position_in_threadgroup]]) {
    int2 p = int2(gid.x, gid.y*2 + ((lid.x % 2 == 0) ? 0 : 1));
    ProcessPixel(images, depthImages, cams, views, refTrig, planes, lowDepths, costs, rngStates, selectedViews, p, iter, prm);
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
                            constant ModelView* views [[buffer(8)]],
                            device const float2* refTrig [[buffer(9)]],
                            uint2 gid [[thread_position_in_grid]],
                            uint2 lid [[thread_position_in_threadgroup]]) {
    int2 p = int2(gid.x, gid.y*2 + ((lid.x % 2 == 0) ? 1 : 0));
    ProcessPixel(images, depthImages, cams, views, refTrig, planes, lowDepths, costs, rngStates, selectedViews, p, iter, prm);
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

// the hand mirror of the camera models evaluated for its unit test (PatchMatch::EvaluateModelMath)
kernel void TestModelMath(device ModelMathSample* samples [[buffer(0)]],
                          constant uint& numSamples [[buffer(1)]],
                          uint i [[thread_position_in_grid]]) {
    if (i < numSamples)
        EvaluateModelMath(samples[i]);
}
