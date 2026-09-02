/*
* SceneRefineCommon.h
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

#ifndef _MVS_SCENEREFINECOMMON_H_
#define _MVS_SCENEREFINECOMMON_H_

// This header is the single reference for scalar math that the CPU
// (SceneRefine.cpp, MeshRefine) and CUDA (SceneRefineCUDA.cpp/.cu,
// MeshRefineCUDA) mesh-refinement backends must compute identically, plus
// the OPTREFINE configuration space (mirrors OPTDENSE, see
// DepthMap.h/DepthMap.cpp) that later refine work packages read from.
//
// Include rules:
//  - Everything above the "#ifndef __CUDACC__" guards below (the REFINE_HD
//    macro, the MVS::Refine constants and ZnccReliability()) is compiled by
//    nvcc as part of SceneRefineCUDA.cu's device code, so it must never drag
//    in OpenCV or SEACAVE types -- plain float/int only (the same constraint
//    that keeps Util.h from referencing Matrix3x4: libs/Common stays
//    untouched by this header).
//  - Everything below those guards (OPTREFINE, PrepareRefineImage) is
//    host-only. Like DepthMap.h/OPTDENSE, it relies on the including
//    translation unit having already done `#include "Common.h"` (for
//    MVS_API/DECOPT_SPACE) and `#include "Scene.h"` (for Image/PlatformArr)
//    before this header; it is never reached while compiling as __CUDACC__.


// I N C L U D E S /////////////////////////////////////////////////

#ifndef __CUDACC__
#include "Image.h"
#endif


// D E F I N E S ///////////////////////////////////////////////////

#ifdef __CUDACC__
#define REFINE_HD __host__ __device__
#else
#define REFINE_HD
#endif


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

namespace Refine {

// window used to compute local image statistics (mean/variance/ZNCC) during
// photo-consistency optimization; shared by both backends so a 7x7 window is
// what each one sees (CUDA used to run its own HalfSize=2, i.e. 5x5)
constexpr int HalfSize = 3;
constexpr int Border = HalfSize;
constexpr int WindowSize = HalfSize*2+1;
constexpr int WindowArea = WindowSize*WindowSize;

// the window statistics below are accumulated over the VALID (successfully warped) pixels of
// the window only, so a pixel next to an occlusion boundary is described by the few pixels that
// really matched instead of by whatever the unwarped ones happened to hold; below this many
// valid samples the mean/variance/covariance are too noisy to steer a vertex and the centre
// pixel is rejected outright
constexpr int MinWindowCount = 25;

// ZNCC reliability weight: down-weights the photometric gradient/score at
// pixels whose local window is low-texture (low variance on either side);
// the same expression on both backends (CPU MeshRefine::ComputeWindowStats,
// SceneRefine.cpp; CUDA kernelComputeWindowStats, SceneRefineCUDA.cu) --
// hoisted here so both call the same code instead of two copies that could
// silently drift.
// the window variances are floored at VarFloor (a flat window is not a measurement) and the
// reliability saturates on the scale of ReliabilityVarOffset
constexpr float VarFloor = 1e-4f;
constexpr float ReliabilityVarOffset = 0.0015f;
REFINE_HD inline float ZnccReliability(float varA, float varB)
{
	const float minVar(varA < varB ? varA : varB);
	return minVar/(minVar+ReliabilityVarOffset);
}
// derivative of ZnccReliability with respect to varB: non-zero only where B's window is the
// less textured of the two and above the floor (the floor makes the weight a constant there)
REFINE_HD inline float ZnccReliabilityDerivativeVarB(float varA, float varB)
{
	if (!(varB < varA) || !(varB > VarFloor))
		return 0.f;
	const float d(varB+ReliabilityVarOffset);
	return ReliabilityVarOffset/(d*d);
}

// local window statistics of the reference image A and the warped image B at one pixel,
// reduced from the six masked sums of the window; see WindowStatsFromSums()
struct WindowStats {
	float muA, muB; // window means
	float varA, varB; // window variances, floored at 1e-4
	float cov; // window covariance
};

// reduce the six masked window sums to means/variances/covariance and apply the two rejection
// gates: a pixel whose two windows disagree on brightness by more than gateMeanDiff, or whose
// variances differ by more than a factor of gateVarRatio, is not a photo-consistency
// measurement of the same surface -- it is a specular highlight, a shadow boundary or a partial
// occlusion the depth test did not catch -- and steering a vertex with its (large, confidently
// wrong) gradient is worse than not steering it.
// n is the number of valid samples, sA/sB/sAA/sBB/sAB their masked sums; gates <= 0 disable.
// Returns false if the pixel must be rejected, in which case stats are left undefined.
REFINE_HD inline bool WindowStatsFromSums(float n, float sA, float sB, float sAA, float sBB, float sAB,
	float gateMeanDiff, float gateVarRatio, WindowStats& s)
{
	if (n < (float)MinWindowCount)
		return false;
	const float invN(1.f/n);
	s.muA = sA*invN;
	s.muB = sB*invN;
	const float vA(sAA*invN - s.muA*s.muA), vB(sBB*invN - s.muB*s.muB);
	s.varA = vA > VarFloor ? vA : VarFloor;
	s.varB = vB > VarFloor ? vB : VarFloor;
	const float meanDiff(s.muA > s.muB ? s.muA-s.muB : s.muB-s.muA);
	if (gateMeanDiff > 0.f && meanDiff > gateMeanDiff)
		return false;
	if (gateVarRatio > 0.f && (s.varA > gateVarRatio*s.varB || s.varB > gateVarRatio*s.varA))
		return false;
	s.cov = sAB*invN - s.muA*s.muB;
	return true;
}

// per-pixel ZNCC, the photometric gradient scale (derivative of the reliability-weighted ZNCC
// with respect to the warped pixel value, negated as the optimizer minimizes 1-ZNCC) and the
// reliability weight, from the window statistics and the two centre pixel values.
// The WindowArea/n factor restores the magnitude a full window would have produced, so a
// partially valid window is not silently down-weighted on top of already being rejected below
// MinWindowCount; it is exactly 1 when every pixel of the window is valid.
REFINE_HD inline void ZnccAndDerivative(const WindowStats& s, float n, float pixA, float pixB,
	float& zncc, float& dzncc, float& conf)
{
	const float invSqrtVAVB(1.f/sqrtf(s.varA*s.varB));
	zncc = s.cov*invSqrtVAVB;
	const float dZ((pixA-s.muA)*invSqrtVAVB - zncc*(pixB-s.muB)/s.varB);
	conf = ZnccReliability(s.varA, s.varB);
	dzncc = -conf*dZ*((float)WindowArea/n);
}

} // namespace Refine


#ifndef __CUDACC__

DECOPT_SPACE(OPTREFINE)

namespace OPTREFINE {
// configuration variables, mirroring OPTDENSE. Consumed today: nImageGradient
// (the derivative stencil in ComputeRefineImageGradient), nOptimizer (the
// stepper arm in Scene::RefineMesh/RefineMeshCUDA) and the two pixel rejection
// gates fGateMeanDiff/fGateVarRatio (WindowStatsFromSums, read by both
// backends' window statistics); the rest are declared, staged mechanism for
// later refine work to read
extern MVS_API int nIgnoreMaskLabel; // label id used during ignore mask filter (<0 - disabled)
extern MVS_API int nImageGradient; // image derivative stencil (0 - 3x5 separable, 1 - central (default), 2 - Sobel, 3 - bilinear interpolant derivative)
extern MVS_API float fGateMeanDiff; // reject a pixel pair whose local mean differs by more than this (0 - disabled)
extern MVS_API float fGateVarRatio; // reject a pixel pair whose local variance ratio exceeds this (0 - disabled)
extern MVS_API int nOptimizer; // vertex position optimizer (0 - bold, 1 - fixed; the trust-ratio, Barzilai-Borwein and momentum arms were measured and lost, see the design document)
} // namespace OPTREFINE

class Scene;

// gather the neighbor views one image contributes refinement pairs with: a scene that never had
// SelectNeighborViews run (a mesh handed to the refiner directly) has no neighbors stored, so they
// are recovered here first, then filtered down to the best ones by the shared thresholds below.
// Both backends call this per image -- the CPU from its thread pool (holding no lock: nothing here
// touches shared refiner state), CUDA serially -- so the two reach the same pair set on the same
// scene. Returns false if the image is invalid and contributes no pair.
MVS_API bool SelectRefineNeighbors(Scene& scene, uint32_t idxImage, unsigned nMaxViews, ViewScoreArr& neighbors);

// load, gray-convert, blur and resize one refine image at the given scale;
// the hoisted common part of CPU MeshRefine::ThInitImage (SceneRefine.cpp)
// and CUDA MeshRefineCUDA::InitImages (SceneRefineCUDA.cpp): both backends
// call this and keep their own tail (CPU: the image-gradient stencil plus
// the optional mean/var precompute; CUDA: nothing else). Behaviour matches
// what each backend did inline before -- same load/gray/blur/resize order,
// same interpolation, same flags. Returns false if the image failed to load.
MVS_API bool PrepareRefineImage(Image& imageData, const PlatformArr& platforms,
	unsigned nResolutionLevel, unsigned nMinResolution, float scale, float sigma, Image32F& gray);

// per-view keep-mask at the working image size PrepareRefineImage just produced for this scale;
// called right after it by both backends (CPU MeshRefine::ThInitImage, SceneRefine.cpp; CUDA
// MeshRefineCUDA::InitImages, SceneRefineCUDA.cpp). Reads OPTREFINE::nIgnoreMaskLabel itself, like
// ComputeRefineImageGradient reads OPTREFINE::nImageGradient. keepMask is left empty (== keep
// everything, zero cost on the hot path) when masks are disabled or the image has no mask file to
// load; a configured-but-missing mask file is not an error and is not reported here -- every image
// is checked once, up front, by apps/RefineMesh (the same entry that assigns image.maskName), so
// this per-scale, per-backend call site never warns on its own.
MVS_API void PrepareRefineImageMask(const Image& imageData, const cv::Size& size, BitMatrix& keepMask);

// image derivative estimate used by the photometric gradient, the same on both
// backends (the CPU samples it bilinearly from its gradient image, CUDA from a
// texture of it): OPTREFINE::nImageGradient selects the stencil, default the
// central difference -- the widest stencils smooth the very detail the
// refinement is there to recover, and the images are already Gaussian-blurred
// per scale by PrepareRefineImage. Mode 3 (the derivative of the bilinear
// interpolant at the warped sample, no precomputed stencil image) is not a
// stencil this function builds: both backends skip calling it entirely and
// sample the raw image directly at the point that needs the derivative.
MVS_API void ComputeRefineImageGradient(const Image32F& gray, Image32F& gradX, Image32F& gradY);


// CPU/CUDA parity diagnostic: export per-vertex gradients and
// per-pixel pair maps to disk so a Python harness (bench/refine_parity.py)
// can compare the two backends on the identical frozen mesh. Entirely
// env-var gated -- no public RefineMesh option -- and the only place in the
// refine code that reads an environment variable; every other function must
// keep going through OPTREFINE/CLI options instead.
namespace RefineDebug {

// OMVS_REFINE_DEBUG_DIR: export directory, read once and cached; empty (the
// env var unset) disables every export below and must cost nothing on the
// hot path -- callers guard with `if (!RefineDebug::Dir().empty())` before
// doing any of the extra work needed to fill in the arguments below.
MVS_API const String& Dir();

// OMVS_REFINE_DEBUG_PAIR=A,B: the single ordered image pair whose per-pixel
// maps get exported (every pair would be too much data); returns false if
// the env var is unset or does not parse.
MVS_API bool Pair(uint32_t& idxImageA, uint32_t& idxImageB);

// per-vertex gradient export: refine_grad_s<scale>_i<iter>.ply, all arrays
// `numVertices` long; `boundary` is a 0/1 byte per vertex. No-op if Dir()
// is empty.
MVS_API void ExportGradients(unsigned nScale, unsigned iter, uint32_t numVertices,
	const Point3f* pos, const Point3f* combined,
	const Point3f* photo, const float* photoCount,
	const Point3f* smooth1, const Point3f* smooth2,
	const uint8_t* boundary);

// per-pixel pair-map export, one 32-bit-float PFM per call:
// pair_<A>_<B>_s<scale>_i<iter>_<name>.pfm; `data` must be `width*height`
// floats, tightly packed (row-major, no padding). No-op if Dir() is empty.
MVS_API void ExportPairMap(unsigned nScale, unsigned iter, uint32_t idxImageA, uint32_t idxImageB,
	const char* name, const float* data, int width, int height);

// per-pixel validity mask export: pair_<A>_<B>_s<scale>_i<iter>_mask.png
// (8-bit, 255 = valid); `mask` is `width*height` bytes, any non-zero value
// counts as valid -- both callers pass 0/1 (the CPU converts its BitMatrix,
// CUDA's device mask is already 0/1). No-op if Dir() is empty.
MVS_API void ExportPairMask(unsigned nScale, unsigned iter, uint32_t idxImageA, uint32_t idxImageB,
	const uint8_t* mask, int width, int height);

} // namespace RefineDebug

#endif // __CUDACC__

} // namespace MVS

#endif // _MVS_SCENEREFINECOMMON_H_
