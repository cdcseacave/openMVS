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

// warp depth test (MeshRefine::IsDepthSimilar / kernelImageMeshWarp): a pixel of A warped into
// B is kept iff one of the 4 depth-map pixels around its projection is valid and not more than
// this far IN FRONT of the point (depth + DepthConstBias >= z): an occlusion test only, points
// behind the surface B sees are accepted. Scene units, the value the CPU has always used
// (MESHOPT_DEPTHCONSTBIAS); CUDA used to run a symmetric 1 % relative test, the CPU's dead
// branch, and rejected ~100 grazing pixels per pair that the CPU keeps.
constexpr float DepthConstBias = 0.05f;

// ZNCC reliability weight: down-weights the photometric gradient/score at
// pixels whose local window is low-texture (low variance on either side);
// identical expression on both backends today (CPU ComputeLocalZNCC,
// SceneRefine.cpp; CUDA kernelComputeImageDZNCC, SceneRefineCUDA.cu) --
// hoisted here so both call the same code instead of two copies that could
// silently drift.
REFINE_HD inline float ZnccReliability(float varA, float varB)
{
	const float minVar(varA < varB ? varA : varB);
	return minVar/(minVar+0.0015f);
}

} // namespace Refine


#ifndef __CUDACC__

DECOPT_SPACE(OPTREFINE)

namespace OPTREFINE {
// configuration variables (nothing reads these yet; later refine work
// packages consume them -- this is only the mechanism, mirroring OPTDENSE)
extern MVS_API int nIgnoreMaskLabel; // label id used during ignore mask filter (<0 - disabled)
extern MVS_API int nPhotoTerm; // photo-consistency term formulation (0 - magnitude, 1 - sign vote, 2 - tanh)
extern MVS_API int nPhotoNorm; // photo-consistency gradient normalization (0 - pair count, 1 - confidence-weighted pixel sum)
extern MVS_API int nImageGradient; // image derivative stencil (0 - 3x5 separable, 1 - central, 2 - Sobel)
extern MVS_API int nBoundaryMode; // boundary vertex handling (0 - legacy, 1 - freeze, 2 - rim Laplacian)
extern MVS_API float fGateMeanDiff; // reject a pixel pair whose local mean differs by more than this (0 - disabled)
extern MVS_API float fGateVarRatio; // reject a pixel pair whose local variance ratio exceeds this (0 - disabled)
extern MVS_API int nOptimizer; // vertex position optimizer (0 - bold, 1 - fixed, 2 - rprop, 3 - adam, 4 - bb, 5 - ceres)
} // namespace OPTREFINE

// load, gray-convert, blur and resize one refine image at the given scale;
// the hoisted common part of CPU MeshRefine::ThInitImage (SceneRefine.cpp)
// and CUDA MeshRefineCUDA::InitImages (SceneRefineCUDA.cpp): both backends
// call this and keep their own tail (CPU: the image-gradient stencil plus
// the optional mean/var precompute; CUDA: nothing else). Behaviour matches
// what each backend did inline before -- same load/gray/blur/resize order,
// same interpolation, same flags. Returns false if the image failed to load.
MVS_API bool PrepareRefineImage(Image& imageData, const PlatformArr& platforms,
	unsigned nResolutionLevel, unsigned nMinResolution, float scale, float sigma, Image32F& gray);

// image derivative estimate used by the photometric gradient, the same on both
// backends (the CPU samples it bilinearly from its gradient image, CUDA from a
// texture of it): OPTREFINE::nImageGradient selects the stencil, default the
// noise-robust 3x5 separable kernel (CreateDerivativeKernel3x5, unit scale).
MVS_API void ComputeRefineImageGradient(const Image32F& gray, Image32F& gradX, Image32F& gradY);


// CPU/CUDA parity diagnostic (Part A WP2): export per-vertex gradients and
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
	const Point3f* photo, const float* photoNorm,
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
