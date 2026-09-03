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

// Everything the CPU (SceneRefine.cpp, MeshRefine) and CUDA (SceneRefineCUDA.cpp/.cu,
// MeshRefineCUDA) mesh-refinement backends share: the scalar math both must compute identically,
// the OPTREFINE configuration space (mirrors OPTDENSE, see DepthMap.h/DepthMap.cpp), the per-view
// image and mask preparation, and the vertex-position stepper (MeshRefineStep) that turns one
// energy evaluation into one step -- one implementation, no CUDA twin to drift.
//
// Include rules:
//  - Everything above the "#ifndef __CUDACC__" guard below (the REFINE_HD macro, the MVS::Refine
//    constants and the window/ZNCC math) is compiled by nvcc as part of SceneRefineCUDA.cu's
//    device code, so it must never drag in OpenCV or SEACAVE types -- plain float/int only.
//  - Everything below the guard is host-only. Like DepthMap.h/OPTDENSE, it relies on the
//    including translation unit having already done `#include "Common.h"` (for
//    MVS_API/DECOPT_SPACE) before this header.


// I N C L U D E S /////////////////////////////////////////////////

#ifndef __CUDACC__
#include "Image.h"
#include "Mesh.h"
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
// Returns false if the pixel must be rejected; stats must not be read then.
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
// configuration variables, mirroring OPTDENSE; read from --refine-config-file by apps/RefineMesh
extern MVS_API int nIgnoreMaskLabel; // label id used during ignore mask filter (<0 - disabled)
extern MVS_API int nImageGradient; // image derivative stencil (0 - 3x5 separable, 1 - central (default), 2 - Sobel, 3 - bilinear interpolant derivative)
extern MVS_API float fGateMeanDiff; // reject a pixel pair whose local mean differs by more than this (0 - disabled)
extern MVS_API float fGateVarRatio; // reject a pixel pair whose local variance ratio exceeds this (0 - disabled)
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
// load; a configured-but-missing mask file is not an error and is not reported here: every image
// is checked once, up front, by apps/RefineMesh (the same entry that assigns image.maskName).
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


// Vertex-position stepper shared by the CPU and CUDA mesh-refinement backends: a bold driver
// that works in pixels and in ZNCC, so its trajectory and its stopping point are properties of
// the surface rather than of the scene's units, the image resolution or the pair count:
//
//   * every step length is a pixel length converted through the per-vertex footprint s_v
//     (scene units per pixel: depth/f minimized over the pair-directions that saw v);
//   * progress is judged on S, the reliability-weighted mean of (1 - ZNCC) over every scored
//     pixel, which lies in [0,2] whatever the scene.
//
// Per evaluation, with rho the rigidity-elasticity ratio and w the regularity weight:
//
//     g_v = photoGrad_v / c_v                             (zero if c_v < 2)
//     m   = median |g_v| / s_v over the seen vertices     (computed ONCE per scale, then held)
//     D_v = -eta * (g_v / (Kappa m) + w * (rho * bilap_v - (1 - rho) * lap_v))
//
// so the median seen vertex moves eta/Kappa px at the scale's first evaluation and every other
// vertex moves in PROPORTION to its own gradient. m is a GLOBAL factor on purpose: recomputing
// it every evaluation would renormalize the median vertex back to the same step every time and
// defeat the stop rule, and a per-vertex normalization flattens the gradient distribution (a
// measured regression, see the design document). The caller owns the vertices and the energy
// evaluation; this class owns only the step state.
class MVS_API MeshRefineStep
{
public:
	typedef TPoint3<float> Grad;
	typedef CLISTDEF0IDX(Grad,uint32_t) GradArr;

	// the single operating point: pixel/ZNCC quantities, scene-independent, hence not exposed
	static constexpr float StepInit = 0.5f; // eta at the start of every scale, px
	static constexpr float StepMax = 1.f; // eta never exceeds this, px
	static constexpr float StepGrow = 1.1f; // eta *= this after an accepted evaluation
	static constexpr float StepShrink = 0.5f; // eta *= this after a rejected one
	static constexpr float StepStop = 0.05f; // median per-vertex step at a full stride below which the scale has converged, px
	static constexpr float ProgressTol = 1e-3f; // relative decrease of S at or below which an evaluation counts as stalled
	static constexpr float Kappa = 2.f; // the median seen vertex moves eta/Kappa px at the first evaluation
	static constexpr unsigned Patience = 3; // consecutive stalled evaluations that end the scale
	static constexpr unsigned MaxRejects = 4; // consecutive rejections that end the scale
	static constexpr unsigned MinIters = 3; // no stop rule fires before this many accepted evaluations
	static constexpr unsigned MaxIters = 45; // evaluation budget of the coarsest scale, see Budget()

	// evaluation budget of a scale (0-based, coarsest first), thinned on the finer, more expensive
	// ones; a safety net behind the stop rules, not a stopping rule: raising it 20x changes no
	// measured result
	static unsigned Budget(unsigned nScale) { return MAXF(MaxIters/(nScale+1), 8u); }

	enum Action {
		APPLY, // the vertices were moved; evaluate the energy again
		REJECT, // half the previous step was undone (vertices sit at v_prev + stepPrev/2) and eta was halved; evaluate the energy again
		STOP // converged, or out of rejects: leave this scale
	};

	// one energy evaluation, as the backend already has it; the pointers are borrowed for the
	// duration of the Evaluate() call only
	struct Terms {
		const Grad* photoGrad; // raw per-vertex photometric gradient sum, not yet divided by photoCount
		const float* photoCount; // c_v: how many pair-directions saw v (0 = unseen)
		const float* footprint; // s_v: scene units per pixel; 0 exactly where c_v == 0
		const Grad* lap; // first-order smoothness term, 0 on boundary vertices
		const Grad* bilap; // second-order smoothness term, 0 on boundary vertices
		float S; // reliability-weighted mean of (1 - ZNCC) over all pairs, in [0,2]
		float rigidity; // rho
		float regularityWeight; // w
		uint32_t numVertices;
		// this evaluation saw only one direction of each image pair, alternating with the
		// evaluation index (nAlternatePair == 1): its S is comparable only with the S of an
		// evaluation of the same parity, so two references are carried instead of one
		bool alternating;
	};

	// what one Evaluate() decided, for the caller's log line
	struct Stats {
		float S; // the S this evaluation reported
		float relChange; // (S - reference) / reference, the reference being the last accepted S (0 while there is none)
		float step; // eta after the decision, px
		float medianPx; // median per-vertex step applied, px (0 unless APPLY)
		uint32_t numMoved; // vertices that received a non-zero step
		unsigned numAccepted; // accepted evaluations so far this scale
		unsigned numRejected; // rejected evaluations so far this scale
		bool accepted; // this evaluation was accepted
	};

public:
	MeshRefineStep() { Reset(0); }

	// begin a scale: eta starts at stepInit, the median normalizer is re-derived at the next
	// evaluation, and that evaluation is accepted unconditionally (nothing to compare it against)
	void Reset(uint32_t numVertices, float stepInit=StepInit);

	// judge one evaluation and, when it is accepted, move the vertices
	Action Evaluate(const Terms& terms, Mesh::VertexArr& vertices, Stats& stats);

	// the mesh changed under us (planar-vertex removal): the S references are dropped, so the
	// next evaluation of either parity is accepted unconditionally, and the undo buffer is rebuilt
	void TopologyChanged(uint32_t numVertices);

	// begin the scale's second, pure-elasticity phase and return its evaluation budget: 3/7 of
	// the evaluations accepted so far, the one budget that is not convergence-driven (running this
	// phase to convergence over-smooths, see the design document). eta and the S references carry
	// over; the stall count restarts, since one the first phase's tail primed would end this
	// phase after a single evaluation, while the reject streak deliberately carries over too (a
	// scale that gave up on its rejections stays given up: resetting it measured worse)
	unsigned BeginSecondPhase() { numStalled = 0; return MAXF(3u, numAccepted*3/7); }

	unsigned GetNumAccepted() const { return numAccepted; }
	// index of the evaluation about to be judged; the caller feeds it to the energy as its
	// iteration number so the pair direction an alternating run scores matches the parity this
	// class compares S against
	unsigned GetNumEvaluated() const { return numEvaluated; }

protected:
	// |g_v|/s_v of the median seen vertex, computed once per scale
	float ComputeMedianScale(const Terms& terms);

protected:
	float step; // eta, px
	float median; // m, the scale's held normalizer; negative until the first evaluation derives it
	float scoreRef; // S of the last accepted evaluation; FLT_MAX until there is one
	float scoreRefAlt; // the same for the odd evaluations of an alternating-pair run
	unsigned numAccepted;
	unsigned numRejected; // CONSECUTIVE rejections, reset by every accepted evaluation
	unsigned numRejectedTotal;
	unsigned numStalled;
	unsigned numEvaluated; // evaluations this scale, accepted or not
	GradArr stepPrev; // the last applied per-vertex step, for the halving undo
	FloatArr scratch; // median workspace, kept across evaluations so none allocates
};
/*----------------------------------------------------------------*/


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
