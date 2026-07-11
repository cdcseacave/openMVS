/*
 * ConfidenceCUDA.h
 *
 * Host-callable entry point for the GPU port of the fusion-faithful confidence recalibration
 * (the CUDA counterpart of SceneDensify.cpp's AdjustConfidenceSweep + ComputeIntraMapPrior).
 * Compiled only when CUDA is enabled; the CPU sweep is the fallback when it isn't.
 *
 * The per-pixel math is shared verbatim with the CPU via ConfidenceRefine.h; this header only
 * declares the launcher, which uploads the reference + neighbor maps, runs the prior and
 * confidence-sweep kernels, and downloads the adjusted confidence.
 */
#ifndef _MVS_CONFIDENCECUDA_H_
#define _MVS_CONFIDENCECUDA_H_

// NOTE: this declaration is intentionally NOT wrapped in #ifdef _USE_CUDA -- it uses only plain
// POD types (no cudaStream_t etc.), so it is harmless to declare in a non-CUDA build. The
// definition (ConfidenceCUDA.cu) is compiled only in CUDA builds, and every call site is guarded
// with #ifdef _USE_CUDA, so a non-CUDA build never references the missing symbol.

#include "ConfidenceRefine.h"

namespace MVS {
namespace CUDA {

// Host-side descriptor for one confirming neighbor view: the fused single-precision projection
// transforms (row-major 3x3 A/Ai/Rrel + 3-vectors b/bi, built exactly as the CPU NeighborProj) plus
// HOST pointers to that neighbor's depth/conf/normal maps (row-major, contiguous). conf/normal may
// be null (treated as "no confidence" / "no normal gate", matching the CPU).
struct ConfNeighborHost {
	float A[9], b[3], Ai[9], bi[3], Rrel[9];
	const float* depth;    // width*height
	const float* conf;     // width*height, or null
	const float* normal;   // 3*width*height (interleaved x,y,z), or null
	int width, height;
};

// Compute the intra-map prior + one-hop multi-view confirmation on the GPU for one reference view,
// writing the recalibrated confidence into confOut (host buffer, W*H). All input pointers are HOST
// memory; the launcher does the H2D/D2H itself. Returns false on any CUDA error, so the caller can
// fall back to the CPU sweep. refNormal may be null (no reference normal -> normal gates neutral).
bool RunConfidenceCUDA(
	int W, int H,
	const float* refDepth, const float* refNormal /*3*W*H or null*/, const float* refConf,
	float k00, float k11, float k02, float k12,           // reference camera intrinsics (skew-free)
	float normalDiffThresholdDeg,                         // OPTDENSE::fNormalDiffThreshold (prior coherence)
	const ConfNeighborHost* neighbors, int nNeighbors,
	const ConfRefine::Params& params, bool softGates, bool priorNormalCoherence,
	float* confOut);

} // namespace CUDA
} // namespace MVS

#endif // _MVS_CONFIDENCECUDA_H_
