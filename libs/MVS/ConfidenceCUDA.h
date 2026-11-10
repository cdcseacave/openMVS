/*
 * ConfidenceCUDA.h
 *
 * Host-callable entry points for the GPU port of the fusion-faithful confidence recalibration
 * (the CUDA counterpart of SceneDensify.cpp's AdjustConfidenceSweep + ComputeIntraMapPrior).
 * Compiled only when CUDA is enabled; the CPU sweep is the fallback when it isn't.
 *
 * The per-pixel math is shared verbatim with the CPU via ConfidenceRefine.h; this header only
 * declares the launchers:
 *  - RunConfidenceCUDA: standalone/fallback variant -- uploads the reference + neighbor maps from
 *    host memory, runs the prior and confidence-sweep kernels, downloads the adjusted confidence.
 *  - RunConfidenceFusedCUDA: fused variant (resident-buffer reuse) -- the reference depth,
 *    normal and raw NCC cost are read straight from the PatchMatch instance's device-resident
 *    buffers (cudaDepthNormalEstimates / cudaDepthNormalCosts); only the neighbors' raw
 *    previous-iteration conf/normal/depth snapshots are uploaded.
 */
#ifndef _MVS_CONFIDENCECUDA_H_
#define _MVS_CONFIDENCECUDA_H_

// NOTE: these declarations are intentionally NOT wrapped in #ifdef _USE_CUDA -- they use only plain
// POD/STL types (no cudaStream_t etc.), so they are harmless to declare in a non-CUDA build. The
// definitions (ConfidenceCUDA.cu) are compiled only in CUDA builds, and every call site is guarded
// with #ifdef _USE_CUDA, so a non-CUDA build never references the missing symbols.

#include "ConfidenceGPU.h"

#include <cstdint>
#include <vector>

namespace MVS {
namespace CUDA {

// the host descriptors shared with the Metal backend (ConfidenceGPU.h)
using MVS::ConfNeighborHost;
using MVS::ConfAdjustRequest;

// Compute the intra-map prior + one-hop multi-view confirmation on the GPU for one reference view,
// writing the recalibrated confidence into confOut (host buffer, W*H). All input pointers are HOST
// memory; the launcher does the H2D/D2H itself. Returns false on any CUDA error, so the caller can
// fall back to the CPU sweep. refNormal may be null (no reference normal -> normal gates neutral).
bool RunConfidenceCUDA(
	int W, int H,
	const float* refDepth, const float* refNormal /*3*W*H or null*/, const float* refConf,
	float fx, float fy, float cx, float cy,
	const ConfNeighborHost* neighbors, int nNeighbors,
	const ConfRefine::Params& params,
	float* confOut);

// Fused variant (resident-buffer reuse): the reference maps are NOT uploaded -- devDepthNormals
// is the PatchMatch instance's resident Point4-per-pixel buffer (xyz = normal, w = depth) and
// devCosts its resident ZNCC cost buffer (raw conf = cost>=1 ? 0 : 1-cost, the same conversion the
// host unpack loop applies). Launches on the caller's stream (pass the instance's cudaStream_t as
// void* to keep this header CUDA-free) so it is ordered after the estimation kernels, and
// synchronizes that stream before returning. Only the prior/output buffers and the neighbor maps
// are allocated/uploaded here. Returns false on any CUDA error (caller falls back to the epilogue
// re-upload path or the CPU sweep).
bool RunConfidenceFusedCUDA(
	int W, int H,
	const void* devDepthNormals, const float* devCosts,
	float fx, float fy, float cx, float cy,
	const ConfNeighborHost* neighbors, int nNeighbors,
	const ConfRefine::Params& params,
	void* stream,
	float* confOut);

} // namespace CUDA
} // namespace MVS

#endif // _MVS_CONFIDENCECUDA_H_
