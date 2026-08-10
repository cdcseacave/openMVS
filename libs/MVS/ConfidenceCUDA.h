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
 *  - RunConfidenceFusedCUDA: fused variant (T14 resident-buffer reuse) -- the reference depth,
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

#include "ConfidenceRefine.h"

#include <vector>

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
	int srcImage;          // index of this neighbor in depthDataRef.images[] (>=1)
	// optional resident depth texture (a cudaTextureObject_t handle, kept as an integer so this
	// header stays CUDA-free): when nonzero the FUSED launcher reads the neighbor depth from it via
	// exact texel-center tex2D fetches instead of uploading `depth`. Set only by the fused call site
	// (PatchMatch::EstimateDepthMap) and only when the resident texture holds the UNRESIZED map
	// (view.depthMap.size() == image.size()), so texture and host buffer are byte-identical; the
	// standalone/epilogue path (RunConfidenceCUDA) always leaves it 0 and uploads.
	unsigned long long texDepth;
};

// Everything the fused in-estimation confidence launch needs, prepared by the dispatch layer
// (DepthMapsData::EstimateDepthMap) and consumed inside PatchMatch::EstimateDepthMap after the
// last geometric-consistency kernels. The neighbor host pointers reference this reference's own
// depthDataRef.images[] maps -- the raw PREVIOUS-iteration snapshots loaded by InitViews
// (loadDepthMaps==2), preserving the raw-neighbor-confidence invariant (single Jacobi pass,
// order-independent; see gt_bench/T14_HANDOFF.md par.4).
struct ConfAdjustRequest {
	std::vector<ConfNeighborHost> neighbors;
	ConfRefine::Params params;             // single-precision OPTDENSE snapshot
	float k00, k11, k02, k12;              // reference camera intrinsics (skew-free)
	float normalDiffThresholdDeg;          // OPTDENSE::fNormalDiffThreshold (prior coherence)
	bool softGates, priorNormalCoherence;
	// outputs
	bool done = false;                     // fused kernels ran and confMap holds the adjusted conf
	long long computeNS = 0;               // wall time of the fused launch (kernels + transfers)
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

// Fused variant (T14 resident-buffer reuse): the reference maps are NOT uploaded -- devDepthNormals
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
	float k00, float k11, float k02, float k12,
	float normalDiffThresholdDeg,
	const ConfNeighborHost* neighbors, int nNeighbors,
	const ConfRefine::Params& params, bool softGates, bool priorNormalCoherence,
	void* stream,
	float* confOut);

} // namespace CUDA
} // namespace MVS

#endif // _MVS_CONFIDENCECUDA_H_
