/*
 * ConfidenceGPU.h
 *
 * Backend-neutral host descriptors of the GPU confidence recalibration (the CUDA launchers of
 * ConfidenceCUDA.h and the Metal PatchMatch of PatchMatchMetal.h): the neighbor views a reference
 * confirms its depths against, and the request the dispatch layer hands to the estimation for the
 * fused launch. Plain POD/STL types only, so any translation unit can include it.
 */
#ifndef _MVS_CONFIDENCEGPU_H_
#define _MVS_CONFIDENCEGPU_H_

#include "ConfidenceRefine.h"

#include <vector>

namespace MVS {

// one confirming neighbor view: the fused single-precision projection transforms (row-major 3x3
// A/Rrel, 3-vectors b/cn, built exactly as the CPU NeighborProj, whose comment defines them for each
// pair of camera models) plus HOST pointers to that neighbor's depth/conf/normal maps (row-major,
// contiguous). conf/normal may be null (treated as "no confidence" / "no normal gate", matching the CPU).
struct ConfNeighborHost {
	float A[9], b[3], Rrel[9];
	float cn[3];           // neighbor centre in the reference camera frame (triangulation angle)
	float K[4];            // neighbor fx, fy, cx, cy at the resolution of its maps: its spherical model
	                       // (a pinhole neighbor's are folded into A and b)
	int spherical;         // the neighbor is equirectangular: A*u+b is its camera-frame point, its columns wrap
	const float* depth;    // width*height
	const float* conf;     // width*height, or null
	const float* normal;   // 3*width*height (interleaved x,y,z), or null
	int width, height;
	int srcImage;          // index of this neighbor in depthDataRef.images[] (>=1)
	// CUDA only: an optional resident depth texture (a cudaTextureObject_t handle, kept as an
	// integer so this header stays CUDA-free): when nonzero the FUSED launcher reads the neighbor
	// depth from it via exact texel-center tex2D fetches instead of uploading `depth`. Set only by
	// the fused call site (CUDA::PatchMatch::EstimateDepthMap) and only when the resident texture
	// holds the UNRESIZED map (view.depthMap.size() == image.size()), so texture and host buffer are
	// byte-identical; every other path leaves it 0 and uploads.
	unsigned long long texDepth;
};

// everything the fused in-estimation confidence launch needs, prepared by the dispatch layer
// (DepthMapsData::EstimateDepthMap) and consumed inside the GPU PatchMatch's EstimateDepthMap after
// the last geometric-consistency kernels. The neighbor host pointers reference this reference's
// own depthDataRef.images[] maps -- the raw PREVIOUS-iteration snapshots loaded by InitViews
// (loadDepthMaps==2), preserving the raw-neighbor-confidence invariant (single Jacobi pass,
// order-independent).
struct ConfAdjustRequest {
	std::vector<ConfNeighborHost> neighbors;
	ConfRefine::Params params;             // single-precision OPTDENSE snapshot
	float fx, fy, cx, cy;                  // reference camera intrinsics (skew-free): its pinhole or spherical model
	bool spherical;                        // the reference is equirectangular
	// outputs
	bool done = false;                     // fused kernels ran and confMap holds the adjusted conf
};

} // namespace MVS

#endif // _MVS_CONFIDENCEGPU_H_
