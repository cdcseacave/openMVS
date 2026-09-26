/*
 * ConfidenceRefine.h
 *
 * Shared, dependency-free per-pixel math for the fusion-faithful confidence recalibration, callable
 * from BOTH the CPU sweep (SceneDensify.cpp: AdjustConfidenceSweep / ComputeIntraMapPrior) and the
 * CUDA kernel (ConfidenceCUDA.cu). Everything here is plain scalar float on POD types -- NO OpenCV /
 * TImage / cv::Matx -- so the same header compiles under the host C++ compiler and under nvcc.
 *
 * Parity contract: on the HOST path these inlines reproduce the exact operations (and, for the
 * transcendental, the exact double-precision std::exp) of the pre-refactor CPU code, so refactoring
 * the CPU onto them is byte-identical. On the DEVICE path the same algebra runs in single precision
 * (expf), which differs only at the ULP level -- well inside the |dROC| <= 0.005 GPU-vs-CPU gate.
 */
#ifndef _MVS_CONFIDENCEREFINE_H_
#define _MVS_CONFIDENCEREFINE_H_

#include <cmath>

namespace MVS {
namespace ConfRefine {

// ---- posterior shape constants ----
// Calibrated jointly against ground-truth depth (BlendedMVS + ETH3D, 28 scene-levels) by sweeping the
// whole grid and scoring the inlier/outlier ROC of the resulting confidence. One global setting won on
// every scene-level, with no meaningful per-resolution split, so these are deliberately compile-time
// constants and not user knobs: they are a single jointly-tuned operating point, and moving one without
// re-sweeping the others degrades the calibration. Retuning means re-running that sweep.
constexpr float PRIOR_STRENGTH  = 2.0f;  // intra-map geometric prior weight, as Beta pseudo-counts
constexpr float CONFIRM_TAU     = 1.5f;  // softness of the multi-view confirmation gate
constexpr float PRIOR_GATE      = 0.3f;  // prior's contribution to the gate when no neighbor confirms
constexpr float PHOTO_FLOOR     = 0.7f;  // minimum multiplicative photometric weight
constexpr float CONF_FLOOR      = 0.03f; // anti-cascade floor (times photometric conf) once K >= 1
constexpr float VIOLATION_W     = 2.0f;  // posterior-denominator weight of the free-space-violation count
constexpr float VIOLATION_MARGIN= 2.0f;  // how far behind our depth (in units of thDepth) a neighbor's
                                         // own depth must lie to count as a violation vs. mere occlusion
// ---- confirmation tolerances (derivation: docs/design/DepthMapConfidence.md) ----
// A confirmation is evidence that the depth is CORRECT, so its tolerances follow the estimation
// noise, not fusion's clustering thresholds: correct depths disagree across views by a RELATIVE depth
// that is constant over triangulation angles (p90 0.3-1% on T&T), so G1 is a relative-depth Gaussian,
// and a pixel reprojection test would only repeat it scaled by f*sin(angle). The angle measures
// instead how INDEPENDENT a vote is (a narrow-baseline neighbor shares the reference's mistakes):
// G2 weighs it by min(1, sin(angle)/CONFIRM_SIN_ANGLE).
constexpr float CONFIRM_DEPTH   = 0.005f;      // relative depth width of a confirmation (G1), also the
                                               // unit of the free-space margin and of the prior band
constexpr float CONFIRM_SIN_ANGLE = 0.34202014f; // sin(20 deg): triangulation angle of a full vote
// fusion's seed/join floor on the RECALIBRATED confidence (a raw photometric confidence keeps the
// estimation's 1 - fNCCThresholdKeep); fusion's contradiction guard removes the disputed clusters,
// so the floor only cuts the hopeless ones (0.07: best mean F1 on four T&T scenes)
constexpr float FUSE_MIN_CONF   = 0.07f;

// single-precision snapshot of the shape constants above + the gate thresholds, uploaded to the
// kernel and passed to Posterior/soft-weight helpers.
struct Params {
	// posterior / gate shape (the constants above)
	float s, tau, kPrior, w0, confFloor;
	// gate thresholds (G3, the normal gate, needs none here: it is a continuous dot-product weight)
	float minConfidence;   // 1 - fNCCThresholdKeep (G4): the estimation's floor on the photometric confidence
	float thDepth;         // CONFIRM_DEPTH (G1)
	// free-space violation
	float lambdaViol, violMargin;
	// soft-gate G4 transition half-width: MAXF(0.5f*minConfidence, 1e-6f)
	float epsConf;
};

// fill the constants; the caller sets minConfidence and epsConf
HOST_DEVICE inline void InitParamsShape(Params& p) {
	p.s = PRIOR_STRENGTH;
	p.tau = CONFIRM_TAU;
	p.kPrior = PRIOR_GATE;
	p.w0 = PHOTO_FLOOR;
	p.confFloor = CONF_FLOOR;
	p.lambdaViol = VIOLATION_W;
	p.violMargin = VIOLATION_MARGIN;
	p.thDepth = CONFIRM_DEPTH;
}

// exp helper: double std::exp on the host (byte-identical to the CPU EXP<float> = float(std::exp(double))),
// single-precision expf on the device.
HOST_DEVICE inline float CRexp(float a) {
#if defined(__CUDA_ARCH__)
	return expf(a);
#else
	return (float)std::exp((double)a);
#endif
}

HOST_DEVICE inline float CRclamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

// ---- final per-pixel posterior -> confidence (mirrors SceneDensify.cpp AdjustConfidenceSweep) ----
// Kf: the accumulated soft confirmation weight. Pconf: weighted sum of confirming neighbor confidences.
// V: free-space-violation count. pGeo: intra-map prior. confPhoto: own NCC conf.
HOST_DEVICE inline float Posterior(float confPhoto, float pGeo, float Kf, float Pconf, float V, const Params& p) {
	const float gate = 1.f - CRexp(-(Kf + p.kPrior * pGeo) / p.tau);
	const float posterior = (p.s * pGeo + Pconf) / (p.s + Pconf + p.lambdaViol * V);
	const float photoFactor = p.w0 + (1.f - p.w0) * confPhoto;
	float conf = CRclamp01(posterior * gate * photoFactor);
	if (Kf >= 1.f) {                                   // anti-cascade floor
		const float fl = p.confFloor * confPhoto;
		conf = conf > fl ? conf : fl;
	}
	return conf;
}

// ---- soft-gate continuous weights, all in [0,1] ----
// GATE 1: Gaussian relative-depth agreement
HOST_DEVICE inline float SoftDepthW(float qz, float dN, float thDepth) {
	const float t = (qz - dN) / (0.5f * thDepth * qz);
	return CRexp(-t * t);
}
// GATE 2: independence of the confirmation, from the triangulation angle at the point: X is the
// point in the reference camera frame (reference centre at the origin), (cx,cy,cz) the neighbor's
// centre in the same frame; sin(angle) = |X x (X-C)| / (|X| |X-C|) = |X x C| / (|X| |X-C|).
// X must lie in front of both cameras (the callers skip a non-positive depth in either), so it is
// neither camera centre and the denominator is positive
HOST_DEVICE inline float AngleW(float x, float y, float z, float cx, float cy, float cz) {
	const float ax = y*cz - z*cy, ay = z*cx - x*cz, az = x*cy - y*cx;
	const float dx = x - cx, dy = y - cy, dz = z - cz;
	const float den2 = (x*x + y*y + z*z) * (dx*dx + dy*dy + dz*dz);
	ASSERT(den2 > 0.f);
	const float w = sqrtf((ax*ax + ay*ay + az*az) / den2) * (1.f / CONFIRM_SIN_ANGLE);
	return w < 1.f ? w : 1.f;
}
// GATE 4: smoothstep on the neighbor confidence around minConfidence
HOST_DEVICE inline float SoftConfW(float cN, float minConfidence, float epsConf) {
	float t = (cN - (minConfidence - epsConf)) * (0.5f / epsConf);
	t = CRclamp01(t);
	return t * t * (3.f - 2.f * t);
}

} // namespace ConfRefine
} // namespace MVS

#endif // _MVS_CONFIDENCEREFINE_H_
