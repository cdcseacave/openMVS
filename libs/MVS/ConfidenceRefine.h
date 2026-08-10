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

#if defined(__CUDACC__)
	#define CR_HD __host__ __device__ __forceinline__
#else
	#define CR_HD inline
#endif

namespace MVS {
namespace ConfRefine {

// plain float triple (a device-safe stand-in for Normal/Point3f)
struct F3 { float x, y, z; };

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

// single-precision snapshot of the shape constants above + the gate thresholds that DO remain runtime
// (they are shared with fusion), uploaded to the kernel and passed to Posterior/soft-weight helpers.
struct Params {
	// posterior / gate shape (the constants above)
	float s, tau, kPrior, w0, confFloor;
	// gate thresholds shared with fusion
	float minConfidence;   // 1 - fNCCThresholdKeep  (G4)
	float thReproj;        // fDepthReprojectionErrorThreshold (G2)
	float thDepth;         // fDepthDiffThreshold (G1)
	float normalError;     // COS(D2R(fNormalDiffThreshold)) (G3)
	// free-space violation
	float lambdaViol, violMargin;
	// soft-gate G4 transition half-width: MAXF(0.5f*minConfidence, 1e-6f)
	float epsConf;
};

// fill the shape constants; the caller sets the runtime gate thresholds
CR_HD void InitParamsShape(Params& p) {
	p.s = PRIOR_STRENGTH;
	p.tau = CONFIRM_TAU;
	p.kPrior = PRIOR_GATE;
	p.w0 = PHOTO_FLOOR;
	p.confFloor = CONF_FLOOR;
	p.lambdaViol = VIOLATION_W;
	p.violMargin = VIOLATION_MARGIN;
}

// mirror of SEACAVE::DepthSimilarity/IsDepthSimilar: ABS(d0-d1)/d0 < thr (d0 > 0 assumed). The
// division (not thr*d0) is deliberate -- it reproduces DepthSimilarity byte-for-byte on the host.
CR_HD bool IsDepthSimilarF(float d0, float d1, float thr) {
	return fabsf(d0 - d1) / d0 < thr;
}

// exp helper: double std::exp on the host (byte-identical to the CPU EXP<float> = float(std::exp(double))),
// single-precision expf on the device.
CR_HD float CRexp(float a) {
#if defined(__CUDA_ARCH__)
	return expf(a);
#else
	return (float)std::exp((double)a);
#endif
}

CR_HD float CRclamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

// ---- final per-pixel posterior -> confidence (mirrors SceneDensify.cpp AdjustConfidenceSweep) ----
// Kf: the accumulated soft confirmation weight. Pconf: weighted sum of confirming neighbor confidences.
// V: free-space-violation count. pGeo: intra-map prior. confPhoto: own NCC conf.
CR_HD float Posterior(float confPhoto, float pGeo, float Kf, float Pconf, float V, const Params& p) {
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
CR_HD float SoftDepthW(float qz, float dN, float thDepth) {
	const float t = (qz - dN) / (0.5f * thDepth * qz);
	return CRexp(-t * t);
}
// GATE 2: forward-backward reprojection residual
CR_HD float SoftReprojW(float du, float dv, float thReproj) {
	const float d = 0.5f * thReproj;
	return CRexp(-(du * du + dv * dv) / (d * d));
}
// GATE 4: smoothstep on the neighbor confidence around minConfidence
CR_HD float SoftConfW(float cN, float minConfidence, float epsConf) {
	float t = (cN - (minConfidence - epsConf)) * (0.5f / epsConf);
	t = CRclamp01(t);
	return t * t * (3.f - 2.f * t);
}

// ---- intra-map geometric prior: local first-order depth-plane least-squares fit ----
// Templated on a depth accessor providing `float operator()(int x,int y) const` and
// `bool inside(int x,int y) const`, so the SAME fit runs on a host TImage and a device float*.
// Byte-identical to DepthGradientEstimator::DepthGradient: fills w (center depth) + (wx,wy) gradient;
// returns false if the center is invalid, <3 depth-similar neighbors, or the 2x2 normal system is singular.
template <typename DepthAcc>
CR_HD bool DepthPlaneFit(const DepthAcc& dm, int cx, int cy, float& w, float& wx, float& wy) {
	w = dm(cx, cy);
	if (w <= 0.f)
		return false;
	int whxx = 0, whxy = 0, whyy = 0;
	float wgx = 0.f, wgy = 0.f;
	int n = 0;
	for (int y = -1; y <= 1; ++y) {
		for (int x = -1; x <= 1; ++x) {
			if (x == 0 && y == 0)
				continue;
			const int px = cx + x, py = cy + y;
			if (!dm.inside(px, py))
				continue;
			const float wi = dm(px, py);
			if (!(wi > 0.f && IsDepthSimilarF(w, wi, 0.03f)))   // DepthGradientEstimator::IsDepthValid
				continue;
			whxx += x * x; whxy += x * y; whyy += y * y;
			wgx += (wi - w) * (float)x; wgy += (wi - w) * (float)y;
			++n;
		}
	}
	if (n < 3)
		return false;
	const int det = whxx * whyy - whxy * whxy;
	if (det == 0)
		return false;
	const float invDet = 1.f / (float)det;
	wx = ((float)whyy * wgx - (float)whxy * wgy) * invDet;
	wy = ((float)(-whxy) * wgx + (float)whxx * wgy) * invDet;
	return true;
}

// surface normal implied by a depth gradient (camera-facing, normalized) -- mirrors
// DepthGradientEstimator::NormalFromGradient (K assumed skew-free). Returned NOT necessarily used on
// the CPU (which keeps its own copy for byte-identity); provided for the device kernel.
CR_HD F3 NormalFromGrad(float k00, float k11, float k02, float k12, int x, int y, float d, float dx, float dy) {
	F3 nrm;
	nrm.x = k00 * dx;
	nrm.y = k11 * dy;
	nrm.z = (k02 - (float)x) * dx + (k12 - (float)y) * dy - d;
	const float inv = 1.f / sqrtf(nrm.x * nrm.x + nrm.y * nrm.y + nrm.z * nrm.z);
	nrm.x *= inv; nrm.y *= inv; nrm.z *= inv;
	return nrm;
}

} // namespace ConfRefine
} // namespace MVS

#endif // _MVS_CONFIDENCEREFINE_H_
