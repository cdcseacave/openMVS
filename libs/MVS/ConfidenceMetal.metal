/*
* ConfidenceMetal.metal
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

// Metal port of the confidence recalibration (docs/design/DepthMapConfidence.md), one thread per
// reference pixel, a hand mirror of ConfidenceCUDA.cu (MSL can not include the shared headers):
//   ConfidencePrior -- the intra-map geometric prior (DepthMapsData::ComputeIntraMapPrior)
//   ConfidenceSweep -- the one-hop multi-view confirmation and the posterior (AdjustConfidenceSweep)
// Compiled into the same library as PatchMatchMetal.metal, after it: its mirror of the camera models
// (FastAtan2, SphericalModel, SphericalProjectFast, SphericalRayFrame, NormalFromDepthGradient) serves
// here too. The reference is read as planes (normal xyz, depth w), the layout of the PatchMatch
// estimates, and its photometric confidence either as such or as the estimation's ZNCC cost (the fused
// launch reads the estimates where they are). The reference and each neighbor are pinhole or
// equirectangular (the columns of a spherical map wrap), the fused transforms of a pair defined as for
// the CPU NeighborProj (SceneDensify.cpp).

#include <metal_stdlib>
using namespace metal;

// ---- host-shared layouts (PatchMatchMetal.mm: MtlConfParams, MtlConfNeighbor) ----

#define CONF_NO_MAP 0xFFFFFFFFu // a neighbor map that is absent

struct ConfParams {
	float fx, fy, cx, cy;      // reference camera intrinsics (skew-free): its pinhole or spherical model
	int width, height;         // reference size
	int numNeighbors;
	int hasNormal;             // the reference planes carry valid normals
	int confFromCost;          // refConf holds the ZNCC cost (conf = cost>=1 ? 0 : 1-cost)
	int spherical;             // the reference is equirectangular
	// ConfRefine::Params, in its order
	float s, tau, kPrior, w0, confFloor;
	float minConfidence, thDepth;
	float lambdaViol, violMargin;
	float epsConf;
};

// a confirming neighbor view: ConfNeighborHost's transforms and the offsets of its maps in the
// packed neighbor buffers (depth and confidence one float per pixel, normal three)
struct ConfNeighbor {
	float A[9], b[3], Rrel[9], cn[3];
	float K[4];                // fx, fy, cx, cy of a spherical neighbor (folded into A, b for a pinhole one)
	int width, height;
	int spherical;             // A*u+b is the neighbor's camera-frame point, its columns wrap
	uint depthOffset, confOffset, normalOffset;
};

// ---- ConfidenceRefine.h ----

constant float CONFIRM_SIN_ANGLE = 0.34202014f; // sin(20 deg)

static inline float Clamp01(float v) { return clamp(v, 0.f, 1.f); }

static inline float Posterior(float confPhoto, float pGeo, float Kf, float Pconf, float V, constant ConfParams& p) {
	const float gate = 1.f - precise::exp(-(Kf + p.kPrior * pGeo) / p.tau);
	const float posterior = (p.s * pGeo + Pconf) / (p.s + Pconf + p.lambdaViol * V);
	const float photoFactor = p.w0 + (1.f - p.w0) * confPhoto;
	float conf = Clamp01(posterior * gate * photoFactor);
	if (Kf >= 1.f) // anti-cascade floor
		conf = max(conf, p.confFloor * confPhoto);
	return conf;
}
// GATE 1: Gaussian relative-depth agreement
static inline float SoftDepthW(float qz, float dN, float thDepth) {
	const float t = (qz - dN) / (0.5f * thDepth * qz);
	return precise::exp(-t * t);
}
// GATE 2: independence of the confirmation, from the triangulation angle at X (reference frame)
static inline float AngleW(float3 X, float3 C) {
	const float3 a = cross(X, C);
	const float3 d = X - C;
	const float den2 = dot(X, X) * dot(d, d);
	return min(precise::sqrt(dot(a, a) / den2) * (1.f / CONFIRM_SIN_ANGLE), 1.f);
}
// GATE 4: smoothstep on the neighbor confidence around minConfidence
static inline float SoftConfW(float cN, float minConfidence, float epsConf) {
	const float t = Clamp01((cN - (minConfidence - epsConf)) * (0.5f / epsConf));
	return t * t * (3.f - 2.f * t);
}

// ---- DepthGeometry.h ----

static inline bool IsDepthSimilar(float d0, float d1, float threshold) {
	return abs(d0 - d1) / d0 < threshold;
}
// the column of an equirectangular map of the given width holding column c, for c within one width of it
static inline int WrapColumn(int c, int width) {
	return c < 0 ? c + width : (c >= width ? c - width : c);
}

// the reference: planes (normal xyz, depth w) and the photometric confidence or cost
struct RefMaps {
	device const float4* planes;
	device const float* conf;
	int width, height;
	bool confFromCost;
	float Depth(int r, int c) const { return planes[r*width + c].w; }
	float Conf(int idx) const {
		const float v = conf[idx];
		return confFromCost ? (v >= 1.f ? 0.f : 1.f - v) : v;
	}
};

// FitDepthGradient: the depth plane at (c,r) over its 3x3 neighbors within 3% of its depth, the
// columns wrapping for an equirectangular map
static bool FitDepthGradient(thread const RefMaps& ref, int c, int r, bool wrapCols, thread float& depth, thread float2& gradient) {
	depth = ref.Depth(r, c);
	if (depth <= 0.f)
		return false;
	int hxx = 0, hxy = 0, hyy = 0, n = 0;
	float gx = 0.f, gy = 0.f;
	for (int j = -1; j <= 1; ++j) {
		const int rr = r + j;
		if (rr < 0 || rr >= ref.height)
			continue;
		for (int i = -1; i <= 1; ++i) {
			const int cc = wrapCols ? WrapColumn(c + i, ref.width) : c + i;
			if ((i == 0 && j == 0) || cc < 0 || cc >= ref.width)
				continue;
			const float d = ref.Depth(rr, cc);
			if (d > 0.f && IsDepthSimilar(depth, d, 0.03f)) {
				hxx += i*i; hxy += i*j; hyy += j*j;
				gx += (d - depth)*float(i); gy += (d - depth)*float(j);
				++n;
			}
		}
	}
	if (n < 3 || hxx*hyy == hxy*hxy)
		return false;
	const float invDet = 1.f/float(hxx*hyy - hxy*hxy);
	gradient = float2((float(hyy)*gx - float(hxy)*gy)*invDet, (float(-hxy)*gx + float(hxx)*gy)*invDet);
	return true;
}

// the sines and cosines of the longitude and of the elevation of pixel (c,r) of the spherical reference,
// from its table (ComputeSphericalTrig: the W columns then the H rows)
static inline float4 RefTrig(device const float2* trig, constant ConfParams& p, int c, int r) {
	return float4(trig[c], trig[p.width + r]);
}
// NormalFromDepthGradient of the reference: the pinhole closed form, or through the spherical ray frame
static inline float3 RefNormalFromDepthGradient(device const float2* trig, constant ConfParams& p, int c, int r, float depth, float2 gradient) {
	if (p.spherical) {
		const float4 t = RefTrig(trig, p, c, r);
		return NormalFromDepthGradient(SphericalRayFrame(SphericalModel{p.fx, p.fy, p.cx, p.cy}, t.x, t.y, t.z, t.w), depth, gradient);
	}
	return normalize(float3(
		p.fx*gradient.x,
		p.fy*gradient.y,
		(p.cx-float(c))*gradient.x + (p.cy-float(r))*gradient.y - depth));
}

// ---- kernels ----

// the intra-map geometric prior (ConfidenceCUDA.cu PriorKernel)
kernel void ConfidencePrior(
	device const float4* planes [[buffer(0)]],
	device const float* refConf [[buffer(1)]],
	constant ConfParams& p [[buffer(2)]],
	device float* priorOut [[buffer(3)]],
	device const float2* trig [[buffer(9)]],
	uint2 gid [[thread_position_in_grid]])
{
	const int c = int(gid.x), r = int(gid.y);
	if (c >= p.width || r >= p.height)
		return;
	const int idx = r*p.width + c;
	const RefMaps ref{planes, refConf, p.width, p.height, p.confFromCost != 0};
	priorOut[idx] = 0.f;
	float w; float2 grad;
	if (!FitDepthGradient(ref, c, r, p.spherical != 0, w, grad))
		return;
	const float band = p.thDepth * 3.f;
	const float invKmin = 1.f/4.f;
	int nInl = 0; float sumE2 = 0.f;
	for (int y = -1; y <= 1; ++y) {
		const int rr = r + y;
		if (rr < 0 || rr >= p.height) continue;
		for (int x = -1; x <= 1; ++x) {
			if (x == 0 && y == 0) continue;
			const int cc = p.spherical ? WrapColumn(c + x, p.width) : c + x;
			if (cc < 0 || cc >= p.width) continue;
			const float dN = ref.Depth(rr, cc);
			if (dN <= 0.f) continue;
			const float dpred = w + grad.x*float(x) + grad.y*float(y);
			const float e = abs(dN - dpred) / w;
			if (e < band) { ++nInl; const float en = e/band; sumE2 += en*en; }
		}
	}
	if (nInl < 3)
		return;
	const float Pplane = precise::exp(-sumE2 / float(nInl));
	const float gate = 1.f - precise::exp(-float(nInl) * invKmin);
	float Pnorm = 1.f;
	if (p.hasNormal)
		Pnorm = max(0.f, dot(RefNormalFromDepthGradient(trig, p, c, r, w, grad), planes[idx].xyz));
	priorOut[idx] = Clamp01(Pplane * Pnorm * gate);
}

// the neighbor depth at integer pixel (x,y), inside the map
static inline float NeighborDepth(device const float* depths, constant ConfNeighbor& nb, int x, int y) {
	return depths[nb.depthOffset + uint(y*nb.width + x)];
}
// SceneDensify.cpp SampleDepthBilinear (edge-aware; false -> the caller takes the nearest depth; the
// columns of a spherical neighbor wrap)
static bool SampleDepthBilinear(device const float* depths, constant ConfNeighbor& nb, float px, float py, float thDepth, thread float& d) {
	const int x0 = int(floor(px)), y0 = int(floor(py));
	if (y0 < 0 || y0 + 1 >= nb.height)
		return false;
	int c0 = x0, c1 = x0 + 1;
	if (nb.spherical) {
		c0 = WrapColumn(c0, nb.width);
		c1 = WrapColumn(c1, nb.width);
	} else if (c0 < 0 || c1 >= nb.width)
		return false;
	const float d00 = NeighborDepth(depths, nb, c0, y0), d01 = NeighborDepth(depths, nb, c1, y0);
	const float d10 = NeighborDepth(depths, nb, c0, y0+1), d11 = NeighborDepth(depths, nb, c1, y0+1);
	if (d00 <= 0.f || d01 <= 0.f || d10 <= 0.f || d11 <= 0.f)
		return false;
	const float dmin = min(min(d00, d01), min(d10, d11));
	const float dmax = max(max(d00, d01), max(d10, d11));
	if (!IsDepthSimilar(dmin, dmax, thDepth))
		return false;
	const float wx = px - float(x0), wy = py - float(y0);
	d = (d00*(1.f-wx) + d01*wx)*(1.f-wy) + (d10*(1.f-wx) + d11*wx)*wy;
	return true;
}

// the one-hop multi-view confirmation and the posterior (ConfidenceCUDA.cu SweepKernel)
kernel void ConfidenceSweep(
	device const float4* planes [[buffer(0)]],
	device const float* refConf [[buffer(1)]],
	constant ConfParams& p [[buffer(2)]],
	device const float* priorMap [[buffer(3)]],
	constant ConfNeighbor* neighbors [[buffer(4)]],
	device const float* nbDepths [[buffer(5)]],
	device const float* nbConfs [[buffer(6)]],
	device const float* nbNormals [[buffer(7)]],
	device float* confOut [[buffer(8)]],
	device const float2* trig [[buffer(9)]],
	uint2 gid [[thread_position_in_grid]])
{
	const int c = int(gid.x), r = int(gid.y);
	if (c >= p.width || r >= p.height)
		return;
	const int idx = r*p.width + c;
	const RefMaps ref{planes, refConf, p.width, p.height, p.confFromCost != 0};
	const float depthRef = ref.Depth(r, c);
	if (depthRef <= 0.f) { confOut[idx] = 0.f; return; }
	const float3 rn = p.hasNormal ? planes[idx].xyz : float3(0.f);

	float K = 0.f, Pconf = 0.f;
	int V = 0;
	// the point in the reference camera frame (triangulation angle, GATE 2), and u the reference pixel
	// in the form the fused transforms take: (c*d,r*d,d) for a pinhole reference, the point itself for
	// a spherical one
	float3 Xr, u;
	if (p.spherical) {
		const float4 t = RefTrig(trig, p, c, r);
		Xr = SphericalRay(t.x, t.y, t.z, t.w) * depthRef;
		u = Xr;
	} else {
		Xr = float3(((float)c - p.cx) / p.fx * depthRef, ((float)r - p.cy) / p.fy * depthRef, depthRef);
		u = float3(float(c) * depthRef, float(r) * depthRef, depthRef);
	}
	for (int k = 0; k < p.numNeighbors; ++k) {
		constant ConfNeighbor& nb = neighbors[k];
		float qz = nb.A[6]*u.x + nb.A[7]*u.y + nb.A[8]*u.z + nb.b[2];
		if (!nb.spherical && qz <= 0.f) continue;
		const float qx = nb.A[0]*u.x + nb.A[1]*u.y + nb.A[2]*u.z + nb.b[0];
		const float qy = nb.A[3]*u.x + nb.A[4]*u.y + nb.A[5]*u.z + nb.b[1];
		float px, py;
		if (nb.spherical) {
			// q is the neighbor's camera-frame point, its depth the range
			const float3 q(qx, qy, qz);
			qz = length(q);
			if (qz <= 0.f) continue;
			float2 x;
			SphericalProjectFast(SphericalModel{nb.K[0], nb.K[1], nb.K[2], nb.K[3]}, q, x);
			px = x.x; py = x.y;
		} else {
			px = qx/qz; py = qy/qz;
		}
		int xN = int(floor(px + 0.5f)); // SEACAVE::Round2Int
		const int yN = int(floor(py + 0.5f));
		if (nb.spherical)
			xN = WrapColumn(xN, nb.width);
		if (xN < 0 || xN >= nb.width || yN < 0 || yN >= nb.height) continue;
		const float dNn = NeighborDepth(nbDepths, nb, xN, yN);
		if (dNn <= 0.f) continue;
		// free-space violation: the neighbor sees past the point along its ray
		if (!IsDepthSimilar(dNn, qz, p.thDepth) && dNn > qz*(1.f + p.violMargin*p.thDepth)) ++V;
		float dN;
		if (!SampleDepthBilinear(nbDepths, nb, px, py, p.thDepth, dN))
			dN = dNn;
		const float wD = SoftDepthW(qz, dN, p.thDepth);
		if (wD <= 0.05f)
			continue; // every other weight is <= 1: the vote is already negligible
		const float wR = AngleW(Xr, float3(nb.cn[0], nb.cn[1], nb.cn[2]));
		float wN = 1.f;
		if (p.hasNormal && nb.normalOffset != CONF_NO_MAP) {
			// the reference normal rotated into the neighbor camera, against the neighbor's own normal
			const float3 rnN(
				nb.Rrel[0]*rn.x + nb.Rrel[1]*rn.y + nb.Rrel[2]*rn.z,
				nb.Rrel[3]*rn.x + nb.Rrel[4]*rn.y + nb.Rrel[5]*rn.z,
				nb.Rrel[6]*rn.x + nb.Rrel[7]*rn.y + nb.Rrel[8]*rn.z);
			const uint n3 = nb.normalOffset + 3u*uint(yN*nb.width + xN);
			wN = max(0.f, dot(rnN, float3(nbNormals[n3], nbNormals[n3+1], nbNormals[n3+2])));
		}
		const float cN = nb.confOffset != CONF_NO_MAP ? nbConfs[nb.confOffset + uint(yN*nb.width + xN)] : 1.f;
		const float w = wD*wR*wN*SoftConfW(cN, p.minConfidence, p.epsConf);
		if (w <= 0.05f) continue;
		K += w; Pconf += w*cN;
	}
	confOut[idx] = Posterior(ref.Conf(idx), priorMap[idx], K, Pconf, float(V), p);
}
