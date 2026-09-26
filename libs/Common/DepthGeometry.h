////////////////////////////////////////////////////////////////////
// DepthGeometry.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_DEPTHGEOMETRY_H__
#define __SEACAVE_DEPTHGEOMETRY_H__


// I N C L U D E S /////////////////////////////////////////////////

#include <cfloat>


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Local geometry of a depth-map seen by a skew-free pinhole camera (focal lengths f and principal
// point pp, in pixels), shared by host code and CUDA device code: depth similarity, the depth
// plane fitted around a pixel, the normal that plane implies, and a neighbor's plane carried to a
// pixel. The including context provides Eigen and ASSERT (Common.h, or CUDA/Maths.h in .cu files).

// relative difference of depth d1 to depth d0
template<typename T>
HOST_DEVICE inline T DepthSimilarity(T d0, T d1) {
	ASSERT(d0 > 0);
	return (d0 > d1 ? d0 - d1 : d1 - d0) / d0;
}
template<typename T>
HOST_DEVICE inline bool IsDepthSimilar(T d0, T d1, T threshold=T(0.01)) {
	return DepthSimilarity(d0, d1) < threshold;
}
/*----------------------------------------------------------------*/


// least-squares depth plane d(x,y) = d0 + g.(x,y) around a pixel of depth d0,
// fitted from the depth differences d(x,y) - d0 of neighbors at pixel offsets (x,y)
struct DepthPlaneFit {
	int hxx = 0, hxy = 0, hyy = 0; // normal matrix, integer as the offsets are
	float gx = 0.f, gy = 0.f; // right-hand side
	int n = 0; // number of neighbors

	HOST_DEVICE void Add(int x, int y, float depthDiff) {
		hxx += x*x; hxy += x*y; hyy += y*y;
		gx += depthDiff*(float)x; gy += depthDiff*(float)y;
		++n;
	}
	// at least 3 neighbors, not all on one line through the pixel
	HOST_DEVICE bool IsValid() const {
		return n >= 3 && hxx*hyy != hxy*hxy;
	}
	// the depth gradient g
	HOST_DEVICE Eigen::Vector2f Gradient() const {
		ASSERT(IsValid());
		const float invDet = 1.f/(float)(hxx*hyy - hxy*hxy);
		return Eigen::Vector2f(
			((float)hyy*gx - (float)hxy*gy)*invDet,
			((float)-hxy*gx + (float)hxx*gy)*invDet);
	}
};

// fit the depth plane at pixel x over its 3x3 neighbors within 3% of its depth: fills the depth at x
// and the plane's gradient; false if x has no depth or the fit is not valid. DEPTHMAP is indexed
// (row,col) and has rows/cols members: a DepthMap on the host, the kernels' own views on the device
template<typename DEPTHMAP>
HOST_DEVICE inline bool FitDepthGradient(const DEPTHMAP& depthMap, const Eigen::Vector2i& x, float& depth, Eigen::Vector2f& gradient) {
	ASSERT(x.x() >= 0 && x.x() < depthMap.cols && x.y() >= 0 && x.y() < depthMap.rows);
	depth = depthMap(x.y(), x.x());
	if (depth <= 0.f)
		return false;
	DepthPlaneFit fit;
	for (int j = -1; j <= 1; ++j) {
		const int r = x.y() + j;
		if (r < 0 || r >= depthMap.rows)
			continue;
		for (int i = -1; i <= 1; ++i) {
			const int c = x.x() + i;
			if ((i == 0 && j == 0) || c < 0 || c >= depthMap.cols)
				continue;
			const float d = depthMap(r, c);
			if (d > 0.f && IsDepthSimilar(depth, d, 0.03f))
				fit.Add(i, j, d - depth);
		}
	}
	if (!fit.IsValid())
		return false;
	gradient = fit.Gradient();
	return true;
}

// camera-facing unit normal of the depth plane through pixel x at the given depth and gradient
HOST_DEVICE inline Eigen::Vector3f NormalFromDepthGradient(const Eigen::Vector2f& f, const Eigen::Vector2f& pp, const Eigen::Vector2i& x, float depth, const Eigen::Vector2f& gradient) {
	ASSERT(depth > 0.f);
	return Eigen::Vector3f(
		f.x()*gradient.x(),
		f.y()*gradient.y(),
		(pp.x()-(float)x.x())*gradient.x() + (pp.y()-(float)x.y())*gradient.y() - depth).normalized();
}

// depth at pixel x of the plane through neighbor pixel nx at depth nDepth with the given normal
// (camera frame): a neighbor on the same column or row is intersected in that slice, any other
// one along the ray through x; nDepth if the plane is parallel to that ray or the new depth is
// outside [dMin,dMax]
HOST_DEVICE inline float InterpolatePlaneDepth(const Eigen::Vector2f& f, const Eigen::Vector2f& pp, const Eigen::Vector2i& x, const Eigen::Vector2i& nx, float nDepth, const Eigen::Vector3f& normal, float dMin, float dMax) {
	ASSERT(nDepth > 0.f && dMin < dMax);
	float num, denom;
	if (x.x() == nx.x()) {
		const float rayY = ((float)x.y()-pp.y())/f.y();
		const float nRayY = ((float)nx.y()-pp.y())/f.y();
		denom = normal.z() + rayY*normal.y();
		num = nDepth*(normal.z() + nRayY*normal.y());
	} else if (x.y() == nx.y()) {
		const float rayX = ((float)x.x()-pp.x())/f.x();
		const float nRayX = ((float)nx.x()-pp.x())/f.x();
		denom = normal.z() + rayX*normal.x();
		num = nDepth*(normal.z() + nRayX*normal.x());
	} else {
		num = normal.dot(Eigen::Vector3f(nDepth*((float)nx.x()-pp.x())/f.x(), nDepth*((float)nx.y()-pp.y())/f.y(), nDepth));
		denom = normal.dot(Eigen::Vector3f(((float)x.x()-pp.x())/f.x(), ((float)x.y()-pp.y())/f.y(), 1.f));
	}
	if (fabsf(denom) < FLT_EPSILON)
		return nDepth;
	const float depth = num/denom;
	return depth >= dMin && depth <= dMax ? depth : nDepth;
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_DEPTHGEOMETRY_H__
