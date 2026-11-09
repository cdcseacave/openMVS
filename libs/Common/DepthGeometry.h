////////////////////////////////////////////////////////////////////
// DepthGeometry.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_DEPTHGEOMETRY_H__
#define __SEACAVE_DEPTHGEOMETRY_H__


// I N C L U D E S /////////////////////////////////////////////////



// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Local geometry of a depth-map, shared by host code and CUDA device code: depth similarity, the depth
// plane fitted around a pixel, the normal that plane implies, a neighbor's plane carried to a pixel,
// and the plane-induced patch warp between two views. The camera-dependent functions take a camera
// model of Math/Camera/CameraModels.h: the generic version works through the model's Ray/RayFrame/
// PixelHomography, and a Pinhole overload keeps the closed-form pinhole operations. The including
// context provides Eigen, ASSERT and the camera models (Common.h, or CUDA/Maths.h in .cu files).

template<typename T> struct Pinhole; // Math/Camera/CameraModels.h

// relative difference of depth d1 to depth d0
template<typename T>
HOST_DEVICE inline T DepthSimilarity(T d0, T d1) {
	ASSERT(d0 > 0);
	return Eigen::numext::abs(d0 - d1) / d0;
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
// (row,col) and has rows/cols members: a DepthMap on the host, the kernels' own views on the device;
// wrapCols makes the first and the last column neighbors (equirectangular maps)
template<typename DEPTHMAP>
HOST_DEVICE inline bool FitDepthGradient(const DEPTHMAP& depthMap, const Eigen::Vector2i& x, float& depth, Eigen::Vector2f& gradient, bool wrapCols=false) {
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
			int c = x.x() + i;
			if (wrapCols)
				c = c < 0 ? c + depthMap.cols : (c >= depthMap.cols ? c - depthMap.cols : c);
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
// (camera frame): the cross product of the plane's tangents along the pixel axes
template<typename MODEL>
HOST_DEVICE inline typename MODEL::Vec3 NormalFromDepthGradient(const MODEL& camera, const Eigen::Vector2i& x, typename MODEL::Scalar depth, const typename MODEL::Vec2& gradient) {
	typedef typename MODEL::Scalar T;
	typedef typename MODEL::Vec3 Vec3;
	ASSERT(depth > T(0));
	const typename MODEL::Mat3 F(camera.RayFrame(x.cast<T>()));
	const Vec3 tx(gradient.x()*F.col(2) + depth*F.col(0));
	const Vec3 ty(gradient.y()*F.col(2) + depth*F.col(1));
	return ty.cross(tx).normalized();
}
// pinhole: the same normal in closed form
template<typename T>
HOST_DEVICE inline typename Pinhole<T>::Vec3 NormalFromDepthGradient(const Pinhole<T>& camera, const Eigen::Vector2i& x, typename Pinhole<T>::Scalar depth, const typename Pinhole<T>::Vec2& gradient) {
	ASSERT(depth > T(0));
	return typename Pinhole<T>::Vec3(
		camera.fx*gradient.x(),
		camera.fy*gradient.y(),
		(camera.cx-(T)x.x())*gradient.x() + (camera.cy-(T)x.y())*gradient.y() - depth).normalized();
}

// depth at pixel x of the plane through neighbor pixel nx at depth nDepth with the given normal
// (camera frame), intersected with the ray through x: depth = nDepth*(n.Ray(nx))/(n.Ray(x));
// nDepth if the plane is parallel to that ray or the new depth is outside [dMin,dMax]
template<typename MODEL>
HOST_DEVICE inline typename MODEL::Scalar InterpolatePlaneDepth(const MODEL& camera, const Eigen::Vector2i& x, const Eigen::Vector2i& nx, typename MODEL::Scalar nDepth, const typename MODEL::Vec3& normal, typename MODEL::Scalar dMin, typename MODEL::Scalar dMax) {
	typedef typename MODEL::Scalar T;
	ASSERT(nDepth > T(0) && dMin < dMax);
	const T denom(normal.dot(camera.Ray(x.cast<T>())));
	if (Eigen::numext::abs(denom) < Eigen::NumTraits<T>::epsilon())
		return nDepth;
	const T depth(nDepth*normal.dot(camera.Ray(nx.cast<T>()))/denom);
	return depth >= dMin && depth <= dMax ? depth : nDepth;
}
// pinhole: a neighbor on the same column or row is intersected in that slice, any other one along
// the ray through x
template<typename T>
HOST_DEVICE inline T InterpolatePlaneDepth(const Pinhole<T>& camera, const Eigen::Vector2i& x, const Eigen::Vector2i& nx, typename Pinhole<T>::Scalar nDepth, const typename Pinhole<T>::Vec3& normal, typename Pinhole<T>::Scalar dMin, typename Pinhole<T>::Scalar dMax) {
	ASSERT(nDepth > T(0) && dMin < dMax);
	T num, denom;
	if (x.x() == nx.x()) {
		const T rayY = ((T)x.y()-camera.cy)/camera.fy;
		const T nRayY = ((T)nx.y()-camera.cy)/camera.fy;
		denom = normal.z() + rayY*normal.y();
		num = nDepth*(normal.z() + nRayY*normal.y());
	} else if (x.y() == nx.y()) {
		const T rayX = ((T)x.x()-camera.cx)/camera.fx;
		const T nRayX = ((T)nx.x()-camera.cx)/camera.fx;
		denom = normal.z() + rayX*normal.x();
		num = nDepth*(normal.z() + nRayX*normal.x());
	} else {
		num = normal.dot(typename Pinhole<T>::Vec3(nDepth*((T)nx.x()-camera.cx)/camera.fx, nDepth*((T)nx.y()-camera.cy)/camera.fy, nDepth));
		denom = normal.dot(typename Pinhole<T>::Vec3(((T)x.x()-camera.cx)/camera.fx, ((T)x.y()-camera.cy)/camera.fy, T(1)));
	}
	if (Eigen::numext::abs(denom) < Eigen::NumTraits<T>::epsilon())
		return nDepth;
	const T depth = num/denom;
	return depth >= dMin && depth <= dMax ? depth : nDepth;
}

// homography H = Rrel + t*n^T/(n.X) mapping reference camera-frame rays to target camera-frame rays
// through the plane with normal n containing the reference camera-frame point X, where Rrel = Rj*Rref^T
// and t = Rj*(Cref-Cj) give the target pose relative to the reference
template<typename MAT3, typename VEC3>
HOST_DEVICE inline MAT3 PlaneRayHomography(const MAT3& Rrel, const VEC3& t, const VEC3& normal, const VEC3& X) {
	ASSERT(normal.dot(X) < typename VEC3::Scalar(0));
	return Rrel + t*(normal.transpose()/normal.dot(X));
}
// plane-induced patch warp from reference pixel offsets (dx,dy,1) around x0 to homogeneous pixels of
// a target view: M = trg.PixelHomography(H*r0)*H*ref.RayFrame(x0), with H the PlaneRayHomography
template<typename REF, typename TRG>
HOST_DEVICE inline typename REF::Mat3 PlanePatchWarp(const REF& ref, const TRG& trg, const typename REF::Mat3& Rrel, const typename REF::Vec3& t, const typename REF::Vec3& normal, const typename REF::Vec3& X, const typename REF::Vec2& x0) {
	typedef typename REF::Mat3 Mat3;
	const Mat3 H(PlaneRayHomography(Rrel, t, normal, X));
	const Mat3 F(ref.RayFrame(x0));
	return trg.PixelHomography(H*F.col(2))*H*F;
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_DEPTHGEOMETRY_H__
