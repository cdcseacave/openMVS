////////////////////////////////////////////////////////////////////
// Plane.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// Construct plane given its normal and the distance from the origin.
template <typename TYPE, int DIMS>
inline TPlane<TYPE,DIMS>::TPlane(const VECTOR& vN, TYPE fD)
	:
	m_vN(vN), m_fD(fD)
{
}
// Construct plane given its normal and a point on the plane.
template <typename TYPE, int DIMS>
inline TPlane<TYPE,DIMS>::TPlane(const VECTOR& vN, const POINT& p)
	:
	m_vN(vN), m_fD(-vN.dot(p))
{
}
// Construct plane given three points on the plane.
template <typename TYPE, int DIMS>
inline TPlane<TYPE,DIMS>::TPlane(const POINT& p0, const POINT& p1, const POINT& p2)
{
	Set(p0, p1, p2);
}
// Construct plane given its standard equation: Ax + By + Cz + D = 0
template <typename TYPE, int DIMS>
inline TPlane<TYPE,DIMS>::TPlane(const TYPE p[DIMS+1])
{
	Set(p);
} // constructors
/*----------------------------------------------------------------*/

template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Set(const VECTOR& vN, TYPE fD)
{
	m_vN = vN;
	m_fD = fD;
}
template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Set(const VECTOR& vN, const POINT& p)
{
	m_vN = vN;
	m_fD = -vN.dot(p);
}
template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Set(const POINT& p0, const POINT& p1, const POINT& p2)
{
	const VECTOR vcEdge1 = p1 - p0;
	const VECTOR vcEdge2 = p2 - p0;
	m_vN = vcEdge1.cross(vcEdge2).normalized();
	m_fD = -m_vN.dot(p0);
}
template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Set(const TYPE p[DIMS+1])
{
	const Eigen::Map<const VECTOR> vN(p);
	const TYPE invD(INVERT(vN.norm()));
	Set(vN*invD, p[3]*invD);
} // Set
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Negate()
{
	m_vN = -m_vN;
	m_fD = -m_fD;
}
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline TYPE TPlane<TYPE,DIMS>::Distance(const TPlane& p) const
{
	return ABS(m_fD - p.m_fD);
}
/*----------------------------------------------------------------*/

// Calculate distance to point. Plane normal must be normalized.
template <typename TYPE, int DIMS>
inline TYPE TPlane<TYPE,DIMS>::Distance(const POINT& p) const
{
	return m_vN.dot(p) + m_fD;
}
template <typename TYPE, int DIMS>
inline TYPE TPlane<TYPE,DIMS>::DistanceAbs(const POINT& p) const
{
	return ABS(Distance(p));
}
/*----------------------------------------------------------------*/


// Classify point to plane.
template <typename TYPE, int DIMS>
inline GCLASS TPlane<TYPE,DIMS>::Classify(const POINT& p) const
{
	const TYPE f(Distance(p));
	if (f >  ZEROTOLERANCE<TYPE,DIMS>()) return FRONT;
	if (f < -ZEROTOLERANCE<TYPE,DIMS>()) return BACK;
	return PLANAR;
}
/*----------------------------------------------------------------*/


// Classify bounding box to plane.
template <typename TYPE, int DIMS>
inline GCLASS TPlane<TYPE,DIMS>::Classify(const AABB& aabb) const
{
	const GCLASS classMin = Classify(aabb.ptMin);
	const GCLASS classMax = Classify(aabb.ptMax);
	if (classMin == classMax) return classMin;
	return CLIPPED;
}
/*----------------------------------------------------------------*/


// clips a ray into two segments if it intersects the plane
template <typename TYPE, int DIMS>
bool TPlane<TYPE,DIMS>::Clip(const RAY& ray, TYPE fL, RAY* pF, RAY* pB) const
{
	POINT ptHit = POINT::ZERO;

	// ray intersects plane at all?
	if (!ray.Intersects(*this, false, fL, NULL, &ptHit)) 
		return false;

	GCLASS n = Classify(ray.m_pOrig);

	// ray comes from planes backside
	if ( n == BACK ) {
		if (pB) pB->Set(ray.m_pOrig, ray.m_vDir);
		if (pF) pF->Set(ptHit, ray.m_vDir);
	}
	// ray comes from planes front side
	else if ( n == FRONT ) {
		if (pF) pF->Set(ray.m_pOrig, ray.m_vDir);
		if (pB) pB->Set(ptHit, ray.m_vDir);
	}

	return true;
} // Clip(Ray)
/*----------------------------------------------------------------*/


// Intersection of two planes.
// Returns the line of intersection.
// http://paulbourke.net/geometry/pointlineplane/
template <typename TYPE, int DIMS>
bool TPlane<TYPE,DIMS>::Intersects(const TPlane& plane, RAY& ray) const
{
	// if crossproduct of normals 0 than planes parallel
	const VECTOR vCross(m_vN.cross(plane.m_vN));
	const TYPE fSqrLength(vCross.squaredNorm());
	if (fSqrLength < ZEROTOLERANCE<TYPE,DIMS>()) 
		return false;

	// find line of intersection
	#if 0
	// the general case
	const TYPE fN00 = m_vN.squaredNorm();
	const TYPE fN01 = m_vN.dot(plane.m_vN);
	const TYPE fN11 = plane.m_vN.squaredNorm();
	const TYPE fDet = fN01*fN01 - fN00*fN11;
	const TYPE fInvDet = INVERT(fDet);
	const TYPE fC0 = (fN11*m_fD - fN01*plane.m_fD) * fInvDet;
	const TYPE fC1 = (fN00*plane.m_fD - fN01*m_fD) * fInvDet;
	#else
	// plane normals assumed to be normalized vectors
	ASSERT(ISEQUAL(m_vN.norm(), TYPE(1)));
	ASSERT(ISEQUAL(plane.m_vN.norm(), TYPE(1)));
	const TYPE fN01 = m_vN.dot(plane.m_vN);
	const TYPE fInvDet = INVERT(fN01*fN01-TYPE(1));
	const TYPE fC0 = (m_fD - fN01*plane.m_fD) * fInvDet;
	const TYPE fC1 = (plane.m_fD - fN01*m_fD) * fInvDet;
	#endif

	ray.m_vDir = vCross * RSQRT(fSqrLength);
	ray.m_pOrig = m_vN*fC0 + plane.m_vN*fC1;
	return true;
} // Intersects(Plane)
/*----------------------------------------------------------------*/


// Intersection of a plane with a triangle. If all vertices of the
// triangle are on the same side of the plane, no intersection occurred. 
template <typename TYPE, int DIMS>
bool TPlane<TYPE,DIMS>::Intersects(const POINT& p0, const POINT& p1, const POINT& p2) const
{
	const GCLASS n(Classify(p0));
	if ((n == Classify(p1)) && 
		(n == Classify(p2)))
		return false;
	return true;
} // Intersects(Tri)
/*----------------------------------------------------------------*/


// Intersection with AABB. Search for AABB diagonal that is most
// aligned to plane normal. Test its two vertices against plane.
// (M�ller/Haines, "Real-Time Rendering")
template <typename TYPE, int DIMS>
bool TPlane<TYPE,DIMS>::Intersects(const AABB& aabb) const
{
	POINT Vmin, Vmax;

	// x component
	if (m_vN(0) >= TYPE(0)) {
		Vmin(0) = aabb.ptMin(0);
		Vmax(0) = aabb.ptMax(0);
	}
	else {
		Vmin(0) = aabb.ptMax(0);
		Vmax(0) = aabb.ptMin(0);
	}

	// y component
	if (m_vN(1) >= TYPE(0)) {
		Vmin(1) = aabb.ptMin(1);
		Vmax(1) = aabb.ptMax(1);
	}
	else {
		Vmin(1) = aabb.ptMax(1);
		Vmax(1) = aabb.ptMin(1);
	}

	// z component
	if (m_vN(2) >= TYPE(0)) {
		Vmin(2) = aabb.ptMin(2);
		Vmax(2) = aabb.ptMax(2);
	}
	else {
		Vmin(2) = aabb.ptMax(2);
		Vmax(2) = aabb.ptMin(2);
	}

	if (((m_vN * Vmin) + m_fD) > TYPE(0))
		return false;

	if (((m_vN * Vmax) + m_fD) >= TYPE(0))
		return true;

	return false;
} // Intersects(AABB)
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

// Construct frustum given a projection matrix.
template <typename TYPE, int DIMS>
inline TFrustum<TYPE,DIMS>::TFrustum(const MATRIX4x4& m)
{
	Set<0>(m);
}
template <typename TYPE, int DIMS>
inline TFrustum<TYPE,DIMS>::TFrustum(const MATRIX3x4& m)
{
	Set<0>(m);
}
template <typename TYPE, int DIMS>
inline TFrustum<TYPE,DIMS>::TFrustum(const MATRIX4x4& m, TYPE w, TYPE h, TYPE n, TYPE f)
{
	Set(m, w, h, n, f);
}
template <typename TYPE, int DIMS>
inline TFrustum<TYPE,DIMS>::TFrustum(const MATRIX3x4& m, TYPE w, TYPE h, TYPE n, TYPE f)
{
	Set(m, w, h, n, f);
} // Constructor
/*----------------------------------------------------------------*/


/**
 * Retrieve active frustum planes, normals pointing outwards.
 * -> IN/OUT: RDFRUSTUM - address to 6 planes
 */
template <typename TYPE, int DIMS>
template <int MODE>
void TFrustum<TYPE,DIMS>::Set(const MATRIX4x4& m)
{
	// left plane
	m_planes[0].Set(-(m.col(3)+m.col(0)));
	// right plane
	if (DIMS > 1)
	m_planes[1].Set(-(m.col(3)-m.col(0)));
	// top plane
	if (DIMS > 2)
	m_planes[2].Set(-(m.col(3)+m.col(1)));
	// bottom plane
	if (DIMS > 3)
	m_planes[3].Set(-(m.col(3)-m.col(1)));
	// near plane
	if (DIMS > 4)
	m_planes[4].Set(MODE ? -(m.col(3)+m.col(2)) : -m.col(2));
	// far plane
	if (DIMS > 5)
	m_planes[5].Set(-(m.col(3)-m.col(2)));
}
// same as above, but the last row of the matrix is (0,0,0,1)
template <typename TYPE, int DIMS>
template <int MODE>
void TFrustum<TYPE,DIMS>::Set(const MATRIX3x4& m)
{
	// left plane
	m_planes[0].Set(VECTOR4(
		-(m(0,3)+m(0,0)),
		-(m(1,3)+m(1,0)),
		-(m(2,3)+m(2,0)),
		-TYPE(1)
	));
	// right plane
	if (DIMS > 1)
	m_planes[1].Set(VECTOR4(
		-(m(0,3)-m(0,0)),
		-(m(1,3)-m(1,0)),
		-(m(2,3)-m(2,0)),
		-TYPE(1)
	));
	// top plane
	if (DIMS > 2)
	m_planes[2].Set(VECTOR4(
		-(m(0,3)+m(0,1)),
		-(m(1,3)+m(1,1)),
		-(m(2,3)+m(2,1)),
		-TYPE(1)
	));
	// bottom plane
	if (DIMS > 3)
	m_planes[3].Set(VECTOR4(
		-(m(0,3)-m(0,1)),
		-(m(1,3)-m(1,1)),
		-(m(2,3)-m(2,1)),
		-TYPE(1)
	));
	// near plane
	if (DIMS > 4)
	m_planes[4].Set(MODE ?
		VECTOR4(
		-(m(0,3)+m(0,2)),
		-(m(1,3)+m(1,2)),
		-(m(2,3)+m(2,2)),
		-TYPE(1))
		:
		VECTOR4(
		-m(0,2),
		-m(1,2),
		-m(2,2),
		TYPE(0))
	);
	// far plane
	if (DIMS > 5)
	m_planes[5].Set(VECTOR4(
		-(m(0,3)-m(0,2)),
		-(m(1,3)-m(1,2)),
		-(m(2,3)-m(2,2)),
		-TYPE(1)
	));
}
// same as above, but from SfM projection matrix and image plane details
template <typename TYPE, int DIMS>
void TFrustum<TYPE,DIMS>::Set(const MATRIX4x4& m, TYPE w, TYPE h, TYPE n, TYPE f)
{
	const VECTOR4 ltn(0,0,n,1), rtn(w*n,0,n,1), lbn(0,h*n,n,1), rbn(w*n,h*n,n,1);
	const VECTOR4 ltf(0,0,f,1), rtf(w*f,0,f,1), lbf(0,h*f,f,1), rbf(w*f,h*f,f,1);
	const MATRIX4x4 inv(m.inverse());
	const VECTOR4 ltn3D(inv*ltn), rtn3D(inv*rtn), lbn3D(inv*lbn), rbn3D(inv*rbn);
	const VECTOR4 ltf3D(inv*ltf), rtf3D(inv*rtf), lbf3D(inv*lbf), rbf3D(inv*rbf);
	m_planes[0].Set(ltn3D.template topRows<3>(), ltf3D.template topRows<3>(), lbf3D.template topRows<3>());
	if (DIMS > 1)
	m_planes[1].Set(rtn3D.template topRows<3>(), rbf3D.template topRows<3>(), rtf3D.template topRows<3>());
	if (DIMS > 2)
	m_planes[2].Set(ltn3D.template topRows<3>(), rtf3D.template topRows<3>(), ltf3D.template topRows<3>());
	if (DIMS > 3)
	m_planes[3].Set(lbn3D.template topRows<3>(), lbf3D.template topRows<3>(), rbf3D.template topRows<3>());
	if (DIMS > 4)
	m_planes[4].Set(ltn3D.template topRows<3>(), lbn3D.template topRows<3>(), rbn3D.template topRows<3>());
	if (DIMS > 5)
	m_planes[5].Set(ltf3D.template topRows<3>(), rtf3D.template topRows<3>(), rbf3D.template topRows<3>());
}
template <typename TYPE, int DIMS>
void TFrustum<TYPE,DIMS>::Set(const MATRIX3x4& m, TYPE w, TYPE h, TYPE n, TYPE f)
{
	MATRIX4x4 M(MATRIX4x4::Identity());
	#ifdef __GNUC__
	M.topLeftCorner(3,4) = m;
	#else
	M.template topLeftCorner<3,4>() = m;
	#endif
	Set(M, w, h, n, f);
} // Set
/*----------------------------------------------------------------*/


/**
 * Culls POINT to n sided frustum. Normals pointing outwards.
 * -> IN:  POINT   - point to be tested
 *    OUT: VISIBLE - point inside frustum
 *         CULLED  - point outside frustum
 */
template <typename TYPE, int DIMS>
GCLASS TFrustum<TYPE,DIMS>::Classify(const POINT& p) const
{
	// check if on the front side of any of the planes
	for (int i=0; i<DIMS; ++i) {
		if (m_planes[i].Classify(p) == FRONT)
			return CULLED;
	} // for
	return VISIBLE;
}

/**
 * Culls SPHERE to n sided frustum. Normals pointing outwards.
 * -> IN:  POINT   - center of the sphere to be tested
 *         TYPE    - radius of the sphere to be tested
 *    OUT: VISIBLE - sphere inside frustum
 *         CLIPPED - sphere clipped by frustum
 *         CULLED  - sphere outside frustum
 */
template <typename TYPE, int DIMS>
GCLASS TFrustum<TYPE,DIMS>::Classify(const SPHERE& s) const
{
	// compute distances to each of the planes
	for (int i=0; i<DIMS; ++i) {
		// compute the distance to this plane
		const TYPE dist(m_planes[i].Distance(s.center));
		// if distance is bigger than the sphere radius, the sphere is outside
		if (dist > s.radius)
			return CULLED;
		// if the distance is between +- radius, the sphere intersects the frustum
		if (ABS(dist) < s.radius)
			return CLIPPED;
	} // for
	// otherwise sphere is fully in view
	return VISIBLE;
}

/**
 * Culls AABB to n sided frustum. Normals pointing outwards.
 * -> IN:  AABB    - bounding box to be tested
 *    OUT: VISIBLE - aabb totally inside frustum
 *         CLIPPED - aabb clipped by frustum
 *         CULLED  - aabb totally outside frustum
 */
template <typename TYPE, int DIMS>
GCLASS TFrustum<TYPE,DIMS>::Classify(const AABB& aabb) const
{
	bool bIntersects = false;

	// find and test extreme points
	for (int i=0; i<DIMS; ++i) {
		const PLANE& plane = m_planes[i];
		POINT ptPlaneMin, ptPlaneMax;

		// x coordinate
		if (plane.m_vN(0) >= TYPE(0)) {
			ptPlaneMin(0) = aabb.ptMin(0);
			ptPlaneMax(0) = aabb.ptMax(0);
		} else {
			ptPlaneMin(0) = aabb.ptMax(0);
			ptPlaneMax(0) = aabb.ptMin(0);
		}
		// y coordinate
		if (plane.m_vN(1) >= TYPE(0)) {
			ptPlaneMin(1) = aabb.ptMin(1);
			ptPlaneMax(1) = aabb.ptMax(1);
		} else {
			ptPlaneMin(1) = aabb.ptMax(1);
			ptPlaneMax(1) = aabb.ptMin(1);
		}
		// z coordinate
		if (plane.m_vN(2) >= TYPE(0)) {
			ptPlaneMin(2) = aabb.ptMin(2);
			ptPlaneMax(2) = aabb.ptMax(2);
		} else {
			ptPlaneMin(2) = aabb.ptMax(2);
			ptPlaneMax(2) = aabb.ptMin(2);
		}

		if (plane.m_vN.dot(ptPlaneMin) > -plane.m_fD)
			return CULLED;

		if (plane.m_vN.dot(ptPlaneMax) >= -plane.m_fD)
			bIntersects = true;
	} // for

	if (bIntersects) return CLIPPED;
	return VISIBLE;
}
/*----------------------------------------------------------------*/
