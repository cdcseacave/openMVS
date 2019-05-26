////////////////////////////////////////////////////////////////////
// Ray.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
inline TTriangle<TYPE,DIMS>::TTriangle(const POINT& p0, const POINT& p1, const POINT& p2)
	:
	a(p0), b(p1), c(p2)
{
} // constructors
/*----------------------------------------------------------------*/

// set attributes
template <typename TYPE, int DIMS>
inline void TTriangle<TYPE,DIMS>::Set(const POINT& p0, const POINT& p1, const POINT& p2)
{
	a = p0;
	b = p1;
	c = p2;
}
/*----------------------------------------------------------------*/


// get AABB
template <typename TYPE, int DIMS>
inline typename TTriangle<TYPE,DIMS>::AABB TTriangle<TYPE,DIMS>::GetAABB() const
{
	return AABB(& operator [] (0), 3);
}
// get plane
template <typename TYPE, int DIMS>
inline typename TTriangle<TYPE,DIMS>::PLANE TTriangle<TYPE,DIMS>::GetPlane() const
{
	return PLANE(a, b, c);
}
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
inline TRay<TYPE,DIMS>::TRay(const POINT& pOrig, const VECTOR& vDir)
	:
	m_vDir(vDir), m_pOrig(pOrig)
{
	ASSERT(ISEQUAL(m_vDir.squaredNorm(), TYPE(1)));
} // constructors
template <typename TYPE, int DIMS>
inline TRay<TYPE,DIMS>::TRay(const POINT& pt0, const POINT& pt1, bool /*bPoints*/)
	:
	m_vDir((pt1-pt0).normalized()), m_pOrig(pt0)
{
} // constructors
/*----------------------------------------------------------------*/

// set attributes
template <typename TYPE, int DIMS>
inline void TRay<TYPE,DIMS>::Set(const POINT& pOrig, const VECTOR& vDir)
{
	m_vDir = vDir;
	m_pOrig = pOrig;
	ASSERT(ISEQUAL(m_vDir.squaredNorm(), TYPE(1)));
}
template <typename TYPE, int DIMS>
inline void TRay<TYPE,DIMS>::SetFromPoints(const POINT& pt0, const POINT& pt1)
{
	m_vDir = (pt1-pt0).normalized();
	m_pOrig = pt0;
}
template <typename TYPE, int DIMS>
inline TYPE TRay<TYPE,DIMS>::SetFromPointsLen(const POINT& pt0, const POINT& pt1)
{
	m_vDir = pt1-pt0;
	const TYPE len = m_vDir.norm();
	if (len > TYPE(0))
		m_vDir *= TYPE(1)/len;
	m_pOrig = pt0;
	return len;
}
/*----------------------------------------------------------------*/


// transform ray into matrix space
template <typename TYPE, int DIMS>
inline void TRay<TYPE,DIMS>::DeTransform(const MATRIX& _m)
{
	STATIC_ASSERT(DIMS == 3);
	MATRIX m(_m);

	// invert translation
	typedef Eigen::Matrix<TYPE,4,1> VECTOR4;
	const VECTOR4 vcOrig(m_pOrig[0]-m(3,0), m_pOrig[1]-m(3,1), m_pOrig[2]-m(3,2));

	// delete it from matrix
	m(3,0) = m(3,1) = m(3,2) = TYPE(0);

	// invert matrix and apply to ray
	const MATRIX mInv = m.inverse();
	m_pOrig = vcOrig * mInv;
	m_vDir = VECTOR4(m_vDir) * mInv;
}
/*----------------------------------------------------------------*/


// Transform the ray by a matrix.
template <typename TYPE, int DIMS>
TRay<TYPE,DIMS> TRay<TYPE,DIMS>::operator*(const MATRIX& m) const
{
	TRay ray;
	ray.SetFromPoints(m_pOrig*m, (m_pOrig+m_vDir)*m);
	return ray;
} // operator *
template <typename TYPE, int DIMS>
inline TRay<TYPE,DIMS>& TRay<TYPE,DIMS>::operator*=(const MATRIX& m)
{
	*this = operator * (m);
	return *this;
} // operator *=
/*----------------------------------------------------------------*/


// test for intersection with triangle
template <typename TYPE, int DIMS>
template <bool bCull>
bool TRay<TYPE,DIMS>::Intersects(const TRIANGLE& tri, TYPE *t) const
{
	const VECTOR edge1(tri.b - tri.a);
	const VECTOR edge2(tri.c - tri.a);

	// if close to 0 ray is parallel
	const VECTOR pvec(m_vDir.cross(edge2));
	const TYPE det(edge1.dot(pvec));
	if ((bCull && (det < ZEROTOLERANCE<TYPE>())) || ISZERO(det))
		return false;

	// distance to plane, < 0 means beyond plane
	const VECTOR tvec(m_pOrig - tri.a);
	const TYPE u(tvec.dot(pvec));
	if (u < TYPE(0) || u > det)
		return false;

	const VECTOR qvec(tvec.cross(edge1));
	const TYPE v(m_vDir.dot(qvec));
	if (v < TYPE(0) || u+v > det)
		return false;

	if (t)
		*t = (edge2.dot(qvec)) / det;

	return true;
} // Intersects(Tri)
/*----------------------------------------------------------------*/

// test for intersection with triangle at certain length (line segment),
// same as above but test distance to intersection vs segment length.
template <typename TYPE, int DIMS>
template <bool bCull>
bool TRay<TYPE,DIMS>::Intersects(const TRIANGLE& tri, TYPE fL, TYPE *t) const
{
	const VECTOR edge1(tri.b - tri.a);
	const VECTOR edge2(tri.c - tri.a);

	// if close to 0 ray is parallel
	const VECTOR pvec(m_vDir.cross(edge2));
	const TYPE det(edge1.dot(pvec));
	if ((bCull && (det < ZEROTOLERANCE<TYPE>())) || ISZERO(det))
		return false;

	// distance to plane, < 0 means beyond plane
	const VECTOR tvec(m_pOrig - tri.a);
	const TYPE u(tvec.dot(pvec));
	if (u < TYPE(0) || u > det)
		return false;

	const VECTOR qvec(tvec.cross(edge1));
	const TYPE v(m_vDir.dot(qvec));
	if (v < TYPE(0) || u+v > det)
		return false;

	if (t) {
		*t = (edge2.dot(qvec)) / det;
		// collision but not on segment?
		if (*t > fL) return false;
	}
	else {
		// collision but not on segment?
		if ((edge2.dot(qvec)) / det > fL) return false;
	}

	return true;
} // Intersects(Tri at length)
/*----------------------------------------------------------------*/


// test for intersection with triangle
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const POINT& p0, const POINT& p1, const POINT& p2, bool bCull, TYPE *t) const
{
	const VECTOR edge1(p1 - p0);
	const VECTOR edge2(p2 - p0);

	// if close to 0 ray is parallel
	const VECTOR pvec = m_vDir.cross(edge2);
	const TYPE det = edge1.dot(pvec);
	if ((bCull && (det < ZEROTOLERANCE<TYPE>())) || ISZERO(det))
		return false;

	// distance to plane, < 0 means beyond plane
	const VECTOR tvec = m_pOrig - p0;
	const TYPE u = tvec * pvec;
	if (u < TYPE(0) || u > det)
		return false;

	const VECTOR qvec = tvec.cross(edge1);
	const TYPE v = m_vDir.dot(qvec);
	if (v < TYPE(0) || u+v > det)
		return false;

	if (t)
		*t = (edge2.dot(qvec)) / det;

	return true;
} // Intersects(Tri)
/*----------------------------------------------------------------*/


// test for intersection with triangle at certain length (line segment),
// same as above but test distance to intersection vs segment length.
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const POINT& p0, const POINT& p1, const POINT& p2, bool bCull, TYPE fL, TYPE *t) const
{
	const VECTOR edge1(p1 - p0);
	const VECTOR edge2(p2 - p0);

	// if close to 0 ray is parallel
	const VECTOR pvec = m_vDir.cross(edge2);
	const TYPE det = edge1.dot(pvec);
	if ((bCull && (det < ZEROTOLERANCE<TYPE>())) || ISZERO(det))
		return false;

	// distance to plane, < 0 means beyond plane
	const VECTOR tvec = m_pOrig - p0;
	const TYPE u = tvec.dot(pvec);
	if (u < 0.0f || u > det)
		return false;

	const VECTOR qvec = tvec.cross(edge1);
	const TYPE v = m_vDir.dot(qvec);
	if (v < TYPE(0) || u+v > det)
		return false;

	if (t) {
		*t = (edge2.dot(qvec)) / det;
		// collision but not on segment?
		if (*t > fL) return false;
	}
	else {
		// collision but not on segment?
		if ((edge2.dot(qvec)) / det > fL) return false;
	}

	return true;
} // Intersects(Tri at length)
/*----------------------------------------------------------------*/


// test if the ray intersects the given sphere
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const SPHERE& sphere) const
{
	TYPE dSq;
	if (!DistanceSq(sphere.center, dSq))
		return false;
	return dSq <= SQUARE(sphere.radius);
}
// same as above, but returns also the distance on the ray corresponding to the closest point
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const SPHERE& sphere, TYPE& t) const
{
	const VECTOR a(sphere.center - m_pOrig);
	t = a.dot(m_vDir);
	// point behind the ray origin
	if (t < TYPE(0))
		return false;
	const TYPE dSq((a - m_vDir*t).squaredNorm());
	return dSq <= SQUARE(sphere.radius);
} // Intersects(Sphere)
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const AABB &aabb) const
{
	bool to_infinity = true;
	TYPE _min, _max;
	// first on x value
	if (m_vDir[0] == TYPE(0)) {
		if (m_pOrig[0] < aabb.ptMin[0] || m_pOrig[0] > aabb.ptMax[0])
			return false; // NO_INTERSECTION
	} else {
		if (m_vDir[0] > TYPE(0)) {
			_min = (aabb.ptMin[0]-m_pOrig[0])/m_vDir[0];
			_max = (aabb.ptMax[0]-m_pOrig[0])/m_vDir[0];
		} else {
			_min = (aabb.ptMax[0]-m_pOrig[0])/m_vDir[0];
			_max = (aabb.ptMin[0]-m_pOrig[0])/m_vDir[0];
		}
		to_infinity = false;
	}
	// now on y value
	if (m_vDir[1] == TYPE(0)) {
		if (m_pOrig[1] < aabb.ptMin[1] || m_pOrig[1] > aabb.ptMax[1])
			return false; // NO_INTERSECTION
	} else {
		TYPE newmin, newmax;
		if (m_vDir[1] > TYPE(0)) {
			newmin = (aabb.ptMin[1]-m_pOrig[1])/m_vDir[1];
			newmax = (aabb.ptMax[1]-m_pOrig[1])/m_vDir[1];
		} else {
			newmin = (aabb.ptMax[1]-m_pOrig[1])/m_vDir[1];
			newmax = (aabb.ptMin[1]-m_pOrig[1])/m_vDir[1];
		}
		#if 0
		if (to_infinity) {
			_min = newmin;
			_max = newmax;
		} else {
		#else
		if (to_infinity)
			return true;
		{
		#endif
			if (newmin > _min)
				_min = newmin;
			if (newmax < _max)
				_max = newmax;
			if (_max < _min)
				return false; // NO_INTERSECTION
		}
		to_infinity = false;
	}
	// now on z value
	if (DIMS == 3) {
	if (m_vDir[2] == TYPE(0)) {
		if (m_pOrig[2] < aabb.ptMin[2] || m_pOrig[2] > aabb.ptMax[2])
			return false; // NO_INTERSECTION
	} else {
		TYPE newmin, newmax;
		if (m_vDir[2] > TYPE(0)) {
			newmin = (aabb.ptMin[2]-m_pOrig[2])/m_vDir[2];
			newmax = (aabb.ptMax[2]-m_pOrig[2])/m_vDir[2];
		} else {
			newmin = (aabb.ptMax[2]-m_pOrig[2])/m_vDir[2];
			newmax = (aabb.ptMin[2]-m_pOrig[2])/m_vDir[2];
		}
		#if 0
		if (to_infinity) {
			_min = newmin;
			_max = newmax;
		} else {
		#else
		if (to_infinity)
			return true;
		{
		#endif
			if (newmin > _min)
				_min = newmin;
			if (newmax < _max)
				_max = newmax;
			if (_max < _min)
				return false; // NO_INTERSECTION
		}
		to_infinity = false;
	}
	}
	ASSERT(!to_infinity);
	#if 0
	if (_max < _min)
		return true; // POINT_INTERSECTION
	#endif
	return true; // SEGMENT_INTERSECTION
} // Intersects(AABB)

// test for intersection with aabb, original code by Andrew Woo,
// from "Geometric Tools...", Morgan Kaufmann Publ., 2002
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const AABB &aabb, TYPE& t) const
{
	TYPE t0, t1, tmp;
	TYPE tNear(-999999);
	TYPE tFar ( 999999);

	// first pair of planes
	if (ISZERO(m_vDir[0])) {
		if ((m_pOrig[0] < aabb.ptMin[0]) ||
			(m_pOrig[0] > aabb.ptMax[0]))
			return false;
	}
	t0 = (aabb.ptMin[0] - m_pOrig[0]) / m_vDir[0];
	t1 = (aabb.ptMax[0] - m_pOrig[0]) / m_vDir[0];
	if (t0 > t1) { tmp=t0; t0=t1; t1=tmp; }
	if (t0 > tNear) tNear = t0;
	if (t1 < tFar)  tFar = t1;
	if (tNear > tFar) return false;
	if (tFar < TYPE(0)) return false;

	// second pair of planes
	if (ISZERO(m_vDir[1])) {
		if ((m_pOrig[1] < aabb.ptMin[1]) ||
			(m_pOrig[1] > aabb.ptMax[1]) )
			return false;
	}
	t0 = (aabb.ptMin[1] - m_pOrig[1]) / m_vDir[1];
	t1 = (aabb.ptMax[1] - m_pOrig[1]) / m_vDir[1];
	if (t0 > t1) { tmp=t0; t0=t1; t1=tmp; }
	if (t0 > tNear) tNear = t0;
	if (t1 < tFar)  tFar = t1;
	if (tNear > tFar) return false;
	if (tFar < TYPE(0)) return false;

	if (DIMS == 3) {
	// third pair of planes
	if (ISZERO(m_vDir[2])) {
		if ((m_pOrig[2] < aabb.ptMin[2]) ||
			(m_pOrig[2] > aabb.ptMax[2]) )
			return false;
	}
	t0 = (aabb.ptMin[2] - m_pOrig[2]) / m_vDir[2];
	t1 = (aabb.ptMax[2] - m_pOrig[2]) / m_vDir[2];
	if (t0 > t1) { tmp=t0; t0=t1; t1=tmp; }
	if (t0 > tNear) tNear = t0;
	if (t1 < tFar)  tFar = t1;
	if (tNear > tFar) return false;
	if (tFar < TYPE(0)) return false;
	}

	t = (tNear > TYPE(0) ? tNear : tFar);
	return true;
} // Intersects(AABB)

// test for intersection with aabb, original code by Andrew Woo,
// from "Geometric Tools...", Morgan Kaufmann Publ., 2002
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const AABB &aabb, TYPE fL, TYPE *t) const
{
	TYPE t0, t1, tmp, tFinal;
	TYPE tNear(-999999);
	TYPE tFar ( 999999);

	// first pair of planes
	if (ISZERO(m_vDir[0])) {
		if ((m_pOrig[0] < aabb.ptMin[0]) ||
			(m_pOrig[0] > aabb.ptMax[0]) )
			return false;
	}
	t0 = (aabb.ptMin[0] - m_pOrig[0]) / m_vDir[0];
	t1 = (aabb.ptMax[0] - m_pOrig[0]) / m_vDir[0];
	if (t0 > t1) { tmp=t0; t0=t1; t1=tmp; }
	if (t0 > tNear) tNear = t0;
	if (t1 < tFar)  tFar = t1;
	if (tNear > tFar) return false;
	if (tFar < TYPE(0)) return false;

	// second pair of planes
	if (ISZERO(m_vDir[1])) {
		if ((m_pOrig[1] < aabb.ptMin[1]) ||
			(m_pOrig[1] > aabb.ptMax[1]) )
			return false;
	}
	t0 = (aabb.ptMin[1] - m_pOrig[1]) / m_vDir[1];
	t1 = (aabb.ptMax[1] - m_pOrig[1]) / m_vDir[1];
	if (t0 > t1) { tmp=t0; t0=t1; t1=tmp; }
	if (t0 > tNear) tNear = t0;
	if (t1 < tFar)  tFar = t1;
	if (tNear > tFar) return false;
	if (tFar < TYPE(0)) return false;

	if (DIMS == 3) {
	// third pair of planes
	if (ISZERO(m_vDir[2])) {
		if ((m_pOrig[2] < aabb.ptMin[2]) ||
			(m_pOrig[2] > aabb.ptMax[2]) )
			return false;
	}
	t0 = (aabb.ptMin[2] - m_pOrig[2]) / m_vDir[2];
	t1 = (aabb.ptMax[2] - m_pOrig[2]) / m_vDir[2];
	if (t0 > t1) { tmp=t0; t0=t1; t1=tmp; }
	if (t0 > tNear) tNear = t0;
	if (t1 < tFar)  tFar = t1;
	if (tNear > tFar) return false;
	if (tFar < 0) return false;
	}

	tFinal = (tNear > TYPE(0) ? tNear : tFar);

	if (tFinal > fL) return false;
	if (t) *t = tFinal;
	return true;
} // Intersects(AABB) at length
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const OBB& obb) const
{
	TYPE t;
	return Intersects(obb, t);
} // Intersects(OBB)

// test for intersection with obb, slaps method
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const OBB& obb, TYPE& t) const
{
	TYPE e, f, t1, t2, temp;
	TYPE tmin(-999999);
	TYPE tmax( 999999);

	const POINT vP = obb.m_pos - m_pOrig;

	// 1st slap
	e = obb.m_rot.row(0) * vP;
	f = obb.m_rot.row(0) * m_vDir;
	if (!ISZERO(f)) {
		t1 = (e + obb.m_ext[0]) / f;
		t2 = (e - obb.m_ext[0]) / f;

		if (t1 > t2) { temp=t1; t1=t2; t2=temp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
		if (tmax < 0.0f) return false;
	} else
	if (((-e - obb.m_ext[0]) > 0.0f) || ((-e + obb.m_ext[0]) < 0.0f))
		return false;

	// 2nd slap
	e = obb.m_rot.row(1) * vP;
	f = obb.m_rot.row(1) * m_vDir;
	if (!ISZERO(f)) {
		t1 = (e + obb.m_ext[1]) / f;
		t2 = (e - obb.m_ext[1]) / f;

		if (t1 > t2) { temp=t1; t1=t2; t2=temp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
		if (tmax < 0.0f) return false;
	} else
	if (((-e - obb.m_ext[1]) > 0.0f) || ((-e + obb.m_ext[1]) < 0.0f))
		return false;

	// 3rd slap
	e = obb.m_rot.row(2) * vP;
	f = obb.m_rot.row(2) * m_vDir;
	if (!ISZERO(f)) {
		t1 = (e + obb.m_ext[2]) / f;
		t2 = (e - obb.m_ext[2]) / f;

		if (t1 > t2) { temp=t1; t1=t2; t2=temp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
		if (tmax < 0.0f) return false;
	} else
	if (((-e - obb.m_ext[2]) > 0) || ((-e + obb.m_ext[2]) < 0))
		return false;

	if (tmin > 0) {
		if (t) *t = tmin;
		return true;
	}

	t = tmax;
	return true;
} // Intersects(AABB)

// test for intersection with obb at certain length (line segment),
// slaps method but compare result if true to length prior return.
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const OBB& obb, TYPE fL, TYPE *t) const
{
	TYPE e, f, t1, t2, temp;
	TYPE tmin(-999999);
	TYPE tmax( 999999);

	const POINT vP = obb.m_pos - m_pOrig;

	// 1st slap
	e = obb.m_rot.row(0) * vP;
	f = obb.m_rot.row(0) * m_vDir;
	if (!ISZERO(f)) {
		t1 = (e + obb.m_ext[0]) / f;
		t2 = (e - obb.m_ext[0]) / f;

		if (t1 > t2) { temp=t1; t1=t2; t2=temp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
		if (tmax < 0.0f) return false;
	} else
	if (((-e - obb.m_ext[0]) > 0) || ((-e + obb.m_ext[0]) < 0))
		return false;

	// 2nd slap
	e = obb.m_rot.row(1) * vP;
	f = obb.m_rot.row(1) * m_vDir;
	if (!ISZERO(f)) {
		t1 = (e + obb.m_ext[1]) / f;
		t2 = (e - obb.m_ext[1]) / f;

		if (t1 > t2) { temp=t1; t1=t2; t2=temp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
		if (tmax < 0.0f) return false;
	} else
	if (((-e - obb.m_ext[1]) > 0) || ((-e + obb.m_ext[1]) < 0))
		return false;

	// 3rd slap
	e = obb.m_rot.row(2) * vP;
	f = obb.m_rot.row(2) * m_vDir;
	if (!ISZERO(f)) {
		t1 = (e + obb.m_ext[2]) / f;
		t2 = (e - obb.m_ext[2]) / f;

		if (t1 > t2) { temp=t1; t1=t2; t2=temp; }
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
		if (tmax < 0.0f) return false;
	} else
	if (((-e - obb.m_ext[2]) > 0) || ((-e + obb.m_ext[2]) < 0))
		return false;

	if ((tmin > 0) && (tmin <= fL)) {
		if (t) *t = tmin;
		return true;
	}

	// intersection on line but not on segment
	if (tmax > fL) return false;

	if (t) *t = tmax;

	return true;
} // Intersects(AABB) at length
/*----------------------------------------------------------------*/


// Intersection with PLANE from origin till infinity.
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const PLANE& plane, bool bCull, TYPE *t, POINT* pPtHit) const
{
	const TYPE Vd(plane.m_vN.dot(m_vDir));

	// ray parallel to plane
	if (ISZERO(Vd))
		return false;

	// normal pointing away from ray dir
	// => intersection backface if any
	if (bCull && (Vd > TYPE(0)))
		return false;

	const TYPE Vo(-plane.Distance(m_pOrig));

	const TYPE _t(Vo / Vd);

	// intersection behind ray origin
	if (_t < TYPE(0))
		return false;

	if (pPtHit)
		(*pPtHit) = m_pOrig + (m_vDir * _t);

	if (t)
		(*t) = _t;

	return true;
} // Intersects(PLANE)
// same as above, but no checks
template <typename TYPE, int DIMS>
inline TYPE TRay<TYPE,DIMS>::IntersectsDist(const PLANE& plane) const
{
	const TYPE Vd(plane.m_vN.dot(m_vDir));
	const TYPE Vo(-plane.Distance(m_pOrig));
	return SAFEDIVIDE(Vo, Vd);
} // IntersectsDist(PLANE)
template <typename TYPE, int DIMS>
inline typename TRay<TYPE,DIMS>::POINT TRay<TYPE,DIMS>::Intersects(const PLANE& plane) const
{
	return m_pOrig + (m_vDir * IntersectsDist(plane));
} // Intersects(PLANE)
/*----------------------------------------------------------------*/


// Intersection with PLANE at distance fL.
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const PLANE& plane, bool bCull, TYPE fL, TYPE *t, POINT* pPtHit) const
{
	const TYPE Vd = plane.m_vN.dot(m_vDir);

	// ray parallel to plane
	if (ISZERO(Vd))
		return false;

	// normal pointing away from ray dir
	// => intersection backface if any
	if (bCull && (Vd > TYPE(0)))
		return false;

	const TYPE Vo(-plane.Distance(m_pOrig));

	const TYPE _t(Vo / Vd);

	// intersection behind ray origin or beyond valid range
	if ((_t < TYPE(0)) || (_t > fL))
		return false;

	if (pPtHit)
		(*pPtHit) = m_pOrig + (m_vDir * _t);

	if (t)
		(*t) = _t;

	return true;
} // Intersects(PLANE)
/*----------------------------------------------------------------*/


// Intersection with another ray.
// Rays assumed to be coplanar; returns only the distance from the origin of the first ray.
// P = R.origin + s * R.dir
// http://mathworld.wolfram.com/Line-LineIntersection.html
// (works also for 2D lines if z components is set to 0)
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const TRay& ray, TYPE& s) const
{
	const POINT dir(ray.m_pOrig - m_pOrig);
	const POINT n(m_vDir.cross(ray.m_vDir));
	if (!ISZERO(dir.dot(n)))
		return false; // lines are not coplanar
	s = dir.cross(ray.m_vDir).dot(n) / n.squaredNorm();
	return true;
} // Intersects(Ray)
// Same as above, but returns the actual intersection point:
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const TRay& ray, POINT& p) const
{
	TYPE s;
	if (!Intersects(ray, s))
		return false; // lines are not coplanar
	if (s < TYPE(0))
		return false; // intersection behind ray origin
	p = m_pOrig + m_vDir * s;
	return true;
} // Intersects(Ray)
// No coplanarity assumption; returns the shortest line segment connecting the two rays:
// P1 = R1.origin + s1 * R1.dir
// P2 = R2.origin + s2 * R2.dir
// http://paulbourke.net/geometry/pointlineplane/
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Intersects(const TRay& ray, TYPE& s1, TYPE& s2) const
{
	const TYPE d4321(ray.m_vDir.dot(m_vDir));
	const TYPE d4343(ray.m_vDir.dot(ray.m_vDir));
	const TYPE d2121(m_vDir.dot(m_vDir));
	const TYPE denom = d2121 * d4343 - d4321 * d4321;
	if (ISZERO(denom))
		return false; // lines are parallel
	const POINT dir(m_pOrig - ray.m_pOrig);
	const TYPE d1321(dir.dot(m_vDir));
	const TYPE d1343(dir.dot(ray.m_vDir));
	const TYPE numer = d1343 * d4321 - d1321 * d4343;
	s1 = numer / denom;
	s2 = (d1343 + d4321 * s1) / d4343;
	return true;
} // Intersects(Ray)
// Same as above, but returns only middle point of the shortest line segment:
// P = (P1 + P2) / 2
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::IntersectsAprox(const TRay& ray, POINT& p) const
{
	TYPE s1, s2;
	if (!Intersects(ray, s1, s2))
		return false;
	const POINT P1 = m_pOrig + m_vDir * s1;
	const POINT P2 = ray.m_pOrig + ray.m_vDir * s2;
	p = (P1+P2)*TYPE(0.5);
	return true;
} // Intersects(Ray)
// Same as above, but returns the point on the given ray.
// Returns false if intersection not in front of main ray.
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::IntersectsAprox2(const TRay& ray, POINT& p) const
{
	TYPE s1, s2;
	if (!Intersects(ray, s1, s2))
		return false;
	if (s1 < TYPE(0))
		return false; // intersection behind main ray origin
	p = ray.m_pOrig + ray.m_vDir * s2;
	return true;
} // Intersects(Ray)
/*----------------------------------------------------------------*/


// computes the angle between the two lines
template <typename TYPE, int DIMS>
inline TYPE TRay<TYPE,DIMS>::CosAngle(const TRay& ray) const
{
	ASSERT(ISEQUAL(m_vDir.norm(), TYPE(1)));
	ASSERT(ISEQUAL(ray.m_vDir.norm(), TYPE(1)));
	return m_vDir.dot(ray.m_vDir);
}
// tests if the two lines are coplanar (intersect)
template <typename TYPE, int DIMS>
inline bool TRay<TYPE,DIMS>::Coplanar(const TRay& ray) const
{
	const POINT dir(ray.m_pOrig - m_pOrig);
	const POINT n(m_vDir.cross(ray.m_vDir));
	return ISZERO(dir.dot(n));
}
// tests if the two lines are parallel
template <typename TYPE, int DIMS>
inline bool TRay<TYPE,DIMS>::Parallel(const TRay& ray) const
{
	return ISZERO(m_vDir.cross(ray.m_vDir).norm());
}
/*----------------------------------------------------------------*/


// Computes the position on the ray of the point projection.
// Returns 0 if it coincides with the ray origin, positive value if in front, and negative if behind.
template <typename TYPE, int DIMS>
inline TYPE TRay<TYPE,DIMS>::Classify(const POINT& pt) const
{
	ASSERT(ISEQUAL(m_vDir.norm(), TYPE(1)));
	const VECTOR a(pt - m_pOrig);
	return a.dot(m_vDir);
} // Classify(POINT)
template <typename TYPE, int DIMS>
inline typename TRay<TYPE,DIMS>::POINT TRay<TYPE,DIMS>::ProjectPoint(const POINT& pt) const
{
	ASSERT(ISEQUAL(m_vDir.norm(), TYPE(1)));
	return m_pOrig + m_vDir*Classify(pt);
} // ProjectPoint
/*----------------------------------------------------------------*/

// Computes the distance between the ray and a point.
// Returns false if the point is projecting behind the ray origin.
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::DistanceSq(const POINT& pt, TYPE& d) const
{
	const VECTOR a(pt - m_pOrig);
	const TYPE LenACos(a.dot(m_vDir));
	// point behind the ray origin
	if (LenACos < TYPE(0))
		return false;
	d = (a - m_vDir*LenACos).squaredNorm();
	return true;
} // DistanceSq(POINT)
template <typename TYPE, int DIMS>
bool TRay<TYPE,DIMS>::Distance(const POINT& pt, TYPE& d) const
{
	const VECTOR a(pt - m_pOrig);
	const TYPE LenACos(a.dot(m_vDir));
	// point behind the ray origin
	if (LenACos < TYPE(0))
		return false;
	d = (a - m_vDir*LenACos).norm();
	return true;
} // Distance(POINT)
// Same as above, but returns the distance even if the point projection is behind the origin.
template <typename TYPE, int DIMS>
TYPE TRay<TYPE,DIMS>::DistanceSq(const POINT& pt) const
{
	ASSERT(ISEQUAL(m_vDir.norm(), TYPE(1)));
	const VECTOR a(pt - m_pOrig);
	const VECTOR b(a - m_vDir);
	return b.cross(a).squaredNorm();
} // DistanceSq(POINT)
template <typename TYPE, int DIMS>
TYPE TRay<TYPE,DIMS>::Distance(const POINT& pt) const
{
	return SQRT(DistanceSq(pt));
} // Distance(POINT)
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
bool TCylinder<TYPE,DIMS>::Intersects(const SPHERE& sphere) const
{
	struct Check {
		static bool Intersection(const RAY& ray, const SPHERE& sphere, const VECTOR& CmV, TYPE t, TYPE radius, TYPE height) {
			const POINT O(ray.m_pOrig + ray.m_vDir * height);
			const VECTOR a(sphere.center - O);
			if (a.squaredNorm() <= SQUARE(sphere.radius))
				return true; // ray origin inside the sphere
			const POINT D((CmV - ray.m_vDir * t).normalized());
			const TYPE d = a.dot(D);
			if (d < TYPE(0))
				return false; // intersection behind the ray origin
			const TYPE dSq((a - D*d).squaredNorm());
			const TYPE srSq = SQUARE(sphere.radius);
			if (dSq <= srSq) {
				const TYPE r = d - SQRT(srSq-dSq);
				ASSERT(r >= TYPE(0));
				return r <= radius; // intersection before the ray end
			}
			return false;
		}
	};
	const VECTOR CmV(sphere.center - ray.m_pOrig);
	const TYPE t(ray.m_vDir.dot(CmV));
	// sphere projects behind the cylinder origin
	if (t < minHeight) {
		if (t+sphere.radius < minHeight)
			return false;
		return Check::Intersection(ray, sphere, CmV, t, radius, minHeight);
	}
	// sphere projects after the cylinder end
	if (t > maxHeight) {
		if (t-sphere.radius > maxHeight)
			return false;
		return Check::Intersection(ray, sphere, CmV, t, radius, maxHeight);
	}
	const TYPE lenSq((CmV - ray.m_vDir * t).squaredNorm());
	return lenSq <= SQUARE(sphere.radius + radius);
} // Intersects
/*----------------------------------------------------------------*/

// Classify point to cylinder.
template <typename TYPE, int DIMS>
GCLASS TCylinder<TYPE,DIMS>::Classify(const POINT& p, TYPE& t) const
{
	ASSERT(ISEQUAL(ray.m_vDir.norm(), TYPE(1)));
	const VECTOR D(p - ray.m_pOrig);
	t = ray.m_vDir.dot(D);
	if (t < minHeight) return BACK;
	if (t > maxHeight) return FRONT;
	const TYPE rSq((D - ray.m_vDir*t).squaredNorm());
	const TYPE radiusSq(SQUARE(radius));
	if (rSq > radiusSq) return CULLED;
	if (rSq < radiusSq) return VISIBLE;
	return PLANAR;
} // Classify
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
bool TConeIntersect<TYPE,DIMS>::operator()(const SPHERE& sphere) const
{
	const VECTOR CmV(sphere.center - cone.ray.m_pOrig);
	const VECTOR D(CmV + cone.ray.m_vDir * (sphere.radius*invSinAngle));
	TYPE e = D.dot(cone.ray.m_vDir);
	if (e <= TYPE(0) || e*e < D.squaredNorm()*cosAngleSq)
		return false;
	e = CmV.dot(cone.ray.m_vDir);
	if (e-sphere.radius > cone.maxHeight)
		return false;
	if (e < cone.minHeight) {
		const TYPE lenSq = CmV.squaredNorm();
		if (e*e >= lenSq*sinAngleSq)
			return lenSq <= SQUARE(sphere.radius);
	}
	return true;
} // Intersect
/*----------------------------------------------------------------*/

// Classify point to cone.
template <typename TYPE, int DIMS>
GCLASS TConeIntersect<TYPE,DIMS>::Classify(const POINT& p, TYPE& t) const
{
	ASSERT(ISEQUAL(cone.ray.m_vDir.norm(), TYPE(1)));
	const VECTOR D(p - cone.ray.m_pOrig);
	t = cone.ray.m_vDir.dot(D);
	if (ISZERO(t))
		return PLANAR;
	if (t < cone.minHeight) return BACK;
	if (t > cone.maxHeight) return FRONT;
	ASSERT(!ISZERO(D.norm()));
	const TYPE tSq(SQUARE(t));
	const TYPE dSq(cosAngleSq*D.squaredNorm());
	if (tSq < dSq) return CULLED;
	if (tSq > dSq) return VISIBLE;
	return PLANAR;
} // Classify
/*----------------------------------------------------------------*/
