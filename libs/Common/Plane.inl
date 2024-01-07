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
}
// Construct plane given its standard equation: Ax + By + Cz + D = 0
template <typename TYPE, int DIMS>
inline TPlane<TYPE,DIMS>::TPlane(const Eigen::Matrix<TYPE,DIMS+1,1>& v)
{
	Set(v);
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
	Set(vN*invD, p[DIMS]*invD);
}
template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Set(const Eigen::Matrix<TYPE,DIMS+1,1>& v)
{
	const VECTOR vN = v.template topLeftCorner<3,1>();
	const TYPE invD(INVERT(vN.norm()));
	Set(vN*invD, v(DIMS)*invD);
} // Set
/*----------------------------------------------------------------*/


// least squares refinement of the given plane to the 3D point set
// (return the number of iterations)
template <typename TYPE, int DIMS>
template <typename RobustNormFunctor>
int TPlane<TYPE,DIMS>::Optimize(const POINT* points, size_t size, const RobustNormFunctor& robust, int maxIters)
{
	ASSERT(DIMS == 3);
	ASSERT(size >= numParams);
	struct OptimizationFunctor {
		const POINT* points;
		const size_t size;
		const RobustNormFunctor& robust;
		// construct with the data points
		OptimizationFunctor(const POINT* _points, size_t _size, const RobustNormFunctor& _robust)
			: points(_points), size(_size), robust(_robust) { ASSERT(size < (size_t)std::numeric_limits<int>::max()); }
		static void Residuals(const double* x, int nPoints, const void* pData, double* fvec, double* fjac, int* /*info*/) {
			const OptimizationFunctor& data = *reinterpret_cast<const OptimizationFunctor*>(pData);
			ASSERT((size_t)nPoints == data.size && fvec != NULL && fjac == NULL);
			TPlane<double,3> plane; {
				Point3d N;
				plane.m_fD = x[0];
				Dir2Normal(reinterpret_cast<const Point2d&>(x[1]), N);
				plane.m_vN = N;
			}
			for (size_t i=0; i<data.size; ++i)
				fvec[i] = data.robust(plane.Distance(data.points[i].template cast<double>()));
		}
	} functor(points, size, robust);
	double arrParams[numParams]; {
		arrParams[0] = (double)m_fD;
		const Point3d N(m_vN.x(), m_vN.y(), m_vN.z());
		Normal2Dir(N, reinterpret_cast<Point2d&>(arrParams[1]));
	}
	lm_control_struct control = {1.e-6, 1.e-7, 1.e-8, 1.e-7, 100.0, maxIters}; // lm_control_float;
	lm_status_struct status;
	lmmin(numParams, arrParams, (int)size, &functor, OptimizationFunctor::Residuals, &control, &status);
	switch (status.info) {
	//case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
		DEBUG_ULTIMATE("error: refine plane: %s", lm_infmsg[status.info]);
		return 0;
	}
	// set plane
	{
		Point3d N;
		Dir2Normal(reinterpret_cast<const Point2d&>(arrParams[1]), N);
		Set(Cast<TYPE>(N), (TYPE)arrParams[0]);
	}
	return status.nfev;
}
template <typename TYPE, int DIMS>
int TPlane<TYPE,DIMS>::Optimize(const POINT* points, size_t size, int maxIters)
{
	const auto identity = [](double x) { return x; };
	return Optimize(points, size, identity, maxIters);
} // Optimize
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Invalidate()
{
	m_fD = std::numeric_limits<TYPE>::max();
} // Invalidate
template <typename TYPE, int DIMS>
inline bool TPlane<TYPE,DIMS>::IsValid() const
{
	return m_fD != std::numeric_limits<TYPE>::max();
} // IsValid
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TPlane<TYPE,DIMS>::Negate()
{
	m_vN = -m_vN;
	m_fD = -m_fD;
} // Negate
template <typename TYPE, int DIMS>
inline TPlane<TYPE,DIMS> TPlane<TYPE,DIMS>::Negated() const
{
	return TPlane(-m_vN, -m_fD);
} // Negated
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


// Calculate point's projection on this plane (closest point to this plane).
template <typename TYPE, int DIMS>
inline typename TPlane<TYPE,DIMS>::POINT TPlane<TYPE,DIMS>::ProjectPoint(const POINT& p) const
{
	return p - m_vN*Distance(p);
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


// same as above, but online version
template <typename TYPE, typename TYPEW, bool bFitLineMode>
FitPlaneOnline<TYPE,TYPEW,bFitLineMode>::FitPlaneOnline()
	: sumX(0), sumSqX(0), sumXY(0), sumXZ(0), sumY(0), sumSqY(0), sumYZ(0), sumZ(0), sumSqZ(0), size(0)
{
}
template <typename TYPE, typename TYPEW, bool bFitLineMode>
void FitPlaneOnline<TYPE,TYPEW,bFitLineMode>::Update(const TPoint3<TYPE>& P)
{
	const TYPEW X((TYPEW)P.x), Y((TYPEW)P.y), Z((TYPEW)P.z);
	sumX += X; sumSqX += X*X; sumXY += X*Y; sumXZ += X*Z;
	sumY += Y; sumSqY += Y*Y; sumYZ += Y*Z;
	sumZ += Z; sumSqZ += Z*Z;
	++size;
}
template <typename TYPE, typename TYPEW, bool bFitLineMode>
TPoint3<TYPEW> FitPlaneOnline<TYPE,TYPEW,bFitLineMode>::GetModel(TPoint3<TYPEW>& avg, TPoint3<TYPEW>& dir) const
{
	const TYPEW avgX(sumX/(TYPEW)size), avgY(sumY/(TYPEW)size), avgZ(sumZ/(TYPEW)size);
	// assemble covariance (lower-triangular) matrix
	typedef Eigen::Matrix<TYPEW,3,3> Mat3x3;
	Mat3x3 A;
	A(0,0) = sumSqX - TYPEW(2)*sumX*avgX + avgX*avgX*(TYPEW)size;
	A(1,0) = sumXY - sumX*avgY - avgX*sumY + avgX*avgY*(TYPEW)size;
	A(1,1) = sumSqY - TYPEW(2)*sumY*avgY + avgY*avgY*(TYPEW)size;
	A(2,0) = sumXZ - sumX*avgZ - avgX*sumZ + avgX*avgZ*(TYPEW)size;
	A(2,1) = sumYZ - sumY*avgZ - avgY*sumZ + avgY*avgZ*(TYPEW)size;
	A(2,2) = sumSqZ - TYPEW(2)*sumZ*avgZ + avgZ*avgZ*(TYPEW)size;
	// the plane normal is simply the eigenvector corresponding to least eigenvalue
	const int nAxis(bFitLineMode ? 2 : 0);
	const Eigen::SelfAdjointEigenSolver<Mat3x3> es(A);
	ASSERT(ISEQUAL(es.eigenvectors().col(nAxis).norm(), TYPEW(1)));
	avg = TPoint3<TYPEW>(avgX,avgY,avgZ);
	dir = es.eigenvectors().col(nAxis);
	const TYPEW* const vals(es.eigenvalues().data());
	ASSERT(vals[0] <= vals[1] && vals[1] <= vals[2]);
	return *reinterpret_cast<const TPoint3<TYPEW>*>(vals);
}
template <typename TYPE, typename TYPEW, bool bFitLineMode>
template <typename TYPEE>
TPoint3<TYPEE> FitPlaneOnline<TYPE,TYPEW,bFitLineMode>::GetPlane(TPlane<TYPEE,3>& plane) const
{
	TPoint3<TYPEW> avg, dir;
	const TPoint3<TYPEW> quality(GetModel(avg, dir));
	plane.Set(TPoint3<TYPEE>(dir), TPoint3<TYPEE>(avg));
	return TPoint3<TYPEE>(quality);
}
/*----------------------------------------------------------------*/


// Least squares fits a plane to a 3D point set.
// See http://www.geometrictools.com/Documentation/LeastSquaresFitting.pdf
// Returns a fitting quality (1 - lambda_min/lambda_max):
//  1 is best (zero variance orthogonally to the fitting line)
//  0 is worst (isotropic case, returns a plane with default direction)
template <typename TYPE>
TYPE FitPlane(const TPoint3<TYPE>* points, size_t size, TPlane<TYPE>& plane) {
	// compute a point on the plane, which is shown to be the centroid of the points
	const Eigen::Map< const Eigen::Matrix<TYPE,Eigen::Dynamic,3,Eigen::RowMajor> > vPoints((const TYPE*)points, size, 3);
	const TPoint3<TYPE> c(vPoints.colwise().mean());

	// assemble covariance matrix; matrix numbering:
	// 0          
	// 1 2
	// 3 4 5          
	Eigen::Matrix<TYPE,3,3,Eigen::RowMajor> A(Eigen::Matrix<TYPE,3,3,Eigen::RowMajor>::Zero());
	FOREACHRAWPTR(pPt, points, size) {
		const TPoint3<TYPE> X(*pPt - c);
		A(0,0) += X.x*X.x;
		A(1,0) += X.x*X.y;
		A(1,1) += X.y*X.y;
		A(2,0) += X.x*X.z;
		A(2,1) += X.y*X.z;
		A(2,2) += X.z*X.z;
	}

	// the plane normal is simply the eigenvector corresponding to least eigenvalue
	const Eigen::SelfAdjointEigenSolver< Eigen::Matrix<TYPE,3,3,Eigen::RowMajor> > es(A);
	ASSERT(ISEQUAL(es.eigenvectors().col(0).norm(), TYPE(1)));
	plane.Set(es.eigenvectors().col(0), c);
	const TYPE* const vals(es.eigenvalues().data());
	ASSERT(vals[0] <= vals[1] && vals[1] <= vals[2]);
	return TYPE(1) - vals[0]/vals[1];
}
/*----------------------------------------------------------------*/


// C L A S S  //////////////////////////////////////////////////////

// Construct frustum given a projection matrix.
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


// Set frustum planes, normals pointing outwards, from SfM projection matrix and image plane details
template <typename TYPE, int DIMS>
void TFrustum<TYPE,DIMS>::Set(const MATRIX4x4& m, TYPE w, TYPE h, TYPE n, TYPE f)
{
	const VECTOR ltn(0,0,n), rtn(w*n,0,n), lbn(0,h*n,n), rbn(w*n,h*n,n);
	const VECTOR ltf(0,0,f), rtf(w*f,0,f), lbf(0,h*f,f), rbf(w*f,h*f,f);
	const MATRIX4x4 inv(m.inverse());
	const VECTOR corners[] = {
		(inv*ltn.homogeneous()).template topRows<3>(),
		(inv*rtn.homogeneous()).template topRows<3>(),
		(inv*lbn.homogeneous()).template topRows<3>(),
		(inv*rbn.homogeneous()).template topRows<3>(),
		(inv*ltf.homogeneous()).template topRows<3>(),
		(inv*rtf.homogeneous()).template topRows<3>(),
		(inv*lbf.homogeneous()).template topRows<3>(),
		(inv*rbf.homogeneous()).template topRows<3>()
	};
	Set(corners);
}
template <typename TYPE, int DIMS>
void TFrustum<TYPE,DIMS>::Set(const MATRIX3x4& m, TYPE w, TYPE h, TYPE n, TYPE f)
{
	MATRIX4x4 M(MATRIX4x4::Identity());
	M.template topLeftCorner<3,4>() = m;
	Set(M, w, h, n, f);
}
// Set frustum planes, normals pointing outwards, from the given corners
template <typename TYPE, int DIMS>
void TFrustum<TYPE,DIMS>::Set(const VECTOR corners[numCorners])
{
	// left clipping plane
	m_planes[0].Set(corners[0], corners[4], corners[6]);
	if (DIMS > 1) // right clipping plane
	m_planes[1].Set(corners[1], corners[7], corners[5]);
	if (DIMS > 2) // top clipping plane
	m_planes[2].Set(corners[0], corners[5], corners[4]);
	if (DIMS > 3) // bottom clipping plane
	m_planes[3].Set(corners[2], corners[6], corners[7]);
	if (DIMS > 4) // near clipping plane
	m_planes[4].Set(corners[0], corners[2], corners[3]);
	if (DIMS > 5) // far clipping plane
	m_planes[5].Set(corners[4], corners[5], corners[7]);
} // Set
// Set frustum planes, normals pointing outwards, from the OpenGL projection-view matrix
template <typename TYPE, int DIMS>
void TFrustum<TYPE,DIMS>::SetProjectionGL(const MATRIX4x4& mProjectionView)
{
	// left clipping plane
	m_planes[0].Set(-mProjectionView.row(3) - mProjectionView.row(0));
	if (DIMS > 1) // right clipping plane
	m_planes[1].Set(-mProjectionView.row(3) + mProjectionView.row(0));
	if (DIMS > 2) // top clipping plane
	m_planes[2].Set(-mProjectionView.row(3) + mProjectionView.row(1));
	if (DIMS > 3) // bottom clipping plane
	m_planes[3].Set(-mProjectionView.row(3) - mProjectionView.row(1));
	if (DIMS > 4) // near clipping plane
	m_planes[4].Set(-mProjectionView.row(3) - mProjectionView.row(2));
	if (DIMS > 5) // far clipping plane
	m_planes[5].Set(-mProjectionView.row(3) + mProjectionView.row(2));
}
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
