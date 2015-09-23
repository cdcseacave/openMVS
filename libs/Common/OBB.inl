////////////////////////////////////////////////////////////////////
// OBB.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
inline TOBB<TYPE,DIMS>::TOBB(const POINT* pts, size_t n)
{
	Set(pts, n);
}
template <typename TYPE, int DIMS>
inline TOBB<TYPE,DIMS>::TOBB(const POINT* pts, size_t n, const TRIANGLE* tris, size_t s)
{
	Set(pts, n, tris, s);
} // constructor
/*----------------------------------------------------------------*/


// Inspired from "Fitting Oriented Bounding Boxes" by James Gregson
// http://jamesgregson.blogspot.ro/2011/03/latex-test.html

// build an OBB from a vector of input points.  This
// method just forms the covariance matrix and hands
// it to the build_from_covariance_matrix method
// which handles fitting the box to the points
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const POINT* pts, size_t n)
{
	ASSERT(n >= DIMS);

	// loop over the points to find the mean point
	// location and to build the covariance matrix;
	// note that we only have
	// to build terms for the upper triangular 
	// portion since the matrix is symmetric
	POINT mu(POINT::Zero());
	TYPE cxx=0, cxy=0, cxz=0, cyy=0, cyz=0, czz=0;
	for (size_t i=0; i<n; ++i) {
		const POINT& p = pts[i];
		mu += p;
		cxx += p(0)*p(0); 
		cxy += p(0)*p(1); 
		cxz += p(0)*p(2);
		cyy += p(1)*p(1);
		cyz += p(1)*p(2);
		czz += p(2)*p(2);
	}
	const TYPE invN(TYPE(1)/TYPE(n));
	cxx = (cxx - mu(0)*mu(0)*invN)*invN; 
	cxy = (cxy - mu(0)*mu(1)*invN)*invN; 
	cxz = (cxz - mu(0)*mu(2)*invN)*invN;
	cyy = (cyy - mu(1)*mu(1)*invN)*invN;
	cyz = (cyz - mu(1)*mu(2)*invN)*invN;
	czz = (czz - mu(2)*mu(2)*invN)*invN;

	// now build the covariance matrix
	MATRIX C;
	C(0,0) = cxx; C(0,1) = cxy; C(0,2) = cxz;
	C(1,0) = cxy; C(1,1) = cyy; C(1,2) = cyz;
	C(2,0) = cxz; C(2,1) = cyz; C(2,2) = czz;

	// set the OBB parameters from the covariance matrix
	Set(C, pts, n);
}
// builds an OBB from triangles specified as an array of
// points with integer indices into the point array. Forms
// the covariance matrix for the triangles, then uses the
// method build_from_covariance_matrix method to fit 
// the box.  ALL points will be fit in the box, regardless
// of whether they are indexed by a triangle or not.
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const POINT* pts, size_t n, const TRIANGLE* tris, size_t s)
{
	ASSERT(n >= DIMS);

	// loop over the triangles this time to find the
	// mean location
	POINT mu(POINT::Zero());
	TYPE Am=0;
	TYPE cxx=0, cxy=0, cxz=0, cyy=0, cyz=0, czz=0;
	for (size_t i=0; i<s; ++i) {
		ASSERT(tris[i](0)<n && tris[i](1)<n && tris[i](2)<n);
		const POINT& p = pts[tris[i](0)];
		const POINT& q = pts[tris[i](1)];
		const POINT& r = pts[tris[i](2)];
		const POINT mui = (p+q+r)/TYPE(3);
		const TYPE Ai = (q-p).cross(r-p).normalize()/TYPE(2);
		mu += mui*Ai;
		Am += Ai;

		// these bits set the c terms to Am*E[xx], Am*E[xy], Am*E[xz]....
		const TYPE Ai12 = Ai/TYPE(12);
		cxx += (TYPE(9)*mui(0)*mui(0) + p(0)*p(0) + q(0)*q(0) + r(0)*r(0))*Ai12;
		cxy += (TYPE(9)*mui(0)*mui(1) + p(0)*p(1) + q(0)*q(1) + r(0)*r(1))*Ai12;
		cxz += (TYPE(9)*mui(0)*mui(2) + p(0)*p(2) + q(0)*q(2) + r(0)*r(2))*Ai12;
		cyy += (TYPE(9)*mui(1)*mui(1) + p(1)*p(1) + q(1)*q(1) + r(1)*r(1))*Ai12;
		cyz += (TYPE(9)*mui(1)*mui(2) + p(1)*p(2) + q(1)*q(2) + r(1)*r(2))*Ai12;
		czz += (TYPE(9)*mui(2)*mui(2) + p(2)*p(2) + q(2)*q(2) + r(2)*r(2))*Ai12;
	}

	// divide out the Am fraction from the average position and 
	// covariance terms
	mu /= Am;
	cxx /= Am; cxy /= Am; cxz /= Am; cyy /= Am; cyz /= Am; czz /= Am;

	// now subtract off the E[x]*E[x], E[x]*E[y], ... terms
	cxx -= mu(0)*mu(0); cxy -= mu(0)*mu(1); cxz -= mu(0)*mu(2);
	cyy -= mu(1)*mu(1); cyz -= mu(1)*mu(2); czz -= mu(2)*mu(2);

	// now build the covariance matrix
	MATRIX C;
	C(0,0)=cxx; C(0,1)=cxy; C(0,2)=cxz;
	C(1,0)=cxy; C(1,1)=cyy; C(1,2)=cyz;
	C(2,0)=cxz; C(1,2)=cyz; C(2,2)=czz;

	// set the obb parameters from the covariance matrix
	Set(C, pts, n);
}
// method to set the OBB parameters which produce a box oriented according to
// the covariance matrix C, and that contains the given points
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const MATRIX& C, const POINT* pts, size_t n)
{
	// extract rotation from the covariance matrix
	SetRotation(C);
	// extract size and center from the given points
	SetBounds(pts, n);
}
// method to set the OBB rotation which produce a box oriented according to
// the covariance matrix C (only the rotations is set)
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::SetRotation(const MATRIX& C)
{
	// extract the eigenvalues and eigenvectors from C
	const Eigen::EigenSolver<MATRIX> es(C);
	if (es.info() != Eigen::Success)
		return;
	const MATRIX eigvec(es.eigenvectors().real());
	const POINT eigval(es.eigenvalues().real());
	int indices[3] = {0,1,2};
	std::sort(indices, indices+3, [&] (int i, int j) {
		return eigval(i) < eigval(j);
	});

	// find the right, up and forward vectors from the eigenvectors
	// and set the rotation matrix using the eigenvectors
	m_rot.row(0) = eigvec.col(indices[0]);
	m_rot.row(1) = eigvec.col(indices[1]);
	m_rot.row(2) = eigvec.col(indices[2]);
	ASSERT(ISEQUAL(m_rot.row(0).norm(), TYPE(1)));
	ASSERT(ISEQUAL(m_rot.row(1).norm(), TYPE(1)));
	ASSERT(ISEQUAL(m_rot.row(2).norm(), TYPE(1)));
	if (m_rot.determinant() < 0)
		m_rot = -m_rot;
}
// method to set the OBB center and size that contains the given points
// the rotations should be already set
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::SetBounds(const POINT* pts, size_t n)
{
	ASSERT(n >= DIMS);
	ASSERT(ISEQUAL((m_rot*m_rot.transpose()).trace(), TYPE(3)) && ISEQUAL(m_rot.determinant(), TYPE(1)));

	// build the bounding box extents in the rotated frame
	const TYPE tmax = std::numeric_limits<TYPE>::max();
	POINT minim(tmax, tmax, tmax), maxim(-tmax, -tmax, -tmax);
	for (size_t i=0; i<n; ++i) {
		const POINT p_prime(m_rot * pts[i]);
		if (minim(0) > p_prime(0)) minim(0) = p_prime(0);
		if (minim(1) > p_prime(1)) minim(1) = p_prime(1);
		if (minim(2) > p_prime(2)) minim(2) = p_prime(2);
		if (maxim(0) < p_prime(0)) maxim(0) = p_prime(0);
		if (maxim(1) < p_prime(1)) maxim(1) = p_prime(1);
		if (maxim(2) < p_prime(2)) maxim(2) = p_prime(2);
	}

	// set the center of the OBB to be the average of the 
	// minimum and maximum, and the extents be half of the
	// difference between the minimum and maximum
	const POINT center((maxim+minim)*TYPE(0.5));
	m_pos = m_rot.transpose() * center;
	m_ext = (maxim-minim)*TYPE(0.5);
} // Set
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::BuildBegin()
{
	m_rot = MATRIX::Zero();
	m_pos = POINT::Zero();
	m_ext = POINT::Zero();
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::BuildAdd(const POINT& p)
{
	// store mean in m_pos
	m_pos += p;
	// store covariance params in m_rot
	m_rot(0,0) += p(0)*p(0); 
	m_rot(0,1) += p(0)*p(1); 
	m_rot(0,2) += p(0)*p(2);
	m_rot(1,0) += p(1)*p(1);
	m_rot(1,1) += p(1)*p(2);
	m_rot(1,2) += p(2)*p(2);
	// store count in m_ext
	++(*((size_t*)m_ext.data()));
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::BuildEnd()
{
	const TYPE invN(TYPE(1)/TYPE(*((size_t*)m_ext.data())));
	const TYPE cxx = (m_rot(0,0) - m_pos(0)*m_pos(0)*invN)*invN; 
	const TYPE cxy = (m_rot(0,1) - m_pos(0)*m_pos(1)*invN)*invN; 
	const TYPE cxz = (m_rot(0,2) - m_pos(0)*m_pos(2)*invN)*invN;
	const TYPE cyy = (m_rot(1,0) - m_pos(1)*m_pos(1)*invN)*invN;
	const TYPE cyz = (m_rot(1,1) - m_pos(1)*m_pos(2)*invN)*invN;
	const TYPE czz = (m_rot(1,2) - m_pos(2)*m_pos(2)*invN)*invN;

	// now build the covariance matrix
	MATRIX C;
	C(0,0) = cxx; C(0,1) = cxy; C(0,2) = cxz;
	C(1,0) = cxy; C(1,1) = cyy; C(1,2) = cyz;
	C(2,0) = cxz; C(2,1) = cyz; C(2,2) = czz;
	SetRotation(C);
} // Build
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Enlarge(TYPE x)
{
	m_ext.array() -= x;
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::EnlargePercent(TYPE x)
{
	m_ext *= x;
} // Enlarge
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline typename TOBB<TYPE,DIMS>::POINT TOBB<TYPE,DIMS>::GetCenter() const
{
	return m_pos;
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::GetCenter(POINT& ptCenter) const
{
	ptCenter = m_pos;
} // GetCenter
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline typename TOBB<TYPE,DIMS>::POINT TOBB<TYPE,DIMS>::GetSize() const
{
	return m_ext*2;
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::GetSize(POINT& ptSize) const
{
	ptSize = m_ext*2;
} // GetSize
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::GetCorners(POINT pts[numCorners]) const
{
	if (DIMS == 2) {
		const POINT pEAxis[2] = {
			m_rot.row(0)*m_ext[0],
			m_rot.row(1)*m_ext[1]
		};
		pts[0] = m_pos - pEAxis[0] - pEAxis[1];
		pts[1] = m_pos + pEAxis[0] - pEAxis[1];
		pts[2] = m_pos + pEAxis[0] + pEAxis[1];
		pts[3] = m_pos - pEAxis[0] + pEAxis[1];
	}
	if (DIMS == 3) {
		const POINT pEAxis[3] = {
			m_rot.row(0)*m_ext[0],
			m_rot.row(1)*m_ext[1],
			m_rot.row(2)*m_ext[2]
		};
		pts[0] = m_pos - pEAxis[0] - pEAxis[1] - pEAxis[2];
		pts[1] = m_pos + pEAxis[0] - pEAxis[1] - pEAxis[2];
		pts[2] = m_pos + pEAxis[0] + pEAxis[1] - pEAxis[2];
		pts[3] = m_pos - pEAxis[0] + pEAxis[1] - pEAxis[2];
		pts[4] = m_pos - pEAxis[0] - pEAxis[1] + pEAxis[2];
		pts[5] = m_pos + pEAxis[0] - pEAxis[1] + pEAxis[2];
		pts[6] = m_pos + pEAxis[0] + pEAxis[1] + pEAxis[2];
		pts[7] = m_pos - pEAxis[0] + pEAxis[1] + pEAxis[2];
	}
} // GetCorners
// constructs the corner of the aligned bounding box in world space
template <typename TYPE, int DIMS>
inline typename TOBB<TYPE,DIMS>::AABB TOBB<TYPE,DIMS>::GetAABB() const
{
	if (DIMS == 2) {
		const POINT pEAxis[2] = {
			m_rot.row(0)*m_ext[0],
			m_rot.row(1)*m_ext[1]
		};
		return AABB(
			m_pos - pEAxis[0] - pEAxis[1],
			m_pos + pEAxis[0] + pEAxis[1]
		);
	}
	if (DIMS == 3) {
		const POINT pEAxis[3] = {
			m_rot.row(0)*m_ext[0],
			m_rot.row(1)*m_ext[1],
			m_rot.row(2)*m_ext[2]
		};
		return AABB(
			m_pos - pEAxis[0] - pEAxis[1] - pEAxis[2],
			m_pos + pEAxis[0] + pEAxis[1] + pEAxis[2]
		);
	}
} // GetAABB
/*----------------------------------------------------------------*/

// computes the volume of the OBB, which is a measure of
// how tight the fit is (better OBBs will have smaller volumes)
template <typename TYPE, int DIMS>
inline TYPE TOBB<TYPE,DIMS>::GetVolume() const
{
	return m_ext.prod()*numCorners;
}
/*----------------------------------------------------------------*/


// Update the box by the given pos delta.
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Translate(const POINT& d)
{
	m_pos += d;
}
/*----------------------------------------------------------------*/


// Update the box by the given transform.
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Transform(const MATRIX& m)
{
	m_rot = m * m_rot;
	m_pos = m * m_pos;
}
/*----------------------------------------------------------------*/
