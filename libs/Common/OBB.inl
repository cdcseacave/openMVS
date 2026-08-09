////////////////////////////////////////////////////////////////////
// OBB.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
inline TOBB<TYPE,DIMS>::TOBB(bool)
	:
	m_rot(MATRIX::Identity()),
	m_pos(POINT::Zero()),
	m_ext(POINT::Zero())
{
}
template <typename TYPE, int DIMS>
inline TOBB<TYPE,DIMS>::TOBB(const AABB& aabb)
{
	Set(aabb);
}
template <typename TYPE, int DIMS>
inline TOBB<TYPE,DIMS>::TOBB(const MATRIX& rot, const POINT& ptMin, const POINT& ptMax)
{
	Set(rot, ptMin, ptMax);
}
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
template <typename TYPE, int DIMS>
template <typename CTYPE>
inline TOBB<TYPE,DIMS>::TOBB(const TOBB<CTYPE,DIMS>& rhs)
	:
	m_rot(rhs.m_rot.template cast<TYPE>()),
	m_pos(rhs.m_pos.template cast<TYPE>()),
	m_ext(rhs.m_ext.template cast<TYPE>())
{
} // copy constructor
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Reset()
{
	m_rot.setIdentity();
	m_pos = POINT::Zero();
	m_ext = POINT::Zero();
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const AABB& aabb)
{
	m_rot.setIdentity();
	m_pos = aabb.GetCenter();
	m_ext = aabb.GetSize()/TYPE(2);
}

// build from rotation matrix from world to local, and local min/max corners
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const MATRIX& rot, const POINT& ptMin, const POINT& ptMax)
{
	m_rot = rot;
	m_pos = (ptMax + ptMin) * TYPE(0.5);
	m_ext = (ptMax - ptMin) * TYPE(0.5);
}

// Inspired from "Fitting Oriented Bounding Boxes" by James Gregson
// http://jamesgregson.blogspot.ro/2011/03/latex-test.html

// Build an OBB from a vector of input points.  This
// method just forms the covariance matrix and hands
// it to the build_from_covariance_matrix method
// which handles fitting the box to the points.
//
// If k (number of nearest neighbors) is set, the method will filter
// out inside points and use only the surface points. This is useful
// when the dominant direction of the inside points is not aligned with
// the convex hull which ultimately is used to define the OBB dimensions.
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const POINT* pts, size_t n, int k, int fixedAxis)
{
	ASSERT(n >= DIMS);

	std::vector<POINT> surfacePoints;
	if (k > 0) {
		// Filter surface points based on the k nearest neighbors
		surfacePoints = FilterSurfacePoints(pts, n, k);
		pts = surfacePoints.data();
		n = surfacePoints.size();
	}

	// loop over the points to find the mean point location
	// and to accumulate the second moments
	POINT mu(POINT::Zero());
	MATRIX C(MATRIX::Zero());
	for (size_t i=0; i<n; ++i) {
		const POINT& p = pts[i];
		mu += p;
		C += p * p.transpose();
	}
	// now build the covariance matrix
	const TYPE invN(TYPE(1)/TYPE(n));
	C = (C - mu*mu.transpose()*invN)*invN;

	// set the OBB parameters from the covariance matrix
	Set(C, pts, n, fixedAxis);
}
// Build an OBB from a vector of input points with the last local axis
// aligned to the given up direction; the remaining axes are the ones
// minimizing the footprint of the box: the minimum-area rectangle of
// the points projected on the hyperplane perpendicular to up, computed
// exactly over their convex hull (one side of the optimal rectangle is
// always collinear with a hull edge). Unlike the covariance based fit,
// the resulting orientation is independent of the point density
// distribution.
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const POINT* pts, size_t n, const POINT& up)
{
	ASSERT(n >= DIMS);
	SetRotation(up, pts, n);
	SetBounds(pts, n);
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
	STATIC_ASSERT(DIMS == 3); // a triangle is only defined by three indices in 3D
	ASSERT(n >= DIMS);

	// loop over the triangles this time to find the
	// mean location and accumulate the area weighted second moments
	POINT mu(POINT::Zero());
	TYPE Am=0;
	MATRIX C(MATRIX::Zero());
	for (size_t i=0; i<s; ++i) {
		ASSERT(tris[i](0)<n && tris[i](1)<n && tris[i](2)<n);
		const POINT& p = pts[tris[i](0)];
		const POINT& q = pts[tris[i](1)];
		const POINT& r = pts[tris[i](2)];
		const POINT mui = (p+q+r)/TYPE(3);
		const TYPE Ai = (q-p).cross(r-p).norm()/TYPE(2);
		mu += mui*Ai;
		Am += Ai;

		// this sets the C terms to Am*E[xx], Am*E[xy], Am*E[xz]....
		const TYPE Ai12 = Ai/TYPE(12);
		C += (TYPE(9)*mui*mui.transpose() +
			  p*p.transpose() + q*q.transpose() + r*r.transpose())*Ai12;
	}

	// divide out the Am fraction from the average position and covariance
	// terms, then subtract off the E[x]*E[x], E[x]*E[y], ... terms
	mu /= Am;
	C = C/Am - mu*mu.transpose();

	// set the obb parameters from the covariance matrix
	Set(C, pts, n);
}
// method to set the OBB parameters which produce a box oriented according to
// the covariance matrix C, and that contains the given points
// if fixedAxis is specified, that axis is kept on the world basis and the OBB rotation
// is applied only in the hyperplane perpendicular to it (0=x,1=y,2=z)
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Set(const MATRIX& C, const POINT* pts, size_t n, int fixedAxis)
{
	// extract rotation from the covariance matrix
	if (fixedAxis >= 0)
		SetRotation(C, fixedAxis);
	else
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
	const Eigen::SelfAdjointEigenSolver<MATRIX> es(C);
	ASSERT(es.info() == Eigen::Success);
	// find the right, up and forward vectors from the eigenvectors
	// and set the rotation matrix using the eigenvectors;
	// eigenvalues are sorted ascending, possibly equal for degenerate (isotropic/planar) inputs
	ASSERT(std::is_sorted(es.eigenvalues().data(), es.eigenvalues().data()+DIMS));
	m_rot = es.eigenvectors().transpose();
	if (m_rot.determinant() < 0)
		m_rot = -m_rot;
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::SetRotation(const MATRIX& C, int fixedAxis)
{
	STATIC_ASSERT(DIMS > 1);
	ASSERT(fixedAxis >= 0 && fixedAxis < DIMS);
	// the free axes, in wrap-around order after the fixed one
	enum {DIMSF = DIMS-1};
	int freeAxes[DIMS];
	for (int i=0; i<DIMSF; ++i)
		freeAxes[i] = (fixedAxis + 1 + i) % DIMS;
	// covariance sub-matrix of the free axes
	typedef Eigen::Matrix<TYPE,DIMSF,DIMSF> MATRIXF;
	MATRIXF Cf;
	for (int i=0; i<DIMSF; ++i)
		for (int j=0; j<DIMSF; ++j)
			Cf(i,j) = C(freeAxes[i], freeAxes[j]);
	const Eigen::SelfAdjointEigenSolver<MATRIXF> es(Cf);
	ASSERT(es.info() == Eigen::Success);
	// build the rotation rows (rows = axes): the fixed axis aligns with the world
	// basis, the free axes take the eigenvectors ordered minor -> major
	m_rot.setZero();
	m_rot(fixedAxis, fixedAxis) = TYPE(1);
	for (int i=0; i<DIMSF; ++i)
		for (int j=0; j<DIMSF; ++j)
			m_rot(freeAxes[i], freeAxes[j]) = es.eigenvectors()(j,i);
	// make right-handed: flip the minor row if needed
	if (m_rot.determinant() < TYPE(0))
		m_rot.row(freeAxes[0]) = -m_rot.row(freeAxes[0]);
}
// method to set the OBB rotation such that the last local axis aligns with the given
// up direction, while the remaining axes minimize the box footprint measured in the
// hyperplane perpendicular to it; only a 2D hyperplane (3D box) leaves any orientation
// freedom, resolved exactly by the minimum-area rectangle of the projected points
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::SetRotation(const POINT& upDirection, const POINT* pts, size_t n)
{
	STATIC_ASSERT(DIMS > 1);
	ASSERT(n > 0);
	enum {DIMSP = DIMS-1}; // dimension of the hyperplane perpendicular to up
	typedef Eigen::Matrix<TYPE,DIMSP,1> POINTP;
	typedef Eigen::Matrix<TYPE,DIMSP,DIMSP> MATRIXP;
	const POINT up(upDirection.normalized());
	// orthonormal basis of the hyperplane perpendicular to up: the Householder QR of
	// up returns an orthogonal matrix having up as first column, so its trailing
	// columns span the searched hyperplane
	const Eigen::Matrix<TYPE,DIMS,DIMS> basis(Eigen::HouseholderQR<POINT>(up).householderQ());
	const Eigen::Matrix<TYPE,DIMS,DIMSP> basisP(basis.template rightCols<DIMSP>());
	// find the hyperplane orientation minimizing the footprint
	MATRIXP rotP(MATRIXP::Identity());
	if constexpr (DIMSP == 2) {
		// project the points on the hyperplane
		std::vector<POINTP> ptsP(n);
		for (size_t i=0; i<n; ++i)
			ptsP[i] = basisP.transpose()*pts[i];
		const auto cross2 = [](const POINTP& o, const POINTP& p, const POINTP& q) {
			return (p(0)-o(0))*(q(1)-o(1)) - (p(1)-o(1))*(q(0)-o(0));
		};
		// Akl-Toussaint acceleration: points strictly inside the quadrilateral of the four
		// coordinate-extreme points cannot be hull vertices, so discard them before sorting;
		// a degenerate quadrilateral (collinear/coincident extremes) discards nothing as no
		// point is then strictly inside all four edges
		if (ptsP.size() > 64) {
			size_t iMinX(0), iMaxX(0), iMinY(0), iMaxY(0);
			for (size_t i=1; i<ptsP.size(); ++i) {
				if (ptsP[i](0) < ptsP[iMinX](0)) iMinX = i;
				if (ptsP[i](0) > ptsP[iMaxX](0)) iMaxX = i;
				if (ptsP[i](1) < ptsP[iMinY](1)) iMinY = i;
				if (ptsP[i](1) > ptsP[iMaxY](1)) iMaxY = i;
			}
			const POINTP quad[4] = {ptsP[iMinX], ptsP[iMinY], ptsP[iMaxX], ptsP[iMaxY]}; // CCW
			std::vector<POINTP> border;
			border.reserve(ptsP.size());
			for (const POINTP& p: ptsP) {
				bool inside = true;
				for (int k=0; k<4; ++k) {
					if (cross2(quad[k], quad[(k+1)%4], p) <= TYPE(0)) {
						inside = false;
						break;
					}
				}
				if (!inside)
					border.push_back(p);
			}
			ptsP = std::move(border);
		}
		// compute the 2D convex hull (Andrew's monotone chain)
		std::sort(ptsP.begin(), ptsP.end(), [](const POINTP& l, const POINTP& r) {
			return l(0) < r(0) || (l(0) == r(0) && l(1) < r(1));
		});
		ptsP.erase(std::unique(ptsP.begin(), ptsP.end(), [](const POINTP& l, const POINTP& r) {
			return l(0) == r(0) && l(1) == r(1);
		}), ptsP.end());
		std::vector<POINTP> hull;
		if (ptsP.size() >= 3) {
			hull.resize(2*ptsP.size());
			size_t h = 0;
			for (size_t i = 0; i < ptsP.size(); ++i) {
				while (h >= 2 && cross2(hull[h-2], hull[h-1], ptsP[i]) <= TYPE(0))
					--h;
				hull[h++] = ptsP[i];
			}
			for (size_t i = ptsP.size()-1, t = h+1; i > 0; --i) {
				while (h >= t && cross2(hull[h-2], hull[h-1], ptsP[i-1]) <= TYPE(0))
					--h;
				hull[h++] = ptsP[i-1];
			}
			hull.resize(h-1);
		}
		// find the in-plane direction minimizing the rectangle area:
		// evaluate the extents for each hull edge direction
		POINTP bestDir(1, 0);
		if (hull.size() < 3) {
			// collinear projections: the minimum-area rectangle degenerates to the
			// segment itself, so align with the line instead of the arbitrary basis x-axis
			const std::vector<POINTP>& seg(hull.size() == 2 ? hull : ptsP);
			if (seg.size() >= 2) {
				const POINTP e(seg.back() - seg.front());
				const TYPE len(e.norm());
				if (len > TYPE(0))
					bestDir = e/len;
			}
		} else {
			TYPE bestArea = std::numeric_limits<TYPE>::max();
			for (size_t i = 0; i < hull.size(); ++i) {
				const POINTP e(hull[(i+1)%hull.size()] - hull[i]);
				const TYPE len(e.norm());
				if (len <= TYPE(0))
					continue;
				const POINTP d(e/len);
				TYPE minD(std::numeric_limits<TYPE>::max()), maxD(std::numeric_limits<TYPE>::lowest());
				TYPE minP(std::numeric_limits<TYPE>::max()), maxP(std::numeric_limits<TYPE>::lowest());
				for (const POINTP& v: hull) {
					const TYPE pd( d(0)*v(0) + d(1)*v(1));
					const TYPE pp(-d(1)*v(0) + d(0)*v(1));
					if (pd < minD) minD = pd;
					if (pd > maxD) maxD = pd;
					if (pp < minP) minP = pp;
					if (pp > maxP) maxP = pp;
				}
				const TYPE area((maxD-minD)*(maxP-minP));
				if (area < bestArea) {
					bestArea = area;
					bestDir = d;
				}
			}
		}
		rotP << bestDir(0), bestDir(1),
		       -bestDir(1), bestDir(0);
	}
	// assemble the world-to-local rotation (rows are the local axes):
	// the hyperplane axes first, the up direction last
	m_rot.template topRows<DIMSP>() = rotP * basisP.transpose();
	m_rot.row(DIMSP) = up.transpose();
	// the hyperplane basis can come out with either handedness; flipping the first
	// axis makes the frame right-handed without changing the fitted box
	if (m_rot.determinant() < TYPE(0))
		m_rot.row(0) = -m_rot.row(0);
	ASSERT(ISEQUAL(m_rot.determinant(), TYPE(1)));
}
// method to set the OBB center and size that contains the given points
// the rotations should be already set
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::SetBounds(const POINT* pts, size_t n)
{
	ASSERT(n >= DIMS);
	ASSERT(ISEQUAL((m_rot*m_rot.transpose()).trace(), TYPE(DIMS)) && ISEQUAL(m_rot.determinant(), TYPE(1)));

	// build the bounding box extents in the rotated frame
	AABB aabb(m_rot * pts[0]);
	for (size_t i=1; i<n; ++i)
		aabb.Insert(m_rot * pts[i]);

	// set the center of the OBB to be the average of the
	// minimum and maximum, and the extents be half of the
	// difference between the minimum and maximum
	m_pos = m_rot.transpose() * aabb.GetCenter();
	m_ext = aabb.GetSize() * TYPE(0.5);
} // Set
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::BuildBegin()
{
	// accumulate the second moments in m_rot, the point sum in m_pos
	// and the point count in the m_ext storage
	m_rot = MATRIX::Zero();
	m_pos = POINT::Zero();
	m_ext = POINT::Zero();
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::BuildAdd(const POINT& p)
{
	m_rot += p * p.transpose();
	m_pos += p;
	// the count must stay exact for arbitrary n (a float counter saturates at 2^24),
	// so it lives as an integer in the extents storage, unused during build
	STATIC_ASSERT(sizeof(POINT) >= sizeof(size_t));
	size_t n;
	memcpy(&n, m_ext.data(), sizeof(n));
	++n;
	memcpy(m_ext.data(), &n, sizeof(n));
}
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::BuildEnd()
{
	STATIC_ASSERT(sizeof(POINT) >= sizeof(size_t));
	size_t n;
	memcpy(&n, m_ext.data(), sizeof(n));
	ASSERT(n > 0);
	// build the covariance matrix out of the accumulated moments
	const TYPE invN(TYPE(1)/TYPE(n));
	SetRotation(MATRIX((m_rot - m_pos*m_pos.transpose()*invN)*invN));
} // Build
/*----------------------------------------------------------------*/


// check if the oriented bounding box has positive size
template <typename TYPE, int DIMS>
inline bool TOBB<TYPE,DIMS>::IsValid() const
{
	return m_ext.minCoeff() > TYPE(0);
} // IsValid
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline TOBB<TYPE,DIMS>& TOBB<TYPE,DIMS>::Enlarge(TYPE x)
{
	m_ext.array() += x;
	return *this;
}
template <typename TYPE, int DIMS>
inline TOBB<TYPE,DIMS>& TOBB<TYPE,DIMS>::EnlargePercent(TYPE x)
{
	m_ext *= x;
	return *this;
} // Enlarge
/*----------------------------------------------------------------*/


// Update the box by the given pos delta.
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Translate(const POINT& d)
{
	m_pos += d;
}
// Update the box by the given transform.
template <typename TYPE, int DIMS>
inline void TOBB<TYPE,DIMS>::Transform(const MATRIX& m)
{
	Eigen::Transform<Type, DIMS, Eigen::Affine> transform(m);
	MATRIX rotation, scaling;
	transform.computeRotationScaling(&rotation, &scaling);
	m_rot = m_rot * rotation.transpose();
	m_pos = m * m_pos;
	m_ext = scaling * m_ext;
} // Transform
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
	// generate all corner combinations using bit patterns;
	// use bit j of i to determine sign: 0 = subtract, 1 = add
	POINT axisVectors[DIMS];
	for (int j=0; j<DIMS; ++j)
		axisVectors[j] = m_rot.row(j) * m_ext[j];
	for (int i=0; i<numCorners; ++i) {
		pts[i] = m_pos;
		for (int j=0; j<DIMS; ++j) {
			if (i & (1 << j))
				pts[i] += axisVectors[j];
			else
				pts[i] -= axisVectors[j];
		}
	}
} // GetCorners
// constructs the corner of the aligned bounding box in world space
template <typename TYPE, int DIMS>
inline typename TOBB<TYPE,DIMS>::AABB TOBB<TYPE,DIMS>::GetAABB() const
{
	POINT pts[numCorners];
	GetCorners(pts);
	return AABB(pts, numCorners);
} // GetAABB
/*----------------------------------------------------------------*/

// computes the volume of the OBB, which is a measure of
// how tight the fit is (better OBBs will have smaller volumes)
template <typename TYPE, int DIMS>
inline TYPE TOBB<TYPE,DIMS>::GetVolume() const
{
	return m_ext.prod()*TYPE(numCorners);
}
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
bool TOBB<TYPE,DIMS>::Intersects(const POINT& pt) const
{
	const POINT dist(m_rot * (pt - m_pos));
	return (dist.array().abs() <= m_ext.array()).all();
} // Intersects(POINT)
/*----------------------------------------------------------------*/


// Surface (aproximate) point extraction from 3D point clouds using directional vector summation.
//
// This algorithm approximates which points lie on the surface (outer boundary) of a 3D point cloud,
// based on the spatial distribution of their neighbors.
//
// For each point:
// 1. Find its k nearest neighbors using a KD-tree (via nanoflann).
// 2. Compute unit direction vectors from the point to each neighbor.
// 3. Sum all direction vectors and compute the magnitude of the result.
//    - A large magnitude indicates an asymmetric neighborhood — likely a surface point.
//    - A near-zero magnitude indicates a symmetric (interior) neighborhood.
//
// After computing this "surface score" for each point, the algorithm selects the top N% of points
// with the highest scores as likely surface points.

template <typename TYPE, int DIMS>
struct TPointCloudSurfaceAdaptor {
	const typename TOBB<TYPE,DIMS>::POINT* pts;
	size_t n;
	TPointCloudSurfaceAdaptor(const typename TOBB<TYPE,DIMS>::POINT* pts_, size_t n_) : pts(pts_), n(n_) {}
	inline size_t kdtree_get_point_count() const { return n; }
	inline TYPE kdtree_get_pt(const size_t idx, int dim) const { return pts[idx][dim]; }
	template <class BBOX>
	bool kdtree_get_bbox(BBOX&) const { return false; }
};

template <typename TYPE, int DIMS>
std::vector<TYPE> TOBB<TYPE,DIMS>::ComputeSurfacePointsScores(const POINT* pts, size_t n, int k)
{
	using PointCloudSurfaceAdaptor = TPointCloudSurfaceAdaptor<TYPE, DIMS>;
	using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<TYPE, PointCloudSurfaceAdaptor>,
		PointCloudSurfaceAdaptor, DIMS>;

	PointCloudSurfaceAdaptor adaptor(pts, n);
	KDTree kdtree(DIMS, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams());
	kdtree.buildIndex();

	std::vector<TYPE> scores(n);
	std::vector<size_t> indices(k + 1);
	std::vector<TYPE> dists(k + 1);
	for (size_t i = 0; i < n; ++i) {
		nanoflann::KNNResultSet<TYPE> resultSet(k + 1);
		resultSet.init(indices.data(), dists.data());
		kdtree.findNeighbors(resultSet, &pts[i][0], nanoflann::SearchParameters());
		POINT sum_vector = POINT::Zero();
		for (size_t j = 1; j < resultSet.size(); ++j) { // skip self
			POINT dir = pts[indices[j]] - pts[i];
			TYPE norm = dir.norm();
			if (!ISZERO(norm))
				sum_vector += dir / norm;
		}
		scores[i] = sum_vector.norm();
	}
	return scores;
}

template <typename TYPE, int DIMS>
std::vector<typename TOBB<TYPE,DIMS>::POINT> TOBB<TYPE,DIMS>::FilterSurfacePoints(const POINT* pts, size_t n, int k, TYPE percentile)
{
	auto scores = ComputeSurfacePointsScores(pts, n, k);
	TYPE threshold;
	if (percentile > 0) {
		// Calculate the index for the given percentile
		size_t index = static_cast<size_t>((TYPE(1) - percentile) * scores.size());
		const auto nth = scores.begin() + index;
		std::nth_element(scores.begin(), nth, scores.end());
		threshold = *nth;
	} else {
		// Use given percentile param as threshold
		threshold = -percentile;
	}
	std::vector<POINT> result;
	for (size_t i = 0; i < n; ++i) {
		if (scores[i] > threshold)
			result.push_back(pts[i]);
	}
	return result;
}
/*----------------------------------------------------------------*/
