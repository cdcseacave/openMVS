// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////


// C L A S S  //////////////////////////////////////////////////////

template <typename TYPE, int DIMS>
inline TLine<TYPE,DIMS>::TLine(const POINT& _pt1, const POINT& _pt2)
	:
	pt1(_pt1), pt2(_pt2)
{
	ASSERT(!ISZERO((_pt1-_pt2).norm()));
} // constructor
template <typename TYPE, int DIMS>
template <typename CTYPE>
inline TLine<TYPE,DIMS>::TLine(const TLine<CTYPE,DIMS>& rhs)
	:
	pt1(rhs.pt1.template cast<TYPE>()), pt2(rhs.pt2.template cast<TYPE>())
{
} // copy constructor
/*----------------------------------------------------------------*/


// set attributes
template <typename TYPE, int DIMS>
inline void TLine<TYPE,DIMS>::Set(const POINT& _pt1, const POINT& _pt2)
{
	ASSERT(!ISZERO((_pt1-_pt2).norm()));
	pt1 = _pt1;
	pt2 = _pt2;
}
/*----------------------------------------------------------------*/


// least squares refinement of the line to the given 3D point set
// (return the number of iterations)
template <typename TYPE, int DIMS>
template <typename RobustNormFunctor>
int TLine<TYPE,DIMS>::Optimize(const POINT* points, size_t size, const RobustNormFunctor& robust, int maxIters)
{
	ASSERT(DIMS == 3);
	ASSERT(size >= numParams);
	struct OptimizationFunctor {
		const POINT* points;
		size_t size;
		double scale;
		const RobustNormFunctor& robust;
		// construct with the data points
		OptimizationFunctor(const POINT* _points, size_t _size, const RobustNormFunctor& _robust)
			: points(_points), size(_size), robust(_robust) { ASSERT(size < std::numeric_limits<int>::max()); }
		static void Residuals(const double* x, int nPoints, const void* pData, double* fvec, double* fjac, int* /*info*/) {
			const OptimizationFunctor& data = *reinterpret_cast<const OptimizationFunctor*>(pData);
			ASSERT((size_t)nPoints == data.size && fvec != NULL && fjac == NULL);
			TLine<double,DIMS> line;
			for (int j=0; j<DIMS; ++j)
				line.pt1(j) = x[j];
			DirScale2Vector(x+DIMS, &data.scale, line.pt2.data());
			line.pt2 += line.pt1;
			for (size_t i=0; i<data.size; ++i)
				fvec[i] = data.robust(line.Distance(data.points[i].template cast<double>()));
		}
	} functor(points, size, robust);
	double arrParams[numParams];
	for (int j=0; j<DIMS; ++j)
		arrParams[j] = (double)pt1(j);
	POINT dir(pt2-pt1);
	Vector2DirScale(dir.data(), arrParams+DIMS, &functor.scale);
	lm_control_struct control = {1.e-6, 1.e-7, 1.e-8, 1.e-8, 100.0, maxIters}; // lm_control_float;
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
		DEBUG_ULTIMATE("error: refine line: %s", lm_infmsg[status.info]);
		return 0;
	}
	for (int j=0; j<DIMS; ++j)
		pt1(j) = (TYPE)arrParams[j];
	DirScale2Vector(arrParams+DIMS, &functor.scale, dir.data());
	pt2 = pt1+dir;
	return status.nfev;
}
template <typename TYPE, int DIMS>
int TLine<TYPE,DIMS>::Optimize(const POINT* points, size_t size, int maxIters)
{
	const auto identity = [](double x) { return x; };
	return Optimize(points, size, identity, maxIters);
} // Optimize
/*----------------------------------------------------------------*/


// get attributes
template <typename TYPE, int DIMS>
inline TYPE TLine<TYPE,DIMS>::GetLength() const
{
	return (pt2 - pt1).norm();
}
template <typename TYPE, int DIMS>
inline TYPE TLine<TYPE,DIMS>::GetLengthSq() const
{
	return (pt2 - pt1).squaredNorm();
}
template <typename TYPE, int DIMS>
inline typename TLine<TYPE,DIMS>::POINT TLine<TYPE,DIMS>::GetCenter() const
{
	return (pt2 + pt1) / TYPE(2);
}
template <typename TYPE, int DIMS>
inline typename TLine<TYPE,DIMS>::VECTOR TLine<TYPE,DIMS>::GetDir() const
{
	return (pt2 - pt1);
}
template <typename TYPE, int DIMS>
inline typename TLine<TYPE,DIMS>::VECTOR TLine<TYPE,DIMS>::GetNormDir() const
{
	return (pt2 - pt1).normalized();
}
template <typename TYPE, int DIMS>
inline typename TLine<TYPE,DIMS>::RAY TLine<TYPE,DIMS>::GetRay() const
{
	return RAY(pt1, GetNormDir());
}
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline bool TLine<TYPE,DIMS>::IsSame(const TLine& line, TYPE th) const
{
	const TYPE thSq(SQUARE(th));
	const VECTOR l(pt2-pt1);
	const TYPE invLenSq(INVERT(l.squaredNorm()));
	const VECTOR r1(pt1-line.pt1);
	const TYPE dSq1((l.cross(r1)).squaredNorm()*invLenSq);
	if (dSq1 > thSq)
		return false;
	const VECTOR r2(pt1-line.pt2);
	const TYPE dSq2((l.cross(r2)).squaredNorm()*invLenSq);
	return dSq2 <= thSq;
}
/*----------------------------------------------------------------*/


// test for intersection with aabb
template <typename TYPE, int DIMS>
bool TLine<TYPE,DIMS>::Intersects(const AABB &aabb) const
{
	return GetRay().Intersects(aabb);
} // Intersects(AABB)
template <typename TYPE, int DIMS>
bool TLine<TYPE,DIMS>::Intersects(const AABB &aabb, TYPE& t) const
{
	return GetRay().Intersects(aabb, t);
} // Intersects(AABB)
/*----------------------------------------------------------------*/

// Computes the distance between the line and a point.
template <typename TYPE, int DIMS>
inline TYPE TLine<TYPE,DIMS>::DistanceSq(const POINT& pt) const
{
	const VECTOR l(pt2-pt1), r(pt1-pt);
	if (DIMS == 2)
		return TYPE(SQUARE(l[0]*r[1]-r[0]*l[1])/(l[0]*l[0]+l[1]*l[1]));
	ASSERT(DIMS == 3);
	return TYPE((l.cross(r)).squaredNorm()/l.squaredNorm());
} // DistanceSq(POINT)
template <typename TYPE, int DIMS>
inline TYPE TLine<TYPE,DIMS>::Distance(const POINT& pt) const
{
	return SQRT(DistanceSq(pt));
} // Distance(POINT)
/*----------------------------------------------------------------*/


// Computes the position on the line segment of the point projection.
// Returns 0 if it coincides with the first point, and 1 if it coincides with the second point.
template <typename TYPE, int DIMS>
inline TYPE TLine<TYPE,DIMS>::Classify(const POINT& p) const
{
	const VECTOR vL(pt2 - pt1);
	ASSERT(!ISZERO(vL.squaredNorm()));
	const VECTOR vP(p - pt1);
	return vL.dot(vP) / vL.squaredNorm();
} // Classify(POINT)
// Calculate point's projection on this line (closest point to this line).
template <typename TYPE, int DIMS>
inline typename TLine<TYPE,DIMS>::POINT TLine<TYPE,DIMS>::ProjectPoint(const POINT& p) const
{
	const VECTOR vL(pt2 - pt1);
	ASSERT(!ISZERO(vL.squaredNorm()));
	const VECTOR vP(p - pt1);
	return pt1 + vL * (vL.dot(vP) / vL.squaredNorm());
} // ProjectPoint
/*----------------------------------------------------------------*/


template <typename TYPE, typename TYPEW>
template <typename TYPEE>
TPoint3<TYPEE> FitLineOnline<TYPE,TYPEW>::GetLine(TLine<TYPEE,3>& line) const
{
	TPoint3<TYPEW> avg, dir;
	const TPoint3<TYPEW> quality(this->GetModel(avg, dir));
	const TPoint3<TYPEW> pt2(avg+dir);
	line.Set(TPoint3<TYPEE>(avg), TPoint3<TYPEE>(pt2));
	return TPoint3<TYPEE>(quality);
}
/*----------------------------------------------------------------*/
