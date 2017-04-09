////////////////////////////////////////////////////////////////////
// AABB.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
inline TAABB<TYPE,DIMS>::TAABB(bool)
	:
	ptMin(POINT::Constant(std::numeric_limits<TYPE>::max())),
	ptMax(POINT::Constant(std::numeric_limits<TYPE>::lowest()))
{
}
template <typename TYPE, int DIMS>
inline TAABB<TYPE,DIMS>::TAABB(const POINT& _pt)
	:
	ptMin(_pt), ptMax(_pt)
{
}
template <typename TYPE, int DIMS>
inline TAABB<TYPE,DIMS>::TAABB(const POINT& _ptMin, const POINT& _ptMax)
	:
	ptMin(_ptMin), ptMax(_ptMax)
{
}
template <typename TYPE, int DIMS>
inline TAABB<TYPE,DIMS>::TAABB(const POINT& center, const TYPE& radius)
	:
	ptMin(center-POINT::Constant(radius)), ptMax(center+POINT::Constant(radius))
{
}
template <typename TYPE, int DIMS>
template <typename TPoint>
inline TAABB<TYPE,DIMS>::TAABB(const TPoint* pts, size_t n)
{
	Set(pts, n);
} // constructor
template <typename TYPE, int DIMS>
template <typename CTYPE>
inline TAABB<TYPE,DIMS>::TAABB(const TAABB<CTYPE,DIMS>& rhs)
	:
	ptMin(rhs.ptMin.template cast<TYPE>()), ptMax(rhs.ptMax.template cast<TYPE>())
{
} // copy constructor
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::Reset()
{
	ptMin = POINT::Constant(std::numeric_limits<TYPE>::max());
	ptMax = POINT::Constant(std::numeric_limits<TYPE>::lowest());
}
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::Set(const POINT& _pt)
{
	ptMin = ptMax = _pt;
}
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::Set(const POINT& _ptMin, const POINT& _ptMax)
{
	ptMin = _ptMin;
	ptMax = _ptMax;
}
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::Set(const POINT& center, const TYPE& radius)
{
	ptMin = center-POINT::Constant(radius);
	ptMax = center+POINT::Constant(radius);
}
template <typename TYPE, int DIMS>
template <typename TPoint>
inline void TAABB<TYPE,DIMS>::Set(const TPoint* pts, size_t n)
{
	ASSERT(n > 0);
	ptMin = ptMax = pts[0];
	for (size_t i=1; i<n; ++i)
		Insert(pts[i]);
} // Set
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline bool TAABB<TYPE,DIMS>::IsEmpty() const
{
	for (int i=0; i<DIMS; ++i)
		if (ptMin[i] > ptMax[i])
			return true;
	return false;
} // IsEmpty
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::Enlarge(TYPE x)
{
	ptMin.array() -= x;
	ptMax.array() += x;
}
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::EnlargePercent(TYPE x)
{
	ptMin *= x;
	ptMax *= x;
} // Enlarge
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline typename TAABB<TYPE,DIMS>::POINT TAABB<TYPE,DIMS>::GetCenter() const
{
	return (ptMax + ptMin) * TYPE(0.5);
}
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::GetCenter(POINT& ptCenter) const
{
	ptCenter = (ptMax + ptMin) * TYPE(0.5);
} // GetCenter
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline typename TAABB<TYPE,DIMS>::POINT TAABB<TYPE,DIMS>::GetSize() const
{
	return ptMax - ptMin;
}
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::GetSize(POINT& ptSize) const
{
	ptSize = ptMax - ptMin;
} // GetSize
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::GetCorner(BYTE i, POINT& ptCorner) const
{
	ASSERT(i<numCorners);
	if (DIMS == 1) {
		ptCorner(0) = operator[](i);
	}
	if (DIMS == 2) {
		ptCorner(0) = operator[]((i/2)*2 + 0);
		ptCorner(1) = operator[]((i%2)*2 + 1);
	}
	if (DIMS == 3) {
		ptCorner(0) = operator[]((i/4)*3     + 0);
		ptCorner(1) = operator[](((i/2)%2)*3 + 1);
		ptCorner(2) = operator[]((i%2)*3     + 2);
	}
}
template <typename TYPE, int DIMS>
inline typename TAABB<TYPE,DIMS>::POINT TAABB<TYPE,DIMS>::GetCorner(BYTE i) const
{
	POINT ptCorner;
	GetCorner(i, ptCorner);
	return ptCorner;
} // GetCorner
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::GetCorners(POINT pts[numCorners]) const
{
	for (BYTE i=0; i<numCorners; ++i)
		GetCorner(i, pts[i]);
} // GetCorners
/*----------------------------------------------------------------*/


// Update the box by the given pos delta.
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::Translate(const POINT& d)
{
	ptMin += d;
	ptMax += d;
}
/*----------------------------------------------------------------*/


// Update the box by the given transform.
template <typename TYPE, int DIMS>
inline void TAABB<TYPE,DIMS>::Transform(const MATRIX& m)
{
	ptMin = m * ptMin;
	ptMax = m * ptMax;
}
/*----------------------------------------------------------------*/


// Update the box such that it contains the given point.
template <typename TYPE, int DIMS>
void TAABB<TYPE,DIMS>::InsertFull(const POINT& pt)
{
	if (ptMin[0] > pt[0])
		ptMin[0] = pt[0];
	if (ptMax[0] < pt[0])
		ptMax[0] = pt[0];

	if (DIMS > 1) {
	if (ptMin[1] > pt[1])
		ptMin[1] = pt[1];
	if (ptMax[1] < pt[1])
		ptMax[1] = pt[1];
	}

	if (DIMS > 2) {
	if (ptMin[2] > pt[2])
		ptMin[2] = pt[2];
	if (ptMax[2] < pt[2])
		ptMax[2] = pt[2];
	}
}
// same as above, but for the initialized case
template <typename TYPE, int DIMS>
void TAABB<TYPE,DIMS>::Insert(const POINT& pt)
{
	if (ptMin[0] > pt[0])
		ptMin[0] = pt[0];
	else if (ptMax[0] < pt[0])
		ptMax[0] = pt[0];

	if (DIMS > 1) {
	if (ptMin[1] > pt[1])
		ptMin[1] = pt[1];
	else if (ptMax[1] < pt[1])
		ptMax[1] = pt[1];
	}

	if (DIMS > 2) {
	if (ptMin[2] > pt[2])
		ptMin[2] = pt[2];
	else if (ptMax[2] < pt[2])
		ptMax[2] = pt[2];
	}
}
/*----------------------------------------------------------------*/

// Update the box such that it contains the given point.
template <typename TYPE, int DIMS>
void TAABB<TYPE,DIMS>::Insert(const TAABB& aabb)
{
	if (ptMin[0] > aabb.ptMin[0])
		ptMin[0] = aabb.ptMin[0];
	if (ptMax[0] < aabb.ptMax[0])
		ptMax[0] = aabb.ptMax[0];

	if (DIMS > 1) {
	if (ptMin[1] > aabb.ptMin[1])
		ptMin[1] = aabb.ptMin[1];
	if (ptMax[1] < aabb.ptMax[1])
		ptMax[1] = aabb.ptMax[1];
	}

	if (DIMS > 2) {
	if (ptMin[2] > aabb.ptMin[2])
		ptMin[2] = aabb.ptMin[2];
	if (ptMax[2] < aabb.ptMax[2])
		ptMax[2] = aabb.ptMax[2];
	}
}
/*----------------------------------------------------------------*/


// intersection between two AABBs
template <typename TYPE, int DIMS>
bool TAABB<TYPE,DIMS>::Intersects(const TAABB& aabb) const
{
	if (aabb.ptMax[0] < ptMin[0]) return false;
	if (aabb.ptMin[0] >= ptMax[0]) return false;
	if (DIMS > 1) {
	if (aabb.ptMax[1] < ptMin[1]) return false;
	if (aabb.ptMin[1] >= ptMax[1]) return false;
	}
	if (DIMS > 2) {
	if (aabb.ptMax[2] < ptMin[2]) return false;
	if (aabb.ptMin[2] >= ptMax[2]) return false;
	}
	return true;
}
template <typename TYPE, int DIMS>
bool TAABB<TYPE,DIMS>::IntersectsComplete(const TAABB& aabb, TYPE tolerance) const
{
	if (aabb.ptMax[0]+tolerance < ptMin[0]) return false;
	if (aabb.ptMin[0] > ptMax[0]+tolerance) return false;
	if (DIMS > 1) {
	if (aabb.ptMax[1]+tolerance < ptMin[1]) return false;
	if (aabb.ptMin[1] > ptMax[1]+tolerance) return false;
	}
	if (DIMS > 2) {
	if (aabb.ptMax[2]+tolerance < ptMin[2]) return false;
	if (aabb.ptMin[2] > ptMax[2]+tolerance) return false;
	}
	return true;
}
template <typename TYPE, int DIMS>
bool TAABB<TYPE,DIMS>::IntersectsComplete(const TAABB& aabb) const
{
	return IntersectsComplete(aabb, ZEROTOLERANCE<TYPE>());
} // Intersects(TAABB)
/*----------------------------------------------------------------*/

// does TAABB contain the given point
template <typename TYPE, int DIMS>
bool TAABB<TYPE,DIMS>::Intersects(const POINT& pt) const
{
	if (pt[0] < ptMin[0]) return false;
	if (pt[0] >= ptMax[0]) return false;
	if (DIMS > 1) {
	if (pt[1] < ptMin[1]) return false;
	if (pt[1] >= ptMax[1]) return false;
	}
	if (DIMS > 2) {
	if (pt[2] < ptMin[2]) return false;
	if (pt[2] >= ptMax[2]) return false;
	}
	return true;
}
template <typename TYPE, int DIMS>
bool TAABB<TYPE,DIMS>::IntersectsComplete(const POINT& pt, TYPE tolerance) const
{
	if (pt[0]+tolerance < ptMin[0]) return false;
	if (pt[0] > ptMax[0]+tolerance) return false;
	if (DIMS > 1) {
	if (pt[1]+tolerance < ptMin[1]) return false;
	if (pt[1] > ptMax[1]+tolerance) return false;
	}
	if (DIMS > 2) {
	if (pt[2]+tolerance < ptMin[2]) return false;
	if (pt[2] > ptMax[2]+tolerance) return false;
	}
	return true;
}
template <typename TYPE, int DIMS>
bool TAABB<TYPE,DIMS>::IntersectsComplete(const POINT& pt) const
{
	return IntersectsComplete(pt, ZEROTOLERANCE<TYPE>());
} // Intersects(point)
/*----------------------------------------------------------------*/


// split this TAABB by the given axis
// returns number of children (0 or 2)
template <typename TYPE, int DIMS>
unsigned TAABB<TYPE,DIMS>::SplitBy(TYPE d, int a, TAABB child[2]) const
{
	ASSERT(a < DIMS);
	if (d >= ptMin[a] && d < ptMax[a]) {
		TAABB& child0 = child[0];
		child0.Set(ptMin, ptMax);
		TAABB& child1 = child[1];
		child1.Set(ptMin, ptMax);
		child0.ptMax[a] = child1.ptMin[a] = d;
		return 2;
	}
	return 0;
} // SplitBy(axis)
/*----------------------------------------------------------------*/

// split this TAABB by the given point
// returns number of children (0 - numScalar)
// idxFirst returns the index of the first aabb
template <typename TYPE, int DIMS>
unsigned TAABB<TYPE,DIMS>::SplitBy(const POINT& pt, TAABB child[numChildren], unsigned& idxFirst) const
{
	idxFirst = 0;
	unsigned size;
	int a = 0;
	do {
		size = SplitBy(pt[a], a, child);
		if (++a == DIMS)
			return size;
	} while (size == 0);
	do {
		unsigned n = 0;
		for (unsigned b=0; b<size; ++b) {
			n += child[(idxFirst+b)%numChildren].SplitBy(pt[a], a, child+((idxFirst+size+n)%numChildren));
			if (n == 0)
				goto MAIN_LOOP_END;
		}
		idxFirst += size;
		size = n;
		MAIN_LOOP_END:;
	} while (++a < DIMS);
	return size;
} // SplitBy(point)
/*----------------------------------------------------------------*/
