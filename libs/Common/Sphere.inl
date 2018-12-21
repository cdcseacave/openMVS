////////////////////////////////////////////////////////////////////
// Sphere.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS>
inline TSphere<TYPE,DIMS>::TSphere(const POINT& p1, const POINT& p2, const POINT& p3)
{
	Set(p1, p2, p3);
} // constructor
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TSphere<TYPE,DIMS>::Set(const POINT& c, TYPE r)
{
	center = c;
	radius = r;
}
template <typename TYPE, int DIMS>
inline void TSphere<TYPE, DIMS>::Set(const POINT& p1, const POINT& p2, const POINT& p3)
{
	// compute relative distances
	TYPE A((p1 - p2).squaredNorm());
	TYPE B((p2 - p3).squaredNorm());
	TYPE C((p3 - p1).squaredNorm());

	// re-orient triangle (make A longest side)
	const POINT *a(&p3), *b(&p1), *c(&p2);
	if (B < C) std::swap(B, C), std::swap(b, c);
	if (A < B) std::swap(A, B), std::swap(a, b);

	// if obtuse
	if (B + C <= A) {
		// just use longest diameter
		radius = SQRT(A) / TYPE(2);
		center = (*b + *c) / TYPE(2);
	} else {
		// otherwise circumscribe (http://en.wikipedia.org/wiki/Circumscribed_circle)
		const TYPE cos_a(SQUARE(B + C - A) / (B*C*TYPE(4)));
		radius = SQRT(A / ((TYPE(1) - cos_a)*TYPE(4)));
		const POINT alpha(*a - *c), beta(*b - *c);
		const POINT alphaXbeta(alpha.cross(beta));
		center = (beta * alpha.squaredNorm() - alpha * beta.squaredNorm()).cross(alphaXbeta) /
			(alphaXbeta.squaredNorm() * TYPE(2)) + *c;
	}
}
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS>
inline void TSphere<TYPE,DIMS>::Enlarge(TYPE x)
{
	radius += x;
}
template <typename TYPE, int DIMS>
inline void TSphere<TYPE,DIMS>::EnlargePercent(TYPE x)
{
	radius *= x;
} // Enlarge
/*----------------------------------------------------------------*/


// Classify point to sphere.
template <typename TYPE, int DIMS>
inline GCLASS TSphere<TYPE, DIMS>::Classify(const POINT& p) const
{
	if ((center - p).squaredNorm() > SQUARE(radius))
		return CULLED;
	return VISIBLE;
}
/*----------------------------------------------------------------*/
