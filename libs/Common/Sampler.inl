// Copyright (c) 2012-2015 OpenMVG.
// Copyright (c) 2012-2015 Pierre MOULON.
// Copyright (c) 2015 Romuald Perrot.
// Copyright (c) 2015 cDc@seacave.
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

namespace Sampler {

// Sampling functors
// These functors computes weight associated to each pixels
//
// For a (relative) sampling position x (in [0,1]) between two (consecutive) points :
//
//   A  .... x ......... B
//
//   w[0] is the weight associated to A
//   w[1] is the weight associated to B
//
// Note: The following functors generalize the sampling to more than two neighbors
// They all contains the width variable that specify the number of neighbors used for sampling
//
// All contain the operator () with the following definition:
//
//   @brief Computes weight associated to neighboring pixels
//   @author Romuald Perrot <perrot.romuald_AT_gmail.com>
//   @param x Sampling position
//   @param[out] weigth Sampling factors associated to the neighboring
//   @note weight must be at least width length
// Linear sampling (ie: linear interpolation between two pixels)
template <typename TYPE>
struct Linear {
	typedef TYPE Type;

	enum { halfWidth = 1 };
	enum { width = halfWidth*2 };

	inline Linear() {}
	
	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = TYPE(1) - x;
		weigth[1] = x;
	}
};

// Cubic interpolation between 4 pixels
//
// Interpolation weight is for A,B,C and D pixels given a x position as illustrated as follow :
//
// A      B    x C      D
//
// Cubic Convolution Interpolation for Digital Image Processing , R. Keys, eq(4)
//
// The cubic filter family with the two free parameters usually named B and C:
// if |x|<1
//   ((12-9B-6C)|x|^3 + (-18+12B+6C)|x|^2 + (6-2B))/6
// if 1<=|x|<2
//   ((-B-6C)|x|^3 + (6B+30C)|x|^2 + (-12B-48C)|x| + (8B+24C))/6
template <typename TYPE>
struct Cubic {
	typedef TYPE Type;

	enum { halfWidth = 2 };
	enum { width = halfWidth*2 };

	// Sharpness coefficient used to control sharpness of the cubic curve (between 0.5 to 0.75)
	// cubic(0,1/2): 0.5 gives better mathematically result (ie: approximation at 3 order precision - Catmull-Rom)
	const TYPE sharpness;
	inline Cubic(const TYPE& _sharpness=TYPE(0.5)) : sharpness(_sharpness) {}

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		// remember :
		// A      B    x  C       D

		// weigth[0] -> weight for A
		// weight[1] -> weight for B
		// weight[2] -> weight for C
		// weight[3] -> weigth for D

		weigth[0] = CubicInter12(x + TYPE(1));
		weigth[1] = CubicInter01(x);
		weigth[2] = CubicInter01(TYPE(1) - x);
		weigth[3] = CubicInter12(TYPE(2) - x);
	}

	// Cubic interpolation for x (in [0,1])
	inline TYPE CubicInter01(const TYPE x) const {
		// A = -sharpness
		// f(x) = (A + 2) * x^3 - (A + 3) * x^2 + 1
		return ((TYPE(2)-sharpness)*x - (TYPE(3)-sharpness))*x*x + TYPE(1);
	}
	// Cubic interpolation for x (in [1,2])
	inline TYPE CubicInter12(const TYPE x) const {
		// A = -sharpness
		// f(x) = A * x^3 - 5 * A * x^2 + 8 * A * x - 4 * A
		return (((TYPE(5)-x)*x - TYPE(8))*x + TYPE(4))*sharpness;
	}
};

// Sampler spline16 -> Interpolation on 4 points used for 2D ressampling (16 = 4x4 sampling)
// Cubic interpolation with 0-derivative at edges (ie at A and D points)
// See Helmut Dersch for more details
//
// Some refs :
//  -   http://forum.doom9.org/archive/index.php/t-147117.html
//  -   http://avisynth.nl/index.php/Resampling
//  -   http://www.ipol.im/pub/art/2011/g_lmii/
//
// The idea is to consider 3 cubic splines (f1,f2,f3) in the sampling interval :
//
// A   f1    B    f2    C     f3     D
//
// with curves defined as follow :
// f1(x) = a1 x^3 + b1 x^2 + c1 x + d1
// f2(x) = a2 x^3 + b2 x^2 + c2 x + d2
// f3(x) = a3 x^3 + b2 x^2 + c3 x + d3
//
// We want to compute spline coefs for A,B,C,D assuming that:
// y0 = coef[A] = f1(-1)
// y1 = coef[B] = f1(0) = f2(0)
// y2 = coef[C] = f2(1) = f3(1)
// y3 = coef[D] = f3(2)
//
// coef are computed using the following constraints :
// Curve is continuous, ie:
// f1(0)  = f2(0)
// f2(1)  = f3(1)
// First derivative are equals, ie:
// f1'(0) = f2'(0)
// f2'(1) = f3'(1)
// Second derivative are equals, ie:
// f1''(0) = f2''(0)
// f2''(1) = f3''(0)
// Curve is, at boundary, with second derivative set to zero (it's a constraint introduced by Dersch), ie:
// f1''(-1) = 0
// f3''(2) = 0
//
// Then, you can solve for (a1,a2,a3,b1,b2,b3,c1,c2,c3,d1,d2,d3)
//
// for ex, for curve f2 you find :
//
// d2 = y1                                      // easy since y1 = f2(0)
// c2 = - 7/15 y0 - 1/5 y1 + 4/5 y2 - 2/15 y3
// b2 = 4/5 y0 - 9/5 y1 + 6/5 y2 - 1/5 y3
// a2 = - 1/3 y0 + y1 - y2 + 1/3 y3
//
//
// When you have coefs, you just have to express your curve as a linear combination of the control points, fort ex
// with f2 :
//
//
// f2(x) = w0(x) * y0 + w1(x) + y1 + w2(x) * y2 + w3(x) * y3
//
// with :
//
// w0(x) = - 1/3 * x^3 + 4/5 * x^2 - 7/15 * x
// w1(x) = x^3 - 9/5 * x^2 - 1/5 * x + 1
// w2(x) = -x^3 + 6/5 * x^2 + 4/5 * x
// w3(x) = 1/3 * x^3 - 1/5 * x^2 - 2/15 * x
//
// substituting boundary conditions gives the correct coefficients for y0,y1,y2,y3 giving the final sampling scheme
template <typename TYPE>
struct Spline16 {
	typedef TYPE Type;

	enum { halfWidth = 2 };
	enum { width = halfWidth*2 };

	inline Spline16() {}

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = ((TYPE(-1) / TYPE(3) * x + TYPE(4) / TYPE(5)) * x - TYPE(7) / TYPE(15)) * x;
		weigth[1] = ((x - TYPE(9) / TYPE(5)) * x - TYPE(1) / TYPE(5)) * x + TYPE(1);
		weigth[2] = ((TYPE(6) / TYPE(5) - x) * x + TYPE(4) / TYPE(5)) * x;
		weigth[3] = ((TYPE(1) / TYPE(3) * x - TYPE(1) / TYPE(5)) * x - TYPE(2) / TYPE(15)) * x;
	}
};

// Sampler spline 36
// Same as spline 16 but on 6 neighbors (used for 6x6 frame)
template <typename TYPE>
struct Spline36 {
	typedef TYPE Type;

	enum { halfWidth = 3 };
	enum { width = halfWidth*2 };

	inline Spline36() {}

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = ((TYPE(1) / TYPE(11) * x - TYPE(45) / TYPE(209)) * x + TYPE(26) / TYPE(209)) * x;
		weigth[1] = ((TYPE(-6) / TYPE(11) * x + TYPE(270) / TYPE(209)) * x - TYPE(156) / TYPE(209)) * x;
		weigth[2] = ((TYPE(13) / TYPE(11) * x - TYPE(453) / TYPE(209)) * x - TYPE(3) / TYPE(209)) * x + TYPE(1);
		weigth[3] = ((TYPE(-13) / TYPE(11) * x + TYPE(288) / TYPE(209)) * x + TYPE(168) / TYPE(209)) * x;
		weigth[4] = ((TYPE(6) / TYPE(11) * x - TYPE(72) / TYPE(209)) * x - TYPE(42) / TYPE(209)) * x;
		weigth[5] = ((TYPE(-1) / TYPE(11) * x + TYPE(12) / TYPE(209)) * x + TYPE(7) / TYPE(209)) * x;
	}
};

// Sampler spline 64
// Same as spline 16 but on 8 neighbors (used for 8x8 frame)
template <typename TYPE>
struct Spline64 {
	typedef TYPE Type;

	enum { halfWidth = 4 };
	enum { width = halfWidth*2 };

	inline Spline64() {}

	inline void operator () (const TYPE x, TYPE* const weigth) const {
		weigth[0] = ((TYPE(-1) / TYPE(41) * x + TYPE(168) / TYPE(2911)) * x - TYPE(97) / TYPE(2911)) * x;
		weigth[1] = ((TYPE(6) / TYPE(41) * x - TYPE(1008) / TYPE(2911)) * x +  TYPE(582) / TYPE(2911)) * x;
		weigth[2] = ((TYPE(-24) / TYPE(41) * x + TYPE(4032) / TYPE(2911)) * x - TYPE(2328) / TYPE(2911)) * x;
		weigth[3] = ((TYPE(49) / TYPE(41) * x - TYPE(6387) / TYPE(2911)) * x - TYPE(3) / TYPE(2911)) * x + TYPE(1);
		weigth[4] = ((TYPE(-49) / TYPE(41) * x + TYPE(4050) / TYPE(2911)) * x + TYPE(2340) / TYPE(2911)) * x;
		weigth[5] = ((TYPE(24) / TYPE(41) * x - TYPE(1080) / TYPE(2911)) * x - TYPE(624) / TYPE(2911)) * x;
		weigth[6] = ((TYPE(-6) / TYPE(41) * x + TYPE(270) / TYPE(2911)) * x + TYPE(156) / TYPE(2911)) * x;
		weigth[7] = ((TYPE(1) / TYPE(41) * x - TYPE(45) / TYPE(2911)) * x - TYPE(26) / TYPE(2911)) * x;
	}
};

// Sample image at a specified position
// @param image to be sampled
// @param sampler used to make the sampling
// @param pt X and Y-coordinate of sampling
// @return sampled value
template <typename IMAGE, typename SAMPLER, typename POINT, typename TYPE>
inline TYPE Sample(const IMAGE& image, const SAMPLER& sampler, const POINT& pt)
{
	typedef typename SAMPLER::Type T;

	// integer position of sample (x,y)
	const int grid_x(FLOOR2INT(pt.x));
	const int grid_y(FLOOR2INT(pt.y));

	// compute difference between exact pixel location and sample
	const T dx(pt.x-(T)grid_x);
	const T dy(pt.y-(T)grid_y);

	// get sampler weights
	T coefs_x[SAMPLER::width];
	sampler(dx, coefs_x);
	T coefs_y[SAMPLER::width];
	sampler(dy, coefs_y);

	// Sample a grid around specified grid point
	TYPE res(0);
	for (int i = 0; i < SAMPLER::width; ++i) {
		// get current i value
		// +1 for correct scheme (draw it to be convinced)
		const int cur_i(grid_y + 1 + i - SAMPLER::halfWidth);
		// handle out of range
		if (cur_i < 0 || cur_i >= image.rows)
			continue;
		for (int j = 0; j < SAMPLER::width; ++j) {
			// get current j value
			// +1 for the same reason
			const int cur_j(grid_x + 1 + j - SAMPLER::halfWidth);
			// handle out of range
			if (cur_j < 0 || cur_j >= image.cols)
				continue;
			// sample input image and weight according to sampler
			const T w = coefs_x[j] * coefs_y[i];
			const TYPE pixel = image(cur_i, cur_j);
			res += pixel * w;
		}
	}
	return res;
}

} // namespace Sampler
