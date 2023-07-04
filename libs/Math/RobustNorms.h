/*
* RobustNorms.h
*
* Copyright (c) 2014-2022 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#ifndef _SEACAVE_ROBUSTNORMS_H_
#define _SEACAVE_ROBUSTNORMS_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

// M-estimators to be used during nonlinear refinement for increased robustness to outlier residuals:
//   new_residual = robustNorm(residual)
namespace RobustNorm {

template<typename TYPE>
struct Identity {
	inline TYPE operator()(const TYPE& r) const {
		return r;
	}
};

template<typename TYPE>
struct L1 {
	inline TYPE operator()(const TYPE& r) const {
		return r / (std::sqrt(std::abs(r)) + TYPE(1e-12));
	}
};

template<typename TYPE>
struct Exp {
	inline Exp(const TYPE& _threshold)
		: sigma(TYPE(-1) / (TYPE(2) * SQUARE(_threshold))) { ASSERT(_threshold > 0); }
	inline TYPE operator()(const TYPE& r) const {
		return r * std::exp(SQUARE(r)*sigma);
	}
	const TYPE sigma;
};

template<typename TYPE>
struct BlakeZisserman {
	inline BlakeZisserman(const TYPE& _threshold)
		: threshold(_threshold), invThreshold(TYPE(1)/threshold), eps(std::exp(-SQUARE(threshold))) { ASSERT(_threshold > 0); }
	inline TYPE operator()(const TYPE& r) const {
		return -log(std::exp(-SQUARE(r) * invThreshold) + eps);
	}
	const TYPE threshold;
	const TYPE invThreshold;
	const TYPE eps;
};

template<typename TYPE>
struct Huber {
	inline Huber(const TYPE& _threshold)
		: threshold(_threshold) { ASSERT(_threshold > 0); }
	inline TYPE operator()(const TYPE& r) const {
		const TYPE dAbs(std::abs(r));
		if (dAbs <= threshold)
			return r;
		return std::copysign(std::sqrt(threshold * (TYPE(2) * dAbs - threshold)), r);
	}
	const TYPE threshold;
};

template<typename TYPE>
struct PseudoHuber {
	inline PseudoHuber(const TYPE& _threshold)
		: threshold(_threshold), thresholdSq(SQUARE(_threshold)), invThreshold(TYPE(1)/threshold) { ASSERT(_threshold > 0); }
	inline TYPE operator()(const TYPE& r) const {
		return std::sqrt(TYPE(2) * thresholdSq * (std::sqrt(TYPE(1) + SQUARE(r * invThreshold)) - TYPE(1)));
	}
	const TYPE threshold;
	const TYPE thresholdSq;
	const TYPE invThreshold;
};

template<typename TYPE>
struct GemanMcClure {
	inline GemanMcClure(const TYPE& _threshold)
		: thresholdSq(SQUARE(_threshold)) { ASSERT(_threshold > 0); }
	inline TYPE operator()(const TYPE& r) const {
		return r * std::sqrt(thresholdSq/(thresholdSq+SQUARE(r)));
	}
	const TYPE thresholdSq;
};

template<typename TYPE>
struct Cauchy {
	inline Cauchy(const TYPE& _threshold)
		: thresholdSq(SQUARE(_threshold)), invThresholdSq(TYPE(1)/thresholdSq) { ASSERT(_threshold > 0); }
	inline TYPE operator()(const TYPE& r) const {
		return std::sqrt(thresholdSq*log(TYPE(1)+SQUARE(r)*invThresholdSq));
	}
	const TYPE thresholdSq;
	const TYPE invThresholdSq;
};

template<typename TYPE>
struct Tukey {
	inline Tukey(const TYPE& _threshold)
		: threshold(_threshold), thresholdCb6(CUBE(_threshold)/TYPE(6)), invThreshold(TYPE(1)/threshold), sqrtThresholdCb6(std::sqrt(thresholdCb6)) { ASSERT(_threshold > 0); }
	inline TYPE operator()(const TYPE& r) const {
		return std::abs(r) < threshold ? std::sqrt(thresholdCb6*(TYPE(1)-CUBE(TYPE(1)-SQUARE(r*invThreshold)))) : sqrtThresholdCb6;
	}
	const TYPE threshold;
	const TYPE thresholdCb6;
	const TYPE invThreshold;
	const TYPE sqrtThresholdCb6;
};

} // namespace RobustNorm
/*----------------------------------------------------------------*/


// makes sure the inverse NCC score does not exceed 0.3
#if 0
template <typename T>
inline T robustincc(const T x) { return x / (T(1) + T(3) * x); }
template <typename T>
inline T robustinccg(const T x) { return T(1)/SQUARE(T(1) + T(3) * x); }
template <typename T>
inline T unrobustincc(const T y) { return y / (T(1) - T(3) * y); }
#elif 0
template <typename T>
inline T robustincc(const T x) { return T(1)-EXP(x*x*T(-4)); }
template <typename T>
inline T robustinccg(const T x) { return T(8)*x*EXP(x*x*T(-4)); }
template <typename T>
inline T unrobustincc(const T y) { return SQRT(-LOGN(T(1) - y))/T(2); }
#else
template <typename T>
inline T robustincc(const T x) { return x/SQRT(T(0.3)+x*x); }
template <typename T>
inline T robustinccg(const T x) { return T(0.3)/((T(0.3)+x*x)*SQRT(T(0.3)+x*x)); }
template <typename T>
inline T unrobustincc(const T y) { return (SQRT(30)*y)/(T(10)*SQRT(T(1) - y*y)); }
#endif
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // _SEACAVE_ROBUSTNORMS_H_
