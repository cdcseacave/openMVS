////////////////////////////////////////////////////////////////////
// Random.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_RANDOM_H__
#define __SEACAVE_RANDOM_H__


// I N C L U D E S /////////////////////////////////////////////////



// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Stateless random number generation
// uniform random number generation
FORCEINLINE float random() {
	return RANDOM<float>();
}
FORCEINLINE double randomd() {
	return RANDOM<double>();
}
FORCEINLINE long double randomld() {
	return RANDOM<long double>();
}
STATIC_ASSERT(RAND_MAX < 2147483648); // integer randomRange assumes this is capped
template<typename T>
FORCEINLINE T randomRange(T nMin, T nMax) {
	ASSERT(nMin <= nMax && nMax-nMin+1 < 8589934596); // not to overflow a uint64_t
	return nMin + T((uint64_t(nMax-nMin)*RAND()+RAND_MAX/2)/RAND_MAX);
}
template<>
FORCEINLINE float randomRange<float>(float fMin, float fMax) {
	return fMin + (fMax - fMin) * random();
}
template<>
FORCEINLINE double randomRange<double>(double fMin, double fMax) {
	return fMin + (fMax - fMin) * randomd();
}
template<>
FORCEINLINE long double randomRange<long double>(long double fMin, long double fMax) {
	return fMin + (fMax - fMin) * randomld();
}
template<typename T>
FORCEINLINE T randomMeanRange(T mean, T delta/*=(max-min)/2*/) {
	ASSERT(delta >= 0 && delta*2+1 < 8589934596); // not to overflow a uint64_t
	return (mean + T((uint64_t(delta)*2*RAND()+RAND_MAX/2)/RAND_MAX)) - delta;
}
template<>
FORCEINLINE float randomMeanRange<float>(float mean, float delta/*=(max-min)/2*/) {
	return mean + delta * (2.f * random() - 1.f);
}
template<>
FORCEINLINE double randomMeanRange<double>(double mean, double delta/*=(max-min)/2*/) {
	return mean + delta * (2.0 * randomd() - 1.0);
}
template<>
FORCEINLINE long double randomMeanRange<long double>(long double mean, long double delta/*=(max-min)/2*/) {
	return mean + delta * (2.0L * randomld() - 1.0L);
}
// gaussian random number generation
template<typename T>
FORCEINLINE T gaussian(T val, T sigma) {
	return EXP(-SQUARE(val/sigma)/2)/(SQRT(T(M_PI*2))*sigma);
}
template<typename T>
FORCEINLINE T randomGaussian(T mean, T sigma) {
	T x, y, r2;
	do {
		x = T(-1) + T(2) * RANDOM<T>();
		y = T(-1) + T(2) * RANDOM<T>();
		r2 = x * x + y * y;
	} while (r2 > T(1) || r2 == T(0));
	return mean + sigma * y * SQRT(T(-2) * LOGN(r2) / r2);
}
template<typename T>
FORCEINLINE T randomGaussian(T sigma) {
	return randomGaussian(T(0), sigma);
}
FORCEINLINE float randomGaussian() {
	return randomGaussian(0.f, 1.f);
}
FORCEINLINE double randomGaussiand() {
	return randomGaussian(0.0, 1.0);
}
FORCEINLINE long double randomGaussianld() {
	return randomGaussian(0.0L, 1.0L);
}
/*----------------------------------------------------------------*/


// Encapsulates state for random number generation
// based on C++11 random number generator functionality
struct Random : std::mt19937 {
	typedef std::mt19937 generator_type;

	Random() : generator_type(std::random_device()()) {}
	Random(result_type seed) : generator_type(seed) {}

	// integer randomRange assumes this is capped
	STATIC_ASSERT(max() < 4294967296);

	// returns a uniform random number in the range [0, 1]
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_floating_point<T>::value, T>::type random() {
		return (T)random()/(T)max();
	}
	// returns a uniform random number in the range [0, max()]
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_integral<T>::value, T>::type random() {
		return (T)operator()();
	}

	// returns a uniform random number in the range [nMin, nMax]
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_floating_point<T>::value, T>::type randomRange(T nMin, T nMax) {
		return nMin + (nMax-nMin) * random<T>();
	}
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_integral<T>::value, T>::type randomRange(T nMin, T nMax) {
		ASSERT(nMin <= nMax && nMax-nMin+1 < 4294967297); // not to overflow a uint64_t
		return nMin + (T)(((uint64_t)(nMax-nMin) * random() + max()/2)/max());
	}

	// returns a uniform random number in the range [mean-delta, mean+delta]
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_floating_point<T>::value, T>::type randomMeanRange(T mean, T delta/*=(max-min)/2*/) {
		return mean + delta * (T(2) * random<T>() - T(1));
	}
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_integral<T>::value, T>::type randomMeanRange(T mean, T delta/*=(max-min)/2*/) {
		ASSERT(delta >= 0 && delta*T(2)+1 < 4294967297); // not to overflow a uint64_t
		return mean + (T)(((uint64_t)delta*2 * random() + max()/2)/max()) - delta;
	}

	// returns a uniform random number in the range [nMin, nMax] using std implementation
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_floating_point<T>::value, T>::type randomUniform(T nMin, T nMax) {
		return std::uniform_real_distribution<T>(nMin, nMax)(*this);
	}
	template<typename T=result_type>
	FORCEINLINE typename std::enable_if<std::is_integral<T>::value, T>::type randomUniform(T nMin, T nMax) {
		return std::uniform_int_distribution<T>(nMin, nMax)(*this);
	}

	// returns a gaussian random number using std implementation
	template<typename T=result_type>
	FORCEINLINE T randomGaussian(T mean, T stddev) {
		return std::normal_distribution<T>(mean, stddev)(*this);
	}
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_RANDOM_H__
