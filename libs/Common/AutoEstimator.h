#ifndef __SEACAVE_AUTO_ESTIMATOR_H__
#define __SEACAVE_AUTO_ESTIMATOR_H__

//-------------------
// Generic implementation of ACRANSAC
//-------------------
// The A contrario parametrization have been first explained in [1] and
//  later extended to generic model estimation in [2] (with a demonstration for
//  the homography) and extended and use at large scale for Structure from
//  Motion in [3].
//
//--
//  [1] Lionel Moisan, Berenger Stival,
//  A probalistic criterion to detect rigid point matches between
//  two images and estimate the fundamental matrix.
//  IJCV 04.
//--
//  [2] Lionel Moisan, Pierre Moulon, Pascal Monasse.
//  Automatic Homographic Registration of a Pair of Images,
//    with A Contrario Elimination of Outliers
//  Image Processing On Line (IPOL), 2012.
//  http://dx.doi.org/10.5201/ipol.2012.mmm-oh
//--
//  [3] Pierre Moulon, Pascal Monasse and Renaud Marlet.
//  Adaptive Structure from Motion with a contrario mode estimation.
//  In 11th Asian Conference on Computer Vision (ACCV 2012)
//--
//  [4] cDc@seacave - rewrite it
//


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

// uncomment it to enable std::vector implementation
// (slower)
//#define ACRANSAC_STD_VECTOR

// uncomment it to enable sampling from inliers
// (sometimes does not find the right solution)
//#define ACRANSAC_SAMPLE_INLIERS


// P R O T O T Y P E S /////////////////////////////////////////////

namespace SEACAVE {

/// logarithm (base 10) of binomial coefficient
static inline double logcombi(size_t k, size_t n)
{
	if (k>=n) return(0.0);
	if (n-k<k) k=n-k;
	double r = 0.0;
	for (size_t i = 1; i <= k; i++)
		r += log10((double)(n-i+1))-log10((double)i);
	return r;
}

/// tabulate logcombi(.,n)
template<typename Type>
static inline void makelogcombi_n(size_t n, std::vector<Type>& l)
{
	l.resize(n+1);
	for (size_t k = 0; k <= n; ++k)
		l[k] = static_cast<Type>(logcombi(k,n));
}

/// tabulate logcombi(k,.)
template<typename Type, int k>
static inline void makelogcombi_k(size_t nmax, std::vector<Type>& l)
{
	l.resize(nmax+1);
	for (size_t n = 0; n <= nmax; ++n)
		l[n] = static_cast<Type>(logcombi(k,n));
}

/// Distance and associated index
struct ErrorIndex {
	double first;
	size_t second;
	inline ErrorIndex() {}
	inline ErrorIndex(double _first, size_t _second) : first(_first), second(_second) {}
	inline bool operator==(const ErrorIndex& r) const { return (first == r.first); }
	inline bool operator <(const ErrorIndex& r) const { return (first < r.first); }
};
#ifdef ACRANSAC_STD_VECTOR
typedef std::vector<ErrorIndex> ErrorIndexArr;
#else
typedef SEACAVE::cList<ErrorIndex,const ErrorIndex&,0> ErrorIndexArr;
#endif

/// Find best NFA and its index wrt square error threshold in e.
template<size_t NSAMPLES>
static ErrorIndex bestNFA(
	double logalpha0,
	const ErrorIndexArr& e,
	double loge0,
	double maxThresholdSq,
	const std::vector<float>& logc_n,
	const std::vector<float>& logc_k,
	double multError = 1.0)
{
	ErrorIndex bestIndex(DBL_MAX, NSAMPLES);
	const size_t n = e.size();
	for (size_t k=NSAMPLES+1; k<=n && e[k-1].first<=maxThresholdSq; ++k) {
		const double logalpha = logalpha0 + multError * log10(e[k-1].first+FLT_EPSILON);
		const double NFA = loge0 +
			logalpha * (double)(k-NSAMPLES) +
			logc_n[k] +
			logc_k[k];
		if (NFA < bestIndex.first)
			bestIndex = ErrorIndex(NFA, k);
	}
	return bestIndex;
}

/// Pick a random subset of the integers [0, total), in random order.
/// Note that this can behave badly if num_samples is close to total; runtime
/// could be unlimited!
///
/// This uses a quadratic rejection strategy and should only be used for small
/// num_samples.
///
/// \param NSAMPLES      The number of samples to produce.
/// \param n             The number of samples available.
/// \param samples       num_samples of numbers in [0, total_samples) is placed
///                      here on return.
struct UniformSampler {
	template <size_t NSAMPLES>
	inline void Sample(
		size_t n,
		size_t* samples)
	{
		ASSERT(NSAMPLES > 0);
		size_t* const samplesBegin = samples;
		size_t* const samplesEnd = samples + NSAMPLES;
		*samples = size_t(RAND() % n);
		while (++samples < samplesEnd) {
			do {
				*samples = size_t(RAND() % n);
			} while (std::find(samplesBegin, samples, *samples) != samples);
		}
	}
};
/// Same as above, but ensure that the first point is always included
struct UniformSamplerLockFirst {
	template <size_t NSAMPLES>
	inline void Sample(
		size_t n,
		size_t* samples)
	{
		ASSERT(NSAMPLES > 1);
		size_t* const samplesBegin = samples;
		size_t* const samplesEnd = samples + NSAMPLES;
		*samples++ = 0;
		do {
			do {
				*samples = size_t(RAND() % n);
			} while (std::find(samplesBegin, samples, *samples) != samples);
		} while (++samples < samplesEnd);
	}
};

/// Pick a random sample:
/// get a (sorted) random sample of size NSAMPLES in [0:n-1]
/// \param NSAMPLES      The number of samples to produce.
/// \param vec_index     The possible data indices.
/// \param n             The number of samples available.
/// \param samples       The random sample of NSAMPLES indices (output).
struct UniformSamplerSorted {
	template <size_t NSAMPLES>
	inline void Sample(
		#ifdef ACRANSAC_SAMPLE_INLIERS
		const std::vector<size_t>& vec_index,
		#else
		size_t n,
		#endif
		size_t* samples)
	{
		#ifdef ACRANSAC_SAMPLE_INLIERS
		const size_t n = vec_index.size();
		#endif
		for (size_t i=0; i<NSAMPLES; ++i) {
			size_t r = (RAND()>>3)%(n-i), j;
			for (j=0; j<i && r>=samples[j]; ++j)
				++r;
			size_t j0 = j;
			for (j=i; j>j0; --j)
				samples[j] = samples[j-1];
			samples[j0] = r;
		}
		#ifdef ACRANSAC_SAMPLE_INLIERS
		for (size_t i=0; i<NSAMPLES; ++i)
			samples[i] = vec_index[samples[i]];
		#endif
	}
};
/// Same as above, but ensure that the first point is always included
struct UniformSamplerSortedLockFirst {
	template <size_t NSAMPLES>
	inline void Sample(
		size_t n,
		size_t* samples)
	{
		ASSERT(NSAMPLES > 1);
		samples[0] = 0;
		for (size_t i=1; i<NSAMPLES; ++i) {
			size_t r = (RAND()>>3)%(n-i), j;
			for (j=0; j<i && r>=samples[j]; ++j)
				++r;
			size_t j0 = j;
			for (j=i; j>j0; --j)
				samples[j] = samples[j-1];
			samples[j0] = r;
		}
	}
};

/// Returns the requested matrix rows
template <typename TMat, typename TRows>
TMat ExtractRows(const TMat &A, const TRows &rows) {
	TMat compressed((int)rows.size(), A.cols);
	for (size_t i=0; i<static_cast<size_t>(rows.size()); ++i)
		A.row(rows[i]).copyTo(compressed.row((int)i));
	return compressed;
}

/// ACRANSAC routine (ErrorThreshold, NFA)
template <typename Kernel, typename Sampler>
std::pair<double, double> ACRANSAC(
	Kernel& kernel,
	Sampler& sampler,
	std::vector<size_t>& vec_inliers,
	typename Kernel::Model& model,
	double maxThreshold = DBL_MAX,
	#ifndef ACRANSAC_SAMPLE_INLIERS
	double confidence = 0.9999,
	#endif
	size_t nIter = 0)
{
	const size_t nData = kernel.NumSamples();
	if (nData < Kernel::MINIMUM_SAMPLES) {
		vec_inliers.clear();
		return std::make_pair(0.0,0.0);
	}
	const double maxThresholdSq = SQUARE(maxThreshold);
	std::vector<size_t> vec_sample(Kernel::MINIMUM_SAMPLES); // sample indices
	typename Kernel::Models vec_models; // up to max_models solutions
	if (nData == Kernel::MINIMUM_SAMPLES) {
		vec_inliers.resize(Kernel::MINIMUM_SAMPLES);
		std::iota(vec_inliers.begin(), vec_inliers.end(), 0);
		if (!kernel.Fit(vec_inliers, vec_models) || vec_models.size() != 1) {
			vec_inliers.clear();
			return std::make_pair(0.0,0.0);
		}
		model = vec_models[0];
		return std::make_pair(maxThresholdSq,0.0);
	}

	#ifdef ACRANSAC_SAMPLE_INLIERS
	// Possible sampling indices (could change in the optimization phase)
	std::vector<size_t> vec_index(nData);
	for (size_t i = 0; i < nData; ++i)
		vec_index[i] = i;
	#endif

	// Precompute log combi
	const double loge0 = log10((double)Kernel::MAX_MODELS * (nData-Kernel::MINIMUM_SAMPLES));
	std::vector<float> vec_logc_n, vec_logc_k;
	makelogcombi_n<float>(nData, vec_logc_n);
	makelogcombi_k<float, Kernel::MINIMUM_SAMPLES>(nData, vec_logc_k);

	// Output parameters
	double minNFA = DBL_MAX;
	double errorSqMax = DBL_MAX;

	// Reserve 10% of iterations for focused sampling
	if (nIter == 0)
		nIter = MINF((size_t)4000, MAXF((size_t)300, nData*3/2));
	const size_t nMinIter = nIter*8/100;

	// Main estimation loop
	ErrorIndexArr vec_residuals(nData); // [residual,index]
	for (size_t iter=0; iter<nIter || iter<nMinIter; ++iter) {
		// Get random sample
		sampler.template Sample<Kernel::MINIMUM_SAMPLES>(
			#ifdef ACRANSAC_SAMPLE_INLIERS
			vec_index,
			#else
			nData,
			#endif
			&vec_sample[0]
		);

		if (!kernel.Fit(vec_sample, vec_models))
			continue;

		// Evaluate models
		#ifdef ACRANSAC_SAMPLE_INLIERS
		bool better = false;
		#else
		const size_t nTrialsRound = nIter;
		#endif
		for (typename Kernel::Models::const_iterator itModel=vec_models.cbegin(); itModel!=vec_models.cend(); ++itModel)  {
			// Residuals computation and ordering
			kernel.EvaluateModel(*itModel);
			for (size_t i = 0; i < nData; ++i)
				vec_residuals[i] = ErrorIndex(kernel.Error(i), i);
			std::sort(vec_residuals.begin(), vec_residuals.end());

			// Most meaningful discrimination inliers/outliers
			ErrorIndex best = bestNFA<Kernel::MINIMUM_SAMPLES>(
				kernel.logalpha0(),
				vec_residuals,
				loge0,
				maxThresholdSq,
				vec_logc_n,
				vec_logc_k,
				kernel.multError());

			if (best.first < minNFA /*&& vec_residuals[best.second-1].first < errorSqMax*/)  {
				// A better model was found
				minNFA = best.first;
				vec_inliers.resize(best.second);
				for (size_t i=0; i<best.second; ++i)
					vec_inliers[i] = vec_residuals[i].second;
				errorSqMax = vec_residuals[best.second-1].first; // error threshold
				model = *itModel;
				DEBUG_LEVEL(4, "\titer=% 3u inliers=%u threshold=%g nfa=%g", iter, best.second, SQRT(errorSqMax), minNFA);
				#ifdef ACRANSAC_SAMPLE_INLIERS
				better = true;
				#else
				nIter = cvRANSACUpdateNumIters(confidence, (double)(nData - best.second)/nData, unsigned(Kernel::MINIMUM_SAMPLES), (unsigned)nTrialsRound);
				#endif
			}
		}
		vec_models.clear();

		#ifdef ACRANSAC_SAMPLE_INLIERS
		if (better && minNFA<0 && vec_inliers.size()>nData/3) {
			// Optimization: draw samples among best set of inliers so far
			vec_index = vec_inliers;
		}
		#endif
	}

	if (minNFA >= 0)
		vec_inliers.clear();

	return std::make_pair(errorSqMax, minNFA);
}
/*----------------------------------------------------------------*/


/// RANSAC routine (standard robust fitter, based on a known threshold)
template <typename Kernel, typename Sampler>
void RANSAC(
	Kernel& kernel,
	Sampler& sampler,
	std::vector<size_t>& vec_inliers,
	typename Kernel::Model& model,
	double threshold,
	#ifndef ACRANSAC_SAMPLE_INLIERS
	double confidence = 0.9999,
	#endif
	size_t nIter = 0)
{
	vec_inliers.clear();
	const size_t nData = kernel.NumSamples();
	if (nData < Kernel::MINIMUM_SAMPLES)
		return;
	const double thresholdSq = SQUARE(threshold);
	std::vector<size_t> vec_sample(Kernel::MINIMUM_SAMPLES); // sample indices
	typename Kernel::Models vec_models; // up to max_models solutions

	#ifdef ACRANSAC_SAMPLE_INLIERS
	// Possible sampling indices (could change in the optimization phase)
	std::vector<size_t> vec_index(nData);
	for (size_t i = 0; i < nData; ++i)
		vec_index[i] = i;
	#endif

	// Reserve 10% of iterations for focused sampling
	if (nIter == 0)
		nIter = MINF((size_t)4000, MAXF((size_t)300, nData*3/2));
	const size_t nMinIter = nIter*8/100;

	// Main estimation loop
	#ifdef _USE_MSAC
	double best_score = thresholdSq*nData;
	#endif
	std::vector<size_t> vec_inliers_tmp; vec_inliers_tmp.reserve(nData); vec_inliers.reserve(nData);
	for (size_t iter=0; iter<nIter || iter<nMinIter; ++iter) {
		// Get random sample
		sampler.template Sample<Kernel::MINIMUM_SAMPLES>(
			#ifdef ACRANSAC_SAMPLE_INLIERS
			vec_index,
			#else
			nData,
			#endif
			&vec_sample[0]
		);

		if (!kernel.Fit(vec_sample, vec_models))
			continue;

		// Evaluate models
		#ifdef ACRANSAC_SAMPLE_INLIERS
		bool better = false;
		#else
		const size_t nTrialsRound = nIter;
		#endif
		for (typename Kernel::Models::const_iterator itModel=vec_models.cbegin(); itModel!=vec_models.cend(); ++itModel)  {
			// Residuals computation and ordering
			#ifdef _USE_MSAC
			double score = 0;
			#endif
			vec_inliers_tmp.clear();
			kernel.EvaluateModel(*itModel);
			for (size_t i = 0; i < nData; ++i) {
				const double errSq = kernel.Error(i);
				if (errSq <= thresholdSq) {
					#ifdef _USE_MSAC
					score += errSq;
					#endif
					vec_inliers_tmp.push_back(i);
				} else {
					#ifdef _USE_MSAC
					score += thresholdSq;
					#endif
				}
				#ifdef _USE_MSAC
				if (score >= best_score)
					break;
				#endif
			}
			// Store best model
			#ifdef _USE_MSAC
			if (score < best_score) {
			#else
			if (vec_inliers_tmp.size() > vec_inliers.size()) {
			#endif
				// A better model was found
				model = *itModel;
				vec_inliers.swap(vec_inliers_tmp);
				if (vec_inliers.size() == nData) {
					nIter = 0; // force loop exit
					break;
				}
				#ifdef _USE_MSAC
				best_score = score;
				DEBUG_LEVEL(4, "\titer=%3u inliers=%u score=%g", iter, vec_inliers.size(), best_score);
				#else
				DEBUG_LEVEL(4, "\titer=%3u inliers=%u", iter, vec_inliers.size());
				#endif
				#ifdef ACRANSAC_SAMPLE_INLIERS
				better = true;
				#else
				#ifdef _USE_MSAC
				nIter = cvRANSACUpdateNumIters(confidence, (double)score/((double)nData*thresholdSq), unsigned(Kernel::MINIMUM_SAMPLES), (unsigned)nTrialsRound);
				#else
				nIter = cvRANSACUpdateNumIters(confidence, (double)(nData-vec_inliers.size())/nData, unsigned(Kernel::MINIMUM_SAMPLES), (unsigned)nTrialsRound);
				#endif
				#endif
			}
		}
		vec_models.clear();

		#ifdef ACRANSAC_SAMPLE_INLIERS
		if (better && vec_inliers.size()>nData/3) {
			// Optimization: draw samples among best set of inliers so far
			vec_index = vec_inliers;
		}
		#endif
	}

	if (vec_inliers.size() < Kernel::MINIMUM_SAMPLES)
		vec_inliers.clear();
}
/*----------------------------------------------------------------*/


/// Two view Kernel adaptator for the A contrario model estimator
/// Handle data normalization and compute the corresponding logalpha 0
///  that depends of the error model (point to line, or point to point)
/// This kernel adaptor is working for affine, homography, fundamental matrix
///  estimation.
template <typename SOLVER>
class ACKernelAdaptor
{
public:
	typedef SOLVER Solver;
	typedef typename SOLVER::Model Model;
	typedef typename SOLVER::Models Models;

	enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
	enum { MAX_MODELS = Solver::MAX_MODELS };

	ACKernelAdaptor(
		const DMatrix32F &x1, float w1, float h1,
		const DMatrix32F &x2, float w2, float h2,
		bool bPointToLine = true)
		: x1_(x1), x2_(x2),
		bPointToLine_(bPointToLine)
	{
		ASSERT(2 == x1_.cols);
		ASSERT(x1_.rows == x2_.rows);
		ASSERT(x1_.cols == x2_.cols);
		ASSERT(NumSamples() >= MINIMUM_SAMPLES);

		// LogAlpha0 is used to make error data scale invariant
		const float s2 = 1.0f;
		if (bPointToLine)  {
			// Ratio of containing diagonal image rectangle over image area
			const float D = sqrt(w2*w2 + h2*h2); // diameter
			const float A = w2*h2; // area
			logalpha0_ = log10(2.0f*D/A/s2);
		}
		else  {
			// ratio of area : unit circle over image area
			logalpha0_ = log10(M_PI/(w2*h2)/(s2*s2));
		}
	}

	inline bool Fit(const std::vector<size_t>& samples, Models& models) const {
		const DMatrix32F x1(ExtractRows(x1_, samples));
		if (CheckCollinearity(x1.ptr<const Point2f>(), x1.rows))
			return false;
		const DMatrix32F x2(ExtractRows(x2_, samples));
		if (CheckCollinearity(x2.ptr<const Point2f>(), x2.rows))
			return false;
		Solver::Solve(x1, x2, models);
		return true;
	}

	inline void EvaluateModel(const Model& model) {
		model2evaluate = model;
	}

	inline double Error(size_t sample) const {
		return Solver::Error(model2evaluate, *x1_.ptr<const Point2f>((int)sample), *x2_.ptr<const Point2f>((int)sample));
	}

	inline size_t NumSamples() const { return static_cast<size_t>(x1_.rows); }
	inline double logalpha0() const { return logalpha0_; }
	inline double multError() const { return (bPointToLine_ ? 0.5 : 1.0); }

protected:
	DMatrix32F x1_, x2_; // Normalized input data
	double logalpha0_; // Alpha0 is used to make the error adaptive to the image size
	bool bPointToLine_;// Store if error model is pointToLine or point to point
	Model model2evaluate; // current model to be evaluated
};

/// Pose/Resection Kernel adaptator for the A contrario model estimator
template <typename SOLVER>
class ACKernelAdaptorResection
{
public:
	typedef SOLVER Solver;
	typedef typename SOLVER::Model Model;
	typedef typename SOLVER::Models Models;

	enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
	enum { MAX_MODELS = Solver::MAX_MODELS };

	ACKernelAdaptorResection(const DMatrix32F &x2d, const DMatrix &x3D)
		: x2d_(x2d), x3D_(x3D),
		logalpha0_(log10(M_PI))
	{
		ASSERT(2 == x2d_.cols);
		ASSERT(3 == x3D_.cols);
		ASSERT(x2d_.rows == x3D_.rows);
		ASSERT(NumSamples() >= MINIMUM_SAMPLES);
	}

	inline bool Fit(const std::vector<size_t>& samples, Models& models) const {
		const DMatrix32F x1(ExtractRows(x2d_, samples));
		if (CheckCollinearity(x1.ptr<const Point2f>(), x1.rows))
			return false;
		const DMatrix x2(ExtractRows(x3D_, samples));
		Solver::Solve(x1, x2, models);
		return true;
	}

	inline void EvaluateModel(const Model &model) {
		model2evaluate = model;
	}

	inline double Error(size_t sample) const {
		return Solver::Error(model2evaluate, *x2d_.ptr<const Point2f>((int)sample), *x3D_.ptr<const Point3>((int)sample));
	}

	inline size_t NumSamples() const { return x2d_.rows; }
	inline double logalpha0() const { return logalpha0_; }
	inline double multError() const { return 1.0; } // point to point error

protected:
	DMatrix32F x2d_;
	DMatrix x3D_;
	double logalpha0_; // Alpha0 is used to make the error adaptive to the image size
	Model model2evaluate; // current model to be evaluated
};

/// Essential matrix Kernel adaptator for the A contrario model estimator
template <typename SOLVER>
class ACKernelAdaptorEssential
{
public:
	typedef SOLVER Solver;
	typedef typename SOLVER::Model Model;
	typedef typename SOLVER::Models Models;

	enum { MINIMUM_SAMPLES = Solver::MINIMUM_SAMPLES };
	enum { MAX_MODELS = Solver::MAX_MODELS };

	ACKernelAdaptorEssential(
		const DMatrix32F &x1, float w1, float h1, const Matrix3x3f& invK1,
		const DMatrix32F &x2, float w2, float h2, const Matrix3x3f& invK2)
		: solver(invK1, invK2),
		x1_(x1), x2_(x2)
	{
		ASSERT(2 == x1_.cols);
		ASSERT(x1_.rows == x2_.rows);
		ASSERT(x1_.cols == x2_.cols);
		ASSERT(NumSamples() >= MINIMUM_SAMPLES);

		//Point to line probability (line is the epipolar line)
		const float D = sqrt(w2*w2 + h2*h2); // diameter
		const float A = w2*h2+1.f; // area
		logalpha0_ = log10(2.0f*D/A*0.5f);
	}

	inline bool Fit(const std::vector<size_t>& samples, Models& models) const {
		const DMatrix32F x1(ExtractRows(x1_, samples));
		if (CheckCollinearity(x1.ptr<const Point2f>(), x1.rows))
			return false;
		const DMatrix32F x2(ExtractRows(x2_, samples));
		if (CheckCollinearity(x2.ptr<const Point2f>(), x2.rows))
			return false;
		solver.Solve(x1, x2, models);
		return true;
	}

	inline void EvaluateModel(const Model &model) {
		solver.EvaluateModel(model);
	}

	inline double Error(size_t sample) const {
		return solver.Error(*x1_.ptr<const Point2f>((int)sample), *x2_.ptr<const Point2f>((int)sample));
	}

	inline size_t NumSamples() const { return x1_.rows; }
	inline double logalpha0() const { return logalpha0_; }
	inline double multError() const { return 0.5; } // point to line error

protected:
	Solver solver;
	DMatrix32F x1_, x2_; // image point and camera plane point.
	double logalpha0_; // Alpha0 is used to make the error adaptive to the image size
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_AUTO_ESTIMATOR_H__
