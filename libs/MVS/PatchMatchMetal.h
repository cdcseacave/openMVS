/*
* PatchMatchMetal.h
*
* Metal compute backend for PatchMatch dense densification (Apple Silicon).
* Mirrors the public interface of MVS::CUDA::PatchMatch so SceneDensify can
* drive either backend through the same call shape.
*
* Pure-C++ PIMPL header: no Metal/Objective-C types leak here, so plain C++
* translation units (SceneDensify.cpp) can hold and call this class. The
* Objective-C++ implementation lives in PatchMatchMetal.mm.
*/

#ifndef _MVS_PATCHMATCHMETAL_H_
#define _MVS_PATCHMATCHMETAL_H_

#ifdef _USE_METAL

#include "SceneDensify.h"

namespace MVS {

namespace METAL {

class PatchMatch {
public:
	struct Params {
		int   nNumViews = 5;
		int   nEstimationIters = 3;
		float fDepthMin = 0.f;
		float fDepthMax = 100.f;
		int   nInitTopK = 3;
		bool  bGeomConsistency = false;
		bool  bLowResProcessed = false;
		float fThresholdKeepCost = 0;
	};

	PatchMatch();
	~PatchMatch();

	// returns true if a Metal device was found and pipelines were built
	bool IsValid() const;

	void Init(bool bGeomConsistency);
	void Release();

	void EstimateDepthMap(DepthData&);

	Params params;

private:
	struct Impl;
	Impl* impl;
};

} // namespace METAL

} // namespace MVS

#endif // _USE_METAL

#endif // _MVS_PATCHMATCHMETAL_H_
