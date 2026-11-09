/*
* PatchMatchMetal.h
*
* Copyright (c) 2014-2026 SEACAVE
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

/*
* Metal compute backend for PatchMatch dense densification (Apple Silicon) contributed by leNeo.
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

// the inputs and outputs of the shader's hand mirror of CameraModels.h and DepthGeometry.h evaluated
// on the device (PatchMatch::EvaluateModelMath); PatchMatchMetal.metal holds the same struct
struct ModelMathSample {
	// inputs
	float spherical[4];      // Spherical fx, fy, cx, cy
	float pinhole[4];        // Pinhole fx, fy, cx, cy
	float X[3];              // a camera-frame point
	float trig[4];           // sin and cos of a longitude, then of an elevation
	float pixel[2];          // a pinhole pixel
	float ray[3], nRay[3];   // InterpolatePlaneDepth rays
	float normal[3];         // InterpolatePlaneDepth plane normal
	float nDepth, dMin, dMax;
	float depth, gradient[2];// NormalFromDepthGradient at the ray frame of trig
	// outputs
	float fastAtan2;         // FastAtan2(X.y, X.x)
	float projected[2];      // Spherical::ProjectFast(X)
	float visible;           // its return value (0 or 1)
	float range;             // Spherical::Depth(X)
	float sphRay[3];         // Spherical::Ray(trig)
	float sphFrame[9];       // Spherical::RayFrame(trig), row-major
	float homography[9];     // Spherical::PixelHomography(X, projected, rho), row-major
	float pinFrame[9];       // Pinhole::RayFrame(pixel), row-major
	float pinProjected[2];   // Pinhole::Project(X)
	float pinVisible;        // its return value (0 or 1)
	float interpolated;      // InterpolatePlaneDepth(ray, nRay, nDepth, normal, dMin, dMax)
	float normalOut[3];      // NormalFromDepthGradient(sphFrame, depth, gradient)
};
static_assert(sizeof(ModelMathSample) == 74*sizeof(float), "ModelMathSample must match the shader's layout");

class MVS_API PatchMatch {
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

	// evaluate the shader's camera-model math on the given samples' inputs (TestModelMath kernel),
	// for the unit test against CameraModels.h; false if the kernel did not run
	bool EvaluateModelMath(std::vector<ModelMathSample>& samples) const;

	Params params;

private:
	struct Impl;
	Impl* impl;
};

} // namespace METAL

} // namespace MVS

#endif // _USE_METAL

#endif // _MVS_PATCHMATCHMETAL_H_
