/*
* SemiGlobalMatcher.cpp
*
* Copyright (c) 2014-2015 SEACAVE
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

#include "Common.h"
#include "SemiGlobalMatcher.h"
#include "Scene.h"

using namespace MVS;

using namespace STEREO;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("SemGblMt"));






// S T R U C T S ///////////////////////////////////////////////////

enum EVENT_TYPE {
	EVT_JOB = 0,
	EVT_CLOSE,
};

class EVTClose : public Event
{
public:
	EVTClose() : Event(EVT_CLOSE) {}
};
class EVTPixelProcess : public Event
{
public:
	typedef std::function<void (int,int,int)> FncPixel;
	const cv::Size size;
	volatile Thread::safe_t& idxPixel;
	const FncPixel fncPixel;
	bool Run(void*) override {
		const int numPixels(size.area());
		int idx;
		while ((idx=(int)Thread::safeInc(idxPixel)) < numPixels)
			fncPixel(idx, idx/size.width, idx%size.width);
		return true;
	}
	EVTPixelProcess(cv::Size s, volatile Thread::safe_t& idx, FncPixel f) : Event(EVT_JOB), size(s), idxPixel(idx), fncPixel(f) {}
};
class EVTPixelAccumInc : public Event
{
public:
	typedef std::function<void (int)> FncPixel;
	const int numPixels;
	volatile Thread::safe_t& idxPixel;
	const FncPixel fncPixel;
	bool Run(void*) override {
		int idx;
		while ((idx=(int)Thread::safeInc(idxPixel)) < numPixels)
			fncPixel(idx);
		return true;
	}
	EVTPixelAccumInc(int s, volatile Thread::safe_t& idx, FncPixel f) : Event(EVT_JOB), numPixels(s), idxPixel(idx), fncPixel(f) {}
};
class EVTPixelAccumDec : public Event
{
public:
	typedef std::function<void (int)> FncPixel;
	volatile Thread::safe_t& idxPixel;
	const FncPixel fncPixel;
	bool Run(void*) override {
		int idx;
		while ((idx=(int)Thread::safeDec(idxPixel)) >= 0)
			fncPixel(idx);
		return true;
	}
	EVTPixelAccumDec(volatile Thread::safe_t& idx, FncPixel f) : Event(EVT_JOB), idxPixel(idx), fncPixel(f) {}
};
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

// - P1 and P2s are the smoothness penalties of the original SGM algorithm, on the scale of the
//   matching cost (0-255 per pixel and disparity)
// - alpha & beta form the final P2 as P2*(1+alpha*e^(-DI^2/(2*beta^2)))
//   where DI is the difference in image intensity I(x)-I(x_prev) in [0,255] range
// - subpixelSteps represents how much sub-pixel accuracy is searched/stored;
//   if 1 no sub-pixel precision, if for example 4 a 0.25 sub-pixel accuracy is stored;
//   the stored value is quantized and represented as integer: val=(float)valStored/subpixelSteps
SemiGlobalMatcher::SemiGlobalMatcher(SgmSubpixelMode _subpixelMode, Disparity _subpixelSteps, AccumCost _P1, AccumCost P2, float P2alpha, float P2beta)
	:
	subpixelMode(_subpixelMode),
	subpixelSteps(_subpixelSteps),
	P1(_P1), P2s(GenerateP2s(P2, P2alpha, P2beta))
{
}

SemiGlobalMatcher::~SemiGlobalMatcher()
{
}

CLISTDEF0IDX(SemiGlobalMatcher::AccumCost,int) SemiGlobalMatcher::GenerateP2s(AccumCost P2, float P2alpha, float P2beta)
{
	CLISTDEF0IDX(AccumCost,int) P2s(256);
	FOREACH(i, P2s)
		P2s[i] = (AccumCost)ROUND2INT(P2*(1.f+P2alpha*EXP(-SQUARE((float)i)/(2.f*SQUARE(P2beta)))));
	return P2s;
}


// Depth range of the sparse points seen by the image, widened by 10% on both sides;
// optionally collect those points; return false if the image sees no point in front of it
static bool SparseDepthRange(const Scene& scene, IIndex idxImage, Depth& dMin, Depth& dMax, IndexArr* points=NULL)
{
	const Camera& camera = scene.images[idxImage].camera;
	dMin = FLT_MAX; dMax = 0;
	FOREACH(idxPoint, scene.pointcloud.points) {
		if (scene.pointcloud.pointViews[idxPoint].FindFirst(idxImage) == PointCloud::ViewArr::NO_INDEX)
			continue;
		const Depth depth((Depth)camera.PointDepth(scene.pointcloud.points[idxPoint]));
		if (depth <= 0)
			continue;
		if (dMin > depth)
			dMin = depth;
		if (dMax < depth)
			dMax = depth;
		if (points)
			points->push_back((uint32_t)idxPoint);
	}
	if (dMin >= dMax)
		return false;
	dMin *= 0.9f; dMax *= 1.1f;
	return true;
}

// Compute SGM stereo for this image and each of the neighbor views:
//  - minResolution is the resolution of the top of the pyramid for tSGM;
//    can be 0 to force the standard SGM algorithm
void SemiGlobalMatcher::Match(const Scene& scene, IIndex idxImage, IIndex numNeighbors, unsigned minResolution)
{
	const Image& leftImage = scene.images[idxImage];
	// the points seen by the left image bound its depth range, and hence the disparities searched
	// by the coarsest level, and locate the region each pair has in common
	Depth dMin, dMax;
	IndexArr points;
	if (!SparseDepthRange(scene, idxImage, dMin, dMax, &points))
		return;
	const float fMinScore(MAXF(leftImage.neighbors.front().score*OPTDENSE::fViewMinScoreRatio, OPTDENSE::fViewMinScore));
	FOREACH(idxNeighbor, leftImage.neighbors) {
		const ViewScore& neighbor = leftImage.neighbors[idxNeighbor];
		// exclude neighbors that over the limit or too small score
		ASSERT(scene.images[neighbor.ID].IsValid());
		if ((numNeighbors && idxNeighbor >= numNeighbors) ||
			(neighbor.score < fMinScore))
			break;
		// check if the disparity-map was already estimated for the same image pairs
		const Image& rightImage = scene.images[neighbor.ID];
		const String pairName(MAKE_PATH(String::FormatString("%04u_%04u", leftImage.ID, rightImage.ID)));
		if (File::isPresent((pairName+".dimap").c_str()) || File::isPresent(MAKE_PATH(String::FormatString("%04u_%04u.dimap", rightImage.ID, leftImage.ID))))
			continue;
		TD_TIMER_STARTD();
		Matrix3x3 H; Matrix4x4 Q;
		ViewData leftData, rightData;
		MaskMap leftMaskMap, rightMaskMap; {
		// stereo-rectify the image pair around the projections of the points it has in common
		Point3fArr leftPoints, rightPoints;
		for (uint32_t idxPoint: points) {
			if (scene.pointcloud.pointViews[idxPoint].FindFirst(neighbor.ID) == PointCloud::ViewArr::NO_INDEX)
				continue;
			const Point3 X(scene.pointcloud.points[idxPoint]);
			leftPoints.emplace_back(leftImage.camera.TransformPointW2I3(X));
			rightPoints.emplace_back(rightImage.camera.TransformPointW2I3(X));
		}
		if (leftPoints.empty() || !Image::StereoRectifyImages(leftImage, rightImage, leftPoints, rightPoints, leftData.imageColor, rightData.imageColor, leftMaskMap, rightMaskMap, H, Q))
			continue;
		ASSERT(leftData.imageColor.size() == rightData.imageColor.size());
		}
		// color to gray conversion
		#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
		leftData.imageColor.toGray(leftData.imageGray, cv::COLOR_BGR2GRAY, false, true);
		rightData.imageColor.toGray(rightData.imageGray, cv::COLOR_BGR2GRAY, false, true);
		#else
		leftData.imageColor.toGray(leftData.imageGray, cv::COLOR_BGR2GRAY, true, true);
		rightData.imageColor.toGray(rightData.imageGray, cv::COLOR_BGR2GRAY, true, true);
		#endif
		// compute scale used for the disparity estimation
		REAL scale(1);
		if (minResolution) {
			unsigned resolutionLevel(8);
			Image8U::computeMaxResolution(leftData.imageGray.width(), leftData.imageGray.height(), resolutionLevel, minResolution);
			scale = REAL(1)/MAXF(2,POWI(2,resolutionLevel));
		}
		// match the pair coarse to fine: the coarsest level searches every disparity the depth range
		// of the left image spans, the finer ones only around the disparities of the previous level
		DisparityMap leftDisparityMap, rightDisparityMap; AccumCostMap costMap;
		bool bValidMatch(true);
		do {
			// initialize
			const ViewData leftDataLevel(leftData.GetImage(scale));
			const ViewData rightDataLevel(rightData.GetImage(scale));
			const cv::Size size(leftDataLevel.imageGray.size());
			const cv::Size sizeValid(size.width-2*halfWindowSizeX, size.height-2*halfWindowSizeY);
			const bool bFirstLevel(leftDisparityMap.empty());
			Range range;
			Index numCosts;
			if (bFirstLevel) {
				// resize masks
				cv::resize(leftMaskMap, leftMaskMap, size, 0, 0, cv::INTER_NEAREST);
				cv::resize(rightMaskMap, rightMaskMap, size, 0, 0, cv::INTER_NEAREST);
				const cv::Rect ROI(halfWindowSizeX,halfWindowSizeY, sizeValid.width,sizeValid.height);
				leftMaskMap(ROI).copyTo(leftMaskMap);
				rightMaskMap(ROI).copyTo(rightMaskMap);
				range = DepthRange2Disparity(H, Q, scale, leftMaskMap, dMin, dMax);
				numCosts = range.isValid() ? Range2RangeMap(rightMaskMap, Range{(Disparity)-range.maxDisp, (Disparity)-range.minDisp}) : 0;
			} else {
				// upscale the masks and the disparity-map from the previous level
				UpscaleMask(leftMaskMap, sizeValid);
				UpscaleMask(rightMaskMap, sizeValid);
				FlipDirection(leftDisparityMap, rightDisparityMap);
				numCosts = Disparity2RangeMap(rightDisparityMap, rightMaskMap);
			}
			// estimate right-left disparity-map
			if (numCosts == 0) {
				bValidMatch = false;
				break;
			}
			imageCosts.resize(numCosts);
			imageAccumCosts.resize(numCosts);
			Match(rightDataLevel, leftDataLevel, rightDisparityMap, costMap);
			// estimate left-right disparity-map
			numCosts = bFirstLevel ? Range2RangeMap(leftMaskMap, range) : Disparity2RangeMap(leftDisparityMap, leftMaskMap);
			if (numCosts == 0) {
				bValidMatch = false;
				break;
			}
			imageCosts.resize(numCosts);
			imageAccumCosts.resize(numCosts);
			Match(leftDataLevel, rightDataLevel, leftDisparityMap, costMap);
			// check disparity-map cross-consistency
			ConsistencyCrossCheck(leftDisparityMap, rightDisparityMap);
			if (bFirstLevel) {
				// filter the coarsest disparity-maps rigorously, as they set the validity masks of the next levels
				ConsistencyCrossCheck(rightDisparityMap, leftDisparityMap);
				cv::filterSpeckles(leftDisparityMap, NO_DISP, OPTDENSE::nSpeckleSize, 5);
				cv::filterSpeckles(rightDisparityMap, NO_DISP, OPTDENSE::nSpeckleSize, 5);
				ExtractMask(leftDisparityMap, leftMaskMap);
				ExtractMask(rightDisparityMap, rightMaskMap);
			}
		} while ((scale*=2) < REAL(1)+ZEROTOLERANCE<REAL>());
		if (!bValidMatch)
			continue;
		// sub-pixel disparity-map estimation
		RefineDisparityMap(leftDisparityMap);
		// export disparity-map for the left image
		DEBUG_EXTRA("Disparity-map for images %3u and %3u: %dx%d (%s)", leftImage.ID, rightImage.ID,
			leftImage.width, leftImage.height, TD_TIMER_GET_FMT().c_str());
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2) {
			ExportPointCloud(pairName+".ply", leftImage, leftDisparityMap, Q, subpixelSteps);
			ExportDisparityMap(pairName+".png", leftDisparityMap);
		}
		#endif
		ExportDisparityDataRawFull(pairName+".dimap", leftDisparityMap, costMap, leftImage.GetSize(), H, Q, subpixelSteps);
	}
}

// the R x R texels of a patch as structures of arrays, so the per-texel arithmetic of the matching
// cost runs on SIMD lanes: their projective positions in a neighbor view relative to the patch
// center, and the weights of the reference patch (see InitWeightedPatch)
template <int R>
struct TexelPositions {
	enum { N = R*R };
	typedef Eigen::Array<float,N,1,Eigen::DontAlign> Texels;
	Texels x, y, z;
	// the positions over the plane whose inverse depth changes by slant at each texel, from these
	// positions over the fronto-parallel plane and the view's inverse-depth direction b
	void Warp(const TexelPositions& offsets, const Point3f& b, const Texels& slant) {
		x = offsets.x + b.x*slant;
		y = offsets.y + b.y*slant;
		z = offsets.z + b.z*slant;
	}
	// the open interval of inverse depths invz at which the patch at the positions hx+b*invz+these is
	// in front of the view and inside [0,width1)x[0,height1): the patch is convex under the homography,
	// so its corners bound it, and every bound on a corner is linear in invz
	std::pair<float,float> InsideRange(const Point3f& hx, const Point3f& b, float width1, float height1) const {
		float lo(-FLT_MAX), hi(FLT_MAX);
		const auto bound = [&](float alpha, float beta) { // alpha+beta*invz > 0
			if (beta > 0)
				lo = MAXF(lo, -alpha/beta);
			else if (beta < 0)
				hi = MINF(hi, -alpha/beta);
			else if (alpha <= 0)
				hi = -FLT_MAX;
		};
		for (const int n: {0, R-1, N-R, N-1}) {
			const Point3f h(hx.x+x[n], hx.y+y[n], hx.z+z[n]);
			bound(h.z, b.z);
			bound(h.x, b.x);
			bound(h.y, b.y);
			bound(width1*h.z-h.x, width1*b.z-b.x);
			bound(height1*h.z-h.y, height1*b.z-b.y);
		}
		return {lo, hi};
	}
};
template <int R>
struct TexelPatch {
	enum { N = R*R };
	Eigen::Array<float,N,1> weights, tempWeights;
	float sumWeights, normSq0;
	template <typename WeightedPatch>
	explicit TexelPatch(const WeightedPatch& w) : sumWeights(w.sumWeights), normSq0(w.normSq0) {
		for (int n=0; n<N; ++n) {
			weights[n] = w.weights[n].weight;
			tempWeights[n] = w.weights[n].tempWeight;
		}
	}
};

// WZNCC cost in [0,255] of a reference patch against a gray image sampled bilinearly at the projective
// positions h0+warp of its texels, which must be in front of the view and inside it (InsideRange)
template <int R>
static float TexelsCost(const Image32F& gray, const TexelPositions<R>& warp, const Point3f& h0, const TexelPatch<R>& ref)
{
	enum { N = R*R };
	typedef Eigen::Array<float,N,1> Texels;
	const Texels iz((warp.z+h0.z).inverse());
	const Texels u((warp.x+h0.x)*iz), v((warp.y+h0.y)*iz);
	const Eigen::Array<int,N,1> iu(u.template cast<int>()), iv(v.template cast<int>());
	const Texels fu(u-iu.template cast<float>()), fv(v-iv.template cast<float>());
	Texels f;
	const size_t stride(gray.step1());
	const float* const data(gray.ptr<const float>());
	for (int n=0; n<N; ++n) {
		const float* const p(data+(size_t)iv[n]*stride+iu[n]);
		const float top(p[0]+(p[1]-p[0])*fu[n]), bottom(p[stride]+(p[stride+1]-p[stride])*fu[n]);
		f[n] = top+(bottom-top)*fv[n];
	}
	const Texels fw(f*ref.weights);
	const float sum(fw.sum()), sumSq((f*fw).sum()), nom((f*ref.tempWeights).sum());
	const float normSq1(sumSq-SQUARE(sum)/ref.sumWeights);
	const float nrmSq(ref.normSq0*normSq1);
	const float ncc(nrmSq <= 1e-16f ? 0.f : nom/SQRT(nrmSq));
	return ncc <= 0 ? 255.f : (1.f-MINF(ncc,1.f))*255.f;
}

// Estimate the depth-map of an image by matching it against all its neighbor views at once:
//  - the disparities are inverse-depth samples, uniform and spaced such that one step moves the
//    projection by at most one pixel in every neighbor view, so the coarse-to-fine ranges, the
//    aggregation and the sub-pixel refinement of a rectified pair apply unchanged
//  - the cost of a pixel and sample is the mean of its two best neighbor costs, so a neighbor that
//    does not see the pixel or sees it occluded does not veto it; a neighbor cost is the WZNCC of the
//    reference patch and the neighbor patch warped by the plane through the sample
//  - that plane has the slant of the surface estimated by the previous level (inverse depth is
//    affine in the pixel coordinates over a plane); it is fronto-parallel at the coarsest level
void SemiGlobalMatcher::MatchMultiView(const Scene& scene, IIndex idxImage, IIndex numNeighbors, DepthMap& depthMap, ConfidenceMap& confMap, unsigned minResolution)
{
	const Image& refImage = scene.images[idxImage];
	const cv::Size imageSize(refImage.image.size());
	depthMap.create(imageSize); depthMap.memset(0);
	confMap.create(imageSize); confMap.memset(0);
	#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
	ASSERT("the multi-view matching needs the WZNCC similarity" == NULL);
	#else
	enum { maxViews = 32 }; // neighbor views matched at most
	enum { numBestViews = 2 }; // neighbor costs averaged per sample
	enum { texelStep = 2 }; // every other texel of the patch is matched, as accurate as all of them
	enum { rowTexels = halfWindowSizeX*2/texelStep+1 };
	static_assert(halfWindowSizeX == halfWindowSizeY, "the texels form a square patch");
	TD_TIMER_STARTD();
	// the points seen by the image bound its depth range
	Depth dMin, dMax;
	if (!SparseDepthRange(scene, idxImage, dMin, dMax))
		return;
	const float invzMin(1.f/dMax), invzMax(1.f/dMin);
	// neighbor views: x_k ~ A*x + invz*b maps the reference pixel x at inverse depth invz into view k
	struct NeighborView {
		const Image* image;
		cv::Matx33d R; cv::Vec3d t; // pose relative to the reference camera
		ImageGray grayFull, gray; // full resolution and current level intensities
		Point3f A[3], b; // A stored by columns, at the current level
		TexelPositions<rowTexels> offsets; // A*(j,i,0) for each matched texel
		TexelPositions<windowSizeX> offsetsDense; // A*(j,i,0) for each texel of the window
		float width1, height1; // largest valid sampling position
		void SetLevel(const cv::Matx33d& invKref, const cv::Matx33d& K) {
			const cv::Matx33d _A(K * R * invKref);
			const cv::Vec3d _b(K * t);
			for (int j=0; j<3; ++j)
				A[j] = Point3f((float)_A(0,j), (float)_A(1,j), (float)_A(2,j));
			b = Point3f((float)_b[0], (float)_b[1], (float)_b[2]);
			const auto setOffsets = [this](auto& offsets, int tStep) {
				int n(0);
				for (int i=-halfWindowSizeY; i<=halfWindowSizeY; i+=tStep) {
					for (int j=-halfWindowSizeX; j<=halfWindowSizeX; j+=tStep, ++n) {
						const Point3f o(A[0]*(float)j + A[1]*(float)i);
						offsets.x[n] = o.x; offsets.y[n] = o.y; offsets.z[n] = o.z;
					}
				}
			};
			setOffsets(offsets, texelStep);
			setOffsets(offsetsDense, 1);
			width1 = (float)gray.width()-1.001f;
			height1 = (float)gray.height()-1.001f;
		}
	};
	CLISTDEFIDX(NeighborView,IIndex) views;
	views.reserve(maxViews); // never relocated, as the images hold pointers into themselves
	const float fMinScore(MAXF(refImage.neighbors.front().score*OPTDENSE::fViewMinScoreRatio, OPTDENSE::fViewMinScore));
	FOREACH(idxNeighbor, refImage.neighbors) {
		const ViewScore& neighbor = refImage.neighbors[idxNeighbor];
		ASSERT(scene.images[neighbor.ID].IsValid());
		if ((numNeighbors && idxNeighbor >= numNeighbors) || neighbor.score < fMinScore || views.size() >= maxViews)
			break;
		NeighborView& view = views.emplace_back();
		view.image = &scene.images[neighbor.ID];
		const cv::Matx33d R(view.image->camera.R);
		const cv::Point3d dC(refImage.camera.C - view.image->camera.C);
		view.R = R * cv::Matx33d(refImage.camera.R).t();
		view.t = R * cv::Vec3d(dC.x, dC.y, dC.z);
		view.image->image.toGray(view.grayFull, cv::COLOR_BGR2GRAY, true, true);
	}
	if (views.empty())
		return;
	// inverse-depth step at full resolution: one pixel of motion along the epipolar line of the
	// neighbor where the projection moves the most, measured at the image center and mid depth
	float step0(0); {
		const cv::Matx33d invKref(MVS::Camera::InvK(refImage.camera.K));
		const Point3f x((float)imageSize.width*0.5f, (float)imageSize.height*0.5f, 1.f);
		const float invzMid((invzMin+invzMax)*0.5f);
		float maxMotion(0);
		for (NeighborView& view: views) {
			view.gray = view.grayFull;
			view.SetLevel(invKref, cv::Matx33d(view.image->camera.K));
			const Point3f h(view.A[0]*x.x + view.A[1]*x.y + view.A[2] + view.b*invzMid);
			if (h.z <= 0)
				continue;
			const float motion((float)norm(Point2f(view.b.x*h.z-h.x*view.b.z, view.b.y*h.z-h.y*view.b.z))/SQUARE(h.z));
			if (maxMotion < motion)
				maxMotion = motion;
		}
		if (maxMotion <= 0)
			return;
		step0 = 1.f/maxMotion;
	}
	int numDisp0(CEIL2INT((invzMax-invzMin)/step0)+1);
	const int maxNumDisp0(std::numeric_limits<Disparity>::max()/subpixelSteps-64);
	if (numDisp0 > maxNumDisp0) {
		step0 = (invzMax-invzMin)/(maxNumDisp0-1);
		numDisp0 = maxNumDisp0;
	}
	// compute scale used for the top level
	ViewData refData;
	refData.imageColor = refImage.image;
	refImage.image.toGray(refData.imageGray, cv::COLOR_BGR2GRAY, true, true);
	REAL scale(1);
	if (minResolution) {
		unsigned resolutionLevel(8);
		Image8U::computeMaxResolution(imageSize.width, imageSize.height, resolutionLevel, minResolution);
		scale = REAL(1)/MAXF(2,POWI(2,resolutionLevel));
	}
	// match coarse to fine, as a rectified pair
	DisparityMap disparityMap; AccumCostMap costMap; MaskMap maskMap;
	SlopeMap slopeMap;
	float step(0); // inverse-depth step of the current level
	// inverse-depth change along the slanted plane of each texel, over every tStep-th texel
	auto texelSlant = [&](int r, int c, int tStep, float* slant) {
		const Point2f slope(slopeMap.empty() ? Point2f(0,0) :
			Point2f(slopeMap(CLAMP((r-halfWindowSizeY)/2, 0, slopeMap.rows-1), CLAMP((c-halfWindowSizeX)/2, 0, slopeMap.cols-1))*step));
		for (int i=-halfWindowSizeY, n=0; i<=halfWindowSizeY; i+=tStep)
			for (int j=-halfWindowSizeX; j<=halfWindowSizeX; j+=tStep)
				slant[n++] = slope.x*(float)j + slope.y*(float)i;
	};
	do {
		const ViewData refLevel(refData.GetImage(scale));
		const cv::Size size(refLevel.imageGray.size());
		const cv::Size sizeValid(size.width-2*halfWindowSizeX, size.height-2*halfWindowSizeY);
		const bool bFirstLevel(disparityMap.empty());
		const cv::Matx33d invKref(MVS::Camera::InvK(refImage.camera.GetScaledK(imageSize, size)));
		for (NeighborView& view: views) {
			if (ISEQUAL(scale, REAL(1)))
				view.gray = view.grayFull;
			else
				cv::resize(view.grayFull, view.gray, cv::Size(), scale, scale, cv::INTER_AREA);
			view.SetLevel(invKref, cv::Matx33d(view.image->camera.GetScaledK(view.image->image.size(), view.gray.size())));
		}
		step = (float)(step0/scale);
		Index numCosts;
		if (bFirstLevel) {
			maskMap.create(sizeValid);
			maskMap.setTo(VALID);
			numCosts = Range2RangeMap(maskMap, Range{0, (Disparity)(CEIL2INT(numDisp0*scale)+1)}, false);
		} else {
			FitSlopes(disparityMap, slopeMap);
			UpscaleMask(maskMap, sizeValid);
			numCosts = Disparity2RangeMap(disparityMap, maskMap);
		}
		if (numCosts == 0)
			return;
		imageCosts.resize(numCosts);
		imageAccumCosts.resize(numCosts);
		auto pixel = [&](int idx, int r, int c) {
			const PixelData& pixel = imagePixels[idx];
			if (!pixel.range.isValid())
				return;
			const ImageRef u(c+halfWindowSizeX,r+halfWindowSizeY);
			WeightedPatch w;
			InitWeightedPatch(refLevel, u, w, texelStep);
			const TexelPatch<rowTexels> ref(w);
			TexelPositions<rowTexels>::Texels slant;
			texelSlant(r, c, texelStep, slant.data());
			Point3f hx[maxViews];
			TexelPositions<rowTexels> warps[maxViews];
			std::pair<float,float> inside[maxViews];
			FOREACH(k, views) {
				const NeighborView& view = views[k];
				hx[k] = view.A[0]*(float)u.x + view.A[1]*(float)u.y + view.A[2];
				warps[k].Warp(view.offsets, view.b, slant);
				inside[k] = warps[k].InsideRange(hx[k], view.b, view.width1, view.height1);
			}
			Cost* costs = imageCosts.data()+pixel.idx;
			for (int d=pixel.range.minDisp; d<pixel.range.maxDisp; ++d) {
				const float invz(invzMin+(float)d*step);
				float viewCosts[maxViews];
				int numViewCosts(0);
				if (invz > 0) {
					FOREACH(k, views) {
						if (invz <= inside[k].first || invz >= inside[k].second)
							continue;
						const NeighborView& view = views[k];
						const float cost(TexelsCost(view.gray, warps[k], hx[k] + view.b*invz, ref));
						// keep the view costs sorted
						int i(numViewCosts++);
						for (; i > 0 && viewCosts[i-1] > cost; --i)
							viewCosts[i] = viewCosts[i-1];
						viewCosts[i] = cost;
					}
				}
				if (numViewCosts == 0) {
					*costs++ = 255;
					continue;
				}
				const int numViews(MINF(numViewCosts, (int)numBestViews));
				float cost(0);
				for (int i=0; i<numViews; ++i)
					cost += viewCosts[i];
				*costs++ = (Cost)ROUND2INT(cost/numViews);
			}
		};
		ASSERT(threads.IsEmpty());
		if (!threads.empty()) {
			volatile Thread::safe_t idxPixel(-1);
			FOREACH(i, threads)
				threads.AddEvent(new EVTPixelProcess(sizeValid, idxPixel, pixel));
			WaitThreadWorkers(threads.size());
		} else
		for (int r=0; r<sizeValid.height; ++r)
			for (int c=0; c<sizeValid.width; ++c)
				pixel(r*sizeValid.width+c, r, c);
		Aggregate(refLevel.imageGray, disparityMap, costMap);
		// with no second disparity-map to cross-check against, only the speckles of the coarsest level
		// are removed; the depth-map fusion discards the depths the other views do not confirm
		if (bFirstLevel)
			cv::filterSpeckles(disparityMap, NO_DISP, OPTDENSE::nSpeckleSize, 5);
	} while ((scale*=2) < REAL(1)+ZEROTOLERANCE<REAL>());
	// sub-pixel refinement, first on the aggregated costs, then on the matching cost itself:
	// the index t of a pixel moves to the minimum of the parabola through the costs at t and t±delta,
	// over the whole window and the two views matching best at the first estimate, for two iterations
	// halving delta, as long as the costs are convex around t and the move does not raise its cost
	RefineDisparityMap(disparityMap);
	ASSERT(step == step0);
	auto refineIndex = [&](int r, int c, float t) -> float {
		const ImageRef u(c+halfWindowSizeX,r+halfWindowSizeY);
		WeightedPatch w;
		InitWeightedPatch(refData, u, w);
		const TexelPatch<windowSizeX> ref(w);
		TexelPositions<windowSizeX>::Texels slant;
		texelSlant(r, c, 1, slant.data());
		Point3f hx[maxViews];
		TexelPositions<windowSizeX> warps[maxViews];
		std::pair<float,float> inside[maxViews];
		FOREACH(k, views) {
			const NeighborView& view = views[k];
			hx[k] = view.A[0]*(float)u.x + view.A[1]*(float)u.y + view.A[2];
			warps[k].Warp(view.offsetsDense, view.b, slant);
			inside[k] = warps[k].InsideRange(hx[k], view.b, view.width1, view.height1);
		}
		IIndex best[numBestViews]; int numBest(0); {
			const float invz(invzMin+t*step);
			if (invz <= 0)
				return t;
			float bestCosts[numBestViews];
			FOREACH(k, views) {
				if (invz <= inside[k].first || invz >= inside[k].second)
					continue;
				const NeighborView& view = views[k];
				const float cost(TexelsCost(view.gray, warps[k], hx[k] + view.b*invz, ref));
				// keep the lowest costs sorted
				int i(numBest);
				if (numBest < (int)numBestViews)
					++numBest;
				else if (cost >= bestCosts[--i])
					continue;
				for (; i > 0 && bestCosts[i-1] > cost; --i) {
					bestCosts[i] = bestCosts[i-1]; best[i] = best[i-1];
				}
				bestCosts[i] = cost; best[i] = k;
			}
		}
		if (numBest == 0)
			return t;
		const auto cost = [&](float ti) -> float {
			const float invz(invzMin+ti*step);
			if (invz <= 0)
				return -1.f;
			float sum(0);
			for (int i=0; i<numBest; ++i) {
				const IIndex k(best[i]);
				if (invz <= inside[k].first || invz >= inside[k].second)
					return -1.f;
				const NeighborView& view = views[k];
				sum += TexelsCost(view.gray, warps[k], hx[k] + view.b*invz, ref);
			}
			return sum/numBest;
		};
		float f(cost(t)), delta(0.5f);
		if (f < 0)
			return t;
		for (int iter=0; iter<2; ++iter, delta*=0.5f) {
			const float fm(cost(t-delta)), fp(cost(t+delta));
			const float den(fm-2*f+fp);
			if (fm < 0 || fp < 0 || den <= 0)
				break;
			const float tn(t+CLAMP(delta*(fm-fp)/(2*den), -delta, delta));
			const float fn(cost(tn));
			if (fn >= 0 && fn <= f) {
				t = tn; f = fn;
			}
		}
		return t;
	};
	auto pixel = [&](int idx, int r, int c) {
		const Disparity d(disparityMap(idx));
		if (d == NO_DISP)
			return;
		const float invz(invzMin+refineIndex(r, c, (float)d/subpixelSteps)*step);
		if (invz <= 0)
			return;
		depthMap(r+halfWindowSizeY,c+halfWindowSizeX) = 1.f/invz;
		confMap(r+halfWindowSizeY,c+halfWindowSizeX) = PeakRatioConfidence(idx);
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelProcess(disparityMap.size(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<disparityMap.rows; ++r)
		for (int c=0; c<disparityMap.cols; ++c)
			pixel(r*disparityMap.cols+c, r, c);
	DEBUG_EXTRA("Depth-map for image %3u estimated from %u views: %u depths (%s)",
		refImage.ID, views.size(), cv::countNonZero(depthMap), TD_TIMER_GET_FMT().c_str());
	#endif
}

// Estimate the disparity change per pixel of the surface around each pixel, as the slope of the plane
// fit to the valid disparities of its 7x7 neighborhood; zero where too few are valid
void SemiGlobalMatcher::FitSlopes(const DisparityMap& disparityMap, SlopeMap& slopeMap)
{
	const int halfWindow(3), minValid(6);
	const float maxSlope(4.f);
	slopeMap.create(disparityMap.size());
	auto row = [&](int r) {
		for (int c=0; c<disparityMap.cols; ++c) {
			Point2f& slope = slopeMap(r,c);
			slope = Point2f(0,0);
			if (disparityMap(r,c) == NO_DISP)
				continue;
			float n(0), sx(0), sy(0), sd(0), sxx(0), syy(0), sxy(0), sxd(0), syd(0);
			for (int i=MAXF(r-halfWindow,0), ie=MINF(r+halfWindow,disparityMap.rows-1); i<=ie; ++i) {
				for (int j=MAXF(c-halfWindow,0), je=MINF(c+halfWindow,disparityMap.cols-1); j<=je; ++j) {
					const Disparity d(disparityMap(i,j));
					if (d == NO_DISP)
						continue;
					const float x((float)(j-c)), y((float)(i-r)), v((float)d);
					n += 1; sx += x; sy += y; sd += v;
					sxx += x*x; syy += y*y; sxy += x*y; sxd += x*v; syd += y*v;
				}
			}
			if (n < minValid)
				continue;
			// least-squares plane through the centered samples
			const float cxx(sxx-sx*sx/n), cyy(syy-sy*sy/n), cxy(sxy-sx*sy/n);
			const float cxd(sxd-sx*sd/n), cyd(syd-sy*sd/n);
			const float det(cxx*cyy-cxy*cxy);
			if (det <= 1e-3f*n*n)
				continue;
			slope.x = CLAMP((cyy*cxd-cxy*cyd)/det, -maxSlope, maxSlope);
			slope.y = CLAMP((cxx*cyd-cxy*cxd)/det, -maxSlope, maxSlope);
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(disparityMap.rows, idxPixel, row));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<disparityMap.rows; ++r)
		row(r);
}

#if SGM_SIMILARITY != SGM_SIMILARITY_CENSUS
// Compute the bilateral weights of the patch centered at u and its weighted zero-mean intensities,
// over every texelStep-th texel of each row and column
void SemiGlobalMatcher::InitWeightedPatch(const ViewData& image, const ImageRef& u, WeightedPatch& w, int texelStep)
{
	struct Compute {
		static float NormL1Sq(const Pixel8U& a, const Pixel8U& b) {
			return float(
				SQUARE(unsigned(a[0]<b[0] ? b[0]-a[0] : a[0]-b[0])) +
				SQUARE(unsigned(a[1]<b[1] ? b[1]-a[1] : a[1]-b[1])) +
				SQUARE(unsigned(a[2]<b[2] ? b[2]-a[2] : a[2]-b[2])));
		}
		static float WeightColor(const Image8U3& image, const Pixel8U& center, const ImageRef& x) {
			static const float sigmaColor(-1.f/(2.f*SQUARE(0.3f*255)));
			return Compute::NormL1Sq(image(x), center) * sigmaColor;
		}
		static float WeightSpatial(int x, int y) {
			static const float sigmaSpatial(-1.f/(2.f*SQUARE(0.4f*MAXF<int>(windowSizeX,windowSizeY))));
			return float(SQUARE(x) + SQUARE(y)) * sigmaSpatial;
		}
	};
	w.normSq0 = 0;
	w.sumWeights = 0;
	int n = 0;
	const Pixel8U& colCenter = image.imageColor(u);
	for (int i=-halfWindowSizeY; i<=halfWindowSizeY; i+=texelStep) {
		for (int j=-halfWindowSizeX; j<=halfWindowSizeX; j+=texelStep) {
			const ImageRef x(u.x+j,u.y+i);
			WeightedPatch::Pixel& pw = w.weights[n++];
			w.normSq0 +=
				(pw.tempWeight = image.imageGray(x)) *
				(pw.weight = EXP(Compute::WeightColor(image.imageColor, colCenter, x)+Compute::WeightSpatial(j,i)));
			w.sumWeights += pw.weight;
		}
	}
	ASSERT(texelStep != 1 || n == numTexels);
	const int numTexelsInit(n);
	const float tm(w.normSq0/w.sumWeights);
	w.normSq0 = 0;
	n = 0;
	do {
		WeightedPatch::Pixel& pw = w.weights[n];
		const float t(pw.tempWeight - tm);
		w.normSq0 += (pw.tempWeight = pw.weight * t) * t;
	} while (++n < numTexelsInit);
}
#endif

// Compute SGM stereo on the images
void SemiGlobalMatcher::Match(const ViewData& leftImage, const ViewData& rightImage, DisparityMap& disparityMap, AccumCostMap& costMap)
{
	const cv::Size size(leftImage.imageGray.size());
	const cv::Size sizeValid(size.width-2*halfWindowSizeX, size.height-2*halfWindowSizeY);
	ASSERT(leftImage.imageColor.size() == size);

	// compute costs
	{
	ASSERT(!imageCosts.empty());
	auto pixel = [&](int idx, int r, int c) {
		// ignore pixel if not valid
		const PixelData& pixel = imagePixels[idx];
		if (!pixel.range.isValid())
			return;
		#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
		struct Compute {
			static inline int HammingDistance(uint64_t c1, uint64_t c2) {
				return PopCnt(c1 ^ c2);
			}
		};
		// compute pixel cost
		Cost* costs = imageCosts.data()+pixel.idx;
		const Census lc(leftImage.imageCensus(r,c));
		for (int d=pixel.range.minDisp; d<pixel.range.maxDisp; ++d) {
			const ImageRef x(c+d,r);
			if (!rightImage.imageCensus.isInside(x)) {
				*costs++ = 255;
				continue;
			}
			const Census rc(rightImage.imageCensus(x));
			*costs++ = Compute::HammingDistance(lc, rc)*4;
		}
		#else
		const ImageRef u(c+halfWindowSizeX,r+halfWindowSizeY);
		WeightedPatch w;
		InitWeightedPatch(leftImage, u, w);
		// compute pixel cost
		Cost* costs = imageCosts.data()+pixel.idx;
		const int width(rightImage.imageGray.width());
		for (int d=pixel.range.minDisp; d<pixel.range.maxDisp; ++d) {
			// the patch rows are always inside the image, only its columns can fall outside
			const int x0(u.x+d-halfWindowSizeX);
			if (x0 < 0 || x0+windowSizeX > width) {
				*costs++ = 255;
				continue;
			}
			float sum(0), sumSq(0), nom(0);
			const WeightedPatch::Pixel* pw = w.weights;
			for (int i=-halfWindowSizeY; i<=halfWindowSizeY; ++i) {
				const ImageGray::Type* row = rightImage.imageGray.ptr<const ImageGray::Type>(u.y+i, x0);
				for (int j=0; j<windowSizeX; ++j, ++pw) {
					const float f(row[j]);
					const float fw(f*pw->weight);
					sum += fw;
					sumSq += f*fw;
					nom += f*pw->tempWeight;
				}
			}
			// the intensities are normalized to [0,1], so a texture-less patch has a variance
			// far below any regularization that leaves the textured ones untouched
			const float normSq1(sumSq-SQUARE(sum)/w.sumWeights);
			const float nrmSq(w.normSq0*normSq1);
			const float ncc(nrmSq <= 1e-16f ? 0.f : nom/SQRT(nrmSq));
			*costs++ = (ncc <= 0 ? Cost(255) : (Cost)ROUND2INT((1.f-MINF(ncc,1.f))*255.f));
		}
		#endif
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelProcess(sizeValid, idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<sizeValid.height; ++r)
		for (int c=0; c<sizeValid.width; ++c)
			pixel(r*sizeValid.width+c, r, c);
	}
	Aggregate(leftImage.imageGray, disparityMap, costMap);
}

// accumulate the costs of n consecutive disparities whose three previous costs Lp[-1..1] all exist
static void AccumulateInterior(const SemiGlobalMatcher::AccumCost* RESTRICT Lp, const SemiGlobalMatcher::Cost* RESTRICT costs,
	SemiGlobalMatcher::AccumCost* RESTRICT Ls, SemiGlobalMatcher::AccumCost* RESTRICT accums, int n,
	SemiGlobalMatcher::AccumCost minLp, SemiGlobalMatcher::AccumCost P1, SemiGlobalMatcher::AccumCost minLpP2)
{
	typedef SemiGlobalMatcher::AccumCost AccumCost;
	for (int i=0; i<n; ++i) {
		const AccumCost L(std::min(std::min(Lp[i], minLpP2), (AccumCost)(std::min(Lp[i-1], Lp[i+1])+P1)));
		accums[i] += (Ls[i] = (AccumCost)(costs[i]+L-minLp));
	}
}

// Aggregate the pixel costs along the paths and select the best disparity of each pixel
void SemiGlobalMatcher::Aggregate(const ImageGray& imageGray, DisparityMap& disparityMap, AccumCostMap& costMap)
{
	const cv::Size sizeValid(imageGray.width()-2*halfWindowSizeX, imageGray.height()-2*halfWindowSizeY);

	// accumulate costs
	{
	ASSERT(!imageAccumCosts.empty());
	imageAccumCosts.Memset(0);
	#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
	const ImageGray::Type Igray(127);
	#else
	const ImageGray::Type Igray(0.5f);
	#endif
	struct LineData {
		AccumCost* L;
		Range R;
		~LineData() { delete[] L; }
		AccumCost operator[] (int i) const { return L[i]; }
		AccumCost& operator[] (int i) { return L[i]; }
	};
	auto pixelAccum = [&](const Cost* costs, const LineData& Lp, LineData& Ls, AccumCost* accums, ImageGray::Type DI) {
		struct Compute {
			static inline void MINS(AccumCost& m, AccumCost v) { if (m > v) m = v; }
		};
		ASSERT(Ls.R.isValid());
		#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
		const AccumCost P2(P2s[DI]);
		#else
		const AccumCost P2(P2s[ABS(ROUND2INT(255.f*DI))]);
		#endif
		const Disparity minDisp(MAXF(Lp.R.minDisp, Ls.R.minDisp));
		const Disparity maxDisp(MINF(Lp.R.maxDisp, Ls.R.maxDisp));
		if (minDisp >= maxDisp) {
			// the disparity ranges for the two pixels do not intersect;
			// fill all accumulated costs with L(d)=C(d)+P2
			const Disparity numDisp(Ls.R.numDisp());
			for (int idxDisp=0; idxDisp<numDisp; ++idxDisp)
				accums[idxDisp] += (Ls[idxDisp] = costs[idxDisp]+P2);
		} else {
			// accumulate cost as L(d)=C(d)+min(Lp(dp)+V(d,dp))-min(Lp)
			// where V(d,dp) is:
			//  0  if d=dp
			//  P1 if |d-dp|=1
			//  P2 if |d-dp|>1
			// as P2>=P1, the P2 term is min(Lp)+P2 over all dp, so each L(d) is computed in constant time
			ASSERT(P2 >= P1);
			AccumCost minLp(std::numeric_limits<AccumCost>::max());
			for (const AccumCost *L=Lp.L+(minDisp-Lp.R.minDisp), *endL=L+(maxDisp-minDisp); L<endL; ++L)
				Compute::MINS(minLp, *L);
			const AccumCost minLpP2(minLp+P2);
			const auto accum = [&](Disparity d) {
				const int idxDisp(d-Ls.R.minDisp);
				AccumCost L(minLpP2);
				if (d >= minDisp && d < maxDisp)
					Compute::MINS(L, Lp[d-Lp.R.minDisp]);
				if (d > minDisp && d <= maxDisp)
					Compute::MINS(L, Lp[d-1-Lp.R.minDisp]+P1);
				if (d+1 >= minDisp && d+1 < maxDisp)
					Compute::MINS(L, Lp[d+1-Lp.R.minDisp]+P1);
				accums[idxDisp] += (Ls[idxDisp] = costs[idxDisp]+L-minLp);
			};
			// the disparities with all three previous costs are accumulated without branches
			const Disparity beginIn(MAXF(Ls.R.minDisp, (Disparity)(minDisp+1))), endIn(MINF(Ls.R.maxDisp, (Disparity)(maxDisp-1)));
			Disparity d(Ls.R.minDisp);
			for (; d<beginIn && d<Ls.R.maxDisp; ++d)
				accum(d);
			if (d < endIn) {
				AccumulateInterior(Lp.L+(d-Lp.R.minDisp), costs+(d-Ls.R.minDisp), Ls.L+(d-Ls.R.minDisp), accums+(d-Ls.R.minDisp), endIn-d, minLp, P1, minLpP2);
				d = endIn;
			}
			for (; d<Ls.R.maxDisp; ++d)
				accum(d);
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		struct AccumLines {
			LineData linesBuffer[2];
			LineData* lines[2];
			AccumLines(Disparity maxNumDisp) {
				for (LineData& line: linesBuffer) {
					line.L = new AccumCost[maxNumDisp];
					memset(line.L, 0, sizeof(AccumCost)*maxNumDisp);
					line.R.minDisp = line.R.maxDisp = 0;
				}
				lines[0] = linesBuffer+0;
				lines[1] = linesBuffer+1;
			}
			void NextLine() { std::swap(lines[0], lines[1]); }
			const LineData& operator() (int i) const { return *lines[i]; }
			LineData& operator() (int i) { return *lines[i]; }
		};
		// u walks the valid region, whose pixels sit half a window inside the image
		#define ACCUM_PIXELS(cond) \
			AccumLines lines(maxNumDisp); \
			ImageGray::Type Ip(Igray); \
			do { \
				const int idx(u.y*sizeValid.width+u.x); \
				const PixelData& pixel = imagePixels[idx]; \
				if (!pixel.range.isValid()) { \
					/* the path restarts after an unsearched pixel */ \
					lines(0).R = Range{0,0}; \
					Ip = Igray; \
					continue; \
				} \
				const Cost* costs = imageCosts.cdata()+pixel.idx; \
				AccumCost* accums = imageAccumCosts.data()+pixel.idx; \
				const LineData& Lp = lines(0); \
				LineData& Ls = lines(1); \
				Ls.R = pixel.range; \
				const ImageGray::Type I(imageGray(u.y+halfWindowSizeY,u.x+halfWindowSizeX)); \
				pixelAccum(costs, Lp, Ls, accums, I-Ip); \
				Ip = I; \
				lines.NextLine(); \
			} while (cond)
		{ // width-down
		auto pixels = [&](int x) {
			ImageRef u(x,0);
			ACCUM_PIXELS(++u.y < sizeValid.height);
		};
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.width, idxPixel, pixels));
		WaitThreadWorkers(threads.size());
		}
		{ // height-right
		auto pixels = [&](int y) {
			ImageRef u(0,y);
			ACCUM_PIXELS(++u.x < sizeValid.width);
		};
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.height, idxPixel, pixels));
		WaitThreadWorkers(threads.size());
		}
		{ // width-up
		auto pixels = [&](int x) {
			ImageRef u(x,sizeValid.height-1);
			ACCUM_PIXELS(--u.y >= 0);
		};
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.width, idxPixel, pixels));
		WaitThreadWorkers(threads.size());
		}
		{ // height-left
		auto pixels = [&](int y) {
			ImageRef u(sizeValid.width-1,y);
			ACCUM_PIXELS(--u.x >= 0);
		};
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.height, idxPixel, pixels));
		WaitThreadWorkers(threads.size());
		}
		if (numDirs == 4) {
		// each pair of diagonal sweeps runs concurrently and is waited for as one, after
		// both blocks queuing it have closed, so the pixel counters must outlive them
		volatile Thread::safe_t idxPixels[2];
		{ // width-right-down
		auto pixels = [&](int x) {
			ImageRef u(x,0);
			ACCUM_PIXELS(++u.x < sizeValid.width && ++u.y < sizeValid.height);
		};
		idxPixels[0] = -1;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.width, idxPixels[0], pixels));
		}
		{ // height-right-down
		auto pixels = [&](int y) {
			ImageRef u(0,y);
			ACCUM_PIXELS(++u.x < sizeValid.width && ++u.y < sizeValid.height);
		};
		idxPixels[1] = 0;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.height, idxPixels[1], pixels));
		}
		WaitThreadWorkers(threads.size()*2);
		{ // width-left-down
		auto pixels = [&](int x) {
			ImageRef u(x,0);
			ACCUM_PIXELS(--u.x >= 0 && ++u.y < sizeValid.height);
		};
		idxPixels[0] = -1;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.width-1, idxPixels[0], pixels));
		}
		{ // height-left-down
		auto pixels = [&](int y) {
			ImageRef u(sizeValid.width-1,y);
			ACCUM_PIXELS(--u.x >= 0 && ++u.y < sizeValid.height);
		};
		idxPixels[1] = -1;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.height, idxPixels[1], pixels));
		}
		WaitThreadWorkers(threads.size()*2);
		{ // width-right-up
		auto pixels = [&](int x) {
			ImageRef u(x,sizeValid.height-1);
			ACCUM_PIXELS(++u.x < sizeValid.width && --u.y >= 0);
		};
		idxPixels[0] = 0;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.width, idxPixels[0], pixels));
		}
		{ // height-right-up
		auto pixels = [&](int y) {
			ImageRef u(0,y);
			ACCUM_PIXELS(++u.x < sizeValid.width && --u.y >= 0);
		};
		idxPixels[1] = sizeValid.height;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumDec(idxPixels[1], pixels));
		}
		WaitThreadWorkers(threads.size()*2);
		{ // width-left-up
		auto pixels = [&](int x) {
			ImageRef u(x,sizeValid.height-1);
			ACCUM_PIXELS(--u.x >= 0 && --u.y >= 0);
		};
		idxPixels[0] = sizeValid.width;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumDec(idxPixels[0], pixels));
		}
		{ // height-left-up
		auto pixels = [&](int y) {
			ImageRef u(sizeValid.width-1,y);
			ACCUM_PIXELS(--u.x >= 0 && --u.y >= 0);
		};
		idxPixels[1] = sizeValid.height-1;
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumDec(idxPixels[1], pixels));
		}
		WaitThreadWorkers(threads.size()*2);
		}
		#undef ACCUM_PIXELS
	} else {
	const ImageRef dirs[] = {{-1,0}, {0,-1}, {-1,-1}, {1,-1}};
	struct AccumLines {
		const Disparity maxNumDisp;
		LineData* linesBuffer;
		LineData* lines[numDirs][2];
		AccumLines(Disparity _maxNumDisp) : maxNumDisp(_maxNumDisp), linesBuffer(NULL) {}
		~AccumLines() { delete[] linesBuffer; }
		void Init(int w) {
			const int linewidth(w+2);
			const int buffersize(2*linewidth*numDirs);
			if (linesBuffer == NULL) {
				linesBuffer = new LineData[buffersize];
				for (int i=0; i<buffersize; ++i)
					linesBuffer[i].L = new AccumCost[maxNumDisp];
				LineData* line(linesBuffer-linewidth+1);
				for (int idxDir=0; idxDir<numDirs; ++idxDir) {
					lines[idxDir][0] = (line+=linewidth);
					lines[idxDir][1] = (line+=linewidth);
				}
			}
			for (int i=0; i<buffersize; ++i) {
				LineData& line = linesBuffer[i];
				memset(line.L, 0, sizeof(AccumCost)*maxNumDisp);
				line.R.minDisp = line.R.maxDisp = 0;
			}
		}
		void NextLine() {
			for (int idxDir=0; idxDir<numDirs; ++idxDir)
				std::swap(lines[idxDir][0], lines[idxDir][1]);
		}
		const LineData& operator() (int idxDir, int r, int c) const { return lines[idxDir][r][c]; }
		LineData& operator() (int idxDir, int r, int c) { return lines[idxDir][r][c]; }
	};
	AccumLines lines(maxNumDisp);
	// (r,c) walks the valid region, whose pixels sit half a window inside the image
	#define ACCUM_PIXELS(dx, dy, _x) \
		const int idx(r*sizeValid.width+c); \
		const PixelData& pixel = imagePixels[idx]; \
		if (!pixel.range.isValid()) { \
			/* the paths restart after an unsearched pixel */ \
			for (int idxDir=0; idxDir<numDirs; ++idxDir) \
				lines(idxDir,1,_x).R = Range{0,0}; \
			continue; \
		} \
		const Cost* costs = imageCosts.cdata()+pixel.idx; \
		AccumCost* accums = imageAccumCosts.data()+pixel.idx; \
		for (int idxDir=0; idxDir<numDirs; ++idxDir) { \
			const ImageRef& dir = dirs[idxDir]; \
			const LineData& Lp = lines(idxDir,1+dir.y,_x+dir.x); \
			LineData& Ls = lines(idxDir,1,_x); \
			Ls.R = pixel.range; \
			const ImageRef xp(c+dx, r+dy); \
			const bool bInside(xp.x >= 0 && xp.y >= 0 && xp.x < sizeValid.width && xp.y < sizeValid.height); \
			const ImageGray::Type DI(imageGray(r+halfWindowSizeY,c+halfWindowSizeX)-(bInside?imageGray(xp.y+halfWindowSizeY,xp.x+halfWindowSizeX):Igray)); \
			pixelAccum(costs, Lp, Ls, accums, DI); \
		}
	lines.Init(sizeValid.width);
	for (int r=0; r<sizeValid.height; ++r) {
		for (int c=0; c<sizeValid.width; ++c) {
			ACCUM_PIXELS(dir.x, dir.y, c);
		}
		lines.NextLine();
	}
	lines.Init(sizeValid.width);
	for (int r=sizeValid.height; --r>=0; ) {
		for (int c=sizeValid.width; --c>=0; ) {
			ACCUM_PIXELS(-dir.x, -dir.y, sizeValid.width-1-c);
		}
		lines.NextLine();
	}
	#undef ACCUM_PIXELS
	}
	}

	// select best disparity and cost
	{
	disparityMap.create(sizeValid);
	costMap.create(sizeValid);
	auto pixel = [&](int idx) {
		const PixelData& pixel = imagePixels[idx];
		if (pixel.range.isValid()) {
			const AccumCost* accums = imageAccumCosts.cdata()+pixel.idx;
			const AccumCost* bestAccum = accums;
			for (const AccumCost *accum=accums+1, *accumEnd=accums+pixel.range.numDisp(); accum<accumEnd; ++accum) {
				if (*bestAccum > *accum)
					bestAccum = accum;
			}
			disparityMap(idx) = pixel.range.minDisp+(Disparity)(bestAccum-accums);
			costMap(idx) = *bestAccum;
		} else {
			disparityMap(idx) = pixel.range.minDisp;
			costMap(idx) = NO_ACCUMCOST;
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(sizeValid.area(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<sizeValid.height; ++r)
		for (int c=0; c<sizeValid.width; ++c)
			pixel(r*sizeValid.width+c);
	}
}

#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
// Compute the census bit-mask for all the pixels of the image
void SemiGlobalMatcher::CensusTransform(const Image8U& imageGray, CensusMap& imageCensus)
{
	ASSERT(!imageGray.empty());
	const cv::Size size(imageGray.size());
	const cv::Size sizeValid(size.width-2*halfWindowSizeX, size.height-2*halfWindowSizeY);
	imageCensus.create(sizeValid);

	const Image8U& image = imageGray;

	auto pixel = [&](int, int r, int c) {
		const ImageRef u(c+halfWindowSizeX, r+halfWindowSizeY);
		const uint8_t g(image(u));
		Census& cs = imageCensus(r,c);
		cs = 0;
		for (int i=-halfWindowSizeY; i<=halfWindowSizeY; ++i) {
			for (int j=-halfWindowSizeX; j<=halfWindowSizeX; ++j) {
				cs <<= 1;
				if (g <= image(u.y+i, u.x+j))
					cs += 1;
			}
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelProcess(imageCensus.size(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<imageCensus.rows; ++r)
		for (int c=0; c<imageCensus.cols; ++c)
			pixel(-1, r, c);
}
#endif

// Compute the left-to-right disparity range spanned by the depth range [dMin,dMax] of the left image
// over the valid pixels of the rectified pair scaled by the given factor
SemiGlobalMatcher::Range SemiGlobalMatcher::DepthRange2Disparity(const Matrix3x3& H, const Matrix4x4& Q, REAL scale, const MaskMap& maskMap, Depth dMin, Depth dMax)
{
	Matrix3x3 Hs(H); Matrix4x4 Qs(Q);
	Image::ScaleStereoRectification(Hs, Qs, scale);
	const Matrix3x3 invH(Hs.inv());
	const Matrix4x4 invQ(Qs.inv());
	float minDisp(FLT_MAX), maxDisp(-FLT_MAX);
	const int step(4);
	for (int r=0; r<maskMap.rows; r+=step) {
		for (int c=0; c<maskMap.cols; c+=step) {
			if (maskMap(r,c) == INVALID)
				continue;
			const ImageRef x(c+halfWindowSizeX,r+halfWindowSizeY); Point2f u;
			ProjectVertex_3x3_2_2(invH.val, x.ptr(), u.ptr());
			for (const Depth depth: {dMin, dMax}) {
				float disparity;
				if (!Image::Depth2Disparity(invQ, u, depth, disparity))
					continue;
				if (minDisp > disparity)
					minDisp = disparity;
				if (maxDisp < disparity)
					maxDisp = disparity;
			}
		}
	}
	if (minDisp > maxDisp)
		return Range{NO_DISP, NO_DISP};
	// a margin covers the pixels between the samples; no match can shift a pixel by more than the image width
	const int width(maskMap.width());
	return Range{
		(Disparity)MAXF(FLOOR2INT(minDisp)-step, -width),
		(Disparity)MINF(CEIL2INT(maxDisp)+step+1, width)
	};
}

// Setup pixel-map searching the same disparity range for all pixels valid in the mask-map,
// clamped per column to the disparities keeping the matched patch inside the other rectified image
// if requested; return the total size of the disparities searched
SemiGlobalMatcher::Index SemiGlobalMatcher::Range2RangeMap(const MaskMap& maskMap, const Range& range, bool bClampToWidth)
{
	ASSERT(range.isValid() && maskMap.isContinuous());
	const int width(maskMap.width());
	imagePixels.resize(maskMap.area());
	maxNumDisp = 0;
	Index numCosts(0);
	for (int r=0, idx=0; r<maskMap.rows; ++r) {
		for (int c=0; c<width; ++c, ++idx) {
			PixelData& pixel = imagePixels[idx];
			pixel.idx = numCosts;
			pixel.range = bClampToWidth ? Range{MAXF(range.minDisp, (Disparity)-c), MINF(range.maxDisp, (Disparity)(width-c))} : range;
			if (maskMap(idx) == INVALID || !pixel.range.isValid()) {
				pixel.range = Range{NO_DISP,NO_DISP};
				continue;
			}
			const Disparity numDisp(pixel.range.numDisp());
			numCosts += numDisp;
			if (maxNumDisp < numDisp)
				maxNumDisp = numDisp;
		}
	}
	return numCosts;
}

// Setup the pixel-map at twice the scale of the given disparity-map, each pixel searching around
// the disparities estimated at the previous level in its neighborhood (7x7 if its own disparity is
// valid, 41x41 otherwise), and none where the (already upscaled) mask-map is invalid;
// return the total size of the disparities searched
SemiGlobalMatcher::Index SemiGlobalMatcher::Disparity2RangeMap(const DisparityMap& disparityMap, const MaskMap& maskMap)
{
	ASSERT(!disparityMap.empty() && disparityMap.width()<maskMap.width() && disparityMap.height()<maskMap.height());
	const cv::Size size2x(maskMap.size());

	// compute the search range of each pixel of the previous level
	const Disparity minNumDisp(5);
	CLISTDEF0IDX(Range,int) rangeMap(disparityMap.area());
	auto row = [&](int r) {
		CLISTDEF0IDX(Disparity,Disparity) disps(0, 41*41);
		const Mask* pm(maskMap.ptr<const Mask>(r*2+halfWindowSizeY, halfWindowSizeX));
		Range* ranges(rangeMap.data()+r*disparityMap.cols);
		for (int c=0; c<disparityMap.cols; ++c, pm+=2) {
			Range& range = ranges[c];
			if (*pm == INVALID) {
				range = Range{NO_DISP,NO_DISP};
				continue;
			}
			const bool bInvalid(disparityMap(r,c) == NO_DISP);
			const int hw(bInvalid ? 20 : 3);
			disps.Empty();
			for (int i=MAXF(r-hw,0), ie=MINF(r+hw,disparityMap.rows-1); i<=ie; ++i) {
				for (int j=MAXF(c-hw,0), je=MINF(c+hw,disparityMap.cols-1); j<=je; ++j) {
					const Disparity d(disparityMap(i,j));
					if (d != NO_DISP)
						disps.push_back(d);
				}
			}
			if (disps.size() < 3) {
				// nothing known around this pixel
				range = Range{NO_DISP,NO_DISP};
				continue;
			}
			const Disparity disp(disps.GetMedian<Disparity>()*2);
			const auto minmax(disps.GetMinMax());
			Disparity numDisp((minmax.second-minmax.first)*2);
			const Disparity capNumDisp(bInvalid ? 64 : 32);
			if (numDisp > capNumDisp) {
				// too wide, keep the part of the range around the median
				range.minDisp = disp-(capNumDisp*(disp-minmax.first*2)+1)/numDisp;
				range.maxDisp = disp+(capNumDisp*(minmax.second*2+1-disp)+1)/numDisp;
			} else {
				if (numDisp < minNumDisp)
					numDisp = minNumDisp;
				range.minDisp = disp-numDisp/2;
				range.maxDisp = disp+(numDisp+1)/2;
			}
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(disparityMap.rows, idxPixel, row));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<disparityMap.rows; ++r)
		row(r);

	// setup the pixel-map at twice the scale: pixel (r,c) of the previous level covers the 2x2 block
	// at (2r+halfWindowSizeY,2c+halfWindowSizeX), the border pixels copy the closest block
	imagePixels.resize(size2x.area());
	Index numCosts(0);
	maxNumDisp = 0;
	for (int r2=0, idx=0; r2<size2x.height; ++r2) {
		const Range* ranges(rangeMap.data()+CLAMP((r2-halfWindowSizeY)/2, 0, disparityMap.rows-1)*disparityMap.cols);
		for (int c2=0; c2<size2x.width; ++c2, ++idx) {
			PixelData& pixel = imagePixels[idx];
			pixel.range = ranges[CLAMP((c2-halfWindowSizeX)/2, 0, disparityMap.cols-1)];
			pixel.idx = numCosts;
			if (!pixel.range.isValid())
				continue;
			const Disparity numDisp(pixel.range.numDisp());
			numCosts += numDisp;
			if (maxNumDisp < numDisp)
				maxNumDisp = numDisp;
		}
	}
	return numCosts;
}

// Check for consistency between a left-to-right and right-to-left pair of stereo results;
// the results are expected to be opposite in sign but equal in magnitude;
// the valid disparities are returned in the left map
void SemiGlobalMatcher::ConsistencyCrossCheck(DisparityMap& l2r, const DisparityMap& r2l, Disparity thCross)
{
	ASSERT(thCross >= 0);
	ASSERT(!l2r.empty() && !r2l.empty());
	ASSERT(l2r.height() == r2l.height());

	auto pixel = [&](int, int r, int c) {
		Disparity& ld = l2r(r,c);
		if (ld == NO_DISP)
			return;
		// compute the corresponding disparity pixel according to the disparity value
		const ImageRef v(c+ld,r);
		// check image bounds
		if (v.x < 0 || v.x >= r2l.width()) {
			ld = NO_DISP;
			return;
		}
		// check right disparity is valid
		const Disparity rd = r2l(v);
		if (r2l(v) == NO_DISP) {
			ld = NO_DISP;
			return;
		}
		// check disparity consistency:
		//   since the left and right disparities are opposite in sign,
		//   we determine their similarity by *summing* them, rather
		//   than differencing them as you might expect
		if (ABS(ld + rd) > thCross)
			ld = NO_DISP;
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelProcess(l2r.size(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<l2r.rows; ++r)
		for (int c=0; c<l2r.cols; ++c)
			pixel(-1, r, c);
}

// Mark empty regions on the border of the disparity-map as invalid;
//  thValid is the number of valid disparities encountered to consider the valid region starts
void SemiGlobalMatcher::ExtractMask(const DisparityMap& disparityMap, MaskMap& maskMap, int thValid)
{
	ASSERT(!disparityMap.empty());
	ASSERT(maskMap.empty() || disparityMap.size() == maskMap.size());
	if (disparityMap.size() != maskMap.size()) {
		maskMap.create(disparityMap.size());
		maskMap.setTo(VALID);
	}

	#define MASK_PIXEL() \
		Mask& m = maskMap(r,c); \
		if (m == INVALID) \
			continue; \
		m = INVALID; \
		if (disparityMap(r,c) == NO_DISP) \
			continue; \
		if (++numValid >= thValid) \
			break

	// left-right direction
	{
	auto pixel = [&](int r) {
		int numValid(0);
		for (int c=0; c<disparityMap.cols; ++c) {
			MASK_PIXEL();
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(disparityMap.height(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<disparityMap.rows; ++r)
		pixel(r);
	}

	// right-left direction
	{
	auto pixel = [&](int r) {
		int numValid(0);
		for (int c=disparityMap.cols; --c>=0; ) {
			MASK_PIXEL();
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(disparityMap.height(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<disparityMap.rows; ++r)
		pixel(r);
	}

	#undef MASK_PIXEL
}

// Translate disparity-map between left-to-right and right-to-left stereo pair
// by translating to the x coordinated dictated by the disparity and negating the sign;
// the sub-pixel steps is assumed to be one and
// the input disparity-map has been cross-checked for consistency
void SemiGlobalMatcher::FlipDirection(const DisparityMap& l2r, DisparityMap& r2l)
{
	ASSERT(!l2r.empty());
	ASSERT(r2l.empty() || l2r.height() == r2l.height());
	if (r2l.empty())
		r2l.create(l2r.size());
	r2l.setTo(NO_DISP);

	auto pixel = [&](int r) {
		for (int c=0; c<l2r.cols; ++c) {
			const Disparity d = l2r(r,c);
			if (d == NO_DISP)
				continue;
			// compute the corresponding disparity pixel according to the disparity value and set right disparity value
			for (int x=MAXF(c+d-1,0), xe=MINF(c+d+2,r2l.width()); x<xe; ++x)
				r2l(r,x) = -d;
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(l2r.rows, idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<l2r.rows; ++r)
		pixel(r);
}

// Translate disparity-map between left-to-right and right-to-left stereo pair
// by translating to the x coordinated dictated by the disparity and negating the sign;
// the input disparity-map has been cross-checked for consistency
void SemiGlobalMatcher::UpscaleMask(MaskMap& maskMap, const cv::Size& size2x)
{
	ASSERT(!maskMap.empty());
	MaskMap maskMap2x(size2x, INVALID);

	auto pixel = [&](int, int r, int c) {
		const int r2(r*2+halfWindowSizeY), c2(c*2+halfWindowSizeX);
		const Mask m(maskMap(r,c));
		for (int i=0; i<2; ++i) {
			for (int j=0; j<2; ++j) {
				const ImageRef u(c2+j,r2+i);
				if (maskMap2x.isInside(u))
					maskMap2x(u) = m;
			}
		}
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelProcess(maskMap.size(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<maskMap.rows; ++r)
		for (int c=0; c<maskMap.cols; ++c)
			pixel(-1, r, c);

	cv::swap(maskMap, maskMap2x);
}

// Sub-pixel disparity-map refinement based on the accumulated-cost volume
void SemiGlobalMatcher::RefineDisparityMap(DisparityMap& disparityMap) const
{
	ASSERT(!disparityMap.empty());
	if (subpixelSteps <= 1)
		return;
	if (subpixelMode == SUBPIXEL_NA) {
		// simply multiply disparity to the sub-pixel steps
		auto pixel = [&](int, int r, int c) {
			Disparity& d = disparityMap(r,c);
			if (d == NO_DISP)
				return;
			ASSERT((int)d*subpixelSteps > (int)std::numeric_limits<Disparity>::min());
			ASSERT((int)d*subpixelSteps < (int)std::numeric_limits<Disparity>::max());
			d *= subpixelSteps;
		};
		ASSERT(threads.IsEmpty());
		if (!threads.empty()) {
			volatile Thread::safe_t idxPixel(-1);
			FOREACH(i, threads)
				threads.AddEvent(new EVTPixelProcess(disparityMap.size(), idxPixel, pixel));
			WaitThreadWorkers(threads.size());
		} else
		for (int r=0; r<disparityMap.rows; ++r)
			for (int c=0; c<disparityMap.cols; ++c)
				pixel(-1, r, c);
		return;
	}
	// proposed subpixelMode algorithms
	typedef float real;
	struct Fit {
		static real linear(real x) {
			return x/real(2);
		}
		static real poly4(real x) {
			return (x*x*x*x + x)/real(4);
		}
		static real parabola(real x) {
			return x/(x+real(1));
		}
		static real sine(real x) {
			return real(0.5) * (SIN((x-real(1))*real(HALF_PI)) + real(1));
		}
		static real cosine(real x) {
			return (real(1) - COS(x*(real)(PI/3.0)));
		}
		static real lcBlend(real x) {
			const real factor(real(1.195) - COS(x*(real)(PI/2.3)));
			return cosine(x)*factor + linear(x)*(real(1)-factor);
		}
		// subpixelMode interpolation when only two values are available
		// returns fraction of distance from the primary to the other value
		static real semisubpixel(AccumCost primary, AccumCost other) {
			return real(0.5)*(static_cast<real>(primary) / static_cast<real>(other));
		}
		// compute the offset to be added to the integer disparity to get the final result
		static real subpixelMode(AccumCost prev, AccumCost center, AccumCost next, SgmSubpixelMode subpixelMode) {
			ASSERT(prev != NO_ACCUMCOST && center != NO_ACCUMCOST && next != NO_ACCUMCOST);
			// use a lower quality two value interpolation if only two values are available
			if (prev == center)
				return center == next ? real(0) : semisubpixel(center, next);
			if (center == next)
				return prev == center ? real(0) : -semisubpixel(center, prev);
			// pick which direction to interpolate in
			const AccumCost ld(prev-center);
			const AccumCost rd(next-center);
			real x, mult;
			if (ld < rd) {
				x = static_cast<real>(ld) / static_cast<real>(rd);
				mult = real(1);
			} else {
				x = static_cast<real>(rd) / static_cast<real>(ld);
				mult = real(-1);
			}
			// use the selected subpixelMode function
			real value(0);
			switch (subpixelMode) {
			case SUBPIXEL_LINEAR:   value = linear(x); break;
			case SUBPIXEL_POLY4:    value = poly4(x); break;
			case SUBPIXEL_PARABOLA: value = parabola(x); break;
			case SUBPIXEL_SINE:     value = sine(x); break;
			case SUBPIXEL_COSINE:   value = cosine(x); break;
			case SUBPIXEL_LC_BLEND: value = lcBlend(x); break;
			};
			// complete computation
			return (value - real(0.5))*mult;
		}
	};
	// estimate sub-pixel disparity based on the cost values
	auto pixel = [&](int idx) {
		const PixelData& pixel = imagePixels[idx];
		if (pixel.range.numDisp() < 2)
			return;
		Disparity& d = disparityMap(idx);
		if (d == NO_DISP)
			return;
		const AccumCost* accums = imageAccumCosts.cdata()+pixel.idx;
		const int idxDisp(d-pixel.range.minDisp);
		real disparity((real)d);
		if (d == pixel.range.minDisp)
			disparity += Fit::semisubpixel(accums[idxDisp], accums[idxDisp+1]);
		else if (d+1 == pixel.range.maxDisp)
			disparity -= Fit::semisubpixel(accums[idxDisp], accums[idxDisp-1]);
		else
			disparity += Fit::subpixelMode(accums[idxDisp-1], accums[idxDisp], accums[idxDisp+1], subpixelMode);
		ASSERT(ROUND2INT(disparity*subpixelSteps) > (int)std::numeric_limits<Disparity>::min());
		ASSERT(ROUND2INT(disparity*subpixelSteps) < (int)std::numeric_limits<Disparity>::max());
		d = (Disparity)ROUND2INT(disparity*subpixelSteps);
	};
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		volatile Thread::safe_t idxPixel(-1);
		FOREACH(i, threads)
			threads.AddEvent(new EVTPixelAccumInc(disparityMap.size().area(), idxPixel, pixel));
		WaitThreadWorkers(threads.size());
	} else
	for (int r=0; r<disparityMap.rows; ++r)
		for (int c=0; c<disparityMap.cols; ++c)
			pixel(r*disparityMap.cols+c);
}

// Confidence in [0,1] of the disparity the winner-take-all selects for a pixel: how much lower its
// accumulated cost is than the lowest one of the disparities not adjacent to it (the second peak), as
// the square root of one minus their ratio, which puts it on the scale of the matching score the
// fusion gate 1-fNCCThresholdKeep expects; zero if no such disparity was searched or both costs are zero
float SemiGlobalMatcher::PeakRatioConfidence(Index idxPixel) const
{
	const PixelData& pixel = imagePixels[idxPixel];
	ASSERT(pixel.range.isValid());
	const AccumCost* accums = imageAccumCosts.cdata()+pixel.idx;
	const int numDisp(pixel.range.numDisp());
	const int best((int)(std::min_element(accums, accums+numDisp)-accums));
	AccumCost second(NO_ACCUMCOST);
	for (int d=0; d<numDisp; ++d)
		if ((d < best-1 || d > best+1) && second > accums[d])
			second = accums[d];
	if (second == NO_ACCUMCOST || second == 0)
		return 0.f;
	return SQRT(1.f-(float)accums[best]/(float)second);
}


EventThreadPool SemiGlobalMatcher::threads;
Semaphore SemiGlobalMatcher::sem;

// start worker threads
void SemiGlobalMatcher::CreateThreads(unsigned nMaxThreads)
{
	ASSERT(nMaxThreads > 0);
	ASSERT(threads.IsEmpty() && threads.empty());
	if (nMaxThreads > 1) {
		threads.resize(nMaxThreads);
		threads.start(ThreadWorker);
	}
}
// destroy worker threads
void SemiGlobalMatcher::DestroyThreads()
{
	ASSERT(threads.IsEmpty());
	if (!threads.empty()) {
		FOREACH(i, threads)
			threads.AddEvent(new EVTClose());
		threads.Release();
	}
}

void* SemiGlobalMatcher::ThreadWorker(void*) {
	while (true) {
		CAutoPtr<Event> evt(threads.GetEvent());
		switch (evt->GetID()) {
		case EVT_JOB:
			evt->Run();
			break;
		case EVT_CLOSE:
			return NULL;
		default:
			ASSERT("Should not happen!" == NULL);
		}
		sem.Signal();
	}
	return NULL;
}
void SemiGlobalMatcher::WaitThreadWorkers(unsigned nJobs)
{
	while (nJobs-- > 0)
		sem.Wait();
	ASSERT(threads.IsEmpty());
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

bool SemiGlobalMatcher::ExportDisparityDataRaw(const String& fileName, const DisparityMap& disparityMap, const AccumCostMap& costMap, const cv::Size& imageSize, const Matrix3x3& H, const Matrix4x4& Q, Disparity subpixelSteps)
{
	ASSERT(!disparityMap.empty());
	ASSERT(costMap.empty() || disparityMap.size() == costMap.size());

	FILE *f = fopen(fileName, "wb");
	if (f == NULL)
		return false;

	// write info
	fwrite(&imageSize.width, sizeof(int), 2, f);
	fwrite(H.val, sizeof(REAL), 9, f);
	fwrite(Q.val, sizeof(REAL), 16, f);
	fwrite(&subpixelSteps, sizeof(Disparity), 1, f);

	// write resolution
	fwrite(&disparityMap.cols, sizeof(int), 1, f);
	fwrite(&disparityMap.rows, sizeof(int), 1, f);

	// write disparity-map
	fwrite(disparityMap.getData(), sizeof(Disparity), disparityMap.area(), f);

	// write cost-map
	if (!costMap.empty())
		fwrite(costMap.getData(), sizeof(AccumCost), costMap.area(), f);

	fclose(f);
	return true;
} // ExportDisparityDataRaw
// same as above, but exports also the empty border
bool SemiGlobalMatcher::ExportDisparityDataRawFull(const String& fileName, const DisparityMap& disparityMap, const AccumCostMap& costMap, const cv::Size& imageSize, const Matrix3x3& H, const Matrix4x4& Q, Disparity subpixelSteps)
{
	ASSERT(!disparityMap.empty());
	ASSERT(costMap.empty() || disparityMap.size() == costMap.size());

	const cv::Size size(disparityMap.width()+2*halfWindowSizeX,disparityMap.height()+2*halfWindowSizeY);
	const cv::Rect ROI(halfWindowSizeX,halfWindowSizeY, disparityMap.width(),disparityMap.height());
	DisparityMap disparityMapFull(size, NO_DISP);
	disparityMap.copyTo(disparityMapFull(ROI));
	if (costMap.empty())
		return ExportDisparityDataRaw(fileName, disparityMapFull, costMap, imageSize, H, Q, subpixelSteps);
	AccumCostMap costMapFull(size, NO_ACCUMCOST);
	costMap.copyTo(costMapFull(ROI));
	return ExportDisparityDataRaw(fileName, disparityMapFull, costMapFull, imageSize, H, Q, subpixelSteps);
} // ExportDisparityDataRawFull

bool SemiGlobalMatcher::ImportDisparityDataRaw(const String& fileName, DisparityMap& disparityMap, AccumCostMap& costMap, cv::Size& imageSize, Matrix3x3& H, Matrix4x4& Q, Disparity& subpixelSteps)
{
	FILE *f = fopen(fileName, "rb");
	if (f == NULL)
		return false;

	// read info
	fread(&imageSize.width, sizeof(int), 2, f);
	fread(H.val, sizeof(REAL), 9, f);
	fread(Q.val, sizeof(REAL), 16, f);
	fread(&subpixelSteps, sizeof(Disparity), 1, f);
	ASSERT(imageSize.width > 0 && imageSize.height > 0);

	// read resolution
	int w, h;
	fread(&w, sizeof(int), 1, f);
	fread(&h, sizeof(int), 1, f);
	ASSERT(w > 0 && h > 0);

	// read disparity-map
	disparityMap.create(h,w);
	fread(disparityMap.getData(), sizeof(Disparity), w*h, f);

	// read cost-map
	if (fgetc(f) != EOF) {
		fseek(f, -1, SEEK_CUR);
		costMap.create(h,w);
		fread(costMap.getData(), sizeof(AccumCost), w*h, f);
	}

	fclose(f);
	return true;
} // ImportDisparityDataRaw
// same as above, but imports also the empty border
bool SemiGlobalMatcher::ImportDisparityDataRawFull(const String& fileName, DisparityMap& disparityMap, AccumCostMap& costMap, cv::Size& imageSize, Matrix3x3& H, Matrix4x4& Q, Disparity& subpixelSteps)
{
	if (!ImportDisparityDataRaw(fileName, disparityMap, costMap, imageSize, H, Q, subpixelSteps))
		return false;
	const cv::Size sizeValid(disparityMap.width()-2*halfWindowSizeX,disparityMap.height()-2*halfWindowSizeY);
	const cv::Rect ROI(halfWindowSizeX,halfWindowSizeY, sizeValid.width,sizeValid.height);
	disparityMap = disparityMap(ROI).clone();
	if (!costMap.empty())
		costMap = costMap(ROI).clone();
	return true;
} // ImportDisparityDataRawFull


// export disparity-map as an image (red - maximum disparity, blue - minimum disparity)
Image8U3 SemiGlobalMatcher::DisparityMap2Image(const DisparityMap& disparityMap, Disparity minDisparity, Disparity maxDisparity)
{
	ASSERT(!disparityMap.empty());
	// find min and max values
	if (minDisparity == NO_DISP || maxDisparity == NO_DISP) {
		CLISTDEF0(Disparity) disparities(0, disparityMap.area());
		for (int i=disparityMap.area(); i-- > 0; ) {
			const Disparity disparity = disparityMap[i];
			if (disparity != NO_DISP)
				disparities.emplace_back(disparity);
		}
		if (!disparities.empty()) {
			const std::pair<float,float> th(ComputeX84Threshold<Disparity,float>(disparities));
			minDisparity = (Disparity)ROUND2INT(th.first-th.second);
			maxDisparity = (Disparity)ROUND2INT(th.first+th.second);
		}
		DEBUG_ULTIMATE("\tdisparity range: [%d, %d]", minDisparity, maxDisparity);
	}
	const float sclDepth(1.f/(float)(maxDisparity - minDisparity));
	// create color image
	Image8U3 img(cv::Size(disparityMap.width()+2*halfWindowSizeX,disparityMap.height()+2*halfWindowSizeY), Pixel8U::BLACK);
	for (int r=0; r<disparityMap.rows; ++r) {
		for (int c=0; c<disparityMap.cols; ++c) {
			const Disparity disparity(disparityMap(r,c));
			if (disparity == NO_DISP)
				continue;
			img(r+halfWindowSizeY,c+halfWindowSizeX) = Pixel8U::gray2color(CLAMP((float)(maxDisparity-disparity)*sclDepth, 0.f, 1.f));
		}
	}
	return img;
} // DisparityMap2Image
bool SemiGlobalMatcher::ExportDisparityMap(const String& fileName, const DisparityMap& disparityMap, Disparity minDisparity, Disparity maxDisparity)
{
	if (disparityMap.empty())
		return false;
	return DisparityMap2Image(disparityMap, minDisparity, maxDisparity).Save(fileName);
} // ExportDisparityMap


// export point-cloud
bool SemiGlobalMatcher::ExportPointCloud(const String& fileName, const Image& imageData, const DisparityMap& disparityMap, const Matrix4x4& Q, Disparity subpixelSteps)
{
	ASSERT(!disparityMap.empty());

	// vertex definition
	struct Vertex {
		float x,y,z;
		uint8_t r,g,b;
	};
	// list of property information for a vertex
	static PLY::PlyProperty vert_props[] = {
		{"x", PLY::Float32, PLY::Float32, offsetof(Vertex,x), 0, 0, 0, 0},
		{"y", PLY::Float32, PLY::Float32, offsetof(Vertex,y), 0, 0, 0, 0},
		{"z", PLY::Float32, PLY::Float32, offsetof(Vertex,z), 0, 0, 0, 0},
		{"red", PLY::Uint8, PLY::Uint8, offsetof(Vertex,r), 0, 0, 0, 0},
		{"green", PLY::Uint8, PLY::Uint8, offsetof(Vertex,g), 0, 0, 0, 0},
		{"blue", PLY::Uint8, PLY::Uint8, offsetof(Vertex,b), 0, 0, 0, 0},
	};
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex"
	};

	// count the valid disparities, both to size the write buffer and to avoid
	// creating an empty file for a disparity-map without any valid disparity
	const Disparity* const disparities = disparityMap.ptr<const Disparity>();
	const size_t nPoints((size_t)std::count_if(disparities, disparities+disparityMap.area(),
		[](Disparity disparity) { return disparity != NO_DISP; }));
	if (nPoints == 0)
		return false;

	// create PLY object
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);
	const size_t memBufferSize(PLY::ComputeMemBufferSize(nPoints, sizeof(float)*3 + sizeof(uint8_t)*3));
	PLY ply;
	if (!ply.write(fileName, 1, elem_names, PLY::BINARY_LE, memBufferSize))
		return false;

	// describe what properties go into the vertex elements
	ply.describe_property("vertex", 6, vert_props);

	// export the array of 3D points
	Vertex vertex;
	for (int r=0; r<disparityMap.rows; ++r) {
		for (int c=0; c<disparityMap.cols; ++c) {
			const Disparity& disparity = disparityMap(r,c);
			if (disparity == NO_DISP)
				continue;
			Point2f u;
			Depth depth(Image::Disparity2Depth(Q, ImageRef(c+halfWindowSizeX,r+halfWindowSizeY), (float)disparity/subpixelSteps, u));
			if (depth <= 0)
				continue;
			if (!imageData.image.isInsideWithBorder<float,1>(u))
				continue;
			const Point3f X(imageData.camera.TransformPointI2W(Point3(u,depth)));
			vertex.x = X.x; vertex.y = X.y; vertex.z = X.z;
			const Pixel8U C(imageData.image.empty() ? Pixel8U::WHITE : imageData.image.sample(u));
			vertex.r = C.r; vertex.g = C.g; vertex.b = C.b;
			ply.put_element(&vertex);
		}
	}
	if (ply.get_current_element_count() == 0)
		return false;

	// write to file
	return ply.header_complete();
} // ExportPointCloud

// imports a DIMAP file and converts it to point-cloud
bool SemiGlobalMatcher::ImportPointCloud(const String& fileName, const ImageArr& images, PointCloud& pointcloud)
{
	// load disparity-map
	Disparity subpixelSteps;
	cv::Size imageSize; Matrix3x3 H; Matrix4x4 Q;
	DisparityMap disparityMap; AccumCostMap costMap;
	if (!ImportDisparityDataRawFull(fileName, disparityMap, costMap, imageSize, H, Q, subpixelSteps))
		return false;
	// parse image index from the file name
	const String name(Util::getFileName(fileName));
	IIndex idxImage(NO_ID), idxImagePair(NO_ID);
	if (sscanf(name, "%u_%u", &idxImage, &idxImagePair) != 2 || idxImage == NO_ID || idxImagePair == NO_ID)
		return false;
	const Image& imageData = images[idxImage];
	ASSERT(imageData.image.size() == imageSize);
	// import the array of 3D points
	for (int r=0; r<disparityMap.rows; ++r) {
		for (int c=0; c<disparityMap.cols; ++c) {
			const Disparity& disparity = disparityMap(r,c);
			if (disparity == NO_DISP)
				continue;
			Point2f u;
			Depth depth(Image::Disparity2Depth(Q, ImageRef(c+halfWindowSizeX,r+halfWindowSizeY), (float)disparity/subpixelSteps, u));
			if (depth <= 0)
				continue;
			if (!imageData.image.isInsideWithBorder<float,1>(u))
				continue;
			pointcloud.points.emplace_back(Cast<PointCloud::Point::Type>(imageData.camera.TransformPointI2W(Point3(u,depth))));
			pointcloud.colors.emplace_back(imageData.image.empty() ? Pixel8U::WHITE : imageData.image.sample(u));
		}
	}
	return true;
} // ImportPointCloud
/*----------------------------------------------------------------*/


bool MVS::STEREO::ExportCamerasEngin(const Scene& scene, const String& fileName)
{
	ASSERT(!scene.IsEmpty());

	File f(fileName, File::WRITE, File::CREATE | File::TRUNCATE);
	if (!f.isOpen())
		return false;

	// write header
	f.print("n_cameras %u\n", scene.images.size());
	f.print("n_points %u\n", 0);

	// write cameras
	for (const Image& image: scene.images) {
		if (!image.IsValid())
			continue;
		const Point3 t(image.camera.GetT());
		f.print("%u %u %u %s "
			"%g %g %g %g "
			"%g %g %g %g %g %g %g %g %g "
			"%g %g %g "
			"%g %g "
			"%u",
			image.ID, image.width, image.height, image.name.c_str(),
			image.camera.K(0,0), image.camera.K(1,1), image.camera.K(0,2), image.camera.K(1,2),
			image.camera.R(0,0), image.camera.R(0,1), image.camera.R(0,2),
			image.camera.R(1,0), image.camera.R(1,1), image.camera.R(1,2),
			image.camera.R(2,0), image.camera.R(2,1), image.camera.R(2,2),
			t.x, t.y, t.z,
			0, 0,
			image.neighbors.size()
		);
		for (const auto& neighbor: image.neighbors)
			f.print(" %u", neighbor.ID);
		f.print("\n");
	}

	return true;
} // ExportCamerasEngin
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
