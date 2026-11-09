/*
* DepthMap.h
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

#ifndef _MVS_DEPTHMAP_H_
#define _MVS_DEPTHMAP_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "PointCloud.h"
#include <atomic>


// D E F I N E S ///////////////////////////////////////////////////

// NCC type used for patch-similarity computation during depth-map estimation
#define DENSE_NCC_DEFAULT 0
#define DENSE_NCC_FAST 1
#define DENSE_NCC_WEIGHTED 2
#define DENSE_NCC DENSE_NCC_WEIGHTED

// NCC score aggregation type used during depth-map estimation
#define DENSE_AGGNCC_NTH 0
#define DENSE_AGGNCC_MEAN 1
#define DENSE_AGGNCC_MIN 2
#define DENSE_AGGNCC_MINMEAN 3
#define DENSE_AGGNCC DENSE_AGGNCC_MINMEAN

// type of smoothness used during depth-map estimation
#define DENSE_SMOOTHNESS_NA 0
#define DENSE_SMOOTHNESS_FAST 1
#define DENSE_SMOOTHNESS_PLANE 2
#define DENSE_SMOOTHNESS DENSE_SMOOTHNESS_PLANE

// type of refinement used during depth-map estimation
#define DENSE_REFINE_ITER 0
#define DENSE_REFINE_EXACT 1
#define DENSE_REFINE DENSE_REFINE_ITER

// exp function type used during depth estimation
#define DENSE_EXP_DEFUALT EXP
#define DENSE_EXP_FAST FEXP<true> // ~10% faster, but slightly less precise
#define DENSE_EXP DENSE_EXP_DEFUALT

#define ComposeDepthFilePath(i, e) MAKE_PATH(String::FormatString(("depth%04u." + String(e)).c_str(), i))


// S T R U C T S ///////////////////////////////////////////////////
namespace MVS {
	typedef TMatrix<uint8_t, 4, 1> ViewsID;
}
DEFINE_CVDATATYPE(MVS::ViewsID)

namespace MVS {

DECOPT_SPACE(OPTDENSE)

namespace OPTDENSE {
// configuration variables
enum DepthFlags {
	REMOVE_SPECKLES        = (1 << 0),
	FILL_GAPS              = (1 << 1),
	// default: enable ADJUST_CONFIDENCE only when the recalibration is nearly free, i.e. when CUDA
	// estimates the depth-maps and the confidence runs fused into the last geometric-consistency
	// iteration off the already-resident buffers. On the CPU the recalibration is a separate
	// full-resolution sweep costing roughly as much as a fusion pass, so it stays off unless the
	// user asks for it explicitly (--postprocess-dmaps 8). Scene::ComputeDepthMaps resolves this to
	// either ADJUST_CONFIDENCE or 0 once the estimation backend is known; it never survives past that.
	// Bit values are chosen for config back-compat: AUTO recycles the retired ADJUST_CONFIDENCE_FAST
	// bit (whose "cheap adjust" meaning it inherits), and ADJUST_CONFIDENCE keeps its historical
	// value, so existing configs and scripts keep their behavior.
	ADJUST_CONFIDENCE_AUTO = (1 << 2),
	ADJUST_CONFIDENCE      = (1 << 3), // recalibrate confidence to predict fusion survival (see DepthMapsData::AdjustConfidence)
	OPTIMIZE               = (REMOVE_SPECKLES|FILL_GAPS)
};
enum FuseMode {
	FUSE_NOFILTER = 0,
	FUSE_FILTER,
	FUSE_DENSEFILTER,
};
extern MVS_API unsigned nResolutionLevel;
extern MVS_API unsigned nMaxResolution;
extern MVS_API unsigned nMinResolution;
extern MVS_API unsigned nSubResolutionLevels;
extern MVS_API unsigned nMinViews;
extern MVS_API unsigned nMaxViews;
extern MVS_API unsigned nMinViewsFuse;
extern MVS_API unsigned nMaxViewsFuse;
extern MVS_API unsigned nMinViewsTrustPoint;
extern MVS_API unsigned nNumViews;
extern MVS_API unsigned nMinPixelsFuse;
extern MVS_API unsigned nMaxPointsFuse;
extern MVS_API unsigned nMaxFuseDepth;
extern MVS_API bool bAddCorners;
extern MVS_API bool bInitSparse;
extern MVS_API bool bRemoveDmaps;
extern MVS_API float fViewMinScore;
extern MVS_API float fViewMinScoreRatio;
extern MVS_API float fMinArea;
extern MVS_API float fMinAngle;
extern MVS_API float fOptimAngle;
extern MVS_API float fMaxAngle;
extern MVS_API float fWeightPointInsideROI;
extern MVS_API float fDescriptorMinMagnitudeThreshold;
extern MVS_API float fDepthReprojectionErrorThreshold;
extern MVS_API float fDepthDiffThreshold;
extern MVS_API float fNormalDiffThreshold;
extern MVS_API float fPairwiseMul;
extern MVS_API float fOptimizerEps;
extern MVS_API int nOptimizerMaxIters;
extern MVS_API unsigned nSpeckleSize;
extern MVS_API unsigned nIpolGapSize;
extern MVS_API int nIgnoreMaskLabel;
extern MVS_API unsigned nOptimize;
extern MVS_API unsigned nFuseFilter;
extern MVS_API unsigned nEstimateColors;
extern MVS_API unsigned nEstimateNormals;
extern MVS_API float fNCCThresholdKeep;
// confidence recalibration (DepthMapsData::AdjustConfidence) - see DepthFlags::ADJUST_CONFIDENCE.
// The posterior's shape constants are NOT exposed: they are a single jointly ground-truth-calibrated
// operating point living in ConfidenceRefine.h (see the note there).
// DenseFuseDepthMaps: weight of the intra-map prior as virtual view/pixel support to keep few-view
// inliers (0 disables). The rescue is only granted to points no view contradicts (see
// nFuseViolationMax), which is what makes the generous default 4 pay off on both completeness- and
// accuracy-bound scenes.
extern MVS_API float fFusePriorWeight;
// contradiction guard of fusion's keep-rule, counted during fusion's own join gate from DISTINCT views:
// a point RESCUED by fFusePriorWeight's virtual support may be contradicted by at most N views, either
// seeing behind it (free-space violation) or agreeing with its depth but disputing its normal; a point
// kept on real support alone is dropped when the views disputing its normal outnumber its supporting
// views. Default 0; -1 disables the guard entirely (byte-identical to fusion without it).
extern MVS_API int nFuseViolationMax;
// DenseFuseDepthMaps: hand the pixels of a cluster the keep-rule dropped back to the pool, so that a
// later seed or probe can still use them -- a pixel is otherwise marked consumed for good, so one
// doomed early cluster locks away every pixel a later cluster needed. A completeness-for-precision
// trade (T&T: +17..42% points and +1.6..+5.0pp completeness for -1.3..-3.3pp precision), so it is
// off by default: worth enabling when the dense point-cloud itself is the final output, not when a
// mesh reconstruction follows (which interpolates the extra points anyway).
extern MVS_API bool bFuseRecycleDropped;
extern MVS_API bool bEstimateConfidenceCUDA; // when CUDA estimation is used, run ADJUST_CONFIDENCE on the GPU integrated into the last geometric-consistency iteration (default); 0 forces the CPU version
extern MVS_API unsigned nEstimationIters;
extern MVS_API unsigned nEstimationGeometricIters;
extern MVS_API unsigned nPatchMatchCUDAInstances;
extern MVS_API float fEstimationGeometricWeight;
extern MVS_API unsigned nRandomIters;
extern MVS_API unsigned nRandomMaxScale;
extern MVS_API float fRandomDepthRatio;
extern MVS_API float fRandomAngle1Range;
extern MVS_API float fRandomAngle2Range;
extern MVS_API float fRandomSmoothDepth;
extern MVS_API float fRandomSmoothNormal;
extern MVS_API float fRandomSmoothBonus;
} // namespace OPTDENSE
/*----------------------------------------------------------------*/


typedef TImage<ViewsID> ViewsMap;

template <int nTexels>
struct WeightedPatchFix {
	struct Pixel {
		float weight;
		float tempWeight;
	};
	Pixel weights[nTexels];
	float sumWeights;
	float normSq0;
	WeightedPatchFix() : normSq0(0) {}
};

struct MVS_API DepthData {
	struct ViewData {
		float scale; // image scale relative to the reference image
		Camera camera; // camera matrix corresponding to this image
		Image32F image; // image float intensities
		Image* pImageData; // image data

		Matrix3x3 Hl; //
		Vec3 Hm;      // constants during per-pixel loops
		Matrix3x3 Hr; //

		DepthMap depthMap; // known depth-map (optional)
		Camera cameraDepthMap; // camera matrix corresponding to the depth-map
		Matrix3x3f Tl; //
		Point3f Tm;    // constants during per-pixel geometric-consistent loops
		Matrix3x3f Tr; //
		Point3f Tn;    //

		// the pose of this view relative to the reference, for the warps of any camera model: the
		// reference camera-frame point X is Rrel*X+trel in this view's camera frame, RrelDM*X+trelDM in
		// the frame of its depth-map camera
		Eigen::Matrix3f Rrel;
		Eigen::Vector3f trel;
		Eigen::Matrix3f RrelDM;
		Eigen::Vector3f trelDM;

		// this neighbor's own normal/confidence maps, loaded ALONGSIDE depthMap (same
		// disk file, same InitViews() call -- no extra file open) ONLY during the LAST
		// geometric-consistency iteration when the integrated confidence runs (see
		// InitViews' loadDepthMaps==2 path); empty otherwise. Lets the integrated
		// DepthMapsData::AdjustConfidence(DepthData&) overload reuse this reference's own
		// already-loaded, disk-snapshotted neighbor copies instead of the standalone
		// postprocess phase's shared arrDepthData[] lookup.
		NormalMap normalMap;
		ConfidenceMap confMap;

		inline void Init(const Camera& cameraRef) {
			Hl = camera.K * camera.R * cameraRef.R.t();
			Hm = camera.K * camera.R * (cameraRef.C - camera.C);
			Hr = cameraRef.K.inv();
			const Matrix3x3 R(camera.R * cameraRef.R.t());
			const Vec3 t(camera.R * (cameraRef.C - camera.C));
			Rrel = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(R.val).cast<float>();
			trel = Eigen::Map<const Eigen::Vector3d>(t.val).cast<float>();
			if (!depthMap.empty()) {
				Tl = cameraDepthMap.K * cameraDepthMap.R * cameraRef.R.t();
				Tm = cameraDepthMap.K * cameraDepthMap.R * (cameraRef.C - cameraDepthMap.C);
				Tr = cameraRef.K * cameraRef.R * cameraDepthMap.R.t() * cameraDepthMap.GetInvK();
				Tn = cameraRef.K * cameraRef.R * (cameraDepthMap.C - cameraRef.C);
				const Matrix3x3 RDM(cameraDepthMap.R * cameraRef.R.t());
				const Vec3 tDM(cameraDepthMap.R * (cameraRef.C - cameraDepthMap.C));
				RrelDM = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(RDM.val).cast<float>();
				trelDM = Eigen::Map<const Eigen::Vector3d>(tDM.val).cast<float>();
			}
		}
		// the view is an equirectangular panorama (else a pinhole camera: Brown images are read undistorted)
		inline bool IsSpherical() const {
			return pImageData->model == CameraModelType::SPHERICAL;
		}
		// homography from reference pixels to this view's pixels induced by the plane through the
		// reference camera-frame point X0*depth with the given normal (pinhole cameras, cached constants)
		inline Matrix3x3f ComputeHomography(const Point3& X0, Depth depth, const Normal& normal) const {
			const Vec3 n(normal);
			return (Hl + Hm * (n.t()*INVERT(n.dot(X0)*depth))) * Hr;
		}

		inline IIndex GetID() const {
			return pImageData->ID;
		}
		inline IIndex GetLocalID(const ImageArr& images) const {
			return (IIndex)(pImageData - images.begin());
		}

		static bool NeedScaleImage(float scale) {
			ASSERT(scale > 0);
			return ABS(scale-1.f) >= 0.15f;
		}
		template <typename IMAGE>
		static bool ScaleImage(const IMAGE& image, IMAGE& imageScaled, float scale) {
			if (!NeedScaleImage(scale))
				return false;
			// the camera of the scaled view comes from GetCamera(), so scale on its uniform grid
			Image::ResizeUniform(image, imageScaled, Image8U::computeResize(image.size(), scale), scale>1?cv::INTER_CUBIC:cv::INTER_AREA);
			return true;
		}
	};
	typedef CLISTDEF2IDX(ViewData,IIndex) ViewDataArr;

	ViewDataArr images; // array of images used to compute this depth-map (reference image is the first)
	ViewScoreArr neighbors; // array of all images seeing this depth-map (ordered by decreasing importance)
	IndexArr points; // indices of the sparse 3D points seen by the this image
	BitMatrix mask; // mark pixels to be ignored
	DepthMap depthMap; // depth-map
	NormalMap normalMap; // normal-map in camera space
	ConfidenceMap confMap; // confidence-map
	ConfidenceMap confMapAdjusted; // recalibrated confidence-map computed by AdjustConfidence(), held
		// in memory until the deferred EVT_ADJUSTDEPTHMAP swap (confMap = move(confMapAdjusted));
		// intentionally NOT cleared by Release() so it survives a cache eviction/reload of this
		// DepthData between the filter and adjust events
	ConfidenceMap priorMap; // intra-map geometric prior (DepthMapsData::ComputeIntraMapPrior), lazily
		// computed and cached by DepthMapsData::GetIntraMapPrior() so AdjustConfidence and
		// DenseFuseDepthMaps can share one computation instead of each recomputing its own; unlike
		// confMapAdjusted this IS cleared by Release() -- it is a cheap, recomputable derived cache
		// with no cross-event delivery obligation, so it simply follows the DepthData's own lifetime
	ViewsMap viewsMap; // view-IDs map (indexing images vector starting after first view)
	float dMin, dMax; // global depth range for this image
	cv::Size size; // image size used to estimate this depth-map
	bool bConfAdjusted; // the confidence recalibration already ran for this view -- either fused
		// into the last geometric-consistency estimation itself (resident-buffer reuse, see
		// DepthMapsData::EstimateDepthMap) or restored by Load() from the dmap header's
		// CONF_ADJUSTED flag (Save() persists it there; it is NOT part of the MVS scene
		// serialization) -- so no adjust pass (epilogue or standalone) may recalibrate it again
	unsigned references; // how many times this depth-map is referenced (on 0 can be safely unloaded)
	CriticalSection cs; // used to count references

	inline DepthData() : bConfAdjusted(false), references(0) {}
	DepthData(const DepthData&);

	inline void ReleaseImages() {
		for (ViewData& image: images) {
			image.image.release();
			image.depthMap.release();
			image.normalMap.release(); // neighbor normal/conf loaded only for the last
			image.confMap.release();   // geometric-consistency iteration (see ViewData comment above)
		}
	}
	inline void Release() {
		depthMap.release();
		normalMap.release();
		confMap.release();
		priorMap.release();
		viewsMap.release();
	}

	inline bool IsValid() const {
		return !images.IsEmpty();
	}
	inline bool IsEmpty() const {
		return depthMap.empty();
	}

	const ViewData& GetView() const { return images.front(); }
	const Camera& GetCamera() const { return GetView().camera; }

	void GetNormal(const ImageRef& ir, Point3f& N, const TImage<Point3f>* pPointMap=NULL) const;
	void GetNormal(const Point2f& x, Point3f& N, const TImage<Point3f>* pPointMap=NULL) const;

	void ApplyIgnoreMask(const BitMatrix&);

	bool Save(const String& fileName) const;
	bool Load(const String& fileName, unsigned flags=HeaderDepthDataRaw::CONTENT_MASK);

	unsigned GetRef();
	unsigned IncRef(const String& fileName);
	unsigned DecRef();

	size_t GetMemorySize() const;

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ASSERT(IsValid());
		ar & depthMap;
		ar & normalMap;
		ar & confMap;
		ar & viewsMap;
		ar & dMin;
		ar & dMax;
	}
	#endif
};
typedef CLISTDEFIDX(DepthData,IIndex) DepthDataArr;
/*----------------------------------------------------------------*/


// the camera model of a view: pinhole and spherical models are both parametrized by the focal
// lengths and the principal point of the view's K
template<typename MODEL>
inline MODEL MakeCameraModel(const Camera& camera) {
	typedef typename MODEL::Scalar T;
	ASSERT(ISZERO(camera.K(0,1)));
	return MODEL{T(camera.K(0,0)), T(camera.K(1,1)), T(camera.K(0,2)), T(camera.K(1,2))};
}

// patch warp into a target view: the homogeneous target pixel of the patch's top-left texel, and
// its steps along a patch row (dx) and down a patch column (dy); dehomogenized per texel
struct PatchWarp {
	Point3f base, dx, dy;
};

// the part of the PatchMatch estimator independent of the camera model: patch layout, pixel order
// and the shared types
struct MVS_API DepthEstimatorBase {
	enum { nSizeHalfWindow = 4 };
	enum { nSizeWindow = nSizeHalfWindow*2+1 };
	enum { nSizeStep = 2 };
	enum { TexelChannels = 1 };
	enum { nTexels = SQUARE((nSizeHalfWindow*2+nSizeStep)/nSizeStep)*TexelChannels };

	enum ENDIRECTION {
		LT2RB = 0,
		RB2LT,
		DIRS
	};

	typedef TPoint2<uint16_t> MapRef;
	typedef CLISTDEF0(MapRef) MapRefArr;

	typedef Eigen::Matrix<float,nTexels,1> TexelVec;
	struct NeighborData {
		ImageRef x;
		Depth depth;
		Normal normal;
	};
	#if DENSE_SMOOTHNESS != DENSE_SMOOTHNESS_NA
	struct NeighborEstimate {
		Depth depth;
		Normal normal;
		#if DENSE_SMOOTHNESS == DENSE_SMOOTHNESS_PLANE
		Planef::POINT X;
		#endif
	};
	#endif
	#if DENSE_REFINE == DENSE_REFINE_EXACT
	struct PixelEstimate {
		Depth depth;
		Normal normal;
	};
	#endif

	#if DENSE_NCC == DENSE_NCC_WEIGHTED
	typedef WeightedPatchFix<nTexels> Weight;
	typedef CLISTDEFIDX(Weight,int) WeightMap;
	#endif

	// per-pixel state of a propagation pass: 1 while the pixel is not estimated yet in this pass, 2 while
	// its estimate is written to the maps, 0 once it is
	typedef std::atomic<uint8_t> PixelPending;

	// the mask stored in the scene, else the file next to the image, resized into a local buffer (the
	// Image is never touched, so it is safe to call concurrently for the same Image and across scales);
	// bmask gets the keep-mask (bit set = keep), or pMask the mask != label if given; returns false
	// and leaves both untouched if there is no mask to load
	static bool ImportIgnoreMask(const Image&, const cv::Size&, int nIgnoreMaskLabel, BitMatrix&, Image8U* =NULL);
	static void MapMatrix2ZigzagIdx(const cv::Size& size, MapRefArr& coords, const BitMatrix& mask, int rawStride=16);

	static inline Point3 ComputeRelativeC(const DepthData& depthData) {
		return depthData.images[1].camera.R*(depthData.images[0].camera.C-depthData.images[1].camera.C);
	}
	static inline Matrix3x3 ComputeRelativeR(const DepthData& depthData) {
		RMatrix R;
		ComputeRelativeRotation(depthData.images[0].camera.R, depthData.images[1].camera.R, R);
		return R;
	}

	static const float scaleRanges[12];

	// a spherical target view is sampled through the tangent-plane homography of the patch while the
	// elevation of the patch center seen from the target is at most this (degrees), and through the
	// exact per-texel warp above it, where the homography error grows as 1/cos(elevation)
	static float fSphericalWarpMaxElevation;
};

// PatchMatch depth-map estimator of a reference view seen through the camera model MODEL
// (Math/Camera/CameraModels.h): a depth is the distance lambda along the model's ray, so the
// camera-frame point of pixel x is lambda*Ray(x). The model enters through the reference ray X0,
// the DepthGeometry.h functions called with camera0f, and the members ComputePatchWarp,
// ComputeGeometricConsistency and ComputeNeighborPoint, specialized per reference model in
// DepthMap.cpp; the target model of a neighbor view is resolved there once per (pixel, hypothesis,
// view), never per texel.
// Concurrent estimators pull pixels in the zigzag order of coords; an estimate is written to the maps
// as a whole at the end of its pixel, so a neighbor's estimate is read whole: the one of this pass if
// that neighbor is done, else the previous pass's (a single thread reads lower neighbors final and
// higher ones from the previous pass). The columns of a spherical reference wrap: the first and the
// last column are neighbors.
template<template<typename> class MODEL>
struct DepthEstimatorT : DepthEstimatorBase {
	typedef MODEL<REAL> Model; // in the precision of the reference rays
	typedef MODEL<float> Modelf; // in the precision of the depth geometry
	static constexpr bool bSphericalRef = std::is_same<Model, Spherical<REAL>>::value;

	SEACAVE::Random rnd;

	volatile Thread::safe_t& idxPixel; // current image index to be processed
	PixelPending* const pending; // per pixel of the reference image, its state in this propagation pass
	#if DENSE_SMOOTHNESS == DENSE_SMOOTHNESS_NA
	CLISTDEF0IDX(NeighborData,IIndex) neighbors; // neighbor pixels coordinates to be processed
	#else
	CLISTDEF0IDX(ImageRef,IIndex) neighbors; // neighbor pixels coordinates to be processed
	#endif
	#if DENSE_SMOOTHNESS != DENSE_SMOOTHNESS_NA
	CLISTDEF0IDX(NeighborEstimate,IIndex) neighborsClose; // close neighbor pixel depths to be used for smoothing
	#endif
	CLISTDEF0IDX(float,IIndex) neighborsConf; // the confidence of each of the neighbors
	Point3 X0;	      //
	ImageRef x0;	  // constants during one pixel loop
	float normSq0;	  //
	Matrix3x3f normalFrame0; // spherical reference: the frame random and perturbed normals are drawn in
		// (columns: the longitude and the downward elevation tangents at X0, and X0)
	Eigen::Matrix3f rayFrame0; // Model::RayFrame at x0, if bModelWarp
	CLISTDEF0IDX(Eigen::Matrix3f,IIndex) viewRayFrames0; // per view, Rrel*rayFrame0: the ray frame at x0 in the view's camera frame
	struct TexelRays { Eigen::Vector3f rays[nTexels]; };
	TexelRays texelRays0; // the rays of the patch texels at x0, for the exact warp, filled on first use
	CLISTDEF0IDX(TexelRays,IIndex) viewTexelRays0; // per view, Rrel*texelRays0, filled on first use
	CLISTDEF0IDX(bool,IIndex) bViewTexelRays0; // per view, viewTexelRays0 is filled for x0 (the last entry: texelRays0)
	#if DENSE_NCC != DENSE_NCC_WEIGHTED
	TexelVec texels0; //
	#endif
	#if DENSE_NCC == DENSE_NCC_DEFAULT
	TexelVec texels1;
	#endif
	#if DENSE_AGGNCC == DENSE_AGGNCC_NTH || DENSE_AGGNCC == DENSE_AGGNCC_MINMEAN
	FloatArr scores;
	#else
	Eigen::VectorXf scores;
	#endif
	#if DENSE_SMOOTHNESS == DENSE_SMOOTHNESS_PLANE
	Planef plane; // plane defined by current depth and normal estimate
	#endif
	DepthMap& depthMap0;
	NormalMap& normalMap0;
	ConfidenceMap& confMap0;
	#if DENSE_NCC == DENSE_NCC_WEIGHTED
	WeightMap& weightMap0;
	#endif
	DepthMap lowResDepthMap;

	const unsigned nIteration; // current PatchMatch iteration
	const DepthData::ViewDataArr images; // neighbor images used
	const DepthData::ViewData& image0;
	#if DENSE_NCC != DENSE_NCC_WEIGHTED
	const Image64F& image0Sum; // integral image used to fast compute patch mean intensity
	#endif
	const MapRefArr& coords;
	const Image8U::Size size;
	const Model camera0; // image0's camera model
	const Modelf camera0f; // the same, single precision
	const Depth dMin, dMax;
	const Depth dMinSqr, dMaxSqr;
	const ENDIRECTION dir;
	const bool bModelWarp; // a spherical camera is involved: some view is warped through the camera models
	#if DENSE_AGGNCC == DENSE_AGGNCC_NTH || DENSE_AGGNCC == DENSE_AGGNCC_MINMEAN
	const IDX idxScore;
	#endif

	DepthEstimatorT(
		unsigned nIter, DepthData& _depthData0, volatile Thread::safe_t& _idx, PixelPending* _pending,
		#if DENSE_NCC == DENSE_NCC_WEIGHTED
		WeightMap& _weightMap0,
		#else
		const Image64F& _image0Sum,
		#endif
		const MapRefArr& _coords);

	bool PreparePixelPatch(const ImageRef&);
	bool FillPixelPatch();
	float ScorePixelImage(const DepthData::ViewData& image1, Depth, const Normal&);
	float ScorePixel(Depth, const Normal&);
	void ProcessPixel(IDX idx);
	void ReadEstimate(const ImageRef&, Depth&, Normal&, float& conf) const;
	void EstimatePixel(Depth&, Normal&, float& conf);
	Depth InterpolatePixel(const ImageRef&, Depth, const Normal&) const;
	// model-dependent: the patch warp of the plane (depth, normal) at x0 into a pinhole target view,
	// the geometric-consistency cost of that depth against the target's depth-map, and the
	// camera-frame point of reference pixel nx at the given depth
	PatchWarp ComputePatchWarp(const DepthData::ViewData& image1, Depth, const Normal&) const;
	float ComputeGeometricConsistency(const DepthData::ViewData& image1, Depth) const;
	Point3f ComputeNeighborPoint(const ImageRef& nx, Depth) const;
	// the texels of the patch of the plane (depth, normal) at x0 as seen in the target view, in the
	// order of the reference texels (nTexels values); false if one of them is not seen there
	inline bool SamplePatch(const DepthData::ViewData& image1, Depth depth, const Normal& normal, float* texels) {
		if constexpr (!bSphericalRef) {
			if (!bModelWarp)
				return SamplePinholePatch(image1, depth, normal, texels);
		}
		return SampleModelPatch(image1, depth, normal, texels);
	}
	// the same between pinhole cameras, through the cached plane homography
	bool SamplePinholePatch(const DepthData::ViewData& image1, Depth, const Normal&, float* texels);
	// the same as soon as a spherical camera is involved in the scene
	bool SampleModelPatch(const DepthData::ViewData& image1, Depth, const Normal&, float* texels);
	// the same through the exact per-texel warp into a target of model TRG (single precision)
	template<typename TRG>
	bool SampleExactPatch(const DepthData::ViewData& image1, const TRG& trg, Depth, const Normal&, float* texels);
	// the geometric-consistency cost against the depth-map of a target of model TRG (single precision)
	template<typename TRG>
	float GeometricConsistency(const DepthData::ViewData& image1, const TRG& trg, Depth) const;
	#if DENSE_SMOOTHNESS == DENSE_SMOOTHNESS_PLANE
	void InitPlane(Depth, const Normal&);
	#endif
	#if DENSE_REFINE == DENSE_REFINE_EXACT
	PixelEstimate PerturbEstimate(const PixelEstimate&, float perturbation);
	#endif

	#if DENSE_NCC != DENSE_NCC_WEIGHTED
	inline float GetImage0Sum(const ImageRef& p) const {
		const ImageRef p0(p.x-nSizeHalfWindow, p.y-nSizeHalfWindow);
		const ImageRef p1(p0.x+nSizeWindow, p0.y);
		const ImageRef p2(p0.x, p0.y+nSizeWindow);
		const ImageRef p3(p1.x, p2.y);
		return (float)(image0Sum(p3) - image0Sum(p2) - image0Sum(p1) + image0Sum(p0));
	}
	#endif

	// the reference image column c: wrapped for a spherical reference, whose patches cross the seam
	inline int Col0(int c) const {
		if constexpr (bSphericalRef)
			return c < 0 ? c + size.width : (c >= size.width ? c - size.width : c);
		return c;
	}

	#if DENSE_NCC == DENSE_NCC_WEIGHTED
	float GetWeight(const ImageRef& x, float center) const {
		// color weight [0..1]
		const float sigmaColor(-1.f/(2.f*SQUARE(0.1f)));
		const float wColor(SQUARE(image0.image(x0.y+x.y, Col0(x0.x+x.x))-center) * sigmaColor);
		// spatial weight [0..1]
		const float sigmaSpatial(-1.f/(2.f*SQUARE((int)nSizeHalfWindow-1)));
		const float wSpatial(float(SQUARE(x.x) + SQUARE(x.y)) * sigmaSpatial);
		return DENSE_EXP(wColor+wSpatial);
	}
	#endif

	// generate random depth and normal
	inline Depth RandomDepth(Depth dMinSqr, Depth dMaxSqr) {
		ASSERT(dMinSqr > 0 && dMinSqr < dMaxSqr);
		return SQUARE(rnd.randomRange(dMinSqr, dMaxSqr));
	}
	inline Normal RandomNormal(const Point3f& viewRay) {
		Normal normal;
		Dir2Normal(Point2f(rnd.randomRange(D2R(0.f),D2R(180.f)), rnd.randomRange(D2R(90.f),D2R(180.f))), normal);
		ASSERT(ISEQUAL(norm(normal), 1.f, 1e-2f), "Norm = ", norm(normal));
		normal = NormalFromLocal(normal);
		return normal.dot(viewRay) > 0 ? -normal : normal;
	}
	// random and perturbed normals are drawn as directions (Dir2Normal) in a frame whose z axis is the
	// viewing ray: the camera frame for a pinhole reference, normalFrame0 for a spherical one, where
	// the normal turned into that frame is unit only to the rounding, hence its polar angle by atan2
	inline void NormalToDir(const Normal& normal, Point2f& p) const {
		if constexpr (bSphericalRef) {
			const Normal local(Matrix3x3f(normalFrame0.t()) * normal);
			p.x = ATAN2(local.y, local.x);
			p.y = ATAN2(SQRT(SQUARE(local.x)+SQUARE(local.y)), local.z);
			return;
		}
		Normal2Dir(normal, p);
	}
	inline Normal NormalFromLocal(const Normal& normal) const {
		if constexpr (bSphericalRef)
			return Normal(normalFrame0 * normal);
		return normal;
	}

	// adjust normal such that it makes at most 90 degrees with the viewing angle
	inline void CorrectNormal(Normal& normal) const {
		const Normal viewDir(Cast<float>(X0));
		const float cosAngLen(normal.dot(viewDir));
		if (cosAngLen > 0) {
			// rotation axis = unit(normal x viewDir); RMatrixBaseF requires |axis|==1;
			// if normal is parallel to viewDir, the cross product is zero and no
			// rotation axis exists, so flip the (camera-facing, invalid) normal instead
			const Normal axisRaw(normal.cross(viewDir));
			const float axisN(norm(axisRaw));
			if (!ISZERO(axisN))
				normal = RMatrixBaseF(axisRaw / axisN, MINF((ACOS(cosAngLen/norm(viewDir))-D2R(90.f))*1.01f, -0.001f)) * normal;
			else
				normal = -normal;
		}
		ASSERT(ISEQUAL(norm(normal), 1.f, 1e-2f), "Norm = ", norm(normal));
	}

	const float smoothBonusDepth, smoothBonusNormal;
	const float smoothSigmaDepth, smoothSigmaNormal;
	const float thMagnitudeSq;
	const float angle1Range, angle2Range;
	const float thConfSmall, thConfBig, thConfRand;
	const float thRobust;
	#if DENSE_REFINE == DENSE_REFINE_EXACT
	const float thPerturbation;
	#endif
	const float sinSqWarpMaxElevation; // SQUARE(SIN(fSphericalWarpMaxElevation))
};
template<> FORCEINLINE PatchWarp DepthEstimatorT<Pinhole>::ComputePatchWarp(const DepthData::ViewData&, Depth, const Normal&) const;
template<> FORCEINLINE float DepthEstimatorT<Pinhole>::ComputeGeometricConsistency(const DepthData::ViewData&, Depth) const;
template<> FORCEINLINE Point3f DepthEstimatorT<Pinhole>::ComputeNeighborPoint(const ImageRef&, Depth) const;
template<> FORCEINLINE float DepthEstimatorT<Spherical>::ComputeGeometricConsistency(const DepthData::ViewData&, Depth) const;
template<> FORCEINLINE Point3f DepthEstimatorT<Spherical>::ComputeNeighborPoint(const ImageRef&, Depth) const;
// the estimators are instantiated in DepthMap.cpp, and exported for the tests
#ifdef MVS_EXPORTS
extern template struct DepthEstimatorT<Pinhole>;
extern template struct DepthEstimatorT<Spherical>;
#else
extern template struct MVS_API DepthEstimatorT<Pinhole>;
extern template struct MVS_API DepthEstimatorT<Spherical>;
#endif
typedef DepthEstimatorT<Pinhole> DepthEstimator;
/*----------------------------------------------------------------*/


// Tools
bool TriangulatePoints2DepthMap(
	const Camera& camera, const cv::Size& size, const PointCloud& pointcloud, const IndexArr& points,
	DepthMap& depthMap, NormalMap& normalMap, Depth& dMin, Depth& dMax, float avgDepth=0, bool bSparseOnly=false);
bool TriangulatePoints2DepthMap(
	const Camera& camera, const cv::Size& size, const PointCloud& pointcloud, const IndexArr& points,
	DepthMap& depthMap, Depth& dMin, Depth& dMax, float avgDepth=0, bool bSparseOnly=false);

// Robustly estimate the plane that fits best the given points
MVS_API unsigned EstimatePlane(const Point3Arr&, Plane&, double& maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);
MVS_API unsigned EstimatePlaneLockFirstPoint(const Point3Arr&, Plane&, double& maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);
MVS_API unsigned EstimatePlaneTh(const Point3Arr&, Plane&, double maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);
MVS_API unsigned EstimatePlaneThLockFirstPoint(const Point3Arr&, Plane&, double maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);
// same for float points
MVS_API unsigned EstimatePlane(const Point3fArr&, Planef&, double& maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);
MVS_API unsigned EstimatePlaneLockFirstPoint(const Point3fArr&, Planef&, double& maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);
MVS_API unsigned EstimatePlaneTh(const Point3fArr&, Planef&, double maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);
MVS_API unsigned EstimatePlaneThLockFirstPoint(const Point3fArr&, Planef&, double maxThreshold, bool arrInliers[]=NULL, size_t maxIters=0);

// the physical memory every densify stage leaves untouched: room for the OS file
// cache absorbing the depth-map traffic and for the steps that follow
inline size_t ComputeSafetyMemory(const Util::MemoryInfo& memInfo) {
	return MAXF(ROUND2INT<size_t>(memInfo.totalPhysical * 0.08), size_t(1*1024*1024*1024ull)/*1GB*/);
}

// the images are decoded as they are sampled and released right after, so they do
// not have to be loaded by the caller
MVS_API void EstimatePointColors(ImageArr& images, PointCloud& pointcloud);
MVS_API void EstimatePointSegmentation(const ImageArr& images, PointCloud& pointcloud, unsigned minViews=2);
MVS_API unsigned ColorPointSegmentation(PointCloud& pointcloud);
MVS_API void EstimatePointNormals(const ImageArr& images, PointCloud& pointcloud, int numNeighbors=16/*K-nearest neighbors*/);

MVS_API bool EstimateNormalMap(const Matrix3x3f& K, const DepthMap&, NormalMap&);
// the same for an equirectangular depth-map (columns wrapping), depth being the range
inline void EstimateNormalMap(const Spherical<float>& camera, const DepthMap& depthMap, NormalMap& normalMap) {
	normalMap.create(depthMap.size());
	for (int r=0; r<normalMap.rows; ++r) {
		for (int c=0; c<normalMap.cols; ++c) {
			const ImageRef x(c,r);
			Depth depth;
			Eigen::Vector2f gradient;
			normalMap(r,c) = FitDepthGradient(depthMap, x, depth, gradient, true) ?
				Normal(NormalFromDepthGradient(camera, x, depth, gradient)) : Normal::ZERO;
		}
	}
}

// Standalone confidence estimators, deriving a confidence-map from the geometry of one
// depth-map alone -- no images, no neighboring views, no matching cost. The dense
// pipeline does not use them: its own estimates come with a photometric score, which
// DensifyPointCloud can recalibrate against the neighboring views (see
// OPTDENSE::ADJUST_CONFIDENCE). They are meant for the depth-maps that reach the library
// with no confidence at all -- imported from an external estimator (the Interface* apps,
// scripts/python/ImportDMAPs.py) or produced by a method that does not score its
// estimates -- so that fusion, point ordering and the mesh visibility weights have
// something better than a constant to work with. Both fill confMap with values in [0,1]
// at the depth-map resolution, and both are heuristics with no calibrated meaning: they
// rank the pixels of a map against each other, they do not predict an error.
// Confidence from the local depth coherence: a piecewise-smooth surface scores high,
// the isolated depths a mismatch leaves behind score low. n is how many of the nearest
// neighbor depth differences enter the score.
MVS_API void EstimateConfidenceFromDepth(const DepthData& depthData, ConfidenceMap& confMap, int winHalfSize=1, int n=3);
// Confidence from the local normal coherence; needs depthData.normalMap, which
// EstimateNormalMap() can supply from the depth-map itself.
MVS_API void EstimateConfidenceFromNormal(const DepthData& depthData, ConfidenceMap& confMap, int winHalfSize=1);

MVS_API bool SaveDepthMap(const String& fileName, const DepthMap& depthMap);
MVS_API bool LoadDepthMap(const String& fileName, DepthMap& depthMap);
MVS_API bool SaveNormalMap(const String& fileName, const NormalMap& normalMap);
MVS_API bool LoadNormalMap(const String& fileName, NormalMap& normalMap);
MVS_API bool SaveConfidenceMap(const String& fileName, const ConfidenceMap& confMap);
MVS_API bool LoadConfidenceMap(const String& fileName, ConfidenceMap& confMap);

MVS_API unsigned FilterDepthMap(DepthMap& depthMap, NormalMap& normalMap, const ConfidenceMap& confMap, float thConfidence=0.5f);
MVS_API Image8U3 DepthMap2Image(const DepthMap& depthMap, Depth minDepth=FLT_MAX, Depth maxDepth=0);
MVS_API bool ExportDepthMap(const String& fileName, const DepthMap& depthMap, Depth minDepth=FLT_MAX, Depth maxDepth=0);
MVS_API bool ExportNormalMap(const String& fileName, const NormalMap& normalMap);
MVS_API bool ExportConfidenceMap(const String& fileName, const ConfidenceMap& confMap);
MVS_API bool ExportPointCloud(const String& fileName, const Image&, const DepthMap&, const NormalMap&);

MVS_API bool ExportDepthDataRaw(const String&, const String& imageFileName,
	const IIndexArr&, const cv::Size& imageSize,
	const KMatrix&, const RMatrix&, const CMatrix&,
	Depth dMin, Depth dMax,
	const DepthMap&, const NormalMap&, const ConfidenceMap&, const ViewsMap&,
	bool bConfAdjusted=false/*mark the stored confMap as already recalibrated (CONF_ADJUSTED)*/,
	bool bSpherical=false/*the maps belong to an equirectangular view (SPHERICAL)*/);
MVS_API bool ImportDepthDataRaw(const String&, String& imageFileName,
	IIndexArr&, cv::Size& imageSize,
	KMatrix&, RMatrix&, CMatrix&,
	Depth& dMin, Depth& dMax,
	DepthMap&, NormalMap&, ConfidenceMap&, ViewsMap&,
	unsigned flags=HeaderDepthDataRaw::CONTENT_MASK/*maps to read, all of them by default*/,
	bool* pbConfAdjusted=NULL/*receives the stored CONF_ADJUSTED flag*/,
	bool* pbSpherical=NULL/*receives the stored SPHERICAL flag*/);

MVS_API void CompareDepthMaps(const DepthMap& depthMap, const DepthMap& depthMapGT, uint32_t idxImage, float threshold=0.01f);
MVS_API void CompareNormalMaps(const NormalMap& normalMap, const NormalMap& normalMapGT, uint32_t idxImage);
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_DEPTHMAP_H_
