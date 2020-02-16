/*
* SemiGlobalMatcher.h
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

#ifndef _MVS_SEMIGLOBALMATCHER_H_
#define _MVS_SEMIGLOBALMATCHER_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "DepthMap.h"


// D E F I N E S ///////////////////////////////////////////////////

// similarity method
#define SGM_SIMILARITY_WZNCC 1
#define SGM_SIMILARITY_CENSUS 2
#define SGM_SIMILARITY SGM_SIMILARITY_WZNCC


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

class Scene;

namespace STEREO {

// An implementation of the popular Semi-Global Matching (SGM) algorithm.
class MVS_API SemiGlobalMatcher
{
public:
	typedef uint8_t  Mask;      // used to mark the valid region to be estimated
	typedef int16_t  Disparity; // contains the allowable disparity range
	typedef uint8_t  Cost;      // used to describe a single disparity cost
	typedef uint16_t AccumCost; // used to accumulate CostType values
	typedef uint64_t Index; // index pointing to pixel costs/accumulated-costs

	enum : Mask      { INVALID = 0, VALID = DECLARE_NO_INDEX(Mask) }; // invalid/valid mask value
	enum : Disparity { NO_DISP = DECLARE_NO_INDEX(Disparity) }; // invalid disparity value
	enum : AccumCost { NO_ACCUMCOST = DECLARE_NO_INDEX(AccumCost) }; // invalid accumulated cost value
	enum : Index     { NO_INDEX = DECLARE_NO_INDEX(Index) }; // invalid index value

	struct Range {
		Disparity minDisp;
		Disparity maxDisp;
		Disparity avgDisp() const { return (minDisp+maxDisp)>>1; }
		Disparity numDisp() const { return maxDisp-minDisp; }
		Disparity isValid() const { return minDisp<maxDisp; }
	};
	struct PixelData {
		Index idx; // index to pixel costs/accumulated-costs data
		Range range; // first and last disparity evaluated for this pixel
	};

	typedef TImage<Mask> MaskMap; // image of accumulated disparity cost
	typedef TImage<Disparity> DisparityMap; // disparity image type
	typedef TImage<AccumCost> AccumCostMap; // image of accumulated disparity cost
	typedef CLISTDEF0IDX(PixelData,Index) PixelMap; // map of pixel data
	typedef CLISTDEF0IDX(Cost,Index) CostsMap; // map of pre-computed disparity costs
	typedef CLISTDEF0IDX(AccumCost,Index) AccumCostsMap; // map of accumulated disparity costs

	enum : int { numDirs = 4 }; // 2 or 4 directions accumulated per pixel pass
	#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
	enum : int { halfWindowSizeX = 3, halfWindowSizeY = 4 }; // patch kernel size
	#else
	enum : int { halfWindowSizeX = 3, halfWindowSizeY = 3 }; // patch kernel size
	#endif
	enum : int { windowSizeX = halfWindowSizeX*2+1, windowSizeY = halfWindowSizeY*2+1, numTexels = windowSizeX*windowSizeY }; // patch kernel info

	#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
	typedef uint64_t Census; // used to store Census transform
	typedef TImage<Census> CensusMap; // image of Census transforms
	typedef Image8U ImageGray; // used to store intensities image
	STATIC_ASSERT(sizeof(Census)*8 >= numTexels);
	#else
	typedef WeightedPatchFix<numTexels> WeightedPatch; // pre-computed patch weights
	typedef Image32F ImageGray; // used to store normalized float intensities image
	#endif


	// Available sub-pixel options
	enum SgmSubpixelMode {
		SUBPIXEL_NA = 0, // no sub-pixel processing
		SUBPIXEL_LINEAR,
		SUBPIXEL_POLY4,
		SUBPIXEL_PARABOLA,
		SUBPIXEL_SINE,
		SUBPIXEL_COSINE,
		SUBPIXEL_LC_BLEND // probably the best option
	};

	struct ViewData {
		Image8U3 imageColor; // color image
		ImageGray imageGray; // intensity image
		#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
		CensusMap imageCensus; // image of Census transforms (valid per scale)
		#endif
		ViewData GetImage(REAL scale) const {
			if (ISEQUAL(scale, REAL(1))) {
				#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
				CensusTransform(imageGray, const_cast<CensusMap&>(imageCensus));
				#endif
				return *this;
			}
			ASSERT(scale < 1);
			ViewData scaledView;
			cv::resize(imageColor, scaledView.imageColor, cv::Size(), scale, scale, cv::INTER_AREA);
			cv::resize(imageGray, scaledView.imageGray, cv::Size(), scale, scale, cv::INTER_AREA);
			#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
			CensusTransform(scaledView.imageGray, scaledView.imageCensus);
			#endif
			return scaledView;
		}
	};

	typedef MVS_API Point2f DepthRange;
	typedef MVS_API TImage<DepthRange> DepthRangeMap;

public:
	SemiGlobalMatcher(SgmSubpixelMode subpixelMode=SUBPIXEL_LC_BLEND, Disparity subpixelSteps=4, AccumCost P1=3, AccumCost P2=4, float P2alpha=14, float P2beta=38);
	~SemiGlobalMatcher();

	void Match(const Scene& scene, IIndex idxImage, IIndex numNeighbors, unsigned minResolution=320);
	void Fuse(const Scene& scene, IIndex idxImage, IIndex numNeighbors, unsigned minViews, DepthMap& depthMap, ConfidenceMap& confMap);

	static void CreateThreads(unsigned nMaxThreads=1);
	static void DestroyThreads();
	static void* ThreadWorker(void*);
	static void WaitThreadWorkers(unsigned nJobs);

	static bool ExportDisparityDataRaw(const String& fileName, const DisparityMap&, const AccumCostMap&, const cv::Size& imageSize, const Matrix3x3& H, const Matrix4x4& Q, Disparity subpixelSteps);
	static bool ExportDisparityDataRawFull(const String& fileName, const DisparityMap&, const AccumCostMap&, const cv::Size& imageSize, const Matrix3x3& H, const Matrix4x4& Q, Disparity subpixelSteps);
	static bool ImportDisparityDataRaw(const String& fileName, DisparityMap&, AccumCostMap&, cv::Size& imageSize, Matrix3x3& H, Matrix4x4& Q, Disparity& subpixelSteps);
	static bool ImportDisparityDataRawFull(const String& fileName, DisparityMap&, AccumCostMap&, cv::Size& imageSize, Matrix3x3& H, Matrix4x4& Q, Disparity& subpixelSteps);
	static Image8U3 DisparityMap2Image(const DisparityMap&, Disparity minDisparity=NO_DISP, Disparity maxDisparity=NO_DISP);
	static bool ExportDisparityMap(const String& fileName, const DisparityMap&, Disparity minDisparity=NO_DISP, Disparity maxDisparity=NO_DISP);
	static bool ExportPointCloud(const String& fileName, const Image&, const DisparityMap&, const Matrix4x4& Q, Disparity subpixelSteps);
	static bool ImportPointCloud(const String& fileName, const ImageArr& images, PointCloud&);

protected:
	void Match(const ViewData& leftImage, const ViewData& rightImage, DisparityMap& disparityMap, AccumCostMap& costMap);
	Index Disparity2RangeMap(const DisparityMap& disparityMap, const MaskMap& maskMap, Disparity minNumDisp=3, Disparity minNumDispInvalid=16);
	#if SGM_SIMILARITY == SGM_SIMILARITY_CENSUS
	static void CensusTransform(const Image8U& imageGray, CensusMap& imageCensus);
	#endif
	static void ConsistencyCrossCheck(DisparityMap& l2r, const DisparityMap& r2l, Disparity thCross=1);
	static void FilterByCost(DisparityMap&, const AccumCostMap&, AccumCost th);
	static void ExtractMask(const DisparityMap&, MaskMap&, int thValid=3);
	static void FlipDirection(const DisparityMap& l2r, DisparityMap& r2l);
	static void UpscaleMask(MaskMap& maskMap, const cv::Size& size2x);
	void RefineDisparityMap(DisparityMap& disparityMap) const;
	void DisplayState(const cv::Size& size) const;

	static CLISTDEF0IDX(AccumCost,int) GenerateP2s(AccumCost P2, float P2alpha, float P2beta);
	static void Depth2DisparityMap(const DepthMap&, const Matrix3x3& invH, const Matrix4x4& invQ, Disparity subpixelSteps, DisparityMap&);
	static void Disparity2DepthMap(const DisparityMap&, const AccumCostMap&, const Matrix3x3& H, const Matrix4x4& Q, Disparity subpixelSteps, DepthMap&, ConfidenceMap&);
	static bool ProjectDisparity2DepthMap(const DisparityMap&, const AccumCostMap&, const Matrix4x4& Q, Disparity subpixelSteps, DepthMap&, DepthRangeMap&, ConfidenceMap&);

protected:
	// parameters
	SgmSubpixelMode subpixelMode;
	Disparity subpixelSteps;
	AccumCost P1;
	CLISTDEF0IDX(AccumCost,int) P2s;

	// main memory buffers that must be allocated
	PixelMap imagePixels;
	CostsMap imageCosts;
	AccumCostsMap imageAccumCosts;
	Disparity maxNumDisp; // maximum number of disparities per-pixel

	// multi-threading
	static EventThreadPool threads; // worker threads
	static Semaphore sem; // signal job end
};
/*----------------------------------------------------------------*/


MVS_API bool ExportCamerasEngin(const Scene&, const String& fileName);
/*----------------------------------------------------------------*/

} // namespace STEREO

} // namespace MVS

#endif // _MVS_SEMIGLOBALMATCHER_H_
