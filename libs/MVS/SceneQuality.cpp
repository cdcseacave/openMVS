/*
* SceneQuality.cpp
*
* Copyright (c) 2014-2024 SEACAVE
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
#include "Scene.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define QUALITY_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

// Compute reconstruction quality by rendering the textured mesh from each camera viewpoint
// and comparing the rendered image to the original photograph;
// returns a score in [0,100] combining completeness and SSIM
Scene::ReconstructionQuality Scene::ComputeReconstructionQuality(unsigned nMaxResolution) const
{
	ReconstructionQuality quality;
	if (images.empty() || !mesh.HasTexture()) {
		VERBOSE("warning: cannot compute reconstruction quality: %s",
			images.empty() ? "no images" : "mesh has no texture");
		return quality;
	}
	TD_TIMER_STARTD();
	// score each valid image
	quality.imageScores.resize(images.size());
	#ifdef QUALITY_USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	for (int _i = 0; _i < (int)images.size(); ++_i) {
		const IIndex idxImage((IIndex)_i);
	#else
	FOREACH(idxImage, images) {
	#endif
		ImageScore& imageScore = quality.imageScores[idxImage];
		imageScore.idxImage = idxImage;
		// skip invalid images
		Image& image = const_cast<Image&>(images[idxImage]);
		if (!image.IsValid())
			continue;
		// load original image pixels
		if (!image.ReloadImage(platforms, nMaxResolution)) {
			DEBUG_EXTRA("warning: could not load image %u: %s", idxImage, image.name.c_str());
			continue;
		}
		image.UpdateCamera(platforms);
		// render the textured mesh from this camera
		DepthMap depthMap(image.GetSize());
		Image8U3 renderedImage;
		mesh.Project(image.camera, depthMap, renderedImage);
		// build mask from valid depth pixels
		Image8U mask;
		cv::compare(depthMap, 0, mask, cv::CMP_GT);
		// compute completeness: fraction of image covered by mesh
		const int nCovered = cv::countNonZero(mask);
		imageScore.completeness = depthMap.empty() ? 0 : (float)nCovered / depthMap.area();
		if (nCovered == 0) {
			image.ReleaseImage();
			continue;
		}
		// convert to grayscale float [0,255] for SSIM computation
		Image32F originalF, renderedF; {
			Image8U originalGray, renderedGray;
			image.image.toGray(originalGray, cv::COLOR_BGR2GRAY, false);
			renderedImage.toGray(renderedGray, cv::COLOR_BGR2GRAY, false);
			image.ReleaseImage();
			originalGray.convertTo(originalF, CV_32F);
			renderedGray.convertTo(renderedF, CV_32F);
		}
		// compute SSIM in the covered region
		imageScore.ssim = (float)ComputeSSIM(originalF, renderedF, mask);
		// compute PSNR in the covered region for diagnostics
		imageScore.psnr = (float)ComputePSNR(originalF, renderedF, mask);
		DEBUG_EXTRA("\timage %u: completeness=%.1f%% SSIM=%.3f PSNR=%.1fdB score=%.1f",
			idxImage, imageScore.completeness * 100, imageScore.ssim, imageScore.psnr, imageScore.score());
	}
	// aggregate across all scored images
	unsigned nScoredImages = 0;
	for (const auto& imageScore : quality.imageScores) {
		if (imageScore.completeness > 0 || imageScore.ssim > 0) {
			quality.completeness += imageScore.completeness;
			quality.ssim += imageScore.ssim;
			quality.psnr += imageScore.psnr;
			++nScoredImages;
		}
	}
	if (nScoredImages > 0) {
		quality.completeness /= nScoredImages;
		quality.ssim /= nScoredImages;
		quality.psnr /= nScoredImages;
	}
	DEBUG("Reconstruction quality: %.1f (completeness=%.1f%% SSIM=%.3f PSNR=%.1fdB, %u images, %s)",
		quality.score(), quality.completeness * 100, quality.ssim, quality.psnr,
		nScoredImages, TD_TIMER_GET_FMT().c_str());
	return quality;
} // ComputeReconstructionQuality
/*----------------------------------------------------------------*/
