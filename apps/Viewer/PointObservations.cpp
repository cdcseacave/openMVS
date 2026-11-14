/*
 * PointObservations.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
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
#include "PointObservations.h"
#include "Scene.h"

using namespace VIEWER;


// S T R U C T S ///////////////////////////////////////////////////

namespace {

// crop the square patch of the given side, centered at the given pixel and covering
// the given number of image pixels; the area outside a pinhole image is black, an
// equirectangular image has none: past the seam the patch continues on the other side of
// the image, past a pole on the meridian half a turn away, mirrored
Image8U3 CropPatch(const cv::Mat& image, const Point2f& center, float window, int size, bool bSpherical)
{
	ASSERT(image.type() == CV_8UC3 && window > 0 && size > 0);
	// sample up to one image pixel per patch pixel, so shrinking averages instead of aliasing
	const int sampleSize(CLAMP(CEIL2INT(window), size, size*4));
	const double step((double)window/sampleSize);
	const double offset(0.5*(sampleSize-1)*step);
	Image8U3 patch;
	if (bSpherical) {
		// the image position of every sample, brought back onto the image; the columns wrap in remap
		const int width(image.cols), height(image.rows);
		cv::Mat_<float> mapX(sampleSize, sampleSize), mapY(sampleSize, sampleSize);
		for (int r = 0; r < sampleSize; ++r) {
			for (int c = 0; c < sampleSize; ++c) {
				double x(center.x - offset + c*step), y(center.y - offset + r*step);
				// across a pole (the image rows span [-0.5, height-0.5]): mirrored on the meridian half a turn away
				if (y < -0.5) {
					y = -1.0 - y;
					x += 0.5*width;
				} else if (y > height - 0.5) {
					y = 2.0*height - 1.0 - y;
					x += 0.5*width;
				}
				x -= width*std::floor(x/width);
				mapX(r, c) = (float)x;
				// the half row next to a pole interpolates within the image (the row across the pole is half a turn away)
				mapY(r, c) = (float)CLAMP(y, 0.0, height - 1.0);
			}
		}
		cv::remap(image, patch, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_WRAP);
	} else {
		const cv::Matx23d sample2image(
			step, 0, center.x-offset,
			0, step, center.y-offset);
		cv::warpAffine(image, patch, sample2image, cv::Size(sampleSize, sampleSize),
			cv::INTER_LINEAR|cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT);
	}
	if (sampleSize != size)
		cv::resize(patch, patch, cv::Size(size, size), 0, 0, cv::INTER_AREA);
	return patch;
}

} // namespace
/*----------------------------------------------------------------*/


PointObservations::~PointObservations()
{
	++croppedPatches->generation;
	threadPool.purge();
}

void PointObservations::Update(const Window& window)
{
	const Scene& scene = window.GetScene();
	const Scene::Layer* layer = scene.GetActiveLayer();
	if (layer == NULL || window.selectionType != Window::SEL_POINT || !window.HasSelectionIds() ||
		window.GetSelectionId() >= layer->scene.pointcloud.pointViews.size()) {
		if (!observations.empty())
			Clear();
		return;
	}
	if (layer->id != layerID || layer->revision != layerRevision ||
		window.GetSelectionId() != pointIdx || sortByWeight != sortedByWeight) {
		BuildObservations(window);
		RequestPatches(layer->scene);
	} else if (patchScale != requestedScale || zoom != requestedZoom) {
		RequestPatches(layer->scene);
	}
	UploadPatches();
}

void PointObservations::Release()
{
	if (imageCache == nullptr && observations.empty())
		return;
	Clear();
	imageCache.reset();
	imageCacheLayerID = NO_ID;
}

void PointObservations::Clear()
{
	++croppedPatches->generation;
	threadPool.purge();
	{
		std::lock_guard<std::mutex> lock(croppedPatches->mutex);
		croppedPatches->patches.clear();
	}
	observations.clear();
	layerID = pointIdx = NO_ID;
}

void PointObservations::BuildObservations(const Window& window)
{
	const Scene& scene = window.GetScene();
	const Scene::Layer& layer = *scene.GetActiveLayer();
	const MVS::PointCloud& pointcloud = layer.scene.pointcloud;
	layerID = layer.id;
	layerRevision = layer.revision;
	pointIdx = (uint32_t)window.GetSelectionId();
	sortedByWeight = sortByWeight;
	hasWeights = !pointcloud.pointWeights.empty();
	hasFeatures = !pointcloud.pointObservations.empty();
	if (imageCacheLayerID != layerID) {
		// decode the image files at full resolution (the layer images may be loaded rescaled),
		// as their cameras see them: the copies carry only what ReadPixels needs
		std::vector<MVS::Image> files(layer.scene.images.size());
		FOREACH(i, layer.scene.images) {
			const MVS::Image& imageData = layer.scene.images[i];
			files[i].name = imageData.name;
			files[i].model = imageData.model;
			files[i].lens = imageData.lens;
		}
		imageCache = std::make_shared<MVS::ImageCache>([files = std::move(files)](MVS::IIndex idxImage, cv::Mat& image) {
			ASSERT(idxImage < files.size());
			Image8U3 imageColor;
			if (files[idxImage].ReadPixels(imageColor) == NULL)
				return false;
			image = imageColor;
			return true;
		}, Util::GetMemoryInfo().freePhysical/4);
		imageCacheLayerID = layerID;
	}

	const Point3 X(pointcloud.points[pointIdx]);
	const MVS::PointCloud::ViewArr& views = pointcloud.pointViews[pointIdx];
	ASSERT(!views.empty());
	ASSERT(!hasFeatures || pointcloud.pointObservations[pointIdx].size() == views.size());
	observations.clear();
	observations.resize(views.size());
	FOREACH(v, views) {
		const MVS::IIndex idxImage(views[v]);
		const MVS::Image& imageData = layer.scene.images[idxImage];
		ASSERT(imageData.IsValid());
		const MVS::ModelCamera modelCamera(imageData.GetModelCamera());
		const auto [x, depth] = modelCamera.ProjectPointP(X);
		ASSERT(depth > 0);
		Observation& observation = observations[v];
		observation.idxImage = idxImage;
		observation.idxView = scene.ImageIdxMVS2Viewer(idxImage);
		ASSERT(observation.idxView != NO_ID);
		observation.ID = imageData.ID;
		observation.name = Util::getFileNameExt(imageData.name);
		observation.proj = Point2f((float)x.x, (float)x.y);
		observation.width = (float)imageData.width;
		observation.nativeScale = imageData.scale;
		observation.depth = (float)depth;
		observation.footprint = (float)imageData.camera.GetFootprintImage(depth);
		if (hasFeatures) {
			// compare in the current resolution, where a spherical image wraps at its width
			observation.feature = pointcloud.pointObservations[pointIdx][v];
			const Point2f feature(MVS::Camera::ScaleImagePixel(observation.feature, observation.nativeScale));
			observation.featureOffset = Point2f(
				modelCamera.ColumnOffset(feature.x, observation.proj.x, (int)imageData.width),
				feature.y-observation.proj.y) * (1.f/observation.nativeScale);
		}
		observation.weight = hasWeights ? (float)pointcloud.pointWeights[pointIdx][v] : 0.f;
		observation.angle = -1.f;
		if (!pointcloud.normals.empty()) {
			const Point3f viewDir(Cast<float>(imageData.camera.C - X));
			observation.angle = R2D(ACOS(ComputeAngle(pointcloud.normals[pointIdx].ptr(), viewDir.ptr())));
		}
	}
	if (sortByWeight && hasWeights)
		std::stable_sort(observations.begin(), observations.end(),
			[](const Observation& a, const Observation& b) { return a.weight > b.weight; });

	std::vector<float> footprints;
	footprints.reserve(observations.size());
	for (const Observation& observation: observations)
		footprints.push_back(observation.FootprintNative());
	std::nth_element(footprints.begin(), footprints.begin()+footprints.size()/2, footprints.end());
	footprintRef = footprints[footprints.size()/2];
	ASSERT(footprintRef > 0);
}

void PointObservations::RequestPatches(const MVS::Scene& scene)
{
	requestedScale = patchScale;
	requestedZoom = zoom;
	const uint32_t generation(++croppedPatches->generation);
	threadPool.purge();
	const float window(PATCH_SIZE/zoom); // image pixels covered at the reference footprint
	const bool bEqualized(patchScale == SCALE_EQUALIZED);
	for (uint32_t i = 0; i < (uint32_t)observations.size(); ++i) {
		const Observation& observation = observations[i];
		threadPool.detach_task([imageCache = imageCache, croppedPatches = croppedPatches, generation, i,
			window, bEqualized, footprintRef = footprintRef, idxImage = observation.idxImage,
			proj = observation.proj, width = observation.width, spherical = scene.images[observation.idxImage].IsSpherical(), nativeScale = observation.nativeScale,
			footprint = observation.footprint]() {
			// stop as soon as the request is superseded
			if (croppedPatches->generation != generation)
				return;
			cv::Mat image;
			Image8U3 patch;
			float magnification(0);
			if (imageCache->UseImage(idxImage, image)) {
				if (croppedPatches->generation != generation)
					return;
				// the projection is given in the resolution the camera is scaled to
				const float scale((float)image.cols/width);
				const float windowImage(bEqualized ? window*footprint*scale/footprintRef : window);
				patch = CropPatch(image, MVS::Camera::ScaleImagePixel(proj, scale), windowImage, PATCH_SIZE, spherical);
				magnification = PATCH_SIZE*scale*nativeScale/windowImage;
			}
			if (croppedPatches->generation != generation)
				return;
			{
				std::lock_guard<std::mutex> lock(croppedPatches->mutex);
				croppedPatches->patches.push_back({generation, i, magnification, std::move(patch)});
			}
			glfwPostEmptyEvent();
		});
	}
}

void PointObservations::UploadPatches()
{
	std::vector<CroppedPatches::Patch> patches;
	{
		std::lock_guard<std::mutex> lock(croppedPatches->mutex);
		patches.swap(croppedPatches->patches);
	}
	const uint32_t generation(croppedPatches->generation);
	for (const CroppedPatches::Patch& patch: patches) {
		if (patch.generation != generation)
			continue;
		Observation& observation = observations[patch.idxObservation];
		observation.failed = patch.image.empty();
		if (!observation.failed) {
			observation.patch.Create(patch.image);
			observation.patchMagnification = patch.magnification;
		}
	}
}
/*----------------------------------------------------------------*/
