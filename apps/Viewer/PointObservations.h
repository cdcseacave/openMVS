/*
 * PointObservations.h
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

#pragma once

#include "Texture.h"
#include "../../libs/MVS/ImageCache.h"
#include "../../libs/Common/BS_thread_pool.hpp"

namespace VIEWER {

class Window;

// The observations of the selected point: for every view seeing it, the image patch
// centered on the point's projection, and the feature observation stored for that view
// when the point-cloud has them.
//
// The patches are cropped from the full resolution images on a thread pool, through an
// image cache bound to the active layer. The crop tasks use only the values they are
// given and the cache, never the layer, so they do not count as scene background work.
class PointObservations {
public:
	enum PatchScale {
		SCALE_EQUALIZED = 0, // every patch covers the same surface around the point
		SCALE_ORIGINAL // every patch shows the image pixels at the same magnification
	};

	struct Observation {
		MVS::IIndex idxImage = NO_ID; // image index in the MVS scene
		MVS::IIndex idxView = NO_ID; // image index in the Viewer layer
		uint32_t ID = NO_ID; // image ID
		String name; // image file name
		Point2f proj; // point projection, in the resolution the camera is currently scaled to
		float width = 0; // image width the camera is currently scaled to
		float nativeScale = 1; // current resolution relative to the image file (MVS::Image::scale)
		float depth = 0; // point depth
		float footprint = 0; // pixels covered by a unit of surface at the point, in the current resolution
		float weight = 0; // view confidence (0 if the point-cloud has none)
		float angle = 0; // angle between the point normal and the view direction (degrees, -1 if no normal)
		Point2f feature; // stored feature observation, in the image file resolution (valid if the point-cloud has observations)
		Point2f featureOffset; // feature minus projection, in the image file resolution (shortest column offset for a spherical image)
		bool failed{false}; // the image could not be decoded
		Texture patch; // cropped patch, invalid until its crop task finishes
		float patchMagnification = 0; // patch pixels per image file pixel of the uploaded patch

		// the pixel and the footprint in the image file resolution
		Point2f ProjNative() const { return MVS::Camera::ScaleImagePixel(proj, 1.f/nativeScale); }
		float FootprintNative() const { return footprint/nativeScale; }
		// distance between the feature observation and the projection, in image file pixels
		float ReprojectionError() const { return norm(featureOffset); }
	};
	typedef std::vector<Observation> ObservationArr;

	static constexpr int PATCH_SIZE = 128; // patch side (pixels)

	PatchScale patchScale{SCALE_EQUALIZED};
	float zoom{1.5f}; // patch magnification (of the median view when equalized)
	bool sortByWeight{true}; // list the views by decreasing confidence

public:
	PointObservations() = default;
	~PointObservations();

	// follow the selected point and the patch settings, and upload the patches
	// cropped since the last call; must be called from the OpenGL thread
	void Update(const Window& window);
	// drop the observations, the patches and the image cache
	void Release();

	bool IsEmpty() const { return observations.empty(); }
	uint32_t GetPointIdx() const { return pointIdx; }
	bool HasWeights() const { return hasWeights; }
	bool HasFeatures() const { return hasFeatures; }
	const ObservationArr& GetObservations() const { return observations; }

private:
	void Clear();
	void BuildObservations(const Window& window);
	void RequestPatches(const MVS::Scene& scene);
	void UploadPatches();

private:
	// the patches cropped by the tasks, waiting to be uploaded by the OpenGL thread
	struct CroppedPatches {
		struct Patch {
			uint32_t generation;
			uint32_t idxObservation;
			float magnification; // patch pixels per image file pixel
			Image8U3 image; // empty if the image failed to decode
		};
		std::atomic<uint32_t> generation{0}; // latest request; the tasks of older ones stop
		std::mutex mutex;
		std::vector<Patch> patches;
	};

	// the selection the observations were built for
	uint32_t layerID{NO_ID};
	uint32_t layerRevision{0};
	uint32_t pointIdx{NO_ID};
	bool sortedByWeight{true};
	bool hasWeights{false};
	bool hasFeatures{false};
	ObservationArr observations;
	float footprintRef{0}; // median native footprint, shown at the zoom when equalized
	// patch settings of the last request
	PatchScale requestedScale{SCALE_EQUALIZED};
	float requestedZoom{0};

	std::shared_ptr<MVS::ImageCache> imageCache; // full resolution color images of the layer
	uint32_t imageCacheLayerID{NO_ID};
	std::shared_ptr<CroppedPatches> croppedPatches{std::make_shared<CroppedPatches>()};
	BS::light_thread_pool threadPool; // last member: destroyed first, waiting for its tasks
};
/*----------------------------------------------------------------*/

} // namespace VIEWER
