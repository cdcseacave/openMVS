/*
 * Link2D3D.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "Link2D3D.h"
#include "Scene.h"

using namespace SFM;


// S T R U C T S ///////////////////////////////////////////////////

void CameraCorrespondences::Add(const Image& image, uint32_t featureID, const Point3& X)
{
	ASSERT(featureID < image.keypoints.size());
	const Point2 kp = image.keypoints[featureID].pt;
	bearings.emplace_back(image.pCamera->UnprojectNormalized(kp));
	points.push_back(X);
	dense.push_back(image.IsDenseKeypoint(featureID));
	if (!dense.back())
		++numDescribed;
}

void SFM::CollectTrackCorrespondences(const Scene& scene, IIndex imageID, CameraCorrespondences& correspondences)
{
	correspondences = CameraCorrespondences();
	const Image& image = scene.images[imageID];
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track.observations) {
			if (obs.imageID == imageID) {
				correspondences.Add(image, obs.featureID, track.position);
				break;
			}
		}
	}
}

std::vector<TrackObservationCount> SFM::CountTrackObservations(const Scene& scene)
{
	std::vector<TrackObservationCount> counts(scene.images.size());
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		FOREACH(k, track.observations) {
			TrackObservationCount& count = counts[track.observations[k].imageID];
			++count.observations;
			if (k < track.numInliers)
				++count.inliers;
		}
	}
	return counts;
}
/*----------------------------------------------------------------*/
