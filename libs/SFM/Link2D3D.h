/*
 * Link2D3D.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_LINK2D3D_H_
#define _SFM_LINK2D3D_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;

// How an image's 2D features reach a model's 3D points, for every absolute pose estimated from them:
// the resection's PnP of one image against its model's inlier tracks, and the merge's generalized PnP
// of one block's cameras against another block's points (and its one-camera door). Which pairs join
// an image to a model's registered images (its links) is read in PoseLink.h.

// The angle a pixel error spans in the image's camera, at the camera's feature noise scale: the bar
// every 2D-3D fit of the camera is judged at
inline REAL AngularError(const Image& image, REAL pixels)
{
	return image.pCamera->PixelErrorToAngular(pixels * image.pCamera->GetFeatureNoiseScale());
}

// One camera's 2D-3D correspondences, the input of its absolute pose: per correspondence the unit
// bearing of the keypoint in the camera's frame, the 3D point it observes, and whether the keypoint
// is dense (warp-sampled) rather than described
struct CameraCorrespondences
{
	std::vector<Eigen::Vector3d> bearings; // the absolute pose estimators' poselib::Point3D
	std::vector<Eigen::Vector3d> points;
	std::vector<char> dense;
	unsigned numDescribed{0};

	unsigned size() const { return (unsigned)bearings.size(); }
	// the given keypoint of the image observing the given point
	void Add(const Image& image, uint32_t featureID, const Point3& X);
};

// The image's correspondences to the model's inlier tracks, in track order: its first observation of
// each inlier track it observes (a track can hold two keypoints of one image that BuildTracks merged)
SFM_API void CollectTrackCorrespondences(const Scene& scene, IIndex imageID, CameraCorrespondences& correspondences);

// Per image, its observations of the model's inlier tracks, and of those the ones in the tracks'
// inlier prefix: an image the model has not posed is outside every prefix, so its count is the
// number of 2D-3D correspondences it holds against the model
struct TrackObservationCount
{
	unsigned observations{0};
	unsigned inliers{0};
};
SFM_API std::vector<TrackObservationCount> CountTrackObservations(const Scene& scene);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_LINK2D3D_H_
