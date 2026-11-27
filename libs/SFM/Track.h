/*
 * Track.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 */

#ifndef _SFM_TRACK_H_
#define _SFM_TRACK_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"

#include <functional>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;

/**
 * @brief Observation of a 3D point in an image
 */
struct SFM_API Observation
{
	uint32_t imageID;    // ID of the image seeing this point
	uint32_t featureID;  // ID of the feature in that image

	Observation() : imageID(NO_ID), featureID(NO_ID) {}
	Observation(uint32_t imgID, uint32_t featID) : imageID(imgID), featureID(featID) {}

	bool operator<(const Observation& o) const {
		return imageID < o.imageID || (imageID == o.imageID && featureID < o.featureID);
	}

	bool operator==(const Observation& o) const {
		return imageID == o.imageID && featureID == o.featureID;
	}

	#ifdef _USE_BOOST
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & imageID & featureID;
	}
	#endif
};
typedef SEACAVE::cList<Observation, const Observation&, 0, 4, uint32_t> ObservationArr;
/*----------------------------------------------------------------*/


/**
 * @brief 3D point track with observations from multiple images
 *
 * A track represents a 3D point that has been observed in multiple images.
 * It stores the triangulated 3D position, and all observations
 * (image + feature pairs) that contribute to this point.
 * Inlier observations are stored first in the observations array, with
 * the count stored in numInliers for efficient inlier-only iteration.
 * Optionally, the color is stored in a separate array in the Scene.
 */
struct SFM_API Track
{
	Point3 position;             // 3D position in world coordinates (UNTRIANGULATED_COORD in all three while not triangulated)
	ObservationArr observations; // list of observations in images
	uint8_t numInliers;		     // number of inlier observations (stored first in the array)

	// The fewest views that determine a position: a track needs this many inlier observations to be an
	// inlier, and this many posed ones to be triangulated or judged by the track filter
	inline static constexpr unsigned MIN_VIEWS = 2;

	// Every coordinate of the position of a track not yet triangulated: an improbable point, far beyond any
	// scene (yet finite, so no arithmetic on it turns to INF or NaN, and its square still fits a float), instead
	// of the origin, which a posed camera can sit on (the star puts its reference there): a track judged at
	// this position projects far from its keypoints and its rays meet at no angle, so the filter rejects it
	inline static constexpr REAL UNTRIANGULATED_COORD = 1e18;

	Track() : position(UNTRIANGULATED_COORD), numInliers(0) {}
	Track(const Point3& _pos) : position(_pos), numInliers(0) {}

	// Get number of observations
	inline unsigned GetNumObservations() const { return observations.size(); }
	// Get number of inlier observations
	inline unsigned GetNumInliers() const { return static_cast<unsigned>(numInliers); }

	// Is this track valid (has at least 2 observations)
	inline bool IsValid() const { return observations.size() >= 2; }
	// Is this track triangulated and inlier (has a valid 3D position)
	inline bool IsInlier() const { return numInliers >= MIN_VIEWS; }
	inline bool IsInlier(uint8_t n) const { return numInliers >= n; }

	// Get the number of observations whose image has a pose (a position needs MIN_VIEWS of them)
	unsigned GetNumPosedObservations(const ImageArr&) const;

	// Compute the widest angle between the rays of any two inlier observations (the angle the pair of cameras furthest apart sees the point under)
	float ComputeMaxAngleBetweenRays(const ImageArr&) const;

	// Iterators for inlier observations only
	typedef Observation value_type;
	typedef const value_type* const_iterator;
	typedef value_type* iterator;
	inline const_iterator begin() const { return observations.data(); }
	inline const_iterator end() const { return observations.data() + numInliers; }
	inline iterator begin() { return observations.data(); }
	inline iterator end() { return observations.data() + numInliers; }

	#ifdef _USE_BOOST
	template<class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & position;
		ar & observations;
		ar & numInliers;
	}
	#endif
};
typedef SEACAVE::cList<Track, const Track&, 2, 256, uint32_t> TrackArr;
/*----------------------------------------------------------------*/


/**
 * @brief How BuildTracks settles a component of matched keypoints that holds one image twice
 *
 * The matches of a scene link keypoints into connected components, and a component holding two
 * keypoints of one image is not a track: one of its links is wrong, or the two keypoints are one
 * feature detected twice. See BuildTracks for the rules.
 */
struct SFM_API TrackConflictConfig {
	// cut the least-supported link on the path between the two keypoints, keeping the side the
	// triangles corroborate; false keeps the veto of the union in pair order, where whichever link
	// arrived first stands
	bool cut = true;
	// two keypoints of one image this close (pixels) are one feature: the component stays one
	// track and the keypoint with more links stands for the image (0 disables)
	float mergeDistance = 3.f;
	// a component of more keypoints than this takes the veto whatever `cut` says
	unsigned maxComponentSize = 5000;
};

/**
 * @brief Pre-compute per-image global feature offsets (size images.size() + 1, last element = total features).
 */
SFM_API Unsigned32Arr BuildFeatureOffsets(const ImageArr& images);

/**
 * @brief Build 3D point tracks from 2D feature matches
 *
 * Creates tracks by merging observations connected through pair matches.
 * Uses disjoint-set (union-find) data structure for efficient track building.
 * Stores results in scene.tracks (each Track contains its observations).
 *
 * @param minPairWeight the weight a pair must exceed to be used in creating tracks (0 drops only the pairs without weight, a negative bar drops none)
 * @param conflict how a component holding one image twice is settled (TrackConflictConfig)
 */
SFM_API void BuildTracks(Scene& scene, float minPairWeight = 0, const TrackConflictConfig& conflict = TrackConflictConfig());

/// whether the solve behind a report used the observation of keypoint featureID in image imageID
typedef std::function<bool(uint32_t imageID, uint32_t featureID)> ObservationFilter;

/**
 * @brief Compute mean reprojection error for inlier tracks and log it
 *
 * Without isUsed every observation of the inlier tracks counts. With it (a solve whose per-image budget
 * left observations out) the means, and the views per track of the log line, are taken over the
 * observations isUsed accepts, and the line adds how many it left out.
 * @return mean reprojection error in pixels (first) and degrees (second), over the used observations when isUsed is set
 */
SFM_API std::pair<float, float> ComputeTracksMeanReprojectionError(const Scene& scene, const ObservationFilter& isUsed = nullptr);

/**
 * @brief Pixel reprojection error of a camera-space point against an observed keypoint.
 * @return pixel error and whether the projection is valid (false = behind camera / degenerate)
 */
SFM_API std::pair<float, bool> ComputeReprojectionErrorPixels(const Camera& camera, const Point3& Xcam, const Point2f& kpPt);

/**
 * @brief Pixel reprojection error of a world-space point against an image's observed keypoint.
 */
inline std::pair<float, bool> ComputeReprojectionErrorPixels(const Image& image, const Point3& Xworld, uint32_t featureID)
{
	ASSERT(image.IsValid() && featureID < image.keypoints.size());
	return ComputeReprojectionErrorPixels(*image.pCamera, image.TransformPointW2C(Xworld), image.keypoints[featureID].pt);
}

/**
 * Robust reprojection sigma (median pixel error) over described vs dense observations of inlier tracks.
 */
SFM_API void ComputeObservationSigmas(const Scene& scene,
	double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense);

/**
 * @brief Filter tracks based on angular reprojection error, triangulation angle, and depth bounds.
 *
 * A track seen by fewer than two posed images is left with numInliers = 0.
 * Reprojection error is evaluated in the angular domain via Camera::PixelErrorToAngular, so the
 * check works uniformly for pinhole and spherical cameras.
 *
 * @param scene Scene containing tracks to filter
 * @param maxReprojErrorPixels Maximum allowed reprojection error in pixels (converted to angle per camera)
 * @param minAngleDegrees Minimum required angle between any two observations in degrees
 * @param multDepthNear Multiplier for near depth threshold based on median depth (0 disabled)
 * @param multDepthFar Multiplier for far depth threshold based on median depth (0 disabled)
 * @return mean reprojection error in pixels (first) and degrees (second)
 */
SFM_API std::pair<float, float> FilterTracks(Scene& scene,
	float maxReprojErrorPixels = 3.f,
	float minAngleDegrees = 2.f,
	float multDepthNear = 0.05f,
	float multDepthFar = 20.f);

/**
 * @brief Filter weakly connected images using spatial distribution, triangulation angle, and covisibility k-core.
 *
 * Among images removed by the largest-component passes, separates those whose verified pairs to the kept
 * core contradict the reconstructed pose from those merely lacking evidence.
 *
 * @param scene Scene containing images and tracks (tracks must be pre-filtered by FilterTracks)
 * @param minCovisibilityCount Minimum number of shared tracks to form a covisibility edge (default: 5)
 * @param minObservationArea Fraction of 10x10 image grid cells [0-1] that must contain tracks (default: 0.15)
 * @param minTriangulationAngle Minimum median triangulation angle in degrees (default: 1.5)
 * @param minCovisDegree Minimum independent covisibility neighbors to survive the k-core peel (default: 2)
 * @return the invalidated image IDs, and among them the ones cut for contradicting the model
 */
struct RemovedImages {
	IIndexArr all;           // every image the filter invalidated
	IIndexArr contradicting; // the ones among them whose verified pairs put them elsewhere than the model did
};
SFM_API RemovedImages FilterWeaklyConnectedImages(Scene& scene,
	unsigned minCovisibilityCount = 5,
	float minObservationArea = 0.15f,
	float minTriangulationAngle = 1.5f,
	unsigned minCovisDegree = 2);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_TRACK_H_
