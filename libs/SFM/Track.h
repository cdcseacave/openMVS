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

	// Compute the minimum angle between any two inlier observations
	float ComputeMinAngleBetweenRays(const ImageArr&) const;

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
 * @brief Build 3D point tracks from 2D feature matches
 *
 * Creates tracks by merging observations connected through pair matches.
 * Uses disjoint-set (union-find) data structure for efficient track building.
 * Stores results in scene.tracks (each Track contains its observations).
 *
 * @param minPairWeight minimum weight for a pair to be used in creating tracks (-1 = disabled)
 * @param conflict how a component holding one image twice is settled (TrackConflictConfig)
 */
SFM_API void BuildTracks(Scene& scene, float minPairWeight = 0, const TrackConflictConfig& conflict = TrackConflictConfig());

/**
 * @brief Compute mean reprojection error for inlier tracks
 * @return mean reprojection error in pixels (first) and degrees (second)
 */
SFM_API std::pair<float, float> ComputeTracksMeanReprojectionError(Scene& scene);

/**
 * @brief Pixel reprojection error of an already camera-space point against an observed keypoint,
 * using the observing camera's projection model. This is the exact formula FilterTracks uses when
 * it accumulates its pixel-error statistics for a kept observation; exposed as its own function so
 * any other reader of a track (e.g. an offline per-observation dump) computes the identical number
 * instead of re-deriving it.
 * @param camera the observing camera (current intrinsics)
 * @param Xcam the track position already transformed into this camera's frame (Image::TransformPointW2C)
 * @param kpPt the observed keypoint's pixel position
 * @return pixel error and whether the projection is valid (false = behind the camera / degenerate;
 *         the error is then undefined and must not be used)
 */
SFM_API std::pair<float, bool> ComputeReprojectionErrorPixels(const Camera& camera, const Point3& Xcam, const Point2f& kpPt);

/**
 * Robust reprojection sigma of the two observation populations of the current solution, in pixels:
 * the median reprojection error over the observations on described keypoints, and over those on
 * dense (warp-sampled) ones, each computed with ComputeReprojectionErrorPixels -- the same formula
 * FilterTracks uses, so the two numbers describe the residuals bundle adjustment actually sees.
 * The median of an already non-negative error is taken as the scale directly, with no 1.4826
 * consistency factor: the only consumer takes the RATIO of the two, in which any common factor
 * cancels. Only inlier tracks and valid images take part -- an image without a pose is skipped
 * rather than asserted, unlike ComputeTracksMeanReprojectionError, because this runs at the head
 * of every solve, where a scene still holding images the incremental reconstruction has not
 * registered yet is the normal state and not a bug. An observation whose projection is invalid
 * is counted by neither population.
 * The counts come back so the caller can refuse a sample too small to be a sigma.
 */
SFM_API void ComputeObservationSigmas(const Scene& scene,
	double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense);

/**
 * @brief Filter tracks based on various criteria
 *
 * A track seen by fewer than two posed images has no position that two posed views determine: it is
 * not judged and ends with numInliers 0.
 *
 * Reprojection error is always evaluated in the angular domain — the pixel threshold is
 * converted per-camera via Camera::PixelErrorToAngular, so the same check works uniformly
 * for pinhole and spherical (equirectangular pixel distance has no linear angular meaning).
 *
 * @param scene Scene containing tracks to filter
 * @param maxReprojErrorPixels Maximum allowed reprojection error in pixels (converted to angle per camera)
 * @param minAngleDegrees Minimum required angle between any two observations in degrees
 * @param multDepthNear Multiplier for near depth threshold based on median depth (0 disabled)
 * @param multDepthFar Multiplier for far depth threshold based on median depth (0 disabled)
 * @param denseReprojErrorFactor Multiple of maxReprojErrorPixels a dense (warp-sampled) observation
 *                               is held to: the less precise measurement answers to a looser bar
 *                               (1 = the same bar as a described observation)
 * @return mean reprojection error in pixels (first) and degrees (second)
 */
SFM_API std::pair<float, float> FilterTracks(Scene& scene,
	float maxReprojErrorPixels = 3.f,
	float minAngleDegrees = 2.f,
	float multDepthNear = 0.05f,
	float multDepthFar = 20.f,
	float denseReprojErrorFactor = 1.f);

/**
 * @brief Filter weakly connected images and cluster the remainder based on covisibility
 *
 * Applies two pre-filters to identify and remove weakly connected images:
 * - Tier 1: Spatial Distribution Filter (Effective Inlier Count) - removes images where
 *   tracks are clustered in a small region
 * - Tier 2: Geometric Degeneracy Filter - removes images with small triangulation angles
 *
 * Then builds a covisibility graph based on shared inlier tracks and:
 * - Keeps only the largest connected component
 * - Peels images with too few independent covisibility neighbors (absolute k-core) and
 *   keeps the largest remaining connected component
 *
 * Among the images the largest-component passes cut, the ones the model contradicts are told from
 * the ones it merely lacks evidence for: a cut component whose verified pairs (of at least 15
 * weighted inliers) to the kept images disagree with the pose the model gives it, in rotation or
 * baseline direction beyond 5 degrees, by more inlier weight than agrees is contradicted; one with
 * no such pair, or agreeing ones, is not.
 *
 * @param scene Scene containing images and tracks (tracks must be pre-filtered by FilterTracks)
 * @param minCovisibilityCount Minimum number of shared tracks to form a covisibility edge
 *                             Default: 5 (typically 5-10 for standard images)
 * @param minObservationArea Fraction of image grid cells [0-1] that must contain tracks
 *                           Default: 0.15 (15% occupancy, detects clustering)
 * @param minTriangulationAngle Minimum median triangulation angle in degrees
 *                              Default: 1.5° (conservative)
 * @param minCovisDegree Minimum independent covisibility neighbors to survive the k-core peel
 *                       Default: 2 (sibling absolute threshold to minCovisibilityCount)
 * @param maxPoseInconsistencyAngle Cut covisibility edges whose global relative rotation disagrees
 *                       with the stored two-view relativePose by more than this angle, in degrees
 *                       (detects wrongly positioned views). Default: 0 (disabled)
 * @param maxReprojErrorPixels Enable the agreement-gated per-image backstops (match-survival +
 *                       robust reprojection); pass config.maxFineReprojError. Default: 0 (disabled)
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
	unsigned minCovisDegree = 2,
	float maxPoseInconsistencyAngle = 0.f,
	float maxReprojErrorPixels = 0.f);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_TRACK_H_
