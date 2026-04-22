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
	Point3 position;             // 3D position in world coordinates
	ObservationArr observations; // list of observations in images
	uint8_t numInliers;		     // number of inlier observations (stored first in the array)

	Track() : numInliers(0) {}
	Track(const Point3& _pos) : position(_pos), numInliers(0) {}

	// Get number of observations
	inline unsigned GetNumObservations() const { return observations.size(); }
	// Get number of inlier observations
	inline unsigned GetNumInliers() const { return static_cast<unsigned>(numInliers); }

	// Is this track valid (has at least 2 observations)
	inline bool IsValid() const { return observations.size() >= 2; }
	// Is this track triangulated and inlier (has a valid 3D position)
	inline bool IsInlier() const { return numInliers >= 2; }
	inline bool IsInlier(uint8_t n) const { return numInliers >= n; }

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
 * @brief Build 3D point tracks from 2D feature matches
 *
 * Creates tracks by merging observations connected through pair matches.
 * Uses disjoint-set (union-find) data structure for efficient track building.
 * Stores results in scene.tracks (each Track contains its observations).
 *
 * @param minPairWeight minimum weight for a pair to be used in creating tracks (-1 = disabled)
 */
SFM_API void BuildTracks(Scene& scene, float minPairWeight = 0);

/**
 * @brief Compute mean reprojection error for inlier tracks
 * @return mean reprojection error in pixels (first) and degrees (second)
 */
SFM_API std::pair<float, float> ComputeTracksMeanReprojectionError(Scene& scene);

/**
 * @brief Filter tracks based on various criteria
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
 * @return mean reprojection error in pixels (first) and degrees (second)
 */
SFM_API std::pair<float, float> FilterTracks(Scene& scene,
	float maxReprojErrorPixels = 3.f,
	float minAngleDegrees = 2.f,
	float multDepthNear = 0.05f,
	float multDepthFar = 20.f);

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
 * - Computes an adaptive edge weight threshold using median-MAD
 * - Clusters remaining images using union-find on strong/weak connections
 *
 * @param scene Scene containing images and tracks (tracks must be pre-filtered by FilterTracks)
 * @param minCovisibilityCount Minimum number of shared tracks to form a covisibility edge
 *                             Default: 5 (typically 5-10 for standard images)
 * @param minObservationArea Fraction of image grid cells [0-1] that must contain tracks
 *                           Default: 0.15 (15% occupancy, detects clustering)
 * @param minTriangulationAngle Minimum median triangulation angle in degrees
 *                              Default: 1.5° (conservative)
 * @return Array of invalidated image IDs
 */
SFM_API IIndexArr FilterWeaklyConnectedImages(Scene& scene,
	unsigned minCovisibilityCount = 5,
	float minObservationArea = 0.15f,
	float minTriangulationAngle = 1.5f);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_TRACK_H_
