/*
 * Triangulation.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_TRIANGULATION_H_
#define _SFM_TRIANGULATION_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Track.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;

/**
	* @brief DLT-based triangulation for a single track.
	* It assumes all track observations are valid (the corresponding view has pose).
	*
	* NOTE: This function is PINHOLE-ONLY. It uses the 2D Camera::Unproject() output
	* as a point on the z=1 normalized plane and builds the standard DLT linear system
	* A*X=0 from the full 3x4 projection matrix P = K*[R|t]. For spherical (equirectangular)
	* cameras the 2D unproject is front-hemisphere biased (it aliases back-hemisphere
	* features onto the front), so this formulation cannot represent observations with
	* |longitude| > pi/2. Use TriangulateSkewLLS() instead — it operates on 3D unit
	* bearing vectors from Camera::UnprojectNormalized() and is singularity-free for
	* all camera models.
	*
	* @param track The track to triangulate.
	* @param images Scene images with pinhole cameras and poses
	* @param reprojThreshold Reprojection error threshold (pixels)
	* @param minAngleThreshold Minimum angle between rays (degrees)
	* @param minInliers Minimum number of inlier views
	* @return number of inliers if triangulation successful
	*/
unsigned TriangulateDLT(
	Track& track,
	const ImageArr& images,
	float reprojThreshold = 4.f,
	float minAngleThreshold = 2.f,
	unsigned minInliers = 2);

/**
	* @brief Robust triangulation using the skew-symmetric formulation ([d]_x * (R * Pw + t) = 0).
	* It builds an overdetermined system A * Pw = b with 2 independent rows per observation.
	* It ignores invalid observations (views without pose).
	* @param track The track to triangulate.
	* @param images Scene images with cameras and poses.
	* @param reprojThreshold Reprojection error threshold (pixels).
	* @param minAngleThreshold Minimum triangulation angle threshold (degrees).
	* @param minInliers Minimum number of inlier observations.
	* @return The number of inliers or 0 on failure.
	*/
unsigned TriangulateSkewLLS(
	Track& track,
	const ImageArr& images,
	float reprojThreshold = 4.f,
	float minAngleThreshold = 2.f,
	unsigned minInliers = 2);

/**
	* @brief Triangulate all tracks in scene.
	* @param scene Scene with cameras, poses, and tracks
	* @param outliersOnly If true, triangulate only tracks with outlier observations
	* @param reprojThreshold Reprojection error threshold
	* @param minAngleThreshold Minimum angle between rays (degrees)
	* @return number of inlier tracks
	*/
unsigned TriangulateTracks(
	Scene& scene,
	bool outliersOnly = false,
	float reprojThreshold = 4.f,
	float minAngleThreshold = 2.f);
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_TRIANGULATION_H_
