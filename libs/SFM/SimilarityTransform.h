/*
 * SimilarityTransform.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_SIMILARITYTRANSFORM_H_
#define _SFM_SIMILARITYTRANSFORM_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

/**
 * @brief Estimate similarity transform from 3D point correspondences
 * @param srcPoints Source 3D points
 * @param dstPoints Destination 3D points
 * @param transform Output similarity transform (rotation, scale, translation)
 * @param threshold RANSAC inlier threshold as a (linear) Euclidean distance in destination units
 *                  (the residual is measured as transform*src - dst); if > 0, uses RANSAC to filter outliers
 * @param refine If true, refines the transform after initial estimation
 * @param maxIters RANSAC iteration budget (0 = auto-cap); raise for low inlier ratios
 * @param confidence RANSAC confidence used for the adaptive iteration update
 * @return number of inliers used for the final estimate (RANSAC inlier count when threshold > 0,
 *         otherwise the total correspondence count) or 0 on failure
 */
SFM_API unsigned EstimateSimilarityTransform(
	const Point3Arr& srcPoints,
	const Point3Arr& dstPoints,
	Transform& transform,
	double threshold = 0.0,
	bool refine = true,
	size_t maxIters = 0,
	double confidence = 0.9999);

/**
 * @brief Estimate a similarity transform from paired centers, using the paired rotations
 *        to disambiguate a (nearly) collinear center set
 *
 * Well-spread centers are handled by EstimateSimilarityTransform. When the second principal
 * spread of the destination centers falls below the threshold (a straight-line capture such
 * as a corridor walk), the center-only fit leaves the roll about the common axis
 * unconstrained; the rotation is then estimated by robust rotation averaging over the
 * rotation pairs (dstR ~= srcR * alignR, and poses transform as R_new = R * T.R^t, so
 * T.R = alignR^t), scale and translation follow by least squares with the rotation fixed,
 * and the result must place the median transformed center within the threshold.
 * @param srcCenters,dstCenters corresponding positions (e.g. camera centers) in each frame
 * @param srcRots,dstRots corresponding world-to-camera rotations, parallel to the centers
 * @param transform Output similarity transform (dst = transform * src)
 * @param threshold RANSAC inlier threshold in destination units; also gates the collinearity
 *        test (0 disables both, reducing to the plain center-only estimation)
 * @return number of inliers used for the final estimate, or 0 on failure
 */
SFM_API unsigned EstimateSimilarityTransformWithRotations(
	const Point3Arr& srcCenters,
	const Point3Arr& dstCenters,
	const Matrix3x3Arr& srcRots,
	const Matrix3x3Arr& dstRots,
	Transform& transform,
	double threshold = 0.0);
/*----------------------------------------------------------------*/


// Similarity transform refinement unit test
SFM_API bool TestSimilarityTransform();
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_SIMILARITYTRANSFORM_H_
