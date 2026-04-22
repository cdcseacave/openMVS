////////////////////////////////////////////////////////////////////
// ImagePair.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ImagePair.h"
#include "Image.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

Matrix3x3 ImagePair::ComposeEssentialMatrix(const Pose3D& pose)
{
	// Compose essential matrix from relative pose
	// E = [t]_x * R
	const Point3 t = pose.GetT();
	const Matrix3x3 tx(
		0, -t[2], t[1],
		t[2], 0, -t[0],
		-t[1], t[0], 0
	);
	return tx * pose.R;
}

Pose3D ImagePair::DecomposeEssentialMatrix(const Matrix3x3& E)
{
	// Decompose essential matrix into relative pose using SVD decomposition
	// note: This function does not resolve the 4-fold ambiguity; use RecoverPose() instead
	Matrix3x3 copyE(E);
	cv::SVD svd(copyE, cv::SVD::MODIFY_A);
	// Ensure U and Vt are proper rotations
	cv::Mat U = svd.u;
	cv::Mat Vt = svd.vt;
	if (cv::determinant(U) < 0) U *= -1.0;
	if (cv::determinant(Vt) < 0) Vt *= -1.0;
	Matrix3x3 W(0, -1, 0,
				  1,  0, 0,
				  0,  0, 1);
	// There are four possible solutions for (R, t):
	// R = U * W * V^T or U * W^T * V^T
	// t = +u_3 or -u_3 (last column of U)
	cv::Mat R = U * W * Vt;
	if (cv::determinant(R) < 0)
		R = U * W.t() * Vt;
	Pose3D pose;
	pose.R = R;
	cv::Vec3d t = U.col(2);
	pose.SetT(Point3(t[0], t[1], t[2]));
	return pose;
}

unsigned ImagePair::RecoverPose(
	const Matrix3x3& E,
	const std::vector<Point2f>& points1,
	const std::vector<Point2f>& points2,
	const Matrix3x3& K,
	Pose3D& pose,
	cv::InputOutputArray inliers)
{
	// Use OpenCV's recoverPose which handles the 4-fold ambiguity and cheirality check.
	// TODO: implement our own version to support different K for both images
	cv::Mat R_cv, t_cv;
	int numInliers = cv::recoverPose(E, points1, points2, K, R_cv, t_cv, inliers);
	if (numInliers > 0) {
		pose.R = R_cv;
		pose.SetT(Point3(t_cv.at<double>(0), t_cv.at<double>(1), t_cv.at<double>(2)));
	}
	return static_cast<unsigned>(numInliers);
}

Point3 ImagePair::EpipoleFromEssentialMatrix(const Matrix3x3& E, bool leftImage) {
	Eigen::Matrix3d eE(leftImage ? E : Matrix3x3(E.t()));
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(eE, Eigen::ComputeFullU | Eigen::ComputeFullV);
	return svd.matrixV().block<3, 1>(0, 2).eval();
}

Matrix3x3 ImagePair::ComposeFundamentalMatrix(const Matrix3x3& E, const Matrix3x3& K1, const Matrix3x3& K2, bool normalize)
{
	// Compose fundamental matrix from essential matrix and camera intrinsics
	Matrix3x3 F = K2.t().inv() * E * K1.inv();
	// Normalize F such that determinant is 1
	if (normalize)
		F /= cv::norm(F);
	return F;
}
Matrix3x3 ImagePair::DecomposeFundamentalMatrix(const Matrix3x3& F, const Matrix3x3& K1, const Matrix3x3& K2, bool normalize)
{
	// Decompose fundamental matrix into essential matrix
	Matrix3x3 E = K2.t() * F * K1;
	if (normalize)
		E /= cv::norm(E);
	return E;
}

float ImagePair::ComputeAngleBaselineWeight(float meanAngleDegrees)
{
	if (meanAngleDegrees <= 0.f)
		return 1.f; // no angle info, return neutral weight

	#if 0
	// Multiplicative Factor:
	//   - Small θ_avg (< 2°): Poor baseline - noisy depth estimation
	//   - Medium θ_avg (5-30°): Good baseline - reliable triangulation
	//   - Large θ_avg (> 60°): Wide baseline - harder matching but potentially better if matches exist
	// use a Gaussian-like weighting centered at an optimal angle (e.g., 15°) to favor medium baselines
	constexpr float OPTIMAL_ANGLE = 15.f;
	constexpr float ANGLE_WIDTH = 25.f;
	constexpr float SMALL_ANGLE = 4.f;
    constexpr float STEEPNESS = 0.8f;
	constexpr float COMMON = 6.68099f;

	// Avoid too small angles
	if (meanAngleDegrees < COMMON) {
		// Sigmoid that transition fast to 0 below SMALL_ANGLE
		return 1.f / (1.f + EXP(-STEEPNESS * (meanAngleDegrees - SMALL_ANGLE)));
	}

	// Gaussian-like weighting centered at OPTIMAL_ANGLE
	return EXP(-SQUARE((meanAngleDegrees - OPTIMAL_ANGLE) / ANGLE_WIDTH));

	#else

	// Angle Baseline Weighting:
	// Penalize very small angles (weak baseline) and very large angles (harder matching)
	constexpr float optimalAngle = 15.f; // optimal viewing angle between image pairs (degrees)
	constexpr float slowThreshold = 9.f; // gentle degradation range (degrees)
	constexpr float maxSlowPenalty = 0.3f; // max penalty within slow threshold
	constexpr float minAngle = 1.5f; // below this, rapid penalty for degenerate baseline (degrees)

	const auto QuadraticPenalty = [&](float deviation) {
		const float t = deviation / slowThreshold;
		return 1.f - maxSlowPenalty * t * t; // up to maxSlowPenalty at threshold
	};

	if (meanAngleDegrees < minAngle) {
		// Very small angle: rapid quadratic penalty for weak baseline
		const float maxWeight = QuadraticPenalty(optimalAngle - minAngle); // weight at the edge of slow threshold (C1-continuous with the quadratic penalty)
		return maxWeight * SQUARE(meanAngleDegrees / minAngle);
	}

	const float deviation = meanAngleDegrees - optimalAngle;
	if (deviation <= slowThreshold) {
		// Gentle quadratic penalty near optimal angle
		return QuadraticPenalty(deviation);
	}

	// Rapid exponential decay beyond threshold (C1-continuous with the quadratic penalty)
	const float excessDeviation = deviation - slowThreshold;
	return (1.f - maxSlowPenalty) * EXP(-excessDeviation / 10.5f);
	#endif
}


unsigned ImagePair::PartitionMatchesByMask(const std::vector<char>& mask, int numInliers, bool reorderOnly)
{
	ASSERT(mask.empty() || mask.size() == matches.size());
	ASSERT(numInliers < 0 || (unsigned)numInliers <= matches.size());
	if (mask.empty() || (unsigned)numInliers == matches.size())
		return static_cast<unsigned>(matches.size()); // Nothing to do
	std::vector<DMatch> inliersMatches;
	inliersMatches.reserve(numInliers >= 0 ? (unsigned)numInliers : matches.size() / 2);
	if (!reorderOnly) {
		// Split into inliers and outliers
		outlierMatches.reserve(outlierMatches.size() + (numInliers >= 0 ? matches.size() - (unsigned)numInliers : matches.size() / 2));
		FOREACH(i, matches)
			(mask[i] ? inliersMatches : outlierMatches).push_back(matches[i]);
		matches.swap(inliersMatches);
		return static_cast<unsigned>(matches.size());
	}
	// Reorder only, keep all matches but place inliers first
	std::vector<DMatch> newOutlierMatches;
	newOutlierMatches.reserve(numInliers >= 0 ? matches.size() - (unsigned)numInliers : matches.size() / 2);
	FOREACH(i, matches)
		(mask[i] ? inliersMatches : newOutlierMatches).push_back(matches[i]);
	numInliers = static_cast<int>(inliersMatches.size());
	matches.swap(inliersMatches);
	matches.insert(matches.end(), newOutlierMatches.begin(), newOutlierMatches.end());
	return static_cast<unsigned>(numInliers);
}

std::pair<std::vector<Point2f>, std::vector<Point2f>> ImagePair::GetMatchedPoints(const Image& img1, const Image& img2, bool allInliers, bool allMatches) const
{
	// Extract matched points from keypoints
	std::vector<Point2f> pts1, pts2;
	const unsigned totalMatches = allMatches ? GetNumMatches() : (allInliers ? GetNumInliers() : GetNumFilteredInliers());
	const unsigned inlierMatches = allMatches || allInliers ? GetNumInliers() : GetNumFilteredInliers();
	pts1.reserve(totalMatches);
	pts2.reserve(totalMatches);
	for (unsigned i = 0; i < inlierMatches; ++i) {
		const DMatch& m = matches[i];
		pts1.push_back(img1.keypoints[m.queryIdx].pt);
		pts2.push_back(img2.keypoints[m.trainIdx].pt);
	}
	if (allMatches) {
		for (const DMatch& m : outlierMatches) {
			pts1.push_back(img1.keypoints[m.queryIdx].pt);
			pts2.push_back(img2.keypoints[m.trainIdx].pt);
		}
	}
	return std::make_pair(pts1, pts2);
}

unsigned ImagePair::FilterMatches(const Image& img1, const Image& img2, float minAngle, float reprojThreshold, float epipoleThresh)
{
	if (!relativePose.has_value() || matches.empty())
		return matches.size(); // nothing to do

	// Prepare relative pose data
	const Pose3D& relPose = relativePose.value();
	const Camera& cam1 = *img1.pCamera;
	const Camera& cam2 = *img2.pCamera;

	// Convert pixel-based thresholds to angular (radians) so equirectangular images are handled correctly:
	// a pixel metric near the poles/±π seam doesn't correspond linearly to angular separation.
	// For pinhole this is equivalent to the pixel distance check within atan(δ/f) ≈ δ/f.
	const REAL cosReprojAngle = reprojThreshold > 0 ?
		COS(REAL(0.5) * (cam1.PixelErrorToAngular(reprojThreshold) + cam2.PixelErrorToAngular(reprojThreshold))) :
		REAL(-1);

	// Precompute epipole directions (unit bearings) in each camera frame
	Point3 epipoleDir1(Point3::ZERO), epipoleDir2(Point3::ZERO);
	REAL cosEpipoleAngle = REAL(2); // > 1 means epipole filter never triggers
	if (epipoleThresh > 0) {
		cosEpipoleAngle = COS(REAL(0.5) * (cam1.PixelErrorToAngular(epipoleThresh) + cam2.PixelErrorToAngular(epipoleThresh)));
		// Epipole 1 direction: C2 position in Cam1 frame = relPose.C
		const REAL nC = norm(relPose.C);
		if (nC > ZEROTOLERANCE<REAL>())
			epipoleDir1 = relPose.C / nC;
		// Epipole 2 direction: C1 (origin) in Cam2 frame = relPose.GetT()
		const Point3 t = relPose.GetT();
		const REAL nT = norm(t);
		if (nT > ZEROTOLERANCE<REAL>())
			epipoleDir2 = t / nT;
	}

	// Filter matches
	const REAL maxCosAngle = COS(D2R(minAngle));
	const auto [pts1, pts2] = GetMatchedPoints(img1, img2);
	std::vector<char> mask(matches.size(), 0);
	TAccumulator<float> angleAccumulator; // cos-angle weighted average
	unsigned numInliers = 0;
	FOREACH(i, matches) {
		// Observed unit bearings (works for both pinhole and spherical)
		const Point3 b1 = cam1.UnprojectNormalized(Cast<REAL>(pts1[i]));
		const Point3 b2 = cam2.UnprojectNormalized(Cast<REAL>(pts2[i]));
		// 1. Epipole filtering (angular distance to epipole direction)
		if (epipoleThresh > 0 &&
			(b1.dot(epipoleDir1) > cosEpipoleAngle || b2.dot(epipoleDir2) > cosEpipoleAngle))
			continue;
		// 2. Triangulate (midpoint is scale-invariant on unit bearings)
		Point3 X;
		if (!TriangulatePoint3D(relPose.R, relPose.C, b1, b2, X))
			continue;
		// 3. Cheirality + angular reprojection in Cam1
		const REAL nX = norm(X);
		if (nX < ZEROTOLERANCE<REAL>())
			continue;
		const Point3 b1Proj = X / nX;
		const REAL cosErr1 = b1.dot(b1Proj);
		if (cosErr1 <= 0) // >= 90° from observation (behind camera for pinhole / antipodal for spherical)
			continue;
		// 4. Cheirality + angular reprojection in Cam2
		const Point3 Xcam2 = relPose.TransformPointW2C(X);
		const REAL nXc2 = norm(Xcam2);
		if (nXc2 < ZEROTOLERANCE<REAL>())
			continue;
		const Point3 b2Proj = Xcam2 / nXc2;
		const REAL cosErr2 = b2.dot(b2Proj);
		if (cosErr2 <= 0)
			continue;
		// 5. Reprojection error threshold (angular)
		if (reprojThreshold > 0 && (cosErr1 < cosReprojAngle || cosErr2 < cosReprojAngle))
			continue;
		// 6. Triangulation angle check
		const Point3 V1 = X; // ray from C1 to X (in Cam1 frame)
		const Point3 V2 = X - relPose.C; // ray from C2 to X (in Cam1 frame)
		const REAL cosAngle = ComputeAngle(V1.ptr(), V2.ptr());
		if (minAngle > 0 && cosAngle > maxCosAngle)
			continue;
		// Accepted as inlier; weight mean by inverse angular residual (mirrors 1/pixelErr² semantics)
		const float oneMinusCos = (float)(REAL(1) - MINF(cosErr1, cosErr2));
		angleAccumulator.Add((float)cosAngle, 1.f / MAXF(oneMinusCos, 1e-10f));
		mask[i] = 1;
		++numInliers;
	}

	// Update mean ray angle
	meanRayAngle = numInliers > 0 ? ACOS(angleAccumulator.Normalized()) : 0.f;
	// Partition matches by inlier mask
	return numFilteredInliers = (int)PartitionMatchesByMask(mask, (int)numInliers, true);
}


namespace {
// Helper function to check homography constraint
unsigned CheckEpipolarInliersHomography(
	const Matrix3x3& homography,
	const std::vector<Point2f>& pts1,
	const std::vector<Point2f>& pts2,
	float thresholdSq,
	std::vector<uint8_t>& mask)
{
	const Matrix3x3 homography_inv = homography.inv();
	unsigned numInliers = 0;
	FOREACH(i, pts1) {
		// Forward transfer
		Point2f p2_pred;
		ProjectVertex_3x3_2_2(homography.val, pts1[i].ptr(), p2_pred.ptr());
		const REAL distFwdSq = normSq(p2_pred - pts2[i]);
		// Backward transfer
		Point2f p1_pred;
		ProjectVertex_3x3_2_2(homography_inv.val, pts2[i].ptr(), p1_pred.ptr());
		const REAL distBwdSq = normSq(p1_pred - pts1[i]);
		// Symmetric transfer error
		if (distFwdSq <= thresholdSq && distBwdSq <= thresholdSq) {
			mask[i] = 1;
			++numInliers;
		}
	}
	return numInliers;
}

// Helper function to check epipolar constraint using essential matrix
unsigned CheckEpipolarInliersEssential(
	const Matrix3x3& essential,
	const std::vector<Point2f>& pts1,
	const std::vector<Point2f>& pts2,
	const Matrix3x3& K1,
	const Matrix3x3& K2,
	float thresholdSq,
	std::vector<uint8_t>& mask)
{
	const Matrix3x3 K1_inv = K1.inv();
	const Matrix3x3 K2_inv = K2.inv();
	unsigned numInliers = 0;
	FOREACH(i, pts1) {
		// Normalize points
		Point3 p1Norm = K1_inv * pts1[i];
		Point3 p2Norm = K2_inv * pts2[i];
		// Sampson error for essential matrix (normalized coordinates)
		// num = x2^T E x1; denomSq = (Ex1)_x^2 + (Ex1)_y^2 + (E^T x2)_x^2 + (E^T x2)_y^2
		const Point3 epipolarLine1 = essential * p1Norm; // epipolar line in image 2
		const double num = p2Norm.dot(epipolarLine1);
		const double denomSq = SQUARE(epipolarLine1.x) + SQUARE(epipolarLine1.y) +
			SQUARE(p2Norm.dot(essential.col(0))) + SQUARE(p2Norm.dot(essential.col(1)));
		const double err = (denomSq > 1e-12) ? (SQUARE(num) / denomSq) : 0.0;
		if (err <= thresholdSq) {
			mask[i] = 1;
			++numInliers;
		}
	}
	return numInliers;
}

// Helper function to check fundamental matrix constraint
unsigned CheckEpipolarInliersFundamental(
	const Matrix3x3& fundamental,
	const std::vector<Point2f>& pts1,
	const std::vector<Point2f>& pts2,
	float thresholdSq,
	std::vector<uint8_t>& mask)
{
	unsigned numInliers = 0;
	FOREACH(i, pts1) {
		// Sampson error for fundamental matrix (pixel coordinates)
		const Point3 epipolarLine1 = fundamental * pts1[i]; // epipolar line in image 2
		const Point3 x2h = pts2[i].homogeneous();
		const REAL num = x2h.dot(epipolarLine1); // x2^T F x1
		const REAL denomSq = SQUARE(epipolarLine1.x) + SQUARE(epipolarLine1.y) +
			SQUARE(x2h.dot(fundamental.col(0))) + SQUARE(x2h.dot(fundamental.col(1)));
		const REAL err(denomSq > 1e-12 ? (SQUARE(num) / denomSq) : 0.0);
		if (err <= thresholdSq) {
			mask[i] = 1;
			++numInliers;
		}
	}
	return numInliers;
}
} // anonymous namespace

unsigned ImagePair::CheckEpipolarInliers(const Image& img1, const Image& img2, float threshold, int forceEpipolarType, cv::InputOutputArray inlierMask) const
{
	// Count matches that satisfy the epipolar constraint
	// Priority: relativePose -> H -> E -> F
	if (matches.empty())
		return 0;

	// Extract matched points from keypoints
	auto [pts1, pts2] = GetMatchedPoints(img1, img2);

	// Prepare output mask
	const float thresholdSq = SQUARE(threshold);
	std::vector<uint8_t> mask(matches.size(), 0);
	unsigned numInliers = 0;
	if (relativePose.has_value() && (forceEpipolarType == -1 || forceEpipolarType == 0)) {
		// Use relative pose - convert to essential matrix and check epipolar distance
		const Matrix3x3 essential = ComposeEssentialMatrix(relativePose.value());
		ASSERT(img1.HasCamera() && img2.HasCamera());
		numInliers = CheckEpipolarInliersEssential(essential, pts1, pts2, img1.GetK(), img2.GetK(), thresholdSq, mask);
	}
	else if (E.has_value() && (forceEpipolarType == -1 || forceEpipolarType == 1)) {
		// Use essential matrix - check epipolar distance
		ASSERT(img1.HasCamera() && img2.HasCamera());
		numInliers = CheckEpipolarInliersEssential(E.value(), pts1, pts2, img1.GetK(), img2.GetK(), thresholdSq, mask);
	}
	else if (F.has_value() && (forceEpipolarType == -1 || forceEpipolarType == 2)) {
		// Use fundamental matrix - check epipolar distance in pixel coordinates
		numInliers = CheckEpipolarInliersFundamental(F.value(), pts1, pts2, thresholdSq, mask);
	}
	else if (H.has_value() && (forceEpipolarType == -1 || forceEpipolarType == 3)) {
		// Use homography - check symmetric transfer error
		numInliers = CheckEpipolarInliersHomography(H.value(), pts1, pts2, thresholdSq, mask);
	}

	// Copy mask to output if requested
	if (inlierMask.needed())
		inlierMask.assign(cv::Mat(mask));
	return numInliers;
}
/*----------------------------------------------------------------*/
