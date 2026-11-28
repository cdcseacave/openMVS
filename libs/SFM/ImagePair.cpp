////////////////////////////////////////////////////////////////////
// ImagePair.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ImagePair.h"
#include "Image.h"
#include <PoseLib/misc/essential.h>

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
	const std::vector<Eigen::Vector3d>& bearings1,
	const std::vector<Eigen::Vector3d>& bearings2,
	Pose3D& pose)
{
	ASSERT(bearings1.size() == bearings2.size());
	// the four factorizations of the SVD of E (no correspondence given, so none is filtered out here)
	poselib::CameraPoseVector candidates;
	poselib::motion_from_essential_svd(E, {}, {}, &candidates);
	ASSERT(candidates.size() == 4);
	unsigned numBest = 0;
	for (const poselib::CameraPose& candidate : candidates) {
		unsigned numInFront = 0;
		FOREACH(i, bearings1)
			if (poselib::check_cheirality(candidate, bearings1[i], bearings2[i]))
				++numInFront;
		if (numBest < numInFront) {
			numBest = numInFront;
			pose.R = candidate.R();
			pose.SetT(candidate.t);
		}
	}
	return numBest;
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
		return 1.f; // NOT a neutral value: 1 is this function's MAXIMUM, the score of an optimal
		            // baseline. A pair with no measured angle is therefore not demoted at all, which
		            // is only defensible while "no angle" means "not measurable" -- with a relative
		            // pose the angle IS measurable over every track-forming match (FilterMatches,
		            // ImagePair::ComputeMeanRayAngle), and a pair that has one must carry it

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

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> ImagePair::GetMatchedBearings(const Image& img1, const Image& img2, bool allInliers) const
{
	const unsigned numInliers = allInliers ? GetNumInliers() : GetNumFilteredInliers();
	std::vector<Eigen::Vector3d> bearings1, bearings2;
	bearings1.reserve(numInliers);
	bearings2.reserve(numInliers);
	for (unsigned i = 0; i < numInliers; ++i) {
		const DMatch& m = matches[i];
		bearings1.emplace_back(img1.pCamera->UnprojectNormalized(Cast<REAL>(img1.keypoints[m.queryIdx].pt)));
		bearings2.emplace_back(img2.pCamera->UnprojectNormalized(Cast<REAL>(img2.keypoints[m.trainIdx].pt)));
	}
	return std::make_pair(bearings1, bearings2);
}

bool ImagePair::IsDense(const Image& img1, const Image& img2) const
{
	return GetNumDenseInliers() > 0 || img1.HasDenseKeypoints() || img2.HasDenseKeypoints();
}

bool ImagePair::UseFundamentalForPair(const Image& img1, const Image& img2)
{
	return img1.GetCameraType() == CameraType::PINHOLE && img2.GetCameraType() == CameraType::PINHOLE &&
		(!img1.TrustIntrinsics() || !img2.TrustIntrinsics());
}

bool ImagePair::TriangulateVerifiedMatch(
	const Pose3D& relPose,
	const Camera& cam1,
	const Camera& cam2,
	const Point2f& pt1,
	const Point2f& pt2,
	REAL cosReprojAngle,
	REAL maxCosAngle,
	Point3& X,
	REAL& cosAngle,
	REAL epipoleCosAngle,
	const Point3& epipoleDir1,
	const Point3& epipoleDir2)
{
	const Point3 b1 = cam1.UnprojectNormalized(Cast<REAL>(pt1));
	const Point3 b2 = cam2.UnprojectNormalized(Cast<REAL>(pt2));
	if (epipoleCosAngle <= REAL(1) &&
		(b1.dot(epipoleDir1) > epipoleCosAngle || b2.dot(epipoleDir2) > epipoleCosAngle))
		return false;
	if (!TriangulatePoint3D(relPose.R, relPose.C, b1, b2, X))
		return false;
	const REAL nX = norm(X);
	if (nX < ZEROTOLERANCE<REAL>())
		return false;
	const REAL cosErr1 = b1.dot(X * (REAL(1) / nX));
	if (cosErr1 <= 0)
		return false;
	const Point3 Xcam2 = relPose.TransformPointW2C(X);
	const REAL nXc2 = norm(Xcam2);
	if (nXc2 < ZEROTOLERANCE<REAL>())
		return false;
	const REAL cosErr2 = b2.dot(Xcam2 * (REAL(1) / nXc2));
	if (cosErr2 <= 0)
		return false;
	if (cosReprojAngle > REAL(-1) && (cosErr1 < cosReprojAngle || cosErr2 < cosReprojAngle))
		return false;
	const Point3 V2 = X - relPose.C;
	cosAngle = ComputeAngle(X.ptr(), V2.ptr());
	if (maxCosAngle < REAL(1) && cosAngle > maxCosAngle)
		return false;
	return true;
}

std::pair<std::vector<Point2f>, std::vector<Point2f>> ImagePair::GetTrackFormingPoints(const Image& img1, const Image& img2) const
{
	std::vector<Point2f> pts1, pts2;
	const unsigned numTrackForming = GetNumTrackFormingMatches();
	pts1.reserve(numTrackForming);
	pts2.reserve(numTrackForming);
	for (unsigned i = 0; i < numTrackForming; ++i) {
		const DMatch& m = matches[i];
		pts1.push_back(img1.keypoints[m.queryIdx].pt);
		pts2.push_back(img2.keypoints[m.trainIdx].pt);
	}
	return std::make_pair(pts1, pts2);
}

FloatArr ImagePair::ComputeRayAngleCosines(const Image& img1, const Image& img2) const
{
	if (!relativePose.has_value() || matches.empty())
		return FloatArr(); // no pose, no measurable baseline
	const Pose3D& relPose = relativePose.value();
	const Camera& cam1 = *img1.pCamera;
	const Camera& cam2 = *img2.pCamera;
	const unsigned numTrackForming = GetNumTrackFormingMatches();
	FloatArr cosAngles(0, numTrackForming);
	for (unsigned i = 0; i < numTrackForming; ++i) {
		const DMatch& m = matches[i];
		const Point3 b1 = cam1.UnprojectNormalized(Cast<REAL>(img1.keypoints[m.queryIdx].pt));
		const Point3 b2 = cam2.UnprojectNormalized(Cast<REAL>(img2.keypoints[m.trainIdx].pt));
		Point3 X;
		if (!TriangulatePoint3D(relPose.R, relPose.C, b1, b2, X))
			continue;
		const Point3 V2 = X - relPose.C;
		cosAngles.push_back((float)ComputeAngle(X.ptr(), V2.ptr()));
	}
	return cosAngles;
}

REAL ImagePair::ComputeBaselineDirectionSigma(const Image& img1, const Image& img2, REAL noiseAngle) const
{
	ASSERT(relativePose.has_value());
	ASSERT(noiseAngle > 0);
	const Pose3D& relPose = relativePose.value();
	const REAL baselineLength = norm(relPose.C);
	ASSERT(baselineLength > 0); // a relative pose has a baseline
	const Camera& cam1 = *img1.pCamera;
	const Camera& cam2 = *img2.pCamera;
	const Point3 t(relPose.C * (REAL(1) / baselineLength));
	Point3 e1, e2;
	OrthonormalBasis(t, e1, e2);
	const Matrix3x3 Rt(relPose.R.t());
	REAL a11 = 0, a12 = 0, a22 = 0;
	const unsigned numTrackForming = GetNumTrackFormingMatches();
	for (unsigned i = 0; i < numTrackForming; ++i) {
		const DMatch& m = matches[i];
		const Point3 b1(cam1.UnprojectNormalized(Cast<REAL>(img1.keypoints[m.queryIdx].pt)));
		const Point3 b2(Rt * cam2.UnprojectNormalized(Cast<REAL>(img2.keypoints[m.trainIdx].pt)));
		ASSERT(ISEQUAL(normSq(b1), REAL(1), REAL(1e-4)) && ISEQUAL(normSq(b2), REAL(1), REAL(1e-4)));
		Point3 n(t.cross(b1)); // the normal of the epipolar plane of the match
		const REAL sinBaseline = norm(n);
		if (sinBaseline <= REAL(1e-9))
			continue; // the match lies on the baseline: the epipolar plane is undefined
		n *= REAL(1) / sinBaseline;
		const Point3 j(b1.cross(b2 - n * n.dot(b2)) * (REAL(1) / sinBaseline));
		const REAL j1 = j.dot(e1), j2 = j.dot(e2);
		a11 += j1 * j1;
		a12 += j1 * j2;
		a22 += j2 * j2;
	}
	const REAL lambdaMin = REAL(0.5) * ((a11 + a22) - SQRT((a11 - a22) * (a11 - a22) + REAL(4) * a12 * a12));
	return lambdaMin > 0 ? noiseAngle / SQRT(lambdaMin) : std::numeric_limits<REAL>::infinity();
}

float ImagePair::ComputeMeanRayAngle(const Image& img1, const Image& img2) const
{
	FloatArr cosAngles(ComputeRayAngleCosines(img1, img2));
	return cosAngles.empty() ? 0.f : ACOS(CLAMP(cosAngles.GetMedian(), -1.f, 1.f));
}

float ImagePair::ComputeShareAboveAngle(const Image& img1, const Image& img2, float minAngle) const
{
	const FloatArr cosAngles(ComputeRayAngleCosines(img1, img2));
	const float maxCosAngle = COS(D2R(minAngle));
	unsigned numWide = 0;
	for (float cosAngle : cosAngles)
		numWide += cosAngle <= maxCosAngle ? 1 : 0;
	return (float)numWide / MAXF(GetNumTrackFormingMatches(), 1u);
}

void ImagePair::CheckSparseSegmentIsDescribed(const Image& img1, const Image& img2) const
{
	#ifndef _RELEASE
	if (numFilteredInliers < 0) {
		ASSERT(numDenseInliers == 0); // no partition claimed, so no segment to check
		return;
	}
	ASSERT((size_t)numFilteredInliers + (size_t)numDenseInliers <= matches.size());
	for (int i = 0; i < numFilteredInliers; ++i) {
		const DMatch& m = matches[i];
		ASSERT(!img1.IsDenseKeypoint(m.queryIdx) && !img2.IsDenseKeypoint(m.trainIdx),
			"a dense supplement match drifted into the pair's descriptor evidence");
	}
	#endif
}

unsigned ImagePair::FilterMatches(const Image& img1, const Image& img2, float minAngle, float reprojThreshold, float epipoleThresh)
{
	if (!relativePose.has_value() || matches.empty())
		return matches.size(); // nothing to do

	const Pose3D& relPose = relativePose.value();
	const Camera& cam1 = *img1.pCamera;
	const Camera& cam2 = *img2.pCamera;

	// Angular thresholds work uniformly for pinhole and spherical cameras
	const REAL cosReprojAngle = reprojThreshold > 0 ?
		COS(PairPixelErrorToAngular(cam1, cam2, reprojThreshold)) : REAL(-1);

	Point3 epipoleDir1(Point3::ZERO), epipoleDir2(Point3::ZERO);
	REAL cosEpipoleAngle = REAL(2); // > 1 disables epipole filter
	if (epipoleThresh > 0) {
		cosEpipoleAngle = COS(PairPixelErrorToAngular(cam1, cam2, epipoleThresh));
		const REAL nC = norm(relPose.C);
		if (nC > ZEROTOLERANCE<REAL>())
			epipoleDir1 = relPose.C / nC;
		const Point3 t = relPose.GetT();
		const REAL nT = norm(t);
		if (nT > ZEROTOLERANCE<REAL>())
			epipoleDir2 = t / nT;
	}

	const REAL maxCosAngle = minAngle > 0 ? COS(D2R(REAL(minAngle))) : REAL(2);
	const auto [pts1, pts2] = GetMatchedPoints(img1, img2, true);
	const bool bHasDenseKeypoints = img1.HasDenseKeypoints() || img2.HasDenseKeypoints();
	const auto HasDenseEnd = [&img1, &img2](const DMatch& m) {
		return img1.IsDenseKeypoint(m.queryIdx) || img2.IsDenseKeypoint(m.trainIdx);
	};
	std::vector<char> mask(matches.size(), 0);
	FloatArr cosAngles(0, matches.size());
	unsigned numInliers = 0;
	FOREACH(i, matches) {
		Point3 X;
		REAL cosAngle;
		if (!TriangulateVerifiedMatch(relPose, cam1, cam2, pts1[i], pts2[i],
				cosReprojAngle, maxCosAngle, X, cosAngle, cosEpipoleAngle, epipoleDir1, epipoleDir2))
			continue;
		cosAngles.push_back((float)cosAngle);
		mask[i] = 1;
		++numInliers;
	}

	meanRayAngle = cosAngles.empty() ? 0.f : ACOS(CLAMP(cosAngles.GetMedian(), -1.f, 1.f));
	// Count surviving dense supplement matches whose endpoints both collapsed onto described keypoints
	unsigned numSupplementReclassifiedSparse = 0;
	if (bHasDenseKeypoints && numFilteredInliers >= 0 && numDenseInliers > 0) {
		const unsigned oldDenseBegin = (unsigned)numFilteredInliers;
		const unsigned oldDenseEnd = oldDenseBegin + GetNumDenseInliers();
		for (unsigned i = oldDenseBegin; i < oldDenseEnd; ++i)
			if (mask[i] && !HasDenseEnd(matches[i]))
				++numSupplementReclassifiedSparse;
	}
	numFilteredInliers = (int)PartitionMatchesByMask(mask, (int)numInliers, true);
	// Re-derive the dense segment after reordering so dense matches stay separated from sparse evidence
	if (bHasDenseKeypoints) {
		const auto denseBegin = std::stable_partition(matches.begin(), matches.begin() + numFilteredInliers,
			[&HasDenseEnd](const DMatch& m) { return !HasDenseEnd(m); });
		numDenseInliers = (int)(matches.begin() + numFilteredInliers - denseBegin);
		numFilteredInliers -= numDenseInliers;
		if (numSupplementReclassifiedSparse > 0)
			DEBUG("FilterMatches: %u dense supplement match(es) re-classified as sparse in pair (%u, %u)",
				numSupplementReclassifiedSparse, ID1, ID2);
	}
	CheckSparseSegmentIsDescribed(img1, img2);
	return (unsigned)numFilteredInliers;
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

	// Extract all matched points (all segments, matching inlierMask length)
	auto [pts1, pts2] = GetMatchedPoints(img1, img2, true);

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
