////////////////////////////////////////////////////////////////////
// MatchGeometric.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "MatchGeometric.h"
#include "Image.h"
#include "ImagePair.h"
// PoseLib for robust relative/fundamental estimation
#include <PoseLib/poselib.h>

using namespace SFM;


bool SFM::MatchFeaturesGeometric(
	PairsMatcher& pairsMatcher,
	const Image& img1,
	const Image& img2,
	const std::vector<Point2f>& trackedPoints1,
	const std::vector<Point2f>& trackedPoints2,
	const std::vector<uchar>& trackStatus,
	ImagePair& pair,
	float epipolarThreshold)
{
	// Sanity check: keypoints1 correspond to trackedPoints1 by index
	ASSERT(img1.keypoints.size() == trackedPoints1.size());
	ASSERT(trackedPoints1.size() == trackedPoints2.size());
	ASSERT(trackStatus.size() == trackedPoints1.size());

	pair.Reset();

	// Step 1: Estimate relative pose / F from tracked points
	// Initialize pair with tracked points as initial matches
	for (size_t i = 0; i < trackStatus.size(); ++i)
		if (trackStatus[i])
			pair.matches.emplace_back((uint32_t)i, (uint32_t)i);
	if (pair.matches.size() < pairsMatcher.GetConfig().minMatches) {
		DEBUG("MatchFeaturesGeometric: insufficient tracked points (%zu) for F-matrix estimation", pair.matches.size());
		// Fallback to descriptor-only matching
		pairsMatcher.MatchFeatures(img1.descriptors, img2.descriptors, pair.matches);
		return false;
	}
	{
		// Make copies to avoid modifying original images
		Image img1Copy(img1.ID, img1.fileName, reinterpret_cast<const Pose3D&>(img1), img1.cameraID, img1.pCamera);
		Image img2Copy(img2.ID, img2.fileName, reinterpret_cast<const Pose3D&>(img2), img2.cameraID, img2.pCamera);
		img1Copy.keypoints = ConvertToKeypoints(trackedPoints1);
		img2Copy.keypoints = ConvertToKeypoints(trackedPoints2);
		// Use GeometricFilter to estimate geometry from tracked points
		if (!pairsMatcher.GeometricFilter(img1Copy, img2Copy, pair)) {
			DEBUG("MatchFeaturesGeometric: GeometricFilter failed, falling back to descriptor-only matching");
			pair.matches.clear();
			pairsMatcher.MatchFeatures(img1.descriptors, img2.descriptors, pair.matches);
			return false;
		}
	}
	if (pair.GetNumFilteredInliers() < pairsMatcher.GetConfig().minMatches) {
		DEBUG("MatchFeaturesGeometric: PoseLib estimation failed, falling back to descriptor-only matching");
		pair.ResetMatches();
		pairsMatcher.MatchFeatures(img1.descriptors, img2.descriptors, pair.matches);
		return false;
	}
	pair.ResetMatches();

	// Step 2: Match descriptors with epipolar and ratio constraints
	// For each feature in image1, find matches satisfying both geometric and descriptor constraints.
	// We further restrict the search to a spatial neighborhood around the tracked point in image2
	// (trackedPoints2[i]) to avoid scanning the entire epipolar line.
	// Choose a reasonable spatial radius: at least a few pixels, scaled from epipolarThreshold
	const float spatialThreshold = MAXF(10.f, epipolarThreshold * 6.f);

	// Build a 2D octree over keypoints2 for fast spatial neighbor queries around trackedPoints2.
	typedef CLISTDEF0(Point2f::EVec) Point2fs;
	Point2fs kpts2(img2.keypoints.size());
	FOREACH(i, img2.keypoints) {
		const cv::KeyPoint& keypoint = img2.keypoints[i];
		kpts2[i] = Point2f::EVec(keypoint.pt.x, keypoint.pt.y);
	}
	typedef TOctree<Point2fs, float, 2> Octree2f;
	Octree2f octree(kpts2, [](Octree2f::IDX_TYPE n, Octree2f::Type r) { return n > 16 && r > 8.f; });

	const float matchRatio = pairsMatcher.GetConfig().matchRatio;
	const int normType = pairsMatcher.GetConfig().descriptorsAreBinary ? cv::NORM_HAMMING : cv::NORM_L2;

	// Descriptor-based winner selection shared between the F-based and E-based paths.
	const auto SelectAndAppendBest = [&](std::vector<cv::DMatch>& candidates, size_t i) {
		if (candidates.empty())
			return;
		if (candidates.size() == 1) {
			pair.matches.push_back(candidates[0]);
			return;
		}
		cv::Mat desc1 = img1.descriptors.row((int)i);
		for (auto& candidate : candidates) {
			cv::Mat desc2 = img2.descriptors.row(candidate.trainIdx);
			candidate.distance = (float)cv::norm(desc1, desc2, normType);
		}
		std::sort(candidates.begin(), candidates.end(),
			[](const cv::DMatch& a, const cv::DMatch& b) {
				return a.distance < b.distance;
			});
		// Ratio test: best must be meaningfully better than second-best.
		if (candidates[0].distance < matchRatio * candidates[1].distance)
			pair.matches.push_back(candidates[0]);
	};

	// Branch on pair.F availability. PairsMatcher::GeometricFilter only sets
	// pair.F when BOTH cameras are pinhole — for spherical or mixed pairs
	// the fundamental matrix is not geometrically meaningful (SphericalCamera::GetK
	// returns IDENTITY), and pair.F is left empty. We dispatch:
	//   - F present  -> pinhole pixel-space epipolar line distance (unchanged)
	//   - F absent   -> bearing-space Sampson-on-sphere residual with an
	//                   angular threshold derived per-camera from epipolarThreshold.
	// For pinhole bearings the Sampson-on-sphere formula reduces exactly to the
	// pinhole Sampson form (up to linear scaling), so the two paths agree on
	// pinhole inputs up to the unit of the threshold. Keeping the F path
	// separate preserves zero-regression on all pinhole tests.
	if (pair.F.has_value()) {
		const Matrix3x3f F = pair.F.value();
		FOREACH(i, img1.keypoints) {
			const Point2f& pt1 = img1.keypoints[i].pt;

			// Compute epipolar line in image2: L = F * pt1
			const Point3f line = F * pt1.homogeneous();
			const float normFactor = SQRT(line.x*line.x + line.y*line.y);
			if (normFactor < FZERO_TOLERANCE)
				continue;

			// Find candidate matches near the epipolar line AND (if tracked) close to expectedPt2
			std::vector<cv::DMatch> candidates;
			const auto TestCandidate = [&](size_t j) {
				const cv::Point2f& pt2 = img2.keypoints[j].pt;
				const float distance = ABS(line.x * pt2.x + line.y * pt2.y + line.z) / normFactor;
				if (distance < epipolarThreshold)
					candidates.emplace_back((int)i, (int)j, 0.f);
			};

			if (trackStatus[i]) {
				const Point2f& expectedPt2 = trackedPoints2[i];
				Octree2f::IDXARR_TYPE neighbors;
				octree.Collect(neighbors, expectedPt2, spatialThreshold);
				if (neighbors.empty())
					goto PBruteForceFallback;
				for (const Octree2f::IDX_TYPE idx : neighbors)
					TestCandidate(idx);
			} else {
				PBruteForceFallback:
				// fallback: scan all keypoints2 and use only epipolar constraint
				FOREACH(j, img2.keypoints)
					TestCandidate(j);
			}
			SelectAndAppendBest(candidates, (size_t)i);
		}
	} else if (pair.E.has_value()) {
		// Spherical / mixed path: E-matrix + bearing vectors.
		// Convert the pixel epipolar threshold to a symmetric angular threshold
		// (same averaging convention as PairsMatcher::GeometricFilter). The
		// Sampson-on-sphere residual is radians-scaled in the small-error limit,
		// so we compare r² against angleThreshold².
		const Eigen::Matrix3d E = pair.E.value(); // implicit TMatrix<double,3,3> -> Eigen::Matrix3d
		const REAL angle1 = img1.pCamera->PixelErrorToAngular((REAL)epipolarThreshold);
		const REAL angle2 = img2.pCamera->PixelErrorToAngular((REAL)epipolarThreshold);
		const double angleThreshold = 0.5 * (double)(angle1 + angle2);
		const double angleThresholdSq = angleThreshold * angleThreshold;

		// Precompute unit bearing vectors for all keypoints in both images once.
		// Each bearing costs a single Unproject call, and we reuse them across
		// many candidate probes (up to #img2_keypoints per img1 keypoint in the
		// brute-force case), so hoisting them out of the inner loop is a real win.
		std::vector<Eigen::Vector3d> bearings1(img1.keypoints.size());
		std::vector<Eigen::Vector3d> bearings2(img2.keypoints.size());
		FOREACH(i, img1.keypoints)
			bearings1[i] = img1.pCamera->UnprojectNormalized(Cast<REAL>(img1.keypoints[i].pt));
		FOREACH(i, img2.keypoints)
			bearings2[i] = img2.pCamera->UnprojectNormalized(Cast<REAL>(img2.keypoints[i].pt));

		FOREACH(i, img1.keypoints) {
			const Eigen::Vector3d& b1 = bearings1[i];
			const Eigen::Vector3d Eb1 = E * b1;
			// Sampson (x,y)-subspace term from the "left" bearing — constant across
			// all candidates j for this i.
			const double Cx = Eb1.x() * Eb1.x() + Eb1.y() * Eb1.y();
			if (Cx < 1e-14)
				continue; // degenerate epipolar plane (bearing aligned with baseline)

			// Candidate test closure using Sampson-on-sphere.
			std::vector<cv::DMatch> candidates;
			const auto TestCandidate = [&](size_t j) {
				const Eigen::Vector3d& b2 = bearings2[j];
				const double C = b2.dot(Eb1);
				const Eigen::Vector3d Etb2 = E.transpose() * b2;
				const double Cy = Etb2.x() * Etb2.x() + Etb2.y() * Etb2.y();
				const double r2 = (C * C) / (Cx + Cy);
				if (r2 < angleThresholdSq)
					candidates.emplace_back((int)i, (int)j, 0.f);
			};

			if (trackStatus[i]) {
				const Point2f& expectedPt2 = trackedPoints2[i];
				Octree2f::IDXARR_TYPE neighbors;
				octree.Collect(neighbors, expectedPt2, spatialThreshold);
				if (neighbors.empty())
					goto SBruteForceFallback;
				for (const Octree2f::IDX_TYPE idx : neighbors)
					TestCandidate(idx);
			} else {
				SBruteForceFallback:
				// fallback: scan all keypoints2 and use only epipolar constraint
				FOREACH(j, img2.keypoints)
					TestCandidate(j);
			}
			SelectAndAppendBest(candidates, (size_t)i);
		}
	}
	if (pair.matches.size() < pairsMatcher.GetConfig().minMatches) {
		pair.InvalidateMatches();
		return false;
	}
	if (pairsMatcher.GetConfig().IsMatchesFilterOn()) {
		// Further filter matches based on triangulation angle, reprojection error, epipole proximity
		const unsigned numFilteredInliers = pair.FilterMatches(img1, img2, pairsMatcher.GetConfig().minTriangulationAngle, pairsMatcher.GetConfig().reprojThreshold, pairsMatcher.GetConfig().epipoleFilterThreshold);
		if (numFilteredInliers < pairsMatcher.GetConfig().minMatches) {
			pair.InvalidateMatches();
			return false;
		}
	}
	return true;
}
/*----------------------------------------------------------------*/
