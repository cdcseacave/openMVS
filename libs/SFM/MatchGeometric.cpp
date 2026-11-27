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

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

namespace {

// Number of nearest neighbours fetched per query to find the best keypoint outside the search region.
constexpr int K_NN = 8;

typedef CLISTDEF0(Point2f::EVec) Point2fs;
typedef TOctree<Point2fs, float, 2> Octree2f;

// Build a 2D spatial octree over an image's described keypoints (preserving original keypoint indices).
void BuildDescribedKeypointsOctree(const Image& img, Point2fs& pts, Octree2f& octree)
{
	const uint32_t numDescribed = img.NumDescribedKeypoints();
	pts.Resize(numDescribed);
	for (uint32_t j = 0; j < numDescribed; ++j)
		pts[j] = Point2f(img.keypoints[j].pt);
	octree.Insert(pts, [](Octree2f::IDX_TYPE n, Octree2f::Type r) { return n > 16 && r > 8.f; });
}

} // unnamed namespace


size_t SFM::MatchFeaturesGuided(
	PairsMatcher& pairsMatcher,
	const Image& imgA,
	const Image& imgB,
	const std::vector<Point2f>& trackedB,
	const std::vector<uchar>& trackStatus,
	const GuidedSearch& search,
	unsigned threadIdx,
	std::vector<DMatch>& matches)
{
	// the predictions are indexed like imgA's described keypoints (TrackKeypointsByWarp walks that
	// same prefix), and the caller owns one of pairsMatcher's per-thread descriptor matchers
	ASSERT(threadIdx < pairsMatcher.GetNumMatchers());
	ASSERT(imgA.HasDescriptors() && imgB.HasDescriptors());
	ASSERT(search.length > 0);
	// the per-thread matchers are built with the configured cross-check, and a cross-checking
	// BFMatcher refuses any k > 1 (cv::batchDistance asserts K == 1), so the batched query below
	// would throw; the guided path has no use for it anyway -- the ratio inside the band and the
	// reference outside it are what keep a match honest here, not a train-side collision
	ASSERT(!pairsMatcher.GetConfig().crossCheck);
	const uint32_t numDescribedA = imgA.NumDescribedKeypoints();
	const uint32_t numDescribedB = imgB.NumDescribedKeypoints();
	ASSERT(trackedB.size() == numDescribedA);
	ASSERT(trackStatus.size() == trackedB.size());

	matches.clear();
	if (numDescribedA == 0 || numDescribedB == 0)
		return 0;

	// 2D index over imgB's described keypoints (preserving original keypoint indices).
	Point2fs kptsB;
	Octree2f octree;
	BuildDescribedKeypointsOctree(imgB, kptsB, octree);

	const float length = search.length;
	const float lengthSq = length*length;
	const float halfWidth = MINF(search.halfWidth, length);
	const bool bBand = search.F.has_value() && halfWidth > 0.f;
	Eigen::Matrix3d F;
	Eigen::Vector2d epipoleB(0, 0);
	bool bEpipoleFinite = false;
	if (bBand) {
		F = search.F.value();
		// imgB's epipole is the left null vector of F (F^T e = 0)
		const Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU);
		const Eigen::Vector3d e = svd.matrixU().col(2);
		if (ABS(e.z()) > 1e-9*e.norm()) {
			epipoleB = e.hnormalized();
			bEpipoleFinite = true;
		}
	}
	struct Region {
		Point2f centre;   // the prediction
		float dirX, dirY; // the unit direction of the epipolar line (band only)
		bool band;        // band along that direction, else the disc
	};
	const auto RegionOf = [&](uint32_t i) {
		Region region{trackedB[i], 1.f, 0.f, false};
		if (!bBand)
			return region;
		if (bEpipoleFinite) {
			const double dx = (double)region.centre.x - epipoleB.x(), dy = (double)region.centre.y - epipoleB.y();
			if (dx*dx + dy*dy < 4.0*(double)lengthSq)
				return region;
		}
		const cv::Point2f& kpA = imgA.keypoints[i].pt;
		const Eigen::Vector3d l = F * Eigen::Vector3d(kpA.x, kpA.y, 1.0);
		const double norm = std::hypot(l.x(), l.y());
		if (norm < 1e-12)
			return region; // the line is undefined here (the keypoint is the epipole of imgA)
		region.dirX = (float)(-l.y()/norm);
		region.dirY = (float)(l.x()/norm);
		region.band = true;
		return region;
	};
	const auto Inside = [&](const Region& region, uint32_t j) {
		const cv::Point2f& pt = imgB.keypoints[j].pt;
		const float dx = pt.x - region.centre.x;
		const float dy = pt.y - region.centre.y;
		if (!region.band)
			return dx*dx + dy*dy <= lengthSq;
		const float along = dx*region.dirX + dy*region.dirY;
		const float across = -dx*region.dirY + dy*region.dirX;
		return ABS(along) <= length && ABS(across) <= halfWidth;
	};

	// descriptor distance in the matcher's own norm, evaluated on the stored descriptor rows
	const int normType = pairsMatcher.GetConfig().descriptorsAreBinary ? cv::NORM_HAMMING : cv::NORM_L2;
	const auto DescriptorDistance = [&](uint32_t i, uint32_t j) {
		return (float)cv::norm(imgA.descriptors.row((int)i), imgB.descriptors.row((int)j), normType);
	};

	// Step 1: the winner of every tracked keypoint's region -- the closest described keypoint of imgB
	// inside it -- and its rival, the closest other candidate that is not the winner described a
	// second time (a scale or orientation duplicate within sameFeatureDistance of it). Only these
	// queries can produce a match, so only they are decided below.
	struct Winner {
		uint32_t queryIdx;    // described keypoint of imgA
		uint32_t trainIdx;    // its closest described keypoint of imgB inside the region
		float distance;       // the descriptor distance of the two
		float rivalDistance;  // the descriptor distance of the closest rival inside the region, or -1 without one
		Region region;        // the region searched, for the outside reference
	};
	std::vector<Winner> winners;
	winners.reserve(numDescribedA);
	Octree2f::IDXARR_TYPE neighbors;
	std::vector<std::pair<uint32_t,float>> candidates; // (trainIdx, descriptor distance)
	const float sameFeatureDistanceSq = search.sameFeatureDistance*search.sameFeatureDistance;
	for (uint32_t i = 0; i < numDescribedA; ++i) {
		if (!trackStatus[i])
			continue; // the warp has nothing to say about this keypoint, so neither has this function
		const Region region = RegionOf(i);
		neighbors.Empty();
		octree.Collect(neighbors, region.centre, length);
		Winner winner{i, NO_ID, FLT_MAX, -1.f, region};
		candidates.clear();
		for (const Octree2f::IDX_TYPE idx : neighbors) {
			const uint32_t j = (uint32_t)idx;
			if (!Inside(region, j))
				continue;
			const float distance = DescriptorDistance(i, j);
			candidates.emplace_back(j, distance);
			// an equal distance keeps the smaller trainIdx, so two duplicate keypoints of imgB in
			// the same region always elect the same one of them, whatever order the octree walked
			// its cells in
			if (distance < winner.distance || (distance == winner.distance && j < winner.trainIdx)) {
				winner.trainIdx = j;
				winner.distance = distance;
			}
		}
		if (winner.trainIdx == NO_ID)
			continue;
		const cv::Point2f& ptWinner = imgB.keypoints[winner.trainIdx].pt;
		for (const auto& [j, distance] : candidates) {
			if (j == winner.trainIdx)
				continue;
			const cv::Point2f& pt = imgB.keypoints[j].pt;
			const float dx = pt.x - ptWinner.x, dy = pt.y - ptWinner.y;
			if (dx*dx + dy*dy <= sameFeatureDistanceSq)
				continue; // the winner itself, described again
			if (winner.rivalDistance < 0.f || distance < winner.rivalDistance)
				winner.rivalDistance = distance;
		}
		winners.emplace_back(winner);
	}
	if (winners.empty())
		return 0;

	// Step 2: the ratio test inside the region, where a rival exists: the winner has to beat the
	// closest rival there by the matcher's ratio, strictly, like every other ratio test in the
	// matcher -- two candidates as close as each other along the line is the very ambiguity this
	// test exists to refuse. A winner that passes it, or has no rival to pass, then answers to the
	// reference outside the region (Step 3).
	// Both sides here are TRUE descriptor norms (DescriptorDistance recomputes them), so this test
	// enforces d0/d1 < matchRatio, while PairsMatcher::MatchFeatures applies the same constant to
	// FLANN's SQUARED L2 distances and so enforces d0/d1 < sqrt(matchRatio): at the same setting the
	// guided pass is the stricter of the two (0.8 here against 0.894 there). See
	// MatchConfig::matchRatio (PairsMatcher.h)
	const float matchRatio = pairsMatcher.GetConfig().matchRatio;
	enum : uint8_t { REFUSED, ACCEPTED, UNDECIDED };
	std::vector<uint8_t> verdicts(winners.size(), UNDECIDED);
	std::vector<size_t> standing; // winners past their rival, still to answer to the outside reference
	FOREACH(q, winners) {
		const Winner& winner = winners[q];
		if (winner.rivalDistance >= 0.f && !(winner.distance < matchRatio*winner.rivalDistance))
			verdicts[q] = REFUSED;
		else
			standing.push_back(q);
	}

	// Step 3: the reference outside the region: every winner still standing has to beat the closest
	// described keypoint of imgB OUTSIDE the region by the same ratio, what refuses an impostor where
	// the true keypoint was never detected (see MatchGeometric.h). The thread's own descriptor matcher
	// answers all of them in one batched query over the whole described prefix of imgB, k = K_NN,
	// exactly as PairsMatcher::MatchFeatures runs its own (k = 2) one.
	if (!standing.empty()) {
		cv::Mat queryDescriptors((int)standing.size(), imgA.descriptors.cols, imgA.descriptors.type());
		FOREACH(k, standing)
			imgA.descriptors.row((int)winners[standing[k]].queryIdx).copyTo(queryDescriptors.row((int)k));
		std::vector<std::vector<cv::DMatch>> knnMatches;
		pairsMatcher.KnnMatch(queryDescriptors, imgB.descriptors, knnMatches, K_NN, threadIdx);
		// one neighbour list per query is the matcher's contract, and knnMatch keeps it: with the
		// default compactResult=false it returns one list per query row, and the one path that
		// returns fewer (an empty train set) is excluded by the numDescribedB == 0 early return
		// above. Asserted rather than handled, so the return value below means only how many
		// matches this pass accepted
		ASSERT(knnMatches.size() == standing.size());
		FOREACH(k, standing) {
			const Winner& winner = winners[standing[k]];
			const std::vector<cv::DMatch>& neighbours = knnMatches[k];
			if (neighbours.empty()) {
				verdicts[standing[k]] = REFUSED; // no reference distance, so nothing this match could be shown to beat
				continue;
			}
			// the neighbours come back sorted by distance, so the first one outside the region is
			// the closest keypoint outside it.
			// Their distances are recomputed instead of read off the matcher: an approximate matcher
			// reports its index's own distance (squared, for the FLANN L2 trees), while the winner's
			// distance above is the matcher's norm, and the ratio only means something if the two
			// sides of it are measured the same way. Only the identity of the neighbours is taken
			// from it.
			float outsideDistance = -1.f;
			for (const cv::DMatch& neighbour : neighbours) {
				const uint32_t j = (uint32_t)neighbour.trainIdx;
				if (Inside(winner.region, j))
					continue;
				outsideDistance = DescriptorDistance(winner.queryIdx, j);
				break;
			}
			if (outsideDistance < 0.f) {
				// every neighbour returned sits inside the region: if they are all of imgB's
				// described keypoints there is nothing outside for the winner to beat and it stands,
				// otherwise the K_NN-th distance stands in -- every keypoint outside the region is
				// at least that far away, so the test can only get stricter, never looser
				if (neighbours.size() == (size_t)numDescribedB) {
					verdicts[standing[k]] = ACCEPTED;
					continue;
				}
				outsideDistance = DescriptorDistance(winner.queryIdx, (uint32_t)neighbours.back().trainIdx);
			}
			// strictly better: a rival exactly as close as the winner but outside the region is the
			// very thing this test exists to reject, and equal descriptors (the same feature
			// described twice, in two different places of imgB) make that tie a real case rather
			// than a floating-point curiosity
			verdicts[standing[k]] = winner.distance < matchRatio*outsideDistance ? ACCEPTED : REFUSED;
		}
	}

	// the winners were collected in increasing queryIdx and kept in that order, so the matches are
	// in increasing queryIdx too
	matches.reserve(winners.size());
	FOREACH(q, winners)
		if (verdicts[q] == ACCEPTED)
			matches.emplace_back(winners[q].queryIdx, winners[q].trainIdx);
	return matches.size();
}
/*----------------------------------------------------------------*/


bool SFM::MatchFeaturesGeometric(
	PairsMatcher& pairsMatcher,
	const Image& img1,
	const Image& img2,
	const std::vector<Point2f>& trackedPoints1,
	const std::vector<Point2f>& trackedPoints2,
	const std::vector<uchar>& trackStatus,
	ImagePair& pair,
	float epipolarThreshold,
	unsigned threadIdx)
{
	// Sanity check: the described keypoints of img1 correspond to trackedPoints1 by index
	// (the tracker walks that same prefix, and only those keypoints have a descriptor row for the
	// ratio test below to compare) and the caller owns one of pairsMatcher's per-thread descriptor
	// matchers
	ASSERT(threadIdx < pairsMatcher.GetNumMatchers());
	const uint32_t numDescribed1 = img1.NumDescribedKeypoints();
	const uint32_t numDescribed2 = img2.NumDescribedKeypoints();
	ASSERT(numDescribed1 == trackedPoints1.size());
	ASSERT(trackedPoints1.size() == trackedPoints2.size());
	ASSERT(trackStatus.size() == trackedPoints1.size());

	pair.Reset();

	// Step 1: the geometry Step 2 guides on, estimated from the tracked points.
	// Initialize pair with tracked points as initial matches. A pair that tracked almost nothing
	// has neither the points to fit a geometry with nor the spatial-disc centres Step 2 needs.
	for (size_t i = 0; i < trackStatus.size(); ++i)
		if (trackStatus[i])
			pair.matches.emplace_back((uint32_t)i, (uint32_t)i);
	if (pair.matches.size() < pairsMatcher.GetConfig().minMatches) {
		DEBUG("MatchFeaturesGeometric: insufficient tracked points (%zu) for F-matrix estimation", pair.matches.size());
		// Fallback to descriptor-only matching
		pairsMatcher.MatchFeatures(img1.descriptors, img2.descriptors, pair.matches, threadIdx);
		return false;
	}
	// Make copies to avoid modifying original images
	Image img1Copy(img1.ID, img1.fileName, reinterpret_cast<const Pose3D&>(img1), img1.cameraID, img1.pCamera);
	Image img2Copy(img2.ID, img2.fileName, reinterpret_cast<const Pose3D&>(img2), img2.cameraID, img2.pCamera);
	img1Copy.keypoints = ConvertToKeypoints(trackedPoints1);
	img2Copy.keypoints = ConvertToKeypoints(trackedPoints2);
	// Use GeometricFilter to estimate geometry from tracked points
	if (!pairsMatcher.GeometricFilter(img1Copy, img2Copy, pair)) {
		DEBUG("MatchFeaturesGeometric: GeometricFilter failed, falling back to descriptor-only matching");
		pair.matches.clear();
		pairsMatcher.MatchFeatures(img1.descriptors, img2.descriptors, pair.matches, threadIdx);
		return false;
	}
	if (pair.GetNumFilteredInliers() < pairsMatcher.GetConfig().minMatches) {
		DEBUG("MatchFeaturesGeometric: PoseLib estimation failed, falling back to descriptor-only matching");
		pair.ResetMatches();
		pairsMatcher.MatchFeatures(img1.descriptors, img2.descriptors, pair.matches, threadIdx);
		return false;
	}
	pair.ResetMatches();

	// Step 2: Match descriptors with epipolar and ratio constraints
	// For each feature in image1, find matches satisfying both geometric and descriptor constraints.
	// We further restrict the search to a spatial neighborhood around the tracked point in image2
	// (trackedPoints2[i]) to avoid scanning the entire epipolar line.
	// Choose a reasonable spatial radius: at least a few pixels, scaled from epipolarThreshold
	const float spatialThreshold = MAXF(10.f, epipolarThreshold * 6.f);

	// Build a 2D octree over img2's described keypoints for fast spatial neighbor queries
	Point2fs kpts2;
	Octree2f octree;
	BuildDescribedKeypointsOctree(img2, kpts2, octree);

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
		if (candidates[0].distance < matchRatio * candidates[1].distance)
			pair.matches.push_back(candidates[0]);
	};

	// Candidate lookup: spatial neighbors around trackedPoints2[i], falling back to all described keypoints
	const auto ForEachCandidate = [&](size_t i, const auto& TestCandidate) {
		if (trackStatus[i]) {
			Octree2f::IDXARR_TYPE neighbors;
			octree.Collect(neighbors, trackedPoints2[i], spatialThreshold);
			if (!neighbors.empty()) {
				for (const Octree2f::IDX_TYPE idx : neighbors)
					TestCandidate(idx);
				return;
			}
		}
		for (uint32_t j = 0; j < numDescribed2; ++j)
			TestCandidate(j);
	};

	// Use pixel-space F for distortion-free pinhole pairs (or untrusted intrinsics); otherwise use bearing-space E
	const bool bLens = (img1.pCamera->HasDistortion() || img2.pCamera->HasDistortion()) &&
		img1.TrustIntrinsics() && img2.TrustIntrinsics();
	if (pair.F.has_value() && !(bLens && pair.E.has_value())) {
		const Matrix3x3f F = pair.F.value();
		for (uint32_t i = 0; i < numDescribed1; ++i) {
			const Point2f& pt1 = img1.keypoints[i].pt;

			// Compute epipolar line in image2: L = F * pt1
			const Point3f line = F * pt1.homogeneous();
			const float normFactor = SQRT(line.x*line.x + line.y*line.y);
			if (normFactor < FZERO_TOLERANCE)
				continue;

			std::vector<cv::DMatch> candidates;
			const auto TestCandidate = [&](size_t j) {
				const cv::Point2f& pt2 = img2.keypoints[j].pt;
				const float distance = ABS(line.x * pt2.x + line.y * pt2.y + line.z) / normFactor;
				if (distance < epipolarThreshold)
					candidates.emplace_back((int)i, (int)j, 0.f);
			};

			ForEachCandidate(i, TestCandidate);
			SelectAndAppendBest(candidates, (size_t)i);
		}
	} else if (pair.E.has_value()) {
		// Spherical, mixed, or distorted-pinhole path: Sampson-on-sphere in bearing space
		const Eigen::Matrix3d E = pair.E.value();
		const double angleThreshold = (double)PairPixelErrorToAngular(*img1.pCamera, *img2.pCamera, (REAL)epipolarThreshold);
		const double angleThresholdSq = angleThreshold * angleThreshold;

		std::vector<Eigen::Vector3d> bearings1(numDescribed1);
		std::vector<Eigen::Vector3d> bearings2(numDescribed2);
		for (uint32_t i = 0; i < numDescribed1; ++i)
			bearings1[i] = img1.pCamera->UnprojectNormalized(Cast<REAL>(img1.keypoints[i].pt));
		for (uint32_t i = 0; i < numDescribed2; ++i)
			bearings2[i] = img2.pCamera->UnprojectNormalized(Cast<REAL>(img2.keypoints[i].pt));

		for (uint32_t i = 0; i < numDescribed1; ++i) {
			const Eigen::Vector3d& b1 = bearings1[i];
			const Eigen::Vector3d Eb1 = E * b1;
			if (Eb1.squaredNorm() < 1e-14)
				continue; // degenerate epipolar plane (bearing aligned with baseline)

			std::vector<cv::DMatch> candidates;
			const auto TestCandidate = [&](size_t j) {
				const Eigen::Vector3d& b2 = bearings2[j];
				const double C = b2.dot(Eb1);
				const Eigen::Vector3d Etb2 = E.transpose() * b2;
				// Tangent-plane Sampson denominator on the unit sphere: ||b2 x (E b1)||^2 + ||b1 x (E^T b2)||^2
				const double denom = b2.cross(Eb1).squaredNorm() + b1.cross(Etb2).squaredNorm();
				if (denom > 1e-14 && (C * C) < angleThresholdSq * denom)
					candidates.emplace_back((int)i, (int)j, 0.f);
			};

			ForEachCandidate(i, TestCandidate);
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
