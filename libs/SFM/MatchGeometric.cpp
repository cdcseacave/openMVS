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

// How many nearest neighbours MatchFeaturesGuided fetches per query to find the best keypoint
// outside the search region, for a winner alone in it. The region can only hold the true match and,
// at most, a handful of its scale/orientation duplicates, so the closest keypoint outside it is within
// a short list of the query's nearest neighbours; a list of this length answers that with one batched
// query, instead of an index over the complement of the region, which is a different set for every
// query.
constexpr int K_NN = 8;

} // unnamed namespace


// S T R U C T S ///////////////////////////////////////////////////

float SFM::GuidedBandHalfWidth(std::vector<float> residuals, float factor, float minHalfWidth, float maxHalfWidth)
{
	ASSERT(factor > 0.f && minHalfWidth <= maxHalfWidth);
	if (residuals.empty())
		return minHalfWidth; // nothing measured: the floor is all that can be claimed
	// the upper median, in place: the caller handed the vector over
	const size_t mid = residuals.size()/2;
	std::nth_element(residuals.begin(), residuals.begin()+mid, residuals.end());
	return CLAMP(factor*residuals[mid], minHalfWidth, maxHalfWidth);
}
/*----------------------------------------------------------------*/


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
	// would throw; the guided path has no use for it anyway -- the ratio inside the band and, for a
	// winner alone in it, the reference outside are what keep a match honest here, not a train-side
	// collision
	ASSERT(!pairsMatcher.GetConfig().crossCheck);
	const uint32_t numDescribedA = imgA.NumDescribedKeypoints();
	const uint32_t numDescribedB = imgB.NumDescribedKeypoints();
	ASSERT(trackedB.size() == numDescribedA);
	ASSERT(trackStatus.size() == trackedB.size());

	matches.clear();
	if (numDescribedA == 0 || numDescribedB == 0)
		return 0;

	// 2D index over imgB's described keypoints, the only ones a match may name: a trainIdx reads
	// imgB.descriptors.row(), which exists for the described prefix alone. The octree returns
	// indices into the array it was built from, so a prefix-sized array keeps them keypoint indices.
	typedef CLISTDEF0(Point2f::EVec) Point2fs;
	Point2fs kptsB(numDescribedB);
	for (uint32_t j = 0; j < numDescribedB; ++j) {
		const cv::KeyPoint& keypoint = imgB.keypoints[j];
		kptsB[j] = Point2f::EVec(keypoint.pt.x, keypoint.pt.y);
	}
	typedef TOctree<Point2fs, float, 2> Octree2f;
	Octree2f octree(kptsB, [](Octree2f::IDX_TYPE n, Octree2f::Type r) { return n > 16 && r > 8.f; });

	// The region a query searches: the band along its epipolar line, centred on its prediction --
	// `length` either way along the line, `halfWidth` either way across it -- or, without a line to
	// follow, the disc of radius `length` around the prediction. The band never reaches outside that
	// disc (the half-width is held to the length), so the octree is asked for the disc and the band
	// is cut out of what it returns; this ONE predicate decides both what a candidate is and what
	// "outside" means, so no keypoint can ever be a candidate and the reference distance of the
	// same query at once.
	// The line comes from search.F (x_B^T F x_A = 0): its direction at the prediction is all the band
	// takes from the geometry, the position it sits at is the warp's, in imgB's own pixels -- so an
	// imprecise focal or principal point, or lens distortion the geometry does not model, turns the
	// band by a few degrees and moves it not at all. Near the epipole of imgB every line direction is
	// equally wrong, so a prediction within two lengths of it searches the disc.
	const float length = search.length;
	const float lengthSq = length*length;
	const float halfWidth = MINF(search.halfWidth, length);
	const bool bBand = search.F.has_value() && halfWidth > 0.f;
	Eigen::Matrix3d F;
	Eigen::Vector2d epipoleB(0, 0);
	bool bEpipoleFinite = false;
	if (bBand) {
		F = search.F.value();
		// the epipole of imgB is the left null vector of F (F^T e = 0); at infinity when the lines
		// are parallel, and then never near a prediction
		const Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU);
		const Eigen::Vector3d e = svd.matrixU().col(2);
		if (ABS(e.z()) > 1e-9*e.norm()) {
			epipoleB = Eigen::Vector2d(e.x()/e.z(), e.y()/e.z());
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
	// test exists to refuse. Keypoints outside the region take no part: the geometry says the match
	// is not there.
	// Both sides here are TRUE descriptor norms (DescriptorDistance recomputes them), so this test
	// enforces d0/d1 < matchRatio, while PairsMatcher::MatchFeatures applies the same constant to
	// FLANN's SQUARED L2 distances and so enforces d0/d1 < sqrt(matchRatio): at the same setting the
	// guided pass is the stricter of the two (0.8 here against 0.894 there). See
	// MatchConfig::matchRatio (PairsMatcher.h)
	const float matchRatio = pairsMatcher.GetConfig().matchRatio;
	enum : uint8_t { REFUSED, ACCEPTED, UNDECIDED };
	std::vector<uint8_t> verdicts(winners.size(), UNDECIDED);
	std::vector<size_t> lone; // winners with no rival in their region, decided by the outside reference
	FOREACH(q, winners) {
		const Winner& winner = winners[q];
		if (winner.rivalDistance >= 0.f)
			verdicts[q] = winner.distance < matchRatio*winner.rivalDistance ? ACCEPTED : REFUSED;
		else if (!search.loneOutsideReference)
			verdicts[q] = ACCEPTED; // alone in its region, and asked to stand
		else
			lone.push_back(q);
	}

	// Step 3: a winner alone in its region, when asked to, has to beat the closest described
	// keypoint of imgB OUTSIDE it by the same ratio -- what refuses a lone impostor where the true
	// keypoint was never detected. The thread's own descriptor matcher answers all of them in one
	// batched query over the whole described prefix of imgB, k = K_NN, exactly as
	// PairsMatcher::MatchFeatures runs its own (k = 2) one.
	if (!lone.empty()) {
		cv::Mat queryDescriptors((int)lone.size(), imgA.descriptors.cols, imgA.descriptors.type());
		FOREACH(k, lone)
			imgA.descriptors.row((int)winners[lone[k]].queryIdx).copyTo(queryDescriptors.row((int)k));
		std::vector<std::vector<cv::DMatch>> knnMatches;
		cv::DescriptorMatcher& matcher = pairsMatcher.GetMatcher(threadIdx);
		if (pairsMatcher.GetConfig().descriptorsAreBinary) {
			matcher.knnMatch(queryDescriptors, imgB.descriptors, knnMatches, K_NN);
		} else {
			cv::Mat queryDescriptorsF, trainDescriptorsF;
			queryDescriptors.convertTo(queryDescriptorsF, CV_32F);
			imgB.descriptors.convertTo(trainDescriptorsF, CV_32F);
			matcher.knnMatch(queryDescriptorsF, trainDescriptorsF, knnMatches, K_NN);
		}
		// one neighbour list per query is the matcher's contract, and knnMatch keeps it: with the
		// default compactResult=false it returns one list per query row, and the one path that
		// returns fewer (an empty train set) is excluded by the numDescribedB == 0 early return
		// above. Asserted rather than handled, so the return value below means only how many
		// matches this pass accepted
		ASSERT(knnMatches.size() == lone.size());
		FOREACH(k, lone) {
			const Winner& winner = winners[lone[k]];
			const std::vector<cv::DMatch>& neighbours = knnMatches[k];
			if (neighbours.empty()) {
				verdicts[lone[k]] = REFUSED; // no reference distance, so nothing this match could be shown to beat
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
					verdicts[lone[k]] = ACCEPTED;
					continue;
				}
				outsideDistance = DescriptorDistance(winner.queryIdx, (uint32_t)neighbours.back().trainIdx);
			}
			// strictly better: a rival exactly as close as the winner but outside the region is the
			// very thing this test exists to reject, and equal descriptors (the same feature
			// described twice, in two different places of imgB) make that tie a real case rather
			// than a floating-point curiosity
			verdicts[lone[k]] = winner.distance < matchRatio*outsideDistance ? ACCEPTED : REFUSED;
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

	// Build a 2D octree over keypoints2 for fast spatial neighbor queries around trackedPoints2.
	// Restricted to img2's described prefix, and so are every candidate scan and brute-force
	// fallback below: a candidate's trainIdx is used to read img2.descriptors.row(), which only
	// exists for a described keypoint. The octree returns indices into the array it was built
	// from, so a prefix-sized array keeps them keypoint indices.
	typedef CLISTDEF0(Point2f::EVec) Point2fs;
	Point2fs kpts2(numDescribed2);
	for (uint32_t i = 0; i < numDescribed2; ++i) {
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
			// the band left a single candidate, so there is no second best to run a ratio test
			// against and the band's own verdict stands
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
		for (uint32_t i = 0; i < numDescribed1; ++i) {
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
				// fallback: scan all described keypoints2 and use only epipolar constraint
				for (uint32_t j = 0; j < numDescribed2; ++j)
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
		std::vector<Eigen::Vector3d> bearings1(numDescribed1);
		std::vector<Eigen::Vector3d> bearings2(numDescribed2);
		for (uint32_t i = 0; i < numDescribed1; ++i)
			bearings1[i] = img1.pCamera->UnprojectNormalized(Cast<REAL>(img1.keypoints[i].pt));
		for (uint32_t i = 0; i < numDescribed2; ++i)
			bearings2[i] = img2.pCamera->UnprojectNormalized(Cast<REAL>(img2.keypoints[i].pt));

		for (uint32_t i = 0; i < numDescribed1; ++i) {
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
				// fallback: scan all described keypoints2 and use only epipolar constraint
				for (uint32_t j = 0; j < numDescribed2; ++j)
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
