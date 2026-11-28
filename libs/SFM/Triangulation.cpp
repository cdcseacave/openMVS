/*
 * Triangulation.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "Triangulation.h"
#include "Scene.h"

using namespace SFM;

// S T R U C T S ///////////////////////////////////////////////////

unsigned SFM::TriangulateDLT(
	Track& track,
	const ImageArr& images,
	float reprojThreshold,
	float minAngleThreshold,
	unsigned minInliers)
{
	ASSERT(track.IsValid());
	track.numInliers = 0; // none until the checks below count them, so every return 0 leaves none

	// Collect camera poses and 2D normalized points
	std::vector<PMatrix> projMatrices;
	std::vector<Point2> points2D;
	projMatrices.reserve(track.observations.size());
	points2D.reserve(track.observations.size());
	for (const Observation& obs : track.observations) {
		ASSERT(obs.imageID < images.size());
		const Image& img = images[obs.imageID];
		ASSERT(img.HasCamera() && obs.featureID < img.keypoints.size());
		ASSERT(img.pCamera->GetType() == CameraType::PINHOLE);
		projMatrices.push_back(img.GetPfromRC());
		const cv::KeyPoint& kp = img.keypoints[obs.featureID];
		const Point3 ray = img.pCamera->Unproject(Cast<REAL>(kp.pt));
		points2D.emplace_back(ray.x, ray.y);
	}

	const auto SolveDLT = [&](const uint32_t* indices, int m) -> bool {
		cv::Mat A(2 * m, 4, CV_64F);
		for (int k = 0; k < m; ++k) {
			const int i = indices ? (int)indices[k] : k;
			const PMatrix& P = projMatrices[i];
			const Point2& pt = points2D[i];
			for (int j = 0; j < 4; ++j) {
				A.at<double>(2*k+0, j) = pt.x * P(2, j) - P(0, j);
				A.at<double>(2*k+1, j) = pt.y * P(2, j) - P(1, j);
			}
		}
		cv::Mat w, u, vt;
		cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);
		const cv::Vec4d X_h = vt.row(3);
		const double w_val = X_h(3);
		if (ABS(w_val) < 1e-12)
			return false;
		track.position.x = static_cast<REAL>(X_h(0) / w_val);
		track.position.y = static_cast<REAL>(X_h(1) / w_val);
		track.position.z = static_cast<REAL>(X_h(2) / w_val);
		return true;
	};

	if (!SolveDLT(nullptr, static_cast<int>(projMatrices.size())))
		return 0;

	// Check constraints and mark inliers
	CLISTDEF0IDX(uint32_t, uint32_t) mapIndices(track.observations.size());
	FOREACH(obsIdx, track.observations) {
		const Observation& obs = track.observations[obsIdx];
		const Image& img = images[obs.imageID];
		mapIndices[obsIdx] = obsIdx;
		const auto [error, valid] = ComputeReprojectionErrorPixels(img, track.position, obs.featureID);
		if (!valid || error > reprojThreshold)
			continue;
		if (track.numInliers < obsIdx) {
			std::swap(track.observations[track.numInliers], track.observations[obsIdx]);
			std::swap(mapIndices[track.numInliers], mapIndices[obsIdx]);
		}
		++track.numInliers;
	}
	if (track.numInliers < minInliers ||
		R2D(track.ComputeMaxAngleBetweenRays(images)) < minAngleThreshold) {
		track.numInliers = 0;
		return 0;
	}

	// Refine with inliers only
	if (track.numInliers != track.observations.size())
		SolveDLT(mapIndices.data(), static_cast<int>(track.numInliers));
	return track.numInliers;
}

unsigned SFM::TriangulateSkewLLS(
	Track& track,
	const ImageArr& images,
	float reprojThreshold,
	float minAngleThreshold,
	unsigned minInliers)
{
	ASSERT(track.IsValid());
	track.numInliers = 0;

	// Collect normalized directions in camera space and R, t from each camera.
	// Invariant: cams[j] corresponds to track.observations[j].
	struct CameraData {
		Matrix3x3::EMat DR; // [d]_x * R (all three rows: rank 2, isotropic in ray angle)
		Point3::EVec Dt;    // -[d]_x * t
	};
	CLISTDEF0IDX(CameraData, uint32_t) cams(0, track.observations.size());
	FOREACH(obsIdx, track.observations) {
		const Observation& obs = track.observations[obsIdx];
		ASSERT(obs.imageID < images.size());
		const Image& img = images[obs.imageID];
		ASSERT(img.HasCamera() && obs.featureID < img.keypoints.size());
		if (!img.IsValid())
			continue;
		const cv::KeyPoint& kp = img.keypoints[obs.featureID];
		const Point3 dir = img.pCamera->UnprojectNormalized(Cast<REAL>(kp.pt));
		ASSERT(ISEQUAL(norm(dir), REAL(1)));
		const Matrix3x3 Dcross(
			0, -dir.z, dir.y,
			dir.z, 0, -dir.x,
			-dir.y, dir.x, 0);
		const Matrix3x3 DR = Dcross * img.R;
		const Point3 Dt = -Dcross * img.GetT();
		if (cams.size() < obsIdx)
			std::swap(track.observations[cams.size()], track.observations[obsIdx]);
		cams.emplace_back(DR, Dt);
	}
	if (cams.size() < minInliers)
		return 0;

	const auto SolveSkewLLS = [&](uint32_t count) {
		Eigen::MatrixXd A(3 * count, 3);
		Eigen::VectorXd bvec(3 * count);
		for (uint32_t i = 0; i < count; ++i) {
			A.middleRows<3>(3 * i) = cams[i].DR;
			bvec.segment<3>(3 * i) = cams[i].Dt;
		}
		track.position = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bvec);
		ASSERT(ISFINITE(track.position));
	};

	SolveSkewLLS(cams.size());

	// Validate by cheirality and reprojection, moving inliers to the front
	FOREACH(obsIdx, cams) {
		const Observation& obs = track.observations[obsIdx];
		const Image& img = images[obs.imageID];
		const auto [error, valid] = ComputeReprojectionErrorPixels(img, track.position, obs.featureID);
		if (!valid || error > reprojThreshold)
			continue;
		if (track.numInliers < obsIdx) {
			std::swap(track.observations[track.numInliers], track.observations[obsIdx]);
			std::swap(cams[track.numInliers], cams[obsIdx]);
		}
		++track.numInliers;
	}
	if (track.numInliers < minInliers ||
		R2D(track.ComputeMaxAngleBetweenRays(images)) < minAngleThreshold) {
		track.numInliers = 0;
		return 0;
	}

	// Refine using only inliers (at positions 0..numInliers-1 of both arrays)
	if (track.numInliers < cams.size())
		SolveSkewLLS(track.numInliers);
	return track.numInliers;
}

unsigned SFM::TriangulateTracks(
	Scene& scene,
	bool outliersOnly,
	float reprojThreshold,
	float minAngleThreshold)
{
	TD_TIMER_STARTD();
	ASSERT(!scene.tracks.empty());

	constexpr unsigned minInliers = Track::MIN_VIEWS;
	unsigned numInliers = 0, numInliersPrev = 0, numInvalids = 0, numInliersObservations = 0;
	#ifdef _USE_OPENMP
	#pragma omp parallel for reduction(+:numInliers,numInliersPrev,numInvalids,numInliersObservations) schedule(dynamic)
	#endif
	for (int_t i = 0; i < (int_t)scene.tracks.size(); ++i) {
		Track& track = scene.tracks[i];
		ASSERT(track.IsValid());
		if (outliersOnly && track.IsInlier()) {
			++numInliersPrev;
			continue;
		}
		if (track.GetNumPosedObservations(scene.images) < minInliers) {
			track.numInliers = 0;
			++numInvalids;
			continue;
		}
		unsigned nInliers = TriangulateSkewLLS(track, scene.images, reprojThreshold, minAngleThreshold, minInliers);
		if (nInliers < minInliers)
			continue;
		numInliersObservations += nInliers;
		++numInliers;
	}
	scene.status.nTracks = numInliers + numInliersPrev;
	DEBUG("Triangulated %u tracks successfully (%u failed, %u invalid), total inliers %u from %u tracks, %.2f views/track (%s)",
		numInliers, scene.tracks.size()-numInliersPrev-numInliers-numInvalids, numInvalids, scene.status.nTracks, scene.tracks.size(), (float)numInliersObservations/MAXF(numInliers, 1u), TD_TIMER_GET_FMT().c_str());
	return numInliers;
}
/*----------------------------------------------------------------*/
