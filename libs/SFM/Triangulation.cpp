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

	// Collect camera poses and 2D normalized points
	std::vector<PMatrix> projMatrices;
	std::vector<Point2> points2D;
	projMatrices.reserve(track.observations.size());
	points2D.reserve(track.observations.size());
	for (const Observation& obs : track.observations) {
		ASSERT(obs.imageID < images.size());
		const Image& img = images[obs.imageID];
		ASSERT(img.HasCamera() && obs.featureID < img.keypoints.size());
		// TriangulateDLT is pinhole-only: the linear system assumes the 2D point
		// lies on the z=1 normalized plane, which is front-hemisphere biased and
		// cannot represent back-hemisphere observations of a spherical camera.
		// Use TriangulateSkewLLS() for non-pinhole cameras.
		ASSERT(img.pCamera->GetType() == CameraType::PINHOLE);
		// Build projection matrix P = R*[I|-C]
		projMatrices.push_back(img.GetPfromRC());
		// Get the intrinsics normalized 2D point
		const cv::KeyPoint& kp = img.keypoints[obs.featureID];
		const Point3 ray = img.pCamera->Unproject(Cast<REAL>(kp.pt));
		points2D.emplace_back(ray.x, ray.y);
	}

	// Triangulate using linear multi-view DLT (solve A*X=0)
	// Build matrix A for homogeneous linear system
	{
		const int m = static_cast<int>(projMatrices.size());
		cv::Mat A(2 * m, 4, CV_64F);
		for (int i = 0; i < m; ++i) {
			const PMatrix& P = projMatrices[i];
			const Point2& pt = points2D[i];
			// x * P.row(2) - P.row(0)
			for (int j = 0; j < 4; ++j)
				A.at<double>(2*i+0, j) = pt.x * P(2, j) - P(0, j);
			// y * P.row(2) - P.row(1)
			for (int j = 0; j < 4; ++j)
				A.at<double>(2*i+1, j) = pt.y * P(2, j) - P(1, j);
		}
		// Solve using SVD: A*X = 0
		cv::Mat w, u, vt;
		cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);
		// Solution is last column of V (last row of Vt)
		const cv::Vec4d X_h = vt.row(3);
		const double w_val = X_h(3);
		if (ABS(w_val) < 1e-12)
			return 0;
		track.position.x = static_cast<REAL>(X_h(0) / w_val);
		track.position.y = static_cast<REAL>(X_h(1) / w_val);
		track.position.z = static_cast<REAL>(X_h(2) / w_val);
	}

	// Check constraints and mark inliers
	track.numInliers = 0;
	CLISTDEF0IDX(uint32_t, uint32_t) mapIndices(track.observations.size());
	FOREACH(obsIdx, track.observations) {
		const Observation& obs = track.observations[obsIdx];
		const Image& img = images[obs.imageID];
		mapIndices[obsIdx] = obsIdx;
		// Check reprojection error
		const auto [proj, valid] = img.ProjectPoint(track.position);
		if (!valid)
			continue;
		const cv::KeyPoint& kp = img.keypoints[obs.featureID];
		const float error = norm(Cast<float>(proj) - kp.pt);
		if (error > reprojThreshold)
			continue;
		// This is an inlier observation, move it to the front
		if (track.numInliers < obsIdx) {
			std::swap(track.observations[track.numInliers], track.observations[obsIdx]);
			std::swap(mapIndices[track.numInliers], mapIndices[obsIdx]);
		}
		++track.numInliers;
	}
	if (track.numInliers < minInliers)
		return 0;
	// Check minimum triangulation angle
	const float minAngle = R2D(track.ComputeMinAngleBetweenRays(images));
	if (minAngle < minAngleThreshold)
		return 0;

	// Refine with all inliers
	if (track.numInliers != track.observations.size()) {
		const int m = static_cast<int>(track.numInliers);
		cv::Mat A(2*m, 4, CV_64F);
		int k = 0;
		for (int _i = 0; _i < m; ++_i) {
			const int i = static_cast<int>(mapIndices[_i]);
			const PMatrix& P = projMatrices[i];
			const Point2& pt = points2D[i];
			for (int j = 0; j < 4; ++j)
				A.at<double>(2*k+0, j) = pt.x * P(2, j) - P(0, j);
			for (int j = 0; j < 4; ++j)
				A.at<double>(2*k+1, j) = pt.y * P(2, j) - P(1, j);
			++k;
		}
		cv::Mat w, u, vt;
		cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);
		const cv::Vec4d X_h = vt.row(3);
		const double w_val = X_h(3);
		if (ABS(w_val) > 1e-12) {
			track.position.x = static_cast<REAL>(X_h(0) / w_val);
			track.position.y = static_cast<REAL>(X_h(1) / w_val);
			track.position.z = static_cast<REAL>(X_h(2) / w_val);
		}
	}
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

	// Collect normalized directions in camera space and R, t from each camera.
	// Invariant throughout: cams[j] corresponds to track.observations[j].
	// We maintain this by always performing the same swap on both arrays.
	struct CameraData {
		Matrix3x3::EMat DR; // D_cross * R
		Point3::EVec Dt; // -D_cross * t
	};
	CLISTDEF0IDX(CameraData, uint32_t) cams(0, track.observations.size());
	FOREACH(obsIdx, track.observations) {
		const Observation& obs = track.observations[obsIdx];
		ASSERT(obs.imageID < images.size());
		const Image& img = images[obs.imageID];
		ASSERT(img.HasCamera() && obs.featureID < img.keypoints.size());
		if (!img.IsValid())
			continue;
		// Ray direction in camera coordinates (unproject)
		const cv::KeyPoint& kp = img.keypoints[obs.featureID];
		const Point3 dir = img.pCamera->UnprojectNormalized(Cast<REAL>(kp.pt));
		// Build DR and Dt
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

	// Build the system A * Pw = b (2*N x 3)
	Eigen::MatrixXd A(2*cams.size(), 3);
	Eigen::VectorXd bvec(2*cams.size());
	FOREACH(i, cams) {
		const CameraData& cam = cams[i];
		// Use the first two independent rows
		A.row(2*i+0) = cam.DR.row(0);
		bvec(2*i+0) = cam.Dt(0);
		A.row(2*i+1) = cam.DR.row(1);
		bvec(2*i+1) = cam.Dt(1);
	}
	// Solve least-squares with SVD
	track.position = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bvec);
	ASSERT(ISFINITE(track.position));

	// Validate by cheirality and reprojection,
	// moving inliers to the front of both cams and observations
	track.numInliers = 0;
	FOREACH(obsIdx, cams) {
		const Observation& obs = track.observations[obsIdx];
		const Image& img = images[obs.imageID];
		const auto [proj, valid] = img.ProjectPoint(track.position);
		if (!valid)
			continue;
		const cv::KeyPoint& kp = img.keypoints[obs.featureID];
		const float error = norm(Cast<float>(proj) - kp.pt);
		if (error > reprojThreshold)
			continue;
		// This is an inlier observation, move it to the front
		if (track.numInliers < obsIdx) {
			std::swap(track.observations[track.numInliers], track.observations[obsIdx]);
			std::swap(cams[track.numInliers], cams[obsIdx]);
		}
		++track.numInliers;
	}
	if (track.numInliers < minInliers)
		return 0;
	// Minimum triangulation angle
	const float minAngle = R2D(track.ComputeMinAngleBetweenRays(images));
	if (minAngle < minAngleThreshold)
		return 0;

	// Refine using only inliers (now at positions 0..numInliers-1 of both arrays)
	if (track.numInliers < cams.size()) {
		Eigen::MatrixXd A2(2*track.numInliers, 3);
		Eigen::VectorXd b2(2*track.numInliers);
		for (uint32_t k = 0; k < (uint32_t)track.numInliers; ++k) {
			const CameraData& cam = cams[k];
			A2.row(2 * k + 0) = cam.DR.row(0);
			b2(2 * k + 0) = cam.Dt(0);
			A2.row(2 * k + 1) = cam.DR.row(1);
			b2(2 * k + 1) = cam.Dt(1);
		}
		track.position = A2.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b2);
		ASSERT(ISFINITE(track.position));
	}
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

	// Triangulate each track
	constexpr unsigned minInliers = 2;
	unsigned numInliers = 0, numInliersPrev = 0, numInvalids = 0, numInliersObservations = 0;
	#ifdef _USE_OPENMP
	#pragma omp parallel for reduction(+:numInliers,numInliersPrev,numInliersObservations) schedule(dynamic)
	#endif
	for (int_t i = 0; i < (int_t)scene.tracks.size(); ++i) {
		Track& track = scene.tracks[i];
		ASSERT(track.IsValid());
		if (outliersOnly && track.IsInlier()) {
			++numInliersPrev;
			continue;
		}
		unsigned nObservations = 0;
		for (const Observation& obs : track.observations)
			if (scene.images[obs.imageID].IsValid())
				++nObservations;
		if (nObservations < minInliers) {
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
		numInliers, scene.tracks.size()-numInliersPrev-numInliers-numInvalids, numInvalids, scene.status.nTracks, scene.tracks.size(), (float)numInliersObservations/numInliers, TD_TIMER_GET_FMT().c_str());
	return numInliers;
}
/*----------------------------------------------------------------*/
