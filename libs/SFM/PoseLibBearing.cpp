/*
 * PoseLibBearing.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "PoseLibBearing.h"

#include <PoseLib/robust/ransac_impl.h>
#include <PoseLib/robust/sampling.h>
#include <PoseLib/solvers/p3p.h>
#include <PoseLib/solvers/relpose_5pt.h>

using namespace SFM;

namespace {

double ScoreAbsoluteBearing(
	const poselib::CameraPose& pose,
	const std::vector<poselib::Point3D>& bearings,
	const std::vector<poselib::Point3D>& points3D,
	double sqThreshold,
	std::vector<char>* inliers,
	size_t* inlierCount)
{
	double score(0.0);
	size_t count(0);
	if (inliers)
		inliers->assign(bearings.size(), 0);
	for (size_t i = 0; i < bearings.size(); ++i) {
		const poselib::Point3D Xcam(pose.apply(points3D[i]));
		const double norm(Xcam.norm());
		double sqError(sqThreshold);
		if (norm > 0) {
			const poselib::Point3D projected(Xcam / norm);
			sqError = (bearings[i] - projected).squaredNorm();
		}
		if (sqError < sqThreshold) {
			++count;
			if (inliers)
				(*inliers)[i] = 1;
		}
		score += std::min(sqError, sqThreshold);
	}
	if (inlierCount)
		*inlierCount = count;
	return score;
}

bool HasPositiveRelativeDepth(
	const poselib::CameraPose& pose,
	const poselib::Point3D& bearing1,
	const poselib::Point3D& bearing2)
{
	Eigen::Matrix<double,3,2> A;
	A.col(0) = pose.rotate(bearing1);
	A.col(1) = -bearing2;
	const Eigen::Vector2d depth(A.colPivHouseholderQr().solve(-pose.t));
	return depth(0) > 0 && depth(1) > 0;
}

double BearingEpipolarError(
	const poselib::CameraPose& pose,
	const poselib::Point3D& bearing1,
	const poselib::Point3D& bearing2)
{
	const poselib::Point3D epipolarNormal(pose.t.cross(pose.rotate(bearing1)));
	const double normalNorm(epipolarNormal.norm());
	if (normalNorm <= 0)
		return 1.0;
	return std::abs(bearing2.dot(epipolarNormal / normalNorm));
}

double ScoreRelativeBearing(
	const poselib::CameraPose& pose,
	const std::vector<poselib::Point3D>& bearings1,
	const std::vector<poselib::Point3D>& bearings2,
	double sqThreshold,
	std::vector<char>* inliers,
	size_t* inlierCount)
{
	double score(0.0);
	size_t count(0);
	if (inliers)
		inliers->assign(bearings1.size(), 0);
	for (size_t i = 0; i < bearings1.size(); ++i) {
		double sqError(sqThreshold);
		if (HasPositiveRelativeDepth(pose, bearings1[i], bearings2[i])) {
			const double error(BearingEpipolarError(pose, bearings1[i], bearings2[i]));
			sqError = error * error;
		}
		if (sqError < sqThreshold) {
			++count;
			if (inliers)
				(*inliers)[i] = 1;
		}
		score += std::min(sqError, sqThreshold);
	}
	if (inlierCount)
		*inlierCount = count;
	return score;
}

class AbsoluteBearingEstimator {
public:
	AbsoluteBearingEstimator(
		const poselib::AbsolutePoseOptions& _opt,
		const std::vector<poselib::Point3D>& _bearings,
		const std::vector<poselib::Point3D>& _points3D)
		: num_data(_bearings.size())
		, opt(_opt)
		, bearings(_bearings)
		, points3D(_points3D)
		, sampler(num_data, sample_sz, opt.ransac)
	{
		sample.resize(sample_sz);
		bearingSample.resize(sample_sz);
		pointSample.resize(sample_sz);
	}

	void generate_models(std::vector<poselib::CameraPose>* models) {
		models->clear();
		sampler.generate_sample(&sample);
		for (size_t i = 0; i < sample_sz; ++i) {
			bearingSample[i] = bearings[sample[i]];
			pointSample[i] = points3D[sample[i]];
		}
		poselib::p3p(bearingSample, pointSample, models);
	}

	double score_model(const poselib::CameraPose& pose, size_t* inlierCount) const {
		return ScoreAbsoluteBearing(pose, bearings, points3D, opt.max_error * opt.max_error, nullptr, inlierCount);
	}

	void refine_model(poselib::CameraPose*) const {}

	const size_t sample_sz = 3;
	const size_t num_data;

private:
	const poselib::AbsolutePoseOptions& opt;
	const std::vector<poselib::Point3D>& bearings;
	const std::vector<poselib::Point3D>& points3D;
	poselib::RandomSampler sampler;
	std::vector<size_t> sample;
	std::vector<poselib::Point3D> bearingSample;
	std::vector<poselib::Point3D> pointSample;
};

class RelativeBearingEstimator {
public:
	RelativeBearingEstimator(
		const poselib::RelativePoseOptions& _opt,
		const std::vector<poselib::Point3D>& _bearings1,
		const std::vector<poselib::Point3D>& _bearings2)
		: num_data(_bearings1.size())
		, opt(_opt)
		, bearings1(_bearings1)
		, bearings2(_bearings2)
		, sampler(num_data, sample_sz, opt.ransac)
	{
		sample.resize(sample_sz);
		bearingSample1.resize(sample_sz);
		bearingSample2.resize(sample_sz);
	}

	void generate_models(std::vector<poselib::CameraPose>* models) {
		models->clear();
		sampler.generate_sample(&sample);
		for (size_t i = 0; i < sample_sz; ++i) {
			bearingSample1[i] = bearings1[sample[i]];
			bearingSample2[i] = bearings2[sample[i]];
		}
		poselib::relpose_5pt(bearingSample1, bearingSample2, models);
	}

	double score_model(const poselib::CameraPose& pose, size_t* inlierCount) const {
		return ScoreRelativeBearing(pose, bearings1, bearings2, opt.max_error * opt.max_error, nullptr, inlierCount);
	}

	void refine_model(poselib::CameraPose*) const {}

	const size_t sample_sz = 5;
	const size_t num_data;

private:
	const poselib::RelativePoseOptions& opt;
	const std::vector<poselib::Point3D>& bearings1;
	const std::vector<poselib::Point3D>& bearings2;
	poselib::RandomSampler sampler;
	std::vector<size_t> sample;
	std::vector<poselib::Point3D> bearingSample1;
	std::vector<poselib::Point3D> bearingSample2;
};

} // namespace

poselib::RansacStats SFM::EstimateAbsolutePoseBearings(
	const std::vector<poselib::Point3D>& bearings,
	const std::vector<poselib::Point3D>& points3D,
	const poselib::AbsolutePoseOptions& opt,
	poselib::CameraPose* pose,
	std::vector<char>* inliers)
{
	if (bearings.size() < 3 || points3D.size() != bearings.size()) {
		if (inliers)
			inliers->assign(bearings.size(), 0);
		return poselib::RansacStats();
	}
	AbsoluteBearingEstimator estimator(opt, bearings, points3D);
	poselib::RansacStats stats(poselib::ransac<AbsoluteBearingEstimator>(estimator, opt.ransac, pose));
	ScoreAbsoluteBearing(*pose, bearings, points3D, opt.max_error * opt.max_error, inliers, &stats.num_inliers);
	stats.inlier_ratio = bearings.empty() ? 0.0 : double(stats.num_inliers) / double(bearings.size());
	return stats;
}

poselib::RansacStats SFM::EstimateRelativePoseBearings(
	const std::vector<poselib::Point3D>& bearings1,
	const std::vector<poselib::Point3D>& bearings2,
	const poselib::RelativePoseOptions& opt,
	poselib::CameraPose* relativePose,
	std::vector<char>* inliers)
{
	if (bearings1.size() < 5 || bearings2.size() != bearings1.size()) {
		if (inliers)
			inliers->assign(bearings1.size(), 0);
		return poselib::RansacStats();
	}
	RelativeBearingEstimator estimator(opt, bearings1, bearings2);
	poselib::RansacStats stats(poselib::ransac<RelativeBearingEstimator>(estimator, opt.ransac, relativePose));
	ScoreRelativeBearing(*relativePose, bearings1, bearings2, opt.max_error * opt.max_error, inliers, &stats.num_inliers);
	stats.inlier_ratio = bearings1.empty() ? 0.0 : double(stats.num_inliers) / double(bearings1.size());
	return stats;
}
