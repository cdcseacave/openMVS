/*
 * PoseLibBearing.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_POSELIB_BEARING_H_
#define _SFM_POSELIB_BEARING_H_

#include <PoseLib/poselib.h>

namespace SFM {

inline double PoseLibMaxErrorFromAngle(double angle) {
	return 2.0 * std::sin(0.5 * angle);
}

poselib::RansacStats EstimateAbsolutePoseBearings(
	const std::vector<poselib::Point3D>& bearings,
	const std::vector<poselib::Point3D>& points3D,
	const poselib::AbsolutePoseOptions& opt,
	poselib::CameraPose* pose,
	std::vector<char>* inliers);

poselib::RansacStats EstimateRelativePoseBearings(
	const std::vector<poselib::Point3D>& bearings1,
	const std::vector<poselib::Point3D>& bearings2,
	const poselib::RelativePoseOptions& opt,
	poselib::CameraPose* relativePose,
	std::vector<char>* inliers);

} // namespace SFM

#endif // _SFM_POSELIB_BEARING_H_
