/*
 * BundleAdjustmentCostFunctions.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_BUNDLEADJUSTMENT_COSTFUNCTIONS_H_
#define _SFM_BUNDLEADJUSTMENT_COSTFUNCTIONS_H_


// I N C L U D E S /////////////////////////////////////////////////

#include <Eigen/Core>
#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#pragma pop_macro("VERBOSE")
#pragma pop_macro("LOG")


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Helper: Transform world point to camera space using quaternion pose
// pose[7] = { qw, qx, qy, qz, Cx, Cy, Cz }
// Returns camera-space point (X_cam, Y_cam, Z_cam)
template <typename T>
inline void CameraSpaceTransform(
	const T* const pose,    // quaternion[4] + center[3]
	const T* const point,   // world point[3]
	T* p_cam)               // output: camera-space point[3]
{
	const T* quat = pose;      // [qw, qx, qy, qz]
	const T* center = pose + 4; // [Cx, Cy, Cz]

	// Translate: p_world - C
	T p[3];
	p[0] = point[0] - center[0];
	p[1] = point[1] - center[1];
	p[2] = point[2] - center[2];

	// Rotate: R * (p_world - C)
	// Use UnitQuaternionRotatePoint (faster, no normalization) since QuaternionManifold guarantees unit length
	ceres::UnitQuaternionRotatePoint(quat, p, p_cam);
}

// Pinhole camera projection with distortion
// intrinsics: [fx, fy/fx, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6] (12 params)
// Returns projected 2D pixel coordinates
template <typename T>
inline void ProjectPinhole(
	const T* const intrinsics,  // camera intrinsics
	const T* p_cam,             // camera-space point[3]
	T* projected)               // output: pixel[2]
{
	// Perspective division
	const T inv_z = 1.0 / p_cam[2];
	const T x = p_cam[0] * inv_z;
	const T y = p_cam[1] * inv_z;

	// Extract intrinsics
	const T& fx = intrinsics[0];
	const T fy = fx * intrinsics[1]; // fy = aspect_ratio * fx
	const T& cx = intrinsics[2];
	const T& cy = intrinsics[3];
	const T& k1 = intrinsics[4];
	const T& k2 = intrinsics[5];
	const T& k3 = intrinsics[6];
	const T& p1 = intrinsics[7];
	const T& p2 = intrinsics[8];
	const T& k4 = intrinsics[9];
	const T& k5 = intrinsics[10];
	const T& k6 = intrinsics[11];

	// Radial distortion
	const T r2 = x*x + y*y;
	const T r4 = r2*r2;
	const T r6 = r4*r2;

	// Rational distortion model
	const T radial_numerator = 1.0 + k1*r2 + k2*r4 + k3*r6;
	const T radial_denominator = 1.0 + k4*r2 + k5*r4 + k6*r6;
	const T radial = radial_numerator / radial_denominator;

	// Tangential distortion
	const T dx_tangential = 2.0*p1*x*y + p2*(r2 + 2.0*x*x);
	const T dy_tangential = p1*(r2 + 2.0*y*y) + 2.0*p2*x*y;

	// Apply distortion
	const T xd = x * radial + dx_tangential;
	const T yd = y * radial + dy_tangential;

	// Apply intrinsic matrix
	projected[0] = fx * xd + cx;
	projected[1] = fy * yd + cy;
}

// Spherical camera projection (equirectangular)
// No intrinsics needed, only image size (stored in camera, not optimized)
template <typename T>
inline void ProjectSpherical(
	int width, int height,      // image dimensions
	const T* p_cam,             // camera-space point[3]
	T* projected)               // output: pixel[2]
{
	// Convert to spherical coordinates
	const T longitude = ceres::atan2(p_cam[0], p_cam[2]);
	const T r_xz = ceres::sqrt(p_cam[0]*p_cam[0] + p_cam[2]*p_cam[2]);
	const T latitude = ceres::atan2(-p_cam[1], r_xz);

	// Map to image coordinates
	projected[0] = T(width) * (0.5 + longitude / (2.0 * M_PI));
	projected[1] = T(height) * (0.5 - latitude / M_PI);
}

// Reprojection error cost functor for pinhole cameras
struct PinholeReprojectionError {
	PinholeReprojectionError(double observed_x, double observed_y)
		: observed_x_(observed_x), observed_y_(observed_y) {}

	template <typename T>
	bool operator()(
		const T* const pose,        // 7 params: quat[4] + center[3]
		const T* const intrinsics,  // 12 params: fx,fy/fx,cx,cy,k1-k6,p1,p2
		const T* const point,       // 3 params: X, Y, Z
		T* residuals) const
	{
		// Transform to camera space
		T p_cam[3];
		CameraSpaceTransform(pose, point, p_cam);

		// Soft handling for points behind or very close to camera
		// Instead of returning false (which causes "Step failed to evaluate" warnings),
		// use a large penalty residual to discourage this configuration.
		if (p_cam[2] < T(1e-8)) {
			const T penalty = T(2.0); // large penalty
			residuals[0] = penalty;
			residuals[1] = penalty;
			return true;
		}

		// Project to image
		T projected[2];
		ProjectPinhole(intrinsics, p_cam, projected);
		// Compute residuals
		residuals[0] = projected[0] - observed_x_;
		residuals[1] = projected[1] - observed_y_;
		return true;
	}

	// Factory for pinhole cameras with full intrinsics
	static ceres::CostFunction* Create(double observed_x, double observed_y) {
		return new ceres::AutoDiffCostFunction<PinholeReprojectionError, 2, 7, 12, 3>(
			new PinholeReprojectionError(observed_x, observed_y));
	}

private:
	const double observed_x_, observed_y_;
};



// Analytic version of PinholeReprojectionError with explicit Jacobians
// Parameter blocks: pose[7], intrinsics[12], point[3]
// Residual: 2D (projected - observed)
struct PinholeReprojectionErrorAnalytic : public ceres::SizedCostFunction<2, 7, 12, 3> {
	PinholeReprojectionErrorAnalytic(double observed_x, double observed_y)
		: observed_x_(observed_x), observed_y_(observed_y) {}

	virtual bool Evaluate(double const* const* parameters,
	                      double* residuals,
	                      double** jacobians) const override {
		// Map parameters
		const double* pose = parameters[0];       // [qw, qx, qy, qz, Cx, Cy, Cz]
		const double* intrinsics = parameters[1]; // [fx, fy/fx, cx, cy, k1-k6, p1, p2]
		const double* point = parameters[2];      // [X, Y, Z]

		const double* quat = pose;       // [qw, qx, qy, qz]
		const double* center = pose + 4; // [Cx, Cy, Cz]

		// Transform to camera space: p_cam = R * (p_world - C)
		double p[3];
		p[0] = point[0] - center[0];
		p[1] = point[1] - center[1];
		p[2] = point[2] - center[2];

		double p_cam[3];
		ceres::UnitQuaternionRotatePoint(quat, p, p_cam);

		// Check depth
		if (p_cam[2] < 1e-8) {
			residuals[0] = 2.0;
			residuals[1] = 2.0;
			// Zero out Jacobians if requested
			if (jacobians) {
				if (jacobians[0]) std::memset(jacobians[0], 0, 2 * 7 * sizeof(double));
				if (jacobians[1]) std::memset(jacobians[1], 0, 2 * 12 * sizeof(double));
				if (jacobians[2]) std::memset(jacobians[2], 0, 2 * 3 * sizeof(double));
			}
			return true;
		}

		// Compute forward projection and distortion
		double projected[2];
		ProjectPinhole(intrinsics, p_cam, projected);

		// Compute residuals
		residuals[0] = projected[0] - observed_x_;
		residuals[1] = projected[1] - observed_y_;

		// Compute Jacobians if requested
		if (jacobians) {
			// Pre-calculate partial derivatives using chain rule
			// 1. P_cam = R(q)*(P_world - C)
			// J_Pcam_q (3x4)
			const double qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
			const double dx = p[0], dy = p[1], dz = p[2];
			// dx = p[0], dy = p[1], dz = p[2]
			Eigen::Matrix<double, 3, 4, Eigen::RowMajor> J_Pcam_q;
			J_Pcam_q(0, 0) = 2*dx*qw - 2*dy*qz + 2*dz*qy;
			J_Pcam_q(0, 1) = 2*dx*qx + 2*dy*qy + 2*dz*qz;
			J_Pcam_q(0, 2) = -2*dx*qy + 2*dy*qx + 2*dz*qw;
			J_Pcam_q(0, 3) = -2*dx*qz - 2*dy*qw + 2*dz*qx;
			J_Pcam_q(1, 0) = 2*dx*qz + 2*dy*qw - 2*dz*qx;
			J_Pcam_q(1, 1) = 2*dx*qy - 2*dy*qx - 2*dz*qw;
			J_Pcam_q(1, 2) = 2*dx*qx + 2*dy*qy + 2*dz*qz;
			J_Pcam_q(1, 3) = 2*dx*qw - 2*dy*qz + 2*dz*qy;
			J_Pcam_q(2, 0) = -2*dx*qy + 2*dy*qx + 2*dz*qw;
			J_Pcam_q(2, 1) = 2*dx*qz + 2*dy*qw - 2*dz*qx;
			J_Pcam_q(2, 2) = -2*dx*qw + 2*dy*qz - 2*dz*qy;
			J_Pcam_q(2, 3) = 2*dx*qx + 2*dy*qy + 2*dz*qz;
			// R matrix (3x3)
			Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R;
			R(0, 0) = 1 - 2*qy*qy - 2*qz*qz;
			R(0, 1) = 2*qx*qy - 2*qw*qz;
			R(0, 2) = 2*qx*qz + 2*qw*qy;
			R(1, 0) = 2*qx*qy + 2*qw*qz;
			R(1, 1) = 1 - 2*qx*qx - 2*qz*qz;
			R(1, 2) = 2*qy*qz - 2*qw*qx;
			R(2, 0) = 2*qx*qz - 2*qw*qy;
			R(2, 1) = 2*qy*qz + 2*qw*qx;
			R(2, 2) = 1 - 2*qx*qx - 2*qy*qy;
			// 2. Projection un = x/z, vn = y/z
			// J_norm_Pcam (2x3)
			const double inv_z = 1.0 / p_cam[2];
			const double inv_z2 = inv_z * inv_z;
			const double un = p_cam[0] * inv_z;
			const double vn = p_cam[1] * inv_z;
			Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_norm_Pcam;
			J_norm_Pcam(0, 0) = inv_z;
			J_norm_Pcam(0, 1) = 0;
			J_norm_Pcam(0, 2) = -p_cam[0] * inv_z2;
			J_norm_Pcam(1, 0) = 0;
			J_norm_Pcam(1, 1) = inv_z;
			J_norm_Pcam(1, 2) = -p_cam[1] * inv_z2;
			// 3. Distortion
			const double fx = intrinsics[0];
			const double fy = fx * intrinsics[1];
			const double k1 = intrinsics[4], k2 = intrinsics[5], k3 = intrinsics[6];
			const double p1 = intrinsics[7], p2 = intrinsics[8];
			const double k4 = intrinsics[9], k5 = intrinsics[10], k6 = intrinsics[11];
			const double r2 = un*un + vn*vn;
			const double r4 = r2*r2;
			const double r6 = r4*r2;
			const double num = 1 + k1*r2 + k2*r4 + k3*r6;
			const double den = 1 + k4*r2 + k5*r4 + k6*r6;
			const double inv_den = 1.0 / den;
			const double radial = num * inv_den;
			// Derivatives of radial distortion part w.r.t un, vn
			// D(radial)/Dr2 = ((k1 + 2*k2*r2 + 3*k3*r4)*D - N*(k4 + 2*k5*r2 + 3*k6*r4)) / D^2
			const double dnum_dr2 = k1 + 2*k2*r2 + 3*k3*r4;
			const double dden_dr2 = k4 + 2*k5*r2 + 3*k6*r4;
			const double dradial_dr2 = (dnum_dr2 * den - num * dden_dr2) * inv_den * inv_den;
			// J_dist_norm (2x2)
			// ud = un*radial + 2*p1*un*vn + p2*(r2 + 2*un^2)
			// vd = vn*radial + p1*(r2 + 2*vn^2) + 2*p2*un*vn
			const double dud_dun = radial + un * dradial_dr2 * 2 * un + 2*p1*vn + p2*(2*un + 4*un);
			const double dud_dvn = un * dradial_dr2 * 2 * vn + 2*p1*un + p2*(2*vn);
			const double dvd_dun = vn * dradial_dr2 * 2 * un + p1*(2*un) + 2*p2*vn;
			const double dvd_dvn = radial + vn * dradial_dr2 * 2 * vn + p1*(2*vn + 4*vn) + 2*p2*un;
			// Combined Jacobian J = J_pixel_dist * J_dist_norm * J_norm_Pcam
			// J_pixel_norm = [fx 0; 0 fy] * [dud_dun dud_dvn; dvd_dun dvd_dvn] * [inv_z 0 -x*inv_z2; 0 inv_z -y*inv_z2]
			Eigen::Matrix<double, 2, 2, Eigen::RowMajor> J_pixel_dist;
			J_pixel_dist(0, 0) = fx * dud_dun;
			J_pixel_dist(0, 1) = fx * dud_dvn;
			J_pixel_dist(1, 0) = fy * dvd_dun;
			J_pixel_dist(1, 1) = fy * dvd_dvn;
			const Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_pixel_Pcam = J_pixel_dist * J_norm_Pcam;
			// Jacobian w.r.t. pose (qw, qx, qy, qz, Cx, Cy, Cz)
			if (jacobians[0]) {
				Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_pose(jacobians[0]);
				// J_pose_q = J_pixel_Pcam * J_Pcam_q
				J_pose.leftCols<4>() = J_pixel_Pcam * J_Pcam_q;
				// J_pose_C = J_pixel_Pcam * J_Pcam_C = J_pixel_Pcam * (-R)
				J_pose.rightCols<3>() = J_pixel_Pcam * (-R);
			}
			// Jacobian w.r.t. 3D point (X, Y, Z)
			if (jacobians[2]) {
				Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_pt(jacobians[2]);
				// J_pt = J_pixel_Pcam * J_Pcam_X = J_pixel_Pcam * R
				J_pt = J_pixel_Pcam * R;
			}
			// Jacobian w.r.t. intrinsics (12 params)
			if (jacobians[1]) {
				Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> J_intr(jacobians[1]);
				J_intr.setZero();
				// ud, vd are distorted normalized coords
				const double ud = un * radial + 2.0*p1*un*vn + p2*(r2 + 2.0*un*un);
				const double vd = vn * radial + p1*(r2 + 2.0*vn*vn) + 2.0*p2*un*vn;
				// du/dfx = ud + cx (already handled: u = fx*ud + cx)
				// Actually u = fx*ud + cx, so du/dfx = ud
				J_intr(0, 0) = ud;
				J_intr(0, 2) = 1.0; // du/dcx
				J_intr(1, 0) = intrinsics[1] * vd; // dv/dfx = (fy/fx)*vd
				J_intr(1, 1) = fx * vd;            // dv/d(fy/fx) = fx*vd
				J_intr(1, 3) = 1.0; // dv/dcy
				// Derivatives w.r.t. k1..k6, p1, p2
				const double common_u = fx * un * inv_den;
				const double common_v = fy * vn * inv_den;
				const double common_den_u = -fx * un * num * inv_den * inv_den;
				const double common_den_v = -fy * vn * num * inv_den * inv_den;
				J_intr(0, 4) = common_u * r2;   // du/dk1
				J_intr(0, 5) = common_u * r4;   // du/dk2
				J_intr(0, 6) = common_u * r6;   // du/dk3
				J_intr(0, 7) = fx * 2.0*un*vn;  // du/dp1
				J_intr(0, 8) = fx * (r2 + 2.0*un*un); // du/dp2
				J_intr(0, 9) = common_den_u * r2;  // du/dk4
				J_intr(0, 10) = common_den_u * r4; // du/dk5
				J_intr(0, 11) = common_den_u * r6; // du/dk6
				J_intr(1, 4) = common_v * r2;    // dv/dk1
				J_intr(1, 5) = common_v * r4;    // dv/dk2
				J_intr(1, 6) = common_v * r6;    // dv/dk3
				J_intr(1, 7) = fy * (r2 + 2.0*vn*vn); // dv/dp1
				J_intr(1, 8) = fy * 2.0*un*vn;     // dv/dp2
				J_intr(1, 9) = common_den_v * r2;  // dv/dk4
				J_intr(1, 10) = common_den_v * r4; // dv/dk5
				J_intr(1, 11) = common_den_v * r6; // dv/dk6
			}
		}

		return true;
	}

private:
	const double observed_x_, observed_y_;
};

// Angular reprojection error for Spherical cameras
// No intrinsics parameter block needed
struct SphericalAngularReprojectionError {
	SphericalAngularReprojectionError(double observed_x, double observed_y,
	                                  int width, int height)
	{
		const double longitude = (observed_x/width - 0.5) * 2.0 * M_PI;
		const double latitude = (observed_y/height - 0.5) * M_PI;
		const double cos_lat = COS(latitude);
		const double sin_lat = SIN(latitude);
		const double cos_lon = COS(longitude);
		const double sin_lon = SIN(longitude);
		// Pre-scale tangent basis by pixel scale (width / 2π) for efficiency
		// This converts angular error directly to pixel error without runtime computation
		const double pixel_scale = double(width) * M_1_PI * 0.5;
		// Tangent basis vectors (u, v) at the observation point on the unit sphere
		// u = dP/d_lon normalized = [cos(lon), 0, -sin(lon)] * pixel_scale
		u_ = Eigen::Vector3d(cos_lon * pixel_scale, 0.0, -sin_lon * pixel_scale);
		// v = dP/d_lat normalized = [-sin(lat)sin(lon), cos(lat), -sin(lat)cos(lon)] * pixel_scale
		v_ = Eigen::Vector3d(-sin_lat * sin_lon, cos_lat, -sin_lat * cos_lon) * pixel_scale;
	}

	template <typename T>
	bool operator()(
		const T* const pose,   // 7 params: quat[4] + center[3]
		const T* const point,  // 3 params: X, Y, Z
		T* residuals) const
	{
		typedef Eigen::Matrix<T, 3, 1> Vector3;
		// Transform to camera space
		Vector3 p_cam;
		CameraSpaceTransform(pose, point, p_cam.data());
		// Note: NO z-check for spherical cameras - they can see in all directions!
		const Vector3 pred_ray = p_cam.normalized();
		// Project predicted ray onto tangent plane basis (u, v)
		// res_u = dot(pred, u)
		// res_v = dot(pred, v)
		//
		// WHY TANGENT PLANE PROJECTION?
		// 1. Numerical Stability: Minimizing the angle directly (acos(dot)) has a singularity
		//    at 0 error (derivative -> infinity), causing optimizer instability.
		//    Tangent plane projection behaves like Euclidean distance locally and is stable.
		// 2. Information Density: Returns 2 residuals (u, v) instead of 1 scalar (angle).
		//    This provides a gradient vector pointing to the solution, constraining
		//    the optimization much better than a single scalar magnitude.
		//
		// Residuals are already in pixels (basis vectors are pre-scaled)
		residuals[0] = pred_ray.dot(u_.template cast<T>());
		residuals[1] = pred_ray.dot(v_.template cast<T>());
		return true;
	}

	// Factory
	static ceres::CostFunction* Create(double observed_x, double observed_y,
	                                   int width, int height) {
		return new ceres::AutoDiffCostFunction<SphericalAngularReprojectionError, 2, 7, 3>(
			new SphericalAngularReprojectionError(observed_x, observed_y, width, height));
	}

private:
	Eigen::Vector3d u_, v_; // pre-scaled tangent basis (includes pixel scale)
};

// GPS position error cost functor
// Constrains camera center to known GPS position
struct GPSPositionError {
	GPSPositionError(
		double gps_x, double gps_y, double gps_z,
		double accuracy_horizontal, double accuracy_vertical,
		double weight_horizontal, double weight_vertical)
		: gps_x_(gps_x), gps_y_(gps_y), gps_z_(gps_z),
		  accuracy_h_(accuracy_horizontal), accuracy_v_(accuracy_vertical),
		  weight_h_(weight_horizontal), weight_v_(weight_vertical) {}

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {
		// Extract camera center from pose (quaternion is in pose[0:4])
		const T* center = pose + 4;

		// Compute weighted residuals
		// Horizontal (X, Y)
		residuals[0] = weight_h_ * (center[0] - gps_x_) / accuracy_h_;
		residuals[1] = weight_h_ * (center[1] - gps_y_) / accuracy_h_;

		// Vertical (Z)
		residuals[2] = weight_v_ * (center[2] - gps_z_) / accuracy_v_;

		return true;
	}

	static ceres::CostFunction* Create(
		double gps_x, double gps_y, double gps_z,
		double accuracy_h, double accuracy_v,
		double weight_h, double weight_v)
	{
		return new ceres::AutoDiffCostFunction<GPSPositionError, 3, 7>(
			new GPSPositionError(gps_x, gps_y, gps_z, accuracy_h, accuracy_v, weight_h, weight_v));
	}

	const double gps_x_, gps_y_, gps_z_;
	const double accuracy_h_, accuracy_v_;
	const double weight_h_, weight_v_;
};
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_BUNDLEADJUSTMENT_COSTFUNCTIONS_H_
