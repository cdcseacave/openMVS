/*
 * LensFit.cpp
 *
 * Copyright (c) 2014-2026 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// I N C L U D E S /////////////////////////////////////////////////

#include "Common.h"
#include "LensFit.h"
#include "Scene.h"
#include "PairsMatcher.h"
#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#pragma pop_macro("VERBOSE")
#pragma pop_macro("LOG")
#include <PoseLib/poselib.h>

#include <array>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)

// multi-threading of the fit's work per pair based on OpenMP
#ifdef _USE_OPENMP
#define LENSFIT_USE_OPENMP
#endif

// A lens is held one to one this far past the farthest radius its camera's matches reach while it is
// fitted, and past the image corner to be kept, or, given, to be held: right at the farthest match
// the fit can walk into a lens that folds just beyond it, where the undistortion stops having one
// answer and the cost stops being smooth (the prototype stalled at 1.13 px where the right lens gives
// 0.48 without it)
constexpr REAL LENS_VALID_MARGIN = REAL(1.05);
// Newton iterations of the undistortion inside the fit, on the radius from r_u = r_d: a fixed count,
// so that the automatic derivatives of every residual run through the same iterations; it converges
// wherever the lens is one to one out to the point, which LENS_VALID_MARGIN keeps true
constexpr int LENS_UNDISTORT_ITERATIONS = 10;
// Levenberg-Marquardt iterations of one fit, a bound well past what the fits take: Playground's first
// joint fit converged in 40, the first of a synthetic scene of the strong lens (164 px at the corner)
// in 69
constexpr int LENS_FIT_MAX_ITERATIONS = 200;
// The inlier bar, pixels at each side's focal, of a pair's pose re-estimated through the fitted lens:
// its matches were verified at MatchConfig::maxEpipolarError already, so this only has to tell the
// pose that explains them from a reversed or turned one the pinhole left (the prototype's bar)
constexpr double LENS_REESTIMATION_ERROR = 1.0;
// RANSAC iterations of that re-estimation: the bounds PairsMatcher::GeometricFilter uses
constexpr size_t LENS_REESTIMATION_MIN_ITERATIONS = 100;
constexpr size_t LENS_REESTIMATION_MAX_ITERATIONS = 10000;
// Below this median Sampson error, pixels, the poses explain the matches to the rounding of their
// stored float positions (exact synthetic data sit within 1e-3 px): there is no error for a lens to
// explain, and none is kept
constexpr double LENS_MIN_MEDIAN_ERROR = 0.01;
// Added to the squared gradient of a match's epipolar constraint, which vanishes only on the epipole,
// so that its distance stays finite there
constexpr double LENS_MIN_GRADIENT_SQ = 1e-30;
// The |t.x| of a unit baseline direction t (the cosine of its angle to the x axis) above which the x
// axis is too close to t to build the tangent basis from, their cross product short and poorly
// conditioned, so the y axis is used
constexpr double LENS_TANGENT_MAX_COS_X = 0.9;
// The check of a fit, which tells one that reached its lenses from one held short of them by the pairs
// whose pinhole poses it could not turn: refined alone from the fit, its pose and its own copy of its
// fitted cameras' lenses free, a pair explains its matches about as well as the fit does where the fit is
// right, and far better where it is held short. A fitted camera passes when at least LENS_CHECK_MIN_SHARE
// of its pairs keep the fit's median error within LENS_CHECK_MAX_RATIO of the median they reach alone
// (or under LENS_MIN_MEDIAN_ERROR). Over 100 synthetic scenes of each lens test (barrel lenses of 164
// and 20 px at the corner, pincushions of 80 and 160 px, a second camera's of 22 px) every first fit that
// reached its lens kept 0.975-1.000 of its pairs so, and every one held short of it (more than 10 px
// off) 0.27-0.73, at 0.25 px of noise as at 0.5; the five test captures' fits keep 0.967-1.000. The
// same share of pairs whose median error past 0.7 of the corner radius stays within the loss scale,
// which needs no solve, separates them too at 0.5 px (0.96 and more against 0.87 at most), but its bar
// is absolute: at 0.25 px of noise a fit held short keeps up to 0.92
constexpr double LENS_CHECK_MAX_RATIO = 1.5;
constexpr double LENS_CHECK_MIN_SHARE = 0.85;
// Where the first fit fails the check, the joint fit is started again from its fitted cameras' lenses
// scaled this many times (k1 and k2 both), each pair's pose first re-estimated through that start, and
// the fit of least cost is kept; a start that folds before a camera's valid radius is skipped. A fit held
// short reaches part of the lens and of its sign (25-72% of it at the corner in the synthetic scenes),
// so that doubled it starts past the poses that held the first fit: over 100 scenes of each lens test
// every fit held short so reached its lens. Starts from fixed lenses do not: from radial changes of -5,
// -10 and +5% at the corner, the pincushion of 160 px stays short in 14 of 35 scenes
constexpr double LENS_FIT_RESTART_SCALES[] = {2};


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("LensFit "));

namespace {

inline double Scalar(double x) { return x; }
template <typename T, int N>
inline double Scalar(const ceres::Jet<T, N>& x) { return x.a; }

// The undistortion of a normalized distorted radius rd through the radial lens (k1, k2), as the
// ratio r_u / r_d: Newton on r_u (1 + k1 r_u^2 + k2 r_u^4) = r_d from r_u = r_d
template <typename T>
T UndistortionScale(const T& k1, const T& k2, double rd)
{
	if (rd <= 0)
		return T(1);
	T ru(rd);
	for (int i = 0; i < LENS_UNDISTORT_ITERATIONS; ++i) {
		const T r2(ru*ru);
		const T D(ru*(T(1) + k1*r2 + k2*r2*r2) - T(rd));
		const T dD(T(1) + T(3)*k1*r2 + T(5)*k2*r2*r2);
		ru -= D/dD;
	}
	return ru/T(rd);
}

// J^-1 v, J = s I + c u u^T the Jacobian of the radial distortion at the undistorted point u
// (s = 1 + k1 r^2 + k2 r^4, c = 2 k1 + 4 k2 r^2), by Sherman-Morrison
template <typename T>
void InverseLensJacobianTimes(const T& k1, const T& k2, const T* u, const T* v, T* out)
{
	const T r2(u[0]*u[0] + u[1]*u[1]);
	const T s(T(1) + k1*r2 + k2*r2*r2);
	const T c(T(2)*k1 + T(4)*k2*r2);
	const T k(c*(u[0]*v[0] + u[1]*v[1])/(s + c*r2));
	out[0] = (v[0] - k*u[0])/s;
	out[1] = (v[1] - k*u[1])/s;
}

// An orthonormal basis of the plane orthogonal to the unit vector t
void TangentBasis(const Point3& t, Point3& e1, Point3& e2)
{
	const Point3 a(ABS(t.x) < LENS_TANGENT_MAX_COS_X ? Point3(1, 0, 0) : Point3(0, 1, 0));
	e1 = normalized(t.cross(a));
	e2 = t.cross(e1);
}

// The normalized radius of a pinhole camera's image corner farthest from its principal point
double CornerRadius(const PinholeCamera& pinhole)
{
	double radius = 0;
	for (const Point2d& corner : {Point2d(0, 0), Point2d(pinhole.size.width - 1, 0),
		Point2d(0, pinhole.size.height - 1), Point2d(pinhole.size.width - 1, pinhole.size.height - 1)})
		radius = MAXF(radius, norm(Point2d((corner.x - pinhole.cx)/pinhole.fx, (corner.y - pinhole.cy)/pinhole.fy)));
	return radius;
}

// One match of one pair: the first-order distance of its two measured, distorted pixels to the pair's
// epipolar constraint u2^T [t]x R u1 = 0, each side through its own lens and focal -- the gradient of
// the constraint with respect to each measured pixel passes through the inverse of that side's lens
// Jacobian -- in pixels.
// The pair's pose is 5 numbers about its base pose (R0, t0): R = exp([w]x) R0 and
// t = normalize(t0 + a e1 + b e2), e1 and e2 spanning the plane orthogonal to t0 (x2 = R x1 + t).
// A side's lens is a parameter block where it is fitted, else a constant held here.
// A fitted lens that is not one to one out to its valid radius (the farthest normalized radius its
// camera's matches reach, times LENS_VALID_MARGIN) makes the evaluation fail, which Ceres answers by
// shrinking its trust region; a held side (valid radius 0) is not tested: its lens is one to one past
// its image corner, or the camera would not have been held.
struct LensSampsonError {
	Point2d m1, m2;            // the measured points, normalized by their camera's focal and centre (distorted)
	Point2d f1, f2;            // each side's focal (fx, fy), to bring the gradient back to pixels
	Matrix3x3 R0;              // the base rotation
	Point3 t0, e1, e2;         // the base baseline direction and its tangent basis
	double held1[2], held2[2]; // each side's lens where it is held
	double valid1, valid2;     // each side's valid radius where its lens is fitted, else 0
	bool sameCamera;           // both sides are one camera: one lens block
	bool freeFirst;            // one fitted side of two cameras: whether it is the first

	template <typename T>
	bool Evaluate(const T* pose, const T* lens1, const T* lens2, T* residual) const {
		if (valid1 > 0 && !IsLensOneToOne(Scalar(lens1[0]), Scalar(lens1[1]), valid1))
			return false;
		if (valid2 > 0 && !IsLensOneToOne(Scalar(lens2[0]), Scalar(lens2[1]), valid2))
			return false;
		T dR[9];
		ceres::AngleAxisToRotationMatrix(pose, ceres::RowMajorAdapter3x3(dR));
		T R[9];
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				R[r*3+c] = dR[r*3+0]*T(R0(0,c)) + dR[r*3+1]*T(R0(1,c)) + dR[r*3+2]*T(R0(2,c));
		T t[3] = {
			T(t0.x) + pose[3]*T(e1.x) + pose[4]*T(e2.x),
			T(t0.y) + pose[3]*T(e1.y) + pose[4]*T(e2.y),
			T(t0.z) + pose[3]*T(e1.z) + pose[4]*T(e2.z) };
		const T tNorm(ceres::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]));
		for (T& v : t)
			v /= tNorm;
		const T tx[9] = { T(0), -t[2], t[1],  t[2], T(0), -t[0],  -t[1], t[0], T(0) };
		T E[9];
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c)
				E[r*3+c] = tx[r*3+0]*R[0*3+c] + tx[r*3+1]*R[1*3+c] + tx[r*3+2]*R[2*3+c];
		const T s1(UndistortionScale(lens1[0], lens1[1], norm(m1)));
		const T s2(UndistortionScale(lens2[0], lens2[1], norm(m2)));
		const T u1[2] = { T(m1.x)*s1, T(m1.y)*s1 };
		const T u2[2] = { T(m2.x)*s2, T(m2.y)*s2 };
		const T Eu1[3] = {
			E[0]*u1[0] + E[1]*u1[1] + E[2],
			E[3]*u1[0] + E[4]*u1[1] + E[5],
			E[6]*u1[0] + E[7]*u1[1] + E[8] };
		const T Etu2[2] = {
			E[0]*u2[0] + E[3]*u2[1] + E[6],
			E[1]*u2[0] + E[4]*u2[1] + E[7] };
		const T g(u2[0]*Eu1[0] + u2[1]*Eu1[1] + Eu1[2]);
		T ga[2], gb[2];
		InverseLensJacobianTimes(lens1[0], lens1[1], u1, Etu2, ga);
		InverseLensJacobianTimes(lens2[0], lens2[1], u2, Eu1, gb);
		const T gradSq(ga[0]*ga[0]/T(f1.x*f1.x) + ga[1]*ga[1]/T(f1.y*f1.y) +
			gb[0]*gb[0]/T(f2.x*f2.x) + gb[1]*gb[1]/T(f2.y*f2.y));
		residual[0] = g/ceres::sqrt(gradSq + T(LENS_MIN_GRADIENT_SQ));
		return true;
	}
	// both sides held
	template <typename T>
	bool operator()(const T* pose, T* residual) const {
		const T l1[2] = { T(held1[0]), T(held1[1]) }, l2[2] = { T(held2[0]), T(held2[1]) };
		return Evaluate(pose, l1, l2, residual);
	}
	// one lens block: the camera of both sides, or the one fitted side of two cameras
	template <typename T>
	bool operator()(const T* pose, const T* lens, T* residual) const {
		if (sameCamera)
			return Evaluate(pose, lens, lens, residual);
		if (freeFirst) {
			const T l2[2] = { T(held2[0]), T(held2[1]) };
			return Evaluate(pose, lens, l2, residual);
		}
		const T l1[2] = { T(held1[0]), T(held1[1]) };
		return Evaluate(pose, l1, lens, residual);
	}
	// two lens blocks
	template <typename T>
	bool operator()(const T* pose, const T* lens1, const T* lens2, T* residual) const {
		return Evaluate(pose, lens1, lens2, residual);
	}
};

// One camera of a fit
struct FitCamera {
	unsigned idxScene;      // its index in scene.cameras and the report
	bool free;              // its lens is fitted, else held at `lens`
	double lens[2];         // k1, k2: the start, then the fit
	Point2d focal, centre;
	double reach;           // the farthest normalized distorted radius its drawn matches reach
	double corner;          // the normalized radius of its image corner farthest from the centre
};
// One pair of a fit
struct FitPair {
	IIndex idxPair;              // its index in scene.pairs
	unsigned cam1, cam2;         // into the fit's cameras
	Matrix3x3 R0;                // its base pose (x2 = R x1 + t), the stored one to start with
	Point3 t0;
	std::vector<Point2d> m1, m2; // its drawn matches, normalized and distorted
};

// The residual of match k of pair p; refineLens false holds every lens at its current value
LensSampsonError MakeResidual(const FitPair& p, size_t k, const std::vector<FitCamera>& cameras, bool refineLens)
{
	const FitCamera& c1 = cameras[p.cam1];
	const FitCamera& c2 = cameras[p.cam2];
	LensSampsonError e;
	e.m1 = p.m1[k];
	e.m2 = p.m2[k];
	e.f1 = c1.focal;
	e.f2 = c2.focal;
	e.R0 = p.R0;
	e.t0 = p.t0;
	TangentBasis(p.t0, e.e1, e.e2);
	e.held1[0] = c1.lens[0]; e.held1[1] = c1.lens[1];
	e.held2[0] = c2.lens[0]; e.held2[1] = c2.lens[1];
	const bool free1 = refineLens && c1.free, free2 = refineLens && c2.free;
	e.valid1 = free1 ? LENS_VALID_MARGIN*c1.reach : 0;
	e.valid2 = free2 ? LENS_VALID_MARGIN*c2.reach : 0;
	e.sameCamera = p.cam1 == p.cam2;
	e.freeFirst = free1;
	return e;
}

// Match k of pair p's Sampson error at the pair's base pose and the cameras' current lenses, pixels
double ResidualAt(const FitPair& p, size_t k, const std::vector<FitCamera>& cameras)
{
	static const double zero[5] = {0, 0, 0, 0, 0};
	double residual;
	MakeResidual(p, k, cameras, false).Evaluate(zero, cameras[p.cam1].lens, cameras[p.cam2].lens, &residual);
	return residual;
}

// The soft L1 cost of a pair at its base pose and the cameras' current lenses
double PairCost(const FitPair& p, const std::vector<FitCamera>& cameras, double lossScale)
{
	double cost = 0;
	for (size_t k = 0; k < p.m1.size(); ++k)
		cost += 2*lossScale*lossScale*(SQRT(1 + SQUARE(ResidualAt(p, k, cameras)/lossScale)) - 1);
	return cost;
}

// Move a pair's base to the pose its 5 numbers describe
void Rebase(FitPair& p, const double* pose)
{
	double dR[9];
	ceres::AngleAxisToRotationMatrix(pose, ceres::RowMajorAdapter3x3(dR));
	Point3 e1, e2;
	TangentBasis(p.t0, e1, e2);
	p.R0 = Matrix3x3(dR[0], dR[1], dR[2], dR[3], dR[4], dR[5], dR[6], dR[7], dR[8])*p.R0;
	p.t0 = normalized(p.t0 + e1*pose[3] + e2*pose[4]);
}

// The poses alone at the cameras' current lenses: each pair its own small problem
void RefinePosesAlone(std::vector<FitPair>& pairs, const std::vector<FitCamera>& cameras, double lossScale)
{
	for (FitPair& p : pairs) {
		ceres::Problem problem;
		ceres::LossFunction* const loss = new ceres::SoftLOneLoss(lossScale);
		double pose[5] = {0, 0, 0, 0, 0};
		for (size_t k = 0; k < p.m1.size(); ++k)
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LensSampsonError, 1, 5>(
				new LensSampsonError(MakeResidual(p, k, cameras, false))), loss, pose);
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.max_num_iterations = LENS_FIT_MAX_ITERATIONS;
		options.logging_type = ceres::SILENT;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		Rebase(p, pose);
	}
}

// The residuals of pair p's matches added to a problem over its 5 pose numbers `pose` and the lens block of
// each of its fitted cameras (`cameras`' own), under `loss`
void AddPairResiduals(ceres::Problem& problem, ceres::LossFunction* loss, const FitPair& p, std::vector<FitCamera>& cameras, double* pose)
{
	FitCamera& c1 = cameras[p.cam1];
	FitCamera& c2 = cameras[p.cam2];
	ASSERT(c1.free || c2.free); // a pair is selected for a fitted camera it informs
	for (size_t k = 0; k < p.m1.size(); ++k) {
		LensSampsonError* const e = new LensSampsonError(MakeResidual(p, k, cameras, true));
		if (p.cam1 == p.cam2)
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LensSampsonError, 1, 5, 2>(e), loss, pose, c1.lens);
		else if (c1.free && c2.free)
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LensSampsonError, 1, 5, 2, 2>(e), loss, pose, c1.lens, c2.lens);
		else
			problem.AddResidualBlock(new ceres::AutoDiffCostFunction<LensSampsonError, 1, 5, 2>(e), loss, pose, c1.free ? c1.lens : c2.lens);
	}
}

// The lenses of the fitted cameras and every pair's pose together, from the pairs' bases, each pair's
// 5 pose numbers eliminated first (they touch only their own pair's residuals), so the reduced
// system holds 2 unknowns per fitted camera; the bases then move to the solution.
// Returns whether the solver reached a usable solution; where it did not, no lens or pose of the fit
// may be judged.
bool RefineJointly(std::vector<FitPair>& pairs, std::vector<FitCamera>& cameras, double lossScale, unsigned numThreads)
{
	ceres::Problem problem;
	ceres::LossFunction* const loss = new ceres::SoftLOneLoss(lossScale);
	std::vector<std::array<double, 5>> poses(pairs.size(), std::array<double, 5>{0, 0, 0, 0, 0});
	auto ordering = std::make_shared<ceres::ParameterBlockOrdering>();
	FOREACH(i, pairs) {
		AddPairResiduals(problem, loss, pairs[i], cameras, poses[i].data());
		ordering->AddElementToGroup(poses[i].data(), 0);
	}
	for (FitCamera& c : cameras)
		if (c.free && problem.HasParameterBlock(c.lens))
			ordering->AddElementToGroup(c.lens, 1);
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.linear_solver_ordering = ordering;
	options.max_num_iterations = LENS_FIT_MAX_ITERATIONS;
	options.num_threads = (int)numThreads;
	options.logging_type = ceres::SILENT;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	DEBUG_EXTRA("Lens fit over %u pairs: %s", (unsigned)pairs.size(), summary.BriefReport().c_str());
	FOREACH(i, pairs)
		Rebase(pairs[i], poses[i].data());
	return summary.IsSolutionUsable();
}

// The median absolute Sampson error of every fit camera's matches at the current bases and lenses,
// pixels; a match between two cameras counts for both. A camera with an error that is not finite (an
// undistortion with no answer, which the roles and the fit's valid radius keep every lens clear of)
// gets a NaN median: it fails every comparison of the verdict, which therefore refuses its lens.
std::vector<double> MedianErrors(const std::vector<FitPair>& pairs, const std::vector<FitCamera>& cameras)
{
	std::vector<std::vector<double>> errors(cameras.size());
	for (const FitPair& p : pairs) {
		for (size_t k = 0; k < p.m1.size(); ++k) {
			const double e = ABS(ResidualAt(p, k, cameras));
			errors[p.cam1].push_back(e);
			if (p.cam2 != p.cam1)
				errors[p.cam2].push_back(e);
		}
	}
	std::vector<double> medians(cameras.size(), 0.0);
	FOREACH(c, errors) {
		std::vector<double>& e = errors[c];
		if (e.empty())
			continue;
		if (std::any_of(e.begin(), e.end(), [](double v) { return !ISFINITE(v); })) {
			medians[c] = std::numeric_limits<double>::quiet_NaN();
			continue;
		}
		std::nth_element(e.begin(), e.begin() + e.size()/2, e.end());
		medians[c] = e[e.size()/2];
	}
	return medians;
}

// Every pair's pose estimated again by RANSAC on its matches undistorted through the current lenses,
// and taken where it explains the pair better than the pose the joint fit walked to: a pair whose
// pinhole pose was reversed or turned wrong cannot be walked back by a local fit (on Playground the
// prototype replaced 5 poses, then 1, and moved the corner from 69.4 to 72.3 px, onto COLMAP's 72.5).
// Returns the number of poses replaced.
unsigned ReestimatePoses(std::vector<FitPair>& pairs, const std::vector<FitCamera>& cameras, double lossScale)
{
	unsigned numReplaced = 0;
	for (FitPair& p : pairs) {
		const FitCamera& c1 = cameras[p.cam1];
		const FitCamera& c2 = cameras[p.cam2];
		std::vector<poselib::Point3D> bearings1, bearings2;
		bearings1.reserve(p.m1.size());
		bearings2.reserve(p.m2.size());
		for (size_t k = 0; k < p.m1.size(); ++k) {
			const double s1 = UndistortionScale(c1.lens[0], c1.lens[1], norm(p.m1[k]));
			const double s2 = UndistortionScale(c2.lens[0], c2.lens[1], norm(p.m2[k]));
			bearings1.emplace_back(poselib::Point3D(p.m1[k].x*s1, p.m1[k].y*s1, 1.0).normalized());
			bearings2.emplace_back(poselib::Point3D(p.m2[k].x*s2, p.m2[k].y*s2, 1.0).normalized());
		}
		poselib::RelativePoseOptions opt;
		opt.max_error = 0.5*(ATAN(LENS_REESTIMATION_ERROR/(0.5*(c1.focal.x + c1.focal.y))) +
			ATAN(LENS_REESTIMATION_ERROR/(0.5*(c2.focal.x + c2.focal.y))));
		opt.ransac.min_iterations = LENS_REESTIMATION_MIN_ITERATIONS;
		opt.ransac.max_iterations = LENS_REESTIMATION_MAX_ITERATIONS;
		poselib::CameraPose pose;
		std::vector<char> inliers;
		const poselib::RansacStats stats = poselib::estimate_relative_pose_bearings(bearings1, bearings2, opt, &pose, &inliers);
		if (stats.num_inliers == 0)
			continue;
		const Matrix3x3 R0(p.R0);
		const Point3 t0(p.t0);
		const double costNow = PairCost(p, cameras, lossScale);
		// the conversions of PairsMatcher::GeometricFilter's relative pose
		p.R0 = pose.R();
		p.t0 = normalized(Point3(pose.t));
		if (PairCost(p, cameras, lossScale) < costNow) {
			++numReplaced;
			continue;
		}
		p.R0 = R0;
		p.t0 = t0;
	}
	return numReplaced;
}

// The joint fit and its rounds of pose re-estimation from the pairs' bases and the cameras' lenses as
// they are; rounds receives the poses each round replaced. Returns whether every joint fit reached a
// usable solution (RefineJointly)
bool FitFrom(std::vector<FitPair>& pairs, std::vector<FitCamera>& cameras, const MatchConfig& config, unsigned numThreads, std::vector<unsigned>& rounds)
{
	bool solved = RefineJointly(pairs, cameras, config.lensFitLossScale, numThreads);
	rounds.clear();
	for (unsigned round = 0; solved && round < config.lensFitRounds; ++round) {
		const unsigned numReplaced = ReestimatePoses(pairs, cameras, config.lensFitLossScale);
		rounds.push_back(numReplaced);
		if (numReplaced == 0)
			break;
		solved = RefineJointly(pairs, cameras, config.lensFitLossScale, numThreads);
	}
	return solved;
}

// The soft L1 cost of every pair at its base and the cameras' current lenses
double TotalCost(const std::vector<FitPair>& pairs, const std::vector<FitCamera>& cameras, double lossScale)
{
	double cost = 0;
	for (const FitPair& p : pairs)
		cost += PairCost(p, cameras, lossScale);
	return cost;
}

// The median of `values`, which it reorders
double MedianOf(std::vector<double>& values)
{
	ASSERT(!values.empty());
	std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
	return values[values.size()/2];
}

// Whether every fitted camera passes the check (LENS_CHECK_MIN_SHARE) at the pairs' bases and the
// cameras' current lenses: each pair refined alone from there, its 5 pose numbers and its own copy of its
// fitted cameras' lenses free
bool PassesPairsCheck(const std::vector<FitPair>& pairs, const std::vector<FitCamera>& cameras, double lossScale, unsigned numThreads)
{
	std::vector<char> explained(pairs.size(), 0);
	#ifdef LENSFIT_USE_OPENMP
	#pragma omp parallel for schedule(dynamic) num_threads(numThreads)
	#endif
	for (int_t i = 0; i < (int_t)pairs.size(); ++i) {
		const FitPair& p = pairs[i];
		std::vector<double> errors(p.m1.size());
		for (size_t k = 0; k < p.m1.size(); ++k)
			errors[k] = ABS(ResidualAt(p, k, cameras));
		const double medianFit = MedianOf(errors);
		if (medianFit < LENS_MIN_MEDIAN_ERROR) {
			explained[i] = 1; // nothing left for the pair alone to explain
			continue;
		}
		FitPair alone(p);
		std::vector<FitCamera> own(cameras);
		double pose[5] = {0, 0, 0, 0, 0};
		ceres::Problem problem;
		AddPairResiduals(problem, new ceres::SoftLOneLoss(lossScale), alone, own, pose);
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.max_num_iterations = LENS_FIT_MAX_ITERATIONS;
		options.logging_type = ceres::SILENT;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		Rebase(alone, pose);
		for (size_t k = 0; k < p.m1.size(); ++k)
			errors[k] = ABS(ResidualAt(alone, k, own));
		explained[i] = medianFit <= LENS_CHECK_MAX_RATIO*MedianOf(errors);
	}
	std::vector<unsigned> numPairs(cameras.size(), 0), numExplained(cameras.size(), 0);
	FOREACH(i, pairs) {
		const FitPair& p = pairs[i];
		++numPairs[p.cam1];
		numExplained[p.cam1] += explained[i];
		if (p.cam2 != p.cam1) {
			++numPairs[p.cam2];
			numExplained[p.cam2] += explained[i];
		}
	}
	FOREACH(c, cameras)
		if (cameras[c].free && numExplained[c] < LENS_CHECK_MIN_SHARE*numPairs[c])
			return false;
	return true;
}

} // namespace

std::unordered_set<CameraPtr> LensFitReport::FittedCameras() const
{
	std::unordered_set<CameraPtr> fitted;
	for (const LensFitOutcome& outcome : cameras)
		if (outcome.status == LensFitOutcome::FITTED)
			fitted.insert(outcome.pCamera);
	return fitted;
}

REAL SFM::LensDisplacement(REAL focal, REAL k1, REAL k2, REAL radius)
{
	ASSERT(focal > 0 && radius >= 0);
	const double rd = radius/focal;
	return radius - rd*UndistortionScale<double>(k1, k2, rd)*focal;
}

bool SFM::IsLensOneToOne(REAL k1, REAL k2, REAL radius)
{
	ASSERT(radius > 0);
	// D(r) = r (1 + k1 r^2 + k2 r^4) rises from 0 while D'(r) = 1 + 3 k1 s + 5 k2 s^2 stays positive,
	// s = r^2: it is one to one out to `radius` when it passes it before the first positive root s* of
	// D', or when D' has none (then D rises without bound: k2 > 0, or k2 = 0 and k1 >= 0)
	REAL sStar = -1;
	if (k2 == 0) {
		if (k1 < 0)
			sStar = REAL(-1)/(3*k1);
	} else {
		const REAL disc = 9*k1*k1 - 20*k2;
		if (disc >= 0) {
			const REAL root = SQRT(disc);
			for (const REAL s : {(-3*k1 - root)/(10*k2), (-3*k1 + root)/(10*k2)})
				if (s > 0 && (sStar < 0 || s < sStar))
					sStar = s;
		}
	}
	if (sStar < 0)
		return true;
	return SQRT(sStar)*(1 + k1*sStar + k2*sStar*sStar) > radius;
}

LensFitReport SFM::FitLensesFromPairs(Scene& scene, const MatchConfig& config)
{
	// a fitted camera reads at least one pair and every pair at least one match; the rotation band
	// is not empty, the error drop is a fraction, and the loss has a scale
	ASSERT(config.lensFitMinPairs >= 1 && config.lensFitMaxPairs >= 1 && config.lensFitMatchesPerPair >= 1);
	ASSERT(config.lensFitMinRotation <= config.lensFitMaxRotation);
	ASSERT(config.lensFitMinErrorDrop >= 0 && config.lensFitMinErrorDrop < 1);
	ASSERT(config.lensFitLossScale > 0);
	const auto start = std::chrono::steady_clock::now();
	LensFitReport report;
	const unsigned numCameras = (unsigned)scene.cameras.size();
	report.cameras.resize(numCameras);
	// what every camera is to the fit
	enum class Role : uint8_t {
		FIT,  // its lens is fitted
		HOLD, // its given lens is held, its pairs informing the fitted ones
		LEAVE // it takes no part, nor do its pairs
	};
	std::vector<Role> roles(numCameras, Role::LEAVE);
	std::unordered_map<CameraPtr, unsigned> cameraIndex;
	FOREACH(c, scene.cameras) {
		const CameraPtr pCamera = scene.cameras[c];
		cameraIndex.emplace(pCamera, (unsigned)c);
		LensFitOutcome& outcome = report.cameras[c];
		outcome.pCamera = pCamera;
		if (pCamera->GetType() != CameraType::PINHOLE || !pCamera->TrustIntrinsics())
			continue;
		const PinholeCamera& pinhole = *static_cast<const PinholeCamera*>(pCamera);
		if (!pinhole.HasDistortion() && !pinhole.HasAdditionalDistortion()) {
			roles[c] = Role::FIT;
			continue;
		}
		if (pinhole.k3 != 0 || pinhole.p1 != 0 || pinhole.p2 != 0 || pinhole.HasAdditionalDistortion())
			continue; // a lens the fit's model cannot hold, which held as k1, k2 alone would bias the cameras it pairs with
		if (!IsLensOneToOne(pinhole.k1, pinhole.k2, LENS_VALID_MARGIN*CornerRadius(pinhole)))
			continue; // a lens that folds within its frame: past the fold its undistortion has no answer, and held it would send its pairs' errors, and the fit of every camera it pairs with, astray
		roles[c] = Role::HOLD;
		outcome.status = LensFitOutcome::HELD;
		outcome.k1 = pinhole.k1;
		outcome.k2 = pinhole.k2;
	}
	// the pairs that pass the per-pair bars, heaviest first (ties in pair order)
	std::vector<IIndex> candidates;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.relativePose.has_value() || !pair.HasValidWeight() ||
			pair.GetNumFilteredInliers() < config.lensFitMatchesPerPair)
			continue;
		const REAL angle = R2D(ACOS(ComputeAngle(pair.relativePose->R)));
		if (angle < config.lensFitMinRotation || angle > config.lensFitMaxRotation)
			continue;
		candidates.push_back((IIndex)idxPair);
	}
	std::stable_sort(candidates.begin(), candidates.end(), [&](IIndex a, IIndex b) {
		return scene.pairs[a].GetCompositeWeight() > scene.pairs[b].GetCompositeWeight();
	});
	const auto CamerasOf = [&](const ImagePair& pair) {
		return std::make_pair(cameraIndex.at(scene.images[pair.ID1].pCamera), cameraIndex.at(scene.images[pair.ID2].pCamera));
	};
	while (true) {
		// the eligible pairs: between fitted or held cameras, one at least fitted; a fitted camera with
		// too few leaves with its pairs, which can drop another one under the bar
		std::vector<IIndex> eligible;
		for (bool changed = true; changed; ) {
			changed = false;
			eligible.clear();
			std::vector<unsigned> counts(numCameras, 0);
			for (IIndex idxPair : candidates) {
				const auto [c1, c2] = CamerasOf(scene.pairs[idxPair]);
				if (roles[c1] == Role::LEAVE || roles[c2] == Role::LEAVE || (roles[c1] != Role::FIT && roles[c2] != Role::FIT))
					continue;
				eligible.push_back(idxPair);
				if (roles[c1] == Role::FIT)
					++counts[c1];
				if (c2 != c1 && roles[c2] == Role::FIT)
					++counts[c2];
			}
			for (unsigned c = 0; c < numCameras; ++c) {
				if (roles[c] != Role::FIT)
					continue;
				LensFitOutcome& outcome = report.cameras[c];
				outcome.numEligiblePairs = counts[c];
				if (counts[c] < config.lensFitMinPairs) {
					roles[c] = Role::LEAVE;
					outcome.status = LensFitOutcome::TOO_FEW_PAIRS;
					// what an earlier fit found for it no longer stands
					outcome.k1 = outcome.k2 = 0;
					outcome.numPairs = 0;
					outcome.medianBefore = outcome.medianAfter = outcome.cornerDisplacement = 0;
					changed = true;
				}
			}
		}
		if (std::none_of(roles.begin(), roles.end(), [](Role r) { return r == Role::FIT; }))
			break;
		// heaviest first, until every fitted camera has its pairs; a pair between two counts for both
		std::vector<unsigned> numSelected(numCameras, 0);
		std::vector<IIndex> selected;
		for (IIndex idxPair : eligible) {
			const auto [c1, c2] = CamerasOf(scene.pairs[idxPair]);
			if (!(roles[c1] == Role::FIT && numSelected[c1] < config.lensFitMaxPairs) &&
				!(roles[c2] == Role::FIT && numSelected[c2] < config.lensFitMaxPairs))
				continue;
			selected.push_back(idxPair);
			if (roles[c1] == Role::FIT)
				++numSelected[c1];
			if (c2 != c1 && roles[c2] == Role::FIT)
				++numSelected[c2];
		}
		// the fit's cameras, in the order the selected pairs first name them
		std::vector<FitCamera> cameras;
		std::vector<int> fitIndex(numCameras, -1);
		const auto FitIndexOf = [&](unsigned c) -> unsigned {
			if (fitIndex[c] < 0) {
				const PinholeCamera& pinhole = *static_cast<const PinholeCamera*>(scene.cameras[c]);
				FitCamera f;
				f.idxScene = c;
				f.free = roles[c] == Role::FIT;
				f.lens[0] = f.free ? 0.0 : (double)pinhole.k1;
				f.lens[1] = f.free ? 0.0 : (double)pinhole.k2;
				f.focal = Point2d(pinhole.fx, pinhole.fy);
				f.centre = Point2d(pinhole.cx, pinhole.cy);
				f.reach = 0;
				f.corner = CornerRadius(pinhole);
				fitIndex[c] = (int)cameras.size();
				cameras.push_back(f);
			}
			return (unsigned)fitIndex[c];
		};
		std::vector<FitPair> pairs;
		pairs.reserve(selected.size());
		for (IIndex idxPair : selected) {
			const ImagePair& pair = scene.pairs[idxPair];
			const auto [c1, c2] = CamerasOf(pair);
			FitPair p;
			p.idxPair = idxPair;
			p.cam1 = FitIndexOf(c1);
			p.cam2 = FitIndexOf(c2);
			p.R0 = pair.relativePose->R;
			p.t0 = normalized(pair.relativePose->GetT());
			// lensFitMatchesPerPair of its sparse inliers, drawn by a partial Fisher-Yates shuffle from a
			// generator seeded from the pair's IDs, so that the same pair always contributes the same
			// ones, on every platform: each index is the engine's raw output, whose sequence the standard
			// fixes, modulo the inliers left to draw from (std::uniform_int_distribution's algorithm is
			// each standard library's own); the modulo favours some values by at most n/2^64 for n left,
			// under 1e-15 for a few thousand inliers
			const unsigned numSparse = pair.GetNumFilteredInliers();
			ASSERT(numSparse >= config.lensFitMatchesPerPair); // an eligible pair holds that many
			std::vector<uint32_t> draw(numSparse);
			std::iota(draw.begin(), draw.end(), 0u);
			std::mt19937_64 rng(PairIdx(pair.ID1, pair.ID2).idx);
			for (unsigned i = 0; i < config.lensFitMatchesPerPair; ++i)
				std::swap(draw[i], draw[i + rng() % (numSparse - i)]);
			const Image& img1 = scene.images[pair.ID1];
			const Image& img2 = scene.images[pair.ID2];
			FitCamera& f1 = cameras[p.cam1];
			FitCamera& f2 = cameras[p.cam2];
			for (unsigned i = 0; i < config.lensFitMatchesPerPair; ++i) {
				const DMatch& m = pair.matches[draw[i]];
				const Point2f& x1 = img1.keypoints[m.queryIdx].pt;
				const Point2f& x2 = img2.keypoints[m.trainIdx].pt;
				p.m1.emplace_back((x1.x - f1.centre.x)/f1.focal.x, (x1.y - f1.centre.y)/f1.focal.y);
				p.m2.emplace_back((x2.x - f2.centre.x)/f2.focal.x, (x2.y - f2.centre.y)/f2.focal.y);
				f1.reach = MAXF(f1.reach, norm(p.m1.back()));
				f2.reach = MAXF(f2.reach, norm(p.m2.back()));
			}
			pairs.push_back(std::move(p));
		}
		// the baseline: the poses alone at the starting lenses, so that the lens is credited only with
		// what the poses could not do; the joint fit still starts from the stored poses, which have not
		// absorbed the lens any further
		std::vector<FitPair> baseline(pairs);
		RefinePosesAlone(baseline, cameras, config.lensFitLossScale);
		const std::vector<double> mediansBefore(MedianErrors(baseline, cameras));
		const std::vector<FitPair> storedPairs(pairs);
		const std::vector<FitCamera> startCameras(cameras);
		bool solved = FitFrom(pairs, cameras, config, scene.nMaxThreads, report.reestimatedPoses);
		report.numStarts = 1;
		if (solved && !PassesPairsCheck(pairs, cameras, config.lensFitLossScale, scene.nMaxThreads)) {
			// held short of its lenses: started again from them scaled, from the stored poses each
			// re-estimated through that start, the fit of least cost kept
			const std::vector<FitCamera> firstCameras(cameras);
			double cost = TotalCost(pairs, cameras, config.lensFitLossScale);
			for (const double scale : LENS_FIT_RESTART_SCALES) {
				std::vector<FitPair> restartPairs(storedPairs);
				std::vector<FitCamera> restartCameras(startCameras);
				bool folds = false;
				FOREACH(c, restartCameras) {
					FitCamera& camera = restartCameras[c];
					if (!camera.free)
						continue;
					camera.lens[0] = scale*firstCameras[c].lens[0];
					camera.lens[1] = scale*firstCameras[c].lens[1];
					if (!IsLensOneToOne(camera.lens[0], camera.lens[1], LENS_VALID_MARGIN*camera.reach))
						folds = true;
				}
				if (folds)
					continue; // the fit could not evaluate it: its undistortion has no answer within the matches' reach
				ReestimatePoses(restartPairs, restartCameras, config.lensFitLossScale);
				std::vector<unsigned> rounds;
				++report.numStarts;
				if (!FitFrom(restartPairs, restartCameras, config, scene.nMaxThreads, rounds))
					continue;
				const double restartCost = TotalCost(restartPairs, restartCameras, config.lensFitLossScale);
				if (restartCost >= cost)
					continue;
				pairs.swap(restartPairs);
				cameras.swap(restartCameras);
				report.reestimatedPoses.swap(rounds);
				cost = restartCost;
			}
		}
		const std::vector<double> mediansAfter(MedianErrors(pairs, cameras));
		// the verdict on every fitted camera; a refused one keeps no lens and leaves, and the others
		// are fitted again without its pairs; a fit the solver could not complete is not judged, and
		// refuses every camera it fits
		bool anyRefused = false;
		FOREACH(f, cameras) {
			const FitCamera& fc = cameras[f];
			if (!fc.free)
				continue;
			LensFitOutcome& outcome = report.cameras[fc.idxScene];
			outcome.k1 = (REAL)fc.lens[0];
			outcome.k2 = (REAL)fc.lens[1];
			outcome.numPairs = numSelected[fc.idxScene];
			outcome.medianBefore = mediansBefore[f];
			outcome.medianAfter = mediansAfter[f];
			const bool oneToOne = IsLensOneToOne(outcome.k1, outcome.k2, LENS_VALID_MARGIN*(REAL)fc.corner);
			// the undistortion of the corner has an answer only through a lens one to one out to it
			const REAL focal = REAL(0.5)*(REAL)(fc.focal.x + fc.focal.y);
			outcome.cornerDisplacement = oneToOne ? LensDisplacement(focal, outcome.k1, outcome.k2, (REAL)fc.corner*focal) : REAL(0);
			// kept only when every bar holds, written so that a NaN median, which fails every
			// comparison, refuses the lens
			if (solved && oneToOne && mediansBefore[f] >= LENS_MIN_MEDIAN_ERROR &&
				mediansAfter[f] <= (1.0 - (double)config.lensFitMinErrorDrop)*mediansBefore[f]) {
				outcome.status = LensFitOutcome::FITTED;
				continue;
			}
			outcome.status = (solved && !oneToOne) ? LensFitOutcome::FOLDS : LensFitOutcome::NO_ERROR_DROP;
			roles[fc.idxScene] = Role::LEAVE;
			anyRefused = true;
		}
		if (!anyRefused)
			break;
	}
	for (const LensFitOutcome& outcome : report.cameras)
		if (outcome.status == LensFitOutcome::FITTED)
			static_cast<PinholeCamera*>(outcome.pCamera)->SetDistortion(outcome.k1, outcome.k2, 0, 0);
	report.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	return report;
}

LensFitReport SFM::FitLensesAndReverifyPairs(PairsMatcher& pairsMatcher)
{
	TD_TIMER_START();
	Scene& scene = pairsMatcher.GetScene();
	const MatchConfig& config = pairsMatcher.GetConfig();
	const LensFitReport report = FitLensesFromPairs(scene, config);
	FOREACH(c, report.cameras) {
		const LensFitOutcome& outcome = report.cameras[c];
		switch (outcome.status) {
		case LensFitOutcome::FITTED:
			VERBOSE("Lens fitted to camera %u from %u pairs: k1 %.4f, k2 %.4f (%.1f px at the corner), median error %.3f -> %.3f px",
				(unsigned)c, outcome.numPairs, outcome.k1, outcome.k2, outcome.cornerDisplacement, outcome.medianBefore, outcome.medianAfter);
			break;
		case LensFitOutcome::FOLDS:
		case LensFitOutcome::NO_ERROR_DROP:
			VERBOSE("Lens of camera %u not kept (%s): k1 %.4f, k2 %.4f from %u pairs, median error %.3f -> %.3f px",
				(unsigned)c, outcome.status == LensFitOutcome::FOLDS ? "it folds before the image corner" : "the fit does not lower the median error enough",
				outcome.k1, outcome.k2, outcome.numPairs, outcome.medianBefore, outcome.medianAfter);
			break;
		case LensFitOutcome::TOO_FEW_PAIRS:
			DEBUG("Lens of camera %u not fitted: %u eligible pairs, %u needed", (unsigned)c, outcome.numEligiblePairs, config.lensFitMinPairs);
			break;
		default:
			break;
		}
	}
	const std::unordered_set<CameraPtr> fitted(report.FittedCameras());
	if (fitted.empty())
		return report;
	// the inliers of the pairs a fitted camera holds, before and after they are verified again, and
	// the pairs among them that form tracks, named by their images since the weights recomputed after
	// sort scene.pairs again
	const auto Totals = [&](size_t& numSparse, size_t& numDense, std::unordered_set<PairIdx>& forming) {
		unsigned numPairs = 0;
		numSparse = numDense = 0;
		forming.clear();
		for (const ImagePair& pair : scene.pairs) {
			if (!fitted.count(scene.images[pair.ID1].pCamera) && !fitted.count(scene.images[pair.ID2].pCamera))
				continue;
			++numPairs;
			numSparse += pair.GetNumFilteredInliers();
			numDense += pair.GetNumDenseInliers();
			if (pair.GetNumTrackFormingMatches() > 0)
				forming.emplace(pair.ID1, pair.ID2);
		}
		return numPairs;
	};
	size_t sparseBefore, denseBefore, sparseAfter, denseAfter;
	std::unordered_set<PairIdx> formingBefore, formingAfter;
	const unsigned numPairs = Totals(sparseBefore, denseBefore, formingBefore);
	pairsMatcher.ComputeRelativePoses(true, false, fitted);
	Totals(sparseAfter, denseAfter, formingAfter);
	// the pairs the verification through the lens invalidated: track-forming matches before, none after
	const unsigned numInvalidated = (unsigned)std::count_if(formingBefore.begin(), formingBefore.end(),
		[&](const PairIdx& pair) { return formingAfter.count(pair) == 0; });
	String rounds;
	for (unsigned n : report.reestimatedPoses)
		rounds += String::FormatString(rounds.empty() ? "%u" : ",%u", n);
	if (report.numStarts > 1)
		rounds += String::FormatString(" (the best of %u starts)", report.numStarts);
	VERBOSE("Lens fit: %u of %u cameras fitted in %.1f s, poses re-estimated %s; %u pairs verified again through their lens: sparse inliers %zu -> %zu, dense %zu -> %zu; %u pairs invalidated (%s)",
		(unsigned)fitted.size(), (unsigned)report.cameras.size(), report.seconds, rounds.empty() ? "-" : rounds.c_str(),
		numPairs, sparseBefore, sparseAfter, denseBefore, denseAfter, numInvalidated, TD_TIMER_GET_FMT().c_str());
	return report;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
