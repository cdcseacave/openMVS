/*
 * BundleAdjustment.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

// Include Eigen before OpenCV to avoid header ordering issues
#include "Common.h"
#include "BundleAdjustment.h"
#include "Scene.h"
#include "../Math/GeodeticTransforms.h"
#include "BundleAdjustmentCostFunctions.h"

#include <ceres/crs_matrix.h>
#include <ceres/covariance.h>
#include <Eigen/Sparse>

using namespace SFM;

// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// Convert OpenMVS pose to/from Ceres quaternion parameterization [qw, qx, qy, qz, Cx, Cy, Cz]
void SFM::Pose3DToQuaternionAndCenter(const Pose3D& pose, double* params) {
	ceres::RotationMatrixToQuaternion(ceres::RowMajorAdapter3x3(pose.R.val), params);
	Eigen::Map<Point3d::EVec>(params + 4) = (const Point3d::EVec)pose.C;
}
void SFM::QuaternionAndCenterToPose3D(const double* params, Pose3D& pose) {
	ceres::QuaternionToRotation(params, ceres::RowMajorAdapter3x3(pose.R.val));
	pose.C = Eigen::Map<const Point3d::EVec>(params + 4);
}

// Convert OpenMVS pose to/from Ceres angle-axis parameterization [ax, ay, az, Cx, Cy, Cz]
void SFM::Pose3DToAngleAxisAndCenter(const Pose3D& pose, double* params) {
	ceres::RotationMatrixToAngleAxis(ceres::RowMajorAdapter3x3(pose.R.val), params);
	Eigen::Map<Point3d::EVec>(params + 3) = (const Point3d::EVec)pose.C;
}
void SFM::AngleAxisAndCenterToPose3D(const double* params, Pose3D& pose) {
	ceres::AngleAxisToRotationMatrix(params, ceres::RowMajorAdapter3x3(pose.R.val));
	pose.C = Eigen::Map<const Point3d::EVec>(params + 3);
}
/*----------------------------------------------------------------*/


// =====================================================================================
// Per-image BA pose covariance (Schur complement of points + sparse selected inverse).
// Math adapted from COLMAP estimators/covariance.cc: the 3D points are conditionally
// independent given the cameras, so the Gauss-Newton Hessian H = J^T J is reduced by the
// block-diagonal Schur complement S = H_cc - H_cp H_pp^-1 H_pc; the per-pose marginal
// covariance blocks are then read off the SELECTED inverse of S (Takahashi recursion over
// its sparse Cholesky factor — no dense inverse). Intrinsics are treated as fixed here
// (a globally-shared camera would densify S), giving a pose covariance conditioned on
// intrinsics — adequate as a relative trust signal.
// =====================================================================================
namespace {

inline int CeresTangentSize(const ceres::Problem& problem, const double* block) {
	#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
	return problem.ParameterBlockTangentSize(block);
	#else
	return problem.ParameterBlockLocalSize(block);
	#endif
}

// Sparse selected inverse Z of an SPD sparse matrix S, on the pattern of L+L^T, via the
// Takahashi recursion over the simplicial LDLT factor of a fill-reducing permutation of S.
// Adds increasing diagonal damping to recover from rank deficiency (gauge). Returns false
// if still rank-deficient after retries. On success S^-1(a,b) = Z(permInv(a), permInv(b)).
// condFloorRel bounds the conditioning: every LDLT pivot is driven above condFloorRel*maxDiag,
// so a residual gauge null space left in S (e.g. the 1-DOF global-scale mode of a GPS-free
// network) gets a large-but-finite variance rather than overflowing the recursion to +inf.
// Pass 0 for a full-rank system (GPS-anchored) to damp only up to positive-definiteness.
bool ComputeSelectedInverse(Eigen::SparseMatrix<double>& S,
	Eigen::SparseMatrix<double>& Zout, Eigen::PermutationMatrix<Eigen::Dynamic>& permOut,
	double condFloorRel)
{
	// Scale-aware damping so the (gauge) null space is regularized rather than rejected: add
	// delta*I until the factor is positive-definite, then use the regularized inverse. Any
	// residual gauge null space left by the caller (datum removal or GPS-prior anchoring) is
	// absorbed by this damping, instead of failing outright.
	double maxDiag = 0.0;
	for (int i = 0; i < S.rows(); ++i) maxDiag = std::max(maxDiag, std::abs(S.coeff(i, i)));
	// Start near the numerical-zero scale and grow only until the factor is positive-definite, so
	// the regularization touches just the gauge null space and does not cap (saturate) the
	// covariance of genuinely weakly-constrained poses at 1/damping. When condFloorRel>0 the
	// pivots are additionally driven above pivotFloor: this bounds the conditioning so a residual
	// gauge null space (e.g. the global-scale mode) yields a finite variance instead of an inf.
	const double pivotFloor = condFloorRel * maxDiag; // 0 when condFloorRel==0
	double damping = std::max(1e-15 * maxDiag, 1e-300), applied = 0.0;
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt;
	Eigen::VectorXd D;
	bool ok = false;
	for (int attempt = 0; attempt < 20; ++attempt) {
		const double delta = damping - applied;
		for (int i = 0; i < S.rows(); ++i) S.coeffRef(i, i) += delta;
		applied = damping;
		ldlt.compute(S);
		if (ldlt.info() == Eigen::Success) {
			D = ldlt.vectorD();
			if ((D.array() > pivotFloor).all() && (D.array() > 0.0).all()) { ok = true; break; }
		}
		damping *= 10.0;
	}
	if (!ok) return false;

	const Eigen::SparseMatrix<double> L = ldlt.matrixL();
	permOut = ldlt.permutationP();
	const int n = (int)L.rows();
	Eigen::SparseMatrix<double> Lstrict = L;
	for (int k = 0; k < Lstrict.outerSize(); ++k)
		for (Eigen::SparseMatrix<double>::InnerIterator it(Lstrict, k); it; ++it)
			if (it.row() == it.col()) it.valueRef() = 0.0;
	Lstrict.prune([](int, int, double v) { return v != 0.0; });

	Zout = Lstrict;
	Zout += Eigen::SparseMatrix<double>(Lstrict.transpose());
	for (int i = 0; i < n; ++i) Zout.coeffRef(i, i) = 0.0;
	Zout.makeCompressed();
	Eigen::SparseMatrix<double>& Z = Zout;

	const double dFloor = std::max(damping, pivotFloor);
	for (int j = n - 1; j >= 0; --j) {
		std::vector<int> nz;
		for (Eigen::SparseMatrix<double>::InnerIterator it(Lstrict, j); it; ++it)
			nz.push_back((int)it.row());
		std::sort(nz.begin(), nz.end());
		for (int i : nz) {
			double zij = 0.0;
			for (int k : nz)
				zij -= Lstrict.coeff(k, j) * (k >= i ? Z.coeff(k, i) : Z.coeff(i, k));
			Z.coeffRef(i, j) = zij;
			Z.coeffRef(j, i) = zij;
		}
		double zjj = 1.0 / std::max(D(j), dFloor);
		for (int k : nz)
			zjj -= Lstrict.coeff(k, j) * Z.coeff(k, j);
		Z.coeffRef(j, j) = zjj;
	}
	return true;
}

} // namespace

// Estimate per-image pose uncertainty from the solved BA problem kept alive by Adjust().
// The pose block uses the SE(3) product manifold (tangent size 6, ordered
// [rotation(3), translation(3)]); the per-axis variances are the diagonal of the two 3x3
// marginal-covariance blocks. Returns one entry per image, or an empty array on failure.
PoseUncertaintyArr BundleAdjustment::ComputePoseUncertainty()
{
	TD_TIMER_STARTD();
	constexpr double damping = 1e-8; // regularization for the 3x3 point blocks of the Schur complement
	if (!problem)
		return PoseUncertaintyArr();
	struct PoseRef { IIndex imageID; const double* block; int start; int size; };
	std::vector<PoseRef> poses;
	IIndexArr datumIDs; // valid poses BA held constant: gauge references, perfectly known here
	FOREACH(i, scene.images) {
		if (!scene.images[i].IsValid())
			continue;
		const double* block = poseParams.data() + i * 7;
		if (!problem->HasParameterBlock(const_cast<double*>(block)))
			continue;
		if (problem->IsParameterBlockConstant(const_cast<double*>(block))) { datumIDs.push_back(i); continue; }
		poses.push_back({ (IIndex)i, block, 0, CeresTangentSize(*problem, block) });
	}
	if (poses.size() < 2)
		return PoseUncertaintyArr();
	// If BA did not fix the gauge, choose the best-connected pose as the datum and exclude it
	// from the covariance system, removing the 6-DOF rotation+translation gauge null space
	// (residual scale DOF is absorbed by damping). GPS priors already anchor the gauge, so in
	// that case every pose stays in the system and the covariances are absolute (ENU).
	if (datumIDs.empty() && numGPSResiduals == 0) {
		size_t best = 0;
		for (size_t k = 1; k < poses.size(); ++k)
			if (numReprojResidualsPerImage[poses[k].imageID] > numReprojResidualsPerImage[poses[best].imageID])
				best = k;
		datumIDs.push_back(poses[best].imageID);
		poses.erase(poses.begin() + best);
		if (poses.size() < 2)
			return PoseUncertaintyArr();
	}
	std::vector<const double*> points;
	for (const Track& track : scene.tracks) {
		const double* xyz = track.position.ptr();
		if (problem->HasParameterBlock(const_cast<double*>(xyz)) &&
		    !problem->IsParameterBlockConstant(const_cast<double*>(xyz)))
			points.push_back(xyz);
	}
	int poseNum = 0;
	for (PoseRef& p : poses) { p.start = poseNum; poseNum += p.size; }
	const int pointNum = (int)points.size() * 3;

	// Evaluate the Jacobian in the order [poses, points] (intrinsics excluded -> held fixed).
	ceres::Problem::EvaluateOptions eopts;
	eopts.parameter_blocks.reserve(poses.size() + points.size());
	for (const PoseRef& p : poses) eopts.parameter_blocks.push_back(const_cast<double*>(p.block));
	for (const double* b : points) eopts.parameter_blocks.push_back(const_cast<double*>(b));
	double cost; ceres::CRSMatrix Jcrs;
	if (!problem->Evaluate(eopts, &cost, nullptr, nullptr, &Jcrs)) {
		VERBOSE("warning: pose-covariance Jacobian evaluation failed");
		return PoseUncertaintyArr();
	}
	const Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor>> J(
		Jcrs.num_rows, Jcrs.num_cols, (int)Jcrs.values.size(),
		Jcrs.rows.data(), Jcrs.cols.data(), Jcrs.values.data());

	// Schur-eliminate the points (block diagonal 3x3) -> reduced pose system S.
	Eigen::SparseMatrix<double> S;
	if (pointNum == 0) {
		S = (J.transpose() * J).eval();
	} else {
		const Eigen::SparseMatrix<double> Ja = J.block(0, 0, J.rows(), poseNum);
		const Eigen::SparseMatrix<double> Jp = J.block(0, poseNum, J.rows(), pointNum);
		const Eigen::SparseMatrix<double> Haa = Ja.transpose() * Ja;
		const Eigen::SparseMatrix<double> Hap = Ja.transpose() * Jp;
		Eigen::SparseMatrix<double> Hpp = Jp.transpose() * Jp; // exactly block-diagonal (3x3 per point)
		for (int idx = 0; idx < pointNum; idx += 3) {
			const Eigen::Matrix3d blk = Eigen::Matrix3d(Hpp.block(idx, idx, 3, 3)) + damping * Eigen::Matrix3d::Identity();
			const Eigen::Matrix3d blkInv = blk.inverse();
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					Hpp.coeffRef(idx + r, idx + c) = blkInv(r, c);
		}
		Hpp.makeCompressed();
		// Materialize each sparse product as a concrete column-major matrix so the final
		// subtraction does not hit Eigen's storage-order mismatch (transpose() is row-major).
		const Eigen::SparseMatrix<double> HapT = Hap.transpose();
		const Eigen::SparseMatrix<double> HppHapT = Hpp * HapT;
		const Eigen::SparseMatrix<double> reduced = Hap * HppHapT;
		S = Haa - reduced;
	}

	// With no GPS priors the reduced system still carries the 1-DOF global-scale gauge (the
	// rotation+translation gauge was already removed by excluding the datum pose above), so
	// bound the conditioning to give that scale mode a finite variance. GPS-anchored systems
	// are full rank and use the exact (unbounded-precision) selected inverse.
	const double condFloorRel = (numGPSResiduals == 0) ? 1e-9 : 0.0;
	Eigen::SparseMatrix<double> Z;
	Eigen::PermutationMatrix<Eigen::Dynamic> perm;
	if (!ComputeSelectedInverse(S, Z, perm, condFloorRel)) {
		VERBOSE("warning: pose-covariance selected-inverse failed (rank-deficient gauge)");
		return PoseUncertaintyArr();
	}
	// Eigen's SimplicialLDLT factors P*S*P^T = L*D*L^T (P = perm), so S^-1(i,j) = Z(perm[i], perm[j])
	// where Z is the selected inverse of L*D*L^T -- index Z with the FORWARD permutation.
	const Eigen::PermutationMatrix<Eigen::Dynamic>::IndicesType& permIdx = perm.indices();

	PoseUncertaintyArr uncertainty(scene.images.size());
	const PoseUncertainty invalid{Point3f(-1.f, -1.f, -1.f), Point3f(-1.f, -1.f, -1.f), Point3f(-1.f, -1.f, -1.f)};
	FOREACH(i, uncertainty)
		uncertainty[i] = invalid;
	MeanStdMinMax<float> statR, statT;
	for (const PoseRef& p : poses) {
		if (p.size < 6)
			continue; // partially-fixed pose (subset manifold): leave not-computed
		const auto Zat = [&Z, &permIdx, &p](int r, int c) {
			return Z.coeff(permIdx(p.start + r), permIdx(p.start + c));
		};
		PoseUncertainty& u = uncertainty[p.imageID];
		// The diagonal variances are non-negative for the SPD covariance, but the selected-inverse
		// recursion can emit a tiny negative on an axis whose true variance sits at the roundoff
		// floor; clamp so a stray negative does not become a NaN in the downstream sqrt (1-sigma
		// CSV export / Viewer ellipsoid). Off-diagonals stay signed -- they carry real correlations.
		const auto Zvar = [&Zat](int i) { return MAXF(0.f, (float)Zat(i, i)); };
		u.rotVar = Point3f(Zvar(0), Zvar(1), Zvar(2));
		u.posVar = Point3f(Zvar(3), Zvar(4), Zvar(5));
		u.posCov = Point3f((float)Zat(3, 4), (float)Zat(3, 5), (float)Zat(4, 5));
		statR.Update(u.MaxRotationVariance()); statT.Update(u.MaxPositionVariance());
	}
	for (const IIndex id : datumIDs)
		uncertainty[id].rotVar = uncertainty[id].posVar = uncertainty[id].posCov = Point3f(0.f, 0.f, 0.f); // reference datum
	DEBUG("Pose uncertainty: %u/%u images (rotVar mean %.3g, posVar mean %.3g) in %s",
		(unsigned)poses.size(), (unsigned)scene.images.size(),
		statR.size ? statR.GetMean() : 0.f, statT.size ? statT.GetMean() : 0.f, TD_TIMER_GET_FMT().c_str());
	return uncertainty;
}
/*----------------------------------------------------------------*/


// Reference cross-check of ComputePoseUncertainty() using Ceres' own (slow, dense) covariance
// estimator. Same pose set, same gauge/datum, same conditioning (intrinsics fixed as they are
// held constant in the problem; points marginalized by Ceres) — so the two must agree.
PoseUncertaintyArr BundleAdjustment::ComputePoseUncertaintyCeres()
{
	TD_TIMER_STARTD();
	if (!problem)
		return PoseUncertaintyArr();
	struct PoseRef { IIndex imageID; double* block; int size; };
	std::vector<PoseRef> poses;
	IIndexArr datumIDs;
	FOREACH(i, scene.images) {
		if (!scene.images[i].IsValid())
			continue;
		double* block = poseParams.data() + i * 7;
		if (!problem->HasParameterBlock(block))
			continue;
		if (problem->IsParameterBlockConstant(block)) { datumIDs.push_back(i); continue; }
		poses.push_back({ (IIndex)i, block, CeresTangentSize(*problem, block) });
	}
	if (poses.size() < 2)
		return PoseUncertaintyArr();
	// Match ComputePoseUncertainty()'s gauge handling: with no GPS priors and no BA-fixed pose,
	// hold the best-connected pose constant so the reduced system is (all but the scale mode)
	// full rank; DENSE_SVD's null-space thresholding absorbs the residual gauge freedom.
	std::vector<double*> tempFixed;
	if (datumIDs.empty() && numGPSResiduals == 0) {
		size_t best = 0;
		for (size_t k = 1; k < poses.size(); ++k)
			if (numReprojResidualsPerImage[poses[k].imageID] > numReprojResidualsPerImage[poses[best].imageID])
				best = k;
		datumIDs.push_back(poses[best].imageID);
		problem->SetParameterBlockConstant(poses[best].block);
		tempFixed.push_back(poses[best].block);
		poses.erase(poses.begin() + best);
	}

	PoseUncertaintyArr uncertainty;
	if (poses.size() >= 2) {
		ceres::Covariance::Options options;
		options.algorithm_type = ceres::DENSE_SVD; // slow reference; robust to rank deficiency (gauge)
		options.null_space_rank = -1;              // drop only the numerically-zero (gauge) modes
		options.apply_loss_function = true;        // robustified GN Hessian, as Problem::Evaluate() uses
		options.num_threads = 1;
		ceres::Covariance covariance(options);
		std::vector<std::pair<const double*, const double*>> blocks;
		blocks.reserve(poses.size());
		for (const PoseRef& p : poses)
			blocks.emplace_back(p.block, p.block);
		if (covariance.Compute(blocks, problem.get())) {
			uncertainty.resize(scene.images.size());
			const PoseUncertainty invalid{Point3f(-1.f,-1.f,-1.f), Point3f(-1.f,-1.f,-1.f), Point3f(-1.f,-1.f,-1.f)};
			FOREACH(i, uncertainty)
				uncertainty[i] = invalid;
			for (const PoseRef& p : poses) {
				if (p.size < 6)
					continue;
				double cov[36];
				if (!covariance.GetCovarianceBlockInTangentSpace(p.block, p.block, cov))
					continue;
				// tangent order [rotation(3), position(3)]; cov is row-major 6x6
				PoseUncertainty& u = uncertainty[p.imageID];
				u.rotVar = Point3f((float)cov[0*6+0], (float)cov[1*6+1], (float)cov[2*6+2]);
				u.posVar = Point3f((float)cov[3*6+3], (float)cov[4*6+4], (float)cov[5*6+5]);
				u.posCov = Point3f((float)cov[3*6+4], (float)cov[3*6+5], (float)cov[4*6+5]);
			}
			for (const IIndex id : datumIDs)
				uncertainty[id].rotVar = uncertainty[id].posVar = uncertainty[id].posCov = Point3f(0.f,0.f,0.f);
		} else {
			VERBOSE("warning: reference (Ceres) pose-covariance computation failed");
		}
	}
	// undo the temporary datum fix so the problem is left as it was
	for (double* b : tempFixed)
		problem->SetParameterBlockVariable(b);
	DEBUG("Pose uncertainty (Ceres reference): %u/%u images in %s",
		(unsigned)poses.size(), (unsigned)scene.images.size(), TD_TIMER_GET_FMT().c_str());
	return uncertainty;
}
/*----------------------------------------------------------------*/


unsigned SFM::ExportPoseUncertaintyCSV(const String& fileName, const Scene& scene)
{
	const PoseUncertaintyArr& uncertainty = scene.poseUncertainty;
	if (uncertainty.size() != scene.images.size())
		return 0;
	std::ofstream os(fileName);
	if (!os.is_open())
		return 0;

	// Per-image inlier observation counts
	UnsignedArr numObsPerImage(scene.images.size());
	numObsPerImage.Memset(0);
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track)
			if (obs.imageID < numObsPerImage.size())
				++numObsPerImage[obs.imageID];
	}

	const bool geoAligned = scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN);
	const auto isDatum = [](const PoseUncertainty& u) {
		return u.IsValid() && u.MaxPositionVariance() == 0.f && u.MaxRotationVariance() == 0.f;
	};
	bool hasDatum = false;
	FOREACH(i, uncertainty)
		if (isDatum(uncertainty[i])) { hasDatum = true; break; }

	os << "# pose uncertainty (1-sigma): position in "
	   << (geoAligned ? "ENU meters (East/North/Up)" : "world units")
	   << " (frame: " << (geoAligned ? "ENU" : "local")
	   << ", gauge: " << (hasDatum ? "datum-relative" : "absolute")
	   << "); rotation in degrees about the camera x/y/z axes; -1 = not computed; all-zero = gauge datum\n";
	os << "ID,name,valid,datum,sigmaPosX,sigmaPosY,sigmaPosZ,covPosXY,covPosXZ,covPosYZ,"
	      "sigmaRotX,sigmaRotY,sigmaRotZ,numObs,gpsAccuracyXY,gpsAccuracyZ\n";
	os << std::setprecision(9);

	unsigned numValid = 0, numSpherical = 0;
	FloatArr posSigmas;
	MeanStdMinMax<float> statPos;
	FOREACH(i, scene.images) {
		const Image& image = scene.images[i];
		const PoseUncertainty& u = uncertainty[i];
		const bool valid = image.IsValid() && u.IsValid();
		const bool datum = valid && isDatum(u);
		Point3f sigmaPos(-1.f, -1.f, -1.f), covPos(-1.f, -1.f, -1.f), sigmaRot(-1.f, -1.f, -1.f);
		if (valid) {
			sigmaPos = Point3f(SQRT(u.posVar.x), SQRT(u.posVar.y), SQRT(u.posVar.z));
			covPos = u.posCov;
			sigmaRot = Point3f(R2D(SQRT(u.rotVar.x)), R2D(SQRT(u.rotVar.y)), R2D(SQRT(u.rotVar.z)));
			++numValid;
			if (image.GetCameraType() == CameraType::SPHERICAL)
				++numSpherical;
			if (!datum) {
				const float maxSigma = SQRT(u.MaxPositionVariance());
				posSigmas.push_back(maxSigma);
				statPos.Update(maxSigma);
			}
		}
		// the CSV is parsed positionally by comma, so keep the name a single column: a comma in
		// the file name would shift every following field for a consumer (e.g. the Viewer loader)
		String name = Util::getFileName(image.fileName);
		std::replace(name.begin(), name.end(), ',', '_');
		const View::Metadata& meta = image.View::metadata;
		os << image.ID << ','
		   << name << ','
		   << (valid ? 1 : 0) << ',' << (datum ? 1 : 0) << ','
		   << sigmaPos.x << ',' << sigmaPos.y << ',' << sigmaPos.z << ','
		   << covPos.x << ',' << covPos.y << ',' << covPos.z << ','
		   << sigmaRot.x << ',' << sigmaRot.y << ',' << sigmaRot.z << ','
		   << numObsPerImage[i] << ','
		   << meta.positionAccuracy << ',' << meta.positionAccuracyZ << '\n';
	}

	VERBOSE("Pose quality report: %u/%u images (max position sigma mean %.3g, median %.3g, max %.3g %s) exported to '%s'",
		numValid, scene.images.size(),
		statPos.size ? statPos.GetMean() : 0.f,
		posSigmas.empty() ? 0.f : posSigmas.GetMedian(),
		statPos.size ? statPos.maxVal : 0.f,
		geoAligned ? "m" : "units", fileName.c_str());
	// spherical images are split into fresh-ID cube-map faces by ExportMVS, so their rows here
	// (keyed by the SFM image ID) will not correlate with the exported .mvs image IDs
	if (numSpherical > 0)
		VERBOSE("warning: %u spherical image(s) in the pose quality report will not correlate with the "
			"cube-map faces produced by the MVS export (pose uncertainty is validated for pinhole cameras)",
			numSpherical);
	return numValid;
}
/*----------------------------------------------------------------*/

namespace {
// Set a parameter block constant only if it was actually added to the problem, returning
// whether it was. A pose/intrinsic/point block exists only when a residual referenced it:
// non-inlier tracks, spherical cameras (no intrinsic block), and observations skipped as
// low-confidence keypoints all leave their block unadded, and calling SetParameterBlockConstant
// on a missing block aborts the process via Ceres LOG(FATAL).
inline bool SetParameterBlockConstantIfPresent(ceres::Problem& problem, double* params) {
	if (!problem.HasParameterBlock(params))
		return false;
	problem.SetParameterBlockConstant(params);
	return true;
}

// Pinhole intrinsic parameter block layout: [fx, fy/fx, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6].
// Index 1 stores the aspect ratio fy/fx so focal length and aspect can be refined independently.
inline void ExtractPinholeIntrinsics(const PinholeCamera* cam, double* intr) {
	intr[0] = cam->fx;
	intr[1] = cam->fy / cam->fx;
	intr[2] = cam->cx;
	intr[3] = cam->cy;
	intr[4] = cam->k1;
	intr[5] = cam->k2;
	intr[6] = cam->k3;
	intr[7] = cam->p1;
	intr[8] = cam->p2;
	intr[9] = cam->k4;
	intr[10] = cam->k5;
	intr[11] = cam->k6;
}
inline void ApplyPinholeIntrinsics(const double* intr, PinholeCamera* cam) {
	cam->fx = static_cast<REAL>(intr[0]);
	cam->fy = cam->fx * static_cast<REAL>(intr[1]);
	cam->cx = static_cast<REAL>(intr[2]);
	cam->cy = static_cast<REAL>(intr[3]);
	cam->k1 = static_cast<REAL>(intr[4]);
	cam->k2 = static_cast<REAL>(intr[5]);
	cam->k3 = static_cast<REAL>(intr[6]);
	cam->p1 = static_cast<REAL>(intr[7]);
	cam->p2 = static_cast<REAL>(intr[8]);
	cam->k4 = static_cast<REAL>(intr[9]);
	cam->k5 = static_cast<REAL>(intr[10]);
	cam->k6 = static_cast<REAL>(intr[11]);
}
// Register img's pinhole camera in intrinsicParams (keyed by Camera*), initializing its
// 12-parameter block the first time the camera is seen. No-op for non-pinhole cameras.
inline void AddPinholeIntrinsics(std::unordered_map<const Camera*, DoubleArr>& intrinsicParams, const Image& img) {
	if (img.GetCameraType() != CameraType::PINHOLE)
		return;
	const auto it = intrinsicParams.emplace(img.pCamera, DoubleArr());
	if (!it.second)
		return; // already processed
	it.first->second.resize(12);
	ExtractPinholeIntrinsics(static_cast<const PinholeCamera*>(img.pCamera), it.first->second.data());
}

// Pick the (possibly confidence-scaled) loss for a keypoint observation. Returns false if the
// keypoint is below the confidence threshold and the observation should be skipped.
inline bool SelectReprojectionLoss(const BAConfig& config, const cv::KeyPoint& kp,
	ceres::LossFunction* baseLoss, ceres::LossFunction*& outLoss) {
	outLoss = baseLoss;
	if (!config.useKeypointConfidence)
		return true;
	const double weight = Image::ComputeKeypointPrecision(kp, config.minKeypointResponse);
	if (weight <= 0.0)
		return false; // skip low-confidence keypoint
	if (weight != 1.0)
		outLoss = new ceres::ScaledLoss(baseLoss, weight, ceres::DO_NOT_TAKE_OWNERSHIP);
	return true;
}

// Add a reprojection residual for keypoint kp of img, wiring its pose and point blocks (and,
// for pinhole cameras, the shared intrinsic block from intrinsicParams).
inline void AddReprojectionResidual(ceres::Problem& problem, ceres::LossFunction* loss,
	const Image& img, const cv::KeyPoint& kp, double* posePtr, double* pointPtr,
	std::unordered_map<const Camera*, DoubleArr>& intrinsicParams) {
	switch (img.GetCameraType()) {
	case CameraType::PINHOLE:
		problem.AddResidualBlock(
			#if 0
			new PinholeReprojectionErrorAnalytic(kp.pt.x, kp.pt.y),
			#else
			PinholeReprojectionError::Create(kp.pt.x, kp.pt.y),
			#endif
			loss,
			posePtr,                                  // Pose params
			intrinsicParams.at(img.pCamera).data(),   // Intrinsic params
			pointPtr);                                // Point params
		break;
	case CameraType::SPHERICAL:
		// Spherical error is already scaled to pixels and weighted inside the functor
		problem.AddResidualBlock(
			SphericalAngularReprojectionError::Create(kp.pt.x, kp.pt.y, img.pCamera->GetWidth(), img.pCamera->GetHeight()),
			loss,
			posePtr,     // Pose params
			pointPtr);   // Point params
		break;
	}
}

// Collect the constant indices of a 7-param pose block [qw,qx,qy,qz,Cx,Cy,Cz] for the given
// refinement flags: rotation occupies indices 0-3, position 4-6.
inline void CollectConstantPoseParams(const BAConfig& config, std::vector<int>& constantParams) {
	constantParams.clear();
	if (!config.refinePosesRotation) {
		constantParams.push_back(0);
		constantParams.push_back(1);
		constantParams.push_back(2);
		constantParams.push_back(3);
	}
	if (!config.refinePosesPosition) {
		constantParams.push_back(4);
		constantParams.push_back(5);
		constantParams.push_back(6);
	}
}

// Restrict a 7-param pose block to the non-constant subset given by constantParams (Ceres version-aware).
inline void SetPoseSubsetConstant(ceres::Problem& problem, double* pose, const std::vector<int>& constantParams) {
	#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
	problem.SetManifold(pose, new ceres::SubsetManifold(7, constantParams));
	#else
	problem.SetParameterization(pose, new ceres::SubsetParameterization(7, constantParams));
	#endif
}

// Create a fresh SE(3) manifold for a 7-param pose block (quaternion rotation + Euclidean
// translation, 6 DOF tangent space). Ceres takes ownership once it is attached to a block.
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
inline ceres::Manifold* CreateSE3PoseManifold() {
	return new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>{
		ceres::QuaternionManifold{}, ceres::EuclideanManifold<3>{}};
}
#else
inline ceres::LocalParameterization* CreateSE3PoseManifold() {
	auto* quaternion_param = new ceres::QuaternionParameterization;
	auto* identity_param = new ceres::IdentityParameterization(3);
	return new ceres::ProductParameterization(quaternion_param, identity_param);
}
#endif
} // namespace
/*----------------------------------------------------------------*/


BundleAdjustment::BundleAdjustment(Scene& _scene, const BAConfig& _config)
	: scene(_scene), config(_config)
{
}
BundleAdjustment::~BundleAdjustment() = default;

bool BundleAdjustment::Adjust()
{
	TD_TIMER_STARTD();

	// Count registered images (those with valid poses)
	IIndex nRegisteredImages = 0;
	for (const Image& img : scene.images)
		if (img.IsValid())
			++nRegisteredImages;
	const uint32_t nInlierTracks(scene.status.nTracks > 1000 ? scene.status.nTracks : scene.tracks.size());
	if (nRegisteredImages < 2 || nInlierTracks < 50) {
		VERBOSE("error: insufficient data for bundle adjustment");
		return false;
	}
	DEBUG_EXTRA("Bundle adjustment with %u cameras, %u images, %u tracks",
		scene.cameras.size(), nRegisteredImages, nInlierTracks);

	// Pose parameters: [qw, qx, qy, qz, Cx, Cy, Cz] x nImages
	poseParams.assign(scene.images.size() * 7, 0.0);
	FOREACH(i, scene.images)
		if (scene.images[i].IsValid())
			Pose3DToQuaternionAndCenter(scene.images[i], poseParams.data() + i * 7);

	// Intrinsic parameters: map unique cameras to parameter blocks (member: must outlive the
	// solve so the intrinsic blocks remain valid for post-Adjust covariance evaluation).
	intrinsicParams.clear();
	for (const Image& img : scene.images)
		if (img.IsValid())
			AddPinholeIntrinsics(intrinsicParams, img);

	// Build the Ceres problem as a member (kept alive past the solve so
	// ComputePoseUncertainty() can evaluate the Jacobian on the final state)
	this->problem = std::make_unique<ceres::Problem>();
	ceres::Problem& problem = *this->problem;
	// Use standard Huber loss (threshold in pixels)
	ceres::LossFunction* loss_function = config.robustThreshold > 0.f ?
		new ceres::HuberLoss(config.robustThreshold) : nullptr;

	// Set the SE(3) manifold on every valid pose block (shared instance; Ceres owns it once attached)
	auto* se3_manifold = CreateSE3PoseManifold();
	FOREACH(i, scene.images) {
		if (!scene.images[i].IsValid())
			continue;
		#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
		problem.AddParameterBlock(poseParams.data() + i * 7, 7, se3_manifold);
		#else
		problem.AddParameterBlock(poseParams.data() + i * 7, 7);
		problem.SetParameterization(poseParams.data() + i * 7, se3_manifold);
		#endif
	}

	// Add reprojection residuals
	uint32_t numReprojResiduals = 0;
	uint32_t numSkippedLowConfidence = 0;
	numReprojResidualsPerImage.resize(scene.images.size());
	numReprojResidualsPerImage.Memset(0);
	for (Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const auto& obs : track) {
			const IIndex imgID = obs.imageID;
			const Image& img = scene.images[imgID];
			if (!img.IsValid())
				continue;
			ASSERT(obs.featureID < img.keypoints.size());
			const cv::KeyPoint& kp = img.keypoints[obs.featureID];
			// Compute weight from keypoint response / size (if enabled)
			ceres::LossFunction* residual_loss_function;
			if (!SelectReprojectionLoss(config, kp, loss_function, residual_loss_function)) {
				++numSkippedLowConfidence;
				continue; // skip low-confidence keypoints
			}
			AddReprojectionResidual(problem, residual_loss_function, img, kp,
				poseParams.data() + imgID * 7, track.position.ptr(), intrinsicParams);
			++numReprojResidualsPerImage[imgID];
			++numReprojResiduals;
		}
	}
	if (config.useKeypointConfidence) {
		DEBUG_EXTRA("Created %u reprojection residuals (%u skipped low-confidence)",
		    numReprojResiduals, numSkippedLowConfidence);
	} else {
		DEBUG_EXTRA("Created %u reprojection residuals", numReprojResiduals);
	}

	// Set intrinsic parameter constraints (if refining intrinsics)
	if (config.IsRefiningIntrinsics() && !intrinsicParams.empty()) {
		// Build subset manifold for each camera based on refinement flags
		// Intrinsic layout: [fx, fy/fx, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6]
		std::vector<int> constantParams;
		constantParams.reserve(12);
		if (!config.refineFocalLength) {
			constantParams.push_back(0);  // fx
			constantParams.push_back(1);  // fy/fx
		} else if (!config.refineFocalLengthAspectRatio) {
			constantParams.push_back(1);  // fy/fx
		}
		if (!config.refinePrincipalPoint) {
			constantParams.push_back(2);  // cx
			constantParams.push_back(3);  // cy
		}
		if (!config.refineRadialDistortion123) {
			constantParams.push_back(4);  // k1
			constantParams.push_back(5);  // k2
			constantParams.push_back(6);  // k3
		}
		if (!config.refineTangentialDistortion) {
			constantParams.push_back(7);  // p1
			constantParams.push_back(8);  // p2
		}
		if (!config.refineRadialDistortion456) {
			constantParams.push_back(9);   // k4
			constantParams.push_back(10);  // k5
			constantParams.push_back(11);  // k6
		}
		std::vector<int> internConstantParams(constantParams);
		if (config.refineRadialDistortion456) {
			internConstantParams.push_back(9);   // k4
			internConstantParams.push_back(10);  // k5
			internConstantParams.push_back(11);  // k6
		}

		#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
		auto* intrinsicManifold = new ceres::SubsetManifold(12, constantParams);
		auto* internIntrinsicManifold = new ceres::SubsetManifold(12, internConstantParams);
		#else
		auto* intrinsicManifold = new ceres::SubsetParameterization(12, constantParams);
		auto* internIntrinsicManifold = new ceres::SubsetParameterization(12, internConstantParams);
		#endif
		bool bIntrinsicManifoldUsed = false;
		bool bInternIntrinsicManifoldUsed = false;
		for (auto& pair : intrinsicParams) {
			ASSERT(!pair.second.empty());
			// Skip cameras whose intrinsic block was never added to the problem (no
			// PINHOLE residual referenced it); SetManifold would otherwise LOG(FATAL).
			if (!problem.HasParameterBlock(pair.second.data()))
				continue;
			auto intrManifold = (pair.first->GetType() == CameraType::PINHOLE && !static_cast<const PinholeCamera*>(pair.first)->useAdditionalDistortion ?
				internIntrinsicManifold : intrinsicManifold);
			if (intrManifold == intrinsicManifold)
				bIntrinsicManifoldUsed = true;
			else
				bInternIntrinsicManifoldUsed = true;
			#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
			problem.SetManifold(pair.second.data(), intrManifold);
			#else
			problem.SetParameterization(pair.second.data(), intrManifold);
			#endif
		}
		if (!bIntrinsicManifoldUsed)
			delete intrinsicManifold;
		if (!bInternIntrinsicManifoldUsed)
			delete internIntrinsicManifold;
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (internConstantParams.empty() || !bInternIntrinsicManifoldUsed) {
			DEBUG("Intrinsic parameters refined");
		} else {
			std::string paramStr;
			FOREACH(i, internConstantParams) {
				if (i > 0) paramStr += ", ";
				paramStr += std::to_string(internConstantParams[i]);
			}
			DEBUG("Fixed intrinsic parameters: %s", paramStr.c_str());
		}
		#endif
	} else if (!intrinsicParams.empty()) {
		// Not refining intrinsics: set all intrinsic blocks constant
		for (auto& pair : intrinsicParams) {
			ASSERT(!pair.second.empty());
			SetParameterBlockConstantIfPresent(problem, pair.second.data());
		}
		DEBUG("Fixed all intrinsic parameters");
	}

	// Add GPS position constraints (if enabled)
	numGPSResiduals = 0;
	if (config.IsRefiningGPS() && scene.status.nState.isSet(Scene::Status::STATE::GEO_ALIGN)) {
		// Estimate median distance from tracks
		DoubleArr distances;
		distances.reserve(scene.tracks.size());
		for (const Track& track : scene.tracks) {
			if (!track.IsInlier())
				continue;
			for (const auto& obs : track) {
				ASSERT(obs.imageID < scene.images.size());
				const Image& img = scene.images[obs.imageID];
				ASSERT(img.IsValid());
				double dist = norm(track.position - img.C);
				if (dist > 0.1) // filter out degenerate points
					distances.push_back(dist);
				break; // only need one observation per track
			}
		}
		// Compute scene scale for unit-aware weighting
		ASSERT(!distances.empty());
		const double median_depth = distances.GetMedian();
		// Estimate median focal length
		DoubleArr focals;
		for (const Image& img : scene.images) {
			if (img.IsValid() && img.GetCameraType() == CameraType::PINHOLE) {
				const PinholeCamera* pc = dynamic_cast<const PinholeCamera*>(img.pCamera);
				focals.push_back((pc->fx + pc->fy) / 2.0);
			}
		}
		double median_focal = 1.0; // default fallback
		if (!focals.empty())
			median_focal = focals.GetMedian();
		// Compute pixel-to-meter scale
		const double pixel_scale = median_depth / median_focal;
		const double weight_h_scaled = SQRT(config.gpsPositionWeight * config.gpsWeightScaleFactor * pixel_scale);
		const double weight_v_scaled = SQRT(config.gpsPositionWeightZ * config.gpsWeightScaleFactor * pixel_scale);
		DEBUG_EXTRA("GPS weight scaling: median_depth %.2f m, median_focal %.1f px, pixel_scale %.4f m/px",
		    median_depth, median_focal, pixel_scale);
		DEBUG_EXTRA("Effective GPS weights: horizontal %.4f, vertical %.4f", weight_h_scaled, weight_v_scaled);
		// Collect GPS observations and create GPS residuals
		const Point3d centerECEF = scene.GetCenterECEF();
		double lat0, lon0, alt0;
		ECEFToWGS84(centerECEF.x, centerECEF.y, centerECEF.z, lat0, lon0, alt0);
		FOREACH(i, scene.images) {
			const Image& img = scene.images[i];
			if (!img.IsValid())
				continue;
			const View::Metadata& meta = img.View::metadata;
			if (!meta.HasGPS())
				continue;
			double enu_east, enu_north, enu_up;
			WGS84ToENU(meta.latitude, meta.longitude, meta.altitude,
						lat0, lon0, alt0,
						enu_east, enu_north, enu_up);
			// Create GPS residual
			// Camera coordinate convention: X=East, Y=North, Z=Up (adjust if scene uses different convention)
			// EXIF GPS frequently lacks accuracy tags; the residual divides by the accuracy,
			// so substitute typical consumer-GPS accuracies when unknown
			const double accuracyH = meta.positionAccuracy > 0.f ? meta.positionAccuracy : 10.0;
			const double accuracyV = meta.positionAccuracyZ > 0.f ? meta.positionAccuracyZ : 20.0;
			ceres::CostFunction* gps_cost = GPSPositionError::Create(
				enu_east, enu_north, enu_up,
				accuracyH, accuracyV,
				weight_h_scaled, weight_v_scaled
			);
			problem.AddResidualBlock(
				gps_cost,
				nullptr, // No robust loss for GPS (already weighted by accuracy)
				poseParams.data() + i * 7
			);
			++numGPSResiduals;
		}
		DEBUG("Added %u GPS position constraints (origin: lat=%.6f°, lon=%.6f°, alt=%.1fm)",
			numGPSResiduals, lat0, lon0, alt0);
	}

	// Fix best connected camera (gauge freedom) - unless we have GPS constraints
	if (numGPSResiduals == 0) {
		IIndex bestImgID = NO_ID;
		FOREACH(i, scene.images) {
			if (!scene.images[i].IsValid())
				continue;
			if (bestImgID == NO_ID || numReprojResidualsPerImage[bestImgID] < numReprojResidualsPerImage[i])
				bestImgID = i;
		}
		if (bestImgID != NO_ID) {
			problem.SetParameterBlockConstant(poseParams.data() + bestImgID * 7);
			DEBUG("Fixed view %u (reference, no GPS)", bestImgID);
		}
	}

	// Optionally disable pose/point refinement
	if (!config.IsRefiningPoses()) {
		// Disable all pose refinement
		FOREACH(i, scene.images)
			if (scene.images[i].IsValid())
				problem.SetParameterBlockConstant(poseParams.data() + i * 7);
		DEBUG("Views poses: FIXED");
	} else if (!config.refinePosesRotation || !config.refinePosesPosition) {
		// Selectively disable rotation and/or position refinement
		std::vector<int> constantParams;
		CollectConstantPoseParams(config, constantParams);
		FOREACH(i, scene.images)
			if (scene.images[i].IsValid())
				SetPoseSubsetConstant(problem, poseParams.data() + i * 7, constantParams);
		DEBUG("Views poses: rotation=%s, position=%s",
		      config.refinePosesRotation ? "OPTIMIZED" : "FIXED",
		      config.refinePosesPosition ? "OPTIMIZED" : "FIXED");
	}
	if (!config.refinePoints) {
		// Disable all point refinement (a point block exists only if a residual referenced it;
		// non-inlier tracks and tracks whose observations were all skipped are never added)
		for (Track& track : scene.tracks)
			SetParameterBlockConstantIfPresent(problem, track.position.ptr());
		DEBUG("3D points: FIXED");
	}

	// Configure solver
	ceres::Solver::Options options;
	if (numReprojResiduals < 500000) {
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.preconditioner_type = ceres::IDENTITY; // Not used with DENSE_SCHUR
	} else {
		// For large problems, use SPARSE_SCHUR or ITERATIVE_SCHUR
		// Use ITERATIVE_SCHUR for better numerical stability, especially on macOS Apple Accelerate
		// SPARSE_SCHUR can fail with "Numeric factorisation failed" on poorly conditioned problems
		options.linear_solver_type = ceres::ITERATIVE_SCHUR;
		options.preconditioner_type = ceres::SCHUR_JACOBI; // Robust preconditioner
		options.use_inner_iterations = true; // Improves convergence
		#if 0 && (CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2))
		// DISABLED: Power Bundle Adjustment (Weber et al., CVPR 2022) via the
		// SCHUR_POWER_SERIES_EXPANSION preconditioner, gated on a large camera count (the reduced
		// camera system it is meant to accelerate). Benchmarked against the SCHUR_JACOBI default and
		// it loses at every scale tested, so it is left off. On an i7-13700KF (16C/24T) / RTX 4070 /
		// 32GB / Win11, Ceres 2.2.0 + CUDA 13.0: House (83 cameras) ran 1.2-3.9x slower; Tanks&Temples
		// Courthouse (1106 cameras) ran 1.6-1.7x slower on the 4-5.6M-residual bundles and HUNG for
		// >81 min on a 6.3M-residual bundle (never converged), while SCHUR_JACOBI completed the whole
		// reconstruction in ~54 min. CG convergence was erratic (non-monotonic in problem size).
		// Re-enable/re-tune (e.g. without use_spse_initialization) only with a fresh benchmark on a
		// scene with far more cameras than we had available.
		if (scene.status.nCalibratedImages > 1000) {
			options.preconditioner_type = ceres::SCHUR_POWER_SERIES_EXPANSION;
			options.use_spse_initialization = true;
		}
		#endif
	}
	#ifndef _RELEASE
	options.minimizer_progress_to_stdout = true;
	#else
	options.minimizer_progress_to_stdout = false;
	#endif
	options.max_num_iterations = config.maxIterations;
	// numThreads 0 = auto; either way stay within the scene's thread budget, as
	// clustered sub-scenes solve concurrently
	options.num_threads = (int)MINF(config.numThreads > 0 ? config.numThreads : std::thread::hardware_concurrency(), scene.nMaxThreads);
	options.function_tolerance = config.functionTolerance;

	// Solve
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	DEBUG("BA Summary: %s", summary.BriefReport().c_str());
	if (!summary.IsSolutionUsable()) {
		VERBOSE("error: bundle adjustment failed");
		this->problem.reset(); // no valid solution to estimate uncertainty from
		return false;
	}

	// Update scene with optimized parameters
	FOREACH(i, scene.images)
		if (scene.images[i].IsValid())
			QuaternionAndCenterToPose3D(poseParams.data() + i * 7, scene.images[i]);

	// Update camera intrinsics if refined
	if (config.IsRefiningIntrinsics() && !intrinsicParams.empty()) {
		for (auto& pair : intrinsicParams) {
			PinholeCamera* pinholeCamera = dynamic_cast<PinholeCamera*>(const_cast<Camera*>(pair.first));
			if (!pinholeCamera)
				continue;
			ApplyPinholeIntrinsics(pair.second.data(), pinholeCamera);
			DEBUG_EXTRA("Camera intrinsics updated: %s", pinholeCamera->GetIntrinsicsString().c_str());
		}
		DEBUG("Updated intrinsics for %u cameras", (unsigned)intrinsicParams.size());
	}

	DEBUG("Bundle adjustment complete: %u reprojection residuals, %u GPS residuals, %.4g -> %.4g cost (%s)",
	    numReprojResiduals, numGPSResiduals, summary.initial_cost, summary.final_cost, TD_TIMER_GET_FMT().c_str());

	// Report average reprojection errors
	ComputeTracksMeanReprojectionError(scene);
	return true;
}

bool BundleAdjustment::AdjustLocal(
	const IIndexArr& viewIDs,
	const IIndexArr& fixedViewIDs)
{
	TD_TIMER_STARTD();
	numGPSResiduals = 0; // local BA adds no GPS priors

	// 1. Set local window
	ASSERT(!viewIDs.empty());
	const std::unordered_set<IIndex> localImages(viewIDs.begin(), viewIDs.end());
	const std::unordered_set<IIndex> fixedImages(fixedViewIDs.begin(), fixedViewIDs.end());
	const IIndexArr allImages(viewIDs + fixedViewIDs);

	// 2. Collect relevant points (observed by at least one local image)
	std::vector<uint32_t> activePoints;
	activePoints.reserve(scene.tracks.size() / 10); // heuristic
	FOREACH(i, scene.tracks) {
		const Track& track = scene.tracks[i];
		if (!track.IsInlier())
			continue;
		for (const Observation& obs : track) {
			if (localImages.find(obs.imageID) != localImages.end()) {
				activePoints.push_back(i);
				break;
			}
		}
	}
	if (activePoints.empty()) {
		VERBOSE("warning: no points in local window");
		return true;
	}
	DEBUG_EXTRA("Local bundle adjustment with %u cameras, %u (%u local, %u fixed) images, %u tracks",
		scene.cameras.size(), allImages.size(), viewIDs.size(), fixedViewIDs.size(), (unsigned)activePoints.size());

	// Pose parameters: [qw, qx, qy, qz, Cx, Cy, Cz] x nImages, indexed by image ID like global
	// BA. Only window images (local + fixed) receive a parameter block; the flat layout lets
	// ComputePoseUncertainty() read the solved pose blocks with the same imageID*7 addressing.
	poseParams.assign(scene.images.size() * 7, 0.0);
	for (IIndex imgID : allImages) {
		ASSERT(scene.images[imgID].IsValid());
		Pose3DToQuaternionAndCenter(scene.images[imgID], poseParams.data() + imgID * 7);
	}

	// Intrinsic parameters: always fixed in local BA (not refined). Member storage so the
	// intrinsic blocks outlive the solve for post-Adjust covariance evaluation.
	intrinsicParams.clear();
	for (IIndex imgID : allImages)
		if (scene.images[imgID].IsValid())
			AddPinholeIntrinsics(intrinsicParams, scene.images[imgID]);

	// 3. Build the Ceres problem as a member (kept alive past the solve so
	// ComputePoseUncertainty() can evaluate the Jacobian on the final state)
	this->problem = std::make_unique<ceres::Problem>();
	ceres::Problem& problem = *this->problem;
	// Use standard Huber loss (threshold in pixels)
	ceres::LossFunction* loss_function = config.robustThreshold > 0.f ?
		new ceres::HuberLoss(config.robustThreshold) : nullptr;

	// Add reprojection residuals (only observations from window images: local or fixed)
	uint32_t numReprojResiduals = 0;
	numReprojResidualsPerImage.resize(scene.images.size());
	numReprojResidualsPerImage.Memset(0);
	for (const IIndex pointID : activePoints) {
		Track& track = scene.tracks[pointID];
		ASSERT(track.IsInlier());
		for (const Observation& obs : track) {
			const IIndex imgID = obs.imageID;
			// Only consider observations in local or fixed images
			if (localImages.find(imgID) == localImages.end() &&
			    fixedImages.find(imgID) == fixedImages.end())
				continue;
			const Image& img = scene.images[imgID];
			ASSERT(obs.featureID < img.keypoints.size());
			const cv::KeyPoint& kp = img.keypoints[obs.featureID];
			// Compute weight from keypoint response / size (if enabled)
			ceres::LossFunction* residual_loss_function;
			if (!SelectReprojectionLoss(config, kp, loss_function, residual_loss_function))
				continue; // skip low-confidence keypoints
			AddReprojectionResidual(problem, residual_loss_function, img, kp,
				poseParams.data() + imgID * 7, track.position.ptr(), intrinsicParams);
			++numReprojResidualsPerImage[imgID];
			++numReprojResiduals;
		}
	}

	// Set the SE(3) manifold on every pose block that was actually added to the problem.
	// Ceres takes ownership of the manifold only once it is attached to a block, so if no
	// pose block exists (all observations skipped) we must free it ourselves to avoid a leak.
	auto* se3_manifold = CreateSE3PoseManifold();
	bool poseManifoldUsed = false;
	for (IIndex imgID : allImages) {
		double* pose = poseParams.data() + imgID * 7;
		if (problem.HasParameterBlock(pose)) {
			#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
			problem.SetManifold(pose, se3_manifold);
			#else
			problem.SetParameterization(pose, se3_manifold);
			#endif
			poseManifoldUsed = true;
		}
	}
	if (!poseManifoldUsed)
		delete se3_manifold;

	// 4. Set fixed parameters (only for blocks that were actually added via a residual)
	if (!intrinsicParams.empty()) {
		for (auto& pair : intrinsicParams) {
			ASSERT(!pair.second.empty());
			SetParameterBlockConstantIfPresent(problem, pair.second.data());
		}
		DEBUG("Fixed all intrinsic parameters");
	}

	// Fixed images
	bool bFixedAny = false;
	for (IIndex imgID : fixedViewIDs)
		if (SetParameterBlockConstantIfPresent(problem, poseParams.data() + imgID * 7))
			bFixedAny = true;

	// Fix best-connected local camera if no fixed images (gauge freedom)
	if (!bFixedAny) {
		IIndex bestImgID = NO_ID;
		for (IIndex imgID : viewIDs) {
			if (!problem.HasParameterBlock(poseParams.data() + imgID * 7))
				continue;
			if (bestImgID == NO_ID || numReprojResidualsPerImage[bestImgID] < numReprojResidualsPerImage[imgID])
				bestImgID = imgID;
		}
		if (bestImgID != NO_ID) {
			problem.SetParameterBlockConstant(poseParams.data() + bestImgID * 7);
			VERBOSE("Fixed reference camera %u (local BA)", bestImgID);
		}
	}

	// Optionally disable pose/point refinement
	if (!config.IsRefiningPoses()) {
		for (IIndex imgID : allImages)
			SetParameterBlockConstantIfPresent(problem, poseParams.data() + imgID * 7);
		DEBUG("Views poses (local BA): FIXED");
	} else if (!config.refinePosesRotation || !config.refinePosesPosition) {
		// Selectively disable rotation and/or position refinement
		std::vector<int> constantParams;
		CollectConstantPoseParams(config, constantParams);
		for (IIndex imgID : allImages) {
			double* pose = poseParams.data() + imgID * 7;
			if (problem.HasParameterBlock(pose))
				SetPoseSubsetConstant(problem, pose, constantParams);
		}
		DEBUG("Views poses (local BA): rotation=%s, position=%s",
		      config.refinePosesRotation ? "OPTIMIZED" : "FIXED",
		      config.refinePosesPosition ? "OPTIMIZED" : "FIXED");
	}
	if (!config.refinePoints) {
		// Fix all active points (a point block exists only if a residual referenced it;
		// an active point can have all its in-window observations skipped, e.g. as
		// low-confidence keypoints, leaving its block unadded)
		for (uint32_t pointID : activePoints)
			SetParameterBlockConstantIfPresent(problem, scene.tracks[pointID].position.ptr());
		DEBUG("3D points (local BA): FIXED");
	}

	// Solve
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	#ifndef _RELEASE
	options.minimizer_progress_to_stdout = true;
	#else
	options.minimizer_progress_to_stdout = false;
	#endif
	options.max_num_iterations = config.maxIterations;
	// numThreads 0 = auto; either way stay within the scene's thread budget, as
	// clustered sub-scenes solve concurrently
	options.num_threads = (int)MINF(config.numThreads > 0 ? config.numThreads : std::thread::hardware_concurrency(), scene.nMaxThreads);
	options.function_tolerance = config.functionTolerance;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	DEBUG("Local BA Summary: %s", summary.BriefReport().c_str());
	if (!summary.IsSolutionUsable()) {
		VERBOSE("error: local bundle adjustment failed");
		this->problem.reset(); // no valid solution to estimate uncertainty from
		return false;
	}

	// 5. Update scene (only local images; fixed images stay constant)
	for (IIndex imgID : viewIDs)
		QuaternionAndCenterToPose3D(poseParams.data() + imgID * 7, scene.images[imgID]);

	DEBUG("Local bundle adjustment complete: %u reprojection residuals, %.4g -> %.4g cost (%s)",
	    numReprojResiduals, summary.initial_cost, summary.final_cost, TD_TIMER_GET_FMT().c_str());

	// Report average reprojection errors for local window
	ComputeTracksMeanReprojectionError(scene);
	return true;
}
/*----------------------------------------------------------------*/


bool SFM::PinholeReprojectionJacobianTest()
{
	TD_TIMER_START();
	VERBOSE("\n--- Testing PinholeReprojectionErrorAnalytic Jacobians ---");

	// Create synthetic test data
	const double observed_x = 320.5;
	const double observed_y = 240.7;

	// Test parameters
	double pose[7] = {0.1, 0.2, 0.05, 0.97, 1.0, 0.5, 3.0}; // quat + center
	double intrinsics[12] = {500.0, 1.0, 320.0, 240.0, 0.1, -0.05, 0.01, 0.001, -0.001, 0.0, 0.0, 0.0};
	double point[3] = {2.0, 1.5, 5.0};

	// Normalize quaternion
	Eigen::Map<Eigen::Vector4d>(pose).normalize();

	// Evaluate analytic cost function
	PinholeReprojectionErrorAnalytic analytic_cost(observed_x, observed_y);
	double analytic_residuals[2];
	double* analytic_jacobians[3];
	double analytic_J_pose[2*7];
	double analytic_J_intrinsics[2*12];
	double analytic_J_point[2*3];
	analytic_jacobians[0] = analytic_J_pose;
	analytic_jacobians[1] = analytic_J_intrinsics;
	analytic_jacobians[2] = analytic_J_point;
	const double* params[3] = {pose, intrinsics, point};
	const double* const* params_const = params;
	if (!analytic_cost.Evaluate(params_const, analytic_residuals, analytic_jacobians)) {
		VERBOSE("FAILED: Analytic cost evaluation failed");
		return false;
	}

	// Evaluate auto-diff cost function for comparison
	std::unique_ptr<ceres::CostFunction> autodiff_cost(PinholeReprojectionError::Create(observed_x, observed_y));
	double autodiff_residuals[2];
	double* autodiff_jacobians[3];
	double autodiff_J_pose[2*7];
	double autodiff_J_intrinsics[2*12];
	double autodiff_J_point[2*3];
	autodiff_jacobians[0] = autodiff_J_pose;
	autodiff_jacobians[1] = autodiff_J_intrinsics;
	autodiff_jacobians[2] = autodiff_J_point;
	if (!autodiff_cost->Evaluate(params_const, autodiff_residuals, autodiff_jacobians)) {
		VERBOSE("FAILED: Auto-diff cost evaluation failed");
		return false;
	}

	// Compute numeric Jacobians using finite differences
	const double epsilon = 1e-8;
	double numeric_J_pose[2*7];
	double numeric_J_intrinsics[2*12];
	double numeric_J_point[2*3];

	// Jacobian w.r.t. pose (7 params)
	for (int i = 0; i < 7; ++i) {
		double pose_plus[7], pose_minus[7];
		std::memcpy(pose_plus, pose, 7 * sizeof(double));
		std::memcpy(pose_minus, pose, 7 * sizeof(double));
		pose_plus[i] += epsilon;
		pose_minus[i] -= epsilon;

		// Renormalize quaternion if perturbing quaternion components
		if (i < 4) {
			Eigen::Map<Eigen::Vector4d>(pose_plus).normalize();
			Eigen::Map<Eigen::Vector4d>(pose_minus).normalize();
		}

		double res_plus[2], res_minus[2];
		const double* params_plus[3] = {pose_plus, intrinsics, point};
		const double* params_minus[3] = {pose_minus, intrinsics, point};
		analytic_cost.Evaluate(params_plus, res_plus, nullptr);
		analytic_cost.Evaluate(params_minus, res_minus, nullptr);

		numeric_J_pose[0*7 + i] = (res_plus[0] - res_minus[0]) / (2.0 * epsilon);
		numeric_J_pose[1*7 + i] = (res_plus[1] - res_minus[1]) / (2.0 * epsilon);
	}

	// Jacobian w.r.t. intrinsics (12 params)
	for (int i = 0; i < 12; ++i) {
		double intr_plus[12], intr_minus[12];
		std::memcpy(intr_plus, intrinsics, 12 * sizeof(double));
		std::memcpy(intr_minus, intrinsics, 12 * sizeof(double));
		intr_plus[i] += epsilon;
		intr_minus[i] -= epsilon;

		double res_plus[2], res_minus[2];
		const double* params_plus[3] = {pose, intr_plus, point};
		const double* params_minus[3] = {pose, intr_minus, point};
		analytic_cost.Evaluate(params_plus, res_plus, nullptr);
		analytic_cost.Evaluate(params_minus, res_minus, nullptr);

		numeric_J_intrinsics[0*12 + i] = (res_plus[0] - res_minus[0]) / (2.0 * epsilon);
		numeric_J_intrinsics[1*12 + i] = (res_plus[1] - res_minus[1]) / (2.0 * epsilon);
	}

	// Jacobian w.r.t. point (3 params)
	for (int i = 0; i < 3; ++i) {
		double point_plus[3], point_minus[3];
		std::memcpy(point_plus, point, 3 * sizeof(double));
		std::memcpy(point_minus, point, 3 * sizeof(double));
		point_plus[i] += epsilon;
		point_minus[i] -= epsilon;

		double res_plus[2], res_minus[2];
		const double* params_plus[3] = {pose, intrinsics, point_plus};
		const double* params_minus[3] = {pose, intrinsics, point_minus};
		analytic_cost.Evaluate(params_plus, res_plus, nullptr);
		analytic_cost.Evaluate(params_minus, res_minus, nullptr);

		numeric_J_point[0*3 + i] = (res_plus[0] - res_minus[0]) / (2.0 * epsilon);
		numeric_J_point[1*3 + i] = (res_plus[1] - res_minus[1]) / (2.0 * epsilon);
	}

	// Compare Jacobians (analytic vs numeric vs auto-diff)
	const double jacobian_tol = 2.2e-5; // Tolerance for manifold-aware numerical differentiation
	double max_diff_numeric = 0.0;
	double max_diff_autodiff = 0.0;

	// Check pose Jacobian (2x7)
	for (int i = 0; i < 2; ++i) {
		// Project quaternion part of autodiff Jacobian [i*7, i*7+4) onto tangent space
		// to match manifold-aware derivatives (numeric/analytic)
		double dot = 0.0;
		for (int k = 0; k < 4; ++k) dot += autodiff_J_pose[i*7 + k] * pose[k];
		for (int k = 0; k < 4; ++k) autodiff_J_pose[i*7 + k] -= dot * pose[k];

		for (int j = 0; j < 7; ++j) {
			const int idx = i*7 + j;
			const double diff_numeric = ABS(analytic_J_pose[idx] - numeric_J_pose[idx]);
			const double diff_autodiff = ABS(analytic_J_pose[idx] - autodiff_J_pose[idx]);
			max_diff_numeric = MAX(max_diff_numeric, diff_numeric);
			max_diff_autodiff = MAX(max_diff_autodiff, diff_autodiff);
			if (diff_numeric > jacobian_tol) {
				VERBOSE("FAILED: Pose Jacobian[%d] mismatch (numeric): analytic=%.6e, numeric=%.6e, diff=%.6e",
				        idx, analytic_J_pose[idx], numeric_J_pose[idx], diff_numeric);
				return false;
			}
			if (diff_autodiff > jacobian_tol) {
				VERBOSE("FAILED: Pose Jacobian[%d] mismatch (auto-diff): analytic=%.6e, autodiff=%.6e, diff=%.6e",
				        idx, analytic_J_pose[idx], autodiff_J_pose[idx], diff_autodiff);
				return false;
			}
		}
	}

	// Check intrinsics Jacobian (2x12)
	for (int i = 0; i < 2*12; ++i) {
		const double diff_numeric = ABS(analytic_J_intrinsics[i] - numeric_J_intrinsics[i]);
		const double diff_autodiff = ABS(analytic_J_intrinsics[i] - autodiff_J_intrinsics[i]);
		max_diff_numeric = MAX(max_diff_numeric, diff_numeric);
		max_diff_autodiff = MAX(max_diff_autodiff, diff_autodiff);
		if (diff_numeric > jacobian_tol) {
			VERBOSE("FAILED: Intrinsics Jacobian[%d] mismatch (numeric): analytic=%.6e, numeric=%.6e, diff=%.6e",
			        i, analytic_J_intrinsics[i], numeric_J_intrinsics[i], diff_numeric);
			return false;
		}
		if (diff_autodiff > jacobian_tol) {
			VERBOSE("FAILED: Intrinsics Jacobian[%d] mismatch (auto-diff): analytic=%.6e, autodiff=%.6e, diff=%.6e",
			        i, analytic_J_intrinsics[i], autodiff_J_intrinsics[i], diff_autodiff);
			return false;
		}
	}

	// Check point Jacobian (2x3)
	for (int i = 0; i < 2*3; ++i) {
		const double diff_numeric = ABS(analytic_J_point[i] - numeric_J_point[i]);
		const double diff_autodiff = ABS(analytic_J_point[i] - autodiff_J_point[i]);
		max_diff_numeric = MAX(max_diff_numeric, diff_numeric);
		max_diff_autodiff = MAX(max_diff_autodiff, diff_autodiff);
		if (diff_numeric > jacobian_tol) {
			VERBOSE("FAILED: Point Jacobian[%d] mismatch (numeric): analytic=%.6e, numeric=%.6e, diff=%.6e",
			        i, analytic_J_point[i], numeric_J_point[i], diff_numeric);
			return false;
		}
		if (diff_autodiff > jacobian_tol) {
			VERBOSE("FAILED: Point Jacobian[%d] mismatch (auto-diff): analytic=%.6e, autodiff=%.6e, diff=%.6e",
			        i, analytic_J_point[i], autodiff_J_point[i], diff_autodiff);
			return false;
		}
	}

	VERBOSE("PASSED: All Jacobians match within tolerance (numeric max diff=%.2e, auto-diff max diff=%.2e) %s",
	        max_diff_numeric, max_diff_autodiff, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/
