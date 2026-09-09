/*
 * BundleAdjustment.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

// Include Eigen before OpenCV to avoid header ordering issues
#include "Common.h"
#include "BundleAdjustment.h"
#include "Scene.h"
#include "PoseLink.h"
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

// 1/k^2 for k the ratio of the two populations' robust reprojection sigmas, measured on the scene
// this solve is about to fit. Both sigmas are read off the RAW pixel residuals, so the estimate does
// not carry the weighting the previous solve ran under, and it is recomputed at the head of every
// solve -- a reconstruction runs fifty or more of them, so it settles.
//
// Measured rather than configured because k is a property of the CAPTURE, not of the dense matcher:
// across different captures the dense sigma barely moved (1.14-1.79 px, the warp's sampling
// scale) while the described sigma moved 3.6x, from 0.32 px where the texture is rich to 1.15 px
// where it is not, taking k from 1.55 to 4.02. No constant is right in both places.
//
// Clamped to [MIN_DENSE_OBSERVATION_WEIGHT, 1]: a dense correspondence is never a MORE precise
// measurement than a described one, and never worth nothing -- on a capture where the descriptor
// matcher is very good the ratio can run away, and a weight of zero would discard the only evidence
// a textureless region has. Falls back to the fixed DENSE_OBSERVATION_WEIGHT when either population
// is under MIN_SIGMA_OBSERVATIONS -- the honest answer for an early incremental step whose scene is a
// handful of tracks -- or when either sigma is zero, which is what a scene that fits itself exactly
// (a synthetic one, or a solve that has converged onto its own measurements) would divide by.
constexpr size_t MIN_SIGMA_OBSERVATIONS = 100;
constexpr double MIN_DENSE_OBSERVATION_WEIGHT = 0.01;

double SFM::EstimateDenseObservationWeight(const Scene& scene, const BAConfig& config,
	DenseObservationSigmas* sigmas)
{
	if (sigmas)
		*sigmas = DenseObservationSigmas();
	if (config.denseObservationWeight >= 0.0)
		return config.denseObservationWeight;
	// nothing to weight means nothing to measure, and this is the shipped default: --roma2-match is
	// off, so a descriptor-only reconstruction carries no dense keypoint and SelectReprojectionLoss
	// never multiplies by the weight. The test is O(1) per image, against the full reprojection of
	// every observation of every inlier track it saves -- once per Adjust(), and once per local BA,
	// which incremental reconstruction runs once per registered image.
	if (!std::any_of(scene.images.begin(), scene.images.end(),
			[](const Image& img) { return img.HasDenseKeypoints(); }))
		return DENSE_OBSERVATION_WEIGHT;
	double sigmaDescribed = 0, sigmaDense = 0;
	size_t numDescribed = 0, numDense = 0;
	ComputeObservationSigmas(scene, sigmaDescribed, numDescribed, sigmaDense, numDense);
	if (sigmas)
		*sigmas = DenseObservationSigmas{sigmaDescribed, sigmaDense, numDescribed, numDense, false};
	if (numDescribed < MIN_SIGMA_OBSERVATIONS || numDense < MIN_SIGMA_OBSERVATIONS ||
		sigmaDescribed <= 0.0 || sigmaDense <= 0.0)
		return DENSE_OBSERVATION_WEIGHT;
	if (sigmas)
		sigmas->measured = true;
	const double k = sigmaDense/sigmaDescribed;
	return CLAMP(1.0/(k*k), MIN_DENSE_OBSERVATION_WEIGHT, 1.0);
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

// Which dense (descriptor-less) observations of an over-cap image a solve keeps.
//
// A dense-matched image contributes thousands of warp-sampled observations against a few hundred
// described ones, and each of them is the less precise measurement of the two (see
// SelectReprojectionLoss): together they set the cost of the solve while adding little to what it
// determines. Capping their number per image is what makes that cost bounded, and WHICH of them
// survive decides whether the cap is free: an even cover of the frame constrains the image's pose
// as the full set did, while an arbitrary prefix of it would leave whole regions of the image
// unmeasured and let the pose rotate into them.
//
// Empty = every observation is kept (no image is over the cap); otherwise one flag per dense
// keypoint of every image that carries any, indexed by the keypoint's offset into that image's
// dense suffix (Image.h) -- an under-cap image gets its flags too, all set, since the pass that
// holds a track's views together may still have to drop one of them. Described observations are
// never dropped and never consulted.
class DenseObservationCap
{
public:
	// Whether the solve keeps the observation of keypoint featureID in image imgID
	inline bool Keeps(IIndex imgID, const Image& img, uint32_t featureID) const {
		if (keep.empty())
			return true;
		const std::vector<bool>& imgKeep = keep[imgID];
		if (imgKeep.empty() || !img.IsDenseKeypoint(featureID))
			return true;
		ASSERT(featureID >= numDescribed[imgID]);
		return imgKeep[featureID - numDescribed[imgID]];
	}

	std::vector<std::vector<bool>> keep; // per image, per dense keypoint
	std::vector<uint32_t> numDescribed;  // per image, where its dense suffix starts
};

// Grid cell of a keypoint in an image split into gridSize x gridSize cells
inline uint32_t DenseObservationCell(const Image& img, const cv::KeyPoint& kp, unsigned gridSize) {
	const int width = img.pCamera->GetWidth(), height = img.pCamera->GetHeight();
	if (width <= 0 || height <= 0)
		return 0; // no image size to spread over: one cell, so the track length alone orders them
	const unsigned x = MINF((unsigned)MAXF(kp.pt.x*gridSize/width, 0.f), gridSize-1);
	const unsigned y = MINF((unsigned)MAXF(kp.pt.y*gridSize/height, 0.f), gridSize-1);
	return y*gridSize + x;
}

// Decide which dense observations of each over-cap image take part in the solve: bucket the
// image's dense observations on a sqrt(cap) x sqrt(cap) grid scaled to the image and take them
// round-robin across the cells until the cap is reached, longest track first inside a cell. The
// round-robin is what spreads the survivors over the frame; the track length orders them inside a
// cell because an observation of a point many images see is the one that ties them together.
//
// visitTracks(fn) calls fn(track) for every track the solve builds residuals from and
// inSolve(imageID) answers whether an observation's image takes part: both mirror the residual
// loop, whose filtering this pre-pass reproduces. Returns how many observations were dropped.
template <typename TVisitTracks, typename TInSolve>
uint32_t BuildDenseObservationCap(const Scene& scene, unsigned cap,
	const TVisitTracks& visitTracks, const TInSolve& inSolve, DenseObservationCap& denseCap)
{
	denseCap.keep.clear();
	denseCap.numDescribed.clear();
	if (cap == 0)
		return 0; // uncapped
	// how many dense observations every image contributes, so that only the images actually over
	// the cap are given a decision
	const IIndex numImages = scene.images.size();
	std::vector<uint32_t> numDense(numImages, 0);
	const auto forEachDense = [&](const auto& fn) {
		visitTracks([&](const Track& track) {
			for (const Observation& obs : track) {
				if (!inSolve(obs.imageID))
					continue;
				const Image& img = scene.images[obs.imageID];
				if (img.IsDenseKeypoint(obs.featureID))
					fn(track, obs, img);
			}
		});
	};
	forEachDense([&](const Track&, const Observation& obs, const Image&) { ++numDense[obs.imageID]; });
	bool anyOverCap = false;
	for (IIndex imgID = 0; imgID < numImages && !anyOverCap; ++imgID)
		anyOverCap = numDense[imgID] > cap;
	if (!anyOverCap)
		return 0;

	// gather the candidates of every over-cap image, one sort key per observation:
	// [cell | inverted track length | dense keypoint index], so sorting it groups an image's
	// candidates by cell and orders each cell by descending track length, the keypoint index
	// breaking the ties into one order for a given scene
	const unsigned gridSize = MAXF((unsigned)CEIL2INT(SQRT((float)cap)), 1u);
	denseCap.keep.resize(numImages);
	denseCap.numDescribed.assign(numImages, 0);
	std::vector<std::vector<uint64_t>> candidates(numImages);
	FOREACH(imgID, scene.images) {
		const Image& img = scene.images[imgID];
		if (!img.HasDenseKeypoints())
			continue;
		// every image carrying dense keypoints gets its flags, an under-cap one keeping all of
		// them: the pass that holds a track's views together must be able to drop any of them
		denseCap.numDescribed[imgID] = img.NumDescribedKeypoints();
		denseCap.keep[imgID].assign(img.NumDenseKeypoints(), numDense[imgID] <= cap);
		if (numDense[imgID] > cap)
			candidates[imgID].reserve(numDense[imgID]);
	}
	forEachDense([&](const Track& track, const Observation& obs, const Image& img) {
		if (numDense[obs.imageID] <= cap)
			return; // image under the cap: it keeps everything
		const uint32_t denseIdx = obs.featureID - denseCap.numDescribed[obs.imageID];
		const uint64_t cell = DenseObservationCell(img, img.keypoints[obs.featureID], gridSize);
		const uint64_t invLength = 0xff - MINF(track.GetNumInliers(), 0xffu);
		candidates[obs.imageID].push_back((cell << 40) | (invLength << 32) | denseIdx);
	});

	// take them round-robin across the cells until the cap is reached
	uint32_t numDropped = 0;
	std::vector<uint32_t> cellStart, active;
	FOREACH(imgID, scene.images) {
		std::vector<uint64_t>& imgCandidates = candidates[imgID];
		if (imgCandidates.empty())
			continue;
		std::sort(imgCandidates.begin(), imgCandidates.end());
		// the sorted candidates of one cell are contiguous; remember where each run starts
		cellStart.clear();
		for (size_t i = 0; i < imgCandidates.size(); ++i)
			if (i == 0 || (imgCandidates[i] >> 40) != (imgCandidates[i-1] >> 40))
				cellStart.push_back((uint32_t)i);
		cellStart.push_back((uint32_t)imgCandidates.size()); // sentinel: end of the last run
		active.resize(cellStart.size()-1);
		std::iota(active.begin(), active.end(), 0u);
		std::vector<bool>& imgKeep = denseCap.keep[imgID];
		unsigned numKept = 0;
		for (uint32_t round = 0; numKept < cap && !active.empty(); ++round) {
			size_t numAlive = 0;
			for (uint32_t cell : active) {
				const uint32_t begin = cellStart[cell], end = cellStart[cell+1];
				if (begin + round >= end)
					continue; // this cell ran out of candidates
				imgKeep[(uint32_t)imgCandidates[begin + round]] = true;
				active[numAlive++] = cell;
				if (++numKept == cap)
					break;
			}
			active.resize(numAlive);
		}
		ASSERT(numKept <= imgCandidates.size());
		numDropped += (uint32_t)imgCandidates.size() - numKept;
	}

	// A point one view sees is not determined by it, so a track takes two observations into the
	// solve or none. A track the cap cut that far is a short one -- the round-robin takes the long
	// tracks first, and a two-view dense track is what it takes last -- and it leaves the solve
	// rather than sit in it on a single ray, its position standing until the next triangulation
	// recomputes it from the cameras this solve moved. Where a DESCRIBED observation is the one
	// left standing the track cannot leave, since the cap never drops those, so it is given a dense
	// observation back instead.
	visitTracks([&](const Track& track) {
		unsigned numInSolve = 0, numKept = 0, numDescribedKept = 0;
		for (const Observation& obs : track) {
			if (!inSolve(obs.imageID))
				continue;
			const Image& img = scene.images[obs.imageID];
			++numInSolve;
			if (!denseCap.Keeps(obs.imageID, img, obs.featureID))
				continue;
			++numKept;
			numDescribedKept += !img.IsDenseKeypoint(obs.featureID);
		}
		if (numKept >= 2 || numInSolve < 2)
			return;
		if (numDescribedKept > 0) {
			// give back dropped dense observations until the track is seen twice
			for (const Observation& obs : track) {
				if (numKept >= 2)
					break;
				if (!inSolve(obs.imageID) || denseCap.Keeps(obs.imageID, scene.images[obs.imageID], obs.featureID))
					continue;
				denseCap.keep[obs.imageID][obs.featureID - denseCap.numDescribed[obs.imageID]] = true;
				++numKept;
				--numDropped;
			}
			return;
		}
		// nothing holds the track in the solve: let it go
		for (const Observation& obs : track) {
			if (!inSolve(obs.imageID) || !denseCap.Keeps(obs.imageID, scene.images[obs.imageID], obs.featureID))
				continue;
			denseCap.keep[obs.imageID][obs.featureID - denseCap.numDescribed[obs.imageID]] = false;
			++numDropped;
		}
	});
	return numDropped;
}

// Solve, retrying once with the iterative solver if a sparse solve failed: the sparse
// factorization of the reduced camera system gives up outright on an ill-conditioned problem,
// while the iterative solver never factorizes it, so the failure costs a retry rather than the
// whole adjustment.
inline void SolveBundle(ceres::Solver::Options& options, ceres::Problem& problem, ceres::Solver::Summary& summary) {
	ceres::Solve(options, &problem, &summary);
	if (summary.IsSolutionUsable() || options.linear_solver_type != ceres::SPARSE_SCHUR)
		return;
	VERBOSE("warning: the sparse bundle adjustment failed (%s); solving it iteratively", summary.BriefReport().c_str());
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	#if 0 && (CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2))
	// DISABLED: Power Bundle Adjustment (Weber et al., CVPR 2022) via the
	// SCHUR_POWER_SERIES_EXPANSION preconditioner, gated on a large camera count (the reduced
	// camera system it is meant to accelerate). Benchmarked against the SCHUR_JACOBI default and
	// it loses at every scale tested, so it is left off. On an i7-13700KF (16C/24T) / RTX 4070 /
	// 32GB / Win11, Ceres 2.2.0 + CUDA 13.0: House (83 cameras) ran 1.2-3.9x slower; Tanks&Temples
	// Courthouse (1106 cameras) ran 1.6-1.7x slower on the 4-5.6M-residual bundles and HUNG for
	// >81 min on a 6.3M-residual bundle (never converged), while SCHUR_JACOBI completed the whole
	// reconstruction in ~54 min. CG convergence was erratic (non-monotonic in problem size).
	// Re-enable/re-tune (e.g. without use_spse_initialization, and past a thousand cameras only)
	// with a fresh benchmark on a scene with far more cameras than we had available.
	options.preconditioner_type = ceres::SCHUR_POWER_SERIES_EXPANSION;
	options.use_spse_initialization = true;
	#endif
	ceres::Solve(options, &problem, &summary);
}

// Pick the (possibly confidence-scaled) loss for the observation of keypoint featureID of img.
// Sets bDense for the caller's summary. Returns false if the keypoint is below the confidence
// threshold and the observation should be skipped.
//
// A dense (descriptor-less) keypoint's residual is scaled by denseObservationWeight: its
// position was sampled from a low-resolution warp, while a described keypoint's is sub-pixel at
// full resolution, so the two are not equally precise measurements and BA must not weight them
// alike. The weight follows the KEYPOINT, not the match that created it -- after the described-wins
// dedup of PairsMatcher::FilterRedundantKeypoints an observation created by a dense match can
// reference a described keypoint, and its measured position is then the sub-pixel one, so it takes
// full weight. This models measurement precision only: a wrong correspondence is the robust loss's
// and FilterTracks' job, and charging it here as well would model the same thing twice.
//
// EXCLUSIVE WITH config.useKeypointConfidence, which expresses the same statement by another route:
// ComputeKeypointPrecision's SQUARE(2/max(size,1)) term already reads measurement precision off the
// sampling scale, and the dense `size` convention (Image.h) was chosen precisely so that it reports a
// dense point as the less precise measurement. Multiplying the two charges that once for the size and
// once for the flat factor -- on the documented values it takes the effective ratio to ~116x
// (k ~ 10.8), against the k = 1.55-4.02 the captures actually measure. So exactly one of them applies.
inline bool SelectReprojectionLoss(const BAConfig& config, double denseObservationWeight,
	const Image& img, uint32_t featureID,
	ceres::LossFunction* baseLoss, ceres::LossFunction*& outLoss, bool& bDense) {
	outLoss = baseLoss;
	double weight = 1.0;
	if (config.useKeypointConfidence) {
		weight = Image::ComputeKeypointPrecision(img.keypoints[featureID], config.minKeypointResponse);
		if (weight <= 0.0)
			return false; // skip low-confidence keypoint
	}
	bDense = img.IsDenseKeypoint(featureID);
	// only when the confidence term did not already say it: the two express the same statement (see
	// the comment above), so applying both would charge the dense sampling scale twice
	if (bDense && !config.useKeypointConfidence)
		weight *= denseObservationWeight;
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
			// the hand-written Jacobians of PinholeReprojectionError, an order of magnitude cheaper
			// to evaluate than differentiating the projection through 22 dual numbers, and equal to
			// what that differentiation gives (BAPinholeReprojectionJacobianTest)
			new PinholeReprojectionErrorAnalytic(kp.pt.x, kp.pt.y),
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

// Threshold of the robust loss the two halves of a pair residual share, in sigmas: past three
// standard deviations the pair and the model disagree about more than measurement noise, and the
// residual's pull stops growing rather than dragging a correct model onto a wrong pair.
constexpr double PAIR_CONSTRAINT_HUBER_SIGMAS = 3.0;
// Confidence a pair's inliers lend its relative pose, as the factor its two sigmas are divided by:
// the square root of the weighted inlier count against a reference of 100, the count capped first.
// A pair of 500 inliers then counts sqrt(5) times a pair of 100 and a pair of 25 counts half, while
// the cap keeps the heaviest pairs of a graph from overwhelming the reprojections they complement.
constexpr unsigned MAX_PAIR_CONSTRAINT_INLIERS = 500;
constexpr double PAIR_CONSTRAINT_REFERENCE_INLIERS = 100;
inline double PairConstraintConfidence(unsigned numInliers) {
	return SQRT(MINF(numInliers, MAX_PAIR_CONSTRAINT_INLIERS)/PAIR_CONSTRAINT_REFERENCE_INLIERS);
}

// One relative-pose residual per verified pair whose two images are both part of the solve: the
// pair's relative rotation and baseline direction against the model's, the two halves sharing one
// robust loss. inSolve() answers whether an image takes part; poseParams is the flat
// [qw,qx,qy,qz,Cx,Cy,Cz] array indexed by image ID. Returns how many residuals were added.
//
// A verified pair is a measurement of two images' relative pose from hundreds of correspondences
// that the reprojection residuals never see: they fit the tracks, and where the tracks joining two
// parts of the model are few or two-view only the joint between them is free to bend while every
// pair across it says by how much. Adding the pairs is what holds it straight.
template <typename TInSolve>
uint32_t AddRelativePoseResiduals(ceres::Problem& problem, const Scene& scene,
	const BAConfig& config, double* poseParams, const TInSolve& inSolve)
{
	if (!config.IsUsingPairConstraints())
		return 0;
	const double weightRotation = config.relativeRotationSigma > 0.f ? 1.0/config.relativeRotationSigma : 0.0;
	const double weightTranslation = config.relativeTranslationSigma > 0.f ? 1.0/config.relativeTranslationSigma : 0.0;
	// one loss shared by every pair residual, created with the first of them so that a solve adding
	// none does not leak it; the problem reference-counts the losses it owns
	ceres::LossFunction* pairLoss = NULL;
	uint32_t numResiduals = 0;
	for (const ImagePair& pair : scene.pairs) {
		if (!IsPoseLinkPair(pair))
			continue;
		ASSERT(pair.ID1 < scene.images.size() && pair.ID2 < scene.images.size());
		if (!inSolve(pair.ID1) || !inSolve(pair.ID2))
			continue;
		const double confidence = PairConstraintConfidence(pair.GetNumWeightedInliers());
		if (confidence <= 0)
			continue;
		const Pose3D& relPose = pair.relativePose.value();
		// the pair's own baseline direction, in its first image's frame; a pair whose two images
		// share a viewpoint measured no direction at all, and only its rotation is evidence
		double direction[3] = { relPose.C.x, relPose.C.y, relPose.C.z };
		const double baseline = norm(relPose.C);
		double weightDirection = 0.0;
		if (baseline > ZEROTOLERANCE<double>()) {
			direction[0] /= baseline; direction[1] /= baseline; direction[2] /= baseline;
			weightDirection = weightTranslation;
		}
		if (weightRotation <= 0.0 && weightDirection <= 0.0)
			continue;
		double quatRelative[4];
		ceres::RotationMatrixToQuaternion(ceres::RowMajorAdapter3x3(relPose.R.val), quatRelative);
		if (!pairLoss)
			pairLoss = new ceres::HuberLoss(PAIR_CONSTRAINT_HUBER_SIGMAS);
		problem.AddResidualBlock(
			RelativePoseError::Create(quatRelative, direction,
				weightRotation*confidence, weightDirection*confidence),
			pairLoss,
			poseParams + pair.ID1*7,   // Pose params of the pair's first image
			poseParams + pair.ID2*7);  // Pose params of its second image
		++numResiduals;
	}
	return numResiduals;
}
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

	// resolved once per solve, not per residual: the estimator walks every observation, and the
	// weight is a property of the scene this problem is built from, not of any one of its residuals.
	// Short-circuited to 1.0 under useKeypointConfidence: that mode supersedes this weight entirely
	// (SelectReprojectionLoss never multiplies by it), so running the estimator would be a full scene
	// walk whose result is discarded, and reporting it below would claim a weight nothing applied.
	DenseObservationSigmas denseSigmas; // what that weight was measured on, for the report below
	const double denseWeight = config.useKeypointConfidence ? 1.0 :
		EstimateDenseObservationWeight(scene, config, &denseSigmas);

	// which dense observations each image contributes, decided before the residuals are added
	DenseObservationCap denseCap;
	const uint32_t numDenseDropped = BuildDenseObservationCap(scene, config.maxDenseObservationsPerImage,
		[this](const auto& fn) {
			for (const Track& track : scene.tracks)
				if (track.IsInlier())
					fn(track);
		},
		[this](IIndex imgID) { return scene.images[imgID].IsValid(); }, denseCap);

	// Add reprojection residuals
	uint32_t numReprojResiduals = 0;
	uint32_t numSkippedLowConfidence = 0;
	uint32_t numDenseResiduals = 0;
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
			if (!denseCap.Keeps(imgID, img, obs.featureID))
				continue; // dense observation the per-image cap left out
			// Compute weight from keypoint response / size (if enabled), down-weighted on a dense
			// keypoint (whose position came from the warp, not from the detector)
			ceres::LossFunction* residual_loss_function;
			bool bDense = false;
			if (!SelectReprojectionLoss(config, denseWeight, img, obs.featureID, loss_function, residual_loss_function, bDense)) {
				++numSkippedLowConfidence;
				continue; // skip low-confidence keypoints
			}
			AddReprojectionResidual(problem, residual_loss_function, img, img.keypoints[obs.featureID],
				poseParams.data() + imgID * 7, track.position.ptr(), intrinsicParams);
			++numReprojResidualsPerImage[imgID];
			++numReprojResiduals;
			numDenseResiduals += bDense;
		}
	}
	if (config.useKeypointConfidence) {
		DEBUG_EXTRA("Created %u reprojection residuals (%u skipped low-confidence)",
		    numReprojResiduals, numSkippedLowConfidence);
	} else {
		DEBUG_EXTRA("Created %u reprojection residuals", numReprojResiduals);
	}
	if (numDenseResiduals > 0) {
		// the sigmas are reported only where the weight actually came from them, so that what the
		// line prints is always what produced the number next to it: a weight the config pinned
		// measured nothing, the confidence term supersedes this weight entirely (denseWeight is 1.0
		// in that mode, and what scales a dense residual is that term alone), and a sample the
		// estimator refused fell back to the constant. Reported at all so that a sigma jumping
		// between runs shows up here rather than only as a downstream drift.
		const String capped(numDenseDropped == 0 ? String() :
			String::FormatString(", %u dense dropped by the per-image cap", numDenseDropped));
		if (denseSigmas.measured) {
			DEBUG("Bundle adjustment: %u/%u reprojection residuals are on dense keypoints, weighted %g "
				"(sigma described %g px over %u obs / dense %g px over %u obs)%s",
				numDenseResiduals, numReprojResiduals, denseWeight,
				denseSigmas.sigmaDescribed, (unsigned)denseSigmas.numDescribed,
				denseSigmas.sigmaDense, (unsigned)denseSigmas.numDense, capped.c_str());
		} else {
			DEBUG("Bundle adjustment: %u/%u reprojection residuals are on dense keypoints, weighted %g%s",
				numDenseResiduals, numReprojResiduals, denseWeight, capped.c_str());
		}
	}

	// Add relative-pose residuals from the verified pairs joining two registered images
	const uint32_t numPairResiduals = AddRelativePoseResiduals(problem, scene, config, poseParams.data(),
		[this](IIndex imgID) { return scene.images[imgID].IsValid(); });
	if (config.IsUsingPairConstraints())
		DEBUG("Created %u relative-pose residuals from the verified pairs", numPairResiduals);

	// Set the SE(3) manifold on every pose block the residuals actually created (shared instance;
	// Ceres owns it once attached, so a solve that added none must free it itself). An image no
	// residual reached is left out of the problem rather than added as a free block: nothing would
	// determine it, and the reduced camera system it sits in would be singular.
	auto* se3_manifold = CreateSE3PoseManifold();
	bool poseManifoldUsed = false;
	FOREACH(i, scene.images) {
		double* pose = poseParams.data() + i * 7;
		if (!scene.images[i].IsValid() || !problem.HasParameterBlock(pose))
			continue;
		#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1
		problem.SetManifold(pose, se3_manifold);
		#else
		problem.SetParameterization(pose, se3_manifold);
		#endif
		poseManifoldUsed = true;
	}
	if (!poseManifoldUsed)
		delete se3_manifold;

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
		if (!config.refineRadialDistortion12) {
			constantParams.push_back(4);  // k1
			constantParams.push_back(5);  // k2
		}
		if (!config.refineRadialDistortion3) {
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
			SetParameterBlockConstantIfPresent(problem, poseParams.data() + bestImgID * 7);
			DEBUG("Fixed view %u (reference, no GPS)", bestImgID);
		}
	}

	// Optionally disable pose/point refinement (only for the blocks the residuals created)
	if (!config.IsRefiningPoses()) {
		// Disable all pose refinement
		FOREACH(i, scene.images)
			if (scene.images[i].IsValid())
				SetParameterBlockConstantIfPresent(problem, poseParams.data() + i * 7);
		DEBUG("Views poses: FIXED");
	} else if (!config.refinePosesRotation || !config.refinePosesPosition) {
		// Selectively disable rotation and/or position refinement
		std::vector<int> constantParams;
		CollectConstantPoseParams(config, constantParams);
		FOREACH(i, scene.images)
			if (scene.images[i].IsValid() && problem.HasParameterBlock(poseParams.data() + i * 7))
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
		// Past that size the reduced camera system is factorized sparsely rather than densely.
		// The iterative alternative (ITERATIVE_SCHUR + SCHUR_JACOBI) solves the same system by
		// conjugate gradients, and how many of those a step costs depends on the conditioning of a
		// problem the dense observations have thinned -- on the captures measured it ran slower and
		// far less predictably. A factorization can fail outright where the iterations only converge
		// slowly, which is what SolveBundle's retry is for.
		options.linear_solver_type = ceres::SPARSE_SCHUR;
		options.use_inner_iterations = true; // Improves convergence
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
	SolveBundle(options, problem, summary);
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

	// same whole-scene estimator as Adjust(), not one scoped to this window: the local window's
	// described population is often too small to give a sigma, and two different weights inside
	// one reconstruction would be worse than a slightly stale one. Short-circuited to 1.0 under
	// useKeypointConfidence for the same reason as Adjust(): that mode supersedes this weight, so
	// estimating it would be a wasted whole-scene walk whose result nothing uses.
	DenseObservationSigmas denseSigmas; // what that weight was measured on, for the report below
	const double denseWeight = config.useKeypointConfidence ? 1.0 :
		EstimateDenseObservationWeight(scene, config, &denseSigmas);

	// which dense observations each window image contributes, decided before the residuals are
	// added and over the window alone: an image is capped on what it brings to THIS solve
	const auto inWindow = [&localImages, &fixedImages](IIndex imgID) {
		return localImages.find(imgID) != localImages.end() || fixedImages.find(imgID) != fixedImages.end();
	};
	DenseObservationCap denseCap;
	const uint32_t numDenseDropped = BuildDenseObservationCap(scene, config.maxDenseObservationsPerImage,
		[this, &activePoints](const auto& fn) {
			for (const uint32_t pointID : activePoints)
				fn(scene.tracks[pointID]);
		},
		inWindow, denseCap);

	// Add reprojection residuals (only observations from window images: local or fixed)
	uint32_t numReprojResiduals = 0;
	uint32_t numDenseResiduals = 0;
	numReprojResidualsPerImage.resize(scene.images.size());
	numReprojResidualsPerImage.Memset(0);
	for (const IIndex pointID : activePoints) {
		Track& track = scene.tracks[pointID];
		ASSERT(track.IsInlier());
		for (const Observation& obs : track) {
			const IIndex imgID = obs.imageID;
			// Only consider observations in local or fixed images
			if (!inWindow(imgID))
				continue;
			const Image& img = scene.images[imgID];
			ASSERT(obs.featureID < img.keypoints.size());
			if (!denseCap.Keeps(imgID, img, obs.featureID))
				continue; // dense observation the per-image cap left out
			// Compute weight from keypoint response / size (if enabled), down-weighted on a dense
			// keypoint (whose position came from the warp, not from the detector)
			ceres::LossFunction* residual_loss_function;
			bool bDense = false;
			if (!SelectReprojectionLoss(config, denseWeight, img, obs.featureID, loss_function, residual_loss_function, bDense))
				continue; // skip low-confidence keypoints
			AddReprojectionResidual(problem, residual_loss_function, img, img.keypoints[obs.featureID],
				poseParams.data() + imgID * 7, track.position.ptr(), intrinsicParams);
			++numReprojResidualsPerImage[imgID];
			++numReprojResiduals;
			numDenseResiduals += bDense;
		}
	}
	// reported here as well as in Adjust(): incremental reconstruction runs local BA far more often
	// than the global pass, so reporting only there would let the dense contribution move silently
	// in the path that actually carries it
	if (numDenseResiduals > 0) {
		// the sigmas only where the weight came from them, exactly as in Adjust(): a pinned weight,
		// the confidence term's 1.0, and a refused sample's fallback constant all print alone rather
		// than beside two sigmas that do not produce them
		const String capped(numDenseDropped == 0 ? String() :
			String::FormatString(", %u dense dropped by the per-image cap", numDenseDropped));
		if (denseSigmas.measured) {
			DEBUG("Local bundle adjustment: %u/%u reprojection residuals are on dense keypoints, weighted %g "
				"(sigma described %g px over %u obs / dense %g px over %u obs)%s",
				numDenseResiduals, numReprojResiduals, denseWeight,
				denseSigmas.sigmaDescribed, (unsigned)denseSigmas.numDescribed,
				denseSigmas.sigmaDense, (unsigned)denseSigmas.numDense, capped.c_str());
		} else {
			DEBUG("Local bundle adjustment: %u/%u reprojection residuals are on dense keypoints, weighted %g%s",
				numDenseResiduals, numReprojResiduals, denseWeight, capped.c_str());
		}
	}

	// Add relative-pose residuals from the verified pairs joining two window images, the fixed ones
	// included: a fixed pose block is a constant, so such a residual constrains the free image alone
	const uint32_t numPairResiduals = AddRelativePoseResiduals(problem, scene, config, poseParams.data(), inWindow);
	if (config.IsUsingPairConstraints())
		DEBUG("Created %u relative-pose residuals from the verified pairs (local BA)", numPairResiduals);

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
	SolveBundle(options, problem, summary);
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
	// The analytic and the auto-diff reprojection functors are two derivations of the same
	// projection, and auto-diff differentiates it exactly, so the two Jacobians must agree to
	// numerical precision -- not to a finite-difference tolerance. Every parameter is randomized
	// over many trials with the whole distortion model live (k3, p1, p2 and the rational
	// denominator k4-k6 all non-zero), since a hand-written derivative goes wrong in the terms a
	// single hand-picked test case leaves at zero.
	//
	// The quaternion columns are the one exception: both functors extend the rotation off the unit
	// sphere, by different amounts along the radial direction, and Ceres never sees that direction
	// (the quaternion manifold projects it out), so the auto-diff block is projected onto the
	// tangent space before the comparison -- which is what the solver differentiates.
	// "Relative" is per parameter block: the largest entry-wise difference over the largest entry
	// of the block, so a block whose entries span orders of magnitude is judged on its own scale.
	constexpr double maxRelativeError = 1e-6;
	constexpr unsigned numTrials = 256;
	std::mt19937 rng(20260909u);
	const auto rnd = [&rng](double lo, double hi) {
		return std::uniform_real_distribution<double>(lo, hi)(rng);
	};
	double worstRelativeError = 0, worstResidualError = 0;
	for (unsigned trial = 0; trial < numTrials; ++trial) {
		// random pose
		double pose[7] = { rnd(-1.0, 1.0), rnd(-1.0, 1.0), rnd(-1.0, 1.0), rnd(-1.0, 1.0),
			rnd(-3.0, 3.0), rnd(-3.0, 3.0), rnd(-3.0, 3.0) };
		Eigen::Map<Eigen::Vector4d> quat(pose);
		if (quat.norm() < 0.1)
			continue; // too close to zero to normalize into a rotation
		quat.normalize();
		// random point in front of the camera, placed through the camera-space point it projects
		// to, so that the trial is never spent on a point behind it
		const double pointCamera[3] = { rnd(-4.0, 4.0), rnd(-4.0, 4.0), rnd(1.0, 10.0) };
		const double quatInverse[4] = { pose[0], -pose[1], -pose[2], -pose[3] };
		double point[3];
		ceres::UnitQuaternionRotatePoint(quatInverse, pointCamera, point);
		point[0] += pose[4]; point[1] += pose[5]; point[2] += pose[6];
		// random intrinsics: [fx, fy/fx, cx, cy, k1, k2, k3, p1, p2, k4, k5, k6]
		const double intrinsics[12] = { rnd(400.0, 2000.0), rnd(0.95, 1.05), rnd(300.0, 340.0), rnd(220.0, 260.0),
			rnd(-0.3, 0.3), rnd(-0.2, 0.2), rnd(-0.1, 0.1), rnd(-0.01, 0.01), rnd(-0.01, 0.01),
			rnd(-0.2, 0.2), rnd(-0.1, 0.1), rnd(-0.05, 0.05) };
		const double* parameters[3] = { pose, intrinsics, point };

		const double observedX = rnd(0.0, 640.0), observedY = rnd(0.0, 480.0);
		PinholeReprojectionErrorAnalytic analytic(observedX, observedY);
		double analyticResiduals[2], analyticPose[2*7], analyticIntrinsics[2*12], analyticPoint[2*3];
		double* analyticJacobians[3] = { analyticPose, analyticIntrinsics, analyticPoint };
		std::unique_ptr<ceres::CostFunction> autodiff(PinholeReprojectionError::Create(observedX, observedY));
		double autodiffResiduals[2], autodiffPose[2*7], autodiffIntrinsics[2*12], autodiffPoint[2*3];
		double* autodiffJacobians[3] = { autodiffPose, autodiffIntrinsics, autodiffPoint };
		if (!analytic.Evaluate(parameters, analyticResiduals, analyticJacobians) ||
			!autodiff->Evaluate(parameters, autodiffResiduals, autodiffJacobians)) {
			VERBOSE("BAPinholeReprojectionJacobianTest FAILED: cost evaluation failed");
			return false;
		}
		// project the auto-diff quaternion columns onto the manifold's tangent space
		for (int i = 0; i < 2; ++i) {
			Eigen::Map<Eigen::Vector4d> jacobianQuat(autodiffPose + i*7);
			jacobianQuat -= jacobianQuat.dot(quat)*quat;
		}
		for (int i = 0; i < 2; ++i)
			worstResidualError = MAXF(worstResidualError,
				ABS(analyticResiduals[i] - autodiffResiduals[i])/MAXF(ABS(autodiffResiduals[i]), 1.0));
		const int blockSizes[3] = { 2*7, 2*12, 2*3 };
		const char* const blockNames[3] = { "pose", "intrinsics", "point" };
		for (int block = 0; block < 3; ++block) {
			double maxDifference = 0, maxValue = 0;
			for (int i = 0; i < blockSizes[block]; ++i) {
				maxDifference = MAXF(maxDifference, ABS(analyticJacobians[block][i] - autodiffJacobians[block][i]));
				maxValue = MAXF(maxValue, MAXF(ABS(analyticJacobians[block][i]), ABS(autodiffJacobians[block][i])));
			}
			const double relativeError = maxValue > 0 ? maxDifference/maxValue : maxDifference;
			worstRelativeError = MAXF(worstRelativeError, relativeError);
			if (relativeError > maxRelativeError) {
				VERBOSE("BAPinholeReprojectionJacobianTest FAILED: trial %u %s Jacobian differs by %g relative (max entry %g)",
					trial, blockNames[block], relativeError, maxValue);
				return false;
			}
		}
	}
	VERBOSE("BAPinholeReprojectionJacobianTest PASSED: %u trials, worst relative Jacobian difference %.2e, residual %.2e (%s)",
		numTrials, worstRelativeError, worstResidualError, TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/
