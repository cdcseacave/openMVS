/*
 * GlobalAlignment.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#include "Common.h"
#include "GlobalAlignment.h"
#include "GlobalRotationAveraging.h"
#include "GlobalScaleAveraging.h"
#include "GlobalTranslationAveraging.h"
#include "Resection.h"
#include "Scene.h"
#include "SimilarityTransform.h"
#include "Track.h"
#include "Triangulation.h"
#include "InterfaceMVS.h"
#include <PoseLib/poselib.h>
#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#pragma pop_macro("VERBOSE")
#pragma pop_macro("LOG")

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)

// enable to export intermediate aligned sub-scenes for debugging
#define GLOBALALIGNMENT_DEBUG 0


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("GlbAlign"));


GlobalAlignment::GlobalAlignment(Scene& _scene, const GlobalAlignmentConfig& _config)
	: scene(_scene), config(_config)
{
}

void GlobalAlignment::BuildGlobalToLocalMap(const std::vector<IIndexArr>& localToGlobals)
{
	globalToLocal.clear();
	for (uint32_t sceneIdx = 0; sceneIdx < localToGlobals.size(); ++sceneIdx) {
		const IIndexArr& mapping = localToGlobals[sceneIdx];
		for (IIndex localID = 0; localID < mapping.size(); ++localID) {
			const IIndex globalID = mapping[localID];
			if (globalID == NO_ID)
				continue;
			MAYBEUNUSED const auto [it, inserted] = globalToLocal.emplace(globalID, std::make_pair(sceneIdx, localID));
			ASSERT(inserted, "global image %u appears in multiple sub-scenes (%u:%u and %u:%u)",
				globalID, it->second.first, it->second.second, sceneIdx, localID);
		}
	}
}

// index of the sub-scene holding the most calibrated images, among those flagged eligible
// (all of them when no mask is given); NO_ID if none is eligible
static uint32_t FindLargestSubScene(const std::vector<Scene>& subScenes, const std::vector<bool>* eligible = NULL)
{
	uint32_t best = NO_ID;
	FOREACH(sceneIdx, subScenes)
		if ((eligible == NULL || (*eligible)[sceneIdx]) &&
			(best == NO_ID || subScenes[sceneIdx].status.nCalibratedImages > subScenes[best].status.nCalibratedImages))
			best = (uint32_t)sceneIdx;
	return best;
}

bool GlobalAlignment::MergeScenes(std::vector<Scene>& subScenes, const std::vector<IIndexArr>& localToGlobals)
{
	TD_TIMER_STARTD();

	ASSERT(!subScenes.empty());
	ASSERT(subScenes.size() == localToGlobals.size());
	const uint32_t numSubScenes = (uint32_t)subScenes.size();
	VERBOSE("Merging %u sub-scenes into global scene", numSubScenes);

	#if GLOBALALIGNMENT_DEBUG
	// Export sub-scenes before alignment for debugging
	FOREACH(i, subScenes)
		subScenes[i].ExportPLY(String::FormatString("subscene_%u.ply", i));
	#endif

	BuildGlobalToLocalMap(localToGlobals);

	// Run the staged alignment pipeline; any stage failing breaks out to the fallback below.
	// Every failure occurs before the merge stage (Stage 5) consumes sub-scenes, so on failure
	// all sub-scenes are still intact and the fallback can keep the largest one.
	do {
		// Stage 1: Estimate relative poses between connected sub-scenes
		std::vector<ScenePair> scenePairs;
		if (!EstimateRelativePoses(subScenes, scenePairs)) {
			VERBOSE("error: failed to estimate relative poses");
			break;
		}

		// If only one sub-scene or no connections, just copy directly
		if (numSubScenes == 1 || scenePairs.empty()) {
			VERBOSE("Single sub-scene or no connections, copying directly");
			for (uint32_t sceneIdx = 0; sceneIdx < numSubScenes; ++sceneIdx)
				MergeSingleScene(subScenes[sceneIdx], localToGlobals[sceneIdx], true);
			DEBUG("Single-scene merge completed (%s)", TD_TIMER_GET_FMT().c_str());
			return true;
		}

		// Stage 2: Rotation averaging. Robustly rejects rotation-inconsistent sub-scene pairs and
		// prunes them from scenePairs in place; solves only its largest connected component and
		// leaves every sub-scene it could not place at Point3::INF.
		std::vector<Point3d> globalRotations;
		if (!EstimateGlobalRotations(scenePairs, numSubScenes, globalRotations)) {
			VERBOSE("error: failed to estimate global rotations");
			break;
		}

		// Merge only the sub-scenes the rotation estimator actually placed (finite rotation).
		// Deriving the merge set directly from the estimator's output is authoritative: it cannot
		// disagree with what rotation averaging solved (no separately-recomputed component, no
		// tie-break or filtered-edge mismatch), so an unplaced (INF) rotation can never reach
		// RMatrix() and inject NaN poses (the crash this guards against). Sub-scenes left at INF
		// — those with no rotation-consistent link into the solved component — stay unregistered
		// for the subsequent resection to recover individually.
		std::vector<bool> mergeMask(numSubScenes, false);
		unsigned numMergeScenes = 0;
		for (uint32_t s = 0; s < numSubScenes; ++s)
			if (globalRotations[s] != Point3::INF) { mergeMask[s] = true; ++numMergeScenes; }
		if (numMergeScenes == 0) {
			VERBOSE("error: rotation averaging placed no sub-scenes");
			break;
		}
		if (numMergeScenes < numSubScenes)
			VERBOSE("Merging the rotation-consistent component: %u/%u sub-scenes (%u unalignable, left for resection)",
				numMergeScenes, numSubScenes, numSubScenes - numMergeScenes);

		// Keep only pairs internal to the merged (finite-rotation) component, so scale and
		// translation averaging — whose gauge is fixed to the strongest-weighted node — anchor
		// inside the kept component rather than in a discarded one (which would leave the kept
		// component's translation block unconstrained).
		scenePairs.erase(std::remove_if(scenePairs.begin(), scenePairs.end(),
			[&mergeMask](const ScenePair& sp) { return !mergeMask[sp.sceneA] || !mergeMask[sp.sceneB]; }),
			scenePairs.end());

		// Stage 3: Scale averaging (over the rotation-consistent pairs surviving in scenePairs)
		std::vector<REAL> globalScales;
		if (!EstimateGlobalScales(scenePairs, numSubScenes, globalScales)) {
			VERBOSE("error: failed to estimate global scales");
			break;
		}

		// Stage 4: Translation averaging (over the rotation-consistent pairs surviving in scenePairs)
		std::vector<Point3> globalTranslations;
		if (!EstimateGlobalTranslations(scenePairs, globalRotations, globalScales, numSubScenes, globalTranslations)) {
			VERBOSE("error: failed to estimate global translations");
			break;
		}

		// Stage 4.5: drop the seams the averaged consensus contradicts, re-averaging after each,
		// then validate what is left and re-average the survivors until the verdict is stable
		std::vector<bool> demoted(numSubScenes);
		for (uint32_t s = 0; s < numSubScenes; ++s)
			demoted[s] = !mergeMask[s];
		if (!PruneConflictingSeams(subScenes, scenePairs, globalRotations, globalScales, globalTranslations, demoted))
			break;
		const unsigned numDemotedBefore = (unsigned)std::count(demoted.begin(), demoted.end(), true);
		std::vector<bool> keepMask(numSubScenes);
		for (uint32_t s = 0; s < numSubScenes; ++s)
			keepMask[s] = !demoted[s];
		demoted = ValidateAlignment(subScenes, keepMask, scenePairs,
			globalRotations, globalScales, globalTranslations);
		if ((unsigned)std::count(demoted.begin(), demoted.end(), true) > numDemotedBefore &&
			!RefineDemotedAlignment(subScenes, scenePairs, globalRotations, globalScales, globalTranslations, demoted))
			break;

		// Stage 5: Merge sub-scenes with global transforms (largest connected component only)
		if (!MergeTransformedScenes(subScenes, localToGlobals, globalRotations, globalScales, globalTranslations, demoted)) {
			VERBOSE("error: failed to merge transformed scenes");
			break;
		}

		#if GLOBALALIGNMENT_DEBUG
		// Export merged scene for debugging
		ExportMVS(MAKE_PATH("scene_merged_reconstruction.mvs"), scene);
		#endif

		DEBUG("Global alignment completed: merged %u/%u sub-scenes (%s)",
			numSubScenes - (unsigned)std::count(demoted.begin(), demoted.end(), true),
			numSubScenes, TD_TIMER_GET_FMT().c_str());
		return true;
	} while (0);

	// Fallback: alignment could not complete. Keep the largest successfully-reconstructed
	// sub-scene rather than discarding a good partial reconstruction (downstream BA/resection
	// then refine it in place). Safe because every failure above precedes sub-scene consumption.
	const uint32_t bestIdx = FindLargestSubScene(subScenes);
	VERBOSE("warning: sub-scene merge failed; keeping largest sub-scene %u (%u/%u images)",
		bestIdx, subScenes[bestIdx].status.nCalibratedImages, (unsigned)subScenes[bestIdx].images.size());
	scene.Release();
	scene = std::move(subScenes[bestIdx]);
	return false;
}
/*----------------------------------------------------------------*/


namespace {

// One cross-sub-scene image pair, with the sub-scene each of its two images belongs to already
// resolved, so feature indices (queryIdx/trainIdx) can be mapped consistently.
struct PairLink {
	const ImagePair* pair;
	IIndex localIdA;
	IIndex localIdB;
	bool aIsQuery; // true if pair.ID1 belongs to sub-scene A (i.e. uses queryIdx)
};

// One reprojection residual of the joint seam refinement: a 3D point of one sub-scene seen by a
// camera of the other, each expressed in its own sub-scene's frame.
struct SeamObservation {
	Point3 X;              // 3D point, in the frame of the sub-scene that triangulated it
	Matrix3x3 R;           // observing camera rotation, its own sub-scene's world to camera
	Point3 C;              // observing camera center, in its own sub-scene's frame
	Point3 bearing;        // observed unit bearing, in the observing camera's frame
	double pixelPerRadian; // converts an angular residual into the pixel units the loss is set in
	bool forward;          // true: point in A seen from B (apply T); false: point in B seen from A (apply T^-1)
};

// Reprojection of one seam observation through the A -> B similarity T = (q, t, exp(logScale)).
// The residual is the chord between the predicted unit bearing and the observed one, converted to
// pixels by the observing camera's own angular-to-pixel rate, so the Huber loss set at the pixel
// threshold means the same for every camera in the seam. The chord equals the angular error to
// first order, stays finite for any central camera model, and grows to 2 radians for a point that
// ends up behind the camera -- where the offset in the observed bearing's tangent plane, which this
// replaced, collapses back to zero and reports the worst possible fit as a perfect one.
struct SeamReprojectionError
{
	double X[3], R[9], C[3], b[3], pixelPerRadian;
	bool forward;

	explicit SeamReprojectionError(const SeamObservation& obs) : forward(obs.forward) {
		const Point3 bearing(normalized(obs.bearing));
		for (int i = 0; i < 3; ++i) {
			X[i] = obs.X[i];
			C[i] = obs.C[i];
			b[i] = bearing[i];
			for (int j = 0; j < 3; ++j)
				R[i*3+j] = obs.R(i, j);
		}
		pixelPerRadian = obs.pixelPerRadian;
	}

	template <typename T>
	bool operator()(const T* const q, const T* const t, const T* const logScale, T* residuals) const {
		using std::exp;
		using std::sqrt;
		const T Xw[3] = {T(X[0]), T(X[1]), T(X[2])};
		T p[3];
		if (forward) {
			// p_B = s * R * X_A + t
			T Rx[3];
			ceres::QuaternionRotatePoint(q, Xw, Rx);
			const T s = exp(logScale[0]);
			for (int i = 0; i < 3; ++i)
				p[i] = s * Rx[i] + t[i];
		} else {
			// p_A = R^t * (X_B - t) / s
			const T d[3] = {Xw[0] - t[0], Xw[1] - t[1], Xw[2] - t[2]};
			const T qInv[4] = {q[0], -q[1], -q[2], -q[3]};
			T Rd[3];
			ceres::QuaternionRotatePoint(qInv, d, Rd);
			const T invS = exp(-logScale[0]);
			for (int i = 0; i < 3; ++i)
				p[i] = invS * Rd[i];
		}
		// into the observing camera, then onto the unit sphere
		const T d[3] = {p[0] - T(C[0]), p[1] - T(C[1]), p[2] - T(C[2])};
		T c[3];
		for (int i = 0; i < 3; ++i)
			c[i] = T(R[i*3+0]) * d[0] + T(R[i*3+1]) * d[1] + T(R[i*3+2]) * d[2];
		const T len = sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2]);
		if (len <= T(0))
			return false;
		for (int i = 0; i < 3; ++i)
			residuals[i] = T(pixelPerRadian) * (c[i] / len - T(b[i]));
		return true;
	}
};

// The correspondences of one direction of the camera alignment: one sub-scene's cameras are the
// rig, the other sub-scene's inlier tracks are the points. The estimator wants them grouped per
// rig camera, so only the images that received a correspondence enter the rig and localIDs keeps
// the mapping back to the sub-scene.
struct RigCorrespondences
{
	std::vector<std::vector<poselib::Point3D>> bearings; // per rig camera, unit vectors in that camera's frame
	std::vector<std::vector<poselib::Point3D>> points;   // per rig camera, points in the other sub-scene's frame
	std::vector<poselib::CameraPose> cameraExt;          // per rig camera, its pose in the rig sub-scene's frame
	std::vector<IIndex> localIDs;                        // per rig camera, its image index in the rig sub-scene
	std::vector<double> pixelPerRadian;                  // per rig camera, its angular-to-pixel rate
	double maxError{0};                                  // angular RANSAC threshold, radians
	unsigned numCorrespondences{0};
};

// Gather the observations, in the rig sub-scene's images, of the point sub-scene's inlier tracks:
// one bearing per cross-sub-scene match whose point-side endpoint lies on such a track. Only that
// one side has to be on a track, which is why a seam remains measurable from either direction
// alone even when its other side's keypoints never formed one.
RigCorrespondences CollectRigCorrespondences(
	const Scene& rigScene, const Scene& pointScene,
	const std::unordered_map<PairIdx, Point3>& pointObsToTrackPos,
	const CLISTDEF0IDX(PairLink, uint32_t)& links, bool rigIsB, float maxReprojError)
{
	RigCorrespondences rc;
	std::unordered_map<IIndex, uint32_t> rigIndexOf;
	for (const PairLink& link : links) {
		const IIndex localIdRig = rigIsB ? link.localIdB : link.localIdA;
		const IIndex localIdPoint = rigIsB ? link.localIdA : link.localIdB;
		ASSERT(localIdRig < rigScene.images.size() && localIdPoint < pointScene.images.size());
		const Image& imgRig = rigScene.images[localIdRig];
		const Image& imgPoint = pointScene.images[localIdPoint];
		if (!imgRig.IsValid() || !imgPoint.IsValid())
			continue;
		// the track-forming set, not the sparse count: the tracks the point side is looked up in
		// were built from this same set, so a shorter bound would skip correspondences they prove
		// exist; clamped by the array because the loop dereferences matches[i] before anything
		// inspects the DMatch
		const unsigned numMatches = MINF(link.pair->GetNumTrackFormingMatches(), (unsigned)link.pair->matches.size());
		for (unsigned i = 0; i < numMatches; ++i) {
			const DMatch& match = link.pair->matches[i];
			const uint32_t featureA = link.aIsQuery ? match.queryIdx : match.trainIdx;
			const uint32_t featureB = link.aIsQuery ? match.trainIdx : match.queryIdx;
			const uint32_t featureRig = rigIsB ? featureB : featureA;
			const uint32_t featurePoint = rigIsB ? featureA : featureB;
			const auto it = pointObsToTrackPos.find(PairIdx(localIdPoint, featurePoint));
			if (it == pointObsToTrackPos.end())
				continue;
			if (featureRig >= imgRig.keypoints.size())
				continue;
			const auto [itRig, inserted] = rigIndexOf.emplace(localIdRig, (uint32_t)rc.cameraExt.size());
			if (inserted) {
				rc.cameraExt.emplace_back(imgRig.R, imgRig.GetT());
				rc.localIDs.push_back(localIdRig);
				rc.bearings.emplace_back();
				rc.points.emplace_back();
				// the angle at which this camera reads the pixel threshold, and the rate that
				// converts back, so every camera model in the rig is judged at the same pixel error
				const REAL angular = imgRig.pCamera->PixelErrorToAngular(
					maxReprojError * imgRig.pCamera->GetFeatureNoiseScale());
				rc.pixelPerRadian.push_back(angular > 0 ? (double)(maxReprojError / angular) : 0.0);
				rc.maxError = MAXF(rc.maxError, (double)angular);
			}
			const uint32_t r = itRig->second;
			rc.bearings[r].emplace_back(imgRig.pCamera->UnprojectNormalized(imgRig.keypoints[featureRig].pt));
			rc.points[r].push_back(it->second);
			++rc.numCorrespondences;
		}
	}
	return rc;
}

// One direction of the camera alignment. The estimator solves scale * p_rig = R * p_point + t,
// i.e. it scales the rig's centers into the frame the points live in, so the similarity mapping
// the point sub-scene into the rig sub-scene is p_rig = (1/scale) * R * p_point + (1/scale) * t.
// Its inliers are appended to `observations` for the joint refinement, tagged with which side of
// the seam they came from. Returns 0 when the direction yields no usable estimate.
unsigned EstimateRigAgainstPoints(
	const RigCorrespondences& rc, const Scene& rigScene,
	const poselib::RansacOptions& ransac, bool pointsAreA,
	Transform& T, std::vector<SeamObservation>& observations)
{
	// the scale is observable only from points seen out of at least two distinct rig centers
	if (rc.cameraExt.size() < 2 || rc.numCorrespondences == 0 || rc.maxError <= 0)
		return 0;

	poselib::AbsolutePoseOptions opt;
	opt.ransac = ransac;
	// the estimator scores the whole rig against one angular threshold, so the rig is judged at
	// the widest of its cameras' thresholds; the refinement below charges each camera its own
	opt.max_error = rc.maxError;

	poselib::CameraPose pose;
	double scale = 1;
	std::vector<std::vector<char>> inliers;
	const poselib::RansacStats stats = poselib::estimate_generalized_absolute_pose_scale_bearings(
		rc.bearings, rc.points, rc.cameraExt, opt, &pose, &scale, &inliers);
	if (stats.num_inliers == 0 || !ISFINITE(scale) || scale <= 0)
		return 0;

	T.R = pose.R();
	T.scale = REAL(1) / scale;
	T.t = Point3(pose.t[0], pose.t[1], pose.t[2]) * T.scale;

	// keep the inliers as reprojection observations for the joint refinement
	for (size_t r = 0; r < inliers.size(); ++r) {
		const Image& imgRig = rigScene.images[rc.localIDs[r]];
		for (size_t i = 0; i < inliers[r].size(); ++i) {
			if (!inliers[r][i])
				continue;
			SeamObservation obs;
			obs.X = rc.points[r][i];
			obs.R = imgRig.R;
			obs.C = imgRig.C;
			obs.bearing = rc.bearings[r][i];
			obs.pixelPerRadian = rc.pixelPerRadian[r];
			obs.forward = pointsAreA;
			observations.push_back(obs);
		}
	}
	return (unsigned)stats.num_inliers;
}

// Refine one A -> B similarity against every inlier reprojection the two directions produced: A's
// points into B's cameras through T, B's points into A's cameras through T^-1. Seven parameters
// (unit quaternion on its manifold, translation, log scale) under a Huber loss set at the pixel
// threshold, so the seam ends up fitted to what the images saw rather than to either direction's
// minimal-solver consensus alone.
void RefineSeamTransform(const std::vector<SeamObservation>& observations, float maxReprojError, Transform& T)
{
	if (observations.empty())
		return;
	Eigen::Matrix3d R;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			R(i, j) = T.R(i, j);
	const Eigen::Quaterniond quat0(R);
	double q[4] = {quat0.w(), quat0.x(), quat0.y(), quat0.z()};
	double t[3] = {T.t[0], T.t[1], T.t[2]};
	double logScale = LOGN(T.scale);

	ceres::Problem problem;
	ceres::LossFunction* loss = new ceres::HuberLoss(maxReprojError);
	for (const SeamObservation& obs : observations)
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<SeamReprojectionError, 3, 4, 3, 1>(new SeamReprojectionError(obs)),
			loss, q, t, &logScale);
	problem.SetManifold(q, new ceres::QuaternionManifold);

	ceres::Solver::Options options;
	// eight parameters against thousands of residuals: the normal equations are an 8x8 solve, while
	// a QR would factorize the whole Jacobian for the same answer
	options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 50;
	options.function_tolerance = 1e-8;
	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	if (!summary.IsSolutionUsable() || !ISFINITE(logScale))
		return;

	const Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
	T.R = Eigen::Matrix3d(quat.normalized().toRotationMatrix());
	T.t = Point3(t[0], t[1], t[2]);
	T.scale = EXP(logScale);
}

// Error of one seam observation under a candidate A -> B similarity, in the pixel units the
// threshold is set in: the angle between the predicted and the observed bearing, charged at the
// observing camera's own angular-to-pixel rate. This is the residual the joint refinement
// minimizes, so the same number decides an inlier before, during and after it.
REAL SeamObservationError(const SeamObservation& obs, const Transform& T, const Transform& TInv)
{
	const Point3 p((obs.forward ? T : TInv) * obs.X);
	const Point3 d(obs.R * (p - obs.C));
	const REAL len = norm(d);
	if (!(len > 0))
		return std::numeric_limits<REAL>::max();
	const REAL cosAngle = CLAMP((d / len).dot(normalized(obs.bearing)), REAL(-1), REAL(1));
	return obs.pixelPerRadian * ACOS(cosAngle);
}

// How many of the seam's observations a similarity places within the pixel threshold
unsigned CountSeamInliers(const std::vector<SeamObservation>& observations, const Transform& T, float maxReprojError)
{
	const Transform TInv(T.Invert());
	unsigned numInliers = 0;
	for (const SeamObservation& obs : observations)
		if (SeamObservationError(obs, T, TInv) <= maxReprojError)
			++numInliers;
	return numInliers;
}

// The second opinion on a seam only one direction could measure: take the other direction's
// correspondences — collected already, they were merely spread over too few rig cameras to solve a
// scale from — and keep the ones the estimate explains, as observations for the joint refinement.
// Returns how many of them it explained.
unsigned CollectExplainedObservations(
	const RigCorrespondences& rc, const Scene& rigScene, bool pointsAreA,
	const Transform& T, float maxReprojError, std::vector<SeamObservation>& observations)
{
	const Transform TInv(T.Invert());
	unsigned numExplained = 0;
	for (size_t r = 0; r < rc.cameraExt.size(); ++r) {
		const Image& imgRig = rigScene.images[rc.localIDs[r]];
		for (size_t i = 0; i < rc.points[r].size(); ++i) {
			SeamObservation obs;
			obs.X = rc.points[r][i];
			obs.R = imgRig.R;
			obs.C = imgRig.C;
			obs.bearing = rc.bearings[r][i];
			obs.pixelPerRadian = rc.pixelPerRadian[r];
			obs.forward = pointsAreA;
			if (SeamObservationError(obs, T, TInv) > maxReprojError)
				continue;
			observations.push_back(obs);
			++numExplained;
		}
	}
	return numExplained;
}

// Rig cameras sitting at distinct centres: a rig collapsed onto one point sees no parallax and
// cannot observe a scale, however many cameras it holds
unsigned CountDistinctRigCentres(const RigCorrespondences& rc, const Scene& rigScene)
{
	AABB3 bbox(true);
	for (IIndex localID : rc.localIDs)
		bbox.InsertFull(rigScene.images[localID].C);
	if (bbox.IsEmpty())
		return 0;
	const REAL eps = MAXF(bbox.GetSize().norm() * REAL(1e-4), std::numeric_limits<REAL>::epsilon());
	std::vector<Point3> centres;
	for (IIndex localID : rc.localIDs) {
		const Point3& C = rigScene.images[localID].C;
		bool distinct = true;
		for (const Point3& other : centres)
			if (norm(C - other) <= eps) { distinct = false; break; }
		if (distinct)
			centres.push_back(C);
	}
	return (unsigned)centres.size();
}

} // namespace

bool GlobalAlignment::EstimateSubScenePairs(
	const std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	std::vector<ScenePair>& scenePairs)
{
	BuildGlobalToLocalMap(localToGlobals);
	return EstimateRelativePoses(subScenes, scenePairs);
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateRelativePoses(
	const std::vector<Scene>& subScenes,
	std::vector<ScenePair>& scenePairs)
{
	ASSERT(!globalToLocal.empty());
	const uint32_t numSubScenes = (uint32_t)subScenes.size();
	const bool alignCameras = config.alignment == GlobalAlignmentConfig::ALIGN_CAMERAS;

	// Per-sub-scene cache: PairIdx(localImageID, featureID) -> 3D inlier-track position.
	// Only observations belonging to inlier tracks are indexed; outliers are excluded
	// because their triangulated positions are unreliable.
	std::vector<std::unordered_map<PairIdx, Point3>> sceneObsToTrackPos(numSubScenes);
	for (uint32_t sceneIdx = 0; sceneIdx < numSubScenes; ++sceneIdx) {
		const Scene& subScene = subScenes[sceneIdx];
		size_t numInlierObservations = 0;
		for (const Track& track : subScene.tracks)
			if (track.IsInlier())
				numInlierObservations += track.GetNumInliers();
		auto& obsToTrackPos = sceneObsToTrackPos[sceneIdx];
		obsToTrackPos.reserve(numInlierObservations);
		for (const Track& track : subScene.tracks) {
			if (track.IsInlier())
				for (const Observation& obs : track)
					obsToTrackPos.emplace(PairIdx(obs.imageID, obs.featureID), track.position);
		}
	}

	// Group cross-sub-scene image pairs by sub-scene pair. Each link remembers which
	// side of the image pair corresponds to sub-scene A vs B so feature indices
	// (queryIdx/trainIdx) can be mapped consistently.
	std::unordered_map<PairIdx, CLISTDEF0IDX(PairLink, uint32_t)> linksByScenePair;
	linksByScenePair.reserve(numSubScenes * 2);
	for (const ImagePair& pair : scene.pairs) {
		if (pair.GetNumWeightedInliers() < config.minCommonTracks)
			continue;
		auto it1 = globalToLocal.find(pair.ID1);
		auto it2 = globalToLocal.find(pair.ID2);
		if (it1 == globalToLocal.end() || it2 == globalToLocal.end())
			continue;
		const uint32_t scene1 = it1->second.first;
		const uint32_t scene2 = it2->second.first;
		if (scene1 == scene2)
			continue;
		PairLink link;
		link.pair = &pair;
		if (scene1 < scene2) {
			link.localIdA = it1->second.second;
			link.localIdB = it2->second.second;
			link.aIsQuery = true;
		} else {
			link.localIdA = it2->second.second;
			link.localIdB = it1->second.second;
			link.aIsQuery = false;
		}
		linksByScenePair[MakePairIdx(scene1, scene2)].emplace_back(link);
	}

	// the merge stage cannot reach the reconstruction's RANSAC settings, so the camera alignment
	// runs the absolute-pose estimator on the options the resection configures for its own PnP
	const RansacOptions resectionRansac = ResectionConfig().ransac;
	poselib::RansacOptions ransacOptions;
	ransacOptions.max_iterations = resectionRansac.max_iterations;
	ransacOptions.min_iterations = resectionRansac.min_iterations;
	ransacOptions.success_prob = resectionRansac.confidence;

	scenePairs.clear();
	unsigned numEstimated = 0;
	unsigned numSkippedPairs = 0;
	for (const auto& [pairIdx, links] : linksByScenePair) {
		const Scene& subSceneA = subScenes[pairIdx.i];
		const Scene& subSceneB = subScenes[pairIdx.j];
		const auto& obsToTrackPosA = sceneObsToTrackPos[pairIdx.i];
		const auto& obsToTrackPosB = sceneObsToTrackPos[pairIdx.j];

		Transform T;
		unsigned numInliers = 0, numCorrespondences = 0;
		if (alignCameras) {
			// Both directions: B's cameras against A's tracks, and A's cameras against B's. Every
			// cross pair contributes to at least one of them, which the 3D-3D correspondences
			// cannot promise on a seam whose target-side keypoints never joined a track.
			std::vector<SeamObservation> obsAB, obsBA;
			Transform T_AB, T_BA;
			const RigCorrespondences rcAB = CollectRigCorrespondences(
				subSceneB, subSceneA, obsToTrackPosA, links, true, config.maxReprojError);
			const RigCorrespondences rcBA = CollectRigCorrespondences(
				subSceneA, subSceneB, obsToTrackPosB, links, false, config.maxReprojError);
			const unsigned inliersAB = EstimateRigAgainstPoints(rcAB, subSceneB, ransacOptions, true, T_AB, obsAB);
			const unsigned inliersBA = EstimateRigAgainstPoints(rcBA, subSceneA, ransacOptions, false, T_BA, obsBA);

			// A direction is MEASURED when it produced enough support to connect two sub-scenes.
			// Whether it may be believed is decided below, against the other direction — never on
			// its own numbers alone, because the estimator has no way of knowing that the block it
			// registered so confidently is the wrong one.
			const bool measuredAB = inliersAB >= config.minCommonTracks;
			const bool measuredBA = inliersBA >= config.minCommonTracks;
			DEBUG_ULTIMATE("Sub-scene pair (%u, %u) camera alignment: %u cameras of B on %u points of A -> %u inliers%s; "
				"%u cameras of A on %u points of B -> %u inliers%s",
				pairIdx.i, pairIdx.j,
				(unsigned)rcAB.cameraExt.size(), rcAB.numCorrespondences, inliersAB, measuredAB ? "" : " (unmeasured)",
				(unsigned)rcBA.cameraExt.size(), rcBA.numCorrespondences, inliersBA, measuredBA ? "" : " (unmeasured)");
			if (!measuredAB && !measuredBA) {
				++numSkippedPairs;
				continue;
			}

			// T_BA maps B into A, so its inverse is the second opinion on the A -> B similarity
			const Transform T_BA_inv = measuredBA ? T_BA.Invert() : Transform();
			String verdict;
			std::vector<SeamObservation>* seamObs;
			if (measuredAB && measuredBA) {
				// two independent measurements of the same seam: they decide it between them,
				// before any inlier share is looked at. If they disagree the seam is rejected,
				// never resolved in favour of the better supported one, because a seam accepted
				// wrong merges a whole block into the wrong place.
				const REAL errRot = R2D(ACOS(ComputeAngle(Matrix3x3(T_AB.R), Matrix3x3(T_BA_inv.R))));
				const REAL errScale = MAXF(T_AB.scale / T_BA_inv.scale, T_BA_inv.scale / T_AB.scale);
				if (errRot > config.maxSimRotationError || errScale > config.maxSimScaleRatio) {
					DEBUG("warning: sub-scene pair (%u, %u): the two alignment directions disagree by %.2f deg and "
						"%.1f%% in scale (B rig %u/%u inliers at scale %.4g, A rig %u/%u inliers at scale %.4g); seam rejected",
						pairIdx.i, pairIdx.j, errRot, (errScale - 1) * 100,
						inliersAB, rcAB.numCorrespondences, T_AB.scale,
						inliersBA, rcBA.numCorrespondences, T_BA_inv.scale);
					++numSkippedPairs;
					continue;
				}
				T = inliersAB >= inliersBA ? T_AB : T_BA_inv;
				numInliers = inliersAB + inliersBA;
				numCorrespondences = rcAB.numCorrespondences + rcBA.numCorrespondences;
				obsAB.insert(obsAB.end(), obsBA.begin(), obsBA.end());
				seamObs = &obsAB;
				verdict = String::FormatString("both directions (agreeing to %.2f deg and %.1f%%)",
					errRot, (errScale - 1) * 100);
			} else {
				// Only one direction was measurable, so the seam has no second estimate to be held
				// against. It must earn the right to stand on the other direction's data instead.
				const bool forward = measuredAB;
				const RigCorrespondences& rcOwn = forward ? rcAB : rcBA;
				const RigCorrespondences& rcRev = forward ? rcBA : rcAB;
				T = forward ? T_AB : T_BA_inv;
				numInliers = forward ? inliersAB : inliersBA;
				numCorrespondences = rcOwn.numCorrespondences;
				seamObs = forward ? &obsAB : &obsBA;
				const double ratio = (double)numInliers / (double)MAXF(numCorrespondences, 1u);
				if (ratio < config.minCameraInlierRatio) {
					DEBUG_ULTIMATE("Sub-scene pair (%u, %u): skipped: one direction, weak (%s rig alone at %.1f%% of %u correspondences)",
						pairIdx.i, pairIdx.j, forward ? "B" : "A", ratio * 100, numCorrespondences);
					++numSkippedPairs;
					continue;
				}
				if (!rcRev.cameraExt.empty()) {
					// the reverse direction has cameras, just not enough of them to solve a scale
					// from: its correspondences still say whether this estimate explains them
					const unsigned explained = CollectExplainedObservations(
						rcRev, forward ? subSceneA : subSceneB, !forward, T, config.maxReprojError, *seamObs);
					const double revRatio = (double)explained / (double)MAXF(rcRev.numCorrespondences, 1u);
					if (explained < config.minCommonTracks || revRatio < ratio / 2) {
						DEBUG_ULTIMATE("Sub-scene pair (%u, %u): skipped: one direction, weak (%s rig at %.1f%%, explaining only %u/%u the other way)",
							pairIdx.i, pairIdx.j, forward ? "B" : "A", ratio * 100, explained, rcRev.numCorrespondences);
						++numSkippedPairs;
						continue;
					}
					verdict = String::FormatString("one direction, verified %u/%u on the other",
						explained, rcRev.numCorrespondences);
					numInliers += explained;
					numCorrespondences += rcRev.numCorrespondences;
				} else {
					// nothing to verify against at all: the seam stands only if the one direction
					// is strong enough that no plausible amount of noise produced it
					const unsigned numCentres = CountDistinctRigCentres(rcOwn, forward ? subSceneB : subSceneA);
					if (numCentres < 3 || numInliers < 4 * config.minCommonTracks ||
						ratio < 2 * config.minCameraInlierRatio) {
						DEBUG_ULTIMATE("Sub-scene pair (%u, %u): skipped: one direction, weak (%s rig of %u distinct centres, %u inliers at %.1f%%, nothing to verify against)",
							pairIdx.i, pairIdx.j, forward ? "B" : "A", numCentres, numInliers, ratio * 100);
						++numSkippedPairs;
						continue;
					}
					verdict = String::FormatString("one direction, unverified, strong (%s rig of %u centres)",
						forward ? "B" : "A", numCentres);
				}
			}
			// the refinement fits the seam to what the images saw; its effect on the data it was
			// fitted to is the only measure of it there is on a real capture
			const unsigned inliersBefore = CountSeamInliers(*seamObs, T, config.maxReprojError);
			RefineSeamTransform(*seamObs, config.maxReprojError, T);
			if (!ISFINITE(T.scale) || T.scale <= 0) {
				DEBUG("warning: sub-scene pair (%u, %u): degenerate similarity; seam rejected", pairIdx.i, pairIdx.j);
				++numSkippedPairs;
				continue;
			}
			DEBUG_ULTIMATE("Sub-scene pair (%u, %u) Sim(3): scale=%.4g, inliers %u/%u, %s, refinement %u -> %u within %g px",
				pairIdx.i, pairIdx.j, T.scale, numInliers, numCorrespondences, verdict.c_str(),
				inliersBefore, CountSeamInliers(*seamObs, T, config.maxReprojError), config.maxReprojError);
		} else {
			// Collect 3D-3D correspondences: for each cross-sub-scene match whose
			// endpoints both lie on an existing inlier track, push the two 3D
			// positions (each in its own sub-scene's local frame).
			Point3Arr srcPoints, dstPoints;
			for (const PairLink& link : links) {
				ASSERT(link.localIdA < subSceneA.images.size() && link.localIdB < subSceneB.images.size());
				const Image& imgA = subSceneA.images[link.localIdA];
				const Image& imgB = subSceneB.images[link.localIdB];
				if (!imgA.IsValid() || !imgB.IsValid())
					continue;

				// the track-forming set, not the sparse count: the correspondence search below only
				// keeps a match whose two endpoints already lie on inlier tracks of the two sub-scenes,
				// and those tracks were built from this same set -- bounding it by the sparse count
				// would skip correspondences the tracks prove exist
				// clamped by the array: the loop dereferences matches[i] before anything inspects the
				// DMatch, so a count that outran `matches` would be an out-of-bounds read
				const unsigned numMatches = MINF(link.pair->GetNumTrackFormingMatches(), (unsigned)link.pair->matches.size());
				for (unsigned i = 0; i < numMatches; ++i) {
					const DMatch& match = link.pair->matches[i];
					const uint32_t featureA = link.aIsQuery ? match.queryIdx : match.trainIdx;
					const uint32_t featureB = link.aIsQuery ? match.trainIdx : match.queryIdx;
					const auto itA = obsToTrackPosA.find(PairIdx(link.localIdA, featureA));
					if (itA == obsToTrackPosA.end())
						continue;
					const auto itB = obsToTrackPosB.find(PairIdx(link.localIdB, featureB));
					if (itB == obsToTrackPosB.end())
						continue;
					srcPoints.emplace_back(itA->second);
					dstPoints.emplace_back(itB->second);
				}
			}

			if (srcPoints.size() < config.minCommonTracks) {
				DEBUG_ULTIMATE("Sub-scene pair (%u, %u): skipped (only %u 3D-3D correspondences, need >= %u)",
					pairIdx.i, pairIdx.j, (unsigned)srcPoints.size(), config.minCommonTracks);
				++numSkippedPairs;
				continue;
			}

			// Characteristic length scale for the RANSAC threshold: a fraction (default 1%) of the
			// destination point cloud's bounding-box diagonal. Using a relative scale keeps the
			// criterion invariant to each sub-scene's arbitrary units.
			AABB3 dstBbox(true);
			for (const Point3& p : dstPoints)
				dstBbox.InsertFull(p);
			if (dstBbox.IsEmpty()) {
				++numSkippedPairs;
				continue;
			}
			const double threshold = config.simInlierThresholdFactor * dstBbox.GetSize().norm();

			numCorrespondences = (unsigned)srcPoints.size();
			numInliers = EstimateSimilarityTransform(srcPoints, dstPoints, T, threshold, true, config.simRansacMaxIters);
			if (numInliers == 0) {
				DEBUG_ULTIMATE("warning: sub-scene pair (%u, %u): Sim(3) RANSAC failed (%u correspondences)",
					pairIdx.i, pairIdx.j, numCorrespondences);
				++numSkippedPairs;
				continue;
			}
			if (numInliers < config.minCommonTracks) {
				DEBUG_ULTIMATE("warning: sub-scene pair (%u, %u): skipped (too few inliers %u/%u)",
					pairIdx.i, pairIdx.j, numInliers, numCorrespondences);
				++numSkippedPairs;
				continue;
			}
			const double inlierRatio = (double)numInliers / (double)numCorrespondences;
			if (inlierRatio < config.minSimInlierRatio) {
				DEBUG_ULTIMATE("warning: sub-scene pair (%u, %u): skipped (low inlier ratio %.1f%% = %u/%u)",
					pairIdx.i, pairIdx.j, inlierRatio * 100.0, numInliers, numCorrespondences);
				++numSkippedPairs;
				continue;
			}
			DEBUG_ULTIMATE("Sub-scene pair (%u, %u) Sim(3): scale=%.4g, inliers %u/%u (%.1f%%)",
				pairIdx.i, pairIdx.j, T.scale, numInliers, numCorrespondences, inlierRatio * 100.0);
		}

		ScenePair sp;
		sp.sceneA = pairIdx.i;
		sp.sceneB = pairIdx.j;
		sp.relativeTransform = T;
		sp.numInliers = numInliers;
		scenePairs.push_back(sp);
		++numEstimated;
	}

	std::sort(scenePairs.begin(), scenePairs.end(), [](const ScenePair& a, const ScenePair& b) {
		return a.sceneA < b.sceneA || (a.sceneA == b.sceneA && a.sceneB < b.sceneB);
	});

	DEBUG("Estimated %u relative Sim(3) transforms between sub-scenes (%u skipped, %s alignment)",
		numEstimated, numSkippedPairs, alignCameras ? "camera" : "point");
	return !scenePairs.empty();
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateGlobalRotations(
	std::vector<ScenePair>& scenePairs,
	const uint32_t numSubScenes,
	std::vector<Point3d>& globalRotations)
{
	// Convert scene pairs to rotation pairs (kept 1:1 with scenePairs by index)
	std::vector<RotationPair> rotationPairs;
	rotationPairs.reserve(scenePairs.size());

	for (const ScenePair& sp : scenePairs) {
		RotationPair rp;
		rp.idxA = sp.sceneA;
		rp.idxB = sp.sceneB;
		rp.relativeRotation = sp.relativeTransform.R;
		rp.weight = (float)sp.numInliers;
		rotationPairs.push_back(rp);
	}

	// Use global rotation estimator
	GlobalRotationEstimatorOptions rotOptions;
	rotOptions.skipInitialization = false;
	rotOptions.useWeight = true;

	GlobalRotationEstimator rotEstimator(rotOptions);
	if (!rotEstimator.EstimateRotations(rotationPairs, numSubScenes, globalRotations)) {
		VERBOSE("error: rotation averaging failed");
		return false;
	}

	// Filter relative rotations inconsistent with global estimates and re-solve
	const unsigned numFiltered = GlobalRotationEstimator::FilterRelativeRotations(globalRotations, rotationPairs);
	if (numFiltered > 0) {
		DEBUG("Re-estimating global rotations after filtering %u pairs", numFiltered);
		globalRotations.clear();
		if (!rotEstimator.EstimateRotations(rotationPairs, numSubScenes, globalRotations)) {
			VERBOSE("error: rotation averaging failed after filtering");
			return false;
		}
	}

	// Prune rotation-inconsistent pairs from scenePairs in place. FilterRelativeRotations zeroes
	// (does not remove) the weight of pairs whose relative rotation disagrees with the averaged
	// global rotations, and rotationPairs stays 1:1 with scenePairs, so a single index compaction
	// keeps only the survivors. Downstream scale/translation averaging then use only these.
	ASSERT(rotationPairs.size() == scenePairs.size());
	const unsigned numInputPairs = (unsigned)scenePairs.size();
	unsigned numKept = 0;
	FOREACH(i, scenePairs)
		if (rotationPairs[i].weight > 0)
			scenePairs[numKept++] = scenePairs[i];
	scenePairs.resize(numKept);

	DEBUG("Estimated %u global rotations (%u/%u rotation-consistent pairs)",
		(unsigned)globalRotations.size(), numKept, numInputPairs);
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateGlobalScales(
	const std::vector<ScenePair>& scenePairs,
	const uint32_t numSubScenes,
	std::vector<REAL>& globalScales)
{
	// Each relativeTransform satisfies p_B = (s_A/s_B) * R * p_A + t, so its
	// scale field is s_A/s_B. ScalePair expects the ratio in the opposite
	// direction (s_B/s_A), hence the reciprocal below.
	std::vector<ScalePair> scalePairs;
	scalePairs.reserve(scenePairs.size());
	for (const ScenePair& sp : scenePairs) {
		if (sp.relativeTransform.scale <= 0)
			continue;
		ScalePair scalePair;
		scalePair.idxA = sp.sceneA;
		scalePair.idxB = sp.sceneB;
		scalePair.scaleRatio = REAL(1) / sp.relativeTransform.scale;
		scalePair.weight = (float)sp.numInliers;
		scalePairs.push_back(scalePair);
	}

	if (scalePairs.empty()) {
		// No scale information, use unit scales
		VERBOSE("warning: no scale pairs found, using unit scales");
		globalScales.resize(numSubScenes, REAL(1));
		return true;
	}

	// Estimate global scales
	GlobalScaleEstimator scaleEstimator;
	if (!scaleEstimator.EstimateScales(scalePairs, numSubScenes, globalScales)) {
		VERBOSE("error: scale averaging failed");
		return false;
	}

	DEBUG("Estimated %u global scales from %u pairs",
		(unsigned)globalScales.size(), (unsigned)scalePairs.size());
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateGlobalTranslations(
	const std::vector<ScenePair>& scenePairs,
	const std::vector<Point3d>& globalRotations,
	const std::vector<REAL>& globalScales,
	const uint32_t numSubScenes,
	std::vector<Point3>& globalTranslations)
{
	// Convert scene pairs to translation pairs
	std::vector<TranslationPair> translationPairs;
	translationPairs.reserve(scenePairs.size());

	for (const ScenePair& sp : scenePairs) {
		// Rotation averaging produces R_i mapping global→local, use transpose for local→global.
		const RMatrix RA(globalRotations[sp.sceneA]);
		const REAL sA = globalScales[sp.sceneA];

		// C_{BA}: position of scene B's origin expressed in scene A's local frame.
		// The relativeTransform satisfies p_B = scale * R * p_A + t (A -> B), so the
		// inverse maps B's origin (0 in B) to  -(1/scale) * R^T * t  in A.
		const Transform& T = sp.relativeTransform;
		const Point3 relT_local = (T.R.t() * T.t) * (-REAL(1) / T.scale);

		// Transform to global frame: t_B - t_A = s_A * R_A^T * C_{BA}
		const Point3 relT_global = sA * (RA.t() * relT_local);

		TranslationPair tp;
		tp.idxA = sp.sceneA;
		tp.idxB = sp.sceneB;
		tp.relativeTranslation = relT_global;
		tp.weight = (float)sp.numInliers;
		translationPairs.push_back(tp);
	}

	// Estimate global translations
	GlobalTranslationEstimator translationEstimator;
	if (!translationEstimator.EstimateTranslations(translationPairs, numSubScenes, globalTranslations)) {
		VERBOSE("error: translation averaging failed");
		return false;
	}

	DEBUG("Estimated %u global translations", (unsigned)globalTranslations.size());
	return true;
}
/*----------------------------------------------------------------*/

// the per-sub-scene local→global Sim(3) the merge applies: rotation averaging
// produces R_i mapping global→local (same convention as Image.R) while the
// similarity transform applies local→global, hence the transpose; the validator
// must construct exactly the transform the merge applies, so both build it here
static SEACAVE::Transform BuildGlobalTransform(const Point3d& rotation, REAL scale, const Point3& translation)
{
	SEACAVE::Transform G;
	G.R = RMatrix(rotation).t();
	G.scale = scale;
	G.t = translation;
	return G;
}

// The transform Stage 5 will apply to every sub-scene still in the merge, and the diagonal of the
// box its cameras span in its own frame -- the unit the translation residuals are normalized by.
static void BuildGlobalTransforms(
	const std::vector<Scene>& subScenes, const std::vector<bool>& skip,
	const std::vector<Point3d>& globalRotations, const std::vector<REAL>& globalScales,
	const std::vector<Point3>& globalTranslations,
	std::vector<Transform>& globalTransforms, std::vector<REAL>& camBoxDiags)
{
	const uint32_t numSubScenes = (uint32_t)subScenes.size();
	globalTransforms.assign(numSubScenes, Transform());
	camBoxDiags.assign(numSubScenes, REAL(0));
	for (uint32_t sceneIdx = 0; sceneIdx < numSubScenes; ++sceneIdx) {
		if (skip[sceneIdx])
			continue;
		globalTransforms[sceneIdx] = BuildGlobalTransform(
			globalRotations[sceneIdx], globalScales[sceneIdx], globalTranslations[sceneIdx]);
		AABB3 bbox(true);
		for (const Image& img : subScenes[sceneIdx].images)
			if (img.IsValid())
				bbox.InsertFull(img.C);
		if (!bbox.IsEmpty())
			camBoxDiags[sceneIdx] = bbox.GetSize().norm();
	}
}

// Sim(3) cycle residual of one measured seam: relativeTransform maps A-local to B-local and G_i maps
// each local frame to the global frame, so G_B*T_AB and G_A both map A-local to global and
// E = G_A^-1 * (G_B * T_AB) is the A-frame discrepancy between the measured edge and the averaged
// consensus (identity when perfectly consistent). Each component is also expressed as a factor of
// the limit it must stay under, so the three are comparable and the worst seam of a graph is well
// defined whichever way it is wrong.
struct SeamResidual {
	REAL scale;       // ratio, >= 1
	REAL rotation;    // degrees
	REAL translation; // fraction of the smaller end-point's global camera footprint
	REAL excess;      // largest of the three as a factor of its limit; conflicting above 1
};

static SeamResidual ComputeSeamResidual(
	const ScenePair& sp, const std::vector<Transform>& globalTransforms,
	const std::vector<REAL>& camBoxDiags, const GlobalAlignmentConfig& config)
{
	const Transform E = globalTransforms[sp.sceneA].Invert() * (globalTransforms[sp.sceneB] * sp.relativeTransform);
	SeamResidual res;
	res.scale = MAXF(E.scale, REAL(1) / E.scale);
	res.rotation = R2D(ACOS(ComputeAngle(Matrix3x3(E.R))));
	// E.t is in A's local frame: express the discrepancy in global units and compare
	// it against the smaller of the two global camera footprints, so the verdict does
	// not depend on which endpoint happens to have the lower sub-scene index
	const REAL diagA = camBoxDiags[sp.sceneA] * globalTransforms[sp.sceneA].scale;
	const REAL diagB = camBoxDiags[sp.sceneB] * globalTransforms[sp.sceneB].scale;
	const REAL diag = diagA > 0 && diagB > 0 ? MINF(diagA, diagB) : MAXF(diagA, diagB);
	res.translation = diag > 0 ? globalTransforms[sp.sceneA].scale * norm(E.t) / diag : REAL(0);
	res.excess = MAXF(MAXF(res.scale / config.maxSimScaleRatio, res.rotation / config.maxSimRotationError),
		res.translation / config.maxSimTranslationError);
	return res;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::PruneConflictingSeams(
	const std::vector<Scene>& subScenes,
	std::vector<ScenePair>& scenePairs,
	const std::vector<Point3d>& globalRotations,
	std::vector<REAL>& globalScales,
	std::vector<Point3>& globalTranslations,
	std::vector<bool>& demoted)
{
	const uint32_t numSubScenes = (uint32_t)subScenes.size();
	// every round drops exactly one seam, so there can be no more rounds than seams
	const size_t maxRounds = scenePairs.size();
	for (size_t round = 0; round < maxRounds; ++round) {
		// a seam graph with no cycle is reproduced exactly by the averaging: every residual is
		// zero and no seam can be indicted, however wrong it is
		DisjointSet<uint32_t> ds(numSubScenes);
		for (const ScenePair& sp : scenePairs)
			ds.Union(sp.sceneA, sp.sceneB);
		// only active sub-scenes are ever united, so the root of an active one is active too and
		// the components are counted by their roots
		unsigned numNodes = 0, numComponents = 0;
		for (uint32_t s = 0; s < numSubScenes; ++s)
			if (!demoted[s]) {
				++numNodes;
				if (ds.Find(s) == s)
					++numComponents;
			}
		if (scenePairs.size() + numComponents <= numNodes) {
			if (round == 0)
				VERBOSE("Seam graph is a tree: %u seams over %u sub-scenes, no cycle to validate",
					(unsigned)scenePairs.size(), numNodes);
			return true;
		}

		// score every seam against the consensus and take the one that contradicts it most
		std::vector<Transform> globalTransforms;
		std::vector<REAL> camBoxDiags;
		BuildGlobalTransforms(subScenes, demoted, globalRotations, globalScales, globalTranslations,
			globalTransforms, camBoxDiags);
		size_t worst = scenePairs.size();
		SeamResidual worstRes = {};
		FOREACH(idx, scenePairs) {
			const SeamResidual res = ComputeSeamResidual(scenePairs[idx], globalTransforms, camBoxDiags, config);
			if (res.excess > 1 && (worst == scenePairs.size() || res.excess > worstRes.excess)) {
				worst = idx;
				worstRes = res;
			}
		}
		if (worst == scenePairs.size())
			return true;

		// dropping a seam that is not a bridge only removes evidence; dropping one that is also
		// cuts a side loose, and a side with no link left to the consensus cannot be placed by it
		const ScenePair worstPair = scenePairs[worst];
		DisjointSet<uint32_t> dsCut(numSubScenes);
		FOREACH(idx, scenePairs)
			if (idx != worst)
				dsCut.Union(scenePairs[idx].sceneA, scenePairs[idx].sceneB);
		const bool bridge = dsCut.Find(worstPair.sceneA) != dsCut.Find(worstPair.sceneB);
		VERBOSE("Sub-scene pair (%u, %u) contradicts the averaged alignment by %.1fx its limit "
			"(scale %.1f%%, rotation %.2f deg, translation %.2f%%, weight %u); dropping the seam",
			worstPair.sceneA, worstPair.sceneB, worstRes.excess, (worstRes.scale - 1) * 100,
			worstRes.rotation, worstRes.translation * 100, worstPair.numInliers);
		if (bridge) {
			const uint32_t rootA = dsCut.Find(worstPair.sceneA), rootB = dsCut.Find(worstPair.sceneB);
			unsigned imagesA = 0, imagesB = 0;
			for (uint32_t s = 0; s < numSubScenes; ++s) {
				if (demoted[s])
					continue;
				const uint32_t root = dsCut.Find(s);
				if (root == rootA)
					imagesA += subScenes[s].status.nCalibratedImages;
				else if (root == rootB)
					imagesB += subScenes[s].status.nCalibratedImages;
			}
			const uint32_t losingRoot = imagesA <= imagesB ? rootA : rootB;
			unsigned numDemoted = 0;
			for (uint32_t s = 0; s < numSubScenes; ++s)
				if (!demoted[s] && dsCut.Find(s) == losingRoot) { demoted[s] = true; ++numDemoted; }
			VERBOSE("Sub-scene pair (%u, %u) was the only link of its side; demoting %u sub-scene(s) "
				"(%u images) to be rebuilt by resection", worstPair.sceneA, worstPair.sceneB,
				numDemoted, MINF(imagesA, imagesB));
		}
		scenePairs.erase(scenePairs.begin() + worst);
		if (bridge)
			scenePairs.erase(std::remove_if(scenePairs.begin(), scenePairs.end(),
				[&demoted](const ScenePair& sp) { return demoted[sp.sceneA] || demoted[sp.sceneB]; }),
				scenePairs.end());

		// a single survivor defines the gauge by itself; nothing left to average
		const unsigned numActive = numSubScenes - (unsigned)std::count(demoted.begin(), demoted.end(), true);
		if (numActive < 2 || scenePairs.empty())
			return true;
		globalScales.clear();
		globalTranslations.clear();
		if (!EstimateGlobalScales(scenePairs, numSubScenes, globalScales) ||
			!EstimateGlobalTranslations(scenePairs, globalRotations, globalScales, numSubScenes, globalTranslations)) {
			VERBOSE("error: failed to re-average scales/translations after dropping a seam");
			return false;
		}
	}
	return true;
}
/*----------------------------------------------------------------*/

std::vector<bool> GlobalAlignment::ValidateAlignment(
	const std::vector<Scene>& subScenes,
	const std::vector<bool>& mergeMask,
	const std::vector<ScenePair>& scenePairs,
	const std::vector<Point3d>& globalRotations,
	const std::vector<REAL>& globalScales,
	const std::vector<Point3>& globalTranslations) const
{
	ASSERT(mergeMask.size() == subScenes.size());
	const uint32_t numSubScenes = (uint32_t)subScenes.size();
	// sub-scenes rotation averaging could not place have no usable transform
	std::vector<bool> demoted(numSubScenes);
	for (uint32_t s = 0; s < numSubScenes; ++s)
		demoted[s] = !mergeMask[s];

	// Per-sub-scene transform Stage 5 will apply, plus the local camera-bbox diagonal
	// used to normalize the translation residuals.
	std::vector<Transform> globalTransforms;
	std::vector<REAL> camBoxDiags;
	BuildGlobalTransforms(subScenes, demoted, globalRotations, globalScales, globalTranslations,
		globalTransforms, camBoxDiags);

	// Sim(3) cycle residual per surviving edge (see ComputeSeamResidual)
	struct EdgeStat {
		uint32_t sceneA, sceneB;
		float weight;
		bool conflicting;
	};
	std::vector<EdgeStat> edges;
	edges.reserve(scenePairs.size());
	for (const ScenePair& sp : scenePairs) {
		ASSERT(!demoted[sp.sceneA] && !demoted[sp.sceneB]);
		const SeamResidual res = ComputeSeamResidual(sp, globalTransforms, camBoxDiags, config);
		const unsigned weight = MINF(sp.numInliers, 1000u);
		VERBOSE("Sub-scene pair (%u, %u) similarity residuals: scale %.1f%%, rotation %.2f deg, translation %.2f%% (weight %u)",
			sp.sceneA, sp.sceneB, (res.scale - 1) * 100, res.rotation, res.translation * 100, weight);
		edges.push_back({sp.sceneA, sp.sceneB, (float)weight, res.excess > 1});
	}

	// Vote out the node most dominated by conflicting cycle evidence, one at a time; a node
	// with a single incident edge is satisfied exactly by the averaging, so it carries no
	// cycle evidence and can never be flagged.
	for (;;) {
		uint32_t worst = NO_ID;
		float worstFrac = 0.5f;
		for (uint32_t s = 0; s < numSubScenes; ++s) {
			if (demoted[s])
				continue;
			float total = 0.f, conflict = 0.f;
			unsigned numEdges = 0, numConflicting = 0;
			for (const EdgeStat& e : edges) {
				if (e.sceneA != s && e.sceneB != s)
					continue;
				if (demoted[e.sceneA] || demoted[e.sceneB])
					continue;
				++numEdges;
				total += e.weight;
				if (e.conflicting) {
					++numConflicting;
					conflict += e.weight;
				}
			}
			if (numEdges < 2 || numConflicting < 2)
				continue;
			const float frac = conflict / total;
			if (frac > worstFrac) {
				worstFrac = frac;
				worst = s;
			}
		}
		if (worst == NO_ID)
			break;
		VERBOSE("Sub-scene %u misaligned by similarity cycle consistency (%.0f%% conflicting edge weight); demoting to be rebuilt by resection",
			worst, worstFrac * 100.f);
		demoted[worst] = true;
	}

	// Never demote everything: keep as anchor the largest sub-scene rotation averaging placed
	// (an unplaced one has no transform to anchor with)
	if (std::find(demoted.begin(), demoted.end(), false) == demoted.end()) {
		const uint32_t best = FindLargestSubScene(subScenes, &mergeMask);
		ASSERT(best != NO_ID); // the caller merges only when at least one sub-scene was placed
		demoted[best] = false;
		VERBOSE("warning: all sub-scenes failed validation; keeping sub-scene %u as anchor", best);
	}
	return demoted;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::RefineDemotedAlignment(
	const std::vector<Scene>& subScenes,
	std::vector<ScenePair>& scenePairs,
	const std::vector<Point3d>& globalRotations,
	std::vector<REAL>& globalScales,
	std::vector<Point3>& globalTranslations,
	std::vector<bool>& demoted)
{
	const uint32_t numSubScenes = (uint32_t)subScenes.size();
	for (;;) {
		// Demoting can disconnect the pair graph, while scale and translation averaging pin a
		// single gauge node, so keep only the component holding the most calibrated images.
		DisjointSet<uint32_t> ds(numSubScenes);
		for (const ScenePair& sp : scenePairs)
			if (!demoted[sp.sceneA] && !demoted[sp.sceneB])
				ds.Union(sp.sceneA, sp.sceneB);
		std::vector<unsigned> componentImages(numSubScenes, 0);
		for (uint32_t s = 0; s < numSubScenes; ++s)
			if (!demoted[s])
				componentImages[ds.Find(s)] += subScenes[s].status.nCalibratedImages;
		uint32_t bestRoot = NO_ID;
		for (uint32_t s = 0; s < numSubScenes; ++s) {
			if (demoted[s])
				continue;
			const uint32_t root = ds.Find(s);
			if (bestRoot == NO_ID || componentImages[root] > componentImages[bestRoot])
				bestRoot = root;
		}
		for (uint32_t s = 0; s < numSubScenes; ++s)
			if (!demoted[s] && ds.Find(s) != bestRoot) {
				VERBOSE("Sub-scene %u disconnected from the merge component; demoting to be rebuilt by resection", s);
				demoted[s] = true;
			}
		scenePairs.erase(std::remove_if(scenePairs.begin(), scenePairs.end(),
			[&demoted](const ScenePair& sp) { return demoted[sp.sceneA] || demoted[sp.sceneB]; }),
			scenePairs.end());

		// A single survivor defines the gauge by itself; nothing left to average
		const unsigned numActive = numSubScenes - (unsigned)std::count(demoted.begin(), demoted.end(), true);
		if (numActive < 2 || scenePairs.empty())
			return true;

		globalScales.clear();
		globalTranslations.clear();
		if (!EstimateGlobalScales(scenePairs, numSubScenes, globalScales) ||
			!EstimateGlobalTranslations(scenePairs, globalRotations, globalScales, numSubScenes, globalTranslations)) {
			VERBOSE("error: failed to re-average scales/translations after demotions");
			return false;
		}

		std::vector<bool> keepMask(numSubScenes);
		for (uint32_t s = 0; s < numSubScenes; ++s)
			keepMask[s] = !demoted[s];
		std::vector<bool> revalidated = ValidateAlignment(subScenes, keepMask, scenePairs,
			globalRotations, globalScales, globalTranslations);
		if (revalidated == demoted)
			return true;
		demoted = std::move(revalidated);
	}
}
/*----------------------------------------------------------------*/


bool GlobalAlignment::MergeTransformedScenes(
	std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	const std::vector<Point3d>& globalRotations,
	const std::vector<REAL>& globalScales,
	const std::vector<Point3>& globalTranslations,
	const std::vector<bool>& demoted)
{
	// Transform each trusted sub-scene into the global frame. Demoted sub-scenes are merged
	// without poses below, so transforming them would be wasted work — and those rotation
	// averaging left unplaced (always demoted) have unconstrained averaged transforms that
	// would inject NaN/garbage poses.
	FOREACH(sceneIdx, subScenes) {
		if (demoted[sceneIdx])
			continue;
		Scene& subScene = subScenes[sceneIdx];

		// Apply the similarity transform to the sub-scene
		subScene.Transform(BuildGlobalTransform(
			globalRotations[sceneIdx], globalScales[sceneIdx], globalTranslations[sceneIdx]));

		#if GLOBALALIGNMENT_DEBUG
		// Export aligned sub-scene for debugging
		subScene.ExportPLY(String::FormatString("subscene_%u_aligned.ply", sceneIdx));
		#endif
	}

	// Demoted sub-scenes (see ValidateAlignment) are merged without poses, to be rebuilt by
	// the post-merge resection.
	std::vector<bool> untrustedImages(scene.images.size(), false);
	FOREACH(sceneIdx, subScenes)
		if (demoted[sceneIdx])
			for (const IIndex globalID : localToGlobals[sceneIdx])
				if (globalID != NO_ID && globalID < scene.images.size())
					untrustedImages[globalID] = true;

	// Track per-camera accumulation counts; destination cameras accumulate directly
	std::unordered_map<Camera*, unsigned> cameraAccumCount;

	// Merge each sub-scene into the global scene
	scene.status.nCalibratedImages = 0;
	unsigned numMerged = 0;
	FOREACH(sceneIdx, subScenes) {
		Scene& subScene = subScenes[sceneIdx];
		const IIndexArr& localToGlobal = localToGlobals[sceneIdx];
		const bool trusted = !demoted[sceneIdx];
		if (trusted) {
			++numMerged;
			// Accumulate intrinsics from sub-scene cameras into destination cameras
			// (demoted sub-scenes are excluded: their drifted geometry taints intrinsics too)
			for (IIndex localID = 0; localID < subScene.images.size(); ++localID) {
				const IIndex globalID = localToGlobal[localID];
				if (globalID == NO_ID || globalID >= scene.images.size())
					continue;
				const Image& srcImg = subScene.images[localID];
				Image& dstImg = scene.images[globalID];
				if (!srcImg.IsValid() || !srcImg.HasCamera() || !dstImg.HasCamera())
					continue;
				auto [it, inserted] = cameraAccumCount.emplace(dstImg.pCamera, 0);
				if (inserted)
					dstImg.pCamera->ResetIntrinsics();
				dstImg.pCamera->AccumulateIntrinsics(*srcImg.pCamera);
				++it->second;
			}
		}

		// Merge into global scene
		MergeSingleScene(subScene, localToGlobal, trusted);
	}

	// Finalize intrinsics averaging
	for (const auto& [cam, count] : cameraAccumCount) {
		if (count > 0) {
			cam->ScaleIntrinsics(REAL(1) / count);
			DEBUG_EXTRA("Camera intrinsics averaged (%u sub-scenes): %s", count, cam->GetIntrinsicsString().c_str());
		}
	}

	// Merge sub-scene tracks and connect them via cross-sub-scene pairs
	MergeTracksWithCrossSubScenePairs(untrustedImages);
	FilterTracks(scene, 16.f, 0.5f);

	DEBUG("Merged %u/%u transformed sub-scenes (%u tracks, %u calibrated images)",
		numMerged, (unsigned)subScenes.size(), (unsigned)scene.tracks.size(), scene.status.nCalibratedImages);
	return true;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::MergeSingleScene(Scene& subScene, const IIndexArr& localToGlobal, bool trusted)
{
	// Copy image poses and move back keypoints/descriptors
	// (keypoints/descriptors were moved to sub-scenes during ExtractSubScene to save memory)
	for (IIndex localID = 0; localID < subScene.images.size(); ++localID) {
		const IIndex globalID = localToGlobal[localID];
		if (globalID == NO_ID || globalID >= scene.images.size())
			continue;

		Image& srcImg = subScene.images[localID];
		Image& dstImg = scene.images[globalID];

		if (srcImg.IsValid() && trusted) {
			if (!dstImg.IsValid())
				++scene.status.nCalibratedImages;
			dstImg.R = srcImg.R;
			dstImg.C = srcImg.C;
		}
		// Move keypoints/descriptors back from sub-scene to global scene, described-keypoint
		// boundary included: without it the dense keypoints the sub-scene carried would come back
		// indistinguishable from the described ones
		if (srcImg.HasFeatures() && !dstImg.HasFeatures())
			dstImg.MoveFeaturesFrom(srcImg);
	}

	// Remap and merge image-pairs from local sub-scene into global scene.
	// Ordering invariant: localToGlobal is sorted by global ID, so
	// localID1 < localID2 implies globalID1 < globalID2.
	for (ImagePair& srcPair : subScene.pairs) {
		if (srcPair.ID1 >= localToGlobal.size() || srcPair.ID2 >= localToGlobal.size())
			continue;
		IIndex globalID1 = localToGlobal[srcPair.ID1];
		IIndex globalID2 = localToGlobal[srcPair.ID2];
		if (globalID1 == NO_ID || globalID2 == NO_ID)
			continue;
		ASSERT(globalID1 < globalID2);
		srcPair.ID1 = globalID1;
		srcPair.ID2 = globalID2;
		ASSERT(scene.FindPair(srcPair.ID1, srcPair.ID2) == NULL);
		scene.pairs.emplace_back(std::move(srcPair));
	}

	// Merge tracks and colors together to keep indices aligned
	const bool hasColors = !subScene.colors.empty() && subScene.colors.size() == subScene.tracks.size();
	scene.tracks.reserve(scene.tracks.size() + subScene.tracks.size());
	if (hasColors)
		scene.colors.reserve(scene.colors.size() + subScene.tracks.size());

	FOREACH(srcIdx, subScene.tracks) {
		const Track& srcTrack = subScene.tracks[srcIdx];
		if (!srcTrack.IsValid())
			continue;

		Track dstTrack = srcTrack;

		// Remap observation image IDs from local to global (NO_ID marks an unmapped image)
		for (Observation& obs : dstTrack.observations) {
			ASSERT(obs.imageID < localToGlobal.size());
			obs.imageID = localToGlobal[obs.imageID];
		}

		// Remove invalid observations (decrement numInliers if an inlier was removed)
		for (IIndex i = dstTrack.observations.size(); i-- > 0; ) {
			if (dstTrack.observations[i].imageID == NO_ID) {
				if (i < dstTrack.numInliers)
					--dstTrack.numInliers;
				dstTrack.observations.RemoveAtMove(i);
			}
		}

		// Demoted sub-scene: keep the observations but none of the (drifted) 3D trust
		if (!trusted)
			dstTrack.numInliers = 0;

		if (!dstTrack.IsValid())
			continue;

		if (dstTrack.IsInlier())
			++scene.status.nTracks;
		scene.tracks.push_back(dstTrack);
		if (hasColors)
			scene.colors.push_back(subScene.colors[srcIdx]);
	}
}
/*----------------------------------------------------------------*/


void GlobalAlignment::MergeTracksWithCrossSubScenePairs(const std::vector<bool>& untrustedImages)
{
	// Per-root metadata for union-find: 3D position, inlier count, and the set of images the
	// track already observes (allocated only for the roots that hold a track, which are a
	// small fraction of the features; its presence is what marks a root as holding one)
	struct RootMeta {
		Point3 position{Point3::ZERO};
		uint32_t numInliers{0};
		std::unique_ptr<std::unordered_set<IIndex>> images;
		bool hasPosition{false};

		void InitImages() {
			if (!images)
				images = std::make_unique<std::unordered_set<IIndex>>();
		}
	};

	// Phase 1: Build featureOffsets and initialize DisjointSet from existing tracks
	//
	// Each feature across all images gets a unique global ID via featureOffsets:
	//   globalID = featureOffsets[imageID] + featureID
	// We then seed the union-find by unioning all observations within each
	// sub-scene track into a single set. This preserves the track structure
	// from each independently-reconstructed sub-scene.

	// Compute feature offsets for O(1) global ID lookup (same as BuildTracks)
	Unsigned32Arr featureOffsets(0, scene.images.size() + 1);
	uint32_t totalFeatures = 0;
	for (const Image& img : scene.images) {
		featureOffsets.push_back(totalFeatures);
		totalFeatures += (uint32_t)img.keypoints.size();
	}
	featureOffsets.push_back(totalFeatures); // sentinel
	if (totalFeatures == 0) {
		DEBUG("warning: no features for track merging");
		return;
	}

	DisjointSet<uint32_t> ds(totalFeatures);
	std::vector<RootMeta> rootMeta(totalFeatures);
	std::vector<bool> featureCounted(totalFeatures, false);

	// Initialize union-find sets from existing sub-scene tracks.
	// config.mergeTrackInliersOnly controls whether we seed with only inlier
	// observations (first numInliers entries, which passed reprojection filtering)
	// or all observations including outliers.
	const bool useOnlyInliers = config.mergeTrackInliersOnly;
	AABB3 bbox(true);
	for (const Track& track : scene.tracks) {
		if (!track.IsValid())
			continue;
		// Tracks from demoted sub-scenes carry no inlier/3D trust (numInliers=0), but their
		// observation structure must survive so the post-merge resection can re-triangulate
		// them: seed them with all observations and no position.
		const bool untrustedTrack = !track.IsInlier() &&
			track.observations[0].imageID < untrustedImages.size() &&
			untrustedImages[track.observations[0].imageID];
		const uint32_t numObs = untrustedTrack
			? (uint32_t)track.observations.size()
			: (useOnlyInliers ? (uint32_t)track.numInliers : (uint32_t)track.observations.size());
		if (numObs < 2)
			continue;
		// Union selected observations into one set
		uint32_t firstGid = NO_ID;
		for (uint32_t i = 0; i < numObs; ++i) {
			const Observation& obs = track.observations[i];
			if (obs.imageID >= scene.images.size())
				continue;
			if (obs.featureID >= scene.images[obs.imageID].keypoints.size())
				continue;
			const uint32_t gid = featureOffsets[obs.imageID] + obs.featureID;
			featureCounted[gid] = true;
			if (firstGid == NO_ID)
				firstGid = gid;
			else
				ds.Union(firstGid, gid);
		}
		if (firstGid == NO_ID)
			continue;
		// Store metadata at root
		const uint32_t root = ds.Find(firstGid);
		RootMeta& meta = rootMeta[root];
		meta.position = track.position;
		meta.numInliers = track.numInliers;
		meta.hasPosition = track.IsInlier();
		meta.InitImages();
		for (uint32_t i = 0; i < numObs; ++i) {
			const Observation& obs = track.observations[i];
			if (obs.imageID < scene.images.size() &&
				obs.featureID < scene.images[obs.imageID].keypoints.size())
				meta.images->emplace(obs.imageID);
		}
		if (meta.hasPosition)
			bbox.InsertFull(track.position);
	}

	// Compute proximity threshold from scene bounding box
	const REAL proximityThreshold = bbox.IsEmpty() ? REAL(0) : REAL(0.02) * bbox.GetSize().norm();

	// Phase 2: Process ONLY cross-sub-scene pairs (connecting pairs) to merge
	// tracks across sub-scene boundaries.
	//
	// A pair is a "connecting pair" if its two images belong to different sub-scenes.
	// Intra-sub-scene pairs are skipped: their tracks are already correctly formed
	// by BuildTracks during sub-scene reconstruction. Re-processing them here would
	// over-merge tracks within a sub-scene (because outlier observations removed
	// during reconstruction can lift the duplicate-image guard that originally kept
	// the tracks separate), bloating image sets and blocking legitimate cross-sub-scene
	// connections via the duplicate-image guard.
	unsigned numMerged = 0, numRejectedProximity = 0, numRejectedDupImage = 0, numNewPairTracks = 0;
	unsigned numCrossScenePairs = 0;

	// Ensure root has metadata and feature is counted exactly once;
	// new features from cross-sub-scene pairs are counted as additional observations
	// but do NOT increment numInliers (these are unvalidated matches, not verified inliers)
	auto AccumulateFeature = [&](uint32_t gid, IIndex imgID) {
		if (featureCounted[gid])
			return;
		featureCounted[gid] = true;
		RootMeta& meta = rootMeta[ds.Find(gid)];
		meta.InitImages();
		meta.images->emplace(imgID);
	};

	for (const ImagePair& pair : scene.pairs) {
		if (!pair.HasMatches())
			continue;
		ASSERT(pair.ID1 < scene.images.size() && pair.ID2 < scene.images.size());
		ASSERT(scene.images[pair.ID1].HasFeatures() && scene.images[pair.ID2].HasFeatures());
		// Filter: only process cross-sub-scene pairs.
		// Both images must be in globalToLocal (assigned to a sub-scene)
		// and must belong to different sub-scenes.
		const auto itA = globalToLocal.find(pair.ID1);
		const auto itB = globalToLocal.find(pair.ID2);
		if (itA == globalToLocal.end() || itB == globalToLocal.end())
			continue;
		if (itA->second.first == itB->second.first)
			continue; // same sub-scene, skip
		++numCrossScenePairs;

		const uint32_t offset1 = featureOffsets[pair.ID1];
		const uint32_t offset2 = featureOffsets[pair.ID2];
		// the track-forming set, the same bound BuildTracks' union-find uses: this is that same
		// union across sub-scene boundaries, so it must see the dense supplement too and must
		// still stop before the matches the strict filter rejected -- and, as there, clamped by the
		// array, since matches[i] is dereferenced before the index guard below reads the DMatch
		FOREACHRAW(i, MINF(pair.GetNumTrackFormingMatches(), (unsigned)pair.matches.size())) {
			const DMatch& match = pair.matches[i];
			if ((unsigned)match.queryIdx >= scene.images[pair.ID1].keypoints.size() ||
				(unsigned)match.trainIdx >= scene.images[pair.ID2].keypoints.size())
				continue;
			const uint32_t gid1 = offset1 + match.queryIdx;
			const uint32_t gid2 = offset2 + match.trainIdx;

			// Ensure both features are counted in their root metadata
			AccumulateFeature(gid1, pair.ID1);
			AccumulateFeature(gid2, pair.ID2);

			// Attempt union with image-uniqueness and 3D proximity guards
			ds.UnionIf(gid1, gid2,
				[&](uint32_t rootDst, uint32_t rootSrc) -> bool {
					RootMeta& metaDst = rootMeta[rootDst];
					RootMeta& metaSrc = rootMeta[rootSrc];
					ASSERT(metaDst.images && metaSrc.images);
					// Guard 1: reject if merging would create duplicate image observations
					for (const IIndex imgID : *metaSrc.images) {
						if (metaDst.images->count(imgID)) {
							++numRejectedDupImage;
							return false;
						}
					}
					// Guard 2: if both sides have triangulated 3D positions,
					// reject if they are too far apart (indicates false match)
					if (metaDst.hasPosition && metaSrc.hasPosition && proximityThreshold > 0) {
						if (norm(metaDst.position - metaSrc.position) > proximityThreshold) {
							++numRejectedProximity;
							return false;
						}
					}
					// Merge metadata: weighted-average 3D positions, merge image sets
					if (metaDst.hasPosition && metaSrc.hasPosition) {
						const REAL wDst = (REAL)metaDst.numInliers;
						const REAL wSrc = (REAL)metaSrc.numInliers;
						metaDst.position = (metaDst.position * wDst + metaSrc.position * wSrc) / (wDst + wSrc);
					} else if (metaSrc.hasPosition) {
						metaDst.position = metaSrc.position;
						metaDst.hasPosition = true;
					}
					metaDst.numInliers += metaSrc.numInliers;
					metaDst.images->insert(metaSrc.images->begin(), metaSrc.images->end());
					metaSrc.images.reset();
					++numMerged;
					return true;
				}
			);
		}
	}

	// Phase 3: Assemble final tracks grouped by union-find root
	std::unordered_map<uint32_t, ObservationArr> trackGroups;
	FOREACH(imgID, scene.images) {
		const Image& img = scene.images[imgID];
		const uint32_t offset = featureOffsets[imgID];
		for (uint32_t fid = 0; fid < (uint32_t)img.keypoints.size(); ++fid) {
			const uint32_t gid = offset + fid;
			const uint32_t root = ds.Find(gid);
			// Only include features that belong to a set with metadata (i.e., part of a track)
			if (!rootMeta[root].images)
				continue;
			trackGroups[root].emplace_back(imgID, fid);
		}
	}

	scene.tracks.Release();
	scene.colors.Release(); // colors indexed in parallel with tracks; must be rebuilt
	scene.status.nTracks = 0;
	scene.tracks.reserve(trackGroups.size());

	for (auto& [root, observations] : trackGroups) {
		if (observations.size() < 2)
			continue;
		observations.Sort();
		Track track;
		track.observations = std::move(observations);
		const RootMeta& meta = rootMeta[root];
		if (meta.hasPosition) {
			// Use averaged 3D position from merged sub-scene tracks;
			// numInliers from accumulated metadata (original inliers + cross-sub-scene additions)
			track.position = meta.position;
			track.numInliers = (uint8_t)MINF(MINF(meta.numInliers, (uint32_t)track.observations.size()), 255u);
			++scene.status.nTracks;
		} else {
			// New track without 3D: triangulate
			if (TriangulateSkewLLS(track, scene.images) >= 2) {
				++scene.status.nTracks;
				++numNewPairTracks;
			}
			// tracks with failed triangulation kept with numInliers=0,
			// excluded from BA until next triangulation attempt
		}
		scene.tracks.emplace_back(std::move(track));
	}

	DEBUG("Track merge: %u/%u tracks, %u cross-sub-scene merges from %u connecting pairs, "
		"%u new from pairs, %u rejected by proximity, %u rejected by duplicate image",
		scene.status.nTracks, scene.tracks.size(), numMerged, numCrossScenePairs,
		numNewPairTracks, numRejectedProximity, numRejectedDupImage);
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
