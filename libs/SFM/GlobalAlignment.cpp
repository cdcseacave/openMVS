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
#include "RobustAveraging.h"
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

	// Run the staged alignment pipeline; any stage failing breaks out to the fallback below.
	// Every failure occurs before the merge stage (Stage 5) consumes sub-scenes, so on failure
	// all sub-scenes are still intact and the fallback can keep the largest one.
	do {
		// Stage 1: measure the seam of every connected sub-scene pair (this also fills
		// globalToLocal, which the merge stage reads)
		std::vector<SeamCandidate> candidates;
		if (!EstimateSeamCandidates(subScenes, localToGlobals, candidates)) {
			VERBOSE("error: failed to estimate relative poses");
			break;
		}
		std::vector<ScenePair> scenePairs;
		CandidatesToScenePairs(candidates, scenePairs);
		// Every pair left disputed by its two candidates: nothing places one sub-scene against
		// another, and copying them in at their own local frames would merge them wrong.
		if (scenePairs.empty() && numSubScenes > 1) {
			VERBOSE("error: no sub-scene pair was settled");
			break;
		}

		// A single sub-scene has nothing to align against, so it is copied in as it is
		if (numSubScenes == 1) {
			VERBOSE("Single sub-scene, copying directly");
			MergeSingleScene(subScenes[0], localToGlobals[0], true);
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

// The bars of the seam rules that are not configurable, each with the rule it belongs to.
// A camera supports a transform only when this share of its correspondences are inliers.
constexpr float kVoteSupportFraction = 0.3f;
// A camera contradicts a transform when its inlier share falls under this.
constexpr float kVoteContraFraction = 0.1f;
// Cells per axis of the image grid a supporting camera's inlier keypoints must spread over.
constexpr unsigned kVoteGridCells = 4;
// Reweighting rounds of every averaging of the seam graph.
constexpr unsigned kRobustRounds = 10;
// A seam nothing corroborates stands alone only with this many times the tracks a seam needs at all.
constexpr unsigned kStrongAloneFactor = 4;

// One cross-sub-scene image pair, with the sub-scene each of its two images belongs to already
// resolved, so feature indices (queryIdx/trainIdx) can be mapped consistently.
struct PairLink {
	const ImagePair* pair;
	IIndex localIdA;
	IIndex localIdB;
	IIndex globalIdA;
	IIndex globalIdB;
	bool aIsQuery; // true if pair.ID1 belongs to sub-scene A (i.e. uses queryIdx)
};

// The cross pairs of one sub-scene pair, resolved so that side A is sub-scene `a`
void ResolvePairLinks(
	const Scene& scene, const std::unordered_map<IIndex, std::pair<uint32_t, IIndex>>& globalToLocal,
	const std::vector<uint32_t>& pairIndices, uint32_t a, std::vector<PairLink>& links)
{
	links.clear();
	links.reserve(pairIndices.size());
	for (uint32_t idx : pairIndices) {
		const ImagePair& pair = scene.pairs[idx];
		const auto it1 = globalToLocal.find(pair.ID1);
		const auto it2 = globalToLocal.find(pair.ID2);
		ASSERT(it1 != globalToLocal.end() && it2 != globalToLocal.end());
		PairLink link;
		link.pair = &pair;
		link.aIsQuery = it1->second.first == a;
		link.localIdA = link.aIsQuery ? it1->second.second : it2->second.second;
		link.localIdB = link.aIsQuery ? it2->second.second : it1->second.second;
		link.globalIdA = link.aIsQuery ? pair.ID1 : pair.ID2;
		link.globalIdB = link.aIsQuery ? pair.ID2 : pair.ID1;
		links.push_back(link);
	}
}

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
	std::vector<std::vector<SeamCorrespondence>> matches; // per rig camera, the two images and features behind each
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
	const std::vector<PairLink>& links, bool rigIsB, float maxReprojError)
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
				rc.matches.emplace_back();
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
			rc.matches[r].push_back(SeamCorrespondence{link.globalIdA, link.globalIdB, featureA, featureB});
			++rc.numCorrespondences;
		}
	}
	return rc;
}

// Flatten one direction's correspondences into seam observations, in the order the estimator sees
// them (grouped per rig camera), with the parallel list of the two images and features behind each.
void AppendRigObservations(
	const RigCorrespondences& rc, const Scene& rigScene, bool pointsAreA,
	std::vector<SeamObservation>& observations, std::vector<SeamCorrespondence>& correspondences)
{
	observations.reserve(observations.size() + rc.numCorrespondences);
	correspondences.reserve(correspondences.size() + rc.numCorrespondences);
	for (size_t r = 0; r < rc.cameraExt.size(); ++r) {
		const Image& imgRig = rigScene.images[rc.localIDs[r]];
		for (size_t i = 0; i < rc.points[r].size(); ++i) {
			const SeamCorrespondence& match = rc.matches[r][i];
			SeamObservation obs;
			obs.X = rc.points[r][i];
			obs.R = imgRig.R;
			obs.C = imgRig.C;
			obs.bearing = rc.bearings[r][i];
			obs.pixelPerRadian = rc.pixelPerRadian[r];
			obs.forward = pointsAreA;
			obs.rigImage = pointsAreA ? match.imageB : match.imageA;
			obs.pointImage = pointsAreA ? match.imageA : match.imageB;
			observations.push_back(obs);
			correspondences.push_back(match);
		}
	}
}

// One direction of the camera alignment. The estimator solves scale * p_rig = R * p_point + t,
// i.e. it scales the rig's centers into the frame the points live in, so the similarity mapping
// the point sub-scene into the rig sub-scene is p_rig = (1/scale) * R * p_point + (1/scale) * t.
// Every correspondence of the direction is appended to `observations`, tagged with which side of
// the seam it came from, and `inlierMask` grows with them to say which ones the estimator kept: it
// stays parallel to `observations`, so calling this once per direction leaves one mask over both.
// The seam is scored on all of them, whichever of the two directions ends up carrying it. Returns 0
// when the direction yields no usable estimate, the correspondences being appended all the same.
unsigned EstimateRigAgainstPoints(
	const RigCorrespondences& rc, const Scene& rigScene,
	const poselib::RansacOptions& ransac, bool pointsAreA, Transform& T,
	std::vector<SeamObservation>& observations, std::vector<SeamCorrespondence>& correspondences,
	std::vector<bool>& inlierMask)
{
	const size_t first = observations.size();
	ASSERT(inlierMask.size() == first);
	AppendRigObservations(rc, rigScene, pointsAreA, observations, correspondences);
	inlierMask.resize(observations.size(), false);

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

	// the mask runs in the same order the correspondences were appended in, behind whatever an
	// earlier direction already left in it
	size_t k = first;
	for (size_t r = 0; r < inliers.size(); ++r)
		for (size_t i = 0; i < inliers[r].size(); ++i, ++k)
			inlierMask[k] = inliers[r][i] != 0;
	ASSERT(k == inlierMask.size());
	return (unsigned)stats.num_inliers;
}

// The observations a mask marks as inliers, of one direction (forward 1 or 0) or of both (-1)
std::vector<SeamObservation> SelectInliers(
	const std::vector<SeamObservation>& observations, const std::vector<bool>& inlierMask, int forward = -1)
{
	ASSERT(observations.size() == inlierMask.size());
	std::vector<SeamObservation> inliers;
	for (size_t i = 0; i < observations.size(); ++i)
		if (inlierMask[i] && (forward < 0 || observations[i].forward == (forward != 0)))
			inliers.push_back(observations[i]);
	return inliers;
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

// Rig cameras sitting at distinct centres: a rig collapsed onto one point sees no parallax and
// cannot observe a scale, however many cameras it holds. The centres belong to one sub-scene, so
// they are comparable against that sub-scene's own extent.
unsigned CountDistinctRigCentres(const std::vector<Point3>& rigCentres)
{
	if (rigCentres.empty())
		return 0;
	// the box the centres span may be flat -- cameras on one circle share a height -- so its
	// diagonal, not its emptiness, says how far apart two centres have to be to be two
	AABB3 bbox(true);
	for (const Point3& C : rigCentres)
		bbox.InsertFull(C);
	const REAL eps = MAXF(bbox.GetSize().norm() * REAL(1e-4), std::numeric_limits<REAL>::epsilon());
	std::vector<Point3> distinctCentres;
	for (const Point3& C : rigCentres) {
		bool distinct = true;
		for (const Point3& other : distinctCentres)
			if (norm(C - other) <= eps) { distinct = false; break; }
		if (distinct)
			distinctCentres.push_back(C);
	}
	return (unsigned)distinctCentres.size();
}

// The merge stage cannot reach the reconstruction's RANSAC settings, so the camera alignment runs
// the absolute-pose estimator on the options the resection configures for its own PnP.
poselib::RansacOptions SeamRansacOptions()
{
	const RansacOptions resectionRansac = ResectionConfig().ransac;
	poselib::RansacOptions ransacOptions;
	ransacOptions.max_iterations = resectionRansac.max_iterations;
	ransacOptions.min_iterations = resectionRansac.min_iterations;
	ransacOptions.success_prob = resectionRansac.confidence;
	return ransacOptions;
}

// Mean of the distinct rig camera centres of one direction, in the rig block's own frame
Point3 RigCentroid(const std::vector<SeamObservation>& observations, bool forward)
{
	std::set<IIndex> seen;
	Point3 centroid(Point3::ZERO);
	unsigned numCentres = 0;
	for (const SeamObservation& obs : observations)
		if (obs.forward == forward && seen.insert(obs.rigImage).second) {
			centroid += obs.C;
			++numCentres;
		}
	return numCentres > 0 ? centroid / (REAL)numCentres : centroid;
}

// How a candidate was measured, for the log
const char* SourceWord(SeamCandidate::Source source)
{
	switch (source) {
	case SeamCandidate::RIG_B_ON_A: return "rig B on A";
	case SeamCandidate::RIG_A_ON_B: return "rig A on B";
	case SeamCandidate::POINTS: return "points";
	default: return "union";
	}
}

// Every camera's vote on one transform, the raw material the two vote bars are read from
void LogVotes(const char* label, const SeamScore& score)
{
	for (const CameraVote& vote : score.votes)
		DEBUG_ULTIMATE("%s camera %u: %u/%u inliers, coverage %.2f, %s",
			label, vote.image, vote.inliers, vote.correspondences, vote.coverage,
			vote.vote > 0 ? "support" : (vote.vote < 0 ? "contradict" : "abstain"));
}

// What one surviving candidate rests on. A candidate measured from one direction reports that
// direction as its own and the other as the cross one; a union or a 3D-3D candidate, having no
// single direction, reports the forward one as its own.
void LogSeamCandidate(uint32_t a, uint32_t b, const SeamCandidate& c)
{
	const int ownForward = c.source == SeamCandidate::RIG_A_ON_B ? 0 : 1;
	const int crossForward = 1 - ownForward;
	DEBUG("Seam (%u, %u) %s: inliers %u/%u own, %u/%u cross; votes A %u+/%u- B %u+/%u-; "
		"centres %u; scale %s; weight %.0f",
		a, b, SourceWord(c.source),
		c.NumInliers(ownForward), c.NumObservations(ownForward),
		c.NumInliers(crossForward), c.NumObservations(crossForward),
		c.score.support[0], c.score.contra[0], c.score.support[1], c.score.contra[1],
		c.score.centres, c.scaleObservable ? "observable" : "unobservable", c.weight);
}

} // namespace

void GlobalAlignment::RescaleSeamAboutRig(const Point3& rigCentre, bool rigIsB, REAL scale, Transform& T)
{
	const Point3 pointA(rigIsB ? T.Invert() * rigCentre : rigCentre);
	const Point3 pointB(rigIsB ? rigCentre : T * rigCentre);
	T.scale = scale;
	T.t = pointB - (T.R * pointA) * scale;
}
/*----------------------------------------------------------------*/

unsigned SeamCandidate::NumInliers(int forward) const
{
	const size_t numScored = MINF(observations.size(), score.inlierMask.size());
	unsigned num = 0;
	for (size_t i = 0; i < numScored; ++i)
		if (score.inlierMask[i] && (forward < 0 || observations[i].forward == (forward != 0)))
			++num;
	return num;
}

unsigned SeamCandidate::NumObservations(int forward) const
{
	if (forward < 0)
		return (unsigned)observations.size();
	unsigned num = 0;
	for (const SeamObservation& obs : observations)
		if (obs.forward == (forward != 0))
			++num;
	return num;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateSubScenePairs(
	const std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	std::vector<ScenePair>& scenePairs)
{
	std::vector<SeamCandidate> candidates;
	if (!EstimateSeamCandidates(subScenes, localToGlobals, candidates))
		return false;
	CandidatesToScenePairs(candidates, scenePairs);
	return !scenePairs.empty();
}
/*----------------------------------------------------------------*/

void GlobalAlignment::CandidatesToScenePairs(
	const std::vector<SeamCandidate>& candidates,
	std::vector<ScenePair>& scenePairs)
{
	std::map<std::pair<uint32_t, uint32_t>, unsigned> numPairCandidates;
	for (const SeamCandidate& c : candidates)
		++numPairCandidates[std::make_pair(c.sceneA, c.sceneB)];
	scenePairs.clear();
	for (const SeamCandidate& c : candidates) {
		// a pair its two candidates still dispute has no one similarity to offer
		if (numPairCandidates[std::make_pair(c.sceneA, c.sceneB)] != 1)
			continue;
		ScenePair sp;
		sp.sceneA = c.sceneA;
		sp.sceneB = c.sceneB;
		sp.relativeTransform = c.T;
		sp.numInliers = c.NumInliers();
		scenePairs.push_back(sp);
	}
	std::sort(scenePairs.begin(), scenePairs.end(), [](const ScenePair& a, const ScenePair& b) {
		return a.sceneA < b.sceneA || (a.sceneA == b.sceneA && a.sceneB < b.sceneB);
	});
}
/*----------------------------------------------------------------*/

void GlobalAlignment::PrepareSeamEvidence(
	const std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals)
{
	BuildGlobalToLocalMap(localToGlobals);

	// Per-block cache: PairIdx(localImageID, featureID) -> 3D inlier-track position. Only
	// observations belonging to inlier tracks are indexed; outliers are excluded because their
	// triangulated positions are unreliable.
	blockPointMaps.clear();
	blockPointMaps.resize(subScenes.size());
	FOREACH(blockIdx, subScenes) {
		const Scene& block = subScenes[blockIdx];
		size_t numInlierObservations = 0;
		for (const Track& track : block.tracks)
			if (track.IsInlier())
				numInlierObservations += track.GetNumInliers();
		auto& pointMap = blockPointMaps[blockIdx];
		pointMap.reserve(numInlierObservations);
		for (const Track& track : block.tracks)
			if (track.IsInlier())
				for (const Observation& obs : track)
					pointMap.emplace(PairIdx(obs.imageID, obs.featureID), track.position);
	}

	// Group the cross-block image pairs by block pair, the pair ordered a < b
	blockPairLinks.clear();
	FOREACH(idx, scene.pairs) {
		const ImagePair& pair = scene.pairs[idx];
		if (pair.GetNumWeightedInliers() < config.minCommonTracks)
			continue;
		const auto it1 = globalToLocal.find(pair.ID1);
		const auto it2 = globalToLocal.find(pair.ID2);
		if (it1 == globalToLocal.end() || it2 == globalToLocal.end())
			continue;
		const uint32_t block1 = it1->second.first, block2 = it2->second.first;
		if (block1 == block2)
			continue;
		blockPairLinks[std::make_pair(MINF(block1, block2), MAXF(block1, block2))].push_back((uint32_t)idx);
	}
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::EstimateSeamCandidates(
	const std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	std::vector<SeamCandidate>& candidates)
{
	TD_TIMER_STARTD();
	PrepareSeamEvidence(subScenes, localToGlobals);

	candidates.clear();
	unsigned numAgreed = 0, numDecided = 0, numUndecided = 0, numOneDirection = 0, numSkipped = 0;
	for (const auto& [blockPair, pairIndices] : blockPairLinks) {
		const size_t numBefore = candidates.size();
		EstimateSeamPair(subScenes, blockPair.first, blockPair.second, candidates);
		// what the pair ended with says how its verdict went
		switch (candidates.size() - numBefore) {
		case 0: ++numSkipped; break;
		case 2: ++numUndecided; break;
		default:
			if (candidates.back().oneDirection)
				++numOneDirection;
			else if (candidates.back().source == SeamCandidate::RIG_B_ON_A ||
					 candidates.back().source == SeamCandidate::RIG_A_ON_B)
				++numDecided;
			else
				++numAgreed;
		}
	}

	DEBUG("Measured %u seam candidates on %u block pairs (%u agreed, %u decided by votes, %u undecided, "
		"%u one direction, %u skipped) (%s)",
		(unsigned)candidates.size(), (unsigned)blockPairLinks.size(),
		numAgreed, numDecided, numUndecided, numOneDirection, numSkipped, TD_TIMER_GET_FMT().c_str());
	return !candidates.empty();
}
/*----------------------------------------------------------------*/

void GlobalAlignment::CollectPairObservations(
	const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
	std::vector<SeamObservation>& observations,
	std::vector<SeamCorrespondence>& correspondences) const
{
	ASSERT(a < b);
	observations.clear();
	correspondences.clear();
	const auto itLinks = blockPairLinks.find(std::make_pair(a, b));
	if (itLinks == blockPairLinks.end())
		return;
	std::vector<PairLink> links;
	ResolvePairLinks(scene, globalToLocal, itLinks->second, a, links);
	// block B's cameras against block A's points, then block A's cameras against block B's
	AppendRigObservations(CollectRigCorrespondences(
		subScenes[b], subScenes[a], blockPointMaps[a], links, true, config.maxReprojError),
		subScenes[b], true, observations, correspondences);
	AppendRigObservations(CollectRigCorrespondences(
		subScenes[a], subScenes[b], blockPointMaps[b], links, false, config.maxReprojError),
		subScenes[a], false, observations, correspondences);
}
/*----------------------------------------------------------------*/

void GlobalAlignment::ScoreSeam(
	const std::vector<Scene>& subScenes,
	const std::vector<SeamObservation>& observations,
	const std::vector<SeamCorrespondence>& correspondences,
	const Transform& T,
	const std::function<int(IIndex)>& sideOf,
	const std::vector<Point3>& movingCentres,
	const std::vector<Point3>& fixedCentres,
	SeamScore& score) const
{
	ASSERT(observations.size() == correspondences.size());
	score = SeamScore();

	// what the transform explains
	const Transform TInv(T.Invert());
	score.inlierMask.assign(observations.size(), false);
	FOREACH(i, observations)
		if (SeamObservationError(observations[i], T, TInv) <= config.maxReprojError) {
			score.inlierMask[i] = true;
			++score.inliers;
		}

	// every image the seam touches, in both its roles: the camera whose feature carries the point
	// witnesses the seam as much as the camera that observed it
	std::map<IIndex, std::vector<uint32_t>> imageObservations;
	FOREACH(i, observations) {
		imageObservations[observations[i].rigImage].push_back((uint32_t)i);
		imageObservations[observations[i].pointImage].push_back((uint32_t)i);
	}

	std::map<uint32_t, std::vector<Point3>> supportingCentres; // block -> its supporters' centres
	for (const auto& [image, indices] : imageObservations) {
		if (indices.size() < config.minVoteCorrespondences)
			continue;
		const auto itLocal = globalToLocal.find(image);
		ASSERT(itLocal != globalToLocal.end());
		const Image& img = subScenes[itLocal->second.first].images[itLocal->second.second];
		if (!img.IsValid())
			continue;
		CameraVote vote;
		vote.image = image;
		vote.correspondences = (unsigned)indices.size();
		// how far its inliers spread over its image: a camera whose inliers all sit in one cell
		// cannot support a transform, a single repeated texture patch buying exactly that
		std::vector<bool> cells(kVoteGridCells * kVoteGridCells, false);
		for (uint32_t i : indices) {
			if (!score.inlierMask[i])
				continue;
			++vote.inliers;
			const SeamCorrespondence& corr = correspondences[i];
			const uint32_t feature = corr.imageA == image ? corr.featureA : corr.featureB;
			if (feature >= img.keypoints.size())
				continue;
			const cv::Point2f& pt = img.keypoints[feature].pt;
			const int cellX = CLAMP((int)((float)kVoteGridCells * pt.x / (float)img.pCamera->GetWidth()), 0, (int)kVoteGridCells - 1);
			const int cellY = CLAMP((int)((float)kVoteGridCells * pt.y / (float)img.pCamera->GetHeight()), 0, (int)kVoteGridCells - 1);
			cells[cellY * (int)kVoteGridCells + cellX] = true;
		}
		vote.coverage = (float)std::count(cells.begin(), cells.end(), true) / (float)cells.size();
		const float inlierFraction = (float)vote.inliers / (float)vote.correspondences;
		if (vote.inliers >= config.minVoteInliers && inlierFraction >= kVoteSupportFraction &&
			vote.coverage >= config.minVoteCoverage)
			vote.vote = 1;
		else if (vote.correspondences >= config.minVoteInliers && inlierFraction < kVoteContraFraction)
			vote.vote = -1;
		const int side = sideOf(image);
		ASSERT(side == 0 || side == 1);
		if (vote.vote > 0) {
			++score.support[side];
			supportingCentres[itLocal->second.first].push_back(img.C);
		} else if (vote.vote < 0)
			++score.contra[side];
		score.votes.push_back(vote);
	}
	// each block's centres live in that block's own frame, so they are counted per block
	for (const auto& [block, centres] : supportingCentres)
		score.centres += CountDistinctRigCentres(centres);

	// a transform that drops one block's cameras among the other's has placed them wrong however
	// well it explains the correspondences: after a right one, a camera's nearest neighbour is one
	// of its own. A lone moving camera has no own neighbour to be nearest, and nothing to interleave.
	if (movingCentres.size() >= 2 && !fixedCentres.empty()) {
		std::vector<Point3> moved;
		moved.reserve(movingCentres.size());
		for (const Point3& C : movingCentres)
			moved.emplace_back(T * C);
		unsigned numOwnNeighbours = 0;
		FOREACH(i, moved) {
			REAL nearestOwn = std::numeric_limits<REAL>::max(), nearestOther = nearestOwn;
			FOREACH(j, moved)
				if (j != i)
					nearestOwn = MINF(nearestOwn, norm(moved[i] - moved[j]));
			for (const Point3& C : fixedCentres)
				nearestOther = MINF(nearestOther, norm(moved[i] - C));
			if (nearestOwn <= nearestOther)
				++numOwnNeighbours;
		}
		score.ownNeighbourFraction = (float)numOwnNeighbours / (float)moved.size();
	}
}
/*----------------------------------------------------------------*/

String GlobalAlignment::FailedGates(
	const SeamScore& score, unsigned inliersOther, unsigned bestOther, float voteRatio) const
{
	String failed;
	const auto fail = [&failed](const char* gate) {
		if (!failed.empty())
			failed += ", ";
		failed += gate;
	};
	// enough of the union explained, and enough of what the other direction measured on its own
	if (score.inliers < config.minCommonTracks ||
		(bestOther > 0 && (float)inliersOther < config.minCrossSupportRatio * (float)bestOther))
		fail("union support");
	// the cameras of both sides behind it, out of enough distinct centres
	if ((float)score.support[0] < voteRatio * (float)score.contra[0] ||
		(float)score.support[1] < voteRatio * (float)score.contra[1] ||
		score.centres < config.minSupportingCentres)
		fail("camera votes");
	// the two blocks' cameras left unmixed
	if (score.ownNeighbourFraction < config.minOwnNeighbourFraction)
		fail("interleaving");
	return failed;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::ScoreCandidate(const std::vector<Scene>& subScenes, SeamCandidate& c) const
{
	std::vector<Point3> centresA, centresB;
	for (const Image& img : subScenes[c.sceneA].images)
		if (img.IsValid())
			centresA.emplace_back(img.C);
	for (const Image& img : subScenes[c.sceneB].images)
		if (img.IsValid())
			centresB.emplace_back(img.C);
	const uint32_t blockA = c.sceneA;
	ScoreSeam(subScenes, c.observations, c.correspondences, c.T,
		[this, blockA](IIndex image) { return globalToLocal.at(image).first == blockA ? 0 : 1; },
		centresA, centresB, c.score);
	c.weight = c.score.Weight(config.maxVoteWeight);
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::IsScaleObservable(
	const std::vector<Scene>& subScenes, const SeamCandidate& c, bool forward) const
{
	// the rig cameras of this direction, and how deep in front of them its inlier points sit
	const Transform TInv(c.T.Invert());
	std::set<IIndex> rigImages, rigImagesWithInlier;
	std::vector<REAL> depths;
	FOREACH(i, c.observations) {
		const SeamObservation& obs = c.observations[i];
		if (obs.forward != forward)
			continue;
		rigImages.insert(obs.rigImage);
		if (i < c.score.inlierMask.size() && c.score.inlierMask[i]) {
			rigImagesWithInlier.insert(obs.rigImage);
			depths.push_back(norm((forward ? c.T : TInv) * obs.X - obs.C));
		}
	}
	// the rig cameras that carry the candidate; when fewer than two do, those merely holding an inlier
	std::set<IIndex> supporting;
	for (const CameraVote& vote : c.score.votes)
		if (vote.vote > 0 && rigImages.count(vote.image))
			supporting.insert(vote.image);
	const std::set<IIndex>& rig = supporting.size() >= 2 ? supporting : rigImagesWithInlier;
	if (rig.size() < 2 || depths.empty())
		return false;

	std::vector<Point3> centres;
	for (IIndex image : rig) {
		const auto itLocal = globalToLocal.find(image);
		ASSERT(itLocal != globalToLocal.end());
		centres.emplace_back(subScenes[itLocal->second.first].images[itLocal->second.second].C);
	}
	REAL spread = 0;
	FOREACH(i, centres)
		for (size_t j = i + 1; j < centres.size(); ++j)
			spread = MAXF(spread, norm(centres[i] - centres[j]));
	std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
	const REAL depth = depths[depths.size() / 2];
	return depth > 0 && spread / depth >= config.minRigSpreadRatio;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::EstimateSeamPair(
	const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
	std::vector<SeamCandidate>& candidates) const
{
	ASSERT(a < b && blockPointMaps.size() == subScenes.size());
	const auto itLinks = blockPairLinks.find(std::make_pair(a, b));
	if (itLinks == blockPairLinks.end())
		return;
	if (config.alignment == GlobalAlignmentConfig::ALIGN_POINTS) {
		EstimatePointSeamPair(subScenes, a, b, itLinks->second, candidates);
		return;
	}
	std::vector<PairLink> links;
	ResolvePairLinks(scene, globalToLocal, itLinks->second, a, links);

	// Both directions: B's cameras against A's tracks, and A's cameras against B's. Every cross
	// pair contributes to at least one of them, which the 3D-3D correspondences cannot promise on
	// a seam whose target-side keypoints never joined a track. Both candidates are then scored on
	// the union of the two, so the two opinions are held against the same evidence.
	const RigCorrespondences rc[2] = {
		CollectRigCorrespondences(subScenes[b], subScenes[a], blockPointMaps[a], links, true, config.maxReprojError),
		CollectRigCorrespondences(subScenes[a], subScenes[b], blockPointMaps[b], links, false, config.maxReprojError)};
	std::vector<SeamObservation> observations;
	std::vector<SeamCorrespondence> correspondences;
	std::vector<bool> estimatorMask;
	const poselib::RansacOptions ransacOptions = SeamRansacOptions();
	Transform Tseam[2], T_BA;
	unsigned inliers[2];
	inliers[0] = EstimateRigAgainstPoints(rc[0], subScenes[b], ransacOptions, true,
		Tseam[0], observations, correspondences, estimatorMask);
	inliers[1] = EstimateRigAgainstPoints(rc[1], subScenes[a], ransacOptions, false,
		T_BA, observations, correspondences, estimatorMask);
	// the second direction solved B into A, so its inverse is the seam's own A -> B similarity
	Tseam[1] = T_BA.Invert();

	// each direction fitted to the bearings behind its own inliers before the two are compared
	for (int d = 0; d < 2; ++d)
		if (inliers[d] >= config.minCommonTracks)
			RefineSeamTransform(SelectInliers(observations, estimatorMask, d == 0 ? 1 : 0),
				config.maxReprojError, Tseam[d]);
	const bool measured[2] = {
		inliers[0] >= config.minCommonTracks && ISFINITE(Tseam[0].scale) && Tseam[0].scale > 0,
		inliers[1] >= config.minCommonTracks && ISFINITE(Tseam[1].scale) && Tseam[1].scale > 0};
	DEBUG_ULTIMATE("Seam (%u, %u): %u cameras of B on %u points of A -> %u inliers%s; "
		"%u cameras of A on %u points of B -> %u inliers%s",
		a, b, (unsigned)rc[0].cameraExt.size(), rc[0].numCorrespondences, inliers[0], measured[0] ? "" : " (unmeasured)",
		(unsigned)rc[1].cameraExt.size(), rc[1].numCorrespondences, inliers[1], measured[1] ? "" : " (unmeasured)");
	if (!measured[0] && !measured[1]) {
		DEBUG_ULTIMATE("Seam (%u, %u) skipped: neither direction was measured", a, b);
		return;
	}

	// one candidate per measured direction, each scored on the union
	SeamCandidate candidate[2];
	for (int d = 0; d < 2; ++d) {
		if (!measured[d])
			continue;
		SeamCandidate& c = candidate[d];
		c.sceneA = a;
		c.sceneB = b;
		c.T = Tseam[d];
		c.source = d == 0 ? SeamCandidate::RIG_B_ON_A : SeamCandidate::RIG_A_ON_B;
		c.observations = observations;
		c.correspondences = correspondences;
		ScoreCandidate(subScenes, c);
		c.scaleObservable = IsScaleObservable(subScenes, c, d == 0);
	}

	// the gates, in one place: what the candidate explains of the union and of what the other
	// direction measured on its own, what the cameras of both sides vote, and whether it leaves
	// the two blocks' cameras unmixed
	bool passed[2] = {false, false};
	for (int d = 0; d < 2; ++d) {
		if (!measured[d])
			continue;
		const int other = 1 - d;
		const int otherForward = other == 0 ? 1 : 0;
		const unsigned bestOther =
			measured[other] && candidate[other].NumObservations(otherForward) >= config.minCommonTracks ?
			candidate[other].NumInliers(otherForward) : 0;
		const String failed = FailedGates(candidate[d].score,
			candidate[d].NumInliers(otherForward), bestOther, config.minCameraVoteRatio);
		LogVotes(String::FormatString("Seam (%u, %u) %s", a, b, SourceWord(candidate[d].source)).c_str(),
			candidate[d].score);
		if (failed.empty())
			passed[d] = true;
		else
			DEBUG_ULTIMATE("Seam (%u, %u) %s dropped by %s", a, b, SourceWord(candidate[d].source), failed.c_str());
	}
	if (!passed[0] && !passed[1]) {
		DEBUG_ULTIMATE("Seam (%u, %u) skipped: no direction passed the gates", a, b);
		return;
	}

	if (passed[0] && passed[1]) {
		const REAL errRotation = R2D(ACOS(ComputeAngle(Matrix3x3(candidate[0].T.R), Matrix3x3(candidate[1].T.R))));
		const REAL errScale = MAXF(candidate[0].T.scale / candidate[1].T.scale, candidate[1].T.scale / candidate[0].T.scale);
		const unsigned support[2] = {
			candidate[0].score.support[0] + candidate[0].score.support[1],
			candidate[1].score.support[0] + candidate[1].score.support[1]};
		const int best = support[0] >= support[1] ? 0 : 1;
		if (errRotation <= config.maxSimRotationError && errScale <= config.maxSimScaleRatio) {
			// two measurements of one seam that agree: one candidate, fitted to every bearing the
			// two directions between them explain
			SeamCandidate c(candidate[best]);
			c.source = SeamCandidate::UNION;
			c.scaleObservable = candidate[0].scaleObservable || candidate[1].scaleObservable;
			RefineSeamTransform(SelectInliers(c.observations, c.score.inlierMask), config.maxReprojError, c.T);
			ScoreCandidate(subScenes, c);
			LogSeamCandidate(a, b, c);
			candidates.emplace_back(std::move(c));
			return;
		}
		if ((float)support[best] < config.voteMargin * (float)support[1 - best]) {
			// the two disagree and the cameras are split between them: the seam graph decides
			DEBUG("Seam (%u, %u) undecided: the two directions disagree by %.2f deg and %.1f%% in scale, "
				"on %u and %u supporting cameras", a, b, errRotation, (errScale - 1) * 100, support[0], support[1]);
			for (int d = 0; d < 2; ++d) {
				LogSeamCandidate(a, b, candidate[d]);
				candidates.push_back(candidate[d]);
			}
			return;
		}
		// the cameras settle the disagreement: the direction they are behind takes the seam
		DEBUG("Seam (%u, %u) %s dropped by the camera votes of the other direction (%u against %u supporting cameras)",
			a, b, SourceWord(candidate[1 - best].source), support[best], support[1 - best]);
		SeamCandidate c(candidate[best]);
		RefineSeamTransform(SelectInliers(c.observations, c.score.inlierMask, best == 0 ? 1 : 0),
			config.maxReprojError, c.T);
		ScoreCandidate(subScenes, c);
		LogSeamCandidate(a, b, c);
		candidates.emplace_back(std::move(c));
		return;
	}

	// one survivor: the pair rests on a single direction, which the seam graph or the placement
	// has to confirm before it is believed
	const int d = passed[0] ? 0 : 1;
	SeamCandidate c(candidate[d]);
	c.oneDirection = true;
	RefineSeamTransform(SelectInliers(c.observations, c.score.inlierMask, d == 0 ? 1 : 0),
		config.maxReprojError, c.T);
	if (!c.scaleObservable && measured[1 - d]) {
		// its own rig is too shallow to observe a scale and the other direction measured one:
		// take it, and leave the rig where the seam already put it
		c.scaleObservable = true;
		RescaleSeamAboutRig(RigCentroid(c.observations, d == 0), d == 0, Tseam[1 - d].scale, c.T);
	}
	ScoreCandidate(subScenes, c);
	LogSeamCandidate(a, b, c);
	candidates.emplace_back(std::move(c));
}
/*----------------------------------------------------------------*/

void GlobalAlignment::EstimatePointSeamPair(
	const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
	const std::vector<uint32_t>& pairIndices, std::vector<SeamCandidate>& candidates) const
{
	// Collect 3D-3D correspondences: for each cross-block match whose endpoints both lie on an
	// existing inlier track, the two 3D positions, each in its own block's local frame.
	std::vector<PairLink> links;
	ResolvePairLinks(scene, globalToLocal, pairIndices, a, links);
	const Scene& blockA = subScenes[a];
	const Scene& blockB = subScenes[b];
	Point3Arr srcPoints, dstPoints;
	for (const PairLink& link : links) {
		ASSERT(link.localIdA < blockA.images.size() && link.localIdB < blockB.images.size());
		if (!blockA.images[link.localIdA].IsValid() || !blockB.images[link.localIdB].IsValid())
			continue;
		// the track-forming set, not the sparse count: the search below only keeps a match whose
		// two endpoints already lie on inlier tracks of the two blocks, and those tracks were built
		// from this same set -- bounding it by the sparse count would skip correspondences the
		// tracks prove exist; clamped by the array because the loop dereferences matches[i] before
		// anything inspects the DMatch
		const unsigned numMatches = MINF(link.pair->GetNumTrackFormingMatches(), (unsigned)link.pair->matches.size());
		for (unsigned i = 0; i < numMatches; ++i) {
			const DMatch& match = link.pair->matches[i];
			const uint32_t featureA = link.aIsQuery ? match.queryIdx : match.trainIdx;
			const uint32_t featureB = link.aIsQuery ? match.trainIdx : match.queryIdx;
			const auto itA = blockPointMaps[a].find(PairIdx(link.localIdA, featureA));
			if (itA == blockPointMaps[a].end())
				continue;
			const auto itB = blockPointMaps[b].find(PairIdx(link.localIdB, featureB));
			if (itB == blockPointMaps[b].end())
				continue;
			srcPoints.emplace_back(itA->second);
			dstPoints.emplace_back(itB->second);
		}
	}
	if (srcPoints.size() < config.minCommonTracks) {
		DEBUG_ULTIMATE("Seam (%u, %u) skipped: only %u 3D-3D correspondences, need >= %u",
			a, b, (unsigned)srcPoints.size(), config.minCommonTracks);
		return;
	}

	// Characteristic length scale for the RANSAC threshold: a fraction (default 1%) of the
	// destination point cloud's bounding-box diagonal. Using a relative scale keeps the criterion
	// invariant to each block's arbitrary units.
	AABB3 dstBbox(true);
	for (const Point3& p : dstPoints)
		dstBbox.InsertFull(p);
	const double threshold = config.simInlierThresholdFactor * dstBbox.GetSize().norm();
	if (!(threshold > 0))
		return;
	SeamCandidate c;
	c.sceneA = a;
	c.sceneB = b;
	c.source = SeamCandidate::POINTS;
	const unsigned numInliers = EstimateSimilarityTransform(srcPoints, dstPoints, c.T,
		threshold, true, config.simRansacMaxIters);
	const double inlierRatio = (double)numInliers / (double)srcPoints.size();
	if (numInliers < config.minCommonTracks || inlierRatio < config.minSimInlierRatio) {
		DEBUG_ULTIMATE("Seam (%u, %u) skipped: 3D-3D inliers %u/%u (%.1f%%)",
			a, b, numInliers, (unsigned)srcPoints.size(), inlierRatio * 100.0);
		return;
	}

	// the cameras of both blocks vote on it exactly as they vote on a camera-alignment candidate,
	// over the same correspondences
	CollectPairObservations(subScenes, a, b, c.observations, c.correspondences);
	ScoreCandidate(subScenes, c);
	const String failed = FailedGates(c.score, c.score.inliers, 0, config.minCameraVoteRatio);
	LogVotes(String::FormatString("Seam (%u, %u) %s", a, b, SourceWord(c.source)).c_str(), c.score);
	if (!failed.empty()) {
		DEBUG_ULTIMATE("Seam (%u, %u) %s dropped by %s", a, b, SourceWord(c.source), failed.c_str());
		return;
	}
	LogSeamCandidate(a, b, c);
	candidates.emplace_back(std::move(c));
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

// The diagonal of the box a block's cameras span in its own frame -- the unit the translation
// residuals of its seams are measured in
static REAL BlockExtent(const Scene& block)
{
	AABB3 bbox(true);
	for (const Image& img : block.images)
		if (img.IsValid())
			bbox.InsertFull(img.C);
	return bbox.IsEmpty() ? REAL(0) : (REAL)bbox.GetSize().norm();
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
		camBoxDiags[sceneIdx] = BlockExtent(subScenes[sceneIdx]);
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
	const uint32_t a, const uint32_t b, const Transform& T, const std::vector<Transform>& globalTransforms,
	const std::vector<REAL>& camBoxDiags, const GlobalAlignmentConfig& config)
{
	const Transform E = globalTransforms[a].Invert() * (globalTransforms[b] * T);
	SeamResidual res;
	res.scale = MAXF(E.scale, REAL(1) / E.scale);
	res.rotation = R2D(ACOS(ComputeAngle(Matrix3x3(E.R))));
	// E.t is in A's local frame: express the discrepancy in global units and compare
	// it against the smaller of the two global camera footprints, so the verdict does
	// not depend on which endpoint happens to have the lower sub-scene index
	const REAL diagA = camBoxDiags[a] * globalTransforms[a].scale;
	const REAL diagB = camBoxDiags[b] * globalTransforms[b].scale;
	const REAL diag = diagA > 0 && diagB > 0 ? MINF(diagA, diagB) : MAXF(diagA, diagB);
	res.translation = diag > 0 ? globalTransforms[a].scale * norm(E.t) / diag : REAL(0);
	res.excess = MAXF(MAXF(res.scale / config.maxSimScaleRatio, res.rotation / config.maxSimRotationError),
		res.translation / config.maxSimTranslationError);
	return res;
}

static SeamResidual ComputeSeamResidual(
	const ScenePair& sp, const std::vector<Transform>& globalTransforms,
	const std::vector<REAL>& camBoxDiags, const GlobalAlignmentConfig& config)
{
	return ComputeSeamResidual(sp.sceneA, sp.sceneB, sp.relativeTransform, globalTransforms, camBoxDiags, config);
}
/*----------------------------------------------------------------*/

// One connected component of a seam graph: the seams that span it, the blocks they join, the block
// carrying most of their weight -- the gauge its averaging is anchored at -- and the weight it holds
struct SeamComponent
{
	std::vector<uint32_t> edges;
	uint32_t numBlocks{0};
	uint32_t gauge{NO_ID};
	REAL weight{0};
};

// The connected components the given seams span, the heaviest first
static void GroupSeamComponents(
	const std::vector<SeamCandidate>& candidates, const std::vector<uint32_t>& edges,
	const uint32_t numBlocks, std::vector<SeamComponent>& components)
{
	DisjointSet<uint32_t> blockSets(numBlocks);
	std::vector<REAL> blockWeights(numBlocks, REAL(0));
	std::vector<bool> touched(numBlocks, false);
	for (const uint32_t e : edges) {
		const SeamCandidate& c = candidates[e];
		blockSets.Union(c.sceneA, c.sceneB);
		for (const uint32_t block : {c.sceneA, c.sceneB}) {
			blockWeights[block] += c.weight;
			touched[block] = true;
		}
	}
	components.clear();
	std::unordered_map<uint32_t, uint32_t> componentOfRoot;
	for (uint32_t block = 0; block < numBlocks; ++block) {
		if (!touched[block])
			continue;
		const auto [it, inserted] = componentOfRoot.emplace(blockSets.Find(block), (uint32_t)components.size());
		if (inserted)
			components.emplace_back();
		SeamComponent& component = components[it->second];
		++component.numBlocks;
		if (component.gauge == NO_ID || blockWeights[block] > blockWeights[component.gauge])
			component.gauge = block;
	}
	for (const uint32_t e : edges) {
		const SeamCandidate& c = candidates[e];
		SeamComponent& component = components[componentOfRoot[blockSets.Find(c.sceneA)]];
		component.edges.push_back(e);
		component.weight += c.weight;
	}
	std::sort(components.begin(), components.end(),
		[](const SeamComponent& a, const SeamComponent& b) { return a.weight > b.weight; });
}

// Whether the accepted seams among the given ones join the two ends of `c` on their own
static bool SeamEndsJoined(
	const std::vector<SeamCandidate>& candidates, const std::vector<uint32_t>& edges,
	const uint32_t numBlocks, const SeamCandidate& c,
	const std::function<bool(const SeamCandidate&)>& accept)
{
	DisjointSet<uint32_t> blockSets(numBlocks);
	for (const uint32_t e : edges)
		if (accept(candidates[e]))
			blockSets.Union(candidates[e].sceneA, candidates[e].sceneB);
	return blockSets.Find(c.sceneA) == blockSets.Find(c.sceneB);
}

// The direction a candidate measured on its own: the rig of B over A's points is the forward one
static int SeamOwnDirection(SeamCandidate::Source source)
{
	return source == SeamCandidate::RIG_A_ON_B ? 0 : 1;
}

// What a seam weighs in an averaging: its own weight, halved when only its own evidence backs it
static float SeamEdgeWeight(const SeamCandidate& c)
{
	return c.cls == SeamCandidate::VERIFIED ? c.weight * 0.5f : c.weight;
}

// A seam the consensus of its component sits on; one that cannot observe a scale took part in the
// rotation consensus only and is judged on its rotation alone
static bool IsSeamConsistent(const SeamCandidate& c, const GlobalAlignmentConfig& config)
{
	return c.residualRotation <= config.maxGraphRotationResidual &&
		(!c.scaleObservable || (c.residualScale <= config.maxGraphScaleResidual &&
			c.residualTranslation <= config.maxSimTranslationError));
}

// A seam nothing corroborates is still trusted when the one direction it was measured in saw far
// more than a seam needs, spread over enough of its rig
static bool DoesSeamStandAlone(const SeamCandidate& c, const GlobalAlignmentConfig& config)
{
	return c.oneDirection && c.score.centres >= config.minSupportingCentres &&
		c.NumInliers(SeamOwnDirection(c.source)) >= kStrongAloneFactor * config.minCommonTracks;
}

// What the consensus of a component makes of one of its seams; `reason` is filled when undecided
static SeamCandidate::Class ClassifySeam(
	const std::vector<SeamCandidate>& candidates, const SeamComponent& component,
	const uint32_t numBlocks, const SeamCandidate& c, const GlobalAlignmentConfig& config,
	const char*& reason)
{
	if (c.residualRotation >= FLT_MAX) {
		reason = "split frame";
		return SeamCandidate::UNDECIDED;
	}
	if (IsSeamConsistent(c, config)) {
		// only another path between the same two blocks corroborates a seam; a second opinion on
		// the same pair is not one
		const bool corroborated = SeamEndsJoined(candidates, component.edges, numBlocks, c,
			[&c, &config](const SeamCandidate& d) {
				return IsSeamConsistent(d, config) && (d.sceneA != c.sceneA || d.sceneB != c.sceneB);
			});
		if (corroborated)
			return SeamCandidate::ROBUST;
		if (c.source == SeamCandidate::UNION || c.source == SeamCandidate::POINTS || DoesSeamStandAlone(c, config))
			return SeamCandidate::VERIFIED;
		reason = "bridge, one direction";
		return SeamCandidate::UNDECIDED;
	}
	// a path at least as strong holds the two blocks where this seam does not
	const bool contradicted = SeamEndsJoined(candidates, component.edges, numBlocks, c,
		[&c, &config](const SeamCandidate& d) {
			return IsSeamConsistent(d, config) && d.weight >= c.weight / config.rejectWeightMargin;
		});
	if (contradicted)
		return SeamCandidate::REJECTED;
	reason = "inconsistent, no strong alternative";
	return SeamCandidate::UNDECIDED;
}

// The node a solve is gauged at: the one asked for when its own pairs reach it, and the best
// connected of them otherwise -- a gauge no pair reaches leaves the system without a datum, which
// the least-squares solvers answer with an arbitrary one instead of a failure
template <typename Pair>
static uint32_t SolveGauge(const std::vector<Pair>& pairs, const uint32_t preferred)
{
	std::unordered_map<uint32_t, float> nodeWeights;
	for (const Pair& p : pairs) {
		nodeWeights[p.idxA] += p.weight;
		nodeWeights[p.idxB] += p.weight;
	}
	if (nodeWeights.find(preferred) != nodeWeights.end())
		return preferred;
	uint32_t best = NO_ID;
	for (const Pair& p : pairs)
		for (const uint32_t node : {p.idxA, p.idxB})
			if (best == NO_ID || nodeWeights[node] > nodeWeights[best])
				best = node;
	return best;
}

// The global rotation of every node the seams reach, and which run of the estimator placed it.
// The estimator solves the largest sub-component it can and leaves the rest at INF, so it is run
// again over the seams that stayed inside the nodes it left out, until no run places anything more.
// Two nodes of different frames are each solved in their own gauge and cannot be compared.
static bool SolveRotationFrames(
	const std::vector<RotationPair>& pairs, const uint32_t numNodes,
	const GlobalRotationEstimatorOptions& options,
	std::vector<Point3>& rotations, std::vector<uint32_t>& frameOfNode)
{
	std::vector<Point3> solved(numNodes, Point3::INF);
	std::vector<uint32_t> frames(numNodes, NO_ID);
	std::vector<RotationPair> remaining(pairs);
	uint32_t numFrames = 0;
	while (!remaining.empty()) {
		GlobalRotationEstimator estimator(options);
		std::vector<Point3> partial;
		if (!estimator.EstimateRotations(remaining, numNodes, partial))
			break;
		bool placedAny = false;
		for (uint32_t node = 0; node < numNodes; ++node) {
			if (frames[node] != NO_ID || partial[node] == Point3::INF)
				continue;
			solved[node] = partial[node];
			frames[node] = numFrames;
			placedAny = true;
		}
		if (!placedAny)
			break;
		++numFrames;
		std::vector<RotationPair> unplaced;
		for (const RotationPair& p : remaining)
			if (frames[p.idxA] == NO_ID && frames[p.idxB] == NO_ID)
				unplaced.push_back(p);
		remaining.swap(unplaced);
	}
	if (numFrames == 0)
		return false;
	rotations.swap(solved);
	frameOfNode.swap(frames);
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::AverageBlockPoses(
	const std::vector<SeamCandidate>& candidates,
	const std::vector<uint32_t>& edges,
	const std::vector<REAL>& blockExtents,
	const uint32_t numBlocks, const uint32_t fixedBlock,
	std::vector<BlockPose>& poses,
	std::vector<Point3>& residuals) const
{
	poses.assign(numBlocks, BlockPose());
	// a seam whose blocks the averaging does not place is left without a verdict
	residuals.assign(edges.size(), Point3(REAL(FLT_MAX), REAL(1), REAL(0)));
	if (edges.empty())
		return false;

	// the blocks the seams touch, renumbered 0..n-1 for the solvers
	std::vector<uint32_t> nodeOfBlock(numBlocks, NO_ID), blockOfNode;
	for (const uint32_t e : edges) {
		const SeamCandidate& c = candidates[e];
		for (const uint32_t block : {c.sceneA, c.sceneB})
			if (nodeOfBlock[block] == NO_ID) {
				nodeOfBlock[block] = (uint32_t)blockOfNode.size();
				blockOfNode.push_back(block);
			}
	}
	const uint32_t n = (uint32_t)blockOfNode.size();
	const uint32_t gauge = nodeOfBlock[fixedBlock];
	ASSERT(gauge != NO_ID, "the gauge block carries none of the averaged seams");
	if (gauge == NO_ID)
		return false;

	// rotations: each seam claims R_B * R_A^T
	std::vector<RotationPair> rotationPairs;
	rotationPairs.reserve(edges.size());
	for (const uint32_t e : edges) {
		const SeamCandidate& c = candidates[e];
		rotationPairs.emplace_back(nodeOfBlock[c.sceneA], nodeOfBlock[c.sceneB], Matrix3x3(c.T.R), SeamEdgeWeight(c));
	}
	const GlobalRotationEstimatorOptions rotationOptions;
	std::vector<uint32_t> frameOfNode;
	std::vector<Point3> rotations;
	std::vector<REAL> rotationResiduals;
	if (!RobustAverage(rotationPairs, (REAL)config.maxGraphRotationResidual, kRobustRounds,
		[&](const std::vector<RotationPair>& active, std::vector<Point3>& rots) {
			return SolveRotationFrames(active, n, rotationOptions, rots, frameOfNode);
		},
		[&](size_t i, const std::vector<Point3>& rots) {
			const RotationPair& p = rotationPairs[i];
			if (frameOfNode[p.idxA] == NO_ID || frameOfNode[p.idxA] != frameOfNode[p.idxB])
				return REAL(FLT_MAX);
			const RMatrix RA(rots[p.idxA]), RB(rots[p.idxB]);
			return R2D(ACOS(ComputeAngle(p.relativeRotation, Matrix3x3(RB * RA.t()))));
		},
		rotations, rotationResiduals))
	{
		VERBOSE("error: rotation averaging over %u seams failed", (unsigned)edges.size());
		return false;
	}
	// scales: only the seams the rotation consensus keeps, and only those that observe a scale
	const REAL maxLogScaleResidual = std::log((REAL)config.maxGraphScaleResidual);
	std::vector<uint32_t> scaleEdges; // positions in `edges`
	std::vector<ScalePair> scalePairs;
	FOREACH(k, edges) {
		const SeamCandidate& c = candidates[edges[k]];
		if (!c.scaleObservable || c.T.scale <= 0 || rotationResiduals[k] > config.maxGraphRotationResidual)
			continue;
		scaleEdges.push_back(k);
		scalePairs.emplace_back(nodeOfBlock[c.sceneA], nodeOfBlock[c.sceneB], REAL(1) / c.T.scale, SeamEdgeWeight(c));
	}
	// the scale and translation solves see fewer seams than the rotation one, so they can leave a
	// block the rotations joined without any metric tie to the gauge; such a block gets no pose,
	// and one whose seams observe no scale at all keeps the unit scale it cannot improve on
	std::vector<REAL> scales(n, REAL(1));
	std::vector<Point3> translations(n, Point3::ZERO);
	DisjointSet<uint32_t> metricSets(n);
	uint32_t metricGauge = NO_ID;
	if (!scalePairs.empty()) {
		const uint32_t scaleGauge = SolveGauge(scalePairs, gauge);
		GlobalScaleEstimator scaleEstimator;
		std::vector<REAL> scaleResiduals;
		if (!RobustAverage(scalePairs, maxLogScaleResidual, kRobustRounds,
			[&](const std::vector<ScalePair>& active, std::vector<REAL>& s) { return scaleEstimator.EstimateScales(active, n, scaleGauge, s); },
			[&](size_t i, const std::vector<REAL>& s) {
				const ScalePair& p = scalePairs[i];
				return std::log(p.scaleRatio) - (std::log(s[p.idxB]) - std::log(s[p.idxA]));
			},
			scales, scaleResiduals))
		{
			VERBOSE("error: scale averaging over %u seams failed", (unsigned)scalePairs.size());
			return false;
		}

		// translations: the seams under both bars, each placing one block's origin in the other's frame
		std::vector<TranslationPair> translationPairs;
		FOREACH(i, scalePairs) {
			if (scaleResiduals[i] > maxLogScaleResidual)
				continue;
			const SeamCandidate& c = candidates[edges[scaleEdges[i]]];
			const uint32_t a = nodeOfBlock[c.sceneA], b = nodeOfBlock[c.sceneB];
			// the seam maps A's frame to B's, so B's origin sits at -(1/scale) R^T t in A's frame;
			// A's own rotation and scale carry that offset into the frame the averaging solves in
			const Point3 originBinA = (c.T.R.t() * c.T.t) * (-REAL(1) / c.T.scale);
			translationPairs.emplace_back(a, b, scales[a] * (RMatrix(rotations[a]).t() * originBinA), SeamEdgeWeight(c));
			metricSets.Union(a, b);
		}
		if (!translationPairs.empty()) {
			metricGauge = SolveGauge(translationPairs, scaleGauge);
			GlobalTranslationEstimator translationEstimator;
			std::vector<REAL> translationResiduals;
			if (!RobustAverage(translationPairs, (REAL)config.maxSimTranslationError, kRobustRounds,
				[&](const std::vector<TranslationPair>& active, std::vector<Point3>& t) { return translationEstimator.EstimateTranslations(active, n, metricGauge, t); },
				[&](size_t i, const std::vector<Point3>& t) {
					const TranslationPair& p = translationPairs[i];
					const REAL extent = MINF(blockExtents[blockOfNode[p.idxA]] * scales[p.idxA], blockExtents[blockOfNode[p.idxB]] * scales[p.idxB]);
					return extent > 0 ? norm(t[p.idxB] - t[p.idxA] - p.relativeTranslation) / extent : REAL(0);
				},
				translations, translationResiduals))
			{
				VERBOSE("error: translation averaging over %u seams failed", (unsigned)translationPairs.size());
				return false;
			}
		}
	}

	// every block of the frame the poses are gauged in carries a transform, but only those the
	// metric consensus reaches from the gauge are placed
	const uint32_t poseFrame = frameOfNode[metricGauge != NO_ID ? metricGauge : gauge];
	if (poseFrame == NO_ID)
		return false;
	std::vector<Transform> transforms(numBlocks);
	std::vector<bool> hasTransform(numBlocks, false);
	unsigned numPlaced = 0;
	for (uint32_t node = 0; node < n; ++node) {
		if (frameOfNode[node] != poseFrame)
			continue;
		const uint32_t block = blockOfNode[node];
		transforms[block] = BuildGlobalTransform(rotations[node], scales[node], translations[node]);
		hasTransform[block] = true;
		if (metricGauge == NO_ID || metricSets.Find(node) != metricSets.Find(metricGauge))
			continue;
		poses[block].T = transforms[block];
		poses[block].model = 0;
		++numPlaced;
	}
	// what the consensus says of each seam: a rotation-only seam took part in the rotation
	// consensus alone, and one that measured a scale needs both its blocks placed to be judged
	FOREACH(k, edges) {
		const SeamCandidate& c = candidates[edges[k]];
		if (!hasTransform[c.sceneA] || !hasTransform[c.sceneB])
			continue;
		const SeamResidual residual = ComputeSeamResidual(c.sceneA, c.sceneB, c.T, transforms, blockExtents, config);
		if (!c.scaleObservable)
			residuals[k] = Point3(residual.rotation, REAL(1), REAL(0));
		else if (poses[c.sceneA].model != NO_ID && poses[c.sceneB].model != NO_ID)
			residuals[k] = Point3(residual.rotation, residual.scale, residual.translation);
	}
	return numPlaced > 0;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::ClassifySeamGraph(
	const std::vector<REAL>& blockExtents,
	std::vector<SeamCandidate>& candidates) const
{
	TD_TIMER_STARTD();
	const uint32_t numBlocks = (uint32_t)blockExtents.size();
	std::vector<uint32_t> allEdges(candidates.size());
	std::iota(allEdges.begin(), allEdges.end(), 0u);
	std::vector<SeamComponent> components;
	GroupSeamComponents(candidates, allEdges, numBlocks, components);

	// every candidate is judged afresh
	for (SeamCandidate& c : candidates) {
		c.cls = SeamCandidate::UNCLASSIFIED;
		c.residualRotation = 0.f;
		c.residualScale = 1.f;
		c.residualTranslation = 0.f;
	}
	std::vector<BlockPose> poses;
	std::vector<Point3> residuals;
	for (const SeamComponent& component : components) {
		// two blocks hold no cycle: the averaging would reproduce every seam between them exactly,
		// so their residuals stay at zero and the seams are left to the rules below
		if (component.numBlocks < 3)
			continue;
		// the averaging judges every seam it can, whether or not it could place the blocks, and
		// leaves the rest the infinite rotation residual that keeps them undecided
		AverageBlockPoses(candidates, component.edges, blockExtents, numBlocks, component.gauge, poses, residuals);
		FOREACH(i, component.edges) {
			SeamCandidate& c = candidates[component.edges[i]];
			c.residualRotation = (float)residuals[i].x;
			c.residualScale = (float)residuals[i].y;
			c.residualTranslation = (float)residuals[i].z;
		}
	}

	unsigned numRobust = 0, numVerified = 0, numUndecided = 0, numRejected = 0;
	for (const SeamComponent& component : components) {
		// two blocks joined to nothing else and holding more than one opinion carry no evidence
		// that tells those opinions apart: the placement pools them instead
		const bool disputedPair = component.numBlocks < 3 && component.edges.size() > 1;
		for (const uint32_t e : component.edges) {
			SeamCandidate& c = candidates[e];
			const char* reason = NULL;
			if (disputedPair) {
				c.cls = SeamCandidate::UNDECIDED;
				reason = "isolated pair, no agreement";
			} else
				c.cls = ClassifySeam(candidates, component, numBlocks, c, config, reason);
			switch (c.cls) {
			case SeamCandidate::ROBUST:
				++numRobust;
				break;
			case SeamCandidate::VERIFIED:
				++numVerified;
				break;
			case SeamCandidate::REJECTED:
				++numRejected;
				VERBOSE("Seam (%u, %u) rejected by consensus: rotation %.2f deg, scale %.1f%%, translation %.2f%% (weight %.0f)",
					c.sceneA, c.sceneB, c.residualRotation, (c.residualScale - 1.f) * 100.f,
					c.residualTranslation * 100.f, c.weight);
				break;
			default:
				++numUndecided;
				DEBUG("Seam (%u, %u) undecided: %s", c.sceneA, c.sceneB, reason);
				break;
			}
		}
	}
	VERBOSE("Seam graph: %u blocks, %u components, %u candidates: %u robust, %u verified, %u undecided, %u rejected (%s)",
		numBlocks, (unsigned)components.size(), (unsigned)candidates.size(),
		numRobust, numVerified, numUndecided, numRejected, TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::ComputeInitialBlockPoses(
	const std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t numBlocks,
	std::vector<BlockPose>& poses) const
{
	TD_TIMER_STARTD();
	std::vector<uint32_t> trustedEdges;
	FOREACH(i, candidates)
		if (candidates[i].IsTrusted())
			trustedEdges.push_back(i);
	std::vector<SeamComponent> components;
	GroupSeamComponents(candidates, trustedEdges, numBlocks, components);

	poses.assign(numBlocks, BlockPose());
	std::vector<BlockPose> componentPoses;
	std::vector<Point3> residuals;
	unsigned numPlaced = 0, numModels = 0;
	FOREACH(k, components) {
		const SeamComponent& component = components[k];
		if (!AverageBlockPoses(candidates, component.edges, blockExtents, numBlocks, component.gauge, componentPoses, residuals)) {
			VERBOSE("warning: the %u blocks joined by %u trusted seams cannot be placed together",
				component.numBlocks, (unsigned)component.edges.size());
			continue;
		}
		++numModels;
		for (uint32_t block = 0; block < numBlocks; ++block) {
			if (componentPoses[block].model == NO_ID)
				continue;
			poses[block].T = componentPoses[block].T;
			poses[block].model = k;
			++numPlaced;
		}
	}
	VERBOSE("Initial poses: %u blocks placed in %u models (%s)",
		numPlaced, numModels, TD_TIMER_GET_FMT().c_str());
	return numPlaced > 0;
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
