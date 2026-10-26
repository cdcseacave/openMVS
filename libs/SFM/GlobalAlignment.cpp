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
#include "PoseLink.h"
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

// The diagonal of the box a set of camera centres spans
static REAL CentresExtent(const std::vector<Point3>& centres)
{
	AABB3 bbox(true);
	for (const Point3& C : centres)
		bbox.InsertFull(C);
	return bbox.IsEmpty() ? REAL(0) : (REAL)bbox.GetSize().norm();
}

// The camera footprint of a block in its own frame: the unit the translation residuals of its seams
// are measured in, and the point they are measured at
static BlockFootprint FootprintOf(const Scene& block)
{
	std::vector<Point3> centres;
	BlockFootprint footprint;
	for (const Image& img : block.images)
		if (img.IsValid()) {
			centres.emplace_back(img.C);
			footprint.centre += img.C;
		}
	footprint.extent = CentresExtent(centres);
	if (!centres.empty())
		footprint.centre /= (REAL)centres.size();
	return footprint;
}

namespace {

// The bars of the merge that are not configurable, each with the rule it belongs to.
// (kRobustRounds, the rounds of the cut every averaging of the block graph is judged by, is
// RobustAveraging.h's.)
// The translation unit of a pair is the smaller block's extent, never under this share of the
// larger's: a block of two cameras a millimetre apart has no footprint of its own to be judged by.
constexpr float kMinExtentShare = 0.2f;
// A one-camera door takes its scale from the depths its points have in both blocks only with this many
// of them; with fewer it observes no scale.
constexpr unsigned kDoorMinDepthRatios = 10;
// The L1 reweighting of the scales and the translations of the block graph (the view-graph filter's
// form of it): its rounds, and the error a reading is never weighed as below -- in log-scale, and
// in the footprint the two blocks share
constexpr unsigned kL1Rounds = 40;
constexpr REAL kL1ScaleFloor = REAL(1e-4);
constexpr REAL kL1TranslationFloor = REAL(1e-3);

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

// The seven parameters one block's similarity is solved in: a unit quaternion on its own manifold,
// a translation, and the scale in the log domain the solver moves linearly in.
struct BlockParameters
{
	double q[4], t[3], logScale;

	explicit BlockParameters(const Transform& T) {
		Eigen::Matrix3d R;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				R(i, j) = T.R(i, j);
		const Eigen::Quaterniond quat(R);
		q[0] = quat.w(); q[1] = quat.x(); q[2] = quat.y(); q[3] = quat.z();
		for (int i = 0; i < 3; ++i)
			t[i] = T.t[i];
		logScale = LOGN(T.scale);
	}

	Transform ToTransform() const {
		const Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
		Transform T;
		T.R = Eigen::Matrix3d(quat.normalized().toRotationMatrix());
		T.t = Point3(t[0], t[1], t[2]);
		T.scale = EXP(logScale);
		return T;
	}
};

// Reprojection of one seam observation through the two block similarities it ties together: the
// point is carried from its own block into the model frame, then back into the frame of the block
// the observing camera belongs to, and predicted in that camera. The residual is the chord between
// the predicted unit bearing and the observed one, converted to pixels by the observing camera's
// own angular-to-pixel rate, so the Huber loss set at the pixel threshold means the same for every
// camera in the model. The chord equals the angular error to first order, stays finite for any
// central camera model, and grows to 2 radians for a point that ends up behind the camera -- where
// the offset in the observed bearing's tangent plane, which this replaced, collapses back to zero
// and reports the worst possible fit as a perfect one.
struct BlockSeamReprojectionError
{
	double X[3], R[9], C[3], b[3], pixelPerRadian;

	explicit BlockSeamReprojectionError(const SeamObservation& obs) {
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
	bool operator()(
		const T* const qPoint, const T* const tPoint, const T* const logScalePoint,
		const T* const qCamera, const T* const tCamera, const T* const logScaleCamera,
		T* residuals) const
	{
		using std::exp;
		using std::sqrt;
		// the point's own block into the model frame: p_model = s * R * X + t
		const T Xb[3] = {T(X[0]), T(X[1]), T(X[2])};
		T p[3];
		ceres::QuaternionRotatePoint(qPoint, Xb, p);
		const T sPoint = exp(logScalePoint[0]);
		for (int i = 0; i < 3; ++i)
			p[i] = sPoint * p[i] + tPoint[i];
		// the model frame into the observing camera's block: p_block = R^t * (p_model - t) / s
		const T dModel[3] = {p[0] - tCamera[0], p[1] - tCamera[1], p[2] - tCamera[2]};
		const T qInv[4] = {qCamera[0], -qCamera[1], -qCamera[2], -qCamera[3]};
		T pBlock[3];
		ceres::QuaternionRotatePoint(qInv, dModel, pBlock);
		const T invScale = exp(-logScaleCamera[0]);
		// into the observing camera, then onto the unit sphere
		const T d[3] = {invScale * pBlock[0] - T(C[0]), invScale * pBlock[1] - T(C[1]), invScale * pBlock[2] - T(C[2])};
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

// The rate at which a camera reads an angle in its own pixels at the reprojection bar, so every
// camera model in a rig is judged at the same pixel error
double PixelPerRadian(const Image& image, float maxReprojError)
{
	const REAL angular = image.pCamera->PixelErrorToAngular(maxReprojError * image.pCamera->GetFeatureNoiseScale());
	return angular > 0 ? (double)(maxReprojError / angular) : 0.0;
}

// Gather the observations, in the rig sub-scene's images, of the point sub-scene's inlier tracks:
// one bearing per cross-sub-scene match whose point-side endpoint lies on such a track. Only that
// one side has to be on a track, which is why a seam remains measurable from either direction
// alone even when its other side's keypoints never formed one.
RigCorrespondences CollectRigCorrespondences(
	const Scene& rigScene, const Scene& pointScene,
	const std::unordered_map<PairIdx, uint32_t>& pointObsToTrack,
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
			const auto it = pointObsToTrack.find(PairIdx(localIdPoint, featurePoint));
			if (it == pointObsToTrack.end())
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
				const double pixelPerRadian = PixelPerRadian(imgRig, maxReprojError);
				rc.pixelPerRadian.push_back(pixelPerRadian);
				rc.maxError = MAXF(rc.maxError, pixelPerRadian > 0 ? (double)maxReprojError / pixelPerRadian : 0.0);
			}
			const uint32_t r = itRig->second;
			rc.bearings[r].emplace_back(imgRig.pCamera->UnprojectNormalized(imgRig.keypoints[featureRig].pt));
			rc.points[r].push_back(pointScene.tracks[it->second].position);
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

// The rig of the given observations of one direction, read back off them: per observing camera,
// its pose, bearings and points; `indexOf` holds, per rig camera, the observation behind each
// correspondence
void GroupRigObservations(
	const std::vector<SeamObservation>& observations, const std::vector<uint32_t>& indices,
	float maxReprojError, RigCorrespondences& rc, std::vector<std::vector<uint32_t>>& indexOf)
{
	rc = RigCorrespondences();
	indexOf.clear();
	std::unordered_map<IIndex, uint32_t> rigIndexOf;
	for (const uint32_t i : indices) {
		const SeamObservation& obs = observations[i];
		const auto [it, inserted] = rigIndexOf.emplace(obs.rigImage, (uint32_t)rc.cameraExt.size());
		if (inserted) {
			rc.cameraExt.emplace_back(obs.R, Point3(obs.R * (-obs.C)));
			rc.bearings.emplace_back();
			rc.points.emplace_back();
			indexOf.emplace_back();
			rc.maxError = MAXF(rc.maxError, obs.pixelPerRadian > 0 ? (double)maxReprojError / obs.pixelPerRadian : 0.0);
		}
		rc.bearings[it->second].emplace_back(normalized(obs.bearing));
		rc.points[it->second].push_back(obs.X);
		indexOf[it->second].push_back(i);
		++rc.numCorrespondences;
	}
}

// One rig against one set of points: the generalized absolute pose with scale of the rig, from
// bearings alone. The estimator solves scale * p_rig = R * p_point + t, i.e. it scales the rig's
// centers into the frame the points live in, so the similarity mapping the point frame into the
// rig frame is p_rig = (1/scale) * R * p_point + (1/scale) * t, which is what comes out in `T`.
// `inliers` says, per rig camera, which of its correspondences the estimate kept. Returns 0 when
// the rig yields no usable estimate.
unsigned EstimateRigAgainstPoints(
	const RigCorrespondences& rc, const poselib::RansacOptions& ransac, Transform& T,
	std::vector<std::vector<char>>& inliers)
{
	inliers.clear();
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
	const poselib::RansacStats stats = poselib::estimate_generalized_absolute_pose_scale_bearings(
		rc.bearings, rc.points, rc.cameraExt, opt, &pose, &scale, &inliers);
	if (stats.num_inliers == 0 || !ISFINITE(scale) || scale <= 0)
		return 0;

	T.R = pose.R();
	T.scale = REAL(1) / scale;
	T.t = Point3(pose.t[0], pose.t[1], pose.t[2]) * T.scale;
	return (unsigned)stats.num_inliers;
}


// Error of one seam observation under a candidate A -> B similarity, in the pixel units the
// threshold is set in: the angle between the predicted and the observed bearing, charged at the
// observing camera's own angular-to-pixel rate. This is the residual the refinement minimizes, so
// the same number decides an inlier before, during and after it.
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


// One cross-block match, as the two directions that collect it both name it: the image and the
// feature of side A, and those of side B
struct CorrespondenceKey
{
	PairIdx a, b;
	bool operator==(const CorrespondenceKey& other) const { return a == other.a && b == other.b; }
};
struct CorrespondenceKeyHash
{
	size_t operator()(const CorrespondenceKey& key) const {
		return std::hash<PairIdx>()(key.a) * 0x9E3779B97F4A7C15ULL ^ std::hash<PairIdx>()(key.b);
	}
};
CorrespondenceKey KeyOfCorrespondence(const SeamCorrespondence& corr)
{
	return CorrespondenceKey{PairIdx(corr.imageA, corr.featureA), PairIdx(corr.imageB, corr.featureB)};
}


// How a reading was measured, for the log
const char* SourceWord(SeamCandidate::Source source)
{
	switch (source) {
	case SeamCandidate::RIG_B_ON_A: return "rig B on A";
	case SeamCandidate::RIG_A_ON_B: return "rig A on B";
	default: return "points";
	}
}

// Where one block triangulated what its cameras saw: (local image, feature) -> the inlier track
// holding that observation, what the correspondence collection looks each match up in. Only inlier
// tracks are indexed, an outlier's position being unreliable.
void BuildBlockPointMap(const Scene& block, std::unordered_map<PairIdx, uint32_t>& pointMap)
{
	size_t numInlierObservations = 0;
	for (const Track& track : block.tracks)
		if (track.IsInlier())
			numInlierObservations += track.GetNumInliers();
	pointMap.clear();
	pointMap.reserve(numInlierObservations);
	FOREACH(t, block.tracks)
		if (block.tracks[t].IsInlier())
			for (const Observation& obs : block.tracks[t])
				pointMap.emplace(PairIdx(obs.imageID, obs.featureID), (uint32_t)t);
}

// The similarity carrying one camera's own frame into the frame of the block it belongs to: the
// camera reads that frame through its rotation, and sits where its centre says
Transform CameraToBlock(const Image& image)
{
	Transform T;
	T.R = RMatrix(image.R.t());
	T.t = image.C;
	return T;
}

// The observations at the given indices
std::vector<SeamObservation> SelectObservations(
	const std::vector<SeamObservation>& observations, const std::vector<uint32_t>& indices)
{
	std::vector<SeamObservation> selected;
	selected.reserve(indices.size());
	for (const uint32_t i : indices)
		selected.push_back(observations[i]);
	return selected;
}

// Refine one A -> B similarity against every reprojection the given observations carry: block A
// gauges the fit at the identity, so the seam travels in and out through block B's pose, and every
// observation is charged as the chord between its observed and predicted bearings in the observing
// camera's own pixels under a Huber loss of the reprojection bar
void RefineSimilarity(const std::vector<SeamObservation>& observations, float maxReprojError, Transform& T)
{
	if (observations.empty())
		return;
	BlockParameters gauge((Transform())), blockB(T.Invert());
	ceres::Problem problem;
	ceres::LossFunction* loss = new ceres::HuberLoss(maxReprojError);
	for (const SeamObservation& obs : observations) {
		BlockParameters& point = obs.forward ? gauge : blockB;
		BlockParameters& camera = obs.forward ? blockB : gauge;
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<BlockSeamReprojectionError, 3, 4, 3, 1, 4, 3, 1>(
				new BlockSeamReprojectionError(obs)),
			loss, point.q, point.t, &point.logScale, camera.q, camera.t, &camera.logScale);
	}
	problem.SetManifold(blockB.q, new ceres::QuaternionManifold);
	problem.SetParameterBlockConstant(gauge.q);
	problem.SetParameterBlockConstant(gauge.t);
	problem.SetParameterBlockConstant(&gauge.logScale);
	ceres::Solver::Options options;
	// seven parameters against many thousands of residuals
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 50;
	options.function_tolerance = 1e-8;
	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	if (summary.IsSolutionUsable() && ISFINITE(blockB.logScale))
		T = blockB.ToTransform().Invert();
}

// Whether the rig behind the given observations of one direction observes a scale: the spread of
// the rig cameras holding minCameraInliers of them -- every camera holding one when fewer than two
// do -- over the median depth of their points. A rig collapsed onto one point sees no parallax.
bool RigObservesScale(
	const std::vector<SeamObservation>& observations, const std::vector<uint32_t>& indices,
	const Transform& T, const GlobalAlignmentConfig& config)
{
	const Transform TInv(T.Invert());
	std::map<IIndex, std::pair<unsigned, Point3>> rig;
	std::vector<REAL> depths;
	for (const uint32_t i : indices) {
		const SeamObservation& obs = observations[i];
		std::pair<unsigned, Point3>& camera = rig[obs.rigImage];
		++camera.first;
		camera.second = obs.C;
		depths.push_back(norm((obs.forward ? T : TInv) * obs.X - obs.C));
	}
	std::vector<Point3> centres, holding;
	for (const auto& [image, camera] : rig) {
		holding.push_back(camera.second);
		if (camera.first >= config.minCameraInliers)
			centres.push_back(camera.second);
	}
	if (centres.size() < 2)
		centres.swap(holding);
	if (centres.size() < 2 || depths.empty())
		return false;
	REAL spread = 0;
	FOREACH(i, centres)
		for (size_t j = i + 1; j < centres.size(); ++j)
			spread = MAXF(spread, norm(centres[i] - centres[j]));
	std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
	const REAL depth = depths[depths.size() / 2];
	return depth > 0 && spread / depth >= config.minRigSpreadRatio;
}

// One reading of a block pair: the similarity (A -> B) an estimate found, how it was measured, the
// observations of its direction it explains at the bar and whether it observes a scale
struct SeamReading
{
	Transform T;
	SeamCandidate::Source source{SeamCandidate::POINTS};
	std::vector<uint32_t> inliers;
	bool scaleObservable{true};
};

// What a one-camera door reads beyond the pair's observations: the rig block's own structure, the
// depths of which give the door its scale
struct DoorContext
{
	const Scene& rigScene;
	const std::unordered_map<PairIdx, uint32_t>& rigPointMap;
	const std::unordered_map<IIndex, std::pair<uint32_t, IIndex>>& globalToLocal;
	const std::vector<SeamCorrespondence>& correspondences;
};

// A direction whose rig is one camera, which the generalized estimate cannot measure: that camera's
// pose against the other block's points (P3P RANSAC on its bearings), its own pose in its block giving
// the rotation and the translation, and the scale the median ratio of the depths its points have in
// the two blocks, over the correspondences whose keypoint is also an observation of a track of its
// own block
bool EstimateDoorReading(
	const std::vector<SeamObservation>& observations, const std::vector<uint32_t>& direction,
	bool forward, const DoorContext& door, const GlobalAlignmentConfig& config, SeamReading& reading)
{
	ASSERT(!direction.empty());
	const SeamObservation& first = observations[direction.front()];
	std::vector<poselib::Point3D> bearings, points;
	for (const uint32_t i : direction) {
		ASSERT(observations[i].rigImage == first.rigImage);
		const Point3 b(normalized(observations[i].bearing));
		bearings.emplace_back(b.x, b.y, b.z);
		points.emplace_back(observations[i].X.x, observations[i].X.y, observations[i].X.z);
	}
	poselib::AbsolutePoseOptions opt;
	opt.ransac = SeamRansacOptions();
	opt.max_error = first.pixelPerRadian > 0 ? (double)config.maxReprojError / first.pixelPerRadian : 0.0;
	if (!(opt.max_error > 0))
		return false;
	poselib::CameraPose pose;
	std::vector<char> inliers;
	const poselib::RansacStats stats = poselib::estimate_absolute_pose_bearings(bearings, points, opt, &pose, &inliers);
	if (stats.num_inliers < config.minCommonTracks)
		return false;
	// the camera in the point block's frame (x_cam = R X + t), and the depths its inlier points have
	// there and in the rig block's own triangulation of them
	const RMatrix Rp(Eigen::Matrix3d(pose.R()));
	const Point3 Cp(Rp.t() * Point3(-pose.t[0], -pose.t[1], -pose.t[2]));
	std::vector<REAL> ratios;
	FOREACH(k, direction) {
		if (!inliers[k])
			continue;
		const SeamObservation& obs = observations[direction[k]];
		const SeamCorrespondence& corr = door.correspondences[direction[k]];
		const auto itLocal = door.globalToLocal.find(forward ? corr.imageB : corr.imageA);
		if (itLocal == door.globalToLocal.end())
			continue;
		const auto itTrack = door.rigPointMap.find(PairIdx(itLocal->second.second, forward ? corr.featureB : corr.featureA));
		if (itTrack == door.rigPointMap.end())
			continue;
		const REAL depthRig = norm(door.rigScene.tracks[itTrack->second].position - obs.C);
		const REAL depthPoint = norm(obs.X - Cp);
		if (depthRig > 0 && depthPoint > 0)
			ratios.push_back(depthRig / depthPoint);
	}
	reading.scaleObservable = ratios.size() >= kDoorMinDepthRatios;
	REAL scale = 1;
	if (reading.scaleObservable) {
		std::nth_element(ratios.begin(), ratios.begin() + ratios.size() / 2, ratios.end());
		scale = ratios[ratios.size() / 2];
	}
	// the point block's frame into the rig block's: the camera's two rotations composed, its centre
	// carried across
	Transform pointToRig;
	pointToRig.R = RMatrix(first.R.t() * Rp);
	pointToRig.scale = scale;
	pointToRig.t = first.C - (pointToRig.R * Cp) * scale;
	reading.T = forward ? pointToRig : pointToRig.Invert();
	reading.source = forward ? SeamCandidate::RIG_B_ON_A : SeamCandidate::RIG_A_ON_B;
	DEBUG_EXTRA("Seam door of one camera: %u of %u correspondences, scale %g from %u depth ratios",
		(unsigned)stats.num_inliers, (unsigned)direction.size(), scale, (unsigned)ratios.size());
	return true;
}

// The reading of one direction of a block pair (forward: the rig of B on the points of A): the
// generalized estimate refined over its inliers at the bar, or a door's reading when the rig is one
// camera; none when the direction holds fewer than minCommonTracks correspondences, or the reading
// fewer inliers
bool EstimateDirectionReading(
	const std::vector<SeamObservation>& observations, bool forward,
	const DoorContext& door, const GlobalAlignmentConfig& config, SeamReading& reading)
{
	std::vector<uint32_t> direction;
	FOREACH(i, observations)
		if (observations[i].forward == forward)
			direction.push_back((uint32_t)i);
	if (direction.size() < config.minCommonTracks)
		return false;
	RigCorrespondences rc;
	std::vector<std::vector<uint32_t>> indexOf;
	GroupRigObservations(observations, direction, config.maxReprojError, rc, indexOf);
	// a direction of one rig camera is a door, measured by that camera's own pose
	const bool door1 = rc.cameraExt.size() == 1 && EstimateDoorReading(observations, direction, forward, door, config, reading);
	if (!door1) {
		Transform T;
		std::vector<std::vector<char>> ransacInliers;
		if (EstimateRigAgainstPoints(rc, SeamRansacOptions(), T, ransacInliers) == 0)
			return false;
		// the rig of B on A's points solves A into B, the rig of A on B's points B into A
		reading.T = forward ? T : T.Invert();
		reading.source = forward ? SeamCandidate::RIG_B_ON_A : SeamCandidate::RIG_A_ON_B;
		std::vector<uint32_t> fitted;
		FOREACH(r, ransacInliers)
			FOREACH(k, ransacInliers[r])
				if (ransacInliers[r][k])
					fitted.push_back(indexOf[r][k]);
		RefineSimilarity(SelectObservations(observations, fitted), config.maxReprojError, reading.T);
	}
	if (!ISFINITE(reading.T.scale) || reading.T.scale <= 0)
		return false;
	// what it explains at the bar, of its direction
	const Transform TInv(reading.T.Invert());
	for (const uint32_t i : direction)
		if (SeamObservationError(observations[i], reading.T, TInv) <= config.maxReprojError)
			reading.inliers.push_back(i);
	if (reading.inliers.size() < config.minCommonTracks)
		return false;
	if (!door1)
		reading.scaleObservable = RigObservesScale(observations, reading.inliers, reading.T, config);
	return true;
}

// The reading of a block pair in the point alignment: the 3D-3D similarity of the matches lying on an
// inlier track in both blocks
bool EstimatePointReading(
	const Scene& scene, const std::unordered_map<IIndex, std::pair<uint32_t, IIndex>>& globalToLocal,
	const std::vector<std::unordered_map<PairIdx, uint32_t>>& blockPointMaps,
	const std::vector<Scene>& subScenes, uint32_t a, uint32_t b,
	const std::vector<uint32_t>& pairIndices, const GlobalAlignmentConfig& config,
	SeamReading& reading)
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
			srcPoints.emplace_back(blockA.tracks[itA->second].position);
			dstPoints.emplace_back(blockB.tracks[itB->second].position);
		}
	}
	if (srcPoints.size() < config.minCommonTracks)
		return false;
	// Characteristic length scale for the RANSAC threshold: a fraction (default 1%) of the
	// destination point cloud's bounding-box diagonal. Using a relative scale keeps the criterion
	// invariant to each block's arbitrary units.
	AABB3 dstBbox(true);
	for (const Point3& p : dstPoints)
		dstBbox.InsertFull(p);
	const double threshold = config.simInlierThresholdFactor * dstBbox.GetSize().norm();
	if (!(threshold > 0))
		return false;
	reading.source = SeamCandidate::POINTS;
	if (EstimateSimilarityTransform(srcPoints, dstPoints, reading.T, threshold, true, config.simRansacMaxIters) == 0)
		return false;
	unsigned numInliers = 0;
	FOREACH(i, srcPoints)
		if (norm(reading.T * srcPoints[i] - dstPoints[i]) <= threshold)
			++numInliers;
	if (numInliers < config.minCommonTracks || (double)numInliers < config.minSimInlierRatio * (double)srcPoints.size())
		return false;
	DEBUG_ULTIMATE("Seam (%u, %u) points: %u of %u 3D-3D correspondences", a, b, numInliers, (unsigned)srcPoints.size());
	return true;
}

// the per-block local→model Sim(3) the averaging produces: it solves R_i mapping model→local
// (the same convention as Image.R) while the similarity transform applies local→model, hence
// the transpose
Transform BuildGlobalTransform(const Point3d& rotation, REAL scale, const Point3& translation)
{
	Transform G;
	G.R = RMatrix(rotation).t();
	G.scale = scale;
	G.t = translation;
	return G;
}

// How far apart two similarities that carry the same frame into the same one are
struct SeamResidual {
	REAL scale;       // ratio, >= 1
	REAL rotation;    // degrees
	REAL translation; // fraction of the two frames' shared camera footprint (see PairExtent)
};

// The one unit a translation between two frames is read in, wherever two camera footprints meet:
// the smaller of the two, since that is the block that can least afford the error, floored at a
// share of the larger so that a frame with no extent of its own is still judged by something.
REAL PairExtent(const REAL first, const REAL second)
{
	return MAXF(MINF(first, second), (REAL)kMinExtentShare * MAXF(first, second));
}

// Sim(3) cycle residual of one reading: T maps A-local to B-local and G_i maps each local frame to
// the model frame, so G_B*T and G_A both map A-local to the model and their discrepancy
// E = G_A^-1 * G_B * T is what the reading is off by (identity when perfectly consistent). It is
// read in A's local frame, its translation where it moves A's camera centroid, against the
// footprint the two blocks share there, so the verdict does not depend on which end happens to
// have the lower block index.
SeamResidual ComputeSeamResidual(
	const uint32_t a, const uint32_t b, const Transform& T, const std::vector<Transform>& transforms,
	const std::vector<BlockFootprint>& footprints)
{
	const REAL extent = PairExtent(footprints[a].extent * transforms[a].scale, footprints[b].extent * transforms[b].scale) / transforms[a].scale;
	const Point3& centre = footprints[a].centre;
	const Transform E = transforms[a].Invert() * (transforms[b] * T);
	SeamResidual res;
	res.scale = MAXF(E.scale, REAL(1) / E.scale);
	res.rotation = R2D(ACOS(ComputeAngle(Matrix3x3(E.R))));
	res.translation = extent > 0 ? norm(E * centre - centre) / extent : REAL(0);
	return res;
}

// The node a solve is gauged at: the one asked for when its own pairs reach it, and the best
// connected of them otherwise -- a gauge no pair reaches leaves the system without a datum, which
// the least-squares solvers answer with an arbitrary one instead of a failure
template <typename Pair>
uint32_t SolveGauge(const std::vector<Pair>& pairs, const uint32_t preferred)
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

// What one reading is off by against the solved rotations, in degrees: a reading whose two ends
// came out in different frames was solved in two gauges and cannot be compared at all, which is as
// wrong as a reading gets
REAL RotationResidual(
	const RotationPair& p, const std::vector<Point3>& rotations, const std::vector<uint32_t>& frameOfNode)
{
	if (frameOfNode[p.idxA] == NO_ID || frameOfNode[p.idxA] != frameOfNode[p.idxB])
		return REAL(FLT_MAX);
	const RMatrix RA(rotations[p.idxA]), RB(rotations[p.idxB]);
	return R2D(ACOS(ComputeAngle(p.relativeRotation, Matrix3x3(RB * RA.t()))));
}

// The global rotation of every node the readings reach, and which run of the estimator placed it.
// The estimator solves the largest sub-component it can and leaves the rest at INF, so it is run
// again over the readings that stayed inside the nodes it left out, until no run places anything
// more. Two nodes of different frames are each solved in their own gauge and cannot be compared.
bool SolveRotationFrames(
	const std::vector<RotationPair>& pairs, const uint32_t numNodes,
	std::vector<Point3>& rotations, std::vector<uint32_t>& frameOfNode)
{
	const GlobalRotationEstimatorOptions options;
	rotations.assign(numNodes, Point3::INF);
	frameOfNode.assign(numNodes, NO_ID);
	std::vector<RotationPair> remaining(pairs);
	uint32_t numFrames = 0;
	while (!remaining.empty()) {
		GlobalRotationEstimator estimator(options);
		std::vector<Point3> partial;
		if (!estimator.EstimateRotations(remaining, numNodes, partial))
			break;
		bool placedAny = false;
		for (uint32_t node = 0; node < numNodes; ++node) {
			if (frameOfNode[node] != NO_ID || partial[node] == Point3::INF)
				continue;
			rotations[node] = partial[node];
			frameOfNode[node] = numFrames;
			placedAny = true;
		}
		if (!placedAny)
			break;
		++numFrames;
		std::vector<RotationPair> unplaced;
		for (const RotationPair& p : remaining)
			if (frameOfNode[p.idxA] == NO_ID && frameOfNode[p.idxB] == NO_ID)
				unplaced.push_back(p);
		remaining.swap(unplaced);
	}
	return numFrames > 0;
}

// The weight of an L1 reweighting: the inverse of the reading's error, never under the floor
auto L1Factor(const REAL minError)
{
	return [minError](REAL r) { return (float)(REAL(1) / MAXF(r, minError)); };
}

} // namespace
/*----------------------------------------------------------------*/

bool GlobalAlignment::AverageBlockPoses(
	const std::vector<SeamCandidate>& candidates,
	const std::vector<uint32_t>& edges,
	const std::vector<BlockFootprint>& footprints,
	const uint32_t fixedBlock,
	std::vector<BlockPose>& poses,
	std::vector<Point3>& residuals) const
{
	const uint32_t numBlocks = (uint32_t)footprints.size();
	poses.assign(numBlocks, BlockPose());
	// a reading whose blocks the averaging does not place is left without a verdict
	residuals.assign(edges.size(), Point3(REAL(FLT_MAX), REAL(1), REAL(0)));
	if (edges.empty())
		return false;

	// the blocks the readings touch, renumbered 0..n-1 for the solvers
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
	ASSERT(gauge != NO_ID, "the gauge block carries none of the averaged readings");
	// one connected component: the rotation estimator's tree reaches only the component it is
	// rooted in, so readings outside it would take no part in its initialization
	ASSERT([&]() {
		DisjointSet<uint32_t> component(n);
		for (const uint32_t e : edges)
			component.Union(nodeOfBlock[candidates[e].sceneA], nodeOfBlock[candidates[e].sceneB]);
		for (uint32_t node = 1; node < n; ++node)
			if (component.Find(node) != component.Find(0))
				return false;
		return true;
	}(), "the averaged readings span more than one connected component");

	// rotations: each reading claims R_B * R_A^T
	std::vector<RotationPair> rotationPairs;
	rotationPairs.reserve(edges.size());
	for (const uint32_t e : edges) {
		const SeamCandidate& c = candidates[e];
		rotationPairs.emplace_back(nodeOfBlock[c.sceneA], nodeOfBlock[c.sceneB], Matrix3x3(c.T.R), c.weight);
	}
	std::vector<uint32_t> frameOfNode;
	std::vector<Point3> rotations;
	if (!SolveRotationFrames(rotationPairs, n, rotations, frameOfNode)) {
		VERBOSE("error: rotation averaging over %u readings failed", (unsigned)edges.size());
		return false;
	}
	std::vector<REAL> rotationResiduals(rotationPairs.size());
	FOREACH(i, rotationPairs)
		rotationResiduals[i] = RotationResidual(rotationPairs[i], rotations, frameOfNode);
	// a component that comes back in more than one frame is one the rotations could not carry
	// across: its readings are then judged apart and the placement has nothing joining the two halves
	uint32_t numFrames = 0;
	for (const uint32_t frame : frameOfNode)
		if (frame != NO_ID)
			numFrames = MAXF(numFrames, frame + 1);
	DEBUG("Seam graph: component of %u blocks solved in %u frames", n, numFrames);
	// scales: only the readings within the rotation bar, and only those that observe a scale
	const REAL maxLogScaleResidual = std::log((REAL)config.maxGraphScaleResidual);
	std::vector<uint32_t> scaleEdges; // positions in `edges`
	std::vector<ScalePair> scalePairs;
	FOREACH(k, edges) {
		const SeamCandidate& c = candidates[edges[k]];
		if (!c.scaleObservable || c.T.scale <= 0 || rotationResiduals[k] > config.maxGraphRotationResidual)
			continue;
		scaleEdges.push_back(k);
		scalePairs.emplace_back(nodeOfBlock[c.sceneA], nodeOfBlock[c.sceneB], REAL(1) / c.T.scale, c.weight);
	}
	// the scale and translation solves see fewer readings than the rotation one, so they can leave a
	// block the rotations joined without any metric tie to the gauge; such a block gets no pose,
	// and one whose readings observe no scale at all keeps the unit scale it cannot improve on
	std::vector<REAL> scales(n, REAL(1));
	std::vector<Point3> translations(n, Point3::ZERO);
	DisjointSet<uint32_t> metricSets(n);
	uint32_t metricGauge = NO_ID;
	if (!scalePairs.empty()) {
		const uint32_t scaleGauge = SolveGauge(scalePairs, gauge);
		GlobalScaleEstimator scaleEstimator;
		std::vector<REAL> scaleResiduals;
		if (!RobustAverageWith(scalePairs, kL1Rounds,
			[&](const std::vector<ScalePair>& active, std::vector<REAL>& s) { return scaleEstimator.EstimateScales(active, n, scaleGauge, s); },
			[&](size_t i, const std::vector<REAL>& s) {
				const ScalePair& p = scalePairs[i];
				return std::log(p.scaleRatio) - (std::log(s[p.idxB]) - std::log(s[p.idxA]));
			},
			L1Factor(kL1ScaleFloor), scales, scaleResiduals))
		{
			VERBOSE("error: scale averaging over %u readings failed", (unsigned)scalePairs.size());
			return false;
		}

		// translations: the readings under both bars, each placing one block's cameras' centroid in
		// the other's frame, so what is averaged does not depend on where a block's frame has its origin
		std::vector<TranslationPair> translationPairs;
		FOREACH(i, scalePairs) {
			if (scaleResiduals[i] > maxLogScaleResidual)
				continue;
			const SeamCandidate& c = candidates[edges[scaleEdges[i]]];
			const uint32_t a = nodeOfBlock[c.sceneA], b = nodeOfBlock[c.sceneB];
			// the reading maps A's frame to B's, so B's centroid sits at T^-1 of it in A's frame; A's
			// own rotation and scale carry the offset between the two centroids into the frame the
			// averaging solves in
			const Point3 centreBinA = c.T.Invert() * footprints[c.sceneB].centre;
			translationPairs.emplace_back(a, b,
				scales[a] * (RMatrix(rotations[a]).t() * (centreBinA - footprints[c.sceneA].centre)), c.weight);
			metricSets.Union(a, b);
		}
		if (!translationPairs.empty()) {
			metricGauge = SolveGauge(translationPairs, scaleGauge);
			GlobalTranslationEstimator translationEstimator;
			std::vector<REAL> translationResiduals;
			if (!RobustAverageWith(translationPairs, kL1Rounds,
				[&](const std::vector<TranslationPair>& active, std::vector<Point3>& t) { return translationEstimator.EstimateTranslations(active, n, metricGauge, t); },
				[&](size_t i, const std::vector<Point3>& t) {
					const TranslationPair& p = translationPairs[i];
					// the footprint the two blocks share, as everything that judges a reading's
					// translation reads it
					const REAL extent = PairExtent(footprints[blockOfNode[p.idxA]].extent * scales[p.idxA],
						footprints[blockOfNode[p.idxB]].extent * scales[p.idxB]);
					return extent > 0 ? norm(t[p.idxB] - t[p.idxA] - p.relativeTranslation) / extent : REAL(0);
				},
				L1Factor(kL1TranslationFloor), translations, translationResiduals))
			{
				VERBOSE("error: translation averaging over %u readings failed", (unsigned)translationPairs.size());
				return false;
			}
		}
	}

	// every block of the frame the poses are gauged in carries a transform, but only those the
	// metric averaging reaches from the gauge are placed
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
		// the averaging solved where the block's centroid lands; its frame's origin follows
		const RMatrix R(RMatrix(rotations[node]).t());
		transforms[block] = BuildGlobalTransform(rotations[node], scales[node],
			translations[node] - (R * footprints[block].centre) * scales[node]);
		hasTransform[block] = true;
		if (metricGauge == NO_ID || metricSets.Find(node) != metricSets.Find(metricGauge))
			continue;
		poses[block].T = transforms[block];
		poses[block].model = 0;
		++numPlaced;
	}
	// what the averaging says of each reading: a rotation-only reading took part in the rotation
	// averaging alone, and one that measured a scale needs both its blocks placed to be judged
	FOREACH(k, edges) {
		const SeamCandidate& c = candidates[edges[k]];
		if (!hasTransform[c.sceneA] || !hasTransform[c.sceneB])
			continue;
		const SeamResidual residual = ComputeSeamResidual(c.sceneA, c.sceneB, c.T, transforms, footprints);
		if (!c.scaleObservable)
			residuals[k] = Point3(residual.rotation, REAL(1), REAL(0));
		else if (poses[c.sceneA].model != NO_ID && poses[c.sceneB].model != NO_ID)
			residuals[k] = Point3(residual.rotation, residual.scale, residual.translation);
	}
	return numPlaced > 0;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::PrepareSeamEvidence(
	const std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals)
{
	BuildGlobalToLocalMap(localToGlobals);

	// where every block triangulated what its cameras saw
	blockPointMaps.clear();
	blockPointMaps.resize(subScenes.size());
	FOREACH(blockIdx, subScenes)
		BuildBlockPointMap(subScenes[blockIdx], blockPointMaps[blockIdx]);

	// and which cross pairs join which two blocks
	MapBlockPairLinks();
}
/*----------------------------------------------------------------*/

void GlobalAlignment::MapBlockPairLinks()
{
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
	unsigned numMeasured = 0;
	for (const auto& [blockPair, pairIndices] : blockPairLinks) {
		const size_t numBefore = candidates.size();
		EstimateSeamPair(subScenes, blockPair.first, blockPair.second, candidates);
		numMeasured += candidates.size() > numBefore ? 1 : 0;
	}
	DEBUG("Measured %u readings on %u of %u block pairs (%s)",
		(unsigned)candidates.size(), numMeasured, (unsigned)blockPairLinks.size(), TD_TIMER_GET_FMT().c_str());
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
	const std::vector<SeamObservation>& observations,
	const std::vector<SeamCorrespondence>& correspondences,
	const Transform& T,
	const std::function<int(IIndex)>& sideOf,
	SeamScore& score) const
{
	ASSERT(observations.size() == correspondences.size());
	score = SeamScore();
	const Transform TInv(T.Invert());
	// every image the seam touches, in both its roles: the camera whose feature carries the point
	// witnesses the seam as much as the camera that observed it. A match collected in both directions
	// is one correspondence, whichever of its two observations the transform explains, so each of its
	// two cameras counts it once
	std::unordered_set<CorrespondenceKey, CorrespondenceKeyHash> counted;
	std::unordered_map<IIndex, unsigned> inliersOf;
	FOREACH(i, observations) {
		if (SeamObservationError(observations[i], T, TInv) > config.maxReprojError)
			continue;
		++score.inliers;
		if (!counted.insert(KeyOfCorrespondence(correspondences[i])).second)
			continue;
		++inliersOf[observations[i].rigImage];
		++inliersOf[observations[i].pointImage];
	}
	for (const auto& [image, numInliers] : inliersOf)
		if (numInliers >= config.minCameraInliers) {
			const int side = sideOf(image);
			ASSERT(side == 0 || side == 1);
			++score.support[side];
		}
}
/*----------------------------------------------------------------*/

PairAgreement GlobalAlignment::MeasureSeamPairs(
	const std::vector<Scene>& subScenes, const uint32_t a, const uint32_t b, const Transform& T) const
{
	PairAgreement agreement;
	if (config.maxPairRotationResidual <= 0.f)
		return agreement;
	const auto itLinks = blockPairLinks.find(std::make_pair(a, b));
	if (itLinks == blockPairLinks.end())
		return agreement;
	// both blocks' cameras in block A's frame, block B's where the reading puts it
	const Transform BToA(T.Invert());
	const auto PoseInA = [&](const std::pair<uint32_t, IIndex>& local) {
		const Image& image = subScenes[local.first].images[local.second];
		const Transform cameraToA(local.first == a ? CameraToBlock(image) : BToA * CameraToBlock(image));
		Pose3D pose;
		pose.R = RMatrix(cameraToA.R.t());
		pose.C = cameraToA.t;
		return pose;
	};
	for (const uint32_t idx : itLinks->second) {
		const ImagePair& pair = scene.pairs[idx];
		if (!IsPoseLinkPair(pair) || pair.GetNumWeightedInliers() < config.minPairInliers)
			continue;
		const std::pair<uint32_t, IIndex>& local1 = globalToLocal.at(pair.ID1);
		const std::pair<uint32_t, IIndex>& local2 = globalToLocal.at(pair.ID2);
		if (!subScenes[local1.first].images[local1.second].IsValid() ||
			!subScenes[local2.first].images[local2.second].IsValid())
			continue;
		// block B's image is the one judged, block A's its neighbour
		const bool firstInB = local1.first == b;
		const PairDisagreement d = MeasurePairDisagreement(pair,
			PoseInA(firstInB ? local1 : local2), PoseInA(firstInB ? local2 : local1),
			firstInB ? pair.ID2 : pair.ID1);
		++agreement.numPairs;
		(d.Within(config.maxPairRotationResidual) ? agreement.agree : agreement.disagree) += (float)pair.GetNumWeightedInliers();
	}
	return agreement;
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
	// every correspondence of the pair, both directions, and the readings the configured estimator
	// finds in them
	std::vector<SeamObservation> observations;
	std::vector<SeamCorrespondence> correspondences;
	CollectPairObservations(subScenes, a, b, observations, correspondences);
	std::vector<SeamReading> readings;
	const bool pointAlignment = config.alignment == GlobalAlignmentConfig::ALIGN_POINTS;
	if (pointAlignment) {
		SeamReading reading;
		if (EstimatePointReading(scene, globalToLocal, blockPointMaps, subScenes, a, b, itLinks->second, config, reading))
			readings.emplace_back(std::move(reading));
	} else {
		// B's cameras on A's points, then A's cameras on B's, the rig block of each direction giving
		// a door its scale
		const DoorContext doors[2] = {
			{subScenes[b], blockPointMaps[b], globalToLocal, correspondences},
			{subScenes[a], blockPointMaps[a], globalToLocal, correspondences}};
		for (int d = 0; d < 2; ++d) {
			SeamReading reading;
			if (EstimateDirectionReading(observations, d == 0, doors[d], config, reading))
				readings.emplace_back(std::move(reading));
		}
	}

	// every reading scored on every correspondence of the pair and read against the verified pairs
	// across it
	const auto sideOf = [this, a](IIndex image) { return globalToLocal.at(image).first == a ? 0 : 1; };
	for (const SeamReading& reading : readings) {
		SeamCandidate c;
		ScoreSeam(observations, correspondences, reading.T, sideOf, c.score);
		c.pairs = MeasureSeamPairs(subScenes, a, b, reading.T);
		// a reading of the camera alignment always enters the block graph, weighing the supporting
		// cameras of its weaker side (at least one: a door's single camera) times the share of the
		// verified pairs' inlier weight it reproduces; the 3D-3D reading enters only with supporting
		// cameras on both sides and its verified pairs not against it, weighing its cameras
		const bool enters = !pointAlignment || (c.score.Weight() > 0 && c.pairs.Holds());
		DEBUG("Seam (%u, %u) %s: %u/%u inliers, cameras %u+%u, %u pairs weighing %.0f for / %.0f against%s",
			a, b, SourceWord(reading.source), c.score.inliers, (unsigned)observations.size(), c.score.support[0], c.score.support[1],
			c.pairs.numPairs, c.pairs.agree, c.pairs.disagree, enters ? "" : ", refused");
		if (!enters)
			continue;
		c.sceneA = a;
		c.sceneB = b;
		c.T = reading.T;
		c.source = reading.source;
		c.scaleObservable = reading.scaleObservable;
		c.weight = pointAlignment ? (float)c.score.Weight() : (float)MAXF(c.score.Weight(), 1u) * c.pairs.Share();
		candidates.emplace_back(std::move(c));
	}
}
/*----------------------------------------------------------------*/

void GlobalAlignment::SelectSeamGraph(
	const std::vector<BlockFootprint>& footprints,
	std::vector<SeamCandidate>& candidates,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& edgeSeams) const
{
	TD_TIMER_STARTD();
	const uint32_t numBlocks = (uint32_t)footprints.size();
	// the view-graph filter's pattern over the block graph: every reading is a measurement of its
	// block pair; every component of the readings kept is averaged; every reading is judged against
	// the poses of its component (a reading without scale on its rotation alone) and kept within the
	// bars; repeated until the kept set settles; then a reading on no cycle of the kept readings (the
	// two directions of one block pair are the smallest cycle) is off unless its own witnesses hold,
	// until none changes; the poses are the averaging of each component left
	std::vector<bool> kept(candidates.size(), true);
	// every component of the kept readings averaged; poses per block, the model index its component's
	const auto AverageKept = [&]() {
		poses.assign(numBlocks, BlockPose());
		DisjointSet<uint32_t> blockSets(numBlocks);
		FOREACH(i, candidates)
			if (kept[i])
				blockSets.Union(candidates[i].sceneA, candidates[i].sceneB);
		std::map<uint32_t, std::vector<uint32_t>> componentReadings;
		FOREACH(i, candidates)
			if (kept[i])
				componentReadings[blockSets.Find(candidates[i].sceneA)].push_back((uint32_t)i);
		uint32_t model = 0;
		for (const auto& [root, readings] : componentReadings) {
			// the gauge: the block the readings weigh most on
			std::vector<float> blockWeights(numBlocks, 0.f);
			for (const uint32_t i : readings) {
				blockWeights[candidates[i].sceneA] += candidates[i].weight;
				blockWeights[candidates[i].sceneB] += candidates[i].weight;
			}
			const uint32_t gauge = (uint32_t)(std::max_element(blockWeights.begin(), blockWeights.end()) - blockWeights.begin());
			// a component whose every reading has the verified pairs across it against it weighs
			// nothing: the rotation averaging reads no reading of weight 0, so its blocks get no pose
			if (blockWeights[gauge] > 0.f) {
				std::vector<BlockPose> componentPoses;
				std::vector<Point3> residuals;
				AverageBlockPoses(candidates, readings, footprints, gauge, componentPoses, residuals);
				FOREACH(blk, componentPoses)
					if (componentPoses[blk].model != NO_ID) {
						poses[blk].T = componentPoses[blk].T;
						poses[blk].model = model;
						poses[blk].state = BlockPose::ADMITTED;
					}
			}
			++model;
		}
	};
	// every reading against the poses of its component
	const REAL barR = (REAL)config.maxGraphRotationResidual;
	const REAL barS = (REAL)config.maxGraphScaleResidual;
	const REAL barT = (REAL)config.maxGraphTranslationResidual;
	const auto Within = [&](SeamCandidate& c) {
		const BlockPose &pa = poses[c.sceneA], &pb = poses[c.sceneB];
		if (pa.state != BlockPose::ADMITTED || pb.state != BlockPose::ADMITTED || pa.model != pb.model) {
			c.residualRotation = FLT_MAX;
			return false;
		}
		std::vector<Transform> transforms(numBlocks);
		transforms[c.sceneA] = pa.T;
		transforms[c.sceneB] = pb.T;
		const SeamResidual r(ComputeSeamResidual(c.sceneA, c.sceneB, c.T, transforms, footprints));
		c.residualRotation = (float)r.rotation;
		c.residualScale = c.scaleObservable ? (float)r.scale : 1.f;
		c.residualTranslation = c.scaleObservable ? (float)r.translation : 0.f;
		return r.rotation <= barR && (!c.scaleObservable || (r.scale <= barS && r.translation <= barT));
	};
	unsigned numRounds = 0;
	for (; numRounds < kRobustRounds; ++numRounds) {
		AverageKept();
		std::vector<bool> next(candidates.size());
		FOREACH(i, candidates)
			next[i] = Within(candidates[i]);
		if (next == kept)
			break;
		kept.swap(next);
	}
	// a block pair on no cycle of the other block pairs (a bridge: no other path tests it) keeps a
	// reading only where two witnesses besides the reading's own consistency hold: its weaker side
	// supported by minSeamWeight cameras and the verified image pairs across it not holding against
	// it (PairAgreement::Holds); until none changes
	unsigned numBridgesOff = 0;
	for (bool changed = true; changed; ) {
		changed = false;
		std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> readingsOfPair;
		FOREACH(i, candidates)
			if (kept[i])
				readingsOfPair[std::make_pair(candidates[i].sceneA, candidates[i].sceneB)].push_back((uint32_t)i);
		for (const auto& [blockPair, readings] : readingsOfPair) {
			DisjointSet<uint32_t> blockSets(numBlocks);
			for (const auto& [other, otherReadings] : readingsOfPair)
				if (other != blockPair)
					blockSets.Union(other.first, other.second);
			if (blockSets.Find(blockPair.first) == blockSets.Find(blockPair.second))
				continue;
			for (const uint32_t i : readings) {
				const SeamCandidate& c = candidates[i];
				if (c.score.Weight() >= config.minSeamWeight && c.pairs.Holds())
					continue;
				VERBOSE("Seam (%u, %u) %s off: a bridge of %u cameras on its weaker side, its verified pairs %.0f for / %.0f against",
					blockPair.first, blockPair.second, SourceWord(c.source), c.score.Weight(), c.pairs.agree, c.pairs.disagree);
				kept[i] = false;
				++numBridgesOff;
				changed = true;
			}
		}
	}
	AverageKept();
	edgeSeams.clear();
	std::vector<bool> touched(numBlocks, false), onEdge(numBlocks, false);
	FOREACH(i, candidates) {
		const SeamCandidate& c = candidates[i];
		touched[c.sceneA] = touched[c.sceneB] = true;
		if (!kept[i]) {
			DEBUG_EXTRA("Seam (%u, %u) %s cut: %.2f deg, %.1f%%, %.2f%% from the solution (weight %.2f)", c.sceneA, c.sceneB,
				SourceWord(c.source), c.residualRotation, (c.residualScale - 1) * 100, c.residualTranslation * 100, c.weight);
			continue;
		}
		onEdge[c.sceneA] = onEdge[c.sceneB] = true;
		edgeSeams.push_back((uint32_t)i);
	}
	uint32_t numModels = 0;
	for (const BlockPose& pose : poses)
		if (pose.state == BlockPose::ADMITTED)
			numModels = MAXF(numModels, pose.model + 1);
	FOREACH(blk, poses) {
		BlockPose& pose = poses[blk];
		if (pose.state == BlockPose::ADMITTED)
			continue;
		pose.T = Transform();
		pose.model = numModels++;
		pose.state = BlockPose::ADMITTED;
		pose.reason = !touched[blk] ? "no seam" : (onEdge[blk] ? "no pose from the averaging" : "every reading off");
	}
	VERBOSE("Block graph: %u blocks, %u readings, %u kept after %u rounds, %u bridges off, %u models (%s)",
		numBlocks, (unsigned)candidates.size(), (unsigned)edgeSeams.size(), numRounds + 1, numBridgesOff, numModels, TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::MergeScenes(
	std::vector<Scene>& subScenes, const std::vector<IIndexArr>& localToGlobals, MergeReport& report)
{
	TD_TIMER_STARTD();

	ASSERT(!subScenes.empty());
	ASSERT(subScenes.size() == localToGlobals.size());
	const uint32_t numBlocks = (uint32_t)subScenes.size();
	report = MergeReport();
	report.numBlocks = numBlocks;
	VERBOSE("Merging %u blocks into the global scene", numBlocks);

	#if GLOBALALIGNMENT_DEBUG
	// Export sub-scenes before alignment for debugging
	FOREACH(i, subScenes)
		subScenes[i].ExportPLY(String::FormatString("subscene_%u.ply", i));
	#endif

	// the unit every translation the merge judges is read in, and where
	std::vector<BlockFootprint> footprints(numBlocks);
	FOREACH(b, subScenes)
		footprints[b] = FootprintOf(subScenes[b]);

	// the readings of every block pair the cross pairs join
	std::vector<SeamCandidate> candidates;
	EstimateSeamCandidates(subScenes, localToGlobals, candidates);

	// a single block has nothing to align against, so it is copied in as it is
	if (numBlocks == 1) {
		VERBOSE("Single block, copying directly");
		MergeSingleScene(subScenes[0], localToGlobals[0], true);
		report.poses.assign(1, BlockPose());
		report.poses[0].model = 0;
		report.poses[0].state = BlockPose::ADMITTED;
		report.numPlaced = report.numModels = 1;
		report.numImagesPlaced = scene.status.nCalibratedImages;
		DEBUG("Single-block merge completed (%s)", TD_TIMER_GET_FMT().c_str());
		return true;
	}

	// the one decision over the block graph
	std::vector<BlockPose> poses;
	std::vector<uint32_t> edgeSeams;
	SelectSeamGraph(footprints, candidates, poses, edgeSeams);

	// the model the scene is merged at: the one carrying the most images
	std::map<uint32_t, unsigned> imagesOfModel, blocksOfModel;
	FOREACH(b, poses)
		if (poses[b].state == BlockPose::ADMITTED) {
			imagesOfModel[poses[b].model] += subScenes[b].status.nCalibratedImages;
			++blocksOfModel[poses[b].model];
		}
	uint32_t mergedModel = NO_ID;
	for (const auto& [model, numImages] : imagesOfModel)
		if (mergedModel == NO_ID || numImages > imagesOfModel[mergedModel])
			mergedModel = model;

	// every block outside it is merged without its poses, for the resection to recover its images
	// against the model
	FOREACH(b, poses) {
		BlockPose& pose = poses[b];
		const bool placed = pose.state == BlockPose::ADMITTED && pose.model == mergedModel;
		if (placed)
			++report.numPlaced;
		else {
			// a block of a model of several keeps no reason of its own, being placed in that model
			if (blocksOfModel[pose.model] > 1)
				pose.reason = String::FormatString("separate model of %u blocks", blocksOfModel[pose.model]);
			pose.state = BlockPose::UNPLACEABLE;
		}
		unsigned numImages = 0;
		FOREACH(localID, subScenes[b].images) {
			const IIndex globalID = localToGlobals[b][localID];
			if (globalID == NO_ID || !subScenes[b].images[localID].IsValid())
				continue;
			++numImages;
			if (placed)
				++report.numImagesPlaced;
			else
				report.unplacedImages.push_back(globalID);
		}
		if (!placed)
			VERBOSE("Block %u not placed: %s (%u images)", (unsigned)b, pose.reason.c_str(), numImages);
	}
	report.numImagesUnplaced = report.unplacedImages.size();
	report.numModels = mergedModel == NO_ID ? 0 : 1;
	for (const uint32_t e : edgeSeams)
		if (poses[candidates[e].sceneA].state == BlockPose::ADMITTED && poses[candidates[e].sceneB].state == BlockPose::ADMITTED)
			report.modelSeams.push_back(e);

	// the closure: every correspondence of every pair of blocks the merged model holds, whether or
	// not a reading of it was kept, that the final block poses explain within the loose bar joins its
	// two tracks by construction: the model never closes its cycles exactly, and a block bent within
	// the bars is off by about that at its far end, which the merged model's adjustment then
	// straightens with these matches in its tracks
	const REAL looseBar = config.looseSeamFactor * config.maxReprojError;
	std::vector<SeamCorrespondence> seamInliers;
	for (const auto& [blockPair, pairIndices] : blockPairLinks) {
		const BlockPose &poseA = poses[blockPair.first], &poseB = poses[blockPair.second];
		if (poseA.state != BlockPose::ADMITTED || poseB.state != BlockPose::ADMITTED)
			continue;
		std::vector<SeamObservation> observations;
		std::vector<SeamCorrespondence> correspondences;
		CollectPairObservations(subScenes, blockPair.first, blockPair.second, observations, correspondences);
		const Transform T(poseB.T.Invert() * poseA.T);
		const Transform TInv(T.Invert());
		FOREACH(i, observations)
			if (SeamObservationError(observations[i], T, TInv) <= looseBar)
				seamInliers.push_back(correspondences[i]);
	}

	// the blocks of the merged model moved into its frame, and every block merged in
	MergeTransformedScenes(subScenes, localToGlobals, poses, seamInliers);

	#if GLOBALALIGNMENT_DEBUG
	// Export merged scene for debugging
	ExportMVS(MAKE_PATH("scene_merged_reconstruction.mvs"), scene);
	#endif

	report.candidates = std::move(candidates);
	report.poses = std::move(poses);
	VERBOSE("Merged %u/%u blocks into one model (%u images placed, %u unplaced) (%s)",
		report.numPlaced, numBlocks, report.numImagesPlaced, report.numImagesUnplaced,
		TD_TIMER_GET_FMT().c_str());
	return report.numPlaced >= 1;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::MergeTransformedScenes(
	std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	const std::vector<BlockPose>& poses,
	const std::vector<SeamCorrespondence>& seamInliers)
{
	// A block still admitted is one the merged model took in: every other block is unplaceable,
	// and an unplaced block is merged without poses below, so transforming it would be wasted work
	// -- and its pose, which the merged model does not hold, would inject garbage.
	ASSERT(poses.size() == subScenes.size());
	std::vector<bool> placed(subScenes.size(), false);
	FOREACH(sceneIdx, subScenes) {
		if (poses[sceneIdx].state != BlockPose::ADMITTED)
			continue;
		placed[sceneIdx] = true;
		Scene& subScene = subScenes[sceneIdx];

		// Apply the block's own pose to it, bringing it into the model frame
		subScene.Transform(poses[sceneIdx].T);

		#if GLOBALALIGNMENT_DEBUG
		// Export aligned sub-scene for debugging
		subScene.ExportPLY(String::FormatString("subscene_%u_aligned.ply", sceneIdx));
		#endif
	}

	// Unplaced blocks are merged without poses, to be rebuilt by the post-merge resection.
	std::vector<bool> unplacedImages(scene.images.size(), false);
	FOREACH(sceneIdx, subScenes)
		if (!placed[sceneIdx])
			for (const IIndex globalID : localToGlobals[sceneIdx])
				if (globalID != NO_ID && globalID < scene.images.size())
					unplacedImages[globalID] = true;

	// Track per-camera accumulation counts; destination cameras accumulate directly
	std::unordered_map<Camera*, unsigned> cameraAccumCount;

	// Merge each sub-scene into the global scene
	scene.status.nCalibratedImages = 0;
	unsigned numMerged = 0;
	FOREACH(sceneIdx, subScenes) {
		Scene& subScene = subScenes[sceneIdx];
		const IIndexArr& localToGlobal = localToGlobals[sceneIdx];
		if (placed[sceneIdx]) {
			++numMerged;
			// Accumulate intrinsics from sub-scene cameras into destination cameras
			// (unplaced blocks are excluded: their drifted geometry taints intrinsics too)
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
		MergeSingleScene(subScene, localToGlobal, placed[sceneIdx]);
	}

	// Finalize intrinsics averaging
	for (const auto& [cam, count] : cameraAccumCount) {
		if (count > 0) {
			cam->ScaleIntrinsics(REAL(1) / count);
			DEBUG_EXTRA("Camera intrinsics averaged (%u sub-scenes): %s", count, cam->GetIntrinsicsString().c_str());
		}
	}

	// Merge sub-scene tracks and connect them via cross-sub-scene pairs
	MergeTracksWithCrossSubScenePairs(unplacedImages, seamInliers);
	FilterTracks(scene, 16.f, 0.5f);

	DEBUG("Merged %u/%u transformed sub-scenes (%u tracks, %u calibrated images)",
		numMerged, (unsigned)subScenes.size(), (unsigned)scene.tracks.size(), scene.status.nCalibratedImages);
	return true;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::MergeSingleScene(Scene& subScene, const IIndexArr& localToGlobal, bool placed)
{
	// Copy image poses and move back keypoints/descriptors
	// (keypoints/descriptors were moved to sub-scenes during ExtractSubScene to save memory)
	for (IIndex localID = 0; localID < subScene.images.size(); ++localID) {
		const IIndex globalID = localToGlobal[localID];
		if (globalID == NO_ID || globalID >= scene.images.size())
			continue;

		Image& srcImg = subScene.images[localID];
		Image& dstImg = scene.images[globalID];

		if (srcImg.IsValid() && placed) {
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

		// Unplaced block: keep the observations but none of the (drifted) 3D trust
		if (!placed)
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


void GlobalAlignment::MergeTracksWithCrossSubScenePairs(
	const std::vector<bool>& unplacedImages, const std::vector<SeamCorrespondence>& seamInliers)
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
		// Tracks from unplaced blocks carry no inlier/3D trust (numInliers=0), but their
		// observation structure must survive so the post-merge resection can re-triangulate
		// them: seed them with all observations and no position.
		const bool unplacedTrack = !track.IsInlier() &&
			track.observations[0].imageID < unplacedImages.size() &&
			unplacedImages[track.observations[0].imageID];
		const uint32_t numObs = unplacedTrack
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

	// Cross-sub-scene connectivity: the seam inliers below, then every connecting pair in Phase 2,
	// share one union-find, one set of root guards, and one summary.
	unsigned numMerged = 0, numRejectedProximity = 0, numRejectedDupImage = 0, numNewPairTracks = 0;
	unsigned numCrossScenePairs = 0, numSeamMerged = 0;

	// Ensure root has metadata and feature is counted exactly once;
	// new features from cross-sub-scene pairs and seam inliers are counted as additional
	// observations but do NOT increment numInliers (these are unvalidated matches, not verified
	// inliers)
	auto AccumulateFeature = [&](uint32_t gid, IIndex imgID) {
		if (featureCounted[gid])
			return;
		featureCounted[gid] = true;
		RootMeta& meta = rootMeta[ds.Find(gid)];
		meta.InitImages();
		meta.images->emplace(imgID);
	};

	// Guard 1, shared by every union below: reject if merging would create duplicate image
	// observations (same image twice in one track), which would be geometrically invalid
	auto NoDuplicateImage = [](const RootMeta& metaDst, const RootMeta& metaSrc) -> bool {
		for (const IIndex imgID : *metaSrc.images)
			if (metaDst.images->count(imgID))
				return false;
		return true;
	};

	// The merge itself, shared by every union below once its guards pass: weighted-average 3D
	// positions, accumulate inlier counts, and merge image sets
	auto MergeRoots = [](RootMeta& metaDst, RootMeta& metaSrc) {
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
	};

	// Seam inliers: every kept correspondence of a seam the merged model rests on already agrees
	// with the placement, so its two ends join their tracks on the duplicate-image guard alone --
	// no proximity test, since the seam is itself the evidence the two sides are the same point.
	for (const SeamCorrespondence& corr : seamInliers) {
		ASSERT(corr.imageA < scene.images.size() && corr.imageB < scene.images.size());
		ASSERT(corr.featureA < scene.images[corr.imageA].keypoints.size() &&
			corr.featureB < scene.images[corr.imageB].keypoints.size());
		const uint32_t gid1 = featureOffsets[corr.imageA] + corr.featureA;
		const uint32_t gid2 = featureOffsets[corr.imageB] + corr.featureB;
		AccumulateFeature(gid1, corr.imageA);
		AccumulateFeature(gid2, corr.imageB);
		ds.UnionIf(gid1, gid2,
			[&](uint32_t rootDst, uint32_t rootSrc) -> bool {
				RootMeta& metaDst = rootMeta[rootDst];
				RootMeta& metaSrc = rootMeta[rootSrc];
				ASSERT(metaDst.images && metaSrc.images);
				if (!NoDuplicateImage(metaDst, metaSrc)) {
					++numRejectedDupImage;
					return false;
				}
				MergeRoots(metaDst, metaSrc);
				++numSeamMerged;
				return true;
			}
		);
	}

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
					if (!NoDuplicateImage(metaDst, metaSrc)) {
						++numRejectedDupImage;
						return false;
					}
					// Guard 2: if both sides have triangulated 3D positions,
					// reject if they are too far apart (indicates false match)
					if (metaDst.hasPosition && metaSrc.hasPosition && proximityThreshold > 0) {
						if (norm(metaDst.position - metaSrc.position) > proximityThreshold) {
							++numRejectedProximity;
							return false;
						}
					}
					MergeRoots(metaDst, metaSrc);
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
		"%u from seam inliers, %u new from pairs, %u rejected by proximity, %u rejected by duplicate image",
		scene.status.nTracks, scene.tracks.size(), numMerged, numCrossScenePairs,
		numSeamMerged, numNewPairTracks, numRejectedProximity, numRejectedDupImage);
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
