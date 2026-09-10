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

// The diagonal of the box a set of camera centres spans
static REAL CentresExtent(const std::vector<Point3>& centres)
{
	AABB3 bbox(true);
	for (const Point3& C : centres)
		bbox.InsertFull(C);
	return bbox.IsEmpty() ? REAL(0) : (REAL)bbox.GetSize().norm();
}

// The diagonal of the box a block's cameras span in its own frame -- the unit the translation
// residuals of its seams are measured in
static REAL BlockExtent(const Scene& block)
{
	std::vector<Point3> centres;
	for (const Image& img : block.images)
		if (img.IsValid())
			centres.emplace_back(img.C);
	return CentresExtent(centres);
}

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

	// the unit every translation the merge judges is read in
	std::vector<REAL> blockExtents(numBlocks);
	FOREACH(b, subScenes)
		blockExtents[b] = BlockExtent(subScenes[b]);

	// stages 1 and 2: the evidence, and what every adjacent block pair measures of it
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

	std::vector<BlockPose> poses;
	std::vector<std::vector<uint32_t>> modelSeams;
	uint32_t mergedModel = NO_ID;
	if (!candidates.empty()) {
		ClassifySeamGraph(blockExtents, candidates);                          // stage 3
		ComputeInitialBlockPoses(candidates, blockExtents, numBlocks, poses); // stage 4
		uint32_t numModels = 0;
		for (const BlockPose& pose : poses)
			if (pose.model != NO_ID)
				numModels = MAXF(numModels, pose.model + 1);
		// stage 5: every model grows on its own, the one holding the most trusted weight first
		modelSeams.resize(numModels);
		unsigned mostImages = 0;
		for (uint32_t m = 0; m < numModels; ++m) {
			if (PlaceBlocks(subScenes, candidates, blockExtents, m, poses, modelSeams[m]) == 0)
				continue;
			++report.numModels;
			// stage 6: the model carrying the most images is the one the scene is merged at
			unsigned numImages = 0;
			FOREACH(b, poses)
				if (poses[b].state == BlockPose::ADMITTED && poses[b].model == m)
					numImages += subScenes[b].status.nCalibratedImages;
			if (mergedModel == NO_ID || numImages > mostImages) {
				mergedModel = m;
				mostImages = numImages;
			}
		}
	}
	// no trusted seam carried a model: the largest block seeds one at its own frame and the
	// placement grows it on whatever the graph left undecided, so a good partial reconstruction is
	// never discarded
	if (mergedModel == NO_ID) {
		const uint32_t largest = FindLargestSubScene(subScenes);
		VERBOSE("warning: no trusted seam placed a block; growing a model from block %u (%u/%u images)",
			largest, subScenes[largest].status.nCalibratedImages, (unsigned)subScenes[largest].images.size());
		poses.assign(numBlocks, BlockPose());
		poses[largest].model = mergedModel = 0;
		modelSeams.assign(1, std::vector<uint32_t>());
		PlaceBlocks(subScenes, candidates, blockExtents, mergedModel, poses, modelSeams[mergedModel]);
		report.numModels = 1;
	}

	// every block outside the merged model is merged without its poses, for the resection to
	// recover its images against the consensus
	FOREACH(b, poses) {
		BlockPose& pose = poses[b];
		if (pose.state == BlockPose::ADMITTED && pose.model == mergedModel) {
			++report.numPlaced;
			continue;
		}
		if (pose.state == BlockPose::ADMITTED)
			pose.reason = "separate component";
		else if (pose.reason.empty())
			pose.reason = "no seam";
		pose.state = BlockPose::UNPLACEABLE;
		VERBOSE("Block %u not placed: %s", (unsigned)b, pose.reason.c_str());
	}
	// what the merge registers, and what it leaves to the resection
	FOREACH(b, poses) {
		const bool placed = poses[b].state == BlockPose::ADMITTED;
		FOREACH(localID, subScenes[b].images) {
			const IIndex globalID = localToGlobals[b][localID];
			if (globalID == NO_ID || !subScenes[b].images[localID].IsValid())
				continue;
			if (placed)
				++report.numImagesPlaced;
			else
				report.unplacedImages.push_back(globalID);
		}
	}
	report.numImagesUnplaced = report.unplacedImages.size();

	// stage 7: the placed blocks moved into the model frame, and every block merged in
	MergeTransformedScenes(subScenes, localToGlobals, poses);

	#if GLOBALALIGNMENT_DEBUG
	// Export merged scene for debugging
	ExportMVS(MAKE_PATH("scene_merged_reconstruction.mvs"), scene);
	#endif

	for (const SeamCandidate& c : candidates)
		switch (c.cls) {
		case SeamCandidate::ROBUST: ++report.numRobust; break;
		case SeamCandidate::VERIFIED: ++report.numVerified; break;
		case SeamCandidate::REJECTED: ++report.numRejected; break;
		default: ++report.numUndecided; break;
		}
	report.modelSeams = std::move(modelSeams[mergedModel]);
	report.candidates = std::move(candidates);
	report.poses = std::move(poses);
	VERBOSE("Merged %u/%u blocks into one model (%u images placed, %u unplaced) (%s)",
		report.numPlaced, numBlocks, report.numImagesPlaced, report.numImagesUnplaced,
		TD_TIMER_GET_FMT().c_str());
	return report.numPlaced >= 1;
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
// What an undecided candidate is worth when the placement ranks the blocks waiting to be tried:
// evidence enough to be looked at, not enough to outrank a trusted seam.
constexpr float kUndecidedSupportWeight = 0.25f;
// At most this share of the admitted neighbours that have something to say may contradict a
// placement: one in three, so a lone neighbour cannot veto and a majority still can.
constexpr unsigned kNeighbourContraShare = 3;

// A block the model already holds: admitted, and admitted into this model. Another model placed
// its blocks in a frame this one knows nothing about, so they have no say here.
bool IsInModel(const std::vector<BlockPose>& poses, uint32_t block, uint32_t model)
{
	return poses[block].state == BlockPose::ADMITTED && poses[block].model == model;
}

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

// One direction of a seam: every correspondence of the direction is appended to `observations`,
// tagged with which side of the seam it came from, and `inlierMask` grows with them to say which
// ones the estimator kept: it stays parallel to `observations`, so calling this once per direction
// leaves one mask over both. The seam is scored on all of them, whichever of the two directions
// ends up carrying it. Returns 0 when the direction yields no usable estimate, the correspondences
// being appended all the same.
unsigned EstimateSeamDirection(
	const RigCorrespondences& rc, const Scene& rigScene,
	const poselib::RansacOptions& ransac, bool pointsAreA, Transform& T,
	std::vector<SeamObservation>& observations, std::vector<SeamCorrespondence>& correspondences,
	std::vector<bool>& inlierMask)
{
	const size_t first = observations.size();
	ASSERT(inlierMask.size() == first);
	AppendRigObservations(rc, rigScene, pointsAreA, observations, correspondences);
	inlierMask.resize(observations.size(), false);

	std::vector<std::vector<char>> inliers;
	const unsigned numInliers = EstimateRigAgainstPoints(rc, ransac, T, inliers);
	if (numInliers == 0)
		return 0;

	// the mask runs in the same order the correspondences were appended in, behind whatever an
	// earlier direction already left in it
	size_t k = first;
	for (size_t r = 0; r < inliers.size(); ++r)
		for (size_t i = 0; i < inliers[r].size(); ++i, ++k)
			inlierMask[k] = inliers[r][i] != 0;
	ASSERT(k == inlierMask.size());
	return numInliers;
}

// The rig of one direction of a placement, read straight off the pool: the cameras of the side
// that observes -- the group's for `forward` false, the model's for true -- against the points of
// the other, each already in the frame its own side is judged in. What the seam collector reads
// off two scenes, a placement reads off the pool, so both directions are estimated the same way.
void PoolRigCorrespondences(const PlacementPool& pool, bool forward, float maxReprojError, RigCorrespondences& rc)
{
	rc = RigCorrespondences();
	std::unordered_map<IIndex, uint32_t> rigIndexOf;
	for (const SeamObservation& obs : pool.observations) {
		if (obs.forward != forward)
			continue;
		const auto [it, inserted] = rigIndexOf.emplace(obs.rigImage, (uint32_t)rc.cameraExt.size());
		if (inserted) {
			rc.cameraExt.emplace_back(obs.R, Point3(obs.R * (-obs.C)));
			rc.bearings.emplace_back();
			rc.points.emplace_back();
			// the angle at which this camera reads the pixel threshold, back out of the rate the
			// observation carries to convert the other way
			rc.maxError = MAXF(rc.maxError,
				obs.pixelPerRadian > 0 ? (double)maxReprojError / obs.pixelPerRadian : 0.0);
		}
		rc.bearings[it->second].emplace_back(normalized(obs.bearing));
		rc.points[it->second].push_back(obs.X);
		++rc.numCorrespondences;
	}
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

// Move the correspondences of one block pair (a < b) into the two frames a placement is judged in:
// each side by the frame of the block it belongs to, and the forward flag re-read from the group's
// point of view, so a pooled observation says only whether the point is the group's or the model's.
void AppendPoolObservations(
	const std::vector<SeamObservation>& observations, const std::vector<SeamCorrespondence>& correspondences,
	uint32_t a, uint32_t b, const std::vector<Transform>& frameOf, const std::vector<bool>& inGroup,
	uint32_t candidateSlot, PlacementPool& pool)
{
	ASSERT(observations.size() == correspondences.size());
	FOREACH(i, observations) {
		const SeamObservation& obs = observations[i];
		// the point belongs to one block of the pair, the camera that saw it to the other
		const uint32_t pointBlock = obs.forward ? a : b, cameraBlock = obs.forward ? b : a;
		const Transform& pointFrame = frameOf[pointBlock];
		const Transform& cameraFrame = frameOf[cameraBlock];
		SeamObservation moved(obs);
		moved.X = pointFrame * obs.X;
		moved.R = RMatrix(obs.R * cameraFrame.R.t());
		moved.C = cameraFrame * obs.C;
		moved.forward = inGroup[pointBlock];
		pool.observations.emplace_back(moved);
		pool.correspondences.push_back(correspondences[i]);
		pool.observationCandidate.push_back(candidateSlot);
	}
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

void GlobalAlignment::RefineBlockPoses(
	const std::vector<SeamCandidate>& candidates,
	const std::vector<uint32_t>& modelSeams,
	const uint32_t model, const uint32_t gaugeBlock,
	std::vector<BlockPose>& poses) const
{
	ASSERT(gaugeBlock < poses.size() && IsInModel(poses, gaugeBlock, model));
	std::vector<BlockParameters> parameters;
	parameters.reserve(poses.size());
	for (const BlockPose& pose : poses)
		parameters.emplace_back(pose.T);

	// one residual per inlier observation of every seam the model rests on, tying the block the
	// point belongs to to the block of the camera that saw it
	ceres::Problem problem;
	ceres::LossFunction* loss = new ceres::HuberLoss(config.maxReprojError);
	unsigned numResiduals = 0;
	for (const uint32_t e : modelSeams) {
		const SeamCandidate& c = candidates[e];
		if (!IsInModel(poses, c.sceneA, model) || !IsInModel(poses, c.sceneB, model))
			continue;
		const size_t numScored = MINF(c.observations.size(), c.score.inlierMask.size());
		for (size_t i = 0; i < numScored; ++i) {
			if (!c.score.inlierMask[i])
				continue;
			const SeamObservation& obs = c.observations[i];
			BlockParameters& point = parameters[obs.forward ? c.sceneA : c.sceneB];
			BlockParameters& camera = parameters[obs.forward ? c.sceneB : c.sceneA];
			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<BlockSeamReprojectionError, 3, 4, 3, 1, 4, 3, 1>(
					new BlockSeamReprojectionError(obs)),
				loss, point.q, point.t, &point.logScale, camera.q, camera.t, &camera.logScale);
			++numResiduals;
		}
	}
	if (numResiduals == 0)
		return;
	unsigned numFree = 0;
	FOREACH(b, parameters) {
		if (!problem.HasParameterBlock(parameters[b].q))
			continue;
		problem.SetManifold(parameters[b].q, new ceres::QuaternionManifold);
		++numFree;
	}
	// the gauge: the model frame is this block's own, so its pose is what everything else moves against
	if (problem.HasParameterBlock(parameters[gaugeBlock].q)) {
		problem.SetParameterBlockConstant(parameters[gaugeBlock].q);
		problem.SetParameterBlockConstant(parameters[gaugeBlock].t);
		problem.SetParameterBlockConstant(&parameters[gaugeBlock].logScale);
		--numFree;
	}

	ceres::Solver::Options options;
	// seven parameters per block against many thousands of residuals, and every residual touches
	// only two blocks: the normal equations are sparse and stay small however many blocks there are
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 50;
	options.function_tolerance = 1e-8;
	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	if (!summary.IsSolutionUsable())
		return;

	FOREACH(b, poses)
		if (problem.HasParameterBlock(parameters[b].q) && ISFINITE(parameters[b].logScale))
			poses[b].T = parameters[b].ToTransform();
	DEBUG_ULTIMATE("Refined %u block poses over %u seam observations: cost %g -> %g",
		numFree, numResiduals, summary.initial_cost, summary.final_cost);
}
/*----------------------------------------------------------------*/

void GlobalAlignment::RefineSeamTransform(
	const std::vector<SeamObservation>& observations, const float maxReprojError, Transform& T) const
{
	// the seam is judged at the same pixel bar the joint refinement charges
	ASSERT(maxReprojError == config.maxReprojError);
	if (observations.empty())
		return;
	// a model of two blocks: block A gauges it at the identity, so the seam travels in and out
	// through block B's pose, and every observation given is evidence the fit has to answer for
	SeamCandidate seam;
	seam.sceneA = 0;
	seam.sceneB = 1;
	seam.observations = observations;
	seam.score.inlierMask.assign(observations.size(), true);
	seam.score.inliers = (unsigned)observations.size();
	std::vector<BlockPose> poses(2);
	for (BlockPose& pose : poses) {
		pose.model = 0;
		pose.state = BlockPose::ADMITTED;
	}
	poses[1].T = T.Invert();
	RefineBlockPoses({seam}, {0u}, 0, 0, poses);
	T = poses[1].T.Invert();
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
	refusedSeamPairs.clear();
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

void GlobalAlignment::BuildPlacementPool(
	const std::vector<Scene>& subScenes,
	const std::vector<SeamCandidate>& candidates,
	const std::vector<BlockPose>& poses,
	const uint32_t model,
	const BlockGroup& group,
	PlacementPool& pool) const
{
	ASSERT(group.blocks.size() == group.frames.size() && poses.size() == subScenes.size());
	pool = PlacementPool();
	pool.group = group;

	// where every block that has a say sits: a group block in the group frame, a block this model
	// has admitted in the model frame
	std::vector<Transform> frameOf(poses.size());
	std::vector<bool> inGroup(poses.size(), false), inModel(poses.size(), false);
	FOREACH(g, group.blocks) {
		frameOf[group.blocks[g]] = group.frames[g];
		inGroup[group.blocks[g]] = true;
	}
	FOREACH(b, poses)
		if (IsInModel(poses, b, model) && !inGroup[b]) {
			frameOf[b] = poses[b].T;
			inModel[b] = true;
		}
	const auto JoinsGroupToModel = [&inGroup, &inModel](uint32_t a, uint32_t b) {
		return (inGroup[a] && inModel[b]) || (inModel[a] && inGroup[b]);
	};

	// every candidate of a group-to-model pair contributes what its cameras saw, whatever its class
	std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> pairCandidates;
	FOREACH(i, candidates) {
		const SeamCandidate& c = candidates[i];
		if (JoinsGroupToModel(c.sceneA, c.sceneB))
			pairCandidates[std::make_pair(c.sceneA, c.sceneB)].push_back(i);
	}
	for (const auto& [blockPair, indices] : pairCandidates) {
		// the parallel candidates of a pair were all scored on the same union of both directions,
		// so that union enters the pool once and every one of them is recorded behind it
		const uint32_t slot = (uint32_t)pool.candidateIdx.size();
		for (const uint32_t i : indices)
			pool.candidateIdx.push_back(i);
		const SeamCandidate& c = candidates[indices.front()];
		AppendPoolObservations(c.observations, c.correspondences,
			blockPair.first, blockPair.second, frameOf, inGroup, slot, pool);
	}
	// a pair no candidate covers still carries correspondences, and they are evidence too --
	// unless the gates already weighed that pair and refused it
	for (const auto& [blockPair, links] : blockPairLinks) {
		if (!JoinsGroupToModel(blockPair.first, blockPair.second) ||
			pairCandidates.count(blockPair) || refusedSeamPairs.count(blockPair))
			continue;
		std::vector<SeamObservation> observations;
		std::vector<SeamCorrespondence> correspondences;
		CollectPairObservations(subScenes, blockPair.first, blockPair.second, observations, correspondences);
		AppendPoolObservations(observations, correspondences,
			blockPair.first, blockPair.second, frameOf, inGroup, NO_ID, pool);
	}

	// the cameras of both sides, each already in the frame its side is judged in
	FOREACH(b, poses) {
		if (!inGroup[b] && !inModel[b])
			continue;
		std::vector<Point3>& centres = inGroup[b] ? pool.groupCentres : pool.modelCentres;
		for (const Image& img : subScenes[b].images)
			if (img.IsValid())
				centres.emplace_back(frameOf[b] * img.C);
	}
	DEBUG_ULTIMATE("Placement pool of %u blocks: %u observations from %u candidates, %u against %u cameras",
		(unsigned)group.blocks.size(), (unsigned)pool.observations.size(), (unsigned)pool.candidateIdx.size(),
		(unsigned)pool.groupCentres.size(), (unsigned)pool.modelCentres.size());
}
/*----------------------------------------------------------------*/

void GlobalAlignment::ScoreHypothesis(
	const std::vector<Scene>& subScenes,
	const PlacementPool& pool,
	const unsigned bestOwnInliers,
	const float voteRatio,
	PlacementHypothesis& h) const
{
	// side 0 is the group the hypothesis places, side 1 the model it is placed in
	const std::vector<uint32_t>& groupBlocks = pool.group.blocks;
	ScoreSeam(subScenes, pool.observations, pool.correspondences, h.T,
		[this, &groupBlocks](IIndex image) {
			return std::find(groupBlocks.begin(), groupBlocks.end(), globalToLocal.at(image).first) != groupBlocks.end() ? 0 : 1;
		},
		pool.groupCentres, pool.modelCentres, h.score);
	h.failedGate = FailedGates(h.score, h.score.inliers, bestOwnInliers, voteRatio);
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
	std::vector<SeamCandidate>& candidates)
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
	inliers[0] = EstimateSeamDirection(rc[0], subScenes[b], ransacOptions, true,
		Tseam[0], observations, correspondences, estimatorMask);
	inliers[1] = EstimateSeamDirection(rc[1], subScenes[a], ransacOptions, false,
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
		// the gates weighed the whole pair and threw it out: what its cameras saw is not evidence
		// a placement may pick up again
		refusedSeamPairs.emplace(a, b);
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
	const std::vector<uint32_t>& pairIndices, std::vector<SeamCandidate>& candidates)
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
		refusedSeamPairs.emplace(a, b);
		DEBUG_ULTIMATE("Seam (%u, %u) %s dropped by %s", a, b, SourceWord(c.source), failed.c_str());
		return;
	}
	LogSeamCandidate(a, b, c);
	candidates.emplace_back(std::move(c));
}
/*----------------------------------------------------------------*/

// the per-block local→model Sim(3) the averaging produces: it solves R_i mapping model→local
// (the same convention as Image.R) while the similarity transform applies local→model, hence
// the transpose
static SEACAVE::Transform BuildGlobalTransform(const Point3d& rotation, REAL scale, const Point3& translation)
{
	SEACAVE::Transform G;
	G.R = RMatrix(rotation).t();
	G.scale = scale;
	G.t = translation;
	return G;
}

// How far apart two similarities that carry the same frame into the same one are. Each component
// is also expressed as a factor of the limit it must stay under, so the three are comparable and
// the worst seam of a graph is well defined whichever way it is wrong.
struct SeamResidual {
	REAL scale;       // ratio, >= 1
	REAL rotation;    // degrees
	REAL translation; // fraction of the camera footprint the two frames share
	REAL excess;      // largest of the three as a factor of its limit; conflicting above 1
	// the two say the same thing: every component of the discrepancy stays under its own bar
	bool Agrees() const { return excess <= REAL(1); }
};

// The one comparison of two transforms, which every rule that asks whether two opinions on the
// same two frames agree is written in terms of: the discrepancy E = first^-1 * second, read in the
// frame they both start from, against `extent` -- the camera footprint measured in that same frame.
static SeamResidual CompareTransforms(
	const Transform& first, const Transform& second, const REAL extent, const GlobalAlignmentConfig& config)
{
	const Transform E = first.Invert() * second;
	SeamResidual res;
	res.scale = MAXF(E.scale, REAL(1) / E.scale);
	res.rotation = R2D(ACOS(ComputeAngle(Matrix3x3(E.R))));
	res.translation = extent > 0 ? norm(E.t) / extent : REAL(0);
	res.excess = MAXF(MAXF(res.scale / config.maxSimScaleRatio, res.rotation / config.maxSimRotationError),
		res.translation / config.maxSimTranslationError);
	return res;
}

// Sim(3) cycle residual of one measured seam: T maps A-local to B-local and G_i maps each local
// frame to the model frame, so G_B*T and G_A both map A-local to the model and their discrepancy
// is what the seam is off by (identity when perfectly consistent). It is read in A's local frame,
// against the larger of the two blocks' camera footprints, so the verdict does not depend on which
// end happens to have the lower block index -- and a block whose cameras sit a millimetre apart,
// having no footprint of its own that means anything, is judged against the one it is joined to.
static SeamResidual ComputeSeamResidual(
	const uint32_t a, const uint32_t b, const Transform& T, const std::vector<Transform>& transforms,
	const std::vector<REAL>& blockExtents, const GlobalAlignmentConfig& config)
{
	const REAL diag = MAXF(blockExtents[a] * transforms[a].scale, blockExtents[b] * transforms[b].scale);
	return CompareTransforms(transforms[a], transforms[b] * T, diag / transforms[a].scale, config);
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
					// the larger of the two footprints, as everything that judges a seam's
					// translation reads it: a block of two cameras a millimetre apart has none
					const REAL extent = MAXF(blockExtents[blockOfNode[p.idxA]] * scales[p.idxA],
						blockExtents[blockOfNode[p.idxB]] * scales[p.idxB]);
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

// The camera footprint of a group in its own frame: the unit a discrepancy between two placements
// of that group is read against
static REAL GroupExtent(const BlockGroup& group, const std::vector<REAL>& blockExtents)
{
	REAL extent = 0;
	FOREACH(g, group.blocks)
		extent = MAXF(extent, blockExtents[group.blocks[g]] * group.frames[g].scale);
	return extent;
}

// Where every block sits under one placement: the blocks the model already holds at their model
// poses, the group's blocks where the hypothesis puts them
static void PlacementTransforms(
	const std::vector<BlockPose>& poses, const uint32_t model, const BlockGroup& group,
	const Transform& T, std::vector<Transform>& transforms)
{
	transforms.assign(poses.size(), Transform());
	FOREACH(b, poses)
		if (IsInModel(poses, b, model))
			transforms[b] = poses[b].T;
	FOREACH(g, group.blocks)
		transforms[group.blocks[g]] = T * group.frames[g];
}

// Every candidate joining a block of the group to a block the model has already admitted, with
// that neighbour. The neighbour check of a placement and the admission that follows it walk
// exactly these, so both answer to the same seams. A rejected candidate is not among them: the
// graph has already disowned its transform, and it is only a transform that is asked for here.
static void ForEachGroupSeam(
	const std::vector<SeamCandidate>& candidates, const std::vector<BlockPose>& poses,
	const uint32_t model, const BlockGroup& group,
	const std::function<void(uint32_t, uint32_t)>& visit)
{
	std::vector<bool> inGroup(poses.size(), false);
	for (const uint32_t g : group.blocks)
		inGroup[g] = true;
	FOREACH(i, candidates) {
		const SeamCandidate& c = candidates[i];
		if (c.cls == SeamCandidate::REJECTED)
			continue;
		for (const uint32_t neighbour : {c.sceneA, c.sceneB}) {
			const uint32_t own = neighbour == c.sceneA ? c.sceneB : c.sceneA;
			if (inGroup[own] && !inGroup[neighbour] && IsInModel(poses, neighbour, model))
				visit((uint32_t)i, neighbour);
		}
	}
}

// What the admitted neighbours make of a placement: those whose own seams agree with what it
// implies, those whose trusted seam disagrees -- a cycle the model will have to answer for -- and
// those that simply contradict it
struct NeighbourVerdict { unsigned support{0}, loop{0}, contra{0}; };

static NeighbourVerdict CheckNeighbours(
	const std::vector<SeamCandidate>& candidates, const std::vector<REAL>& blockExtents,
	const std::vector<BlockPose>& poses, const uint32_t model, const BlockGroup& group,
	const std::map<uint32_t, unsigned>& numPooled, const Transform& T,
	const GlobalAlignmentConfig& config)
{
	std::vector<Transform> transforms;
	PlacementTransforms(poses, model, group, T, transforms);
	// one verdict per neighbour, whatever the number of opinions it holds on the group
	struct NeighbourSeams { bool agrees{false}, trusted{false}; };
	std::map<uint32_t, NeighbourSeams> seamsOf;
	ForEachGroupSeam(candidates, poses, model, group, [&](uint32_t i, uint32_t neighbour) {
		// a neighbour is only asked about a group it shares enough of the pool with
		const auto itPooled = numPooled.find(neighbour);
		if (itPooled == numPooled.end() || itPooled->second < config.minCommonTracks)
			return;
		const SeamCandidate& c = candidates[i];
		const SeamResidual residual = ComputeSeamResidual(
			c.sceneA, c.sceneB, c.T, transforms, blockExtents, config);
		NeighbourSeams& seams = seamsOf[neighbour];
		if (residual.Agrees())
			seams.agrees = true;
		else if (c.IsTrusted())
			seams.trusted = true;
		DEBUG_ULTIMATE("Placement against block %u: seam (%u, %u) off by %.2f deg, %.1f%%, %.2f%% -> %s",
			neighbour, c.sceneA, c.sceneB, residual.rotation, (residual.scale - 1) * 100,
			residual.translation * 100, residual.Agrees() ? "agrees" : "disagrees");
	});
	NeighbourVerdict verdict;
	for (const auto& [neighbour, seams] : seamsOf) {
		if (seams.agrees)
			++verdict.support;
		else if (seams.trusted)
			++verdict.loop;
		else
			++verdict.contra;
	}
	return verdict;
}

// What a block weighs against the blocks the model already holds: its trusted seams to them in
// full and its undecided ones at a fraction, so a block only undecided evidence reaches is still
// tried, after every block a trusted seam carries
static float PooledSupport(
	const std::vector<SeamCandidate>& candidates, const std::vector<BlockPose>& poses,
	const uint32_t model, const uint32_t block)
{
	float support = 0;
	for (const SeamCandidate& c : candidates) {
		const uint32_t other = c.sceneA == block ? c.sceneB : (c.sceneB == block ? c.sceneA : NO_ID);
		if (other == NO_ID || !IsInModel(poses, other, model))
			continue;
		if (c.IsTrusted())
			support += c.weight;
		else if (c.cls == SeamCandidate::UNDECIDED)
			support += kUndecidedSupportWeight * c.weight;
	}
	return support;
}

// The 3D-3D similarity of a group against the blocks the model holds: the tracks both sides
// triangulated, paired through the cross correspondence they share -- the point mode's own
// estimate, read off the pool instead of off one block pair. Returns 0 when the evidence does not
// carry an estimate.
static unsigned EstimatePoolSimilarity(
	const PlacementPool& pool, const GlobalAlignmentConfig& config, Transform& T)
{
	// a correspondence collected in both directions has its point triangulated in both frames
	typedef std::tuple<IIndex, IIndex, uint32_t, uint32_t> CorrespondenceKey;
	const auto KeyOf = [](const SeamCorrespondence& corr) {
		return std::make_tuple(corr.imageA, corr.imageB, corr.featureA, corr.featureB);
	};
	std::map<CorrespondenceKey, Point3> groupPoints;
	FOREACH(i, pool.observations)
		if (pool.observations[i].forward)
			groupPoints.emplace(KeyOf(pool.correspondences[i]), pool.observations[i].X);
	Point3Arr srcPoints, dstPoints;
	FOREACH(i, pool.observations) {
		if (pool.observations[i].forward)
			continue;
		const auto it = groupPoints.find(KeyOf(pool.correspondences[i]));
		if (it == groupPoints.end())
			continue;
		srcPoints.emplace_back(it->second);
		dstPoints.emplace_back(pool.observations[i].X);
	}
	if (srcPoints.size() < config.minCommonTracks)
		return 0;
	// the same relative inlier distance the point mode measures a seam at
	AABB3 dstBbox(true);
	for (const Point3& p : dstPoints)
		dstBbox.InsertFull(p);
	const double threshold = config.simInlierThresholdFactor * dstBbox.GetSize().norm();
	if (!(threshold > 0))
		return 0;
	const unsigned numInliers = EstimateSimilarityTransform(
		srcPoints, dstPoints, T, threshold, true, config.simRansacMaxIters);
	return numInliers >= config.minCommonTracks &&
		(double)numInliers >= config.minSimInlierRatio * (double)srcPoints.size() ? numInliers : 0;
}

// How a placement was read, for the log
static const char* PlacementWord(PlacementHypothesis::Source source)
{
	switch (source) {
	case PlacementHypothesis::RIG_ON_MODEL: return "group on the model's points";
	case PlacementHypothesis::MODEL_ON_GROUP: return "model on the group's points";
	default: return "averaged";
	}
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::PlaceGroup(
	const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t model,
	const BlockGroup& group,
	const std::vector<BlockPose>& poses,
	PlacementHypothesis& winner,
	String& reason) const
{
	ASSERT(!group.blocks.empty() && group.blocks.size() == group.frames.size());
	reason.clear();
	PlacementPool pool;
	BuildPlacementPool(subScenes, candidates, poses, model, group, pool);

	// the ways the pool can be read, and what the averaging already said; every one of them is
	// scored over the whole pool below, so they all answer to the same evidence
	std::vector<PlacementHypothesis> hypotheses;
	unsigned bestOwnInliers = 0;
	if (config.alignment == GlobalAlignmentConfig::ALIGN_POINTS) {
		PlacementHypothesis h;
		h.source = PlacementHypothesis::RIG_ON_MODEL;
		bestOwnInliers = EstimatePoolSimilarity(pool, config, h.T);
		if (bestOwnInliers > 0)
			hypotheses.emplace_back(h);
	} else {
		const poselib::RansacOptions ransacOptions = SeamRansacOptions();
		RigCorrespondences rc;
		std::vector<std::vector<char>> inliers;
		Transform T;
		// the group's cameras on the model's points: that direction solves the model into the
		// group's frame, so the seam it measures is its inverse
		PoolRigCorrespondences(pool, false, config.maxReprojError, rc);
		unsigned numInliers = EstimateRigAgainstPoints(rc, ransacOptions, T, inliers);
		if (numInliers > 0) {
			PlacementHypothesis h;
			h.source = PlacementHypothesis::RIG_ON_MODEL;
			h.T = T.Invert();
			bestOwnInliers = MAXF(bestOwnInliers, numInliers);
			hypotheses.emplace_back(h);
		}
		// the model's cameras on the group's points, which solves the group into the model
		PoolRigCorrespondences(pool, true, config.maxReprojError, rc);
		numInliers = EstimateRigAgainstPoints(rc, ransacOptions, T, inliers);
		if (numInliers > 0) {
			PlacementHypothesis h;
			h.source = PlacementHypothesis::MODEL_ON_GROUP;
			h.T = T;
			bestOwnInliers = MAXF(bestOwnInliers, numInliers);
			hypotheses.emplace_back(h);
		}
	}
	// where the averaging put a single block of this model, held to the same gates as the rest
	if (group.blocks.size() == 1 && poses[group.blocks.front()].model == model) {
		PlacementHypothesis h;
		h.source = PlacementHypothesis::INITIAL;
		h.T = poses[group.blocks.front()].T * group.frames.front().Invert();
		hypotheses.emplace_back(h);
	}
	if (hypotheses.empty()) {
		reason = "no hypothesis";
		return false;
	}

	// how much of the pool each admitted block carries: the bar a neighbour clears before its
	// opinion is asked for
	std::map<uint32_t, unsigned> numPooled;
	for (const SeamObservation& obs : pool.observations) {
		// the model side holds the camera when the point is the group's, and the point otherwise
		const auto it = globalToLocal.find(obs.forward ? obs.rigImage : obs.pointImage);
		if (it != globalToLocal.end())
			++numPooled[it->second.first];
	}
	// a model the group reaches over verified seams alone is held to the stricter vote margin
	bool anyRobust = false;
	ForEachGroupSeam(candidates, poses, model, group, [&](uint32_t i, uint32_t) {
		anyRobust = anyRobust || candidates[i].cls == SeamCandidate::ROBUST;
	});
	const float voteRatio = anyRobust ? config.minCameraVoteRatio : config.minCameraVoteRatioVerified;

	const uint32_t firstBlock = group.blocks.front();
	for (PlacementHypothesis& h : hypotheses) {
		ScoreHypothesis(subScenes, pool, bestOwnInliers, voteRatio, h);
		const NeighbourVerdict verdict = CheckNeighbours(
			candidates, blockExtents, poses, model, group, numPooled, h.T, config);
		h.neighbourSupport = verdict.support;
		h.neighbourLoop = verdict.loop;
		h.neighbourContra = verdict.contra;
		// the neighbours the model already holds: one behind the placement at least, and no more
		// than a share of those with something to say against it
		if (h.Passed() && (verdict.support == 0 || verdict.contra * kNeighbourContraShare >
				verdict.support + verdict.loop + verdict.contra))
			h.failedGate = "neighbours";
		LogVotes(String::FormatString("Placement of block %u, %s",
			firstBlock, PlacementWord(h.source)).c_str(), h.score);
		DEBUG_ULTIMATE("Placement of block %u, %s: %u inliers of %u, %u+/%u- neighbours%s%s",
			firstBlock, PlacementWord(h.source), h.score.inliers, (unsigned)pool.observations.size(),
			verdict.support, verdict.contra, h.Passed() ? "" : ", dropped by ", h.failedGate.c_str());
	}

	// the largest vote of the cameras decides between them
	const auto Votes = [](const PlacementHypothesis& h) { return h.score.support[0] + h.score.support[1]; };
	std::sort(hypotheses.begin(), hypotheses.end(),
		[&Votes](const PlacementHypothesis& a, const PlacementHypothesis& b) { return Votes(a) > Votes(b); });
	const auto itWinner = std::find_if(hypotheses.begin(), hypotheses.end(),
		[](const PlacementHypothesis& h) { return h.Passed(); });
	if (itWinner == hypotheses.end()) {
		// nothing carried the group: the best of them says why
		winner = hypotheses.front();
		reason = winner.failedGate;
		return false;
	}
	winner = *itWinner;
	// the unit two placements are held apart in: the larger of the group's own footprint and the
	// model's, read in the group frame their discrepancy lives in
	const REAL extent = MAXF(GroupExtent(group, blockExtents),
		CentresExtent(pool.modelCentres) / winner.T.scale);
	const PlacementHypothesis* rival = NULL;
	for (auto it = itWinner + 1; it != hypotheses.end(); ++it) {
		if (!it->Passed())
			continue;
		if (CompareTransforms(winner.T, it->T, extent, config).Agrees()) {
			// two readings of one placement: what either of them explains, it explains
			FOREACH(k, winner.score.inlierMask)
				if (it->score.inlierMask[k] && !winner.score.inlierMask[k]) {
					winner.score.inlierMask[k] = true;
					++winner.score.inliers;
				}
		} else if (rival == NULL)
			rival = &*it;
	}
	if (rival != NULL && (float)Votes(winner) < config.voteMargin * (float)Votes(*rival)) {
		reason = String::FormatString("two placements disagree (%u against %u supporting cameras)",
			Votes(winner), Votes(*rival));
		return false;
	}
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::AdmitGroup(
	MAYBEUNUSED const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t model,
	const BlockGroup& group,
	const PlacementHypothesis& winner,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams) const
{
	ASSERT(poses.size() == subScenes.size() && group.blocks.size() == group.frames.size());
	// the group takes its place
	FOREACH(g, group.blocks) {
		BlockPose& pose = poses[group.blocks[g]];
		pose.T = winner.T * group.frames[g];
		pose.model = model;
		pose.state = BlockPose::ADMITTED;
		pose.reason.clear();
	}
	std::vector<Transform> transforms;
	PlacementTransforms(poses, model, group, winner.T, transforms);

	// what every seam between the group and the model makes of where the group landed
	ForEachGroupSeam(candidates, poses, model, group, [&](uint32_t i, uint32_t) {
		SeamCandidate& c = candidates[i];
		const SeamResidual residual = ComputeSeamResidual(
			c.sceneA, c.sceneB, c.T, transforms, blockExtents, config);
		c.residualRotation = (float)residual.rotation;
		c.residualScale = (float)residual.scale;
		c.residualTranslation = (float)residual.translation;
		if (residual.Agrees()) {
			// the placement is the second opinion an undecided candidate never had
			if (c.cls == SeamCandidate::UNDECIDED) {
				c.cls = SeamCandidate::VERIFIED;
				VERBOSE("Seam (%u, %u) verified by placement", c.sceneA, c.sceneB);
			}
		} else if (c.IsTrusted()) {
			// a seam the graph trusts that the placement cannot satisfy: the model rests on it all
			// the same, so the joint refinement answers for the cycle it closes
			VERBOSE("Seam (%u, %u) loop discrepancy: %.2f deg, %.1f%%, %.2f%%", c.sceneA, c.sceneB,
				residual.rotation, (residual.scale - 1) * 100, residual.translation * 100);
		} else
			return; // undecided and unconfirmed: it carries nothing into the model
		modelSeams.push_back(i);
	});

	// a block of the group the model now holds from two sides has closed a cycle
	std::vector<bool> inGroup(poses.size(), false);
	for (const uint32_t g : group.blocks)
		inGroup[g] = true;
	std::map<uint32_t, std::set<uint32_t>> neighboursOf;
	for (const uint32_t e : modelSeams) {
		const SeamCandidate& c = candidates[e];
		if (inGroup[c.sceneA] != inGroup[c.sceneB])
			neighboursOf[inGroup[c.sceneA] ? c.sceneA : c.sceneB].insert(
				inGroup[c.sceneA] ? c.sceneB : c.sceneA);
	}
	for (const auto& [block, neighbours] : neighboursOf)
		if (neighbours.size() >= 2)
			return true;
	return false;
}
/*----------------------------------------------------------------*/

unsigned GlobalAlignment::PlaceBlocks(
	const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t model,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams) const
{
	TD_TIMER_STARTD();
	modelSeams.clear();
	// the blocks this model may take in: the ones the averaging placed in it, and the ones no
	// trusted seam reached at all -- an undecided seam is still evidence, and the placement is
	// what judges it
	std::vector<uint32_t> eligible;
	unsigned numOwn = 0;
	FOREACH(b, poses)
		if (poses[b].state == BlockPose::UNPLACED && (poses[b].model == model || poses[b].model == NO_ID)) {
			eligible.push_back((uint32_t)b);
			if (poses[b].model == model)
				++numOwn;
		}
	if (numOwn == 0)
		return 0;

	// the model grows from the block the seam graph is most sure of, which has to be one the
	// averaging could place: it is that pose the model frame is set at
	uint32_t seed = NO_ID;
	float seedRobust = 0, seedTrusted = 0;
	for (const uint32_t b : eligible) {
		if (poses[b].model != model)
			continue;
		float robust = 0, trusted = 0;
		for (const SeamCandidate& c : candidates) {
			if (c.sceneA != b && c.sceneB != b)
				continue;
			if (c.cls == SeamCandidate::ROBUST)
				robust += c.weight;
			if (c.IsTrusted())
				trusted += c.weight;
		}
		if (seed == NO_ID || robust > seedRobust || (robust == seedRobust &&
			(trusted > seedTrusted || (trusted == seedTrusted &&
			 subScenes[b].status.nCalibratedImages > subScenes[seed].status.nCalibratedImages)))) {
			seed = b;
			seedRobust = robust;
			seedTrusted = trusted;
		}
	}
	BlockGroup group;
	group.blocks.assign(1, seed);
	group.frames.assign(1, Transform());
	PlacementHypothesis start;
	start.source = PlacementHypothesis::INITIAL;
	start.T = poses[seed].T;
	AdmitGroup(subScenes, candidates, blockExtents, model, group, start, poses, modelSeams);
	unsigned numAdmitted = 1;
	VERBOSE("Model %u seeded with block %u (%u images)",
		model, seed, subScenes[seed].status.nCalibratedImages);

	// then one block at a time, the best supported first; a block the model could not take is
	// deferred and tried again as soon as the model has changed, since what it could not confirm
	// then it may confirm now
	std::vector<uint32_t> deferred;
	bool modelChanged = true;
	while (true) {
		uint32_t next = NO_ID;
		float bestSupport = 0;
		for (const uint32_t b : eligible) {
			if (poses[b].state != BlockPose::UNPLACED)
				continue;
			const float support = PooledSupport(candidates, poses, model, b);
			if (support <= 0)
				continue;
			if (next == NO_ID || support > bestSupport || (support == bestSupport &&
				subScenes[b].status.nCalibratedImages > subScenes[next].status.nCalibratedImages)) {
				next = b;
				bestSupport = support;
			}
		}
		if (next == NO_ID) {
			if (deferred.empty() || !modelChanged)
				break;
			for (const uint32_t b : deferred)
				poses[b].state = BlockPose::UNPLACED;
			deferred.clear();
			modelChanged = false;
			continue;
		}
		group.blocks.assign(1, next);
		group.frames.assign(1, Transform());
		PlacementHypothesis winner;
		String reason;
		if (!PlaceGroup(subScenes, candidates, blockExtents, model, group, poses, winner, reason)) {
			poses[next].state = BlockPose::DEFERRED;
			poses[next].reason = reason;
			deferred.push_back(next);
			DEBUG("Block %u deferred: %s", next, reason.c_str());
			continue;
		}
		AdmitGroup(subScenes, candidates, blockExtents, model, group, winner, poses, modelSeams);
		++numAdmitted;
		modelChanged = true;
		DEBUG("Block %u placed on %u neighbours (%u support, %u loop, %u contradict); "
			"votes %u+/%u- block, %u+/%u- model; %s", next,
			winner.neighbourSupport + winner.neighbourLoop + winner.neighbourContra,
			winner.neighbourSupport, winner.neighbourLoop, winner.neighbourContra,
			winner.score.support[0], winner.score.contra[0],
			winner.score.support[1], winner.score.contra[1], PlacementWord(winner.source));
		// every admitted block fitted again to the seams the model rests on, so a seam's two ends
		// can move apart instead of passing the error on
		RefineBlockPoses(candidates, modelSeams, model, seed, poses);
	}

	VERBOSE("Model %u: %u/%u blocks placed on %u seams (%s)", model, numAdmitted,
		(unsigned)eligible.size(), (unsigned)modelSeams.size(), TD_TIMER_GET_FMT().c_str());
	return numAdmitted;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::MergeTransformedScenes(
	std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	const std::vector<BlockPose>& poses)
{
	// A block still admitted is one the merged model took in: stage 6 left every other block
	// unplaceable, and an unplaced block is merged without poses below, so transforming it would
	// be wasted work -- and its pose, which no placement ever confirmed, would inject garbage.
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
	MergeTracksWithCrossSubScenePairs(unplacedImages);
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


void GlobalAlignment::MergeTracksWithCrossSubScenePairs(const std::vector<bool>& unplacedImages)
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
