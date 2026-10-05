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
#include "SceneCluster.h"
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

// A block the model already holds: admitted, and admitted into this model. Another model placed
// its blocks in a frame this one knows nothing about, so they have no say here.
static bool IsInModel(const std::vector<BlockPose>& poses, uint32_t block, uint32_t model)
{
	return poses[block].state == BlockPose::ADMITTED && poses[block].model == model;
}

// The blocks one model holds, and the block it is gauged at -- the first of them, whose frame the
// model is therefore expressed in. Any of them serves: every averaging of a model is anchored back
// at the pose the model already has at its gauge, so the model keeps its frame whichever is chosen.
static std::vector<uint32_t> ModelBlocks(const std::vector<BlockPose>& poses, const uint32_t model)
{
	std::vector<uint32_t> blocks;
	FOREACH(b, poses)
		if (IsInModel(poses, b, model))
			blocks.push_back((uint32_t)b);
	return blocks;
}

static uint32_t ModelSeed(const std::vector<BlockPose>& poses, const uint32_t model)
{
	FOREACH(b, poses)
		if (IsInModel(poses, b, model))
			return (uint32_t)b;
	return NO_ID;
}

// The images one model carries: what the models are ranked by, the merge being built on the one
// that registers the most of the scene
static unsigned ModelImages(
	const std::vector<Scene>& subScenes, const std::vector<BlockPose>& poses, const uint32_t model)
{
	unsigned numImages = 0;
	FOREACH(b, poses)
		if (IsInModel(poses, b, model))
			numImages += subScenes[b].status.nCalibratedImages;
	return numImages;
}

// The one way a block leaves the merge: its images go to the post-merge resection, and the report
// says what refused it
static void UnplaceBlock(BlockPose& pose, String reason)
{
	pose.state = BlockPose::UNPLACEABLE;
	pose.reason = std::move(reason);
}

bool GlobalAlignment::MergeScenes(
	std::vector<Scene>& subScenes, std::vector<IIndexArr>& localToGlobals, MergeReport& report)
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
	ClassifySeamGraph(blockExtents, candidates);                          // stage 3
	ComputeInitialBlockPoses(candidates, blockExtents, numBlocks, poses); // stage 4
	uint32_t numModels = 0;
	for (const BlockPose& pose : poses)
		if (pose.model != NO_ID)
			numModels = MAXF(numModels, pose.model + 1);
	// stage 5: every model grows on its own, the one holding the most trusted weight first
	modelSeams.resize(numModels);
	for (uint32_t m = 0; m < numModels; ++m)
		PlaceBlocks(subScenes, localToGlobals, candidates, blockExtents, m, poses, modelSeams[m]);

	// stage 6: what every model left over forms models of its own, and every model is placed
	// against the one carrying the most images, which is the only model left holding blocks
	PlaceRemainingBlocks(subScenes, localToGlobals, candidates, blockExtents, poses, modelSeams);
	// the model the scene is merged at: the one carrying the most images, ranked as stage 6 ranked
	// them, and the only one it has left holding blocks
	uint32_t mergedModel = NO_ID;
	unsigned mostImages = 0;
	for (uint32_t m = 0; m < (uint32_t)modelSeams.size(); ++m) {
		if (ModelBlocks(poses, m).empty())
			continue;
		const unsigned numImages = ModelImages(subScenes, poses, m);
		if (mergedModel == NO_ID || numImages > mostImages) {
			mergedModel = m;
			mostImages = numImages;
		}
	}
	// and every block it admitted is judged once more against the model it ended up in; a block it
	// then let go keeps its images out, for the resection to recover one by one, while the parts of
	// what it cut are offered to it one last time
	if (mergedModel != NO_ID && RevalidateBlocks(subScenes, localToGlobals, blockExtents, candidates,
			mergedModel, poses, modelSeams[mergedModel]) > 0)
		PlaceBlocks(subScenes, localToGlobals, candidates, blockExtents, mergedModel, poses, modelSeams[mergedModel]);

	// every block outside the merged model is merged without its poses, for the resection to
	// recover its images against the consensus; a block that was cut has nothing left of its own,
	// its images and its points belonging to its two parts
	FOREACH(b, poses) {
		BlockPose& pose = poses[b];
		if (pose.state == BlockPose::SPLIT)
			continue;
		const bool placed = pose.state == BlockPose::ADMITTED && pose.model == mergedModel;
		if (placed)
			++report.numPlaced;
		else
			// a block no placement ever weighed carries no reason of its own, and is left the only
			// one there is to give it
			UnplaceBlock(pose, pose.reason.empty() ? String("no seam") : pose.reason);
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
	// the models the merge ends with: the one it is built on, every other model's blocks having
	// been taken into it or left to the resection
	report.numModels = mergedModel == NO_ID ? 0 : 1;
	if (mergedModel != NO_ID)
		report.modelSeams = std::move(modelSeams[mergedModel]);

	// the kept correspondences of every seam the merged model rests on: each one already agrees
	// with the placement, so its two ends can join their tracks by construction, not by proximity
	std::vector<SeamCorrespondence> seamInliers;
	for (const uint32_t seamIdx : report.modelSeams) {
		const SeamCandidate& c = candidates[seamIdx];
		const size_t numScored = MINF(c.correspondences.size(), c.score.inlierMask.size());
		for (size_t i = 0; i < numScored; ++i)
			if (c.score.inlierMask[i])
				seamInliers.push_back(c.correspondences[i]);
	}

	// a model whose seams still disagree where the block poses leave them holds a block bent inside
	// its own reconstruction, which no pose of it can answer for: every camera of the placed blocks
	// is then given a similarity of its own, so the merge hands over seams that meet
	if (mergedModel != NO_ID)
		RelaxCameras(subScenes, candidates, report.modelSeams, blockExtents,
			ModelSeed(poses, mergedModel), poses, report);

	// stage 7: the placed blocks moved into the model frame, and every block merged in
	MergeTransformedScenes(subScenes, localToGlobals, poses, seamInliers);

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
// A camera contradicts a transform when its inlier share falls under this ...
constexpr float kVoteContraFraction = 0.1f;
// ... and fewer than this share of its correspondences fall within kLooseSeamFactor times the bar:
// a camera merely imprecise -- the edge of a right block a few pixels off, or dense correspondences
// scattered about the bar -- misses the bar and not the loose one; a camera placed elsewhere
// misses both. On Alameda the cameras this told apart sat at a median 6-12 px off a right
// placement, against 40-2400 px for the cameras of a bent block or a false seam.
constexpr float kVoteContraLooseFraction = 0.5f;
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
// The translation unit of a pair is the smaller block's extent, never under this share of the
// larger's: a block of two cameras a millimetre apart has no footprint of its own to be judged by.
constexpr float kMinExtentShare = 0.2f;
// Times maxReprojError: the threshold a predicted seam collects its inliers at, before it is refined
// over them and judged at the bar itself. A model never predicts a pair exactly, and what it is off
// by is the very thing the seam is there to answer for.
constexpr unsigned kLooseSeamFactor = 3;
// How many of its own block's cameras each camera is held to when the cameras are relaxed: enough
// for the block to keep its shape along every direction its covisibility reaches, few enough that
// the graph stays as sparse as the seams it has to close.
constexpr unsigned kIntraBlockEdgesPerCamera = 3;
// What a seam weighs in an averaging when nothing but its own evidence stands behind it: half of
// what a seam another path confirms is worth.
constexpr float kVerifiedWeightShare = 0.5f;
// Where the robust loss of the camera relaxation turns: every residual there is already divided by
// the bar its own part answers to, so one is a camera edge off by exactly what a seam may be off by.
constexpr double kCameraGraphHuber = 1.0;
// The angle two of its rays must open for a track to be triangulated again from the relaxed
// cameras: read from the very field the reconstruction triangulates at, so the relaxation hands the
// tail the tracks its own filtering would have kept anyway and the two cannot drift apart.
static const float kRelaxedTrackMinAngle = ReconstructionConfig().minAngleThreshold;

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

// One edge of a camera pose graph: what a measurement between two cameras says against what the
// two camera similarities imply. The discrepancy E = M^-1 * S_second^-1 * S_first is the identity
// when the edge is satisfied, and each of its three parts is divided by the bar that part is held
// to, so a residual of one means the edge is off by exactly as much as a seam is allowed to be off
// by, whichever of the three ways it is wrong -- which is what lets one robust loss weigh rotation,
// translation and scale at once. It is read in the first camera's own frame, whose units the
// translation bar is therefore given in.
struct CameraGraphError
{
	double q[4], t[3], logScale; // M^-1: the second camera's frame -> the first's
	double rotationWeight, translationWeight, scaleWeight;

	CameraGraphError(const Transform& M, double weight, double rotationBar, double translationBar, double scaleBar) {
		const BlockParameters inverse(M.Invert());
		for (int i = 0; i < 4; ++i)
			q[i] = inverse.q[i];
		for (int i = 0; i < 3; ++i)
			t[i] = inverse.t[i];
		logScale = inverse.logScale;
		rotationWeight = weight * (180.0 / M_PI) / rotationBar;
		translationWeight = weight / translationBar;
		scaleWeight = weight / scaleBar;
	}

	template <typename T>
	bool operator()(
		const T* const qFirst, const T* const tFirst, const T* const logScaleFirst,
		const T* const qSecond, const T* const tSecond, const T* const logScaleSecond,
		T* residuals) const
	{
		using std::exp;
		// the first camera's frame into the second's, as the two similarities imply it
		const T qInv[4] = {qSecond[0], -qSecond[1], -qSecond[2], -qSecond[3]};
		const T d[3] = {tFirst[0] - tSecond[0], tFirst[1] - tSecond[1], tFirst[2] - tSecond[2]};
		T qImplied[4], tImplied[3];
		ceres::QuaternionProduct(qInv, qFirst, qImplied);
		ceres::QuaternionRotatePoint(qInv, d, tImplied);
		const T invScale = exp(-logScaleSecond[0]);
		for (int i = 0; i < 3; ++i)
			tImplied[i] = invScale * tImplied[i];
		// and the measurement inverted onto it: what is left over is the discrepancy
		const T qMeasured[4] = {T(q[0]), T(q[1]), T(q[2]), T(q[3])};
		T qError[4], tError[3];
		ceres::QuaternionProduct(qMeasured, qImplied, qError);
		ceres::QuaternionRotatePoint(qMeasured, tImplied, tError);
		const T scaleMeasured = exp(T(logScale));
		T angleAxis[3];
		ceres::QuaternionToAngleAxis(qError, angleAxis);
		for (int i = 0; i < 3; ++i) {
			residuals[i] = T(rotationWeight) * angleAxis[i];
			residuals[3+i] = T(translationWeight) * (scaleMeasured * tError[i] + T(t[i]));
		}
		residuals[6] = T(scaleWeight) * (T(logScale) + logScaleFirst[0] - logScaleSecond[0]);
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
	PlacementPool& pool)
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
	}
}

// One cross-block match, as the two directions that collect it both name it: the two images and the
// two features, packed so that telling two matches apart costs one comparison of two words
struct CorrespondenceKey
{
	uint64_t a, b; // (image << 32) | feature, side A and side B
	bool operator==(const CorrespondenceKey& other) const { return a == other.a && b == other.b; }
};
struct CorrespondenceKeyHash
{
	size_t operator()(const CorrespondenceKey& key) const {
		return (size_t)(key.a * 0x9E3779B97F4A7C15ULL ^ (key.b + 0x517CC1B727220A95ULL));
	}
};
static CorrespondenceKey KeyOfCorrespondence(const SeamCorrespondence& corr)
{
	return CorrespondenceKey{
		((uint64_t)corr.imageA << 32) | (uint64_t)corr.featureA,
		((uint64_t)corr.imageB << 32) | (uint64_t)corr.featureB};
}

// A pair the seam graph indicted: it holds opinions and every one of them was rejected, which is
// the graph saying a stronger consistent path contradicts what this pair's cameras saw. Its
// correspondences are then evidence that has already been weighed, exactly like a pair the gates
// refused, and no placement may pick them up again. A pair whose two blocks' cameras split over it
// is not this: it never becomes a candidate at all, and its raw correspondences are what carries a
// folded block to the placement that cuts it.
bool IsPairRejected(const std::vector<SeamCandidate>& candidates, const std::vector<uint32_t>& indices)
{
	for (const uint32_t i : indices)
		if (candidates[i].cls != SeamCandidate::REJECTED)
			return false;
	return !indices.empty();
}
// the same question asked by a caller that does not already hold the pair's candidates
bool IsPairRejected(const std::vector<SeamCandidate>& candidates, const uint32_t a, const uint32_t b)
{
	std::vector<uint32_t> indices;
	FOREACH(i, candidates)
		if (candidates[i].sceneA == a && candidates[i].sceneB == b)
			indices.push_back((uint32_t)i);
	return IsPairRejected(candidates, indices);
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

// Every camera's vote on one transform, the raw material the two vote bars are read from, with the
// share of its correspondences within the loose bar
void LogVotes(const char* label, const SeamScore& score)
{
	for (const CameraVote& vote : score.votes)
		DEBUG_ULTIMATE("%s camera %u: %u/%u inliers, coverage %.2f, %s (%u within %ux)",
			label, vote.image, vote.inliers, vote.correspondences, vote.coverage,
			vote.vote > 0 ? "support" : (vote.vote < 0 ? "contradict" : "abstain"),
			vote.looseInliers, kLooseSeamFactor);
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

// Where one block triangulated what its cameras saw: (local image, feature) -> the position of the
// inlier track holding that observation, what the correspondence collection looks each match up in.
// Only inlier tracks are indexed, an outlier's position being unreliable.
void BuildBlockPointMap(const Scene& block, std::unordered_map<PairIdx, Point3>& pointMap)
{
	size_t numInlierObservations = 0;
	for (const Track& track : block.tracks)
		if (track.IsInlier())
			numInlierObservations += track.GetNumInliers();
	pointMap.clear();
	pointMap.reserve(numInlierObservations);
	for (const Track& track : block.tracks)
		if (track.IsInlier())
			for (const Observation& obs : track)
				pointMap.emplace(PairIdx(obs.imageID, obs.featureID), track.position);
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
	const std::vector<SeamObservation>& observations, Transform& T) const
{
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

	// where every block triangulated what its cameras saw
	blockPointMaps.clear();
	blockPointMaps.resize(subScenes.size());
	FOREACH(blockIdx, subScenes)
		BuildBlockPointMap(subScenes[blockIdx], blockPointMaps[blockIdx]);

	// and which cross pairs join which two blocks; nothing has been refused yet
	refusedSeamPairs.clear();
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

void GlobalAlignment::ExtendSeamEvidence(
	const std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	const uint32_t firstBlock)
{
	ASSERT(subScenes.size() == localToGlobals.size() && firstBlock < subScenes.size());
	blockPointMaps.resize(subScenes.size());
	for (uint32_t b = firstBlock; b < (uint32_t)subScenes.size(); ++b) {
		// the images of a new block answer to it now, and not to the block it was cut from
		FOREACH(localID, localToGlobals[b]) {
			const IIndex globalID = localToGlobals[b][localID];
			if (globalID != NO_ID)
				globalToLocal[globalID] = std::make_pair(b, (IIndex)localID);
		}
		BuildBlockPointMap(subScenes[b], blockPointMaps[b]);
	}
	// every cross pair read again, so a pair that ran to the block a part was cut from now runs to
	// the part holding its images
	MapBlockPairLinks();
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
	unsigned numAgreed = 0, numDecided = 0, numUndecided = 0, numOneDirection = 0, numPoints = 0, numSkipped = 0;
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
			else if (candidates.back().source == SeamCandidate::UNION)
				++numAgreed;
			else
				++numPoints;
		}
	}

	DEBUG("Measured %u seam candidates on %u block pairs (%u agreed, %u decided by votes, %u undecided, "
		"%u one direction, %u from points, %u skipped) (%s)",
		(unsigned)candidates.size(), (unsigned)blockPairLinks.size(),
		numAgreed, numDecided, numUndecided, numOneDirection, numPoints, numSkipped, TD_TIMER_GET_FMT().c_str());
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
	SeamScore& score) const
{
	ASSERT(observations.size() == correspondences.size());
	score = SeamScore();

	// what the transform explains; every error is kept for the votes below to read how far off
	// each camera's correspondences sit, beyond whether they pass the bar
	const Transform TInv(T.Invert());
	score.inlierMask.assign(observations.size(), false);
	std::vector<REAL> errors(observations.size());
	FOREACH(i, observations) {
		errors[i] = SeamObservationError(observations[i], T, TInv);
		if (errors[i] <= config.maxReprojError) {
			score.inlierMask[i] = true;
			++score.inliers;
		}
	}
	const REAL looseBar = (REAL)kLooseSeamFactor * config.maxReprojError;

	// every image the seam touches, in both its roles: the camera whose feature carries the point
	// witnesses the seam as much as the camera that observed it. A match whose two endpoints both
	// lie on a track is collected in both directions, and one camera then meets that same match
	// once in each role -- it is one correspondence either way and counts once, or the bars a vote
	// answers to are halved on exactly the seams whose tracks are best formed. The two collections
	// of one match name the same two images, so keeping the first of them is what lets both of those
	// cameras count it once, and one pass over the union does for every camera at once
	std::unordered_set<CorrespondenceKey, CorrespondenceKeyHash> counted;
	counted.reserve(observations.size());
	std::map<IIndex, std::vector<uint32_t>> imageObservations;
	FOREACH(i, observations) {
		if (!counted.insert(KeyOfCorrespondence(correspondences[i])).second)
			continue;
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
			if (errors[i] <= looseBar)
				++vote.looseInliers;
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
		const float looseFraction = (float)vote.looseInliers / (float)vote.correspondences;
		if (vote.inliers >= config.minVoteInliers && inlierFraction >= kVoteSupportFraction &&
			vote.coverage >= config.minVoteCoverage)
			vote.vote = 1;
		else if (vote.correspondences >= config.minVoteInliers && inlierFraction < kVoteContraFraction &&
			looseFraction < kVoteContraLooseFraction)
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
	return failed;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::ScoreCandidate(const std::vector<Scene>& subScenes, SeamCandidate& c) const
{
	const uint32_t blockA = c.sceneA;
	ScoreSeam(subScenes, c.observations, c.correspondences, c.T,
		[this, blockA](IIndex image) { return globalToLocal.at(image).first == blockA ? 0 : 1; },
		c.score);
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
	// -- unless the graph rejected every opinion the pair holds, which is a verdict on its evidence
	std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> pairCandidates;
	FOREACH(i, candidates) {
		const SeamCandidate& c = candidates[i];
		if (JoinsGroupToModel(c.sceneA, c.sceneB))
			pairCandidates[std::make_pair(c.sceneA, c.sceneB)].push_back(i);
	}
	for (const auto& [blockPair, indices] : pairCandidates) {
		if (IsPairRejected(candidates, indices))
			continue;
		// the parallel candidates of a pair were all scored on the same union of both directions,
		// so that union enters the pool once and every one of them is recorded behind it
		for (const uint32_t i : indices)
			pool.candidateIdx.push_back(i);
		const SeamCandidate& c = candidates[indices.front()];
		AppendPoolObservations(c.observations, c.correspondences,
			blockPair.first, blockPair.second, frameOf, inGroup, pool);
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
			blockPair.first, blockPair.second, frameOf, inGroup, pool);
	}

	// the model's cameras in the model frame: the footprint two placements of the group are told
	// apart in
	FOREACH(b, poses) {
		if (!inModel[b])
			continue;
		for (const Image& img : subScenes[b].images)
			if (img.IsValid())
				pool.modelCentres.emplace_back(frameOf[b] * img.C);
	}
	DEBUG_ULTIMATE("Placement pool of %u blocks: %u observations from %u candidates against %u cameras",
		(unsigned)group.blocks.size(), (unsigned)pool.observations.size(), (unsigned)pool.candidateIdx.size(),
		(unsigned)pool.modelCentres.size());
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
		h.score);
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
			RefineSeamTransform(SelectInliers(observations, estimatorMask, d == 0 ? 1 : 0), Tseam[d]);
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
	String failedGates[2];
	for (int d = 0; d < 2; ++d) {
		if (!measured[d])
			continue;
		const int other = 1 - d;
		const int otherForward = other == 0 ? 1 : 0;
		const unsigned bestOther =
			measured[other] && candidate[other].NumObservations(otherForward) >= config.minCommonTracks ?
			candidate[other].NumInliers(otherForward) : 0;
		failedGates[d] = FailedGates(candidate[d].score,
			candidate[d].NumInliers(otherForward), bestOther, config.minCameraVoteRatio);
		LogVotes(String::FormatString("Seam (%u, %u) %s", a, b, SourceWord(candidate[d].source)).c_str(),
			candidate[d].score);
		if (failedGates[d].empty())
			passed[d] = true;
		else
			DEBUG_ULTIMATE("Seam (%u, %u) %s dropped by %s", a, b, SourceWord(candidate[d].source),
				failedGates[d].c_str());
	}
	if (!passed[0] && !passed[1]) {
		// a pair every direction refused on the votes alone, and only because one of its two blocks
		// contradicts itself -- as many of its own cameras behind the seam as against it -- has not
		// been weighed and found wanting: no one similarity can carry a block that is two blocks,
		// which is a thing the seam of a pair cannot say and a placement can. Its correspondences
		// stay the evidence they are. Every measured direction has to say that and nothing else: a
		// direction the gates threw out for a reason of its own is one whose evidence they weighed,
		// and the pair carries it too.
		bool blockSplit = true;
		for (int d = 0; d < 2; ++d) {
			if (!measured[d])
				continue;
			bool sideSplit = false;
			for (int s = 0; s < 2; ++s)
				sideSplit = sideSplit ||
					(candidate[d].score.support[s] >= config.minSupportingCentres &&
					 candidate[d].score.contra[s] >= config.minSupportingCentres);
			blockSplit = blockSplit && sideSplit && failedGates[d] == "camera votes";
		}
		if (blockSplit) {
			DEBUG("Seam (%u, %u) skipped: the cameras of one of its two blocks are split over it", a, b);
			return;
		}
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
			RefineSeamTransform(SelectInliers(c.observations, c.score.inlierMask), c.T);
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
		RefineSeamTransform(SelectInliers(c.observations, c.score.inlierMask, best == 0 ? 1 : 0), c.T);
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
	RefineSeamTransform(SelectInliers(c.observations, c.score.inlierMask, d == 0 ? 1 : 0), c.T);
	if (!c.scaleObservable && measured[1 - d] && candidate[1 - d].scaleObservable) {
		// its own rig is too shallow to observe a scale and the other direction, whose rig is deep
		// enough to have one, measured it: take that, and leave the rig where the seam already put
		// it. Two shallow rigs facing each other observe no scale between them at all, and a seam
		// that says otherwise carries a number nothing measured into the scale averaging
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
	REAL translation; // fraction of the two frames' shared camera footprint (see PairExtent)
	REAL excess;      // largest of the three as a factor of its limit; conflicting above 1
	// the two say the same thing: every component of the discrepancy stays under its own bar
	bool Agrees() const { return excess <= REAL(1); }
};

// The one unit a translation between two frames is read in, wherever two camera footprints meet:
// the smaller of the two, since that is the block that can least afford the error, floored at a
// share of the larger so that a frame with no extent of its own is still judged by something.
static REAL PairExtent(const REAL first, const REAL second)
{
	return MAXF(MINF(first, second), (REAL)kMinExtentShare * MAXF(first, second));
}

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
// against the footprint the two blocks share there, so the verdict does not depend on which end
// happens to have the lower block index.
static SeamResidual ComputeSeamResidual(
	const uint32_t a, const uint32_t b, const Transform& T, const std::vector<Transform>& transforms,
	const std::vector<REAL>& blockExtents, const GlobalAlignmentConfig& config)
{
	const REAL diag = PairExtent(blockExtents[a] * transforms[a].scale, blockExtents[b] * transforms[b].scale);
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
	return c.cls == SeamCandidate::VERIFIED ? c.weight * kVerifiedWeightShare : c.weight;
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

// What a seam nothing corroborates is worth on its own evidence: verified when the two directions
// agreed on it, when the points of both blocks measured it, or when the one direction it has stands
// alone; undecided otherwise. The one rule for a seam no second path can speak for, whether the
// graph is reading it or a block cut in two has just measured it
static SeamCandidate::Class ClassifyUncorroboratedSeam(
	const SeamCandidate& c, const GlobalAlignmentConfig& config)
{
	return c.source == SeamCandidate::UNION || c.source == SeamCandidate::POINTS ||
		DoesSeamStandAlone(c, config) ? SeamCandidate::VERIFIED : SeamCandidate::UNDECIDED;
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
		const SeamCandidate::Class alone = ClassifyUncorroboratedSeam(c, config);
		if (alone == SeamCandidate::UNDECIDED)
			reason = "bridge, one direction";
		return alone;
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

// What one seam is off by against the solved rotations, in degrees: a seam whose two ends came out
// in different frames was solved in two gauges and cannot be compared at all, which is as wrong as
// a seam gets
static REAL RotationResidual(
	const RotationPair& p, const std::vector<Point3>& rotations, const std::vector<uint32_t>& frameOfNode)
{
	if (frameOfNode[p.idxA] == NO_ID || frameOfNode[p.idxA] != frameOfNode[p.idxB])
		return REAL(FLT_MAX);
	const RMatrix RA(rotations[p.idxA]), RB(rotations[p.idxB]);
	return R2D(ACOS(ComputeAngle(p.relativeRotation, Matrix3x3(RB * RA.t()))));
}

// What the seams a solution leaves unexplained weigh between them, which is how two readings of the
// same component are told apart: weight, because everything else the graph decides it decides by
// weight, and a reading that explains three thin seams does not beat one that explains two heavy
// ones
static float InconsistentRotationWeight(
	const std::vector<RotationPair>& pairs, const std::vector<Point3>& rotations,
	const std::vector<uint32_t>& frameOfNode, const REAL maxResidual)
{
	float weight = 0;
	for (const RotationPair& p : pairs)
		if (RotationResidual(p, rotations, frameOfNode) > maxResidual)
			weight += p.weight;
	return weight;
}

// The global rotation of every node the seams reach, and which run of the estimator placed it.
// The estimator solves the largest sub-component it can and leaves the rest at INF, so it is run
// again over the seams that stayed inside the nodes it left out, until no run places anything more.
// Two nodes of different frames are each solved in their own gauge and cannot be compared.
//
// The initialization carries the rotations out along the heaviest seams, so one wrong seam heavy
// enough to enter that tree turns everything the tree reaches through it, and the true seams
// bridging the two halves are then the ones that look wrong -- the largest residuals sit on them,
// not on the seam that caused it, so no reading of the solution finds the culprit. Every seam the
// tree is built through is therefore tried left out of it, and the reading whose unexplained seams
// weigh least is the one kept, ties going to the first.
// @param sweepTree try every seam of the tree (the first reading of a component), or only the one
// `treeLeftOut` carries in (every reading after it): the first sweep costs at most one extra solve
// per seam of the tree, each one after it at most one, and a component its seams already explain
// costs none at all
// @param treeLeftOut in: the seam the previous reading of this component kept out of its tree, or
// NO_ID; out: the seam this reading kept out
static bool SolveRotationFrames(
	const std::vector<RotationPair>& pairs, const uint32_t numNodes,
	const GlobalRotationEstimatorOptions& options, const REAL maxResidual, const bool sweepTree,
	std::vector<Point3>& rotations, std::vector<uint32_t>& frameOfNode,
	std::pair<uint32_t, uint32_t>& treeLeftOut)
{
	// one reading of the component: the estimator run over the seams it can reach, again and again
	// over what it left behind. `leftOut` names two blocks the initialization tree may not run
	// between; every seam takes part in the solve itself whatever the tree did with it
	const auto Solve = [&](const std::pair<uint32_t, uint32_t>& leftOut,
		std::vector<Point3>& solved, std::vector<uint32_t>& frames) {
		solved.assign(numNodes, Point3::INF);
		frames.assign(numNodes, NO_ID);
		std::vector<RotationPair> remaining(pairs);
		uint32_t numFrames = 0;
		while (!remaining.empty()) {
			GlobalRotationEstimator estimator(options);
			std::vector<Point3> partial;
			// an empty initialization is the estimator's own tree, which is what a reading that
			// keeps no seam out asks for
			if (leftOut.first != NO_ID &&
				GlobalRotationEstimator::RotationsFromMaximumSpanningTree(
					numNodes, remaining, leftOut, partial) == NO_ID)
				break;
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
		return numFrames;
	};

	const std::pair<uint32_t, uint32_t> previousLeftOut(treeLeftOut);
	treeLeftOut = std::make_pair(NO_ID, NO_ID);
	std::vector<Point3> bestRotations;
	std::vector<uint32_t> bestFrames;
	if (Solve(treeLeftOut, bestRotations, bestFrames) == 0)
		return false;
	float bestWeight = InconsistentRotationWeight(pairs, bestRotations, bestFrames, maxResidual);
	if (bestWeight > 0) {
		// the seams to try left out of the tree: every tree seam of the component the first time it
		// is read, and the one that won then every time after, the weights having moved and not the
		// tree
		std::vector<std::pair<uint32_t, uint32_t>> leftOuts;
		if (sweepTree) {
			std::vector<Point3> ignored;
			GlobalRotationEstimator::RotationsFromMaximumSpanningTree(
				numNodes, pairs, std::make_pair((uint32_t)NO_ID, (uint32_t)NO_ID), ignored, &leftOuts);
		} else if (previousLeftOut.first != NO_ID)
			leftOuts.push_back(previousLeftOut);
		for (const std::pair<uint32_t, uint32_t>& leftOut : leftOuts) {
			std::vector<Point3> retryRotations;
			std::vector<uint32_t> retryFrames;
			if (Solve(leftOut, retryRotations, retryFrames) == 0)
				continue;
			const float weight =
				InconsistentRotationWeight(pairs, retryRotations, retryFrames, maxResidual);
			if (weight >= bestWeight)
				continue;
			bestWeight = weight;
			bestRotations.swap(retryRotations);
			bestFrames.swap(retryFrames);
			treeLeftOut = leftOut;
			if (!(bestWeight > 0))
				break; // nothing left for another tree to explain
		}
	}
	rotations.swap(bestRotations);
	frameOfNode.swap(bestFrames);
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
	// one connected component: the tree the rotations are carried out along reaches only the
	// component it is rooted in, so seams outside it would take no part in the initialization nor
	// in the sweep that keeps one of its seams out
	ASSERT([&]() {
		DisjointSet<uint32_t> component(n);
		for (const uint32_t e : edges)
			component.Union(nodeOfBlock[candidates[e].sceneA], nodeOfBlock[candidates[e].sceneB]);
		for (uint32_t node = 1; node < n; ++node)
			if (component.Find(node) != component.Find(0))
				return false;
		return true;
	}(), "the averaged seams span more than one connected component");

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
	std::pair<uint32_t, uint32_t> treeLeftOut(NO_ID, NO_ID);
	unsigned rotationRound = 0;
	if (!RobustAverage(rotationPairs, (REAL)config.maxGraphRotationResidual, kRobustRounds,
		[&](const std::vector<RotationPair>& active, std::vector<Point3>& rots) {
			// only the first reading of the component sweeps the whole tree; every reweighted one
			// after it tries the plain tree and whichever seam the sweep found worth keeping out
			return SolveRotationFrames(active, n, rotationOptions,
				(REAL)config.maxGraphRotationResidual, rotationRound++ == 0,
				rots, frameOfNode, treeLeftOut);
		},
		[&](size_t i, const std::vector<Point3>& rots) {
			return RotationResidual(rotationPairs[i], rots, frameOfNode);
		},
		rotations, rotationResiduals))
	{
		VERBOSE("error: rotation averaging over %u seams failed", (unsigned)edges.size());
		return false;
	}
	// a component that comes back in more than one frame is one the rotations could not carry
	// across: its seams are then judged apart and the placement has nothing joining the two halves
	uint32_t numFrames = 0;
	for (const uint32_t frame : frameOfNode)
		if (frame != NO_ID)
			numFrames = MAXF(numFrames, frame + 1);
	DEBUG("Seam graph: component of %u blocks solved in %u frames", n, numFrames);
	if (treeLeftOut.first != NO_ID)
		DEBUG("Seam graph: the rotations were carried out without the seam (%u, %u) in the tree",
			blockOfNode[treeLeftOut.first], blockOfNode[treeLeftOut.second]);
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
					// the footprint the two blocks share, as everything that judges a seam's
					// translation reads it
					const REAL extent = PairExtent(blockExtents[blockOfNode[p.idxA]] * scales[p.idxA],
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

unsigned GlobalAlignment::AverageTrustedComponents(
	const std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const std::vector<bool>& eligible,
	const uint32_t firstModel,
	std::vector<BlockPose>& poses) const
{
	const uint32_t numBlocks = (uint32_t)poses.size();
	ASSERT(eligible.size() == numBlocks);
	std::vector<uint32_t> trustedEdges;
	FOREACH(i, candidates) {
		const SeamCandidate& c = candidates[i];
		if (c.IsTrusted() && eligible[c.sceneA] && eligible[c.sceneB])
			trustedEdges.push_back((uint32_t)i);
	}
	std::vector<SeamComponent> components;
	GroupSeamComponents(candidates, trustedEdges, numBlocks, components);

	std::vector<BlockPose> componentPoses;
	std::vector<Point3> residuals;
	FOREACH(k, components) {
		const SeamComponent& component = components[k];
		if (!AverageBlockPoses(candidates, component.edges, blockExtents, numBlocks, component.gauge, componentPoses, residuals)) {
			VERBOSE("warning: the %u blocks joined by %u trusted seams cannot be placed together",
				component.numBlocks, (unsigned)component.edges.size());
			continue;
		}
		for (uint32_t block = 0; block < numBlocks; ++block) {
			if (componentPoses[block].model == NO_ID)
				continue;
			poses[block].T = componentPoses[block].T;
			poses[block].model = firstModel + (uint32_t)k;
		}
	}
	return (unsigned)components.size();
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::ComputeInitialBlockPoses(
	const std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t numBlocks,
	std::vector<BlockPose>& poses) const
{
	TD_TIMER_STARTD();
	poses.assign(numBlocks, BlockPose());
	AverageTrustedComponents(candidates, blockExtents, std::vector<bool>(numBlocks, true), 0, poses);
	unsigned numPlaced = 0, numModels = 0;
	for (const BlockPose& pose : poses)
		if (pose.model != NO_ID) {
			++numPlaced;
			numModels = MAXF(numModels, pose.model + 1);
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

// Whether the admitted neighbours are behind a placement: one of them at least, and no more than a
// share of those with something to say against it
static bool NeighboursBehind(const unsigned support, const unsigned loop, const unsigned contra)
{
	return support > 0 && contra * kNeighbourContraShare <= support + loop + contra;
}

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

// How much of a pool each admitted block carries: the bar a neighbour clears before its opinion of
// a placement is asked for
static void CountPooledPerBlock(
	const PlacementPool& pool,
	const std::unordered_map<IIndex, std::pair<uint32_t, IIndex>>& globalToLocal,
	std::map<uint32_t, unsigned>& numPooled)
{
	numPooled.clear();
	for (const SeamObservation& obs : pool.observations) {
		// the model side holds the camera when the point is the group's, and the point otherwise
		const auto it = globalToLocal.find(obs.forward ? obs.rigImage : obs.pointImage);
		if (it != globalToLocal.end())
			++numPooled[it->second.first];
	}
}

// The camera-vote margin a placement into this model is held to: a model the group reaches over
// verified seams alone is the stricter one
static float ModelVoteRatio(
	const std::vector<SeamCandidate>& candidates, const std::vector<BlockPose>& poses,
	const uint32_t model, const BlockGroup& group, const GlobalAlignmentConfig& config)
{
	bool anyRobust = false;
	ForEachGroupSeam(candidates, poses, model, group, [&](uint32_t i, uint32_t) {
		anyRobust = anyRobust || candidates[i].cls == SeamCandidate::ROBUST;
	});
	return anyRobust ? config.minCameraVoteRatio : config.minCameraVoteRatioVerified;
}

// What a block weighs against the blocks the model already holds: its trusted seams to them in
// full and its undecided ones at a fraction, so a block only undecided evidence reaches is still
// tried, after every block a trusted seam carries
static float PooledSupport(
	const std::vector<SeamCandidate>& candidates, const std::vector<BlockPose>& poses,
	const std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>>& blockPairLinks,
	const std::set<std::pair<uint32_t, uint32_t>>& refusedSeamPairs,
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
	if (support > 0)
		return support;
	// a block no seam of the model reaches may still share correspondences with it: a pair nothing
	// could be measured on, or one refused because the block itself is folded. The pool reads those
	// like any other evidence, so the block is tried on them -- at the floor weight a seam no camera
	// could vote on carries, discounted like everything else the graph has not confirmed. A pair the
	// gates refused and one the graph rejected outright are not among them: the pool does not read
	// those either, so a block they are all that reaches has nothing to be tried on
	for (const auto& [blockPair, links] : blockPairLinks)
		if (refusedSeamPairs.count(blockPair) == 0 &&
			((blockPair.first == block && IsInModel(poses, blockPair.second, model)) ||
			 (blockPair.second == block && IsInModel(poses, blockPair.first, model))) &&
			!IsPairRejected(candidates, blockPair.first, blockPair.second))
			return kUndecidedSupportWeight;
	return 0;
}

// The 3D-3D similarity of a group against the blocks the model holds: the tracks both sides
// triangulated, paired through the cross correspondence they share -- the point mode's own
// estimate, read off the pool instead of off one block pair. Returns 0 when the evidence does not
// carry an estimate.
static unsigned EstimatePoolSimilarity(
	const PlacementPool& pool, const GlobalAlignmentConfig& config, Transform& T)
{
	// a correspondence collected in both directions has its point triangulated in both frames, and
	// the two collections of one match are named the one way every count of them is named
	std::unordered_map<CorrespondenceKey, Point3, CorrespondenceKeyHash> groupPoints;
	FOREACH(i, pool.observations)
		if (pool.observations[i].forward)
			groupPoints.emplace(KeyOfCorrespondence(pool.correspondences[i]), pool.observations[i].X);
	Point3Arr srcPoints, dstPoints;
	FOREACH(i, pool.observations) {
		if (pool.observations[i].forward)
			continue;
		const auto it = groupPoints.find(KeyOfCorrespondence(pool.correspondences[i]));
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

// The seams a model rested on through a block it let go: they carry nothing now
static void DropBlockSeams(
	const std::vector<SeamCandidate>& candidates, const uint32_t block, std::vector<uint32_t>& modelSeams)
{
	modelSeams.erase(std::remove_if(modelSeams.begin(), modelSeams.end(),
		[&candidates, block](const uint32_t e) {
			return candidates[e].sceneA == block || candidates[e].sceneB == block;
		}), modelSeams.end());
}

// The parts of a block that came apart take its pose, its model and its place in the queue; the
// block is left behind, its images and its points now carried by them
static void QueueSplitParts(
	const uint32_t block, const std::pair<uint32_t, uint32_t>& parts, std::vector<BlockPose>& poses)
{
	poses.resize(MAXF((size_t)parts.second + 1, poses.size()));
	for (const uint32_t part : {parts.first, parts.second}) {
		poses[part].T = poses[block].T;
		poses[part].model = poses[block].model;
		poses[part].state = BlockPose::UNPLACED;
	}
	poses[block].state = BlockPose::SPLIT;
}

// A group whose own cameras split in two over its best placement: enough of them behind it and
// enough against it that no pose of it can answer to both, which is what a block reconstructed from
// two halves that never saw each other looks like
static bool VotesSplit(const PlacementHypothesis& best, const GlobalAlignmentConfig& config)
{
	return !best.Passed() && best.failedGate.find("camera votes") != String::npos &&
		best.score.support[0] >= config.minSupportingCentres &&
		best.score.contra[0] >= config.minSupportingCentres;
}

// Whether the given images hold together on the given pairs alone, counting only the images that
// have something to hold on to: an image the block never registered, and one its own weighting left
// without a single pair, reaches nothing and would refuse every fold it happens to sit in
static bool IsSideConnected(
	const std::vector<std::vector<std::pair<IIndex, float>>>& neighbours,
	const std::vector<int>& side, const std::vector<bool>& holds, const IIndexArr& images)
{
	IIndexArr counted;
	for (const IIndex image : images)
		if (holds[image])
			counted.push_back(image);
	if (counted.empty())
		return false;
	std::set<IIndex> reached{counted.front()};
	std::vector<IIndex> front{counted.front()};
	while (!front.empty()) {
		const IIndex image = front.back();
		front.pop_back();
		for (const auto& [other, weight] : neighbours[image])
			if (holds[other] && side[other] == side[image] && reached.insert(other).second)
				front.push_back(other);
	}
	return reached.size() == counted.size();
}
/*----------------------------------------------------------------*/

static Transform CameraToBlock(const Image& image);

PairAgreement GlobalAlignment::MeasurePlacementPairs(
	const std::vector<Scene>& subScenes,
	const uint32_t model,
	const BlockGroup& group,
	const Transform& T,
	const std::vector<BlockPose>& poses) const
{
	PairAgreement agreement;
	if (config.maxPairRotationResidual <= 0.f)
		return agreement;
	// where every block stands in the model frame: the group's where T puts them, the admitted
	// ones where the model holds them
	std::unordered_map<uint32_t, Transform> blockToModel;
	FOREACH(k, group.blocks)
		blockToModel[group.blocks[k]] = T * group.frames[k];
	FOREACH(b, poses)
		if (poses[b].state == BlockPose::ADMITTED && poses[b].model == model && !blockToModel.count((uint32_t)b))
			blockToModel[(uint32_t)b] = poses[b].T;
	const auto InModel = [&subScenes, &blockToModel](const std::pair<uint32_t, IIndex>& local) {
		const Image& image = subScenes[local.first].images[local.second];
		const Transform cameraToModel(blockToModel.at(local.first) * CameraToBlock(image));
		Pose3D pose;
		pose.R = RMatrix(cameraToModel.R.t());
		pose.C = cameraToModel.t;
		return pose;
	};
	// every admitted neighbour's pairs to the group, weighed for and against: a pair inside the
	// group says nothing about its placement
	for (const auto& [b, F] : blockToModel) {
		if (std::find(group.blocks.begin(), group.blocks.end(), b) != group.blocks.end())
			continue;
		float agree = 0.f, disagree = 0.f;
		unsigned numPairs = 0;
		for (const uint32_t g : group.blocks) {
			const auto it = blockPairLinks.find(std::make_pair(MINF(g, b), MAXF(g, b)));
			if (it == blockPairLinks.end())
				continue;
			for (const uint32_t idx : it->second) {
				const ImagePair& pair = scene.pairs[idx];
				if (!IsPoseLinkPair(pair) || pair.GetNumWeightedInliers() < config.minVoteInliers)
					continue;
				const std::pair<uint32_t, IIndex>& local1 = globalToLocal.at(pair.ID1);
				const std::pair<uint32_t, IIndex>& local2 = globalToLocal.at(pair.ID2);
				if (!subScenes[local1.first].images[local1.second].IsValid() ||
					!subScenes[local2.first].images[local2.second].IsValid())
					continue;
				// the group's image is the one judged, the model's its neighbour
				const bool firstInGroup = local1.first == g;
				const PairDisagreement d = MeasurePairDisagreement(pair,
					InModel(firstInGroup ? local1 : local2), InModel(firstInGroup ? local2 : local1),
					firstInGroup ? pair.ID2 : pair.ID1);
				++numPairs;
				(d.Within(config.maxPairRotationResidual) ? agree : disagree) += (float)pair.GetNumWeightedInliers();
			}
		}
		if (numPairs == 0)
			continue;
		agreement.numPairs += numPairs;
		agreement.agree += agree;
		agreement.disagree += disagree;
		++agreement.numNeighbours;
		if (disagree > agree)
			++agreement.numContra;
	}
	return agreement;
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
		// nothing to hand back but the refusal itself, which the caller reads off the winner like
		// any other: a hypothesis that was never formed has failed every gate there is
		reason = "no hypothesis";
		winner = PlacementHypothesis();
		winner.failedGate = reason;
		return false;
	}

	// how much of the pool each admitted block carries, and the margin the cameras are held to
	std::map<uint32_t, unsigned> numPooled;
	CountPooledPerBlock(pool, globalToLocal, numPooled);
	const float voteRatio = ModelVoteRatio(candidates, poses, model, group, config);

	const uint32_t firstBlock = group.blocks.front();
	for (PlacementHypothesis& h : hypotheses) {
		ScoreHypothesis(subScenes, pool, bestOwnInliers, voteRatio, h);
		const NeighbourVerdict verdict = CheckNeighbours(
			candidates, blockExtents, poses, model, group, numPooled, h.T, config);
		h.neighbourSupport = verdict.support;
		h.neighbourLoop = verdict.loop;
		h.neighbourContra = verdict.contra;
		// the neighbours the model already holds have to be behind it
		if (h.Passed() && !NeighboursBehind(verdict.support, verdict.loop, verdict.contra))
			h.failedGate = "neighbours";
		// and so have the verified image pairs across it, neighbour by neighbour
		const PairAgreement pairs = MeasurePlacementPairs(subScenes, model, group, h.T, poses);
		if (h.Passed() && !pairs.Holds())
			h.failedGate = "pair agreement";
		LogVotes(String::FormatString("Placement of block %u, %s",
			firstBlock, PlacementWord(h.source)).c_str(), h.score);
		DEBUG_ULTIMATE("Placement of block %u, %s: %u inliers of %u, %u+/%u- neighbours, %u pairs across weighing %.0f for / %.0f against, %u of %u neighbours contradicting%s%s",
			firstBlock, PlacementWord(h.source), h.score.inliers, (unsigned)pool.observations.size(),
			verdict.support, verdict.contra, pairs.numPairs, pairs.agree, pairs.disagree,
			pairs.numContra, pairs.numNeighbours,
			h.Passed() ? "" : ", dropped by ", h.failedGate.c_str());
	}

	// the largest vote of the cameras decides between them, and where two draw, the order they were
	// formed in: the group read against the model's points first, the model against the group's
	// next, the pose the averaging already had last -- a measurement of this pair before one of the
	// whole graph, and the same winner every run
	const auto Votes = [](const PlacementHypothesis& h) { return h.score.support[0] + h.score.support[1]; };
	std::stable_sort(hypotheses.begin(), hypotheses.end(),
		[&Votes](const PlacementHypothesis& a, const PlacementHypothesis& b) { return Votes(a) > Votes(b); });
	const auto itWinner = std::find_if(hypotheses.begin(), hypotheses.end(),
		[](const PlacementHypothesis& h) { return h.Passed(); });
	if (itWinner == hypotheses.end()) {
		// nothing carried the group: the best of them says why, and among them one its admitted
		// neighbours are behind says it best -- that is the reading the group's own seams support,
		// and the only one a cycle can still be closed through
		winner = hypotheses.front();
		for (const PlacementHypothesis& h : hypotheses)
			if (NeighboursBehind(h.neighbourSupport, h.neighbourLoop, h.neighbourContra)) {
				winner = h;
				break;
			}
		reason = winner.failedGate;
		return false;
	}
	winner = *itWinner;
	// the unit two placements are held apart in: the footprint the group and the model share, read
	// in the group frame their discrepancy lives in
	const REAL extent = PairExtent(GroupExtent(group, blockExtents),
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

// The worst the given seams are off, as the consensus last read them: their largest rotation
// discrepancy, in degrees
static float LargestSeamResidual(
	const std::vector<SeamCandidate>& candidates, const std::vector<uint32_t>& modelSeams)
{
	float largest = 0;
	for (const uint32_t e : modelSeams)
		largest = MAXF(largest, candidates[e].residualRotation);
	return largest;
}

// The pairs of a model that carry correspondences but no seam it rests on: both blocks admitted,
// cross links between them, and nothing but undecided candidates on them. A pair the graph settled
// already has its seam, and one it rejected has been weighed and contradicted, so neither is asked
// again; a pair the gates refused is, since what is asked of it here is not what it can measure on
// its own but what it makes of where the model puts its two blocks.
static void CollectWeakPairs(
	const std::vector<SeamCandidate>& candidates,
	const std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>>& blockPairLinks,
	const std::vector<BlockPose>& poses, const uint32_t model,
	std::vector<std::pair<uint32_t, uint32_t>>& pairs)
{
	std::set<std::pair<uint32_t, uint32_t>> settled;
	for (const SeamCandidate& c : candidates)
		if (c.cls != SeamCandidate::UNDECIDED)
			settled.emplace(c.sceneA, c.sceneB);
	pairs.clear();
	for (const auto& [blockPair, links] : blockPairLinks)
		if (IsInModel(poses, blockPair.first, model) && IsInModel(poses, blockPair.second, model) &&
			settled.count(blockPair) == 0)
			pairs.push_back(blockPair);
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::AverageModelPoses(
	const std::vector<SeamCandidate>& candidates,
	const std::vector<uint32_t>& seams,
	const std::vector<REAL>& blockExtents,
	const uint32_t model, const uint32_t seed,
	std::vector<BlockPose>& poses,
	std::vector<Point3>& residuals) const
{
	std::vector<BlockPose> averaged;
	if (!AverageBlockPoses(candidates, seams, blockExtents,
		(uint32_t)poses.size(), seed, averaged, residuals) || averaged[seed].model == NO_ID)
		return false;
	// the consensus comes out in a frame of its own, and is anchored back where the model already
	// stands: a block the consensus could not reach then stays in the same frame as the blocks it
	// moved
	const Transform frame(poses[seed].T * averaged[seed].T.Invert());
	FOREACH(b, poses)
		if (IsInModel(poses, b, model) && averaged[b].model != NO_ID)
			poses[b].T = frame * averaged[b].T;
	return true;
}
/*----------------------------------------------------------------*/

std::pair<float, float> GlobalAlignment::RelaxBlockPoses(
	const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t model, const uint32_t seed,
	const std::vector<uint32_t>& modelSeams,
	std::vector<BlockPose>& poses) const
{
	ASSERT(poses.size() == subScenes.size());
	const float before = LargestSeamResidual(candidates, modelSeams);

	// the whole model at once, over the seams it rests on and about the block it is gauged at
	std::vector<Point3> residuals;
	if (AverageModelPoses(candidates, modelSeams, blockExtents, model, seed, poses, residuals)) {
		// what the consensus makes of every seam where it has just put the blocks; one it could not
		// reach keeps the verdict it had
		FOREACH(k, modelSeams) {
			if (residuals[k].x >= REAL(FLT_MAX))
				continue;
			SeamCandidate& c = candidates[modelSeams[k]];
			c.residualRotation = (float)residuals[k].x;
			c.residualScale = (float)residuals[k].y;
			c.residualTranslation = (float)residuals[k].z;
		}
	}
	// and the observations behind those seams read again from the poses that came out
	RefineBlockPoses(candidates, modelSeams, model, seed, poses);
	return std::make_pair(before, LargestSeamResidual(candidates, modelSeams));
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::CloseCycleThrough(
	const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t model, const uint32_t seed,
	const BlockGroup& group,
	PlacementHypothesis& best,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams,
	bool& closedCycle,
	String& reason) const
{
	closedCycle = false;
	// only a group refused by what it explains or by the votes alone, with nothing among the
	// admitted blocks against it, can be facing a cycle instead of failing on its own. A neighbour
	// that agrees is not asked for: a trusted seam the model's drift has pushed past the bars is a
	// loop discrepancy, which is the very thing the averaging spreads, and a group small enough
	// carries every one of its seams there at once. A neighbour whose own seam is not trusted and
	// disagrees all the same is another matter: nothing says the model drifted rather than the
	// group being wrong.
	if (best.Passed() || best.neighbourContra > 0)
		return false;
	// and only a group two or more admitted blocks trust: a single seam closes nothing
	std::vector<uint32_t> tentativeSeams(modelSeams);
	std::set<uint32_t> trustedNeighbours;
	ForEachGroupSeam(candidates, poses, model, group, [&](uint32_t i, uint32_t neighbour) {
		if (!candidates[i].IsTrusted())
			return;
		tentativeSeams.push_back(i);
		trustedNeighbours.insert(neighbour);
	});
	if (trustedNeighbours.size() < 2)
		return false;

	// the group taken in on trust, and the model averaged over the seams it would then rest on: the
	// cycle's discrepancy is spread over them and the group lands where its own seams put it
	const std::vector<BlockPose> savedPoses(poses);
	FOREACH(g, group.blocks) {
		BlockPose& pose = poses[group.blocks[g]];
		pose.T = best.T * group.frames[g];
		pose.model = model;
		pose.state = BlockPose::ADMITTED;
	}
	std::vector<Point3> residuals;
	PlacementHypothesis relaxed;
	float spread = 0;
	relaxed.source = PlacementHypothesis::INITIAL;
	if (AverageModelPoses(candidates, tentativeSeams, blockExtents, model, seed, poses, residuals)) {
		// what the cycle's discrepancy came down to once the consensus carried it, read off the
		// residuals the averaging returned: writing them into the candidates, which is where
		// LargestSeamResidual reads a model's, would leave a mark behind on the path that rolls back
		for (const Point3& residual : residuals)
			if (residual.x < REAL(FLT_MAX))
				spread = MAXF(spread, (float)residual.x);
		// the observations behind those seams read again from where the consensus put the blocks:
		// the cameras are asked a question about pixels, and the averaging answers one about seams
		RefineBlockPoses(candidates, tentativeSeams, model, seed, poses);
		// what the pool says of the group where the relaxed model puts it: the same gates, now
		// answering to a model that has taken the cycle in
		relaxed.T = poses[group.blocks.front()].T * group.frames.front().Invert();
		PlacementPool pool;
		BuildPlacementPool(subScenes, candidates, poses, model, group, pool);
		std::map<uint32_t, unsigned> numPooled;
		CountPooledPerBlock(pool, globalToLocal, numPooled);
		ScoreHypothesis(subScenes, pool, 0, ModelVoteRatio(candidates, poses, model, group, config), relaxed);
		const NeighbourVerdict verdict = CheckNeighbours(
			candidates, blockExtents, poses, model, group, numPooled, relaxed.T, config);
		relaxed.neighbourSupport = verdict.support;
		relaxed.neighbourLoop = verdict.loop;
		relaxed.neighbourContra = verdict.contra;
		if (relaxed.Passed() && !NeighboursBehind(verdict.support, verdict.loop, verdict.contra))
			relaxed.failedGate = "neighbours";
		LogVotes(String::FormatString("Placement of block %u, relaxed over its cycle",
			group.blocks.front()).c_str(), relaxed.score);
	} else
		relaxed.failedGate = "cycle averaging";
	DEBUG("Block %u judged against the model its cycle was spread over (largest seam residual "
		"%.2f deg): %u inliers of %u, votes %u+/%u- block, %u+/%u- model, %u+/%u- neighbours%s%s",
		group.blocks.front(), spread,
		relaxed.score.inliers, (unsigned)relaxed.score.inlierMask.size(),
		relaxed.score.support[0], relaxed.score.contra[0],
		relaxed.score.support[1], relaxed.score.contra[1],
		relaxed.neighbourSupport, relaxed.neighbourContra,
		relaxed.Passed() ? "" : ", dropped by ", relaxed.failedGate.c_str());
	if (!relaxed.Passed()) {
		// nothing was spread that the cameras could then follow: the model is left as it was
		poses = savedPoses;
		reason = relaxed.failedGate;
		return false;
	}
	// the relaxed poses stand, and the group comes in against them like any other
	best = relaxed;
	closedCycle = AdmitGroup(subScenes, candidates, blockExtents, model, group, relaxed, poses, modelSeams);
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::CandidateFromPrediction(
	const std::vector<Scene>& subScenes, const uint32_t a, const uint32_t b,
	const Transform& T, SeamCandidate& c) const
{
	ASSERT(a < b);
	c = SeamCandidate();
	c.sceneA = a;
	c.sceneB = b;
	c.T = T;
	c.source = SeamCandidate::UNION;

	// the prediction is the hypothesis: the pair's own correspondences are read under it at the
	// loose threshold, and what it explains there is what it is then fitted to
	CollectPairObservations(subScenes, a, b, c.observations, c.correspondences);
	const Transform TInv(T.Invert());
	std::vector<SeamObservation> inliers;
	for (const SeamObservation& obs : c.observations)
		if (SeamObservationError(obs, T, TInv) <= (REAL)kLooseSeamFactor * config.maxReprojError)
			inliers.push_back(obs);
	if (inliers.size() < config.minCommonTracks) {
		DEBUG_ULTIMATE("Seam (%u, %u) predicted: only %u of the pair's %u correspondences read under it",
			a, b, (unsigned)inliers.size(), (unsigned)c.observations.size());
		return false;
	}
	RefineSeamTransform(inliers, c.T);
	ScoreCandidate(subScenes, c);

	// the one gate it answers to: enough of the pair explained. Its own cameras' votes are not
	// asked, a pair this thin having none to cast
	const String failed = FailedGates(c.score, c.score.inliers, 0, config.minCameraVoteRatio);
	if (failed.find("union support") != String::npos) {
		DEBUG_ULTIMATE("Seam (%u, %u) predicted: %u inliers of the %u correspondences it was fitted to, "
			"dropped by %s", a, b, c.score.inliers, (unsigned)inliers.size(), failed.c_str());
		return false;
	}
	// what such a seam weighs: the cameras behind it, and at least enough to hold its two blocks
	c.weight = MAXF(1.f, c.score.Weight(config.maxVoteWeight));
	// and what it may speak for: the scale it carries is the model's own unless one of the two rigs
	// saw enough parallax to measure one, which is the question a measured seam answers here too
	c.scaleObservable = IsScaleObservable(subScenes, c, true) || IsScaleObservable(subScenes, c, false);
	return true;
}
/*----------------------------------------------------------------*/

unsigned GlobalAlignment::VerifyWeakSeams(
	const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t model,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams) const
{
	std::vector<std::pair<uint32_t, uint32_t>> pairs;
	CollectWeakPairs(candidates, blockPairLinks, poses, model, pairs);
	std::vector<Transform> transforms(poses.size());
	FOREACH(b, poses)
		if (IsInModel(poses, b, model))
			transforms[b] = poses[b].T;

	unsigned numVerified = 0;
	for (const auto& [a, b] : pairs) {
		SeamCandidate c;
		if (!CandidateFromPrediction(subScenes, a, b, poses[b].T.Invert() * poses[a].T, c))
			continue;
		// the seam the model predicted, read against the model like every other one it rests on
		const SeamResidual residual = ComputeSeamResidual(a, b, c.T, transforms, blockExtents, config);
		c.residualRotation = (float)residual.rotation;
		c.residualScale = (float)residual.scale;
		c.residualTranslation = (float)residual.translation;
		c.cls = SeamCandidate::VERIFIED;
		VERBOSE("Seam (%u, %u) verified after loop closure: %u inliers, %u votes",
			a, b, c.score.inliers, c.score.support[0] + c.score.support[1]);
		modelSeams.push_back((uint32_t)candidates.size());
		candidates.emplace_back(std::move(c));
		++numVerified;
	}
	return numVerified;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::CloseModel(
	const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t model, const uint32_t seed,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams) const
{
	std::pair<float, float> residual = RelaxBlockPoses(
		subScenes, candidates, blockExtents, model, seed, modelSeams, poses);
	// what the model now predicts of the pairs that could not measure a seam of their own, and the
	// model relaxed once more over the cycles those close
	if (VerifyWeakSeams(subScenes, candidates, blockExtents, model, poses, modelSeams) > 0)
		residual = RelaxBlockPoses(subScenes, candidates, blockExtents, model, seed, modelSeams, poses);
	DEBUG("Model %u rests on %u seams: largest seam residual %.2f deg before, %.2f deg after",
		model, (unsigned)modelSeams.size(), residual.first, residual.second);
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::SplitFoldedBlock(
	std::vector<Scene>& subScenes,
	std::vector<IIndexArr>& localToGlobals,
	std::vector<REAL>& blockExtents,
	std::vector<SeamCandidate>& candidates,
	const uint32_t block,
	const PlacementHypothesis& best,
	std::pair<uint32_t, uint32_t>& parts)
{
	// the block's own scene is read by index throughout: the two parts appended below may move the
	// blocks about, and a reference into them would not survive it
	ASSERT(subScenes.size() == localToGlobals.size() && subScenes.size() == blockExtents.size());
	const IIndex numImages = (IIndex)subScenes[block].images.size();

	// the two sides of the fold, as the block's own cameras voted on the placement
	std::vector<int> side(numImages, 0);
	unsigned numVotes[2] = {0, 0};
	for (const CameraVote& vote : best.score.votes) {
		if (vote.vote == 0)
			continue;
		const auto it = globalToLocal.find(vote.image);
		if (it == globalToLocal.end() || it->second.first != block)
			continue;
		side[it->second.second] = vote.vote;
		++numVotes[vote.vote > 0 ? 0 : 1];
	}

	// a camera that could not vote goes with the side it shares the most of its own block with,
	// spreading outward from the cameras that did until nothing more is reached
	std::vector<std::vector<std::pair<IIndex, float>>> neighbours(numImages);
	for (const ImagePair& pair : subScenes[block].pairs) {
		const float weight = pair.GetCompositeWeight();
		if (!(weight > 0))
			continue;
		neighbours[pair.ID1].emplace_back(pair.ID2, weight);
		neighbours[pair.ID2].emplace_back(pair.ID1, weight);
	}
	for (bool changed = true; changed; ) {
		changed = false;
		FOREACH(i, side) {
			if (side[i] != 0)
				continue;
			float weights[2] = {0.f, 0.f};
			for (const auto& [other, weight] : neighbours[i])
				if (side[other] != 0)
					weights[side[other] > 0 ? 0 : 1] += weight;
			if (weights[0] == 0 && weights[1] == 0)
				continue;
			side[i] = weights[0] >= weights[1] ? 1 : -1;
			changed = true;
		}
	}
	// a camera the block's own pairs never reach stays with the cameras that carried the placement,
	// which is what the connectedness of its side then answers for
	for (int& s : side)
		if (s == 0)
			s = 1;

	// what the cut costs against what holds either side together, and how much of a block each side
	// is left being -- counted in the cameras a part would be reconstructed from, an image the block
	// never registered carrying nothing either way
	IIndexArr images[2];
	unsigned numViews[2] = {0, 0};
	FOREACH(i, side) {
		const int s = side[i] > 0 ? 0 : 1;
		images[s].push_back((IIndex)i);
		if (subScenes[block].images[i].IsValid())
			++numViews[s];
	}
	float cut = 0, internal[2] = {0.f, 0.f};
	for (const ImagePair& pair : subScenes[block].pairs) {
		const float weight = pair.GetCompositeWeight();
		if (side[pair.ID1] == side[pair.ID2])
			internal[side[pair.ID1] > 0 ? 0 : 1] += weight;
		else
			cut += weight;
	}
	// the images whose connectedness answers for the fold: the ones the block registered and its own
	// weighting left a pair of
	std::vector<bool> holds(numImages, false);
	FOREACH(i, holds)
		holds[i] = subScenes[block].images[i].IsValid() && !neighbours[i].empty();
	const char* refused = NULL;
	if (numViews[0] < config.minFoldPartViews || numViews[1] < config.minFoldPartViews)
		refused = "one of its sides is too small";
	else if (cut > config.maxFoldCutRatio * MINF(internal[0], internal[1]))
		refused = "its two sides are not cut apart";
	else if (!IsSideConnected(neighbours, side, holds, images[0]) ||
			 !IsSideConnected(neighbours, side, holds, images[1]))
		refused = "one of its sides does not hold together";
	if (refused != NULL) {
		DEBUG("Block %u not split (votes %u+/%u-, %u and %u views, cut %.2f of %.2f): %s",
			block, numVotes[0], numVotes[1], numViews[0], numViews[1],
			cut, MINF(internal[0], internal[1]), refused);
		return false;
	}

	// the block becomes two, each side taking its own images, pairs and points with it; the pairs
	// that ran across the cut stay behind with the block that was cut, and no merge brings them
	// back to the scene -- they are the matches that glued the fold, and the parts are the cameras
	// they should never have joined
	parts = std::make_pair((uint32_t)subScenes.size(), (uint32_t)subScenes.size() + 1);
	const IIndexArr blockToGlobal(localToGlobals[block]);
	std::vector<IIndexArr> partToBlock;
	const ClusterConfig clusterCfg;
	std::vector<Scene> split = SceneCluster(subScenes[block], clusterCfg).SplitSceneByClusters(
		{images[0], images[1]}, &partToBlock);
	ASSERT(split.size() == 2 && partToBlock.size() == 2);
	FOREACH(p, split) {
		IIndexArr localToGlobal(partToBlock[p].size());
		FOREACH(k, partToBlock[p])
			localToGlobal[k] = blockToGlobal[partToBlock[p][k]];
		subScenes.emplace_back(std::move(split[p]));
		subScenes.back().RecomputeCalibratedImages();
		localToGlobals.emplace_back(std::move(localToGlobal));
		blockExtents.push_back(BlockExtent(subScenes.back()));
	}
	ExtendSeamEvidence(subScenes, localToGlobals, parts.first);

	// what the block measured was measured for a reconstruction that was cut in two
	for (SeamCandidate& c : candidates)
		if (c.sceneA == block || c.sceneB == block)
			c.cls = SeamCandidate::REJECTED;
	// and every pair a part now holds is measured like any other block pair. The graph cannot be
	// asked again about blocks that did not exist when it spoke, so what comes out is read by the
	// one rule it keeps for a seam no second path corroborates
	std::vector<std::pair<uint32_t, uint32_t>> partPairs;
	for (const auto& [blockPair, links] : blockPairLinks)
		if (blockPair.first >= parts.first || blockPair.second >= parts.first)
			partPairs.push_back(blockPair);
	const size_t numMeasured = candidates.size();
	for (const auto& [a, b] : partPairs)
		EstimateSeamPair(subScenes, a, b, candidates);
	for (size_t i = numMeasured; i < candidates.size(); ++i)
		candidates[i].cls = ClassifyUncorroboratedSeam(candidates[i], config);

	VERBOSE("Block %u split into %u and %u (votes %u+/%u-, cut %.2f)",
		block, parts.first, parts.second, numVotes[0], numVotes[1], cut);
	return true;
}
/*----------------------------------------------------------------*/

unsigned GlobalAlignment::PlaceBlocks(
	std::vector<Scene>& subScenes,
	std::vector<IIndexArr>& localToGlobals,
	std::vector<SeamCandidate>& candidates,
	std::vector<REAL>& blockExtents,
	const uint32_t model,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams)
{
	TD_TIMER_STARTD();
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

	// a model that already holds blocks is being grown again over what it let go, and keeps them
	// and the seams it rests on; a new one grows from the block the seam graph is most sure of,
	// which has to be one the averaging could place: it is that pose the model frame is set at
	BlockGroup group;
	uint32_t seed = ModelSeed(poses, model);
	if (seed == NO_ID) {
		modelSeams.clear();
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
		group.blocks.assign(1, seed);
		group.frames.assign(1, Transform());
		PlacementHypothesis start;
		start.source = PlacementHypothesis::INITIAL;
		start.T = poses[seed].T;
		AdmitGroup(subScenes, candidates, blockExtents, model, group, start, poses, modelSeams);
		VERBOSE("Model %u seeded with block %u (%u images)",
			model, seed, subScenes[seed].status.nCalibratedImages);
	}

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
			const float support = PooledSupport(
				candidates, poses, blockPairLinks, refusedSeamPairs, model, b);
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
		bool closedCycle = false;
		if (PlaceGroup(subScenes, candidates, blockExtents, model, group, poses, winner, reason))
			closedCycle = AdmitGroup(
				subScenes, candidates, blockExtents, model, group, winner, poses, modelSeams);
		// the block the placement could not carry may be the one a cycle runs through: it is judged
		// again once the model has taken that cycle in, and comes in with it when it holds there
		else if (!CloseCycleThrough(subScenes, candidates, blockExtents, model, seed, group, winner,
				poses, modelSeams, closedCycle, reason)) {
			// or it may be two blocks the reconstruction folded into one, which its own cameras
			// saying opposite things about the same pose is what gives away
			if (VotesSplit(winner, config)) {
				std::pair<uint32_t, uint32_t> parts;
				if (SplitFoldedBlock(subScenes, localToGlobals, blockExtents, candidates, next, winner, parts)) {
					DropBlockSeams(candidates, next, modelSeams);
					QueueSplitParts(next, parts, poses);
					eligible.push_back(parts.first);
					eligible.push_back(parts.second);
					continue;
				}
				reason = "votes split";
			}
			poses[next].state = BlockPose::DEFERRED;
			poses[next].reason = reason;
			deferred.push_back(next);
			DEBUG("Block %u deferred: %s", next, reason.c_str());
			continue;
		}
		modelChanged = true;
		DEBUG("Block %u placed on %u neighbours (%u support, %u loop, %u contradict); "
			"votes %u+/%u- block, %u+/%u- model; %s", next,
			winner.neighbourSupport + winner.neighbourLoop + winner.neighbourContra,
			winner.neighbourSupport, winner.neighbourLoop, winner.neighbourContra,
			winner.score.support[0], winner.score.contra[0],
			winner.score.support[1], winner.score.contra[1], PlacementWord(winner.source));
		if (closedCycle) {
			// the block joined the model from two sides: the error the chain accumulated is spread
			// over the cycle it just closed, instead of being left at the seam that closed it
			const auto [before, after] = RelaxBlockPoses(
				subScenes, candidates, blockExtents, model, seed, modelSeams, poses);
			VERBOSE("Loop closed through block %u: largest seam residual %.2f deg before, %.2f deg after",
				next, before, after);
		} else
			// every admitted block fitted again to the seams the model rests on, so a seam's two
			// ends can move apart instead of passing the error on
			RefineBlockPoses(candidates, modelSeams, model, seed, poses);
	}

	// the model as a whole, once it has taken in everything it can: averaged over its own seams, and
	// over the pairs of it that only the model itself can read
	CloseModel(subScenes, candidates, blockExtents, model, seed, poses, modelSeams);

	// a block this model could not take but no model owns is left where it was found, so the next
	// model may try it too; it keeps the reason of the last model that looked at it
	for (const uint32_t b : deferred)
		if (poses[b].state == BlockPose::DEFERRED && poses[b].model != model)
			poses[b].state = BlockPose::UNPLACED;

	const unsigned numAdmitted = (unsigned)ModelBlocks(poses, model).size();
	VERBOSE("Model %u: %u/%u blocks placed on %u seams (%s)", model, numAdmitted,
		(unsigned)eligible.size(), (unsigned)modelSeams.size(), TD_TIMER_GET_FMT().c_str());
	return numAdmitted;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::PlaceModel(
	const std::vector<Scene>& subScenes,
	std::vector<SeamCandidate>& candidates,
	const std::vector<REAL>& blockExtents,
	const uint32_t src, const uint32_t dst,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams,
	String& reason) const
{
	// the whole model as one group, every block of it where its own model put it: the blocks of a
	// model are rigid against each other, so one similarity carries all of them and the cameras of
	// both models vote on that one
	BlockGroup group;
	group.blocks = ModelBlocks(poses, src);
	ASSERT(!group.blocks.empty());
	for (const uint32_t b : group.blocks)
		group.frames.push_back(poses[b].T);

	const uint32_t seed = ModelSeed(poses, dst);
	PlacementHypothesis winner;
	if (!PlaceGroup(subScenes, candidates, blockExtents, dst, group, poses, winner, reason))
		return false;
	AdmitGroup(subScenes, candidates, blockExtents, dst, group, winner, poses, modelSeams);
	// the two models are one model now: averaged over every seam it rests on, and the pairs only
	// the union can read verified against it
	CloseModel(subScenes, candidates, blockExtents, dst, seed, poses, modelSeams);
	VERBOSE("Model %u (%u blocks) placed on model %u: %u inliers, votes %u+/%u- src, %u+/%u- dst",
		src, (unsigned)group.blocks.size(), dst, winner.score.inliers,
		winner.score.support[0], winner.score.contra[0],
		winner.score.support[1], winner.score.contra[1]);
	return true;
}
/*----------------------------------------------------------------*/

unsigned GlobalAlignment::RevalidateBlocks(
	std::vector<Scene>& subScenes,
	std::vector<IIndexArr>& localToGlobals,
	std::vector<REAL>& blockExtents,
	std::vector<SeamCandidate>& candidates,
	const uint32_t model,
	std::vector<BlockPose>& poses,
	std::vector<uint32_t>& modelSeams)
{
	TD_TIMER_STARTD();
	const std::vector<uint32_t> admitted = ModelBlocks(poses, model);
	if (admitted.size() < 2)
		return 0;

	unsigned numLetGo = 0;
	for (const uint32_t b : admitted) {
		if (!IsInModel(poses, b, model))
			continue; // let go while another block of the model was being judged
		// a block the others were let go around has nothing left to be judged against: the votes of
		// an empty pool refuse anything, and would leave the model holding no block at all
		if (ModelBlocks(poses, model).size() < 2)
			break;
		// the block against every other block the model holds, at the pose the model left it at
		BlockGroup group;
		group.blocks.assign(1, b);
		group.frames.assign(1, Transform());
		PlacementPool pool;
		BuildPlacementPool(subScenes, candidates, poses, model, group, pool);
		PlacementHypothesis current;
		current.source = PlacementHypothesis::INITIAL;
		current.T = poses[b].T;
		const float voteRatio = ModelVoteRatio(candidates, poses, model, group, config);
		ScoreHypothesis(subScenes, pool, 0, voteRatio, current);
		// only the cameras and the verified pairs unplace a block the model already holds: what it
		// explains of a pool that has grown around it, and how its cameras sit among the model's,
		// is not what it came in on
		const PairAgreement pairs = MeasurePlacementPairs(subScenes, model, group, current.T, poses);
		if (!pairs.Holds())
			current.failedGate += current.failedGate.empty() ? "pair agreement" : ", pair agreement";
		if (current.failedGate.find("camera votes") == String::npos &&
			current.failedGate.find("pair agreement") == String::npos)
			continue;
		LogVotes(String::FormatString("Block %u judged again", b).c_str(), current.score);
		DropBlockSeams(candidates, b, modelSeams);
		++numLetGo;
		// a block whose own cameras split is the fold that a single admitted neighbour hid: only
		// one of its halves had anything to answer to when it came in
		std::pair<uint32_t, uint32_t> parts;
		if (VotesSplit(current, config) &&
			SplitFoldedBlock(subScenes, localToGlobals, blockExtents, candidates, b, current, parts)) {
			QueueSplitParts(b, parts, poses);
			continue;
		}
		const unsigned numContra = current.score.contra[0] + current.score.contra[1];
		UnplaceBlock(poses[b], numContra > 0 ?
			String::FormatString("contradicted by %u cameras", numContra) : !pairs.Holds() ?
			String::FormatString("contradicted by the verified pairs (%.0f against, %.0f for)", pairs.disagree, pairs.agree) :
			String::FormatString("supported by %u camera centres, under the %u asked", current.score.centres, config.minSupportingCentres));
		VERBOSE("Block %u let go by the model that held it: %s", b, poses[b].reason.c_str());
	}
	// the model without them, and without the seams it rested on through them
	const uint32_t seed = ModelSeed(poses, model);
	if (numLetGo > 0 && seed != NO_ID)
		CloseModel(subScenes, candidates, blockExtents, model, seed, poses, modelSeams);
	DEBUG("Model %u judged again: %u of its %u blocks let go (%s)",
		model, numLetGo, (unsigned)admitted.size(), TD_TIMER_GET_FMT().c_str());
	return numLetGo;
}
/*----------------------------------------------------------------*/

void GlobalAlignment::PlaceRemainingBlocks(
	std::vector<Scene>& subScenes,
	std::vector<IIndexArr>& localToGlobals,
	std::vector<SeamCandidate>& candidates,
	std::vector<REAL>& blockExtents,
	std::vector<BlockPose>& poses,
	std::vector<std::vector<uint32_t>>& modelSeams)
{
	TD_TIMER_STARTD();
	// the blocks no model took in: they start over, on the trusted seams among themselves alone
	std::vector<bool> remaining(poses.size(), false);
	unsigned numRemaining = 0;
	FOREACH(b, poses)
		if (poses[b].state != BlockPose::ADMITTED && poses[b].state != BlockPose::SPLIT) {
			remaining[b] = true;
			poses[b].state = BlockPose::UNPLACED;
			poses[b].model = NO_ID;
			++numRemaining;
		}
	if (numRemaining > 0) {
		const uint32_t firstModel = (uint32_t)modelSeams.size();
		modelSeams.resize(firstModel +
			AverageTrustedComponents(candidates, blockExtents, remaining, firstModel, poses));
		for (uint32_t m = firstModel; m < (uint32_t)modelSeams.size(); ++m)
			PlaceBlocks(subScenes, localToGlobals, candidates, blockExtents, m, poses, modelSeams[m]);
		// a block no trusted seam reaches is a model of one, the largest first: the placement grows
		// it over whatever the graph left undecided, and the next one takes what it leaves
		while (true) {
			uint32_t next = NO_ID;
			FOREACH(b, poses)
				if (poses[b].state == BlockPose::UNPLACED && poses[b].model == NO_ID &&
					(next == NO_ID || subScenes[b].status.nCalibratedImages > subScenes[next].status.nCalibratedImages))
					next = (uint32_t)b;
			if (next == NO_ID)
				break;
			const uint32_t model = (uint32_t)modelSeams.size();
			poses[next].T = Transform();
			poses[next].model = model;
			modelSeams.emplace_back();
			PlaceBlocks(subScenes, localToGlobals, candidates, blockExtents, model, poses, modelSeams[model]);
		}
	}

	// the model the merge is built on, and the others ranked behind it: a model is placed against
	// everything already taken in, so the largest goes first and the smallest answers to the most
	std::vector<std::pair<unsigned, uint32_t>> models;
	for (uint32_t m = 0; m < (uint32_t)modelSeams.size(); ++m)
		if (!ModelBlocks(poses, m).empty())
			models.emplace_back(ModelImages(subScenes, poses, m), m);
	if (models.empty())
		return;
	std::sort(models.begin(), models.end(),
		[](const std::pair<unsigned, uint32_t>& a, const std::pair<unsigned, uint32_t>& b) {
			return a.first > b.first || (a.first == b.first && a.second < b.second); });
	const uint32_t merged = models.front().second;

	for (const std::pair<unsigned, uint32_t>& entry : models) {
		const uint32_t src = entry.second;
		if (src == merged)
			continue;
		const std::vector<uint32_t> blocks = ModelBlocks(poses, src);
		// a model nothing joins to the merged one has no evidence to be placed on at all
		bool linked = false;
		for (const auto& [blockPair, links] : blockPairLinks)
			if ((IsInModel(poses, blockPair.first, src) && IsInModel(poses, blockPair.second, merged)) ||
				(IsInModel(poses, blockPair.second, src) && IsInModel(poses, blockPair.first, merged))) {
				linked = true;
				break;
			}
		String reason("no seam");
		if (linked) {
			// the seams it rests on come with it, one similarity moving all of its blocks at once
			std::vector<uint32_t>& seams = modelSeams[merged];
			const size_t numSeams = seams.size();
			seams.insert(seams.end(), modelSeams[src].begin(), modelSeams[src].end());
			if (PlaceModel(subScenes, candidates, blockExtents, src, merged, poses, seams, reason)) {
				modelSeams[src].clear();
				continue;
			}
			seams.resize(numSeams);
			// a model of more than one block says only that it stands apart, whatever gate refused
			// it; a model of one keeps the gates, which is all there is to say of a single block
			if (blocks.size() > 1)
				reason = String::FormatString("separate model of %u blocks", (unsigned)blocks.size());
		}
		VERBOSE("Model %u (%u blocks) not placed: %s", src, (unsigned)blocks.size(), reason.c_str());
		for (const uint32_t b : blocks)
			UnplaceBlock(poses[b], reason);
	}
	DEBUG("Blocks left over placed in %u models against model %u (%s)",
		(unsigned)models.size(), merged, TD_TIMER_GET_FMT().c_str());
}
/*----------------------------------------------------------------*/

// The similarity carrying one camera's own frame into the frame of the block it belongs to: the
// camera reads that frame through its rotation, and sits where its centre says
static Transform CameraToBlock(const Image& image)
{
	Transform T;
	T.R = RMatrix(image.R.t());
	T.t = image.C;
	return T;
}

// What one edge of the camera pose graph measures: the first camera's frame into the second's,
// through the transform between the two blocks they sit in -- the identity when both belong to the
// same block, and the seam itself when they face each other across one
static Transform CameraMeasurement(const Image& first, const Image& second, const Transform& blockToBlock)
{
	return CameraToBlock(second).Invert() * blockToBlock * CameraToBlock(first);
}

// Where the blocks the given seams touch hold the points their cameras triangulated, as they now
// stand: a seam observation carries the geometry its block had when the seam was measured, so this
// is what it has to be read against once anything has moved the blocks
static std::vector<std::unordered_map<PairIdx, Point3>> SeamPointMaps(
	const std::vector<Scene>& subScenes,
	const std::vector<SeamCandidate>& candidates,
	const std::vector<uint32_t>& modelSeams)
{
	std::vector<std::unordered_map<PairIdx, Point3>> pointMaps(subScenes.size());
	std::vector<bool> touched(subScenes.size(), false);
	for (const uint32_t seamIdx : modelSeams) {
		touched[candidates[seamIdx].sceneA] = true;
		touched[candidates[seamIdx].sceneB] = true;
	}
	FOREACH(b, subScenes)
		if (touched[b])
			BuildBlockPointMap(subScenes[b], pointMaps[b]);
	return pointMaps;
}

// The worst the seams of one model disagree, as the blocks now stand: per seam, the middle error
// its inlier observations report at the similarity its two block poses imply, in the pixels the
// reprojection bar is set in. Every observation is read off the blocks again -- the camera where
// its block now puts it, the point where the block that triangulated it now holds it -- so the same
// number measures the seams before a camera relaxation and after it.
static float LargestSeamError(
	const std::vector<Scene>& subScenes,
	const std::vector<SeamCandidate>& candidates,
	const std::vector<uint32_t>& modelSeams,
	const std::vector<BlockPose>& poses,
	const std::unordered_map<IIndex, std::pair<uint32_t, IIndex>>& globalToLocal,
	const uint32_t model)
{
	const std::vector<std::unordered_map<PairIdx, Point3>> pointMaps(
		SeamPointMaps(subScenes, candidates, modelSeams));
	float worst = 0.f;
	for (const uint32_t seamIdx : modelSeams) {
		const SeamCandidate& c = candidates[seamIdx];
		if (!IsInModel(poses, c.sceneA, model) || !IsInModel(poses, c.sceneB, model))
			continue;
		// the seam the two block poses imply, which is where a rigid placement leaves the pair
		const Transform T(poses[c.sceneB].T.Invert() * poses[c.sceneA].T);
		const Transform TInv(T.Invert());
		std::vector<REAL> errors;
		const size_t numScored = MINF(MINF(c.observations.size(), c.correspondences.size()), c.score.inlierMask.size());
		for (size_t i = 0; i < numScored; ++i) {
			if (!c.score.inlierMask[i])
				continue;
			SeamObservation obs(c.observations[i]);
			const auto itRig = globalToLocal.find(obs.rigImage);
			const auto itPoint = globalToLocal.find(obs.pointImage);
			if (itRig == globalToLocal.end() || itPoint == globalToLocal.end())
				continue;
			const Image& rig = subScenes[itRig->second.first].images[itRig->second.second];
			const SeamCorrespondence& corr = c.correspondences[i];
			const std::unordered_map<PairIdx, Point3>& pointMap = pointMaps[itPoint->second.first];
			const auto itX = pointMap.find(PairIdx(itPoint->second.second, obs.forward ? corr.featureA : corr.featureB));
			if (!rig.IsValid() || itX == pointMap.end())
				continue;
			obs.R = rig.R;
			obs.C = rig.C;
			obs.X = itX->second;
			errors.push_back(SeamObservationError(obs, T, TInv));
		}
		if (errors.empty())
			continue;
		std::nth_element(errors.begin(), errors.begin() + errors.size() / 2, errors.end());
		worst = MAXF(worst, (float)errors[errors.size() / 2]);
	}
	return worst;
}

// One block moved to where the relaxation put its cameras: each camera's similarity is read back
// out of the model frame into the block's own, its rotation and centre are what the block stores,
// and every point follows the camera of its first inlier observation -- the only place the scale a
// camera picked up can go, a pose having nowhere to keep it. A track whose observers did not all
// move the same way is then triangulated again from where they now stand, so the tail's own
// filtering reads a point its cameras agree on rather than one carried by the first of them.
static void MoveBlockCameras(
	const Transform& blockPose, const std::vector<uint32_t>& nodeOf,
	const std::vector<BlockParameters>& parameters, const float maxReprojError, Scene& block)
{
	const Transform blockPoseInv(blockPose.Invert());
	std::vector<Transform> deltas(block.images.size());
	std::vector<bool> moved(block.images.size(), false);
	FOREACH(localID, block.images) {
		const uint32_t node = nodeOf[localID];
		if (node == NO_ID || !ISFINITE(parameters[node].logScale))
			continue;
		Image& image = block.images[localID];
		const Transform relaxed(blockPoseInv * parameters[node].ToTransform());
		deltas[localID] = relaxed * CameraToBlock(image).Invert();
		moved[localID] = true;
		image.R = RMatrix(relaxed.R.t());
		image.C = relaxed.t;
	}
	for (Track& track : block.tracks) {
		bool anyMoved = false;
		for (const Observation& obs : track)
			if (obs.imageID < moved.size() && moved[obs.imageID]) {
				track.position = deltas[obs.imageID] * track.position;
				anyMoved = true;
				break;
			}
		// the cameras that saw it have the last word, so what the tail filters is a point they
		// agree on and not one the first of them carried; a track they cannot agree on comes back
		// with too few inliers and is dropped there, like any other the cameras cannot hold
		if (anyMoved && track.IsValid())
			TriangulateSkewLLS(track, block.images, maxReprojError, kRelaxedTrackMinAngle);
	}
}

bool GlobalAlignment::RelaxCameras(
	std::vector<Scene>& subScenes,
	const std::vector<SeamCandidate>& candidates,
	const std::vector<uint32_t>& modelSeams,
	const std::vector<REAL>& blockExtents,
	const uint32_t seed,
	std::vector<BlockPose>& poses,
	MergeReport& report) const
{
	ASSERT(poses.size() == subScenes.size());
	if (seed >= poses.size() || poses[seed].state != BlockPose::ADMITTED)
		return false;
	const uint32_t model = poses[seed].model;
	report.seamErrorBeforeRelax = report.seamErrorAfterRelax =
		LargestSeamError(subScenes, candidates, modelSeams, poses, globalToLocal, model);
	// a model whose seams meet where the block poses leave them has nothing to relax: no block of it
	// is bent by more than the placement has already answered for
	if (report.seamErrorBeforeRelax <= config.relaxSeamResidualFactor * config.maxReprojError)
		return false;

	// one similarity per camera of the placed blocks, starting where its own block pose puts it
	std::vector<BlockParameters> parameters;
	std::vector<std::vector<uint32_t>> nodeOf(subScenes.size());
	FOREACH(b, subScenes) {
		nodeOf[b].assign(subScenes[b].images.size(), NO_ID);
		if (!IsInModel(poses, (uint32_t)b, model))
			continue;
		const Scene& block = subScenes[b];
		FOREACH(localID, block.images) {
			if (!block.images[localID].IsValid())
				continue;
			nodeOf[b][localID] = (uint32_t)parameters.size();
			parameters.emplace_back(poses[b].T * CameraToBlock(block.images[localID]));
		}
	}

	ceres::Problem problem;
	ceres::LossFunction* loss = new ceres::HuberLoss(kCameraGraphHuber);
	const REAL scaleBar = LOGN((REAL)config.maxSimScaleRatio);
	const auto AddEdge = [&](const uint32_t first, const uint32_t second, const Transform& M,
		const REAL extent, const float weight) {
		const REAL translationBar = (REAL)config.maxSimTranslationError * extent;
		// a block with no footprint of its own gives its translations no unit to be judged in
		if (!(translationBar > 0))
			return false;
		BlockParameters& i = parameters[first];
		BlockParameters& j = parameters[second];
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<CameraGraphError, 7, 4, 3, 1, 4, 3, 1>(
				new CameraGraphError(M, weight, config.maxSimRotationError, translationBar, scaleBar)),
			loss, i.q, i.t, &i.logScale, j.q, j.t, &j.logScale);
		return true;
	};

	// every camera held to the strongest covisible cameras of its own block: what the block's own
	// reconstruction says about where its cameras sit relative to each other, which is all that
	// keeps a block from coming apart once its cameras move one by one
	unsigned numIntraEdges = 0;
	FOREACH(b, subScenes) {
		if (!IsInModel(poses, (uint32_t)b, model))
			continue;
		const Scene& block = subScenes[b];
		std::vector<std::vector<std::pair<float, IIndex>>> covisible(block.images.size());
		for (const ImagePair& pair : block.pairs) {
			if (nodeOf[b][pair.ID1] == NO_ID || nodeOf[b][pair.ID2] == NO_ID)
				continue;
			// a pair the weighting refused carries no evidence that the two cameras see the same
			// thing, and must not take one of the three places a camera has to give
			const float weight = pair.GetCompositeWeight();
			if (!(weight > 0))
				continue;
			covisible[pair.ID1].emplace_back(weight, pair.ID2);
			covisible[pair.ID2].emplace_back(weight, pair.ID1);
		}
		std::set<std::pair<IIndex, IIndex>> edges;
		for (IIndex localID = 0; localID < (IIndex)covisible.size(); ++localID) {
			std::vector<std::pair<float, IIndex>>& ranked = covisible[localID];
			const size_t numKept = MINF(ranked.size(), (size_t)kIntraBlockEdgesPerCamera);
			std::partial_sort(ranked.begin(), ranked.begin() + numKept, ranked.end(), std::greater<>());
			for (size_t k = 0; k < numKept; ++k) {
				// the edge says the same thing read from either end, so it is added once
				const IIndex other = ranked[k].second;
				if (!edges.emplace(MINF(localID, other), MAXF(localID, other)).second)
					continue;
				if (AddEdge(nodeOf[b][localID], nodeOf[b][other],
						CameraMeasurement(block.images[localID], block.images[other], Transform()),
						blockExtents[b], config.intraBlockEdgeWeight))
					++numIntraEdges;
			}
		}
	}

	// and every camera that observed across a seam held to the cameras whose features it saw, at
	// the similarity the seam measured between their two blocks: a camera with too little to say
	// across the seam is not asked, the same bar its vote on the seam answered to
	unsigned numSeamEdges = 0;
	for (const uint32_t seamIdx : modelSeams) {
		const SeamCandidate& c = candidates[seamIdx];
		if (!IsInModel(poses, c.sceneA, model) || !IsInModel(poses, c.sceneB, model))
			continue;
		// ordered, so the edges enter the problem -- and the normal equations sum -- the same way
		// every run, whatever the standard library does with a hash
		std::map<IIndex, std::vector<size_t>> observedBy;
		const size_t numScored = MINF(c.observations.size(), c.score.inlierMask.size());
		for (size_t i = 0; i < numScored; ++i)
			if (c.score.inlierMask[i])
				observedBy[c.observations[i].rigImage].push_back(i);
		const Transform TInv(c.T.Invert());
		for (const auto& [rigImage, observations] : observedBy) {
			if (observations.size() < config.minVoteCorrespondences)
				continue;
			const auto itRig = globalToLocal.find(rigImage);
			if (itRig == globalToLocal.end() || nodeOf[itRig->second.first][itRig->second.second] == NO_ID)
				continue;
			std::set<IIndex> pointImages;
			for (const size_t i : observations) {
				const SeamObservation& obs = c.observations[i];
				if (!pointImages.insert(obs.pointImage).second)
					continue;
				const auto itPoint = globalToLocal.find(obs.pointImage);
				if (itPoint == globalToLocal.end() || nodeOf[itPoint->second.first][itPoint->second.second] == NO_ID)
					continue;
				const Scene& pointBlock = subScenes[itPoint->second.first];
				// the similarity carrying the point camera's block into the rig camera's
				const Transform& pointToRig = obs.forward ? c.T : TInv;
				// an edge across a seam is read in the unit the seam itself is judged in, the
				// footprint the two blocks share -- and it is read in the point camera's own frame,
				// so the rig block's extent enters it in the point block's units: each block is
				// reconstructed at a scale of its own, and comparing the two raw would hold the
				// edge to a bar off by whatever that scale is. One then means the same here as at
				// every seam bar
				if (AddEdge(nodeOf[itPoint->second.first][itPoint->second.second],
						nodeOf[itRig->second.first][itRig->second.second],
						CameraMeasurement(pointBlock.images[itPoint->second.second],
							subScenes[itRig->second.first].images[itRig->second.second], pointToRig),
						PairExtent(blockExtents[itPoint->second.first],
							blockExtents[itRig->second.first] / pointToRig.scale), 1.f))
					++numSeamEdges;
			}
		}
	}
	if (numSeamEdges == 0) {
		DEBUG("Cameras not relaxed: the seams left %.1f px apart carry no camera pair to hold",
			report.seamErrorBeforeRelax);
		return false;
	}

	unsigned numCameras = 0;
	FOREACH(k, parameters) {
		if (!problem.HasParameterBlock(parameters[k].q))
			continue;
		problem.SetManifold(parameters[k].q, new ceres::QuaternionManifold);
		++numCameras;
	}
	// the gauge: the model frame is the seed block's first camera that carries an edge, so its
	// similarity is what everything else moves against -- a camera no residual reaches anchors
	// nothing, and leaving the problem with no datum at all is what the solver answers with an
	// arbitrary one
	uint32_t gauge = NO_ID;
	for (IIndex localID = 0; gauge == NO_ID && localID < (IIndex)nodeOf[seed].size(); ++localID) {
		const uint32_t node = nodeOf[seed][localID];
		if (node != NO_ID && problem.HasParameterBlock(parameters[node].q))
			gauge = node;
	}
	if (gauge == NO_ID) {
		DEBUG("Cameras not relaxed: the seams left %.1f px apart reach no camera of the seed block",
			report.seamErrorBeforeRelax);
		return false;
	}
	problem.SetParameterBlockConstant(parameters[gauge].q);
	problem.SetParameterBlockConstant(parameters[gauge].t);
	problem.SetParameterBlockConstant(&parameters[gauge].logScale);

	ceres::Solver::Options options;
	// seven parameters per camera and two cameras per edge: the normal equations stay as sparse as
	// the covisibility the edges were read off
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 100;
	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	if (!summary.IsSolutionUsable()) {
		DEBUG("Cameras not relaxed: the graph of %u cameras over %u edges did not solve, "
			"the seams left %.1f px apart",
			numCameras, numIntraEdges + numSeamEdges, report.seamErrorBeforeRelax);
		return false;
	}

	// the relaxed cameras written back into their blocks' own frames: the block poses stay as the
	// placement left them, so what the relaxation bent is the reconstruction inside each block
	FOREACH(b, subScenes)
		if (IsInModel(poses, (uint32_t)b, model))
			MoveBlockCameras(poses[b].T, nodeOf[b], parameters, config.maxReprojError, subScenes[b]);
	report.seamErrorAfterRelax = LargestSeamError(subScenes, candidates, modelSeams, poses, globalToLocal, model);
	report.camerasRelaxed = true;
	VERBOSE("Cameras relaxed: %u cameras, %u intra-block edges, %u seam edges; "
		"largest seam residual %.1f px -> %.1f px",
		numCameras, numIntraEdges, numSeamEdges, report.seamErrorBeforeRelax, report.seamErrorAfterRelax);
	return true;
}
/*----------------------------------------------------------------*/

bool GlobalAlignment::MergeTransformedScenes(
	std::vector<Scene>& subScenes,
	const std::vector<IIndexArr>& localToGlobals,
	const std::vector<BlockPose>& poses,
	const std::vector<SeamCorrespondence>& seamInliers)
{
	// A block still admitted is one the merged model took in: stage 6 left every other block
	// unplaceable, and an unplaced block is merged without poses below, so transforming it would
	// be wasted work -- and its pose, which no placement ever confirmed, would inject garbage. A
	// block that was cut along a fold is merged neither way: its images, its pairs and its points
	// went to its two parts, which are blocks of their own here.
	ASSERT(poses.size() == subScenes.size());
	std::vector<bool> merged(subScenes.size(), true), placed(subScenes.size(), false);
	FOREACH(sceneIdx, subScenes) {
		merged[sceneIdx] = poses[sceneIdx].state != BlockPose::SPLIT;
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
		if (merged[sceneIdx] && !placed[sceneIdx])
			for (const IIndex globalID : localToGlobals[sceneIdx])
				if (globalID != NO_ID && globalID < scene.images.size())
					unplacedImages[globalID] = true;

	// Track per-camera accumulation counts; destination cameras accumulate directly
	std::unordered_map<Camera*, unsigned> cameraAccumCount;

	// Merge each sub-scene into the global scene
	scene.status.nCalibratedImages = 0;
	unsigned numMerged = 0;
	FOREACH(sceneIdx, subScenes) {
		if (!merged[sceneIdx])
			continue;
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
