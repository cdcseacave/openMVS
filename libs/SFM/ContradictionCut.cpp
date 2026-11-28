////////////////////////////////////////////////////////////////////
// ContradictionCut.cpp
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "ContradictionCut.h"
#include "GlobalRotationAveraging.h"
#include "GlobalScaleAveraging.h"
#include "GlobalTranslationAveraging.h"
#include "PairsWeighting.h"
#include "PoseLink.h"
#include "RobustAveraging.h"
#include "Scene.h"
#include "SequenceChain.h"
#include "StarInitializer.h"
#include "../Math/DisjointSet.h"

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ContrCut"));

namespace {

// Constants of the scale + centres solve (SolveScaleComponents)
constexpr unsigned minScaleComponent = 10;  // pairs a component of the pair-scale graph needs to be solved
constexpr unsigned numL1Rounds = 40;        // reweighting rounds of the centres
constexpr REAL l1Floor = REAL(1e-3);        // L1 weight's error floor, in median pair lengths
constexpr float maxScaleResidual = 1.05f;   // Geman-McClure scale of the pair scales

// Constants of the contradiction-cut repair (RepairContradictedPairs)
constexpr REAL kRepairAngle = 1;            // degrees between a match's ray and its epipolar plane
constexpr unsigned kRepairRounds = 200;     // RANSAC rounds of the direction fit
constexpr unsigned kRepairMinMatches = 8;   // track-forming matches that must agree
constexpr float kRepairMinShare = 0.8f;     // share of track-forming matches that must agree
constexpr REAL kRepairMaxDirection = 20;    // degrees between fitted direction and centres

} // namespace
/*----------------------------------------------------------------*/

std::vector<Point3> SFM::ComputeWorldBaselines(const Scene& scene, const std::vector<Point3>& rotations, const std::vector<uint32_t>& componentOf)
{
	ASSERT(rotations.size() == scene.images.size() && componentOf.size() == scene.images.size());
	std::vector<Point3> baselines(scene.pairs.size(), Point3::INF);
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (pair.relativePose.has_value() && componentOf[pair.ID1] != NO_ID && componentOf[pair.ID1] == componentOf[pair.ID2])
			baselines[idxPair] = RMatrix(rotations[pair.ID1]).t() * pair.relativePose->C;
	}
	return baselines;
}
bool SFM::HasBaseline(const Point3& baseline)
{
	return baseline != Point3::INF;
}
bool SFM::HasBaselineLength(const Point3& baseline)
{
	return HasBaseline(baseline) && norm(baseline) > ZEROTOLERANCE<REAL>();
}
bool SFM::HasBaselineDirection(const ImagePair& pair, const Point3& baseline)
{
	return HasBaselineLength(baseline) && pair.meanRayAngle >= D2R(kNearDuplicateRayAngle);
}

std::vector<ScaleComponentSolve> SFM::SolveScaleComponents(const Scene& scene, const std::vector<ScalePair>& equations,
	const std::vector<Point3>& baselines, const std::vector<float>& weights, unsigned& numComponents)
{
	const uint32_t numPairs = (uint32_t)scene.pairs.size();
	const uint32_t numImages = (uint32_t)scene.images.size();
	ASSERT(baselines.size() == numPairs && weights.size() == numPairs);
	// Components of the pair-scale graph, largest first (by pairs, ties to the lower pair)
	DisjointSet<uint32_t> sets(numPairs);
	std::vector<uint8_t> inEquation(numPairs, 0);
	for (const ScalePair& eq : equations) {
		sets.UnionMin(eq.idxA, eq.idxB);
		inEquation[eq.idxA] = inEquation[eq.idxB] = 1;
	}
	struct Component { uint32_t root; std::vector<uint32_t> pairs; std::vector<ScalePair> equations; };
	std::unordered_map<uint32_t, Component> componentOfRoot;
	for (const ScalePair& eq : equations) {
		const uint32_t root = sets.Find(eq.idxA);
		Component& comp = componentOfRoot[root];
		comp.root = root;
		comp.equations.push_back(eq);
	}
	for (uint32_t idxPair = 0; idxPair < numPairs; ++idxPair)
		if (inEquation[idxPair])
			componentOfRoot[sets.Find(idxPair)].pairs.push_back(idxPair);
	std::vector<Component*> components;
	components.reserve(componentOfRoot.size());
	for (auto& [root, comp] : componentOfRoot)
		components.push_back(&comp);
	std::sort(components.begin(), components.end(), [](const Component* a, const Component* b) {
		return a->pairs.size() != b->pairs.size() ? a->pairs.size() > b->pairs.size() : a->root < b->root;
	});
	numComponents = (unsigned)components.size();

	// One solve per component
	std::vector<ScaleComponentSolve> solves;
	const REAL maxLogScaleResidual = LOGN((REAL)maxScaleResidual);
	for (const Component* pComponent : components) {
		if (pComponent->pairs.size() < minScaleComponent)
			break;
		ScaleComponentSolve solve;
		solve.pairs = pComponent->pairs;
		std::vector<REAL> scales, scaleResiduals;
		if (!RobustAverageScales(pComponent->equations, numPairs, NO_ID, maxLogScaleResidual, kRobustRounds, scales, scaleResiduals))
			continue;
		// Camera centres: C_2 - C_1 = scale * baseline of every pair, L1-reweighted
		std::vector<TranslationPair> offsets;
		std::vector<REAL> lengths;
		offsets.reserve(solve.pairs.size());
		lengths.reserve(solve.pairs.size());
		solve.scales.reserve(solve.pairs.size());
		for (const uint32_t idxPair : solve.pairs) {
			const ImagePair& pair = scene.pairs[idxPair];
			ASSERT(HasBaselineLength(baselines[idxPair]));
			solve.scales.push_back(scales[idxPair]);
			offsets.emplace_back(pair.ID1, pair.ID2, baselines[idxPair] * solve.scales.back(), weights[idxPair]);
			lengths.push_back(norm(offsets.back().relativeTranslation));
		}
		std::nth_element(lengths.begin(), lengths.begin() + lengths.size() / 2, lengths.end());
		const REAL floorLength = l1Floor * lengths[lengths.size() / 2];
		GlobalTranslationEstimator translationEstimator;
		if (!RobustAverageWith(offsets, numL1Rounds,
			[&](const std::vector<TranslationPair>& active, std::vector<Point3>& t) {
				// Estimator scales rows by weight (LS weight = weight^2), so take sqrt for L1
				std::vector<TranslationPair> rows(active);
				for (TranslationPair& row : rows)
					row.weight = SQRT(row.weight);
				return translationEstimator.EstimateTranslations(rows, numImages, NO_ID, t);
			},
			[&](size_t i, const std::vector<Point3>& t) {
				const TranslationPair& p = offsets[i];
				return norm(t[p.idxB] - t[p.idxA] - p.relativeTranslation);
			},
			[&](REAL r) { return (float)(REAL(1) / MAXF(r, floorLength)); },
			solve.centres, solve.centreResiduals))
			continue;
		solve.placed.assign(numImages, false);
		for (const TranslationPair& p : offsets)
			solve.placed[p.idxA] = solve.placed[p.idxB] = true;
		solves.push_back(std::move(solve));
	}
	return solves;
}

namespace {

// Translation stage of the contradiction cut: solve pair scales and camera centres (SolveScaleComponents)
// on kept pairs, then mark pairs whose baseline direction exceeds maxDirectionResidual as CUT (never LINK).
unsigned RemoveTranslationContradictions(const Scene& scene, const std::vector<Point3>& baselines, const std::vector<float>& scores,
	float minPairWeight, float maxDirectionResidual, std::vector<PairState>& states, std::vector<ScaleComponentSolve>& solves)
{
	TD_TIMER_START();
	const uint32_t numPairs = (uint32_t)scene.pairs.size();
	ASSERT(baselines.size() == numPairs && scores.size() == numPairs && states.size() == numPairs);
	IIndexArr judged, solved;
	std::vector<float> weights(numPairs, 0.f);
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (IsRemoved(states[idxPair]) || !HasBaselineDirection(pair, baselines[idxPair]))
			continue;
		judged.push_back((IIndex)idxPair);
		if (pair.ExceedsWeightBar(minPairWeight) && scores[idxPair] != 0.f) {
			solved.push_back((IIndex)idxPair);
			weights[idxPair] = pair.GetCompositeWeight() * (scores[idxPair] < 0.f ? 1.f : scores[idxPair]);
		}
	}
	const std::vector<ScalePair> equations = ComputePairScales(scene, solved);
	unsigned numScaleComponents;
	solves = SolveScaleComponents(scene, equations, baselines, weights, numScaleComponents);

	// Track each pair's owning solve (if any) in O(1); a component's own pairs use its solve,
	// other judged pairs use the largest solve placing both images
	std::vector<uint32_t> ownerSolve(numPairs, NO_ID);
	FOREACH(s, solves)
		for (const uint32_t idxPair : solves[s].pairs)
			ownerSolve[idxPair] = (uint32_t)s;

	std::vector<REAL> angles(numPairs, std::numeric_limits<REAL>::quiet_NaN());
	FOREACH(s, solves) {
		const ScaleComponentSolve& solve = solves[s];
		for (const IIndex idxPair : judged) {
			const ImagePair& pair = scene.pairs[idxPair];
			const bool own = ownerSolve[idxPair] == (uint32_t)s;
			if (!own && (ownerSolve[idxPair] != NO_ID || !std::isnan(angles[idxPair]) || !solve.placed[pair.ID1] || !solve.placed[pair.ID2]))
				continue;
			const Point3 centresOffset = solve.centres[pair.ID2] - solve.centres[pair.ID1];
			if (norm(centresOffset) <= ZEROTOLERANCE<REAL>())
				continue;
			angles[idxPair] = DirectionAngleDeg(baselines[idxPair], centresOffset);
		}
	}
	unsigned numJudged = 0, numRemoved = 0, numProtected = 0;
	for (const IIndex idxPair : judged) {
		if (std::isnan(angles[idxPair]))
			continue;
		++numJudged;
		if (angles[idxPair] <= maxDirectionResidual)
			continue;
		if (states[idxPair] == PairState::LINK) {
			++numProtected;
			continue;
		}
		states[idxPair] = PairState::CUT;
		++numRemoved;
	}
	REAL medianResidual = 0;
	if (!solves.empty()) {
		std::vector<REAL> residuals(solves.front().centreResiduals);
		std::nth_element(residuals.begin(), residuals.begin() + residuals.size() / 2, residuals.end());
		medianResidual = residuals[residuals.size() / 2];
	}
	VERBOSE("Translation cut: removed %u/%u pairs whose baseline direction is more than %g degrees from the centres of %u of %u scale components "
		"(%u pairs scaled by %u equations, median residual of the largest %.3g) (%s)",
		numRemoved, numJudged, maxDirectionResidual, (unsigned)solves.size(), numScaleComponents, (unsigned)solved.size(), (unsigned)equations.size(),
		medianResidual, TD_TIMER_GET_FMT().c_str());
	if (numProtected != 0)
		VERBOSE("Translation cut: kept %u link(s) of the sequence chain it would have removed", numProtected);
	return numProtected;
}

// Fit unit baseline direction t (x_2 = R x_1 + t) to the first numMatches bearing pairs at fixed rotation R.
// RANSAC over pairs of epipolar planes, refined by least squares and oriented by cheirality.
bool FitBaselineDirection(const Eigen::Matrix3d& R, const std::vector<Eigen::Vector3d>& bearings1,
	const std::vector<Eigen::Vector3d>& bearings2, unsigned numMatches, unsigned seed, Eigen::Vector3d& t, std::vector<char>& agree)
{
	ASSERT(bearings1.size() >= numMatches && bearings2.size() >= numMatches);
	if (numMatches < kRepairMinMatches)
		return false;
	std::vector<Eigen::Vector3d> rays(numMatches), planes(numMatches);
	for (unsigned i = 0; i < numMatches; ++i) {
		rays[i] = R * bearings1[i];
		planes[i] = rays[i].cross(bearings2[i]).normalized();
	}
	const REAL maxSin = SIN(D2R(kRepairAngle));
	const auto agrees = [&](unsigned i, const Eigen::Vector3d& direction) { return ABS(planes[i].dot(direction)) <= maxSin; };
	std::mt19937 rng(seed);
	std::uniform_int_distribution<unsigned> pick(0, numMatches - 1);
	Eigen::Vector3d best(Eigen::Vector3d::Zero());
	unsigned numBest = 0;
	for (unsigned round = 0; round < kRepairRounds; ++round) {
		Eigen::Vector3d direction = planes[pick(rng)].cross(planes[pick(rng)]);
		if (direction.norm() <= ZEROTOLERANCE<double>())
			continue;
		direction.normalize();
		unsigned numAgreeing = 0;
		for (unsigned i = 0; i < numMatches; ++i)
			numAgreeing += agrees(i, direction);
		if (numAgreeing > numBest) {
			numBest = numAgreeing;
			best = direction;
		}
	}
	if (numBest < kRepairMinMatches || numBest < kRepairMinShare * numMatches)
		return false;
	Eigen::Matrix3d M(Eigen::Matrix3d::Zero());
	for (unsigned i = 0; i < numMatches; ++i)
		if (agrees(i, best))
			M += planes[i] * planes[i].transpose();
	t = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(M).eigenvectors().col(0);
	int numFront = 0;
	for (unsigned i = 0; i < numMatches; ++i) {
		if (!agrees(i, t))
			continue;
		const double a12 = -rays[i].dot(bearings2[i]), det = 1 - a12 * a12;
		if (det <= 1e-12)
			continue;
		const double r1 = -rays[i].dot(t), r2 = bearings2[i].dot(t);
		const double d1 = (r1 - a12 * r2) / det, d2 = (r2 - a12 * r1) / det;
		numFront += (d1 > 0 && d2 > 0) - (d1 < 0 && d2 < 0);
	}
	if (numFront < 0)
		t = -t;
	agree.resize(numMatches);
	for (unsigned i = 0; i < numMatches; ++i)
		agree[i] = agrees(i, t);
	return true;
}

// Repair CUT pairs whose matches agree on a baseline direction at the averaged rotation confirmed by
// the translation-stage centres and vouched for by the same spot or a common third-spot neighbour.
unsigned RepairContradictedPairs(Scene& scene, const std::vector<Point3>& rotations,
	const std::vector<ScaleComponentSolve>& solves, std::vector<PairState>& states)
{
	TD_TIMER_START();
	ASSERT(states.size() == scene.pairs.size() && rotations.size() == scene.images.size());
	const uint32_t numImages = (uint32_t)scene.images.size();
	DisjointSet<uint32_t> spots(numImages);
	std::vector<IIndexArr> neighbours(numImages);
	IIndexArr candidates;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (states[idxPair] == PairState::VETOED)
			continue;
		if (pair.relativePose.has_value() && !(pair.meanRayAngle >= D2R(kNearDuplicateRayAngle)))
			spots.Union(pair.ID1, pair.ID2);
		if (states[idxPair] == PairState::CUT) {
			ASSERT(pair.relativePose.has_value());
			candidates.push_back((IIndex)idxPair);
		} else {
			neighbours[pair.ID1].push_back(pair.ID2);
			neighbours[pair.ID2].push_back(pair.ID1);
		}
	}
	std::vector<uint32_t> spotOf(numImages);
	for (uint32_t i = 0; i < numImages; ++i) {
		spotOf[i] = spots.Find(i);
		std::sort(neighbours[i].begin(), neighbours[i].end());
	}
	struct Repair { Pose3D pose; std::vector<char> agree; bool repaired{false}; };
	std::vector<Repair> repairs(candidates.size());
	#pragma omp parallel for schedule(dynamic, 8)
	for (int_t k = 0; k < (int_t)candidates.size(); ++k) {
		const ImagePair& pair = scene.pairs[candidates[k]];
		Repair& repair = repairs[k];
		bool vouched = spotOf[pair.ID1] == spotOf[pair.ID2];
		if (!vouched) {
			IIndexArr common;
			std::set_intersection(neighbours[pair.ID1].begin(), neighbours[pair.ID1].end(),
				neighbours[pair.ID2].begin(), neighbours[pair.ID2].end(), std::back_inserter(common));
			for (const IIndex c : common)
				if (spotOf[c] != spotOf[pair.ID1] && spotOf[c] != spotOf[pair.ID2]) {
					vouched = true;
					break;
				}
		}
		if (!vouched)
			continue;
		const ScaleComponentSolve* pSolve = NULL;
		for (const ScaleComponentSolve& solve : solves)
			if (solve.placed[pair.ID1] && solve.placed[pair.ID2]) {
				pSolve = &solve;
				break;
			}
		if (pSolve == NULL)
			continue;
		const Point3 centresOffset = pSolve->centres[pair.ID2] - pSolve->centres[pair.ID1];
		if (norm(centresOffset) <= ZEROTOLERANCE<REAL>())
			continue;
		const RMatrix R1(rotations[pair.ID1]);
		const Matrix3x3 R = RMatrix(rotations[pair.ID2]) * R1.t();
		const Eigen::Matrix3d eR(R);
		const auto [bearings1, bearings2] = pair.GetMatchedBearings(scene.images[pair.ID1], scene.images[pair.ID2], true);
		Eigen::Vector3d t;
		if (!FitBaselineDirection(eR, bearings1, bearings2, pair.GetNumTrackFormingMatches(), candidates[k], t, repair.agree))
			continue;
		const Point3 C(-(eR.transpose() * t));
		repair.pose = Pose3D(RMatrix(R), CMatrix(C));
		repair.repaired = DirectionAngleDeg(R1.t() * C, centresOffset) <= kRepairMaxDirection;
	}
	unsigned numRepaired = 0;
	FOREACH(k, candidates) {
		const Repair& repair = repairs[k];
		if (!repair.repaired)
			continue;
		ImagePair& pair = scene.pairs[candidates[k]];
		const Image& image1 = scene.images[pair.ID1];
		const Image& image2 = scene.images[pair.ID2];
		const unsigned numSparse = pair.GetNumFilteredInliers(), numTrackForming = pair.GetNumTrackFormingMatches();
		ASSERT(repair.agree.size() == numTrackForming);
		std::vector<DMatch> sparse, dense;
		for (unsigned i = 0; i < numTrackForming; ++i) {
			if (repair.agree[i])
				(i < numSparse ? sparse : dense).push_back(pair.matches[i]);
			else if (i < numSparse)
				pair.outlierMatches.push_back(pair.matches[i]);
		}
		pair.outlierMatches.insert(pair.outlierMatches.end(), pair.matches.begin() + numTrackForming, pair.matches.end());
		pair.numFilteredInliers = (int)sparse.size();
		pair.numDenseInliers = (int)dense.size();
		pair.matches = std::move(sparse);
		pair.matches.insert(pair.matches.end(), dense.begin(), dense.end());
		pair.weightedInliers = -1.f;
		pair.returnedByRepair = true;
		pair.relativePose = repair.pose;
		pair.E = ImagePair::ComposeEssentialMatrix(repair.pose);
		if (image1.pCamera->GetType() == CameraType::PINHOLE && image2.pCamera->GetType() == CameraType::PINHOLE)
			pair.F = ImagePair::ComposeFundamentalMatrix(pair.E.value(), image1.GetK(), image2.GetK());
		pair.meanRayAngle = pair.ComputeMeanRayAngle(image1, image2);
		states[candidates[k]] = PairState::KEPT;
		++numRepaired;
	}
	VERBOSE("Contradiction cut: repaired %u of the %u removed pairs (%s)", numRepaired, (unsigned)candidates.size(), TD_TIMER_GET_FMT().c_str());
	return numRepaired;
}

} // namespace
/*----------------------------------------------------------------*/

unsigned SFM::FilterPairsByContradiction(Scene& scene, const std::vector<float>& scores,
	float minPairWeight, float maxDirectionResidual, const PairsWeightingConfig& weightingCfg, bool repair,
	const SequenceNeighbours* neighbours, SequenceFilterStats* stats)
{
	ASSERT(scores.size() == scene.pairs.size());
	ASSERT(maxDirectionResidual >= 0 && maxDirectionResidual <= 180);
	ASSERT(!repair || maxDirectionResidual > 0);
	TD_TIMER_STARTD();
	std::vector<RotationPair> rotationPairs;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.relativePose.has_value() || !pair.ExceedsWeightBar(minPairWeight))
			continue;
		const float weight = pair.GetCompositeWeight() * (scores[idxPair] < 0.f ? 1.f : scores[idxPair]);
		if (weight > 0.f)
			rotationPairs.emplace_back(pair.ID1, pair.ID2, pair.relativePose->R, weight);
	}
	const GlobalRotationEstimatorOptions options;
	const uint32_t numImages = (uint32_t)scene.images.size();
	std::vector<Point3> rotations;
	std::vector<uint32_t> componentOf;
	const unsigned numComponents = GlobalRotationEstimator(options).EstimateAllComponents(rotationPairs, numImages, rotations, componentOf);
	const std::vector<Point3> baselines = ComputeWorldBaselines(scene, rotations, componentOf);
	const REAL minCosAngle = COS(D2R(options.maxRelativeRotationAngle));
	std::vector<PairState> states(scene.pairs.size(), PairState::KEPT);
	unsigned numJudged = 0, numCut = 0;
	float maxRemovedWeight = 0.f;
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!HasBaseline(baselines[idxPair]))
			continue;
		++numJudged;
		const Matrix3x3 solvedR = RMatrix(rotations[pair.ID2]) * RMatrix(rotations[pair.ID1]).t();
		if (ComputeAngle(pair.relativePose->R, solvedR) >= minCosAngle)
			continue;
		states[idxPair] = PairState::CUT;
		maxRemovedWeight = MAXF(maxRemovedWeight, pair.GetCompositeWeight());
		++numCut;
	}
	VERBOSE("Contradiction cut: removed %u/%u pairs more than %g degrees from the rotations of the weighted averaging of %u usable components (heaviest %.1f)",
		numCut, numJudged, options.maxRelativeRotationAngle, numComponents, maxRemovedWeight);
	SequenceFilterStats sequence;
	unsigned numRepaired = 0;
	if (maxDirectionResidual > 0) {
		if (neighbours != nullptr)
			ApplySequenceChain(scene, baselines, *neighbours, states, sequence);
		std::vector<ScaleComponentSolve> solves;
		sequence.numProtected = RemoveTranslationContradictions(scene, baselines, scores, minPairWeight, maxDirectionResidual, states, solves);
		if (repair && std::find(states.begin(), states.end(), PairState::CUT) != states.end())
			numRepaired = RepairContradictedPairs(scene, rotations, solves, states);
	}
	if (stats != nullptr)
		*stats = sequence;
	const unsigned numRemoved = (unsigned)std::count_if(states.begin(), states.end(), IsRemoved);
	if (numRemoved == 0 && numRepaired == 0)
		return 0;
	const unsigned numPairs = scene.pairs.size();
	unsigned numKept = 0;
	for (unsigned idxPair = 0; idxPair < numPairs; ++idxPair) {
		if (IsRemoved(states[idxPair]))
			continue;
		if (numKept != idxPair)
			scene.pairs[numKept] = std::move(scene.pairs[idxPair]);
		++numKept;
	}
	ASSERT(numPairs - numKept == numRemoved);
	scene.pairs.RemoveLast(numRemoved);
	ComputePairsWeights(scene, weightingCfg);
	DEBUG("Filtered the view graph by contradiction with its %s averaging: %u pairs removed (%s)",
		maxDirectionResidual > 0 ? "rotation and translation" : "rotation", numRemoved, TD_TIMER_GET_FMT().c_str());
	return numRemoved;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
