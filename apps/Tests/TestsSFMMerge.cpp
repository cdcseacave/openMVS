/*
 * TestsSFMMerge.cpp
 *
 * Copyright (c) 2014-2025 SEACAVE
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
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "../../libs/SFM.h"
#include "../../libs/SFM/GlobalRotationAveraging.h"
#include "../../libs/SFM/GlobalScaleAveraging.h"
#include "../../libs/SFM/GlobalTranslationAveraging.h"
#include "../../libs/Math/DisjointSet.h"
#include "../../libs/SFM/RobustAveraging.h"
#include "../../libs/SFM/PairsWeighting.h"
#include "../../libs/SFM/ViewGraphCalibrator.h"
#include "../../libs/SFM/BundleAdjustment.h"
#include "../../libs/SFM/SceneCluster.h"
#include "../../libs/SFM/SceneBlocks.h"
#include "../../libs/SFM/GlobalAlignment.h"
#include "../../libs/SFM/PoseLink.h"
#include "../../libs/SFM/Link2D3D.h"
#include "../../libs/SFM/Triangulation.h"
#include "../../libs/SFM/MatchGeometric.h"
#include "../../libs/SFM/LensFit.h"
#include "../../libs/SFM/RoMa2Matcher.h"
#include "../../libs/SFM/FlexGlueMatcher.h"
#include "../../libs/Math/Camera/CameraUndistort.h"
#include "../../libs/SFM/InterfaceMVS.h"
#include "../../libs/MVS.h"
#include <nlohmann/json.hpp>
#include "Tests.h"
#include "TestsSFMFramework.h"
#include <filesystem>
#include <chrono>
#include <random>


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("TestSFM "));

namespace SFM {


// ===============================================================================
// Phase 3: Global Alignment Tests
// ===============================================================================


// A block graph for the selection alone: blocks at random true frames, and hypotheses made from the exact
// relative similarity of their two blocks, turned or shifted in block A's frame by what is planted in them.
// A clean graph has blocks of unit scale (so a shift reads the same share of the seam's spread from either end);
// a noisy one keeps the random scales and gives every hypothesis what a reconstruction's seam is off by:
// a turn about a random axis (the planted angle plus up to 0.3 deg) and up to 1% of scale
struct PlantedSeamGraph
{
	// the spread of every reading's seam cameras, the unit a planted shift is a share of; the seam's centre is
	// the origin of block A's frame, the point the planted turn leaves in place
	static constexpr REAL kExtent = 20;
	// how far a block may sit from where the truth puts it against the block it is read from
	static constexpr REAL kPlacedRotation = 1, kPlacedShift = 0.02;
	const bool noisy;
	std::mt19937 rng;
	std::uniform_real_distribution<REAL> noiseDist{-1, 1};
	std::vector<SEACAVE::Transform> G;
	std::vector<SeamCandidate> candidates;
	std::vector<BlockPose> poses;
	std::vector<uint32_t> edgeSeams;

	PlantedSeamGraph(uint32_t numBlocks, uint32_t seed, bool noisy = false) : noisy(noisy), rng(seed), G(numBlocks) {
		for (SEACAVE::Transform& T : G) {
			T = SEACAVE::Transform::Random(rng, 180, 10, 0.5);
			if (!noisy)
				T.scale = 1;
		}
	}
	// a reading of (a, b) held by `cameras` cameras on either side, turned by `rotationDeg` about the
	// vertical of block A's frame through the seam's centre and shifted along its x axis by `shift` of the
	// seam's spread (a noisy graph turns about a random axis and shifts nothing); the seam's centre is the
	// origin of block A's frame, or the given point of the true frame; returns its index
	size_t Add(uint32_t a, uint32_t b, unsigned cameras, REAL rotationDeg = 0, REAL shift = 0, const Point3* seamCentre = NULL) {
		ASSERT(a < b && b < G.size() && (!noisy || (shift == 0 && seamCentre == NULL)));
		SeamCandidate c;
		c.sceneA = a;
		c.sceneB = b;
		c.extent = kExtent;
		if (seamCentre != NULL)
			c.centre = G[a].Invert() * (*seamCentre);
		if (noisy) {
			c.T = G[b].Invert() * G[a];
			Point3 axis(noiseDist(rng), noiseDist(rng), noiseDist(rng));
			axis *= D2R(rotationDeg + REAL(0.3) * noiseDist(rng)) / norm(axis);
			RMatrix noiseR;
			noiseR.SetRotationAxisAngle(axis);
			c.T.R = RMatrix(noiseR * c.T.R);
			c.T.scale *= REAL(1) + REAL(0.01) * noiseDist(rng);
		} else {
			SEACAVE::Transform planted;
			planted.R.SetRotationAxisAngle(Point3(0, 0, D2R(rotationDeg)));
			planted.t = c.centre - planted.R * c.centre + Point3(shift * kExtent, 0, 0);
			c.T = G[b].Invert() * G[a] * planted;
		}
		c.score.support[0] = c.score.support[1] = cameras;
		c.weight = GlobalAlignment::ReadingWeight(c.score, c.pairs);
		candidates.push_back(std::move(c));
		return candidates.size() - 1;
	}
	// the planted readings that pass the entry test (each observes a scale) into the block graph, as
	// EstimateSeamPair lets them in; edgeSeams is mapped back to indices into `candidates`, which get
	// the residuals the selection measured
	void Select() {
		Scene scene;
		const GlobalAlignmentConfig alignCfg;
		GlobalAlignment alignment(scene, alignCfg);
		std::vector<SeamCandidate> entered;
		std::vector<size_t> indexOf;
		FOREACH(i, candidates)
			if (GlobalAlignment::RefusedReading(candidates[i].score, true, candidates[i].pairs, alignCfg) == NULL) {
				entered.push_back(candidates[i]);
				indexOf.push_back(i);
			}
		alignment.SelectSeamGraph((uint32_t)G.size(), entered, poses, edgeSeams);
		FOREACH(k, entered)
			candidates[indexOf[k]] = entered[k];
		for (uint32_t& e : edgeSeams)
			e = (uint32_t)indexOf[e];
	}
	bool IsOn(size_t i) const { return std::find(edgeSeams.begin(), edgeSeams.end(), (uint32_t)i) != edgeSeams.end(); }
	// the angle in degrees between the placement of block b against block a and the truth's, and the scale
	// ratio (placed over true)
	REAL Placement(uint32_t a, uint32_t b, REAL* scaleRatio = NULL) const {
		const SEACAVE::Transform estimated(poses[b].T.Invert() * poses[a].T), truth(G[b].Invert() * G[a]);
		if (scaleRatio != NULL)
			*scaleRatio = estimated.scale / truth.scale;
		return RotationErrorDeg(estimated.R, truth.R);
	}
	// whether block `b` sits in the same model as block `a`, where the truth puts it against it
	bool PlacedRight(uint32_t a, uint32_t b) const {
		if (poses[a].state != BlockPose::ADMITTED || poses[b].state != BlockPose::ADMITTED || poses[a].model != poses[b].model)
			return false;
		const SEACAVE::Transform E((G[b].Invert() * G[a]).Invert() * (poses[b].T.Invert() * poses[a].T));
		return Placement(a, b) <= kPlacedRotation && norm(E.t) <= kPlacedShift * kExtent;
	}
	// the blocks of the model block `a` is in
	unsigned ModelSize(uint32_t a) const {
		unsigned num = 0;
		for (const BlockPose& pose : poses)
			num += pose.state == BlockPose::ADMITTED && pose.model == poses[a].model ? 1 : 0;
		return num;
	}
};

// The selection of the seam graph over planted block graphs, one arm each:
// 1. twelve blocks in a ring with chords, a false chord (60 degrees off, as strong as a true seam), a weaker rival of a
//    covered pair and two pairs joined to nothing else: every true reading stays, the wrong ones are cut, the lone pairs
//    stand and each component is placed in its own frame;
// 2. block 3 tied to a triangle by two readings of (2, 3) within the graph bars of each other, 20 cameras on the weaker
//    side of the true one and 2 on one 3 degrees and 0.1 of the seam's spread off: a reading weighs its cameras times its
//    pairs' share, so the averaging puts block 3 within a quarter of the offset from the truth (weighed by the share
//    alone the two weigh alike and block 3 lands halfway);
// 3. a chord wrong by 30 degrees and heavier than any true seam, across two cycles of true seams (two seams and five):
//    the two seams the chord would put off together outweigh it and it stays off, where a solve that shares its error
//    out charges the short cycle's two seams most and sacrifices them one after the other;
// 4. a bridge (2, 3) of two directions, alike (both stand and carry block 3) or one turned by ten degrees past the
//    rotation bar (the averaging follows the heavier and the turned one is cut);
// 5. both directions of the bridge reading block 3 turned by 90 degrees about the vertical, as a symmetric facade gives
//    it: their own cycle cannot tell, the verified pairs across can: held against it the bridge goes off and block 3 is
//    a model of its own, held for it the bridge stands;
// 6. the true reading of a pair no cycle tests, its weaker side held by one camera and then by two: it enters the block
//    graph only with minSeamWeight cameras on its weaker side;
// 7. the true reading of (2, 3) and a lighter copy shifted by 60% of the seam's spread, past the 0.20 bar of either: a
//    least-squares solve puts block 3 between the two, the inverse-error reweighting places it on the true one and cuts
//    the copy;
// 8. four readings along one line at 0, 0.10, 0.25 and 0.50 of the seam's spread weighing 1, 2, 1, 2 cameras: the first
//    averaging settles on 0.25 and cuts the two 0.25 away from it, and the next settles on 0.10 where the one at 0 is
//    within the bar again and is kept;
// 9. a triangle whose seams lie 50 spreads from every block's origin, each reading turned by 3 degrees about its own
//    seam's centre: a reading is averaged and judged where it was measured, so the three stay on, each within 0.05 of
//    its spread of the solution (read at a frame's origin, the degree the averaging leaves of each turn is a spread there).
bool SeamGraphSelectionTest()
{
	TestScope scope("SeamGraphSelectionTest");
	constexpr unsigned TRIANGLE_CAMERAS = 5;
	// the triangle of true seams 0-1-2 the bridge arms tie block 3 to
	const auto Triangle = [](PlantedSeamGraph& graph) {
		graph.Add(0, 1, TRIANGLE_CAMERAS);
		graph.Add(1, 2, TRIANGLE_CAMERAS);
		graph.Add(0, 2, TRIANGLE_CAMERAS);
	};

	// 1. the ring with chords, a false chord, a rival and two lone pairs
	{
		constexpr uint32_t RING_BLOCKS = 12, NUM_BLOCKS = 16;
		PlantedSeamGraph graph(NUM_BLOCKS, 9137, true);
		for (uint32_t i = 0; i < RING_BLOCKS; ++i) {
			const uint32_t next = (i + 1) % RING_BLOCKS, next2 = (i + 2) % RING_BLOCKS;
			graph.Add(MINF(i, next), MAXF(i, next), 20);
			graph.Add(MINF(i, next2), MAXF(i, next2), 20);
		}
		const size_t numTrueSeams = graph.candidates.size();
		const size_t falseChord = graph.Add(2, 8, 20, 60);    // a chord neither block ever saw, 60 degrees from the truth
		const size_t rival = graph.Add(5, 6, 6, 20);          // a weaker hypothesis on a pair a true seam covers
		const size_t lonePair = graph.Add(12, 13, 20);        // two pairs joined to nothing else
		graph.Add(14, 15, 20);
		graph.Select();
		for (size_t i = 0; i < numTrueSeams; ++i)
			TEST_CHECK(graph.IsOn(i), "ring: the true seam (%u, %u) is off: rotation %.2f deg, scale %.3f, translation %.3f",
				graph.candidates[i].sceneA, graph.candidates[i].sceneB, graph.candidates[i].residualRotation,
				graph.candidates[i].residualScale, graph.candidates[i].residualTranslation);
		TEST_CHECK(!graph.IsOn(falseChord) && graph.candidates[falseChord].residualRotation > 30.f,
			"ring: the false chord is %s with rotation residual %.2f deg (off and over 30 expected)",
			graph.IsOn(falseChord) ? "on" : "off", graph.candidates[falseChord].residualRotation);
		TEST_CHECK(!graph.IsOn(rival), "ring: the wrong rival on (5, 6) carries its edge");
		TEST_CHECK(graph.IsOn(lonePair) && graph.IsOn(lonePair + 1), "ring: the lone pairs' readings are %s and %s (both on expected)",
			graph.IsOn(lonePair) ? "on" : "off", graph.IsOn(lonePair + 1) ? "on" : "off");
		// the ring and the two lone pairs are three models of several blocks
		std::map<uint32_t, unsigned> blocksOfModel;
		for (const BlockPose& pose : graph.poses)
			++blocksOfModel[pose.model];
		const auto ModelOf = [&](uint32_t block) {
			return graph.poses[block].state == BlockPose::ADMITTED && blocksOfModel[graph.poses[block].model] > 1 ?
				graph.poses[block].model : (uint32_t)NO_ID;
		};
		for (uint32_t i = 0; i < NUM_BLOCKS; ++i)
			TEST_CHECK(ModelOf(i) != NO_ID, "ring: block %u is alone, not in a model of several", i);
		for (uint32_t i = 1; i < RING_BLOCKS; ++i)
			TEST_CHECK(ModelOf(i) == ModelOf(0), "ring: ring block %u is in model %u, block 0 in %u", i, ModelOf(i), ModelOf(0));
		TEST_CHECK(ModelOf(12) == ModelOf(13) && ModelOf(14) == ModelOf(15) && ModelOf(12) != ModelOf(0) &&
			ModelOf(14) != ModelOf(0) && ModelOf(12) != ModelOf(14), "ring: the two lone pairs are not models of their own");
		for (const uint32_t a : {12u, 14u})
			TEST_CHECK(graph.Placement(a, a + 1) <= 1, "ring: the lone pair (%u, %u) placed %.2f deg off (1 allowed)", a, a + 1, graph.Placement(a, a + 1));
		for (uint32_t i = 0; i < RING_BLOCKS; ++i) {
			const uint32_t j = (i + 1) % RING_BLOCKS;
			REAL scaleRatio;
			const REAL angle = graph.Placement(i, j, &scaleRatio);
			TEST_CHECK(angle <= 1 && MAXF(scaleRatio, REAL(1) / scaleRatio) <= 1.03, "ring: blocks (%u, %u) placed %.2f deg and %.1f%% off "
				"(1 deg and 3%% allowed)", i, j, angle, (MAXF(scaleRatio, REAL(1) / scaleRatio) - 1) * 100);
		}
	}

	// 2. two readings of one pair, weighed by their cameras
	{
		constexpr unsigned STRONG = 20, WEAK = 2;
		constexpr REAL OFF_DEG = 3, OFF_SHIFT = 0.1;
		PlantedSeamGraph graph(4, 3301);
		Triangle(graph);
		const size_t strong = graph.Add(2, 3, STRONG), weak = graph.Add(2, 3, WEAK, OFF_DEG, OFF_SHIFT);
		graph.Select();
		TEST_CHECK(graph.IsOn(strong) && graph.IsOn(weak) && graph.ModelSize(0) == 4,
			"weights: premise, the strong reading is %s, the weak one %s, the model holds %u blocks (both on, 4 expected)",
			graph.IsOn(strong) ? "on" : "off", graph.IsOn(weak) ? "on" : "off", graph.ModelSize(0));
		const REAL angle = graph.Placement(2, 3);
		TEST_CHECK(angle <= OFF_DEG / 4, "weights: block 3 sits %.2f deg from the truth (under %.2f expected: the reading of %u cameras "
			"outweighs the one of %u, %.0f deg off)", angle, OFF_DEG / 4, STRONG, WEAK, OFF_DEG);
	}

	// 3. a heavy chord wrong by 30 degrees across a short and a long cycle
	{
		constexpr unsigned TRUE_CAMERAS = 4, CHORD_CAMERAS = 7; // the chord heavier than one true seam, lighter than two
		constexpr REAL CHORD_DEG = 30;
		PlantedSeamGraph graph(7, 4513);
		const std::pair<uint32_t, uint32_t> trueSeams[] = {{0, 1}, {1, 2}, {0, 3}, {3, 4}, {4, 5}, {5, 6}, {2, 6}};
		for (const auto& [a, b] : trueSeams)
			graph.Add(a, b, TRUE_CAMERAS);
		const size_t chord = graph.Add(0, 2, CHORD_CAMERAS, CHORD_DEG);
		graph.Select();
		TEST_CHECK(!graph.IsOn(chord), "chord: the chord wrong by %.0f deg is on", CHORD_DEG);
		for (size_t i = 0; i < chord; ++i)
			TEST_CHECK(graph.IsOn(i), "chord: the true seam (%u, %u) is off", graph.candidates[i].sceneA, graph.candidates[i].sceneB);
		for (uint32_t b = 1; b < 7; ++b)
			TEST_CHECK(graph.PlacedRight(0, b), "chord: block %u is not placed on the truth against block 0", b);
	}

	// 4. a bridge whose two directions agree, or one of them turned past the rotation bar
	{
		constexpr unsigned BRIDGE_CAMERAS = 4, TURNED_CAMERAS = 3;
		constexpr REAL TURN_DEG = 10;
		for (const bool turned : {false, true}) {
			PlantedSeamGraph graph(4, 6151);
			Triangle(graph);
			const size_t forward = graph.Add(2, 3, BRIDGE_CAMERAS);
			const size_t backward = turned ? graph.Add(2, 3, TURNED_CAMERAS, TURN_DEG) : graph.Add(2, 3, BRIDGE_CAMERAS);
			graph.Select();
			TEST_CHECK(graph.IsOn(forward) && graph.IsOn(backward) != turned,
				"bridge: with the second direction %s, the first reading is %s and the second %s (on and %s expected)",
				turned ? "turned by 10 deg" : "agreeing", graph.IsOn(forward) ? "on" : "off", graph.IsOn(backward) ? "on" : "off",
				turned ? "off" : "on");
			TEST_CHECK(graph.PlacedRight(0, 3), "bridge: with the second direction %s, block 3 is not placed on the truth",
				turned ? "turned by 10 deg" : "agreeing");
		}
	}

	// 5. a symmetric copy read by both directions, the verified pairs against it or for it
	{
		constexpr unsigned BRIDGE_CAMERAS = 4;
		constexpr REAL COPY_DEG = 90;
		constexpr float HEAVY = 400.f, LIGHT = 100.f; // inlier weight of the verified pairs for and against
		for (const bool against : {true, false}) {
			PlantedSeamGraph graph(4, 7027);
			Triangle(graph);
			const size_t readings[2] = {graph.Add(2, 3, BRIDGE_CAMERAS, COPY_DEG), graph.Add(2, 3, BRIDGE_CAMERAS, COPY_DEG)};
			for (const size_t i : readings) {
				SeamCandidate& c = graph.candidates[i];
				c.pairs.numPairs = 4;
				c.pairs.agree = against ? LIGHT : HEAVY;
				c.pairs.disagree = against ? HEAVY : LIGHT;
				c.weight = (float)c.score.Weight() * c.pairs.Share();
			}
			graph.Select();
			const bool on = graph.IsOn(readings[0]) && graph.IsOn(readings[1]);
			const bool off = !graph.IsOn(readings[0]) && !graph.IsOn(readings[1]);
			TEST_CHECK(against ? off : on, "symmetric: with the verified pairs %s the 90-deg bridge, its readings are %s and %s",
				against ? "against" : "for", graph.IsOn(readings[0]) ? "on" : "off", graph.IsOn(readings[1]) ? "on" : "off");
			const bool inModel = graph.poses[3].model == graph.poses[0].model;
			TEST_CHECK(inModel != against && graph.ModelSize(0) == (against ? 3u : 4u), "symmetric: with the verified pairs %s the bridge, "
				"block 3 is %s the triangle's model of %u blocks (%u expected)", against ? "against" : "for", inModel ? "in" : "out of",
				graph.ModelSize(0), against ? 3u : 4u);
		}
	}

	// 6. a reading whose weaker side is one camera, then two
	{
		constexpr unsigned STRONG_SIDE = 6;
		for (const unsigned weakSide : {1u, 2u}) {
			PlantedSeamGraph graph(4, 8089);
			Triangle(graph);
			const size_t bridge = graph.Add(2, 3, STRONG_SIDE);
			SeamCandidate& c = graph.candidates[bridge];
			c.score.support[1] = weakSide;
			c.weight = (float)c.score.Weight();
			graph.Select();
			const bool expected = weakSide >= GlobalAlignmentConfig().minSeamWeight;
			TEST_CHECK(graph.IsOn(bridge) == expected && graph.PlacedRight(0, 3) == expected,
				"one camera: the bridge whose weaker side holds %u camera(s) is %s and block 3 %s (%s expected)", weakSide,
				graph.IsOn(bridge) ? "on" : "off", graph.PlacedRight(0, 3) ? "placed" : "not placed", expected ? "on and placed" : "off and not placed");
		}
	}

	// 7. the true reading and a lighter copy shifted past the bar of either
	{
		constexpr unsigned TRUE_CAMERAS = 5, COPY_CAMERAS = 4;
		constexpr REAL COPY_SHIFT = 0.6;
		PlantedSeamGraph graph(4, 9203);
		Triangle(graph);
		const size_t right = graph.Add(2, 3, TRUE_CAMERAS), copy = graph.Add(2, 3, COPY_CAMERAS, 0, COPY_SHIFT);
		graph.Select();
		TEST_CHECK(graph.IsOn(right) && !graph.IsOn(copy) && graph.PlacedRight(0, 3), "reweighting: the true reading is %s, the copy %s, "
			"block 3 %s (on, off and on the truth expected)", graph.IsOn(right) ? "on" : "off", graph.IsOn(copy) ? "on" : "off",
			graph.PlacedRight(0, 3) ? "on the truth" : "not on the truth");
	}

	// 8. four readings along a line: the one cut in the first round is within the bar of the second
	{
		constexpr unsigned WITNESSES = 5;
		const REAL shifts[4] = {0.0, 0.10, 0.25, 0.50};
		const unsigned weights[4] = {1, 2, 1, 2};
		PlantedSeamGraph graph(4, 9203);
		Triangle(graph);
		size_t readings[4];
		for (int i = 0; i < 4; ++i) {
			readings[i] = graph.Add(2, 3, weights[i], 0, shifts[i]);
			SeamCandidate& c = graph.candidates[readings[i]];
			c.score.support[0] = c.score.support[1] = WITNESSES; // every reading passes the entry test: only the bars decide
		}
		graph.Select();
		TEST_CHECK(graph.IsOn(readings[0]) && graph.IsOn(readings[1]) && graph.IsOn(readings[2]) && !graph.IsOn(readings[3]),
			"re-entry: the readings at 0, 0.10, 0.25 and 0.50 are %s, %s, %s and %s (on, on, on and off expected: the one at 0, cut "
			"in the first round, is within the bar of the second)", graph.IsOn(readings[0]) ? "on" : "off", graph.IsOn(readings[1]) ? "on" : "off",
			graph.IsOn(readings[2]) ? "on" : "off", graph.IsOn(readings[3]) ? "on" : "off");
	}
	// 9. readings turned about their own seams, far from the frames' origins
	{
		constexpr REAL TURN_DEG = 3, FAR = 50 * PlantedSeamGraph::kExtent, MAX_RESIDUAL = 0.05;
		PlantedSeamGraph graph(3, 9311);
		const std::pair<uint32_t, uint32_t> seams[] = {{0, 1}, {1, 2}, {0, 2}};
		const Point3 centres[] = {Point3(FAR, 10, 0), Point3(FAR - 10, -5, 5), Point3(FAR + 5, 0, -10)};
		for (int i = 0; i < 3; ++i)
			graph.Add(seams[i].first, seams[i].second, TRIANGLE_CAMERAS, TURN_DEG, 0, &centres[i]);
		graph.Select();
		for (size_t i = 0; i < 3; ++i) {
			const SeamCandidate& c = graph.candidates[i];
			TEST_CHECK(graph.IsOn(i) && c.residualTranslation <= MAX_RESIDUAL, "far seams: the reading (%u, %u), turned by %.0f deg about "
				"its seam %.0f spreads from the origin, is %s with %.2f deg and %.3f of its spread from the solution (on, within %.2f expected)",
				c.sceneA, c.sceneB, TURN_DEG, FAR / PlantedSeamGraph::kExtent, graph.IsOn(i) ? "on" : "off", c.residualRotation, c.residualTranslation, MAX_RESIDUAL);
		}
	}
	return scope.Pass("the selection keeps the true readings of a 16-block graph and cuts the false chord and the rival, weighs readings "
		"by cameras, cuts a heavy wrong chord across two cycles, tells a turned direction, a symmetric copy and a one-camera side, "
		"judges a cut reading again in the next round, and reads every reading at its own seam");
}
/*----------------------------------------------------------------*/

// Rotation of the world frame about the ring axis (z through the ring centre, the origin) by `deg`
static SEACAVE::Transform RingRotation(REAL deg)
{
	SEACAVE::Transform T;
	T.R.SetRotationAxisAngle(Point3(0, 0, D2R(deg)));
	return T;
}

// A block whose two halves were never seen together: the pairs that would have joined them are
// dropped before the block is split off, so its reconstruction has nothing tying one to the other
static void DropFoldPairs(Scene& scene, const IIndexArr& block, unsigned half)
{
	std::unordered_map<IIndex, unsigned> localOf;
	FOREACH(k, block)
		localOf.emplace(block[k], k);
	for (uint32_t i = 0; i < scene.pairs.size(); ++i) {
		const auto it1 = localOf.find(scene.pairs[i].ID1);
		const auto it2 = localOf.find(scene.pairs[i].ID2);
		if (it1 != localOf.end() && it2 != localOf.end() && (it1->second < half) != (it2->second < half))
			scene.pairs.RemoveAt(i--);
	}
}

// The internal drift of one block: camera k is turned about the ring axis and pushed away from the
// block's first camera along a ramp, so the block comes out bent and stretched in its own frame
static void BendRingBlock(const RingSceneConfig& cfg, Scene& sub)
{
	if (cfg.driftDegPerBlock == 0 && cfg.driftScalePerBlock == 0)
		return;
	const REAL steps = MAXF(REAL(cfg.camsPerBlock) - 1, REAL(1));
	const Point3 first(sub.images[0].C);
	FOREACH(k, sub.images) {
		Image& img = sub.images[k];
		const REAL ramp = REAL(k) / steps;
		RMatrix R;
		R.SetRotationAxisAngle(Point3(0, 0, D2R(cfg.driftDegPerBlock * ramp)));
		img.R = RMatrix(img.R * R.t());
		img.C = first + (R * img.C - first) * (REAL(1) + cfg.driftScalePerBlock * ramp);
	}
}

// A block reconstructed folded: it holds no track its two halves share, and its second half sits
// turned by `foldDeg` about the ring axis through the block's own centroid
static void FoldRingBlock(Scene& sub, unsigned half, REAL foldDeg)
{
	for (uint32_t t = sub.tracks.size(); t-- > 0; ) {
		bool inHalf[2] = {false, false};
		for (const Observation& obs : sub.tracks[t].observations)
			inHalf[obs.imageID < half ? 0 : 1] = true;
		if (inHalf[0] && inHalf[1])
			sub.tracks.RemoveAt(t);
	}
	Point3 centroid(Point3::ZERO);
	for (const Image& img : sub.images)
		centroid += img.C;
	centroid /= (REAL)sub.images.size();
	RMatrix R;
	R.SetRotationAxisAngle(Point3(0, 0, D2R(foldDeg)));
	for (IIndex k = half; k < sub.images.size(); ++k) {
		Image& img = sub.images[k];
		img.R = RMatrix(img.R * R.t());
		img.C = centroid + R * (img.C - centroid);
	}
}

// RingFixture::Split() for the rings that carry what it does not apply: the drift of every block and the
// folded block (its halves' pairs dropped first, its second half turned), and for the ones that hand out
// only some of their blocks (`handedOut`; the images of the others form the pool)
static void SplitRing(RingFixture& ring, const std::vector<IIndexArr>& handedOut = {})
{
	const RingSceneConfig& cfg = ring.cfg;
	if (cfg.foldedBlock != NO_ID)
		DropFoldPairs(ring.scene, ring.blocks[cfg.foldedBlock], cfg.camsPerBlock / 2);
	ring.applied = SplitOnTruth(ring.scene, ring.split, handedOut.empty() ? ring.blocks : handedOut, ring.truth, cfg.seed + 1,
		[&cfg](uint32_t b, Scene& sub) {
			BendRingBlock(cfg, sub);
			if (b == cfg.foldedBlock)
				FoldRingBlock(sub, cfg.camsPerBlock / 2, cfg.foldDeg);
		});
}

// The final adjustment of a hierarchical reconstruction, run on a merged scene: the merged model's
// own adjustment at the loose bar, then the tracks filtered and re-triangulated at the bar and the
// global bundle adjustment
static void RunFinalAdjustment(Scene& scene)
{
	const ReconstructionConfig cfg;
	AdjustMergedScene(scene, cfg);
	FilterTracks(scene, cfg.maxReprojError, cfg.minAngleThreshold, cfg.multDepthNear, cfg.multDepthFar);
	TriangulateTracks(scene, true, cfg.maxReprojError, cfg.minAngleThreshold);
	FilterTracks(scene, cfg.maxReprojError, cfg.minAngleThreshold, cfg.multDepthNear, cfg.multDepthFar);
	scene.status.nState.set(Scene::Status::STATE::CALIBRATED);
	BundleAdjustment::Adjust(scene, BAConfig());
	FilterTracks(scene, cfg.maxFineReprojError, cfg.minAngleThreshold, cfg.multDepthNear, cfg.multDepthFar);
}

// The observations a seam edit removes, as (image, feature) keys; a match resting on one of them goes with it
typedef std::set<PairIdx::PairIndex> DroppedObservations;
static void DropMatchesOf(Scene& scene, const DroppedObservations& dropped)
{
	for (ImagePair& pair : scene.pairs) {
		const IIndex ID1 = pair.ID1, ID2 = pair.ID2;
		pair.matches.erase(std::remove_if(pair.matches.begin(), pair.matches.end(),
			[&dropped, ID1, ID2](const DMatch& match) {
				return dropped.count(PairIdx(ID1, match.queryIdx).idx) > 0 || dropped.count(PairIdx(ID2, match.trainIdx).idx) > 0;
			}), pair.matches.end());
	}
}

// Weaken the direction "cameras of rigBlock on the points of pointBlock": of the tracks observed in
// both blocks, `keepTracks` keep exactly two pointBlock observations (their first two) and exactly
// one rigBlock observation (assigned round-robin over the first `rigCameras` cameras of rigBlock
// that see the point); every other shared track loses all its rigBlock observations; the matches
// follow the observations. The seam collector makes one correspondence per cross match, so each
// kept track yields two correspondences in that direction and none in the reverse one (one
// observation cannot be triangulated in rigBlock). keepTracks = 0 disconnects the pair. Call
// before the split
static void ThinSeam(Scene& scene, const std::vector<IIndexArr>& blocks, uint32_t pointBlock,
	uint32_t rigBlock, unsigned keepTracks, unsigned rigCameras)
{
	const std::set<IIndex> pointImages(blocks[pointBlock].begin(), blocks[pointBlock].end());
	const std::set<IIndex> rigImages(blocks[rigBlock].begin(), blocks[rigBlock].end());
	const unsigned numRig = MINF(rigCameras, (unsigned)blocks[rigBlock].size());

	DroppedObservations dropped;
	unsigned kept = 0, nextCamera = 0;
	for (Track& track : scene.tracks) {
		bool seenByPoints = false, seenByRig = false;
		for (const Observation& obs : track.observations) {
			seenByPoints = seenByPoints || pointImages.count(obs.imageID) > 0;
			seenByRig = seenByRig || rigImages.count(obs.imageID) > 0;
		}
		if (!seenByPoints || !seenByRig)
			continue;
		// the one rig camera this track is left with, taken in turn from those that see it
		IIndex rigImage = NO_ID;
		if (kept < keepTracks) {
			for (unsigned k = 0; k < numRig && rigImage == NO_ID; ++k) {
				const IIndex image = blocks[rigBlock][(nextCamera + k) % numRig];
				for (const Observation& obs : track.observations)
					if (obs.imageID == image) {
						rigImage = image;
						break;
					}
			}
			if (rigImage != NO_ID) {
				++kept;
				nextCamera = (nextCamera + 1) % numRig;
			}
		}
		// the survivors keep their order, so the inliers the track holds first stay first
		ObservationArr keptObservations;
		unsigned keptInliers = 0, numPointObservations = 0;
		FOREACH(i, track.observations) {
			const Observation& obs = track.observations[i];
			const bool isPoint = pointImages.count(obs.imageID) > 0;
			// every rig observation but the chosen one, and the point observations past the two a
			// track needs to be triangulated where the seam reads it
			if ((rigImages.count(obs.imageID) > 0 && obs.imageID != rigImage) ||
				(isPoint && rigImage != NO_ID && numPointObservations >= 2)) {
				dropped.insert(PairIdx(obs.imageID, obs.featureID).idx);
				continue;
			}
			if (isPoint)
				++numPointObservations;
			if (i < track.numInliers)
				++keptInliers;
			keptObservations.emplace_back(obs);
		}
		if (keptObservations.size() == track.observations.size())
			continue;
		track.observations = keptObservations;
		track.numInliers = (uint8_t)keptInliers;
	}
	DropMatchesOf(scene, dropped);
}

// Plant a false seam: up to `count` tracks of pointBlock no camera of rigBlock ever saw each get
// ONE new observation, in a camera of rigBlock (round-robin over the `rigCameras` cameras from
// `firstCamera` on that see the moved point), of the point moved by `wrong` (a Sim(3) of the world
// frame); the projection becomes a new keypoint of that image and a match of the cross pair with
// every pointBlock image observing the track (pairs created when absent). Usable in the direction
// "rig of rigBlock on the points of pointBlock" only, since one observation cannot be triangulated
// in rigBlock. Returns the number planted. Call before the split
static unsigned PlantFalseSeam(Scene& scene, const std::vector<IIndexArr>& blocks, uint32_t rigBlock,
	uint32_t pointBlock, unsigned count, unsigned rigCameras, const SEACAVE::Transform& wrong,
	unsigned firstCamera = 0)
{
	ASSERT(firstCamera < blocks[rigBlock].size());
	const std::set<IIndex> rigImages(blocks[rigBlock].begin(), blocks[rigBlock].end());
	const std::set<IIndex> pointImages(blocks[pointBlock].begin(), blocks[pointBlock].end());
	const unsigned numRig = MINF(rigCameras, (unsigned)blocks[rigBlock].size() - firstCamera);
	// the pairs by their two images, so a cross pair is found once and created once
	std::unordered_map<PairIdx::PairIndex, uint32_t> pairOf;
	FOREACH(i, scene.pairs)
		pairOf.emplace(PairIdx(scene.pairs[i].ID1, scene.pairs[i].ID2).idx, (uint32_t)i);
	unsigned planted = 0, nextCamera = 0;
	FOREACH(t, scene.tracks) {
		if (planted >= count)
			break;
		const Track& track = scene.tracks[t];
		bool seenByRig = false, seenByPoints = false;
		for (const Observation& obs : track.observations) {
			seenByRig = seenByRig || rigImages.count(obs.imageID) > 0;
			seenByPoints = seenByPoints || pointImages.count(obs.imageID) > 0;
		}
		if (seenByRig || !seenByPoints)
			continue;
		// where the wrong similarity puts the point, in the first rig camera that sees it there
		IIndex rigImage = NO_ID;
		Point2 projection;
		const Point3 X(wrong * track.position);
		for (unsigned k = 0; k < numRig && rigImage == NO_ID; ++k) {
			const IIndex image = blocks[rigBlock][firstCamera + (nextCamera + k) % numRig];
			const auto [proj, valid] = scene.images[image].ProjectPoint(X);
			if (valid && Image8U::isInside(proj, scene.images[image].GetSize())) {
				rigImage = image;
				projection = proj;
			}
		}
		if (rigImage == NO_ID)
			continue;
		nextCamera = (nextCamera + 1) % numRig;
		const uint32_t feature = (uint32_t)scene.images[rigImage].keypoints.size();
		scene.images[rigImage].keypoints.emplace_back(projection, 0.f, 0.f, 10.f);
		for (const Observation& obs : track.observations) {
			if (!pointImages.count(obs.imageID))
				continue;
			const auto [it, inserted] = pairOf.emplace(
				PairIdx(obs.imageID, rigImage).idx, (uint32_t)scene.pairs.size());
			if (inserted)
				scene.pairs.emplace_back(MINF(obs.imageID, rigImage), MAXF(obs.imageID, rigImage));
			ImagePair& pair = scene.pairs[it->second];
			pair.matches.emplace_back(
				pair.ID1 == rigImage ? feature : obs.featureID,
				pair.ID1 == rigImage ? obs.featureID : feature);
		}
		++planted;
	}
	return planted;
}

// Remove observations of the given cameras (local indices) of `block` on the tracks also seen in
// `other`, keeping the fraction `keepFraction` of them (every observation whose running index per
// camera satisfies (index % 100) < keepFraction*100; 0 removes all), with their matches. Call
// before the split
static void DropCrossObservations(Scene& scene, const std::vector<IIndexArr>& blocks, uint32_t block,
	const std::vector<unsigned>& cameras, uint32_t other, REAL keepFraction)
{
	const std::set<IIndex> otherImages(blocks[other].begin(), blocks[other].end());
	std::set<IIndex> targets;
	for (const unsigned k : cameras)
		if (k < blocks[block].size())
			targets.insert(blocks[block][k]);
	const unsigned keep = (unsigned)ROUND2INT(keepFraction * 100);

	DroppedObservations dropped;
	std::unordered_map<IIndex, unsigned> running; // per camera, how many of its observations came before
	for (Track& track : scene.tracks) {
		bool seenInOther = false;
		for (const Observation& obs : track.observations)
			seenInOther = seenInOther || otherImages.count(obs.imageID) > 0;
		if (!seenInOther)
			continue;
		// the survivors keep their order, so the inliers the track holds first stay first
		ObservationArr kept;
		unsigned keptInliers = 0;
		FOREACH(i, track.observations) {
			const Observation& obs = track.observations[i];
			if (targets.count(obs.imageID) > 0 && running[obs.imageID]++ % 100 >= keep) {
				dropped.insert(PairIdx(obs.imageID, obs.featureID).idx);
				continue;
			}
			if (i < track.numInliers)
				++keptInliers;
			kept.emplace_back(obs);
		}
		if (kept.size() == track.observations.size())
			continue;
		track.observations = kept;
		track.numInliers = (uint8_t)keptInliers;
	}
	DropMatchesOf(scene, dropped);
}

// Of the cross pairs of two blocks keep those `keep(imageOfA, imageOfB)` accepts, and weigh the pairs again
static void KeepCrossPairs(Scene& scene, const IIndexArr& blockA, const IIndexArr& blockB, const std::function<bool(IIndex, IIndex)>& keep)
{
	const std::set<IIndex> a(blockA.begin(), blockA.end()), b(blockB.begin(), blockB.end());
	RFOREACH(i, scene.pairs) {
		const ImagePair& pair = scene.pairs[i];
		const IIndex imgA = a.count(pair.ID1) ? pair.ID1 : pair.ID2, imgB = imgA == pair.ID1 ? pair.ID2 : pair.ID1;
		if (a.count(imgA) && b.count(imgB) && !keep(imgA, imgB))
			scene.pairs.RemoveAtMove(i);
	}
	ComputePairsWeights(scene);
}

// The bars the block poses a merge leaves a ring at are held to, before the final adjustment: every
// seam of the ring is biased by a few hundredths of a degree the same way (0.03-0.09 at 0.3 px of
// keypoint noise), and the averaging spreads what the ring's seams miss it by evenly over it
static constexpr REAL kRingMergeRotation = 0.3, kRingMergePosition = 0.002;

// The one way a merge test asserts its result: the blocks placed, no image left behind, every
// registered image accounted for by a placed block, and the merged poses on the truth (the median of
// the images' errors under the best similarity) under the two bars
static bool CheckRingMerge(const TestScope& scope, const char* what, const MergeReport& rep, const Scene& scene,
	const std::vector<Pose3D>& truth, unsigned expectedPlaced, REAL maxRotDeg, REAL maxPosFraction)
{
	TEST_CHECK(rep.numPlaced == expectedPlaced, "%s: %u/%u blocks placed, expected %u", what, rep.numPlaced, rep.numBlocks, expectedPlaced);
	TEST_CHECK(rep.unplacedImages.empty(), "%s: %u images were left unplaced", what, (unsigned)rep.unplacedImages.size());
	TEST_CHECK(scene.status.nCalibratedImages == rep.numImagesPlaced, "%s: %u images registered against %u placed",
		what, scene.status.nCalibratedImages, rep.numImagesPlaced);
	const PoseError e = PoseErrors(scene, truth);
	VERBOSE("  %s: %u blocks, %u images, %.4f deg and %.4f%% from the truth", what, rep.numPlaced, rep.numImagesPlaced,
		e.rotationMedian, e.positionMedian * 100);
	TEST_CHECK(e.rotationMedian <= maxRotDeg && e.positionMedian <= maxPosFraction,
		"%s: the merged model is %.4f deg and %.3f%% off the truth (%.4f deg and %.3f%% allowed)",
		what, e.rotationMedian, e.positionMedian * 100, maxRotDeg, maxPosFraction * 100);
	return true;
}
/*----------------------------------------------------------------*/

// The diagonal of the box a block's cameras span, the unit its seams' translations are judged in
static std::vector<REAL> RingBlockExtents(const std::vector<Scene>& subScenes)
{
	std::vector<REAL> blockExtents(subScenes.size(), REAL(0));
	FOREACH(b, subScenes) {
		AABB3 bbox(true);
		for (const Image& img : subScenes[b].images)
			if (img.IsValid())
				bbox.InsertFull(img.C);
		blockExtents[b] = bbox.IsEmpty() ? REAL(0) : (REAL)bbox.GetSize().norm();
	}
	return blockExtents;
}

// How far one A -> B similarity sits from another: the rotation between them in degrees, and the
// translation of their discrepancy as a fraction of the extent of block A's cameras
static std::pair<REAL, REAL> SeamOffset(const SEACAVE::Transform& T, const SEACAVE::Transform& truth, REAL extentA)
{
	const SEACAVE::Transform E(truth.Invert() * T);
	return std::make_pair(RotationErrorDeg(T.R, truth.R), extentA > 0 ? norm(E.t) / extentA : REAL(0));
}

// The pixel error of one seam observation under an A -> B similarity, as the merge charges it: the
// angle between the observed bearing and the one the transformed point gives, at the camera's rate
static REAL SeamPixelError(const SeamObservation& obs, const SEACAVE::Transform& T)
{
	const Point3 p(obs.forward ? T * obs.X : T.Invert() * obs.X);
	const Point3 d(obs.R * (p - obs.C));
	const REAL len = norm(d);
	if (!(len > 0))
		return REAL(FLT_MAX);
	return obs.pixelPerRadian * ACOS(CLAMP((d / len).dot(normalized(obs.bearing)), REAL(-1), REAL(1)));
}

// Every correspondence of every block pair the cross pairs join, collected as the merge collects
// them, before a merge consumes the blocks
struct BlockPairCorrespondences
{
	uint32_t a, b;
	std::vector<SeamObservation> observations;
	std::vector<SeamCorrespondence> correspondences;
};
static std::vector<BlockPairCorrespondences> CollectBlockPairCorrespondences(RingFixture& ring)
{
	const std::vector<Scene>& subScenes = ring.split.blocks;
	const GlobalAlignmentConfig alignCfg;
	GlobalAlignment alignment(ring.scene, alignCfg);
	alignment.PrepareSeamEvidence(subScenes, ring.split.localToGlobal);
	std::vector<BlockPairCorrespondences> all;
	for (uint32_t a = 0; a < subScenes.size(); ++a)
		for (uint32_t b = a + 1; b < subScenes.size(); ++b) {
			BlockPairCorrespondences pair{a, b, {}, {}};
			alignment.CollectPairObservations(subScenes, a, b, pair.observations, pair.correspondences);
			if (!pair.observations.empty())
				all.emplace_back(std::move(pair));
		}
	return all;
}

// The block pairs the merged model rests on, each once
static std::set<std::pair<uint32_t, uint32_t>> ModelBlockPairs(const MergeReport& rep)
{
	std::set<std::pair<uint32_t, uint32_t>> pairs;
	for (const uint32_t i : rep.modelSeams)
		pairs.emplace(rep.candidates[i].sceneA, rep.candidates[i].sceneB);
	return pairs;
}

// The hypothesis of the merged model on the pair (a, b), NULL when the model does not rest on one
static const SeamCandidate* ModelSeamOf(const MergeReport& rep, uint32_t a, uint32_t b)
{
	for (const uint32_t i : rep.modelSeams)
		if (rep.candidates[i].sceneA == a && rep.candidates[i].sceneB == b)
			return &rep.candidates[i];
	return NULL;
}

// A copy one period along the wall: a translation along the ring's tangent at the given angle, what
// a repeated facade makes of the structure across a seam
static SEACAVE::Transform WallCopy(REAL angle, REAL period)
{
	SEACAVE::Transform copy;
	copy.t = Point3(-SIN(angle), COS(angle), 0) * period;
	return copy;
}

// The readings of a block pair that came from one direction, NULL when it gave none
static const SeamCandidate* ReadingOf(const std::vector<SeamCandidate>& readings, SeamCandidate::Source source);
// An image of a block left with observations on `keep` tracks only
static void ThinImage(Scene& block, IIndex image, unsigned keep);
/*----------------------------------------------------------------*/

// A whole ring, block by block: every block arrives in its own frame and the merge has to take all
// of them into one model, on the evidence of the seams alone. Three arms of six blocks:
// 1. the honest ring: the block poses the averaging leaves, then the model the final adjustment makes of them (every
//    seam is biased by a few hundredths of a degree the same way, which the averaging spreads evenly round the ring and
//    the adjustment takes out); the readings lie on the ring's pairs and the merged model rests on every one of them;
// 2. every block reconstructed bent and stretched: every seam is off the same way, so an open chain of the six ends
//    about three degrees and three percent from where it started; the averaging of the whole ring spreads that over its
//    cycle, every reading within the bars of the solution, and the final adjustment closes the ring on the truth. The
//    bars are held an order of magnitude wider than what the merge leaves, because what they have to catch is a cycle
//    left open: a ring carried the long way round and never closed lands at the discrepancy of the whole chain;
// 3. the honest ring with one camera of a block a billion units from its place and on no track, as a block's adjustment
//    can leave an image that lost its points: the merge reads every seam at its own cameras, so the six blocks are
//    placed as without it and every other camera lands on the truth (read at a block's camera centroid, in units of
//    its camera box, the stray camera moves both and the blocks miss their place).
bool RingMergeTest()
{
	TestScope scope("RingMergeTest");
	constexpr unsigned NUM_BLOCKS = 6;
	{
		RingFixture ring({NUM_BLOCKS, 10});
		ring.Split();
		const MergeReport rep = ring.Merge();
		if (!CheckRingMerge(scope, "honest ring", rep, ring.scene, ring.truth, NUM_BLOCKS, kRingMergeRotation, kRingMergePosition))
			return false;
		RunFinalAdjustment(ring.scene);
		if (!CheckRingMerge(scope, "honest ring, adjusted", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.1, 0.001))
			return false;
		// every block overlaps the next and the ring closes: the readings lie on the ring's pairs, and the merged model
		// rests on every one of them
		std::set<std::pair<uint32_t, uint32_t>> pairs;
		for (const SeamCandidate& c : rep.candidates)
			pairs.emplace(c.sceneA, c.sceneB);
		TEST_CHECK(pairs.size() == NUM_BLOCKS && rep.modelSeams.size() == rep.candidates.size(),
			"honest ring: %u readings on %u pairs, %u of them in the merged model (%u pairs, all of them in the model expected)",
			(unsigned)rep.candidates.size(), (unsigned)pairs.size(), (unsigned)rep.modelSeams.size(), NUM_BLOCKS);
	}
	{
		RingSceneConfig cfg{NUM_BLOCKS, 10};
		cfg.driftDegPerBlock = 0.5;
		cfg.driftScalePerBlock = 0.005;
		RingFixture ring(cfg);
		SplitRing(ring);
		const MergeReport rep = ring.Merge();
		if (!CheckRingMerge(scope, "bent ring", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.6, 0.005))
			return false;
		RunFinalAdjustment(ring.scene);
		if (!CheckRingMerge(scope, "bent ring, adjusted", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.3, 0.005))
			return false;
	}
	{
		constexpr uint32_t STRAY_BLOCK = 2;
		constexpr IIndex STRAY_LOCAL = 4;
		RingFixture ring({NUM_BLOCKS, 10});
		ring.Split();
		const IIndex stray = ring.split.localToGlobal[STRAY_BLOCK][STRAY_LOCAL];
		ThinImage(ring.split.blocks[STRAY_BLOCK], STRAY_LOCAL, 0);
		ring.split.blocks[STRAY_BLOCK].images[STRAY_LOCAL].C = Point3(1e9, 1e9, 1e9);
		const MergeReport rep = ring.Merge();
		IIndexArr others;
		FOREACH(g, ring.scene.images)
			if ((IIndex)g != stray)
				others.push_back((IIndex)g);
		const PoseError e = PoseErrors(ring.scene, ring.truth, others);
		VERBOSE("  stray camera: %u blocks placed, the other cameras %.4f deg and %.4f%% from the truth", rep.numPlaced, e.rotationMedian, e.positionMedian * 100);
		TEST_CHECK(rep.numPlaced == NUM_BLOCKS && rep.unplacedImages.empty() && e.rotationMedian <= kRingMergeRotation && e.positionMedian <= kRingMergePosition,
			"stray camera: %u/%u blocks placed, %u images unplaced, the other cameras %.4f deg and %.3f%% off the truth (%.1f deg and %.2f%% allowed)",
			rep.numPlaced, NUM_BLOCKS, (unsigned)rep.unplacedImages.size(), e.rotationMedian, e.positionMedian * 100, kRingMergeRotation, kRingMergePosition * 100);
	}
	return scope.Pass("a ring of six blocks is placed whole on its seams and the final adjustment takes it to the truth, drifting or not, "
		"and a camera a billion units from its place in one block moves no block");
}
/*----------------------------------------------------------------*/

// The track join of the merge across two blocks: a right and a false cross match whose two roots
// lie the same 3D distance apart, at different depths. The right one is a far point the second
// block placed 0.5 too deep along its own camera's ray (well under a pixel in every view); the false
// one joins two different near points 0.5 apart across the ray (50 px). The same for a one-sided root,
// a block-0 track matched to a keypoint of block 1 no track holds, once at its true projection and once
// 50 px off. With the join guard off the merge joins every one of them on the duplicate-image and length
// guards alone; at a guard under 50 px the false ones are refused. The merged model's adjustment
// (AdjustMergedScene: the loose-bar filter, the triangulation, the filter again) decides each observation:
// the right matches end as inliers of one track, the false ones never do. And the closure's join priority: a
// far point whose seam match (i, j) and a competing match of a pair (a0, j) visited before it in the connecting
// pairs' order both put a keypoint of j on its track: the seam match joins first, the competing one is refused
// for holding image j twice. With `chain`, 300 pool images (in no block, unposed), each matched to the next
// through one keypoint, join in the same merge's track union one image at a time until the next join would make
// 256 observations: a track never holds more than 255 (its inlier count is a byte), so that join and every later
// one into it is refused and the chain ends in tracks of at most 255 observations.
static bool CrossBlockJoinCase(const TestScope& scope, const char* arm, float maxJoinReprojError, bool chain)
{
	RingFixture ring({2, 20});
	Scene& scene = ring.scene;
	// the four cameras at a seam: two of block 0, two of block 1, and the cross pair (i, j)
	const IIndex a0 = ring.blocks[0][ring.blocks[0].size() - 2], i = ring.blocks[0].back(), j = ring.blocks[1].front(), b1 = ring.blocks[1][1];
	ring.Split();
	std::vector<Scene>& subScenes = ring.split.blocks;
	const std::vector<IIndexArr>& localToGlobals = ring.split.localToGlobal;
	ImagePair* crossPair = scene.FindPair(i, j);
	TEST_CHECK(crossPair != NULL, "%s: no pair across the seam between images %u and %u", arm, i, j);
	const auto LocalOf = [&](uint32_t b, IIndex g) -> IIndex {
		FOREACH(k, localToGlobals[b])
			if (localToGlobals[b][k] == g)
				return k;
		return NO_ID;
	};
	const auto Ray = [&](IIndex g, const Point3& X) { return Point3(normalized(X - Point3(ring.truth[g].C))); };
	// a keypoint at the projection of the world point X in global image g (of block b); its feature ID
	const auto AddKeypoint = [&](uint32_t b, IIndex g, const Point3& X, uint32_t& feature) -> bool {
		Image& img = subScenes[b].images[LocalOf(b, g)];
		const auto [proj, valid] = img.ProjectPoint(ring.applied[b] * X);
		if (!valid || !Image8U::isInside(proj, img.GetSize()))
			return false;
		feature = (uint32_t)img.keypoints.size();
		img.keypoints.emplace_back(proj, 0.f, 0.f, 10.f);
		return true;
	};
	// a track of block b over two of its images seeing the world point X, held at the world position P;
	// the feature of its observation in the seam image (i for block 0, j for block 1)
	const auto AddTrack = [&](uint32_t b, IIndex g1, IIndex g2, const Point3& X, const Point3& P, uint32_t& seamFeature, uint32_t* otherFeature = NULL) -> bool {
		uint32_t f1, f2;
		if (!AddKeypoint(b, g1, X, f1) || !AddKeypoint(b, g2, X, f2))
			return false;
		seamFeature = (g1 == i || g1 == j) ? f1 : f2;
		if (otherFeature)
			*otherFeature = (g1 == i || g1 == j) ? f2 : f1;
		Track track(ring.applied[b] * P);
		track.observations.emplace_back(LocalOf(b, g1), f1);
		track.observations.emplace_back(LocalOf(b, g2), f2);
		track.numInliers = 2;
		subScenes[b].tracks.emplace_back(std::move(track));
		return true;
	};
	const Point3 mid((Point3(ring.truth[i].C) + Point3(ring.truth[j].C)) / REAL(2));
	const Point3 out(normalized(Point3(mid.x, mid.y, 0)));       // radially outward at the seam
	const Point3 up(0, 0, 1);
	constexpr REAL d = 0.5;
	// two far points (depth about 30) and two near ones (depth about 4), a little apart
	const Point3 far1(mid + out * REAL(30) + up * REAL(1)), far2(mid + out * REAL(30) - up * REAL(1));
	const Point3 near1(mid + out * REAL(4) + up * REAL(0.3)), near2(mid + out * REAL(4) - up * REAL(0.3));
	const Point3 across(normalized(Ray(j, near1).cross(up)));   // perpendicular to camera j's ray
	uint32_t fRightI, fRightJ, fFalseI, fFalseJ, fOneRightI, fOneRightJ, fOneFalseI, fOneFalseJ;
	uint32_t fFirstI, fFirstA0, fFirstJ, fFirstBadJ; // the join priority: a far point with a seam match and a competing one
	const Point3 far3(mid + out * REAL(30));
	// two positioned roots, the right match: block 1 holds the far point d too deep along camera j's ray
	// two positioned roots, the false match: block 1's track is another point d across camera j's ray
	// one positioned root: block 0's track matched to a lone keypoint of image j, right and 50 px off
	TEST_CHECK(AddTrack(0, a0, i, far1, far1, fRightI) && AddTrack(1, j, b1, far1, far1 + Ray(j, far1) * d, fRightJ) &&
		AddTrack(0, a0, i, near1, near1, fFalseI) && AddTrack(1, j, b1, near1 + across * d, near1 + across * d, fFalseJ) &&
		AddTrack(0, a0, i, far2, far2, fOneRightI) && AddKeypoint(1, j, far2, fOneRightJ) &&
		AddTrack(0, a0, i, near2, near2, fOneFalseI) && AddKeypoint(1, j, near2 + across * d, fOneFalseJ) &&
		AddTrack(0, a0, i, far3, far3, fFirstI, &fFirstA0) && AddTrack(1, j, b1, far3, far3, fFirstJ) &&
		AddKeypoint(1, j, far3 + across * REAL(4), fFirstBadJ), "%s: a test point does not project into the seam's four images", arm);
	crossPair->matches.emplace_back(fRightI, fRightJ);
	crossPair->matches.emplace_back(fFalseI, fFalseJ);
	crossPair->matches.emplace_back(fOneRightI, fOneRightJ);
	crossPair->matches.emplace_back(fOneFalseI, fOneFalseJ);
	// the priority: the seam match (i, j) of the far point, and before it in the pairs' order a pair (a0, j) whose
	// match puts another keypoint of j, 4 across the point, on the same track
	crossPair->matches.emplace_back(fFirstI, fFirstJ);
	if (scene.FindPair(a0, j) == NULL)
		scene.pairs.emplace_back(a0, j);
	{
		const auto IndexOf = [&](IIndex x, IIndex y) { FOREACH(k, scene.pairs) if (scene.pairs[k].ID1 == x && scene.pairs[k].ID2 == y) return (IIndex)k; return NO_ID; };
		const IIndex kCompeting = IndexOf(a0, j), kSeam = IndexOf(i, j);
		ASSERT(kCompeting != NO_ID && kSeam != NO_ID);
		if (kCompeting > kSeam)
			std::swap(scene.pairs[kCompeting], scene.pairs[kSeam]);
		scene.FindPair(a0, j)->matches.emplace_back(fFirstA0, fFirstBadJ);
	}
	constexpr unsigned numChain = 300;
	const IIndex firstChain = (IIndex)scene.images.size();
	if (chain) {
		for (unsigned k = 0; k < numChain; ++k) {
			Image& img = scene.images.emplace_back((IIndex)scene.images.size(), String(), Pose3D(), 0, scene.cameras[0]);
			img.InvalidatePose();
			img.keypoints.emplace_back(Point2f(320.f, 240.f), 0.f, 0.f, 10.f);
		}
		for (unsigned k = 0; k + 1 < numChain; ++k)
			scene.pairs.emplace_back(firstChain + k, firstChain + k + 1).matches.emplace_back(0, 0);
	}

	GlobalAlignmentConfig alignCfg;
	alignCfg.maxJoinReprojError = maxJoinReprojError;
	const MergeReport rep = ring.Merge(true, NO_ID, {}, alignCfg);
	TEST_CHECK(rep.numPlaced == 2, "%s: %u of 2 blocks placed", arm, rep.numPlaced);
	// joined: one merged track holds both ends of the match (as inliers, when asked)
	const auto Joined = [&](uint32_t fI, uint32_t fJ, bool inliers) -> bool {
		for (const Track& track : scene.tracks) {
			bool hasI = false, hasJ = false;
			FOREACH(k, track.observations) {
				const Observation& obs = track.observations[k];
				if (inliers && k >= track.numInliers)
					continue;
				hasI |= obs.imageID == i && obs.featureID == fI;
				hasJ |= obs.imageID == j && obs.featureID == fJ;
			}
			if (hasI && hasJ)
				return true;
		}
		return false;
	};
	// the seam-consistent match took the far point's track; the competing one, visited first, was refused
	const bool firstRight = Joined(fFirstI, fFirstJ, false), firstWrong = Joined(fFirstI, fFirstBadJ, false);
	TEST_CHECK(firstRight && !firstWrong, "%s: the seam match of the far point %s and the competing match of the pair visited "
		"before it %s (the seam match expected to join first, the other refused for holding image %u twice)",
		arm, firstRight ? "joined" : "did not join", firstWrong ? "joined" : "did not join", j);
	const bool joined[4] = {Joined(fRightI, fRightJ, false), Joined(fFalseI, fFalseJ, false),
		Joined(fOneRightI, fOneRightJ, false), Joined(fOneFalseI, fOneFalseJ, false)};
	// with the guard off every match joins; at 48 px the false ones (50 px off) are refused
	const bool guarded = maxJoinReprojError > 0.f;
	TEST_CHECK(joined[0] && joined[1] != guarded && joined[2] && joined[3] != guarded,
		"%s: the merge joined right %d, false %d, one-sided right %d, one-sided 50 px off %d (the right ones expected joined, the false ones %s at a join guard of %g px)",
		arm, joined[0], joined[1], joined[2], joined[3], guarded ? "refused" : "joined", maxJoinReprojError);
	if (chain) {
		unsigned longest = 0, numChainTracks = 0, numChainObservations = 0;
		for (const Track& track : scene.tracks) {
			longest = MAXF(longest, (unsigned)track.observations.size());
			if (track.observations[0].imageID >= firstChain) {
				++numChainTracks;
				numChainObservations += (unsigned)track.observations.size();
			}
		}
		TEST_CHECK(longest <= 255 && numChainTracks >= 2 && numChainObservations == numChain, "%s: the longest merged track holds %u "
			"observations (at most 255 expected); the chain of %u images is in %u tracks holding %u observations (at least 2 tracks, all %u observations expected)",
			arm, longest, numChain, numChainTracks, numChainObservations, numChain);
	}
	AdjustMergedScene(scene, ReconstructionConfig());
	const bool right = Joined(fRightI, fRightJ, true), wrong = Joined(fFalseI, fFalseJ, true);
	const bool oneRight = Joined(fOneRightI, fOneRightJ, true), oneWrong = Joined(fOneFalseI, fOneFalseJ, true);
	TEST_CHECK(right && !wrong && oneRight && !oneWrong, "%s: after the merged model's adjustment, two positioned roots d=%g apart: right match %s "
		"(expected inliers of one track), false match %s (expected not); one positioned root: right %s (expected inliers), 50 px off %s (expected not)",
		arm, d, right ? "inliers" : "not", wrong ? "inliers" : "not", oneRight ? "inliers" : "not", oneWrong ? "inliers" : "not");
	return true;
}

// The seams the merged model rests on already agree with the placement: every one of their kept
// correspondences must land its two ends on the same track once the merge is done, not wait on
// proximity to find each other.
static bool SeamInliersCase(const TestScope& scope)
{
	RingFixture ring({4, 10});
	ring.Split();
	// (global image, feature) -> the sub-scene track it belongs to (block, local track index); captured now because
	// the merge consumes the blocks
	std::unordered_map<PairIdx::PairIndex, PairIdx::PairIndex> subTrackOf;
	FOREACH(b, ring.split.blocks)
		FOREACH(t, ring.split.blocks[b].tracks)
			for (const Observation& obs : ring.split.blocks[b].tracks[t].observations)
				subTrackOf.emplace(PairIdx(ring.split.localToGlobal[b][obs.imageID], obs.featureID).idx, PairIdx((uint32_t)b, (uint32_t)t).idx);
	const std::vector<BlockPairCorrespondences> blockPairs(CollectBlockPairCorrespondences(ring));
	const GlobalAlignmentConfig alignCfg;
	const MergeReport rep = ring.Merge();
	if (!CheckRingMerge(scope, "seam inliers", rep, ring.scene, ring.truth, 4, kRingMergeRotation, kRingMergePosition))
		return false;
	TEST_CHECK(!rep.modelSeams.empty(), "seam inliers: the merged model rests on no seam");

	// which block owns every global image, to tell a track spanning two blocks from one that does not
	std::vector<uint32_t> blockOf(ring.scene.images.size(), NO_ID);
	FOREACH(b, ring.blocks)
		for (const IIndex image : ring.blocks[b])
			blockOf[image] = (uint32_t)b;
	// (image, feature) -> the track it ended up on, after the merge
	std::unordered_map<PairIdx::PairIndex, uint32_t> trackOf;
	FOREACH(t, ring.scene.tracks)
		for (const Observation& obs : ring.scene.tracks[t].observations)
			trackOf.emplace(PairIdx(obs.imageID, obs.featureID).idx, (uint32_t)t);

	unsigned numChecked = 0, numViolations = 0;
	std::set<std::pair<PairIdx::PairIndex, PairIdx::PairIndex>> distinctTrackPairs;
	const std::set<std::pair<uint32_t, uint32_t>> modelPairs(ModelBlockPairs(rep));
	for (const BlockPairCorrespondences& pair : blockPairs) {
		if (!modelPairs.count(std::make_pair(pair.a, pair.b)))
			continue;
		// the pair's inliers at the bar under the final block poses
		const SEACAVE::Transform T(rep.poses[pair.b].T.Invert() * rep.poses[pair.a].T);
		FOREACH(i, pair.observations) {
			if (SeamPixelError(pair.observations[i], T) > alignCfg.maxReprojError)
				continue;
			const SeamCorrespondence& corr = pair.correspondences[i];
			++numChecked;
			const auto itA = trackOf.find(PairIdx(corr.imageA, corr.featureA).idx);
			const auto itB = trackOf.find(PairIdx(corr.imageB, corr.featureB).idx);
			if (itA == trackOf.end() || itB == trackOf.end() || itA->second != itB->second)
				++numViolations;
			// the pair of sub-scene tracks this correspondence bridges, deduplicated: several raw matches bridge the same two
			// sub-scene tracks (one per crossing camera pair, up to r x p of them for r rig and p point cameras on the same 3D
			// point), and the merge only owes one merged track per distinct pair, not one per match
			const auto preA = subTrackOf.find(PairIdx(corr.imageA, corr.featureA).idx);
			const auto preB = subTrackOf.find(PairIdx(corr.imageB, corr.featureB).idx);
			if (preA != subTrackOf.end() && preB != subTrackOf.end())
				distinctTrackPairs.emplace(MINF(preA->second, preB->second), MAXF(preA->second, preB->second));
		}
	}
	TEST_CHECK(numChecked > 0, "seam inliers: the seams the merged model rests on hold no inlier correspondence");
	TEST_CHECK(numViolations == 0, "seam inliers: %u/%u seam inliers did not land their two ends on the same track", numViolations, numChecked);
	TEST_CHECK(!distinctTrackPairs.empty(), "seam inliers: no seam inlier bridges two sub-scene tracks");
	// every distinct pair of sub-scene tracks a seam inlier bridges must come out as one track spanning two blocks; equality
	// is the ideal merge, fewer means the merge collapsed distinct points into one track
	unsigned numCrossBlockTracks = 0;
	for (const Track& track : ring.scene.tracks) {
		std::set<uint32_t> tracksBlocks;
		for (const Observation& obs : track.observations)
			if (obs.imageID < blockOf.size() && blockOf[obs.imageID] != NO_ID)
				tracksBlocks.insert(blockOf[obs.imageID]);
		if (tracksBlocks.size() >= 2)
			++numCrossBlockTracks;
	}
	TEST_CHECK(numCrossBlockTracks >= distinctTrackPairs.size(), "seam inliers: only %u tracks span two blocks, out of %u distinct "
		"sub-scene track pairs", numCrossBlockTracks, (unsigned)distinctTrackPairs.size());
	VERBOSE("  seam inliers: %u checked, %u distinct sub-scene track pairs, %u tracks span two blocks", numChecked,
		(unsigned)distinctTrackPairs.size(), numCrossBlockTracks);
	return true;
}

// The cross-block track join in three parts: the join guard of the merge (refusing the matches that put the two ends
// of a seam match 50 px apart, joining the right ones, the seam match before a competing one, and the tracks of a
// chain of pool images at most 255 observations long) once with the guard off and once at its default of 48 px, and
// the seam inliers of the ring: each lands its two ends on one track.
bool MergeTrackJoinTest()
{
	TestScope scope("MergeTrackJoinTest");
	if (!CrossBlockJoinCase(scope, "join guard off", 0.f, true) ||
		!CrossBlockJoinCase(scope, "join guard 48 px", GlobalAlignmentConfig().maxJoinReprojError, false) ||
		!SeamInliersCase(scope))
		return false;
	return scope.Pass("the seam match joins before a competing one, the right matches join and the false ones (50 px off) are refused at "
		"the default guard, a chain of 300 pool images ends in tracks of at most 255 observations, and every seam inlier lands "
		"its two ends on one track");
}
/*----------------------------------------------------------------*/

// A block pair read from light cross pairs together: the closing pair (0, last) of the ring keeps every cross pair, but
// each one weighs POOLED_PAIR_WEIGHT weighted inliers, under the minCommonTracks a pair once needed to link two blocks
// and under the minPairInliers it needed to witness a reading. Read together, the pairs give the closing pair its two
// readings and witness them, and the ring is placed whole. And the witness has a bar on the total: the closing pair
// keeps POOLED_UNDER_BAR_PAIRS cross pairs of one weighted inlier each, under minPairInliers together; it is still read,
// and its readings are witnessed by no pair.
bool PooledSeamReadingTest()
{
	TestScope scope("PooledSeamReadingTest");
	constexpr float POOLED_PAIR_WEIGHT = 12.f;
	constexpr unsigned POOLED_UNDER_BAR_PAIRS = 12, NUM_BLOCKS = 6;
	static_assert(POOLED_UNDER_BAR_PAIRS < 30, "the pairs' total must stay under GlobalAlignmentConfig::minPairInliers");
	const GlobalAlignmentConfig alignCfg;
	const uint32_t closing = NUM_BLOCKS - 1;
	// whether a pair of the scene joins the first block to the closing one
	const auto IsCross = [&](const RingFixture& ring, const ImagePair& pair) {
		const std::set<IIndex> first(ring.blocks[0].begin(), ring.blocks[0].end()), last(ring.blocks[closing].begin(), ring.blocks[closing].end());
		return (first.count(pair.ID1) && last.count(pair.ID2)) || (first.count(pair.ID2) && last.count(pair.ID1));
	};
	// the readings of the closing pair, and how many of them `witnessed` accepts
	const auto ReadClosingPair = [&](RingFixture& ring, const std::function<bool(const SeamCandidate&)>& witnessed) {
		GlobalAlignment alignment(ring.scene, alignCfg);
		std::vector<SeamCandidate> candidates;
		alignment.EstimateSeamCandidates(ring.split.blocks, ring.split.localToGlobal, candidates);
		unsigned numReadings = 0, numWitnessed = 0;
		for (const SeamCandidate& c : candidates)
			if (c.sceneA == 0 && c.sceneB == closing) {
				++numReadings;
				numWitnessed += witnessed(c) ? 1u : 0u;
			}
		return std::make_pair(numReadings, numWitnessed);
	};
	{
		RingFixture ring({NUM_BLOCKS, 10});
		unsigned numLight = 0;
		for (ImagePair& pair : ring.scene.pairs)
			if (IsCross(ring, pair)) {
				pair.weightedInliers = POOLED_PAIR_WEIGHT;
				++numLight;
			}
		// the premise: several light pairs, each under both bars, together over the witness bar
		TEST_CHECK(numLight >= 3 && POOLED_PAIR_WEIGHT < (float)alignCfg.minCommonTracks && POOLED_PAIR_WEIGHT < (float)alignCfg.minPairInliers &&
			numLight * POOLED_PAIR_WEIGHT >= (float)alignCfg.minPairInliers, "premise: %u cross pairs of weight %g across the closing pair "
			"(at least 3, each under %u and %u, together over %u expected)", numLight, POOLED_PAIR_WEIGHT,
			alignCfg.minCommonTracks, alignCfg.minPairInliers, alignCfg.minPairInliers);
		ring.Split();
		const auto [numReadings, numWitnessed] = ReadClosingPair(ring, [&](const SeamCandidate& c) {
			return c.pairs.numPairs == numLight && c.pairs.agree >= (float)alignCfg.minPairInliers;
		});
		TEST_CHECK(numReadings == 2 && numWitnessed == 2, "the closing pair (0, %u) of %u light cross pairs gave %u readings, %u of them "
			"witnessed by all its pairs (2 and 2 expected)", closing, numLight, numReadings, numWitnessed);
		const MergeReport rep = ring.Merge();
		if (!CheckRingMerge(scope, "light pairs read together", rep, ring.scene, ring.truth, NUM_BLOCKS, kRingMergeRotation, kRingMergePosition))
			return false;
	}
	{
		RingFixture ring({NUM_BLOCKS, 10});
		unsigned numKept = 0;
		RFOREACH(i, ring.scene.pairs) {
			if (!IsCross(ring, ring.scene.pairs[i]))
				continue;
			if (numKept == POOLED_UNDER_BAR_PAIRS) {
				ring.scene.pairs.RemoveAtMove(i);
				continue;
			}
			ring.scene.pairs[i].weightedInliers = 1.f;
			++numKept;
		}
		ring.Split();
		const auto [numReadings, numWitnessed] = ReadClosingPair(ring, [](const SeamCandidate& c) { return c.pairs.numPairs > 0; });
		TEST_CHECK(numKept == POOLED_UNDER_BAR_PAIRS && numReadings > 0 && numWitnessed == 0, "thin: %u cross pairs of one weighted inlier "
			"each across (0, %u) gave %u readings, %u of them witnessed (readings expected, none witnessed: %u in total is under "
			"minPairInliers %u)", numKept, closing, numReadings, numWitnessed, numKept, alignCfg.minPairInliers);
	}
	return scope.Pass("light cross pairs read together give the closing pair its readings, witness them and close the ring; under the bar in total "
		"they witness nothing");
}
/*----------------------------------------------------------------*/

// The merge as rounds (GlobalAlignment::MergeScenes keeping or releasing the unplaced blocks, SceneBlocks::RegroupAfterMerge,
// SceneBlocks::MergeInRounds), each part on a ring (RingFixture) some of whose seams are cut (ThinSeam with no track kept):
// 1. the states between rounds: a round keeping the unplaced blocks whole leaves the cut-off block in its scene, the regrouping
//    makes the merged model block 0, and the next round, anchored at block 0 and releasing, places the model again and hands the
//    kept block's images back to the global scene for the resection;
// 2. the merge's own loop on a ring cut in two halves that weigh alike: three rounds, one half posed, the other released;
// 3. a seam resting on pool-registered cameras alone is refused and its block released, one a model camera supports is not;
// 4. the anchor: the model holding block 0 is merged whatever the other models weigh.
bool MergeRoundsTest()
{
	TestScope scope("MergeRoundsTest");
	constexpr unsigned CAMS = 10; // cameras per block of every ring below
	// a ring of numBlocks blocks with the given seams cut, handed out to the merge
	const auto CutRing = [](RingFixture& ring, const std::vector<std::pair<uint32_t, uint32_t>>& cuts) {
		for (const auto& [pointBlock, rigBlock] : cuts)
			ThinSeam(ring.scene, ring.blocks, pointBlock, rigBlock, 0, CAMS);
		ring.Split();
	};
	const auto HasPose = [](const Scene& scene, const IIndexArr& images) {
		return std::all_of(images.begin(), images.end(), [&](IIndex g) { return scene.images[g].HasPose(); });
	};
	// back in the global scene for the resection: features, no pose
	const auto Released = [](const Scene& scene, const IIndexArr& images) {
		return std::all_of(images.begin(), images.end(), [&](IIndex g) { return scene.images[g].HasFeatures() && !scene.images[g].HasPose(); });
	};

	// 1. rounds on a ring of MERGE_ROUNDS_BLOCKS whose block KEPT has no cross match to any other block
	{
		// six blocks: the states between rounds are the same as on twelve, at a quarter of the time
		constexpr uint32_t MERGE_ROUNDS_BLOCKS = 6, KEPT = 5;
		RingFixture ring({MERGE_ROUNDS_BLOCKS, CAMS});
		CutRing(ring, {{KEPT - 1, KEPT}, {KEPT, (KEPT + 1) % MERGE_ROUNDS_BLOCKS}});
		SceneBlocks& split = ring.split;
		const IIndexArr keptImages(split.localToGlobal[KEPT]);
		const size_t keptTracks = split.blocks[KEPT].tracks.size();
		// round 1, keeping the unplaced blocks whole: every block but KEPT placed, KEPT left in its scene with its images,
		// features and tracks, its images without features and pose in the global scene
		MergeReport rep = ring.Merge(false);
		const Scene& keptBlock = split.blocks[KEPT];
		bool keptWhole = rep.numPlaced == MERGE_ROUNDS_BLOCKS - 1 && rep.poses[KEPT].state != BlockPose::ADMITTED &&
			keptBlock.images.size() == keptImages.size() && keptBlock.tracks.size() == keptTracks;
		for (const Image& img : keptBlock.images)
			keptWhole = keptWhole && img.HasFeatures() && img.HasPose();
		for (const IIndex g : keptImages)
			keptWhole = keptWhole && !ring.scene.images[g].HasFeatures() && !ring.scene.images[g].HasPose();
		TEST_CHECK(keptWhole, "round 1 placed %u of %u blocks; block %u (no cross match) is not left whole in its scene with its "
			"%u images, features and %u tracks, its images without features and pose in the global scene",
			rep.numPlaced, MERGE_ROUNDS_BLOCKS, KEPT, (unsigned)keptImages.size(), (unsigned)keptTracks);
		// the regrouping: the merged model is block 0 (every posed image, its features and the scene's tracks moved in), the
		// kept block is block 1 as it was
		std::vector<bool> placed(rep.poses.size());
		FOREACH(b, rep.poses)
			placed[b] = rep.poses[b].state == BlockPose::ADMITTED;
		split.RegroupAfterMerge(placed);
		const unsigned numModel = (unsigned)(ring.scene.images.size() - keptImages.size());
		bool regrouped = split.blocks.size() == 2 && split.blocks[0].images.size() == numModel &&
			split.blocks[0].status.nCalibratedImages == numModel && !split.blocks[0].tracks.empty() &&
			ring.scene.tracks.empty() && split.localToGlobal[1] == keptImages && split.blocks[1].tracks.size() == keptTracks;
		FOREACH(g, ring.scene.images)
			regrouped = regrouped && split.blockOf[g] == (std::find(keptImages.begin(), keptImages.end(), (IIndex)g) != keptImages.end() ? 1u : 0u);
		for (const Image& img : split.blocks[0].images)
			regrouped = regrouped && img.HasFeatures() && img.HasPose();
		TEST_CHECK(regrouped, "after the regrouping %u blocks (2 expected: the model of %u posed images with its features and tracks, "
			"then block %u as it was), %u tracks left in the global scene",
			(unsigned)split.blocks.size(), numModel, KEPT, (unsigned)ring.scene.tracks.size());
		// round 2, releasing and anchored at block 0 as MergeInRounds anchors it: the model placed again, the kept block's
		// images released
		rep = ring.Merge(true, 0u);
		TEST_CHECK(rep.numPlaced == 1 && rep.numImagesUnplaced == keptImages.size() && ring.scene.status.nCalibratedImages == numModel &&
			Released(ring.scene, keptImages), "the releasing round placed %u blocks (the model alone expected), %u images unplaced (%u "
			"expected, back in the global scene with their features and no pose), %u calibrated (%u expected)",
			rep.numPlaced, rep.numImagesUnplaced, (unsigned)keptImages.size(), ring.scene.status.nCalibratedImages, numModel);
	}

	// 2. MergeInRounds on a ring of four blocks cut between blocks 1-2 and 3-0: two models of 20 images that weigh alike.
	// Round 1 merges one half and keeps the other whole; the pool step enters no image; round 2 reads the kept half
	// against the merged model, block 0, and with the two models still weighing alike only the anchor at block 0 makes
	// it merge block 0's; nothing is placed, the fixed point is reached and round 3 releases the kept half.
	{
		RingFixture ring({4, CAMS});
		CutRing(ring, {{1, 2}, {3, 0}});
		const ReconstructionConfig config;
		ResectionConfig resectionCfg = config.resectionCfg;
		resectionCfg.DeriveBAConfigs(config.baConfig);
		const unsigned numRounds = ring.split.MergeInRounds(resectionCfg, config);
		const IIndexArr halves[2] = {IIndexArr{ring.blocks[0]}.Join(ring.blocks[1]), IIndexArr{ring.blocks[2]}.Join(ring.blocks[3])};
		const bool onePosed = HasPose(ring.scene, halves[0]) != HasPose(ring.scene, halves[1]);
		const bool oneReleased = Released(ring.scene, halves[0]) != Released(ring.scene, halves[1]);
		TEST_CHECK(numRounds == 3 && onePosed && oneReleased && ring.scene.status.nCalibratedImages == 2 * CAMS,
			"%u rounds (3 expected), one half posed %d and the other released %d (1 and 1 expected), %u calibrated images (%u expected)",
			numRounds, (int)onePosed, (int)oneReleased, ring.scene.status.nCalibratedImages, 2 * CAMS);
	}

	// 3. A round on the model, block 0, with the images a pool step registered on it: on the ring cut in halves block 1
	// links to the model by one seam, which lands on block 0's cameras only. When all of them are pool images no camera
	// supports the reading on the model's side, the entry test refuses it ("cameras"), block 1 stays unplaced and the
	// releasing round leaves its images to the resection; when only one is, the others support the seam and block 1 is placed.
	for (const bool allPool : {true, false}) {
		RingFixture ring({4, CAMS});
		CutRing(ring, {{1, 2}, {3, 0}});
		std::unordered_set<IIndex> poolImages;
		if (allPool)
			poolImages.insert(ring.blocks[0].begin(), ring.blocks[0].end());
		else
			poolImages.insert(ring.blocks[0][0]);
		const MergeReport rep = ring.Merge(true, 0u, poolImages);
		const bool placed1 = rep.poses[1].state == BlockPose::ADMITTED;
		TEST_CHECK(allPool ? !placed1 && Released(ring.scene, ring.blocks[1]) && rep.numPlaced == 1 : placed1 && rep.numPlaced == 2,
			"with %s of the model's cameras pool images block 1 is %s, %s, %u blocks placed (%s expected)", allPool ? "every one" : "one",
			placed1 ? "placed" : "unplaced", Released(ring.scene, ring.blocks[1]) ? "released" : "not released", rep.numPlaced,
			allPool ? "unplaced, released, 1 placed" : "placed, 2 placed");
	}

	// 4. The anchor: on a ring of four whose block 0 has no cross match to any other, the averaging gives block 0 a model
	// of its own and blocks 1-3 one of 30 images. Anchored at block 0 the merge places block 0 alone and keeps blocks 1-3
	// whole; unanchored it merges the heavier model.
	for (const uint32_t anchor : {0u, NO_ID}) {
		RingFixture ring({4, CAMS});
		CutRing(ring, {{0, 1}, {3, 0}});
		const MergeReport rep = ring.Merge(false, anchor);
		const bool placed0 = rep.poses[0].state == BlockPose::ADMITTED;
		TEST_CHECK(anchor == 0 ? rep.numPlaced == 1 && placed0 : rep.numPlaced == 3 && !placed0,
			"%s, %u blocks placed, block 0 %s (%s expected)", anchor == 0 ? "anchored at block 0" : "unanchored", rep.numPlaced,
			placed0 ? "placed" : "not placed", anchor == 0 ? "block 0 alone" : "blocks 1-3");
	}
	return scope.Pass("round 1 kept the cut-off block whole, the regrouped model was placed again and the kept block released; "
		"the loop ends in 3 rounds with one half posed; a seam on pool cameras alone is refused; the anchor's model is merged");
}
/*----------------------------------------------------------------*/

// An image of a block left with observations on `keep` tracks only, fewer than the cells the weak-image filter asks
// an image's points to cover (FilterWeaklyConnectedImages); the other observations keep their order, the inliers first
static void ThinImage(Scene& block, IIndex image, unsigned keep)
{
	unsigned numSeen = 0;
	for (Track& track : block.tracks)
		FOREACH(i, track.observations)
			if (track.observations[i].imageID == image) {
				if (numSeen++ >= keep) {
					if (i < track.numInliers)
						--track.numInliers;
					track.observations.RemoveAtMove(i);
				}
				break;
			}
}

// The weak-image filter before the merge (SceneBlocks::MergeInRounds, FilterWeakImages), on an uncut ring of four blocks
// some of whose images are left with points in WEAK_TRACKS cells of the 15 the filter asks for (ThinImage):
// 1. one such image in a block: the block is placed without it, the image ends in the global scene with its features and
//    no pose, every other image is posed;
// 2. every image of one block: the block is left without a posed image, hands its images to the pool and is removed, and
//    the three other blocks, still a chain, are merged in the one round;
// 3. every image of every block: no block is left and the merge ends before its first round, every image handed to the pool;
// 4. an image of the merged model: on the ring cut in two halves that share no match, one round run by hand merges one half
//    and the regrouping makes it block 0; with an image of it thinned then, the merge's own rounds take it from the model:
//    it ends in the global scene with its features and without the pose the earlier round left its shell there, the rest of
//    the model stays posed, and the other half is released in the second round;
// 5. such an image registered again: on the ring with block 3 cut off, the round run by hand merges blocks 0 and 1 (block 2's
//    cameras held for pool images, which support no reading). The merge's first round takes the weak image from the model and
//    places block 2, its second gives the image tracks from its pairs and the pool step registers it, which is no image
//    entered: with no block placed either the third round releases block 3 (counted as entered, the image costs a round more).
bool MergeWeakImagesTest()
{
	TestScope scope("MergeWeakImagesTest");
	constexpr unsigned NUM_BLOCKS = 4, CAMS = 10, WEAK_TRACKS = 5;
	constexpr uint32_t WEAK_BLOCK = 2;
	constexpr IIndex WEAK_LOCAL = 3;
	const ReconstructionConfig config;
	ResectionConfig resectionCfg = config.resectionCfg;
	resectionCfg.DeriveBAConfigs(config.baConfig);
	for (const unsigned part : {1u, 2u, 3u}) {
		RingFixture ring({NUM_BLOCKS, CAMS});
		ring.Split();
		// the weak images, by global ID
		std::set<IIndex> weak;
		FOREACH(b, ring.split.blocks)
			FOREACH(l, ring.split.blocks[b].images)
				if (part == 3 || (b == WEAK_BLOCK && (part == 2 || l == WEAK_LOCAL))) {
					ThinImage(ring.split.blocks[b], (IIndex)l, WEAK_TRACKS);
					weak.insert(ring.split.localToGlobal[b][l]);
				}
		const unsigned numRounds = ring.split.MergeInRounds(resectionCfg, config);
		unsigned numWeakReleased = 0, numOthersPosed = 0;
		FOREACH(g, ring.scene.images) {
			const Image& img = ring.scene.images[g];
			if (weak.count((IIndex)g))
				numWeakReleased += img.HasFeatures() && !img.HasPose() ? 1u : 0u;
			else
				numOthersPosed += img.HasPose() ? 1u : 0u;
		}
		const unsigned numOthers = NUM_BLOCKS * CAMS - (unsigned)weak.size();
		const unsigned numRoundsExpected = part == 3 ? 0u : 1u;
		TEST_CHECK(numRounds == numRoundsExpected && numWeakReleased == weak.size() && numOthersPosed == numOthers &&
			(part == 3 || ring.scene.status.nCalibratedImages == numOthers),
			"part %u: %u rounds (%u expected), %u of the %u weak images in the global scene with their features and no pose, "
			"%u of the %u other images posed, %u calibrated", part, numRounds, numRoundsExpected, numWeakReleased, (unsigned)weak.size(),
			numOthersPosed, numOthers, ring.scene.status.nCalibratedImages);
	}
	{
		RingFixture ring({NUM_BLOCKS, CAMS});
		ThinSeam(ring.scene, ring.blocks, 1, 2, 0, CAMS);
		ThinSeam(ring.scene, ring.blocks, 3, 0, 0, CAMS);
		ring.Split();
		const MergeReport rep = ring.Merge(false);
		std::vector<bool> placed(rep.poses.size());
		FOREACH(b, rep.poses)
			placed[b] = rep.poses[b].state == BlockPose::ADMITTED;
		ring.split.RegroupAfterMerge(placed);
		const IIndexArr model(ring.split.localToGlobal[0]);
		const IIndex weakImage = model[WEAK_LOCAL];
		ThinImage(ring.split.blocks[0], WEAK_LOCAL, WEAK_TRACKS);
		const unsigned numRounds = ring.split.MergeInRounds(resectionCfg, config);
		unsigned numModelPosed = 0;
		for (const IIndex g : model)
			numModelPosed += ring.scene.images[g].HasPose() ? 1u : 0u;
		const Image& weak = ring.scene.images[weakImage];
		TEST_CHECK(numRounds == 2 && model.size() == 2 * CAMS && weak.HasFeatures() && !weak.HasPose() && numModelPosed == model.size() - 1 &&
			ring.scene.status.nCalibratedImages == model.size() - 1,
			"part 4: %u rounds (2 expected), a model of %u images (%u expected), its weak image with features %d and pose %d (1 and 0 "
			"expected), %u of its images posed and %u calibrated (%u expected)", numRounds, (unsigned)model.size(), 2 * CAMS,
			(int)weak.HasFeatures(), (int)weak.HasPose(), numModelPosed, ring.scene.status.nCalibratedImages, (unsigned)model.size() - 1);
	}
	{
		RingFixture ring({NUM_BLOCKS, CAMS});
		ThinSeam(ring.scene, ring.blocks, 2, 3, 0, CAMS);
		ThinSeam(ring.scene, ring.blocks, 3, 0, 0, CAMS);
		ring.Split();
		const MergeReport rep = ring.Merge(false, 0u, std::unordered_set<IIndex>(ring.blocks[2].begin(), ring.blocks[2].end()));
		std::vector<bool> placed(rep.poses.size());
		FOREACH(b, rep.poses)
			placed[b] = rep.poses[b].state == BlockPose::ADMITTED;
		ring.split.RegroupAfterMerge(placed);
		const unsigned numModel = (unsigned)ring.split.localToGlobal[0].size();
		const IIndex weakImage = ring.split.localToGlobal[0][WEAK_LOCAL];
		ThinImage(ring.split.blocks[0], WEAK_LOCAL, WEAK_TRACKS);
		const unsigned numRounds = ring.split.MergeInRounds(resectionCfg, config);
		// the tracks the image is an inlier of: the five it was left unless the merge built its tracks again from its pairs,
		// which it does only for an image in no block
		unsigned numWeakTracks = 0;
		for (const Track& track : ring.scene.tracks)
			for (uint8_t i = 0; i < track.numInliers; ++i)
				numWeakTracks += track.observations[i].imageID == weakImage ? 1u : 0u;
		TEST_CHECK(numModel == 2 * CAMS && numRounds == 3 && ring.scene.images[weakImage].HasPose() && numWeakTracks > WEAK_TRACKS &&
			ring.scene.status.nCalibratedImages == 3 * CAMS,
			"part 5: a model of %u images by hand (%u expected), %u rounds (3 expected), the weak image with pose %d (1 expected) on "
			"%u tracks (more than %u expected), %u calibrated images (%u expected)", numModel, 2 * CAMS, numRounds,
			(int)ring.scene.images[weakImage].HasPose(), numWeakTracks, WEAK_TRACKS, ring.scene.status.nCalibratedImages, 3 * CAMS);
	}
	return scope.Pass("a weak image leaves its block before the merge and the block is placed without it; a block of weak images "
		"alone is handed to the pool and the others are merged; with no block left every image is handed to the pool; a weak "
		"image of the merged model loses its pose there and in the global scene; registered again by the pool step it does not hold the rounds open");
}
/*----------------------------------------------------------------*/
// A block of two cameras a millimetre apart, and two of them side by side on the ring: a rig that thin sees no parallax,
// so its scale can only come from the other direction of its seams. It must still be placed, and at the right size.
// Of two such blocks side by side neither direction of the seam between them sees any parallax, so its readings observe
// no scale and do not enter the block graph: the two blocks take their place and size from the seams that reach the
// rest of the ring instead, and the merge has to come out at the ring's own scale all the same
bool UnobservableScaleTest()
{
	TestScope scope("UnobservableScaleTest");
	constexpr unsigned NUM_BLOCKS = 8;
	for (const std::set<uint32_t>& tiny : {std::set<uint32_t>{3}, std::set<uint32_t>{3, 4}}) {
		RingSceneConfig cfg{NUM_BLOCKS, 10};
		cfg.tinyBlocks = tiny;
		cfg.noisePx = 0;
		RingFixture ring(cfg);
		ring.Split();
		const MergeReport rep = ring.Merge();
		// the position bar is what proves the tiny blocks came out at the ring's own scale
		const String what = String::FormatString("%u tiny block(s)", (unsigned)tiny.size());
		if (!CheckRingMerge(scope, what, rep, ring.scene, ring.truth, NUM_BLOCKS, 0.1, 0.001))
			return false;
		for (const uint32_t b : tiny)
			for (const IIndex image : ring.blocks[b])
				TEST_CHECK(ring.scene.images[image].IsValid(), "%s: image %u of the two-camera block %u is not registered", what.c_str(), image, b);
		// the readings between the two shallow rigs observe no scale, so none enters the block graph
		if (tiny.size() == 2)
			for (const SeamCandidate& c : rep.candidates)
				TEST_CHECK(!(tiny.count(c.sceneA) && tiny.count(c.sceneB)), "%s: a reading (%u, %u) between two rigs a millimetre wide "
					"entered the block graph (%s)", what.c_str(), c.sceneA, c.sceneB,
					c.source == SeamCandidate::RIG_B_ON_A ? "the rig of B on A" : "the rig of A on B");
	}
	return scope.Pass("a block of two cameras a millimetre apart, and two side by side, are placed at the ring's scale, and no reading "
		"between the two shallow rigs enters the block graph");
}
/*----------------------------------------------------------------*/

// What the verified image pairs across a seam say. Block 1 of a ring keeps a thinned share of its true evidence toward
// block 0 (keepFraction), and its first plantedCameras cameras are given block 0's structure turned by turnDeg on top
// of it, so the planted correspondences can outnumber the true ones in the direction that registers those cameras and
// that direction measures a similarity of its own; the verified pairs across hold against the planted one and for the
// true one. Handing out the first numBlocks blocks; returns the number of tracks planted.
static unsigned PlantAmbiguousPair(RingFixture& ring, unsigned numBlocks, REAL keepFraction, REAL turnDeg, unsigned plantedCameras = 3)
{
	DropCrossObservations(ring.scene, ring.blocks, 1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}, 0, keepFraction);
	const unsigned planted = PlantFalseSeam(ring.scene, ring.blocks, 1, 0, UINT32_MAX, plantedCameras, RingRotation(turnDeg));
	SplitRing(ring, std::vector<IIndexArr>(ring.blocks.begin(), ring.blocks.begin() + numBlocks));
	return planted;
}

// The verified image pairs across a seam decide whether a reading stands, by inlier weight over all of them:
// 1. one pair, two directions and only one of them right (PlantAmbiguousPair, true evidence thinned to a share, the
//    structure planted turned by twenty degrees): the pair on its own gives the true reading alone, the planted one
//    refused by the pairs across it, which hold for the true one; with a third block, in a chain without a cycle, the
//    true reading carries the pair; and the same pair alone is merged;
// 2. the pair with none of its true evidence left: the planted direction is the only seam there is and three cameras
//    stand behind it, the cameras of block 0 it planted on having nothing to hold against it. With the pairs unread
//    (maxPairRotationResidual 0) block 1 is placed twenty degrees wrong; read, they refuse the reading and block 1 is
//    left unplaced;
// 3. the arithmetic: two thin pairs that disagree cannot outvote twenty-four heavy ones that agree.
bool PairAgreementMergeTest()
{
	TestScope scope("PairAgreementMergeTest");
	const RingSceneConfig cfg{6, 10};
	const GlobalAlignmentConfig alignCfg;
	{
		// how much of its true evidence block 1 keeps toward block 0, which has to leave the planted majority winning its
		// own direction while the true reading is still measured
		constexpr REAL KEEP_FRACTION = 0.15;
		// the bar the true reading is held to: a seam measured on a thinned sixth of its correspondences carries that much
		// less of the truth
		constexpr REAL MAX_ROTATION = 0.5;
		RingFixture ring(cfg);
		TEST_CHECK(PlantAmbiguousPair(ring, 3, KEEP_FRACTION, 20) > 0, "ambiguous pair: nothing could be planted on block 1's first three cameras");
		const SEACAVE::Transform truth(ring.applied[1] * ring.applied[0].Invert());
		const auto Angle = [&truth](const SeamCandidate& c) { return RotationErrorDeg(c.T.R, truth.R); };
		// the pair on its own: each direction is read, the planted one far off the truth with the verified pairs across it
		// against it, which keeps it out of the block graph, the true one within the bar with them for it
		{
			GlobalAlignment alignment(ring.scene, alignCfg);
			std::vector<SeamCandidate> cands;
			alignment.PrepareSeamEvidence(ring.split.blocks, ring.split.localToGlobal);
			alignment.EstimateSeamPair(ring.split.blocks, 0, 1, cands);
			TEST_CHECK(cands.size() == 1 && Angle(cands[0]) <= MAX_ROTATION && cands[0].pairs.Holds(), "ambiguous pair: %u readings of the "
				"pair entered, expected the true one alone (the planted one refused by its verified pairs), the first %.2f deg off (%.1f allowed)",
				(unsigned)cands.size(), cands.empty() ? -1. : Angle(cands[0]), MAX_ROTATION);
		}
		// three blocks, two seams in a chain and no cycle: the true reading carries the pair on its own
		const MergeReport rep = ring.Merge();
		const SeamCandidate* seam = ModelSeamOf(rep, 0, 1);
		TEST_CHECK(seam != NULL && Angle(*seam) <= MAX_ROTATION, "ambiguous pair: the model rests on %s hypothesis of the pair among three blocks",
			seam == NULL ? "no" : "a wrong");
		if (!CheckRingMerge(scope, "ambiguous pair, three blocks", rep, ring.scene, ring.truth, 3, 0.3, 0.001))
			return false;
	}
	{
		// the same pair alone
		RingFixture ring(cfg);
		TEST_CHECK(PlantAmbiguousPair(ring, 2, 0.15, 20) > 0, "ambiguous pair: nothing could be planted on block 1's first three cameras");
		const MergeReport rep = ring.Merge();
		TEST_CHECK(ModelSeamOf(rep, 0, 1) != NULL, "ambiguous pair: the lone pair was not merged");
		if (!CheckRingMerge(scope, "ambiguous pair, two blocks", rep, ring.scene, ring.truth, 2, 0.3, 0.001))
			return false;
	}
	// a merge consumes its blocks, so each run builds the fixture anew
	constexpr REAL PLANTED_TURN = 20; // degrees the planted structure is turned by (the other way lands out of every camera's view on this ring)
	for (const bool pairsRead : {false, true}) {
		RingFixture ring(cfg);
		TEST_CHECK(PlantAmbiguousPair(ring, 2, 0, PLANTED_TURN) > 0, "contradicted placement: nothing could be planted on block 1's first three cameras");
		GlobalAlignmentConfig mergeCfg;
		if (!pairsRead)
			mergeCfg.maxPairRotationResidual = 0;
		const SEACAVE::Transform truth(ring.applied[1] * ring.applied[0].Invert());
		const MergeReport rep = ring.Merge(true, NO_ID, {}, mergeCfg);
		if (!pairsRead) {
			// with the pairs unread, the planted seam carries block 1 into the model, turned
			const SeamCandidate* seam = NULL;
			for (const SeamCandidate& c : rep.candidates)
				if (c.sceneA == 0 && c.sceneB == 1)
					seam = &c;
			TEST_CHECK(seam != NULL && rep.numPlaced == 2, "contradicted placement: with the pairs unread %u blocks were placed on %s seam, "
				"so the scene does not pose the problem", rep.numPlaced, seam == NULL ? "no" : "the planted");
			const REAL angle = RotationErrorDeg(seam->T.R, truth.R);
			TEST_CHECK(angle >= ABS(PLANTED_TURN) / 2, "contradicted placement: the seam block 1 was placed on is only %.2f deg off the truth, "
				"so nothing was planted", angle);
		} else {
			// read, they refuse the reading (the one difference from the run above): block 1 is left unplaced rather than placed wrong
			TEST_CHECK(rep.numPlaced == 1, "contradicted placement: %u blocks were placed although every verified pair between them "
				"contradicts the placement", rep.numPlaced);
			for (const SeamCandidate& c : rep.candidates)
				TEST_CHECK(!(c.sceneA == 0 && c.sceneB == 1), "contradicted placement: a reading of the pair entered the block graph (%.0f for / %.0f against)",
					c.pairs.agree, c.pairs.disagree);
		}
	}
	// what the verified pairs across a seam decide is settled by inlier weight over all of them
	struct Case { float agree, disagree; unsigned numPairs; bool holds; float share; const char* why; };
	const Case cases[] = {
		{ 3084.f, 728.f, 26, true, 0.809f, "two thin pairs against twenty-four heavy pairs agreeing" },
		{ 0.f, 708.f, 4, false, 0.f, "every pair across says otherwise" },
		{ 369.f, 1066.f, 9, false, 0.257f, "most of the weight says otherwise" },
		{ 0.f, 0.f, 0, true, 1.f, "nothing read holds nothing against a hypothesis" },
	};
	for (const Case& c : cases) {
		PairAgreement pairs;
		pairs.agree = c.agree;
		pairs.disagree = c.disagree;
		pairs.numPairs = c.numPairs;
		TEST_CHECK(pairs.Holds() == c.holds, "agreement: %.0f for / %.0f against over %u pairs was %s; %s",
			c.agree, c.disagree, c.numPairs, c.holds ? "refused" : "held", c.why);
		TEST_CHECK(ABS(pairs.Share() - c.share) <= 0.001f, "agreement: %.0f for / %.0f against over %u pairs reproduced %.3f of the weight, "
			"expected %.3f; %s", c.agree, c.disagree, c.numPairs, pairs.Share(), c.share, c.why);
	}
	return scope.Pass("the verified pairs across a seam refuse a planted reading and keep the true one, refuse a seam its cameras carry, and "
		"decide by inlier weight");
}
/*----------------------------------------------------------------*/

// The closing of a ring on one seam, five arms, on rings of six blocks (the closing pair is (0, 5)) but the third:
// 1. biased: the drifting ring with the seam that closes it wrong by three quarters of a degree: the closing pair keeps no
//    true evidence and is read on planted correspondences alone, so the graph reads there a seam nothing can tell from a true
//    one, its error the size of the drift the ring already carries, within every bar. The averaging spreads it over the ring
//    like any other loop discrepancy. Either the closing block is left out, or the merged model still answers to the truth;
// 2. weak: the pair that closes a ring, left with two correspondences on each of thirty tracks in one direction: a thin seam
//    at best, held by the two or three cameras of each block the tracks concentrate on. The seams that were measured carry
//    the chain around to within a fraction of a degree of its far end, the closing pair's correspondences join their tracks,
//    and the final adjustment closes the ring on them;
// 3. rival modes: a pair (3, 4) on a ring of twelve blocks carrying two consistent readings on the cameras of block 4 nearest block 3, the true one
//    thinned on the first three and a copy of block 3's structure one period along the wall on the first five (the reading
//    a repeated facade gives). The copy weighs more than the truth (more cameras on its weaker side), so the
//    pair on its own could take it; only the cycle the pair closes, by the translation it puts block 4 at, can tell it is
//    the wrong one;
// 4. false chord: two cameras of block 1 given block 4's wall, turned half a turn about the ring axis onto block 1's own arc
//    and pushed out to 1.6 times the radius, so the implied placement sits outside block 4's own cameras while the chord
//    contradicts the way round the ring by half a turn. The entry test keeps it out of the block graph, or the cycles it
//    closes cut it;
// 5. second model: a ring cut open between blocks 0 and 5, whose pair (3, 4) is left a thin bridge measurable from one
//    direction only (78 correspondences carried by three cameras: two of block 3 and one of block 4): blocks 4 and 5 reach the
//    rest of the ring through it alone, and as it holds one hypothesis nothing rivals it and the ring is one model.
bool RingClosingSeamTest()
{
	TestScope scope("RingClosingSeamTest");
	constexpr unsigned NUM_BLOCKS = 6, CAMS = 10, CLOSING = NUM_BLOCKS - 1;
	const GlobalAlignmentConfig alignCfg;

	// 1. the closing seam wrong by 0.75 degrees
	{
		RingSceneConfig cfg{NUM_BLOCKS, CAMS};
		cfg.driftDegPerBlock = 0.5;
		cfg.driftScalePerBlock = 0.005;
		RingFixture ring(cfg);
		// the closing pair left with nothing of its own, then given block 0's structure turned about the ring axis: what block
		// 5's cameras then see of block 0 is a consistent seam, and the wrong one
		std::vector<unsigned> allCameras(CAMS);
		std::iota(allCameras.begin(), allCameras.end(), 0u);
		DropCrossObservations(ring.scene, ring.blocks, CLOSING, allCameras, 0, 0);
		const unsigned planted = PlantFalseSeam(ring.scene, ring.blocks, CLOSING, 0, 600, CAMS, RingRotation(0.75));
		TEST_CHECK(planted >= 150, "biased: only %u tracks of block 0 could be planted on block %u (150 expected)", planted, CLOSING);
		ComputePairsWeights(ring.scene);
		SplitRing(ring);
		const MergeReport rep = ring.Merge();
		// what the graph made of the biased seam: a seam this close to the drift the ring carries is one no residual can indict.
		// Either the model refuses the block the biased seam closes the ring through, or it takes it in and may not come out wrong
		if (rep.numPlaced >= NUM_BLOCKS) {
			if (!CheckRingMerge(scope, "biased", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.6, 0.005))
				return false;
			RunFinalAdjustment(ring.scene);
			if (!CheckRingMerge(scope, "biased, adjusted", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.3, 0.005))
				return false;
		}
		VERBOSE("  biased: the seam planted on %u tracks is %s, %u of %u blocks placed", planted,
			ModelSeamOf(rep, 0, CLOSING) != NULL ? "in the model" : "off", rep.numPlaced, NUM_BLOCKS);
	}

	// 2. the closing pair thin: two correspondences on each of thirty tracks
	{
		RingSceneConfig cfg{NUM_BLOCKS, CAMS};
		// the blocks come out slightly bent, so the chain has an error of its own to carry around the ring, and the keypoints
		// are read more precisely than the generator's default: what the chain accumulates is what the closing pair is off by
		// when its correspondences are joined
		cfg.driftDegPerBlock = 0.1;
		cfg.noisePx = 0.1;
		RingFixture ring(cfg);
		ThinSeam(ring.scene, ring.blocks, 0, CLOSING, 30, CAMS);
		SplitRing(ring);
		// what the pair is worth on its own evidence, while the seams that carry its two blocks into the chain are measured
		// as they always were
		{
			GlobalAlignment alignment(ring.scene, alignCfg);
			std::vector<SeamCandidate> candidates;
			TEST_CHECK(alignment.EstimateSeamCandidates(ring.split.blocks, ring.split.localToGlobal, candidates),
				"weak: the thinned ring yielded no seam candidate at all");
			bool chained[2] = {false, false};
			for (const SeamCandidate& c : candidates) {
				if (c.sceneA == 0 && c.sceneB == CLOSING)
					VERBOSE("  the thinned pair (0, %u) yields a reading of weight %.0f on %u inliers", CLOSING, c.weight, c.score.inliers);
				chained[0] = chained[0] || (c.sceneA == 0 && c.sceneB == 1);
				chained[1] = chained[1] || (c.sceneA == CLOSING - 1 && c.sceneB == CLOSING);
			}
			TEST_CHECK(chained[0] && chained[1], "weak: of the %u candidates measured, the seams (0, 1) and (%u, %u) that carry the thinned "
				"pair's own blocks are missing", (unsigned)candidates.size(), CLOSING - 1, CLOSING);
		}
		// the seams carry the chain round the ring, every one of them a bridge holding one hypothesis, and the closing pair's
		// correspondences join their tracks under the cross-pair guards for the final adjustment to close the ring on them
		const MergeReport rep = ring.Merge();
		if (!CheckRingMerge(scope, "weak", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.6, 0.005))
			return false;
		RunFinalAdjustment(ring.scene);
		if (!CheckRingMerge(scope, "weak, adjusted", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.3, 0.005))
			return false;
	}

	// 3. a repeated facade: the truth thinned against a heavier copy one period along the wall
	{
		constexpr unsigned RIVAL_BLOCKS = 12; // the copy outweighs the truth only where the cameras of both blocks are as close as on this ring
		constexpr uint32_t PAIR_A = 3, PAIR_B = 4;
		constexpr REAL PERIOD = 5;   // how far along the wall the copy sits, in the ring's units
		constexpr REAL KEEP = 0.4;   // the share of its true evidence block PAIR_B keeps on its first three cameras
		RingFixture ring({RIVAL_BLOCKS, CAMS});
		DropCrossObservations(ring.scene, ring.blocks, PAIR_B, {3, 4, 5, 6, 7, 8, 9}, PAIR_A, 0);
		DropCrossObservations(ring.scene, ring.blocks, PAIR_B, {0, 1, 2}, PAIR_A, KEEP);
		const unsigned planted = PlantFalseSeam(ring.scene, ring.blocks, PAIR_B, PAIR_A, 600, 5,
			WallCopy(REAL(2) * REAL(M_PI) * PAIR_B / RIVAL_BLOCKS, PERIOD));
		TEST_CHECK(planted >= 150, "rival: only %u tracks of block %u could be copied onto block %u (150 expected)", planted, PAIR_A, PAIR_B);
		ComputePairsWeights(ring.scene);
		ring.Split();
		const REAL extentA = RingBlockExtents(ring.split.blocks)[PAIR_A];
		const SEACAVE::Transform truth(ring.applied[PAIR_B] * ring.applied[PAIR_A].Invert());
		// the verified pairs across the seam kept their true word and would refuse the copy on their own: left unread, the
		// cycle is what has to
		GlobalAlignmentConfig unreadCfg;
		unreadCfg.maxPairRotationResidual = 0;
		const MergeReport rep = ring.Merge(true, NO_ID, {}, unreadCfg);
		if (!CheckRingMerge(scope, "rival", rep, ring.scene, ring.truth, RIVAL_BLOCKS, kRingMergeRotation, kRingMergePosition))
			return false;
		RunFinalAdjustment(ring.scene);
		if (!CheckRingMerge(scope, "rival, adjusted", rep, ring.scene, ring.truth, RIVAL_BLOCKS, 0.1, 0.001))
			return false;
		// the pair's hypotheses: the copy, the heaviest, and the true one, the one the model rests on
		uint32_t right = NO_ID, copy = NO_ID;
		FOREACH(i, rep.candidates) {
			const SeamCandidate& c = rep.candidates[i];
			if (c.sceneA != PAIR_A || c.sceneB != PAIR_B)
				continue;
			const auto [rotation, translation] = SeamOffset(c.T, truth, extentA);
			VERBOSE("  rival: pair (%u, %u) hypothesis %u: weight %.0f, %.3f deg and %.2f%% off the truth", PAIR_A, PAIR_B, (unsigned)i,
				c.weight, rotation, translation * 100);
			// the truth within the graph bars, measured on a thinned share of its evidence; the copy a period away, past the
			// extent of the block
			if (rotation < alignCfg.maxGraphRotationResidual && translation < alignCfg.maxGraphTranslationResidual)
				right = (uint32_t)i;
			else if (translation > 1 && (copy == NO_ID || c.weight > rep.candidates[copy].weight))
				copy = (uint32_t)i;
		}
		TEST_CHECK(right != NO_ID && copy != NO_ID && rep.candidates[copy].weight > rep.candidates[right].weight, "rival: the pair holds %s true "
			"hypothesis and %s copy, the copy %s (both expected, the copy heavier than the truth)", right == NO_ID ? "no" : "a",
			copy == NO_ID ? "no" : "a", right == NO_ID || copy == NO_ID ? "-" : "lighter than the truth");
		const auto RestsOn = [&rep](uint32_t i) { return std::find(rep.modelSeams.begin(), rep.modelSeams.end(), i) != rep.modelSeams.end(); };
		TEST_CHECK(RestsOn(right) && !RestsOn(copy), "rival: the merged model rests on the %s (the true reading expected)",
			RestsOn(copy) ? "copy" : "neither of the two");
	}

	// 4. a chord that reads perfectly on its own evidence
	{
		constexpr uint32_t CHORD_RIG = 1, CHORD_POINTS = 4;
		RingFixture ring({NUM_BLOCKS, CAMS});
		SEACAVE::Transform wrong(RingRotation(180));
		wrong.scale = 1.6;
		const unsigned planted = PlantFalseSeam(ring.scene, ring.blocks, CHORD_RIG, CHORD_POINTS, 600, 2, wrong);
		TEST_CHECK(planted >= 150, "chord: only %u tracks of block %u could be planted on block %u (150 expected)", planted, CHORD_POINTS, CHORD_RIG);
		ring.Split();
		LogCapture log;
		const MergeReport rep = ring.Merge();
		// the chord is measured and either refused by the entry test or cut by the cycles it closes
		uint32_t chord = NO_ID;
		FOREACH(i, rep.candidates)
			if (rep.candidates[i].sceneA == CHORD_RIG && rep.candidates[i].sceneB == CHORD_POINTS)
				chord = (uint32_t)i;
		TEST_CHECK(log.Contains(String::FormatString("Seam (%u, %u) rig", CHORD_RIG, CHORD_POINTS).c_str()), "chord: the planted chord was never measured");
		TEST_CHECK(ModelSeamOf(rep, CHORD_RIG, CHORD_POINTS) == NULL && (chord == NO_ID || rep.candidates[chord].residualRotation > 30),
			"chord: the planted chord was kept or not indicted (rotation residual %.1f deg, over 30 expected)",
			chord == NO_ID ? 0.f : rep.candidates[chord].residualRotation);
		if (!CheckRingMerge(scope, "chord", rep, ring.scene, ring.truth, NUM_BLOCKS, kRingMergeRotation, kRingMergePosition))
			return false;
	}

	// 5. the ring cut open with one thin bridge
	{
		constexpr uint32_t BRIDGE_A = 3, BRIDGE_B = 4;
		RingSceneConfig cfg{NUM_BLOCKS, CAMS};
		cfg.noisePx = 0;
		RingFixture ring(cfg);
		// the ring cut open between blocks 0 and 5; and of what blocks 3 and 4 share, only block 3's last three cameras keep
		// anything, of which 48 tracks then keep two observations there and one on block 4's first three cameras. That leaves the
		// pair 78 correspondences in the direction "the rig of block 4 on the points of block 3" and none in the other, carried by
		// three cameras -- two of block 3 and one of block 4 -- so the pair is read in that one direction alone, a bridge its two
		// cameras a side hold
		ThinSeam(ring.scene, ring.blocks, 0, CLOSING, 0, 0);
		DropCrossObservations(ring.scene, ring.blocks, BRIDGE_A, {0, 1, 2, 3, 4, 5, 6}, BRIDGE_B, 0);
		ThinSeam(ring.scene, ring.blocks, BRIDGE_A, BRIDGE_B, 48, 3);
		ring.Split();
		// what the merge starts from: one hypothesis on the bridge, and nothing at all on (0, 5)
		{
			GlobalAlignment alignment(ring.scene, alignCfg);
			std::vector<SeamCandidate> candidates;
			alignment.EstimateSeamCandidates(ring.split.blocks, ring.split.localToGlobal, candidates);
			unsigned numBridges = 0;
			for (const SeamCandidate& c : candidates) {
				TEST_CHECK(!(c.sceneA == 0 && c.sceneB == CLOSING), "second model: the severed pair (0, %u) still yielded a hypothesis of weight %.0f",
					CLOSING, c.weight);
				numBridges += c.sceneA == BRIDGE_A && c.sceneB == BRIDGE_B ? 1u : 0u;
			}
			TEST_CHECK(numBridges == 1, "second model: the pair (%u, %u) yielded %u hypotheses, expected exactly one", BRIDGE_A, BRIDGE_B, numBridges);
		}
		const MergeReport rep = ring.Merge();
		if (!CheckRingMerge(scope, "second model", rep, ring.scene, ring.truth, NUM_BLOCKS, 0.1, 0.001))
			return false;
		// the bridge is what carries blocks 4 and 5, and the merged model rests on it
		TEST_CHECK(ModelSeamOf(rep, BRIDGE_A, BRIDGE_B) != NULL, "second model: the bridge is not among the %u seams the merged model rests on",
			(unsigned)rep.modelSeams.size());
	}
	return scope.Pass("the ring closes on a biased seam within the truth or leaves its block out, on a thin seam through the final adjustment, "
		"on the true reading against a heavier copy, past a planted chord, and as one model over a lone bridge");
}
/*----------------------------------------------------------------*/

// A seam one camera holds is not a seam. Three blocks of a ring (3 blocks of 16 cameras), each joined to the other two:
// 1. a pair whose every cross pair runs through one camera of block 1: block 0's cameras register against block 1's points
//    through that one camera's features, and the camera on the other side holds every correspondence of block 1's side of
//    the seam. However many correspondences that camera holds, the seam weighs what one camera weighs, and as a bridge, which
//    no cycle corroborates, it places no block;
// 2. a pair seen through one camera on one side gives no reading that enters the block graph (a block is joined by all its
//    cameras together, never by one camera's pose): the cross pairs of (0, 2) kept only where they hold block 0's first
//    camera give no reading of (0, 2) (the direction of that single camera and the rig of block 2 on block 0's points, whose
//    weaker side that one camera supports, are both refused; the test does not tell which bar refuses which) and the ring is
//    placed whole through (0, 1) and (1, 2); then (0, 2) and (1, 2) each kept to one cross pair (one camera a side): block 2
//    has no rig reading, stays unplaced, and its images are released (merged with their features, without poses);
// 3. a reading observes a scale only when two of its rig's cameras each hold minCameraInliers of its inliers: the cross pairs
//    of (0, 2) kept only on two cameras of block 2. In the thin arm the keypoints of the second camera are displaced far past
//    the reprojection bar but for a few, so it holds fewer than minCameraInliers inliers of the direction of block 2's rig on
//    block 0's points while its correspondences still hold on block 2's points seen by block 0's cameras: two cameras support
//    each side of the seam (the other direction's reading enters with the weight the cameras bar asks) and the rig reading is
//    refused for its scale alone. With the second camera's keypoints in place it observes one and enters.
//    The thin arm is read a second time with no bar on the rig's spread: one camera has no spread, and the reading must
//    still be refused.
bool SeamCameraSupportTest()
{
	TestScope scope("SeamCameraSupportTest");
	const RingSceneConfig cfg{3, 16};
	const GlobalAlignmentConfig alignCfg;

	// 1. one camera holds block 1's side of the pair (0, 1)
	{
		constexpr unsigned MIN_CORRESPONDENCES = 300; // what the one camera has to hold for the test to mean anything
		RingFixture ring(cfg);
		const IIndex door = ring.blocks[1][0];
		KeepCrossPairs(ring.scene, ring.blocks[0], ring.blocks[1], [door](IIndex, IIndex imgB) { return imgB == door; });
		SplitRing(ring, {ring.blocks[0], ring.blocks[1]});
		const std::set<IIndex> block1(ring.blocks[1].begin(), ring.blocks[1].end());
		{
			GlobalAlignment alignment(ring.scene, alignCfg);
			alignment.PrepareSeamEvidence(ring.split.blocks, ring.split.localToGlobal);
			std::vector<SeamObservation> observations;
			std::vector<SeamCorrespondence> correspondences;
			alignment.CollectPairObservations(ring.split.blocks, 0, 1, observations, correspondences);
			std::map<IIndex, unsigned> perCamera;
			for (const SeamObservation& obs : observations)
				for (const IIndex image : {obs.rigImage, obs.pointImage})
					if (block1.count(image))
						++perCamera[image];
			TEST_CHECK(perCamera.size() == 1 && perCamera.begin()->second >= MIN_CORRESPONDENCES, "one side: block 1's side of the seam spans %u "
				"cameras, the first holding %u correspondences (one camera holding at least %u expected)", (unsigned)perCamera.size(),
				perCamera.empty() ? 0u : perCamera.begin()->second, MIN_CORRESPONDENCES);
			std::vector<SeamCandidate> candidates;
			alignment.EstimateSeamPair(ring.split.blocks, 0, 1, candidates);
			for (const SeamCandidate& c : candidates)
				TEST_CHECK(c.score.Weight() <= 1, "one side: the seam held by one camera's %u correspondences yielded a hypothesis of %u cameras",
					perCamera.begin()->second, c.score.Weight());
		}
		const MergeReport rep = ring.Merge();
		TEST_CHECK(rep.numPlaced == 1, "one side: %u blocks placed on a seam one camera holds (1 expected)", rep.numPlaced);
	}

	// 2. a direction seen by one camera gives no reading, and a block joined by single cameras only is released
	{
		RingFixture ring(cfg);
		const IIndex door0 = ring.blocks[0].front();
		KeepCrossPairs(ring.scene, ring.blocks[0], ring.blocks[2], [door0](IIndex imgA, IIndex) { return imgA == door0; });
		ring.Split();
		GlobalAlignment alignment(ring.scene, alignCfg);
		std::vector<SeamCandidate> candidates;
		alignment.EstimateSeamCandidates(ring.split.blocks, ring.split.localToGlobal, candidates);
		unsigned numReadings = 0, numRig = 0;
		for (const SeamCandidate& c : candidates)
			if (c.sceneA == 0 && c.sceneB == 2) {
				++numReadings;
				numRig += c.source == SeamCandidate::RIG_B_ON_A ? 1u : 0u;
			}
		TEST_CHECK(numReadings == 0, "one direction: the pair (0, 2) seen by one camera of block 0 gave %u readings, %u of them block 2's rig on "
			"block 0's points (none expected: one camera holds a side)", numReadings, numRig);
		const MergeReport rep = ring.Merge();
		if (!CheckRingMerge(scope, "one direction", rep, ring.scene, ring.truth, cfg.numBlocks, kRingMergeRotation, kRingMergePosition))
			return false;
	}
	{
		RingFixture ring(cfg);
		const IIndex door0 = ring.blocks[0].front(), door1 = ring.blocks[1].back(), door2 = ring.blocks[2].back(), door2b = ring.blocks[2].front();
		KeepCrossPairs(ring.scene, ring.blocks[0], ring.blocks[2], [=](IIndex imgA, IIndex imgB) { return imgA == door0 && imgB == door2; });
		KeepCrossPairs(ring.scene, ring.blocks[1], ring.blocks[2], [=](IIndex imgA, IIndex imgB) { return imgA == door1 && imgB == door2b; });
		ring.Split();
		const MergeReport rep = ring.Merge(true);
		bool released = rep.numPlaced == 2 && rep.poses[2].state != BlockPose::ADMITTED;
		for (const IIndex g : ring.blocks[2])
			released = released && ring.scene.images[g].HasFeatures() && !ring.scene.images[g].HasPose();
		TEST_CHECK(released, "one direction: block 2, joined by one camera a side only, %s; %u blocks placed (2 expected, block 2 released without poses)",
			rep.poses[2].state == BlockPose::ADMITTED ? "was placed" : "is not released whole", rep.numPlaced);
	}

	// 3. a rig whose second camera holds too few inliers observes no scale
	{
		const auto ReadPair02 = [&](bool displaceSecond, const GlobalAlignmentConfig& readCfg) {
			RingFixture ring(cfg);
			const std::set<IIndex> a(ring.blocks[0].begin(), ring.blocks[0].end());
			const IIndex first = ring.blocks[2].back(), second = ring.blocks[2][ring.blocks[2].size() - 2];
			KeepCrossPairs(ring.scene, ring.blocks[0], ring.blocks[2], [=](IIndex, IIndex imgB) { return imgB == first || imgB == second; });
			ring.Split();
			// its matched keypoints displaced far past the bar but those of minCameraInliers / 2 matches (the blocks' points are
			// triangulated by now, which would have dropped them): block 2's rig keeps the few inliers that fix its scale to its
			// first camera, the second one holding fewer than minCameraInliers of them, while its correspondences still hold on
			// block 2's points seen by block 0's cameras
			if (displaceSecond) {
				std::map<uint32_t, unsigned> numMatches;
				for (const ImagePair& pair : ring.scene.pairs)
					if ((pair.ID1 == second && a.count(pair.ID2)) || (pair.ID2 == second && a.count(pair.ID1)))
						for (const DMatch& match : pair.matches)
							++numMatches[pair.ID1 == second ? match.queryIdx : match.trainIdx];
				std::mt19937 rng(cfg.seed + 2);
				std::uniform_real_distribution<float> shift(-150.f, 150.f);
				unsigned budget = alignCfg.minCameraInliers / 2;
				FOREACH(l, ring.split.localToGlobal[2])
					if (ring.split.localToGlobal[2][l] == second)
						for (const auto& [feature, count] : numMatches) {
							if (count <= budget) {
								budget -= count;
								continue;
							}
							ring.split.blocks[2].images[l].keypoints[feature].pt += cv::Point2f(shift(rng), shift(rng));
						}
			}
			GlobalAlignment alignment(ring.scene, readCfg);
			alignment.PrepareSeamEvidence(ring.split.blocks, ring.split.localToGlobal);
			std::vector<SeamCandidate> readings;
			alignment.EstimateSeamPair(ring.split.blocks, 0, 2, readings);
			return readings;
		};
		const std::vector<SeamCandidate> thin = ReadPair02(true, alignCfg), full = ReadPair02(false, alignCfg);
		const SeamCandidate* const thinOther = ReadingOf(thin, SeamCandidate::RIG_A_ON_B);
		TEST_CHECK(ReadingOf(thin, SeamCandidate::RIG_B_ON_A) == NULL && thinOther != NULL && thinOther->score.Weight() >= alignCfg.minSeamWeight &&
			ReadingOf(full, SeamCandidate::RIG_B_ON_A) != NULL, "rig scale: the rig of block 2 with one camera holding inliers %s the block graph (it "
			"observes no scale: not expected), the seam %s weigh %u cameras on its weaker side with the other direction (%u expected), the rig of two "
			"cameras holding theirs %s (expected)", ReadingOf(thin, SeamCandidate::RIG_B_ON_A) ? "entered" : "did not enter",
			thinOther ? "did" : "did not", thinOther ? thinOther->score.Weight() : 0u, alignCfg.minSeamWeight,
			ReadingOf(full, SeamCandidate::RIG_B_ON_A) ? "entered" : "did not enter");
		// the rule is two cameras, whatever the bar on their spread: one camera has no spread, and with the bar at 0 (anything observes a
		// scale) the rig of one camera holding inliers must still observe none
		GlobalAlignmentConfig noSpreadBar = alignCfg;
		noSpreadBar.minRigSpreadRatio = 0.f;
		const std::vector<SeamCandidate> thinNoBar = ReadPair02(true, noSpreadBar);
		TEST_CHECK(ReadingOf(thinNoBar, SeamCandidate::RIG_B_ON_A) == NULL, "rig scale with no bar on the spread: the rig of block 2 with one camera holding "
			"inliers entered the block graph (one camera observes no scale whatever the bar)");
	}
	return scope.Pass("a seam one camera holds weighs one camera and places no block, a pair seen by one camera gives no reading and a block joined by "
		"single cameras only is released, a rig whose second camera holds too few inliers observes no scale and does not enter");
}
/*----------------------------------------------------------------*/

// A block reconstructed folded: its two halves never saw each other, and the second sits turned in the block's own
// frame. No pose of the block can answer to both its neighbours, and the cycle of the ring says so: one of its two seams is
// off the solution. The fold must not put the model wrong -- every other block lands on the truth -- whatever the half of
// it its kept seam carries brings along. Two arms, the fold of half a turn and that of a symmetric scene (45 degrees, each half
// seen by one neighbour). Twenty cameras per block, so either half of the fold is still a block of ten views.
bool FoldedBlockTest()
{
	TestScope scope("FoldedBlockTest");
	constexpr unsigned NUM_BLOCKS = 6, FOLDED = 3;
	for (const REAL foldDeg : {REAL(180), REAL(45)}) {
		RingSceneConfig cfg{NUM_BLOCKS, 20};
		cfg.foldedBlock = FOLDED;
		cfg.foldDeg = foldDeg;
		RingFixture ring(cfg);
		SplitRing(ring);
		const MergeReport rep = ring.Merge();
		// the folded block reaches the ring through two seams the cycle cannot both keep
		TEST_CHECK(ModelSeamOf(rep, FOLDED - 1, FOLDED) == NULL || ModelSeamOf(rep, FOLDED, FOLDED + 1) == NULL,
			"fold of %.0f deg: the merged model rests on both seams of the folded block", foldDeg);
		// every block but the folded one placed, and every one of their cameras on the truth
		TEST_CHECK(rep.numPlaced >= NUM_BLOCKS - 1, "fold of %.0f deg: %u of the %u blocks placed (%u expected)", foldDeg, rep.numPlaced, NUM_BLOCKS,
			NUM_BLOCKS - 1);
		IIndexArr unfolded;
		FOREACH(b, ring.blocks)
			if (b != FOLDED)
				unfolded.Join(ring.blocks[b]);
		const PoseError e = PoseErrors(ring.scene, ring.truth, unfolded);
		TEST_CHECK(e.rotationMedian <= kRingMergeRotation && e.positionMedian <= kRingMergePosition, "fold of %.0f deg: the unfolded blocks are "
			"%.4f deg and %.3f%% off the truth (%.1f deg and %.2f%% allowed)", foldDeg, e.rotationMedian, e.positionMedian * 100, kRingMergeRotation,
			kRingMergePosition * 100);
		VERBOSE("  fold of %.0f deg: %u blocks placed, the unfolded ones %.4f deg and %.4f%% from the truth", foldDeg, rep.numPlaced,
			e.rotationMedian, e.positionMedian * 100);
	}
	return scope.Pass("a block folded by half a turn or by 45 degrees leaves one of its two seams off the model and every other block on the truth");
}
/*----------------------------------------------------------------*/

// Block 4 of a six-block ring, cut loose from every block before the split (only blocks 3 and 5
// ever shared tracks with it, so thinning it against the rest is a no-op there): the merge has
// nothing to place it against, and has to leave it out, name it and its ten images in the report,
// and keep its own tracks intact for the post-merge resection to pick up.
bool UnplacedBlocksReportedTest()
{
	TestScope scope("UnplacedBlocksReportedTest");
	constexpr unsigned NUM_BLOCKS = 6, CUT = 4;
	RingFixture ring({NUM_BLOCKS, 10});
	for (uint32_t b = 0; b < NUM_BLOCKS; ++b)
		if (b != CUT)
			ThinSeam(ring.scene, ring.blocks, b, CUT, 0, 0);
	// the pair weights GenerateRingScene computed are stale once ThinSeam empties a pair's matches;
	// MapBlockPairLinks reads them, so they need to reflect what is actually left before the split
	ComputePairsWeights(ring.scene);
	ring.Split();
	LogCapture log;
	const MergeReport rep = ring.Merge();
	// the merge report holds no reason per block: it is read from the log
	TEST_CHECK(log.Contains("Block 4 not placed: no reading entered (no cross pairs, or every reading refused) (10 images)"),
		"the log does not report block %u by name", CUT);
	TEST_CHECK(rep.numPlaced == NUM_BLOCKS - 1, "%u blocks placed, expected %u", rep.numPlaced, NUM_BLOCKS - 1);
	const std::set<IIndex> expected(ring.blocks[CUT].begin(), ring.blocks[CUT].end());
	const std::set<IIndex> reported(rep.unplacedImages.begin(), rep.unplacedImages.end());
	TEST_CHECK(reported == expected, "%u images reported unplaced, expected exactly block %u's %u", (unsigned)reported.size(), CUT, (unsigned)expected.size());
	for (const IIndex image : ring.blocks[CUT])
		TEST_CHECK(!ring.scene.images[image].IsValid(), "block %u's image %u is registered", CUT, image);
	unsigned numOtherValid = 0;
	FOREACH(b, ring.blocks)
		if (b != CUT)
			for (const IIndex image : ring.blocks[b])
				numOtherValid += ring.scene.images[image].IsValid() ? 1u : 0u;
	TEST_CHECK(numOtherValid == ring.scene.images.size() - expected.size(), "%u of the other %u images are registered", numOtherValid,
		(unsigned)(ring.scene.images.size() - expected.size()));
	// block 4's own tracks must have survived the merge, observations and all, for the resection to have something to register
	// every one of its images against
	std::set<IIndex> imagesWithObs;
	for (const Track& track : ring.scene.tracks)
		for (const Observation& obs : track.observations)
			if (expected.count(obs.imageID))
				imagesWithObs.insert(obs.imageID);
	TEST_CHECK(imagesWithObs.size() == expected.size(), "only %u/%u of block %u's images kept an observation in scene.tracks",
		(unsigned)imagesWithObs.size(), (unsigned)expected.size(), CUT);
	return scope.Pass("block %u left out with its %u images, all kept an observation for the resection", CUT, (unsigned)expected.size());
}
/*----------------------------------------------------------------*/

// The correspondences of the model's block pairs that the merged block poses leave past the reprojection bar and within the
// loose one (a bent block is off by that at its far end)
static std::vector<SeamCorrespondence> FarSeamMatches(const MergeReport& rep, const std::vector<BlockPairCorrespondences>& blockPairs,
	const GlobalAlignmentConfig& alignCfg)
{
	const REAL bar = alignCfg.maxReprojError, looseBar = alignCfg.looseSeamFactor * alignCfg.maxReprojError;
	const std::set<std::pair<uint32_t, uint32_t>> modelPairs(ModelBlockPairs(rep));
	std::vector<SeamCorrespondence> farMatches;
	for (const BlockPairCorrespondences& pair : blockPairs) {
		if (!modelPairs.count(std::make_pair(pair.a, pair.b)))
			continue;
		const SEACAVE::Transform T(rep.poses[pair.b].T.Invert() * rep.poses[pair.a].T);
		FOREACH(i, pair.observations) {
			const REAL error = SeamPixelError(pair.observations[i], T);
			if (error > bar && error <= looseBar)
				farMatches.push_back(pair.correspondences[i]);
		}
	}
	return farMatches;
}

// The bent ring of BentRingTest merged once under alignCfg, and its far matches (FarSeamMatches) checked joined into one track at the
// merge. With the cross pairs as they are the closure and the cross-block join both can join them; with lightPairs they weigh under
// minCommonTracks, the closure takes none of them and only the cross-block join can; with alignCfg's join guard at the reprojection bar
// the guard refuses the matches the bend leaves past it and only the closure can.
static bool BentRingJoinCase(const TestScope& scope, const char* arm, const RingSceneConfig& cfg, const GlobalAlignmentConfig& alignCfg, bool lightPairs)
{
	constexpr float LIGHT_PAIR_WEIGHT = 12.f;
	constexpr unsigned MIN_FAR_MATCHES = 100;
	constexpr REAL MIN_JOINED_SHARE = 0.9;
	RingFixture ring(cfg);
	if (lightPairs) {
		ASSERT(LIGHT_PAIR_WEIGHT < (float)alignCfg.minCommonTracks);
		std::vector<uint32_t> blockOf(ring.scene.images.size(), NO_ID);
		FOREACH(b, ring.blocks)
			for (const IIndex image : ring.blocks[b])
				blockOf[image] = (uint32_t)b;
		for (ImagePair& pair : ring.scene.pairs)
			if (blockOf[pair.ID1] != blockOf[pair.ID2])
				pair.weightedInliers = LIGHT_PAIR_WEIGHT;
	}
	SplitRing(ring);
	const std::vector<BlockPairCorrespondences> blockPairs(CollectBlockPairCorrespondences(ring));
	const MergeReport rep = ring.Merge(true, NO_ID, {}, alignCfg);
	TEST_CHECK(rep.numPlaced == cfg.numBlocks, "%s: %u of %u bent blocks placed", arm, rep.numPlaced, cfg.numBlocks);
	const std::vector<SeamCorrespondence> farMatches(FarSeamMatches(rep, blockPairs, GlobalAlignmentConfig()));
	TEST_CHECK(farMatches.size() >= MIN_FAR_MATCHES, "%s: the bend left only %u matches past the bar (%u expected)", arm, (unsigned)farMatches.size(), MIN_FAR_MATCHES);
	std::unordered_map<PairIdx::PairIndex, uint32_t> trackOf;
	FOREACH(t, ring.scene.tracks)
		for (const Observation& obs : ring.scene.tracks[t].observations)
			trackOf.emplace(PairIdx(obs.imageID, obs.featureID).idx, (uint32_t)t);
	unsigned numJoined = 0;
	for (const SeamCorrespondence& corr : farMatches) {
		const auto itA = trackOf.find(PairIdx(corr.imageA, corr.featureA).idx), itB = trackOf.find(PairIdx(corr.imageB, corr.featureB).idx);
		numJoined += itA != trackOf.end() && itB != trackOf.end() && itA->second == itB->second ? 1u : 0u;
	}
	TEST_CHECK(numJoined >= MIN_JOINED_SHARE * farMatches.size(), "%s: %u of the %u far matches joined their tracks at the merge (%.0f%% expected)",
		arm, numJoined, (unsigned)farMatches.size(), MIN_JOINED_SHARE * 100);
	return true;
}

// A ring of blocks each reconstructed bent and stretched inside, along its own cameras: the rigid block poses miss the loop by
// more than a seam's bar while each seam is within its bar of the averaged poses, so the merged model leaves the correspondences
// of every seam a few pixels apart, past the reprojection bar and within the loose one. In one run:
// 1. every seam between neighbours stays in the merged model, so the closure joins every one of them;
// 2. those far matches join their tracks at the merge and hold them through the final adjustment, which ends them inliers
//    within the bar;
// 3. the adjustment straightens the blocks: the cameras it leaves are well under half the rigid block poses' errors, and on the truth.
// 4. the far matches have two ways to join, the closure (heavy pairs, no guard) and the cross-block join (every cross pair, under the
//    join guard): the ring is merged twice more, once with the guard at the reprojection bar (the closure alone joins them) and once
//    with the cross pairs lightened under minCommonTracks (the cross-block join alone), and joins them at the merge both times.
bool BentRingTest()
{
	TestScope scope("BentRingTest");
	constexpr unsigned MIN_FAR_MATCHES = 100;         // what the bend has to leave past the bar for the test to mean anything
	constexpr REAL MIN_KEPT_SHARE = 0.9;              // of those, the share that must end inliers within the bar
	constexpr REAL MAX_ROTATION_SHARE = 0.5, MAX_POSITION_SHARE = 0.5; // of the rigid block poses' errors
	RingSceneConfig cfg{6, 20};
	cfg.driftDegPerBlock = 1;
	cfg.driftScalePerBlock = 0.005;
	RingFixture ring(cfg);
	SplitRing(ring);
	const std::vector<BlockPairCorrespondences> blockPairs(CollectBlockPairCorrespondences(ring));
	const MergeReport rep = ring.Merge();
	TEST_CHECK(rep.numPlaced == cfg.numBlocks, "%u of %u bent blocks placed", rep.numPlaced, cfg.numBlocks);
	for (uint32_t b = 0; b < cfg.numBlocks; ++b) {
		const uint32_t next = (b + 1) % cfg.numBlocks;
		TEST_CHECK(ModelSeamOf(rep, MINF(b, next), MAXF(b, next)) != NULL, "the seam (%u, %u) is not in the merged model", MINF(b, next), MAXF(b, next));
	}
	// the far matches: the correspondences of the model's block pairs its block poses leave past the bar and within the loose bar
	const GlobalAlignmentConfig alignCfg;
	const REAL bar = alignCfg.maxReprojError;
	const std::vector<SeamCorrespondence> farMatches(FarSeamMatches(rep, blockPairs, alignCfg));
	TEST_CHECK(farMatches.size() >= MIN_FAR_MATCHES, "the bend left only %u matches past the bar (%u expected)", (unsigned)farMatches.size(), MIN_FAR_MATCHES);
	// where every observation ended up: its track and whether it is an inlier of it
	typedef std::unordered_map<PairIdx::PairIndex, std::pair<uint32_t, bool>> TrackIndex;
	const auto IndexTracks = [&]() {
		TrackIndex index;
		FOREACH(t, ring.scene.tracks) {
			const Track& track = ring.scene.tracks[t];
			FOREACH(k, track.observations)
				index.emplace(PairIdx(track.observations[k].imageID, track.observations[k].featureID).idx, std::make_pair((uint32_t)t, k < track.numInliers));
		}
		return index;
	};
	// the track both ends of a correspondence are on, NO_ID when they are on none or on two
	const auto TrackOf = [](const TrackIndex& index, const SeamCorrespondence& corr, bool& inliers) {
		const auto itA = index.find(PairIdx(corr.imageA, corr.featureA).idx), itB = index.find(PairIdx(corr.imageB, corr.featureB).idx);
		if (itA == index.end() || itB == index.end() || itA->second.first != itB->second.first)
			return (uint32_t)NO_ID;
		inliers = itA->second.second && itB->second.second;
		return itA->second.first;
	};
	unsigned numJoined = 0;
	{
		const TrackIndex index(IndexTracks());
		for (const SeamCorrespondence& corr : farMatches) {
			bool inliers;
			numJoined += TrackOf(index, corr, inliers) != NO_ID ? 1u : 0u;
		}
	}
	const PoseError rigid = PoseErrors(ring.scene, ring.truth);
	RunFinalAdjustment(ring.scene);
	// and what the final adjustment made of them: one track, both ends inliers within the bar
	unsigned numKept = 0;
	{
		const TrackIndex index(IndexTracks());
		for (const SeamCorrespondence& corr : farMatches) {
			bool inliers;
			const uint32_t t = TrackOf(index, corr, inliers);
			if (t == NO_ID || !inliers)
				continue;
			bool within = true;
			for (const auto& end : {std::make_pair(corr.imageA, corr.featureA), std::make_pair(corr.imageB, corr.featureB)}) {
				const Image& image = ring.scene.images[end.first];
				const auto [proj, valid] = image.ProjectPoint(ring.scene.tracks[t].position);
				within = within && valid && norm(proj - Point2(image.keypoints[end.second].pt)) <= bar;
			}
			numKept += within ? 1u : 0u;
		}
	}
	const PoseError adjusted = PoseErrors(ring.scene, ring.truth);
	VERBOSE("  %u far matches: %u joined at the merge, %u inliers within the bar after the final adjustment; rigid block poses %.4f deg and "
		"%.4f%%, adjusted %.4f deg and %.4f%% from the truth", (unsigned)farMatches.size(), numJoined, numKept, rigid.rotationMedian,
		rigid.positionMedian * 100, adjusted.rotationMedian, adjusted.positionMedian * 100);
	TEST_CHECK(numKept >= MIN_KEPT_SHARE * farMatches.size(), "%u of the %u far matches end inliers within the bar (%.0f%% expected)", numKept,
		(unsigned)farMatches.size(), MIN_KEPT_SHARE * 100);
	if (!CheckRingMerge(scope, "bent ring, adjusted", rep, ring.scene, ring.truth, cfg.numBlocks, 0.1, 0.002))
		return false;
	TEST_CHECK(adjusted.rotationMedian < rigid.rotationMedian * MAX_ROTATION_SHARE && adjusted.positionMedian < rigid.positionMedian * MAX_POSITION_SHARE,
		"the adjusted cameras are %.4f deg and %.4f%% off the truth, against the block poses' %.4f deg and %.4f%% (under half of them expected)",
		adjusted.rotationMedian, adjusted.positionMedian * 100, rigid.rotationMedian, rigid.positionMedian * 100);
	// the far matches have two ways to join: the closure (the heavy pairs' correspondences within the loose bar, no guard) and the cross-block
	// track join (every match of every cross pair, under the join guard); the run above holds both, each of the two runs below only one
	GlobalAlignmentConfig closureOnly;
	closureOnly.maxJoinReprojError = bar; // the join guard refuses the matches that sit past the reprojection bar: the closure alone joins them
	if (!BentRingJoinCase(scope, "closure alone", cfg, closureOnly, false) || !BentRingJoinCase(scope, "join guard alone", cfg, alignCfg, true))
		return false;
	return scope.Pass("%u far matches joined and kept through the final adjustment, every seam of the bent ring in the model, the cameras straightened to "
		"%.4f deg and %.4f%% from the truth, and they join by the closure alone (join guard at the bar) and by the cross-block join alone (light pairs)",
		(unsigned)farMatches.size(), adjusted.rotationMedian, adjusted.positionMedian * 100);
}
/*----------------------------------------------------------------*/

/*----------------------------------------------------------------*/

// ===============================================================================
// End-to-End Hierarchical SFM Tests
// ===============================================================================

// A scene split into blocks (SceneCluster), each block handed to the merge at the true poses of its cameras (SplitOnTruth),
// and merged back (GlobalAlignment::MergeScenes); every arm keeps the bars of the test it came from:
// 1. ten images of the default circle in blocks of 3-6, left in the true frame: the merge succeeds, at most two images are
//    left out, the keypoints the split moved into the blocks come back to at least all but two images, and every merged
//    track names global images;
// 2. 24 images every 15 degrees in blocks of 5-14, left in the true frame: at most two left out, every image within 5
//    degrees and 10% of the extent of the truth, 70% of the tracks back;
// 3. 20 images every 18 degrees in blocks of 5-12, each block moved by a random similarity: at most four left out, every
//    image within 8 degrees, half of the tracks back.
bool MergeRoundtripTest()
{
	TestScope scope("MergeRoundtripTest");
	struct Arm {
		const char* name; unsigned numImages, numPoints; REAL stepDeg; unsigned maxViews, minViews; bool everyCluster; bool moved;
		unsigned maxLeftOut; REAL maxRotation, maxPosition, minTrackShare; bool restoresKeypoints;
	};
	const Arm arms[] = {
		{"10 images", 10, 60, 15, 6, 3, true, false, 2, 180, FLT_MAX, 0, true},
		{"24 images", 24, 150, 15, 14, 5, false, false, 2, 5, 0.1, 0.7, false},
		{"20 images moved", 20, 120, 18, 12, 5, true, true, 4, 8, FLT_MAX, 0.5, false},
	};
	String summary;
	for (const Arm& arm : arms) {
		Scene scene;
		SceneConfig cfg;
		cfg.numImages = arm.numImages;
		cfg.numPoints = arm.numPoints;
		cfg.rotationAngleStep = arm.stepDeg;
		cfg.generateDescriptors = true;
		cfg.generatePairs = true;
		GenerateTestScene(scene, cfg);
		ComputePairsWeights(scene);
		const unsigned numTracks = (unsigned)scene.tracks.size();
		std::vector<Pose3D> truth;
		std::vector<size_t> numKeypoints;
		for (const Image& img : scene.images) {
			truth.emplace_back(img.R, img.C);
			numKeypoints.push_back(img.keypoints.size());
		}
		ClusterConfig clusterCfg;
		clusterCfg.maxViewsPerCluster = arm.maxViews;
		clusterCfg.minViewsPerCluster = arm.minViews;
		// the blocks are this fixture, and its handful of points makes every seam look thin to a rule written for a
		// capture's tracks: no cluster is merged away here for want of strong neighbours
		if (arm.everyCluster)
			clusterCfg.minClusterDegree = 0;
		const std::vector<IIndexArr> blocks(SceneCluster(scene, clusterCfg).Partition().clusters);
		TEST_CHECK(blocks.size() >= 2, "%s: %u blocks, the split is what is tested", arm.name, (unsigned)blocks.size());
		SceneBlocks split(scene);
		SplitOnTruth(scene, split, blocks, truth, arm.moved ? 123 : NO_ID);
		MergeReport report;
		GlobalAlignment alignment(scene, GlobalAlignmentConfig());
		TEST_CHECK(alignment.MergeScenes(split.blocks, split.localToGlobal, report), "%s: MergeScenes failed", arm.name);
		const PoseError e = PoseErrors(scene, truth);
		TEST_CHECK(e.count + arm.maxLeftOut >= arm.numImages && e.rotationMax <= arm.maxRotation && e.positionMax <= arm.maxPosition,
			"%s: %u of %u images merged (%u may be left out), %.3f deg and %.4f of the extent off the truth at most (%.1f and %.2f allowed)",
			arm.name, e.count, arm.numImages, arm.maxLeftOut, e.rotationMax, e.positionMax, arm.maxRotation, arm.maxPosition);
		TEST_CHECK(scene.tracks.size() >= arm.minTrackShare * numTracks, "%s: %u of %u tracks back (%.0f%% expected)", arm.name,
			(unsigned)scene.tracks.size(), numTracks, arm.minTrackShare * 100);
		for (const Track& track : scene.tracks)
			for (const Observation& obs : track.observations)
				TEST_CHECK(obs.imageID < scene.images.size(), "%s: a merged track names image %u, not a global image", arm.name, obs.imageID);
		if (arm.restoresKeypoints) {
			unsigned restored = 0;
			FOREACH(i, scene.images)
				if (scene.images[i].keypoints.size() == numKeypoints[i])
					++restored;
			TEST_CHECK(restored + 2 >= arm.numImages, "%s: only %u of %u images got their keypoints back", arm.name, restored, arm.numImages);
		}
		summary += String::FormatString("%s%s: %u blocks, %u merged, %.3f deg and %.4f off at most, %u of %u tracks", summary.empty() ? "" : "; ",
			arm.name, (unsigned)blocks.size(), e.count, e.rotationMax, e.positionMax, (unsigned)scene.tracks.size(), numTracks);
	}
	return scope.Pass("%s", summary.c_str());
}
/*----------------------------------------------------------------*/

// ===============================================================================
// Test 17: aligning two sub-scenes by registering one's cameras against the other's tracks
// ===============================================================================

// The two known similarities the sub-scenes are placed at: distinct rotations and translations,
// and scales half and double, so the seam has a real scale to recover in either direction.
static void KnownSubSceneTransforms(Transform transforms[2])
{
	std::mt19937 rng(7);
	transforms[0].R = GenerateRandomRotation(rng, 0.4);
	transforms[0].t = Point3(1.5, -2.0, 0.75);
	transforms[0].scale = 0.5;
	transforms[1].R = GenerateRandomRotation(rng, 0.9);
	transforms[1].t = Point3(-3.0, 0.5, 2.25);
	transforms[1].scale = 2.0;
}

// Two sub-scenes of one two-cluster scene, each placed at its own known similarity: poses are the
// truth mapped by transforms[s], tracks are rebuilt and triangulated in that frame. This is what a
// pair of finished sub-scenes looks like to the merge, without running a reconstruction.
static bool BuildTransformedSubScenes(
	Scene& scene, std::vector<Scene>& subScenes, std::vector<IIndexArr>& localToGlobals,
	std::vector<Pose3D>& gtPoses, const Transform transforms[2], bool hubBridge = false)
{
	GenerateTwoClusterScene(scene, 12, 12, 10, 60, 150, 42, hubBridge);
	gtPoses.resize(scene.images.size());
	for (unsigned i = 0; i < scene.images.size(); ++i) {
		gtPoses[i].R = scene.images[i].R;
		gtPoses[i].C = scene.images[i].C;
	}

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 14;
	clusterCfg.minViewsPerCluster = 5;
	SceneCluster cluster(scene, clusterCfg);
	localToGlobals.clear();
	subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);
	if (subScenes.size() != 2)
		return false;

	for (unsigned s = 0; s < 2; ++s) {
		Scene& sub = subScenes[s];
		for (IIndex localID = 0; localID < sub.images.size(); ++localID) {
			const IIndex globalID = localToGlobals[s][localID];
			sub.images[localID].R = gtPoses[globalID].R;
			sub.images[localID].C = gtPoses[globalID].C;
		}
		sub.RecomputeCalibratedImages();
		BuildTracks(sub);
		if (TriangulateTracks(sub) == 0)
			return false;
		sub.Transform(transforms[s]);
	}
	return true;
}

// Diagonal of the box the sub-scene's cameras span, the unit the translation error is judged in
static REAL SubSceneExtent(const Scene& subScene)
{
	AABB3 bbox(true);
	for (const Image& img : subScene.images)
		if (img.IsValid())
			bbox.InsertFull(img.C);
	return bbox.IsEmpty() ? REAL(0) : bbox.GetSize().norm();
}

// Error of one measured seam against the similarity the two sub-scenes were placed at. The score
// sums the three in comparable units (degrees and percent) so two measurements of the same seam can
// be ranked against each other.
struct SeamError {
	REAL rotation;    // degrees
	REAL scale;       // fraction
	REAL translation; // fraction of the destination sub-scene's camera extent
	REAL score;
};

static SeamError ComputeSeamError(const Transform& measured, const Transform& expected, REAL extent)
{
	SeamError err;
	err.rotation = R2D(ACOS(ComputeAngle(Matrix3x3(measured.R), Matrix3x3(expected.R))));
	err.scale = ABS(measured.scale / expected.scale - REAL(1));
	err.translation = norm(measured.t - expected.t) / extent;
	err.score = err.rotation + 100 * err.scale + 100 * err.translation;
	return err;
}

// Compare one measured seam against the similarity the two sub-scenes were placed at
static bool CheckSeamTransform(const char* what, const Transform& measured, const Transform& expected, REAL extent,
	REAL maxRotation = 0.1, REAL maxScale = 0.01, REAL maxTranslation = 0.01, SeamError* outErr = NULL)
{
	const SeamError err = ComputeSeamError(measured, expected, extent);
	if (outErr)
		*outErr = err;
	if (err.rotation > maxRotation || err.scale > maxScale || err.translation > maxTranslation) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: %s seam off by %.4f deg, %.2f%% scale, %.2f%% translation",
			what, err.rotation, err.scale * 100, err.translation * 100);
		return false;
	}
	VERBOSE("  %s seam: %.4f deg, %.3f%% scale, %.3f%% translation", what, err.rotation, err.scale * 100, err.translation * 100);
	return true;
}

// How one seam measurement is set up: which sub-scene contributes no 3D points (so only one
// direction of the camera alignment has correspondences at all), whether the bridge runs through a
// single image of the first cluster (so that side's rig holds one camera, too few to observe a
// scale, and the seam has to be verified against it instead), whether one sub-scene's cameras are
// moved away from its own points (so the two directions measure different similarities), and how
// much noise the observed bearings carry.
struct SeamCase {
	uint32_t blindScene{NO_ID};
	bool hubBridge{false};
	uint32_t rescaledPoses{NO_ID};
	REAL poseScale{2};
	REAL keypointNoise{0}; // pixels
};

// Measure the seam of two sub-scenes prepared as above, under one such setup: the readings the pair
// yields, false when the setup failed or the pair yielded none
static bool MeasureSeam(const Transform transforms[2], unsigned alignment, const SeamCase& setup,
	std::vector<SeamCandidate>& readings)
{
	readings.clear();
	Scene scene;
	std::vector<Scene> subScenes;
	std::vector<IIndexArr> localToGlobals;
	std::vector<Pose3D> gtPoses;
	if (!BuildTransformedSubScenes(scene, subScenes, localToGlobals, gtPoses, transforms, setup.hubBridge))
		return false;
	if (setup.blindScene != NO_ID)
		for (Track& track : subScenes[setup.blindScene].tracks)
			track.numInliers = 0;
	if (setup.rescaledPoses != NO_ID)
		for (Image& img : subScenes[setup.rescaledPoses].images)
			if (img.IsValid())
				img.C *= setup.poseScale;
	if (setup.keypointNoise > 0) {
		// the tracks were triangulated before this, so the 3D points stay exact and only the
		// bearings the two directions are estimated from move
		std::mt19937 rng(11);
		std::normal_distribution<float> noise(0.f, (float)setup.keypointNoise);
		for (Scene& subScene : subScenes)
			for (Image& img : subScene.images)
				for (cv::KeyPoint& keypoint : img.keypoints) {
					keypoint.pt.x += noise(rng);
					keypoint.pt.y += noise(rng);
				}
	}

	GlobalAlignmentConfig alignCfg;
	alignCfg.alignment = alignment;
	GlobalAlignment alignment_(scene, alignCfg);
	return alignment_.EstimateSeamCandidates(subScenes, localToGlobals, readings);
}

// The reading of the given direction among a pair's readings, NULL when the pair yielded none
static const SeamCandidate* ReadingOf(const std::vector<SeamCandidate>& readings, SeamCandidate::Source source)
{
	for (const SeamCandidate& c : readings)
		if (c.source == source)
			return &c;
	return NULL;
}

// One direction alone: the pair yields one reading, of that direction, explaining enough of the seam
static bool CheckOneDirectionReading(
	const char* what, const std::vector<SeamCandidate>& readings, SeamCandidate::Source source, unsigned minCommonTracks)
{
	if (readings.size() != 1 || readings.front().source != source || readings.front().score.inliers < minCommonTracks) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: %s came out as %u readings, the first of source %u with %u inliers",
			what, (unsigned)readings.size(), readings.empty() ? 0u : (unsigned)readings.front().source,
			readings.empty() ? 0u : readings.front().score.inliers);
		return false;
	}
	return true;
}

// Merge two such sub-scenes and check every image came back at the truth, up to the one global
// similarity the merged frame is free to choose.
static bool CheckMerge(const Transform transforms[2], unsigned alignment, const char* what)
{
	Scene scene;
	std::vector<Scene> subScenes;
	std::vector<IIndexArr> localToGlobals;
	std::vector<Pose3D> gtPoses;
	if (!BuildTransformedSubScenes(scene, subScenes, localToGlobals, gtPoses, transforms)) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: could not prepare the sub-scenes for the %s merge", what);
		return false;
	}

	GlobalAlignmentConfig alignCfg;
	alignCfg.alignment = alignment;
	GlobalAlignment merger(scene, alignCfg);
	MergeReport report;
	if (!merger.MergeScenes(subScenes, localToGlobals, report)) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: %s merge returned false", what);
		return false;
	}

	Point3Arr mergedCenters, gtCenters;
	FOREACH(i, scene.images) {
		if (!scene.images[i].IsValid()) {
			VERBOSE("HierarchicalCameraAlignmentTest FAILED: image %u invalid after the %s merge", i, what);
			return false;
		}
		mergedCenters.emplace_back(scene.images[i].C);
		gtCenters.emplace_back(gtPoses[i].C);
	}

	// one similarity for the whole merged scene, then every pose must sit on the truth under it
	Transform T;
	if (EstimateSimilarityTransform(mergedCenters, gtCenters, T) == 0) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: could not fit the %s merge to the truth", what);
		return false;
	}
	AABB3 gtBbox(true);
	for (const Point3& C : gtCenters)
		gtBbox.InsertFull(C);
	const REAL extent = gtBbox.GetSize().norm();
	REAL maxPos = 0, maxRot = 0;
	FOREACH(i, scene.images) {
		maxPos = MAXF(maxPos, norm(T * mergedCenters[i] - gtCenters[i]) / extent);
		maxRot = MAXF(maxRot, (REAL)R2D(ACOS(ComputeAngle(Matrix3x3(scene.images[i].R * T.R.t()), Matrix3x3(gtPoses[i].R)))));
	}
	if (maxPos > 0.01 || maxRot > 0.5) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: %s merge is %.2f%% and %.3f deg off the truth",
			what, maxPos * 100, maxRot);
		return false;
	}
	VERBOSE("  %s merge: %u images, %.3f%% position and %.4f deg from the truth",
		what, (unsigned)scene.images.size(), maxPos * 100, maxRot);
	return true;
}

// A scene of three clusters, each pair of them bridged by its own cross pairs over images no other
// bridge uses, so one seam can be made wrong without touching the two next to it.
static void GenerateThreeClusterScene(
	Scene& scene, unsigned clusterSize, unsigned numCrossPairs, unsigned matchesPerCrossPair, unsigned numPoints)
{
	SceneConfig cfg;
	cfg.randomSeed = 42;
	cfg.numImages = 3 * clusterSize;
	cfg.numPoints = numPoints;
	cfg.poseMode = SceneConfig::CIRCULAR_ARRANGEMENT;
	cfg.rotationAngleStep = 360.0 / cfg.numImages;
	cfg.generateDescriptors = true;
	cfg.generatePairs = true;
	GenerateTestScene(scene, cfg);

	std::vector<bool> keepCross(scene.pairs.size(), false);
	std::set<IIndex> used;
	for (unsigned first = 0; first < 3; ++first)
		for (unsigned second = first + 1; second < 3; ++second) {
			unsigned kept = 0;
			FOREACH(i, scene.pairs) {
				if (kept >= numCrossPairs)
					break;
				const ImagePair& pair = scene.pairs[i];
				const unsigned cluster1 = pair.ID1 / clusterSize, cluster2 = pair.ID2 / clusterSize;
				if (MINF(cluster1, cluster2) != first || MAXF(cluster1, cluster2) != second)
					continue;
				if (used.count(pair.ID1) || used.count(pair.ID2))
					continue;
				used.insert(pair.ID1);
				used.insert(pair.ID2);
				keepCross[i] = true;
				++kept;
			}
		}

	RFOREACH(i, scene.pairs) {
		ImagePair& pair = scene.pairs[i];
		if (pair.ID1 / clusterSize == pair.ID2 / clusterSize) {
			pair.weightSpatial = 10.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 10.f;
			pair.weightTriplet = 10.f;
		} else if (keepCross[i]) {
			if (pair.matches.size() > matchesPerCrossPair)
				pair.matches.resize(matchesPerCrossPair);
			pair.weightSpatial = 1.f;
			pair.overlapArea = 1.f; // the coverage, a full factor of the composite weight
			pair.weightConnectivity = 1.f;
			pair.weightTriplet = 0.f;
		} else {
			scene.pairs.RemoveAt(i);
		}
	}
}

// Three sub-scenes at three known similarities, with the seam between the first and the third made
// wrong: the third contributes no 3D points, so both of its seams rest on a single direction, and
// the cameras that bridge it to the first are moved away from its own points, so that one seam
// comes out at twice the scale it should while the other two stay right. The triangle is the only
// thing that can tell them apart.
static bool BuildTriangleSubScenes(
	Scene& scene, std::vector<Scene>& subScenes, std::vector<IIndexArr>& localToGlobals,
	std::vector<Pose3D>& gtPoses, Transform transforms[3], std::vector<IIndex>& perturbedGlobals)
{
	GenerateThreeClusterScene(scene, 12, 6, 80, 200);
	gtPoses.resize(scene.images.size());
	for (unsigned i = 0; i < scene.images.size(); ++i) {
		gtPoses[i].R = scene.images[i].R;
		gtPoses[i].C = scene.images[i].C;
	}

	ClusterConfig clusterCfg;
	clusterCfg.maxViewsPerCluster = 14;
	clusterCfg.minViewsPerCluster = 5;
	SceneCluster cluster(scene, clusterCfg);
	localToGlobals.clear();
	subScenes = ExtractBlocks(scene, cluster.Partition(), &localToGlobals);
	if (subScenes.size() != 3)
		return false;

	std::mt19937 rng(5);
	const REAL scales[3] = {0.5, 1.5, 2.0};
	for (unsigned s = 0; s < 3; ++s) {
		transforms[s].R = GenerateRandomRotation(rng, 0.3 + 0.3 * s);
		transforms[s].t = Point3(1.0 - s, 0.5 * s - 1.0, 2.0 * s - 1.5);
		transforms[s].scale = scales[s];
		Scene& sub = subScenes[s];
		for (IIndex localID = 0; localID < sub.images.size(); ++localID) {
			const IIndex globalID = localToGlobals[s][localID];
			sub.images[localID].R = gtPoses[globalID].R;
			sub.images[localID].C = gtPoses[globalID].C;
		}
		sub.RecomputeCalibratedImages();
		BuildTracks(sub);
		if (TriangulateTracks(sub) == 0)
			return false;
		sub.Transform(transforms[s]);
	}

	// the third sub-scene has no points to offer, so neither of its seams can be measured both ways
	for (Track& track : subScenes[2].tracks)
		track.numInliers = 0;

	// the images of the third sub-scene that bridge to the first, doubled away from its own points
	std::unordered_map<IIndex, std::pair<uint32_t, IIndex>> globalToLocal;
	for (uint32_t s = 0; s < 3; ++s)
		FOREACH(localID, localToGlobals[s])
			globalToLocal.emplace(localToGlobals[s][localID], std::make_pair(s, (IIndex)localID));
	std::set<IIndex> bridgeLocals;
	for (const ImagePair& pair : scene.pairs) {
		const auto it1 = globalToLocal.find(pair.ID1), it2 = globalToLocal.find(pair.ID2);
		if (it1 == globalToLocal.end() || it2 == globalToLocal.end())
			continue;
		if (it1->second.first == 2 && it2->second.first == 0)
			bridgeLocals.insert(it1->second.second);
		else if (it2->second.first == 2 && it1->second.first == 0)
			bridgeLocals.insert(it2->second.second);
	}
	perturbedGlobals.clear();
	for (IIndex localID : bridgeLocals) {
		subScenes[2].images[localID].C *= REAL(2);
		perturbedGlobals.push_back(localToGlobals[2][localID]);
	}
	// the sub-scenes carry the poses from here on: the global scene is the uncalibrated one the
	// merge writes its answer into
	FOREACH(i, scene.images)
		scene.images[i].InvalidatePose();
	scene.status.nCalibratedImages = 0;
	return bridgeLocals.size() >= 3;
}

// The seams of those three sub-scenes: all three must be measured, and the corrupted one must be the
// one that is wrong
static bool CheckTriangleSeams()
{
	Scene scene;
	std::vector<Scene> subScenes;
	std::vector<IIndexArr> localToGlobals;
	std::vector<Pose3D> gtPoses;
	Transform transforms[3];
	std::vector<IIndex> perturbedGlobals;
	if (!BuildTriangleSubScenes(scene, subScenes, localToGlobals, gtPoses, transforms, perturbedGlobals)) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: could not prepare the three sub-scenes");
		return false;
	}

	GlobalAlignmentConfig alignCfg;
	GlobalAlignment merger(scene, alignCfg);
	std::vector<SeamCandidate> candidates;
	merger.EstimateSeamCandidates(subScenes, localToGlobals, candidates);
	std::set<std::pair<uint32_t, uint32_t>> measured;
	for (const SeamCandidate& c : candidates)
		measured.emplace(c.sceneA, c.sceneB);
	if (measured.size() != 3) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: %u of the 3 seams of the triangle were measured",
			(unsigned)measured.size());
		return false;
	}
	for (const SeamCandidate& c : candidates) {
		const Transform expected = transforms[c.sceneB] * transforms[c.sceneA].Invert();
		const REAL errScale = ABS(c.T.scale / expected.scale - REAL(1));
		const bool corrupted = (c.sceneA == 0 && c.sceneB == 2);
		VERBOSE("  triangle seam (%u, %u): %.1f%% off in scale, weight %.0f",
			c.sceneA, c.sceneB, errScale * 100, c.weight);
		if (corrupted != (errScale > 0.5)) {
			VERBOSE("HierarchicalCameraAlignmentTest FAILED: seam (%u, %u) is %.1f%% off in scale, expected %s",
				c.sceneA, c.sceneB, errScale * 100, corrupted ? "the corrupted one" : "a right one");
			return false;
		}
	}
	return true;
}

// Global image ID -> local image index of one sub-scene
static std::unordered_map<IIndex, IIndex> LocalOf(const IIndexArr& localToGlobal)
{
	std::unordered_map<IIndex, IIndex> localOf;
	FOREACH(localID, localToGlobal)
		if (localToGlobal[localID] != NO_ID)
			localOf.emplace(localToGlobal[localID], (IIndex)localID);
	return localOf;
}

// A sub-scene left with two cameras a millimetre apart: the direction whose rig they are sees no
// parallax and cannot observe a scale, so the seam has to take one from the other direction.
static bool CheckUnobservableScale(const Transform transforms[2], const Transform& expected)
{
	Scene scene;
	std::vector<Scene> subScenes;
	std::vector<IIndexArr> localToGlobals;
	std::vector<Pose3D> gtPoses;
	if (!BuildTransformedSubScenes(scene, subScenes, localToGlobals, gtPoses, transforms)) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: could not prepare the sub-scenes for the shallow rig");
		return false;
	}
	// the generated points behind every keypoint, so the camera that moves can be re-projected
	std::map<PairIdx, Point3> truthPoints;
	for (const Track& track : scene.tracks)
		for (const Observation& obs : track.observations)
			truthPoints.emplace(PairIdx(obs.imageID, obs.featureID), track.position);

	// the two cameras kept are the ones the bridge leans on hardest, so the seam still has
	// correspondences after the rest of the sub-scene is dropped
	const std::unordered_map<IIndex, IIndex> localOf0 = LocalOf(localToGlobals[0]);
	const std::unordered_map<IIndex, IIndex> localOf1 = LocalOf(localToGlobals[1]);
	std::map<IIndex, unsigned> bridgeWeight;
	for (const ImagePair& pair : scene.pairs) {
		const IIndex globalIn1 = localOf1.count(pair.ID1) ? pair.ID1 : (localOf1.count(pair.ID2) ? pair.ID2 : NO_ID);
		const IIndex globalIn0 = localOf0.count(pair.ID1) ? pair.ID1 : (localOf0.count(pair.ID2) ? pair.ID2 : NO_ID);
		if (globalIn0 == NO_ID || globalIn1 == NO_ID)
			continue;
		bridgeWeight[localOf1.at(globalIn1)] += (unsigned)pair.matches.size();
	}
	if (bridgeWeight.size() < 2) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: the bridge runs through %u cameras of sub-scene 1",
			(unsigned)bridgeWeight.size());
		return false;
	}
	std::vector<std::pair<unsigned, IIndex>> ranked;
	for (const auto& [localID, weight] : bridgeWeight)
		ranked.emplace_back(weight, localID);
	std::sort(ranked.rbegin(), ranked.rend());
	const IIndex keptFirst = ranked[0].second, keptSecond = ranked[1].second;

	// the second camera a millimetre from the first, seeing the same points from its new place
	Scene& shallow = subScenes[1];
	shallow.images[keptSecond].C = shallow.images[keptFirst].C + Point3(0.001, 0, 0);
	Image& moved = shallow.images[keptSecond];
	const IIndex movedGlobal = localToGlobals[1][keptSecond];
	FOREACH(feature, moved.keypoints) {
		const auto it = truthPoints.find(PairIdx(movedGlobal, (uint32_t)feature));
		if (it == truthPoints.end())
			continue;
		const auto [proj, inFront] = moved.ProjectPoint(transforms[1] * it->second);
		if (inFront)
			moved.keypoints[feature].pt = cv::Point2f((float)proj.x, (float)proj.y);
	}
	// every other camera of the sub-scene is gone, and so are the observations it carried
	FOREACH(localID, shallow.images)
		if (localID != keptFirst && localID != keptSecond)
			shallow.images[localID].InvalidatePose();
	shallow.RecomputeCalibratedImages();
	for (Track& track : shallow.tracks) {
		ObservationArr kept;
		for (const Observation& obs : track.observations)
			if (obs.imageID == keptFirst || obs.imageID == keptSecond)
				kept.emplace_back(obs);
		track.observations.Swap(kept);
		track.numInliers = 0;
		// the millimetre of baseline is under the two degrees the default floor asks for
		if (track.observations.size() >= 2)
			TriangulateSkewLLS(track, shallow.images, 4.f, 0.f);
	}

	GlobalAlignmentConfig alignCfg;
	GlobalAlignment merger(scene, alignCfg);
	merger.PrepareSeamEvidence(subScenes, localToGlobals);
	std::vector<SeamCandidate> readings;
	merger.EstimateSeamPair(subScenes, 0, 1, readings);
	// the direction registering the two-camera sub-scene cannot observe a scale, so its reading does
	// not enter; the other one can, and measures it
	const SeamCandidate* shallowRig = ReadingOf(readings, SeamCandidate::RIG_B_ON_A);
	const SeamCandidate* deepRig = ReadingOf(readings, SeamCandidate::RIG_A_ON_B);
	if (shallowRig != NULL || deepRig == NULL) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: the shallow rig's reading %s, the other direction's %s",
			shallowRig == NULL ? "did not enter (expected)" : "entered", deepRig == NULL ? "is missing" : "entered (expected)");
		return false;
	}
	const REAL errScale = ABS(deepRig->T.scale / expected.scale - REAL(1));
	if (errScale > 0.02) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: the other direction's reading is %.2f%% off in scale", errScale * 100);
		return false;
	}
	VERBOSE("  shallow rig: its own reading does not enter, the other direction's is %.2f%% off", errScale * 100);
	return true;
}

// The merge of the two sub-scenes under a seam setup: sub-scene 1 either left out, or its cameras
// placed on the truth against sub-scene 0's, up to the one similarity the merged frame is free to
// choose -- whatever its own points say
static bool CheckDisagreeingMerge(const Transform transforms[2], const SeamCase& setup)
{
	Scene scene;
	std::vector<Scene> subScenes;
	std::vector<IIndexArr> localToGlobals;
	std::vector<Pose3D> gtPoses;
	if (!BuildTransformedSubScenes(scene, subScenes, localToGlobals, gtPoses, transforms, setup.hubBridge)) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: could not prepare the disagreeing sub-scenes for the merge");
		return false;
	}
	for (Image& img : subScenes[setup.rescaledPoses].images)
		if (img.IsValid())
			img.C *= setup.poseScale;
	const IIndexArr first(localToGlobals[0]), second(localToGlobals[1]);
	const GlobalAlignmentConfig alignCfg;
	GlobalAlignment merger(scene, alignCfg);
	MergeReport report;
	merger.MergeScenes(subScenes, localToGlobals, report);
	if (report.numPlaced < 2) {
		VERBOSE("  two disagreeing directions merged: sub-scene 1 left out");
		return true;
	}
	// the similarity sub-scene 0's cameras fix, then sub-scene 1's cameras under it
	Point3Arr merged0, truth0;
	for (const IIndex i : first)
		if (scene.images[i].IsValid()) {
			merged0.emplace_back(scene.images[i].C);
			truth0.emplace_back(gtPoses[i].C);
		}
	Transform T;
	if (EstimateSimilarityTransform(merged0, truth0, T) == 0) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: could not fit the disagreeing merge to the truth");
		return false;
	}
	AABB3 gtBbox(true);
	for (const Pose3D& pose : gtPoses)
		gtBbox.InsertFull(pose.C);
	const REAL extent = gtBbox.GetSize().norm();
	REAL maxPos = 0, maxRot = 0;
	for (const IIndex i : second) {
		if (!scene.images[i].IsValid())
			continue;
		maxPos = MAXF(maxPos, norm(T * scene.images[i].C - gtPoses[i].C) / extent);
		maxRot = MAXF(maxRot, (REAL)R2D(ACOS(ComputeAngle(Matrix3x3(scene.images[i].R * T.R.t()), Matrix3x3(gtPoses[i].R)))));
	}
	if (maxPos > 0.01 || maxRot > 0.5) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: the disagreeing merge placed sub-scene 1's cameras %.2f%% and %.3f deg off the truth",
			maxPos * 100, maxRot);
		return false;
	}
	VERBOSE("  two disagreeing directions merged: sub-scene 1's cameras %.3f%% and %.4f deg from the truth", maxPos * 100, maxRot);
	return true;
}

bool HierarchicalCameraAlignmentTest()
{
	TD_TIMER_START();

	Transform transforms[2];
	KnownSubSceneTransforms(transforms);
	// the sub-scenes sit at transforms[s] * truth, so the seam maps sub-scene 0 into sub-scene 1
	const Transform expected = transforms[1] * transforms[0].Invert();

	// the extent the translation error is measured against, in the frame the seam maps into
	Scene refScene;
	std::vector<Scene> refSubScenes;
	std::vector<IIndexArr> refLocalToGlobals;
	std::vector<Pose3D> refGTPoses;
	if (!BuildTransformedSubScenes(refScene, refSubScenes, refLocalToGlobals, refGTPoses, transforms)) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: the two-cluster scene did not split into two sub-scenes");
		return false;
	}
	const REAL extent = SubSceneExtent(refSubScenes[1]);
	if (extent <= 0) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: the second sub-scene has no camera extent");
		return false;
	}

	// each direction on its own, then both together: blinding one sub-scene's tracks leaves the
	// other sub-scene's cameras as the only rig with anything to register against
	const GlobalAlignmentConfig defaultCfg;
	std::vector<SeamCandidate> readings;
	SeamCase setup;
	setup.blindScene = 1;
	if (!MeasureSeam(transforms, GlobalAlignmentConfig::ALIGN_CAMERAS, setup, readings) ||
		!CheckOneDirectionReading("cameras of sub-scene 1 on points of sub-scene 0", readings,
			SeamCandidate::RIG_B_ON_A, defaultCfg.minCommonTracks) ||
		!CheckSeamTransform("cameras of sub-scene 1 on points of sub-scene 0", readings.front().T, expected, extent))
		return false;
	setup.blindScene = 0;
	if (!MeasureSeam(transforms, GlobalAlignmentConfig::ALIGN_CAMERAS, setup, readings) ||
		!CheckOneDirectionReading("cameras of sub-scene 0 on points of sub-scene 1", readings,
			SeamCandidate::RIG_A_ON_B, defaultCfg.minCommonTracks) ||
		!CheckSeamTransform("cameras of sub-scene 0 on points of sub-scene 1", readings.front().T, expected, extent))
		return false;
	// both directions: two readings, each on the truth, observing a scale, the cameras of both sides
	// behind each
	setup.blindScene = NO_ID;
	if (!MeasureSeam(transforms, GlobalAlignmentConfig::ALIGN_CAMERAS, setup, readings) || readings.size() != 2 ||
		ReadingOf(readings, SeamCandidate::RIG_B_ON_A) == NULL || ReadingOf(readings, SeamCandidate::RIG_A_ON_B) == NULL) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: both directions gave %u readings, expected one of each", (unsigned)readings.size());
		return false;
	}
	for (const SeamCandidate& c : readings) {
		if (!CheckSeamTransform(c.source == SeamCandidate::RIG_B_ON_A ? "both directions, rig of sub-scene 1" :
			"both directions, rig of sub-scene 0", c.T, expected, extent))
			return false;
		if (c.score.support[0] < 2 || c.score.support[1] < 2) {
			VERBOSE("HierarchicalCameraAlignmentTest FAILED: the reading of source %u came out with %u+%u cameras supporting",
				(unsigned)c.source, c.score.support[0], c.score.support[1]);
			return false;
		}
		if (c.weight != (float)c.score.Weight() * c.pairs.Share()) {
			VERBOSE("HierarchicalCameraAlignmentTest FAILED: the reading of source %u weighs %.3f, its weaker side's %u cameras times "
				"its pairs' share %.3f", (unsigned)c.source, c.weight, c.score.Weight(), c.pairs.Share());
			return false;
		}
	}

	// with noise on the observed bearings, every reading still on the truth within the noisy bars
	SeamCase noisy;
	noisy.keypointNoise = 1.0;
	for (const uint32_t blind : {(uint32_t)1, (uint32_t)0, (uint32_t)NO_ID}) {
		noisy.blindScene = blind;
		if (!MeasureSeam(transforms, GlobalAlignmentConfig::ALIGN_CAMERAS, noisy, readings)) {
			VERBOSE("HierarchicalCameraAlignmentTest FAILED: the noisy pair (blind sub-scene %d) yielded no reading", (int)blind);
			return false;
		}
		for (const SeamCandidate& c : readings)
			if (!CheckSeamTransform(c.source == SeamCandidate::RIG_B_ON_A ? "noisy, rig of sub-scene 1" : "noisy, rig of sub-scene 0",
				c.T, expected, extent, 1.0, 0.05, 0.05))
				return false;
	}

	// the whole bridge running through a single image of the first cluster: that side of the seam
	// is one camera, however many correspondences it holds, so no reading of it enters the block graph
	SeamCase starved;
	starved.hubBridge = true;
	MeasureSeam(transforms, GlobalAlignmentConfig::ALIGN_CAMERAS, starved, readings);
	if (!readings.empty()) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: %u readings of the seam one image carries on its side entered, the first of "
			"%u cameras", (unsigned)readings.size(), readings.front().score.Weight());
		return false;
	}
	VERBOSE("  a bridge through one image: no reading enters");

	// the two directions made to disagree: sub-scene 1's cameras are moved away from its own points,
	// so the direction that registers them measures a different similarity than the one that
	// registers its points; the verified pairs across, which saw the cameras where they truly are,
	// hold for one of the two and against the other, which does not enter
	SeamCase disagreeing;
	disagreeing.rescaledPoses = 1;
	if (!MeasureSeam(transforms, GlobalAlignmentConfig::ALIGN_CAMERAS, disagreeing, readings) || readings.size() != 1 ||
		!readings[0].pairs.Holds()) {
		VERBOSE("HierarchicalCameraAlignmentTest FAILED: the two disagreeing directions let %u readings in (the one the pairs hold "
			"for expected alone)", (unsigned)readings.size());
		return false;
	}
	VERBOSE("  two disagreeing directions: the one the pairs across hold against does not enter (%.0f for / %.0f against for the other)",
		readings[0].pairs.agree, readings[0].pairs.disagree);
	// and the merge of that pair puts sub-scene 1's cameras where they truly are, or leaves it out
	if (!CheckDisagreeingMerge(transforms, disagreeing))
		return false;

	// a sub-scene of two cameras a millimetre apart
	if (!CheckUnobservableScale(transforms, expected))
		return false;

	// the whole merge, in both modes, so the flag is exercised end to end
	if (!CheckMerge(transforms, GlobalAlignmentConfig::ALIGN_CAMERAS, "camera"))
		return false;
	if (!CheckMerge(transforms, GlobalAlignmentConfig::ALIGN_POINTS, "point"))
		return false;

	// three sub-scenes, two right seams and a wrong one closing the triangle: all three measured, the
	// wrong one off in scale
	if (!CheckTriangleSeams())
		return false;

	VERBOSE("HierarchicalCameraAlignmentTest PASSED (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/

} // namespace SFM
