/*
 * TestsMath.cpp
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
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "../../libs/Math/Common.h"
#include "../../libs/Math/TetraFlow.h"
#include "../../libs/Math/LBP.h"
#include "Tests.h"
#include "TestsMath.h"

#include <random>
#pragma push_macro("VERBOSE")
#undef VERBOSE
#pragma push_macro("LOG")
#undef LOG
#include <ceres/jet.h>
#pragma pop_macro("VERBOSE")
#pragma pop_macro("LOG")


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SEACAVE {

namespace {

struct FlowTestEdge {
	uint32_t u, v;
	float capUV, capVU;
};
struct FlowTestGraph {
	uint32_t numNodes = 0;
	std::vector<float> capSource, capSink;
	std::vector<FlowTestEdge> edges;
};

// random graph with at most four arcs per node (multi-edges allowed); integer capacities
// keep every intermediate sum exact in single precision, so the results must match exactly
FlowTestGraph RandomFlowTestGraph(std::mt19937& rng, uint32_t numNodes, bool integerCaps)
{
	FlowTestGraph g;
	g.numNodes = numNodes;
	g.capSource.assign(numNodes, 0.f);
	g.capSink.assign(numNodes, 0.f);
	auto randCap = [&]() -> float {
		return integerCaps ? float(rng() % 16) : float(rng() % 100000) / 4096.f;
	};
	for (uint32_t n = 0; n < numNodes; ++n) {
		switch (rng() % 4) {
		case 0: g.capSource[n] = randCap(); break;
		case 1: g.capSink[n] = randCap(); break;
		case 2: g.capSource[n] = randCap(); g.capSink[n] = randCap(); break;
		default: break;
		}
	}
	if (numNodes > 1) {
		std::vector<unsigned> degree(numNodes, 0);
		const uint32_t targetEdges = numNodes * 2 - 1 - rng() % numNodes;
		for (uint32_t attempts = 0, e = 0; e < targetEdges && attempts < targetEdges * 8; ++attempts) {
			const uint32_t u = rng() % numNodes;
			const uint32_t v = rng() % 2 ? rng() % numNodes : (u + 1 + rng() % 3) % numNodes; // mostly local
			if (u == v || degree[u] == 4 || degree[v] == 4)
				continue;
			++degree[u]; ++degree[v];
			g.edges.push_back({u, v, randCap(), randCap()});
			++e;
		}
	}
	return g;
}

// exact reference: Edmonds-Karp in double precision, with explicit source and sink nodes
double ReferenceMaxFlow(const FlowTestGraph& g)
{
	const uint32_t S = g.numNodes, T = g.numNodes + 1, N = g.numNodes + 2;
	struct Arc { uint32_t to; double cap; };
	std::vector<Arc> arcs;
	std::vector<std::vector<uint32_t>> adj(N);
	auto addArc = [&](uint32_t u, uint32_t v, double capUV, double capVU) {
		adj[u].push_back((uint32_t)arcs.size()); arcs.push_back({v, capUV});
		adj[v].push_back((uint32_t)arcs.size()); arcs.push_back({u, capVU});
	};
	for (uint32_t n = 0; n < g.numNodes; ++n) {
		if (g.capSource[n] > 0) addArc(S, n, g.capSource[n], 0);
		if (g.capSink[n] > 0) addArc(n, T, g.capSink[n], 0);
	}
	for (const FlowTestEdge& e : g.edges)
		addArc(e.u, e.v, e.capUV, e.capVU);
	double flow = 0;
	std::vector<uint32_t> parentArc(N), queue;
	for (;;) {
		std::fill(parentArc.begin(), parentArc.end(), UINT32_MAX);
		queue.assign(1, S);
		parentArc[S] = UINT32_MAX - 1;
		for (size_t k = 0; k < queue.size() && parentArc[T] == UINT32_MAX; ++k) {
			const uint32_t x = queue[k];
			for (uint32_t a : adj[x]) {
				if (arcs[a].cap <= 0 || parentArc[arcs[a].to] != UINT32_MAX)
					continue;
				parentArc[arcs[a].to] = a;
				queue.push_back(arcs[a].to);
			}
		}
		if (parentArc[T] == UINT32_MAX)
			return flow;
		double d = std::numeric_limits<double>::infinity();
		for (uint32_t x = T; x != S; x = arcs[parentArc[x] ^ 1].to)
			d = std::min(d, arcs[parentArc[x]].cap);
		for (uint32_t x = T; x != S; x = arcs[parentArc[x] ^ 1].to) {
			arcs[parentArc[x]].cap -= d;
			arcs[parentArc[x] ^ 1].cap += d;
		}
		flow += d;
	}
}

// capacity of the cut given by the source-side flags: equals the max-flow at optimum
double CutCapacity(const FlowTestGraph& g, const std::vector<bool>& srcSide)
{
	double cut = 0;
	for (uint32_t n = 0; n < g.numNodes; ++n)
		cut += srcSide[n] ? (double)g.capSink[n] : (double)g.capSource[n];
	for (const FlowTestEdge& e : g.edges) {
		if (srcSide[e.u] && !srcSide[e.v]) cut += e.capUV;
		else if (srcSide[e.v] && !srcSide[e.u]) cut += e.capVU;
	}
	return cut;
}

} // namespace

bool TestTetraFlow()
{
	std::mt19937 rng(20260826u);
	TetraFlow solver; // reused across iterations: exercises Reset()
	std::vector<bool> srcSide;
	for (int iter = 0; iter < 400; ++iter) {
		const uint32_t numNodes = iter < 2 ? (uint32_t)iter : 2 + rng() % 300; // includes the empty and the single-node graph
		const bool integerCaps = (iter % 3) != 0;
		const FlowTestGraph g = RandomFlowTestGraph(rng, numNodes, integerCaps);
		solver.Reset(numNodes);
		if (iter % 4 == 2) {
			// the slot-addressed construction: the capacities are accumulated in place, before or after the edge is linked
			for (uint32_t n = 0; n < numNodes; ++n) {
				const float s = g.capSource[n] * 0.5f, t = g.capSink[n] * 0.5f;
				solver.SourceCapacity(n) += s; solver.SourceCapacity(n) += g.capSource[n] - s;
				solver.SinkCapacity(n) = t; solver.SinkCapacity(n) += g.capSink[n] - t;
			}
			std::vector<unsigned> nextSlot(numNodes, 0);
			for (size_t k = 0; k < g.edges.size(); ++k) {
				const FlowTestEdge& e = g.edges[k];
				const unsigned iu = nextSlot[e.u]++, iv = nextSlot[e.v]++;
				if (k % 2)
					solver.LinkEdge(e.u, iu, e.v, iv);
				solver.EdgeCapacity(e.u, iu) += e.capUV;
				solver.EdgeCapacity(e.v, iv) += e.capVU;
				if (!(k % 2))
					solver.LinkEdge(e.u, iu, e.v, iv);
			}
		} else {
			// the classic construction: the capacities are given when the nodes and edges are added
			for (uint32_t n = 0; n < numNodes; ++n) {
				if (iter % 2) {
					// the terminal capacities may be accumulated over several calls
					const float s = g.capSource[n] * 0.5f, t = g.capSink[n] * 0.5f;
					solver.AddNode(n, s, t);
					solver.AddNode(n, g.capSource[n] - s, g.capSink[n] - t);
				} else
					solver.AddNode(n, g.capSource[n], g.capSink[n]);
			}
			for (const FlowTestEdge& e : g.edges)
				solver.AddEdge(e.u, e.v, e.capUV, e.capVU);
		}
		const double flow = solver.ComputeMaxFlow();
		if (!solver.CheckMaxFlow()) {
			VERBOSE("error: TetraFlow iteration %d: an augmenting path remains", iter);
			return false;
		}
		srcSide.resize(numNodes);
		for (uint32_t n = 0; n < numNodes; ++n)
			srcSide[n] = solver.IsNodeOnSrcSide(n);
		const double reference = ReferenceMaxFlow(g);
		const double cut = CutCapacity(g, srcSide);
		const double tolerance = integerCaps ? 0 : 1e-5 * std::max(1.0, std::abs(reference));
		if (std::abs(flow - reference) > tolerance) {
			VERBOSE("error: TetraFlow iteration %d: flow %.9g differs from the reference %.9g", iter, flow, reference);
			return false;
		}
		if (std::abs(cut - reference) > tolerance) {
			VERBOSE("error: TetraFlow iteration %d: cut capacity %.9g differs from the max-flow %.9g", iter, cut, reference);
			return false;
		}
		if (iter % 50 == 49)
			solver.Release(); // exercises the reuse after a full release
	}
	return true;
}
/*----------------------------------------------------------------*/

namespace {
LBPInference::EnergyType STCALL TestSmoothnessPotts(LBPInference::NodeID, LBPInference::NodeID, LBPInference::LabelID l1, LBPInference::LabelID l2) {
	return l1 == l2 && l1 != 0 ? LBPInference::EnergyType(0) : LBPInference::EnergyType(1);
}
} // namespace

bool TestLBPPotts()
{
	std::mt19937 rng(20261012u);
	std::uniform_real_distribution<float> unit(0.f, 1.f);
	for (int iter = 0; iter < 50; ++iter) {
		// a ring of nodes plus random chords; each node a random label subset, in random order, label 0 included at times
		const unsigned numNodes(2 + rng() % 200), numLabels(1 + rng() % 40);
		LBPInference generic(numNodes), potts(numNodes);
		generic.SetSmoothCost(TestSmoothnessPotts);
		potts.SetSmoothCostPotts(TestSmoothnessPotts, 1);
		for (unsigned n = 0; n < numNodes; ++n) {
			const unsigned m((n + 1) % numNodes), c(rng() % numNodes);
			const float weight(0.1f + unit(rng));
			generic.SetNeighbors(n, m, weight);
			potts.SetNeighbors(n, m, weight);
			if (c != n && c != m && rng() % 3 == 0) {
				generic.SetNeighbors(n, c, weight);
				potts.SetNeighbors(n, c, weight);
			}
		}
		std::vector<LBPInference::LabelID> labels(numLabels+1);
		std::iota(labels.begin(), labels.end(), LBPInference::LabelID(0));
		for (unsigned n = 0; n < numNodes; ++n) {
			std::shuffle(labels.begin(), labels.end(), rng);
			const unsigned count(1 + rng() % (numLabels+1));
			for (unsigned k = 0; k < count; ++k) {
				const float cost(2.f * unit(rng));
				generic.SetDataCost(labels[k], n, cost);
				potts.SetDataCost(labels[k], n, cost);
			}
		}
		// the labels, not the energies: the energy follows from the labels, and ComputeEnergy's
		// OpenMP reduction sums in no fixed order
		for (unsigned i = 0; i < 6; ++i) {
			generic.Optimize(1);
			potts.Optimize(1);
			for (unsigned n = 0; n < numNodes; ++n) {
				if (generic.GetLabel(n) != potts.GetLabel(n)) {
					VERBOSE("error: LBP iteration %d/%u: node %u has label %u by the Potts path, %u by the generic one", iter, i, n, potts.GetLabel(n), generic.GetLabel(n));
					return false;
				}
			}
		}
	}
	return true;
}
/*----------------------------------------------------------------*/


// camera models: round trips, the pixel-center and axis conventions, Brown inversion over
// realistic lenses, and evaluation on Ceres Jets (values matching the double evaluation, derivatives
// matching central differences)
bool TestCameraModels()
{
	typedef Eigen::Vector2d Vec2;
	typedef Eigen::Vector3d Vec3;
	std::mt19937 rng(7);
	std::uniform_real_distribution<double> U(0, 1);
	const auto R = [&](double a, double b) { return a + (b - a) * U(rng); };

	// pinhole: Project(Ray(x)) == x, Depth(lambda*Ray(x)) == lambda
	{
		const Pinhole<double> cam{2850.5, 2851.25, 1999.75, 1501.25};
		double maxErr(0), maxDepthErr(0);
		for (int i = 0; i < 10000; ++i) {
			const Vec2 x(R(-0.5, 3999.5), R(-0.5, 2999.5));
			const double lambda(R(0.1, 100));
			Vec2 y;
			if (!cam.Project(cam.Ray(x) * lambda, y)) {
				VERBOSE("error: camera models: pinhole point in front of the camera reported invisible");
				return false;
			}
			maxErr = MAXF(maxErr, (y - x).cwiseAbs().maxCoeff());
			maxDepthErr = MAXF(maxDepthErr, ABS(Pinhole<double>::Depth(cam.Ray(x) * lambda) - lambda) / lambda);
		}
		Vec2 y;
		if (maxErr > 1e-9 || maxDepthErr > 1e-12 || cam.Project(Vec3(0.1, 0.2, -1.0), y)) {
			VERBOSE("error: camera models: pinhole round trip %g px, depth %g, or a point behind reported visible", maxErr, maxDepthErr);
			return false;
		}
	}

	// Brown: Distort(Undistort(p)) == p over the whole image of realistic lenses
	{
		struct Lens { const char* name; int width, height; double f; BrownDistortion<double> d; };
		const Lens lenses[] = {
			{"mild",       6048, 4032, 3410, {-0.05,  0.02,   1e-4,  1e-4,  0,     0,    0,    0}},
			{"wide",       4000, 3000, 2600, {-0.25,  0.07,   5e-4,  3e-4, -0.008, 0,    0,    0}},
			{"pincushion", 1920, 1080, 1700, { 0.12, -0.03,  -3e-4,  2e-4,  0.004, 0,    0,    0}},
			{"rational",   2048, 1536, 1800, {-0.05,  0.01,   1e-4, -2e-4,  0,     0.25, 0.02, 0.001}},
		};
		for (const Lens& lens : lenses) {
			double maxResidual(0), maxInverse(0);
			for (int y = 0; y <= lens.height; y += 7) {
				for (int x = 0; x <= lens.width; x += 7) {
					const Vec2 p((x - lens.width*0.5) / lens.f, (y - lens.height*0.5) / lens.f);
					maxResidual = MAXF(maxResidual, (lens.d.Distort(lens.d.Undistort(p)) - p).cwiseAbs().maxCoeff());
					maxInverse = MAXF(maxInverse, (lens.d.Undistort(lens.d.Distort(p)) - p).cwiseAbs().maxCoeff());
				}
			}
			if (maxResidual > 1e-9 || maxInverse > 1e-9) {
				VERBOSE("error: camera models: Brown %s lens inversion residual %g / %g (normalized)", lens.name, maxResidual, maxInverse);
				return false;
			}
		}
	}

	// spherical: conventions
	{
		const int width(4096), height(2048);
		const Spherical<double> sph(Spherical<double>::FromSize(width, height));
		const double cx((width-1)*0.5), cy((height-1)*0.5);
		struct Case { Vec3 X; Vec2 x; const char* name; };
		const Case cases[] = {
			{Vec3( 0, 0, 1), Vec2(cx, cy), "forward +Z at the image center"},
			{Vec3( 1, 0, 0), Vec2(cx + width*0.25, cy), "right +X at 3/4 width"},
			{Vec3(-1, 0, 0), Vec2(cx - width*0.25, cy), "left -X at 1/4 width"},
			{Vec3( 0, 0,-1), Vec2(cx + width*0.5, cy), "back -Z on the right seam"},
			{Vec3( 0,-1, 0), Vec2(cx, -0.5), "up -Y on the top edge"},
			{Vec3( 0, 1, 0), Vec2(cx, height-0.5), "down +Y on the bottom edge"},
		};
		for (const Case& c : cases) {
			Vec2 x;
			if (!sph.Project(c.X * 3.0, x) || (x - c.x).cwiseAbs().maxCoeff() > 1e-9 || ABS(Spherical<double>::Depth(c.X * 3.0) - 3.0) > 1e-12) {
				VERBOSE("error: camera models: spherical %s: projected to (%g, %g), expected (%g, %g)", c.name, x.x(), x.y(), c.x.x(), c.x.y());
				return false;
			}
		}
		// pixel centers at integer coordinates: column 0 is centered half a pixel right of longitude -pi
		const Vec2 a0(sph.Angles(Vec2(0, cy)));
		const Vec2 aTop(sph.Angles(Vec2(cx, 0)));
		if (ABS(a0.x() - (-M_PI + M_PI/width)) > 1e-12 || ABS(a0.y()) > 1e-12 || ABS(aTop.y() - (M_PI_2 - M_PI_2/height)) > 1e-12) {
			VERBOSE("error: camera models: spherical pixel centers: column 0 longitude %.15g, row 0 elevation %.15g", a0.x(), aTop.y());
			return false;
		}
		Vec2 x;
		if (sph.Project(Vec3::Zero(), x)) {
			VERBOSE("error: camera models: spherical zero vector reported visible");
			return false;
		}
		// round trips over the pixel-center rows: random pixels, the seam columns, both hemispheres and
		// the rows next to the poles
		const auto RoundTrip = [&](const Vec2& x) {
			const Vec3 ray(sph.Ray(x));
			Vec2 y;
			if (ABS(ray.norm() - 1) > 1e-12 || !sph.Project(ray * 7.5, y))
				return 1e9;
			// longitude wraps: the two seam edges -0.5 and W-0.5 are the same meridian
			const double du(ABS(y.x() - x.x()));
			return MAXF(MINF(du, ABS(du - width)), ABS(y.y() - x.y()));
		};
		double maxErr(0);
		for (int i = 0; i < 20000; ++i)
			maxErr = MAXF(maxErr, RoundTrip(Vec2(R(-0.5, width-0.5), R(0, height-1))));
		const double us[] = {-0.5, -0.49, 0, 0.25, 1, width*0.25-0.5, width*0.5-0.5, width*0.75-0.5, width-2.0, width-1.0, width-0.75, width-0.5};
		const double vs[] = {0, 0.25, 1, 2, cy, height-3.0, height-2.0, height-1.0, height-1.25};
		for (double u : us)
			for (double v : vs)
				maxErr = MAXF(maxErr, RoundTrip(Vec2(u, v)));
		if (maxErr > 1e-8) {
			VERBOSE("error: camera models: spherical round trip error %g px", maxErr);
			return false;
		}
	}

	// Ceres Jets: values matching the double evaluation (Jet division multiplies by the reciprocal), derivatives matching central differences
	{
		typedef ceres::Jet<double,3> Jet;
		typedef Eigen::Matrix<Jet,2,1> JVec2;
		typedef Eigen::Matrix<Jet,3,1> JVec3;
		const Spherical<double> sph(Spherical<double>::FromSize(2048, 1024));
		const Spherical<Jet> sphJ{Jet(sph.fx), Jet(sph.fy), Jet(sph.cx), Jet(sph.cy)};
		const Pinhole<double> cam{1500, 1510, 960, 540};
		const Pinhole<Jet> camJ{Jet(cam.fx), Jet(cam.fy), Jet(cam.cx), Jet(cam.cy)};
		const BrownDistortion<double> dist{-0.12, 0.05, 2e-4, -3e-4, -0.01, 0.1, 0.01, 0.001};
		const BrownDistortion<Jet> distJ{Jet(dist.k1), Jet(dist.k2), Jet(dist.p1), Jet(dist.p2), Jet(dist.k3), Jet(dist.k4), Jet(dist.k5), Jet(dist.k6)};
		const auto ProjectBrown = [&](const Vec3& X) {
			return Vec2(cam.NormalizedToPixel(dist.Distort(Pinhole<double>::Perspective(X))));
		};
		const auto ProjectSpherical = [&](const Vec3& X) {
			Vec2 x;
			sph.Project(X, x);
			return x;
		};
		double maxValue(0), maxDerivative(0);
		for (int i = 0; i < 1000; ++i) {
			const Vec3 X(R(-1, 1), R(-1, 1), R(0.5, 2) * (i%2 ? 1 : -1));
			const JVec3 XJ(Jet(X.x(), 0), Jet(X.y(), 1), Jet(X.z(), 2));
			JVec2 xs;
			sphJ.Project(XJ, xs);
			const Vec2 ys(ProjectSpherical(X));
			maxValue = MAXF(maxValue, ABS(xs.x().a - ys.x()) + ABS(xs.y().a - ys.y()));
			const bool front(X.z() > 0);
			JVec2 xb(Jet(0), Jet(0));
			if (front) {
				xb = camJ.NormalizedToPixel(distJ.Distort(Pinhole<Jet>::Perspective(XJ)));
				const Vec2 yb(ProjectBrown(X));
				maxValue = MAXF(maxValue, ABS(xb.x().a - yb.x()) + ABS(xb.y().a - yb.y()));
			}
			for (int k = 0; k < 3; ++k) {
				const double h(1e-6);
				Vec3 Xp(X), Xm(X);
				Xp[k] += h; Xm[k] -= h;
				const Vec2 ds((ProjectSpherical(Xp) - ProjectSpherical(Xm)) / (2*h));
				maxDerivative = MAXF(maxDerivative, MAXF(ABS(xs.x().v[k] - ds.x()), ABS(xs.y().v[k] - ds.y())) / MAXF(1.0, ds.cwiseAbs().maxCoeff()));
				if (front) {
					const Vec2 db((ProjectBrown(Xp) - ProjectBrown(Xm)) / (2*h));
					maxDerivative = MAXF(maxDerivative, MAXF(ABS(xb.x().v[k] - db.x()), ABS(xb.y().v[k] - db.y())) / MAXF(1.0, db.cwiseAbs().maxCoeff()));
				}
			}
		}
		if (maxValue > 1e-9 || maxDerivative > 1e-5) {
			VERBOSE("error: camera models: Jet evaluation differs by %g in value, %g (relative) in derivative", maxValue, maxDerivative);
			return false;
		}
	}
	return true;
}
/*----------------------------------------------------------------*/

} // namespace SEACAVE
