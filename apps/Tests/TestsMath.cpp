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
#include "Tests.h"
#include "TestsMath.h"

#include <array>
#include <cmath>
#include <random>
#include <stdexcept>


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
	{
		bool overflowRejected = false;
		try {
			TetraFlow solver(1);
			const float maxCapacity = std::numeric_limits<float>::max();
			solver.AddNode(0, maxCapacity, maxCapacity);
			solver.AddNode(0, maxCapacity, maxCapacity);
			(void)solver.ComputeMaxFlow();
		} catch (const std::overflow_error&) {
			overflowRejected = true;
		}
		if (!overflowRejected) {
			VERBOSE("error: TetraFlow accepted non-finite aggregate terminal capacities");
			return false;
		}
	}
	{
		bool overflowRejected = false;
		try {
			TetraFlow solver(2);
			const float maxCapacity = std::numeric_limits<float>::max();
			solver.EdgeCapacity(0, 0) = maxCapacity;
			solver.EdgeCapacity(1, 0) = maxCapacity;
			solver.LinkEdge(0, 0, 1, 0);
			(void)solver.ComputeMaxFlow();
		} catch (const std::overflow_error&) {
			overflowRejected = true;
		}
		if (!overflowRejected) {
			VERBOSE("error: TetraFlow accepted residual capacities whose sum overflows");
			return false;
		}
	}
	{
		bool overflowRejected = false;
		try {
			TetraFlow solver(3);
			const float maxCapacity = std::numeric_limits<float>::max();
			solver.AddNode(0, maxCapacity, 0);
			solver.AddNode(1, 0, maxCapacity);
			solver.AddNode(2, 0, maxCapacity);
			solver.AddEdge(0, 1, maxCapacity, 0);
			solver.AddEdge(0, 2, maxCapacity, 0);
			(void)solver.ComputeMaxFlow();
		} catch (const std::overflow_error&) {
			overflowRejected = true;
		}
		if (!overflowRejected) {
			VERBOSE("error: TetraFlow accepted a node whose batched deficit can overflow");
			return false;
		}
	}
	{
		bool overflowRejected = false;
		try {
			TetraFlow solver(3);
			const float maxCapacity = std::numeric_limits<float>::max();
			solver.AddEdge(0, 1, 0, maxCapacity);
			solver.AddEdge(0, 2, 0, maxCapacity);
			(void)solver.ComputeMaxFlow();
		} catch (const std::overflow_error&) {
			overflowRejected = true;
		}
		if (!overflowRejected) {
			VERBOSE("error: TetraFlow node capacity check depends on arc orientation");
			return false;
		}
	}
	for (int order = 0; order < 2; ++order) {
		bool overflowRejected = false;
		try {
			TetraFlow solver(2);
			const float maxCapacity = std::numeric_limits<float>::max();
			solver.AddEdge(0, 1, order ? 1.f : maxCapacity, order ? maxCapacity : 1.f);
		} catch (const std::overflow_error&) {
			overflowRejected = true;
		}
		if (!overflowRejected) {
			VERBOSE("error: TetraFlow capacity overflow check depends on operand order");
			return false;
		}
	}
	{
		const std::array<float, 4> capacities = {
			0x1.fffffep+125f,
			0x1.fffffep+126f,
			0x1.fffffep+122f,
			0x1.cp+125f
		};
		std::array<unsigned, 4> order = {0, 1, 2, 3};
		do {
			bool overflowRejected = false;
			try {
				TetraFlow solver(5);
				for (unsigned i = 0; i < order.size(); ++i)
					solver.AddEdge(0, i + 1, capacities[order[i]], 0);
				(void)solver.ComputeMaxFlow();
			} catch (const std::overflow_error&) {
				overflowRejected = true;
			}
			if (!overflowRejected) {
				VERBOSE("error: TetraFlow accepted a rounded multi-edge capacity overflow");
				return false;
			}
		} while (std::next_permutation(order.begin(), order.end()));
	}
	{
		const float maxCapacity = std::numeric_limits<float>::max();
		const std::array<float, 3> capacities = {
			std::nextafter(maxCapacity, 0.f),
			0x1.8p+103f,
			0x1p+102f
		};
		std::array<unsigned, 3> order = {0, 1, 2};
		do {
			TetraFlow solver(4);
			for (unsigned i = 0; i < order.size(); ++i)
				solver.AddEdge(0, i + 1, capacities[order[i]], 0);
			try {
				if (solver.ComputeMaxFlow() != 0) {
					VERBOSE("error: TetraFlow reported flow for a terminal-free graph");
					return false;
				}
			} catch (const std::overflow_error&) {
				VERBOSE("error: TetraFlow rejected an exact boundary capacity sum");
				return false;
			}
		} while (std::next_permutation(order.begin(), order.end()));
	}
	const float invalidCapacities[] = {
		-1.f,
		-std::numeric_limits<float>::infinity(),
		std::numeric_limits<float>::infinity(),
		std::numeric_limits<float>::quiet_NaN()
	};
	for (float invalidCapacity : invalidCapacities) {
		bool invalidRejected = false;
		try {
			TetraFlow solver(1);
			solver.AddNode(0, invalidCapacity, 0);
		} catch (const std::exception&) {
			invalidRejected = true;
		}
		if (!invalidRejected) {
			VERBOSE("error: TetraFlow accepted an invalid AddNode capacity");
			return false;
		}
	}
	{
		TetraFlow solver(2);
		solver.AddNode(0, 1, 0);
		solver.AddNode(1, 0, 1);
		solver.AddEdge(0, 1, 1, 0);
		for (float invalidCapacity : invalidCapacities) {
			solver.SourceCapacity(1) = invalidCapacity;
			try {
				(void)solver.ComputeMaxFlow();
				VERBOSE("error: TetraFlow accepted an invalid direct capacity");
				return false;
			} catch (const std::exception&) {
			}
		}
		solver.SourceCapacity(1) = 0;
		if (solver.ComputeMaxFlow() != 1 || !solver.CheckMaxFlow()) {
			VERBOSE("error: TetraFlow validation failure modified the construction graph");
			return false;
		}
	}
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

} // namespace SEACAVE
