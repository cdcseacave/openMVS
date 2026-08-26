/*
 * GraphCutBench.cpp
 *
 * Benchmark harness for the max-flow solvers used by the mesh-reconstruction
 * graph cut. Loads a graph dumped by ReconstructMesh (env MVS_EXPORT_GRAPH,
 * format "MVSGCUT1") and measures per solver: construction time, solve time,
 * flow value, cut size and peak process memory. Also hosts a fuzz self-test
 * that cross-checks the solvers on random degree<=4 graphs.
 *
 * Usage:
 *   GraphCutBench <graph.gcut> [--solver tetra|ibfs] [--subgraph N] [--reorder] [--save out.gcut] [--repeat N] [--verify]
 *     --subgraph N  solve only the first N nodes (a spatially coherent prefix of the scene)
 *     --reorder     renumber the nodes in BFS order before solving (locality experiment)
 *     --save FILE   write the (subgraphed/reordered) graph in the same format
 *     --verify      check the cut-value certificate (cut capacity == max flow) and the
 *                   residual-graph optimality check of the solver
 *     --sides FILE  write the source-side flag of every node (one byte each; implies --verify),
 *                   to compare the cuts of two solvers with cmp
 *   GraphCutBench --selftest [--skip-ibfs]      fuzz the solvers on random degree<=4 graphs
 *   GraphCutBench --selftest-iter K             print and re-run a single fuzz iteration
 *   GraphCutBench --stress [--iters N] [--seed S] [--max-nodes N] [--skip-ibfs] [--no-dinic] [--trace]
 *                 [--ibfs-min-nodes N] [--dinic-max-nodes N]
 *     extended fuzz: several graph families (random degree<=4, 4-regular pairing model, torus
 *     grids, long chains), capacity regimes (small integers, mixed, huge terminal weights like the
 *     pipeline's maxCap, wide dynamic range), terminal densities (sparse, 1/3, all), solver object
 *     reuse through Reset(), split AddNode() calls, 0- and 1-node graphs; every instance is checked
 *     with the cut certificate, the residual optimality check, the IBFS oracle and (small graphs)
 *     an exact double-precision Dinic; prints the aggregated TetraFlow operation counters when
 *     compiled with TETRAFLOW_STATS=1 (shows which code paths the run exercised)
 *   GraphCutBench --stress-iter K [--seed S]    print and re-run a single stress iteration
 *
 * Solver memory is roughly 200 B/node (IBFS) and 70 B/node (TetraFlow); the loaded graph is
 * mmap-ed and does not count towards the reported peak footprint.
 */

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>
#ifdef __APPLE__
#include <libproc.h>
#include <sys/resource.h>
#else
#include <sys/resource.h>
#endif

#include "../../libs/Math/IBFS/IBFS.h"
#include "../../libs/Math/TetraFlow.h"

namespace {

double Now() {
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// peak physical memory of this process, in bytes
uint64_t PeakMemory() {
	#ifdef __APPLE__
	rusage_info_current info;
	if (proc_pid_rusage(getpid(), RUSAGE_INFO_CURRENT, (rusage_info_t*)&info) == 0)
		return info.ri_lifetime_max_phys_footprint;
	return 0;
	#else
	struct rusage ru;
	getrusage(RUSAGE_SELF, &ru);
	return (uint64_t)ru.ru_maxrss*1024;
	#endif
}

#pragma pack(push, 1)
struct EdgeRec { uint32_t u, v; float capUV, capVU; };
struct NodeRec { float s, t; };
#pragma pack(pop)

struct Dump {
	uint64_t numNodes = 0, numEdges = 0;
	const EdgeRec* edges = nullptr;
	const NodeRec* nodes = nullptr;
	void* map = nullptr;
	size_t mapSize = 0;
	// non-mmap storage for synthetic graphs
	std::vector<EdgeRec> ownEdges;
	std::vector<NodeRec> ownNodes;

	bool Load(const char* path) {
		const int fd = open(path, O_RDONLY);
		if (fd < 0) { fprintf(stderr, "error: can not open '%s'\n", path); return false; }
		struct stat st;
		fstat(fd, &st);
		mapSize = (size_t)st.st_size;
		map = mmap(nullptr, mapSize, PROT_READ, MAP_PRIVATE, fd, 0);
		close(fd);
		if (map == MAP_FAILED) { fprintf(stderr, "error: mmap failed\n"); return false; }
		madvise(map, mapSize, MADV_SEQUENTIAL);
		const char* p = (const char*)map;
		if (mapSize < 24 || memcmp(p, "MVSGCUT1", 8) != 0) { fprintf(stderr, "error: bad magic\n"); return false; }
		memcpy(&numNodes, p+8, 8);
		memcpy(&numEdges, p+16, 8);
		const size_t expected = 24 + sizeof(EdgeRec)*numEdges + sizeof(NodeRec)*numNodes;
		if (mapSize != expected) {
			fprintf(stderr, "error: size mismatch (%zu != %zu)\n", mapSize, expected);
			return false;
		}
		edges = (const EdgeRec*)(p + 24);
		nodes = (const NodeRec*)(p + 24 + sizeof(EdgeRec)*numEdges);
		return true;
	}
	// keep only the nodes [0, n) and the edges among them: node ids follow the Delaunay
	// cell iteration order, so a prefix is a spatially coherent chunk of the scene
	void Subgraph(uint64_t n) {
		if (n >= numNodes)
			return;
		ownNodes.assign(nodes, nodes + n);
		ownEdges.clear();
		for (uint64_t e=0; e<numEdges; ++e)
			if (edges[e].u < n && edges[e].v < n)
				ownEdges.push_back(edges[e]);
		numNodes = n;
		numEdges = ownEdges.size();
		edges = ownEdges.data();
		nodes = ownNodes.data();
		if (map && map != MAP_FAILED) { munmap(map, mapSize); map = nullptr; }
	}
	// renumber the nodes in BFS order over the adjacency so that neighbors tend to be close
	// in memory (experiment: locality of the pipeline's Delaunay cell order vs BFS order);
	// edges are re-sorted by (u,v) to mimic the pipeline's per-cell emission order
	void Reorder() {
		std::vector<uint32_t> off(numNodes+1, 0);
		for (uint64_t e=0; e<numEdges; ++e) { ++off[edges[e].u+1]; ++off[edges[e].v+1]; }
		for (uint64_t i=1; i<=numNodes; ++i) off[i] += off[i-1];
		std::vector<uint32_t> adj(2*numEdges), fill(off.begin(), off.end()-1);
		for (uint64_t e=0; e<numEdges; ++e) { adj[fill[edges[e].u]++] = edges[e].v; adj[fill[edges[e].v]++] = edges[e].u; }
		std::vector<uint32_t> newId(numNodes, UINT32_MAX), order; order.reserve(numNodes);
		for (uint32_t seed=0; seed<numNodes; ++seed) {
			if (newId[seed] != UINT32_MAX) continue;
			newId[seed] = (uint32_t)order.size(); order.push_back(seed);
			for (size_t head=order.size()-1; head<order.size(); ++head) {
				const uint32_t x = order[head];
				for (uint32_t k=off[x]; k<off[x+1]; ++k) {
					const uint32_t y = adj[k];
					if (newId[y] == UINT32_MAX) { newId[y] = (uint32_t)order.size(); order.push_back(y); }
				}
			}
		}
		std::vector<NodeRec> nn(numNodes);
		for (uint64_t i=0; i<numNodes; ++i) nn[newId[i]] = nodes[i];
		std::vector<EdgeRec> ne(edges, edges+numEdges);
		for (auto& e : ne) { e.u = newId[e.u]; e.v = newId[e.v]; if (e.u > e.v) { std::swap(e.u, e.v); std::swap(e.capUV, e.capVU); } }
		std::sort(ne.begin(), ne.end(), [](const EdgeRec& a, const EdgeRec& b) { return a.u < b.u || (a.u == b.u && a.v < b.v); });
		ownNodes.swap(nn); ownEdges.swap(ne);
		nodes = ownNodes.data(); edges = ownEdges.data();
		if (map && map != MAP_FAILED) { munmap(map, mapSize); map = nullptr; }
	}
	// write the (possibly reordered/subgraphed) graph in the same MVSGCUT1 format
	bool Save(const char* path) const {
		FILE* f = fopen(path, "wb");
		if (!f) { fprintf(stderr, "error: can not create '%s'\n", path); return false; }
		fwrite("MVSGCUT1", 1, 8, f);
		fwrite(&numNodes, 8, 1, f);
		fwrite(&numEdges, 8, 1, f);
		fwrite(edges, sizeof(EdgeRec), numEdges, f);
		fwrite(nodes, sizeof(NodeRec), numNodes, f);
		fclose(f);
		return true;
	}
	~Dump() { if (map && map != MAP_FAILED) munmap(map, mapSize); }
};

struct Result {
	double buildTime = 0, initTime = 0, solveTime = 0;
	double flow = 0;
	uint64_t numSrcSide = 0;
	uint64_t solverBytes = 0;   // analytic solver allocation
	std::vector<uint8_t> srcSide; // filled only when verifying
};

Result RunIBFS(const Dump& d, bool keepSides) {
	Result r;
	double t0 = Now();
	IBFS::IBFSGraph g;
	g.initSize((int)d.numNodes, (int)d.numEdges);
	for (uint64_t i=0; i<d.numNodes; ++i)
		g.addNode((int)i, d.nodes[i].s, d.nodes[i].t);
	for (uint64_t e=0; e<d.numEdges; ++e) {
		const EdgeRec& er = d.edges[e];
		g.addEdge((int)er.u, (int)er.v, er.capUV, er.capVU);
	}
	double t1 = Now();
	g.initGraph();
	double t2 = Now();
	r.flow = (double)g.computeMaxFlow();
	double t3 = Now();
	r.buildTime = t1-t0; r.initTime = t2-t1; r.solveTime = t3-t2;
	{
		IBFS::IBFSStats st = g.getStats();
		if (st.getAugs() >= 0)
			printf("STATS solver=ibfs augs=%.0f pushes=%.0f orphans=%.0f growthArcs=%.0f orphanArcs=%.0f/%.0f/%.0f\n",
				st.getAugs(), st.getPushes(), st.getOrphans(), st.getGrowthArcs(), st.getOrphanArcs1(), st.getOrphanArcs2(), st.getOrphanArcs3());
	}
	if (keepSides) r.srcSide.resize(d.numNodes);
	for (uint64_t i=0; i<d.numNodes; ++i) {
		const bool s = g.isNodeOnSrcSide((int)i);
		r.numSrcSide += s ? 1 : 0;
		if (keepSides) r.srcSide[i] = s ? 1 : 0;
	}
	// analytic memory (mirrors IBFSGraph::initSize): Node=48B x (n+1),
	// memArcs = max(2E*sizeof(TmpArc=16) + E*sizeof(TmpEdge=24), 2E*sizeof(Arc=24)),
	// three ActiveList Node* arrays (8B x n each), Buckets buckets+prevPtrs (8B x n each)
	const uint64_t n = d.numNodes, E = d.numEdges;
	const uint64_t arcMem = std::max<uint64_t>(2*E*16 + E*24, 2*E*24);
	r.solverBytes = 48*(n+1) + arcMem + 3*8*n + 2*8*n;
	return r;
}

// reuse: solve with an existing solver object (Reset path) instead of a fresh one;
// splitNodes: accumulate the terminal capacities through two AddNode() calls per node
Result RunTetra(const Dump& d, bool keepSides, bool check, SEACAVE::TetraFlow* reuse = nullptr, bool splitNodes = false, SEACAVE::TetraFlow::Stats* stats = nullptr) {
	Result r;
	double t0 = Now();
	SEACAVE::TetraFlow own;
	SEACAVE::TetraFlow& g = reuse ? *reuse : own;
	g.Reset(d.numNodes);
	for (uint64_t i=0; i<d.numNodes; ++i) {
		if (splitNodes) {
			const float s = d.nodes[i].s*0.5f, t = d.nodes[i].t*0.5f; // exact halves, exact sum
			g.AddNode((uint32_t)i, s, t);
			g.AddNode((uint32_t)i, d.nodes[i].s-s, d.nodes[i].t-t);
		} else
			g.AddNode((uint32_t)i, d.nodes[i].s, d.nodes[i].t);
	}
	for (uint64_t e=0; e<d.numEdges; ++e) {
		const EdgeRec& er = d.edges[e];
		g.AddEdge(er.u, er.v, er.capUV, er.capVU);
	}
	double t1 = Now();
	r.flow = g.ComputeMaxFlow(); // finalize + solve
	double t2 = Now();
	r.buildTime = t1-t0; r.initTime = 0; r.solveTime = t2-t1;
	if (keepSides) r.srcSide.resize(d.numNodes);
	for (uint64_t i=0; i<d.numNodes; ++i) {
		const bool s = g.IsNodeOnSrcSide((uint32_t)i);
		r.numSrcSide += s ? 1 : 0;
		if (keepSides) r.srcSide[i] = s ? 1 : 0;
	}
	r.solverBytes = g.GetMemoryBytes();
	if (check && !g.CheckMaxFlow()) {
		fprintf(stderr, "FAIL: tetra max-flow certificate failed (augmenting path remains)\n");
		exit(2);
	}
	if (stats) {
		const SEACAVE::TetraFlow::Stats& st = g.GetStats();
		stats->augmentations += st.augmentations; stats->walkBottleneckS += st.walkBottleneckS; stats->walkBottleneckT += st.walkBottleneckT;
		stats->walkPushS += st.walkPushS; stats->walkPushT += st.walkPushT; stats->orphans += st.orphans; stats->orphansBucket += st.orphansBucket;
		stats->arcScansGrowth += st.arcScansGrowth; stats->arcScansAdoption += st.arcScansAdoption; stats->growthPassesS += st.growthPassesS;
		stats->growthPassesT += st.growthPassesT; stats->deficitPulls += st.deficitPulls; stats->deficitOrphans += st.deficitOrphans;
		stats->conversions += st.conversions; stats->deficitFrees += st.deficitFrees; stats->endedOnS += st.endedOnS;
		stats->boundExhaustions += st.boundExhaustions; stats->phantomFlow += st.phantomFlow;
	}
	return r;
}

// capacity of the cut induced by the src-side assignment; equals the max flow at optimum
double CutValue(const Dump& d, const std::vector<uint8_t>& srcSide) {
	double cut = 0;
	for (uint64_t i=0; i<d.numNodes; ++i)
		cut += srcSide[i] ? (double)d.nodes[i].t : (double)d.nodes[i].s;
	for (uint64_t e=0; e<d.numEdges; ++e) {
		const EdgeRec& er = d.edges[e];
		const bool su = srcSide[er.u] != 0, sv = srcSide[er.v] != 0;
		if (su && !sv) cut += (double)er.capUV;
		else if (sv && !su) cut += (double)er.capVU;
	}
	return cut;
}

bool CheckClose(double a, double b, double relTol, const char* what) {
	const double diff = std::fabs(a-b);
	const double scale = std::max({std::fabs(a), std::fabs(b), 1.0});
	if (diff <= relTol*scale)
		return true;
	fprintf(stderr, "FAIL: %s mismatch: %.9g vs %.9g (rel %.3g)\n", what, a, b, diff/scale);
	return false;
}

// build a random degree<=4 graph
void RandomGraph(std::mt19937& rng, uint32_t numNodes, uint32_t targetEdges, Dump& d) {
	auto randCap = [&rng]() -> float {
		const uint32_t k = rng()%10;
		if (k < 3) return 0.f;                       // saturated/absent direction
		if (k < 6) return (float)(rng()%8);          // small integers (exact float math)
		return (float)(rng()%10000)/32.f;            // general values
	};
	d.ownNodes.resize(numNodes);
	for (auto& n : d.ownNodes) {
		n.s = (rng()%3 == 0) ? randCap() : 0.f;
		n.t = (rng()%3 == 0) ? randCap() : 0.f;
	}
	std::vector<uint8_t> deg(numNodes, 0);
	d.ownEdges.clear();
	for (uint32_t attempts=0; attempts<targetEdges*4 && d.ownEdges.size()<targetEdges; ++attempts) {
		const uint32_t u = rng()%numNodes, v = rng()%numNodes;
		if (u == v || deg[u] >= 4 || deg[v] >= 4)
			continue;
		++deg[u]; ++deg[v];
		d.ownEdges.push_back({u, v, randCap(), randCap()});
	}
	d.numNodes = numNodes;
	d.numEdges = d.ownEdges.size();
	d.edges = d.ownEdges.data();
	d.nodes = d.ownNodes.data();
}

int SelfTestOne(int iter, bool print, bool skipIBFS) {
	// per-iteration seed so any failure reproduces standalone via --selftest-iter
	std::mt19937 rng(20260826u + (uint32_t)iter*2654435761u);
	Dump d;
	const uint32_t n = 2 + rng()%(iter < 600 ? 24u : 3000u);
	RandomGraph(rng, n, rng()%(2*n+1), d);
	if (print) {
		printf("graph iter=%d n=%" PRIu64 " e=%" PRIu64 "\n", iter, d.numNodes, d.numEdges);
		for (uint64_t i=0; i<d.numNodes; ++i)
			printf("n %" PRIu64 " %g %g\n", i, d.nodes[i].s, d.nodes[i].t);
		for (uint64_t e=0; e<d.numEdges; ++e)
			printf("a %u %u %g %g\n", d.edges[e].u, d.edges[e].v, d.edges[e].capUV, d.edges[e].capVU);
		fflush(stdout);
	}
	// tetra first: its correctness is provable standalone through the certificates
	// (cut capacity == flow, and no augmenting path left in the residual graph)
	const Result rt = RunTetra(d, true, true);
	bool ok = CheckClose(CutValue(d, rt.srcSide), rt.flow, 1e-5, "tetra cut certificate");
	if (!skipIBFS) {
		// the reference implementation as an independent oracle for the flow value
		const Result ri = RunIBFS(d, true);
		ok = CheckClose(ri.flow, rt.flow, 1e-5, "flow ibfs/tetra") && ok;
		ok = CheckClose(CutValue(d, ri.srcSide), ri.flow, 1e-5, "ibfs cut certificate") && ok;
		if (ri.numSrcSide != rt.numSrcSide)
			printf("note: iter %d srcside tetra=%" PRIu64 " ibfs=%" PRIu64 " (both cuts are minimum)\n", iter, rt.numSrcSide, ri.numSrcSide);
	}
	if (!ok)
		fprintf(stderr, "FAIL at iter %d: n=%u e=%" PRIu64 "\n", iter, n, d.numEdges);
	return ok ? 0 : 1;
}

int SelfTest(int only, bool skipIBFS) {
	if (only >= 0)
		return SelfTestOne(only, true, skipIBFS);
	for (int iter=0; iter<800; ++iter)
		if (SelfTestOne(iter, false, skipIBFS))
			return 1;
	printf("selftest OK: 800 random graphs%s, cut certificates hold\n",
		skipIBFS ? " (tetra only)" : ", ibfs==tetra flow");
	return 0;
}

// exact reference for small graphs: Dinic max-flow in double precision (exact for integer capacities);
// a third, structurally different oracle for the flow value
double DinicMaxFlow(const Dump& d) {
	struct Arc { uint32_t to, rev; double cap; };
	const uint32_t n = (uint32_t)d.numNodes, S = n, T = n+1;
	std::vector<std::vector<Arc>> adj(n+2);
	auto add = [&adj](uint32_t u, uint32_t v, double cuv, double cvu) {
		adj[u].push_back({v, (uint32_t)adj[v].size(), cuv});
		adj[v].push_back({u, (uint32_t)adj[u].size()-1, cvu});
	};
	for (uint32_t i=0; i<n; ++i) {
		if (d.nodes[i].s > 0) add(S, i, d.nodes[i].s, 0);
		if (d.nodes[i].t > 0) add(i, T, d.nodes[i].t, 0);
	}
	for (uint64_t e=0; e<d.numEdges; ++e)
		add(d.edges[e].u, d.edges[e].v, d.edges[e].capUV, d.edges[e].capVU);
	std::vector<int> level(n+2), it(n+2);
	std::vector<uint32_t> queue(n+2);
	double flow = 0;
	std::function<double(uint32_t,double)> dfs = [&](uint32_t u, double f) -> double {
		if (u == T) return f;
		for (int& i = it[u]; i < (int)adj[u].size(); ++i) {
			Arc& a = adj[u][i];
			if (a.cap > 0 && level[a.to] == level[u]+1) {
				const double r = dfs(a.to, std::min(f, a.cap));
				if (r > 0) { a.cap -= r; adj[a.to][a.rev].cap += r; return r; }
			}
		}
		return 0;
	};
	for (;;) {
		std::fill(level.begin(), level.end(), -1);
		level[S] = 0;
		size_t qh = 0, qt = 0;
		queue[qt++] = S;
		while (qh < qt) {
			const uint32_t u = queue[qh++];
			for (const Arc& a : adj[u])
				if (a.cap > 0 && level[a.to] < 0) { level[a.to] = level[u]+1; queue[qt++] = a.to; }
		}
		if (level[T] < 0) break;
		std::fill(it.begin(), it.end(), 0);
		for (double f; (f = dfs(S, 1e300)) > 0; ) flow += f;
	}
	return flow;
}

enum class Family { Random, Regular, Torus, Chain, Count };
enum class Caps { Int, Mixed, Huge, Wide, Count };
enum class Terms { Sparse, Third, All, Count };
const char* FamilyName(Family f) { static const char* n[] = {"random", "regular4", "torus", "chain"}; return n[(int)f]; }
const char* CapsName(Caps c) { static const char* n[] = {"int", "mixed", "huge", "wide"}; return n[(int)c]; }
const char* TermsName(Terms t) { static const char* n[] = {"sparse", "third", "all"}; return n[(int)t]; }

// capacity generator for the stress families
struct CapGen {
	Caps kind;
	std::mt19937& rng;
	float Mixed() {
		const uint32_t k = rng()%10;
		if (k < 3) return 0.f;
		if (k < 6) return (float)(rng()%8);
		return (float)(rng()%10000)/32.f;
	}
	float Wide() {
		if (rng()%5 == 0) return 0.f;
		const double e = (double)(rng()%2401)/100.0 - 12.0; // 10^[-12, 12]
		return (float)std::pow(10.0, e);
	}
	float Edge() {
		switch (kind) {
		case Caps::Int: return (rng()%10 < 3) ? 0.f : (float)(rng()%8);
		case Caps::Mixed: return Mixed();
		case Caps::Huge: return (rng()%20 == 0) ? 3.402823466e+34f : Mixed();
		default: return Wide();
		}
	}
	float Term() {
		switch (kind) {
		case Caps::Int: return (float)(1 + rng()%7);
		case Caps::Mixed: return Mixed();
		case Caps::Huge: return (rng()%5 == 0) ? 3.402823466e+34f : Mixed(); // the pipeline's maxCap sink weight
		default: return Wide();
		}
	}
};

// build a stress graph: edges (degree <= 4) from the family, terminals from the density pattern
void StressGraph(std::mt19937& rng, Family fam, Caps caps, Terms terms, uint32_t n, Dump& d) {
	CapGen cg{caps, rng};
	d.ownEdges.clear();
	d.ownNodes.assign(n, NodeRec{0.f, 0.f});
	std::vector<uint8_t> deg(n, 0);
	auto addEdge = [&](uint32_t u, uint32_t v) {
		if (u == v || deg[u] >= 4 || deg[v] >= 4) return;
		++deg[u]; ++deg[v];
		d.ownEdges.push_back({u, v, cg.Edge(), cg.Edge()});
	};
	switch (fam) {
	case Family::Random: {
		const uint32_t target = rng()%(2*n+1);
		for (uint32_t attempts=0; attempts<target*4 && d.ownEdges.size()<target; ++attempts)
			addEdge(rng()%n, rng()%n);
		break; }
	case Family::Regular: { // configuration model: four stubs per node, paired at random (multi-edges allowed)
		std::vector<uint32_t> stubs; stubs.reserve(4*(size_t)n);
		for (uint32_t i=0; i<n; ++i) for (int k=0; k<4; ++k) stubs.push_back(i);
		std::shuffle(stubs.begin(), stubs.end(), rng);
		for (size_t k=0; k+1<stubs.size(); k+=2) addEdge(stubs[k], stubs[k+1]);
		break; }
	case Family::Torus: { // W x H grid with wrap-around: exactly four arcs per node (double edges when W or H == 2)
		uint32_t W = 2 + rng()%(std::max(1u, (uint32_t)std::sqrt((double)n))), H = std::max(2u, n/W);
		n = W*H; d.ownNodes.assign(n, NodeRec{0.f, 0.f}); deg.assign(n, 0);
		for (uint32_t y=0; y<H; ++y) for (uint32_t x=0; x<W; ++x) {
			addEdge(y*W+x, y*W+(x+1)%W);
			addEdge(y*W+x, ((y+1)%H)*W+x);
		}
		break; }
	case Family::Chain: { // a path plus a few short chords: BFS depth ~ n, exercises deep relabeling
		for (uint32_t i=0; i+1<n; ++i) addEdge(i, i+1);
		for (uint32_t i=0; i<n; ++i) if (rng()%3 == 0) addEdge(i, std::min(n-1, i+2+(uint32_t)(rng()%4)));
		break; }
	default: break;
	}
	if (n == 0)
		terms = Terms::Count; // nothing to place
	switch (terms) {
	case Terms::Sparse: { // 1..3 sources and sinks; on chains at the two ends (longest augmenting paths)
		const uint32_t ks = 1 + rng()%3, kt = 1 + rng()%3, band = std::max(1u, n/20);
		for (uint32_t k=0; k<ks; ++k) d.ownNodes[fam == Family::Chain ? rng()%band : rng()%n].s += cg.Term();
		for (uint32_t k=0; k<kt; ++k) d.ownNodes[fam == Family::Chain ? n-1-rng()%band : rng()%n].t += cg.Term();
		break; }
	case Terms::Third:
		for (auto& nd : d.ownNodes) { if (rng()%3 == 0) nd.s = cg.Term(); if (rng()%3 == 0) nd.t = cg.Term(); }
		break;
	case Terms::All:
		for (auto& nd : d.ownNodes) { nd.s = cg.Term(); nd.t = cg.Term(); }
		break;
	default: break;
	}
	d.numNodes = n;
	d.numEdges = d.ownEdges.size();
	d.edges = d.ownEdges.data();
	d.nodes = d.ownNodes.data();
}

struct StressOptions {
	int iters = 4000;
	uint32_t seed = 20260826u;
	uint32_t maxNodes = 3000;
	bool skipIBFS = false, useDinic = true;
	bool trace = false; // print every instance before solving it (locates a crashing iteration)
	// the IBFS reference is only used from ibfsMinNodes nodes up: on tiny graphs its free-node sentinel
	// (label == numNodes) collides with a legitimate tree label and its fixed-size active lists overflow
	// when nodes are freed and re-attached within one growth pass (heap corruption); neither can happen
	// on the multi-million node Delaunay duals it is otherwise verified on
	uint32_t ibfsMinNodes = 1000;
	uint32_t dinicMaxNodes = 20000; // exact reference, O(V^2 E) worst case
};

// the IBFS reference labels stale T nodes as sink-side on some tiny graphs (its flow value is right, the
// cut it reports is not minimum): counted, not failed, so that the campaign keeps checking TetraFlow
struct StressReport {
	uint64_t ibfsCutMismatches = 0;
	int firstIbfsCutMismatch = -1;
};

int StressOne(const StressOptions& opt, int iter, bool print, SEACAVE::TetraFlow& reuse, SEACAVE::TetraFlow::Stats& agg, StressReport& report) {
	std::mt19937 rng(opt.seed + (uint32_t)iter*2654435761u);
	Family fam = (Family)(rng()%(uint32_t)Family::Count);
	const Caps caps = (Caps)(rng()%(uint32_t)Caps::Count);
	const Terms terms = (Terms)(rng()%(uint32_t)Terms::Count);
	uint32_t n;
	if (iter < 2) { n = (uint32_t)iter; fam = Family::Random; }        // the degenerate sizes first
	else if (rng()%2) n = 2 + rng()%std::min(31u, std::max(1u, opt.maxNodes-1)); // small graphs hit corner cases often
	else n = 2 + rng()%std::max(1u, opt.maxNodes-1);
	Dump d;
	StressGraph(rng, fam, caps, terms, n, d);
	const bool useReuse = rng()%2 == 0, split = rng()%3 == 0;
	if (opt.trace && !print) {
		printf("iter=%d family=%s caps=%s terms=%s n=%" PRIu64 " e=%" PRIu64 " reuse=%d split=%d\n",
			iter, FamilyName(fam), CapsName(caps), TermsName(terms), d.numNodes, d.numEdges, useReuse ? 1 : 0, split ? 1 : 0);
		fflush(stdout);
	}
	if (print) {
		printf("graph iter=%d family=%s caps=%s terms=%s n=%" PRIu64 " e=%" PRIu64 " reuse=%d split=%d\n",
			iter, FamilyName(fam), CapsName(caps), TermsName(terms), d.numNodes, d.numEdges, useReuse ? 1 : 0, split ? 1 : 0);
		for (uint64_t i=0; i<d.numNodes; ++i)
			printf("n %" PRIu64 " %.9g %.9g\n", i, d.nodes[i].s, d.nodes[i].t);
		for (uint64_t e=0; e<d.numEdges; ++e)
			printf("a %u %u %.9g %.9g\n", d.edges[e].u, d.edges[e].v, d.edges[e].capUV, d.edges[e].capVU);
		fflush(stdout);
	}
	// float solvers agree to rounding; the wide/huge regimes accumulate more absorption
	const double tol = (caps == Caps::Int) ? 0.0 : (caps == Caps::Mixed ? 1e-5 : 1e-4);
	const Result rt = RunTetra(d, true, true, useReuse ? &reuse : nullptr, split, &agg);
	bool ok = CheckClose(CutValue(d, rt.srcSide), rt.flow, std::max(tol, 1e-6), "tetra cut certificate");
	if (rt.srcSide.size() != d.numNodes) ok = false;
	if (!opt.skipIBFS && d.numNodes >= opt.ibfsMinNodes) {
		const Result ri = RunIBFS(d, true);
		ok = CheckClose(ri.flow, rt.flow, std::max(tol, 1e-6), "flow ibfs/tetra") && ok;
		const double cutIBFS = CutValue(d, ri.srcSide);
		if (std::fabs(cutIBFS-ri.flow) > std::max(tol, 1e-6)*std::max({std::fabs(cutIBFS), std::fabs(ri.flow), 1.0})) {
			if (report.ibfsCutMismatches++ == 0)
				report.firstIbfsCutMismatch = iter;
			if (print)
				printf("note: ibfs cut certificate mismatch: cut %.9g vs flow %.9g\n", cutIBFS, ri.flow);
		}
	}
	if (opt.useDinic && d.numNodes <= opt.dinicMaxNodes) {
		const double fd = DinicMaxFlow(d);
		if (caps == Caps::Int) {
			if (fd != rt.flow) { fprintf(stderr, "FAIL: exact flow dinic/tetra mismatch: %.17g vs %.17g\n", fd, rt.flow); ok = false; }
		} else
			ok = CheckClose(fd, rt.flow, tol, "flow dinic/tetra") && ok;
	}
	if (!ok)
		fprintf(stderr, "FAIL at stress iter %d (seed %u): family=%s caps=%s terms=%s n=%" PRIu64 " e=%" PRIu64 " reuse=%d split=%d\n",
			iter, opt.seed, FamilyName(fam), CapsName(caps), TermsName(terms), d.numNodes, d.numEdges, useReuse ? 1 : 0, split ? 1 : 0);
	return ok ? 0 : 1;
}

int Stress(const StressOptions& opt, int only) {
	SEACAVE::TetraFlow reuse; // shared across iterations: exercises Reset() on a used object
	SEACAVE::TetraFlow::Stats agg;
	StressReport report;
	if (only >= 0)
		return StressOne(opt, only, true, reuse, agg, report);
	for (int iter=0; iter<opt.iters; ++iter)
		if (StressOne(opt, iter, false, reuse, agg, report))
			return 1;
	printf("stress OK: %d graphs (seed %u, <= %u nodes)%s%s\n", opt.iters, opt.seed, opt.maxNodes,
		opt.skipIBFS ? "" : ", ibfs==tetra flow (>= ibfs-min-nodes)", opt.useDinic ? ", dinic==tetra flow (<= dinic-max-nodes)" : "");
	if (report.ibfsCutMismatches)
		printf("note: the ibfs reference reported a non-minimum cut on %" PRIu64 " graphs (first at iter %d; flow values still matched)\n",
			report.ibfsCutMismatches, report.firstIbfsCutMismatch);
	if (agg.augmentations)
		printf("STATS aggregate: augs=%" PRIu64 " orphans=%" PRIu64 " orphansBucket=%" PRIu64 " growthS=%" PRIu64 " growthT=%" PRIu64
			" deficitPulls=%" PRIu64 " deficitOrphans=%" PRIu64 " conversions=%" PRIu64 " deficitFrees=%" PRIu64
			" boundExhaustions=%" PRIu64 " endedOnS=%" PRIu64 " phantomFlow=%.6g\n",
			agg.augmentations, agg.orphans, agg.orphansBucket, agg.growthPassesS, agg.growthPassesT,
			agg.deficitPulls, agg.deficitOrphans, agg.conversions, agg.deficitFrees, agg.boundExhaustions, agg.endedOnS, agg.phantomFlow);
	return 0;
}

} // namespace

int main(int argc, char** argv) {
	const char* file = nullptr;
	const char* save = nullptr;
	const char* sidesFile = nullptr;
	std::string solver = "tetra";
	int repeat = 1, selftestIter = -1, stressIter = -1;
	uint64_t subgraph = 0;
	bool verify = false, selftest = false, skipIBFS = false, reorder = false, stress = false;
	StressOptions stressOpt;
	for (int i=1; i<argc; ++i) {
		if (!strcmp(argv[i], "--solver") && i+1 < argc) solver = argv[++i];
		else if (!strcmp(argv[i], "--stress")) stress = true;
		else if (!strcmp(argv[i], "--stress-iter") && i+1 < argc) { stress = true; stressIter = atoi(argv[++i]); }
		else if (!strcmp(argv[i], "--iters") && i+1 < argc) stressOpt.iters = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--seed") && i+1 < argc) stressOpt.seed = (uint32_t)strtoul(argv[++i], nullptr, 10);
		else if (!strcmp(argv[i], "--max-nodes") && i+1 < argc) stressOpt.maxNodes = (uint32_t)strtoul(argv[++i], nullptr, 10);
		else if (!strcmp(argv[i], "--no-dinic")) stressOpt.useDinic = false;
		else if (!strcmp(argv[i], "--trace")) stressOpt.trace = true;
		else if (!strcmp(argv[i], "--ibfs-min-nodes") && i+1 < argc) stressOpt.ibfsMinNodes = (uint32_t)strtoul(argv[++i], nullptr, 10);
		else if (!strcmp(argv[i], "--dinic-max-nodes") && i+1 < argc) stressOpt.dinicMaxNodes = (uint32_t)strtoul(argv[++i], nullptr, 10);
		else if (!strcmp(argv[i], "--subgraph") && i+1 < argc) subgraph = strtoull(argv[++i], nullptr, 10);
		else if (!strcmp(argv[i], "--repeat") && i+1 < argc) repeat = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--verify")) verify = true;
		else if (!strcmp(argv[i], "--selftest")) selftest = true;
		else if (!strcmp(argv[i], "--selftest-iter") && i+1 < argc) { selftest = true; selftestIter = atoi(argv[++i]); }
		else if (!strcmp(argv[i], "--skip-ibfs")) skipIBFS = true;
		else if (!strcmp(argv[i], "--reorder")) reorder = true;
		else if (!strcmp(argv[i], "--save") && i+1 < argc) save = argv[++i];
		else if (!strcmp(argv[i], "--sides") && i+1 < argc) { sidesFile = argv[++i]; verify = true; }
		else if (argv[i][0] != '-') file = argv[i];
		else { fprintf(stderr, "unknown option '%s'\n", argv[i]); return 1; }
	}
	if (stress) {
		stressOpt.skipIBFS = skipIBFS;
		return Stress(stressOpt, stressIter);
	}
	if (selftest)
		return SelfTest(selftestIter, skipIBFS);
	if (!file) {
		fprintf(stderr, "usage: GraphCutBench <graph.gcut> [--solver tetra|ibfs] [--subgraph N] [--reorder] [--save out.gcut] [--repeat N] [--verify] [--sides out.bin]\n"
			"       GraphCutBench --selftest [--skip-ibfs] | --selftest-iter K\n"
			"       GraphCutBench --stress [--iters N] [--seed S] [--max-nodes N] [--skip-ibfs] [--no-dinic] [--ibfs-min-nodes N] [--dinic-max-nodes N] [--trace] | --stress-iter K\n");
		return 1;
	}

	Dump d;
	if (!d.Load(file))
		return 1;
	if (subgraph > 0)
		d.Subgraph(subgraph);
	if (reorder)
		d.Reorder();
	if (save) {
		if (!d.Save(save))
			return 1;
		printf("saved '%s'\n", save);
	}
	printf("loaded '%s': %" PRIu64 " nodes, %" PRIu64 " edges (est. peak: tetra %.0f MB, ibfs %.0f MB)\n",
		file, d.numNodes, d.numEdges, 70.0*d.numNodes/1048576.0, 200.0*d.numNodes/1048576.0);

	for (int it=0; it<repeat; ++it) {
		Result r;
		if (solver == "tetra") r = RunTetra(d, verify, verify);
		else if (solver == "ibfs") r = RunIBFS(d, verify);
		else { fprintf(stderr, "unknown solver '%s'\n", solver.c_str()); return 1; }
		double cut = 0;
		if (verify) {
			cut = CutValue(d, r.srcSide);
			if (!CheckClose(cut, r.flow, 1e-4, "cut certificate"))
				return 2;
		}
		printf("RESULT file=%s solver=%s nodes=%" PRIu64 " edges=%" PRIu64
			" build=%.3f init=%.3f solve=%.3f total=%.3f flow=%.9g srcside=%" PRIu64
			" solverMB=%.1f peakMB=%.1f%s\n",
			file, solver.c_str(), d.numNodes, d.numEdges,
			r.buildTime, r.initTime, r.solveTime, r.buildTime+r.initTime+r.solveTime,
			r.flow, r.numSrcSide,
			(double)r.solverBytes/(1024.0*1024.0), (double)PeakMemory()/(1024.0*1024.0),
			verify ? " verified=1" : "");
		if (verify)
			printf("CUT solver=%s cut=%.9g flow=%.9g diff=%.6g rel=%.3g\n", solver.c_str(), cut, r.flow, cut-r.flow, std::fabs(cut-r.flow)/std::max(1.0, std::fabs(r.flow)));
		if (sidesFile && it == 0) {
			// one byte per node (1: source side), to compare the cuts of two solvers with cmp
			FILE* f = fopen(sidesFile, "wb");
			if (!f) { fprintf(stderr, "error: can not create '%s'\n", sidesFile); return 1; }
			fwrite(r.srcSide.data(), 1, r.srcSide.size(), f);
			fclose(f);
		}
		fflush(stdout);
	}
	return 0;
}
