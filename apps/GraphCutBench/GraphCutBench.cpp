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
 *   GraphCutBench <graph.gcut> [--solver ibfs|gc4] [--subgraph N] [--reorder] [--repeat N] [--verify]
 *     --subgraph N  solve only the first N nodes (a spatially coherent prefix of the scene)
 *     --reorder     renumber the nodes in BFS order before solving (locality experiment)
 *     --verify      check the cut-value certificate (cut capacity == max flow)
 *   GraphCutBench --selftest [--skip-ibfs]      fuzz both solvers on random degree<=4 graphs
 *   GraphCutBench --selftest-iter K             print and re-run a single fuzz iteration
 *
 * Solver memory is roughly 200 B/node (IBFS) and 76 B/node (GC4); the loaded graph is
 * mmap-ed and does not count towards the reported peak footprint.
 */

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
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
#include "../../libs/Math/GraphCut4.h"

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

Result RunGC4(const Dump& d, bool keepSides, bool check) {
	Result r;
	double t0 = Now();
	GC4::Graph g;
	g.initSize(d.numNodes);
	for (uint64_t i=0; i<d.numNodes; ++i)
		g.addNode((uint32_t)i, d.nodes[i].s, d.nodes[i].t);
	for (uint64_t e=0; e<d.numEdges; ++e) {
		const EdgeRec& er = d.edges[e];
		g.addEdge(er.u, er.v, er.capUV, er.capVU);
	}
	double t1 = Now();
	g.initGraph();
	double t2 = Now();
	r.flow = (double)g.computeMaxFlow();
	double t3 = Now();
	r.buildTime = t1-t0; r.initTime = t2-t1; r.solveTime = t3-t2;
	if (GC4_STATS) {
		const GC4::Graph::Stats& st = g.getStats();
		printf("STATS solver=gc4 augs=%llu pushes=%llu orphans=%llu growthArcs=%llu\n",
			(unsigned long long)st.augs, (unsigned long long)st.pushes, (unsigned long long)st.orphans, (unsigned long long)st.growthArcs);
	}
	if (keepSides) r.srcSide.resize(d.numNodes);
	for (uint64_t i=0; i<d.numNodes; ++i) {
		const bool s = g.isNodeOnSrcSide((uint32_t)i);
		r.numSrcSide += s ? 1 : 0;
		if (keepSides) r.srcSide[i] = s ? 1 : 0;
	}
	r.solverBytes = g.getMemoryBytes();
	if (check && !g.checkMaxFlow()) {
		fprintf(stderr, "FAIL: gc4 max-flow certificate failed (augmenting path remains)\n");
		exit(2);
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
	// gc4 first: its correctness is provable standalone through the certificates
	const Result rg = RunGC4(d, true, true);
	bool ok = CheckClose(CutValue(d, rg.srcSide), rg.flow, 1e-5, "gc4 cut certificate");
	if (!skipIBFS) {
		const Result ri = RunIBFS(d, true);
		ok = CheckClose(ri.flow, rg.flow, 1e-5, "flow ibfs/gc4") && ok;
		ok = CheckClose(CutValue(d, ri.srcSide), ri.flow, 1e-5, "ibfs cut certificate") && ok;
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
		skipIBFS ? " (gc4 only)" : ", ibfs==gc4 flow");
	return 0;
}

} // namespace

int main(int argc, char** argv) {
	const char* file = nullptr;
	std::string solver = "gc4";
	int repeat = 1, selftestIter = -1;
	uint64_t subgraph = 0;
	bool verify = false, selftest = false, skipIBFS = false, reorder = false;
	for (int i=1; i<argc; ++i) {
		if (!strcmp(argv[i], "--solver") && i+1 < argc) solver = argv[++i];
		else if (!strcmp(argv[i], "--subgraph") && i+1 < argc) subgraph = strtoull(argv[++i], nullptr, 10);
		else if (!strcmp(argv[i], "--repeat") && i+1 < argc) repeat = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--verify")) verify = true;
		else if (!strcmp(argv[i], "--selftest")) selftest = true;
		else if (!strcmp(argv[i], "--selftest-iter") && i+1 < argc) { selftest = true; selftestIter = atoi(argv[++i]); }
		else if (!strcmp(argv[i], "--skip-ibfs")) skipIBFS = true;
		else if (!strcmp(argv[i], "--reorder")) reorder = true;
		else if (argv[i][0] != '-') file = argv[i];
		else { fprintf(stderr, "unknown option '%s'\n", argv[i]); return 1; }
	}
	if (selftest)
		return SelfTest(selftestIter, skipIBFS);
	if (!file) {
		fprintf(stderr, "usage: GraphCutBench <graph.gcut> [--solver ibfs|gc4] [--subgraph N] [--reorder] [--repeat N] [--verify] | --selftest\n");
		return 1;
	}

	Dump d;
	if (!d.Load(file))
		return 1;
	if (subgraph > 0)
		d.Subgraph(subgraph);
	if (reorder)
		d.Reorder();
	printf("loaded '%s': %" PRIu64 " nodes, %" PRIu64 " edges (est. peak: ibfs %.0f MB, gc4 %.0f MB)\n",
		file, d.numNodes, d.numEdges, 200.0*d.numNodes/1048576.0, 64.0*d.numNodes/1048576.0);

	for (int it=0; it<repeat; ++it) {
		Result r;
		if (solver == "ibfs") r = RunIBFS(d, verify);
		else if (solver == "gc4") r = RunGC4(d, verify, verify);
		else { fprintf(stderr, "unknown solver '%s'\n", solver.c_str()); return 1; }
		if (verify && !CheckClose(CutValue(d, r.srcSide), r.flow, 1e-4, "cut certificate"))
			return 2;
		printf("RESULT file=%s solver=%s nodes=%" PRIu64 " edges=%" PRIu64
			" build=%.3f init=%.3f solve=%.3f total=%.3f flow=%.9g srcside=%" PRIu64
			" solverMB=%.1f peakMB=%.1f%s\n",
			file, solver.c_str(), d.numNodes, d.numEdges,
			r.buildTime, r.initTime, r.solveTime, r.buildTime+r.initTime+r.solveTime,
			r.flow, r.numSrcSide,
			(double)r.solverBytes/(1024.0*1024.0), (double)PeakMemory()/(1024.0*1024.0),
			verify ? " verified=1" : "");
		fflush(stdout);
	}
	return 0;
}
