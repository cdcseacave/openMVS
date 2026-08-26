////////////////////////////////////////////////////////////////////
// TetraFlow.h
//
// Copyright 2026 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _MATH_TETRAFLOW_H_
#define _MATH_TETRAFLOW_H_


// I N C L U D E S /////////////////////////////////////////////////

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <vector>
#if defined(__has_include)
#if __has_include(<version>)
#include <version>
#endif
#endif
#if defined(__cpp_lib_bitops) && __cpp_lib_bitops >= 201907L
#include <bit>
#endif


// D E F I N E S ///////////////////////////////////////////////////

// software prefetch (no-op on compilers without a builtin; may be overridden)
#ifndef TETRAFLOW_PREFETCH
#if defined(__GNUC__) || defined(__clang__)
#define TETRAFLOW_PREFETCH(ptr) __builtin_prefetch(ptr)
#elif defined(_MSC_VER) && (defined(_M_IX86) || defined(_M_X64))
#include <xmmintrin.h>
#define TETRAFLOW_PREFETCH(ptr) _mm_prefetch(reinterpret_cast<const char*>(ptr), _MM_HINT_T0)
#else
#define TETRAFLOW_PREFETCH(ptr) static_cast<void>(ptr)
#endif
#endif
// look-ahead (in scan-list entries) of the node-line prefetch during growth
#ifndef TETRAFLOW_PREFETCH_DIST
#define TETRAFLOW_PREFETCH_DIST 4
#endif
// 1: batched source-side augmentation: the augmentations of a growth pass record deficits at their
// S endpoint that one level-ordered sweep pulls down the tree afterwards; a deficit node that loses
// its tree keeps the deficit while free until S re-adopts it or T reaches it (0: exact phase-1 walks)
#ifndef TETRAFLOW_DEFICITS
#define TETRAFLOW_DEFICITS 1
#endif
// 1 (with TETRAFLOW_DEFICITS): bound the amount committed at a bridge by a cached estimate of the
// S-side bottleneck (the root's unclaimed source excess min'ed with every tree arc down to the node);
// a node that has used up its estimate is scanned again after the deficits of the pass are settled
#ifndef TETRAFLOW_DEFICIT_BOUND
#define TETRAFLOW_DEFICIT_BOUND 0
#endif
#if TETRAFLOW_DEFICIT_BOUND && !TETRAFLOW_DEFICITS
#error "TETRAFLOW_DEFICIT_BOUND requires TETRAFLOW_DEFICITS"
#endif

// branch hints (no-ops on compilers without the builtin)
#if defined(__GNUC__) || defined(__clang__)
#define TETRAFLOW_LIKELY(cond) __builtin_expect(!!(cond), 1)
#define TETRAFLOW_UNLIKELY(cond) __builtin_expect(!!(cond), 0)
#else
#define TETRAFLOW_LIKELY(cond) (cond)
#define TETRAFLOW_UNLIKELY(cond) (cond)
#endif

// C++20 branch attributes (empty in C++17)
#if defined(__has_cpp_attribute) && __cplusplus >= 202002L
#if __has_cpp_attribute(likely) >= 201803L && __has_cpp_attribute(unlikely) >= 201803L
#define TETRAFLOW_LIKELY_BRANCH [[likely]]
#define TETRAFLOW_UNLIKELY_BRANCH [[unlikely]]
#endif
#endif
#ifndef TETRAFLOW_LIKELY_BRANCH
#define TETRAFLOW_LIKELY_BRANCH
#define TETRAFLOW_UNLIKELY_BRANCH
#endif

// debug-only statements (bookkeeping that exists only to feed assertions)
#ifndef NDEBUG
#define TETRAFLOW_DEBUG(...) __VA_ARGS__
#else
#define TETRAFLOW_DEBUG(...)
#endif

// optional operation counters: define TETRAFLOW_STATS=1 to compile them in (zero cost otherwise)
#ifndef TETRAFLOW_STATS
#define TETRAFLOW_STATS 0
#endif
#if TETRAFLOW_STATS
#define TETRAFLOW_STAT(counter) (++stats.counter)
#define TETRAFLOW_STAT_ADD(counter, value) (stats.counter += (value))
#else
#define TETRAFLOW_STAT(counter) static_cast<void>(0)
#define TETRAFLOW_STAT_ADD(counter, value) static_cast<void>(0)
#endif


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Independent implementation of the incremental breadth-first search max-flow algorithm
// (A. V. Goldberg, S. Hed, H. Kaplan, R. E. Tarjan, R. F. Werneck, "Maximum flows by incremental
// breadth-first search", ESA 2011), written from a behavioral description of the published method,
// with the data structures specialized for graphs in which every node has exactly four arcs (the
// dual graph of a tetrahedralization: one node per cell, one edge per facet shared by two cells).
// No source code of any existing max-flow implementation was consulted while writing it.
//
// Two breadth-first trees are maintained in the residual graph: S grows from the source (positive
// labels = tree depth), T grows from the sink (negative labels); free nodes have label 0. The trees
// grow one full level at a time; whenever a tree arc reaches a node of the other tree the augmenting
// path is saturated and the orphaned sub-trees are re-attached (or relabeled deeper, or freed) by the
// adoption procedure. The flow is maximum when one of the trees can no longer grow.
//
// Every node occupies exactly one 64-byte cache line holding its four arcs and all its tree state.
class TetraFlow
{
public:
	using NodeID = uint32_t;
	using Cap = float;
	using Label = int32_t;
	static constexpr NodeID NO_NODE = ~NodeID(0);
	static constexpr size_t MAX_NODES = size_t(0x7FFFFFFF);

	TetraFlow() = default;
	// same as Reset(numNodes)
	explicit TetraFlow(size_t numNodes) { Reset(numNodes); }

	// (re)allocate storage for numNodes nodes; all capacities zero; any previous state is discarded
	void Reset(size_t numNodes) {
		assert(numNodes <= MAX_NODES);
		nodes.clear();
		if (nodes.capacity() < numNodes || nodes.capacity() > numNodes + numNodes/2) {
			std::vector<Node>().swap(nodes);
			nodes.reserve(numNodes);
		}
		nodes.resize(numNodes); // value-initialized: all zero
		assert(nodes.empty() || reinterpret_cast<uintptr_t>(nodes.data()) % alignof(Node) == 0);
		frontierS.clear();
		frontierT.clear();
		scan.clear();
		bucketHead.clear();
		#if TETRAFLOW_DEFICITS
		deficitQueue.clear();
		#endif
		ResetSolverState();
	}

	// accumulate terminal capacities of node n (may be called several times for the same node)
	void AddNode(NodeID n, Cap capSource, Cap capSink) {
		assert(n < nodes.size());
		assert(capSource >= 0 && capSink >= 0);
		Node& node = nodes[n];
		node.excess += capSource;  // holds the accumulated source capacity until ComputeMaxFlow()
		node.capSink += capSink;   // aliases the (unused until then) prev link
	}

	// add the undirected edge (u,v) with capacity capUV for u->v and capVU for v->u;
	// called exactly once per edge; u != v; every node must end up with at most 4 edges
	void AddEdge(NodeID u, NodeID v, Cap capUV, Cap capVU) {
		assert(u < nodes.size() && v < nodes.size() && u != v);
		assert(capUV >= 0 && capVU >= 0);
		Node& nu = nodes[u];
		Node& nv = nodes[v];
		assert(nu.fill < NUM_SLOTS && nv.fill < NUM_SLOTS);
		const unsigned iu = nu.fill++;
		const unsigned iv = nv.fill++;
		nu.head[iu] = v; nu.rcap[iu] = capUV; SetRev(nu, iu, iv);
		nv.head[iv] = u; nv.rcap[iv] = capVU; SetRev(nv, iv, iu);
	}

	// finalize the graph and compute the maximum flow; returns its value; call once
	double ComputeMaxFlow() {
		Init();
		bool dirS = true;
		for (;;) {
			if (dirS) {
				++levelS;
				Grow<true>();
			} else {
				++levelT;
				Grow<false>();
			}
			#if TETRAFLOW_DEFICITS
			ResolveDeficits();
			if (frontierT.empty())
				ConvertPendingFree(); // T exhausted: the free deficit nodes become sinks, T may grow again
			#endif
			if (frontierS.empty() || frontierT.empty()) {
				TETRAFLOW_STAT_ADD(endedOnS, frontierS.empty() ? 1 : 0);
				break;
			}
			// grow next the tree that produced fewer unique orphans so far; alternate on a tie
			dirS = !((uniqOrphansT == uniqOrphansS && dirS) || uniqOrphansT < uniqOrphansS);
		}
		return flow;
	}

	// value returned by ComputeMaxFlow()
	[[nodiscard]] double GetFlow() const noexcept { return flow; }

	// true if node n is on the source side of the minimum cut (valid after ComputeMaxFlow):
	// if T could not grow it is exactly the set of nodes that can still reach the sink and everything
	// else is on the source side; otherwise S is exactly the set of nodes reachable from the source
	[[nodiscard]] bool IsNodeOnSrcSide(NodeID n) const noexcept {
		assert(n < nodes.size());
		const Label label = nodes[n].label;
		return label > 0 || (label == 0 && frontierT.empty());
	}

	// number of nodes
	[[nodiscard]] size_t GetNumNodes() const noexcept { return nodes.size(); }

	// bytes of heap memory currently owned by this object (nodes + auxiliary buffers, use capacities)
	[[nodiscard]] size_t GetMemoryBytes() const noexcept {
		size_t bytes = nodes.capacity() * sizeof(Node) +
			(frontierS.capacity() + frontierT.capacity() + scan.capacity() + bucketHead.capacity() +
			 pathS.capacity() + pathT.capacity()) * sizeof(NodeID);
		#if TETRAFLOW_DEFICITS
		bytes += deficitQueue.capacity() * sizeof(std::vector<NodeID>);
		bytes += pendingFree.capacity() * sizeof(NodeID);
		#if TETRAFLOW_DEFICIT_BOUND
		bytes += (cutS.capacity() + cutT.capacity()) * sizeof(NodeID);
		#endif
		for (const std::vector<NodeID>& level: deficitQueue)
			bytes += level.capacity() * sizeof(NodeID);
		#endif
		return bytes;
	}

	// optimality check for tests: breadth-first search from all nodes with residual source capacity
	// over residual arcs must not reach a node with residual sink capacity; O(N)
	[[nodiscard]] bool CheckMaxFlow() const {
		std::vector<uint8_t> marks(nodes.size(), 0);
		std::vector<NodeID> queue;
		for (size_t x = 0; x < nodes.size(); ++x) {
			if (nodes[x].excess > 0) {
				marks[x] = 1;
				queue.push_back(NodeID(x));
			}
		}
		for (size_t k = 0; k < queue.size(); ++k) {
			const Node& node = nodes[queue[k]];
			for (unsigned i = 0; i < NUM_SLOTS; ++i) {
				if (node.rcap[i] <= 0)
					continue;
				const NodeID y = node.head[i];
				if (marks[y])
					continue;
				if (nodes[y].excess < 0)
					return false;
				marks[y] = 1;
				queue.push_back(y);
			}
		}
		return true;
	}

	// operation counters, updated only when compiled with TETRAFLOW_STATS=1
	struct Stats {
		uint64_t augmentations = 0;    // augmenting paths saturated
		uint64_t walkBottleneckS = 0;  // steps of the bottleneck walks on the S side
		uint64_t walkBottleneckT = 0;  // steps of the bottleneck walks on the T side
		uint64_t walkPushS = 0;        // steps of the push walks on the S side
		uint64_t walkPushT = 0;        // steps of the push walks on the T side
		uint64_t orphans = 0;          // orphans processed by adoption
		uint64_t orphansBucket = 0;    // orphans that entered the three-pass buckets
		uint64_t arcScansGrowth = 0;   // arcs examined by growth
		uint64_t arcScansAdoption = 0; // arcs examined by adoption (including the three-pass procedure)
		uint64_t growthPassesS = 0;    // growth passes on S
		uint64_t growthPassesT = 0;    // growth passes on T
		uint64_t deficitPulls = 0;     // steps of the deficit sweeps (TETRAFLOW_DEFICITS)
		uint64_t deficitOrphans = 0;   // S orphans created by deficit pulls
		uint64_t conversions = 0;      // S nodes converted into T roots while holding a deficit
		uint64_t deficitFrees = 0;     // deficit nodes that lost their tree and wait free for S or T
		uint64_t endedOnS = 0;         // 1 if the main loop ended because S could not grow (0: T)
		uint64_t boundExhaustions = 0; // scans cut short by a node using up its bottleneck estimate (TETRAFLOW_DEFICIT_BOUND)
		double phantomFlow = 0;        // total deficit converted into sink capacity
	};
	// the counters (all zero unless TETRAFLOW_STATS=1)
	[[nodiscard]] const Stats& GetStats() const noexcept { return stats; }

private:
	using Slot = uint8_t;
	static constexpr unsigned NUM_SLOTS = 4;
	static constexpr Slot NO_SLOT = 0xFF;
	static constexpr size_t PREFETCH_DIST = TETRAFLOW_PREFETCH_DIST; // scan-list look-ahead of the node-line prefetch

	// one node = one cache line: the four arcs (residual capacity and neighbor id per slot) and the
	// complete tree state, so that every step of a path walk touches a single line
	struct alignas(64) Node {
		Cap rcap[NUM_SLOTS];      // residual capacity of the arc through slot i (this node -> head[i])
		NodeID head[NUM_SLOTS];   // neighbor reached through slot i
		Cap excess;               // residual source capacity (> 0, S root) or minus the residual sink capacity (< 0, T root); 0 otherwise
		Label label;              // depth in S (> 0), minus the depth in T (< 0), or 0 if free
		NodeID next;              // link for the orphan list and the buckets
		union {
			NodeID prev;          // link for the buckets
			Cap capSink;          // accumulated sink capacity during construction (prev is unused until ComputeMaxFlow)
		};
		#if TETRAFLOW_DEFICIT_BOUND
		union {
			NodeID parent;        // T nodes: parent node id (== head[parentSlot], cached to shorten the walks)
			Cap sBound;           // S nodes: estimate of the residual bottleneck on the path from the root, less what was committed
		};
		#else
		NodeID parent;            // parent node id (== head[parentSlot], cached to shorten the walks)
		#endif
		#if TETRAFLOW_DEFICITS
		Cap deficit;              // flow pushed towards the sink but not yet received from the parent (S nodes only)
		uint16_t lastAugTs;       // timestamp of the last push phase in which the node was orphaned
		#else
		uint32_t lastAugTs;       // timestamp of the last push phase in which the node was orphaned
		uint16_t pad;
		#endif
		Slot parentSlot;          // slot towards the parent, NO_SLOT for roots and free nodes
		uint8_t isParentCurr;     // true if the slots before parentSlot are known useless as parent arcs at the current label
		uint8_t revres;           // bit i set iff the reverse arc head[i] -> this node has residual capacity
		uint8_t revSlots;         // 2 bits per slot: the slot of head[i] through which it sees this node back
		uint8_t children;         // bit i set iff head[i] is a child of this node in its tree
		union {
			uint8_t fill;         // slots used so far by AddEdge (construction only)
			uint8_t listState;    // debug-only: which intrusive list holds the node (LIST_NONE/LIST_ORPHAN/LIST_BUCKET)
		};
	};
	static_assert(sizeof(Node) == 64, "a node must occupy exactly one cache line");
	static_assert(alignof(Node) == 64, "a node must be cache-line aligned");

	enum : uint8_t { LIST_NONE = 0, LIST_ORPHAN = 1, LIST_BUCKET = 2 };
	#if TETRAFLOW_DEFICITS
	using AugTs = uint16_t; // wraps around: harmless, it only feeds the unique-orphan heuristic
	#else
	using AugTs = uint32_t;
	#endif

	static constexpr uint8_t Bit(unsigned i) noexcept { return uint8_t(1u << i); }
	// index of the lowest set bit of a non-zero mask
	static unsigned LowestBit(unsigned mask) noexcept {
		assert(mask != 0);
		#if defined(__cpp_lib_bitops) && __cpp_lib_bitops >= 201907L
		return unsigned(std::countr_zero(mask));
		#elif defined(__GNUC__) || defined(__clang__)
		return unsigned(__builtin_ctz(mask));
		#else
		unsigned i = 0;
		while (!(mask & (1u << i)))
			++i;
		return i;
		#endif
	}
	// the node array, with its alignment made known to the compiler
	Node* NodeData() noexcept {
		#if defined(__cpp_lib_assume_aligned) && __cpp_lib_assume_aligned >= 201811L
		return std::assume_aligned<alignof(Node)>(nodes.data());
		#else
		return nodes.data();
		#endif
	}
	// rev(x,i): slot of head[i] through which it sees x back
	static unsigned Rev(const Node& node, unsigned i) noexcept { return (node.revSlots >> (2*i)) & 3u; }
	static void SetRev(Node& node, unsigned i, unsigned j) noexcept { node.revSlots = uint8_t((node.revSlots & ~(3u << (2*i))) | (j << (2*i))); }
	// signed label of the level L in tree S/T
	template <bool S>
	static constexpr Label Sgn(Label L) noexcept { return S ? L : -L; }
	// residual arc in the growth direction of the tree: away from the root (S: x->y, T: y->x)
	template <bool S>
	static bool Forward(const Node& node, unsigned i) noexcept { return S ? node.rcap[i] > 0 : ((node.revres >> i) & 1u) != 0; }
	// residual arc towards the root of the tree (S: y->x, T: x->y): the candidate-parent direction
	template <bool S>
	static bool Backward(const Node& node, unsigned i) noexcept { return S ? ((node.revres >> i) & 1u) != 0 : node.rcap[i] > 0; }
	// a root or a node with a parent (as opposed to a pending orphan waiting in a bucket)
	static bool IsAttached(const Node& node) noexcept { return node.excess != 0 || node.parentSlot != NO_SLOT; }
	// the parent of an attached node
	static NodeID ParentOf(const Node& node) noexcept { return node.head[node.parentSlot]; }
	#if TETRAFLOW_DEFICIT_BOUND
	// what an S node may still commit at bridges: a root's source excess not yet claimed by its deficit,
	// otherwise its cached path bottleneck estimate
	static Cap BudgetOf(const Node& node) noexcept { return node.excess > 0 ? node.excess - node.deficit : node.sBound; }
	#endif
	// record the parent of the node being attached through the tree arc of residual arcCap from the
	// parent (S with the bound: the path bottleneck estimate; otherwise the parent id)
	template <bool S>
	static void SetParent(Node& child, NodeID parent, const Node& parentNode, Cap arcCap) noexcept {
		#if TETRAFLOW_DEFICIT_BOUND
		if constexpr (S) {
			child.sBound = std::min(BudgetOf(parentNode), arcCap);
			static_cast<void>(parent);
			return;
		}
		#endif
		static_cast<void>(parentNode);
		static_cast<void>(arcCap);
		child.parent = parent;
	}

	template <bool S>
	std::vector<NodeID>& Frontier() noexcept { if constexpr (S) return frontierS; else return frontierT; }
	template <bool S>
	Label Top() const noexcept { return S ? levelS : levelT; }
	template <bool S>
	uint64_t& UniqOrphans() noexcept { if constexpr (S) return uniqOrphansS; else return uniqOrphansT; }
	#if TETRAFLOW_DEFICIT_BOUND
	template <bool S>
	std::vector<NodeID>& Cut() noexcept { if constexpr (S) return cutS; else return cutT; }
	#endif

	void ResetSolverState() noexcept {
		orphanHead = orphanTail = NO_NODE;
		bucketMaxLevel = 0;
		#if TETRAFLOW_DEFICITS
		for (std::vector<NodeID>& level: deficitQueue)
			level.clear();
		maxDeficitLevel = 0;
		pendingFree.clear();
		#if TETRAFLOW_DEFICIT_BOUND
		cutS.clear();
		cutT.clear();
		#endif
		#endif
		levelS = levelT = 1;
		uniqOrphansS = uniqOrphansT = 0;
		augTs = 0;
		flow = 0;
	}

	// fold the terminal capacities into the initial flow, create the tree roots, fill the unused slots
	// with zero-capacity self-arcs and compute the reverse-residual bits
	void Init() {
		frontierS.clear();
		frontierT.clear();
		scan.clear();
		bucketHead.assign(4, NO_NODE);
		ResetSolverState();
		const size_t numNodes = nodes.size();
		Node* const nodeData = NodeData();
		for (size_t x = 0; x < numNodes; ++x) {
			Node& node = nodeData[x];
			for (unsigned i = node.fill; i < NUM_SLOTS; ++i) {
				node.head[i] = NodeID(x);
				node.rcap[i] = 0;
				SetRev(node, i, i);
			}
			const Cap capSource = node.excess;
			const Cap capSink = node.capSink;
			flow += double(std::min(capSource, capSink));
			node.excess = capSource - capSink;
			node.prev = NO_NODE;
			node.next = NO_NODE;
			node.parent = NO_NODE;
			node.lastAugTs = 0;
			#if TETRAFLOW_DEFICITS
			node.deficit = 0;
			#endif
			node.parentSlot = NO_SLOT;
			node.isParentCurr = 0;
			node.children = 0;
			node.listState = LIST_NONE;
			if (node.excess > 0) {
				node.label = 1;
				frontierS.push_back(NodeID(x));
			} else if (node.excess < 0) {
				node.label = -1;
				frontierT.push_back(NodeID(x));
			} else {
				node.label = 0;
			}
			uint8_t revres = 0;
			for (unsigned i = 0; i < NUM_SLOTS; ++i)
				if (nodes[node.head[i]].rcap[Rev(node, i)] > 0)
					revres |= Bit(i);
			node.revres = revres;
		}
	}

	// grow tree S/T by one level: scan the nodes of the current deepest level, attach their free
	// neighbors as the new level and augment along every arc that reaches the other tree
	template <bool S>
	void Grow() {
		std::vector<NodeID>& frontier = Frontier<S>();
		scan.swap(frontier);
		frontier.clear();
		const Label top = Top<S>();
		if (bucketHead.size() <= size_t(top))
			bucketHead.resize(size_t(top) + 1, NO_NODE);
		if constexpr (S)
			TETRAFLOW_STAT(growthPassesS);
		else
			TETRAFLOW_STAT(growthPassesT);
		const size_t numScan = scan.size();
		for (size_t k = 0; k < numScan; ++k) {
			if (k + PREFETCH_DIST < numScan)
				TETRAFLOW_PREFETCH(&nodes[scan[k + PREFETCH_DIST]]);
			if (k + 1 < numScan) {
				const Node& nodeNext = nodes[scan[k + 1]];
				for (unsigned i = 0; i < NUM_SLOTS; ++i)
					TETRAFLOW_PREFETCH(&nodes[nodeNext.head[i]]);
			}
			ScanNode<S>(scan[k], top);
		}
		#if TETRAFLOW_DEFICIT_BOUND
		// nodes whose scan was cut short: settle the deficits (their estimates are refreshed, or they leave
		// the level) and scan them again, until every node of the level is fully scanned
		while (!Cut<S>().empty()) {
			ResolveDeficits();
			scan.swap(Cut<S>());
			Cut<S>().clear();
			for (NodeID x: scan)
				ScanNode<S>(x, top);
		}
		#endif
		#ifdef TETRAFLOW_CHECK_INVARIANTS
		CheckInvariants();
		#endif
	}

	// scan one node of level top-1 of tree S/T
	template <bool S>
	void ScanNode(NodeID x, Label top) {
		const Label labelScan = Sgn<S>(top - 1);
		Node& nx = nodes[x];
		if (nx.label != labelScan)
			return; // stale entry: x was relabeled or freed since it was appended to the frontier
		const Label labelChild = Sgn<S>(top);
		for (unsigned i = 0; i < NUM_SLOTS; ++i) {
			TETRAFLOW_STAT(arcScansGrowth);
			while (Forward<S>(nx, i)) {
				const NodeID y = nx.head[i];
				Node& ny = nodes[y];
				if (TETRAFLOW_LIKELY(ny.label == 0)) TETRAFLOW_LIKELY_BRANCH {
					#if TETRAFLOW_DEFICITS
					if constexpr (!S) {
						if (ny.deficit > 0) {
							ConvertToSink(y, ny); // a free node holding a deficit becomes a sink at this level
							break;
						}
					}
					#endif
					// free node: it becomes a child of x at the new level
					ny.label = labelChild;
					ny.parentSlot = Slot(Rev(nx, i));
					SetParent<S>(ny, x, nx, nx.rcap[i]);
					ny.isParentCurr = 0;
					nx.children |= Bit(i);
					Frontier<S>().push_back(y);
					#if TETRAFLOW_DEFICITS
					if constexpr (S) {
						if (ny.deficit > 0)
							DeficitAdd(y, ny.label); // the sweep pulls the deficit it carried while free
					}
					#endif
					break;
				}
				if (Sgn<S>(ny.label) > 0)
					break; // y is in the same tree
				// y is in the other tree: augment along source -> ... -> x -> y -> ... -> sink
				const bool exhausted = S ? Augment(x, i) : Augment(y, Rev(nx, i));
				#if TETRAFLOW_DEFICIT_BOUND
				if (exhausted) {
					// the S endpoint has used up its estimate: scan x again once the deficits of the pass are settled
					TETRAFLOW_STAT(boundExhaustions);
					Cut<S>().push_back(x);
					return;
				}
				#else
				static_cast<void>(exhausted);
				#endif
				if (nx.label != labelScan)
					return; // x left this level (it is in the frontier if it was relabeled to top)
				// re-examine the same slot: y may still be in the other tree, or be free now
			}
		}
	}

	// bottleneck of the augmenting path through the bridge x -> y (x in S, y in T) given the bridge
	// residual d: walk from x up to its S root (tree arc parent -> n) and from y down to its T root (tree
	// arc n -> parent); the two walks are independent dependent-load chains, so their steps are
	// interleaved to let the cache misses overlap; both paths are recorded (leaf first, root last) so
	// that the push walks iterate an array instead of re-chasing links
	Cap Bottleneck(NodeID x, NodeID y, Cap d) {
		const Node* ns = &nodes[x];
		const Node* nt = &nodes[y];
		Cap dS = d, dT = d;
		pathS.clear();
		pathT.clear();
		pathS.push_back(x);
		pathT.push_back(y);
		auto stepS = [&]() {
			TETRAFLOW_STAT(walkBottleneckS);
			const Node& q = nodes[ns->parent];
			dS = std::min(dS, q.rcap[Rev(*ns, ns->parentSlot)]);
			pathS.push_back(ns->parent);
			ns = &q;
		};
		auto stepT = [&]() {
			TETRAFLOW_STAT(walkBottleneckT);
			dT = std::min(dT, nt->rcap[nt->parentSlot]);
			pathT.push_back(nt->parent);
			nt = &nodes[nt->parent];
		};
		while (ns->excess == 0 && nt->excess == 0) {
			stepS();
			stepT();
		}
		while (ns->excess == 0)
			stepS();
		while (nt->excess == 0)
			stepT();
		dS = std::min(dS, ns->excess);  // residual source capacity of the S root
		dT = std::min(dT, -nt->excess); // residual sink capacity of the T root
		return std::min(dS, dT);
	}
	// bottleneck of the T side only (walk from y down to its T root, recording the path)
	Cap BottleneckT(NodeID y, Cap d) {
		const Node* nt = &nodes[y];
		pathT.clear();
		pathT.push_back(y);
		while (nt->excess == 0) {
			TETRAFLOW_STAT(walkBottleneckT);
			d = std::min(d, nt->rcap[nt->parentSlot]);
			pathT.push_back(nt->parent);
			nt = &nodes[nt->parent];
		}
		return std::min(d, -nt->excess); // residual sink capacity of the T root
	}

	// x in S, y = head[i] in T, rcap(x,i) > 0: push the bottleneck of the path source -> x -> y -> sink;
	// returns true if x has used up its bottleneck estimate (TETRAFLOW_DEFICIT_BOUND)
	bool Augment(NodeID x, unsigned i) {
		Node& nx = nodes[x];
		const NodeID y = nx.head[i];
		Node& ny = nodes[y];
		const unsigned j = Rev(nx, i);
		assert(nx.label > 0 && ny.label < 0 && nx.rcap[i] > 0);
		assert(orphanHead == NO_NODE);
		TETRAFLOW_STAT(augmentations);
		bool budgetExhausted = false;
		Cap d = nx.rcap[i];
		#if TETRAFLOW_DEFICITS
		d = BottleneckT(y, d);
		#if TETRAFLOW_DEFICIT_BOUND
		const Cap budget = BudgetOf(nx);
		if (budget > 0 && budget < d)
			d = budget; // commit no more than the S path is estimated to carry
		#endif
		#else
		d = Bottleneck(x, y, d);
		#endif
		assert(d > 0);
		// push d on the bridge
		nx.rcap[i] -= d;
		ny.rcap[j] += d;
		nx.revres |= Bit(i);
		if (nx.rcap[i] > 0)
			ny.revres |= Bit(j);
		else
			ny.revres &= uint8_t(~Bit(j));
		// push on the T path and adopt the T orphans
		++augTs;
		Push<false>(pathT, d);
		Adopt<false>();
		#if TETRAFLOW_DEFICITS
		// the S side is settled by the deficit sweep at the end of the growth pass: x records what it pushed
		if (nx.deficit == 0)
			DeficitAdd(x, nx.label);
		nx.deficit += d;
		#if TETRAFLOW_DEFICIT_BOUND
		if (nx.excess <= 0)
			nx.sBound -= d;
		budgetExhausted = BudgetOf(nx) <= 0;
		#endif
		#else
		// then the same for S
		++augTs;
		Push<true>(pathS, d);
		Adopt<true>();
		flow += double(d);
		#endif
		assert(orphanHead == NO_NODE);
		#ifdef TETRAFLOW_CHECK_INVARIANTS
		CheckInvariants();
		#endif
		return budgetExhausted; // true: the S endpoint has used up its bottleneck estimate
	}

	// push d along the recorded tree path (leaf first, root last); saturated tree arcs detach their
	// child, and a root that loses its terminal arc is detached too; the orphans are pushed to the
	// front of the orphan list so that the one nearest the root is processed first
	template <bool S>
	void Push(const std::vector<NodeID>& path, Cap d) {
		const size_t len = path.size();
		assert(len > 0);
		for (size_t k = 0; k + 1 < len; ++k)
			PushArc<S>(path[k], nodes[path[k]], nodes[path[k + 1]], d);
		PushRoot<S>(path[len - 1], nodes[path[len - 1]], d);
	}
	// push d along the tree arc between n and its parent q
	template <bool S>
	void PushArc(NodeID n, Node& pn, Node& nq, Cap d) {
		const unsigned p = pn.parentSlot;
		const unsigned r = Rev(pn, p);
		if constexpr (S) {
			TETRAFLOW_STAT(walkPushS);
			// tree arc q -> n is (q,r); n -> q gains residual
			pn.rcap[p] += d;
			nq.revres |= Bit(r);
			nq.rcap[r] -= d;
			if (nq.rcap[r] == 0) {
				pn.revres &= uint8_t(~Bit(p));
				nq.children &= uint8_t(~Bit(r));
				OrphanPushFront(n);
			}
		} else {
			TETRAFLOW_STAT(walkPushT);
			// tree arc n -> q is (n,p); q -> n gains residual
			nq.rcap[r] += d;
			pn.revres |= Bit(p);
			pn.rcap[p] -= d;
			if (pn.rcap[p] == 0) {
				nq.revres &= uint8_t(~Bit(r));
				nq.children &= uint8_t(~Bit(r));
				OrphanPushFront(n);
			}
		}
	}
	// push d through the terminal arc of the root n
	template <bool S>
	void PushRoot(NodeID n, Node& pn, Cap d) {
		if constexpr (S)
			pn.excess -= d;
		else
			pn.excess += d;
		if (pn.excess == 0)
			OrphanPushFront(n); // the root lost its terminal arc
	}

	// process the orphan list of tree S/T: re-attach every orphan to a parent of the same level if
	// possible, otherwise orphan its children and relabel it deeper (directly, or through the buckets
	// of the three-pass procedure when the same nodes keep getting orphaned) or free it
	template <bool S>
	void Adopt() {
		const Label top = Top<S>();
		bool threePass = false;
		size_t numOrphans = 0, numUniq = 0;
		while (orphanHead != NO_NODE) {
			const NodeID x = OrphanPopFront();
			if (orphanHead != NO_NODE)
				TETRAFLOW_PREFETCH(&nodes[orphanHead]);
			Node& nx = nodes[x];
			assert(nx.excess == 0);
			TETRAFLOW_STAT(orphans);
			++numOrphans;
			if (nx.lastAugTs != AugTs(augTs)) {
				nx.lastAugTs = AugTs(augTs);
				++UniqOrphans<S>();
				++numUniq;
			}
			if (numOrphans >= 3*numUniq)
				threePass = true;
			const Label lx = Sgn<S>(nx.label);
			assert(lx > 0 && lx <= top);
			#if TETRAFLOW_DEFICITS
			assert(lx == 1 || nx.parentSlot != NO_SLOT || !S); // converted T roots can be at any level
			#else
			assert(lx == 1 || nx.parentSlot != NO_SLOT);
			#endif
			// (a) look for a parent at the same level as before, starting from the current arc
			const unsigned start = nx.isParentCurr ? nx.parentSlot : 0;
			nx.isParentCurr = 1;
			nx.parentSlot = NO_SLOT;
			if (lx != 1) { // roots have no possible parent
				const Label want = Sgn<S>(lx - 1);
				for (unsigned i = start; i < NUM_SLOTS; ++i) {
					TETRAFLOW_STAT(arcScansAdoption);
					if (!Backward<S>(nx, i))
						continue;
					const NodeID y = nx.head[i];
					Node& ny = nodes[y];
					if (ny.label == want) {
						nx.parentSlot = Slot(i);
						SetParent<S>(nx, y, ny, ny.rcap[Rev(nx, i)]);
						ny.children |= Bit(Rev(nx, i));
						Attached<S>(x, nx);
						break;
					}
				}
				if (nx.parentSlot != NO_SLOT)
					continue; // adopted, label unchanged
			}
			// (b) no parent at that level: the children become orphans
			for (unsigned mask = nx.children; mask != 0; mask &= mask - 1)
				OrphanPushBack(nx.head[LowestBit(mask)]);
			nx.children = 0;
			// (c) a node of the deepest level is dropped: growth re-discovers it if it is reachable
			if (lx == top) {
				Release<S>(x, nx);
				continue;
			}
			// (d) bucket mode: the relabeling is deferred to the three-pass procedure
			if (threePass) {
				TETRAFLOW_STAT(orphansBucket);
				nx.label = Sgn<S>(lx + 1);
				BucketAdd(x);
				continue;
			}
			// (e) relabel: attach to the candidate parent with the lowest level
			Label best = top;
			unsigned bestSlot = NO_SLOT;
			for (unsigned i = 0; i < NUM_SLOTS; ++i) {
				TETRAFLOW_STAT(arcScansAdoption);
				if (!Backward<S>(nx, i))
					continue;
				const Label ly = Sgn<S>(nodes[nx.head[i]].label);
				if (ly > 0 && ly < best) {
					best = ly;
					bestSlot = i;
					if (best == lx)
						break; // cannot do better than lx+1: labels are BFS distances
				}
			}
			if (bestSlot != NO_SLOT) {
				const NodeID y = nx.head[bestSlot];
				nx.label = Sgn<S>(best + 1);
				nx.parentSlot = Slot(bestSlot);
				SetParent<S>(nx, y, nodes[y], nodes[y].rcap[Rev(nx, bestSlot)]);
				nodes[y].children |= Bit(Rev(nx, bestSlot));
				Attached<S>(x, nx);
				if (best + 1 == top)
					Frontier<S>().push_back(x); // it must be scanned in the next growth pass
			} else {
				Release<S>(x, nx); // no way back into the tree
			}
		}
		if (threePass)
			ThreePass<S>();
	}

	// relabel the pending orphans stored in the buckets level by level: pass 2 attaches each of them
	// to its lowest attached candidate parent (or moves it to a deeper bucket, or frees it), pass 3
	// pulls its free or deeper-pending neighbors down to the level below it
	template <bool S>
	void ThreePass() {
		const Label top = Top<S>();
		for (Label L = 2; L <= bucketMaxLevel; ++L) { // bucketMaxLevel may grow while iterating
			for (NodeID x; (x = BucketPopFront(L)) != NO_NODE; ) {
				Node& nx = nodes[x];
				assert(Sgn<S>(nx.label) == L);
				if (nx.parentSlot == NO_SLOT) {
					// pass 2: the candidate parent with the lowest level among the attached nodes
					Label best = top;
					unsigned bestSlot = NO_SLOT;
					const Label dest = L - 1;
					for (unsigned i = 0; i < NUM_SLOTS; ++i) {
						TETRAFLOW_STAT(arcScansAdoption);
						if (!Backward<S>(nx, i))
							continue;
						const Node& ny = nodes[nx.head[i]];
						const Label ly = Sgn<S>(ny.label);
						if (ly > 0 && ly < best && IsAttached(ny)) {
							best = ly;
							bestSlot = i;
							if (best == dest)
								break;
						}
					}
					if (bestSlot == NO_SLOT) {
						Release<S>(x, nx); // free (or, holding a deficit, converted into a T root)
						continue;
					}
					nx.parentSlot = Slot(bestSlot);
					SetParent<S>(nx, nx.head[bestSlot], nodes[nx.head[bestSlot]], nodes[nx.head[bestSlot]].rcap[Rev(nx, bestSlot)]);
					nx.label = Sgn<S>(best + 1);
					if (best + 1 != L) {
						assert(best + 1 > L);
						BucketAdd(x); // moved to a deeper bucket, processed later
						continue;
					}
				}
				// pass 3: pull the free or deeper-pending neighbors down to level L+1
				if (L != top) {
					const Label childLabel = L + 1;
					for (unsigned i = 0; i < NUM_SLOTS; ++i) {
						TETRAFLOW_STAT(arcScansAdoption);
						if (!Forward<S>(nx, i))
							continue;
						const NodeID y = nx.head[i];
						Node& ny = nodes[y];
						const Label ly = Sgn<S>(ny.label);
						if (ly == 0 || ly > childLabel) {
							#if TETRAFLOW_DEFICITS
							if constexpr (!S) {
								if (ly == 0 && ny.deficit > 0) {
									ConvertToSink(y, ny); // a free node holding a deficit becomes a sink at the top level
									continue;
								}
							}
							#endif
							// y is free, or a pending orphan in a deeper bucket (an attached node can never be deeper than L+1)
							if (ly != 0)
								BucketRemove(y);
							ny.label = Sgn<S>(childLabel);
							ny.parentSlot = Slot(Rev(nx, i));
							SetParent<S>(ny, x, nx, nx.rcap[i]);
							BucketAdd(y);
						}
					}
				}
				// attach x to its parent
				nodes[ParentOf(nx)].children |= Bit(Rev(nx, nx.parentSlot));
				nx.isParentCurr = 0;
				Attached<S>(x, nx);
				if (L == top)
					Frontier<S>().push_back(x);
			}
		}
		bucketMaxLevel = 0;
	}

	// hook: the node x of tree S/T has just been attached to a parent
	template <bool S>
	void Attached(NodeID x, Node& nx) {
		#if TETRAFLOW_DEFICITS
		if constexpr (S) {
			if (nx.deficit > 0)
				DeficitAdd(x, nx.label); // the sweep must pull the deficit through the new arc
			return;
		}
		#endif
		static_cast<void>(x);
		static_cast<void>(nx);
	}
	// hook: the node x of tree S/T has no way back into its tree: it becomes free; an S node holding a
	// deficit keeps it while free (until S re-adopts it or T reaches it) and is remembered
	template <bool S>
	void Release(NodeID x, Node& nx) {
		nx.label = 0;
		#if TETRAFLOW_DEFICITS
		if constexpr (S) {
			if (nx.deficit > 0) {
				TETRAFLOW_STAT(deficitFrees);
				pendingFree.push_back(x);
			}
		}
		#endif
		static_cast<void>(x);
	}

	#if TETRAFLOW_DEFICITS
	// a free node holding a deficit that T has reached (or that is left when T is exhausted) becomes a
	// T root at T's current top level: the flow it pushed is still on the arcs, so its deficit is exactly
	// the sink capacity that any S node reaching it later can fill (counted at the source root as usual)
	void ConvertToSink(NodeID x, Node& nx) {
		assert(nx.label == 0 && nx.deficit > 0 && nx.excess == 0 && nx.parentSlot == NO_SLOT && nx.children == 0);
		TETRAFLOW_STAT(conversions);
		TETRAFLOW_STAT_ADD(phantomFlow, double(nx.deficit));
		nx.excess = -nx.deficit;
		nx.deficit = 0;
		nx.label = -levelT;
		nx.isParentCurr = 0;
		frontierT.push_back(x);
	}
	// T is exhausted: every free node still holding a deficit becomes a sink at T's top level
	void ConvertPendingFree() {
		for (NodeID x: pendingFree) {
			Node& nx = nodes[x];
			if (nx.label == 0 && nx.deficit > 0)
				ConvertToSink(x, nx);
		}
		pendingFree.clear();
		#if TETRAFLOW_DEFICIT_BOUND
		cutS.clear();
		cutT.clear();
		#endif
	}
	// queue x (an S node holding a deficit) for the sweep at level L
	void DeficitAdd(NodeID x, Label L) {
		assert(L > 0);
		if (size_t(L) >= deficitQueue.size())
			deficitQueue.resize(size_t(L) + 1);
		deficitQueue[L].push_back(x);
		if (L > maxDeficitLevel)
			maxDeficitLevel = L;
	}
	// settle the deficits recorded by the augmentations of the last growth pass: deepest level first,
	// each deficit is pulled through the tree arc from the parent (which inherits it) until a root
	// consumes it from its source capacity; a saturated tree arc or an exhausted root orphans the node,
	// and the usual adoption re-attaches it (re-queueing its leftover deficit) or frees it (the deficit
	// then waits on the free node); each tree arc is traversed once per sweep however many
	// augmentations used it
	void ResolveDeficits() {
		for (;;) {
			while (maxDeficitLevel > 0 && deficitQueue[maxDeficitLevel].empty())
				--maxDeficitLevel;
			if (maxDeficitLevel == 0)
				break;
			const Label L = maxDeficitLevel;
			const NodeID x = deficitQueue[L].back();
			deficitQueue[L].pop_back();
			Node& nx = nodes[x];
			if (nx.deficit == 0 || nx.label <= 0)
				continue; // stale entry: resolved, freed or converted meanwhile
			if (nx.label != L) {
				DeficitAdd(x, nx.label); // relabeled meanwhile: process it at its real level
				continue;
			}
			TETRAFLOW_STAT(deficitPulls);
			if (nx.parentSlot == NO_SLOT) {
				// an S root: consume source capacity
				assert(nx.excess > 0);
				const Cap d = std::min(nx.deficit, nx.excess);
				nx.excess -= d;
				nx.deficit -= d;
				flow += double(d);
				if (nx.excess == 0) {
					// terminal arc saturated: x is an orphan (adoption re-queues or converts a leftover deficit)
					TETRAFLOW_STAT(deficitOrphans);
					OrphanPushFront(x);
					++augTs;
					Adopt<true>();
				}
				continue;
			}
			// pull through the tree arc q -> x = (q,r); x -> q gains residual
			const unsigned p = nx.parentSlot;
			const NodeID q = ParentOf(nx);
			Node& nq = nodes[q];
			const unsigned r = Rev(nx, p);
			assert(q != x && nq.label == nx.label - 1 && nq.head[r] == x);
			const Cap d = std::min(nx.deficit, nq.rcap[r]);
			assert(d > 0);
			nq.rcap[r] -= d;
			nx.rcap[p] += d;
			nq.revres |= Bit(r);
			nx.deficit -= d;
			if (nq.deficit == 0)
				DeficitAdd(q, nq.label);
			nq.deficit += d;
			#if TETRAFLOW_DEFICIT_BOUND
			nx.sBound = std::min(BudgetOf(nq), nq.rcap[r]); // refresh the estimate through the pulled arc
			#endif
			if (nq.rcap[r] == 0) {
				// tree arc saturated: x is an orphan
				TETRAFLOW_STAT(deficitOrphans);
				nx.revres &= uint8_t(~Bit(p));
				nq.children &= uint8_t(~Bit(r));
				OrphanPushFront(x);
				++augTs;
				Adopt<true>();
			}
		}
		#ifndef NDEBUG
		for (const Node& node: nodes)
			assert(node.label <= 0 || node.deficit == 0); // every tree deficit was drained (free nodes may keep one)
		#endif
	}
	#endif

	// orphan list: intrusive singly-linked through next, with head and tail
	void OrphanPushFront(NodeID x) noexcept {
		Node& node = nodes[x];
		assert(node.listState == LIST_NONE);
		TETRAFLOW_DEBUG(node.listState = LIST_ORPHAN);
		node.next = orphanHead;
		orphanHead = x;
		if (orphanTail == NO_NODE)
			orphanTail = x;
	}
	void OrphanPushBack(NodeID x) noexcept {
		Node& node = nodes[x];
		assert(node.listState == LIST_NONE);
		TETRAFLOW_DEBUG(node.listState = LIST_ORPHAN);
		node.next = NO_NODE;
		if (orphanTail == NO_NODE)
			orphanHead = x;
		else
			nodes[orphanTail].next = x;
		orphanTail = x;
	}
	NodeID OrphanPopFront() noexcept {
		const NodeID x = orphanHead;
		Node& node = nodes[x];
		assert(node.listState == LIST_ORPHAN);
		TETRAFLOW_DEBUG(node.listState = LIST_NONE);
		orphanHead = node.next;
		if (orphanHead == NO_NODE)
			orphanTail = NO_NODE;
		return x;
	}

	// buckets: one intrusive doubly-linked list per absolute label, linked through next/prev
	void BucketAdd(NodeID x) noexcept {
		Node& node = nodes[x];
		assert(node.listState == LIST_NONE);
		TETRAFLOW_DEBUG(node.listState = LIST_BUCKET);
		const Label L = std::abs(node.label);
		assert(L >= 2 && size_t(L) < bucketHead.size());
		const NodeID head = bucketHead[L];
		node.next = head;
		node.prev = NO_NODE;
		if (head != NO_NODE)
			nodes[head].prev = x;
		bucketHead[L] = x;
		if (L > bucketMaxLevel)
			bucketMaxLevel = L;
	}
	void BucketRemove(NodeID x) noexcept {
		Node& node = nodes[x];
		assert(node.listState == LIST_BUCKET);
		TETRAFLOW_DEBUG(node.listState = LIST_NONE);
		const Label L = std::abs(node.label);
		assert(L >= 2 && size_t(L) < bucketHead.size());
		if (node.prev == NO_NODE) {
			assert(bucketHead[L] == x);
			bucketHead[L] = node.next;
		} else {
			nodes[node.prev].next = node.next;
		}
		if (node.next != NO_NODE)
			nodes[node.next].prev = node.prev;
	}
	NodeID BucketPopFront(Label L) noexcept {
		const NodeID x = bucketHead[L];
		if (x != NO_NODE) {
			Node& node = nodes[x];
			assert(node.listState == LIST_BUCKET);
			TETRAFLOW_DEBUG(node.listState = LIST_NONE);
			bucketHead[L] = node.next;
			if (node.next != NO_NODE)
				nodes[node.next].prev = NO_NODE;
		}
		return x;
	}

	#ifdef TETRAFLOW_CHECK_INVARIANTS
	// exhaustive O(N) consistency check of the tree state; valid between augmentations
	void CheckInvariants() const {
		assert(orphanHead == NO_NODE && orphanTail == NO_NODE);
		assert(bucketMaxLevel == 0);
		for (NodeID h: bucketHead)
			assert(h == NO_NODE);
		for (size_t x = 0; x < nodes.size(); ++x) {
			const Node& nx = nodes[x];
			assert(nx.listState == LIST_NONE);
			for (unsigned i = 0; i < NUM_SLOTS; ++i) {
				const Node& ny = nodes[nx.head[i]];
				assert(nx.rcap[i] >= 0);
				assert(ny.head[Rev(nx, i)] == x && Rev(ny, Rev(nx, i)) == i);
				assert(((nx.revres >> i) & 1u) == (ny.rcap[Rev(nx, i)] > 0 ? 1u : 0u));
			}
			if (nx.label == 0) {
				assert(nx.excess == 0 && nx.parentSlot == NO_SLOT && nx.children == 0);
				continue;
			}
			const bool inS = nx.label > 0;
			const Label lx = inS ? nx.label : -nx.label;
			#if TETRAFLOW_DEFICITS
			assert(inS || nx.deficit == 0); // only S nodes carry deficits
			#endif
			assert(lx <= (inS ? levelS : levelT));
			if (nx.excess != 0) {
				#if TETRAFLOW_DEFICITS
				assert((nx.excess > 0) == inS && (lx == 1 || !inS) && nx.parentSlot == NO_SLOT); // converted T roots sit at any level
				#else
				assert((nx.excess > 0) == inS && lx == 1 && nx.parentSlot == NO_SLOT);
				#endif
			} else {
				assert(nx.parentSlot < NUM_SLOTS);
				#if TETRAFLOW_DEFICIT_BOUND
				assert(inS || nx.parent == nx.head[nx.parentSlot]); // an S node's slot holds the bottleneck estimate
				#else
				assert(nx.parent == nx.head[nx.parentSlot]);
				#endif
				const Node& np = nodes[ParentOf(nx)];
				assert(np.label == (inS ? nx.label - 1 : nx.label + 1));
				assert((np.children >> Rev(nx, nx.parentSlot)) & 1u);
				assert(inS ? np.rcap[Rev(nx, nx.parentSlot)] > 0 : nx.rcap[nx.parentSlot] > 0);
			}
			for (unsigned i = 0; i < NUM_SLOTS; ++i) {
				const Node& ny = nodes[nx.head[i]];
				if (nx.children & Bit(i))
					assert(ParentOf(ny) == x && ny.parentSlot == Rev(nx, i) && ny.label == (inS ? nx.label + 1 : nx.label - 1));
				// no shortcut: a residual arc away from the root cannot skip a level within a tree
				const bool forward = inS ? nx.rcap[i] > 0 : ((nx.revres >> i) & 1u) != 0;
				if (forward && (ny.label > 0) == inS && ny.label != 0)
					assert((inS ? ny.label : -ny.label) <= lx + 1);
				// scanned nodes have no residual arc towards a free node
				if (forward && ny.label == 0)
					assert(lx + 1 >= (inS ? levelS : levelT));
			}
		}
	}
	#endif

private:
	std::vector<Node> nodes;
	std::vector<NodeID> frontierS;  // nodes of the deepest S level, to be scanned by the next S growth
	std::vector<NodeID> frontierT;  // same for T
	std::vector<NodeID> scan;       // the level being scanned by the current growth pass
	std::vector<NodeID> bucketHead; // head of the bucket of each absolute label (three-pass adoption)
	std::vector<NodeID> pathS, pathT; // the augmenting path recorded by the bottleneck walks
	#if TETRAFLOW_DEFICITS
	std::vector<std::vector<NodeID>> deficitQueue; // S nodes holding a deficit, by level (may hold stale entries)
	Label maxDeficitLevel = 0;                     // deepest possibly non-empty level of deficitQueue
	std::vector<NodeID> pendingFree;               // free nodes holding a deficit (may hold stale entries)
	#if TETRAFLOW_DEFICIT_BOUND
	std::vector<NodeID> cutS, cutT;                // nodes whose scan was cut short by their budget (scanned again after settling)
	#endif
	#endif
	NodeID orphanHead = NO_NODE, orphanTail = NO_NODE;
	Label bucketMaxLevel = 0;       // deepest non-empty bucket
	Label levelS = 1, levelT = 1;   // depth of the deepest level created so far in each tree
	uint64_t uniqOrphansS = 0, uniqOrphansT = 0; // unique orphans produced by each tree (growth heuristic)
	uint32_t augTs = 0;             // push-phase timestamp (incremented twice per augmentation)
	double flow = 0;
	Stats stats;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // _MATH_TETRAFLOW_H_
