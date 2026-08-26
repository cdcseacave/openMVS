/*
#########################################################
#                                                       #
#  GraphCut4 - Maximum s-t Flow / Minimum s-t Cut       #
#              specialized for graphs of degree <= 4    #
#              (e.g. the dual graph of a tetrahedral    #
#              mesh, where every cell has exactly 4     #
#              facet neighbors)                         #
#                                                       #
#  2026 - cDc@seacave                                   #
#                                                       #
#########################################################

Implements the IBFS (Incremental Breadth First Search) maximum flow algorithm from
	"Maximum flows by incremental breadth-first search"
	Andrew V. Goldberg, Sagi Hed, Haim Kaplan, Robert E. Tarjan, and Renato F. Werneck.
	In Proceedings of the 19th European conference on Algorithms, ESA'11, pages 457-468.
	2011
re-housed on a compact fixed-degree data structure. Derived from the reference IBFS
implementation by Haim Kaplan and Sagi Hed (http://www.cs.tau.ac.il/~sagihed/ibfs/),
which can be used for research purposes only; the same restriction applies to this file.
Cite the aforementioned paper in any resulting publication.

Key differences to the reference implementation, exploiting the fixed degree <= 4:
 - one 64-byte cache-line block per node holds everything: the 4 arc capacities, the 4
   neighbor ids, and the BFS state (excess, label, list links, parent slot, current-arc
   flag, reverse-residual mask, reverse-slot table, tree-son mask, timestamp);
 - no Arc structs, no firstArc pointers, no arc iteration bounds: arc s of node n is slot s;
 - no reverse-arc pointers: the reverse of arc (n,s) is (head[s], revSlot[s]);
 - no firstSon/sibling lists: a node has at most one tree son per arc, so the son list is
   a 4-bit mask, making orphan removal a single bit-clear;
 - 32-bit indices everywhere instead of 64-bit pointers;
 - the parent walk (the hot loop of every augmentation) is the same two dependent loads as
   the reference implementation: parent slot, then neighbor id.
Net: 64 bytes per node plus 12 bytes for the three active lists, versus ~200 bytes per node
for the reference implementation, with identical algorithmics.
Limits: max 2^31-1 nodes, at most 4 incident edges per node.
*/

#ifndef _GRAPHCUT4_H__
#define _GRAPHCUT4_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef _MSC_VER
#include <intrin.h>
#include <malloc.h>
#endif

namespace GC4 {

typedef float EdgeCap;

// operation counters (augmentations, pushes, orphans, scanned arcs); off by default
#ifndef GC4_STATS
#define GC4_STATS 0
#endif
// software prefetch of the neighbor nodes and of the upcoming active/orphan nodes
#ifndef GC4_PREFETCH
#define GC4_PREFETCH 1
#endif
#ifdef _MSC_VER
#define GC4_PF(p) _mm_prefetch((const char*)(p), _MM_HINT_T0)
#else
#define GC4_PF(p) __builtin_prefetch(p)
#endif
// keep the phases as separate functions (profiling only)
#if defined(GC4_NOINLINE) && !defined(_MSC_VER)
#define GC4_NI __attribute__((noinline))
#else
#define GC4_NI
#endif

class Graph
{
public:
	typedef uint32_t NodeID;

	Graph() {
		memset(this, 0, sizeof(Graph));
	}
	~Graph() {
		AlignedFree(nodes);
		free(active0.list); free(activeS1.list); free(activeT1.list);
		free(buckets);
	}

	// numEdges is accepted for interface compatibility and ignored:
	// the fixed-degree layout always holds 4 arc slots per node
	void initSize(size_t nNodes, size_t /*numEdges*/=0) {
		assert(nNodes < (size_t)0x7FFFFFFF);
		numNodes = (uint32_t)nNodes;
		freeLabel = (int32_t)nNodes;
		nodes = (Node*)AllocZero(sizeof(Node)*numNodes);
		active0.init(numNodes);
		activeS1.init(numNodes);
		activeT1.init(numNodes);
		flow = 0;
	}

	// capacities towards the terminals; can be called repeatedly per node (accumulates)
	inline void addNode(NodeID n, EdgeCap capSource, EdgeCap capSink) {
		assert(n < numNodes);
		const EdgeCap f(nodes[n].excess);
		if (f > 0)
			capSource += f;
		else
			capSink -= f;
		flow += (capSource < capSink ? capSource : capSink);
		nodes[n].excess = capSource - capSink;
	}

	// one edge = two directed arcs; each endpoint must have a free slot (degree <= 4)
	inline void addEdge(NodeID u, NodeID v, EdgeCap cap, EdgeCap revCap) {
		assert(u < numNodes && v < numNodes && u != v);
		Node& nu = nodes[u];
		Node& nv = nodes[v];
		const unsigned su(nu.fill++);
		const unsigned sv(nv.fill++);
		assert(su < 4 && sv < 4);
		nu.rCap[su] = cap;
		nu.head[su] = v;
		nu.revSlot |= (uint8_t)(sv << (2*su));
		nv.rCap[sv] = revCap;
		nv.head[sv] = u;
		nv.revSlot |= (uint8_t)(su << (2*sv));
	}

	// finalize construction: compute the reverse-residual masks and seed the trees
	void initGraph() {
		for (uint32_t i=0; i<numNodes; ++i) {
			Node& x = nodes[i];
			uint8_t revRes(0);
			for (unsigned s=0; s<x.fill; ++s)
				if (nodes[x.head[s]].rCap[revSlotOf(x, s)] != 0)
					revRes |= (uint8_t)(1u << s);
			x.revRes = revRes;
			x.parent = PARENT_NONE;
			x.curr = 0;
			x.sons = 0;
			x.lastAugTs = 0;
			if (x.excess == 0) {
				x.label = freeLabel;
				continue;
			}
			if (x.excess > 0) {
				x.label = 1;
				activeS1.add(i);
			} else {
				x.label = -1;
				activeT1.add(i);
			}
		}
	}

	EdgeCap computeMaxFlow() {
		orphanFirst = END;
		topLevelS = topLevelT = 1;
		bool dirS(true);
		ActiveList::swap(active0, activeS1);
		while (true) {
			// BFS level
			if (dirS) {
				++topLevelS;
				growth<true>();
			} else {
				++topLevelT;
				growth<false>();
			}
			if (activeS1.len == 0 || activeT1.len == 0)
				break;
			// grow the tree that generated fewer unique orphans (IB_ALTERNATE_SMART)
			if ((uniqOrphansT == uniqOrphansS && dirS) || uniqOrphansT < uniqOrphansS) {
				ActiveList::swap(active0, activeT1);
				dirS = false;
			} else {
				ActiveList::swap(active0, activeS1);
				dirS = true;
			}
		}
		return (EdgeCap)flow;
	}

	inline EdgeCap getFlow() const { return (EdgeCap)flow; }
	inline size_t getNumNodes() const { return numNodes; }

	struct Stats { uint64_t augs, pushes, orphans, growthArcs; };
	inline const Stats& getStats() const { return stats; }

	inline bool isNodeOnSrcSide(NodeID n) const {
		const int32_t l(nodes[n].label);
		if (l == freeLabel || l == 0)
			return activeT1.len == 0;
		return l > 0;
	}

	// bytes allocated by the solver (excluding transient checkMaxFlow buffers)
	size_t getMemoryBytes() const {
		return (sizeof(Node) + sizeof(uint32_t)*3)*(size_t)numNodes + sizeof(uint32_t)*(size_t)bucketCap;
	}

	// optimality certificate: after computeMaxFlow no node with remaining sink
	// capacity may be reachable from a node with remaining source capacity
	// through positive-residual arcs; O(N) time, transient N bytes
	bool checkMaxFlow() const {
		uint8_t* seen((uint8_t*)calloc(numNodes, 1));
		uint32_t* queue((uint32_t*)malloc(sizeof(uint32_t)*numNodes));
		size_t qLen(0);
		bool valid(true);
		for (uint32_t i=0; i<numNodes; ++i) {
			if (nodes[i].excess > 0) {
				seen[i] = 1;
				queue[qLen++] = i;
			}
		}
		while (qLen > 0) {
			const uint32_t xi(queue[--qLen]);
			const Node& x = nodes[xi];
			if (x.excess < 0) {
				valid = false;
				break;
			}
			for (int s=0; s<4; ++s) {
				if (x.rCap[s] == 0)
					continue;
				const uint32_t yi(x.head[s]);
				if (!seen[yi]) {
					seen[yi] = 1;
					queue[qLen++] = yi;
				}
			}
		}
		free(seen);
		free(queue);
		return valid;
	}

private:
	static constexpr uint32_t END = 0xFFFFFFFFu; // list terminator / empty marker
	static constexpr uint8_t PARENT_NONE = 7;

	struct Node { // one cache line: the 4 arcs and the whole BFS state of a node
		EdgeCap rCap[4];     // residual capacity per arc slot
		uint32_t head[4];    // head node id per arc slot
		EdgeCap excess;      // > 0: capacity from s, < 0: -capacity to t
		int32_t label;       // > 0: distance from s, < 0: -distance from t, == numNodes: free
		uint32_t next;       // orphan-list / orphan-bucket forward link (END terminated)
		uint32_t prev;       // orphan-bucket back link
		uint8_t parent;      // arc slot towards the tree parent, PARENT_NONE if none
		uint8_t curr;        // the parent slot is a valid current-arc position
		uint8_t revRes;      // bit s: the reverse of arc s has residual capacity
		uint8_t revSlot;     // bits 2s..2s+1: slot of the reverse of arc s in its head
		uint8_t sons;        // bit s: the neighbor over arc s is a tree son
		uint8_t fill;        // construction: number of filled arc slots
		uint16_t lastAugTs;  // timestamp of the last augmentation this node was orphaned in
		uint8_t pad[8];
	};
	static_assert(sizeof(Node) == 64, "Node must pack to one 64-byte cache line");

	// 64-byte aligned zeroed allocation, so that every node block is exactly one cache line
	static void* AllocZero(size_t bytes) {
		bytes = (bytes + 63) & ~(size_t)63;
		#ifdef _MSC_VER
		void* p = _aligned_malloc(bytes, 64);
		#else
		void* p = nullptr;
		if (posix_memalign(&p, 64, bytes) != 0)
			p = nullptr;
		#endif
		if (p) memset(p, 0, bytes);
		return p;
	}
	static void AlignedFree(void* p) {
		#ifdef _MSC_VER
		_aligned_free(p);
		#else
		free(p);
		#endif
	}

	struct ActiveList {
		uint32_t* list;
		uint32_t len;
		inline void init(uint32_t numNodes) {
			list = (uint32_t*)malloc(sizeof(uint32_t)*numNodes);
			len = 0;
		}
		inline void add(uint32_t x) { list[len++] = x; }
		inline void clear() { len = 0; }
		inline static void swap(ActiveList& a, ActiveList& b) {
			const ActiveList tmp(a); a = b; b = tmp;
		}
	};

	static inline int CTZ(unsigned m) {
		#ifdef _MSC_VER
		unsigned long i; _BitScanForward(&i, m); return (int)i;
		#else
		return __builtin_ctz(m);
		#endif
	}
	static inline int revSlotOf(const Node& n, int s) {
		return (n.revSlot >> (2*s)) & 3;
	}

	// members
	Node* nodes;
	uint32_t* buckets;     // orphan-bucket heads, indexed by |label|; grown on demand
	int32_t bucketCap;
	int32_t maxBucket;
	uint32_t numNodes;
	int32_t freeLabel;     // == numNodes: the label of nodes in neither tree
	double flow;           // accumulated in double: float32 loses small augmentations once large
	uint16_t augTimestamp;
	uint32_t uniqOrphansS, uniqOrphansT;
	uint32_t orphanFirst, orphanLast;
	int32_t topLevelS, topLevelT;
	ActiveList active0, activeS1, activeT1;
	Stats stats;

	// orphan FIFO/LIFO list (matches the reference ADD_ORPHAN_BACK/FRONT macros)
	inline void addOrphanBack(uint32_t n) {
		if (orphanFirst != END)
			nodes[orphanLast].next = n;
		else
			orphanFirst = n;
		orphanLast = n;
		nodes[n].next = END;
	}
	inline void addOrphanFront(uint32_t n) {
		if (orphanFirst == END) {
			nodes[n].next = END;
			orphanLast = orphanFirst = n;
		} else {
			nodes[n].next = orphanFirst;
			orphanFirst = n;
		}
	}

	// orphan buckets by |label|, intrusive doubly-linked through next/prev
	inline void bucketEnsure(int32_t b) {
		if (b >= bucketCap) {
			const int32_t newCap(b + 64 > bucketCap*2 ? b + 64 : bucketCap*2);
			buckets = (uint32_t*)realloc(buckets, sizeof(uint32_t)*newCap);
			for (int32_t i=bucketCap; i<newCap; ++i)
				buckets[i] = END;
			bucketCap = newCap;
		}
	}
	template <bool sTree>
	inline void bucketAdd(uint32_t xi) {
		Node& x = nodes[xi];
		const int32_t b(sTree ? x.label : -x.label);
		bucketEnsure(b);
		const uint32_t head(buckets[b]);
		if (head == END) {
			x.next = END;
		} else {
			x.next = head;
			nodes[head].prev = xi;
		}
		buckets[b] = xi;
		if (b > maxBucket)
			maxBucket = b;
	}
	inline uint32_t bucketPopFront(int32_t b) {
		const uint32_t x(buckets[b]);
		if (x != END)
			buckets[b] = nodes[x].next;
		return x;
	}
	template <bool sTree>
	inline void bucketRemove(uint32_t xi) {
		const Node& x = nodes[xi];
		const int32_t b(sTree ? x.label : -x.label);
		assert(b < bucketCap);
		if (buckets[b] == xi) {
			buckets[b] = x.next;
		} else {
			nodes[x.prev].next = x.next;
			if (x.next != END)
				nodes[x.next].prev = x.prev;
		}
	}

	template <bool sTree>
	GC4_NI void augmentTree(uint32_t xi, EdgeCap bottleneck) {
		for (;;) {
			Node& x = nodes[xi];
			if (x.excess != 0) {
				x.excess += (sTree ? -bottleneck : bottleneck);
				if (x.excess == 0)
					addOrphanFront(xi);
				return;
			}
			const int p(x.parent);
			const uint32_t par(x.head[p]);
			const int prs(revSlotOf(x, p));
			Node& y = nodes[par];
			if (sTree) {
				x.rCap[p] += bottleneck;
				y.revRes |= (uint8_t)(1u << prs); // the reverse arc gained residual
				y.rCap[prs] -= bottleneck;
			} else {
				y.rCap[prs] += bottleneck;
				x.revRes |= (uint8_t)(1u << p);
				x.rCap[p] -= bottleneck;
			}
			// saturated?
			if ((sTree ? y.rCap[prs] : x.rCap[p]) == 0) {
				if (sTree)
					x.revRes &= (uint8_t)~(1u << p);
				else
					y.revRes &= (uint8_t)~(1u << prs);
				y.sons &= (uint8_t)~(1u << prs); // unlink x from its parent
				addOrphanFront(xi);
			}
			xi = par;
		}
	}

	// bridge arc (bu -> slot bs), oriented from the S tree into the T tree
	GC4_NI void augment(uint32_t bu, int bs) {
		if (GC4_STATS) ++stats.augs;
		Node& nbu = nodes[bu];
		const uint32_t bv(nbu.head[bs]);
		const int brs(revSlotOf(nbu, bs));
		// bottleneck in S
		EdgeCap bottleneck(nbu.rCap[bs]);
		for (uint32_t xi=bu; ; ) {
			const Node& x = nodes[xi];
			if (GC4_STATS) ++stats.pushes;
			if (x.excess != 0) {
				if (bottleneck > x.excess)
					bottleneck = x.excess;
				break;
			}
			const int p(x.parent);
			const uint32_t par(x.head[p]);
			const EdgeCap c(nodes[par].rCap[revSlotOf(x, p)]); // residual of (parent -> x)
			if (bottleneck > c)
				bottleneck = c;
			xi = par;
		}
		// bottleneck in T
		for (uint32_t xi=bv; ; ) {
			const Node& x = nodes[xi];
			if (GC4_STATS) ++stats.pushes;
			if (x.excess != 0) {
				if (bottleneck > -x.excess)
					bottleneck = -x.excess;
				break;
			}
			const int p(x.parent);
			const EdgeCap c(x.rCap[p]); // residual of (x -> parent)
			if (bottleneck > c)
				bottleneck = c;
			xi = x.head[p];
		}
		// augment the bridge arc
		nodes[bv].rCap[brs] += bottleneck;
		nbu.revRes |= (uint8_t)(1u << bs);
		nbu.rCap[bs] -= bottleneck;
		if (nbu.rCap[bs] == 0)
			nodes[bv].revRes &= (uint8_t)~(1u << brs);
		// augment the T tree, then the S tree
		++augTimestamp;
		augmentTree<false>(bv, bottleneck);
		adoption<false>();
		++augTimestamp;
		augmentTree<true>(bu, bottleneck);
		adoption<true>();
		flow += bottleneck;
	}

	template <bool sTree>
	GC4_NI void adoption() {
		bool threePass(false);
		int numOrphans(0), numOrphansUniq(0);
		while (orphanFirst != END) {
			const uint32_t xi(orphanFirst);
			Node& x = nodes[xi];
			orphanFirst = x.next;
			#if GC4_PREFETCH
			if (orphanFirst != END)
				GC4_PF(&nodes[orphanFirst]);
			for (int s=0; s<4; ++s)
				GC4_PF(&nodes[x.head[s]]);
			#endif
			++numOrphans;
			if (GC4_STATS) ++stats.orphans;
			if (x.lastAugTs != augTimestamp) {
				x.lastAugTs = augTimestamp;
				if (sTree) ++uniqOrphansS;
				else ++uniqOrphansT;
				++numOrphansUniq;
			}
			if (numOrphans >= 3*numOrphansUniq)
				threePass = true; // orphans are cascading: switch to the 3-pass strategy
			// look for a same-level parent, starting at the current arc
			int s0;
			if (x.curr) {
				s0 = x.parent;
			} else {
				s0 = 0;
				x.curr = 1;
			}
			x.parent = PARENT_NONE;
			if (x.label != (sTree ? 1 : -1)) {
				const int32_t minLabel(x.label - (sTree ? 1 : -1));
				for (int s=s0; s<4; ++s) {
					if (sTree ? ((x.revRes & (1u << s)) == 0) : (x.rCap[s] == 0))
						continue;
					Node& y = nodes[x.head[s]];
					if (y.label == minLabel) {
						x.parent = (uint8_t)s;
						y.sons |= (uint8_t)(1u << revSlotOf(x, s));
						break;
					}
				}
			}
			if (x.parent != PARENT_NONE)
				continue;
			// no parent at the same level: relabel
			// (1) the sons become orphans
			for (unsigned m=x.sons; m != 0; m&=m-1)
				addOrphanBack(x.head[CTZ(m)]);
			x.sons = 0;
			// on the top level there is no need to relabel
			if (x.label == (sTree ? topLevelS : -topLevelT)) {
				x.label = freeLabel;
				continue;
			}
			// 3-pass relabeling: move to the buckets structure
			if (threePass) {
				x.label += (sTree ? 1 : -1);
				bucketAdd<sTree>(xi);
				continue;
			}
			// (2) relabel: find the lowest-level parent
			int32_t minLabel(sTree ? topLevelS : -topLevelT);
			int best(-1);
			if (x.label != minLabel) for (int s=0; s<4; ++s) {
				if (sTree ? ((x.revRes & (1u << s)) == 0) : (x.rCap[s] == 0))
					continue;
				const Node& y = nodes[x.head[s]];
				if ((sTree ? (y.label > 0) : (y.label < 0)) &&
					(sTree ? (y.label < minLabel) : (y.label > minLabel))) {
					minLabel = y.label;
					best = s;
					if (minLabel == x.label)
						break;
				}
			}
			// (3) relabel onto the new parent
			if (best >= 0) {
				x.parent = (uint8_t)best;
				x.label = minLabel + (sTree ? 1 : -1);
				nodes[x.head[best]].sons |= (uint8_t)(1u << revSlotOf(x, best));
				// add to the active list of the next growth phase
				if (x.label == (sTree ? topLevelS : -topLevelT))
					(sTree ? activeS1 : activeT1).add(xi);
			} else {
				x.label = freeLabel;
			}
		}
		if (threePass)
			adoption3Pass<sTree>();
	}

	template <bool sTree>
	GC4_NI void adoption3Pass() {
		for (int32_t level=2; level<=maxBucket; ++level) {
			uint32_t xi;
			while ((xi = bucketPopFront(level)) != END) {
				Node& x = nodes[xi];
				#if GC4_PREFETCH
				if (buckets[level] != END)
					GC4_PF(&nodes[buckets[level]]);
				for (int s=0; s<4; ++s)
					GC4_PF(&nodes[x.head[s]]);
				#endif
				// pass 2: find the lowest-level parent
				if (x.parent == PARENT_NONE) {
					int32_t minLabel(sTree ? topLevelS : -topLevelT);
					const int32_t destLabel(x.label - (sTree ? 1 : -1));
					int best(-1);
					for (int s=0; s<4; ++s) {
						if (sTree ? ((x.revRes & (1u << s)) == 0) : (x.rCap[s] == 0))
							continue;
						const Node& y = nodes[x.head[s]];
						if ((y.excess != 0 || y.parent != PARENT_NONE) && // not an orphan
							(sTree ? (y.label > 0) : (y.label < 0)) &&
							(sTree ? (y.label < minLabel) : (y.label > minLabel))) {
							best = s;
							if ((minLabel = y.label) == destLabel)
								break;
						}
					}
					if (best < 0) {
						x.label = freeLabel;
						continue;
					}
					x.parent = (uint8_t)best;
					x.label = minLabel + (sTree ? 1 : -1);
					if (x.label != (sTree ? level : -level)) {
						bucketAdd<sTree>(xi);
						continue;
					}
				}
				// pass 3: lower the potential sons and/or set their first parent
				if (x.label != (sTree ? topLevelS : -topLevelT)) {
					const int32_t minLabel(x.label + (sTree ? 1 : -1));
					for (int s=0; s<4; ++s) {
						if (sTree ? (x.rCap[s] == 0) : ((x.revRes & (1u << s)) == 0))
							continue;
						const uint32_t yi(x.head[s]);
						Node& y = nodes[yi];
						if ((!sTree && y.label == freeLabel) ||
							// the above implicitly holds by the condition below when sTree
							(sTree ? (minLabel < y.label) : (minLabel > y.label))) {
							if (y.label != freeLabel)
								bucketRemove<sTree>(yi);
							y.label = minLabel;
							y.parent = (uint8_t)revSlotOf(x, s); // parent = reverse arc
							bucketAdd<sTree>(yi);
						}
					}
				}
				// relabel onto the new parent
				const int p(x.parent);
				nodes[x.head[p]].sons |= (uint8_t)(1u << revSlotOf(x, p));
				x.curr = 0;
				// add to the active list of the next growth phase
				if (x.label == (sTree ? topLevelS : -topLevelT))
					(sTree ? activeS1 : activeT1).add(xi);
			}
		}
		maxBucket = 0;
	}

	template <bool dirS>
	GC4_NI void growth() {
		const int32_t curLevel(dirS ? topLevelS-1 : -(topLevelT-1));
		for (uint32_t idx=0, len=active0.len; idx<len; ++idx) {
			const uint32_t xi(active0.list[idx]);
			#if GC4_PREFETCH
			if (idx+8 < len)
				GC4_PF(&nodes[active0.list[idx+8]]);
			#endif
			Node& x = nodes[xi];
			// node no longer at the growth level
			if (x.label != curLevel)
				continue;
			#if GC4_PREFETCH
			for (int s=0; s<4; ++s)
				GC4_PF(&nodes[x.head[s]]);
			#endif
			for (int s=0; s<4; ++s) {
				if (GC4_STATS) ++stats.growthArcs;
				if (dirS ? (x.rCap[s] == 0) : ((x.revRes & (1u << s)) == 0))
					continue;
				const uint32_t yi(x.head[s]);
				Node& y = nodes[yi];
				if (y.label == freeLabel) {
					// grow the tree: adopt y as a son of x
					y.parent = (uint8_t)revSlotOf(x, s);
					y.curr = 0;
					y.label = x.label + (dirS ? 1 : -1);
					x.sons |= (uint8_t)(1u << s);
					(dirS ? activeS1 : activeT1).add(yi);
				} else if (dirS ? (y.label < 0) : (y.label > 0)) {
					// the trees touch: augment through the bridge arc
					if (dirS)
						augment(xi, s);
					else
						augment(yi, revSlotOf(x, s));
					if (x.label != curLevel)
						break;
					if (dirS ? (x.rCap[s] != 0) : ((x.revRes & (1u << s)) != 0))
						--s; // the bridge is still residual: retry it
				}
			}
		}
		active0.clear();
	}
};

} // namespace GC4

#endif
