////////////////////////////////////////////////////////////////////
// DisjointSet.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef _MATH_DISJOINTSET_H_
#define _MATH_DISJOINTSET_H_


// I N C L U D E S /////////////////////////////////////////////////

#include <vector>
#include <utility>


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

/**
 * @brief Disjoint-set data structure for union-find
 */
template <typename T = uint32_t>
class DisjointSet
{
public:
	typedef T Type;

protected:
	std::vector<Type> parent; // Parent pointer for each element (representative if parent[x] == x)
	std::vector<Type> rank;   // Upper bound on tree height for union-by-rank heuristic

public:
	// Initialize with each element in its own set and rank 0.
	DisjointSet(size_t n) : parent(n), rank(n, 0) {
		std::iota(parent.begin(), parent.end(), static_cast<Type>(0));
	}

	DisjointSet& Reset(size_t n) {
		parent.resize(n);
		rank.assign(n, 0);
		std::iota(parent.begin(), parent.end(), static_cast<Type>(0));
		return *this;
	}

	// Find representative with full path compression (every node on the path then points to the root, which
	// CompressAllPaths relies on); iterative, since UnionMin (no rank) can build chains as long as the set.
	Type Find(Type x) {
		Type root = x;
		while (parent[root] != root)
			root = parent[root];
		while (parent[x] != root) {
			const Type next = parent[x];
			parent[x] = root;
			x = next;
		}
		return root;
	}

	// Standard union-by-rank merge; no metadata guards.
	void Union(Type x, Type y) {
		const Type px = Find(x);
		const Type py = Find(y);
		if (px == py) return;
		if (rank[px] < rank[py]) {
			parent[px] = py;
		} else if (rank[px] > rank[py]) {
			parent[py] = px;
		} else {
			parent[py] = px;
			++rank[px];
		}
	}

	// Deterministic union keeping the smaller element as root; returns true if two sets merged.
	bool UnionMin(Type x, Type y) {
		Type px = Find(x);
		Type py = Find(y);
		if (px == py)
			return false;
		if (px > py)
			std::swap(px, py);
		parent[py] = px;
		return true;
	}

	// Union with a guard+merge callback.
	// The callback operates on the finalized root ordering (dst, src)
	// after union-by-rank selection. It must perform any necessary
	// validation and metadata merge; returning false vetoes the union.
	// Return true if the sets are now united (or were already united), false if blocked
	template <typename GuardMergeFn>
	bool UnionIf(Type x, Type y, GuardMergeFn&& guardMerge) {
		Type px = Find(x);
		Type py = Find(y);
		if (px == py)
			return true;
		// Decide destination/source roots using rank heuristic
		Type dst = px;
		Type src = py;
		if (rank[dst] < rank[src])
			std::swap(dst, src);
		// Callback performs guard and merge; veto if false
		if (!guardMerge(dst, src))
			return false;
		parent[src] = dst;
		if (rank[dst] == rank[src])
			++rank[dst];
		return true;
	}

	// Compress all paths to point directly to their root representative.
	// Call this before using const query methods for accurate results.
	DisjointSet& CompressAllPaths() {
		FOREACH(i, parent)
			Find(static_cast<Type>(i));
		return *this;
	}

	// Get all connected components and their sizes.
	// Returns map of root element -> component size
	// Note: Call CompressAllPaths() first to make sure all paths are compressed and parent pointers are accurate.
	std::unordered_map<Type, unsigned> GetComponentSizes() const {
		std::unordered_map<Type, unsigned> componentSizes;
		for (const Type root : parent)
			componentSizes[root]++;
		return componentSizes;
	}

	// Get connected components as a vector of component IDs.
	// Returns the number of components and fills the provided vector where result[i] is the component ID for node i.
	// Component IDs are sequential integers starting from 0.
	// Note: Call CompressAllPaths() first to make sure all paths are compressed and parent pointers are accurate.
	unsigned GetComponents(std::vector<Type>& components) const {
		components.resize(parent.size());
		std::unordered_map<Type, Type> rootToComponentId;
		Type nextComponentId = 0;
		FOREACH(i, parent) {
			const Type root = parent[i];
			auto ret = rootToComponentId.emplace(root, nextComponentId);
			if (ret.second)
				++nextComponentId;
			components[i] = ret.first->second;
		}
		return rootToComponentId.size();
	}
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // _MATH_DISJOINTSET_H_
