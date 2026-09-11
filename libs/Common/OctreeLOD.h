////////////////////////////////////////////////////////////////////
// OctreeLOD.h
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_OCTREELOD_H__
#define __SEACAVE_OCTREELOD_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "AABB.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// Multi-level LOD octree that distributes points across all tree levels.
// Unlike TOctree (which stores all points in leaves for spatial queries),
// this structure assigns each point to exactly one level using a configurable
// subsampling functor — coarser levels contain spatially representative points,
// finer levels add progressive detail.
//
// Like TOctree, uses a single shared index array (m_indices) where each node
// stores only an offset and size, and spatial info (center, radius) is computed
// on-the-fly during traversal from the root center + radius.
//
// The subsampling functor controls LOD point selection at each node, mirroring
// how TOctree::Insert() accepts a split functor. The functor signature is:
//   void subsample(IDXARR_TYPE& selectedOut, const IDXARR_TYPE& candidates,
//                  const ITEM_TYPE* items, const AABB_TYPE& nodeAABB, unsigned depth)
// It receives candidate indices and must populate selectedOut with the subset
// to represent this LOD level. Unselected points propagate to children.
//
// Usage with built-in grid subsampler:
//   TOctreeLOD<PointArr, float, 3> lod;
//   lod.Insert(points, GridSubsample<PointArr, float, 3>(128));
//
// Usage with custom subsampler (lambda):
//   lod.Insert(points, [](auto& sel, const auto& cand, const auto* items,
//                          const auto& aabb, unsigned depth) {
//       for (size_t i = 0; i < cand.size(); i += (1u << depth))
//           sel.Insert(cand[i]);
//   });
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
class TOctreeLOD
{
	static_assert(DIMS > 0 && DIMS <= 3);

public:
	typedef TYPE Type;
	typedef typename ITEMARR_TYPE::Type ITEM_TYPE;
	typedef typename ITEMARR_TYPE::IDX IDX_TYPE;
	typedef SEACAVE::cList<IDX_TYPE,IDX_TYPE,0,1024,IDX_TYPE> IDXARR_TYPE;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT_TYPE;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB_TYPE;
	typedef uint32_t SIZE_TYPE;
	enum { numChildren = (2<<(DIMS-1)) };

	// Node stores only index range + child pointers.
	// Spatial info (center, radius) is computed on-the-fly during traversal.
	struct Node {
		IDX_TYPE idxBegin;                              // offset into shared m_indices array
		SIZE_TYPE size;                                 // number of point indices at THIS level
		uint8_t childMask;                              // bit i set if children[i] is populated
		std::unique_ptr<Node> children[2<<(DIMS-1)];    // up to 8 children for 3D

		inline Node() : idxBegin(0), size(0), childMask(0) {}

		inline bool IsLeaf() const { return childMask == 0; }
		inline IDX_TYPE GetFirstItemIdx() const { return idxBegin; }
		inline IDX_TYPE GetLastItemIdx() const { return idxBegin + size; }
		inline SIZE_TYPE GetNumItems() const { return size; }
		size_t GetNumItemsHeld() const;
	};

public:
	inline TOctreeLOD() : m_items(NULL), m_radius(0), m_maxDepth(0), m_totalNodes(0), m_spacing(0) {}

	template <typename Functor>
	inline TOctreeLOD(const ITEMARR_TYPE& items, Functor subsample, unsigned maxDepth = 20);
	template <typename Functor>
	inline TOctreeLOD(const ITEMARR_TYPE& items, const AABB_TYPE& aabb, Functor subsample, unsigned maxDepth = 20);

	inline void Release();

	// Build the LOD octree from items using the given subsampling functor
	template <typename Functor>
	void Insert(const ITEMARR_TYPE& items, Functor subsample, unsigned maxDepth = 20);
	template <typename Functor>
	void Insert(const ITEMARR_TYPE& items, const AABB_TYPE& aabb, Functor subsample, unsigned maxDepth = 20);

	// Accessors
	inline const Node& GetRoot() const { return m_root; }
	inline AABB_TYPE GetAABB() const { return AABB_TYPE(m_center, m_radius); }
	inline const POINT_TYPE& GetCenter() const { return m_center; }
	inline Type GetRadius() const { return m_radius; }
	inline unsigned GetMaxDepth() const { return m_maxDepth; }
	inline size_t GetTotalNodes() const { return m_totalNodes; }
	inline Type GetSpacing() const { return m_spacing; }
	inline bool IsEmpty() const { return m_items == NULL; }
	inline const ITEM_TYPE* GetItems() const { return m_items; }
	inline const IDXARR_TYPE& GetIndexArr() const { return m_indices; }

	// Breadth-first traversal: visitor(const Node& node, const POINT_TYPE& center, TYPE radius)
	template <typename Visitor>
	void TraverseBFS(Visitor&& visitor) const;

	// Depth-first traversal: visitor(const Node& node, const POINT_TYPE& center, TYPE radius)
	template <typename Visitor>
	void TraverseDFS(Visitor&& visitor) const;

public:
	typedef struct DEBUGINFO_TYPE {
		size_t totalNodes;
		size_t leafNodes;
		size_t internalNodes;
		size_t totalPoints;     // sum of sizes across all nodes
		unsigned minDepth;
		unsigned maxDepth;
		float avgPointsPerNode;
		void Init() { memset(this, 0, sizeof(DEBUGINFO_TYPE)); minDepth = UINT_MAX; }
	} DEBUGINFO;

	void GetDebugInfo(DEBUGINFO* = NULL, bool bPrintStats = false) const;

protected:
	// Compute which octant a point falls into relative to the given center
	static inline unsigned ComputeChild(const POINT_TYPE& item, const POINT_TYPE& center);

	// Compute child center from parent center + child radius + octant index
	// (same formula as TOctree::CELL_TYPE::ComputeChildCenter)
	static inline POINT_TYPE ComputeChildCenter(const POINT_TYPE& center, TYPE childRadius, unsigned idxChild);

	template <typename Functor>
	void _Insert(Node& node, IDXARR_TYPE& candidateIndices, const POINT_TYPE& center, TYPE radius, unsigned depth, unsigned maxDepth, Functor& subsample);

	template <typename Visitor>
	void _TraverseDFS(const Node& node, const POINT_TYPE& center, TYPE radius, Visitor& visitor) const;

	void _GetDebugInfo(const Node& node, unsigned depth, DEBUGINFO& info) const;

protected:
	const ITEM_TYPE* m_items;   // pointer to original items array
	IDXARR_TYPE m_indices;      // shared flat array of point indices, rearranged by LOD level
	Node m_root;                // root node of the LOD tree
	POINT_TYPE m_center;        // center of root cell
	TYPE m_radius;              // half-extent of root cell
	unsigned m_maxDepth;        // actual maximum depth reached during build
	size_t m_totalNodes;        // total node count
	Type m_spacing;             // root-level grid spacing
}; // class TOctreeLOD
/*----------------------------------------------------------------*/


// Built-in grid-based subsampling functor (state-of-the-art for LOD octrees).
// At each node, divides the AABB into a uniform grid and selects the point
// closest to each occupied cell's center. Produces spatially uniform LOD
// distributions — the same strategy used by PotreeConverter 2.0 and Entwine.
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
struct GridSubsample {
	typedef typename ITEMARR_TYPE::Type ITEM_TYPE;
	typedef typename ITEMARR_TYPE::IDX IDX_TYPE;
	typedef SEACAVE::cList<IDX_TYPE,IDX_TYPE,0,1024,IDX_TYPE> IDXARR_TYPE;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT_TYPE;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB_TYPE;

	unsigned gridResolution; // cells per axis (default 128)

	GridSubsample(unsigned _gridResolution = 128) : gridResolution(_gridResolution) {}

	void operator()(IDXARR_TYPE& selectedOut, const IDXARR_TYPE& candidates,
	                const ITEM_TYPE* items, const AABB_TYPE& aabb, unsigned /*depth*/) const
	{
		ASSERT(!candidates.empty());
		const POINT_TYPE aabbSize(aabb.GetSize());
		const TYPE maxExtent = aabbSize.maxCoeff();
		if (maxExtent <= TYPE(0)) {
			selectedOut = candidates;
			return;
		}
		const TYPE spacing = maxExtent / TYPE(gridResolution);
		const TYPE invSpacing = TYPE(1) / spacing;

		Eigen::Matrix<unsigned,DIMS,1> gridDims;
		for (int d = 0; d < DIMS; ++d)
			gridDims[d] = std::max(1u, (unsigned)std::ceil(aabbSize[d] * invSpacing));

		std::unordered_map<uint64_t, std::pair<IDX_TYPE, TYPE>> grid;
		grid.reserve(std::min((size_t)candidates.size(), (size_t)(gridResolution * gridResolution)));

		for (IDX_TYPE ci = 0; ci < (IDX_TYPE)candidates.size(); ++ci) {
			const IDX_TYPE idx = candidates[ci];
			const POINT_TYPE& pt = reinterpret_cast<const POINT_TYPE&>(items[idx]);
			const POINT_TYPE rel = (pt - aabb.ptMin) * invSpacing;

			Eigen::Matrix<unsigned,DIMS,1> cell;
			for (int d = 0; d < DIMS; ++d)
				cell[d] = std::min((unsigned)std::max(TYPE(0), rel[d]), gridDims[d] - 1);

			uint64_t key = cell[0];
			if (DIMS > 1) key += (uint64_t)cell[1] * gridDims[0];
			if (DIMS > 2) key += (uint64_t)cell[2] * (uint64_t)gridDims[0] * gridDims[1];

			POINT_TYPE cellCenter;
			for (int d = 0; d < DIMS; ++d)
				cellCenter[d] = aabb.ptMin[d] + (TYPE(cell[d]) + TYPE(0.5)) * spacing;
			const TYPE distSq = (pt - cellCenter).squaredNorm();

			auto it = grid.find(key);
			if (it == grid.end()) {
				grid.emplace(key, std::make_pair(idx, distSq));
			} else if (distSq < it->second.second) {
				it->second = std::make_pair(idx, distSq);
			}
		}

		selectedOut.Reserve((IDX_TYPE)grid.size());
		for (const auto& kv : grid)
			selectedOut.Insert(kv.second.first);
	}
}; // struct GridSubsample
/*----------------------------------------------------------------*/


#include "OctreeLOD.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_OCTREELOD_H__
