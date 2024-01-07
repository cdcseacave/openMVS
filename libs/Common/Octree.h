////////////////////////////////////////////////////////////////////
// Octree.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_OCTREE_H__
#define __SEACAVE_OCTREE_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "AABB.h"
#include "Ray.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

// basic octree class
// each item should define the operator const POINT_TYPE& returning its center;
// at build time, a functor must be supplied that returns true as long as
// the current cell should be split farther, given its number of items and radius, ex:
//	 [](Octree::IDX_TYPE size, Octree::Type radius) {
//	 	return size > SIZE && radius > RADIUS;
//	 }
// where:
//   SIZE is the minimum number of items contained by the cell so that this to be divided further
//   RADIUS is the minimum size of the cell allowed to be divided further
// both conditions represent exclusive limits and both should be true for the division to take place
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE=uint32_t>
class TOctree
{
	STATIC_ASSERT(DIMS > 0 && DIMS <= 3);

public:
	typedef TYPE Type;
	typedef typename ITEMARR_TYPE::Type ITEM_TYPE;
	typedef typename ITEMARR_TYPE::IDX IDX_TYPE;
	typedef SEACAVE::cList<IDX_TYPE,IDX_TYPE,0,1024,IDX_TYPE> IDXARR_TYPE;
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT_TYPE;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB_TYPE;
	typedef uint32_t SIZE_TYPE;

	class CELL_TYPE {
	public:
		typedef struct {
			POINT_TYPE center; // center of the current cell
		} NODE_TYPE;
		typedef struct {
			IDX_TYPE idxBegin; // index in the global array of the first item contained by this cell
			SIZE_TYPE size; // number of items contained by this cell in the global array
			DATA_TYPE data; // user data associated with this leaf
		} LEAF_TYPE;
		enum { numChildren = (2<<(DIMS-1)) };

	public:
		inline CELL_TYPE();
		inline ~CELL_TYPE();

		inline void Release();
		inline void Swap(CELL_TYPE&);

		inline unsigned ComputeChild(const POINT_TYPE& item) const;
		static void ComputeCenter(POINT_TYPE []);
		static inline POINT_TYPE ComputeChildCenter(const POINT_TYPE&, TYPE, unsigned);

		inline bool	IsLeaf() const { return (m_child==NULL); }
		inline const CELL_TYPE& GetChild(int i) const { ASSERT(!IsLeaf() && i<numChildren); return m_child[i]; }
		inline const NODE_TYPE& Node() const { ASSERT(!IsLeaf()); return m_node; }
		inline NODE_TYPE& Node() { ASSERT(!IsLeaf()); return m_node; }
		inline const LEAF_TYPE& Leaf() const { ASSERT(IsLeaf()); return m_leaf; }
		inline LEAF_TYPE& Leaf() { ASSERT(IsLeaf()); return m_leaf; }
		inline const POINT_TYPE& GetCenter() const { return Node().center; }
		inline AABB_TYPE GetAabb(TYPE radius) const { return AABB_TYPE(Node().center, radius); }
		inline AABB_TYPE GetChildAabb(unsigned idxChild, TYPE radius) const { const TYPE childRadius(radius / TYPE(2)); return AABB_TYPE(ComputeChildCenter(Node().center, childRadius, idxChild), childRadius); }
		inline IDX_TYPE GetFirstItemIdx() const { return Leaf().idxBegin; }
		inline IDX_TYPE GetLastItemIdx() const { return Leaf().idxBegin + Leaf().size; }
		inline SIZE_TYPE GetNumItems() const { return Leaf().size; }
		inline const DATA_TYPE& GetUserData() const { return Leaf().data; }
		inline DATA_TYPE& GetUserData() { return Leaf().data; }
		size_t GetNumItemsHeld() const;

	public:
		CELL_TYPE* m_child; // if not a leaf, 2^DIMS child objects
		union { // a LEAF_TYPE or NODE_TYPE object, if it is a leaf or not respectively
			NODE_TYPE m_node;
			LEAF_TYPE m_leaf;
		};
	};
	typedef SEACAVE::cList<CELL_TYPE*,CELL_TYPE*,0,256,IDX_TYPE> CELLPTRARR_TYPE;

	struct IndexInserter {
		IDXARR_TYPE& indices;
		IndexInserter(IDXARR_TYPE& _indices) : indices(_indices) {}
		void operator()(IDX_TYPE idx) { indices.Insert(idx); }
		void operator()(const IDX_TYPE* idices, SIZE_TYPE size) { indices.Join(idices, size); }
	};

	struct CellInserter {
		CELLPTRARR_TYPE& cells;
		CellInserter(CELLPTRARR_TYPE& _cells) : cells(_cells) {}
		void operator()(CELL_TYPE& cell) { cells.Insert(&cell); }
	};

public:
	inline TOctree() {}
	template <typename Functor>
	inline TOctree(const ITEMARR_TYPE&, Functor split);
	template <typename Functor>
	inline TOctree(const ITEMARR_TYPE&, const AABB_TYPE&, Functor split);

	inline void Release();
	inline void Swap(TOctree&);

	template <typename Functor>
	void Insert(const ITEMARR_TYPE&, Functor split);
	template <typename Functor>
	void Insert(const ITEMARR_TYPE&, const AABB_TYPE&, Functor split);

	template <typename INSERTER>
	inline void CollectCells(INSERTER&) const;
	template <typename INSERTER>
	void CollectCells(const CELL_TYPE&, INSERTER&) const;
	template <typename PARSER>
	inline void ParseCells(PARSER&);

	template <typename INSERTER>
	inline void Collect(INSERTER& inserter, const AABB_TYPE& aabb) const;
	inline void Collect(IDXARR_TYPE& indices, const AABB_TYPE& aabb) const;
	inline void Collect(IDX_TYPE maxNeighbors, IDXARR_TYPE& indices, const AABB_TYPE& aabb) const;

	template <typename INSERTER>
	inline void Collect(INSERTER& inserter, const POINT_TYPE& center, TYPE radius) const;
	inline void Collect(IDXARR_TYPE& indices, const POINT_TYPE& center, TYPE radius) const;
	inline void Collect(IDX_TYPE maxNeighbors, IDXARR_TYPE& indices, const POINT_TYPE& center, TYPE radius) const;

	template <typename INSERTER, typename COLLECTOR>
	inline void Collect(INSERTER& inserter, const COLLECTOR& collector) const;
	template <typename COLLECTOR>
	inline void Collect(IDXARR_TYPE& indices, const COLLECTOR& collector) const;

	template <typename FTYPE, int FDIMS, typename INSERTER>
	inline void Traverse(const TFrustum<FTYPE,FDIMS>&, INSERTER&) const;
	template <typename FTYPE, int FDIMS>
	inline void Traverse(const TFrustum<FTYPE,FDIMS>&, IDXARR_TYPE&) const;
	template <typename FTYPE, int FDIMS, typename PARSER>
	inline void TraverseCells(const TFrustum<FTYPE,FDIMS>&, PARSER&);
	template <typename FTYPE, int FDIMS>
	inline void TraverseCells(const TFrustum<FTYPE,FDIMS>&, CELLPTRARR_TYPE&);

	template <typename AREAESTIMATOR, typename CHUNKINSERTER>
	void SplitVolume(float maxArea, AREAESTIMATOR& areaEstimator, CHUNKINSERTER& chunkInserter);

	inline const CELL_TYPE& GetRoot() const { return m_root; }
	inline TYPE GetRadius() const { return m_radius; }
	inline AABB_TYPE GetAabb() const { return m_root.GetAabb(m_radius); }
	inline bool IsEmpty() const { return m_indices.empty(); }
	inline size_t GetNumItems() const { return m_indices.size(); }
	inline const IDXARR_TYPE& GetIndexArr() const { return m_indices; }
	inline const ITEM_TYPE* GetItems() const { return m_items; }
	inline void ResetItems() { m_items = NULL; }

protected:
	template <typename Functor>
	struct _InsertData {
		enum : IDX_TYPE { NO_INDEX = DECLARE_NO_INDEX(IDX_TYPE) };
		IDXARR_TYPE successors; // single connected list of next item indices
		Functor split; // used to decide if a cell needs to be split farther
	};
	template <typename Functor, bool bForceSplit>
	void _Insert(CELL_TYPE&, const POINT_TYPE& center, TYPE radius, IDX_TYPE start, IDX_TYPE size, _InsertData<Functor>&);

	template <typename PARSER>
	void _ParseCells(CELL_TYPE&, TYPE, PARSER&);

	template <typename INSERTER>
	void _Collect(const CELL_TYPE&, const AABB_TYPE&, INSERTER&) const;
	template <typename INSERTER, typename COLLECTOR>
	void _Collect(const CELL_TYPE&, TYPE, const COLLECTOR&, INSERTER&) const;

	template <typename FTYPE, int FDIMS, typename INSERTER>
	void _Traverse(const CELL_TYPE&, TYPE, const TFrustum<FTYPE,FDIMS>&, INSERTER&) const;
	template <typename FTYPE, int FDIMS, typename PARSER>
	void _TraverseCells(CELL_TYPE&, TYPE, const TFrustum<FTYPE,FDIMS>&, PARSER&);

	template <typename AREAESTIMATOR, typename CHUNKINSERTER>
	void _SplitVolume(const CELL_TYPE& parentCell, TYPE parentRadius, unsigned idxChild, float maxArea, AREAESTIMATOR& areaEstimator, CHUNKINSERTER& chunkInserter, const UnsignedArr& indices=UnsignedArr{0,1,2,3,4,5,6,7});

protected:
	const ITEM_TYPE* m_items; // original input items (the only condition is that every item to resolve to a position)
	IDXARR_TYPE m_indices; // indices to input items re-arranged spatially (as dictated by the octree)
	CELL_TYPE m_root; // first cell of the tree (always of Node type)
	TYPE m_radius; // size of the sphere containing all cells

public:
	typedef struct DEBUGINFO_TYPE {
		size_t memSize;		// total memory used
		size_t memStruct;	// memory used for the tree structure
		size_t memItems;	// memory used for the contained items
		size_t numItems;	// number of contained items
		size_t numNodes;	// total nodes...
		size_t numLeaves;	// ... from which this number of leaves
		size_t minDepth;	// minimum tree depth
		size_t maxDepth;	// maximum tree depth
		float avgDepth;		// average tree depth
		void Init() { memset(this, 0, sizeof(DEBUGINFO_TYPE)); }
		void operator += (const DEBUGINFO_TYPE& r) {
			avgDepth = avgDepth*numNodes + r.avgDepth*r.numNodes;
			memSize += r.memSize; memStruct += r.memStruct; memItems += r.memItems;
			numItems += r.numItems; numNodes += r.numNodes; numLeaves += r.numLeaves;
			if (minDepth > r.minDepth) minDepth = r.minDepth;
			if (maxDepth < r.maxDepth) maxDepth = r.maxDepth;
			avgDepth /= numNodes;
		}
	} DEBUGINFO;

	void GetDebugInfo(DEBUGINFO* =NULL, bool bPrintStats=false) const;
	static void LogDebugInfo(const DEBUGINFO&);

protected:
	void _GetDebugInfo(const CELL_TYPE&, unsigned, DEBUGINFO&) const;
}; // class TOctree
/*----------------------------------------------------------------*/


#include "Octree.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_OCTREE_H__
