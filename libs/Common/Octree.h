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

// raw array wrapper
template <typename TYPE>
class TItemArr
{
public:
	typedef TYPE Type;

	inline TItemArr() {}
	inline TItemArr(const TYPE* data, IDX size) : m_data(data), m_size(size) {}
	inline void Set(const TYPE* data, IDX size) { m_data = data; m_size = size; }

	inline const TYPE& operator[](IDX i) const { return m_data[i]; }
	inline const TYPE* Begin() const { return m_data; }
	inline const TYPE* GetData() const { return m_data; }
	inline IDX GetSize() const { return m_size; }

protected:
	const TYPE* m_data;
	IDX m_size;
}; // class TItemArr
/*----------------------------------------------------------------*/


// octree cell class
template <typename TYPE, int DIMS, typename DATA_TYPE>
class TOctreeCell
{
	STATIC_ASSERT(DIMS > 0 && DIMS <= 3);

public:
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT_TYPE;
	typedef SEACAVE::TAABB<TYPE,DIMS> AABB_TYPE;
	typedef uint32_t SIZE_TYPE;

	typedef struct {
		POINT_TYPE center; // center of the current cell
	} NODE_TYPE;
	typedef struct {
		IDX idxBegin; // index in the global array of the first item contained by this cell
		SIZE_TYPE size; // number of items contained by this cell in the global array
		DATA_TYPE data; // user data associated with this leaf
	} LEAF_TYPE;
	enum { dataSize = (sizeof(NODE_TYPE)>sizeof(LEAF_TYPE) ? sizeof(NODE_TYPE) : sizeof(LEAF_TYPE)) };
	enum { numChildren = (2<<(DIMS-1)) };

public:
	inline TOctreeCell();
	inline TOctreeCell(TOctreeCell*);
	inline ~TOctreeCell();

	inline void Release();
	inline void Swap(TOctreeCell&);

	inline unsigned ComputeChild(const POINT_TYPE& item) const;
	static void ComputeCenter(POINT_TYPE []);

	inline bool	IsLeaf() const { return (m_child==NULL); }
	inline const TOctreeCell& GetChild(int i) const { ASSERT(!IsLeaf() && i<numChildren); return m_child[i]; }
	inline const NODE_TYPE& Node() const { ASSERT(!IsLeaf()); return *((const NODE_TYPE*)m_data); }
	inline NODE_TYPE& Node() { ASSERT(!IsLeaf()); return *((NODE_TYPE*)m_data); }
	inline const LEAF_TYPE& Leaf() const { ASSERT(IsLeaf()); return *((const LEAF_TYPE*)m_data); }
	inline LEAF_TYPE& Leaf() { ASSERT(IsLeaf()); return *((LEAF_TYPE*)m_data); }
	inline const POINT_TYPE& GetCenter() const { return Node().center; }
	inline AABB_TYPE GetAabb(TYPE radius) const { return AABB_TYPE(Node().center, radius); }
	inline IDX GetLastItem() const { return Leaf().idxBegin + Leaf().size; }
	inline SIZE_TYPE GetNumItems() const { return Leaf().size; }
	inline const DATA_TYPE& GetUserData() const { return Leaf().data; }
	inline DATA_TYPE& GetUserData() { return Leaf().data; }
	size_t GetNumItemsHeld() const;

public:
	TOctreeCell* m_child; // if not a leaf, 2^DIMS child objects
	uint8_t m_data[dataSize]; // a LEAF_TYPE or NODE_TYPE object, if it is a leaf or not respectively
}; // class TOctreeCell
/*----------------------------------------------------------------*/


// basic octree class
// each item should define the operator const POINT_TYPE& returning its center
// SIZE is the minimum number of items contained by the cell so that this to be divided further
// NOM/DENOM is the size of the cell minimum allowed to be divided further
// both conditions represent exclusive limits and both should be true for the division to take place
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE=uint32_t, int SIZE=16, int NOM=0, int DENOM=1>
class TOctree
{
public:
	typedef TYPE Type;
	typedef TOctreeCell<TYPE,DIMS,DATA_TYPE> CELL_TYPE;
	typedef typename ITEMARR_TYPE::Type ITEM_TYPE;
	typedef SEACAVE::cList<IDX,IDX,0,1024> IDXARR_TYPE;
	typedef SEACAVE::cList<CELL_TYPE*,CELL_TYPE*,0,256> CELLPTRARR_TYPE;
	typedef typename IDXARR_TYPE::Type IDX_TYPE;
	typedef typename CELL_TYPE::POINT_TYPE POINT_TYPE;
	typedef typename CELL_TYPE::AABB_TYPE AABB_TYPE;

	struct IndexInserter {
		IDXARR_TYPE& indices;
		IndexInserter(IDXARR_TYPE& _indices) : indices(_indices) {}
		void operator()(IDX_TYPE idx) { indices.Insert(idx); }
		void operator()(const IDX_TYPE* idices, size_t size) { indices.Join(idices, size); }
	};

	struct CellInserter {
		CELLPTRARR_TYPE& cells;
		CellInserter(CELLPTRARR_TYPE& _cells) : cells(_cells) {}
		void operator()(CELL_TYPE& cell) { cells.Insert(&cell); }
	};

public:
	inline TOctree() {}
	inline TOctree(const ITEMARR_TYPE&);
	inline TOctree(const ITEMARR_TYPE&, const AABB_TYPE&);

	inline void Release();
	inline void Swap(TOctree&);

	void Insert(const ITEMARR_TYPE&);
	void Insert(const ITEMARR_TYPE&, const AABB_TYPE&);

	template <typename INSERTER>
	inline void CollectCells(INSERTER&) const;
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

	inline AABB_TYPE GetAabb() const { return m_root.GetAabb(m_radius); }
	inline TYPE GetRadius(const AABB_TYPE& aabb) const {
		// radius of the root cell
		const POINT_TYPE size(aabb.GetSize() / TYPE(2));
		TYPE radius = size[0];
		if (DIMS > 1 && radius < size[1])
			radius = size[1];
		if (DIMS > 2 && radius < size[2])
			radius = size[2];
		return radius;
	}
	inline bool IsEmpty() const { return m_indices.IsEmpty(); }
	inline size_t GetNumItems() const { return m_indices.GetSize(); }
	inline const IDXARR_TYPE& GetIndexArr() const { return m_indices; }
	inline const ITEM_TYPE* GetItems() const { return m_items; }

protected:
	static inline POINT_TYPE ComputeChildCenter(const POINT_TYPE&, TYPE, unsigned);

	void _Insert(CELL_TYPE&, TYPE, IDXARR_TYPE []);
	void _Insert(CELL_TYPE&, const POINT_TYPE&, TYPE, IDXARR_TYPE&);

	template <typename INSERTER>
	void _CollectCells(const CELL_TYPE&, INSERTER&) const;
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

protected:
	const ITEM_TYPE* m_items; // original input items (the only condition is that every item to resolve to a position)
	IDXARR_TYPE m_indices; // indices to input items re-arranged spatially (as dictated by the octree)
	CELL_TYPE m_root; // first cell of the tree (always of Node type)
	TYPE m_radius; // size of the sphere containing all cells

#ifndef _RELEASE
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
#endif
}; // class TOctree
/*----------------------------------------------------------------*/


#include "Octree.inl"
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_OCTREE_H__
