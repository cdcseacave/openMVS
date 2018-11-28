////////////////////////////////////////////////////////////////////
// Octree.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////

// max number of items in one cell
#define OCTREE_CELLITEMS SIZE

#ifdef _USE_OPENMP
// minimum number of polygons for which we do multi-threading
#define OCTREE_MIN_ITEMS_MINTHREAD 1024*2
#endif

// size of a cell
#define OCTREE_CELLSIZE (TYPE(NOM)/TYPE(DENOM))
// radius of a cell
#define OCTREE_CELLRADIUS (OCTREE_CELLSIZE/2)


// S T R U C T S ///////////////////////////////////////////////////

template <typename TYPE, int DIMS, typename DATA_TYPE>
inline TOctreeCell<TYPE,DIMS,DATA_TYPE>::TOctreeCell()
	:
	m_child(NULL)
{
} // constructor
template <typename TYPE, int DIMS, typename DATA_TYPE>
inline TOctreeCell<TYPE,DIMS,DATA_TYPE>::TOctreeCell(TOctreeCell* children)
	:
	m_child(children)
{
} // constructor
/*----------------------------------------------------------------*/

template <typename TYPE, int DIMS, typename DATA_TYPE>
inline TOctreeCell<TYPE,DIMS,DATA_TYPE>::~TOctreeCell()
{
	delete[] m_child;
} // destructor
/*----------------------------------------------------------------*/


template <typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctreeCell<TYPE,DIMS,DATA_TYPE>::Release()
{
	delete[] m_child;
	m_child = NULL;
} // destructor
// swap the two octrees
template <typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctreeCell<TYPE,DIMS,DATA_TYPE>::Swap(TOctreeCell& rhs)
{
	std::swap(m_child, rhs.m_child);
	uint8_t tmpData[dataSize];
	memcpy(tmpData, m_data, dataSize);
	memcpy(m_data, rhs.m_data, dataSize);
	memcpy(rhs.m_data, tmpData, dataSize);
} // Swap
/*----------------------------------------------------------------*/


// compute item's index corresponding to the containing cell
template <typename TYPE, int DIMS, typename DATA_TYPE>
inline unsigned TOctreeCell<TYPE,DIMS,DATA_TYPE>::ComputeChild(const POINT_TYPE& item) const
{
	ASSERT(!IsLeaf());
	unsigned idx = 0;
	if (item[0] >= Node().center[0])
		idx |= (1<<0);
	if (DIMS > 1)
	if (item[1] >= Node().center[1])
		idx |= (1<<1);
	if (DIMS > 2)
	if (item[2] >= Node().center[2])
		idx |= (1<<2);
	return idx;
} // ComputeChild
/*----------------------------------------------------------------*/

template <typename TYPE, int DIMS, typename DATA_TYPE>
void TOctreeCell<TYPE,DIMS,DATA_TYPE>::ComputeCenter(POINT_TYPE centers[])
{
	if (DIMS == 1) {
		centers[0] << -1;
		centers[1] <<  1;
	} else
	if (DIMS == 2) {
		centers[0] << -1,-1;
		centers[1] <<  1,-1;
		centers[2] << -1, 1;
		centers[3] <<  1, 1;
	} else
	if (DIMS == 3) {
		centers[0] << -1,-1,-1;
		centers[1] <<  1,-1,-1;
		centers[2] << -1, 1,-1;
		centers[3] <<  1, 1,-1;
		centers[4] << -1,-1, 1;
		centers[5] <<  1,-1, 1;
		centers[6] << -1, 1, 1;
		centers[7] <<  1, 1, 1;
	}
} // ComputeCenter
/*----------------------------------------------------------------*/


// count the number of items contained by the given octree-cell
template <typename TYPE, int DIMS, typename DATA_TYPE>
size_t TOctreeCell<TYPE,DIMS,DATA_TYPE>::GetNumItemsHeld() const
{
	if (IsLeaf())
		return GetNumItems();
	size_t numItems = 0;
	for (int i=0; i<numChildren; ++i)
		numItems += GetChild(i).GetNumItemsHeld();
	return numItems;
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

// build tree with the given items
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::TOctree(const ITEMARR_TYPE& items)
{
	Insert(items);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::TOctree(const ITEMARR_TYPE& items, const AABB_TYPE& aabb)
{
	Insert(items, aabb);
} // constructor
/*----------------------------------------------------------------*/


// destroy tree
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Release()
{
	m_indices.Release();
	m_root.Release();
} // Release
// swap the two octrees
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Swap(TOctree& rhs)
{
	std::swap(m_items, rhs.m_items);
	m_indices.Swap(rhs.m_indices);
	m_root.Swap(rhs.m_root);
	std::swap(m_radius, rhs.m_radius);
} // Swap
/*----------------------------------------------------------------*/


// destroy tree
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline typename TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::POINT_TYPE TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::ComputeChildCenter(const POINT_TYPE& center, TYPE radius, unsigned idxChild)
{
	struct CENTERARR_TYPE {
		POINT_TYPE child[CELL_TYPE::numChildren];
		inline CENTERARR_TYPE() { CELL_TYPE::ComputeCenter(child); }
	};
	static const CENTERARR_TYPE centers;
	return center + centers.child[idxChild] * radius;
} // ComputeChildCenter
/*----------------------------------------------------------------*/

// add the given item to the tree
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_Insert(CELL_TYPE& cell, TYPE radius, IDXARR_TYPE childrenIndices[])
{
	ASSERT(!cell.IsLeaf());
	// setup each cell
	const POINT_TYPE& center = cell.Node().center;
	const TYPE childRadius = radius / TYPE(2);
	for (int i=0; i<CELL_TYPE::numChildren; ++i) {
		IDXARR_TYPE& childIndices = childrenIndices[i];
		CELL_TYPE& child = cell.m_child[i];
		// if this child cell needs to be divided further
		if (childIndices.GetSize() > OCTREE_CELLITEMS && (NOM == 0 || childRadius > OCTREE_CELLRADIUS)) {
			// proceed recursively
			const POINT_TYPE childCenter(ComputeChildCenter(center, childRadius, i));
			_Insert(child, childCenter, childRadius, childIndices);
		} else {
			// init leaf
			typename CELL_TYPE::LEAF_TYPE& leaf = child.Leaf();
			leaf.idxBegin = m_indices.GetSize();
			leaf.size = (typename CELL_TYPE::SIZE_TYPE)childIndices.GetSize();
			// store items
			m_indices.Join(childIndices);
			childIndices.Release();
		}
	}
} // _Insert
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_Insert(CELL_TYPE& cell, const POINT_TYPE& center, TYPE radius, IDXARR_TYPE& indices)
{
	ASSERT(cell.IsLeaf());
	ASSERT(indices.GetSize() > OCTREE_CELLITEMS);
	ASSERT(NOM == 0 || radius > OCTREE_CELLRADIUS);
	// divide cell
	// transform this cell in node
	cell.m_child = new CELL_TYPE[CELL_TYPE::numChildren];
	cell.Node().center = center;
	// divide the items in separate indices lists corresponding to each child
	const size_t reserveSize = indices.GetSize() / 3;
	IDXARR_TYPE childrenIndices[CELL_TYPE::numChildren];
	for (int i=0; i<CELL_TYPE::numChildren; ++i)
		childrenIndices[i].Reserve(reserveSize);
	FOREACHPTR(it, indices) {
		const unsigned idx = cell.ComputeChild((const POINT_TYPE)m_items[*it]);
		childrenIndices[idx].Insert(*it);
	}
	indices.Release();
	// setup each cell
	_Insert(cell, radius, childrenIndices);
} // _Insert
/*----------------------------------------------------------------*/

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Insert(const ITEMARR_TYPE& items, const AABB_TYPE& aabb)
{
	Release();
	m_items = items.Begin();
	m_radius = GetRadius(aabb);
	// build tree
	// create root as node, even if we do not need to divide
	m_indices.Reserve(items.GetSize());
	// divide cell
	m_root.m_child = new CELL_TYPE[CELL_TYPE::numChildren];
	m_root.Node().center = aabb.GetCenter();
	// divide the items in separate indices lists corresponding to each child
	const size_t reserveSize = items.GetSize() / 3;
	IDXARR_TYPE childrenIndices[CELL_TYPE::numChildren];
	for (int i=0; i<CELL_TYPE::numChildren; ++i)
		childrenIndices[i].Reserve(reserveSize);
	FOREACH(i, items) {
		const unsigned idx = m_root.ComputeChild((const POINT_TYPE)items[i]);
		childrenIndices[idx].Insert(i);
	}
	// setup each cell
	_Insert(m_root, m_radius, childrenIndices);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Insert(const ITEMARR_TYPE& items)
{
	ASSERT(!items.IsEmpty());
	AABB_TYPE aabb;
	ASSERT(sizeof(POINT_TYPE) == sizeof(typename ITEMARR_TYPE::Type));
	aabb.Set((const POINT_TYPE*)items.Begin(), items.GetSize());
	aabb.Enlarge(ZEROTOLERANCE<TYPE>()*TYPE(10));
	Insert(items, aabb);
} // Insert
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_CollectCells(const CELL_TYPE& cell, INSERTER& inserter) const
{
	if (cell.IsLeaf()) {
		inserter(m_indices.Begin()+cell.Leaf().idxBegin, cell.GetNumItems());
		return;
	}
	for (int i=0; i<CELL_TYPE::numChildren; ++i)
		_CollectCells(cell.m_child[i], inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename PARSER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_ParseCells(CELL_TYPE& cell, TYPE radius, PARSER& parser)
{
	if (cell.IsLeaf()) {
		parser(cell, radius);
		return;
	}
	const TYPE childRadius = radius / TYPE(2);
	for (int i=0; i<CELL_TYPE::numChildren; ++i)
		_ParseCells(cell.m_child[i], childRadius, parser);
}
/*----------------------------------------------------------------*/

// calls parser for each leaf of the octree (the IDX_TYPE operator has to be defined)
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::CollectCells(INSERTER& inserter) const
{
	_CollectCells(m_root, inserter);
}
// calls parser for each leaf of the octree (the CELL_TYPE operator has to be defined)
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename PARSER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::ParseCells(PARSER& parser)
{
	_ParseCells(m_root, m_radius, parser);
}
/*----------------------------------------------------------------*/


// find all items contained by the given bounding box
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_Collect(const CELL_TYPE& cell, const AABB_TYPE& aabb, INSERTER& inserter) const
{
	if (cell.IsLeaf()) {
		// add all items contained by the bounding-box
		for (IDX i=0; i<cell.Leaf().size; ++i) {
			const IDX idx = m_indices[cell.Leaf().idxBegin + i];
			if (aabb.Intersects((const POINT_TYPE)m_items[idx]))
				inserter(idx);
		}
		return;
	}
	// find the cells containing the box extremes
	const unsigned idxMin = cell.ComputeChild(aabb.ptMin);
	const unsigned idxMax = cell.ComputeChild(aabb.ptMax);
	// if both extremes are in the same cell
	if (idxMin == idxMax) {
		// all searched items are inside a single child
		return _Collect(cell.m_child[idxMin], aabb, inserter);
	}
	// divide the bounding-box in 2^DIMS boxes split by the cell center
	AABB_TYPE aabbs[CELL_TYPE::numChildren];
	unsigned first;
	unsigned n = aabb.SplitBy(cell.Node().center, aabbs, first);
	if (n == 0) {
		// the aabb has one of the sides right on the cell division line
		#if 0
		// so find the cell containing the aabb center and add its items
		const unsigned idx = cell.ComputeChild(aabb.GetCenter());
		_Collect(cell.m_child[idx], aabb, inserter);
		#else
		// so collect both intersecting cells
		// (seems to work better for points right on the border)
		_Collect(cell.m_child[idxMin], aabb, inserter);
		_Collect(cell.m_child[idxMax], aabb, inserter);
		#endif
		return;
	}
	// collect each cell
	for (unsigned i=0; i<n; ++i) {
		// all searched items are inside a single child
		const AABB_TYPE& aabbChild = aabbs[(first+i)%CELL_TYPE::numChildren];
		const unsigned idx = cell.ComputeChild(aabbChild.ptMin);
		_Collect(cell.m_child[idx], aabbChild, inserter);
	}
} // Collect
// find all items contained by the cells intersected by the given line
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename INSERTER, typename COLLECTOR>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_Collect(const CELL_TYPE& cell, TYPE radius, const COLLECTOR& collector, INSERTER& inserter) const
{
	ASSERT(!cell.IsLeaf());
	const TYPE childRadius = radius / TYPE(2);
	for (int i=0; i<CELL_TYPE::numChildren; ++i) {
		const CELL_TYPE& childCell = cell.m_child[i];
		if (childCell.IsLeaf()) {
			if (collector.Intersects(ComputeChildCenter(cell.GetCenter(), childRadius, i), childRadius))
				inserter(m_indices.Begin()+childCell.Leaf().idxBegin, childCell.GetNumItems());
		} else {
			if (collector.Intersects(childCell.Node().center, childRadius))
				_Collect(childCell, childRadius, collector, inserter);
		}
	}
} // Collect
/*----------------------------------------------------------------*/

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename INSERTER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(INSERTER& inserter, const AABB_TYPE& aabb) const
{
	_Collect(m_root, aabb, inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(IDXARR_TYPE& indices, const AABB_TYPE& aabb) const
{
	_Collect(m_root, aabb, IndexInserter(indices));
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename INSERTER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(INSERTER& inserter, const POINT_TYPE& center, TYPE radius) const
{
	_Collect(m_root, AABB_TYPE(center, radius), inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(IDXARR_TYPE& indices, const POINT_TYPE& center, TYPE radius) const
{
	_Collect(m_root, AABB_TYPE(center, radius), IndexInserter(indices));
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename INSERTER, typename COLLECTOR>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(INSERTER& inserter, const COLLECTOR& collector) const
{
	_Collect(m_root, m_radius, collector, inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename COLLECTOR>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(IDXARR_TYPE& indices, const COLLECTOR& collector) const
{
	_Collect(m_root, m_radius, collector, IndexInserter(indices));
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(IDX_TYPE maxNeighbors, IDXARR_TYPE& indices, const AABB_TYPE& aabb) const
{
	_Collect(m_root, aabb, IndexInserter(indices));
	if (indices.GetSize() > maxNeighbors) {
		// keep only the closest neighbors
		typedef TIndexScore<IDX_TYPE,TYPE> ItemIndexScore;
		typedef cList<ItemIndexScore, const ItemIndexScore&, 0> ItemIndexScoreArr;
		ItemIndexScoreArr indexscores(indices.GetSize());
		const POINT_TYPE center(aabb.GetCenter());
		FOREACH(i, indices) {
			const IDX_TYPE& idx = indices[i];
			const TYPE score(-(center-(const POINT_TYPE)m_items[idx]).squaredNorm());
			indexscores[i] = ItemIndexScore(idx,score);
		}
		indices.Empty();
		indexscores.Sort();
		for (IDX i=0; i<maxNeighbors; ++i)
			indices.Insert(indexscores[i].idx);
	}
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Collect(IDX_TYPE maxNeighbors, IDXARR_TYPE& indices, const POINT_TYPE& center, TYPE radius) const
{
	Collect(maxNeighbors, indices, AABB_TYPE(center, radius));
} // Collect
/*----------------------------------------------------------------*/


// walk through the tree and collect visible indices
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename FTYPE, int FDIMS, typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_Traverse(const CELL_TYPE& cell, TYPE radius, const TFrustum<FTYPE,FDIMS>& frustum, INSERTER& inserter) const
{
	ASSERT(!cell.IsLeaf());
	switch (frustum.Classify(cell.GetAabb(radius))) {
	case CLIPPED: {
		const TYPE childRadius = radius / TYPE(2);
		for (int i=0; i<CELL_TYPE::numChildren; ++i) {
			const CELL_TYPE& childCell = cell.m_child[i];
			if (childCell.IsLeaf()) {
				const AABB_TYPE childAabb(ComputeChildCenter(cell.GetCenter(), childRadius, i), childRadius);
				if (frustum.Classify(childAabb) != CULLED)
					inserter(m_indices.Begin()+childCell.Leaf().idxBegin, childCell.GetNumItems());
			} else {
				_Traverse(childCell, childRadius, frustum, inserter);
			}
		}
		break; }
	case VISIBLE: {
		for (int i=0; i<CELL_TYPE::numChildren; ++i)
			_CollectCells(cell.m_child[i], inserter);
		break; }
	}
}
// walk through the tree and collect visible leafs
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename FTYPE, int FDIMS, typename PARSER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_TraverseCells(CELL_TYPE& cell, TYPE radius, const TFrustum<FTYPE,FDIMS>& frustum, PARSER& parser)
{
	ASSERT(!cell.IsLeaf());
	switch (frustum.Classify(cell.GetAabb(radius))) {
	case CLIPPED: {
		const TYPE childRadius = radius / TYPE(2);
		for (int i=0; i<CELL_TYPE::numChildren; ++i) {
			const CELL_TYPE& childCell = cell.m_child[i];
			if (childCell.IsLeaf()) {
				const AABB_TYPE childAabb(ComputeChildCenter(cell.GetCenter(), childRadius, i), childRadius);
				if (frustum.Classify(childAabb) != CULLED)
					parser(childCell);
			} else {
				_TraverseCells(childCell, childRadius, frustum, parser);
			}
		}
		break; }
	case VISIBLE: {
		for (int i=0; i<CELL_TYPE::numChildren; ++i)
			_ParseCells(cell.m_child[i], parser);
		break; }
	}
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename FTYPE, int FDIMS, typename INSERTER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Traverse(const TFrustum<FTYPE,FDIMS>& frustum, INSERTER& inserter) const
{
	_Traverse(m_root, m_radius, frustum, inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename FTYPE, int FDIMS>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::Traverse(const TFrustum<FTYPE,FDIMS>& frustum, IDXARR_TYPE& indices) const
{
	_Traverse(m_root, m_radius, frustum, IndexInserter(indices));
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename FTYPE, int FDIMS, typename PARSER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::TraverseCells(const TFrustum<FTYPE,FDIMS>& frustum, PARSER& parser)
{
	_TraverseCells(m_root, m_radius, frustum, parser);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
template <typename FTYPE, int FDIMS>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::TraverseCells(const TFrustum<FTYPE,FDIMS>& frustum, CELLPTRARR_TYPE& leaves)
{
	_TraverseCells(m_root, m_radius, frustum, CellInserter(leaves));
} // Traverse
/*----------------------------------------------------------------*/


#ifndef _RELEASE
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::_GetDebugInfo(const CELL_TYPE& cell, unsigned nDepth, DEBUGINFO& info) const
{
	if (cell.IsLeaf()) {
		if (info.minDepth > nDepth)
			info.minDepth = nDepth;
		if (info.maxDepth < nDepth)
			info.maxDepth = nDepth;
		info.avgDepth += nDepth;
		info.numLeaves++;
		return;
	}
	nDepth++;
	info.numNodes++;
	for (int i=0; i<CELL_TYPE::numChildren; ++i)
		_GetDebugInfo(cell.m_child[i], nDepth, info);
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::GetDebugInfo(DEBUGINFO* pInfo, bool bPrintStats) const
{
	DEBUGINFO localInfo;
	DEBUGINFO& info = (pInfo ? *pInfo : localInfo);
	info.Init();
	_GetDebugInfo(m_root, 0, info);
	info.avgDepth /= info.numLeaves;
	info.numItems  = GetNumItems();
	info.numNodes += info.numLeaves;
	info.memStruct = info.numNodes*sizeof(CELL_TYPE) + sizeof(TOctree);
	info.memItems  = sizeof(IDX_TYPE)*info.numItems;
	info.memSize   = info.memStruct + info.memItems;
	if (pInfo == NULL || bPrintStats)
		LogDebugInfo(info);
} // GetDebugInfo
/*----------------------------------------------------------------*/

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE, int SIZE, int NOM, int DENOM>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE,SIZE,NOM,DENOM>::LogDebugInfo(const DEBUGINFO& info)
{
	//VERBOSE("NoItems: %d; Mem %s; MemItems %s; MemStruct %s; AvgMemStruct %.2f%%%%; NoNodes %d; NoLeaf %d; AvgLeaf %.2f%%%%; AvgDepth %.2f; MinDepth %d; MaxDepth %d",
	VERBOSE("NumItems %d; Mem %s (%s items, %s struct - %.2f%%%%); NumNodes %d (leaves %d - %.2f%%%%); Depth %.2f (%d min, %d max)",
		info.numItems,
		Util::formatBytes(info.memSize).c_str(), Util::formatBytes(info.memItems).c_str(), Util::formatBytes(info.memStruct).c_str(), double(info.memStruct)*100.0/info.memSize,
		info.numNodes, info.numLeaves, float(info.numLeaves*100)/info.numNodes,
		info.avgDepth, info.minDepth, info.maxDepth);
} // LogDebugInfo
/*----------------------------------------------------------------*/

// if everything works fine, this function should return true
template <typename TYPE, int DIMS>
inline bool OctreeTest(unsigned iters, unsigned maxItems=1000, bool bRandom=true) {
	STATIC_ASSERT(DIMS > 0 && DIMS <= 3);
	srand(bRandom ? (unsigned)time(NULL) : 0);
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT_TYPE;
	typedef SEACAVE::cList<POINT_TYPE,const POINT_TYPE&,0> TestArr;
	typedef TOctree<TestArr,TYPE,DIMS,uint32_t,16,10,1> TestTree;
	const TYPE ptMinData[] = {0,0,0}, ptMaxData[] = {640,480,240};
	typename TestTree::AABB_TYPE aabb;
	aabb.Set(Eigen::Map<const POINT_TYPE>(ptMinData), Eigen::Map<const POINT_TYPE>(ptMaxData));
	aabb.Enlarge(ZEROTOLERANCE<TYPE>()*TYPE(10));
	unsigned nTotalMatches = 0;
	unsigned nTotalMissed = 0;
	unsigned nTotalExtra = 0;
	#ifndef _RELEASE
	typename TestTree::DEBUGINFO_TYPE totalInfo;
	totalInfo.Init();
	#endif
	for (unsigned iter=0; iter<iters; ++iter) {
		// generate random items
		const unsigned elems = maxItems/10+RAND()%maxItems;
		TestArr items(elems);
		FOREACH(i, items)
			for (int j=0; j<DIMS; ++j)
				items[i](j) = RAND()%ROUND2INT(ptMaxData[j]);
		// random query point
		POINT_TYPE pt;
		pt(0) = RAND()%ROUND2INT(ptMaxData[0]);
		if (DIMS > 1) pt(1) = RAND()%ROUND2INT(ptMaxData[1]);
		if (DIMS > 2) pt(2) = RAND()%ROUND2INT(ptMaxData[2]);
		const TYPE radius(TYPE(3+RAND()%30));
		// build octree and find interest items
		TestTree tree(items, aabb);
		typename TestTree::IDXARR_TYPE indices;
		tree.Collect(indices, pt, radius);
		// find interest items by brute force
		typename TestTree::IDXARR_TYPE trueIndices;
		#if 1
		// use square bound
		typename TestTree::AABB_TYPE aabbQuery(pt, radius);
		FOREACH(i, items)
			if (aabbQuery.Intersects(items[i]))
				trueIndices.Insert(i);
		#else
		// use circle bound
		FOREACH(i, items)
			if ((items[i]-pt).norm() < radius)
				trueIndices.Insert(i);
		#endif
		// compare results
		unsigned nMatches = 0;
		FOREACH(i, trueIndices) {
			const IDX idx = trueIndices[i];
			FOREACH(j, indices) {
				if (indices[j] == idx) {
					++nMatches;
					break;
				}
			}
		}
		nTotalMatches += nMatches;
		nTotalMissed += trueIndices.GetSize()-nMatches;
		nTotalExtra += indices.GetSize()-nMatches;
		#ifndef _RELEASE
		// print stats
		typename TestTree::DEBUGINFO_TYPE info;
		tree.GetDebugInfo(&info);
		totalInfo += info;
		#endif
	}
	#ifndef _RELEASE
	TestTree::LogDebugInfo(totalInfo);
	VERBOSE("Test %s (TotalMissed %d, TotalExtra %d)", (nTotalMissed == 0 && nTotalExtra == 0 ? "successful" : "FAILED"), nTotalMissed, nTotalExtra);
	#endif
	return (nTotalMissed == 0 && nTotalExtra == 0);
}
#endif
