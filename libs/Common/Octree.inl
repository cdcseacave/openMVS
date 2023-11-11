////////////////////////////////////////////////////////////////////
// Octree.inl
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// D E F I N E S ///////////////////////////////////////////////////

#ifdef _USE_OPENMP
// minimum number of polygons for which we do multi-threading
#define OCTREE_MIN_ITEMS_MINTHREAD 1024*2
#endif


// S T R U C T S ///////////////////////////////////////////////////

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::CELL_TYPE()
	:
	m_child(NULL)
{
} // constructor
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::~CELL_TYPE()
{
	delete[] m_child;
} // destructor
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::Release()
{
	delete[] m_child;
	m_child = NULL;
} // Release
// swap the two octrees
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::Swap(CELL_TYPE& rhs)
{
	std::swap(m_child, rhs.m_child);
	if (IsLeaf())
		std::swap(m_leaf, rhs.m_leaf);
	else
		std::swap(m_node, rhs.m_node);
} // Swap
/*----------------------------------------------------------------*/


// compute item's index corresponding to the containing cell
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline unsigned TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::ComputeChild(const POINT_TYPE& item) const
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

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::ComputeCenter(POINT_TYPE centers[])
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

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline typename TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::POINT_TYPE TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::ComputeChildCenter(const POINT_TYPE& center, TYPE radius, unsigned idxChild)
{
	struct CENTERARR_TYPE {
		POINT_TYPE child[CELL_TYPE::numChildren];
		inline CENTERARR_TYPE() { CELL_TYPE::ComputeCenter(child); }
	};
	static const CENTERARR_TYPE centers;
	return center + centers.child[idxChild] * radius;
} // ComputeChildCenter
/*----------------------------------------------------------------*/


// count the number of items contained by the given octree-cell
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
size_t TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CELL_TYPE::GetNumItemsHeld() const
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
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename Functor>
inline TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::TOctree(const ITEMARR_TYPE& items, Functor split)
{
	Insert(items, split);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename Functor>
inline TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::TOctree(const ITEMARR_TYPE& items, const AABB_TYPE& aabb, Functor split)
{
	Insert(items, aabb, split);
} // constructor
/*----------------------------------------------------------------*/


// destroy tree
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Release()
{
	m_indices.Release();
	m_root.Release();
} // Release
// swap the two octrees
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Swap(TOctree& rhs)
{
	std::swap(m_items, rhs.m_items);
	m_indices.Swap(rhs.m_indices);
	m_root.Swap(rhs.m_root);
	std::swap(m_radius, rhs.m_radius);
} // Swap
/*----------------------------------------------------------------*/


// add the given item to the tree
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename Functor, bool bForceSplit>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_Insert(CELL_TYPE& cell, const POINT_TYPE& center, TYPE radius, IDX_TYPE start, IDX_TYPE size, _InsertData<Functor>& insertData)
{
	ASSERT(size > 0);
	// if this child cell needs to be divided further
	if (bForceSplit || insertData.split(size, radius)) {
		// init node and proceed recursively
		ASSERT(cell.m_child == NULL);
		cell.m_child = new CELL_TYPE[CELL_TYPE::numChildren];
		cell.Node().center = center;
		struct ChildData {
			enum { ESTART=0, EEND=CELL_TYPE::numChildren, ESIZE=CELL_TYPE::numChildren*2, EALL=CELL_TYPE::numChildren*3};
			IDX_TYPE data[EALL];
			ChildData() { memset(data, 0, sizeof(IDX_TYPE)*EALL); }
			inline IDX_TYPE Start(unsigned i) const { return data[ESTART+i]; }
			inline IDX_TYPE& Start(unsigned i) { return data[ESTART+i]; }
			inline IDX_TYPE End(unsigned i) const { return data[EEND+i]; }
			inline IDX_TYPE& End(unsigned i) { return data[EEND+i]; }
			inline IDX_TYPE Size(unsigned i) const { return data[ESIZE+i]; }
			inline IDX_TYPE& Size(unsigned i) { return data[ESIZE+i]; }
		} childD;
		IDX_TYPE idx(start);
		for (IDX_TYPE i=0; i<size; ++i) {
			const unsigned idxChild(cell.ComputeChild(m_items[idx]));
			if (childD.Size(idxChild) == 0)
				childD.Start(idxChild) = idx;
			else
				insertData.successors[childD.End(idxChild)] = idx;
			childD.End(idxChild) = idx;
			++childD.Size(idxChild);
			idx = insertData.successors[idx];
		}
		ASSERT(idx == _InsertData<Functor>::NO_INDEX);
		const TYPE childRadius(radius / TYPE(2));
		for (unsigned i=0; i<CELL_TYPE::numChildren; ++i) {
			CELL_TYPE& child = cell.m_child[i];
			if (childD.Size(i) == 0) {
				child.Leaf().idxBegin = m_indices.size();
				child.Leaf().size = 0;
				continue;
			}
			insertData.successors[childD.End(i)] = _InsertData<Functor>::NO_INDEX; // mark the end of child successors
			const POINT_TYPE childCenter(CELL_TYPE::ComputeChildCenter(center, childRadius, i));
			_Insert<Functor,false>(child, childCenter, childRadius, childD.Start(i), childD.Size(i), insertData);
		}
	} else {
		// init leaf
		cell.Leaf().idxBegin = m_indices.size();
		cell.Leaf().size = (SIZE_TYPE)size;
		for (IDX_TYPE idx=start; idx!=_InsertData<Functor>::NO_INDEX; idx=insertData.successors[idx])
			m_indices.push_back(idx);
	}
} // _Insert
/*----------------------------------------------------------------*/

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename Functor>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Insert(const ITEMARR_TYPE& items, const AABB_TYPE& aabb, Functor split)
{
	Release();
	m_items = items.data();
	// create root as node, even if we do not need to divide
	m_indices.Reserve(items.size());
	// divide cell
	const POINT_TYPE center = aabb.GetCenter();
	m_radius = aabb.GetSize().maxCoeff()/Type(2);
	// single connected list of next item indices
	_InsertData<Functor> insertData = {items.size(), split};
	std::iota(insertData.successors.begin(), insertData.successors.end(), IDX_TYPE(1));
	insertData.successors.back() = _InsertData<Functor>::NO_INDEX;
	// setup each cell
	_Insert<Functor,true>(m_root, center, m_radius, 0, items.size(), insertData);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename Functor>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Insert(const ITEMARR_TYPE& items, Functor split)
{
	ASSERT(!items.IsEmpty());
	ASSERT(sizeof(POINT_TYPE) == sizeof(typename ITEMARR_TYPE::Type));
	AABB_TYPE aabb((const POINT_TYPE*)items.data(), items.size());
	aabb.Enlarge(ZEROTOLERANCE<TYPE>()*TYPE(10));
	Insert(items, aabb, split);
} // Insert
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CollectCells(const CELL_TYPE& cell, INSERTER& inserter) const
{
	if (cell.IsLeaf()) {
		inserter(m_indices.data()+cell.GetFirstItemIdx(), cell.GetNumItems());
		return;
	}
	for (int i=0; i<CELL_TYPE::numChildren; ++i)
		CollectCells(cell.m_child[i], inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename PARSER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_ParseCells(CELL_TYPE& cell, TYPE radius, PARSER& parser)
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
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::CollectCells(INSERTER& inserter) const
{
	CollectCells(m_root, inserter);
}
// calls parser for each leaf of the octree (the CELL_TYPE operator has to be defined)
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename PARSER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::ParseCells(PARSER& parser)
{
	_ParseCells(m_root, m_radius, parser);
}
/*----------------------------------------------------------------*/


// find all items contained by the given bounding box
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_Collect(const CELL_TYPE& cell, const AABB_TYPE& aabb, INSERTER& inserter) const
{
	if (cell.IsLeaf()) {
		// add all items contained by the bounding-box
		for (IDX_TYPE i=0; i<cell.Leaf().size; ++i) {
			const IDX_TYPE idx = m_indices[cell.Leaf().idxBegin + i];
			if (aabb.Intersects(m_items[idx]))
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
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename INSERTER, typename COLLECTOR>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_Collect(const CELL_TYPE& cell, TYPE radius, const COLLECTOR& collector, INSERTER& inserter) const
{
	ASSERT(!cell.IsLeaf());
	const TYPE childRadius = radius / TYPE(2);
	for (int i=0; i<CELL_TYPE::numChildren; ++i) {
		const CELL_TYPE& childCell = cell.m_child[i];
		if (childCell.IsLeaf()) {
			if (collector.Intersects(CELL_TYPE::ComputeChildCenter(cell.GetCenter(), childRadius, i), childRadius))
				inserter(m_indices.data()+childCell.GetFirstItemIdx(), childCell.GetNumItems());
		} else {
			if (collector.Intersects(childCell.Node().center, childRadius))
				_Collect(childCell, childRadius, collector, inserter);
		}
	}
} // Collect
/*----------------------------------------------------------------*/

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename INSERTER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(INSERTER& inserter, const AABB_TYPE& aabb) const
{
	_Collect(m_root, aabb, inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(IDXARR_TYPE& indices, const AABB_TYPE& aabb) const
{
	IndexInserter inserter(indices);
	_Collect(m_root, aabb, inserter);
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename INSERTER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(INSERTER& inserter, const POINT_TYPE& center, TYPE radius) const
{
	_Collect(m_root, AABB_TYPE(center, radius), inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(IDXARR_TYPE& indices, const POINT_TYPE& center, TYPE radius) const
{
	IndexInserter inserter(indices);
	_Collect(m_root, AABB_TYPE(center, radius), inserter);
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename INSERTER, typename COLLECTOR>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(INSERTER& inserter, const COLLECTOR& collector) const
{
	_Collect(m_root, m_radius, collector, inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename COLLECTOR>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(IDXARR_TYPE& indices, const COLLECTOR& collector) const
{
	IndexInserter inserter(indices);
	_Collect(m_root, m_radius, collector, inserter);
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(IDX_TYPE maxNeighbors, IDXARR_TYPE& indices, const AABB_TYPE& aabb) const
{
	_Collect(m_root, aabb, IndexInserter(indices));
	if (indices.size() > maxNeighbors) {
		// keep only the closest neighbors
		typedef TIndexScore<IDX_TYPE,TYPE> ItemIndexScore;
		typedef cList<ItemIndexScore, const ItemIndexScore&, 0> ItemIndexScoreArr;
		ItemIndexScoreArr indexscores(indices.size());
		const POINT_TYPE center(aabb.GetCenter());
		FOREACH(i, indices) {
			const IDX_TYPE& idx = indices[i];
			const TYPE score(-(center-m_items[idx]).squaredNorm());
			indexscores[i] = ItemIndexScore(idx,score);
		}
		indices.Empty();
		indexscores.Sort();
		for (IDX_TYPE i=0; i<maxNeighbors; ++i)
			indices.Insert(indexscores[i].idx);
	}
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Collect(IDX_TYPE maxNeighbors, IDXARR_TYPE& indices, const POINT_TYPE& center, TYPE radius) const
{
	Collect(maxNeighbors, indices, AABB_TYPE(center, radius));
} // Collect
/*----------------------------------------------------------------*/


// walk through the tree and collect visible indices
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename FTYPE, int FDIMS, typename INSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_Traverse(const CELL_TYPE& cell, TYPE radius, const TFrustum<FTYPE,FDIMS>& frustum, INSERTER& inserter) const
{
	ASSERT(!cell.IsLeaf());
	switch (frustum.Classify(cell.GetAabb(radius))) {
	case CLIPPED: {
		const TYPE childRadius = radius / TYPE(2);
		for (int i=0; i<CELL_TYPE::numChildren; ++i) {
			const CELL_TYPE& childCell = cell.m_child[i];
			if (childCell.IsLeaf()) {
				const AABB_TYPE childAabb(CELL_TYPE::ComputeChildCenter(cell.GetCenter(), childRadius, i), childRadius);
				if (frustum.Classify(childAabb) != CULLED)
					inserter(m_indices.data()+childCell.GetFirstItemIdx(), childCell.GetNumItems());
			} else {
				_Traverse(childCell, childRadius, frustum, inserter);
			}
		}
		break; }
	case VISIBLE: {
		for (int i=0; i<CELL_TYPE::numChildren; ++i)
			CollectCells(cell.m_child[i], inserter);
		break; }
	}
}
// walk through the tree and collect visible leafs
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename FTYPE, int FDIMS, typename PARSER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_TraverseCells(CELL_TYPE& cell, TYPE radius, const TFrustum<FTYPE,FDIMS>& frustum, PARSER& parser)
{
	ASSERT(!cell.IsLeaf());
	switch (frustum.Classify(cell.GetAabb(radius))) {
	case CLIPPED: {
		const TYPE childRadius = radius / TYPE(2);
		for (int i=0; i<CELL_TYPE::numChildren; ++i) {
			const CELL_TYPE& childCell = cell.m_child[i];
			if (childCell.IsLeaf()) {
				const AABB_TYPE childAabb(CELL_TYPE::ComputeChildCenter(cell.GetCenter(), childRadius, i), childRadius);
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

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename FTYPE, int FDIMS, typename INSERTER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Traverse(const TFrustum<FTYPE,FDIMS>& frustum, INSERTER& inserter) const
{
	_Traverse(m_root, m_radius, frustum, inserter);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename FTYPE, int FDIMS>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::Traverse(const TFrustum<FTYPE,FDIMS>& frustum, IDXARR_TYPE& indices) const
{
	_Traverse(m_root, m_radius, frustum, IndexInserter(indices));
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename FTYPE, int FDIMS, typename PARSER>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::TraverseCells(const TFrustum<FTYPE,FDIMS>& frustum, PARSER& parser)
{
	_TraverseCells(m_root, m_radius, frustum, parser);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename FTYPE, int FDIMS>
inline void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::TraverseCells(const TFrustum<FTYPE,FDIMS>& frustum, CELLPTRARR_TYPE& leaves)
{
	_TraverseCells(m_root, m_radius, frustum, CellInserter(leaves));
} // Traverse
/*----------------------------------------------------------------*/

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename AREAESTIMATOR, typename CHUNKINSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_SplitVolume(const CELL_TYPE& parentCell, TYPE parentRadius, unsigned idxChild, float maxArea, AREAESTIMATOR& areaEstimator, CHUNKINSERTER& chunkInserter, const UnsignedArr& indices)
{
	ASSERT(!indices.empty());
	typedef std::pair<UnsignedArr,UnsignedArr> PairIndices;
	struct GenerateSamples {
		const UnsignedArr& indices;
		const unsigned numSamples;
		const unsigned halfSamples;
		const unsigned numCommonAxis;
		POINT_TYPE centers[8];
		cList<PairIndices> arrHalfIndices;
		GenerateSamples(const UnsignedArr& _indices)
			: indices(_indices), numSamples((unsigned)indices.size()), halfSamples(numSamples/2), numCommonAxis(halfSamples==4?1:2), arrHalfIndices(0, numSamples)
		{
			ASSERT(indices.size()%2 == 0 && indices.IsSorted());
			ASSERT(halfSamples == 4 || halfSamples == 2);
			CELL_TYPE::ComputeCenter(centers);
			UnsignedArr samples(halfSamples);
			for (unsigned hs=0; hs<halfSamples; ++hs) {
				samples[0] = hs;
				GenerateIndices(1, samples);
			}
		}
		void GenerateIndices(unsigned idxSample, UnsignedArr& samples) {
			if (idxSample == samples.size()) {
				InsertIndices(samples);
				return;
			}
			for (unsigned i=samples[idxSample-1]+1; i<numSamples; ++i) {
				samples[idxSample] = i;
				GenerateIndices(idxSample+1, samples);
			}
		}
		void InsertIndices(const UnsignedArr& samples) {
			UnsignedArr halfIndices(halfSamples);
			for (unsigned s=0; s<halfSamples; ++s)
				halfIndices[s] = indices[samples[s]];
			// check all samples have one/two common axis
			unsigned commonAxis(0);
			for (unsigned a=0; a<3; ++a) {
				unsigned na(1);
				for (unsigned s=1; s<halfSamples; ++s) {
					if (centers[halfIndices[0]][a] == centers[halfIndices[s]][a])
						++na;
				}
				if (na == halfSamples && ++commonAxis == numCommonAxis)
					goto CommonAxis;
			}
			return;
			CommonAxis:
			// check not complementary samples
			for (const PairIndices& pairHalfIndices: arrHalfIndices) {
				unsigned nCommon(0);
				for (unsigned s=0; s<halfSamples; ++s)
					if (halfIndices[s] == pairHalfIndices.second[s])
						++nCommon;
				if (nCommon == halfSamples)
					return;
			}
			// generate complementary indices
			ASSERT(halfIndices.IsSorted());
			UnsignedArr compHalfIndices(0, halfSamples);
			unsigned i(0);
			for (unsigned idx: indices) {
				if (i < halfSamples && idx == halfIndices[i])
					++i;
				else
					compHalfIndices.push_back(idx);
			}
			ASSERT(compHalfIndices.size() == halfSamples);
			ASSERT(compHalfIndices.IsSorted());
			arrHalfIndices.emplace_back(std::move(halfIndices), std::move(compHalfIndices));
		}
	};
	const CELL_TYPE& cell(parentCell.GetChild(idxChild));
	const TYPE radius(parentRadius / TYPE(2));
	// handle particular cases
	if (cell.IsLeaf()) {
		chunkInserter(parentCell, parentRadius, UnsignedArr{idxChild});
		return;
	}
	if (indices.size() == 1) {
		_SplitVolume(cell, radius, indices.front(), maxArea, areaEstimator, chunkInserter);
		return;
	}
	if (indices.size() == 2) {
		_SplitVolume(cell, radius, indices.front(), maxArea, areaEstimator, chunkInserter);
		_SplitVolume(cell, radius, indices.back(), maxArea, areaEstimator, chunkInserter);
		return;
	}
	// measure surface area for each child
	float childArea[8], totalArea(0);
	for (unsigned c=0; c<8; ++c) {
		CollectCells(cell.GetChild(c), areaEstimator);
		totalArea += childArea[c] = areaEstimator.PopArea();
	}
	if (totalArea < maxArea*1.01f) {
		chunkInserter(parentCell, parentRadius, UnsignedArr{idxChild});
		return;
	}
	// check if all parts are over the limit
	unsigned numOverAreas(0);
	for (unsigned c: indices)
		if (childArea[c] == 0 || childArea[c] >= maxArea)
			++numOverAreas;
	if (numOverAreas == indices.size()) {
		for (unsigned c: indices)
			if (childArea[c] > 0)
				_SplitVolume(cell, radius, c, maxArea, areaEstimator, chunkInserter);
		return;
	}
	// split mesh children and retain the components with surface smaller than the given area
	const cList<PairIndices> halfIndices(std::move(GenerateSamples(indices).arrHalfIndices));
	IDX bestSplit(NO_ID);
	float bestArea(0);
	Point2f bestAs;
	FOREACH(idx, halfIndices) {
		const PairIndices& pairHalfIndices = halfIndices[idx];
		ASSERT(pairHalfIndices.first.size() == pairHalfIndices.second.size());
		Point2f as(Point2f::ZERO);
		for (unsigned i=0; i<pairHalfIndices.first.size(); ++i) {
			as[0] += childArea[pairHalfIndices.first[i]];
			as[1] += childArea[pairHalfIndices.second[i]];
		}
		for (unsigned i=0; i<2; ++i) {
			if (as[i] < maxArea && bestArea < as[i]) {
				bestArea = as[i];
				bestAs = as;
				bestSplit = idx;
			}
		}
	}
	if (bestSplit != NO_ID) {
		// store found clusters
		const PairIndices& pairHalfIndices = halfIndices[bestSplit];
		if (bestAs[0] < maxArea)
			chunkInserter(cell, radius, pairHalfIndices.first);
		else
			_SplitVolume(parentCell, parentRadius, idxChild, maxArea, areaEstimator, chunkInserter, pairHalfIndices.first);
		if (bestAs[1] < maxArea)
			chunkInserter(cell, radius, pairHalfIndices.second);
		else
			_SplitVolume(parentCell, parentRadius, idxChild, maxArea, areaEstimator, chunkInserter, pairHalfIndices.second);
		return;
	}
	// farther split each half into quarters
	if (halfIndices.front().first.size() == 4) {
		UnsignedArr bestQIndices[4];
		float bestArea(0);
		Eigen::Vector4f bestAs;
		for (const PairIndices& pairHalfIndices: halfIndices) {
			const cList<PairIndices> qIndicesFirst(std::move(GenerateSamples(pairHalfIndices.first).arrHalfIndices));
			const cList<PairIndices> qIndicesSecond(std::move(GenerateSamples(pairHalfIndices.second).arrHalfIndices));
			ASSERT(qIndicesFirst.size() == qIndicesSecond.size());
			FOREACH(q, qIndicesFirst) {
				const PairIndices& qFirst = qIndicesFirst[q];
				const PairIndices& qSecond = qIndicesSecond[q];
				Eigen::Vector4f as(Eigen::Vector4f::Zero());
				for (unsigned i=0; i<qFirst.first.size(); ++i) {
					as[0] += childArea[qFirst.first[i]];
					as[1] += childArea[qFirst.second[i]];
					as[2] += childArea[qSecond.first[i]];
					as[3] += childArea[qSecond.second[i]];
				}
				float area(0);
				for (unsigned i=0; i<4; ++i) {
					if (as[i] < maxArea)
						area += as[i];
				}
				if (bestArea < area) {
					bestArea = area;
					bestAs = as;
					bestQIndices[0] = qFirst.first;
					bestQIndices[1] = qFirst.second;
					bestQIndices[2] = qSecond.first;
					bestQIndices[3] = qSecond.second;
				}
			}
		}
		if (bestArea > 0) {
			// store found clusters
			for (unsigned i=0; i<4; ++i) {
				if (bestAs[i] < maxArea) {
					chunkInserter(cell, radius, bestQIndices[i]);
				} else {
					_SplitVolume(cell, radius, bestQIndices[i][0], maxArea, areaEstimator, chunkInserter);
					_SplitVolume(cell, radius, bestQIndices[i][1], maxArea, areaEstimator, chunkInserter);
				}
			}
			return;
		}
	}
	// split each child
	for (unsigned c: indices) {
		if (childArea[c] == 0)
			continue;
		if (childArea[c] < maxArea)
			chunkInserter(cell, radius, UnsignedArr{c});
		else
			_SplitVolume(cell, radius, c, maxArea, areaEstimator, chunkInserter);
	}
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
template <typename AREAESTIMATOR, typename CHUNKINSERTER>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::SplitVolume(float maxArea, AREAESTIMATOR& areaEstimator, CHUNKINSERTER& chunkInserter)
{
	CELL_TYPE parent;
	parent.m_child = new CELL_TYPE[1];
	parent.m_child[0].m_child = m_root.m_child;
	parent.m_child[0].Node() = m_root.Node();
	parent.Node().center = m_root.Node().center + POINT_TYPE::Constant(m_radius);
	_SplitVolume(parent, m_radius*TYPE(2), 0, maxArea, areaEstimator, chunkInserter);
	parent.m_child[0].m_child = NULL;
} // SplitVolume
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::_GetDebugInfo(const CELL_TYPE& cell, unsigned nDepth, DEBUGINFO& info) const
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

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::GetDebugInfo(DEBUGINFO* pInfo, bool bPrintStats) const
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

template <typename ITEMARR_TYPE, typename TYPE, int DIMS, typename DATA_TYPE>
void TOctree<ITEMARR_TYPE,TYPE,DIMS,DATA_TYPE>::LogDebugInfo(const DEBUGINFO& info)
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
	typedef CLISTDEF0(POINT_TYPE) TestArr;
	typedef TOctree<TestArr,TYPE,DIMS,uint32_t> TestTree;
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
				items[i](j) = static_cast<TYPE>(RAND()%ROUND2INT(ptMaxData[j]));
		// random query point
		POINT_TYPE pt;
		for (int j=0; j<DIMS; ++j)
			pt(j) = static_cast<TYPE>(RAND()%ROUND2INT(ptMaxData[j]));
		const TYPE radius(TYPE(3+RAND()%30));
		// build octree and find interest items
		TestTree tree(items, aabb, [](typename TestTree::IDX_TYPE size, typename TestTree::Type radius) {
			return size > 16 && radius > 10;
		});
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
			const typename TestTree::IDX_TYPE idx = trueIndices[i];
			FOREACH(j, indices) {
				if (indices[j] == idx) {
					++nMatches;
					break;
				}
			}
		}
		nTotalMatches += nMatches;
		nTotalMissed += (unsigned)trueIndices.size()-nMatches;
		nTotalExtra += (unsigned)indices.size()-nMatches;
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
