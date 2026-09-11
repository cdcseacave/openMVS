////////////////////////////////////////////////////////////////////
// OctreeLOD.inl
//
// Copyright 2025 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)


// S T R U C T S ///////////////////////////////////////////////////

template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
size_t TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::Node::GetNumItemsHeld() const
{
	size_t count = size;
	if (!IsLeaf()) {
		for (int i = 0; i < (2<<(DIMS-1)); ++i) {
			if (children[i])
				count += children[i]->GetNumItemsHeld();
		}
	}
	return count;
}
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Functor>
inline TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::TOctreeLOD(const ITEMARR_TYPE& items, Functor subsample, unsigned maxDepth)
	: m_items(NULL), m_radius(0), m_maxDepth(0), m_totalNodes(0), m_spacing(0)
{
	Insert(items, subsample, maxDepth);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Functor>
inline TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::TOctreeLOD(const ITEMARR_TYPE& items, const AABB_TYPE& aabb, Functor subsample, unsigned maxDepth)
	: m_items(NULL), m_radius(0), m_maxDepth(0), m_totalNodes(0), m_spacing(0)
{
	Insert(items, aabb, subsample, maxDepth);
}
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
inline void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::Release()
{
	m_indices.Release();
	m_root = Node();
	m_items = NULL;
	m_center = POINT_TYPE::Zero();
	m_radius = 0;
	m_maxDepth = 0;
	m_totalNodes = 0;
	m_spacing = 0;
}
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Functor>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::Insert(const ITEMARR_TYPE& items, Functor subsample, unsigned maxDepth)
{
	ASSERT(!items.IsEmpty());
	ASSERT(sizeof(POINT_TYPE) == sizeof(typename ITEMARR_TYPE::Type));
	AABB_TYPE aabb((const POINT_TYPE*)items.data(), items.size());
	aabb.Enlarge(ZEROTOLERANCE<TYPE>() * TYPE(10));
	Insert(items, aabb, subsample, maxDepth);
}

template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Functor>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::Insert(const ITEMARR_TYPE& items, const AABB_TYPE& aabb, Functor subsample, unsigned maxDepth)
{
	Release();
	m_items = items.data();

	// compute root center and radius from AABB
	m_center = aabb.GetCenter();
	m_radius = aabb.GetSize().maxCoeff() / Type(2);

	// compute root-level spacing
	m_spacing = m_radius * Type(2) / TYPE(128);

	// pre-allocate shared index array
	m_indices.Reserve((IDX_TYPE)items.size());

	// create initial candidate list (all point indices)
	IDXARR_TYPE candidates;
	candidates.Reserve((IDX_TYPE)items.size());
	for (IDX_TYPE i = 0; i < (IDX_TYPE)items.size(); ++i)
		candidates.Insert(i);

	// build recursively
	_Insert(m_root, candidates, m_center, m_radius, 0, maxDepth, subsample);
}
/*----------------------------------------------------------------*/


// recursive LOD octree build
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Functor>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::_Insert(Node& node, IDXARR_TYPE& candidateIndices,
	const POINT_TYPE& center, TYPE radius, unsigned depth, unsigned maxDepth, Functor& subsample)
{
	++m_totalNodes;
	if (depth > m_maxDepth)
		m_maxDepth = depth;

	if (candidateIndices.empty()) {
		node.idxBegin = (IDX_TYPE)m_indices.size();
		node.size = 0;
		return;
	}

	if (depth >= maxDepth) {
		node.idxBegin = (IDX_TYPE)m_indices.size();
		node.size = (SIZE_TYPE)candidateIndices.size();
		m_indices.Join(candidateIndices.data(), candidateIndices.size());
		return;
	}

	// compute AABB on-the-fly for the subsample functor
	const AABB_TYPE aabb(center, radius);
	IDXARR_TYPE selected;
	subsample(selected, candidateIndices, m_items, aabb, depth);

	if (selected.size() >= candidateIndices.size()) {
		node.idxBegin = (IDX_TYPE)m_indices.size();
		node.size = (SIZE_TYPE)candidateIndices.size();
		m_indices.Join(candidateIndices.data(), candidateIndices.size());
		return;
	}

	// append selected indices to the shared array
	node.idxBegin = (IDX_TYPE)m_indices.size();
	node.size = (SIZE_TYPE)selected.size();
	m_indices.Join(selected.data(), selected.size());

	// build selected set for fast lookup
	std::unordered_set<IDX_TYPE> selectedSet;
	selectedSet.reserve(selected.size());
	for (IDX_TYPE i = 0; i < (IDX_TYPE)selected.size(); ++i)
		selectedSet.insert(selected[i]);
	selected.Release();

	// partition remaining into child octants
	IDXARR_TYPE childIndices[numChildren];
	for (IDX_TYPE i = 0; i < (IDX_TYPE)candidateIndices.size(); ++i) {
		const IDX_TYPE idx = candidateIndices[i];
		if (selectedSet.count(idx))
			continue;
		const unsigned octant = ComputeChild(reinterpret_cast<const POINT_TYPE&>(m_items[idx]), center);
		childIndices[octant].Insert(idx);
	}

	// recurse into non-empty children
	const TYPE childRadius = radius / TYPE(2);
	for (unsigned i = 0; i < numChildren; ++i) {
		if (childIndices[i].empty())
			continue;
		node.childMask |= (uint8_t)(1u << i);
		node.children[i] = std::make_unique<Node>();
		_Insert(*node.children[i], childIndices[i], ComputeChildCenter(center, childRadius, i), childRadius, depth + 1, maxDepth, subsample);
	}
}
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
inline unsigned TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::ComputeChild(const POINT_TYPE& item, const POINT_TYPE& center)
{
	unsigned idx = 0;
	if (item[0] >= center[0])
		idx |= (1<<0);
	if (DIMS > 1)
	if (item[1] >= center[1])
		idx |= (1<<1);
	if (DIMS > 2)
	if (item[2] >= center[2])
		idx |= (1<<2);
	return idx;
}

// compute child center from parent center, child radius and octant index
// (matches TOctree::CELL_TYPE::ComputeChildCenter)
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
inline typename TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::POINT_TYPE
TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::ComputeChildCenter(const POINT_TYPE& center, TYPE childRadius, unsigned idxChild)
{
	POINT_TYPE childCenter(center);
	for (int d = 0; d < DIMS; ++d)
		childCenter[d] += (idxChild & (1u << d)) ? childRadius : -childRadius;
	return childCenter;
}
/*----------------------------------------------------------------*/


// breadth-first traversal passing center+radius to visitor
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Visitor>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::TraverseBFS(Visitor&& visitor) const
{
	if (m_items == NULL)
		return;
	struct QueueEntry {
		const Node* node;
		POINT_TYPE center;
		TYPE radius;
	};
	std::queue<QueueEntry> queue;
	queue.push({&m_root, m_center, m_radius});
	while (!queue.empty()) {
		const QueueEntry entry = queue.front();
		queue.pop();
		visitor(*entry.node, entry.center, entry.radius);
		if (!entry.node->IsLeaf()) {
			const TYPE childRadius = entry.radius / TYPE(2);
			for (unsigned i = 0; i < numChildren; ++i) {
				if (entry.node->children[i])
					queue.push({entry.node->children[i].get(), ComputeChildCenter(entry.center, childRadius, i), childRadius});
			}
		}
	}
}
/*----------------------------------------------------------------*/


// depth-first traversal passing center+radius to visitor
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Visitor>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::TraverseDFS(Visitor&& visitor) const
{
	if (m_items == NULL)
		return;
	_TraverseDFS(m_root, m_center, m_radius, visitor);
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
template <typename Visitor>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::_TraverseDFS(const Node& node, const POINT_TYPE& center, TYPE radius, Visitor& visitor) const
{
	visitor(node, center, radius);
	if (!node.IsLeaf()) {
		const TYPE childRadius = radius / TYPE(2);
		for (unsigned i = 0; i < numChildren; ++i) {
			if (node.children[i])
				_TraverseDFS(*node.children[i], ComputeChildCenter(center, childRadius, i), childRadius, visitor);
		}
	}
}
/*----------------------------------------------------------------*/


template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::GetDebugInfo(DEBUGINFO* pInfo, bool bPrintStats) const
{
	DEBUGINFO info;
	info.Init();
	if (m_items != NULL)
		_GetDebugInfo(m_root, 0, info);
	if (info.totalNodes > 0)
		info.avgPointsPerNode = (float)info.totalPoints / info.totalNodes;
	if (pInfo)
		*pInfo = info;
	if (bPrintStats) {
		VERBOSE("OctreeLOD: %zu nodes (%zu internal, %zu leaves), %zu points, depth %u-%u, avg %.1f pts/node",
			info.totalNodes, info.internalNodes, info.leafNodes, info.totalPoints,
			info.minDepth, info.maxDepth, info.avgPointsPerNode);
	}
}
template <typename ITEMARR_TYPE, typename TYPE, int DIMS>
void TOctreeLOD<ITEMARR_TYPE,TYPE,DIMS>::_GetDebugInfo(const Node& node, unsigned depth, DEBUGINFO& info) const
{
	++info.totalNodes;
	info.totalPoints += node.size;
	if (node.IsLeaf()) {
		++info.leafNodes;
		if (depth < info.minDepth) info.minDepth = depth;
		if (depth > info.maxDepth) info.maxDepth = depth;
	} else {
		++info.internalNodes;
		for (unsigned i = 0; i < numChildren; ++i) {
			if (node.children[i])
				_GetDebugInfo(*node.children[i], depth + 1, info);
		}
	}
}
/*----------------------------------------------------------------*/


// Test function for TOctreeLOD (matches OctreeTest pattern from Octree.inl)
template <typename TYPE, int DIMS>
inline bool OctreeLODTest(unsigned iters, unsigned maxItems=10000, bool bRandom=true) {
	static_assert(DIMS > 0 && DIMS <= 3);
	srand(bRandom ? (unsigned)time(NULL) : 0);
	typedef Eigen::Matrix<TYPE,DIMS,1> POINT_TYPE;
	typedef CLISTDEF0(POINT_TYPE) TestArr;
	typedef TOctreeLOD<TestArr,TYPE,DIMS> TestTree;
	typedef GridSubsample<TestArr,TYPE,DIMS> TestSubsample;
	const TYPE ptMaxData[] = {640,480,240};
	unsigned nTotalErrors = 0;
	for (unsigned iter = 0; iter < iters; ++iter) {
		// generate random items
		const unsigned elems = maxItems/10 + RAND()%maxItems;
		TestArr items(elems);
		FOREACH(i, items)
			for (int j = 0; j < DIMS; ++j)
				items[i](j) = static_cast<TYPE>(RAND()%ROUND2INT(ptMaxData[j]));

		// build LOD octree with grid subsampler
		const unsigned testMaxDepth = 10;
		TestTree tree(items, TestSubsample(32), testMaxDepth);

		// 1. Verify partition completeness: sum of all node sizes == input count
		const auto& indices = tree.GetIndexArr();
		if (indices.size() != items.size()) {
			VERBOSE("ERROR: OctreeLODTest partition completeness: %zu != %zu", indices.size(), (size_t)items.size());
			++nTotalErrors;
			continue;
		}

		// 2. Verify index validity and uniqueness
		std::vector<bool> seen(items.size(), false);
		bool hasDuplicate = false;
		bool hasInvalid = false;
		for (size_t i = 0; i < indices.size(); ++i) {
			const auto idx = indices[i];
			if (idx >= items.size()) {
				hasInvalid = true;
				break;
			}
			if (seen[idx]) {
				hasDuplicate = true;
				break;
			}
			seen[idx] = true;
		}
		if (hasInvalid) {
			VERBOSE("ERROR: OctreeLODTest invalid index found");
			++nTotalErrors;
			continue;
		}
		if (hasDuplicate) {
			VERBOSE("ERROR: OctreeLODTest duplicate index found");
			++nTotalErrors;
			continue;
		}

		// 3. Verify all indices are present (no gaps)
		for (size_t i = 0; i < items.size(); ++i) {
			if (!seen[i]) {
				VERBOSE("ERROR: OctreeLODTest missing index %zu", i);
				++nTotalErrors;
				hasInvalid = true;
				break;
			}
		}
		if (hasInvalid)
			continue;

		// 4. Verify traversal consistency: BFS and DFS visit same node count
		size_t bfsNodes = 0, bfsPoints = 0;
		tree.TraverseBFS([&](const typename TestTree::Node& node, const POINT_TYPE& /*center*/, TYPE /*radius*/) {
			++bfsNodes;
			bfsPoints += node.GetNumItems();
		});
		size_t dfsNodes = 0, dfsPoints = 0;
		tree.TraverseDFS([&](const typename TestTree::Node& node, const POINT_TYPE& /*center*/, TYPE /*radius*/) {
			++dfsNodes;
			dfsPoints += node.GetNumItems();
		});
		if (bfsNodes != tree.GetTotalNodes() || dfsNodes != tree.GetTotalNodes()) {
			VERBOSE("ERROR: OctreeLODTest traversal node count mismatch: BFS=%zu DFS=%zu Total=%zu",
				bfsNodes, dfsNodes, tree.GetTotalNodes());
			++nTotalErrors;
			continue;
		}
		if (bfsPoints != items.size() || dfsPoints != items.size()) {
			VERBOSE("ERROR: OctreeLODTest traversal point count mismatch: BFS=%zu DFS=%zu expected=%zu",
				bfsPoints, dfsPoints, (size_t)items.size());
			++nTotalErrors;
			continue;
		}

		// 5. Verify GetDebugInfo consistency
		typename TestTree::DEBUGINFO_TYPE info;
		tree.GetDebugInfo(&info);
		if (info.totalPoints != items.size()) {
			VERBOSE("ERROR: OctreeLODTest debug info point count mismatch: %zu != %zu",
				info.totalPoints, (size_t)items.size());
			++nTotalErrors;
			continue;
		}
		if (info.totalNodes != tree.GetTotalNodes()) {
			VERBOSE("ERROR: OctreeLODTest debug info node count mismatch");
			++nTotalErrors;
			continue;
		}
		if (info.maxDepth > testMaxDepth) {
			VERBOSE("ERROR: OctreeLODTest depth %u exceeds max %u", info.maxDepth, testMaxDepth);
			++nTotalErrors;
			continue;
		}

		// 6. Verify GetAABB contains all points
		const auto aabb = tree.GetAABB();
		bool allContained = true;
		FOREACH(i, items) {
			if (!aabb.Intersects(reinterpret_cast<const typename TestTree::POINT_TYPE&>(items[i]))) {
				allContained = false;
				break;
			}
		}
		if (!allContained) {
			VERBOSE("ERROR: OctreeLODTest AABB does not contain all points");
			++nTotalErrors;
			continue;
		}

		// 7. Test with custom lambda subsampler (take every Nth point)
		TestTree tree2;
		tree2.Insert(items, [](typename TestTree::IDXARR_TYPE& sel, const typename TestTree::IDXARR_TYPE& cand,
		                       const typename TestTree::ITEM_TYPE* /*items*/, const typename TestTree::AABB_TYPE& /*aabb*/, unsigned depth) {
			const unsigned stride = 1u << std::min(depth, 4u);
			for (size_t i = 0; i < cand.size(); i += stride)
				sel.Insert(cand[(typename TestTree::IDX_TYPE)i]);
		}, testMaxDepth);
		if (tree2.GetIndexArr().size() != items.size()) {
			VERBOSE("ERROR: OctreeLODTest custom functor partition failed: %zu != %zu",
				tree2.GetIndexArr().size(), (size_t)items.size());
			++nTotalErrors;
			continue;
		}

	}
	#ifndef _RELEASE
	VERBOSE("OctreeLOD test %s (%u errors in %u iterations)", (nTotalErrors == 0 ? "successful" : "FAILED"), nTotalErrors, iters);
	#endif
	return (nTotalErrors == 0);
}
/*----------------------------------------------------------------*/
