/*
* Mesh.cpp
*
* Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
* Please read <http://foxel.ch/license> for more information.
*
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This file is part of the FOXEL project <http://foxel.ch>.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*
*      You are required to attribute the work as explained in the "Usage and
*      Attribution" section of <http://foxel.ch/license>.
*/

#include "Common.h"
#include "Mesh.h"
// fix non-manifold vertices
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>
// VCG: mesh reconstruction post-processing
#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/hole.h>
// VCG: mesh simplification
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/selection.h>
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

// free all memory
void Mesh::Release()
{
	vertices.Release();
	faces.Release();
	vertexFaces.Release();
	vertexBoundary.Release();
} // Release
/*----------------------------------------------------------------*/


// extract array of triangles incident to each vertex
void Mesh::ListIncidenteFaces()
{
	vertexFaces.Empty();
	vertexFaces.Resize(vertices.GetSize());
	FOREACH(i, faces) {
		const Face& face = faces[i];
		for (uint32_t v=0; v<3; ++v) {
			ASSERT(vertexFaces[face[v]].Find(i) == IndexArr::NO_INDEX);
			vertexFaces[face[v]].Insert((FIndex)i);
		}
	}
}

// check each vertex if it is at the boundary or not
// (make sure you called ListIncidenteFaces() before)
void Mesh::ListBoundaryVertices()
{
	vertexBoundary.Empty();
	vertexBoundary.Resize(vertices.GetSize());
	vertexBoundary.Memset(0);
	typedef FaceCount VertCount;
	typedef std::unordered_map<VIndex,VertCount> VertCountMap;
	VertCountMap mapVerts; mapVerts.reserve(12*2);
	FOREACH(idxV, vertices) {
		const FaceIdxArr& vf = vertexFaces[idxV];
		// count how many times vertices in the first triangle ring are seen;
		// usually they are seen two times each as the vertex in not at the boundary
		// so there are two triangles (on the ring) containing same vertex
		ASSERT(mapVerts.empty());
		FOREACHPTR(pFaceIdx, vf) {
			const Face& face = faces[*pFaceIdx];
			for (int i=0; i<3; ++i) {
				const VIndex idx(face[i]);
				if (idx != idxV)
					++mapVerts[idx].count;
			}
		}
		for (const auto& vc: mapVerts) {
			ASSERT(vc.second.count == 1 || vc.second.count == 2);
			if (vc.second.count != 2) {
				vertexBoundary[idxV] = true;
				break;
			}
		}
		mapVerts.clear();
	}
}
/*----------------------------------------------------------------*/


#if 0
#define DEFINE_FACE_VERTS(n) \
	const Face& f##n = faces[componentFaces[n]]; \
	const uint32_t idx##n(Mesh::FindVertex(f##n, (VIndex)v)); \
	const VIndex v##n##1(f##n[(idx##n+1)%3]); \
	const VIndex v##n##2(f##n[(idx##n+2)%3])
#define IS_LOOP_FACE4(a, b, c, d) \
	(v##a##2 == v##b##1 && v##b##2 == v##c##1 && v##c##2 == v##d##1 && v##d##2 == v##a##1)
#define IS_LOOP_FACE3(a, b, c) \
	(v##a##2 == v##b##1 && v##b##2 == v##c##1 && v##c##2 == v##a##1)

namespace FIX_NONMANIFOLD {
typedef Mesh::VIndex VIndex;
typedef Mesh::FIndex FIndex;
typedef uint32_t Index;
struct Node;
struct Edge {
	Node* pPrev; // a node that points to this node
	Node* pNext; // the next node
	FIndex fIdx; // index of the face that generated this edge (the link from this node to the next)
	inline Edge() : pPrev(NULL), pNext(NULL), fIdx(NO_ID) {}
	inline bool IsEmpty() const { return (fIdx == NO_ID); }
};
struct Node {
	VIndex vIdx;
	Edge edge;
	uint32_t idComponent;
	inline Node(VIndex _vIdx) : vIdx(_vIdx), idComponent(NO_ID) {}
};
struct Graph {
	typedef std::unordered_map<VIndex, Index> VertexMap;
	typedef SEACAVE::cList<Node> Nodes;

	VertexMap index2idx;
	Nodes nodes;
	UnsignedArr components;

	inline void Clear() {
		nodes.Empty();
		components.Empty();
		index2idx.clear();
	}
	inline Index NumNodes() const { return (Index)nodes.GetSize(); }
	void AddEdge(VIndex vIdx0, VIndex vIdx1, FIndex fIdx) {
		const auto vert0(index2idx.insert(std::make_pair(vIdx0, NumNodes())));
		Node& n0 = (vert0.second ? nodes.AddConstruct(vIdx0) : nodes[vert0.first->second]);
		const auto vert1(index2idx.insert(std::make_pair(vIdx1, NumNodes())));
		Node& n1 = (vert1.second ? nodes.AddConstruct(vIdx1) : nodes[vert1.first->second]);
		n0.edge.pNext = &n1;
		n0.edge.fIdx = fIdx;
		n1.edge.pPrev = &n0;
	}
	VIndex ComputeComponents() {
		ASSERT(components.IsEmpty());
		VIndex vIdxMultiComponent(NO_ID);
		unsigned nCount(0);
		do {
			// find first node not visited yet
			Node* pNode;
			FOREACHPTR(pN, nodes) {
				if (pN->idComponent == NO_ID) {
					pNode = pN;
					break;
				}
			}
			const uint32_t id((uint32_t)components.GetSize());
			unsigned& size = components.AddConstruct(0);
			Node* const pStartNode(pNode);
			do {
				++size;
				pNode->idComponent = id;
				if (pNode->edge.pNext == NULL)
					break;
				ASSERT(pNode->edge.pNext->edge.pPrev != NULL);
				if (pNode->edge.pNext->edge.pPrev != pNode)
					vIdxMultiComponent = pNode->edge.pNext->vIdx;
			} while ((pNode=pNode->edge.pNext) != pStartNode && pNode->idComponent == NO_ID);
			nCount += size;
			if (pNode != NULL && pNode->idComponent < id) {
				const uint32_t prev_id(pNode->idComponent);
				components.RemoveLast();
				pNode = pStartNode;
				do {
					pNode->idComponent = prev_id;
				} while ((pNode=pNode->edge.pNext)->idComponent != prev_id);
			}
		} while (nCount < nodes.GetSize());
		return vIdxMultiComponent;
	}
};
} // namespace FIX_NONMANIFOLD

// find all non-manifold vertices and for each, duplicate the vertex,
// assigning the new vertex to the smallest connected set of faces
// return true if problems were found
bool Mesh::FixNonManifold()
{
	TD_TIMER_STARTD();
	using namespace FIX_NONMANIFOLD;
	ASSERT(!vertices.IsEmpty() && !faces.IsEmpty());
	if (vertexFaces.GetSize() != vertices.GetSize())
		ListIncidenteFaces();
	Graph graph;
	IndexArr componentFaces;
	IndexArr componentVertices;
	std::unordered_set<FIndex> removeFaces;
	unsigned nNonManifoldVertices(0), nPyramid3(0), nPyramid4(0);
	FOREACH(v, vertices) {
		const FaceIdxArr& vFaces = vertexFaces[v];
		FOREACHPTR(pFIdx, vFaces) {
			const Face& f(faces[*pFIdx]);
			const uint32_t i(FindVertex(f, (VIndex)v));
			graph.AddEdge(f[(i+1)%3], f[(i+2)%3], (uint32_t)*pFIdx);
		}
		// find all connected sub-graphs
		const VIndex vIdxMultiComponent(graph.ComputeComponents());
		if (graph.components.GetSize() <= 1) {
			graph.Clear();
			continue;
		}
		// there are at least two connected components (usually exactly two);
		// duplicate the vertex and assign the duplicate to the smallest component
		ASSERT(graph.components.GetSize() > 1);
		size_t nLongestCompIdx(0);
		FOREACH(c, graph.components) {
			if (graph.components[nLongestCompIdx] < graph.components[c])
				nLongestCompIdx = c;
		}
		FOREACH(c, graph.components) {
			if (c == nLongestCompIdx)
				continue;
			ASSERT(componentVertices.IsEmpty() && componentFaces.IsEmpty());
			FOREACHPTR(pNode, graph.nodes) {
				if (pNode->idComponent != c)
					continue;
				ASSERT(!pNode->edge.IsEmpty());
				componentVertices.Insert(pNode->vIdx);
				componentFaces.Insert(pNode->edge.fIdx);
			}
			if (componentFaces.GetSize() == 3 && componentVertices.GetSize() == 3 && graph.components.GetSize() == 2 && vFaces.GetSize() > 6) {
				// this is the case of a legitimate vertex on the surface and a pyramid rising from the neighboring surface
				// having the apex this vertex and the base formed by 3 vertices;
				// check that 3 faces form a loop
				DEFINE_FACE_VERTS(0);
				DEFINE_FACE_VERTS(1);
				DEFINE_FACE_VERTS(2);
				ASSERT(IS_LOOP_FACE3(0,1,2));
				// to find the right vertex order for the new face,
				// set first two vertices in the order appearing in any of the three existing faces,
				// and the third as the remaining one
				faces.AddConstruct(
					v01,
					v02,
					(v02 == v11 ? v12 : v22)
				);
				// remove component faces and create a new face from the three component vertices
				ASSERT(componentVertices.GetSize() == graph.components[c]);
				FOREACHPTR(pFIdx, componentFaces)
					removeFaces.insert(*pFIdx);
				++nPyramid3;
			#if 1
			} else if (componentFaces.GetSize() == 4 && componentVertices.GetSize() == 4 && graph.components.GetSize() == 2 && vFaces.GetSize() > 8) {
				// this is the case of a legitimate vertex on the surface and a pyramid rising from the neighboring surface
				// having the apex this vertex and the base formed by 4 vertices;
				// check that 3 faces form a loop
				DEFINE_FACE_VERTS(0);
				DEFINE_FACE_VERTS(1);
				DEFINE_FACE_VERTS(2);
				DEFINE_FACE_VERTS(3);
				// to find the right vertex order for the new faces,
				// use the fact that the faces are already in link order,
				// so set first new face as the linked vertices of the first two faces
				// and second new face as the linked vertices of the last two faces
				ASSERT(IS_LOOP_FACE4(0,1,2,3));
				faces.AddConstruct(v01, v02, v12);
				faces.AddConstruct(v21, v22, v32);
				// remove component faces and create two new faces from the four component vertices
				ASSERT(componentVertices.GetSize() == graph.components[c]);
				FOREACHPTR(pFIdx, componentFaces)
					removeFaces.insert(*pFIdx);
				++nPyramid4;
			#endif
			} else {
				// simply duplicate the vertex and assign it to the component faces
				const VIndex newIndex((VIndex)vertices.GetSize());
				Vertex& pos(vertices.AddEmpty());
				pos = vertices[v];
				FOREACHPTR(pFIdx, componentFaces)
					GetVertex(faces[*pFIdx], (VIndex)v) = newIndex;
			}
			componentVertices.Empty();
			componentFaces.Empty();
		}
		graph.Clear();
		++nNonManifoldVertices;
	}
	if (!removeFaces.empty()) {
		// remove old faces;
		// delete them in reverse order since the remove operation is simply replacing the removed item with the last item
		std::vector<FIndex> orderedRemoveFaces;
		orderedRemoveFaces.reserve(removeFaces.size());
		for (FIndex fIdx: removeFaces)
			orderedRemoveFaces.push_back(fIdx);
		std::sort(orderedRemoveFaces.begin(), orderedRemoveFaces.end());
		std::vector<FIndex>::const_iterator it(orderedRemoveFaces.cend());
		do {
			faces.RemoveAt(*(--it));
		} while (it != orderedRemoveFaces.cbegin());
	}
	DEBUG("Fixed %u non-manifold vertices and %u faces removed: %u pyramid3 and %u pyramid4 (%s)", nNonManifoldVertices, removeFaces.size(), nPyramid3, nPyramid4, TD_TIMER_GET_FMT().c_str());
	return (nNonManifoldVertices > 0);
} // FixNonManifold
#undef IS_LINK_FACE3
#undef IS_LOOP_FACE3
#undef IS_LOOP_FACE4
#undef DEFINE_FACE_VERTS
#else
#define DEFINE_FACE_VERTS(n) \
	const Face& f##n = faces[*itFace++]; \
	const uint32_t idx##n(Mesh::FindVertex(f##n, (VIndex)v)); \
	const VIndex v##n##1(f##n[(idx##n+1)%3]); \
	const VIndex v##n##2(f##n[(idx##n+2)%3])
#define IS_LOOP_FACE3(a, b, c) \
	(v##a##2 == v##b##1 && v##b##2 == v##c##1 && v##c##2 == v##a##1)
#define IS_LINK_FACE3(a, b, c) \
	(v##a##2 == v##b##1 && v##b##2 == v##c##1)
#define DEFINE_FACES4(a, b, c, d, go2) \
	if (IS_LINK_FACE3(a,b,c)) { \
		if (!IS_LINK_FACE3(c,d,a)) \
			goto go2; \
		faces.AddConstruct(v##a##1, v##a##2, v##b##2); \
		faces.AddConstruct(v##c##1, v##c##2, v##d##2); \
	}
#define DEFINE_REMOVE3(go2) \
	/* check that 3 faces form a loop */ \
	itFace = componentFaces.cbegin(); \
	DEFINE_FACE_VERTS(0); \
	DEFINE_FACE_VERTS(1); \
	DEFINE_FACE_VERTS(2); \
	if (!IS_LOOP_FACE3(0,1,2) && !IS_LOOP_FACE3(0,2,1)) \
		goto go2; \
	/* to find the right vertex order for the new face, */ \
	/* set first two vertices in the order appearing in any of the three existing faces, */ \
	/* and the third as the remaining one */ \
	faces.AddConstruct( \
		v01, \
		v02, \
		(v02 == v11 ? v12 : v22) \
	); \
	/* remove component faces and create a new face from the three component vertices */ \
	for (auto fIdx: componentFaces) \
		removeFaces.insert(fIdx); \
	++nPyramid3
#define DEFINE_REMOVE4(go2) \
	/* check that 3 faces form a loop */ \
	itFace = componentFaces.cbegin(); \
	DEFINE_FACE_VERTS(0); \
	DEFINE_FACE_VERTS(1); \
	DEFINE_FACE_VERTS(2); \
	DEFINE_FACE_VERTS(3); \
	/* to find the right vertex order for the new faces, */ \
	/* find the link order of the face */ \
	DEFINE_FACES4(0,1,2,3, go2) else \
	DEFINE_FACES4(0,1,3,2, go2) else \
	DEFINE_FACES4(0,2,1,3, go2) else \
	DEFINE_FACES4(0,2,3,1, go2) else \
	DEFINE_FACES4(0,3,1,2, go2) else \
	DEFINE_FACES4(0,3,2,1, go2) else \
		goto go2; \
	/* remove component faces and create two new faces from the four component vertices */ \
	for (auto fIdx: componentFaces) \
		removeFaces.insert(fIdx); \
	++nPyramid4
#define DEFINE_REMOVE_FACES \
	if (!removeFaces.empty()) { \
		/* remove old faces; */ \
		/* delete them in reverse order since the remove operation is simply replacing the removed item with the last item */ \
		std::vector<FIndex> orderedRemoveFaces; \
		orderedRemoveFaces.reserve(removeFaces.size()); \
		nRemoveFaces += (unsigned)removeFaces.size(); \
		for (FIndex fIdx: removeFaces) \
			orderedRemoveFaces.push_back(fIdx); \
		removeFaces.clear(); \
		std::sort(orderedRemoveFaces.begin(), orderedRemoveFaces.end()); \
		std::vector<FIndex>::const_iterator it(orderedRemoveFaces.cend()); \
		do { \
			faces.RemoveAt(*(--it)); \
		} while (it != orderedRemoveFaces.cbegin()); \
	}

struct VertexInfo {
	typedef Mesh::VIndex VIndex;
	typedef Mesh::FIndex FIndex;
	typedef boost::property<boost::vertex_index1_t, VIndex> VertexProperty;
	typedef boost::property<boost::edge_index_t, FIndex> EdgeProperty;
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty, EdgeProperty> Graph;
	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<Graph>::edge_descriptor Edge;
	typedef boost::property_map<Graph, boost::vertex_index1_t>::type VertexIndex1Map;
	typedef boost::property_map<Graph, boost::edge_index_t>::type EdgeIndexMap;
	typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
	typedef boost::graph_traits<Graph>::out_edge_iterator EdgeIter;
	typedef std::unordered_map<Vertex, Graph::vertices_size_type> Components;
	typedef boost::associative_property_map<Components> ComponentMap;
	typedef std::unordered_map<VIndex, Vertex> VertexMap;
	typedef std::unordered_set<Vertex> VertexSet;
	struct FilterVertex {
		FilterVertex() {}
		FilterVertex(const VertexSet* _filterVerts) : filterVerts(_filterVerts) {}
		template <typename Vertex>
		bool operator()(const Vertex& v) const {
			return (filterVerts->find(v) == filterVerts->cend());
		}
		const VertexSet* filterVerts;
	};
	struct FilterEdge {
		FilterEdge() {}
		FilterEdge(const Graph* _graph, const VertexSet* _filterVerts) : graph(_graph), filterVerts(_filterVerts) {}
		template <typename Edge>
		bool operator()(const Edge& e) const {
			return (filterVerts->find(boost::source(e,*graph)) == filterVerts->cend() &&
					filterVerts->find(boost::target(e,*graph)) == filterVerts->cend());
		}
		const VertexSet* filterVerts;
		const Graph* graph;
	};

	VertexMap index2idx; // useful/valid only during graph creation
	Graph graph;
	VertexIndex1Map vertexIndex1;
	EdgeIndexMap edgeIndex;
	Components components;
	VertexSet filterVerts;

	inline VertexInfo() {
		vertexIndex1 = boost::get(boost::vertex_index1, graph);
		edgeIndex = boost::get(boost::edge_index, graph);
	}
	Vertex AddVertex(VIndex v) {
		auto vert(index2idx.insert(std::make_pair(v, Vertex())));
		if (vert.second) {
			vert.first->second = boost::add_vertex(graph);
			vertexIndex1[vert.first->second] = v;
		}
		return vert.first->second;
	}
	void AddEdge(VIndex v0, VIndex v1, FIndex f) {
		boost::add_edge(AddVertex(v0), AddVertex(v1), f, graph);
	}
	size_t ComputeComponents() {
		components.clear();
		ComponentMap componentMap(components);
		return boost::connected_components(graph, componentMap);
	}
	size_t ComputeFilteredComponents() {
		ASSERT(!filterVerts.empty());
		FilterEdge filterEdge(&graph, &filterVerts);
		FilterVertex filterVertex(&filterVerts);
		boost::filtered_graph<Graph, FilterEdge, FilterVertex> filterGraph(graph, filterEdge, filterVertex);
		components.clear();
		ComponentMap componentMap(components);
		const size_t nComponents(boost::connected_components(filterGraph, componentMap));
		filterVerts.clear();
		return nComponents;
	}
	void Clear() {
		graph.clear();
		index2idx.clear();
	}
};

// find all non-manifold edges/vertices and for each, duplicate the vertex,
// assigning the new vertex to the smallest connected set of faces;
// return true if problems were found
bool Mesh::FixNonManifold()
{
	TD_TIMER_STARTD();
	ASSERT(!vertices.IsEmpty() && !faces.IsEmpty());
	if (vertexFaces.GetSize() != vertices.GetSize())
		ListIncidenteFaces();
	VertexInfo vertexInfo;
	IntArr sizes;
	unsigned nNonManifoldVertices(0), nNonManifoldEdges(0), nRemoveFaces(0), nPyramid3(0), nPyramid4(0);
	std::unordered_set<FIndex> seenFaces;
	std::unordered_set<FIndex> removeFaces;
	std::unordered_set<FIndex> componentFaces;
	std::unordered_set<FIndex>::const_iterator itFace;
	VertexInfo::EdgeIter ei, eie;
	// fix non-manifold edges
	ASSERT(seenFaces.empty());
	FOREACH(v, vertices) {
		const FaceIdxArr& vFaces = vertexFaces[v];
		if (vFaces.GetSize() < 3)
			continue;
		FOREACHPTR(pFIdx, vFaces) {
			const Face& f(faces[*pFIdx]);
			const uint32_t i(FindVertex(f, (VIndex)v));
			vertexInfo.AddEdge(f[(i+1)%3], f[(i+2)%3], (FIndex)*pFIdx);
		}
		for (const auto& idx2id: vertexInfo.index2idx) {
			boost::tie(ei, eie) = boost::out_edges(idx2id.second, vertexInfo.graph);
			if (std::distance(ei, eie) >= 4) {
				ASSERT(vertexInfo.filterVerts.empty());
				// do not proceed, if any of the faces was removed
				FOREACHPTR(pFIdx, vFaces) {
					if (seenFaces.find(*pFIdx) != seenFaces.cend())
						goto ABORT_EDGE;
				}
				{
				// current vertex and this vertex form the non-manifold edge
				if (vertexInfo.ComputeComponents() > 1) {
					// filter-out all vertices not belonging to this component
					const size_t mainComp(vertexInfo.components[idx2id.second]);
					for (const auto& idx2id: vertexInfo.index2idx) {
						if (vertexInfo.components[idx2id.second] != mainComp)
							vertexInfo.filterVerts.insert(idx2id.second);
					}
				}
				// filter-out this vertex to find the two components to be split
				vertexInfo.filterVerts.insert(idx2id.second);
				const size_t nComponents(vertexInfo.ComputeFilteredComponents());
				if (nComponents < 2)
					break; // something is wrong, the vertex configuration is not as expected
				// find all vertices in the smallest component
				sizes.Resize(nComponents);
				sizes.Memset(0);
				for (const auto& comp: vertexInfo.components)
					++sizes[comp.second];
				size_t nLongestCompIdx(0);
				for (size_t s=1; s<sizes.GetSize(); ++s) {
					if (sizes[nLongestCompIdx] < sizes[s])
						nLongestCompIdx = s;
				}
				FOREACH(s, sizes) {
					if (s == nLongestCompIdx)
						continue;
					ASSERT(componentFaces.empty());
					Mesh::Vertex pos(vertices[idx2id.first]);
					for (const auto& comp: vertexInfo.components) {
						if (comp.second == s) {
							for (boost::tie(ei, eie) = boost::out_edges(comp.first, vertexInfo.graph); ei != eie; ++ei)
								componentFaces.insert(vertexInfo.edgeIndex[*ei]);
							pos += vertices[vertexInfo.vertexIndex1[comp.first]];
						}
					}
					const size_t nComponentVertices(sizes[s]+1); // including intersection vertex (this vertex)
					if (componentFaces.size() != nComponentVertices) {
						componentFaces.clear();
						break; // something is wrong, the vertex configuration is not as expected
					}
					if (componentFaces.size() == 3/* && vertexInfo.components.size() > 6*/) {
						// this is the case of a legitimate vertex on the surface and a pyramid rising from the neighboring surface
						// having the apex the current vertex and the base formed by 3 vertices - one being this vertex;
 						DEFINE_REMOVE3(GENERAL_EDGE);
					#if 1
					} else if (componentFaces.size() == 4/* && vertexInfo.components.size() > 8*/) {
						// this is the case of a legitimate vertex on the surface and a pyramid rising from the neighboring surface
						// having the apex the current vertex and the base formed by 4 vertices - one being this vertex;
						DEFINE_REMOVE4(GENERAL_EDGE);
					#endif
					} else {
						GENERAL_EDGE:
						// simply duplicate the vertex and assign it to the component faces
						const Mesh::VIndex newIndex((Mesh::VIndex)vertices.GetSize());
						for (auto fIdx: componentFaces)
							GetVertex(faces[fIdx], (VIndex)v) = newIndex;
						vertices.Insert(pos / nComponentVertices);
					}
					for (auto fIdx: componentFaces)
						seenFaces.insert(fIdx);
					componentFaces.clear();
				}
				++nNonManifoldEdges;
				}
				ABORT_EDGE:
				break;
			}
		}
		vertexInfo.Clear();
	}
	seenFaces.clear();
	DEFINE_REMOVE_FACES;
	// fix non-manifold vertices
	if (nNonManifoldEdges)
		ListIncidenteFaces();
	ASSERT(seenFaces.empty());
	FOREACH(v, vertices) {
		const FaceIdxArr& vFaces = vertexFaces[v];
		if (vFaces.GetSize() < 3)
			continue;
		FOREACHPTR(pFIdx, vFaces) {
			const Face& f(faces[*pFIdx]);
			const uint32_t i(FindVertex(f, (VIndex)v));
			vertexInfo.AddEdge(f[(i+1)%3], f[(i+2)%3], (FIndex)*pFIdx);
		}
		// find all connected sub-graphs
		const size_t nComponents(vertexInfo.ComputeComponents());
		if (nComponents == 1)
			goto ABORT_VERTEX;
		// do not proceed, if any of the faces was removed
		FOREACHPTR(pFIdx, vFaces) {
			if (seenFaces.find(*pFIdx) != seenFaces.cend())
				goto ABORT_VERTEX;
		}
		{
		// there are at least two connected components (usually exactly two);
		// duplicate the vertex and assign the duplicate to the smallest component
		ASSERT(nComponents > 1);
		sizes.Resize(nComponents);
		sizes.Memset(0);
		for (const auto& comp: vertexInfo.components)
			++sizes[comp.second];
		size_t nLongestCompIdx(0);
		for (size_t s=1; s<sizes.GetSize(); ++s) {
			if (sizes[nLongestCompIdx] < sizes[s])
				nLongestCompIdx = s;
		}
		FOREACH(s, sizes) {
			if (s == nLongestCompIdx)
				continue;
			ASSERT(componentFaces.empty());
			for (const auto& idx2id: vertexInfo.index2idx) {
				if (vertexInfo.components[idx2id.second] == s) {
					for (boost::tie(ei, eie) = boost::out_edges(idx2id.second, vertexInfo.graph); ei != eie; ++ei)
						componentFaces.insert(vertexInfo.edgeIndex[*ei]);
				}
			}
			if (componentFaces.size() == 3 && sizes[s] == 3 && nComponents == 2/* && vFaces.GetSize() > 6*/) {
				// this is the case of a legitimate vertex on the surface and a pyramid rising from the neighboring surface
				// having the apex this vertex and the base formed by 3 vertices;
				DEFINE_REMOVE3(GENERAL_VERTEX);
			#if 1
			} else if (componentFaces.size() == 4 && sizes[s] == 4 && nComponents == 2/* && vFaces.GetSize() > 8*/) {
				// this is the case of a legitimate vertex on the surface and a pyramid rising from the neighboring surface
				// having the apex this vertex and the base formed by 4 vertices;
				DEFINE_REMOVE4(GENERAL_VERTEX);
			#endif
			} else {
				GENERAL_VERTEX:
				// simply duplicate the vertex and assign it to the component faces
				const VIndex newIndex((VIndex)vertices.GetSize());
				Vertex& pos(vertices.AddEmpty());
				pos = vertices[v];
				for (auto fIdx: componentFaces)
					GetVertex(faces[fIdx], (VIndex)v) = newIndex;
			}
			for (auto fIdx: componentFaces)
				seenFaces.insert(fIdx);
			componentFaces.clear();
		}
		++nNonManifoldVertices;
		}
		ABORT_VERTEX:;
		vertexInfo.Clear();
	}
	seenFaces.clear();
	if (nNonManifoldVertices)
		vertexFaces.Empty();
	DEFINE_REMOVE_FACES;
	DEBUG("Fixed %u/%u non-manifold edges/vertices and %u faces removed: %u pyramid3 and %u pyramid4 (%s)", nNonManifoldEdges, nNonManifoldVertices, nRemoveFaces, nPyramid3, nPyramid4, TD_TIMER_GET_FMT().c_str());
	return (nNonManifoldEdges > 0 || nNonManifoldVertices > 0);
} // FixNonManifold
#undef DEFINE_REMOVE_FACES
#undef DEFINE_REMOVE3
#undef DEFINE_REMOVE4
#undef DEFINE_FACES4
#undef IS_LINK_FACE3
#undef IS_LOOP_FACE3
#undef DEFINE_FACE_VERTS
#endif
/*----------------------------------------------------------------*/

namespace CLEAN {
// define mesh type
class Vertex; class Edge; class Face;
struct UsedTypes : public vcg::UsedTypes<
	vcg::Use<Vertex>::AsVertexType,
	vcg::Use<Edge>  ::AsEdgeType,
	vcg::Use<Face>  ::AsFaceType   > {};

class Vertex : public vcg::Vertex<UsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::VFAdj, vcg::vertex::Mark, vcg::vertex::BitFlags> {};
class Face   : public vcg::Face<  UsedTypes, vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::FFAdj, vcg::face::VFAdj, vcg::face::Mark, vcg::face::BitFlags> {};
class Edge   : public vcg::Edge<  UsedTypes, vcg::edge::VertexRef> {};

class Mesh : public vcg::tri::TriMesh< std::vector<Vertex>, std::vector<Face>, std::vector<Edge> > {};

// decimation helper classes
typedef	vcg::SimpleTempData< Mesh::VertContainer, vcg::math::Quadric<double> > QuadricTemp;

class QHelper
{
public:
	QHelper() {}
	static void Init() {}
	static vcg::math::Quadric<double> &Qd(Vertex &v) { return TD()[v]; }
	static vcg::math::Quadric<double> &Qd(Vertex *v) { return TD()[*v]; }
	static Vertex::ScalarType W(Vertex * /*v*/) { return 1.0; }
	static Vertex::ScalarType W(Vertex & /*v*/) { return 1.0; }
	static void Merge(Vertex & /*v_dest*/, Vertex const & /*v_del*/) {}
	static QuadricTemp* &TDp() { static QuadricTemp *td; return td; }
	static QuadricTemp &TD() { return *TDp(); }
};

typedef BasicVertexPair<Vertex> VertexPair;

class TriEdgeCollapse : public vcg::tri::TriEdgeCollapseQuadric<Mesh, VertexPair, TriEdgeCollapse, QHelper> {
public:
	typedef vcg::tri::TriEdgeCollapseQuadric<Mesh, VertexPair, TriEdgeCollapse, QHelper> TECQ;
	inline TriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp) :TECQ(p, i, pp) {}
};
}

// decimate, clean and smooth mesh
// fDecimate factor is in range (0..1], if 1 no decimation takes place
void Mesh::Clean(float fDecimate, float fSpurious, bool bRemoveSpikes, unsigned nCloseHoles, unsigned nSmooth)
{
	TD_TIMER_STARTD();
	// create VCG mesh
	ASSERT(!vertices.IsEmpty() && !faces.IsEmpty());
	CLEAN::Mesh mesh;
	{
		CLEAN::Mesh::VertexIterator vi = vcg::tri::Allocator<CLEAN::Mesh>::AddVertices(mesh, vertices.GetSize());
		FOREACHPTR(pVert, vertices) {
			const Vertex& p(*pVert);
			CLEAN::Vertex::CoordType& P((*vi).P());
			P[0] = p.x;
			P[1] = p.y;
			P[2] = p.z;
			++vi;
		}
		vertices.Release();
		vi = mesh.vert.begin();
		std::vector<CLEAN::Mesh::VertexPointer> indices(mesh.vert.size());
		for (CLEAN::Mesh::VertexPointer& idx: indices) {
			idx = &*vi;
			++vi;
		}
		CLEAN::Mesh::FaceIterator fi = vcg::tri::Allocator<CLEAN::Mesh>::AddFaces(mesh, faces.GetSize());
		FOREACHPTR(pFace, faces) {
			const Face& f(*pFace);
			ASSERT((*fi).VN() == 3);
			ASSERT(f[0]>=0 && f[0]<(uint32_t)mesh.vn);
			(*fi).V(0) = indices[f[0]];
			ASSERT(f[1]>=0 && f[1]<(uint32_t)mesh.vn);
			(*fi).V(1) = indices[f[1]];
			ASSERT(f[2]>=0 && f[2]<(uint32_t)mesh.vn);
			(*fi).V(2) = indices[f[2]];
			++fi;
		}
		faces.Release();
	}

	// decimate mesh
	if (fDecimate < 1) {
		ASSERT(fDecimate > 0);
		vcg::tri::TriEdgeCollapseQuadricParameter pp;
		pp.QualityThr = 0.3; // Quality Threshold for penalizing bad shaped faces: the value is in the range [0..1], 0 accept any kind of face (no penalties), 0.5 penalize faces with quality < 0.5, proportionally to their shape
		pp.PreserveBoundary = false; // the simplification process tries to not affect mesh boundaries during simplification
		pp.BoundaryWeight = 1; // the importance of the boundary during simplification: the value is in the range (0..+inf), default (1.0) means that the boundary has the same importance as the rest; values greater than 1.0 raise boundary importance and has the effect of removing less vertices on the border
		pp.PreserveTopology = false; // avoid all collapses that cause a topology change in the mesh (like closing holes, squeezing handles, etc); if checked the genus of the mesh should stay unchanged
		pp.QualityWeight = false; // use the Per-Vertex quality as a weighting factor for the simplification: the weight is used as an error amplification value, so a vertex with a high quality value will not be simplified and a portion of the mesh with low quality values will be aggressively simplified
		pp.NormalCheck = false; // try to avoid face flipping effects and try to preserve the original orientation of the surface
		pp.OptimalPlacement = true; // each collapsed vertex is placed in the position minimizing the quadric error; it can fail (creating bad spikes) in case of very flat areas; if disabled edges are collapsed onto one of the two original vertices and the final mesh is composed by a subset of the original vertices
		pp.QualityQuadric = false; // add additional simplification constraints that improves the quality of the simplification of the planar portion of the mesh
		// decimate
		vcg::tri::UpdateTopology<CLEAN::Mesh>::VertexFace(mesh);
		vcg::tri::UpdateFlags<CLEAN::Mesh>::FaceBorderFromVF(mesh);
		const int TargetFaceNum(ROUND2INT(fDecimate*mesh.fn));
		vcg::math::Quadric<double> QZero;
		QZero.SetZero();
		CLEAN::QuadricTemp TD(mesh.vert, QZero);
		CLEAN::QHelper::TDp()=&TD;
		if (pp.PreserveBoundary) {
			pp.FastPreserveBoundary = true;
			pp.PreserveBoundary = false;
		}
		if (pp.NormalCheck)
			pp.NormalThrRad = M_PI/4.0;
		const int OriginalFaceNum(mesh.fn);
		Util::Progress progress(_T("Decimated faces"), OriginalFaceNum-TargetFaceNum);
		vcg::LocalOptimization<CLEAN::Mesh> DeciSession(mesh, &pp);
		DeciSession.Init<CLEAN::TriEdgeCollapse>();
		DeciSession.SetTargetSimplices(TargetFaceNum);
		DeciSession.SetTimeBudget(0.1f); // this allow to update the progress bar 10 time for sec...
		while (DeciSession.DoOptimization() && mesh.fn>TargetFaceNum)
			progress.display(OriginalFaceNum - mesh.fn);
		DeciSession.Finalize<CLEAN::TriEdgeCollapse>();
		progress.close();
		DEBUG("Mesh decimated: %d -> %d faces", OriginalFaceNum, TargetFaceNum);
	}

	// clean mesh
	{
		vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		const int nZeroAreaFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveZeroAreaFace(mesh);
		DEBUG("Removed %d zero-area faces", nZeroAreaFaces);
		const int nDuplicateFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateFace(mesh);
		DEBUG("Removed %d duplicate faces", nDuplicateFaces);
		const int nNonManifoldFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldFace(mesh);
		DEBUG("Removed %d non-manifold faces", nNonManifoldFaces);
		const int nDegenerateVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveDegenerateVertex(mesh);
		DEBUG("Removed %d degenerate vertices", nDegenerateVertices);
		#if 0
		const int nDuplicateVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateVertex(mesh);
		DEBUG("Removed %d duplicate vertices", nDuplicateVertices);
		#endif
		const int nNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldVertex(mesh);
		DEBUG("Removed %d non-manifold vertices", nNonManifoldVertices);
		const int nUnreferencedVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveUnreferencedVertex(mesh);
		DEBUG("Removed %d unreferenced vertices", nUnreferencedVertices);
		vcg::tri::Allocator<CLEAN::Mesh>::CompactFaceVector(mesh);
		vcg::tri::Allocator<CLEAN::Mesh>::CompactVertexVector(mesh);
		vcg::tri::UpdateTopology<CLEAN::Mesh>::AllocateEdge(mesh);
	}

	// remove spurious components
	if (fSpurious > 0) {
		FloatArr edgeLens(0, mesh.EN());
		for (CLEAN::Mesh::EdgeIterator ei=mesh.edge.begin(); ei!=mesh.edge.end(); ++ei) {
			const CLEAN::Vertex::CoordType& P0((*ei).V(0)->P());
			const CLEAN::Vertex::CoordType& P1((*ei).V(1)->P());
			edgeLens.Insert((P1-P0).Norm());
		}
		#if 0
		const auto ret(ComputeX84Threshold<float,float>(edgeLens.Begin(), edgeLens.GetSize(), 3.f*fSpurious));
		const float thLongEdge(ret.first+ret.second);
		#else
		const float thLongEdge(edgeLens.GetNth(edgeLens.GetSize()*9/10)*fSpurious);
		#endif
		// remove faces with too long edges
		const size_t numLongFaces(vcg::tri::UpdateSelection<CLEAN::Mesh>::FaceOutOfRangeEdge(mesh, 0, thLongEdge));
		for (CLEAN::Mesh::FaceIterator fi=mesh.face.begin(); fi!=mesh.face.end(); ++fi)
			if (!(*fi).IsD() && (*fi).IsS())
				vcg::tri::Allocator<CLEAN::Mesh>::DeleteFace(mesh, *fi);
		DEBUG("Removed %d faces with edges longer than %f", numLongFaces, thLongEdge);
		// remove isolated components
		vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		const std::pair<int, int> delInfo(vcg::tri::Clean<CLEAN::Mesh>::RemoveSmallConnectedComponentsDiameter(mesh, thLongEdge*1.5f));
		DEBUG("Removed %d connected components out of %d", delInfo.second, delInfo.first);
	}

	// remove spikes
	if (bRemoveSpikes) {
		int nTotalSpikes(0);
		vcg::tri::RequireVFAdjacency(mesh);
		while (true) {
			if (fSpurious <= 0 || nTotalSpikes != 0)
				vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
			vcg::tri::UpdateTopology<CLEAN::Mesh>::VertexFace(mesh);
			int nSpikes(0);
			for (CLEAN::Mesh::VertexIterator vi=mesh.vert.begin(); vi!=mesh.vert.end(); ++vi) {
				if (vi->IsD())
					continue;
				CLEAN::Face* const start(vi->cVFp());
				if (start == NULL) {
					vcg::tri::Allocator<CLEAN::Mesh>::DeleteVertex(mesh, *vi);
					continue;
				}
				vcg::face::JumpingPos<CLEAN::Face> p(start, vi->cVFi(), &*vi);
				int count(0);
				do {
					++count;
					p.NextFE();
				} while (p.f!=start);
				if (count == 1) {
					vcg::tri::Allocator<CLEAN::Mesh>::DeleteVertex(mesh, *vi);
					++nSpikes;
				}
			}
			if (nSpikes == 0)
				break;
			for (CLEAN::Mesh::FaceIterator fi=mesh.face.begin(); fi!=mesh.face.end(); ++fi) {
				if (!fi->IsD() &&
					(fi->V(0)->IsD() ||
					 fi->V(1)->IsD() ||
					 fi->V(2)->IsD()))
					vcg::tri::Allocator<CLEAN::Mesh>::DeleteFace(mesh, *fi);
			}
			nTotalSpikes += nSpikes;
		}
		DEBUG("Removed %d spikes", nTotalSpikes);
	}

	// close holes
	if (nCloseHoles > 0) {
		if (fSpurious <= 0 && !bRemoveSpikes)
			vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		ASSERT(vcg::tri::Clean<CLEAN::Mesh>::CountNonManifoldEdgeFF(mesh) == 0);
		const int OriginalSize(mesh.fn);
		#if 1
		// When closing holes it tries to prevent the creation of faces that intersect faces adjacent to
		// the boundary of the hole. It is an heuristic, non intersecting hole filling can be NP-complete.
		const int holeCnt(vcg::tri::Hole<CLEAN::Mesh>::EarCuttingIntersectionFill< vcg::tri::SelfIntersectionEar<CLEAN::Mesh> >(mesh, (int)nCloseHoles, false));
		#else
		const int holeCnt = vcg::tri::Hole<CLEAN::Mesh>::EarCuttingFill< vcg::tri::MinimumWeightEar<CLEAN::Mesh> >(mesh, (int)nCloseHoles, false);
		#endif
		DEBUG("Closed %d holes and added %d new faces", holeCnt, mesh.fn-OriginalSize);
	}

	// smooth mesh
	if (nSmooth > 0) {
		if (fSpurious <= 0 && !bRemoveSpikes && nCloseHoles <= 0)
			vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		vcg::tri::UpdateFlags<CLEAN::Mesh>::FaceBorderFromFF(mesh);
		#if 1
		vcg::tri::Smooth<CLEAN::Mesh>::VertexCoordLaplacian(mesh, (int)nSmooth, false, false);
		#else
		vcg::tri::Smooth<CLEAN::Mesh>::VertexCoordLaplacianHC(mesh, (int)nSmooth, false);
		#endif
		DEBUG("Smoothed %d vertices", mesh.vn);
	}

	// clean mesh
	{
		const int nNonManifoldFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldFace(mesh);
		DEBUG("Removed %d non-manifold faces", nNonManifoldFaces);
		const int nNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldVertex(mesh);
		DEBUG("Removed %d non-manifold vertices", nNonManifoldVertices);
	}

	// import VCG mesh
	{
		ASSERT(vertices.IsEmpty() && faces.IsEmpty());
		vertices.Reserve(mesh.VN());
		vcg::SimpleTempData<CLEAN::Mesh::VertContainer, VIndex> indices(mesh.vert);
		VIndex idx(0);
		for (CLEAN::Mesh::VertexIterator vi=mesh.vert.begin(); vi!=mesh.vert.end(); ++vi) {
			if (vi->IsD())
				continue;
			Vertex& p(vertices.AddEmpty());
			const CLEAN::Vertex::CoordType& P((*vi).P());
			p.x = P[0];
			p.y = P[1];
			p.z = P[2];
			indices[vi] = idx++;
		}
		faces.Reserve(mesh.FN());
		for (CLEAN::Mesh::FaceIterator fi=mesh.face.begin(); fi!=mesh.face.end(); ++fi) {
			if (fi->IsD())
				continue;
			CLEAN::Mesh::FacePointer fp(&(*fi));
			Face& f(faces.AddEmpty());
			f[0] = indices[fp->cV(0)];
			f[1] = indices[fp->cV(1)];
			f[2] = indices[fp->cV(2)];
		}
	}
	DEBUG("Cleaned mesh: %u vertices, %u faces (%s)", vertices.GetSize(), faces.GetSize(), TD_TIMER_GET_FMT().c_str());
} // Clean
/*----------------------------------------------------------------*/


// define a PLY file format composed only of vertices and triangles
namespace BasicPLY {
	// list of property information for a vertex
	static const PLY::PlyProperty vert_props[] = {
		{"x", PLY::Float32, PLY::Float32, offsetof(Mesh::Vertex,x), 0, 0, 0, 0},
		{"y", PLY::Float32, PLY::Float32, offsetof(Mesh::Vertex,y), 0, 0, 0, 0},
		{"z", PLY::Float32, PLY::Float32, offsetof(Mesh::Vertex,z), 0, 0, 0, 0}
	};
	// list of property information for a face
	struct Facet {
		uint8_t num;
		Mesh::Face* pFace;
	};
	static const PLY::PlyProperty face_props[] = {
		{"vertex_indices", PLY::Uint32, PLY::Uint32, offsetof(Facet,pFace), 1, PLY::Uint8, PLY::Uint8, offsetof(Facet,num)}
	};
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex",
		"face"
	};
} // namespace BasicPLY

// import the mesh as a PLY file
bool Mesh::Load(const String& fileName)
{
	ASSERT(!fileName.IsEmpty());
	Release();

	// open PLY file and read header
	PLY ply;
	if (!ply.read(fileName)) {
		DEBUG_EXTRA("error: invalid PLY file");
		return false;
	}
	for (int i = 0; i < ply.elems.size(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			vertices.Resize(elem_count);
		} else
		if (PLY::equal_strings(BasicPLY::elem_names[1], elem_name)) {
			faces.Resize(elem_count);
		}
	}
	if (vertices.IsEmpty() || faces.IsEmpty()) {
		DEBUG_EXTRA("error: invalid mesh file");
		return false;
	}

	// read PLY body
	BasicPLY::Facet facet;
	for (int i = 0; i < ply.elems.size(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			ASSERT(vertices.GetSize() == elem_count);
			ply.setup_property(BasicPLY::vert_props[0]);
			ply.setup_property(BasicPLY::vert_props[1]);
			ply.setup_property(BasicPLY::vert_props[2]);
			FOREACHPTR(pVert, vertices)
				ply.get_element(pVert);
		} else
		if (PLY::equal_strings(BasicPLY::elem_names[1], elem_name)) {
			ASSERT(faces.GetSize() == elem_count);
			ply.setup_property(BasicPLY::face_props[0]);
			FOREACHPTR(pFace, faces) {
				ply.get_element(&facet);
				if (facet.num != 3) {
					DEBUG_EXTRA("error: unsupported mesh file (face not triangle)");
					return false;
				}
				memcpy(pFace, facet.pFace, sizeof(VIndex)*3);
				delete[] facet.pFace;
			}
		} else {
			ply.get_other_element();
		}
	}
	return true;
} // Load
/*----------------------------------------------------------------*/

// export the mesh as a PLY file
bool Mesh::Save(const String& fileName, bool bBinary) const
{
	ASSERT(!fileName.IsEmpty());
	Util::ensureDirectory(fileName);

	// create PLY object
	const size_t bufferSize(vertices.GetSize()*(4*3/*pos*/+2/*eol*/) + faces.GetSize()*(1*1/*len*/+4*3/*idx*/+2/*eol*/) + 2048/*extra size*/);
	PLY ply;
	if (!ply.write(fileName, 2, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII, bufferSize)) {
		DEBUG_EXTRA("error: can not create the mesh file");
		return false;
	}

	// describe what properties go into the vertex elements
	ply.describe_property(BasicPLY::elem_names[0], 3, BasicPLY::vert_props);

	// export the array of vertices
	FOREACHPTR(pVert, vertices)
		ply.put_element(pVert);
	if (ply.get_current_element_count() == 0)
		return false;

	// describe what properties go into the vertex elements
	ply.describe_property(BasicPLY::elem_names[1], 1, BasicPLY::face_props);

	// export the array of faces
	BasicPLY::Facet facet; facet.num = 3;
	FOREACHPTR(pFace, faces) {
		facet.pFace = pFace;
		ply.put_element(&facet);
	}
	if (ply.get_current_element_count() == 0)
		return false;

	// write to file
	if (!ply.header_complete())
		return false;
	return true;
} // Save
/*----------------------------------------------------------------*/

bool Mesh::Save(const VertexArr& vertices, const String& fileName, bool bBinary)
{
	ASSERT(!fileName.IsEmpty());
	Util::ensureDirectory(fileName);

	// create PLY object
	const size_t bufferSize(vertices.GetSize()*(4*3/*pos*/+2/*eol*/) + 2048/*extra size*/);
	PLY ply;
	if (!ply.write(fileName, 1, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII, bufferSize)) {
		DEBUG_EXTRA("error: can not create the mesh file");
		return false;
	}

	// describe what properties go into the vertex elements
	ply.describe_property(BasicPLY::elem_names[0], 3, BasicPLY::vert_props);

	// export the array of vertices
	FOREACHPTR(pVert, vertices)
		ply.put_element(pVert);
	if (ply.get_current_element_count() == 0)
		return false;

	// write to file
	if (!ply.header_complete())
		return false;
	return true;
}
/*----------------------------------------------------------------*/
