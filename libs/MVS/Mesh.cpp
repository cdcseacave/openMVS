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
	ReleaseExtra();
} // Release
void Mesh::ReleaseExtra()
{
	vertexNormals.Release();
	vertexFaces.Release();
	vertexBoundary.Release();
	faceNormals.Release();
	faceTexcoords.Release();
	textureDiffuse.release();
} // ReleaseExtra
void Mesh::EmptyExtra()
{
	vertexNormals.Empty();
	vertexFaces.Empty();
	vertexBoundary.Empty();
	faceNormals.Empty();
	faceTexcoords.Empty();
	textureDiffuse.release();
} // EmptyExtra
/*----------------------------------------------------------------*/


// extract array of triangles incident to each vertex
void Mesh::ListIncidenteFaces()
{
	vertexFaces.Empty();
	vertexFaces.Resize(vertices.GetSize());
	FOREACH(i, faces) {
		const Face& face = faces[i];
		for (int v=0; v<3; ++v) {
			ASSERT(vertexFaces[face[v]].Find(i) == FaceIdxArr::NO_INDEX);
			vertexFaces[face[v]].Insert(i);
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

// compute normal for all faces
void Mesh::ComputeNormalFaces()
{
	faceNormals.Resize(faces.GetSize());
	FOREACH(idxFace, faces) {
		const Face& face = faces[idxFace];
		const Vertex& v0 = vertices[face[0]];
		const Vertex& v1 = vertices[face[1]];
		const Vertex& v2 = vertices[face[2]];
		Normal& normal = faceNormals[idxFace];
		normal = normalized((v1-v0).cross(v2-v0));
	}
}

// compute normal for all vertices
#if 1
// computes the vertex normal as the area weighted face normals average
void Mesh::ComputeNormalVertices()
{
	vertexNormals.Resize(vertices.GetSize());
	vertexNormals.Memset(0);
	FOREACHPTR(pFace, faces) {
		const Face& face = *pFace;
		const Vertex& v0 = vertices[face[0]];
		const Vertex& v1 = vertices[face[1]];
		const Vertex& v2 = vertices[face[2]];
		const Normal t((v1 - v0).cross(v2 - v0));
		vertexNormals[face[0]] += t;
		vertexNormals[face[1]] += t;
		vertexNormals[face[2]] += t;
	}
	FOREACHPTR(pVertexNormal, vertexNormals)
		normalize(*pVertexNormal);
}
#else
// computes the vertex normal as an angle weighted average
// (the vertex first ring of faces and the face normals are used)
//
// The normal of a vertex v computed as a weighted sum f the incident face normals.
// The weight is simply the angle of the involved wedge. Described in:
// G. Thurmer, C. A. Wuthrich "Computing vertex normals from polygonal facets", Journal of Graphics Tools, 1998
inline float AngleN(const Mesh::Normal& p1, const Mesh::Normal& p2) {
	float t(p1.dot(p2));
	if (t>1) t = 1; else
	if (t<-1) t = -1;
	return acosf(t);
}
void Mesh::ComputeNormalVertices()
{
	ASSERT(!faceNormals.IsEmpty());
	vertexNormals.Resize(vertices.GetSize());
	vertexNormals.Memset(0);
	FOREACH(idxFace, faces) {
		const Face& face = faces[idxFace];
		const Normal& t = faceNormals[idxFace];
		const Vertex& v0 = vertices[face[0]];
		const Vertex& v1 = vertices[face[1]];
		const Vertex& v2 = vertices[face[2]];
		const Normal e0(normalized(v1-v0));
		const Normal e1(normalized(v2-v1));
		const Normal e2(normalized(v0-v2));
		vertexNormals[face[0]] += t*AngleN(e0, -e2);
		vertexNormals[face[1]] += t*AngleN(-e0, e1);
		vertexNormals[face[2]] += t*AngleN(-e1, e2);
	}
	FOREACHPTR(pVertexNormal, vertexNormals)
		normalize(*pVertexNormal);
}
#endif
/*----------------------------------------------------------------*/


void Mesh::GetEdgeFaces(VIndex v0, VIndex v1, FaceIdxArr& afaces) const
{
	const FaceIdxArr& faces0 = vertexFaces[v0];
	const FaceIdxArr& faces1 = vertexFaces[v1];
	std::unordered_set<FIndex> setFaces1(faces1.Begin(), faces1.End());
	FOREACH(i, faces0) {
		if (setFaces1.find(faces0[i]) != setFaces1.end())
			afaces.Insert(faces0[i]);
	}
}

void Mesh::GetFaceFaces(FIndex f, FaceIdxArr& afaces) const
{
	const Face& face = faces[f];
	const FaceIdxArr& faces0 = vertexFaces[face[0]];
	const FaceIdxArr& faces1 = vertexFaces[face[1]];
	const FaceIdxArr& faces2 = vertexFaces[face[2]];
	std::unordered_set<FIndex> setFaces(faces1.Begin(), faces1.End());
	FOREACHPTR(pIdxFace, faces0) {
		if (f != *pIdxFace && setFaces.find(*pIdxFace) != setFaces.end())
			afaces.Insert(*pIdxFace);
	}
	FOREACHPTR(pIdxFace, faces2) {
		if (f != *pIdxFace && setFaces.find(*pIdxFace) != setFaces.end())
			afaces.Insert(*pIdxFace);
	}
	setFaces.clear();
	setFaces.insert(faces2.Begin(), faces2.End());
	FOREACHPTR(pIdxFace, faces0) {
		if (f != *pIdxFace && setFaces.find(*pIdxFace) != setFaces.end())
			afaces.Insert(*pIdxFace);
	}
}

void Mesh::GetEdgeVertices(FIndex f0, FIndex f1, uint32_t* vs0, uint32_t* vs1) const
{
	const Face& face0 = faces[f0];
	const Face& face1 = faces[f1];
	int i(0);
	for (int v=0; v<3; ++v) {
		if ((vs1[i] = FindVertex(face1, face0[v])) != NO_ID) {
			vs0[i] = v;
			if (++i == 2)
				return;
		}
	}
}

void Mesh::GetAdjVertices(VIndex v, VertexIdxArr& indices) const
{
	ASSERT(vertexFaces.GetSize() == vertices.GetSize());
	const FaceIdxArr& idxFaces = vertexFaces[v];
	std::unordered_set<VIndex> setIndices;
	FOREACHPTR(pIdxFace, idxFaces) {
		const Face& face = faces[*pIdxFace];
		for (int i=0; i<3; ++i) {
			const VIndex vAdj(face[i]);
			if (vAdj != v && setIndices.insert(vAdj).second)
				indices.Insert(vAdj);
		}
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
			const uint32_t i(FindVertex(f, v));
			vertexInfo.AddEdge(f[(i+1)%3], f[(i+2)%3], *pFIdx);
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
			const uint32_t i(FindVertex(f, v));
			vertexInfo.AddEdge(f[(i+1)%3], f[(i+2)%3], *pFIdx);
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
	DEBUG_ULTIMATE("Fixed %u/%u non-manifold edges/vertices and %u faces removed: %u pyramid3 and %u pyramid4 (%s)", nNonManifoldEdges, nNonManifoldVertices, nRemoveFaces, nPyramid3, nPyramid4, TD_TIMER_GET_FMT().c_str());
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
		DEBUG_ULTIMATE("Mesh decimated: %d -> %d faces", OriginalFaceNum, TargetFaceNum);
	}

	// clean mesh
	{
		vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		const int nZeroAreaFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveZeroAreaFace(mesh);
		DEBUG_ULTIMATE("Removed %d zero-area faces", nZeroAreaFaces);
		const int nDuplicateFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateFace(mesh);
		DEBUG_ULTIMATE("Removed %d duplicate faces", nDuplicateFaces);
		const int nNonManifoldFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldFace(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold faces", nNonManifoldFaces);
		const int nDegenerateVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveDegenerateVertex(mesh);
		DEBUG_ULTIMATE("Removed %d degenerate vertices", nDegenerateVertices);
		#if 0
		const int nDuplicateVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateVertex(mesh);
		DEBUG_ULTIMATE("Removed %d duplicate vertices", nDuplicateVertices);
		#endif
		const int nUnreferencedVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveUnreferencedVertex(mesh);
		DEBUG_ULTIMATE("Removed %d unreferenced vertices", nUnreferencedVertices);
		const int nNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldVertex(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold vertices", nNonManifoldVertices);
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
		DEBUG_ULTIMATE("Removed %d faces with edges longer than %f", numLongFaces, thLongEdge);
		// remove isolated components
		vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		const std::pair<int, int> delInfo(vcg::tri::Clean<CLEAN::Mesh>::RemoveSmallConnectedComponentsDiameter(mesh, thLongEdge*1.5f));
		DEBUG_ULTIMATE("Removed %d connected components out of %d", delInfo.second, delInfo.first);
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
		DEBUG_ULTIMATE("Removed %d spikes", nTotalSpikes);
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
		DEBUG_ULTIMATE("Closed %d holes and added %d new faces", holeCnt, mesh.fn-OriginalSize);
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
		DEBUG_ULTIMATE("Smoothed %d vertices", mesh.vn);
	}

	// clean mesh
	if (fSpurious > 0 || bRemoveSpikes || nCloseHoles > 0 || nSmooth > 0) {
		const int nNonManifoldFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldFace(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold faces", nNonManifoldFaces);
		const int nNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldVertex(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold vertices", nNonManifoldVertices);
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
	struct VertexNormal {
		Mesh::Vertex v;
		Mesh::Normal n;
	};
	static const PLY::PlyProperty vert_normal_props[] = {
		{ "x", PLY::Float32, PLY::Float32, offsetof(VertexNormal,v.x), 0, 0, 0, 0},
		{ "y", PLY::Float32, PLY::Float32, offsetof(VertexNormal,v.y), 0, 0, 0, 0},
		{ "z", PLY::Float32, PLY::Float32, offsetof(VertexNormal,v.z), 0, 0, 0, 0},
		{"nx", PLY::Float32, PLY::Float32, offsetof(VertexNormal,n.x), 0, 0, 0, 0},
		{"ny", PLY::Float32, PLY::Float32, offsetof(VertexNormal,n.y), 0, 0, 0, 0},
		{"nz", PLY::Float32, PLY::Float32, offsetof(VertexNormal,n.z), 0, 0, 0, 0}
	};
	// list of property information for a face
	struct Face {
		uint8_t num;
		Mesh::Face* pFace;
	};
	static const PLY::PlyProperty face_props[] = {
		{"vertex_indices", PLY::Uint32, PLY::Uint32, offsetof(Face,pFace), 1, PLY::Uint8, PLY::Uint8, offsetof(Face,num)}
	};
	struct TexCoord {
		uint8_t num;
		Mesh::TexCoord* pTex;
	};
	struct FaceTex {
		Face face;
		TexCoord tex;
	};
	static const PLY::PlyProperty face_tex_props[] = {
		{"vertex_indices", PLY::Uint32, PLY::Uint32, offsetof(FaceTex,face.pFace), 1, PLY::Uint8, PLY::Uint8, offsetof(FaceTex,face.num)},
		{"texcoord", PLY::Float32, PLY::Float32, offsetof(FaceTex,tex.pTex), 1, PLY::Uint8, PLY::Uint8, offsetof(FaceTex,tex.num)}
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
	BasicPLY::Face face;
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
				ply.get_element(&face);
				if (face.num != 3) {
					DEBUG_EXTRA("error: unsupported mesh file (face not triangle)");
					return false;
				}
				memcpy(pFace, face.pFace, sizeof(VIndex)*3);
				delete[] face.pFace;
			}
		} else {
			ply.get_other_element();
		}
	}
	return true;
} // Load
/*----------------------------------------------------------------*/

// export the mesh as a PLY file
bool Mesh::Save(const String& fileName, const cList<String>& comments, bool bBinary) const
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

	// export comments
	FOREACHPTR(pStr, comments)
		ply.append_comment(pStr->c_str());

	// export texture file name as comment if needed
	String textureFileName;
	if (!faceTexcoords.IsEmpty() && !textureDiffuse.empty()) {
		textureFileName = Util::getFullFileName(fileName)+_T(".png");
		ply.append_comment((_T("TextureFile ")+Util::getFileFullName(textureFileName)).c_str());
	}

	if (vertexNormals.IsEmpty()) {
		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[0], 3, BasicPLY::vert_props);

		// export the array of vertices
		FOREACHPTR(pVert, vertices)
			ply.put_element(pVert);
	} else {
		ASSERT(vertices.GetSize() == vertexNormals.GetSize());

		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[0], 6, BasicPLY::vert_normal_props);

		// export the array of vertices
		BasicPLY::VertexNormal vn;
		FOREACH(i, vertices) {
			vn.v = vertices[i];
			vn.n = vertexNormals[i];
			ply.put_element(&vn);
		}
	}
	if (ply.get_current_element_count() == 0)
		return false;

	if (faceTexcoords.IsEmpty()) {
		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[1], 1, BasicPLY::face_props);

		// export the array of faces
		BasicPLY::Face face = {3};
		FOREACHPTR(pFace, faces) {
			face.pFace = pFace;
			ply.put_element(&face);
		}
	} else {
		ASSERT(faceTexcoords.GetSize() == faces.GetSize()*3);

		// describe what properties go into the vertex elements
		ply.describe_property(BasicPLY::elem_names[1], 2, BasicPLY::face_tex_props);

		// export the array of faces
		BasicPLY::FaceTex face = {{3},{6}};
		FOREACH(f, faces) {
			face.face.pFace = faces.Begin()+f;
			face.tex.pTex = faceTexcoords.Begin()+f*3;
			ply.put_element(&face);
		}

		// export the texture
		if (!textureDiffuse.empty())
			textureDiffuse.Save(textureFileName);
	}
	if (ply.get_current_element_count() == 0)
		return false;

	// write to file
	return ply.header_complete();
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
	return ply.header_complete();
}
/*----------------------------------------------------------------*/



// Ensure edge size and improve vertex valence;
// inspired by TransforMesh library of Andrei Zaharescu (cooperz@gmail.com)
// Code: https://scm.gforge.inria.fr/anonscm/svn/mvviewer
// Paper: http://perception.inrialpes.fr/Publications/2007/ZBH07/

#include <CGAL/Simple_cartesian.h>

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#define CURVATURE_TH 0 // 0.1
#define ROBUST_NORMALS 0 // 4
#define ENSURE_MIN_AREA 0 // 2
#define SPLIT_BORDER_EDGES 1
#define STRONG_EDGE_COLLAPSE_CHECK 0

namespace CLN {
typedef CGAL::Simple_cartesian<double>                 Kernel;
typedef Kernel::Point_3                                Point;
typedef Kernel::Vector_3                               Vector;
typedef Kernel::Triangle_3                             Triangle;
typedef Kernel::Plane_3	                               Plane;

template <class Refs, class T, class P, class Normal>
class MeshVertex : public CGAL::HalfedgeDS_vertex_base<Refs, T, P>
{
public:
	typedef CGAL::HalfedgeDS_vertex_base<Refs, T, P> Base;

	enum FALGS {
		FLG_EULER = (1 << 0), // Euler operations
		FLG_BORDER = (1 << 1), // border edge
	};
	Flags flags;

	Normal normal;
	Normal laplacian;
	Normal laplacian_deriv;
	#if CURVATURE_TH>0
	float mean_curvature;
	#endif

	MeshVertex() {}
	MeshVertex(const P& pt) : CGAL::HalfedgeDS_vertex_base<Refs, T, P>(pt) {}

	void setBorder() { flags.set(FLG_BORDER); }
	void unsetBorder() { flags.unset(FLG_BORDER); }
	bool isBorder() const { return flags.isSet(FLG_BORDER); }

	void move(const Vector& offset) {
		if (isBorder()) return;
		this->point() = this->point() + offset;
	}
};

template <class Refs, class T, class Normal>
class MeshFacet : public CGAL::HalfedgeDS_face_base<Refs, T>
{
public:
	typedef CGAL::HalfedgeDS_face_base<Refs, T>  Base;
	typedef typename Refs::Vertex_handle         Vertex_handle;
	typedef typename Refs::Vertex_const_handle   Vertex_const_handle;
	typedef typename Refs::Halfedge_handle       Halfedge_handle;
	typedef typename Refs::Halfedge_const_handle Halfedge_const_handle;
	typedef typename Refs::Face_handle           Face_handle;
	typedef typename Refs::Face_const_handle     Face_const_handle;

	char removal_status; // for self intersection removal: U -  unvisited; 'P' - partially valid; V - valid
						 // for connected components: U - unvisited; V - visited

	MeshFacet() : removal_status('U') {}

	inline bool isTrinagle() const {
		return this->halfedge()->vertex() == this->halfedge()->next()->next()->next()->vertex();
	}

	inline Triangle triangle() const {
		ASSERT(isTrinagle());
		return Triangle(get_point(0), get_point(1), get_point(2));
	}

	inline Point center() const {
		ASSERT(isTrinagle());
		return baricentric(0.333f, 0.333f);
	}

	inline Point baricentric(float u1, float u2) const {
		ASSERT(isTrinagle());
		Point p[3];
		Point result;
		p[0] = get_point(0);
		p[1] = get_point(1);
		p[2] = get_point(2);
		return Point(p[0].x()*u1 + p[1].x()*u2 + p[2].x()*(1.f - u1 -u2),
					 p[0].y()*u1 + p[1].y()*u2 + p[2].y()*(1.f - u1 -u2),
					 p[0].z()*u1 + p[1].z()*u2 + p[2].z()*(1.f - u1 -u2));

	}

	inline Halfedge_const_handle get_edge(int index) const {
		ASSERT(isTrinagle());
		switch (index) {
		case 0: return this->halfedge();
		case 1: return this->halfedge()->next();
		case 2: return this->halfedge()->next()->next();
		}
		ASSERT("invalid index" == NULL);
		return Halfedge_const_handle();
	}
	inline Halfedge_handle get_edge(int index) {
		ASSERT(isTrinagle());
		switch (index) {
		case 0: return this->halfedge();
		case 1: return this->halfedge()->next();
		case 2: return this->halfedge()->next()->next();
		}
		ASSERT("invalid index" == NULL);
		return Halfedge_handle();
	}
	inline Vertex_const_handle get_vertex(int index) const {
		return get_edge(index)->vertex();
	}
	inline Vertex_handle get_vertex(int index) {
		return get_edge(index)->vertex();
	}
	inline Point get_point(int index) const {
		return get_edge(index)->vertex()->point();
	}

	inline double edgeStatistics(int mode=1) const { // 0 - min; 1-avg; 2-max
		ASSERT(isTrinagle());
		const double e1(v_norm(get_point(0)-get_point(1)));
		const double e2(v_norm(get_point(0)-get_point(2)));
		const double e3(v_norm(get_point(1)-get_point(2)));
		switch (mode) {
		case 0: return MINF3(e1, e2, e3);
		case 1: return (e1+e2+e3) / 3;
		case 2: return MAXF3(e1, e2, e3);
		}
		ASSERT("invalid mode" == NULL);
		return 0;
	}

	inline double edgeMin(Halfedge_handle& h) {
		ASSERT(isTrinagle());
		const double e1(v_norm(get_point(0)-get_point(2)));
		const double e2(v_norm(get_point(1)-get_point(0)));
		const double e3(v_norm(get_point(2)-get_point(1)));
		if (e1 < e2) {
			if (e1 < e3) {
				h = get_edge(0);
				return e1;
			} else {
				h = get_edge(2);
				return e3;
			}
		} else {
			if (e2 < e3) {
				h = get_edge(1);
				return e2;
			} else {
				h = get_edge(2);
				return e3;
			}
		}
	}

	inline Normal normal() const {
		return CGAL::cross_product(get_point(1)-get_point(0), get_point(2)-get_point(0));
	}

	inline double area() const {
		return v_norm(normal())/2;
	}
};

class MeshItems : public CGAL::Polyhedron_items_3
{
public:
	template <class Refs, class Traits>
	struct Vertex_wrapper {
		typedef typename Traits::Point_3  Point;
		typedef typename Traits::Vector_3 Normal;
		typedef MeshVertex<Refs, CGAL::Tag_true, Point, Normal> Vertex;
	};
	template <class Refs, class Traits>
	struct Face_wrapper {
		typedef typename Traits::Vector_3 Normal;
		typedef MeshFacet<Refs, CGAL::Tag_true, Normal> Face;
	};
};

typedef CGAL::Polyhedron_3<Kernel, MeshItems>          Polyhedron;

typedef Polyhedron::Vertex                             Vertex;
typedef Polyhedron::Facet                              Facet;
typedef Polyhedron::Halfedge                           Halfedge;

typedef Polyhedron::Vertex_iterator                    Vertex_iterator;
typedef Polyhedron::Vertex_const_iterator              Vertex_const_iterator;
typedef Polyhedron::Facet_iterator                     Facet_iterator;
typedef Polyhedron::Facet_const_iterator               Facet_const_iterator;

typedef Polyhedron::Point_iterator                     Point_iterator;
typedef Polyhedron::Point_const_iterator               Point_const_iterator;
typedef Polyhedron::Edge_iterator                      Edge_iterator;
typedef Polyhedron::Edge_const_iterator                Edge_const_iterator;
typedef Polyhedron::Halfedge_iterator                  Halfedge_iterator;
typedef Polyhedron::Halfedge_const_iterator            Halfedge_const_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator   HF_circulator;
typedef Polyhedron::Halfedge_around_vertex_circulator  HV_circulator;

inline double v_norm(const Vector& A) {
	return SQRT(A*A); // operator * is overloaded as dot product
}
inline Vector v_normalized(const Vector& A) {
	const double nrmSq(A*A);
	return (nrmSq==0 ? A : A / SQRT(nrmSq));
}
inline double v_angle(const Vector& A, const Vector& B) {
	return acos(MAXF(-1.0, MINF(1.0, v_normalized(A)*v_normalized(B))));
}
inline double p_angle(const Point& A, const Point& B, const Point& C) {
	return v_angle(A-B, C-B);
}
#define edge_size(h) v_norm((h)->vertex()->point() - (h)->next()->next()->vertex()->point())

struct Stats {
	double min, avg, stdDev, max;
};
static void ComputeStatsArea(const Polyhedron& p, Stats& stats)
{
	MeanStd<double> mean;
	stats.min = FLT_MAX;
	stats.max = 0;
	for (Facet_const_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++) {
		const double tmpArea(fi->area());
		if (stats.min > tmpArea)
			stats.min = tmpArea;
		if (stats.max < tmpArea)
			stats.max = tmpArea;
		mean.Update(tmpArea);
	}
	stats.avg = mean.GetMean();
	stats.stdDev = mean.GetStdDev();
}
static void ComputeStatsEdge(const Polyhedron& p, Stats& stats)
{
	MeanStd<double> mean;
	stats.min = FLT_MAX;
	stats.max = 0;
	for (Edge_const_iterator ei = p.edges_begin(); ei!=p.edges_end(); ei++) {
		const double tmpEdge(v_norm(ei->vertex()->point() - ei->prev()->vertex()->point()));
		if (stats.min > tmpEdge)
			stats.min = tmpEdge;
		if (stats.max < tmpEdge)
			stats.max = tmpEdge;
		mean.Update(tmpEdge);
	}
	stats.avg = mean.GetMean();
	stats.stdDev = mean.GetStdDev();
}
static void ComputeStatsLaplacian(const Polyhedron& p, Stats& stats)
{
	MeanStd<double> mean;
	stats.min = FLT_MAX;
	stats.max = 0;
	for (Vertex_const_iterator vi = p.vertices_begin(); vi !=p.vertices_end(); vi++) {
		double tmpNorm(v_norm(vi->laplacian));
		if (vi->laplacian*vi->normal<0.f)
			tmpNorm = -tmpNorm;
		if (stats.min > tmpNorm)
			stats.min = tmpNorm;
		if (stats.max < tmpNorm)
			stats.max = tmpNorm;
		mean.Update(tmpNorm);
	}
	stats.avg = mean.GetMean();
	stats.stdDev = mean.GetStdDev();
}

inline double OppositeAngle(Vertex::Halfedge_handle h) {
	ASSERT(h->facet()->is_triangle());
	return v_angle(h->vertex()->point() - h->next()->vertex()->point(), h->prev()->vertex()->point() - h->next()->vertex()->point());
}

inline bool CanCollapseCenterVertex(Vertex::Vertex_handle v) {
	if (!v->is_trivalent()) return false;
	if (v->isBorder()) return false;
	return (v->halfedge()->prev()->opposite()->facet() != v->halfedge()->opposite()->next()->opposite()->facet());
}

#define REPLACE_POINT(V,P1,P2,PMIDDLE) (((V==P1)||(V==P2)) ? (PMIDDLE) : (V))
static bool CanCollapseEdge(Vertex::Halfedge_handle v0v1)
{
	Vertex::Halfedge_handle v1v0 = v0v1->opposite();
	Vertex::Vertex_handle v0 = v0v1->vertex();
	Vertex::Vertex_handle v1 = v1v0->vertex();

	if (v0v1->is_border_edge()) return false;
	if (v0->isBorder() || v1->isBorder()) return false;
	Vertex::Vertex_handle vl, vr, vTmp;
	Vertex::Halfedge_handle h1, h2;

	if (v0v1->next()->opposite()->facet() == v1v0->prev()->opposite()->facet()) return false;

	if (!v0v1->is_border()) {
		vl = v0v1->next()->vertex();
		h1 = v0v1->next();
		h2 = v0v1->next()->next();
		if (h1->is_border() || h2->is_border())
			return false;
	}

	if (!v1v0->is_border()) {
		vr = v1v0->next()->vertex();
		h1 = v1v0->next();
		h2 = v1v0->next()->next();
		if (h1->is_border() || h2->is_border())
			return false;
	}
	// if vl and vr are equal or both invalid -> fail
	if (vl == vr) return false;

	HV_circulator c, d;

	// test intersection of the one-rings of v0 and v1
	c = v0->vertex_begin(); d = c;
	CGAL_For_all(c, d)
		c->opposite()->vertex()->flags.unset(Vertex::FLG_EULER);

	c = v1->vertex_begin(); d = c;
	CGAL_For_all(c, d)
		c->opposite()->vertex()->flags.set(Vertex::FLG_EULER);

	c = v0->vertex_begin(); d = c;
	CGAL_For_all(c, d) {
		vTmp =c->opposite()->vertex();
		if (vTmp->flags.isSet(Vertex::FLG_EULER) && (vTmp!=vl) && (vTmp!=vr))
			return false;
	}

	// test weather when performing the edge collapse we change the signed area of any triangle
	#if STRONG_EDGE_COLLAPSE_CHECK==1
	Point p0 = v0->point();
	Point p1 = v1->point();
	Point p_middle = p0 + (p1 - p0) / 2;
	Point t1, t2, t3;
	Vector a1, a2;
	for (int x=0; x<2; x++) {
		if (x==0) {
			c = v0->vertex_begin(); d = c;
		} else {
			c = v1->vertex_begin(); d = c;
		}
		CGAL_For_all(c, d) {
			t1 = c->vertex()->point();
			t2 = c->next()->vertex()->point();
			t3 = c->next()->next()->vertex()->point();
			a1 = CGAL::cross_product(t2-t1, t3-t2);
			t1 = REPLACE_POINT(t1, p0, p1, p_middle);
			t2 = REPLACE_POINT(t2, p0, p1, p_middle);
			t3 = REPLACE_POINT(t3, p0, p1, p_middle);
			a2 = CGAL::cross_product(t2-t1, t3-t2);

			if ((v_norm(a2) != 0) && (v_angle(a1, a2) > PI/2))
				return false;
		}
	}
	#endif
	return true;
}
static void CollapseEdge(Polyhedron& p, Vertex::Halfedge_handle h)
{
	Vertex::Halfedge_handle h1 = h->next();
	Vertex::Halfedge_handle h2 = h->opposite()->prev();
	Point p1 = h->vertex()->point();
	Point p2 = h->opposite()->vertex()->point();
	Point p3 = p1 + (p2-p1) /2;
	size_t degree_p1 = h->vertex()->vertex_degree();
	size_t degree_p2 = h->opposite()->vertex()->vertex_degree();

	#if 0
	if (h->vertex()->isBorder())
		p3=p1;
	else if (h->opposite()->vertex()->isBorder())
		p3=p2;
	else
	#endif
	if (degree_p1 > degree_p2)
		p3=p1;
	else
		p3=p2;

	h->vertex()->point() = p3;

	p.join_facet(h1->opposite());
	p.join_facet(h2->opposite());
	p.join_vertex(h);
}

static bool CanFlipEdge(Vertex::Halfedge_handle h)
{
	if (h->is_border_edge()) return false;
	const Vertex::Halfedge_handle null_h;
	if ((h->next() == null_h) || (h->prev() == null_h) || (h->opposite() == null_h) || (h->opposite()->next() == null_h)) return false;
	Vertex::Vertex_handle v0 = h->next()->vertex();
	Vertex::Vertex_handle v1 = h->opposite()->next()->vertex();

	v0->flags.unset(Vertex::FLG_EULER);

	HV_circulator c = v1->vertex_begin();
	HV_circulator d = c;
	CGAL_For_all(c, d)
		c->opposite()->vertex()->flags.set(Vertex::FLG_EULER);

	if (v0->flags.isSet(Vertex::FLG_EULER)) return false;

	// check if it increases the quality overall
	double a1 = OppositeAngle(h);
	double a2 = OppositeAngle(h->next());
	double a3 = OppositeAngle(h->next()->next());
	double b1 = OppositeAngle(h->opposite());
	double b2 = OppositeAngle(h->opposite()->next());
	double b3 = OppositeAngle(h->opposite()->next()->next());

	if ((a1*a1 + b1*b1) / (a2*a2 + a3*a3 + b2*b2 + b3*b3) < 1.01) return false;

	Vector v_perp_1 = CGAL::cross_product(h->vertex()->point() - h->next()->vertex()->point(), h->vertex()->point() - h->prev()->vertex()->point());
	//Vector v_perp_2 = CGAL::cross_product(h->opposite()->vertex()->point()-h->opposite()->next()->vertex()->point(),h->opposite()->vertex()->point()-h->opposite()->prev()->vertex()->point());
	Vector v_perp_2 = CGAL::cross_product(h->opposite()->next()->vertex()->point() - h->opposite()->vertex()->point(), h->vertex()->point()-h->prev()->vertex()->point());
	if (v_angle(v_perp_1, v_perp_2) > D2R(20)) return false;

	return (h->next()->opposite()->facet() != h->opposite()->prev()->opposite()->facet()) &&
		(h->prev()->opposite()->facet() != h->opposite()->next()->opposite()->facet()) &&
		(CGAL::circulator_size(h->opposite()->vertex_begin()) >= 3) &&
		(CGAL::circulator_size(h->vertex_begin()) >= 3);
}
inline void FlipEdge(Polyhedron& p, Vertex::Halfedge_handle h) {
	p.flip_edge(h);
}

#if SPLIT_BORDER_EDGES>0
inline bool CanSplitEdge(Vertex::Halfedge_handle& h) {
	if (h->vertex()->point() == h->opposite()->vertex()->point())
		return false;
	if (h->face() == Vertex::Face_handle())
		h = h->opposite();
	return true;
}
#else
inline bool CanSplitEdge(Vertex::Halfedge_handle h) {
	if (h->is_border_edge()) return false;
	if (h->facet() == h->opposite()->facet()) return false;
	ASSERT(h->facet()->is_triangle() && h->opposite()->facet()->is_triangle());
	return (h->vertex()->point() != h->opposite()->vertex()->point());
}
#endif
// 1 - middle ; 2-projection of the 3rd vertex
static void SplitEdge(Polyhedron& p, Vertex::Halfedge_handle h, int mode=1)
{
	ASSERT(mode == 1 || mode == 2);
	Point p1 = h->vertex()->point();
	Point p2 = h->opposite()->vertex()->point();
	Point p3 = h->next()->vertex()->point();

	Point p_midddle;
	if (mode==1) { // middle
		const double ratio(0.5);
		p_midddle = p1 + (p2-p1) * ratio;
	} else { // projection of the 3rd vertex
		const double ratio(v_norm(p3-p2) * cos(OppositeAngle(h->next())) / v_norm(p1-p2));
		p_midddle = p2 + (p1-p2) * ratio;
	}

	Vertex::Halfedge_handle hnew = p.split_edge(h);
	hnew->vertex()->point() = p_midddle;

	p.split_facet(hnew, h->next());
	#if SPLIT_BORDER_EDGES>0
	if (h->opposite()->face() != Vertex::Face_handle())
	#endif
	p.split_facet(h->opposite(), hnew->opposite()->next());
}


// mode : 0 - min, 1 - avg, 2- max
static float ComputeVertexStatistics(Vertex& v, int mode)
{
	ASSERT((mode>=0) && (mode <=2));
	if (v.vertex_degree()==0) return 0;
	HV_circulator h = v.vertex_begin();
	float edge_stats((float)edge_size(h));
	int no_h(1);
	do {
		switch (mode) {
		case 0: edge_stats = MINF((float)edge_size(h), edge_stats); break;
		case 1: edge_stats += (float)edge_size(h), ++no_h; break;
		case 2: edge_stats = MAXF((float)edge_size(h), edge_stats); break;
		}
	} while (++h != v.vertex_begin());
	if (mode==1) edge_stats = (edge_stats/no_h);
	return edge_stats;
}

static int ImproveVertexValence(Polyhedron& p, int valence_mode=2)
{
	int total_no_ops(0);
	switch (valence_mode) {
	case 1: {
		//erase all the center triangles!
		for (Vertex_iterator vi=p.vertices_begin(); vi!=p.vertices_end(); ) {
			Vertex::Vertex_handle old_vi = vi;
			vi++;
			size_t degree = old_vi->vertex_degree();
			if (CanCollapseCenterVertex(old_vi))
				p.erase_center_vertex(old_vi->halfedge());
		}
		for (Vertex_iterator vi=p.vertices_begin(); vi!=p.vertices_end(); ) {
			Vertex::Vertex_handle old_vi = vi;
			vi++;
			size_t degree = old_vi->vertex_degree();
			//std::cout << "degree" << degree << std::endl;
			//if (CanCollapseCenterVertex(old_vi))
			//	p.erase_center_vertex(old_vi->halfedge());
			//else
			if (degree==4) {
				double edge_stats = ComputeVertexStatistics(*old_vi, 0); //min
				//std::cout << edge_stats << std::endl;
				HV_circulator c, d;
				c = old_vi->vertex_begin();
				float current_edge_stats;
				current_edge_stats = (float)edge_size(c);

				while (edge_stats!=current_edge_stats) {
					//std::cout << "current edge stats:" << current_edge_stats << std::endl;
					c++;
					current_edge_stats = (float)edge_size(c);
				}
				d = c;
				bool collapsed(false);
				CGAL_For_all(c, d) {
					if (CanCollapseEdge(c->opposite())) {
						//if ((c->opposite()->vertex()==vi) && (vi!=p.vertices_end())) vi++;
						CollapseEdge(p, c->opposite());
						collapsed = true;
						total_no_ops++;
						break;
					}
				}
				//if (!collapsed) std::cout << "could not collapse edge!" << std::endl;
			}
		}
	} break;

	case 2: {
		int iters(0), no_ops;
		do {
			iters++;
			no_ops = 0;
			for (Edge_iterator ei=p.edges_begin(); ei!=p.edges_end(); ) {
				if (ei->is_border_edge()) {
					ei++;
					continue;
				}
				int d1_1 = (int)ei->vertex()->vertex_degree();
				int d1_2 = (int)ei->opposite()->vertex()->vertex_degree();
				int d2_1 = (int)ei->next()->vertex()->vertex_degree();
				int d2_2 = (int)ei->opposite()->next()->vertex()->vertex_degree();
				Vertex::Halfedge_handle h = ei;
				ei++;
				if (((d1_1+d1_2) - (d2_1+d2_2) > 2) && CanFlipEdge(h)) {
					FlipEdge(p, h);
					no_ops++;
				}
			}
			//FixDegeneracy(p, 0.2,150);
			total_no_ops += no_ops;
		} while ((no_ops>0) && (iters<1));
	} break;

	case 3: {
		int iters(0), no_ops;
		do {
			iters++;
			no_ops = 0;
			for (Edge_iterator ei=p.edges_begin(); ei!=p.edges_end(); ) {
				if (ei->is_border_edge()) {
					ei++;
					continue;
				}
				Point p1=ei->vertex()->point();
				Point p2=ei->next()->vertex()->point();
				Point p3=ei->prev()->vertex()->point();
				Point p4=ei->opposite()->next()->vertex()->point();

				float cost1((float)MINF(MINF(MINF(MINF(MINF(p_angle(p3, p1, p2), p_angle(p1, p3, p2)), p_angle(p4, p1, p3)), p_angle(p4, p3, p1)), p_angle(p1, p4, p2)), p_angle(p1, p2, p3)));
				float cost2((float)MINF(MINF(MINF(MINF(MINF(p_angle(p1, p2, p4), p_angle(p1, p4, p2)), p_angle(p3, p4, p2)), p_angle(p4, p2, p3)), p_angle(p4, p1, p2)), p_angle(p4, p3, p2)));

				Vertex::Halfedge_handle h = ei;
				ei++;
				if ((cost2 > cost1) && CanFlipEdge(h)) {
					FlipEdge(p, h);
					no_ops++;
				}
			}
			//FixDegeneracy(p, 0.2,150);
			total_no_ops += no_ops;
		} while ((no_ops>0) && (iters<1));
	} break;
	}
	return total_no_ops;
}


static void UpdateMeshData(Polyhedron& p);

// Description: 
//  It iterates through all the mesh vertices and it tries to fix degenerate triangles.
//  There are conditions that check for large and small angles.
// Parameters:
//  - degenerateAngleDeg 
//     - for large angles: if an angle is bigger than degenerateAngleDeg.
//     - a good values to use is typically 170
//  - collapseRatio 
//     - for small angles: given the corresponding edges (a,b,c) in all permutations, if (a/b < collapseRatio) & (a/c < collapseRatio)
//     - a good value to use is 0.1 
static int FixDegeneracy(Polyhedron& p, double collapseRatio, double degenerateAngleDeg)
{
	DEBUG_LEVEL(3, "Fix degeneracy: %g collapse-ratio, %g degenerate-angle", collapseRatio, degenerateAngleDeg);
	double edge[3];
	int no_ops(0), counter(0);
	Vertex::Halfedge_handle edge_h[3];
	for (Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); ) {
		counter++;
		if (fi->triangle().is_degenerate()) {
			const double avg_edge(fi->edgeStatistics(1));
			DEBUG_LEVEL(3, "Degenerate angle: %g (avg edge %g)", v_angle(fi->get_point(0)-fi->get_point(1), fi->get_point(0)-fi->get_point(2)), avg_edge);
			const double delta(avg_edge*0.01);
			fi->halfedge()->vertex()->point() = fi->halfedge()->vertex()->point() + Vector(0, 0, delta);
			fi->halfedge()->next()->vertex()->point() = fi->halfedge()->next()->vertex()->point() + Vector(0, delta, 0);
			fi->halfedge()->next()->next()->vertex()->point() = fi->halfedge()->next()->next()->vertex()->point() + Vector(delta, 0, 0);
		}

		// collect facet statistics
		HF_circulator hf = fi->facet_begin();
		if (hf == NULL) continue;
		int i(0);
		do {
			ASSERT(ISFINITE(v_norm(hf->vertex()->point() - CGAL::ORIGIN)));
			ASSERT(i < 3); // a triangular mesh
			edge_h[i] = hf;
			edge[i++] = v_norm(hf->vertex()->point() - hf->prev()->vertex()->point());
		} while (++hf !=fi->facet_begin());

		fi++;
		//we should have the 3 sizes of the edges by now
		for (i=0; i<3; i++) {
			if ((edge[i]/edge[(i+1)%3] < collapseRatio) && (edge[i]/edge[(i+2)%3] < collapseRatio)) {
				if (CanCollapseEdge(edge_h[i])) {
					while ((fi!=p.facets_end()) && (fi==edge_h[i]->opposite()->facet())) fi++;
					CollapseEdge(p, edge_h[i]);
					no_ops++;
					break;
				}
				#if 0
				if (CanCollapseEdge(edge_h[i]->opposite())) {
					while ((fi!=p.facets_end()) && (fi==edge_h[i]->facet())) fi++;
					CollapseEdge(p, edge_h[i]->opposite());
					no_ops++;
					break;
				}
				#endif
			} else {
				const double tmpAngle(R2D(OppositeAngle(edge_h[i])));
				if (tmpAngle > degenerateAngleDeg && CanFlipEdge(edge_h[i])) {
					FlipEdge(p, edge_h[i]);
					no_ops++;
					break;
				}
				#if 0
				if (tmpAngle > degenerateAngleDeg && CanSplitEdge(edge_h[i])) {
					SplitEdge(p, edge_h[i], 2);
					if (CanCollapseEdge(edge_h[i]->prev())) {
						while ((fi!=p.facets_end()) && (fi==edge_h[i]->prev()->opposite()->facet())) fi++;
						CollapseEdge(p, edge_h[i]->prev());
						no_ops++;
					}
					no_ops++;
					break;
				}
				#endif
			}
		}
	}
	if (no_ops)
		UpdateMeshData(p);
	return no_ops;
}
static void FixAllDegeneracy(Polyhedron& p, double collapseRatio, double degenerateAngleDeg)
{
	int runs(0);
	do {
		if (FixDegeneracy(p, collapseRatio, degenerateAngleDeg) == 0)
			break;
		ImproveVertexValence(p);
	} while (runs++ < 3);
}


class MeshConnectedComponent {
public:
	Vertex::Facet_handle start_facet;
	int size;
	float area;
	float edge_min, edge_avg, edge_max;
	bool is_open;
	MeshConnectedComponent() {
		start_facet = NULL;
		size=0;
		edge_min=FLT_MAX;
		edge_max=0;
		edge_avg=0;
		area=0.f;
		is_open=false;
	}
	void setParams(Vertex::Facet_handle start_facet_, int size_) {
		start_facet = start_facet_;
		size = size_;
	}
	void updateStats(float edge_size) {
		edge_max=MAXF(edge_max, edge_size);
		edge_min=MINF(edge_min, edge_size);
		edge_avg+=edge_size;
	}
};
static void ComputeConnectedComponents(Polyhedron& p, std::vector<MeshConnectedComponent>& connected_components)
{
	connected_components.clear();
	std::queue<Vertex::Facet_handle> facet_queue;
	MeshConnectedComponent lastComponent;

	// reset the facet status
	for (Facet_iterator i = p.facets_begin(); i != p.facets_end(); ++i)
		i->removal_status = 'U';

	std::cout << "Connected components of: ";
	// traverse the mesh via facets
	for (Facet_iterator i = p.facets_begin(); i != p.facets_end(); ++i) {
		if (i->removal_status=='U') { // start a new component
			lastComponent.setParams(i, 0);
			i->removal_status='V'; // it is now visited
			facet_queue.push(i);
			while (!facet_queue.empty()) { // fill the current component
				Vertex::Facet_handle f = facet_queue.front(); facet_queue.pop();
				lastComponent.size++;
				HF_circulator h = f->facet_begin();
				do {
					if (h->is_border_edge()) continue;
					lastComponent.is_open=true;
					float edge_size = (float)v_norm(h->vertex()->point() - h->prev()->vertex()->point());
					lastComponent.updateStats(edge_size);
					lastComponent.area += (float)f->area();
					Vertex::Facet_handle opposite_f = h->opposite()->facet();
					if ((opposite_f!=Vertex::Facet_handle()) && (opposite_f->removal_status=='U')) {
						opposite_f->removal_status='V'; // it is now visited
						facet_queue.push(opposite_f);
					}

				} while (++h != f->facet_begin());
			} // done traversing the current component
			lastComponent.edge_avg/=lastComponent.size*3;
			connected_components.push_back(lastComponent);
			std::cout << lastComponent.size << " faces ";
		} // found a new component
	} // done traversing the mesh
	std::cout << "(" << connected_components.size() << " components)" << std::endl;
}
static void RemoveConnectedComponents(Polyhedron& p, int size_threshold, float edge_threshold)
{
	std::vector<MeshConnectedComponent> connected_components;
	ComputeConnectedComponents(p, connected_components);
	for (std::vector<MeshConnectedComponent>::iterator vi=connected_components.begin(); vi!=connected_components.end(); ) {
		if ((vi->size<=size_threshold) || (vi->edge_max<=edge_threshold) || ((vi->area<edge_threshold*edge_threshold*PI*2) && (vi->is_open==false))) {
			p.erase_connected_component(vi->start_facet->facet_begin());
			vi = connected_components.erase(vi);
		} else
			vi++;
	}
}

// mode : 0 - tangetial; 1 - across the normal
inline Vector ComputeVectorComponent(Vector n, Vector v, int mode)
{
	ASSERT((mode>=0) && (mode<2));
	Vector across_normal(n*(n*v));
	if (mode==1)
		return across_normal;
	else
		return v - across_normal;
}

static void Smooth(Polyhedron& p, double delta, int mode=0, bool only_visible=false)
{
	// 0 - both components;
	// 1 - tangetial;
	// 2 - normal;
	// 3 - second order;
	// 4 - combined;
	// 5 - tangential only if bigger than normal;
	// 6 - both components - laplacian_avg;
	ASSERT((mode>=0) && (mode<=6));
	Stats laplacian;
	if (mode==6)
		ComputeStatsLaplacian(p, laplacian);
	for (Vertex_iterator vi=p.vertices_begin(); vi!=p.vertices_end(); vi++) {
		Vector displacement;
		switch (mode) {
		case 0: displacement = vi->laplacian*delta/*vert->getMeanCurvatureFlow().Norm()*/; break;
		case 1: displacement = ComputeVectorComponent(vi->normal, vi->laplacian, 0) *delta; break;
		case 2: displacement = ComputeVectorComponent(vi->normal, vi->laplacian, 1) *delta; break;
		case 3: displacement = vi->laplacian_deriv*delta; break;
		case 4: displacement = vi->laplacian*delta - vi->laplacian_deriv*delta; break;
		case 5: {
			Vector d_tan(ComputeVectorComponent(vi->normal, vi->laplacian, 0)*delta);
			Vector d_norm(ComputeVectorComponent(vi->normal, vi->laplacian, 1)*delta);
			displacement = (v_norm(d_tan) > 2*v_norm(d_norm) ? d_tan : Vector(0, 0, 0));
		} break;
		case 6: displacement = vi->laplacian *delta - vi->normal*laplacian.avg*delta; break;
		}
		vi->move(displacement);
	}
	UpdateMeshData(p);
}


// Description: 
// - The goal of this method is to ensure that all the edges of the mesh are within the interval [epsilonMin,epsilonMax].
//   In order to do so, edge collapses and edge split operations are performed.
// - The method also attempts to fix degeneracies by invoking FixDegeneracy(collapseRatio,degenerate_angle_deg) and performs some local smoothing, based on the operating mode.
// Parameters:
// - [epsilonMin, epsilonMax] - the desired edge interval (negative if to be used as multiplier of the initial mean edge length)
// - collapseRatio, degenerate_angle_deg - parameters used to invoke FixDegeneracy (see function for more details)
// - mode : 0 - fixDegeneracy=No  smoothing=Yes;
//          1 - fixDegeneracy=Yes smoothing=Yes; (default)
//         10 - fixDegeneracy=Yes smoothing=No;
// - max_iter (default=30) - maximum number of iterations to be performed; since there is no guarantee that one operations (such as a collapse, for example)
//   will not in turn generate new degeneracies, operations are being performed on the mesh in an iterative fashion. 
static void EnsureEdgeSize(Polyhedron& p, double epsilonMin, double epsilonMax, double collapseRatio, double degenerate_angle_deg, int mode, int max_iters)
{
	if (mode>0)
		FixDegeneracy(p, collapseRatio, degenerate_angle_deg);

	#if ENSURE_MIN_AREA>0
	Stats area;
	ComputeStatsArea(p, area);
	const float thArea((float)area.avg/ENSURE_MIN_AREA);
	#endif

	Stats edge;
	ComputeStatsEdge(p, edge);
	if (epsilonMin < 0)
		epsilonMin = edge.avg * (-epsilonMin);
	if (epsilonMax < 0)
		epsilonMax = MAXF(edge.avg * (-epsilonMax), epsilonMin * 2);
	DEBUG_LEVEL(3, "Ensuring edge size in [%g, %g] with edges currently in [%g, %g]", epsilonMin, epsilonMax, edge.min, edge.max);

	typedef TIndexScore<Vertex::Halfedge_handle, float> EdgeScore;
	typedef CLISTDEF0(EdgeScore) EdgeScoreArr;
	EdgeScoreArr bigEdges(0, 1024);
	int iters(0), total_no_ops(0), no_ops(1);
	while ((edge.min<epsilonMin || edge.max>epsilonMax) && (no_ops>0) && (++iters<max_iters)) {
		no_ops = 0;
		if (iters > 1)
			ComputeStatsEdge(p, edge);

		// process big edges
		ASSERT(bigEdges.IsEmpty());
		for (Halfedge_iterator h = p.edges_begin(); h != p.edges_end(); ++h, ++h) {
			ASSERT(++Halfedge_iterator(h) == h->opposite());
			const double edgeSize(edge_size(h));
			if (edgeSize > epsilonMax)
				bigEdges.AddConstruct(h, (float)edgeSize);
		}
		DEBUG_LEVEL(3, "Big edges: %u", bigEdges.GetSize());
		bigEdges.Sort(); // process big edges first
		for (EdgeScoreArr::IDX i=0; i<bigEdges.GetSize(); ++i) {
			Vertex::Halfedge_handle h(bigEdges[i].idx);
			if (!CanSplitEdge(h))
				continue;
			#if CURVATURE_TH>0
			const float avg_mean_curv(MAXF(ABS(h->vertex()->mean_curvature), ABS(h->prev()->vertex()->mean_curvature)));
			if (avg_mean_curv < CURVATURE_TH)
				continue;
			#endif
			#if ENSURE_MIN_AREA>0
			if (h->facet()->area() < thArea)
				continue;
			#endif
			SplitEdge(p, h);
			no_ops++;
		}
		bigEdges.Empty();

		// process small edges
		Vertex::Halfedge_handle h;
		Facet_iterator f(p.facets_begin());
		while (f != p.facets_end()) {
			const double minEdge(f->edgeMin(h));
			f++;
			if (minEdge < epsilonMin && CanCollapseEdge(h)) {
				while ((f!=p.facets_end()) && ((f==h->opposite()->facet()) || (f==h->facet()))) f++;
				CollapseEdge(p, h);
				no_ops++;
			}
		}

		if (mode <= 10)
			ImproveVertexValence(p);
		if (mode == 10)
			FixDegeneracy(p, collapseRatio, degenerate_angle_deg);

		UpdateMeshData(p);
		if (mode < 10)
			Smooth(p, 0.1, 1);

		total_no_ops += no_ops;
	}
	if (mode > 0) {
		FixAllDegeneracy(p, collapseRatio, degenerate_angle_deg);
		if (mode <10)
			Smooth(p, 0.1, 1);
	}

	RemoveConnectedComponents(p, 100, (float)edge.min*2);

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2) {
		ComputeStatsEdge(p, edge);
		VERBOSE("Edge size in [%g, %g] (requested in [%g, %g]): %d ops, %d iters", edge.min, edge.max, epsilonMin, epsilonMax, total_no_ops, iters);
	}
	#endif	
}

static void ComputeVertexNormals(Polyhedron& p)
{
	for (Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++)
		vi->normal = CGAL::NULL_VECTOR;
	for (Facet_iterator fi = p.facets_begin(); fi!=p.facets_end(); fi++) {
		const Vector t(fi->normal());
		Vector& n0(fi->get_vertex(0)->normal); n0 = n0 + t;
		Vector& n1(fi->get_vertex(1)->normal); n1 = n1 + t;
		Vector& n2(fi->get_vertex(2)->normal); n2 = n2 + t;
	}
	for (Vertex_iterator vi = p.vertices_begin(); vi!=p.vertices_end(); vi++) {
		Vector& normal(vi->normal);
		const double nrm(v_norm(normal));
		#if ROBUST_NORMALS==0
		if (nrm != 0)
			normal = normal / nrm;
		#else
		// only temporarily set here, it will be set in normal when computing the robust measure
		vi->laplacian_deriv = (nrm != 0 ? (normal / nrm) : CGAL::NULL_VECTOR);
		#endif
	}
}
#if ROBUST_NORMALS>0
static std::vector< std::pair<Vertex*, int> > GetRingNeighbourhood(Vertex& v, int ring_size, bool include_original) {
	std::map<Vertex*, int> neigh_map;
	std::map<Vertex*, int>::iterator iter;

	std::queue<Vertex*> elems;
	std::vector< std::pair<Vertex*, int> > result;

	// add base level	
	elems.push(&v);
	neigh_map[&v]=0;

	if (ring_size < 0) return result;

	while (elems.size() > 0) {
		Vertex* el = elems.front(); elems.pop();
		if ((el != &v) || include_original)
			result.push_back(std::pair<Vertex*, int>(el, neigh_map[el]));
		if (neigh_map[el]==ring_size) continue;
		//circulate one ring neighborhood
		HV_circulator c = el->vertex_begin();
		HV_circulator d = c;
		CGAL_For_all(c, d) {
			Vertex* next_el = &(*(c->opposite()->vertex()));
			iter=neigh_map.find(next_el);
			if (iter == neigh_map.end()) { // if the vertex has not been already taken
				elems.push(next_el);
				neigh_map[next_el]=neigh_map[el]+1;
			}
		}
	}

	return result;
}
static void ComputeVertexRobustNormal(Vertex& v, int maxRings)
{
	std::vector< std::pair<Vertex*, int> > neighs(GetRingNeighbourhood(v, maxRings, true));
	Vector& normal(v.normal);
	normal = CGAL::NULL_VECTOR;
	for (std::vector< std::pair<Vertex*, int> >::const_iterator n_it=neighs.cbegin(); n_it!=neighs.cend(); ++n_it) {
		Vertex* neigh_v = n_it->first;
		//const float w = n_it->second / maxRings;
		normal = normal + neigh_v->laplacian_deriv;//*w;
	}
	const double nrm(v_norm(normal));
	if (nrm != 0)
		normal = normal / nrm;
}
#endif

static void ComputeVertexLaplacian(Vertex& v)
{
	// formula taken from "Mesh Smoothing via Mean and Median Filtering Applied to Face Normals"
	HV_circulator vi = v.vertex_begin();
	ASSERT(vi != NULL);
	Vector result_laplacian(0, 0, 0);
	#ifndef LAPLACIAN_ROBUST
	size_t order = 0;
	do {
		++order;
		if (vi->is_border_edge()) {
			v.laplacian = Vector(0, 0, 0);
			return;
		}
		result_laplacian = result_laplacian + (vi->prev()->vertex()->point() - CGAL::ORIGIN);
	} while (++vi != v.vertex_begin());
	result_laplacian = result_laplacian/(double)order - (v.point() - CGAL::ORIGIN);
	#else
	float w_total(0);
	Vector e, e_next, e_prev;
	do {
		e_next = vi->next()->vertex()->point() - vi->vertex()->point();
		e = vi->prev()->vertex()->point() - vi->vertex()->point();
		e_prev = vi->opposite()->next()->vertex()->point() - vi->vertex()->point();

		float theta_1((float)v_angle(e, e_next));
		float theta_2((float)v_angle(e, e_prev));
		float w((tan(theta_1/2)+tan(theta_2/2))/v_norm(e));

		w_total += w;
		result_laplacian = result_laplacian + w*e;
	} while (++vi != v.vertex_begin());
	result_laplacian = result_laplacian / (double)w_total;
	#endif
	v.laplacian = result_laplacian;
}
static void ComputeVertexLaplacianDeriv(Vertex& v)
{
	HV_circulator vi = v.vertex_begin();
	ASSERT(vi != NULL);
	v.laplacian_deriv = Vector(0, 0, 0);
	size_t order(0);
	do {
		++order;
		v.laplacian_deriv = v.laplacian_deriv + (vi->prev()->vertex()->laplacian-v.laplacian);
	} while (++vi != v.vertex_begin());
	v.laplacian_deriv = v.laplacian_deriv / (double)order;
}

#if CURVATURE_TH>0
static void ComputeVertexCurvature(Vertex& v)
{
	float edge_avg(ComputeVertexStatistics(v, 2));
	float mean_curv(ABS((float)v_norm(v.laplacian) / edge_avg));
	if (v.laplacian*v.normal<0)
		mean_curv = -mean_curv;
	if (!ISFINITE(mean_curv))
		mean_curv = 0;
	v.mean_curvature = mean_curv;
}
#endif

static void UpdateMeshData(Polyhedron& p)
{
	p.normalize_border();

	// compute vertex normal
	ComputeVertexNormals(p);
	#if ROBUST_NORMALS>0
	// compute robust vertex normal		
	for (Vertex_iterator vi=p.vertices_begin(); vi!=p.vertices_end(); vi++)
		ComputeVertexRobustNormal(*vi, ROBUST_NORMALS);
	#endif

	// compute Laplacians
	for (Vertex_iterator vi=p.vertices_begin(); vi!=p.vertices_end(); vi++) {
		vi->unsetBorder();
		ComputeVertexLaplacian(*vi);
	}

	// compute curvature and Laplacian derivative
	for (Vertex_iterator vi=p.vertices_begin(); vi!=p.vertices_end(); vi++) {
		#if CURVATURE_TH>0
		ComputeVertexCurvature(*vi);
		#endif
		ComputeVertexLaplacianDeriv(*vi);
	}

	// set border edges	
	for (Halfedge_iterator hi=p.border_halfedges_begin(); hi!=p.halfedges_end(); hi++)
		hi->vertex()->setBorder();
}

// a modifier creating a triangle with the incremental builder
template <class HDS, class K>
class TMeshBuilder : public CGAL::Modifier_base<HDS>
{
public:
	const Mesh::VertexArr& vertices;
	const Mesh::FaceArr& faces;
	bool bProblems;

	TMeshBuilder(const Mesh::VertexArr& _vertices, const Mesh::FaceArr& _faces) : vertices(_vertices), faces(_faces), bProblems(false) {}

	void operator() (HDS& hds) {
		typedef typename HDS::Vertex::Point Point;
		CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, false);
		B.begin_surface(vertices.GetSize(), faces.GetSize());
		// add the vertices		
		FOREACH(i, vertices) {
			const Mesh::Vertex& v = vertices[i];
			B.add_vertex(Point(v.x, v.y, v.z));
		}
		// add the facets
		FOREACH(i, faces) {
			const Mesh::Face& f = faces[i];
			if (!B.test_facet(f.ptr(), f.ptr()+3)) {
				if (!bProblems) {
					std::cout << "WARNING: following facet(s) violate the manifold constraint (will be ignored):";
					bProblems=true;
				}
				std::cout << " " << i;
				std::cout.flush();
				continue;
			}
			B.add_facet(f.ptr(), f.ptr()+3);
		}
		if (bProblems)
			std::cout << std::endl;
		//if (B.check_unconnected_vertices()) {
		//	std::cout << "WARNING: unconnected vertices exists. They will be removed" << std::endl;
		//	B.remove_unconnected_vertices();
		//}
		B.end_surface();
	}
};
typedef TMeshBuilder<Polyhedron::HalfedgeDS, Kernel> MeshBuilder;
static bool ImportMesh(Polyhedron& p, const Mesh::VertexArr& vertices, const Mesh::FaceArr& faces) {
	MeshBuilder builder(vertices, faces);
	p.delegate(builder);
	if (builder.bProblems)
		return false;
	UpdateMeshData(p);
	DEBUG_ULTIMATE("Mesh imported: %u vertices, %u facets (%u border edges)", p.size_of_vertices(), p.size_of_facets(), p.size_of_border_edges());
	return true;
}
static bool ExportMesh(const Polyhedron& p, Mesh::VertexArr& vertices, Mesh::FaceArr& faces) {
	if (p.size_of_vertices() >= std::numeric_limits<Mesh::VIndex>::max())
		return false;
	if (p.size_of_facets() >= std::numeric_limits<Mesh::FIndex>::max())
		return false;
	unsigned nCount;
	// extract vertices
	nCount = 0;
	vertices.Resize((Mesh::VIndex)p.size_of_vertices());
	for (Polyhedron::Vertex_const_iterator it=p.vertices_begin(), ite=p.vertices_end(); it!=ite; ++it) {
		Mesh::Vertex& v = vertices[nCount++];
		v.x = (float)it->point().x();
		v.y = (float)it->point().y();
		v.z = (float)it->point().z();
	}
	// extract the faces
	nCount = 0;
	faces.Resize((Mesh::FIndex)p.size_of_facets());
	for (Polyhedron::Face_const_iterator it=p.facets_begin(), ite=p.facets_end(); it!=ite; ++it) {
		ASSERT(it->is_triangle());
		Polyhedron::Halfedge_around_facet_const_circulator j = it->facet_begin();
		ASSERT(CGAL::circulator_size(j) == 3);
		Mesh::Face& facet = faces[nCount++];
		unsigned i(0);
		do {
			facet[i++] = (Mesh::FIndex)std::distance(p.vertices_begin(), j->vertex());
		} while (++j != it->facet_begin());
	}
	DEBUG_ULTIMATE("Mesh exported: %u vertices, %u facets (%u border edges)", p.size_of_vertices(), p.size_of_facets(), p.size_of_border_edges());
	return true;
}
} // namespace CLN

void Mesh::EnsureEdgeSize(float epsilonMin, float epsilonMax, float collapseRatio, float degenerate_angle_deg, int mode, int max_iters)
{
	CLN::Polyhedron p;
	CLN::ImportMesh(p, vertices, faces);
	Release();
	CLN::EnsureEdgeSize(p, epsilonMin, epsilonMax, collapseRatio, degenerate_angle_deg, mode, max_iters);
	CLN::ExportMesh(p, vertices, faces);
}
/*----------------------------------------------------------------*/
