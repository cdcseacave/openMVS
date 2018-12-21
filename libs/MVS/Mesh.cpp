/*
* Mesh.cpp
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
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
*/

#include "Common.h"
#include "Mesh.h"
// fix non-manifold vertices
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>
#ifdef _MSC_VER
#  pragma warning(push)
#  pragma warning(disable: 4244 4267 4305)
#endif
// VCG: mesh reconstruction post-processing
#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/polygon_support.h>
// VCG: mesh simplification
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/selection.h>
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#ifdef _MSC_VER
#  pragma warning(pop)
#endif

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
	vertexVertices.Release();
	vertexFaces.Release();
	vertexBoundary.Release();
	faceNormals.Release();
	faceTexcoords.Release();
	textureDiffuse.release();
} // ReleaseExtra
void Mesh::EmptyExtra()
{
	vertexNormals.Empty();
	vertexVertices.Empty();
	vertexFaces.Empty();
	vertexBoundary.Empty();
	faceNormals.Empty();
	faceTexcoords.Empty();
	textureDiffuse.release();
} // EmptyExtra
/*----------------------------------------------------------------*/


// compute the axis-aligned bounding-box of the mesh
Mesh::Box Mesh::GetAABB() const
{
	Box box(true);
	for (const Vertex& X: vertices)
		box.InsertFull(X);
	return box;
}
// same, but only for vertices inside the given AABB
Mesh::Box Mesh::GetAABB(const Box& bound) const
{
	Box box(true);
	for (const Vertex& X: vertices)
		if (bound.Intersects(X))
			box.InsertFull(X);
	return box;
}
/*----------------------------------------------------------------*/


// extract array of vertices incident to each vertex
void Mesh::ListIncidenteVertices()
{
	vertexVertices.Empty();
	vertexVertices.Resize(vertices.GetSize());
	FOREACH(i, faces) {
		const Face& face = faces[i];
		for (int v=0; v<3; ++v) {
			VertexIdxArr& verts(vertexVertices[face[v]]);
			for (int i=1; i<3; ++i) {
				const VIndex idxVert(face[(v+i)%3]);
				if (verts.Find(idxVert) == VertexIdxArr::NO_INDEX)
					verts.Insert(idxVert);
			}
		}
	}
}

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
	#ifndef _USE_CUDA
	FOREACH(idxFace, faces)
		faceNormals[idxFace] = normalized(FaceNormal(faces[idxFace]));
	#else
	reportCudaError(kernelComputeFaceNormal((int)faces.GetSize(),
		vertices,
		faces,
		CUDA::KernelRT::OutputParam(faceNormals.GetDataSize()),
		faces.GetSize()
	));
	reportCudaError(kernelComputeFaceNormal.GetResult(0,
		faceNormals
	));
	kernelComputeFaceNormal.Reset();
	#endif
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
			afaces.InsertSortUnique(*pIdxFace);
	}
	FOREACHPTR(pIdxFace, faces2) {
		if (f != *pIdxFace && setFaces.find(*pIdxFace) != setFaces.end())
			afaces.InsertSortUnique(*pIdxFace);
	}
	setFaces.clear();
	setFaces.insert(faces2.Begin(), faces2.End());
	FOREACHPTR(pIdxFace, faces0) {
		if (f != *pIdxFace && setFaces.find(*pIdxFace) != setFaces.end())
			afaces.InsertSortUnique(*pIdxFace);
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

void Mesh::GetAdjVertexFaces(VIndex idxVCenter, VIndex idxVAdj, FaceIdxArr& indices) const
{
	ASSERT(vertexFaces.GetSize() == vertices.GetSize());
	const FaceIdxArr& idxFaces = vertexFaces[idxVCenter];
	FOREACHPTR(pIdxFace, idxFaces) {
		const Face& face = faces[*pIdxFace];
		ASSERT(FindVertex(face, idxVCenter) != NO_ID);
		if (FindVertex(face, idxVAdj) != NO_ID)
			indices.Insert(*pIdxFace);
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
		const Graph* graph;
		const VertexSet* filterVerts;
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
		if (vFaces.GetSize() < 2)
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
void Mesh::Clean(float fDecimate, float fSpurious, bool bRemoveSpikes, unsigned nCloseHoles, unsigned nSmooth, bool bLastClean)
{
	if (vertices.IsEmpty() || faces.IsEmpty())
		return;
	TD_TIMER_STARTD();
	// create VCG mesh
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
			ASSERT(f[0]<(uint32_t)mesh.vn);
			(*fi).V(0) = indices[f[0]];
			ASSERT(f[1]<(uint32_t)mesh.vn);
			(*fi).V(1) = indices[f[1]];
			ASSERT(f[2]<(uint32_t)mesh.vn);
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
		const int nUnreferencedVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveUnreferencedVertex(mesh);
		DEBUG_ULTIMATE("Removed %d unreferenced vertices", nUnreferencedVertices);
		#if 1
		const int nDuplicateVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateVertex(mesh);
		DEBUG_ULTIMATE("Removed %d duplicate vertices", nDuplicateVertices);
		#endif
		#if 1
		vcg::tri::Allocator<CLEAN::Mesh>::CompactFaceVector(mesh);
		vcg::tri::Allocator<CLEAN::Mesh>::CompactVertexVector(mesh);
		for (int i=0; i<10; ++i) {
			vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
			vcg::tri::UpdateTopology<CLEAN::Mesh>::VertexFace(mesh);
			const int nSplitNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::SplitNonManifoldVertex(mesh, 0.1f);
			DEBUG_ULTIMATE("Split %d non-manifold vertices", nSplitNonManifoldVertices);
			if (nSplitNonManifoldVertices == 0)
				break;
		}
		#else
		const int nNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldVertex(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold vertices", nNonManifoldVertices);
		vcg::tri::Allocator<CLEAN::Mesh>::CompactFaceVector(mesh);
		vcg::tri::Allocator<CLEAN::Mesh>::CompactVertexVector(mesh);
		#endif
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
		const float thLongEdge(edgeLens.GetNth(edgeLens.GetSize()*95/100)*fSpurious);
		#endif
		// remove faces with too long edges
		const size_t numLongFaces(vcg::tri::UpdateSelection<CLEAN::Mesh>::FaceOutOfRangeEdge(mesh, 0, thLongEdge));
		for (CLEAN::Mesh::FaceIterator fi=mesh.face.begin(); fi!=mesh.face.end(); ++fi)
			if (!(*fi).IsD() && (*fi).IsS())
				vcg::tri::Allocator<CLEAN::Mesh>::DeleteFace(mesh, *fi);
		DEBUG_ULTIMATE("Removed %d faces with edges longer than %f", numLongFaces, thLongEdge);
		// remove isolated components
		vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		const std::pair<int, int> delInfo(vcg::tri::Clean<CLEAN::Mesh>::RemoveSmallConnectedComponentsDiameter(mesh, thLongEdge));
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
		vcg::tri::UpdateNormal<CLEAN::Mesh>::PerFaceNormalized(mesh);
		vcg::tri::UpdateNormal<CLEAN::Mesh>::PerVertexAngleWeighted(mesh);
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
	if (bLastClean && (fSpurious > 0 || bRemoveSpikes || nCloseHoles > 0 || nSmooth > 0)) {
		const int nNonManifoldFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldFace(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold faces", nNonManifoldFaces);
		#if 0 // not working
		vcg::tri::Allocator<CLEAN::Mesh>::CompactEveryVector(mesh);
		vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		vcg::tri::UpdateTopology<CLEAN::Mesh>::VertexFace(mesh);
		const int nSplitNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::SplitNonManifoldVertex(mesh, 0.1f);
		DEBUG_ULTIMATE("Split %d non-manifold vertices", nSplitNonManifoldVertices);
		#else
		const int nNonManifoldVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldVertex(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold vertices", nNonManifoldVertices);
		#endif
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

// import the mesh from the given file
bool Mesh::Load(const String& fileName)
{
	TD_TIMER_STARTD();
	const String ext(Util::getFileExt(fileName).ToLower());
	bool ret;
	if (ext == _T(".obj"))
		ret = LoadOBJ(fileName);
	else
		ret = LoadPLY(fileName);
	if (!ret)
		return false;
	DEBUG_EXTRA("Mesh loaded: %u vertices, %u faces (%s)", vertices.GetSize(), faces.GetSize(), TD_TIMER_GET_FMT().c_str());
	return true;
}
// import the mesh as a PLY file
bool Mesh::LoadPLY(const String& fileName)
{
	ASSERT(!fileName.IsEmpty());
	Release();

	// open PLY file and read header
	PLY ply;
	if (!ply.read(fileName)) {
		DEBUG_EXTRA("error: invalid PLY file");
		return false;
	}
	for (int i = 0; i < (int)ply.elems.size(); ++i) {
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
	for (int i = 0; i < (int)ply.elems.size(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			ASSERT(vertices.GetSize() == (VIndex)elem_count);
			ply.setup_property(BasicPLY::vert_props[0]);
			ply.setup_property(BasicPLY::vert_props[1]);
			ply.setup_property(BasicPLY::vert_props[2]);
			FOREACHPTR(pVert, vertices)
				ply.get_element(pVert);
		} else
		if (PLY::equal_strings(BasicPLY::elem_names[1], elem_name)) {
			ASSERT(faces.GetSize() == (FIndex)elem_count);
			if (ply.find_property(ply.elems[i], BasicPLY::face_tex_props[1].name.c_str()) == -1) {
				// load vertex indices
				BasicPLY::Face face;
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
				// load vertex indices and texture coordinates
				faceTexcoords.Resize((FIndex)elem_count*3);
				BasicPLY::FaceTex face;
				ply.setup_property(BasicPLY::face_tex_props[0]);
				ply.setup_property(BasicPLY::face_tex_props[1]);
				FOREACH(f, faces) {
					ply.get_element(&face);
					if (face.face.num != 3) {
						DEBUG_EXTRA("error: unsupported mesh file (face not triangle)");
						return false;
					}
					memcpy(faces.Begin()+f, face.face.pFace, sizeof(VIndex)*3);
					delete[] face.face.pFace;
					if (face.tex.num != 6) {
						DEBUG_EXTRA("error: unsupported mesh file (texture coordinates not per face vertex)");
						return false;
					}
					memcpy(faceTexcoords.Begin()+f*3, face.tex.pTex, sizeof(TexCoord)*3);
					delete[] face.tex.pTex;
				}
				// load the texture
				for (const std::string& comment: ply.get_comments()) {
					if (_tcsncmp(comment.c_str(), _T("TextureFile "), 12) == 0) {
						const String textureFileName(comment.substr(12));
						textureDiffuse.Load(Util::getFilePath(fileName)+textureFileName);
						break;
					}
				}
			}
		} else {
			ply.get_other_element();
		}
	}
	return true;
}
// import the mesh as a OBJ file
bool Mesh::LoadOBJ(const String& fileName)
{
	ASSERT(!fileName.IsEmpty());
	Release();

	// open and parse OBJ file
	ObjModel model;
	if (!model.Load(fileName)) {
		DEBUG_EXTRA("error: invalid OBJ file");
		return false;
	}
	if (model.get_vertices().empty() || model.get_groups().size() != 1 || model.get_groups()[0].faces.empty()) {
		DEBUG_EXTRA("error: invalid mesh file");
		return false;
	}

	// store vertices
	ASSERT(sizeof(ObjModel::Vertex) == sizeof(Vertex));
	ASSERT(model.get_vertices().size() < std::numeric_limits<VIndex>::max());
	vertices.CopyOf(&model.get_vertices()[0], (VIndex)model.get_vertices().size());

	// store vertex normals
	ASSERT(sizeof(ObjModel::Normal) == sizeof(Normal));
	ASSERT(model.get_vertices().size() < std::numeric_limits<VIndex>::max());
	if (!model.get_normals().empty()) {
		ASSERT(model.get_normals().size() == model.get_vertices().size());
		vertexNormals.CopyOf(&model.get_normals()[0], (VIndex)model.get_normals().size());
	}

	// store faces
	const ObjModel::Group& group = model.get_groups()[0];
	ASSERT(group.faces.size() < std::numeric_limits<FIndex>::max());
	faces.Reserve((FIndex)group.faces.size());
	for (const ObjModel::Face& f: group.faces) {
		ASSERT(f.vertices[0] != NO_ID);
		faces.AddConstruct(f.vertices[0], f.vertices[1], f.vertices[2]);
		if (f.texcoords[0] != NO_ID) {
			for (int i=0; i<3; ++i)
				faceTexcoords.AddConstruct(model.get_texcoords()[f.texcoords[i]]);
		}
		if (f.normals[0] != NO_ID) {
			Normal& n = faceNormals.AddConstruct(Normal::ZERO);
			for (int i=0; i<3; ++i)
				n += normalized(model.get_normals()[f.normals[i]]);
			normalize(n);
		}
	}

	// store texture
	ObjModel::MaterialLib::Material* pMaterial(model.GetMaterial(group.material_name));
	if (pMaterial && pMaterial->LoadDiffuseMap())
		cv::swap(textureDiffuse, pMaterial->diffuse_map);
	return true;
} // Load
/*----------------------------------------------------------------*/

// export the mesh to the given file
bool Mesh::Save(const String& fileName, const cList<String>& comments, bool bBinary) const
{
	TD_TIMER_STARTD();
	const String ext(Util::getFileExt(fileName).ToLower());
	bool ret;
	if (ext == _T(".obj"))
		ret = SaveOBJ(fileName);
	else
		ret = SavePLY(ext != _T(".ply") ? String(fileName+_T(".ply")) : fileName, comments, bBinary);
	if (!ret)
		return false;
	DEBUG_EXTRA("Mesh saved: %u vertices, %u faces (%s)", vertices.GetSize(), faces.GetSize(), TD_TIMER_GET_FMT().c_str());
	return true;
}
// export the mesh as a PLY file
bool Mesh::SavePLY(const String& fileName, const cList<String>& comments, bool bBinary) const
{
	ASSERT(!fileName.IsEmpty());
	Util::ensureFolder(fileName);

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
		textureFileName = Util::getFileFullName(fileName)+_T(".png");
		ply.append_comment((_T("TextureFile ")+Util::getFileNameExt(textureFileName)).c_str());
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
}
// export the mesh as a OBJ file
bool Mesh::SaveOBJ(const String& fileName) const
{
	ASSERT(!fileName.IsEmpty());
	Util::ensureFolder(fileName);

	// create the OBJ model
	ObjModel model;

	// store vertices
	ASSERT(sizeof(ObjModel::Vertex) == sizeof(Vertex));
	model.get_vertices().insert(model.get_vertices().begin(), vertices.Begin(), vertices.End());

	// store vertex normals
	ASSERT(sizeof(ObjModel::Normal) == sizeof(Normal));
	ASSERT(model.get_vertices().size() < std::numeric_limits<VIndex>::max());
	if (!vertexNormals.IsEmpty()) {
		ASSERT(vertexNormals.GetSize() == vertices.GetSize());
		model.get_normals().insert(model.get_normals().begin(), vertexNormals.Begin(), vertexNormals.End());
	}

	// store face texture coordinates
	ASSERT(sizeof(ObjModel::TexCoord) == sizeof(TexCoord));
	if (!faceTexcoords.IsEmpty()) {
		ASSERT(faceTexcoords.GetSize() == faces.GetSize()*3);
		model.get_texcoords().insert(model.get_texcoords().begin(), faceTexcoords.Begin(), faceTexcoords.End());
	}

	// store faces
	ObjModel::Group& group = model.AddGroup(_T("material_0"));
	group.faces.reserve(faces.GetSize());
	FOREACH(idxFace, faces) {
		const Face& face = faces[idxFace];
		ObjModel::Face f;
		memset(&f, 0xFF, sizeof(ObjModel::Face));
		for (int i=0; i<3; ++i) {
			f.vertices[i] = face[i];
			if (!faceTexcoords.IsEmpty())
				f.texcoords[i] = idxFace*3+i;
			if (!vertexNormals.IsEmpty())
				f.normals[i] = face[i];
		}
		group.faces.push_back(f);
	}

	// store texture
	ObjModel::MaterialLib::Material* pMaterial(model.GetMaterial(group.material_name));
	ASSERT(pMaterial != NULL);
	pMaterial->diffuse_map = textureDiffuse;

	return model.Save(fileName);
} // Save
/*----------------------------------------------------------------*/

bool Mesh::Save(const VertexArr& vertices, const String& fileName, bool bBinary)
{
	ASSERT(!fileName.IsEmpty());
	Util::ensureFolder(fileName);

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

#include <CGAL/Inverse_index.h>

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
	if (v0v1->is_border_edge())
		return false;
	Vertex::Halfedge_handle v1v0 = v0v1->opposite();
	if (v0v1->next()->opposite()->facet() == v1v0->prev()->opposite()->facet())
		return false;
	Vertex::Vertex_handle v0 = v0v1->vertex();
	if (v0->isBorder())
		return false;
	Vertex::Vertex_handle v1 = v1v0->vertex();
	if (v1->isBorder())
		return false;

	Vertex::Vertex_handle vl, vr;
	Vertex::Halfedge_handle h1, h2;
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
	if (vl == vr)
		return false;

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
		Vertex::Vertex_handle vTmp =c->opposite()->vertex();
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
		for (Vertex_iterator vi=p.vertices_begin(); vi!=p.vertices_end(); ++vi) {
			Vertex::Vertex_handle old_vi = vi;
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
				//bool collapsed(false);
				CGAL_For_all(c, d) {
					if (CanCollapseEdge(c->opposite())) {
						//if ((c->opposite()->vertex()==vi) && (vi!=p.vertices_end())) vi++;
						CollapseEdge(p, c->opposite());
						//collapsed = true;
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
			for (Edge_iterator ei=p.edges_begin(); ei!=p.edges_end(); ++ei) {
				if (ei->is_border_edge())
					continue;
				int d1_1 = (int)ei->vertex()->vertex_degree();
				int d1_2 = (int)ei->opposite()->vertex()->vertex_degree();
				int d2_1 = (int)ei->next()->vertex()->vertex_degree();
				int d2_2 = (int)ei->opposite()->next()->vertex()->vertex_degree();
				Vertex::Halfedge_handle h = ei;
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
			for (Edge_iterator ei=p.edges_begin(); ei!=p.edges_end(); ++ei) {
				if (ei->is_border_edge())
					continue;
				Point p1=ei->vertex()->point();
				Point p2=ei->next()->vertex()->point();
				Point p3=ei->prev()->vertex()->point();
				Point p4=ei->opposite()->next()->vertex()->point();

				float cost1((float)MINF(MINF(MINF(MINF(MINF(p_angle(p3, p1, p2), p_angle(p1, p3, p2)), p_angle(p4, p1, p3)), p_angle(p4, p3, p1)), p_angle(p1, p4, p2)), p_angle(p1, p2, p3)));
				float cost2((float)MINF(MINF(MINF(MINF(MINF(p_angle(p1, p2, p4), p_angle(p1, p4, p2)), p_angle(p3, p4, p2)), p_angle(p4, p2, p3)), p_angle(p4, p1, p2)), p_angle(p4, p3, p2)));

				Vertex::Halfedge_handle h = ei;
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

static void Smooth(Polyhedron& p, double delta, int mode=0)
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
static void EnsureEdgeSize(Polyhedron& p, double epsilonMin, double epsilonMax, double collapseRatio, double degenerate_angle_deg, int mode, int max_iters, int comp_size_threshold)
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

	if (comp_size_threshold > 0)
		RemoveConnectedComponents(p, comp_size_threshold, (float)edge.min*2);

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
		#if TD_VERBOSE != TD_VERBOSE_OFF
		String msgFaces;
		#endif
		FOREACH(i, faces) {
			const Mesh::Face& f = faces[i];
			if (!B.test_facet(f.ptr(), f.ptr()+3)) {
				bProblems = true;
				#if TD_VERBOSE != TD_VERBOSE_OFF
				if (VERBOSITY_LEVEL > 1)
					msgFaces += String::FormatString(" %u", i);
				#endif
				continue;
			}
			B.add_facet(f.ptr(), f.ptr()+3);
		}
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (bProblems)
			DEBUG_EXTRA("warning: ignoring the following facet(s) violating the manifold constraint:%s", msgFaces.c_str());
		#endif
		if (B.check_unconnected_vertices()) {
			DEBUG_EXTRA("warning: remove unconnected vertices");
			B.remove_unconnected_vertices();
		}
		B.end_surface();
	}
};
typedef TMeshBuilder<Polyhedron::HalfedgeDS, Kernel> MeshBuilder;
static bool ImportMesh(Polyhedron& p, const Mesh::VertexArr& vertices, const Mesh::FaceArr& faces) {
	MeshBuilder builder(vertices, faces);
	p.delegate(builder);
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
		v.x = (float)CGAL::to_double(it->point().x());
		v.y = (float)CGAL::to_double(it->point().y());
		v.z = (float)CGAL::to_double(it->point().z());
	}
	// extract the faces
	nCount = 0;
	faces.Resize((Mesh::FIndex)p.size_of_facets());
	CGAL::Inverse_index<Polyhedron::Vertex_const_iterator> index(p.vertices_begin(), p.vertices_end());
	for (Polyhedron::Face_const_iterator it=p.facets_begin(), ite=p.facets_end(); it!=ite; ++it) {
		ASSERT(it->is_triangle());
		Polyhedron::Halfedge_around_facet_const_circulator hc = it->facet_begin();
		ASSERT(CGAL::circulator_size(hc) == 3);
		Mesh::Face& facet = faces[nCount++];
		#if 0
		Polyhedron::Halfedge_around_facet_const_circulator hc_end = hc;
		unsigned i(0);
		do {
			facet[i++] = (Mesh::FIndex)index[Polyhedron::Vertex_const_iterator(hc->vertex())];
		} while (++hc != hc_end);
		#else
		for (int i=0; i<3; ++i, ++hc)
			facet[i] = (Mesh::FIndex)index[Polyhedron::Vertex_const_iterator(hc->vertex())];
		#endif
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
	CLN::EnsureEdgeSize(p, epsilonMin, epsilonMax, collapseRatio, degenerate_angle_deg, mode, max_iters, 0);
	CLN::ExportMesh(p, vertices, faces);
}
/*----------------------------------------------------------------*/

// subdivide mesh faces if its projection area
// is bigger than the given number of pixels
void Mesh::Subdivide(const AreaArr& maxAreas, uint32_t maxArea)
{
	ASSERT(vertexFaces.GetSize() == vertices.GetSize());

	// each face that needs to split, remember for each edge the new vertex index
	// (each new vertex index corresponds to the edge opposed to the existing vertex index)
	struct SplitFace {
		VIndex idxVert[3];
		bool bSplit;
		enum {NO_VERT = (VIndex)-1};
		inline SplitFace() : bSplit(false) { memset(idxVert, 0xFF, sizeof(VIndex)*3); }
		static VIndex FindSharedEdge(const Face& f, const Face& a) {
			for (int i=0; i<2; ++i) {
				const VIndex v(f[i]);
				if (v != a[0] && v != a[1] && v != a[2])
					return i;
			}
			ASSERT(f[2] != a[0] && f[2] != a[1] && f[2] != a[2]);
			return 2;
		}
	};
	typedef std::unordered_map<FIndex,SplitFace> FacetSplitMap;

	// used to find adjacent face
	typedef Mesh::FacetCountMap FacetCountMap;

	// for each image, compute the projection area of visible faces
	FacetSplitMap mapSplits; mapSplits.reserve(faces.GetSize());
	FacetCountMap mapFaces; mapFaces.reserve(12*3);
	vertices.Reserve(vertices.GetSize()*2);
	faces.Reserve(faces.GetSize()*3);
	FOREACH(f, maxAreas) {
		const AreaArr::Type area(maxAreas[f]);
		if (area <= maxArea)
			continue;
		// split face in four triangles
		// by adding a new vertex at the middle of each edge
		faces.ReserveExtra(4);
		Face& newface = faces.AddEmpty(); // defined by the three new vertices
		const Face& face = faces[(FIndex)f];
		SplitFace& split = mapSplits[(FIndex)f];
		for (int i=0; i<3; ++i) {
			// if the current edge was already split, used the existing vertex
			if (split.idxVert[i] != SplitFace::NO_VERT) {
				newface[i] = split.idxVert[i];
				continue;
			}
			// create a new vertex at the middle of the current edge
			// (current edge is the opposite edge to the current vertex index)
			split.idxVert[i] = newface[i] = vertices.GetSize();
			vertices.AddConstruct((vertices[face[(i+1)%3]]+vertices[face[(i+2)%3]])*0.5f);
		}
		// create the last three faces, defined by one old and two new vertices
		for (int i=0; i<3; ++i) {
			Face& nf = faces.AddEmpty();
			nf[0] = face[i];
			nf[1] = newface[(i+2)%3];
			nf[2] = newface[(i+1)%3];
		}
		split.bSplit = true;
		// find all three adjacent faces and inform them of the split
		ASSERT(mapFaces.empty());
		for (int i=0; i<3; ++i) {
			const Mesh::FaceIdxArr& vf = vertexFaces[face[i]];
			FOREACHPTR(pFace, vf)
				++mapFaces[*pFace].count;
		}
		for (const auto& fc: mapFaces) {
			ASSERT(fc.second.count <= 2 || (fc.second.count == 3 && fc.first == f));
			if (fc.second.count != 2)
				continue;
			if (fc.first < f && maxAreas[fc.first] > maxArea) {
				// already fully split, nothing to do
				ASSERT(mapSplits[fc.first].idxVert[SplitFace::FindSharedEdge(faces[fc.first], face)] == newface[SplitFace::FindSharedEdge(face, faces[fc.first])]);
				continue;
			}
			const VIndex idxVertex(newface[SplitFace::FindSharedEdge(face, faces[fc.first])]);
			VIndex& idxSplit = mapSplits[fc.first].idxVert[SplitFace::FindSharedEdge(faces[fc.first], face)];
			ASSERT(idxSplit == SplitFace::NO_VERT || idxSplit == idxVertex);
			idxSplit = idxVertex;
		}
		mapFaces.clear();
	}

	// add all faces partially split
	int indices[3];
	for (const auto& s: mapSplits) {
		const SplitFace& split = s.second;
		if (split.bSplit)
			continue;
		int count(0);
		for (int i=0; i<3; ++i) {
			if (split.idxVert[i] != SplitFace::NO_VERT)
				indices[count++] = i;
		}
		ASSERT(count > 0);
		faces.ReserveExtra(4);
		const Face& face = faces[s.first];
		switch (count) {
		case 1: {
			// one edge is split; create two triangles
			const int i(indices[0]);
			Face& nf0 = faces.AddEmpty();
			nf0[0] = split.idxVert[i];
			nf0[1] = face[(i+2)%3];
			nf0[2] = face[i];
			Face& nf1 = faces.AddEmpty();
			nf1[0] = split.idxVert[i];
			nf1[1] = face[i];
			nf1[2] = face[(i+1)%3];
			break; }
		case 2: {
			// two edges are split; create three triangles
			const int i0(indices[0]);
			const int i1(indices[1]);
			Face& nf0 = faces.AddEmpty();
			Face& nf1 = faces.AddEmpty();
			Face& nf2 = faces.AddEmpty();
			if (i0==0) {
				if (i1==1) {
					nf0[0] = split.idxVert[1];
					nf0[1] = split.idxVert[0];
					nf0[2] = face[2];
					nf1[0] = face[0];
					nf1[1] = face[1];
					nf1[2] = split.idxVert[0];
					nf2[0] = face[0];
					nf2[1] = split.idxVert[0];
					nf2[2] = split.idxVert[1];
				} else {
					nf0[0] = split.idxVert[2];
					nf0[1] = face[1];
					nf0[2] = split.idxVert[0];
					nf1[0] = face[0];
					nf1[1] = split.idxVert[2];
					nf1[2] = face[2];
					nf2[0] = split.idxVert[2];
					nf2[1] = split.idxVert[0];
					nf2[2] = face[2];
				}
			} else {
				ASSERT(i0==1 && i1==2);
				nf0[0] = face[0];
				nf0[1] = split.idxVert[2];
				nf0[2] = split.idxVert[1];
				nf1[0] = split.idxVert[1];
				nf1[1] = face[1];
				nf1[2] = face[2];
				nf2[0] = split.idxVert[2];
				nf2[1] = face[1];
				nf2[2] = split.idxVert[1];
			}
			break; }
		case 3: {
			// all three edges are split; create four triangles
			// create the new triangle in the middle
			Face& newface = faces.AddEmpty();
			newface[0] = split.idxVert[0];
			newface[1] = split.idxVert[1];
			newface[2] = split.idxVert[2];
			// create the last three faces, defined by one old and two new vertices
			for (int i=0; i<3; ++i) {
				Face& nf = faces.AddEmpty();
				nf[0] = face[i];
				nf[1] = newface[(i+2)%3];
				nf[2] = newface[(i+1)%3];
			}
			break; }
		}
	}

	// remove all faces that split
	ASSERT(faces.GetSize()-(faces.GetCapacity()/3)/*initial size*/ > mapSplits.size());
	for (const auto& s: mapSplits)
		faces.RemoveAt(s.first);
}
/*----------------------------------------------------------------*/

// decimate mesh by removing the given list of vertices
//#define DECIMATE_JOINHOLES // not finished
void Mesh::Decimate(VertexIdxArr& verticesRemove)
{
	ASSERT(vertices.GetSize() == vertexFaces.GetSize());
	FaceIdxArr facesRemove(0, verticesRemove.GetSize()*8);
	#ifdef DECIMATE_JOINHOLES
	cList<VertexIdxArr> holes;
	#endif
	FOREACHPTR(pIdxV, verticesRemove) {
		const VIndex idxV(*pIdxV);
		ASSERT(idxV < vertices.GetSize());
		// create the list of consecutive vertices around selected vertex
		VertexIdxArr verts;
		{
			FaceIdxArr& vf(vertexFaces[idxV]);
			if (vf.IsEmpty())
				continue;
			const FIndex n(vf.GetSize());
			facesRemove.Join(vf);
			ASSERT(verts.IsEmpty());
			{
				// add vertices of the first face
				const Face& f = faces[vf.First()];
				const uint32_t i(FindVertex(f, idxV));
				verts.Insert(f[(i+1)%3]);
				verts.Insert(f[(i+2)%3]);
				vf.RemoveAt(0);
			}
			while (verts.GetSize() < n) {
				// find the face that contains our vertex and the last added vertex
				const VIndex idxVL(verts.Last());
				FOREACH(idxF, vf) {
					const Face& f = faces[vf[idxF]];
					ASSERT(FindVertex(f, idxV) != NO_ID);
					const uint32_t i(FindVertex(f, idxVL));
					if (i == NO_ID)
						continue;
					// add the missing vertex at the end
					ASSERT(f[(i+2)%3] == idxV);
					const FIndex idxVN(f[(i+1)%3]);
					ASSERT(verts.First() != idxVN);
					verts.Insert(idxVN);
					vf.RemoveAt(idxF);
					goto NEXT_FACE_FORWARD;
				}
			#ifndef DECIMATE_JOINHOLES
				vf.Release();
				goto NEXT_VERTEX;
				NEXT_FACE_FORWARD:;
			}
			vf.Release();
			#else
				break;
				NEXT_FACE_FORWARD:;
			}
			while (!vf.IsEmpty()) {
				// find the face that contains our vertex and the first added vertex
				const VIndex idxVF(verts.First());
				FOREACH(idxF, vf) {
					const Face& f = faces[vf[idxF]];
					ASSERT(FindVertex(f, idxV) != NO_ID);
					const uint32_t i(FindVertex(f, idxVF));
					if (i == NO_ID)
						continue;
					// add the missing vertex at the beginning
					ASSERT(f[(i+1)%3] == idxV);
					const FIndex idxVP(f[(i+2)%3]);
					ASSERT(verts.Last() != idxVP || vf.GetSize() == 1);
					if (verts.Last() != idxVP)
						verts.InsertAt(0, idxVP);
					vf.RemoveAt(idxF);
					goto NEXT_FACE_BACKWARD;
				}
				vf.Release();
				goto NEXT_VERTEX;
				NEXT_FACE_BACKWARD:;
			}
			#endif
		}
		// remove the deleted faces from each vertex face list
		FOREACHPTR(pV, verts) {
			FaceIdxArr& vf(vertexFaces[*pV]);
			RFOREACH(i, vf) {
				const Face& f = faces[vf[i]];
				if (FindVertex(f, idxV) != NO_ID)
					vf.RemoveAt(i);
			}
		}
		#ifdef DECIMATE_JOINHOLES
		// find the hole that contains the vertex to be deleted
		FOREACHPTR(pHole, holes) {
			const VIndex idxVH(pHole->Find(idxV));
			if (idxVH == VertexIdxArr::NO_INDEX)
				continue;
			// extend the hole with the new loop vertices
			VertexIdxArr& hole(*pHole);
			hole.RemoveAtMove(idxVH);
			const VIndex idxS((idxVH+hole.GetSize()-1)%hole.GetSize());
			const VIndex idxL(verts.Find(hole[idxS]));
			ASSERT(idxL != VertexIdxArr::NO_INDEX);
			ASSERT(verts[(idxL+verts.GetSize()-1)%verts.GetSize()] == hole[(idxS+1)%hole.GetSize()]);
			const VIndex n(verts.GetSize()-2);
			for (VIndex v=1; v<=n; ++v)
				hole.InsertAt(idxS+v, verts[(idxL+v)%verts.GetSize()]);
			goto NEXT_VERTEX;
		}
		// or create a new hole
		if (verts.GetSize() < 3)
			continue;
		verts.Swap(holes.AddEmpty());
		#else
		// close the holes defined by the complete loop of consecutive vertices
		// (the loop can be opened, cause some of the vertices can be on the border)
		if (verts.GetSize() > 2)
			CloseHoleQuality(verts);
		#endif
		NEXT_VERTEX:;
	}
	#ifndef _RELEASE
	// check all removed vertices are completely disconnected from the mesh
	FOREACHPTR(pIdxV, verticesRemove)
		ASSERT(vertexFaces[*pIdxV].IsEmpty());
	#endif

	// remove deleted faces
	RemoveFaces(facesRemove, true);

	// remove deleted vertices
	RemoveVertices(verticesRemove);

	#ifdef DECIMATE_JOINHOLES
	// close the holes defined by the complete loop of consecutive vertices
	// (the loop can be opened, cause some of the vertices can be on the border)
	FOREACHPTR(pHole, holes) {
		ASSERT(pHole->GetSize() > 2);
		CloseHoleQuality(*pHole);
	}
	#endif

	#ifndef _RELEASE
	// check all faces see valid vertices
	FOREACH(idxF, faces) {
		const Face& face = faces[idxF];
		for (int v=0; v<3; ++v)
			ASSERT(face[v] < vertices.GetSize());
	}
	#endif
}
/*----------------------------------------------------------------*/

// given a hole defined by a complete loop of consecutive vertices,
// split it recursively in two halves till the splits becomes a face
void Mesh::CloseHole(VertexIdxArr& split0)
{
	ASSERT(split0.GetSize() >= 3);
	if (split0.GetSize() == 3) {
		const FIndex idxF(faces.GetSize());
		faces.AddConstruct(split0[0], split0[1], split0[2]);
		for (int v=0; v<3; ++v) {
			#ifndef _RELEASE
			FaceIdxArr indices;
			GetAdjVertexFaces(split0[v], split0[(v+1)%3], indices);
			ASSERT(indices.GetSize() < 2);
			indices.Empty();
			GetAdjVertexFaces(split0[v], split0[(v+2)%3], indices);
			ASSERT(indices.GetSize() < 2);
			#endif
			vertexFaces[split0[v]].Insert(idxF);
		}
		return;
	}
	const VIndex i(split0.GetSize() >> 1);
	const VIndex j(split0.GetSize()-i);
	VertexIdxArr split1(0, j+1);
	split1.Join(split0.Begin()+i, j);
	split1.Insert(split0.First());
	split0.RemoveLast(j-1);
	CloseHole(split0);
	CloseHole(split1);
}

// given a hole defined by a complete loop of consecutive vertices,
// fills it using an heap to choose the best candidate face to be added
void Mesh::CloseHoleQuality(VertexIdxArr& verts)
{
	struct CandidateFace
	{
		Face face;
		float angle;
		float dihedral;
		float aspectRatio;

		CandidateFace() {}
		// the vertices of the given face must be in the order they appear on the border of the hole
		// (the middle face vertex must be between the first and third on the border)
		CandidateFace(VIndex v0, VIndex v1, VIndex v2, const Mesh& mesh) : face(v0,v1,v2) {
			const Normal n(mesh.FaceNormal(face));
			// compute the angle between the two existing edges of the face
			// (the angle computation takes into account the case of reversed face)
			angle = ACOS(ComputeAngle<float,float>(mesh.vertices[face[1]].ptr(), mesh.vertices[face[0]].ptr(), mesh.vertices[face[2]].ptr()));
			if (n.dot(mesh.VertexNormal(face[1])) < 0)
				angle = float(2*M_PI) - angle;
			// compute quality as a composition of dihedral angle and area/sum(edge^2);
			// the dihedral angle uses the normal of the edge faces
			// which are possible not to exist if the edges are on the border
			FaceIdxArr indices;
			mesh.GetAdjVertexFaces(face[2], face[0], indices);
			if (indices.GetSize() > 1) {
				aspectRatio = -1;
				return;
			}
			indices.Empty();
			mesh.GetAdjVertexFaces(face[0], face[1], indices);
			if (indices.GetSize() > 1) {
				aspectRatio = -1;
				return;
			}
			const FIndex i0(indices.GetSize());
			mesh.GetAdjVertexFaces(face[1], face[2], indices);
			if (indices.GetSize()-i0 > 1) {
				aspectRatio = -1;
				return;
			}
			if (indices.IsEmpty())
				dihedral = FD2R(33.f);
			else {
				const Normal n0(mesh.FaceNormal(mesh.faces[indices[0]]));
				if (indices.GetSize() == 1)
					dihedral = ACOS(ComputeAngle<float,float>(n.ptr(), n0.ptr()));
				else {
					const Normal n1(mesh.FaceNormal(mesh.faces[indices[1]]));
					dihedral = MAXF(ACOS(ComputeAngle<float,float>(n.ptr(), n0.ptr())), ACOS(ComputeAngle<float,float>(n.ptr(), n1.ptr())));
				}
			}
			aspectRatio = ComputeTriangleQuality(mesh.vertices[face[0]], mesh.vertices[face[1]], mesh.vertices[face[2]]);
		}

		inline operator const Face&() const { return face; }
		inline bool IsConcave() const { return angle > (float)M_PI; }
		inline float GetQuality() const { return aspectRatio - 0.3f/*diedral weight*/*(dihedral/(float)M_PI); }

		// In the heap, by default, we retrieve the LARGEST value,
		// so if we need the ear with minimal dihedral angle, we must reverse the sign of the comparison.
		// The concave elements must be all in the end of the heap, sorted accordingly,
		// So if only one of the two ear is Concave that one is always the minimum one.
		inline bool operator < (const CandidateFace& c) const {
			if ( IsConcave() && !c.IsConcave()) return true;
			if (!IsConcave() &&  c.IsConcave()) return false;
			return GetQuality() < c.GetQuality();
		}
	};

	// create the initial list of new possible face along the edge of the hole
	ASSERT(verts.GetSize() > 2);
	cList<CandidateFace> candidateFaces(0, verts.GetSize());
	FOREACH(v, verts) {
		if (candidateFaces.AddConstruct(verts[v], verts[(v+1)%verts.GetSize()], verts[(v+2)%verts.GetSize()], *this).aspectRatio < 0)
			candidateFaces.RemoveLast();
	}
	candidateFaces.Sort();

	// add new faces until there are only two vertices left
	while(true) {
		// add the best candidate face
		ASSERT(!candidateFaces.IsEmpty());
		const Face& candidateFace = candidateFaces.Last();
		ASSERT(verts.Find(candidateFace[0]) != VertexIdxArr::NO_INDEX);
		ASSERT(verts.Find(candidateFace[1]) != VertexIdxArr::NO_INDEX);
		ASSERT(verts.Find(candidateFace[2]) != VertexIdxArr::NO_INDEX);
		const FIndex idxF(faces.GetSize());
		faces.Insert(candidateFace);
		for (int v=0; v<3; ++v) {
			#ifndef _RELEASE
			FaceIdxArr indices;
			GetAdjVertexFaces(candidateFace[v], candidateFace[(v+1)%3], indices);
			ASSERT(indices.GetSize() < 2);
			indices.Empty();
			GetAdjVertexFaces(candidateFace[v], candidateFace[(v+2)%3], indices);
			ASSERT(indices.GetSize() < 2);
			#endif
			vertexFaces[candidateFace[v]].Insert(idxF);
		}
		if (verts.GetSize() <= 3)
			break;
		const VIndex idxV(verts.Find(candidateFace[1]));
		// remove all candidate face containing this vertex
		{
		candidateFaces.RemoveLast();
		const VIndex idxVert(verts[idxV]);
		int n(0);
		RFOREACH(c, candidateFaces)
			if (FindVertex(candidateFaces[c].face, idxVert) != NO_ID) {
				candidateFaces.RemoveAtMove(c);
				if (++n == 2)
					break;
			}
		}
		// insert the two new candidate faces
		const VIndex idxB(idxV+verts.GetSize());
		const VIndex idxVB2(verts[(idxB-2)%verts.GetSize()]);
		const VIndex idxVB1(verts[(idxB-1)%verts.GetSize()]);
		const VIndex idxVF1(verts[(idxV+1)%verts.GetSize()]);
		const VIndex idxVF2(verts[(idxV+2)%verts.GetSize()]);
		{
			const CandidateFace newCandidateFace(idxVB2, idxVB1, idxVF1, *this);
			if (newCandidateFace.aspectRatio >= 0)
				candidateFaces.InsertSort(newCandidateFace);
		}
		{
			const CandidateFace newCandidateFace(idxVB1, idxVF1, idxVF2, *this);
			if (newCandidateFace.aspectRatio >= 0)
				candidateFaces.InsertSort(newCandidateFace);
		}
		verts.RemoveAtMove(idxV);
	}
}
/*----------------------------------------------------------------*/

// remove the given list of faces
void Mesh::RemoveFaces(FaceIdxArr& facesRemove, bool bUpdateLists)
{
	facesRemove.Sort();
	FIndex idxLast(FaceIdxArr::NO_INDEX);
	if (!bUpdateLists || vertexFaces.IsEmpty()) {
		RFOREACHPTR(pIdxF, facesRemove) {
			const FIndex idxF(*pIdxF);
			if (idxLast == idxF)
				continue;
			faces.RemoveAt(idxF);
			idxLast = idxF;
		}
	} else {
		ASSERT(vertices.GetSize() == vertexFaces.GetSize());
		RFOREACHPTR(pIdxF, facesRemove) {
			const FIndex idxF(*pIdxF);
			if (idxLast == idxF)
				continue;
			{
				// remove face from vertex face list
				const Face& face = faces[idxF];
				for (int v=0; v<3; ++v) {
					const VIndex idxV(face[v]);
					FaceIdxArr& vf(vertexFaces[idxV]);
					const FIndex idx(vf.Find(idxF));
					if (idx != FaceIdxArr::NO_INDEX)
						vf[idx] = idxF;
				}
			}
			const FIndex idxFM(faces.GetSize()-1);
			if (idxF < idxFM) {
				// update all vertices of the moved face
				const Face& face = faces[idxFM];
				for (int v=0; v<3; ++v) {
					const VIndex idxV(face[v]);
					FaceIdxArr& vf(vertexFaces[idxV]);
					const FIndex idx(vf.Find(idxFM));
					if (idx != FaceIdxArr::NO_INDEX)
						vf[idx] = idxF;
				}
			}
			faces.RemoveAt(idxF);
			idxLast = idxF;
		}
	}
	vertexVertices.Release();
}
/*----------------------------------------------------------------*/

// remove the given list of vertices
void Mesh::RemoveVertices(VertexIdxArr& vertexRemove, bool bUpdateLists)
{
	ASSERT(vertices.GetSize() == vertexFaces.GetSize());
	vertexRemove.Sort();
	VIndex idxLast(VertexIdxArr::NO_INDEX);
	if (!bUpdateLists) {
		RFOREACHPTR(pIdxV, vertexRemove) {
			const VIndex idxV(*pIdxV);
			if (idxLast == idxV)
				continue;
			const VIndex idxVM(vertices.GetSize()-1);
			if (idxV < idxVM) {
				// update all faces of the moved vertex
				const FaceIdxArr& vf(vertexFaces[idxVM]);
				FOREACHPTR(pIdxF, vf)
					GetVertex(faces[*pIdxF], idxVM) = idxV;
			}
			vertexFaces.RemoveAt(idxV);
			vertices.RemoveAt(idxV);
			idxLast = idxV;
		}
		return;
	}
	FaceIdxArr facesRemove;
	RFOREACHPTR(pIdxV, vertexRemove) {
		const VIndex idxV(*pIdxV);
		if (idxLast == idxV)
			continue;
		const VIndex idxVM(vertices.GetSize()-1);
		if (idxV < idxVM) {
			// update all faces of the moved vertex
			const FaceIdxArr& vf(vertexFaces[idxVM]);
			FOREACHPTR(pIdxF, vf)
				GetVertex(faces[*pIdxF], idxVM) = idxV;
		}
		if (!vertexFaces.IsEmpty()) {
			facesRemove.Join(vertexFaces[idxV]);
			vertexFaces.RemoveAt(idxV);
		}
		if (!vertexVertices.IsEmpty())
			vertexVertices.RemoveAt(idxV);
		vertices.RemoveAt(idxV);
		idxLast = idxV;
	}
	RemoveFaces(facesRemove);
}
/*----------------------------------------------------------------*/


// computes the area of the mesh surface as the sum of the signed areas of its faces
REAL Mesh::ComputeArea() const
{
	REAL area(0);
	for (const Face& face: faces)
		area += ComputeTriangleArea(vertices[face[0]], vertices[face[1]], vertices[face[2]]);
	return area;
}

// computes the signed volume of the domain bounded by the mesh surface
// (note: valid only for closed and orientable manifolds)
REAL Mesh::ComputeVolume() const
{
	REAL volume(0);
	for (const Face& face: faces)
		volume += ComputeTriangleVolume(vertices[face[0]], vertices[face[1]], vertices[face[2]]);
	return volume;
}
/*----------------------------------------------------------------*/


// project mesh to the given camera plane
void Mesh::SamplePoints(unsigned numberOfPoints, PointCloud& pointcloud) const
{
	// total mesh surface
	const REAL area(ComputeArea());
	if (area < ZEROTOLERANCE<float>()) {
		pointcloud.Release();
		return;
	}
	const REAL samplingDensity(numberOfPoints / area);
	return SamplePoints(samplingDensity, numberOfPoints, pointcloud);
}
void Mesh::SamplePoints(REAL samplingDensity, PointCloud& pointcloud) const
{
	// compute the total area to deduce the number of points
	const REAL area(ComputeArea());
	const unsigned theoreticNumberOfPoints((unsigned)CEIL2INT(area * samplingDensity));
	return SamplePoints(samplingDensity, theoreticNumberOfPoints, pointcloud);
}
void Mesh::SamplePoints(REAL samplingDensity, unsigned mumPointsTheoretic, PointCloud& pointcloud) const
{
	ASSERT(!IsEmpty());
	pointcloud.Release();
	if (mumPointsTheoretic > 0) {
		pointcloud.points.reserve(mumPointsTheoretic);
		if (HasTexture())
			pointcloud.colors.reserve(mumPointsTheoretic);
	}

	// for each triangle
	std::mt19937 rnd((std::random_device())());
	std::uniform_real_distribution<REAL> dist(0,1);
	FOREACH(idxFace, faces) {
		const Face& face = faces[idxFace];

		// vertices (OAB)
		const Vertex& O = vertices[face[0]];
		const Vertex& A = vertices[face[1]];
		const Vertex& B = vertices[face[2]];

		// edges (OA and OB)
		const Vertex u(A - O);
		const Vertex v(B - O);

		// compute triangle area
		const REAL area(norm(u.cross(v)) * REAL(0.5));

		// deduce the number of points to generate on this face
		const REAL fPointsToAdd(area*samplingDensity);
		unsigned pointsToAdd(static_cast<unsigned>(fPointsToAdd));

		// take care of the remaining fractional part;
		// add a point with the same probability as its (relative) area
		const REAL fracPart(fPointsToAdd - static_cast<REAL>(pointsToAdd));
		if (dist(rnd) <= fracPart)
			pointsToAdd++;

		for (unsigned i = 0; i < pointsToAdd; ++i) {
			// generate random points as in:
			// "Generating random points in triangles", Greg Turk;
			// in A. S. Glassner, editor, Graphics Gems, pages 24-28. Academic Press, 1990
			REAL x(dist(rnd));
			REAL y(dist(rnd));

			// test if the generated point lies on the right side of (AB)
			if (x + y > REAL(1)) {
				x = REAL(1) - x;
				y = REAL(1) - y;
			}

			// compute position
			pointcloud.points.emplace_back(O + static_cast<Vertex::Type>(x)*u + static_cast<Vertex::Type>(y)*v);

			if (HasTexture()) {
				// compute color
				const FIndex idxTexCoord(idxFace*3);
				const TexCoord& TO = faceTexcoords[idxTexCoord+0];
				const TexCoord& TA = faceTexcoords[idxTexCoord+1];
				const TexCoord& TB = faceTexcoords[idxTexCoord+2];
				const TexCoord xt(TO + static_cast<TexCoord::Type>(x)*(TA - TO) + static_cast<TexCoord::Type>(y)*(TB - TO));
				pointcloud.colors.emplace_back(textureDiffuse.sampleSafe(Point2f(xt.x*textureDiffuse.width(), (1.f-xt.y)*textureDiffuse.height())));
			}
		}
	}
}
/*----------------------------------------------------------------*/


// project mesh to the given camera plane
void Mesh::Project(const Camera& camera, DepthMap& depthMap) const
{
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		RasterMesh(const VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap)
			: Base(_vertices, _camera, _depthMap) {}
	};
	RasterMesh rasterer(vertices, camera, depthMap);
	rasterer.Clear();
	for (const Face& facet: faces)
		rasterer.Project(facet);
}
void Mesh::Project(const Camera& camera, DepthMap& depthMap, Image8U3& image) const
{
	ASSERT(!faceTexcoords.IsEmpty() && !textureDiffuse.empty());
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		const Mesh& mesh;
		Image8U3& image;
		FIndex idxFaceTex;
		TexCoord xt;
		RasterMesh(const Mesh& _mesh, const Camera& _camera, DepthMap& _depthMap, Image8U3& _image)
			: Base(_mesh.vertices, _camera, _depthMap), mesh(_mesh), image(_image) {}
		inline void Clear() {
			Base::Clear();
			image.memset(0);
		}
		void Raster(const ImageRef& pt) {
			if (!depthMap.isInsideWithBorder<float,3>(pt))
				return;
			const float z((float)INVERT(normalPlane.dot(camera.TransformPointI2C(Point2(pt)))));
			ASSERT(z > 0);
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				const Point3f b(CorrectBarycentricCoordinates(BarycentricCoordinatesUV(pti[0], pti[1], pti[2], Point2f(pt))));
				xt  = mesh.faceTexcoords[idxFaceTex+0] * b[0];
				xt += mesh.faceTexcoords[idxFaceTex+1] * b[1];
				xt += mesh.faceTexcoords[idxFaceTex+2] * b[2];
				image(pt) = mesh.textureDiffuse.sampleSafe(Point2f(xt.x*mesh.textureDiffuse.width(), (1.f-xt.y)*mesh.textureDiffuse.height()));
			}
		}
	};
	if (image.size() != depthMap.size())
		image.create(depthMap.size());
	RasterMesh rasterer(*this, camera, depthMap, image);
	rasterer.Clear();
	FOREACH(idxFace, faces) {
		const Face& facet = faces[idxFace];
		rasterer.idxFaceTex = idxFace*3;
		rasterer.Project(facet);
	}
}
// project mesh to the given camera plane using orthographic projection
void Mesh::ProjectOrtho(const Camera& camera, DepthMap& depthMap) const
{
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		RasterMesh(const VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap)
			: Base(_vertices, _camera, _depthMap) {}
		inline bool ProjectVertex(const Mesh::Vertex& pt, int v) {
			ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt));
			pti[v] = camera.TransformPointC2I(reinterpret_cast<const Point2&>(ptc[v]));
			return depthMap.isInsideWithBorder<float,3>(pti[v]);
		}
		void Raster(const ImageRef& pt) {
			if (!depthMap.isInsideWithBorder<float,3>(pt))
				return;
			const Point3f b(CorrectBarycentricCoordinates(BarycentricCoordinatesUV(pti[0], pti[1], pti[2], Point2f(pt))));
			const float z((float)(ptc[0].z*b[0] + ptc[1].z*b[1] + ptc[2].z*b[2]));
			ASSERT(z > 0);
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z)
				depth = z;
		}
	};
	RasterMesh rasterer(vertices, camera, depthMap);
	rasterer.Clear();
	for (const Face& facet: faces)
		rasterer.Project(facet);
}
void Mesh::ProjectOrtho(const Camera& camera, DepthMap& depthMap, Image8U3& image) const
{
	ASSERT(!faceTexcoords.IsEmpty() && !textureDiffuse.empty());
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		const Mesh& mesh;
		Image8U3& image;
		FIndex idxFaceTex;
		TexCoord xt;
		RasterMesh(const Mesh& _mesh, const Camera& _camera, DepthMap& _depthMap, Image8U3& _image)
			: Base(_mesh.vertices, _camera, _depthMap), mesh(_mesh), image(_image) {}
		inline void Clear() {
			Base::Clear();
			image.memset(0);
		}
		inline bool ProjectVertex(const Mesh::Vertex& pt, int v) {
			ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt));
			pti[v] = camera.TransformPointC2I(reinterpret_cast<const Point2&>(ptc[v]));
			return depthMap.isInsideWithBorder<float,3>(pti[v]);
		}
		inline bool CheckNormal(const Point3& /*faceCenter*/) {
			// skip face if the (cos) angle between
			// the face normal and the view direction is negative
			return camera.Direction().dot(normalPlane) >= ZEROTOLERANCE<REAL>();
		}
		void Raster(const ImageRef& pt) {
			if (!depthMap.isInsideWithBorder<float,3>(pt))
				return;
			const Point3f b(CorrectBarycentricCoordinates(BarycentricCoordinatesUV(pti[0], pti[1], pti[2], Point2f(pt))));
			const float z((float)(ptc[0].z*b[0] + ptc[1].z*b[1] + ptc[2].z*b[2]));
			ASSERT(z > 0);
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				xt  = mesh.faceTexcoords[idxFaceTex+0] * b[0];
				xt += mesh.faceTexcoords[idxFaceTex+1] * b[1];
				xt += mesh.faceTexcoords[idxFaceTex+2] * b[2];
				image(pt) = mesh.textureDiffuse.sampleSafe(Point2f(xt.x*mesh.textureDiffuse.width(), (1.f-xt.y)*mesh.textureDiffuse.height()));
			}
		}
	};
	if (image.size() != depthMap.size())
		image.create(depthMap.size());
	RasterMesh rasterer(*this, camera, depthMap, image);
	rasterer.Clear();
	FOREACH(idxFace, faces) {
		const Face& facet = faces[idxFace];
		rasterer.idxFaceTex = idxFace*3;
		rasterer.Project(facet);
	}
}
// assuming the mesh is properly oriented, ortho-project it to a camera looking from top to down
void Mesh::ProjectOrthoTopDown(unsigned resolution, Image8U3& image, Image8U& mask, Point3& center) const
{
	ASSERT(!IsEmpty() && !textureDiffuse.empty());
	// initialize camera
	const AABB3f box(vertices.Begin(), vertices.GetSize());
	const Point3 size(Vertex(box.GetSize())*1.01f/*border*/);
	center = Vertex(box.GetCenter());
	Camera camera;
	camera.R.SetFromDirUp(Vec3(Point3(0,0,-1)), Vec3(Point3(0,1,0)));
	camera.C = center;
	camera.C.z += size.z;
	camera.K = KMatrix::IDENTITY;
	if (size.x > size.y) {
		image.create(CEIL2INT(size.y*(resolution-1)/size.x), (int)resolution);
		camera.K(0,0) = camera.K(1,1) = (resolution-1)/size.x;
	} else {
		image.create((int)resolution, CEIL2INT(size.x*(resolution-1)/size.y));
		camera.K(0,0) = camera.K(1,1) = (resolution-1)/size.y;
	}
	camera.K(0,2) = (REAL)(image.width()-1)/2;
	camera.K(1,2) = (REAL)(image.height()-1)/2;
	// project mesh
	DepthMap depthMap(image.size());
	ProjectOrtho(camera, depthMap, image);
	// create mask for the valid image pixels
	if (mask.size() != depthMap.size())
		mask.create(depthMap.size());
	for (int r=0; r<mask.rows; ++r)
		for (int c=0; c<mask.cols; ++c)
			mask(r,c) = depthMap(r,c) > 0 ? 255 : 0;
	// compute 3D coordinates for the image center
	const ImageRef xCenter(image.width()/2, image.height()/2);
	const Depth depthCenter(depthMap(xCenter));
	center = camera.TransformPointI2W(Point3(xCenter.x, xCenter.y, depthCenter > 0 ? depthCenter : camera.C.z-center.z));
}
/*----------------------------------------------------------------*/


#ifdef _USE_CUDA
CUDA::KernelRT Mesh::kernelComputeFaceNormal;

bool Mesh::InitKernels(int device)
{
	// initialize CUDA device if needed
	if (CUDA::devices.IsEmpty() && CUDA::initDevice(device) != CUDA_SUCCESS)
		return false;

	// initialize CUDA kernels
	if (!kernelComputeFaceNormal.IsValid()) {
		// kernel used to compute face normal, given the array of face vertices and vertex positions
		STATIC_ASSERT(sizeof(Vertex) == sizeof(float)*3);
		STATIC_ASSERT(sizeof(Face) == sizeof(VIndex)*3 && sizeof(VIndex) == sizeof(uint32_t));
		STATIC_ASSERT(sizeof(Normal) == sizeof(float)*3);
		#define FUNC "ComputeFaceNormal"
		LPCSTR const szKernel =
			".version 3.2\n"
			".target sm_20\n"
			".address_size 64\n"
			"\n"
			".visible .entry " FUNC "(\n"
			"	.param .u64 .ptr param_1, // array vertices (float*3 * numVertices)\n"
			"	.param .u64 .ptr param_2, // array faces (uint32_t*3 * numFaces)\n"
			"	.param .u64 .ptr param_3, // array normals (float*3 * numFaces) [out]\n"
			"	.param .u32 param_4 // numFaces = numNormals (uint32_t)\n"
			")\n"
			"{\n"
			"	.reg .f32 %f<32>;\n"
			"	.reg .pred %p<2>;\n"
			"	.reg .u32 %r<17>;\n"
			"	.reg .u64 %rl<18>;\n"
			"\n"
			"	ld.param.u64 %rl4, [param_1];\n"
			"	ld.param.u64 %rl5, [param_2];\n"
			"	ld.param.u64 %rl6, [param_3];\n"
			"	ld.param.u32 %r2,  [param_4];\n"
			"	cvta.to.global.u64 %rl1, %rl6;\n"
			"	cvta.to.global.u64 %rl2, %rl4;\n"
			"	cvta.to.global.u64 %rl3, %rl5;\n"
			"	mov.u32 %r3, %ntid.x;\n"
			"	mov.u32 %r4, %ctaid.x;\n"
			"	mov.u32 %r5, %tid.x;\n"
			"	mad.lo.u32 %r1, %r3, %r4, %r5;\n"
			"	setp.ge.u32 %p1, %r1, %r2;\n"
			"	@%p1 bra BB00_1;\n"
			"\n"
			"	mul.lo.u32 %r6, %r1, 3;\n"
			"	mul.wide.u32 %rl7, %r6, 4;\n"
			"	add.u64 %rl8, %rl3, %rl7;\n"
			"	ld.global.u32 %r8, [%rl8];\n"
			"	mul.lo.u32 %r10, %r8, 3;\n"
			"	mul.wide.u32 %rl11, %r10, 4;\n"
			"	add.u64 %rl12, %rl2, %rl11;\n"
			"	ld.global.u32 %r11, [%rl8+4];\n"
			"	mul.lo.u32 %r13, %r11, 3;\n"
			"	mul.wide.u32 %rl13, %r13, 4;\n"
			"	add.u64 %rl14, %rl2, %rl13;\n"
			"	ld.global.u32 %r14, [%rl8+8];\n"
			"	mul.lo.u32 %r16, %r14, 3;\n"
			"	mul.wide.u32 %rl15, %r16, 4;\n"
			"	add.u64 %rl16, %rl2, %rl15;\n"
			"\n"
			"	ld.global.f32 %f1, [%rl14];\n"
			"	ld.global.f32 %f2, [%rl12];\n"
			"	sub.f32 %f3, %f1, %f2;\n"
			"	ld.global.f32 %f4, [%rl14+4];\n"
			"	ld.global.f32 %f5, [%rl12+4];\n"
			"	sub.f32 %f6, %f4, %f5;\n"
			"	ld.global.f32 %f7, [%rl14+8];\n"
			"	ld.global.f32 %f8, [%rl12+8];\n"
			"	sub.f32 %f9, %f7, %f8;\n"
			"	ld.global.f32 %f10, [%rl16];\n"
			"	sub.f32 %f11, %f10, %f2;\n"
			"	ld.global.f32 %f12, [%rl16+4];\n"
			"	sub.f32 %f13, %f12, %f5;\n"
			"	ld.global.f32 %f14, [%rl16+8];\n"
			"	sub.f32 %f15, %f14, %f8;\n"
			"\n"
			"	mul.f32 %f16, %f6, %f15;\n"
			"	neg.f32 %f17, %f9;\n"
			"	fma.rn.f32 %f18, %f17, %f13, %f16;\n"
			"	mul.f32 %f19, %f9, %f11;\n"
			"	neg.f32 %f20, %f3;\n"
			"	fma.rn.f32 %f21, %f20, %f15, %f19;\n"
			"	mul.f32 %f22, %f3, %f13;\n"
			"	neg.f32 %f23, %f6;\n"
			"\n"
			"	fma.rn.f32 %f24, %f23, %f11, %f22;\n"
			"	mul.f32 %f25, %f21, %f21;\n"
			"	fma.rn.f32 %f26, %f18, %f18, %f25;\n"
			"	fma.rn.f32 %f27, %f24, %f24, %f26;\n"
			"\n"
			"	sqrt.rn.f32 %f28, %f27;\n"
			"	div.rn.f32 %f29, %f18, %f28;\n"
			"	div.rn.f32 %f30, %f21, %f28;\n"
			"	div.rn.f32 %f31, %f24, %f28;\n"
			"\n"
			"	add.u64 %rl17, %rl1, %rl7;\n"
			"	st.global.f32 [%rl17], %f29;\n"
			"	st.global.f32 [%rl17+4], %f30;\n"
			"	st.global.f32 [%rl17+8], %f31;\n"
			"\n"
			"	BB00_1:\n"
			"	ret;\n"
			"}\n";
		if (kernelComputeFaceNormal.Reset(szKernel, FUNC) != CUDA_SUCCESS)
			return false;
		ASSERT(kernelComputeFaceNormal.IsValid());
		#undef FUNC
	}

	return true;
}
/*----------------------------------------------------------------*/
#endif
