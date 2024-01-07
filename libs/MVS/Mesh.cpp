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
#pragma warning(push)
#pragma warning(disable: 4244 4267 4305)
#ifdef _SUPPORT_CPP17
namespace std {
template <typename ArgumentType, typename ResultType>
struct unary_function {
};
} // namespace std
#endif // _SUPPORT_CPP17
#endif // _MSC_VER
// VCG: mesh reconstruction post-processing
#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/stat.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/polygon_support.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
// VCG: mesh simplification
#include <vcg/complex/algorithms/update/position.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/selection.h>
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#undef Split
#ifdef _MSC_VER
#  pragma warning(pop)
#endif
// GLTF: mesh import/export
#define JSON_NOEXCEPTION
#define TINYGLTF_NOEXCEPTION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_INCLUDE_JSON
#define TINYGLTF_NO_INCLUDE_STB_IMAGE
#define TINYGLTF_NO_INCLUDE_STB_IMAGE_WRITE
#define TINYGLTF_IMPLEMENTATION
#include "../IO/json.hpp"
#include "../IO/tiny_gltf.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define MESH_USE_OPENMP
#endif

// select fast ray-face intersection search method
#define USE_MESH_BF 0 // brute-force
#define USE_MESH_OCTREE 1 // octree (misses some triangles)
#define USE_MESH_BVH 2 // BVH (misses some triangles)
#define USE_MESH_INT USE_MESH_BVH

#if USE_MESH_INT == USE_MESH_BVH
#include <unsupported/Eigen/BVH>
#endif


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
	faceFaces.Release();
	faceTexcoords.Release();
	texturesDiffuse.Release();
} // ReleaseExtra
void Mesh::EmptyExtra()
{
	vertexNormals.Empty();
	vertexVertices.Empty();
	vertexFaces.Empty();
	vertexBoundary.Empty();
	faceNormals.Empty();
	faceFaces.Empty();
	faceTexcoords.Empty();
	texturesDiffuse.Empty();
} // EmptyExtra
void Mesh::Swap(Mesh& rhs)
{
	vertices.Swap(rhs.vertices);
	faces.Swap(rhs.faces);
	vertexNormals.Swap(rhs.vertexNormals);
	vertexVertices.Swap(rhs.vertexVertices);
	vertexFaces.Swap(rhs.vertexFaces);
	vertexBoundary.Swap(rhs.vertexBoundary);
	faceNormals.Swap(rhs.faceNormals);
	faceFaces.Swap(rhs.faceFaces);
	faceTexcoords.Swap(rhs.faceTexcoords);
	faceTexindices.Swap(rhs.faceTexindices);
	std::swap(texturesDiffuse, rhs.texturesDiffuse);
} // Swap
// combine this mesh with the given mesh, without removing duplicate vertices
void Mesh::Join(const Mesh& mesh)
{
	ASSERT(!HasTexture() && !mesh.HasTexture());
	vertexVertices.Release();
	vertexFaces.Release();
	vertexBoundary.Release();
	faceFaces.Release();
	if (IsEmpty()) {
		*this = mesh;
		return;
	}
	const VIndex offsetV(vertices.size());
	vertices.Join(mesh.vertices);
	vertexNormals.Join(mesh.vertexNormals);
	faces.ReserveExtra(mesh.faces.size());
	for (const Face& face: mesh.faces)
		faces.emplace_back(face.x+offsetV, face.y+offsetV, face.z+offsetV);
	faceNormals.Join(mesh.faceNormals);
}
/*----------------------------------------------------------------*/


bool Mesh::IsWatertight()
{
	if (vertexBoundary.empty()) {
		if (vertexFaces.empty())
			ListIncidenteFaces();
		ListBoundaryVertices();
	}
	for (const bool b : vertexBoundary)
		if (b)
			return false;
	return true;
}

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

// compute the center of the point-cloud as the median
Mesh::Vertex Mesh::GetCenter() const
{
	const VIndex step(5);
	const VIndex numPoints(vertices.size()/step);
	if (numPoints == 0)
		return Vertex::INF;
	typedef CLISTDEF0IDX(Vertex::Type,VIndex) Scalars;
	Scalars x(numPoints), y(numPoints), z(numPoints);
	for (VIndex i=0; i<numPoints; ++i) {
		const Vertex& X = vertices[i*step];
		x[i] = X.x;
		y[i] = X.y;
		z[i] = X.z;
	}
	return Vertex(x.GetMedian(), y.GetMedian(), z.GetMedian());
}
/*----------------------------------------------------------------*/


// extract array of vertices incident to each vertex
void Mesh::ListIncidenteVertices()
{
	vertexVertices.clear();
	vertexVertices.resize(vertices.size());
	FOREACH(i, faces) {
		const Face& face = faces[i];
		for (int v=0; v<3; ++v) {
			VertexIdxArr& verts(vertexVertices[face[v]]);
			for (int i=1; i<3; ++i) {
				const VIndex idxVert(face[(v+i)%3]);
				if (verts.Find(idxVert) == VertexIdxArr::NO_INDEX)
					verts.emplace_back(idxVert);
			}
		}
	}
}

// extract the (ordered) array of triangles incident to each vertex
void Mesh::ListIncidenteFaces()
{
	vertexFaces.clear();
	vertexFaces.resize(vertices.size());
	FOREACH(i, faces) {
		const Face& face = faces[i];
		for (int v=0; v<3; ++v) {
			ASSERT(vertexFaces[face[v]].Find(i) == FaceIdxArr::NO_INDEX);
			vertexFaces[face[v]].emplace_back(i);
		}
	}
}

// extract array face adjacencies for each face in the mesh (3 * number of faces);
// each triple describes the adjacent face triangles for a given face
// in the following edge order: v1v2, v2v3, v3v1;
// NO_ID indicates there is no adjacent face on that edge
void Mesh::ListIncidenteFaceFaces()
{
	ASSERT(vertexFaces.size() == vertices.size());
	struct inserter_data_t {
		const FIndex idxF;
		FaceFaces& faces;
		int idx;
		inline inserter_data_t(FIndex _idxF, FaceFaces& _faces) : idxF(_idxF), faces(_faces), idx(0) {}
		inline void operator=(FIndex f) { faces[idx++] = f; }
	};
	struct face_back_inserter_t {
		inserter_data_t* data;
		inline face_back_inserter_t(inserter_data_t& _data) : data(&_data) {}
		inline face_back_inserter_t& operator*() { return *this; }
		inline face_back_inserter_t& operator++() { return *this; }
		inline void operator=(FIndex f) { if (f != data->idxF) *data = f; }
	};
	faceFaces.resize(faces.size());
	FOREACH(f, faces) {
		const Face& face = faces[f];
		const FaceIdxArr* const pFaces[] = {&vertexFaces[face[0]], &vertexFaces[face[1]], &vertexFaces[face[2]]};
		inserter_data_t inserterData(f, faceFaces[f]);
		face_back_inserter_t faceBackInserter(inserterData);
		for (int v=0; v<3; ++v) {
			const FaceIdxArr& facesI = *pFaces[v];
			const FaceIdxArr& facesJ = *pFaces[(v+1)%3];
			std::set_intersection(
				facesI.begin(), facesI.end(),
				facesJ.begin(), facesJ.end(),
				faceBackInserter);
			if (inserterData.idx == v)
				inserterData = NO_ID;
		}
	}
}

// check each vertex if it is at the boundary or not
// (make sure you called ListIncidenteFaces() before)
void Mesh::ListBoundaryVertices()
{
	vertexBoundary.clear();
	vertexBoundary.resize(vertices.size());
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
	faceNormals.resize(faces.size());
	#ifndef _USE_CUDA
	FOREACH(idxFace, faces)
		faceNormals[idxFace] = normalized(FaceNormal(faces[idxFace]));
	#else
	if (kernelComputeFaceNormal.IsValid()) {
		reportCudaError(kernelComputeFaceNormal((int)faces.size(),
			vertices,
			faces,
			CUDA::KernelRT::OutputParam(faceNormals.GetDataSize()),
			faces.size()
		));
		reportCudaError(kernelComputeFaceNormal.GetResult(0,
			faceNormals
		));
		kernelComputeFaceNormal.Reset();
	} else {
		FOREACH(idxFace, faces)
			faceNormals[idxFace] = normalized(FaceNormal(faces[idxFace]));
	}
	#endif
}

// compute normal for all vertices
#if 1
// computes the vertex normal as the area weighted face normals average
void Mesh::ComputeNormalVertices()
{
	vertexNormals.resize(vertices.size());
	vertexNormals.Memset(0);
	for (const Face& face: faces) {
		const Vertex& v0 = vertices[face[0]];
		const Vertex& v1 = vertices[face[1]];
		const Vertex& v2 = vertices[face[2]];
		const Normal t((v1 - v0).cross(v2 - v0));
		vertexNormals[face[0]] += t;
		vertexNormals[face[1]] += t;
		vertexNormals[face[2]] += t;
	}
	for (Normal& vertexNormal: vertexNormals)
		normalize(vertexNormal);
}
#else
// computes the vertex normal as an angle weighted average
// (the vertex first ring of faces and the face normals are used)
//
// The normal of a vertex v computed as a weighted sum f the incident face normals.
// The weight is simply the angle of the involved wedge. Described in:
// G. Thurmer, C. A. Wuthrich "Computing vertex normals from polygonal facets", Journal of Graphics Tools, 1998
void Mesh::ComputeNormalVertices()
{
	ASSERT(!faceNormals.empty());
	vertexNormals.resize(vertices.size());
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
		vertexNormals[face[0]] += t*ACOS(-ComputeAngleN(e0.ptr(), e2.ptr()));
		vertexNormals[face[1]] += t*ACOS(-ComputeAngleN(e0.ptr(), e1.ptr()));
		vertexNormals[face[2]] += t*ACOS(-ComputeAngleN(e1.ptr(), e2.ptr()));
	}
	for (Normal& vertexNormal: vertexNormals)
		normalize(vertexNormal);
}
#endif

// Smoothen the normals for each face
//  - fMaxGradient: maximum angle (in degrees) difference between neighbor normals that is
//    allowed to take into consideration; higher angles are ignored
//  - fOriginalWeight: weight (0..1] to use for current normal value when averaging with neighbor normals
//  - nIterations: number of times to repeat the smoothening process
void Mesh::SmoothNormalFaces(float fMaxGradient, float fOriginalWeight, unsigned nIterations) {
	if (faceNormals.size() != faces.size())
		ComputeNormalFaces();
	if (vertexFaces.size() != vertices.size())
		ListIncidenteFaces();
	if (faceFaces.size() != faces.size())
		ListIncidenteFaceFaces();
	const float cosMaxGradient = COS(FD2R(fMaxGradient));
	for (unsigned rep = 0; rep < nIterations; ++rep) {
		NormalArr newFaceNormals(faceNormals.size());
		FOREACH(idxFace, faces) {
			const Normal& originalNormal = faceNormals[idxFace];
			Normal sumNeighborNormals = Normal::ZERO;
			for (int i = 0; i < 3; ++i) {
				const FIndex fIdx = faceFaces[idxFace][i];
				if (fIdx == NO_ID)
					continue;
				const Normal& neighborNormal = faceNormals[fIdx];
				if (ComputeAngleN(originalNormal.ptr(), neighborNormal.ptr()) >= cosMaxGradient)
					sumNeighborNormals += neighborNormal;
			}
			const Normal avgNeighborsNormal = normalized(sumNeighborNormals);
			const Normal newFaceNormal = normalized(originalNormal * fOriginalWeight + avgNeighborsNormal * (1.f - fOriginalWeight));
			newFaceNormals[idxFace] = newFaceNormal;
		}
		newFaceNormals.Swap(faceNormals);
	}
}
/*----------------------------------------------------------------*/


void Mesh::GetEdgeFaces(VIndex v0, VIndex v1, FaceIdxArr& afaces) const
{
	const FaceIdxArr& faces0 = vertexFaces[v0];
	const FaceIdxArr& faces1 = vertexFaces[v1];
	std::unordered_set<FIndex> setFaces1(faces1.begin(), faces1.end());
	for (FIndex idxFace: faces0) {
		if (setFaces1.find(idxFace) != setFaces1.end())
			afaces.Insert(idxFace);
	}
}

void Mesh::GetFaceFaces(FIndex f, FaceIdxArr& afaces) const
{
	const Face& face = faces[f];
	const FaceIdxArr& faces0 = vertexFaces[face[0]];
	const FaceIdxArr& faces1 = vertexFaces[face[1]];
	const FaceIdxArr& faces2 = vertexFaces[face[2]];
	std::unordered_set<FIndex> setFaces(faces1.begin(), faces1.end());
	for (FIndex idxFace: faces0) {
		if (f != idxFace && setFaces.find(idxFace) != setFaces.end())
			afaces.InsertSortUnique(idxFace);
	}
	for (FIndex idxFace: faces2) {
		if (f != idxFace && setFaces.find(idxFace) != setFaces.end())
			afaces.InsertSortUnique(idxFace);
	}
	setFaces.clear();
	setFaces.insert(faces2.begin(), faces2.end());
	for (FIndex idxFace: faces0) {
		if (f != idxFace && setFaces.find(idxFace) != setFaces.end())
			afaces.InsertSortUnique(idxFace);
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

// get the edge orientation in the given face:
// return false for backward, true for forward
bool Mesh::GetEdgeOrientation(FIndex idxFace, VIndex iV0, VIndex iV1) const
{
	const Face& face = faces[idxFace];
	const VIndex i0 = FindVertex(face, iV0);
	ASSERT(i0 != NO_ID);
	ASSERT(face[(i0+1)%3] == iV1 || face[(i0+2)%3] == iV1);
	return face[(i0+1)%3] == iV1;
}

// find the adjacent face for the given face edge;
// return NO_ID if no adjacent faces exist OR
// more than one adjacent face exist OR
// the edge have opposite orientations in each face
Mesh::FIndex Mesh::GetEdgeAdjacentFace(FIndex idxFace, VIndex iV0, VIndex iV1) const
{
	// iterate over all faces containing the first vertex
	ASSERT(vertexFaces.size() == vertices.size());
	const bool edgeOrientation = GetEdgeOrientation(idxFace, iV0, iV1);
	FIndex idxFaceAdj = NO_ID;
	for (FIndex iF: vertexFaces[iV0]) {
		// if this adjacent face is not the analyzed face
		if (iF != idxFace) {
			// iterate over all face vertices
			const Face& face = faces[iF];
			for (int i = 0; i < 3; ++i) {
				// if the face vertex is the second vertex
				if (face[i] == iV1) {
					// check if there are more than two adjacent faces (manifold constraint)
					if (idxFaceAdj != NO_ID)
						return NO_ID;
					// check if edge vertices ordering is opposite in the two faces (manifold constraint)
					if (GetEdgeOrientation(iF, iV0, iV1) == edgeOrientation)
						return NO_ID;
					idxFaceAdj = iF;
				}
			}
		}
	}
	return idxFaceAdj;
}

void Mesh::GetAdjVertices(VIndex v, VertexIdxArr& indices) const
{
	ASSERT(vertexFaces.size() == vertices.size());
	const FaceIdxArr& idxFaces = vertexFaces[v];
	std::unordered_set<VIndex> setIndices;
	for (FIndex idxFace: idxFaces) {
		const Face& face = faces[idxFace];
		for (int i=0; i<3; ++i) {
			const VIndex vAdj(face[i]);
			if (vAdj != v && setIndices.insert(vAdj).second)
				indices.emplace_back(vAdj);
		}
	}
}

void Mesh::GetAdjVertexFaces(VIndex idxVCenter, VIndex idxVAdj, FaceIdxArr& indices) const
{
	ASSERT(vertexFaces.size() == vertices.size());
	const FaceIdxArr& idxFaces = vertexFaces[idxVCenter];
	for (FIndex idxFace: idxFaces) {
		const Face& face = faces[idxFace];
		ASSERT(FindVertex(face, idxVCenter) != NO_ID);
		if (FindVertex(face, idxVAdj) != NO_ID)
			indices.emplace_back(idxFace);
	}
}
/*----------------------------------------------------------------*/

// fix non-manifold vertices and edges;
// return the number of non-manifold issues found
unsigned Mesh::FixNonManifold(float magDisplacementDuplicateVertices, VertexIdxArr* duplicatedVertices)
{
	ASSERT(!vertices.empty() && !faces.empty());
	if (vertexFaces.size() != vertices.size())
		ListIncidenteFaces();
	// iterate over all vertices and separates the components
	// incident to the same vertex by duplicating the vertex
	unsigned numNonManifoldIssues(0);
	CLISTDEF0IDX(int, FIndex) components(faces.size());
	FOREACH(idxVert, vertices) {
		// reset component indices to which each face connected to this vertex
		const FaceIdxArr& vertFaces = vertexFaces[idxVert];
		for (FIndex iF: vertFaces)
			components[iF] = -1;
		// find the components connected to this vertex
		FaceIdxArr queueFaces;
		queueFaces.reserve(vertFaces.size());
		FIndex idxFaceNext(0);
		int component(0);
		for ( ; ; ++component) {
			// find one face not yet belonging to a component
			while (idxFaceNext < vertFaces.size()) {
				const FIndex iF(vertFaces[idxFaceNext++]);
				if (components[iF] == -1) {
					// add component as seed to the list
					queueFaces.push_back(iF);
					// mark the current face with a new component
					components[iF] = component;
					// process component
					goto ProcessComponent;
				}
			}
			// no more components found
			break;
			ProcessComponent:
			// grow seed face component until no more connected faces found
			do {
				const FIndex idxFaceCurrent(queueFaces.back());
				queueFaces.pop_back();
				const Face& face = faces[idxFaceCurrent];
				// go over all vertices of the current face
				for (int i = 0; i < 3; ++i) {
					const VIndex idxVertAdj(face[i]);
					if (idxVertAdj == idxVert)
						continue;
					// if there is exactly one face adjacent to this edge
					// tag it with the current component and add it to the queue
					const FIndex idxFaceAdj(GetEdgeAdjacentFace(idxFaceCurrent, idxVert, idxVertAdj));
					if (idxFaceAdj != NO_ID && components[idxFaceAdj] == -1) {
						components[idxFaceAdj] = component;
						queueFaces.push_back(idxFaceAdj);
					}
				}
			} while (!queueFaces.empty());
		}
		// if there is only one component, continue with the next vertex
		if (component <= 1)
			continue;
		// separate the vertex components
		for (int c = 1; c < component; ++c) {
			// duplicate the point to achieve the separation
			const VIndex idxVertNew = vertices.size();
			const Vertex v = vertices[idxVert];
			vertices.emplace_back(v);
			if (duplicatedVertices)
				duplicatedVertices->emplace_back(idxVert);
			// update the face indices of the current component
			FaceIdxArr& vertFacesNew = vertexFaces.emplace_back();
			FaceIdxArr& vertFaces = vertexFaces[idxVert];
			RFOREACH(ivf, vertFaces) {
				const FIndex idxFace = vertFaces[ivf];
				if (components[idxFace] != c)
					continue;
				// link face to the new vertex and remove it from the original vertex
				Face& face = faces[idxFace];
				for (int i = 0; i < 3; ++i) {
					if (face[i] == idxVert) {
						face[i] = idxVertNew;
						vertFacesNew.InsertAt(0, idxFace);
						break;
					}
				}
				vertFaces.RemoveAtMove(ivf);
			}
			++numNonManifoldIssues;
		}
		// adjust vertex positions
		if (magDisplacementDuplicateVertices > 0) {
			// list changed vertices
			VertexIdxArr verts(component);
			verts[0] = idxVert;
			for (int c = 1; c < component; ++c)
				verts[c] = vertices.size()-(component-c);
			// adjust the position of the vertices in the direction
			// to the center of the first ring of faces
			FOREACH(i, verts) {
				const VIndex idxVert(verts[i]);
				VertexIdxArr adjVerts;
				GetAdjVertices(idxVert, adjVerts);
				TAccumulator<Vertex> accum;
				for (VIndex iV: adjVerts)
					accum.Add(vertices[iV], 1.f);
				const Vertex bv(accum.Normalized());
				Vertex& v(vertices[idxVert]);
				const Vertex dir(bv-v);
				v += dir * magDisplacementDuplicateVertices;
			}
		}
	}
	vertexFaces.Release();
	return numNonManifoldIssues;
}
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
class Edge   : public vcg::Edge<  UsedTypes, vcg::edge::VertexRef, vcg::edge::Mark, vcg::edge::BitFlags> {};

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

typedef vcg::tri::BasicVertexPair<Vertex> VertexPair;

class TriEdgeCollapse : public vcg::tri::TriEdgeCollapseQuadric<Mesh, VertexPair, TriEdgeCollapse, QHelper> {
public:
	typedef vcg::tri::TriEdgeCollapseQuadric<Mesh, VertexPair, TriEdgeCollapse, QHelper> TECQ;
	inline TriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp) :TECQ(p, i, pp) {}
};
}

// decimate, clean and smooth mesh
// fDecimate factor is in range (0..1], if 1 no decimation takes place
void Mesh::Clean(float fDecimate, float fSpurious, bool bRemoveSpikes, unsigned nCloseHoles, unsigned nSmooth, float fEdgeLength, bool bLastClean)
{
	if (vertices.empty() || faces.empty())
		return;
	TD_TIMER_STARTD();
	// create VCG mesh
	CLEAN::Mesh mesh;
	{
		CLEAN::Mesh::VertexIterator vi = vcg::tri::Allocator<CLEAN::Mesh>::AddVertices(mesh, vertices.size());
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
		CLEAN::Mesh::FaceIterator fi = vcg::tri::Allocator<CLEAN::Mesh>::AddFaces(mesh, faces.size());
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
		const int nZeroAreaFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveZeroAreaFace(mesh);
		DEBUG_ULTIMATE("Removed %d zero-area faces", nZeroAreaFaces);
		const int nDuplicateFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateFace(mesh);
		DEBUG_ULTIMATE("Removed %d duplicate faces", nDuplicateFaces);
		const int nUnreferencedVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveUnreferencedVertex(mesh);
		DEBUG_ULTIMATE("Removed %d unreferenced vertices", nUnreferencedVertices);
		vcg::tri::TriEdgeCollapseQuadricParameter pp;
		pp.QualityThr = 0.3; // Quality Threshold for penalizing bad shaped faces: the value is in the range [0..1], 0 accept any kind of face (no penalties), 0.5 penalize faces with quality < 0.5, proportionally to their shape
		pp.PreserveBoundary = false; // the simplification process tries to not affect mesh boundaries during simplification
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
		const int nZeroAreaFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveZeroAreaFace(mesh);
		DEBUG_ULTIMATE("Removed %d zero-area faces", nZeroAreaFaces);
		const int nDuplicateFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateFace(mesh);
		DEBUG_ULTIMATE("Removed %d duplicate faces", nDuplicateFaces);
		const int nUnreferencedVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveUnreferencedVertex(mesh);
		DEBUG_ULTIMATE("Removed %d unreferenced vertices", nUnreferencedVertices);
		vcg::tri::UpdateTopology<CLEAN::Mesh>::FaceFace(mesh);
		const int nNonManifoldFaces = vcg::tri::Clean<CLEAN::Mesh>::RemoveNonManifoldFace(mesh);
		DEBUG_ULTIMATE("Removed %d non-manifold faces", nNonManifoldFaces);
		const int nDegenerateVertices = vcg::tri::Clean<CLEAN::Mesh>::RemoveDegenerateVertex(mesh);
		DEBUG_ULTIMATE("Removed %d degenerate vertices", nDegenerateVertices);
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
		const auto ret(ComputeX84Threshold<float,float>(edgeLens.Begin(), edgeLens.size(), 3.f*fSpurious));
		const float thLongEdge(ret.first+ret.second);
		#else
		const float thLongEdge(edgeLens.GetNth(edgeLens.size()*95/100)*fSpurious);
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

	// remesh
	if (fEdgeLength > 0) {
		vcg::tri::Clean<CLEAN::Mesh>::RemoveDuplicateVertex(mesh);
		vcg::tri::Clean<CLEAN::Mesh>::RemoveUnreferencedVertex(mesh);
		vcg::tri::Allocator<CLEAN::Mesh>::CompactEveryVector(mesh);
		CLEAN::Mesh original;
		vcg::tri::Append<CLEAN::Mesh,CLEAN::Mesh>::MeshCopy(original, mesh);
		vcg::tri::IsotropicRemeshing<CLEAN::Mesh>::Params params;
		params.SetTargetLen(fEdgeLength);
		params.iter = 3;
		params.surfDistCheck = false;
		params.maxSurfDist = fEdgeLength * 0.4f;
		params.cleanFlag = true;
		params.userSelectedCreases = false;
		try {
			vcg::tri::IsotropicRemeshing<CLEAN::Mesh>::Do(mesh, original, params);
		}
		catch(vcg::MissingPreconditionException& e) {
			VERBOSE("error: %s", e.what());
		}
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
		ASSERT(vertices.empty() && faces.empty());
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
	DEBUG("Cleaned mesh: %u vertices, %u faces (%s)", vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
} // Clean
/*----------------------------------------------------------------*/


// project vertices and compute bounding-box;
// account for diferences in pixel center convention: while OpenMVS uses the same convention as OpenCV and DirectX 9 where the center
// of a pixel is defined at integer coordinates, i.e. the center is at (0, 0) and the top left corner is at (-0.5, -0.5),
// DirectX 10+, OpenGL, and Vulkan convention is the center of a pixel is defined at half coordinates, i.e. the center is at (0.5, 0.5)
// and the top left corner is at (0, 0)
static const Mesh::TexCoord halfPixel(0.5f, 0.5f);

// translate, normalize and flip Y axis of the texture coordinates
void Mesh::FaceTexcoordsNormalize(TexCoordArr& newFaceTexcoords, bool flipY) const
{
	ASSERT(HasTextureCoordinates());
	newFaceTexcoords.resize(faceTexcoords.size());
	if (texturesDiffuse.empty()) {
		if (flipY) {
			FOREACH(i, faceTexcoords) {
				const TexCoord& texcoord = faceTexcoords[i];
				newFaceTexcoords[i] = TexCoord(texcoord.x, 1.f-texcoord.y);
			}
		} else
			newFaceTexcoords = faceTexcoords;
	} else {
		TexCoordArr invNorms(texturesDiffuse.size());
		FOREACH(i, texturesDiffuse) {
			ASSERT(!texturesDiffuse[i].empty());
			invNorms[i] = TexCoord(1.f/(float)texturesDiffuse[i].cols, 1.f/(float)texturesDiffuse[i].rows);
		}
		if (flipY) {
			FOREACH(i, faceTexcoords) {
				const TexCoord& texcoord = faceTexcoords[i];
				const TexCoord& invNorm = invNorms[GetFaceTextureIndex(i/3)];
				newFaceTexcoords[i] = TexCoord(
					(texcoord.x+halfPixel.x)*invNorm.x,
					1.f-(texcoord.y+halfPixel.y)*invNorm.y
				);
			}
		} else {
			FOREACH(i, faceTexcoords) {
				const TexCoord& invNorm = invNorms[GetFaceTextureIndex(i/3)];
				newFaceTexcoords[i] = (faceTexcoords[i]+halfPixel)*invNorm;
			}
		}
	}
} // FaceTexcoordsNormalize

// flip Y axis, unnormalize and translate back texture coordinates
void Mesh::FaceTexcoordsUnnormalize(TexCoordArr& newFaceTexcoords, bool flipY) const
{
	ASSERT(HasTextureCoordinates());
	newFaceTexcoords.resize(faceTexcoords.size());
	if (texturesDiffuse.empty()) {
		if (flipY) {
			FOREACH(i, faceTexcoords) {
				const TexCoord& texcoord = faceTexcoords[i];
				newFaceTexcoords[i] = TexCoord(texcoord.x, 1.f-texcoord.y);
			}
		} else
			newFaceTexcoords = faceTexcoords;
	} else {
		TexCoordArr scales(texturesDiffuse.size());
		FOREACH(i, texturesDiffuse) {
			ASSERT(!texturesDiffuse[i].empty());
			scales[i] = TexCoord((float)texturesDiffuse[i].cols, (float)texturesDiffuse[i].rows);
		}
		if (flipY) {
			FOREACH(i, faceTexcoords) {
				const TexCoord& texcoord = faceTexcoords[i];
				const TexCoord& scale = scales[GetFaceTextureIndex(i/3)];
				newFaceTexcoords[i] = TexCoord(
					texcoord.x*scale.x-halfPixel.x,
					(1.f-texcoord.y)*scale.y-halfPixel.y
				);
			}
		} else {
			FOREACH(i, faceTexcoords) {
				const TexCoord& scale = scales[GetFaceTextureIndex(i/3)];
				newFaceTexcoords[i] = faceTexcoords[i]*scale - halfPixel;
			}
		}
	}
} // FaceTexcoordsUnnormalize
/*----------------------------------------------------------------*/


// define a PLY file format composed only of vertices and triangles
namespace MeshInternal {
namespace BasicPLY {
	// list of the kinds of elements in the PLY
	static const char* elem_names[] = {
		"vertex",
		"face"
	};
	// list of property information for a vertex
	struct Vertex {
		Mesh::Vertex v;
		Mesh::Normal n;
		static void InitLoadProps(PLY& ply, int elem_count,
			Mesh::VertexArr& vertices, Mesh::NormalArr& vertexNormals)
		{
			PLY::PlyElement* elm = ply.find_element(elem_names[0]);
			const size_t nMaxProps(SizeOfArray(props));
			for (size_t p=0; p<nMaxProps; ++p) {
				if (ply.find_property(elm, props[p].name.c_str()) < 0)
					continue;
				ply.setup_property(props[p]);
				switch (p) {
				case 0: vertices.resize((IDX)elem_count); break;
				case 3: vertexNormals.resize((IDX)elem_count); break;
				}
			}
		}
		static void Select(PLY& ply) {
			ply.put_element_setup(elem_names[0]);
		}
		static void InitSaveProps(PLY& ply, int elem_count,
			bool bNormals)
		{
			ply.describe_property(elem_names[0], 3, props+0);
			if (bNormals)
				ply.describe_property(elem_names[0], 3, props+3);
			if (elem_count)
				ply.element_count(elem_names[0], elem_count);
		}
		static const PLY::PlyProperty props[9];
	};
	const PLY::PlyProperty Vertex::props[] = {
		{"x",             PLY::Float32, PLY::Float32, offsetof(Vertex,v.x), 0, 0, 0, 0},
		{"y",             PLY::Float32, PLY::Float32, offsetof(Vertex,v.y), 0, 0, 0, 0},
		{"z",             PLY::Float32, PLY::Float32, offsetof(Vertex,v.z), 0, 0, 0, 0},
		{"nx",            PLY::Float32, PLY::Float32, offsetof(Vertex,n.x), 0, 0, 0, 0},
		{"ny",            PLY::Float32, PLY::Float32, offsetof(Vertex,n.y), 0, 0, 0, 0},
		{"nz",            PLY::Float32, PLY::Float32, offsetof(Vertex,n.z), 0, 0, 0, 0}
	};
	// list of property information for a face
	struct Face {
		struct FaceIndices {
			uint8_t num;
			Mesh::Face* pFace;
		} face;
		struct TexCoord {
			uint8_t num;
			Mesh::TexCoord* pTex;
		} tex;
		Mesh::TexIndex texId;
		float weight;
		static void InitLoadProps(PLY& ply, int elem_count,
			Mesh::FaceArr& faces, Mesh::TexCoordArr& faceTexcoords, Mesh::TexIndexArr& faceTexindices)
		{
			PLY::PlyElement* elm = ply.find_element(elem_names[1]);
			const size_t nMaxProps(SizeOfArray(props));
			for (size_t p=0; p<nMaxProps; ++p) {
				if (ply.find_property(elm, props[p].name.c_str()) < 0)
					continue;
				ply.setup_property(props[p]);
				switch (p) {
				case 0: faces.resize((IDX)elem_count); break;
				case 1: faceTexcoords.resize((IDX)elem_count*3); break;
				case 2: faceTexindices.resize((IDX)elem_count); break;
				}
			}
		}
		static void Select(PLY& ply) {
			ply.put_element_setup(elem_names[1]);
		}
		static void InitSaveProps(
			PLY& ply, int elem_count,
			bool bFaces, bool bTexcoord, bool bTexnumber, bool bFaceweight=false)
		{
			if (bFaces)
				ply.describe_property(elem_names[1], props[0]);
			if (bTexcoord)
				ply.describe_property(elem_names[1], props[1]);
			if (bTexnumber)
				ply.describe_property(elem_names[1], props[2]);
			if (bFaceweight)
				ply.describe_property(elem_names[1], props[3]);
			if (elem_count)
				ply.element_count(elem_names[1], elem_count);
		}
		static const PLY::PlyProperty props[4];
	};
	const PLY::PlyProperty Face::props[] = {
		{"vertex_indices", PLY::Uint32,  PLY::Uint32,  offsetof(Face,face.pFace), 1, PLY::Uint8, PLY::Uint8, offsetof(Face,face.num)},
		{"texcoord",       PLY::Float32, PLY::Float32, offsetof(Face,tex.pTex),   1, PLY::Uint8, PLY::Uint8, offsetof(Face,tex.num)},
		{"texnumber",      PLY::Int32,   PLY::Uint8,   offsetof(Face,texId),      0, 0,          0,          0},
		{"weight",         PLY::Float32, PLY::Float32, offsetof(Face,weight),     0, 0,          0,          0},
	};
} // namespace BasicPLY
} // namespace MeshInternal

// import the mesh from the given file
bool Mesh::Load(const String& fileName)
{
	TD_TIMER_STARTD();
	const String ext(Util::getFileExt(fileName).ToLower());
	bool ret;
	if (ext == _T(".obj"))
		ret = LoadOBJ(fileName);
	else
	if (ext == _T(".gltf") || ext == _T(".glb"))
		ret = LoadGLTF(fileName, ext == _T(".glb"));
	else
		ret = LoadPLY(fileName);
	if (!ret)
		return false;
	DEBUG_EXTRA("Mesh loaded: %u vertices, %u faces (%s)", vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}
// import the mesh as a PLY file
bool Mesh::LoadPLY(const String& fileName)
{
	ASSERT(!fileName.empty());
	Release();

	// open PLY file and read header
	using namespace MeshInternal;
	PLY ply;
	if (!ply.read(fileName)) {
		DEBUG_EXTRA("error: invalid PLY file");
		return false;
	}
	for (int i = 0; i < ply.get_elements_count(); ++i) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			vertices.resize(elem_count);
		} else
		if (PLY::equal_strings(BasicPLY::elem_names[1], elem_name)) {
			faces.resize(elem_count);
		}
	}
	if (vertices.empty() && faces.empty())
		return true;
	if (vertices.empty() || faces.empty()) {
		DEBUG_EXTRA("error: invalid mesh file");
		return false;
	}

	// read PLY body
	for (int i = 0; i < ply.get_elements_count(); i++) {
		int elem_count;
		LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
		if (PLY::equal_strings(BasicPLY::elem_names[0], elem_name)) {
			ASSERT(vertices.size() == (VIndex)elem_count);
			BasicPLY::Vertex::InitLoadProps(ply, elem_count, vertices, vertexNormals);
			if (vertexNormals.empty()) {
				for (Vertex& vert: vertices)
					ply.get_element(&vert);
			} else {
				BasicPLY::Vertex vertex;
				for (int v=0; v<elem_count; ++v) {
					ply.get_element(&vertex);
					vertices[v] = vertex.v;
					vertexNormals[v] = vertex.n;
				}
			}
		} else
		if (PLY::equal_strings(BasicPLY::elem_names[1], elem_name)) {
			ASSERT(faces.size() == (FIndex)elem_count);
			BasicPLY::Face::InitLoadProps(ply, elem_count, faces, faceTexcoords, faceTexindices);
			BasicPLY::Face face;
			FOREACH(f, faces) {
				ply.get_element(&face);
				if (face.face.num != 3) {
					DEBUG_EXTRA("error: unsupported mesh file (face not triangle)");
					return false;
				}
				memcpy(faces.data()+f, face.face.pFace, sizeof(Face));
				delete[] face.face.pFace;
				if (!faceTexcoords.empty()) {
					if (face.tex.num != 6) {
						DEBUG_EXTRA("error: unsupported mesh file (texture coordinates not per face vertex)");
						return false;
					}
					memcpy(faceTexcoords.data()+f*3, face.tex.pTex, sizeof(TexCoord)*3);
					delete[] face.tex.pTex;
				}
				if (!faceTexindices.empty())
					faceTexindices[f] = face.texId;
			}
			if (!faceTexcoords.empty()) {
				// load the texture
				for (const std::string& comment: ply.get_comments()) {
					if (_tcsncmp(comment.c_str(), _T("TextureFile "), 12) == 0) {
						const String textureFileName(comment.substr(12));
						texturesDiffuse.emplace_back().Load(Util::getFilePath(fileName)+textureFileName);
					}
				}
				if (texturesDiffuse.size() <= 1)
					faceTexindices.Release();
				// flip Y axis, unnormalize and translate back texture coordinates
				TexCoordArr unnormFaceTexcoords;
				FaceTexcoordsUnnormalize(unnormFaceTexcoords, true);
				faceTexcoords.Swap(unnormFaceTexcoords);
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
	ASSERT(!fileName.empty());
	Release();

	// open and parse OBJ file
	ObjModel model;
	if (!model.Load(fileName)) {
		DEBUG_EXTRA("error: invalid OBJ file");
		return false;
	}

	if (model.get_vertices().empty() || model.get_groups().empty()) {
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
	FOREACH(groupIdx, model.get_groups()) {
		const auto& group = model.get_groups()[groupIdx];
		ASSERT(group.faces.size() < std::numeric_limits<FIndex>::max());
		faces.reserve((FIndex)group.faces.size());
		for (const ObjModel::Face& f: group.faces) {
			ASSERT(f.vertices[0] != NO_ID);
			faces.emplace_back(f.vertices[0], f.vertices[1], f.vertices[2]);
			if (f.texcoords[0] != NO_ID) {
				for (int i=0; i<3; ++i)
					faceTexcoords.emplace_back(model.get_texcoords()[f.texcoords[i]]);
				faceTexindices.emplace_back((TexIndex)groupIdx);
			}
			if (f.normals[0] != NO_ID) {
				Normal& n = faceNormals.emplace_back(Normal::ZERO);
				for (int i=0; i<3; ++i)
					n += normalized(model.get_normals()[f.normals[i]]);
				normalize(n);
			}
		}
		// store texture
		ObjModel::MaterialLib::Material* pMaterial(model.GetMaterial(group.material_name));
		if (pMaterial && pMaterial->LoadDiffuseMap())
			texturesDiffuse.emplace_back(pMaterial->diffuse_map);
	}

	// flip Y axis, unnormalize and translate back texture coordinates
	if (!faceTexcoords.empty()) {
		if (texturesDiffuse.size() <= 1)
			faceTexindices.Release();
		TexCoordArr unnormFaceTexcoords;
		FaceTexcoordsUnnormalize(unnormFaceTexcoords, true);
		faceTexcoords.Swap(unnormFaceTexcoords);
	}
	return true;
}
// import the mesh as a GLTF file
bool Mesh::LoadGLTF(const String& fileName, bool bBinary)
{
	ASSERT(!fileName.empty());
	Release();

	// load model
	tinygltf::Model gltfModel; {
		tinygltf::TinyGLTF loader;
		std::string err, warn;
		if (bBinary ?
			!loader.LoadBinaryFromFile(&gltfModel, &err, &warn, fileName) :
			!loader.LoadASCIIFromFile(&gltfModel, &err, &warn, fileName))
			return false;
		if (!err.empty()) {
			VERBOSE("error: %s", err.c_str());
			return false;
		}
		if (!warn.empty())
			DEBUG("warning: %s", warn.c_str());
	}

	// parse model
	for (const tinygltf::Mesh& gltfMesh : gltfModel.meshes) {
		for (const tinygltf::Primitive& gltfPrimitive : gltfMesh.primitives) {
			if (gltfPrimitive.mode != TINYGLTF_MODE_TRIANGLES)
				continue;
			Mesh mesh;
			// read vertices
			{
				const tinygltf::Accessor& gltfAccessor = gltfModel.accessors[gltfPrimitive.attributes.at("POSITION")];
				if (gltfAccessor.type != TINYGLTF_TYPE_VEC3)
					continue;
				const tinygltf::BufferView& gltfBufferView = gltfModel.bufferViews[gltfAccessor.bufferView];
				const tinygltf::Buffer& buffer = gltfModel.buffers[gltfBufferView.buffer];
				const uint8_t* pData = buffer.data.data() + gltfBufferView.byteOffset + gltfAccessor.byteOffset;
				mesh.vertices.resize((VIndex)gltfAccessor.count);
				if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
					ASSERT(gltfBufferView.byteLength == sizeof(Vertex) * gltfAccessor.count);
					memcpy(mesh.vertices.data(), pData, gltfBufferView.byteLength);
				}
				else if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_DOUBLE) {
					for (VIndex i = 0; i < gltfAccessor.count; ++i)
						mesh.vertices[i] = ((const Point3d*)pData)[i];
				}
				else {
					VERBOSE("error: unsupported vertices (component type)");
					continue;
				}
			}
			// read faces
			{
				const tinygltf::Accessor& gltfAccessor = gltfModel.accessors[gltfPrimitive.indices];
				if (gltfAccessor.type != TINYGLTF_TYPE_SCALAR)
					continue;
				const tinygltf::BufferView& gltfBufferView = gltfModel.bufferViews[gltfAccessor.bufferView];
				const tinygltf::Buffer& buffer = gltfModel.buffers[gltfBufferView.buffer];
				const uint8_t* pData = buffer.data.data() + gltfBufferView.byteOffset + gltfAccessor.byteOffset;
				mesh.faces.resize((FIndex)(gltfAccessor.count/3));
				if (gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_INT ||
					gltfAccessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
					ASSERT(gltfBufferView.byteLength == sizeof(uint32_t) * gltfAccessor.count);
					memcpy(mesh.faces.data(), pData, gltfBufferView.byteLength);
				}
				else {
					VERBOSE("error: unsupported faces (component type)");
					continue;
				}
			}
			Join(mesh);
		}
	}
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
	if (ext == _T(".gltf") || ext == _T(".glb"))
		ret = SaveGLTF(fileName, ext == _T(".glb"));
	else
		ret = SavePLY(ext != _T(".ply") ? String(fileName+_T(".ply")) : fileName, comments, bBinary);
	if (!ret)
		return false;
	DEBUG_EXTRA("Mesh saved: %u vertices, %u faces (%s)", vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}
// export the mesh as a PLY file
bool Mesh::SavePLY(const String& fileName, const cList<String>& comments, bool bBinary, bool bTexLossless) const
{
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);

	// create PLY object
	using namespace MeshInternal;
	PLY ply;
	if (!ply.write(fileName, 2, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII)) {
		DEBUG_EXTRA("error: can not create the mesh file");
		return false;
	}

	// export comments
	for (const String& comment: comments)
		ply.append_comment(comment);

	// export texture file name as comment if needed
	if (HasTexture()) {
		FOREACH(texId, texturesDiffuse) {
		    const String textureFileName(Util::getFileFullName(fileName) + std::to_string(texId).c_str() + (bTexLossless?_T(".png"):_T(".jpg")));
		    ply.append_comment((_T("TextureFile ")+Util::getFileNameExt(textureFileName)).c_str());
		    texturesDiffuse[texId].Save(textureFileName);
		}
	}

	// describe what properties go into vertex and face elements
	ASSERT(vertexNormals.empty() || vertexNormals.size() == vertices.size());
	BasicPLY::Vertex::InitSaveProps(ply, (int)vertices.size(), !vertexNormals.empty());
	BasicPLY::Face::InitSaveProps(ply, (int)faces.size(), !faces.empty(), !faceTexcoords.empty(), !faceTexindices.empty());
	if (!ply.header_complete())
		return false;

	// export the array of vertices
	BasicPLY::Vertex::Select(ply);
	if (vertexNormals.empty()) {
		FOREACHPTR(pVert, vertices)
			ply.put_element(pVert);
	} else {
		BasicPLY::Vertex v;
		FOREACH(i, vertices) {
			v.v = vertices[i];
			v.n = vertexNormals[i];
			ply.put_element(&v);
		}
	}
	ASSERT(ply.get_current_element_count() == (int)vertices.size());

	// export the array of faces
	BasicPLY::Face::Select(ply);
	BasicPLY::Face face = {{3},{6}};
	if (faceTexcoords.empty()) {
		FOREACHPTR(pFace, faces) {
			face.face.pFace = const_cast<Face*>(pFace);
			ply.put_element(&face);
		}
	} else {
		// translate, normalize and flip Y axis of the texture coordinates
		TexCoordArr normFaceTexcoords;
		FaceTexcoordsNormalize(normFaceTexcoords, true);

		// export the array of faces
		FOREACH(f, faces) {
			face.face.pFace = faces.data()+f;
			face.tex.pTex = normFaceTexcoords.data()+f*3;
			if (!faceTexindices.empty())
				face.texId = faceTexindices[f];
			ply.put_element(&face);
		}
	}
	ASSERT(ply.get_current_element_count() == (int)faces.size());

	return true;
}
// export the mesh as a OBJ file
bool Mesh::SaveOBJ(const String& fileName) const
{
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);

	// create the OBJ model
	ObjModel model;

	// store vertices
	ASSERT(sizeof(ObjModel::Vertex) == sizeof(Vertex));
	model.get_vertices().insert(model.get_vertices().begin(), vertices.begin(), vertices.end());

	// store vertex normals
	ASSERT(sizeof(ObjModel::Normal) == sizeof(Normal));
	ASSERT(model.get_vertices().size() < std::numeric_limits<VIndex>::max());
	if (!vertexNormals.empty()) {
		ASSERT(vertexNormals.size() == vertices.size());
		model.get_normals().insert(model.get_normals().begin(), vertexNormals.begin(), vertexNormals.end());
	}

	// store face texture coordinates
	ASSERT(sizeof(ObjModel::TexCoord) == sizeof(TexCoord));
	if (!faceTexcoords.empty()) {
		// translate, normalize and flip Y axis of the texture coordinates
		TexCoordArr normFaceTexcoords;
		FaceTexcoordsNormalize(normFaceTexcoords, true);
		ASSERT(normFaceTexcoords.size() == faces.size()*3);
		model.get_texcoords().insert(model.get_texcoords().begin(), normFaceTexcoords.begin(), normFaceTexcoords.end());
	}

	// store faces
	FOREACH(idxTexture, texturesDiffuse) {
		ObjModel::Group& group = model.AddGroup(_T("material_" + std::to_string(idxTexture)));
		group.faces.reserve(faces.size());
		FOREACH(idxFace, faces) {
			const auto texIdx = faceTexindices[idxFace];
			if (texIdx != idxTexture)
			    continue;

			const Face& face = faces[idxFace];
			ObjModel::Face f;
			memset(&f, 0xFF, sizeof(ObjModel::Face));
			for (int i=0; i<3; ++i) {
				f.vertices[i] = face[i];
				if (!faceTexcoords.empty())
					f.texcoords[i] = idxFace*3+i;
				if (!vertexNormals.empty())
					f.normals[i] = face[i];
			}
			group.faces.push_back(f);
		}
		ObjModel::MaterialLib::Material* pMaterial(model.GetMaterial(group.material_name));
		ASSERT(pMaterial != NULL);
		pMaterial->diffuse_map = texturesDiffuse[idxTexture];
	}

	return model.Save(fileName);
}
// export the mesh as a GLTF file
template <typename T>
void ExtendBufferGLTF(const T* src, size_t size, tinygltf::Buffer& dst, size_t& byte_offset, size_t& byte_length) {
	byte_offset = dst.data.size();
	byte_length = sizeof(T) * size;
	byte_length = ((byte_length + 3) / 4) * 4;
	dst.data.resize(byte_offset + byte_length);
	memcpy(&dst.data[byte_offset], &src[0], byte_length);
}

bool Mesh::SaveGLTF(const String& fileName, bool bBinary) const
{
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);

	std::vector<Mesh> meshes;
	if (texturesDiffuse.size() > 1) {
		meshes = SplitMeshPerTextureBlob();
		for (Mesh& mesh: meshes) {
			Mesh convertedMesh;
			mesh.ConvertTexturePerVertex(convertedMesh);
			mesh.Swap(convertedMesh);
		}
	} else {
		Mesh convertedMesh;
		ConvertTexturePerVertex(convertedMesh);
		meshes.emplace_back(std::move(convertedMesh));
	}

	// create GLTF model
	tinygltf::Model gltfModel;
	tinygltf::Scene gltfScene;
	tinygltf::Mesh gltfMesh;
	tinygltf::Buffer gltfBuffer;
	gltfScene.name = "scene";
	gltfMesh.name = "mesh";

	for (size_t meshId = 0; meshId < meshes.size(); meshId++) {
		const Mesh& mesh = meshes[meshId];
		ASSERT(mesh.HasTextureCoordinatesPerVertex());
		tinygltf::Primitive gltfPrimitive;
		// setup vertices
		{
			STATIC_ASSERT(3 * sizeof(Vertex::Type) == sizeof(Vertex)); // VertexArr should be continuous
			const Box box(GetAABB());
			gltfPrimitive.attributes["POSITION"] = (int)gltfModel.accessors.size();
			tinygltf::Accessor vertexPositionAccessor;
			vertexPositionAccessor.name = "vertexPositionAccessor";
			vertexPositionAccessor.bufferView = (int)gltfModel.bufferViews.size();
			vertexPositionAccessor.type = TINYGLTF_TYPE_VEC3;
			vertexPositionAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
			vertexPositionAccessor.count = mesh.vertices.size();
			vertexPositionAccessor.minValues = {box.ptMin.x(), box.ptMin.y(), box.ptMin.z()};
			vertexPositionAccessor.maxValues = {box.ptMax.x(), box.ptMax.y(), box.ptMax.z()};
			gltfModel.accessors.emplace_back(std::move(vertexPositionAccessor));
			// setup vertices buffer
			tinygltf::BufferView vertexPositionBufferView;
			vertexPositionBufferView.name = "vertexPositionBufferView";
			vertexPositionBufferView.buffer = (int)gltfModel.buffers.size();
			ExtendBufferGLTF(mesh.vertices.data(), mesh.vertices.size(), gltfBuffer,
				vertexPositionBufferView.byteOffset, vertexPositionBufferView.byteLength);
			gltfModel.bufferViews.emplace_back(std::move(vertexPositionBufferView));
		}

		// setup faces
		{
			STATIC_ASSERT(3 * sizeof(Face::Type) == sizeof(Face)); // FaceArr should be continuous
			gltfPrimitive.indices = (int)gltfModel.accessors.size();
			tinygltf::Accessor triangleAccessor;
			triangleAccessor.name = "triangleAccessor";
			triangleAccessor.bufferView = (int)gltfModel.bufferViews.size();
			triangleAccessor.type = TINYGLTF_TYPE_SCALAR;
			triangleAccessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
			triangleAccessor.count = mesh.faces.size() * 3;
			gltfModel.accessors.emplace_back(std::move(triangleAccessor));
			// setup triangles buffer
			tinygltf::BufferView triangleBufferView;
			triangleBufferView.name = "triangleBufferView";
			triangleBufferView.buffer = (int)gltfModel.buffers.size();
			ExtendBufferGLTF(mesh.faces.data(), mesh.faces.size(), gltfBuffer,
				triangleBufferView.byteOffset, triangleBufferView.byteLength);
			gltfModel.bufferViews.emplace_back(std::move(triangleBufferView));
			gltfPrimitive.mode = TINYGLTF_MODE_TRIANGLES;
		}

		// setup material
		gltfPrimitive.material = (int)gltfModel.materials.size();
		tinygltf::Material gltfMaterial;
		gltfMaterial.name = "material";
		gltfMaterial.doubleSided = true;
		if (mesh.HasTexture()) {
			// setup texture
			gltfMaterial.emissiveFactor = std::vector<double>{0,0,0};
			gltfMaterial.pbrMetallicRoughness.baseColorTexture.index = (int)gltfModel.textures.size();
			gltfMaterial.pbrMetallicRoughness.baseColorTexture.texCoord = 0;
			gltfMaterial.pbrMetallicRoughness.baseColorFactor = std::vector<double>{1,1,1,1};
			gltfMaterial.pbrMetallicRoughness.metallicFactor = 0;
			gltfMaterial.pbrMetallicRoughness.roughnessFactor = 1;
			gltfMaterial.extensions = {{"KHR_materials_unlit", {}}};
			gltfModel.extensionsUsed = {"KHR_materials_unlit"};
			// setup texture coordinates accessor
			gltfPrimitive.attributes["TEXCOORD_0"] = (int)gltfModel.accessors.size();
			tinygltf::Accessor vertexTexcoordAccessor;
			vertexTexcoordAccessor.name = "vertexTexcoordAccessor";
			vertexTexcoordAccessor.bufferView = (int)gltfModel.bufferViews.size();
			vertexTexcoordAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
			vertexTexcoordAccessor.count = mesh.faceTexcoords.size();
			vertexTexcoordAccessor.type = TINYGLTF_TYPE_VEC2;
			gltfModel.accessors.emplace_back(std::move(vertexTexcoordAccessor));
			// setup texture coordinates
			STATIC_ASSERT(2 * sizeof(TexCoord::Type) == sizeof(TexCoord)); // TexCoordArr should be continuous
			ASSERT(mesh.vertices.size() == mesh.faceTexcoords.size());
			tinygltf::BufferView vertexTexcoordBufferView;
			vertexTexcoordBufferView.name = "vertexTexcoordBufferView";
			vertexTexcoordBufferView.buffer = (int)gltfModel.buffers.size();
			TexCoordArr normFaceTexcoords;
			mesh.FaceTexcoordsNormalize(normFaceTexcoords, false);
			ExtendBufferGLTF(normFaceTexcoords.data(), normFaceTexcoords.size(), gltfBuffer,
				vertexTexcoordBufferView.byteOffset, vertexTexcoordBufferView.byteLength);
			gltfModel.bufferViews.emplace_back(std::move(vertexTexcoordBufferView));
			// setup texture
			tinygltf::Texture texture;
			texture.name = "texture";
			texture.source = (int)gltfModel.images.size();
			texture.sampler = (int)gltfModel.samplers.size();
			gltfModel.textures.emplace_back(std::move(texture));
			// setup texture image
			tinygltf::Image image;
			image.name = Util::getFileFullName(fileName) + "_" + std::to_string(meshId).c_str();
			image.width = mesh.texturesDiffuse[0].cols;
			image.height = mesh.texturesDiffuse[0].rows;
			image.component = 3;
			image.bits = 8;
			image.pixel_type = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
			image.mimeType = "image/png";
			image.image.resize(mesh.texturesDiffuse[0].size().area() * 3);
			mesh.texturesDiffuse[0].copyTo(cv::Mat(mesh.texturesDiffuse[0].size(), CV_8UC3, image.image.data()));
			gltfModel.images.emplace_back(std::move(image));
			// setup texture sampler
			tinygltf::Sampler sampler;
			sampler.name = "sampler";
			sampler.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
			sampler.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
			sampler.wrapS = TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE;
			sampler.wrapT = TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE;
			gltfModel.samplers.emplace_back(std::move(sampler));
		}
		gltfModel.materials.emplace_back(std::move(gltfMaterial));
		gltfModel.buffers.emplace_back(std::move(gltfBuffer));
		gltfMesh.primitives.emplace_back(std::move(gltfPrimitive));
	}

	// setup scene node
	gltfScene.nodes.emplace_back((int)gltfModel.nodes.size());
	tinygltf::Node node;
	node.name = "node";
	node.mesh = (int)gltfModel.meshes.size();
	gltfModel.nodes.emplace_back(std::move(node));
	gltfModel.meshes.emplace_back(std::move(gltfMesh));
	gltfModel.scenes.emplace_back(std::move(gltfScene));
	gltfModel.asset.generator = "OpenMVS";
	gltfModel.asset.version = "2.0";
	gltfModel.defaultScene = 0;

	// setup GLTF
	struct Tools {
		static bool WriteImageData(const std::string *basepath, const std::string *filename,
			tinygltf::Image *image, bool embedImages, void *) {
			ASSERT(!embedImages);
			image->uri = Util::isFullPath(filename->c_str()) ?
				Util::getRelativePath(*basepath, *filename) : String(*filename);
			String basePath(*basepath);
			return cv::imwrite(
				Util::ensureFolderSlash(basePath) + image->uri,
				cv::Mat(image->height, image->width, CV_8UC3, image->image.data()));
		}
	};
	tinygltf::TinyGLTF gltf;
	gltf.SetImageWriter(Tools::WriteImageData, NULL);
	const bool bEmbedImages(false), bEmbedBuffers(true), bPrettyPrint(true);
	return gltf.WriteGltfSceneToFile(&gltfModel, fileName, bEmbedImages, bEmbedBuffers, bPrettyPrint, bBinary);
} // Save
/*----------------------------------------------------------------*/

bool Mesh::Save(const FacesChunkArr& chunks, const String& fileName, const cList<String>& comments, bool bBinary) const
{
	if (chunks.size() < 2)
		return Save(fileName, comments, bBinary);
	FOREACH(i, chunks) {
		const Mesh mesh(SubMesh(chunks[i].faces));
		if (!mesh.Save(Util::insertBeforeFileExt(fileName, String::FormatString("_chunk%02u", i)), comments, bBinary))
			return false;
	}
	return true;
}

bool Mesh::Save(const VertexArr& vertices, const String& fileName, bool bBinary)
{
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);

	// create PLY object
	using namespace MeshInternal;
	PLY ply;
	if (!ply.write(fileName, 1, BasicPLY::elem_names, bBinary?PLY::BINARY_LE:PLY::ASCII)) {
		DEBUG_EXTRA("error: can not create the mesh file");
		return false;
	}

	// describe what properties go into vertex and face elements
	BasicPLY::Vertex::InitSaveProps(ply, (int)vertices.size(), false);
	if (!ply.header_complete())
		return false;

	// export the array of vertices
	FOREACHPTR(pVert, vertices)
		ply.put_element(pVert);
	ASSERT(ply.get_current_element_count() == (int)vertices.size());

	return true;
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

// mode : 0 - tangential; 1 - across the normal
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
	// 1 - tangential;
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
		ASSERT(bigEdges.empty());
		for (Halfedge_iterator h = p.edges_begin(); h != p.edges_end(); ++h, ++h) {
			ASSERT(++Halfedge_iterator(h) == h->opposite());
			const double edgeSize(edge_size(h));
			if (edgeSize > epsilonMax)
				bigEdges.emplace_back(h, (float)edgeSize);
		}
		DEBUG_LEVEL(3, "Big edges: %u", bigEdges.size());
		bigEdges.Sort(); // process big edges first
		for (EdgeScoreArr::IDX i=0; i<bigEdges.size(); ++i) {
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
		B.begin_surface(vertices.size(), faces.size());
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
	ASSERT(vertexFaces.size() == vertices.size());

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
	FacetSplitMap mapSplits; mapSplits.reserve(faces.size());
	FacetCountMap mapFaces; mapFaces.reserve(12*3);
	vertices.Reserve(vertices.size()*2);
	faces.Reserve(faces.size()*3);
	const uint32_t maxAreaTh(2*maxArea);
	FOREACH(f, maxAreas) {
		const AreaArr::Type area(maxAreas[f]);
		if (area <= maxAreaTh)
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
			split.idxVert[i] = newface[i] = vertices.size();
			vertices.emplace_back((vertices[face[(i+1)%3]]+vertices[face[(i+2)%3]])*0.5f);
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
			if (fc.first < f && maxAreas[fc.first] > maxAreaTh) {
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
	ASSERT(faces.size()-(faces.capacity()/3)/*initial size*/ > mapSplits.size());
	for (const auto& s: mapSplits)
		faces.RemoveAt(s.first);
}
/*----------------------------------------------------------------*/

// decimate mesh by removing the given list of vertices
//#define DECIMATE_JOINHOLES // not finished
void Mesh::Decimate(VertexIdxArr& verticesRemove)
{
	ASSERT(vertices.size() == vertexFaces.size());
	FaceIdxArr facesRemove(0, verticesRemove.size()*8);
	#ifdef DECIMATE_JOINHOLES
	cList<VertexIdxArr> holes;
	#endif
	FOREACHPTR(pIdxV, verticesRemove) {
		const VIndex idxV(*pIdxV);
		ASSERT(idxV < vertices.size());
		// create the list of consecutive vertices around selected vertex
		VertexIdxArr verts;
		{
			FaceIdxArr& vf(vertexFaces[idxV]);
			if (vf.empty())
				continue;
			const FIndex n(vf.size());
			facesRemove.Join(vf);
			ASSERT(verts.empty());
			{
				// add vertices of the first face
				const Face& f = faces[vf.front()];
				const uint32_t i(FindVertex(f, idxV));
				verts.Insert(f[(i+1)%3]);
				verts.Insert(f[(i+2)%3]);
				vf.RemoveAt(0);
			}
			while (verts.size() < n) {
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
					ASSERT(verts.front() != idxVN);
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
			while (!vf.empty()) {
				// find the face that contains our vertex and the first added vertex
				const VIndex idxVF(verts.front());
				FOREACH(idxF, vf) {
					const Face& f = faces[vf[idxF]];
					ASSERT(FindVertex(f, idxV) != NO_ID);
					const uint32_t i(FindVertex(f, idxVF));
					if (i == NO_ID)
						continue;
					// add the missing vertex at the beginning
					ASSERT(f[(i+1)%3] == idxV);
					const FIndex idxVP(f[(i+2)%3]);
					ASSERT(verts.Last() != idxVP || vf.size() == 1);
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
			const VIndex idxS((idxVH+hole.size()-1)%hole.size());
			const VIndex idxL(verts.Find(hole[idxS]));
			ASSERT(idxL != VertexIdxArr::NO_INDEX);
			ASSERT(verts[(idxL+verts.size()-1)%verts.size()] == hole[(idxS+1)%hole.size()]);
			const VIndex n(verts.size()-2);
			for (VIndex v=1; v<=n; ++v)
				hole.InsertAt(idxS+v, verts[(idxL+v)%verts.size()]);
			goto NEXT_VERTEX;
		}
		// or create a new hole
		if (verts.size() < 3)
			continue;
		verts.Swap(holes.AddEmpty());
		#else
		// close the holes defined by the complete loop of consecutive vertices
		// (the loop can be opened, cause some of the vertices can be on the border)
		if (verts.size() > 2)
			CloseHoleQuality(verts);
		#endif
		NEXT_VERTEX:;
	}
	#ifndef _RELEASE
	// check all removed vertices are completely disconnected from the mesh
	FOREACHPTR(pIdxV, verticesRemove)
		ASSERT(vertexFaces[*pIdxV].empty());
	#endif

	// remove deleted faces
	RemoveFaces(facesRemove, true);

	// remove deleted vertices
	RemoveVertices(verticesRemove);

	#ifdef DECIMATE_JOINHOLES
	// close the holes defined by the complete loop of consecutive vertices
	// (the loop can be opened, cause some of the vertices can be on the border)
	FOREACHPTR(pHole, holes) {
		ASSERT(pHole->size() > 2);
		CloseHoleQuality(*pHole);
	}
	#endif

	#ifndef _RELEASE
	// check all faces see valid vertices
	for (const Face& face: faces)
		for (int v=0; v<3; ++v)
			ASSERT(face[v] < vertices.size());
	#endif
}
/*----------------------------------------------------------------*/

// given a hole defined by a complete loop of consecutive vertices,
// split it recursively in two halves till the splits becomes a face
void Mesh::CloseHole(VertexIdxArr& split0)
{
	ASSERT(split0.size() >= 3);
	if (split0.size() == 3) {
		const FIndex idxF(faces.size());
		faces.emplace_back(split0[0], split0[1], split0[2]);
		for (int v=0; v<3; ++v) {
			#ifndef _RELEASE
			FaceIdxArr indices;
			GetAdjVertexFaces(split0[v], split0[(v+1)%3], indices);
			ASSERT(indices.size() < 2);
			indices.Empty();
			GetAdjVertexFaces(split0[v], split0[(v+2)%3], indices);
			ASSERT(indices.size() < 2);
			#endif
			vertexFaces[split0[v]].Insert(idxF);
		}
		return;
	}
	const VIndex i(split0.size() >> 1);
	const VIndex j(split0.size()-i);
	VertexIdxArr split1(0, j+1);
	split1.Join(split0.data()+i, j);
	split1.emplace_back(split0.front());
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
			angle = ACOS(ComputeAngle(mesh.vertices[face[1]].ptr(), mesh.vertices[face[0]].ptr(), mesh.vertices[face[2]].ptr()));
			if (n.dot(mesh.VertexNormal(face[1])) < 0)
				angle = float(2*M_PI) - angle;
			// compute quality as a composition of dihedral angle and area/sum(edge^2);
			// the dihedral angle uses the normal of the edge faces
			// which are possible not to exist if the edges are on the border
			FaceIdxArr indices;
			mesh.GetAdjVertexFaces(face[2], face[0], indices);
			if (indices.size() > 1) {
				aspectRatio = -1;
				return;
			}
			indices.Empty();
			mesh.GetAdjVertexFaces(face[0], face[1], indices);
			if (indices.size() > 1) {
				aspectRatio = -1;
				return;
			}
			const FIndex i0(indices.size());
			mesh.GetAdjVertexFaces(face[1], face[2], indices);
			if (indices.size()-i0 > 1) {
				aspectRatio = -1;
				return;
			}
			if (indices.empty())
				dihedral = FD2R(33.f);
			else {
				const Normal n0(mesh.FaceNormal(mesh.faces[indices[0]]));
				if (indices.size() == 1)
					dihedral = ACOS(ComputeAngle(n.ptr(), n0.ptr()));
				else {
					const Normal n1(mesh.FaceNormal(mesh.faces[indices[1]]));
					dihedral = MAXF(ACOS(ComputeAngle(n.ptr(), n0.ptr())), ACOS(ComputeAngle(n.ptr(), n1.ptr())));
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
	ASSERT(verts.size() > 2);
	cList<CandidateFace> candidateFaces(0, verts.size());
	FOREACH(v, verts) {
		if (candidateFaces.emplace_back(verts[v], verts[(v+1)%verts.size()], verts[(v+2)%verts.size()], *this).aspectRatio < 0)
			candidateFaces.RemoveLast();
	}
	candidateFaces.Sort();

	// add new faces until there are only two vertices left
	while(true) {
		// add the best candidate face
		ASSERT(!candidateFaces.empty());
		const Face& candidateFace = candidateFaces.Last();
		ASSERT(verts.Find(candidateFace[0]) != VertexIdxArr::NO_INDEX);
		ASSERT(verts.Find(candidateFace[1]) != VertexIdxArr::NO_INDEX);
		ASSERT(verts.Find(candidateFace[2]) != VertexIdxArr::NO_INDEX);
		const FIndex idxF(faces.size());
		faces.Insert(candidateFace);
		for (int v=0; v<3; ++v) {
			#ifndef _RELEASE
			FaceIdxArr indices;
			GetAdjVertexFaces(candidateFace[v], candidateFace[(v+1)%3], indices);
			ASSERT(indices.size() < 2);
			indices.Empty();
			GetAdjVertexFaces(candidateFace[v], candidateFace[(v+2)%3], indices);
			ASSERT(indices.size() < 2);
			#endif
			vertexFaces[candidateFace[v]].Insert(idxF);
		}
		if (verts.size() <= 3)
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
		const VIndex idxB(idxV+verts.size());
		const VIndex idxVB2(verts[(idxB-2)%verts.size()]);
		const VIndex idxVB1(verts[(idxB-1)%verts.size()]);
		const VIndex idxVF1(verts[(idxV+1)%verts.size()]);
		const VIndex idxVF2(verts[(idxV+2)%verts.size()]);
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

// crop mesh such that none of its faces is touching or outside the given bounding-box
void Mesh::RemoveFacesOutside(const OBB3f& obb) {
	ASSERT(obb.IsValid());
	VertexIdxArr vertexRemove;
	FOREACH(i, vertices)
		if (!obb.Intersects(vertices[i]))
			vertexRemove.emplace_back(i);
	if (!vertexRemove.empty()) {
		if (vertices.size() != vertexFaces.size())
			ListIncidenteFaces();
		RemoveVertices(vertexRemove, true);
	}
}

// remove the given list of faces
void Mesh::RemoveFaces(FaceIdxArr& facesRemove, bool bUpdateLists)
{
	facesRemove.Sort();
	FIndex idxLast(FaceIdxArr::NO_INDEX);
	if (!bUpdateLists || vertexFaces.empty()) {
		RFOREACHPTR(pIdxF, facesRemove) {
			const FIndex idxF(*pIdxF);
			if (idxLast == idxF)
				continue;
			faces.RemoveAt(idxF);
			if (!faceTexcoords.empty())
				faceTexcoords.RemoveAt(idxF * 3, 3);
			idxLast = idxF;
		}
	} else {
		ASSERT(vertices.size() == vertexFaces.size());
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
						vf.RemoveAt(idx);
				}
			}
			const FIndex idxFM(faces.size()-1);
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
			if (!faceTexcoords.empty())
				faceTexcoords.RemoveAt(idxF * 3, 3);
			idxLast = idxF;
		}
	}
	vertexVertices.Release();
}

// remove the given list of vertices, together with all faces containing them
void Mesh::RemoveVertices(VertexIdxArr& vertexRemove, bool bUpdateLists)
{
	ASSERT(vertices.size() == vertexFaces.size());
	vertexRemove.Sort();
	VIndex idxLast(VertexIdxArr::NO_INDEX);
	if (!bUpdateLists) {
		RFOREACHPTR(pIdxV, vertexRemove) {
			const VIndex idxV(*pIdxV);
			if (idxLast == idxV)
				continue;
			const VIndex idxVM(vertices.size()-1);
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
		const VIndex idxVM(vertices.size()-1);
		if (idxV < idxVM) {
			// update all faces of the moved vertex
			const FaceIdxArr& vf(vertexFaces[idxVM]);
			FOREACHPTR(pIdxF, vf)
				GetVertex(faces[*pIdxF], idxVM) = idxV;
		}
		if (!vertexFaces.empty()) {
			facesRemove.Join(vertexFaces[idxV]);
			vertexFaces.RemoveAt(idxV);
		}
		if (!vertexVertices.empty())
			vertexVertices.RemoveAt(idxV);
		vertices.RemoveAt(idxV);
		idxLast = idxV;
	}
	if (!facesRemove.empty())
		RemoveFaces(facesRemove);
}

// remove all vertices that are not assigned to any face
// (require vertexFaces)
Mesh::VIndex Mesh::RemoveUnreferencedVertices(bool bUpdateLists)
{
	ASSERT(vertices.size() == vertexFaces.size());
	VertexIdxArr vertexRemove;
	FOREACH(idxV, vertexFaces) {
		if (vertexFaces[idxV].empty())
			vertexRemove.push_back(idxV);
	}
	if (vertexRemove.empty())
		return 0;
	RemoveVertices(vertexRemove, bUpdateLists);
	return vertexRemove.size();
}

// convert textured mesh to store texture coordinates per vertex instead of per face
void Mesh::ConvertTexturePerVertex(Mesh& mesh) const
{
	ASSERT(HasTexture());
	mesh.vertices = vertices;
	mesh.faces.resize(faces.size());
	mesh.faceTexcoords.resize(vertices.size());
	if (!faceTexindices.empty())
		mesh.faceTexindices.resize(vertices.size());

	VertexIdxArr mapVertices(vertices.size(), vertices.size()*3/2);
	mapVertices.Memset(0xff);
	FOREACH(idxF, faces) {
		// face vertices inside a patch are simply copied;
		// face vertices on the patch boundary are duplicated,
		// with the same position, but different texture coordinates
		const Face& face = faces[idxF];
		Face& newface = mesh.faces[idxF];
		const TexIndex ti = !faceTexindices.empty() ? faceTexindices[idxF] : 0;
		for (int i=0; i<3; ++i) {
			const TexCoord& tc = faceTexcoords[idxF*3+i];
			VIndex idxV(face[i]);
			while (true) {
				VIndex& idxVT = mapVertices[idxV];
				if (idxVT == NO_ID) {
					// vertex seen for the first time, so just copy it
					mesh.faceTexcoords[newface[i] = idxVT = idxV] = tc;
					if (!faceTexindices.empty())
						mesh.faceTexindices[newface[i] = idxVT = idxV] = ti;
					break;
				}
				// vertex already seen in an other face, check the texture coordinates
				if (mesh.faceTexcoords[idxV] == tc) {
					// texture coordinates equal, patch interior vertex, link to it
					newface[i] = idxV;
					break;
				}
				if (idxVT == idxV) {
					// duplicate vertex, copy position, but update its texture coordinates
					mapVertices.emplace_back(newface[i] = idxVT = mesh.vertices.size());
					mesh.vertices.emplace_back(vertices[face[i]]);
					mesh.faceTexcoords.emplace_back(tc);
					if (!faceTexindices.empty())
						mesh.faceTexindices.emplace_back(ti);
					break;
				}
				// continue with the next linked vertex which share the position,
				// but use different texture coordinates
				idxV = idxVT;
			}
		}
	}
	mesh.texturesDiffuse = texturesDiffuse;
} // ConvertTexturePerVertex
/*----------------------------------------------------------------*/


// estimate the ground-plane as the plane agreeing with most vertices
//  - sampleMesh: uniformly samples points on the mesh (0 - disabled, <0 - number of points, >0 - sample density per square unit)
//  - planeThreshold: threshold used to estimate the ground plane (0 - auto)
Planef Mesh::EstimateGroundPlane(const ImageArr& images, float sampleMesh, float planeThreshold, const String& fileExportPlane) const
{
	ASSERT(!IsEmpty());
	PointCloud pointcloud;
	if (sampleMesh != 0) {
		// create the point cloud by sampling the mesh
		if (sampleMesh > 0)
			SamplePoints(sampleMesh, 0, pointcloud);
		else
			SamplePoints(ROUND2INT<unsigned>(-sampleMesh), pointcloud);
	} else {
		// create the point cloud containing all vertices
		for (const Vertex& X: vertices)
			pointcloud.points.emplace_back(X);
	}
	return pointcloud.EstimateGroundPlane(images, planeThreshold, fileExportPlane);
}
/*----------------------------------------------------------------*/


// computes the centroid of the given mesh face
Mesh::Vertex Mesh::ComputeCentroid(FIndex idxFace) const
{
	const Face& face = faces[idxFace];
	return (vertices[face[0]] + vertices[face[1]] + vertices[face[2]]) * (Type(1)/Type(3));
}

// computes the area of the given mesh face
Mesh::Type Mesh::ComputeArea(FIndex idxFace) const
{
	const Face& face = faces[idxFace];
	return ComputeTriangleArea(vertices[face[0]], vertices[face[1]], vertices[face[2]]);
}

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
	const unsigned theoreticNumberOfPoints(CEIL2INT<unsigned>(area * samplingDensity));
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
				const TexIndex& TI = faceTexindices[idxFace];
				const TexCoord xt(TO + static_cast<TexCoord::Type>(x)*(TA - TO) + static_cast<TexCoord::Type>(y)*(TB - TO));
				pointcloud.colors.emplace_back(texturesDiffuse[TI].sampleSafe(xt));
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
	ASSERT(!faceTexcoords.empty() && !texturesDiffuse.empty());
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
		void Raster(const ImageRef& pt, const Point3f& bary) {
			const Point3f pbary(PerspectiveCorrectBarycentricCoordinates(bary));
			const Depth z(ComputeDepth(pbary));
			ASSERT(z > Depth(0));
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				xt  = mesh.faceTexcoords[idxFaceTex+0] * pbary[0];
				xt += mesh.faceTexcoords[idxFaceTex+1] * pbary[1];
				xt += mesh.faceTexcoords[idxFaceTex+2] * pbary[2];
				const auto texIdx = mesh.faceTexindices[idxFaceTex / 3];
				image(pt) = mesh.texturesDiffuse[texIdx].sampleSafe(xt);
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
// project mesh to the given camera plane, computing also the normal-map (in camera space)
void Mesh::Project(const Camera& camera, DepthMap& depthMap, NormalMap& normalMap) const
{
	ASSERT(vertexNormals.size() == vertices.size());
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		const Mesh& mesh;
		NormalMap& normalMap;
		const Face::Type* idxVerts;
		const Matrix3x3f R;
		RasterMesh(const Mesh& _mesh, const Camera& _camera, DepthMap& _depthMap, NormalMap& _normalMap)
			: Base(_mesh.vertices, _camera, _depthMap), mesh(_mesh), normalMap(_normalMap), R(camera.R) {}
		inline void Clear() {
			Base::Clear();
			normalMap.memset(0);
		}
		inline void Project(const Face& facet) {
			idxVerts = facet.ptr();
			Base::Project(facet);
		}
		void Raster(const ImageRef& pt, const Point3f& bary) {
			const Point3f pbary(PerspectiveCorrectBarycentricCoordinates(bary));
			const Depth z(ComputeDepth(pbary));
			ASSERT(z > Depth(0));
			Depth& depth = depthMap(pt);
			if (depth == Depth(0) || depth > z) {
				depth = z;
				normalMap(pt) = R * normalized(
					mesh.vertexNormals[idxVerts[0]] * pbary[0]+
					mesh.vertexNormals[idxVerts[1]] * pbary[1]+
					mesh.vertexNormals[idxVerts[2]] * pbary[2]
				);
			}
		}
	};
	if (normalMap.size() != depthMap.size())
		normalMap.create(depthMap.size());
	RasterMesh rasterer(*this, camera, depthMap, normalMap);
	rasterer.Clear();
	// render the entire mesh
	for (const Face& facet: faces)
		rasterer.Project(facet);
}
// project mesh to the given camera plane using orthographic projection
void Mesh::ProjectOrtho(const Camera& camera, DepthMap& depthMap) const
{
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		RasterMesh(const VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap)
			: Base(_vertices, _camera, _depthMap) {}
		inline bool ProjectVertex(const Mesh::Vertex& pt, int v) {
			return (ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt))).z > 0 &&
				depthMap.isInsideWithBorder<float,3>(pti[v] = camera.TransformPointOrthoC2I(ptc[v]));
		}
		void Raster(const ImageRef& pt, const Point3f& bary) {
			const Depth z(ComputeDepth(bary));
			ASSERT(z > Depth(0));
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
	ASSERT(!faceTexcoords.empty() && !texturesDiffuse.empty());
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
			return (ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt))).z > 0 &&
				depthMap.isInsideWithBorder<float,3>(pti[v] = camera.TransformPointOrthoC2I(ptc[v]));
		}
		void Raster(const ImageRef& pt, const Point3f& bary) {
			const Depth z(ComputeDepth(bary));
			ASSERT(z > Depth(0));
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				xt  = mesh.faceTexcoords[idxFaceTex+0] * bary[0];
				xt += mesh.faceTexcoords[idxFaceTex+1] * bary[1];
				xt += mesh.faceTexcoords[idxFaceTex+2] * bary[2];
				auto texIdx = mesh.faceTexindices[idxFaceTex / 3];
				image(pt) = mesh.texturesDiffuse[texIdx].sampleSafe(xt);
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
	ASSERT(!IsEmpty() && !texturesDiffuse.empty());
	// initialize camera
	const AABB3f box(vertices.data(), vertices.size());
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


// split mesh into sub-meshes such that each has maxArea
bool Mesh::Split(FacesChunkArr& chunks, float maxArea)
{
	TD_TIMER_STARTD();
	Octree octree;
	FacesInserter::CreateOctree(octree, *this);
	FloatArr areas(faces.size());
	FOREACH(i, faces)
		areas[i] = ComputeArea(i);
	struct AreaInserter {
		const FloatArr& areas;
		float area;
		inline void operator() (const Octree::IDX_TYPE* indices, Octree::SIZE_TYPE size) {
			FOREACHRAWPTR(pIdx, indices, size)
				area += areas[*pIdx];
		}
		inline float PopArea() {
			const float a(area);
			area = 0;
			return a;
		}
	} areaEstimator{areas, 0.f};
	struct ChunkInserter {
		const Octree& octree;
		FacesChunkArr& chunks;
		void operator() (const Octree::CELL_TYPE& parentCell, Octree::Type parentRadius, const UnsignedArr& children) {
			ASSERT(!children.empty());
			FaceChunk& chunk = chunks.AddEmpty();
			struct Inserter {
				FaceIdxArr& faces;
				inline void operator() (const Octree::IDX_TYPE* indices, Octree::SIZE_TYPE size) {
					faces.Join(indices, size);
				}
			} inserter{chunk.faces};
			if (children.size() == 1) {
				octree.CollectCells(parentCell.GetChild(children.front()), inserter);
				chunk.box = parentCell.GetChildAabb(children.front(), parentRadius);
			} else {
				chunk.box.Reset();
				for (unsigned c: children) {
					octree.CollectCells(parentCell.GetChild(c), inserter);
					chunk.box.Insert(parentCell.GetChildAabb(c, parentRadius));
				}
			}
			if (chunk.faces.empty())
				chunks.RemoveLast();
		}
	} chunkInserter{octree, chunks};
	octree.SplitVolume(maxArea, areaEstimator, chunkInserter);
	if (chunks.size() < 2)
		return false;
	DEBUG_EXTRA("Mesh split (%g max-area): %u chunks (%s)", maxArea, chunks.size(), TD_TIMER_GET_FMT().c_str());
	return true;
} // Split
/*----------------------------------------------------------------*/

// extract the sub-mesh corresponding to the given chunk of faces
Mesh Mesh::SubMesh(const FaceIdxArr& chunk) const
{
	ASSERT(!chunk.empty());
	Mesh mesh;
	mesh.vertices = vertices;
	mesh.faces.reserve(chunk.size());
	if (!faceTexcoords.empty())
		mesh.faceTexcoords.reserve(chunk.size()*3);
	for (FIndex idxFace: chunk) {
		mesh.faces.emplace_back(faces[idxFace]);
		if (!faceTexcoords.empty()) {
			const TexCoord* tri = faceTexcoords.data()+idxFace*3;
			for (int i = 0; i < 3; ++i)
				mesh.faceTexcoords.emplace_back(tri[i]);
		}
	}
	mesh.ListIncidenteFaces();
	mesh.RemoveUnreferencedVertices();
	mesh.FixNonManifold();
	return mesh;
} // SubMesh
/*----------------------------------------------------------------*/

// extract one sub-mesh for each texture, i.e. for each value of faceTexindices;
std::vector<Mesh> Mesh::SplitMeshPerTextureBlob() const {

	ASSERT(HasTexture());
	if (texturesDiffuse.size() == 1)
		return {*this};
	ASSERT(faceTexindices.size() == faces.size());
	std::vector<Mesh> submeshes;
	submeshes.reserve(texturesDiffuse.size());
	FOREACH(texId, texturesDiffuse) {
		FaceIdxArr chunk;
		FOREACH(idxFace, faceTexindices) {
			if (faceTexindices[idxFace] == texId)
				chunk.push_back(idxFace);
		}
		Mesh submesh = SubMesh(chunk);
		submesh.texturesDiffuse.emplace_back(texturesDiffuse[texId]);
		submeshes.emplace_back(std::move(submesh));
	}
	return submeshes;
}


// transfer the texture of this mesh to the new mesh;
// the two meshes should be aligned and the new mesh to have UV-coordinates
#if USE_MESH_INT == USE_MESH_BVH
struct FaceBox {
	Eigen::AlignedBox3f box;
	Mesh::FIndex idxFace;
};
inline Eigen::AlignedBox3f bounding_box(const FaceBox& faceBox) {
	return faceBox.box;
}
#endif
bool Mesh::TransferTexture(Mesh& mesh, const FaceIdxArr& faceSubsetIndices, unsigned borderSize, unsigned textureSize)
{
	ASSERT(HasTexture() && mesh.HasTextureCoordinates());
	if (mesh.texturesDiffuse.empty()) {
		// create the texture at specified resolution and
		// scale the UV-coordinates to the new resolution (assuming normalized coordinates)
		mesh.texturesDiffuse.emplace_back(textureSize, textureSize).memset(0);
		for (TexCoord& tex: mesh.faceTexcoords) {
			ASSERT(tex.x <= 1 && tex.y <= 1);
			tex *= (Mesh::Type)textureSize;
		}
	}
	Image8U mask(mesh.texturesDiffuse.back().size(), uint8_t(255));
	const FIndex num_faces(faceSubsetIndices.empty() ? mesh.faces.size() : faceSubsetIndices.size());
	if (vertices == mesh.vertices && faces == mesh.faces) {
		// the two meshes are identical, only the texture coordinates are different;
		// directly transfer the texture onto the new coordinates
		#ifdef MESH_USE_OPENMP
		#pragma omp parallel for schedule(dynamic)
		for (int_t i=0; i<(int_t)num_faces; ++i) {
			const FIndex idx((FIndex)i);
		#else
		FOREACHRAW(idx, num_faces) {
		#endif
			const FIndex idxFace(faceSubsetIndices.empty() ? idx : faceSubsetIndices[idx]);
			struct RasterTriangle {
				const Mesh& meshRef;
				Mesh& meshTrg;
				Image8U& mask;
				const TexCoord* tri;
				const TexIndex texId;
				inline cv::Size Size() const { return meshTrg.texturesDiffuse[0].size(); }
				inline void operator()(const ImageRef& pt, const Point3f& bary) {
					ASSERT(meshTrg.texturesDiffuse[texId].isInside(pt));
					const TexCoord x(tri[0]*bary.x + tri[1]*bary.y + tri[2]*bary.z);
					const Pixel8U color(meshRef.texturesDiffuse[texId].sample(x));
					meshTrg.texturesDiffuse[texId](pt) = color;
					mask(pt) = 0;
				}
			} data{*this, mesh, mask, faceTexcoords.data()+idxFace*3, mesh.faceTexindices[idxFace]};
			// render triangle and for each pixel interpolate the color
			// from the triangle corners using barycentric coordinates
			const TexCoord* tri = mesh.faceTexcoords.data()+idxFace*3;
			Image8U::RasterizeTriangleBary<TexCoord::Type,RasterTriangle,false>(tri[0], tri[1], tri[2], data);
		}
	} else {
		// the two meshes are different, transfer the texture by finding the closest point
		// on the two surfaces
		if (vertexFaces.size() != vertices.size())
			ListIncidenteFaces();
		if (mesh.vertexNormals.size() != mesh.vertices.size())
			mesh.ComputeNormalVertices();
		#if USE_MESH_INT == USE_MESH_BVH
		std::vector<FaceBox> boxes;
		boxes.reserve(faces.size());
		FOREACH(idxFace, faces)
			boxes.emplace_back([this](FIndex idxFace) {
				const Face& face = faces[idxFace];
				Eigen::AlignedBox3f box;
				box.extend<Eigen::Vector3f>(vertices[face[0]]);
				box.extend<Eigen::Vector3f>(vertices[face[1]]);
				box.extend<Eigen::Vector3f>(vertices[face[2]]);
				return FaceBox{box, idxFace};
			} (idxFace));
		typedef Eigen::KdBVH<Type,3,FaceBox> BVH;
		BVH tree(boxes.begin(), boxes.end());
		#endif
		struct IntersectRayMesh {
			const Mesh& mesh;
			const Ray3f& ray;
			IndexDist pick;
			IntersectRayMesh(const Mesh& _mesh, const Ray3f& _ray)
				: mesh(_mesh), ray(_ray) {
				#if USE_MESH_INT == USE_MESH_BF
				FOREACH(idxFace, mesh.faces)
					IntersectsRayFace(idxFace);
				#endif
			}
			inline void IntersectsRayFace(FIndex idxFace) {
				const Face& face = mesh.faces[idxFace];
				Type dist;
				if (ray.Intersects<false>(Triangle3f(
					mesh.vertices[face.x], mesh.vertices[face.y], mesh.vertices[face.z]), &dist)) {
					if (pick.dist > ABS(dist)) {
						pick.dist = ABS(dist);
						pick.idx = idxFace;
					}
				}
			}
			#if USE_MESH_INT == USE_MESH_BVH
			inline bool intersectVolume(const BVH::Volume &volume) {
				return ray.Intersects(AABB3f(volume.min(), volume.max()));
			}
			inline bool intersectObject(const BVH::Object &object) {
				IntersectsRayFace(object.idxFace);
				return false;
			}
			#endif
		};
		#if USE_MESH_INT == USE_MESH_BF || USE_MESH_INT == USE_MESH_BVH
		#elif USE_MESH_INT == USE_MESH_OCTREE
		const Octree octree(vertices, [](Octree::IDX_TYPE size, Octree::Type /*radius*/) {
			return size > 8;
		});
		struct OctreeIntersectRayMesh : IntersectRayMesh {
			OctreeIntersectRayMesh(const Octree& octree, const Mesh& _mesh, const Ray3f& _ray)
				: IntersectRayMesh(_mesh, _ray) {
				octree.Collect(*this, *this);
			}
			inline bool Intersects(const Octree::POINT_TYPE& center, Octree::Type radius) const {
				return ray.Intersects(AABB3f(center, radius));
			}
			void operator() (const Octree::IDX_TYPE* idices, Octree::IDX_TYPE size) {
				// store all contained faces only once
				std::unordered_set<FIndex> set;
				FOREACHRAWPTR(pIdx, idices, size) {
					const VIndex idxVertex((VIndex)*pIdx);
					const FaceIdxArr& faces = mesh.vertexFaces[idxVertex];
					set.insert(faces.begin(), faces.end());
				}
				// test face intersection and keep the closest
				for (FIndex idxFace : set)
					IntersectsRayFace(idxFace);
			}
		};
		#endif
		#ifdef MESH_USE_OPENMP
		#pragma omp parallel for schedule(dynamic)
		for (int_t i=0; i<(int_t)num_faces; ++i) {
			const FIndex idx((FIndex)i);
		#else
		FOREACHRAW(idx, num_faces) {
		#endif
			const FIndex idxFace(faceSubsetIndices.empty() ? idx : faceSubsetIndices[idx]);
			struct RasterTriangle {
				#if USE_MESH_INT == USE_MESH_OCTREE
				const Octree& octree;
				#elif USE_MESH_INT == USE_MESH_BVH
				BVH& tree;
				#endif
				const Mesh& meshRef;
				Mesh& meshTrg;
				Image8U& mask;
				const Face& face;
				const TexIndex texId;
				inline cv::Size Size() const { return meshTrg.texturesDiffuse.back().size(); }
				inline void operator()(const ImageRef& pt, const Point3f& bary) {
					ASSERT(meshTrg.texturesDiffuse[texId].isInside(pt));
					const Vertex X(meshTrg.vertices[face.x]*bary.x
								 + meshTrg.vertices[face.y]*bary.y
								 + meshTrg.vertices[face.z]*bary.z);
					const Normal N(normalized(meshTrg.vertexNormals[face.x]*bary.x
											+ meshTrg.vertexNormals[face.y]*bary.y
											+ meshTrg.vertexNormals[face.z]*bary.z));
					const Ray3f ray(X, N);
					#if USE_MESH_INT == USE_MESH_BF
					const IntersectRayMesh intRay(meshRef, ray);
					#elif USE_MESH_INT == USE_MESH_BVH
					IntersectRayMesh intRay(meshRef, ray);
					Eigen::BVIntersect(tree, intRay);
					#else
					const OctreeIntersectRayMesh intRay(octree, meshRef, ray);
					#endif
					if (intRay.pick.IsValid()) {
						const FIndex refIdxFace((FIndex)intRay.pick.idx);
						const Face& refFace = meshRef.faces[refIdxFace];
						const Vertex refX(ray.GetPoint((Type)intRay.pick.dist));
						const Vertex baryRef(CorrectBarycentricCoordinates(BarycentricCoordinatesUV(meshRef.vertices[refFace[0]], meshRef.vertices[refFace[1]], meshRef.vertices[refFace[2]], refX)));
						const TexCoord* tri = meshRef.faceTexcoords.data()+refIdxFace*3;
						const TexCoord x(tri[0]*baryRef.x + tri[1]*baryRef.y + tri[2]*baryRef.z);
						const Pixel8U color(meshRef.texturesDiffuse[texId].sample(x));
						meshTrg.texturesDiffuse.back()(pt) = color;
						mask(pt) = 0;
					}
				}
			#if USE_MESH_INT == USE_MESH_BF
			} data{*this, mesh, mask, mesh.faces[idxFace], mesh.GetFaceTextureIndex(idxFace)};
			#elif USE_MESH_INT == USE_MESH_BVH
			} data{tree, *this, mesh, mask, mesh.faces[idxFace], mesh.GetFaceTextureIndex(idxFace)};
			#else
			} data{octree, *this, mesh, mask, mesh.faces[idxFace], mesh.GetFaceTextureIndex(idxFace)};
			#endif
			// render triangle and for each pixel interpolate the color
			// from the triangle corners using barycentric coordinates
			const TexCoord* tri = mesh.faceTexcoords.data()+idxFace*3;
			Image8U::RasterizeTriangleBary<TexCoord::Type,RasterTriangle,false>(tri[0], tri[1], tri[2], data);
		}
	}
	// fill border
	if (borderSize > 0) {
		ASSERT(mask.size().area() == mesh.texturesDiffuse[0].size().area());
		const int border(static_cast<int>(borderSize));
		CLISTDEF0(int) idx_valid_pixels;
		idx_valid_pixels.push_back(-1);
		ASSERT(mask.isContinuous());
		const int size(mask.size().area());
		for (int i=0; i<size; ++i)
			if (!mask(i))
				idx_valid_pixels.push_back(i);
		Image32F dists; cv::Mat_<int32_t> labels;
		cv::distanceTransform(mask, dists, labels, cv::DIST_L1, 3, cv::DIST_LABEL_PIXEL);
		ASSERT(mesh.texturesDiffuse[0].isContinuous());
		for (int i=0; i<size; ++i) {
			const int dist = static_cast<int>(dists(i));
			if (dist > 0 && dist <= border) {
				const int label(labels(i));
				const int idx_closest_pixel(idx_valid_pixels[label]);
				mesh.texturesDiffuse[0](i) = mesh.texturesDiffuse[0](idx_closest_pixel);
			}
		}
	}
	return true;
} // TransferTexture
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
