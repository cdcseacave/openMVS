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
// CGAL: mesh cleaning, simplification, and repair
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/manifoldness.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/tangential_relaxation.h>
#include <CGAL/Polygon_mesh_processing/angle_and_area_smoothing.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_triangle_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_stop_predicate.h>
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

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("Mesh    "));

// free all memory
void Mesh::Release()
{
	vertices.Release();
	faces.Release();
	ReleaseExtra();
} // Release
void Mesh::ReleaseExtra()
{
	ReleaseComputable();
	vertexNormals.Release();
	faceNormals.Release();
	faceTexcoords.Release();
	texturesDiffuse.Release();
} // ReleaseExtra
void Mesh::ReleaseComputable()
{
	vertexVertices.Release();
	vertexFaces.Release();
	vertexBoundary.Release();
	faceFaces.Release();
} // ReleaseComputable
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
Mesh& Mesh::Swap(Mesh& rhs)
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
	return *this;
} // Swap
// combine this mesh with the given mesh, without removing duplicate vertices
Mesh& Mesh::Join(const Mesh& mesh)
{
	ASSERT(!HasTexture() && !mesh.HasTexture());
	if (mesh.IsEmpty())
		return *this;
	vertexVertices.Release();
	vertexFaces.Release();
	vertexBoundary.Release();
	faceFaces.Release();
	if (IsEmpty()) {
		*this = mesh;
		return *this;
	}
	const VIndex offsetV(vertices.size());
	vertices.Join(mesh.vertices);
	vertexNormals.Join(mesh.vertexNormals);
	faces.ReserveExtra(mesh.faces.size());
	for (const Face& face: mesh.faces)
		faces.emplace_back(face.x+offsetV, face.y+offsetV, face.z+offsetV);
	faceNormals.Join(mesh.faceNormals);
	return *this;
}
/*----------------------------------------------------------------*/


bool Mesh::IsWatertight()
{
	if (vertexBoundary.empty()) {
		if (vertexFaces.empty())
			ListIncidentFaces();
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
// compute the axis-aligned bounding-box of the mesh
// considering only vertices within the given percentile range per axis
Mesh::Box Mesh::GetAABB(float minPercentile, float maxPercentile) const
{
	// get percentile bounds for each axis
	const Box percentileBounds(GetPercentileAABB(minPercentile, maxPercentile));
	// compute AABB from vertices within percentile bounds
	return GetAABB(percentileBounds);
}
// compute the percentile axis-aligned bounding-box of the mesh
// considering only vertices within the given percentile range per axis
Mesh::Box Mesh::GetPercentileAABB(float minPercentile, float maxPercentile) const
{
	ASSERT(minPercentile >= 0.f && minPercentile <= 1.f);
	ASSERT(maxPercentile >= 0.f && maxPercentile <= 1.f);
	ASSERT(minPercentile < maxPercentile);
	// collect points per axis
	typedef CLISTDEF0IDX(Type,VIndex) Scalars;
	Scalars x, y, z;
	x.reserve(vertices.size());
	y.reserve(vertices.size());
	z.reserve(vertices.size());
	for (const Vertex& X: vertices) {
		x.push_back(X.x);
		y.push_back(X.y);
		z.push_back(X.z);
	}
	if (x.empty())
		return Box(true);
	// compute percentile indices
	x.Sort();
	y.Sort();
	z.Sort();
	const float numPoints(x.size() - 1);
	const VIndex idxMin(MAXF(VIndex(0), ROUND2INT<VIndex>(minPercentile * numPoints)));
	const VIndex idxMax(MINF(static_cast<VIndex>(numPoints), ROUND2INT<VIndex>(maxPercentile * numPoints)));
	// return percentile bounds for each axis
	return Box(
		Box::POINT(x[idxMin], y[idxMin], z[idxMin]),
		Box::POINT(x[idxMax], y[idxMax], z[idxMax]));
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
void Mesh::ListIncidentVertices()
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
void Mesh::ListIncidentFaces()
{
	vertexFaces.clear();
	vertexFaces.resize(vertices.size());
	FOREACH(iF, faces) {
		const Face& face = faces[iF];
		for (int v=0; v<3; ++v) {
			FaceIdxArr& vfs = vertexFaces[face[v]];
			ASSERT(vfs.Find(iF) == FaceIdxArr::NO_INDEX || vfs.Find(iF) == vfs.size()-1/*for degenerate faces*/);
			if (vfs.empty() || vfs.back() != iF)
				vfs.emplace_back(iF);
		}
	}
}

// extract array face adjacencies for each face in the mesh (3 * number of faces);
// each triple describes the adjacent face triangles for a given face
// in the following edge order: v1v2, v2v3, v3v1;
// NO_ID indicates there is no adjacent face on that edge
void Mesh::ListIncidentFaceFaces()
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
// (make sure you called ListIncidentFaces() before)
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
	FOREACH(idxFace, faces)
		faceNormals[idxFace] = normalized(FaceNormal(faces[idxFace]));
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
		ListIncidentFaces();
	if (faceFaces.size() != faces.size())
		ListIncidentFaceFaces();
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

bool Mesh::GetEdgeVertices(FIndex f0, FIndex f1, uint32_t* vs0, uint32_t* vs1) const
{
	const Face& face0 = faces[f0];
	const Face& face1 = faces[f1];
	int i(0);
	for (int v=0; v<3; ++v) {
		if ((vs1[i] = FindVertex(face1, face0[v])) != NO_ID) {
			vs0[i] = v;
			if (++i == 2)
				return true;
		}
	}
	return false;
}

bool Mesh::GetEdgeVertices(FIndex f0, FIndex f1, VIndex* vs) const
{
	const Face& face0 = faces[f0];
	const Face& face1 = faces[f1];
	int i(0);
	for (int v=0; v<3; ++v) {
		if (FindVertex(face1, face0[v]) != NO_ID) {
			vs[i] = face0[v];
			if (++i == 2)
				return true;
		}
	}
	return false;
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
		ListIncidentFaces();
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
	if (numNonManifoldIssues > 0) {
		vertexFaces.Release();
		DEBUG_ULTIMATE("Removed %u non-manifold issues", numNonManifoldIssues);
	}
	return numNonManifoldIssues;
}
/*----------------------------------------------------------------*/

namespace CLEAN {
typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
typedef K::Point_3                                             Point_3;
typedef CGAL::Surface_mesh<Point_3>                            SurfaceMesh;
typedef SurfaceMesh::Vertex_index                              vertex_descriptor;
typedef SurfaceMesh::Face_index                                face_descriptor;
typedef SurfaceMesh::Edge_index                                edge_descriptor;
typedef SurfaceMesh::Halfedge_index                            halfedge_descriptor;
namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;

// import/export mesh from/to CGAL surface mesh (source mesh is released after import)
static void ImportMesh(SurfaceMesh& sm, Mesh& mesh) {
	mesh.ReleaseExtra();
	// add vertices
	std::vector<vertex_descriptor> vmap(mesh.vertices.size());
	FOREACH(i, mesh.vertices) {
		const Mesh::Vertex& v = mesh.vertices[i];
		vmap[i] = sm.add_vertex(Point_3(v.x, v.y, v.z));
	}
	mesh.vertices.Release();
	// add faces (skip non-manifold faces)
	#if TD_VERBOSE != TD_VERBOSE_OFF
	int nSkipped(0);
	#endif
	FOREACH(i, mesh.faces) {
		const Mesh::Face& f = mesh.faces[i];
		const face_descriptor fd = sm.add_face(vmap[f[0]], vmap[f[1]], vmap[f[2]]);
		if (fd == SurfaceMesh::null_face()) {
			#if TD_VERBOSE != TD_VERBOSE_OFF
			++nSkipped;
			#endif
		}
	}
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (nSkipped > 0)
		DEBUG_EXTRA("warning: skipped %d non-manifold faces during import", nSkipped);
	#endif
	mesh.faces.Release();
	// fix non-manifold vertices by duplication
	PMP::duplicate_non_manifold_vertices(sm);
	// remove degenerate faces
	PMP::remove_degenerate_faces(sm);
	sm.collect_garbage();
	DEBUG_ULTIMATE("Mesh imported: %u vertices, %u faces", (unsigned)sm.number_of_vertices(), (unsigned)sm.number_of_faces());
}
static void ExportMesh(SurfaceMesh& sm, Mesh& mesh) {
	sm.collect_garbage();
	// build vertex index map (vertex descriptors are contiguous after collect_garbage)
	auto vIdxMap = sm.add_property_map<vertex_descriptor, Mesh::VIndex>("v:export_idx", 0).first;
	mesh.vertices.Resize((Mesh::VIndex)sm.number_of_vertices());
	Mesh::VIndex idx(0);
	for (vertex_descriptor vd : sm.vertices()) {
		const Point_3& pt = sm.point(vd);
		Mesh::Vertex& v = mesh.vertices[idx];
		v.x = (float)pt.x();
		v.y = (float)pt.y();
		v.z = (float)pt.z();
		vIdxMap[vd] = idx++;
	}
	// extract faces
	mesh.faces.Resize((Mesh::FIndex)sm.number_of_faces());
	Mesh::FIndex fIdx(0);
	for (face_descriptor fd : sm.faces()) {
		Mesh::Face& f = mesh.faces[fIdx++];
		halfedge_descriptor hd = sm.halfedge(fd);
		for (int i = 0; i < 3; ++i) {
			f[i] = vIdxMap[sm.target(hd)];
			hd = sm.next(hd);
		}
	}
	DEBUG_ULTIMATE("Mesh exported: %u vertices, %u faces", (unsigned)sm.number_of_vertices(), (unsigned)sm.number_of_faces());
}
} // namespace CLEAN

// decimate, clean and smooth mesh
// fDecimate factor is in range (0..1], if 1 no decimation takes place
void Mesh::Clean(float fDecimate, float fSpurious, bool bRemoveSpikes, unsigned nCloseHoles, unsigned nSmooth, float fEdgeLength, bool bLastClean)
{
	if (vertices.empty() || faces.empty())
		return;
	TD_TIMER_STARTD();
	// import mesh and fix manifoldness (once)
	CLEAN::SurfaceMesh sm;
	CLEAN::ImportMesh(sm, *this);

	// remove spurious components
	if (fSpurious > 0) {
		// compute edge squared lengths
		FloatArr edgeLens(0, (VIndex)sm.number_of_edges());
		for (CLEAN::edge_descriptor ed : sm.edges()) {
			const CLEAN::halfedge_descriptor hd = sm.halfedge(ed);
			const CLEAN::Point_3& p0 = sm.point(sm.source(hd));
			const CLEAN::Point_3& p1 = sm.point(sm.target(hd));
			const double dx = p1.x()-p0.x(), dy = p1.y()-p0.y(), dz = p1.z()-p0.z();
			edgeLens.Insert((float)(dx*dx + dy*dy + dz*dz));
		}

		// remove faces with too long edges
		const float thLongEdge(SQRT(edgeLens.GetNth(edgeLens.size()*95/100))*fSpurious);
		const float thLongEdgeSq(thLongEdge*thLongEdge);
		{
			std::vector<CLEAN::face_descriptor> longFaces;
			for (CLEAN::face_descriptor fd : sm.faces()) {
				CLEAN::halfedge_descriptor hd = sm.halfedge(fd);
				for (int i = 0; i < 3; ++i) {
					const CLEAN::Point_3& p0 = sm.point(sm.source(hd));
					const CLEAN::Point_3& p1 = sm.point(sm.target(hd));
					const double dx = p1.x()-p0.x(), dy = p1.y()-p0.y(), dz = p1.z()-p0.z();
					if (dx*dx + dy*dy + dz*dz > thLongEdgeSq) {
						longFaces.push_back(fd);
						break;
					}
					hd = sm.next(hd);
				}
			}
			for (CLEAN::face_descriptor fd : longFaces) {
				if (!sm.is_removed(fd))
					CGAL::Euler::remove_face(sm.halfedge(fd), sm);
			}
			DEBUG_ULTIMATE("Removed %u faces with edges longer than %f", (unsigned)longFaces.size(), thLongEdge);
		}

		// remove small connected components
		sm.collect_garbage();
		{
			const float thLongSize(SQRT(edgeLens.GetNth(edgeLens.size()*55/100))*fSpurious);
			auto fccmap = sm.add_property_map<CLEAN::face_descriptor, std::size_t>("f:cc", 0).first;
			const std::size_t numCC = CLEAN::PMP::connected_components(sm, fccmap);
			if (numCC > 1) {
				// compute AABB diagonal per component
				struct BBox {
					double minx, miny, minz, maxx, maxy, maxz;
					bool valid;
					BBox() : minx(DBL_MAX), miny(DBL_MAX), minz(DBL_MAX), maxx(-DBL_MAX), maxy(-DBL_MAX), maxz(-DBL_MAX), valid(false) {}
					void Update(const CLEAN::Point_3& pt) {
						valid = true;
						if (pt.x() < minx) minx = pt.x();
						if (pt.y() < miny) miny = pt.y();
						if (pt.z() < minz) minz = pt.z();
						if (pt.x() > maxx) maxx = pt.x();
						if (pt.y() > maxy) maxy = pt.y();
						if (pt.z() > maxz) maxz = pt.z();
					}
					double Diameter() const {
						if (!valid) return 0;
						const double dx = maxx-minx, dy = maxy-miny, dz = maxz-minz;
						return SQRT(dx*dx + dy*dy + dz*dz);
					}
				};
				std::vector<BBox> ccBBox(numCC);
				for (CLEAN::face_descriptor fd : sm.faces()) {
					const std::size_t cc = fccmap[fd];
					CLEAN::halfedge_descriptor hd = sm.halfedge(fd);
					for (int i = 0; i < 3; ++i) {
						ccBBox[cc].Update(sm.point(sm.target(hd)));
						hd = sm.next(hd);
					}
				}
				// count and collect faces from small components
				int numSmallCC(0);
				for (std::size_t c = 0; c < numCC; ++c) {
					if (ccBBox[c].Diameter() < thLongSize)
						++numSmallCC;
				}
				std::vector<CLEAN::face_descriptor> smallFaces;
				for (CLEAN::face_descriptor fd : sm.faces()) {
					if (ccBBox[fccmap[fd]].Diameter() < thLongSize)
						smallFaces.push_back(fd);
				}
				for (CLEAN::face_descriptor fd : smallFaces) {
					if (!sm.is_removed(fd))
						CGAL::Euler::remove_face(sm.halfedge(fd), sm);
				}
				sm.collect_garbage();
				DEBUG_ULTIMATE("Removed %d connected components out of %u", numSmallCC, (unsigned)numCC);
			}
		}
	}

	// remove spikes
	if (bRemoveSpikes) {
		int nTotalSpikes(0);
		for (int spikeIter = 0; spikeIter < 100; ++spikeIter) {
			std::vector<CLEAN::vertex_descriptor> spikeVerts;
			for (CLEAN::vertex_descriptor vd : sm.vertices()) {
				int faceCount(0);
				for (CLEAN::halfedge_descriptor hd : CGAL::halfedges_around_target(vd, sm)) {
					if (!sm.is_border(hd))
						++faceCount;
				}
				if (faceCount <= 1)
					spikeVerts.push_back(vd);
			}
			if (spikeVerts.empty())
				break;
			for (CLEAN::vertex_descriptor vd : spikeVerts) {
				if (sm.is_isolated(vd)) {
					sm.remove_vertex(vd);
					continue;
				}
				std::vector<CLEAN::halfedge_descriptor> toRemove;
				for (CLEAN::halfedge_descriptor hd : CGAL::halfedges_around_target(vd, sm)) {
					if (!sm.is_border(hd))
						toRemove.push_back(hd);
				}
				for (CLEAN::halfedge_descriptor hd : toRemove) {
					if (!sm.is_removed(sm.face(hd)))
						CGAL::Euler::remove_face(hd, sm);
				}
			}
			sm.collect_garbage();
			nTotalSpikes += (int)spikeVerts.size();
		}
		DEBUG_ULTIMATE("Removed %d spikes", nTotalSpikes);
	}

	// decimate mesh
	if (fDecimate < 1) {
		ASSERT(fDecimate > 0);
		const int OriginalFaceNum((int)sm.number_of_faces());
		const int TargetFaceNum(ROUND2INT(fDecimate * OriginalFaceNum));
		Util::Progress progress(_T("Decimated faces"), OriginalFaceNum - TargetFaceNum);
		typedef CLEAN::SMS::GarlandHeckbert_triangle_policies<CLEAN::SurfaceMesh, CLEAN::K> GH_policies;
		GH_policies gh_policies(sm);
		CLEAN::SMS::Face_count_stop_predicate<CLEAN::SurfaceMesh> stop(TargetFaceNum);
		CLEAN::SMS::edge_collapse(sm, stop,
			CGAL::parameters::get_cost(gh_policies.get_cost())
			                 .get_placement(gh_policies.get_placement()));
		sm.collect_garbage();
		progress.close();
		DEBUG_ULTIMATE("Mesh decimated: %d -> %d faces", OriginalFaceNum, (int)sm.number_of_faces());
	}

	// close holes
	if (nCloseHoles > 0) {
		std::vector<CLEAN::halfedge_descriptor> borderCycles;
		CLEAN::PMP::extract_boundary_cycles(sm, std::back_inserter(borderCycles));
		const int OriginalSize((int)sm.number_of_faces());
		int holeCnt(0);
		for (CLEAN::halfedge_descriptor hd : borderCycles) {
			// count boundary edges in this cycle
			int numEdges(0);
			CLEAN::halfedge_descriptor h = hd;
			do {
				++numEdges;
				h = sm.next(h);
			} while (h != hd);
			if (numEdges > (int)nCloseHoles)
				continue;
			// fill the hole with triangulation + refinement + fairing
			std::vector<CLEAN::face_descriptor> newFaces;
			std::vector<CLEAN::vertex_descriptor> newVertices;
			try {
				CLEAN::PMP::triangulate_refine_and_fair_hole(sm, hd,
					std::back_inserter(newFaces),
					std::back_inserter(newVertices));
				++holeCnt;
			} catch (const std::exception&) {
				// hole filling can fail for degenerate boundaries; skip
			}
		}
		DEBUG_ULTIMATE("Closed %d holes and added %d new faces", holeCnt, (int)sm.number_of_faces() - OriginalSize);
	}

	// smooth mesh
	if (nSmooth > 0) {
		#if 1
		// implicit mean curvature flow: smooths surface along normals (equivalent to VCG VertexCoordLaplacian)
		// time parameter controls intensity: 1e-4 mild, 1e-3 moderate, 1e-2 strong
		CLEAN::PMP::smooth_shape(sm.faces(), sm, 1e-3,
			CGAL::parameters::number_of_iterations(nSmooth));
		#elif 0
		// tangential relaxation: moves vertices along tangent plane (improves triangle quality, no shape change)
		CLEAN::PMP::tangential_relaxation(sm,
			CGAL::parameters::number_of_iterations(nSmooth));
		#else
		// angle and area smoothing: optimizes triangle angles/areas (requires Ceres for area component)
		CLEAN::PMP::angle_and_area_smoothing(sm.faces(), sm,
			CGAL::parameters::number_of_iterations(nSmooth)
			                 .use_safety_constraints(true));
		#endif
		DEBUG_ULTIMATE("Smoothed %u vertices", (unsigned)sm.number_of_vertices());
	}

	// remesh
	if (fEdgeLength > 0) {
		try {
			CLEAN::PMP::isotropic_remeshing(sm.faces(), (double)fEdgeLength, sm,
				CGAL::parameters::number_of_iterations(3));
		} catch (const std::exception& e) {
			VERBOSE("error: isotropic remeshing failed: %s", e.what());
		}
	}

	// final clean
	if (bLastClean && (fSpurious > 0 || bRemoveSpikes || nCloseHoles > 0 || nSmooth > 0)) {
		CLEAN::PMP::remove_degenerate_faces(sm);
		sm.collect_garbage();
	}

	// export mesh
	CLEAN::ExportMesh(sm, *this);
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
	DEBUG_EXTRA("Mesh '%s' loaded: %u vertices, %u faces (%s)",
		Util::getFileNameExt(fileName).c_str(), vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
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
		Release();
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
	STATIC_ASSERT(sizeof(ObjModel::Vertex) == sizeof(Vertex));
	ASSERT(model.get_vertices().size() < std::numeric_limits<VIndex>::max());
	vertices.CopyOf(&model.get_vertices()[0], (VIndex)model.get_vertices().size());

	// store vertex normals
	STATIC_ASSERT(sizeof(ObjModel::Normal) == sizeof(Normal));
	ASSERT(model.get_vertices().size() < std::numeric_limits<VIndex>::max());
	if (!model.get_normals().empty()) {
		ASSERT(model.get_normals().size() == model.get_vertices().size());
		vertexNormals.CopyOf(&model.get_normals()[0], (VIndex)model.get_normals().size());
	}

	// store faces
	ASSERT_ARE_SAME_TYPE(ObjModel::TexCoord, TexCoord);
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

	// check if the model contains any mesh
	bool bHasMesh = false;
	for (const tinygltf::Mesh& gltfMesh : gltfModel.meshes) {
		for (const tinygltf::Primitive& gltfPrimitive : gltfMesh.primitives) {
			if (gltfPrimitive.mode == TINYGLTF_MODE_TRIANGLES) {
				bHasMesh = true;
				break;
			}
		}
		if (bHasMesh) break;
	}
	if (!bHasMesh)
		return false;

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
bool Mesh::Save(const String& fileName, const cList<String>& comments, bool bBinary, bool bTexLossless) const
{
	if (IsEmpty())
		return false;
	TD_TIMER_STARTD();
	const String ext(Util::getFileExt(fileName).ToLower());
	bool ret;
	if (ext == _T(".obj"))
		ret = SaveOBJ(fileName);
	else
	if (ext == _T(".gltf") || ext == _T(".glb"))
		ret = SaveGLTF(fileName, ext == _T(".glb"));
	else
		ret = SavePLY(ext != _T(".ply") ? String(fileName+_T(".ply")) : fileName, comments, bBinary, bTexLossless);
	if (!ret)
		return false;
	DEBUG_EXTRA("Mesh '%s' saved: %u vertices, %u faces (%s)",
		Util::getFileNameExt(fileName).c_str(), vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
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
		    const String textureFileName(Util::getFileFullName(fileName) + std::to_string((unsigned)texId).c_str() + (bTexLossless?_T(".png"):_T(".jpg")));
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
	STATIC_ASSERT(sizeof(ObjModel::Vertex) == sizeof(Vertex));
	model.get_vertices().insert(model.get_vertices().begin(), vertices.begin(), vertices.end());

	// store vertex normals
	STATIC_ASSERT(sizeof(ObjModel::Normal) == sizeof(Normal));
	ASSERT(model.get_vertices().size() < std::numeric_limits<VIndex>::max());
	if (!vertexNormals.empty()) {
		ASSERT(vertexNormals.size() == vertices.size());
		model.get_normals().insert(model.get_normals().begin(), vertexNormals.begin(), vertexNormals.end());
	}

	// store face texture coordinates
	STATIC_ASSERT(sizeof(ObjModel::TexCoord) == sizeof(TexCoord));
	if (!faceTexcoords.empty()) {
		// translate, normalize and flip Y axis of the texture coordinates
		TexCoordArr normFaceTexcoords;
		FaceTexcoordsNormalize(normFaceTexcoords, true);
		ASSERT(normFaceTexcoords.size() == faces.size()*3);
		model.get_texcoords().insert(model.get_texcoords().begin(), normFaceTexcoords.begin(), normFaceTexcoords.end());
	}

	// store faces
	TexIndex idxTexture(0);
	do {
		ObjModel::Group& group = model.AddGroup(HasTexture() ? String::FormatString("material_%02u", idxTexture) : String(""));
		group.faces.reserve(texturesDiffuse.empty() ? faces.size() : faces.size()/texturesDiffuse.size());
		FOREACH(idxFace, faces) {
			if (!faceTexindices.empty() && faceTexindices[idxFace] != idxTexture)
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
			group.faces.emplace_back(f);
		}

		// store texture
		if (HasTexture()) {
			ObjModel::MaterialLib::Material* pMaterial(model.GetMaterial(group.material_name));
			ASSERT(pMaterial != NULL);
			pMaterial->diffuse_map = texturesDiffuse[idxTexture];
		}
	} while (++idxTexture < texturesDiffuse.size());

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
			return SaveImage(
				cv::Mat(image->height, image->width, CV_8UC3, image->image.data()),
				Util::ensureFolderSlash(basePath) + image->uri);
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

void Mesh::EnsureEdgeSize(float epsilonMin, float epsilonMax, float collapseRatio, float degenerate_angle_deg, int mode, int max_iters)
{
	TD_TIMER_STARTD();
	CLEAN::SurfaceMesh sm;
	CLEAN::ImportMesh(sm, *this);

	// compute mean edge length for negative parameter interpretation
	double totalEdgeLen(0);
	std::size_t nEdges(0);
	for (CLEAN::edge_descriptor ed : sm.edges()) {
		const CLEAN::halfedge_descriptor hd = sm.halfedge(ed);
		const CLEAN::Point_3& p0 = sm.point(sm.source(hd));
		const CLEAN::Point_3& p1 = sm.point(sm.target(hd));
		const double dx = p1.x()-p0.x(), dy = p1.y()-p0.y(), dz = p1.z()-p0.z();
		totalEdgeLen += SQRT(dx*dx + dy*dy + dz*dz);
		++nEdges;
	}
	const double meanEdge = nEdges > 0 ? totalEdgeLen / nEdges : 1.0;

	// interpret negative parameters as multipliers of mean edge length
	double targetMin = epsilonMin < 0 ? meanEdge * (-epsilonMin) : epsilonMin;
	double targetMax = epsilonMax < 0 ? MAXF(meanEdge * (-epsilonMax), targetMin * 2) : epsilonMax;
	const double targetEdge = (targetMin + targetMax) / 2;

	// PMP::isotropic_remeshing replaces the entire custom edge size control:
	// split long edges, collapse short edges, flip edges for valence, relocate vertices
	CLEAN::PMP::isotropic_remeshing(
		sm.faces(), targetEdge, sm,
		CGAL::parameters::number_of_iterations(max_iters > 0 ? MINF(max_iters, 10) : 3));

	CLEAN::ExportMesh(sm, *this);
	DEBUG("Ensured edge size [%g, %g] (target %g, mean was %g) (%s)",
		targetMin, targetMax, targetEdge, meanEdge, TD_TIMER_GET_FMT().c_str());
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


// remove degenerate faces, with one or more identical vertices or very close vertices (0 - disabled);
// unreferenced vertices and non-manifold edges/vertices can be created,
// so should be followed by RemoveUnreferencedVertices() and FixNonManifold()
Mesh::FIndex Mesh::RemoveDegenerateFaces(Type thArea) {
	if (vertexFaces.size() != vertices.size())
		ListIncidentFaces();
	const Type thDoubleAreaSq = SQUARE(thArea * 2);
	FaceIdxArr facesRemove;
	typedef std::pair<VIndex/*replace with*/, VIndex> Vertex2Vertex;
	CLISTDEF0(Vertex2Vertex) vertexPairs;
	RFOREACH(idxFace, faces) {
		const Face& face = faces[idxFace];
		// check first case when one or more vertices have same index
		if (face[0] == face[1] || face[0] == face[2] || face[1] == face[2]) {
			// just remove the face
			facesRemove.emplace_back(idxFace);
			continue;
		}
		if (thDoubleAreaSq <= 0)
			continue;
		// check if the face has almost 0 area (see EdgeFunction())
		const Vertex& v0 = vertices[face[0]];
		const Vertex& v1 = vertices[face[1]];
		const Vertex& v2 = vertices[face[2]];
		const Vertex A(v2 - v0);
		const Vertex B(v1 - v0);
		const Type doubleAreaSq = normSq(B.cross(A));
		if (doubleAreaSq <= thDoubleAreaSq) {
			// remove the face
			facesRemove.emplace_back(idxFace);
			const Type lenghSqA = normSq(A);
			const Type lenghSqB = normSq(B);
			const Type lenghSqC = normSq(v2 - v1);
			// remove two of the vertices,
			// moving all adjacent face to the remaining vertex
			if (lenghSqA <= thArea && lenghSqB <= thArea) {
				vertexPairs.emplace_back(face[2], face[0]);
				vertexPairs.emplace_back(face[1], face[0]);
			}
			else if (lenghSqA <= thArea && lenghSqC <= thArea) {
				vertexPairs.emplace_back(face[0], face[2]);
				vertexPairs.emplace_back(face[1], face[2]);
			}
			else if (lenghSqB <= thArea && lenghSqC <= thArea) {
				vertexPairs.emplace_back(face[0], face[1]);
				vertexPairs.emplace_back(face[2], face[1]);
			} else
			// remove one of the vertices,
			// moving all adjacent face to the closest remaining vertices
			if (lenghSqA <= thArea) {
				vertexPairs.emplace_back(face[2], face[0]);
			}
			else if (lenghSqB <= thArea) {
				vertexPairs.emplace_back(face[1], face[0]);
			}
			else if (lenghSqC <= thArea) {
				vertexPairs.emplace_back(face[1], face[2]);
			} else {
				// the vertices are (almost) collinear, remove the smallest edge
				if (lenghSqA < lenghSqB) {
					if (lenghSqA < lenghSqC)
						vertexPairs.emplace_back(face[2], face[0]);
					else
						vertexPairs.emplace_back(face[2], face[1]);
				} else {
					if (lenghSqB < lenghSqC)
						vertexPairs.emplace_back(face[1], face[0]);
					else
						vertexPairs.emplace_back(face[2], face[1]);
				}
			}
		}
	}
	if (facesRemove.empty())
		return 0;
	RemoveFaces(facesRemove, true);
	if (vertexPairs.empty()) {
		DEBUG("Removed %u degenerate faces", facesRemove.size());
		return facesRemove.size();
	}
	// replace first vertex with the second
	VertexIdxArr mapRemovedVerts(vertices.size());
	mapRemovedVerts.MemsetValue(NO_ID);
	const auto TraceMovedVertex = [&mapRemovedVerts](IIndex idx) {
		while (mapRemovedVerts[idx] != NO_ID)
			idx = mapRemovedVerts[idx];
		return idx;
	};
	vertexPairs.RemoveDuplicates();
	RFOREACHPTR(ptrIdxPair, vertexPairs) {
		Vertex2Vertex p = *ptrIdxPair;
		p.first = TraceMovedVertex(p.first);
		p.second = TraceMovedVertex(p.second);
		if (p.first == p.second)
			continue;
		FaceIdxArr& firstVfs = vertexFaces[p.first];
		for (const FIndex idxFace : firstVfs) {
			Face& face = faces[idxFace];
			for (VIndex i = 0; i < 3; ++i)
				if (face[i] == p.first)
					face[i] = p.second;
		}
		FaceIdxArr& secondVfs = vertexFaces[p.second];
		secondVfs.Join(firstVfs);
		secondVfs.RemoveDuplicates();
		firstVfs.Release();
		mapRemovedVerts[p.first] = p.second;
	}
	const FIndex numRemovedFaces = facesRemove.size() + RemoveDegenerateFaces(0.f);
	if (numRemovedFaces > 0)
		DEBUG_ULTIMATE("Removed %u zero-area faces", numRemovedFaces);
	return numRemovedFaces;
}

// removing zero-area-faces can generate some new zero-area-faces,
// so iterate till no zero-area faces are encountered or max number of iterations is reached
Mesh::FIndex Mesh::RemoveDegenerateFaces(unsigned maxIterations, Type thArea) {
	FIndex totalNumRemovedFaces = 0;
	for (unsigned iter=0; iter<maxIterations; ++iter) {
		const FIndex numRemovedFaces = RemoveDegenerateFaces(thArea);
		if (numRemovedFaces == 0)
			break;
		totalNumRemovedFaces += numRemovedFaces;
	}
	return totalNumRemovedFaces;
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
			ListIncidentFaces();
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
			if (!faceNormals.empty())
				faceNormals.RemoveAt(idxF);
			if (!faceTexcoords.empty())
				faceTexcoords.RemoveAt(idxF * 3, 3);
			if (!faceTexindices.empty())
				faceTexindices.RemoveAt(idxF);
			idxLast = idxF;
		}
		vertexFaces.Release();
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
			if (!faceNormals.empty())
				faceNormals.RemoveAt(idxF);
			if (!faceTexcoords.empty())
				faceTexcoords.RemoveAt(idxF * 3, 3);
			if (!faceTexindices.empty())
				faceTexindices.RemoveAt(idxF);
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
				for (const FIndex idxF : vf)
					GetVertex(faces[idxF], idxVM) = idxV;
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
			for (const FIndex idxF : vf)
				GetVertex(faces[idxF], idxVM) = idxV;
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

// remove duplicate vertices (equal coordinates);
// return the number of removed vertices, and optionally the list of duplicated vertices
Mesh::VIndex Mesh::RemoveDuplicatedVertices(VertexIdxArr* duplicatedVertices) {
	// create a map of unique vertices
	VIndex numUniqueVertices(0);
    VertexIdxArr mapVertices(vertices.size()); {
		std::unordered_map<Vertex,VIndex,std::hash<Vertex::Base>> mapVertexIndex;
		FOREACH(i, vertices) {
			const Vertex& vertex = vertices[i];
			const auto ret = mapVertexIndex.emplace(vertex, (VIndex)mapVertexIndex.size());
			if (ret.second) {
				// new vertex found
				mapVertices[i] = i;
			} else {
				// duplicate vertex found
				mapVertices[i] = ret.first->second;
				if (duplicatedVertices)
					duplicatedVertices->push_back(i);
			}
		}
		numUniqueVertices = (VIndex)mapVertexIndex.size();
		if (numUniqueVertices == vertices.size())
			return 0;
		ReleaseComputable();
	}
    // update the vertices and vertexNormals arrays
    VertexArr newVertices(0, numUniqueVertices);
    VertexArr newVertexNormals;
	if (!vertexNormals.empty())
		newVertexNormals.reserve(numUniqueVertices);
	FOREACH(i, vertices) {
		VIndex& uniqueIndex = mapVertices[i];
		if (uniqueIndex != i)
			continue;
		uniqueIndex = newVertices.size();
        newVertices.emplace_back(vertices[i]);
		if (!vertexNormals.empty())
			newVertexNormals.emplace_back(vertexNormals[i]);
    }
    vertices = std::move(newVertices);
	if (!vertexNormals.empty())
		vertexNormals = std::move(newVertexNormals);
    // update the vertex indices in the faces
    for (Face& face: faces) {
        for (int i = 0; i < 3; ++i) {
            face[i] = mapVertices[face[i]];
			ASSERT(face[i] != NO_ID);
        }
    }
    const VIndex numDuplicated(mapVertices.size() - vertices.size());
	DEBUG_ULTIMATE("Removed %u duplicated vertices", numDuplicated);
	return numDuplicated;
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
	DEBUG_ULTIMATE("Removed %u unreferenced vertices", vertexRemove.size());
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
		const TexIndex ti = GetFaceTextureIndex(idxF);
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
		// create the point-cloud by sampling the mesh
		if (sampleMesh > 0)
			SamplePoints(sampleMesh, 0, pointcloud);
		else
			SamplePoints(ROUND2INT<unsigned>(-sampleMesh), pointcloud);
	} else {
		// create the point-cloud containing all vertices
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
	RasterMesh::Triangle triangle;
	RasterMesh::TriangleRasterizer triangleRasterizer(triangle, rasterer);
	rasterer.Clear();
	for (const Face& facet: faces)
		rasterer.Project(facet, triangleRasterizer);
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
		void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
			const Point3f pbary(PerspectiveCorrectBarycentricCoordinates(t, bary));
			const Depth z(ComputeDepth(t, pbary));
			ASSERT(z > Depth(0));
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				xt  = mesh.faceTexcoords[idxFaceTex+0] * pbary[0];
				xt += mesh.faceTexcoords[idxFaceTex+1] * pbary[1];
				xt += mesh.faceTexcoords[idxFaceTex+2] * pbary[2];
				const auto texIdx = mesh.GetFaceTextureIndex(idxFaceTex / 3);
				image(pt) = mesh.texturesDiffuse[texIdx].sampleSafe(xt);
			}
		}
	};
	if (image.size() != depthMap.size())
		image.create(depthMap.size());
	RasterMesh rasterer(*this, camera, depthMap, image);
	RasterMesh::Triangle triangle;
	RasterMesh::TriangleRasterizer triangleRasterizer(triangle, rasterer);
	rasterer.Clear();
	FOREACH(idxFace, faces) {
		const Face& facet = faces[idxFace];
		rasterer.idxFaceTex = idxFace*3;
		rasterer.Project(facet, triangleRasterizer);
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
		inline void Project(const Face& facet, TriangleRasterizer& tr) {
			idxVerts = facet.ptr();
			Base::Project(facet, tr);
		}
		void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
			const Point3f pbary(PerspectiveCorrectBarycentricCoordinates(t, bary));
			const Depth z(ComputeDepth(t, pbary));
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
	RasterMesh::Triangle triangle;
	RasterMesh::TriangleRasterizer triangleRasterizer(triangle, rasterer);
	rasterer.Clear();
	// render the entire mesh
	for (const Face& facet: faces)
		rasterer.Project(facet, triangleRasterizer);
}
// project mesh to the given camera plane using orthographic projection
void Mesh::ProjectOrtho(const Camera& camera, DepthMap& depthMap) const
{
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		RasterMesh(const VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap)
			: Base(_vertices, _camera, _depthMap) {}
		inline bool ProjectVertex(const Mesh::Vertex& pt, int v, Triangle& t) {
			return (t.ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt))).z > 0 &&
				depthMap.isInsideWithBorder<float,3>(t.pti[v] = camera.TransformPointOrthoC2I(t.ptc[v]));
		}
		void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
			const Depth z(ComputeDepth(t, bary));
			ASSERT(z > Depth(0));
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z)
				depth = z;
		}
	};
	RasterMesh rasterer(vertices, camera, depthMap);
	RasterMesh::Triangle triangle;
	RasterMesh::TriangleRasterizer triangleRasterizer(triangle, rasterer);
	rasterer.Clear();
	for (const Face& facet: faces)
		rasterer.Project(facet, triangleRasterizer);
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
		inline bool ProjectVertex(const Mesh::Vertex& pt, int v, Triangle& t) {
			return (t.ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt))).z > 0 &&
				depthMap.isInsideWithBorder<float,3>(t.pti[v] = camera.TransformPointOrthoC2I(t.ptc[v]));
		}
		void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
			const Depth z(ComputeDepth(t, bary));
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
	RasterMesh::Triangle triangle;
	RasterMesh::TriangleRasterizer triangleRasterizer(triangle, rasterer);
	rasterer.Clear();
	FOREACH(idxFace, faces) {
		const Face& facet = faces[idxFace];
		rasterer.idxFaceTex = idxFace*3;
		rasterer.Project(facet, triangleRasterizer);
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
	mesh.ListIncidentFaces();
	mesh.RemoveUnreferencedVertices();
	mesh.FixNonManifold();
	return mesh;
} // SubMesh
/*----------------------------------------------------------------*/

// extract one sub-mesh for each texture, i.e. for each value of faceTexindices:
//  - mapFaceSubsetIndices: if not null, for each face of the original mesh,
//    contains the index of the face in the corresponding sub-mesh
std::vector<Mesh> Mesh::SplitMeshPerTextureBlob(FaceIdxArr* mapFaceSubsetIndices) const {
	ASSERT(HasTexture());
	if (texturesDiffuse.size() == 1)
		return {*this};
	if (mapFaceSubsetIndices)
		mapFaceSubsetIndices->resize(faces.size());
	ASSERT(faceTexindices.size() == faces.size());
	std::vector<Mesh> submeshes;
	submeshes.reserve(texturesDiffuse.size());
	FOREACH(texId, texturesDiffuse) {
		FaceIdxArr chunk;
		FOREACH(idxFace, faceTexindices) {
			if (faceTexindices[idxFace] == texId) {
				if (mapFaceSubsetIndices)
					(*mapFaceSubsetIndices)[idxFace] = chunk.size();
				chunk.push_back(idxFace);
			}
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
	const FIndex numFaces(faceSubsetIndices.empty() ? mesh.faces.size() : faceSubsetIndices.size());
	if (vertices == mesh.vertices && faces == mesh.faces) {
		// the two meshes are identical, only the texture coordinates are different;
		// directly transfer the texture onto the new coordinates
		#ifdef MESH_USE_OPENMP
		#pragma omp parallel for schedule(dynamic)
		for (int_t i=0; i<(int_t)numFaces; ++i) {
			const FIndex idx((FIndex)i);
		#else
		FOREACHRAW(idx, numFaces) {
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
			ListIncidentFaces();
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
		for (int_t i=0; i<(int_t)numFaces; ++i) {
			const FIndex idx((FIndex)i);
		#else
		FOREACHRAW(idx, numFaces) {
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

// compute the memory size occupied by the mesh (in bytes)
size_t MVS::Mesh::GetMemorySize() const {
	if (IsEmpty())
		return 0;
	size_t nBytes = vertices.GetMemorySize();
	nBytes += faces.GetMemorySize();
	nBytes += vertexNormals.GetMemorySize();
	nBytes += vertexVertices.GetMemorySize();
	nBytes += vertexFaces.GetMemorySize();
	nBytes += vertexBoundary.GetMemorySize();
	nBytes += faceNormals.GetMemorySize();
	nBytes += faceFaces.GetMemorySize();
	nBytes += faceTexcoords.GetMemorySize();
	nBytes += faceTexindices.GetMemorySize();
	nBytes += texturesDiffuse.GetMemorySize();
	for (const Image8U3& textureDiffuse: texturesDiffuse)
		nBytes += textureDiffuse.memory_size();
	return nBytes;
}
/*----------------------------------------------------------------*/


#ifdef _USE_OPENMP
// test mesh projection on the image using multi-threaded and single-threaded rasterization
bool MVS::TestMeshProjectionMT(const Mesh& mesh, const Image& image) {
	// used to render the mesh
	typedef TImage<cuint32_t> FaceMap;
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		FaceMap& faceMap;
		RasterMesh(const Mesh::VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap, FaceMap& _faceMap)
			: Base(_vertices, _camera, _depthMap), faceMap(_faceMap) {}
		void Clear() {
			Base::Clear();
			faceMap.memset((uint8_t)NO_ID);
		}
		void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary, Mesh::FIndex idxFace) {
			const Point3f pbary(PerspectiveCorrectBarycentricCoordinates(t, bary));
			const Depth z(ComputeDepth(t, pbary));
			ASSERT(z > Depth(0));
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				faceMap(pt) = idxFace;
			}
		}
	};
	struct TriangleRasterizer {
		RasterMesh* rasterizer;
		RasterMesh::Triangle triangle;
		Mesh::FIndex idxFace;
		inline cv::Size Size() const {
			return rasterizer->Size();
		}
		inline void operator()(const ImageRef& pt, const Point3f& bary) const {
			rasterizer->Raster(pt, triangle, bary, idxFace);
		}
	};
	// project mesh on the image
	DepthMap depthMapMT(image.GetSize());
	FaceMap faceMapMT(image.GetSize());
	{	// multi-threaded rasterization
		RasterMesh rasterer(mesh.vertices, image.camera, depthMapMT, faceMapMT);
		TriangleRasterizer triangleRasterizer{&rasterer};
		rasterer.Clear();
		#pragma omp parallel for firstprivate(triangleRasterizer) schedule(dynamic)
		for (int_t i=0; i<(int_t)mesh.faces.size(); ++i) {
			const Mesh::FIndex idxFace = (Mesh::FIndex)i;
			const Mesh::Face& facet = mesh.faces[idxFace];
			triangleRasterizer.idxFace = idxFace;
			rasterer.Project(facet, triangleRasterizer);
		}
	}
	DepthMap depthMapST(image.GetSize());
	FaceMap faceMapST(image.GetSize());
	{	// single-threaded rasterization
		RasterMesh rasterer(mesh.vertices, image.camera, depthMapST, faceMapST);
		TriangleRasterizer triangleRasterizer{&rasterer};
		rasterer.Clear();
		FOREACH(idxFace, mesh.faces) {
			const Mesh::Face& facet = mesh.faces[idxFace];
			triangleRasterizer.idxFace = idxFace;
			rasterer.Project(facet, triangleRasterizer);
		}
	}
	// compare results
	unsigned numDiffDepths(0), numDiffFaces(0);
	for (int y = 0; y<depthMapST.rows; ++y) {
		for (int x = 0; x<depthMapST.cols; ++x) {
			const Depth depthMT = depthMapMT(y,x);
			const Depth depthST = depthMapST(y,x);
			if (depthMT != depthST)
				++numDiffDepths;
			const cuint32_t faceMT = faceMapMT(y,x);
			const cuint32_t faceST = faceMapST(y,x);
			if (faceMT != faceST)
				++numDiffFaces;
		}
	}
	VERBOSE("Mesh rasterization: %u different depths, %u different faces", numDiffDepths, numDiffFaces);
	return numDiffDepths == 0 && numDiffFaces == 0;
}
/*----------------------------------------------------------------*/
#endif // _USE_OPENMP

#pragma pop_macro("VERBOSE")
