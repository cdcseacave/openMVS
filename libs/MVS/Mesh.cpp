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
	vertexColors.Release();
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
	vertexColors.Empty();
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
	vertexColors.Swap(rhs.vertexColors);
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

// compute the normal of every vertex as the area weighted average of its face normals
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
	const float cosMaxGradient = COS(D2R(fMaxGradient));
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
			// no neighbour within the gradient (an isolated face, or one on a crease sharper than it on
			// every side) keeps its own normal: the zero sum has no direction to blend in
			if (sumNeighborNormals == Normal::ZERO) {
				newFaceNormals[idxFace] = originalNormal;
				continue;
			}
			const Normal avgNeighborsNormal = normalized(sumNeighborNormals);
			const Normal newFaceNormal = normalized(originalNormal * fOriginalWeight + avgNeighborsNormal * (1.f - fOriginalWeight));
			newFaceNormals[idxFace] = newFaceNormal;
		}
		newFaceNormals.Swap(faceNormals);
	}
}
/*----------------------------------------------------------------*/


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

/*----------------------------------------------------------------*/

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
		ret = LoadGLTF(fileName);
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

	// open the PLY file and read its header
	PLY ply;
	if (!ply.Open(fileName)) {
		DEBUG_EXTRA("error: invalid PLY file");
		return false;
	}
	const std::vector<PLY::Element>& elements(ply.GetElements());
	const auto FindElement = [&elements](const char* name) -> size_t {
		for (size_t e = 0; e < elements.size(); ++e)
			if (elements[e].name == name)
				return e;
		return SIZE_MAX;
	};
	const size_t idxVertex(FindElement("vertex")), idxFace(FindElement("face"));
	const size_t numVertices(idxVertex != SIZE_MAX ? elements[idxVertex].count : 0);
	const size_t numFaces(idxFace != SIZE_MAX ? elements[idxFace].count : 0);
	if (numVertices == 0 && numFaces == 0)
		return true;
	// a PLY without faces is a point-cloud
	if (numVertices == 0 || numFaces == 0 ||
		numVertices > std::numeric_limits<VIndex>::max() || numFaces > std::numeric_limits<FIndex>::max())
		return false;

	// read the vertices and the faces, in file order
	for (size_t e = 0; e < elements.size(); ++e) {
		const PLY::Element& elem(elements[e]);
		PLY::Bindings bindings;
		if (e == idxVertex) {
			vertices.resize((VIndex)numVertices);
			bindings = {
				PLY::Scalar("x", &vertices[0].x, sizeof(Vertex)),
				PLY::Scalar("y", &vertices[0].y, sizeof(Vertex)),
				PLY::Scalar("z", &vertices[0].z, sizeof(Vertex))};
			if (elem.HasProperties({"nx", "ny", "nz"})) {
				vertexNormals.resize(vertices.size());
				bindings.push_back(PLY::Scalar("nx", &vertexNormals[0].x, sizeof(Normal)));
				bindings.push_back(PLY::Scalar("ny", &vertexNormals[0].y, sizeof(Normal)));
				bindings.push_back(PLY::Scalar("nz", &vertexNormals[0].z, sizeof(Normal)));
			}
			if (elem.HasProperties({"red", "green", "blue"})) {
				vertexColors.resize(vertices.size());
				bindings.push_back(PLY::Scalar("red", &vertexColors[0].r, sizeof(Color)));
				bindings.push_back(PLY::Scalar("green", &vertexColors[0].g, sizeof(Color)));
				bindings.push_back(PLY::Scalar("blue", &vertexColors[0].b, sizeof(Color)));
			}
		} else if (e == idxFace) {
			faces.resize((FIndex)numFaces);
			bindings = {PLY::List(elem.FindProperty("vertex_indices") ? "vertex_indices" : "vertex_index", &faces[0].x, 3, sizeof(Face))};
			if (elem.FindProperty("texcoord")) {
				faceTexcoords.resize(faces.size()*3);
				bindings.push_back(PLY::List("texcoord", &faceTexcoords[0].x, 6, sizeof(TexCoord)*3));
				if (elem.FindProperty("texnumber")) {
					faceTexindices.resize(faces.size());
					bindings.push_back(PLY::Scalar("texnumber", &faceTexindices[0], sizeof(TexIndex)));
				}
			}
		} else {
			continue;
		}
		if (!ply.ReadElement(e, bindings)) {
			Release();
			return false;
		}
	}
	if (*std::max_element(&faces[0].x, &faces[0].x+faces.size()*3) >= vertices.size()) {
		DEBUG_EXTRA("error: invalid mesh file (face vertex index out of range)");
		Release();
		return false;
	}
	if (!faceTexcoords.empty()) {
		// load the texture
		for (const std::string& comment: ply.GetComments()) {
			if (_tcsncmp(comment.c_str(), _T("TextureFile "), 12) == 0) {
				const String textureFileName(comment.substr(12));
				texturesDiffuse.emplace_back().Load(Util::getFilePath(fileName)+textureFileName);
			}
		}
		if (texturesDiffuse.size() <= 1)
			faceTexindices.Release();
		else if (std::any_of(faceTexindices.begin(), faceTexindices.end(), [this](TexIndex idx) { return idx >= texturesDiffuse.size(); })) {
			DEBUG_EXTRA("error: invalid mesh file (face texture index without a texture file)");
			Release();
			return false;
		}
		// flip Y axis, unnormalize and translate back texture coordinates
		TexCoordArr unnormFaceTexcoords;
		FaceTexcoordsUnnormalize(unnormFaceTexcoords, true);
		faceTexcoords.Swap(unnormFaceTexcoords);
	}
	return true;
}
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
		ret = SaveOBJ(fileName, bTexLossless);
	else
	if (ext == _T(".gltf") || ext == _T(".glb"))
		ret = SaveGLTF(fileName, ext == _T(".glb"), bTexLossless);
	else
		ret = SavePLY(ext != _T(".ply") ? String(fileName+_T(".ply")) : fileName, comments, bBinary, bTexLossless);
	if (!ret) {
		VERBOSE("error: failed saving mesh '%s'", fileName.c_str());
		return false;
	}
	DEBUG_EXTRA("Mesh '%s' saved: %u vertices, %u faces (%s)",
		Util::getFileNameExt(fileName).c_str(), vertices.size(), faces.size(), TD_TIMER_GET_FMT().c_str());
	return true;
}

// export the mesh as a PLY file
bool Mesh::SavePLY(const String& fileName, const cList<String>& comments, bool bBinary, bool bTexLossless) const
{
	ASSERT(!fileName.empty());
	ASSERT(vertexNormals.empty() || vertexNormals.size() == vertices.size());
	ASSERT(vertexColors.empty() || vertexColors.size() == vertices.size());
	Util::ensureFolder(fileName);
	PLY ply;

	// export comments
	for (const String& comment: comments)
		ply.AddComment(comment);

	// export texture file name as comment if needed
	if (HasTexture()) {
		FOREACH(texId, texturesDiffuse) {
		    const String textureFileName(Util::getFileFullName(fileName) + std::to_string((unsigned)texId).c_str() + (bTexLossless?_T(".png"):_T(".jpg")));
		    ply.AddComment(_T("TextureFile ")+Util::getFileNameExt(textureFileName));
		    if (!texturesDiffuse[texId].Save(textureFileName))
		        return false;
		}
	}

	// export the vertices and the faces; both elements are written even without records, so an empty
	// mesh, or one without faces, still reads as a mesh (the vertex positions are always declared)
	const Vertex* const firstVertex(vertices.empty() ? NULL : vertices.data());
	PLY::Bindings vertexBindings{
		PLY::Scalar("x", firstVertex ? &firstVertex->x : NULL, sizeof(Vertex)),
		PLY::Scalar("y", firstVertex ? &firstVertex->y : NULL, sizeof(Vertex)),
		PLY::Scalar("z", firstVertex ? &firstVertex->z : NULL, sizeof(Vertex))};
	if (!vertexNormals.empty()) {
		vertexBindings.push_back(PLY::Scalar("nx", &vertexNormals[0].x, sizeof(Normal)));
		vertexBindings.push_back(PLY::Scalar("ny", &vertexNormals[0].y, sizeof(Normal)));
		vertexBindings.push_back(PLY::Scalar("nz", &vertexNormals[0].z, sizeof(Normal)));
	}
	if (!vertexColors.empty()) {
		vertexBindings.push_back(PLY::Scalar("red", &vertexColors[0].r, sizeof(Color)));
		vertexBindings.push_back(PLY::Scalar("green", &vertexColors[0].g, sizeof(Color)));
		vertexBindings.push_back(PLY::Scalar("blue", &vertexColors[0].b, sizeof(Color)));
	}
	ply.AddElement("vertex", vertices.size(), std::move(vertexBindings));
	PLY::Bindings faceBindings;
	TexCoordArr normFaceTexcoords;
	if (!faces.empty()) {
		faceBindings.push_back(PLY::List("vertex_indices", &faces[0].x, 3, sizeof(Face)));
		if (!faceTexcoords.empty()) {
			// translate, normalize and flip Y axis of the texture coordinates
			FaceTexcoordsNormalize(normFaceTexcoords, true);
			faceBindings.push_back(PLY::List("texcoord", &normFaceTexcoords[0].x, 6, sizeof(TexCoord)*3));
			if (!faceTexindices.empty())
				faceBindings.push_back(PLY::Scalar("texnumber", &faceTexindices[0], sizeof(TexIndex), PLY::Int32));
		}
	}
	ply.AddElement("face", faces.size(), std::move(faceBindings));
	return ply.Save(fileName, bBinary ? PLY::BINARY_LE : PLY::ASCII);
}
bool Mesh::Save(const FacesChunkArr& chunks, const String& fileName, const cList<String>& comments, bool bBinary) const
{
	if (chunks.size() < 2)
		return Save(fileName, comments, bBinary);
	std::vector<FaceIdxArr> faceChunks;
	faceChunks.reserve(chunks.size());
	for (const FaceChunk& chunk: chunks)
		faceChunks.emplace_back(chunk.faces);
	const std::vector<Mesh> meshes(SubMeshes(faceChunks));
	FOREACH(i, chunks)
		if (!meshes[i].Save(Util::insertBeforeFileExt(fileName, String::FormatString("_chunk%02u", i)), comments, bBinary))
			return false;
	return true;
}

bool Mesh::Save(const VertexArr& vertices, const String& fileName, bool bBinary)
{
	ASSERT(!fileName.empty());
	Util::ensureFolder(fileName);
	const Vertex* const firstVertex(vertices.empty() ? NULL : vertices.data());
	PLY ply;
	ply.AddElement("vertex", vertices.size(), {
		PLY::Scalar("x", firstVertex ? &firstVertex->x : NULL, sizeof(Vertex)),
		PLY::Scalar("y", firstVertex ? &firstVertex->y : NULL, sizeof(Vertex)),
		PLY::Scalar("z", firstVertex ? &firstVertex->z : NULL, sizeof(Vertex))});
	return ply.Save(fileName, bBinary ? PLY::BINARY_LE : PLY::ASCII);
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
			RemoveVertexAttributes(idxV);
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
		RemoveVertexAttributes(idxV);
		vertices.RemoveAt(idxV);
		idxLast = idxV;
	}
	if (!facesRemove.empty())
		RemoveFaces(facesRemove);
}

// convert textured mesh to store texture coordinates per vertex instead of per face,
// splitting vertices shared by faces with different coordinates; for rendering only,
// the result is not a valid input for the library code (see faceTexcoords)
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
/*----------------------------------------------------------------*/


// project mesh to the given camera plane
void Mesh::SamplePoints(unsigned numberOfPoints, PointCloud& pointcloud, uint32_t seed) const
{
	// total mesh surface
	const REAL area(ComputeArea());
	if (area < ZEROTOLERANCE<float>()) {
		pointcloud.Release();
		return;
	}
	const REAL samplingDensity(numberOfPoints / area);
	return SamplePoints(samplingDensity, numberOfPoints, pointcloud, seed);
}
void Mesh::SamplePoints(REAL samplingDensity, PointCloud& pointcloud, uint32_t seed) const
{
	// compute the total area to deduce the number of points
	const REAL area(ComputeArea());
	const unsigned theoreticNumberOfPoints(CEIL2INT<unsigned>(area * samplingDensity));
	return SamplePoints(samplingDensity, theoreticNumberOfPoints, pointcloud, seed);
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
	std::vector<FaceIdxArr> chunks(texturesDiffuse.size());
	FOREACH(idxFace, faceTexindices) {
		ASSERT(faceTexindices[idxFace] < texturesDiffuse.size());
		FaceIdxArr& chunk = chunks[faceTexindices[idxFace]];
		if (mapFaceSubsetIndices)
			(*mapFaceSubsetIndices)[idxFace] = chunk.size();
		chunk.push_back(idxFace);
	}
	// an unused texture, e.g. after cropping, gets an empty mesh
	std::vector<Mesh> submeshes(SubMeshes(chunks));
	ASSERT(std::all_of(submeshes.begin(), submeshes.end(), [](const Mesh& submesh) { return submesh.IsEmpty() || submesh.texturesDiffuse.size() == 1; }));
	return submeshes;
}


// compute the memory size occupied by the mesh (in bytes)
size_t MVS::Mesh::GetMemorySize() const {
	if (IsEmpty())
		return 0;
	size_t nBytes = vertices.GetMemorySize();
	nBytes += faces.GetMemorySize();
	nBytes += vertexNormals.GetMemorySize();
	nBytes += vertexColors.GetMemorySize();
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
