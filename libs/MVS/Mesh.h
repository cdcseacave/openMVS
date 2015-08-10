/*
* Mesh.h
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

#ifndef _MVS_MESH_H_
#define _MVS_MESH_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Platform.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// a mesh represented by a list vertices and triangles (faces)
class Mesh
{
public:
	typedef TPoint3<float> Vertex;
	typedef uint32_t VIndex;
	typedef TPoint3<VIndex> Face;
	typedef uint32_t FIndex;

	typedef TPoint3<float> Normal;
	typedef cList<Normal,const Normal&,0,8192> NormalArr;

	typedef cList<Vertex,const Vertex&,0,8192,VIndex> VertexArr;
	typedef cList<Face,const Face&,0,8192> FaceArr;

	typedef cList<FIndex,FIndex,0,8> FaceIdxArr;
	typedef cList<FaceIdxArr> VertexFacesArr;

	// used to find adjacent face
	struct FaceCount {
		int count;
		inline FaceCount() : count(0) {}
	};
	typedef std::unordered_map<FIndex,FaceCount> FacetCountMap;

public:
	VertexArr vertices;
	FaceArr faces;

	NormalArr vertexNormals; // for each vertex, the normal to the surface in that point (optional)
	VertexFacesArr vertexFaces; // for each vertex, the list of faces containing it (optional)
	BoolArr vertexBoundary; // for each vertex, stores if it is at the boundary or not (optional)

	NormalArr faceNormals; // for each face, the normal to it (optional)

public:
	inline Mesh() {}

	void Release();
	void ReleaseExtra();
	void EmptyExtra();
	inline bool IsEmpty() const { return vertices.IsEmpty(); }

	void ListIncidenteFaces();
	void ListBoundaryVertices();
	void ComputeNormalFaces();
	void ComputeNormalVertices();

	bool FixNonManifold();
	void Clean(float fDecimate=0.7f, float fSpurious=10.f, bool bRemoveSpikes=true, unsigned nCloseHoles=30, unsigned nSmoothMesh=2);

	void EnsureEdgeSize(float minEdge=-0.5f, float maxEdge=-4.f, float collapseRatio=0.2, float degenerate_angle_deg=150, int mode=1, int max_iters=50);

	// file IO
	bool Load(const String& fileName);
	bool Save(const String& fileName, bool bBinary=true) const;
	static bool Save(const VertexArr& vertices, const String& fileName, bool bBinary=true);

	static inline uint32_t FindVertex(const Face& f, VIndex v) { for (uint32_t i=0; i<3; ++i) if (f[i] == v) return i; return NO_ID; }
	static inline VIndex GetVertex(const Face& f, VIndex v) { const uint32_t idx(FindVertex(f, v)); ASSERT(idx != NO_ID); return f[idx]; }
	static inline VIndex& GetVertex(Face& f, VIndex v) { const uint32_t idx(FindVertex(f, v)); ASSERT(idx != NO_ID); return f[idx]; }

	#ifdef _USE_BOOST
	// implement BOOST serialization
	template <class Archive>
	void serialize(Archive& ar, const unsigned int version) {
		ar & vertices;
		ar & faces;
	}
	#endif
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_MESH_H_
