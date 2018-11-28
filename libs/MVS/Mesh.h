/*
* Mesh.h
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

#ifndef _MVS_MESH_H_
#define _MVS_MESH_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Platform.h"
#include "PointCloud.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// a mesh represented by a list vertices and triangles (faces)
class MVS_API Mesh
{
public:
	typedef float Type;

	typedef TPoint3<Type> Vertex;
	typedef uint32_t VIndex;
	typedef TPoint3<VIndex> Face;
	typedef uint32_t FIndex;

	typedef cList<Vertex,const Vertex&,0,8192,VIndex> VertexArr;
	typedef cList<Face,const Face&,0,8192,FIndex> FaceArr;

	typedef cList<VIndex,VIndex,0,8,VIndex> VertexIdxArr;
	typedef cList<FIndex,FIndex,0,8,FIndex> FaceIdxArr;
	typedef cList<VertexIdxArr> VertexVerticesArr;
	typedef cList<FaceIdxArr> VertexFacesArr;

	typedef TPoint3<Type> Normal;
	typedef cList<Normal,const Normal&,0,8192,FIndex> NormalArr;

	typedef TPoint2<Type> TexCoord;
	typedef cList<TexCoord,const TexCoord&,0,8192,FIndex> TexCoordArr;

	// used to find adjacent face
	struct FaceCount {
		int count;
		inline FaceCount() : count(0) {}
	};
	typedef std::unordered_map<FIndex,FaceCount> FacetCountMap;
	typedef FaceCount VertCount;
	typedef std::unordered_map<VIndex,VertCount> VertCountMap;

	typedef AABB3f Box;

public:
	VertexArr vertices;
	FaceArr faces;

	NormalArr vertexNormals; // for each vertex, the normal to the surface in that point (optional)
	VertexVerticesArr vertexVertices; // for each vertex, the list of adjacent vertices (optional)
	VertexFacesArr vertexFaces; // for each vertex, the list of faces containing it (optional)
	BoolArr vertexBoundary; // for each vertex, stores if it is at the boundary or not (optional)

	NormalArr faceNormals; // for each face, the normal to it (optional)
	TexCoordArr faceTexcoords; // for each face, the texture-coordinates corresponding to the contained vertices (optional)

	Image8U3 textureDiffuse; // texture containing the diffuse color (optional)

	#ifdef _USE_CUDA
	static CUDA::KernelRT kernelComputeFaceNormal;
	#endif

public:
	inline Mesh() {
		#ifdef _USE_CUDA
		InitKernels();
		#endif
	}

	void Release();
	void ReleaseExtra();
	void EmptyExtra();
	inline bool IsEmpty() const { return vertices.IsEmpty(); }
	inline bool HasTexture() const { ASSERT(faceTexcoords.IsEmpty() == textureDiffuse.empty()); return !faceTexcoords.IsEmpty(); }

	Box GetAABB() const;
	Box GetAABB(const Box& bound) const;

	void ListIncidenteVertices();
	void ListIncidenteFaces();
	void ListBoundaryVertices();
	void ComputeNormalFaces();
	void ComputeNormalVertices();

	void GetEdgeFaces(VIndex, VIndex, FaceIdxArr&) const;
	void GetFaceFaces(FIndex, FaceIdxArr&) const;
	void GetEdgeVertices(FIndex, FIndex, uint32_t vs0[2], uint32_t vs1[2]) const;
	void GetAdjVertices(VIndex, VertexIdxArr&) const;
	void GetAdjVertexFaces(VIndex, VIndex, FaceIdxArr&) const;

	bool FixNonManifold();
	void Clean(float fDecimate=0.7f, float fSpurious=10.f, bool bRemoveSpikes=true, unsigned nCloseHoles=30, unsigned nSmoothMesh=2, bool bLastClean=true);

	void EnsureEdgeSize(float minEdge=-0.5f, float maxEdge=-4.f, float collapseRatio=0.2, float degenerate_angle_deg=150, int mode=1, int max_iters=50);

	typedef cList<uint16_t,uint16_t,0,16,FIndex> AreaArr;
	void Subdivide(const AreaArr& maxAreas, uint32_t maxArea);
	void Decimate(VertexIdxArr& verticesRemove);
	void CloseHole(VertexIdxArr& vertsLoop);
	void CloseHoleQuality(VertexIdxArr& vertsLoop);
	void RemoveFaces(FaceIdxArr& facesRemove, bool bUpdateLists=false);
	void RemoveVertices(VertexIdxArr& vertexRemove, bool bUpdateLists=false);

	inline Normal FaceNormal(const Face& f) const {
		return ComputeTriangleNormal(vertices[f[0]], vertices[f[1]], vertices[f[2]]);
	}
	inline Normal VertexNormal(VIndex idxV) const {
		ASSERT(vertices.GetSize() == vertexFaces.GetSize());
		const FaceIdxArr& vf = vertexFaces[idxV];
		ASSERT(!vf.IsEmpty());
		Normal n(Normal::ZERO);
		FOREACHPTR(pIdxF, vf)
			n += normalized(FaceNormal(faces[*pIdxF]));
		return n;
	}

	REAL ComputeArea() const;
	REAL ComputeVolume() const;

	void SamplePoints(unsigned numberOfPoints, PointCloud&) const;
	void SamplePoints(REAL samplingDensity, PointCloud&) const;
	void SamplePoints(REAL samplingDensity, unsigned mumPointsTheoretic, PointCloud&) const;

	void Project(const Camera& camera, DepthMap& depthMap) const;
	void Project(const Camera& camera, DepthMap& depthMap, Image8U3& image) const;
	void ProjectOrtho(const Camera& camera, DepthMap& depthMap) const;
	void ProjectOrtho(const Camera& camera, DepthMap& depthMap, Image8U3& image) const;
	void ProjectOrthoTopDown(unsigned resolution, Image8U3& image, Image8U& mask, Point3& center) const;

	// file IO
	bool Load(const String& fileName);
	bool Save(const String& fileName, const cList<String>& comments=cList<String>(), bool bBinary=true) const;
	static bool Save(const VertexArr& vertices, const String& fileName, bool bBinary=true);

	static inline uint32_t FindVertex(const Face& f, VIndex v) { for (uint32_t i=0; i<3; ++i) if (f[i] == v) return i; return NO_ID; }
	static inline VIndex GetVertex(const Face& f, VIndex v) { const uint32_t idx(FindVertex(f, v)); ASSERT(idx != NO_ID); return f[idx]; }
	static inline VIndex& GetVertex(Face& f, VIndex v) { const uint32_t idx(FindVertex(f, v)); ASSERT(idx != NO_ID); return f[idx]; }

protected:
	bool LoadPLY(const String& fileName);
	bool LoadOBJ(const String& fileName);

	bool SavePLY(const String& fileName, const cList<String>& comments=cList<String>(), bool bBinary=true) const;
	bool SaveOBJ(const String& fileName) const;

	#ifdef _USE_CUDA
	static bool InitKernels(int device=-1);
	#endif

	#ifdef _USE_BOOST
	// implement BOOST serialization
	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive& ar, const unsigned int /*version*/) {
		ar & vertices;
		ar & faces;
		ar & vertexNormals;
		ar & vertexVertices;
		ar & vertexFaces;
		ar & vertexBoundary;
		ar & faceNormals;
		ar & faceTexcoords;
		ar & textureDiffuse;
	}
	#endif
};
/*----------------------------------------------------------------*/


// used to render a mesh
template <typename DERIVED>
struct TRasterMesh {
	const Mesh::VertexArr& vertices;
	const Camera& camera;

	DepthMap& depthMap;

	Point3 normalPlane;
	Point3 ptc[3];
	Point2f pti[3];

	TRasterMesh(const Mesh::VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap)
		: vertices(_vertices), camera(_camera), depthMap(_depthMap) {}

	inline void Clear() {
		depthMap.memset(0);
	}

	inline bool ProjectVertex(const Mesh::Vertex& pt, int v) {
		ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt));
		pti[v] = camera.TransformPointC2I(ptc[v]);
		return depthMap.isInsideWithBorder<float,3>(pti[v]);
	}
	inline bool CheckNormal(const Point3& faceCenter) {
		// skip face if the (cos) angle between
		// the face normal and the face to view vector is negative
		if (faceCenter.dot(normalPlane) >= ZEROTOLERANCE<REAL>())
			return false;
		// prepare vector used to compute the depth during rendering
		normalPlane /= normalPlane.dot(ptc[0]);
		return true;
	}
	void Project(const Mesh::Face& facet) {
		// project face vertices to image plane
		for (int v=0; v<3; ++v) {
			// skip face if not completely inside
			if (!static_cast<DERIVED*>(this)->ProjectVertex(vertices[facet[v]], v))
				return;
		}
		// compute the face center, which is also the view to face vector
		// (cause the face is in camera view space)
		const Point3 faceCenter((ptc[0]+ptc[1]+ptc[2])/3);
		// skip face if the (cos) angle between
		// the view to face vector and the view direction is negative
		if (faceCenter.z <= REAL(0))
			return;
		// compute the plane defined by the 3 points
		const Point3 edge1(ptc[1]-ptc[0]);
		const Point3 edge2(ptc[2]-ptc[0]);
		normalPlane = edge1.cross(edge2);
		// check the face is facing the camera and
		// prepare vector used to compute the depth during rendering
		if (!static_cast<DERIVED*>(this)->CheckNormal(faceCenter))
			return;
		// draw triangle and for each pixel compute depth as the ray intersection with the plane
		Image8U3::RasterizeTriangle(pti[0], pti[1], pti[2], *this);
	}

	void Raster(const ImageRef& pt) {
		if (!depthMap.isInsideWithBorder<float,3>(pt))
			return;
		const Depth z((Depth)INVERT(normalPlane.dot(camera.TransformPointI2C(Point2(pt)))));
		ASSERT(z > 0);
		Depth& depth = depthMap(pt);
		if (depth == 0 || depth > z)
			depth = z;
	}
	inline void operator()(const ImageRef& pt) {
		static_cast<DERIVED*>(this)->Raster(pt);
	}
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_MESH_H_
