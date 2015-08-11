/*
* SceneTexture.cpp
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
#include "Scene.h"
#include "RectsBinPack.h"
// inference algorithm 
#include "../Math/LBP.h"
// find connected components
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define TEXTURE_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

typedef Mesh::Vertex Vertex;
typedef Mesh::VIndex VIndex;
typedef Mesh::Face Face;
typedef Mesh::FIndex FIndex;
typedef Mesh::TexCoord TexCoord;

struct MeshTexture {
	// store necessary data about a view
	struct View {
		Image32F imageGradMag; // image pixel gradient magnitudes
	};
	typedef cList<View,const View&,2> ViewsArr;

	// used to render the surface to a view camera
	typedef TImage<cuint32_t> FaceMap;
	struct RasterMeshData {
		const Camera& P;
		FaceMap& faceMap;
		DepthMap& depthMap;
		Point3 normalPlane;
		FIndex idxFace;
		inline void operator()(const ImageRef& pt) {
			if (!depthMap.isInside(pt))
				return;
			const float z((float)INVERT(normalPlane.dot(P.TransformPointI2C(Point2(pt)))));
			ASSERT(z > 0);
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				faceMap(pt) = idxFace;
			}
		}
	};

	// used to represent a pixel color
	typedef Point3f Color;
	typedef CLISTDEF0(Color) Colors;

	// used to store info about a face (view, quality)
	struct FaceData {
		VIndex idxView;// the view seeing this face
		float quality; // how well the face is seen by this view
	};
	typedef cList<FaceData,const FaceData&,0,8,uint32_t> FaceDataArr; // store information about one face seen from several views
	typedef cList<FaceDataArr,const FaceDataArr&,2,1024,FIndex> FaceDataViewArr; // store data for all the faces of the mesh

	// used to assign a view to a face
	typedef uint32_t Label;
	typedef cList<Label,Label,0,1024,FIndex> LabelArr;

	// represents a texture patch
	struct TexturePatch {
		Label label; // view index
		Mesh::FaceIdxArr faces; // indices of the faces contained by the patch
		RectsBinPack::Rect rect; // the bounding box in the view containing the patch
	};
	typedef cList<TexturePatch,const TexturePatch&,1,1024,FIndex> TexturePatchArr;

	// used to optimize texture patches
	struct SeamVertex {
		struct Patch {
			struct Edge {
				uint32_t idxSeamVertex; // the other vertex of this edge
				FIndex idxFace; // the face containing this edge in this patch

				inline Edge() {}
				inline Edge(uint32_t _idxSeamVertex) : idxSeamVertex(_idxSeamVertex) {}
				inline bool operator == (uint32_t _idxSeamVertex) const {
					return (idxSeamVertex == _idxSeamVertex);
				}
			};
			typedef CLISTDEF0IDX(Edge, uint32_t) Edges;

			uint32_t idxPatch; // the patch containing this vertex
			Point2f proj; // the projection of this vertex in this patch
			Edges edges; // the edges starting from this vertex, contained in this patch (exactly two for manifold meshes)

			inline Patch() {}
			inline Patch(uint32_t _idxPatch) : idxPatch(_idxPatch) {}
			inline bool operator == (uint32_t _idxPatch) const {
				return (idxPatch == _idxPatch);
			}
		};
		typedef CLISTDEFIDX(Patch, uint32_t) Patches;

		VIndex idxVertex; // the index of this vertex
		Patches patches; // the patches meeting at this vertex (two or more)

		inline SeamVertex() {}
		inline SeamVertex(uint32_t _idxVertex) : idxVertex(_idxVertex) {}
		inline bool operator == (uint32_t _idxVertex) const {
			return (idxVertex == _idxVertex);
		}
		Patch& GetPatch(uint32_t idxPatch) {
			const uint32_t idx(patches.Find(idxPatch));
			if (idx == NO_ID)
				return patches.AddConstruct(idxPatch);
			return patches[idx];
		}
		inline void SortByPatchIndex(IndexArr& indices) const {
			indices.Resize(patches.GetSize());
			std::iota(indices.Begin(), indices.End(), 0);
			std::sort(indices.Begin(), indices.End(), [&](const IndexArr::Type i0, const IndexArr::Type i1) -> bool {
				return patches[i0].idxPatch < patches[i1].idxPatch;
			});
		}
	};
	typedef CLISTDEFIDX(SeamVertex, uint32_t) SeamVertices;

	// used to iterate vertex labels
	struct PatchIndex {
		bool bIndex;
		union {
			uint32_t idxPatch;
			uint32_t idxSeamVertex;
		};
	};
	typedef CLISTDEF0(PatchIndex) PatchIndices;
	struct VertexPatchIterator {
		uint32_t idx;
		uint32_t idxPatch;
		const SeamVertex::Patches* pPatches;
		inline VertexPatchIterator(const PatchIndex& patchIndex, const SeamVertices& seamVertices) : idx(NO_ID) {
			if (patchIndex.bIndex) {
				pPatches = &seamVertices[patchIndex.idxSeamVertex].patches;
			} else {
				idxPatch = patchIndex.idxPatch;
				pPatches = NULL;
			}
		}
		inline operator uint32_t () const {
			return idxPatch;
		}
		inline bool Next() {
			if (pPatches == NULL)
				return (idx++ == NO_ID);
			if (++idx >= pPatches->GetSize())
				return false;
			idxPatch = (*pPatches)[idx].idxPatch;
			return true;
		}
	};


public:
	MeshTexture(Scene& _scene, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640);
	~MeshTexture();

	bool InitImages();

	void ListVertexFaces();

	void ListCameraFaces(FaceDataViewArr&);

	bool FaceOutlierDetection(FaceDataArr& faceDatas) const;

	void FaceViewSellection();

	void GenerateTexture();

	template <typename PIXEL>
	static inline PIXEL RGB2YCBCR(const PIXEL& v) {
		typedef typename PIXEL::Type T;
		return PIXEL(
			v[0] * T(0.299) + v[1] * T(0.587) + v[2] * T(0.114),
			v[0] * T(-0.168736) + v[1] * T(-0.331264) + v[2] * T(0.5) + T(128),
			v[0] * T(0.5) + v[1] * T(-0.418688) + v[2] * T(-0.081312) + T(128)
		);
	}
	template <typename PIXEL>
	static inline PIXEL YCBCR2RGB(const PIXEL& v) {
		typedef typename PIXEL::Type T;
		const T v1(v[1] - T(128));
		const T v2(v[2] - T(128));
		return PIXEL(
			v[0]/* * T(1) + v1 * T(0)*/ + v2 * T(1.402),
			v[0]/* * T(1)*/ + v1 * T(-0.34414) + v2 * T(-0.71414),
			v[0]/* * T(1)*/ + v1 * T(1.772)/* + v2 * T(0)*/
		);
	}


public:
	const unsigned nResolutionLevel; // how many times to scale down the images before mesh optimization
	const unsigned nMinResolution; // how many times to scale down the images before mesh optimization

	// store found texture patches
	TexturePatchArr texturePatches;

	// used to compute the seam leveling
	PairIdxArr seamEdges; // the (face-face) edges connecting different texture patches
	Mesh::FaceIdxArr components; // for each face, stores the texture patch index to which belongs
	IndexArr mapIdxPatch; // remap texture patch indices after invalid patches removal

	// valid the entire time
	Mesh::VertexFacesArr& vertexFaces; // for each vertex, the list of faces containing it
	BoolArr& vertexBoundary; // for each vertex, stores if it is at the boundary or not
	Mesh::TexCoordArr& faceTexcoords; // for each face, the texture-coordinates of the vertices
	Image8U3& textureDiffuse; // texture containing the diffuse color

	// constant the entire time
	Mesh::VertexArr& vertices;
	Mesh::FaceArr& faces;
	ImageArr& images;
	ViewsArr views; // views' data

	Scene& scene; // the mesh vertices and faces
};

MeshTexture::MeshTexture(Scene& _scene, unsigned _nResolutionLevel, unsigned _nMinResolution)
	:
	nResolutionLevel(_nResolutionLevel),
	nMinResolution(_nMinResolution),
	vertices(_scene.mesh.vertices),
	faces(_scene.mesh.faces),
	vertexFaces(_scene.mesh.vertexFaces),
	vertexBoundary(_scene.mesh.vertexBoundary),
	faceTexcoords(_scene.mesh.faceTexcoords),
	textureDiffuse(_scene.mesh.textureDiffuse),
	images(_scene.images),
	scene(_scene)
{
}
MeshTexture::~MeshTexture()
{
	vertexFaces.Release();
	vertexBoundary.Release();
}

// load and initialize all images and
// compute the gradient magnitudes for each input image
bool MeshTexture::InitImages()
{
	views.Resize(images.GetSize());
	typedef float real;
	Image32F img;
	cv::Mat grad[2];
	Eigen::Matrix<real,Eigen::Dynamic,1> mGrad[2], mMag;
	#ifdef TEXTURE_USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for private(img, grad, mGrad, mMag)
	for (int_t iID=0; iID<(int_t)images.GetSize(); ++iID) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
		const uint32_t ID((uint32_t)iID);
	#else
	FOREACH(ID, images) {
	#endif
		Image& imageData = images[ID];
		if (!imageData.IsValid())
			continue;
		// load image
		unsigned level(nResolutionLevel);
		const unsigned imageSize(imageData.RecomputeMaxResolution(level, nMinResolution));
		if ((imageData.image.empty() || MAXF(imageData.width,imageData.height) != imageSize) && FAILED(imageData.ReloadImage(imageSize))) {
			#ifdef TEXTURE_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			#else
			return false;
			#endif
		}
		imageData.UpdateCamera(scene.platforms);
		// compute gradient magnitude
		imageData.image.toGray(img, cv::COLOR_BGR2GRAY, true);
		const size_t len((size_t)img.area());
		mGrad[0].resize(len);
		grad[0] = cv::Mat(img.rows, img.cols, cv::DataType<real>::type, (void*)mGrad[0].data());
		mGrad[1].resize(len);
		grad[1] = cv::Mat(img.rows, img.cols, cv::DataType<real>::type, (void*)mGrad[1].data());
		#if 1
		cv::Sobel(img, grad[0], cv::DataType<real>::type, 1, 0, 3, 1.0/8.0);
		cv::Sobel(img, grad[1], cv::DataType<real>::type, 0, 1, 3, 1.0/8.0);
		#elif 1
		const TMatrix<real,3,5> kernel(CreateDerivativeKernel3x5());
		cv::filter2D(img, grad[0], cv::DataType<real>::type, kernel);
		cv::filter2D(img, grad[1], cv::DataType<real>::type, kernel.t());
		#else
		const TMatrix<real,5,7> kernel(CreateDerivativeKernel5x7());
		cv::filter2D(img, grad[0], cv::DataType<real>::type, kernel);
		cv::filter2D(img, grad[1], cv::DataType<real>::type, kernel.t());
		#endif
		mMag = (mGrad[0].cwiseAbs2()+mGrad[1].cwiseAbs2()).cwiseSqrt();
		cv::Mat(img.rows, img.cols, cv::DataType<real>::type, (void*)mMag.data()).copyTo(views[ID].imageGradMag);
	}
	#ifdef TEXTURE_USE_OPENMP
	return !bAbort;
	#else
	return true;
	#endif
}

// extract array of triangles incident to each vertex
// and check each vertex if it is at the boundary or not
void MeshTexture::ListVertexFaces()
{
	scene.mesh.EmptyExtra();
	scene.mesh.ListIncidenteFaces();
	scene.mesh.ListBoundaryVertices();
}

// extract array of faces viewed by each image
void MeshTexture::ListCameraFaces(FaceDataViewArr& facesDatas)
{
	facesDatas.Resize(faces.GetSize());
	typedef std::unordered_set<FIndex> CameraFaces;
	CameraFaces cameraFaces;
	struct FacesInserter {
		FacesInserter(const Mesh::VertexFacesArr& _vertexFaces, CameraFaces& _cameraFaces)
			: vertexFaces(_vertexFaces), cameraFaces(_cameraFaces) {}
		inline void operator() (IDX idxVertex) {
			const Mesh::FaceIdxArr& vertexTris = vertexFaces[idxVertex];
			FOREACHPTR(pTri, vertexTris)
				cameraFaces.emplace(*pTri);
		}
		inline void operator() (const IDX* idices, size_t size) {
			FOREACHRAWPTR(pIdxVertex, idices, size)
				operator()(*pIdxVertex);
		}
		const Mesh::VertexFacesArr& vertexFaces;
		CameraFaces& cameraFaces;
	};
	typedef TOctree<Mesh::VertexArr,float,3> Octree;
	const Octree octree(vertices);
	#if 1 && !defined(_RELEASE)
	Octree::DEBUGINFO_TYPE info;
	octree.GetDebugInfo(&info);
	Octree::LogDebugInfo(info);
	#endif
	FaceMap faceMap;
	DepthMap depthMap;
	Point3 ptc[3]; Point2f pti[3];
	FOREACH(idxView, images) {
		const Image& imageData = images[idxView];
		if (!imageData.IsValid())
			continue;
		// select faces inside view frustum
		typedef TFrustum<float,5> Frustum;
		FacesInserter inserter(vertexFaces, cameraFaces);
		const Frustum frustum(Frustum::MATRIX3x4(((const PMatrix::EMatMap)imageData.camera.P).cast<float>()), (float)imageData.width, (float)imageData.height);
		octree.Traverse(frustum, inserter);
		// project all triangles in this view and keep the closest ones
		const Camera& camera = imageData.camera;
		faceMap.create(imageData.height, imageData.width);
		faceMap.memset((uint8_t)NO_ID);
		depthMap.create(imageData.height, imageData.width);
		depthMap.memset(0);
		RasterMeshData data = {camera, faceMap, depthMap};
		for (auto idxFace : cameraFaces) {
			const Face& facet = faces[idxFace];
			for (unsigned v=0; v<3; ++v) {
				const Vertex& pt = vertices[facet[v]];
				ptc[v] = camera.TransformPointW2C(CastReal(pt));
				pti[v] = camera.TransformPointC2I(ptc[v]);
				// skip face if not completely inside
				if (!depthMap.isInsideWithBorder<float,3>(pti[v]))
					goto CONTIUE2NEXTFACE;
			}
			{
			// compute the face center, which is also the view to face vector
			// (cause the face is in camera view space)
			const Point3 faceCenter((ptc[0]+ptc[1]+ptc[2])/3);
			// skip face if the (cos) angle between
			// the view to face vector and the view direction is negative
			if (faceCenter.z < 0)
				continue;
			// compute the plane defined by the 3 points
			const Point3 edge1(ptc[1]-ptc[0]);
			const Point3 edge2(ptc[2]-ptc[0]);
			data.normalPlane = edge1.cross(edge2);
			// skip face if the (cos) angle between
			// the face normal and the face to view vector is negative
			if (faceCenter.dot(data.normalPlane) > 0)
				continue;
			// prepare vector used to compute the depth during rendering
			data.normalPlane *= INVERT(data.normalPlane.dot(ptc[0]));
			// draw triangle and for each pixel compute depth as the ray intersection with the plane
			data.idxFace = idxFace;
			Image8U3::RasterizeTriangle(pti[0], pti[1], pti[2], data);
			}
			CONTIUE2NEXTFACE:;
		}
		// compute the projection area of visible faces
		const View& view = views[idxView];
		for (int j=0; j<faceMap.rows; ++j) {
			for (int i=0; i<faceMap.cols; ++i) {
				const FIndex& idxFace = faceMap(j,i);
				ASSERT((idxFace == NO_ID && depthMap(j,i) == 0) || (idxFace != NO_ID && depthMap(j,i) > 0));
				if (idxFace == NO_ID)
					continue;
				FaceDataArr& faceDatas = facesDatas[idxFace];
				if (faceDatas.IsEmpty() || faceDatas.Last().idxView != idxView) {
					// create new face-data
					FaceData& faceData = faceDatas.AddEmpty();
					faceData.idxView = idxView;
					faceData.quality = view.imageGradMag(j,i);
				} else {
					// update face-data
					ASSERT(!faceDatas.IsEmpty());
					FaceData& faceData = faceDatas.Last();
					ASSERT(faceData.idxView == idxView);
					faceData.quality += view.imageGradMag(j,i);
				}
			}
		}
	}

	// release image gradient magnitudes
	views.Release();
}

// Potts model as smoothness function
LBPInference::EnergyType STCALL SmoothnessPotts(LBPInference::NodeID, LBPInference::NodeID, LBPInference::LabelID l1, LBPInference::LabelID l2) {
	return l1 == l2 && l1 != 0 && l2 != 0 ? LBPInference::EnergyType(0) : LBPInference::EnergyType(LBPInference::MaxEnergy);
}

void MeshTexture::FaceViewSellection()
{
	// extract array of triangles incident to each vertex
	ListVertexFaces();

	// create texture patches
	{
		// list all views for each face
		FaceDataViewArr facesDatas;
		ListCameraFaces(facesDatas);

		// create faces graph
		typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
		typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
		typedef boost::graph_traits<Graph>::edge_descriptor Edge;
		typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
		typedef boost::graph_traits<Graph>::out_edge_iterator EdgeOutIter;
		Graph graph;
		{
			Mesh::FaceIdxArr afaces;
			std::unordered_set<PairIdx::PairIndex> setEdges;
			setEdges.reserve(faces.GetSize()*2);
			FOREACH(idxFace, faces) {
				scene.mesh.GetFaceFaces(idxFace, afaces);
				ASSERT(ISINSIDE((int)afaces.GetSize(), 1, 4));
				FOREACHPTR(pIdxFace, afaces) {
					const FIndex idxFaceAdj = *pIdxFace;
					if (idxFace >= idxFaceAdj)
						continue;
					if (facesDatas[idxFace].IsEmpty() || facesDatas[idxFaceAdj].IsEmpty()) {
						seamEdges.AddConstruct(idxFace, idxFaceAdj);
						continue;
					}
					if (setEdges.insert(PairIdx(idxFace, idxFaceAdj).idx).second)
						boost::add_edge(idxFace, *pIdxFace, graph);
				}
				afaces.Empty();
			}
		}

		// assign the best view to each face
		LabelArr labels(faces.GetSize());
		components.Resize(faces.GetSize());
		{
			// find connected components
			const FIndex nComponents(boost::connected_components(graph, components.Begin()));

			// map face ID from global to component space
			typedef LBPInference::NodeID NodeID;
			typedef cList<NodeID, NodeID, 0, 128, NodeID> NodeIDs;
			NodeIDs nodeIDs(faces.GetSize());
			NodeIDs sizes(nComponents);
			sizes.Memset(0);
			FOREACH(c, components)
				nodeIDs[c] = sizes[components[c]]++;

			// normalize quality values
			float maxQuality(0);
			FOREACHPTR(pFaceDatas, facesDatas) {
				const FaceDataArr& faceDatas = *pFaceDatas;
				FOREACHPTR(pFaceData, faceDatas)
					if (maxQuality < pFaceData->quality)
						maxQuality = pFaceData->quality;
			}
			Histogram32F hist(std::make_pair(0.f, maxQuality), 1000);
			FOREACHPTR(pFaceDatas, facesDatas) {
				const FaceDataArr& faceDatas = *pFaceDatas;
				FOREACHPTR(pFaceData, faceDatas)
					hist.Add(pFaceData->quality);
			}
			const float normQuality(hist.GetApproximatePermille(0.95f));

			// initialize inference structures
			cList<LBPInference, const LBPInference&, 1, 8, FIndex> inferences(nComponents);
			{
				FOREACH(s, sizes) {
					const NodeID numNodes(sizes[s]);
					ASSERT(numNodes > 0);
					if (numNodes <= 1)
						continue;
					LBPInference& inference = inferences[s];
					inference.SetNumNodes(numNodes);
					inference.SetSmoothCost(SmoothnessPotts);
				}
				EdgeOutIter ei, eie;
				FOREACH(f, faces) {
					LBPInference& inference = inferences[components[f]];
					for (boost::tie(ei, eie) = boost::out_edges(f, graph); ei != eie; ++ei) {
						ASSERT(f == (FIndex)ei->m_source);
						const FIndex fAdj((FIndex)ei->m_target);
						ASSERT(components[f] == components[fAdj]);
						if (f < fAdj) // add edges only once
							inference.SetNeighbors(nodeIDs[f], nodeIDs[fAdj]);
					}
				}
			}

			// set data costs
			{
				// set costs for label 0 (undefined)
				FOREACH(s, inferences) {
					LBPInference& inference = inferences[s];
					if (inference.GetNumNodes() == 0)
						continue;
					const NodeID numNodes(sizes[s]);
					for (NodeID nodeID=0; nodeID<numNodes; ++nodeID)
						inference.SetDataCost((Label)0, nodeID, LBPInference::MaxEnergy);
				}
				// set data costs for all labels (except label 0 - undefined)
				FOREACH(f, facesDatas) {
					const FaceDataArr& faceDatas = facesDatas[f];
					const NodeID nodeID(nodeIDs[f]);
					LBPInference& inference = inferences[components[f]];
					if (inference.GetNumNodes() == 0)
						continue;
					FOREACHPTR(pFaceData, faceDatas) {
						const FaceData& faceData = *pFaceData;
						const Label label((Label)faceData.idxView+1);
						const float normalizedQuality(faceData.quality>=normQuality ? 1.f : faceData.quality/normQuality);
						const float dataCost((1.f-normalizedQuality)*LBPInference::MaxEnergy);
						inference.SetDataCost(label, nodeID, dataCost);
					}
				}
			}

			// assign the optimal view (label) to each face
			// (label 0 is reserved as undefined)
			FOREACH(s, inferences) {
				LBPInference& inference = inferences[s];
				if (inference.GetNumNodes() == 0)
					continue;
				inference.Optimize();
			}
			// extract resulting labeling
			labels.Memset(0xFF);
			FOREACH(l, labels) {
				LBPInference& inference = inferences[components[l]];
				if (inference.GetNumNodes() == 0)
					continue;
				const Label label(inference.GetLabel(nodeIDs[l]));
				ASSERT(label >= 0 && label < images.GetSize()+1);
				if (label > 0)
					labels[l] = label-1;
			}
		}

		// create texture patches
		{
			// divide graph in sub-graphs of connected faces having the same label
			EdgeIter ei, eie;
			const PairIdxArr::IDX startLabelSeamEdges(seamEdges.GetSize());
			for (boost::tie(ei, eie) = boost::edges(graph); ei != eie; ++ei) {
				const FIndex fSource((FIndex)ei->m_source);
				const FIndex fTarget((FIndex)ei->m_target);
				ASSERT(components[fSource] == components[fTarget]);
				if (labels[fSource] != labels[fTarget])
					seamEdges.AddConstruct(fSource, fTarget);
			}
			for (const PairIdx *pEdge=seamEdges.Begin()+startLabelSeamEdges, *pEdgeEnd=seamEdges.End(); pEdge!=pEdgeEnd; ++pEdge)
				boost::remove_edge(pEdge->i, pEdge->j, graph);

			// find connected components: texture patches
			const FIndex nComponents(boost::connected_components(graph, components.Begin()));

			// create texture patches;
			// last texture patch contains all faces with no texture
			LabelArr sizes(nComponents);
			sizes.Memset(0);
			FOREACH(c, components)
				++sizes[components[c]];
			texturePatches.Resize(nComponents+1);
			texturePatches.Last().label = NO_ID;
			FOREACH(f, faces) {
				const Label label(labels[f]);
				const FIndex c(components[f]);
				TexturePatch& texturePatch = texturePatches[c];
				ASSERT(texturePatch.label == label || texturePatch.faces.IsEmpty());
				if (label == NO_ID) {
					texturePatch.label = NO_ID;
					texturePatches.Last().faces.Insert(f);
				} else {
					if (texturePatch.faces.IsEmpty()) {
						texturePatch.label = label;
						texturePatch.faces.Reserve(sizes[c]);
					}
					texturePatch.faces.Insert(f);
				}
			}
			// remove all patches with invalid label (except the last one)
			// and create the map from the old index to the new one
			mapIdxPatch.Resize(nComponents);
			std::iota(mapIdxPatch.Begin(), mapIdxPatch.End(), 0);
			for (FIndex t = nComponents; t-- > 0; ) {
				if (texturePatches[t].label == NO_ID) {
					texturePatches.RemoveAtMove(t);
					mapIdxPatch.RemoveAtMove(t);
				}
			}
			const unsigned numPatches(texturePatches.GetSize()-1);
			uint32_t idxPatch(0);
			for (IndexArr::IDX i=0; i<mapIdxPatch.GetSize(); ++i) {
				while (i < mapIdxPatch[i])
					mapIdxPatch.InsertAt(i++, numPatches);
				mapIdxPatch[i] = idxPatch++;
			}
			while (mapIdxPatch.GetSize() <= nComponents)
				mapIdxPatch.Insert(numPatches);
		}
	}
}


void MeshTexture::GenerateTexture()
{
	// project patches in the corresponding view and compute texture-coordinates and bounding-box
	const int border(2);
	const unsigned numPatches(texturePatches.GetSize()-1);
	faceTexcoords.Resize(faces.GetSize()*3);
	for (TexturePatch *pTexturePatch=texturePatches.Begin(), *pTexturePatchEnd=texturePatches.End()-1; pTexturePatch<pTexturePatchEnd; ++pTexturePatch) {
		TexturePatch& texturePatch = *pTexturePatch;
		const Image& imageData = images[texturePatch.label];
		// project vertices and  compute bounding-box
		AABB2f aabb(true);
		FOREACHPTR(pIdxFace, texturePatch.faces) {
			const FIndex f(*pIdxFace);
			const Face& face = faces[f];
			TexCoord* texcoords = faceTexcoords.Begin()+f*3;
			for (int i=0; i<3; ++i) {
				texcoords[i] = imageData.camera.ProjectPointP(vertices[face[i]]);
				ASSERT(imageData.image.isInsideWithBorder(texcoords[i], border));
				aabb.InsertFull(texcoords[i]);
			}
		}
		// compute relative texture coordinates
		ASSERT(imageData.image.isInside(Point2f(aabb.ptMin)));
		ASSERT(imageData.image.isInside(Point2f(aabb.ptMax)));
		texturePatch.rect.x = FLOOR2INT(aabb.ptMin[0])-border;
		texturePatch.rect.y = FLOOR2INT(aabb.ptMin[1])-border;
		texturePatch.rect.width = CEIL2INT(aabb.ptMax[0]-aabb.ptMin[0])+border*2;
		texturePatch.rect.height = CEIL2INT(aabb.ptMax[1]-aabb.ptMin[1])+border*2;
		ASSERT(imageData.image.isInside(texturePatch.rect.tl()));
		ASSERT(imageData.image.isInside(texturePatch.rect.br()));
	}
	{
		// init last patch to point to a small uniform color patch
		TexturePatch& texturePatch = texturePatches[numPatches];
		const int sizePatch(border*2+1);
		texturePatch.rect = cv::Rect(0,0, sizePatch,sizePatch);
		FOREACHPTR(pIdxFace, texturePatch.faces) {
			const FIndex f(*pIdxFace);
			TexCoord* texcoords = faceTexcoords.Begin()+f*3;
			for (int i=0; i<3; ++i)
				texcoords[i] = TexCoord(0.5f, 0.5f);
		}
	}

	// merge texture patches with overlapping rectangles
	for (unsigned i=0; i<texturePatches.GetSize()-2; ++i) {
		TexturePatch& texturePatchBig = texturePatches[i];
		for (unsigned j=1; j<texturePatches.GetSize()-1; ++j) {
			if (i == j)
				continue;
			TexturePatch& texturePatchSmall = texturePatches[j];
			if (texturePatchBig.label != texturePatchSmall.label)
				continue;
			if (!IsContainedIn(texturePatchSmall.rect, texturePatchBig.rect))
				continue;
			texturePatchBig.faces.JoinRemove(texturePatchSmall.faces);
			texturePatches.RemoveAtMove(j--);
		}
	}

	// create texture
	{
		// arrange texture patches to fit the smallest possible texture image
		RectsBinPack::RectArr rects(texturePatches.GetSize());
		FOREACH(i, texturePatches)
			rects[i] = texturePatches[i].rect;
		int textureSize(RectsBinPack::ComputeTextureSize(rects));
		// increase texture size till all patches fit
		while (true) {
			RectsBinPack pack(textureSize, textureSize);
			if (pack.Insert(rects))
				break;
			textureSize *= 2;
		}

		// create texture image
		const float invNorm(1.f/(float)(textureSize-1));
		textureDiffuse.create(textureSize, textureSize);
		textureDiffuse.setTo(cv::Scalar(39, 127, 255));
		TexCoord offset;
		int x, y;
		#ifdef TEXTURE_USE_OPENMP
		#pragma omp parallel for private(offset, x, y)
		for (int_t i=0; i<(int_t)texturePatches.GetSize(); ++i) {
		#else
		FOREACH(i, texturePatches) {
		#endif
			const uint32_t idxPatch((uint32_t)i);
			const TexturePatch& texturePatch = texturePatches[idxPatch];
			const RectsBinPack::Rect& rect = rects[idxPatch];
			// copy patch image
			ASSERT((rect.width == texturePatch.rect.width && rect.height == texturePatch.rect.height) ||
				   (rect.height == texturePatch.rect.width && rect.width == texturePatch.rect.height));
			if (texturePatch.label != NO_ID) {
				const Image& imageData = images[texturePatch.label];
				cv::Mat patch(imageData.image(texturePatch.rect));
				if (rect.width != texturePatch.rect.width) {
					// flip patch and texture-coordinates
					patch = patch.t();
					x = 1; y = 0;
					offset.x = float(rect.x - texturePatch.rect.y);
					offset.y = float(rect.y - texturePatch.rect.x);
				} else {
					x = 0; y = 1;
					offset = rect.tl()-texturePatch.rect.tl();
				}
				patch.copyTo(textureDiffuse(rect));
			} else {
				x = 0; y = 1;
				offset = rect.tl();
			}
			// compute final texture coordinates
			FOREACHPTR(pIdxFace, texturePatch.faces) {
				const FIndex f(*pIdxFace);
				TexCoord* texcoords = faceTexcoords.Begin()+f*3;
				for (int v=0; v<3; ++v) {
					TexCoord& texcoord = texcoords[v];
					// translate, normalize and flip Y axis
					texcoord = TexCoord(
						(texcoord[x]+(float)offset.x)*invNorm,
						1.f-(texcoord[y]+(float)offset.y)*invNorm
					);
				}
			}
		}
	}
}

// texture mesh
bool Scene::TextureMesh(unsigned nResolutionLevel, unsigned nMinResolution)
{
	MeshTexture texture(*this, nResolutionLevel, nMinResolution);

	// init images
	{
		TD_TIMER_STARTD();
		texture.InitImages();
		DEBUG_EXTRA("Initializing images completed: %u images (%s)", images.GetSize(), TD_TIMER_GET_FMT().c_str());
	}

	// assign the best view to each face 
	{
		TD_TIMER_STARTD();
		texture.FaceViewSellection();
		DEBUG_EXTRA("Assigning the best view to each face completed: %u faces (%s)", mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());
	}

	// generate the texture image and atlas 
	{
		TD_TIMER_STARTD();
		texture.GenerateTexture();
		DEBUG_EXTRA("Generating texture atlas and image completed: %u patches, %u image size (%s)", texture.texturePatches.GetSize(), mesh.textureDiffuse.width(), TD_TIMER_GET_FMT().c_str());
	}

	return _OK;
} // TextureMesh
/*----------------------------------------------------------------*/
