/*
* SceneRefine.cpp
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

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define REFINE_USE_OPENMP
#endif

// uncomment to ensure edge size and improve vertex valence
// (should enable more stable flow)
#define MESHOPT_ENSUREEDGESIZE 1 // 0 - at all resolution

// uncomment to project data gradient term on the vertex normal
// (should filter out the various noise in the photo-consistency gradient term)
#define MESHOPT_PHOTOGRADNORMAL

// uncomment to use constant z-buffer bias
// (should be enough, as the numerical error does not depend on the depth)
#define MESHOPT_DEPTHCONSTBIAS 0.05f

// uncomment to use robust data term
// (somewhat slower and not seem to make much of a difference)
#define MESHOPT_ROBUST

// uncomment to refine mesh using also a smoothing term
// (somewhat slower, but increases a lot the quality)
#define MESHOPT_SMOOTH

// uncomment to use for refine mesh only vertices with the entire ring-1 triangle-fun visible in the pair images
// (somewhat slower, and does not seem to increase the quality)
//#define MESHOPT_VERTEXFULLRING

// uncomment to refine mesh using numerical method
// (somewhat slower and should be less accurate compared to the analytical method)
//#define MESHOPT_NUMERICAL

// uncomment to generate more debug info during mesh optimization
// (quite fast, but uses more memory)
//#define MESHOPT_DEBUG 2


// S T R U C T S ///////////////////////////////////////////////////

typedef Mesh::Vertex Vertex;
typedef Mesh::VIndex VIndex;
typedef Mesh::Face Face;
typedef Mesh::FIndex FIndex;

struct MeshRefine {
	// store necessary data about a view
	struct View {
		typedef TImage<Point2f> ImageGrad;
		Image32F image; // image pixels
		ImageGrad imageGrad; // image pixel gradients
	};
	typedef SEACAVE::cList<View,const View&,2> ViewsArr;

	// used to render a mesh for optimization
	typedef TImage<cuint32_t> VertexMap;
	typedef SEACAVE::cList<VertexMap,const VertexMap&,2> VertexMapArr;
	typedef TImage<cuint32_t> FaceMap;
	typedef SEACAVE::cList<FaceMap,const FaceMap&,2> FaceMapArr;
	struct ViewVertex {
		uint32_t area; // area of the faces adjacent to this vertex in this image (in pixels)
		uint32_t size; // number of pixels fetched for this vertex
		float* texA; // pixel values corresponding to image A
		float* texB; // pixel values corresponding to image B back-projected on image A
		inline ViewVertex() : area(0), size(0), texA(NULL), texB(NULL) {}
		inline ~ViewVertex() { delete[] texA; }
		inline bool IsEmpty() const { return size == 0; }
		inline bool IsValid() const { return size >= 6; } // minimum number of pixels a vertex patch should have to be considered
		inline void Init() {
			if (area == 0)
				return;
			// allocate space to store two texture patches
			texA = new float[area*2];
			texB = texA + area;
		}
		inline void InitGradient() {
			if (area == 0)
				return;
			// allocate space to store two texture patches
			// and three slightly modified texture patches of patch B
			texA = new float[area*5];
			texB = texA + area;
		}
		inline void InitIndices() {
			if (area == 0)
				return;
			// allocate space to store two texture patches
			// and one array of indices in the image A (stored as uint32_t = i*width+j
			texA = new float[area*3];
			texB = texA + area;
		}
		inline void Insert(float pixelA, float pixelB) {
			ASSERT(area != 0);
			texA[size] = pixelA;
			texB[size] = pixelB;
			++size;
		}
		inline void Insert(uint32_t i, float pixelB) {
			ASSERT(area != 0);
			texB[area*(i+1)+size] = pixelB;
		}
		inline void Insert(float pixelA, float pixelB, uint32_t idxA) {
			ASSERT(area != 0);
			texA[size] = pixelA;
			texB[size] = pixelB;
			((uint32_t*)(texB+area))[size] = idxA;
			++size;
		}
		inline float* GetBufferA() const {
			ASSERT(area != 0);
			return texA;
		}
		inline float* GetBufferB() const {
			ASSERT(area != 0);
			return texB;
		}
		inline float* GetBufferB(uint32_t i) const {
			ASSERT(area != 0);
			return texB + area*(i+1);
		}
		inline uint32_t GetIndexA(uint32_t i) const {
			ASSERT(area != 0);
			return ((uint32_t*)(texB+area))[i];
		}
		inline float Score() {
			ASSERT(size <= area && IsValid());
			#if 1
			const float normSqA(normSq<float,float>(texA, size));
			const float normSqB(normSq<float,float>(texB, size));
			#else
			const float normSqA(normSqZeroMean<float,float>(texA, size));
			const float normSqB(normSqZeroMean<float,float>(texB, size));
			#endif
			const float den(SQRT(normSqA*normSqB));
			if (den < FLT_EPSILON)
				return 0.5f;
			const float ncc(CLAMP(1.f-dot<float,float>(texA, texB, size)/den, 0.f, 2.f));
			#ifdef MESHOPT_ROBUST
			return robustincc(ncc);
			#else
			return ncc;
			#endif
		}
		inline float PreScore() {
			ASSERT(size <= area && IsValid());
			#if 1
			return normSq<float,float>(texA, size);
			#else
			return normSqZeroMean<float,float>(texA, size);
			#endif
		}
		inline float Score(float normSqA, float* pixB) {
			ASSERT(size <= area && IsValid());
			ASSERT(normSqA > 0);
			#if 1
			const float normSqB(normSq<float,float>(pixB, size));
			#else
			const float normSqB(normSqZeroMean<float,float>(pixB, size));
			#endif
			if (normSqB < FLT_EPSILON)
				return 0.5f;
			const float ncc(CLAMP(1.f-dot<float,float>(texA, pixB, size)/SQRT(normSqA*normSqB), 0.f, 2.f));
			#ifdef MESHOPT_ROBUST
			return robustincc(ncc);
			#else
			return ncc;
			#endif
		}
		struct DerivativeData {
			float NCC;
			float normSqA;
			float invSQRTnormSqAnormSqB;
		};
		inline float Score(DerivativeData& dData) {
			ASSERT(size <= area && IsValid());
			dData.normSqA = normSq<float,float>(texA, size);
			const float normSqB(normSq<float,float>(texB, size));
			const float normSqAnormSqB(dData.normSqA*normSqB);
			if (normSqAnormSqB < FLT_EPSILON*FLT_EPSILON)
				return 0.5f;
			dData.invSQRTnormSqAnormSqB = 1.f/SQRT(normSqAnormSqB);
			dData.NCC = dot<float,float>(texA, texB, size)*dData.invSQRTnormSqAnormSqB;
			const float ncc(CLAMP(1.f-dData.NCC, 0.f, 2.f));
			#ifdef MESHOPT_ROBUST
			return robustincc(ncc);
			#else
			return ncc;
			#endif
		}
		inline float ScoreDerivative(const DerivativeData& dData, uint32_t idx) const {
			ASSERT(idx < size && IsValid());
			return (dData.NCC*dData.normSqA*texB[idx]*dData.invSQRTnormSqAnormSqB - texA[idx])*dData.invSQRTnormSqAnormSqB
				#ifdef MESHOPT_ROBUST
				* robustinccg(1.f-dData.NCC)
				#endif
			;
		}
		inline float ReliabilityFactor() const {
			ASSERT(size <= area && IsValid());
			const MeanStd<float> varA(texA, size);
			const MeanStd<float> varB(texB, size);
			const float var(MINF(varA.GetVariance(), varB.GetVariance()));
			ASSERT(ISFINITE(var));
			return var/(var+0.001f);
		}
	};
	typedef cList<ViewVertex,const ViewVertex&,1,16,VIndex> ViewVertexArr;

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
		inline void operator()(const ImageRef& pt, float z) {
			if (!depthMap.isInside(pt))
				return;
			ASSERT(z > 0);
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				faceMap(pt) = idxFace;
			}
		}
	};

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

	// used to compute the analytical gradient of a vertex
	struct GradientData {
		Point3f g;  // pre-gradient of a 3D point on the surface
		Point3f xb; // barycentric coordinates of a 3D point on the surface, corresponding to the triangle it belongs to
	};
	typedef CLISTDEF0(GradientData) GradientDataArr; // used to store gradient data of a 3D point on the surface, corresponding to each pixel in an image


public:
	MeshRefine(Scene& _scene, double _weightRegularity=0.2f, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640, unsigned nMaxViews=8);
	~MeshRefine();

	bool InitImages(double scale, double sigma=0);

	void ListVertexFaces();
	void ListCameraFaces();

	typedef cList<uint16_t,uint16_t,0> AreaArr;
	void ListFaceAreas(AreaArr& maxAreas);
	void SubdivideMesh(uint32_t maxArea, float fDecimate=1.f, unsigned nCloseHoles=15, unsigned nEnsureEdgeSize=1);

	void ScoreMesh(double& score);
	#ifdef MESHOPT_NUMERICAL
	void ScoreMesh(double& score, double* gradients, double eps);
	#endif
	#ifndef MESHOPT_NUMERICAL
	void ScoreMesh(double& score, double* gradients);
	#endif
	#ifdef MESHOPT_DEBUG
	void DumpScoreMesh(const double* gradients, const String& fileName=String(_T("MeshScore.ply")));
	#endif

	// curvature related functions
	bool vertex_mean_curvature_normal(VIndex idxV, Point3f& Kh) const;
	bool vertex_gaussian_curvature(VIndex idxV, float& Kg) const;
	bool vertex_principal_curvatures(VIndex idxV, float& K1, float& K2) const;
	void vertex_principal_curvatures(float Kh, float Kg, float& K1, float& K2) const;
	float vertex_first_ring_area(VIndex idxV) const;
	bool umbrella_operator(VIndex idxV, Point3f& U) const;
	bool umbrella_operator(VIndex idxV, Point3f& U2, const Mesh::VertexArr& arrVertsU, const bool* valids) const;
	Point3f umbrella_operator_gradient(const Point3f& U, const Point3f& U2) const;

	// given a vertex position and a projection camera, compute the projected position and its derivative
	template <typename TP, typename TX, typename T>
	static T ProjectVertex(const TP* P, const TX* X, T* x, T* jacobian=NULL);


public:
	const float ratioRigidityElasticity; // a scalar weight used to compute the regularity gradient
	const double weightRegularity; // a scalar regularity weight to balance between photo-consistency and regularization terms
	const unsigned nResolutionLevel; // how many times to scale down the images before mesh optimization
	const unsigned nMinResolution; // how many times to scale down the images before mesh optimization

	// valid after ListCameraFaces()
	#ifdef MESHOPT_VERTEXFULLRING
	typedef SEACAVE::cList<BitMatrix> BitMatrixArr;
	BitMatrixArr vertexMaps; // remember for each vertex if the entire ring-1 triangle-fun is visible in the image
	#endif
	FaceMapArr faceMaps; // remember for each pixel what face projects there
	DepthMapArr depthMaps;
	#ifndef MESHOPT_NUMERICAL
	Mesh::NormalArr& faceNormals; // normals corresponding to each face
	#endif
	#ifdef MESHOPT_PHOTOGRADNORMAL
	Mesh::NormalArr& vertexNormals; // normals corresponding to each vertex
	#endif

	// valid the entire time, but changes
	Mesh::VertexArr& vertices;
	Mesh::FaceArr& faces;
	Mesh::VertexFacesArr& vertexFaces; // for each vertex, the list of faces containing it
	BoolArr& vertexBoundary; // for each vertex, stores if it is at the boundary or not

	// constant the entire time
	ImageArr& images;
	ViewsArr views; // views' data
	PairIdxArr pairs; // image pairs used to refine the mesh

	Scene& scene; // the mesh vertices and faces

	#if defined(MESHOPT_DEBUG) && MESHOPT_DEBUG>1
	unsigned _level; // current level
	unsigned _iteration; // current iteration
	#endif
};

MeshRefine::MeshRefine(Scene& _scene, double _weightRegularity, unsigned _nResolutionLevel, unsigned _nMinResolution, unsigned nMaxViews)
	:
	ratioRigidityElasticity(0.8f),
	weightRegularity(_weightRegularity),
	nResolutionLevel(_nResolutionLevel),
	nMinResolution(_nMinResolution),
	#ifndef MESHOPT_NUMERICAL
	faceNormals(_scene.mesh.faceNormals),
	#endif
	#ifdef MESHOPT_PHOTOGRADNORMAL
	vertexNormals(_scene.mesh.vertexNormals),
	#endif
	vertices(_scene.mesh.vertices),
	faces(_scene.mesh.faces),
	vertexFaces(_scene.mesh.vertexFaces),
	vertexBoundary(_scene.mesh.vertexBoundary),
	images(_scene.images),
	scene(_scene)
{
	// keep only best neighbor views for each image
	const float fMinArea(0.12f);
	const float fMinScale(0.2f), fMaxScale(3.2f);
	const float fMinAngle(FD2R(3)), fMaxAngle(FD2R(45));
	typedef std::unordered_set<uint64_t> PairsMap;
	PairsMap mapPairs;
	mapPairs.reserve(images.GetSize()*nMaxViews);
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		ViewScoreArr neighbors(imageData.neighbors);
		Scene::FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews);
		FOREACHPTR(pNeighbor, neighbors) {
			ASSERT(images[pNeighbor->idx.ID].IsValid());
			mapPairs.insert(MakePairIdx((uint32_t)idxImage, pNeighbor->idx.ID));
		}
	}
	pairs.Reserve(mapPairs.size());
	for (uint64_t pair: mapPairs)
		pairs.AddConstruct(pair);
}
MeshRefine::~MeshRefine()
{
	scene.mesh.ReleaseExtra();
}

// load and initialize all images at the given scale
// and compute the gradient for each input image
// optional: blur them using the given sigma
bool MeshRefine::InitImages(double scale, double sigma)
{
	views.Resize(images.GetSize());
	typedef float real;
	TImage<real> grad[2];
	#ifdef REFINE_USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for private(grad)
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
		// load and init image
		unsigned level(nResolutionLevel);
		const unsigned imageSize(imageData.RecomputeMaxResolution(level, nMinResolution));
		if ((imageData.image.empty() || MAXF(imageData.width,imageData.height) != imageSize) && FAILED(imageData.ReloadImage(imageSize))) {
			#ifdef REFINE_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			#else
			return false;
			#endif
		}
		View& view = views[ID];
		Image32F& img = view.image;
		imageData.image.toGray(img, cv::COLOR_BGR2GRAY, true);
		imageData.image.release();
		if (scale < 1.0) {
			cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_LINEAR);
			imageData.width = img.width(); imageData.height = img.height();
		}
		imageData.UpdateCamera(scene.platforms);
		if (sigma > 0)
			cv::GaussianBlur(img, img, cv::Size(), sigma);
		#ifndef MESHOPT_NUMERICAL
		// compute image gradient
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
		cv::merge(grad, 2, view.imageGrad);
		#endif
	}
	#ifdef REFINE_USE_OPENMP
	return !bAbort;
	#else
	return true;
	#endif
}

// extract array of triangles incident to each vertex
// and check each vertex if it is at the boundary or not
void MeshRefine::ListVertexFaces()
{
	scene.mesh.EmptyExtra();
	scene.mesh.ListIncidenteFaces();
	scene.mesh.ListBoundaryVertices();
}

// extract array of faces viewed by each image
void MeshRefine::ListCameraFaces()
{
	// extract array of faces viewed by each camera
	typedef std::unordered_set<FIndex> CameraFaces;
	typedef cList<CameraFaces> CameraFacesArr;
	CameraFacesArr arrCameraFaces(images.GetSize());
	{
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
	FOREACH(ID, images) {
		const Image& imageData = images[ID];
		if (!imageData.IsValid())
			continue;
		typedef TFrustum<float,5> Frustum;
		FacesInserter inserter(vertexFaces, arrCameraFaces[ID]);
		const Frustum frustum(Frustum::MATRIX3x4(((const PMatrix::EMatMap)imageData.camera.P).cast<float>()), (float)imageData.width, (float)imageData.height);
		octree.Traverse(frustum, inserter);
	}
	}

	// project mesh to each camera plane
	#ifdef MESHOPT_VERTEXFULLRING
	BoolArr faceValids(faces.GetSize());
	vertexMaps.Resize(images.GetSize());
	#endif
	faceMaps.Resize(images.GetSize());
	depthMaps.Resize(images.GetSize());
	Point3 ptc[3]; Point2f pti[3];
	FOREACH(ID, images) {
		const Image& imageData = images[ID];
		if (!imageData.IsValid())
			continue;
		// init view data
		FaceMap& faceMap = faceMaps[ID];
		faceMap.create(imageData.height, imageData.width);
		faceMap.memset((uint8_t)NO_ID);
		DepthMap& depthMap = depthMaps[ID];
		depthMap.create(imageData.height, imageData.width);
		depthMap.memset(0);
		const Camera& camera = imageData.camera;
		RasterMeshData data = {camera, faceMap, depthMap};
		// project all triangles on this image and keep the closest ones
		const CameraFaces& cameraFaces = arrCameraFaces[ID];
		#ifdef MESHOPT_VERTEXFULLRING
		faceValids.Memset(0);
		#endif
		for (auto idxFace : cameraFaces) {
			data.idxFace = idxFace;
			const Face& facet = faces[idxFace];
			for (unsigned v=0; v<3; ++v) {
				const Vertex& pt = vertices[facet[v]];
				ptc[v] = camera.TransformPointW2C(CastReal(pt));
				pti[v] = camera.TransformPointC2I(ptc[v]);
				// skip face if not completely inside
				if (!depthMap.isInsideWithBorder<float,3>(pti[v]))
					goto CONTIUE2NEXTFACE;
			}
			#ifdef MESHOPT_VERTEXFULLRING
			faceValids[idxFace] = true;
			#endif
			{
			#if 1
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
			Image8U3::RasterizeTriangle(pti[0], pti[1], pti[2], data);
			#else
			Image8U3::RasterizeTriangleDepth(Point3f(pti[0],float(ptc[0].z)), Point3f(pti[1],float(ptc[1].z)), Point3f(pti[2],float(ptc[2].z)), data);
			#endif
			}
			CONTIUE2NEXTFACE:;
		}
		#ifdef MESHOPT_VERTEXFULLRING
		// check for each vertex if the entire ring-1 triangle-fun is visible in the image
		BitMatrix& vertexMap = vertexMaps[ID];
		vertexMap.create(vertices.GetSize());
		vertexMap.memset(0);
		FOREACH(idxVert, vertices) {
			const Mesh::FaceIdxArr& vf = vertexFaces[idxVert];
			FOREACHPTR(pFace, vf)
				if (!faceValids[*pFace])
					goto CONTIUE2NEXTVERTEX;
			vertexMap.set(idxVert);
			CONTIUE2NEXTVERTEX:;
		}
		#endif
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 4) {
			// export depth-map
			const String fileName(String::FormatString(_T("MeshView%04u_depth.png"), ID));
			DepthMap dmap; depthMap.convertTo(dmap, CV_32F);
			ExportDepthMap(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName), dmap);
		}
		#endif
	}

	#ifndef MESHOPT_NUMERICAL
	// compute face normals
	scene.mesh.ComputeNormalFaces();
	#endif

	#ifdef MESHOPT_PHOTOGRADNORMAL
	// compute vertex normals
	scene.mesh.ComputeNormalVertices();
	#endif
}

// compute for each face the projection area as the maximum area in both images of a pair
// (make sure ListCameraFaces() was called before)
void MeshRefine::ListFaceAreas(AreaArr& maxAreas)
{
	ASSERT(maxAreas.IsEmpty());
	// for each image, compute the projection area of visible faces
	typedef cList<AreaArr> ImageAreaArr;
	ImageAreaArr viewAreas(images.GetSize());
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		AreaArr& areas = viewAreas[idxImage];
		areas.Resize(faces.GetSize());
		areas.Memset(0);
		// compute area covered by all vertices (incident faces) viewed by this image
		const FaceMap& faceMap = faceMaps[idxImage];
		for (int j=0; j<faceMap.rows; ++j) {
			for (int i=0; i<faceMap.cols; ++i) {
				const FIndex& idxFace = faceMap(j,i);
				ASSERT((idxFace == NO_ID && depthMaps[idxImage](j,i) == 0) || (idxFace != NO_ID && depthMaps[idxImage](j,i) > 0));
				if (idxFace == NO_ID)
					continue;
				++areas[idxFace];
			}
		}
	}
	// for each pair, mark the faces that have big projection areas in both images
	maxAreas.Resize(faces.GetSize());
	maxAreas.Memset(0);
	FOREACHPTR(pPair, pairs) {
		const AreaArr& areasA = viewAreas[pPair->i];
		const AreaArr& areasB = viewAreas[pPair->j];
		ASSERT(areasA.GetSize() == areasB.GetSize());
		FOREACH(f, areasA) {
			const uint16_t minArea(MINF(areasA[f], areasB[f]));
			uint16_t& maxArea = maxAreas[f];
			if (maxArea < minArea)
				maxArea = minArea;
		}
	}
}

// decimate or subdivide mesh such that for each face there is no image pair in which
// its projection area is bigger than the given number of pixels in both images
void MeshRefine::SubdivideMesh(uint32_t maxArea, float fDecimate, unsigned nCloseHoles, unsigned nEnsureEdgeSize)
{
	AreaArr maxAreas;

	// first decimate if necessary
	const bool bNoDecimation(fDecimate >= 1.f);
	const bool bNoSimplification(maxArea == 0);
	if (!bNoDecimation) {
		if (fDecimate > 0.f) {
			// decimate to the desired resolution
			scene.mesh.Clean(fDecimate, 0, false, nCloseHoles, 0);

			#ifdef MESHOPT_ENSUREEDGESIZE
			// make sure there are no edges too small or too long
			if (nEnsureEdgeSize > 0 && bNoSimplification)
				scene.mesh.EnsureEdgeSize();
			#endif

			// re-map vertex and camera faces
			ListVertexFaces();
		} else {
			// extract array of faces viewed by each camera
			ListCameraFaces();

			// estimate the faces' area that have big projection areas in both images of a pair
			ListFaceAreas(maxAreas);

			const float maxArea((float)(maxArea > 0 ? maxArea : 64));
			const float medianArea(6.f*(float)AreaArr(maxAreas).GetMedian());
			if (medianArea < maxArea) {
				maxAreas.Empty();

				// decimate to the auto detected resolution
				scene.mesh.Clean(medianArea/maxArea, 0, false, nCloseHoles, 0);

				#ifdef MESHOPT_ENSUREEDGESIZE
				// make sure there are no edges too small or too long
				if (nEnsureEdgeSize > 0 && bNoSimplification)
					scene.mesh.EnsureEdgeSize();
				#endif

				// re-map vertex and camera faces
				ListVertexFaces();
			}
		}
	}
	if (bNoSimplification)
		return;

	if (maxAreas.IsEmpty()) {
		// extract array of faces viewed by each camera
		ListCameraFaces();

		// estimate the faces' area that have big projection areas in both images of a pair
		ListFaceAreas(maxAreas);
	}

	// free some memory (these need to be recomputed anyway)
	faceMaps.Release();
	depthMaps.Release();

	// for each image, compute the projection area of visible faces
	const size_t numVertsOld(vertices.GetSize());
	const size_t numFacesOld(faces.GetSize());
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
		Face& newface = faces.AddConstruct(3); // defined by the three new vertices
		const Face& face = faces[(FIndex)f];
		SplitFace& split = mapSplits[(FIndex)f];
		for (unsigned i=0; i<3; ++i) {
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
			Face& nf = faces.AddConstruct(3);
			nf[0] = face[i];
			nf[1] = newface[(i+2)%3];
			nf[2] = newface[(i+1)%3];
		}
		split.bSplit = true;
		// find all three adjacent faces and inform them of the split
		ASSERT(mapFaces.empty());
		for (unsigned i=0; i<3; ++i) {
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
			Face& nf0 = faces.AddConstruct(3);
			nf0[0] = split.idxVert[i];
			nf0[1] = face[(i+2)%3];
			nf0[2] = face[i];
			Face& nf1 = faces.AddConstruct(3);
			nf1[0] = split.idxVert[i];
			nf1[1] = face[i];
			nf1[2] = face[(i+1)%3];
			break; }
		case 2: {
			// two edges are split; create three triangles
			const int i0(indices[0]);
			const int i1(indices[1]);
			Face& nf0 = faces.AddConstruct(3);
			Face& nf1 = faces.AddConstruct(3);
			Face& nf2 = faces.AddConstruct(3);
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
			Face& newface = faces.AddConstruct(3);
			newface[0] = split.idxVert[0];
			newface[1] = split.idxVert[1];
			newface[2] = split.idxVert[2];
			// create the last three faces, defined by one old and two new vertices
			for (int i=0; i<3; ++i) {
				Face& nf = faces.AddConstruct(3);
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

	#ifdef MESHOPT_ENSUREEDGESIZE
	// make sure there are no edges too small or too long
	#if MESHOPT_ENSUREEDGESIZE==1
	if ((nEnsureEdgeSize == 1 && !bNoDecimation) || nEnsureEdgeSize > 1)
	#endif
		scene.mesh.EnsureEdgeSize();
	#endif

	// re-map vertex and camera faces
	ListVertexFaces();

	DEBUG_EXTRA("Mesh subdivided: %u/%u -> %u/%u vertices/faces", numVertsOld, numFacesOld, vertices.GetSize(), faces.GetSize());

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 3)
		scene.mesh.Save(MAKE_PATH("MeshSubdivided.ply"));
	#endif
}

// score mesh using photo-consistency
void MeshRefine::ScoreMesh(double& score)
{
	score = 0;

	// extract array of faces viewed by each camera
	ListCameraFaces();

	// for each pair of images, compute a photo-consistency score
	// between the reference image and the pixels of the second image
	// projected in the reference image through the mesh surface
	#ifdef MESHOPT_DEBUG
	Image32F image;
	#endif
	typedef Sampler::Cubic<REAL> Sampler;
	const Sampler sampler;
	ViewVertexArr viewVertices(0, vertices.GetSize());
	cList<uint16_t,uint16_t,0> countVertices(vertices.GetSize());
	countVertices.Memset(0);
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		// fetch view A data
		const uint32_t idxImageA(pPair->i);
		const Image& imageDataA = images[idxImageA];
		ASSERT(imageDataA.IsValid());
		#ifdef MESHOPT_VERTEXFULLRING
		const BitMatrix& vertexMapA = vertexMaps[idxImageA];
		#endif
		const FaceMap& faceMapA = faceMaps[idxImageA];
		const DepthMap& depthMapA = depthMaps[idxImageA];
		const Image32F& imageA = views[idxImageA].image;
		const Camera& cameraA = imageDataA.camera;
		// fetch view B data
		const uint32_t idxImageB(pPair->j);
		const Image& imageDataB = images[idxImageB];
		ASSERT(imageDataB.IsValid());
		#ifdef MESHOPT_VERTEXFULLRING
		const BitMatrix& vertexMapB = vertexMaps[idxImageB];
		#endif
		const DepthMap& depthMapB = depthMaps[idxImageB];
		const Image32F& imageB = views[idxImageB].image;
		const Camera& cameraB = imageDataB.camera;
		// compute area covered by all vertices (incident faces) viewed by this image
		ASSERT(viewVertices.IsEmpty());
		viewVertices.Resize(vertices.GetSize());
		for (int j=0; j<faceMapA.rows; ++j) {
			for (int i=0; i<faceMapA.cols; ++i) {
				const FIndex& idxFace = faceMapA(j,i);
				ASSERT((idxFace == NO_ID && depthMapA(j,i) == 0) || (idxFace != NO_ID && depthMapA(j,i) > 0));
				if (idxFace == NO_ID)
					continue;
				const Face& facet = faces[idxFace];
				for (unsigned v=0; v<3; ++v)
					++viewVertices[facet[v]].area;
			}
		}
		// init vertex textures
		FOREACHPTR(pViewVertex, viewVertices)
			pViewVertex->Init();
		// project the surface seen by this image on the pair image
		// and store pixel values
		#ifdef MESHOPT_DEBUG
		image.create(imageA.size());
		image.memset(0);
		#endif
		for (int j=0; j<depthMapA.rows; ++j) {
			for (int i=0; i<depthMapA.cols; ++i) {
				const Depth& depthA = depthMapA(j,i);
				if (depthA <= 0)
					continue;
				const Point3 X(cameraA.TransformPointI2W(Point3(i,j,depthA)));
				const Point3f ptC(cameraB.TransformPointW2C(X));
				const Point2f pt(cameraB.TransformPointC2I(ptC));
				const ImageRef ir(ROUND2INT(pt));
				if (!depthMapB.isInsideWithBorder<float,3>(ir))
					continue;
				const Depth& depthB = depthMapB(ir);
				#ifndef MESHOPT_DEPTHCONSTBIAS
				if (depthB <= 0 || !IsDepthSimilar(depthB, ptC.z, 0.01f))
				#else
				if (depthB <= 0 || depthB+MESHOPT_DEPTHCONSTBIAS < ptC.z)
				#endif
					continue;
				const float pixelB((float)imageB.sample<Sampler,Sampler::Type>(sampler, pt));
				#ifdef MESHOPT_DEBUG
				image(j,i) = pixelB;
				#endif
				const float pixelA(imageA(j,i));
				// store pixel value in each vertex of the face
				const FIndex& idxFace = faceMapA(j,i);
				ASSERT(idxFace != NO_ID);
				const Face& facet = faces[idxFace];
				for (unsigned v=0; v<3; ++v)
					viewVertices[facet[v]].Insert(pixelA, pixelB);
			}
		}
		#ifdef MESHOPT_DEBUG
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 1) {
			// export back-projected image
			Image8U grayImage;
			image.convertTo(grayImage, CV_8U, 255.0);
			const String fileName(String::FormatString(_T("MeshView%04u_%04u.png"), idxImageA, idxImageB));
			grayImage.Save(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
		}
		#endif
		#endif
		// init vertex textures
		#ifdef MESHOPT_DEBUG
		image.create(imageA.size());
		image.memset(0);
		#endif
		const float weightA(SQUARE(imageDataA.avgDepth/(float)cameraA.GetFocalLength()));
		FOREACH(idxV, viewVertices) {
			ViewVertex& viewVertex = viewVertices[idxV];
			if (!viewVertex.IsValid() || vertexBoundary[idxV]
				#ifdef MESHOPT_VERTEXFULLRING
				|| !vertexMapA.isSet(idxV) || !vertexMapB.isSet(idxV)
				#endif
			) continue;
			++countVertices[idxV];
			const float vscore(viewVertex.Score());
			ASSERT(ISFINITE(vscore) && ISFINITE(weightA));
			score += vscore*weightA*viewVertex.ReliabilityFactor()*viewVertex.size;
			#ifdef MESHOPT_DEBUG
			const Point2f pt(cameraA.TransformPointW2I(CastDouble(vertices[idxV])));
			image(ROUND2INT(pt)) = vscore;
			#endif
		}
		#ifdef MESHOPT_DEBUG
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 1) {
			// export back-projected image
			Image8U grayImage;
			image.convertTo(grayImage, CV_8U, 255.0*3);
			const String fileName(String::FormatString(_T("MeshView%04u_%04u_Score.png"), idxImageA, idxImageB));
			grayImage.Save(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
		}
		#endif
		#endif
		viewVertices.Empty();
	}

	#ifdef MESHOPT_SMOOTH
	// loop through all vertices and add the smoothing term
	FOREACH(idxV, vertices) {
		const unsigned count(countVertices[idxV]);
		if (count == 0)
			continue;
		ASSERT(vertexBoundary[idxV] == false);
		#if 0
		float k1, k2;
		vertex_principal_curvatures(idxV, k1, k2);
		const float regularityScore(MINF((ABS(k1)+ABS(k2))/*(k1*k1+k2*k2)*/, 30.f)*vertex_first_ring_area(idxV)/3);
		#else
		Point3f U;
		umbrella_operator(idxV, U);
		const float regularityScore(norm(U));
		#endif
		ASSERT(ISFINITE(regularityScore));
		score += weightRegularity*regularityScore;
	}
	#endif
}

#ifdef MESHOPT_NUMERICAL
// score mesh using photo-consistency
// and compute vertices gradient using numerical method
void MeshRefine::ScoreMesh(double& score, double* gradients, double eps)
{
	score = 0;
	memset(gradients, 0, sizeof(double)*vertices.GetSize()*3);

	// extract array of faces viewed by each camera
	ListCameraFaces();

	// for each pair of images, compute a photo-consistency score
	// between the reference image and the pixels of the second image
	// projected in the reference image through the mesh surface
	#ifdef MESHOPT_DEBUG
	Image32F image;
	#endif
	typedef Sampler::Cubic<REAL> Sampler;
	const Sampler sampler;
	ViewVertexArr viewVertices(0, vertices.GetSize());
	cList<uint16_t,uint16_t,0> countVertices(vertices.GetSize());
	countVertices.Memset(0);
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		// fetch view A data
		const uint32_t idxImageA(pPair->i);
		const Image& imageDataA = images[idxImageA];
		ASSERT(imageDataA.IsValid());
		#ifdef MESHOPT_VERTEXFULLRING
		const BitMatrix& vertexMapA = vertexMaps[idxImageA];
		#endif
		const FaceMap& faceMapA = faceMaps[idxImageA];
		const DepthMap& depthMapA = depthMaps[idxImageA];
		const Image32F& imageA = views[idxImageA].image;
		const Camera& cameraA = imageDataA.camera;
		// fetch view B data
		const uint32_t idxImageB(pPair->j);
		const Image& imageDataB = images[idxImageB];
		ASSERT(imageDataB.IsValid());
		#ifdef MESHOPT_VERTEXFULLRING
		const BitMatrix& vertexMapB = vertexMaps[idxImageB];
		#endif
		const DepthMap& depthMapB = depthMaps[idxImageB];
		const Image32F& imageB = views[idxImageB].image;
		const Camera& cameraB = imageDataB.camera;
		// compute area covered by all vertices (incident faces) viewed by this image
		ASSERT(viewVertices.IsEmpty());
		viewVertices.Resize(vertices.GetSize());
		for (int j=0; j<faceMapA.rows; ++j) {
			for (int i=0; i<faceMapA.cols; ++i) {
				const FIndex& idxFace = faceMapA(j,i);
				ASSERT((idxFace == NO_ID && depthMapA(j,i) == 0) || (idxFace != NO_ID && depthMapA(j,i) > 0));
				if (idxFace == NO_ID)
					continue;
				const Face& facet = faces[idxFace];
				for (unsigned v=0; v<3; ++v)
					++viewVertices[facet[v]].area;
			}
		}
		// init vertex textures for the score and the three derivatives
		FOREACHPTR(pViewVertex, viewVertices)
			pViewVertex->InitGradient();
		// project the surface seen by this image on the pair image
		// and store pixel values
		#ifdef MESHOPT_DEBUG
		image.create(imageA.size());
		image.memset(0);
		#endif
		for (int j=0; j<depthMapA.rows; ++j) {
			for (int i=0; i<depthMapA.cols; ++i) {
				const Depth& depthA = depthMapA(j,i);
				if (depthA <= 0)
					continue;
				const Point3 X(cameraA.TransformPointI2W(Point3(i,j,depthA)));
				const Point3f ptC(cameraB.TransformPointW2C(X));
				const Point2f pt(cameraB.TransformPointC2I(ptC));
				const ImageRef ir(ROUND2INT(pt));
				if (!depthMapB.isInsideWithBorder<float,3>(ir))
					continue;
				const Depth& depthB = depthMapB(ir);
				#ifndef MESHOPT_DEPTHCONSTBIAS
				if (depthB <= 0 || !IsDepthSimilar(depthB, ptC.z, 0.01f))
				#else
				if (depthB <= 0 || depthB+MESHOPT_DEPTHCONSTBIAS < ptC.z)
				#endif
					continue;
				const float pixelA(imageA(j,i));
				const FIndex& idxFace = faceMapA(j,i);
				ASSERT(idxFace != NO_ID);
				const Face& facet = faces[idxFace];
				// store pixel value in each vertex of the face
				const float pixelB((float)imageB.sample<Sampler,Sampler::Type>(sampler, pt));
				#ifdef MESHOPT_DEBUG
				image(j,i) = pixelB;
				#endif
				// store pixel value in each vertex of the face corresponding to the derivative
				const Point3d verts[] = {vertices[facet[0]], vertices[facet[1]], vertices[facet[2]]};
				for (unsigned v=0; v<3; ++v) {
					ViewVertex& viewVert = viewVertices[facet[v]];
					const Point3d& vert(verts[v]);
					const double step(eps*cameraB.PointDepth(vert));
					ASSERT(step > 0);
					const Ray3d ray(cameraA.C, cameraA.TransformPointI2W(Point3(i,j,1)), true);
					for (unsigned i=0; i<3; ++i) {
						Point3d vertd(vert);
						vertd[i] += step;
						const Planed plane(vertd, verts[(v+1)%3], verts[(v+2)%3]);
						const Point3d X(ray.Intersects(plane));
						const Point2f pt(cameraB.TransformPointW2I(X));
						if (depthMapB.isInsideWithBorder<float,2>(pt))
							viewVert.Insert(i, (float)imageB.sample<Sampler,Sampler::Type>(sampler, pt));
						else
							viewVert.Insert(i, pixelB);
					}
					viewVert.Insert(pixelA, pixelB);
				}
			}
		}
		#ifdef MESHOPT_DEBUG
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 1) {
			// export back-projected image
			Image8U grayImage;
			image.convertTo(grayImage, CV_8U, 255.0);
			const String fileName(String::FormatString(_T("MeshView%04u_%04u.png"), idxImageA, idxImageB));
			grayImage.Save(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
		}
		#endif
		#endif
		// compute score and gradients
		#ifdef MESHOPT_DEBUG
		image.create(imageA.size());
		image.memset(0);
		#endif
		const float weightA(SQUARE(imageDataA.avgDepth/(float)cameraA.GetFocalLength()));
		FOREACH(idxV, viewVertices) {
			ViewVertex& viewVertex = viewVertices[idxV];
			if (!viewVertex.IsValid() || vertexBoundary[idxV]
				#ifdef MESHOPT_VERTEXFULLRING
				|| !vertexMapA.isSet(idxV) || !vertexMapB.isSet(idxV)
				#endif
			) continue;
			++countVertices[idxV];
			const float normSqA(viewVertex.PreScore());
			if (normSqA < FLT_EPSILON)
				continue;
			// compute score
			const float w(weightA*viewVertex.ReliabilityFactor()*viewVertex.size);
			const float vscore(viewVertex.Score(normSqA, viewVertex.GetBufferB()));
			ASSERT(ISFINITE(vscore) && ISFINITE(w));
			score += vscore*w;
			#ifdef MESHOPT_DEBUG
			const Point2f pt(cameraA.TransformPointW2I(CastDouble(vertices[idxV])));
			image(ROUND2INT(pt)) = vscore;
			#endif
			// vertex gradient
			double* grad = gradients+idxV*3;
			for (unsigned i=0; i<3; ++i) {
				const double step(eps*cameraB.PointDepth(CastDouble(vertices[idxV])));
				ASSERT(step > 0);
				const float vscored(viewVertex.Score(normSqA, viewVertex.GetBufferB(i)));
				const double g(((double)vscored-(double)vscore)*w/step);
				ASSERT(ISFINITE(g));
				grad[i] += g;
			}
		}
		#ifdef MESHOPT_DEBUG
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 1) {
			// export back-projected image
			Image8U grayImage;
			image.convertTo(grayImage, CV_8U, 255.0*3);
			const String fileName(String::FormatString(_T("MeshView%04u_%04u_Score.png"), idxImageA, idxImageB));
			grayImage.Save(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
		}
		#if MESHOPT_DEBUG>1
		if (VERBOSITY_LEVEL > 3)
			DumpScoreMesh(gradients, String::FormatString(_T("MeshScore_Lvl%u_Iter%04u_View%04u_%04u.ply"), _level, _iteration, idxImageA, idxImageB));
		#endif
		#endif
		#endif
		viewVertices.Empty();
	}
	#ifdef MESHOPT_PHOTOGRADNORMAL
	// project gradient vector onto the vertex normal
	FOREACH(idxV, vertices) {
		const Point3d N(vertexNormals[idxV]);
		Point3d& grad = ((Point3d*)gradients)[idxV];
		grad = N*(grad.dot(N));
	}
	#endif
	#if defined(MESHOPT_DEBUG) && MESHOPT_DEBUG>1
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		DumpScoreMesh(gradients, String::FormatString(_T("MeshScore_Lvl%u_Iter%04u.ply"), _level, _iteration));
	#endif
	#endif

	#ifdef MESHOPT_SMOOTH
	// loop through all vertices and add the smoothing term
	#if 0
	size_t countDist(0);
	Vertex avgDist(Vertex::ZERO);
	FOREACHPTR(pFace, arrFaces) {
		const Face& face = *pFace;
		const Vertex& v0(arrVerts[face[0]]);
		const Vertex& v1(arrVerts[face[1]]);
		const Vertex& v2(arrVerts[face[2]]);
		avgDist += ABS(v0-v1);
		avgDist += ABS(v1-v2);
		avgDist += ABS(v2-v0);
		countDist += 3;
	}
	avgDist *= 1.f/countDist;
	const double eps2(eps*(avgDist.x+avgDist.y+avgDist.y)/3);
	ASSERT(eps2 > 0);
	FOREACH(idxV, arrVerts) {
		const unsigned count(countVertices[idxV]);
		if (count == 0)
			continue;
		ASSERT(vertexBoundary[idxV] == false);
		float k1, k2;
		vertex_principal_curvatures(idxV, k1, k2);
		const float regularityScore(MINF((ABS(k1)+ABS(k2))/*(k1*k1+k2*k2)*/, 30.f)*vertex_first_ring_area(idxV)/*/3.f*/);
		ASSERT(ISFINITE(regularityScore));
		score += weightRegularity*regularityScore;
		// vertex gradient
		double* grad = gradients+idxV*3;
		// compute the smoothing term gradient numerically
		Vertex& vertd(arrVerts[idxV]);
		const Vertex vert(vertd);
		for (unsigned i=0; i<3; ++i) {
			vertd[i] += eps2;
			vertex_principal_curvatures(idxV, k1, k2);
			const float regularityScored(MINF((ABS(k1)+ABS(k2))/*(k1*k1+k2*k2)*/, 30.f)*vertex_first_ring_area(idxV)/*/3.f*/);
			const double g(((double)regularityScored-(double)regularityScore)*weightRegularity/eps2);
			ASSERT(ISFINITE(g));
			grad[i] += g;
			vertd = vert;
		}
	}
	#else
	Mesh::VertexArr arrVertsU(vertices.GetSize());
	BoolArr validsU(vertices.GetSize());
	validsU.Memset(0);
	FOREACH(idxV, vertices) {
		const unsigned count(countVertices[idxV]);
		if (count == 0)
			continue;
		ASSERT(vertexBoundary[idxV] == false);
		Vertex& U = arrVertsU[idxV];
		validsU[idxV] = umbrella_operator(idxV, U);
		const float regularityScore(norm(U));
		ASSERT(ISFINITE(regularityScore));
		score += weightRegularity*regularityScore;
	}
	Point3f U2;
	FOREACH(idxV, vertices) {
		const unsigned count(countVertices[idxV]);
		if (count == 0)
			continue;
		ASSERT(vertexBoundary[idxV] == false);
		// compute the thin-plate energy gradient using the umbrella operator
		if (!umbrella_operator(idxV, U2, arrVertsU, validsU.Begin()))
			continue; // vertex at the border of the mesh
		// vertex gradient
		Point3d& grad = ((Point3d*)gradients)[idxV];
		ASSERT(ISFINITE(U2));
		grad += weightRegularity*CastDouble(
			#if 0
			umbrella_operator_gradient(arrVertsU[idxV], U2)
			#else
			U2
			#endif
		);
	}
	#endif
	#if defined(MESHOPT_DEBUG) && MESHOPT_DEBUG>1
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		DumpScoreMesh(gradients, String::FormatString(_T("MeshScore_Lvl%u_Iter%04u_Smooth.ply"), _level, _iteration));
	#endif
	++_iteration;
	#endif
	#endif
}
#endif

#ifndef MESHOPT_NUMERICAL
// score mesh using photo-consistency
// and compute vertices gradient using analytical method
void MeshRefine::ScoreMesh(double& score, double* gradients)
{
	score = 0;
	memset(gradients, 0, sizeof(double)*vertices.GetSize()*3);

	// extract array of faces viewed by each camera
	ListCameraFaces();

	// for each pair of images, compute a photo-consistency score
	// between the reference image and the pixels of the second image
	// projected in the reference image through the mesh surface
	#ifdef MESHOPT_DEBUG
	Image32F image;
	#endif
	typedef Sampler::Cubic<REAL> Sampler;
	const Sampler sampler;
	ViewVertexArr viewVertices(0, vertices.GetSize());
	cList<uint16_t,uint16_t,0> countVertices(vertices.GetSize());
	countVertices.Memset(0);
	GradientDataArr gradDatas;
	TMatrix<float,2,3> xJac;
	Point2f pt;
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		// fetch view A data
		const uint32_t idxImageA(pPair->i);
		const Image& imageDataA = images[idxImageA];
		ASSERT(imageDataA.IsValid());
		#ifdef MESHOPT_VERTEXFULLRING
		const BitMatrix& vertexMapA = vertexMaps[idxImageA];
		#endif
		const FaceMap& faceMapA = faceMaps[idxImageA];
		const DepthMap& depthMapA = depthMaps[idxImageA];
		const Image32F& imageA = views[idxImageA].image;
		const Camera& cameraA = imageDataA.camera;
		// fetch view B data
		const uint32_t idxImageB(pPair->j);
		const Image& imageDataB = images[idxImageB];
		ASSERT(imageDataB.IsValid());
		#ifdef MESHOPT_VERTEXFULLRING
		const BitMatrix& vertexMapB = vertexMaps[idxImageB];
		#endif
		const DepthMap& depthMapB = depthMaps[idxImageB];
		const Image32F& imageB = views[idxImageB].image;
		const Camera& cameraB = imageDataB.camera;
		const View::ImageGrad& imageGradB = views[idxImageB].imageGrad;
		// compute area covered by all vertices (incident faces) viewed by this image
		ASSERT(viewVertices.IsEmpty());
		viewVertices.Resize(vertices.GetSize());
		for (int j=0; j<faceMapA.rows; ++j) {
			for (int i=0; i<faceMapA.cols; ++i) {
				const FIndex& idxFace = faceMapA(j,i);
				ASSERT((idxFace == NO_ID && depthMapA(j,i) == 0) || (idxFace != NO_ID && depthMapA(j,i) > 0));
				if (idxFace == NO_ID)
					continue;
				const Face& facet = faces[idxFace];
				for (unsigned v=0; v<3; ++v)
					++viewVertices[facet[v]].area;
			}
		}
		// init vertex textures
		FOREACHPTR(pViewVertex, viewVertices)
			pViewVertex->InitIndices();
		// project the surface seen by this image on the pair image
		// and store pixel values
		#ifdef MESHOPT_DEBUG
		image.create(imageA.size());
		image.memset(0);
		#endif
		gradDatas.Resize(faceMapA.area());
		for (int j=0; j<depthMapA.rows; ++j) {
			for (int i=0; i<depthMapA.cols; ++i) {
				const Depth& depthA = depthMapA(j,i);
				if (depthA <= 0)
					continue;
				const Point3 X(cameraA.TransformPointI2W(Point3(i,j,depthA)));
				// project point in second image and
				// projection Jacobian matrix in the second image of the 3D point on the surface
				const float z(ProjectVertex(cameraB.P.val, X.ptr(), pt.ptr(), xJac.val));
				const ImageRef ir(ROUND2INT(pt));
				if (!depthMapB.isInsideWithBorder<float,3>(ir))
					continue;
				const Depth& depthB = depthMapB(ir);
				#ifndef MESHOPT_DEPTHCONSTBIAS
				if (depthB <= 0 || !IsDepthSimilar(depthB, z, 0.01f))
				#else
				if (depthB <= 0 || depthB+MESHOPT_DEPTHCONSTBIAS < z)
				#endif
					continue;
				const float pixelB((float)imageB.sample<Sampler,Sampler::Type>(sampler, pt));
				#ifdef MESHOPT_DEBUG
				image(j,i) = pixelB;
				#endif
				const float pixelA(imageA(j,i));
				// store pixel value and its index in each vertex of the face
				const uint32_t idxA(j*depthMapA.width()+i);
				const FIndex& idxFace = faceMapA(j,i);
				ASSERT(idxFace != NO_ID);
				const Face& face = faces[idxFace];
				const VIndex f0(face[0]);
				const VIndex f1(face[1]);
				const VIndex f2(face[2]);
				viewVertices[f0].Insert(pixelA, pixelB, idxA);
				viewVertices[f1].Insert(pixelA, pixelB, idxA);
				viewVertices[f2].Insert(pixelA, pixelB, idxA);
				#if 0
				// numerical differentiation
				typedef double TX;
				Point3f fjac;
				{
				const TX eps = 1.e-5;
				TX cX[3];
				memcpy(cX, X.ptr(), sizeof(TX)*3);
				TX pos[2];
				TX x0, x1;
				ProjectVertex_3x4_3_2(cameraB.P.val, cX, pos);
				x0 = imageB.sample<Sampler,Sampler::Type>(sampler, *((const TPoint2<TX>*)pos));
				for (unsigned j=0; j<3; ++j) {
					const TX temp = cX[j];
					const TX step = eps*MAXF(eps,abs(temp));
					cX[j] += step;
					ProjectVertex_3x4_3_2(cameraB.P.val, cX, pos);
					x1 = imageB.sample<Sampler,Sampler::Type>(sampler, *((const TPoint2<TX>*)pos));
					cX[j] = temp;
					fjac[j] = (float)((x1 - x0) / step);
				}
				}
				const Point3f del((const TMatrix<float,1,2>&)Point2f(imageGradB.sample< Sampler,TPoint2<Sampler::Type> >(sampler, pt))*xJac);
				//const Point3f g = (fjac.dot(d)/N.dot(d))*N;
				#endif
				// store data needed to compute the gradient as the barycentric coordinates
				GradientData& gradData = gradDatas[idxA];
				const Point3f d(X-cameraA.C);
				const Point3f& N(faceNormals[idxFace]);
				const Point2f gj(imageGradB.sample< Sampler,TPoint2<Sampler::Type> >(sampler, pt));
				gradData.g = (gj.dot(Point2f(xJac*(const Vec3f&)d))/N.dot(d))*N;
				gradData.xb = CorrectBarycentricCoordinates(BarycentricCoordinatesUV(vertices[f0], vertices[f1], vertices[f2], CastFloat(X)));
			}
		}
		#ifdef MESHOPT_DEBUG
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 1) {
			// export back-projected image
			Image8U grayImage;
			image.convertTo(grayImage, CV_8U, 255.0);
			const String fileName(String::FormatString(_T("MeshView%04u_%04u.png"), idxImageA, idxImageB));
			grayImage.Save(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
		}
		#endif
		#endif
		// init vertex textures
		#ifdef MESHOPT_DEBUG
		image.create(imageA.size());
		image.memset(0);
		#endif
		const float weightA(SQUARE(imageDataA.avgDepth/(float)cameraA.GetFocalLength()));
		ViewVertex::DerivativeData dData;
		FOREACH(idxV, viewVertices) {
			ViewVertex& viewVertex = viewVertices[idxV];
			if (!viewVertex.IsValid() || vertexBoundary[idxV]
				#ifdef MESHOPT_VERTEXFULLRING
				|| !vertexMapA.isSet(idxV) || !vertexMapB.isSet(idxV)
				#endif
			) continue;
			++countVertices[idxV];
			// compute score
			const float w(weightA*viewVertex.ReliabilityFactor()*viewVertex.size);
			const float vscore(viewVertex.Score(dData));
			ASSERT(ISFINITE(vscore) && ISFINITE(w));
			score += vscore*w;
			#ifdef MESHOPT_DEBUG
			const Point2f pt(cameraA.TransformPointW2I(CastDouble(vertices[idxV])));
			image(ROUND2INT(pt)) = vscore;
			#endif
			// vertex gradient
			Point3d& grad = ((Point3d*)gradients)[idxV];
			for (uint32_t i=0; i<viewVertex.size; ++i) {
				const uint32_t idxA(viewVertex.GetIndexA(i));
				ASSERT(faceMapA.isContinuous() && (int)idxA < faceMapA.area());
				const FIndex& idxFace = faceMapA.ptr<const FIndex>()[idxA];
				const Face& face = faces[idxFace];
				const GradientData& gradData = gradDatas[idxA];
				const double b(gradData.xb[Mesh::FindVertex(face, idxV)]);
				const double dNCC(viewVertex.ScoreDerivative(dData, i));
				const double s(w*b*dNCC);
				ASSERT(ISFINITE(s) && ISFINITE(gradData.g));
				grad += CastDouble(gradData.g)*s;
			}
		}
		#ifdef MESHOPT_DEBUG
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 1) {
			// export back-projected image
			Image8U grayImage;
			image.convertTo(grayImage, CV_8U, 255.0*3);
			const String fileName(String::FormatString(_T("MeshView%04u_%04u_Score.png"), idxImageA, idxImageB));
			grayImage.Save(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
		}
		#if MESHOPT_DEBUG>1
		if (VERBOSITY_LEVEL > 3)
			DumpScoreMesh(gradients, String::FormatString(_T("MeshScore_Lvl%u_Iter%04u_View%04u_%04u.ply"), _level, _iteration, idxImageA, idxImageB));
		#endif
		#endif
		#endif
		viewVertices.Empty();
	}
	#ifdef MESHOPT_PHOTOGRADNORMAL
	// project gradient vector onto the vertex normal
	FOREACH(idxV, vertices) {
		const Point3d N(vertexNormals[idxV]);
		Point3d& grad = ((Point3d*)gradients)[idxV];
		grad = N*(grad.dot(N));
	}
	#endif
	#if defined(MESHOPT_DEBUG) && MESHOPT_DEBUG>1
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		DumpScoreMesh(gradients, String::FormatString(_T("MeshScore_Lvl%u_Iter%04u.ply"), _level, _iteration));
	#endif
	#endif

	#ifdef MESHOPT_SMOOTH
	// loop through all vertices and add the smoothing term
	Mesh::VertexArr arrVertsU(vertices.GetSize());
	BoolArr validsU(vertices.GetSize());
	validsU.Memset(0);
	FOREACH(idxV, vertices) {
		const unsigned count(countVertices[idxV]);
		if (count == 0)
			continue;
		ASSERT(vertexBoundary[idxV] == false);
		Vertex& U = arrVertsU[idxV];
		validsU[idxV] = umbrella_operator(idxV, U);
		const float regularityScore(norm(U));
		ASSERT(ISFINITE(regularityScore));
		score += weightRegularity*regularityScore;
	}
	Point3f U2;
	FOREACH(idxV, vertices) {
		const unsigned count(countVertices[idxV]);
		if (count == 0)
			continue;
		ASSERT(vertexBoundary[idxV] == false);
		// compute the thin-plate energy gradient using the umbrella operator
		if (!umbrella_operator(idxV, U2, arrVertsU, validsU.Begin()))
			continue; // vertex at the border of the mesh
		// vertex gradient
		Point3d& grad = ((Point3d*)gradients)[idxV];
		ASSERT(ISFINITE(U2));
		grad += weightRegularity*CastDouble(
			#if 0
			umbrella_operator_gradient(arrVertsU[idxV], U2)
			#else
			U2
			#endif
		);
	}
	#if defined(MESHOPT_DEBUG) && (MESHOPT_DEBUG>1)
	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 2)
		DumpScoreMesh(gradients, String::FormatString(_T("MeshScore_Lvl%u_Iter%04u_Smooth.ply"), _level, _iteration));
	#endif
	++_iteration;
	#endif
	#endif
}
#endif

#ifdef MESHOPT_DEBUG
void MeshRefine::DumpScoreMesh(const double* gradients, const String& fileName)
{
	Mesh::NormalArr vertexNormalsOrg;
	vertexNormalsOrg.Swap(vertexNormals);
	vertexNormals.Resize(vertices.GetSize());
	FOREACH(i, vertexNormals)
		vertexNormals[i] = ((const Point3d*)gradients)[i];
	scene.mesh.Save(MAKE_PATH_FULL(WORKING_FOLDER_FULL, fileName));
	vertexNormals.Swap(vertexNormalsOrg);
}
#endif



/** 
 * Computes the discrete curvature approximation as in the paper [Meyer et al 2002]:
 * "Discrete Differential-Geometry Operators for Triangulated 2-Manifolds"
 * Mark Meyer, Mathieu Desbrun, Peter Schroder, Alan H. Barr
 * VisMath '02, Berlin (Germany) 
 * http://www-grail.usc.edu/pubs.html
 */

inline bool angle_obtuse(const Vertex& v, const Vertex& v1, const Vertex& v2) {
	return ((v1-v).dot(v2-v) < 0);
}
inline bool triangle_obtuse(const Vertex& v, const Vertex& v1, const Vertex& v2) {
	return (angle_obtuse(v, v1, v2) ||
			angle_obtuse(v1, v2, v) ||
			angle_obtuse(v2, v, v1));
} 

inline float cotan(const Vertex& vo, const Vertex& v1, const Vertex& v2) {
	// cf. Appendix B of [Meyer et al 2002]
	const Vertex u(v1-vo);
	const Vertex v(v2-vo);

	const float udotv(u.dot(v));
	const float denom(SQRT(normSq(u)*normSq(v)-udotv*udotv));

	// denom can be zero if u==v
	// returning 0 is acceptable, based on the callers of this function below
	if (denom == 0)
		return 0;
	return (udotv/denom);
}
inline float angle_from_cotan(const Vertex& vo, const Vertex& v1, const Vertex& v2) {
	// cf. Appendix B and the caption of Table 1 from [Meyer et al 2002]
	// Note: I assume this is what they mean by using atan2()
	const Vertex u(v1-vo);
	const Vertex v(v2-vo);

	const float udotv(u.dot(v));
	const float denom(SQRT(normSq(u)*normSq(v)-udotv*udotv));

	// tan = denom/udotv = y/x (see help for atan2)
	return ABS(atan2(denom, udotv));
}

inline float region_area(const Vertex& v, const Vertex& v1, const Vertex& v2) {
	// cf. Section 3.3 of [Meyer et al 2002]

	// computes the area of the triangle
	const Vertex vec1(v1-v);
	const Vertex vec2(v2-v);
	const Vertex normal(vec1.cross(vec2));
	const float area(norm(normal)*0.5f);
	if (area == 0)
		return 0;

	if (triangle_obtuse(v, v1, v2)) {
		if (angle_obtuse(v, v1, v2))
			return (area/2.f);
		else
			return (area/4.f);
	}
	return ((cotan(v1, v, v2)*normSq(vec2) + cotan(v2, v, v1)*normSq(vec1))/8.f);
}

/**
 * vertex_mean_curvature_normal:
 * @v: a vertex index
 * @Kh: the Mean Curvature Normal at the given vertex
 *
 * Computes the Discrete Mean Curvature Normal approximation at @v.
 * The mean curvature at @v is half the magnitude of the vector @Kh.
 * See [Meyer et al 2002].
 *
 * Note: the normal computed is not unit length, and may point either
 * into or out of the surface, depending on the curvature at @v.  It
 * is the responsibility of the caller of the function to use the mean
 * curvature normal appropriately.
 *
 * Returns: %TRUE if the operator could be evaluated, %FALSE if the
 * evaluation failed for some reason (@v is boundary or is the
 * endpoint of a non-manifold edge).
 */
bool MeshRefine::vertex_mean_curvature_normal(VIndex idxV, Point3f& Kh) const
{
	#if 1
	ASSERT(!vertexBoundary[idxV]);
	#else
	// this operator is not defined for boundary edges
	if (vertexBoundary[idxV])
		return false;
	#endif

	float area(0);
	Kh = Point3f::ZERO;
	const Vertex& v(vertices[idxV]);
	const Mesh::FaceIdxArr& vf(vertexFaces[idxV]);
	ASSERT(vf.GetSize() > 2);
	FOREACHPTR(pFaceIdx, vf) {
		const Face& face = faces[*pFaceIdx];
		const uint32_t idx(Mesh::FindVertex(face, idxV));
		ASSERT(idx != NO_ID);
		const Vertex& v1(vertices[face[(idx+1)%3]]);
		const Vertex& v2(vertices[face[(idx+2)%3]]);
		Kh += cotan(v1, v, v2)*(v2-v);
		Kh += cotan(v2, v, v1)*(v1-v);
		area += region_area(v, v1, v2);
	}
	if (area <= 0)
		return false;
	Kh /= area*2;
	return true;
}

/**
 * vertex_gaussian_curvature:
 * @v: a vertex index
 * @Kg: the Discrete Gaussian Curvature approximation at @v
 *
 * Computes the Discrete Gaussian Curvature approximation at @v.
 * See [Meyer et al 2002].
 *
 * Returns: %TRUE if the operator could be evaluated, %FALSE if the
 * evaluation failed for some reason (@v is boundary or is the
 * endpoint of a non-manifold edge).
 */
bool MeshRefine::vertex_gaussian_curvature(VIndex idxV, float& Kg) const
{
	#if 1
	ASSERT(!vertexBoundary[idxV]);
	#else
	// this operator is not defined for boundary edges
	if (vertexBoundary[idxV])
		return false;
	#endif

	float area(0);
	float angle_sum(0);
	const Vertex& v(vertices[idxV]);
	const Mesh::FaceIdxArr& vf(vertexFaces[idxV]);
	ASSERT(vf.GetSize() > 2);
	FOREACHPTR(pFaceIdx, vf) {
		const Face& face = faces[*pFaceIdx];
		const uint32_t idx(Mesh::FindVertex(face, idxV));
		ASSERT(idx != NO_ID);
		const Vertex& v1(vertices[face[(idx+1)%3]]);
		const Vertex& v2(vertices[face[(idx+2)%3]]);
		angle_sum += angle_from_cotan(v, v1, v2);
		area += region_area(v, v1, v2);
	}
	if (area <= 0)
		return false;
	Kg = (2.f*FPI - angle_sum)/area;
	return true;
}

/**
 * vertex_principal_curvatures:
 * @v: a vertex index
 * @K1: first principal curvature
 * @K2: second principal curvature
 *
 * Computes the principal curvatures at a point by combining the mean and
 * Gaussian curvatures at that point, calculated from the
 * discrete differential operators.
 * See [Meyer et al 2002].
 *
 * Returns: %TRUE if the operator could be evaluated, %FALSE if the
 * evaluation failed for some reason (@v is boundary or is the
 * endpoint of a non-manifold edge).
 */
bool MeshRefine::vertex_principal_curvatures(VIndex idxV, float& K1, float& K2) const
{
	#if 1
	ASSERT(!vertexBoundary[idxV]);
	#else
	// this operator is not defined for boundary edges
	if (vertexBoundary[idxV])
		return false;
	#endif

	float area(0);
	float angle_sum(0);
	Point3f Kh(Point3f::ZERO);
	const Vertex& v(vertices[idxV]);
	const Mesh::FaceIdxArr& vf(vertexFaces[idxV]);
	ASSERT(vf.GetSize() > 2);
	FOREACHPTR(pFaceIdx, vf) {
		const Face& face = faces[*pFaceIdx];
		const uint32_t idx(Mesh::FindVertex(face, idxV));
		ASSERT(idx != NO_ID);
		const Vertex& v1(vertices[face[(idx+1)%3]]);
		const Vertex& v2(vertices[face[(idx+2)%3]]);
		Kh += cotan(v1, v, v2)*(v2-v);
		Kh += cotan(v2, v, v1)*(v1-v);
		angle_sum += angle_from_cotan(v, v1, v2);
		area += region_area(v, v1, v2);
	}
	if (area <= 0)
		return false;
	const float Kg((2.f*FPI - angle_sum)/area);
	const float halfNormKh(norm(Kh)/(area*4));
	vertex_principal_curvatures(halfNormKh, Kg, K1, K2);
	return true;
}

/**
 * vertex_principal_curvatures:
 * @Kh: mean curvature
 * @Kg: Gaussian curvature
 * @K1: first principal curvature
 * @K2: second principal curvature
 *
 * Computes the principal curvatures at a point given the mean and
 * Gaussian curvatures at that point, calculated from the
 * discrete differential operators.
 *
 * The mean curvature can be computed as one-half the magnitude of the
 * vector computed by vertex_mean_curvature_normal().
 *
 * The Gaussian curvature can be computed with
 * vertex_gaussian_curvature().
 */
void MeshRefine::vertex_principal_curvatures(float Kh, float Kg, float& K1, float& K2) const
{
	float temp(Kh*Kh - Kg);
	if (temp < 0)
		temp = 0;
	temp = SQRT(temp);
	K1 = Kh + temp;
	K2 = Kh - temp;
}

/**
 * vertex_first_ring_area:
 * @v: a vertex index
 *
 * Computes the area of the first triangle ring at the given point.
 */
float MeshRefine::vertex_first_ring_area(VIndex idxV) const
{
	#if 1
	ASSERT(!vertexBoundary[idxV]);
	#else
	// this operator is not defined for boundary edges
	if (vertexBoundary[idxV])
		return false;
	#endif

	float area(0);
	const Mesh::FaceIdxArr& vf(vertexFaces[idxV]);
	ASSERT(vf.GetSize() > 2);
	FOREACHPTR(pFaceIdx, vf) {
		const Face& face = faces[*pFaceIdx];
		// computes the area of the triangle
		const Vertex& v (vertices[face[0]]);
		const Vertex& v1(vertices[face[1]]);
		const Vertex& v2(vertices[face[2]]);
		const Vertex normal((v1-v).cross(v2-v));
		area += norm(normal)*0.5f;
	}
	return area;
}

/**
 * umbrella_operator:
 * @v: a vertex index
 * @U: the discrete analog of the Laplacian
 *
 * Computes the discrete analog of the Laplacian using
 * the umbrella-operator on the first triangle ring at the given point.
 */
bool MeshRefine::umbrella_operator(VIndex idxV, Point3f& U) const
{
	#if 1
	ASSERT(!vertexBoundary[idxV]);
	#else
	// this operator is not defined for boundary edges
	if (vertexBoundary[idxV])
		return false;
	#endif

	U = Point3f::ZERO;
	const Mesh::FaceIdxArr& vf(vertexFaces[idxV]);
	ASSERT(vf.GetSize() > 2);
	std::unordered_set<VIndex> setVertices(vf.GetSize());
	FOREACHPTR(pFaceIdx, vf) {
		const Face& face = faces[*pFaceIdx];
		for (unsigned i=0; i<3; ++i) {
			const VIndex idx(face[i]);
			if (idx != idxV && setVertices.insert(idx).second)
				U += vertices[idx];
		}
	}
	ASSERT(vf.GetSize() == setVertices.size());
	const Vertex& v(vertices[idxV]);
	#if 1
	// normalize using vertex valence
	U = U/(float)vf.GetSize() - v;
	#else
	// normalize using vertex valence and 1st ring area
	U = (U/(float)vf.GetSize() - v) * vertex_first_ring_area(idxV)/*/3.f*/;
	#endif
	return true;
}
// same as above, but used to compute level 2;
// normalized as in "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004
bool MeshRefine::umbrella_operator(VIndex idxV, Point3f& U2, const Mesh::VertexArr& arrVertsU, const bool* valids) const
{
	#if 1
	ASSERT(!vertexBoundary[idxV]);
	#else
	// this operator is not defined for boundary edges
	if (vertexBoundary[idxV])
		return false;
	#endif

	U2 = Point3f::ZERO;
	const Mesh::FaceIdxArr& vf(vertexFaces[idxV]);
	ASSERT(vf.GetSize() > 2);
	std::unordered_set<VIndex> setVertices(vf.GetSize());
	float fNorm(0);
	FOREACHPTR(pFaceIdx, vf) {
		const Face& face = faces[*pFaceIdx];
		for (unsigned i=0; i<3; ++i) {
			const VIndex idx(face[i]);
			if (idx != idxV && setVertices.insert(idx).second) {
				if (!valids[idx])
					return false;
				U2 += arrVertsU[idx];
				fNorm += 1.f/(float)vertexFaces[idx].GetSize();
			}
		}
	}
	ASSERT(vf.GetSize() == setVertices.size());
	const Vertex& U(arrVertsU[idxV]);
	// normalize using vertices valence
	U2 = (U2/(float)vf.GetSize() - U) / (1.f + fNorm/(float)vf.GetSize());
	return true;
}
// compute level 1 and 2 of the Laplacian operator to compute the final smoothing gradient;
// (see page 105 of "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004)
Point3f MeshRefine::umbrella_operator_gradient(const Point3f& U, const Point3f& U2) const
{
	return U2*ratioRigidityElasticity - U*(1.f-ratioRigidityElasticity);
}


// given a vertex position and a projection camera, compute the projected position and its derivative
// returns the depth
template <typename TP, typename TX, typename T>
T MeshRefine::ProjectVertex(const TP* P, const TX* X, T* x, T* jacobian)
{
	#if 0
	// numerical differentiation
	T fjac[2*3];
	{
	const TX eps = 1.e-7;
	TX cX[3];
	memcpy(cX, X, sizeof(TX)*3);
	TX x0[2], x1[2];
	ProjectVertexVisualization_3x4_3_2(P, X, x0);
	for (int j=0; j<3; j++) {
		const TX temp = cX[j];
		const TX step = eps*MAXF(eps,abs(temp));
		cX[j] += step;
		ProjectVertexVisualization_3x4_3_2(P, cX, x1);
		cX[j] = temp;
		for (int i=0; i<2; i++)
			fjac[i*3+j] = (x1[i] - x0[i]) / step;
	}
	}
	#endif

	const TX&  x1(X[0]);
	const TX&  x2(X[1]);
	const TX&  x3(X[2]);

	const TP& p1_1(P[ 0]);
	const TP& p1_2(P[ 1]);
	const TP& p1_3(P[ 2]);
	const TP& p1_4(P[ 3]);
	const TP& p2_1(P[ 4]);
	const TP& p2_2(P[ 5]);
	const TP& p2_3(P[ 6]);
	const TP& p2_4(P[ 7]);
	const TP& p3_1(P[ 8]);
	const TP& p3_2(P[ 9]);
	const TP& p3_3(P[10]);
	const TP& p3_4(P[11]);

	const TP t5(p3_4+p3_1*x1+p3_2*x2+p3_3*x3);
	const TP t6(1.0/t5);
	const TP t10(p1_4+p1_1*x1+p1_2*x2+p1_3*x3);
	const TP t11(t10*t6);
	const TP t15(p2_4+p2_1*x1+p2_2*x2+p2_3*x3);
	const TP t16(t15*t6);
	x[0] = T(t11);
	x[1] = T(t16);
	if (jacobian) {
		jacobian[0] = T((p1_1-p3_1*t11)*t6);
		jacobian[1] = T((p1_2-p3_2*t11)*t6);
		jacobian[2] = T((p1_3-p3_3*t11)*t6);
		jacobian[3] = T((p2_1-p3_1*t16)*t6);
		jacobian[4] = T((p2_2-p3_2*t16)*t6);
		jacobian[5] = T((p2_3-p3_3*t16)*t6);
	}
	return T(t5);
}


// S T R U C T S ///////////////////////////////////////////////////

#pragma push_macro("LOG")
#undef LOG
#pragma push_macro("CHECK")
#undef CHECK
#pragma push_macro("ERROR")
#undef ERROR
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#pragma pop_macro("ERROR")
#pragma pop_macro("CHECK")
#pragma pop_macro("LOG")

namespace ceres {
class MeshProblem : public FirstOrderFunction
{
public:
	MeshProblem(MeshRefine& _refine) : refine(_refine), params(refine.vertices.GetSize()*3) {
		// init params
		FOREACH(i, refine.vertices)
			*((Point3d*)params.Begin()+i) = refine.vertices[i];
	}

	void ApplyParams() const {
		FOREACH(i, refine.vertices)
			refine.vertices[i] = *((Point3d*)params.Begin()+i);
	}
	void ApplyParams(const double* parameters) const {
		memcpy(params.Begin(), parameters, sizeof(double)*params.GetSize());
		ApplyParams();
	}

	bool Evaluate(const double* const parameters, double* cost, double* gradient) const {
		// update surface parameters
		ApplyParams(parameters);
		// evaluate residuals
		*cost = 0;
		if (!gradient) {
			refine.ScoreMesh(*cost);
			return true;
		}
		// evaluate residuals and gradients
		#ifdef MESHOPT_NUMERICAL
		refine.ScoreMesh(*cost, gradient, 1e-4);
		#else
		refine.ScoreMesh(*cost, gradient);
		#endif
		return true;
	}

	int NumParameters() const { return (int)params.GetSize(); }
	const double* GetParameters() const { return params.Begin(); }
	double* GetParameters() { return params.Begin(); }

protected:
	MeshRefine& refine;
	DoubleArr params;
};
} // namespace ceres

// optimize mesh using photo-consistency
bool Scene::RefineMesh(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews, float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nScales, float fScaleStep, float fRegularityWeight, unsigned nMaxFaceArea, float fConfidenceThreshold)
{
	MeshRefine refine(*this, fRegularityWeight, nResolutionLevel, nMinResolution, nMaxViews);

	// run the mesh optimization on multiple scales (coarse to fine)
	for (unsigned iter=0; iter<nScales; ++iter) {
		#if defined(MESHOPT_DEBUG) && MESHOPT_DEBUG>1
		refine._level = iter;
		refine._iteration = 0;
		#endif

		// init images
		const double scale(powi(fScaleStep, (int)(nScales-iter-1)));
		DEBUG_ULTIMATE("Refine mesh at: %.2f image scale", scale);
		//const double step(4); // => 16 area
		//refine.InitImages(scale, 0.12*step+0.2);
		refine.InitImages(scale);

		// extract array of triangles incident to each vertex
		refine.ListVertexFaces();

		// automatic mesh subdivision
		refine.SubdivideMesh(nMaxFaceArea, iter == 0 ? fDecimateMesh : 1.f, nCloseHoles, nEnsureEdgeSize);

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefine%u.ply", nScales-iter-1)));
		#endif

		// minimize
		if (fConfidenceThreshold >= -1.f) {
		//#if 0
		// DefineProblem
		ceres::MeshProblem* problemData(new ceres::MeshProblem(refine));
		ceres::GradientProblem problem(problemData);
		// SetMinimizerOptions
		ceres::GradientProblemSolver::Options options;
		if (VERBOSITY_LEVEL > 1) {
			options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
			options.minimizer_progress_to_stdout = true;
		} else {
			options.logging_type = ceres::LoggingType::SILENT;
			options.minimizer_progress_to_stdout = false;
		}
		options.function_tolerance = 1e-3;
		options.gradient_tolerance = 1e-7;
		options.max_num_line_search_step_size_iterations = 10;
		ceres::GradientProblemSolver::Summary summary;
		// SolveProblem
		ceres::Solve(options, problem, problemData->GetParameters(), &summary);
		DEBUG_ULTIMATE(summary.FullReport().c_str());
		switch (summary.termination_type) {
		case ceres::TerminationType::NO_CONVERGENCE:
			DEBUG_EXTRA("CERES: maximum number of iterations reached!");
		case ceres::TerminationType::CONVERGENCE:
		case ceres::TerminationType::USER_SUCCESS:
			break;
		default:
			VERBOSE("CERES surface refine error: %s!", summary.message.c_str());
			return false;
		}
		ASSERT(summary.IsSolutionUsable());
		problemData->ApplyParams();
		} else {
		//#else
		// loop a constant number of iterations and apply the gradient
		int iters(25);
		double gstep(0.05);
		if (fConfidenceThreshold < 0) {
			iters = FLOOR2INT(-fConfidenceThreshold);
			gstep = (-fConfidenceThreshold-(float)iters)*10;
		}
		Eigen::Matrix<double,Eigen::Dynamic,3> gradients(refine.vertices.GetSize(),3);
		for (int iter=0; iter<iters; ++iter) {
			// evaluate residuals and gradients
			double cost;
			#ifdef MESHOPT_NUMERICAL
			refine.ScoreMesh(cost, gradients.data(), 1e-4);
			#else
			refine.ScoreMesh(cost, gradients.data());
			#endif
			// apply gradients
			double gv(0);
			FOREACH(v, refine.vertices) {
				const Eigen::Vector3f delta((gradients.row(v)*gstep).cast<float>());
				Vertex& vert = refine.vertices[v];
				vert -= Vertex(delta);
				gv += gradients.row(v).norm();
			}
			DEBUG_EXTRA("% 2d. f: %g (%g)\tg: %g (%g - %g)\ts: %g", iter, cost, cost/refine.vertices.GetSize(), gradients.norm(), gradients.norm()/refine.vertices.GetSize(), gv/refine.vertices.GetSize(), gstep);
			gstep *= 0.99;
		}
		}
		//#endif

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefined%u.ply", nScales-iter-1)));
		#endif
	}

	return _OK;
} // RefineMesh
/*----------------------------------------------------------------*/
