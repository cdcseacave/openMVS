/*
* SceneRefine.cpp
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
#include "Scene.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to ensure edge size and improve vertex valence
// (should enable more stable flow)
#define MESHOPT_ENSUREEDGESIZE 1 // 0 - at all resolution

// uncomment to use constant z-buffer bias
// (should be enough, as the numerical error does not depend on the depth)
#define MESHOPT_DEPTHCONSTBIAS 0.05f


// S T R U C T S ///////////////////////////////////////////////////

typedef Mesh::Vertex Vertex;
typedef Mesh::VIndex VIndex;
typedef Mesh::Face Face;
typedef Mesh::FIndex FIndex;

struct MeshRefine {
	typedef double Real;

	typedef TPoint3<Real> Grad;
	typedef CLISTDEF0IDX(Grad,VIndex) GradArr;

	typedef std::unordered_set<FIndex> CameraFaces;
	typedef SEACAVE::cList<CameraFaces,const CameraFaces&,2> CameraFacesArr;

	typedef TImage<cuint32_t> FaceMap;
	typedef TImage<Point3f> BaryMap;

	// store necessary data about a view
	struct View {
		typedef TPoint2<float> Grad;
		typedef TImage<Grad> ImageGrad;
		Image32F image; // image pixels
		ImageGrad imageGrad; // image pixel gradients
		TImage<Real> imageMean; // image pixels mean
		TImage<Real> imageVar; // image pixels variance
		FaceMap faceMap; // remember for each pixel what face projects there
		DepthMap depthMap; // depth-map
		BaryMap baryMap; // barycentric coordinates
	};
	typedef SEACAVE::cList<View,const View&,2> ViewsArr;

	// used to render a mesh for optimization
	struct RasterMeshData {
		const Camera& P;
		FaceMap& faceMap;
		DepthMap& depthMap;
		BaryMap& baryMap;
		const Point2f* pti;
		Point3 normalPlane;
		FIndex idxFace;
		inline void operator()(const ImageRef& pt) {
			if (!depthMap.isInsideWithBorder<int,4>(pt))
				return;
			const float z((float)INVERT(normalPlane.dot(P.TransformPointI2C(Point2(pt)))));
			ASSERT(z > 0);
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				faceMap(pt) = idxFace;
				baryMap(pt) = CorrectBarycentricCoordinates(BarycentricCoordinatesUV(pti[0], pti[1], pti[2], Point2f(pt)));
			}
		}
		inline void operator()(const ImageRef& pt, float z) {
			if (!depthMap.isInsideWithBorder<int,4>(pt))
				return;
			ASSERT(z > 0);
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				faceMap(pt) = idxFace;
				baryMap(pt) = CorrectBarycentricCoordinates(BarycentricCoordinatesUV(pti[0], pti[1], pti[2], Point2f(pt)));
			}
		}
	};


public:
	MeshRefine(Scene& _scene, Real _weightRegularity=1.5f, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640, unsigned nMaxViews=8, unsigned nMaxThreads=1);
	~MeshRefine();

	bool InitImages(Real scale, Real sigma=0);

	void ListVertexFacesPre();
	void ListVertexFacesPost();
	void ListCameraFaces();

	void ListFaceAreas(Mesh::AreaArr& maxAreas);
	void SubdivideMesh(uint32_t maxArea, float fDecimate=1.f, unsigned nCloseHoles=15, unsigned nEnsureEdgeSize=1);

	double ScoreMesh(double* gradients);

	// given a vertex position and a projection camera, compute the projected position and its derivative
	template <typename TP, typename TX, typename T, typename TJ>
	static T ProjectVertex(const TP* P, const TX* X, T* x, TJ* jacobian=NULL);

	static bool IsDepthSimilar(const DepthMap& depthMap, const Point2f& pt, Depth z);
	static void ProjectMesh(
		const Mesh::VertexArr& vertices, const Mesh::FaceArr& faces, const CameraFaces& cameraFaces,
		const Camera& camera, const Image8U::Size& size,
		DepthMap& depthMap, FaceMap& faceMap, BaryMap& baryMap);
	static void ImageMeshWarp(
		const DepthMap& depthMapA, const Camera& cameraA,
		const DepthMap& depthMapB, const Camera& cameraB,
		const Image32F& imageB, Image32F& imageA, Image8U& mask);
	static void ComputeLocalVariance(
		const Image32F& image, const Image8U& mask,
		TImage<Real>& imageMean, TImage<Real>& imageVar);
	static float ComputeLocalZNCC(
		const Image32F& imageA, const TImage<Real>& imageMeanA, const TImage<Real>& imageVarA,
		const Image32F& imageB, const TImage<Real>& imageMeanB, const TImage<Real>& imageVarB,
		const Image8U& mask, TImage<Real>& imageZNCC, TImage<Real>& imageDZNCC);
	static void ComputePhotometricGradient(
		const Mesh::FaceArr& faces, const Mesh::NormalArr& normals,
		const DepthMap& depthMapA, const FaceMap& faceMapA, const BaryMap& baryMapA, const Camera& cameraA,
		const Camera& cameraB, const View& viewB,
		const TImage<Real>& imageDZNCC, const Image8U& mask, GradArr& photoGrad, FloatArr& photoGradNorm, Real RegularizationScale);
	static float ComputeSmoothnessGradient1(
		const Mesh::VertexArr& vertices, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
		GradArr& smoothGrad1, VIndex idxStart, VIndex idxEnd);
	static void ComputeSmoothnessGradient2(
		const GradArr& smoothGrad1, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
		GradArr& smoothGrad2, VIndex idxStart, VIndex idxEnd);

	static void* ThreadWorkerTmp(void*);
	void ThreadWorker();
	void WaitThreadWorkers(size_t nJobs);
	void ThSelectNeighbors(uint32_t idxImage, std::unordered_set<uint64_t>& mapPairs, unsigned nMaxViews);
	void ThInitImage(uint32_t idxImage, Real scale, Real sigma);
	void ThProjectMesh(uint32_t idxImage, const CameraFaces& cameraFaces);
	void ThProcessPair(uint32_t idxImageA, uint32_t idxImageB);
	void ThSmoothVertices1(VIndex idxStart, VIndex idxEnd);
	void ThSmoothVertices2(VIndex idxStart, VIndex idxEnd);

public:
	const Real weightRegularity; // a scalar regularity weight to balance between photo-consistency and regularization terms
	const unsigned nResolutionLevel; // how many times to scale down the images before mesh optimization
	const unsigned nMinResolution; // how many times to scale down the images before mesh optimization

	Scene& scene; // the mesh vertices and faces

	// gradient related
	float scorePhoto;
	float scoreSmooth;
	GradArr photoGrad;
	FloatArr photoGradNorm;
	GradArr smoothGrad1;
	GradArr smoothGrad2;

	// valid after ListCameraFaces()
	Mesh::NormalArr& faceNormals; // normals corresponding to each face

	// valid the entire time, but changes
	Mesh::VertexArr& vertices;
	Mesh::FaceArr& faces;
	Mesh::VertexVerticesArr& vertexVertices; // for each vertex, the list of adjacent vertices
	Mesh::VertexFacesArr& vertexFaces; // for each vertex, the list of faces containing it
	BoolArr& vertexBoundary; // for each vertex, stores if it is at the boundary or not

	// constant the entire time
	ImageArr& images;
	ViewsArr views; // views' data
	PairIdxArr pairs; // image pairs used to refine the mesh

	// multi-threading
	static SEACAVE::EventQueue events; // internal events queue (processed by the working threads)
	static SEACAVE::cList<SEACAVE::Thread> threads; // worker threads
	static CriticalSection cs; // mutex
	static Semaphore sem; // signal job end

	static const int HalfSize = 3; // half window size used to compute ZNCC
};

enum EVENT_TYPE {
	EVT_JOB = 0,
	EVT_CLOSE,
};

class EVTClose : public Event
{
public:
	EVTClose() : Event(EVT_CLOSE) {}
};
class EVTSelectNeighbors : public Event
{
public:
	uint32_t idxImage;
	std::unordered_set<uint64_t>& mapPairs;
	unsigned nMaxViews;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThSelectNeighbors(idxImage, mapPairs, nMaxViews);
		return true;
	}
	EVTSelectNeighbors(uint32_t _idxImage, std::unordered_set<uint64_t>& _mapPairs, unsigned _nMaxViews) : Event(EVT_JOB), idxImage(_idxImage), mapPairs(_mapPairs), nMaxViews(_nMaxViews) {}
};
class EVTInitImage : public Event
{
public:
	uint32_t idxImage;
	double scale, sigma;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThInitImage(idxImage, scale, sigma);
		return true;
	}
	EVTInitImage(uint32_t _idxImage, double _scale, double _sigma) : Event(EVT_JOB), idxImage(_idxImage), scale(_scale), sigma(_sigma) {}
};
class EVTProjectMesh : public Event
{
public:
	uint32_t idxImage;
	const MeshRefine::CameraFaces& cameraFaces;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThProjectMesh(idxImage, cameraFaces);
		return true;
	}
	EVTProjectMesh(uint32_t _idxImage, const MeshRefine::CameraFaces& _cameraFaces) : Event(EVT_JOB), idxImage(_idxImage), cameraFaces(_cameraFaces) {}
};
class EVTProcessPair : public Event
{
public:
	uint32_t idxImageA, idxImageB;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThProcessPair(idxImageA, idxImageB);
		return true;
	}
	EVTProcessPair(uint32_t _idxImageA, uint32_t _idxImageB) : Event(EVT_JOB), idxImageA(_idxImageA), idxImageB(_idxImageB) {}
};
class EVTSmoothVertices1 : public Event
{
public:
	VIndex idxStart, idxEnd;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThSmoothVertices1(idxStart, idxEnd);
		return true;
	}
	EVTSmoothVertices1(VIndex _idxStart, VIndex _idxEnd) : Event(EVT_JOB), idxStart(_idxStart), idxEnd(_idxEnd) {}
};
class EVTSmoothVertices2 : public Event
{
public:
	VIndex idxStart, idxEnd;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThSmoothVertices2(idxStart, idxEnd);
		return true;
	}
	EVTSmoothVertices2(VIndex _idxStart, VIndex _idxEnd) : Event(EVT_JOB), idxStart(_idxStart), idxEnd(_idxEnd) {}
};

SEACAVE::EventQueue MeshRefine::events;
SEACAVE::cList<SEACAVE::Thread> MeshRefine::threads;
CriticalSection MeshRefine::cs;
Semaphore MeshRefine::sem;

MeshRefine::MeshRefine(Scene& _scene, Real _weightRegularity, unsigned _nResolutionLevel, unsigned _nMinResolution, unsigned nMaxViews, unsigned nMaxThreads)
	:
	weightRegularity(_weightRegularity),
	nResolutionLevel(_nResolutionLevel),
	nMinResolution(_nMinResolution),
	scene(_scene),
	faceNormals(_scene.mesh.faceNormals),
	vertices(_scene.mesh.vertices),
	faces(_scene.mesh.faces),
	vertexVertices(_scene.mesh.vertexVertices),
	vertexFaces(_scene.mesh.vertexFaces),
	vertexBoundary(_scene.mesh.vertexBoundary),
	images(_scene.images)
{
	// start worker threads
	ASSERT(nMaxThreads > 0);
	ASSERT(threads.IsEmpty());
	threads.Resize(nMaxThreads);
	FOREACHPTR(pThread, threads)
		pThread->start(ThreadWorkerTmp, this);
	// keep only best neighbor views for each image
	std::unordered_set<uint64_t> mapPairs;
	mapPairs.reserve(images.GetSize()*nMaxViews);
	ASSERT(events.IsEmpty());
	FOREACH(idxImage, images)
		events.AddEvent(new EVTSelectNeighbors(idxImage, mapPairs, nMaxViews));
	WaitThreadWorkers(images.GetSize());
	pairs.Reserve(mapPairs.size());
	for (uint64_t pair: mapPairs)
		pairs.AddConstruct(pair);
}
MeshRefine::~MeshRefine()
{
	// wait for the working threads to close
	FOREACH(i, threads)
		events.AddEvent(new EVTClose());
	FOREACHPTR(pThread, threads)
		pThread->join();
	scene.mesh.ReleaseExtra();
}

// load and initialize all images at the given scale
// and compute the gradient for each input image
// optional: blur them using the given sigma
bool MeshRefine::InitImages(Real scale, Real sigma)
{
	views.Resize(images.GetSize());
	ASSERT(events.IsEmpty());
	FOREACH(idxImage, images)
		events.AddEvent(new EVTInitImage(idxImage, scale, sigma));
	WaitThreadWorkers(images.GetSize());
	return true;
}

// extract array of triangles incident to each vertex
// and check each vertex if it is at the boundary or not
void MeshRefine::ListVertexFacesPre()
{
	scene.mesh.EmptyExtra();
	scene.mesh.ListIncidenteFaces();
}
void MeshRefine::ListVertexFacesPost()
{
	scene.mesh.ListIncidenteVertices();
	scene.mesh.ListBoundaryVertices();
}

// extract array of faces viewed by each image
void MeshRefine::ListCameraFaces()
{
	// extract array of faces viewed by each camera
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
	#if 0 && !defined(_RELEASE)
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
		const Frustum frustum(Frustum::MATRIX3x4(((PMatrix::CEMatMap)imageData.camera.P).cast<float>()), (float)imageData.width, (float)imageData.height);
		octree.Traverse(frustum, inserter);
	}
	}

	// project mesh to each camera plane
	ASSERT(events.IsEmpty());
	FOREACH(idxImage, images)
		events.AddEvent(new EVTProjectMesh(idxImage, arrCameraFaces[idxImage]));
	WaitThreadWorkers(images.GetSize());
}

// compute for each face the projection area as the maximum area in both images of a pair
// (make sure ListCameraFaces() was called before)
void MeshRefine::ListFaceAreas(Mesh::AreaArr& maxAreas)
{
	ASSERT(maxAreas.IsEmpty());
	// for each image, compute the projection area of visible faces
	typedef cList<Mesh::AreaArr> ImageAreaArr;
	ImageAreaArr viewAreas(images.GetSize());
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		Mesh::AreaArr& areas = viewAreas[idxImage];
		areas.Resize(faces.GetSize());
		areas.Memset(0);
		const FaceMap& faceMap = views[idxImage].faceMap;
		// compute area covered by all vertices (incident faces) viewed by this image
		for (int j=0; j<faceMap.rows; ++j) {
			for (int i=0; i<faceMap.cols; ++i) {
				const FIndex idxFace(faceMap(j,i));
				ASSERT((idxFace == NO_ID && views[idxImage].depthMap(j,i) == 0) || (idxFace != NO_ID && views[idxImage].depthMap(j,i) > 0));
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
		const Mesh::AreaArr& areasA = viewAreas[pPair->i];
		const Mesh::AreaArr& areasB = viewAreas[pPair->j];
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
	Mesh::AreaArr maxAreas;

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
			ListVertexFacesPre();
		} else {
			// extract array of faces viewed by each camera
			ListCameraFaces();

			// estimate the faces' area that have big projection areas in both images of a pair
			ListFaceAreas(maxAreas);
			ASSERT(!maxAreas.IsEmpty());

			const float maxArea((float)(maxArea > 0 ? maxArea : 64));
			const float medianArea(6.f*(float)Mesh::AreaArr(maxAreas).GetMedian());
			if (medianArea < maxArea) {
				maxAreas.Empty();

				// decimate to the auto detected resolution
				scene.mesh.Clean(MAXF(0.1f, medianArea/maxArea), 0, false, nCloseHoles, 0);

				#ifdef MESHOPT_ENSUREEDGESIZE
				// make sure there are no edges too small or too long
				if (nEnsureEdgeSize > 0 && bNoSimplification)
					scene.mesh.EnsureEdgeSize();
				#endif

				// re-map vertex and camera faces
				ListVertexFacesPre();
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

	// subdivide mesh faces if its projection area is bigger than the given number of pixels
	const size_t numVertsOld(vertices.GetSize());
	const size_t numFacesOld(faces.GetSize());
	scene.mesh.SubdivideMesh(maxAreas, maxArea);

	#ifdef MESHOPT_ENSUREEDGESIZE
	// make sure there are no edges too small or too long
	#if MESHOPT_ENSUREEDGESIZE==1
	if ((nEnsureEdgeSize == 1 && !bNoDecimation) || nEnsureEdgeSize > 1)
	#endif
		scene.mesh.EnsureEdgeSize();
	#endif

	// re-map vertex and camera faces
	ListVertexFacesPre();

	DEBUG_EXTRA("Mesh subdivided: %u/%u -> %u/%u vertices/faces", numVertsOld, numFacesOld, vertices.GetSize(), faces.GetSize());

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 3)
		scene.mesh.Save(MAKE_PATH("MeshSubdivided.ply"));
	#endif
}


// score mesh using photo-consistency
// and compute vertices gradient using analytical method
double MeshRefine::ScoreMesh(double* gradients)
{
	// extract array of faces viewed by each camera
	ListCameraFaces();

	// compute face normals
	scene.mesh.ComputeNormalFaces();

	// for each pair of images, compute a photo-consistency score
	// between the reference image and the pixels of the second image
	// projected in the reference image through the mesh surface
	scorePhoto = 0;
	photoGrad.Resize(vertices.GetSize());
	photoGrad.Memset(0);
	photoGradNorm.Resize(vertices.GetSize());
	photoGradNorm.Memset(0);
	ASSERT(events.IsEmpty());
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		for (int ip=0; ip<2; ++ip)
			events.AddEvent(ip ? new EVTProcessPair(pPair->j,pPair->i) : new EVTProcessPair(pPair->i,pPair->j));
	}
	WaitThreadWorkers(pairs.GetSize()*2);

	// loop through all vertices and compute the smoothing score
	scoreSmooth = 0;
	const VIndex idxStep((vertices.GetSize()+(VIndex)threads.GetSize()-1)/(VIndex)threads.GetSize());
	smoothGrad1.Resize(vertices.GetSize());
	{
	ASSERT(events.IsEmpty());
	VIndex idx(0);
	while (idx<vertices.GetSize()) {
		const VIndex idxNext(MINF(idx+idxStep, vertices.GetSize()));
		events.AddEvent(new EVTSmoothVertices1(idx, idxNext));
		idx = idxNext;
	}
	WaitThreadWorkers(threads.GetSize());
	}
	// loop through all vertices and compute the smoothing gradient
	smoothGrad2.Resize(vertices.GetSize());
	{
	ASSERT(events.IsEmpty());
	VIndex idx(0);
	while (idx<vertices.GetSize()) {
		const VIndex idxNext(MINF(idx+idxStep, vertices.GetSize()));
		events.AddEvent(new EVTSmoothVertices2(idx, idxNext));
		idx = idxNext;
	}
	WaitThreadWorkers(threads.GetSize());
	}

	// set the final gradient as the combination of photometric and smoothness gradients
	FOREACH(v, vertices)
		((Point3d*)gradients)[v] = photoGradNorm[v] > 0 ?
			Cast<double>(photoGrad[v]/photoGradNorm[v] + smoothGrad2[v]*weightRegularity) :
			Cast<double>(smoothGrad2[v]*weightRegularity);
	return 0.15f*scorePhoto + 0.1f*scoreSmooth;
}


// given a vertex position and a projection camera, compute the projected position and its derivative
// returns the depth
template <typename TP, typename TX, typename T, typename TJ>
T MeshRefine::ProjectVertex(const TP* P, const TX* X, T* x, TJ* jacobian)
{
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
		jacobian[0] = TJ((p1_1-p3_1*t11)*t6);
		jacobian[1] = TJ((p1_2-p3_2*t11)*t6);
		jacobian[2] = TJ((p1_3-p3_3*t11)*t6);
		jacobian[3] = TJ((p2_1-p3_1*t16)*t6);
		jacobian[4] = TJ((p2_2-p3_2*t16)*t6);
		jacobian[5] = TJ((p2_3-p3_3*t16)*t6);
	}
	return T(t5);
}


// check if any of the depths surrounding the given coordinate is similar to the given value
bool MeshRefine::IsDepthSimilar(const DepthMap& depthMap, const Point2f& pt, Depth z)
{
	const ImageRef tl(FLOOR2INT(pt));
	for (int x=0; x<2; ++x) {
		for (int y=0; y<2; ++y) {
			const ImageRef ir(tl.x+x, tl.y+y);
			if (!depthMap.isInsideWithBorder<int,3>(ir))
				continue;
			const Depth& depth = depthMap(ir);
			#ifndef MESHOPT_DEPTHCONSTBIAS
			if (depth <= 0 || ABS(depth-z) > z*0.01f /*!IsDepthSimilar(depth, z, 0.01f)*/)
			#else
			if (depth <= 0 || depth+MESHOPT_DEPTHCONSTBIAS < z)
			#endif
				continue;
			return true;
		}
	}
	return false;
}

// project mesh to the given camera plane
void MeshRefine::ProjectMesh(
	const Mesh::VertexArr& vertices, const Mesh::FaceArr& faces, const CameraFaces& cameraFaces,
	const Camera& camera, const Image8U::Size& size,
	DepthMap& depthMap, FaceMap& faceMap, BaryMap& baryMap)
{
	// init view data
	faceMap.create(size);
	depthMap.create(size);
	baryMap.create(size);
	// project all triangles on this image and keep the closest ones
	faceMap.memset((uint8_t)NO_ID);
	depthMap.memset(0);
	baryMap.memset(0);
	Point3 ptc[3]; Point2f pti[3];
	RasterMeshData data = {camera, faceMap, depthMap, baryMap, pti};
	for (auto idxFace : cameraFaces) {
		data.idxFace = idxFace;
		const Face& facet = faces[idxFace];
		for (int v=0; v<3; ++v) {
			const Vertex& pt = vertices[facet[v]];
			ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt));
			pti[v] = camera.TransformPointC2I(ptc[v]);
			// skip face if not completely inside
			if (!depthMap.isInsideWithBorder<float,3>(pti[v]))
				goto CONTIUE2NEXTFACE;
		}
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
}

// project image from view B to view A through the mesh;
// the projected image is stored in imageA
// (imageAB is assumed to be initialize to the right size)
void MeshRefine::ImageMeshWarp(
	const DepthMap& depthMapA, const Camera& cameraA,
	const DepthMap& depthMapB, const Camera& cameraB,
	const Image32F& imageB, Image32F& imageA, Image8U& mask)
{
	ASSERT(!imageA.empty());
	typedef Sampler::Linear<float> Sampler;
	const Sampler sampler{};
	mask.create(imageA.size());
	mask.memset(0);
	for (int j=0; j<depthMapA.rows; ++j) {
		for (int i=0; i<depthMapA.cols; ++i) {
			const Depth& depthA = depthMapA(j,i);
			if (depthA <= 0)
				continue;
			const Point3 X(cameraA.TransformPointI2W(Point3(i,j,depthA)));
			const Point3f ptC(cameraB.TransformPointW2C(X));
			const Point2f pt(cameraB.TransformPointC2I(ptC));
			if (!IsDepthSimilar(depthMapB, pt, ptC.z))
				continue;
			imageA(j,i) = imageB.sample<Sampler,Sampler::Type>(sampler, pt);
			mask(j,i) = 1;
		}
	}
}

// compute local variance for each image pixel
void MeshRefine::ComputeLocalVariance(const Image32F& image, const Image8U& mask, TImage<Real>& imageMean, TImage<Real>& imageVar)
{
	ASSERT(image.size() == mask.size());
	imageMean.create(image.size());
	imageVar.create(image.size());
	imageMean.memset(0);
	imageVar.memset(0);
	const int RowsEnd(image.rows-HalfSize);
	const int ColsEnd(image.cols-HalfSize);
	const int n(SQUARE(HalfSize*2+1));
	Image64F imageSum, imageSumSq;
	#if CV_MAJOR_VERSION > 2
	cv::integral(image, imageSum, imageSumSq, CV_64F, CV_64F);
	#else
	cv::integral(image, imageSum, imageSumSq, CV_64F);
	#endif
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r,c))
				continue;
			imageMean(r,c) = (Real)((
				imageSum(r+HalfSize+1, c+HalfSize+1) -
				imageSum(r+HalfSize+1, c-HalfSize  ) -
				imageSum(r-HalfSize,   c+HalfSize+1) +
				imageSum(r-HalfSize,   c-HalfSize  ) ) * (1.0/(double)n));
		}
	}
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r,c))
				continue;
			const Real sumSq = (Real)((
				imageSumSq(r+HalfSize+1, c+HalfSize+1) -
				imageSumSq(r+HalfSize+1, c-HalfSize  ) -
				imageSumSq(r-HalfSize,   c+HalfSize+1) +
				imageSumSq(r-HalfSize,   c-HalfSize  ) ) * (1.0/(double)n));
			imageVar(r,c) = MAXF(sumSq-SQUARE(imageMean(r,c)), Real(0.0001));
		}
	}
}

// compute local ZNCC and its gradient for each image pixel
float MeshRefine::ComputeLocalZNCC(
	const Image32F& imageA, const TImage<Real>& imageMeanA, const TImage<Real>& imageVarA,
	const Image32F& imageB, const TImage<Real>& imageMeanB, const TImage<Real>& imageVarB,
	const Image8U& mask, TImage<Real>& imageZNCC, TImage<Real>& imageDZNCC)
{
	ASSERT(imageA.size() == mask.size() && imageB.size() == mask.size() && !mask.empty());
	float score(0);
	imageZNCC.create(mask.size());
	imageDZNCC.create(mask.size());
	const int RowsEnd(mask.rows-HalfSize);
	const int ColsEnd(mask.cols-HalfSize);
	const int n(SQUARE(HalfSize*2+1));
	imageZNCC.memset(0);
	Image32F imageAB;
	cv::multiply(imageA, imageB, imageAB);
	Image64F imageABSum;
	cv::integral(imageAB, imageABSum, CV_64F);
	TImage<Real> imageInvSqrtVAVB(mask.size());
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r, c))
				continue;
			const Real cv = (Real)((
				imageABSum(r+HalfSize+1, c+HalfSize+1) -
				imageABSum(r+HalfSize+1, c-HalfSize  ) -
				imageABSum(r-HalfSize,   c+HalfSize+1) +
				imageABSum(r-HalfSize,   c-HalfSize  ) ) * (1.0/(double)n));
			const Real invSqrtVAVB(Real(1)/SQRT(imageVarA(r,c)*imageVarB(r,c)));
			imageZNCC(r,c) = (cv - imageMeanA(r,c)*imageMeanB(r,c)) * invSqrtVAVB;
			imageInvSqrtVAVB(r,c) = invSqrtVAVB;
		}
	}
	imageDZNCC.memset(0);
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r,c))
				continue;
			#if 1
			const Real ZNCC(imageZNCC(r,c));
			const Real invSqrtVAVB(imageInvSqrtVAVB(r,c));
			const Real ZNCCinvVB(ZNCC/imageVarB(r,c));
			const Real dZNCC((Real)imageA(r,c)*invSqrtVAVB - (Real)imageB(r,c)*ZNCCinvVB + imageMeanB(r,c)*ZNCCinvVB - imageMeanA(r,c)*invSqrtVAVB);
			#else
			Real sumA(0), sumB(0), sumC(0);
			int n(0);
			for (int i=-HalfSize; i<=HalfSize; ++i) {
				const int rw(r+i);
				for (int j=-HalfSize; j<=HalfSize; ++j) {
					const int cw(c+j);
					if (!mask(rw,cw))
						continue;
					const Real invSqrtVAVB(imageInvSqrtVAVB(rw,cw));
					const Real ZNCCinvVB(imageZNCC(rw,cw)/imageVarB(rw,cw));
					sumA += invSqrtVAVB;
					sumB -= ZNCCinvVB;
					sumC += imageMeanB(rw,cw)*ZNCCinvVB - imageMeanA(rw,cw)*invSqrtVAVB;
					++n;
				}
			}
			if (n == 0)
				continue;
			const Real dZNCC((sumA*imageA(r,c) + sumB*imageB(r,c) + sumC)/(Real)n);
			const Real ZNCC(imageZNCC(r,c));
			#endif
			const Real minVAVB(MINF(imageVarA(r,c),imageVarB(r,c)));
			const Real ReliabilityFactor(minVAVB/(minVAVB+Real(0.0015)));
			imageDZNCC(r,c) = -ReliabilityFactor*dZNCC;
			score += (float)(ReliabilityFactor*(Real(1)-ZNCC));
		}
	}
	return score;
}

// compute the photometric gradient for all vertices seen by an image pair
void MeshRefine::ComputePhotometricGradient(
	const Mesh::FaceArr& faces, const Mesh::NormalArr& normals,
	const DepthMap& depthMapA, const FaceMap& faceMapA, const BaryMap& baryMapA, const Camera& cameraA,
	const Camera& cameraB, const View& viewB,
	const TImage<Real>& imageDZNCC, const Image8U& mask, GradArr& photoGrad, FloatArr& photoGradNorm, Real RegularizationScale)
{
	ASSERT(faces.GetSize() == normals.GetSize() && !faces.IsEmpty());
	ASSERT(depthMapA.size() == mask.size() && faceMapA.size() == mask.size() && baryMapA.size() == mask.size() && imageDZNCC.size() == mask.size() && !mask.empty());
	ASSERT(viewB.image.size() == viewB.imageGrad.size() && !viewB.image.empty());
	const int RowsEnd(mask.rows-HalfSize);
	const int ColsEnd(mask.cols-HalfSize);
	typedef Sampler::Linear<View::Grad::Type> Sampler;
	const Sampler sampler{};
	TMatrix<Real,2,3> xJac;
	Point2f xB;
	GradArr _photoGrad(photoGrad.GetSize());
	UnsignedArr photoGradPixels(photoGrad.GetSize());
	_photoGrad.Memset(0);
	photoGradPixels.Memset(0);
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r,c))
				continue;
			const FIndex idxFace(faceMapA(r,c));
			ASSERT(idxFace != NO_ID);
			const Grad N(normals[idxFace]);
			const Point3 rayA(cameraA.RayPoint(Point2(c,r)));
			const Grad dA(normalized(rayA));
			const Real Nd(N.dot(dA));
			#if 1
			if (Nd > -0.1)
				continue;
			#endif
			const Depth depthA(depthMapA(r,c));
			ASSERT(depthA > 0);
			const Point3 X(rayA*REAL(depthA)+cameraA.C);
			// project point in second image and
			// projection Jacobian matrix in the second image of the 3D point on the surface
			const float depthB(ProjectVertex(cameraB.P.val, X.ptr(), xB.ptr(), xJac.val));
			ASSERT(depthB > 0);
			// compute gradient in image B
			const TMatrix<Real,1,2> gB(viewB.imageGrad.sample<Sampler,View::Grad>(sampler, xB));
			// compute gradient scale
			const Real dZNCC(imageDZNCC(r,c));
			const Real sg((gB*(xJac*(const TMatrix<Real,3,1>&)dA))(0)*dZNCC*RegularizationScale/Nd);
			// add gradient to the three vertices
			const Face& face(faces[idxFace]);
			const Point3f& b(baryMapA(r,c));
			for (int v=0; v<3; ++v) {
				const Grad g(N*(sg*(Real)b[v]));
				const VIndex idxVert(face[v]);
				_photoGrad[idxVert] += g;
				++photoGradPixels[idxVert];
			}
		}
	}
	Lock l(cs);
	FOREACH(i, photoGrad) {
		if (photoGradPixels[i] > 0) {
			photoGrad[i] += _photoGrad[i];
			photoGradNorm[i] += 1.f;
		}
	}
}

float MeshRefine::ComputeSmoothnessGradient1(
	const Mesh::VertexArr& vertices, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
	GradArr& smoothGrad1, VIndex idxStart, VIndex idxEnd)
{
	ASSERT(!vertices.IsEmpty() && vertices.GetSize() == vertexVertices.GetSize() && vertices.GetSize() == smoothGrad1.GetSize());
	float score(0);
	for (VIndex idxV=idxStart; idxV<idxEnd; ++idxV) {
		Grad& grad = smoothGrad1[idxV];
		#if 1
		if (vertexBoundary[idxV]) {
			grad = Grad::ZERO;
			continue;
		}
		#endif
		const Mesh::VertexIdxArr& verts = vertexVertices[idxV];
		if (verts.IsEmpty()) {
			grad = Grad::ZERO;
			continue;
		}
		const Real nrm(Real(1)/(Real)verts.GetSize());
		grad = vertices[idxV];
		FOREACH(v, verts)
			grad -= Cast<Real>(vertices[verts[v]]) * nrm;
		const float regularityScore((float)norm(grad));
		ASSERT(ISFINITE(regularityScore));
		score += regularityScore;
	}
	return score;
}
void MeshRefine::ComputeSmoothnessGradient2(
	const GradArr& smoothGrad1, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
	GradArr& smoothGrad2, VIndex idxStart, VIndex idxEnd)
{
	ASSERT(!smoothGrad1.IsEmpty() && smoothGrad1.GetSize() == vertexVertices.GetSize() && smoothGrad1.GetSize() == smoothGrad2.GetSize());
	for (VIndex idxV=idxStart; idxV<idxEnd; ++idxV) {
		Grad& grad = smoothGrad2[idxV];
		#if 1
		if (vertexBoundary[idxV]) {
			grad = Grad::ZERO;
			continue;
		}
		#endif
		const Mesh::VertexIdxArr& verts = vertexVertices[idxV];
		if (verts.IsEmpty()) {
			grad = Grad::ZERO;
			continue;
		}
		const Real numV((Real)verts.GetSize());
		const Real nrm(Real(1)/numV);
		grad = smoothGrad1[idxV];
		Real w(1);
		FOREACH(v, verts) {
			const VIndex idxVert(verts[v]);
			grad -= smoothGrad1[idxVert] * nrm;
			const VIndex numVert(vertexVertices[idxVert].GetSize());
			if (numVert > 0)
				w += numV / (Real)numVert;
		}
		grad /= w;
	}
}


void* MeshRefine::ThreadWorkerTmp(void* arg) {
	MeshRefine& refine = *((MeshRefine*)arg);
	refine.ThreadWorker();
	return NULL;
}
void MeshRefine::ThreadWorker()
{
	while (true) {
		CAutoPtr<Event> evt(events.GetEvent());
		switch (evt->GetID()) {
		case EVT_JOB:
			evt->Run(this);
			break;
		case EVT_CLOSE:
			return;
		default:
			ASSERT("Should not happen!" == NULL);
		}
		sem.Signal();
	}
}
void MeshRefine::WaitThreadWorkers(size_t nJobs)
{
	while (nJobs-- > 0)
		sem.Wait();
	ASSERT(events.IsEmpty());
}
void MeshRefine::ThSelectNeighbors(uint32_t idxImage, std::unordered_set<uint64_t>& mapPairs, unsigned nMaxViews)
{
	// keep only best neighbor views
	const float fMinArea(0.12f);
	const float fMinScale(0.2f), fMaxScale(3.2f);
	const float fMinAngle(FD2R(3)), fMaxAngle(FD2R(45));
	const Image& imageData = images[idxImage];
	if (!imageData.IsValid())
		return;
	ViewScoreArr neighbors(imageData.neighbors);
	Scene::FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews);
	Lock l(cs);
	FOREACHPTR(pNeighbor, neighbors) {
		ASSERT(images[pNeighbor->idx.ID].IsValid());
		mapPairs.insert(MakePairIdx((uint32_t)idxImage, pNeighbor->idx.ID));
	}
}
void MeshRefine::ThInitImage(uint32_t idxImage, Real scale, Real sigma)
{
	Image& imageData = images[idxImage];
	if (!imageData.IsValid())
		return;
	// load and init image
	unsigned level(nResolutionLevel);
	const unsigned imageSize(imageData.RecomputeMaxResolution(level, nMinResolution));
	if ((imageData.image.empty() || MAXF(imageData.width,imageData.height) != imageSize) && FAILED(imageData.ReloadImage(imageSize)))
		ABORT("can not load image");
	View& view = views[idxImage];
	Image32F& img = view.image;
	imageData.image.toGray(img, cv::COLOR_BGR2GRAY, true);
	imageData.image.release();
	if (sigma > 0)
		cv::GaussianBlur(img, img, cv::Size(), sigma);
	if (scale < 1.0) {
		cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_LINEAR);
		imageData.width = img.width(); imageData.height = img.height();
	}
	imageData.UpdateCamera(scene.platforms);
	// compute image mean and variance
	ComputeLocalVariance(img, Image8U(img.size(), 1), view.imageMean, view.imageVar);
	// compute image gradient
	typedef View::Grad::Type GradType;
	TImage<GradType> grad[2];
	#if 0
	cv::Sobel(img, grad[0], cv::DataType<GradType>::type, 1, 0, 3, 1.0/8.0);
	cv::Sobel(img, grad[1], cv::DataType<GradType>::type, 0, 1, 3, 1.0/8.0);
	#elif 1
	const TMatrix<GradType,3,5> kernel(CreateDerivativeKernel3x5());
	cv::filter2D(img, grad[0], cv::DataType<GradType>::type, kernel);
	cv::filter2D(img, grad[1], cv::DataType<GradType>::type, kernel.t());
	#else
	const TMatrix<GradType,5,7> kernel(CreateDerivativeKernel5x7());
	cv::filter2D(img, grad[0], cv::DataType<GradType>::type, kernel);
	cv::filter2D(img, grad[1], cv::DataType<GradType>::type, kernel.t());
	#endif
	cv::merge(grad, 2, view.imageGrad);
}
void MeshRefine::ThProjectMesh(uint32_t idxImage, const CameraFaces& cameraFaces)
{
	const Image& imageData = images[idxImage];
	if (!imageData.IsValid())
		return;
	// project mesh to the given camera plane
	View& view = views[idxImage];
	ProjectMesh(vertices, faces, cameraFaces, imageData.camera, view.image.size(),
				view.depthMap, view.faceMap, view.baryMap);
}
void MeshRefine::ThProcessPair(uint32_t idxImageA, uint32_t idxImageB)
{
	// fetch view A data
	const Image& imageDataA = images[idxImageA];
	ASSERT(imageDataA.IsValid());
	const View& viewA = views[idxImageA];
	const BaryMap& baryMapA = viewA.baryMap;
	const FaceMap& faceMapA = viewA.faceMap;
	const DepthMap& depthMapA = viewA.depthMap;
	const Image32F& imageA = viewA.image;
	const Camera& cameraA = imageDataA.camera;
	// fetch view B data
	const Image& imageDataB = images[idxImageB];
	ASSERT(imageDataB.IsValid());
	const View& viewB = views[idxImageB];
	const DepthMap& depthMapB = viewB.depthMap;
	const Image32F& imageB = viewB.image;
	const Camera& cameraB = imageDataB.camera;
	// warp imageB to imageA using the mesh
	Image8U mask;
	Image32F imageAB; imageA.copyTo(imageAB);
	ImageMeshWarp(depthMapA, cameraA, depthMapB, cameraB, imageB, imageAB, mask);
	// init vertex textures
	const TImage<Real>& imageMeanA = viewA.imageMean;
	const TImage<Real>& imageVarA = viewA.imageVar;
	TImage<Real> imageMeanAB, imageVarAB;
	ComputeLocalVariance(imageAB, mask, imageMeanAB, imageVarAB);
	TImage<Real> imageZNCC, imageDZNCC;
	const float score(ComputeLocalZNCC(imageA, imageMeanA, imageVarA, imageAB, imageMeanAB, imageVarAB, mask, imageZNCC, imageDZNCC));
	const Real RegularizationScale((Real)((REAL)(imageDataA.avgDepth*imageDataB.avgDepth)/(cameraA.GetFocalLength()*cameraB.GetFocalLength())));
	ComputePhotometricGradient(faces, faceNormals, depthMapA, faceMapA, baryMapA, cameraA, cameraB, viewB, imageDZNCC, mask, photoGrad, photoGradNorm, RegularizationScale);
	Lock l(cs);
	scorePhoto += (float)RegularizationScale*score;
}
void MeshRefine::ThSmoothVertices1(VIndex idxStart, VIndex idxEnd)
{
	const float score(ComputeSmoothnessGradient1(vertices, vertexVertices, vertexBoundary, smoothGrad1, idxStart, idxEnd));
	Lock l(cs);
	scoreSmooth += score;
}
void MeshRefine::ThSmoothVertices2(VIndex idxStart, VIndex idxEnd)
{
	ComputeSmoothnessGradient2(smoothGrad1, vertexVertices, vertexBoundary, smoothGrad2, idxStart, idxEnd);
}
/*----------------------------------------------------------------*/



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
		// evaluate residuals and gradients
		Point3dArr gradients;
		if (!gradient) {
			gradients.Resize(refine.vertices.GetSize());
			gradient = (double*)gradients.Begin();
		}
		*cost = refine.ScoreMesh(gradient);
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
bool Scene::RefineMesh(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews, float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nMaxFaceArea, unsigned nScales, float fScaleStep, float fRegularityWeight, float fGradientStep)
{
	MeshRefine refine(*this, fRegularityWeight, nResolutionLevel, nMinResolution, nMaxViews, nMaxThreads);

	// run the mesh optimization on multiple scales (coarse to fine)
	for (unsigned nScale=0; nScale<nScales; ++nScale) {
		// init images
		const double scale(powi(fScaleStep, (int)(nScales-nScale-1)));
		const double step(powi(2.0, (int)(nScales-nScale)));
		DEBUG_ULTIMATE("Refine mesh at: %.2f image scale", scale);
		if (!refine.InitImages(scale, 0.12*step+0.2))
			return false;

		// extract array of triangles incident to each vertex
		refine.ListVertexFacesPre();

		// automatic mesh subdivision
		refine.SubdivideMesh(nMaxFaceArea, nScale == 0 ? fDecimateMesh : 1.f, nCloseHoles, nEnsureEdgeSize);

		// extract array of triangle normals
		refine.ListVertexFacesPost();

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefine%u.ply", nScales-nScale-1)));
		#endif

		// minimize
		if (fGradientStep == 0) {
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
			// loop a constant number of iterations and apply the gradient
			int iters(75);
			double gstep(0.4);
			if (fGradientStep > 1) {
				iters = FLOOR2INT(fGradientStep);
				gstep = (fGradientStep-(float)iters)*10;
			}
			iters = MAXF(iters/(int)(nScale+1),8);
			Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> gradients(refine.vertices.GetSize(),3);
			for (int iter=0; iter<iters; ++iter) {
				// evaluate residuals and gradients
				const double cost = refine.ScoreMesh(gradients.data());
				// apply gradients
				double gv(0);
				FOREACH(v, refine.vertices) {
					Vertex& vert = refine.vertices[v];
					vert -= Vertex((gradients.row(v)*gstep).cast<float>());
					gv += gradients.row(v).norm();
				}
				DEBUG_EXTRA("\t%2d. f: %.5f (%.4e)\tg: %.5f (%.4e - %.4e)\ts: %.3f", iter+1, cost, cost/refine.vertices.GetSize(), gradients.norm(), gradients.norm()/refine.vertices.GetSize(), gv/refine.vertices.GetSize(), gstep);
				gstep *= 0.98;
			}
		}

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefined%u.ply", nScales-nScale-1)));
		#endif
	}

	return true;
} // RefineMesh
/*----------------------------------------------------------------*/
