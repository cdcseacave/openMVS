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
#include "SceneRefineCommon.h"
#include "SceneRefineStep.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to ensure edge size and improve vertex valence
// (should enable more stable flow)
#define MESHOPT_ENSUREEDGESIZE 1 // 0 - at all resolution

// uncomment to enable memory pool
// (should reduce the allocation times for frequent used images)
#define MESHOPT_TYPEPOOL

// uncomment to enable CERES optimization module
// (similar performance with the custom minimizer)
#ifdef _USE_CERES
#define MESHOPT_CERES
#endif

#ifdef MESHOPT_TYPEPOOL
#define DEC_BitMatrix(var)		BitMatrix& var = *BitMatrixPool()
#define DEC_Image(type, var)	TImage<type>& var = *ImagePool<type>()
#define DST_BitMatrix(var)		BitMatrixPool(&(var))
#define DST_Image(var)			ImagePool(&(var))
#else
#define DEC_BitMatrix(var)		BitMatrix var;
#define DEC_Image(type, var)	TImage<type> var;
#define DST_BitMatrix(var)
#define DST_Image(var)
#endif

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ScnRefne"));

typedef float Real;
typedef Mesh::Vertex Vertex;
typedef Mesh::VIndex VIndex;
typedef Mesh::Face Face;
typedef Mesh::FIndex FIndex;

class MeshRefine {
public:
	typedef TPoint3<Real> Grad;
	typedef CLISTDEF0IDX(Grad,VIndex) GradArr;

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
	typedef CLISTDEF2(View) ViewsArr;

	// used to render a mesh for optimization
	struct RasterMesh : TRasterMesh<RasterMesh> {
		typedef TRasterMesh<RasterMesh> Base;
		FaceMap& faceMap;
		BaryMap& baryMap;
		FIndex idxFace;
		RasterMesh(const Mesh::VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap, FaceMap& _faceMap, BaryMap& _baryMap)
			: Base(_vertices, _camera, _depthMap), faceMap(_faceMap), baryMap(_baryMap) {}
		void Clear() {
			Base::Clear();
			faceMap.memset((uint8_t)NO_ID);
			baryMap.memset(0);
		}
		// accept any vertex in front of the camera and let RasterizeTriangleBary (CULL=true by
		// default, Types.h/.inl) clip the triangle to the image; the inherited
		// TRasterMeshBase::ProjectVertex additionally requires the projected point to be inside a
		// hardcoded 3px border, which drops the WHOLE face -- including its valid interior pixels
		// -- the moment any single one of its 3 vertices is within that border of (or outside) the
		// image edge. Border rejection now happens per-pixel in Raster() below instead.
		inline bool ProjectVertex(const Point3f& pt, int v, Triangle& t) {
			t.ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt));
			if (t.ptc[v].z <= 0)
				return false;
			t.pti[v] = camera.TransformPointC2I(t.ptc[v]);
			return true;
		}
		void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
			// RasterizeTriangleBary only clips to the image itself; since ProjectVertex above no
			// longer enforces a per-vertex border, enforce the shared Refine::Border margin here,
			// per-pixel (same margin the per-pixel window-statistics code requires elsewhere)
			if (pt.x < Refine::Border || pt.y < Refine::Border ||
				pt.x >= depthMap.cols-Refine::Border || pt.y >= depthMap.rows-Refine::Border)
				return;
			const Point3f pbary(PerspectiveCorrectBarycentricCoordinates(t, bary));
			const Depth z(ComputeDepth(t, pbary));
			ASSERT(z > Depth(0));
			Depth& depth = depthMap(pt);
			if (depth == 0 || depth > z) {
				depth = z;
				faceMap(pt) = idxFace;
				baryMap(pt) = pbary;
			}
		}
	};


public:
	MeshRefine(Scene& _scene, unsigned _nReduceMemory, unsigned _nAlternatePair=true, Real _weightRegularity=1.5f, Real _ratioRigidityElasticity=0.8f, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640, unsigned nMaxViews=8, unsigned nMaxThreads=1);
	~MeshRefine();

	bool IsValid() const { return !pairs.IsEmpty(); }

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
		const Mesh::VertexArr& vertices, const Mesh::FaceArr& faces, const Mesh::FaceIdxArr& cameraFaces,
		const Camera& camera, const Image8U::Size& size,
		DepthMap& depthMap, FaceMap& faceMap, BaryMap& baryMap);
	static void ImageMeshWarp(
		const DepthMap& depthMapA, const Camera& cameraA,
		const DepthMap& depthMapB, const Camera& cameraB,
		const Image32F& imageB, Image32F& imageA, BitMatrix& mask);
	static void ComputeLocalVariance(
		const Image32F& image, const BitMatrix& mask,
		TImage<Real>& imageMean, TImage<Real>& imageVar);
	// the two sums one pair-direction contributes to the reliability-weighted score S:
	// sumRZ over its masked pixels, and the weight sum sumR that normalizes it
	struct PairScore {
		float sumRZ; // Sum r*(1-ZNCC)
		float sumR; // Sum r
	};
	static PairScore ComputeLocalZNCC(
		const Image32F& imageA, const TImage<Real>& imageMeanA, const TImage<Real>& imageVarA,
		const Image32F& imageB, const TImage<Real>& imageMeanB, const TImage<Real>& imageVarB,
		const BitMatrix& mask, TImage<Real>& imageZNCC, TImage<Real>& imageDZNCC);
	static void ComputePhotometricGradient(
		const Mesh::FaceArr& faces, const Mesh::NormalArr& normals,
		const DepthMap& depthMapA, const FaceMap& faceMapA, const BaryMap& baryMapA, const Camera& cameraA,
		const Camera& cameraB, const View& viewB,
		const TImage<Real>& imageDZNCC, const BitMatrix& mask, GradArr& photoGrad, UnsignedArr& photoGradNorm, FloatArr& footprint, Real RegularizationScale,
		TImage<Real>* debugSG = NULL); // WP2 parity diagnostic only: receives the per-pixel photometric scalar (see RefineDebug)
	static float ComputeSmoothnessGradient1(
		const Mesh::VertexArr& vertices, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
		GradArr& smoothGrad1, VIndex idxStart, VIndex idxEnd);
	static void ComputeSmoothnessGradient2(
		const GradArr& smoothGrad1, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
		GradArr& smoothGrad2, VIndex idxStart, VIndex idxEnd);
	template<typename TYPE>
	static TYPE* TypePool(TYPE* = NULL);
	template<typename TYPE>
	static inline TImage<TYPE>* ImagePool(TImage<TYPE>* pImage = NULL) { return TypePool< TImage<TYPE> >(pImage); }
	static inline BitMatrix* BitMatrixPool(BitMatrix* pMask = NULL) { return TypePool<BitMatrix>(pMask); }

	static void* ThreadWorkerTmp(void*);
	void ThreadWorker();
	void WaitThreadWorkers(size_t nJobs);
	void ThSelectNeighbors(uint32_t idxImage, std::unordered_set<uint64_t>& mapPairs, unsigned nMaxViews);
	void ThInitImage(uint32_t idxImage, Real scale, Real sigma);
	void ThProjectMesh(uint32_t idxImage, const Mesh::FaceIdxArr& cameraFaces);
	void ThProcessPair(uint32_t idxImageA, uint32_t idxImageB);
	void ThSmoothVertices1(VIndex idxStart, VIndex idxEnd);
	void ThSmoothVertices2(VIndex idxStart, VIndex idxEnd);

public:
	const Real weightRegularity; // a scalar regularity weight to balance between photo-consistency and regularization terms
	Real ratioRigidityElasticity; // a scalar ratio used to compute the regularity gradient as a combination of rigidity and elasticity
	const unsigned nResolutionLevel; // how many times to scale down the images before mesh optimization
	const unsigned nMinResolution; // how many times to scale down the images before mesh optimization
	const unsigned nReduceMemory; // recompute image mean and variance in order to reduce memory requirements
	unsigned nAlternatePair; // using an image pair alternatively as reference image (0 - both, 1 - alternate, 2 - only left, 3 - only right)
	unsigned iteration; // current refinement iteration
	unsigned nScale; // current refinement scale (0-based, coarsest first; RefineDebug export file naming only)

	Scene& scene; // the mesh vertices and faces

	// gradient related
	float scorePhoto;
	float scoreSmooth;
	GradArr photoGrad;
	FloatArr photoGradNorm;
	FloatArr vertexDepth; // per-pair-direction min-seen avgDepth; feeds only the planar-vertex
	                      // removal pass in Scene::RefineMesh (fThPlanarVertex) -- unrelated to
	                      // footprint below, which is a real per-vertex depth/focal-length ratio
	GradArr smoothGrad1;
	GradArr smoothGrad2;

	// reliability-weighted photo-consistency score (S = sumRZ/sumR): invariant to scene scale,
	// contrast, resolution and pair count, unlike scorePhoto which is weighted by
	// RegularizationScale and summed rather than averaged; sumR/sumRZ accumulate the reliability
	// weight r and r*(1-ZNCC) over every pixel that contributes to a pair-direction's score
	float sumR;
	float sumRZ;
	float S;
	// per-vertex footprint, scene units per pixel: min over pair-directions of the camera-A
	// depth/focal-length ratio at the current scale; 0 for a vertex no pair-direction saw, so
	// footprint[v] > 0 exactly where photoGradNorm[v] (c_v) > 0
	FloatArr footprint;

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

	enum { HalfSize = Refine::HalfSize }; // half window size used to compute ZNCC (shared with CUDA, SceneRefineCommon.h)
};

// call with empty parameter to get an unused image;
// call with an image pointer retrieved earlier to signal that is not needed anymore
template<typename TYPE>
TYPE* MeshRefine::TypePool(TYPE* pObj)
{
	typedef CAutoPtr<TYPE> TypePtr;
	static CriticalSection cs;
	static cList<TypePtr,TYPE*> objects;
	static cList<TYPE*,TYPE*,0> unused;
	Lock l(cs);
	if (pObj == NULL) {
		if (unused.IsEmpty())
			return objects.AddConstruct(new TYPE);
		return unused.RemoveTail();
	} else {
		ASSERT(objects.Find(pObj) != NO_IDX);
		ASSERT(unused.Find(pObj) == NO_IDX);
		unused.Insert(pObj);
		return NULL;
	}
}


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
	Real scale, sigma;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThInitImage(idxImage, scale, sigma);
		return true;
	}
	EVTInitImage(uint32_t _idxImage, Real _scale, Real _sigma) : Event(EVT_JOB), idxImage(_idxImage), scale(_scale), sigma(_sigma) {}
};
class EVTProjectMesh : public Event
{
public:
	uint32_t idxImage;
	const Mesh::FaceIdxArr& cameraFaces;
	bool Run(void* pArgs) {
		((MeshRefine*)pArgs)->ThProjectMesh(idxImage, cameraFaces);
		return true;
	}
	EVTProjectMesh(uint32_t _idxImage, const Mesh::FaceIdxArr& _cameraFaces) : Event(EVT_JOB), idxImage(_idxImage), cameraFaces(_cameraFaces) {}
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

MeshRefine::MeshRefine(Scene& _scene, unsigned _nReduceMemory, unsigned _nAlternatePair, Real _weightRegularity, Real _ratioRigidityElasticity, unsigned _nResolutionLevel, unsigned _nMinResolution, unsigned nMaxViews, unsigned nMaxThreads)
	:
	weightRegularity(_weightRegularity),
	ratioRigidityElasticity(_ratioRigidityElasticity),
	nResolutionLevel(_nResolutionLevel),
	nMinResolution(_nMinResolution),
	nReduceMemory(_nReduceMemory),
	nAlternatePair(_nAlternatePair),
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
	iteration = 0;
	nScale = 0;
	return true;
}

// extract array of triangles incident to each vertex
// and check each vertex if it is at the boundary or not
void MeshRefine::ListVertexFacesPre()
{
	scene.mesh.EmptyExtra();
	scene.mesh.ListIncidentFaces();
}
void MeshRefine::ListVertexFacesPost()
{
	scene.mesh.ListIncidentVertices();
	scene.mesh.ListBoundaryVertices();
}

// extract array of faces viewed by each image
void MeshRefine::ListCameraFaces()
{
	// extract array of faces viewed by each camera
	typedef CLISTDEF2(Mesh::FaceIdxArr) CameraFacesArr;
	CameraFacesArr arrCameraFaces(images.GetSize()); {
		Mesh::Octree octree;
		Mesh::FacesInserter::CreateOctree(octree, scene.mesh);
		FOREACH(ID, images) {
			const Image& imageData = images[ID];
			if (!imageData.IsValid())
				continue;
			const TFrustum<float,5> frustum(Matrix3x4f(imageData.camera.P), (float)imageData.width, (float)imageData.height);
			Mesh::FacesInserter inserter(arrCameraFaces[ID]);
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
	// remeshing to the midpoint of the [0.5x, 4x] mean-edge band the refinement
	// wants is expressed as a negative (relative) target edge length, so it runs
	// as the remesh stage of the same Clean pass instead of a second round trip
	constexpr float fEnsureEdgeLength(-2.25f);
	const auto cleanMesh = [&](float simplifyTarget, float edgeLength=0.f) {
		Mesh::CleanParams params;
		params.simplifyTarget = simplifyTarget;
		params.maxHoleEdges = nCloseHoles;
		params.edgeLength = edgeLength;
		params.remeshIterations = 10;
		scene.mesh.Clean(params);
	};

	// first decimate if necessary
	const bool bNoDecimation(fDecimate >= 1.f);
	const bool bNoSimplification(maxArea == 0);
	if (!bNoDecimation) {
		if (fDecimate > 0.f) {
			// decimate to the desired resolution
			cleanMesh(fDecimate);

			#ifdef MESHOPT_ENSUREEDGESIZE
			// make sure there are no edges too small or too long
			if (nEnsureEdgeSize > 0 && bNoSimplification)
				cleanMesh(1.f, fEnsureEdgeLength);
			#endif

			// re-map vertex and camera faces
			ListVertexFacesPre();
		} else {
			// extract array of faces viewed by each camera
			ListCameraFaces();

			// estimate the faces' area that have big projection areas in both images of a pair
			ListFaceAreas(maxAreas);
			ASSERT(!maxAreas.IsEmpty());

			const float fMaxArea((float)(maxArea > 0 ? maxArea : 64));
			const float fMedianArea(6.f*(float)Mesh::AreaArr(maxAreas).GetMedian());
			if (fMedianArea < fMaxArea) {
				maxAreas.Empty();

				// decimate to the auto detected resolution
				cleanMesh(MAXF(0.1f, fMedianArea/fMaxArea));

				#ifdef MESHOPT_ENSUREEDGESIZE
				// make sure there are no edges too small or too long
				if (nEnsureEdgeSize > 0 && bNoSimplification)
					cleanMesh(1.f, fEnsureEdgeLength);
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
	scene.mesh.Subdivide(maxAreas, maxArea);

	#ifdef MESHOPT_ENSUREEDGESIZE
	// make sure there are no edges too small or too long
	#if MESHOPT_ENSUREEDGESIZE==1
	if ((nEnsureEdgeSize == 1 && !bNoDecimation) || nEnsureEdgeSize > 1)
	#endif
		cleanMesh(1.f, fEnsureEdgeLength);
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
	sumR = 0;
	sumRZ = 0;
	photoGrad.Resize(vertices.GetSize());
	photoGrad.Memset(0);
	photoGradNorm.Resize(vertices.GetSize());
	photoGradNorm.Memset(0);
	footprint.Resize(vertices.GetSize());
	footprint.MemsetValue(FLT_MAX); // sentinel, resolved to 0 for unseen vertices once every pair-direction has run
	if (!vertexDepth.IsEmpty()) {
		ASSERT(vertexDepth.GetSize() == vertices.GetSize());
		vertexDepth.MemsetValue(FLT_MAX);
	}
	ASSERT(events.IsEmpty());
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		switch (nAlternatePair) {
		case 1:
			events.AddEvent(iteration%2 ? new EVTProcessPair(pPair->j,pPair->i) : new EVTProcessPair(pPair->i,pPair->j));
			break;
		case 2:
			events.AddEvent(new EVTProcessPair(pPair->i, pPair->j));
			break;
		case 3:
			events.AddEvent(new EVTProcessPair(pPair->j, pPair->i));
			break;
		default:
			for (int ip=0; ip<2; ++ip)
				events.AddEvent(ip ? new EVTProcessPair(pPair->j,pPair->i) : new EVTProcessPair(pPair->i,pPair->j));
		}
	}
	WaitThreadWorkers(nAlternatePair ? pairs.GetSize() : pairs.GetSize()*2);

	// resolve the footprint sentinel: a vertex no pair-direction saw keeps the documented 0
	// (contract: footprint[v] > 0 exactly where photoGradNorm[v] > 0)
	FOREACH(v, footprint) {
		if (photoGradNorm[v] > 0) {
			ASSERT(footprint[v] > 0 && footprint[v] < FLT_MAX);
		} else {
			footprint[v] = 0;
		}
	}

	// S = sumRZ/sumR, the reliability-weighted mean of (1-ZNCC); degenerate only if no
	// pair-direction contributed a single pixel, which ScoreMesh is not designed to run on
	ASSERT(sumR > 0);
	S = sumRZ/sumR;
	ASSERT(S >= 0 && S <= 2);

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
	if (ratioRigidityElasticity >= 1.f) {
		FOREACH(v, vertices)
			((Point3d*)gradients)[v] = photoGradNorm[v] > 0 ?
				Cast<double>(photoGrad[v]/photoGradNorm[v] + smoothGrad2[v]*weightRegularity) :
				Cast<double>(smoothGrad2[v]*weightRegularity);
	} else {
		// compute smoothing gradient as a combination of level 1 and 2 of the Laplacian operator;
		// (see page 105 of "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004)
		const Real rigidity((Real(1)-ratioRigidityElasticity)*weightRegularity);
		const Real elasticity(ratioRigidityElasticity*weightRegularity);
		FOREACH(v, vertices)
			((Point3d*)gradients)[v] = photoGradNorm[v] > 0 ?
				Cast<double>(photoGrad[v]/photoGradNorm[v] + smoothGrad2[v]*elasticity - smoothGrad1[v]*rigidity) :
				Cast<double>(smoothGrad2[v]*elasticity - smoothGrad1[v]*rigidity);
	}

	// WP2 parity diagnostic: dump the per-vertex terms this iteration combined,
	// right before they are gone (photoGrad/smoothGrad1/smoothGrad2 get
	// overwritten next ScoreMesh() call); no-op unless OMVS_REFINE_DEBUG_DIR is set
	if (!RefineDebug::Dir().empty()) {
		GradArr combined(vertices.GetSize());
		FOREACH(v, vertices)
			combined[v] = Cast<float>(((const Point3d*)gradients)[v]);
		cList<uint8_t,uint8_t,0> boundary(vertices.GetSize());
		FOREACH(v, vertices)
			boundary[v] = vertexBoundary[v] ? 1 : 0;
		RefineDebug::ExportGradients(nScale, iteration, vertices.GetSize(),
			vertices.Begin(), combined.Begin(), photoGrad.Begin(), photoGradNorm.Begin(),
			smoothGrad1.Begin(), smoothGrad2.Begin(), boundary.Begin());
	}

	return (nAlternatePair ? 0.2f : 0.1f)*scorePhoto + 0.01f*scoreSmooth;
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
	// the whole 2x2 neighbourhood read below must be inside the shared Refine::Border margin,
	// tested as a single block (same rule as CUDA's kernelImageMeshWarp); the old per-corner
	// isInsideWithBorder<int,3>(ir) test independently skipped a failing corner instead of
	// rejecting the whole block, so its effective margin was 1px short of the intended 3px.
	// The range test is in FLOAT, before FLOOR2INT: the caller projects a point whose depth in the
	// other camera can be positive but arbitrarily small, so pt can be far outside any int range,
	// and converting first then testing "tl.x+1 >= cols-Border" overflows and wrongly passes.
	// Border and the sizes being integers, this is exactly the floor-based test it replaces, and it
	// also rejects a non-finite pt. The identical hole in the CUDA twin indexed a depth map 3.4 GB
	// out of bounds; here it only read whatever heap followed the depth map, which is why it went
	// unnoticed for years.
	if (!(pt.x >= (float)Refine::Border && pt.y >= (float)Refine::Border &&
		  pt.x < (float)(depthMap.cols-Refine::Border-1) && pt.y < (float)(depthMap.rows-Refine::Border-1)))
		return false;
	const ImageRef tl(FLOOR2INT(pt));
	ASSERT(tl.x >= Refine::Border && tl.y >= Refine::Border &&
		   tl.x+1 < depthMap.cols-Refine::Border && tl.y+1 < depthMap.rows-Refine::Border);
	for (int x=0; x<2; ++x) {
		for (int y=0; y<2; ++y) {
			const ImageRef ir(tl.x+x, tl.y+y);
			const Depth& depth = depthMap(ir);
			// occlusion test only (shared Refine::DepthConstBias, same rule as kernelImageMeshWarp);
			// the symmetric 1 % test that used to sit behind an #ifndef here was never compiled in
			if (depth <= 0 || depth+Refine::DepthConstBias < z)
				continue;
			return true;
		}
	}
	return false;
}

// project mesh to the given camera plane
void MeshRefine::ProjectMesh(
	const Mesh::VertexArr& vertices, const Mesh::FaceArr& faces, const Mesh::FaceIdxArr& cameraFaces,
	const Camera& camera, const Image8U::Size& size,
	DepthMap& depthMap, FaceMap& faceMap, BaryMap& baryMap)
{
	// init view data
	depthMap.create(size);
	faceMap.create(size);
	baryMap.create(size);
	// project all triangles on this image and keep the closest ones
	RasterMesh rasterer(vertices, camera, depthMap, faceMap, baryMap);
	RasterMesh::Triangle triangle;
	RasterMesh::TriangleRasterizer triangleRasterizer(triangle, rasterer);
	rasterer.Clear();
	for (auto idxFace : cameraFaces) {
		const Face& facet = faces[idxFace];
		rasterer.idxFace = idxFace;
		rasterer.Project(facet, triangleRasterizer);
	}
}

// project image from view B to view A through the mesh;
// the projected image is stored in imageA
// (imageAB is assumed to be initialize to the right size)
void MeshRefine::ImageMeshWarp(
	const DepthMap& depthMapA, const Camera& cameraA,
	const DepthMap& depthMapB, const Camera& cameraB,
	const Image32F& imageB, Image32F& imageA, BitMatrix& mask)
{
	ASSERT(!imageA.empty());
	typedef Sampler::Linear<float> Sampler;
	const Sampler sampler;
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
			mask.set(j,i);
		}
	}
}

// compute local variance for each image pixel
void MeshRefine::ComputeLocalVariance(const Image32F& image, const BitMatrix& mask, TImage<Real>& imageMean, TImage<Real>& imageVar)
{
	ASSERT(image.size() == mask.size());
	imageMean.create(image.size());
	imageVar.create(image.size());
	imageMean.memset(0);
	imageVar.memset(0);
	const int RowsEnd(image.rows-HalfSize);
	const int ColsEnd(image.cols-HalfSize);
	const int n(SQUARE(HalfSize*2+1));
	DEC_Image(double, imageSum);
	DEC_Image(double, imageSumSq);
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
	DST_Image(imageSum);
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
	DST_Image(imageSumSq);
}

// compute local ZNCC and its gradient for each image pixel
MeshRefine::PairScore MeshRefine::ComputeLocalZNCC(
	const Image32F& imageA, const TImage<Real>& imageMeanA, const TImage<Real>& imageVarA,
	const Image32F& imageB, const TImage<Real>& imageMeanB, const TImage<Real>& imageVarB,
	const BitMatrix& mask, TImage<Real>& imageZNCC, TImage<Real>& imageDZNCC)
{
	ASSERT(imageA.size() == mask.size() && imageB.size() == mask.size() && !mask.empty());
	float score(0);
	imageZNCC.create(mask.size());
	imageDZNCC.create(mask.size());
	const int RowsEnd(mask.rows-HalfSize);
	const int ColsEnd(mask.cols-HalfSize);
	const int n(SQUARE(HalfSize*2+1));
	imageZNCC.memset(0);
	DEC_Image(double, imageABSum);
	{
		DEC_Image(float, imageAB);
		cv::multiply(imageA, imageB, imageAB);
		cv::integral(imageAB, imageABSum, CV_64F);
		DST_Image(imageAB);
	}
	DEC_Image(Real, imageInvSqrtVAVB);
	imageInvSqrtVAVB.create(mask.size());
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r, c))
				continue;
			const Real cv = (Real)((
				imageABSum(r+HalfSize+1, c+HalfSize+1) -
				imageABSum(r+HalfSize+1, c-HalfSize  ) -
				imageABSum(r-HalfSize,   c+HalfSize+1) +
				imageABSum(r-HalfSize,   c-HalfSize  ) ) * (1.0/(double)n));
			// ComputeLocalVariance floors both variances at 1e-4, so this can never be a divide
			// by zero or a denormal -- assert the producer's contract instead of guarding here
			ASSERT(imageVarA(r,c) > 0 && imageVarB(r,c) > 0);
			const Real invSqrtVAVB(Real(1)/SQRT(imageVarA(r,c)*imageVarB(r,c)));
			imageZNCC(r,c) = (cv - imageMeanA(r,c)*imageMeanB(r,c)) * invSqrtVAVB;
			imageInvSqrtVAVB(r,c) = invSqrtVAVB;
		}
	}
	DST_Image(imageABSum);
	imageDZNCC.memset(0);
	Real sumR(0);
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r,c))
				continue;
			const Real ZNCC(imageZNCC(r,c));
			const Real invSqrtVAVB(imageInvSqrtVAVB(r,c));
			const Real ZNCCinvVB(ZNCC/imageVarB(r,c));
			const Real dZNCC((Real)imageA(r,c)*invSqrtVAVB - (Real)imageB(r,c)*ZNCCinvVB + imageMeanB(r,c)*ZNCCinvVB - imageMeanA(r,c)*invSqrtVAVB);
			const Real ReliabilityFactor(Refine::ZnccReliability(imageVarA(r,c), imageVarB(r,c)));
			imageDZNCC(r,c) = -ReliabilityFactor*dZNCC;
			score += (float)(ReliabilityFactor*(Real(1)-ZNCC));
			sumR += ReliabilityFactor;
		}
	}
	DST_Image(imageInvSqrtVAVB);
	return PairScore{score, (float)sumR};
}

// compute the photometric gradient for all vertices seen by an image pair
void MeshRefine::ComputePhotometricGradient(
	const Mesh::FaceArr& faces, const Mesh::NormalArr& normals,
	const DepthMap& depthMapA, const FaceMap& faceMapA, const BaryMap& baryMapA, const Camera& cameraA,
	const Camera& cameraB, const View& viewB,
	const TImage<Real>& imageDZNCC, const BitMatrix& mask, GradArr& photoGrad, UnsignedArr& photoGradNorm, FloatArr& footprint, Real RegularizationScale,
	TImage<Real>* debugSG)
{
	ASSERT(faces.GetSize() == normals.GetSize() && !faces.IsEmpty());
	ASSERT(debugSG == NULL || debugSG->size() == mask.size());
	ASSERT(depthMapA.size() == mask.size() && faceMapA.size() == mask.size() && baryMapA.size() == mask.size() && imageDZNCC.size() == mask.size() && !mask.empty());
	ASSERT(viewB.image.size() == viewB.imageGrad.size() && !viewB.image.empty());
	const int RowsEnd(mask.rows-HalfSize);
	const int ColsEnd(mask.cols-HalfSize);
	typedef Sampler::Linear<View::Grad::Type> Sampler;
	const Sampler sampler;
	TMatrix<Real,2,3> xJac;
	Point2f xB;
	photoGrad.Memset(0);
	photoGradNorm.Memset(0);
	footprint.MemsetValue(FLT_MAX); // sentinel; ThProcessPair only min-merges the vertices this direction actually saw
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
			// scene units per pixel at this vertex, camera A, current scale (Camera::GetFootprintWorld)
			const Real f(cameraA.GetFootprintWorld(depthA));
			const Point3 X(rayA*REAL(depthA)+cameraA.C);
			// project point in second image and
			// projection Jacobian matrix in the second image of the 3D point on the surface
			MAYBEUNUSED const float depthB(ProjectVertex(cameraB.P.val, X.ptr(), xB.ptr(), xJac.val));
			ASSERT(depthB > 0);
			// compute gradient in image B
			const TMatrix<Real,1,2> gB(viewB.imageGrad.sample<Sampler,View::Grad>(sampler, xB));
			// compute gradient scale
			const Real dZNCC(imageDZNCC(r,c));
			const Real sg((gB*(xJac*(const TMatrix<Real,3,1>&)dA))(0)*dZNCC*RegularizationScale/Nd);
			if (debugSG)
				(*debugSG)(r,c) = sg;
			// add gradient to the three vertices
			const Face& face(faces[idxFace]);
			const Point3f& b(baryMapA(r,c));
			for (int v=0; v<3; ++v) {
				const Grad g(N*(sg*(Real)b[v]));
				const VIndex idxVert(face[v]);
				photoGrad[idxVert] += g;
				++photoGradNorm[idxVert];
				if (f < footprint[idxVert])
					footprint[idxVert] = f;
			}
		}
	}
}

// computes the discrete analog of the Laplacian using
// the umbrella-operator on the first triangle ring at each point
float MeshRefine::ComputeSmoothnessGradient1(
	const Mesh::VertexArr& vertices, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
	GradArr& smoothGrad1, VIndex idxStart, VIndex idxEnd)
{
	ASSERT(!vertices.IsEmpty() && vertices.GetSize() == vertexVertices.GetSize() && vertices.GetSize() == smoothGrad1.GetSize());
	float score(0);
	for (VIndex idxV=idxStart; idxV<idxEnd; ++idxV) {
		Grad& grad = smoothGrad1[idxV];
		grad = Grad::ZERO;
		#if 1
		if (vertexBoundary[idxV])
			continue;
		#endif
		const Mesh::VertexIdxArr& verts = vertexVertices[idxV];
		if (verts.IsEmpty())
			continue;
		FOREACH(v, verts)
			grad += Cast<Real>(vertices[verts[v]]);
		grad = grad/(Real)verts.GetSize() - Cast<Real>(vertices[idxV]);
		const float regularityScore((float)norm(grad));
		ASSERT(ISFINITE(regularityScore));
		score += regularityScore;
	}
	return score;
}
// same as above, but used to compute level 2;
// normalized as in "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004
void MeshRefine::ComputeSmoothnessGradient2(
	const GradArr& smoothGrad1, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
	GradArr& smoothGrad2, VIndex idxStart, VIndex idxEnd)
{
	ASSERT(!smoothGrad1.IsEmpty() && smoothGrad1.GetSize() == vertexVertices.GetSize() && smoothGrad1.GetSize() == smoothGrad2.GetSize());
	for (VIndex idxV=idxStart; idxV<idxEnd; ++idxV) {
		Grad& grad = smoothGrad2[idxV];
		grad = Grad::ZERO;
		#if 1
		if (vertexBoundary[idxV])
			continue;
		#endif
		const Mesh::VertexIdxArr& verts = vertexVertices[idxV];
		if (verts.IsEmpty())
			continue;
		Real w(0);
		FOREACH(v, verts) {
			const VIndex idxVert(verts[v]);
			grad += smoothGrad1[idxVert];
			const VIndex numVert(vertexVertices[idxVert].GetSize());
			if (numVert > 0)
				w += Real(1)/(Real)numVert;
		}
		const Real numVert((Real)verts.GetSize());
		const Real nrm(Real(1)/(Real(1)+w/numVert));
		grad = grad*(nrm/numVert) - smoothGrad1[idxV]*nrm;
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
	const float fMinArea(0.1f);
	const float fMinScale(0.2f), fMaxScale(3.2f);
	const float fMinAngle(D2R(2.5f)), fMaxAngle(D2R(45.f));
	Image& imageData = images[idxImage];
	if (!imageData.IsValid())
		return;
	if (imageData.neighbors.IsEmpty()) {
		IndexArr points;
		scene.SelectNeighborViews(idxImage, points);
	}
	ViewScoreArr neighbors(imageData.neighbors);
	Scene::FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews);
	Lock l(cs);
	for (const ViewScore& neighbor: neighbors) {
		ASSERT(images[neighbor.ID].IsValid());
		mapPairs.insert(MakePairIdx((uint32_t)idxImage, neighbor.ID));
	}
}
void MeshRefine::ThInitImage(uint32_t idxImage, Real scale, Real sigma)
{
	Image& imageData = images[idxImage];
	if (!imageData.IsValid())
		return;
	// load and init image
	View& view = views[idxImage];
	Image32F& img = view.image;
	if (!PrepareRefineImage(imageData, scene.platforms, nResolutionLevel, nMinResolution, scale, sigma, img))
		ABORT("can not load image");
	if (!nReduceMemory) {
		// compute image mean and variance
		ComputeLocalVariance(img, BitMatrix(img.size(), 0xFF), view.imageMean, view.imageVar);
	}
	// compute image gradient (the same estimator on both backends)
	Image32F grad[2];
	ComputeRefineImageGradient(img, grad[0], grad[1]);
	cv::merge(grad, 2, view.imageGrad);
}
void MeshRefine::ThProjectMesh(uint32_t idxImage, const Mesh::FaceIdxArr& cameraFaces)
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
	DEC_BitMatrix(mask);
	DEC_Image(float, imageAB);
	imageA.copyTo(imageAB);
	ImageMeshWarp(depthMapA, cameraA, depthMapB, cameraB, imageB, imageAB, mask);
	// compute ZNCC and its gradient
	const TImage<Real> *imageMeanA, *imageVarA;
	if (nReduceMemory) {
		DEC_Image(Real, _imageMeanA);
		DEC_Image(Real, _imageVarA);
		ComputeLocalVariance(viewA.image, mask, _imageMeanA, _imageVarA);
		imageMeanA = &_imageMeanA;
		imageVarA = &_imageVarA;
	} else {
		imageMeanA = &viewA.imageMean;
		imageVarA = &viewA.imageVar;
	}
	DEC_Image(Real, imageMeanAB);
	DEC_Image(Real, imageVarAB);
	ComputeLocalVariance(imageAB, mask, imageMeanAB, imageVarAB);
	DEC_Image(Real, imageZNCC);
	DEC_Image(Real, imageDZNCC);
	const PairScore pairScore(ComputeLocalZNCC(imageA, *imageMeanA, *imageVarA, imageAB, imageMeanAB, imageVarAB, mask, imageZNCC, imageDZNCC));

	// WP2 parity diagnostic: dump this pair's maps before the pooled buffers
	// above get recycled; no-op unless OMVS_REFINE_DEBUG_DIR/_PAIR are both set
	// and match this exact (A,B) direction
	uint32_t dbgImageA, dbgImageB;
	const bool dbgPair(!RefineDebug::Dir().empty() && RefineDebug::Pair(dbgImageA, dbgImageB) &&
		dbgImageA == idxImageA && dbgImageB == idxImageB);
	if (dbgPair) {
		DEC_Image(Real, conf);
		conf.create(mask.size());
		conf.memset(0);
		for (int r=0; r<mask.rows; ++r)
			for (int c=0; c<mask.cols; ++c)
				if (mask(r,c))
					conf(r,c) = Refine::ZnccReliability((*imageVarA)(r,c), imageVarAB(r,c));
		Image8U maskU8(mask.rows, mask.cols);
		for (int r=0; r<mask.rows; ++r)
			for (int c=0; c<mask.cols; ++c)
				maskU8(r,c) = mask(r,c) ? 1 : 0;
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "imageA", imageA.getData(), imageA.width(), imageA.height());
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "imageAB", imageAB.getData(), imageAB.width(), imageAB.height());
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "zncc", imageZNCC.getData(), imageZNCC.width(), imageZNCC.height());
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "dzncc", imageDZNCC.getData(), imageDZNCC.width(), imageDZNCC.height());
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "conf", conf.getData(), conf.width(), conf.height());
		RefineDebug::ExportPairMask(nScale, iteration, idxImageA, idxImageB, maskU8.getData(), maskU8.width(), maskU8.height());
		DST_Image(conf);
	}

	#ifdef MESHOPT_TYPEPOOL
	DST_Image(imageZNCC);
	DST_Image(imageVarAB);
	DST_Image(imageMeanAB);
	if (nReduceMemory) {
		DST_Image(*((TImage<Real>*)imageMeanA));
		DST_Image(*((TImage<Real>*)imageVarA));
	}
	DST_Image(imageAB);
	#endif
	// compute field gradient
	GradArr _photoGrad(photoGrad.GetSize());
	UnsignedArr _photoGradNorm(photoGrad.GetSize());
	FloatArr _footprint(photoGrad.GetSize());
	const Real RegularizationScale((Real)((REAL)(imageDataA.avgDepth*imageDataB.avgDepth)/(cameraA.GetFocalLength()*cameraB.GetFocalLength())));
	TImage<Real> dbgSG;
	if (dbgPair) {
		dbgSG.create(mask.size());
		dbgSG.memset(0);
	}
	ComputePhotometricGradient(faces, faceNormals, depthMapA, faceMapA, baryMapA, cameraA, cameraB, viewB, imageDZNCC, mask, _photoGrad, _photoGradNorm, _footprint, RegularizationScale, dbgPair ? &dbgSG : NULL);
	if (dbgPair) {
		// WP2 parity diagnostic, continued: the per-pixel photometric scalar each pixel
		// hands to its face's 3 vertices, and the raw face map of A (-1 where nothing was
		// rasterised; NOT masked, so rasterisation and warp differences can be told apart)
		Image32F faceF(mask.size());
		for (int r=0; r<mask.rows; ++r)
			for (int c=0; c<mask.cols; ++c)
				faceF(r,c) = faceMapA(r,c) == NO_ID ? -1.f : (float)faceMapA(r,c);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "sg", dbgSG.getData(), dbgSG.width(), dbgSG.height());
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "face", faceF.getData(), faceF.width(), faceF.height());
	}
	DST_Image(imageDZNCC);
	DST_BitMatrix(mask);
	Lock l(cs);
	if (vertexDepth.IsEmpty()) {
		FOREACH(i, photoGrad) {
			if (_photoGradNorm[i] > 0) {
				photoGrad[i] += _photoGrad[i];
				photoGradNorm[i] += 1.f;
				if (_footprint[i] < footprint[i])
					footprint[i] = _footprint[i];
			}
		}
	} else {
		const float depth(MINF(imageDataA.avgDepth, imageDataB.avgDepth));
		FOREACH(i, photoGrad) {
			if (_photoGradNorm[i] > 0) {
				photoGrad[i] += _photoGrad[i];
				photoGradNorm[i] += 1.f;
				if (_footprint[i] < footprint[i])
					footprint[i] = _footprint[i];
				if (vertexDepth[i] > depth)
					vertexDepth[i] = depth;
			}
		}
	}
	// raw (unscaled by RegularizationScale) reliability sums for S = sumRZ/sumR (ScoreMesh)
	sumR += pairScore.sumR;
	sumRZ += pairScore.sumRZ;
	scorePhoto += (float)RegularizationScale*pairScore.sumRZ;
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

#ifdef MESHOPT_CERES

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
class MeshProblem : public FirstOrderFunction, public IterationCallback
{
public:
	MeshProblem(MeshRefine& _refine) : refine(_refine), params(refine.vertices.GetSize()*3) {
		// init params
		FOREACH(i, refine.vertices)
			*((Point3d*)params.Begin()+i) = refine.vertices[i];
	}
	virtual ~MeshProblem() {}

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

	CallbackReturnType operator()(const IterationSummary& summary) {
		refine.iteration = summary.iteration;
		return ceres::SOLVER_CONTINUE;
	}

	int NumParameters() const { return (int)params.GetSize(); }
	const double* GetParameters() const { return params.Begin(); }
	double* GetParameters() { return params.Begin(); }

protected:
	MeshRefine& refine;
	DoubleArr params;
};
} // namespace ceres

#endif // MESHOPT_CERES


// optimize mesh using photo-consistency
// fThPlanarVertex - threshold used to remove vertices on planar patches (percentage of the minimum depth, 0 - disable)
bool Scene::RefineMesh(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews,
					   float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nMaxFaceArea,
					   unsigned nScales, float fScaleStep,
					   unsigned nAlternatePair, float fRegularityWeight, float fRatioRigidityElasticity, float fGradientStep, float fThPlanarVertex, unsigned nReduceMemory)
{
	// "--gradient-step N.s" means N iterations per scale and s*10 pixels of initial step (0 selects
	// the Ceres arm where it is built in). The stepper measures its step in pixels and never lets it
	// exceed MeshRefineStep::StepMax, so the fractional part has a real valid range; a value outside
	// it is a user error and is reported here, at the entry point every caller goes through, rather
	// than reaching the optimizer as a step nothing would honour
	if (fGradientStep != 0) {
		const float stepInit((fGradientStep-(float)FLOOR2INT(fGradientStep))*10.f);
		if (stepInit <= 0 || stepInit > MeshRefineStep::StepMax) {
			VERBOSE("error: --gradient-step %g asks for an initial step of %g px, outside (0, %g]:"
				" pass N.s with a fractional part in (0, %g], for example 45.05 = 45 iterations at 0.5 px",
				fGradientStep, stepInit, MeshRefineStep::StepMax, MeshRefineStep::StepMax*0.1f);
			return false;
		}
	}

	bool bGeneratedPointcloud(false);
	if (pointcloud.IsEmpty() && !ImagesHaveNeighbors()) {
		SampleMeshWithVisibility();
		bGeneratedPointcloud = true;
	}

	MeshRefine refine(*this, nReduceMemory, nAlternatePair, fRegularityWeight, fRatioRigidityElasticity, nResolutionLevel, nMinResolution, nMaxViews, nMaxThreads);
	if (bGeneratedPointcloud)
		pointcloud.Release();
	if (!refine.IsValid())
		return false;

	// run the mesh optimization on multiple scales (coarse to fine)
	for (unsigned nScale=0; nScale<nScales; ++nScale) {
		// init images
		const Real scale(POWI(fScaleStep, nScales-nScale-1));
		const Real step(POWI(2.f, nScales-nScale));
		DEBUG_ULTIMATE("Refine mesh at: %.2f image scale", scale);
		if (!refine.InitImages(scale, Real(0.12)*step+Real(0.2)))
			return false;
		refine.nScale = nScale;

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
		#ifdef MESHOPT_CERES
		if (fGradientStep == 0) {
			// DefineProblem
			refine.ratioRigidityElasticity = 1.f;
			ceres::MeshProblem* problemData(new ceres::MeshProblem(refine));
			#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 3)
			ceres::GradientProblem problem{std::unique_ptr<ceres::FirstOrderFunction>(problemData)};
			#else
			ceres::GradientProblem problem(problemData);
			#endif
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
			options.callbacks.push_back(problemData);
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
		} else
		#endif // MESHOPT_CERES
		{
			// pixel-unit stepper (SceneRefineStep.h): fGradientStep is N.s, N the same per-scale
			// iteration budget as before, s*10 the initial step in pixels
			const int N(FLOOR2INT(fGradientStep));
			const float eta0((fGradientStep-(float)N)*10.f);
			const int cap(MAXF(N/(int)(nScale+1),8));
			const bool bAlternating(nAlternatePair == 1);
			const bool bPlanarHook(fThPlanarVertex > 0);

			MeshRefineStep stepper;
			stepper.Reset(refine.vertices.GetSize(), eta0, OPTREFINE::nOptimizer == 0);

			Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> gradients(refine.vertices.GetSize(),3);
			// mirrors of the stepper's own accept-only S references, kept only to print the
			// relative-change column below (the stepper exposes no getter for them)
			float logScoreRef(FLT_MAX), logScoreRefAlt(FLT_MAX);
			GET_LOGCONSOLE().Pause();

			// one energy evaluation, stepper decision and log line; phaseCap/allowPlanarHook let
			// the same body serve both phases below
			const auto RunEvaluation = [&](Real rho, bool allowPlanarHook, int idx, int phaseCap) -> MeshRefineStep::Action {
				refine.iteration = stepper.GetNumEvaluated();
				refine.nAlternatePair = nAlternatePair;
				refine.ratioRigidityElasticity = rho;
				const unsigned numAcceptedSoFar(stepper.GetNumAccepted());
				const bool bAdaptMesh(allowPlanarHook && numAcceptedSoFar >= 4 && (numAcceptedSoFar-4)%3 == 0 && phaseCap-idx > 5);
				if (bAdaptMesh)
					refine.vertexDepth.Resize(refine.vertices.GetSize());
				refine.ScoreMesh(gradients.data());
				VIndex numVertsRemoved(0);
				if (bAdaptMesh) {
					// remove planar vertices (small gradient and almost on the center of their surrounding patch)
					ASSERT(refine.vertexDepth.GetSize() == refine.vertices.GetSize());
					Mesh::VertexIdxArr vertexRemove;
					FOREACH(v, refine.vertices) {
						const Point3d grad(gradients.row(v));
						const double gn(norm(grad));
						const float depth(refine.vertexDepth[v]);
						if (depth < FLT_MAX) {
							const float th(depth*fThPlanarVertex);
							if (!refine.vertexBoundary[v] && (float)gn < th && norm(refine.smoothGrad1[v]) < th)
								vertexRemove.Insert(v);
						}
					}
					if (!vertexRemove.IsEmpty()) {
						numVertsRemoved = vertexRemove.GetSize();
						mesh.RemoveVerticesAndFill(vertexRemove);
						refine.ListVertexFacesPost();
						// the surface changed under the stepper: its undo buffer and its S no
						// longer describe the same vertices, so the next evaluation cannot be
						// rejected against the S just measured
						stepper.TopologyChanged(refine.vertices.GetSize());
					}
					refine.vertexDepth.Empty();
				}

				MeshRefineStep::Terms terms;
				terms.photoGrad = refine.photoGrad.Begin();
				terms.photoCount = refine.photoGradNorm.Begin();
				terms.footprint = refine.footprint.Begin();
				terms.lap = refine.smoothGrad1.Begin();
				terms.bilap = refine.smoothGrad2.Begin();
				terms.S = refine.S;
				terms.rigidity = rho;
				terms.regularityWeight = refine.weightRegularity;
				terms.numVertices = refine.vertices.GetSize();
				terms.bounded = false; // the sign-vote/tanh photo terms are not implemented yet
				terms.alternating = bAlternating;

				const unsigned evalIdx(stepper.GetNumEvaluated()); // 0-based, before Evaluate() consumes it
				float& logReference = (bAlternating && (evalIdx&1)) ? logScoreRefAlt : logScoreRef;
				const float relChange(logReference < FLT_MAX ? (terms.S-logReference)/logReference : 0.f);
				const unsigned prevAccepted(stepper.GetNumAccepted());

				MeshRefineStep::Stats stats;
				const MeshRefineStep::Action action(stepper.Evaluate(terms, refine.vertices, stats));
				const bool accepted(stepper.GetNumAccepted() > prevAccepted);
				if (accepted)
					logReference = terms.S;

				DEBUG_EXTRA("\t%2d. S: %.5f (%+.2e)\tstep: %.3fpx\tmed: %.3fpx\tv: %5u\t%s",
					(int)stepper.GetNumEvaluated(), stats.S, relChange, stats.step, stats.medianPx, numVertsRemoved, accepted ? "acc" : "rej");
				return action;
			};

			// Phase A: caller's rigidity/elasticity ratio, with the optional planar-vertex hook
			{
				Util::Progress progress(_T("Processed iterations"), cap);
				for (int idx=0; idx<cap; ++idx) {
					const MeshRefineStep::Action action(RunEvaluation(fRatioRigidityElasticity, bPlanarHook, idx, cap));
					progress.display(idx+1);
					if (action == MeshRefineStep::STOP)
						break;
				}
				progress.close();
			}

			// Phase B: pure elasticity, planar hook off. The legacy 70/30 iteration split is
			// preserved against phase A's ACCEPTED count rather than its raw iteration budget,
			// since a rejected evaluation consumes budget without buying any convergence; step and
			// the S references carry over unchanged, only the stall counter is given a fresh start
			const unsigned nA(stepper.GetNumAccepted());
			const int capB(MAXF(3, 3*(int)nA/7));
			stepper.ResetStall();
			{
				Util::Progress progress(_T("Processed iterations"), capB);
				for (int idx=0; idx<capB; ++idx) {
					const MeshRefineStep::Action action(RunEvaluation(Real(1), false, idx, capB));
					progress.display(idx+1);
					if (action == MeshRefineStep::STOP)
						break;
				}
				progress.close();
			}

			GET_LOGCONSOLE().Play();
		}

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefined%u.ply", nScales-nScale-1)));
		#endif
	}

	return true;
} // RefineMesh
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
