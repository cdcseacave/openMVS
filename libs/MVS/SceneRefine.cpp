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
#include <unordered_map>

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
		FaceMap faceMap; // remember for each pixel what face projects there
		DepthMap depthMap; // depth-map
		BaryMap baryMap; // barycentric coordinates
		BitMatrix keepMask; // per-pixel keep-mask (bit set = keep); empty == no mask == keep everything
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
	MeshRefine(Scene& _scene, unsigned _nAlternatePair=true, Real _weightRegularity=1.5f, Real _ratioRigidityElasticity=0.8f, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640, unsigned nMaxViews=8, unsigned nMaxThreads=1);
	~MeshRefine();

	bool IsValid() const { return !pairs.IsEmpty(); }

	bool InitImages(Real scale, Real sigma=0);

	void ListVertexFacesPre();
	void ListVertexFacesPost();
	void ListCameraFaces();

	void ListFaceAreas(Mesh::AreaArr& maxAreas);
	void SubdivideMesh(uint32_t maxArea, float fDecimate=1.f, unsigned nCloseHoles=15, unsigned nEnsureEdgeSize=1);

	// score the mesh and fill in every per-vertex term; `gradients` receives the combined
	// photometric+smoothness gradient as one Point3d per vertex and may be NULL, in which case
	// that combination is skipped -- the stepper consumes the terms separately and only the Ceres
	// arm, the planar-vertex hook and the debug export need them summed
	double ScoreMesh(double* gradients);

	// given a vertex position and a projection camera, compute the projected position and its derivative
	template <typename TP, typename TX, typename T, typename TJ>
	static T ProjectVertex(const TP* P, const TX* X, T* x, TJ* jacobian=NULL);

	// keepMask is the B-side per-pixel keep-mask (bit set = keep, empty = keep everything), tested
	// at the same rounded nearest tap the occlusion check reads
	static bool IsDepthSimilar(const DepthMap& depthMap, const Point2f& pt, Depth z, const BitMatrix& keepMask);
	static void ProjectMesh(
		const Mesh::VertexArr& vertices, const Mesh::FaceArr& faces, const Mesh::FaceIdxArr& cameraFaces,
		const Camera& camera, const Image8U::Size& size,
		DepthMap& depthMap, FaceMap& faceMap, BaryMap& baryMap);
	// keepA/keepB are the per-pixel keep-masks of image A/B (bit set = keep, empty = keep
	// everything): a masked-out pixel of A never seeds a warp sample, and a projection landing on a
	// masked-out pixel of B is rejected like an occluded one (see IsDepthSimilar)
	static void ImageMeshWarp(
		const DepthMap& depthMapA, const Camera& cameraA,
		const DepthMap& depthMapB, const Camera& cameraB,
		const Image32F& imageB, Image32F& imageA, BitMatrix& mask,
		const BitMatrix& keepA, const BitMatrix& keepB);
	// the two sums one pair-direction contributes to the reliability-weighted score S:
	// sumRZ over its masked pixels, and the weight sum sumR that normalizes it
	struct PairScore {
		float sumRZ; // Sum r*(1-ZNCC)
		float sumR; // Sum r
		// the same Sum r*(1-ZNCC) accumulated in double, filled only in the exact-derivative mode:
		// the finite-difference gate resolves relative energy changes of ~1e-5, well under what a
		// float running sum over ~1e5 pixel terms can carry
		double energyRZ;
	};
	// masked local window statistics of A and of B-warped-into-A, and from them the per-pixel
	// photometric gradient scale; pixels whose window holds too few valid samples or that fail a
	// rejection gate are REMOVED FROM THE MASK, which is therefore in/out and stays the single
	// validity source for everything downstream (except under bExactDerivative below)
	// bExactDerivative replaces the pointwise imageDZNCC (the derivative of the window centred at
	// the pixel alone, times WindowArea/n) with the exact derivative of the whole window sum
	// Sum_c r_c (1-ZNCC_c) with respect to one warped pixel value: the energy mode's gradient must
	// be the gradient of the energy the same call returns, and the pointwise form is not. It also
	// leaves the rejected pixels IN the mask, since the score still depends on their values
	// through their neighbours' windows (see the rejection branch)
	static PairScore ComputeWindowStats(
		const Image32F& imageA, const Image32F& imageB, BitMatrix& mask,
		TImage<Real>& imageDZNCC, TImage<Real>* imageZNCC = NULL, TImage<Real>* imageConf = NULL,
		bool bExactDerivative = false);
	// the per-pixel maps one pair-direction hands to the photometric scatter
	struct PairMaps {
		const TImage<Real>& dzncc; // reliability-weighted ZNCC derivative
		const BitMatrix& mask;
	};
	// the per-vertex accumulators one pair-direction fills before ThProcessPair merges them into
	// the shared ones
	struct PairGrads {
		GradArr& photoGrad;
		UnsignedArr& photoGradPixels; // pixels of THIS direction that touched the vertex (0 = unseen)
		FloatArr& footprint;
	};
	// derivative of the bilinear reconstruction of `image` at `pt`, with the same taps, weights and
	// out-of-range convention (a tap outside the image contributes nothing) as the Linear sampler
	// the warp itself reads image B with -- so it is the derivative of the value the energy
	// actually scored, not of a smoothed estimate of it
	static void BilinearGradient(const Image32F& image, const Point2f& pt, Real& gx, Real& gy);
	// bExactDerivative: take the B-side image derivative from BilinearGradient above instead of
	// the precomputed stencil map viewB.imageGrad -- true in the energy mode (bEnergyMode) and
	// whenever OPTREFINE::nImageGradient == 3 asks for it on the stepper path too. The stencil is
	// a smoothed estimate: on texture approaching the sampling limit it under-states the slope of
	// the interpolant the warp used, which belongs in neither this energy's gradient nor mode 3's
	static void ComputePhotometricGradient(
		const Mesh::FaceArr& faces, const Mesh::NormalArr& normals,
		const DepthMap& depthMapA, const FaceMap& faceMapA, const BaryMap& baryMapA, const Camera& cameraA,
		const Camera& cameraB, const View& viewB,
		const PairMaps& maps, const PairGrads& grads, Real RegularizationScale, bool bExactDerivative,
		TImage<Real>* debugSG = NULL); // CPU/CUDA parity diagnostic only: receives the per-pixel photometric scalar (see RefineDebug)
	static void ComputeSmoothnessGradient1(
		const Mesh::VertexArr& vertices, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
		GradArr& smoothGrad1, VIndex idxStart, VIndex idxEnd);
	static void ComputeSmoothnessGradient2(
		const GradArr& smoothGrad1, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
		GradArr& smoothGrad2, VIndex idxStart, VIndex idxEnd);
	// exact gradient of E_smooth = 1/2 Sum_{v interior} ||L(v)||^2 with L the umbrella operator
	// ComputeSmoothnessGradient1 evaluates: (L^T y)_v = Sum_{u in N(v), u interior} y_u/|N(u)| -
	// [v interior] y_v, with y = L v (smoothGrad1). The division is by the NEIGHBOUR's valence,
	// which is what makes this the transpose of L rather than another averaging operator -- it is
	// not the Hernandez normalized level-2 operator ComputeSmoothnessGradient2 applies
	static void ComputeSmoothnessGradientLtL(
		const GradArr& smoothGrad1, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
		GradArr& smoothGradLtL, VIndex idxStart, VIndex idxEnd);
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
	unsigned nAlternatePair; // using an image pair alternatively as reference image (0 - both, 1 - alternate, 2 - only left, 3 - only right)
	unsigned iteration; // current refinement iteration
	unsigned nScale; // current refinement scale (0-based, coarsest first; RefineDebug export file naming only)

	// exact-energy mode, selected by the Ceres arm and by its finite-difference gate: ScoreMesh
	// returns the exact energy E = E_photo + weightRegularity*E_smooth whose exact gradient it
	// writes, instead of the stepper's descent-only score (whose constants and smoothing
	// functional do not match the gradient it fills, which is all a step-and-test optimizer needs
	// but not what a line search with a curvature condition requires)
	bool bEnergyMode;
	// energy mode only: false skips the photometric pass entirely, leaving E = E_smooth alone --
	// the term-by-term split the finite-difference gate needs
	bool bEnergyPhoto;

	Scene& scene; // the mesh vertices and faces

	// energy mode only: the two terms of E, accumulated in double: the energy is what a line
	// search compares across a step of a fraction of a pixel, and a float running sum over ~1e5
	// pixel terms cannot resolve a change that small
	double energyPhoto; // Sum_pairs RegScale_p Sum_pixels r (1-ZNCC)
	double energySmooth; // 1/2 Sum_{v interior} ||L(v)||^2
	// energy mode only: multiplies the photometric energy and its gradient (1 = the raw sum); the
	// Ceres arm derives it from its calibration evaluation at every scale start
	double energyPhotoScale;
	// energy mode only: the reliability sum of every pair-direction of the current evaluation,
	// keyed by PairKey(A,B), and the same at the scale's calibration evaluation (empty = every
	// weight 1): a direction's photometric term is weighted by sumR0/sumR
	std::unordered_map<uint64_t,float> pairSumR;
	std::unordered_map<uint64_t,float> pairSumR0;
	static inline uint64_t PairKey(uint32_t idxImageA, uint32_t idxImageB) { return ((uint64_t)idxImageA<<32)|idxImageB; }
	GradArr photoGrad;
	FloatArr photoGradNorm;
	GradArr smoothGrad1;
	GradArr smoothGrad2;
	GradArr smoothGradLtL; // energy mode only: L^T L v, the exact gradient of 1/2 Sum ||L v||^2

	// reliability-weighted photo-consistency score (S = sumRZ/sumR): invariant to scene scale,
	// contrast, resolution and pair count; sumR/sumRZ accumulate the reliability weight r and
	// r*(1-ZNCC) over every pixel that contributes to a pair-direction's score
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

MeshRefine::MeshRefine(Scene& _scene, unsigned _nAlternatePair, Real _weightRegularity, Real _ratioRigidityElasticity, unsigned _nResolutionLevel, unsigned _nMinResolution, unsigned nMaxViews, unsigned nMaxThreads)
	:
	weightRegularity(_weightRegularity),
	ratioRigidityElasticity(_ratioRigidityElasticity),
	nResolutionLevel(_nResolutionLevel),
	nMinResolution(_nMinResolution),
	nAlternatePair(_nAlternatePair),
	bEnergyMode(false),
	bEnergyPhoto(true),
	scene(_scene),
	energyPhotoScale(1),
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
	// the thread list is static: leaving the joined (dead) threads in it makes the next
	// MeshRefine of the same process -- a second refinement from the Viewer, the Python
	// bindings or a test -- trip the ASSERT(threads.IsEmpty()) contract of the constructor
	// and then start its workers on top of the stale entries
	threads.Release();
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
	energyPhoto = 0;
	pairSumR.clear();
	sumR = 0;
	sumRZ = 0;
	photoGrad.Resize(vertices.GetSize());
	photoGrad.Memset(0);
	photoGradNorm.Resize(vertices.GetSize());
	photoGradNorm.Memset(0);
	footprint.Resize(vertices.GetSize());
	footprint.MemsetValue(FLT_MAX); // sentinel, resolved to 0 for unseen vertices once every pair-direction has run
	ASSERT(events.IsEmpty());
	// bEnergyPhoto is false only in the energy mode's smoothness-alone evaluation: no
	// pair-direction runs, so photoGrad stays zero and every footprint falls to the
	// documented 0 below
	if (bEnergyPhoto) {
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
	}

	// resolve the footprint sentinel: a vertex no pair-direction saw keeps the documented 0
	// (contract: footprint[v] > 0 exactly where photoGradNorm[v] > 0)
	FOREACH(v, footprint) {
		if (photoGradNorm[v] > 0) {
			ASSERT(footprint[v] > 0 && footprint[v] < FLT_MAX);
		} else {
			footprint[v] = 0;
		}
	}

	// S = sumRZ/sumR, the reliability-weighted mean of (1-ZNCC). sumR == 0 means no
	// pair-direction contributed a single masked pixel -- broken outside-world input (no
	// pair overlap, bad poses), not an internal invariant, so it gets the runtime-validation
	// treatment instead of an ASSERT alone (which would compile out in Release and feed the
	// stepper NaN): S is left negative and the caller fails the refinement loudly
	if (sumR > 0) {
		S = sumRZ/sumR;
		ASSERT(S >= 0 && S <= 2);
	} else
		S = -1.f;

	// loop through all vertices and compute the smoothing terms
	energySmooth = 0;
	const VIndex idxStep((vertices.GetSize()+(VIndex)threads.GetSize()-1)/(VIndex)threads.GetSize());
	smoothGrad1.Resize(vertices.GetSize());
	// the chunking rounds idxStep up, so it hands out FEWER chunks than there are threads for most
	// vertex counts (100 vertices over 24 threads is 20 chunks of 5): the number of jobs to wait
	// for is the number actually queued, never the thread count
	{
	ASSERT(events.IsEmpty());
	VIndex idx(0);
	size_t nJobs(0);
	while (idx<vertices.GetSize()) {
		const VIndex idxNext(MINF(idx+idxStep, vertices.GetSize()));
		events.AddEvent(new EVTSmoothVertices1(idx, idxNext));
		idx = idxNext;
		++nJobs;
	}
	WaitThreadWorkers(nJobs);
	}
	// loop through all vertices and compute the smoothing gradient
	smoothGrad2.Resize(vertices.GetSize());
	if (bEnergyMode)
		smoothGradLtL.Resize(vertices.GetSize());
	{
	ASSERT(events.IsEmpty());
	VIndex idx(0);
	size_t nJobs(0);
	while (idx<vertices.GetSize()) {
		const VIndex idxNext(MINF(idx+idxStep, vertices.GetSize()));
		events.AddEvent(new EVTSmoothVertices2(idx, idxNext));
		idx = idxNext;
		++nJobs;
	}
	WaitThreadWorkers(nJobs);
	}

	// set the final gradient as the combination of photometric and smoothness gradients;
	// only the callers that consume the combination ask for it (see the declaration)
	if (gradients) {
		if (bEnergyMode) {
			// exact gradient of the exact energy returned below: the raw per-vertex photometric
			// sum (no 1/c_v, which is the gradient of nothing) plus w*L^T L v
			FOREACH(v, vertices)
				((Point3d*)gradients)[v] = Cast<double>(photoGrad[v])*energyPhotoScale + Cast<double>(smoothGradLtL[v])*(double)weightRegularity;
		} else {
		// g_v, the photometric gradient the combination adds: divided by the number of
		// pair-directions that saw this vertex
		const auto PhotoTerm = [this](VIndex v) { return photoGrad[v]/photoGradNorm[v]; };
		if (ratioRigidityElasticity >= 1.f) {
			FOREACH(v, vertices)
				((Point3d*)gradients)[v] = photoGradNorm[v] > 0 ?
					Cast<double>(PhotoTerm(v) + smoothGrad2[v]*weightRegularity) :
					Cast<double>(smoothGrad2[v]*weightRegularity);
		} else {
			// compute smoothing gradient as a combination of level 1 and 2 of the Laplacian operator;
			// (see page 105 of "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004)
			const Real rigidity((Real(1)-ratioRigidityElasticity)*weightRegularity);
			const Real elasticity(ratioRigidityElasticity*weightRegularity);
			FOREACH(v, vertices)
				((Point3d*)gradients)[v] = photoGradNorm[v] > 0 ?
					Cast<double>(PhotoTerm(v) + smoothGrad2[v]*elasticity - smoothGrad1[v]*rigidity) :
					Cast<double>(smoothGrad2[v]*elasticity - smoothGrad1[v]*rigidity);
		}

		// CPU/CUDA parity diagnostic: dump the per-vertex terms this iteration combined,
		// right before they are gone (photoGrad/smoothGrad1/smoothGrad2 get
		// overwritten next ScoreMesh() call); no-op unless OMVS_REFINE_DEBUG_DIR is set
		// (which is also what makes the caller pass a non-NULL gradients here)
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
		}
	}

	// energy mode returns exactly the energy the gradient above belongs to:
	//   E = Sum_pairs RegScale_p Sum_pixels r (1-ZNCC) + w * 1/2 Sum_{v interior} ||L(v)||^2
	// the energy, accumulated in energy mode only: the stepper reads S and the per-vertex terms
	return energyPhoto*energyPhotoScale + (double)weightRegularity*energySmooth;
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


// check whether the nearest depth-map texel to the projected coordinate is not an occluder
bool MeshRefine::IsDepthSimilar(const DepthMap& depthMap, const Point2f& pt, Depth z, const BitMatrix& keepMask)
{
	// the rounded nearest tap read below must sit inside the shared Refine::Border margin, the
	// window half-size every per-pixel window statistic downstream needs around it; the bound is
	// one pixel tighter than the tap itself needs so that a pt anywhere in the accepted range
	// rounds into the margin (same rule as CUDA's kernelImageMeshWarp). The test is in FLOAT,
	// before FLOOR2INT: a point whose depth in the other camera is arbitrarily small projects far
	// outside any int range, and converting first would overflow and wrongly pass; this form also
	// rejects a non-finite pt
	if (!(pt.x >= (float)Refine::Border && pt.y >= (float)Refine::Border &&
		  pt.x < (float)(depthMap.cols-Refine::Border-1) && pt.y < (float)(depthMap.rows-Refine::Border-1)))
		return false;
	const ImageRef ir(FLOOR2INT(pt+Point2f(0.5f,0.5f)));
	ASSERT(ir.x >= Refine::Border && ir.y >= Refine::Border &&
		   ir.x < depthMap.cols-Refine::Border && ir.y < depthMap.rows-Refine::Border);
	const Depth& depth(depthMap(ir));
	if (!(depth > 0 && depth*1.0002f >= z))
		return false;
	// the same rounded tap the occlusion check above just read must also be kept in B
	return keepMask.empty() || keepMask.isSet(ir);
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
	const Image32F& imageB, Image32F& imageA, BitMatrix& mask,
	const BitMatrix& keepA, const BitMatrix& keepB)
{
	ASSERT(!imageA.empty());
	typedef Sampler::Linear<float> Sampler;
	const Sampler sampler;
	mask.create(imageA.size());
	mask.memset(0);
	for (int j=0; j<depthMapA.rows; ++j) {
		for (int i=0; i<depthMapA.cols; ++i) {
			// a masked-out pixel of A never seeds a warp sample, before any back-projection work
			if (!keepA.empty() && !keepA.isSet(j,i))
				continue;
			const Depth& depthA = depthMapA(j,i);
			if (depthA <= 0)
				continue;
			const Point3 X(cameraA.TransformPointI2W(Point3(i,j,depthA)));
			const Point3f ptC(cameraB.TransformPointW2C(X));
			// a point behind camera B is not seen by it: the mirrored projection can still land
			// inside the image and IsDepthSimilar's "depth*1.0002 >= z" is trivially true for a
			// non-positive z, so the producer has to reject it here (same test as CUDA's
			// kernelImageMeshWarp) -- this is what makes every consumer's positive-depth contract hold
			if (ptC.z <= 0)
				continue;
			const Point2f pt(cameraB.TransformPointC2I(ptC));
			if (!IsDepthSimilar(depthMapB, pt, ptC.z, keepB))
				continue;
			imageA(j,i) = imageB.sample<Sampler,Sampler::Type>(sampler, pt);
			mask.set(j,i);
		}
	}
}

// compute masked local window statistics and the per-pixel photometric gradient scale
MeshRefine::PairScore MeshRefine::ComputeWindowStats(
	const Image32F& imageA, const Image32F& imageB, BitMatrix& mask,
	TImage<Real>& imageDZNCC, TImage<Real>* imageZNCC, TImage<Real>* imageConf,
	bool bExactDerivative)
{
	ASSERT(imageA.size() == mask.size() && imageB.size() == mask.size() && !mask.empty());
	imageDZNCC.create(mask.size());
	imageDZNCC.memset(0);
	if (imageZNCC) { imageZNCC->create(mask.size()); imageZNCC->memset(0); }
	if (imageConf) { imageConf->create(mask.size()); imageConf->memset(0); }
	// the six window sums, each accumulated over the VALID pixels only: the products are zeroed
	// wherever the warp failed, so an unwarped pixel contributes nothing instead of contributing
	// image A's own value (which is what the old imageA.copyTo(imageAB) + unmasked integral
	// images did, driving ZNCC towards 1 exactly at the occlusion boundaries that need it least)
	DEC_Image(float, m);
	DEC_Image(float, pA); DEC_Image(float, pB);
	DEC_Image(float, pAA); DEC_Image(float, pBB); DEC_Image(float, pAB);
	m.create(mask.size());
	for (int r=0; r<mask.rows; ++r)
		for (int c=0; c<mask.cols; ++c)
			m(r,c) = mask(r,c) ? 1.f : 0.f;
	cv::multiply(imageA, m, pA);
	cv::multiply(imageB, m, pB);
	cv::multiply(pA, imageA, pAA);
	cv::multiply(pB, imageB, pBB);
	cv::multiply(pA, imageB, pAB);
	const cv::Size ksize(Refine::WindowSize, Refine::WindowSize);
	const cv::Point anchor(-1,-1);
	// every pixel visited below is at least HalfSize from the image edge, so the border mode
	// never actually contributes; normalize=false gives the raw window sums
	cv::boxFilter(m, m, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
	cv::boxFilter(pA, pA, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
	cv::boxFilter(pB, pB, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
	cv::boxFilter(pAA, pAA, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
	cv::boxFilter(pBB, pBB, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
	cv::boxFilter(pAB, pAB, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
	const int RowsEnd(mask.rows-HalfSize);
	const int ColsEnd(mask.cols-HalfSize);
	const float gateMeanDiff(OPTREFINE::fGateMeanDiff);
	const float gateVarRatio(OPTREFINE::fGateVarRatio);
	// the four per-window numerators of the exact derivative below, each zero at a window centre
	// that contributes nothing to the score (unwarped, or rejected by a gate)
	DEC_Image(float, q1); DEC_Image(float, q2); DEC_Image(float, q3); DEC_Image(float, q4);
	DEC_Image(float, q5); DEC_Image(float, q6);
	if (bExactDerivative) {
		q1.create(mask.size()); q1.memset(0);
		q2.create(mask.size()); q2.memset(0);
		q3.create(mask.size()); q3.memset(0);
		q4.create(mask.size()); q4.memset(0);
		q5.create(mask.size()); q5.memset(0);
		q6.create(mask.size()); q6.memset(0);
	}
	float sumRZ(0), sumR(0);
	double energyRZ(0);
	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r,c))
				continue;
			const float n(m(r,c));
			Refine::WindowStats s;
			if (!Refine::WindowStatsFromSums(n, pA(r,c), pB(r,c), pAA(r,c), pBB(r,c), pAB(r,c), gateMeanDiff, gateVarRatio, s)) {
				// a rejected pixel scores nothing, but its VALUE still enters the window sums of
				// every neighbour that was not rejected -- the sums above were taken over the warp
				// mask, before any of this. The stepper only ever reads the mask as "pixels that
				// scored", so it drops them; the energy mode has to keep them, or the derivative
				// with respect to those pixels is missing from the gradient of an energy that
				// really does depend on them. Their own q entries below stay zero, which is what
				// keeps them out of the score
				if (!bExactDerivative)
					mask.unset(r,c);
				continue;
			}
			float zncc, dzncc, conf;
			Refine::ZnccAndDerivative(s, n, imageA(r,c), imageB(r,c), zncc, dzncc, conf);
			imageDZNCC(r,c) = dzncc;
			if (imageZNCC) (*imageZNCC)(r,c) = zncc;
			if (imageConf) (*imageConf)(r,c) = conf;
			sumRZ += conf*(1.f-zncc);
			sumR += conf;
			if (bExactDerivative) {
				energyRZ += (double)conf*(double)(1.f-zncc);
				// d/dB_p of window c's own ZNCC is
				//   (1/n_c)[ (A_p-muA_c)/(sigmaA sigmaB)_c - ZNCC_c (B_p-muB_c)/sigmaB_c^2 ]
				// so the four sums below, box-filtered over the windows containing p, are the only
				// per-window quantities the exact derivative needs. The per-window 1/n_c is folded
				// in here rather than divided out once at p, so a window whose valid count differs
				// from its neighbours' (the mask border) is still weighted exactly right
				const float rn(conf/n);
				const float wAB(rn/sqrtf(s.varA*s.varB));
				const float wZ(rn*zncc/s.varB);
				q1(r,c) = wAB;
				q2(r,c) = wAB*s.muA;
				q3(r,c) = wZ;
				q4(r,c) = wZ*s.muB;
				// the reliability weight itself depends on the warped image through varB when
				// B's window is the less textured one: d conf/d varB times d varB/d B_p =
				// (2/n)(B_p - muB), so the window's term conf (1-ZNCC) also contributes
				// (1-ZNCC) conf' (2/n) (B_p - muB) at every pixel it contains -- two more sums,
				// the factor of B_p and its muB multiple. Without them the gradient was blind to
				// the energy's reward for flattening the warped image where the match is poor
				const float dconf(Refine::ZnccReliabilityDerivativeVarB(s.varA, s.varB));
				if (dconf > 0) {
					const float q((1.f-zncc)*dconf*2.f/n);
					q5(r,c) = q;
					q6(r,c) = q*s.muB;
				}
			}
		}
	}
	DST_Image(m);
	DST_Image(pA); DST_Image(pB);
	DST_Image(pAA); DST_Image(pBB); DST_Image(pAB);
	if (bExactDerivative) {
		cv::boxFilter(q1, q1, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
		cv::boxFilter(q2, q2, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
		cv::boxFilter(q3, q3, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
		cv::boxFilter(q4, q4, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
		cv::boxFilter(q5, q5, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
		cv::boxFilter(q6, q6, -1, ksize, anchor, false, cv::BORDER_CONSTANT);
		// d/dB_p Sum_c r_c (1-ZNCC_c) = -( A_p S1 - S2 - B_p S3 + S4 ) + ( B_p S5 - S6 ), the
		// pointwise value written above replaced by the sum over every window that contains p. A
		// pixel outside the mask enters no window sum (every product is masked), so its derivative
		// does not exist and its entry stays the 0 the map was created with
		for (int r=HalfSize; r<RowsEnd; ++r) {
			for (int c=HalfSize; c<ColsEnd; ++c) {
				if (!mask(r,c))
					continue;
				imageDZNCC(r,c) = -(imageA(r,c)*q1(r,c) - q2(r,c) - imageB(r,c)*q3(r,c) + q4(r,c)) + (imageB(r,c)*q5(r,c) - q6(r,c));
			}
		}
	}
	DST_Image(q1); DST_Image(q2); DST_Image(q3); DST_Image(q4);
	DST_Image(q5); DST_Image(q6);
	return PairScore{sumRZ, sumR, energyRZ};
}

// derivative of the bilinear reconstruction the warp samples with (see the declaration)
void MeshRefine::BilinearGradient(const Image32F& image, const Point2f& pt, Real& gx, Real& gy)
{
	const int x0(FLOOR2INT(pt.x)), y0(FLOOR2INT(pt.y));
	const Real dx((Real)pt.x-(Real)x0), dy((Real)pt.y-(Real)y0);
	const auto At = [&image](int y, int x) -> Real {
		return (x < 0 || x >= image.cols || y < 0 || y >= image.rows) ? Real(0) : (Real)image(y,x);
	};
	const Real v00(At(y0,x0)), v01(At(y0,x0+1)), v10(At(y0+1,x0)), v11(At(y0+1,x0+1));
	gx = (Real(1)-dy)*(v01-v00) + dy*(v11-v10);
	gy = (Real(1)-dx)*(v10-v00) + dx*(v11-v01);
}

// compute the photometric gradient for all vertices seen by an image pair
void MeshRefine::ComputePhotometricGradient(
	const Mesh::FaceArr& faces, const Mesh::NormalArr& normals,
	const DepthMap& depthMapA, const FaceMap& faceMapA, const BaryMap& baryMapA, const Camera& cameraA,
	const Camera& cameraB, const View& viewB,
	const PairMaps& maps, const PairGrads& grads, Real RegularizationScale, bool bExactDerivative,
	TImage<Real>* debugSG)
{
	const BitMatrix& mask(maps.mask);
	ASSERT(faces.GetSize() == normals.GetSize() && !faces.IsEmpty());
	ASSERT(debugSG == NULL || debugSG->size() == mask.size());
	ASSERT(depthMapA.size() == mask.size() && faceMapA.size() == mask.size() && baryMapA.size() == mask.size() && maps.dzncc.size() == mask.size() && !mask.empty());
	// imageGrad is only built when it is actually read (ThInitImage skips it in mode 3)
	ASSERT(!viewB.image.empty() && (bExactDerivative || viewB.image.size() == viewB.imageGrad.size()));
	const int RowsEnd(mask.rows-HalfSize);
	const int ColsEnd(mask.cols-HalfSize);
	typedef Sampler::Linear<View::Grad::Type> Sampler;
	const Sampler sampler;
	TMatrix<Real,2,3> xJac;
	Point2f xB;
	grads.photoGrad.Memset(0);
	grads.photoGradPixels.Memset(0);
	grads.footprint.MemsetValue(FLT_MAX); // sentinel; ThProcessPair only min-merges the vertices this direction actually saw

	// the per-pixel geometry ReadPixelGeom below reads off the maps of image A alone
	struct PixelGeom {
		FIndex idxFace;
		Point3 rayA;
		Grad N, dA;
		Real Nd, f;
		Depth depth;
	};
	// the part read off the maps of image A alone; false rejects a pixel too grazing to steer a
	// vertex with (its 1/Nd would amplify whatever noise it holds)
	const auto ReadPixelGeom = [&](int r, int c, PixelGeom& g) -> bool {
		g.idxFace = faceMapA(r,c);
		ASSERT(g.idxFace != NO_ID);
		g.N = normals[g.idxFace];
		g.rayA = cameraA.RayPoint(Point2(c,r));
		g.dA = normalized(g.rayA);
		g.Nd = g.N.dot(g.dA);
		if (g.Nd > -0.1)
			return false;
		g.depth = depthMapA(r,c);
		ASSERT(g.depth > 0);
		// scene units per pixel at this vertex, camera A, current scale (Camera::GetFootprintWorld)
		g.f = cameraA.GetFootprintWorld(g.depth);
		return true;
	};
	// forward-project into B and sample its image gradient there, giving gB.(J.dA), the geometric
	// factor the ZNCC derivative is multiplied by
	const auto ProjectedGradient = [&](const PixelGeom& g) -> Real {
		const Point3 X(g.rayA*REAL(g.depth)+cameraA.C);
		// project point in second image and
		// projection Jacobian matrix in the second image of the 3D point on the surface
		MAYBEUNUSED const float depthB(ProjectVertex(cameraB.P.val, X.ptr(), xB.ptr(), xJac.val));
		ASSERT(depthB > 0);
		// compute gradient in image B
		TMatrix<Real,1,2> gB;
		if (bExactDerivative)
			BilinearGradient(viewB.image, xB, gB(0), gB(1));
		else
			gB = viewB.imageGrad.sample<Sampler,View::Grad>(sampler, xB);
		return (gB*(xJac*(const TMatrix<Real,3,1>&)g.dA))(0);
	};

	for (int r=HalfSize; r<RowsEnd; ++r) {
		for (int c=HalfSize; c<ColsEnd; ++c) {
			if (!mask(r,c))
				continue;
			PixelGeom g;
			if (!ReadPixelGeom(r, c, g))
				continue;
			// compute gradient scale
			const Real dot(ProjectedGradient(g));
			const Real dZNCC(maps.dzncc(r,c));
			const Real gp(dot*dZNCC*RegularizationScale/g.Nd);
			if (debugSG)
				(*debugSG)(r,c) = gp;
			// add gradient to the three vertices
			const Face& face(faces[g.idxFace]);
			const Point3f& b(baryMapA(r,c));
			for (int v=0; v<3; ++v) {
				const Grad grad(g.N*(gp*(Real)b[v]));
				const VIndex idxVert(face[v]);
				grads.photoGrad[idxVert] += grad;
				++grads.photoGradPixels[idxVert];
				if (g.f < grads.footprint[idxVert])
					grads.footprint[idxVert] = g.f;
			}
		}
	}
}

// computes the discrete analog of the Laplacian using
// the umbrella-operator on the first triangle ring at each point
void MeshRefine::ComputeSmoothnessGradient1(
	const Mesh::VertexArr& vertices, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
	GradArr& smoothGrad1, VIndex idxStart, VIndex idxEnd)
{
	ASSERT(!vertices.IsEmpty() && vertices.GetSize() == vertexVertices.GetSize() && vertices.GetSize() == smoothGrad1.GetSize());
	for (VIndex idxV=idxStart; idxV<idxEnd; ++idxV) {
		Grad& grad = smoothGrad1[idxV];
		grad = Grad::ZERO;
		if (vertexBoundary[idxV])
			continue;
		const Mesh::VertexIdxArr& verts = vertexVertices[idxV];
		if (verts.IsEmpty())
			continue;
		// accumulate the differences to the centre, not the coordinates: summing coordinates of
		// order 1e2 and subtracting the centre afterwards left a residual of order 1e-3 with a
		// relative error the energy arm's tolerance could not tell from a real change
		const Grad center(Cast<Real>(vertices[idxV]));
		FOREACH(v, verts)
			grad += Cast<Real>(vertices[verts[v]])-center;
		grad = grad/(Real)verts.GetSize();
		ASSERT(ISFINITE(grad));
	}
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
		if (vertexBoundary[idxV])
			continue;
		const Mesh::VertexIdxArr& verts = vertexVertices[idxV];
		if (verts.IsEmpty())
			continue;
		Real w(0);
		FOREACH(v, verts) {
			const VIndex idxVert(verts[v]);
			grad += smoothGrad1[idxVert];
			// adjacency is symmetric, so a vertex listed as a neighbour has this vertex among its
			// own: its valence is at least 1 by construction (ListIncidentVertices)
			const VIndex numVert(vertexVertices[idxVert].GetSize());
			ASSERT(numVert > 0);
			w += Real(1)/(Real)numVert;
		}
		const Real numVert((Real)verts.GetSize());
		const Real nrm(Real(1)/(Real(1)+w/numVert));
		grad = grad*(nrm/numVert) - smoothGrad1[idxV]*nrm;
	}
}
// exact gradient of the thin-plate energy 1/2 Sum_{v interior} ||L(v)||^2 (see the declaration)
void MeshRefine::ComputeSmoothnessGradientLtL(
	const GradArr& smoothGrad1, const Mesh::VertexVerticesArr& vertexVertices, const BoolArr& vertexBoundary,
	GradArr& smoothGradLtL, VIndex idxStart, VIndex idxEnd)
{
	ASSERT(!smoothGrad1.IsEmpty() && smoothGrad1.GetSize() == vertexVertices.GetSize() && smoothGrad1.GetSize() == smoothGradLtL.GetSize());
	for (VIndex idxV=idxStart; idxV<idxEnd; ++idxV) {
		Grad& grad = smoothGradLtL[idxV];
		grad = Grad::ZERO;
		const Mesh::VertexIdxArr& verts = vertexVertices[idxV];
		FOREACH(v, verts) {
			const VIndex idxVert(verts[v]);
			// a boundary neighbour carries no residual of its own, so it drops out of the transpose
			if (vertexBoundary[idxVert])
				continue;
			// adjacency is symmetric (ListIncidentVertices), so this vertex is one of idxVert's own
			// neighbours and its valence is at least 1
			const VIndex numVert(vertexVertices[idxVert].GetSize());
			ASSERT(numVert > 0);
			grad += smoothGrad1[idxVert]/(Real)numVert;
		}
		// the [v interior] factor of the second term needs no test: a boundary vertex's own
		// residual is identically zero. A boundary vertex still keeps the first sum -- it moves the
		// umbrella of every interior neighbour, so the energy really does depend on it, and this
		// solver optimizes its coordinates like any other. Zeroing the row instead (what the
		// legacy descent direction does) would put the cost and the gradient back out of step,
		// which is the whole defect this mode exists to remove
		grad -= smoothGrad1[idxV];
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
	ViewScoreArr neighbors;
	if (!SelectRefineNeighbors(scene, idxImage, nMaxViews, neighbors))
		return;
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
	// compute image gradient (the same estimator on both backends); mode 3 samples the bilinear
	// interpolant of the image directly (ComputePhotometricGradient's BilinearGradient) and never
	// reads this precomputed stencil, so building it here would be wasted work
	if (OPTREFINE::nImageGradient != 3) {
		Image32F grad[2];
		ComputeRefineImageGradient(img, grad[0], grad[1]);
		cv::merge(grad, 2, view.imageGrad);
	}
	// per-view keep-mask at this scale's working size (the same estimator on both backends)
	PrepareRefineImageMask(imageData, img.size(), view.keepMask);
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
	imageAB.create(imageA.size());
	// invalid pixels stay 0: ComputeWindowStats masks them out of every window sum
	imageAB.memset(0);
	ImageMeshWarp(depthMapA, cameraA, depthMapB, cameraB, imageB, imageAB, mask, viewA.keepMask, viewB.keepMask);
	// compute the masked window statistics, the rejection gates and the ZNCC derivative
	// (this also prunes the mask, which every consumer below therefore reads after the call)
	uint32_t dbgImageA, dbgImageB;
	const bool dbgPair(!RefineDebug::Dir().empty() && RefineDebug::Pair(dbgImageA, dbgImageB) &&
		dbgImageA == idxImageA && dbgImageB == idxImageB);
	DEC_Image(Real, imageDZNCC);
	DEC_Image(Real, imageZNCC);
	DEC_Image(Real, conf);
	// the reliability weight is a per-pixel map only for the debug pair export
	const PairScore pairScore(ComputeWindowStats(imageA, imageAB, mask, imageDZNCC,
		dbgPair ? &imageZNCC : NULL, dbgPair ? &conf : NULL, bEnergyMode));

	// CPU/CUDA parity diagnostic: dump this pair's maps before the pooled buffers
	// above get recycled; no-op unless OMVS_REFINE_DEBUG_DIR/_PAIR are both set
	// and match this exact (A,B) direction
	if (dbgPair) {
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
	}

	#ifdef MESHOPT_TYPEPOOL
	DST_Image(imageZNCC);
	DST_Image(imageAB);
	#endif
	// compute field gradient
	GradArr _photoGrad(photoGrad.GetSize());
	UnsignedArr _photoGradPixels(photoGrad.GetSize());
	FloatArr _footprint(photoGrad.GetSize());
	const Real RegularizationScale((Real)((REAL)(imageDataA.avgDepth*imageDataB.avgDepth)/(cameraA.GetFocalLength()*cameraB.GetFocalLength())));
	// energy mode: this direction's frozen-domain weight, its
	// reliability sum at the scale's calibration evaluation over the current one, so a step that
	// drops scored pixels does not lower the energy just by dropping them; 1 outside the energy
	// mode, before the calibration, and for a direction that scored nothing then
	Real energyWeight(1);
	if (bEnergyMode && !pairSumR0.empty()) {
		const auto it(pairSumR0.find(PairKey(idxImageA, idxImageB)));
		if (it != pairSumR0.end() && it->second > 0 && pairScore.sumR > 0)
			energyWeight = (Real)(it->second/pairScore.sumR);
	}
	TImage<Real> dbgSG;
	if (dbgPair) {
		dbgSG.create(mask.size());
		dbgSG.memset(0);
	}
	const PairMaps maps{imageDZNCC, mask};
	const PairGrads grads{_photoGrad, _photoGradPixels, _footprint};
	// bEnergyMode always wants the bilinear derivative (the energy's own consistency requirement);
	// OPTREFINE::nImageGradient == 3 asks for the same source on the stepper path too
	ComputePhotometricGradient(faces, faceNormals, depthMapA, faceMapA, baryMapA, cameraA, cameraB, viewB, maps, grads, RegularizationScale*energyWeight, bEnergyMode || OPTREFINE::nImageGradient == 3, dbgPair ? &dbgSG : NULL);
	if (dbgPair) {
		// CPU/CUDA parity diagnostic, continued: the per-pixel photometric scalar each pixel
		// hands to its face's 3 vertices, and the raw face map of A (-1 where nothing was
		// rasterised; NOT masked, so rasterisation and warp differences can be told apart)
		Image32F faceF(mask.size());
		for (int r=0; r<mask.rows; ++r)
			for (int c=0; c<mask.cols; ++c)
				faceF(r,c) = faceMapA(r,c) == NO_ID ? -1.f : (float)faceMapA(r,c);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "sg", dbgSG.getData(), dbgSG.width(), dbgSG.height());
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "face", faceF.getData(), faceF.width(), faceF.height());
	}
	DST_Image(conf);
	DST_Image(imageDZNCC);
	DST_BitMatrix(mask);
	Lock l(cs);
	FOREACH(i, photoGrad) {
		if (_photoGradPixels[i] > 0) {
			photoGrad[i] += _photoGrad[i];
			photoGradNorm[i] += 1.f;
			if (_footprint[i] < footprint[i])
				footprint[i] = _footprint[i];
		}
	}
	// raw (unscaled by RegularizationScale) reliability sums for S = sumRZ/sumR (ScoreMesh)
	sumR += pairScore.sumR;
	sumRZ += pairScore.sumRZ;
	if (bEnergyMode) {
		energyPhoto += (double)(RegularizationScale*energyWeight)*pairScore.energyRZ;
		pairSumR[PairKey(idxImageA, idxImageB)] = pairScore.sumR;
	}
}
void MeshRefine::ThSmoothVertices1(VIndex idxStart, VIndex idxEnd)
{
	ComputeSmoothnessGradient1(vertices, vertexVertices, vertexBoundary, smoothGrad1, idxStart, idxEnd);
	// energy mode scores the thin-plate energy 1/2 Sum ||L(v)||^2 that L^T L v is the exact
	// gradient of
	if (!bEnergyMode)
		return;
	double energy(0);
	for (VIndex idxV=idxStart; idxV<idxEnd; ++idxV)
		if (!vertexBoundary[idxV])
			energy += (double)normSq(smoothGrad1[idxV]);
	Lock l(cs);
	energySmooth += 0.5*energy;
}
void MeshRefine::ThSmoothVertices2(VIndex idxStart, VIndex idxEnd)
{
	if (bEnergyMode)
		ComputeSmoothnessGradientLtL(smoothGrad1, vertexVertices, vertexBoundary, smoothGradLtL, idxStart, idxEnd);
	else
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
#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#pragma pop_macro("ERROR")
#pragma pop_macro("CHECK")
#pragma pop_macro("LOG")

namespace ceres {
// the Ceres arm's problem: MeshRefine runs in energy mode, so the cost this returns and the
// gradient it fills are one consistent pair -- what a line search enforcing a curvature condition
// tests, and what the legacy pair (different constants, a different smoothing functional and a
// per-vertex 1/c_v that is the gradient of nothing) could not provide
class MeshProblem : public FirstOrderFunction, public IterationCallback
{
public:
	// the parameters are the vertex coordinates in scene units, deliberately not pixels: Ceres's
	// first line search moves the largest-gradient coordinate by one parameter unit, which in
	// scene units is below a pixel for the whole mesh (about half a pixel for the median vertex
	// under the stepper-scaled energy) while in pixel units it is a full pixel for one outlier
	// vertex and nothing for the rest, a first curvature pair L-BFGS cannot use (measured, see
	// the design document)
	MeshProblem(MeshRefine& _refine) : refine(_refine), params(refine.vertices.GetSize()*3), bestCost(DBL_MAX) {
		ASSERT(refine.bEnergyMode);
		// init params
		FOREACH(i, refine.vertices)
			*((Point3d*)params.Begin()+i) = refine.vertices[i];
	}
	virtual ~MeshProblem() {}

	// the solver's parameter block: Ceres writes the solution into it when the solve is usable
	// and nothing else touches it (the probes Evaluate receives go to the mesh directly), so after
	// Solve it holds either the solution or the scale's start
	void ApplyParams() const { Apply(params.Begin()); }
	// the lowest-cost surface any evaluation reached -- at least as good as the last accepted
	// iterate, and what a solve that ends in FAILURE leaves behind; empty only if not a single
	// evaluation was valid
	bool ApplyBestParams() const {
		if (best.IsEmpty())
			return false;
		Apply(best.Begin());
		return true;
	}

	bool Evaluate(const double* const parameters, double* cost, double* gradient) const {
		// update surface parameters
		Apply(parameters);
		// evaluate residuals and gradients
		Point3dArr gradients;
		if (!gradient) {
			gradients.Resize(refine.vertices.GetSize());
			gradient = (double*)gradients.Begin();
		}
		*cost = refine.ScoreMesh(gradient);
		// a surface the energy cannot score (no pair-direction contributed a pixel) or a
		// non-finite energy is not an iterate the solver may accept: false makes the line search
		// contract its step instead of comparing garbage
		if (!(refine.S >= 0) || !std::isfinite(*cost))
			return false;
		if (*cost < bestCost) {
			bestCost = *cost;
			best.Resize(params.GetSize());
			memcpy(best.Begin(), parameters, sizeof(double)*params.GetSize());
		}
		return true;
	}

	CallbackReturnType operator()(const IterationSummary& summary) {
		refine.iteration = summary.iteration;
		// one line per L-BFGS iteration (bench/refine_log.py RE_ITER_CERES reads the first two
		// fields): the energy, the accepted line-search step, and the score and reliability sum of
		// the iterate so a run can be read for the two things the energy alone hides -- whether
		// the per-pixel score really improves, and whether the solver is shrinking the scored
		// domain instead (a sum over valid pixels rewards losing pixels)
		DEBUG_EXTRA("\t%2d. E: %.6g\tstep: %.3g\tS: %.5f\tn: %.6g", (int)summary.iteration, summary.cost, summary.step_size, refine.S, refine.sumR);
		return ceres::SOLVER_CONTINUE;
	}

	int NumParameters() const { return (int)params.GetSize(); }
	const double* GetParameters() const { return params.Begin(); }
	double* GetParameters() { return params.Begin(); }

protected:
	void Apply(const double* parameters) const {
		FOREACH(i, refine.vertices)
			refine.vertices[i] = *((const Point3d*)parameters+i);
	}

protected:
	MeshRefine& refine;
	DoubleArr params;
	mutable DoubleArr best; // lowest-cost surface any evaluation reached (empty until one is valid)
	mutable double bestCost;
};
} // namespace ceres

#endif // MESHOPT_CERES


// optimize mesh using photo-consistency
// fThPlanarVertex - threshold used to remove vertices on planar patches, as a fraction of the
//   vertex depth (its footprint times the median view focal length; 0 - disable)
// bUseCeres - minimize the exact energy with Ceres instead of the bold-driver stepper (slower and
//   no better, kept as the reference arm; see the design document)
bool Scene::RefineMesh(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews,
					   float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nMaxFaceArea,
					   unsigned nScales, float fScaleStep,
					   unsigned nAlternatePair, float fRegularityWeight, float fRatioRigidityElasticity, float fThPlanarVertex, bool bUseCeres)
{
	// the externally controlled knobs are validated here, at the entry point every caller goes
	// through: their ASSERT twins inside the stepper are contracts, not input checks
	if (bUseCeres) {
		#ifndef MESHOPT_CERES
		VERBOSE("error: this build does not include the Ceres optimizer");
		return false;
		#else
		// the Ceres arm minimizes one explicit energy
		//   E = Sum_pairs RegScale_p Sum_pixels r (1-ZNCC) + w/2 Sum_{v interior} ||L(v)||^2
		// and hands the line search its exact gradient: it can neither change the vertex set under
		// the solver (its parameter count is fixed for the whole solve) nor the functional between
		// evaluations (an alternating pair direction), and --rigidity-elasticity-ratio has no
		// meaning in its pure thin-plate term
		if (fThPlanarVertex > 0) {
			VERBOSE("error: the Ceres optimizer cannot remove planar vertices (--planar-vertex-ratio must be 0)");
			return false;
		}
		if (nAlternatePair != 0) {
			VERBOSE("error: the Ceres optimizer scores both directions of every pair (--alternate-pair must be 0)");
			return false;
		}
		#endif
	}
	// the upper bound is the stepper's explicit-flow stability limit; the line-search arm has no
	// such bound, its weight balances two terms of one energy and any non-negative value is valid
	if (!(fRegularityWeight >= 0) || (!bUseCeres && fRegularityWeight*MeshRefineStep::StepMax > 1.f)) {
		VERBOSE("error: --regularity-weight %g is outside [0, %g]: one full step of the"
			" regularization term must not amplify the Laplacian it is applied to"
			" (explicit-flow stability)",
			fRegularityWeight, 1.f/MeshRefineStep::StepMax);
		return false;
	}
	// an unknown stencil id used to fall silently to the default one, so an A/B that asked for a
	// stencil this build does not have measured the default twice
	if (OPTREFINE::nImageGradient < 0 || OPTREFINE::nImageGradient > 3) {
		VERBOSE("error: image gradient stencil %d is not implemented (0 - 3x5 separable, 1 - central, 2 - Sobel, 3 - bilinear interpolant derivative)", OPTREFINE::nImageGradient);
		return false;
	}

	bool bGeneratedPointcloud(false);
	if (pointcloud.IsEmpty() && !ImagesHaveNeighbors()) {
		SampleMeshWithVisibility();
		bGeneratedPointcloud = true;
	}

	MeshRefine refine(*this, nAlternatePair, fRegularityWeight, fRatioRigidityElasticity, nResolutionLevel, nMinResolution, nMaxViews, nMaxThreads);
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
		if (bUseCeres) {
			// DefineProblem: energy mode, so the cost and the gradient are the same functional
			refine.bEnergyMode = true;
			refine.bEnergyPhoto = true;
			refine.iteration = 0;
			refine.energyPhotoScale = 1;
			refine.pairSumR0.clear();
			// calibration evaluation: the scale-start constants of the normalized energy
			refine.ScoreMesh(NULL);
			if (refine.S < 0) {
				VERBOSE("error: no image pair scores a single pixel at scale %u", nScale);
				return false;
			}
			// the photometric term scaled like the stepper's direction: its gradient at
			// the scale start has median |g_v|/s_v = 1/kappa pixels, so the regularity
			// weight balances it as it balances the stepper's normalized photometric step
			FloatArr calib(0, refine.vertices.GetSize());
			FOREACH(v, refine.vertices)
				if (refine.footprint[v] > 0)
					calib.Insert(norm(refine.photoGrad[v])/refine.footprint[v]);
			const float m(calib.IsEmpty() ? 0.f : calib.GetMedian());
			if (m > 0)
				refine.energyPhotoScale = 1.0/((double)MeshRefineStep::Kappa*m);
			// the pair-direction count per seen vertex is the factor by which this energy's
			// photometric pull exceeds the stepper's (which divides by it) at the same weight
			double sumCount(0);
			VIndex numSeen(0);
			FOREACH(v, refine.vertices) {
				if (refine.photoGradNorm[v] > 0) {
					sumCount += refine.photoGradNorm[v];
					++numSeen;
				}
			}
			DEBUG_EXTRA("Ceres arm scale %u: median |g|/s %g px, %.1f pair-directions per seen vertex, photometric energy scale %g",
				nScale, m, numSeen ? sumCount/numSeen : 0.0, refine.energyPhotoScale);
			refine.pairSumR0 = refine.pairSumR;
			ceres::MeshProblem* problemData(new ceres::MeshProblem(refine));
			#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 3)
			ceres::GradientProblem problem{std::unique_ptr<ceres::FirstOrderFunction>(problemData)};
			#else
			ceres::GradientProblem problem(problemData);
			#endif
			// SetMinimizerOptions
			ceres::GradientProblemSolver::Options options;
			// the iteration callback below logs every iteration into the app's own log; Ceres's
			// progress lines would go to stdout through glog, a second sink nobody reads
			options.logging_type = ceres::LoggingType::SILENT;
			options.minimizer_progress_to_stdout = false;
			// the Ceres 2.2 defaults: L-BFGS (rank 20, no Oren-Luenberger scaling, which Ceres
			// documents as harmful where the parameter sensitivities vary as widely as they do
			// here) with the Wolfe line search. A sweep of every direction and line search the
			// solver offers, the L-BFGS rank, the scaling and the function tolerance found no
			// configuration that wins on more than one of three ground-truth scenes (design
			// document, the Ceres section)
			options.line_search_direction_type = ceres::LBFGS;
			options.line_search_type = ceres::WOLFE;
			// the stopping rule is the relative function tolerance below; the cap is a safety net,
			// never reached on any measured scene (the longest scale ran 88 iterations), and an
			// L-BFGS iteration costs one energy evaluation whenever the unit step satisfies the
			// Wolfe conditions, which it does on most of them
			options.max_num_iterations = 100;
			options.function_tolerance = 1e-4;
			// both are absolute thresholds on quantities carrying the scene's own units (the
			// photometric energy scales with RegularizationScale and the pixel count), so no fixed
			// value means the same thing on two scenes: the relative function tolerance and the
			// iteration cap are the only stopping rules this arm can state honestly
			options.gradient_tolerance = 0;
			options.parameter_tolerance = 0;
			// the callback only logs; the best-cost snapshot lives in Evaluate
			options.update_state_every_iteration = false;
			options.callbacks.push_back(problemData);
			ceres::GradientProblemSolver::Summary summary;
			// SolveProblem
			ceres::Solve(options, problem, problemData->GetParameters(), &summary);
			DEBUG_ULTIMATE("%s", summary.FullReport().c_str());
			switch (summary.termination_type) {
			case ceres::TerminationType::NO_CONVERGENCE:
				DEBUG_EXTRA("CERES: maximum number of iterations reached!");
				[[fallthrough]];
			case ceres::TerminationType::CONVERGENCE:
			case ceres::TerminationType::USER_SUCCESS:
				problemData->ApplyParams();
				break;
			default:
				// FAILURE: a numerical failure of the line search (a non-finite or invalid
				// evaluation it could not step away from, a direction with no descent), which on
				// this only-piecewise-smooth energy does happen. Ceres then leaves the parameter
				// block at the scale's start; apply the best surface any evaluation reached and
				// carry on with the next scale, abandoning the whole refinement threw away every
				// scale already solved
				VERBOSE("CERES surface refine stopped at scale %u: %s", nScale, summary.message.c_str());
				if (!problemData->ApplyBestParams())
					problemData->ApplyParams();
				break;
			}
			refine.bEnergyMode = false;
		} else
		#endif // MESHOPT_CERES
		{
			// pixel-unit bold driver (MeshRefineStep, SceneRefineCommon.h)
			const int cap((int)MeshRefineStep::Budget(nScale));
			const bool bAlternating(nAlternatePair == 1);
			const bool bPlanarHook(fThPlanarVertex > 0);
			// the combined gradient is a full extra pass over the vertices, only the planar hook
			// and the debug export read it, and ScoreMesh skips it when it is not asked for
			const bool bNeedGradients(bPlanarHook || !RefineDebug::Dir().empty());
			// the planar threshold is a fraction of the vertex depth, reconstructed from the
			// per-vertex footprint the stepper already gets (footprint = depth/focal at the current
			// scale) times the median focal of the views scoring this scale; a per-vertex depth
			// instead of the whole-image average depth the legacy threshold used
			float fMedianFocal(0);
			if (bPlanarHook) {
				FloatArr focals(0, images.GetSize());
				for (const Image& imageData: images)
					if (imageData.IsValid())
						focals.Insert((float)imageData.camera.K(0,0));
				if (!focals.IsEmpty())
					fMedianFocal = focals.GetMedian();
			}

			MeshRefineStep stepper;
			stepper.Reset(refine.vertices.GetSize());

			Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> gradients(refine.vertices.GetSize(),3);
			// set when an evaluation finds the scene cannot be scored at all (S < 0, see
			// ScoreMesh): both phase loops stop and the whole refinement fails
			bool bScoreFailed(false);
			GET_LOGCONSOLE().Pause();

			// one energy evaluation, stepper decision and log line; phaseCap/allowPlanarHook let
			// the same body serve both phases below
			const auto RunEvaluation = [&](Real rho, bool allowPlanarHook, int idx, int phaseCap) -> MeshRefineStep::Action {
				refine.iteration = stepper.GetNumEvaluated();
				refine.ratioRigidityElasticity = rho;
				const unsigned numAcceptedSoFar(stepper.GetNumAccepted());
				const bool bAdaptMesh(allowPlanarHook && numAcceptedSoFar >= 4 && (numAcceptedSoFar-4)%3 == 0 && phaseCap-idx > 5);
				refine.ScoreMesh(bNeedGradients ? gradients.data() : NULL);
				if (refine.S < 0) {
					// no image pair contributed a single pixel (see ScoreMesh): an expected,
					// recoverable outside-world failure -- abort the refinement loudly
					VERBOSE("error: no image pair overlap at scale %u: mesh refinement aborted", nScale);
					bScoreFailed = true;
					return MeshRefineStep::STOP;
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
				terms.alternating = bAlternating;

				MeshRefineStep::Stats stats;
				const MeshRefineStep::Action action(stepper.Evaluate(terms, refine.vertices, stats));

				// remove planar vertices (small gradient and almost on the center of their
				// surrounding patch) AFTER the step was applied: the gradient and smoothing terms
				// it tests were measured on the pre-step surface, and the removal permutes the
				// vertex indexing (swap-with-last), so everything the stepper indexes by vertex
				// must already have been consumed
				VIndex numVertsRemoved(0);
				if (bAdaptMesh && action == MeshRefineStep::APPLY) {
					// one evaluation removes vertices with pairwise DISJOINT one-rings only, so every
					// hole the removal opens is one vertex's ring: a simple loop the fill spans as a
					// fan, leaving the surface manifold and closed. Removing a whole planar patch at
					// once opened multi-ring holes (non-manifold after the fill, and the next scale's
					// subdivision asserts on those), and even non-adjacent vertices whose rings share
					// a vertex leave two holes touching at that vertex, one non-simple boundary loop
					// the fill skips. The rest of the patch goes in the following evaluations, each
					// judged on the re-listed adjacency. A ring that touches the mesh's own open
					// boundary is skipped for the same reason: the hole's loop would meet the boundary
					// loop at that vertex and the fill leaves such a merged loop open
					Mesh::VertexIdxArr vertexRemove;
					BoolArr vertexTaken(refine.vertices.GetSize()); // in the ring (or the center) of a vertex already selected
					vertexTaken.Memset(0);
					FOREACH(v, refine.vertices) {
						// a vertex no pair-direction saw has no depth and no photometric evidence
						// that it is planar
						const float footprint(refine.footprint[v]);
						if (footprint == 0 || refine.vertexBoundary[v] || vertexTaken[v])
							continue;
						const float th(fThPlanarVertex*footprint*fMedianFocal);
						const double gn(norm(Point3d(gradients.row(v))));
						if ((float)gn >= th || norm(refine.smoothGrad1[v]) >= th)
							continue;
						const Mesh::VertexIdxArr& ring = refine.vertexVertices[v];
						bool bRingBlocked(false);
						for (VIndex n: ring) {
							if (vertexTaken[n] || refine.vertexBoundary[n]) {
								bRingBlocked = true;
								break;
							}
						}
						if (bRingBlocked)
							continue;
						vertexTaken[v] = true;
						for (VIndex n: ring)
							vertexTaken[n] = true;
						vertexRemove.Insert(v);
					}
					if (!vertexRemove.IsEmpty()) {
						// RemoveVerticesAndFill returns the number of holes it spanned, and the
						// removal can drop further vertices the patched surface no longer
						// references, so the count reported below is the change in vertices
						const VIndex numVertsBefore(refine.vertices.GetSize());
						mesh.RemoveVerticesAndFill(vertexRemove); // rebuilds the adjacency and boundary lists it found
						numVertsRemoved = numVertsBefore-refine.vertices.GetSize();
						// the surface changed under the stepper: its undo buffer and its S no
						// longer describe the same vertices, so the next evaluation cannot be
						// rejected against the S just measured
						stepper.TopologyChanged(refine.vertices.GetSize());
					}
				}

				DEBUG_EXTRA("\t%2d. S: %.5f (%+.2e)\tstep: %.3fpx\tmed: %.3fpx\tv: %5u\t%s",
					(int)stepper.GetNumEvaluated(), stats.S, stats.relChange, stats.step, stats.medianPx, numVertsRemoved, stats.accepted ? "acc" : "rej");
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
			if (bScoreFailed) {
				GET_LOGCONSOLE().Play();
				return false;
			}

			// Phase B: pure elasticity, planar hook off; eta and the S references carry over, the
			// budget is the stepper's
			const int capB((int)stepper.BeginSecondPhase());
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
			if (bScoreFailed) {
				GET_LOGCONSOLE().Play();
				return false;
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


// finite-difference consistency gate for the energy the Ceres arm minimizes; the whole point of
// the arm is that this pair agrees, and the arm is not compiled in every build, so the gate has to
// be reachable without it (see the declaration in Scene.h)
bool Scene::RefineMeshEnergyProbe(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews, RefineEnergyProbe& probe)
{
	bool bGeneratedPointcloud(false);
	if (pointcloud.IsEmpty() && !ImagesHaveNeighbors()) {
		SampleMeshWithVisibility();
		bGeneratedPointcloud = true;
	}
	// both pair-directions of every pair (nAlternatePair 0), so the energy does not depend on the
	// iteration parity the alternating arm switches on; the rigidity/elasticity ratio is unused by
	// the energy mode
	MeshRefine refine(*this, 0, probe.regularityWeight, 1.f, nResolutionLevel, nMinResolution, nMaxViews, nMaxThreads);
	if (bGeneratedPointcloud)
		pointcloud.Release();
	if (!refine.IsValid())
		return false;
	refine.bEnergyMode = true;
	refine.bEnergyPhoto = probe.photometric;
	// the single-scale case of RefineMesh's schedule above: image scale 1, blur 0.12*2+0.2
	if (!refine.InitImages(1.f, Real(0.44)))
		return false;
	refine.ListVertexFacesPre();
	refine.ListVertexFacesPost();

	// The direction u: a seeded but BAND-LIMITED field, three sinusoids of the vertex position per
	// axis with random phases and roughly half a period across the mesh, scaled so the largest
	// per-vertex displacement is exactly the offset in probe.steps (which can therefore be stated
	// as a fraction of the pixel footprint). It has to be band-limited: a per-vertex white-noise
	// direction moves neighbouring vertices apart, and a surface roughened at sub-pixel scale
	// drops warp samples out of the depth-similarity test -- each dropped sample changes the
	// statistics of all WindowArea windows containing it and, having removed a matching sample,
	// always RAISES 1-ZNCC. That is an always-positive term of the energy that grows with |t| and
	// no gradient predicts, so it swamps the directional derivative of a small step. It is the
	// piecewise smoothness of this energy, not an error in the gradient, and a band-limited step
	// leaves the surface as smooth as it found it
	const VIndex numVertices(mesh.vertices.GetSize());
	Mesh::VertexArr dir(numVertices);
	uint32_t state(probe.seed);
	const auto NextUniform = [&state]() {
		state = state*1664525u + 1013904223u;
		return (Real)(state>>8)/(Real)(1u<<24);
	};
	AABB3f aabb(mesh.vertices.Begin(), numVertices);
	const Point3f center(aabb.GetCenter());
	const Point3f extent(aabb.GetSize()*0.5f);
	ASSERT(extent.x > 0 && extent.y > 0); // the field below is parameterized over x and y
	Real phase[6], freq[6];
	for (int i=0; i<6; ++i) {
		phase[i] = NextUniform()*Real(2*M_PI);
		freq[i] = Real(1) + NextUniform()*Real(2); // radians per half-extent: well under one period
	}
	Real maxLen(0);
	FOREACH(v, dir) {
		const Real p((mesh.vertices[v].x-center.x)/extent.x);
		const Real q((mesh.vertices[v].y-center.y)/extent.y);
		const Real dx((Real)(SIN(freq[0]*p+phase[0])*COS(freq[1]*q+phase[1])));
		const Real dy((Real)(SIN(freq[2]*q+phase[2])*COS(freq[3]*p+phase[3])));
		const Real dz((Real)(SIN(freq[4]*p+phase[4])*SIN(freq[5]*q+phase[5])));
		dir[v] = Mesh::Vertex(dx, dy, dz);
		const Real len((Real)norm(dir[v]));
		if (len > maxLen)
			maxLen = len;
	}
	if (!(maxLen > 0))
		return false; // the field collapsed to zero everywhere: nothing to vary
	FOREACH(v, dir)
		dir[v] = dir[v]*(Real(1)/maxLen);

	// E and its exact gradient at the current mesh
	Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> gradients(numVertices,3);
	probe.energy = refine.ScoreMesh(gradients.data());
	probe.dirDerivative = 0;
	FOREACH(v, dir)
		probe.dirDerivative +=
			gradients(v,0)*(double)dir[v].x + gradients(v,1)*(double)dir[v].y + gradients(v,2)*(double)dir[v].z;

	// E at the offset meshes, then put the mesh back exactly as it came in
	Mesh::VertexArr vertices0;
	vertices0.CopyOf(mesh.vertices);
	probe.steppedEnergies.Resize(probe.steps.GetSize());
	FOREACH(i, probe.steps) {
		const Real t(probe.steps[i]);
		FOREACH(v, mesh.vertices)
			mesh.vertices[v] = vertices0[v] + dir[v]*t;
		probe.steppedEnergies[i] = refine.ScoreMesh(NULL);
	}
	mesh.vertices.CopyOf(vertices0);
	return true;
} // RefineMeshEnergyProbe
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
