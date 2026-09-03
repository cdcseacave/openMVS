/*
* SceneRefineCUDA.cpp
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

using namespace MVS;

#ifdef _USE_CUDA

#include "SceneRefineCUDA.inl"

// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define MESHCUDAOPT_USE_OPENMP
#endif

// uncomment to ensure edge size and improve vertex valence
// (should enable more stable flow)
#define MESHOPT_ENSUREEDGESIZE 1 // 0 - at all resolution


// S T R U C T S ///////////////////////////////////////////////////

// Convert MVS::Camera (double precision, OpenCV types) to MVS::CUDA::Camera (float precision, Eigen types)
static MVS::CUDA::Camera MakeCUDACamera(const Camera& camera, const Image8U::Size& size) {
	return MVS::CUDA::Camera(
		Eigen::Map<const SEACAVE::Matrix3x3::EMat>(camera.K.val).cast<float>(),
		Eigen::Map<const SEACAVE::Matrix3x3::EMat>(camera.R.val).cast<float>(),
		Eigen::Map<const SEACAVE::Point3::EVec>(camera.C.ptr()).cast<float>(),
		size.width, size.height);
}


// S T R U C T S ///////////////////////////////////////////////////

typedef Mesh::Vertex Vertex;
typedef Mesh::VIndex VIndex;
typedef Mesh::Face Face;
typedef Mesh::FIndex FIndex;

class MeshRefineCUDA {
public:
	typedef Mesh::FaceIdxArr CameraFaces;
	typedef CLISTDEF2(CameraFaces) CameraFacesArr;

	// store necessary data about a view
	struct View {
		Image32F imageHost; // store temporarily the image pixels
		Image32F imageGradHost[2]; // store temporarily the image x/y derivatives
		BitMatrix keepMaskHost; // store temporarily the per-view keep-mask (bit set = keep); empty == no mask == keep everything
		Image8U::Size size;
		SEACAVE::CUDA::ArrayRT32F image; // float like the CPU's View::image (was half-float; see readSurfFloat in the .cu)
		SEACAVE::CUDA::ArrayRT32F imageGrad[2]; // x/y derivatives (ComputeRefineImageGradient), sampled by the photometric kernel; kept in float like the CPU's View::imageGrad (half-float cannot hold the sub-6e-5 values a derivative stencil produces on flat regions)
		SEACAVE::CUDA::MemDevice depthMap;
		SEACAVE::CUDA::MemDevice faceMap;
		SEACAVE::CUDA::MemDevice baryMap;
		SEACAVE::CUDA::MemDevice keepMask; // one byte per pixel (non-zero = keep), uploaded from keepMaskHost only when non-empty; invalid (unallocated) == keep everything
	};
	typedef CLISTDEF2(View) ViewsArr;

	// GPU texture/surface objects per view
	struct ViewGPU {
		cudaTextureObject_t texObj = 0;   // LINEAR filter for bilinear sampling
		cudaTextureObject_t texGrad[2] = {0, 0}; // x/y derivative textures, same filtering
		cudaSurfaceObject_t surfObj = 0;  // surface for direct read/write
		void Release() {
			if (texObj) { cudaDestroyTextureObject(texObj); texObj = 0; }
			for (cudaTextureObject_t& tex: texGrad)
				if (tex) { cudaDestroyTextureObject(tex); tex = 0; }
			if (surfObj) { cudaDestroySurfaceObject(surfObj); surfObj = 0; }
		}
	};


public:
	MeshRefineCUDA(Scene& _scene, unsigned _nAlternatePair=true, float _weightRegularity=1.5f, float _ratioRigidityElasticity=0.8f, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640, unsigned nMaxViews=8);
	~MeshRefineCUDA();

	bool IsValid() const { return !pairs.IsEmpty(); }

	bool InitKernels();
	bool InitImages(float scale, float sigma=0);

	void ListVertexFacesPre();
	void ListVertexFacesPost();
	void ListCameraFaces();

	void ListFaceAreas(Mesh::AreaArr& maxAreas);
	void SubdivideMesh(uint32_t maxArea, float fDecimate=1.f, unsigned nCloseHoles=15, unsigned nEnsureEdgeSize=1);

	void ComputeNormalFaces();

	// downloads the raw per-vertex terms (BEFORE any combine) into the caller's host arrays,
	// each numVertices long, and leaves the reliability-weighted score in S (S < 0 means no
	// image pair contributed a single masked pixel -- see the sumR check below); the stepper
	// (MeshRefineStep, SceneRefineCommon.h) does the combining, not this function. Returns false
	// (having already VERBOSE'd why) if a GPU->host download failed: a non-sticky copy failure
	// is not caught by the caller's cuCtxSynchronize() check and would otherwise hand the
	// stepper finite-looking garbage.
	bool ScoreMesh(Point3f* photoGradOut, float* photoGradNormOut, float* footprintOut, Point3f* smoothGrad1Out, Point3f* smoothGrad2Out);

	void ProjectMesh(
		const CameraFaces& cameraFaces,
		const Camera& camera, const Image8U::Size& size, uint32_t idxImage);
	void ProcessPair(uint32_t idxImageA, uint32_t idxImageB);
	void ImageMeshWarp(
		const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
		uint32_t idxImageA, uint32_t idxImageB);
	void ComputeWindowStats(cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj, const Image8U::Size& size, bool exportMaps);
	void ComputePhotometricGradient(const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
		uint32_t idxImageA, uint32_t idxImageB, uint32_t numVertices, float RegularizationScale);
	void ComputeSmoothnessGradient(uint32_t numVertices);
	void CombineGradients(uint32_t numVertices);

public:
	const float weightRegularity; // a scalar regularity weight to balance between photo-consistency and regularization terms
	float ratioRigidityElasticity; // a scalar ratio used to compute the regularity gradient as a combination of rigidity and elasticity
	const unsigned nResolutionLevel; // how many times to scale down the images before mesh optimization
	const unsigned nMinResolution; // how many times to scale down the images before mesh optimization
	unsigned nAlternatePair; // using an image pair alternatively as reference image (0 - both, 1 - alternate, 2 - only left, 3 - only right)
	unsigned iteration; // current refinement iteration
	unsigned nScale; // current refinement scale (0-based, coarsest first), for the log and the RefineDebug export names

	// reliability-weighted photo-consistency score (S = sumRZ/sumR): invariant to scene scale,
	// contrast, resolution and pair count; set by ScoreMesh (see SceneRefineCommon.h), read by the
	// caller's stepper the same way the CPU reads MeshRefine::S
	float S;

	Scene& scene; // the mesh vertices and faces

	// constant the entire time
	ImageArr& images;
	ViewsArr views; // views' data
	PairIdxArr pairs; // image pairs used to refine the mesh

	CLISTDEF2(ViewGPU) viewGPU; // per-view texture/surface objects
	cudaSurfaceObject_t surfImageProjObj = 0; // surface for projected image (imageAB)

	SEACAVE::CUDA::MemDevice vertices;
	SEACAVE::CUDA::MemDevice faces;
	SEACAVE::CUDA::MemDevice faceNormals;
	SEACAVE::CUDA::MemDevice mask; // warp validity, the window-sum input
	SEACAVE::CUDA::MemDevice maskStats; // mask pruned by MinWindowCount and the rejection gates; what every consumer after ComputeWindowStats reads
	SEACAVE::CUDA::MemDevice projKey; // rasterizer scratch, one (depth,face) 64-bit key per pixel of the largest view
	size_t projKeyPixels = 0; // pixels projKey was allocated for (ProjectMesh asserts every view fits)
	SEACAVE::CUDA::ArrayRT32F imageAB; // warped image B in A, float like the CPU's imageAB
	SEACAVE::CUDA::MemDevice imageZNCC; // parity diagnostic only, written when the RefineDebug pair matches
	SEACAVE::CUDA::MemDevice imageConf; // reliability weight; allocated for the parity export only
	SEACAVE::CUDA::MemDevice imageDZNCC;
	SEACAVE::CUDA::MemDevice photoGrad;
	SEACAVE::CUDA::MemDevice photoGradNorm;
	SEACAVE::CUDA::MemDevice footprint; // per-vertex scene units per pixel; 0 exactly where photoGradNorm == 0 (SceneRefineCommon.h)
	// per-FACE private accumulators of one pair-direction's photometric term: the atomic-free
	// replacement for the old per-pixel scatter (see kernelAccumulateFacePhoto). Sized for the
	// whole mesh and rewritten in full by every accumulation launch, so they never need clearing.
	SEACAVE::CUDA::MemDevice faceAcc; // 3 floats per face: Sum g_p*b_c per corner
	SEACAVE::CUDA::MemDevice facePixels; // 1 float per face: contributing pixel count
	SEACAVE::CUDA::MemDevice faceFoot; // 1 float per face: min footprint over the face's pixels
	SEACAVE::CUDA::MemDevice sumR; // device scalar: Sum r, accumulated by kernelComputeWindowStats/kernelReduceBlockSums over one ScoreMesh() call
	SEACAVE::CUDA::MemDevice sumRZ; // device scalar: Sum r*(1-ZNCC), same accumulation window as sumR
	SEACAVE::CUDA::MemDevice statsBlockSums; // 2 floats per window-stats block of the largest view: the partials sumR/sumRZ are folded from, in block order
	SEACAVE::CUDA::MemDevice debugSG; // WP2 parity diagnostic only: per-pixel photometric scalar, allocated when the RefineDebug pair matches
	SEACAVE::CUDA::MemDevice vertexVerticesCont;
	SEACAVE::CUDA::MemDevice vertexVerticesSizes;
	SEACAVE::CUDA::MemDevice vertexVerticesPointers;
	// the same flattening for the INCIDENT FACES of each vertex (Mesh::ListIncidentFaces order):
	// the gather half of the photometric accumulation walks it instead of the pixels scattering
	SEACAVE::CUDA::MemDevice vertexFacesCont;
	SEACAVE::CUDA::MemDevice vertexFacesSizes;
	SEACAVE::CUDA::MemDevice vertexFacesPointers;
	SEACAVE::CUDA::MemDevice vertBoundary; // per-vertex 0/1 boundary flag (shared valence/boundary split, see ListVertexFacesPost())
	SEACAVE::CUDA::MemDevice smoothGrad1;
	SEACAVE::CUDA::MemDevice smoothGrad2;
};

MeshRefineCUDA::MeshRefineCUDA(Scene& _scene, unsigned _nAlternatePair, float _weightRegularity, float _ratioRigidityElasticity, unsigned _nResolutionLevel, unsigned _nMinResolution, unsigned nMaxViews)
	:
	weightRegularity(_weightRegularity),
	ratioRigidityElasticity(_ratioRigidityElasticity),
	nResolutionLevel(_nResolutionLevel),
	nMinResolution(_nMinResolution),
	nAlternatePair(_nAlternatePair),
	scene(_scene),
	images(_scene.images)
{
	if (!InitKernels())
		return;
	// keep only best neighbor views for each image
	std::unordered_set<uint64_t> mapPairs;
	mapPairs.reserve(images.GetSize()*nMaxViews);
	FOREACH(idxImage, images) {
		ViewScoreArr neighbors;
		if (!SelectRefineNeighbors(scene, idxImage, nMaxViews, neighbors))
			continue;
		for (const ViewScore& neighbor: neighbors) {
			ASSERT(images[neighbor.ID].IsValid());
			mapPairs.insert(MakePairIdx((uint32_t)idxImage, neighbor.ID));
		}
	}
	pairs.Reserve(mapPairs.size());
	for (uint64_t pair: mapPairs)
		pairs.AddConstruct(pair);
}
MeshRefineCUDA::~MeshRefineCUDA()
{
	for (auto& v : viewGPU)
		v.Release();
	if (surfImageProjObj) cudaDestroySurfaceObject(surfImageProjObj);
	scene.mesh.ReleaseExtra();
}

bool MeshRefineCUDA::InitKernels()
{
	// initialize CUDA device if needed
	if (!SEACAVE::CUDA::isEnabled() && SEACAVE::CUDA::initDevices(SEACAVE::CUDA::desiredDeviceIDs) != CUDA_SUCCESS)
		return false;
	return true;
}

// load and initialize all images at the given scale
// and compute the gradient for each input image
// optional: blur them using the given sigma
bool MeshRefineCUDA::InitImages(float scale, float sigma)
{
	views.Resize(images.GetSize());
	#ifdef MESHCUDAOPT_USE_OPENMP
	bool bAbort(false);
	#pragma omp parallel for
	for (int_t ID=0; ID<(int_t)images.GetSize(); ++ID) {
		#pragma omp flush (bAbort)
		if (bAbort)
			continue;
		const uint32_t idxImage((uint32_t)ID);
	#else
	FOREACH(idxImage, images) {
	#endif
		Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		// load and init image
		View& view = views[idxImage];
		Image32F& img = view.imageHost;
		if (!PrepareRefineImage(imageData, scene.platforms, nResolutionLevel, nMinResolution, scale, sigma, img)) {
			#ifdef MESHCUDAOPT_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return false;
			#endif
		}
		// mode 3 samples the bilinear interpolant of the image directly (computePhotoPixel's
		// bilinearGradient) and never reads this precomputed stencil, so computing (and, below,
		// uploading) it would be wasted work
		if (OPTREFINE::nImageGradient != 3)
			ComputeRefineImageGradient(img, view.imageGradHost[0], view.imageGradHost[1]);
		// per-view keep-mask at this scale's working size (the same estimator on both backends)
		PrepareRefineImageMask(imageData, img.size(), view.keepMaskHost);
	}
	#ifdef MESHCUDAOPT_USE_OPENMP
	if (bAbort)
		return false;
	#endif
	// init GPU memory
	Image8U::Size maxSize(0,0);
	// destroy old texture/surface objects before recreating
	for (auto& v : viewGPU)
		v.Release();
	if (surfImageProjObj) { cudaDestroySurfaceObject(surfImageProjObj); surfImageProjObj = 0; }
	viewGPU.Resize(views.GetSize());
	// the texture and surface objects are the only runtime-API calls here; a failure leaves a
	// null handle the kernels would sample silently, so report it like the driver-API ones
	const auto reportRtError = [](cudaError_t err, const char* what) {
		if (err != cudaSuccess)
			VERBOSE("error: %s: %s", what, cudaGetErrorString(err));
		return err == cudaSuccess;
	};
	// texture object with bilinear filtering over a 2D array (element-type reads: 16F and 32F arrays both fetch as float)
	const auto createTexture = [&reportRtError](const auto& array, cudaTextureObject_t& tex) {
		cudaResourceDesc resDesc = {};
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = (cudaArray_t)(CUarray)array;
		cudaTextureDesc texDesc = {};
		texDesc.filterMode = cudaFilterModeLinear;
		texDesc.addressMode[0] = cudaAddressModeClamp;
		texDesc.addressMode[1] = cudaAddressModeClamp;
		texDesc.readMode = cudaReadModeElementType;
		return reportRtError(cudaCreateTextureObject(&tex, &resDesc, &texDesc, nullptr), "cudaCreateTextureObject");
	};
	FOREACH(idxImage, views) {
		View& view = views[idxImage];
		if (view.imageHost.empty())
			continue;
		Image8U::Size& size(view.size);
		size = view.imageHost.size();
		reportCudaError(view.image.Reset(size, CUDA_ARRAY3D_SURFACE_LDST));
		reportCudaError(view.image.SetData(view.imageHost));
		view.imageHost.release();
		// no gradient stencil texture in mode 3 -- see the comment above where it is computed
		if (OPTREFINE::nImageGradient != 3) {
			for (int i=0; i<2; ++i) {
				ASSERT(view.imageGradHost[i].size() == size);
				reportCudaError(view.imageGrad[i].Reset(size, CUDA_ARRAY3D_SURFACE_LDST));
				reportCudaError(view.imageGrad[i].SetData(view.imageGradHost[i]));
				view.imageGradHost[i].release();
				if (!createTexture(view.imageGrad[i], viewGPU[idxImage].texGrad[i]))
					return false;
			}
		}
		const size_t area((size_t)size.area());
		reportCudaError(view.depthMap.Reset(sizeof(float)*area));
		reportCudaError(view.faceMap.Reset(sizeof(FIndex)*area));
		reportCudaError(view.baryMap.Reset(sizeof(hfloat)*3*area));
		if (view.keepMaskHost.empty()) {
			view.keepMask.Release();
		} else {
			// dense byte mask (non-zero = keep) the kernel can index directly; BitMatrix::copyTo()
			// already does this exact bit-to-byte expansion (255/0)
			Image8U keepBytes;
			view.keepMaskHost.copyTo(keepBytes);
			reportCudaError(view.keepMask.Reset(keepBytes));
			view.keepMaskHost.release();
		}
		if (maxSize.width < size.width)
			maxSize.width = size.width;
		if (maxSize.height < size.height)
			maxSize.height = size.height;
		// create texture and surface objects for this view
		cudaResourceDesc resDesc = {};
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = (cudaArray_t)(CUarray)view.image;
		if (!reportRtError(cudaCreateSurfaceObject(&viewGPU[idxImage].surfObj, &resDesc), "cudaCreateSurfaceObject") ||
			!createTexture(view.image, viewGPU[idxImage].texObj))
			return false;
	}
	const size_t area(maxSize.area());
	reportCudaError(mask.Reset(sizeof(uint8_t)*area));
	reportCudaError(maskStats.Reset(sizeof(uint8_t)*area));
	reportCudaError(projKey.Reset(sizeof(uint64_t)*area));
	projKeyPixels = area;
	reportCudaError(imageAB.Reset(maxSize, CUDA_ARRAY3D_SURFACE_LDST));
	reportCudaError(imageDZNCC.Reset(sizeof(float)*area));
	if (!RefineDebug::Dir().empty()) {
		reportCudaError(imageZNCC.Reset(sizeof(float)*area));
		reportCudaError(imageConf.Reset(sizeof(float)*area));
	}
	reportCudaError(sumR.Reset(sizeof(float)));
	reportCudaError(sumRZ.Reset(sizeof(float)));
	// one slot per 16x16 block of the window-statistics grid; the largest view's width and height
	// need not come from the same view, so this is an upper bound rather than an exact count
	{
		const size_t maxBlocks(((size_t)maxSize.width + 15)/16 * (((size_t)maxSize.height + 15)/16));
		reportCudaError(statsBlockSums.Reset(sizeof(float)*2*maxBlocks));
	}
	// create surface object for projected image
	{
		cudaResourceDesc resDesc = {};
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = (cudaArray_t)(CUarray)imageAB;
		if (!reportRtError(cudaCreateSurfaceObject(&surfImageProjObj, &resDesc), "cudaCreateSurfaceObject"))
			return false;
	}
	iteration = 0;
	nScale = 0;
	return true;
}

// extract array of triangles incident to each vertex
// and check each vertex if it is at the boundary or not
void MeshRefineCUDA::ListVertexFacesPre()
{
	scene.mesh.EmptyExtra();
	scene.mesh.ListIncidentFaces();
	reportCudaError(faces.Reset(scene.mesh.faces));
}
void MeshRefineCUDA::ListVertexFacesPost()
{
	scene.mesh.ListIncidentVertices();
	scene.mesh.ListBoundaryVertices();
	ASSERT(!scene.mesh.vertices.IsEmpty() && scene.mesh.vertices.GetSize() == scene.mesh.vertexVertices.GetSize());
	// list adjacent vertices for each vertex, uploading the TRUE valence for every vertex
	// (boundary or not) plus a separate per-vertex boundary flag: kernelComputeSmoothnessGradient
	// zeroes a boundary vertex's OWN gradient using vertBoundary[], exactly like CPU
	// MeshRefine::ComputeSmoothnessGradient1/2, but every OTHER vertex's valence-weighted sum
	// still divides by a boundary neighbour's true valence
	// the incident FACES of each vertex are flattened the same way, in the order
	// Mesh::ListIncidentFaces produced (ListVertexFacesPre(), above): kernelGatherVertexPhoto
	// walks that fixed order to sum one pair-direction's photometric term for the vertex, which
	// is what makes the sum reproducible without atomics
	const size_t numVertices(scene.mesh.vertices.GetSize());
	ASSERT(scene.mesh.vertexFaces.GetSize() == numVertices);
	Unsigned32Arr _vertexVerticesCont(0, numVertices*6);
	Unsigned32Arr _vertexVerticesSizes(0, numVertices);
	Unsigned32Arr _vertexVerticesPointers(0, numVertices);
	Unsigned8Arr _vertexBoundary(0, numVertices);
	Unsigned32Arr _vertexFacesCont(0, scene.mesh.faces.GetSize()*3);
	Unsigned32Arr _vertexFacesSizes(0, numVertices);
	Unsigned32Arr _vertexFacesPointers(0, numVertices);
	uint32_t lastPosition(0), lastFacePosition(0);
	FOREACH(idxV, scene.mesh.vertices) {
		const Mesh::VertexIdxArr& verts = scene.mesh.vertexVertices[idxV];
		ASSERT(!verts.IsEmpty()); // true valence must be > 0 for every vertex
		_vertexVerticesCont.Join(verts.GetData(), verts.GetSize());
		_vertexVerticesSizes.Insert(verts.GetSize());
		_vertexVerticesPointers.Insert(lastPosition); lastPosition += verts.GetSize();
		_vertexBoundary.Insert(scene.mesh.vertexBoundary[idxV] ? 1 : 0);
		const Mesh::FaceIdxArr& vfaces = scene.mesh.vertexFaces[idxV];
		ASSERT(!vfaces.IsEmpty()); // a vertex with no incident face has no place in the mesh
		_vertexFacesCont.Join(vfaces.GetData(), vfaces.GetSize());
		_vertexFacesSizes.Insert(vfaces.GetSize());
		_vertexFacesPointers.Insert(lastFacePosition); lastFacePosition += vfaces.GetSize();
	}
	reportCudaError(vertexVerticesCont.Reset(_vertexVerticesCont));
	reportCudaError(vertexVerticesSizes.Reset(_vertexVerticesSizes));
	reportCudaError(vertexVerticesPointers.Reset(_vertexVerticesPointers));
	reportCudaError(vertBoundary.Reset(_vertexBoundary));
	reportCudaError(vertexFacesCont.Reset(_vertexFacesCont));
	reportCudaError(vertexFacesSizes.Reset(_vertexFacesSizes));
	reportCudaError(vertexFacesPointers.Reset(_vertexFacesPointers));
	// init memory
	reportCudaError(photoGrad.Reset(sizeof(Point3f)*numVertices));
	reportCudaError(photoGradNorm.Reset(sizeof(float)*numVertices));
	reportCudaError(footprint.Reset(sizeof(float)*numVertices));
	reportCudaError(smoothGrad1.Reset(sizeof(Point3f)*numVertices));
	reportCudaError(smoothGrad2.Reset(sizeof(Point3f)*numVertices));
	// the per-face accumulators the photometric pass reduces into (20 bytes per face); this mesh
	// is final for the scale, so they are sized once here
	const size_t numFaces(scene.mesh.faces.GetSize());
	reportCudaError(faceAcc.Reset(sizeof(float)*3*numFaces));
	reportCudaError(facePixels.Reset(sizeof(float)*numFaces));
	reportCudaError(faceFoot.Reset(sizeof(float)*numFaces));
}

// extract array of faces viewed by each image
void MeshRefineCUDA::ListCameraFaces()
{
	// extract array of faces viewed by each camera
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
	reportCudaError(vertices.Reset(scene.mesh.vertices));
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (imageData.IsValid())
			ProjectMesh(arrCameraFaces[idxImage], imageData.camera, views[idxImage].size, idxImage);
	}
}

// compute for each face the projection area as the maximum area in both images of a pair
// (make sure ListCameraFaces() was called before)
void MeshRefineCUDA::ListFaceAreas(Mesh::AreaArr& maxAreas)
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
		areas.Resize(scene.mesh.faces.GetSize());
		areas.Memset(0);
		// get faceMap from the GPU memory
		TImage<FIndex> faceMap(imageData.height, imageData.width);
		views[idxImage].faceMap.GetData(faceMap);
		// compute area covered by all vertices (incident faces) viewed by this image
		for (int j=0; j<faceMap.rows; ++j) {
			for (int i=0; i<faceMap.cols; ++i) {
				const FIndex idxFace(faceMap(j,i));
				if (idxFace == NO_ID)
					continue;
				++areas[idxFace];
			}
		}
	}
	// for each pair, mark the faces that have big projection areas in both images
	maxAreas.Resize(scene.mesh.faces.GetSize());
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
void MeshRefineCUDA::SubdivideMesh(uint32_t maxArea, float fDecimate, unsigned nCloseHoles, unsigned nEnsureEdgeSize)
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

			const float maxAreaf((float)(maxArea > 0 ? maxArea : 64));
			const float medianArea(6.f*(float)Mesh::AreaArr(maxAreas).GetMedian());
			if (medianArea < maxAreaf) {
				maxAreas.Empty();

				// decimate to the auto detected resolution
				cleanMesh(MAXF(0.1f, medianArea/maxAreaf));

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
	const size_t numVertsOld(scene.mesh.vertices.GetSize());
	const size_t numFacesOld(scene.mesh.faces.GetSize());
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

	DEBUG_EXTRA("Mesh subdivided: %u/%u -> %u/%u vertices/faces", numVertsOld, numFacesOld, scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize());

	#if TD_VERBOSE != TD_VERBOSE_OFF
	if (VERBOSITY_LEVEL > 3)
		scene.mesh.Save(MAKE_PATH("MeshSubdivided.ply"));
	#endif
}


// compute face normals
void MeshRefineCUDA::ComputeNormalFaces()
{
	const FIndex numFaces(scene.mesh.faces.GetSize());
	reportCudaError(faceNormals.Reset(sizeof(Point3f)*numFaces));
	MVS::CUDA::LaunchComputeFaceNormal(
		(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
		(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
		(MVS::CUDA::Point3*)(CUdeviceptr)faceNormals,
		numFaces);
}


// score mesh using photo-consistency
// and download the raw per-vertex terms (photoGrad/photoGradNorm/footprint/smoothGrad1/smoothGrad2)
// the caller's stepper (MeshRefineStep, SceneRefineCommon.h) combines into a step
bool MeshRefineCUDA::ScoreMesh(Point3f* photoGradOut, float* photoGradNormOut, float* footprintOut, Point3f* smoothGrad1Out, Point3f* smoothGrad2Out)
{
	// every GPU->host download this function's callers depend on goes through this: reportCudaError
	// alone only logs and keeps going, so a non-sticky copy failure (unlike a poisoned context,
	// which cuCtxSynchronize() catches) would otherwise hand the stepper finite-looking garbage
	const auto CheckDownload = [this](CUresult ret, const char* name) -> bool {
		if (ret != CUDA_SUCCESS) {
			VERBOSE("error: failed downloading %s from the GPU at scale %u, iteration %u", name, nScale, iteration);
			return false;
		}
		return true;
	};

	// extract array of faces viewed by each camera
	ListCameraFaces();

	// compute face normals
	ComputeNormalFaces();

	// init memory
	const VIndex numVertices(scene.mesh.vertices.GetSize());
	reportCudaError(cuMemsetD32(photoGrad, 0, numVertices*3));
	reportCudaError(cuMemsetD32(photoGradNorm, 0, numVertices));
	reportCudaError(cuMemsetD32(footprint, 0x7F7FFFFF, numVertices)); // FLT_MAX sentinel, resolved below
	reportCudaError(cuMemsetD32(sumR, 0, 1));
	reportCudaError(cuMemsetD32(sumRZ, 0, 1));

	// for each pair of images, compute a photo-consistency score
	// between the reference image and the pixels of the second image
	// projected in the reference image through the mesh surface
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		switch (nAlternatePair) {
		case 1: {
			const PairIdx pair(iteration%2 ? PairIdx(pPair->j,pPair->i) : PairIdx(pPair->i,pPair->j));
			ProcessPair(pair.i, pair.j);
			break; }
		case 2: {
			ProcessPair(pPair->i, pPair->j);
			break; }
		case 3: {
			ProcessPair(pPair->j, pPair->i);
			break; }
		default:
			for (int ip=0; ip<2; ++ip) {
				const PairIdx pair(ip ? PairIdx(pPair->j,pPair->i) : PairIdx(pPair->i,pPair->j));
				ProcessPair(pair.i, pair.j);
			}
		}
	}

	// resolve the footprint sentinel now that photoGradNorm holds its final per-vertex count
	// (contract: footprint[v] > 0 exactly where photoGradNorm[v] > 0), exactly like the CPU's
	// ScoreMesh resolves it at the same point
	MVS::CUDA::LaunchFinalizePhotoGrad(
		(const float*)(CUdeviceptr)photoGradNorm,
		(float*)(CUdeviceptr)footprint,
		numVertices);

	// S = sumRZ/sumR, the reliability-weighted mean of (1-ZNCC). sumR == 0 means no pair-direction
	// contributed a single masked pixel -- broken outside-world input (no pair overlap, bad
	// poses), not an internal invariant, so it gets the runtime-validation treatment instead of
	// an ASSERT alone (which would compile out in Release and feed the stepper NaN): S is left
	// negative and the caller fails the refinement loudly, exactly like MeshRefine::ScoreMesh
	float hSumR, hSumRZ;
	if (!CheckDownload(reportCudaError(sumR.GetData(&hSumR, sizeof(float))), "sumR") ||
		!CheckDownload(reportCudaError(sumRZ.GetData(&hSumRZ, sizeof(float))), "sumRZ"))
		return false;
	if (hSumR > 0) {
		S = hSumRZ/hSumR;
		ASSERT(S >= 0 && S <= 2);
	} else {
		S = -1.f;
	}

	// loop through all vertices and compute the smoothing score
	ComputeSmoothnessGradient(numVertices);

	// ALWAYS download the raw per-vertex terms before any combine: CombineGradients() overwrites
	// photoGrad in place, and on the hot path (no OMVS_REFINE_DEBUG_DIR) it is never called --
	// the caller's stepper does the combining instead
	if (!CheckDownload(reportCudaError(photoGrad.GetData(photoGradOut, sizeof(Point3f)*numVertices)), "photoGrad") ||
		!CheckDownload(reportCudaError(photoGradNorm.GetData(photoGradNormOut, sizeof(float)*numVertices)), "photoGradNorm") ||
		!CheckDownload(reportCudaError(footprint.GetData(footprintOut, sizeof(float)*numVertices)), "footprint") ||
		!CheckDownload(reportCudaError(smoothGrad1.GetData(smoothGrad1Out, sizeof(Point3f)*numVertices)), "smoothGrad1") ||
		!CheckDownload(reportCudaError(smoothGrad2.GetData(smoothGrad2Out, sizeof(Point3f)*numVertices)), "smoothGrad2"))
		return false;

	// WP2 parity diagnostic: combine and export on top of the raw terms just downloaded above;
	// no-op unless OMVS_REFINE_DEBUG_DIR is set
	if (!RefineDebug::Dir().empty()) {
		Point3fArr pos(numVertices), combined(numVertices);
		cList<uint8_t,uint8_t,0> boundary(numVertices);
		reportCudaError(vertices.GetData(pos));
		reportCudaError(vertBoundary.GetData(boundary));
		// set the final gradient as the combination of photometric and smoothness gradients
		CombineGradients(numVertices);
		reportCudaError(photoGrad.GetData(combined));
		RefineDebug::ExportGradients(nScale, iteration, numVertices,
			pos.Begin(), combined.Begin(), photoGradOut, photoGradNormOut,
			smoothGrad1Out, smoothGrad2Out, boundary.Begin());
	}
	return true;
}


// project mesh to the given camera plane
void MeshRefineCUDA::ProjectMesh(
	const CameraFaces& cameraFaces,
	const Camera& camera, const Image8U::Size& size, uint32_t idxImage)
{
	View& view = views[idxImage];
	ASSERT(projKey.IsValid() && (size_t)size.area() <= projKeyPixels);
	// pass 1 needs every pixel key at "no face yet" = ~0ull (larger than any real (depth,face) key)
	reportCudaError(cuMemsetD32(projKey, 0xFFFFFFFFu, 2*(size_t)size.area()));
	// fetch only the faces viewed by this camera
	Mesh::FaceIdxArr faceIDsView(0, (FIndex)cameraFaces.size());
	for (auto idxFace : cameraFaces)
		faceIDsView.Insert(idxFace);
	// project mesh: pass 1 elects per pixel the nearest face (first in cameraFaces order on a
	// depth tie, the CPU's rule), pass 2 lets the winner write its payload, then the uncovered
	// pixels are cleared (both maps are preset so no pixel can carry this view's previous
	// iteration forward: faceMap to NO_ID, which the Debug check in the last kernel would
	// otherwise let a stale id satisfy, and depthMap to 0, so that a pixel missing its payload
	// reads as uncovered downstream instead of pairing a stale depth with a NO_ID face)
	SEACAVE::CUDA::MemDevice devFaceIDs(faceIDsView);
	const MVS::CUDA::Camera cudaCamera(MakeCUDACamera(camera, size));
	reportCudaError(cuMemsetD32(view.faceMap, NO_ID, size.area()));
	reportCudaError(cuMemsetD32(view.depthMap, 0, size.area()));
	for (int pass=0; pass<2; ++pass)
		MVS::CUDA::LaunchProjectMesh(
			(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
			(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
			(const uint32_t*)(CUdeviceptr)devFaceIDs,
			(unsigned long long*)(CUdeviceptr)projKey,
			(float*)(CUdeviceptr)view.depthMap,
			(uint32_t*)(CUdeviceptr)view.faceMap,
			(uint16_t*)(CUdeviceptr)view.baryMap,
			cudaCamera,
			faceIDsView.GetSize(),
			pass == 1);
	#ifdef _DEBUG
	// every covered pixel must hold the payload of exactly the thread that won its key
	MVS::CUDA::LaunchCheckProjection(
		(const uint32_t*)(CUdeviceptr)devFaceIDs,
		(const unsigned long long*)(CUdeviceptr)projKey,
		(const float*)(CUdeviceptr)view.depthMap,
		(const uint32_t*)(CUdeviceptr)view.faceMap,
		size.width, size.height);
	#endif
}

void MeshRefineCUDA::ProcessPair(uint32_t idxImageA, uint32_t idxImageB)
{
	// fetch view A data
	const Image& imageDataA = images[idxImageA];
	ASSERT(imageDataA.IsValid());
	const Camera& cameraA = imageDataA.camera;
	const Image8U::Size& sizeA(views[idxImageA].size);
	// fetch view B data
	const Image& imageDataB = images[idxImageB];
	ASSERT(imageDataB.IsValid());
	const Camera& cameraB = imageDataB.camera;
	// warp imageB to imageA using the mesh
	ImageMeshWarp(cameraA, cameraB, sizeA, idxImageA, idxImageB);
	// masked window statistics, rejection gates, ZNCC and its derivative; prunes mask into maskStats
	uint32_t dbgImageA, dbgImageB;
	const bool dbgPair(!RefineDebug::Dir().empty() && RefineDebug::Pair(dbgImageA, dbgImageB) &&
		dbgImageA == idxImageA && dbgImageB == idxImageB);
	ComputeWindowStats(viewGPU[idxImageA].surfObj, surfImageProjObj, sizeA, dbgPair);

	// WP2 parity diagnostic: download this pair's maps from the persistent
	// device buffers above; no-op unless OMVS_REFINE_DEBUG_DIR/_PAIR are both
	// set and match this exact (A,B) direction
	if (dbgPair) {
		const int width(sizeA.width), height(sizeA.height);
		Image32F imgA(sizeA), imgAB(sizeA);
		Image8U _mask(sizeA);
		Image32F _zncc(sizeA), _dzncc(sizeA), _conf(sizeA);
		reportCudaError(views[idxImageA].image.GetData(imgA));
		reportCudaError(imageAB.GetData(imgAB));
		reportCudaError(maskStats.GetData(_mask));
		reportCudaError(imageZNCC.GetData(_zncc));
		reportCudaError(imageConf.GetData(_conf));
		reportCudaError(imageDZNCC.GetData(_dzncc));
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "imageA", imgA.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "imageAB", imgAB.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "zncc", _zncc.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "dzncc", _dzncc.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "conf", _conf.getData(), width, height);
		RefineDebug::ExportPairMask(nScale, iteration, idxImageA, idxImageB, _mask.getData(), width, height);
	}

	const float RegularizationScale((float)((REAL)(imageDataA.avgDepth*imageDataB.avgDepth)/(cameraA.GetFocalLength()*cameraB.GetFocalLength())));
	ComputePhotometricGradient(cameraA, cameraB, sizeA, idxImageA, idxImageB, scene.mesh.vertices.GetSize(), RegularizationScale);
}

// project image from view B to view A through the mesh;
// the projected image is stored in imageA
void MeshRefineCUDA::ImageMeshWarp(
	const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
	uint32_t idxImageA, uint32_t idxImageB)
{
	// project image
	MVS::CUDA::LaunchImageMeshWarp(
		(const float*)(CUdeviceptr)views[idxImageA].depthMap,
		(const float*)(CUdeviceptr)views[idxImageB].depthMap,
		views[idxImageA].keepMask.IsValid() ? (const uint8_t*)(CUdeviceptr)views[idxImageA].keepMask : NULL,
		views[idxImageB].keepMask.IsValid() ? (const uint8_t*)(CUdeviceptr)views[idxImageB].keepMask : NULL,
		(uint8_t*)(CUdeviceptr)mask,
		MakeCUDACamera(cameraA, size),
		MakeCUDACamera(cameraB, views[idxImageB].size),
		viewGPU[idxImageB].texObj,
		surfImageProjObj);
}

// masked window statistics, rejection gates, ZNCC and its per-pixel derivative, in one kernel
void MeshRefineCUDA::ComputeWindowStats(cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj, const Image8U::Size& size, bool exportMaps)
{
	MVS::CUDA::LaunchComputeWindowStats(
		(const uint8_t*)(CUdeviceptr)mask,
		(uint8_t*)(CUdeviceptr)maskStats,
		(float*)(CUdeviceptr)imageDZNCC,
		exportMaps ? (float*)(CUdeviceptr)imageZNCC : NULL,
		imageConf.IsValid() ? (float*)(CUdeviceptr)imageConf : NULL,
		surfImageA, surfImageProj,
		(float*)(CUdeviceptr)sumR,
		(float*)(CUdeviceptr)sumRZ,
		(float*)(CUdeviceptr)statsBlockSums,
		OPTREFINE::fGateMeanDiff, OPTREFINE::fGateVarRatio,
		size.width, size.height);
}

// compute the photometric gradient for all vertices seen by an image pair
void MeshRefineCUDA::ComputePhotometricGradient(const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
	uint32_t idxImageA, uint32_t idxImageB, uint32_t numVertices, float RegularizationScale)
{
	// WP2 parity diagnostic: per-pixel photometric scalar (the CPU's sg) + face id, same
	// gating as the pair-map block in ProcessPair(); no-op unless the debug pair matches
	uint32_t dbgImageA, dbgImageB;
	const bool dbgPair(!RefineDebug::Dir().empty() && RefineDebug::Pair(dbgImageA, dbgImageB) &&
		dbgImageA == idxImageA && dbgImageB == idxImageB);
	const size_t area((size_t)size.area());
	if (dbgPair) {
		reportCudaError(debugSG.Reset(sizeof(float)*area));
		reportCudaError(cuMemsetD32(debugSG, 0, area));
	}
	const MVS::CUDA::Camera cudaCamA(MakeCUDACamera(cameraA, size));
	const MVS::CUDA::Camera cudaCamB(MakeCUDACamera(cameraB, views[idxImageB].size));
	const FIndex numFaces(scene.mesh.faces.GetSize());
	// mode 3 samples the bilinear interpolant of image B directly and needs no precomputed
	// gradient stencil texture (InitImages never uploads one in that mode)
	const bool bBilinearGrad(OPTREFINE::nImageGradient == 3);
	// the atomic-free accumulation: reduce every face's pixels into its own slots, then let each
	// vertex gather its incident faces in a fixed order, so no float sum depends on the schedule
	// and two identical runs produce identical meshes (see kernelAccumulateFacePhoto)
	MVS::CUDA::LaunchAccumulateFacePhoto(
		(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
		(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
		(const MVS::CUDA::Point3*)(CUdeviceptr)faceNormals,
		(const float*)(CUdeviceptr)views[idxImageA].depthMap,
		(const uint32_t*)(CUdeviceptr)views[idxImageA].faceMap,
		(const uint16_t*)(CUdeviceptr)views[idxImageA].baryMap,
		(const float*)(CUdeviceptr)imageDZNCC,
		(const uint8_t*)(CUdeviceptr)maskStats,
		(float*)(CUdeviceptr)faceAcc,
		(float*)(CUdeviceptr)facePixels,
		(float*)(CUdeviceptr)faceFoot,
		dbgPair ? (float*)(CUdeviceptr)debugSG : NULL,
		cudaCamA, cudaCamB,
		viewGPU[idxImageB].texObj, viewGPU[idxImageB].texGrad[0], viewGPU[idxImageB].texGrad[1],
		bBilinearGrad,
		RegularizationScale,
		numFaces);
	MVS::CUDA::LaunchGatherVertexPhoto(
		(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
		(const MVS::CUDA::Point3*)(CUdeviceptr)faceNormals,
		(const uint32_t*)(CUdeviceptr)vertexFacesCont,
		(const uint32_t*)(CUdeviceptr)vertexFacesSizes,
		(const uint32_t*)(CUdeviceptr)vertexFacesPointers,
		(const float*)(CUdeviceptr)faceAcc,
		(const float*)(CUdeviceptr)facePixels,
		(const float*)(CUdeviceptr)faceFoot,
		(MVS::CUDA::Point3*)(CUdeviceptr)photoGrad,
		(float*)(CUdeviceptr)photoGradNorm,
		(float*)(CUdeviceptr)footprint,
		numVertices);
	if (dbgPair) {
		// the raw face map of A (-1 where nothing was rasterised; NOT masked, so rasterisation
		// and warp differences can be told apart), same as the CPU export
		Image32F sg(size), faceF(size);
		TImage<uint32_t> faceMap(size);
		reportCudaError(debugSG.GetData(sg.getData(), sizeof(float)*area));
		reportCudaError(views[idxImageA].faceMap.GetData(faceMap.getData(), sizeof(uint32_t)*area));
		for (int r=0; r<size.height; ++r)
			for (int c=0; c<size.width; ++c)
				faceF(r,c) = faceMap(r,c) == NO_ID ? -1.f : (float)faceMap(r,c);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "sg", sg.getData(), size.width, size.height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "face", faceF.getData(), size.width, size.height);
		debugSG.Release();
	}
	// this pair-direction's photoGradNorm bookkeeping is done by the gather above; the footprint
	// sentinel is resolved separately, once, after every pair-direction of this ScoreMesh() has
	// run (see kernelFinalizePhotoGrad)
}

void MeshRefineCUDA::ComputeSmoothnessGradient(uint32_t numVertices)
{
	// compute smoothness gradient for all vertices
	MVS::CUDA::LaunchComputeSmoothnessGradient(
		(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
		(const uint32_t*)(CUdeviceptr)vertexVerticesCont,
		(const uint32_t*)(CUdeviceptr)vertexVerticesSizes,
		(const uint32_t*)(CUdeviceptr)vertexVerticesPointers,
		(const uint8_t*)(CUdeviceptr)vertBoundary,
		(MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad1,
		numVertices, uint8_t(0));
	MVS::CUDA::LaunchComputeSmoothnessGradient(
		(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad1,
		(const uint32_t*)(CUdeviceptr)vertexVerticesCont,
		(const uint32_t*)(CUdeviceptr)vertexVerticesSizes,
		(const uint32_t*)(CUdeviceptr)vertexVerticesPointers,
		(const uint8_t*)(CUdeviceptr)vertBoundary,
		(MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad2,
		numVertices, uint8_t(1));
}

void MeshRefineCUDA::CombineGradients(uint32_t numVertices)
{
	// divide by c_v, the number of pair-directions that saw each vertex
	const float* photoGradNormPtr((const float*)(CUdeviceptr)photoGradNorm);
	// compute smoothness gradient for all vertices
	if (ratioRigidityElasticity >= 1.f) {
		MVS::CUDA::LaunchCombineGradients(
			(MVS::CUDA::Point3*)(CUdeviceptr)photoGrad,
			photoGradNormPtr,
			(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad2,
			numVertices, weightRegularity);
	} else {
		// compute smoothing gradient as a combination of level 1 and 2 of the Laplacian operator;
		// (see page 105 of "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004)
		const float rigidity((1.f-ratioRigidityElasticity)*weightRegularity);
		const float elasticity(ratioRigidityElasticity*weightRegularity);
		MVS::CUDA::LaunchCombineAllGradients(
			(MVS::CUDA::Point3*)(CUdeviceptr)photoGrad,
			photoGradNormPtr,
			(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad1,
			(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad2,
			numVertices, rigidity, elasticity);
	}
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

// optimize mesh using photo-consistency
bool Scene::RefineMeshCUDA(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews,
						   float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nMaxFaceArea,
						   unsigned nScales, float fScaleStep, unsigned nAlternatePair, float fRegularityWeight, float fRatioRigidityElasticity,
						   float fThPlanarVertex)
{
	// the externally controlled knobs, validated exactly like the CPU path (Scene::RefineMesh):
	// their ASSERT twins inside the shared stepper are contracts, not input checks
	if (!(fRegularityWeight >= 0 && fRegularityWeight*MeshRefineStep::StepMax <= 1.f)) {
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
	// the planar-vertex removal hook lives only on the CPU path, which owns the mid-scale mesh
	// surgery; refuse it here rather than silently refining without it, so the caller falls back
	// to the CPU path and the option is honoured
	if (fThPlanarVertex > 0) {
		VERBOSE("error: planar-vertex removal (--planar-vertex-ratio) is not implemented on the CUDA backend");
		return false;
	}

	bool bGeneratedPointcloud(false);
	if (pointcloud.IsEmpty() && !ImagesHaveNeighbors()) {
		SampleMeshWithVisibility();
		bGeneratedPointcloud = true;
	}

	MeshRefineCUDA refine(*this, nAlternatePair, fRegularityWeight, fRatioRigidityElasticity, nResolutionLevel, nMinResolution, nMaxViews);
	if (bGeneratedPointcloud)
		pointcloud.Release();
	if (!refine.IsValid())
		return false;

	// run the mesh optimization on multiple scales (coarse to fine)
	for (unsigned nScale=0; nScale<nScales; ++nScale) {
		// init images
		const float scale(POWI(fScaleStep, nScales-nScale-1));
		const float step(POWI(2.f, nScales-nScale));
		DEBUG_ULTIMATE("Refine mesh at: %.2f image scale", scale);
		if (!refine.InitImages(scale, 0.12f*step+0.2f))
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

		// pixel-unit bold driver (MeshRefineStep, SceneRefineCommon.h); mirrors the CPU's
		// Scene::RefineMesh loop (SceneRefine.cpp) minus the CPU-only planar-vertex hook, which
		// the entry validation above refuses rather than ignores
		const int cap((int)MeshRefineStep::Budget(nScale));
		const bool bAlternating(nAlternatePair == 1);

		MeshRefineStep stepper;
		stepper.Reset(mesh.vertices.GetSize());

		// host-side landing buffers for the raw per-vertex terms ScoreMesh downloads every
		// evaluation; unlike the CPU, CUDA has no mid-scale vertex removal, so these are sized
		// once here instead of every ScoreMesh() call
		const uint32_t numVertices(mesh.vertices.GetSize());
		Point3fArr photoGrad(numVertices), smoothGrad1(numVertices), smoothGrad2(numVertices);
		FloatArr photoGradNorm(numVertices), footprint(numVertices);

		bool bCudaFailed(false);
		GET_LOGCONSOLE().Pause();

		// one energy evaluation, stepper decision and log line; shared by both phases below,
		// which differ only in the rho they pass in
		const auto RunEvaluation = [&](float rho) -> MeshRefineStep::Action {
			refine.iteration = stepper.GetNumEvaluated();
			refine.ratioRigidityElasticity = rho;
			const bool bScoreOK(refine.ScoreMesh(photoGrad.Begin(), photoGradNorm.Begin(), footprint.Begin(), smoothGrad1.Begin(), smoothGrad2.Begin()));
			// a CUDA fault poisons the whole context: every later call fails, so without this the
			// loop would keep "refining" a mesh nothing updates any more and still return success;
			// giving up lets the caller fall back to the CPU path. A false bScoreOK is a download
			// failure ScoreMesh already reported, folded into the same branch without a second message
			if (!bScoreOK || cuCtxSynchronize() != CUDA_SUCCESS) {
				GET_LOGCONSOLE().Play();
				if (bScoreOK)
					VERBOSE("error: CUDA mesh refinement failed at scale %u, iteration %u", nScale, refine.iteration+1);
				bCudaFailed = true;
				return MeshRefineStep::STOP;
			}
			if (refine.S < 0) {
				// no image pair contributed a single masked pixel (see ScoreMesh): an expected,
				// recoverable outside-world failure -- abort the refinement loudly
				VERBOSE("error: no image pair overlap at scale %u: mesh refinement aborted", nScale);
				GET_LOGCONSOLE().Play();
				bCudaFailed = true;
				return MeshRefineStep::STOP;
			}
			#ifdef _DEBUG
			// a non-finite gradient is a producer bug and must fire, not be silently skipped (the
			// legacy loop's `if (!ISFINITE(grad)) continue;` is gone along with the legacy loop)
			FOREACH(v, mesh.vertices) {
				ASSERT(ISFINITE(photoGrad[v]));
				ASSERT(ISFINITE(photoGradNorm[v]));
				ASSERT(ISFINITE(footprint[v]));
				ASSERT(ISFINITE(smoothGrad1[v]));
				ASSERT(ISFINITE(smoothGrad2[v]));
			}
			#endif

			MeshRefineStep::Terms terms;
			terms.photoGrad = photoGrad.Begin();
			terms.photoCount = photoGradNorm.Begin();
			terms.footprint = footprint.Begin();
			terms.lap = smoothGrad1.Begin();
			terms.bilap = smoothGrad2.Begin();
			terms.S = refine.S;
			terms.rigidity = rho;
			terms.regularityWeight = refine.weightRegularity;
			terms.numVertices = numVertices;
			terms.alternating = bAlternating;

			MeshRefineStep::Stats stats;
			const MeshRefineStep::Action action(stepper.Evaluate(terms, mesh.vertices, stats));
			DEBUG_EXTRA("\t%2d. S: %.5f (%+.2e)\tstep: %.3fpx\tmed: %.3fpx\t%s",
				(int)stepper.GetNumEvaluated(), stats.S, stats.relChange, stats.step, stats.medianPx, stats.accepted ? "acc" : "rej");
			return action;
		};

		// Phase A: caller's rigidity/elasticity ratio
		{
			Util::Progress progress(_T("Processed iterations"), cap);
			for (int idx=0; idx<cap; ++idx) {
				const MeshRefineStep::Action action(RunEvaluation(fRatioRigidityElasticity));
				progress.display(idx+1);
				if (action == MeshRefineStep::STOP)
					break;
			}
			progress.close();
		}
		if (bCudaFailed)
			return false;

		// Phase B: pure elasticity; eta and the S references carry over, the budget is the stepper's
		const int capB((int)stepper.BeginSecondPhase());
		{
			Util::Progress progress(_T("Processed iterations"), capB);
			for (int idx=0; idx<capB; ++idx) {
				const MeshRefineStep::Action action(RunEvaluation(1.f));
				progress.display(idx+1);
				if (action == MeshRefineStep::STOP)
					break;
			}
			progress.close();
		}
		if (bCudaFailed)
			return false;

		GET_LOGCONSOLE().Play();

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefined%u.ply", nScales-nScale-1)));
		#endif
	}

	return true;
} // RefineMeshCUDA
/*----------------------------------------------------------------*/

#endif // _USE_CUDA
