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
		SEACAVE::CUDA::MemDevice baryMap; // one ushort4 per pixel: the three half barycentrics and a pad, one 8-byte word each way
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
	MeshRefineCUDA(Scene& _scene, unsigned _nAlternatePair=true, float _weightRegularity=1.5f, unsigned _nResolutionLevel=0, unsigned _nMinResolution=640, unsigned nMaxViews=8);
	~MeshRefineCUDA();

	bool IsValid() const { return !pairs.IsEmpty(); }

	bool InitKernels();
	bool InitImages(float scale, float sigma=0);

	void ListVertexFacesPre();
	void ListVertexFacesPost();
	void ListCameraFaces();

	void ListFaceAreas(Mesh::AreaArr& maxAreas);
	void SubdivideMesh(uint32_t maxArea, float fDecimate=1.f, unsigned nCloseHoles=15, unsigned nEnsureEdgeSize=1);
	// decimate the refined mesh within the given reprojection tolerance, measured in the
	// projected areas ListFaceAreas() reports (px at the working resolution)
	void SimplifyMesh(float tolerancePx);

	void ComputeNormalFaces();

	// one energy evaluation: leaves the raw per-vertex terms (BEFORE any combine) in the pinned
	// host buffer `terms` is pointed into, and the reliability-weighted score in S (S < 0 means
	// no image pair contributed a single masked pixel -- see the sumR check below); the stepper
	// (MeshRefineStep, SceneRefineCommon.h) does the combining, not this function. Returns false
	// (having already VERBOSE'd why) if the GPU->host download failed: a non-sticky copy failure
	// is not caught by the caller's cuCtxSynchronize() check and would otherwise hand the
	// stepper finite-looking garbage.
	bool ScoreMesh(MeshRefineStep::Terms& terms);

	void ProjectMesh(const Camera& camera, const Image8U::Size& size, uint32_t idxImage);
	// one pair-direction: warp B into A, the window statistics with the per-pixel photometric
	// term, then its accumulation; numSlots counts the window-statistics blocks written so far
	// into statsBlockSums, which ScoreMesh() reduces once at the end
	void ProcessPair(uint32_t idxImageA, uint32_t idxImageB, uint32_t& numSlots);
	void ComputeSmoothnessGradient(uint32_t numVertices);

	// float offsets of the fields of `terms`/`termsHost` for N vertices
	struct TermsLayout {
		size_t photoGrad, lap, bilap, photoCount, footprint, sums, size;
		explicit TermsLayout(size_t N) : photoGrad(0), lap(3*N), bilap(6*N), photoCount(9*N), footprint(10*N), sums(11*N), size(11*N+2) {}
	};
	CUdeviceptr TermsPtr(size_t offset) const { return (CUdeviceptr)terms + sizeof(float)*offset; }
	// how many pair-directions one ScoreMesh() runs
	uint32_t NumDirections() const { return (uint32_t)pairs.GetSize() * (nAlternatePair == 0 ? 2u : 1u); }

public:
	const float weightRegularity; // a scalar regularity weight to balance between photo-consistency and regularization terms
	const unsigned nResolutionLevel; // how many times to scale down the images before mesh optimization
	const unsigned nMinResolution; // how many times to scale down the images before mesh optimization
	unsigned nAlternatePair; // using an image pair alternatively as reference image (0 - both, 1 - alternate, 2 - only left, 3 - only right)
	unsigned iteration; // current refinement iteration
	unsigned nScale; // current refinement scale (0-based, coarsest first), for the log

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
	SEACAVE::CUDA::MemDevice maskStats; // the pixels that contribute a photometric term: mask pruned by MinWindowCount, the rejection gates and the grazing-angle test (kernelComputeWindowStats)
	SEACAVE::CUDA::MemDevice projKey; // rasterizer scratch, one (depth,face) 64-bit key per pixel of the largest view
	size_t projKeyPixels = 0; // pixels projKey was allocated for (ProjectMesh asserts every view fits)
	SEACAVE::CUDA::ArrayRT32F imageAB; // warped image B in A, float like the CPU's imageAB
	SEACAVE::CUDA::MemDevice pixelGrad; // per pixel of the largest view: g_p, the photometric term the covering face's corners share (kernelComputeWindowStats)
	SEACAVE::CUDA::MemDevice statsBlockSums; // 2 floats per window-statistics block of every pair-direction of one ScoreMesh(): the partials S is folded from, in slot order
	// the raw per-vertex terms of one evaluation in ONE device buffer, so that a single download
	// into its pinned host mirror hands the stepper everything it reads (five pageable copies used
	// to be a sync point each, staged through the driver's bounce buffer); see TermsLayout
	SEACAVE::CUDA::MemDevice terms;
	float* termsHost = NULL;
	// per-FACE private accumulators of the photometric term, the atomic-free replacement for a
	// per-pixel scatter (see kernelAccumulateFacePhoto), in one buffer: 3 floats per face for the
	// corners' Sum g_p*b_c, 1 for the pixel count, 1 for the min footprint; accumulated over the
	// pair-directions of one ScoreMesh() and cleared once per call
	SEACAVE::CUDA::MemDevice faceTerms;
	SEACAVE::CUDA::MemDevice vertexSeen; // 1 byte per vertex: the mark a direction's contributing faces leave for kernelCountSeenVertices
	SEACAVE::CUDA::MemDevice vertexVerticesCont;
	SEACAVE::CUDA::MemDevice vertexVerticesSizes;
	SEACAVE::CUDA::MemDevice vertexVerticesPointers;
	// the same flattening for the INCIDENT FACES of each vertex (Mesh::ListIncidentFaces order):
	// the gather half of the photometric accumulation walks it instead of the pixels scattering
	SEACAVE::CUDA::MemDevice vertexFacesCont;
	SEACAVE::CUDA::MemDevice vertexFacesSizes;
	SEACAVE::CUDA::MemDevice vertexFacesPointers;
	SEACAVE::CUDA::MemDevice vertBoundary; // per-vertex 0/1 boundary flag (shared valence/boundary split, see ListVertexFacesPost())
};

MeshRefineCUDA::MeshRefineCUDA(Scene& _scene, unsigned _nAlternatePair, float _weightRegularity, unsigned _nResolutionLevel, unsigned _nMinResolution, unsigned nMaxViews)
	:
	weightRegularity(_weightRegularity),
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
	if (termsHost) reportCudaError(cuMemFreeHost(termsHost));
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
		reportCudaError(view.baryMap.Reset(sizeof(ushort4)*area));
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
	reportCudaError(pixelGrad.Reset(sizeof(float)*area));
	// one slot per 16x16 block of the window-statistics grid per pair-direction; the largest
	// view's width and height need not come from the same view, so this is an upper bound
	{
		const size_t maxBlocks(((size_t)maxSize.width + 15)/16 * (((size_t)maxSize.height + 15)/16));
		reportCudaError(statsBlockSums.Reset(sizeof(float)*2*maxBlocks*NumDirections()));
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
	// walks that fixed order to sum the photometric term for the vertex, which is what makes the
	// sum reproducible without atomics
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
	// the evaluation buffers, sized once here since this mesh is final for the scale: the terms
	// and their pinned mirror, the per-face accumulators (20 bytes per face) and the vertex marks
	// (cleared once; kernelCountSeenVertices leaves them clear)
	const TermsLayout layout(numVertices);
	reportCudaError(terms.Reset(sizeof(float)*layout.size));
	if (termsHost)
		reportCudaError(cuMemFreeHost(termsHost));
	reportCudaError(cuMemAllocHost((void**)&termsHost, sizeof(float)*layout.size));
	reportCudaError(faceTerms.Reset(sizeof(float)*5*scene.mesh.faces.GetSize()));
	reportCudaError(vertexSeen.Reset(numVertices));
	reportCudaError(cuMemsetD8(vertexSeen, 0, numVertices));
}

// upload the current vertices and rasterize the mesh into every view (depth, face and
// barycentric maps): the whole mesh each time, the kernel rejecting the faces a view does not
// see -- see kernelProjectMesh for why no host-side frustum cull shortlists them
void MeshRefineCUDA::ListCameraFaces()
{
	reportCudaError(vertices.Reset(scene.mesh.vertices));
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (imageData.IsValid())
			ProjectMesh(imageData.camera, views[idxImage].size, idxImage);
	}
}

// compute for each face the one projected area the preparation and the decimation both read:
// rasterized per view, then reduced over the refinement's own pairs by ReduceFaceAreasOverPairs
// (the smaller view of a pair, the largest pair); ListCameraFaces() must have run before
void MeshRefineCUDA::ListFaceAreas(Mesh::AreaArr& maxAreas)
{
	ASSERT(maxAreas.IsEmpty());
	// for each image a pair uses, compute the projection area of visible faces
	Unsigned8Arr usedImages;
	ListPairImages(pairs, images.GetSize(), usedImages);
	ViewAreaArr viewAreas(images.GetSize());
	FOREACH(idxImage, images) {
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid() || !usedImages[idxImage])
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
	maxAreas.Resize(scene.mesh.faces.GetSize());
	ReduceFaceAreasOverPairs(viewAreas, pairs, maxAreas);
}

// the shared preparation (PrepareRefineMesh, SceneRefineCommon.h): decimate, remesh and
// subdivide so that no face projects larger than the area cap in both images of a pair
void MeshRefineCUDA::SubdivideMesh(uint32_t maxArea, float fDecimate, unsigned nCloseHoles, unsigned nEnsureEdgeSize)
{
	PrepareRefineMesh(*this, maxArea, fDecimate, nCloseHoles, nEnsureEdgeSize);
}


void MeshRefineCUDA::SimplifyMesh(float tolerancePx)
{
	// the tolerance is a reprojection error, so it is measured in the same projected areas the
	// preparation splits against -- every other reading of "how big is this face on screen" in
	// this file goes through ListFaceAreas, and a decimation that used a different one would
	// keep faces no pair of the refinement can see
	ListCameraFaces();
	Mesh::AreaArr seenAreas;
	ListFaceAreas(seenAreas);
	FloatArr pixelFactors;
	SeenAreasToPixelFactors(scene.mesh, seenAreas, pixelFactors);
	SimplifyMeshWithinTolerance(scene.mesh, pixelFactors, tolerancePx);
	ListVertexFacesPre();
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


// score mesh using photo-consistency and leave the raw per-vertex terms in `out`, pointing into
// the pinned host buffer; the caller's stepper (MeshRefineStep, SceneRefineCommon.h) combines
// them into a step
bool MeshRefineCUDA::ScoreMesh(MeshRefineStep::Terms& out)
{
	// rasterize the current vertices into every view, then the terms that need nothing else:
	// the face normals and the two smoothness terms queue up behind the rasterization and run
	// while the host is still issuing the pairs
	ListCameraFaces();
	ComputeNormalFaces();
	const VIndex numVertices(scene.mesh.vertices.GetSize());
	const TermsLayout layout(numVertices);
	ComputeSmoothnessGradient(numVertices);

	// clear this evaluation's accumulators: the per-vertex direction count and the per-face
	// slots (0, and the FLT_MAX the footprint mins start from)
	const FIndex numFaces(scene.mesh.faces.GetSize());
	reportCudaError(cuMemsetD32(TermsPtr(layout.photoCount), 0, numVertices));
	reportCudaError(cuMemsetD32(faceTerms, 0, (size_t)numFaces*4));
	reportCudaError(cuMemsetD32((CUdeviceptr)faceTerms + sizeof(float)*4*numFaces, 0x7F7FFFFF, numFaces));

	// for each pair of images, compute a photo-consistency score
	// between the reference image and the pixels of the second image
	// projected in the reference image through the mesh surface
	uint32_t numSlots(0);
	FOREACHPTR(pPair, pairs) {
		ASSERT(pPair->i < pPair->j);
		switch (nAlternatePair) {
		case 1: {
			const PairIdx pair(iteration%2 ? PairIdx(pPair->j,pPair->i) : PairIdx(pPair->i,pPair->j));
			ProcessPair(pair.i, pair.j, numSlots);
			break; }
		case 2: {
			ProcessPair(pPair->i, pPair->j, numSlots);
			break; }
		case 3: {
			ProcessPair(pPair->j, pPair->i, numSlots);
			break; }
		default:
			for (int ip=0; ip<2; ++ip) {
				const PairIdx pair(ip ? PairIdx(pPair->j,pPair->i) : PairIdx(pPair->i,pPair->j));
				ProcessPair(pair.i, pair.j, numSlots);
			}
		}
	}

	// the two sums S is made of and the per-vertex photometric term, then everything comes down
	// in one copy
	MVS::CUDA::LaunchReduceBlockSums(
		(const float*)(CUdeviceptr)statsBlockSums, numSlots,
		(float*)TermsPtr(layout.sums), (float*)TermsPtr(layout.sums+1));
	MVS::CUDA::LaunchGatherVertexPhoto(
		(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
		(const MVS::CUDA::Point3*)(CUdeviceptr)faceNormals,
		(const uint32_t*)(CUdeviceptr)vertexFacesCont,
		(const uint32_t*)(CUdeviceptr)vertexFacesSizes,
		(const uint32_t*)(CUdeviceptr)vertexFacesPointers,
		(const float*)(CUdeviceptr)faceTerms,
		(const float*)((CUdeviceptr)faceTerms + sizeof(float)*3*numFaces),
		(const float*)((CUdeviceptr)faceTerms + sizeof(float)*4*numFaces),
		(MVS::CUDA::Point3*)TermsPtr(layout.photoGrad),
		(float*)TermsPtr(layout.footprint),
		numVertices);
	if (reportCudaError(terms.GetData(termsHost, sizeof(float)*layout.size)) != CUDA_SUCCESS) {
		VERBOSE("error: failed downloading the refinement terms from the GPU at scale %u, iteration %u", nScale, iteration);
		return false;
	}

	// S = sumRZ/sumR, the reliability-weighted mean of (1-ZNCC). sumR == 0 means no pair-direction
	// contributed a single masked pixel -- broken outside-world input (no pair overlap, bad
	// poses), not an internal invariant, so it gets the runtime-validation treatment instead of
	// an ASSERT alone (which would compile out in Release and feed the stepper NaN): S is left
	// negative and the caller fails the refinement loudly, exactly like MeshRefine::ScoreMesh
	const float sumR(termsHost[layout.sums]), sumRZ(termsHost[layout.sums+1]);
	if (sumR > 0) {
		S = sumRZ/sumR;
		ASSERT(S >= 0 && S <= 2);
	} else {
		S = -1.f;
	}
	out.photoGrad = (const MeshRefineStep::Grad*)(termsHost + layout.photoGrad);
	out.photoCount = termsHost + layout.photoCount;
	out.footprint = termsHost + layout.footprint;
	out.lap = (const MeshRefineStep::Grad*)(termsHost + layout.lap);
	out.bilap = (const MeshRefineStep::Grad*)(termsHost + layout.bilap);
	out.S = S;
	out.numVertices = numVertices;
	return true;
}


// project mesh to the given camera plane
void MeshRefineCUDA::ProjectMesh(const Camera& camera, const Image8U::Size& size, uint32_t idxImage)
{
	View& view = views[idxImage];
	ASSERT(projKey.IsValid() && (size_t)size.area() <= projKeyPixels);
	// pass 1 needs every pixel key at "no face yet" = ~0ull (larger than any real (depth,face) key)
	reportCudaError(cuMemsetD32(projKey, 0xFFFFFFFFu, 2*(size_t)size.area()));
	// project mesh: pass 1 elects per pixel the nearest face (the lower id on a depth tie), pass
	// 2 lets the winner write its payload, then the uncovered pixels are cleared (both maps are
	// preset so no pixel can carry this view's previous iteration forward: faceMap to NO_ID,
	// which the Debug check in the last kernel would otherwise let a stale id satisfy, and
	// depthMap to 0, so that a pixel missing its payload reads as uncovered downstream instead of
	// pairing a stale depth with a NO_ID face)
	const MVS::CUDA::Camera cudaCamera(MakeCUDACamera(camera, size));
	const FIndex numFaces(scene.mesh.faces.GetSize());
	reportCudaError(cuMemsetD32(view.faceMap, NO_ID, size.area()));
	reportCudaError(cuMemsetD32(view.depthMap, 0, size.area()));
	for (int pass=0; pass<2; ++pass)
		MVS::CUDA::LaunchProjectMesh(
			(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
			(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
			(unsigned long long*)(CUdeviceptr)projKey,
			(float*)(CUdeviceptr)view.depthMap,
			(uint32_t*)(CUdeviceptr)view.faceMap,
			(ushort4*)(CUdeviceptr)view.baryMap,
			cudaCamera, numFaces, pass == 1);
	#ifdef _DEBUG
	// every covered pixel must hold the payload of exactly the face that won its key
	MVS::CUDA::LaunchCheckProjection(
		(const unsigned long long*)(CUdeviceptr)projKey,
		(const float*)(CUdeviceptr)view.depthMap,
		(const uint32_t*)(CUdeviceptr)view.faceMap,
		size.width, size.height);
	#endif
}

void MeshRefineCUDA::ProcessPair(uint32_t idxImageA, uint32_t idxImageB, uint32_t& numSlots)
{
	const Image& imageDataA = images[idxImageA];
	const Image& imageDataB = images[idxImageB];
	ASSERT(imageDataA.IsValid() && imageDataB.IsValid());
	const View& viewA = views[idxImageA];
	const View& viewB = views[idxImageB];
	const MVS::CUDA::Camera cudaCamA(MakeCUDACamera(imageDataA.camera, viewA.size));
	const MVS::CUDA::Camera cudaCamB(MakeCUDACamera(imageDataB.camera, viewB.size));
	// warp imageB to imageA using the mesh
	MVS::CUDA::LaunchImageMeshWarp(
		(const float*)(CUdeviceptr)viewA.depthMap,
		(const float*)(CUdeviceptr)viewB.depthMap,
		viewA.keepMask.IsValid() ? (const uint8_t*)(CUdeviceptr)viewA.keepMask : NULL,
		viewB.keepMask.IsValid() ? (const uint8_t*)(CUdeviceptr)viewB.keepMask : NULL,
		(uint8_t*)(CUdeviceptr)mask,
		cudaCamA, cudaCamB, viewGPU[idxImageB].texObj, surfImageProjObj);
	// masked window statistics, rejection gates, ZNCC and its derivative, and the per-pixel
	// photometric term; mode 3 samples the bilinear interpolant of image B directly and needs no
	// precomputed gradient stencil texture (InitImages never uploads one in that mode)
	const float RegularizationScale((float)((REAL)(imageDataA.avgDepth*imageDataB.avgDepth)/(imageDataA.camera.GetFocalLength()*imageDataB.camera.GetFocalLength())));
	numSlots += MVS::CUDA::LaunchComputeWindowStats(
		(const uint8_t*)(CUdeviceptr)mask,
		(uint8_t*)(CUdeviceptr)maskStats,
		(float*)(CUdeviceptr)pixelGrad,
		(float*)((CUdeviceptr)statsBlockSums + sizeof(float)*2*numSlots),
		viewGPU[idxImageA].surfObj, surfImageProjObj,
		(const MVS::CUDA::Point3*)(CUdeviceptr)faceNormals,
		(const uint32_t*)(CUdeviceptr)viewA.faceMap,
		(const float*)(CUdeviceptr)viewA.depthMap,
		cudaCamA, cudaCamB,
		viewGPU[idxImageB].texObj, viewGPU[idxImageB].texGrad[0], viewGPU[idxImageB].texGrad[1],
		OPTREFINE::nImageGradient == 3, RegularizationScale,
		OPTREFINE::fGateMeanDiff, OPTREFINE::fGateVarRatio,
		viewA.size.width, viewA.size.height);
	// the atomic-free accumulation: every face folds its pixels into its own slots, then the
	// vertices they reach count the direction (see kernelAccumulateFacePhoto)
	const FIndex numFaces(scene.mesh.faces.GetSize());
	const VIndex numVertices(scene.mesh.vertices.GetSize());
	MVS::CUDA::LaunchAccumulateFacePhoto(
		(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
		(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
		(const float*)(CUdeviceptr)viewA.depthMap,
		(const uint32_t*)(CUdeviceptr)viewA.faceMap,
		(const ushort4*)(CUdeviceptr)viewA.baryMap,
		(const float*)(CUdeviceptr)pixelGrad,
		(const uint8_t*)(CUdeviceptr)maskStats,
		(float*)(CUdeviceptr)faceTerms,
		(float*)((CUdeviceptr)faceTerms + sizeof(float)*3*numFaces),
		(float*)((CUdeviceptr)faceTerms + sizeof(float)*4*numFaces),
		(uint8_t*)(CUdeviceptr)vertexSeen,
		cudaCamA, numFaces);
	MVS::CUDA::LaunchCountSeenVertices(
		(uint8_t*)(CUdeviceptr)vertexSeen,
		(float*)TermsPtr(TermsLayout(numVertices).photoCount),
		numVertices);
}

void MeshRefineCUDA::ComputeSmoothnessGradient(uint32_t numVertices)
{
	// compute smoothness gradient for all vertices
	const TermsLayout layout(numVertices);
	MVS::CUDA::LaunchComputeSmoothnessGradient(
		(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
		(const uint32_t*)(CUdeviceptr)vertexVerticesCont,
		(const uint32_t*)(CUdeviceptr)vertexVerticesSizes,
		(const uint32_t*)(CUdeviceptr)vertexVerticesPointers,
		(const uint8_t*)(CUdeviceptr)vertBoundary,
		(MVS::CUDA::Point3*)TermsPtr(layout.lap),
		numVertices, uint8_t(0));
	MVS::CUDA::LaunchComputeSmoothnessGradient(
		(const MVS::CUDA::Point3*)TermsPtr(layout.lap),
		(const uint32_t*)(CUdeviceptr)vertexVerticesCont,
		(const uint32_t*)(CUdeviceptr)vertexVerticesSizes,
		(const uint32_t*)(CUdeviceptr)vertexVerticesPointers,
		(const uint8_t*)(CUdeviceptr)vertBoundary,
		(MVS::CUDA::Point3*)TermsPtr(layout.bilap),
		numVertices, uint8_t(1));
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

	MeshRefineCUDA refine(*this, nAlternatePair, fRegularityWeight, nResolutionLevel, nMinResolution, nMaxViews);
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
		// the sigma multiplier is tied to MeshRefineStep::StepGrow, see the note there
		if (!refine.InitImages(scale, 0.09f*step+0.15f))
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

		bool bCudaFailed(false);
		GET_LOGCONSOLE().Pause();

		// one energy evaluation, stepper decision and log line; shared by both phases below,
		// which differ only in the rho they pass in
		const auto RunEvaluation = [&](float rho) -> MeshRefineStep::Action {
			refine.iteration = stepper.GetNumEvaluated();
			MeshRefineStep::Terms terms;
			const bool bScoreOK(refine.ScoreMesh(terms));
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
				ASSERT(ISFINITE(terms.photoGrad[v]));
				ASSERT(ISFINITE(terms.photoCount[v]));
				ASSERT(ISFINITE(terms.footprint[v]));
				ASSERT(ISFINITE(terms.lap[v]));
				ASSERT(ISFINITE(terms.bilap[v]));
			}
			#endif

			terms.rigidity = rho;
			terms.regularityWeight = refine.weightRegularity;
			terms.alternating = bAlternating;

			MeshRefineStep::Stats stats;
			const MeshRefineStep::Action action(stepper.Evaluate(terms, mesh.vertices, stats));
			// same fields and order as the CPU line in SceneRefine.cpp, including the
			// vertex-removal count (always 0 here: this backend removes none), so the two traces
			// read alike
			DEBUG_EXTRA("\t%2d. S: %.5f (%+.2e)\tstep: %.3fpx\tmed: %.3fpx\tv: %5u\t%s",
				(int)stepper.GetNumEvaluated(), stats.S, stats.relChange, stats.step, stats.medianPx, 0u, stats.accepted ? "acc" : "rej");
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

	// the deliverable: the refined surface within a reprojection tolerance (the same pass as the CPU)
	if (OPTREFINE::fSimplifyTolerance > 0)
		refine.SimplifyMesh(OPTREFINE::fSimplifyTolerance);

	return true;
} // RefineMeshCUDA
/*----------------------------------------------------------------*/

#endif // _USE_CUDA
