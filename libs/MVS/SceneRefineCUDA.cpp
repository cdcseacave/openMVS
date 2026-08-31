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
		Image8U::Size size;
		SEACAVE::CUDA::ArrayRT32F image; // float like the CPU's View::image (was half-float; see readSurfFloat in the .cu)
		SEACAVE::CUDA::ArrayRT32F imageGrad[2]; // x/y derivatives (ComputeRefineImageGradient), sampled by the photometric kernel; kept in float like the CPU's View::imageGrad (half-float cannot hold the sub-6e-5 values a derivative stencil produces on flat regions)
		SEACAVE::CUDA::MemDevice depthMap;
		SEACAVE::CUDA::MemDevice faceMap;
		SEACAVE::CUDA::MemDevice baryMap;
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

	void ScoreMesh(float* gradients);

	void ProjectMesh(
		const CameraFaces& cameraFaces,
		const Camera& camera, const Image8U::Size& size, uint32_t idxImage);
	void ProcessPair(uint32_t idxImageA, uint32_t idxImageB);
	void ImageMeshWarp(
		const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
		uint32_t idxImageA, uint32_t idxImageB);
	void ComputeLocalVariance(cudaSurfaceObject_t surfImage, const Image8U::Size& size,
		SEACAVE::CUDA::MemDevice& imageMean, SEACAVE::CUDA::MemDevice& imageVar);
	void ComputeLocalZNCC(cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj, const Image8U::Size& size);
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
	unsigned nScale; // current refinement scale (0-based, coarsest first; RefineDebug export file naming only)

	Scene& scene; // the mesh vertices and faces

	// constant the entire time
	ImageArr& images;
	ViewsArr views; // views' data
	PairIdxArr pairs; // image pairs used to refine the mesh

	std::vector<ViewGPU> viewGPU; // per-view texture/surface objects
	cudaSurfaceObject_t surfImageProjObj = 0; // surface for projected image (imageAB)

	SEACAVE::CUDA::MemDevice vertices;
	SEACAVE::CUDA::MemDevice faces;
	SEACAVE::CUDA::MemDevice faceNormals;
	SEACAVE::CUDA::MemDevice mask;
	SEACAVE::CUDA::MemDevice projKey; // rasterizer scratch, one (depth,face) 64-bit key per pixel of the largest view
	size_t projKeyPixels = 0; // pixels projKey was allocated for (ProjectMesh asserts every view fits)
	SEACAVE::CUDA::MemDevice imageMeanA;
	SEACAVE::CUDA::MemDevice imageVarA;
	SEACAVE::CUDA::ArrayRT32F imageAB; // warped image B in A, float like the CPU's imageAB
	SEACAVE::CUDA::MemDevice imageMeanAB;
	SEACAVE::CUDA::MemDevice imageVarAB;
	SEACAVE::CUDA::MemDevice imageCov;
	SEACAVE::CUDA::MemDevice imageZNCC;
	SEACAVE::CUDA::MemDevice imageDZNCC;
	SEACAVE::CUDA::MemDevice photoGrad;
	SEACAVE::CUDA::MemDevice photoGradNorm;
	SEACAVE::CUDA::MemDevice photoGradPixels;
	SEACAVE::CUDA::MemDevice debugSG; // WP2 parity diagnostic only: per-pixel photometric scalar, allocated when the RefineDebug pair matches
	SEACAVE::CUDA::MemDevice vertexVerticesCont;
	SEACAVE::CUDA::MemDevice vertexVerticesSizes;
	SEACAVE::CUDA::MemDevice vertexVerticesPointers;
	SEACAVE::CUDA::MemDevice vertBoundary; // per-vertex 0/1 boundary flag (shared valence/boundary split, see ListVertexFacesPost())
	SEACAVE::CUDA::MemDevice smoothGrad1;
	SEACAVE::CUDA::MemDevice smoothGrad2;

	enum { HalfSize = Refine::HalfSize }; // half window size used to compute ZNCC (shared with CPU, SceneRefineCommon.h)
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
		// keep only best neighbor views
		const float fMinArea(0.1f);
		const float fMinScale(0.2f), fMaxScale(3.2f);
		const float fMinAngle(D2R(2.5f)), fMaxAngle(D2R(45.f));
		const Image& imageData = images[idxImage];
		if (!imageData.IsValid())
			continue;
		ViewScoreArr neighbors(imageData.neighbors);
		Scene::FilterNeighborViews(neighbors, fMinArea, fMinScale, fMaxScale, fMinAngle, fMaxAngle, nMaxViews);
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
		ComputeRefineImageGradient(img, view.imageGradHost[0], view.imageGradHost[1]);
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
	viewGPU.resize(views.GetSize());
	// texture object with bilinear filtering over a 2D array (element-type reads: 16F and 32F arrays both fetch as float)
	const auto createTexture = [](const auto& array, cudaTextureObject_t& tex) {
		cudaResourceDesc resDesc = {};
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = (cudaArray_t)(CUarray)array;
		cudaTextureDesc texDesc = {};
		texDesc.filterMode = cudaFilterModeLinear;
		texDesc.addressMode[0] = cudaAddressModeClamp;
		texDesc.addressMode[1] = cudaAddressModeClamp;
		texDesc.readMode = cudaReadModeElementType;
		cudaCreateTextureObject(&tex, &resDesc, &texDesc, nullptr);
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
		for (int i=0; i<2; ++i) {
			ASSERT(view.imageGradHost[i].size() == size);
			reportCudaError(view.imageGrad[i].Reset(size, CUDA_ARRAY3D_SURFACE_LDST));
			reportCudaError(view.imageGrad[i].SetData(view.imageGradHost[i]));
			view.imageGradHost[i].release();
			createTexture(view.imageGrad[i], viewGPU[idxImage].texGrad[i]);
		}
		const size_t area((size_t)size.area());
		reportCudaError(view.depthMap.Reset(sizeof(float)*area));
		reportCudaError(view.faceMap.Reset(sizeof(FIndex)*area));
		reportCudaError(view.baryMap.Reset(sizeof(hfloat)*3*area));
		if (maxSize.width < size.width)
			maxSize.width = size.width;
		if (maxSize.height < size.height)
			maxSize.height = size.height;
		// create texture and surface objects for this view
		cudaResourceDesc resDesc = {};
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = (cudaArray_t)(CUarray)view.image;
		// surface object
		cudaCreateSurfaceObject(&viewGPU[idxImage].surfObj, &resDesc);
		// texture object with bilinear filtering
		createTexture(view.image, viewGPU[idxImage].texObj);
	}
	const size_t area(maxSize.area());
	reportCudaError(mask.Reset(sizeof(uint8_t)*area));
	reportCudaError(projKey.Reset(sizeof(uint64_t)*area));
	projKeyPixels = area;
	reportCudaError(imageMeanA.Reset(sizeof(float)*area));
	reportCudaError(imageVarA.Reset(sizeof(float)*area));
	reportCudaError(imageAB.Reset(maxSize, CUDA_ARRAY3D_SURFACE_LDST));
	reportCudaError(imageMeanAB.Reset(sizeof(float)*area));
	reportCudaError(imageVarAB.Reset(sizeof(float)*area));
	reportCudaError(imageCov.Reset(sizeof(float)*area));
	reportCudaError(imageZNCC.Reset(sizeof(float)*area));
	reportCudaError(imageDZNCC.Reset(sizeof(float)*area));
	// create surface object for projected image
	{
		cudaResourceDesc resDesc = {};
		resDesc.resType = cudaResourceTypeArray;
		resDesc.res.array.array = (cudaArray_t)(CUarray)imageAB;
		cudaCreateSurfaceObject(&surfImageProjObj, &resDesc);
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
	// MeshRefine::ComputeSmoothnessGradient1/2, but still needs every OTHER vertex's
	// valence-weighted sum to see a boundary neighbour's true valence -- matching that code's
	// unconditional vertexVertices[idxVert].GetSize(). Previously this
	// zeroed vertSizes[] for boundary vertices to short-circuit their own gradient, which also
	// corrupted every interior neighbour's 1/vertSizes[ni] weight term (divide by zero -> +inf ->
	// that neighbour's smoothGrad2 collapsed to 0 instead of including the boundary contribution).
	const size_t numVertices(scene.mesh.vertices.GetSize());
	Unsigned32Arr _vertexVerticesCont(0, numVertices*6);
	Unsigned32Arr _vertexVerticesSizes(0, numVertices);
	Unsigned32Arr _vertexVerticesPointers(0, numVertices);
	Unsigned8Arr _vertexBoundary(0, numVertices);
	uint32_t lastPosition(0);
	FOREACH(idxV, scene.mesh.vertices) {
		const Mesh::VertexIdxArr& verts = scene.mesh.vertexVertices[idxV];
		ASSERT(!verts.IsEmpty()); // true valence must be > 0 for every vertex
		_vertexVerticesCont.Join(verts.GetData(), verts.GetSize());
		_vertexVerticesSizes.Insert(verts.GetSize());
		_vertexVerticesPointers.Insert(lastPosition); lastPosition += verts.GetSize();
		_vertexBoundary.Insert(scene.mesh.vertexBoundary[idxV] ? 1 : 0);
	}
	reportCudaError(vertexVerticesCont.Reset(_vertexVerticesCont));
	reportCudaError(vertexVerticesSizes.Reset(_vertexVerticesSizes));
	reportCudaError(vertexVerticesPointers.Reset(_vertexVerticesPointers));
	reportCudaError(vertBoundary.Reset(_vertexBoundary));
	// init memory
	reportCudaError(photoGrad.Reset(sizeof(Point3f)*numVertices));
	reportCudaError(photoGradNorm.Reset(sizeof(float)*numVertices));
	reportCudaError(photoGradPixels.Reset(sizeof(float)*numVertices));
	reportCudaError(smoothGrad1.Reset(sizeof(Point3f)*numVertices));
	reportCudaError(smoothGrad2.Reset(sizeof(Point3f)*numVertices));
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
// and compute vertices gradient using analytical method
void MeshRefineCUDA::ScoreMesh(float* gradients)
{
	// extract array of faces viewed by each camera
	ListCameraFaces();

	// compute face normals
	ComputeNormalFaces();

	// init memory
	const VIndex numVertices(scene.mesh.vertices.GetSize());
	reportCudaError(cuMemsetD32(photoGrad, 0, numVertices*3));
	reportCudaError(cuMemsetD32(photoGradNorm, 0, numVertices));

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

	// loop through all vertices and compute the smoothing score
	ComputeSmoothnessGradient(numVertices);

	// WP2 parity diagnostic: download the per-vertex terms before CombineGradients()
	// overwrites photoGrad in-place with the combined result; no-op unless
	// OMVS_REFINE_DEBUG_DIR is set
	if (!RefineDebug::Dir().empty()) {
		Point3fArr pos(numVertices), photo(numVertices), smooth1(numVertices), smooth2(numVertices);
		FloatArr photoNorm(numVertices);
		cList<uint8_t,uint8_t,0> boundary(numVertices);
		reportCudaError(vertices.GetData(pos));
		reportCudaError(photoGrad.GetData(photo));
		reportCudaError(photoGradNorm.GetData(photoNorm));
		reportCudaError(smoothGrad1.GetData(smooth1));
		reportCudaError(smoothGrad2.GetData(smooth2));
		reportCudaError(vertBoundary.GetData(boundary));
		// set the final gradient as the combination of photometric and smoothness gradients
		CombineGradients(numVertices);
		reportCudaError(photoGrad.GetData(gradients, sizeof(Point3f)*numVertices));
		RefineDebug::ExportGradients(nScale, iteration, numVertices,
			pos.Begin(), (const Point3f*)gradients, photo.Begin(), photoNorm.Begin(),
			smooth1.Begin(), smooth2.Begin(), boundary.Begin());
	} else {
		// set the final gradient as the combination of photometric and smoothness gradients
		CombineGradients(numVertices);
		reportCudaError(photoGrad.GetData(gradients, sizeof(Point3f)*numVertices));
	}
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
	MVS::CUDA::LaunchResolveProjection(
		(const uint32_t*)(CUdeviceptr)devFaceIDs,
		(const unsigned long long*)(CUdeviceptr)projKey,
		(float*)(CUdeviceptr)view.depthMap,
		(uint32_t*)(CUdeviceptr)view.faceMap,
		(uint16_t*)(CUdeviceptr)view.baryMap,
		size.width, size.height);
	#if 0
	// debug view
	DepthMap depthMap(size);
	TImage<FIndex> faceMap(size);
	TImage<Point3hf> baryMap(size);
	view.depthMap.GetData(depthMap);
	view.faceMap.GetData(faceMap);
	view.baryMap.GetData(baryMap);
	TImage<Point3f> _baryMap(cvtImage<Point3hf,Point3f>(baryMap));
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
	// init vertex textures
	ComputeLocalVariance(surfImageProjObj, sizeA, imageMeanAB, imageVarAB);
	ComputeLocalVariance(viewGPU[idxImageA].surfObj, sizeA, imageMeanA, imageVarA);
	ComputeLocalZNCC(viewGPU[idxImageA].surfObj, surfImageProjObj, sizeA);

	// WP2 parity diagnostic: download this pair's maps from the persistent
	// device buffers above; no-op unless OMVS_REFINE_DEBUG_DIR/_PAIR are both
	// set and match this exact (A,B) direction
	uint32_t dbgImageA, dbgImageB;
	if (!RefineDebug::Dir().empty() && RefineDebug::Pair(dbgImageA, dbgImageB) &&
		dbgImageA == idxImageA && dbgImageB == idxImageB)
	{
		const int width(sizeA.width), height(sizeA.height);
		Image32F imgA(sizeA), imgAB(sizeA);
		Image8U _mask(sizeA);
		Image32F _varA(sizeA), _varAB(sizeA), _zncc(sizeA), _dzncc(sizeA);
		reportCudaError(views[idxImageA].image.GetData(imgA));
		reportCudaError(imageAB.GetData(imgAB));
		reportCudaError(mask.GetData(_mask));
		reportCudaError(imageVarA.GetData(_varA));
		reportCudaError(imageVarAB.GetData(_varAB));
		reportCudaError(imageZNCC.GetData(_zncc));
		reportCudaError(imageDZNCC.GetData(_dzncc));
		Image32F conf(sizeA);
		for (int r=0; r<height; ++r)
			for (int c=0; c<width; ++c)
				conf(r,c) = _mask(r,c) ? Refine::ZnccReliability(_varA(r,c), _varAB(r,c)) : 0.f;
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "imageA", imgA.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "imageAB", imgAB.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "zncc", _zncc.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "dzncc", _dzncc.getData(), width, height);
		RefineDebug::ExportPairMap(nScale, iteration, idxImageA, idxImageB, "conf", conf.getData(), width, height);
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
		(uint8_t*)(CUdeviceptr)mask,
		MakeCUDACamera(cameraA, size),
		MakeCUDACamera(cameraB, views[idxImageB].size),
		viewGPU[idxImageB].texObj,
		viewGPU[idxImageA].surfObj,
		surfImageProjObj);
}

// compute local variance for each image pixel
void MeshRefineCUDA::ComputeLocalVariance(cudaSurfaceObject_t surfImage, const Image8U::Size& size,
	SEACAVE::CUDA::MemDevice& imageMean, SEACAVE::CUDA::MemDevice& imageVar)
{
	MVS::CUDA::LaunchComputeImageMean(
		(const uint8_t*)(CUdeviceptr)mask,
		(float*)(CUdeviceptr)imageMean,
		surfImage,
		size.width, size.height, HalfSize);
	MVS::CUDA::LaunchComputeImageVar(
		(const float*)(CUdeviceptr)imageMean,
		(const uint8_t*)(CUdeviceptr)mask,
		(float*)(CUdeviceptr)imageVar,
		surfImage,
		size.width, size.height, HalfSize);
	#if 0
	// debug view
	Image32F mean(size);
	Image32F var(size);
	imageMean.GetData(mean);
	imageVar.GetData(var);
	#endif
}

// compute local ZNCC and its gradient for each image pixel
void MeshRefineCUDA::ComputeLocalZNCC(cudaSurfaceObject_t surfImageA, cudaSurfaceObject_t surfImageProj, const Image8U::Size& size)
{
	MVS::CUDA::LaunchComputeImageCov(
		(const float*)(CUdeviceptr)imageMeanA,
		(const float*)(CUdeviceptr)imageMeanAB,
		(const uint8_t*)(CUdeviceptr)mask,
		(float*)(CUdeviceptr)imageCov,
		surfImageA, surfImageProj,
		size.width, size.height, HalfSize);
	MVS::CUDA::LaunchComputeImageZNCC(
		(const float*)(CUdeviceptr)imageCov,
		(const float*)(CUdeviceptr)imageVarA,
		(const float*)(CUdeviceptr)imageVarAB,
		(const uint8_t*)(CUdeviceptr)mask,
		(float*)(CUdeviceptr)imageZNCC,
		size.width, size.height, HalfSize);
	MVS::CUDA::LaunchComputeImageDZNCC(
		(const float*)(CUdeviceptr)imageMeanA,
		(const float*)(CUdeviceptr)imageMeanAB,
		(const float*)(CUdeviceptr)imageVarA,
		(const float*)(CUdeviceptr)imageVarAB,
		(const float*)(CUdeviceptr)imageZNCC,
		(const uint8_t*)(CUdeviceptr)mask,
		(float*)(CUdeviceptr)imageDZNCC,
		surfImageA, surfImageProj,
		size.width, size.height, HalfSize);
}

// compute the photometric gradient for all vertices seen by an image pair
void MeshRefineCUDA::ComputePhotometricGradient(const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
	uint32_t idxImageA, uint32_t idxImageB, uint32_t numVertices, float RegularizationScale)
{
	// compute photometric gradient for all visible vertices
	reportCudaError(cuMemsetD32(photoGradPixels, 0, numVertices));
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
	MVS::CUDA::LaunchComputePhotometricGradient(
		(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
		(const MVS::CUDA::Point3*)(CUdeviceptr)faceNormals,
		(const float*)(CUdeviceptr)views[idxImageA].depthMap,
		(const uint32_t*)(CUdeviceptr)views[idxImageA].faceMap,
		(const uint16_t*)(CUdeviceptr)views[idxImageA].baryMap,
		(const float*)(CUdeviceptr)imageDZNCC,
		(const uint8_t*)(CUdeviceptr)mask,
		(MVS::CUDA::Point3*)(CUdeviceptr)photoGrad,
		(float*)(CUdeviceptr)photoGradPixels,
		dbgPair ? (float*)(CUdeviceptr)debugSG : NULL,
		MakeCUDACamera(cameraA, size),
		MakeCUDACamera(cameraB, views[idxImageB].size),
		viewGPU[idxImageB].texGrad[0], viewGPU[idxImageB].texGrad[1],
		RegularizationScale,
		size.width, size.height);
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
	// update photometric gradient norm for all visible vertices
	MVS::CUDA::LaunchUpdatePhotoGradNorm(
		(float*)(CUdeviceptr)photoGradNorm,
		(const float*)(CUdeviceptr)photoGradPixels,
		numVertices);
	#if 0
	// debug view
	Point3fArr _photoGrad(numVertices);
	FloatArr _photoGradPixels(numVertices);
	FloatArr _photoGradNorm(numVertices);
	photoGrad.GetData(_photoGrad);
	photoGradPixels.GetData(_photoGradPixels);
	photoGradNorm.GetData(_photoGradNorm);
	#endif
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
	#if 0
	// debug view
	Point3fArr _smoothGrad1(numVertices);
	Point3fArr _smoothGrad2(numVertices);
	smoothGrad1.GetData(_smoothGrad1);
	smoothGrad2.GetData(_smoothGrad2);
	#endif
}

void MeshRefineCUDA::CombineGradients(uint32_t numVertices)
{
	// compute smoothness gradient for all vertices
	if (ratioRigidityElasticity >= 1.f) {
		MVS::CUDA::LaunchCombineGradients(
			(MVS::CUDA::Point3*)(CUdeviceptr)photoGrad,
			(const float*)(CUdeviceptr)photoGradNorm,
			(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad2,
			numVertices, weightRegularity);
	} else {
		// compute smoothing gradient as a combination of level 1 and 2 of the Laplacian operator;
		// (see page 105 of "Stereo and Silhouette Fusion for 3D Object Modeling from Uncalibrated Images Under Circular Motion" C. Hernandez, 2004)
		const float rigidity((1.f-ratioRigidityElasticity)*weightRegularity);
		const float elasticity(ratioRigidityElasticity*weightRegularity);
		MVS::CUDA::LaunchCombineAllGradients(
			(MVS::CUDA::Point3*)(CUdeviceptr)photoGrad,
			(const float*)(CUdeviceptr)photoGradNorm,
			(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad1,
			(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad2,
			numVertices, rigidity, elasticity);
	}
	#if 0
	// debug view
	Point3fArr _photoGrad(numVertices);
	photoGrad.GetData(_photoGrad);
	#endif
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

// optimize mesh using photo-consistency
bool Scene::RefineMeshCUDA(unsigned nResolutionLevel, unsigned nMinResolution, unsigned nMaxViews,
						   float fDecimateMesh, unsigned nCloseHoles, unsigned nEnsureEdgeSize, unsigned nMaxFaceArea,
						   unsigned nScales, float fScaleStep, unsigned nAlternatePair, float fRegularityWeight, float fRatioRigidityElasticity, float fGradientStep)
{
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

		// loop a constant number of iterations and apply the gradient
		int iters(25);
		float gstep(0.05f);
		if (fGradientStep > 1) {
			iters = FLOOR2INT(fGradientStep);
			gstep = (fGradientStep-(float)iters)*10;
		}
		iters = MAXF(iters/(int)(nScale+1),8);
		const int iterStop(iters*7/10);
		Eigen::Matrix<float,Eigen::Dynamic,3,Eigen::RowMajor> gradients(mesh.vertices.GetSize(),3);
		Util::Progress progress(_T("Processed iterations"), iters);
		GET_LOGCONSOLE().Pause();
		for (int iter=0; iter<iters; ++iter) {
			refine.iteration = (unsigned)iter;
			refine.nAlternatePair = (iter+1 < iters ? nAlternatePair : 0);
			refine.ratioRigidityElasticity = (iter <= iterStop ? fRatioRigidityElasticity : 1.f);
			// evaluate residuals and gradients
			refine.ScoreMesh(gradients.data());
			// a CUDA fault poisons the whole context: every later call fails, so without this the
			// loop keeps "refining" a mesh nothing updates any more and still returns success --
			// an illegal access in kernelImageMeshWarp did exactly that for an entire benchmark
			// round, logging 85k errors and producing a half-refined mesh with a zero exit code.
			// Giving up here lets the caller fall back to the CPU path, which produces a real mesh.
			if (cuCtxSynchronize() != CUDA_SUCCESS) {
				GET_LOGCONSOLE().Play();
				VERBOSE("error: CUDA mesh refinement failed at scale %u, iteration %d", nScale, iter+1);
				return false;
			}
			// apply gradients
			float gv(0);
			FOREACH(v, mesh.vertices) {
				Vertex& vert = mesh.vertices[v];
				const Point3f grad(gradients.row(v));
				if (!ISFINITE(grad))
					continue;
				vert -= Vertex(grad*gstep);
				gv += norm(grad);
			}
			DEBUG_EXTRA("\t%2d. g: %.5f (%.3e - %.3e)\ts: %.3f", iter+1, gradients.norm(), gradients.norm()/mesh.vertices.GetSize(), gv/mesh.vertices.GetSize(), gstep);
			gstep *= 0.98f;
			progress.display(iter);
		}
		GET_LOGCONSOLE().Play();
		progress.close();

		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			mesh.Save(MAKE_PATH(String::FormatString("MeshRefined%u.ply", nScales-nScale-1)));
		#endif
	}

	return true;
} // RefineMeshCUDA
/*----------------------------------------------------------------*/

#endif // _USE_CUDA
