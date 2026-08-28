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
		Image8U::Size size;
		SEACAVE::CUDA::ArrayRT16F image;
		SEACAVE::CUDA::MemDevice depthMap;
		SEACAVE::CUDA::MemDevice faceMap;
		SEACAVE::CUDA::MemDevice baryMap;
	};
	typedef CLISTDEF2(View) ViewsArr;

	// GPU texture/surface objects per view
	struct ViewGPU {
		cudaTextureObject_t texObj = 0;   // LINEAR filter for bilinear sampling
		cudaSurfaceObject_t surfObj = 0;  // surface for direct read/write
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

	Scene& scene; // the mesh vertices and faces

	// constant the entire time
	ImageArr& images;
	ViewsArr views; // views' data
	PairIdxArr pairs; // image pairs used to refine the mesh

	std::vector<ViewGPU> viewGPU; // per-view texture/surface objects
	cudaSurfaceObject_t surfImageProjObj = 0; // surface for projected image (imageAB)

	SEACAVE::CUDA::MemDevice vertices;
	SEACAVE::CUDA::MemDevice vertexVertices;
	SEACAVE::CUDA::MemDevice faces;
	SEACAVE::CUDA::MemDevice faceNormals;
	SEACAVE::CUDA::MemDevice mask;
	SEACAVE::CUDA::MemDevice imageMeanA;
	SEACAVE::CUDA::MemDevice imageVarA;
	SEACAVE::CUDA::ArrayRT16F imageAB;
	SEACAVE::CUDA::MemDevice imageMeanAB;
	SEACAVE::CUDA::MemDevice imageVarAB;
	SEACAVE::CUDA::MemDevice imageCov;
	SEACAVE::CUDA::MemDevice imageZNCC;
	SEACAVE::CUDA::MemDevice imageDZNCC;
	SEACAVE::CUDA::MemDevice photoGrad;
	SEACAVE::CUDA::MemDevice photoGradNorm;
	SEACAVE::CUDA::MemDevice photoGradPixels;
	SEACAVE::CUDA::MemDevice vertexVerticesCont;
	SEACAVE::CUDA::MemDevice vertexVerticesSizes;
	SEACAVE::CUDA::MemDevice vertexVerticesPointers;
	SEACAVE::CUDA::MemDevice smoothGrad1;
	SEACAVE::CUDA::MemDevice smoothGrad2;

	enum { HalfSize = 2 }; // half window size used to compute ZNCC
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
	for (auto& v : viewGPU) {
		if (v.texObj) cudaDestroyTextureObject(v.texObj);
		if (v.surfObj) cudaDestroySurfaceObject(v.surfObj);
	}
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
		unsigned level(nResolutionLevel);
		const unsigned imageSize(imageData.RecomputeMaxResolution(level, nMinResolution));
		if ((imageData.image.empty() || MAXF(imageData.width,imageData.height) != imageSize) && !imageData.ReloadImage(imageSize)) {
			#ifdef MESHCUDAOPT_USE_OPENMP
			bAbort = true;
			#pragma omp flush (bAbort)
			continue;
			#else
			return false;
			#endif
		}
		View& view = views[idxImage];
		Image32F& img = view.imageHost;
		imageData.image.toGray(img, cv::COLOR_BGR2GRAY, true);
		imageData.image.release();
		if (sigma > 0)
			cv::GaussianBlur(img, img, cv::Size(), sigma);
		if (scale < 1.0) {
			cv::resize(img, img, cv::Size(), scale, scale, cv::INTER_AREA);
			imageData.width = img.width(); imageData.height = img.height();
		}
		imageData.UpdateCamera(scene.platforms);
	}
	#ifdef MESHCUDAOPT_USE_OPENMP
	if (bAbort)
		return false;
	#endif
	// init GPU memory
	Image8U::Size maxSize(0,0);
	// destroy old texture/surface objects before recreating
	for (auto& v : viewGPU) {
		if (v.texObj) { cudaDestroyTextureObject(v.texObj); v.texObj = 0; }
		if (v.surfObj) { cudaDestroySurfaceObject(v.surfObj); v.surfObj = 0; }
	}
	if (surfImageProjObj) { cudaDestroySurfaceObject(surfImageProjObj); surfImageProjObj = 0; }
	viewGPU.resize(views.GetSize());
	FOREACH(idxImage, views) {
		View& view = views[idxImage];
		if (view.imageHost.empty())
			continue;
		Image8U::Size& size(view.size);
		size = view.imageHost.size();
		reportCudaError(view.image.Reset(size, CUDA_ARRAY3D_SURFACE_LDST));
		reportCudaError(view.image.SetData(cvtImage<float,hfloat>(view.imageHost)));
		view.imageHost.release();
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
		cudaTextureDesc texDesc = {};
		texDesc.filterMode = cudaFilterModeLinear;
		texDesc.addressMode[0] = cudaAddressModeClamp;
		texDesc.addressMode[1] = cudaAddressModeClamp;
		texDesc.readMode = cudaReadModeElementType;
		cudaCreateTextureObject(&viewGPU[idxImage].texObj, &resDesc, &texDesc, nullptr);
	}
	const size_t area(maxSize.area());
	reportCudaError(mask.Reset(sizeof(uint8_t)*area));
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
	// set vertex vertices
	reportCudaError(vertexVertices.Reset(scene.mesh.vertexVertices));
	// list adjacent vertices for each vertex
	const size_t numVertices(scene.mesh.vertices.GetSize());
	Unsigned32Arr _vertexVerticesCont(0, numVertices*6);
	Unsigned32Arr _vertexVerticesSizes(0, numVertices);
	Unsigned32Arr _vertexVerticesPointers(0, numVertices);
	uint32_t lastPosition(0);
	FOREACH(idxV, scene.mesh.vertices) {
		if (scene.mesh.vertexBoundary[idxV]) {
			_vertexVerticesSizes.Insert(0);
			_vertexVerticesPointers.Insert(lastPosition);
			continue;
		}
		const Mesh::VertexIdxArr& verts = scene.mesh.vertexVertices[idxV];
		_vertexVerticesCont.Join(verts.GetData(), verts.GetSize());
		_vertexVerticesSizes.Insert(verts.GetSize());
		_vertexVerticesPointers.Insert(lastPosition); lastPosition += verts.GetSize();
	}
	reportCudaError(vertexVerticesCont.Reset(_vertexVerticesCont));
	reportCudaError(vertexVerticesSizes.Reset(_vertexVerticesSizes));
	reportCudaError(vertexVerticesPointers.Reset(_vertexVerticesPointers));
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

	// set the final gradient as the combination of photometric and smoothness gradients
	CombineGradients(numVertices);
	reportCudaError(photoGrad.GetData(gradients, sizeof(Point3f)*numVertices));
}


// project mesh to the given camera plane
void MeshRefineCUDA::ProjectMesh(
	const CameraFaces& cameraFaces,
	const Camera& camera, const Image8U::Size& size, uint32_t idxImage)
{
	View& view = views[idxImage];
	// init depth-map and face-map (matching CPU's RasterMesh::Clear())
	reportCudaError(cuMemsetD32(view.depthMap, CastF2I(FLT_MAX).i, size.area()));
	reportCudaError(cuMemsetD32(view.faceMap, NO_ID, size.area()));
	// fetch only the faces viewed by this camera
	Mesh::FaceIdxArr faceIDsView(0, (FIndex)cameraFaces.size());
	for (auto idxFace : cameraFaces)
		faceIDsView.Insert(idxFace);
	// project mesh
	SEACAVE::CUDA::MemDevice devFaceIDs(faceIDsView);
	MVS::CUDA::LaunchProjectMesh(
		(const MVS::CUDA::Point3*)(CUdeviceptr)vertices,
		(const MVS::CUDA::Point3u*)(CUdeviceptr)faces,
		(const uint32_t*)(CUdeviceptr)devFaceIDs,
		(float*)(CUdeviceptr)view.depthMap,
		(uint32_t*)(CUdeviceptr)view.faceMap,
		(uint16_t*)(CUdeviceptr)view.baryMap,
		MakeCUDACamera(camera, size),
		faceIDsView.GetSize());
	// cross-check valid depth and face index
	MVS::CUDA::LaunchCrossCheckProjection(
		(float*)(CUdeviceptr)view.depthMap,
		(uint32_t*)(CUdeviceptr)view.faceMap,
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
		MakeCUDACamera(cameraB, size),
		viewGPU[idxImageB].texObj,
		viewGPU[idxImageA].surfObj,
		surfImageProjObj);
	#if 0
	// debug view
	Image16F _imageAB(size);
	Image8U _mask(size);
	imageAB.GetData(_imageAB);
	mask.GetData(_mask);
	Image32F __imageAB(cvtImage<hfloat,float>(_imageAB));
	#endif
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
	#if 0
	// debug view
	Image32F _imageZNCC(size);
	Image32F _imageDZNCC(size);
	imageZNCC.GetData(_imageZNCC);
	imageDZNCC.GetData(_imageDZNCC);
	#endif
}

// compute the photometric gradient for all vertices seen by an image pair
void MeshRefineCUDA::ComputePhotometricGradient(const Camera& cameraA, const Camera& cameraB, const Image8U::Size& size,
	uint32_t idxImageA, uint32_t idxImageB, uint32_t numVertices, float RegularizationScale)
{
	// compute photometric gradient for all visible vertices
	reportCudaError(cuMemsetD32(photoGradPixels, 0, numVertices));
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
		MakeCUDACamera(cameraA, size),
		MakeCUDACamera(cameraB, size),
		viewGPU[idxImageB].texObj,
		RegularizationScale,
		size.width, size.height);
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
		(MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad1,
		numVertices, uint8_t(0));
	MVS::CUDA::LaunchComputeSmoothnessGradient(
		(const MVS::CUDA::Point3*)(CUdeviceptr)smoothGrad1,
		(const uint32_t*)(CUdeviceptr)vertexVerticesCont,
		(const uint32_t*)(CUdeviceptr)vertexVerticesSizes,
		(const uint32_t*)(CUdeviceptr)vertexVerticesPointers,
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
