//#include "../Mesh.h"
#include "../CUDA/DeviceBuffer.h"
#include "../CUDA/DeviceMat.h"
namespace MVS {
class CURAST {
public:
	// construct a CURAST instance, allocating per-face GPU buffers for scoring and best-view selection
	// _ptrStream: CUDA stream used for all asynchronous GPU operations
	// _stackSize: maximum number of view samples accumulated per texture pixel
	// _nViews: number of candidate best views tracked per face
	// nFaces: total number of triangular faces in the mesh
	CURAST(const SEACAVE::CUDA::CudaStreamSharedPtr& _ptrStream, const uint _stackSize, const uint _nViews, const uint nFaces);

	// return the raw CUDA stream handle associated with this instance
	cudaStream_t Stream() const {
		return ptrStream.get();
	}

	// copy the per-face best-view scores and indices from device to host buffers (hBestViewsScore, hBestViewsIdx)
	void DownloadBestViewsScore();
	// upload the best-view index and score data for the given face chunk from host to device
	void LoadBestViewsDataOnDevice( std::vector<uint>& chunk);
	// download a single texture image from the given pyramid level and stack slot
	void CopyFinalTextureToHost(cv::Mat& finalTextureHost, uint level, uint nStack);
	// download the per-pixel visibility score map used for texture blending
	void CopyVisibilityMapTexToHost(cv::Mat& visibilityMapHost);
	// download the raw per-pixel visibility map from the last mesh projection
	void CopyVisibilityMapToHost(cv::Mat& visibilityMapHost);
	// allocate or resize all buffers of texture dimensions
	void InitializeTexturePyramidOnDevice(const cv::Size size,const int level);
	// allocate and zero-initialize the face map and visibility map device images for the given image resolution
	// if useVisibility is true, the visibility map is also allocated
	void InitializeFaceMap(cv::Size imgSize, bool useVisibility = true);
	// upload the camera for the given view index to device-side camera state
	void SetCameraMatrices(const CUDA::Point3& C, const CUDA::Matrix3& R, const CUDA::Matrix3& K, const uint _viewIdx);
	// mark which faces are visible from the given view and upload the full face list to the isBestView device buffer
	void SetFacesForView(uint viewIdx, std::vector<uint>& faceList);
	// mark only the top nBest scoring faces for the given view and upload them to the isBestView device buffer
	void SetFacesForView(uint viewIdx, std::vector<uint>& faceList, uint nBest);
	// zero all Laplacian pyramid texture stack device buffers and their associated view-index maps in preparation for a new tile
	void ResetPyramidTextureStacks();
	// build a custom image pyramid from input image with aggressive downsampling for the lower resolution levels
	void MakeImageCustomPyramid(const cv::Mat& inputImage, const uint nLevels);

	// return a const reference to the host-side per-face best-view score vector
	const std::vector<float>& GetBestViewScore() const { return hBestViewsScore; }
	// return a const reference to the host-side per-face best-view index vector
	const std::vector<int>& GetBestViewIdxRef() const { return hBestViewsIdx; }
	// return a const reference to the host-side per-face projected pixel count vector
	const std::vector<uint>& GetResFaceRef() const { return hResFace; }
protected:
	// host data
	std::vector<float> hBestViewsScore; // host copy of the best-view scores, one entry per face
	std::vector<int> hBestViewsIdx;     // host copy of the best-view indices, one entry per face
	std::vector<uint> hResFace;         // host copy of the projected pixel count per face
	std::vector<uint> meshChunk;        // face indices of the mesh subset currently loaded on device

	uint viewIdx; // index of the current view being processed
	CUDA::TDeviceBuffer<int> isBestView; // per-face flag: 1 if the current view is among the best views for that face, 0 otherwise
	SEACAVE::CUDA::CudaStreamSharedPtr ptrStream; // CUDA stream used for all asynchronous GPU operations
	
	// camera data
	CUDA::Point3 C;  // camera centre in world coordinates
	CUDA::Matrix3 R; // rotation matrix transforming world coordinates to camera coordinates
	CUDA::Matrix3 K; // camera intrinsic matrix (focal lengths and principal point)

	// mesh data	
	CUDA::TDeviceBuffer<CUDA::Point3> bufferPositions; // vertex positions of the current mesh chunk on device

	// data of mesh projection on image plan
	CUDA::TDeviceMat<float> depthMap;      // per-pixel depth of the nearest projected face
	CUDA::TDeviceMat<int> faceMap;         // per-pixel index of the nearest projected face (-1 if none)
	CUDA::TDeviceMat<float> visibilityMap; // per-pixel surface visibility/normal score

	// data for resolution and score systems
	uint nViews; // number of best views retained per face during view selection
	CUDA::TDeviceBuffer<float> faceScores;       // per-face quality score for the current view
	CUDA::TDeviceBuffer<uint> resFace;           // per-face projected pixel count (resolution proxy) for the current view
	CUDA::TDeviceBuffer<int> bestViewsIdx;       // indices of the best views for each face (nViews entries per face)
	CUDA::TDeviceBuffer<float> bestViewsScore;   // scores of the best views for each face (nViews entries per face)
	CUDA::TDeviceMat<float> visibilityScores;    // per-pixel visibility scores accumulated across stacked view samples
	
	// image pyramid on device
	std::vector<CUDA::TDeviceMat<float>> laplacePyramidImages; // Laplacian pyramid levels of the current camera image on device
	std::vector<size_t> laplacePyramidSize;                    // byte size of each pyramid level device buffer

	// texture components on device
	uint stackSize = 5; // maximum number of view samples stacked per texture pixel
	CUDA::TDeviceMat<float> texAverage; // low-resolution per-pixel colour background average accumulated across all views
	CUDA::TDeviceMat<float> bufferCount; // per-pixel count of contributions to texAverage, used for normalisation
	// Laplacian pyramid texture stacks: one TDeviceMat per pyramid level
	// each mat stores stackSize*channels*texture_width*texture_height samples,
	// where successive slices along the first dimension correspond to different view samples
	std::vector<CUDA::TDeviceMat<float>> laplacePyramidTextures; 
	CUDA::TDeviceMat<int> idxStacks; // per-pixel view index of each sample slot in the texture stacks
};

// CURAST subclass that implements the texture synthesis pipeline for a single texture of the output texture atlas
class TextureCURAST : public CURAST {
public :	
	TextureCURAST(const SEACAVE::CUDA::CudaStreamSharedPtr& _ptrStream, const uint _stackSize, const uint _nViews, const uint nFaces) : CURAST(_ptrStream, _stackSize,_nViews,nFaces){};
	// project the mesh onto the image plane and build depth map, face map and visibility map
	void ProjectMesh();
	// upload a mesh chunk with UV coordinates to device, optionally skipping UV upload when loadUV is false
	void LoadMeshChunkOnDevice(CUDA::Point3* positions, CUDA::Point3i* faces, CUDA::Point2* uvs, std::vector<uint>& chunk, bool loadUV );
	// upload a mesh chunk without UV coordinates
	void LoadMeshChunkOnDevice(CUDA::Point3* positions, CUDA::Point3i* faces, std::vector<uint>& chunk) { 
		LoadMeshChunkOnDevice(positions, faces, nullptr, chunk, false);
	};
	// release device memory holding the current mesh chunk geometry and UV data
	void ClearMeshBuffer();
	// add the current view data into the texture stacks according to the outlier rejection process and add contribution to average low resolution color background
	void TextureRasterize();
	// fuse all stacked view samples across pyramid levels and collapse the Laplacian pyramid into the final texture
	void CollapseTexturePyramid();
private:
	CUDA::TDeviceBuffer<CUDA::Point2> bufferUVs; // per-vertex UV texture coordinates of the current mesh chunk on device
};
namespace CUDA {
	// compact device-side description of a mesh, grouping raw device pointers with the triangle count.
	struct DeviceMesh {
		CUDA::Point3* positions; // positions of vertices on the device
		CUDA::Point2* uvs; // texture coordinates of triangle vertices on the device (may be null for orthographic pipeline)
		int numTriangles; // total number of triangles in this mesh
	};
	// compact camera model bundling the three matrices needed for projection.
	struct Camera {
		CUDA::Point3 C; // camera centre in world coordinates
		CUDA::Matrix3 R; // rotation matrix (world to camera)
		CUDA::Matrix3 K; // intrinsic matrix (camera to image)
	};
	namespace CURAST {
		// project the mesh onto the perspective image plane and build depth map, face map and visibility map.
		void ProjectMesh(float* depthMap, int* faceMap, float* visibilityMap, Point3* positions,
			uint numTriangles, const CUDA::Camera camera, const Point2i imgSize, cudaStream_t stream);
		// compute and homogenize visibility accross plausible surface, and compute resolution and update best views for each face
		void UpdateBestViews(int* faceMap, float* visibilityMap, float* depthMap, float* bestViewsScore, uint* resFace, 
			int* bestViewsIdx, const uint numFaces, const Point2i imgSize, uint nViews, uint viewIdx, float fx, float fy, cudaStream_t stream);
		// build a custom image pyramid from input image with aggressive downsampling for the lower resolution levels
		void MakeCustomPyramid(std::vector<TDeviceMat<float>>& pyramidImages, const Point2i imgSize, const uint nLevels, cudaStream_t stream);
		// add the current view data into the texture stacks according to the outlier rejection process and add contribution to average low resolution color background.
		void TextureRasterize(std::vector<CUDA::TDeviceMat<float>>& texStacks, float* texAverage, float* bufferCount, const uint stackSize, float* visibilityScores, 
			const float* faceScores, const int* isBestView, const std::vector<CUDA::TDeviceMat<float>>& pyramidImage, int* idxStacks, const Point3* positions, 
			const Point2* texCoords, const uint numViewFaces, const uint numTriangles, const CUDA::Camera camera, uint viewIdx, cudaStream_t stream);
		void CollapseStacks(std::vector<CUDA::TDeviceMat<float>>& pyramidTex, float* texAverage, float* bufferCount, float* visibilityScores, 
			int* idxStacks, Point2i texSize, unsigned levelCount, uint stackSize, cudaStream_t stream);
		// compute value from stacked samples to build each pyramid level and collapse the pyramid into a single texture,
		// applying optional contrast, saturation, and detail enhancement corrections
		void CollapsePyramid(std::vector<CUDA::TDeviceMat<float>>& pyramidTexStack, Point2i texSize, unsigned levels, uint stackSize, cudaStream_t stream, float correctContrast = 1.f, float correctSaturation = 1.1f);

		// fill an array on device with a given value
		template <typename T>
		void Fill(T* data, T value, uint nElement, cudaStream_t stream);
		extern template void Fill<int>(int*, int, uint, cudaStream_t);
		extern template void Fill<float>(float*, float, uint, cudaStream_t);		
		// extract one image from a level of the stack
		template<typename T>
		void ExtractImgFromStack(const T* d_stack, T* d_output, Point2i imgSize, uint stackSize, int chanels, uint nStack, cudaStream_t stream);
	}/// CUDA::CURAST
}/// CUDA
}/// MVS
