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
	// mark which faces are visible from the given view and upload the scores of the view for those faces
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
	// stores one float3 colour per pyramid level (up to 3 levels) for a single texture pixel,
	struct PyrPix {
		public:
		// default constructor: initialise all pyramid level colours to black (0, 0, 0)
		__host__ __device__ PyrPix() { for (uint i = 0; i < 3; i++) p[i] = Point3(0.f,0.f,0.f);}
		// compute and return the summed colour across all pyramid levels.
		__host__ __device__ Point3 GetColor() {
			Point3 C(0.f, 0.f, 0.f);
			for (int i = 0; i < 3; i++) {
				C += p[i];
			}
			return C;
		}
		Point3 p[3]; // colour data for each pyramid level (index 0 = finest, 2 = coarsest)
		
		__host__ __device__
		PyrPix operator+(const PyrPix& other) const {
			PyrPix result;
			for (int i = 0; i < 3; i++) {
				result.p[i] = this->p[i] + other.p[i];
			}
			return result;
		}
		__host__ __device__
		PyrPix& operator+=(const PyrPix& other) {
			for (int i = 0; i < 3; i++) {
				this->p[i] += other.p[i];
			}
			return *this;
		}
		__host__ __device__
		PyrPix operator*(float scalar) const {
			PyrPix result;
			for (int i = 0; i < 3; i++) {
				result.p[i] = this->p[i] * scalar;
			}
			return result;
		}
		__host__ __device__
		PyrPix operator/(float scalar) const {
			PyrPix result;
			for (int i = 0; i < 3; i++) {
				result.p[i] = this->p[i] / scalar;
			}
			return result;
		}
	};

	// 2D integer pixel coordinate used as a seed point for the Jump Flood Algorithm (JFA).
	struct Seed
	{
		int x; 
		int y;

		__host__ __device__ Seed() :
			x(-1), y(-1) {}
		__host__ __device__ Seed(int x, int y) : 
			x(x), y(y) {}
	};
} 

// pre-allocated GPU workspace for the orthographic texture collapse pass 
struct CollapseWorkspace {
	CUDA::TDeviceBuffer<float*>        texStacks;       // device pointers to each pyramid-level texture stack, one pointer per level
	CUDA::TDeviceBuffer<CUDA::PyrPix>  finalPyr;        // final collapsed pyramid colour for each pixel
	CUDA::TDeviceBuffer<int>           bestViewIdxMap;  // index of the selected best view for each pixel
	CUDA::TDeviceBuffer<CUDA::Seed>    seedMap1;        // primary JFA seed map: nearest valid seed coordinate for each pixel
	CUDA::TDeviceBuffer<CUDA::Seed>    seedMap2;        // secondary JFA seed map used as ping-pong buffer during propagation
	CUDA::TDeviceBuffer<CUDA::PyrPix>  savedResult;     // saved collapsed result before hole-filling, used for blending
	CUDA::TDeviceBuffer<int>           tagsMap;         // per-pixel integer tag to indentify empty areas in final pyr
	CUDA::TDeviceBuffer<int>           tmpInt;          // general-purpose temporary integer buffer
	CUDA::TDeviceBuffer<float>         distMap;         // per-pixel distance map used for blending
	CUDA::TDeviceBuffer<float>         distanceMap;     // per-pixel distance map used for blending
	CUDA::TDeviceBuffer<CUDA::Point3>  average;         // per-pixel colour mean across view samples 	
	CUDA::TDeviceBuffer<CUDA::Matrix3> covariance;      // per-pixel colour covariance matrix across view samples (for Mahalanobis distance)
	CUDA::TDeviceBuffer<CUDA::PyrPix>  saveTmp;         // secondary temporary pyramid pixel buffer 	
	CUDA::TDeviceBuffer<char>          mask1;           // primary per-pixel binary mask 
	CUDA::TDeviceBuffer<char>          mask2;           // secondary per-pixel binary mask used as ping-pong buffer
	CUDA::TDeviceBuffer<char>          mask3;			// tertiary per-pixel binary mask used for hole-filling
	CUDA::TDeviceBuffer<CUDA::PyrPix>  bestViewPyr;     // pyramid colour of the single best view for each pixel
	CUDA::TDeviceBuffer<int>           keys;            // per-pixel sort keys used when ordering candidate views by score
	CUDA::TDeviceBuffer<float>         patchDist;       // per-pixel patch-level distance metric between candidate views
	static constexpr uint kMaxBestViews = 5; // maximum number of candidate best views considered per pixel during collapse

	// allocate all workspace buffers 
	// only reallocates if the requested size exceeds its current capacity.
	void Init(uint nPixels, uint levelCount) {
		texStacks.ReallocateToAtLeastSize(levelCount);
		finalPyr.ReallocateToAtLeastSize(nPixels);
		bestViewIdxMap.ReallocateToAtLeastSize(nPixels);
		seedMap1.ReallocateToAtLeastSize(nPixels);
		seedMap2.ReallocateToAtLeastSize(nPixels);
		savedResult.ReallocateToAtLeastSize(nPixels);
		tagsMap.ReallocateToAtLeastSize(nPixels);
		tmpInt.ReallocateToAtLeastSize(nPixels);
		distMap.ReallocateToAtLeastSize(nPixels);
		distanceMap.ReallocateToAtLeastSize(nPixels);
		average.ReallocateToAtLeastSize(nPixels);
		covariance.ReallocateToAtLeastSize(nPixels);
		saveTmp.ReallocateToAtLeastSize(nPixels);
		mask1.ReallocateToAtLeastSize(nPixels);
		mask2.ReallocateToAtLeastSize(nPixels);
		mask3.ReallocateToAtLeastSize(nPixels);
		bestViewPyr.ReallocateToAtLeastSize(nPixels);
		keys.ReallocateToAtLeastSize(nPixels);
		patchDist.ReallocateToAtLeastSize(nPixels);

	}
};

class OrthoCURAST : public CURAST {
public: 
	OrthoCURAST(const SEACAVE::CUDA::CudaStreamSharedPtr& _ptrStream, const uint _stackSize, const uint _nViews, const uint nFaces) : CURAST(_ptrStream, _stackSize,_nViews,nFaces){
	};
	// allocate or resize per-tile pyramid texture stack device buffers to the given image dimensions, pyramid level count, and stack depth
	void InitializeOrthoViewPyramidOnDevice(const cv::Size size, const int level, int stackSize);
	// project the mesh onto the orthographic image plane for the current camera view and build depth map, face map and normal map
	void ProjectMeshForOrtho();
	// compute the per-pixel world-space normal map for the given tile origin and size under orthographic projection
	void GenerateNormalOrthoMap(float2 tileOrigne, float2 tileSize, cv::Size tileRes);
	// rasterize the current view's Laplacian image pyramid into the per-tile texture stacks for the given tile extent
	void OrthoRasterize(uint viewIdx, float2 tileOrigin, float2 tileSize);
	// collapse the per-tile Laplacian pyramid tile stacks into a single final texture using outlier rejection and hole-filling
	void CollapseTexturePyramid();
	// upload a mesh chunk (positions and face indices only, no UVs) to device for orthographic rasterization
	void LoadMeshChunkOnDevice(CUDA::Point3* positions, CUDA::Point3i* faces, std::vector<uint>& chunk);
	// release device memory holding the current mesh chunk geometry.
	void ClearMeshBuffer();

private:
	int maxStackSize = 0;                   // store the maximum of stack allocated so far (use to only reallocate if upper than maxStackSize)
	CollapseWorkspace collapseWorkspace;    // pre-allocated GPU workspace reused across all tile collapse passes
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
		// project the mesh under orthographic projection and build depth map, face map and normal map.
		void ProjectMeshOrtho(float* depthMap, int* faceMap, float* normalMap, Point3* positions, 
			uint numTriangles, const CUDA::Camera camera, const Point2i imgSize, cudaStream_t stream);
		// compute the per-pixel world-space surface normal map for an orthographic tile defined by tileSize and tileRes.
		void GetNormalOrthoMap(float* normalMap, Point3* positions, uint numTriangles, const Point2 tileSize,
			const Point2i tileRes, cudaStream_t stream, Point2 origine);
		// compute and homogenize visibility across plausible surface, and compute resolution and update best views for each face (perspective).
		void UpdateBestViews(int* faceMap, float* visibilityMap, float* depthMap, float* bestViewsScore, uint* resFace, 
			int* bestViewsIdx, const uint numFaces, const Point2i imgSize, uint nViews, uint viewIdx, float fx, float fy, cudaStream_t stream);
		// compute and homogenize visibility across plausible surface, and update best views per face for the orthographic pipeline; also tracks per-view maximum score in maxScorePerView.
		void UpdateBestViewsOrtho(int* faceMap, float* visibilityMap, float* depthMap, float* bestViewsScore, uint* resFace, 
			int* bestViewsIdx, const uint numFaces, const Point2i imgSize, uint nViews, uint viewIdx, float fx, float fy, cudaStream_t stream);
		// build a custom image pyramid from input image with aggressive downsampling for the lower resolution levels.
		void MakeCustomPyramid(std::vector<TDeviceMat<float>>& pyramidImages, const Point2i imgSize, const uint nLevels, cudaStream_t stream);
		// add the current view data into the texture stacks according to the outlier rejection process and add contribution to average low resolution color background.
		void TextureRasterize(std::vector<CUDA::TDeviceMat<float>>& texStacks, float* texAverage, float* bufferCount, const uint stackSize, float* visibilityScores, 
			const float* faceScores, const int* isBestView, const std::vector<CUDA::TDeviceMat<float>>& pyramidImage, int* idxStacks, const Point3* positions, 
			const Point2* texCoords, const uint numViewFaces, const uint numTriangles, const CUDA::Camera camera, uint viewIdx, cudaStream_t stream);
		// rasterize the current view's Laplacian pyramid into the orthographic tile texture stacks for the given tile origin and size.
		void OrthoRasterize(std::vector<CUDA::TDeviceMat<float>>& texStacks, const uint stackSize, float* visibilityScores, const float* faceScores, 
			const std::vector<CUDA::TDeviceMat<float>>& pyramidImage, const Point3* positions, const uint numViewFaces, 
			const uint numTriangles, const CUDA::Camera camera, const uint viewIdx, const Point2 tileOrigin, const Point2 tileSize, cudaStream_t stream);
		// collapse the multi-view Laplacian texture stacks (perspective pipeline) into a single blended texture per pyramid level using the accumulated average and count buffers
		void CollapseStacks(std::vector<CUDA::TDeviceMat<float>>& pyramidTex, float* texAverage, float* bufferCount, float* visibilityScores, 
			int* idxStacks, Point2i texSize, unsigned levelCount, uint stackSize, cudaStream_t stream);
		// collapse the multi-view Laplacian texture stacks (orthographic pipeline) into a single texture per level using Mahalanobis outlier rejection and best view blending
		void CollapseStacksOrtho(std::vector<CUDA::TDeviceMat<float>>& pyramidTex, float* visibilityScores, float* normalMap, Point2i texSize, unsigned levelCount, uint stackSize, ::MVS::CollapseWorkspace& workspace, cudaStream_t stream);
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
