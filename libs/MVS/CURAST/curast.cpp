#include "curast.h"
#include <cuda_runtime.h>
namespace MVS {

CURAST::CURAST(const SEACAVE::CUDA::CudaStreamSharedPtr& _ptrStream, const uint _stackSize, const uint _nViews, const uint nFaces) : 
	ptrStream(_ptrStream), stackSize(_stackSize), nViews(_nViews) {
	
	hBestViewsScore.resize(nFaces*_nViews);
	std::fill(hBestViewsScore.begin(), hBestViewsScore.end(), 0.f);
	hBestViewsIdx.resize(nFaces*_nViews);
	std::fill(hBestViewsIdx.begin(), hBestViewsIdx.end(), -1);
	hResFace.resize(nFaces*_nViews);
	std::fill(hResFace.begin(), hResFace.end(), 0);
}
// uploads per-face best-view scores, view indices, and resolution counts for the given
// mesh chunk from the host arrays to the corresponding GPU buffers,
// only the rows for faces in `chunk` are packed and transferred, keeping the transfer
// size proportional to the chunk rather than the full mesh
void CURAST::LoadBestViewsDataOnDevice( std::vector<uint>& chunk) {
	std::vector<float> cBestViewsScore(chunk.size()*nViews);
	std::vector<uint> cResFaces(chunk.size()*nViews);
	std::vector<int> cBestViewsIdx(chunk.size()*nViews);
	meshChunk = chunk;
	for (int i = 0; i < chunk.size(); i++) {
		uint faceIdx = chunk[i];
		std::copy(hBestViewsScore.begin() + faceIdx * nViews, hBestViewsScore.begin() + (faceIdx + 1) * nViews, cBestViewsScore.begin() + i * nViews);
		std::copy(hResFace.begin() + faceIdx * nViews, hResFace.begin() + (faceIdx + 1) * nViews, cResFaces.begin() + i * nViews);
		std::copy(hBestViewsIdx.begin() + faceIdx * nViews, hBestViewsIdx.begin() + (faceIdx+1) * nViews, cBestViewsIdx.begin() + i * nViews);
	}
	bestViewsScore.ReallocateToAtLeastSize(cBestViewsScore.size());
	bestViewsIdx.ReallocateToAtLeastSize(cBestViewsIdx.size());
	resFace.ReallocateToAtLeastSize(cResFaces.size());
	cudaMemcpy(bestViewsScore.GetDeviceData(), cBestViewsScore.data(), cBestViewsScore.size() * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(bestViewsIdx.GetDeviceData(), cBestViewsIdx.data(), cBestViewsIdx.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(resFace.GetDeviceData(), cResFaces.data(), cResFaces.size() * sizeof(uint), cudaMemcpyHostToDevice);
}
// downloads per-face best-view data (scores, view indices, resolution counts) from GPU
// back to host after a projection pass, results are scattered back into the global
// host arrays at the correct global face indices stored in meshChunk.
void CURAST::DownloadBestViewsScore() {
	uint nElem = meshChunk.size();
	std::vector<float> scores(nElem*nViews);
	std::vector<int> idxs(nElem*nViews);
	std::fill(idxs.begin(),idxs.end(),-1);
	std::vector<uint> res(nElem*nViews);
	cudaMemcpy(scores.data(), bestViewsScore.GetDeviceData(), nElem * nViews * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(idxs.data(), bestViewsIdx.GetDeviceData(), nElem * nViews * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(res.data(), resFace.GetDeviceData(), nElem * nViews * sizeof(uint), cudaMemcpyDeviceToHost);
	for ( int i = 0; i < nElem; i++ ) {
		uint faceIdx = meshChunk[i];
		std::copy(scores.begin() + nViews * i, scores.begin() + nViews * (i + 1), hBestViewsScore.begin() + faceIdx * nViews);
		std::copy(idxs.begin() + nViews * i, idxs.begin() + nViews * (i + 1), hBestViewsIdx.begin() + faceIdx * nViews);
		std::copy(res.begin() + nViews * i, res.begin() + nViews * (i + 1), hResFace.begin() + faceIdx * nViews);
}
}
void CURAST::InitializeTexturePyramidOnDevice(const cv::Size size,const int level) {
	laplacePyramidTextures.resize(level + 1);
	for (int i = 0; i <= level; i++) {
		laplacePyramidTextures[i].Resize(size, 3*stackSize);
		cudaMemset(laplacePyramidTextures[i].GetDeviceData(), 0, laplacePyramidTextures[i].SizeInBytes());
	}
	visibilityScores.Resize(size, stackSize);
	cudaMemset(visibilityScores.GetDeviceData(), 0, visibilityScores.SizeInBytes());
	idxStacks.Resize(size, stackSize);
	CUDA::CURAST::Fill(idxStacks.GetDeviceData(), -1, size.area()*stackSize, Stream());
	texAverage.Resize(size, 3);
	cudaMemset(texAverage.GetDeviceData(), 0, texAverage.SizeInBytes());
	bufferCount.Resize(size, 1);
	cudaMemset(bufferCount.GetDeviceData(), 0, bufferCount.SizeInBytes());
}
void CURAST::InitializeFaceMap(cv::Size imgSize, bool useVisibility) {
	faceMap.Resize(imgSize, 1); 
	depthMap.Resize(imgSize, 1);
	if (useVisibility) {
	visibilityMap.Resize(imgSize, 1);
	cudaMemset(visibilityMap.GetDeviceData(), 0, visibilityMap.SizeInBytes());
}
}
void CURAST::SetFacesForView(uint viewIdx, std::vector<uint>& faceList, uint nBest) {
	std::vector<int> _isBestView(faceList.size());
	std::vector<float> _facesScore(faceList.size());
	std::memset(_facesScore.data(),0,_facesScore.size()*sizeof(float));
	std::memset(_isBestView.data(),0,_isBestView.size()*sizeof(int));
	for(int i = 0; i < faceList.size(); i++) {
		uint faceIdx = faceList[i];
		for (int j = 0; j < nViews; j++) {
			int idx = faceIdx * nViews + j;
			int vIdx = hBestViewsIdx[idx];
			if ( viewIdx == vIdx ) {
				if (j < nBest) 
					_isBestView[i] = 1;
				_facesScore[i] = hBestViewsScore[idx];
				break;
			}
		}
}
	faceScores.ReallocateIfNotSize(_facesScore.size());
	faceScores.UploadFromHost(_facesScore.data(), _facesScore.size(), Stream());
	isBestView.ReallocateIfNotSize(_isBestView.size());
	isBestView.UploadFromHost(_isBestView.data(), _isBestView.size(), Stream());
}
void CURAST::SetFacesForView(uint _viewIdx, std::vector<uint>& faceList) {
	viewIdx = _viewIdx;
	std::vector<float> _facesScore(faceList.size());
	for(int i = 0; i < faceList.size(); i++) {
		for (int j = 0; j < nViews; j++) {
			uint faceIdx = faceList[i];
			int idx = faceIdx * nViews + j;
			int vIdx = hBestViewsIdx[idx];
			if ( _viewIdx == vIdx ) {
				_facesScore[i] = hBestViewsScore[idx];
				break;
			}
		}
	}
	faceScores.ReallocateIfNotSize(_facesScore.size());
	faceScores.UploadFromHost(_facesScore.data(), _facesScore.size(), Stream());
}
void CURAST::ResetPyramidTextureStacks() {
	for (int l = 0; l < laplacePyramidTextures.size(); l++) {
		cudaMemset(laplacePyramidTextures[l].GetDeviceData(), 0, laplacePyramidTextures[l].SizeInBytes());
		cudaMemset(visibilityScores.GetDeviceData(), 0, visibilityScores.SizeInBytes());
	}
}
void CURAST::SetCameraMatrices(const CUDA::Point3& C, const CUDA::Matrix3& R, const CUDA::Matrix3& K, const uint _viewIdx) {
	this->C = C;
	this->R = R;
	this->K = K;
	this->viewIdx = _viewIdx;
}
void CURAST::CopyVisibilityMapTexToHost(cv::Mat& visibilityMapHost) {
	CUDA::TDeviceMat<float> tmpVisibilityMap;
	Point2i dims = visibilityScores.CudaImageSize();
	tmpVisibilityMap.Resize(cv::Size(dims[0],dims[1]), 1);
	CUDA::CURAST::ExtractImgFromStack(visibilityScores.GetDeviceData(), tmpVisibilityMap.GetDeviceData(), dims, stackSize, 1, 0, Stream());
	tmpVisibilityMap.Download(visibilityMapHost, Stream());
}
void CURAST::CopyVisibilityMapToHost(cv::Mat& visibilityMapHost) {
	visibilityMap.Download(visibilityMapHost, Stream());
}
void CURAST::CopyFinalTextureToHost(cv::Mat& finalTextureHost, uint level, uint nStack) {
	CUDA::TDeviceMat<float> tmpTexture;
	Point2i texSize = laplacePyramidTextures[level].CudaImageSize();
	tmpTexture.Resize(texSize, 3);
	CUDA::CURAST::ExtractImgFromStack(laplacePyramidTextures[level].GetDeviceData(), tmpTexture.GetDeviceData(), texSize, stackSize, 3, nStack, Stream());
	tmpTexture.Download(finalTextureHost, Stream());
	tmpTexture.Resize(Point2i(0,0),1);
}
void CURAST::MakeImageCustomPyramid(const cv::Mat& inputImage, const uint nLevels) {
	laplacePyramidImages.clear();
	laplacePyramidImages.resize(nLevels);
	laplacePyramidImages[0].Resize(cv::Size(inputImage.cols, inputImage.rows), inputImage.channels());
	laplacePyramidImages[0].Upload(inputImage, Stream());
	CUDA_CHECK_LAST_ERROR;
	CUDA::CURAST::MakeCustomPyramid(laplacePyramidImages, CUDA::Point2i(inputImage.cols, inputImage.rows), nLevels, Stream());
	CUDA_CHECK_LAST_ERROR;
}
// projects the mesh chunk onto the current perspective view (using camera C/R/K),
// fills the depth map and face map on GPU, then calls UpdateBestViews to compare
// this view's resolution against stored per-face best-view data and update rankings,
// results are downloaded back to host via DownloadBestViewsScore.
void TextureCURAST::ProjectMesh() {
	CUDA::Camera camera;
	camera.C = C;
	camera.R = R;
	camera.K = K;
	CUDA::CURAST::ProjectMesh( depthMap.GetDeviceData(), faceMap.GetDeviceData(), visibilityMap.GetDeviceData(), bufferPositions.GetDeviceData(), 
		meshChunk.size(), camera, depthMap.CudaImageSize(), Stream());
	float fx = K(0,0);
	float fy = K(1,1);
	CUDA::CURAST::UpdateBestViews(faceMap.GetDeviceData(), visibilityMap.GetDeviceData(), depthMap.GetDeviceData(), bestViewsScore.GetDeviceData(), resFace.GetDeviceData(), bestViewsIdx.GetDeviceData(),
		meshChunk.size(), depthMap.CudaImageSize(), nViews, viewIdx, fx, fy, Stream());
	DownloadBestViewsScore();
}
void TextureCURAST::LoadMeshChunkOnDevice(CUDA::Point3* positions, CUDA::Point3i* faces, CUDA::Point2* uvs, std::vector<uint>& chunk, bool loadUV) {
	std::vector<CUDA::Point3> pos;
	std::vector<CUDA::Point2> tex;
	meshChunk = chunk;
	for (int i = 0; i < chunk.size(); i++) {
		uint faceIdx = chunk[i];
		CUDA::Point3i face = faces[faceIdx];
		for (int v = 0; v < 3; v++) {
			pos.push_back(positions[face(v)]);
			if (loadUV) {
				tex.push_back(uvs[faceIdx * 3 + v]);
			}
		}
	}
	bufferPositions.ReallocateToAtLeastSize(pos.size());
	bufferPositions.UploadFromHost(pos.data(), pos.size(), Stream());
	if (loadUV) {
		bufferUVs.ReallocateToAtLeastSize(tex.size());
		bufferUVs.UploadFromHost(tex.data(), tex.size(), Stream());
	}
}
void TextureCURAST::TextureRasterize() {
	CUDA::Camera camera;
	camera.C = C;
	camera.R = R;
	camera.K = K;
	CUDA::CURAST::TextureRasterize(laplacePyramidTextures, texAverage.GetDeviceData(), bufferCount.GetDeviceData(), stackSize, visibilityScores.GetDeviceData(), faceScores.GetDeviceData(),
		isBestView.GetDeviceData(), laplacePyramidImages, idxStacks.GetDeviceData(), bufferPositions.GetDeviceData(), bufferUVs.GetDeviceData(), isBestView.NumElements(),
		meshChunk.size(), camera, viewIdx, Stream());
}
void TextureCURAST::CollapseTexturePyramid() {
	CUDA::CURAST::CollapseStacks(laplacePyramidTextures, texAverage.GetDeviceData(), bufferCount.GetDeviceData(), visibilityScores.GetDeviceData(), idxStacks.GetDeviceData(), laplacePyramidTextures[0].CudaImageSize(), laplacePyramidTextures.size(), stackSize, Stream());
	CUDA::CURAST::CollapsePyramid(laplacePyramidTextures, laplacePyramidTextures[0].CudaImageSize(), laplacePyramidTextures.size(), stackSize, Stream());
}
void TextureCURAST::ClearMeshBuffer() {
	bufferPositions.ReallocateIfNotSize(0);
	bufferUVs.ReallocateIfNotSize(0);
	bestViewsIdx.ReallocateIfNotSize(0);
	bestViewsScore.ReallocateIfNotSize(0);
	resFace.ReallocateIfNotSize(0);
}
}/// MVS

