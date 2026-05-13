// TextureMeshCuda 
typedef cv::Rect2f RectType;

// Keep the port isolated from the CPU texturing rework by mapping the old
// rect-packer calls onto the current atlas packer API.
using RectsBinPack = AtlasPacker;
struct MaxRectsBinPack : public AtlasPacker {
	enum FreeRectChoiceHeuristic {
		kDefaultHeuristic = 0
	};
	MaxRectsBinPack(int width, int height)
		: AtlasPacker(width, height) {}
	RectsBinPack::RectWIdxArr Insert(RectsBinPack::RectWIdxArr& rects, FreeRectChoiceHeuristic) {
		return AtlasPacker::Insert(rects);
	}
};

inline float TriangleTexArea(const Mesh::TexCoord& v0, const Mesh::TexCoord& v1, const Mesh::TexCoord& v2) {
	return std::fabs(0.5f * (v1 - v0).cross(v2 - v0));
}

// morton helpers
uint64_t ExpandBits(uint64_t v) {
	v = (v | (v << 16)) & 0x030000FF;
	v = (v | (v << 8))  & 0x0300F00F;
	v = (v | (v << 4))  & 0x030C30C3;
	v = (v | (v << 2))  & 0x09249249;
	return v;
}
uint64_t ExpandBits2D(uint64_t v)
{
	uint64_t x = v;
	x = (x | (x << 16)) & 0x0000FFFF0000FFFFULL;
	x = (x | (x << 8))  & 0x00FF00FF00FF00FFULL;
	x = (x | (x << 4))  & 0x0F0F0F0F0F0F0F0FULL;
	x = (x | (x << 2))  & 0x3333333333333333ULL;
	x = (x | (x << 1))  & 0x5555555555555555ULL; // insertion d'un zero
	return x;
}

uint64 Morton3D(uint32_t x, uint32_t y, uint32_t z) {
	return (ExpandBits(x) << 2) | (ExpandBits(y) << 1) | ExpandBits(z);
}

uint64 Morton2D(uint32_t x, uint32_t y) {
	return (ExpandBits2D(x) << 1) | ExpandBits2D(y);
}	

bool Scene::TextureMeshCuda(unsigned _maxTexRes, unsigned _maxImgRes, bool rePack, bool reParametrize) {
	// parameters
	const uint maxTextRes = _maxTexRes;			// maximal textures aresolution
	const uint maxImgRes = _maxImgRes;			// maximal images resolution
	const uint nKeepViews = images.size();  // number of views to consider per face for view selection
	const uint stackSize = 5;				// number of samples kept per texel for outlier removal
	const uint nBests = 10;					// minimal rank to be considered as best view for a face
	const uint levels = 2;					// number of level for the pyramid (in addition to the base level)
	const uint pad = 5;						// padding around each patch in the atlas
	const uint gutterSize = 10;				// gutter size between patches in the atlas
	const float maxBatchArea = maxTextRes > 0 ? float(maxTextRes) * float(maxTextRes) : std::numeric_limits<float>::infinity();

	// initial image size and view size
	const cv::Size viewSize = images[0].GetSize();
	images[0].ReloadImage(maxImgRes);
	cv::Size imageSize = images[0].GetSize();
	images[0].image.release();
	images[0].ResizeImage(viewSize.width);
	if (maxImgRes == 0)
		imageSize = viewSize;
	VERBOSE(" Texture generation with image size (%d,%d) and max texture size : (%d,%d)",imageSize.width, imageSize.height,maxTextRes,maxTextRes);
	
	SEACAVE::CUDA::CudaStreamSharedPtr ptrStream(SEACAVE::CUDA::CreateSharedStream());
	TextureCURAST cudaRasterizer(ptrStream, stackSize, images.size(), mesh.faces.size());
	
	Util::Progress progress(_T("Projecting mesh to views"), images.size());
	if (VERBOSITY_LEVEL < 3)
		GET_LOGCONSOLE().Pause();
	
	// generate best views per face and resolution scores for atlas generation
	std::vector<uint> listViews; // keep view seeing the mesh
	{
		Mesh::Octree octree;
		Mesh::FacesInserter::CreateOctree(octree, mesh);
		CUDA::Point3* posPtr = reinterpret_cast<CUDA::Point3*>(mesh.vertices.GetData());
		CUDA::Point3i* facePtr = reinterpret_cast<CUDA::Point3i*>(mesh.faces.GetData());
		cudaRasterizer.InitializeFaceMap(imageSize);
	for (uint viewIdx = 0; viewIdx < images.size(); ++viewIdx)
	{	
		if (VERBOSITY_LEVEL < 3)
			progress.display(viewIdx+1);
		else 
			VERBOSE("Project mesh on view %u / %u", viewIdx, images.size());
			// get faces in the view frustum
			Mesh::FaceIdxArr meshChunk;
		MVS::Image& view = images[viewIdx];
			{
				if (!view.IsValid())
					continue;
				const TFrustum<float,5> frustum(Matrix3x4f(view.camera.P), (float)view.width, (float)view.height);
				Mesh::FacesInserter inserter(meshChunk);
				octree.Traverse(frustum, inserter);
			}
			if (meshChunk.size() == 0) continue;
			std::vector<uint> facesIdx(meshChunk.data(), meshChunk.data() + meshChunk.size());
			// load mesh chunk relative to the view
			cudaRasterizer.LoadMeshChunkOnDevice(posPtr,facePtr,facesIdx);
			cudaRasterizer.LoadBestViewsDataOnDevice(facesIdx);	

		const Camera camera = view.camera.GetScaled(viewSize,imageSize);
		CUDA::Matrix3 R;
		CUDA::Matrix3 K;
		CUDA::Point3 C;
		for (int r = 0; r < 3; ++r) {
			C[r] = camera.C[r];
			for (int c = 0; c < 3; ++c) {
				R(r, c) = camera.R(r, c);
				K(r, c) = camera.K(r, c);
			}
		}
		cudaRasterizer.SetCameraMatrices(C, R, K, viewIdx);
		cudaRasterizer.ProjectMesh();
			if (VERBOSITY_LEVEL > 3) {
				cv::Mat visibilityMap;
				cudaRasterizer.CopyVisibilityMapToHost(visibilityMap);
				double minVal, maxVal;
				cv::Mat visibilityMapNorm;
				cv::minMaxLoc(visibilityMap, &minVal, &maxVal);
				cv::Scalar mean = cv::mean(visibilityMap, visibilityMap != 0);
				VERBOSE("View %u: visibility map min = %f, max = %f, mean = %f\n", viewIdx, minVal, maxVal, mean[0]);
				visibilityMap.convertTo(visibilityMapNorm, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
				cv::imwrite(MAKE_PATH("visibility_map_view" + String::ToString(viewIdx) + ".png"), visibilityMapNorm);
			}
		}
	}
	progress.close();
	if (VERBOSITY_LEVEL < 3)
		GET_LOGCONSOLE().Play();
	// storage for best views by face
	const std::vector<int>& faceViewsIdx = cudaRasterizer.GetBestViewIdxRef();
	const std::vector<float>& faceViewsScore = cudaRasterizer.GetBestViewScore();
	const std::vector<uint>& faceViewsRes = cudaRasterizer.GetResFaceRef();

	std::vector<std::vector<uint>> patches;
	const String pathPatches = MAKE_PATH("patches.txt");
	if (mesh.faceTexcoords.empty() || !std::filesystem::exists(pathPatches.c_str()) || reParametrize) {
		// parametrization
		{
			VERBOSE("Mesh parametrization (xatlas) starting on %u verts, %u faces",
				mesh.vertices.size(), mesh.faces.size());
			TD_TIMER_STARTD();
			mesh.faceTexcoords.clear();
			xatlas::Atlas* atlas = xatlas::Create();
			xatlas::MeshDecl meshDecl;
			meshDecl.vertexCount = mesh.vertices.size();
			meshDecl.vertexPositionData = mesh.vertices.GetData();
			meshDecl.vertexPositionStride = sizeof(SEACAVE::TPoint3<float>);
			meshDecl.indexCount = mesh.faces.size() * 3;
			meshDecl.indexData = mesh.faces.GetData();
			meshDecl.indexFormat = xatlas::IndexFormat::UInt32;
			if (xatlas::AddMesh(atlas, meshDecl) != xatlas::AddMeshError::Success) {
				 ABORT("Error: xatlas::AddMesh failed");
				return EXIT_FAILURE;
			}
			VERBOSE("xatlas: AddMesh done @%s; calling ComputeCharts...", TD_TIMER_GET_FMT().c_str());
			xatlas::ChartOptions chartOptions = xatlas::ChartOptions();
			xatlas::ComputeCharts(atlas, chartOptions);
			VERBOSE("xatlas: ComputeCharts done @%s (%u charts); calling PackCharts...",
				TD_TIMER_GET_FMT().c_str(), atlas->chartCount);
			xatlas::PackOptions packOptions = xatlas::PackOptions();
			xatlas::PackCharts(atlas, packOptions);
			VERBOSE("xatlas: PackCharts done @%s (%u atlases, %ux%u resolution)",
				TD_TIMER_GET_FMT().c_str(), atlas->atlasCount, atlas->width, atlas->height);
			const xatlas::Mesh& atlasMesh = atlas->meshes[0];
			mesh.faceTexcoords.resize(mesh.faces.size() * 3);
			for (int i = 0; i < mesh.faces.size(); ++i) {
				for (int j = 0; j < 3; ++j) {
					const xatlas::Vertex& v = atlasMesh.vertexArray[atlasMesh.indexArray[i * 3 + j]];
					mesh.faceTexcoords[i * 3 + j] = MVS::Mesh::TexCoord(v.uv[0], v.uv[1]);
				}
			}
			// store charts as patches
			patches.resize(atlas->chartCount);
			for (int i = 0; i < atlas->chartCount; ++i) {
				uint count = atlasMesh.chartArray[i].faceCount;
				uint* faceArr = atlasMesh.chartArray[i].faceArray;
				std::vector<uint> patch(faceArr, faceArr + count);
				patches[i] = patch;
			}
			xatlas::Destroy(atlas);
			// save patches to patches.txt
			std::ofstream ofs(pathPatches);
			for (const auto& patch : patches) {
				ofs << patch.size();
				for (uint v : patch) ofs << " " << v;
				ofs << "\n";
			}
		}
	} else {
		 //load patches from patches.txt
		std::ifstream ifs(pathPatches);
		if (!ifs)
			DEBUG("Failed to open %s", pathPatches.c_str());
		patches.clear();
		std::string line;
		while (std::getline(ifs, line)) {
			std::istringstream iss(line);
			uint n;
			if (!(iss >> n)) continue;
			std::vector<uint> patch(n);
			for (uint i = 0; i < n; i++) {
				if (!(iss >> patch[i])) {
					DEBUG("Failed to read patch data");
					break;
				}
			}
			patches.push_back(std::move(patch));
		}
	}
	// store resolution scores per face
	std::vector<uint> faceResScores(mesh.faces.size(), 0);
	{
		for (int i = 0; i < mesh.faces.size(); i++) {
			for (int j = 0; j < images.size(); j++) {
				uint pixs = faceViewsRes[i*images.size() + j];
				if (pixs > faceResScores[i])
					faceResScores[i] = pixs;
			}
		}
	}

	// packing patches into textures
	if (mesh.texturesDiffuse.empty() || rePack || reParametrize) {
		
		VERBOSE("Packing patches into  textures :Number of patches: %lu", patches.size());
		mesh.faceTexindices.clear();
		mesh.faceTexindices.resize(mesh.faces.size());
		std::vector<uint> faceTextIdx(mesh.faces.size(), 0);
		
		std::vector<float> patchPixs(patches.size(), 0.f);
		std::vector<float> patchAreas(patches.size(), 0.f);
		std::vector<RectType> patchRects(patches.size());
					
		// compute patch resolution area score, bounding boxe and morton order
		std::vector<uint> patchIndices(patches.size()); {
			std::vector<uint64_t> patchMortonCodes(patches.size());
			AABB3f totalBBox = mesh.GetAABB();
			for (int i = 0; i<patches.size(); i++) {
				AABB2f boundingBox(true);
				const uint patchIdx = i;
				Point3f pos(0.,0.,0.);
				for (int k = 0; k<patches[patchIdx].size(); k++) {
					const int faceIdx = patches[patchIdx][k];
					float facePixs = float(faceResScores[faceIdx]); // resolution score is the number of pixels the face project on the image
					const Point3f facePos = mesh.vertices[mesh.faces[faceIdx][0]];
					pos += facePos; 
					patchPixs[patchIdx] += facePixs;
					const Mesh::TexCoord* textCoord = &mesh.faceTexcoords[faceIdx * 3];
					patchAreas[patchIdx] += TriangleTexArea(textCoord[0], textCoord[1], textCoord[2]);
				
					for (int j = 0; j < 3; ++j) {
							boundingBox.InsertFull(Point2f(textCoord[j].x, textCoord[j].y));
					}
				}
				pos /= (float)(patches[patchIdx].size());
				const float scale = (1 << 21) - 1;
				float maxDim =(float)MAX(totalBBox.GetSize().x(), MAX(totalBBox.GetSize().y(), totalBBox.GetSize().z()));
				float minDim =(float)MIN(totalBBox.GetSize().x(), MIN(totalBBox.GetSize().y(), totalBBox.GetSize().z()));
				if (maxDim/(float)minDim > 5) { // if one dimension is much smaller we order in 2D as for a plane
					uint32_t a, b;
					if (minDim == totalBBox.GetSize().x()) {
						a = ((pos.y - totalBBox.ptMin.y()) / maxDim) * scale;
						b = ((pos.z - totalBBox.ptMin.z()) / maxDim) * scale;
					} else if (minDim == totalBBox.GetSize().y()) {
						a = ((pos.x - totalBBox.ptMin.x()) / maxDim) * scale;
						b = ((pos.z - totalBBox.ptMin.z()) / maxDim) * scale;
					} else {
						a = ((pos.x - totalBBox.ptMin.x()) / maxDim) * scale;
						b = ((pos.y - totalBBox.ptMin.y()) / maxDim) * scale;
					}
					patchMortonCodes[patchIdx] = Morton2D(a, b);	

				} else { 
					uint32_t x = ((pos.x - totalBBox.ptMin.x()) / maxDim) * scale;
					uint32_t y = ((pos.y - totalBBox.ptMin.y()) / maxDim) * scale;
					uint32_t z = ((pos.z - totalBBox.ptMin.z()) / maxDim) * scale;
					patchMortonCodes[patchIdx] = Morton3D(x, y, z);
				}
				patchIndices[patchIdx] = patchIdx;

				patchRects[patchIdx].x = boundingBox.ptMin.x() - pad;
				patchRects[patchIdx].y = boundingBox.ptMin.y() - pad;
				patchRects[patchIdx].width = boundingBox.ptMax.x() - boundingBox.ptMin.x() + pad;
				patchRects[patchIdx].height = boundingBox.ptMax.y() - boundingBox.ptMin.y() + pad;
			}
			// order patches indices by morton code of their position
			std::sort(patchIndices.begin(), patchIndices.end(),
				[&](int a, int b){
				return patchMortonCodes[a] < patchMortonCodes[b];
			});
		}
		for (int i = 0; i<patches.size(); i++) {
			const int patchIdx = i;
			float scale = 0;
			if (patchAreas[patchIdx] != 0. )
				scale = sqrtf(patchPixs[patchIdx]/patchAreas[patchIdx]); // scaling to target resolution is approximatively the sqrt of the targeted number of pixels on the current area
			else 
				scale = 0.;

			patchRects[patchIdx].x *= scale;
			patchRects[patchIdx].y *= scale;
			patchRects[patchIdx].width *= scale;
			patchRects[patchIdx].height *= scale;
			patchRects[patchIdx].width += 5; // add border
			patchRects[patchIdx].height += 5;

			// clamp to max texture resolution
			if (maxTextRes > 0) {
				float maxRectDim = MAX(patchRects[patchIdx].width, patchRects[patchIdx].height);
				if (maxRectDim > maxTextRes) {
					float s;
					if (patchRects[patchIdx].width > patchRects[patchIdx].height)
						s = (float)(maxTextRes-20) / patchRects[patchIdx].width;
					else
						s = (float)(maxTextRes-20) / patchRects[patchIdx].height;
					patchRects[patchIdx].width *= s;
					patchRects[patchIdx].height *= s;
					patchRects[patchIdx].x *= s;
					patchRects[patchIdx].y *= s;
					scale *= s;
				}
			}
			
			// rescale uv coordinates
			for (int k = 0; k < patches[patchIdx].size(); k++) {
				const int faceIdx = patches[patchIdx][k];
				Mesh::TexCoord* textCoord = &mesh.faceTexcoords[faceIdx * 3];
				for (int j = 0; j < 3; ++j) {
					textCoord[j].x = (textCoord[j].x * scale) - patchRects[patchIdx].x;
					textCoord[j].y = (textCoord[j].y * scale) - patchRects[patchIdx].y;
					ASSERT(textCoord[j].x >= 0 && textCoord[j].x <= patchRects[patchIdx].width+0.01);
					ASSERT(textCoord[j].y >= 0 && textCoord[j].y <= patchRects[patchIdx].height+0.01);
				}
			}
		}
		
		// prepare rects to be packed
		RectsBinPack::RectWIdxArr unplacedRects(patches.size());
		for(int i = 0; i<patchRects.size(); i++) {
			uint pIdx = patchIndices[i];
			RectsBinPack::Rect rect;
			rect.x = FLOOR2INT(patchRects[pIdx].x);
			rect.y = FLOOR2INT(patchRects[pIdx].y);
			rect.width = CEIL2INT(patchRects[pIdx].width);
			rect.height = CEIL2INT(patchRects[pIdx].height);
			unplacedRects[i] = {rect, (unsigned)pIdx};
		}
		// make a first batch to be placed
		RectsBinPack::RectWIdxArr batchRects; { 
			float totalArea = 0.f;
			while (totalArea < maxBatchArea && !unplacedRects.empty()){
				batchRects.emplace_back(unplacedRects[0]);
				totalArea += unplacedRects[0].rect.width * unplacedRects[0].rect.height;
				unplacedRects.RemoveAt(0);
				if (totalArea > maxBatchArea)
					break;
			}
		}
		
		mesh.texturesDiffuse.clear();
		CLISTDEF2IDX(RectsBinPack::RectWIdxArr, TexIndex) placedRects;
		{
			int nRectPackingHeuristic = 0;
			const unsigned typeRectsBinPack(nRectPackingHeuristic/100);
			const unsigned typeSplit((nRectPackingHeuristic-typeRectsBinPack*100)/10);
			const unsigned typeHeuristic(nRectPackingHeuristic%10);
			int textureSize = 0;
			while (!batchRects.empty()) {
				TD_TIMER_STARTD();
				if (textureSize == 0) {
					textureSize = RectsBinPack::ComputeTextureSize(batchRects);
					if ( maxTextRes > 0 && textureSize > maxTextRes)
						textureSize = maxTextRes;
				}

				RectsBinPack::RectWIdxArr newPlacedRects;
				MaxRectsBinPack pack(textureSize, textureSize);
				newPlacedRects = pack.Insert(batchRects, (MaxRectsBinPack::FreeRectChoiceHeuristic)0);
			
				if (textureSize == maxTextRes || batchRects.empty()) {
					// create texture image
					placedRects.emplace_back(std::move(newPlacedRects));
					Pixel8U colEmpty(0, 0, 0);
					mesh.texturesDiffuse.emplace_back(textureSize, textureSize).setTo(cv::Scalar(colEmpty.b, colEmpty.g, colEmpty.r));
					textureSize = 0;
						if (!unplacedRects.empty()) {
							// prepare next batch
							float totalArea = 0.f;
							for (int i = 0; i < batchRects.size(); i++)
								totalArea += batchRects[i].rect.width * batchRects[i].rect.height;
							while (totalArea < maxBatchArea && !unplacedRects.empty()){
								batchRects.emplace_back(unplacedRects[0]);
								totalArea += unplacedRects[0].rect.width * unplacedRects[0].rect.height;
								unplacedRects.RemoveAt(0);
								if (totalArea > maxBatchArea)
									break;
							}
						}
				} else {
					// try again with a bigger texture
					textureSize *= 2;
					if (maxTextRes > 0)
						textureSize = MIN(textureSize, (int)maxTextRes);
					batchRects.JoinRemove(newPlacedRects);
				}
			}
		}
		for (int i = 0; i < placedRects.size(); ++i) {
			for (int j = 0; j < placedRects[i].size(); ++j) {
				const TexIndex idxTexture((TexIndex)i);
				const uint32_t idxPlacedPatch((uint32_t)j);
				const uint patchIdx = placedRects[idxTexture][idxPlacedPatch].patchIdx;
				const RectsBinPack::Rect& rect = placedRects[idxTexture][idxPlacedPatch].rect;
				for (const uint faceIdx : patches[patchIdx]) {
					Mesh::TexCoord* textCoord = &mesh.faceTexcoords[faceIdx * 3];
					mesh.faceTexindices[faceIdx] = idxTexture;
					for (int j = 0; j < 3; ++j) {
						if (rect.width == CEIL2INT(patchRects[patchIdx].height) && 
							rect.height == CEIL2INT(patchRects[patchIdx].width)) { // if patch was rotated swap uv coords
							std::swap(textCoord[j].x, textCoord[j].y);
						}
						Mesh::TexCoord& texcoord = textCoord[j];
						texcoord[0] = texcoord[0] + rect.x;
						texcoord[1] = texcoord[1] + rect.y;
						ASSERT(texcoord[0] >= rect.x && texcoord[0] <= rect.x + rect.width+0.01);
						ASSERT(texcoord[1] >= rect.y && texcoord[1] <= rect.y + rect.height+0.01);
					}
				}
			}
		}
	}

	VERBOSE("Number of textures : %lu", mesh.texturesDiffuse.size());
	if (VERBOSITY_LEVEL > 1) {
		VERBOSE("Texture size : ");
		for (uint t = 0; t < mesh.texturesDiffuse.size(); t++) {
			VERBOSE("  Texture %u : size (%d,%d)", t, mesh.texturesDiffuse[t].cols, mesh.texturesDiffuse[t].rows);
		}
	}

	// init CUDA rasterer
	CUDA::Point3* posPtr = reinterpret_cast<CUDA::Point3*>(mesh.vertices.GetData());
	CUDA::Point3i* triPtr = reinterpret_cast<CUDA::Point3i*>(mesh.faces.GetData());
	CUDA::Point2* texPtr = reinterpret_cast<CUDA::Point2*>(mesh.faceTexcoords.GetData());
		
	for (uint texIdx = 0; texIdx < mesh.texturesDiffuse.size(); texIdx++) {
		std::vector<std::vector<uint>> viewsFaces(images.size());
		std::vector<uint> texViews;
		// get views and faces for that texture
		for (uint faceIdx = 0; faceIdx < mesh.faces.size(); faceIdx++) {
			if (!mesh.faceTexindices.empty() && mesh.faceTexindices[faceIdx] != texIdx)
				continue;
			for (int j = 0; j < nKeepViews; ++j) {
				int vIdx = faceViewsIdx[faceIdx * nKeepViews + j];
				float vScore = faceViewsScore[faceIdx * nKeepViews + j];
				if (vIdx == -1 || vScore == 0.f)
					break;	
				viewsFaces[vIdx].push_back(faceIdx);
				bool found = false;
				for (uint k = 0; k < texViews.size(); ++k) {
					if (texViews[k] == vIdx) {
						found = true;
						break;
					}
				}
				if (!found)
					texViews.push_back((uint)vIdx);
			}
		}
		std::sort(texViews.begin(), texViews.end());

		Util::Progress progress2(_T("Rasterizing view "),texViews.size());
		VERBOSE("Processing texture %u / %u", texIdx+1, mesh.texturesDiffuse.size());
		if (VERBOSITY_LEVEL < 3)
			GET_LOGCONSOLE().Pause();
		cudaRasterizer.InitializeTexturePyramidOnDevice(mesh.texturesDiffuse[texIdx].size(), levels);
		uint cmpt = 0;
		// make pyramid and rasterize for each views
		for (uint viewIdx : texViews) 
		{
			cmpt++;
			if (VERBOSITY_LEVEL < 3)
				progress2.display(cmpt);
			else
				VERBOSE("Rasterizing texture %u / %u from view %u / %u", texIdx + 1, mesh.texturesDiffuse.size(), cmpt, texViews.size()); // TODO
			// prepare image pyramid for that view
			MVS::Image& view = images[viewIdx];
			{
				int imageRes = MAX(imageSize.width, imageSize.height);
				if (VERBOSITY_LEVEL > 2)
					VERBOSE("Preparing image at resolution %d", imageRes);
				view.ResizeImage(imageRes);
				if (view.image.empty())
					view.ReloadImage(imageRes);
				cv::Mat imageFloat;	
				view.image.convertTo(imageFloat, CV_32FC3, 1.0 / 255.0);
				cv::Mat imageLab;
				cv::cvtColor(imageFloat, imageLab, cv::COLOR_RGB2Lab);
				cudaRasterizer.MakeImageCustomPyramid(imageLab, levels+1);
			}

			// set camera
			const Camera camera = view.camera.GetScaled(viewSize, view.GetSize());
			CUDA::Matrix3 R;
			CUDA::Matrix3 K;
			CUDA::Point3 C;
			for (int r = 0; r < 3; ++r) {
				C[r] = camera.C[r];
				for (int c = 0; c < 3; ++c) {
					R(r, c) = camera.R(r, c);
					K(r, c) = camera.K(r, c);
				}
			}
			cudaRasterizer.SetCameraMatrices(C, R, K, viewIdx);
			// set faces to raster for that view
			cudaRasterizer.LoadMeshChunkOnDevice(posPtr,triPtr,texPtr,viewsFaces[viewIdx],true);
			cudaRasterizer.SetFacesForView(viewIdx, viewsFaces[viewIdx],nBests);
			cudaRasterizer.TextureRasterize();
			view.image.release();
		}
		// collapse texture pyramid
		cudaRasterizer.CollapseTexturePyramid();
		
		// get final texture and convert it from Lab to RGB
		cv::Mat finalTextureLab;
		cudaRasterizer.CopyFinalTextureToHost(finalTextureLab, 0,0);
		cv::Mat finalTexture;
		cv::cvtColor(finalTextureLab, finalTexture, cv::COLOR_Lab2RGB);	
		finalTexture.convertTo(mesh.texturesDiffuse[texIdx], CV_8UC3, 255.0);

		//add texture gutter
		cv::Mat validText;
		cudaRasterizer.CopyVisibilityMapTexToHost(validText);
		const Image8U mask(validText == 0);
		if (VERBOSITY_LEVEL >2)
			mask.Save(MAKE_PATH("mask_save" + String::ToString(texIdx) + ".png"));
		if (gutterSize > 0){
			std::vector<int> idxValidPixels;
			idxValidPixels.push_back(-1); 
			for ( int i=0; i<mask.area(); i++) {
				if (!mask[i]) {
					idxValidPixels.push_back(i);
				}
			}
			Image32F dists;
			cv::Mat_<int32_t> labels;
			cv::distanceTransform(mask, dists, labels, cv::DIST_L2, cv::DIST_MASK_PRECISE,cv::DIST_LABEL_PIXEL);
			for (int i = 0; i<mesh.texturesDiffuse[texIdx].area(); i++) {
				const int dist = dists[i];
				if (dist <= gutterSize && dist > 0) {
					const int idxClosest = idxValidPixels[labels(i)];
					mesh.texturesDiffuse[texIdx](i) = mesh.texturesDiffuse[texIdx](idxClosest);
				}
			}
		}
		if (VERBOSITY_LEVEL > 2)
			cv::imwrite(MAKE_PATH("texture_cuda_" + String::ToString(texIdx) + ".png"), mesh.texturesDiffuse[texIdx]);
		if (VERBOSITY_LEVEL < 3)
			GET_LOGCONSOLE().Play();
		progress2.close();

	}
	return true;
}
