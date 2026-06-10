// use to estimate memoryLoad for given tile carateristics 
static size_t EstimateMemoryLoad(uint nFaces, uint nViews, uint tileResWithOverlap, uint levels) {
	const size_t nPixels = (size_t)tileResWithOverlap * tileResWithOverlap;
	// Pyramid texture stacks: (levels+1) levels × pixels × nViews × 3 channels (float)
	const size_t pyramidStacksMem = (size_t)(levels + 1) * nPixels * nViews * 3 * sizeof(float);
	// Visibility scores: pixels × nViews (float)
	const size_t visMem = nPixels * nViews * sizeof(float);
	// Mesh chunk: 3 vertices per face × sizeof(Point3=float3)
	const size_t meshMem = (size_t)nFaces * 3 * 3 * sizeof(float);
	// Collapse workspace (~246 bytes/pixel) + other CURAST mats (depth/face/vis maps, averages, etc.)
	const size_t workspaceMem = nPixels * (246 + 40 + (size_t)(levels + 1) * 3 * sizeof(float));
	return pyramidStacksMem + visMem + meshMem + workspaceMem;
}

// GenerateOrthoMap — generate an orthographic map tiles for the scene mesh.
//
// overview:
//    - view projection: for each input image, load the mesh chunk seen by the view and project the mesh 
//      onto the view and compute per-face view scores and resoultion
//    - tile size binary search: when _tileSize==0 (auto mode), perform a binary search
//      over world-space tile sizes to find the largest tile whose estimated pixel resolution stays
//      ≤ maxTileRes and (if useGPUMemConstraint) whose worst-case GPU memory load ≤ targetGPUMem.
//    - build view/face lists: for each tile, collect the ordered set of views that
//      contribute faces, and build per-view face index lists used during rasterization.
//    - rasterization: for each tile in the grid:
//        - for each contributing view: load the image, build its Laplacian pyramid on GPU,
//          load the mesh chunk, and call sample the image into the tile's per-view tile 
//          stacks using orthographic projection.
//        - generate the per-pixel normal map for the tile, it will be used to limit the
//          remove to the flattest areas
//        - Collapse the multi-view stacks via best views blending, outlier removal and 
//          laplacian pyramid recombination
//   - tile assembly: blend all tile PNGs into the final orthomap using overlap
//     feathering weights, normalize accumulated colours, and write the output image
//
// parameters:
//   _tileSize           — fixed world-space tile size (0 = auto binary search)
//   _maxImgRes          — maximum resolution at which input images are loaded
//   _maxTileRes         — upper bound on per-tile pixel resolution
//   outputName          — file name for the final assembled atlas
//   ignoreFilling       — skip filling-only faces (vertexQuality==1) when building face lists
//   useGPUMemConstraint — when true, additionally constrain tile size by available GPU memory
bool Scene::GenerateOrthoMap(const float _tileSize, const int _maxImgRes, const int _maxTileRes, const SEACAVE::String outputName, bool bResume) {
	
	bool useGPUMemConstraint = false;
	if (_tileSize == 0)
		useGPUMemConstraint = true;

	// parameters
	bool optimizeTileRes = true;
	float tileSize = _tileSize;
	const uint maxTileRes = _maxTileRes;
	const uint maxImgRes = _maxImgRes;		// maximal images resolution
	uint po = images.size();
	const uint nKeepViews = po;  // number of views to consider per face for view selection
	const uint stackSize = po;				// number of samples kept per texel for outlier removal
	const uint bBests = po;					// minimal rank to be considered as best view for a face
	const uint levels = 2;					// number of level for the pyramid (in addition to the base level)
	const uint pad = 5;						// padding around each patch in the atlas
	const float overlapRatio = 0.05f;      // 5% overlap between tiles
	// initial image size and view size
	const cv::Size viewSize = images[0].GetSize();
	images[0].ReloadImage(maxImgRes);
	cv::Size imageSize = images[0].GetSize();
	images[0].image.release();
	images[0].ResizeImage(viewSize.width);
	if (maxImgRes == 0)
		imageSize = viewSize;

	// init CUDA rasterer
	SEACAVE::CUDA::CudaStreamSharedPtr ptrStream(SEACAVE::CUDA::CreateSharedStream());
	OrthoCURAST cudaRasterizer(ptrStream, stackSize, nKeepViews,mesh.faces.size());

	Util::Progress progress(_T("Projecting mesh to views"), images.size());
	if (VERBOSITY_LEVEL < 3)
		GET_LOGCONSOLE().Pause();
	
	
	// project mesh onto each view and compute per-face best-view scores.
	// for each view that has faces visible in its frustum, upload the mesh chunk
	// and camera matrices to GPU, then run ProjectMeshForOrtho to update the
	// per-face ranked view list (score, index, resolution) stored on the host.
	{
		Mesh::Octree octree;
		Mesh::FacesInserter::CreateOctree(octree, mesh);
		cudaRasterizer.InitializeFaceMap(imageSize, true);
		CUDA::Point3* posPtr = reinterpret_cast<CUDA::Point3*>(mesh.vertices.GetData());
		CUDA::Point3i* facePtr = reinterpret_cast<CUDA::Point3i*>(mesh.faces.GetData());
		for (uint viewIdx = 0; viewIdx < images.size(); ++viewIdx) {	
			if (VERBOSITY_LEVEL < 3)
				progress.display(viewIdx+1);
			else 
				VERBOSE("Project mesh on view %u / %u", viewIdx, images.size());

			// load mesh chunk relative to the view
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
			if (meshChunk.size() == 0) {
				DEBUG_LEVEL(2,"No faces in view %u, skipping\n", viewIdx);
				continue;
				}
			std::vector<uint> facesIdx(meshChunk.data(), meshChunk.data() + meshChunk.size());
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
			cudaRasterizer.ProjectMeshForOrtho();
		}
	}
	cudaRasterizer.ClearMeshBuffer();
	progress.close();
	if (VERBOSITY_LEVEL < 3)
		GET_LOGCONSOLE().Play() ;
	const std::vector<int>& faceViewsIdx = cudaRasterizer.GetBestViewIdxRef();
	const std::vector<float>& faceViewsScore = cudaRasterizer.GetBestViewScore();
	const std::vector<uint>& faceViewsRes = cudaRasterizer.GetResFaceRef();

	// if tileSize is not fixed tries to find the optimal size given the maxTileRes
	// else tries to find the optimal tile res
	// if optimal lower than maxTileRes then takes the optimal
	const Mesh::Box bBox = mesh.GetAABB(0.0001f,.9999f);
	const Point3f sizeBBox = bBox.GetSize();
	const float maxSizeBB = MAX(sizeBBox.x,sizeBBox.y);
	Point3f minCorn = bBox.ptMin;
	Point3f maxCorn = bBox.ptMax;

	int maxAttempt = 20; // number of attempts to determine tile size
	uint cmpt = 0;
	uint tileRes;
	Point2i GridDims;
	std::vector<std::vector<uint>> facesPerTile;
	uint nTiles;
 	if (tileSize == 0)
		tileSize = maxSizeBB*(1+0.0001);
	float prevLow = 0;
	float prevHigh = 0;
	// query available GPU memory for the memory-based tile size estimation mode
	size_t targetGPUMem = 0;
  	if (useGPUMemConstraint) {
		size_t freeMem, totalMem;
		cudaMemGetInfo(&freeMem, &totalMem);
		targetGPUMem = (size_t)(freeMem * 0.75);
		DEBUG_LEVEL(2,"GPU memory constraint: %.0f MB free, using %.0f MB as target", freeMem / 1e6, targetGPUMem / 1e6);
	}
	// best valid configuration found so far in the binary search
	float bestTileSize = 0;
	uint bestTileRes = 0;
	std::vector<std::vector<uint>> bestFacesPerTile;
	Point2i bestGridDims;
	uint bestNTiles = 0;
	// binary search for optimal tile size
	// Find the largest tile size such that the tile resolution ≤ maxTileRes
	// and (if useGPUMemConstraint) the worst-case GPU memory load per tile ≤ targetGPUMem.
	// Each iteration recomputes the tile grid, estimates the optimal pixel resolution
	// from face best-view data, and adjusts tileSize via bisection.
	for (int i = 0; i < maxAttempt; i++) {
		// tiles grid
		GridDims = Point2i(CEIL2INT(sizeBBox.x / tileSize), CEIL2INT(sizeBBox.y / tileSize));
		nTiles = GridDims.x * GridDims.y;
		facesPerTile.resize(nTiles);
		std::fill(facesPerTile.begin(), facesPerTile.end(), std::vector<uint>(0));

		// fill facesPerTile
		{
			const float overlapWorldIter = tileSize * overlapRatio;
			for (uint faceIdx = 0; faceIdx < mesh.faces.size(); ++faceIdx) {
				const Point3i& face = mesh.faces[faceIdx];
				const Point3f p[3] = {
					mesh.vertices[face[0]],
					mesh.vertices[face[1]],
					mesh.vertices[face[2]]
				};

				// bbox XY the face
				const float minX = MINF(p[0].x, MINF(p[1].x, p[2].x));
				const float maxX = MAXF(p[0].x, MAXF(p[1].x, p[2].x));
				const float minY = MINF(p[0].y, MINF(p[1].y, p[2].y));
				const float maxY = MAXF(p[0].y, MAXF(p[1].y, p[2].y));

				// range coverd by the tile 
				int tileMinX = FLOOR2INT((minX - minCorn.x - overlapWorldIter) / tileSize);
				int tileMaxX = FLOOR2INT((maxX - minCorn.x + overlapWorldIter) / tileSize);
				int tileMinY = FLOOR2INT((minY - minCorn.y - overlapWorldIter) / tileSize);
				int tileMaxY = FLOOR2INT((maxY - minCorn.y + overlapWorldIter) / tileSize);

				tileMinX = MAXF(0, tileMinX);
				tileMinY = MAXF(0, tileMinY);
				tileMaxX = MINF(GridDims.x - 1, tileMaxX);
				tileMaxY = MINF(GridDims.y - 1, tileMaxY);

				if (tileMinX > tileMaxX || tileMinY > tileMaxY)
					continue;

				for (int ty = tileMinY; ty <= tileMaxY; ++ty) {
					for (int tx = tileMinX; tx <= tileMaxX; ++tx) {
						const uint tileIdx = ty * GridDims.x + tx;
						facesPerTile[tileIdx].push_back(faceIdx);
					}
				}

			}
		}

		// Estimate the optimal pixel resolution for the current tile size.
		// For each tile, accumulate res*cos(angle) for all upward-facing faces,
		// then take sqrt (treating the sum as a pixel-area estimate). Average over
		// non-empty tiles only to avoid dilution from tiles with no geometry.
		uint optRes;
		{
			float averageRes = 0;
			uint nonEmptyTiles = 0;
			for (int yTile = 0; yTile < GridDims.y; yTile++) {
				for (int xTile = 0; xTile < GridDims.x; xTile++) {
					float SumRes = 0.f;
					uint tileIdx = yTile * GridDims.x + xTile;
					for (uint faceIdx : facesPerTile[tileIdx]) {
						// compute the face normal
						TPoint3<float> p[3] = {mesh.vertices[mesh.faces[faceIdx][0]], mesh.vertices[mesh.faces[faceIdx][1]], mesh.vertices[mesh.faces[faceIdx][2]]};
						TPoint3<float> normal = (p[1] - p[0]).cross(p[2] - p[0]);
						float norm = sqrt(normal.dot(normal));
						if (norm < 1e-6)
							continue;
						normal /= norm;
						// skip downward-facing faces (normal.z <= 0 causes uint underflow)
						float res = faceViewsRes[faceIdx * nKeepViews];
						SumRes += res * abs(normal.z);
					}
					if (SumRes > 0.f) {
						averageRes += SumRes;
						++nonEmptyTiles;
					}
				}
			}
			// divide by non-empty tiles only to avoid pulling the average down with empty tiles
			if (nonEmptyTiles > 0)
				averageRes /= nonEmptyTiles;
			optRes = CEIL2INT(sqrt(averageRes));
		}
		// check GPU memory constraint if requested
		bool memFits = true;
		if (useGPUMemConstraint && _tileSize == 0) {
			const uint tileResEst = MIN(optRes, (uint)maxTileRes);
			const uint overlapPxEst = MAXF(1u, (uint)CEIL2INT((float)tileResEst * overlapRatio));
			const uint tileResWO = tileResEst + 2 * overlapPxEst;
			size_t maxMemLoad = 0;
			for (int yTile = 0; yTile < GridDims.y; yTile++) {
				for (int xTile = 0; xTile < GridDims.x; xTile++) {
					const uint tileIdx = yTile * GridDims.x + xTile;
					const uint nTileFaces = (uint)facesPerTile[tileIdx].size();
					// count unique views for this tile
					std::unordered_set<int> tileViews;
					for (uint faceIdx : facesPerTile[tileIdx]) {
						for (int j = 0; j < (int)nKeepViews; ++j) {
							const int vIdx = faceViewsIdx[faceIdx * nKeepViews + j];
							if (vIdx == -1) break;
							tileViews.insert(vIdx);
						}
					}
					const size_t memLoad = EstimateMemoryLoad(nTileFaces, (uint)tileViews.size(), tileResWO, levels);
					if (memLoad > maxMemLoad) maxMemLoad = memLoad;
				}
			}
			memFits = (maxMemLoad <= targetGPUMem);
		  	DEBUG_LEVEL(2,"GPU memory check: max tile load=%.0f MB, target=%.0f MB (%s)", maxMemLoad / 1e6, targetGPUMem / 1e6, memFits ? "OK" : "too large");
		}
		if (_tileSize != 0) {
			tileRes = MIN(optRes,maxTileRes);
			break;
		} else {
			tileRes = optRes;
			const bool tooLarge = (optRes > (uint)maxTileRes) || !memFits;
			if (!tooLarge) {
				// this configuration satisfies all constraints — record it as best
				bestTileSize = tileSize;
				bestTileRes = tileRes;
				bestFacesPerTile = facesPerTile;
				bestGridDims = GridDims;
				bestNTiles = nTiles;
				prevLow = tileSize;
				if (i == 0) break;
				else if (prevHigh != 0)
					tileSize += (prevHigh-tileSize) / 2.f;
				else 
					tileSize += tileSize;
			} else {
				prevHigh = tileSize;
				if (prevLow != 0)
					tileSize -= (tileSize - prevLow) / 2.f;
				else 
					tileSize -= 0.5 * tileSize;
			}
		}
		DEBUG_LEVEL(2,"tile resolution found: %u", optRes);
		DEBUG_LEVEL(2,"trying tile size : %f", tileSize);
	}
	// restore the best valid configuration found during the binary search
	if (bestTileSize > 0) {
		tileSize = bestTileSize;
		tileRes = bestTileRes;
		facesPerTile = std::move(bestFacesPerTile);
		GridDims = bestGridDims;
		nTiles = bestNTiles;
	} else if (_tileSize == 0) {
		// no candidate satisfied maxTileRes / GPU memory constraints during the search:
		// fall back to the last evaluated grid but clamp the tile pixel resolution to maxTileRes
		// so we never allocate beyond the user/GPU budget
		if (tileRes > (uint)maxTileRes) {
			VERBOSE("warning: no tile size satisfied max-tile-res=%d; clamping tile resolution from %u to %d", maxTileRes, tileRes, maxTileRes);
			tileRes = (uint)maxTileRes;
		}
	}
	VERBOSE("Orthomap generation with image size (%d,%d), tileSize : (%f,%f), %u tiles, tile res : %u", imageSize.width, imageSize.height, tileSize, tileSize, nTiles, tileRes);
	
	// adding overlap 	
	const float overlapWorld = tileSize * overlapRatio;
	const uint overlapPx = MAXF(1u, (uint)CEIL2INT((float)tileRes * overlapRatio));
	const uint tileResWithOverlap = tileRes + 2 * overlapPx;

	// build per-tile view and face lists for rasterization.
	// for each tile, iterate over its assigned faces, look up the ranked view list,
	// and collect the unique contributing views in priority order. 
	// maxViewsPerTile is determined here to pre-allocate GPU stack buffers.
	int maxViewsPerTile = 0;
	// find max number of views per tile to allocate buffers on GPU
	for (int yTile = 0; yTile < GridDims.y; yTile++) {
		for (int xTile = 0; xTile < GridDims.x; xTile++) {

			float2 tileOrigin = make_float2(
				bBox.GetCorner(0).x() + xTile * tileSize - overlapWorld,
				bBox.GetCorner(0).y() + yTile * tileSize - overlapWorld
			);

			uint tileIdx = yTile * GridDims.x + xTile;
						
			std::vector<uint> viewsTile;
			std::vector<std::vector<uint>> faceByView;
			for (uint faceIdx : facesPerTile[tileIdx]) {
				std::vector<int> faceViews(nKeepViews); 
				std::copy(faceViewsIdx.begin() + faceIdx * nKeepViews, faceViewsIdx.begin() + (faceIdx + 1) * nKeepViews, faceViews.begin());
				for (int j = 0; j < nKeepViews; ++j) {
					int viewTileIdx = -1;
					int vIdx = faceViews[j];

					if (vIdx == -1)
						break;
					bool found = false;
					for (uint k = 0; k < viewsTile.size(); ++k) {
						if (viewsTile[k] == vIdx) {
							found = true;
							viewTileIdx = k;
							break;
						}
					}
					if (!found) {
						viewsTile.push_back((uint)vIdx);
						faceByView.push_back(std::vector<uint>());
						faceByView.back().push_back(faceIdx);
					} else {
						faceByView[viewTileIdx].push_back(faceIdx);
					}
				}
			}
			if (viewsTile.size() > maxViewsPerTile)
				maxViewsPerTile = viewsTile.size();
		}
	}
	if (maxViewsPerTile == 0) {
		VERBOSE("No faces to rasterize, skipping orthomap generation");
		return true;
	}

	cudaRasterizer.InitializeOrthoViewPyramidOnDevice(cv::Size(tileResWithOverlap,tileResWithOverlap), levels, maxViewsPerTile);

	// rasterize each tile 
	// for each tile, for each contributing view, load the image and build its 
	// Laplacian pyramid on GPU, load the mesh chunk, then call OrthoRasterize 
	// to sample the image into the per-view tile stacks, after all views are processed, 
	// collapse the stacks to the final tile color doing outlier removal and blending best
	// views 
	for (int yTile = 0; yTile < GridDims.y; yTile++) {
		for (int xTile = 0; xTile < GridDims.x; xTile++) {
			// if the option to resume est set on,
			// if the png file already exists, skip the rasterization of that tile
			const String pathTile = MAKE_PATH("tile_" + String::ToString(xTile) + "_" + String::ToString(yTile) + ".png");
			if (bResume && std::filesystem::exists(pathTile.c_str())) {
				VERBOSE("Tile (%d,%d) already computed (%s)", xTile, yTile, pathTile.c_str());
				continue;
			}
			cudaRasterizer.ResetPyramidTextureStacks();
			float2 tileOrigin = make_float2( bBox.GetCorner(0).x() + xTile * tileSize - overlapWorld, bBox.GetCorner(0).y() + yTile * tileSize - overlapWorld );
			uint tileIdx = yTile * GridDims.x + xTile;
			std::vector<uint> viewsTile;
			std::vector<std::vector<uint>> faceByView;

			// gather views needed for that tile and face list for each views
			for (uint faceIdx : facesPerTile[tileIdx]) {
				std::vector<int> faceViews(nKeepViews); 
				std::copy(faceViewsIdx.begin() + faceIdx * nKeepViews, faceViewsIdx.begin() + (faceIdx + 1) * nKeepViews, faceViews.begin());
				for (int j = 0; j < nKeepViews; ++j) {
					int viewTileIdx = -1;
					int vIdx = faceViews[j];
					if (vIdx == -1)
						break;
					bool found = false;
					for (uint k = 0; k < viewsTile.size(); ++k) {
						if (viewsTile[k] == vIdx) {
							found = true;
							viewTileIdx = k;
							break;
						}
					}
					if (!found) {
						viewsTile.push_back((uint)vIdx);
						faceByView.push_back(std::vector<uint>());
						faceByView.back().push_back(faceIdx);
					} else {
						faceByView[viewTileIdx].push_back(faceIdx);
					}
				}
			}
			if (viewsTile.empty())
				continue;
			Util::Progress progress2(_T("Rasterizing view "),viewsTile.size());
			VERBOSE("Processing tile %u / %u", tileIdx+1,GridDims.x*GridDims.y);
			if (VERBOSITY_LEVEL < 3)
				GET_LOGCONSOLE().Pause();

			uint cmpt = 0;
		    // processing views
			for (int v = 0; v < viewsTile.size(); v++) {
				uint viewIdx = viewsTile[v];
				std::vector<uint>& faceIndices = faceByView[v];
				cmpt++;
				if (VERBOSITY_LEVEL < 3)
					progress2.display(cmpt);
				else
					VERBOSE("Rasterizing tile %u / %u from view %u / %u", tileIdx + 1, GridDims.x*GridDims.y, cmpt, viewsTile.size());

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
				CUDA::Point3* posPtr = reinterpret_cast<CUDA::Point3*>(mesh.vertices.GetData());
				CUDA::Point3i* facePtr = reinterpret_cast<CUDA::Point3i*>(mesh.faces.GetData());
				cudaRasterizer.LoadMeshChunkOnDevice(posPtr, facePtr, faceIndices);
				cudaRasterizer.SetFacesForView(viewIdx,faceIndices);
				cudaRasterizer.OrthoRasterize(v, tileOrigin, float2(tileSize + 2.f * overlapWorld, tileSize + 2.f * overlapWorld));
				view.image.release();
			}

			// load mesh chunk for the tile
			CUDA::Point3* posPtr = reinterpret_cast<CUDA::Point3*>(mesh.vertices.GetData());
			CUDA::Point3i* facePtr = reinterpret_cast<CUDA::Point3i*>(mesh.faces.GetData());
			cudaRasterizer.LoadMeshChunkOnDevice(posPtr, facePtr, facesPerTile[tileIdx]);
			// generate normal map for the tile to guide the stack collapsing
			cudaRasterizer.GenerateNormalOrthoMap(tileOrigin, float2(tileSize + 2.f * overlapWorld, tileSize + 2.f * overlapWorld),	cv::Size((int)tileResWithOverlap, (int)tileResWithOverlap));
			// collapse multi-view stacks: outlier removal + Laplacian blending
			cudaRasterizer.CollapseTexturePyramid();

			// get final texture and convert it from Lab to RGB
			cv::Mat finalTileLab;
			cudaRasterizer.CopyFinalTextureToHost(finalTileLab, 0, 0);
			cv::Mat finalTexture;
			cv::cvtColor(finalTileLab, finalTexture, cv::COLOR_Lab2RGB);	
			// save tile
			cv::Mat finalTexture8U;
			finalTexture.convertTo(finalTexture8U, CV_8UC3, 255.0);
			cv::imwrite(pathTile, finalTexture8U);
			if (VERBOSITY_LEVEL < 3)
				GET_LOGCONSOLE().Play();
			progress2.close();
		}
	}

	// assemble the final orthomap by blending all saved tile PNGs using feathered overlap
	// weights, normalize the accumulated colour values, and write the output image.
	#if 1
	// assemble ortho map from tiles with overlap blending
	const int orthoW = tileRes * GridDims.x;
	const int orthoH = tileRes * GridDims.y;

	// precompute tile blending weights based on distance to tile borders
	auto BuildWeight = [&](int xTile, int yTile, int w, int h) -> cv::Mat {
		cv::Mat weight(h, w, CV_32FC1, cv::Scalar(1.f));
		const bool fadeL = (xTile > 0);
		const bool fadeR = (xTile < GridDims.x - 1);
		const bool fadeT = (yTile < GridDims.y - 1);
		const bool fadeB = (yTile > 0);
		for (int y = 0; y < h; ++y) {
			float wy = 1.f;
			if (fadeT && y < (int)overlapPx) wy = MINF(wy, float(y + 1) / float(overlapPx + 1));
			if (fadeB && y >= h - (int)overlapPx) wy = MINF(wy, float(h - y) / float(overlapPx + 1));

			float* wRow = weight.ptr<float>(y);
			for (int x = 0; x < w; ++x) {
				float wx = 1.f;
				if (fadeL && x < (int)overlapPx) wx = MINF(wx, float(x + 1) / float(overlapPx + 1));
				if (fadeR && x >= w - (int)overlapPx) wx = MINF(wx, float(w - x) / float(overlapPx + 1));
				wRow[x] = MINF(wx, wy);
			}
		}
		return weight;
	};
	cv::Mat orthoAccum(orthoH, orthoW, CV_32FC3, cv::Scalar::all(0));
	cv::Mat orthoWeight(orthoH, orthoW, CV_32FC1, cv::Scalar::all(0));

	// for each tile, load the tile image then accumulate the weighted colours into the ortho map
	for (int yTile = 0; yTile < GridDims.y; ++yTile) {
		for (int xTile = 0; xTile < GridDims.x; ++xTile) {
			const String pathTile = MAKE_PATH("tile_" + String::ToString(xTile) + "_" + String::ToString(yTile) + ".png");
			cv::Mat tile = cv::imread(pathTile);
			if (tile.empty()) {
				DEBUG("Failed to load tile image: %s", pathTile.c_str());
				continue;
			}
			if (tile.cols != (int)tileResWithOverlap || tile.rows != (int)tileResWithOverlap)
				cv::resize(tile, tile, cv::Size(tileResWithOverlap, tileResWithOverlap));
			cv::Mat tileF;
			tile.convertTo(tileF, CV_32FC3, 1.0 / 255.0);
			cv::Mat tileW = BuildWeight(xTile, yTile, tile.cols, tile.rows);
			const int dstX = xTile * (int)tileRes - (int)overlapPx;
			const int dstY = (GridDims.y - 1 - yTile) * (int)tileRes - (int)overlapPx;

			const int srcX0 = MAXF(0, -dstX);
			const int srcY0 = MAXF(0, -dstY);
			const int dstX0 = MAXF(0, dstX);
			const int dstY0 = MAXF(0, dstY);

			const int copyW = MINF(tile.cols - srcX0, orthoW - dstX0);
			const int copyH = MINF(tile.rows - srcY0, orthoH - dstY0);
			if (copyW <= 0 || copyH <= 0)
				continue;

			const cv::Rect srcRect(srcX0, srcY0, copyW, copyH);
			const cv::Rect dstRect(dstX0, dstY0, copyW, copyH);

			cv::Mat srcTile = tileF(srcRect);
			cv::Mat srcW = tileW(srcRect);
			cv::Mat dstAcc = orthoAccum(dstRect);
			cv::Mat dstW = orthoWeight(dstRect);

			for (int y = 0; y < copyH; ++y) {
				const cv::Vec3f* sT = srcTile.ptr<cv::Vec3f>(y);
				const float* sW = srcW.ptr<float>(y);
				cv::Vec3f* dA = dstAcc.ptr<cv::Vec3f>(y);
				float* dW = dstW.ptr<float>(y);
				for (int x = 0; x < copyW; ++x) {
					dA[x] += sT[x] * sW[x];
					dW[x] += sW[x];
				}
			}
		}
	}

	// normalize accumulated colours and save final ortho map
	cv::Mat orthoMap(orthoH, orthoW, CV_8UC3, cv::Scalar::all(0));
	for (int y = 0; y < orthoH; ++y) {
		const cv::Vec3f* aRow = orthoAccum.ptr<cv::Vec3f>(y);
		const float* wRow = orthoWeight.ptr<float>(y);
		cv::Vec3b* oRow = orthoMap.ptr<cv::Vec3b>(y);
		for (int x = 0; x < orthoW; ++x) {
			if (wRow[x] <= 1e-6f)
				continue;
			const cv::Vec3f c = aRow[x] * (1.f / wRow[x]);
			oRow[x][0] = (uint8_t)CLAMP(ROUND2INT(c[0] * 255.f), 0, 255);
			oRow[x][1] = (uint8_t)CLAMP(ROUND2INT(c[1] * 255.f), 0, 255);
			oRow[x][2] = (uint8_t)CLAMP(ROUND2INT(c[2] * 255.f), 0, 255);
		}
	}

	// crop empty borders on all four sides
	int left   = orthoMap.cols;
	int top    = orthoMap.rows;
	int right  = -1;
	int bottom = -1;
	for (int y = 0; y < orthoMap.rows; ++y) {
		for (int x = 0; x < orthoMap.cols; ++x) {
			if (orthoMap.at<cv::Vec3b>(y, x) != cv::Vec3b(0, 0, 0)) {
				left   = std::min(left,   x);
				top    = std::min(top,    y);
				right  = std::max(right,  x);
				bottom = std::max(bottom, y);
			}
		}
	}
	// if the assembled orthomap is entirely empty, skip cropping (otherwise the ROI is degenerate)
	const cv::Mat cropped = (right < left || bottom < top)
		? orthoMap
		: cv::Mat(orthoMap(cv::Rect(left, top, right - left + 1, bottom - top + 1)).clone());
	cv::imwrite(MAKE_PATH_SAFE(outputName), cropped);
	#endif
	return true;
}
