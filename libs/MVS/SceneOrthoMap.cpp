/*
* SceneOrthoMap.cpp
*
* Copyright (c) 2014-2025 SEACAVE
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
#include "SceneOrthoMap.h"
#include "../Math/LBP.h"
#include "../Math/DisjointSet.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#ifdef _USE_OPENMP
#define ORTHOMAP_USE_OPENMP
#endif

// Step 2 constants
#define ORTHO_BLOCK_SIZE      4    // pixels per block side
#define ORTHO_BRIEF_SAMPLES  32    // 16 within block + 16 in 8-neighbors
#define ORTHO_BRIEF_PAIRS   256    // 256-bit descriptor

// MRF energy constants (matching texturing pipeline convention)
constexpr LBPInference::EnergyType OrthoLBPMaxEnergy(1.f);
constexpr LBPInference::EnergyType OrthoLBPMinWeight(0.5f);

// Potts smoothness: 0 if same label, 1 if different (scaled by edge weight)
static LBPInference::EnergyType STCALL OrthoSmoothnessPotts(
	LBPInference::NodeID, LBPInference::NodeID,
	LBPInference::LabelID l1, LBPInference::LabelID l2)
{
	return l1 == l2 && l1 != 0 && l2 != 0 ? LBPInference::EnergyType(0) : OrthoLBPMaxEnergy;
}

// static BRIEF comparison pair table: 256 pairs of indices [0..31]
static uint8_t briefPairs[ORTHO_BRIEF_PAIRS][2];
static bool briefPairsInitialized = false;

static void InitBRIEFPairs()
{
	if (briefPairsInitialized)
		return;
	// deterministic PRNG with fixed seed
	std::mt19937 rng(0x4F525448); // "ORTH"
	for (int i = 0; i < ORTHO_BRIEF_PAIRS; ++i) {
		briefPairs[i][0] = (uint8_t)(rng() % ORTHO_BRIEF_SAMPLES);
		uint8_t b;
		do { b = (uint8_t)(rng() % ORTHO_BRIEF_SAMPLES); } while (b == briefPairs[i][0]);
		briefPairs[i][1] = b;
	}
	briefPairsInitialized = true;
}


// S T R U C T S ///////////////////////////////////////////////////

// ortho G-buffer rasterizer: depth-only, orthographic projection
// follows the pattern of Mesh::ProjectOrtho (Mesh.cpp)
struct RasterOrthoGBuffer : TRasterMesh<RasterOrthoGBuffer> {
	typedef TRasterMesh<RasterOrthoGBuffer> Base;
	RasterOrthoGBuffer(const Mesh::VertexArr& _vertices, const Camera& _camera, DepthMap& _depthMap)
		: Base(_vertices, _camera, _depthMap) {}
	inline bool ProjectVertex(const Mesh::Vertex& pt, int v, Triangle& t) {
		return (t.ptc[v] = camera.TransformPointW2C(Cast<REAL>(pt))).z > 0 &&
			depthMap.isInsideWithBorder<float,3>(t.pti[v] = camera.TransformPointOrthoC2I(t.ptc[v]));
	}
	void Raster(const ImageRef& pt, const Triangle& t, const Point3f& bary) {
		const Depth z(ComputeDepth(t, bary));
		ASSERT(z > Depth(0));
		Depth& depth = depthMap(pt);
		if (depth == 0 || depth > z)
			depth = z;
	}
};
/*----------------------------------------------------------------*/


// compute GSD from nadir-looking cameras or use user override
float OrthoMapContext::ComputeGSD() const
{
	// user override
	if (config.targetGSD > 0)
		return config.targetGSD;
	// ensure camera distances are computed
	bool bNeedDistances = false;
	FOREACH(idx, scene.images) {
		const Image& image = scene.images[idx];
		if (image.IsValid() && image.avgDepth <= 0) {
			bNeedDistances = true;
			break;
		}
	}
	if (bNeedDistances)
		const_cast<Scene&>(scene).ComputeDistanceCameras2Scene();
	// collect per-camera GSD for nadir-looking cameras
	FloatArr gsdValues;
	FOREACH(idx, scene.images) {
		const Image& image = scene.images[idx];
		if (!image.IsValid() || image.avgDepth <= 0)
			continue;
		// nadir test: down-looking cameras have dir ~= (0,0,-1), so abs(dir.z) near 1.0
		const Point3 dir(image.camera.Direction());
		if (ABS(dir.z) < config.nadirThreshold)
			continue;
		// GSD = avgDepth / focalLength (world-meters-per-pixel)
		const REAL focalLength = image.camera.GetFocalLength();
		if (focalLength <= 0)
			continue;
		gsdValues.emplace_back((float)(image.avgDepth / focalLength));
	}
	if (gsdValues.empty()) {
		VERBOSE("error: no nadir-looking cameras found for GSD estimation (threshold: %.2f)", config.nadirThreshold);
		return 0;
	}
	return gsdValues.GetMedian();
}

// compute grid layout and create tile array
bool OrthoMapContext::ComputeGrid()
{
	TD_TIMER_START();
	// compute mesh AABB
	AABB3f box(scene.mesh.GetAABB());
	// expand XY by margin (do NOT expand Z -- camera sits above max Z)
	const float extentX = box.ptMax.x() - box.ptMin.x();
	const float extentY = box.ptMax.y() - box.ptMin.y();
	const float margin = MAXF(0.1f, MAXF(extentX, extentY) * config.marginFactor);
	box.ptMin.x() -= margin;
	box.ptMin.y() -= margin;
	box.ptMax.x() += margin;
	box.ptMax.y() += margin;
	grid.sceneAABB = box;
	// compute GSD
	grid.gsd = ComputeGSD();
	if (grid.gsd <= 0)
		return false;
	// compute total grid dimensions in pixels
	const float sizeX = grid.sceneAABB.ptMax.x() - grid.sceneAABB.ptMin.x();
	const float sizeY = grid.sceneAABB.ptMax.y() - grid.sceneAABB.ptMin.y();
	grid.sizePx = cv::Size(
		CEIL2INT(sizeX / grid.gsd),
		CEIL2INT(sizeY / grid.gsd));
	// compute tile layout
	const unsigned effectiveStride = config.tileSize - config.tileOverlap;
	grid.tiles.x = ((unsigned)grid.sizePx.width + effectiveStride - 1) / effectiveStride;
	grid.tiles.y = ((unsigned)grid.sizePx.height + effectiveStride - 1) / effectiveStride;
	// allocate and initialize tiles
	tiles.resize(grid.tiles.x * grid.tiles.y);
	for (unsigned ty = 0; ty < grid.tiles.y; ++ty) {
		for (unsigned tx = 0; tx < grid.tiles.x; ++tx) {
			OrthoTile& tile = tiles[ty * grid.tiles.x + tx];
			tile.tile = Point2u(tx, ty);
			tile.p0 = Point2u(tx * effectiveStride, ty * effectiveStride);
			const unsigned pxEnd = MINF(tile.p0.x + config.tileSize, (unsigned)grid.sizePx.width);
			const unsigned pyEnd = MINF(tile.p0.y + config.tileSize, (unsigned)grid.sizePx.height);
			tile.size = cv::Size(pxEnd - tile.p0.x, pyEnd - tile.p0.y);
			// world bounds: pixel row 0 = north (max world Y), rows increase southward
			tile.worldBounds.ptMin.x() = grid.sceneAABB.ptMin.x() + tile.p0.x * grid.gsd;
			tile.worldBounds.ptMax.x() = grid.sceneAABB.ptMin.x() + pxEnd * grid.gsd;
			tile.worldBounds.ptMax.y() = grid.sceneAABB.ptMax.y() - tile.p0.y * grid.gsd;   // north edge
			tile.worldBounds.ptMin.y() = grid.sceneAABB.ptMax.y() - pyEnd * grid.gsd;        // south edge
			tile.worldBounds.ptMin.z() = grid.sceneAABB.ptMin.z();
			tile.worldBounds.ptMax.z() = grid.sceneAABB.ptMax.z();
			// setup orthographic camera
			tile.camera = SetupTileCamera(tile);
		}
	}
	VERBOSE("OrthoMap grid: %u x %u tiles, %d x %d pixels, GSD %.4f m/px (%s)",
		grid.tiles.x, grid.tiles.y, grid.sizePx.width, grid.sizePx.height, grid.gsd,
		TD_TIMER_GET_FMT().c_str());
	return true;
}

// setup orthographic camera for a tile
// follows the pattern of Mesh::ProjectOrthoTopDown (Mesh.cpp)
Camera OrthoMapContext::SetupTileCamera(const OrthoTile& tile) const
{
	Camera camera;
	// look down along -Z, Y-up in world
	// R = [[1,0,0],[0,-1,0],[0,0,-1]]
	camera.R.SetFromDirUp(Vec3(Point3(0, 0, -1)), Vec3(Point3(0, 1, 0)));
	// position camera above the tile center, above the highest point
	const float centerX = (tile.worldBounds.ptMin.x() + tile.worldBounds.ptMax.x()) * 0.5f;
	const float centerY = (tile.worldBounds.ptMin.y() + tile.worldBounds.ptMax.y()) * 0.5f;
	camera.C = Point3(centerX, centerY, grid.sceneAABB.ptMax.z() + 1.0);
	// orthographic intrinsics: pixels per meter
	camera.K = KMatrix::IDENTITY;
	camera.K(0, 0) = camera.K(1, 1) = 1.0 / grid.gsd;
	camera.K(0, 2) = (REAL)(tile.size.width - 1) / 2;
	camera.K(1, 2) = (REAL)(tile.size.height - 1) / 2;
	return camera;
}

// build octree spatial index from mesh face centroids
void OrthoMapContext::BuildSpatialIndex()
{
	Mesh::FacesInserter::CreateOctree(octree, scene.mesh);
}

// rasterize mesh faces into a single tile's depth map
void OrthoMapContext::RasterizeTile(OrthoTile& tile, const Mesh::FaceIdxArr& tileFaces) const
{
	// allocate depth map
	tile.depthMap.create(tile.size);
	// setup rasterizer
	RasterOrthoGBuffer rasterer(scene.mesh.vertices, tile.camera, tile.depthMap);
	RasterOrthoGBuffer::Triangle triangle;
	RasterOrthoGBuffer::TriangleRasterizer triangleRasterizer(triangle, rasterer);
	rasterer.Clear();
	// rasterize all faces
	for (const Mesh::FIndex idxFace : tileFaces)
		rasterer.Project(scene.mesh.faces[idxFace], triangleRasterizer);
}

// main loop: build spatial index, rasterize all tiles in parallel
bool OrthoMapContext::RasterizeTiles()
{
	TD_TIMER_START();
	// build spatial index
	BuildSpatialIndex();
	// rasterize tiles in parallel
	#ifdef ORTHOMAP_USE_OPENMP
	#pragma omp parallel for schedule(dynamic)
	for (int _i = 0; _i < (int)tiles.size(); ++_i) {
		OrthoTile& tile = tiles[(uint32_t)_i];
	#else
	FOREACH(i, tiles) {
		OrthoTile& tile = tiles[i];
	#endif
		// query octree for faces overlapping this tile
		Mesh::FaceIdxArr tileFaces;
		AABB3f queryBounds(tile.worldBounds);
		queryBounds.Enlarge(grid.gsd * 16);  // safety margin for faces straddling tile edges
		Mesh::FacesInserterAABB inserter(tileFaces, queryBounds);
		octree.Collect(inserter, inserter);
		// rasterize
		RasterizeTile(tile, tileFaces);
	}
	VERBOSE("OrthoMap DEM generation completed: %u tiles (%s)",
		tiles.GetSize(), TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// S T E P  2 //////////////////////////////////////////////////////

// -- OrthoImageCache ----------------------------------------------

const OrthoImageCache::CachedImage& OrthoImageCache::UseImage(IIndex idxImage, Scene& scene)
{
	std::lock_guard<std::mutex> lock(mutex);
	auto it = images.find(idxImage);
	if (it != images.end()) {
		fifo.Put(idxImage);
		return it->second;
	}
	// load image (sequential tile processing, so no contention concern)
	Image& sceneImage = scene.images[idxImage];
	sceneImage.ReloadImage(0, true);
	CachedImage cached;
	cached.color = sceneImage.image.clone();
	sceneImage.ReleaseImage();
	cached.color.toGray(cached.gray, cv::COLOR_BGR2GRAY, true);
	cached.memoryBytes = (size_t)cached.color.area() * cached.color.pixel_stride()
		+ (size_t)cached.gray.area() * cached.gray.pixel_stride();
	images[idxImage] = std::move(cached);
	usedMemory += images[idxImage].memoryBytes;
	fifo.Put(idxImage);
	Eject();
	return images[idxImage];
}

void OrthoImageCache::EjectOldest()
{
	if (fifo.IsEmpty())
		return;
	const IIndex oldest = fifo.Pop();
	auto it = images.find(oldest);
	if (it != images.end()) {
		usedMemory -= it->second.memoryBytes;
		images.erase(it);
	}
}

void OrthoImageCache::Eject()
{
	if (maxMemory == 0)
		return;
	while (usedMemory > maxMemory && !fifo.IsEmpty())
		EjectOldest();
}
/*----------------------------------------------------------------*/

// -- Helpers ------------------------------------------------------

Point3 OrthoMapContext::OrthoPixelToWorld(const OrthoTile& tile, float px, float py, Depth depth) const
{
	return tile.camera.TransformPointOrthoI2W(Point3(px, py, depth));
}
/*----------------------------------------------------------------*/

// -- Step 2.1: FindTileViews --------------------------------------

IIndexArr OrthoMapContext::FindTileViews(const OrthoTile& tile) const
{
	IIndexArr result;
	// expand tile bounds slightly to catch views at the edge
	const AABB3f tileBounds(tile.worldBounds);
	// collect the 8 AABB corners
	Point3f corners[8];
	for (int i = 0; i < 8; ++i) {
		corners[i] = Point3f(
			(i & 1) ? tileBounds.ptMax.x() : tileBounds.ptMin.x(),
			(i & 2) ? tileBounds.ptMax.y() : tileBounds.ptMin.y(),
			(i & 4) ? tileBounds.ptMax.z() : tileBounds.ptMin.z());
	}
	FOREACH(idx, scene.images) {
		const Image& image = scene.images[idx];
		if (!image.IsValid() || !image.HasResolution())
			continue;
		const Camera& cam = image.camera;
		// skip cameras below the tile (looking up)
		if (cam.C.z <= (REAL)tileBounds.ptMin.z())
			continue;
		// project all 8 AABB corners into this view
		const Point2f imageSize((float)image.width, (float)image.height);
		bool anyInside = false;
		for (int c = 0; c < 8; ++c) {
			const Point3 pt3D(Cast<REAL>(corners[c]));
			const Point3 ptC(cam.TransformPointW2C(pt3D));
			if (ptC.z <= 0)
				continue; // behind camera
			const Point2 pt2D(cam.TransformPointC2I(Point2(ptC.x / ptC.z, ptC.y / ptC.z)));
			if (pt2D.x >= 0 && pt2D.x < imageSize.x && pt2D.y >= 0 && pt2D.y < imageSize.y) {
				anyInside = true;
				break;
			}
		}
		if (anyInside)
			result.push_back(idx);
	}
	return result;
}
/*----------------------------------------------------------------*/

// -- Step 2.1: OrthoProjectView -----------------------------------

void OrthoMapContext::OrthoProjectView(
	const OrthoTile& tile, IIndex idxView,
	const OrthoImageCache::CachedImage& img,
	OrthoBlockArr& blocks, const cv::Size& blockGrid) const
{
	const Camera& viewCamera = scene.images[idxView].camera;
	// nadir weight: quadratic preference for down-looking cameras
	const Point3 viewDir(viewCamera.Direction());
	const float cosNadir((float)ABS(viewDir.z));
	const float nadirWeight = cosNadir * cosNadir;
	const cv::Size imgSize(img.color.width(), img.color.height());
	const Point2f imgSizeF((float)imgSize.width, (float)imgSize.height);

	for (int by = 0; by < blockGrid.height; ++by) {
		for (int bx = 0; bx < blockGrid.width; ++bx) {
			// block center in tile pixel coords
			const float cx = ((float)bx + 0.5f) * ORTHO_BLOCK_SIZE;
			const float cy = ((float)by + 0.5f) * ORTHO_BLOCK_SIZE;
			const int icx = CLAMP(ROUND2INT(cx), 0, tile.depthMap.width()-1);
			const int icy = CLAMP(ROUND2INT(cy), 0, tile.depthMap.height()-1);
			const Depth depth = tile.depthMap(icy, icx);
			if (depth == 0)
				continue; // no surface

			// ortho pixel -> 3D world via DEM
			const Point3 worldPt(OrthoPixelToWorld(tile, cx, cy, depth));

			// project into source view
			const Point3 ptC(viewCamera.TransformPointW2C(worldPt));
			if (ptC.z <= 0)
				continue; // behind camera
			const Point2f pt2D(viewCamera.TransformPointC2I(Point2(ptC.x / ptC.z, ptC.y / ptC.z)));
			if (pt2D.x < 1 || pt2D.x >= imgSizeF.x - 1 || pt2D.y < 1 || pt2D.y >= imgSizeF.y - 1)
				continue; // outside with border

			// compute mean color over block pixels
			Pixel32F meanColor(Pixel32F::BLACK);
			int colorCount = 0;
			const int pxStart = bx * ORTHO_BLOCK_SIZE;
			const int pyStart = by * ORTHO_BLOCK_SIZE;
			const int pxEnd = MINF(pxStart + ORTHO_BLOCK_SIZE, tile.depthMap.width());
			const int pyEnd = MINF(pyStart + ORTHO_BLOCK_SIZE, tile.depthMap.height());
			for (int py = pyStart; py < pyEnd; ++py) {
				for (int px = pxStart; px < pxEnd; ++px) {
					const Depth d = tile.depthMap(py, px);
					if (d == 0)
						continue;
					const Point3 wp(OrthoPixelToWorld(tile, (float)px + 0.5f, (float)py + 0.5f, d));
					const Point3 pc(viewCamera.TransformPointW2C(wp));
					if (pc.z <= 0)
						continue;
					const Point2f p2d(viewCamera.TransformPointC2I(Point2(pc.x / pc.z, pc.y / pc.z)));
					if (p2d.x < 0 || p2d.x >= imgSizeF.x - 1 || p2d.y < 0 || p2d.y >= imgSizeF.y - 1)
						continue;
					const Pixel8U px8u(img.color.sample(Point2f(p2d.x, p2d.y)));
					meanColor.r += (float)px8u.r;
					meanColor.g += (float)px8u.g;
					meanColor.b += (float)px8u.b;
					++colorCount;
				}
			}
			if (colorCount == 0)
				continue;
			const float invCount = 1.f / (float)colorCount;
			meanColor.r *= invCount;
			meanColor.g *= invCount;
			meanColor.b *= invCount;

			// add candidate view to this block
			OrthoBlock& block = blocks[by * blockGrid.width + bx];
			OrthoBlockView bv;
			bv.idxView = idxView;
			bv.nadirWeight = nadirWeight;
			bv.meanColor = meanColor;
			bv.clusterID = 0;
			block.views.emplace_back(bv);
		}
	}
}
/*----------------------------------------------------------------*/

// -- Step 2.2: BRIEF Descriptor Computation -----------------------

BRIEFDescriptor OrthoMapContext::ComputeBRIEFDescriptor(
	const OrthoTile& tile, unsigned bx, unsigned by,
	const cv::Size& blockGrid, IIndex idxView,
	const OrthoImageCache::CachedImage& img) const
{
	const Camera& viewCamera = scene.images[idxView].camera;
	const cv::Size imgSize(img.gray.width(), img.gray.height());
	const Point2f imgSizeF((float)imgSize.width, (float)imgSize.height);

	// generate 32 sample points on DEM surface
	float intensities[ORTHO_BRIEF_SAMPLES];
	bool valid[ORTHO_BRIEF_SAMPLES];
	int idx = 0;

	// 16 points within block: 4x4 regular sub-grid
	for (int sy = 0; sy < 4; ++sy) {
		for (int sx = 0; sx < 4; ++sx) {
			valid[idx] = false;
			const float px = (float)(bx * ORTHO_BLOCK_SIZE) + (float)sx + 0.5f;
			const float py = (float)(by * ORTHO_BLOCK_SIZE) + (float)sy + 0.5f;
			if (px >= 0 && px < tile.depthMap.width() && py >= 0 && py < tile.depthMap.height()) {
				const Depth d = tile.depthMap(ROUND2INT(py), ROUND2INT(px));
				if (d > 0) {
					const Point3 wp(OrthoPixelToWorld(tile, px, py, d));
					const Point3 pc(viewCamera.TransformPointW2C(wp));
					if (pc.z > 0) {
						const Point2f p2d(viewCamera.TransformPointC2I(Point2(pc.x / pc.z, pc.y / pc.z)));
						if (p2d.x >= 0 && p2d.x < imgSizeF.x - 1 && p2d.y >= 0 && p2d.y < imgSizeF.y - 1) {
							intensities[idx] = img.gray.sample(p2d);
							valid[idx] = true;
						}
					}
				}
			}
			++idx;
		}
	}

	// 16 points in 8-connected neighbor blocks: 2 per neighbor
	static const int dx8[] = {-1,-1,-1, 0, 0, 1, 1, 1};
	static const int dy8[] = {-1, 0, 1,-1, 1,-1, 0, 1};
	for (int n = 0; n < 8; ++n) {
		const int nbx = (int)bx + dx8[n];
		const int nby = (int)by + dy8[n];
		for (int s = 0; s < 2; ++s) {
			valid[idx] = false;
			if (nbx >= 0 && nbx < blockGrid.width && nby >= 0 && nby < blockGrid.height) {
				const float frac = ((float)s + 1.f) / 3.f;
				const float px = (float)(nbx * ORTHO_BLOCK_SIZE) + frac * ORTHO_BLOCK_SIZE;
				const float py = (float)(nby * ORTHO_BLOCK_SIZE) + frac * ORTHO_BLOCK_SIZE;
				if (px >= 0 && px < tile.depthMap.width() && py >= 0 && py < tile.depthMap.height()) {
					const Depth d = tile.depthMap(ROUND2INT(py), ROUND2INT(px));
					if (d > 0) {
						const Point3 wp(OrthoPixelToWorld(tile, px, py, d));
						const Point3 pc(viewCamera.TransformPointW2C(wp));
						if (pc.z > 0) {
							const Point2f p2d(viewCamera.TransformPointC2I(Point2(pc.x / pc.z, pc.y / pc.z)));
							if (p2d.x >= 0 && p2d.x < imgSizeF.x - 1 && p2d.y >= 0 && p2d.y < imgSizeF.y - 1) {
								intensities[idx] = img.gray.sample(p2d);
								valid[idx] = true;
							}
						}
					}
				}
			}
			++idx;
		}
	}
	ASSERT(idx == ORTHO_BRIEF_SAMPLES);

	// compute 256-bit descriptor from comparison pairs
	BRIEFDescriptor desc;
	desc.Clear();
	for (int p = 0; p < ORTHO_BRIEF_PAIRS; ++p) {
		const uint8_t a = briefPairs[p][0];
		const uint8_t b = briefPairs[p][1];
		if (!valid[a] || !valid[b])
			continue;
		if (intensities[a] < intensities[b])
			desc.SetBit(p);
	}
	return desc;
}
/*----------------------------------------------------------------*/

// -- Step 2.2: Pairwise Clustering --------------------------------

void OrthoMapContext::ClusterBlockViews(
	const OrthoTile& tile, OrthoBlockArr& blocks,
	const cv::Size& blockGrid) const
{
	for (int by = 0; by < blockGrid.height; ++by) {
		for (int bx = 0; bx < blockGrid.width; ++bx) {
			OrthoBlock& block = blocks[by * blockGrid.width + bx];
			const uint32_t N = block.views.size();
			if (N == 0)
				continue;
			if (N == 1) {
				// single view -> single cluster
				block.views[0].clusterID = 0;
				OrthoBlockCluster c;
				c.representativeView = block.views[0].idxView;
				c.bestNadirWeight = block.views[0].nadirWeight;
				c.size = 1;
				c.dataWeight = block.views[0].nadirWeight;
				block.clusters.emplace_back(c);
				continue;
			}

			// compute BRIEF descriptors for all views
			CLISTDEF0(BRIEFDescriptor) descs(N);
			for (uint32_t i = 0; i < N; ++i) {
				const OrthoBlockView& bv = block.views[i];
				const auto it = imageCache.images.find(bv.idxView);
				ASSERT(it != imageCache.images.end());
				descs[i] = ComputeBRIEFDescriptor(tile, (unsigned)bx, (unsigned)by, blockGrid,
					bv.idxView, it->second);
			}

			// all-pairwise similarities + Union-Find clustering
			DisjointSet<uint32_t> ds(N);
			for (uint32_t i = 0; i < N - 1; ++i)
				for (uint32_t j = i + 1; j < N; ++j)
					if (descs[i].Similarity(descs[j]) >= config.briefSimilarityThreshold)
						ds.Union(i, j);

			// extract cluster assignments
			ds.CompressAllPaths();
			std::vector<uint32_t> componentIDs(N);
			const unsigned numClusters = ds.GetComponents(componentIDs);
			for (uint32_t i = 0; i < N; ++i)
				block.views[i].clusterID = componentIDs[i];

			// build cluster stats
			block.clusters.resize(numClusters);
			for (unsigned c = 0; c < numClusters; ++c) {
				block.clusters[c].representativeView = NO_ID;
				block.clusters[c].bestNadirWeight = 0;
				block.clusters[c].size = 0;
				block.clusters[c].dataWeight = 0;
			}
			for (uint32_t i = 0; i < N; ++i) {
				OrthoBlockCluster& cluster = block.clusters[block.views[i].clusterID];
				cluster.size++;
				if (block.views[i].nadirWeight > cluster.bestNadirWeight) {
					cluster.bestNadirWeight = block.views[i].nadirWeight;
					cluster.representativeView = block.views[i].idxView;
				}
			}

			// compute data weights
			const bool allSingletons = (numClusters == N);
			for (unsigned c = 0; c < numClusters; ++c) {
				OrthoBlockCluster& cluster = block.clusters[c];
				if (allSingletons) {
					// fallback: pure nadir, let MRF use spatial context
					cluster.dataWeight = cluster.bestNadirWeight;
				} else {
					const float consensus = (float)cluster.size / (float)N;
					cluster.dataWeight = config.consensusWeight * consensus
						+ (1.f - config.consensusWeight) * cluster.bestNadirWeight;
				}
			}

			// sort clusters by size descending and remap view clusterIDs
			if (numClusters > 1) {
				// record representative per old cluster index before sort
				CLISTDEF0(IIndex) oldReps(numClusters);
				for (unsigned c = 0; c < numClusters; ++c)
					oldReps[c] = block.clusters[c].representativeView;
				// sort
				block.clusters.Sort([](const OrthoBlockCluster& a, const OrthoBlockCluster& b) {
					return a.size > b.size;
				});
				// build old->new mapping: for each old cluster, find its new position by representative
				CLISTDEF0(uint32_t) oldToNew(numClusters);
				for (unsigned oldC = 0; oldC < numClusters; ++oldC) {
					for (unsigned newC = 0; newC < numClusters; ++newC) {
						if (block.clusters[newC].representativeView == oldReps[oldC]) {
							oldToNew[oldC] = newC;
							break;
						}
					}
				}
				for (uint32_t i = 0; i < N; ++i)
					block.views[i].clusterID = oldToNew[block.views[i].clusterID];
			}
		}
	}
}
/*----------------------------------------------------------------*/

// -- Step 2.3-2.4: MRF Block Selection ----------------------------

// find mean color in a block for a specific view
static Pixel32F FindBlockViewMeanColor(const OrthoBlock& block, IIndex idxView)
{
	FOREACHPTR(pView, block.views)
		if (pView->idxView == idxView)
			return pView->meanColor;
	return Pixel32F::BLACK;
}

// compute color-diff edge weight between two neighboring blocks
static float ComputeEdgeWeight(const OrthoBlock& block1, const OrthoBlock& block2)
{
	if (block1.clusters.empty() || block2.clusters.empty())
		return OrthoLBPMinWeight;
	// get representative views of the two largest clusters
	const IIndex rep1 = block1.clusters[0].representativeView;
	const IIndex rep2 = block2.clusters[0].representativeView;
	if (rep1 == rep2)
		return OrthoLBPMinWeight;
	// find mean colors of rep1 and rep2 in both blocks
	const Pixel32F c1_r1(FindBlockViewMeanColor(block1, rep1));
	const Pixel32F c1_r2(FindBlockViewMeanColor(block1, rep2));
	const Pixel32F c2_r1(FindBlockViewMeanColor(block2, rep1));
	const Pixel32F c2_r2(FindBlockViewMeanColor(block2, rep2));
	// color difference (working in [0,255] range)
	const float diff1 = SQRT(SQUARE(c1_r1.r-c1_r2.r) + SQUARE(c1_r1.g-c1_r2.g) + SQUARE(c1_r1.b-c1_r2.b));
	const float diff2 = SQRT(SQUARE(c2_r1.r-c2_r2.r) + SQUARE(c2_r1.g-c2_r2.g) + SQUARE(c2_r1.b-c2_r2.b));
	const float colorDiff = (diff1 + diff2) * 0.5f;
	// normalize: max possible diff is sqrt(3)*255 ~ 441; use 100 as "significant" threshold
	const float normalizedDiff = MINF(1.f, colorDiff / 100.f);
	return MAXF(OrthoLBPMinWeight, normalizedDiff);
}

void OrthoMapContext::RunBlockMRF(
	const OrthoTile& tile, OrthoBlockArr& blocks,
	const cv::Size& blockGrid) const
{
	const unsigned numBlocks = (unsigned)(blockGrid.width * blockGrid.height);
	if (numBlocks == 0)
		return;

	// collect all unique labels across all blocks
	// label 0 = undefined; label = representativeView + 1
	LBPInference inference;
	inference.SetNumNodes(numBlocks);
	inference.SetSmoothCost(OrthoSmoothnessPotts);

	// add 4-connected grid edges with color-diff-based weights
	for (int by = 0; by < blockGrid.height; ++by) {
		for (int bx = 0; bx < blockGrid.width; ++bx) {
			const unsigned nodeID = (unsigned)(by * blockGrid.width + bx);
			// right neighbor
			if (bx + 1 < blockGrid.width) {
				const unsigned rightID = nodeID + 1;
				const float weight = ComputeEdgeWeight(blocks[nodeID], blocks[rightID]);
				inference.SetNeighbors(nodeID, rightID, config.smoothnessWeight * weight);
			}
			// bottom neighbor
			if (by + 1 < blockGrid.height) {
				const unsigned bottomID = (unsigned)((by + 1) * blockGrid.width + bx);
				const float weight = ComputeEdgeWeight(blocks[nodeID], blocks[bottomID]);
				inference.SetNeighbors(nodeID, bottomID, config.smoothnessWeight * weight);
			}
		}
	}

	// set data costs per cluster label
	for (unsigned nodeID = 0; nodeID < numBlocks; ++nodeID) {
		OrthoBlock& block = blocks[nodeID];
		if (block.clusters.empty()) {
			inference.SetDataCost(0, nodeID, OrthoLBPMaxEnergy);
			continue;
		}
		FOREACHPTR(pCluster, block.clusters) {
			const LBPInference::LabelID label = (LBPInference::LabelID)(pCluster->representativeView + 1);
			const LBPInference::EnergyType dataCost = (1.f - pCluster->dataWeight) * OrthoLBPMaxEnergy;
			inference.SetDataCost(label, nodeID, dataCost);
		}
	}

	// optimize
	TD_TIMER_STARTD();
	inference.Optimize();
	DEBUG_EXTRA("OrthoMap MRF optimized: %u blocks (%s)", numBlocks, TD_TIMER_GET_FMT().c_str());

	// extract results: map label back to cluster
	for (unsigned nodeID = 0; nodeID < numBlocks; ++nodeID) {
		OrthoBlock& block = blocks[nodeID];
		const LBPInference::LabelID label = inference.GetLabel(nodeID);
		if (label == 0) {
			block.selectedCluster = NO_ID;
			continue;
		}
		const IIndex repView = (IIndex)(label - 1);
		block.selectedCluster = NO_ID;
		FOREACH(ci, block.clusters) {
			if (block.clusters[ci].representativeView == repView) {
				block.selectedCluster = ci;
				break;
			}
		}
	}
}
/*----------------------------------------------------------------*/

// -- Step 2: Extract tile view set --------------------------------

void OrthoMapContext::ExtractTileViewSet(OrthoTile& tile)
{
	std::unordered_set<IIndex> viewSet;
	FOREACHPTR(pBlock, tile.blocks) {
		if (pBlock->selectedCluster == NO_ID)
			continue;
		const uint32_t clusterID = pBlock->selectedCluster;
		// add all views in the selected cluster
		FOREACHPTR(pView, pBlock->views)
			if (pView->clusterID == clusterID)
				viewSet.insert(pView->idxView);
	}
	tile.tileViews.resize((IIndex)viewSet.size());
	IIndex i = 0;
	for (IIndex v : viewSet)
		tile.tileViews[i++] = v;
	tile.tileViews.Sort();
}
/*----------------------------------------------------------------*/

// -- Step 2: Main loop --------------------------------------------

bool OrthoMapContext::ProjectAndSelectViews()
{
	TD_TIMER_START();

	// initialize BRIEF pair table
	InitBRIEFPairs();

	// initialize image cache with memory budget
	imageCache.maxMemory = (size_t)4 * 1024 * 1024 * 1024; // 4 GB default

	unsigned numProcessed = 0;
	FOREACH(i, tiles) {
		OrthoTile& tile = tiles[i];
		if (tile.depthMap.empty())
			continue;

		// Step 2.1: find views seeing this tile
		const IIndexArr viewIndices(FindTileViews(tile));
		if (viewIndices.empty())
			continue;

		// setup block grid
		const cv::Size blockGrid(
			(tile.size.width + ORTHO_BLOCK_SIZE - 1) / ORTHO_BLOCK_SIZE,
			(tile.size.height + ORTHO_BLOCK_SIZE - 1) / ORTHO_BLOCK_SIZE);
		OrthoBlockArr blocks((uint32_t)(blockGrid.width * blockGrid.height));

		// Step 2.1: ortho-project each view onto block grid
		for (IIndex idx : viewIndices) {
			const OrthoImageCache::CachedImage& cachedImg = imageCache.UseImage(idx, scene);
			OrthoProjectView(tile, idx, cachedImg, blocks, blockGrid);
		}

		// Step 2.2: BRIEF pairwise clustering
		ClusterBlockViews(tile, blocks, blockGrid);

		// Step 2.3-2.4: MRF cluster selection
		RunBlockMRF(tile, blocks, blockGrid);

		// store results in tile for Step 3
		tile.blockGridSize = blockGrid;
		tile.blocks = std::move(blocks);
		ExtractTileViewSet(tile);

		// release depth map (no longer needed after projection)
		tile.depthMap.release();
		++numProcessed;

		DEBUG("OrthoMap tile %u/%u: %u views, %u blocks, %u selected views",
			i, tiles.GetSize(), viewIndices.size(),
			(unsigned)(blockGrid.width * blockGrid.height),
			tile.tileViews.size());
	}

	VERBOSE("OrthoMap view selection completed: %u/%u tiles processed (%s)",
		numProcessed, tiles.GetSize(), TD_TIMER_GET_FMT().c_str());
	return true;
}
/*----------------------------------------------------------------*/


// Scene entry point for orthomap generation
bool Scene::ComputeOrthoMap(float orthoResolution, unsigned tileSize, unsigned tileOverlap)
{
	if (mesh.IsEmpty()) {
		VERBOSE("error: empty mesh");
		return false;
	}
	// build configuration
	OrthoConfig config;
	config.targetGSD = orthoResolution;
	config.tileSize = tileSize;
	config.tileOverlap = tileOverlap;
	// create context and run pipeline
	OrthoMapContext ctx(*this, config);
	if (!ctx.ComputeGrid())
		return false;
	if (!ctx.RasterizeTiles())
		return false;
	// Step 2: ortho projection, clustering, MRF view selection
	if (!ctx.ProjectAndSelectViews())
		return false;
	return true;
}
/*----------------------------------------------------------------*/
