/*
* SceneOrthoMap.h
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

#ifndef _MVS_SCENEORTHOMAP_H_
#define _MVS_SCENEORTHOMAP_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "../Common/ListFIFO.h"
#include <mutex>
#include <unordered_map>


// S T R U C T S ///////////////////////////////////////////////////

namespace MVS {

// forward declarations
class MVS_API Scene;

// pipeline configuration parameters
struct MVS_API OrthoConfig {
	float targetGSD;         // 0 = auto-compute from cameras; >0 = user override (m/px)
	unsigned tileSize;       // tile dimension in pixels
	unsigned tileOverlap;    // overlap in pixels for blending seams
	float nadirThreshold;    // abs(dir.z) threshold for nadir camera filter
	float marginFactor;      // AABB expansion: max(0.1f, extent * marginFactor)
	// Step 2 parameters
	float briefSimilarityThreshold; // BRIEF pairwise clustering threshold (0=skip clustering)
	float consensusWeight;          // alpha: weight of cluster size vs nadir in data term
	float smoothnessWeight;         // lambda: MRF smoothness weight

	OrthoConfig()
		: targetGSD(0)
		, tileSize(4096)
		, tileOverlap(16)
		, nadirThreshold(0.3f)
		, marginFactor(0.01f)
		, briefSimilarityThreshold(0.65f)
		, consensusWeight(0.6f)
		, smoothnessWeight(1.0f) {}
};

// computed grid layout
struct MVS_API OrthoGrid {
	AABB3f sceneAABB;    // expanded mesh AABB
	float gsd;           // final GSD (m/px)
	cv::Size sizePx;     // total orthomap dimensions in pixels
	Point2u tiles;       // tile counts (x, y)

	OrthoGrid() : gsd(0), tiles(0, 0) {}
};

// 256-bit BRIEF descriptor for local texture structure comparison
struct BRIEFDescriptor {
	uint64_t bits[4]; // 256 bits = 4 x 64-bit words
	inline void Clear() { bits[0]=bits[1]=bits[2]=bits[3]=0; }
	inline void SetBit(unsigned i) { bits[i/64] |= (uint64_t(1) << (i%64)); }
	inline unsigned HammingDistance(const BRIEFDescriptor& o) const {
		return (unsigned)(
			PopCnt(bits[0]^o.bits[0]) +
			PopCnt(bits[1]^o.bits[1]) +
			PopCnt(bits[2]^o.bits[2]) +
			PopCnt(bits[3]^o.bits[3]));
	}
	inline float Similarity(const BRIEFDescriptor& o) const {
		return 1.f - (float)HammingDistance(o) / 256.f;
	}
};

// per-block candidate view data
struct OrthoBlockView {
	IIndex idxView;      // scene image index
	float nadirWeight;   // cos^2(viewDir.z), [0,1]
	Pixel32F meanColor;  // mean BGR in block from this view (for smoothness edge weights)
	uint32_t clusterID;  // assigned after BRIEF clustering (0-based)
};
typedef SEACAVE::cList<OrthoBlockView, const OrthoBlockView&, 0, 8, uint32_t> OrthoBlockViewArr;

// per-block cluster data (after BRIEF clustering)
struct OrthoBlockCluster {
	IIndex representativeView; // MRF label: view with best nadir weight in cluster
	float bestNadirWeight;     // nadir weight of representative
	uint32_t size;             // number of views in this cluster
	float dataWeight;          // MRF data term weight (combines consensus + nadir)
};
typedef SEACAVE::cList<OrthoBlockCluster, const OrthoBlockCluster&, 0, 4, uint32_t> OrthoBlockClusterArr;

// per-block aggregated data
struct OrthoBlock {
	OrthoBlockViewArr views;        // candidate views (with cluster assignments)
	OrthoBlockClusterArr clusters;  // BRIEF-derived clusters (sorted by size descending)
	uint32_t selectedCluster;       // MRF-selected cluster index (NO_ID if none)
	OrthoBlock() : selectedCluster(NO_ID) {}
};
typedef SEACAVE::cList<OrthoBlock, const OrthoBlock&, 2, 16, uint32_t> OrthoBlockArr;

// thread-safe LRU image cache for orthomap pipeline
struct OrthoImageCache {
	struct CachedImage {
		Image8U3 color;      // loaded BGR pixels
		Image32F gray;       // grayscale float [0,1]
		size_t memoryBytes;  // tracked for eviction budget
		CachedImage() : memoryBytes(0) {}
	};
	std::unordered_map<IIndex, CachedImage> images;
	ListFIFO<IIndex> fifo;
	mutable std::mutex mutex;
	size_t maxMemory;
	size_t usedMemory;

	OrthoImageCache() : maxMemory(0), usedMemory(0) {}

	const CachedImage& UseImage(IIndex idxImage, Scene& scene);
	void Eject();
	void EjectOldest();
};

// per-tile data
struct MVS_API OrthoTile {
	Point2u tile;        // tile grid indices
	Point2u p0;          // start pixel in global grid
	cv::Size size;       // tile pixel dimensions (may be smaller at edges)
	AABB3f worldBounds;  // world XY region this tile covers (Z = full scene Z range)
	Camera camera;       // orthographic camera for this tile
	DepthMap depthMap;   // G-buffer: depth (camera-space Z, min = highest world surface)
	// Step 2 output
	IIndexArr tileViews;    // unique views in selected clusters for this tile
	OrthoBlockArr blocks;   // per-block data with cluster assignments and MRF selection
	cv::Size blockGridSize; // block grid dimensions
};
typedef SEACAVE::cList<OrthoTile, const OrthoTile&, 2, 16, uint32_t> OrthoTileArr;

// orchestrator: not a Scene member; takes Scene& reference
class MVS_API OrthoMapContext {
public:
	Scene& scene;
	OrthoConfig config;
	OrthoGrid grid;
	OrthoTileArr tiles;
	Mesh::Octree octree;
	OrthoImageCache imageCache;

public:
	OrthoMapContext(Scene& _scene, const OrthoConfig& _config = OrthoConfig())
		: scene(_scene), config(_config) {}

	// Step 1.2: compute GSD from nadir cameras or use override
	float ComputeGSD() const;

	// Step 1.3: compute grid layout and create tile array
	bool ComputeGrid();

	// Step 1.4: setup orthographic camera for a tile
	Camera SetupTileCamera(const OrthoTile& tile) const;

	// Step 1.5: build octree spatial index from mesh face centroids
	void BuildSpatialIndex();

	// Step 1.6: rasterize mesh faces into a single tile's depth map
	void RasterizeTile(OrthoTile& tile, const Mesh::FaceIdxArr& tileFaces) const;

	// Step 1: main loop -- build index, rasterize all tiles (parallel)
	bool RasterizeTiles();

	// Step 2: ortho project views, cluster, MRF select -- sequential tile loop
	bool ProjectAndSelectViews();

private:
	// Step 2 helpers
	Point3 OrthoPixelToWorld(const OrthoTile& tile, float px, float py, Depth depth) const;
	IIndexArr FindTileViews(const OrthoTile& tile) const;
	void OrthoProjectView(const OrthoTile& tile, IIndex idxView,
		const OrthoImageCache::CachedImage& img,
		OrthoBlockArr& blocks, const cv::Size& blockGrid) const;
	BRIEFDescriptor ComputeBRIEFDescriptor(const OrthoTile& tile,
		unsigned bx, unsigned by, const cv::Size& blockGrid,
		IIndex idxView, const OrthoImageCache::CachedImage& img) const;
	void ClusterBlockViews(const OrthoTile& tile,
		OrthoBlockArr& blocks, const cv::Size& blockGrid) const;
	void RunBlockMRF(const OrthoTile& tile,
		OrthoBlockArr& blocks, const cv::Size& blockGrid) const;
	static void ExtractTileViewSet(OrthoTile& tile);
};
/*----------------------------------------------------------------*/

} // namespace MVS

#endif // _MVS_SCENEORTHOMAP_H_
