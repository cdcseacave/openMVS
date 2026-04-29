# Step 2: Ortho Projection per View & Pixel-block-Based View Selection

## Context

Step 1 (Grid Initialization & DEM Generation) is complete in `libs/MVS/SceneOrthoMap.h/.cpp`. It computes the scene AABB, GSD, tile decomposition, and rasterizes the mesh into per-tile depth maps under orthographic projection. Step 2 takes these per-tile depth maps and performs: (a) ortho-projection of view images onto each tile using the DEM, (b) pixel-block scoring with nadir preference, (c) BRIEF-based pairwise view clustering using DisjointSet to group views that agree on appearance, and (d) MRF-based cluster selection using LBPInference on a regular 2D block grid where labels are view clusters and the data term prefers larger clusters (more consensus). The outcome is a per-block selected cluster (with its inlier views) for each tile, consumed by Step 3 (multi-band blending).

**Files to modify**: `libs/MVS/SceneOrthoMap.h` (structs + declarations), `libs/MVS/SceneOrthoMap.cpp` (all implementation).

---

## A. New Data Structures (in SceneOrthoMap.h)

### A.1 Compile-Time Constants (SceneOrthoMap.cpp defines section)

```cpp
#define ORTHO_BLOCK_SIZE      4    // pixels per block side (power of 2)
#define ORTHO_BRIEF_SAMPLES  32    // 16 within block + 16 in 8-neighbors
#define ORTHO_BRIEF_PAIRS   256    // 256-bit descriptor
#define ORTHO_LBP_MAX_ITERS 100    // max LBP convergence iterations
```

### A.2 BRIEFDescriptor (SceneOrthoMap.h, standalone struct)

```cpp
struct BRIEFDescriptor {
    uint64_t bits[4];  // 256 bits
    void Clear();
    void SetBit(unsigned i);
    unsigned HammingDistance(const BRIEFDescriptor& o) const;  // uses PopCnt() from Maths.h
    float Similarity(const BRIEFDescriptor& o) const;          // 1.0 - hamming/256
};
```

Use `PopCnt(uint64_t)` from `libs/Common/Maths.h` (cross-platform: `__builtin_popcountll` on GCC, SSE intrinsics on MSVC).

### A.3 OrthoBlockView — per-block candidate view data

```cpp
struct OrthoBlockView {
    IIndex idxView;         // scene image index
    float nadirWeight;      // cos^2(viewDir.z), [0,1]
    Pixel32F meanColor;     // mean BGR in block from this view (for smoothness edge weights)
    uint32_t clusterID;     // assigned after BRIEF clustering (0-based)
};
typedef cList<OrthoBlockView, const OrthoBlockView&, 0, 8, uint32_t> OrthoBlockViewArr;
```

### A.4 OrthoBlockCluster — per-block cluster data (after BRIEF clustering)

```cpp
struct OrthoBlockCluster {
    IIndex representativeView;  // MRF label: view with best nadir weight in cluster
    float bestNadirWeight;      // nadir weight of representative
    uint32_t size;              // number of views in this cluster
    float dataWeight;           // MRF data term weight (combines consensus + nadir)
};
typedef cList<OrthoBlockCluster, const OrthoBlockCluster&, 0, 4, uint32_t> OrthoBlockClusterArr;
```

### A.5 OrthoBlock — per-block aggregated data

```cpp
struct OrthoBlock {
    OrthoBlockViewArr views;        // candidate views (with cluster assignments)
    OrthoBlockClusterArr clusters;  // BRIEF-derived clusters (sorted by size descending)
    IIndex selectedCluster;         // MRF-selected cluster index (NO_ID if none)
    OrthoBlock() : selectedCluster(NO_ID) {}
};
typedef cList<OrthoBlock, const OrthoBlock&, 2, 16, uint32_t> OrthoBlockArr;
```

### A.6 OrthoConfig additions

Add to `OrthoConfig`:
```cpp
float briefSimilarityThreshold;  // BRIEF pairwise clustering threshold, default 0.65
float consensusWeight;           // alpha: weight of cluster size vs nadir in data term, default 0.6
float smoothnessWeight;          // lambda: MRF smoothness weight, default 1.0
```

### A.7 OrthoImageCache — thread-safe LRU image cache

Follows the `DMapCache` pattern (libs/MVS/DMapCache.h): `ListFIFO<IIndex>` for LRU tracking, `std::mutex` for thread safety, memory-bounded eviction.

```cpp
struct OrthoImageCache {
    struct CachedImage {
        Image8U3 color;      // loaded BGR pixels
        Image32F gray;       // grayscale float [0,1]
        size_t memoryBytes;  // tracked for eviction
    };
    std::unordered_map<IIndex, CachedImage> images;
    ListFIFO<IIndex> fifo;
    mutable std::mutex mutex;
    size_t maxMemory;
    size_t usedMemory;

    const CachedImage& UseImage(IIndex idxImage, const Scene& scene);
    void Eject();           // evict LRU until under budget
    void EjectOldest();     // evict single oldest entry
};
```

**Thread safety**: Lock mutex for `fifo`/`images`/`usedMemory` access. Load image data (ReloadImage) outside the lock to avoid blocking other threads. Use a per-image loading flag to prevent double-loading.

### A.8 OrthoTile extensions (Step 2 output)

Add to `OrthoTile`:
```cpp
IIndexArr tileViews;              // unique views selected for this tile (from all block clusters)
OrthoBlockArr blocks;             // per-block data with cluster assignments and MRF selection
cv::Size blockGridSize;           // block grid dimensions
```

### A.9 OrthoMapContext new methods

```cpp
// Step 2 methods
IIndexArr FindTileViews(const OrthoTile& tile) const;
void OrthoProjectView(const OrthoTile& tile, IIndex idxView,
                       const OrthoImageCache::CachedImage& img,
                       OrthoBlockArr& blocks, const cv::Size& blockGrid) const;
void ClusterBlockViews(const OrthoTile& tile,
                        OrthoBlockArr& blocks,
                        const cv::Size& blockGrid) const;
void RunBlockMRF(const OrthoTile& tile, OrthoBlockArr& blocks,
                  const cv::Size& blockGrid) const;
bool ProjectAndSelectViews();  // main Step 2 entry point

// Helpers
Point3 OrthoPixelToWorld(const OrthoTile& tile, float px, float py, Depth depth) const;
BRIEFDescriptor ComputeBRIEFDescriptor(const OrthoTile& tile,
                                        unsigned bx, unsigned by,
                                        const cv::Size& blockGrid,
                                        IIndex idxView,
                                        const OrthoImageCache::CachedImage& img) const;
```

Add member:
```cpp
OrthoImageCache imageCache;  // shared image cache
```

---

## B. Step 2.1 — Find Views per Tile & Ortho Projection

### B.1 FindTileViews

For each valid scene image, test if any of the 8 corners of `tile.worldBounds` (AABB) project inside the view's image bounds. This is simpler and more reliable than frustum construction for perspective cameras.

```
FindTileViews(tile):
    result = []
    for each valid image idx:
        camera = image.camera  // already has P cached
        // project all 8 AABB corners into view
        corners = tile.worldBounds.GetCorners()  // 8 corners
        anyInside = false
        for each corner:
            pt2D = camera.ProjectPointP(corner)
            if 0 <= pt2D.x < width && 0 <= pt2D.y < height:
                anyInside = true; break
        if anyInside:
            result.push_back(idx)
    return result
```

**Note**: Also check that the camera is on the correct side of the tile (camera.C.z > tile.worldBounds.ptMin.z) to avoid back-projection.

### B.2 OrthoProjectView

For each pixel block, project the block center through the DEM into the source view. If visible, compute nadir weight and mean color.

```
OrthoProjectView(tile, idxView, img, blocks, blockGrid):
    viewCamera = scene.images[idxView].camera
    viewDir = viewCamera.Direction()
    cosNadir = ABS(viewDir.z)
    nadirWeight = cosNadir * cosNadir  // quadratic nadir preference

    for by = 0..blockGrid.height-1:
        for bx = 0..blockGrid.width-1:
            OrthoBlock& block = blocks[by * blockGrid.width + bx]
            // Block center in tile pixel coords
            cx = (bx + 0.5f) * ORTHO_BLOCK_SIZE
            cy = (by + 0.5f) * ORTHO_BLOCK_SIZE
            depth = tile.depthMap(ROUND2INT(cy), ROUND2INT(cx))
            if depth == 0: continue  // no surface

            // Ortho pixel → 3D world via DEM
            worldPt = tile.camera.TransformPointOrthoI2W(Point3(cx, cy, depth))

            // Project into source view
            pt2D = viewCamera.TransformPointW2I(Cast<REAL>(worldPt))
            if !img.color.isInsideWithBorder<float,2>(pt2D): continue

            // Compute mean color over block (sample all BLOCK_SIZE^2 pixels)
            meanColor = sampleBlockMeanColor(tile, bx, by, viewCamera, img)

            // Add candidate
            OrthoBlockView bv;
            bv.idxView = idxView;
            bv.nadirWeight = nadirWeight;
            bv.totalWeight = nadirWeight;
            bv.briefSimilarity = 0;  // computed in step 2.2
            bv.meanColor = meanColor;
            block.views.InsertSort(bv, [](a,b){ return a.totalWeight > b.totalWeight; });
```

**sampleBlockMeanColor**: Iterate all pixels in the block, project each through DEM into view, bilinear sample color, return average. Use `img.color.sample<LinearSampler, Pixel32F>()` for bilinear interpolation. Skip pixels with depth==0 or that project outside the view.

### B.3 OrthoPixelToWorld helper

```cpp
inline Point3 OrthoMapContext::OrthoPixelToWorld(
    const OrthoTile& tile, float px, float py, Depth depth) const {
    return tile.camera.TransformPointOrthoI2W(Point3(px, py, depth));
}
```

Uses existing `Camera::TransformPointOrthoI2W()` (Camera.h:410).

### B.4 Image Cache Implementation

```
UseImage(idxImage, scene):
    lock(mutex)
    if images.count(idxImage):
        fifo.Put(idxImage)
        return images[idxImage]
    unlock(mutex)

    // Load outside lock
    scene.images[idxImage].ReloadImage(0, true)
    CachedImage cached;
    cached.color = scene.images[idxImage].image.clone()
    cv::cvtColor(cached.color, cached.gray, cv::COLOR_BGR2GRAY)
    cached.gray.convertTo(cached.gray, CV_32F, 1.0/255.0)
    cached.memoryBytes = color_bytes + gray_bytes

    lock(mutex)
    images[idxImage] = std::move(cached)
    usedMemory += cached.memoryBytes
    fifo.Put(idxImage)
    Eject()  // evict if over budget
    return images[idxImage]
```

Memory budget: `min(availableRAM * 0.3, 4GB)`. Each image ~96 MB at 4000x3000, so ~20-40 images fit.

---

## C. Step 2.2 — BRIEF Descriptor Computation

### C.1 Static BRIEF Pair Table

```cpp
static uint8_t briefPairs[ORTHO_BRIEF_PAIRS][2];  // 256 pairs of indices [0..31]
```

Initialize deterministically with a fixed-seed PRNG. Each pair selects two of the 32 sample points.

### C.2 Sample Point Generation

Per block at (bx, by), generate 32 3D points on the DEM surface:
- **16 points within block**: 4x4 regular sub-grid at sub-pixel centers
- **16 points in 8-connected neighbors**: 2 per neighbor at deterministic sub-positions (1/3, 2/3)
- Out-of-bounds or depth==0 samples marked invalid

### C.3 Descriptor Computation

For a given view and block:
1. Generate 32 sample 3D points
2. Project each into view camera, sample grayscale intensity
3. For each of 256 comparison pairs: if `intensity[a] < intensity[b]`, set bit
4. Skip pairs where either sample is invalid

### C.4 Pairwise Clustering with DisjointSet (replaces reference-based similarity)

**Key change**: Instead of picking one reference view and comparing all others to it (which biases toward the reference), compute ALL pairwise similarities and cluster views using Union-Find. This produces unbiased, symmetric clusters where each cluster represents a set of views that agree on what the block looks like.

**Function**: `void OrthoMapContext::ClusterBlockViews(tile, blocks, blockGrid)`

Per block `b` with N candidate views:

```
ClusterBlockViews(tile, blocks, blockGrid):
    for each block b:
        N = b.views.size()
        if N < 2:
            // Single view → single cluster of size 1
            b.clusters.resize(1)
            b.clusters[0] = { b.views[0].idxView, b.views[0].nadirWeight, 1, ... }
            b.views[0].clusterID = 0
            continue

        // Step A: Compute BRIEF descriptors for ALL views
        BRIEFDescriptor descs[N]   // stack-allocated, N ≤ ~50
        for i = 0..N-1:
            descs[i] = ComputeBRIEFDescriptor(tile, bx, by, blockGrid,
                                               b.views[i].idxView, cachedImg[i])

        // Step B: All-pairwise similarities + Union-Find clustering
        DisjointSet<uint32_t> ds(N)
        for i = 0..N-2:
            for j = i+1..N-1:
                if descs[i].Similarity(descs[j]) >= config.briefSimilarityThreshold:
                    ds.Union(i, j)

        // Step C: Extract cluster assignments
        ds.CompressAllPaths()
        std::vector<uint32_t> componentIDs(N)
        unsigned numClusters = ds.GetComponents(componentIDs)
        for i = 0..N-1:
            b.views[i].clusterID = componentIDs[i]

        // Step D: Build cluster stats
        b.clusters.resize(numClusters)
        for each cluster c (0..numClusters-1):
            c.size = 0
            c.bestNadirWeight = 0
            c.representativeView = NO_ID
        for i = 0..N-1:
            cluster = b.clusters[b.views[i].clusterID]
            cluster.size++
            if b.views[i].nadirWeight > cluster.bestNadirWeight:
                cluster.bestNadirWeight = b.views[i].nadirWeight
                cluster.representativeView = b.views[i].idxView

        // Step E: Compute data weights
        bool allSingletons = (numClusters == N)  // every view disagrees with every other
        for each cluster c:
            if allSingletons:
                // Fallback: pure nadir selection, let MRF use spatial context
                c.dataWeight = c.bestNadirWeight
            else:
                // alpha * consensus + (1-alpha) * nadir quality
                consensus = (float)c.size / (float)N
                c.dataWeight = config.consensusWeight * consensus
                              + (1.f - config.consensusWeight) * c.bestNadirWeight

        // Sort clusters by size descending (largest first)
        b.clusters.Sort([](a, b) { return a.size > b.size; })
```

**Threshold `tau = 0.65`** (configurable via `config.briefSimilarityThreshold`): Same-surface views typically produce BRIEF similarity > 0.7; transient-affected views produce < 0.5. The 0.65 threshold sits in the clean separation gap. The BRIEF descriptor space is well-calibrated (256 independent bits), so a fixed threshold works reliably.

**Memory**: Per block with N=20 views: 20 descriptors × 32 bytes = 640 bytes + DisjointSet(20) = 40 bytes + 190 pairwise comparisons. All on stack, discarded after clustering. Only cluster assignments + stats persist.

**Complexity**: O(N² × 256) for pairwise BRIEF comparisons per block. With N ≤ 50 and 1M blocks, this is ~1M × 50² × 1 = 2.5B comparisons, each being 4 popcount operations. At ~1 ns per comparison, ~2.5 seconds total — acceptable.

### C.5 Cluster Properties

After clustering, each block has K clusters (K ≤ N). Each cluster:
- **representativeView**: The view with the best nadir weight in the cluster. This becomes the MRF label. Labels are comparable across blocks: if two neighboring blocks both have a cluster whose representative is view V5, they share label V5 and the Potts smoothness cost is 0.
- **size**: Number of agreeing views. Larger clusters = stronger consensus = more likely to represent the true scene (not transient objects).
- **dataWeight**: Combined consensus + nadir quality, used for MRF data term.
- **Member views**: All views in the cluster (accessible via `views[i].clusterID == c`). These become the inlier set for Step 3 blending.

**Edge cases**:
- **N=1**: Single cluster, MRF has no choice. Data cost = `1.0 - nadirWeight`.
- **All views agree** (1 cluster of size N): Strong consensus, very low data cost. MRF trivially assigns this label.
- **All singletons** (N clusters of size 1): Every view disagrees. Fallback to pure nadir selection; MRF uses spatial context from neighbors to choose.
- **Equal-sized clusters** (e.g., 2 clusters of 2 views each): Data term is ambiguous. This is the key case where the MRF smoothness term matters — it looks at neighbors to break the tie. If surrounding blocks prefer cluster A, this block follows.

---

## D. Step 2.3-2.4 — MRF-Based Cluster Selection

### D.1 MRF Solver Choice: LBPInference with Potts + Edge Weights

Uses **LBPInference** (`libs/Math/LBP.h`) following the texturing pipeline pattern (`SceneTexture.cpp:1131-1193`). Well-tested, OpenMP-parallel, supports per-node label sets.

**Key pattern** (SceneTexture.cpp:1130-1194):
- Potts smoothness callback: returns 0 if same label, 1.0 if different
- Color-difference information baked into **edge weights** via `SetNeighbors(n1, n2, weight)`
- Data cost = `(1.0 - normalizedWeight) * MaxEnergy`

### D.2 MRF Labels

Each MRF label is `representativeView + 1` (label 0 = undefined, matching texturing convention). Labels are globally meaningful: a label identifies a specific scene view. When two neighboring blocks both select a cluster whose representative is the same view, the Potts smoothness cost is 0.

Per block, the valid label set = { cluster.representativeView + 1 : for each cluster in block }. Labels not present in a block have infinite data cost (not registered via SetDataCost).

### D.3 Data Term

```
E_data(b, label_k) = (1.0 - cluster_k.dataWeight) * MaxEnergy

where dataWeight = alpha * (clusterSize / N) + (1 - alpha) * bestNadirWeight
      alpha = config.consensusWeight (default 0.6)
      MaxEnergy = 1.0 (matching LBPMaxEnergy from texturing)
```

- Larger clusters → higher dataWeight → lower data cost → preferred
- Better nadir → higher dataWeight → lower data cost → preferred
- Alpha=0.6 means consensus is weighted more than nadir quality (transient removal is primary goal)

### D.4 Graph Structure & Edge Weights

Regular 4-connected grid. Add edges only once (n1 < n2, same pattern as texturing).

```
RunBlockMRF(tile, blocks, blockGrid):
    numBlocks = blockGrid.width * blockGrid.height
    LBPInference inference;
    inference.SetNumNodes(numBlocks);
    inference.SetSmoothCost(OrthoSmoothnessPotts);

    // Add edges with color-diff-based weights
    for by, bx:
        nodeID = by * width + bx
        if bx+1 < width:
            rightID = nodeID + 1
            weight = ComputeEdgeWeight(blocks[nodeID], blocks[rightID])
            inference.SetNeighbors(nodeID, rightID, config.smoothnessWeight * weight)
        if by+1 < height:
            bottomID = nodeID + width
            weight = ComputeEdgeWeight(blocks[nodeID], blocks[bottomID])
            inference.SetNeighbors(nodeID, bottomID, config.smoothnessWeight * weight)

    // Set data costs per cluster label
    for each block b at nodeID:
        if b.clusters.empty():
            inference.SetDataCost(0, nodeID, MaxEnergy)
            continue
        for each cluster c in b.clusters:
            label = c.representativeView + 1
            dataCost = (1.0f - c.dataWeight) * MaxEnergy
            inference.SetDataCost(label, nodeID, dataCost)

    inference.Optimize();

    // Extract results: map label back to cluster
    for each block b at nodeID:
        label = inference.GetLabel(nodeID)
        if label == 0:
            b.selectedCluster = NO_ID
        else:
            IIndex repView = label - 1
            // Find which cluster has this representative
            FOREACH(ci, b.clusters):
                if b.clusters[ci].representativeView == repView:
                    b.selectedCluster = ci; break
```

### D.5 Edge Weight Computation (color-diff based)

For each edge (n1, n2), measure how visible a label change would be by comparing boundary colors of the two blocks' largest-cluster representatives:

```
ComputeEdgeWeight(block1, block2):
    if block1.clusters.empty() || block2.clusters.empty():
        return LBPMinWeight  // 0.5, matching texturing

    // Get representative views of the two largest clusters
    rep1 = block1.clusters[0].representativeView  // largest cluster (sorted)
    rep2 = block2.clusters[0].representativeView

    if rep1 == rep2:
        return LBPMinWeight  // same view → minimal penalty for label changes

    // Find mean colors of rep1 and rep2 in both blocks
    c1_rep1 = findMeanColor(block1, rep1)  // from OrthoBlockView.meanColor
    c1_rep2 = findMeanColor(block1, rep2)
    c2_rep1 = findMeanColor(block2, rep1)
    c2_rep2 = findMeanColor(block2, rep2)

    // Color difference: average of cross-view differences at both blocks
    colorDiff = (norm(c1_rep1 - c1_rep2) + norm(c2_rep1 - c2_rep2)) * 0.5f
    // Normalize to [0,1] (max possible = sqrt(3) ≈ 1.73 for Pixel32F in [0,1])
    colorDiff = MINF(1.0f, colorDiff / 0.4f)

    return MAXF(LBPMinWeight, colorDiff)
```

Note: if a representative view is not visible in the neighbor block (no OrthoBlockView entry), use LBPMinWeight as fallback. This is uncommon because neighboring blocks typically share most views.

### D.6 Smoothness Callback

```cpp
constexpr LBPInference::EnergyType OrthoLBPMaxEnergy(1.f);
constexpr LBPInference::EnergyType OrthoLBPMinWeight(0.5f);

static LBPInference::EnergyType STCALL OrthoSmoothnessPotts(
    LBPInference::NodeID, LBPInference::NodeID,
    LBPInference::LabelID l1, LBPInference::LabelID l2) {
    return l1 == l2 && l1 != 0 && l2 != 0 ? 0.f : OrthoLBPMaxEnergy;
}
```

Identical pattern to `SmoothnessPotts` in SceneTexture.cpp:133. The edge weight (from D.5) scales the Potts penalty.

---

## E. Step 2.5 — Per-Tile MRF with Overlap

**V1 approach**: Independent per-tile MRFs using existing 16-pixel tile overlap. The overlap is sufficient because Step 3 (multi-band blending) smooths boundary inconsistencies. If artifacts appear in practice, increase overlap to 128 pixels (32 blocks).

Block grid dimensions:
```
blockGrid.width  = (tile.size.width  + ORTHO_BLOCK_SIZE - 1) / ORTHO_BLOCK_SIZE
blockGrid.height = (tile.size.height + ORTHO_BLOCK_SIZE - 1) / ORTHO_BLOCK_SIZE
```

Edge blocks may be smaller than ORTHO_BLOCK_SIZE. Sampling functions must bounds-check.

---

## F. Main Loop — ProjectAndSelectViews()

### F.1 Parallelism Strategy

**Sequential tile processing** (NOT parallel outer loop). Reasons:
1. ~960 MB per tile — parallel tiles would exceed memory on most machines
2. LBPInference::Optimize() uses internal `#pragma omp parallel for` — nested OpenMP causes issues
3. Sequential outer + parallel LBP inner gives good CPU utilization

```cpp
bool OrthoMapContext::ProjectAndSelectViews() {
    TD_TIMER_START();
    // Initialize image cache
    imageCache.maxMemory = min(GetAvailableMemory() * 0.3, 4GB);
    // Initialize BRIEF pair table (once)
    InitBRIEFPairs();

    FOREACH(i, tiles) {
        OrthoTile& tile = tiles[i];
        if (tile.depthMap.empty()) continue;

        // 2.1: Find views seeing this tile
        IIndexArr viewIndices = FindTileViews(tile);
        if (viewIndices.empty()) continue;

        // Setup block grid
        const cv::Size blockGrid(
            (tile.size.width + ORTHO_BLOCK_SIZE - 1) / ORTHO_BLOCK_SIZE,
            (tile.size.height + ORTHO_BLOCK_SIZE - 1) / ORTHO_BLOCK_SIZE);
        OrthoBlockArr blocks(blockGrid.width * blockGrid.height);

        // 2.1: Ortho-project each view onto block grid (nadir weight + mean color)
        for (IIndex idx : viewIndices) {
            const auto& img = imageCache.UseImage(idx, scene);
            OrthoProjectView(tile, idx, img, blocks, blockGrid);
        }

        // 2.2: BRIEF pairwise clustering — all-vs-all similarity + DisjointSet
        ClusterBlockViews(tile, blocks, blockGrid);

        // 2.3-2.4: MRF cluster selection — LBP with Potts + color-diff edge weights
        RunBlockMRF(tile, blocks, blockGrid);

        // Store results in tile for Step 3 (blending)
        tile.blockGridSize = blockGrid;
        tile.blocks = std::move(blocks);
        // Build tileViews: deduplicated list of all views in selected clusters
        ExtractTileViewSet(tile);

        // Release depthMap (no longer needed after projection)
        tile.depthMap.release();
    }

    VERBOSE("OrthoMap view selection: %u tiles (%s)",
        tiles.GetSize(), TD_TIMER_GET_FMT().c_str());
    return true;
}
```

### F.2 Scene Entry Point Update

Add `ctx.ProjectAndSelectViews()` call after `ctx.RasterizeTiles()` in `Scene::ComputeOrthoMap()`.

---

## G. Memory Budget

Per tile at 4096x4096, BLOCK_SIZE=4, ~20 candidate views:
| Component | Size |
|-----------|------|
| DepthMap (4096x4096 x float) | 64 MB |
| OrthoBlockArr (1M blocks x ~20 views x 28B) | ~560 MB |
| OrthoBlockClusterArr (1M blocks x ~5 clusters x 20B) | ~100 MB |
| LBPInference graph (~4M edges with messages) | ~256 MB |
| BRIEF descriptors (computed per-block, stack, discarded) | ~0 MB |
| **Total per tile** | **~980 MB** |

Sequential processing: peak ~980 MB + image cache (~2 GB) = ~3 GB total. Fits 8 GB machines.

---

## H. Implementation Sequence

### Phase 1: Data Structures & Image Cache (Steps 1-5)
1. Add `BRIEFDescriptor`, `OrthoBlockView`, `OrthoBlockCluster`, `OrthoBlock` structs to `SceneOrthoMap.h`
2. Add new fields to `OrthoConfig` (briefSimilarityThreshold, consensusWeight, smoothnessWeight)
3. Add `OrthoImageCache` struct to `SceneOrthoMap.h`
4. Implement `OrthoImageCache::UseImage/Eject/EjectOldest` in `SceneOrthoMap.cpp`
5. Add `OrthoPixelToWorld` helper, extend `OrthoTile` with Step 2 output fields, add `#include "../Math/LBP.h"` and `#include "../Math/DisjointSet.h"`

### Phase 2: View Finding & Projection (Steps 6-8)
6. Implement `FindTileViews()` — AABB corner projection test
7. Implement `OrthoProjectView()` — per-block DEM-based reprojection + nadir weight + mean color
8. Implement block mean color sampling (bilinear over all block pixels)

### Phase 3: BRIEF Pairwise Clustering (Steps 9-13)
9. Implement `InitBRIEFPairs()` — deterministic 256-pair table with fixed seed
10. Implement sample point generation (16 within block + 16 from 8-neighbors)
11. Implement `ComputeBRIEFDescriptor()` using `PopCnt` from `Maths.h`
12. Implement `ClusterBlockViews()` — all-pairwise BRIEF + DisjointSet clustering + cluster stats
13. Handle edge cases: N=1, all singletons, all-in-one-cluster

### Phase 4: MRF Cluster Selection (Steps 14-17)
14. Implement `ComputeEdgeWeight()` — color-diff between largest-cluster representatives
15. Implement `RunBlockMRF()` — LBPInference setup, Potts + edge weights, cluster label extraction
16. Implement `OrthoSmoothnessPotts` callback (identical to texturing's SmoothnessPotts)
17. Implement `ExtractTileViewSet()` — collect unique views from all selected clusters

### Phase 5: Integration (Steps 18-20)
18. Implement `ProjectAndSelectViews()` — main sequential tile loop
19. Update `Scene::ComputeOrthoMap()` to call `ProjectAndSelectViews()` after `RasterizeTiles()`
20. Add all new method declarations to `OrthoMapContext` in header

---

## I. Key Reuse Points

| Component | Source | Usage |
|-----------|--------|-------|
| LBPInference | `libs/Math/LBP.h` | MRF solver (used as-is) |
| DisjointSet | `libs/Math/DisjointSet.h` | Union-Find for BRIEF-based view clustering |
| Potts smoothness pattern | `SceneTexture.cpp:130-134` | Smoothness callback |
| ListFIFO | `libs/Common/ListFIFO.h` | LRU tracking in image cache |
| DMapCache pattern | `libs/MVS/DMapCache.h` | Blueprint for OrthoImageCache |
| PopCnt | `libs/Common/Maths.h:487-498` | Cross-platform Hamming distance |
| Camera::TransformPointOrthoI2W | `Camera.h:410` | Ortho pixel → world |
| Camera::ProjectPointP | `Camera.h:288` | World → view projection |
| TImage::sample | image sampling | Bilinear interpolation |
| TImage::isInsideWithBorder | image bounds check | Safe sampling |

---

## J. Verification

1. **Projection test**: Project a known view onto a flat tile (horizontal plane at Z=0). The ortho-projected image should match the original view's ground-plane region pixel-for-pixel (within bilinear sampling tolerance).
2. **BRIEF pairwise test**: Compute descriptors for the same block from two nearly-identical views. Similarity should be > 0.9. Compute for same block where one view sees a car and the other doesn't — similarity should be < 0.6.
3. **Clustering test**: Given 5 views where V1,V3 see ground and V2,V4,V5 see ground+car, clustering should produce 2 clusters: {V1,V3} and {V2,V4,V5} (or similar grouping). Verify cluster sizes and representative selection.
4. **MRF consensus test**: On a synthetic case with 2 equal-sized clusters, verify the MRF uses spatial context from neighbors to break ties consistently (not random).
5. **MRF coherence test**: On a 2-view scene, verify the MRF produces spatially coherent regions (not checkerboard noise).
6. **End-to-end**: Run on a real aerial dataset. Inspect per-tile results — should show large coherent cluster regions with boundaries along natural seam lines. Transient objects (if present) should be in non-selected clusters.
7. **Memory**: Monitor RSS during processing. Should stay under ~4 GB for sequential tile processing with 20 views.
