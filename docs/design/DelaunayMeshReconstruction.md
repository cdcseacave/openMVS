# Delaunay Mesh Reconstruction

## 1. Purpose and scope

`Scene::ReconstructMesh` (`libs/MVS/SceneReconstruct.cpp`) builds a watertight mesh from a dense,
view-annotated point cloud: it triangulates the points with CGAL Delaunay, accumulates a
Labatut/Pons/Keriven visibility energy on the tetrahedralization, solves an s-t min-cut, and
extracts the cut boundary as the mesh surface. `Mesh::Clean` (`libs/MVS/MeshHalfMesh.cpp`, delegating
to the `halfmesh` library) then repairs and simplifies that raw surface — this is where the
long-edge ("webbing") gate lives.

Scope: the reconstruction and cleaning stages driven by `apps/ReconstructMesh/ReconstructMesh.cpp`.
`RefineMesh` (shape refinement against images) and `TextureMesh` are separate pipeline stages with
their own design documents and are out of scope here.

---

## 2. Algorithm as implemented

### 2.1 Point insertion / visibility weighting (`SceneReconstruct.cpp:Scene::ReconstructMesh`)

Points are inserted into a CGAL Delaunay triangulation in spatial-sort order. If
`distInsert > 0`, a candidate point is merged into the nearest existing vertex instead of inserted
as its own vertex when, in every one of its views, it projects within `distInsert` pixels of that
vertex at a similar depth; `distInsert = 0` inserts every point. Each vertex accumulates the union
of the views that contributed to it (`vert_info_t::InsertViews`); a view's vote weight is 1 unless
the point cloud carries `pointWeights`, in which case the weight is that view's per-view confidence
— the `ReconstructMesh` app implements `--constant-weight` by releasing `pointcloud.pointWeights`
before calling into the library, so the library itself has no such switch.

**Scale.** `medianEdge`, the median length over all finite Delaunay edges, is measured once and
used two ways:
- **Canonical rescale** (`bCanonicalRescale`, `coord_rescale_t`): if `medianEdge` falls outside
  `[2^-10, 2^10]`, the whole triangulation is rescaled in place by a power of two so it lands near
  1. The ray-walk `orientation()` predicate tests an *unnormalized* determinant (grows as edge
  length cubed) against a fixed absolute epsilon, so it is only calibrated near unit scale; a
  power-of-two factor is exact in IEEE arithmetic (mantissa untouched) and CGAL's exact predicates
  are scale-invariant, so nothing is retriangulated. Only the extracted mesh vertices are mapped
  back to world space.
- **Sigma**: `sigma = medianEdge * kSigma` is the visibility fall-off / positional-uncertainty
  radius. With `bAdaptiveSigma` on, each vertex gets its own `sigma_v = kSigma × median incident
  finite-Delaunay-edge length`, clamped to `[0.25, 4] × sigma` — CGAL's
  `finite_incident_edges_threadsafe` is required here under OpenMP (the plain traversal races on
  shared TDS marker state against the ray-walk threads).

**Camera hard constraints (D_out).** For every image, the cell containing the camera is located;
if that cell is infinite (the common case — cameras usually sit outside the sparse cloud's convex
hull), every hull-adjacent infinite cell that is both camera-facing and inside the camera's
frustum (`fetchCellFacets`) gets `SourceCapacity = kInf`, forcing it to the free (source) side.

**Per-point votes (D_in).** For each (vertex, view) pair, a ray is walked from the camera to the
point; every facet it crosses accumulates `w = alpha_vis * (1 - exp(-dist² / (2·sigma_v²)))` on
its directed arc (the camera-side crossing). The walk continues past the point to
`point + sigma_v · direction`; facets crossed there accumulate the same weight on the *mirrored*
facet (`delaunay.mirror_facet`, the arc away from the camera), and the end cell's `SinkCapacity`
gets `+= alpha_vis` unconditionally, independent of the fall-off.

**Optional free-space support (WSS, `bUseFreeSpaceSupport`).** For each (vertex, view), the
maximum crossed weight `beta` towards the camera (window `kf · sigma_v`) and the mean of the
extreme crossed weights `gamma` past the point (window `kb · sigma_v`) are computed. When
`gamma/beta < kRel`, `beta - gamma > kAbs` and `gamma < kOutl`, the point is an interface point
and the end cell's `SinkCapacity` is multiplied by `beta - gamma`. A cell whose `t` is exactly 0
(never reached by a D_in vote) stays 0 — this no-op is intentional: enforcing on cells no
visibility vote ever reached would let the classifier plant surface priors in unobserved free
space.

### 2.2 Graph-cut (solver)

`libs/Math/TetraFlow.h:TetraFlow` is the sole max-flow solver (`typedef SEACAVE::TetraFlow
maxflow_t`) — an incremental breadth-first-search solver (Goldberg et al., ESA 2011) specialized
for the 4-regular Delaunay cell graph: one node per cell, `NodeID = uint32_t`, construction writes
directly into the solver's own node storage (`EdgeCapacity`/`SourceCapacity`/`SinkCapacity`/
`LinkEdge`), no separate per-cell weight array. Before linking, each internal facet's arc
capacities get a quality term added on both sides —
`q = (1 - min(computePlaneSphereAngle(f), computePlaneSphereAngle(mirror(f)))) * kQual`, the
cosine of the angle between the facet's plane and its two incident cells' circumscribed spheres, a
β-skeleton-style regularizer favoring round, well-formed cells. Sink capacities are clamped to
`maxCap = FLT_MAX * 1e-4` before `ComputeMaxFlow()`; `IsNodeOnSrcSide` gives the per-cell side,
after which the solver is released (it is the reconstruction's memory peak).

### 2.3 Surface extraction

A face is emitted for every internal facet whose two adjacent cells land on opposite sides of the
cut — that is the entire rule. **There is no edge-length or other geometric gate inside
`Scene::ReconstructMesh`**: every cut facet becomes a mesh face, including arbitrarily long ones
spanning occluded ("webbing") space. `Mesh::FixNonManifold` runs once afterwards to split any
vertex whose incident faces span more than one connected component.

### 2.4 The Clean stage and the long-edge gate (`MeshHalfMesh.cpp:Mesh::Clean`)

`Clean` converts to a `halfmesh::Mesh` once, runs every enabled stage on that single instance, and
converts back once. In order:

1. **`RemoveLongEdgeFacesLocal(maxEdgeScale, 3)`** (if `maxEdgeScale > 0`) — the webbing gate. Each vertex's *local edge scale* is the median length of the edges in its 3-ring on the extracted mesh (the ring count is a constant: the 1-ring median is inflated by the very long edges the gate should catch, and the choice is validated in §5); each face's scale is the max of its three vertices' scales (deliberately conservative at density transitions, so a uniformly sparse surface survives while a face bridging a sparse and a dense region does not). A face is removed when its longest edge exceeds `maxEdgeScale ×` its scale — purely geometric, on the extracted mesh, no image projection or per-vertex view lists.
2. **`RemoveLongEdgeFaces(spuriousFactor)` + `RemoveSpuriousComponents(spuriousFactor)`** (if `spuriousFactor > 0`) — a global (scene-wide p95 edge length) long-edge pass, then an isolated small-component removal pass.
3. **`RemoveSpikes(maxSpikeIterations)`**.
4. **`Simplify`** — target-magnitude decimation (`simplifyTarget`, a fraction in `(0,1)` or an absolute face count above 1), or, when a `vertexMaxError` array is supplied, per-vertex bounded decimation (an edge collapses only while its collapse point stays within the smaller quadric-distance bound of its two endpoints).
5. **`CloseHoles(maxHoleEdges)`**.
6. **`SmoothTaubin(smoothIterations)`** — λ/μ band-pass smoothing (λ=0.65, μ=−0.69 inside halfmesh); scale-free, and being a band-pass rather than a low-pass filter it removes high-frequency noise at ≈zero volume loss instead of shrinking the surface toward one-ring centroids.
7. **`RemeshIsotropic`** (if `edgeLength != 0`) — absolute or mean-relative target edge length, optionally graded by a per-vertex `vertexSizing` field.
8. **Finalize** (if `finalize`, default on): degenerate-face removal, unreferenced-vertex removal, non-manifold repair.

### 2.5 Regression fixtures (`apps/Tests/TestsMVS.cpp`, run by `Tests.exe 0`)

Two hand-derived synthetic scenes lock cut *topology* (vertex/face counts, which named vertices
survive) rather than internal per-cell flow values, which are not observable through the public
API; both are reconstructed with `bAdaptiveSigma = false`, `bCanonicalRescale = false`,
`bUseFreeSpaceSupport = false`, `kQual = 0`, `distInsert = 0` (single global-sigma, ungated), not
the shipped app defaults. A regression that relocates a vote onto the wrong cell or flips a
`mirror_facet` arc fails one of them.

- **`MeshBipyramidFixtureTest`** — 2 finite tetrahedra sharing one facet, 1 camera, 1 contributing
  point (equilateral triangle A/B/C at `z=0`, apexes D and E on the z-axis, camera above looking at
  E). Asserts exactly 1 face, 3 vertices: two of {A,B,C} plus E, never D — the D_in vote the
  camera's ray deposits behind the point (past E) must survive on the correct mirrored cell.
- **`MeshTetraInteriorPointFixtureTest`** — 4 tetrahedra around one interior point P inside a hull
  of 4 outer vertices, 2 cameras. Asserts an **empty** mesh: every cell reachable from P's votes
  ends up on the same (free) side by the solver's own tie-break on zero-capacity nodes, not by any
  error in `mirror_facet` — see the in-code comment on `ReportFixtureSolverTieBreak` for the full
  reachability argument; this is a solver-behavior fixture, not a `mirror_facet` correctness proof.

Other reconstruction-stage coverage in the same file: `EmptyROIMeshGuardTest` /
`TooFewPointsMeshGuardTest` (degenerate-input guards fail cleanly, §4), `UnitWeightsFallbackTest`
(empty vs. explicit all-1 `pointWeights` must agree within 1% of face count),
`PointWeightsArchiveRoundTripTest` (`pointWeights` round-trips through the interface archive),
`MeshCleanPerVertexTest` (per-vertex decimation bound routes correctly into halfmesh).

---

## 3. Parameters and defaults

`distInsert` and `bUseFreeSpaceSupport` are the two fields where the `ReconstructMeshParams`
constructor's own default differs from the `ReconstructMesh` app's CLI default; the CLI default is
what ships. Fields with no CLI flag are fixed at their struct default in the app.

| CLI option | Struct field | Default | Meaning |
|---|---|---|---|
| `-d, --min-point-distance` | `ReconstructMeshParams::distInsert` | 1.5 (CLI); 2.f (struct) | max pixel distance, in every view, for two points to merge into one Delaunay vertex; 0 inserts every point |
| `--integrate-only-roi` | `bUseOnlyROI` | false | triangulate only points inside the scene ROI |
| `--constant-weight` | *(app-level; releases `pointcloud.pointWeights`)* | true | vote weight per view: 1 if true, else the point's per-view confidence |
| `-f, --free-space-support` | `bUseFreeSpaceSupport` | false (CLI); true (struct) | enforce the WSS t-edge multiplier for weakly-observed interface points |
| `--thickness-factor` | `kSigma` | 1.0 | multiplier on the visibility fall-off / uncertainty radius sigma |
| `--quality-factor` | `kQual` | 1.0 | multiplier on the plane/circumsphere-angle quality term added to each facet's arc capacity |
| *(none)* | `kb` | 4.0 | WSS backward (past-point) search window, in units of sigma |
| *(none)* | `kf` | 3.0 | WSS forward (camera-side) search window, in units of sigma |
| *(none)* | `kRel` | 0.1 | WSS interface test: `gamma/beta` must be below this |
| *(none)* | `kAbs` | 1000.0 | WSS interface test: `beta - gamma` must exceed this |
| *(none)* | `kOutl` | 400.0 | WSS interface test: `gamma` must be below this |
| *(none)* | `kInf` | `kInfCapacity` = `INT_MAX/8` | hard source capacity for camera / D_out links |
| `--adaptive-sigma` | `bAdaptiveSigma` | true | per-vertex sigma from the vertex's median incident Delaunay edge length, clamped to `[0.25,4]×` global sigma |
| `--canonical-rescale` | `bCanonicalRescale` | true | rescale the triangulation by a power of two so the median edge lands near 1 |
| `--max-edge-scale` | `Mesh::CleanParams::maxEdgeScale` | 4.0 | drop faces whose longest edge exceeds this × the local (k-ring) edge scale; 0 disables |
| `--remove-spurious` | `spuriousFactor` | 20.0 | global p95-based long-edge + isolated-component removal factor; 0 disables |
| `--remove-spikes` | `removeSpikes` | true | remove spike faces |
| *(none)* | `maxSpikeIterations` | 100 | iteration cap for spike removal |
| `--decimate` / `--target-face-num` | `simplifyTarget` | 1.0 / 0 | fraction of faces to keep in `(0,1)`, or an absolute face count above 1; 1 disables |
| `--close-holes` | `maxHoleEdges` | 30 | close boundary loops spanned by at most this many edges; 0 disables |
| `--smooth` | `smoothIterations` | 10 | Taubin band-pass smoothing iterations; 0 disables |
| `--edge-length` | `edgeLength` | 0 | isotropic remesh target edge length (absolute, or negative × current mean); 0 disables |
| *(none)* | `remeshIterations` | 3 | isotropic remesh iteration count |
| *(none)* | `finalize` | true | degenerate-face/unreferenced-vertex removal + non-manifold repair, run after every other stage |

---

## 4. Invariants and constraints

- `orientation()` tests an **unnormalized determinant against a fixed absolute epsilon (1e-12)**; the determinant grows as edge-length cubed, so a scene whose median edge sits far from ~1 scene unit either collapses every ray-walk step to COPLANAR (too small) or loses robustness to float noise near true degeneracies (too large). This is why `--canonical-rescale` exists, and the rescale must precede camera-cell location, not just triangulation. `finite_incident_edges_threadsafe` is **required** under OpenMP for the adaptive-sigma fill — the plain (non-threadsafe) traversal writes shared TDS marker state and races against the ray-walk threads reading the same cells.
- The WSS `t==0` no-op (a cell no D_in vote ever reached keeps `t=0` even if a later WSS enforcement would otherwise multiply it) is **structurally protective**, not a bug: enforcing on such cells would let the classifier plant surface priors in deep, unobserved free space.
- **NaN bypasses the sink-capacity clamp**: `if (t > maxCap) t = maxCap;` — any comparison against NaN is false, so a NaN weight (traced source: `computePlaneSphereAngle`'s facet-normal normalization on a zero-area facet) reaches the solver unclamped; `+inf`, by contrast, *is* correctly clamped. `TetraFlow::NodeID` is `uint32_t`: the graph-cut is limited to 2^31-1 cells (~330M points).
- Camera D_out is realized as hard `kInf` source links on **every** frustum-visible hull-adjacent infinite cell, not just the camera's own located cell — on 360-degree or inward-facing captures this annihilates every D_in vote whose sigma-shifted end cell exits the convex hull, which is why OpenMVS meshes stay open at the hull boundary regardless of evidence. Intentional, unaddressed.
- `PointCloud::Point` storage is `float`, which quantizes UTM-magnitude scenes to ~6cm before triangulation runs; the canonical rescale cannot repair geometry storage has already destroyed — the open fix is load-time centering (§7), not a mesh-stage change.
- `Mesh::SamplePoints` must use the **fixed-seed overload** in any benchmark; the `random_device`-seeded default is noise-only and not reproducible. PatchMatch depth estimation is itself unseeded, so point count (and mesh F1) is noisy at the several-percent level between densifications of the same build and flags — any reconstruction or Clean A/B must run on one frozen `scene_dense.mvs`, never on two separate densifications.
- Mesh-stage cost scales with vertex count and memory is the binding limit: peak RSS runs roughly ~1.95 kB per Delaunay vertex (~313 B per cell at ~6.3-6.4 cells/vertex); wall time is super-linear in points on scenes with high point redundancy. Any change that pushes completeness needs a cost argument alongside it.

---

## 5. Validation of the shipped defaults

Every row is a controlled A/B: both arms run the same pipeline and differ only in the default
under test.

| Default | Comparison | Result |
|---|---|---|
| Solver = TetraFlow | vs the previous IBFS, on ball/room/Truck/Courthouse (1.5M/6.5M/8.0M/18.4M cells) | solve 0.70/6.9/6.3/17.0s vs IBFS 1.27+0.22/12.0+0.9/11.0+1.1/27.0+2.6s (solve+init); peak solver memory 99/423/530/1248MB vs 293/1236/1518/3509MB; raw meshes byte-identical (ASan/UBSan-checked against an exact reference solver; `apps/Tests/TestsMath.cpp` keeps a reference-checked unit test) |
| `--canonical-rescale 1` | scene forced to 1e-6 / 1e6 of the calibrated band | at 1e-6, all 72542/72542 camera rays are dropped (every walk step reads COPLANAR), producing an empty mesh; at 1e6, 7 vertices are lost to float-noise near-degeneracies — the rescale is a correctness fix, not only a speed one |
| `--adaptive-sigma 1` | vs a single global sigma, all four T&T scenes | raw graph-cut surface ΔF1: Ignatius +0.039, Truck +0.015, Barn +0.012, Meetingroom +0.008 (positive on every scene); also the fastest arm tested (Ignatius graph-cut solve 32.4s vs 35-50s for every alternative arm) |
| `--thickness-factor 1` (library `kSigma=1.f`) | vs the old library default `kSigma=2` | ΔF1 in favor of 1: Ignatius +0.146, Truck +0.043 |
| `--free-space-support 0` | vs enabling it at default WSS constants | ΔF1 cost of enabling: Ignatius −0.048, Truck −0.052 |
| `--max-edge-scale 4`, 3-ring (`RemoveLongEdgeFacesLocal`) | see the gate table below | mean F1 over Herz-Jesu-P8 / Ignatius / Truck 0.6222 vs 0.5905 ungated |

Long-edge gate, all arms cleaned from the same ungated graph-cut surface per scene (Herz-Jesu-P8:
EPFL mesh-to-mesh evaluator, tau 0.01, 500k samples, completeness restricted to camera-visible GT;
Truck, Ignatius: Tanks and Temples toolbox at the official tau). "ring k" = the halfmesh k-ring median
statistic at factor 4; "reconstruction-side" arms are the gates that used to live inside
`Scene::ReconstructMesh` (full reconstruction with that binary, so not the same raw surface).

| arm | Herz-Jesu-P8 F1 (P / R) | Ignatius F1 (P / R) | Truck F1 (P / R) |
|---|---|---|---|
| ungated | 0.5141 (0.5625 / 0.4733) | 0.7376 (0.7546 / 0.7212) | 0.5198 (0.4211 / 0.6788) |
| ring 1 | 0.5143 (0.5635 / 0.4730) | 0.7375 (0.7545 / 0.7212) | 0.5201 (0.4215 / 0.6789) |
| ring 2 | 0.5228 (0.5904 / 0.4691) | 0.7386 (0.7572 / 0.7210) | 0.5435 (0.4522 / 0.6808) |
| **ring 3 (shipped)** | 0.5251 (0.6113 / 0.4602) | 0.7415 (0.7629 / 0.7211) | 0.6000 (0.5343 / 0.6841) |
| ring 3, factor 3 | 0.5249 (0.6197 / 0.4552) | 0.7425 (0.7650 / 0.7212) | 0.6129 (0.5551 / 0.6840) |
| ring 3, factor 6 | 0.5232 (0.5960 / 0.4663) | 0.7393 (0.7585 / 0.7211) | 0.5810 (0.5055 / 0.6830) |
| reconstruction-side local + common-view gate (rejected, §6) | 0.5151 (0.5654 / 0.4731) | 0.7424 (0.7651 / 0.7211) | 0.6420 (0.6074 / 0.6808) |
| reconstruction-side global-median gate (rejected, §6) | 0.4739 (0.6160 / 0.3850) | 0.7427 (0.7654 / 0.7212) | 0.6606 (0.6456 / 0.6762) |

The shipped gate is the best mean F1 of the mesh-side arms (0.6222; the global-median gate's mean is 0.6257 but only by trading Herz-Jesu recall 0.4733 -> 0.3850 for Truck precision). The ring count is a constant because the ordering is the same on all three scenes: ring 1 is a
no-op (a webbing face's own edges dominate its vertices' 1-ring medians), ring 3 is the best mean
F1 and costs under 0.5 s of Clean wall on a 5M-face mesh. Statistics that are more aggressive than
the k-ring median (minimum or lower-quartile edge length in the k-ring, or the minimum of the
neighbouring vertices' medians) reach Truck F1 0.65-0.68 but cut Herz-Jesu recall to 0.25-0.44:
Herz-Jesu's background is a real, sparsely sampled surface, and without visibility information no
edge-length statistic separates it from Truck's webbing better than the k-ring median does.

---

## 6. Rejected alternatives

- **Global-median cut-facet gate inside reconstruction**: one scene-wide median edge length removes
  directly-observed sparse background along with true webbing (Herz-Jesu recall collapse; most
  rejected facets had a camera common to all three vertices).
- **Per-vertex Delaunay-star scale + common-view ("hybrid") gate inside reconstruction**: needs
  per-vertex view lists and extra state carried through the Delaunay stage; the mesh-side
  post-process (`RemoveLongEdgeFacesLocal`, §2.4) reproduces the useful part with no image
  projection or ray casting.
- **Visibility-mass gate** (`--min-surface-evidence`): ~60% of true-surface facets also carry
  exactly zero accumulated mass (a ray crosses only 1-2 facets of a vertex's ~20-facet umbrella); no
  mass threshold separates webbing from true surface.
- **Grazing-incidence down-weighting**: harmful once measured on the corrected smoother; the
  apparent old gain was an artifact of a since-fixed smoothing bug.
- **WSS enforcement semantics variants** (`add`, `max` vs the shipped `product`): `add` and `max`
  both break the protective `t==0` no-op by firing on cells no vote ever reached; `paper` is
  ≡ `product` on dense clouds, so it buys nothing.
- **Footprint-based sigma** (per-pixel range/focal as an alternative sigma_v source): physically the
  same signal as the confidence-based entries below (within noise on every scene tested) and
  slower.
- **Confidence sigma shrink** (`sigma_v *= 1 - s·conf_v`): does not stack with adaptive sigma (same
  underlying information); at or below the weight-1 baseline on recalibrated clouds.
- **Weighted votes as an unconditional default**: every data-term capacity shrinks by the mean point
  confidence while the quality term and camera hard constraints keep unit-vote calibration,
  collapsing the cut on un-recalibrated clouds. Stays an explicit opt-in (`--constant-weight 0`).
- **Quality co-scale** (`kQual *= mean consumed confidence`, meant to rehabilitate the item above):
  within noise of the bare weighted-vote result; not worth a second code path.
- **kAbs/kOutl proportional rescale**: the product-semantics WSS enforcement saturates the t-edge at
  nearly every setting tested regardless of the constants, so a rescale only changes *which* cells
  get nuked, never *how hard*.
- **EIBFS solver** (vs the then-bundled IBFS): speed-neutral on real graphs and crashes at scale;
  carries a research-only license.
- **Carve-only rays from unfused pixels** (`--carve-rays-file`): below the default-flip acceptance
  gate, and by construction cannot reach true webbing (a ray that could reach it would have produced
  a fused point there).
- **No-decimation control** (`--min-point-distance 0`): worse F1 at several times the graph-cut
  cost; decimation is not a source of fidelity loss.

---

## 7. Open items

- **Load-time centering** for float-quantized large-coordinate point clouds (~6cm quantization at
  UTM magnitude, §4) — an import-side fix touching every pipeline that produces a `PointCloud`
  (Interface importers, `CreateStructure`), not a mesh-stage change. Not started.
- **Depth-maps as direct mesh input**, bypassing or supplementing the fused cloud, so the mesh stage
  can recover the geometrically-consistent depth fusion discards for failing to cluster into
  `nMinPixelsFuse` agreeing estimates. Not started.
- **Gate-validation numbers** for the current mesh-side `RemoveLongEdgeFacesLocal` gate (§5's
  placeholder row) — the T&T-scene numbers validating the previous in-reconstruction gates no
  longer apply now that the gate has moved to `Mesh::Clean`.
- **Acceptance gates for future work on this energy**: mean paired mesh-F1 >= +0.003 beyond the
  0.0006 noise floor; no scene regressing more than 0.003 F1; >=5% median improvement for
  exact-result speed changes. Judge every reconstruction-stage change on the raw graph-cut surface,
  never on the cleaned mesh.
- Standing guardrails: confidence enters the visibility data term only — never `kQual`/circumsphere
  quality, camera hard constraints, or a second generic per-cell unary; no generic k-NN/smoothing
  prefilters by default (they erase thin structure); face count is not a completeness metric — score
  by F1 on ground truth.
