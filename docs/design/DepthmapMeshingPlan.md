# Depth-maps as Direct Mesh Input — Implementation Plan

Target: a new, opt-in input mode for `Scene::ReconstructMesh` (`libs/MVS/SceneReconstruct.cpp:1007`)
that consumes the per-view `.dmap` files directly instead of the fused point-cloud, skipping
`DepthMapsData::DenseFuseDepthMaps` (`libs/MVS/SceneDensify.cpp:2634`) entirely.

This is the full version of the idea the executed mesh improvement plan (since consolidated into
`docs/design/DelaunayMeshReconstruction.md`; the plan file itself lives in git history) staged
as **5.2 — direct dmap-sourced meshing**, parked after the carve-rays slice. It is written against the
measurements that phase produced, so most of the "is the dropped evidence real?" question is
already answered and does not need re-measuring — see § 1.

Conventions: numbered slices, one reversible commit each, the acceptance gates recorded in
`docs/design/DelaunayMeshReconstruction.md` § 8, per-slice verdicts appended to that same
record.

---

## 0. Verified ground truth

Facts read out of the code and out of the existing benchmark record. Do not re-derive.

### 0.1 The current input contract of `Scene::ReconstructMesh`

| Fact | Where |
|---|---|
| Input is `scene.pointcloud`: `points`, `pointViews` (per-point view list), optional `pointWeights` (per-view [0,1] confidence) | `SceneReconstruct.cpp:1060-1091` |
| Points are spatial-sorted (`CGAL::spatial_sort`) then inserted **serially** into `delaunay_t` with a hint; the `distInsert` (`--min-point-distance`, CLI default 1.5 px) test locates the nearest existing vertex and inserts only if the new point projects further than `distInsert` **or** at a dissimilar depth **in at least one of its views** | `:1078-1151` |
| Per-vertex visibility is accumulated by `vert_info_t::InsertViews`, which **sums** the weight of repeated views: a vertex merged from N points sharing a view carries weight N from that view (weight 1 each under `bConstantVotes`) | `:243-270` |
| `pointcloud.Release()` right after insertion — nothing downstream can look at the cloud again | `:1203` |
| The global σ base is the **median finite Delaunay edge**, measured once the triangulation is complete | `:1232-1239` |
| Camera cells + their `kInf` s-links are located after the optional canonical rescale | `:1243-1268` |
| Ray walking is a **separate** pass over `vertexHandles`, two walks per (vertex, view) pair: camera→point (`intersect()`, `:698`) and endpoint→point (σ-shifted `D_in`, `:1529-1565`) | `:1456-1574` |
| WSS (`DELAUNAY_WEAKSURF`) is a third pass reading the free-space field the weighting loop deposited | `:1581-1730` |
| **Insertion and ray-walking are already decoupled** — the carve replay (`:1739-1809`) walks rays for points that are not vertices at all, using `intersectFace()` (`:827`, `:854`) | `:1739-1809` |
| Carve rays arrive as an `UnfusedPixel` sidecar (20-byte records, `"MVSU"` header) loaded by `loadCarveRays()` before anything is built | `:894-929`, `DepthMap.h:186-200` |
| Parameters live in the trailing `ReconstructMeshParams` struct; `carveRaysFile` is the one positional string added on top | `Scene.h:153-221` |
| App call site passes every knob from one place; `--constant-weight 1` (default) releases `pointWeights` before the call | `apps/ReconstructMesh/ReconstructMesh.cpp:509-525` |

### 0.2 The `.dmap` side

| Fact | Where |
|---|---|
| The codec lives **only** in `MVS/Interface.h`: `ExportDepthDataRaw`/`ImportDepthDataRaw` | `Interface.h:1168,1267,1284,1376` |
| A dmap carries: image + depth resolution, `dMin/dMax`, reference image path, **global view IDs** (reference first, then neighbors), and the reference `K`/`R`/`C` **in double**, then the maps | `Interface.h:866-904` |
| Storage is 11 bytes/pixel: depth as `half` after an exact power-of-two rescale (relative error 4.9e-4), normal as two `int16` octahedral, confidence as `uint8` over `[0, confScale]`, optional views-map (up to 4 supporting neighbor indices per pixel) | `Interface.h:856-888` |
| `CONF_ADJUSTED` flag marks a dmap whose confMap is the recalibrated (#1292, fusion-survival) posterior | `Interface.h:875` |
| File naming: `ComposeDepthFilePath(image.ID, "dmap")` → `depth%04u.dmap` under the working folder — the same convention `DensifyPointCloud.cpp:359` reads back | `DepthMap.h:72` |
| `DepthData::Load/Save` wrap the codec; `DMapCache` (`DMapCache.h:54`) is the memory-bounded LRU that fusion already streams dmaps through | `DepthMap.h:340-345` |
| A per-pixel depth+confidence → point-cloud conversion pattern already exists (single-map import) | `Scene.cpp:406-428` |

### 0.3 What fusion drops (Phase 5.0, current defaults, R1/V12 — **do not re-measure**)

| scene (imgs) | valid depths | admitted | dropped | low-conf | min-pixels | violation | dropped @ conf ≥ 0.1 |
|---|---|---|---|---|---|---|---|
| Ignatius (263) | 132.2 M | 48.0 % | 52.0 % | 26.4 % | 25.4 % | 0.20 % | 33.9 M |
| Meetingroom (371) | 175.1 M | 30.1 % | **69.9 %** | 34.6 % | 35.0 % | 0.25 % | **61.7 M** |
| Truck (251) | 126.5 M | 56.0 % | 44.0 % | 23.6 % | 20.3 % | 0.15 % | 25.8 M |

The low-confidence channel is exactly the `[0,0.1)` histogram bin in every scene (the
`1 − fNCCThresholdKeep = 0.1` cut); everything dropped above it is **min-pixels** — geometrically
consistent depth that simply failed to cluster into ≥ 5 pixels. Every scene throws away 9–10 M
pixels at confidence ≥ 0.7.

### 0.4 Fusion wall time (read out of the same runs' logs, cached dmaps, R1/V12)

| scene | fusion stage | whole `DensifyPointCloud` run (dmaps already on disk) |
|---|---|---|
| Truck | 47.4 s / 50.1 s | 1m14s / 1m32s |
| Ignatius | 50.8 s / 49.1 s | 1m20s / 1m31s |
| Meetingroom | 65.8 s / 62.8 s | 1m48s / 2m00s |

Source: `<scene>/runFusionStats/DensifyPointCloud-2608191*.log`, lines
`Depth-maps dense fused and filtered:` and `Densifying point-cloud completed:`.

### 0.5 Mesh-stage unit costs (bench `out_mesh/results.csv`, `@runFusionStats` clouds)

| scene | pts inserted | DT verts | cells | tri | weight | graph-cut | recon | wall | peak RSS |
|---|---|---|---|---|---|---|---|---|---|
| Truck | 9.72 M | 5.22 M | 32.8 M | 34.8 s | 47.3 s | 56.9 s | 159 s | 238 s | 10.3 GB |
| Ignatius | 9.84 M | 4.75 M | 29.7 M | 34.8 s | 59.6 s | 70.9 s | 184 s | 243 s | 9.3 GB |
| Meetingroom | 10.00 M | 6.50 M | 41.5 M | 76.1 s | 169.2 s | 143.3 s | 440 s | 598 s | 12.8 GB |

Derived unit costs, consistent across all three scenes and the load-bearing input to § 3:

- **Peak RSS ≈ 1.95 kB per Delaunay vertex** (10256/5.22, 9262/4.75, 12768/6.50 — 1.966 / 1.949 /
  1.965), i.e. ≈ 313 B per cell at 6.3–6.4 cells/vertex. Includes the loaded scene, so it is a
  small over-estimate of the triangulation alone and the right number for a memory budget.
- **Insertion ≈ 3.5–7.6 µs per input point** (`tri_s`/`pts_inserted`: 3.59 / 3.52 / 7.61 µs).
- **`distInsert=1.5` keeps 48–65 %** of the fused points as vertices.
- **Carve ray ≈ 1.6–3.3 µs** (Phase 5.1 A/B deltas: Truck +12.2 s / 7.43 M rays, Ignatius
  +8.0 s / 4.75 M, Meetingroom +23.9 s / 7.31 M).

### 0.6 The carve-only-rays verdict (Phase 5.1)

Best-scene +0.0033 mesh F1 (Truck), Meetingroom +0.0022, Ignatius −0.0014; noise floor 0.0006,
§ 8 gate +0.003. **Opt-in, not default.** The sign pattern tracks the fusion-drop mass, so the
evidence is real, but the effect is sub-gate under the current energy — and the same table shows
`fss` losing 0.03–0.07 F1 on these denser clouds, i.e. the energy's absolute-scale constants are
miscalibrated for dense inputs. Direct dmap meshing inherits both facts.

---

## 1. Goal and non-goals

**Goal.** Let `ReconstructMesh` build the Delaunay visibility graph from the per-view depth maps,
so that (a) the fusion stage can be skipped end-to-end and (b) the mesh energy sees the evidence
fusion currently destroys — 44–70 % of all valid depth pixels lose *both* their point and their
visibility ray today (§ 0.3).

**Non-goals.**

1. **Not a replacement for fusion.** Users who want `scene_dense.mvs` — a point-cloud product,
   colors, normals, downstream tools, `--export-*` paths — keep the current pipeline unchanged.
   The new mode is one more way to *feed the mesh stage*, selected explicitly.
2. **Not a default flip in this plan.** The default path must remain byte-identical; a default
   decision, if it ever comes, is a separate maintainer call with its own gate (the same
   convention as Phase 2, Phase 4.3 and Phase 3.4).
3. **Not a volumetric method.** TSDF-style depth fusion (Curless & Levoy VRIP 1996 and its
   learned successors, e.g. RoutedFusion) integrates depth into a regular grid and has no
   visibility-ray energy at all — a different family, out of scope, named here only to delimit it.
4. **Not a new energy.** The visibility/WSS/quality energy stays exactly as shipped (WSS
   enforcement-semantics switches and grazing-incidence down-weighting no longer exist in the
   code at all — both were removed as universally harmful, see `DelaunayMeshReconstruction.md`);
   this plan changes *where the vertices and rays come from*, nothing about what they cost.
5. **Not a re-litigation of Phase 5.0.** Its drop accounting is the input, not a slice.

---

## 2. Why the speed argument must be stated honestly up front

The motivation lists speed first. The measurements in § 0.4–0.5 say the speed case is real but
**small, and easy to spend twice over**:

- Skipping fusion saves **47–66 s** per scene, plus writing and re-reading a multi-GB
  `scene_dense.mvs` (part of the 60–160 s gap between `recon_s` and `recon_wall_s`).
- Walking a ray for **every** valid depth pixel costs, at the measured 1.6–3.3 µs/ray:
  Truck 126.5 M × 1.64 µs ≈ **208 s**, Ignatius ≈ 222 s, Meetingroom 176.1 M × 3.27 µs ≈ **576 s**.
  A cross-check from the other direction agrees on the order: today's weighting loop already
  walks ~2 segments per (vertex, view) pair over ~40–70 M pairs in 47–169 s, so all-pixel
  evidence is ≈ 2–4× that stage, not 10×.
- Inserting **every** pixel costs, at 3.5–7.6 µs/point, 454 s (Truck) to 1340 s (Meetingroom) of
  triangulation alone, and at 1.95 kB/vertex would need **100–220 GB** of RAM (§ 3, alternative A).

So: **the headline speed claim is only defensible in a budget-equalized configuration** — one where
the number of Delaunay vertices, and ideally the number of walked rays, is held near today's. In
that configuration the net saving is the fusion stage plus the cloud round-trip, i.e. roughly
60–120 s per scene against a 160–440 s mesh stage: **a 15–30 % end-to-end saving on the
fusion+mesh half of the pipeline**, and a single-digit percentage of a pipeline that also runs
depth estimation. Anything above that budget buys quality with time, and must be sold as such.

This is the plan's first design constraint, not a caveat added at the end.

---

## 3. Design alternatives

All five are evaluated against the same three axes: **speed** (vs today's densify-fusion + mesh),
**memory** (peak RSS, using 1.95 kB/DT-vertex), **quality risk**, **complexity**.

### A. Naive — every depth pixel is a vertex, with its single-view ray

- **Vertices**: 126–176 M input points. Even assuming `distInsert` keeps only the same 48–65 %
  it keeps today, 61–114 M vertices → **119–222 GB peak RSS**. On a 64 GB box this is not a
  tuning problem, it is a category error.
- **Speed**: 454–1340 s of insertion + 208–576 s of ray walking, against a 47–66 s saving.
- **Quality**: every noise pixel becomes a vertex; the graph absorbs outliers by design, but the
  min-pixels channel that fusion drops (20–35 % of all depths) is exactly the population most
  likely to be isolated noise.
- **Verdict: rejected as an implementation target.** It stays in the plan only as the asymptote
  the stride knob interpolates toward, and as the thing a "stride 1" run would do if someone
  asks for it on a small scene.

### B. Decimated vertices + dense rays *(recommended core)*

Insert a **stride-decimated, confidence-gated** subset of each view's pixels as vertices; walk
rays for **every** (or a much larger stride of) valid confident pixel, whether or not it became a
vertex. This is the exact generalization of what carve-only rays proved works: the code already
separates "who becomes a vertex" (`:1088-1183`) from "what evidence is accumulated"
(`:1456-1574` + `:1739-1809`).

- **Vertices**: a per-axis stride `s` samples 1/s² of the raster. To land at today's ≈ 10 M input
  points on these scenes, `s = 4` (126.5 M/16 = 7.9 M Truck, 8.3 M Ignatius, 11.0 M Meetingroom).
  Peak RSS then matches today's 9–13 GB by construction.
- **Rays**: independent stride `r`. `r = 1` (all pixels) costs +208…576 s; `r = 2` costs
  +52…144 s — i.e. **about what the skipped fusion stage cost**, while carrying 4× the ray mass
  Phase 5.1 tested and with no admission filter in front of it.
- **Quality**: the strongest form of the Phase 5.1 result. Rays from *all* dropped pixels, not
  only those that reached a cluster and then failed `nMinViewsFuse`/min-pixels; and the rays land
  **before** the WSS classifier reads the free-space field, fixing the ordering asymmetry
  5.1 explicitly documented as a deliberate one-line deferral.
- **Complexity**: moderate. One new reader, one new sampling pass, one reuse of the carve walk.
- **Risk**: the `InsertViews` weight-summing hazard (§ 6, risk R4) and the loss of any multi-view
  admission test.

### C. Lightweight on-the-fly fusion — cheap dedup, keep every ray

Run a consistency pass shaped like fusion but with none of its admission thresholds, purely to
decide which pixels collapse onto which vertex.

The key realization: **`distInsert` already is that pass.** Its test projects the candidate into
its own views and compares against the nearest existing vertex — for a per-view raster this
merges neighbouring pixels of the same view *and* pixels of other views that already produced a
vertex within 1.5 px and a similar depth. Multi-view consolidation therefore comes for free, which
is precisely what the original Phase 5.2 sketch predicted ("let `distInsert` merging do the
multi-view consolidation").

What C would add on top of B is giving the *non-sampled* pixels a proper surface term instead of
a carve-only ray: locate the nearest vertex, and if the pixel agrees with it, append its view to
that vertex (so it earns a `D_in` end-cell term too). Cost: one `locate` per pixel (≈ 1–3 µs, i.e.
another 130–530 s at r=1) plus atomic contention on shared vertex view-lists.

- **Verdict: a later slice, not the entry point.** It doubles the per-pixel cost of B for an
  effect (upgrading carve rays to full votes) whose sign is unknown, and it re-introduces exactly
  the vote-mass-∝-density coupling § 2 note 1 of the improvement plan warns about.

### D. Hierarchical / streaming insertion

Process views one at a time, inserting into the DT incrementally (CGAL already supports it — the
current loop *is* incremental) and releasing each dmap after use, reusing `DMapCache`
(`DMapCache.h:54`) or the same on-demand pattern densification uses.

- This is not an alternative to B; it is **how B is implemented**. The only real choice inside it
  is whether to keep a global `CGAL::spatial_sort` (better locality, deterministic, needs the
  sampled records buffered) or to insert per view as read (streaming, but insertion order — and
  therefore the triangulation — becomes a function of view order).
- Buffering the *sampled* records is cheap: 8–12 M × 20 B = **160–240 MB**, against a 9–13 GB
  peak. So keep the global spatial sort and the determinism; stream only the ray pass, which
  needs no buffer at all.
- **Verdict: adopted as the implementation shape of B.**

### E. Hybrid — fused cloud for vertices + all rejected-pixel rays

This is Phase 5.1 as shipped (`--carve-rays-file`), with its sidecar generated by fusion.

- **Why the full mode should beat it**: 5.1's sidecar only contains pixels that *joined a cluster
  and were then dropped*, gated at conf ≥ 0.5 and strided down to ≤ 8 M records; the
  low-confidence fate never reaches it at all, and neither does any pixel whose seed was consumed
  as `already-fused` (≈ 60 % of probes). Direct mode sees the raster. It also removes the
  weight asymmetry 5.1 recorded (fused votes weight 1 vs carve rays ≤ 1 confidence) and the
  WSS ordering deferral, and it can put vertices where fusion put none at all — the min-pixels
  channel is 20–35 % of every scene's depths and is *geometrically consistent*, so it should add
  surface, not just carve it.
- **When it will not beat it**: if the mesh energy's failure on these clouds is calibration
  (`fss` −0.03…−0.07 F1) rather than evidence starvation, more evidence at the same calibration
  moves nothing — which is precisely how 5.1 came out at +0.0033/−0.0014. E is the control arm
  the direct mode must beat, and it is cheap to run because it already exists.

### Summary

| | vertices | added time vs today | peak RSS | quality risk | complexity |
|---|---|---|---|---|---|
| A naive | 61–114 M | +660…1900 s | 119–222 GB | outliers everywhere | low code, impossible cost |
| **B decimated + dense rays** | **8–11 M in → 4–7 M DT** | **−60…−120 s at r=2; +150…+500 s at r=1** | **9–13 GB (matched)** | no admission test | moderate |
| C on-the-fly fusion | as B | B +130…530 s | as B | density-coupled votes | high |
| D streaming | — | implementation of B | bounded | — | moderate |
| E hybrid (shipped) | fused cloud | +8…24 s | +0 | none new | zero (exists) |

---

## 4. Recommended staged approach

**Start at B, implemented as D, gated against E, with A reachable only as a stride setting.**

Concretely, the direct mode decomposes into three passes that reuse the existing machinery almost
entirely:

- **Pass A — vertices.** Stream every dmap once; for each view emit the stride-`s`,
  confidence-gated pixels as `(X, viewIdx, conf)` records into a `cList`; `CGAL::spatial_sort` the
  whole batch; run the *existing* insertion loop over it (`:1088-1183`), with `InsertViews`
  fed one view per record. Multi-view consolidation is `distInsert`'s job (alternative C).
- **Pass B — the vertices' own rays.** Unchanged: the existing weighting loop (`:1456-1574`) walks
  camera→vertex and the σ-shifted end segment for every (vertex, view) pair Pass A produced.
- **Pass C — complement rays.** Stream every dmap a second time; for each valid confident pixel
  that Pass A did *not* sample, walk the camera→pixel ray with the carve replay code
  (`:1739-1809`) — same `intersectFace` stepper, same soft weight, no vertex, no unary term.
  Deterministic complement: same stride arithmetic, so no bookkeeping is needed.

Everything downstream (σ arms, WSS, graph build, solve, extraction) is untouched. The only genuinely
new code is the dmap reader/sampler and the parameter plumbing; the two walk passes are existing
loops pointed at a different source.

Three consequences worth stating:

1. **The footprint σ of Phase 5.3 becomes exact and free here.** `footprint = range/focal` is a
   per-pixel quantity in the direct mode, not a per-vertex average reconstructed at mesh time —
   and Phase 5.3 found the physical field owns the object-scene gains (Ignatius +0.0087). Same for
   `--sigma-conf-shrink`: the dmap confidence is right there, `CONF_ADJUSTED`-flagged.
2. **Positions get *better*, not worse.** The dmap carries `K`/`R`/`C` in double
   (`Interface.h:901-903`); back-projection can be done in `REAL` and only the final vertex is
   float. Today's path round-trips through `PointCloud::Point` float storage, which the Phase 3.4
   audit flagged as an unrepairable-at-mesh-time quantization at large coordinates. The half-coded
   depth costs 4.9e-4 relative — well below the float-storage floor at any scene magnitude.
3. **WSS finally reads a complete free-space field**, since Pass C runs before the classifier
   rather than after it (the asymmetry 5.1 recorded).

Gate before attempting A: only if B at r=1 clears the § 8 gate *and* someone shows a scene where
vertex density is the binding constraint does the stride go below 2.

---

## 5. CLI / API surface

Project convention: no parameter bloat on the already-long `Scene::ReconstructMesh` signature —
everything new goes in the trailing `ReconstructMeshParams`. `carveRaysFile` is the one positional
string and it is **not** a precedent to extend.

### `libs/MVS/Scene.h`, inside `struct ReconstructMeshParams` (`:153-214`)

```cpp
// Build the triangulation from the per-view depth-maps instead of the fused point-cloud:
// each valid, confident depth pixel back-projects to a candidate vertex, distInsert does the
// multi-view consolidation fusion would have done, and the pixels not sampled as vertices
// still contribute their camera ray. Empty disables the mode and leaves the point-cloud path
// byte-identical; a non-empty folder is where depth%04u.dmap files are looked up
// (see ComposeDepthFilePath)
String depthMapsFolder;
// per-axis raster stride of the pixels promoted to Delaunay vertices; 0 picks the smallest
// power-of-two stride whose total sampled pixel count fits depthMapsVertexBudget, so a scene
// of any size lands on a bounded triangulation (peak RSS is ~2 kB per vertex)
unsigned depthMapsVertexStride = 0;
// target number of candidate vertices the automatic stride aims at (ignored when the stride
// is given explicitly); the shipped fused clouds sit at 9-10M on T&T scenes
uint32_t depthMapsVertexBudget = 10000000;
// per-axis raster stride of the pixels contributing a carve-only ray; 1 walks every valid
// confident pixel, which is the full-evidence configuration and the expensive one
unsigned depthMapsRayStride = 1;
// minimum stored confidence a pixel needs to be used at all, as a vertex or as a ray;
// the default is fusion's own low-confidence cut (1 - OPTDENSE::fNCCThresholdKeep)
float depthMapsMinConfidence = 0.1f;
// deposit at most one unit of vote per (vertex, view) pair instead of summing one per merged
// pixel: in the direct mode a vertex can absorb tens of same-view pixels, and InsertViews
// sums them, so the fused path's "one vote per view per point" calibration needs this
bool depthMapsCapViewVotes = true;
```

### `apps/ReconstructMesh/ReconstructMesh.cpp`

Five options, all in the visible `config_main` group except the folder override:

| option | default | meaning |
|---|---|---|
| `--use-depth-maps` | `0` | build from `depth%04u.dmap` instead of the point-cloud |
| `--depth-maps-folder` | *(working folder)* | where to look for them (hidden) |
| `--depth-maps-vertex-stride` | `0` (auto) | per-axis stride for vertices |
| `--depth-maps-ray-stride` | `1` | per-axis stride for carve rays |
| `--depth-maps-min-confidence` | `0.1` | confidence gate |

Wiring rules:

- `--use-depth-maps 1` sets `params.depthMapsFolder` to `--depth-maps-folder` or `WORKING_FOLDER`;
  `0` leaves it empty. `MAKE_PATH_SAFE` handling copies the `carveRaysFile` precedent
  (`ReconstructMesh.cpp:519-521`) — an unset path must stay an empty string to mean disabled.
- The app must **not** require `-i scene_dense.mvs`: `-i scene.mvs` (poses + images, no cloud) is
  the point of the mode. The existing `ASSERT(!pointcloud.IsEmpty())` at `:1015` becomes
  `ASSERT(!pointcloud.IsEmpty() || bDepthMaps)`.
- If a cloud *is* loaded and the flag is on, **ignore the cloud** and log it. Single-variable A/B
  beats a silent union.
- `--use-depth-maps 1 --carve-rays-file <f>` is rejected by the app (the sidecar is a strict
  subset of what the mode already reads).

### Off-state contract

The mode is entered from exactly one branch, `const bool bDepthMaps(!params.depthMapsFolder.empty())`,
evaluated once at the top of `ReconstructMesh`. When false: no allocation, no file probe, no
changed arithmetic, no new call — the same anchors-md5 discipline every slice since Phase 3.1 has
used (4–6 reference meshes, raw and cleaned, plus bit-identical max-flow).

---

## 6. Slices

One reversible commit each. Every experimental slice appends its before/after table and an
accepted/rejected verdict to `docs/design/DelaunayMeshReconstruction.md`.

### Slice 0 — measurement only, no product code *(0.5 day)*

Answers the questions § 3 leaves open, cheaply, before any design is committed.

1. **End-to-end wall breakdown per scene.** Time, on the `runFusionStats` folders (dmaps already
   cached), arm A = `DensifyPointCloud` (fuse + save) + `ReconstructMesh` (load + recon), and
   record the components. Fusion stage times are already in § 0.4; what is missing is the
   `scene_dense.mvs` write and read cost, which is the other half of the speed claim.
   *Do not re-measure the drop accounting — cite § 0.3.*
2. **dmap inventory**: for each bench scene, total on-disk bytes, per-file resolution, whether the
   shipped dmaps carry `HAS_VIEWS` (a free per-pixel multi-view support count of up to 4 —
   `Interface.h:851` — which would give an admission ladder for nothing) and whether `CONF_ADJUSTED`
   is set. This is a `python`+`struct` header read, no build needed.
3. **Projected budget table**: for strides 1/2/4/8, the sampled pixel count, the projected DT
   vertex count (using the measured 48–65 % `distInsert` survival), the projected peak RSS at
   1.95 kB/vertex, and the projected ray cost at 1.6–3.3 µs.

**Kill criterion**: if (2) shows the confidence maps are not `CONF_ADJUSTED` on the bench scenes,
the whole confidence-gating design rests on a different signal than #1292 and slice 1 must be
re-scoped before it starts.

### Slice 1 — dmap source + vertex pass (alternative B, vertices only) *(2–3 days)*

`--use-depth-maps 1` with `--depth-maps-ray-stride 0` (rays disabled): the triangulation is built
from stride-sampled depth pixels, the existing weighting loop walks their rays, everything else is
untouched. This is the cheapest complete test of the **input-model change** — no new energy, no new
evidence, just "fusion replaced by `distInsert` consolidation".

Sketch (SEACAVE containers, per project convention — `libs/Common/List.h`):

```cpp
// one sampled depth pixel: the candidate vertex, the view that saw it, and its confidence
struct dmap_sample_t {
    Point3f X;
    IIndex  idxView;
    float   conf;
};
typedef CLISTDEF0IDX(dmap_sample_t,uint64_t) DMapSampleArr;
```

- Reader: `ImportDepthDataRaw` per image (`HAS_DEPTH|HAS_CONF`), camera from the dmap's own
  double `K`/`R`/`C`, resolution taken from the header (never assume it equals the image).
- Sampler: `for (r = 0; r < rows; r += s) for (c = 0; c < cols; c += s)`, skipping `depth <= 0` and
  `conf < depthMapsMinConfidence`; `bUseOnlyROI` applies per sample exactly as it does per point.
- Insertion: the *existing* loop, with `InsertViews` given a one-view record and the
  `depthMapsCapViewVotes` clamp.
- Instrumentation (aggregate only, `DEBUG_EXTRA`): dmaps read, pixels seen / gated / sampled,
  candidates → DT vertices, stride chosen, read wall time.

**Gates**: off-state byte-identical on all anchors (agent + independent reviewer, the standing
convention); `ctest` green; on-state produces a valid mesh on SceauxCastle and on one T&T scene;
DT vertex count and peak RSS land inside the slice-0 projection ±20 %.

**Kill criterion**: if peak RSS at the auto-stride exceeds the fused-path peak by > 25 % on any
bench scene, the budget model is wrong and the stride policy is re-derived before slice 2.

### Slice 2 — complement rays (alternative B, full) *(1–2 days)*

`--depth-maps-ray-stride r ≥ 1`: the second streaming pass, reusing the carve replay verbatim.
Runs **before** the WSS classifier, so the free-space field WSS reads is complete.

**Gates**: § 8 energy gate on the two-stage bench (§ 7): mean paired mesh-F1 ≥ +0.003 beyond the
0.0006 noise floor, no scene regressing > 0.003, P/R each within 1 pp unless the net is a clear F
win. Must also beat the `carve` variant (alternative E) on the same clouds — that is the control.

**Kill criteria**:
- direct mode ≤ `carve` on 3 of 4 scenes → the extra evidence is not what is missing; stop and
  record it, exactly as 5.1 recorded its own sub-gate result;
- r=1 wall time exceeds the fused path's total (fusion + mesh) by more than the quality gain
  justifies at r=2 → ship r=2 as the recommended setting and say so.

### Slice 3 — per-pixel σ sources *(1 day)*

Plumb the per-pixel footprint (`depth/focal`, exact here) and the per-pixel confidence into the
existing σ_v accumulator (`vert_accum_t` in `SceneReconstruct.cpp`), so `--sigma-conf-shrink` works on
the direct path with a *better* input than the vertex-mean reconstruction it uses today.
`--footprint-sigma` itself was removed from the code (proven equivalent in effect to the
confidence signal on every scene tested — see `DelaunayMeshReconstruction.md`); this arm would
need to reintroduce it from git history (commit `c34d2623`) before it can be plumbed here. No
new switches otherwise.

**Gate**: the § 8 exact-result discipline — the arms must be no-ops when their flags are off — plus
a bench arm showing the direct+footprint combination against direct alone.

**Kill criterion**: if the direct mode's own σ statistics (median σ_v/σ, clamp rates) look nothing
like the fused path's (Phase 4.3: median 0.881 adaptive / 1.000 footprint, clamp < 3 %), the
sampling is producing a pathological density field — investigate before benching.

### Slice 4 — admission ladder *(1 day, conditional)*

Only if slices 2–3 show outlier vertices as a measurable failure mode. Cheapest first:
per-pixel support count from the dmap's own views-map (free, if slice 0 item 2 says it is there),
then a confidence percentile gate, then — last — a genuine cross-view check (alternative C).
Phase 4.4 of the improvement plan was VOID for lack of a driving failure; do not resurrect it
speculatively.

### Slice 5 — memory and streaming *(1–2 days, conditional)*

Only if peak RSS is the binding constraint after slice 2. Reuse `DMapCache`, release each dmap
after its ray pass, and consider a per-view release of the sample buffer. Gate: bounded peak with
scene size, identical output.

### Slice 6 — documentation and default decision *(0.5 day)*

`docs/design/DelaunayMeshReconstruction.md` verdict entry, README/CLI help, and an explicit
maintainer-reserved recommendation (default stays off unless the gate is cleared *and* the mode
works without dmaps present — which it cannot, so realistically: off forever, documented as the
high-evidence path).

---

## 7. Benchmark protocol

Harness: `bench/run_mesh.py` (two-stage, seeded sampling, in-crop face mask, official T&T
evaluator). New variants in `bench/mesh_variants.json`.

**Scenes**: Truck / Barn / Ignatius / Meetingroom. Truck / Ignatius / Meetingroom already have
`runFusionStats/` folders with **kept dmaps** from the Phase 5.0 re-densification — those are the
inputs. Barn needs the same treatment (one re-densification at
`--resolution-level 1 --number-views 12 --estimate-roi 0 --crop-to-roi 0 --tower-mode 0`) before it
can join.

**Baselines**: freshly measured on the same dmaps. *Never* compare against the frozen-cloud rows —
the Phase 5.1 caveat holds verbatim: the frozen clouds came from June code with a `Densify.ini`
overriding defaults and a pre-recalibration confidence lineage, and are 1.6–2.8× sparser.

**Two-stage scoring**: the cloud-F1 stage is undefined for the direct mode (there is no cloud).
Report the **fused** cloud's F1 as the shared reference row for the scene and compare mesh F1
only; the `d_f1` column then means "mesh vs the cloud the *other* arm would have built", which is
the honest reading and keeps the existing column semantics.

**Variants** to add:

```json
"dmap-s4-r0":   ["--use-depth-maps", "1", "--depth-maps-ray-stride", "0"],
"dmap-s4-r2":   ["--use-depth-maps", "1", "--depth-maps-ray-stride", "2"],
"dmap-s4-r1":   ["--use-depth-maps", "1", "--depth-maps-ray-stride", "1"],
"dmap-s2-r1":   ["--use-depth-maps", "1", "--depth-maps-vertex-stride", "2", "--depth-maps-ray-stride", "1"]
```

(The `--footprint-sigma` variant this slice originally planned is not currently runnable:
footprint sigma was removed from the code as proven equivalent in effect to the confidence
signal — see `DelaunayMeshReconstruction.md`. Reintroduce it from git history, commit
`c34d2623`, before adding a `-foot` variant back here.)

with `baseline` and `carve` (alternative E) as the two controls.

**Quality gate** (`DelaunayMeshReconstruction.md` § 8, unchanged): mean paired mesh-F1
≥ **+0.003** beyond the measured **0.0006** noise floor; **no scene regressing > 0.003**; P/R each
within 1 pp unless the complementary gain nets a clear F win; no topology regression; runtime and
memory documented.

**Speed gate — the headline claim, and it must include the skipped fusion**:

```
arm A (today)  = t(DensifyPointCloud: load dmaps -> fuse -> save scene_dense.mvs)
               + t(ReconstructMesh:   load scene_dense.mvs -> recon -> save mesh)
arm B (direct) = t(ReconstructMesh:   load scene.mvs -> read dmaps -> recon -> save mesh)
```

Both wall-clock, ≥ 3 runs, medians, identical machine and thread count, dmaps already on disk in
both arms (depth estimation is identical and excluded). Report peak RSS for both. Report the
per-stage split from `-v 3` (`tri_s` / `weight_s` / `wss_s` / `graphcut_s` plus new dmap-read and
complement-ray timers) so the trade is visible, not just the total.

**Reporting**: one table per slice with `mesh_f1`, `Δ vs baseline`, `Δ vs carve`, `recon_s`,
end-to-end wall for both arms, `peak_ws_mb`, `pts_inserted`, `delaunay_verts`, `cells`, and the new
counters. Same shape as every table already in `DelaunayMeshReconstruction.md`.

---

## 8. Risks and kill criteria

**R1 — memory blow-up.** *Quantified*: 1.95 kB/DT-vertex. Mitigation: the vertex budget is a
first-class parameter with an automatic power-of-two stride, not an afterthought.
*Kill*: peak RSS > 1.25× the fused path at the auto-stride (slice 1).

**R2 — ray cost eats the fusion saving and more.** *Quantified*: 1.6–3.3 µs/ray → +208…576 s at
r=1 against a 47–66 s saving. Mitigation: `r` is a knob, r=2 is wall-neutral, and the plan says so
before the code is written. *Kill*: nothing — this is a documented trade, but a slice that reports
only the quality number and hides the wall number is not accepted.

**R3 — the evidence is not what is missing.** Phase 5.1 got +0.0033 best-scene from the same class
of evidence, and the same table showed `fss` losing 0.03–0.07 F1 on dense clouds: the energy's
absolute-scale constants are miscalibrated for dense inputs. A sub-gate evidence effect cannot be
judged inside a miscalibrated energy. *Kill*: direct ≤ `carve` on 3/4 scenes → record and stop;
re-test only after a recalibration slice, exactly as 5.1 concluded.

**R4 — vote-mass inflation via `InsertViews`.** `InsertViews` (`:243-270`) **sums** the weight of
repeated views. Today a vertex merges a handful of fused points; in the direct mode it can absorb
tens of same-view pixels, so a vertex's α_vis from one view becomes proportional to local raster
density — the scene-dependent coupling § 2 note 1 of the improvement plan rejects by construction,
now amplified an order of magnitude. **Mitigation is mandatory, not optional**:
`depthMapsCapViewVotes` clamps to one unit per (vertex, view). *Kill*: if the capped and uncapped
arms differ by more than noise, the uncapped arm is a measurement artefact and must not be shipped
as a variant.

**R5 — determinism.** CGAL insertion is serial and order-dependent; a per-view streaming insert
would make the mesh a function of view enumeration order. Mitigation: buffer the sampled records
(160–240 MB, negligible) and keep the global `CGAL::spatial_sort`. *Kill*: two runs at the same
settings producing different `delaunay_verts` or `maxflow`.

**R6 — outlier vertices.** No `nMinViewsFuse`, no min-pixels, no reprojection or normal gate. The
confidence gate at 0.1 reproduces only fusion's *first* filter. Mitigation ladder in slice 4;
`footprint`/`conf`-driven σ_v (Phase 5.3: the physical field owns the object-scene gains) is the
principled soft version. *Kill*: precision drops > 1 pp with no recall compensation.

**R7 — logic drift against fusion.** Two places computing "depth pixel → world point" is how a
codebase grows a silent divergence. Mitigation: the direct path reuses the same back-projection
call as `Scene.cpp:406-428` / the fusion probe, and the reader is the shipped
`ImportDepthDataRaw` — the `.dmap` codec lives only in `MVS/Interface.h` and must not be
re-implemented (that mistake already cost this project three days once).

**R8 — dmaps missing, stale, or at the wrong resolution.** The mode reads
`ComposeDepthFilePath(image.ID, "dmap")` — a stale cache from a different `--resolution-level`
is a known crash source in this codebase. Mitigation: validate every header against
`images[i].GetSize()`-derived expectations, refuse with a clear message, never silently skip; and
document "clear `*.dmap` when changing resolution" in the CLI help.

**R9 — confidence provenance.** The gate assumes the recalibrated #1292 posterior
(`CONF_ADJUSTED`). Pre-#1292 or SGM-path dmaps store raw matching costs on a per-file `confScale`
(`Interface.h:863-865`), where a 0.1 gate means something else entirely. Mitigation: read the flag,
log the provenance, and refuse the confidence gate (or fall back to "any valid depth") when it is
absent.

**R10 — scope creep into fusion.** The temptation to "just also fix fusion while we're in there"
is high. This plan touches `SceneDensify.cpp` **not at all**.

---

## 9. Open questions

1. **Does the shipped dmap carry `HAS_VIEWS`?** If yes, every pixel arrives with up to 4 supporting
   neighbor indices — a free multi-view support count that makes slice 4's admission ladder nearly
   costless. Slice 0 answers it. (Note the known trap: the views-map is an index list, not a
   bitmask; a prior harness decoded it wrongly and produced a data void.)
2. **Should the mode be able to *write* a point-cloud?** Users who want both a mesh and a cloud
   would otherwise run fusion anyway, erasing the speed win. A `--export-sampled-cloud` on the
   direct path is cheap but expands the non-goal list; deferred.
3. **Is `distInsert` the right consolidation rule for a raster?** Its test breaks on the *first*
   view that says "far enough", so a point with more views is *more* likely to be inserted; a
   single-view record is therefore merged more aggressively than a fused point. That is probably
   the behaviour we want, but it means `--min-point-distance` has a different effective meaning in
   the two modes and the help text must say so.
4. **Does the complement-ray pass want the σ-shifted `D_in` term too?** Carve rays deliberately
   have none (no reliable surface position). With the raster, the position *is* the measurement —
   giving them a `t`-edge would make them full votes without a vertex. That is alternative C by
   another route; measure before assuming.
5. **Interaction with the 2026-08-22 default flips** (`--adaptive-sigma` on, `--canonical-rescale`
   on, `--max-edge-scale` 4, library `kSigma` 1.f) — all landed as defaults. Benches must state
   which side of those flips they ran on; anything predating them ran on the old side.
6. **Barn has no kept-dmap folder yet** — one re-densification is a prerequisite for a 4-scene gate.
7. **The mesh-vs-cloud fidelity gap** (Ignatius cloud 0.77 → mesh 0.34) is RESOLVED: it was a
   mesh-*cleaning* artefact (fixed-time MCF smoothing, fixed by the scale-free Laplacian) plus
   webbing on Truck-class scenes (fixed by the edge gate) — see `DelaunayMeshReconstruction.md`
   §3-§4. Consequence for this plan: score every arm on the raw+gated surface, and the headroom
   this plan chases is the remaining cloud-vs-mesh delta (e.g. Truck 0.7060 vs 0.6611), not the
   old 0.4-point chasm.

---

## 10. Related work — scope delimitation

Not a literature survey; only what bounds this plan's design space.

- **Labatut, Pons & Keriven (ICCV 2007)** — visibility-driven Delaunay graph cut on interest
  points: the energy `Scene::ReconstructMesh` implements. Its input was sparse keypoints with
  visibility, i.e. exactly "points + rays", which is why the code's separation of insertion from
  ray-walking is native rather than bolted on, and why feeding it rays without vertices is sound.
- **Jancosek & Pajdla (CVPR 2011, ISRN 2014)** — weakly-supported surfaces: the free-space-support
  classifier at `SceneReconstruct.cpp:1106` (the corrected ISRN triple, per § 0 of the improvement
  plan — do not reimplement). CMPMVS consumed depth-map-derived points directly, which is the
  input model this plan restores.
- **Vu, Labatut, Pons & Keriven (PAMI 2012)** — the dense-input, refinement-coupled version of the
  same pipeline; the reason "more points is not automatically better" is a known result rather than
  a surprise.
- **Volumetric / TSDF fusion** (Curless & Levoy 1996 and successors, including learned variants
  such as RoutedFusion) — a different family: regular grid, no visibility rays, no graph cut.
  Named to delimit scope; explicitly not proposed.
- **GPU visibility-based reconstruction (CAGD 2021)** — a full-GPU port of this exact pipeline
  (three-level Delaunay index, GPU ray traversal, GPU graph cut), recorded during the executed
  mesh plan's literature scan (git history). Relevant here only because the direct mode
  makes ray count the dominant cost, which is precisely what that work parallelizes.
