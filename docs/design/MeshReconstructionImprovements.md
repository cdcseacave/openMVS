# Improving OpenMVS Delaunay-Based Mesh Reconstruction — A Literature-Grounded Triage

> **Mode.** Research-only, focused exclusively on `Scene::ReconstructMesh` (the
> Delaunay + visibility + s-t cut surface extractor). The supplied images show the
> *raw* mesh — `Mesh::Clean()` is not on the path, so post-graph-cut cleanup,
> decimation, hole-closing, refinement, etc. are out of scope. No implementation
> here; this document reports what to consider doing, why, and where.
>
> **Method.** Re-derive the algorithm from its base paper (Labatut–Pons–Keriven,
> *CGF* 2009), audit the current OpenMVS implementation against it, then survey
> the published improvements 2009–2025 that touch the same family. Every code
> location is anchored.

---

## 1. Context

`Scene::ReconstructMesh` ([SceneReconstruct.cpp:773](libs/MVS/SceneReconstruct.cpp#L773))
is a faithful, mostly-canonical implementation of:

> **\[LPK09\]** Labatut, Pons, Keriven, *“Robust and Efficient Surface
> Reconstruction From Range Data”*, **CGF 28(8):2275–2290, 2009.**
> [Wiley](https://onlinelibrary.wiley.com/doi/abs/10.1111/j.1467-8659.2009.01530.x) ·
> [HAL](https://hal.science/hal-00712261/) · [PDF](https://www.cs.jhu.edu/~misha/ReadingSeminar/Papers/Labatut09.pdf)

extended with the *weakly-supported surfaces* free-space-support trick of:

> **\[JP14\]** Jancosek, Pajdla, *“Exploiting Visibility Information in Surface
> Reconstruction to Preserve Weakly Supported Surfaces”*, **ISRN 2014/798595**
> (journal version of CVPR 2011 paper).
> [Wiley/Hindawi](https://onlinelibrary.wiley.com/doi/10.1155/2014/798595) ·
> [Semantic Scholar](https://www.semanticscholar.org/paper/Multi-view-reconstruction-preserving-surfaces-Jancosek-Pajdla/8fb10b57a8c2fc19540c878a39b67a7c848e9ec4)

The visible failures on the test scene are:

1. **Coarse triangles in dense regions** (left wall and lower-right ivy) — input
   points are abundant, surfaces are oblique to the cameras, but the raw mesh
   spans them with a handful of large facets.
2. **Loss of multi-scale geometry** — the far house (right) reduces to a single
   oversized triangle even though hundreds of points support it in the cloud.

These are the canonical failure modes of the LPK09 family on multi-scale,
oblique-but-textureless data, and the literature has produced specific remedies.

---

## 2. The base algorithm \[LPK09\] in one page

The algorithm constructs the 3-D Delaunay tetrahedralization of the input points
and labels each tetrahedron `t ∈ T` *inside* (S) or *outside* (T̄) by minimising

```
                 E(x) = E_vis(x) + λ · E_qual(x)
```

via min-`s`-`t`-cut on the dual graph of `T`. The output surface is the union of
finite facets between cells of opposite labels.

* **Visibility (data) term `E_vis`.** Every line-of-sight from a sample point
  `p` to one of its observing cameras `c` traverses tetrahedra in two arcs:
  *camera-to-p* (must be empty) and *behind-p along the ray* (must be matter).
  For each facet `f` crossed by such a ray at distance `d` from `p`, LPK09
  contributes a soft penalty
  ```
              w_f = α(p) · (1 − exp(−d² / σ²))                    (Eq. 2 in [LPK09])
  ```
  where `α(p)` is the per-ray quality (Labatut uses scanner confidence; in MVS
  we plug in matching score) and `σ` controls how strict free-space is. These
  weights are added to the *forward arc* of the s-t graph for facets between
  the camera-cell and `p`-cell, and to the *t-arc* for facets behind `p`.

* **Quality (smoothness) term `E_qual`.** For each pair of tetrahedra `(c, c′)`
  sharing facet `f`, LPK09 (Eq. 4) adds
  ```
              R(f) = (1 − cos β(f)) · area(f)                     (Eq. 4 in [LPK09])
  ```
  where `β(f)` is the angle between `f`’s normal and the line connecting the
  two cells’ circumcentres (the *plane–sphere angle*). This term gives the
  energy a *minimal-surface* character — the cut prefers configurations that
  use less total area, and the `(1 − cos β)` factor punishes “sliver” cells.
  *Critically, the `area(f)` factor is what makes the regulariser **scale with
  the geometry**: large facets cost much more than small facets, so the cut
  cannot trivially span a big region with one giant triangle.*

* **Boundary conditions.** The infinite cells get `t = +∞` (forced outside).
  Cells that contain a camera centre (or are on a known "definitely empty"
  ray) get `s = +∞` (forced inside-camera, i.e. on the source side, which
  represents free space).

* **Why no shrinking bias.** Because `E_vis` is one-sided per ray (it pulls
  cells toward the matter side, never both ways) and the quality term scales
  with `area(f)` rather than counting facets, LPK09 escapes the standard
  graph-cut shrinking bias.

---

## 3. Audit: does OpenMVS implement \[LPK09\] faithfully?

Comparing line-by-line:

| LPK09 ingredient | OpenMVS | Verdict |
|---|---|---|
| 3-D Delaunay over input points | CGAL `Triangulation_3` at [SceneReconstruct.cpp:783–877](libs/MVS/SceneReconstruct.cpp#L783) | ✅ |
| Spatial sort for cache-friendly bulk insert | [SceneReconstruct.cpp:805](libs/MVS/SceneReconstruct.cpp#L805) | ✅ |
| Source anchor on camera cells | `s = kInf` for camera-cell facets ([:907–917](libs/MVS/SceneReconstruct.cpp#L907)) | ✅ |
| Visibility data term, forward arc `(1 − exp(−d²/σ²))` | `α_vis · (1 − exp(−d²/2σ²))` at [:980, :1007](libs/MVS/SceneReconstruct.cpp#L980) — same up to convention | ✅ |
| `α(p)` per-ray quality | `view.weight` ([:965](libs/MVS/SceneReconstruct.cpp#L965)) — populated from PatchMatch confidence in `pointcloud.pointWeights[i][v]` ([SceneDensify.cpp:1488,1828](libs/MVS/SceneDensify.cpp#L1488)) — but **default** `bUseConstantWeight=true` ([ReconstructMesh.cpp:130](apps/ReconstructMesh/ReconstructMesh.cpp#L130)) overrides it to 1 | ⚠️ deviates by config |
| `σ` chosen as a property of the data | `σ = √(median_edge²) · kSigma` — **single global value** for the whole tetrahedralisation ([:933](libs/MVS/SceneReconstruct.cpp#L933)) | ⚠️ single-scale only |
| Quality term **`(1 − cos β) · area(f)`** | `q = (1 − min(cos β_c, cos β_c′)) · kQual` ([:1119](libs/MVS/SceneReconstruct.cpp#L1119)) — **`area(f)` is omitted**; uses `min` over the two adjacent cells, not the symmetric form | ❌ deviates |
| `min`-`s`-`t`-cut over dual graph | IBFS or Boykov–Kolmogorov ([:1106, :1125](libs/MVS/SceneReconstruct.cpp#L1106)) | ✅ |
| Surface = boundary of the labelling | Direct boundary extraction with winding correction ([:1130–1162](libs/MVS/SceneReconstruct.cpp#L1130)) | ✅ |
| Manifoldness | Post-hoc `FixNonManifold` ([:1165](libs/MVS/SceneReconstruct.cpp#L1165)) | ⚠️ post-hoc, not built into the cut |
| **\[JP14\]** weakly-supported extension | `bUseFreeSpaceSupport` block at [:1027–1095](libs/MVS/SceneReconstruct.cpp#L1027) with the `(epsRel < kRel ∧ epsAbs > kAbs ∧ γ < kOutl)` interface classifier | ✅ implemented; defaulted *true* in test path ([Scene.h:148](libs/MVS/Scene.h#L148)) but *false* in CLI ([ReconstructMesh.cpp:131](apps/ReconstructMesh/ReconstructMesh.cpp#L131)) |

**One genuine deviation, two configuration mismatches, one architectural
limitation:**

1. **The `area(f)` factor is missing from the quality term.** This is not a
   minor stylistic choice; it changes what surface is cheapest. Without it the
   regulariser only depends on local shape (plane–sphere angle) and not on the
   *size* of the patch. The cut can place the surface on one giant facet
   instead of many small ones, **as long as the giant facet sits between
   well-shaped cells**, because all facets along the path are scored only by
   their `(1 − cos β)`. This is the algorithmic explanation for both the
   coarse-walls and one-triangle-per-far-house failure modes.

2. **`σ` is a single global value.** In a multi-scale scene, the median edge
   in the dense house dominates; `σ` is small; rays around the far house cross
   only a handful of cells and `1 − exp(−d²/σ²) → 1` for almost every facet
   they touch, so the data term provides no discrimination there.

3. **`bUseConstantWeight=true`** discards `pointcloud.pointWeights[i][v]`
   (PatchMatch matching confidence) that is computed and saved by
   `SceneDensify` and would be exactly the LPK09 `α(p)` were it used.

4. **Manifoldness fixed post-hoc** rather than enforced in the energy / graph
   construction — the modern alternative is \[RDPM16\] below.

---

## 4. Improvements to \[LPK09\] that have been published since (and how they map onto OpenMVS)

### 4.1 Surface-area-weighted regulariser — restore \[LPK09 Eq. 4\]

The simplest and best-justified change: re-introduce the missing area factor.

```cpp
// SceneReconstruct.cpp:1119, conceptually
const float A = facet_area(delaunay, ci, i);
const float q = (1.f - min_cos_beta) * A * kQual;
```

The Delaunay facet has three known vertices already in scope; the area is one
cross-product. This restores LPK09’s minimal-surface property, and does so in
units commensurate with `E_vis` (visibility weights summed along a ray scale
with the number of cells crossed, which itself scales with how the cells are
packed — area is the right geometric companion).

Calibration risk: `kQual` was tuned without the area factor, so the absolute
default will need to be re-fit. A first-order rescale is to divide `kQual` by
the median facet area at the time `σ` is computed (already on hand at
[:933](libs/MVS/SceneReconstruct.cpp#L933)).

> *Reference.* \[LPK09\] §3.2 Eq. 4. Also adopted explicitly in the Vu et al.
> (2012) and Jancosek-Pajdla (2014) formulations.

### 4.2 Per-cell or per-ray local σ — multi-scale visibility

> **\[MPFB17\]** Mostegel, Prettenthaler, Fraundorfer, Bischof, *“Scalable
> Surface Reconstruction from Point Clouds with Extreme Scale and Density
> Diversity”*, **CVPR 2017**.
> [arXiv:1705.00949](https://arxiv.org/abs/1705.00949) ·
> [PWC](https://paperswithcode.com/paper/scalable-surface-reconstruction-from-point)

Mostegel et al. demonstrate that a single `σ` cannot work across density
variations of more than ~1 order of magnitude (their experiments report 4
orders). Their solution is a coarse octree partition where each leaf builds a
*local* Delaunay tetrahedralisation; surface hypotheses are then merged by a
second graph cut.

Two ways to bring this idea into OpenMVS without adopting their full pipeline:

(a) **Per-vertex σ from k-NN distances.** Replace the global
   `σ = √(median_edge²)·kSigma` ([:933](libs/MVS/SceneReconstruct.cpp#L933))
   with `σ_v = mean(k-NN distance of v)` (CGAL kd-tree already present in the
   codebase). For every facet crossed by a ray from vertex `v`, use `σ_v` (or
   the mean of the two endpoint vertices’ σ) instead of the global one. This
   sharpens the data term in dense regions without sacrificing it in sparse
   ones. No partitioning or double cut; one extra `O(N log N)` pre-pass.

(b) **Octree-partitioned Delaunay (full \[MPFB17\]).** Substantially more
   intrusive (needs a tile-merge scheme, sub-Delaunay graph cuts, hypothesis
   merging). Worth considering if the goal is also to lift OpenMVS’s memory
   ceiling for very large scenes; otherwise, (a) captures most of the benefit
   for the user’s symptom.

### 4.3 Photo-consistency-aware `α(p)` — actually use the existing weights

> **\[VLPK12\]** Vu, Labatut, Pons, Keriven, *“High Accuracy and
> Visibility-Consistent Dense Multiview Stereo”*, **TPAMI 34(5):889–901, 2012.**
> [PubMed](https://pubmed.ncbi.nlm.nih.gov/21844631/) ·
> [HAL](https://enpc.hal.science/hal-00712178)

Vu et al. argue that the per-ray quality `α` should reflect the *photometric
consistency* of the patch around `p` in image `c`. OpenMVS already computes
exactly this: `pointcloud.pointWeights[i][v]` is filled from the PatchMatch
confidence map at densification time
([SceneDensify.cpp:1488,1828](libs/MVS/SceneDensify.cpp#L1488)). Today
[ReconstructMesh.cpp:130](apps/ReconstructMesh/ReconstructMesh.cpp#L130)
defaults `bUseConstantWeight=true`, so all `α_vis = 1`. Flipping the default
(or normalising the weights so their mean is 1 to keep `λ` roughly the same)
gives noisy / occluded rays a smaller voice while the consensus over a wall
gets stronger.

### 4.4 Manifold-preserving cut — Romanoni et al.

> **\[RDPM16\]** Romanoni, Delaunoy, Pollefeys, Matteucci, *“Automatic 3D
> Reconstruction of Manifold Meshes via Delaunay Triangulation and Mesh
> Sweeping”*, **WACV 2016**.
> [arXiv:1604.06258](https://arxiv.org/abs/1604.06258)

Instead of letting the cut produce non-manifold vertices and fixing them
post-hoc with `mesh.FixNonManifold()` ([:1165](libs/MVS/SceneReconstruct.cpp#L1165)),
\[RDPM16\] enforces manifoldness during reconstruction. They also iterate
*cut → mesh-sweep (Steiner-point insertion in the surface’s neighbourhood) →
re-cut*, which addresses the same problem as \[MPFB17\] from a different
angle. The Steiner-point part overlaps closely with adaptive-Delaunay schemes
\[SP07\] below.

### 4.5 Adaptive subdivision — Sinha & Pollefeys

> **\[SP07\]** Sinha, Mordohai, Pollefeys, *“Multi-View Stereo via Graph Cuts
> on the Dual of an Adaptive Tetrahedral Mesh”*, **ICCV 2007 / 3DPVT 2007**.
> [PDF](https://mordohai.github.io/public/Sinha_GraphCutsAdaptiveMesh07.pdf)

Predates \[LPK09\] but anticipates the multi-scale problem: a coarse
tetrahedral mesh of the bounding volume is *adaptively subdivided* where
photo-consistency suggests the surface lies. The Steiner-point variant of this
idea is the natural sequel to \[LPK09\]: after one graph cut, identify
tetrahedra where the data term gradient is high but `σ` saturated (i.e. many
rays accumulated near-equal weights), insert their incentre as a new vertex,
and re-cut. Two passes, no library swap; the upper bound on cost is a second
graph cut whose graph size is a small multiple of the first.

### 4.6 Learning-based labelling — Sulzer et al.

> **\[SLMV21\]** Sulzer, Landrieu, Marlet, Vallet, *“Scalable Surface
> Reconstruction with Delaunay-Graph Neural Networks”*, **SGP 2021** (CGF 40-5).
> [arXiv:2107.06130](https://arxiv.org/abs/2107.06130) ·
> [Code](https://github.com/raphaelsulzer/dgnn)

Replaces the hand-engineered visibility/quality energies with a graph neural
network on the Delaunay 1-skeleton, then still solves a graph cut over the
learned per-cell potentials. Trained on synthetic data, generalises to real
MVS, and explicitly outperforms classical \[LPK09\]/\[JP14\]/\[VLPK12\]
pipelines on defect-laden inputs. This is a research direction more than a
practical OpenMVS change today (training data, runtime), but it confirms that
the limit of hand-engineered energies is now visible.

### 4.7 Other relevant work

- *DeepDT* (Liu, Wang, Cheng, *AAAI 2021*) — also a learning-based per-cell
  classifier on the Delaunay graph, similar spirit to \[SLMV21\].
- *AliceVision* — open-source MVS that includes a maintained \[JP14\] derivative
  with several robustness tweaks (per-camera weighting, robust outlier
  pre-filtering); worth diffing the AliceVision source for ideas that directly
  port back.
  [GitHub](https://github.com/alicevision/AliceVision)
- *Hallucination-Free Multi-View Stereo* (Mauro et al., *ECCV 2012*) — adds a
  consistency term aimed at suppressing surfaces in regions with no support
  from any view; the symmetric problem to weakly-supported surfaces.
- *Scale Robust MVS* (Kuhn et al., *ECCV 2012*) — explicit scale handling;
  predates \[MPFB17\] and motivates per-point σ.

---

## 5. Mapping the literature to specific code changes in `Scene::ReconstructMesh`

| # | Change | Citation | Anchor | Symptoms targeted |
|---|---|---|---|---|
| **1** | Re-introduce `area(f)` in the smoothness term | \[LPK09\] Eq. 4 | [:1119](libs/MVS/SceneReconstruct.cpp#L1119) | Coarse walls **and** one-triangle far house — both stem from the missing area factor |
| **2** | Replace global `σ` with per-vertex (k-NN-derived) `σ_v` | \[MPFB17\] §3, \[Kuhn12\] | [:933 + ray loops :980, :1007](libs/MVS/SceneReconstruct.cpp#L933) | Far house (multi-scale) primarily; some sharpening of dense walls |
| **3** | Default `bUseConstantWeight=false` (or expose calibration) so PatchMatch confidence is actually used | \[VLPK12\] §3 | [ReconstructMesh.cpp:130](apps/ReconstructMesh/ReconstructMesh.cpp#L130), [SceneReconstruct.cpp:965](libs/MVS/SceneReconstruct.cpp#L965) | Both — but particularly removes spurious facets in low-confidence corners |
| **4** | Add Steiner refinement: one extra cut where data term saturated | \[SP07\] §4, \[RDPM16\] §3.3 | new pre-extraction pass after [:1125](libs/MVS/SceneReconstruct.cpp#L1125) | Coarse walls (subdivides exactly where the data is uncertain) |
| **5** | Manifold-aware cut to eliminate the post-hoc fix | \[RDPM16\] §3.2 | replaces [:1165](libs/MVS/SceneReconstruct.cpp#L1165) `FixNonManifold` | Quality / topology, not the visible symptoms — but reduces a class of artifacts the user has not yet flagged |
| **6** | Per-ray solid-angle (or `1/depth`) modulation of `α_vis` | \[VLPK12\] §3.2 | [:980, :1007](libs/MVS/SceneReconstruct.cpp#L980) | Multi-scale: rays from far cameras don’t over-vote on close-up tets, and vice versa |
| **7** | Symmetric quality term: replace `min(cos β_c, cos β_c′)` with the average / sum (LPK09 sums both directions on the same edge) | \[LPK09\] §3.2 | [:1119](libs/MVS/SceneReconstruct.cpp#L1119) | Slight; mostly a faithfulness fix that pairs with #1 |
| **8** | Anchor unbounded cells **outside** when no ray reaches them | beyond \[LPK09\]; standard practice (e.g. AliceVision) | new pass at [:907–917](libs/MVS/SceneReconstruct.cpp#L907) | Far house — currently `kInf` only flows from cameras to source; sink side is never anchored, so unsupported far cells can flip arbitrarily |
| **9** | Replace energies with a learned GNN (research direction) | \[SLMV21\], DeepDT | wholesale | All; out of scope for an incremental fix |

The single highest-leverage entry is **#1**, because it is (a) the one outright
deviation from the base paper, (b) explains both visible symptoms via a single
mechanism, (c) is a one-call code change at a known site, and (d) does not
require touching the `α_vis` calibration that \[VLPK12\]-style improvements
also touch.

---

## 6. Symptom → recommended changes (read this if you only have time for one tier)

### Coarse triangles in dense, oblique-to-camera surfaces

Root mechanism: the data term saturates over many adjacent cells (every ray to
a nearby surface point contributes near-1 weight to many facets), so the
choice of cut is dominated by the smoothness term. Without an area factor (#1),
that smoothness term cannot tell apart "one giant facet" from "many small
facets that follow the supporting points" — they have similar `(1 − cos β)`
along well-shaped cells.

→ **Fix order:** #1 (area) → #4 (Steiner where saturated) → #3 (real `α_vis`
to break ties) → #7 (symmetric q).

### Distant points collapse to one large triangle

Root mechanism: a single global `σ`, fitted on the dense region, makes the
data term in the sparse far region effectively binary — any facet a ray
crosses gets weight ≈ α, regardless of distance. Combined with the
no-area-factor smoothness, the cheapest cut is one large facet through the
sparse region. Compounded by the absence of a sink-side anchor for distant
unbounded cells.

→ **Fix order:** #1 (area) → #2 (local σ) → #8 (anchor unbounded) → #6
(distance-aware ray weight).

---

## 7. Suggested order of investigation (research)

The user has authorised updating the test bounds in
[apps/Tests/TestsMVS.cpp:65, :73](apps/Tests/TestsMVS.cpp#L65) as quality
improves. A pragmatic experiment sequence:

1. **Baseline freeze.** With `verbose=true`, capture
   `data/scene_dense_mesh.ply` per current defaults (already present in the
   working tree).
2. **Single-knob sweeps** to measure the design space of the existing energy
   before changing it: vary `kSigma`, `kQual`, `bUseConstantWeight`,
   `bUseFreeSpaceSupport`. Plot face count, `ComputeReconstructionQuality()`
   ([TestsMVS.cpp:88](apps/Tests/TestsMVS.cpp#L88)), and a chamfer distance
   from cloud → mesh (one helper using existing `ComputeDistance` machinery).
3. **Apply #1 (area-weighted q)** and re-sweep `kQual` (its physical units
   change). Expect both symptoms to improve markedly with no other change.
4. **Layer #2 (local σ)** on top, sweep the σ floor.
5. **Layer #3 (`bUseConstantWeight=false` calibrated)**, sweep a global α
   normaliser so the data/smoothness balance is preserved.
6. Optional — try #4 (Steiner refinement) only if the residual coarseness
   still matters after 3-5.

---

## 8. Verification (how to know an improvement is real)

- `ComputeReconstructionQuality().score()` floor in
  [TestsMVS.cpp:88](apps/Tests/TestsMVS.cpp#L88): currently 45.
- Per-component face count and edge-length histogram, dumped near
  [SceneReconstruct.cpp:1162](libs/MVS/SceneReconstruct.cpp#L1162). A
  one-large-triangle component identifies the multi-scale failure mode without
  human inspection of the renders.
- Cloud→mesh chamfer distance over the test point cloud (a small helper using
  `ComputeDistance` in `Mesh.cpp` is sufficient).
- Visual side-by-sides of `data/scene_dense.ply` vs `data/scene_dense_mesh.ply`
  (verbose path already saves both).
- Test bound update protocol: change `ISINSIDE(faces.size(), …)` ranges
  ([TestsMVS.cpp:65, :73](apps/Tests/TestsMVS.cpp#L65)) in the same commit as
  any energy change that shifts the count, with the new lower bound set
  conservatively given existing PatchMatch threading non-determinism (memory
  `project_tests2_flakiness`).

---

## 9. Files most likely to be touched if any of this is implemented

| File | Reason |
|---|---|
| [libs/MVS/SceneReconstruct.cpp](libs/MVS/SceneReconstruct.cpp) | Quality term (#1), σ (#2), `α_vis` use (#3, #6), Steiner pass (#4), unbounded anchor (#8), symmetric q (#7) |
| [libs/MVS/Scene.h:148](libs/MVS/Scene.h#L148) | Defaults for `ReconstructMesh` |
| [apps/ReconstructMesh/ReconstructMesh.cpp:130](apps/ReconstructMesh/ReconstructMesh.cpp#L130) | `bUseConstantWeight` default, possible new flags |
| [apps/Tests/TestsMVS.cpp:65, :73](apps/Tests/TestsMVS.cpp#L65) | Test bound updates |
| (read-only context) [libs/MVS/SceneDensify.cpp:1488](libs/MVS/SceneDensify.cpp#L1488) | Source of `pointcloud.pointWeights` already populated by PatchMatch |

---

## 10. Out of scope (per user)

- `Mesh::Clean` (decimation, spurious removal, hole closing, smoothing,
  isotropic remeshing).
- `RefineMesh` and any post-cut variational / photometric refinement.
- Replacement of the IBFS / Boykov–Kolmogorov solver — not a quality knob.
- GPU acceleration of any stage — orthogonal to fidelity.

---

## References

* **\[LPK09\]** Labatut, Pons, Keriven. *Robust and Efficient Surface
  Reconstruction From Range Data.* CGF 28(8), 2009.
  [Wiley](https://onlinelibrary.wiley.com/doi/abs/10.1111/j.1467-8659.2009.01530.x) ·
  [HAL](https://hal.science/hal-00712261/) ·
  [PDF](https://www.cs.jhu.edu/~misha/ReadingSeminar/Papers/Labatut09.pdf)
* **\[VLPK12\]** Vu, Labatut, Pons, Keriven. *High Accuracy and
  Visibility-Consistent Dense Multiview Stereo.* TPAMI 34(5), 2012.
  [PubMed](https://pubmed.ncbi.nlm.nih.gov/21844631/) ·
  [HAL](https://enpc.hal.science/hal-00712178) ·
  [PDF](http://islab.ulsan.ac.kr/files/announcement/441/PAMI-2012%20High%20Accuracy%20and%20Visibility-Consistent%20Dense%20Multiview%20Stereo.pdf)
* **\[JP14\]** Jancosek, Pajdla. *Exploiting Visibility Information in Surface
  Reconstruction to Preserve Weakly Supported Surfaces.* ISRN 798595, 2014
  (extends CVPR 2011).
  [Wiley/Hindawi](https://onlinelibrary.wiley.com/doi/10.1155/2014/798595) ·
  [Semantic Scholar (CVPR 2011)](https://www.semanticscholar.org/paper/Multi-view-reconstruction-preserving-surfaces-Jancosek-Pajdla/8fb10b57a8c2fc19540c878a39b67a7c848e9ec4)
* **\[MPFB17\]** Mostegel, Prettenthaler, Fraundorfer, Bischof. *Scalable
  Surface Reconstruction from Point Clouds with Extreme Scale and Density
  Diversity.* CVPR 2017.
  [arXiv:1705.00949](https://arxiv.org/abs/1705.00949)
* **\[RDPM16\]** Romanoni, Delaunoy, Pollefeys, Matteucci. *Automatic 3D
  Reconstruction of Manifold Meshes via Delaunay Triangulation and Mesh
  Sweeping.* WACV 2016.
  [arXiv:1604.06258](https://arxiv.org/abs/1604.06258)
* **\[SP07\]** Sinha, Mordohai, Pollefeys. *Multi-View Stereo via Graph Cuts
  on the Dual of an Adaptive Tetrahedral Mesh.* ICCV 2007 / 3DPVT 2007.
  [PDF](https://mordohai.github.io/public/Sinha_GraphCutsAdaptiveMesh07.pdf)
* **\[SLMV21\]** Sulzer, Landrieu, Marlet, Vallet. *Scalable Surface
  Reconstruction with Delaunay-Graph Neural Networks.* SGP 2021 (CGF 40-5).
  [arXiv:2107.06130](https://arxiv.org/abs/2107.06130) ·
  [Code](https://github.com/raphaelsulzer/dgnn)
* **\[Kuhn12\]** Kuhn, Mayer. *Scale Robust Multi-View Stereo.* ECCV 2012.
  [Springer](https://link.springer.com/chapter/10.1007/978-3-642-33712-3_29)
* AliceVision Meshing module (open-source \[JP14\]/\[VLPK12\] derivative).
  [GitHub](https://github.com/alicevision/AliceVision)
