# Delaunay Mesh Reconstruction

Consolidated record of the Delaunay-visibility mesh reconstruction effort
(`Scene::ReconstructMesh`, `libs/MVS/SceneReconstruct.cpp`, cleaning in `libs/MVS/Mesh.cpp`).
This document is the single source of truth for: what ships today and why, the validated
before/after numbers, and a registry of every idea that was tried and rejected — read the
registry (§5) before re-proposing any of them, the mechanism that killed each one is recorded
there. The phase-by-phase task list and per-slice experimental log that produced this record
have been superseded and removed; `git log` / prior commits carry the full history if a
derivation needs to be re-checked.

Effort dates: 2026-08-18 to 2026-08-22. Two adjacent tracks are out of scope here and have their
own plans: depth-maps as direct mesh input (bypassing the fused cloud) is
`docs/design/DepthmapMeshingPlan.md`; dense-fusion re-baselining and the family-B benefit
analysis is `docs/design/FusionImprovementPlan.md`.

**Adjudication note.** Every mesh-F1 number recorded before 2026-08-21 was scored through a
mesh-cleaning smoother bug that crushed scores by 20-50 points of F1 on fine-resolution scenes
(§3). Verdicts and effect sizes from that period are unreliable — several signs flipped once the
bug was fixed and everything was re-scored (§5). This document reports only the corrected,
re-evaluated numbers; do not resurrect a pre-2026-08-21 number from git history as evidence.

---

## 1. The algorithm as shipped

`ReconstructMesh` builds a Delaunay tetrahedralization of the input point cloud, accumulates a
Labatut/Pons/Keriven visibility energy on its cells and facets (camera-to-point soft-visibility
votes, a σ-shifted `D_in` unary, a β-skeleton quality term, and an optional free-space-support
(WSS) classifier for weakly-observed surfaces), solves an s-t min-cut (IBFS), and extracts the
cut boundary as the mesh surface. `Mesh::Clean` then removes long/spurious/spike faces, closes
holes, and smooths.

### Current defaults, and why

| Default | Value | One-line reason |
|---|---|---|
| `--adaptive-sigma` | on | per-vertex σ_v = kSigma × median incident Delaunay edge length, clamped to [0.25,4]× the global σ — a universal win across all four T&T scenes and simultaneously the fastest arm (§2) |
| `--canonical-rescale` | on | rescales the triangulation by a power of two so the median edge lands near 1, where the ray-walk `orientation()` predicate's fixed 1e-12 epsilon is calibrated; provably a no-op inside the band every normal scene lives in, and a correctness fix (not just a speed one) outside it (§6) |
| `--max-edge-scale` | 4 | drops cut facets whose longest edge exceeds 4× the median cut-facet longest edge — the webbing gate (§4); a universal win, better-or-equal to k=6 on all four scenes, recall untouched |
| library `kSigma` | 1.f | matches the CLI's long-standing `--thickness-factor` default of 1; the old library default of 2 loses 0.043-0.146 F1 to this value on every scene (§5) |
| `Mesh::Clean` smoothing | scale-free Laplacian | replaces `CGAL::PMP::smooth_shape`, whose fixed absolute time step over-smoothed fine meshes by ~20x (§3); the new smoother moves each vertex relative to its own one-ring scale, so it is unit- and resolution-independent |

### Opt-ins, and when to reach for them

- **Object-scene stack** (`--constant-weight 0 --quality-co-scale 1`, on top of adaptive σ): the
  single best result on object-centric captures (Ignatius 0.7497, Truck 0.6558) but costs Barn
  −0.020 and Meetingroom −0.019 — use it only when the capture is object-centric and there is no
  planar/facade content in frame (§5).
- **`--sigma-conf-shrink`**: real signal, but redundant once adaptive σ is on (they draw on the
  same information and do not stack, §5) — use it only with `--adaptive-sigma 0`, i.e. as an
  alternative per-vertex σ source, not an addition to the default.
- **`--carve-rays-file`**: unfused high-confidence depth pixels replayed as free-space-only rays,
  no vertex inserted (introduced by this effort) — feed it via
  `DensifyPointCloud --export-unfused-file`; below the default-flip gate but never harmful, keep
  as an opt-in evidence source, particularly for weakly-covered scenes.
- **`--free-space-support`**: the long-standing upstream WSS classifier for weakly-supported
  surfaces (e.g. thin/textureless walls with few crossing rays); default off — it costs ~0.05
  F1 on the two dense object scenes tested (§5), so reach for it only on a genuinely
  weak-surface use case, not as a general-purpose accuracy lever.

---

## 2. Validated results

**Scoring protocol.** Frozen `scene_dense.mvs` per scene (identical geometry/views/confidence
across all variants compared), raw graph-cut surface (pre-`Mesh::Clean`) with the
`--max-edge-scale` gate applied, in-crop 10M-sample area-uniform mesh sampling (seeded), scored
against ground truth by the official Tanks-and-Temples evaluation toolbox at each scene's
official τ. Noise floor (paired identical baseline runs, 4 scenes): **max |ΔF1| = 0.0006**. Every
number below is a mean over ≥1 run at that noise floor; §5 entries with per-run spread say so.

### Recommended default vs originally shipped vs input cloud

| scene | originally shipped | adaptive-σ + gate k=4 | input cloud |
|---|---|---|---|
| Ignatius | 0.3295 | **0.7427** | 0.7381 |
| Truck | 0.3569 | **0.6611** | 0.7060 |
| Barn | 0.5576 | **0.6257** | 0.5988 |
| Meetingroom | 0.3379 | **0.4036** | 0.3225 |

Three of four scenes now score above their input cloud (Barn, Ignatius, Meetingroom); Truck
reaches 94% of its cloud's F1. Versus the originally shipped defaults, the recommended
configuration gains +0.07 to +0.41 F1 per scene — all of that gain is the compounded effect of
fixing the smoother (§4) plus adopting adaptive σ and the webbing gate, not any single change in
isolation.

**End-to-end validation of the flipped binary** (2026-08-23, defaults only, no flags): the raw
surface reproduces the table above — Ignatius 0.7441, Truck 0.6614, Barn 0.6258, Meetingroom
0.3974 (the small Meetingroom delta is the in-recon k=4 gate vs the offline k=6 scoring of the
campaign arm) — and the full default pipeline including `Mesh::Clean` delivers Ignatius 0.7358,
Truck 0.6604, Barn **0.6360**, Meetingroom **0.4037**: the clean now trades a little recall for
precision on the object scenes and outright improves both τ=10mm scenes.

### Webbing-gate k-sweep (raw graph-cut surface, no other change)

| scene | raw (ungated) | k=8 | k=6 | k=4 | input cloud |
|---|---|---|---|---|---|
| Ignatius | 0.6986 | 0.7021 | 0.7036 | **0.7048** | 0.7381 |
| Truck | 0.4835 | 0.6202 | 0.6298 | **0.6441** | 0.7060 |
| Barn | 0.5704 | — | 0.6069 | **0.6144** | 0.5988 |
| Meetingroom | 0.2185 | — | 0.3959 | **0.3961** | 0.3225 |

k=4 is better-or-equal to k=6 on all four scenes; recall never moves by more than 0.008 at any k.
Gated raw already beats the input cloud on Barn and Meetingroom before adaptive σ is even added.

### Object-scene stacking (gated k=6 raw surface)

| config | Ignatius | Truck | Barn | Meetingroom |
|---|---|---|---|---|
| gated baseline | 0.7036 | 0.6298 | 0.6069 | 0.3959 |
| adaptive-σ | 0.7427 | 0.6451 | 0.6191 | 0.4036 |
| adaptive + weighted + co-scale | **0.7497** | **0.6558** | 0.5989 | 0.3848 |
| + conf-shrink 0.5 (triple stack) | 0.7492 | 0.6579 | — | — |

The weighted+co-scale stack adds a further +0.007/+0.011 over adaptive alone on the two object
scenes, but regresses both planar scenes (Barn −0.020, Meetingroom −0.019) — an opt-in, not a
default (§1, §5).

### Speed

Adaptive σ is not just the most accurate arm, it is also the fastest: Ignatius graph-cut solve
32.4s vs 35-50s for every other single arm tested; Truck 65.5s, on par with the conf-shrink arm.
The weighted-vote arms pay 1.5-3x more solve time (Ignatius weighted+co-scale 108s). Free-space
support carves fastest on Truck (42.5s) but loses 0.05 F1, so the speed is not worth taking.

---

## 3. Why every earlier number was wrong

The mesh stage appeared to *destroy* fidelity relative to its input cloud (e.g. Ignatius cloud
0.77 -> mesh 0.34) in every measurement since the VCG-to-CGAL cleaning switch landed in March
2026. The cause was never the graph-cut estimation — it was `Mesh::Clean`'s smoothing step.

**Root cause** (commit `c99883fc`, "mesh: remove VCG and use CGAL for cleaning"): VCG's
scale-free Laplacian smoothing was replaced by CGAL `PMP::smooth_shape` — implicit mean-curvature
flow with a fixed absolute time step of 1e-3. That constant has units of *squared scene length*:
it was evidently tuned against ~3cm-edge meshes (0.03² ≈ 1e-3) and over-smooths by roughly 20x on
Ignatius' ~7mm-edge statue mesh, or on any metric-scale fine-resolution scene — and a single
global time constant cannot fit a mixed-resolution mesh (fine statue + coarse background) at any
setting.

**The dose-response is airtight** (Ignatius, official eval):

| stage | F1 |
|---|---|
| input cloud | 0.7381 |
| raw graph-cut mesh (pre-clean) | 0.6986 |
| after full Clean minus smooth (`--smooth 0`) | 0.6986 |
| after 1 smooth iteration (old MCF) | 0.4645 |
| after 2 smooth iterations = shipped default | 0.3295 |

Every non-smooth clean step combined — long-edge removal, component removal, spike removal,
hole-closing — costs exactly nothing (0.6986 → 0.6986). The two MCF smoothing iterations produce
the entire collapse, with a clean monotone dose-response, and the cleaned mesh loses 21% of its
in-crop surface area to MCF shrinkage.

**Fix**: `Mesh::Clean` now smooths with a classic per-vertex uniform-Laplacian relaxation
(λ=0.5, `nSmooth` iterations, borders fixed, deterministic double-buffered update) — each vertex
moves relative to its own one-ring scale, so the result is unit- and resolution-independent.
Landed as commit `c6446c3c`; the webbing gate (§4) followed separately as `67c94292`. Post-fix,
the shipped default
(smooth=2) lands at or above the raw mesh on both object scenes (Ignatius −0.0026 vs raw with
precision up 0.699→0.718; Truck +0.018 vs raw) and clearly above the previously shipped result on
all four scenes (Barn +0.035, Meetingroom +0.027, the latter now above its own input cloud).

**Consequence for this record**: any mesh-F1 number from before 2026-08-21 in git history —
including every A/B verdict this effort produced during Phases 0-5.3 — was scored through the
broken smoother. Effect sizes are unreliable and several signs are wrong (§5 lists every case
where the corrected number reverses or dominates the old one). Do not cite a pre-2026-08-21
mesh-F1 number as evidence for anything.

---

## 4. The webbing gate

**Webbing**: the visibility cut stretches surface across occluded space it has no evidence
about — under vehicles, behind interior walls, anywhere the camera ring cannot see. These facets
carry zero visibility votes and are uncarvable by construction, since no ray reaches occluded
space to begin with. Truck is the diagnostic case: raw mesh recall is healthy (0.686, ≈ its
cloud) but precision is 0.373 — 10% of faces sit more than 30mm from any input point (p99 305mm)
and carry 41% of the in-crop sampled area. The default `--remove-spurious 20` cannot touch them;
its threshold resolves to ~10 meters on these meshes.

**First attempt, REFUTED — visibility-mass gate.** The obvious estimation-side signal is the
α_vis crossing mass each facet accumulates during the visibility walk (webbing should be
mass-zero — no ray enters occluded space). Implemented as `--min-surface-evidence` and
benchmarked:

| arm | facets removed | P | R | F1 |
|---|---|---|---|---|
| Truck raw (no gate) | — | 0.3733 | 0.6860 | 0.4835 |
| Truck mass < 1e-6 | 2.99M / 4.97M | 0.3850 | 0.5701 | 0.4596 |
| Truck mass < 0.05 | 3.01M / 4.97M | 0.3836 | 0.5669 | 0.4576 |
| Ignatius raw (no gate) | — | 0.6992 | 0.6980 | 0.6986 |
| Ignatius mass < 1e-6 | 2.54M / 4.11M | 0.6876 | 0.6175 | 0.6507 |
| Ignatius mass < 0.05 | 2.57M / 4.11M | 0.6849 | 0.6078 | 0.6440 |

It does not work: ~60% of cut facets carry mass exactly zero on *both* scenes, including most of
Ignatius' true statue surface, which has essentially no webbing. Mechanism: each ray is a 1D
needle through the tetrahedralization — it crosses only 1-2 facets of a vertex's ~20-facet
umbrella, so vote mass lives on a sparse subset of the real surface. No mass threshold separates
webbing (zero) from true surface (also mostly zero). Recall collapses, precision barely moves.
Removed from code.

**Shipped gate — `--max-edge-scale`.** Every Delaunay vertex IS an input point, so a facet can
only stray far from the observed cloud by spanning it with long edges — a purely geometric
signal, and it works. Drops extracted cut facets whose longest edge exceeds k× the median
cut-facet longest edge (medians computed in the triangulation's working space so the canonical
rescale cancels out — ratio of medians, scale-free). Calibration: Truck's raw median max-edge is
17.9mm, so k=6 (107mm) drops 9.2% of faces, matching an offline 100mm-threshold prototype
(9.7%); Ignatius' median is 42mm (background-dominated), so the same k=6 (254mm) sits far above
the ~7mm statue facets and removes only true gap-spanners. See §2 for the k-sweep table — k=4 is
better-or-equal to k=6 on all four scenes, recall untouched in every case. Landed as commit
`67c94292`.

---

## 5. Failed and rejected ideas — do not retry without new evidence

Every entry below was benchmarked on the honest post-2026-08-21 metric (§2's protocol) unless
marked otherwise. Numbers are Δ vs that scene's gated baseline.

**Grazing-incidence down-weighting** (`--grazing-floor` / `--grazing-exponent`). Scaled each
crossed-facet vote by `max(floor, |cos(ray, facet_normal)|^exp)`. Old (broken-smoother) numbers
suggested a small object-scene win; re-evaluated it is harmful everywhere: Ignatius −0.048, Truck
−0.027 at floor 0.2. The apparent old gain was entirely an artifact of the broken smoother.
**REMOVED from code.**

**WSS enforcement semantics** (`--wss-semantics paper|add|max`, vs the shipped `product`).
`paper` (ISRN-2014's literal sum-then-multiply) turns out to be ≡ `product` on dense clouds — a
single multiplication at the absolute α scale the classifier fires at is already effectively
infinite, so the two forms produce near-identical cuts (Ignatius/Meetingroom byte-identical face
counts). `add` (`t += kw·εabs`) collapses Ignatius 0.272→0.045 because the structural `t==0`
no-op — base t deposited at ~1σ behind the point, enforcement targeting the ~4σ walk-end cell,
usually a different cell — is *protective*, not a defect: it keeps the classifier from planting
surface priors in deep free space. **Never "fix" the t==0 no-op.** `max` still costs Ignatius
−0.046 for the same reason (it still fires on t==0 cells). **REMOVED from code**; the shipped
per-firing `product` is the only enforcement behavior again.

**Footprint-based σ** (`--footprint-sigma`, per-pixel range/focal as an alternative σ_v source
to adaptive). Proven ≡ `--sigma-conf-shrink` in effect on every scene tested (within noise on
all four: 0.5574/0.5569, 0.3382/0.3380, 0.3384/0.3385, 0.3599/0.3605) — two independent
implementations of the same physical signal (near/well-observed → tighter σ). Loses to adaptive
σ as a base (Barn −0.0073) and runs up to 2.3x slower on some scenes (Ignatius 229s vs ~98s for
the confidence arms). **REMOVED from code.** The σ_v design space has exactly two independent
signals — *physical* (confidence ≡ footprint) and *sampling-density* (median incident edge) —
and in the `1 − s·conf` shrink formulation they do not stack (see next entry).

**Confidence-shrink stacking on adaptive σ** (`--sigma-conf-shrink` on top of
`--adaptive-sigma`). ±0.002, scene-inconsistent — noise. **Kept opt-in** because alone (adaptive
off) it is real: +0.018 on Ignatius (conf-shrink-only vs gated baseline).

**Weighted votes alone** (`--constant-weight 0`, no co-scale). Ignatius cut collapses to 241k
faces (F1 0.4898, −0.214 vs baseline); Truck −0.011. Mechanism: every data-term capacity shrinks
by the mean point confidence (~0.3-0.7) while the quality term `q` and the camera `kInf`
constraints keep their unit-vote calibration, so the cut collapses inward toward the smoothness
term. **Rejected alone**; see the co-scale stack below for the fix.

**Object-scene stack** (`--constant-weight 0 --quality-co-scale 1`, on adaptive σ). Rehabilitates
the collapse above by scaling `kQual` by the same mean confidence: Ignatius 0.7497 (cloud+0.012),
Truck 0.6558 — but Barn −0.020, Meetingroom −0.019. **Opt-in, not default** (§1).

**kAbs/kOutl proportional rescale** (WSS absolute-scale constants swept 0.5x-4x together, under
`--free-space-support 1`). All 9 rows land below the no-fss baseline; the preferred direction is
scene-inconsistent (outdoor scenes prefer 0.5x, indoor prefers 4x). Mechanism: 43-88% of firings
saturate the t-edge at *every* setting tested — the product-semantics enforcement is effectively
a binary cell-nuke, and the constants only choose *which* cells get nuked, never *how hard*.
**Rejected**; the fix (if any) is in the enforcement semantics, not the constants, and every
semantics alternative was independently rejected above.

**Solver swap, EIBFS vs the bundled IBFS.** Speed-neutral on the real 22.7M-node Truck graph:
EIBFS-I-NR solve 27.1-27.7s vs IBFS 26.2-27.0s, interleaved runs. The stronger sibling (EIBFS-I)
crashes at scale (access violation in `augmentExcesses`, reproducible, independent of index
width). No license-clean *and* faster solver exists: the fastest candidates (EIBFS-I) carry the
TAU "research purposes only" license, same restriction class as the in-tree IBFS; the only truly
open alternative (Boost's Boykov-Kolmogorov) is the paper's slowest serial tier. **IBFS stays.**

**Free-space-support default-on.** Costs −0.048 (Ignatius) to −0.052 (Truck) at default constants
even after every recalibration/semantics attempt above failed to rescue it. **Stays available**
(long-standing upstream feature) for genuinely weakly-supported-surface use cases; **default
off.**

**Thickness-factor 2 / old library `kSigma`=2.** Strictly worse than kSigma=1 on every scene:
Ignatius −0.146, Truck −0.043. **Library default corrected to 1.f**, matching the CLI's
long-standing `--thickness-factor` default.

**Carve-only rays** (`--carve-rays-file`, unfused high-confidence depth pixels replayed as
free-space-only evidence). Best scene (Truck) +0.0033 under the pre-fix energy — below the
+0.003-beyond-noise default-flip gate, and the rays cannot reach occluded webbing by construction
(a ray that could reach the webbing region would have produced a fused point there), so the
webbing gate (§4) now supersedes the purpose this was meant to serve. **Kept opt-in**, not
re-benchmarked on the corrected metric (its prior A/B baseline also used different, older clouds
than the current default pipeline).

**Visibility-mass gate** (`--min-surface-evidence`). See §4 for the full mechanism and dose
table. **Removed from code.**

**No-decimation control** (`--min-point-distance 0`, inserting all 5.2M points instead of the
decimated set). Scores *worse*: 0.6764 raw vs the decimated 0.6986, at 4.4x the graph-cut cost.
`--min-point-distance` decimation is exonerated — it was never the source of any fidelity loss.

**The WSS admission ladder.** Never implemented — it was gated on a weighting rollout
(`--constant-weight 0` as a viable default) that Phase 2's ablation rejected outright. Void.

---

## 6. Durable engineering constraints and known limitations

- `orientation()` tests an **unnormalized determinant against a fixed absolute epsilon (1e-12)**;
  the determinant grows as edge-length cubed, so scenes whose median edge sits far from ~1 scene
  unit either silently collapse to COPLANAR at every ray-walk step (too small) or lose robustness
  to float noise near true degeneracies (too large). This is the entire reason
  `--canonical-rescale` exists.
- The rescale **must precede camera-cell location**, not just triangulation — at tiny scale every
  facet reads COPLANAR and every camera ray dies before it starts (verified: the 1e-6 control
  dropped all 72542/72542 rays, producing an empty mesh). The epsilon is not behaviorally free at
  either extreme either: the 1e6 control loses 7 vertices to float-noise near-degeneracies even
  though it does not collapse outright.
- CGAL's `finite_incident_edges_threadsafe` is **required** under OpenMP for the adaptive-σ fill —
  the plain (non-threadsafe) traversal writes shared TDS marker state and races against the
  ray-walk threads reading the same cells.
- The WSS `t==0` no-op (base t at ~1σ behind a point, enforcement targeting the ~4σ walk-end
  cell) is **structurally protective**, not a bug — see §5's `add`/`max` entries for what breaks
  when it is "fixed".
- **NaN passes the `maxCap` clamp** at `AddNode` (`std::min` returns its first argument when the
  comparison is false, so `MINF(NaN, maxCap)` is NaN, not `maxCap`) — the traced entry point is
  `normalized()` on a zero-area facet. Overflow to `+inf`, by contrast, *is* correctly clamped.
  IBFS's arc-count arithmetic is `int` and overflows around 79M points.
- Camera `D_out` is realized as hard `kInf` s-links on **every** frustum-visible hull-adjacent
  infinite cell, not just the sensor's own cell — on 360-degree or inward-facing captures this
  annihilates every `D_in` vote whose σ-shifted end cell exits the convex hull, which is why
  OpenMVS meshes stay open at the hull boundary regardless of evidence. Documented, intentional,
  **unaddressed** — no fix mandated.
- `PointCloud::Point` storage is `float`, which quantizes UTM-magnitude scenes to ~6cm — mesh-time
  rescale cannot repair geometry already destroyed by storage before triangulation runs. The open
  fix is **load-time centering** (§8), not a mesh-stage change.
- The ray-walk accounting counters (bad-ends, WSS `t==0`/saturation rates) are the **regression
  signal** for any future change to this energy — instrument first, judge by the counters, not
  just the final F1.
- `Mesh::SamplePoints` must use the **fixed-seed overload** in any benchmark; the legacy
  `random_device`-seeded default is noise-only and not reproducible.

---

## 7. Fixture appendix

Fixtures A and B below back live regression tests in `apps/Tests/TestsMVS.cpp` (search
`Fixture-A`/`Fixture-B`) and lock the cut topology of two hand-derived synthetic scenes — a
regression that drops or relocates the orphaned `D_in` vote, or flips a `mirror_facet` arc, will
fail these tests. Further hand-solvable fixture ideas (for the quality term, the free-space-support
triple test, and the WSS enforcement arithmetic) were designed during this effort but never wired
into the test suite; their specs live in git history if needed later.

### Common harness notes

Apply to both fixtures below:

* Build the `Scene` in memory: `scene.pointcloud.points/pointViews/pointWeights` +
  `scene.images` with valid `Camera` (`camera.C`, `camera.P`, `imageData.width/height`,
  `imageData.ID`), then call
  `scene.ReconstructMesh(distInsert=0.f, bUseFreeSpaceSupport=false, bUseOnlyROI=false,
   kSigma=<below>, kQual=0.f, ...)`.
  * `distInsert = 0` ⇒ the "insert all points" branch, no vertex merging.
  * `bUseFreeSpaceSupport = false` ⇒ the WSS block is skipped, so `t` is not multiplied.
  * `kQual = 0` ⇒ `q ≡ 0`, so arc capacity == `f` exactly.
  * Pass an explicit `ReconstructMeshParams` with `bAdaptiveSigma=false`,
    `bCanonicalRescale=false`, `maxEdgeScale=0` — both fixtures are hand-solved under the
    single global sigma and the ungated extraction, and the shipped defaults differ.
* `pointWeights` left empty ⇒ every `α_vis = 1`.
* **Do not assume CGAL cell indices.** Identify cells by `delaunay.locate(<interior probe
  point>)` and facets by `cell->index(vertexHandleOf(X))`; identify vertices by
  `delaunay.nearest_vertex(point_t(...))`. Both fixtures are Delaunay-unique, so the
  combinatorics are stable, but the numbering is not.
* Cameras must look at the scene with a wide FOV: `width = height = 640`,
  `K = [200 0 320; 0 200 240; 0 0 1]`, `R` as stated, `C` as stated. Any FOV containing the
  whole point set works — the frustum only gates infinite cells.
* Tolerance: `1e-6` absolute on `edge_cap_t` (float) comparisons.

### Fixture A — "bipyramid": 2 finite tetrahedra, 1 camera, 1 contributing point

**Points** (all 5 inserted; each has `pointViews = {0}`):

| name | coordinates |
|---|---|
| A | `( 1.0,  0.0,               0.0)` |
| B | `(-0.5,  0.8660254037844386, 0.0)` |
| C | `(-0.5, -0.8660254037844386, 0.0)` |
| D | `( 0.0,  0.0,               3.0)` |
| E | `( 0.0,  0.0,              -3.0)` |

`A,B,C` = equilateral triangle, circumradius 1, in the plane `z = 0`, centred on the z-axis.

**Camera 0**: `C = (0, 0, 1.5)`, looking along −z (any pose whose frustum contains the whole
bipyramid).

**Delaunay uniqueness (verified numerically)**: circumsphere(A,B,C,D) centre `(0,0,4/3)`,
r=`5/3`; `|E−centre| = 4.333 > r`. Circumsphere(A,B,C,E) centre `(0,0,−4/3)`, r=`5/3`;
`|D−centre| = 4.333 > r`. So the triangulation is exactly `T_up = {A,B,C,D}`,
`T_dn = {A,B,C,E}` sharing facet `ABC`, plus 6 infinite cells.

**σ**: finite edges are `AB,BC,CA` (len²=3, ×3) and `AD,BD,CD,AE,BE,CE` (len²=10, ×6); 9 values
⇒ median = 10. Pass `kSigma = 0.31622776601683794` (=1/√10) ⇒ σ = 1.0 exactly.

**Ray inventory**: rays to A, B, C, D each hit a vertex of the camera's own cell `T_up` on the
first `intersect` call ⇒ zero contribution, no `t`. The ray to E crosses facet `ABC` at its
centroid, enters `T_dn`, terminates at vertex E.

**Expected state (α=1, kQual=0)**:

| quantity | expected |
|---|---|
| `infoCells[T_up].f[T_up->index(D)]` | `0.9888910034617577` (= `1 − e^{−4.5}`, d=3) |
| every other `f[·]` | `0.0` |
| `infoCells[T_up].s` | `kInf` |
| `s` of every other cell (incl. all 6 infinite) | `0.0` |
| `Σ_cells t` | `1.0` |
| the single cell with `t != 0` | infinite, incident to vertex E |
| arc `T_up → T_dn` capacity | `0.9888910034617577` |
| arc `T_dn → T_up` capacity | `0.0` |

**What this proves**: the free→full capacity for a camera-side crossing sits on the arc
`T_up → T_dn` (along the ray), the reverse arc is exactly zero; the weight is
`α(1−e^{−d²/2σ²})` with d measured from **P = E** (d=3), not from the camera (d=1.5, which
would give a visibly different 0.6753475); the finite-camera-cell branch stamps exactly one
cell.

### Fixture B — "tetra + interior point": `mirror_facet` and the σ-shifted `D_in`

**Points** (all 5 inserted). Let `s3 = 1.7320508075688772`.

| name | coordinates | `pointViews` |
|---|---|---|
| P  | `( 0.0, 0.0, 0.0)` | `{0}` |
| V0 | `( 1.5, 0.5, 6.0)` | `{1}` |
| V1 | `( 4.0, 0.0, -2.0)` | `{0}` |
| V2 | `(-2.0,  2*s3, -2.0)` | `{0}` |
| V3 | `(-2.0, -2*s3, -2.0)` | `{0}` |

**Camera 0**: `C = (0, 0, -10)`, looking +z, wide FOV. **Camera 1**: `C = (1.5, 0.5, 26)`,
looking −z, wide FOV (exists only so V0 has a view whose ray provably contributes nothing).

**Triangulation**: P is strictly inside tetra `V0V1V2V3` ⇒ a unique star-of-P triangulation:
`Ca={P,V1,V2,V3}`, `Cb={P,V0,V2,V3}`, `Cc={P,V0,V1,V3}`, `Cd={P,V0,V1,V2}`, plus 4 infinite
cells over the hull facets.

**σ**: 10 finite edge lengths² sorted, median = 48 ⇒ `kSigma = 0.5773502691896258` (=1/√3) ⇒
σ = 4.0 exactly.

**Ray inventory**: V1/V2/V3 from camera 0 and V0 from camera 1 all hit their own vertex on the
first `intersect` call ⇒ no contribution. P from camera 0 is the only contributing ray: walk 1
crosses hull facet V1V2V3 at its centroid (d₁=2.0), enters Ca, terminates at P; walk 2's end
point `P + 4·(0,0,1) = (0,0,4)` is outside the hull, so the +z ray from P enters `Cb` and exits
through facet V0V2V3 at a strictly-interior point (d₂ = 18/7 = 2.5714285714285716).

**Expected state (α=1, kQual=0)**:

| quantity | expected |
|---|---|
| `infoCells[infCell(V1V2V3)].f[·]` for facet V1V2V3 | `0.11750309741540454` (=`1−e^{−0.125}`, d=2) |
| `infoCells[Cb].f[Cb->index(vP)]` (facet V0V2V3) | `0.18668163487015432` (=`1−e^{−(18/7)²/32}`, d=18/7) |
| `infoCells[Ca].f[Ca->index(vP)]` (mirror of V1V2V3) | `0.0` |
| `infoCells[infCell(V0V2V3)].f[·]` (mirror) | `0.0` |
| every other `f[·]` | `0.0` |
| `s` of Ca,Cb,Cc,Cd (all finite) | `0.0` |
| `s` of all 4 infinite cells | `kInf` |
| `Σ_cells t` | `1.0` |
| the single cell with `t != 0` | infinite, contains `(0,0,4)` |

**What this proves**: the behind-the-point crossing is deposited through `mirror_facet` on the
arc away from the camera (`Cb → infCell(V0V2V3)`), reverse arc exactly zero — if `mirror_facet`
were dropped, `0.18668163` would land on the wrong cell and the test fails; both distances (2
and 18/7) are measured from P, not the camera; `t` lands undecayed on the cell at `P + σ·dir`
and nowhere else; all infinite cells are hard-stamped while the 4 finite cells are untouched.

---

## 8. Open items

- **Load-time centering** for float-quantized large-coordinate point clouds (~6cm quantization at
  UTM magnitude, §6) — an import-side fix touching all pipelines (Interface importers /
  CreateStructure), not a mesh-stage change. Not started.
- **Depth-maps as direct mesh input**, bypassing or supplementing the fused cloud — tracked in
  `docs/design/DepthmapMeshingPlan.md`.
- **Fusion re-baseline and family-B benefits** — tracked in
  `docs/design/FusionImprovementPlan.md`.
- **Acceptance gates for future work on this energy**: mean paired mesh-F1 ≥ +0.003 beyond the
  0.0006 noise floor; no scene regressing more than 0.003 F1; ≥5% median improvement for
  exact-result speed changes. Judge every change on the **raw+gated surface** (§2's protocol),
  never on the cleaned mesh — that is exactly the measurement mistake this whole effort had to
  recover from (§3).
- Other standing guardrails from the executed plan, still binding: confidence enters the
  visibility data term only — never `kQual`/circumsphere quality, camera hard constraints, or a
  second generic per-cell unary; no generic k-NN/smoothing prefilters by default (they erase thin
  structure); don't replace IBFS without profiling first (§5's solver-swap entry is why); leave
  `RefineMesh` untouched; face count is not a completeness metric — score by F1 on ground truth.
