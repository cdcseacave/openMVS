# Delaunay Mesh Reconstruction — Integrated Improvement Plan

Target: `Scene::ReconstructMesh` (`libs/MVS/SceneReconstruct.cpp`) — speed and accuracy/completeness,
exploiting the recalibrated depth-map confidence (#1292, `docs/design/DepthMapConfidence.md`).

This v3 merges two independent audits plus the maintainer's review decisions (§ 2):
- the v1 plan produced in this workspace (literature survey → code audit → task list),
- the colleague handoff `bench/delaunay_mesh_implementation_handoff.md` (source audit → partial
  edits → phased roadmap with acceptance gates).

Where the two audits differed, § 1 records the resolution. Per-slice experimental results
(before/after tables, accepted/rejected conclusions) go to a permanent record,
`docs/design/DelaunayMeshReconstruction.md`, created with the first landed slice.

---

## 0. Combined verified ground truth

Facts both audits verified in code, plus each audit's unique finds. Do not re-derive.

| Fact | Where |
|---|---|
| **The app discards confidence by default**: `--constant-weight` defaults `true` → `pointWeights.Release()` before reconstruction. The #1292 signal is serialized and wired to α_vis, but a default run never uses it | `ReconstructMesh.cpp:130`, `:448` *(handoff's key find — corrects v1's "already flows by default")* |
| **WSS is off by default at the CLI**: `--free-space-support` defaults `false` (library default `true`) | `ReconstructMesh.cpp:131` |
| CLI/library default mismatches: `--thickness-factor 1.0` vs library `kSigma=2.0`; `--min-point-distance 1.5` vs library `2` | `ReconstructMesh.cpp:128,132` vs `Scene.h:149` |
| α_vis = per-view weight accumulated in `InsertViews`; with weights enabled it is the [0,1] recalibrated posterior (ROC 0.844→0.926); missing weights fall back to unit votes (legacy `.mvs` safe) | `SceneReconstruct.cpp:982`, `:241` |
| Pre-#1292 `pointWeights` held `Conf2Weight = 1/((1−conf)·depth²)` — scene-unit-dependent; the constants' operating point silently varied per scene. Weights are dimensionless (~0.3–0.7 mean) for the first time | *(v1 find)* `git show f61bffef^` |
| WSS is the **corrected ISRN-2014** classifier (epsRel/epsAbs/kOutl triple), not CVPR-2011. Do not reimplement it | `SceneReconstruct.cpp:1106` |
| WSS enforcement `t *= epsAbs` is a **silent no-op when `t==0`** — exactly the weakly-supported cell it targets — and its units are α² (quadratic in confidence scale) | *(v1 find)* `SceneReconstruct.cpp:1111` |
| Both weighting loops advance a shared CGAL iterator inside `#pragma omp critical` — serialization + poor load balance | `SceneReconstruct.cpp:959`, `:1052` |
| Ray walks can fail silently: the "Bad end" fallback drops the remainder of a ray with no accounting | `SceneReconstruct.cpp:649` |
| `nItersFixNonManifold` is accepted but ignored; one `FixNonManifold()` call with defaults | `SceneReconstruct.cpp:784`, `:1186` |
| `Mesh::SamplePoints` seeds `mt19937` from `random_device` → mesh evaluation has sampling noise until a fixed seed is added | *(handoff find)* `Mesh.cpp:2930` |
| A per-pixel depth+confidence → point-cloud conversion pattern already exists (single-map scene import) — reference for the depth-maps-direct track | `Scene.cpp:406-428` |
| Uncommitted in-tree partial patch: ROI index compaction + empty-ROI error + `viewsInfo`/cache gating on `bUseFreeSpaceSupport`, plus a TestsMVS ROI preflight. Build of `Tests` and the `MVSPipelineTest` run were **not** completed | working tree; handoff "Phase 0" |
| Repo instructions bind edits/builds/tests: `.github/instructions/{cpp-style-openmvs, cmake-build-workflow, testing-workflow, review-focus}.instructions.md` | verified present |

## 1. Reconciliation — where the two audits differed

| Topic | v1 plan | Handoff | Integrated resolution |
|---|---|---|---|
| First quality item | WSS constants retune | Confidence ablation | **Ablation first.** v1 assumed confidence already flowed at default; `--constant-weight=true` disproves that. Turning on an existing, validated signal is the cheapest possible win, and WSS calibration must run under whichever weight regime the ablation selects |
| WSS threshold scale | Scale-free variant normalizing β/γ by vote mass | "Calibrate the consumer, don't renormalize" | Per § 2 note 1: **the confidence itself is never rescaled** — it enters as-is. Threshold recalibration is primary; quantile-derived thresholds are a secondary arm (Phase 4.1) |
| WSS enforcement semantics | A/B `t *= epsAbs` vs `t = max(t, kw·epsAbs)` vs `+=` | Not identified | Kept — folded into Phase 4.1 (the `t==0` no-op share is measured in Phase 1 first) |
| Per-point adaptive σ | Task T5 | Absent | Kept as Phase 4.3; the depth-maps track (Phase 5) can supply a principled per-point scale |
| Confidence-tiered insertion (who claims the Delaunay vertex) | Task T3 | Absent | Kept as step 0 of the Phase 4.4 admission ladder — reorders insertion, drops nothing |
| Partial-patch validation | Not covered | Phase 0 | Adopted — the edits are in this tree and unvalidated |
| Benchmark determinism | Noted noise, no fix | Seeded `SamplePoints` + separate `run_mesh.py` | Adopted |
| Paired confidence experiment | `--constant-weight` A/B only | Raw-vs-adjusted pairing via preserved dmaps + same-cloud weight replacement | Adopted — mirrors the §8.2 pairing lesson |
| Acceptance gates | Qualitative | Numeric | Adopted verbatim (§ 8) |
| Preemptive manifold repair / octree / DGNN | Deferred | Deferred with hard constraints | Agreement; handoff's constraints adopted |

Shared guardrails both audits arrived at independently (binding): confidence goes in the **data
term only** — never into `kQual`/circumsphere quality, camera hard constraints, or a second
generic per-cell unary; no generic k-NN/smoothing prefilters by default (thin structures); no
threads×cells accumulator; don't replace IBFS before profiling; don't touch `RefineMesh`;
face count is not a completeness metric.

## 2. Maintainer review decisions (2026-08-18) — binding directives

1. **The weight is the plain [0,1] confidence, or nothing.** No normalization (it is already
   [0,1]) and no combination with depth or any other scene-dependent value — reintroducing a
   unit-dependent factor repeats the §8.6 mistake and is rejected by construction. The open
   question Phase 2 must answer is only: *does confidence alone beat constant votes?* The one
   remaining implicit combination — `InsertViews` **summing** same-view weights of merged points
   (vote mass ∝ local density) — gets a low-priority arm (sum vs max vs mean) in Phase 2.
2. **Progress is measured on T&T ground truth at two stages**: the F1 of the frozen dense cloud
   (input, existing `run_cell.py` protocol) and the F1 of every reconstructed mesh (sampled,
   seeded). Every benchmark row reports the pair; the mesh−cloud delta shows whether the mesh
   stage adds or destroys fidelity. Formalized in Phase 1.
3. **Investigate depth-maps as direct input** to mesh reconstruction — bypassing or supplementing
   the fused cloud — for speed and/or accuracy/completeness. New Phase 5 track.
4. **Verify the correctness of the existing implementation early.** Phase 0 is widened from
   "validate the partial patch" to a soundness audit of the whole reconstruction path (0.B).

---

## Phase 0 — Correctness and soundness of what exists  *(3–5 days)*

**0.A Validate the in-tree partial patch** *(<1 day)*
1. `git status`/`git diff` to confirm live state; build `Tests`; run
   `ctest -C RelWithDebInfo -R MVSPipelineTest --output-on-failure`.
2. WSS-off equivalence spot-check (the patch must be exact-result there): same vertex/cell
   counts, same max-flow, same raw face count; reduced peak RSS is the expected win.
3. Keep the ROI preflight only if `MVSPipelineTest` stays practical; never weaken the existing
   face-count envelope to accommodate it. Commit as slice 1 when green.

**0.B Runtime soundness of the existing path** *(1–2 days, mostly measurement)*
- **Solver cross-check**: both max-flow backends exist behind `DELAUNAY_MAXFLOW_IBFS`. Build
  both, run identical graphs on 1–2 small scenes, compare flow values and cut labels. Validates
  graph construction and the IBFS wrapper in one shot (and doubles as the harness for the
  Phase 3.3 solver A/B).
- **Threading check**: single-threaded run (reference) vs OpenMP run — bounds the atomic-order
  noise *and* catches any race; the difference must be explainable by float accumulation order.
- **Failed-ray accounting**: count rays hitting the silent "Bad end" fallback (`:649`) and
  traversals aborted mid-walk, per scene and vs coordinate magnitude (see 0.C item 8).
- **WSS micro-trace**: on a tiny synthetic scene, log β/γ/epsAbs/epsRel per observation and
  verify the classifier fires exactly where the geometry says it should (this seeds the Phase
  4.1 fixture).
- Unit tests locked in now: empty `pointWeights` → unit votes; weights survive serialization and
  insertion-time aggregation; ROI edge cases (empty; <4 non-coplanar points); seeded sampling
  determinism (once Phase 1 adds the seed).

**0.C Math & formula verification against the papers** *(1–2 days; review + hand-solvable
micro-fixtures)*
Each energy term is re-derived from its source paper and checked against the code; derivations
and verdicts go in the results record. Where a term is checkable, build a 2–3 cell
triangulation whose capacities are computable by hand and assert them.
1. **Soft-visibility weight** `w = α·(1−exp(−d²/2σ²))` (`:997`, `:1026`): confirm `inter.dist`
   measures distance from the *point* (not the camera) in both walk directions, so the tube
   behaves per Labatut-2009 — crossing near P is cheap, far from P costs full α.
2. **Directed facet capacities**: camera→point crossings must charge the free→full transition
   in walk direction; the behind-point walk must charge the reverse via `mirror_facet`
   (`:1025`). Cross-check against the edge assembly
   `AddEdge(ciID, cjID, ciInfo.f[i]+q, cjInfo.f[j]+q)` (`:1141`) on a hand-solvable fixture.
3. **D_in placement**: `t += α` on the cell located at P + σ·dir (`:1013`) — verify against the
   paper's "cell just behind P", including when σ spans several cells.
4. **Camera D_out** realized as hard `kInf` s-links on every camera-cell facet (`:932`) instead
   of per-ray α on T₁ — document the (presumably intentional) deviation and its effect.
5. **Quality term** `computePlaneSphereAngle` (`:738`): verify the facet-normal /
   circumcenter-cotangent construction reproduces Labatut's `w_f = 1 − min(cos β, cos γ)`,
   including the `0.5` degenerate-case returns and the symmetric addition of `q` (`:1140`).
6. **Free-space support**: `freeSpaceSupport()` (`:697`) sums the 4 *incoming* mirror-facet
   capacities — confirm this matches the ISRN-2014 emptiness measure; then verify that
   β = max over the front walk and γ = (min+max)/2 over the back walk (`:1082-1101`) is a
   faithful discretization of the paper's ⟨−3σ, +4σ⟩ jump window, and that the triple test
   (`:1106`) maps to K_rel ∧ K_abs ∧ ¬K_outl with the inequality directions right.
7. **WSS enforcement** `t *= epsAbs` (`:1111`): document what the paper prescribes (a t-edge
   *set* from the jump) vs the multiplicative form — the audit records the discrepancy; the
   decision is the Phase 4.1 semantics arms.
8. **`orientation()` uses an absolute epsilon (`1e-12`) on an unnormalized determinant**
   (`:367`) — scale-dependent: georeferenced (large-coordinate) or miniature scenes shift the
   effective tolerance and are the prime suspect for the "Bad end" failures. Audit: correlate
   0.B's failure rate with coordinate magnitude. Candidate fix (Phase 3): normalize the cloud
   into a canonical box before triangulation and invert at extraction; the disabled
   relative-epsilon variant (`:370-377`) is the slower fallback.
9. **σ estimation** (`:945-949`): median over all finite edges — confirm robustness to
   convex-hull and sliver edges.
10. **Capacity hygiene**: all terms non-negative (asserted); `maxCap` clamps only t (`:1129`) —
   confirm `t *= epsAbs` cannot overflow float before the clamp; submodularity holds for every
   term as implemented.

**0.D Literature refresh** *(performed 2026-08-18; re-run at each phase boundary)*
Searched 2022–2026 for successors to the classical pipeline. Verdict: the plan's
classical-first stance stands; nothing found supersedes visibility-based Delaunay graph-cut for
MVS point-cloud input.
- **No new classical visibility/WSS energy work since 2021.** Vis2Mesh and DGNN remain the last
  word on learned/adaptive unaries; the TPAMI-2025 survey confirms classical Delaunay cuts stay
  the most robust out-of-domain.
- **Actionable (solver)**: the TPAMI-2022 min-cut/max-flow review (arXiv:2202.00418) benchmarks
  Hochbaum pseudoflow and **Excesses IBFS (EIBFS)** as fastest on unstructured graphs — EIBFS
  is a natural A/B candidate against the bundled IBFS in Phase 3.3 (reference implementations
  exist; license needs the same scrutiny as IBFS's).
- **Known art (speed)**: a full-GPU port of this exact pipeline exists — CAGD 2021
  "A visibility-based surface reconstruction method on the GPU" (three-level Delaunay index,
  GPU ray traversal, dedicated GPU graph cut). Reference for an eventual Phase 6 GPU track;
  large engineering, not an early move.
- **Adjacent track, watch only** (different input: per-scene differentiable optimization, not
  point clouds + visibility): Gaussian-splatting surface extraction — GOF (ToG 2024) extracts
  meshes via Delaunay over Gaussian primitives + Marching Tetrahedra; SOF (2025) parallelizes
  that tetrahedra pass ~10×; MILo (2025) mesh-in-the-loop differentiable extraction; DMesh
  (2024) differentiable Delaunay-style meshes; MPF (ICML 2026) thin-structure implicits from
  unoriented points. None consume MVS clouds with visibility, so none replace this pipeline —
  but the parallel-tetrahedra extraction machinery is worth a look if Phase 6 ever targets GPU.

## Phase 1 — Two-stage T&T benchmark + instrumentation  *(1–2 days)*

`bench/run_cell.py` stays the dense-stage bench. New `bench/run_mesh.py`:

- Consumes a **frozen** `scene_dense.mvs` per scene; all mesh variants share identical geometry,
  views, and stored confidence.
- **Two-stage scoring (per § 2 note 2)**: each row records (a) the input cloud's F1 (computed
  once per frozen cloud with the existing T&T evaluator) and (b) the mesh F1 — mesh →
  area-uniform samples via `Mesh::SamplePoints` with a new optional fixed seed
  (backward-compatible default stays random) → same evaluator at scene τ. Track both across the
  whole project; the mesh−cloud delta is the mesh stage's own contribution.
- Scenes: Truck / Barn / Ignatius / Meetingroom at R1/V8 for screening; promote finalists to
  R0/V16. Optional stretch for Phase 4.1: one or two ETH3D textureless levels — T&T alone
  under-tests weak surfaces.
- Metrics per run: P/R/F at τ (both stages); raw+cleaned vertex/face counts; surface area;
  component count and largest-component ratio; boundary and non-manifold counts; per-stage wall
  times (triangulation / weighting / WSS / graph build / solve / extraction / repair); peak
  working set; input vs inserted points, rays, facet crossings, failed walks.
- Noise floor first: two identical baseline runs; every later delta is judged against it.

Instrumentation (aggregate-only, `DEBUG_EXTRA`/`DEBUG_ULTIMATE`, never per-ray): weight
availability + confidence distribution + summed weight per vertex before `pointcloud.Release()`;
views-per-vertex distribution; ray success/failure counts; WSS β/γ/epsAbs/epsRel percentiles,
trigger rate, reinforcement magnitudes, and **the share of triggers landing on `t==0` cells**.

## Phase 2 — Confidence ablation + rollout decision  *(1 day + runs)*

The cheapest large win available: the signal exists, is wired, and is discarded by a CLI default.
Per § 2 note 1 the hypothesis is minimal: **plain confidence alone vs constant votes** — no
shaping, no normalization, no scene-dependent factors.

- 2×2 matrix per frozen cloud: `--constant-weight {1,0}` × `--free-space-support {0,1}`.
- Raw-NCC vs adjusted provenance, properly paired (CUDA PatchMatch is nondeterministic — never
  compare independent densifications): preserve raw dmaps, fuse once with raw and once with
  adjusted confidence on the same maps. If fusion admission diverges, split into (a) same-cloud
  experiment — overwrite only stored `pointWeights`; (b) full-pipeline experiment.
- Optional low-priority arm (§ 2 note 1): same-view merge aggregation sum (current) vs max vs
  mean — only if the views-per-vertex stats show heavy merging.
- Decision rule (gates § 8): if both raw and adjusted weighting beat constant votes, flip the app
  default to `--constant-weight=false` (missing weights fall back to 1 — legacy-safe). If only
  adjusted passes, weighting stays opt-in until provenance is representable; do not infer
  provenance from the numeric distribution.

## Phase 3 — Exact-result speed + API cleanup  *(1–2 days)*

- **3.1** Replace the `omp critical` iterator handoff in both loops: pre-collect finite-vertex
  handles (skip empty-view vertices), indexed `parallel for schedule(dynamic)`, one reserved
  facet buffer per thread; atomics stay. Retain if: identical ray counts, capacities within
  float tolerance, same cut topology, ≥5 % median weighting speedup over three runs, ≤10 % peak
  memory growth.
- **3.2** Resolve `nItersFixNonManifold`: iterate until clean or cap (log per-pass counts), or
  delete the parameter if one pass is provably exhaustive. Check topology after ROI cropping too.
- **3.3** Profile-selected only (from Phase 1 timers): `view_info_t` arena; IBFS arc
  preallocation; sampled edge-length σ estimation; **solver A/B: EIBFS vs bundled IBFS** on
  identical graphs (reuses the 0.B cross-check harness; license check first — see 0.D).
  Rejected up front: threads×cells accumulators, unbenchmarked solver swaps.
- **3.4** Contingent on 0.C item 8: **canonical-box coordinate normalization** before
  triangulation (invert at extraction) to make the `orientation()` epsilon scale-independent —
  a correctness fix for georeferenced/miniature scenes, gated on the audit confirming the
  failure correlation.

## Phase 4 — Energy/quality improvements, each gate-checked

**4.1 WSS calibration under calibrated confidence + enforcement semantics**  *(2–4 days)*
`kAbs=1000`/`kOutl=400` were tuned around unit votes; pre-#1292 the effective scale was
scene-unit-dependent, so calibration is meaningful and transferable for the first time. The
confidence values themselves enter unchanged (§ 2 note 1) — only the consumer is calibrated.
Sweep arms on the Phase 1 bench (mesh-only reruns are cheap):
1. current thresholds (anchor);
2. thresholds recalibrated directly on the bench (primary);
3. scene-relative thresholds from support quantiles (secondary);
4. enforcement semantics: `t *= epsAbs` (current; α² units, no-op at `t==0`) vs
   `t = max(t, kw·epsAbs)` (paper-faithful jump) vs `t += kw·epsAbs`.
`kRel` stays untouched initially (dimensionless). Primary metric: weak-surface recall/F on the
synthetic fixture + ETH3D stretch scenes, precision guarded on T&T. Build the **synthetic
fixture** first (deterministic, region-labeled, grown from the 0.B micro-trace): sparse plane
bounded by passing rays, thin strip/rod, coherent ghost surface from mutually-supporting
outliers, random floaters, frontal + grazing cameras; score region-specific
coverage/distance/false-volume/topology.

**4.2 Grazing-incidence down-weighting (Vis2Mesh)**  *(2–3 days, optional flag)*
`w *= clamp(|cos(ray, facet_normal)|, floor, 1)` at the two crossed-facet accumulation sites;
the facet plane is already computed per crossing, so cost is near-zero. One floor + one
exponent, with a neutral setting reproducing current behavior. Validate on the grazing fixture
region + facade/oblique scenes; T&T as the neutrality guard.

**4.3 Per-point adaptive σ**  *(3–5 days, staged)*
Replace the single global σ where it acts as per-point uncertainty: per-vertex scale s_v =
median incident-edge length (parallel O(V)), σ_v = kSigma·s_v clamped to [0.25, 4]× global —
or, if Phase 5 lands its scale export, the principled per-point footprint (depth/f) from the
depth maps. Use the target vertex's σ_v in the soft-visibility exponent, the end-cell offset,
and the WSS window lengths. Stage 2 (separate A/B): shrink σ_v for high-confidence vertices.
Built-in guard: uniform-scale scenes give σ_v ≈ σ, so T&T deltas ≈ 0; wins expected on
thin-structure and mixed-scale scenes. While here, resolve the `--thickness-factor` (1.0) vs
library `kSigma` (2.0) default mismatch deliberately.

**4.4 Confidence-aware point admission ladder**  *(3–5 days, one rung at a time)*
Only after Phase 2 accepts confidence weighting:
0. **Confidence-tiered insertion**: 2–3 confidence tiers, spatial-sorted within each, high tier
   first — high-confidence points claim the Delaunay vertices (= mesh vertex positions);
   nothing is dropped.
1. Drop points with negligible effective support.
2. Confidence-modulated `distInsert` for low-confidence redundant points.
3. Local-scale fusion only if 1–2 are insufficient.
Data-gated extra: top-K views per vertex (K ∈ {8,12,16,∞}) — only if Phase 1 shows a fat
views-per-vertex tail. Every rung reports counts, rays, crossings, runtime, memory, both F1
stages, and fixture metrics. No generic k-NN removal or smoothing by default.

## Phase 5 — Depth-maps as direct input (investigation, § 2 note 3)

Today the mesh stage sees only what fusion admitted; every depth pixel fusion rejected (few-view
clusters, `nMinViewsFuse`, admission order) loses both its **point** and its **ray** — and rays
are evidence even when points are unreliable. Staged, cheapest-question-first:

- **5.0 Measure what fusion drops** *(instrumentation only)*: per scene, depth pixels vs fused
  points; confidence distribution of dropped pixels; where they concentrate (weak-texture
  regions?). If the dropped mass is small or junk, the track stops here.
- **5.1 Experiment A — carve-only rays**: unfused **high-confidence** depth pixels contribute
  free-space carving (and s_jump evidence for WSS) without inserting vertices — graph topology
  unchanged, ray count bounded by a confidence gate + subsampling. Targets completeness on
  weakly-supported surfaces; costs traversal time only. Natural companion to Phase 4.1.
- **5.2 Experiment B — direct dmap-sourced meshing**: bypass the fused cloud; insert
  confidence-gated, subsampled per-view depth pixels with per-pixel confidence as weight, and
  let `distInsert` merging do the multi-view consolidation (this is closer to the original
  CMPMVS input model; the per-pixel conversion pattern exists at `Scene.cpp:406-428`). Compare
  **end-to-end** wall time (fusion skipped) and both F1 stages against the standard path.
  Honest expectation: speed could win (one less pass, no full-cloud peak) but the mesh stage
  grows; quality could win on completeness (nothing pre-filtered) but the graph must absorb
  more outliers — exactly what the energy is designed for, now with confidence gating.
- **5.3 Scale export**: persist (or read at mesh time) the per-point pixel footprint
  (depth/f) — the principled σ_v input for Phase 4.3.

Go/no-go after 5.0+5.1; 5.2 only if A shows the dropped evidence matters.

## Phase 6 — Deferred tracks

- **Preemptive manifold repair on the DT** (Romanoni 2020): only if Phase 1's residual
  non-manifold counts justify it.
- **Mostegel-style partitioning** *(3–5 weeks)*: only for a real memory-bound use case. Hard
  requirements: overlapping 8-leaf corner neighborhoods, consensus facets, centricity-scored
  patches, secondary boundary-minimizing cut; validate by equivalence against the global method
  first. Naive independent voxel cuts rejected up front.
- **DGNN/ONNX learned unary** *(2–4 weeks)*: only after classical improvements plateau; ONNX
  behind CMake+CLI flags, exact classical fallback, Labatut binary + camera constraints
  retained, no required libtorch.

---

## 8. Acceptance gates *(adopted from the handoff)*

**Exact-result changes**: focused tests pass; max-flow/cut identical or float-order differences
proven immaterial; mesh metrics within noise; ≥5 % median stage/wall improvement; ≤10 % peak
memory growth. (Pure correctness fixes need no speed threshold.)

**Energy/filtering changes**: mean paired mesh-F ≥ +0.003 beyond measured noise; no scene
regresses > 0.003 F; P/R each within 1 pp unless the complementary gain nets a clear F win;
fixture metrics improve in the intended region; no topology regression; runtime/memory cost
documented and proportionate.

**Scalable mode**: agreement with global output on feasible scenes; explicit
seam/crack/duplicate checks; bounded peak memory with scene size; no hidden O(all points/cells)
global structure.

## 9. Execution order and commit slices

P0.A (validate patch) → P0.B+0.C (runtime soundness + math/formula audit; 0.D literature
refresh already done, re-run at phase boundaries) → P1 (two-stage bench + instrumentation +
noise floor) → P2 (confidence ablation → default decision) → P3 (exact-result speed, manifold
API, solver A/B, predicate-scale fix if confirmed) → P5.0/5.1 (fusion-drop measurement +
carve-only rays; pairs with 4.1) → P4.1 (WSS calibration + semantics + fixture) → P4.2
(grazing) → P4.3 (per-point σ, fed by 5.3) → P4.4 (admission ladder) → P5.2 (direct dmap
meshing, if 5.0/5.1 justify) → P6 (only if justified).

One reversible commit per slice; every experimental slice adds its before/after table and an
accepted/rejected verdict to `docs/design/DelaunayMeshReconstruction.md`. Builds and tests
follow `.github/instructions/`; performance claims use ≥3 runs, medians, identical inputs.

Phases 0–3 are ≈ one person-week and contain the two highest-certainty wins (turning on the
existing confidence signal; removing the weighting-loop serialization), now preceded by the
correctness audit the rest of the work stands on.
