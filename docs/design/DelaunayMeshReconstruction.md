# Delaunay Mesh Reconstruction — Results Record

Permanent record of per-slice experimental results and audit verdicts for `Scene::ReconstructMesh`
(`libs/MVS/SceneReconstruct.cpp`). Complements `docs/design/DelaunayMeshImprovementPlan.md`, which
owns the phased task list, acceptance gates, and execution order — this document owns the evidence:
what was checked, what was found, and the numbers later unit tests must reproduce. Every experimental
slice adds its own before/after table and accepted/rejected verdict here, appended in execution order.

Date started: 2026-08-19.

---

## Phase 0.C — math & formula audit (2026-08-19)

Ten energy-term / numerical-robustness items from plan §0.C, each re-derived from its source paper
(Labatut/Pons/Keriven CGF-2009, via Labatut's thesis tel-00844020 §2.3.3/§3.1.3, cross-checked against
Vu et al. PAMI-2012, and Jancosek & Pajdla ISRN-2014 art. 798595) and checked against the code. Full
derivations, verified-by-execution numeric checks, and complete line-by-line reasoning live in the three
source audit reports (`audit_terms_1to4.md`, `audit_terms_5to7.md`, `audit_terms_8to10.md`); this section
distills each to its paper prescription, what the code does, the verdict, and the consequence for later
phases.

**Line-number caveat.** All `:NNN` references below are the audits' *snapshot* line numbers
(`SceneReconstruct_snapshot.cpp`, stated by the auditors to be identical to
`libs/MVS/SceneReconstruct.cpp` at audit time). The working tree carries an uncommitted partial patch
(plan §0.A) at the time of writing, so live line numbers may already have drifted — treat these as
pointers into the audited snapshot, not a live-file guarantee. All refs are to
`libs/MVS/SceneReconstruct.cpp` unless stated otherwise.

### Summary

| # | Item | Verdict | Severity | Consequence |
|---|---|---|---|---|
| 1 | Soft-visibility weight `w=α(1−e^{−d²/2σ²})`, `d` from **P** in both walks | CONFIRMED-CORRECT | none (2 wrong comments) | locks Fixture A/B; comment fix only |
| 2 | Directed facet capacities + `mirror_facet` vs `AddEdge` arc convention | CONFIRMED-CORRECT | none | locks Fixture A/B regression tests |
| 3 | `D_in` on cell at `P+σ·dir` | CONFIRMED-CORRECT core | cosmetic-to-behavioral | 2 low-severity deviations feed Phase 0.B accounting; 3.3 interaction charged to item 4 |
| 4 | Camera `D_out` as `kInf` hard s-links on a **set** of cells | DISCREPANCY (intentional) | behavioral | 360°/inward captures annihilate hull-exiting `D_in` votes; documented, no fix mandated |
| 5 | Quality term `computePlaneSphereAngle` (β-skeleton) | CONFIRMED-CORRECT | cosmetic | signed-cosine range must not change without a benchmark |
| 6 | `freeSpaceSupport` β/γ/triple-test | DISCREPANCY (support measure); triple test CONFIRMED-CORRECT | behavioral, moderate | rewrites Phase 4.1's "anchor" premise; CLI halves the jump window |
| 7 | WSS enforcement `t *= epsAbs` | DISCREPANCY | behavioral, severe | rewrites Phase 4.1 semantics arms (now 4, not 3) |
| 8 | `orientation()` absolute epsilon + ray-walk failure paths | DISCREPANCY | MEDIUM (HIGH sub-case) | redirects Phase 3.4's fix target from M to L / float-precision; feeds Phase 0.B counters |
| 9 | σ estimation (median edge length) | CONFIRMED-CORRECT | none | enables Phase 3.3 subsampling without a fidelity caveat |
| 10 | Capacity hygiene (non-negativity, `maxCap`, submodularity, container sizing) | mixed: submodularity/non-negativity CONFIRMED-CORRECT; overflow/NaN + container DISCREPANCY | MEDIUM-HIGH (WSS-on) | corrects plan's overflow framing; NaN, not overflow, is the corrupting path |

### Item 1 — Soft-visibility weight `w = α·(1−e^{−d²/2σ²})`

**Paper** (thesis §3.1.3, verbatim quote audited): the final tetrahedron on a line of sight is shifted
σ past the sample point P; every oriented facet crossed by the extended chain `Q→P→P+σ·dir` gets weight
`α_vis(1−e^{−d²/2σ²})` where **d is measured from P**, symmetric in front of and behind P. `D_in` itself
is undecayed.

**Code:** the ray in `intersection_t` (`:988-990`) originates at **P**, direction `(P−C)/|P−C|`; `dist`
is filled via `TRay::IntersectsDist` (`:549, :555-560`), which returns the signed Euclidean distance
from the ray's own origin — i.e. from P, not the camera. Both weight sites (`:997` camera→point walk,
`:1026` behind-point walk) use `SQUARE(inter.dist)`, discarding sign; the behind-point walk reuses the
same `Ray3` (cannot be re-seeded), only flipping the traversal predicate, so its distances also run from
P (≈σ down to 0). `alpha_vis = view.weight` (`:982`), the per-view accumulated confidence.

**Verdict — CONFIRMED-CORRECT.** The tube behaves exactly as prescribed: crossings near P are free,
crossings far from P in either direction cost full α, σ=0 degenerates to hard visibility. Two cosmetic
nits: the comment at `:444` ("distance from starting point (camera)") is wrong — origin is P, not the
camera — and is almost certainly the origin of the suspicion that prompted this audit; the parenthetical
at `:446` is backwards.

**Consequence:** no code change. Fixture A/B (appendix) pin `w` to 7 decimals for known `d,σ`; a future
"fix" of the wrong comments should not touch the arithmetic.

### Item 2 — Directed facet capacities and `mirror_facet`

**Paper:** `V_align(l_Ci,l_Cj)=α_vis·1[l_Ci=0∧l_Cj=1]`; since a cut pays `u→v` iff `u∈𝕊(free),v∈𝕋(full)`,
the capacity must sit on the arc pointing along the ray, away from the camera, for every facet of the
extended chain.

**Code:** camera→point walk (`:992-1003`) always has `inter.facet.first` = the cell being exited (nearer
the camera); `w` is added to `f[inter.facet.second]` of that cell (`:998-1002`), directly on the correct
arc. The behind-point walk (`:1023-1032`) runs backwards, so `inter.facet.first` is the farther cell —
the code first takes `delaunay.mirror_facet(inter.facet)` (`:1025`) to get the nearer cell/index pair,
then deposits there, restoring the correct along-the-ray orientation. Assembly (`:1132-1141`)
`AddEdge(ciID,cjID,ciInfo.f[i]+q,cjInfo.f[j]+q)` confirms `cell_info_t.f[i]` is the capacity of the
directed arc `ci→neighbor(i)` in both the IBFS and Boost backends (`:87-95`, `:140-166`).

**Verdict — CONFIRMED-CORRECT.** Both walk directions place capacity on the correct arc; `mirror_facet`
is the necessary and correctly-applied correction for the reverse walk.

**Consequence:** no code change. Fixture A pins the forward arc, Fixture B the `mirror_facet` reverse
arc — if `mirror_facet` were ever dropped by a refactor, Fixture B's expected `0.18668163487015432`
moves to the wrong cell and the test fails immediately.

### Item 3 — `D_in` placement (`t += α` on the cell at `P + σ·dir`)

**Paper:** the σ-shifted final tetrahedron gets an undecayed `α_vis` t-link; everything between P and
that cell is an ordinary `V_align` member with decayed facet weights.

**Code** (`:1012-1022`): `endPoint = P + σ·(P−C)/|P−C|` — exactly "σ along the line of sight"; the t-weight
goes to `locate(endPoint)`, whatever cell that is, undecayed (`t += alpha_vis`); intermediate cells
(when σ spans several) get decayed facet weights via the same backward walk and `mirror_facet`, i.e.
ordinary `V_align` links.

**Verdict — CONFIRMED-CORRECT core**, with two low-severity behavioral deviations: **(3.1)** the whole
`(P,view)` contribution — including `D_in` — is silently dropped when the very first forward
`intersect()` fails (`:993-994`), which happens when P is a vertex of the camera's own cell (essentially
impossible, since OpenMVS never inserts camera centres as triangulation vertices) or on the rare "Bad
end" fallback; **(3.2)** if the forward walk instead ends badly mid-chain, the debug `ASSERT` at `:1004`
would fail but in release the code proceeds to plant the full `D_in` anyway — conservative (favours
closed surfaces) but not paper-faithful. **(3.3, the important interaction):** when `P+σ·dir` falls
outside the convex hull (common — any back/outward surface point), `endCell` is infinite, and item 4's
`s=kInf` stamping also targets infinite cells; a node carrying both is unconditionally source-side, so
the `D_in` vote is annihilated. Charged to item 4.

**Consequence:** feeds Phase 0.B's failed-ray accounting (3.1/3.2 are countable exits) and Phase 4.1's
synthetic-fixture design (3.3 is the mechanism behind open meshes at the hull boundary).

### Item 4 — Camera `D_out` realised as hard `kInf` s-links on a *set* of cells

**Paper:** `D_out(l(C₁))=α_vis·1[l(C₁)=1]`, accumulated per line of sight, on the **single** cell
containing the sensor — large in practice but finite, so a cut may still label the camera cell inside if
surrounding evidence outweighs it.

**Code** (`:920-933`): locates the camera cell; `fetchCellFacets` has two branches — a finite camera cell
stamps `s=kInf` on exactly that one cell (all 4 facets share `.first==cell`); an infinite camera cell (the
**normal** path — OpenMVS tetrahedralises only the point cloud, so any object-centric or fly-around camera
sits outside the hull) stamps `s=kInf` on **every infinite cell** whose hull facet is front-facing to that
camera and inside its frustum — `hullFacets` entries store `.first` = the infinite cell, verified, so this
is a real band of typically hundreds-to-thousands of cells, not one. The loop also stamps for any
`IsValid()` image regardless of whether it contributed a single surviving ray.

**Verdict — DISCREPANCY, behavioral, intentional design deviation** (verified, not merely suspected: the
comment "link all cells contained by the camera to the source" plus the dedicated `kInf` parameter confirm
intent). Three packed deviations: hard `kInf` instead of finite `Σα_vis`; a frustum×hull *band* of infinite
cells instead of the one sensor cell; stamping by images with zero contributing rays.

**Consequence:** on 360°/inward captures, every hull-adjacent infinite cell any camera front-faces is
hard-free, so every `D_in` whose end cell exits the hull (item 3.3) contributes nothing — this is exactly
why OpenMVS meshes stay open at the boundary, but it means those inside-votes are silently discarded
rather than merely outweighed, and the hull can never be capped inside any camera's FOV regardless of
evidence. Documented as intentional; no fix mandated. Phase 4.1's kAbs/kOutl retune should note that
`kInf=2.684e8` is finite while the WSS-enforced `t` clamps at `maxCap=3.4e34` (item 10) — a saturated WSS
cell *can* in principle outrank a camera hard constraint on the same node.

### Item 5 — Quality term `computePlaneSphereAngle`

**Paper** (Labatut 2009 §2.3, verbatim): "soft" 3D β-skeleton — for a facet's two adjacent tetrahedra,
their circumscribing spheres intersect the facet's plane at angles φ, ψ; weight
`1 − min{cos φ, cos ψ}` is added to **both** oriented weights of the facet. Derivable closed form for a
facet circumcircle of radius `r` with apex height `h`: `cos φ = (h²−r²)/(h²+r²)`.

**Code** (`:738-775`): computes `u/R` where `u = fn·(cc−v0)` (signed distance from facet plane to
circumcentre along the inward facet normal `fn`) and `R = |cc−v0|` (exact, since `v0` is on the
circumsphere); `:1140-1141` takes `MINF` of both cells' values, adds `q=(1−min)·kQual` symmetrically to
both `AddEdge` directions, exactly once per facet.

**Verdict — CONFIRMED-CORRECT** (verified by hand for a canonical tetrahedron `A=(0,0,0),B=(1,0,0),
C=(0,1,0),D=(0,0,1)`: `fn` is inward, sign is right). Two cosmetic deviations: **(5a)** the code keeps the
sign of `cosφ` (range `[-1,1]`, `q∈[0,2]·kQual`) rather than Labatut's literal unsigned reading
(`q∈[0,1]`) — arguably the more faithful generalisation of the 2D β-skeleton's "circumcircles on opposite
sides" criterion; **do not change without a benchmark**, it roughly halves the penalty for
far-side-circumcentre facets. **(5b)** the two degenerate-facet guards (`fnLenSq==0`, `ctLenSq==0`) use
exact float equality and essentially never fire on real slivers (a tiny-but-nonzero cross product still
falls through to a `CLAMP`-bounded but numerically meaningless cosine).

**Consequence:** no code change. F5.1–F5.4 (appendix) pin exact values for a canonical tetrahedron, a
symmetric bipyramid (including the sign-discriminating "wrong side" row), and a regular tetrahedron —
regression-lock candidates for Phase 4.1's fixture work.

### Item 6 — `freeSpaceSupport()`, β/γ walks, and the triple test

**Paper** (ISRN-2014 Eq. 1, 2, 6): `f(T)=Σ_{(c,p)∈S_T} α(p)` — a bare vote count, **no distance
weighting**; `β=max` over the front window `⟨−k_f,0⟩`, `γ=(max+min)/2` over the back window `⟨0,k_b⟩`;
classifier `K = K_rel ∧ K_abs ∧ K_outl` (Eq. 6), all three **un-negated**. §4.1: window is `⟨−3σ,4σ⟩`.
Table 1 "Used": `k_rel=0.1, k_abs=1000, k_outl=400, k_f=3, k_b=4`; §3.1: `σ = 2×median edge length`.

**Code:** `freeSpaceSupport` (`:697-707`) sums the 4 incoming mirror-facet `f[]` capacities — which
inherit the same soft-visibility attenuation `1−e^{−d²/2σ²}` and `[0,1]`-confidence-weighted `α_vis` used
for the visibility energy, not the paper's bare count. β/γ walks (`:1077-1101`) sample the exited cell per
step (the window-endpoint's own cell is never sampled — minor, opposite-signed bias vs the attenuation
bias). Triple test (`:1106`) `epsRel<kRel && epsAbs>kAbs && gamma<kOutl` is a literal, exact transcription
of Eq. 6; defaults (`Scene.h:150-152`) match Table 1 exactly. CLI ships `--thickness-factor` default
`1.0` (`ReconstructMesh.cpp:132`) as `kSigma`, vs the library default `kSigma=2.0` and the paper's
`σ=2×median`.

**Verdict — DISCREPANCY (behavioral, moderate) on the support measure; CONFIRMED-CORRECT on the triple
test.** The plan brief's stated "`¬K_outl`" is the **error**, not the code — Eq. 5 defines
`K_outl(c,p)=INT for (γ<k_outl)`, already un-negated; code and paper agree. Three stacked deviations in
the support measure: distance attenuation biases γ systematically downward (inflating `ε_abs`, deflating
`ε_rel` ⇒ classifier fires **more** than the paper's calibration assumes); `α_vis` is a `[0,1]` confidence
sum, not the paper's integer count `α(p)≥1`, rescaling `k_abs`/`k_outl` by `1/E[weight]`; the shipped CLI
measures the jump over `⟨−3L,+4L⟩` instead of the paper's `⟨−6L,+8L⟩` — half the intended window,
simultaneously halving σ in the attenuation exponent — because `--thickness-factor` defaults to `1.0`
against a library default of `2.0`.

**Consequence:** rewrites the premise of Phase 4.1's "current thresholds (anchor)" arm — the anchor is
not directly comparable to the paper's own calibration (three independent scale shifts already stack
before any retune); directly motivates arm 2 (thresholds recalibrated on the bench) and confirms the
CLI/library `kSigma` mismatch flagged in Phase 4.3 needs a deliberate resolution, not just a note.

### Item 7 — WSS enforcement `t *= epsAbs`

**Paper** (ISRN-2014 §7/§8, Eq. 8 verbatim): **multiplicative** enforcement — the paper explicitly states
it does *not* set t-edges to infinity, it *enlarges* them: `w(e) = w(e)·Σ_{(c,p)∈S_K(v)} ε_abs(k_f,k_b)`,
a **single multiply by the sum** of every qualifying segment's `ε_abs`, target cell
`L_c^p(k_bσ)` (the cell 4σ behind the point).

**Code** (`:1104-1111`): inside the per-view loop, `t *= epsAbs` fires once per qualifying (vertex, view)
pair — a **per-firing-view product**, not the paper's single sum-then-multiply. Target cell
`inter.ncell == L_c^p(k_bσ)` matches Eq. 8 exactly (verified); the factor `ε_abs` and the gate (item 6's
triple test) both match.

**Verdict — DISCREPANCY, behavioral, severe.** The plan's own premise is corrected here: it is **not**
true that "ISRN-2014 sets a t-edge" — the paper prescribes multiplication, and the code also multiplies.
The real deviations are: **(a)** the code multiplies **per firing view** instead of once by the **sum** —
geometric divergence (`t·Πε` vs `t·Σε`) that overflows `float` after as few as ~13 same-cell
reinforcements at the default `k_abs=1000`, saturating at `maxCap` into an effectively infinite t-link —
precisely the hard constraint the paper explicitly rejected in favour of enforcement. **(b)** the resulting
`α²` unit mismatch (`t` and `ε_abs` both carry units of α, so `t·ε_abs` is quadratic in the confidence
scale while every other graph capacity is linear) is **inherited from the paper's own Eq. 8**, not
introduced by OpenMVS — any candidate fix that removes this defect (`+=`, `max(t,·)`) is a deliberate
departure from the paper, not a restoration of fidelity, and should be labelled as such. **(c)** the
`t==0` no-op is **structural**, not incidental: the base `t` is deposited at **1σ** behind the point
(`:1013-1022`, the ~1σ back-walk-start cell) while enforcement targets the walk's end cell at **4σ**
(`:1107`) — usually a different cell, since a typical tetrahedron spans roughly one median edge; the
target's `t` stays 0 unless some *other* point's 1σ endpoint happens to land in it. The failure is
anti-correlated with the mechanism's design purpose: occluder interiors with no input points never
receive a nonzero base `t`, so the classifier's flagship use case is exactly where it is silently inert.

**Consequence:** rewrites Phase 4.1's enforcement-semantics arms from three to **four**: (1) current
per-firing-view product (anchor); (2) paper-faithful Σ-then-multiply-once; (3) additive
`t += kw·epsAbs`; (4) `t = max(t, kw·epsAbs)`. Arms 3/4 are honest departures from the paper (they fix a
defect the paper itself carries), not restorations of fidelity, and the writeup should say so explicitly.

### Item 8 — `orientation()` absolute epsilon and the ray-walk failure paths

**Analysis basis:** no paper prescribes this constant; it is a numerical-robustness question answered by
Shewchuk's `orient3d` static-filter theory — a determinant that scales as `L³` needs an epsilon that scales
as `L³`, not a fixed constant.

**Code** (`:354-386`): `orientation()` computes a 3×3 determinant in **double** precision (not float —
verified; every intermediate is `double`) with fixed `eps=1e-12` (`:368`); the disabled `#else` branch
(`:369-377`) is a hand-rolled relative-epsilon attempt, currently unreachable (`#if 1` at `:366`).

**Verdict — DISCREPANCY, MEDIUM overall, HIGH if triggered.** The fixed `1e-12` is correctly calibrated
only near tetrahedron edge length `L≈6` scene units (the break-even point where `1e-12` equals the true
Shewchuk noise floor `≈4.7e-15·L³`). **Below `L≈1.12e-4` units, total silent collapse**: `0.707·L³ <
1e-12` for a regular tetrahedron, so `orientation()` returns `COPLANAR` from every call; the walker's
`switch` has no `case 3` (nb_coplanar==3 falls through to `break`, snapshot `:646`), so **every ray in the
scene Bad-ends at its first step**, producing an empty/garbage mesh with no diagnostic. Above `L≈6` the
epsilon is inert, but low-severity, because the two degeneracies that actually matter — segment-endpoint-
hits-a-vertex, and an edge-coplanar shared facet pair — are handled by **exact** float cancellation and
**exact** antisymmetry (`det(p,q,x,y) == -det(p,q,y,x)` bitwise, verified over 20,000 random inputs),
independent of epsilon.

**The plan's causal hypothesis is OVERTURNED, verified by direct execution.** `orientation()` subtracts
coordinates before forming the determinant — every matrix entry is a first-order difference of doubles —
so the predicate is provably translation/magnitude-independent (bit-exact identical behaviour verified at
`M=1e6`; Sterbenz's lemma makes the subtractions exact for `M≫L`). **Georeferenced scenes do not fail
through this epsilon.** They fail upstream: `PointCloud::Point` (`libs/MVS/PointCloud.h:57`) is `float`,
giving ~6 cm quantization at UTM-easting magnitude (`M≈1e6`) — the geometry is destroyed by storage
*before* triangulation ever runs (coincident/near-coincident vertices, zero-area facets → NaN plane
normals, item 10). Mesh-time normalization cannot recover data already lost to float storage; the fix
needs centering/rescaling **at load time**. Separately, the disabled relative-epsilon variant (`8b`) is
**confirmed incorrect as written** and must not be enabled: it is dimensionally wrong (`eps~L¹` vs
`det~L³`, so the error itself grows as `L²` and is wrong in both directions around `L=1`), uses per-row
maxima instead of CGAL's per-**column** maxima, drops the third row entirely (the vector that actually
varies along a ray walk), and has no magnitude-window check or exact fallback.

**Consequence:** redirects Phase 3.4's "canonical-box coordinate normalization" fix — a naive unit-box
**extent** target actually *creates* the `L≲1.1e-4` collapse regime at ≥8.3e7 points (extent shrinks as
`N^(−1/2)` for a surface cloud); the correct rescale target is **median-edge-length≈1**
(`s=1/L̄`, a two-pass triangulate→measure→rescale→retriangulate, or an a-priori L̄ estimate), plus a
**separate** fix for float-quantized large-coordinate point clouds applied at load time, not mesh time.
Also feeds Phase 0.B directly: item 8(c)'s inventory of 13 distinct silent ray-walk exits (none counted
today; 2 — the `:1004`/`:1033` ASSERTs — silently corrupt the cached WSS walk-start cell in release
builds; no step cap anywhere; 4 unguarded NaN/non-finite entry points, the dominant one being degenerate
facets) is an implementation-ready counter-insertion plan for the failed-ray accounting task already
scoped in plan §0.B.

### Item 9 — σ estimation

**Paper:** `σ = 2×median Delaunay edge length` (ISRN-2014 §3.1).

**Code** (`:943-951`): true median (`cList::GetMedian`, `std::nth_element`, not binned) of **squared**
finite Delaunay edge lengths, square-rooted; `SQRT(median(d²)) ≡ median(d)` exactly, since median
commutes with any monotone transform — the correct construction (the obvious alternative
`SQRT(mean(d²))` would be dominated by exactly the long-edge tail this guards against).

**Verdict — CONFIRMED-CORRECT.** Hull/sliver contamination of the finite-edge population is bounded by
`≈2·N^(−1/2)` (0.2% at N=1e6, 0.03% at N=5e7) — two orders of magnitude below the median's 50% breakdown
point, so it neither saturates nor shifts, even under a generous worst-case bound. Subsampling to
`k≈1e5` edges changes σ by `<0.3%` (asymptotic median SE, verified across CV=0.4–0.8 lognormal edge-length
models), far below `kSigma`'s own tuning granularity (a 2× library-vs-CLI disagreement already exists,
item 6) — **safe**, provided the sample is strided/reservoir rather than a prefix: insertion uses
`CGAL::spatial_sort`, so `finite_edges_begin()` is spatially coherent and a prefix would sample a single
biased Hilbert-curve segment.

**Consequence:** enables Phase 3.3's "sampled edge-length σ estimation" profiling item without a fidelity
caveat. Also documents σ as a **"typical local cell size"** (~1.5–3× point spacing, not point spacing
itself, since each vertex has ≈15.5 Delaunay neighbours spanning 2nd/3rd rings) — a non-defect any future
`kSigma`/`k_f`/`k_b` retune must be aware of. The transient `FloatArr` allocation (1.45 GiB at N=5e7) is
the practical argument for subsampling, ahead of CPU time.

### Item 10 — Capacity hygiene

**Theory:** two-terminal graph-cut representability requires `θ(0,0)+θ(1,1) ≤ θ(0,1)+θ(1,0)`, i.e.
`cap(i→j)+cap(j→i) ≥ 0`; non-negative capacities in both directions is sufficient. `maxCap` should bound
any WSS-enforced `t` before it reaches the solver.

**Code:** every accumulation site (`:902` zero-init, `:932` `s=kInf`, `:998-1002`/`:1026-1031` `f+=w`,
`:1018-1022` `t+=alpha_vis`, `:1104-1111` `t*=epsAbs`, `:1140-1141` `q` and `AddEdge`) is non-negative
under the documented `[0,1]`-confidence contract, but **unvalidated** at the API boundary (no runtime
check on deserialised `pointWeights` sign, nor on caller-supplied `kInf`/`kQual`/`kAbs`/`kSigma`).
`MINF(ciInfo.t, maxCap)` at `:1133` (graph-build scope) is **strictly downstream** of the `:1111` multiply
(weighting scope) — it cannot prevent overflow, only remap it afterward.

**Verdict — mixed.** **CONFIRMED-CORRECT:** submodularity (every pairwise capacity non-negative in both
directions is sufficient and holds on every path; the symmetric `q` term cannot break it regardless of
`kQual`'s magnitude; the WSS term is unary, so submodularity is vacuous for it) and non-negativity under
the documented contract. **DISCREPANCY, MEDIUM-HIGH on WSS-on paths — but the plan's framing needs
correcting:** `t *= epsAbs` does overflow to `+inf` after as few as **13** same-cell reinforcements at
default `k_abs=1000` — **but `MINF`/`std::min` correctly clamps `+inf→maxCap` (3.4e34)**, so overflow is a
**semantics distortion, not corruption**: a soft reinforcement silently becomes a hard constraint 26
orders of magnitude above the camera's own `kInf` hard constraint (item 4), and 13–130 reinforcements
produce byte-identical (fully saturated) graphs — lossy and unobservable, but not unsafe. **The actual
corrupting path is NaN:** `std::min` returns its **first** argument when the comparison is false, so
`MINF(NaN, maxCap) = NaN` — NaN is **not** clamped and reaches `graph.AddNode` intact in release builds
(the `ASSERT(ISFINITE(sink)&&sink>=0)` at `:88` is compiled out). The traced NaN entry point is
`normalized()` on a zero-area facet (`libs/Common/Plane.inl:66`, `0/0`), reachable via degenerate facets
from item 8's float-quantized/large-coordinate clouds. **DISCREPANCY, MEDIUM on container hygiene**
(separate from the above): the IBFS arc buffer is sized with **zero slack** (`2·numCells` edges, exact
fit) behind a release-stripped `assert` (`IBFS.h:431`); there is no `dimension()==3` guard before the
graph build unconditionally dereferences 4 neighbours per cell (a degenerate/coplanar input reaches
`initSize(0,0)` then `addEdge` calls — silent heap overflow); and `int` arithmetic in the arc-count
computation (`libs/MVS/Scene.h` graph ctor, `IBFS.cpp:129-131`) overflows at ≈79M points.

**Consequence:** corrects the plan's item-10 framing from "confirm `t*=epsAbs` cannot overflow before the
clamp" (it **does** overflow, but the overflow **is** caught) to "confirm no NaN reaches the solver" — the
NaN path is the one to instrument. Feeds Phase 0.B's failed-ray accounting (add an `ISFINITE` counter at
graph build, not only an overflow counter) and ties directly to item 8's degenerate-facet root cause. The
container-sizing issues are independent hardening candidates (a `dimension()==3` guard) ahead of any
large-aerial-job push, not gated on the energy-term work.

---

## Appendix — Fixture specifications

Every numbered hand-solvable fixture from the three source audits, preserved verbatim (geometry and
expected values unchanged) for consumption by later unit-test work under plan Phase 0.C / 4.1. Full
derivation and cross-checks are in the source audit reports; only the fixture specifications themselves
are reproduced here.

### Common harness notes (fixtures for items 1–4)

Apply to all four fixtures (A, B, C, D) below:

* Build the `Scene` in memory: `scene.pointcloud.points/pointViews/pointWeights` +
  `scene.images` with valid `Camera` (`camera.C`, `camera.P`, `imageData.width/height`,
  `imageData.ID`), then call
  `scene.ReconstructMesh(distInsert=0.f, bUseFreeSpaceSupport=false, bUseOnlyROI=false,
   nItersFixNonManifold=0, kSigma=<below>, kQual=0.f, ...)`.
  * `distInsert = 0` ⇒ the "insert all points" branch (**839–842**), no vertex merging.
  * `bUseFreeSpaceSupport = false` ⇒ the WSS block (**1048–1117**) is skipped, so `t` is not
    multiplied.
  * `kQual = 0` ⇒ `q ≡ 0` at **1140**, so arc capacity == `f` exactly.
* `pointWeights` left empty ⇒ every `α_vis = 1` (**252**).
* The energy state (`infoCells`) is local to `ReconstructMesh` and cleared at **1144**, so a
  unit test must either (i) expose a test hook that snapshots `infoCells` + the cell↔index map
  before **1144**, or (ii) assert on the resulting `mesh` only. **Recommendation: add a
  `#ifdef _USE_TEST_HOOKS` callback taking `(const delaunay_t&, const std::vector<cell_info_t>&)`
  right before `infoCells.clear();` (snapshot 1144).** Everything below is written against that
  hook.
* **Do not assume CGAL cell indices.** Identify cells by `delaunay.locate(<interior probe
  point>)` and facets by `cell->index(vertexHandleOf(X))`; identify vertices by
  `delaunay.nearest_vertex(point_t(...))`. All the fixtures below are Delaunay-**unique**
  (verified in §5.1/§5.2), so the combinatorics are stable, but the *numbering* is not.
* Cameras must look at the scene with a wide FOV: `width = height = 640`,
  `K = [200 0 320; 0 200 240; 0 0 1]`, `R` as stated, `C` as stated. Any FOV that contains the
  whole point set works — the frustum only gates `fetchCellFacets` on **infinite** cells.
* Tolerance: `1e-6` absolute on `edge_cap_t` (float) comparisons.

### Fixture A — "bipyramid": 2 finite tetrahedra, 1 camera, 1 contributing point
*(the requested hand-solvable item-2 fixture; also pins item 1 and the item-4 finite branch)*

**Points** (all 5 inserted; each has `pointViews = {0}`):

| name | coordinates |
|---|---|
| A | `( 1.0,  0.0,               0.0)` |
| B | `(-0.5,  0.8660254037844386, 0.0)` |
| C | `(-0.5, -0.8660254037844386, 0.0)` |
| D | `( 0.0,  0.0,               3.0)` |
| E | `( 0.0,  0.0,              -3.0)` |

`A,B,C` = equilateral triangle, circumradius 1, in the plane `z = 0`, centred on the z-axis.

**Camera 0**: `C = (0, 0, 1.5)`, `R = diag(1,1,-1)`-style pose looking along **−z**
(any pose whose frustum contains the whole bipyramid).

**Delaunay uniqueness (verified numerically):**
circumsphere(A,B,C,D) = centre `(0,0,4/3)`, r = `5/3 = 1.6666667`; `|E − centre| = 4.3333333 > r` ✔
circumsphere(A,B,C,E) = centre `(0,0,−4/3)`, r = `5/3`; `|D − centre| = 4.3333333 > r` ✔
⇒ the triangulation is exactly `T_up = {A,B,C,D}`, `T_dn = {A,B,C,E}` sharing facet `ABC`,
plus 6 infinite cells (one per hull facet `ABD, BCD, CAD, ABE, BCE, CAE`).
`(0,0,1.5)` is strictly inside `T_up` ✔ (verified).

**σ:** finite edges are `AB,BC,CA` (`len² = 3`, ×3) and `AD,BD,CD,AE,BE,CE` (`len² = 10`, ×6);
9 values ⇒ odd ⇒ `GetMedian` = `GetNth(4)` = **10**. Pass
`kSigma = 0.31622776601683794` (= 1/√10) ⇒ **σ = 1.0 exactly**, `inv2SigmaSq = 0.5`.

**Ray inventory (derived, not assumed):**
* rays to `A, B, C, D` — each is a vertex of the camera's own cell `T_up`, so the very first
  `intersect` (snapshot **993**) hits that vertex and returns `false` ⇒ `continue`
  ⇒ **zero contribution, no `t`** (this is sub-finding 3.1 in action).
* ray to `E` — the segment `(0,0,1.5) → (0,0,−3)` crosses facet `ABC` at its centroid
  `(0,0,0)` (strictly interior to the triangle), enters `T_dn`, then terminates at vertex `E`.

**Expected state at the hook (α = 1, kQual = 0, kInf as passed):**

| quantity | expected |
|---|---|
| `infoCells[T_up].f[T_up->index(D)]` | `0.9888910034617577`  (= `1 − e^{−4.5}`, `d = 3`) |
| every other `f[·]` in the whole triangulation | `0.0` |
| `infoCells[T_up].s` | `kInf` |
| `s` of **every** other cell (incl. all 6 infinite) | `0.0` |
| `Σ_cells t` | `1.0` |
| the single cell with `t != 0` | is **infinite** and is incident to vertex `E` |
| arc `T_up → T_dn` capacity | `0.9888910034617577` |
| arc `T_dn → T_up` capacity | `0.0` |

Locators: `T_up = delaunay.locate(point_t(0,0,1.0))`, `T_dn = delaunay.locate(point_t(0,0,-1.0))`;
`vD = delaunay.nearest_vertex(point_t(0,0,3))`, `vE = delaunay.nearest_vertex(point_t(0,0,-3))`;
facet `ABC` from `T_up` is `facet_t(T_up, T_up->index(vD))`, from `T_dn` it is
`facet_t(T_dn, T_dn->index(vE))`.

**What this fixture proves:** (item 2) the free→full capacity for a camera-side crossing sits on
the arc `T_up → T_dn`, i.e. *along the ray*, and the reverse arc is exactly zero; (item 1) the
weight is `α(1 − e^{−d²/2σ²})` with `d` = 3 = distance from **P = E**, *not* 1.5 = distance from
the camera (which would give `1 − e^{−1.125} = 0.6753475`, an unmistakably different number —
this single assertion is the whole point of item 1); (item 4) the finite-camera-cell branch
stamps exactly one cell.

### Fixture B — "tetra + interior point": `mirror_facet` and the σ-shifted `D_in`
*(item 2 reverse arc, item 3 core, item 4 infinite branch)*

**Points** (all 5 inserted). Let `s3 = 1.7320508075688772`.

| name | coordinates | `pointViews` |
|---|---|---|
| P  | `( 0.0, 0.0, 0.0)` | `{0}` |
| V0 | `( 1.5, 0.5, 6.0)` | `{1}` |
| V1 | `( 4.0, 0.0, -2.0)` | `{0}` |
| V2 | `(-2.0,  2*s3, -2.0)` = `(-2.0,  3.4641016151377544, -2.0)` | `{0}` |
| V3 | `(-2.0, -2*s3, -2.0)` = `(-2.0, -3.4641016151377544, -2.0)` | `{0}` |

**Camera 0**: `C = (0, 0, -10)`, looking along **+z**, wide FOV.
**Camera 1**: `C = (1.5, 0.5, 26)`, looking along **−z**, wide FOV.
(Camera 1 exists only so `V0` has a view whose ray provably contributes nothing — see below. If
the harness can run with `NDEBUG` and an empty `pointViews[V0]`, camera 1 can be dropped; the
`continue` at snapshot **971** handles empty view lists, only the `ASSERT`s at **243/832**
object.)

**Triangulation:** `P` is strictly inside tetra `V0V1V2V3` (verified) ⇒ the point set has a
**unique** triangulation, the star of `P`:
`Ca = {P,V1,V2,V3}`, `Cb = {P,V0,V2,V3}`, `Cc = {P,V0,V1,V3}`, `Cd = {P,V0,V1,V2}`, plus 4
infinite cells over the hull facets `V1V2V3, V0V1V2, V0V2V3, V0V1V3`.

**σ:** the 10 finite edge lengths² sorted are
`[20, 20, 20, 38.5, 48, 48, 48, 70.5, 85.0359, 91.9641]`; `n = 10` (even) ⇒
`GetMedian = (sq[4]+sq[5])/2 = (48+48)/2 = 48`. Pass
`kSigma = 0.5773502691896258` (= 1/√3) ⇒ **σ = 4.0 exactly**, `inv2SigmaSq = 0.03125`.

**Ray inventory (derived):**
* `V1, V2, V3` from camera 0: the segments `(0,0,-10) → Vi` lie entirely in `z ≤ −2`, i.e.
  entirely outside the hull except at the endpoint ⇒ first `intersect` hits the vertex ⇒
  `continue` ⇒ **no contribution, no `t`**.
* `V0` from camera 1: the segment `(1.5,0.5,26) → (1.5,0.5,6)` lies entirely in `z ≥ 6`, outside
  the hull except at `V0` ⇒ same ⇒ **no contribution, no `t`**.
* `P` from camera 0 — the only contributing ray:
  * walk 1: crosses hull facet `V1V2V3` at `(0,0,-2)` (the base triangle's centroid, strictly
    interior), enters `Ca`, terminates at vertex `P`. `d₁ = 2.0` exactly.
  * walk 2: `endPoint = P + 4·(0,0,1) = (0,0,4)`, outside the hull. The +z ray from `P` enters
    cell **`Cb = {P,V0,V2,V3}`** (verified by point-in-tetra on `P + εẑ`) and exits through
    facet `V0V2V3` at `(0, 0, 18/7)` — barycentric `(0.5714286, 0.1730464, 0.2555250)` w.r.t.
    `(V0,V2,V3)`, i.e. **strictly interior** to the triangle. `d₂ = 18/7 = 2.5714285714285716`.

**Expected state at the hook (α = 1, kQual = 0):**

| quantity | expected |
|---|---|
| `infoCells[infCell(V1V2V3)].f[·]` for facet `V1V2V3` | `0.11750309741540454`  (= `1 − e^{−0.125}`, `d = 2`) |
| `infoCells[Cb].f[Cb->index(vP)]` (facet `V0V2V3`) | `0.18668163487015432`  (= `1 − e^{−(18/7)²/32}`, `d = 18/7`) |
| `infoCells[Ca].f[Ca->index(vP)]` (mirror of `V1V2V3`) | `0.0` |
| `infoCells[infCell(V0V2V3)].f[·]` for facet `V0V2V3` (mirror) | `0.0` |
| every other `f[·]` | `0.0` |
| `s` of `Ca, Cb, Cc, Cd` (all finite cells) | `0.0` |
| `s` of **all 4** infinite cells | `kInf` (camera 0 front-faces `V1V2V3`; camera 1 front-faces the other three — verified by outward-normal sign tests) |
| `Σ_cells t` | `1.0` |
| the single cell with `t != 0` | is **infinite** and is the one containing `(0,0,4)` |

Locators: `Ca = delaunay.locate(point_t(0, 0, -1.5))`, `Cb = delaunay.locate(point_t(-0.6, 0, 1.0))`
(or simply `delaunay.locate(point_t(0,0,1.0))` — the +z probe from `P`),
`vP = delaunay.nearest_vertex(point_t(0,0,0))`.

**What this fixture proves:** (item 2) the behind-the-point crossing is deposited through
`mirror_facet` on the arc `Cb → infCell(V0V2V3)`, i.e. **away from the camera**, and the reverse
arc is exactly zero — if `mirror_facet` were dropped, `0.18668163` would appear on
`infoCells[infCell(V0V2V3)].f[·]` instead and the test flips; (item 1) both `d`'s are distances
from `P` (2 and 18/7) — a camera-origin `d` would give 8 and 12.571, i.e. `w₁ = 0.8646647`,
`w₂ = 0.9926966`; (item 3) `t` lands on the cell at `P + σ·dir` and nowhere else, undecayed;
(item 4) all infinite cells are hard-stamped while the 4 finite cells are untouched.

### Fixture C — negative test for sub-finding 3.1 (whole ray dropped)

Take **Fixture A** unchanged and assert what the ray to `D` produces:

* `D = (0,0,3)` is a vertex of the camera cell `T_up`, so the ray from `(0,0,1.5)` crosses no
  facet.
* Expected under the current code: **`Σ_cells t == 1.0`** (only `E`'s ray), i.e. `D`'s `D_in`
  is *missing*.
* Expected under the paper: `Σ_cells t == 5.0` (one per point/view pair), with `D`'s `α_vis`
  landing on the cell containing `D + 1·(0,0,1) = (0,0,4)`.

Write the assertion against the **current** value (`1.0`) with a comment naming this file, so a
future change of the `continue` at snapshot **993–994** trips the test deliberately rather than
silently.

### Fixture D — `D_in` annihilation probe (item 4, consequence 2)

Take **Fixture B** and add **camera 2** at `C = (0, 0, 30)` looking along **−z** with a wide FOV
(a second "top" camera; it needs no points of its own — an image with no rays still stamps, which
is deviation D3, and that is precisely what this fixture documents).

* Camera 2 front-faces the same three upper hull facets as camera 1, so
  `infCell(V0V2V3).s == kInf` (unchanged from Fixture B).
* `P`'s `D_in` still lands on `infCell(V0V2V3)`'s neighbourhood with `t == 1.0`.
* **Assertion:** there exists a cell with `s == kInf` **and** `t > 0` simultaneously, and after
  `ComputeMaxFlow` that cell satisfies `IsNodeOnSrcSide(cell) == true` — i.e. the inside-vote is
  provably discarded. Assert this explicitly, again with a pointer to this document, so the
  behaviour is locked and visible rather than emergent.

### Item 5 fixtures — `computePlaneSphereAngle`

All values below are exact or exact-to-7-digits and require no reference implementation.

**F5.1 — orientation/sign probe, "corner tetrahedron" (single cell + infinite neighbours).**
Insert exactly `A=(0,0,0), B=(1,0,0), C=(0,1,0), D=(0,0,1)`. One finite cell; circumcentre `(0.5,0.5,0.5)`,
`R = √3/2 = 0.8660254`.

| facet (opposite vertex) | plane | circumcentre side | expected `computePlaneSphereAngle` |
|---|---|---|---|
| opp. `A` | `x+y+z=1` | far side | `−1/3 = −0.3333333` |
| opp. `B` | `x=0` | apex side | `+1/√3 = +0.5773503` |
| opp. `C` | `y=0` | apex side | `+1/√3` |
| opp. `D` | `z=0` | apex side | `+1/√3` |

(Match by geometry, not by CGAL facet index — the index depends on insertion order.) A sign flip in the facet
normal convention shows up immediately as all four values negated. Every facet here is a hull facet, so the
`min` at `:1140` pairs each with the infinite cell's `1.f`, giving `q = (1 − value)·kQual`:
`q_opp.A = 4/3·kQual`, `q_others = (1 − 1/√3)·kQual = 0.4226497·kQual`.

**F5.2 — symmetric bipyramid, exercises the two-sided `min` and the symmetric `AddEdge`.**
Insert `P0=(0,0,0)`, `P1=(1,0,0)`, `P2=(0.5, √3/2, 0)` (equilateral, side 1, circumradius `r = 1/√3`,
circumcircle centre `(0.5, √3/6, 0)`), plus `Q⁺=(0.5, √3/6, +h)` and `Q⁻=(0.5, √3/6, −h)`.
Both tetrahedra have their apex on the facet's circumcircle axis, so `cos φ = (h² − r²)/(h² + r²)` with
`r² = 1/3`, and by mirror symmetry `cos ψ = cos φ`, hence `q = (1 − cos φ)·kQual` on the shared facet `z = 0`:

| `h` | `u = (h²−r²)/(2h)` | `R = (h²+r²)/(2h)` | `cos φ = cos ψ` | expected `q/kQual` | note |
|---|---|---|---|---|---|
| `3` | `1.4444444` | `1.5555556` | `13/14 = 0.9285714` | `1/14 = 0.0714286` | large empty spheres → tiny penalty ✔ Labatut |
| `1` | `1/3` | `2/3` | `0.5` | `0.5` | |
| `1/√3 = 0.5773503` | `0` | `0.5773503` | `0` | `1.0` | facet circle is a great circle |
| `1/3` | `−0.3333333` | `0.6666667` | `−0.5` | `1.5` | circumcentre on the **wrong** side → `q > 1`; this row is the one that discriminates the signed vs. unsigned reading (unsigned would give `q = 0.5`) |

Delaunay validity check for `h=1`: sphere centre `(0.5,√3/6,1/3)`, `R=2/3`; distance to `Q⁻` is `4/3 > 2/3`, so
the sphere is empty. ✔ (`h=3` and `h=1/√3` likewise; for `h=1/3` verify emptiness before use.)

**F5.3 — regular tetrahedron reference value.** Vertices `(1,1,1), (1,−1,−1), (−1,1,−1), (−1,−1,1)`.
Circumcentre = origin, `R = √3`, inradius = `R/3`. Expected `computePlaneSphereAngle = +1/3` on all four
facets; with an identical mirrored neighbour, `q = 2/3·kQual`.

**F5.4 — assembly assertion.** For any facet shared by cells `ci`/`cj`, assert
`capacity(ci→cj) − f_ci[i] == capacity(cj→ci) − f_cj[j] == q` (same `q` both directions), and assert `q` is
applied exactly once (no double-add) by counting `AddEdge` calls == number of distinct facets.

### Item 6 fixtures — `freeSpaceSupport()` / β,γ walks / triple test

**F6.1 — nominal fire.** `front = [10, 900, 1200]`, `back = [5, 3, 0, 2]`.
`β = 1200`; `γ = (0+5)/2 = 2.5`; `ε^abs = 1197.5 > 1000` ✔; `ε^rel = 0.00208333 < 0.1` ✔; `γ = 2.5 < 400` ✔
⇒ **INT**.

**F6.2 — all three reject.** `front = [10, 900, 1200]`, `back = [5, 3, 0, 2000]`.
`γ = (0+2000)/2 = 1000`; `ε^abs = 200` (fails `>1000`); `ε^rel = 0.8333` (fails `<0.1`); `γ = 1000` (fails
`<400`) ⇒ **NOI**.

**F6.3 — `K_outl` is the only rejecter, and the inequality is strict.**
`front = [5000]`, `back = [390, 410]` → `γ = 400.0` exactly, `ε^abs = 4600 > 1000` ✔,
`ε^rel = 0.08 < 0.1` ✔, `γ < 400` is **false** ⇒ **NOI**.
Perturb to `back = [389.8, 410]` → `γ = 399.9` ⇒ **INT**. This pair pins the strictness of `<` at `k_outl`
and proves `¬K_outl` (the brief's reading) is *not* what is implemented — under `¬K_outl` the two rows would
swap.

**F6.4 — degenerate `β`.** `front = [0, 0]`, `back = [0, 0]` → `ε^rel = NaN`, `ε^abs = 0` ⇒ **NOI**, and assert
no NaN is written to any `t`.

**F6.5 — `freeSpaceSupport` == sum of incoming, on a 5-point triangulation.** Reuse F5.2's bipyramid
(`P0,P1,P2,Q⁺,Q⁻` → 2 finite cells + 6 infinite cells). Bypass the ray walk: write known values directly into
`infoCells[*].f[*]`, then assert `freeSpaceSupport(T⁺)` equals the sum of the four `f` entries of the four
*neighbours* indexed **toward** `T⁺`, and that permuting which side of each facet holds the weight changes the
result (i.e. the test fails if `mirror_facet` is dropped). This is the only cheap way to lock the
incoming-vs-outgoing convention, which is the one thing in item 6 that no comment proves.

**F6.6 — σ.** Point set with hand-known Delaunay edge lengths (e.g. F5.2's bipyramid at `h=1`: edges
`1,1,1` for the triangle and `|P_i − Q^±|` all equal by symmetry). Assert
`sigma == median(edge lengths) * kSigma` and, separately, assert the value actually reached from
`apps/ReconstructMesh` with default flags is `1×` not `2×` the median — that is the deviation to record.

### Item 7 fixtures — WSS enforcement `t *= epsAbs`

**F7.1 — sum-vs-product discriminator (pure arithmetic, no triangulation needed).**
Given one target cell with `t_0 = 2.5` and two INT segments with `ε_1 = 1200`, `ε_2 = 1500`:

| formulation | expected `t_final` |
|---|---|
| code `t *= ε` per segment | `2.5 · 1200 · 1500 = 4 500 000` |
| paper Eq. (8) `t · Σ ε` | `2.5 · (1200 + 1500) = 6 750` |
| candidate `t = max(t, kw·ε)` with `kw=1` | `1500` |
| candidate `t += kw·ε` with `kw=1` | `2.5 + 2700 = 2702.5` |

Pin the current behaviour at `4 500 000` so any later change is a deliberate, visible test edit.

**F7.2 — the `t == 0` no-op.** Same as F7.1 but `t_0 = 0` ⇒ **all four** of `t*=ε` and `t·Σε` give `0`, while
`max(t, kw·ε)` gives `1500` and `t += kw·ε` gives `2700`. This single row is the whole Phase-4 experimental
question; it belongs in the test suite as a *characterisation* test now and becomes the assertion later.

**F7.3 — target-cell identity.** On a synthetic scene with one camera at `C`, one point `p`, and enough filler
points that the ray `C→p` crosses ≥ 6 cells within `4σ` behind `p`, assert:
`infoCells[inter.ncell].` is the cell containing `p + k_b·σ·û` (compare against
`delaunay.locate(p + kb*sigma*û)`), and assert it is **not** the cell containing `p + 1·σ·û` (the cell that
received the base `t`). That single inequality is the mechanical proof of D-3 and is worth having as a red test.

**F7.4 — saturation.** Feed 15 INT segments with `ε = 1200` at one cell with `t_0 = 1`. Assert
`infoCells[cell].t` is `+inf` (or ≥ `maxCap`) and that `MINF(t, maxCap)` at `:1133` clamps it to
`3.402823466e+34f`, i.e. an effectively infinite t-link — documenting that the "enforce, don't set to infinity"
design intent is violated in practice.

**F7.5 — overflow/NaN guard.** Assert no `NaN` ever reaches `graph.AddNode` (combination of F6.4's `β=0` path
and F7.4's overflow path).

*(Items 8, 9, and 10 are numerical-robustness/hygiene audits, not hand-solvable graph-cut fixtures — the
counter-insertion plan in item 8's writeup above is a diagnostics harness, not a fixture spec, and is not
reproduced here. See the "left out / contradictory" note in the closing summary.)*

---

## Phase 3.1 — weighting-loop deserialization (2026-08-19) — ACCEPTED

Replaced the `#pragma omp critical` iterator handoff in both weighting loops with a
pre-collected, view-filtered vertex-handle vector (built once, reused by both loops),
`schedule(dynamic)` indexed parallel-for, and per-thread reserved facet buffers; all capacity
atomics unchanged. Gates: single-thread outputs md5-identical (4/4 PLYs, WSS on+off); every
order-independent walk counter identical multithreaded on SceauxCastle and Truck (31.7 M
walks, 4.07 G steps); Truck median weighting −7.0 % (α_vis) and −9.5 % (t-edge loop),
non-overlapping run distributions with untouched-stage controls flat; peak RSS +0.0 %; both
test suites green. Timing includes the new pre-collection pass (net figures). Note: the
`saturated` WSS counter is order-dependent across runs by design (documented in code) —
observed drifting ±2 in baseline as well; not a regression.

## Consequences fed into the plan

* **Item 4** (verified `kInf` band-stamping on frustum-visible hull-adjacent infinite cells, not just
  the camera cell) confirms plan §0.C item 4's premise exactly; no plan phase requires a code change, but
  Phase 4.1's `kAbs`/`kOutl` retune should account for the fact that a saturated WSS `t` (item 10) can in
  principle outrank the camera's own `kInf` hard constraint on a shared node.
* **Item 6** (`freeSpaceSupport` is distance-attenuated and `[0,1]`-confidence-weighted, not the paper's
  bare integer vote count; CLI `--thickness-factor=1.0` halves the intended `⟨−3σ,+4σ⟩` window to
  `⟨−3L,+4L⟩` against the library's own `kSigma=2.0` default) directly rewrites the premise of Phase
  4.1's "current thresholds (anchor)" arm — the anchor is not comparable to the paper's calibration before
  any retune — and motivates arm 2 (recalibrate on the bench). It also confirms the CLI/library `kSigma`
  mismatch flagged in Phase 4.3 needs a deliberate, tracked resolution rather than a passing note. Triple
  test itself needed no plan change (confirmed correct; the plan brief's `¬K_outl` reading was the error).
* **Item 7** (enforcement is a per-firing-view product, not the paper's sum-then-multiply-once; the
  `t==0` no-op is structural — base `t` at 1σ, enforcement target at 4σ, usually different cells) rewrites
  Phase 4.1's semantics arms from three to four: current per-view product (anchor), paper-faithful
  Σ-then-multiply-once, additive `t += kw·epsAbs`, and `t = max(t, kw·epsAbs)`. The α² unit defect is
  inherited from the paper's own Eq. 8 — arms that remove it are deliberate departures, not fidelity
  restorations, and Phase 4.1's writeup should label them as such.
* **Item 8** (the predicate is provably translation-independent; the plan's georeferenced-scene
  hypothesis is overturned; the real failure modes are miniature-scene epsilon collapse below `L≈1.1e-4`
  and upstream `float`-precision loss in `PointCloud::Point` at UTM magnitude) redirects Phase 3.4's
  "canonical-box coordinate normalization" fix: the rescale target must be median-edge-length≈1, not
  raw extent — a naive unit-box extent target creates the same collapse regime at ≥8.3e7 points — and a
  **separate** load-time fix is needed for float-quantized large-coordinate clouds; mesh-time
  normalization cannot recover data already lost to `float` storage. The disabled relative-epsilon
  variant referenced in plan §0.C item 8 ("the slower fallback") is confirmed incorrect as written and
  must not be enabled without a full rewrite to CGAL's column-max, sorted, windowed form.
* **Item 8(c)**'s ray-walk failure-path inventory (13 silent exits, 2 that corrupt the cached WSS
  walk-start cell in release, no step cap, 4 unguarded NaN entry points) is an implementation-ready
  counter-insertion plan that directly fulfils Phase 0.B's "failed-ray accounting" bullet.
* **Item 10(b)** (overflow of `t*=epsAbs` to `+inf` IS clamped by `MINF`/`std::min` at `AddNode`; NaN is
  the unclamped, corrupting path, traced to `normalized()` on zero-area facets) corrects plan §0.C item
  10's framing from "confirm the multiply cannot overflow before the clamp" (it can, and does, but the
  overflow is caught) to "confirm no NaN reaches the solver" — Phase 0.B's instrumentation should add an
  `ISFINITE` counter at graph build, which also closes the loop with item 8's degenerate-facet root cause.
* **Item 9** (median-based σ estimation confirmed correct; subsampling to `k≈1e5` edges changes σ by
  `<0.3%`, far below the parameter's own tuning granularity, provided the sample is strided/reservoir)
  clears Phase 3.3's "sampled edge-length σ estimation" profiling item for use without a fidelity caveat.
* **Items 1, 2, 3 (core), 5, 10(a), 10(c)** are CONFIRMED-CORRECT and require no plan changes; their
  fixtures (appendix above) are candidates for the regression-lock unit tests referenced throughout Phase
  4.1's "synthetic fixture" work, so that a future refactor cannot silently flip a sign, drop a
  `mirror_facet`, or change an arc direction without a failing test.

---

## Phase 0.A — partial-patch validation (2026-08-19) — ACCEPTED, landed as slice 1

Patch under test: ROI spatial-sort index compaction (points outside the ROI no longer alias
index 0), empty-ROI hard error, `viewsInfo`/`cell2Cam`/`cell2End` allocation gated on
`bUseFreeSpaceSupport`, plus a `PipelineTest` ROI preflight (first point deliberately outside
the ROI — the exact case the old code got wrong).

| Check | Result | Evidence |
|---|---|---|
| viewsInfo read-gating | PASS | both unguarded reads (`:1080`, `:1091`) sit inside the single `if (bUseFreeSpaceSupport)` block (`:1050`–`:1116`); sole allocation site (`:975`) gated by the same flag |
| WSS-off exact-result | PASS | SceauxCastle `scene_dense.mvs`, `--max-threads 1`: baseline vs patched raw+cleaned PLY md5-identical (11948/23877 raw → 11975/23946 cleaned) |
| WSS-on exact-result | PASS | md5-identical (11922/23819 → 11946/23888) |
| Peak RSS, WSS off | PASS | −2.58 % (56.72→55.26 MB); replicated 4-vs-6 runs, inter-cluster gap 1.27 MB vs ≤0.26 MB noise |
| MVSPipelineTest | PASS | exit 0 (~3–4 s: 4-image synthetic scene, RTX 4070); independently re-run by the reviewer; ROI preflight is unconditional in `PipelineTest`, so exit 0 proves it passed |
| CommonUnitTests | PASS | exit 0 |

Notes: the mesh-stage equivalence was also predicted statically — with ROI off the compacted
index list is `[0..N-1]`, identical to the old pre-sized vector, and the gated writes are
consumed only by WSS code. ROI-on results intentionally differ (bug fix), covered by the new
preflight instead of an A/B.

## Phase 0.B — runtime soundness

**Instrumentation landed (2026-08-19, slice 2):** aggregate ray-walk accounting (18 counters,
per-thread slots, zero-behavior-change — WSS-on and WSS-off outputs verified md5-identical to
the slice-1 binaries) reported at `DEBUG_ULTIMATE`, with a `DEBUG_EXTRA` warning when bad-ends
occur; `M` (scene coordinate magnitude) and `L` (median edge length) printed on the same line
for the item-8 magnitude-correlation analysis. Also landed: optional deterministic seed for
`Mesh::SamplePoints` (`--sample-mesh-seed`, sentinel `NO_ID` = legacy random_device), verified
reproducible (same seed ⇒ identical PLY, different seed ⇒ different) — removes sampler noise
from every Phase 1+ mesh-F1 measurement.

First readout (SceauxCastle sample, WSS on, single-threaded, `M=115.9`, `L=0.119`):
72 542 camera walks (41 dropped, 0 aborted), 2 812 618 steps, **0 bad-ends** — the walk is
healthy in this comfortable-magnitude regime. WSS: 5128 fired, of which **922 (18 %) were
`t==0` no-ops and 1648 (32 %) saturated to non-finite** — half of all reinforcements are
degenerate, empirically confirming the item-7 verdict (structural no-op + per-view product
divergence) on the first scene measured.

**Unit tests landed (slice 3):** empty-ROI and 3-point degenerate inputs fail cleanly (new
`dimension() < 3` guard after insertion — previously undefined behavior downstream), empty
`pointWeights` reconstructs equivalently to explicit unit weights (1 % face-count tolerance;
exact equality is not assertable under process-wide OpenMP float ordering), `pointWeights`
round-trip bit-exactly through the interface archive, and seeded `SamplePoints` is
reproducible (seed 42 twice identical, seed 7 diverges). Amusing regression during
development: the empty-ROI test OBB was first placed at 1e6 with a 0.01 extent — which
collapses to zero in float (ulp 0.0625 at 1e6), the exact item-8 quantization mode.

**Solver cross-check (2026-08-19) — PASS, solver-agnostic confirmed.** With
`DELAUNAY_MAXFLOW_IBFS` commented out (Boost `boykov_kolmogorov_max_flow` fallback,
compiled clean — not bit-rotted), SceauxCastle single-thread WSS-on and WSS-off meshes are
**byte-identical** to the IBFS references (all 4 PLYs, raw + cleaned). Flow values: WSS-off
18878.6 vs 18878.4 (1e-5 relative, float summation order), WSS-on exact at printed precision;
vertex/face counts exact. The graph construction is therefore solver-independent — any future
solver swap (Phase 3.3) is validated by this same harness. Directional timing context: the BK
fallback's solve took 142–149 ms vs IBFS 59 ms on this scene (~2.4× slower), consistent with
the TPAMI-2022 tier ranking. IBFS define restored and verified (rebuilt binary reproduces the
reference md5).

**Single-thread vs OpenMP mesh comparison (2026-08-19) — PASS on SceauxCastle.** Default
multithreaded runs (WSS on and off) produced meshes **byte-identical** to the single-thread
references, all 4 PLYs. Caveat kept honest: atomic float capacity accumulation is
order-dependent in principle, so this shows no divergence on this scene/run rather than
proving determinism in general; the Phase 1 noise floor (0.0006 F1) already bounds any
practical effect on large scenes.

**Fixture tests landed (2026-08-19, slice 8).** Appendix Fixtures A and B now run inside
`MVSPipelineTest` as cut-topology locks: Fixture A (bipyramid) deterministically extracts
exactly the one face behind the observed point that carries the orphaned `D_in` vote — a
regression dropping or relocating that vote (item 3.1) changes the result; Fixture B (star of
4 tetrahedra) locks its analytically expected empty mesh and full-pipeline determinism, with
in-code comments documenting that it cannot discriminate a `mirror_facet` flip (both arc
placements yield the same observable result). Two findings from building them: CGAL's
`is_infinite(cell,i)` only excludes wing-facets of the infinite cell itself (hull-boundary
facets are mesh-eligible), and the in-tree IBFS resolves disconnected zero-capacity cells to
the free side but an unreachable cell with nonzero local `t` to the full side — the same
free-node convention subtlety the EIBFS wrapper had to pin explicitly.

Closed without a test: the WSS micro-trace. The `t *= epsAbs` enforcement runs on internal
`infoCells` state with no public observation point, and a test that re-derives the formula in
its own arithmetic pins nothing about the library (one such tautological test was written and
rejected in review). The empirical walk counters (18 %/32 % no-op/saturation readout above)
are the actual regression signal until Phase 4.1's semantics arms add a real hook as part of
their instrumentation. Phase 0.B is otherwise complete.

## Phase 1 — two-stage benchmark + noise floor (2026-08-19)

Harness: `bench/run_mesh.py` (gitignored bench/) — frozen `runMetashape/scene_dense.mvs` per
scene, per-variant ReconstructMesh args, two-stage scoring (cloud F1 cached once per frozen
cloud; mesh F1 from seeded area-uniform sampling → the official T&T evaluator at scene τ),
per-stage times, peak RSS, and the slice-2 ray-walk counters parsed per run.

**Methodology defect found and fixed before any comparison ran**: the frozen scenes are
unbounded, so the full-scene mesh spans hundreds of units while the T&T evaluation crop is a
few units — a fixed 10 M area-uniform budget starves the crop (mesh recall 0.3–9 % on
Ignatius/Truck/Barn, pure dilution). Fix (bench-side only): per-scene refined scene→GT
alignment captured once from the evaluator's own output during the cached cloud eval; in-crop
face mask replicating Open3D `SelectionPolygonVolume::CropInPolygon` exactly (cross-validated
against open3d 0.19); seeded numpy sampling spends the whole budget on the in-crop submesh.
Coordinates stay in the scene frame end-to-end (no double alignment).

**Noise floor (paired identical baseline runs, 4 scenes): max |ΔF1| = 0.0006** (Meetingroom;
Truck 0.0000, Barn 0.0002, Ignatius 0.0001). Raw face counts and max-flow are bit-identical
across paired runs — reconstruction is deterministic at fixed thread count; the residual F1
jitter is evaluator-side. Every future energy/filter delta is judged against 0.0006 + the
§8 gate (+0.003).

**Two-stage finding**: the mesh stage *loses* large fidelity vs its input cloud on
Truck (mesh 0.357 vs cloud 0.706) and Ignatius (0.329 vs 0.738), is near-parity on Barn
(0.558 vs 0.599), and *beats* the cloud on Meetingroom (0.338 vs 0.323). The mesh−cloud gap
on Truck/Ignatius is now a first-class open question (candidate causes: default clean/smooth
steps, crop-boundary faces, per-scene τ interaction) — queue a raw-vs-cleaned scoring arm
with the Phase 3 work.

## Phase 2 — confidence ablation (2026-08-19) — REJECTED, default unchanged

2×2 matrix on the frozen clouds (adjusted #1292 confidence provenance), 4 scenes, mesh-F1
deltas vs baseline mean (noise floor 0.0006, gate ±0.003):

| Scene | weighted | fss | weighted-fss |
|---|---|---|---|
| Truck | **+0.053** | −0.060 | +0.018 |
| Barn | **−0.293** | −0.021 | −0.295 |
| Ignatius | −0.010 | −0.039 | −0.076 |
| Meetingroom | **−0.177** | −0.025 | −0.232 |

**Decision (per the Phase 2 rule): the app default stays `--constant-weight=true`.**
Adjusted-provenance weighting fails the no-regression gate by two orders of magnitude on
Barn/Meetingroom; `fss` alone is consistently mildly negative; the combination inherits the
worst of both. The raw-NCC pairing arm is moot for the default question (adjusted already
fails) but stays available for Phase 4.1 diagnosis.

**Mechanism (hypothesis, to be tested in Phase 4.1, not asserted)**: enabling weights shrinks
every data-term capacity by the mean confidence (~0.3–0.7) while the quality term `q` and the
camera `kInf` constraints keep their unit-vote-calibrated scale — the relative energy balance
tilts toward the smoothness/quality term and the cut collapses inward (raw faces −90 % Barn,
−80 % Meetingroom). Testable prediction: co-scaling `--quality-factor` by the mean confidence
should restore much of the weighted arm's parity; that is a cheap early Phase 4.1 arm. Truck
(+0.053 with weights) shows the signal itself carries real information once the balance is
right — the consumer, not the signal, is miscalibrated (consistent with § 2 note 1).

## Phase 3.2 — inert `nItersFixNonManifold` removal (2026-08-19) — ACCEPTED, landed as slice 6

`Scene::ReconstructMesh` accepted `nItersFixNonManifold` but called `Mesh::FixNonManifold`
exactly once regardless; a single pass is provably exhaustive (each vertex splits into one
duplicate per incident manifold component; splitting never alters another vertex's
incident-face set). Removed from declaration, definition, and both call sites passing the
hardcoded `4` (ReconstructMesh app, Viewer — the latter a call site the initial audit missed).
Gates: SceauxCastle WSS-on/off meshes md5-identical to the slice-2 references (raw and
cleaned, 4/4), both test suites green. API hazard recorded in the commit message: an external
caller passing a 4th positional argument intended as the iteration count now silently binds it
to `kSigma` (in-tree `PythonWrapper` passes 3 args, unaffected).

## Phase 3.3 — graph-cut sub-stage split (2026-08-19) — instrumentation landed as slice 7; solve dominates

The `graphcut_s` stage (largest mesh stage on Truck) is now split by three `TD_TIMER` scopes
reporting at `DEBUG_ULTIMATE` as `graph construction` / `graph-cut` / `surface extraction`
(names matched by `bench/run_mesh.py`'s substage columns). Exact-result verified: meshes
md5-identical, sub-times sum to the stage total.

First readout, default CLI settings, multithreaded (the three phases are all serial, so the
split is thread-independent):

| scene | graph build | max-flow solve | surface extraction | stage total |
|---|---|---|---|---|
| SceauxCastle (11 img) | 22 ms | 63 ms (68 %) | 7 ms | 100 ms |
| Truck (251 img, 22.7 M cells) | 5.83 s | 28.45 s (**75 %**) | 2.47 s | 38.0 s |

**Verdict: the IBFS solve is the Phase 3.3 lever** — 75 % of the stage, ~26 % of the whole
1m49s Truck mesh reconstruction. A 2× solver speedup would clear the ≥5 % exact-result gate
with wide margin; next step is the EIBFS candidate (TPAMI-2022 review) license check per §0.D,
then an A/B behind the existing `DELAUNAY_MAXFLOW_IBFS` switch pattern. Graph build (15 %) and
extraction (6.5 %) are not worth touching.

Bad-end telemetry from the same run: 7 walks aborted / 5 rays dropped out of 4.07 G steps
(0.0000 %) with `M=172` — the slice-2 warning path works and confirms ray-walk failures are
negligible on a healthy scene (SceauxCastle had exactly 0).

### Solver-candidate license check (2026-08-19) — no license-clean fast solver exists

Per §0.D, licenses verified by fetching the actual license files/headers (not repo README
claims), against the TPAMI-2022 review (arXiv:2202.00418) and its code repo
(`github.com/patmjen/maxflow_algorithms`):

* **EIBFS-I / EIBFS-I-NR** — the strongest one-shot serial performers on the instance families
  closest to our Delaunay-tetrahedra graph (wins "Mesh segmentation" RP 0.95, "3D separated
  surfaces" 0.92, stereo ~1.0; the "-I" means *index-based*, not incremental — the speed win
  does not depend on warm-start reuse we don't have). BUT: despite sitting in the repo's
  MIT-labeled `reimpls/` folder, their source headers carry the **unmodified TAU
  "research purposes only"** license (Kaplan/Hed) — the same license as the IBFS already in
  `libs/Math/IBFS/`. The folder-level MIT labeling is that repo's error, not a relicense.
* **HPF** (highest mean RP overall, wins the "Multi-view" family — which is voxel
  photo-consistency, a weaker analog than mesh segmentation): UC Regents non-commercial,
  "not an open source license". No freed successor found.
* **MBK (`reimpls/mbk.h`)** — verified genuinely clean-room MIT, and Boost's
  `boykov_kolmogorov_max_flow` is BSL-1.0 — but the BK family is the paper's bottom serial
  tier (mean RP ≈ 0.27, worst cases 1000×+ slower); both would likely be a speed *regression*
  vs IBFS.
* No permissively-licensed EIBFS implementation exists anywhere findable; the BK-family
  GitHub forks are all Kolmogorov-lineage.

Consequences for Phase 3.3: (a) an EIBFS-I A/B is **license-neutral** relative to the status
quo (same research-only class as the in-tree IBFS) and remains justified by the 75 %-of-stage
solve share — but it cannot *improve* the licensing story; (b) the only path to speed *plus* a
real license fix is asking the TAU authors (or the DTU review authors, who already negotiated
derivative rights once) for explicit permission, or a clean-room EIBFS reimplementation from
the ESA papers (the algorithm is not copyrightable; nobody has done this yet); (c) vendoring
`eibfs_i.h` into the tree is a maintainer decision — the A/B result should be produced first
without committing the vendored solver.

### EIBFS A/B (2026-08-19) — NEGATIVE, solver swap closed; no code committed

Measured via an uncommitted third `MaxFlow` wrapper branch plus a standalone dump-replay
harness (the production Truck graph — 22 722 599 nodes, 45 445 198 edges with capacities —
dumped from the wrapper and replayed against each solver in isolation; harness + experiment
diff + graph dump preserved in the session scratchpad under `eibfs/`).

* **Correctness first**: on SceauxCastle the EIBFS-I wrapper (with `freeNodeValue=1` to match
  the in-tree convention that unreached nodes land on the source side — the two
  implementations *differ* in this default) reproduced all 4 reference meshes byte-identically.
* **`reimpls/eibfs_i.h` (EIBFS-I) crashes on the Truck graph**: access violation in
  `augmentExcesses<false>` (excess-bucket pop, `eibfs_i.h:1186`) after a clean `initGraph`;
  independent of ArcIdx width (uint32 and uint64 both crash) and not caused by bad capacities
  (0 non-finite/negative capacities counted at AddNode/AddEdge — incidentally a clean item-10
  ISFINITE readout for Truck). A separate latent defect was found reading the code: bucket
  `allocate()` doubles capacity once per call instead of until sufficient. Upstream repo issue
  not filed (maintainer's call).
* **`reimpls/eibfs_i_nr.h` (EIBFS-I-NR) works but is not faster**, interleaved ×2 on the
  identical graph: IBFS init 1.23–1.29 s / solve 26.2–27.0 s; EIBFS-I-NR init 0.07–0.08 s /
  solve 27.1–27.7 s. Total solver time is a wash. The TPAMI-2022 "Mesh segmentation" advantage
  does **not** transfer to Delaunay-visibility graphs.
* Side observation: the in-tree IBFS accumulates its reported `flow` in `float`, which at
  Truck's ~8.1 M flow magnitude rounds away sub-unit increments — EIBFS-I-NR (double
  accumulation) reports 8 312 066 vs IBFS 8 121 759 on the identical graph (+2.3 %). The flow
  value is log-only (the cut itself is what the mesh consumes), so this is cosmetic; worth
  knowing when comparing flow values across solvers or scenes.

**Phase 3.3 verdict: keep IBFS.** No faster licensable solver exists (license check above),
and the best available EIBFS implementation is speed-neutral here while its stronger sibling
crashes. The remaining 3.x speed levers are the graph-build (15 %) and extraction (6.5 %)
stages — both previously judged not worth touching — so Phase 3 closes with 3.1 (weighting
deserialization) as its only landed speed win, and Phase 3.4 (coordinate normalization)
remains a *robustness* item, not a speed item. The experiment edits were fully reverted; the
restored binary reproduces the SceauxCastle reference md5.

## Phase 5.0 — fusion drop accounting (2026-08-19) — instrumentation landed as slice 7; first readout

`DepthMapsData::DenseFuseDepthMaps` (the `--fusion-filter 2` default path) now prints at
verbosity 3: per-probe rejection reasons (outside, no-depth, already-fused, low-confidence,
depth-diff with free-space-violation subcount, reprojection-error, normal-diff), per-pixel
lifetime fates (admitted, low-confidence, min-pixels, min-views, violation), and 10-bin
confidence histograms of admitted vs dropped pixels — all in added `else` branches with
self-checking residuals; fusion logic untouched (verified line-by-line, plus md5-equal outputs
in the pipeline test).

First readout (pipeline-test fixture, 4 depth maps): residuals 0 on both invariants;
**47.95 % of valid depths never fuse** — 27.5 % dropped for low confidence, 20.4 % for
min-pixels; the dropped-pixel confidence histogram is dominated by the `[0,0.1)` bin (312 k of
545 k), i.e. most of the dropped mass is *low*-confidence, but the `[0.1,0.7)` bins still hold
~228 k pixels the mesh stage never sees. The go/no-go measurement for Phase 5.1 (carve-only
rays from unfused high-confidence pixels) is the same accounting on real T&T scenes.

### T&T readout (2026-08-19) — **GO for Phase 5.1**

Full re-densification per scene (frozen-cloud settings `--resolution-level 1
--number-views 12 --estimate-roi 0 --crop-to-roi 0 --tower-mode 0`; the frozen clouds' cached
dmaps no longer existed), run in fresh `<scene>/runFusionStats/` folders whose dmaps are
**kept** for fast fusion-only 5.1 iteration (~40 s vs a full densify). All runs: both
accounting residuals 0, min-views 0.

| scene (imgs) | valid depths | admitted | dropped | low-conf | min-pixels | violation | dropped @ conf ≥ 0.1 | dropped @ conf ≥ 0.7 (vs admitted) |
|---|---|---|---|---|---|---|---|---|
| Ignatius (263) | 132.2 M | 48.0 % | 52.0 % | 26.4 % | 25.4 % | 0.20 % | 33.9 M | 10.0 M (15.8 %) |
| Meetingroom (371) | 175.1 M | 30.1 % | **69.9 %** | 34.6 % | 35.0 % | 0.25 % | **61.7 M** | 8.9 M (17.0 %) |
| Truck (251) | 126.5 M | 56.0 % | 44.0 % | 23.6 % | 20.3 % | 0.15 % | 25.8 M | 8.7 M (12.3 %) |

Findings:
1. **The low-confidence channel is exactly the `[0,0.1)` histogram bin in every scene** — the
   threshold separates cleanly; everything dropped above 0.1 confidence is min-pixels or
   violation mass, i.e. *geometrically consistent depth that simply failed to cluster*.
2. **min-pixels (cluster < 5 px) is the dominant recoverable channel**: 20–35 % of all valid
   depths, 20–50 M dropped clusters per scene. Every scene throws away ~9–10 M pixels at
   confidence ≥ 0.7 — 12–17 % of the admitted mass.
3. Meetingroom — the recall-starved indoor scene — discards more than twice what it keeps;
   its dropped ≥ 0.1-confidence mass (61.7 M) exceeds the admitted mass (52.8 M). The
   correlation with the mesh-vs-cloud fidelity gap is suggestive, not yet causal evidence.
4. Probe-level (context): already-fused dominates rejections (~60 %), outside 15–22 %;
   free-space violations are rare (~0.1 % of probes) but nonzero everywhere.

Verdict: the preconditions for Phase 5.1 hold — a large, high-confidence, geometrically
consistent pixel population never reaches the mesh stage. Next: feed these unfused pixels as
carve-only rays (free-space evidence without surface points) and A/B mesh F1 on the two-stage
bench. Caveat: the re-densified clouds differ *substantially* from the frozen bench inputs —
9.4 M vs 5.2 M points (Ignatius), 8.8 M vs 3.1 M (Meetingroom), 9.4 M vs 6.0 M (Truck). The
frozen clouds were produced in June by older code with a `Densify.ini` in the working folder
overriding defaults (reprojection threshold 1.2 vs today's 1.0, geometric weight 0.1 vs 0.3,
pre-recalibration confidence lineage); the readout above uses current defaults, which is the
correct basis for 5.1 (that is the pipeline 5.1 modifies) but means: (a) 5.1 A/Bs starting
from these dmaps need their own freshly-measured baseline row, never the frozen-cloud rows;
(b) the per-channel percentages would shift some under the June settings (looser reprojection
grows clusters and shrinks the min-pixels channel), though not enough to flip the verdict.

## Phase 5.1 — carve-only rays (2026-08-19) — implemented (slice 9); A/B borderline, gate arms running

Implementation (`39859e76`): `DensifyPointCloud --export-unfused-file` writes a 20-byte-record
sidecar of the pixels that joined a cluster fusion then dropped (min-pixels / min-views /
violation fates; the low-confidence fate never reaches a cluster), gated at confidence ≥ 0.5
and capped at 8 M records by a deterministic power-of-two stride (`{i : i mod stride == 0}`).
`ReconstructMesh --carve-rays-file` replays each record as a pure free-space ray: the
camera→point segment is walked with the t-edge loop's facet stepper (`intersectFace` — a ray
with no target vertex needs no bad-end path) and adds the standard distance-weighted α_vis
scaled by the pixel's confidence to the crossed directed facets; no vertex, no unary term.
The pass runs after both weighting loops, so **the WSS classifier still reads the fused points
alone** — β/γ are computed before carve capacity lands; feeding it is a deliberate one-line
later arm. (Also learned: `viewsInfo` is a walk-restart memo, not evidence — what WSS reads is
`freeSpaceSupport()` = incoming `f` sums, which carve rays do feed, just after the fact.)
Off-state proven exact (4/4 reference md5s, byte-identical fusion from identical dmaps, OMP
and serial carve passes md5-identical). Known asymmetry: at default `--constant-weight 1`
fused votes carry weight 1 while carve rays keep their ≤1 confidence.

Sidecars (fusion rerun on the kept dmaps): Ignatius 4.75 M records of 19.0 M candidates
(stride 4), Meetingroom 7.31 M of 29.3 M (stride 4), Truck 7.43 M of 14.9 M (stride 2).

First A/B (fresh `@runFusionStats` baselines, single runs, python sampler, seed 42, noise
floor 0.0006 from the frozen pipeline):

| scene | baseline | carve | Δ | fss | carve-fss | Δ vs fss |
|---|---|---|---|---|---|---|
| Ignatius | 0.3348 | 0.3334 | −0.0014 | 0.2723 | 0.2726 | +0.0003 |
| Meetingroom | 0.3566 | 0.3588 | +0.0022 | 0.3289 | 0.3312 | +0.0023 |
| Truck | 0.3511 | **0.3544** | **+0.0033** | 0.2778 | 0.2808 | +0.0030 |

Reading: consistent small positive on the two scenes where fusion drops the most, tiny
negative on Ignatius; only Truck touches the §8 ≥+0.003 gate, no scene violates the −0.003
clause.

The conf ≥ 0.7 arm (offline filter of the same records to 2.96 M / 3.52 M / 4.99 M rays)
tested whether the 0.5–0.7 band drives Ignatius' regression: Ignatius −0.0007, Meetingroom
+0.0011, Truck +0.0025 vs baseline. Both the gains and the regression shrink roughly
proportionally with ray mass — the band is not selectively harmful, the whole effect just
scales. The full-gate (0.5) variant dominates.

**Verdict: carve-only rays stay OPT-IN (`--carve-rays-file`), not default.** The dropped
evidence matters — the sign pattern is consistent across both gate settings and strongest
exactly where fusion drops the most — but at +0.0033 best-scene it does not clear the §8
default gate under the *current* energy. The same table shows why: `fss` loses 0.03–0.07 F1
on these clouds, i.e. the energy's absolute-scale constants are miscalibrated for dense
inputs, and a sub-0.003 evidence-injection effect cannot be judged defaultable inside a
miscalibrated energy. Re-test the carve arm as a Phase 4.1 companion after recalibration (the
plan itself pairs 5.1 with 4.1); Phase 5.2 (bypass fusion entirely) stays parked on this
evidence — the marginal value of dropped pixels does not justify a full input-model change
today. Repeat-run significance tightening was considered and skipped: no plausible repeat
outcome changes the opt-in call.

Side findings from the same table: (a) **`fss` collapses on these denser clouds** — −0.06 to
−0.07 F1 on Ignatius/Truck vs the frozen-cloud "mildly negative" — consistent with the audit's
unit-dependence verdict on kAbs/kOutl (absolute-scale constants meeting 1.6–2.8× more α
mass); Phase 4.1's recalibration case just got stronger. (b) The mesh-vs-cloud fidelity gap
persists on dense clouds (Ignatius cloud 0.773 → mesh 0.335; Truck 0.716 → 0.351;
Meetingroom 0.437 → 0.357) — still an open investigation.

## Phase 4.1 — WSS calibration & enforcement switches (2026-08-19) — implemented (slice 10); benches pending

Implementation (`12811a32`): the five graph-cut free-space-support constants are on the
ReconstructMesh CLI (`--support-factor` kb=4, `--front-factor` kf=3, `--relative-factor`
kRel=0.1, `--absolute-factor` kAbs=1000, `--outlier-factor` kOutl=400 — defaults equal to the
Scene.h declarations), and the previous two-call ternary (default path vs carve path
respelling the constants) collapsed to a single call site; the camera-cell capacity is now
`Scene::kInfCapacity`, spelled once. Two genuinely new switches live in a trailing defaulted
`ReconstructMeshParams` (so PythonWrapper/Viewer/Tests call sites compile unchanged —
`cl /Zs`-verified): `--wss-semantics product|paper|add|max` and `--quality-co-scale`.

Enforcement arms, at the classifier's firing branch (`epsRel<kRel && epsAbs>kAbs &&
gamma<kOutl`, target = the ~4σ walk-end cell, classifier untouched in every arm):

| arm | update | nature |
|---|---|---|
| `product` | `t *= εabs` per firing (vertex,view) pair | shipped behavior, byte-exact default |
| `paper` | `E(c) += εabs` per pair, then one `t *= E(c)` per cell (serial pass, only where `E>0`) | ISRN-2014 Eq. 8 fidelity restoration |
| `add` | `t += kw·εabs` per pair (kw=1) | departure: drops the multiply's α² unit defect |
| `max` | `t = max(t, kw·εabs)` per pair (named critical — no atomic max) | departure, order-independent |

The item-7 fixture rows (F7.1/F7.2, appendix) are reproduced exactly by all four arms
(product 4 500 000 / paper 6 750 / add 2 702.5 / max 1 500; t₀=0 row: 0/0/2700/1500).
The `paper` accumulator is one float per cell, allocated only in that arm (~90 MB at Truck's
22.7 M cells). Counter semantics per arm are documented at `walk_stats_t`: product/add/max
count no-op/saturated per firing pair, paper per target cell; add/max cannot leave `t==0`
by construction.

Validation (frozen SceauxCastle, WSS on, single thread; **independently re-run by the
reviewer from a fresh build**): default byte-exact — all 4 reference mesh md5s reproduced
(`ec75a12e…`/`e14c9f89…`/`376635fb…`/`a473f42a…`); all-options-at-defaults run identical;
both ctest suites green; unknown semantics rejected at Initialize.

| arm | raw V/F | max-flow | fired | t==0 no-op | saturated |
|---|---|---|---|---|---|
| `product` | 11920/23819 | 5.37e+08 | 5128 | 922 (pairs) | **1648** (pairs) |
| `paper` | 11920/23821 | 6.68e+06 | 5128 | 226 (cells) | **0** |
| `add` | 11436/22865 | 4.73e+05 | 5128 | 0 | 0 |
| `max` | 11436/22865 | 8.53e+04 | 5128 | 0 | 0 |

Readings: `paper` eliminates the saturation channel entirely (1648 → 0) and drops the flow by
two orders of magnitude — the geometric per-view divergence is confirmed to be the whole
saturation story. The structural `t==0` no-op survives in `paper` (226 cells) as predicted —
base t at ~1σ, enforcement at ~4σ, different cells. `add` and `max` produce different graphs
(different flows) but the same cut on this scene: every fired cell lands ≥ kAbs=1000, far
above neighbouring facet capacities. Cut-equality says nothing about scenes with real weak
surfaces — that is what the bench arms are for.

Quality co-scaling (Phase 2 mechanism arm): scales kQual by the mean of exactly the per-view
confidences InsertViews consumed (over the inserted points), logged at DEBUG_EXTRA; strict
no-op + VERBOSE warning when the cloud carries no weights (md5-proven on two inputs). On the
re-fused SceauxCastle (mean conf 0.627): weighted 50 956 raw vertices → co-scaled 51 476 vs
constant-weight 51 558 — co-scaling moves the weighted result back toward the constant-weight
one, the direction the Phase 2 mechanism hypothesis predicts (one small scene, no F1 claim).

Known cosmetic pre-existing issue recorded: the insertion block's local `lt` shadows
`DEFINE_LOG_NAME(lt, …)`, mis-tagging log lines emitted from that scope (e.g. "Delaunay
tetrahedralization completed" prints as `[ImgCache]`); the co-scale DEBUG_EXTRA inherits it.
Fixing the shadowing would change existing log output — deferred.

Bench sequence (read each before launching the next; no builds while benches run):
(C) quality co-scaling on the frozen clouds where Phase 2 weighted rows exist — tests the
Barn −0.293 mechanism hypothesis; (A) kAbs/kOutl recalibration sweep at fss=1 on the
`@runFusionStats` clouds — the fss collapse is the sharpest signal; (B) semantics arms at
default constants; then the carve companion re-test (Phase 5.1 verdict).

### Arm C — quality co-scaling (frozen clouds, tag `phase41-qscale`) — MECHANISM CONFIRMED

Paired single runs, same seed/sampler as Phase 2; the re-run `weighted` rows reproduce the
Phase 2 rows to ≤0.0002 (noise floor 0.0006) — no binary drift on the weighted path across
slices 5–10.

| scene | baseline mean | weighted | weighted-qscale | qscale vs weighted | qscale vs baseline |
|---|---|---|---|---|---|
| Barn | 0.5576 | 0.2648 | 0.5107 | **+0.246** | −0.047 |
| Meetingroom | 0.3379 | 0.1606 | 0.3305 | **+0.170** | −0.007 |
| Ignatius | 0.3295 | 0.3190 | 0.3425 | +0.024 | **+0.013** |
| Truck | 0.3569 | 0.4094 | 0.4274 | +0.018 | **+0.071** |

**The Phase 2 mechanism hypothesis is confirmed.** One global scalar — kQual × mean point
confidence — recovers 84 % of Barn's collapse (−0.293 → −0.047) and 96 % of Meetingroom's
(−0.177 → −0.007), and where weighting already helped it helps more (Truck +0.053 → +0.071,
now with Ignatius positive too). The collapse signature reverses accordingly: Barn raw faces
grow 0.75 M → 6.6 M (the inward cut un-collapses), Meetingroom's mesh F1 lands *above* its
cloud F1 (+0.008 d), and recon time drops (Meetingroom 135 s → 68 s) — the solver stops
fighting a mis-balanced energy.

**Not defaultable as-is**: Barn −0.047 and Meetingroom −0.007 still violate the §8
no-regression clause vs the constant-weight baseline, so `--constant-weight 1` stays the
default. But the diagnosis is now precise: the confidence *signal* was never the problem —
the quality term's unit-vote calibration was, and the mean-confidence co-scale is only a
first-order correction (the camera kInf links and the WSS absolute constants also keep unit
scale, and a global mean cannot fix a spatially varying confidence deficit). Follow-ups
recorded, not run: co-scale exponent sweep, per-region co-scale — deferred until after arms
A/B.

### Arm A — kAbs/kOutl proportional rescale at fss=1 (dense clouds, tag `phase41-kabs`) — REFUTED

Hypothesis under test: the fss collapse on the dense `@runFusionStats` clouds (−0.03..−0.07
vs no-fss baseline, Phase 5.1) is a units artifact — kAbs=1000/kOutl=400 are absolute-scale
constants meeting 1.6–2.8× more α mass — fixable by scaling both proportionally. Sweep:
x05 = 500/200, x1 = 1000/400 (Phase 5.1 rows), x2 = 2000/800, x4 = 4000/1600; single runs,
same seed/sampler; WSS firing counters from the kept run logs (`fired` = enforcement
(vertex,view) pairs, `sat%` = fraction that saturated the t edge).

| scene | baseline | x05 | x1 | x2 | x4 | best vs baseline |
|---|---|---|---|---|---|---|
| Ignatius | 0.3348 | **0.3011** | 0.2723 | 0.2514 | 0.2303 | −0.034 (x05) |
| Meetingroom | 0.3566 | 0.3288 | 0.3289 | 0.3315 | **0.3401** | −0.017 (x4) |
| Truck | 0.3511 | **0.3020** | 0.2778 | 0.2767 | 0.2738 | −0.049 (x05) |

| scene | fired x05/x1/x2/x4 (M) | sat% x05/x1/x2/x4 |
|---|---|---|
| Ignatius | 18.6 / 20.2 / 19.7 / 17.0 | 82 / 86 / 87 / 88 |
| Meetingroom | 13.8 / 10.1 / 7.0 / 4.5 | 43 / 44 / 44 / 44 |
| Truck | 21.7 / 20.7 / 18.4 / 15.5 | 63 / 65 / 66 / 67 |

**Refuted.** All 9 rows stay below the no-fss baseline, and the preferred direction is
scene-inconsistent (outdoor Ignatius/Truck best at x05, indoor Meetingroom monotone toward
x4) — no global rescale exists. The counters say why the sweep is impotent: on
Ignatius/Truck an 8× sweep of both thresholds moves the firing volume by <25 % (17–22 M),
i.e. the firing population sits at extreme epsAbs (≫4000) and near-zero γ (≪200) — deep
free space far from the classifier's decision boundary at *any* of these settings. And with
product semantics each firing multiplies t by epsAbs itself (thousands), so 43–88 % of
firings saturate the edge at every setting: enforcement is effectively a binary cell-nuke,
and the constants only choose *which* cells get nuked, never *how hard*. Meetingroom's
apparent improvement toward x4 is pure dilution (firings 13.8 M → 4.5 M, saturation ratio
frozen at 43–44 %), converging toward the fss-off no-op, not a recalibration win.

Verdict: the collapse is not a units-calibration artifact; the unbounded product semantics
is the prime suspect. That is exactly what arm B isolates — paper/add/max at default
constants, where the paper arm already eliminated saturation on the F7 fixture (1648 → 0)
and cut flow 80× on SceauxCastle.

### Arm B — enforcement semantics at default constants (dense clouds, tag `phase41-sem`) — NEGATIVE; the t==0 no-op is protective

Same three clouds, single runs; classifier unchanged, so all arms fire on the identical
(vertex,view) pair set (e.g. Ignatius 20 163 406 in all four semantics).

| scene | baseline | product | paper | add | max |
|---|---|---|---|---|---|
| Ignatius | 0.3348 | 0.2723 | 0.2722 | 0.0453 | 0.0437 |
| Meetingroom | 0.3566 | 0.3289 | 0.3286 | 0.3001 | 0.3017 |
| Truck | 0.3511 | 0.2778 | 0.2782 | 0.2220 | **0.3054** |

Three clean findings:

1. **paper ≡ product on dense clouds.** Ignatius and Meetingroom produce *identical cuts*
   (Ignatius: same 3 053 611 vertices / 6 126 586 faces, flow 1.59654e7 vs 1.59649e7 —
   δ 0.003 % in t-edge magnitudes below capacity, same argmin); Truck is near-identical
   (6 254 463 vs 6 253 471 clean faces, F1 δ +0.0004). With epsAbs at absolute α scale
   (arm A: the firing population sits at epsAbs ≫ 4000), a *single* multiplicative
   enforcement is already effectively infinite — ISRN-2014's per-cell-sum vs the shipped
   per-pair-product only matters at fixture scale, where it did change the mesh md5. Paper's
   saturation count is 0 everywhere, yet the cut doesn't move: saturation was a symptom, not
   the mechanism.
2. **add − paper isolates the structural t==0 population, and it is the whole catastrophe.**
   For t0 > 0, t0 + Σ epsAbs and t0 × Σ epsAbs are both effectively uncuttable — the only
   functional difference is that add enforces on t0 == 0 cells (counters: t==0 no-op drops
   to 0 in add/max). Result: Ignatius 0.2722 → 0.0453 (in-crop faces 736 K → 350 K), Truck
   0.278 → 0.222, Meetingroom 0.329 → 0.300. Planting surface priors in deep free space
   (the ~4σ walk-end cells that never received base t) destroys the outdoor object scenes.
   The structural no-op — base t at ~1σ behind the point, enforcement targeting the ~4σ
   cell — is *protective*, not a defect; do not "fix" it.
3. **max bounds the magnitude and it shows.** max(t0, epsAbs) caps a cell at a single
   epsAbs instead of a Σ/Π over hundreds of firings: it recovers most of add's Truck damage
   (0.222 → 0.3054 — the best fss variant on Truck, above product) but shares the t==0
   catastrophe on Ignatius (0.0437). Magnitude bounding helps; free-space enforcement kills.

### Phase 4.1 close-out — all arms measured, defaults unchanged

Combined verdict (arms A, B, C): no constant rescale, no enforcement semantics, and no
global quality co-scale rescues free-space-support on dense clouds or makes point-weighting
defaultable. Every shipped default stays: `--free-space-support 0`, `--constant-weight 1`,
product semantics, kRel/kAbs/kOutl as-is, co-scale off. The switches remain as cheap,
tested diagnostics.

The Phase 5.1 carve companion re-test is **moot as specified**: it was conditioned on a
recalibrated energy, and no recalibration was adopted — the opt-in verdict stands on its
original evidence. The WSS-feeding carve variant is closed on the same grounds: feeding
carve mass to β/γ can only raise γ and shrink the firing set, and arm A showed pure
dilution converges to fss-off without ever recovering baseline — its value is bounded by
"fss off", not worth a bench.

Recorded for a possible later phase, not scheduled: bounded/relative enforcement (cap the
enforcement at s-side scale — every tested arm is effectively binary per cell; max's Truck
win is the one hint bounding has value), arm C's co-scale exponent / per-region co-scale.
Next slice: Phase 4.2.

## Phase 4.2 — grazing-incidence down-weighting (2026-08-20) — implemented (slice 11); NEGATIVE for default, stays opt-in

Implementation (`51de49dd`): each crossed-facet vote is scaled by
`max(floor, |cos(ray, facet_normal)|^exp)` at all three walk sites (camera→point,
behind-point σ-extension, carve replay); the end-cell unary and the WSS pass are untouched.
Parameters ride `ReconstructMeshParams` (`--grazing-floor`, default 1 = off;
`--grazing-exponent`, default 1), validated at CLI parse. A hoisted `grazing_t` functor
branches around the factor entirely when off — off-state proven byte-identical (4/4
SceauxCastle reference md5s, agent + independent reviewer runs), all 3 ctest suites green.
Geometry facts verified in source: the facet-plane normal is unit by construction
(`Plane.inl:66` normalizes) and every walk seeds a unit ray direction (the carve walk with
an explicit zero-length guard), so the dot product is the incidence cosine. Degenerate
(zero-area) facets produce a NaN normal, but those already produce a NaN crossing distance
in the existing soft weight — same pre-existing hazard class (0.C item 10), no new entry.

Bench (frozen clouds, tag `phase42-grazing`, single runs; baselines = Phase 2 means):

| scene | baseline | floor 0.5 | floor 0.2 | floor 0.2 exp 2 |
|---|---|---|---|---|
| Barn | 0.5576 | 0.5376 (−0.020) | 0.5032 (−0.054) | 0.4572 (−0.100) |
| Ignatius | 0.3295 | 0.3308 (+0.001) | 0.3341 (+0.005) | 0.3341 (+0.005) |
| Meetingroom | 0.3379 | 0.3184 (−0.020) | 0.2702 (−0.068) | 0.2183 (−0.120) |
| Truck | 0.3569 | 0.3638 (+0.007) | 0.3667 (+0.010) | 0.3554 (−0.001) |

**Verdict: opt-in only; no default at any setting.** The sign splits by scene type and the
dose-response is clean and monotone: the object-centric captures (Ignatius, Truck) gain a
little (+0.001..+0.010), the planar/facade scenes (Barn, Meetingroom) lose 3–10× more, and
losses scale with strength (mesh mass erodes with it — Meetingroom clean faces 2.89 M →
1.54 M from floor 0.5 to the exp-2 arm). This *inverts* the plan's Vis2Mesh expectation
(wins on facades): down-weighting oblique free-space votes while the unary keeps full
strength starves exactly the surfaces that are only ever crossed obliquely — walls and
floors swept by cameras moving parallel to them. No setting can clear §8: at the mildest
arm the Truck gain (+0.0069) is already 3× smaller than the Barn/Meetingroom regressions,
and both scale together toward zero as floor → 1. Interpolating a floor ≈ 0.75 lands Truck
at ≈ +0.003 with Barn still ≈ −0.01 — the gate is unreachable on T&T. A per-scene or
per-region gate (apply only away from planar structures) would be a new mechanism, not a
tuning of this one; not scheduled.

Next slice: Phase 4.3 (per-point adaptive σ; resolve the CLI `--thickness-factor` 1.0 vs
library kSigma 2.0 mismatch deliberately while there).

## Phase 4.3 stage 1 — per-vertex adaptive σ (2026-08-20) — implemented (slice 12); GATE MET, default flip recommended

Implementation (`25f989df`): `--adaptive-sigma` derives σ per vertex as kSigma × median
incident finite-Delaunay-edge length, clamped to [0.25, 4] × the global σ (the global σ is
the same statistic over *all* finite edges, so the local form is its restriction), consumed
in the three roles where σ is the point's own uncertainty — soft-visibility exponent,
end-cell offset, WSS window lengths — and nowhere else (carve replay keeps the global).
The parallel fill uses CGAL's `finite_incident_edges_threadsafe` (the plain traversal
writes shared per-cell/per-vertex marker state — a data race under OMP against the cells
the ray-walks read; caught in review, confirmed race-free by 3 multi-threaded repeats with
byte-identical statistics). Fill cost 3 ms parallel on SceauxCastle, one float per vertex,
allocated only when enabled. Off-state proven byte-identical (4/4 anchors, agent +
independent reviewer); all ctest suites green. σ_v health on SceauxCastle: median
σ_v/σ = 0.881 (expected median-of-medians bias — the per-vertex median de-weights the
long-edge tail owned by sparse/hull vertices), 0.09 % clamped low, 2.51 % high.

Bench (frozen clouds, tags `phase43-sigma` + `phase43-sigma-b`; adaptive n=3 runs/scene,
thickness-088 n=2, thickness-2 n=1; baselines = Phase 1/2 means, per-scene repeat spread
≤ 0.0013):

| scene | baseline | adaptive-σ | Δ | thickness-088 | Δ | thickness-2 | Δ |
|---|---|---|---|---|---|---|---|
| Barn | 0.5576 | 0.5647 | **+0.0071** | 0.5564 | −0.0012 | 0.5552 | −0.0024 |
| Ignatius | 0.3295 | 0.3321–0.3325 | **+0.0027..30** | 0.3286 | −0.0009 | 0.3246 | −0.0049 |
| Meetingroom | 0.3379 | 0.3383–0.3396 | +0.0004..17 | 0.3388 | +0.0009 | 0.3309 | −0.0070 |
| Truck | 0.3569 | 0.3601–0.3602 | **+0.0032..33** | 0.3607 | +0.0038 | 0.3376 | −0.0193 |

**§8 gate: MET.** Barn clears ≥+0.003 with 10× margin, Truck and Ignatius sit at the gate,
Meetingroom is neutral-positive; no scene regresses in any run. Bonus: recon is *faster*
than baseline on all four scenes (−2..−6 %) with 4–10 % denser meshes. The attribution
control (`thickness-088`, a global σ matched to the adaptive median) splits the mechanism:
on Truck/Meetingroom the scalar replicates the win, but on Barn/Ignatius it does not
(−0.001 both) — there the per-vertex adaptation itself carries +0.004..+0.008. So the win
is not reducible to "shrink σ globally by 12 %"; mixed local density inside these scenes
(Barn's structure vs vegetation, Ignatius' statue vs ground) is exactly where local σ
helps, as the plan predicted for mixed-scale content.

**Default-flip recommendation, reserved to the maintainer** (same convention as the
Phase 2 rollout decision): flip `bAdaptiveSigma` to true / `--adaptive-sigma` default 1.
The gate is met with repeats and the change is one line, but it alters every default run's
output, so it ships only with maintainer sign-off.

Side results:
- `adaptive-sigma-fss` vs the fss rows is mixed (Ignatius +0.023, Barn −0.006,
  Meetingroom −0.011, Truck −0.022) — fss stays off-default regardless (Phase 4.1), not
  actionable.
- **kSigma default mismatch resolved on data**: `thickness-2` (the *library* default
  kSigma=2 under the CLI) is strictly worse on all four scenes (−0.002..−0.019). The CLI
  default 1.0 is the right side. Recommendation: align the library default 2.f → 1.f
  (affects PythonWrapper/Viewer/API callers — maintainer sign-off; a follow-up slice
  should validate those callers when flipped).
- Oddity on record: `thickness-088` Barn recon is ~1.5× slower than baseline (229 s vs
  152 s, both repeats) while `adaptive-sigma` Barn is the *fastest* (147 s) — walk cost is
  sensitive to the σ field in a non-obvious way; not investigated.

Stage 2 (shrink σ_v for high-confidence vertices) is a separate A/B per the plan: note the
default path carries no confidence at mesh time (`--constant-weight 1` releases the weights
before insertion), so stage 2 needs plumbing that keeps per-vertex confidence for the σ
consumer without touching vote scale — next slice.

## Phase 4.3 stage 2 — confidence-shrunk σ_v (2026-08-20) — implemented (slice 13); gate NOT met, opt-in retained

Implementation (`1be150d0`): `--sigma-conf-shrink s` scales σ_v by `1 − s·conf_v`, where
conf_v is the mean of the per-view confidences consumed at insertion for the points merged
into the vertex, sharing the single [0.25, 4] × global-σ clamp with the adaptive arm; s
validated in [0,1), logged no-op when the cloud carries no weights. The Phase 2 trap
(weighted votes collapse the cut) is closed by construction: `bConstantVotes` keeps the
votes at unit scale while the weights stay loaded, so confidence reaches σ *only*. Proven
by the epsilon probe — `s = 1e-9` retains the weights, allocates and consumes the
confidence channel (logged conf_v mean 0.611), yet is byte-identical to the plain run with
bit-identical max-flow; and directionally — shrink moves the flow up (+0.5 %) and densifies
the mesh, the opposite of the weighted-vote regime (−5.1 %, sparser). Off-state and
`--adaptive-sigma` outputs byte-identical to `25f989df` (6/6 anchors, agent + independent
reviewer); channel cost +3 MB transient, net wall neutral; ctest green.

Bench (frozen clouds, tag `phase43-sigma2`, n=3 runs/scene; references = stage-1 rows and
Phase 1/2 baselines above):

| scene | adaptive-σ (stage 1) | +conf03 | Δ | +conf05 | Δ | conf05-only | Δ vs baseline |
|---|---|---|---|---|---|---|---|
| Barn | 0.5647 | 0.5649 | +0.0002 | 0.5642 | −0.0005 | 0.5569 | −0.0007 |
| Ignatius | 0.3321–0.3325 | 0.3316 | −0.0009 | 0.3328 | +0.0003 | **0.3380** | **+0.0085** |
| Meetingroom | 0.3383–0.3396 | 0.3386 | ~0 | 0.3385 | ~0 | 0.3385 | +0.0006 |
| Truck | 0.3601–0.3602 | 0.3624 | +0.0022 | 0.3631 | +0.0029 | 0.3605 | +0.0036 |

**§8 gate: NOT met on top of adaptive σ.** Stacked on the recommended default, the shrink
is noise on three scenes and +0.0022..29 on Truck only (mean +0.0007..14, below +0.003);
no scene regresses beyond noise, so the switch is safe but stays opt-in at 0.

**The confidence signal itself is real — and largely redundant with adaptivity.** The
shrink-only arm (global σ base) posts the best Ignatius of the whole phase: 0.3380
(+0.0085 over baseline, +0.0055 over adaptive σ), with better precision *and* recall on an
18 % smaller in-crop mesh, plus Truck +0.0036 — but Barn −0.0007 and a +0.0030 mean that
sits exactly at, not beyond, the gate. Crucially the two mechanisms do not stack: adaptive
+ conf05 on Ignatius (0.3328) recovers almost none of the conf-only gain (0.3380) — where
confident points cluster, the local-median σ is already tight and the shared 0.25 σ floor
saturates. Verdict: confidence carries geometry information orthogonal to local density on
object scenes, but under the adaptive default there is no headroom left at this
formulation. This is direct motivation for Phase 5.3 (per-point *scale* export from
densification): a principled per-point σ source could replace the heuristic
`1 − s·conf` shrink rather than multiply into it.

Interaction on record (from review): with `--constant-weight 1 --sigma-conf-shrink >0` the
weights now survive into reconstruction, so adding `--quality-co-scale 1` to such a run
activates the co-scale where it used to no-op silently. Both flags are opt-in and
default-off; no default run is affected.

## Phase 3.4 — canonical power-of-two rescale (2026-08-20) — implemented (slice 14); robustness fix, default flip recommended

Implementation (`3c51e995`): `--canonical-rescale` multiplies the finished triangulation in
place by `s = 2^(−round(log2 medianEdge))`, engaged only when |log2(median edge)| > 10. No
retriangulation: the kernel's exact predicates are scale-invariant and a power-of-two
factor is exact in IEEE arithmetic, so every cell/vertex handle survives and the inverse at
extraction round-trips bit-for-bit. The median edge is now measured once at triangulation
time (hoist proven bit-exact by the off anchors) and reused for σ; camera centers, carve
rays, the end-cell locate and the hull-facet culling all run in the working space, with the
frustum test mapped back to the camera's own space through the exact round-trip. Off state
and flag-on-inside-the-band are byte-identical to `cc3e4080` (5/5 anchors, agent +
independent reviewer); ctest green, no new warnings.

Validation on ×1e−6 / ×1e+6 copies of the SceauxCastle anchor (via TransformScene,
`diag(s,s,s)` 3×4 matrix; float-storage quantization is relative, so both magnitudes carry
the same ≈6e-5-of-an-edge noise as the native cloud — verified):
- **tiny + off: total collapse**, exactly as the item-8 audit predicted (L=1.19e-7 ≪ the
  1.1e-4 threshold): 72542/72542 camera rays dropped with empty facet lists (the root site
  is `fetchCellFacets`, whose hull-facet `orientation()` answers COPLANAR for everything —
  this is why the rescale must precede camera-cell location, a mid-slice design correction),
  zero flow, empty mesh.
- **tiny + on: full rescue** — engages at 2^23, every counter matches native (walk steps
  within 5, from the scaled cloud's float round-trip costing 2 Delaunay cells), flow
  18878.3 vs 18878.4, vertex/face counts identical; raw-mesh vertices agree with native to
  3e-6 scene units (float-storage floor; the zero-deviation off-vs-on-native control
  validates the metric).
- **huge + off does NOT collapse but is not clean either** (new finding beyond the audit):
  at L=1.2e5 the epsilon is ~1e-27 relative and never fires, and near-degenerate
  configurations resolved by float noise cost 7 vertices/14 faces and 0.13 % of the flow;
  **huge + on restores the native flow and counts exactly**. The "inert above L≈6" verdict
  was right about severity, wrong about the epsilon being behaviourally free.
- Cross-scale consistency: the tiny-on and huge-on meshes, 1e12 apart in input scale, agree
  to 4e-6 of a median edge.
- WSS arms (fss 1): tiny/huge + on reproduce every WSS counter, the flow and the counts of
  the native fss anchor; tiny + off collapses identically (WSS skipped 72542).

**Default-flip recommendation, reserved to the maintainer**: flip `bCanonicalRescale` to
true. The case is stronger than the adaptive-σ one: inside the band the flag provably does
nothing (branch, no work, byte-identical output — anchors), and every standard scene is
deep inside it (T&T frozen scenes L=0.021–0.047, SceauxCastle 0.119, vs band edge 9.8e-4),
so the flip cannot change any normal run; outside the band the current behavior is an empty
or silently degraded mesh.

On record, out of this slice's scope (the audit's other half of item 8): geometry already
quantized by the float `PointCloud::Point` storage at large coordinates (~6 cm at UTM
magnitude) cannot be repaired at mesh time — that needs centering at load/import time,
where doubles still exist (Interface importers / CreateStructure). Separate slice, touches
all pipelines, needs its own validation.

Post-`ReconstructMesh` observation (not this slice): the app's mesh *cleaning* stage is
scale-sensitive — all scaled arms, including huge+off where the rescale code never runs,
show ≈0.15-edge median deviation in the cleaned (non-raw) mesh vs native, while the raw
meshes sit at the float floor. Worth a look if extreme-scale scenes become a supported
workflow.

## Phase 5.3 — footprint-based σ_v (2026-08-20) — implemented (slice 15); gate NOT met vs adaptive, opt-in retained; footprint ≡ confidence in effect

Implementation (`c34d2623`): `--footprint-sigma` derives σ_v from the mean pixel footprint
(range/focal over the vertex's (point,view) pairs), read at mesh time from the views alone —
no .mvs format change, no densification change, works on weightless clouds. Consumed
relatively (calibrated by σ/median(footprint), so the absolute scale stays the established
global σ and the canonical-rescale factor is carried exactly — proven identical across
twelve orders of magnitude of input scale), same [0.25,4]×σ clamp, conf-shrink composes on
top, competing base with `--adaptive-sigma` (app rejects the combination). The slice-13
accumulator was widened to carry both per-vertex sums in one table; the shrink arm's
outputs are byte-identical (HEAD cross-check). Off state byte-exact (5/5 anchors, agent +
independent reviewer); ctest green. The field is structurally unlike the adaptive one:
median pinned at exactly 1.0 by the calibration (no global bias, vs adaptive's 0.881),
0.02 % clamped vs 2.6 %, ~50× cheaper to fill (no edge traversal).

Bench (frozen clouds, tag `phase53-footprint`, n=3):

| scene | baseline | adaptive-σ | footprint | Δ vs base | Δ vs adaptive | foot+conf05 | conf05-only (stage 2) |
|---|---|---|---|---|---|---|---|
| Barn | 0.5576 | 0.5647 | 0.5574 | −0.0002 | −0.0073 | 0.5570 | 0.5569 |
| Ignatius | 0.3295 | 0.3321–25 | 0.3382 | **+0.0087** | +0.0057 | 0.3364 | 0.3380 |
| Meetingroom | 0.3379 | 0.3383–96 | 0.3384 | +0.0005 | ~0 | 0.3392 | 0.3385 |
| Truck | 0.3569 | 0.3601–02 | 0.3599 | +0.0030 | ~0 | 0.3579 | 0.3605 |

**§8 gate: NOT met.** Mean vs baseline +0.0030 — at the gate, not beyond it — and vs the
recommended adaptive default it is mean-neutral with a redistribution (Ignatius +0.0057,
Barn −0.0073). Opt-in retained.

**The decisive finding: footprint ≡ confidence-shrink in effect.** The footprint arm
reproduces the stage-2 shrink-only row on every scene to within noise (0.5574/0.5569,
0.3382/0.3380, 0.3384/0.3385, 0.3599/0.3605) — two independent implementations of the same
physical signal (range-driven observation quality: near/well-observed → tighter σ), one
from geometry, one from the recalibrated confidence. And stacking the shrink on the
footprint base double-counts that signal (foot+conf05 ≤ footprint on 3/4 scenes),
mirroring stage 2's no-stacking result on the adaptive base. The σ_v picture is now
complete: there are exactly TWO independent signals — the *physical* one (footprint ≡
confidence), which owns the object-scene gains (Ignatius +0.0085, Truck +0.003), and the
*sampling-density* one (median incident edge), which owns Barn (+0.0071) — and they do not
combine additively in this formulation. Adaptive σ remains the best single default; the
physical field is the better choice on object-centric captures, available equally via
`--footprint-sigma` (weightless clouds) or `--sigma-conf-shrink` (weighted clouds).

Speed note on record: the footprint arms run the walks noticeably slower on some scenes
(Ignatius 229 s vs ~98 s for the conf arms, Meetingroom 139 s vs ~64 s, and Barn
foot+conf05 332 s) — same σ-field-dependent walk-cost phenomenon as the stage-1
thickness-088 Barn oddity, still uninvestigated; the adaptive field is the fast one.

### Phase 5.3 / §9 close-out
This was the last §9 slice. Standing open items across the effort: (1) maintainer
sign-offs — flip `--adaptive-sigma` default on, flip `--canonical-rescale` default on,
align library kSigma 2.f→1.f; (2) load-time centering for float-quantized
large-coordinate clouds (import-side half of Phase 3.4); (3) the mesh-vs-cloud fidelity
gap — RESOLVED, root cause was the cleaning smoother, see the 2026-08-21 section below;
(4) P5.2 direct dmap meshing parked (implementation plan now written:
`DepthmapMeshingPlan.md`), P6 deferred; (5) optional: upstream EIBFS-I excess-bucket
crash report; the scale-sensitive mesh-cleaning stage observed in Phase 3.4 — RESOLVED,
same 2026-08-21 section (it was the fixed-time MCF).

## Mesh-vs-cloud fidelity gap — ROOT CAUSE FOUND & FIXED (2026-08-21)

The standing open item ("Ignatius cloud 0.77 -> mesh 0.34") is resolved. The gap was never in
the mesh estimation: **the graph-cut surface scores within a few points of the input cloud; the
`Mesh::Clean` smoothing stage was destroying it.** A second, smaller mechanism (Truck-class
scenes) is unsupported "webbing" surface, now removable by an opt-in evidence gate.

### The investigation (Ignatius first, per instruction; frozen family-A clouds, official T&T eval)

Instrumented scoring of every pipeline stage (same in-crop 10M-sample protocol as the bench;
`d_f1` rows in `bench/out_mesh/results.csv` cross-checked to +-0.0006):

| stage scored | P | R | F1 |
|---|---|---|---|
| input cloud | 0.7271 | 0.7494 | 0.7381 |
| Delaunay vertices kept by the cut (points only) | 0.762 | 0.425 | 0.545 |
| **raw graph-cut mesh (pre-clean)** | **0.6992** | **0.6980** | **0.6986** |
| after full Clean minus smooth (`--smooth 0`) | 0.6994 | 0.6979 | 0.6986 |
| after 1 smooth iteration (old MCF) | 0.4989 | 0.4346 | 0.4645 |
| after 2 smooth iterations = shipped default | 0.3630 | 0.3016 | 0.3295 |

Attribution is airtight: every non-smooth clean step combined (long-edge removal, component
removal, spikes, hole-closing) costs exactly nothing (0.6986 -> 0.6986), and the two MCF
iterations produce the entire collapse with clean dose-response. The cleaned mesh also loses
21% of its in-crop surface area to MCF shrinkage. Error structure before/after: the raw mesh
has median surface error 1.77mm (fixed frame), no gross junk (P reaches 0.91 at 15mm); the
cleaned mesh has median error ~4.6mm — a uniform blur, which is why only the small-tau scenes
(Ignatius tau=3mm, Truck 5mm) collapsed while Barn/Meetingroom (tau=10mm) looked acceptable.

**Root cause** (`libs/MVS/Mesh.cpp`, commit `c99883fc` "mesh: remove VCG and use CGAL for
cleaning"): the VCG scale-free Laplacian smoothing was replaced by CGAL
`PMP::smooth_shape` — implicit mean-curvature flow with a **fixed absolute time step 1e-3**.
That time constant has units of squared scene length: it was evidently calibrated on ~3cm-edge
meshes (0.03^2 ~= 1e-3) and over-smooths ~20x on Ignatius' ~7mm-edge statue mesh, on any
metric-scale fine-resolution scene, while a single global time cannot fit a mixed-resolution
mesh (fine statue + coarse background) at all. This is also the mechanism behind the
"scale-sensitive mesh cleaning" observation recorded in Phase 3.4.

**Fix**: `Mesh::Clean` now smooths with a classic per-vertex uniform-Laplacian relaxation
(lambda=0.5, `nSmooth` iterations, borders fixed, deterministic double-buffered update) — each
vertex moves relative to its own one-ring scale, so the result is unit- and
resolution-independent, restoring the pre-CGAL behavior class.

### Validation (shipped defaults = smooth 2, official T&T eval)

| scene | cloud F1 | shipped before | raw (no clean) | **shipped after fix** |
|---|---|---|---|---|
| Ignatius (tau 3mm) | 0.7381 | 0.3295 | 0.6986 | **0.6960** |
| Truck (tau 5mm) | 0.7060 | 0.3569 | 0.4835 | **0.5015** |
| Barn (tau 10mm) | 0.5988 | 0.5576 | 0.5704 | **0.5926** |
| Meetingroom (tau 10mm) | 0.3225 | 0.3379 | 0.2185 | **0.3650** |

The fixed clean now denoises instead of destroying: on both object scenes it lands at or above
the raw mesh (Ignatius -0.0026 vs raw with precision up 0.699->0.718; Truck +0.018 vs raw), and
the two tau=10mm scenes — the ones the old MCF appeared to serve acceptably — also improve over
the previously shipped result (Barn +0.035, Meetingroom +0.027, the latter now above its input
cloud). Meetingroom's low raw score is not a counter-signal: its raw indoor mesh carries ~5x the
cleaned surface area in gross junk faces (16k vs 3.1k area units in-crop) that the non-smooth
clean steps (long-edge/spurious/component removal) legitimately strip; smoothing choice is
orthogonal to that.

### Second mechanism — unsupported webbing (Truck-class scenes)

Truck's raw mesh still trails its cloud (0.4835 vs 0.7060) for a different reason: healthy
recall (0.686 ~= cloud) but precision 0.373. The one-sided ground-level camera ring leaves
occluded regions (under the chassis, behind walls) where the visibility mesh grows facets no
ray supports; 10% of faces sit >30mm from ANY input point (p99: 305mm) and carry 41% of the
in-crop sampled area. The default `--remove-spurious 20` cannot touch them (its threshold
resolves to ~10 meters on these meshes). Offline face-filter prototypes on the raw mesh:

| filter | faces dropped | P | R | F1 |
|---|---|---|---|---|
| none (raw) | — | 0.3733 | 0.6860 | 0.4835 |
| max edge > 100mm | 9.7% | 0.5871 | 0.6856 | 0.6326 |
| centroid > 4x median NN spacing from cloud | 11.7% | 0.6082 | 0.6865 | **0.6450** |

Recall is untouched in both — the dropped surface supports nothing real.

**First attempt, REFUTED — visibility-mass gate.** The obvious estimation-side signal is the
alpha_vis crossing mass the votes accumulate on each facet (webbing should be mass zero: no ray
enters occluded space). Implemented and validated as `--min-surface-evidence`; it does not work:

| arm | facets removed | P | R | F1 |
|---|---|---|---|---|
| Truck raw (no gate) | — | 0.3733 | 0.6860 | 0.4835 |
| Truck mass < 1e-6 | 2.99M / 4.97M | 0.3850 | 0.5701 | 0.4596 |
| Truck mass < 0.05 | 3.01M / 4.97M | 0.3836 | 0.5669 | 0.4576 |
| Ignatius raw (no gate) | — | 0.6992 | 0.6980 | 0.6986 |
| Ignatius mass < 1e-6 | 2.54M / 4.11M | 0.6876 | 0.6175 | 0.6507 |
| Ignatius mass < 0.05 | 2.57M / 4.11M | 0.6849 | 0.6078 | 0.6440 |

~60% of cut facets carry mass EXACTLY ZERO on both scenes — including most of the true statue
surface of Ignatius, which has essentially no webbing. The mechanism: each ray is a 1D needle
through the tetrahedralization; it crosses only 1-2 facets of a vertex's ~20-facet umbrella, so
the vote mass lives on a sparse subset of the real surface and no threshold separates webbing
(zero) from surface (mostly also zero). Recall collapses, precision barely moves. The parameter
was removed again.

**Shipped gate — `--max-edge-scale <k>`.** The signals that do work offline are geometric, and
they are nearly the same signal: every Delaunay vertex IS an input point, so a facet can only
stray far from the observed cloud by spanning it with long edges. The gate drops extracted cut
facets whose longest edge exceeds k x the median cut-facet longest edge (two passes over the cut
facets, medians in working space so canonical rescale cancels; ratio units, scene-independent;
0 = off, byte-identical). Calibration on the raw meshes: Truck median max-edge 17.9mm, so k=6
(107mm) drops 9.2% of faces — matching the 100mm prototype (9.7%); Ignatius' global median is
42mm (background-dominated), so k=6 = 254mm sits far above the ~7mm statue facets and only
removes gap-spanners. Validation (official eval, `--max-edge-scale 6`):

| arm | facets removed | P | R | F1 |
|---|---|---|---|---|
| Truck raw, no gate | — | 0.3733 | 0.6860 | 0.4835 |
| Truck raw, k=6 | 457,918 (9.2%) | 0.5814 | 0.6869 | **0.6298** |
| Truck k=6 + fixed clean | | 0.5988 | 0.6725 | **0.6335** |
| Ignatius raw, no gate | — | 0.6992 | 0.6980 | 0.6986 |
| Ignatius raw, k=6 | 608,815 (14.8%) | 0.7094 | 0.6978 | **0.7036** |

Recall is untouched on both scenes and precision jumps only where webbing existed — the gate
reproduces its offline prototype in-pipeline (0.6298 vs 0.6326) and is a small pure win even on
the webbing-free Ignatius (+0.005). Truck end-to-end: 0.3569 shipped before this work, 0.5015
with the smoothing fix, 0.6335 with fix + gate — 90% of its input cloud's 0.7060. The gate
ships opt-in (default 0); flipping a default (k~=6) awaits a Barn/Meetingroom no-harm arm and
the maintainer's call, alongside the other pending default flips.

### Scope notes

- `--min-point-distance` decimation exonerated: inserting all 5.2M points (no decimation)
  scores 0.6764 raw — slightly WORSE than the decimated 0.6986, at 4.4x graph-cut cost.
- The per-eval ICP realignment of the T&T toolbox explains small cross-eval comparison
  paradoxes (vertex-set vs sampled-surface recall); within-protocol comparisons are unaffected.
- The Phase 5.3 close-out's "mesh-vs-cloud fidelity gap" open item and its "raw-vs-cleaned
  scoring arm unrun" note are both closed by this section.

## Re-evaluation of every estimation idea on the raw+gated surface (2026-08-21)

Every A/B verdict of the §9 effort was scored on the *cleaned* mesh — through the fixed-time
MCF smoother now known to have been crushing the small-tau scenes to the ~0.33 regime. This
campaign re-scores the ideas on the honest metric: the raw graph-cut surface with the k=6
`--max-edge-scale` gate applied (offline face filter, validated to reproduce the in-recon gate
to 68 faces in 4.5M), official T&T eval, frozen family-A clouds. Baselines (gated raw):
Ignatius 0.7036, Truck 0.6298, Barn 0.6069, Meetingroom 0.3959.

### The gate itself is a universal win, and k=4 dominates k=6

| scene | raw (ungated) | k=8 | k=6 | k=4 | input cloud |
|---|---|---|---|---|---|
| Ignatius | 0.6986 | 0.7021 | 0.7036 | **0.7048** | 0.7381 |
| Truck | 0.4835 | 0.6202 | 0.6298 | **0.6441** | 0.7060 |
| Barn | 0.5704 | — | 0.6069 | **0.6144** | 0.5988 |
| Meetingroom | 0.2185 | — | 0.3959 | **0.3961** | 0.3225 |

k=4 is better-or-equal on all four scenes (recall never moves by more than 0.008). Gated raw
already beats the input cloud on Barn and Meetingroom. Recommended default: ON at k=4
(k=6 as the conservative choice).

### Single-idea re-verdicts (gated k=6 raw F1; Δ vs the scene's gated baseline)

| arm | Ignatius | Truck | Barn | Meetingroom | old (blurred) verdict → new verdict |
|---|---|---|---|---|---|
| adaptive-σ | 0.7427 (+.039) | 0.6451 (+.015) | 0.6191 (+.012) | 0.4036 (+.008) | "+0.003..0.007, gate met" → **universal win, 5-12x larger than reported; default-flip case now overwhelming** |
| conf-shrink 0.5 (global base) | 0.7215 (+.018) | 0.6392 (+.009) | — | — | "+0.0085 best-Ignatius" → real but dominated by adaptive; still does not stack with it (a+c05: Ign +0.0008, Truck +0.0024) |
| weighted (`--constant-weight 0`) alone | 0.4898 (−.214) | 0.6191 (−.011) | — | — | REJECTED → still rejected alone (Ignatius cut collapses to 241k faces) |
| weighted + quality co-scale | 0.7380 (+.034) | 0.6455 (+.016) | — | — | "recovers the collapse, not defaultable" → **major win on objects** |
| free-space-support | 0.6554 (−.048) | 0.5783 (−.052) | — | — | off → confirmed off |
| grazing floor 0.2 | 0.6552 (−.048) | 0.6024 (−.027) | — | — | "+0.005..+0.010 on objects" → **FLIPPED: harmful everywhere; the apparent object-scene gain was an artifact of the broken smoother** |
| thickness-factor 2 (library kSigma) | 0.5574 (−.146) | 0.5869 (−.043) | — | — | "strictly worse" → confirmed, much larger; kSigma 2.f→1.f alignment now urgent |

### Stacking (the old "doesn't stack" answers were also blur-scored)

| config | Ignatius | Truck | Barn | Meetingroom |
|---|---|---|---|---|
| gated baseline | 0.7036 | 0.6298 | 0.6069 | 0.3959 |
| adaptive-σ | 0.7427 | 0.6451 | 0.6191 | 0.4036 |
| adaptive + weighted + co-scale | **0.7497** | **0.6558** | 0.5989 | 0.3848 |
| + conf-shrink 0.5 (triple) | 0.7492 | 0.6579 | — | — |

adaptive + weighted+co-scale STACKS on the object scenes (+0.007/+0.011 over adaptive alone) —
a genuinely new result the blur had hidden — but it regresses both planar scenes (Barn −0.020,
Meetingroom −0.019, recall-paid), the historical Barn objection surviving in attenuated form.
So the stack is a documented OBJECT-SCENE OPTION, not a default. conf-shrink remains redundant
in the stack (±0.002, scene-inconsistent). The stack's cut is nearly webbing-free even before
gating (45-67k faces gated vs ~380-720k for unit votes). Ignatius' raw surface exceeds its
input cloud by 0.012.

### Speed (graph-cut stage, from the same runs)

adaptive-σ is the fastest arm as well as the most accurate: Ignatius 32.4s (vs 35-50s all
other single arms), Truck 65.5s ≈ conf05. The weighted arms pay ~1.5-3x in the solve
(Ignatius wcoscale 108s, a_wcos 53.6s; Truck a_wcos comparable to baseline). fss carves the
most and solves fastest on Truck (42.5s) but loses 0.05 F1.

### Recommended default: adaptive-σ + gate (all four scenes, raw surface)

| scene | originally shipped | adaptive + gate k6 | adaptive + gate k4 | input cloud |
|---|---|---|---|---|
| Ignatius | 0.3295 | 0.7427 | 0.7427* | 0.7381 |
| Truck | 0.3569 | 0.6451 | **0.6611** | 0.7060 |
| Barn | 0.5576 | 0.6191 | **0.6257** | 0.5988 |
| Meetingroom | 0.3379 | 0.4036 | 0.4036* | 0.3225 |

(*k4≈k6 on these scenes per the baseline sweep; not re-run.) Three of four scenes now score
ABOVE their input cloud (Truck at 94% of its), and adaptive-σ is simultaneously the fastest
arm. Versus the originally shipped defaults the raw surface gains +0.07..+0.41 F1 per scene.

### Consequences

- The three pending maintainer flips (adaptive-sigma on, canonical-rescale on, kSigma 2→1)
  are all re-confirmed with 5-12x larger margins; `--max-edge-scale` ON (k≈4) joins them as
  a recommended default.
- Grazing down-weighting (P4.2) should be considered for removal or at least documented as
  harmful: its only support was a scoring artifact of the broken smoother.
- The weighted+co-scale path (P2/P4.1 arm C) is rehabilitated as an object-scene opt-in
  (+0.007..+0.011 over adaptive) with a documented planar-scene cost (−0.02).
- Not re-run: carve-only rays (its rays cannot reach occluded webbing by construction, and
  the gate now removes that surface geometrically; its prior A/B baseline also used different
  clouds) and footprint-σ (shown ≡ conf-shrink, which re-confirmed as redundant here).
