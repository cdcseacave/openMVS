# Dense fill budget as a density, BA dense weight as a measurement — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the one-pass matcher's two constants — a flat 2000-correspondence dense fill cap and a flat 0.25 bundle-adjustment weight for dense observations — with the quantities they stand for: a correspondence density over the overlap the sparse matches did not cover, and a weight measured from the two observation populations' own reprojection residuals.

**Architecture:** The dense fill's bucket grid stops being sized per draw and becomes a constant of the configuration (one bucket per dense match per frame), which turns the sampler's existing "an occupied bucket yields nothing" rule into exactly the uncovered-overlap density rule, with the verdict's smaller inlier area as a ceiling so the density is bounded in image B too. Bundle adjustment resolves its dense observation weight at the head of every solve as `(sigma_described/sigma_dense)^2`, both sigmas being medians of raw pixel reprojection errors of the current solution. The view graph's dense discount stops sharing that number, because connectivity evidence is not measurement precision.

**Tech Stack:** C++17, OpenMVS `libs/SFM`, Ceres (bundle adjustment), OpenCV, ONNX Runtime (RoMa v2), the in-repo test harness `apps/Tests`.

**Spec:** `docs/superpowers/specs/2026-09-04-dense-budget-and-weight-design.md`

## Global Constraints

- **No backward compatibility, at any level.** Superseded code, config fields, CLI text and documentation are deleted or rewritten, never kept alongside the new rule. There are no deprecation shims and no "old behaviour" flags.
- **Build tree:** `/home/ubuntu/.claude/worktrees/roma2-onnx/make`. Build: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28`. It must finish **warning-free**.
- **Tests:** `ctest -C Release --output-on-failure` from that build tree — 3 suites (CommonUnitTests, SFMPipelineTest, MVSPipelineTest). All must pass. The ROMA2 tests that need an exported model skip themselves when `OPENMVS_ROMA2_MODEL_PATH` is unset; every test this plan adds must run **without** a model.
- **Never touch `/home/ubuntu/openMVS`** (a different checkout). All work happens in the worktree above, on branch `feature/roma2-onnx`.
- **Commits** are authored `cDc <cdc.seacave@gmail.com>`. No `Co-Authored-By` trailer, no `Claude-Session` trailer, no Claude or AI attribution of any kind, in commit messages or in code comments.
- **Never `git stash` / `git stash pop`** (the stash stack is shared with other worktrees). Set work aside with a WIP commit.
- **Comment style:** this library explains *why*, in prose, at the point the reader needs it. Match the surrounding density. A comment that restates the code is worse than none; a rule with a non-obvious reason must carry the reason.
- **Do not run the matcher or the reconstruction end to end.** Measurement is the controller's, after the branch is green (see "Measurement", not a task).
- **Never dispatch subagents.** If you are an implementer, you implement; review arrives from the controller.

## File structure

| File | Responsibility after this plan |
|---|---|
| `libs/SFM/ROMA2Warp.h` | `DenseFillGridSide` (new, inline) — the fill's fixed pitch; `SampleWarpComplementary` declaration gains `bucketGridSide` |
| `libs/SFM/ROMA2Warp.cpp` | `SampleWarpComplementary` uses the caller's pitch; the per-draw-grid rationale is gone. `WarpBucketGridSide` stays, now `SampleWarpByCoverage`'s alone |
| `libs/SFM/MatchROMA2.h` | `ROMA2Config::denseMatchesPerFrame` (renamed); `DenseFillCeiling` (new, inline) |
| `libs/SFM/MatchROMA2.cpp` | `AssemblePairROMA2` passes pitch and ceiling; the per-pair log record gains both |
| `libs/SFM/Track.h/.cpp` | `ComputeObservationSigmas` (new) — the two populations' robust reprojection sigmas |
| `libs/SFM/BundleAdjustment.h` | `denseObservationWeight` becomes an override, default `-1` = estimate |
| `libs/SFM/BundleAdjustment.cpp` | `EstimateDenseObservationWeight` (new); both solves resolve the weight once and pass it to `SelectReprojectionLoss` |
| `libs/SFM/ImagePair.h`, `libs/SFM/PairsWeighting.h` | `DENSE_OBSERVATION_WEIGHT` becomes the view graph's own constant and BA's fallback; the "one quantity" comments say why there are two |
| `apps/CreateStructure/CreateStructure.cpp` | `--roma2-dense-matches` and `--ba-dense-weight` help rewritten; the line wiring BA's weight into the view graph deleted |
| `libs/SFM/PythonWrapper.cpp` | the `dense_matches` binding takes the new field name |
| `apps/Tests/TestsSFM.{h,cpp}`, `apps/Tests/Tests.cpp` | `ROMA2DenseFillDensityTest`, `ROMA2DenseFillCeilingTest`, `ObservationSigmasTest`, `DenseObservationWeightEstimateTest`; `ROMA2ComplementaryDrawTest` updated |
| `libs/SFM/README.md`, `docs/design/ROMA2InProcess.md` | the fill's rule, the estimated weight, the view graph's own constant |

---

### Task 1: The dense fill's pitch becomes a constant of the configuration

**Files:**
- Modify: `libs/SFM/ROMA2Warp.h:194-225` (the `SampleWarpComplementary` header comment and declaration), and the helper block near `libs/SFM/ROMA2Warp.h:108`
- Modify: `libs/SFM/ROMA2Warp.cpp:299-380` (`SampleWarpComplementary`)
- Modify: `libs/SFM/MatchROMA2.cpp:757-766` (the one call site)
- Test: `apps/Tests/TestsSFM.cpp` (`ROMA2ComplementaryDrawTest`, and a new `ROMA2DenseFillDensityTest`), `apps/Tests/TestsSFM.h`, `apps/Tests/Tests.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `SFM_API int DenseFillGridSide(unsigned denseMatchesPerFrame, int warpSide)` (inline, `ROMA2Warp.h`) and the new `SampleWarpComplementary` signature with `int bucketGridSide` immediately before `unsigned maxSamples`. Task 2 calls both.

- [ ] **Step 1: Write the failing test**

Add to `apps/Tests/TestsSFM.cpp`, next to `ROMA2ComplementaryDrawTest`. It builds the same identity-warp two-image setup that test uses (copy that setup — a 640x480 pinhole camera shared by two images, a 160-cell warp mapping every cell back to its own pixel, so a drawn point of A comes back unmoved in B), then checks the four properties the fixed pitch is for:

```cpp
bool ROMA2DenseFillDensityTest()
{
	TD_TIMER_START();

	// --- the identity-warp setup of ROMA2ComplementaryDrawTest -------------------------------
	Scene scene;
	const int width = 640, height = 480;
	scene.cameras.emplace_back(new PinholeCamera(cv::Size(width, height),
		REAL(600), REAL(600), REAL(width)/2, REAL(height)/2));
	scene.images.emplace_back((IIndex)0, String("a.jpg"));
	scene.images.emplace_back((IIndex)1, String("b.jpg"));
	FOREACH(i, scene.images) {
		scene.images[i].cameraID = 0;
		scene.images[i].pCamera = scene.cameras[0];
	}
	const Image& imgA = scene.images[0];
	const Image& imgB = scene.images[1];
	const int cells = 160;
	Image32F2 warp(cells, cells);
	for (int y = 0; y < cells; ++y)
		for (int x = 0; x < cells; ++x)
			warp(y, x) = Point2f(
				((x*(width-1.f)/(cells-1)) + 0.5f)*2.f/width - 1.f,
				((y*(height-1.f)/(cells-1)) + 0.5f)*2.f/height - 1.f);
	const auto CellToPixel = [&](int cx, int cy) {
		return Point2f((float)cx*(width-1.f)/(float)(cells-1), (float)cy*(height-1.f)/(float)(cells-1));
	};

	// --- 1) THE PITCH IS A FUNCTION OF THE DENSITY ALONE -------------------------------------
	// one bucket per dense match at full overlap, clamped to the warp side, never below 1
	if (DenseFillGridSide(2000, cells) != 45 || DenseFillGridSide(0, cells) != 1 ||
		DenseFillGridSide(1000000, cells) != cells) {
		VERBOSE("ROMA2DenseFillDensityTest FAILED: pitch %d/%d/%d for densities 2000/0/1000000",
			DenseFillGridSide(2000, cells), DenseFillGridSide(0, cells), DenseFillGridSide(1000000, cells));
		return false;
	}

	// --- 2) THE DRAW SCALES WITH THE OVERLAP AREA, NOT WITH THE BUDGET -----------------------
	// the same pitch over two confident regions, one four times the other: the draw has to come out
	// about four times larger, which is what "a density" means and what the old per-draw grid (sized
	// so that ANY overlap yielded ~maxSamples) could not do
	const unsigned density = 2000;
	const int grid = DenseFillGridSide(density, cells);
	const std::vector<Point2f> none;
	std::vector<Point2f> bigA, bigB, smallA, smallB;
	std::vector<float> bigC, smallC;
	Image32F confidence(cells, cells);
	const auto DrawOver = [&](int cellsWide, int cellsHigh, const std::vector<Point2f>& occupied,
		std::vector<Point2f>& outA, std::vector<Point2f>& outB, std::vector<float>& outC) {
		confidence.setTo(0.f);
		for (int y = 0; y < cellsHigh; ++y)
			for (int x = 0; x < cellsWide; ++x)
				confidence(y, x) = 0.9f;
		return SampleWarpComplementary(imgA, imgB, warp, confidence, 0.3f, grid, density,
			occupied, outA, outB, outC);
	};
	const size_t numBig = DrawOver(80, 80, none, bigA, bigB, bigC);      // a quarter of the frame
	const size_t numSmall = DrawOver(40, 40, none, smallA, smallB, smallC); // a sixteenth
	if (numBig < 3*numSmall || numBig > 5*numSmall || numSmall == 0) {
		VERBOSE("ROMA2DenseFillDensityTest FAILED: a 4x larger overlap drew %u against %u points",
			(unsigned)numBig, (unsigned)numSmall);
		return false;
	}

	// --- 3) THE SPARSE MATCHES TAKE THEIR SHARE OUT OF THE DRAW ------------------------------
	// the same region drawn three times: uncovered, half covered by guided matches, fully covered.
	// The yield has to fall with the uncovered area and reach zero when nothing is left uncovered.
	std::vector<Point2f> halfOccupied, allOccupied;
	for (int y = 0; y < 80; ++y)
		for (int x = 0; x < 80; ++x) {
			if (x < 40)
				halfOccupied.push_back(CellToPixel(x, y));
			allOccupied.push_back(CellToPixel(x, y));
		}
	std::vector<Point2f> halfA, halfB, fullA, fullB;
	std::vector<float> halfC, fullC;
	const size_t numHalf = DrawOver(80, 80, halfOccupied, halfA, halfB, halfC);
	const size_t numFull = DrawOver(80, 80, allOccupied, fullA, fullB, fullC);
	if (numHalf == 0 || numHalf > numBig*3/5 || numHalf < numBig/3) {
		VERBOSE("ROMA2DenseFillDensityTest FAILED: half the overlap covered drew %u of %u",
			(unsigned)numHalf, (unsigned)numBig);
		return false;
	}
	if (numFull != 0) {
		VERBOSE("ROMA2DenseFillDensityTest FAILED: a fully covered overlap still drew %u points",
			(unsigned)numFull);
		return false;
	}

	// --- 4) TWO PAIRS SHARING IMAGE A AGREE ON WHERE THEY SAMPLE IT --------------------------
	// the property the fixed pitch buys downstream: two pairs that both hold A stratify it on the
	// same grid, so their winners are the same cells and their A-side keypoints the same pixels --
	// which is what lets FilterRedundantKeypoints chain them into a track longer than two. The
	// second pair sees a LARGER confident region, so this is not two identical inputs: the shared
	// part of the two draws still has to agree pixel for pixel.
	std::vector<Point2f> wideA, wideB;
	std::vector<float> wideC;
	DrawOver(120, 120, none, wideA, wideB, wideC);
	size_t numShared = 0;
	for (const Point2f& pt : bigA)
		if (std::find_if(wideA.begin(), wideA.end(), [&](const Point2f& q) {
				return normSq(q - pt) < 1e-4f; }) != wideA.end())
			++numShared;
	if (numShared*10 < bigA.size()*9) {
		VERBOSE("ROMA2DenseFillDensityTest FAILED: only %u of %u samples of A are shared by a second pair",
			(unsigned)numShared, (unsigned)bigA.size());
		return false;
	}

	VERBOSE("ROMA2 dense fill density test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
```

Declare it in `apps/Tests/TestsSFM.h` beside `ROMA2ComplementaryDrawTest`, with a comment in that file's style saying what it pins, and call it from `apps/Tests/Tests.cpp` right after `ROMA2ComplementaryDrawTest`.

- [ ] **Step 2: Run the build to verify it fails**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28`
Expected: FAIL — `DenseFillGridSide` undeclared, and `SampleWarpComplementary` called with 11 arguments where it takes 10.

- [ ] **Step 3: Add the pitch function**

In `libs/SFM/ROMA2Warp.h`, immediately after `WarpTolerance` (which ends the small-helper block around line 111):

```cpp
// Side of the square bucket grid the dense fill stratifies the WHOLE warp grid on. Unlike the
// verdict sampler's grid (WarpBucketGridSide) this is a constant of the configuration and not of the
// pair: one bucket per dense match at full overlap, so a pair's overlap holds
// denseMatchesPerFrame * overlapArea buckets whatever that overlap is, and the draw's density -- the
// thing the reconstruction actually pays for -- is the same in every pair. Clamped to the warp side,
// because a bucket finer than a cell would hold at most one candidate and stratify nothing.
inline int DenseFillGridSide(unsigned denseMatchesPerFrame, int warpSide) {
	ASSERT(warpSide > 0);
	return MINF(warpSide, MAXF(1, (int)std::ceil(std::sqrt((double)denseMatchesPerFrame))));
}
```

- [ ] **Step 4: Give the sampler the caller's pitch**

In `libs/SFM/ROMA2Warp.h`, add `int bucketGridSide,` to the declaration immediately before `unsigned maxSamples,`. In the header comment above it (`ROMA2Warp.h:194-213`), replace the phrase "with n sized from maxSamples -- this draw's budget" and the two bullets' surrounding text so it reads as the new rule. The comment must state, in prose:

- the pitch arrives from the caller (`DenseFillGridSide`) and is the same for every pair, so a pair's overlap holds `density * overlapArea` buckets and the draw is a density over the overlap rather than a count per pair;
- an occupied bucket still yields nothing, and *that* is what makes the draw a density over the **uncovered** overlap — the sparse matches take their share out of it with no separate coverage term;
- `maxSamples` remains the ceiling `ThinSampleEvenly` applies, and is what bounds the density in image B (the grid lives in A's frame and cannot see B's).

In `libs/SFM/ROMA2Warp.cpp`, add the parameter to the definition, and replace

```cpp
	// the grid is sized for THIS draw's budget, which is what makes the sample complementary in
	// scale as well as in position: a pair whose sparse matches already fill most of its budget
	// asks for few dense points and gets a coarse grid, one that has almost none asks for many and
	// gets a fine one
	const int numBuckets = WarpBucketGridSide(confidence, candidates.size(), maxSamples);
```

with

```cpp
	// the caller's pitch, the same for every pair (DenseFillGridSide): the draw is a density over
	// the overlap, so the number of buckets a pair's overlap holds is what varies, not the pitch
	ASSERT(bucketGridSide > 0);
	const int numBuckets = MINF(bucketGridSide, confidence.cols);
```

- [ ] **Step 5: Update the call site**

In `libs/SFM/MatchROMA2.cpp`, in the `SampleWarpComplementary` call inside `AssemblePairROMA2`, pass the pitch:

```cpp
		SampleWarpComplementary(imgA, imgB, inlierWarp, inlierConfidence, config.minConfidence,
			DenseFillGridSide(config.denseMatches, warpSize), config.denseMatches,
			occupiedA, dense.pointsA, dense.pointsB, dense.confidences);
```

(The ceiling is still the flat `config.denseMatches` here; Task 2 replaces it and renames the field.)

- [ ] **Step 6: Update `ROMA2ComplementaryDrawTest` to the new contract**

Four things change in that test, and nothing else:

1. Its `BucketGridSide` lambda (`apps/Tests/TestsSFM.cpp:1423-1425`) is deleted; every use becomes `DenseFillGridSide(<the density that draw uses>, cells)`.
2. Every `SampleWarpComplementary` call gains the pitch argument before the budget.
3. Case 1's expected winner count is no longer "the budget". Compute it exactly instead, which the test already has the pieces for: mark the buckets holding an eligible cell (`MarkCandidateBuckets`), strike out the buckets holding a `sparseA` point (`PixelToBucket`), and the draw must return exactly that many points, thinned to the budget if it is over. Assert that number, not an approximation.
4. Case 3 ("OVER BUDGET, WHERE THE THINNING SHOWS") drove the winners over the budget by making the grid finer through a small `E`. With a fixed pitch it does that directly: pass a deliberately fine pitch (`cells/2`) with a small `maxSamples` (400), so the unoccupied-bucket count far exceeds the budget and `ThinSampleEvenly` runs. The quadrant-spread assertions that follow are unchanged and are the point of the case.

The zero-budget case (`maxSamples == 0` clears the output arrays and returns 0) keeps its meaning — pass any valid pitch with it.

- [ ] **Step 7: Build and run the tests**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28 && ctest --test-dir /home/ubuntu/.claude/worktrees/roma2-onnx/make -C Release --output-on-failure`
Expected: warning-free build, all 3 suites PASS.

- [ ] **Step 8: Commit**

```bash
git add libs/SFM/ROMA2Warp.h libs/SFM/ROMA2Warp.cpp libs/SFM/MatchROMA2.cpp apps/Tests/TestsSFM.h apps/Tests/TestsSFM.cpp apps/Tests/Tests.cpp
git -c user.name='cDc' -c user.email='cdc.seacave@gmail.com' commit -m "sfm: fix the dense fill's bucket pitch so its draw is a density over the uncovered overlap"
```

---

### Task 2: The ceiling is the verdict's smaller inlier area, and the knob is a density

**Files:**
- Modify: `libs/SFM/MatchROMA2.h:57` (the config field), and the block after `PairVerdict` (~`:110`) for the new inline
- Modify: `libs/SFM/MatchROMA2.cpp` (the `SampleWarpComplementary` call in `AssemblePairROMA2`)
- Modify: `apps/CreateStructure/CreateStructure.cpp:176` (help text), `:369` (the assignment)
- Modify: `libs/SFM/PythonWrapper.cpp:313`
- Test: `apps/Tests/TestsSFM.cpp` (`ROMA2DenseFillCeilingTest`), `apps/Tests/TestsSFM.h`, `apps/Tests/Tests.cpp`

**Interfaces:**
- Consumes: `DenseFillGridSide` and the `SampleWarpComplementary` signature from Task 1.
- Produces: `ROMA2Config::denseMatchesPerFrame` (`unsigned`, default 2000) and `inline unsigned DenseFillCeiling(const ROMA2Config&, const PairVerdict&)` in `MatchROMA2.h`. Task 6 names both in the log record and the documentation.

- [ ] **Step 1: Write the failing test**

```cpp
bool ROMA2DenseFillCeilingTest()
{
	TD_TIMER_START();

	// The ceiling is the configured density over the SMALLER of the verdict's two inlier areas: the
	// bucket grid lives in A's frame and cannot see how much of B the pair's dense matches would
	// land on, so this is the only term that bounds their density in B.
	ROMA2Config config;
	config.denseMatchesPerFrame = 2000;
	PairVerdict verdict;

	verdict.inlierAreaA = 0.50f; verdict.inlierAreaB = 0.50f;
	const unsigned symmetric = DenseFillCeiling(config, verdict);
	verdict.inlierAreaA = 0.50f; verdict.inlierAreaB = 0.10f;
	const unsigned narrowB = DenseFillCeiling(config, verdict);
	verdict.inlierAreaA = 0.10f; verdict.inlierAreaB = 0.50f;
	const unsigned narrowA = DenseFillCeiling(config, verdict);
	verdict.inlierAreaA = 1.00f; verdict.inlierAreaB = 1.00f;
	const unsigned identical = DenseFillCeiling(config, verdict);
	if (symmetric != 1000 || narrowB != 200 || narrowA != 200 || identical != 2000) {
		VERBOSE("ROMA2DenseFillCeilingTest FAILED: ceilings %u/%u/%u/%u for .5|.5, .5|.1, .1|.5, 1|1",
			symmetric, narrowB, narrowA, identical);
		return false;
	}
	// the two sides are symmetric -- it is min(), not "A's area" -- and the density is linear in the
	// knob, which is what makes --roma2-dense-matches readable as matches per frame of overlap
	config.denseMatchesPerFrame = 500;
	verdict.inlierAreaA = 0.50f; verdict.inlierAreaB = 0.50f;
	if (DenseFillCeiling(config, verdict) != 250) {
		VERBOSE("ROMA2DenseFillCeilingTest FAILED: ceiling %u at density 500 over half an overlap",
			DenseFillCeiling(config, verdict));
		return false;
	}

	VERBOSE("ROMA2 dense fill ceiling test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
```

Declare in `apps/Tests/TestsSFM.h`, call from `apps/Tests/Tests.cpp` after `ROMA2DenseFillDensityTest`.

- [ ] **Step 2: Run the build to verify it fails**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28`
Expected: FAIL — no member `denseMatchesPerFrame`, `DenseFillCeiling` undeclared.

- [ ] **Step 3: Rename the field to what it now means**

In `libs/SFM/MatchROMA2.h`, replace

```cpp
	unsigned denseMatches = 2000;  // dense fill cap per pair
```

with

```cpp
	// Dense correspondences the fill may draw per FULL FRAME of overlap: a density, not a count per
	// pair. A pair's draw is that density over the part of its overlap its guided matches did not
	// already cover (DenseFillGridSide sets the pitch that makes it so), capped by DenseFillCeiling.
	unsigned denseMatchesPerFrame = 2000;
```

Rename every use: `MatchROMA2.cpp`'s call site, `CreateStructure.cpp:369`, `PythonWrapper.cpp:313` (the Python attribute becomes `dense_matches_per_frame`). There must be no occurrence of `denseMatches` left — grep for it.

- [ ] **Step 4: Add the ceiling**

In `libs/SFM/MatchROMA2.h`, immediately after the `PairVerdict` struct:

```cpp
// The most dense correspondences one pair may keep: the configured density over the SMALLER of the
// two inlier areas the verdict measured. The fill's bucket grid lives in image A's frame, so it
// bounds the dense keypoint density there and nowhere else -- but every dense correspondence costs a
// keypoint in B as well, and a warp that puts a large part of A onto a small part of B would pile
// them up in it. This is the term that stops that, and it binds only when B is the constraining
// frame: with inlierAreaA <= inlierAreaB it sits above the draw the pitch produces anyway, so it
// never charges the sparse matches' coverage a second time.
inline unsigned DenseFillCeiling(const ROMA2Config& config, const PairVerdict& verdict) {
	return (unsigned)ROUND2INT((float)config.denseMatchesPerFrame *
		MINF(verdict.inlierAreaA, verdict.inlierAreaB));
}
```

- [ ] **Step 5: Use it at the call site**

In `AssemblePairROMA2`, the call becomes

```cpp
		SampleWarpComplementary(imgA, imgB, inlierWarp, inlierConfidence, config.minConfidence,
			DenseFillGridSide(config.denseMatchesPerFrame, warpSize), DenseFillCeiling(config, verdict),
			occupiedA, dense.pointsA, dense.pointsB, dense.confidences);
```

Update the numbered comment above it (step "1) the dense fill") so it describes the density rule and names both terms; it currently says "cap `config.denseMatches`". Update the `AssemblePairROMA2` header comment in `MatchROMA2.h` the same way — it names `cap config.denseMatches` too.

- [ ] **Step 6: Rewrite the CLI help**

`apps/CreateStructure/CreateStructure.cpp:176` becomes:

```cpp
		("roma2-dense-matches", boost::program_options::value(&OPT::nROMA2DenseMatches)->default_value(2000), "dense matching: correspondences the dense fill adds per FULL FRAME of overlap; a pair draws that density over the part of its overlap its guided sparse matches did not already cover, capped by the smaller of the two inlier areas the verdict measured, so the dense keypoint density is bounded in both images; each costs a keypoint in both images plus a track")
```

- [ ] **Step 7: Build and run the tests**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28 && ctest --test-dir /home/ubuntu/.claude/worktrees/roma2-onnx/make -C Release --output-on-failure`
Expected: warning-free build, all 3 suites PASS.

- [ ] **Step 8: Commit**

```bash
git add libs/SFM/MatchROMA2.h libs/SFM/MatchROMA2.cpp libs/SFM/PythonWrapper.cpp apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.h apps/Tests/TestsSFM.cpp apps/Tests/Tests.cpp
git -c user.name='cDc' -c user.email='cdc.seacave@gmail.com' commit -m "sfm: bound the dense fill by the verdict's smaller inlier area and name the knob a density"
```

---

### Task 3: The two observation populations' robust reprojection sigmas

**Files:**
- Modify: `libs/SFM/Track.h` (declare after `ComputeReprojectionErrorPixels`, ~`:143`), `libs/SFM/Track.cpp` (define after `ComputeTracksMeanReprojectionError`, ~`:300`)
- Test: `apps/Tests/TestsSFM.cpp` (`ObservationSigmasTest`), `apps/Tests/TestsSFM.h`, `apps/Tests/Tests.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `SFM_API void ComputeObservationSigmas(const Scene& scene, double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense)`. Task 4 is its only caller.

- [ ] **Step 1: Write the failing test**

Build a synthetic posed scene with the existing helper `GenerateTestScene` (declared and used in `apps/Tests/TestsSFM.cpp` around `:4074` — take a small `SceneConfig`, 4 images and ~200 points, no descriptors), then close each image's described prefix and append dense keypoints displaced by a known multiple of the described ones' displacement:

```cpp
bool ObservationSigmasTest()
{
	TD_TIMER_START();

	// a posed synthetic scene whose observations are exact, so every reprojection error below is
	// one this test put there
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 4;
	cfg.numPoints = 200;
	GenerateTestScene(scene, cfg);

	// displace the described observations by 0.5 px and give every track a second, DENSE observation
	// of the same point displaced by 2.0 px: k = 4 by construction
	constexpr float describedError = 0.5f, denseError = 2.0f;
	for (Image& img : scene.images)
		img.SetNumDescribedKeypoints((uint32_t)img.keypoints.size());
	for (Track& track : scene.tracks) {
		const size_t numDescribedObs = track.observations.size();
		for (size_t i = 0; i < numDescribedObs; ++i) {
			const Observation obs = track.observations[i];
			Image& img = scene.images[obs.imageID];
			img.keypoints[obs.featureID].pt.x += describedError;
			const cv::KeyPoint dense(img.keypoints[obs.featureID].pt.x - describedError + denseError,
				img.keypoints[obs.featureID].pt.y, 10.f, -1.f, 0.9f);
			const uint32_t featID = (uint32_t)img.keypoints.size();
			img.keypoints.push_back(dense);
			track.observations.emplace_back(obs.imageID, featID);
		}
		track.numInliers = (uint8_t)MINF(track.observations.size(), (size_t)255);
	}

	double sigmaDescribed = 0, sigmaDense = 0;
	size_t numDescribed = 0, numDense = 0;
	ComputeObservationSigmas(scene, sigmaDescribed, numDescribed, sigmaDense, numDense);
	if (numDescribed == 0 || numDense != numDescribed) {
		VERBOSE("ObservationSigmasTest FAILED: %u described and %u dense observations",
			(unsigned)numDescribed, (unsigned)numDense);
		return false;
	}
	// the medians are the displacements this test applied, and their ratio is the k the weight is
	// computed from -- checked to a hundredth of a pixel, because nothing here is approximate
	if (ABS(sigmaDescribed - describedError) > 0.01 || ABS(sigmaDense - denseError) > 0.01) {
		VERBOSE("ObservationSigmasTest FAILED: sigmas %.4f described / %.4f dense against %.2f / %.2f",
			sigmaDescribed, sigmaDense, describedError, denseError);
		return false;
	}

	// a scene with no dense keypoints reports none, and reports the described population anyway
	Scene sparseOnly;
	GenerateTestScene(sparseOnly, cfg);
	ComputeObservationSigmas(sparseOnly, sigmaDescribed, numDescribed, sigmaDense, numDense);
	if (numDense != 0 || numDescribed == 0) {
		VERBOSE("ObservationSigmasTest FAILED: a scene with no dense keypoints reported %u of them",
			(unsigned)numDense);
		return false;
	}

	VERBOSE("Observation sigmas test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
```

Declare in `apps/Tests/TestsSFM.h`, call from `apps/Tests/Tests.cpp` next to the other bundle-adjustment tests (after `BAPinholeReprojectionJacobianTest`).

- [ ] **Step 2: Run the build to verify it fails**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28`
Expected: FAIL — `ComputeObservationSigmas` undeclared.

- [ ] **Step 3: Implement it**

`libs/SFM/Track.h`, after `ComputeReprojectionErrorPixels`:

```cpp
/**
 * Robust reprojection sigma of the two observation populations of the current solution, in pixels:
 * the median reprojection error over the observations on described keypoints, and over those on
 * dense (warp-sampled) ones, each computed with ComputeReprojectionErrorPixels -- the same formula
 * FilterTracks uses, so the two numbers describe the residuals bundle adjustment actually sees.
 * The median of an already non-negative error is taken as the scale directly, with no 1.4826
 * consistency factor: the only consumer takes the RATIO of the two, in which any common factor
 * cancels. Only inlier tracks and valid images take part, and an observation whose projection is
 * invalid is counted by neither population.
 * The counts come back so the caller can refuse a sample too small to be a sigma.
 */
SFM_API void ComputeObservationSigmas(const Scene& scene,
	double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense);
```

`libs/SFM/Track.cpp` — collect both populations in one pass over the tracks (the loop shape is `ComputeTracksMeanReprojectionError`'s, a few lines above), then take each median with `std::nth_element`:

```cpp
void SFM::ComputeObservationSigmas(const Scene& scene,
	double& sigmaDescribed, size_t& numDescribed, double& sigmaDense, size_t& numDense)
{
	std::vector<float> errorsDescribed, errorsDense;
	for (const Track& track : scene.tracks) {
		if (!track.IsInlier())
			continue;
		for (const auto& obs : track) {
			const Image& img = scene.images[obs.imageID];
			if (!img.IsValid())
				continue;
			ASSERT(obs.featureID < img.keypoints.size());
			const Point3 Xcam = img.TransformPointW2C(track.position);
			const auto [pixelError, valid] =
				ComputeReprojectionErrorPixels(*img.pCamera, Xcam, img.keypoints[obs.featureID].pt);
			if (!valid)
				continue;
			(img.IsDenseKeypoint(obs.featureID) ? errorsDense : errorsDescribed).push_back(pixelError);
		}
	}
	const auto Median = [](std::vector<float>& errors) {
		if (errors.empty())
			return 0.0;
		const size_t half = errors.size()/2;
		std::nth_element(errors.begin(), errors.begin() + half, errors.end());
		return (double)errors[half];
	};
	numDescribed = errorsDescribed.size();
	numDense = errorsDense.size();
	sigmaDescribed = Median(errorsDescribed);
	sigmaDense = Median(errorsDense);
}
```

- [ ] **Step 4: Build and run the tests**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28 && ctest --test-dir /home/ubuntu/.claude/worktrees/roma2-onnx/make -C Release --output-on-failure`
Expected: warning-free build, all 3 suites PASS.

- [ ] **Step 5: Commit**

```bash
git add libs/SFM/Track.h libs/SFM/Track.cpp apps/Tests/TestsSFM.h apps/Tests/TestsSFM.cpp apps/Tests/Tests.cpp
git -c user.name='cDc' -c user.email='cdc.seacave@gmail.com' commit -m "sfm: measure the reprojection sigma of the described and dense observation populations"
```

---

### Task 4: Bundle adjustment estimates the dense observation weight

**Files:**
- Modify: `libs/SFM/BundleAdjustment.h:56-71` (the config field and the paragraphs above it)
- Modify: `libs/SFM/BundleAdjustment.cpp:509-556` (`SelectReprojectionLoss` and the comment block above it), `:671-730` (`Adjust`), `:1068-1110` (local bundle adjustment)
- Modify: `apps/CreateStructure/CreateStructure.cpp:192` (help and default)
- Test: `apps/Tests/TestsSFM.cpp` (`DenseObservationWeightEstimateTest`), `apps/Tests/TestsSFM.h`, `apps/Tests/Tests.cpp`

**Interfaces:**
- Consumes: `ComputeObservationSigmas` from Task 3.
- Produces: `SFM_API double EstimateDenseObservationWeight(const Scene& scene, const BAConfig& config)` (declared in `BundleAdjustment.h`, defined in `BundleAdjustment.cpp`), and `SelectReprojectionLoss` taking the effective weight as a parameter. Nothing later consumes these.

- [ ] **Step 1: Write the failing test**

```cpp
bool DenseObservationWeightEstimateTest()
{
	TD_TIMER_START();

	// the scene of ObservationSigmasTest: described observations off by 0.5 px, dense ones by
	// 2.0 px, so k = 4 and the weight the estimator must return is 1/16
	Scene scene;
	SceneConfig cfg;
	cfg.numImages = 4;
	cfg.numPoints = 200;
	GenerateTestScene(scene, cfg);
	constexpr float describedError = 0.5f, denseError = 2.0f;
	for (Image& img : scene.images)
		img.SetNumDescribedKeypoints((uint32_t)img.keypoints.size());
	for (Track& track : scene.tracks) {
		const size_t numDescribedObs = track.observations.size();
		for (size_t i = 0; i < numDescribedObs; ++i) {
			const Observation obs = track.observations[i];
			Image& img = scene.images[obs.imageID];
			img.keypoints[obs.featureID].pt.x += describedError;
			const cv::KeyPoint dense(img.keypoints[obs.featureID].pt.x - describedError + denseError,
				img.keypoints[obs.featureID].pt.y, 10.f, -1.f, 0.9f);
			const uint32_t featID = (uint32_t)img.keypoints.size();
			img.keypoints.push_back(dense);
			track.observations.emplace_back(obs.imageID, featID);
		}
		track.numInliers = (uint8_t)MINF(track.observations.size(), (size_t)255);
	}

	BAConfig config;
	config.denseObservationWeight = -1.0; // estimate
	const double weight = EstimateDenseObservationWeight(scene, config);
	if (ABS(weight - 1.0/16.0) > 0.005) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: weight %.4f against 1/k^2 = %.4f for k = 4",
			weight, 1.0/16.0);
		return false;
	}

	// a dense population no less precise than the described one is not worth MORE than it
	for (Image& img : scene.images)
		for (uint32_t k = img.NumDescribedKeypoints(); k < img.keypoints.size(); ++k)
			img.keypoints[k].pt.x -= denseError - describedError;
	const double clamped = EstimateDenseObservationWeight(scene, config);
	if (clamped != 1.0) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: an equally precise dense population weighs %.4f",
			clamped);
		return false;
	}

	// and a scene with no dense keypoints at all falls back to the configured constant rather than
	// dividing by a sigma it does not have
	Scene sparseOnly;
	GenerateTestScene(sparseOnly, cfg);
	const double fallback = EstimateDenseObservationWeight(sparseOnly, config);
	if (fallback != DENSE_OBSERVATION_WEIGHT) {
		VERBOSE("DenseObservationWeightEstimateTest FAILED: fallback %.4f against the constant %.4f",
			fallback, DENSE_OBSERVATION_WEIGHT);
		return false;
	}

	VERBOSE("Dense observation weight estimate test passed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
```

Declare in `apps/Tests/TestsSFM.h`, call from `apps/Tests/Tests.cpp` right after `ObservationSigmasTest`.

- [ ] **Step 2: Run the build to verify it fails**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28`
Expected: FAIL — `EstimateDenseObservationWeight` undeclared.

- [ ] **Step 3: Turn the config field into an override**

`libs/SFM/BundleAdjustment.h` — replace the whole `denseObservationWeight` block (the paragraph starting "Loss weight of a reprojection residual on a DENSE (descriptor-less) keypoint" through the field) with:

```cpp
	// Loss weight of a reprojection residual on a DENSE (descriptor-less) keypoint relative to the
	// 1.0 a described one carries, as an OVERRIDE: negative (the default) means the weight is
	// measured on the scene each solve is about to fit, positive pins it -- which is what a sweep or
	// a regression test wants. It is a measurement-precision weight, not a fifth gate threshold, and
	// it follows the KEYPOINT rather than the match that created it (see SelectReprojectionLoss).
	// Ignored when useKeypointConfidence is set: that term expresses the same thing by another
	// route, and only one of the two may apply.
	// This is NOT the view graph's dense discount (PairsWeightingConfig::denseObservationWeight),
	// which answers a different question -- see DENSE_OBSERVATION_WEIGHT in ImagePair.h.
	double denseObservationWeight = -1.0;
```

Declare the estimator next to the `Adjust` entry points:

```cpp
// The weight a dense reprojection residual carries relative to a described one, measured rather than
// configured: 1/k^2 for k = sigma_dense/sigma_described, the two populations' robust reprojection
// sigmas on the scene as it stands (ComputeObservationSigmas). Returns config.denseObservationWeight
// when that is non-negative, and DENSE_OBSERVATION_WEIGHT when either population is too small to
// give a sigma or the scene carries no dense keypoints at all.
SFM_API double EstimateDenseObservationWeight(const Scene& scene, const BAConfig& config);
```

- [ ] **Step 4: Implement the estimator and rewrite the comment it replaces**

In `libs/SFM/BundleAdjustment.cpp`, **delete** the PROVISIONAL-default comment block (the paragraphs beginning "PROVISIONAL DEFAULT (BAConfig::denseObservationWeight = 0.25)" through "replace this default with the measured one") and the sentence in the block above it that says a `--ba-dense-weight` sweep must be run with `useKeypointConfidence` off. Keep the paragraphs that explain what the weight models and why it is exclusive with `useKeypointConfidence`; they are still true. In their place, above the new function:

```cpp
// 1/k^2 for k the ratio of the two populations' robust reprojection sigmas, measured on the scene
// this solve is about to fit. Both sigmas are read off the RAW pixel residuals, so the estimate does
// not carry the weighting the previous solve ran under, and it is recomputed at the head of every
// solve -- a reconstruction runs fifty or more of them, so it settles.
//
// Measured rather than configured because k is a property of the CAPTURE, not of the dense matcher:
// on the campaign's three captures the dense sigma barely moved (1.14-1.79 px, the warp's sampling
// scale) while the described sigma moved 3.6x, from 0.32 px where the texture is rich to 1.15 px
// where it is not, taking k from 1.55 to 4.02. No constant is right in both places.
//
// Clamped to [MIN_DENSE_OBSERVATION_WEIGHT, 1]: a dense correspondence is never a MORE precise
// measurement than a described one, and never worth nothing -- on a capture where the descriptor
// matcher is very good the ratio can run away, and a weight of zero would discard the only evidence
// a textureless region has. Falls back to the fixed DENSE_OBSERVATION_WEIGHT when either population
// is under MIN_SIGMA_OBSERVATIONS, which is the honest answer for an early incremental step whose
// scene is a handful of tracks.
constexpr size_t MIN_SIGMA_OBSERVATIONS = 100;
constexpr double MIN_DENSE_OBSERVATION_WEIGHT = 0.01;

double SFM::EstimateDenseObservationWeight(const Scene& scene, const BAConfig& config)
{
	if (config.denseObservationWeight >= 0.0)
		return config.denseObservationWeight;
	double sigmaDescribed = 0, sigmaDense = 0;
	size_t numDescribed = 0, numDense = 0;
	ComputeObservationSigmas(scene, sigmaDescribed, numDescribed, sigmaDense, numDense);
	if (numDescribed < MIN_SIGMA_OBSERVATIONS || numDense < MIN_SIGMA_OBSERVATIONS ||
		sigmaDescribed <= 0.0 || sigmaDense <= 0.0)
		return DENSE_OBSERVATION_WEIGHT;
	const double k = sigmaDense/sigmaDescribed;
	return CLAMP(1.0/(k*k), MIN_DENSE_OBSERVATION_WEIGHT, 1.0);
}
```

(If `CLAMP` is not available in this translation unit, use `MINF(MAXF(...))` in the codebase's own idiom — check what neighbouring code uses rather than adding an include.)

- [ ] **Step 5: Pass the resolved weight through both solves**

`SelectReprojectionLoss` takes the weight instead of reading it from the config: add a `double denseObservationWeight` parameter after `config`, and inside use it in place of `config.denseObservationWeight`. Its exclusivity check (`if (bDense && !config.useKeypointConfidence)`) is unchanged.

In `BundleAdjustment::Adjust`, before the residual loop (next to where `loss_function` is created), resolve it once:

```cpp
	// resolved once per solve, not per residual: the estimator walks every observation, and the
	// weight is a property of the scene this problem is built from, not of any one of its residuals
	const double denseWeight = EstimateDenseObservationWeight(scene, config);
```

and pass `denseWeight` at both `SelectReprojectionLoss` call sites. Do the same in the local bundle adjustment, using the same whole-scene estimator: a local window's described population is often too small to give a sigma, and two different weights inside one reconstruction would be worse than a slightly stale one — say that in a comment there.

Both DEBUG lines that report the dense residual count currently print `config.useKeypointConfidence ? 1.0 : config.denseObservationWeight`; they print `denseWeight` instead, and when the weight was estimated (`config.denseObservationWeight < 0`) they also print the two sigmas it came from, so a run's log says what the weight was measured on.

- [ ] **Step 6: Rewrite the CLI option**

`apps/CreateStructure/CreateStructure.cpp:192` becomes:

```cpp
		("ba-dense-weight", boost::program_options::value(&OPT::baDenseWeight)->default_value(-1.0), "bundle adjustment: loss weight of a reprojection residual on a dense (warp-sampled) keypoint, relative to the 1.0 a described one carries; negative (default) measures it as (sigma described/sigma dense)^2 on the scene each solve is about to fit, a value in [0,1] pins it (1 = no down-weighting)")
```

- [ ] **Step 7: Build and run the tests**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28 && ctest --test-dir /home/ubuntu/.claude/worktrees/roma2-onnx/make -C Release --output-on-failure`
Expected: warning-free build, all 3 suites PASS. `SFMPipelineTest` exercises reconstruction end to end with no dense keypoints, so it takes the fallback path — if it moves, the fallback is wrong.

- [ ] **Step 8: Commit**

```bash
git add libs/SFM/BundleAdjustment.h libs/SFM/BundleAdjustment.cpp apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.h apps/Tests/TestsSFM.cpp apps/Tests/Tests.cpp
git -c user.name='cDc' -c user.email='cdc.seacave@gmail.com' commit -m "sfm: measure the dense observation weight from the residuals instead of configuring it"
```

---

### Task 5: The view graph keeps its own dense discount

**Files:**
- Modify: `apps/CreateStructure/CreateStructure.cpp:391` (delete)
- Modify: `libs/SFM/ImagePair.h:58-61` (the constant) and `:116-124` (the `weightedInliers` paragraph)
- Modify: `libs/SFM/PairsWeighting.h:14` (the include comment), `:69` (the field's comment)
- Test: none new — this task deletes a coupling and rewrites comments; `SFMPipelineTest` covers that the weighting still runs.

**Interfaces:**
- Consumes: the `BAConfig::denseObservationWeight` semantics from Task 4 (negative = estimate), which is why the assignment must go.
- Produces: nothing new; `DENSE_OBSERVATION_WEIGHT` keeps its name, its value (0.25) and its type.

- [ ] **Step 1: Delete the coupling**

Remove `apps/CreateStructure/CreateStructure.cpp:391`:

```cpp
	cfg.matchCfg.weightingCfg.denseObservationWeight = (float)OPT::baDenseWeight;
```

It is a bug now rather than a simplification: `--ba-dense-weight` defaults to `-1` after Task 4, and a negative view-graph discount would make a dense-only pair's evidence negative.

- [ ] **Step 2: Rewrite the comments that call them one quantity**

`libs/SFM/ImagePair.h`, at `DENSE_OBSERVATION_WEIGHT`: state that it is the view graph's own constant — how much a dense correspondence is worth as EVIDENCE that two images see the same thing — and that bundle adjustment uses it only as the fallback for a weight it otherwise measures, because the two answer different questions. Give the reason in one sentence: a warp correspondence localizes a point several times less precisely than a descriptor one, which is what bundle adjustment charges it for, but it says nearly as much about whether the two images overlap, and charging the precision twice would demote exactly the pairs that carry a capture the descriptor matcher cannot match at all.

The `weightedInliers` paragraph at `:116-124` says the weighting pass is "the one holder of that weight (PairsWeightingConfig::denseObservationWeight, DENSE_OBSERVATION_WEIGHT above)" — it stays the one holder of the view graph's discount; make the sentence say that, and stop implying it is bundle adjustment's number.

`libs/SFM/PairsWeighting.h:14`'s include comment ("the one definition of the dense discount") and the field comment at `:69` get the same treatment. `PairsWeightingConfig::denseObservationWeight` keeps `DENSE_OBSERVATION_WEIGHT` as its default and gains one line saying it has no CLI flag because no measurement has asked for one, and that it is deliberately held fixed while the bundle adjustment weight moves.

- [ ] **Step 3: Build and run the tests**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28 && ctest --test-dir /home/ubuntu/.claude/worktrees/roma2-onnx/make -C Release --output-on-failure`
Expected: warning-free build, all 3 suites PASS.

- [ ] **Step 4: Commit**

```bash
git add libs/SFM/ImagePair.h libs/SFM/PairsWeighting.h apps/CreateStructure/CreateStructure.cpp
git -c user.name='cDc' -c user.email='cdc.seacave@gmail.com' commit -m "sfm: stop the view graph sharing bundle adjustment's dense weight"
```

---

### Task 6: The per-pair record and the documentation say the new rules

**Files:**
- Modify: `libs/SFM/MatchROMA2.cpp` (the `ROMA2 pair` log record and the one-pass summary's neighbours)
- Modify: `libs/SFM/README.md:188`
- Modify: `docs/design/ROMA2InProcess.md:39`, `:370`, `:521`, and the sections on the dense fill and on bundle adjustment
- Test: none new.

**Interfaces:**
- Consumes: `DenseFillCeiling` and `DenseFillGridSide` (Tasks 1-2), the estimated weight (Task 4), the split constant (Task 5).
- Produces: the log field list the measurement tooling parses.

- [ ] **Step 1: Add the two deciding numbers to the per-pair record**

The record is emitted in `MatchROMA2.cpp` and today reads

```
ROMA2 pair 0-1: conf 0.9523 0.9658 inl 0.9393 0.8929 ADMIT guided 749 sparse 736 dense 1642 301ms
```

Insert `cap %u grid %d` between the verdict word and `guided`, so a run's own log is enough to replay the rule that produced it:

```
ROMA2 pair 0-1: conf 0.9523 0.9658 inl 0.9393 0.8929 ADMIT cap 1786 grid 45 guided 749 sparse 736 dense 1642 301ms
```

A rejected pair draws nothing, so it keeps its present shorter form.

- [ ] **Step 2: Rewrite the documentation**

Four sites, rewritten so no text states the superseded rule beside the new one:

- `libs/SFM/README.md:188` — "the rest of the confident overlap is filled with up to `--roma2-dense-matches` (2000) warp correspondences drawn where the sparse matches are not" becomes the density rule: `--roma2-dense-matches` is correspondences per full frame of overlap, a pair draws that density over the part of its overlap its guided matches did not cover, and the verdict's smaller inlier area caps it so the density is bounded in both images.
- `docs/design/ROMA2InProcess.md:39` and `:370` — the flag tables' "dense fill cap per pair" becomes "dense correspondences per full frame of uncovered overlap"; the `--ba-dense-weight` row (if present in either table) becomes the estimate-by-default description.
- `docs/design/ROMA2InProcess.md:521` — the latency paragraph asserts the fill "runs at the `--roma2-dense-matches` cap (2000) on essentially every admitted pair". That measurement described the old rule. Replace the claim with what the new rule does and mark the per-pair latency numbers as measured under the flat cap, to be re-measured; the controller replaces them with the new campaign's numbers.
- The design document's sections on the dense fill and on bundle adjustment gain: the fixed pitch and why it makes the coverage discount exact, the ceiling and what it bounds, the estimated weight with the measured k of the three captures, and the reason the view graph keeps its own constant.

- [ ] **Step 3: Build and run the tests**

Run: `cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release -j 28 && ctest --test-dir /home/ubuntu/.claude/worktrees/roma2-onnx/make -C Release --output-on-failure`
Expected: warning-free build, all 3 suites PASS.

- [ ] **Step 4: Commit**

```bash
git add libs/SFM/MatchROMA2.cpp libs/SFM/README.md docs/design/ROMA2InProcess.md
git -c user.name='cDc' -c user.email='cdc.seacave@gmail.com' commit -m "sfm: record the dense fill's cap and pitch, and document both new rules"
```

---

## Measurement (the controller's, after the branch is green)

Not a task and not a subagent's: it runs the pipeline, which no implementer does. Per the spec's section 6 — two arms per capture on 8d2f4877, 38004114 and Truck, `openmvs-roma2-20260904-capdensity` (Tasks 1-2 only, `--ba-dense-weight 0.25` pinned) and `openmvs-roma2-20260904-capdensity-autoweight` (everything), against the recorded 2026-09-03 `onepass` runs. Results go to `<capture>/openmvs-roma2-20260904-<arm>/` with their logs. `task6lib.py`'s one-pass per-pair parser and `dense_cap_predict.py` take the new log field list first.

## Self-review

- **Spec coverage.** 3.1 → Tasks 1-2. 3.2 → Tasks 3-4. 3.3 → Task 5. 3.4 → Task 6 step 1. 3.5 → Task 6 step 2. Section 4's deletions: the per-draw grid rationale (Task 1 step 4), the PROVISIONAL block (Task 4 step 4), `CreateStructure.cpp:391` (Task 5 step 1), the `WarpBucketGridSide` call in the fill (Task 1 step 4), every use of `denseMatches` including the Python binding (Task 2 step 3). Section 5's six tests: 1, 2, 3 and 5 are `ROMA2DenseFillDensityTest` (Task 1), 4 is `ROMA2DenseFillCeilingTest` (Task 2), 6 is split into `ObservationSigmasTest` (Task 3) and `DenseObservationWeightEstimateTest` (Task 4). Section 6 → the Measurement section, controller-owned.
- **Names.** `DenseFillGridSide(unsigned, int)`, `DenseFillCeiling(const ROMA2Config&, const PairVerdict&)`, `ComputeObservationSigmas(const Scene&, double&, size_t&, double&, size_t&)`, `EstimateDenseObservationWeight(const Scene&, const BAConfig&)`, `ROMA2Config::denseMatchesPerFrame` — each defined in exactly one task and used under that spelling everywhere after it.
- **Ordering.** Tasks 1-2 both touch `MatchROMA2.cpp`'s call site and Tasks 4-5 both touch `CreateStructure.cpp`; they run in order, and Task 5 removes a line Task 4 leaves valid but wrong (a negative weight reaching the view graph), so Task 5 must not be reordered before Task 4.
