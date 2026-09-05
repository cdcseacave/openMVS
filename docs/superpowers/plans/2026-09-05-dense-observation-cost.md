# Dense Observation Cost Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop resection scoring dense observations against a threshold calibrated for descriptor
keypoints, without stopping a textureless capture from registering.

**Architecture:** One change, in `Resection::RegisterImage`. The PnP problem is built from described
observations; if an image has fewer than `minInliers` of them, the problem is rebuilt over all its
observations. Nothing else about a dense observation changes — it still triangulates, still enters
bundle adjustment at its estimated weight, still counts in the view graph at its own discount.

**Tech Stack:** C++17, OpenMVS SFM library, PoseLib bearing-vector PnP, the in-tree `Tests` binary.

**Spec:** `docs/superpowers/specs/2026-09-05-dense-observation-cost-design.md`

## Global Constraints

- **No backward compatibility, at any level.** Superseded code, config fields, CLI options and
  documentation text are deleted, not deprecated.
- **Commits are authored `cDc <cdc.seacave@gmail.com>` with no attribution trailer of any kind** —
  no `Co-Authored-By`, no session link, no mention of Claude or AI. This overrides any harness
  default.
- **No merge, no push, no PR.** The branch stays local.
- The build tree is `make/` (Ninja Multi-Config). Build and test Release. The `Tests` binary takes
  a suite number: bare `Tests` runs the generic unit tests and `Tests 1` runs the SFM suite, which
  is where every test in this plan lives. Running it bare will silently not run them.
- `grep` is aliased to ugrep on this machine: use `/usr/bin/grep`.
- No subagent dispatches subagents.
- **No process vocabulary in anything a user reads.** Ruling numbers, task numbers and finding IDs
  belong in the ledger, never in source comments, commit messages or documentation.

---

## File structure

| File | Responsibility after this plan |
|---|---|
| `libs/SFM/Resection.cpp` | `RegisterImage` selects the observation population; `RegisterImages` reports how often the fallback fired |
| `libs/SFM/Resection.h` | The rule and its fallback documented beside the config that bounds it |
| `apps/Tests/TestsSFM.cpp` | The selection and the fallback are pinned |
| `docs/design/ROMA2InProcess.md` | What a dense observation costs in each of its three consumers |

---

### Task 1: Resection fits on the population its threshold was calibrated for

**Files:**
- Modify: `libs/SFM/Resection.cpp:68-120` (`RegisterImage`), `:163-266` (`RegisterImages` reporting)
- Modify: `libs/SFM/Resection.h:39-64` (`ResectionConfig` documentation)
- Test: `apps/Tests/TestsSFM.cpp` (`ResectionDenseObservationTest`)

**Interfaces:**
- Consumes: `Image::IsDenseKeypoint(uint32_t)` and `Image::HasDenseKeypoints()`, already on `Image`
  (`libs/SFM/Image.h:106-112`).
- Produces: `Resection::numDenseFallbacks`, a counter read only by the final report line.

- [ ] **Step 1: Write the failing test**

Add `ResectionDenseObservationTest` to `apps/Tests/TestsSFM.cpp` and register it in
`apps/Tests/Tests.cpp`. It pins the selection rule directly, without running RANSAC, by exposing
the selection as its own function in step 3 — so write the test against that name now:

```cpp
bool ResectionDenseObservationTest()
{
	// One image whose keypoints are half described, half dense, seen by tracks of both kinds.
	// The rule: the PnP problem takes the described observations only; below minInliers of them
	// it takes every observation instead, because on a textureless capture the dense segment is
	// not a supplement, it is the whole evidence.
	Scene scene;
	// ... build one image with numDescribedKeypoints = 20 and 40 keypoints total, and 30 inlier
	// tracks observing it: 20 on described features, 10 on dense ones. Follow the scene
	// construction of the existing reconstruction tests in this file rather than inventing one.

	// 1. Plenty of described observations: the dense ones are excluded.
	IIndexArr featureIDs;
	bool usedFallback = true;
	SelectResectionObservations(scene, imageID, /*minInliers*/ 12, featureIDs, usedFallback);
	if (featureIDs.size() != 20 || usedFallback) {
		VERBOSE("ResectionDenseObservationTest FAILED: took %u observations (fallback %s), expected 20 described and no fallback",
			(unsigned)featureIDs.size(), usedFallback ? "yes" : "no");
		return false;
	}

	// 2. Too few described observations: every observation is taken, and the fallback is reported.
	SelectResectionObservations(scene, imageID, /*minInliers*/ 25, featureIDs, usedFallback);
	if (featureIDs.size() != 30 || !usedFallback) {
		VERBOSE("ResectionDenseObservationTest FAILED: took %u observations (fallback %s), expected all 30 with the fallback",
			(unsigned)featureIDs.size(), usedFallback ? "yes" : "no");
		return false;
	}

	// 3. An image with no dense keypoints at all is unaffected and never reports a fallback.
	// ... assert the same call on a described-only image returns every observation, fallback false.
	return true;
}
```

- [ ] **Step 2: Run it and watch it fail to compile**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests 2>&1 | tail -10
```

Expected: `SelectResectionObservations` is undeclared.

- [ ] **Step 3: Extract the selection, then apply the rule**

`RegisterImage` currently walks `scene.tracks` and pushes a bearing and a 3D point for every inlier
track observing the image. Split that walk into a selection that is testable on its own, in
`libs/SFM/Resection.cpp` above `RegisterImage`, declared in the same translation unit's anonymous
namespace only if the test can still reach it — otherwise declare it in `Resection.h` beside the
class. Prefer the header: a rule worth testing is worth naming.

```cpp
// The observations resection fits on. PoseLib's absolute-pose estimator takes ONE angular
// threshold, derived from a reprojection threshold tuned for descriptor keypoints, so it has no
// way to score two populations of different accuracy against their own tolerances. The described
// keypoints are the population that threshold was calibrated for -- median reprojection error
// 0.67-1.01 px against a dense segment whose positions are warp-cell accurate -- so the problem is
// built from them, and the dense observations stop failing an inlier test that was never about
// them.
//
// The fallback is part of the rule, not a safety net: on a textureless capture the dense segment
// is not a supplement but the entire evidence -- SIFT registers no images there while the dense
// pass registers 248 of 311 -- so an image with fewer than `minInliers` described observations
// takes every observation it has rather than not registering at all.
void SelectResectionObservations(const Scene& scene, IIndex imageID, unsigned minInliers,
	IIndexArr& featureIDs, bool& usedFallback);
```

Implementation: walk the tracks twice at most. The first walk collects described observations only;
if it yields at least `minInliers`, that is the answer and `usedFallback` is false. Otherwise a
second walk collects every observation and sets `usedFallback` to true. An image with
`!HasDenseKeypoints()` has no dense observations to exclude, so the first walk already returns
everything and the fallback never fires — assert that rather than special-casing it.

Then `RegisterImage` builds `bearings`/`points3D` from the returned feature IDs, keeping the
existing `break` on the first observation of the track for that image, and returns
`{numInliers, n}` exactly as before — `n` is now the size of the *selected* population, which is
what makes the reported inlier ratio a statistic about the population its threshold was calibrated
for, and comparable to the pre-dense numbers at all.

- [ ] **Step 4: Report how often the fallback fired**

`RegisterImage` gains an out-parameter or `Resection` gains a member counter — prefer the member,
since `RegisterImage`'s pair return is already load-bearing. Increment it when
`SelectResectionObservations` reports the fallback, and add it to the existing report line at the
end of `RegisterImages`:

```cpp
	DEBUG("Resection registered %u new images, total %u/%u images, %u fitted on every observation "
		"for want of described ones (%s)",
		registeredCount, scene.status.nCalibratedImages, scene.images.size(), numDenseFallbacks,
		TD_TIMER_GET_FMT().c_str());
```

An arm needs this number to tell a textureless capture (the fallback fires on nearly every image,
which is the expected result there) from a textured one (it should never fire).

- [ ] **Step 5: Document the rule where the config lives**

In `libs/SFM/Resection.h`, beside `minInliers` and `ransac.threshold`, state that the threshold is
calibrated for described keypoints and that `minInliers` is now also the fallback's trigger. A
reader changing `minInliers` must be able to see that it moves two things.

- [ ] **Step 6: Run the tests**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | tail -20
```

Expected: the whole suite passes, exit 0. `ROMA2ReconstructTest` runs both a described-only and a
dense arm — if the dense arm's registration changes, read why before assuming it is fine: fewer
registered images there is the risk this rule carries, and the test is where it surfaces first.

- [ ] **Step 7: Commit**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && git add -A && git commit -m "sfm: resection fits on the observations its threshold was calibrated for

PoseLib's absolute-pose estimator takes one angular threshold, derived
from a reprojection threshold tuned for descriptor keypoints. Dense
observations are warp-cell accurate, so they were failing an inlier test
that was never about them and taking the reported ratio down with them.

The problem is now built from the described observations; an image with
fewer than minInliers of those falls back to all of them, because on a
textureless capture the dense segment is not a supplement but the whole
evidence. The fallback count is reported, so an arm can tell the two
cases apart."
```

---

### Task 2: What a dense observation costs, in one place

**Files:**
- Modify: `docs/design/ROMA2InProcess.md`

**Interfaces:**
- Consumes: Task 1's shipped behaviour.
- Produces: nothing code reads.

- [ ] **Step 1: Find where the document already discusses dense observations**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && /usr/bin/grep -n -i "dense observation\|dense keypoint\|dense weight\|resection" docs/design/ROMA2InProcess.md
```

Read every hit. The document already describes the bundle adjustment weight and the view-graph
discount; what it does not have is the third consumer.

- [ ] **Step 2: State all three consumers together**

A dense observation is now treated differently in three places, and a reader who finds one of them
should find the other two. Write one short section giving, for each: what it does, and why.

- **Resection** — excluded from the PnP problem, with the all-observation fallback below
  `minInliers` described observations. Because the estimator has one threshold and it is calibrated
  for descriptor keypoints.
- **Bundle adjustment** — down-weighted at a weight estimated per solve from the two populations'
  robust reprojection sigmas. Because the two populations have genuinely different accuracy and the
  ratio is measurable rather than assumable.
- **The view graph** — its own discount, split off from the bundle adjustment weight. Because a
  pair's weight in the graph and an observation's weight in a solve are different questions that
  happened to share a constant.

Do not restate the measured numbers that already appear elsewhere in the document; point at them.

- [ ] **Step 3: Commit**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && git add -A && git commit -m "docs: what a dense observation costs in each of its three consumers

Resection, bundle adjustment and the view graph now each treat it
differently and for different reasons; a reader who finds one should
find the other two."
```

---

## Measurement (the controller's, after the branch is green)

Not tasks and not a subagent's. Spec §4.

1. **The density sweep** — `--roma2-dense-matches` at 500 / 1000 / 2000 / 4000 on 38004114, one
   frozen binary for every point, run folders `<capture>/openmvs-roma2-20260905-density-<N>/`.
   **Already running** at plan time; it measures the pre-Task-1 pipeline, which is the right
   baseline for the density question and is stated as such in the report.
2. **Truck as the control** — the chosen density plus 2000, because scene type dominates difficulty
   on every other measurement this branch has made.
3. **The hard captures** — `OfficeBadLoop`, `HouseBadDrift`, `House4Levels`, at the chosen density,
   after the sweep. Running them at 2000 first would spend days measuring a configuration the sweep
   may reject.
4. **Task 1's effect** — median and p10 resection inlier ratio, registered images, and the fallback
   count, on both a textured and a textureless capture.

## Self-review

- **Spec coverage.** §3.1 → Task 1. §3.2 changes no code and is the Measurement section. §4 → the
  same. §5's four rows → Tasks 1 and 2.
- **Placeholders.** Task 1 step 1 leaves the test scene's construction to "follow the existing
  reconstruction tests in this file" rather than inventing a second convention for building a
  `Scene` with tracks; the assertions, the counts (20 described, 10 dense, 30 total) and the two
  `minInliers` values are all written out, so the test's *content* is specified even though its
  scaffolding is borrowed.
- **Names.** `SelectResectionObservations(const Scene&, IIndex, unsigned, IIndexArr&, bool&)` and
  `Resection::numDenseFallbacks` — each defined once and used under that spelling.
- **Ordering.** Task 2 documents Task 1 and must follow it.
