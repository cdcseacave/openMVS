# View-Graph Disambiguation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the camera-triplet filter remove edges only for bad evidence, pick its threshold from
the survivor graph instead of a constant, and export a second independent per-edge cue.

**Architecture:** Everything lands in the one file that already owns the method,
`libs/SFM/ViewGraphTriplets.{h,cpp}`, plus the CLI, the Python bindings, the pairs CSV writer and
the tests. The scoring pass (`ComputeTripletScores`) is untouched — the scores were never the
problem — so the changes are: the removal predicate, a survivor-graph evaluator and a sweep that
uses it, and one new independent scoring function that reuses the same edge list and sorted
adjacency the triplet enumeration already builds.

**Tech Stack:** C++17, OpenMVS SFM library, Boost.ProgramOptions, boost::python bindings, the
in-tree `Tests` binary.

**Spec:** `docs/superpowers/specs/2026-09-05-view-graph-disambiguation-design.md`

## Global Constraints

- **No backward compatibility, at any level.** Superseded code, config fields, CLI options and
  documentation text are deleted, not deprecated.
- **Commits are authored `cDc <cdc.seacave@gmail.com>` with no attribution trailer of any kind** —
  no `Co-Authored-By`, no session link, no mention of Claude or AI. This overrides any harness
  default.
- **No merge, no push, no PR.** The branch stays local.
- Never write into a dataset capture's `keyframes/`; run artifacts go to
  `<capture>/<descriptive-run-name>/` with the log beside them. Nothing this plan builds writes
  outside the worktree.
- The build tree is `make/` (Ninja Multi-Config). Build and test Release.
- `grep` is aliased to ugrep on this machine: use `/usr/bin/grep`.
- No subagent dispatches subagents.
- Never edit a running shell script in place.
- **No process vocabulary in anything a user reads.** Ruling numbers, task numbers and finding IDs
  belong in the ledger, never in source comments, commit messages or documentation.

---

## File structure

| File | Responsibility after this plan |
|---|---|
| `libs/SFM/ViewGraphTriplets.h` | The config, the score struct, the survivor-graph struct, and the four entry points: score, evaluate a survivor graph, score the bipartite cue, filter |
| `libs/SFM/ViewGraphTriplets.cpp` | All of the above. The file gains a shared private helper that builds the edge list and sorted adjacency, because two scoring passes now need it |
| `libs/SFM/PairsMatcher.cpp` | `ExportPairsCSV` writes the second cue beside `TripletScore` |
| `apps/CreateStructure/CreateStructure.cpp` | `--triplet-auto-tau`; corrected help for the two existing options |
| `libs/SFM/PythonWrapper.cpp` | `auto_tau` on the config; the cue exposed beside `compute_triplet_scores` |
| `apps/Tests/TestsSFM.cpp` | `TripletFilterTest` extended; the path-graph assertion flipped |
| `docs/design/TripletDisambiguation.md` | Rewritten to describe the shipped rule, the threshold and the two cues |

---

### Task 1: An edge is removed for bad evidence, never for absent evidence

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.cpp:228-268` (`FilterPairsByTriplets`)
- Modify: `libs/SFM/ViewGraphTriplets.h:63-80` (the two header comments)
- Modify: `apps/CreateStructure/CreateStructure.cpp:157` (`--filter-triplets` help)
- Test: `apps/Tests/TestsSFM.cpp:7718-7885` (`TripletFilterTest`)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: the removal rule every later task assumes — an unscored pair (`score < 0`) is kept.
  `FilterPairsByTriplets` still returns the number of pairs removed.

- [ ] **Step 1: Update the three existing filter assertions first**

`TripletFilterTest` has three cases that count removals, and **all three change**. Do not assume
only the path-graph case moves — the 8-node graph has five unscored edges too, and they were being
counted among the removals.

The scene is `pairSpecs`: `(0,1)100 (0,2)100 (1,2)70 (1,3)50 (2,3)40 (3,4)30 (3,5)20 (5,6)10
(6,7)10 (5,7)10`. Its triplets are `{0,1,2}`, `{1,2,3}` (one component, sharing edge `(1,2)`) and
`{5,6,7}` (isolated, so outside the largest component). Five pairs are scored —
`(0,1)=1.0 (0,2)=1.0 (1,2)=0.85 (1,3)=0.714 (2,3)=0.571` — and five are unscored:
`(3,4) (3,5) (5,6) (6,7) (5,7)`.

**Case (c), m = 0.3, tau = 0.825.** Scored below tau: `(1,3)`, `(2,3)`. Was 7 removed / 3 kept;
becomes **2 removed / 8 kept**:

```cpp
	// Only the two scored pairs below tau go. The five pairs in no triplet of the largest
	// component are unscored -- no evidence either way -- and the filter keeps them.
	if (FilterPairsByTriplets(scene, filterCfg, weightingCfg) != 2 || scene.pairs.size() != 8) {
		VERBOSE("TripletFilterTest FAILED: m=0.3 left %u pairs, expected 8", scene.pairs.size());
		return false;
	}
	const std::set<std::pair<IIndex,IIndex>> expectedKept03{
		{0,1}, {0,2}, {1,2}, {3,4}, {3,5}, {5,6}, {6,7}, {5,7}};
```

**Case (d), m = 0.6, tau = 0.9.** Scored below tau: `(1,2)=0.85`, `(1,3)`, `(2,3)`. Was 8 removed /
2 kept; becomes **3 removed / 7 kept**:

```cpp
	if (FilterPairsByTriplets(scene06, filterCfg, weightingCfg) != 3 || scene06.pairs.size() != 7) {
		VERBOSE("TripletFilterTest FAILED: m=0.6 left %u pairs, expected 7", scene06.pairs.size());
		return false;
	}
	const std::set<std::pair<IIndex,IIndex>> expectedKept06{
		{0,1}, {0,2}, {3,4}, {3,5}, {5,6}, {6,7}, {5,7}};
```

**Case (e), the path graph.** No triplet at all, so nothing is scored and nothing is removed. Was 3
removed / 0 kept; becomes **0 removed / 3 kept**. Its comment currently reads "the filter empties
it" — rewrite that too:

```cpp
	// (e) a graph with no triplet at all scores nothing, so the filter has no evidence to act on
	// and leaves every pair in place
	...
	if (FilterPairsByTriplets(scenePath, filterCfg, weightingCfg) != 0 || scenePath.pairs.size() != 3) {
		VERBOSE("TripletFilterTest FAILED: a triplet-free graph lost %u of 3 pairs",
			3u - (unsigned)scenePath.pairs.size());
		return false;
	}
```

Cases (a) and (b) assert scores and statistics, which this task does not touch, and case (f) asserts
the disabled filter is a no-op. All three stay exactly as they are.

- [ ] **Step 2: Run the test and watch all three cases fail**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 2>&1 | /usr/bin/grep -i -B2 -A3 "TripletFilterTest"
```

Expected: `TripletFilterTest FAILED: m=0.3 left 3 pairs, expected 8`. If it passes, the edit did not
land — stop and re-read the file.

- [ ] **Step 3: Change the removal rule**

In `FilterPairsByTriplets`, the loop currently removes on `score < tripletScores.tau`, and an
unscored pair carries `-1`, so it is removed by the same comparison. Separate the two cases:

```cpp
	const unsigned numPairs = scene.pairs.size();
	unsigned numUnscored = 0, numBelowTau = 0, numKept = 0;
	for (unsigned idxPair = 0; idxPair < numPairs; ++idxPair) {
		const float score = tripletScores.scores[idxPair];
		// An unscored pair is one the method has no evidence about: it takes part in no triangle,
		// or in none inside the largest triplet-graph component. Those are overwhelmingly TRUE
		// pairs -- 426 of 490 and 415 of 441 on the two labelled references -- so absence of
		// evidence keeps the pair. Only a scored pair below tau is removed.
		if (score < 0.f) {
			++numUnscored;
		} else if (score < tripletScores.tau) {
			++numBelowTau;
			continue;
		}
		if (numKept != idxPair)
			scene.pairs[numKept] = std::move(scene.pairs[idxPair]);
		++numKept;
	}
	const unsigned numRemoved = numPairs - numKept;
	ASSERT(numRemoved == numBelowTau, "FilterPairsByTriplets: removal count mismatch");
```

- [ ] **Step 4: Correct the log line**

The line must stop reporting unscored pairs among the removals:

```cpp
	VERBOSE("Triplet filter: kept %u/%u pairs (tau %.3f from m %.2f; %u nodes, max degree %u; "
		"%u triplets in %u components; %u below tau removed, %u unscored kept)",
		numKept, numPairs, tripletScores.tau, config.minScore,
		tripletScores.numNodes, tripletScores.maxDegree,
		tripletScores.numTriplets, tripletScores.numTripletComponents, numBelowTau, numUnscored);
```

- [ ] **Step 5: Correct every comment and help string that promises the old behaviour**

Three places currently state the old rule and become wrong the moment step 3 lands. Read each and
rewrite the claim, do not merely soften it:

1. `libs/SFM/ViewGraphTriplets.h`, the `ComputeTripletScores` comment ends "...and every edge
   outside G_LCT, including the edges in no triplet at all — stays unscored (-1)." That sentence is
   still true and stays. The `FilterPairsByTriplets` comment below it begins "Apply Algorithm 1
   step 10 to the scene: remove the pairs scoring below tau *and* the unscored" — rewrite it to
   state that only scored pairs below tau are removed, and that this is a deliberate departure from
   the paper's step 1, with the labelled-reference counts as the reason.
2. `apps/CreateStructure/CreateStructure.cpp:157`, the `--filter-triplets` help ends "...and the
   pairs in no triangle at all". Delete that clause; the option now removes only pairs whose inlier
   count is systematically weak in the triangles they belong to.
3. Search for any other statement of the old rule before finishing:

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && /usr/bin/grep -rn -i "no triangle\|unscored" --include=*.h --include=*.cpp --include=*.md . | /usr/bin/grep -v "^./make"
```

Read every hit. A hit that describes the *scoring* pass leaving an edge unscored is correct and
stays; a hit that says the *filter* removes it is now wrong.

- [ ] **Step 6: Run the test and watch it pass**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 2>&1 | tail -20
```

Expected: the whole suite passes, exit 0, with cases (c), (d) and (e) now asserting the counts from
step 1. Cases (a), (b) and (f) must be untouched — a change there means the edit reached the scoring
pass or the disabled path, neither of which this task touches.

- [ ] **Step 7: Commit**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && git add -A && git commit -m "sfm: the triplet filter removes an edge for bad evidence, never for absent evidence

Algorithm 1 step 1 discards every edge outside the largest triplet-graph
component, which includes every edge in no triangle at all. On the two
labelled references those are overwhelmingly true pairs -- 426 of 490
unscored pairs on one, 415 of 441 on the other -- so the step was costing
correct edges to remove nothing. An unscored pair is now kept; only a
scored pair below tau is removed.

The scores are untouched: they were never the weak part."
```

---

### Task 2: The threshold comes from the survivor graph

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`TripletFilterConfig::autoTau`, `SurvivorGraph`, `EvaluateSurvivorGraph`)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`EvaluateSurvivorGraph`, the sweep in `FilterPairsByTriplets`)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (`--triplet-auto-tau`, corrected `--triplet-min-score` help)
- Modify: `libs/SFM/PythonWrapper.cpp` (`auto_tau`)
- Test: `apps/Tests/TestsSFM.cpp` (`TripletAutoTauTest`)

**Interfaces:**
- Consumes: Task 1's removal rule — the survivor graph at a threshold keeps every unscored pair.
- Produces: `SurvivorGraph`, `EvaluateSurvivorGraph(const Scene&, const std::vector<float>&, float)`
  and `TripletFilterConfig::autoTau`, all used by Task 4's documentation and by the offline
  analysis.

- [ ] **Step 1: Write the failing test**

Add `TripletAutoTauTest` to `apps/Tests/TestsSFM.cpp`, beside `TripletFilterTest`, and register it
in `apps/Tests/Tests.cpp` next to the `TripletFilterTest` call. It pins three things: the evaluator,
a sweep that picks a threshold, and a sweep that stands down.

```cpp
bool TripletAutoTauTest()
{
	// Two triangles joined by one bridge edge. The bridge sits in no triangle, so Task 1's rule
	// keeps it whatever the threshold does, and the two triangles are what the sweep can cut.
	//   0-1-2 triangle (strong, 100 inliers each), 3-4-5 triangle (strong, 100 each),
	//   2-3 bridge (weak, 10), plus a weak edge 0-2 replaced below to make one triangle cuttable.
	Scene scene;
	const auto AddPair = [&scene](IIndex i, IIndex j, unsigned inliers) { ... };
	// (build 6 images and the edges above; follow TripletFilterTest's own scene construction
	// verbatim for how a pair is given geometric verification and an inlier count)

	const TripletScores scores = ComputeTripletScores(scene, 0.f);

	// 1. The unfiltered graph: tau = 0 keeps every scored pair, and unscored pairs are always
	// kept, so this is the graph the sweep measures itself against.
	const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, scores.scores, 0.f);
	if (unfiltered.numNodes != 6 || unfiltered.largestComponent != 6) {
		VERBOSE("TripletAutoTauTest FAILED: unfiltered graph %u nodes, largest component %u, expected 6 and 6",
			unfiltered.numNodes, unfiltered.largestComponent);
		return false;
	}

	// 2. A threshold above every score removes every scored edge; only the unscored bridge
	// survives, so the graph fragments and the low-degree count explodes.
	const SurvivorGraph shredded = EvaluateSurvivorGraph(scene, scores.scores, 1.01f);
	if (shredded.largestComponent != 2 || shredded.numLowDegree != 6) {
		VERBOSE("TripletAutoTauTest FAILED: shredded graph largest component %u, low-degree %u, expected 2 and 6",
			shredded.largestComponent, shredded.numLowDegree);
		return false;
	}

	// 3. Auto-tau on this graph must stand down: no threshold both removes something and keeps
	// 0.99 of the largest component with under 1% of nodes below degree 2.
	TripletFilterConfig cfg;
	cfg.enabled = true;
	cfg.autoTau = true;
	cfg.minScore = 0.3f;
	Scene sceneAuto(scene);
	const PairsWeightingConfig weightingCfg;
	if (FilterPairsByTriplets(sceneAuto, cfg, weightingCfg) != 0 || sceneAuto.pairs.size() != scene.pairs.size()) {
		VERBOSE("TripletAutoTauTest FAILED: auto-tau removed %u pairs on a graph where every threshold fragments",
			(unsigned)(scene.pairs.size() - sceneAuto.pairs.size()));
		return false;
	}
	return true;
}
```

Build the scene so that case 3 genuinely stands down; if the graph you build admits a safe
threshold, the test asserts nothing. Verify by printing the sweep's own per-candidate decision once
while developing, then remove the print.

- [ ] **Step 2: Run it and watch it fail to compile**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests 2>&1 | tail -20
```

Expected: `SurvivorGraph`/`EvaluateSurvivorGraph`/`autoTau` are undeclared.

- [ ] **Step 3: Declare the survivor graph in the header**

In `libs/SFM/ViewGraphTriplets.h`, add `autoTau` to the config and document `minScore`'s new
meaning:

```cpp
struct SFM_API TripletFilterConfig
{
	bool enabled = false;   // remove the pairs the triplet score rejects (opt-in, see docs/design/TripletDisambiguation.md)
	// The sweep below picks the threshold from the graph it would leave behind, because the
	// paper's constant is the part that fails on video keyframes: it is not calibrated for them,
	// and at m = 0.6 every measured capture loses registered images.
	bool autoTau = true;
	// With autoTau, the FLOOR of the sweep -- the most permissive m it will consider. Without it,
	// the threshold itself: the paper's minimum edge score m (0.6 generic/large-scale, 0.9 highly
	// ambiguous, 0.3 medium/small ambiguous).
	float minScore = 0.6f;
};

// The view graph the filter would leave behind at a given threshold: what the sweep judges.
struct SFM_API SurvivorGraph
{
	unsigned numNodes;          // images incident to at least one edge of the UNFILTERED graph
	unsigned largestComponent;  // images in the largest connected component of the kept edges
	unsigned numLowDegree;      // of those nodes, how many have degree < 2 in the kept graph
	unsigned numKept;           // kept edges
};

// Evaluate the graph left by keeping every unscored pair and every pair scoring at or above `tau`.
// Pass tau = 0 for the unfiltered graph: scores lie in [0,1] and unscored pairs are always kept.
// Nodes are counted on the unfiltered graph, so an image that loses all its edges still counts as
// a node -- with degree 0, which is exactly what the low-degree test is there to catch.
SurvivorGraph SFM_API EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau);
```

- [ ] **Step 4: Implement the evaluator**

In `libs/SFM/ViewGraphTriplets.cpp`, after `ComputeTripletScores`. It is a union-find over images,
not over edges — a different graph from `EdgeUnionFind`, so do not reuse that class:

```cpp
SurvivorGraph SFM::EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau)
{
	SurvivorGraph result{0, 0, 0, 0};
	ASSERT(scores.size() == scene.pairs.size(), "EvaluateSurvivorGraph: one score per scene pair");
	const IIndex numImages = scene.images.size();
	if (numImages == 0)
		return result;
	// degree in the kept graph, and membership of the unfiltered one, in a single pass
	std::vector<unsigned> degree(numImages, 0);
	std::vector<bool> isNode(numImages, false);
	std::vector<uint32_t> parent(numImages);
	FOREACH(i, parent)
		parent[i] = (uint32_t)i;
	const std::function<uint32_t(uint32_t)> Find = [&](uint32_t x) {
		while (parent[x] != x)
			x = parent[x] = parent[parent[x]];
		return x;
	};
	FOREACH(idxPair, scene.pairs) {
		const ImagePair& pair = scene.pairs[idxPair];
		if (!pair.HasGeometricVerification() || pair.GetNumWeightedInliers() == 0 || pair.ID1 == pair.ID2)
			continue;
		isNode[pair.ID1] = isNode[pair.ID2] = true;
		const float score = scores[idxPair];
		if (score >= 0.f && score < tau)
			continue; // removed: scored, and below the threshold
		++degree[pair.ID1];
		++degree[pair.ID2];
		++result.numKept;
		const uint32_t a = Find(pair.ID1), b = Find(pair.ID2);
		if (a != b)
			parent[MAXF(a, b)] = MINF(a, b);
	}
	std::unordered_map<uint32_t, unsigned> componentSize;
	for (IIndex i = 0; i < numImages; ++i) {
		if (!isNode[i])
			continue;
		++result.numNodes;
		if (degree[i] < 2)
			++result.numLowDegree;
		result.largestComponent = MAXF(result.largestComponent, ++componentSize[Find((uint32_t)i)]);
	}
	return result;
}
```

Note the component count only walks nodes of the unfiltered graph, so an isolated image that was
never matched at all does not inflate anything.

- [ ] **Step 5: Sweep in the filter**

At the top of `FilterPairsByTriplets`, after `ComputeTripletScores`, replace the single `tau` with
the sweep. Eqn. 3 maps m to tau through the connectivity of `G_LCT`, and `TripletScores` already
carries both terms, so no rescoring is needed:

```cpp
	const TripletScores tripletScores = ComputeTripletScores(scene, config.minScore);
	float tau = tripletScores.tau;
	if (config.autoTau) {
		// Eqn. 3 on G_LCT: tau(m) = m (1 - d_max/|V|) + d_max/|V|, monotone in m, so the
		// strictest safe m is the first accepted candidate on a downward sweep.
		const float degreeRatio = tripletScores.numNodes > 0
			? (float)tripletScores.maxDegree / (float)tripletScores.numNodes : 0.f;
		const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
		// A candidate is safe when it neither fragments the reconstruction nor strands images:
		// it keeps 99% of the largest component and leaves under 1% of nodes below degree 2.
		const unsigned minComponent = (unsigned)std::ceil(0.99 * (double)unfiltered.largestComponent);
		const unsigned maxLowDegree = (unsigned)std::floor(0.01 * (double)unfiltered.numNodes);
		tau = -1.f;
		for (int step = 19; step >= 0; --step) { // m = 0.95 .. 0.00 in steps of 0.05
			const float m = (float)step * 0.05f;
			if (m < config.minScore)
				break;
			const float candidate = m * (1.f - degreeRatio) + degreeRatio;
			const SurvivorGraph survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, candidate);
			if (survivor.largestComponent >= minComponent && survivor.numLowDegree <= maxLowDegree) {
				tau = candidate;
				VERBOSE("Triplet filter: auto-tau chose m %.2f (tau %.3f); survivor graph keeps "
					"%u/%u images in its largest component, %u below degree 2",
					m, candidate, survivor.largestComponent, unfiltered.largestComponent,
					survivor.numLowDegree);
				break;
			}
		}
		if (tau < 0.f) {
			// A graph where no threshold is safe is a graph this filter has no business touching.
			VERBOSE("Triplet filter: auto-tau found no threshold that keeps %u/%u images connected "
				"with at most %u below degree 2; leaving the view graph alone",
				minComponent, unfiltered.largestComponent, maxLowDegree);
			return 0;
		}
	}
```

Then use `tau` in place of `tripletScores.tau` in the loop and in the log line below it. The log
line's `m %.2f` no longer describes the threshold under auto-tau — replace that field with the tau
actually applied, and let the auto-tau line above report which m produced it.

- [ ] **Step 6: Wire the CLI and the bindings**

`apps/CreateStructure/CreateStructure.cpp`, beside the two existing options:

```cpp
		("triplet-auto-tau", boost::program_options::value<bool>(&OPT::bTripletAutoTau)->default_value(TripletFilterConfig().autoTau), "camera-triplet filter: choose the threshold from the graph the filter would leave behind -- the strictest one that still keeps 99% of the largest connected component and leaves under 1% of images below degree 2 -- instead of using --triplet-min-score directly; if no threshold qualifies, the filter leaves the view graph alone")
```

Declare `bTripletAutoTau` beside `bFilterTriplets`, assign `cfg.tripletFilterCfg.autoTau` beside
the existing `minScore` assignment at line 395, and rewrite `--triplet-min-score`'s help so it
states both meanings — the sweep's floor under auto-tau, the threshold itself without it.

`libs/SFM/PythonWrapper.cpp`, in the `TripletFilterConfig` class definition:

```cpp
		.def_readwrite("auto_tau", &SFM::TripletFilterConfig::autoTau)
```

- [ ] **Step 7: Run the tests**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 2>&1 | tail -20
```

**Before running it, fix `TripletFilterTest`** — this is required, not conditional. That test sets no
`autoTau`, so it would silently start running under the new default `true`, and on its 8-node graph
the sweep stands down at every candidate: `G_LCT` has 4 nodes and max degree 3, so
`tau(m) = 0.25 m + 0.75` never drops below 0.75, `(1,3)=0.714` and `(2,3)=0.571` are removed at
every candidate, and that splits the graph into `{0,1,2}` and `{3,4,5,6,7}` — a largest component of
5 against the unfiltered 8, far below the 99 % bar. Cases (c) and (d) would then assert 0 removals
and pin nothing at all.

Set it explicitly, with the reason:

```cpp
	TripletFilterConfig filterCfg;
	filterCfg.enabled = true;
	// This test pins the paper's threshold arithmetic -- Eqn. 3 at a given m -- so it uses m
	// directly. The sweep is TripletAutoTauTest's subject, and on this small graph it stands
	// down at every candidate anyway.
	filterCfg.autoTau = false;
	filterCfg.minScore = 0.3f;
```

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 2>&1 | tail -20
```

Expected: the whole suite passes including `TripletAutoTauTest`, with `TripletFilterTest`'s counts
unchanged from Task 1.

- [ ] **Step 8: Commit**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && git add -A && git commit -m "sfm: the triplet filter picks its threshold from the graph it would leave behind

The paper derives tau from a constant m and the connectivity of the
triplet graph, and nothing in that derivation knows the view graph is
about to fragment -- which is why m = 0.6 costs registered images on
every measured capture. The filter now sweeps m downward and takes the
strictest value whose survivor graph still keeps 99% of the largest
connected component with under 1% of images below degree 2, and leaves
the graph alone when no value qualifies.

The scores do not depend on m, so the whole sweep costs one scoring pass
and a union-find per candidate. --triplet-min-score becomes the floor of
the sweep; --triplet-auto-tau turns it off."
```

---

### Task 3: A second, independent cue

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`ComputeBipartiteClusteringScores`)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (the shared edge/adjacency helper, and the cue)
- Modify: `libs/SFM/PairsMatcher.cpp:2304` and its CSV header (the new column)
- Modify: `libs/SFM/PythonWrapper.cpp` (expose the cue)
- Test: `apps/Tests/TestsSFM.cpp` (`BipartiteClusteringTest`)

**Interfaces:**
- Consumes: nothing from Tasks 1-2; the cue is deliberately independent of the triplet score.
- Produces: `std::vector<float> ComputeBipartiteClusteringScores(const Scene&)`, one entry per
  `scene.pairs` index, in `[0,1]`, `-1` where the pair is not an edge of the view graph.

- [ ] **Step 1: Write the failing test**

```cpp
bool BipartiteClusteringTest()
{
	// A doppelganger bridge against a genuine edge, in one scene:
	//   images 0,1,2 fully connected; images 3,4,5 fully connected; edge 2-3 joins them.
	// For edge (2,3): N(2)\{3} = {0,1}, N(3)\{2} = {4,5}, and NO cross edge exists, so the
	// bipartite local clustering coefficient is 0 -- the signature of a doppelganger.
	// For edge (0,1): N(0)\{1} = {2}, N(1)\{0} = {2}; the only ordered pair is (2,2), which is
	// excluded as a self-pair, leaving no admissible pair, so the score is 0 by the empty-set
	// convention below -- assert that convention explicitly rather than leaving it implicit.
	// For edge (0,2) in a scene where 0,1,2,3 are fully connected, the neighbourhoods DO
	// cross-connect and the score is 1.
	...
}
```

Build two small scenes: the barbell above, and a 4-clique. Assert the barbell bridge scores 0, a
4-clique edge scores 1, and a pair with no geometric verification scores -1. Register the test in
`apps/Tests/Tests.cpp`.

- [ ] **Step 2: Run it and watch it fail to compile**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests 2>&1 | tail -10
```

- [ ] **Step 3: Factor out the edge list and adjacency**

`ComputeTripletScores` steps 1-2 build the edge list, the `scene.pairs` → edge map and the sorted
adjacency. The cue needs exactly the same three. Move them into a private helper in the anonymous
namespace — a struct holding `edgeImages`, `edgeInliers`, `edgeOfScenePair`, `adjacency`, and a
`Neighbor` type — and have `ComputeTripletScores` call it. This is a pure extraction: the suite
must still pass unchanged after it, before any new behaviour is added. Run the tests here, not
after step 4.

- [ ] **Step 4: Implement the cue**

```cpp
std::vector<float> SFM::ComputeBipartiteClusteringScores(const Scene& scene)
{
	// Wilson & Snavely, "Network Principles for SfM: Disambiguating Repeated Structures with
	// Local Context", ICCV 2013. A true edge sits inside a well-connected neighbourhood; a
	// doppelganger edge joins two neighbourhoods that share nothing but the false edge itself.
	// So score (i,j) by how densely N(i)\{j} and N(j)\{i} are cross-connected.
	//
	// Independent of the triplet score by construction: that one reads the inlier COUNTS in the
	// triangles an edge belongs to, this one reads only the graph's connectivity around it. The
	// one fusion measurement on record says independence is what pays -- two cues at AUC 0.875
	// and 0.847, correlated at r = 0.353, whose mean scores 0.916.
	std::vector<float> scores(scene.pairs.size(), -1.f);
	// (build the shared edge list and adjacency from step 3)
	//
	// Cost bound: the honest cost is |N(i)| x |N(j)| per edge, which an exhaustively matched
	// graph makes quadratic in the image count. Cap each neighbourhood at the
	// MAX_NEIGHBORS strongest neighbours by inlier count -- deterministic, and it bounds the cue
	// at MAX_NEIGHBORS^2 membership tests per edge. A neighbourhood larger than the cap is
	// already dense enough that its clustering coefficient is decided by its strongest members.
	constexpr size_t MAX_NEIGHBORS = 128;
	...
}
```

For each edge `(i,j)`: take the adjacency of `i` minus `j` and of `j` minus `i`, truncate each to
the `MAX_NEIGHBORS` strongest by `edgeInliers` (sort a copy by inlier count descending, then
re-sort the truncated list by image index so the membership test stays a binary search), and count
the ordered pairs `(a,b)` with `a != b` where `(a,b)` is an edge. Score = matches / admissible
pairs, and `0.f` when there are no admissible pairs. Write the score to every `scene.pairs` index
mapping to that edge, so duplicates agree exactly as they do for the triplet score.

Parallelise the per-edge loop with OpenMP if the serial pass measures above a few hundred
milliseconds on the 6241-pair graph; the triplet pass is deliberately serial for the same reason
and costs a couple of milliseconds, so measure before adding a pragma.

- [ ] **Step 5: Export the column**

`libs/SFM/PairsMatcher.cpp:2304` already computes `ComputeTripletScores(scene, 0.f)` for the CSV.
Compute the cue beside it and add one column. Find the header string that names `TripletScore` and
add `BipartiteScore` after it, then write the value in the matching position of each row, empty
where the score is `-1` — the same convention `TripletScore` already uses for an unscored pair.
Read the row writer before editing: the column order in the header and in the row must match, and
nothing else in the file may shift.

- [ ] **Step 6: Expose it to Python**

Beside `compute_triplet_scores` in `libs/SFM/PythonWrapper.cpp`, following that function's own
dict-returning shape so the offline analysis can read both cues the same way.

- [ ] **Step 7: Run the tests**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 2>&1 | tail -20
```

- [ ] **Step 8: Commit**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && git add -A && git commit -m "sfm: a second, independent view-graph cue beside the triplet score

The bipartite local clustering coefficient of Wilson & Snavely reads only
the connectivity around an edge, where the triplet score reads the inlier
counts inside its triangles -- so the two disagree independently, which is
what the one fusion measurement on record says pays: two cues at AUC 0.875
and 0.847, correlated at r = 0.353, whose mean scores 0.916.

It rejects nothing. Both cues are exported per pair and the fusion is
measured offline before either gates an edge."
```

---

### Task 4: The design note says what ships

**Files:**
- Modify: `docs/design/TripletDisambiguation.md`

**Interfaces:**
- Consumes: the shipped behaviour of Tasks 1-3.
- Produces: nothing code reads.

- [ ] **Step 1: Read the whole note, then rewrite the three sections that are now wrong**

`docs/design/TripletDisambiguation.md` is 150 lines. Its "The default, and why" section describes a
threshold rule the filter no longer uses by default, and its "Limitations and follow-ups" section
lists as open two things this plan closed and two it did not.

- The method section must state the departure from Algorithm 1 step 1 (Task 1) as part of the
  method, with the labelled-reference counts as its evidence.
- The threshold section must describe the sweep (Task 2), including that it stands down when no
  threshold is safe, and that `--triplet-min-score` is now its floor.
- The follow-ups section: **close** "tau is the weak part" and "discarding the unscored pairs is
  what costs the images"; **keep** "auto-enabling is the obvious next step" (the sweep chooses a
  threshold, it does not choose whether to run — `--filter-triplets` is still an explicit flag) and
  "filtering before view-graph calibration is untested"; **add** the second cue with what it is and
  the fact that it gates nothing until its fusion is measured.

- [ ] **Step 2: Keep the measured tables**

Every number in the note came from a measurement. Do not restate, round or re-derive any of them —
the registered-image counts, the AUC figures, the Doppelgangers table and the default-rule
conjunction all stay exactly as they are. Only the claims about what the code *does* change.

- [ ] **Step 3: Check the note against the code one last time**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && /usr/bin/grep -n "tau\|unscored\|minScore\|min-score\|auto" docs/design/TripletDisambiguation.md
```

Read each hit against `libs/SFM/ViewGraphTriplets.{h,cpp}` as it now stands. A sentence that
describes the paper is fine; a sentence that describes this implementation must be true of it.

- [ ] **Step 4: Commit**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && git add -A && git commit -m "docs: the triplet note describes the filter that ships

The removal rule, the swept threshold and the second cue, with the
follow-ups this closed struck and the two it did not left standing."
```

---

## Measurement (the controller's, after the branch is green)

Not tasks and not a subagent's: they run the pipeline and read datasets, which no implementer does.
Spec §5.

1. **The offline replay** — simulate both removal rules and the whole tau ladder on every
   `pairs.csv` this branch has recorded, from the `TripletScore` column and the inlier counts.
   Costs no GPU. This is what says whether Task 1's predicted table reproduces on the current
   pipeline's own graphs, and whether Rule A survives Task 1.
2. **The outdoor check** — `Truck` and `Courthouse`, where a rule fitted on indoor captures is most
   likely to misfire.
3. **Filter placement** — the filter before `ViewGraphCalibrator` (`Scene.cpp:639`) against after it
   (`Scene.cpp:697`), reporting calibrated focals and registration.
4. **The overlap-gate re-test** — on top of the filter, stating whether the clause still adds
   anything now the one-pass dense verdict exists, and recommending its removal if it does not.
5. **No default flips.** The recorded promotion conjunction is re-run, not relaxed.

## Self-review

- **Spec coverage.** §3.1 → Task 1. §3.2 → Task 2. §3.3 Cue 1 → Task 3; Cue 2 is explicitly a later
  step and no task claims it. §3.4 → the Measurement section, controller-owned. §5 → the same.
- **Placeholders.** Task 3 step 1 and step 4 carry `...` where the test scene and the shared helper
  are built. Both are marked as "follow `TripletFilterTest`'s own scene construction" and
  "build from step 3" rather than left blank, because the exact construction depends on the helper
  extracted in step 3 — an implementer reading step 3 then step 4 has what it needs. Every value
  that must match across tasks (`MAX_NEIGHBORS`, the 0.99 and 1 % thresholds, the 0.05 ladder) is
  written out.
- **Names.** `SurvivorGraph`, `EvaluateSurvivorGraph`, `TripletFilterConfig::autoTau`,
  `ComputeBipartiteClusteringScores`, `BipartiteScore` — each defined in exactly one task and used
  under that spelling everywhere after it.
- **Ordering.** Task 2's evaluator assumes Task 1's rule (unscored pairs are kept), so it must not
  be reordered before it. Task 3's step 3 extraction touches the function Task 2 edits, so Task 3
  runs after Task 2. Task 4 describes all three.
