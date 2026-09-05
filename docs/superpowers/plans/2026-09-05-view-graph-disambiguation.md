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
- The build tree is `make/` (Ninja Multi-Config). Build and test Release. The `Tests` binary takes
  a suite number: bare `Tests` runs the generic unit tests and `Tests 1` runs the SFM suite, which
  is where every test in this plan lives. Running it bare will silently not run them.
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
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | /usr/bin/grep -i -B2 -A3 "TripletFilterTest"
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
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | tail -20
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
in `apps/Tests/Tests.cpp` next to the `TripletFilterTest` call.

**It needs two scenes, and the reason is worth understanding before you write either.** On a small
graph the sweep can only ever stand down: any removal is a large fraction of a handful of edges, and
almost any removal strands a node. A test built only on a six-node scene would assert "stands down"
in both cases and would keep passing if the sweep were broken. So the stand-down path gets a small
scene where standing down is over-determined, and the fires-and-relaxes path gets a ring big enough
to have room.

Reuse `TripletFilterTest`'s own scene construction (a pair carries an inlier count and
`F = Matrix3x3::IDENTITY` as its stand-in geometric verification, and the images need a camera);
factor that into a shared local helper rather than copying it.

**Scene 1, the barbell — the sweep must stand down.** Six images, eight pairs:

```
	(0,1)=100 (0,2)=100 (1,2)=100     triangle A
	(3,4)=100 (3,5)=100 (4,5)=100     triangle B
	(2,3)=10  (2,4)=10                with (3,4), triangle C -- the only link between A and B
```

Triangle C shares edge `(3,4)` with B, so B and C are one triplet-graph component (2 triplets) and A
is another (1 triplet). The largest is BC, so A's three edges are **unscored and always kept**,
while the scored edges are `(3,4)=(3,5)=(4,5)=1.0` and `(2,3)=(2,4)=0.1`. `G_LCT` has 4 nodes and
max degree 3, so `tau(m) = 0.25 m + 0.75` never drops below 0.75 and the two 0.1 edges are removed at
every candidate — which severs A from B. Standing down is over-determined here: the survivor's
largest component is 3 against 6, *and* 2 of 8 edges is 25 % against the 20 % bound. Assert it:

```cpp
	// The unfiltered graph, which is what the sweep measures itself against: tau = 0 keeps every
	// scored pair, and an unscored pair is kept regardless.
	const TripletScores scores = ComputeTripletScores(barbell, 0.f);
	const SurvivorGraph unfiltered = EvaluateSurvivorGraph(barbell, scores.scores, 0.f);
	if (unfiltered.numNodes != 6 || unfiltered.largestComponent != 6 || unfiltered.numKept != 8) {
		VERBOSE("TripletAutoTauTest FAILED: unfiltered barbell %u nodes, component %u, %u edges; expected 6, 6, 8",
			unfiltered.numNodes, unfiltered.largestComponent, unfiltered.numKept);
		return false;
	}
	// Cutting the two weak edges severs the two triangles: 3 images in the largest component.
	const SurvivorGraph severed = EvaluateSurvivorGraph(barbell, scores.scores, 0.75f);
	if (severed.largestComponent != 3 || severed.numKept != 6) {
		VERBOSE("TripletAutoTauTest FAILED: severed barbell component %u, %u edges; expected 3 and 6",
			severed.largestComponent, severed.numKept);
		return false;
	}
	TripletFilterConfig cfg;
	cfg.enabled = true;
	cfg.autoTau = true;
	cfg.minScore = 0.6f;
	const PairsWeightingConfig weightingCfg;
	Scene barbellAuto(barbell);
	if (FilterPairsByTriplets(barbellAuto, cfg, weightingCfg) != 0 || barbellAuto.pairs.size() != 8) {
		VERBOSE("TripletAutoTauTest FAILED: auto-tau removed %u pairs from a graph every threshold severs",
			8u - (unsigned)barbellAuto.pairs.size());
		return false;
	}
```

**Scene 2, the ring — the sweep must relax from 0.60 to 0.45 and remove exactly two edges.** 24
images, all indices modulo 24, 74 pairs:

```cpp
	// A ring of chained triangles, dense enough that removing a couple of edges costs nothing --
	// which is exactly the situation the connectivity tests cannot judge on their own.
	Scene ring;
	for (IIndex i = 0; i < 24; ++i) {
		AddPair(ring, i, (i + 1) % 24, 100);   // structural, scores 1.00
		AddPair(ring, i, (i + 2) % 24, 100);   // structural, scores 1.00
		AddPair(ring, i, (i + 3) % 24, 65);    // medium,     scores 0.65
	}
	AddPair(ring, 0, 12, 5);                   // the two edges the filter should find
	AddPair(ring, 1, 12, 5);                   //   both score 0.05
```

Every `(i,i+1)` and `(i,i+2)` edge sits only in triangles whose strongest edge is 100, so all of them
score exactly 1.0. Each `(i,i+3)` edge sits in two triangles, `{i,i+1,i+3}` and `{i,i+2,i+3}`, whose
maximum is 100 in both, so it scores `65/100 = 0.65`. The two added edges sit in the single triangle
`{0,1,12}` whose maximum is 100, so they score `0.05`. `G_LCT` has 24 nodes and max degree 8 (node
12 carries both added edges), giving `tau(m) = (2/3) m + 1/3`.

That makes the sweep's arithmetic:

| m | tau | removed | verdict |
|---|---|---|---|
| 0.60 | 0.733 | 24 medium + 2 weak = 26 of 74 (35 %) | rejected: over the 20 % bound |
| 0.50 | 0.667 | 26 of 74 | rejected |
| 0.45 | **0.633** | 2 of 74 (2.7 %) | **accepted** — 0.633 is below the medium 0.65 |
| — | — | largest component stays 24, nothing below degree 2 | — |

```cpp
	Scene ringAuto(ring);
	// Starts at 0.60, where it would strip every medium edge, and relaxes to 0.45, where only the
	// two genuinely weak edges fall. Node 12 keeps four ring edges, so nothing is stranded.
	if (FilterPairsByTriplets(ringAuto, cfg, weightingCfg) != 2 || ringAuto.pairs.size() != 72) {
		VERBOSE("TripletAutoTauTest FAILED: the ring lost %u pairs, expected exactly 2",
			74u - (unsigned)ringAuto.pairs.size());
		return false;
	}
	// and they must be the RIGHT two -- a sweep that relaxed too far would also remove 2 by
	// coincidence only if it removed these, so name them
	for (const ImagePair& pair : ringAuto.pairs) {
		if ((pair.ID1 == 0 && pair.ID2 == 12) || (pair.ID1 == 1 && pair.ID2 == 12)) {
			VERBOSE("TripletAutoTauTest FAILED: the ring kept the weak pair (%u,%u)", pair.ID1, pair.ID2);
			return false;
		}
	}
```

The two rejected candidates are 0.017 clear of the medium score on either side (0.667 and 0.633
against 0.65), so nothing here sits on a float knife-edge. If your build disagrees with the table,
**do not adjust the expectations to match** — print `tau` and the survivor counts per candidate,
find out which of the three bars actually fired, and report it: either the arithmetic above is wrong
or the sweep is, and both are worth knowing before this ships.

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
	// Relax minScore against the graph the filter would leave behind, rather than applying it as
	// given. The paper's constant is the part that fails on video keyframes: it is not calibrated
	// for them, and at m = 0.6 every measured capture loses registered images while a densely
	// connected orbit loses two thirds of its edges.
	bool autoTau = true;
	// The strictness asked for: the paper's minimum edge score m (0.6 generic/large-scale, 0.9
	// highly ambiguous, 0.3 medium/small ambiguous). With autoTau the sweep starts here and only
	// ever relaxes, so this is a ceiling on how aggressive the filter may be, never a floor.
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
		// Eqn. 3 on G_LCT: tau(m) = m (1 - d_max/|V|) + d_max/|V|, monotone in m. The sweep starts
		// at the m that was asked for and only ever relaxes: strictness is a cost, not a virtue.
		// Sweeping UP instead -- taking the strictest threshold that survives the tests below --
		// removes 92% of a well-connected outdoor orbit while passing every one of them.
		const float degreeRatio = tripletScores.numNodes > 0
			? (float)tripletScores.maxDegree / (float)tripletScores.numNodes : 0.f;
		const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
		// A candidate is accepted when it fragments nothing, strands nobody and rewrites little.
		//
		// The first two bars are RELATIVE to the unfiltered graph, and the second one has to be: a
		// real capture already has images below degree 2 before any filtering -- 5 of 209 on the
		// measured lidar graph -- so an absolute "under 1% of nodes" bar refuses every threshold,
		// including the one that removes nothing.
		//
		// The third bar is the one connectivity cannot supply. At the paper's own m = 0.6 the
		// filter removes 65-67% of a healthy, fully connected orbit and the first two bars still
		// pass, because the score is relative: on a dense graph the mean of n_ij / max(n_kl) over
		// many triangles sits well below 1 for almost every edge, doppelganger or not. A filter
		// removing two thirds of a graph is not finding outliers, and doppelgangers are a minority
		// by construction. MAX_REMOVED_FRACTION says what the filter is for; it is a policy choice
		// rather than a measured constant, and the log line below reports what was actually
		// removed so a wrong value shows up in the first run rather than being inferred.
		constexpr double MAX_REMOVED_FRACTION = 0.20;
		const unsigned minComponent = (unsigned)std::ceil(0.99 * (double)unfiltered.largestComponent);
		const unsigned maxLowDegree = unfiltered.numLowDegree +
			(unsigned)std::floor(0.01 * (double)unfiltered.numNodes);
		const unsigned minKept = unfiltered.numKept -
			(unsigned)std::floor(MAX_REMOVED_FRACTION * (double)unfiltered.numKept);
		tau = -1.f;
		for (int step = (int)std::floor(config.minScore / 0.05f); step >= 0; --step) {
			const float m = (float)step * 0.05f;
			const float candidate = m * (1.f - degreeRatio) + degreeRatio;
			const SurvivorGraph survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, candidate);
			if (survivor.largestComponent >= minComponent && survivor.numLowDegree <= maxLowDegree &&
				survivor.numKept >= minKept) {
				tau = candidate;
				VERBOSE("Triplet filter: auto-tau relaxed m %.2f to %.2f (tau %.3f); survivor graph "
					"keeps %u/%u images in its largest component, %u below degree 2 (%u before), "
					"and %u/%u edges",
					config.minScore, m, candidate, survivor.largestComponent,
					unfiltered.largestComponent, survivor.numLowDegree, unfiltered.numLowDegree,
					survivor.numKept, unfiltered.numKept);
				break;
			}
		}
		if (tau < 0.f) {
			// A graph where no threshold is safe is a graph this filter has no business touching.
			VERBOSE("Triplet filter: auto-tau found no threshold at or below m %.2f that keeps "
				"%u/%u images connected, at most %u below degree 2 (%u before) and at least %u/%u "
				"edges; leaving the view graph alone",
				config.minScore, minComponent, unfiltered.largestComponent, maxLowDegree,
				unfiltered.numLowDegree, minKept, unfiltered.numKept);
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
		("triplet-auto-tau", boost::program_options::value<bool>(&OPT::bTripletAutoTau)->default_value(TripletFilterConfig().autoTau), "camera-triplet filter: relax --triplet-min-score against the graph the filter would leave behind instead of applying it as given -- back off until the survivor graph keeps 99% of the largest connected component, adds at most 1% of the images to those below degree 2, and removes at most a fifth of the pairs; if no threshold qualifies, the filter leaves the view graph alone")
```

Declare `bTripletAutoTau` beside `bFilterTriplets`, assign `cfg.tripletFilterCfg.autoTau` beside
the existing `minScore` assignment at line 395, and rewrite `--triplet-min-score`'s help so it
states both meanings — under auto-tau the strictness the sweep starts from and only ever relaxes,
without it the threshold itself.

`libs/SFM/PythonWrapper.cpp`, in the `TripletFilterConfig` class definition:

```cpp
		.def_readwrite("auto_tau", &SFM::TripletFilterConfig::autoTau)
```

- [ ] **Step 7: Run the tests**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | tail -20
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
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | tail -20
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
requested m and relaxes it until the survivor graph keeps 99% of the
largest connected component, strands no more images than a hundredth of
the graph, and loses at most a fifth of its pairs -- leaving the graph
alone when no value qualifies.

That last bound is the one connectivity cannot supply: at m = 0.6 the
filter removes two thirds of a healthy, fully connected orbit and every
connectivity test still passes, because the score is relative and on a
dense graph almost every edge scores below 1.

The scores do not depend on m, so the whole sweep costs one scoring pass
and a union-find per candidate. --triplet-min-score is the strictness the
sweep starts from; --triplet-auto-tau turns the relaxation off."
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
  `scene.pairs` index, in `[0,1]`, and `-1` where the cue has nothing to say — either the pair is
  not an edge of the view graph, or its two neighbourhoods admit no cross pair to measure.

- [ ] **Step 1: Write the failing test**

Three expectations, and they must be three *different* values. The reason matters: an edge whose
neighbourhoods admit no cross pair is **unscored (`-1`)**, not zero. Those are different claims, and
conflating them is the same mistake Task 1 removed from the filter — a score of 0 says the two
neighbourhoods share nothing, which is damning, while `-1` says there was nothing to measure.

```cpp
bool BipartiteClusteringTest()
{
	// The barbell: images 0,1,2 fully connected, images 3,4,5 fully connected, and the single
	// edge (2,3) joining them -- the shape of a doppelganger link.
	//
	//   (2,3): N(2)\{3} = {0,1}, N(3)\{2} = {4,5}. Four ordered cross pairs, none of them an
	//          edge, so the coefficient is 0.0 -- measured, and damning.
	//   (0,1): N(0)\{1} = {2}, N(1)\{0} = {2}. The only ordered pair is (2,2), excluded as a
	//          self-pair, so there is NOTHING to measure and the score is -1. Scoring this 0.0
	//          would make a perfectly good triangle edge indistinguishable from the bridge above,
	//          which is exactly the distinction this cue exists to draw.
	//
	// Then a second scene, the 4-clique on {0,1,2,3}:
	//   (0,1): N(0)\{1} = {2,3}, N(1)\{0} = {2,3}. The admissible ordered pairs are (2,3) and
	//          (3,2), both edges, so the score is 1.0.
	//
	// And a pair without HasGeometricVerification() scores -1 in either scene: not an edge.
	...
}
```

Assert all four rows. Two of them share the `-1` sentinel for different reasons, and that is
correct — both mean the cue has nothing to say, and `pairs.csv` writes an empty cell for both.

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
pairs; where there are **no** admissible pairs the edge stays at `-1`, unscored, because nothing was
measured — not `0.f`, which would claim the neighbourhoods were checked and found to share nothing.
Write the score to every `scene.pairs` index mapping to that edge, so duplicates agree exactly as
they do for the triplet score.

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
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | tail -20
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

### Task 4: The design note describes the filter that ships

**Files:**
- Modify: `docs/design/TripletDisambiguation.md`
- Modify: `libs/SFM/README.md` (the triplet paragraph near line 198)

**Interfaces:**
- Consumes: the shipped behaviour of `libs/SFM/ViewGraphTriplets.{h,cpp}` and `libs/SFM/StarInitializer.{h,cpp}` as they now stand, and the measurements below.
- Produces: nothing code reads.

The note was written for an earlier version of the filter and is now wrong in three places and silent
about five rules. Everything a sentence claims about *this implementation* must be true of the code as
it now stands; a sentence describing the paper may stay. Every number below came from a measurement:
use it exactly as given, never rounded, restated or re-derived, and never invent a number that is not
here. The note has no readers of the plan: no task numbers, ruling numbers or process words.

- [ ] **Step 1: Read the whole note and the two headers**

Read `docs/design/TripletDisambiguation.md` end to end, then the comment blocks in
`libs/SFM/ViewGraphTriplets.h` (the file header, `TripletFilterConfig`, `SurvivorGraph`,
`EvaluateSurvivorGraph`, `ComputeTripletScores`, `FilterPairsByTriplets`) and
`libs/SFM/StarInitializer.h` (`StarInitConfig::seedViews`, `SelectReferenceView`). Those comments are
the authority on what ships; the note must agree with them.

- [ ] **Step 2: The algorithm section says what ships**

Rewrite "The algorithm" so that it states, in order:

1. The view graph and the strength `s_ij = n_ij * c_ij` (as now).
2. The triplet graph and `G_LCT`; every other edge is **unscored** (as now).
3. The score, with the yield rule (as now; `minYield` is `TripletFilterConfig::minYield`, 0.4, a
   configuration field without a command-line flag).
4. The ceiling `tau(m)` of Eqn. 3 over `G_LCT` at the paper's generic `m` 0.6 (as now, with the
   default corrected).
5. **Selection**, corrected: a scored pair is removed iff its score is below the threshold; an
   **unscored pair is kept** — the paper's step 1 discards every edge outside `G_LCT`, but on the two
   labelled references those pairs are overwhelmingly true (426 of 490 on one capture, 415 of 441 on
   the other), so absence of evidence keeps a pair. The note currently says "unscored pairs are
   removed": that sentence is wrong and goes.
6. **The threshold below the ceiling** (`--triplet-auto-tau`, the default): a *piece* is a component
   of the ceiling's survivor graph that lies inside the unfiltered graph's largest component and holds
   at least 1 % of it (so on a set under 101 images every such component is a piece); when the largest
   piece holds a strict majority of the images the pieces hold together, the ceiling is applied as
   given and the smaller pieces stay apart; otherwise the threshold is the strictest one whose survivor
   graph joins every piece in one component. Stragglers (smaller than a piece, or in a different
   component of the unfiltered graph) are neither chased nor removed. The paper's step 11 (largest
   component) is not applied. Keep the note's existing reasons for each of these.
7. **The second face** (`--triplet-second-face-score`, 0.75, part of `--triplet-auto-tau`): before
   any of item 6, the ceiling at `m` 0.75 is tried first, and it is the ceiling when the graph it
   leaves is *two-faced* — its largest piece holds a strict majority of the images in pieces and its
   second-largest piece holds at least a third of the largest; otherwise the paper's ceiling at 0.6
   stands. Why: on a two-faced building the paper's ceiling sits among the scores of the pairs
   bridging the facades (the church: 0.942 at 0.6, bridges scoring 0.89-0.96, merged in two matchings
   of four; at 0.75 every one of seven graphs splits into the facades, 130-135 and 80-83 images),
   while on a building whose graph is one face the stricter ceiling cuts the graph so thin that the
   reconstruction discards most of what it registers (Big Ben: 147 of 403 images at 0.75, 371 at
   0.6, one piece either way); a second piece of a third is the bar because the other face of a
   two-faced building holds a substantial share of the views and a night or detail cluster hanging
   off the largest piece holds a few percent (Brandenburg: 7 of 102 at 0.75).
8. **Seeding**: the filter reports the images of the largest piece the ceiling leaves (ties to the
   piece holding the lowest image index), and `StarInitializer::SelectReferenceView` chooses the
   reconstruction's reference view among them — the heaviest by weighted inliers that has at least
   `minViews - 1` valid pairs (three by default), else the next, else every image. Why: the resection
   refuses the doppelganger bridges the descent lets through but cannot choose the side it starts on,
   and the heaviest image overall sits in the densest cluster of look-alike views (Radcliffe: the
   45-image piece, so the 120-image piece never registered; Street: the largest piece's heaviest image
   had two pairs, the star needs three, and the run reconstructed nothing).

Keep the two closing paragraphs ("Why it catches what the existing cycle test cannot", "Implementation")
as they are.

- [ ] **Step 3: "Where it runs, and the flags" shows the current log lines and flags**

Replace the quoted `VERBOSE` example (the one beginning `Triplet filter: kept 599/2078 pairs`) with the
filter's current output, four lines from the Radcliffe run and, for contrast, the church's verdict on its
second ceiling:

```
Triplet filter: the ceiling at the second-face score 0.75 (0.967) leaves no second face (largest piece 110, second 52): the paper's ceiling stands
Triplet filter: tau 0.842, the strictest threshold that joins every piece (ceiling 0.948 at m 0.60, d_max/|V| 0.869); the ceiling leaves 5 pieces (components of at least 3 images) holding 254 images between them, and 28 stragglers; survivor graph keeps 268/282 images in its largest component, 18 below degree 2 (0 before), and 2843/20298 distinct image pairs
Triplet filter: kept 5567/23022 scene pairs (tau 0.842; 282 nodes, max degree 245; 796087 triplets in 1 components, 254099 doppelganger triplets gave no evidence; 17455 below tau removed, 2724 unscored kept); the reconstruction seeds in the largest piece the ceiling leaves (121 images)
Selected reference view 130 with 45757 connections over 42 pairs among 121 seed views
```

```
Triplet filter: the ceiling at the second-face score 0.75 (0.964) leaves two faces, pieces of 131 and 82 images: used
Triplet filter: tau 0.964, the ceiling applied as given, its largest piece holding a majority (ceiling 0.964 at m 0.75, the second face's, d_max/|V| 0.856); the ceiling leaves 7 pieces (components of at least 3 images) holding 237 images between them, and 40 stragglers; survivor graph keeps 131/277 images in its largest component, 62 below degree 2 (0 before), and 812/20004 distinct image pairs
```

In the flags table, the `--triplet-auto-tau` row reads: "treat `tau(m)` as a ceiling: below it, the
strictest threshold that joins every piece the ceiling leaves, unless the largest piece already holds a
majority of the images in pieces, in which case the ceiling is applied as given; off applies `tau(m)`
as given, the second face included". The `--triplet-min-score` and `--triplet-second-face-score` rows
stay as they are. Add one sentence after the table: "`TripletFilterConfig::minYield` (0.4) has no flag."
Check the Python sentence against `libs/SFM/PythonWrapper.cpp` (`/usr/bin/grep -n "triplet\|Triplet"
libs/SFM/PythonWrapper.cpp`) and make it name exactly the bindings that exist. Check that
`scripts/python/tests/triplet_disambiguation.py` still exists (it does today) and describes what the
"Harness" section says; if a subcommand named there is gone, delete the claim.

- [ ] **Step 4: Measurements — keep the old tables, add the ambiguous-scene campaign**

Keep every existing table and number in "Measurements" exactly as it is (parity, discrimination,
reconstruction effect, Doppelgangers): they are measurements of an earlier rule set on hand-held
captures and stay as the record of why the default is off. Add, at the end of the section, a
subsection **"Ambiguous-scene datasets"** with this content and these numbers:

The datasets are the ones the disambiguation literature is written about: the video sets of Yan et al.
2017 (`books` 21 images, `cereal` 25, `cup` 64, `desk` 31, `oats` 23, `street` 19, and the Temple of
Heaven `ToH` 338) and the internet collections of Heinly et al. 2014 (`indoor` 153, `brandenburg_gate`
176, `church_on_spilled_blood` 278, `radcliffe_camera` 283, `big_ben` 403, `arc_de_triomphe` 435,
`alexander_nevsky_cathedral` 449). Reference counts are registered images: the paper's own filter
`G_F` (Manam and Govindu 2024, Table 1, `m = 0.3` on these sets) and Doppelgangers++ (Xiangli et al.,
Table 2, `a+b` = two models). Matching is exhaustive, as in the paper's reference implementation; the
filter's minimum score is its default 0.6 and its second-face score 0.75. On the sets whose ceiling
shatters the graph `m` only sets a ceiling the descent replaces, and the paper's 0.6 or 0.3 give the
same thresholds; on the church the second ceiling decides: at 0.6 the ceiling (0.942) sits among the
scores of the pairs bridging the facades and two matchings of four merged them, at 0.75 (0.964) every
graph seen leaves the two facades as a majority piece and a second piece of two thirds of it, so the
stricter ceiling is used; on Big Ben (one piece at 0.75), Brandenburg (a second piece of 7 images) and
Radcliffe (no majority) the paper's ceiling stands.

Video sets (all images register with and without the filter, so the verdict is whether the camera
path folds; "fold pairs" are camera pairs within one median step of each other at least five frames
apart, "spread" the extent of the camera path relative to its step):

| set | images | without the filter | with the filter | paper's `G_F` | verdict |
|---|---|---|---|---|---|
| books | 21 | 21, folded | 21; 2 fold pairs, the hover at frames 1-6, genuine | 9 | unfolded |
| cereal | 25 | 25, folded (spread 0.237) | 25, still folded (spread 0.277; frames 8-14): the true junction (13,14) carries 320 inliers at score 0.556, the doppelganger (7,16) 862 at 0.893, weaker in every cue | 7 | folded |
| cup | 64 | 64, folded | 64; 0 fold pairs, an open ring | 40 (their one failure) | unfolded |
| desk | 31 | 31, folded | 31; 23 fold pairs, all the start hover and the genuine revisit | 12 | unfolded |
| oats | 23 | 23, folded (spread 0.293) | 23; 0 fold pairs (spread 0.707) | 9 | unfolded |
| street | 19 | 19, folded | 19; 0 fold pairs | 19 | unfolded |
| ToH | 338 | 338, folded on the temple's one-third turn (804 fold pairs at gaps of 20 frames or more) | 338; 4 fold pairs, all the genuine closure of frames 0-9 onto 330-339 | — | unfolded |

Internet collections (the "without the filter" column is the branch's default matching, a vocabulary
tree at 50 pairs per image, which folds every two-faced building; the filter column is exhaustive
matching; a model is "one-sided" when every registered camera lies on one side of the facade plane,
checked with photos of known side):

| set | images | without the filter | with the filter | paper's `G_F` | Doppelgangers++ | verdict |
|---|---|---|---|---|---|---|
| indoor | 153 | 152 | 152, one loop | 42 | 152 | the loop is real; the paper over-splits it |
| brandenburg_gate | 176 | 173, folded | 127, folded: at 0.75 the second piece (7 images) is a cluster, not a face, so the paper's ceiling (0.973) stands and leaves one piece of 127 holding both faces and a majority, applied as given; the stricter ceiling on its own leaves 94, still folded | 129 | 151 | folded |
| church_on_spilled_blood | 278 | 270, folded | 126, one-sided: the stricter ceiling (0.964) leaves the south facade with the canal views (131 images) and the north facade (82) as two faces, so it is used; its largest piece holds a majority, the reconstruction seeds in it and the north facade stays unregistered (129-135 in earlier matchings of the same set) | 136 | 157+106 | unfolded |
| radcliffe_camera | 283 | 277, folded | 181: at 0.75 the largest piece (110 images) holds no majority, so the paper's ceiling (0.948) stands; it leaves pieces of 121, 55, 44, 19 and 15 images, none a majority, the descent to 0.842 joins them, the reconstruction seeds in the 121-piece, crosses into the 55-piece through a 312-inlier pair scoring 0.932 and refuses the 95-inlier bridge at 0.842 into the other three | 177 | 186+94 | unfolded |
| big_ben | 403 | 391 | 377: at 0.75 the graph is one piece (374 images), so the paper's ceiling (0.866) stands and leaves one piece of 391, applied as given; the stricter ceiling on its own keeps so few pairs that the reconstruction discards most of what it registers (147) | 379 | 394 | one face |
| arc_de_triomphe | 435 | 405 | 363: at 0.75 the second piece (43 images) is under a third of the largest (320), so the paper's ceiling (0.816) stands and leaves one piece of 395, applied as given; the 39 stragglers and 32 images of the piece do not register (the stricter ceiling on its own: 302) | 394 | 392 | side not checked |
| alexander_nevsky_cathedral | 449 | 442 | 418: at 0.75 the graph is one piece (411 images), so the paper's ceiling (0.954) stands and leaves one piece of 420, applied as given (the stricter ceiling on its own: 413) | 429 | 445 | one face |

Then one paragraph on what these say: on the two-faced buildings the filter matches the paper (church
126 against 136, Radcliffe 181 against 177) and, like the paper and Doppelgangers++, produces one face
per model; on the one-faced buildings the second ceiling is not used and the paper's ceiling keeps the
model whole (Big Ben 377 against 379, Nevsky 418 against 429, the Arc 363 against 394); Brandenburg
and cereal are the two misses, and in both the doppelganger pairs outscore the true junction in every
cue the filter has; on the video sets every path but cereal's unfolds where the paper over-splits
(books 9 of 21, oats 9 of 23, desk 12 of 31) or fails (cup).

- [ ] **Step 5: "The default, and why"**

Keep the pre-registered rule and its numbers as they are (they are why the default is off on ordinary
captures), then add: the filter is a tool for scenes with repeated structure, matched exhaustively;
on the retrieval-matched graphs the branch builds by default (50 pairs per image) the ceiling leaves
one piece holding both faces of every two-faced building here, because retrieval prefers the look-alike
pairs and the triangles a doppelganger sits in hold few of the strong true pairs that would score it
down. `--filter-triplets` stays an explicit flag, to be set with exhaustive matching on such a scene.

- [ ] **Step 6: "Limitations and follow-ups"**

Delete "`tau` is the weak part, not the score" (the ceiling and the descent replaced Eqn. 3's fixed
threshold) and "Discarding the unscored pairs is what costs the images" (they are kept). Keep
"Auto-enabling is the obvious next step" and "Filtering before view-graph calibration is untested",
rewording the first so it is true of the current rules (the threshold chooses itself; whether to run
is still a flag). Add, one bullet each:

* Brandenburg Gate: both faces sit inside one ceiling piece (127 images at the paper's 0.6, 94 at 0.75; the second piece at 0.75 holds 7 images, so the stricter ceiling is not used)
  because the night photos of the two faces match each other as strongly as neighbours do; no
  inlier-count cue separates them, and the filter has no other.
* cereal: the true junction is weaker than the doppelganger in every cue (320 inliers at 0.556
  against 862 at 0.893); the paper over-splits it instead.
* One model per run: the pieces the ceiling leaves apart stay unregistered (the church's north facade,
  Radcliffe's three look-alike pieces), where Doppelgangers++ reports two models. Reconstructing the
  remaining pieces as further models is a pipeline question, not the filter's.
* Retrieval-matched graphs: the method needs the exhaustive graph; see the default.
* The offline replay (`triplet_replay.py` beside the datasets) approximates the run's strength from
  the match count; its pieces at the ceiling differ from the run's by tens of images on these
  near-complete graphs, and only the run's own `TripletScore` column is evidence.

- [ ] **Step 6b: The library README's triplet paragraph**

`libs/SFM/README.md`, the paragraph on the triplet filter near line 198, still says the edge weight is
"one integer per edge, its epipolar inlier count" and the score `n_ij / max n_kl`, presents `tau` as the
threshold, and omits `--triplet-auto-tau`. Rewrite that paragraph in the README's own register to say:
the strength is the inlier count discounted by the inliers' grid coverage; a triangle whose three pairs
all yield below 0.4 of the graph's own envelope gives no evidence; unscored pairs are kept; `tau(m)` is a
ceiling (default `m` 0.6, or 0.75 when the graph that stricter ceiling leaves is two-faced, a majority
piece with a second piece of at least a third of it) below which the threshold is the strictest one
joining every piece unless the largest piece already holds a majority; the reconstruction seeds in the
largest ceiling piece. Keep it one paragraph and point at `docs/design/TripletDisambiguation.md` for
the rest.

Also, in the design note: the "Harness" section must name only subcommands and column names the
script `scripts/python/tests/triplet_disambiguation.py` actually has after its own rewrite (it scores the
shipped rule from the export's `NumMatches`, `Coverage` and `MeanRayAngle` columns, and `parity` checks
the export's `TripletScore` column against that); the parity figure quoted in "Measurements" ("maximum
absolute C++/Python difference 5.3e-7", "7 graphs") was measured on an older rule and is replaced by the
parity the rewritten script reports on the two exhaustive exports it is checked against: a maximum
absolute difference of 5.140e-07 on `street` (171 pairs) and 5.287e-07 on `radcliffe_camera` (22197
pairs, 19383 of them scored); and the CSV column list in "Where it runs, and the flags" must be the
export's actual header:
`ImageA,ImageB,NumMatches,Coverage,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle,TripletScore`.

- [ ] **Step 7: Check every claim against the code**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && /usr/bin/grep -n "tau\|unscored\|minScore\|min-score\|auto\|seed\|piece\|majority\|straggler" docs/design/TripletDisambiguation.md
```

Read each hit against `libs/SFM/ViewGraphTriplets.{h,cpp}` and `libs/SFM/StarInitializer.{h,cpp}` as
they now stand. Delete any sentence that describes behaviour the code no longer has.

- [ ] **Step 8: Commit**

```bash
git add docs/design/TripletDisambiguation.md libs/SFM/README.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "docs: the triplet note describes the filter that ships

Unscored pairs are kept, the ceiling looks for the second face before the threshold descends
from it to the strictest one joining the pieces unless a majority piece stands, the
reconstruction seeds in the largest ceiling piece, and the ambiguous-scene results sit beside
the earlier hand-held measurements: the church and Radcliffe at the paper's counts and
one-sided, Brandenburg and cereal the misses."
```

---

### Task 5: The threshold is the strictest one that keeps the graph together

Supersedes the threshold rule Task 2 shipped. Spec §3.2 as rewritten on 2026-09-05: the paper's
`tau(m)` is a ceiling, and below it the threshold is the **strictest** value whose survivor graph
keeps 99 % of the unfiltered largest component in one component. The three-bar sweep (largest
component, low-degree images, at most 20 % removed) is deleted outright — it stands down on every
one of the ambiguous-scene datasets, because the correct answer there removes 66-96 % of the pairs
and leaves a chain's two endpoints at degree 1.

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`TripletFilterConfig` comments, `SurvivorGraph` comment)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`FilterPairsByTriplets`, and the small items below)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (`--triplet-auto-tau`, `--triplet-min-score` help)
- Modify: `libs/SFM/PythonWrapper.cpp` (the `auto_tau` docstring, if it describes the old bars)
- Modify: `docs/design/TripletDisambiguation.md` (the threshold paragraphs and the flags table only)
- Modify: `scripts/python/tests/triplet_disambiguation.py` (only if it describes the old bars)
- Test: `apps/Tests/TestsSFM.cpp` (`TripletAutoTauTest`, rewritten scene by scene)

**Interfaces:**
- Consumes: `ComputeTripletScores`, `EvaluateSurvivorGraph`, `SurvivorGraph` exactly as Task 2 left
  them. `TripletFilterConfig` keeps its three fields; `autoTau` keeps its default `true`.
- Produces: nothing new. `FilterPairsByTriplets` keeps its signature and its return value.

- [ ] **Step 1: Rewrite `TripletAutoTauTest` so every scene asserts the new rule's answer**

Seven scenes exist. Under the new rule their expected outcomes are, with the arithmetic:

| scene | ceiling tau(0.6) | outcome under the new rule | assertion |
|---|---|---|---|
| barbell | `G_LCT` = {2,3,4,5}, d_max 3, r 0.75 → 0.9 | at 0.9 the two 0.1 edges go and the largest component is 3 of 6; the only score below the ceiling is 0.1, so tau = 0.1 and nothing is removed | 0 removed, 8 pairs left (unchanged) |
| ring | d_max 8 (node 12), r 1/3 → 0.7333 | at the ceiling the 48 structural edges (1.0) keep all 24 images together, so the ceiling is applied as given: 24 medium (0.65) + 2 weak (0.05) removed | **26 removed, 48 left**; neither `(0,12)` nor `(1,12)` nor any `(i,i+3)` pair survives |
| bridge | `G_LCT` = right K10 + the two bridge edges, 11 nodes, d_max 10 (node 10) → 0.9636 | the ceiling severs the cliques (component 10 of 20); the only score below it is 0.01 → nothing removed | 0 removed, 92 pairs left (unchanged) |
| pendant | 11 nodes, d_max 8, r 8/11 → 0.8909 | dropping the three weak spokes (0.01) keeps all 11 together → ceiling applied as given | **3 removed, 31 left**; the three `EvaluateSurvivorGraph` assertions on the candidate (3 below degree 2, component 11, 31 edges) stay exactly as they are |
| baseline | 10 scored nodes, d_max 9 → 0.96 | `(8,9)` at 0.01 goes; the pendant's unscored edge is kept regardless → 1 removed | 1 removed, 45 left (unchanged) |
| boundary | 10 nodes, d_max 9 → 0.96 | `(8,9)` scores exactly 0.96 = the ceiling and a score equal to tau is kept | 0 removed, 33 left (unchanged; the comment now says this pins `>=` at the ceiling, not a ladder start) |
| ringGap | — | **delete the scene.** It pinned the old sweep's stopping rung; under the new rule it duplicates the ring |

And one new scene, **the pan**, which is the one that pins the new rule's actual work — a
complete graph over a short sequence, one doppelganger pair, one weak chain edge that the ceiling
severs, and an isolated verified pair that makes the component bar relative:

```cpp
	// Scene 7, the pan: six frames of one camera sweep, exhaustively matched so every pair
	// verifies (identical facades give every pair some inliers), plus one doppelganger pair and a
	// separate verified pair that is in no triangle. The strong chain is (i,i+1) = 100 except
	// (2,3) = 83; (i,i+2) = 50; every longer gap 10; the doppelganger (0,5) = 30 -- STRONGER than
	// the true low-overlap pairs, as measured on the real street set. Scores, each edge in four
	// triangles: chain edges 1.0 except (2,3) = (1 + 0.83 + 0.83 + 1)/4 = 0.915; (0,2) and (3,5)
	// 0.7756, (1,3) and (2,4) 0.625; (0,5) = (0.3 + 0.6 + 0.6 + 0.3)/4 = 0.45; the gap-3/4/5
	// pairs 0.1 to 0.13. G_LCT is the K6, so r = 5/6 and the ceiling at m = 0.6 is 0.9333.
	//
	// At the ceiling only the four 1.0 chain edges survive: components {0,1,2}, {3,4,5}, {6,7},
	// largest 3 against the unfiltered 6 (the K6; (6,7) is its own component and the bar is 99% of
	// the LARGEST component, not of all 8 images). The distinct scores below the ceiling are
	// 0.915, 0.7756, 0.625, 0.45, 0.13, 0.125, 0.1; the strictest that reconnects the chain is
	// 0.915, (2,3)'s own score. Kept: the five chain edges and the unscored (6,7); removed: the ten
	// others, the doppelganger among them. A search that stopped one candidate looser would keep
	// (0,2) and (3,5); one measuring the bar against all images would never pass and remove nothing.
	Scene pan;
	AddTripletImages(pan, 8);
	static const unsigned panInliers[6][6] = {
		//   0    1    2    3    4    5
		{    0, 100,  50,  10,  10,  30 }, // 0: (0,5) = 30 is the doppelganger
		{    0,   0, 100,  50,  10,  10 },
		{    0,   0,   0,  83,  50,  10 }, // (2,3) = 83: the weak chain edge the ceiling severs
		{    0,   0,   0,   0, 100,  50 },
		{    0,   0,   0,   0,   0, 100 },
		{    0,   0,   0,   0,   0,   0 },
	};
	for (IIndex i = 0; i < 6; ++i)
		for (IIndex j = i + 1; j < 6; ++j)
			AddTripletPair(pan, i, j, panInliers[i][j]);
	AddTripletPair(pan, 6, 7, 100); // verified, in no triangle: unscored, kept, and not the largest component

	const TripletScores panScores = ComputeTripletScores(pan, 0.6f);
	if (!ISEQUAL(panScores.tau, 0.93333f, 1e-4f) || panScores.numScoredPairs != 15) {
		VERBOSE("TripletAutoTauTest FAILED: pan ceiling %g with %u scored pairs, expected 0.9333 and 15",
			panScores.tau, panScores.numScoredPairs);
		return false;
	}
	FOREACH(idxPair, pan.pairs) {
		const ImagePair& pair = pan.pairs[idxPair];
		const float score = panScores.scores[idxPair];
		if ((pair.ID1 == 2 && pair.ID2 == 3 && !ISEQUAL(score, 0.915f, 1e-4f)) ||
			(pair.ID1 == 0 && pair.ID2 == 5 && !ISEQUAL(score, 0.45f, 1e-4f)) ||
			(pair.ID1 == 6 && pair.ID2 == 7 && score >= 0.f)) {
			VERBOSE("TripletAutoTauTest FAILED: pan pair (%u,%u) scored %g", pair.ID1, pair.ID2, score);
			return false;
		}
	}
	const SurvivorGraph panUnfiltered = EvaluateSurvivorGraph(pan, panScores.scores, 0.f);
	const SurvivorGraph panAtCeiling = EvaluateSurvivorGraph(pan, panScores.scores, panScores.tau);
	if (panUnfiltered.largestComponent != 6 || panUnfiltered.numNodes != 8 || panAtCeiling.largestComponent != 3) {
		VERBOSE("TripletAutoTauTest FAILED: pan components %u of %u unfiltered, %u at the ceiling; expected 6 of 8 and 3",
			panUnfiltered.largestComponent, panUnfiltered.numNodes, panAtCeiling.largestComponent);
		return false;
	}
	Scene panAuto(pan);
	const unsigned numPanRemoved = FilterPairsByTriplets(panAuto, cfg, weightingCfg);
	const std::set<std::pair<IIndex,IIndex>> expectedPanKept{{0,1}, {1,2}, {2,3}, {3,4}, {4,5}, {6,7}};
	if (numPanRemoved != 10 || panAuto.pairs.size() != 6 || TripletKeptPairs(panAuto) != expectedPanKept) {
		VERBOSE("TripletAutoTauTest FAILED: the pan lost %u pairs leaving %u; expected exactly the chain and the isolated pair",
			numPanRemoved, panAuto.pairs.size());
		return false;
	}
```

`ISEQUAL` with a tolerance: use whatever the file already uses for approximate float comparison
(`TripletFilterTest` compares scores with a tolerance — copy its idiom).

Also in this step, the duplicate-pair coverage the review found missing: add a second
`AddTripletPair(bridge, 0, 1, 100)` to the bridge scene beside the existing one. The existing
assertion `bridgeUnfiltered.numKept != 92` then fires the moment `EvaluateSurvivorGraph`'s
`seenEdges` skip is removed (it would read 93). Verify both directions before committing: with the
skip present the suite passes; with it temporarily deleted, `TripletAutoTauTest` fails. Restore it.

And the bridge scene's comment gets one sentence on its invariant: `G_LCT` is the *right* clique
only because its component carries 121 triplets against the left's 120 (the bridge triangle's third
edge `(10,11)` belongs to the right clique); if that flipped, the right clique's edges would be
unscored and the scene would stop testing the component bar while still passing.

- [ ] **Step 2: Build and run, and watch the ring, pendant and pan assertions fail against the old rule**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests 2>&1 | tail -3 && ./bin/Release/Tests 1 2>&1 | /usr/bin/grep -E "TripletAutoTauTest|passed|failed"
```

Expected: `TripletAutoTauTest FAILED: the ring lost 2 pairs` (the old sweep relaxes to 0.45 and removes only the two weak pairs).

- [ ] **Step 3: Replace the sweep**

In `FilterPairsByTriplets`, delete everything from the `if (config.autoTau) {` line through its
closing brace — the comment block about the three bars, `MAX_REMOVED_FRACTION`, the `minComponent`
/ `maxLowDegree` / `minKept` bars, the `for (int step ...)` ladder, and the stand-down branch — and
put this in its place:

```cpp
	const float degreeRatio = tripletScores.numNodes > 0
		? (float)tripletScores.maxDegree / (float)tripletScores.numNodes : 0.f;
	// Eqn. 3 on G_LCT, tau(m) = m (1 - d_max/|V|) + d_max/|V|, is the CEILING: the filter is never
	// stricter than the m it was given. On a complete view graph -- every pair verified, which is
	// what identical facades produce under exhaustive matching -- d_max/|V| is (|V|-1)/|V| and the
	// ceiling sits at 0.95-0.99 whatever m is; on the sparse graphs of video captures it is 0.7 or
	// below.
	const float ceiling = tripletScores.tau;
	float tau = ceiling;
	SurvivorGraph unfiltered{0, 0, 0, 0}, survivor{0, 0, 0, 0};
	if (config.autoTau) {
		// Below the ceiling, the threshold is the STRICTEST one that keeps the graph together: the
		// largest value whose survivor graph keeps 99% of the unfiltered largest component in one
		// piece. Everything scored below that is either a weak true pair the backbone does not
		// need or a doppelganger, and nothing in the inlier counts tells the two apart -- on the
		// ambiguous-scene datasets the doppelganger pairs OUTSCORE the true low-overlap pairs, and
		// a doppelganger's triangles are mutually consistent -- so the only defensible cut keeps
		// the strong edges and exactly enough of them. Measured on those datasets that answer
		// removes 66-96% of the pairs and leaves a chain's two endpoints at degree 1, which is why
		// there is no bar on how much is removed and none on low-degree images: an image with one
		// strong edge is in the component and can be resected from it.
		unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
		const unsigned minComponent = (unsigned)std::ceil(0.99 * (double)unfiltered.largestComponent);
		survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, ceiling);
		if (survivor.largestComponent < minComponent) {
			// The largest component only grows as tau falls, so among the distinct scores below
			// the ceiling, strictest first, the first that passes is a binary search away. The
			// loosest candidate keeps every scored pair -- the unfiltered graph itself -- so it
			// always passes, and the ceiling fragmenting the graph means at least one scored pair
			// sits below it.
			std::vector<float> candidates;
			candidates.reserve(tripletScores.numScoredPairs);
			for (float score : tripletScores.scores)
				if (score >= 0.f && score < ceiling)
					candidates.push_back(score);
			std::sort(candidates.begin(), candidates.end(), std::greater<float>());
			candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
			ASSERT(!candidates.empty(), "FilterPairsByTriplets: the ceiling fragments a graph with no score below it");
			size_t lo = 0, hi = candidates.size() - 1;
			while (lo < hi) {
				const size_t mid = (lo + hi) / 2;
				if (EvaluateSurvivorGraph(scene, tripletScores.scores, candidates[mid]).largestComponent >= minComponent)
					hi = mid;
				else
					lo = mid + 1;
			}
			tau = candidates[lo];
			survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, tau);
		}
		VERBOSE("Triplet filter: tau %.3f, %s (ceiling %.3f at m %.2f, d_max/|V| %.3f); survivor graph "
			"keeps %u/%u images in its largest component, %u below degree 2 (%u before), "
			"and %u/%u distinct image pairs",
			tau, tau < ceiling ? "the strictest threshold that keeps the graph together" : "the ceiling applied as given",
			ceiling, minScore, degreeRatio, survivor.largestComponent, unfiltered.largestComponent,
			survivor.numLowDegree, unfiltered.numLowDegree, survivor.numKept, unfiltered.numKept);
	}
```

`std::greater` needs `<functional>`; `std::sort`/`std::unique` need `<algorithm>`. The
`minScore` clamp above stays; add, right after it, a warning when the clamp changed the value —
`CreateStructure` already refuses an out-of-range `--triplet-min-score`, so this only ever fires
through the library or Python API, but a mistyped value that silently became a different filter is
worse than a refused one:

```cpp
	if (minScore != config.minScore)
		VERBOSE("warning: triplet filter: minimum edge score %g is outside [0,1], using %g", config.minScore, minScore);
```

(A NaN compares unequal to everything, so this line fires for NaN too.)

The removal loop, the summary `VERBOSE` and the `ComputePairsWeights` re-run below stay as they are.

- [ ] **Step 4: Comments and help that describe the old rule**

- `libs/SFM/ViewGraphTriplets.h`: `autoTau`'s comment says the paper's constant is uncalibrated and
  describes a sweep that "only ever relaxes" — replace with: the paper's `tau(m)` is a ceiling, and
  below it the threshold is the strictest one whose survivor graph keeps 99 % of the unfiltered
  largest component together; off, `tau(m)` is applied as given. `minScore`'s comment: the paper's
  `m` in `[0,1]` (the domain the implementation enforces), the ceiling when `autoTau` is on.
  `SurvivorGraph`'s comment: "what the search judges" rather than "what the sweep judges".
- `apps/CreateStructure/CreateStructure.cpp`: `--triplet-auto-tau` help → "camera-triplet filter:
  treat the paper's threshold tau(m) as a ceiling and relax below it to the strictest threshold that
  still keeps 99% of the largest connected component of the view graph together; off applies tau(m)
  as given". `--triplet-min-score` help → "camera-triplet filter: the paper's minimum edge score m
  in (0,1), from which the threshold tau = m(1-r)+r is derived with r the maximum-degree ratio of
  the scored graph; with --triplet-auto-tau that threshold is the ceiling the filter starts from
  (0.6 generic scenes, 0.9 highly ambiguous, 0.3 medium/small ambiguous)".
- `libs/SFM/PythonWrapper.cpp` and `scripts/python/tests/triplet_disambiguation.py`: grep for
  `20%`, `fifth`, `strand`, `sweep`, `relax`; any sentence describing the three bars is rewritten
  to describe the ceiling-and-connectivity rule, nothing else changes.
- `docs/design/TripletDisambiguation.md`: the `Selection` bullet under "The algorithm", the flags
  table, and the "Two caveats" paragraph describe what ships: the ceiling, the strictest connecting
  threshold, the 99 % bar. Do not touch any measured table or number; the full rewrite is Task 4.
- `libs/SFM/ViewGraphTriplets.cpp` header order: it is the only file in `libs/SFM` that puts the
  standard headers before `"Common.h"`. Reorder to match the directory. And `seenEdges` in
  `EvaluateSurvivorGraph` is rebuilt unreserved on every call: `reserve(scene.pairs.size())`,
  matching what `ComputeTripletScores` already does.

- [ ] **Step 5: Run the suite**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx/make && ninja -f build-Release.ninja Tests SFM 2>&1 | tail -3 && ./bin/Release/Tests 1 2>&1 | tail -5
```

Expected: 61 passed, 0 failed (the same count: one scene deleted, one added inside the same test).

Then the mutation kills, each applied alone, the suite run, the mutant reverted — report the table:

1. binary search returns the next looser candidate (`hi = mid + 1` / `lo = mid`): the pan keeps `(0,2)` and `(3,5)`.
2. the bar measured against `unfiltered.numNodes` instead of `unfiltered.largestComponent`: the pan can never pass and removes nothing.
3. search from the loosest candidate upward, stopping at the first that passes: the pan and the ring remove nothing.
4. skip the check at the ceiling and always search below it: the ring relaxes to 0.65 and removes only 2.
5. `score > tau` instead of `>=` in `EvaluateSurvivorGraph`: the boundary scene removes `(8,9)`.
6. the `seenEdges` skip deleted: the bridge scene's unfiltered edge count reads 93.

- [ ] **Step 6: Commit**

```bash
cd /home/ubuntu/.claude/worktrees/roma2-onnx && git add -A && git commit -m "sfm: the triplet filter's threshold is the strictest one that keeps the graph together

The paper's tau(m) is a ceiling; below it the filter relaxes only to the
strictest threshold whose survivor graph keeps 99% of the largest component
in one piece. The three-bar sweep it replaces stood down on every one of
the ambiguous-scene datasets: there every pair verifies, the true pairs are
a minority of the complete graph, and the right answer removes most of the
edges and leaves the chain's endpoints at degree 1 -- all three things the
bars forbade. Measured on the six yan2017 sets the new rule keeps every
chain whole where the paper's fixed threshold fragments five of them."
```

---

### Task 6: The strength of an edge is its inlier count discounted by the inliers' coverage

Spec §3.5. The paper weighs an edge by its inlier count `n_ij`; on the ambiguous small sets that
ranks the doppelganger above the true junction (oats: `(6,21)` 880 inliers against `(6,7)` 627;
cup: `(1,38)` 443 against `(11,12)` 358), and Task 5's descent then reconnects the graph through
the doppelganger and the model folds. What separates the two is where the inliers sit: a
doppelganger's matches lie on the duplicated object alone, a true adjacent pair's spread over the
whole overlap. From now on the strength of an edge is `s_ij = n_ij * c_ij`, with `c_ij` the
fraction of a `gridSize x gridSize` grid the pair's track-forming matches occupy (the smaller of the
two images' fractions), and the triplet score is the paper's ratio over `s` instead of `n`.
Replayed offline on the exported graphs, this keeps no doppelganger on oats or cup and leaves
street, books and desk as they were; cereal stays a known failure. No parameter, no power, no angle
term: `weightSpatial`'s ray-angle factor measures triangulation conditioning and would drop the
adjacent pairs of a walk (measured on ToH: pairs with 6000-9000 inliers below 1.5 degrees of
baseline).

**Files:**
- Modify: `libs/SFM/PairsWeighting.h` (declare `ComputePairCoverage`)
- Modify: `libs/SFM/PairsWeighting.cpp` (`ComputePairCoverage` extracted out of `ComputeIntrinsicWeight`, which calls it)
- Modify: `libs/SFM/ViewGraphTriplets.h` (header comment, `ComputeTripletScores` signature and comment)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`ComputeTripletScores`: edge strength; `FilterPairsByTriplets`: passes the grid size)
- Modify: `libs/SFM/PairsMatcher.h`, `libs/SFM/PairsMatcher.cpp` (`ExportPairsCSV` takes the grid size, writes a `Coverage` column)
- Modify: `libs/SFM/Scene.cpp` (`ExportMatchingCSVs` passes `config.matchCfg.weightingCfg.gridSize`)
- Modify: `libs/SFM/PythonWrapper.cpp` (`ComputeTripletScoresDict` gains a `grid_size` argument)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (`--filter-triplets` help text)
- Modify: `apps/Tests/TestsSFM.cpp` (`AddTripletPair` builds real matches; every `ComputeTripletScores` call passes a grid size; new `TripletCoverageTest`), `apps/Tests/TestsSFM.h`, `apps/Tests/Tests.cpp` (register it after `TripletAutoTauTest`)
- Modify: `docs/design/TripletDisambiguation.md` (the "one integer per edge" and `n_ij` sentences; the full rewrite stays Task 4)

**Interfaces:**
- Consumes: `ImagePair::GetTrackFormingPoints(img1, img2)`, `ImagePair::GetNumWeightedInliers()`, `PairsWeightingConfig::gridSize`, the pinhole/spherical binning already inside `ComputeIntrinsicWeight`.
- Produces: `float SFM_API ComputePairCoverage(const ImagePair& pair, const Image& img1, const Image& img2, int gridSize)` in `PairsWeighting.h`; `TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore, int gridSize)`; `static bool PairsMatcher::ExportPairsCSV(const Scene& scene, const String& fileName, float minWeight, int gridSize)`; `pairs.csv` header `ImageA,ImageB,NumMatches,Coverage,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle,TripletScore`.

- [ ] **Step 1: Make the test scenes carry real matches, and write the failing coverage test**

`AddTripletPair` currently sets `numFilteredInliers` on a pair with no matches and no keypoints,
which is enough for a count but not for a coverage. Replace it (keep `AddTripletImages`,
`BuildTripletScene`, `TripletPairSpec` and `TripletKeptPairs` as they are):

```cpp
// A pair of numInliers track-forming matches whose keypoints occupy the first cellsA (cellsB)
// cells of a 10x10 grid over each 640x480 image, walked row by row from the top left; 100 cells
// is full coverage, which leaves the pair's triplet strength equal to its inlier count -- what
// every scene below relies on unless it says otherwise.
void AddTripletPair(Scene& scene, IIndex idA, IIndex idB, unsigned numInliers, bool verified = true,
	unsigned cellsA = 100, unsigned cellsB = 100)
{
	ASSERT(cellsA >= 1 && cellsA <= 100 && cellsB >= 1 && cellsB <= 100);
	ImagePair pair(idA, idB);
	Image& imageA = scene.images[idA];
	Image& imageB = scene.images[idB];
	const auto cellCentre = [](unsigned cell) {
		return cv::Point2f((float)(cell % 10) * 64.f + 32.f, (float)(cell / 10) * 48.f + 24.f);
	};
	for (unsigned m = 0; m < numInliers; ++m) {
		imageA.keypoints.emplace_back(cellCentre(m % cellsA), 1.f);
		imageB.keypoints.emplace_back(cellCentre(m % cellsB), 1.f);
		pair.matches.emplace_back((int)imageA.keypoints.size() - 1, (int)imageB.keypoints.size() - 1, 0.f);
	}
	pair.numFilteredInliers = numInliers;
	if (verified)
		pair.F = Matrix3x3::IDENTITY; // stands in for the geometric verification
	scene.pairs.emplace_back(std::move(pair));
}
```

Update the comment above the helpers (line ~7714, "a pair carries an inlier count and ...") to say
a pair carries an inlier count, its matches' coverage, and a verification flag.

Then add, after `TripletAutoTauTest`:

```cpp
// The strength of an edge is its inlier count discounted by the fraction of the frame its inliers
// cover (ViewGraphTriplets.h): a doppelganger's matches sit on the duplicated object alone, a true
// adjacent pair's spread over the whole overlap. Triangle {0,1,2}: (0,1) and (1,2) carry 600
// inliers over all 100 cells, (0,2) carries 900 inliers over 10 cells of image 0 and all 100 of
// image 2. Strengths 600, 600 and 900 * min(0.1, 1.0) = 90, so the scores are (0,1) = (1,2) = 1.0
// and (0,2) = 0.15. By count alone (0,2) would score 1.0 and the other two 600/900 = 0.667.
bool TripletCoverageTest()
{
	TD_TIMER_START();
	const PairsWeightingConfig weightingCfg; // defaults; the coverage grid is its gridSize
	Scene scene;
	AddTripletImages(scene, 3);
	AddTripletPair(scene, 0, 1, 600);
	AddTripletPair(scene, 1, 2, 600);
	AddTripletPair(scene, 0, 2, 900, true, 10, 100);
	const TripletScores scores = ComputeTripletScores(scene, 0.f, weightingCfg.gridSize);
	if (scores.numTriplets != 1 || scores.numScoredPairs != 3 ||
		!ISEQUAL(scores.scores[0], 1.f) || !ISEQUAL(scores.scores[1], 1.f) || !ISEQUAL(scores.scores[2], 0.15f)) {
		VERBOSE("TripletCoverageTest FAILED: %u triplets, %u scored, scores %g %g %g; expected 1, 3, 1 1 0.15",
			scores.numTriplets, scores.numScoredPairs, scores.scores[0], scores.scores[1], scores.scores[2]);
		return false;
	}
	// The coverage is measured on the grid the caller names, the same one the pair weighting
	// uses: on a 2x2 grid the ten cells of image 0 are the top row of the 10x10 grid, i.e. the
	// two upper cells of the coarse one, coverage 0.5, strength 450, score 450/600 = 0.75.
	const TripletScores coarse = ComputeTripletScores(scene, 0.f, 2);
	if (!ISEQUAL(coarse.scores[0], 1.f) || !ISEQUAL(coarse.scores[1], 1.f) || !ISEQUAL(coarse.scores[2], 0.75f)) {
		VERBOSE("TripletCoverageTest FAILED: 2x2 grid scores %g %g %g; expected 1 1 0.75",
			coarse.scores[0], coarse.scores[1], coarse.scores[2]);
		return false;
	}
	// And the filter acts on it: G_LCT has 3 nodes and max degree 2, so at m = 0.5 the threshold
	// is 0.5 * (1 - 2/3) + 2/3 = 0.833, applied as given. The doppelganger goes and the chain
	// stays; by count alone it would be the chain that goes.
	TripletFilterConfig filterCfg;
	filterCfg.enabled = true;
	filterCfg.autoTau = false;
	filterCfg.minScore = 0.5f;
	const unsigned numRemoved = FilterPairsByTriplets(scene, filterCfg, weightingCfg);
	const std::set<std::pair<IIndex,IIndex>> expected{{0,1},{1,2}};
	if (numRemoved != 1 || TripletKeptPairs(scene) != expected) {
		VERBOSE("TripletCoverageTest FAILED: %u pairs removed, %u kept; expected 1 removed, (0,1) and (1,2) kept",
			numRemoved, (unsigned)scene.pairs.size());
		return false;
	}
	VERBOSE("TripletCoverageTest PASSED: doppelganger (0,2) scores 0.15 against the chain's 1.0 and is the one removed (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
```

Declare it in `apps/Tests/TestsSFM.h` next to `TripletAutoTauTest` and run it from
`apps/Tests/Tests.cpp` right after it, in the same style as the surrounding calls.

Every existing `ComputeTripletScores(x, m)` call in `TripletFilterTest` and `TripletAutoTauTest`
becomes `ComputeTripletScores(x, m, weightingCfg.gridSize)` where a `weightingCfg` is in scope and
`ComputeTripletScores(x, m, PairsWeightingConfig().gridSize)` otherwise. Their expected numbers do
not change: full coverage is the default of the helper.

- [ ] **Step 2: Build and run, expect the new test to fail to compile**

Run: `cd make && ninja -f build-Release.ninja Tests`
Expected: compile errors on the three-argument `ComputeTripletScores` (it does not exist yet).

- [ ] **Step 3: Extract the coverage out of the intrinsic weight**

In `libs/SFM/PairsWeighting.h`, after `PairsWeightingConfig`:

```cpp
// The fraction of a gridSize x gridSize grid over the image that the pair's track-forming matches
// occupy, in [0,1], taken as the smaller of the two images' fractions: how much of the frame the
// pair's evidence covers, whatever its count. Pinhole images bin on a uniform pixel grid,
// spherical ones on equal-solid-angle cells. 0 for a pair with no stored matches.
float SFM_API ComputePairCoverage(const ImagePair& pair, const Image& img1, const Image& img2, int gridSize);
```

In `libs/SFM/PairsWeighting.cpp`, before `ComputeIntrinsicWeight`, move the grid part of it into:

```cpp
float SFM::ComputePairCoverage(const ImagePair& pair, const Image& img1, const Image& img2, int gridSize)
{
	ASSERT(gridSize > 0);
	if (!pair.HasMatches())
		return 0.f;
	// The coverage runs over the TRACK-FORMING matches, dense supplement included: it measures
	// where this pair has correspondences, and a dense draw covers the frame it was drawn over
	// whether or not that counts as descriptor evidence.
	const auto [points1, points2] = pair.GetTrackFormingPoints(img1, img2);
	// Divide each view into gridSize x gridSize cells:
	//  - pinhole  : uniform pixel grid (each cell = equal pixel area)
	//  - spherical: equal-solid-angle bins on the unit sphere via (azimuth, sin(latitude));
	//               each cell covers 4*pi/gridSize^2 sr, and azimuth binning wraps
	//               across the equirectangular seam (u=0 ~ u=W)
	const auto binFeature = [gridSize](const Point2f& p, const Image& img) {
		int gx, gy;
		if (img.pCamera->GetType() == CameraType::SPHERICAL) {
			const Point3 b = img.pCamera->UnprojectNormalized(Cast<REAL>(p));
			const REAL azimuth = ATAN2(b.x, b.z); // [-pi, pi]
			gx = MINF((int)((azimuth + REAL(M_PI)) / (REAL(2) * REAL(M_PI)) * REAL(gridSize)), gridSize - 1);
			gy = MINF((int)((b.y + REAL(1)) * REAL(0.5) * REAL(gridSize)), gridSize - 1);
		} else {
			gx = (int)(p.x / (float)img.GetWidth() * gridSize);
			gy = (int)(p.y / (float)img.GetHeight() * gridSize);
		}
		return std::make_pair(gx, gy);
	};
	const auto occupied = [&](const std::vector<Point2f>& points, const Image& img) {
		std::vector<bool> grid(gridSize * gridSize, false);
		for (const Point2f& p : points) {
			const auto [gx, gy] = binFeature(p, img);
			if (gx >= 0 && gx < gridSize && gy >= 0 && gy < gridSize)
				grid[gy * gridSize + gx] = true;
		}
		return (int)std::count(grid.begin(), grid.end(), true);
	};
	return (float)MINF(occupied(points1, img1), occupied(points2, img2)) / (float)(gridSize * gridSize);
}
```

`ComputeIntrinsicWeight` keeps its validity floor and its angle term and replaces everything from
`const auto [points1, points2] = ...` through `const float areaScore = ...` with
`const float areaScore = ComputePairCoverage(pair, img1, img2, gridSize);`, keeping the
`pair.overlapArea` proxy line and the `angleScore` product after it. Fold the "AREA SCORE runs over
the track-forming matches" paragraph of its comment into the new function's (it is there above);
what stays on `ComputeIntrinsicWeight` is the floor and the angle term. Include `<algorithm>` if
`std::count` needs it.

- [ ] **Step 4: Score the triplets on the discounted strength**

`libs/SFM/ViewGraphTriplets.h`: the header comment's second paragraph currently says each edge
carries "a single integer: its epipolar inlier count n_ij". Rewrite that paragraph:

```cpp
// The view graph G = (V,E) has the images as nodes and the geometrically verified pairs as
// edges, each carrying one strength s_ij = n_ij * c_ij: its epipolar inlier count n_ij discounted
// by c_ij, the fraction of the frame those inliers cover (ComputePairCoverage, the grid the pair
// weighting measures on). Wrong edges -- the repeated-structure ("doppelganger") pairs a
// retrieval step happily proposes and two-view geometry happily verifies -- are found purely from
// how that strength is distributed over the triangles of the graph: a true edge is, in every
// triangle it belongs to, comparable to the strongest edge of that triangle, while a false edge is
// systematically the weak side of triangles built around true edges. The paper weighs edges by
// n_ij alone; the coverage is what tells a doppelganger with more inliers than the true junction
// beside it (matches on the duplicated object and nowhere else) from that junction (matches over
// the whole overlap), which the counts and the triangles cannot.
```

`ComputeTripletScores` becomes `TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore, int gridSize);`
and its comment says the strength is `s_ij = n_ij * c_ij`, the score `q_ij` is the mean over the
edge's triplets of `s_ij / max_{(k,l) in t} s_kl`, `gridSize` is the coverage grid
(`PairsWeightingConfig::gridSize`, so the filter measures coverage on the grid the pair weighting
does), and a verified pair whose matches are not stored has a coverage of 0 and is not an edge.

`libs/SFM/ViewGraphTriplets.cpp`, `ComputeTripletScores`:
- add `#include "PairsWeighting.h"` if the header does not already bring it in;
- `std::vector<unsigned> edgeInliers; // n_ij` becomes `std::vector<float> edgeStrength; // s_ij = n_ij * c_ij`;
- in the edge loop, after the existing admission test:

```cpp
		// s_ij: the count discounted by the fraction of the frame the inliers cover. A verified
		// pair whose matches are not stored has nothing to measure coverage on and is no edge --
		// EvaluateSurvivorGraph still counts it, so it ends up unscored and kept, which is what
		// "no evidence" means here.
		const float strength = (float)numInliers *
			ComputePairCoverage(pair, scene.images[pair.ID1], scene.images[pair.ID2], gridSize);
		if (strength <= 0.f)
			continue;
```
  with `edgeStrength.emplace_back(strength)` on insertion and `else if (strength > edgeStrength[...])`
  on a duplicate (the comment above the loop says "weighted by the strongest of them" -- keep it,
  it is now literally true);
- step 5: `const float maxStrength = MAXF3(edgeStrength[e0], edgeStrength[e1], edgeStrength[e2]);`,
  `ASSERT(maxStrength > 0.f, "ComputeTripletScores: triplet with no strength");`, and the three sums
  divide `edgeStrength[e]` by it (`double` arithmetic as now);
- the step-5 comment's `q^t_ij = n_ij / max n_kl` becomes `q^t_ij = s_ij / max s_kl`.

`FilterPairsByTriplets` passes `weightingCfg.gridSize`:
`ComputeTripletScores(scene, minScore, weightingCfg.gridSize)`.

- [ ] **Step 5: The other callers, the CSV, the help text, the design note**

- `libs/SFM/PairsMatcher.h`: `static bool ExportPairsCSV(const Scene& scene, const String& fileName, float minWeight, int gridSize);`
  (drop the `= 0.f` default: the one caller passes both).
- `libs/SFM/PairsMatcher.cpp`: `ComputeTripletScores(scene, 0.f, gridSize)`; the header line becomes
  `"ImageA,ImageB,NumMatches,Coverage,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle,TripletScore\n"`
  and each row writes `ComputePairCoverage(pair, scene.images[pair.ID1], scene.images[pair.ID2], gridSize)`
  right after the match count. Extend the comment above: the coverage is exported so the
  discount can be replayed offline against the raw count.
- `libs/SFM/Scene.cpp`, `ExportMatchingCSVs`: `ExportPairsCSV(scene, config.exportPairsCSV, config.minPairWeight, config.matchCfg.weightingCfg.gridSize)`.
- `libs/SFM/PythonWrapper.cpp`: `ComputeTripletScoresDict(const Scene& scene, float minScore, int gridSize)`
  calling `ComputeTripletScores(scene, minScore, gridSize)`; where the function is registered with
  boost.python, expose the new argument as `grid_size` with the default `PairsWeightingConfig().gridSize`
  in the same way the file gives other optional arguments their defaults (look at how `ExportRetrievalRankingsCSVFile`'s `maxRank` is exposed and follow it); extend the docstring/comment: the scores depend on the coverage grid.
- `apps/CreateStructure/CreateStructure.cpp`, `--filter-triplets` help: `"disambiguate the matched view graph with the camera-triplet filter (Manam & Govindu, CVPR 2024): remove the pairs whose inlier count, discounted by the image area those inliers cover, is systematically weak in the triangles they belong to"`.
- `docs/design/TripletDisambiguation.md`: the Overview's "**one integer per edge**: the epipolar inlier count" becomes "one strength per edge: the epipolar inlier count discounted by the fraction of the frame the inliers cover", and the algorithm's `n_ij = ImagePair::GetNumFilteredInliers()` line becomes `s_ij = n_ij * c_ij`, `n_ij = ImagePair::GetNumWeightedInliers()` and `c_ij = ComputePairCoverage(...)`, with `q^t_ij = s_ij / max s_kl`. One or two sentences on why (the doppelganger with more inliers than the true junction); the full rewrite is Task 4.

- [ ] **Step 6: Build everything and run the SFM suite**

Run: `cd make && ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM && ./bin/Release/Tests 1`
Expected: builds without warnings; `TripletFilterTest`, `TripletAutoTauTest`, `TripletCoverageTest` PASSED; exit code 0; no other test regresses (`grep -i FAILED` finds only the deliberate "matching failed" line of another test).

Then apply each mutation below one at a time, rebuild `Tests`, confirm the named assertion fails, revert:

| mutation | what fails |
|---|---|
| `strength = (float)numInliers` (coverage dropped) | TripletCoverageTest: scores 0.667 0.667 1.0, and the filter removes the chain |
| `MAXF` instead of `MINF` over the two images in `ComputePairCoverage` | TripletCoverageTest: (0,2) coverage 1.0, score 1.0 |
| `gridSize` ignored in `ComputePairCoverage` (10 hard-coded) | TripletCoverageTest: 2x2 grid score 0.15 instead of 0.75 |

- [ ] **Step 7: Commit**

```bash
git add libs/SFM/PairsWeighting.h libs/SFM/PairsWeighting.cpp libs/SFM/ViewGraphTriplets.h libs/SFM/ViewGraphTriplets.cpp libs/SFM/PairsMatcher.h libs/SFM/PairsMatcher.cpp libs/SFM/Scene.cpp libs/SFM/PythonWrapper.cpp apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.cpp apps/Tests/TestsSFM.h apps/Tests/Tests.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: weigh triplet edges by inliers discounted by the image area they cover

A doppelganger's matches lie on the duplicated object alone while a true adjacent pair's
spread over the whole overlap, and on the ambiguous small sets the doppelganger has the
larger count (oats 880 against 627, cup 443 against 358), so the paper's count-based score
ranks it first and one such edge folds the model. The strength of an edge is now its inlier
count times the fraction of a grid its inliers occupy, measured by the same function the pair
weighting uses; pairs.csv exports the coverage beside the count."
```

### Task 7: A triangle of three inlier-deficient pairs is no evidence

Spec §3.6. On ToH — a 338-frame orbit of a round, three-fold symmetric temple — both arms fold
the orbit onto itself at a frame gap of 110 (one third of a turn), and Task 6's coverage cannot
see it: the whole frame is the duplicated object, so look-alike pairs have the coverage of true
ones. Nothing in the triangles sees it either, because look-alike pairs form triangles among
themselves (three copies of one facade, each pair as strong as the other two) and such a triangle
scores every edge at 1. What separates them is a deficit: two-view geometry reads a look-alike pair
as a near-duplicate viewpoint (median ray angle 2-3 degrees, the angle of a consecutive pair), yet
it carries a fifth of the inliers a consecutive pair carries, because only the repeated structure
matches. The *yield* of a pair is how much of what its two images can deliver it delivered,
against what pairs at its ray angle deliver in this graph; a triangle whose three edges all yield
less than `minYield` = 0.4 is a doppelganger triangle and contributes zero to its edges' score sums
while still counting in the divisor. Replayed offline, ToH keeps zero edges in every look-alike
band and its closure and consecutive pairs as before; every small set replays identically to
Task 6. The yield never scales a strength and never removes a pair on its own — both were tried
and both lose a set (spec §3.6).

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`TripletFilterConfig::minYield`, `TripletScores::numDoppelgangerTriplets`, `ComputeTripletScores` signature, header and function comments)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`ComputeEdgeYields`; `ComputeTripletScores` step 1 records inliers and ray angle per edge, step 5 skips doppelganger triangles; `FilterPairsByTriplets` passes `config.minYield` and logs the count)
- Modify: `libs/SFM/PythonWrapper.cpp` (`compute_triplet_scores` gains `min_yield`, the dict gains `num_doppelganger_triplets`, `TripletFilterConfig` exposes `min_yield`)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (`--filter-triplets` help text)
- Modify: `apps/Tests/TestsSFM.cpp` (new `TripletYieldTest`; the three existing triplet tests switch the rule off), `apps/Tests/TestsSFM.h`, `apps/Tests/Tests.cpp` (register it after `TripletCoverageTest`)
- Modify: `docs/design/TripletDisambiguation.md` (the Overview's strength sentence and a yield step in "The algorithm"; the full rewrite stays Task 4)

**Interfaces:**
- Consumes: `ImagePair::meanRayAngle` (radians; the median triangulation angle over the track-forming matches, 0 when there is no measurable baseline), `ImagePair::GetNumWeightedInliers()`, the edge arrays `edgeImages`/`edgeStrength` and the `ForEachTriplet` walk already inside `ComputeTripletScores`.
- Produces: `float TripletFilterConfig::minYield = 0.4f`; `unsigned TripletScores::numDoppelgangerTriplets`; `TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore, float minYield, int gridSize)`; Python `compute_triplet_scores(scene, min_score=0, min_yield=TripletFilterConfig().minYield, grid_size=PairsWeightingConfig().gridSize)` and `TripletFilterConfig.min_yield`.

- [ ] **Step 1: Write the failing test**

Add to `apps/Tests/TestsSFM.cpp` right after `TripletCoverageTest`:

```cpp
// Look-alike copies of one structure vouch for one another: the triangles they form among
// themselves score every edge at 1 whatever the counts, and no per-pair statistic tells such a
// pair from a true one -- its coverage is the true pair's, its ray angle is a consecutive pair's.
// What does is the yield: a look-alike pair delivers a fraction of the inliers a pair at its ray
// angle delivers between these images, because only the repeated structure matches. A 15-image
// walk in five 3-image copies (0-2, 3-5, 6-8, 9-11, 12-14): consecutive images share 1000
// inliers at 2 degrees, images two apart 600 at 4 degrees (every consecutive triple is a
// triangle, so the walk is one triplet component), and the first image of every copy pairs with
// the first image of every other copy on 300 inliers at 1 degree -- a near-duplicate viewpoint
// by its geometry, with less than a third of the inliers a real one delivers.
//   capacities: every image's strongest pair carries 1000, so u = 1.0 / 0.6 / 0.3
//   envelope: bin 1 holds the ten look-alikes (90th percentile 0.3), bin 2 the fourteen
//   consecutive pairs (1.0), bin 4 the thirteen gap-2 pairs (0.6); the suffix maximum lifts
//   bin 1 to 1.0, so the look-alikes yield 0.3 and everything else 1.0
//   (0,3): triangles (0,1,3) and (0,2,3) are mixed and give 300/1000 each; (0,3,6), (0,3,9)
//   and (0,3,12) are look-alike triangles and give nothing: (0.3 + 0.3 + 0 + 0 + 0) / 5 = 0.12
//   (0,6): only look-alike triangles, with 3, 9 and 12: 0
//   (0,1): 1 in (0,1,2) and in (0,1,3); (0,2): 600/1000 in (0,1,2) and in (0,2,3): 0.6
//   31 triplets: 13 consecutive triples, 10 among the five look-alikes, 8 mixed
// Without the rule the same edges score 0.72 and 1.0, and the filter keeps the six look-alike
// pairs between non-adjacent copies at any ceiling: that is the fold on ToH.
bool TripletYieldTest()
{
	TD_TIMER_START();
	const PairsWeightingConfig weightingCfg;
	const auto build = [](Scene& scene) {
		AddTripletImages(scene, 15);
		const auto add = [&scene](IIndex a, IIndex b, unsigned numInliers, float rayAngleDeg) {
			AddTripletPair(scene, a, b, numInliers);
			scene.pairs.Last().meanRayAngle = (float)D2R(rayAngleDeg);
		};
		for (IIndex i = 0; i + 1 < 15; ++i)
			add(i, i + 1, 1000, 2.f);
		for (IIndex i = 0; i + 2 < 15; ++i)
			add(i, i + 2, 600, 4.f);
		for (IIndex a = 0; a < 15; a += 3)
			for (IIndex b = a + 3; b < 15; b += 3)
				add(a, b, 300, 1.f);
	};
	Scene scene;
	build(scene);
	const unsigned idx01 = 0, idx02 = 14, idx03 = 27, idx06 = 28; // in order of insertion
	const TripletFilterConfig defaults;
	const TripletScores scores = ComputeTripletScores(scene, 0.f, defaults.minYield, weightingCfg.gridSize);
	if (scores.numTriplets != 31 || scores.numDoppelgangerTriplets != 10 || scores.numScoredPairs != 37 ||
		!ISEQUAL(scores.scores[idx01], 1.f) || !ISEQUAL(scores.scores[idx02], 0.6f) ||
		!ISEQUAL(scores.scores[idx03], 0.12f) || !ISEQUAL(scores.scores[idx06], 0.f)) {
		VERBOSE("TripletYieldTest FAILED: %u triplets (%u doppelganger), %u scored; (0,1) %g (0,2) %g (0,3) %g (0,6) %g; "
			"expected 31 (10), 37; 1 0.6 0.12 0",
			scores.numTriplets, scores.numDoppelgangerTriplets, scores.numScoredPairs,
			scores.scores[idx01], scores.scores[idx02], scores.scores[idx03], scores.scores[idx06]);
		return false;
	}
	// minYield 0 is the paper's scoring: the look-alike triangles count like any other
	const TripletScores off = ComputeTripletScores(scene, 0.f, 0.f, weightingCfg.gridSize);
	if (off.numDoppelgangerTriplets != 0 || !ISEQUAL(off.scores[idx03], 0.72f) || !ISEQUAL(off.scores[idx06], 1.f) ||
		!ISEQUAL(off.scores[idx01], 1.f) || !ISEQUAL(off.scores[idx02], 0.6f)) {
		VERBOSE("TripletYieldTest FAILED: with the rule off, %u doppelganger triplets; (0,3) %g (0,6) %g (0,1) %g (0,2) %g; "
			"expected 0; 0.72 1 1 0.6",
			off.numDoppelgangerTriplets, off.scores[idx03], off.scores[idx06], off.scores[idx01], off.scores[idx02]);
		return false;
	}
	// The filter: G_LCT has 15 nodes and max degree 8 (image 3: 0,1,2,4,5,6,9,12), so the ceiling
	// at m = 0.6 is 0.6 * (1 - 8/15) + 8/15 = 0.813; the fourteen consecutive pairs score 1 and
	// hold all fifteen images together, so the ceiling applies as given and they are all that
	// stays. With the rule off the six look-alike pairs between non-adjacent copies score 1 too
	// and stay with them.
	TripletFilterConfig filterCfg;
	filterCfg.enabled = true;
	std::set<std::pair<IIndex,IIndex>> expected;
	for (IIndex i = 0; i + 1 < 15; ++i)
		expected.emplace(i, i + 1);
	const unsigned numRemoved = FilterPairsByTriplets(scene, filterCfg, weightingCfg);
	if (numRemoved != 23 || TripletKeptPairs(scene) != expected) {
		VERBOSE("TripletYieldTest FAILED: %u pairs removed, %u kept; expected 23 removed and the 14 consecutive pairs kept",
			numRemoved, (unsigned)scene.pairs.size());
		return false;
	}
	Scene sceneOff;
	build(sceneOff);
	filterCfg.minYield = 0.f;
	const unsigned numRemovedOff = FilterPairsByTriplets(sceneOff, filterCfg, weightingCfg);
	std::set<std::pair<IIndex,IIndex>> expectedOff(expected);
	for (const auto& lookAlike : {std::make_pair(0u,6u), std::make_pair(0u,9u), std::make_pair(0u,12u),
			std::make_pair(3u,9u), std::make_pair(3u,12u), std::make_pair(6u,12u)})
		expectedOff.emplace((IIndex)lookAlike.first, (IIndex)lookAlike.second);
	if (numRemovedOff != 17 || TripletKeptPairs(sceneOff) != expectedOff) {
		VERBOSE("TripletYieldTest FAILED: with the rule off, %u pairs removed, %u kept; expected 17 removed, "
			"the 14 consecutive pairs and the 6 look-alike pairs between non-adjacent copies kept",
			numRemovedOff, (unsigned)sceneOff.pairs.size());
		return false;
	}
	VERBOSE("TripletYieldTest PASSED: look-alike triangles give no evidence, (0,6) scores 0 against 1 with the rule off, "
		"and the filter keeps the walk alone (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
```

Declare `bool TripletYieldTest();` in `apps/Tests/TestsSFM.h` right after `TripletCoverageTest`,
and register it in `apps/Tests/Tests.cpp` right after `TripletCoverageTest`'s block, in the same
form.

The three existing triplet tests build scenes whose pairs carry no ray angle (every
`meanRayAngle` is 0, one bin, nothing for an envelope to measure) and assert hand-computed
literals, so they run the paper's scoring: every `ComputeTripletScores(scene, m, weightingCfg.gridSize)`
call in `TripletFilterTest`, `TripletAutoTauTest` and `TripletCoverageTest` becomes
`ComputeTripletScores(scene, m, 0.f, weightingCfg.gridSize)`, and every `TripletFilterConfig` they
build sets `filterCfg.minYield = 0.f`, each with a one-line comment: "no ray angles here, so the
yield rule is off; it is TripletYieldTest's subject". Do not change their literals.

- [ ] **Step 2: Run the suite to verify it fails**

Run: `cd make && ninja -f build-Release.ninja Tests 2>&1 | tail -5`
Expected: compile errors — `minYield`, `numDoppelgangerTriplets` and the four-argument
`ComputeTripletScores` do not exist yet.

- [ ] **Step 3: The configuration, the statistic and the signature**

In `libs/SFM/ViewGraphTriplets.h`:

```cpp
struct SFM_API TripletFilterConfig
{
	bool enabled = false;   // remove the pairs the triplet score rejects (opt-in, see docs/design/TripletDisambiguation.md)
	// The paper's tau(m) is a ceiling: below it, the threshold is the strictest one whose survivor
	// graph keeps 99% of the unfiltered largest component together. Off, tau(m) is applied as given.
	bool autoTau = true;
	// The paper's minimum edge score m, in [0,1] (the domain this implementation enforces): 0.6
	// generic/large-scale, 0.9 highly ambiguous, 0.3 medium/small ambiguous. With autoTau this is
	// the ceiling the threshold is derived from and never exceeds.
	float minScore = 0.6f;
	// A triangle whose three pairs all yield less than this fraction of the inliers pairs at their
	// ray angle deliver in this graph (ComputeTripletScores) is a doppelganger triangle -- look-alike
	// copies vouching for one another -- and gives its edges no evidence. 0 switches the rule off.
	float minYield = 0.4f;
};
```

`TripletScores` gains, after `numTripletComponents`:

```cpp
	unsigned numDoppelgangerTriplets; // triplets of G_LCT whose three edges all yield below minYield: counted, no evidence
```

The declaration becomes
`TripletScores SFM_API ComputeTripletScores(const Scene& scene, float minScore, float minYield, int gridSize);`
and its comment gains, after the sentence ending "measures coverage on the grid the pair weighting does).":

```
// The yield of an edge is u_ij / H(theta_ij), capped at 1: u_ij = n_ij / min(K_i, K_j) with K_i
// the inlier count of image i's strongest pair, and H the graph's own envelope -- the 90th
// percentile of u over the edges in each 1-degree bin of median ray angle (bins holding at least
// five edges), made non-increasing in the angle. A triplet whose three edges all yield less than
// minYield is a doppelganger triplet -- three look-alike copies vouching for one another, each
// pair reading as a near-duplicate viewpoint while delivering a fraction of the inliers such a
// pair delivers -- and adds nothing to its edges' score sums while still counting in their
// divisor; minYield 0 is the paper's scoring.
```

In the header paragraph above `TripletFilterConfig`, after the sentence ending "which the counts
and the triangles cannot.", add:

```
// Coverage cannot see a look-alike that fills the frame -- a round, symmetric building seen from
// a third of a turn away -- and neither can the triangles, since such pairs form triangles among
// themselves that score every edge at 1. What can is the yield: two-view geometry reads such a
// pair as a near-duplicate viewpoint, yet it delivers a fraction of the inliers a near-duplicate
// pair of these images delivers, because only the repeated structure matches. A triangle whose
// three pairs all yield poorly is no evidence for any of them.
```

- [ ] **Step 4: The yields, and the triangles that give no evidence**

In `libs/SFM/ViewGraphTriplets.cpp`, step 1 of `ComputeTripletScores` keeps, beside
`edgeStrength`, the inlier count and ray angle of the scene pair that supplied the strength:

```cpp
	std::vector<float> edgeStrength;    // s_ij = n_ij * c_ij
	std::vector<unsigned> edgeInliers;  // n_ij of the scene pair that supplied the strength
	std::vector<float> edgeRayAngle;    // its median ray angle, radians
	...
		if (inserted.second) {
			edgeImages.emplace_back(imagePair);
			edgeStrength.emplace_back(strength);
			edgeInliers.emplace_back(numInliers);
			edgeRayAngle.emplace_back(pair.meanRayAngle);
		} else if (strength > edgeStrength[inserted.first->second]) {
			edgeStrength[inserted.first->second] = strength;
			edgeInliers[inserted.first->second] = numInliers;
			edgeRayAngle[inserted.first->second] = pair.meanRayAngle;
		}
```

Add, above `ComputeTripletScores` in the anonymous/static section of the file:

```cpp
// The yield of every edge: how much of what its two images can deliver the pair delivered,
// against what pairs at its ray angle deliver in this graph. u_e = n_e / min(K_i, K_j), K_i the
// inlier count of image i's strongest edge; the envelope H is the 90th percentile of u over the
// edges of each 1-degree bin of ray angle among bins holding at least five edges, made
// non-increasing in the angle by a suffix maximum (a near-duplicate viewpoint never promises
// less than a wider one), so a bin without an envelope of its own takes the nearest populated
// bin above it and the bins above the highest populated one keep its value; the yield is
// min(1, u_e / H). No populated bin at all -- fewer than five edges everywhere -- means no
// envelope and every yield 1. The percentile, the bin width and the bin floor are properties of
// the estimate, not of the scene: 75, 90 and 95 replay identically on every reference set.
static std::vector<float> ComputeEdgeYields(const std::vector<PairIdx>& edgeImages,
	const std::vector<unsigned>& edgeInliers, const std::vector<float>& edgeRayAngle, IIndex numImages)
{
	constexpr unsigned numBins = 90;        // 1-degree bins; 90 degrees and beyond share the last
	constexpr size_t minEdgesPerBin = 5;
	constexpr float percentile = 0.9f;
	const uint32_t numEdges = (uint32_t)edgeImages.size();
	std::vector<float> yields(numEdges, 1.f);
	std::vector<unsigned> capacity(numImages, 0);
	for (uint32_t e = 0; e < numEdges; ++e) {
		capacity[edgeImages[e].i] = MAXF(capacity[edgeImages[e].i], edgeInliers[e]);
		capacity[edgeImages[e].j] = MAXF(capacity[edgeImages[e].j], edgeInliers[e]);
	}
	std::vector<float> delivered(numEdges);
	std::vector<unsigned> binOfEdge(numEdges);
	std::vector<std::vector<float>> bins(numBins);
	for (uint32_t e = 0; e < numEdges; ++e) {
		delivered[e] = (float)edgeInliers[e] / (float)MINF(capacity[edgeImages[e].i], capacity[edgeImages[e].j]);
		binOfEdge[e] = (unsigned)MINF((int)std::floor(R2D(edgeRayAngle[e])), (int)numBins - 1);
		bins[binOfEdge[e]].push_back(delivered[e]);
	}
	std::vector<float> envelope(numBins, -1.f);
	for (unsigned b = 0; b < numBins; ++b) {
		std::vector<float>& values = bins[b];
		if (values.size() < minEdgesPerBin)
			continue;
		const size_t rank = MINF(values.size() - 1, (size_t)((float)values.size() * percentile));
		std::nth_element(values.begin(), values.begin() + rank, values.end());
		envelope[b] = values[rank];
	}
	float best = -1.f;
	for (unsigned b = numBins; b-- > 0; ) {
		best = MAXF(best, envelope[b]);
		envelope[b] = best;
	}
	if (best < 0.f)
		return yields;
	for (unsigned b = 1; b < numBins; ++b)
		if (envelope[b] < 0.f)
			envelope[b] = envelope[b - 1];
	for (uint32_t e = 0; e < numEdges; ++e)
		yields[e] = MINF(1.f, delivered[e] / envelope[binOfEdge[e]]);
	return yields;
}
```

(`binOfEdge` uses `std::floor` of a non-negative angle, so the cast to `int` is safe; `envelope`
values are percentiles of `delivered`, which is in `(0,1]`, so the division is safe once `best`
is non-negative.)

Step 5 becomes:

```cpp
	// 5. Score the edges of G_LCT (Algorithm 1 steps 4-8): a second streaming pass over the
	// triplets of the largest component accumulates the per-triplet maximum s_kl and the per-edge
	// running sum of q^t_ij = s_ij / max_{(k,l) in t} s_kl. A triplet whose three edges all yield
	// below minYield is look-alike copies vouching for one another: it stays in the divisor
	// (numTripletsOfEdge) and adds nothing to the sum.
	const std::vector<float> yields = minYield > 0.f
		? ComputeEdgeYields(edgeImages, edgeInliers, edgeRayAngle, numImages) : std::vector<float>();
	std::vector<double> scoreSumOfEdge(numEdges, 0.0);
	ForEachTriplet([&](uint32_t e0, uint32_t e1, uint32_t e2) {
		if (components.Find(e0) != largestComponent)
			return;
		if (minYield > 0.f && MAXF3(yields[e0], yields[e1], yields[e2]) < minYield) {
			++result.numDoppelgangerTriplets;
			return;
		}
		const float maxStrength = MAXF3(edgeStrength[e0], edgeStrength[e1], edgeStrength[e2]);
		ASSERT(maxStrength > 0.f, "ComputeTripletScores: triplet with no strength");
		scoreSumOfEdge[e0] += (double)edgeStrength[e0] / (double)maxStrength;
		scoreSumOfEdge[e1] += (double)edgeStrength[e1] / (double)maxStrength;
		scoreSumOfEdge[e2] += (double)edgeStrength[e2] / (double)maxStrength;
	});
```

`result.numDoppelgangerTriplets` is zeroed with the other counters at the top of the function.
The signature gains `float minYield` before `int gridSize`.

In `FilterPairsByTriplets`: guard `config.minYield` exactly as `minScore` is guarded (NaN → 0,
otherwise clamped to `[0,1]`, with the same style of warning naming "minimum yield"), pass it to
`ComputeTripletScores`, and extend the final log line:

```cpp
	VERBOSE("Triplet filter: kept %u/%u scene pairs (tau %.3f; %u nodes, max degree %u; "
		"%u triplets in %u components, %u doppelganger triplets gave no evidence; %u below tau removed, %u unscored kept)",
		numKept, numPairs, tau,
		tripletScores.numNodes, tripletScores.maxDegree,
		tripletScores.numTriplets, tripletScores.numTripletComponents, tripletScores.numDoppelgangerTriplets,
		numBelowTau, numUnscored);
```

- [ ] **Step 5: Callers, the binding, the help text, the design note**

- `libs/SFM/PythonWrapper.cpp`: `ComputeTripletScoresDict(const Scene& scene, float minScore, float minYield, int gridSize)` passes `minYield` through and adds `out["num_doppelganger_triplets"] = tripletScores.numDoppelgangerTriplets;` after `num_triplet_components`; the `def` becomes `(arg("scene"), arg("min_score")=0.f, arg("min_yield")=SFM::TripletFilterConfig().minYield, arg("grid_size")=SFM::PairsWeightingConfig().gridSize)`; the comment above the function gains one sentence: "min_yield is the doppelganger-triplet bar of TripletFilterConfig (0 for the paper's scoring)". `TripletFilterConfig` exposes `.def_readwrite("min_yield", &SFM::TripletFilterConfig::minYield)` after `min_score`.
- `apps/CreateStructure/CreateStructure.cpp`: the `--filter-triplets` help text becomes: "disambiguate the matched view graph with the camera-triplet filter (Manam & Govindu, CVPR 2024): remove the pairs whose inlier count, discounted by the image area those inliers cover, is systematically weak in the triangles they belong to; a triangle of three pairs that all deliver far fewer inliers than pairs at their ray angle do is look-alike copies vouching for one another and counts for nothing".
- `docs/design/TripletDisambiguation.md`: in the Overview, after "which is what tells apart a doppelganger with more inliers than the true junction beside it." add: "A triangle whose three pairs all *yield* poorly — each reads as a near-duplicate viewpoint by its ray angle yet delivers a fraction of the inliers such pairs deliver between these images — is look-alike copies vouching for one another and carries no evidence." In "The algorithm", step 2 becomes: "**Score.** `q^t_ij = s_ij / max_{(k,l) in t} s_kl` per triplet `t`; `q_ij` is its mean over the triplets of `G_LCT` containing `(i,j)`. A triplet whose three edges all yield less than `minYield` (0.4) contributes 0 to that mean: the yield of an edge is `n_ij / min(K_i, K_j)` (each `K` the image's strongest pair) against the graph's own 90th-percentile envelope of that ratio per degree of median ray angle, capped at 1."

- [ ] **Step 6: Build, run the SFM suite, mutate**

Run: `cd make && ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM 2>&1 | tail -3 && ./bin/Release/Tests 1 2>&1 | /usr/bin/grep -E 'Triplet|PASSED|FAILED' | tail -8`
Expected: `TripletFilterTest`, `TripletAutoTauTest`, `TripletCoverageTest` and `TripletYieldTest`
PASSED, every other test of the suite unchanged, exit code 0.

Then each mutation in turn (rebuild `Tests`, run, restore):

| mutation | expected failure |
|---|---|
| in step 5, delete the `++result.numDoppelgangerTriplets; return;` branch so doppelganger triplets score normally | `TripletYieldTest`: (0,6) scores 1, (0,3) 0.72, 0 doppelganger triplets |
| `MAXF3(yields...) < minYield` → `MINF3(yields...) < minYield` (any deficient edge silences the triangle) | `TripletYieldTest`: (0,1) scores 0.5 — its mixed triangle (0,1,3) is silenced |
| delete the suffix-maximum loop (`best`) in `ComputeEdgeYields` | `TripletYieldTest`: bin 1's own envelope is 0.3, the look-alikes yield 1, 0 doppelganger triplets |

Report the exact failing line of each in the report file, and confirm the suite is green again
with the mutations reverted.

- [ ] **Step 7: Commit**

```bash
git add libs/SFM/ViewGraphTriplets.h libs/SFM/ViewGraphTriplets.cpp libs/SFM/PythonWrapper.cpp apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.cpp apps/Tests/TestsSFM.h apps/Tests/Tests.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: give a triangle of three inlier-deficient pairs no say in the triplet score

On a round, symmetric building every frame has look-alikes a third of a turn away, and
the look-alike pairs form triangles among themselves that score each edge at 1: the
coverage cannot see them (the whole frame is the repeated structure) and the orbit folds.
What tells them apart is a deficit: two-view geometry reads such a pair as a near-duplicate
viewpoint, yet it delivers a fraction of the inliers a near-duplicate pair of these images
delivers, because only the repeated structure matches. A pair's yield is its inlier count
against its two images' capacity and the graph's own envelope of that ratio per degree of
ray angle; a triplet whose three pairs all yield below 0.4 contributes nothing to their
scores. Replayed on the reference sets this empties every look-alike band of the orbit and
changes nothing on the small sets; the yield never scales a strength."
```

### Task 8: The descent does not chase stragglers

Spec §3.7. The descent of Task 5 lowers the threshold until 99 % of the unfiltered largest
component is back in one piece. On the internet collections the ceiling leaves one big component
and a tail of one- to four-image stragglers, each attached to the graph by a single weak pair, and
fetching them costs everything between the ceiling and that pair's score: church descends from
0.723 to 0.426 and keeps 800 more pairs for 20 images; radcliffe from 0.734 to 0.293 for twelve.
A straggler is not an over-split. From now on a *piece* is a component of the survivor graph at
the ceiling that lies inside the unfiltered graph's largest component and holds at least
`ceil(n0 / 100)` images (`n0` that component's size), and the threshold is the strictest one whose
survivor graph joins every piece — its largest component holds at least as many images as the
pieces hold together. Stragglers are left as they are, and so is any component of the unfiltered
graph outside its largest one: no threshold can join an island that shares no pair with the rest,
and counting it as a piece would set a target no candidate reaches. On sets under 101 images every
component inside the largest one is a piece, so the six small sets and their tests are unchanged.

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`SurvivorGraph` gains `numPieces` and `numInPieces`; `EvaluateSurvivorGraph` gains `minPiece`; comments)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`EvaluateSurvivorGraph` counts pieces; `FilterPairsByTriplets` derives the target from the pieces and logs them)
- Modify: `apps/Tests/TestsSFM.cpp` (`TripletAutoTauTest` gains the straggler scene)
- Modify: `docs/design/TripletDisambiguation.md` (the threshold paragraph)
- Modify: `docs/superpowers/specs/2026-09-05-view-graph-disambiguation-design.md` §3.2 first paragraph — already amended by the controller; do not touch.

**Interfaces:**
- Consumes: `EvaluateSurvivorGraph`, `TripletScores`, `FilterPairsByTriplets`'s existing descent (binary search over the distinct scores below the ceiling).
- Produces: `SurvivorGraph SFM_API EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau, unsigned minPiece = 1)`; `unsigned SurvivorGraph::numPieces` (components of at least `minPiece` nodes) and `unsigned SurvivorGraph::numInPieces` (nodes in them).

- [ ] **Step 1: Write the failing test**

In `apps/Tests/TestsSFM.cpp`, inside `TripletAutoTauTest`, right before its final `VERBOSE(... PASSED ...)`
line, add the straggler scene. It uses the test's existing `weightingCfg` (grid size 1) and follows
the test's existing pattern of building a `TripletFilterConfig` with `enabled = true` and
`minYield = 0.f`:

```cpp
	// Scene 5, the walk with stragglers. A 120-image walk: consecutive images share 1000 inliers,
	// images two apart 600, every consecutive triple a triangle. Two stragglers hang off it by one
	// weak triangle each -- image 120 off (0,1) with 50 and 40 inliers, image 121 off (60,61) the
	// same way -- and a two-image piece {122,123} (1000 inliers between them) hangs off the far end
	// through (119,122)=200, (119,123)=200 and (118,122)=150.
	//   scores: walk (i,i+1) 1.0, (i,i+2) 0.6; (122,123) 1.0; (119,122)=(119,123)=0.2, (118,122)=0.15;
	//           (0,120)=(60,121)=0.05, (1,120)=(61,121)=0.04
	//   G_LCT: 124 nodes, max degree 5 (images 60 and 61), ceiling 0.6*(1-5/124)+5/124 = 0.616
	//   at the ceiling: the walk (120 images), {122,123}, {120}, {121} -- and n0 = 124, so a piece
	//   is a component of at least ceil(124/100) = 2 images: two pieces holding 122 images, two
	//   stragglers. The strictest threshold joining both pieces is the 2-piece's bridge, 0.2.
	//   Everything from 0.2 up stays (the 600-inlier pairs included); the five weaker pairs go,
	//   and images 120 and 121 are left with no pair -- the 99% rule would have descended to
	//   0.05 for them and kept the two 0.05 pairs as well.
	Scene walk;
	AddTripletImages(walk, 124);
	for (IIndex i = 0; i + 1 < 120; ++i)
		AddTripletPair(walk, i, i + 1, 1000);
	for (IIndex i = 0; i + 2 < 120; ++i)
		AddTripletPair(walk, i, i + 2, 600);
	AddTripletPair(walk, 0, 120, 50);
	AddTripletPair(walk, 1, 120, 40);
	AddTripletPair(walk, 60, 121, 50);
	AddTripletPair(walk, 61, 121, 40);
	AddTripletPair(walk, 122, 123, 1000);
	AddTripletPair(walk, 119, 122, 200);
	AddTripletPair(walk, 118, 122, 150);
	AddTripletPair(walk, 119, 123, 200);
	const TripletScores walkScores = ComputeTripletScores(walk, 0.6f, 0.f, weightingCfg.gridSize);
	const SurvivorGraph walkUnfiltered = EvaluateSurvivorGraph(walk, walkScores.scores, 0.f);
	const SurvivorGraph walkCeiling = EvaluateSurvivorGraph(walk, walkScores.scores, walkScores.tau, 2);
	if (walkUnfiltered.largestComponent != 124 || walkCeiling.largestComponent != 120 ||
		walkCeiling.numPieces != 2 || walkCeiling.numInPieces != 122 || walkCeiling.numNodes != 124) {
		VERBOSE("TripletAutoTauTest FAILED: walk at the ceiling %g: component %u of %u, %u pieces holding %u; "
			"expected 120 of 124, 2 pieces holding 122",
			walkScores.tau, walkCeiling.largestComponent, walkUnfiltered.largestComponent,
			walkCeiling.numPieces, walkCeiling.numInPieces);
		return false;
	}
	TripletFilterConfig walkCfg;
	walkCfg.enabled = true;
	walkCfg.minYield = 0.f; // no ray angles here, so the yield rule is off; it is TripletYieldTest's subject
	const unsigned walkRemoved = FilterPairsByTriplets(walk, walkCfg, weightingCfg);
	const std::set<std::pair<IIndex,IIndex>> walkKept = TripletKeptPairs(walk);
	const std::set<std::pair<IIndex,IIndex>> walkGone{{0,120},{1,120},{60,121},{61,121},{118,122}};
	bool walkRight = walkRemoved == 5 && walkKept.size() == 240;
	for (const auto& pair : walkGone)
		walkRight = walkRight && walkKept.count(pair) == 0;
	walkRight = walkRight && walkKept.count({119,122}) == 1 && walkKept.count({119,123}) == 1 && walkKept.count({0,2}) == 1;
	if (!walkRight) {
		VERBOSE("TripletAutoTauTest FAILED: walk with stragglers removed %u pairs, kept %u; expected the five weakest "
			"pairs removed (both stragglers cut loose, the 2-piece joined at 0.2) and 240 kept",
			walkRemoved, (unsigned)walkKept.size());
		return false;
	}
```

Update the test's leading comment block (the one describing its scenes, if any) and its PASSED
message to mention the straggler scene.

- [ ] **Step 2: Run the suite to verify it fails**

Run: `cd make && ninja -f build-Release.ninja Tests 2>&1 | tail -5`
Expected: compile errors — `numPieces`, `numInPieces` and the four-argument `EvaluateSurvivorGraph`
do not exist yet.

- [ ] **Step 3: Pieces in the survivor graph**

In `libs/SFM/ViewGraphTriplets.h`:

```cpp
// The view graph the filter would leave behind at a given threshold: what the search judges.
struct SFM_API SurvivorGraph
{
	unsigned numNodes;          // images incident to at least one edge of the UNFILTERED graph
	unsigned largestComponent;  // images in the largest connected component of the kept edges
	unsigned numLowDegree;      // of those nodes, how many have degree < 2 in the kept graph
	unsigned numKept;           // kept edges: a scene pair duplicating an already-counted image
	                            // pair counts once, matching ComputeTripletScores' own collapse
	unsigned numPieces;         // components of the kept graph holding at least minPiece nodes
	unsigned numInPieces;       // nodes in those components; the rest are stragglers
};

// Evaluate the graph left by keeping every unscored pair and every pair scoring at or above `tau`.
// Pass tau = 0 for the unfiltered graph: scores lie in [0,1] and unscored pairs are always kept.
// Nodes are counted on the unfiltered graph, so an image that loses all its edges still counts as
// a node -- with degree 0, which is exactly what the low-degree test is there to catch.
// A component of at least minPiece nodes inside the unfiltered graph's largest component is a
// piece; the filter's descent joins pieces and lets stragglers be (see FilterPairsByTriplets).
SurvivorGraph SFM_API EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau, unsigned minPiece = 1);
```

In `libs/SFM/ViewGraphTriplets.cpp`, `EvaluateSurvivorGraph` initialises the two new counters to 0
with the others and, after the loop that fills `componentSize`, counts:

```cpp
	for (const auto& component : componentSize) {
		if (component.second < minPiece)
			continue;
		++result.numPieces;
		result.numInPieces += component.second;
	}
```

- [ ] **Step 4: The descent joins pieces**

In `FilterPairsByTriplets`, the block under `if (config.autoTau)` becomes:

```cpp
	if (config.autoTau) {
		// Below the ceiling, the threshold is the STRICTEST one that joins every piece: the largest
		// value whose survivor graph holds, in one component, every image that the ceiling's
		// pieces hold together. A piece is a component of the survivor graph at the ceiling that
		// lies inside the unfiltered largest component and holds at least 1% of it; anything
		// smaller is a straggler -- an image the graph vouches for through a single weak pair --
		// and so is anything outside that component, which no threshold can join. Fetching a
		// straggler would admit
		// every edge between the ceiling and that pair's score to gain one image (church: from
		// 0.72 to 0.43 for twenty such images). Stragglers are neither chased nor removed: an
		// unscored pair still carries them, and so does a bridge above the chosen threshold.
		// Everything scored below the threshold is either a weak true pair the pieces do not
		// need or a doppelganger, and nothing in the inlier counts tells the two apart -- on the
		// ambiguous-scene datasets the doppelganger pairs OUTSCORE the true low-overlap pairs --
		// so the only defensible cut keeps the strong edges and exactly enough of them. On the
		// small sets, where every component is a piece, that removes 66-96% of the pairs and
		// leaves a chain's two endpoints at degree 1, which is why there is no bar on how much is
		// removed and none on low-degree images.
		const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
		const unsigned minPiece = (unsigned)std::ceil(0.01 * (double)unfiltered.largestComponent);
		SurvivorGraph survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, ceiling, minPiece);
		const unsigned minComponent = survivor.numInPieces;
		const unsigned numPieces = survivor.numPieces;
		const unsigned numStragglers = survivor.numNodes - survivor.numInPieces;
		if (survivor.largestComponent < minComponent) {
			// The largest component only grows as tau falls, so among the distinct scores below
			// the ceiling, strictest first, the first that passes is a binary search away. The
			// loosest candidate keeps every scored pair -- the unfiltered graph itself, whose
			// largest component holds every piece -- so it always passes, and the ceiling
			// leaving a piece apart means at least one scored pair sits below it.
			std::vector<float> candidates;
			candidates.reserve(tripletScores.numScoredPairs);
			for (float score : tripletScores.scores)
				if (score >= 0.f && score < ceiling)
					candidates.push_back(score);
			std::sort(candidates.begin(), candidates.end(), std::greater<float>());
			candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
			ASSERT(!candidates.empty(), "FilterPairsByTriplets: the ceiling leaves a piece apart with no score below it");
			size_t lo = 0, hi = candidates.size() - 1;
			while (lo < hi) {
				const size_t mid = (lo + hi) / 2;
				if (EvaluateSurvivorGraph(scene, tripletScores.scores, candidates[mid]).largestComponent >= minComponent)
					hi = mid;
				else
					lo = mid + 1;
			}
			tau = candidates[lo];
			survivor = EvaluateSurvivorGraph(scene, tripletScores.scores, tau);
		}
		VERBOSE("Triplet filter: tau %.3f, %s (ceiling %.3f at m %.2f, d_max/|V| %.3f); the ceiling leaves "
			"%u pieces of at least %u images holding %u, %u stragglers; survivor graph keeps %u/%u images "
			"in its largest component, %u below degree 2 (%u before), and %u/%u distinct image pairs",
			tau, tau < ceiling ? "the strictest threshold that joins every piece" : "the ceiling applied as given",
			ceiling, minScore, degreeRatio, numPieces, minPiece, minComponent, numStragglers,
			survivor.largestComponent, unfiltered.largestComponent,
			survivor.numLowDegree, unfiltered.numLowDegree, survivor.numKept, unfiltered.numKept);
	}
```

Delete the previous comment and code of that block entirely (the 99 % target, `minComponent` from
`0.99 * largestComponent`, and the "keeps the graph together" wording); nothing of the old rule
stays. Check the file's header comment and the `TripletFilterConfig::autoTau` comment for the
words "99%" or "keeps the graph together" and reword them to "joins every piece the ceiling
leaves" — the header paragraph and the `autoTau` comment both mention the rule.

- [ ] **Step 5: The design note**

In `docs/design/TripletDisambiguation.md`, the paragraph or step describing the connectivity-driven
threshold (search for "99") states the new rule in two sentences: below the ceiling the threshold
is the strictest one whose survivor graph joins every piece — a component of the ceiling's survivor
graph that lies inside the unfiltered largest component and holds at least 1 % of it — and
stragglers smaller than that, or outside it, are neither chased nor removed.

- [ ] **Step 6: Build, run the SFM suite, mutate**

Run: `cd make && ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM 2>&1 | tail -3 && ./bin/Release/Tests 1 2>&1 | /usr/bin/grep -E 'Triplet|FAILED' | tail -8`
Expected: `TripletFilterTest`, `TripletAutoTauTest` (all five scenes), `TripletCoverageTest`,
`TripletYieldTest` PASSED, exit code 0. The four earlier scenes of `TripletAutoTauTest` must keep
every literal: on graphs under 101 nodes `minPiece` is 1, every component is a piece, and "every
piece joined" is what `ceil(0.99 * n)` rounds to there.

Then each mutation in turn (rebuild `Tests`, run, restore):

| mutation | expected failure |
|---|---|
| `minPiece` forced to 1 in `FilterPairsByTriplets` | walk scene: the target becomes 124, the descent goes to 0.05 and removes 2 pairs instead of 5 |
| `component.second < minPiece` → `component.second <= minPiece` in `EvaluateSurvivorGraph` | walk scene: `numPieces` 1, `numInPieces` 120 at the ceiling — the first assertion fails |
| `minComponent = survivor.numInPieces` → `survivor.largestComponent` (the target equals what the ceiling already has) | walk scene: tau stays at the ceiling, the 2-piece is not joined, 5 + 118 pairs removed |

Report the exact failing line of each in the report file, and confirm the suite is green again
with the mutations reverted.

- [ ] **Step 7: Commit**

```bash
git add libs/SFM/ViewGraphTriplets.h libs/SFM/ViewGraphTriplets.cpp apps/Tests/TestsSFM.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: the triplet threshold joins the pieces the ceiling leaves and lets stragglers be

On the internet collections the ceiling leaves one large component and a tail of one- to
four-image stragglers, each attached by a single weak pair, and descending to fetch them
admits every edge between the ceiling and that pair's score: church went from 0.72 to 0.43
and kept 800 more pairs for twenty images. A piece is a component of the ceiling's survivor
graph holding at least 1% of the unfiltered largest component; the threshold is now the
strictest one that joins every piece, and a straggler keeps whatever pairs sit above it.
On sets under 101 images every component is a piece and nothing changes."
```

### Task 9: The descent only repairs a shattered ceiling

Spec §3.8. The descent of Task 5, as Task 8 left it, lowers the threshold until every piece the
ceiling leaves is joined. That is the whole method on graphs the ceiling shatters — the small sets
and every exhaustively matched collection, where `d_max/|V|` is near 1, the ceiling sits at
0.94-0.995 and the pieces are fragments of two to forty images. On the church matched exhaustively
the ceiling (0.942) instead leaves two pieces of 143 and 85 images — the south facade with the canal
views, and the north facade, split the way Doppelgangers++ splits them — joined at 0.931 by one pair
of 253 inliers, and the descent joined them. A ceiling whose largest piece holds most of the images
has done the paper's job; what hangs below it is a straggler or the other face of a symmetric
building, and no threshold can tell which.

From now on the descent happens only when no piece holds a strict majority of the images the pieces
hold together. If the largest piece holds more than half of them, the ceiling is applied as given
and the smaller pieces stay apart, like stragglers. A strict majority, so that a graph cut into two
equal halves (the pan scene, three and three) is still repaired. Small sets (largest piece 17-39 %
of the images in pieces), indoor (28/152) and the pan scene are unchanged; the walk-with-stragglers
scene (120 of 122) becomes a ceiling-as-given case and its expectations change accordingly.

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`SurvivorGraph` gains `largestPiece`)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`EvaluateSurvivorGraph` fills it; `FilterPairsByTriplets` descends only when the ceiling shattered the graph, and says which case it is)
- Modify: `apps/Tests/TestsSFM.cpp` (`TripletAutoTauTest`: the walk scene's expectations, a new three-chains scene)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (`--triplet-auto-tau` help text)
- Modify: `docs/design/TripletDisambiguation.md` (the threshold paragraph)

**Interfaces:**
- Consumes: `SurvivorGraph {numNodes, largestComponent, numLowDegree, numKept, numPieces, numInPieces}`, `EvaluateSurvivorGraph(scene, scores, tau, minPiece = 1)` (Task 8), `AddTripletImages`, `AddTripletPair`, `TripletKeptPairs` (Tasks 5-7).
- Produces: `SurvivorGraph::largestPiece` (unsigned; the images in the largest piece, 0 when there is none). No signature changes.

- [ ] **Step 1: Rewrite the walk scene's expectations (the test now fails)**

In `apps/Tests/TestsSFM.cpp`, the walk-with-stragglers scene of `TripletAutoTauTest` (the comment block
beginning `// Scene 8, the walk with stragglers`). Replace the last three sentences of that comment
block — from `The strictest threshold joining both pieces is the 2-piece's bridge, 0.2.` to the end
of the block — with:

```cpp
	//   stragglers. The walk holds 120 of the 122 images in pieces, a majority: the ceiling has done
	//   its job and is applied as given. Everything scoring below 0.616 goes -- the 118 pairs two
	//   apart (0.6), the 2-piece's three bridges and the four straggler pairs, 125 in all -- and the
	//   120 kept pairs are the 119 consecutive ones and (122,123): the 2-piece stays its own
	//   component, images 120 and 121 are left with no pair. Before this rule the descent went to
	//   0.2 for the 2-piece and kept the 600-inlier pairs with it.
```

Extend the ceiling assertion so it also checks the largest piece — replace

```cpp
	if (walkUnfiltered.largestComponent != 124 || walkCeiling.largestComponent != 120 ||
		walkCeiling.numPieces != 2 || walkCeiling.numInPieces != 122 || walkCeiling.numNodes != 124) {
		VERBOSE("TripletAutoTauTest FAILED: walk at the ceiling %g: component %u of %u, %u pieces holding %u; "
			"expected 120 of 124, 2 pieces holding 122",
			walkScores.tau, walkCeiling.largestComponent, walkUnfiltered.largestComponent,
			walkCeiling.numPieces, walkCeiling.numInPieces);
		return false;
	}
```

with

```cpp
	if (walkUnfiltered.largestComponent != 124 || walkCeiling.largestComponent != 120 ||
		walkCeiling.numPieces != 2 || walkCeiling.numInPieces != 122 || walkCeiling.largestPiece != 120 ||
		walkCeiling.numNodes != 124) {
		VERBOSE("TripletAutoTauTest FAILED: walk at the ceiling %g: component %u of %u, %u pieces holding %u, the largest %u; "
			"expected 120 of 124, 2 pieces holding 122, the largest 120",
			walkScores.tau, walkCeiling.largestComponent, walkUnfiltered.largestComponent,
			walkCeiling.numPieces, walkCeiling.numInPieces, walkCeiling.largestPiece);
		return false;
	}
```

and replace the filter check — from `const std::set<std::pair<IIndex,IIndex>> walkGone` through the
`return false;` of its failure branch — with

```cpp
	const std::set<std::pair<IIndex,IIndex>> walkGone{{0,120},{1,120},{60,121},{61,121},{118,122},{119,122},{119,123},{0,2},{117,119}};
	bool walkRight = walkRemoved == 125 && walkKept.size() == 120;
	for (const auto& pair : walkGone)
		walkRight = walkRight && walkKept.count(pair) == 0;
	walkRight = walkRight && walkKept.count({0,1}) == 1 && walkKept.count({118,119}) == 1 && walkKept.count({122,123}) == 1;
	if (!walkRight) {
		VERBOSE("TripletAutoTauTest FAILED: walk with stragglers removed %u pairs, kept %u; expected the ceiling applied "
			"as given (its walk holds a majority): the 118 pairs two apart, the 2-piece's three bridges and the four "
			"straggler pairs removed, 120 kept",
			walkRemoved, (unsigned)walkKept.size());
		return false;
	}
```

- [ ] **Step 2: Add the three-chains scene (the test still fails)**

Immediately after the walk scene's filter check (before the `TripletAutoTauTest PASSED` line), add:

```cpp
	// Scene 9, three chains. 120 images in three walks of 40 (0-39, 40-79, 80-119): consecutive
	// images share 1000 inliers, images two apart 600, every consecutive triple a triangle. The
	// walks meet through one 500-inlier pair each, (39,40) and (79,80), and each of those sits in
	// two triangles closed by a 300-inlier pair: (38,40) and (39,41), (78,80) and (79,81).
	//   scores: (i,i+1) 1.0, (i,i+2) 0.6 inside a walk; (39,40)=(79,80)=0.5; the four 300-inlier
	//           pairs 0.3
	//   G_LCT: 120 nodes, max degree 4, ceiling 0.6*(1-4/120)+4/120 = 0.6133
	//   at the ceiling: only the consecutive pairs survive, so three pieces of 40 holding 120, no
	//   stragglers, and no piece holds a majority -- the ceiling shattered the graph, so the
	//   descent runs: 0.6 leaves the walks apart, 0.5 joins them through (39,40) and (79,80).
	//   tau 0.5: the four 300-inlier pairs go, 233 pairs stay.
	Scene chains;
	AddTripletImages(chains, 120);
	for (IIndex c = 0; c < 120; c += 40) {
		for (IIndex i = c; i + 1 < c + 40; ++i)
			AddTripletPair(chains, i, i + 1, 1000);
		for (IIndex i = c; i + 2 < c + 40; ++i)
			AddTripletPair(chains, i, i + 2, 600);
	}
	AddTripletPair(chains, 39, 40, 500);
	AddTripletPair(chains, 38, 40, 300);
	AddTripletPair(chains, 39, 41, 300);
	AddTripletPair(chains, 79, 80, 500);
	AddTripletPair(chains, 78, 80, 300);
	AddTripletPair(chains, 79, 81, 300);
	const TripletScores chainsScores = ComputeTripletScores(chains, 0.6f, 0.f, weightingCfg.gridSize);
	const SurvivorGraph chainsCeiling = EvaluateSurvivorGraph(chains, chainsScores.scores, chainsScores.tau, 2);
	if (!ISEQUAL(chainsScores.tau, 0.6f*(1.f-4.f/120.f)+4.f/120.f) || chainsCeiling.largestComponent != 40 ||
		chainsCeiling.numPieces != 3 || chainsCeiling.numInPieces != 120 || chainsCeiling.largestPiece != 40) {
		VERBOSE("TripletAutoTauTest FAILED: three chains at the ceiling %g: component %u, %u pieces holding %u, the largest %u; "
			"expected ceiling 0.6133, component 40, 3 pieces holding 120, the largest 40",
			chainsScores.tau, chainsCeiling.largestComponent, chainsCeiling.numPieces, chainsCeiling.numInPieces,
			chainsCeiling.largestPiece);
		return false;
	}
	TripletFilterConfig chainsCfg;
	chainsCfg.enabled = true;
	chainsCfg.minYield = 0.f;
	const unsigned chainsRemoved = FilterPairsByTriplets(chains, chainsCfg, weightingCfg);
	const std::set<std::pair<IIndex,IIndex>> chainsKept = TripletKeptPairs(chains);
	const std::set<std::pair<IIndex,IIndex>> chainsGone{{38,40},{39,41},{78,80},{79,81}};
	bool chainsRight = chainsRemoved == 4 && chainsKept.size() == 233;
	for (const auto& pair : chainsGone)
		chainsRight = chainsRight && chainsKept.count(pair) == 0;
	chainsRight = chainsRight && chainsKept.count({39,40}) == 1 && chainsKept.count({79,80}) == 1 && chainsKept.count({0,2}) == 1;
	if (!chainsRight) {
		VERBOSE("TripletAutoTauTest FAILED: three chains removed %u pairs, kept %u; expected the descent to 0.5 "
			"(no piece holds a majority): the four 300-inlier pairs removed, 233 kept",
			chainsRemoved, (unsigned)chainsKept.size());
		return false;
	}
```

Update the test's leading comment block (the scene list at the top of `TripletAutoTauTest`) with one
line for the three-chains scene, and change the PASSED line to:

```cpp
	VERBOSE("TripletAutoTauTest PASSED: the descent repairs a shattered ceiling and leaves a majority piece's "
		"ceiling as given (%s)", TD_TIMER_GET_FMT().c_str());
```

- [ ] **Step 3: Run the test to verify it fails**

Run (from `make/`): `ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | /usr/bin/grep -E 'TripletAutoTauTest'`
Expected: the build fails on `largestPiece` (no such member); after Step 4's header change alone it
fails at "walk with stragglers removed 5 pairs, kept 240".

- [ ] **Step 4: `SurvivorGraph::largestPiece`**

In `libs/SFM/ViewGraphTriplets.h`, after `numInPieces`:

```cpp
	unsigned largestPiece;      // images in the largest piece (0 when there is none)
```

In `libs/SFM/ViewGraphTriplets.cpp`, `EvaluateSurvivorGraph`: initialise `largestPiece` to 0 with
the other counters, and in the final loop that counts pieces (the one testing `component.second <
minPiece` and the unfiltered-largest-component membership), where a component qualifies as a piece,
add `result.largestPiece = MAXF(result.largestPiece, component.second);` beside the existing
`++result.numPieces; result.numInPieces += component.second;`. Every other early return that builds
a `SurvivorGraph` must leave `largestPiece` 0.

- [ ] **Step 5: The descent runs only on a shattered ceiling**

In `FilterPairsByTriplets`, after `const unsigned numStragglers = survivor.numNodes - survivor.numInPieces;`
add

```cpp
		// The descent repairs a ceiling that shattered the graph -- the small sets and every
		// exhaustively matched collection, where d_max/|V| is near 1 and the ceiling leaves
		// fragments of a few images each. A ceiling whose largest piece already holds a strict
		// majority of the images in pieces has done the paper's job: what hangs below it is a
		// straggler or the other face of a symmetric building (the church matched exhaustively
		// splits into its two facades at the ceiling, 143 and 85 images, and one 253-inlier pair
		// at 0.931 would join them), and nothing in the scores tells the two apart, so the
		// ceiling is applied as given and the smaller pieces stay apart. A strict majority, so a
		// graph cut into two equal halves is still repaired.
		const bool shattered = 2 * survivor.largestPiece <= survivor.numInPieces;
```

and change the descent's condition `if (survivor.largestComponent < minComponent) {` to
`if (shattered && survivor.largestComponent < minComponent) {`. Replace the VERBOSE line's case
expression `tau < ceiling ? "the strictest threshold that joins every piece" : "the ceiling applied as given"`
with

```cpp
			tau < ceiling ? "the strictest threshold that joins every piece" :
				numPieces > 1 && !shattered ? "the ceiling applied as given, its largest piece holding a majority" :
				"the ceiling applied as given",
```

Delete the comment sentence in that function which says the descent joins every piece without
qualification if any remains contradicted (the paragraph above the `unfiltered` line still describes
the descent; leave it, it is what happens when the descent runs).

- [ ] **Step 6: Help text and design note**

In `apps/CreateStructure/CreateStructure.cpp`, the `--triplet-auto-tau` help string: after "the
strictest threshold that joins every piece the ceiling leaves apart" add ", when no piece holds a
majority of the images (a ceiling whose largest piece does is applied as given)". Keep the rest.

In `docs/design/TripletDisambiguation.md`, the algorithm's threshold/selection item (the one that
says the threshold actually used is the strictest one whose survivor graph joins every piece), add
after "or the ceiling itself when it already does": ", or when its largest piece already holds a
majority of the images in pieces — then the ceiling is applied as given and the smaller pieces stay
apart: a ceiling that keeps most of the graph together has done its job, and what hangs below it
may be the other face of a symmetric building". Reflow the paragraph.

- [ ] **Step 7: Build, run the suite, mutate**

Run (from `make/`): `ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM && ./bin/Release/Tests 1`
Expected: every test PASSED, exit code 0, no FAILED line.

Then, one at a time, rebuild `Tests` only, run, confirm the named failure, revert, rebuild, confirm
green:

| mutation | expected failure |
|---|---|
| `shattered` defined as `2 * survivor.largestPiece < survivor.numInPieces` (strict) | pan scene: no descent (3 of 6 is no longer shattered), 11 removed instead of 10 |
| `shattered` replaced by `true` | walk scene: descent to 0.2, "removed 5 pairs, kept 240" |
| `largestPiece` never updated (stays 0) | walk scene: 0 ≤ 122 reads as shattered, same failure as above |

- [ ] **Step 8: Commit**

```bash
git add libs/SFM/ViewGraphTriplets.h libs/SFM/ViewGraphTriplets.cpp apps/Tests/TestsSFM.cpp apps/CreateStructure/CreateStructure.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: the triplet threshold descends only when the ceiling shattered the graph

The descent below the paper's threshold was written for graphs the ceiling shatters into
fragments of a few images, the small sets and every exhaustively matched collection, where
reassembling them is the whole job. On the church matched exhaustively the ceiling instead
leaves its two facades as pieces of 143 and 85 images, split as Doppelgangers++ splits them,
and one 253-inlier pair at 0.931 joined them. When the largest piece already holds a strict
majority of the images in pieces the ceiling has done its job and is applied as given; the
smaller pieces stay apart, like stragglers. Small sets and indoor are unchanged."
```

### Task 10: The reconstruction seeds in the largest piece the ceiling leaves

Spec §3.9. `StarInitializer::SelectReferenceView` (`libs/SFM/StarInitializer.cpp`) picks the image
whose valid pairs carry the most weighted inliers, and the star, then resection, grow from it.
That measure favours the densest cluster of look-alike views: on Radcliffe matched exhaustively
the seed lands in the 45-image piece (median 21,000 weighted inliers per image against 11,500 in
the 120-image piece), resection refuses the doppelganger bridges the descent let through, and the
model is 45+19+4 images while the 120-piece never registers; on the church the seed lands in the
85-piece and the 140-piece never registers. From now on the filter reports the images of the
largest piece the ceiling leaves, and the reference view is the heaviest of them. The descent is
unchanged.

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`SurvivorGraph`, `FilterPairsByTriplets`)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`EvaluateSurvivorGraph`, `FilterPairsByTriplets`, and one stale comment)
- Modify: `libs/SFM/StarInitializer.h`, `libs/SFM/StarInitializer.cpp` (`StarInitConfig`, `SelectReferenceView`, `Initialize`)
- Modify: `libs/SFM/Scene.h`, `libs/SFM/Scene.cpp` (`ExportMatchingCSVsAndFilterPairs`, `Reconstruct`, `ReconstructHierarchical`)
- Modify: `libs/SFM/PythonWrapper.cpp` (`pyReconstructHierarchical`)
- Modify: `apps/Tests/TestsSFM.cpp`, `apps/Tests/TestsSFM.h`, `apps/Tests/Tests.cpp` (a new `StarReferenceViewTest` registered like `TripletYieldTest`; two assertions added to `TripletAutoTauTest`)
- Modify: `docs/design/TripletDisambiguation.md` (the section "Where it runs, and the flags")

**Interfaces:**
- Consumes: `EvaluateSurvivorGraph(scene, scores, tau, minPiece)` and its union-find (`parent`, whose root is the smallest image index of its component, since a union always hangs the larger root under the smaller), `SurvivorGraph`, `FilterPairsByTriplets(scene, config, weightingCfg)` (Tasks 6-9); `StarInitializer::SelectReferenceView(const Scene&)`; `ImagePair::relativePose` (`std::optional<Pose3D>`), `ImagePair::HasValidWeight()`, `ImagePair::GetNumWeightedInliers()`; `Scene::ReconstructHierarchical(config)` and its `localToGlobals` (one `IIndexArr` per sub-scene, local index → global index; empty when the scene was not clustered); the test helpers `AddTripletImages(scene, n)`, `AddTripletPair(scene, a, b, numInliers)` (a pair they build has no relative pose and no weights) and `TripletKeptPairs(scene)`; `IIndexArr` is SEACAVE's `cList` (`push_back`, `back()`, `Empty()`, `begin()`/`end()`, `FOREACH`).
- Produces: `SurvivorGraph::largestPieceViews` (`IIndexArr`, ascending image indices of the largest piece, empty when there is none; ties between equally large pieces go to the one holding the lowest image index); `unsigned FilterPairsByTriplets(Scene&, const TripletFilterConfig&, const PairsWeightingConfig&, IIndexArr* pSeedViews = NULL)` — `*pSeedViews` receives the largest ceiling piece's images (emptied when the filter is off or leaves no piece); `StarInitConfig::seedViews` (`IIndexArr`, empty = every image); `IIndex StarInitializer::SelectReferenceView(const Scene&, const IIndexArr& seedViews)`; `bool Scene::ReconstructHierarchical(const ReconstructionConfig&, const IIndexArr& seedViews)`.

- [ ] **Step 1: Write the failing tests**

In `apps/Tests/TestsSFM.cpp`, immediately after `TripletYieldTest`, add:

```cpp
// The star initializer's reference view is chosen among the seed views the caller names -- the
// triplet filter's largest ceiling piece -- and only among every image when none is named, or
// none of the named has a valid pair. After the filter, the seed's side of a symmetric building
// becomes the model, and the heaviest image overall sits in the densest cluster of look-alike
// views, which is the worst place to start.
bool StarReferenceViewTest()
{
	TD_TIMER_START();
	// Ten images. A triangle (0-2) joined by 3000-inlier pairs: 6000 weighted inliers each, the
	// heaviest in the scene. A chain (3-8): 500-inlier consecutive pairs, 300-inlier pairs two
	// apart, and one 100-inlier pair (5,8), so that image 5 (1700) outweighs 6 (1600), 4 and 7
	// (1300), 8 (900) and 3 (800). Image 9 has no pair at all.
	Scene scene;
	AddTripletImages(scene, 10);
	const auto addPair = [&scene](IIndex a, IIndex b, unsigned numInliers) {
		AddTripletPair(scene, a, b, numInliers);
		ImagePair& pair = scene.pairs.back();
		pair.relativePose = Pose3D::Identity();
		pair.weightSpatial = 1.f;
		pair.weightConnectivity = 1.f;
	};
	addPair(0, 1, 3000);
	addPair(1, 2, 3000);
	addPair(0, 2, 3000);
	for (IIndex i = 3; i + 1 < 9; ++i)
		addPair(i, i + 1, 500);
	for (IIndex i = 3; i + 2 < 9; ++i)
		addPair(i, i + 2, 300);
	addPair(5, 8, 100);
	const IIndex refAll = StarInitializer::SelectReferenceView(scene, IIndexArr());
	if (refAll != 0) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with no seed views; expected 0, the heaviest image", refAll);
		return false;
	}
	IIndexArr chain;
	for (IIndex i = 3; i < 9; ++i)
		chain.push_back(i);
	const IIndex refChain = StarInitializer::SelectReferenceView(scene, chain);
	if (refChain != 5) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u among the chain's images; expected 5, the heaviest "
			"of them, not an image of the heavier triangle", refChain);
		return false;
	}
	IIndexArr lonely;
	lonely.push_back(9);
	const IIndex refLonely = StarInitializer::SelectReferenceView(scene, lonely);
	if (refLonely != 0) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with a seed view that has no valid pair; expected "
			"the fallback to every image, 0", refLonely);
		return false;
	}
	VERBOSE("StarReferenceViewTest PASSED: the reference view is the heaviest seed view, and the heaviest image "
		"when none is named or usable (%s)", TD_TIMER_GET_FMT().c_str());
	return true;
}
```

Declare it in `apps/Tests/TestsSFM.h` beside `TripletYieldTest` (same one-line style), and register it
in `apps/Tests/Tests.cpp` immediately after `TripletYieldTest`, in exactly the way `TripletYieldTest`
is registered there (same macro or call, same suite).

In `TripletAutoTauTest`, the walk scene's filter call currently reads
`const unsigned walkRemoved = FilterPairsByTriplets(walk, walkCfg, weightingCfg);`. Replace it with:

```cpp
	IIndexArr walkSeeds;
	const unsigned walkRemoved = FilterPairsByTriplets(walk, walkCfg, weightingCfg, &walkSeeds);
```

and, after the block that reports `walkRight`, add:

```cpp
	// the largest piece the ceiling leaves is the walk itself, images 0-119, in order: the
	// reconstruction seeds there, not in the 2-piece or at a straggler
	bool walkSeedsRight = walkSeeds.size() == 120;
	FOREACH(i, walkSeeds)
		walkSeedsRight = walkSeedsRight && walkSeeds[i] == (IIndex)i;
	if (!walkSeedsRight) {
		VERBOSE("TripletAutoTauTest FAILED: the walk's seed views hold %u images (first %u, last %u); expected images "
			"0-119, the largest piece the ceiling leaves",
			(unsigned)walkSeeds.size(), walkSeeds.empty() ? NO_ID : walkSeeds.front(), walkSeeds.empty() ? NO_ID : walkSeeds.back());
		return false;
	}
```

The three-chains scene's filter call currently reads
`const unsigned chainsRemoved = FilterPairsByTriplets(chains, chainsCfg, weightingCfg);`. Replace it with:

```cpp
	IIndexArr chainsSeeds;
	const unsigned chainsRemoved = FilterPairsByTriplets(chains, chainsCfg, weightingCfg, &chainsSeeds);
```

and, after the block that reports `chainsRight`, add:

```cpp
	// three equal pieces at the ceiling: the seed views are the one holding the lowest image index,
	// the first chain (0-39), reported at the ceiling even though the descent then joins all three
	bool chainsSeedsRight = chainsSeeds.size() == 40;
	FOREACH(i, chainsSeeds)
		chainsSeedsRight = chainsSeedsRight && chainsSeeds[i] == (IIndex)i;
	if (!chainsSeedsRight) {
		VERBOSE("TripletAutoTauTest FAILED: the three chains' seed views hold %u images (first %u, last %u); expected "
			"images 0-39, the first of three equal pieces, taken at the ceiling before the descent",
			(unsigned)chainsSeeds.size(), chainsSeeds.empty() ? NO_ID : chainsSeeds.front(), chainsSeeds.empty() ? NO_ID : chainsSeeds.back());
		return false;
	}
```

Extend the test's closing PASSED message so it also says that the seed views are the largest ceiling piece.

- [ ] **Step 2: Run the build to verify the tests fail**

Run (from `make/`): `ninja -f build-Release.ninja Tests 2>&1 | tail -20`
Expected: compilation errors on `FilterPairsByTriplets`'s fourth argument and `SelectReferenceView`'s second — the red state; nothing in this step passes.

- [ ] **Step 3: The filter reports the largest ceiling piece**

In `libs/SFM/ViewGraphTriplets.h`, add to `SurvivorGraph`, after `largestPiece`:

```cpp
	IIndexArr largestPieceViews; // the images of the largest piece, ascending (empty when there is none);
	                            // between equally large pieces, the one holding the lowest image index
```

Change the declaration of `FilterPairsByTriplets` to

```cpp
unsigned SFM_API FilterPairsByTriplets(Scene& scene, const TripletFilterConfig& config,
	const PairsWeightingConfig& weightingCfg, IIndexArr* pSeedViews = NULL);
```

and append to its comment block: "`pSeedViews`, when given, receives the images of the largest piece
the ceiling leaves (§ the piece rule: at the ceiling, before any descent; ties to the piece holding
the lowest image index), ascending, and is emptied when the filter is off or the ceiling leaves no
piece: the reconstruction chooses its reference view among them (StarInitConfig::seedViews), since
after the filter the seed's side of a symmetric building is the model and the heaviest image
overall sits in the densest cluster of look-alike views." Write it as prose in the file's style, without the section sign.

In `libs/SFM/ViewGraphTriplets.cpp`, `EvaluateSurvivorGraph`: the aggregate initialiser
`SurvivorGraph result{0, 0, 0, 0, 0, 0, 0};` gains nothing (the array default-constructs; add a trailing
`{}` if the compiler asks for it). In the loop over `componentSize` that counts pieces, track the largest
piece's root:

```cpp
	uint32_t largestPieceRoot = NO_INDEX;
	for (const auto& component : componentSize) {
		if (component.second < minPiece)
			continue;
		// (existing island test and counters unchanged)
		...
		result.numInPieces += component.second;
		// the root of a component is its smallest image index (a union hangs the larger root under
		// the smaller), so the smaller root among equally large pieces is the piece holding the
		// lowest image index: deterministic, and the same tie-break the triplet components use
		if (component.second > result.largestPiece ||
			(component.second == result.largestPiece && component.first < largestPieceRoot)) {
			result.largestPiece = component.second;
			largestPieceRoot = component.first;
		}
	}
	if (largestPieceRoot != NO_INDEX)
		for (IIndex i = 0; i < numImages; ++i)
			if (isNode[i] && Find(parent, (uint32_t)i) == largestPieceRoot)
				result.largestPieceViews.push_back(i);
```

replacing the existing `result.largestPiece = MAXF(result.largestPiece, component.second);`.

In `FilterPairsByTriplets`, the ceiling's survivor graph must exist on both paths (with `autoTau` off
nothing is evaluated today). Hoist `unfiltered`, `minPiece` and the ceiling's survivor out of the
`if (config.autoTau)` block, keep the ceiling's copy, and report it:

```cpp
	const float ceiling = tripletScores.tau;
	float tau = ceiling;
	const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
	const unsigned minPiece = (unsigned)std::ceil(0.01 * (double)unfiltered.largestComponent);
	const SurvivorGraph atCeiling = EvaluateSurvivorGraph(scene, tripletScores.scores, ceiling, minPiece);
	// The reconstruction seeds in the largest piece the ceiling leaves, whatever the descent joins
	// to it afterwards: the resection refuses the doppelganger bridges the descent lets through but
	// cannot choose the side it starts on, and the heaviest image overall sits in the densest
	// cluster of look-alike views (Radcliffe matched exhaustively: the 45-image piece, while the
	// 120-image piece never registered).
	if (pSeedViews)
		*pSeedViews = atCeiling.largestPieceViews;
	if (config.autoTau) {
		// (the existing comment)
		SurvivorGraph survivor = atCeiling;
		const unsigned minComponent = survivor.numInPieces;
		...
```

with the rest of the block as it is (the descent still reassigns `survivor`). Move the three `const`
locals it used (`minComponent`, `numPieces`, `numStragglers`) and the VERBOSE line unchanged. When the
filter is disabled (`!config.enabled`, the early `return 0`), empty `*pSeedViews` first if it was given.
Extend the final `VERBOSE("Triplet filter: kept %u/%u scene pairs ...")` line with
`; the reconstruction seeds in the largest piece the ceiling leaves (%u images)` fed
`atCeiling.largestPiece`.

Correct the stale numbers in the comment above `shattered` (the church sentence): it reads "143 and 85
images, and one 253-inlier pair at 0.931"; the run's own scores give "140 and 85 images, and one
253-inlier pair at 0.892". Change only those numbers.

- [ ] **Step 4: The star initializer chooses among the seed views**

In `libs/SFM/StarInitializer.h`, add to `StarInitConfig`, after `globalRotations`:

```cpp
	// The images the reference view is chosen among; empty, every image. The triplet filter fills
	// it with the largest piece its ceiling leaves (ViewGraphTriplets.h).
	IIndexArr seedViews;
```

and change the declaration and comment of `SelectReferenceView` to

```cpp
	/**
	 * @brief Select the reference view: the image whose valid pairs carry the most weighted
	 * inliers, among the seed views when any of them has a valid pair, else among every image
	 * @param scene Scene with image pairs
	 * @param seedViews Candidate images (empty: every image)
	 * @return Image ID of reference view
	 */
	static IIndex SelectReferenceView(const Scene& scene, const IIndexArr& seedViews);
```

In `libs/SFM/StarInitializer.cpp`, replace the body of `SelectReferenceView` with:

```cpp
	// weighted inliers per view over its valid pairs, dense supplement discounted and included: a
	// dense-only pair is a real connection of both its images, and the star initializer must see
	// the same graph the weights that let it through were computed on
	UnsignedArr degree(scene.images.size());
	degree.Memset(0);
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		degree[pair.ID1] += pair.GetNumWeightedInliers();
		degree[pair.ID2] += pair.GetNumWeightedInliers();
	}
	// the heaviest candidate: the seed views when any of them has a valid pair, else every image
	const auto Heaviest = [&degree](const auto& views, IIndex& bestView) {
		unsigned maxDegree = 0;
		for (IIndex i : views) {
			if (degree[i] > maxDegree) {
				maxDegree = degree[i];
				bestView = i;
			}
		}
		return maxDegree;
	};
	IIndex bestView = NO_ID;
	unsigned maxDegree = Heaviest(seedViews, bestView);
	if (maxDegree > 0) {
		VERBOSE("Selected reference view %u with %u connections among %u seed views", bestView, maxDegree, seedViews.size());
		return bestView;
	}
	if (!seedViews.empty())
		VERBOSE("warning: none of the %u seed views has a valid pair, choosing the reference view among every image", seedViews.size());
	std::vector<IIndex> every(scene.images.size());
	std::iota(every.begin(), every.end(), IIndex(0));
	maxDegree = Heaviest(every, bestView);
	if (bestView == NO_ID) {
		VERBOSE("error: no valid reference view found");
		return NO_ID;
	}
	VERBOSE("Selected reference view %u with %u connections", bestView, maxDegree);
	return bestView;
```

(`#include <numeric>` for `std::iota` if the file does not already have it.) In `Initialize`, change
the call to `SelectReferenceView(scene, config.seedViews)`.

- [ ] **Step 5: The seed views travel from the filter to the star**

In `libs/SFM/Scene.cpp`, change the helper `ExportMatchingCSVsAndFilterPairs` to return the seed views:

```cpp
// Export the matching diagnostics of a freshly matched scene, then disambiguate its view graph
// with the camera-triplet filter (ViewGraphTriplets.h, off unless the caller enables it). The
// order is deliberate: the CSVs describe the whole matched graph and carry the triplet score of
// every pair, including the pairs the filter is about to remove, so a run can be re-scored and
// re-thresholded offline from its own export alone. Returns the images the filter's ceiling
// vouches for most (its largest piece), the reconstruction's seed views; empty without the filter.
IIndexArr ExportMatchingCSVsAndFilterPairs(Scene& scene, const ReconstructionConfig& config)
{
	ExportMatchingCSVs(scene, config);
	IIndexArr seedViews;
	FilterPairsByTriplets(scene, config.tripletFilterCfg, config.matchCfg.weightingCfg, &seedViews);
	return seedViews;
}
```

Add beside it, in the same anonymous namespace:

```cpp
// The seed views of one sub-scene: the images of `seedViews` (indices of the whole scene) it holds,
// in its own local indices. An unclustered scene is its own single sub-scene and the indices coincide.
IIndexArr SubSceneSeedViews(const IIndexArr& seedViews, const std::vector<IIndexArr>& localToGlobals, IIndex idxSubScene)
{
	if (localToGlobals.empty())
		return seedViews;
	std::unordered_set<IIndex> isSeed(seedViews.begin(), seedViews.end());
	const IIndexArr& localToGlobal = localToGlobals[idxSubScene];
	IIndexArr local;
	FOREACH(l, localToGlobal)
		if (isSeed.count(localToGlobal[l]))
			local.push_back((IIndex)l);
	return local;
}
```

In `Scene::Reconstruct`: declare `IIndexArr seedViews;` just before the `#if 1` that starts the
feature/matching block, so both preprocessor branches see it; at the second call site (after
`MatchPairs`) write `seedViews = ExportMatchingCSVsAndFilterPairs(*this, config);` (the first call
site, the already-matched early return, ignores the return as today); and call
`ReconstructHierarchical(config, seedViews)`.

Change `ReconstructHierarchical` in `libs/SFM/Scene.h` and `libs/SFM/Scene.cpp` to
`bool ReconstructHierarchical(const ReconstructionConfig& config, const IIndexArr& seedViews);`
(comment: "seedViews: the images the star initializer chooses its reference view among, in this
scene's indices; empty, every image"). In its sub-scene loop, after `initCfg.baConfig = config.baConfig;`
add `initCfg.seedViews = SubSceneSeedViews(seedViews, localToGlobals, i);`.

In `libs/SFM/PythonWrapper.cpp`, `pyReconstructHierarchical` calls `ReconstructHierarchical(config, IIndexArr())`.

- [ ] **Step 6: Run the tests to verify they pass**

Run (from `make/`): `ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM && ./bin/Release/Tests 1`
Expected: `StarReferenceViewTest PASSED`, `TripletAutoTauTest PASSED`, every other test PASSED, exit
code 0, no FAILED line and no compiler warning from the changed files. `TripletStarInitTest` and the
other star-initializer tests run with empty seed views and must be unchanged.

- [ ] **Step 7: Mutate**

One at a time, rebuild `Tests` only, run, confirm the named failure, revert, rebuild, confirm green:

| mutation | expected failure |
|---|---|
| `SelectReferenceView` ignores `seedViews` (calls `Heaviest(every, ...)` first) | `StarReferenceViewTest FAILED: reference view 0 among the chain's images; expected 5` |
| the seed views taken from the descended `survivor` instead of `atCeiling` | `TripletAutoTauTest FAILED: the three chains' seed views hold 120 images` |
| the tie-break reversed (`component.first > largestPieceRoot`) | `TripletAutoTauTest FAILED: the three chains' seed views hold 40 images (first 80, last 119)` |

- [ ] **Step 8: Design note**

In `docs/design/TripletDisambiguation.md`, section "Where it runs, and the flags", after the paragraph
ending "the log line says so.", add:

"The reconstruction that follows seeds in the largest piece the ceiling leaves: the filter reports
that piece's images, and `StarInitializer::SelectReferenceView` takes the heaviest of them rather
than the heaviest image overall, which sits in the densest cluster of look-alike views. The descent
still joins the pieces; the resection then crosses the true bridges and refuses the doppelganger
ones, and starting on the right side is what lets it register the larger face rather than the
denser one (Radcliffe matched exhaustively: the 120-image piece rather than the 45-image one)."

- [ ] **Step 9: Commit**

```bash
git add libs/SFM/ViewGraphTriplets.h libs/SFM/ViewGraphTriplets.cpp libs/SFM/StarInitializer.h libs/SFM/StarInitializer.cpp libs/SFM/Scene.h libs/SFM/Scene.cpp libs/SFM/PythonWrapper.cpp apps/Tests/TestsSFM.cpp apps/Tests/TestsSFM.h apps/Tests/Tests.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: the reconstruction seeds in the largest piece the triplet ceiling leaves

The star initializer's reference view was the image whose pairs carry the most weighted
inliers, which after the triplet filter sits in the densest cluster of look-alike views: on
Radcliffe matched exhaustively the model grew from a 45-image piece and the 120-image piece
never registered, and on the church from the 85-image piece while the 140-image one never
did. The filter now reports the images of the largest piece its ceiling leaves and the
reference view is the heaviest of them; every image remains a candidate without the filter,
or when none of the reported images has a valid pair. The descent is unchanged: the
resection crosses the true bridges and refuses the doppelganger ones from the right side."
```

### Task 11: The reference view needs enough pairs to centre a star

Spec §3.9, "The star must be able to grow". `StarInitializer::Initialize` builds a star with one arm
per valid pair of the reference view and refuses a star of fewer than `minViews - 1` arms. The
reference view chosen among the seed views is the heaviest of them, and on a sparse kept graph the
heaviest can have too few pairs: on Street matched exhaustively the filter keeps 20 of 171 pairs,
the largest ceiling piece holds five images, its heaviest has two pairs, the initializer reported
"insufficient initial views (2 < 3)" and the run reconstructed nothing. From now on a candidate
needs at least `minPairs` valid pairs; the candidate sets and their order are unchanged.

**Files:**
- Modify: `libs/SFM/StarInitializer.h` (`SelectReferenceView` declaration and comment)
- Modify: `libs/SFM/StarInitializer.cpp` (`SelectReferenceView`, the call in `Initialize`)
- Modify: `apps/Tests/TestsSFM.cpp` (`StarReferenceViewTest`)
- Modify: `docs/design/TripletDisambiguation.md` (one clause in the seed paragraph)

**Interfaces:**
- Consumes: `StarInitializer::SelectReferenceView(const Scene&, const IIndexArr& seedViews)` and `StarInitConfig::seedViews` / `minViews` (Task 10); `StarReferenceViewTest`'s ten-image scene (Task 10): triangle 0-2 with 3000-inlier pairs (two pairs per image), chain 3-8 with 500-inlier consecutive pairs, 300-inlier pairs two apart and the 100-inlier pair (5,8), image 9 without a pair — so the valid pairs per image are 0,1,2: 2; 3: 2; 4: 3; 5: 5; 6: 4; 7: 3; 8: 3; 9: 0, and the weighted inliers 6000, 6000, 6000, 800, 1300, 1700, 1600, 1300, 900, 0.
- Produces: `IIndex StarInitializer::SelectReferenceView(const Scene&, const IIndexArr& seedViews, unsigned minPairs)`.

- [ ] **Step 1: Write the failing test**

In `apps/Tests/TestsSFM.cpp`, `StarReferenceViewTest`: every existing call gains a third argument, `2`
(the triangle's images have two pairs, so nothing else changes), and two checks are added before the
PASSED line. The three existing calls become

```cpp
	const IIndex refAll = StarInitializer::SelectReferenceView(scene, IIndexArr(), 2);
	...
	const IIndex refChain = StarInitializer::SelectReferenceView(scene, chain, 2);
	...
	const IIndex refLonely = StarInitializer::SelectReferenceView(scene, lonely, 2);
```

with their expectations (0, 5, 0) and messages unchanged. After the `refLonely` check add:

```cpp
	// A seed view with two pairs cannot centre a star of three arms: image 3 is skipped, and so are
	// the triangle's images (two pairs each) when the choice falls to every image; the reference is
	// the heaviest image with three or more pairs, 5. This is the Street failure: the largest ceiling
	// piece's heaviest image had two pairs and the star initializer refused it.
	IIndexArr three;
	three.push_back(3);
	const IIndex refThree = StarInitializer::SelectReferenceView(scene, three, 3);
	if (refThree != 5) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with a seed view of two pairs and a star of three arms "
			"required; expected 5, the heaviest image with three or more pairs", refThree);
		return false;
	}
	// No image has six pairs: the heaviest image is returned and the initializer reports the shortfall.
	const IIndex refSix = StarInitializer::SelectReferenceView(scene, IIndexArr(), 6);
	if (refSix != 0) {
		VERBOSE("StarReferenceViewTest FAILED: reference view %u with six arms required and no image having them; "
			"expected 0, the heaviest image", refSix);
		return false;
	}
```

Change the PASSED message to `"StarReferenceViewTest PASSED: the reference view is the heaviest seed view
with enough pairs to centre a star, and the heaviest such image when none is named or usable (%s)"`, and
extend the test's leading comment with one sentence: "A candidate also needs enough valid pairs to
centre a star, since the initializer refuses a star smaller than its minimum."

- [ ] **Step 2: Run the build to verify the test fails**

Run (from `make/`): `ninja -f build-Release.ninja Tests 2>&1 | tail -5`
Expected: a compilation error on `SelectReferenceView`'s third argument — the red state.

- [ ] **Step 3: The reference view needs enough pairs**

In `libs/SFM/StarInitializer.h`, change the declaration and its comment to

```cpp
	/**
	 * @brief Select the reference view: the image whose valid pairs carry the most weighted
	 * inliers, among the seed views when any of them qualifies, else among every image. A
	 * candidate qualifies with at least minPairs valid pairs, the smallest star the caller
	 * accepts less its centre; when no image qualifies the heaviest image is returned, so that
	 * the caller reports the shortfall.
	 * @param scene Scene with image pairs
	 * @param seedViews Candidate images (empty: every image)
	 * @param minPairs Valid pairs a candidate needs (StarInitConfig::minViews - 1)
	 * @return Image ID of reference view
	 */
	static IIndex SelectReferenceView(const Scene& scene, const IIndexArr& seedViews, unsigned minPairs);
```

In `libs/SFM/StarInitializer.cpp`, replace `SelectReferenceView` with:

```cpp
IIndex StarInitializer::SelectReferenceView(const Scene& scene, const IIndexArr& seedViews, unsigned minPairs)
{
	// weighted inliers and valid pairs per view, dense supplement discounted and included: a
	// dense-only pair is a real connection of both its images, and the star initializer must see
	// the same graph the weights that let it through were computed on
	UnsignedArr degree(scene.images.size()), numPairs(scene.images.size());
	degree.Memset(0);
	numPairs.Memset(0);
	for (const ImagePair& pair : scene.pairs) {
		if (!pair.relativePose.has_value() || !pair.HasValidWeight())
			continue;
		degree[pair.ID1] += pair.GetNumWeightedInliers();
		degree[pair.ID2] += pair.GetNumWeightedInliers();
		++numPairs[pair.ID1];
		++numPairs[pair.ID2];
	}
	// the heaviest of the given views with at least minArms valid pairs: the star built around the
	// reference view has one arm per valid pair and the initializer refuses a star smaller than
	// its minimum, so a heavier view with fewer arms is no use -- on a sparse kept graph (a chain
	// of 19 images and 20 pairs) the heaviest seed view had two
	const auto Heaviest = [&degree, &numPairs](const auto& views, unsigned minArms, IIndex& bestView) {
		unsigned maxDegree = 0;
		for (IIndex i : views) {
			if (numPairs[i] >= minArms && degree[i] > maxDegree) {
				maxDegree = degree[i];
				bestView = i;
			}
		}
		return maxDegree;
	};
	const unsigned minArms = MAXF(minPairs, 1u);
	IIndex bestView = NO_ID;
	unsigned maxDegree = Heaviest(seedViews, minArms, bestView);
	if (maxDegree > 0) {
		VERBOSE("Selected reference view %u with %u connections over %u pairs among %u seed views",
			bestView, maxDegree, numPairs[bestView], seedViews.size());
		return bestView;
	}
	if (!seedViews.empty())
		VERBOSE("warning: none of the %u seed views has %u valid pairs, choosing the reference view among every image",
			seedViews.size(), minArms);
	std::vector<IIndex> every(scene.images.size());
	std::iota(every.begin(), every.end(), IIndex(0));
	maxDegree = Heaviest(every, minArms, bestView);
	if (maxDegree == 0) // no image has enough pairs: the heaviest image, and the caller reports the shortfall
		maxDegree = Heaviest(every, 1u, bestView);
	if (bestView == NO_ID) {
		VERBOSE("error: no valid reference view found");
		return NO_ID;
	}
	VERBOSE("Selected reference view %u with %u connections over %u pairs", bestView, maxDegree, numPairs[bestView]);
	return bestView;
}
```

In `Initialize`, the call becomes
`SelectReferenceView(scene, config.seedViews, config.minViews > 0 ? config.minViews - 1 : 0)`.

- [ ] **Step 4: Run the tests to verify they pass**

Run (from `make/`): `ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM && ./bin/Release/Tests 1`
Expected: `StarReferenceViewTest PASSED`, every other test PASSED, exit code 0, no FAILED line, no
compiler warning from the changed files.

- [ ] **Step 5: Mutate**

One at a time, rebuild `Tests` only, run, confirm the named failure, revert, rebuild, confirm green:

| mutation | expected failure |
|---|---|
| the arm test dropped (`numPairs[i] >= minArms &&` removed) | `StarReferenceViewTest FAILED: reference view 3 with a seed view of two pairs and a star of three arms required; expected 5` |
| the last resort dropped (the `if (maxDegree == 0)` line and its call removed) | `StarReferenceViewTest FAILED: reference view 4294967295 with six arms required and no image having them; expected 0` |

- [ ] **Step 6: Design note**

In `docs/design/TripletDisambiguation.md`, in the paragraph beginning "The reconstruction that
follows seeds in the largest piece the ceiling leaves", change "takes the heaviest of them rather than
the heaviest image overall" to "takes the heaviest of them that has enough pairs to centre a star
(three, the smallest star it accepts less its centre) rather than the heaviest image overall".

- [ ] **Step 7: Commit**

```bash
git add libs/SFM/StarInitializer.h libs/SFM/StarInitializer.cpp apps/Tests/TestsSFM.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: the reference view needs enough pairs to centre a star

The star initializer refuses a star of fewer arms than its minimum, and the reference view
chosen among the seed views was the heaviest of them regardless of its pairs: on Street matched
exhaustively the kept graph is a chain of 20 pairs, the largest ceiling piece's heaviest image
has two, and the run reconstructed nothing. A candidate now needs at least the minimum number
of arms; the candidate sets and their order are unchanged, and when no image qualifies the
heaviest is chosen and the initializer reports the shortfall as before."
```

### Task 12: The default minimum score is 0.75

Spec §3.10. `TripletFilterConfig::minScore`, the paper's `m`, defaults to 0.6. On the church matched
exhaustively the ceiling at 0.6 is 0.942, inside the band (0.89-0.96) where the north-south bridge
pairs score, so two of four matchings split the facades and two merge them. At 0.75 the ceiling is
0.964 and all four split with the south facade whole; the small sets, ToH, the indoor loop and
Radcliffe are unchanged because their descents reach the same threshold whatever `m` is. The
default moves to 0.75; the tests whose scenes were derived at 0.6 say so explicitly.

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`TripletFilterConfig::minScore` and its comment)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (the `--triplet-min-score` help text)
- Modify: `apps/Tests/TestsSFM.cpp` (`TripletAutoTauTest`'s `walkCfg` and `chainsCfg`, and any other config whose expectations depend on the default)
- Modify: `docs/design/TripletDisambiguation.md` (the `--triplet-min-score` row of the flags table)

**Interfaces:**
- Consumes: `TripletFilterConfig` (Task 1), the test scenes of `TripletAutoTauTest` (Tasks 5, 8, 9: every expected ceiling there is `0.6 * (1 - d_max/|V|) + d_max/|V|`).
- Produces: nothing new; `TripletFilterConfig().minScore == 0.75f`, which `CreateStructure` reads as the flag's default.

- [ ] **Step 1: Write the failing test**

In `apps/Tests/TestsSFM.cpp`, `TripletFilterTest`, there is a config declared `const TripletFilterConfig defaults;`
(near the check of `minYield`). Add, at that place, a check that the default minimum score is 0.75:

```cpp
	if (!ISEQUAL(defaults.minScore, 0.75f)) {
		VERBOSE("TripletFilterTest FAILED: default minimum score %g; expected 0.75, the middle of the band in which "
			"an exhaustively matched two-faced building splits at the ceiling whatever pairs the matcher verifies",
			defaults.minScore);
		return false;
	}
```

If `defaults` is declared in a different test than `TripletFilterTest`, put the check where it is declared and
name that test in the message instead.

- [ ] **Step 2: Run the test to verify it fails**

Run (from `make/`): `ninja -f build-Release.ninja Tests && ./bin/Release/Tests 1 2>&1 | /usr/bin/grep -E 'FAILED'`
Expected: `... FAILED: default minimum score 0.6; expected 0.75, ...`, and nothing else FAILED.

- [ ] **Step 3: The default and its comment**

In `libs/SFM/ViewGraphTriplets.h`, change `float minScore = 0.6f;` to `float minScore = 0.75f;` and replace the
comment above it with:

```cpp
	// The paper's minimum edge score m, in [0,1] (the domain this implementation enforces). With
	// autoTau this is the ceiling the threshold is derived from and never exceeds. The paper's
	// values are 0.6 generic/large-scale, 0.9 highly ambiguous, 0.3 medium/small ambiguous, for a
	// score without coverage or yield; 0.75 here is the middle of the band in which an exhaustively
	// matched two-faced building (the church, four matchings) splits into its faces at the ceiling
	// whatever north-south pairs the matcher happens to verify -- at 0.6 the ceiling (0.942) sits
	// among those pairs' scores and two matchings of four merge the faces. Sets whose ceiling
	// shatters the graph are untouched: the descent reaches the same threshold whatever m is.
```

- [ ] **Step 4: The tests derived at 0.6 say so**

In `TripletAutoTauTest`, `walkCfg` and `chainsCfg` are constructed without a minimum score, and the walk's
and the chains' expected ceilings (`0.616129`, `0.6f*(1.f-4.f/120.f)+4.f/120.f`) were derived at 0.6. Add to
each, right after `enabled = true`:

```cpp
	walkCfg.minScore = 0.6f; // the scene's ceiling and the counts below were derived at the paper's generic m
```

(and the same line for `chainsCfg`). Then run the suite (`./bin/Release/Tests 1`) and read every FAILED line:
any other test whose expectation moved is a config that relied on the old default — give it
`minScore = 0.6f` with the same comment, never change an expected value. `ComputeTripletScores` calls that
pass `0.6f` literally are unaffected.

- [ ] **Step 5: The help text and the design note**

In `apps/CreateStructure/CreateStructure.cpp`, the `--triplet-min-score` option's help text ends with the
paper's values; it keeps them and gains, at its end, ", 0.75 here (the default): the ceiling at which an
exhaustively matched two-faced building splits into its faces whatever pairs the matcher verifies". Keep the
sentence in the style of the neighbouring options' text.

In `docs/design/TripletDisambiguation.md`, the flags table's `--triplet-min-score` row: default `0.75`, and
its text gains "; 0.75 rather than the paper's 0.6 because the church matched exhaustively splits at the
ceiling in every matching at 0.75 and in half of them at 0.6".

- [ ] **Step 6: Run the tests to verify they pass**

Run (from `make/`): `ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM && ./bin/Release/Tests 1`
Expected: every test PASSED, exit code 0, no FAILED line, no compiler warning from the changed files.
Then `./bin/Release/CreateStructure --help 2>&1 | /usr/bin/grep -A2 'triplet-min-score'` shows the default
`0.75`.

- [ ] **Step 7: Mutate**

Revert `minScore` to `0.6f`, rebuild `Tests`, run: expected `... FAILED: default minimum score 0.6; expected 0.75`.
Restore 0.75, rebuild, confirm green.

- [ ] **Step 8: Commit**

```bash
git add libs/SFM/ViewGraphTriplets.h apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: the triplet filter's default minimum score is 0.75

At the paper's generic 0.6 the ceiling on an exhaustively matched two-faced building sits among
the scores of the pairs that bridge its faces: on the church, four matchings give two graphs that
split at the ceiling and two that merge, and the model is one facade or both folded by the
matcher's luck. At 0.75 all four split with the south facade whole; the sets whose ceiling shatters
the graph reach the same threshold by the descent whatever the minimum is."
```

### Task 13: The descent joins the pieces themselves, a ceiling leaving no piece still descends, a zero ray angle is no measurement

Spec §3.6 ("A ray angle of zero is never measured") and §3.7 ("The bar is the pieces, not a count";
"A ceiling that leaves no piece"). Three defects the whole-branch reading found, each with the test
that pins it, plus four stale texts in the files this task touches anyway.

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`SurvivorGraph`, `EvaluateSurvivorGraph`'s declaration and comment)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`ComputeEdgeYields`'s guard and comment; `EvaluateSurvivorGraph`; `FilterPairsByTriplets`'s descent)
- Modify: `libs/SFM/Scene.cpp` (`#include <unordered_set>`)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (`--triplet-min-score` help: `[0,1]`)
- Modify: `apps/Tests/TestsSFM.cpp` (`TripletAutoTauTest`: two new scenes and a disabled-filter check; one stale comment), `apps/Tests/TestsSFM.h` (the `TripletAutoTauTest` comment)

**Interfaces:**
- Consumes: `SurvivorGraph` (Tasks 5, 8, 9, 10: `numNodes, largestComponent, numLowDegree, numKept, numPieces, numInPieces, largestPiece, largestPieceViews`), `EvaluateSurvivorGraph(scene, scores, tau, minPiece = 1)` whose union-find hangs the larger root under the smaller so a component's root is its smallest image index, `FilterPairsByTriplets`'s descent (Tasks 8-10), the test helpers `AddTripletImages`, `AddTripletPair(scene, a, b, numInliers)` and `TripletKeptPairs`, and `TripletAutoTauTest`'s existing scenes (its configs pin `minScore = 0.6f`, and every ceiling below is `0.6 (1 - d_max/|V|) + d_max/|V|`).
- Produces: `SurvivorGraph::pieceRoots` (`IIndexArr`, the smallest image index of every piece, ascending); `SurvivorGraph::viewsJoined` (`bool`); `EvaluateSurvivorGraph(const Scene&, const std::vector<float>& scores, float tau, unsigned minPiece = 1, const IIndexArr* views = NULL)`.

- [ ] **Step 1: Write the failing tests**

In `apps/Tests/TestsSFM.cpp`, `TripletAutoTauTest`, before the closing PASSED line, add three checks.

**The straggler flood** (the bar is the pieces, not a count). 310 images: chain A is images 0-99 and
chain B images 100-199, each with 1000-inlier consecutive pairs and 600-inlier pairs two apart;
110 stragglers, images 200-309, each hung on one consecutive pair of chain A by a 500-inlier pair to
its first image and a 400-inlier pair to its second (straggler `s` hangs on `(a, a+1)` with
`a = (s - 200) % 98`); one bridge, `(99, 100)` with 300 inliers, given its triangle by `(98, 100)` with
200. Scores: consecutive 1.0, two apart 0.6, the stragglers' 0.5 and 0.4, the bridge 0.3 and its helper
0.2; chain B's own edges lie in a second triplet component and are unscored, so the scored graph
`G_LCT` holds 211 nodes (chain A, the stragglers and image 100) and its `d_max` is 8 (images 2-11 of
chain A carry four straggler edges beside their four chain edges): the ceiling is
`0.6 (1 - 8/211) + 8/211 = 0.615166`. The unfiltered graph is one component of 310 images (the bridge
joins chain B), so the floor is `ceil(3.1) = 4`. At the ceiling the pieces are
chain A (100, its consecutive edges) and chain B (100, unscored edges), 200 images between them,
neither a majority; the 110 stragglers are apart. A count of nodes is met at 0.5, where the stragglers
join chain A (210 of 200) with chain B still apart; the pieces share a root only at 0.3. Expected: the
threshold 0.3, one pair removed (`(98,100)`), 615 kept of 616, the bridge `(99,100)` kept, and the seed
views chain A (images 0-99).

```cpp
	// The straggler flood: a count of nodes is met by stragglers accreting onto one piece while
	// the other is still apart; the bar is that the pieces share a component.
	Scene flood;
	AddTripletImages(flood, 310);
	for (IIndex i = 0; i + 1 < 100; ++i)
		AddTripletPair(flood, i, i + 1, 1000);
	for (IIndex i = 0; i + 2 < 100; ++i)
		AddTripletPair(flood, i, i + 2, 600);
	for (IIndex i = 100; i + 1 < 200; ++i)
		AddTripletPair(flood, i, i + 1, 1000);
	for (IIndex i = 100; i + 2 < 200; ++i)
		AddTripletPair(flood, i, i + 2, 600);
	for (IIndex s = 200; s < 310; ++s) {
		const IIndex a = (s - 200) % 98;
		AddTripletPair(flood, a, s, 500);
		AddTripletPair(flood, a + 1, s, 400);
	}
	AddTripletPair(flood, 99, 100, 300);
	AddTripletPair(flood, 98, 100, 200);
	const TripletScores floodScores = ComputeTripletScores(flood, 0.6f, 0.f, weightingCfg.gridSize);
	const SurvivorGraph floodCeiling = EvaluateSurvivorGraph(flood, floodScores.scores, floodScores.tau, 4);
	if (!ISEQUAL(floodScores.tau, 0.6f*(1.f-8.f/211.f)+8.f/211.f) || floodCeiling.numPieces != 2 ||
		floodCeiling.numInPieces != 200 || floodCeiling.largestPiece != 100 ||
		floodCeiling.pieceRoots.size() != 2 || floodCeiling.pieceRoots[0] != 0 || floodCeiling.pieceRoots[1] != 100) {
		VERBOSE("TripletAutoTauTest FAILED: straggler flood at the ceiling %g: %u pieces holding %u, the largest %u, roots %u; "
			"expected 2 pieces of 100 holding 200 with roots 0 and 100",
			floodScores.tau, floodCeiling.numPieces, floodCeiling.numInPieces, floodCeiling.largestPiece,
			(unsigned)floodCeiling.pieceRoots.size());
		return false;
	}
	const SurvivorGraph floodAtHalf = EvaluateSurvivorGraph(flood, floodScores.scores, 0.5f, 1, &floodCeiling.pieceRoots);
	const SurvivorGraph floodAtBridge = EvaluateSurvivorGraph(flood, floodScores.scores, 0.3f, 1, &floodCeiling.pieceRoots);
	if (floodAtHalf.largestComponent != 210 || floodAtHalf.viewsJoined || !floodAtBridge.viewsJoined) {
		VERBOSE("TripletAutoTauTest FAILED: straggler flood at 0.5: component %u, pieces joined %s; at 0.3 joined %s; "
			"expected 210 and not joined, then joined",
			floodAtHalf.largestComponent, floodAtHalf.viewsJoined ? "yes" : "no", floodAtBridge.viewsJoined ? "yes" : "no");
		return false;
	}
	TripletFilterConfig floodCfg;
	floodCfg.enabled = true;
	floodCfg.minYield = 0.f;
	floodCfg.minScore = 0.6f; // the scene's ceiling and the counts below were derived at the paper's generic m
	IIndexArr floodSeeds;
	const unsigned floodRemoved = FilterPairsByTriplets(flood, floodCfg, weightingCfg, &floodSeeds);
	const std::set<std::pair<IIndex,IIndex>> floodKept = TripletKeptPairs(flood);
	bool floodRight = floodRemoved == 1 && floodKept.size() == 615 && floodKept.count({99,100}) == 1 &&
		floodKept.count({98,100}) == 0 && floodSeeds.size() == 100;
	FOREACH(i, floodSeeds)
		floodRight = floodRight && floodSeeds[i] == (IIndex)i;
	if (!floodRight) {
		VERBOSE("TripletAutoTauTest FAILED: straggler flood removed %u pairs, kept %u, seed views %u; expected the descent to "
			"0.3 (the pieces share a root only at the bridge): 1 removed, 615 kept, the bridge kept, seed views 0-99",
			floodRemoved, (unsigned)floodKept.size(), (unsigned)floodSeeds.size());
		return false;
	}
```

**The chain of pairs** (a ceiling that leaves no piece). 210 images in one chain: consecutive pairs
`(i, i+1)` with 1000 inliers for even `i` and 100 for odd `i`, and 100-inlier pairs two apart. Every
1000-pair scores 1.0 and every 100-pair 0.1; `d_max` is 4, the ceiling `0.6 (1 - 4/210) + 4/210 =
0.607619`; the ceiling keeps only the 105 strong pairs, components of two images, below the floor
`ceil(2.1) = 3`: no piece. Under the rule every component of the unfiltered largest component is then a
piece: 105 pieces of two, none a majority, the descent's only candidate is 0.1 and nothing is removed;
the seed views are the piece holding the lowest image index, `{0, 1}`.

```cpp
	// The chain of pairs: the ceiling leaves only components below the floor; then every component
	// is a piece and the descent joins them all rather than leaving the graph as pairs.
	Scene pairsChain;
	AddTripletImages(pairsChain, 210);
	for (IIndex i = 0; i + 1 < 210; ++i)
		AddTripletPair(pairsChain, i, i + 1, i % 2 == 0 ? 1000 : 100);
	for (IIndex i = 0; i + 2 < 210; ++i)
		AddTripletPair(pairsChain, i, i + 2, 100);
	const TripletScores pairsChainScores = ComputeTripletScores(pairsChain, 0.6f, 0.f, weightingCfg.gridSize);
	const SurvivorGraph pairsChainCeiling = EvaluateSurvivorGraph(pairsChain, pairsChainScores.scores, pairsChainScores.tau, 3);
	if (!ISEQUAL(pairsChainScores.tau, 0.6f*(1.f-4.f/210.f)+4.f/210.f) || pairsChainCeiling.numPieces != 0 ||
		pairsChainCeiling.largestComponent != 2 || pairsChainCeiling.numKept != 105) {
		VERBOSE("TripletAutoTauTest FAILED: chain of pairs at the ceiling %g: %u pieces, largest component %u, %u kept; "
			"expected no piece of 3, components of 2, 105 kept",
			pairsChainScores.tau, pairsChainCeiling.numPieces, pairsChainCeiling.largestComponent, pairsChainCeiling.numKept);
		return false;
	}
	TripletFilterConfig pairsChainCfg;
	pairsChainCfg.enabled = true;
	pairsChainCfg.minYield = 0.f;
	pairsChainCfg.minScore = 0.6f; // the scene's ceiling and the counts below were derived at the paper's generic m
	IIndexArr pairsChainSeeds;
	const unsigned pairsChainRemoved = FilterPairsByTriplets(pairsChain, pairsChainCfg, weightingCfg, &pairsChainSeeds);
	if (pairsChainRemoved != 0 || pairsChain.pairs.size() != 417 || pairsChainSeeds.size() != 2 ||
		pairsChainSeeds[0] != 0 || pairsChainSeeds[1] != 1) {
		VERBOSE("TripletAutoTauTest FAILED: chain of pairs removed %u, %u pairs left, %u seed views; expected a ceiling "
			"leaving no piece to make every component a piece: 0 removed, 417 pairs, seed views {0, 1}",
			pairsChainRemoved, (unsigned)pairsChain.pairs.size(), (unsigned)pairsChainSeeds.size());
		return false;
	}
```

**The disabled filter clears the seed views.** Right after the chain of pairs:

```cpp
	// A disabled filter reports no seed views, whatever the caller's array held.
	TripletFilterConfig offCfg;
	offCfg.enabled = false;
	IIndexArr offSeeds;
	offSeeds.push_back(7);
	if (FilterPairsByTriplets(pairsChain, offCfg, weightingCfg, &offSeeds) != 0 || !offSeeds.empty()) {
		VERBOSE("TripletAutoTauTest FAILED: a disabled filter left %u seed views; expected none", (unsigned)offSeeds.size());
		return false;
	}
```

Extend the PASSED message so it also names the straggler flood and the chain of pairs, and add the two
scenes to the test's leading scene index (one line each, in its style: "the straggler flood pins that
the descent's bar is the pieces sharing a component, not a count of nodes; the chain of pairs pins that a
ceiling leaving no piece makes every component a piece").

- [ ] **Step 2: Run the build to verify the tests fail**

Run (from `make/`): `ninja -f build-Release.ninja Tests 2>&1 | tail -5`
Expected: compilation errors on `pieceRoots`, `viewsJoined` and `EvaluateSurvivorGraph`'s fifth argument — the red state.

- [ ] **Step 3: The survivor graph reports its piece roots and joins**

In `libs/SFM/ViewGraphTriplets.h`, add to `SurvivorGraph` after `largestPieceViews`:

```cpp
	IIndexArr pieceRoots;       // one image per piece, its smallest index, ascending
	bool viewsJoined;           // the images passed as `views` all lie in one component (true when none were passed)
```

and change the declaration to

```cpp
SurvivorGraph SFM_API EvaluateSurvivorGraph(const Scene& scene, const std::vector<float>& scores, float tau,
	unsigned minPiece = 1, const IIndexArr* views = NULL);
```

adding to its comment: "`views`, when given, are images whose joining the caller asks about — the
descent passes the pieces' roots of the ceiling's graph and reads `viewsJoined` at each candidate
threshold: joining every piece means the pieces share one component, not the largest component
reaching a count of nodes, which stragglers accreting onto one piece can satisfy with another piece
still apart."

In `libs/SFM/ViewGraphTriplets.cpp`, `EvaluateSurvivorGraph`: initialise the two new fields
(`viewsJoined` true); in the loop over `componentSize` that counts pieces, after `++result.numPieces`,
`result.pieceRoots.push_back((IIndex)component.first);` — the root is the component's smallest image
index since a union hangs the larger root under the smaller — and after that loop sort `pieceRoots`
ascending (`std::sort(result.pieceRoots.begin(), result.pieceRoots.end())`). Then, before returning:

```cpp
	if (views && !views->empty()) {
		const uint32_t root = Find(parent, (uint32_t)(*views)[0]);
		FOREACH(i, *views)
			if (Find(parent, (uint32_t)(*views)[i]) != root) {
				result.viewsJoined = false;
				break;
			}
	}
```

- [ ] **Step 4: The descent tests the pieces, and a ceiling leaving no piece descends**

In `FilterPairsByTriplets`, after `atCeiling` is computed:

```cpp
	// A ceiling that leaves no piece -- every component below the floor, a large collection
	// shattered into pairs -- is the graph that most needs repair, not one to leave alone: every
	// component of the unfiltered largest component is then a piece, as on a small set.
	if (atCeiling.numPieces == 0 && minPiece > 1) {
		minPiece = 1;
		atCeiling = EvaluateSurvivorGraph(scene, tripletScores.scores, ceiling, minPiece);
	}
```

(`minPiece` and `atCeiling` lose their `const` for this.) In the descent, replace the condition
`if (shattered && survivor.largestComponent < minComponent)` by `if (shattered && atCeiling.numPieces > 1)`,
and the binary search's test `EvaluateSurvivorGraph(scene, tripletScores.scores, candidates[mid]).largestComponent >= minComponent`
by `EvaluateSurvivorGraph(scene, tripletScores.scores, candidates[mid], 1, &atCeiling.pieceRoots).viewsJoined`.
Update the comment above the search: the largest component only grows as tau falls becomes "the pieces,
once joined, stay joined as tau falls"; the loosest candidate keeps every scored pair, whose graph joins
every piece (they all lie in the unfiltered largest component). `minComponent` stays as the VERBOSE's
count of images the pieces hold; if nothing else reads it, keep it for that alone.

- [ ] **Step 5: A zero ray angle is the never-measured sentinel**

In `ComputeEdgeYields`, the guard `if (!ISFINITE(edgeRayAngle[e]) || edgeRayAngle[e] < 0.f)` becomes
`if (!ISFINITE(edgeRayAngle[e]) || edgeRayAngle[e] <= 0.f)`, the sentence in the comment above the
function "An edge whose ray angle is not finite or is negative has no measurable geometry" becomes "An
edge whose ray angle is not finite, negative or zero -- zero being ImagePair::meanRayAngle's value on a
pair whose relative pose was never decomposed -- has no measurable geometry", and the closing comment
after the function says "no finite, positive ray angle" where it says "no finite, non-negative ray
angle". Add to `TripletYieldTest` (the test with the NaN-angle case) one check in the same style:
give the pair that case uses a `meanRayAngle` of exactly 0 instead of NaN, run `ComputeTripletScores`
with `minYield 0.4`, and expect the same doppelganger-triplet count and the same scores the NaN case
expects.

- [ ] **Step 6: The stale texts**

- `apps/Tests/TestsSFM.h`: the comment above `TripletAutoTauTest` (near lines 271-277) describes a
  low-degree bar, a ladder and a gapped ring the test no longer has; replace it with one sentence per
  scene the test holds (barbell, ring, bridge, pendant, baseline, boundary, pan, walk, three chains,
  straggler flood, chain of pairs), each saying what it pins, matching the scene index at the top of
  the test body.
- `apps/Tests/TestsSFM.cpp` near line 8222: the sentence mentioning "99 %" describes a rule the filter
  no longer has; reword it to the piece rule ("the strictest threshold joining every piece").
- `apps/CreateStructure/CreateStructure.cpp`: the `--triplet-min-score` help says "in (0,1)"; the code
  accepts `[0,1]` (`CLAMP` in `FilterPairsByTriplets`, the error message at the option check aside);
  make the help and the option's range check agree on `[0,1]`.
- `libs/SFM/Scene.cpp`: add `#include <unordered_set>` beside the other standard includes
  (`SubSceneSeedViews` uses it).

- [ ] **Step 7: Run the tests to verify they pass**

Run (from `make/`): `ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM && ./bin/Release/Tests 1`
Expected: `TripletAutoTauTest PASSED`, `TripletYieldTest PASSED`, every other test PASSED, exit code 0,
no compiler warning from the changed files. The walk, three-chains, pan and every earlier scene keep
their expected values: on them a count of nodes and the piece test agree.

- [ ] **Step 8: Mutate**

One at a time, rebuild `Tests` only, run, confirm the named failure, revert, rebuild, confirm green:

| mutation | expected failure |
|---|---|
| the search tests `largestComponent >= minComponent` again instead of `viewsJoined` | `TripletAutoTauTest FAILED: straggler flood removed 112 pairs, kept 504` |
| the no-piece fallback removed | `TripletAutoTauTest FAILED: chain of pairs removed 312` |
| the guard back to `< 0.f` | `TripletYieldTest FAILED` on the zero-angle check |

- [ ] **Step 9: Commit**

```bash
git add libs/SFM/ViewGraphTriplets.h libs/SFM/ViewGraphTriplets.cpp libs/SFM/Scene.cpp apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.cpp apps/Tests/TestsSFM.h
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: the descent joins the pieces themselves, and a ceiling leaving no piece still descends

The search below the ceiling stopped when the largest component held as many images as the
pieces, which stragglers accreting onto one piece can satisfy with another piece still apart;
it now stops when the pieces share a component. A ceiling leaving every component below the
floor skipped the descent and the seed views; every component is then a piece. A ray angle of
zero is the never-measured value, not a zero baseline, and no longer feeds the yield envelope."
```

### Task 14: The Python harness scores the rule that ships

`scripts/python/tests/triplet_disambiguation.py` is the independent NumPy reimplementation whose
`parity` subcommand is the loud consistency check between the C++ scores and the paper's algorithm. It
still scores from `NumMatches` alone — the paper's rule — while the C++ column it compares against is the
shipped one: strength discounted by coverage, doppelganger triangles giving no evidence, a zero ray angle
outside the envelope. `parity` therefore fails on every real graph. The script learns the shipped rule
from the export's own columns and is checked against two real exports.

**Files:**
- Modify: `scripts/python/tests/triplet_disambiguation.py`

**Interfaces:**
- Consumes: the pairs export of `CreateStructure --export-pairs-csv`, header
  `ImageA,ImageB,NumMatches,Coverage,Weight,WeightSpatial,WeightConnectivity,WeightTriplet,MeanRayAngle,TripletScore`,
  where `NumMatches` is the pair's filtered epipolar inlier count `n_ij`, `Coverage` the fraction of
  the grid the inliers cover `c_ij` (`ComputePairCoverage`), `MeanRayAngle` the pair's mean ray angle in
  **degrees** (0 when never measured), `TripletScore` the C++ score (empty = unscored); and the C++
  definition in `libs/SFM/ViewGraphTriplets.cpp`: `ComputeEdgeYields` (the envelope) and
  `ComputeTripletScores` (the triangles, `G_LCT`, the mean with doppelganger triangles contributing 0).
  Two real exports to check against, copied into the plan's workspace (not the repository):
  `.superpowers/sdd/2026-09-05-view-graph-disambiguation/exports/street-pairs.csv` (171 pairs) and
  `.../exports/radcliffe-pairs.csv` (23,021 pairs, 20,306 scored, 311 of them with a zero ray angle).
- Produces: the same three subcommands (`score`, `parity`, `roc`) with the shipped score; nothing else reads the script.

- [ ] **Step 1: Run parity to see it fail**

Run (from the worktree root):
`python3 scripts/python/tests/triplet_disambiguation.py parity --pairs .superpowers/sdd/2026-09-05-view-graph-disambiguation/exports/street-pairs.csv`
Expected: exits non-zero with a maximum difference far above 1e-5 — the red state.

- [ ] **Step 2: The shipped score**

Rewrite the scoring so that it is, in NumPy, what `ComputeTripletScores` does. Read that function and
`ComputeEdgeYields` in `libs/SFM/ViewGraphTriplets.cpp` in full first; the rule is:

1. An edge is a pair with `NumMatches > 0` and `Coverage > 0`; its strength is `s = NumMatches * Coverage`.
   Two rows describing the same image pair collapse onto one edge with the larger strength.
2. Triangles of that graph; the triplet graph's components (triangles sharing an edge); `G_LCT` is the
   component with the most triangles (ties: the component whose smallest edge index is smallest, where
   edges are indexed in the order the rows come, after the collapse); every edge outside `G_LCT`, or in
   no triangle, is unscored (NaN).
3. Yields over the edges of the whole graph: `u_e = NumMatches / min(K_i, K_j)` with `K_i` the largest
   `NumMatches` over image `i`'s edges; the envelope per 1-degree bin of `floor(MeanRayAngle)`, bins
   at and beyond 90 degrees sharing the last (index 89); a bin with at least five edges gets the value
   at rank `min(n - 1, floor(0.9 * n))` of its sorted `u` values; a suffix maximum from the highest bin
   downward makes the envelope non-increasing in the angle; then an empty bin below the lowest populated
   one takes the value of the bin below it (a forward fill from bin 1 upward, after the suffix maximum);
   if no bin is populated every yield is 1. An edge whose `MeanRayAngle` is not finite, negative **or
   zero** takes no part in any bin and keeps yield 1; every other edge's yield is `min(1, u_e / H(bin))`.
4. The score of an edge of `G_LCT` is the mean over the triangles of `G_LCT` containing it of
   `s_e / max strength in the triangle`, where a triangle whose three yields are all below `minYield`
   (0.4) contributes 0 to that sum and still counts in the mean's divisor. `tau = m (1 - d_max/|V|) +
   d_max/|V|` over `G_LCT`'s nodes and degrees.

Keep the script's structure (its `TripletScores` record, `read_csv_rows`, the subcommands) and its
docstring's shape, and rewrite the docstring's description of the rule to the above, in the same
register; delete the sentence about `NumMatches > 0` alone being the edge test. `score` gains
`--min-yield` (default 0.4) and writes `Coverage` beside `NumMatches`; `parity` and `roc` take the same
option. The default `-m` becomes 0.75 (the C++ default), and the `roc` subcommand's default `m` list stays
`0.3, 0.6, 0.9` with 0.75 added.

- [ ] **Step 3: Run parity on both exports**

```
python3 scripts/python/tests/triplet_disambiguation.py parity --pairs .superpowers/sdd/2026-09-05-view-graph-disambiguation/exports/street-pairs.csv
python3 scripts/python/tests/triplet_disambiguation.py parity --pairs .superpowers/sdd/2026-09-05-view-graph-disambiguation/exports/radcliffe-pairs.csv
```

Expected: both exit 0, identical unscored sets, maximum absolute difference at most 1e-5 (the C++ sums
in double and stores float). If Radcliffe disagrees only on pairs whose `MeanRayAngle` is 0, the export
predates the zero-angle rule: say so in the report with the count and the maximum difference among the
other pairs, and the controller will refresh the export. Record the two maximum differences in the
script's docstring, one sentence: "Checked against two exhaustive exports (Street, 171 pairs; Radcliffe
Camera, 23,021 pairs): maximum absolute difference X and Y."

- [ ] **Step 4: Mutate**

One at a time, run parity on Street, confirm it fails, revert: (a) the coverage discount dropped
(`s = NumMatches`); (b) the doppelganger triangles contributing their ratios instead of 0 (with
`--min-yield 0.4`); (c) the zero-angle edges binned at 0 degrees instead of kept out of the envelope
(this one on Radcliffe, where such edges exist).

- [ ] **Step 5: Commit**

```bash
git add scripts/python/tests/triplet_disambiguation.py
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "scripts: the triplet harness scores the rule that ships

The NumPy reimplementation scored the paper's rule, inlier counts alone, while the C++ column it
checks carries the shipped one: the strength discounted by coverage, a triangle whose three pairs
all yield below the graph's envelope giving no evidence, a zero ray angle outside the envelope.
It now reads the export's Coverage and MeanRayAngle columns and reproduces the column on two
exhaustive exports."
```

### Task 15: The ceiling looks for the second face

Spec §3.10 (the default `m` returns to 0.6) and §3.11. The default minimum score was moved to 0.75
because the church's facades split at that ceiling in every matching and merge at 0.6 in half of them;
Big Ben matched exhaustively then showed the other side: at 0.75 its one-piece graph is cut so thin that
the reconstruction keeps 147 of 403 images, at 0.6 it keeps 371. Two ceilings are now evaluated, and the
higher is used only when the graph it leaves has two faces: a majority piece and a second piece holding
at least a third of it.

**Files:**
- Modify: `libs/SFM/ViewGraphTriplets.h` (`TripletFilterConfig::minScore` back to `0.6f` and a new `secondFaceScore`; `SurvivorGraph::secondPiece`)
- Modify: `libs/SFM/ViewGraphTriplets.cpp` (`EvaluateSurvivorGraph`'s piece loop; `FilterPairsByTriplets`'s ceiling choice and its VERBOSE)
- Modify: `apps/CreateStructure/CreateStructure.cpp` (the `--triplet-min-score` help; a `--triplet-second-face-score` option)
- Modify: `apps/Tests/TestsSFM.cpp` (`TripletYieldTest`'s default check; two new scenes in `TripletAutoTauTest`)
- Modify: `docs/design/TripletDisambiguation.md` (the flags table)

**Interfaces:**
- Consumes: `TripletFilterConfig` (Tasks 1, 7, 12), `SurvivorGraph` and `EvaluateSurvivorGraph(scene, scores, tau, minPiece, views)` (Tasks 5, 8-10, 13), `FilterPairsByTriplets`'s ceiling/descent structure (Tasks 9, 10, 13: `degreeRatio`, `ceiling`, `unfiltered`, `minPiece`, `atCeiling`, the no-piece fallback, the majority test, the descent), the test helpers and `TripletAutoTauTest`'s scene index.
- Produces: `TripletFilterConfig::secondFaceScore` (`float`, 0.75); `SurvivorGraph::secondPiece` (`unsigned`, images in the second-largest piece, 0 when there is none); the option `--triplet-second-face-score`.

- [ ] **Step 1: Write the failing tests**

In `TripletYieldTest`, the check on `defaults.minScore` expects 0.75; make it expect `0.6f` with the message
"default minimum score %g; expected 0.6, the paper's generic value", and add beside it a check that
`defaults.secondFaceScore` is `0.75f` with the message "default second-face score %g; expected 0.75, the
ceiling that splits a two-faced building".

In `TripletAutoTauTest`, before the closing PASSED line, two scenes.

**The two faces.** 100 images: chain A is images 0-59 and chain B images 60-99, each with 1000-inlier
consecutive pairs and 600-inlier pairs two apart; three 700-inlier bridges `(59,60)`, `(58,60)`,
`(59,61)`, which sit in the triangles `(58,59,60)` and `(59,60,61)` and score 0.7. `d_max` is 4 and
`|V|` 100, so `tau(0.6) = 0.6 (1 - 4/100) + 4/100 = 0.616` and `tau(0.75) = 0.76`. At 0.616 the bridges
are kept and the graph is one piece of 100, a majority the ceiling would be applied on, merging the two
chains. At 0.76 the pieces are A (60) and B (40): a majority, and B holds two thirds of A — two faces —
so 0.76 is the ceiling and is applied as given: the 96 pairs two apart and the 3 bridges go, the 98
consecutive pairs stay (of 197), the seed views are 0-59.

```cpp
	// The two faces: at the paper's ceiling the bridges join the chains into one majority piece;
	// at the higher ceiling the chains are two pieces, the second holding two thirds of the first,
	// so the higher ceiling is the one used and the bridges go.
	Scene faces;
	AddTripletImages(faces, 100);
	for (IIndex i = 0; i + 1 < 60; ++i)
		AddTripletPair(faces, i, i + 1, 1000);
	for (IIndex i = 0; i + 2 < 60; ++i)
		AddTripletPair(faces, i, i + 2, 600);
	for (IIndex i = 60; i + 1 < 100; ++i)
		AddTripletPair(faces, i, i + 1, 1000);
	for (IIndex i = 60; i + 2 < 100; ++i)
		AddTripletPair(faces, i, i + 2, 600);
	AddTripletPair(faces, 59, 60, 700);
	AddTripletPair(faces, 58, 60, 700);
	AddTripletPair(faces, 59, 61, 700);
	const TripletScores facesScores = ComputeTripletScores(faces, 0.6f, 0.f, weightingCfg.gridSize);
	const SurvivorGraph facesLow = EvaluateSurvivorGraph(faces, facesScores.scores, facesScores.tau);
	const SurvivorGraph facesHigh = EvaluateSurvivorGraph(faces, facesScores.scores, 0.75f*(1.f-4.f/100.f)+4.f/100.f);
	if (!ISEQUAL(facesScores.tau, 0.6f*(1.f-4.f/100.f)+4.f/100.f) || facesLow.numPieces != 1 || facesLow.largestPiece != 100 ||
		facesHigh.numPieces != 2 || facesHigh.largestPiece != 60 || facesHigh.secondPiece != 40) {
		VERBOSE("TripletAutoTauTest FAILED: two faces at %g: %u pieces, largest %u; at 0.76: %u pieces, largest %u, second %u; "
			"expected one piece of 100, then two of 60 and 40",
			facesScores.tau, facesLow.numPieces, facesLow.largestPiece, facesHigh.numPieces, facesHigh.largestPiece, facesHigh.secondPiece);
		return false;
	}
	TripletFilterConfig facesCfg;
	facesCfg.enabled = true;
	facesCfg.minYield = 0.f;
	IIndexArr facesSeeds;
	const unsigned facesRemoved = FilterPairsByTriplets(faces, facesCfg, weightingCfg, &facesSeeds);
	const std::set<std::pair<IIndex,IIndex>> facesKept = TripletKeptPairs(faces);
	bool facesRight = facesRemoved == 99 && facesKept.size() == 98 && facesKept.count({59,60}) == 0 &&
		facesKept.count({58,59}) == 1 && facesKept.count({60,61}) == 1 && facesSeeds.size() == 60;
	FOREACH(i, facesSeeds)
		facesRight = facesRight && facesSeeds[i] == (IIndex)i;
	if (!facesRight) {
		VERBOSE("TripletAutoTauTest FAILED: two faces removed %u pairs, kept %u, seed views %u; expected the higher ceiling "
			"(a majority piece with a second piece of two thirds): 99 removed, 98 kept, the bridges gone, seed views 0-59",
			facesRemoved, (unsigned)facesKept.size(), (unsigned)facesSeeds.size());
		return false;
	}
```

**The face and its cluster.** The same, with chain B only images 60-74 (15 images): `|V|` 75,
`tau(0.6) = 0.6 (1 - 4/75) + 4/75 = 0.621333`, `tau(0.75) = 0.763333`. At 0.763 the pieces are A (60)
and B (15): a majority, but 15 is less than a third of 60 — a cluster, not a face — so the paper's
ceiling 0.621 is the one used: the bridges (0.7) stay, the graph is one piece of 75, applied as given:
the 71 pairs two apart go, the 73 consecutive pairs and the 3 bridges stay (of 147), the seed views are
0-74.

```cpp
	// The face and its cluster: the higher ceiling leaves a majority piece with a second piece of
	// a quarter of it -- a cluster hanging off the building, not its other face -- so the paper's
	// ceiling stands and the cluster stays joined.
	Scene cluster;
	AddTripletImages(cluster, 75);
	for (IIndex i = 0; i + 1 < 60; ++i)
		AddTripletPair(cluster, i, i + 1, 1000);
	for (IIndex i = 0; i + 2 < 60; ++i)
		AddTripletPair(cluster, i, i + 2, 600);
	for (IIndex i = 60; i + 1 < 75; ++i)
		AddTripletPair(cluster, i, i + 1, 1000);
	for (IIndex i = 60; i + 2 < 75; ++i)
		AddTripletPair(cluster, i, i + 2, 600);
	AddTripletPair(cluster, 59, 60, 700);
	AddTripletPair(cluster, 58, 60, 700);
	AddTripletPair(cluster, 59, 61, 700);
	TripletFilterConfig clusterCfg;
	clusterCfg.enabled = true;
	clusterCfg.minYield = 0.f;
	IIndexArr clusterSeeds;
	const unsigned clusterRemoved = FilterPairsByTriplets(cluster, clusterCfg, weightingCfg, &clusterSeeds);
	const std::set<std::pair<IIndex,IIndex>> clusterKept = TripletKeptPairs(cluster);
	if (clusterRemoved != 71 || clusterKept.size() != 76 || clusterKept.count({59,60}) != 1 || clusterSeeds.size() != 75) {
		VERBOSE("TripletAutoTauTest FAILED: the face and its cluster removed %u pairs, kept %u, seed views %u; expected the "
			"paper's ceiling (the second piece is a quarter of the first): 71 removed, 76 kept, the bridges kept, seed views 0-74",
			clusterRemoved, (unsigned)clusterKept.size(), (unsigned)clusterSeeds.size());
		return false;
	}
```

Both scenes use the default `minScore` and `secondFaceScore` on purpose (they pin the defaults); do not
pin them. Extend the PASSED message to name the two faces and the cluster, and add both scenes to the
test's scene index (one line each: "the two faces pin that the ceiling at the second-face score is used
when it leaves a majority piece with a second piece of at least a third; the face and its cluster pin that
a second piece smaller than that leaves the paper's ceiling in force").

- [ ] **Step 2: Run the build to verify the tests fail**

Run (from `make/`): `ninja -f build-Release.ninja Tests 2>&1 | tail -5`
Expected: compilation errors on `secondFaceScore` and `secondPiece` — the red state.

- [ ] **Step 3: The configuration and the survivor graph**

In `libs/SFM/ViewGraphTriplets.h`, `TripletFilterConfig`: `minScore` back to `0.6f`, its comment back to
the paper's values with autoTau's role ("The paper's minimum edge score m, in [0,1] (the domain this
implementation enforces): 0.6 generic/large-scale, 0.9 highly ambiguous, 0.3 medium/small ambiguous.
With autoTau this is the ceiling the threshold is derived from and never exceeds -- unless the graph
shows a second face, see secondFaceScore."), and after it:

```cpp
	// A second, stricter ceiling, tau(secondFaceScore), tried first: it is the ceiling used when
	// the graph it leaves has two faces -- its largest piece holds a strict majority of the images
	// in pieces and its second-largest piece at least a third of the largest. A two-faced building
	// matched exhaustively (the church: seven matchings) splits into its faces at this ceiling and
	// merges them at tau(minScore) in half the matchings, while a building whose graph is one face
	// (Big Ben) is cut so thin at this ceiling that the reconstruction keeps a third of it. Values
	// at or below minScore switch the second ceiling off.
	float secondFaceScore = 0.75f;
```

In `SurvivorGraph`, after `largestPiece`: `unsigned secondPiece; // images in the second-largest piece (0 when there is none)`.
In `EvaluateSurvivorGraph`, initialise it and, in the piece loop where `largestPiece` and its root are
tracked, keep the top two sizes: when a component becomes the largest, the previous largest becomes
`secondPiece`; otherwise `secondPiece = MAXF(secondPiece, component.second)`.

- [ ] **Step 4: The ceiling choice**

In `FilterPairsByTriplets`, where `ceiling` is set from `tripletScores.tau` and `atCeiling` computed
(after `unfiltered` and `minPiece`), the ceiling becomes a choice:

```cpp
	// Two ceilings: the paper's tau(m), and the stricter tau at the second-face score, tried first.
	// The stricter one is used only when the graph it leaves has two faces -- a majority piece and a
	// second piece holding at least a third of it. On the church matched exhaustively the paper's
	// ceiling sits among the scores of the pairs bridging the facades and merges them in half the
	// matchings, while the stricter one splits them in every matching; on Big Ben, whose graph is one
	// face, the stricter ceiling keeps so few pairs that the reconstruction discards two thirds of
	// what it registers, while the paper's keeps 371 of 403. What hangs off a majority piece at the
	// stricter ceiling with less than a third of its images is a cluster, not a face, and the paper's
	// ceiling stands.
	float ceiling = tripletScores.tau;
	const SurvivorGraph unfiltered = EvaluateSurvivorGraph(scene, tripletScores.scores, 0.f);
	unsigned minPiece = (unsigned)std::ceil(0.01 * (double)unfiltered.largestComponent);
	const float secondFace = std::isnan(config.secondFaceScore) ? 0.f : CLAMP(config.secondFaceScore, 0.f, 1.f);
	const float secondCeiling = secondFace * (1.f - degreeRatio) + degreeRatio;
	bool twoFaced = false;
	if (config.autoTau && secondFace > minScore) {
		const SurvivorGraph atSecond = EvaluateSurvivorGraph(scene, tripletScores.scores, secondCeiling, minPiece);
		twoFaced = 2 * atSecond.largestPiece > atSecond.numInPieces && 3 * atSecond.secondPiece >= atSecond.largestPiece;
		if (twoFaced)
			ceiling = secondCeiling;
	}
	SurvivorGraph atCeiling = EvaluateSurvivorGraph(scene, tripletScores.scores, ceiling, minPiece);
```

(the existing `minPiece`/`atCeiling` lines are replaced by these; the no-piece fallback, the seed views,
the majority test and the descent follow unchanged and read `ceiling` and `atCeiling`). Change the
descent's VERBOSE so its ceiling clause reads `(ceiling %.3f at m %.2f%s, d_max/|V| %.3f)` where the
`%s` is `", the second face's"` when `twoFaced` and `""` otherwise, and `m` printed is the score the
ceiling came from (`twoFaced ? secondFace : minScore`). Add one line after the ceiling is chosen, at
VERBOSE level: `"Triplet filter: the ceiling at the second-face score %.2f (%.3f) leaves %s"` with either
`"two faces, pieces of %u and %u images: used"` or `"no second face (largest piece %u, second %u): the paper's ceiling stands"`
— write it as one VERBOSE with the two texts chosen by `twoFaced`, filled from `atSecond`'s
`largestPiece` and `secondPiece` (hoist `atSecond` so it is readable there, or print inside the `if`).

- [ ] **Step 5: The option and the note**

In `apps/CreateStructure/CreateStructure.cpp`: the `--triplet-min-score` help loses its sentence on 0.75
and reads, after the paper's values, "; the default is the paper's generic 0.6". Add, right after it, the
option `triplet-second-face-score` bound to a new `OPT::fTripletSecondFaceScore` with default
`TripletFilterConfig().secondFaceScore` and the help "camera-triplet filter: a stricter minimum edge score
whose ceiling is used instead of the default's when the graph it leaves has two faces (a majority piece
and a second piece of at least a third of it); at or below --triplet-min-score it is off", copy it into
`cfg.tripletFilterCfg.secondFaceScore` beside the other two, and give it the same `[0,1]` range check as
the minimum score with the message "--triplet-second-face-score must be in [0,1] (got %g)".

In `docs/design/TripletDisambiguation.md`, the flags table: `--triplet-min-score` default `0.6` with its
clause on 0.75 deleted, and a new row `--triplet-second-face-score F | 0.75 | a stricter minimum score whose
ceiling replaces the default's when the graph it leaves has two faces: a majority piece and a second piece
of at least a third of it`.

- [ ] **Step 6: Run the tests to verify they pass**

Run (from `make/`): `ninja -f build-Release.ninja Tests SFM CreateStructure SceneAnalyzeSFM && ./bin/Release/Tests 1`
Expected: every test PASSED, exit code 0, no compiler warning from the changed files. The walk, chains,
yield, flood and pairs scenes pin their own `minScore` at 0.6 and keep their values; the pan scene and
the older scenes (barbell, ring, bridge, pendant, baseline, boundary) run at the default and must keep
their values too — on each, the second ceiling leaves no majority piece with a large second piece (say so
in the report if any of them changed, and stop: that is a derivation to check, not an expectation to edit).

- [ ] **Step 7: Mutate**

One at a time, rebuild `Tests` only, run, confirm the named failure, revert, rebuild, confirm green:

| mutation | expected failure |
|---|---|
| the second ceiling never used (`twoFaced` forced false) | `TripletAutoTauTest FAILED: two faces removed 96 pairs, kept 101` |
| the third dropped (`3 * secondPiece >= largestPiece` removed) | `TripletAutoTauTest FAILED: auto-tau removed 1 pairs at the requested m itself, expected exactly 0` -- the boundary scene, which runs first: without the third a graph the second ceiling leaves in one piece is two-faced by the majority test alone, so every one-piece scene switches ceilings; the face and its cluster would fail after it (74 removed, 73 kept) |

- [ ] **Step 8: Commit**

```bash
git add libs/SFM/ViewGraphTriplets.h libs/SFM/ViewGraphTriplets.cpp apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.cpp docs/design/TripletDisambiguation.md
git -c user.name=cDc -c user.email=cdc.seacave@gmail.com commit -m "sfm: the triplet ceiling looks for the second face

The default minimum score returns to the paper's 0.6. A second, stricter ceiling at 0.75 is tried
first and used only when the graph it leaves has two faces, a majority piece with a second piece
of at least a third of it: on the church matched exhaustively the paper's ceiling sits among the
scores of the pairs bridging the facades and merges them in half the matchings while the stricter
one splits every matching, and on Big Ben, whose graph is one face, the stricter ceiling cuts the
graph so thin that the reconstruction keeps 147 of 403 images where the paper's keeps 371."
```

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
