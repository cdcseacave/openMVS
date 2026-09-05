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
  threshold is safe, and that `--triplet-min-score` is the strictness it starts from and only ever
  relaxes.
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
