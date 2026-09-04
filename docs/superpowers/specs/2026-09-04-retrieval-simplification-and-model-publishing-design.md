# RETRIEVAL simplification, manifest v1, and model publishing — design

**Status:** approved by the user 2026-09-04, executing.

**Goal:** make the global-descriptor pair selection a first-class *alternative* to the vocabulary
tree rather than a backend that silently substitutes itself into it, reset the export manifest to
version 1 while the branch is unreleased, and give every OpenMVS user a one-command way to obtain
the exported RoMa v2 model.

**Non-goals:** any backward compatibility (explicitly waived by the user); fp16 export (see §3.1);
a disk-backed descriptor cache (see §4 — the measurement does not ask for one yet).

---

## 0. What was measured first, and what it settles

Three of the user's questions are answered by the code as it stands. They are recorded here because
two of them mean *no change*, and a spec that does not say so invites someone to change them anyway.

### 0.1 RETRIEVAL-only already runs the fast path

With pair selection on the global descriptors and `--roma2-match false`:

- **The coarse-match graph is never loaded.** `RoMa2Onnx::Load` calls only `LoadDescriptor`;
  the match session comes from `Impl::EnsureMatch()`, whose sole caller is `MatchCoarse`
  (`RoMa2Matcher.cpp:521`). A retrieval-only run therefore never pays the 448 MB of coarse-graph
  weights, nor its session, nor its four host warp tensors.
- **`value_facets` is never read back.** `Describe` binds it to `facetsScratch`, a device tensor;
  `facetsHost` is allocated lazily and only the parity test ever asks for it.
- **One `layers` tensor is allocated for the whole pass**, not one per image
  (`ComputeGlobalDescriptorsROMA2`, `MatchROMA2.cpp:604`).
- **8 KB crosses the bus per image** — the graph pools GeM(p=3) → concat → signed power → L2 on
  device and returns the finished 2048-D vector as the `retrieval` output.
- Image load and preprocessing are pipelined on the thread pool ahead of the single-threaded
  `Describe` through `PrefetchRing`.

Measured: 225 images described in 9.7 s on an A100 (43 ms/image), against ~2 min 45 s for the same
scene's dense matching pass.

**The one residue, and why it is left alone.** The descriptor graph still emits `layers`
(blocks 11 and 17) which a retrieval-only pass never reads. Note this is the opposite of the
assumption in the request: `value_facets` (blocks 15, 20) is what *retrieval* needs — it is the
tensor the on-device pooling consumes — while `layers` is what the *coarse matcher* consumes as
`descriptors_A`/`descriptors_B`. Pruning `layers` would need a third exported graph, and buys:

- **12.5 MB of VRAM** (`[1,2,40,40,1024]` fp32), allocated once for the whole pass, not per image;
- the device-side write of that tensor per image — ~13 MB at ~1.5 TB/s, i.e. well under 1 ms
  against a 43 ms ViT-L forward pass.

The backbone must run through block 20 to produce `value_facets` either way, so no transformer
compute is saved. **Decision: no retrieval-only graph.** A third graph to publish, a third manifest
key and a third parity fixture set, for under 1 % of the pass. Revisit only if VRAM, not time,
becomes the binding constraint.

### 0.2 Retrieval scales linearly; the vectors stay in RAM

`GlobalDescriptors` holds one `N x 2048` float matrix — 41 MB at 5 000 images, 164 MB at 20 000 —
and `Query` is a single GEMV plus a `partial_sort`, O(N·D) per query with no N×N matrix anywhere.
The user's guess is confirmed: the retrieval vectors are small enough to stay resident and need no
cache. `Image::globalDescriptor` is 8 KB per image and is serialized with the scene, so a second run
skips the describe pass entirely (§1.4 makes that path reachable without loading the model at all).

### 0.3 The dense-matching descriptor cache exists and is optimal-by-construction

For `--roma2-match true` the per-image artefact is the 12.5 MB `layers` tensor, and it cannot all
live on the device: 5 000 images would be 61 GB. The existing cache is `MakeSlotPlan`
(`MatchROMA2.cpp:241`) — **Belady's optimal replacement** over the pair list in `(ID1,ID2)` order:

- `--roma2-slots N` bounds device residency at `N x 12.5 MB` (the 64 default = 800 MB; the campaign
  ran 139–251 slots for 225–251 image captures, holding every image);
- the plan grows only to what it needs, so a scene smaller than the budget never evicts;
- eviction costs a **re-describe** (43 ms), not correctness — there is no disk spill;
- the pass reports `loads` and `reloads` separately, so the cache's cost is visible per run.

Belady is optimal *given the visit order*, and the visit order is chosen for locality —
`(ID1,ID2)` sorting keeps each image's pairs adjacent. The campaign's 225-image scenes recorded
**218 loads, 0 reloads**. Whether a 5 000-image capture stays near zero depends on its pair graph's
bandwidth, which is a property of the capture, not of the code.

**Decision: no disk-backed cache yet.** Adding one is a real option — 12.5 MB/image on disk against
43 ms of recompute — but it trades a measured-zero cost for guaranteed I/O. §4 documents the
mechanism and the knob so the decision can be made on a measurement instead of a fear.

---

## 1. Pair selection: two modes, one job each

Today `PairsMatcher::UseGlobalDescriptors()` lets the global descriptors substitute themselves into
VOCABULARY mode whenever the scene happens to carry them and ROMA2 is opted in, so "VOCABULARY"
names two different algorithms and `CollectVocabularyPairs` has to label its own summary line with
whichever one ran. That is the mixing the user asked to remove.

### 1.1 The rule

| `--match-mode` | ranking backend | model needed |
|---|---|---|
| `1` VOCABULARY | the SIFT/AKAZE vocabulary tree, **always** | no |
| `4` RETRIEVAL  | the DINOv3 + GeM(p=3) global descriptors, **always** | only to compute them |

Nothing else changes about either mode: the reciprocal-rank fusion, the mutual top-K agreement and
the component bridging stay in `CollectFusedRetrievalPairs`, shared, because they are properties of
the ranking and not of the backend.

### 1.2 Deletions

- `PairsMatcher::UseGlobalDescriptors()` — the whole substitution rule.
- `PairsMatcher::EnsureRetrievalIndex()` — the backend chooser. `CollectVocabularyPairs` calls
  `EnsureVocabularyTree()`, `CollectRetrievalPairs` calls `EnsureGlobalDescriptorsIndex()`.
- The `backendName` parameter of `CollectFusedRetrievalPairs`: each caller now has exactly one
  backend, so each passes its own constant literal.
- The pre-match special case in `MatchRound` (`PairsMatcher.cpp:2207`), which existed only to build
  a vocabulary tree for pre-matching when the global descriptors had displaced it. VOCABULARY builds
  the tree by definition now; RETRIEVAL never pre-matches (design decision 10, unchanged).

### 1.3 `--roma2-retrieval` goes away

The flag answered "should the global descriptors displace the tree", which is no longer a question.
`ROMA2Config::useRetrieval` is deleted. What remains is a derived fact, not a setting:

```cpp
// Does this configuration need the scene described?
bool NeedsGlobalDescriptors() const;   // match mode is RETRIEVAL, or the retrieval CSV was asked for
bool NeedsWarps() const { return useMatching; }
```

`ROMA2Config` cannot see the match mode, so `Scene::MatchPairs` decides it, from
`config.mode == MatchConfig::RETRIEVAL || !config.exportRetrievalCSV.empty()`.

### 1.4 What `Scene::MatchPairs` does then

```
needsDescriptors = (matchMode == RETRIEVAL || retrieval CSV requested) && !scene has GLOBAL_DESCRIPTORS
needsWarps       = roma2Cfg.useMatching

if !needsDescriptors && !needsWarps:   load nothing, touch no session
else:                                  load the model, run the describe pass if needsDescriptors
```

A second run over a scene already carrying descriptors loads **no ONNX session at all** and ranks
straight off `Image::globalDescriptor`. That path exists today but is gated behind `useRetrieval`;
after this change it is simply what happens.

### 1.5 CLI

- `--roma2-retrieval` removed from `CreateStructure`.
- `--roma2` alone is now a configuration with nothing to do unless `--roma2-match true` or
  `--match-mode 4` accompanies it. Reject it up front, in the style of the existing
  `--roma2-match needs --roma2 true` check:
  `error: --roma2 has nothing to do without --roma2-match true or --match-mode 4 (RETRIEVAL)`
- `--export-retrieval-csv` validation becomes: needs `--roma2 true` (it no longer needs a second
  flag), and it makes the describe pass run even under VOCABULARY selection — the CSV is a
  diagnostic *about* the descriptors, so asking for it is asking for them.
- `--match-mode 4` **without** `--roma2` stays legal and unchanged: a scene loaded from disk may
  already carry descriptors, and `GlobalDescriptors::Build` already fails loudly and by name when it
  does not. No new early check — the option parser cannot know what an `.sfm` contains.

### 1.6 Out-of-tree consumers

`~/virginia/datasets/openmvs-roma2-20260901-task6-tools/run_*.sh` pass `--roma2-retrieval true`.
They must drop it, or every future campaign run dies in option parsing. They are measurement
runners, not repo files; update them in the same change and note it in the run folders' provenance.

---

## 2. Manifest format version 1

The branch is unreleased and carries no compatibility duty, so the shipped format is version 1 and
the loader accepts exactly that. Version 3 was the running count of an export format nobody outside
this branch has ever consumed.

- `scripts/python/roma2/export.py`: `FORMAT_VERSION = 1`, and the comment on it states the format
  itself (bidirectional coarse graph; `retrieval` output on the descriptor graph) rather than the
  history of versions 2 and 3.
- `RoMa2Manifest::Load` (`RoMa2Matcher.cpp:116-121`): accept `1`, reject everything else with the
  same message shape it uses now.
- The archaeology comments naming version 2 and 3 (`RoMa2Matcher.cpp:119-120,346`, `export.py`
  around the `img_A`/`img_B` dead inputs and the `check` stale-reference hint) go with them.
- Re-export so the shipped manifests declare 1 (§3.2). Hand-patching the JSON would leave the
  `*.export.json` sidecars disagreeing with their own manifests.

---

## 3. Publishing the model

### 3.1 What ships, and at what precision

**fp32, `base` preset only, in the first bundle.** The approved proposal included fp16, and the
size case for it is real (1.4 GB → ~715 MB), but the exporter emits fp32 by construction
(`export.py` writes `"precision": "fp32"` and traces the eager model as-is); adding fp16 means
either tracing a halved model or post-converting the graph, and then re-establishing parity against
the eager references for both stages. That is its own piece of work with its own measurement, and
blocking the publishing plumbing on it would be the wrong order. fp32 `base` is 1.4 GB, which fits
under the 2 GB GitHub release-asset cap and is unremarkable for the Hub.

`turbo` and `fast` are published later from the same repo layout, and fp16 lands as a sibling
directory when it has been measured — the fetch script takes `--setting` and `--precision`
precisely so neither addition changes anything already shipped.

Excluded from the published bundle: the `*.reference/` fixture directories (218 MB). They are the
parity test's inputs, not a runtime requirement.

### 3.2 The export run

One export, producing `format_version 1` fp32 graphs for `base` into
`~/virginia/models/roma2-onnx/roma2onnx-<date>-v1/`, checked with `export.py check` against the
eager references exactly as the current bundle was. The existing bidirectional export stays in place
untouched — the campaign's recorded runs name it in their provenance.

### 3.3 Licence: the constraint that shapes the layout

The descriptor graph contains DINOv3 ViT-L weights. RoMa v2's own code is MIT (Johan Edstedt), but
DINOv3 is under Meta's DINOv3 License (last updated 2025-08-19), which:

- **§1(a)** grants a royalty-free, worldwide right to use, reproduce, **distribute**, copy, create
  derivative works of and modify the materials — no revenue threshold, no user cap, no acceptable-use
  annex, no branding requirement;
- **§1(b)(i)** requires that any redistribution of the materials *or derivatives* be **under that
  same Agreement, with a copy of it supplied**;
- **§1(b)(ii)** requires acknowledgement in published research;
- **§1(b)(iii),(v)** require trade-control compliance and forbid ITAR / military / weapons /
  nuclear / espionage uses;
- **§8** lets Meta amend the Agreement at any time — so a redistributor pins the version it shipped.

Upstream RoMa v2 already redistributes DINOv3-derived weights this way, from its own GitHub release
(`Parskatt/RoMaV2/releases/download/v2.0.1/romav2.0.1.pt`).

**Consequences, and they are architectural, not cosmetic:**

1. The weights are **not** AGPL and must never enter the OpenMVS repository or its release tarballs.
   They are a separate artefact, fetched on demand — which is already how the code is wired
   (`--roma2-model` / `$OPENMVS_ROMA2_MODEL_PATH`), so nothing needs redesigning.
2. The published bundle carries `LICENSE-DINOv3.md` (the pinned text), plus a `README.md` naming
   RoMa v2 (MIT) and DINOv3 (Meta), the upstream checkpoint SHA256, the `romav2_commit` and
   `dinov3_hub_commit` the manifests already record, and the acknowledgement wording.
3. OpenMVS's own docs state that running `--roma2` means accepting the DINOv3 License.

### 3.4 Hosting

**Canonical: a Hugging Face model repo** — free, CDN-fronted, resumable, and revision-pinnable, with
no per-file ceiling in the range that matters. **Mirror: a GitHub release asset** on the OpenMVS
release that introduces `--roma2`, giving a zero-dependency `curl` path for anyone who cannot reach
the Hub. The fetch script tries the Hub first and falls back to the mirror.

Repo layout, one directory per (setting, precision):

```
<repo>/
  README.md                  RoMa v2 + DINOv3 attribution, usage, the SHA256 table
  LICENSE-DINOv3.md          the pinned Agreement text (§1(b)(i))
  base-fp32/
    roma_base.json
    roma_base_descriptor_fp32.onnx  + .onnx.data
    roma_base_match_coarse_fp32.onnx + .onnx.data
    roma_base_descriptor_fp32.export.json
    roma_base_match_coarse_fp32.export.json
```

**The upload itself is not part of this change.** Creating a public repo under the user's account
and pushing 1.4 GB is an outward-facing action on their identity; the change delivers the bundle,
the checksums and the tooling, and the user runs the two documented commands. The fetch script's
default repo id is a constant in one place, so it is a one-line edit if they choose another name.

### 3.5 `scripts/fetch_roma2_model.py`

```
usage: fetch_roma2_model.py [--setting base] [--precision fp32]
                            [--dest DIR] [--repo ID] [--revision REV] [--mirror URL]
```

- Resolves `--dest` to `$OPENMVS_ROMA2_MODEL_PATH` when unset, else the CMake-installed
  `<prefix>/share/openMVS/roma2`.
- Prefers `huggingface_hub.snapshot_download` when importable — resumable, deduplicating, and
  revision-pinned. Falls back to streaming `urllib` from the GitHub mirror when it is not, writing
  through a `.part` file so an interrupted fetch never leaves a truncated graph in place.
- **Verifies SHA256 of every file against `models/roma2/checksums.txt`, which lives in the OpenMVS
  repo.** That is what pins the artefact without the repo carrying a byte of it, and it is what
  makes a mirror as trustworthy as the canonical host. A mismatch deletes the file and fails.
- Idempotent: a file already present with the right digest is left alone, so re-running costs one
  hash per file.
- Prints the DINOv3 licence notice and the path to set `OPENMVS_ROMA2_MODEL_PATH` to.

### 3.6 CMake

An **opt-in target**, never a configure-time download — 1.4 GB during `cmake ..` would be hostile
and would break offline and CI builds:

```cmake
if(OpenMVS_USE_ONNXRUNTIME)
  add_custom_target(roma2-model
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/fetch_roma2_model.py
            --setting base --precision fp32
            --dest ${CMAKE_INSTALL_PREFIX}/share/openMVS/roma2
    COMMENT "Fetching the RoMa v2 ONNX model (~1.4 GB, DINOv3 License)"
    USES_TERMINAL)
endif()
```

Excluded from `all`. CI is unaffected: the test suites already skip cleanly with
`OPENMVS_ROMA2_MODEL_PATH` unset, and no CI job depends on this target.

### 3.7 `ResolveModelPath()` gains an install-prefix fallback

So that a user who ran the target just passes `--roma2` and nothing else:

```
--roma2-model                      (explicit, wins)
$OPENMVS_ROMA2_MODEL_PATH          (as today)
<install prefix>/share/openMVS/roma2   (new, last)
```

The install prefix reaches the binary as a compile definition set by CMake. The fallback is only
*returned* when that directory exists, so an uninstalled build keeps producing the existing
"needs a model" error rather than a path that is not there.

---

## 4. Documentation

`docs/design/ROMA2InProcess.md` gains a **Scaling** section carrying §0.1–0.3 of this spec as
prose: what a retrieval-only pass computes and what it does not, the per-image and per-slot memory
arithmetic, the Belady cache and its `loads`/`reloads` accounting, the `--roma2-slots` knob, and the
explicit statement that eviction costs a re-describe rather than correctness. It ends with the
disk-cache option stated as an option, with the numbers a decision would need.

`libs/SFM/README.md` drops `--roma2-retrieval` from the flag list and restates the two selection
modes as the table in §1.1.

A new `docs/RoMa2Model.md` covers obtaining the model: the target, the script, the licence, and the
two commands that publish a new bundle.

---

## 5. Testing

- `RetrievalModeTest` keeps its end-to-end assertion but loses the half that compares RETRIEVAL
  against "opted-in VOCABULARY" — that configuration no longer exists.
- A new case asserts VOCABULARY ranks by the tree **even when every image carries a global
  descriptor**, which is the substitution this change removes and the one a future edit could
  reintroduce without any other test noticing.
- A manifest fixture at `format_version: 3` must now be rejected, and one at `1` accepted.
- `fetch_roma2_model.py` gets a unit test over its digest check on a temporary directory: right
  digest passes and is idempotent, wrong digest deletes and fails. No network.

## 6. Ordering

§2 (version) and §1 (simplification) are independent of each other and of §3.5–3.7 (tooling).
§3.2 (the export run) depends on §2, and `checksums.txt` depends on §3.2. So: simplification and
version reset in either order, export once both are in, checksums and docs last.
