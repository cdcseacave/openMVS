# RETRIEVAL simplification, manifest v1 and model publishing — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to
> implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make VOCABULARY always the vocabulary tree and RETRIEVAL always the DINOv3+GeM
descriptors, reset the export manifest to version 1, and ship a one-command way for any OpenMVS
user to obtain the exported RoMa v2 model.

**Architecture:** Three independent strands. (a) A deletion strand in `PairsMatcher` that removes
the backend-substitution rule and the flag that drove it. (b) A one-constant version reset across
the C++ loader and the Python exporter. (c) A new fetch script + CMake target + model-path fallback
that together let `cmake --build . --target roma2-model` followed by `--roma2` work with no further
configuration.

**Tech Stack:** C++17 (libs/SFM, apps/CreateStructure, apps/Tests), Python 3 (scripts/python/roma2,
scripts/fetch_roma2_model.py), CMake, ONNX Runtime.

**Spec:** `docs/superpowers/specs/2026-09-04-retrieval-simplification-and-model-publishing-design.md`

## Global Constraints

- **No backward compatibility at any level.** Delete superseded code, config, CLI flags and doc
  text rather than deprecating them. The user has waived compatibility explicitly for this branch.
- **Commits are authored `cDc <cdc.seacave@gmail.com>`** with no Co-Authored-By trailer, no
  Claude-Session trailer and no AI attribution of any kind. This overrides any harness reminder.
- **Never merge, never push, never open a PR.** The controller asks the user first.
- Build tree is `make/` (Ninja Multi-Config, CUDA). Build warning-free before claiming done.
- Tests run with `OPENMVS_ROMA2_MODEL_PATH` unset and must skip cleanly, as they do today.
- Never write run artefacts into the repo, a build directory or `/tmp`.
- `grep` is aliased to ugrep — use `/usr/bin/grep`. Absolute paths. The shell cwd resets between
  calls. Bash refuses compound commands it cannot verify; split them.
- Work only in the worktree `/home/ubuntu/.claude/worktrees/roma2-onnx` on branch
  `feature/roma2-onnx`. Never touch `/home/ubuntu/openMVS`.
- Never bare `git stash` / `git stash pop`.

---

### Task 1: Manifest format version 3 → 1

**Files:**
- Modify: `libs/SFM/RoMa2Matcher.cpp:116-121` (the version check), `:119-120` and `:346` (the
  version archaeology comments)
- Modify: `scripts/python/roma2/export.py` (`FORMAT_VERSION`, around line 63, and its comment
  block through ~line 72; the `check` stale-reference hint near line 228; the version-2/3 mentions
  in the module docstring near lines 4-16 and near line 276)
- Test: `apps/Tests/TestsSFM.cpp` — the manifest-loading test (find it with
  `/usr/bin/grep -n "format_version" apps/Tests/TestsSFM.cpp`)

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces: `RoMa2Manifest::Load` accepts `format_version == 1` and nothing else. No signature
  changes.

- [ ] **Step 1: Read the current check and its comments**

Read `libs/SFM/RoMa2Matcher.cpp:110-130` and `:340-350`. The check today accepts only `3` and the
comments explain what versions 2 and 3 each added.

- [ ] **Step 2: Write the failing test**

In `apps/Tests/TestsSFM.cpp`, find the existing manifest test. Add (or adapt) two cases: a manifest
JSON whose `format_version` is `1` loads, and one whose `format_version` is `3` is rejected. If no
manifest test exists, add `RoMa2ManifestVersionTest` next to the other RoMa2 manifest tests,
writing a minimal manifest to a temporary file, and register it in `apps/Tests/TestsSFM.h` and
`apps/Tests/Tests.cpp` alongside its neighbours.

- [ ] **Step 3: Run it and watch it fail**

```
cmake --build /home/ubuntu/.claude/worktrees/roma2-onnx/make --config Release --target Tests
```
Then run the SFM test binary. Expected: the `format_version: 1` case fails ("unsupported version").

- [ ] **Step 4: Flip the constant in C++**

In `RoMa2Matcher.cpp`, accept `1` instead of `3`. Rewrite the comment so it states **the format**
— a bidirectional coarse-match graph returning `warp`/`confidence` and `warp_BA`/`confidence_BA`
from one `Run`, and a descriptor graph whose third output `retrieval` is the FACETS recipe pooled
on device — with no mention of what versions 2 or 3 used to be. Delete the version-2 archaeology at
`:346` (the dead `img_A`/`img_B` input note keeps only the part describing today's behaviour).

- [ ] **Step 5: Flip the constant in the exporter**

`scripts/python/roma2/export.py`: `FORMAT_VERSION = 1`, comment rewritten the same way. Update the
module docstring and the `check` subcommand's stale-reference hint so neither names version 2 or 3.

- [ ] **Step 6: Verify**

Rebuild and run the tests. Both new cases pass; no other test regresses.

- [ ] **Step 7: Commit**

```bash
git add libs/SFM/RoMa2Matcher.cpp scripts/python/roma2/export.py apps/Tests/TestsSFM.cpp apps/Tests/TestsSFM.h apps/Tests/Tests.cpp
git commit -m "sfm: the exported manifest format is version 1, and says what it is rather than what it was"
```

---

### Task 2: Delete the global-descriptor substitution from VOCABULARY

**Files:**
- Modify: `libs/SFM/PairsMatcher.h` (declarations of `UseGlobalDescriptors`, `EnsureRetrievalIndex`,
  `CollectFusedRetrievalPairs`; the VOCABULARY enum comment around `:163-166`)
- Modify: `libs/SFM/PairsMatcher.cpp:715` (`UseGlobalDescriptors`), `:744` (`EnsureRetrievalIndex`),
  `:801` (`CollectVocabularyPairs`), `:806` (the labelled fusion call), `:809`
  (`CollectRetrievalPairs`), `:2207` (the pre-match special case)
- Test: `apps/Tests/TestsSFM.cpp:8414+` (`RetrievalModeTest`)

**Interfaces:**
- Consumes: nothing from Task 1.
- Produces: `PairsMatcher` no longer declares `UseGlobalDescriptors()` or `EnsureRetrievalIndex()`.
  `CollectFusedRetrievalPairs(unsigned topK)` loses its `backendName` parameter — Task 3 does not
  call it, so no cross-task coupling.

- [ ] **Step 1: Read the four call sites**

`/usr/bin/grep -n "UseGlobalDescriptors\|EnsureRetrievalIndex\|CollectFusedRetrievalPairs" libs/SFM/*.cpp libs/SFM/*.h apps/Tests/TestsSFM.cpp`
Every hit must be accounted for by the end of this task.

- [ ] **Step 2: Write the failing test**

In `apps/Tests/TestsSFM.cpp`, add `VocabularyIgnoresGlobalDescriptorsTest`: build a scene where
every image carries a global descriptor **and** `Scene::Status::STATE::GLOBAL_DESCRIPTORS` is set
(the `RetrievalModeTest` construction at `:8422-8437` is the model to copy), run `PairsMatcher` in
`MatchConfig::VOCABULARY` mode, and assert the vocabulary tree was the backend. Assert it on an
observable: the DEBUG summary names `Vocabulary`, or — preferably, since a test should not parse
logs — that the produced pair set differs from the RETRIEVAL one on a scene constructed so the two
backends disagree. Give the two backends different truth: the global descriptors cluster the images
one way (as in `RetrievalModeTest`), the local descriptors another. Register it in `TestsSFM.h` and
`Tests.cpp`.

- [ ] **Step 3: Run it and watch it fail**

Today, with `ROMA2Config::enabled = true` and `useRetrieval = true`, VOCABULARY silently ranks by
the global descriptors, so the test sees the RETRIEVAL pair set and fails.

- [ ] **Step 4: Delete `UseGlobalDescriptors()`**

Remove the method from `PairsMatcher.h` and `.cpp:715-727` entirely.

- [ ] **Step 5: Delete `EnsureRetrievalIndex()` and re-point its callers**

Remove it from `PairsMatcher.h` and `.cpp:744-757`. `CollectVocabularyPairs` (`:801`) calls
`EnsureVocabularyTree()` and returns an empty `PairIdxArr` when the tree could not be built;
`CollectRetrievalPairs` (`:809`) keeps calling `EnsureGlobalDescriptorsIndex()`.

- [ ] **Step 6: Drop the `backendName` parameter**

`CollectFusedRetrievalPairs(unsigned topK)`. Each caller's DEBUG summary now carries its own
constant: `_T("Vocabulary")` from `CollectVocabularyPairs`, `_T("Global-descriptor")` from
`CollectRetrievalPairs`. Keep the summary line's shape unchanged. Update the doc comment on the
declaration so it no longer says "over whichever backend the caller already ensured" — it is now
"over the backend its caller built".

- [ ] **Step 7: Delete the pre-match special case**

At `PairsMatcher.cpp:2207` remove the
`if (matchMode != MatchConfig::RETRIEVAL && config.preMatchThreshold > 0 && UseGlobalDescriptors()) EnsureVocabularyTree();`
line and the comment paragraph above it explaining why it existed. The `if (vocabularyTree)` block
that follows is unchanged: VOCABULARY has built the tree by construction, RETRIEVAL never has one,
and that is now the whole rule. Leave a one-line comment saying so.

- [ ] **Step 8: Update the VOCABULARY enum comment**

`PairsMatcher.h` around `:163-166` — VOCABULARY is "Use vocabulary tree retrieval over the local
descriptors (recommended)", with the clause about global descriptors and `UseGlobalDescriptors`
deleted. `RETRIEVAL` gains the matching one-liner.

- [ ] **Step 9: Fix `RetrievalModeTest`**

At `:8442-8443` the comment explains that RETRIEVAL works "without the ROMAv2 opt-in that VOCABULARY
needs" — rewrite it: the two modes now name two backends and neither consults a gate. Delete the
half of the test that compares RETRIEVAL against opted-in VOCABULARY (`:8487` and its setup): that
configuration no longer exists. Keep the end-to-end `Match()` case at `:8530`.

- [ ] **Step 10: Verify**

Rebuild warning-free; run the full test suite. `VocabularyIgnoresGlobalDescriptorsTest` passes,
`RetrievalModeTest` passes, nothing else regresses.

- [ ] **Step 11: Commit**

```bash
git add libs/SFM/PairsMatcher.h libs/SFM/PairsMatcher.cpp apps/Tests/TestsSFM.cpp apps/Tests/TestsSFM.h apps/Tests/Tests.cpp
git commit -m "sfm: one backend per match mode -- the vocabulary tree ranks VOCABULARY, the global descriptors rank RETRIEVAL"
```

---

### Task 3: Remove `--roma2-retrieval`, and let the match mode decide the describe pass

**Files:**
- Modify: `libs/SFM/MatchROMA2.h:47-84` (`ROMA2Config`: delete `useRetrieval`, replace
  `IsInProcessEnabled()`, add `NeedsGlobalDescriptors` semantics per the spec)
- Modify: `libs/SFM/Scene.cpp:564-646` (`Scene::MatchPairs`)
- Modify: `apps/CreateStructure/CreateStructure.cpp:171` (the flag), `:259-262`
  (`--export-retrieval-csv` validation), `:296-305` (the `--roma2-match` / `--roma2` checks),
  `:362-374` (config wiring)
- Test: `apps/Tests/TestsSFM.cpp` — wherever `useRetrieval` appears
  (`/usr/bin/grep -n "useRetrieval" apps/Tests/TestsSFM.cpp libs/ apps/`)

**Interfaces:**
- Consumes: Task 2's deletions (this task must not reintroduce a backend gate).
- Produces: `ROMA2Config` without `useRetrieval`. `ROMA2Config::IsInProcessEnabled()` becomes
  `enabled && !ResolveModelPath().empty()`; the "is any pass using it" question moves to the
  caller, which is the only place that can see the match mode.

- [ ] **Step 1: Reshape `ROMA2Config`**

Delete `bool useRetrieval`. Keep `NeedsWarps() { return useMatching; }`. `IsInProcessEnabled()`
becomes `enabled && !ResolveModelPath().empty()` — the config can no longer answer "is a pass using
this", so it stops pretending to. Update the comments accordingly.

- [ ] **Step 2: Move the decision into `Scene::MatchPairs`**

Replace the current gate (`Scene.cpp:583`) with the spec's §1.4:

```cpp
// Which passes this run needs. Only the caller can answer the first one: the config cannot see
// the match mode, and RETRIEVAL is what makes the global descriptors necessary.
const bool wantsDescriptors = (config.mode == MatchConfig::RETRIEVAL || !exportRetrievalCSV.empty());
const bool needsDescriptors = wantsDescriptors && !status.nState.isSet(Status::STATE::GLOBAL_DESCRIPTORS);
const bool needsWarps = roma2Cfg.useMatching;
```

The model is loaded iff `roma2Cfg.enabled && (needsDescriptors || needsWarps)`, and the existing
"requested but no model path" error keeps its shape. The existing reuse DEBUG line
(`Scene.cpp:598-600`) stays and becomes reachable in more cases.

`Scene::MatchPairs` does not currently receive the retrieval CSV path. Pass it: it is already on
`ReconstructionConfig::exportRetrievalCSV`, so add a parameter to `MatchPairs` and thread it from
`Scene::Reconstruct` (`Scene.cpp:736`). Update the declaration in `Scene.h`.

- [ ] **Step 3: Update the CLI**

Delete the `roma2-retrieval` option (`CreateStructure.cpp:171`) and `OPT::bROMA2Retrieval`
everywhere. `--export-retrieval-csv` now validates against `--roma2` alone (`:259-262`). Replace
the `--roma2 && (retrieval || match)` check at `:300` with the spec's new one:

```cpp
if (OPT::bROMA2 && !OPT::bROMA2Match && OPT::matchMode != 4) {
    LOG("error: --roma2 has nothing to do without --roma2-match true or --match-mode 4 (RETRIEVAL)");
    return false;
}
```

Do **not** add a check that `--match-mode 4` requires `--roma2`: a loaded scene may already carry
descriptors, and `GlobalDescriptors::Build` already fails by name when it does not.

- [ ] **Step 4: Update the tests**

Every `useRetrieval` reference disappears. `apps/Tests/TestsSFM.cpp:3971` sets `useMatching`; check
its neighbourhood for a `useRetrieval` sibling.

- [ ] **Step 5: Verify**

Rebuild warning-free, full test suite green. Then confirm the CLI by hand:
`make/bin/Release/CreateStructure --help` lists no `--roma2-retrieval`, and
`CreateStructure <anything> --roma2 true` alone reports the new error.

- [ ] **Step 6: Commit**

```bash
git add libs/SFM/MatchROMA2.h libs/SFM/Scene.h libs/SFM/Scene.cpp apps/CreateStructure/CreateStructure.cpp apps/Tests/TestsSFM.cpp
git commit -m "sfm: the match mode decides whether the scene needs describing, not a second flag"
```

---

### Task 4: `ResolveModelPath()` falls back to the installed model directory

**Files:**
- Modify: `libs/SFM/MatchROMA2.h` (`ROMA2Config::ResolveModelPath`)
- Modify: `libs/SFM/CMakeLists.txt` (the compile definition carrying the install prefix)
- Test: `apps/Tests/TestsSFM.cpp` (a new `ResolveModelPathTest`)

**Interfaces:**
- Consumes: Task 3's reshaped `ROMA2Config`.
- Produces: no signature change; `ResolveModelPath()` gains a third source.

- [ ] **Step 1: Add the compile definition**

In `libs/SFM/CMakeLists.txt`, define `OPENMVS_ROMA2_MODEL_INSTALL_DIR` as
`"${CMAKE_INSTALL_PREFIX}/share/openMVS/roma2"` on the SFM target. Guard it so a build that has not
set an install prefix still compiles.

- [ ] **Step 2: Write the failing test**

`ResolveModelPathTest`: with an explicit `modelPath` set, that wins; with it empty and
`OPENMVS_ROMA2_MODEL_PATH` set in the environment, that wins; with both absent and the install
directory absent on disk, the result is empty. Do not test the "install directory exists" branch by
creating files under the install prefix — assert the precedence of the first two and the empty
result of the third, which is the whole contract the pipeline depends on.

- [ ] **Step 3: Implement**

```cpp
inline String ResolveModelPath() const {
    if (!modelPath.empty())
        return modelPath;
    const char* const envModelPath = getenv("OPENMVS_ROMA2_MODEL_PATH");
    if (envModelPath)
        return String(envModelPath);
    #ifdef OPENMVS_ROMA2_MODEL_INSTALL_DIR
    // last: what `cmake --build . --target roma2-model` fetched. Only when it is actually there --
    // an uninstalled build must keep producing the "needs a model" error rather than a path that
    // does not exist.
    if (File::isFolder(OPENMVS_ROMA2_MODEL_INSTALL_DIR))
        return String(OPENMVS_ROMA2_MODEL_INSTALL_DIR);
    #endif
    return String();
}
```

Check `File::isFolder`'s exact name in `libs/Common` before using it.

- [ ] **Step 4: Verify and commit**

```bash
git add libs/SFM/MatchROMA2.h libs/SFM/CMakeLists.txt apps/Tests/TestsSFM.cpp apps/Tests/TestsSFM.h apps/Tests/Tests.cpp
git commit -m "sfm: --roma2 finds the model the build fetched, without an environment variable"
```

---

### Task 5: The fetch script, its checksum pin, and the CMake target

**Files:**
- Create: `scripts/fetch_roma2_model.py`
- Create: `models/roma2/checksums.txt` (placeholder header only — the controller fills it after the
  export run; the script must handle an entry-less file by failing with a clear message)
- Create: `scripts/python/tests/test_fetch_roma2_model.py`
- Modify: `CMakeLists.txt` (root) — the `roma2-model` target

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `scripts/fetch_roma2_model.py --setting <s> --precision <p> --dest <dir>`.

- [ ] **Step 1: Write the failing test**

`scripts/python/tests/test_fetch_roma2_model.py`, runnable with plain `python3 -m unittest` and no
network:
- `verify_digest(path, expected)` returns True for a file whose SHA256 matches and False otherwise.
- `fetch` over a `file://`-style local source: a file already present with the right digest is left
  untouched (assert the mtime is unchanged) and reported as cached.
- A file present with the **wrong** digest is deleted and the call fails.
- An empty `checksums.txt` fails with a message naming the file.

Structure `fetch_roma2_model.py` so these are importable functions, not one `main()`.

- [ ] **Step 2: Run it and watch it fail** — the module does not exist yet.

- [ ] **Step 3: Write the script**

Per spec §3.5. Required behaviour:
- `argparse` with `--setting` (default `base`), `--precision` (default `fp32`), `--dest`,
  `--repo` (default constant, one place), `--revision` (default `main`), `--mirror`.
- `--dest` default: `$OPENMVS_ROMA2_MODEL_PATH` if set, else `./roma2-model`. The CMake target
  always passes `--dest` explicitly, so this default only serves a hand-run.
- Prefer `huggingface_hub.snapshot_download(repo_id, revision, allow_patterns=[f"{setting}-{precision}/*"])`
  when importable; on ImportError fall back to streaming each file from the mirror URL with
  `urllib.request`, writing to `<name>.part` and renaming only after the digest check passes.
- Read `models/roma2/checksums.txt` — lines of `<sha256>  <relative/path>`, `#` comments allowed —
  located relative to the script, not the cwd.
- Verify every fetched file. On mismatch: delete it, print the expected and actual digests, exit 1.
- Print the DINOv3 licence notice and the resulting path, and remind the caller they can set
  `OPENMVS_ROMA2_MODEL_PATH` to it (or that Task 4's fallback already finds it if `--dest` was the
  install directory).
- Exit 0 and print "already present" when everything verifies without downloading.

- [ ] **Step 4: Write `models/roma2/checksums.txt`**

Header comment only for now, explaining the format, that it pins the published artefact without the
repository carrying it, and that the controller regenerates it from the export run.

- [ ] **Step 5: Add the CMake target**

In the root `CMakeLists.txt`, guarded by `OpenMVS_USE_ONNXRUNTIME`, exactly as spec §3.6. It must
**not** be part of `all`. Find how Python is located in this build (`Python3_EXECUTABLE` or a
project-specific variable) before writing the command.

- [ ] **Step 6: Verify**

`python3 -m unittest scripts/python/tests/test_fetch_roma2_model.py` passes. `cmake` re-configures
without error and `roma2-model` appears in `cmake --build make --target help`. Do **not** run the
target — there is nothing published yet.

- [ ] **Step 7: Commit**

```bash
git add scripts/fetch_roma2_model.py scripts/python/tests/test_fetch_roma2_model.py models/roma2/checksums.txt CMakeLists.txt
git commit -m "build: fetch the RoMa v2 model on demand, pinned by a checksum the repository carries"
```

---

### Task 6: Documentation

**Files:**
- Modify: `docs/design/ROMA2InProcess.md` (new "Scaling" section; every `--roma2-retrieval` mention)
- Modify: `libs/SFM/README.md:188` region (the RoMa v2 paragraph and its flag list)
- Modify: `docs/features_catalog.md` (any `--roma2-retrieval` or `useRetrieval` mention)
- Create: `docs/RoMa2Model.md`

**Interfaces:** consumes the final CLI shape from Tasks 3-5.

- [ ] **Step 1: Sweep for stale text**

```
/usr/bin/grep -rn "roma2-retrieval\|useRetrieval\|UseGlobalDescriptors\|format_version 3" docs/ libs/ apps/ README.md
```
Every hit is either updated or is a deliberate historical note in this plan's own spec. Record the
count before and after.

- [ ] **Step 2: Write the Scaling section**

In `docs/design/ROMA2InProcess.md`, add "Scaling" carrying spec §0.1-0.3 as prose: what a
retrieval-only pass loads and computes and what it does not (the coarse graph is never loaded, the
`layers` residue and why it is 12.5 MB of VRAM rather than time, `value_facets` never read back,
8 KB per image on the bus); the retrieval index's linear memory (41 MB at 5 000 images, 164 MB at
20 000) and why it needs no cache; the Belady slot cache, `--roma2-slots`, 12.5 MB per slot, the
`loads`/`reloads` accounting, and that eviction costs a 43 ms re-describe rather than correctness.
Close with the disk-cache option stated as an option, with the numbers a decision would need.

- [ ] **Step 3: Restate the two selection modes**

`libs/SFM/README.md` — replace the sentence describing `--roma2-retrieval` with the spec §1.1 table,
and drop the flag from the flag list.

- [ ] **Step 4: Write `docs/RoMa2Model.md`**

Obtaining the model: the `roma2-model` target, the script's options, where it lands, the licence
statement (RoMa v2 MIT, DINOv3 under Meta's DINOv3 License, and that running `--roma2` means
accepting it), and the two commands that publish a new bundle for whoever maintains it.

- [ ] **Step 5: Commit**

```bash
git add docs/ libs/SFM/README.md
git commit -m "docs: how the model is obtained, and how the two passes scale"
```

---

## Controller's own work, after the branch is green

Not tasks and not a subagent's — they run tools no implementer has:

1. **The export run** — `export.py` at `FORMAT_VERSION = 1`, `base` preset, fp32, into
   `~/virginia/models/roma2-onnx/roma2onnx-<date>-v1/`, then `export.py check` against the eager
   references. The existing `roma2onnx-20260903-bidir` bundle stays untouched: the campaign's
   recorded runs name it.
2. **`models/roma2/checksums.txt`** — regenerated from that bundle.
3. **The publishing hand-off** — assemble the bundle directory with `README.md` and
   `LICENSE-DINOv3.md`, then give the user the two commands. Do not create the repo or upload.
4. **The campaign report** — the six finished capdensity arms, and the Truck SIFT A/B now running.
