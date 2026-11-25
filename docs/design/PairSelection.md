# Pair Selection

## Purpose and scope

Matching all pairs of N images costs O(N^2) matching and verification calls. `PairsMatcher::Match` first chooses
the candidate pairs by one of the match modes of `--match-mode`, matches and verifies them, and for the selective modes
then spends a held-back share of the pair budget on pairs suggested by the verified result (verification feedback).
This document covers the modes, the sequential prior, the budget and the feedback. How a candidate is verified is in
`TwoViewVerificationAndLens.md`; the dense matcher that can replace the descriptor round is in `DenseMatching.md`; the
view-graph weight that ranks the matched pairs is in `PairWeight.md`.

Source: `PairsMatcher.h/cpp` (`Match`, `CollectVocabularyPairs`, `CollectRetrievalPairs`, `CollectFusedRetrievalPairs`,
`CollectKnownPosePairs`, `CollectVerificationFeedbackPairs`, `AddSequentialPairs`, `AddNeighbourPairs`, `PreMatch`),
`SequenceNeighbours.h/cpp`, `VocabularyTree.h/cpp`, `GlobalDescriptors.h/cpp`.

## Match modes

| `--match-mode` | `MatchConfig::MatchMode` | candidates |
|---|---|---|
| -1 | SKIP | no matching: the saved scene's own matches are used; valid only when the loaded input is a saved scene (`.sfm`) whose pairs hold matches (a `--match-images-only` run or ExtractKeyframes stores them): `CreateStructure` refuses it at start for any other source or with a `--dense-matcher` other than `none`, `Scene::Reconstruct` refuses a scene in which no pair holds a match before any work, and `Scene::MatchPairs` then only refits the missing relative poses and marks the scene matched |
| 0 | EXHAUSTIVE | every pair |
| 1 (default) | VOCABULARY | pairs ranked by a vocabulary tree over the local descriptors |
| 2 | SEQUENTIAL | consecutive images |
| 3 | KNOWN_POSES | pairs chosen geometrically from imported camera poses |
| 4 | RETRIEVAL | pairs ranked by the global descriptors of a dense matcher |

The choice of mode alone decides the ranking backend, with no crossover: VOCABULARY always ranks with the vocabulary
tree, RETRIEVAL always with the global descriptors, and a missing descriptor in RETRIEVAL is an error, never a fallback.
VOCABULARY on fewer than `maxPairsPerImage * 6/5` images (60 with the default) is remapped to EXHAUSTIVE. A command line
that imports camera poses and does not pass `--match-mode` selects KNOWN_POSES.

### EXHAUSTIVE and SEQUENTIAL

EXHAUSTIVE enumerates all N(N-1)/2 pairs; it is practical below about a hundred images. SEQUENTIAL proposes, for each
image, the next `--match-sequence-overlap` images in import order (default 3). When there are at least twice as many
images as the overlap, the sequence wraps around the end to the start so the loop of a closed trajectory is matched;
the diametrically opposite pair is emitted once. Images import in sorted numeric-stem order (`Scene::Import`), so for
a video or hand-held capture the index order is the capture order.

### VOCABULARY and RETRIEVAL (fused ranking)

Both modes rank candidates per image, then share one selection (`CollectFusedRetrievalPairs`):

1. **Rank.** For every image, the ranked list of the most similar images, `queryDepth = min(max(4 * topK, 100), N-1)`
   deep (`QueryRetrieval`). VOCABULARY queries a vocabulary tree built on demand from at most
   `maxDescriptorsPerImage` descriptors per image (1000 for SIFT and SIFTGPU, 2000 for AKAZE and ORB), a hierarchical
   k-means tree with 10 children per node and 6 levels (about 10^6 words), 10 k-means iterations, seed 42, over
   quantized float descriptors by L2 or binary descriptors by Hamming distance. RETRIEVAL queries the cosine index over
   the stored `Image::globalDescriptor` (a single matrix-vector product and a partial sort per query).
2. **Fuse.** Every pair retrieved by either endpoint scores the sum over its two directed lists of `1 / (10 + rank)`:
   symmetric reciprocal-rank fusion, so a pair both images retrieve early outranks a pair only one image rates high, and
   the rank-based score is immune to the drift of raw similarity scales.
3. **Mutual agreement.** A pair is kept only if it is in the fused top-K list of both endpoints, which suppresses the
   one-sided, mostly false tail of each list.
4. **Bridge.** The connected components of the selected graph are joined by the highest-scoring cross-component pairs
   (Kruskal over all retrieved pairs, the union-find seeded with the mutual pairs), so a sparse selection cannot split
   the view graph.

The fused scores are kept for the feedback round.

### KNOWN_POSES

Pairs are chosen from the poses the import gave (`ImportAndCameras.md`), without descriptors:

1. The scene scale is the median nearest-neighbour distance between posed camera centres, so scores do not depend on the
   units of the poses.
2. A pair scores `2 b o / (b^2 + o^2) * (1 - angle / 75 deg)`, with `b` the baseline in scene-scale units, `o = 2` the
   preferred baseline and `angle` the angle between the optical axes. The baseline term peaks at `o` and decays
   symmetrically on a log scale towards no parallax and no overlap; it is a ranking preference, not a rejection
   criterion, because the distance at which two views overlap varies by orders of magnitude between close-range and
   aerial captures. A pair whose optical axes diverge by more than 75 degrees is rejected. There is deliberately no
   camera-centre cheirality test: in an orbit capture neighbouring centres lie tangentially to the view direction and in
   a nadir capture perpendicularly to it, so the strongest overlapping pairs would fail it.
3. Each image keeps its best `topK` posed partners; a pair is kept if both endpoints list it (mutual agreement).
4. Every posed image also keeps its 2 nearest posed cameras regardless of the view-angle gate: under occlusion (an
   indoor camera turning back at a corridor's end) all top partners can exceed the gate.
5. Remaining components are bridged by Boruvka rounds over a two-tier score: any pair that passes the gate outranks every
   gated one, and the gated tier prefers the nearest cameras.
6. If the poses cover only part of the images, the vocabulary tree adds the retrieved candidates of every unposed image
   (always the vocabulary tree, even in a run that carries global descriptors) so the reconstruction tail can resect
   them; if retrieval is unavailable, every pair touching an unposed image is matched.

With fewer than two posed images, or all centres coincident, the mode matches exhaustively.

### Sequential prior

VOCABULARY and RETRIEVAL rank by appearance with no notion of capture order, and on a video or hand-held walk they
leave out a share of the truly overlapping consecutive pairs, exactly the ones that hold an interior's matched graph
together end to end. `AddSequentialPairs` adds, unconditionally, for every image its next `--match-sequence-overlap`
neighbours in import order (`k = 1..overlap`, no wrap-around), skipping pairs already present; 0 turns it off. It is
added before the snapshot the feedback round attempts against, so the feedback round never re-proposes them. It does not
apply to the other modes, nor with `--image-sequence`, where the neighbour pairs take its place (the prior would also add
the pairs across a name break).

### Sequence neighbours

`--image-sequence` declares that the images, in import order, are frames of a sequence that may be interrupted
(several takes, a capture stopped and resumed). Two images are neighbours (`SequenceNeighbours::AreNeighbours`) when
they are at most `--match-sequence-overlap` positions apart and no name break lies between them. A name is the
directory and the stem of the file name (the extension is ignored); its number is the trailing decimal digits of the
stem (none, or more than 18 digits: no number) and its prefix is the rest. `ComputeNameBreaks` puts a break between two
adjacent names that both have a number when the prefixes differ (another directory included), or the number does not
rise, or it rises by more than `--sequence-max-gap` times the median rise over the adjacent same-prefix rising pairs
of the input (with no such pair there is no gap rule). A name without a number makes no break with either neighbour.
The definition is shared by everything that reads the capture as a sequence.

With the option the neighbour pairs are always matched. In every mode that matches (all but SKIP), `Match` appends the
neighbour pairs the mode did not pick (`AddNeighbourPairs`) before the snapshot the feedback round attempts against, in
place of the sequential prior of VOCABULARY and RETRIEVAL; the other candidates of a mode are unchanged (SEQUENTIAL keeps
its wrap-around pairs, which are no neighbours, and normally lacks none). The pre-match filter (`PreMatch`) never removes a
neighbour pair, though it may still store the geometry it found for one; a neighbour pair the scene already holds verified
leaves the list like any other, being already matched. A VOCABULARY or RETRIEVAL selection that comes out empty (its error
is still logged) does not end the stage with the option: the neighbour pairs are matched alone, and `Match` stops only
when there are none either. A neighbour pair goes through the normal matching and verification and
is stored only if it verifies. Without the option nothing of this runs: every mode's candidates are those described above.

The view-graph filter reads the same neighbours: the contradiction cut solves the chain of the verified neighbour pairs,
vetoes the pairs that contradict it and removes none of its links (`ContradictionCut.md`, "The chain of an image
sequence").

## Budget and verification feedback

`--vocab-max-pairs` (`maxPairsPerImage`, default 50) is the target number of pairs per image for VOCABULARY, RETRIEVAL
and KNOWN_POSES, a total of `maxPairsPerImage * N / 2`. The per-image list length `topK` is inflated in single-round
matching (`8/5` of the target for the ranked modes, `4/3` for KNOWN_POSES) because the mutual-agreement rule is stricter
than a one-sided union, so that it selects about the configured volume.

**Verification feedback** (`--match-verification-feedback`, default on) runs for VOCABULARY, RETRIEVAL and KNOWN_POSES
when `maxPairsPerImage >= 10` and the scene has at least 3 candidates to propose among (posed images, for
KNOWN_POSES). Matching then runs in two rounds:

1. The first round uses uninflated lists at 80% of the target (`topK = 4/5 * maxPairsPerImage`) and matches them.
2. The budget left, up to `maxPairsPerImage * N / 2` attempted pairs in all, is invested in pairs suggested by the
   verified pairs of round 1 (a pair is verified when it has matches and, with verification on, a geometric
   verification). KNOWN_POSES closes the triangles of the verified graph: two images sharing verified neighbours likely
   overlap, ranked by the number of common neighbours and then by the pose score. RETRIEVAL and VOCABULARY propagate
   each verified pair (a, b) to the top 5 retrieval candidates c of each endpoint, scoring the proposal
   `fused(a, c) + fused(b, c)`. Proposals are taken by votes then score until the budget is spent.
3. Images with the weakest verified connectivity refill any remaining budget from their next best-ranked first-round
   candidates, at most 2 pairs per image.

The second round runs the same matching round as the first (the descriptor round, or the dense one pass).

## What happens to a selected pair

With the descriptor flow (when `--dense-match` is off), `MatchPairsBatch` orders the pairs to group those sharing a first
image, matches descriptors, and verifies:

- Matching is a k-NN (k = 2) search on FLANN (LSH for binary, KD-tree for float descriptors) or brute force when
  `crossCheck` is on, one matcher per thread. A match must satisfy the absolute distance bound `matchDistance` (100
  AKAZE, 64 ORB, none for SIFT) and the ratio `best < matchRatio * second` (0.9 for AKAZE and ORB, 0.8 for SIFT and
  SIFTGPU). The FLANN L2 index reports squared distances, so the descriptor path's ratio is a true-norm ratio of
  `sqrt(matchRatio)`; the guided matching of the dense flow applies the ratio to true norms.
- An image with fewer descriptors than `minMatches` matches nothing, and a pair with fewer matches than `minMatches` is
  dropped before verification.
- `GeometricFilter` verifies the pair (`TwoViewVerificationAndLens.md`). With `--dense-match`, the whole round is
  instead the dense one pass of `DenseMatching.md`.
- Afterwards the descriptors are released (`releaseDescriptors`), redundant keypoints within 0.1 px are filtered (this is
  where a dense keypoint appended by two pairs of one image becomes one point and a track can exceed two views), and the
  pair weights are computed.

## Parameters and defaults

| parameter | default | effect |
|---|---|---|
| `--match-mode` | 1 (3 when poses were imported and the option is not given) | the mode |
| `--match-sequence-overlap` | 3 | images ahead matched by SEQUENTIAL; sequential prior of VOCABULARY and RETRIEVAL; with `--image-sequence` the farthest neighbours; 0 = no prior (and no neighbours) |
| `--image-sequence` | off | the images in import order are frames of a possibly interrupted sequence, whose neighbour pairs every mode matches (see Sequence neighbours) and whose chain the contradiction cut trusts (`ContradictionCut.md`) |
| `--sequence-max-gap` | 10 | with `--image-sequence`: a frame number rising by more than this many median rises is a break; refused without `--image-sequence` |
| `--sequence-min-covisibility` | 0.75 | with `--image-sequence`: a neighbour pair carrying a covisibility below this is no chain link; in [0,1], refused without `--image-sequence` |
| `--vocab-max-pairs` | 50 | target pairs per image (VOCABULARY, RETRIEVAL, KNOWN_POSES) |
| `--match-verification-feedback` | true | two-round matching, active when `--vocab-max-pairs >= 10` |
| `maxDescriptorsPerImage` | 1000 SIFT/SIFTGPU, 2000 AKAZE/ORB | descriptors per image the vocabulary tree uses |
| `minFeatureDistance` | 0 (off) | `MatchConfig` only, no command-line option: the minimum pixel distance between the two keypoints of a match; above 0 `GeometricFilter` refuses the pair by name, because skipping matches there desynchronises the RANSAC inlier mask from the match list |
| `preMatchThreshold` | 0 (off) | `MatchConfig` only: a pre-matching filter on the vocabulary tree's top descriptors, with no command-line option |
| reciprocal-rank constant | 10 | fusion damping, constant |
| view-angle gate, preferred baseline | 75 deg, 2 scene units | KNOWN_POSES score, constants |
| nearest-camera floor, feedback propagation | 2 cameras, top 5 | KNOWN_POSES floor; feedback propagation depth, constants |

## Invariants

- Candidates are unordered pairs listed once (`MakePairIdx` puts the smaller index first); the dense one pass relies on it.
- A requested backend never degrades silently into another ranking.
- The feedback round proposes only pairs not attempted in round 1, including pairs the dense verdict rejected.
- With `--image-sequence` every neighbour pair is attempted in every mode that matches, and none across a name break is
  added by the option.
- Round 2 is never run when the feedback budget is spent, when fewer than 3 images are eligible, or when no verified pair
  exists; KNOWN_POSES that fell back to exhaustive matching skips it.

## Rejected alternatives

- A camera-centre cheirality (mutual frustum) test for KNOWN_POSES: it discards the strongest overlapping pairs in orbit
  and nadir captures.
- Treating baseline as a rejection criterion: only the view angle rejects; the baseline only ranks.
- One-sided top-K union instead of mutual agreement: its tail wastes the matching budget at small `maxPairsPerImage`.
- Ranking the KNOWN_POSES unposed images by the global descriptors when a run carries them: the mode's meaning is fixed
  to the vocabulary tree.

## Open items

- `preMatchThreshold` and `PreMatch` are reachable only from the library, not from the command line.
