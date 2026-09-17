# Structure from Motion: what was tried and retired

The design notes describe what ships. This record keeps what was measured on the way and did not
ship, with the numbers that decided it and the commit where the code last lived, so that none of it
has to be re-run to be believed or re-invented to be tried again. "Alameda" is the Tanks and Temples
training scene of 1734 images with a COLMAP reference; "SIFT" means the same images matched with
SIFT through the vocabulary tree; pose errors are against that reference after a similarity fit.

## Guided sparse matching of a RoMa2 pair

**The disc** (until 995d642, 2026-09-16). Each described keypoint of A was matched among B's
keypoints inside a disc of two warp cells around the warp's prediction, the winner having to beat
the best keypoint outside the disc by the matcher's ratio. On alameda's 30249 pairs shared with SIFT
the disc found a third more matches per pair than SIFT and lost almost none of SIFT's, but 13.3% of
the components of three or more matched keypoints held one image twice, against 5.3% for the matches
SIFT has too. A disc of 35 px on a 2789 px frame holds 26 rivals on average; the wrong one sits 3-35
px from the right one ALONG the epipolar line (52% within 10 degrees of it, median 7.5 degrees),
where the 4 px epipolar test cannot see it, and the outside reference never looked at it. A further
6.7% of the winners that passed the ratio were then refused by the epipolar test: an impostor OFF
the line had taken the disc from the true candidate. Replaced by the epipolar band (995d642).

**The band without the outside reference** (995d642 asked the reference of a winner alone in its
band only; 2409497 made it a three-way choice, `guidedOutsideReference`; the choice went after
cfb9660, every winner answering). Asked of lone winners only, the band admitted 32.7M matches SIFT
does not have on the shared pairs (947 per pair, against 91 with the disc) at a median Sampson error
of 1.07 px under the SIFT poses (0.54 px with the disc, 0.36 for the matches both have), 6.2% of them
beyond 4 px, and 24% of the winners were refused by the pair's own epipolar test; asked of none, 35.7M
at 1.14 px. Those matches glued three quarters of all keypoints into one component and 76-78% of
the tracks built on them were inconsistent, whatever the track policy. Cause: where the true
keypoint was never detected the winner is a random one of the band's handful, and a handful's second
best is beaten by the ratio a third of the time, where the best of the image's thousands never is.
Asked of every winner (the rule that ships) the band keeps 9.13M of the matches SIFT has (-5.4%) and
adds 3.52M SIFT has not, at 0.50 px median, 1.0% beyond 4 px; impostors 2.3% (disc 6.7%), components
holding an image twice 11.8% (disc 13.3%, SIFT 6.5%), and no wall-time cost (the dense pass 2131 s
against the disc's 2657 s, the whole matching 35m31s against 44m17s).

**The adaptive band width** (`guidedBandResidualFactor`: the half-width as three times the median
epipolar residual of the verdict's inlier cells, floored at the epipolar bar, capped at the band's
length; 995d642 until cfb9660). Cleaner -- impostors 1.2%, components holding an image twice 10.8%,
0.48 px median -- but it loses 22% of the matches SIFT has too (2.94M, 96.7% of them within 4 px) and
costs 210 s more per pass; with the reference asked of lone winners or none it flooded like the
fixed width (38.4M and 48.2M guided-only matches at 1.22 and 1.36 px). Not a default; a wider floor
than the epipolar bar is the untested middle.

**Track policies for a component holding one image twice**, simulated offline on the alameda
matches against the SIFT poses (2026-09-16). The pair-order veto (the builder until cfb9660) keeps
the wrong keypoint in 22.5% of the judgeable conflicts and drops one million links, half of them
matches SIFT has too. Ordering the pairs by weight changes nothing, the pairs already arriving sorted
by composite weight (`ComputePairsWeights`). Merging two keypoints of one image within 3 px as one
feature halves the conflicts that end with neither keypoint kept. Cutting the least-supported link on
the path between the two keypoints, with that merge, keeps the wrong keypoint in 17.7% (disc), 17.2%
(band) and 17.0% (adaptive band) and adds 1.6-2% good observations. Track lengths are identical
across matching variants and policies (median 2, 90th percentile 8, 99th 31-33, mean 4.2). The cut
with the merge ships (cfb9660); reconstructed from the same matches it registers the same 1734
images at the same accuracy as the veto (median rotation error 0.048 against 0.048 degrees, centre
error 0.0098% against 0.0103% of the scene diagonal) in 8.5% less reconstruction time and with 4.9%
fewer points, the builder itself in 29.5 s against 34.0 s over 60.7M observations. The veto remains
for components above `TrackConflictConfig::maxComponentSize` and behind `cut = false`.

## The dense segment of a RoMa2 pair

**The half-cell tolerance** (the tolerance until 8384abf, 2026-09-14; kept behind
`denseEpipolarErrorFactor` 0 until cfb9660). A dense correspondence was held to half a warp cell
(8.7 px on a 2789 px frame with the 160-cell grid), the accuracy the grid itself claims, and a
correspondence admitted there pollutes every track it enters. Held to the matcher's own epipolar bar
(factor 1) alameda registers the same images at the same accuracy.

**The dense two-view gate and the dense supplement** (9c05448 to 5a192dd, 2026-08-31 to 09-02). A
RoMa2 warp validated a pair before its descriptor matching and supplemented a validated pair the
descriptors left weak. The gate's inlier ratio was measured to be the wrong variable (16ed92b); the
design was superseded on 2026-09-03 by the one-pass dense pair matching (bfce5f6), which judges,
matches and fills a pair from one bidirectional warp. The round-1 replacement flags
`--roma2-skip-healthy` and `--roma2-max-replace` (706cbee) and the NPZ import of RoMa2 matches
(`ImportROMA2`, removed at 1380970) belong to the same superseded design.

## The hierarchical merge

**The interleaving gate** (fa20bbb, dropped at 7a051d7 on 2026-09-15). A placement was vetoed when
fewer than 0.8 of the moved block's cameras had one of their own as nearest neighbour afterwards.
Measured on eight captures against the same code with it off, in twelve merge runs it never refused
a placement the other gates let through, and it refused right, heavily supported blocks: on alameda
four blocks of 14 (350 images, every one right against the reference), on the 554 capture a block
right to 0.29 degrees, on OfficeBadLoop a block at 72 supporting cameras and none against. The blocks
a clustering cuts from one capture overlap in space, and a right placement leaves their cameras mixed.

**The cluster size** was 150 images while the seams were measured and is 200 again since 85f2e18
(2026-09-12).

## The camera-triplet filter

Its first threshold rule, a sweep relaxing the threshold until the graph held together (a4b2039 to
32a4a6c), was replaced by the rule that ships: the strictest threshold whose ceiling leaves the graph
in pieces the filter joins. Its default minimum score went from the paper's 0.3 to 0.75 (51126ea) and
back to 0.3 (d67dd9b) within two days. The cutting rule stays behind `--triplet-cut`, the keep mode
being the default since 210ada5. The validation runs, the offline harness and the planning documents
were removed at 128d47f and 1e32426 (2026-09-08).

## The retrieval descriptors

The pooling of the retrieval descriptors on the CPU was superseded by the pooling inside the ONNX
graph (fdf705b) and removed with its fixtures at 68bf9f0 (2026-08-31).

## Development tooling removed after cfb9660 (2026-09-17)

- A scene saved after the hierarchical merge could be given back as the source and resumed at the
  final adjustment (53bcffd), and a `-v 3` run wrote the scene after the merge, after the final
  adjustment, before the image filter, before the final filter and every reconstructed sub-scene
  (`scene_post_merge.sfm`, `scene_post_ba.sfm`, `scene_pre_filter.sfm`, `scene_pre_final_filter.sfm`,
  `scene_block_<i>.sfm`; 53bcffd, 7bf0cb1, 3834399). Every per-block measurement above was read from
  those files. The post-matching `scene_pre_reconstruction.sfm` and the reconstruction from a matched
  `.sfm` stay (`libs/SFM/AGENTS.md`).
- `SceneAnalyzeSFM` (c3ff487 to 6ecf20c), a read-only exporter of a saved scene's tracks,
  observations, pairs, stored matches and images as CSV and a flat binary, which every offline
  measurement above was read from.
- `--export-retrieval-csv` and `ExportRetrievalRankingsCSV` (7ef69b2): the per-image retrieval
  rankings as CSV, and the describe pass it forced under every match mode.
- The per-pair wall clock and the guided matching's summed thread time in the one-pass records
  (15e755c, 995d642); the dense track-length histogram of `BuildTracks`; the per-component report of
  the image filter's cuts (3834399, 85ccdbb; the junction measurement deciding which cut images
  contradict the model stays); the median reprojection error kept on a camera's vote for the log
  (e405745; the loose-bar count deciding a contradiction stays).
