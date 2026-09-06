#!/usr/bin/env python3
"""Score, check and evaluate the camera-triplet view-graph disambiguation of openMVS.

An independent NumPy reimplementation of `libs/SFM/ViewGraphTriplets.cpp`, which itself follows
Algorithm 1 of

    S. M. Manam and V. M. Govindu, "Leveraging Camera Triplets for Efficient and Accurate
    Structure-from-Motion", CVPR 2024, pp. 4959-4968.

The view graph has the images as nodes and the verified pairs as edges, each carrying a strength
``s_ij = n_ij * c_ij``: its epipolar inlier count ``n_ij`` discounted by ``c_ij``, the fraction of
the frame those inliers cover -- the coverage is what tells a doppelganger with more inliers than
the true junction beside it (matches crowded onto one repeated patch) from that junction (matches
spread over the whole overlap), which the count alone cannot. The triplet graph ``G_T`` has the
triangles of that graph as nodes, two of them adjacent iff they share an edge; the edges of ``G``
taking part in the largest connected component of ``G_T`` form ``G_LCT`` (ties broken by the
smallest edge index in the component). Every edge of ``G_LCT`` is scored by the mean over its
triangles of ``s_ij / max_{(k,l) in t} s_kl`` -- a triangle whose three edges all deliver less than
``minYield`` of what pairs at their ray angle typically deliver in this graph contributes 0 to that
mean while still counting in its divisor, since three look-alike copies vouching for one another
are no evidence for any of them. The score is removed only if it falls below

    tau = m * (1 - d_max/|V|) + d_max/|V|,

with ``|V|`` and ``d_max`` the node count and maximum degree **of G_LCT**. Everything else --
including every edge in no triangle at all -- is unscored: it carries no evidence either way, and
the filter keeps it. An edge whose ray angle was never measured, or is zero or negative, plays no
part in any yield envelope and keeps a yield of 1 -- absence of evidence is not evidence of a
deficit.

Subcommands
-----------
``score``   read a pairs CSV (``ImageA,ImageB,NumMatches,Coverage,...,MeanRayAngle,TripletScore``,
            as written by ``CreateStructure --export-pairs-csv``) and write ``triplet_scores.csv``
            with this script's own score per pair plus the threshold for a given m.
``parity``  compare those scores against the ``TripletScore`` column the C++ side wrote into the
            same CSV: the unscored sets must be identical and the maximum absolute difference at
            most the tolerance (1e-5 by default). Exits non-zero when they disagree. Checked
            against two exhaustive exports (Street, 171 pairs; Radcliffe Camera, 22,197 pairs):
            maximum absolute difference 5.140e-07 and 5.287e-07.
``roc``     join the scores with a labels CSV and report AUC plus, at tau(m) for
            m in {0.3, 0.6, 0.75, 0.9}, the precision, recall and kept fraction of the true and
            false edges; writes the full ROC curve as a CSV. An unscored pair is kept at every m,
            so it has no score to rank by: the AUC is reported only over the scored labelled
            pairs, and the kept/precision/recall figures count every unscored labelled pair as kept.

**Labels.** ``--labels`` accepts the output of ``scripts/python/tests/pair_gt_labels.py`` (run it
in ``coverage`` mode, which reads the capture's own Polycam depth maps: ``pair_gt_labels.py
<capture> --mode coverage --pairs <pairs.csv> -o <labels.csv>``) or any CSV carrying a ``label``
column and one of the pair-key column sets ``ImageA/ImageB``, ``stem_a/stem_b`` or ``idA/idB``.
A **plausible** pair counts as a true edge, an **implausible** pair as a false edge, and
**ambiguous** pairs are excluded from every number reported here. Leading ``#`` comment lines are
skipped in every CSV this script reads, and written into the ones it produces.

**Note on the edge test.** The C++ side takes a pair as an edge of the view graph when it has a
stored two-view geometry *and* a positive strength; a CSV cannot record the stored-geometry flag
itself, so this script takes ``NumMatches > 0`` and ``Coverage > 0`` -- together, a positive
strength -- as the proxy for it. ``parity`` is what establishes that the two agree on a real
matched graph -- it fails loudly if they ever do not.

Usage:
    triplet_disambiguation.py score  --pairs pairs.csv -o triplet_scores.csv [-m 0.75] [--min-yield 0.4]
    triplet_disambiguation.py parity --pairs pairs.csv [--tolerance 1e-5] [--min-yield 0.4]
    triplet_disambiguation.py roc    --pairs pairs.csv --labels labels.csv -o roc_curve.csv [--min-yield 0.4]
"""

import argparse
import csv
import os
import sys

import numpy as np


# ---------------------------------------------------------------------------- CSV helpers

def read_csv_rows(path):
    """Rows of a CSV as dicts, skipping the leading '#' comment lines."""
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(line for line in handle if not line.startswith("#")))
    if not rows:
        sys.exit("error: '%s' has no data rows" % path)
    return rows


def pair_key(row, columns):
    """The unordered key of a pair row, as a sorted tuple of the two identifiers."""
    a, b = row[columns[0]], row[columns[1]]
    if columns[0] == "ImageA":  # image paths: join on the file stem, as pair_gt_labels.py does
        a = os.path.splitext(os.path.basename(a))[0]
        b = os.path.splitext(os.path.basename(b))[0]
    return (a, b) if a <= b else (b, a)


def key_columns(row, path):
    for columns in (("ImageA", "ImageB"), ("stem_a", "stem_b"), ("idA", "idB"), ("a", "b")):
        if columns[0] in row and columns[1] in row:
            return columns
    sys.exit("error: '%s' has no pair-key columns (ImageA/ImageB, stem_a/stem_b, idA/idB or a/b)" % path)


# ---------------------------------------------------------------------------- the algorithm

class TripletScores:
    """The per-pair scores of one view graph, and the statistics of the graph they came from."""

    def __init__(self, scores, tau, num_triplets, num_triplet_components, num_scored_pairs,
                 num_nodes, max_degree):
        self.scores = scores                                  # one float per input row, NaN = unscored
        self.tau = tau
        self.num_triplets = num_triplets
        self.num_triplet_components = num_triplet_components
        self.num_scored_pairs = num_scored_pairs
        self.num_nodes = num_nodes                            # |V| of G_LCT
        self.max_degree = max_degree                          # d_max of G_LCT

    def threshold(self, min_score):
        """tau of Eqn. 3 for a given m, in the float arithmetic the C++ side uses."""
        if self.num_nodes == 0:
            return float(min_score)
        ratio = np.float32(self.max_degree) / np.float32(self.num_nodes)
        return float(np.float32(min_score) * (np.float32(1) - ratio) + ratio)

    def summary(self):
        return ("%d triplets in %d components; %d of %d pairs scored; G_LCT has %d nodes, "
                "max degree %d" % (self.num_triplets, self.num_triplet_components,
                                   self.num_scored_pairs, len(self.scores),
                                   self.num_nodes, self.max_degree))


def compute_edge_yields(edge_nodes, edge_inliers, edge_ray_angle):
    """The yield u_e/H(bin) of every edge, capped at 1 (ComputeEdgeYields).

    u_e = n_e / min(K_i, K_j), K_i the largest n over any edge touching image i; H is the 90th
    percentile of u among the edges of each 1-degree bin of `floor(edge_ray_angle)` (bins 89 and
    beyond share the last), among bins holding at least 5 edges, made non-increasing in the angle
    by a suffix maximum and forward-filled from the lowest populated bin upward. An edge whose ray
    angle is not finite, negative or zero takes no part in any bin and keeps a yield of 1; if no
    bin is populated at all, every yield is 1.
    """
    numEdges = len(edge_nodes)
    capacity = {}
    for (a, b), inliers in zip(edge_nodes, edge_inliers):
        if inliers > capacity.get(a, 0.0):
            capacity[a] = inliers
        if inliers > capacity.get(b, 0.0):
            capacity[b] = inliers
    capA = np.asarray([capacity[a] for a, _b in edge_nodes])
    capB = np.asarray([capacity[b] for _a, b in edge_nodes])
    delivered = edge_inliers / np.minimum(capA, capB)  # in (0,1]: capacity >= this edge's own count

    validRay = np.isfinite(edge_ray_angle) & (edge_ray_angle > 0.0)
    binOfEdge = np.minimum(np.floor(edge_ray_angle).astype(np.int64), 89)

    numBins, minEdgesPerBin, percentile = 90, 5, np.float32(0.9)
    envelope = np.full(numBins, -1.0)
    for b in range(numBins):
        values = delivered[validRay & (binOfEdge == b)]
        if values.size < minEdgesPerBin:
            continue
        rank = min(values.size - 1, int(np.float32(values.size) * percentile))
        envelope[b] = np.sort(values)[rank]
    best = -1.0
    for b in range(numBins - 1, -1, -1):  # suffix maximum: non-increasing in the angle
        best = max(best, envelope[b])
        envelope[b] = best
    yields = np.ones(numEdges)
    if best < 0.0:
        return yields  # no bin ever reached 5 edges: no envelope at all
    for b in range(1, numBins):  # forward-fill an empty bin from the one below it
        if envelope[b] < 0.0:
            envelope[b] = envelope[b - 1]
    yields[validRay] = np.minimum(1.0, delivered[validRay] / envelope[binOfEdge[validRay]])
    return yields


def compute_triplet_scores(pairs, min_score, min_yield):
    """Score a view graph given as a list of (nodeA, nodeB, numMatches, coverage, meanRayAngle)
    five-tuples, meanRayAngle in degrees, in that order.

    Returns a TripletScores whose `scores[i]` belongs to `pairs[i]`; NaN marks an unscored pair.
    Duplicate pairs collapse onto one edge weighted by the larger strength and all share its
    score, exactly as the C++ side does.
    """
    scores = np.full(len(pairs), np.nan)

    # 1. the edges: one per unordered node pair whose strength s = numMatches * coverage is
    #    positive (NumMatches > 0 and Coverage > 0); two rows describing the same pair collapse
    #    onto the edge with the larger strength, which also supplies its inlier count and ray angle
    edge_of_row = np.full(len(pairs), -1, dtype=np.int64)
    edge_of_key = {}
    edge_nodes = []       # (a, b) node keys, a <= b
    edge_strength = []    # s_ij of the edge
    edge_inliers = []     # the n_ij that produced that strength
    edge_ray_angle = []   # its mean ray angle, degrees
    for i, (a, b, numMatches, coverage, rayAngle) in enumerate(pairs):
        if a == b:
            continue
        strength = numMatches * coverage
        if strength <= 0.0:
            continue
        key = (a, b) if a <= b else (b, a)
        edge = edge_of_key.get(key)
        if edge is None:
            edge = edge_of_key[key] = len(edge_nodes)
            edge_nodes.append(key)
            edge_strength.append(strength)
            edge_inliers.append(numMatches)
            edge_ray_angle.append(rayAngle)
        elif strength > edge_strength[edge]:
            edge_strength[edge] = strength
            edge_inliers[edge] = numMatches
            edge_ray_angle[edge] = rayAngle
        edge_of_row[i] = edge
    if not edge_nodes:
        return TripletScores(scores, float(min_score), 0, 0, 0, 0, 0)
    numEdges = len(edge_nodes)
    edge_strength = np.asarray(edge_strength, dtype=np.float64)
    edge_inliers = np.asarray(edge_inliers, dtype=np.float64)
    edge_ray_angle = np.asarray(edge_ray_angle, dtype=np.float64)

    # 2. adjacency, sorted per node, so the common neighbours of an edge's two nodes can be found
    #    by merging two sorted lists in one linear pass, exactly as the C++ side does
    adjacency = {}
    for edge, (a, b) in enumerate(edge_nodes):
        adjacency.setdefault(a, []).append((b, edge))
        adjacency.setdefault(b, []).append((a, edge))
    for neighbors in adjacency.values():
        neighbors.sort()

    # 3. the triplets: for every edge (a,b), the common neighbours c ordered after b, so each
    #    triangle (e_ab, e_ac, e_bc) is emitted exactly once. Streamed straight into three parallel
    #    lists -- never a list of triangle objects -- since a capture matched exhaustively puts on
    #    the order of 800,000 triangles on ~20,000 edges.
    triangle_ab, triangle_ac, triangle_bc = [], [], []
    for e, (a, b) in enumerate(edge_nodes):
        adjA, adjB = adjacency[a], adjacency[b]
        ia = ib = 0
        lenA, lenB = len(adjA), len(adjB)
        while ia < lenA and ib < lenB:
            na, ea = adjA[ia]
            nb, eb = adjB[ib]
            if na < nb:
                ia += 1
            elif nb < na:
                ib += 1
            else:
                if na > b:  # na > b (and so > a, since a <= b): the triangle's largest node, visited once
                    triangle_ab.append(e)
                    triangle_ac.append(ea)
                    triangle_bc.append(eb)
                ia += 1
                ib += 1
    numTriplets = len(triangle_ab)
    if numTriplets == 0:
        return TripletScores(scores, float(min_score), 0, 0, 0, 0, 0)
    triangle_ab = np.asarray(triangle_ab, dtype=np.int64)
    triangle_ac = np.asarray(triangle_ac, dtype=np.int64)
    triangle_bc = np.asarray(triangle_bc, dtype=np.int64)

    # 4. components of the triplet graph G_T: two triangles sharing an edge land in the same
    #    component, so this is connected components of the graph on EDGES with an (e_ab,e_ac) and
    #    an (e_ab,e_bc) link per triangle. Computed by vectorised label propagation -- repeatedly
    #    broadcasting the smaller label across every link, then path-compressing by pointer-jumping
    #    -- rather than a union-find walked one triangle at a time, so the ~800,000-triangle graphs
    #    this script is checked against cost a handful of whole-array passes, not a Python loop over
    #    every triangle. The result is deterministic either way: a label only ever falls from an
    #    edge's own index towards a neighbour's, so every component converges on its smallest edge
    #    index -- the same tie-break the C++ union-find's "keep the smaller root" rule gives.
    u = np.concatenate([triangle_ab, triangle_ab])
    v = np.concatenate([triangle_ac, triangle_bc])
    labels = np.arange(numEdges, dtype=np.int64)
    while True:
        newLabels = labels.copy()
        lu, lv = labels[u], labels[v]
        np.minimum.at(newLabels, u, lv)
        np.minimum.at(newLabels, v, lu)
        newLabels = newLabels[newLabels]  # pointer-jumping: flatten one level of the parent chain
        if np.array_equal(newLabels, labels):
            labels = newLabels
            break
        labels = newLabels
    triangleComponent = labels[triangle_ab]  # == labels[triangle_ac] == labels[triangle_bc]
    uniqueRoots, counts = np.unique(triangleComponent, return_counts=True)
    largest = int(uniqueRoots[counts == counts.max()][0])  # ties: the smallest root (roots ascending)
    numTripletComponents = int(uniqueRoots.size)

    # 5. the per-edge mean of s_ij / max_{(k,l) in t} s_kl over the triplets of the largest
    #    component: a triangle whose three edges all yield below minYield is look-alike copies
    #    vouching for one another and contributes 0 to the sum while still counting in the divisor.
    #    Vectorised over the whole triangle array at once -- the per-triangle work, not just the
    #    graph walk that found the triangles.
    numTripletsOfEdge = np.bincount(
        np.concatenate([triangle_ab, triangle_ac, triangle_bc]), minlength=numEdges)
    inLargest = triangleComponent == largest
    if min_yield > 0.0:
        yields = compute_edge_yields(edge_nodes, edge_inliers, edge_ray_angle)
        tripletYield = np.maximum(np.maximum(yields[triangle_ab], yields[triangle_ac]), yields[triangle_bc])
        contributing = inLargest & (tripletYield >= min_yield)
    else:
        contributing = inLargest
    ab, ac, bc = triangle_ab[contributing], triangle_ac[contributing], triangle_bc[contributing]
    maxStrength = np.maximum(np.maximum(edge_strength[ab], edge_strength[ac]), edge_strength[bc])
    scoreSum = np.zeros(numEdges, dtype=np.float64)
    np.add.at(scoreSum, ab, edge_strength[ab] / maxStrength)
    np.add.at(scoreSum, ac, edge_strength[ac] / maxStrength)
    np.add.at(scoreSum, bc, edge_strength[bc] / maxStrength)

    # 6. |V| and d_max of G_LCT: only edges taking part in a triplet of the largest component (an
    #    edge of that component takes part in no triplet outside it, so numTripletsOfEdge already
    #    counts only its G_LCT triangles)
    isScoredEdge = (numTripletsOfEdge > 0) & (labels == largest)
    degree_of_node = {}
    max_degree = 0
    for e in np.flatnonzero(isScoredEdge):
        a, b = edge_nodes[e]
        for node in (a, b):
            degree = degree_of_node[node] = degree_of_node.get(node, 0) + 1
            if degree > max_degree:
                max_degree = degree
    num_nodes = len(degree_of_node)

    # 7. spread the edge scores back onto the input rows
    edge_score = np.full(numEdges, np.nan)
    edge_score[isScoredEdge] = scoreSum[isScoredEdge] / numTripletsOfEdge[isScoredEdge]
    scored = edge_of_row >= 0
    scores[scored] = edge_score[edge_of_row[scored]]
    result = TripletScores(scores, 0.0, numTriplets, numTripletComponents,
                           int(np.count_nonzero(~np.isnan(scores))), num_nodes, max_degree)
    result.tau = result.threshold(min_score)
    return result


def load_view_graph(path):
    """The (rows, pairs, cpp_scores) of a pairs CSV written by --export-pairs-csv."""
    rows = read_csv_rows(path)
    columns = key_columns(rows[0], path)
    for column in ("NumMatches", "Coverage", "MeanRayAngle"):
        if column not in rows[0]:
            sys.exit("error: '%s' has no %s column" % (path, column))
    pairs, cpp_scores = [], []
    for row in rows:
        key = pair_key(row, columns)
        pairs.append((key[0], key[1], float(row["NumMatches"]), float(row["Coverage"]),
                      float(row["MeanRayAngle"])))
        cell = (row.get("TripletScore") or "").strip()
        cpp_scores.append(float(cell) if cell else np.nan)
    return rows, pairs, np.asarray(cpp_scores)


# ---------------------------------------------------------------------------- subcommands

def cmd_score(args):
    rows, pairs, _ = load_view_graph(args.pairs)
    result = compute_triplet_scores(pairs, args.min_score, args.min_yield)
    with open(args.output, "w", newline="") as handle:
        handle.write("# triplet_disambiguation.py score, m=%g, min_yield=%g, tau=%.6f\n"
                     % (args.min_score, args.min_yield, result.tau))
        handle.write("# %s\n" % result.summary())
        handle.write("# TripletScore empty = unscored (no triplet in the largest triplet-graph component)\n")
        writer = csv.writer(handle)
        writer.writerow(["ImageA", "ImageB", "NumMatches", "Coverage", "TripletScore", "Kept"])
        for (a, b, numMatches, coverage, _rayAngle), score in zip(pairs, result.scores):
            # unscored (no evidence either way) is kept; a scored pair is kept iff it reaches tau
            kept = True if np.isnan(score) else bool(score >= result.tau)
            writer.writerow([a, b, int(numMatches), coverage, "" if np.isnan(score) else "%.6f" % score,
                             int(kept)])
    kept = int(np.count_nonzero(np.isnan(result.scores) | (result.scores >= result.tau)))
    print("%s\ntau %.6f at m %g, min_yield %g; kept %d of %d pairs -> '%s'"
          % (result.summary(), result.tau, args.min_score, args.min_yield, kept, len(pairs), args.output))
    return 0


def cmd_parity(args):
    rows, pairs, cpp_scores = load_view_graph(args.pairs)
    if np.all(np.isnan(cpp_scores)):
        sys.exit("error: '%s' has no TripletScore values (exported by an older build?)" % args.pairs)
    result = compute_triplet_scores(pairs, 0.0, args.min_yield)
    py_unscored, cpp_unscored = np.isnan(result.scores), np.isnan(cpp_scores)
    disagree = np.flatnonzero(py_unscored != cpp_unscored)
    both = ~py_unscored & ~cpp_unscored
    max_diff = float(np.max(np.abs(result.scores[both] - cpp_scores[both]))) if np.any(both) else 0.0
    print("%s\n%d pairs, %d scored by both; max abs difference %.3e (tolerance %.0e)"
          % (result.summary(), len(pairs), int(np.count_nonzero(both)), max_diff, args.tolerance))
    if disagree.size:
        print("FAILED: %d pairs disagree on being scored, first at row %d (%s)"
              % (disagree.size, disagree[0], pairs[disagree[0]][:2]))
        return 1
    if max_diff > args.tolerance:
        worst = int(np.argmax(np.abs(np.where(both, result.scores - cpp_scores, 0.0))))
        print("FAILED: pair %s scored %.8f here against %.8f in C++"
              % (pairs[worst][:2], result.scores[worst], cpp_scores[worst]))
        return 1
    print("PASSED: the Python and C++ scores agree")
    return 0


def roc_auc(scores, is_true_edge):
    """Area under the ROC curve by the rank (Mann-Whitney U) identity, ties averaged."""
    numTrue, numFalse = int(np.count_nonzero(is_true_edge)), int(np.count_nonzero(~is_true_edge))
    if numTrue == 0 or numFalse == 0:
        return float("nan")
    order = np.argsort(scores, kind="mergesort")
    ranks = np.empty(len(scores))
    sortedScores = scores[order]
    start = 0
    for i in range(1, len(sortedScores) + 1):  # average the ranks within each tie group
        if i == len(sortedScores) or sortedScores[i] != sortedScores[start]:
            ranks[order[start:i]] = (start + i + 1) / 2.0
            start = i
    return float((np.sum(ranks[is_true_edge]) - numTrue * (numTrue + 1) / 2.0) / (numTrue * numFalse))


def cmd_roc(args):
    rows, pairs, _ = load_view_graph(args.pairs)
    result = compute_triplet_scores(pairs, 0.0, args.min_yield)

    labelRows = read_csv_rows(args.labels)
    labelColumns = key_columns(labelRows[0], args.labels)
    if "label" not in labelRows[0]:
        sys.exit("error: '%s' has no label column" % args.labels)
    labelOfKey = {pair_key(row, labelColumns): row["label"].strip() for row in labelRows}

    # a plausible pair is a true edge, an implausible pair a false edge, ambiguous pairs are dropped
    scores, isTrue, numAmbiguous, numUnlabelled = [], [], 0, 0
    for (a, b, _numMatches, _coverage, _rayAngle), score in zip(pairs, result.scores):
        label = labelOfKey.get((a, b) if a <= b else (b, a))
        if label is None:
            numUnlabelled += 1
            continue
        if label not in ("plausible", "implausible"):
            numAmbiguous += 1
            continue
        scores.append(score)
        isTrue.append(label == "plausible")
    scores, isTrue = np.asarray(scores), np.asarray(isTrue, dtype=bool)
    if scores.size == 0:
        sys.exit("error: no pair of '%s' is labelled in '%s'" % (args.pairs, args.labels))
    scored = ~np.isnan(scores)
    # an unscored pair carries no score and no threshold ever removes it: it is kept unconditionally,
    # at every m, so it has no position in a score-ranked sweep at all. The AUC below is therefore
    # reported only over the scored pairs -- the only ones the score actually ranks; see the
    # module docstring's note on why an all-pairs AUC would be silently meaningless now.

    print("%s" % result.summary())
    print("%d labelled pairs (%d true, %d false), %d ambiguous and %d unlabelled dropped; "
          "%d of the labelled pairs are scored"
          % (scores.size, int(np.count_nonzero(isTrue)), int(np.count_nonzero(~isTrue)),
             numAmbiguous, numUnlabelled, int(np.count_nonzero(scored))))
    # the largest-triplet-graph-component step alone leaves these unscored, before any threshold
    # applies -- and, being unscored, they are kept regardless of m
    print("the %d unscored labelled pairs are %d true and %d false (always kept)"
          % (int(np.count_nonzero(~scored)), int(np.count_nonzero(~scored & isTrue)),
             int(np.count_nonzero(~scored & ~isTrue))))
    auc = roc_auc(scores[scored], isTrue[scored])
    print("AUC %.4f over the %d scored labelled pairs (unscored pairs are always kept and carry "
          "no rank)" % (auc, int(np.count_nonzero(scored))))

    numTrue, numFalse = int(np.count_nonzero(isTrue)), int(np.count_nonzero(~isTrue))
    for m in args.min_scores:
        tau = result.threshold(m)
        kept = ~scored | (scores >= tau)  # unscored: no evidence either way, always kept
        keptTrue, keptFalse = int(np.count_nonzero(kept & isTrue)), int(np.count_nonzero(kept & ~isTrue))
        precision = keptTrue / max(keptTrue + keptFalse, 1)
        print("m %.2f: tau %.4f | kept %d/%d labelled (%.1f%%) | true kept %d/%d (%.1f%%) | "
              "false kept %d/%d (%.1f%%) | precision %.4f recall %.4f"
              % (m, tau, keptTrue + keptFalse, scores.size, 100.0 * (keptTrue + keptFalse) / scores.size,
                 keptTrue, numTrue, 100.0 * keptTrue / max(numTrue, 1),
                 keptFalse, numFalse, 100.0 * keptFalse / max(numFalse, 1),
                 precision, keptTrue / max(numTrue, 1)))

    with open(args.output, "w", newline="") as handle:
        handle.write("# triplet_disambiguation.py roc, pairs '%s', labels '%s'\n"
                     % (os.path.basename(args.pairs), os.path.basename(args.labels)))
        handle.write("# %s\n" % result.summary())
        handle.write("# plausible = true edge, implausible = false edge, ambiguous excluded; "
                     "an unscored pair carries no score and is kept at every threshold below\n")
        handle.write("# AUC %.6f over the scored labelled pairs (unscored pairs are always kept "
                     "and carry no rank)\n" % auc)
        handle.write("# the %d unscored labelled pairs are %d true and %d false\n"
                     % (int(np.count_nonzero(~scored)), int(np.count_nonzero(~scored & isTrue)),
                        int(np.count_nonzero(~scored & ~isTrue))))
        writer = csv.writer(handle)
        writer.writerow(["threshold", "kept", "kept_true", "kept_false", "tpr", "fpr", "precision"])
        # sweep only the scored labelled values: an unscored pair is kept at every one of them, so
        # it never changes which threshold moves the kept set and needs no entry of its own
        scoredValues = scores[scored]
        thresholds = np.unique(scoredValues) if scoredValues.size else np.array([-np.inf])
        for threshold in thresholds:
            kept = ~scored | (scores >= threshold)
            keptTrue, keptFalse = int(np.count_nonzero(kept & isTrue)), int(np.count_nonzero(kept & ~isTrue))
            writer.writerow(["%.6f" % threshold, keptTrue + keptFalse, keptTrue, keptFalse,
                             "%.6f" % (keptTrue / max(numTrue, 1)),
                             "%.6f" % (keptFalse / max(numFalse, 1)),
                             "%.6f" % (keptTrue / max(keptTrue + keptFalse, 1))])
    print("curve -> '%s'" % args.output)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest="command", required=True)

    scoreParser = subparsers.add_parser("score", help="score a pairs CSV with this script's own implementation")
    scoreParser.add_argument("--pairs", required=True, help="pairs CSV (ImageA,ImageB,NumMatches,Coverage,...,MeanRayAngle,TripletScore)")
    scoreParser.add_argument("-o", "--output", required=True, help="output CSV")
    scoreParser.add_argument("-m", "--min-score", type=float, default=0.3, help="the paper's minimum edge score m (default 0.3, the shipped one)")
    scoreParser.add_argument("--min-yield", type=float, default=0.4,
                             help="a triangle whose three edges all yield less than this fraction of what pairs "
                                  "at their ray angle typically deliver gives its edges no evidence (default 0.4; 0 disables it)")
    scoreParser.set_defaults(func=cmd_score)

    parityParser = subparsers.add_parser("parity", help="check the scores against the CSV's own TripletScore column")
    parityParser.add_argument("--pairs", required=True, help="pairs CSV carrying a TripletScore column")
    parityParser.add_argument("--tolerance", type=float, default=1e-5, help="maximum allowed absolute difference (default 1e-5)")
    parityParser.add_argument("--min-yield", type=float, default=0.4,
                              help="must match the minYield the CSV's TripletScore column was computed with (default 0.4)")
    parityParser.set_defaults(func=cmd_parity)

    rocParser = subparsers.add_parser("roc", help="evaluate the scores against ground-truth pair labels")
    rocParser.add_argument("--pairs", required=True, help="pairs CSV (ImageA,ImageB,NumMatches,Coverage,...,MeanRayAngle,TripletScore)")
    rocParser.add_argument("--labels", required=True,
                           help="labels CSV with a label column of plausible/implausible/ambiguous "
                                "(pair_gt_labels.py in coverage mode); a plausible pair is a true edge, "
                                "an implausible pair a false edge, ambiguous pairs are excluded")
    rocParser.add_argument("-o", "--output", required=True, help="output ROC curve CSV")
    rocParser.add_argument("--min-scores", type=float, nargs="+", default=[0.3, 0.6, 0.9, 0.75],
                           help="the values of m to report an operating point for (default 0.3 0.6 0.9 0.75)")
    rocParser.add_argument("--min-yield", type=float, default=0.4,
                           help="a triangle whose three edges all yield less than this fraction of what pairs "
                                "at their ray angle typically deliver gives its edges no evidence (default 0.4; 0 disables it)")
    rocParser.set_defaults(func=cmd_roc)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
