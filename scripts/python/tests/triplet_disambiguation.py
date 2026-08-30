#!/usr/bin/env python3
"""Score, check and evaluate the camera-triplet view-graph disambiguation of openMVS.

An independent NumPy reimplementation of `libs/SFM/ViewGraphTriplets.cpp`, which itself follows
Algorithm 1 of

    S. M. Manam and V. M. Govindu, "Leveraging Camera Triplets for Efficient and Accurate
    Structure-from-Motion", CVPR 2024, pp. 4959-4968.

The view graph has the images as nodes and the verified pairs as edges, each carrying its
epipolar inlier count ``n_ij``. The triplet graph ``G_T`` has the triangles of that graph as
nodes, two of them adjacent iff they share an edge; the edges of ``G`` taking part in the largest
connected component of ``G_T`` form ``G_LCT``. Every edge of ``G_LCT`` is scored by the mean over
its triangles of ``n_ij / max_{(k,l) in t} n_kl``, and kept iff that score reaches

    tau = m * (1 - d_max/|V|) + d_max/|V|,

with ``|V|`` and ``d_max`` the node count and maximum degree **of G_LCT**. Everything else --
including every edge in no triangle at all -- is unscored, and the filter removes it.

Subcommands
-----------
``score``   read a pairs CSV (``ImageA,ImageB,NumMatches,...``, as written by
            ``CreateStructure --export-pairs-csv``) and write ``triplet_scores.csv`` with this
            script's own score per pair plus the threshold for a given m.
``parity``  compare those scores against the ``TripletScore`` column the C++ side wrote into the
            same CSV: the unscored sets must be identical and the maximum absolute difference at
            most the tolerance (1e-5 by default). Exits non-zero when they disagree.
``roc``     join the scores with a labels CSV and report AUC plus, at tau(m) for
            m in {0.3, 0.6, 0.9}, the precision, recall and kept fraction of the true and false
            edges; writes the full ROC curve as a CSV.

**Labels.** ``--labels`` accepts the output of ``scripts/python/tests/pair_gt_labels.py`` (run it
in ``coverage`` mode, which reads the capture's own Polycam depth maps: ``pair_gt_labels.py
<capture> --mode coverage --pairs <pairs.csv> -o <labels.csv>``) or any CSV carrying a ``label``
column and one of the pair-key column sets ``ImageA/ImageB``, ``stem_a/stem_b`` or ``idA/idB``.
A **plausible** pair counts as a true edge, an **implausible** pair as a false edge, and
**ambiguous** pairs are excluded from every number reported here. Leading ``#`` comment lines are
skipped in every CSV this script reads, and written into the ones it produces.

**Note on the edge test.** The C++ side takes a pair as an edge of the view graph when it has a
stored two-view geometry *and* at least one inlier; a CSV records only the inlier count, so this
script uses ``NumMatches > 0`` alone. ``parity`` is what establishes that the two agree on a real
matched graph -- it fails loudly if they ever do not.

Usage:
    triplet_disambiguation.py score  --pairs pairs.csv -o triplet_scores.csv [-m 0.6]
    triplet_disambiguation.py parity --pairs pairs.csv [--tolerance 1e-5]
    triplet_disambiguation.py roc    --pairs pairs.csv --labels labels.csv -o roc_curve.csv
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


def compute_triplet_scores(pairs, min_score):
    """Score a view graph given as a list of (nodeA, nodeB, inliers) triples, in that order.

    Returns a TripletScores whose `scores[i]` belongs to `pairs[i]`; NaN marks an unscored pair.
    Duplicate pairs collapse onto one edge weighted by the strongest of them and all share its
    score, exactly as the C++ side does.
    """
    scores = np.full(len(pairs), np.nan)

    # 1. the edges: one per unordered node pair with at least one inlier
    edge_of_row = np.full(len(pairs), -1, dtype=np.int64)
    edge_of_key = {}
    edge_nodes, edge_inliers = [], []
    for i, (a, b, inliers) in enumerate(pairs):
        if inliers <= 0 or a == b:
            continue
        key = (a, b) if a <= b else (b, a)
        edge = edge_of_key.get(key)
        if edge is None:
            edge = edge_of_key[key] = len(edge_nodes)
            edge_nodes.append(key)
            edge_inliers.append(inliers)
        elif inliers > edge_inliers[edge]:
            edge_inliers[edge] = inliers
        edge_of_row[i] = edge
    if not edge_nodes:
        return TripletScores(scores, float(min_score), 0, 0, 0, 0, 0)
    edge_inliers = np.asarray(edge_inliers, dtype=np.float64)

    # 2. adjacency, keyed by node
    adjacency = {}
    for edge, (a, b) in enumerate(edge_nodes):
        adjacency.setdefault(a, set()).add(b)
        adjacency.setdefault(b, set()).add(a)

    # 3. the triplets: for every edge (a,b), the common neighbours c ordered after b, so each
    #    triangle is emitted exactly once
    triplets = []
    for edge, (a, b) in enumerate(edge_nodes):
        # sorted(): the triplet order, and with it the union-find roots and the largest-component
        # tie-break, must not depend on Python's per-process set iteration order
        for c in sorted(adjacency[a] & adjacency[b]):
            if c > b and c > a:
                triplets.append((edge_of_key[(a, c) if a <= c else (c, a)],
                                 edge_of_key[(b, c) if b <= c else (c, b)],
                                 edge))
    if not triplets:
        return TripletScores(scores, float(min_score), 0, 0, 0, 0, 0)

    # 4. components of the triplet graph: every triplet is merged with the first triplet seen on
    #    each of its three edges, which links all the triplets sharing that edge
    parents = list(range(len(triplets)))

    def find(x):
        while parents[x] != x:
            parents[x] = parents[parents[x]]
            x = parents[x]
        return x

    first_triplet_of_edge = {}
    for t, edges in enumerate(triplets):
        for edge in edges:
            other = first_triplet_of_edge.setdefault(edge, t)
            if other != t:
                ra, rb = find(t), find(other)
                if ra != rb:
                    parents[max(ra, rb)] = min(ra, rb)
    size_of_component = {}
    for t in range(len(triplets)):
        root = find(t)
        size_of_component[root] = size_of_component.get(root, 0) + 1
    largest = min(size_of_component, key=lambda root: (-size_of_component[root], root))

    # 5. the per-edge mean of q^t_ij over the triplets of the largest component
    score_sum = np.zeros(len(edge_nodes))
    triplet_count = np.zeros(len(edge_nodes), dtype=np.int64)
    for t, edges in enumerate(triplets):
        if find(t) != largest:
            continue
        max_inliers = max(edge_inliers[e] for e in edges)
        for edge in edges:
            score_sum[edge] += edge_inliers[edge] / max_inliers
            triplet_count[edge] += 1

    # 6. |V| and d_max of G_LCT, and the threshold of Eqn. 3
    degree_of_node = {}
    max_degree = 0
    for edge, (a, b) in enumerate(edge_nodes):
        if triplet_count[edge] == 0:
            continue
        for node in (a, b):
            degree_of_node[node] = degree_of_node.get(node, 0) + 1
            max_degree = max(max_degree, degree_of_node[node])
    num_nodes = len(degree_of_node)

    # 7. spread the edge scores back onto the input rows
    edge_score = np.divide(score_sum, triplet_count, out=np.full(len(edge_nodes), np.nan),
                           where=triplet_count > 0)
    scored = edge_of_row >= 0
    scores[scored] = edge_score[edge_of_row[scored]]
    result = TripletScores(scores, 0.0, len(triplets), len(size_of_component),
                           int(np.count_nonzero(~np.isnan(scores))), num_nodes, max_degree)
    result.tau = result.threshold(min_score)
    return result


def load_view_graph(path):
    """The (rows, pairs, cpp_scores) of a pairs CSV written by --export-pairs-csv."""
    rows = read_csv_rows(path)
    columns = key_columns(rows[0], path)
    if "NumMatches" not in rows[0]:
        sys.exit("error: '%s' has no NumMatches column" % path)
    pairs, cpp_scores = [], []
    for row in rows:
        key = pair_key(row, columns)
        pairs.append((key[0], key[1], int(float(row["NumMatches"]))))
        cell = (row.get("TripletScore") or "").strip()
        cpp_scores.append(float(cell) if cell else np.nan)
    return rows, pairs, np.asarray(cpp_scores)


# ---------------------------------------------------------------------------- subcommands

def cmd_score(args):
    rows, pairs, _ = load_view_graph(args.pairs)
    result = compute_triplet_scores(pairs, args.min_score)
    with open(args.output, "w", newline="") as handle:
        handle.write("# triplet_disambiguation.py score, m=%g, tau=%.6f\n" % (args.min_score, result.tau))
        handle.write("# %s\n" % result.summary())
        handle.write("# TripletScore empty = unscored (no triplet in the largest triplet-graph component)\n")
        writer = csv.writer(handle)
        writer.writerow(["ImageA", "ImageB", "NumMatches", "TripletScore", "Kept"])
        for (a, b, inliers), score in zip(pairs, result.scores):
            kept = bool(score >= result.tau) if not np.isnan(score) else False
            writer.writerow([a, b, inliers, "" if np.isnan(score) else "%.6f" % score, int(kept)])
    kept = int(np.count_nonzero(np.nan_to_num(result.scores, nan=-1.0) >= result.tau))
    print("%s\ntau %.6f at m %g; kept %d of %d pairs -> '%s'"
          % (result.summary(), result.tau, args.min_score, kept, len(pairs), args.output))
    return 0


def cmd_parity(args):
    rows, pairs, cpp_scores = load_view_graph(args.pairs)
    if np.all(np.isnan(cpp_scores)):
        sys.exit("error: '%s' has no TripletScore values (exported by an older build?)" % args.pairs)
    result = compute_triplet_scores(pairs, 0.0)
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
    result = compute_triplet_scores(pairs, 0.0)

    labelRows = read_csv_rows(args.labels)
    labelColumns = key_columns(labelRows[0], args.labels)
    if "label" not in labelRows[0]:
        sys.exit("error: '%s' has no label column" % args.labels)
    labelOfKey = {pair_key(row, labelColumns): row["label"].strip() for row in labelRows}

    # a plausible pair is a true edge, an implausible pair a false edge, ambiguous pairs are dropped
    scores, isTrue, numAmbiguous, numUnlabelled = [], [], 0, 0
    for (a, b, _inliers), score in zip(pairs, result.scores):
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
    # an unscored pair carries no score but is removed by the filter, so it ranks below every
    # scored pair: -1 is the rank the filter itself gives it
    ranked = np.where(scored, scores, -1.0)

    print("%s" % result.summary())
    print("%d labelled pairs (%d true, %d false), %d ambiguous and %d unlabelled dropped; "
          "%d of the labelled pairs are scored"
          % (scores.size, int(np.count_nonzero(isTrue)), int(np.count_nonzero(~isTrue)),
             numAmbiguous, numUnlabelled, int(np.count_nonzero(scored))))
    # the largest-triplet-graph-component step alone removes these, before any threshold applies
    print("the %d unscored labelled pairs are %d true and %d false"
          % (int(np.count_nonzero(~scored)), int(np.count_nonzero(~scored & isTrue)),
             int(np.count_nonzero(~scored & ~isTrue))))
    aucAll = roc_auc(ranked, isTrue)
    aucScored = roc_auc(scores[scored], isTrue[scored])
    print("AUC %.4f over every labelled pair (unscored ranked last), %.4f over the scored ones alone"
          % (aucAll, aucScored))

    numTrue, numFalse = int(np.count_nonzero(isTrue)), int(np.count_nonzero(~isTrue))
    for m in args.min_scores:
        tau = result.threshold(m)
        kept = scored & (scores >= tau)
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
                     "unscored pairs enter at threshold -1\n")
        handle.write("# AUC %.6f over every labelled pair, %.6f over the scored ones alone\n"
                     % (aucAll, aucScored))
        handle.write("# the %d unscored labelled pairs are %d true and %d false\n"
                     % (int(np.count_nonzero(~scored)), int(np.count_nonzero(~scored & isTrue)),
                        int(np.count_nonzero(~scored & ~isTrue))))
        writer = csv.writer(handle)
        writer.writerow(["threshold", "kept", "kept_true", "kept_false", "tpr", "fpr", "precision"])
        for threshold in np.unique(np.concatenate(([-1.0], ranked))):
            kept = ranked >= threshold
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
    scoreParser.add_argument("--pairs", required=True, help="pairs CSV (ImageA,ImageB,NumMatches,...)")
    scoreParser.add_argument("-o", "--output", required=True, help="output CSV")
    scoreParser.add_argument("-m", "--min-score", type=float, default=0.6, help="the paper's minimum edge score m (default 0.6)")
    scoreParser.set_defaults(func=cmd_score)

    parityParser = subparsers.add_parser("parity", help="check the scores against the CSV's own TripletScore column")
    parityParser.add_argument("--pairs", required=True, help="pairs CSV carrying a TripletScore column")
    parityParser.add_argument("--tolerance", type=float, default=1e-5, help="maximum allowed absolute difference (default 1e-5)")
    parityParser.set_defaults(func=cmd_parity)

    rocParser = subparsers.add_parser("roc", help="evaluate the scores against ground-truth pair labels")
    rocParser.add_argument("--pairs", required=True, help="pairs CSV (ImageA,ImageB,NumMatches,...)")
    rocParser.add_argument("--labels", required=True,
                           help="labels CSV with a label column of plausible/implausible/ambiguous "
                                "(pair_gt_labels.py in coverage mode); a plausible pair is a true edge, "
                                "an implausible pair a false edge, ambiguous pairs are excluded")
    rocParser.add_argument("-o", "--output", required=True, help="output ROC curve CSV")
    rocParser.add_argument("--min-scores", type=float, nargs="+", default=[0.3, 0.6, 0.9],
                           help="the values of m to report an operating point for (default 0.3 0.6 0.9)")
    rocParser.set_defaults(func=cmd_roc)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
