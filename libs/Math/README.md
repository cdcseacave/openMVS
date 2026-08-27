# Math Library

The Math library provides specialized mathematical algorithms for photogrammetry and 3D computer vision. It includes robust statistics, non-linear optimization, geometric transformations, graph algorithms, and geodetic coordinate conversions.

Unlike the Common library (which provides general-purpose math types like points and matrices), the Math library focuses on **algorithms** -- the computational building blocks that the MVS and SFM pipelines use for estimation, optimization, and inference.

## What You Need to Know First

### This library bridges Eigen, OpenCV, and domain-specific math

The Math library works with types from Common (which wraps OpenCV and Eigen). Its functions accept `Point3`, `Matrix3x3`, `RMatrix` etc. and often convert internally to Eigen for computation. If you need to pass Eigen types directly, most functions provide overloads or you can use the conversion operators defined in Common/Types.

### Robust norms are critical for understanding the pipeline

In photogrammetry, data is always contaminated with outliers (wrong feature matches, occluded points, etc.). The robust norms in this library are used throughout the pipeline to prevent outliers from corrupting results. Understanding them helps you tune algorithm behavior.

## Robust Norms (`RobustNorms.h`)

M-estimators that downweight outliers during optimization. Each is a functor that transforms a residual value:

| Norm | Behavior | Where it's used |
|------|----------|-----------------|
| **Identity** | No change (standard L2) | Clean data, no outliers expected |
| **Huber** | L2 for small residuals, L1 for large ones | Bundle adjustment (smooth transition) |
| **Cauchy** | Gradually reduces influence of large residuals | Tolerant to large outliers |
| **GemanMcClure** | Bounded influence -- outliers can't dominate | Rotation averaging (IRLS) |
| **Tukey** | Completely ignores residuals beyond a threshold | Hard outlier rejection |
| **L1** | Sum of absolute values | General robustness |
| **PseudoHuber** | Smooth approximation to Huber (differentiable everywhere) | When you need gradients |
| **BlakeZisserman** | Graduated non-convexity | Multi-view geometry |
| **Exp** | Exponential falloff | Soft rejection |

**Usage**: These are functors -- you pass them to optimization routines:
```cpp
RobustNorm::Huber huber(threshold);
double weighted_residual = huber(raw_residual);
```

## Confidence Intervals (`ConfidenceInterval.h`)

Statistical tools for determining bounds on estimates:

- **`ComputeConfidenceIntervalTCritical()`**: Classical Student's t-based confidence intervals. Good when data is approximately normal.
- **`ComputeConfidenceIntervalX84()`**: Robust X84 method using median absolute deviation. Preferred when outliers are present (which is almost always in photogrammetry).
- **`Median()`**: Robust center estimation (used instead of mean in many places).

These are used to determine reprojection error thresholds, camera pose uncertainty, and to filter outlier tracks.

## Disjoint Set / Union-Find (`DisjointSet.h`)

A graph connectivity data structure that's central to several pipeline stages:

```cpp
DisjointSet<uint32_t> ds(numElements);

// Merge elements
ds.Union(a, b);

// Find which component an element belongs to
uint32_t root = ds.Find(x);  // Path compression for O(α(n)) amortized

// Conditional merge (only if callback approves)
ds.UnionIf(a, b, [](uint32_t rootA, uint32_t rootB) {
    return shouldMerge(rootA, rootB);
});

// Analyze results
auto sizes = ds.GetComponentSizes();
auto components = ds.GetComponents();
```

**Where it's used**:
- **Track building** (SFM): Merging feature observations across image pairs into tracks
- **Track merging** (GlobalAlignment): Combining tracks from different sub-scenes
- **Mesh connectivity**: Finding connected components in triangle meshes

## Similarity Transform (`SimilarityTransform.h/cpp`)

7-DOF transformation: rotation (3) + translation (3) + uniform scale (1).

```cpp
struct Transform {
    RMatrix R;     // 3x3 rotation
    Point3 t;      // Translation
    REAL scale;    // Uniform scale factor
};
```

**Key functions**:
- **`SimilarityTransform(points, pointsRef)`**: Estimates transform from 3D-3D correspondences using Umeyama's closed-form algorithm
- **`DecomposeSimilarityTransform(T4x4, R, t, s)`**: Extracts R, t, scale from a 4x4 matrix
- **`EstimateRotationAlignment()`**: Robust rotation alignment using IRLS with Tukey weighting

**Projection matrix utilities**:
- **`DecomposeProjectionMatrix(P, K, R, C)`**: RQ decomposition to extract intrinsics (K), rotation (R), and camera center (C) from a 3x4 projection matrix
- **`AssembleProjectionMatrix(K, R, C, P)`**: Construct P from components

These are used in sub-scene alignment, GPS registration, and coordinate frame conversions.

## Geodetic Transforms (`GeodeticTransforms.h/cpp`)

Coordinate system conversions for GPS integration. OpenMVS uses these when aligning reconstructions to real-world coordinates.

The three coordinate systems:
1. **WGS84** (latitude, longitude, altitude): What GPS receivers output
2. **ECEF** (Earth-Centered Earth-Fixed): Cartesian coordinates relative to Earth's center
3. **ENU** (East-North-Up): Local tangent plane -- the most useful for reconstruction work

```cpp
// GPS → local coordinates
WGS84ToENU(lat, lon, alt,        // GPS position
           lat0, lon0, alt0,     // Reference origin
           east, north, up);     // Output local coords

// Local coordinates → GPS
ENUToWGS84(east, north, up,
           lat0, lon0, alt0,
           lat, lon, alt);
```

The reference origin is typically the first camera's GPS position. All subsequent coordinates are meters in the local tangent plane.

## Optimization Algorithms

### Least Absolute Deviation Solver (`LeastAbsoluteDeviationSolver.h/cpp`)

Solves `min ||Ax - b||₁` (L1 norm) using ADMM (Alternating Direction Method of Multipliers). L1 minimization is more robust to outliers than L2 (least squares).

**Solver options**:
- `rho`: Augmented Lagrangian parameter (controls convergence speed vs. accuracy)
- `alpha`: Over-relaxation parameter (1.0-1.8, higher = faster but less stable)
- Linear solver: Eigen's `SimplicialLLT` or SuiteSparse's `CholmodSupernodalLLT` (faster, optional)

### Levenberg-Marquardt (`LMFit/lmmin.h/cpp`)

Non-linear least-squares optimization. You provide a callback function that evaluates residuals at each iteration, and LM finds the parameter values that minimize the sum of squared residuals.

This is a self-contained implementation (not Ceres). It's used for simpler optimization problems like model fitting and pose refinement.

## Graph Algorithms

### Max-Flow / Min-Cut (`TetraFlow.h`)

`TetraFlow` is an independent, header-only implementation of the Incremental Breadth-First Search max-flow algorithm (Goldberg, Hed, Kaplan, Tarjan, Werneck, ESA 2011) specialized for graphs in which every node has exactly four arcs: the dual graph of a tetrahedralization, one node per cell and one edge per facet, which is what the Delaunay mesh reconstruction cuts. Every node occupies one 64-byte cache line holding its four arcs and its complete tree state, node ids are 32-bit, and the source side of the augmentations is settled in level-ordered batches (one tree-arc traversal per growth pass instead of one per path). On multi-million-cell scenes it solves ~1.7x faster than the reference IBFS with ~3x less solver memory. Boost license.

Usage: `TetraFlow g(numNodes); g.AddNode(n, capSource, capSink); g.AddEdge(u, v, capUV, capVU); flow = g.ComputeMaxFlow(); g.IsNodeOnSrcSide(n)`. Alternatively the caller assigns the arc slots itself and accumulates the capacities in place — `g.EdgeCapacity(n, slot) += w; g.SourceCapacity(n) = s; g.SinkCapacity(n) += t; g.LinkEdge(u, slotU, v, slotV)` (link once per edge, before or after accumulating) — which is how the mesh reconstruction gathers its visibility weights directly in the solver's nodes (slot i of a cell = its facet i) with no separate per-cell weight array; `Release()` frees everything once the sides have been read. `CheckMaxFlow()` is an O(N) optimality check for tests (`apps/Tests/TestsMath.cpp` verifies both construction paths against an exact reference on random graphs).

`TetraFlow` replaces the previously installed `IBFS::IBFSGraph` implementation but is deliberately not API-compatible: it supports only degree-four graphs and uses a different construction and lifecycle API. Downstream users that require arbitrary-degree max-flow must retain a separate general-purpose solver. When upgrading an installation in place, remove stale `Math/IBFS` headers from the old prefix because installing a newer OpenMVS version does not delete removed headers.

### Loopy Belief Propagation (`LBP.h`)

Message-passing inference on graphical models. Minimizes energy functions defined over discrete labels with pairwise smoothness terms. Supports OpenMP parallelization.

Used for: labeling problems where you need to assign discrete labels to graph nodes while respecting pairwise consistency (e.g., depth label assignment, mesh face labeling).

## File Organization

```
libs/Math/
├── Common.h/cpp                         # Library entry, precompiled header
├── RobustNorms.h                        # M-estimator functors (9 types)
├── ConfidenceInterval.h                 # Statistical confidence bounds
├── DisjointSet.h                        # Union-Find data structure
├── SimilarityTransform.h/cpp            # 7-DOF transform estimation
├── GeodeticTransforms.h/cpp             # WGS84/ECEF/ENU conversions
├── LeastAbsoluteDeviationSolver.h/cpp   # ADMM-based L1 solver
├── LBP.h                               # Loopy Belief Propagation
├── LMFit/
│   └── lmmin.h/cpp                      # Levenberg-Marquardt fitting
├── TetraFlow.h                          # Max-flow / min-cut on 4-regular graphs
└── CMakeLists.txt                       # Build config
```

## Dependencies

- **Common** (required): Base types and Eigen3 integration
- **Eigen3** (inherited): Sparse solvers, matrix operations
- **Boost** (inherited): `boost::math::students_t` for statistical distributions
- **SuiteSparse/CHOLMOD** (optional): Faster sparse linear algebra for large problems. Enabled with `_USE_SUITESPARSE`.
