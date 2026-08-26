# Math Library

Mathematical algorithms and utilities for photogrammetry and 3D computer vision. Provides robust statistics, non-linear optimization, geometric transformations, graph algorithms, and geodetic coordinate conversions.

## Robust Norms (`RobustNorms.h`)
M-estimator functors for reducing outlier influence during optimization. Each implements `operator()(residual)` to transform residuals.

| Norm | Description | Use Case |
|------|-------------|----------|
| `RobustNorm::Identity` | No weighting (standard L2) | Clean data |
| `RobustNorm::L1` | Sum of absolute values | General robustness |
| `RobustNorm::Huber` | Smooth L1/L2 transition | Bundle adjustment |
| `RobustNorm::PseudoHuber` | Smooth approximation to Huber | Differentiable variant |
| `RobustNorm::Cauchy` | Heavy-tailed distribution | Large outliers |
| `RobustNorm::GemanMcClure` | Bounded influence | Rotation averaging (IRLS) |
| `RobustNorm::Tukey` | Biweight (zero beyond threshold) | Hard outlier rejection |
| `RobustNorm::BlakeZisserman` | Graduated non-convexity | Multi-view geometry |
| `RobustNorm::Exp` | Exponential downweighting | Soft rejection |

## Confidence Intervals (`ConfidenceInterval.h`)
- `ComputeTCriticalValue<T>()` - Student's t-distribution critical values
- `ComputeConfidenceIntervalTCritical<T>()` - Classical confidence intervals
- `ComputeConfidenceIntervalX84<T>()` - Robust X84 method (median absolute deviation)
- `Median<T>()` - Robust median computation
- `TConfidenceInterval<T>` - Struct holding bounds + mean + t-critical

## Disjoint Set / Union-Find (`DisjointSet.h`)
```cpp
DisjointSet<T> ds(size);
ds.Find(x);                    // Find representative (path compression)
ds.Union(x, y);                // Merge by rank
ds.UnionIf(x, y, guardMerge);  // Conditional merge with callback
ds.GetComponentSizes();         // Connected component enumeration
ds.GetComponents();             // Assign component IDs
```
Used for: track building, camera clustering, mesh connectivity analysis.

## Similarity Transform (`SimilarityTransform.h/cpp`)
7-DOF transformation (rotation + translation + uniform scale).

```cpp
struct Transform {
    RMatrix R;    // 3x3 rotation
    Point3 t;     // 3x1 translation
    REAL scale;   // Uniform scale
    Transform Invert() const;
    Transform operator*(const Transform&) const;  // Composition
    Point3 operator*(const Point3&) const;        // Transform point
};
```

**Key functions:**
- `SimilarityTransform(points, pointsRef)` - Closed-form via Umeyama's algorithm
- `DecomposeSimilarityTransform(T4x4, R, t, s)` - Extract R, t, scale from 4x4 matrix
- `EstimateRotationAlignment(srcRots, dstRots, alignR, threshold, maxIters)` - Robust IRLS rotation averaging with Tukey weighting

**Projection matrix utilities:**
- `DecomposeProjectionMatrix(P, K, R, C)` - RQ decomposition of P=K[R|-RC]
- `AssembleProjectionMatrix(K, R, C, P)` - Construct P from components

## Geodetic Transforms (`GeodeticTransforms.h/cpp`)
WGS84 coordinate conversions for GPS integration.

- `WGS84ToECEF()` / `ECEFToWGS84()` - Geodetic <-> geocentric
- `ECEFToENU()` / `ENUToECEF()` - Geocentric <-> local East-North-Up
- `WGS84ToENU()` / `ENUToWGS84()` - Direct geodetic <-> ENU

## Least Absolute Deviation Solver (`LeastAbsoluteDeviationSolver.h/cpp`)
ADMM-based L1 norm minimization: `min ||Ax - b||_1`.

**Options:** `rho` (augmented Lagrangian), `alpha` (over-relaxation), tolerances, max iterations.
**Linear solvers:** `SimplicialLLTLinearSolver` (Eigen), `SupernodalCholmodLLTLinearSolver` (SuiteSparse, optional).

## Levenberg-Marquardt Fitting (`LMFit/lmmin.h/cpp`)
Non-linear least-squares optimization with callback-based residual evaluation. Used in bundle adjustment, camera pose refinement, and model fitting.

## Graph Algorithms

### Max-Flow / Min-Cut (`TetraFlow.h`)
`TetraFlow`: header-only Incremental Breadth-First Search max-flow (Goldberg et al., ESA 2011) specialized for graphs with exactly four arcs per node (the Delaunay cell graph of the mesh reconstruction): one 64-byte node per cell, 32-bit ids, batched source-side augmentation. API: `AddNode(n, capSource, capSink)`, `AddEdge(u, v, capUV, capVU)`, or the slot-addressed construction `EdgeCapacity(n, slot)` / `SourceCapacity(n)` / `SinkCapacity(n)` accumulators plus `LinkEdge(u, slotU, v, slotV)` (the mesh reconstruction accumulates its weights directly in the nodes); `ComputeMaxFlow()`, `IsNodeOnSrcSide(n)`, `Release()`; `CheckMaxFlow()` for tests. Boost license, no third-party code.

### Loopy Belief Propagation (`LBP.h`)
Message-passing inference on graphical models for energy minimization over discrete labels. Supports OpenMP parallelization (`LBP_USE_OPENMP`).

## Build & Dependencies
- **Required**: Common library, Eigen3 (inherited)
- **Optional**: SuiteSparse/CHOLMOD (`_USE_SUITESPARSE`) for faster sparse solvers
- **Precompiled header**: `Common.h`
