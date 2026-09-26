# Common Library

Foundation framework layer for all OpenMVS libraries. Provides custom containers, geometry primitives, cross-platform utilities, logging, threading, and memory management. Every other library depends on Common via the precompiled header `Common.h`.

## Core Components

### Custom Containers (`List.h`)
- **`cList<TYPE, ARG_TYPE, useConstruct, grow, IDX_TYPE>`** - Custom vector used throughout the codebase, fully compatible with `std::vector`.
  - `useConstruct=0`: No constructors/destructors (raw memory, POD types)
  - `useConstruct=1`: Uses memcpy/memmove (default)
  - `useConstruct=2`: Always uses copy constructors
  - `grow`: Elements to pre-allocate when expanding (default 16)
- Shortcut macros: `CLISTDEFSCALAR(TYPE)` (value types), `CLISTDEF0(TYPE)` (objects), `CLISTDEF2(TYPE)` (copy-constructible objects)
- Extra methods vs std::vector: `GetMean()`, `GetMedian()`, `Sort()`, `Push()`, `Pop()`

### Iteration Macros
```cpp
FOREACH(idx, container)      // Forward iteration by index
RFOREACH(idx, container)     // Reverse iteration by index
FOREACHPTR(ptr, container)   // Forward iteration by pointer
RFOREACHPTR(ptr, container)  // Reverse iteration by pointer
```

### Logging & Debug Macros (`Common.h`, `Log.h`)
```cpp
VERBOSE("format %s", arg);    // Always prints (critical info)
DEBUG("format %s", arg);      // Level 0 (debug builds)
DEBUG_EXTRA("format");        // Level 1 (verbose)
DEBUG_ULTIMATE("format");     // Level 2 (most verbose)
```

### Timing Macros (`Timer.h`)
```cpp
TD_TIMER_START();                    // Start a timer
TD_TIMER_GET_FMT();                  // Get elapsed time as formatted string
TD_TIMER_STARTD();                   // Start timer (pairs with DEBUG)
```

### Path Macros
```cpp
MAKE_PATH(str)        // Add working directory prefix
MAKE_PATH_SAFE(str)   // Add prefix only if not already a full path
GET_PATH_FULL(str)    // Get full path
```

### Math Constants & Functions (`Maths.h`)
- Constants: `PI`, `HALF_PI`, `TWO_PI`, `SQRT_2`, `ZERO_TOLERANCE` (1e-7)
- Conversion: `D2R(degrees)`, `R2D(radians)`
- Functions: `MINF`, `MAXF`, `FLOOR`, `CEIL`, `ROUND`, `POWI` (integer power), `LOG2I`

## Geometry Primitives (Eigen3-based)

All geometry types are templated on `TYPE` (float/double) and `DIMS` (2/3).

| Type | Header | Description |
|------|--------|-------------|
| `TAABB<T,D>` | `AABB.h` | Axis-aligned bounding box (ptMin, ptMax). Insert, Intersects, GetCenter, Transform |
| `TOBB<T,D>` | `OBB.h` | Oriented bounding box (rotation, center, extents) |
| `TRay<T,D>` | `Ray.h` | Ray (origin + direction). Intersects triangle/plane/sphere/AABB/OBB |
| `TTriangle<T,D>` | `Ray.h` | Triangle (3 vertices). GetAABB, GetPlane, GetCenter |
| `TPlane<T,D>` | `Plane.h` | Plane in Hessian normal form (normal + distance). Distance, ProjectPoint, Classify |
| `TSphere<T,D>` | `Sphere.h` | Bounding sphere (center + radius) |
| `TLine<T,D>` | `Line.h` | Line segment with endpoints |
| `TQuaternion<T>` | `Rotation.h` | Quaternion rotation (qx,qy,qz,qw). Inverse, MultVec, angle/axis conversion |
| `TOctree<...>` | `Octree.h` | Spatial partitioning tree. Build from items, query via Collect(aabb/point+radius) |

Common typedefs: `AABB3f`, `OBB3f`, `Ray3f`, `Plane3f`, `Sphere3f`, `Line3f`, `Triangle3f` (float, 3D).

## Depth-Map Geometry (`DepthGeometry.h`)
Host and CUDA device functions on Eigen types for a skew-free pinhole camera (focal `f`, principal
point `pp`), the single implementation used by the CPU depth-map code and the CUDA kernels:
`DepthSimilarity`/`IsDepthSimilar` (relative depth difference), `DepthPlaneFit` (least-squares depth
plane from neighbor offsets), `FitDepthGradient` (that fit over the 3x3 depth-similar neighbors of a
pixel, on any map indexed `(row,col)` with `rows`/`cols`), `NormalFromDepthGradient` (the plane's
camera-facing normal) and `InterpolatePlaneDepth` (a neighbor's plane carried to a pixel, as in
PatchMatch propagation). `HOST_DEVICE` (`Config.h`) marks functions compiled for both sides.

## Threading & Synchronization

| Class | Header | Purpose |
|-------|--------|---------|
| `Thread` | `Thread.h` | Cross-platform thread (start, stop, join, priority). Atomic ops: `safeInc`, `safeDec`, `safeExchange` |
| `CriticalSection` | `CriticalSection.h` | Recursive mutex |
| `FastCriticalSection` | `CriticalSection.h` | Non-recursive spinlock (lightweight) |
| `RWLock` | `CriticalSection.h` | Reader-writer lock |
| `Lock` / `FastLock` | `CriticalSection.h` | RAII scoped lock wrappers |

## Memory Management
- `CSharedPtr<T>` (`SharedPtr.h`) - Reference-counted smart pointer (thread-safe)
- `CAutoPtr<T>` (`AutoPtr.h`) - Unique ownership pointer

## Utilities

| Component | Header | Key Features |
|-----------|--------|-------------|
| `String` | `Strings.h` | Extends std::string: `Format()`, `ToUpper/Lower()`, `ToString<T>()`, `FromString<T>()` |
| `File` | `File.h` | File I/O with FILEINFO struct, Open/Close/Read/Write/Seek |
| `TFlags<T>` | `Util.h` | Bit flag operations: `isSet()`, `set()`, `unset()`, `flip()` |
| `THistogram<T>` | `Util.h` | Histogram with bins, `GetApproximatePermille()` for percentiles |
| `Random` | `Random.h` | mt19937-based: `random()`, `randomRange()`, `randomGaussian()` |
| `MemFile` | `MemFile.h` | Memory-mapped file I/O |
| `HalfFloat` | `HalfFloat.h` | float32 <-> float16 conversion |
| `RunningAverage` | `RunningAverage.h` | Online mean/variance computation |

## Configuration System (`Common.h`)
```cpp
DEFVAR_string(SPACE, name, title, description, default)
DEFVAR_bool(SPACE, name, title, description, default)
DEFVAR_int32(SPACE, name, title, description, default, min, max)
DEFVAR_float(SPACE, name, title, description, default, min, max)
```

## Key Type Definitions (`Types.h`)
```cpp
typedef double REAL;                    // Default floating precision
constexpr uint32_t NO_ID = (uint32_t)-1; // Invalid index marker
DECLARE_SINGLETON(ClassName)            // Static singleton pattern
```

## Hash Specializations (`Types.inl`)
Custom `std::hash` for: `std::pair`, `std::tuple`, `cv::Point_<T>`, `cv::Point3_<T>`, `SEACAVE::PairIdx`.

## Build & Dependencies
- **Precompiled header**: `Common.h` (includes Eigen3, OpenCV, Boost, nanoflann)
- **External deps**: Eigen3 (linear algebra), OpenCV (image processing), Boost (serialization), nanoflann (KD-trees), optional CUDA
- All other OpenMVS libs link against Common
