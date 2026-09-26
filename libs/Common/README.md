# Common Library

The Common library is the foundation layer that every other OpenMVS library builds on. It provides a custom framework of containers, geometric primitives, cross-platform utilities, threading, logging, and memory management. If you're working anywhere in OpenMVS, you're implicitly using Common.

## What You Need to Know First

### Everything includes `Common.h`

Every library in OpenMVS uses `Common.h` as its precompiled header. This single header pulls in Eigen3, OpenCV, Boost serialization, nanoflann, and all the custom types. You'll never see explicit `#include <Eigen/Dense>` in most files -- it comes through `Common.h`.

### The SEACAVE namespace

All Common types live in the `SEACAVE` namespace. You'll see types like `SEACAVE::Point3f`, `SEACAVE::String`, `SEACAVE::cList<>` throughout the codebase. The `using namespace SEACAVE;` directive is applied in most translation units, so you can usually use the short names.

### `cList` is the primary container, not `std::vector`

The most important type to understand is `cList` (`List.h`). It's a custom dynamic array used everywhere instead of `std::vector`. It's API-compatible with `std::vector` (iterators, `size()`, `operator[]`, etc.) but adds:

- **Fine-grained memory control**: The template parameter `useConstruct` controls whether elements use constructors (`=2`), memcpy (`=1`, the default), or raw memory with no initialization (`=0`). This matters for performance with large arrays of POD types like 3D points.
- **Extra operations**: `GetMean()`, `GetMedian()`, `Sort()`, `Push()`, `Pop()`, `RemoveAt()`.
- **Custom growth**: The `grow` parameter (default 16) controls pre-allocation batch size.

You'll encounter shortcut macros that declare common configurations:
```cpp
CLISTDEFSCALAR(float)   // cList for scalar/POD types (no constructors)
CLISTDEF0(MyClass)      // cList for objects with default constructors
CLISTDEF2(MyClass)      // cList for objects that need copy constructors
```

### Iteration macros

Instead of range-based for loops, you'll frequently see:
```cpp
FOREACH(i, myList) {
    // i is the index, use myList[i] to access elements
}

RFOREACH(i, myList) {
    // Same, but iterates in reverse (useful when removing elements)
}

FOREACHPTR(pItem, myList) {
    // pItem is a pointer to each element
}
```

These macros are defined in `List.h`. They're simple `for` loop wrappers, but understanding them is essential for reading any OpenMVS code.

## Logging and Debugging

OpenMVS has its own logging system with verbosity levels:

```cpp
VERBOSE("Critical info: %s", str);   // Always prints -- use for important messages
DEBUG("Debug info: %d", val);        // Only in debug builds (level 0)
DEBUG_EXTRA("Verbose: %f", val);     // Level 1 -- needs higher verbosity setting
DEBUG_ULTIMATE("Trace: %d", val);    // Level 2 -- extremely verbose
```

These are printf-style macros. They go through the `Log` singleton (`Log.h`) which supports multi-threaded buffering and custom listeners.

### Performance timing

```cpp
TD_TIMER_START();
// ... expensive operation ...
VERBOSE("Operation took %s", TD_TIMER_GET_FMT().c_str());
```

There's also `TD_TIMER_STARTD()` which pairs with `DEBUG()` instead of `VERBOSE()`.

## Geometric Primitives

Common provides a full set of 3D geometry types built on Eigen3. They're all templated on precision (float/double) and dimensionality (2D/3D):

| Type | What it is | Key operations |
|------|-----------|----------------|
| `TAABB` | Axis-aligned bounding box | `Insert(point)`, `Intersects(other)`, `GetCenter()`, `Transform(matrix)` |
| `TOBB` | Oriented bounding box | `Set(pointCloud)`, `Intersects(point)`, `GetAABB()` |
| `TRay` | Ray (origin + direction) | `Intersects(triangle/plane/sphere/AABB)`, `ProjectPoint()`, `Distance()` |
| `TTriangle` | Triangle (3 vertices) | `GetAABB()`, `GetPlane()`, `GetCenter()` |
| `TPlane` | Plane (normal + distance) | `Distance(point)`, `ProjectPoint()`, `Classify()`, `Optimize(points)` |
| `TSphere` | Bounding sphere | `Classify(point)`, `Enlarge()` |
| `TLine` | Line segment | Similar to Ray but with endpoints |
| `TQuaternion` | Quaternion rotation | `Inverse()`, `MultVec()`, angle/axis conversion |
| `TOctree` | Spatial partitioning tree | `Collect(aabb)`, `Collect(point, radius)` |

Common typedefs you'll see everywhere:
```cpp
AABB3f   // float, 3D bounding box
OBB3f    // float, 3D oriented box
Ray3f    // float, 3D ray
Plane3f  // float, 3D plane
```

**Important**: These geometry types use Eigen internally but are not Eigen types themselves. They provide `.IsValid()`, `.IsEmpty()` methods and conversion operators to/from Eigen. See the CLAUDE.md in the root project for notes on type interop.

## Threading

Common provides cross-platform threading primitives:

- **`Thread`** (`Thread.h`): Create threads with `start(fn, data)`, control with `stop()`, `join()`. Also provides atomic operations: `safeInc()`, `safeDec()`, `safeExchange()`.
- **`CriticalSection`** (`CriticalSection.h`): Recursive mutex. Use with `Lock cs(critSec);` RAII wrapper.
- **`FastCriticalSection`**: Lightweight non-recursive spinlock for short critical sections.
- **`RWLock`**: Reader-writer lock for read-heavy workloads.

Most high-level parallelism in OpenMVS uses OpenMP or `BS::light_thread_pool` (a header-only thread pool), but these primitives are used for fine-grained synchronization.

## Memory Management

- **`CSharedPtr<T>`** (`SharedPtr.h`): Reference-counted smart pointer with thread-safe refcount updates. You'll see this used for shared resources.
- **`CAutoPtr<T>`** (`AutoPtr.h`): Unique ownership pointer (similar to `std::unique_ptr`).

The codebase predates widespread C++11 adoption, so these custom smart pointers are used instead of `std::shared_ptr` / `std::unique_ptr` in many places.

## Other Utilities Worth Knowing

### String (`Strings.h`)
`SEACAVE::String` extends `std::string` with:
```cpp
String s;
s.Format("Value: %d", 42);           // printf-style formatting
String upper = s.ToUpper();           // Case conversion
int val = String::FromString<int>("42"); // Type conversion
```

### Flags (`Util.h`)
```cpp
TFlags<uint32_t> flags;
flags.set(FLAG_A);                    // Set bits
if (flags.isSet(FLAG_A | FLAG_B))     // Check bits
    flags.unset(FLAG_A);              // Clear bits
```

### Random Numbers (`Random.h`)
```cpp
float r = SEACAVE::random();                    // [0, 1] uniform
int n = SEACAVE::randomRange(1, 10);            // [1, 10] uniform
float g = SEACAVE::randomGaussian(0.f, 1.f);    // Normal distribution
```

### Configuration System
Runtime options are declared with macros:
```cpp
DEFVAR_float(OPT, optThreshold, "Threshold", "Description", 0.5f, 0.f, 1.f)
```
These integrate with Boost program_options for command-line parsing.

## Key Constants

```cpp
NO_ID           // ((uint32_t)-1) -- invalid index sentinel, used everywhere
ZERO_TOLERANCE  // 1e-7 -- floating point comparison epsilon
PI, HALF_PI, TWO_PI  // Math constants
D2R(degrees)    // Degree to radian conversion
R2D(radians)    // Radian to degree conversion
```

## File Organization

```
libs/Common/
├── Common.h/cpp          # Main header (precompiled), logging macros, path macros
├── Types.h/Types.inl     # Fundamental typedefs (REAL, NO_ID, hash specializations)
├── Config.h              # Build-time configuration
├── Maths.h               # Math constants and utility functions
├── List.h                # cList container + iteration macros
├── ListFIFO.h            # FIFO queue variant
├── AABB.h/inl            # Axis-aligned bounding box
├── OBB.h/inl             # Oriented bounding box
├── Ray.h/inl             # Ray + Triangle
├── Plane.h/inl           # Plane
├── Sphere.h/inl          # Bounding sphere
├── Line.h/inl            # Line segment
├── Rotation.h/inl        # Quaternion
├── Octree.h/inl          # Spatial partitioning
├── DepthGeometry.h       # Host/device depth similarity, depth-plane fit, plane normal and propagation
├── Thread.h              # Cross-platform threading
├── CriticalSection.h     # Mutexes, locks, RWLock
├── Semaphore.h           # Semaphore
├── Timer.h/cpp           # Performance timing
├── Log.h/cpp             # Logging system
├── Strings.h             # Enhanced string class
├── File.h                # File I/O
├── Streams.h             # Stream abstractions
├── MemFile.h             # Memory-mapped files
├── Util.h/inl/cpp        # Flags, Histogram, path utilities
├── Random.h              # Random number generation
├── SharedPtr.h           # Reference-counted pointer
├── AutoPtr.h             # Unique ownership pointer
├── HalfFloat.h           # float16 support
├── Filters.h             # Signal/image filters
├── RunningAverage.h      # Online mean/variance
├── AutoEstimator.h       # Automatic parameter estimation
├── Sampler.inl           # Sampling utilities
├── EventQueue.h/cpp      # Event dispatching
├── ConfigTable.h/cpp     # Configuration management
├── SML.h/cpp             # Simple Markup Language parser
├── Hash.h                # Hash utilities
├── Queue.h               # Queue container
└── UtilCUDA.cpp          # CUDA utilities (optional)
```

## Dependencies

- **Eigen3**: All geometry types and matrix operations
- **OpenCV**: Image types, some math operations
- **Boost**: Serialization framework
- **nanoflann**: KD-tree spatial indexing (header-only)
- **CUDA** (optional): GPU utilities
