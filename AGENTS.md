# OpenMVS General Instructions

OpenMVS is a comprehensive photogrammetry library implementing a complete pipeline from image sequences to textured 3D models. It includes Structure-from-Motion (SFM) for camera pose estimation and sparse reconstruction, plus Multi-View Stereo (MVS) for dense reconstruction and mesh generation. The codebase is mature C++ with custom framework patterns.

## Project Architecture

### Core Namespaces & Structure
- **SFM namespace**: Structure-from-Motion reconstruction algorithms (`libs/SFM/`)
- **MVS namespace**: Multi-view Stereo reconstruction algorithms (`libs/MVS/`)
- **VIEWER namespace**: 3D visualization application (`apps/Viewer/`)
- **SEACAVE namespace**: Low-level utilities (`libs/Common/`)

### Key Libraries (libs/)
- `Common/`: Custom framework (types, logging, containers, math utilities)
- `IO/`: File format support (PLY, OBJ, MVS formats)
- `Math/`: Mathematical primitives and operations
- `SFM/`: Core SFM algorithms (Scene, Image, Camera, FeaturesExtractor, PairsMatcher, BundleAdjustment, etc.)
- `MVS/`: Core MVS algorithms (Scene, Image, Camera, Mesh, PointCloud, etc.)

### Applications (apps/)
Each app is a standalone executable for specific pipeline stages:

**SFM Stage:**
- `CreateStructure`: Initialize SFM scene from image metadata and camera parameters

**MVS Stage:**
- `DensifyPointCloud`: Dense reconstruction from sparse SFM points
- `ReconstructMesh`: Surface reconstruction from dense point clouds
- `RefineMesh`: Mesh quality improvement and optimization
- `TextureMesh`: Texture mapping onto reconstructed meshes

**Utilities:**
- `ExtractKeyframes`: Extract representative frames from video sequences
- `TransformScene`: Apply transformations to scene geometry
- `Tests`: Test suite for SFM and MVS algorithms
- `Viewer`: Interactive 3D visualization with OpenGL

**Interface/Import-Export:**
- `InterfaceCOLMAP`: COLMAP format import/export
- `InterfaceOpenMVG`: OpenMVG format import/export
- `InterfaceMetashape`: Metashape format import/export
- `InterfaceMVSNet`: MVSNet format import/export
- `InterfacePolycam`: Polycam format import/export

## Build System & Workflows

### Building
```bash
# Standard build (uses vcpkg for dependencies)
mkdir make && cd make
cmake ..
cmake --build . -j4  # or ninja (generates ninja files)
```

### Key Build Tools
- **vcpkg**: Automatic dependency management (see `vcpkg.json`)
- **CMake**: Primary build system with custom utilities in `build/Utils.cmake`
- **ninja**: Preferred generator (faster than make)

### Development Builds
- Debug builds in `make/bin/Debug/`
- Built executables: `./bin/Debug/Viewer`, `./bin/Debug/DensifyPointCloud`, etc.
- Use `cmake --build . -j4` from `make/` directory for incremental builds

## Code Patterns & Conventions

### Memory Management
- Reference counting with automatic cleanup
- RAII patterns throughout

### Logging & Debugging
```cpp
DEBUG("Message");           // Level 0 (always shown in debug)
DEBUG_EXTRA("Details");     // Level 1 (verbose)
VERBOSE("Info: %s", str);   // General logging
```

### Error Handling
- `ASSERT(condition)` for debug checks
- Return false/NULL for failures

### Common Typedefs
```cpp
typedef SEACAVE::TPoint3<float> Point3f;  // 3D points compatible with both OpenCV and Eigen
typedef SEACAVE::TMatrix<float,3,3> Matrix3x3f;  // 3x3 floats matrix compatible with both OpenCV and Eigen
typedef SEACAVE::String String;  // String type
#define NO_ID ((uint32_t)-1)  // Invalid index
```
Most of OpenMVS code uses custom point and matrix types derived from OpenCV types, e.g., `SEACAVE::Point3f`, `SEACAVE::Matrix4f`. Hoewever, some components use Eigen3 types, e.g. `SEACAVE::AABB3d` and `SEACAVE::Ray3d` classes. There custom types support convertion operation to and from Eigen3 types.

SEACAVE::cList template class is a custom vector implementation used throughout the codebase, fully compatible with std::vector. It provides additional functionality such as using or not the constructor for the elements, custom size type, and additional operations like GetMean, GetMedian, Sort, etc. By default it uses memcpy to manage elements, but it can be configured (useConstruct) to use constructors and destructors when needed. It also provides a custom size type (IDX_TYPE) which is typically defined as size_t, but can be changed if needed.

Often used is also FOREACH macro for iterating over any vector-like container:
```cpp
FOREACH(index, container) {
    auto& element = container[index];
    // Do something with element
}
```
Similarly, RFOREACH macro iterates in reverse order.

`SEACAVE::PairIdx` (in `libs/Common/Types.h`) packs two `uint32_t` indices into a single `uint64_t` via a union. Use it — never hand-rolled `(uint64_t(a) << 32) | b` bit-packing — whenever you need a composite key from two 32-bit indices. Common cases: `(imageID, featureID)` for per-observation lookups, `(platformID, cameraID)`, or image-pair buckets. Two forms:
- `PairIdx(a, b)` — raw constructor; stores the two fields in order, no reordering. Use this when `a` and `b` have *different meaning* (e.g. image vs feature). Works as an `unordered_map` key out-of-the-box since `std::hash<PairIdx>` is already specialized in `Types.inl`.
- `MakePairIdx(a, b)` — asserts `a != b` and swaps so the smaller index comes first. Use this only for *symmetric* pairs where `(a,b)` and `(b,a)` should hash to the same bucket, e.g. image-pair buckets in match graphs.

TD_TIMER_START() and TD_TIMER_GET_FMT() macros are used for performance measurements. TD_TIMER_START() (similarly TD_TIMER_STARTD() paird with DEBUG() prints) starts a timer, and TD_TIMER_GET_FMT() returns a formatted string with the elapsed time since the timer was started.

VERBOSE() macro is used for general logging messages. It works similarly to DEBUG() macro, but is intended for critical information, as it always prints regardless of debug level. DEBUG_EXTRA() and DEBUG_ULTIMATE() macros are used for more verbose logging, with DEBUG_ULTIMATE() being the most verbose.

### Pixel & Image Types
`TPixel<TYPE>` is a 3-channel color type with BGR memory layout (matching OpenCV). Access channels by name (`p.r`, `p.g`, `p.b`) or by index (`p.c[0]=b, c[1]=g, c[2]=r`). Common typedefs: `Pixel8U` (`TPixel<uint8_t>`), `Pixel32F` (`TPixel<float>`).

`TImage<TYPE>` wraps `cv::Mat_<TYPE>`. Common typedefs: `Image8U3` (`TImage<Pixel8U>`), `Image32F3` (`TImage<Pixel32F>`), `Image8U`, `Image32F`, `Image16U`.

**Critical**: `Point3f` (`TPoint3<float>`) has `{x, y, z}` fields mapping to memory positions `[0, 1, 2]`, while `Pixel32F` stores `{b, g, r}` at positions `[0, 1, 2]`. When `Image32F3` stores `Pixel32F` but is accessed via `Point3f&`, field `.x` reads `b`, not `r`. Always use `Pixel32F` with named fields (`.r`, `.g`, `.b`) for pixel operations, not `Point3f`.

**Pixel conversions**:
- Use `Pixel32F::cast<uint8_t>()` for float→uint8 conversion (proper channel mapping with clamping)
- Use `Pixel32F(Pixel8U::RED)` to construct from named constants (channel-correct)
- Named constants: `Pixel8U::RED`, `BLACK`, `WHITE`, `GREEN`, `BLUE`, `CYAN`, `GRAY` (uint8 range 0-255); `Pixel32F::RED` etc. use float range [0,1]

**Image sampling** — use `TImage` built-in samplers instead of manual bilinear interpolation:
```cpp
typedef Sampler::Linear<float> LinearSampler;
static const LinearSampler linearSampler;
// bilinear sample returning Pixel32F from an Image8U3
Pixel32F color = img.sample<LinearSampler, Pixel32F>(linearSampler, pt);
```

**Bounds checking** — use `TImage::isInsideWithBorder` instead of manual coordinate checks:
```cpp
// check that bilinear sampling (border=1) won't read out of bounds
if (img.isInsideWithBorder<float, 1>(pt))
    color = img.sample<LinearSampler, Pixel32F>(linearSampler, pt);
```

**Image I/O** — use `TImage::Load`/`TImage::Save` instead of `cv::imread`/`cv::imwrite`:
```cpp
Image8U4 image;
image.Load(fileName);  // loads with correct channel/depth conversion
image.Save(fileName);  // saves via OpenCV with correct format
```

### Headless Debug Mode (`_HEADLESS_DEBUG`)
- Build flag: `cmake -DOpenMVS_HEADLESS_DEBUG=ON` — controlled via CMake OPTION at CMakeLists.txt:45
- Gating: `ConfigLocal.h.in` template line 71 expands `#cmakedefine _HEADLESS_DEBUG` when the CMake variable is ON
- Effect: prints `[ASSERT]` to stderr and continues (no modal dialogs, no `_CrtDbgBreak()`); `LogConsole::Open()` short-circuits to leave stdout/stderr on inherited terminal
- Implementation: `Config.h` lines 277–284 redefine `PRINT_ASSERT_MSG` macro and define `_ASSERT_BREAK()` empty; `Config.h` lines 294–300 skip `_CrtDbgReport()` modal when flag is set; `Log.cpp` line 272 short-circuits redirection
- Use case: CI/test runners capture all invariant failures in one pass without blocking on popups
- Production builds (flag OFF): zero code change — byte-identical to baseline

### Configuration
- Build-time config in `ConfigLocal.h` (generated) included in every code file
- Runtime options via boost::program_options pattern
- Feature flags like `OpenMVS_USE_CUDA`, `OpenMVS_USE_CERES`, `OpenMVS_HEADLESS_DEBUG`
- Each library uses a precompiled header (`Common.h`) for common includes, like Eigen, OpenCV, etc.

## Viewer Application Specifics

### Key Classes
- `Scene`: Main data container (MVS::Scene + rendering state)
- `Window`: GLFW window management + input handling
- `Renderer`: OpenGL rendering (points, meshes, cameras)
- `Camera`: View/projection matrices + navigation
- `UI`: ImGui interface components

### Rendering Pipeline
```cpp
window.Run(scene) →
  Render(scene) →
    renderer->RenderPointCloud/RenderMesh →
      OpenGL draw calls
```

### Event System
- GLFW events → Window callbacks → Control system updates
- Render-only-on-change optimization uses `glfwWaitEventsTimeout()`
- `Window::RequestRedraw()` triggers frame updates

## Integration Points

### File Formats
- `.mvs`: Native binary format (boost serialization)
- `.ply`: Point clouds and meshes (ASCII/binary)
- `.obj`: Mesh export with MTL materials
- Interface apps handle external formats (COLMAP, etc.)

### External Dependencies
- **Eigen3**: Linear algebra (matrices, vectors)
- **OpenCV**: Image processing and I/O
- **CGAL**: Computational geometry
- **Boost**: Serialization, program options, containers
- **CUDA**: GPU acceleration (optional)
- **GLFW/OpenGL**: Viewer rendering

### Cross-Component Communication
- `SFM::Scene` is used for sparse reconstruction and camera pose management
- `MVS::Scene` is the central data exchange format for dense reconstruction
- Applications typically: load scene → process → save scene
- Viewer loads and visualizes any stage of the pipeline

## Pipeline Overview

The typical photogrammetry workflow:
1. **SFM Stage**: Feature extraction → Pair matching → Bundle adjustment → Global alignment/scale averaging
2. **MVS Stage**: Dense point cloud generation → Mesh reconstruction → Mesh refinement → Texturing → Viewer visualization

## Testing & Debugging

### Running Tests
```bash
# From make/ directory
ctest                    # Run all tests
./bin/Debug/Tests        # Direct test executable
```

### Common Debugging
- Use `DEBUG()` macros liberally
- Check `ASSERT()` failures for logic errors
- Use `TD_TIMER_START()` for performance timing.
- Viewer: Use F1 for help dialog, check console output
- Memory issues: Build with `_DEBUG` for additional checks

## Code Style & Conventions

- Naming: Functions `CamelCase()`, variables `lowerCamelCase`, type prefixes: `n` (numeric), `f` (float), `b` (bool), `p` (pointer), `_` (private members). Constants/enums UPPERCASE.
- Formatting: K&R brackets, tabs for indentation.
- Patterns: Early returns, range-based loops preferred, STL/cList containers, const correctness.

## Performance Considerations

- Multi-threading via OpenMP (`#pragma omp parallel`) for simple parallelism and `BS::light_thread_pool` for task-based parallelism.
- CUDA kernels for GPU acceleration (when enabled)
- Memory-mapped files for large datasets
- Spatial data structures (octrees) for efficient queries
- Viewer optimizations: frustum culling, render-only-on-change mode

## Available Tasks

The task files in `.github/instructions/` define step-by-step
workflows. Read and follow the relevant one when the context matches a defined workflow.

# Use the analyze-codebase agent to document this codebase using the full orchestrated analysis
claude --agent analyze-codebase
# Or run individual agents directly
claude --agent catalog-features
claude --agent suggest-improvements
