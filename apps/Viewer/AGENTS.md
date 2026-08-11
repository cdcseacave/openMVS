# Viewer Application

Interactive 3D visualization and workflow execution tool for OpenMVS scenes. Lives in the `VIEWER` namespace.

## Architecture

### Class Composition

```
Scene (top-level container)
├── LayerArr            — independently visible scene/geometry layers
│   └── Layer
│       ├── MVS::Scene  — core photogrammetry data
│       ├── ImageArr    — valid photographs for this layer
│       └── appearance, bounds, working folder, dirty/uncertainty state
├── Window              — GLFW window + event loop
│   ├── Camera          — view/projection matrices, perspective/orthographic
│   ├── Renderer        — OpenGL rendering, GPU buffers, shaders, picker FBO
│   ├── UI              — ImGui interface (panels, dialogs, menus)
│   ├── ArcballControls — virtual trackball navigation
│   ├── FirstPersonControls — FPS-style free-flight camera
│   └── SelectionController — geometry selection (box/lasso/circle)
└── Workflow state      — async pipeline execution with worker thread
```

### Precompiled Header Chain
`Common.h` → `MVS/Common.h` → `glad/glad.h` (with `GLAD_GL_IMPLEMENTATION`) → `GLFW/glfw3.h` → `OpenGLDebug.h`

### Coordinate System
`Common.h` defines `gs_convert` — a static 4x4 matrix converting from OpenGL default (Y-up) to camera coordinates (Y-down, Z-backward/NADIR). Helper functions `TransW2L(R,t)` and `TransL2W(R,t)` build world-to-local and local-to-world 4x4 Eigen matrices from rotation + translation.

## Application Lifecycle

```
main() [Viewer.cpp]
  → Application::Initialize()     — boost::program_options, logging
  → Scene::Initialize(size, name, file, geometry)
      → Window::Initialize()      — GLFW context, OpenGL setup, ImGui init
      → Scene::Open()             — load MVS project (if file provided)
  → Scene::Run() → Window::Run()  — main event loop
```

### Event Loop (`Window::Run`)
1. `UpdateTiming()` — delta time calculation
2. `Scene::CheckWorkflowCompletion()` — poll async workflow state
3. Update active control system (arcball/first-person/selection) with delta time
4. `glfwWaitEventsTimeout()` (render-on-change) or `glfwPollEvents()` (continuous)
5. `Render()` → `glfwSwapBuffers()`
6. `UI::UpdateFrameStats()`

Use `Window::RequestRedraw()` to post a GLFW event that wakes the wait-for-events loop.

### Background Worker
`Scene::thread` processes `Scene::events` for async workflow and image-load execution. Workflows are tied to a stable layer ID; results are finalized on the main thread via `CheckWorkflowCompletion()` → `FinalizeWorkflow()`. Layer-container mutation is disabled while background work holds layer/image references.

## Rendering Pipeline

### Data Upload (CPU → GPU)
- `UploadLayers(Scene, Window)` — refresh all visible layer geometry
- `UploadPointClouds(Scene, normalLength)` — aggregate visible points + optional normals
- `UploadMeshes(Scene)` — aggregate visible meshes, normals, texcoords, and texture partitions
- `UploadCameras(Window)` — camera frustum line geometry
- `UploadUncertaintyEllipsoids(Window)` — per-camera pose-uncertainty solid shaded ellipsoids
- `UploadSelection(Window)` — highlighted primitive geometry
- `UploadBounds(MVS::Scene)` — AABB wireframe

### Frame Cycle
```
BeginFrame(camera, clearColor)    — upload ViewProjection UBO
SetLighting(dir, intensity, color) — upload Lighting UBO
RenderPointCloud()                 — GL_POINTS with dynamic point size
RenderMesh()                       — solid + wireframe + textured variants
RenderCameras()                    — frustum line rendering
RenderUncertaintyEllipsoids()      — pose-uncertainty translucent shaded solids (ellipsoidShader)
RenderImageOverlays()              — 3D photo planes with per-image opacity
RenderSelection()                  — highlighted primitives
RenderSelectionOverlay()           — 2D screen-space selection UI
RenderBounds()                     — AABB wireframe
RenderCoordinateAxes()
RenderArcballGizmos()              — virtual trackball visualization
EndFrame()
```

Non-UI screenshots (`--screenshot-file` without the `u` flag) are captured after ALL 3D layers
(cameras, ellipsoids, bounds, overlays) but before any ImGui window — the `--screenshot-show`
layer flags (`c`, `b`, ...) therefore apply to the capture.

### Pose-Uncertainty Ellipsoids
`Scene::LoadPoseUncertainty(csv)` (triggered by `--pose-quality-file`) parses a CreateStructure
pose-quality report, matches rows to scene images by ID (`scene.images[image.idx].ID` — preserved
from the SFM scene by ExportMVS), and fills the active `Layer::cameraUncertainty` array (per
VIEWER-image index). Visible layers with reports are rendered together.
`UploadUncertaintyEllipsoids` eigen-decomposes each 3x3 position covariance into an oriented
solid (triangulated UV-sphere) ellipsoid at the camera center, scaled by
`cameraUncertaintyAutoScale * Window::uncertaintyEllipsoidScale` (scene auto-fit base times the user
slider) and colored via the jet colormap normalized to the 95th-percentile sigma
(`Layer::cameraUncertaintyNorm`). It is drawn by `ellipsoidShader` (lit head-light shading + per-vertex color)
as a translucent surface — blended, depth-tested but not depth-writing, so the camera frustum at the
center stays visible. `LoadPoseUncertainty` AUTO-SETS `cameraUncertaintyAutoScale` — kept SEPARATE from
the user-facing `Window::uncertaintyEllipsoidScale` (which multiplies it, defaulting to x1) so the
deferred ImGui-ini load of that persisted slider cannot clobber the auto-fit — so the MEDIAN
ellipsoid's largest axis is ~3% of the scene bbox diagonal (`Camera::GetSceneSize().norm()`) — raw
sigmas are world-units and would otherwise render sub-pixel (tiny sigma) or scene-spanning (no-GPS
needle covariances); it logs matched/drawable/datum counts + the chosen scale via DEBUG. NOTE
`Pixel32F::gray2color(0)` is RED
and `(1)` is BLUE — pass `1 - value` for blue = good / red = bad ramps. Gauge-datum entries have zero
covariance (nothing drawn; the selection overlay labels them "reference"). UI: checkbox + log-scale
magnification slider in Render Settings; per-axis sigmas shown for the selected camera.

### UBO Layout (std140)
- **ViewProjection**: `view`, `projection`, `viewProjection` (mat4) + `cameraPos` (vec3)
- **Lighting**: `lightDirection` (vec3), `lightIntensity` (float), `lightColor` (vec3), `ambientStrength` (float), `ambientColor` (vec3)

### Picker System
Off-screen FBO with `R32UI` color texture + depth renderbuffer. `PickPrimitiveAt(screenPos, radius)` renders primitive IDs, reads back to identify clicked point/triangle/camera. Returns index + triangle corner points + is-point flag.

### Sub-Mesh Management
Visible meshes are packed into shared GPU buffers. `meshFaceCounts` stores cumulative sub-mesh face offsets, `meshTextureIndices` selects each sub-mesh's texture (or `NO_ID`), and layer-local/global face maps preserve picking and selection after texture-based face reordering. Each sub-mesh can be independently toggled via `Window::meshSubMeshVisible`.

### Compare View (A|B)
`Window::compareMode` renders side-A layers left and side-B layers right of a vertical divider in one of two modes:

- **Swipe** (`COMPARE_SWIPE`): both passes share the full-window projection and differ only by the scissor rectangle and the renderer layer pass filter (`Renderer::SetLayerPassFilter`), so aligned scenes line up pixel-exact across the draggable divider (`Window::compareSplitPos`).
- **Split** (`COMPARE_SPLIT`): two equal side-by-side viewports (`Window::GetCompareViewport`), each pass rendered with its own half-window projection (`Renderer::UpdateViewProjection` re-primes the view-projection UBO per pass), so each scene appears centered in its own full frustum.

Camera synchronization (`Window::compareSyncCameras`, default on) renders both sides from the same `Camera` object, so the views cannot drift apart. Unchecking it copies the current view into a second camera (`Window::cameraB`, driven by a second `ArcballControls`) and routes mouse input to the camera of the viewport under the cursor — the side is latched at button-press (`compareDragSide`) so a drag crossing the divider never switches cameras mid-gesture, and cursor positions are normalized into the side's viewport NDC. The main `camera` always renders the ACTIVE layer's side (poses are swapped when the active layer changes sides, see `Window::UpdateCompareState`), which keeps every active-layer interaction (selection, bbox edit, camera view mode, overlays) on the main camera and thus correct by construction; `Window::GetSize()` reports the framebuffer size independently of the (possibly half-window) camera viewport size.

Per-layer GPU buffer sub-ranges (points, normals, camera EBO point/line blocks, ellipsoid slots, per-sub-mesh layer IDs) make the filtered passes cheap draw-call subsets, with no re-upload when sides change. Active-layer extras (selection, bounds, bbox gizmos, image overlay) draw only in the pass showing the active layer; `PickPrimitiveAt` restricts the pick pass to the layers displayed under the cursor and rasterizes it with that side's camera and viewport. Side assignment lives in `Layer::compareRight` (Layers panel A/B buttons; enabling compare defaults the active layer to A, the rest to B; switching between swipe and split keeps the assignment).

### Layer Alignment
`Scene::AlignLayersToActive()` moves every other layer onto the active layer with a similarity transform (`SimilarityTransform` + `DecomposeSimilarityTransform` from libs/Math, applied via `MVS::Scene::Transform`) estimated from camera centers matched by photo file name, falling back to the preserved SFM image ID. Requires ≥3 shared cameras per layer; aligned layers are marked dirty and all render data is re-uploaded while the current viewpoint is kept.

## Shader System

29 shader files in `shaders/`, organized by render pass:

| Pass | Files |
|------|-------|
| Point cloud | `pointcloud.vert/frag` |
| Point normals | `pointcloudnormals.vert/frag` |
| Mesh (solid/wireframe) | `mesh.vert/frag` |
| Mesh (textured) | `meshtextured.vert/frag` |
| Camera frustums | `camera.vert/frag` |
| Selection highlight | `selection.vert/geom/frag` |
| Selection overlay (2D) | `selectionoverlay.vert/frag` |
| Picker (points) | `picker_points.vert/frag` |
| Picker (mesh) | `picker_mesh.vert/frag` |
| Geometry selection | `geometryselection.vert/frag` |
| Image overlay | `imageoverlay.vert/frag` |
| Bounding box | `bounds.vert/frag` |
| Coordinate axes | `axes.vert/frag` |
| Arcball gizmo | `gizmo.vert/frag` |

The `selection.geom` geometry shader expands line segments into screen-aligned quads for variable-width line rendering.

`Shader` class (`Shader.h`) handles compilation, linking, uniform location caching, and typed uniform setters (Matrix4/3, Vector3/2, float, uint, int, bool).

## Control Systems

Switched via `Window::SetControlMode(ControlMode)`:

### Arcball (`CONTROL_ARCBALL`)
Virtual trackball with states: IDLE, ROTATE, PAN, SCALE, FOV, FOCUS, ZROTATE, TOUCH_MULTI, ANIMATION_FOCUS, ANIMATION_ROTATE. Configurable mouse button/key/operation bindings. Double-click triggers smooth focus animation to clicked point. Gizmo rendering for visual feedback.

### First Person (`CONTROL_FIRST_PERSON`)
Mouse look (yaw/pitch) + WASD movement. Configurable base speed, sprint multiplier, and mouse sensitivity. Mouse wheel adjusts movement speed.

### Selection (`CONTROL_SELECTION`)
Three selection shapes: BOX (rectangular), LASSO (free-form polygon), CIRCLE. Three operations: REPLACE (default), ADD (Shift), SUBTRACT (Ctrl). States: IDLE → SELECTING → SELECTED. Uses 2D geometric tests (point-in-polygon, point-in-circle, point-in-box) with world-to-screen projection.

## Selection & Picking

**Single-click picking**: Renderer FBO renders primitive IDs → readback identifies the owning layer and local point/face index; camera cone picking also scans visible layers. Selecting geometry in another layer activates that layer.

**Multi-select**: SelectionController classifies the active visible layer's point cloud points and mesh triangles against the 2D selection region. Results are stored as active-layer-local indices in `Window::selectionIdx`.

**Actions on selection**: `Scene::RemoveSelectedGeometry()` deletes selected points/triangles. `Scene::SetROIFromSelection()` sets region-of-interest from selection. `Scene::CropToPoints()` extracts sub-scene from selected points.

**Index mapping**: `Scene::ImageIdxMVS2Viewer()` converts between active-layer MVS image indices and Viewer-local image indices (which skip invalid images).

## UI System

ImGui with docking support, GLFW/OpenGL3 backends, persistent `.ini` settings.

### Panels
- Layers (visibility/solo/active, compare off/swipe/split with A|B sides and camera sync, align-to-active), scene info, camera controls, selection controls, render settings
- Console overlay (log output), performance overlay (frame stats)
- Viewport overlay, selection overlay

### Dialogs
- About, help (F1), export, camera info, selection info, save prompt

### Workflow Windows
- EstimateROI, Densify, ReconstructMesh, RefineMesh, TextureMesh, Batch

### Menu
Auto-hiding main menu bar with configurable fade delay (~2s). `UpdateMenuVisibility()` manages show/hide transitions.

## Integrated Workflows

Async MVS pipeline stages executable from the Viewer UI:

| Workflow | Options Struct | Key Parameters |
|----------|---------------|----------------|
| EstimateROI | `EstimateROIWorkflowOptions` | `scaleROI`, `upAxis` |
| Densify | `DensifyWorkflowOptions` | `resolutionLevel`, `numViews`, `minViews`, `fusionMode`, `cropToROI` |
| ReconstructMesh | `ReconstructMeshWorkflowOptions` | `minPointDistance`, `decimateMesh`, `closeHoles`, `smoothSteps` |
| RefineMesh | `RefineMeshWorkflowOptions` | `resolutionLevel`, `maxViews`, `scales`, `regularityWeight` |
| TextureMesh | `TextureMeshWorkflowOptions` | `resolutionLevel`, `ratioDataSmoothness`, `globalSeamLeveling`, `maxTextureSize` |

### State Machine
`WorkflowState`: IDLE → RUNNING → COMPLETED / FAILED. Tracked via atomics (`workflowState`, `currentWorkflowType`, `geometryModified`). Each worker event owns an immutable copy of its options. Batch execution queues workflow types and starts the next stage from main-thread finalization, so stages run sequentially against the same active layer. `workflowHistory` records duration and success for stats display. Protected by `workflowMutex`.

## I/O Interface

### Command-Line Options
```
-i, --input-file      MVS project file (positional)
-l, --layer-file      Additional scene/geometry layer (repeatable)
-g, --geometry-file   Mesh/point-cloud to override existing geometry
    --pose-quality-file  Pose-quality CSV (CreateStructure --export-pose-quality) shown as
                         per-camera uncertainty ellipsoids
-o, --output-file     Output filename for saving
    --export-type     Export format: ply or obj
    --archive-type    Project format: -1=interface, 0=text, 1=binary, 2=compressed
-w, --working-folder  Working directory
-c, --config-file     Options file (default: Viewer.cfg)
    --log-file        Enable file logging
-v, --verbosity       Log verbosity level
```

### Runtime I/O
- **Drag-and-drop**: Files dropped on the window are added as layers
- **Save**: `Scene::Save()` writes the active layer as a self-contained `.mvs` project
- **Export**: export the active layer or merge all visible layer geometry
- **Screenshot**: `Window::RequestScreenshot(path, includeUI)` captures framebuffer

### Track-Based Neighbors
`Scene::PrecomputeTrackBasedNeighbors()` computes per-image neighbor lists with shared 3D point indices, stored as `trackBasedNeighbors` (array of `ViewScoreWithPointsArr`). Used for camera neighbor visualization in the UI.

## Build & Dependencies

**Required packages** (all via vcpkg): GLAD, GLFW3, ImGui, portable-file-dialogs
**Links against**: MVS library (which brings in Common, IO, Math, Eigen, OpenCV, etc.)

### Platform-Specific
- **macOS**: App bundle with `Info.plist.in` template, `.icns` icon, Cocoa framework link for `MacOpenFiles.mm` (ObjC++ bridge for Finder file-open events). `.mm` files skip PCH.
- **Windows**: `WIN32_EXECUTABLE` (no console), `.ico` icon via `.rc` file, `Viewer-fileassoc.reg.in` for file association.
- **Linux**: `.desktop` file, SVG icon in hicolor theme, MIME type registration for `.mvs`/`.dmap` via `openmvs-mime.xml.in`, `update-mime-database` at install time.

## Viewer-Specific Conventions

- **OpenGL error checking**: Wrap all GL calls with `GL_CHECK()` macro from `OpenGLDebug.h`. Use `GL_DEBUG_SCOPE(name)` for RAII scoped checks.
- **Eigen over OpenCV for transforms**: View/projection matrices and camera transforms use `Eigen::Matrix4d`, `Eigen::Vector3d` throughout the Viewer (unlike MVS core which mixes both).
- **Render-on-change**: Default mode only redraws when input occurs. Any code that modifies visual state must call `Window::RequestRedraw()`.
- **Layer identity**: Async jobs and renderer mappings use stable `Layer::id` values, never vector addresses or indices that can change after removal.
- **Working folders**: Activate a layer's own `workingFolder` before loading, saving, or running a workflow; restore/activate the current layer after temporary operations.
- **Image index duality**: `MVS::Scene::images` includes invalid entries; each layer's Viewer `images` array only has valid ones. Always use `Scene::ImageIdxMVS2Viewer()` for the active layer when crossing the boundary.
- **GPU buffer ownership**: `Renderer` owns all VAO/VBO/EBO/UBO/FBO resources. Upload methods are the only path from CPU data to GPU.
- **Control mode exclusivity**: Only one control system (arcball/first-person/selection) is active at a time. Switch via `Window::SetControlMode()`.
