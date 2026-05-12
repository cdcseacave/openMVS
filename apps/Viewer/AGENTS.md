# Viewer Application

Interactive 3D visualization and workflow execution tool for OpenMVS scenes. Lives in the `VIEWER` namespace.

## Architecture

### Class Composition

```
Scene (top-level container)
├── MVS::Scene          — core photogrammetry data
├── ImageArr            — valid scene photographs
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
`Scene::thread` processes `Scene::events` queue for async workflow execution. Workflow results are finalized on the main thread via `CheckWorkflowCompletion()` → `FinalizeWorkflow()`.

## Rendering Pipeline

### Data Upload (CPU → GPU)
- `UploadPointCloud(MVS::PointCloud, normalLength)` — points + optional normals
- `UploadMesh(MVS::Mesh)` — vertices, faces, normals, texcoords, sub-mesh partitioning
- `UploadCameras(Window)` — camera frustum line geometry
- `UploadSelection(Window)` — highlighted primitive geometry
- `UploadBounds(MVS::Scene)` — AABB wireframe

### Frame Cycle
```
BeginFrame(camera, clearColor)    — upload ViewProjection UBO
SetLighting(dir, intensity, color) — upload Lighting UBO
RenderPointCloud()                 — GL_POINTS with dynamic point size
RenderMesh()                       — solid + wireframe + textured variants
RenderCameras()                    — frustum line rendering
RenderImageOverlays()              — 3D photo planes with per-image opacity
RenderSelection()                  — highlighted primitives
RenderSelectionOverlay()           — 2D screen-space selection UI
RenderBounds()                     — AABB wireframe
RenderCoordinateAxes()
RenderArcballGizmos()              — virtual trackball visualization
EndFrame()
```

### UBO Layout (std140)
- **ViewProjection**: `view`, `projection`, `viewProjection` (mat4) + `cameraPos` (vec3)
- **Lighting**: `lightDirection` (vec3), `lightIntensity` (float), `lightColor` (vec3), `ambientStrength` (float), `ambientColor` (vec3)

### Picker System
Off-screen FBO with `R32UI` color texture + depth renderbuffer. `PickPrimitiveAt(screenPos, radius)` renders primitive IDs, reads back to identify clicked point/triangle/camera. Returns index + triangle corner points + is-point flag.

### Sub-Mesh Management
Meshes are partitioned by texture into sub-meshes via `mapFaceSubsetIndices`/`mapSubsetFaceIndices`/`meshFaceCounts`. Each sub-mesh can be independently toggled visible (`Window::meshSubMeshVisible`).

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

**Single-click picking**: Renderer FBO renders primitive IDs → readback identifies SEL_POINT, SEL_TRIANGLE, or SEL_CAMERA.

**Multi-select**: SelectionController classifies visible point cloud points and mesh triangles against 2D selection region. Results stored in `Window::selectionIdx`.

**Actions on selection**: `Scene::RemoveSelectedGeometry()` deletes selected points/triangles. `Scene::SetROIFromSelection()` sets region-of-interest from selection. `Scene::CropToPoints()` extracts sub-scene from selected points.

**Index mapping**: `Scene::ImageIdxMVS2Viewer()` converts between MVS image indices and Viewer-local image array indices (which skip invalid images).

## UI System

ImGui with docking support, GLFW/OpenGL3 backends, persistent `.ini` settings.

### Panels
- Scene info, camera controls, selection controls, render settings
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
`WorkflowState`: IDLE → RUNNING → COMPLETED / FAILED. Tracked via atomics (`workflowState`, `currentWorkflowType`, `geometryModified`). `workflowHistory` records duration and success for stats display. Protected by `workflowMutex`.

## I/O Interface

### Command-Line Options
```
-i, --input-file     MVS project file (positional)
-g, --geometry-file   Mesh/point-cloud to override existing geometry
-o, --output-file     Output filename for saving
    --export-type     Export format: ply or obj
    --archive-type    Project format: -1=interface, 0=text, 1=binary, 2=compressed
-w, --working-folder  Working directory
-c, --config-file     Options file (default: Viewer.cfg)
    --log-file        Enable file logging
-v, --verbosity       Log verbosity level
```

### Runtime I/O
- **Drag-and-drop**: Files dropped on window trigger `Scene::Open()`
- **Save**: `Scene::Save()` writes `.mvs` project
- **Export**: `Scene::Export()` writes `.ply` or `.obj`
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
- **Image index duality**: MVS::Scene image indices include invalid entries; Viewer `images` array only has valid ones. Always use `Scene::ImageIdxMVS2Viewer()` when crossing the boundary.
- **GPU buffer ownership**: `Renderer` owns all VAO/VBO/EBO/UBO/FBO resources. Upload methods are the only path from CPU data to GPU.
- **Control mode exclusivity**: Only one control system (arcball/first-person/selection) is active at a time. Switch via `Window::SetControlMode()`.
