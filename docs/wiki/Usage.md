Next a usage example of the available modules is presented. The walkthrough uses the [Sceaux Castle](https://github.com/openMVG/ImageDataset_SceauxCastle) images, reconstructed end-to-end with the **native OpenMVS pipeline** — sparse Structure-from-Motion (`CreateStructure`), dense reconstruction, meshing, refinement and texturing — without any external SfM dependency. If you prefer another SfM solver (OpenMVG, COLMAP, Metashape, Polycam, …), the corresponding importers are documented in the *Convert SfM scene* sections further down. All output presented here is the original output obtained automatically by the `OpenMVS` pipeline, with no manual manipulation of the results. Pre-built binaries for Windows x64 (with and without CUDA), Ubuntu x64 and macOS arm64 are attached to every tagged release on the [OpenMVS releases page](https://github.com/cdcseacave/openMVS/releases/latest).

All `OpenMVS` binaries support some command line parameters, which are explained in detail if executed with no parameters or with `-h`.

<details>
<summary><strong>Extract Keyframes from a Video</strong></summary>

When starting from video (for example a 360° tour, a drone flyover, or any hand-held capture), the `ExtractKeyframes` module selects a stable, well-spaced subset of frames and writes them out together with a native `.sfm` project containing the initial calibration, per-frame features and the pairwise matches discovered during keyframe selection — so the next step does not have to redo any of that work:

```
ExtractKeyframes -i input.mp4 -o scene_keyframes.sfm -d keyframes
```

Notable options:

- `--overlap-threshold` (default `0.85`) — minimum feature-overlap kept between two consecutive keyframes.
- `--detector-type` (`SIFT` | `AKAZE` | `ORB` | `SIFTGPU`, default `SIFT`) — feature detector used for the overlap estimation.
- `--focal-length` (default `0`, auto-calibrate) — known focal length in pixels; the module will auto-calibrate from the fundamental matrices when left at `0`.
- `--camera-type` (`0` pinhole, `1` spherical, default `0`) — set to `1` for equirectangular 360° video.
- `--cubemap-faces` (`4`, `6`, `8`, `12` or `20`, default `6`) — number of tangent-pinhole faces used internally for feature extraction on spherical frames.
- `--blur-size` (default `0`, disabled) — Gaussian kernel applied to the optical-flow image to reject motion-blurred frames.
- `--refine-calibration` (`0` disabled, `1` two-view, `2` three-view, `3` view-graph; default `3`) — level of intrinsic refinement performed during matching.

Running the native SfM step (see next section) on the resulting `.sfm` produces the sparse reconstruction shown below — the keyframe camera frustums and the triangulated sparse cloud are visualized together:

```
CreateStructure -s scene_keyframes.sfm -o scene.sfm --export-mvs scene.mvs --extract-colors 1
```

![keyframe-driven sparse reconstruction](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_keyframes.jpg)

</details>

<details>
<summary><strong>Sparse Reconstruction with Native SfM</strong></summary>

From either the keyframes produced above or any folder / semicolon-separated list of still images, the `CreateStructure` module performs a full incremental SfM and writes a sparse reconstruction. The `--export-mvs` option additionally writes a ready-to-consume `.mvs` project for the downstream dense-reconstruction step, and `--extract-colors` samples the input images to attach a per-point color to the sparse cloud (needed for any sparse-cloud visualization):

```
CreateStructure -s images_folder -o scene.sfm --export-mvs scene.mvs --extract-colors 1
```

![native sparse reconstruction](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_sparse.jpg)

When the input is a `.sfm` produced by `ExtractKeyframes` or `CreateStructure` with feature extraction and matching only, point `--source` at that file directly: `CreateStructure` loads the saved features and pair matches and skips re-detection.

Notable options:

- `--detector-type` (default `SIFTGPU`) — feature detector; same choices as above.
- `--match-mode` (`-1` skip, `0` exhaustive, `1` vocabulary, `2` sequential; default `1`) — pairwise-matching strategy.
- `--match-sequence-overlap` (default `3`) — sequence overlap when using sequential matching.
- `--vocab-max-pairs` (default `50`) — maximum pairs per image for vocabulary-tree matching.
- `--import-poses-csv` (default `poses.csv`) with `--import-poses-mode` (`0` none, `1` all, `2` extrinsics only, `3` positions only) — seed the reconstruction from a CSV of known poses.
- `--export-poses-csv` — write the recovered poses alongside the scene.
- `--import-openmvg-dir` / `--export-openmvg-dir` — interoperate with OpenMVG feature files.
- `--focal-length` (default `0`, disabled) and `--default-focal-ratio` (default `1.2`, used as `ratio * max(width,height)` when the focal length is unknown) — intrinsic overrides.
- `--extract-colors` (default `false`) — attach image colors to the reconstructed sparse points.

</details>

<details>
<summary><strong>End-to-End Pipeline: Video &rarr; Textured Mesh</strong></summary>

A complete reconstruction starting from a 360° video file:

```
ExtractKeyframes -i pano.mp4 -o scene_keyframes.sfm -d frames --camera-type 1
CreateStructure -s scene_keyframes.sfm -o scene.sfm --export-mvs scene.mvs --extract-colors 1
DensifyPointCloud scene.mvs
ReconstructMesh scene_dense.mvs -p scene_dense.ply
TextureMesh scene_dense.mvs -m scene_dense_mesh.ply
```

![spherical untextured mesh](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_spherical_dense_mesh.jpg)
![spherical reconstruction result](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_spherical_dense.jpg)

Equirectangular images and video are supported end-to-end: the SfM stage processes them natively, and the downstream MVS modules receive an automatic six-face cube-map projection — no manual flat-panorama unwrapping is needed.

</details>

<details>
<summary><strong><code>MvgMvsPipeline.py</code> &mdash; end-to-end one-shot helper (optional)</strong></summary>

The bundled [script](https://github.com/cdcseacave/openMVS/blob/master/scripts/python/MvgMvsPipeline.py) chains a sparse SfM frontend with the full `OpenMVS` dense → mesh → refine → texture pipeline in a single command. Three frontends are supported — the **native OpenMVS** `CreateStructure` (default), **OpenMVG** (incremental or global), and **COLMAP** — selectable through `--preset`. The script auto-discovers `CreateStructure` / `ReconstructMesh` in `PATH` (`OpenMVS`), `openMVG_main_SfMInit_ImageListing` (`OpenMVG`) and `colmap` (`COLMAP`), and only prompts for the folders it cannot find **and** that are actually needed by the chosen preset — so the default native run does not require OpenMVG or COLMAP to be installed.

Default run — native OpenMVS SfM followed by the full OpenMVS dense / mesh / refine / texture pipeline. When `<input>` is a single video file (`.mp4`, `.mov`, `.mkv`, `.avi`, `.webm`, …) instead of an image folder, the script automatically prepends an `ExtractKeyframes` step and points `CreateStructure` at the resulting `scene_keyframes.sfm` so the keyframe-time features and matches are reused:

```
python MvgMvsPipeline.py <images_folder_or_video> <output_folder>
```

Use `--preset` to switch frontend or skip stages. The built-in presets are:

| Preset | Frontend → backend |
|---|---|
| `NATIVE` *(default)* | Native OpenMVS `CreateStructure` → OpenMVS dense / mesh / refine / texture |
| `SEQUENTIAL` | OpenMVG incremental SfM → OpenMVS dense / mesh / refine / texture |
| `GLOBAL` | OpenMVG global SfM → OpenMVS dense / mesh / refine / texture |
| `COLMAP_MVS` | COLMAP feature extraction / matching / mapper / undistort → OpenMVS dense / mesh / refine / texture |
| `COLMAP` | COLMAP only, stopping after image undistortion |
| `MVG_SEQ` / `MVG_GLOBAL` | OpenMVG only, stopping after `openMVG_main_openMVG2openMVS` |
| `MVS` | OpenMVS only — assumes a `scene.mvs` already exists in `<output_folder>/mvs/` |
| `MVS_SGM` | OpenMVS Semi-Global Matching densification only |

Examples:

```
# Native SfM + full OpenMVS backend (default)
python MvgMvsPipeline.py <images_folder> <output_folder>

# Drive the full chain through COLMAP instead of the native SfM
python MvgMvsPipeline.py <images_folder> <output_folder> --preset COLMAP_MVS

# Drive the full chain through OpenMVG (incremental SfM)
python MvgMvsPipeline.py <images_folder> <output_folder> --preset SEQUENTIAL
```

Per-step options can be appended with `--<step-number> <key> <value>` (drop the `-` prefix from the underlying option name). For example, set the `OpenMVG` feature describer to `HIGH` on 8 threads, and the matcher to `ANNL2`:

```
python MvgMvsPipeline.py <images_folder> <output_folder> --preset SEQUENTIAL --1 p HIGH n 8 --2 n ANNL2
```

For the full step / preset / passthrough reference, invoke `-h`:

```
python MvgMvsPipeline.py -h
```

</details>

<details>
<summary><strong>Convert SfM scene from <code>COLMAP</code></strong></summary>

After `COLMAP` finishes calibrating and stitching the input images, the undistorted cameras and images must be created:
```
colmap image_undistorter --image_path <images_path> --input_path sparse/0 --output_path dense --output_type COLMAP
```
The undistorted camera poses and images, plus the sparse point-cloud generated by `COLMAP` can be imported by `OpenMVS` into project `scene.mvs`:

```
InterfaceCOLMAP -i dense -o scene.mvs --image-folder dense/images
```

</details>

<details>
<summary><strong>Convert SfM scene from <code>OpenMVG</code></strong></summary>

After all camera views are calibrated and stitched, `OpenMVG` will generate by default the `sfm_data.bin` file containing camera poses and the sparse point-cloud. Using the exporter tool provided by `OpenMVG`, we convert it to the `OpenMVS` project `scene.mvs`:

```
openMVG_main_openMVG2openMVS -i sfm_data.bin -o scene.mvs -d scene_undistorted_images
```

The directory made with the -d switch will store the undistorted images.

</details>

<details>
<summary><strong>Convert SfM scene from <code>Metashape</code> / <code>iTwin Capture Modeler</code> and <code>Polycam</code></strong></summary>

`OpenMVS` has importers for other well known SfM solutions, like `Metashape` (aka `Photoscan`) / `iTwin Capture Modeler` (aka `ContextCapture`) using the BlocksExchange format, and `Polycam` using the raw export scene.

</details>

<details>
<summary><strong>Convert SfM scene from any other format</strong></summary>

`OpenMVS` can process any scene, calibrated by any Structure-from-Motion solver, as long as it receives as input the camera poses, the sparse point-cloud and the corresponding undistorted images. All that needs to be done is to store this information in the `MVS` file format as described in [Interface.h](https://github.com/cdcseacave/openMVS/blob/master/libs/MVS/Interface.h) header file. This file is stand-alone, and can be copied as it is in the SfM solver code and use it directly to export the data in `MVS` format.

A typical sparse point-cloud and camera poses obtained by the previous steps will look like this:

![sparse point-cloud](https://github.com/cdcseacave/openMVS_sample/blob/master/Sparse.jpg)

`Viewer` module can be used to visualize any `OpenMVS` scene file (`MVS` project, `SFM` sparse reconstruction, or individual `DMAP` depth-map) or geometry file (`PLY`, `OBJ`, `OFF`, `GLTF`, `GLB`). The viewer expects the input file either on the command line or to drag & drop it inside the viewer window. `Viewer` is used to create all the screenshots below.

The output of each `OpenMVS` module is displayed by default both on the console and stored in a `LOG` file. Example of the generated `LOG` files can also be found at [OpenMVS_sample](https://github.com/cdcseacave/openMVS_sample).

</details>

<details>
<summary><strong>Dense Point-Cloud Reconstruction (optional)</strong></summary>

If scene parts are missing, the dense reconstruction module can recover them by estimating a dense point-cloud, employing by default a Patch-Match approach:

```
DensifyPointCloud scene.mvs
```

The obtained dense point-cloud (please note the vertex colors are roughly estimated only for visualization, they do not contribute farther down the pipeline):

![dense point-cloud](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_dense.jpg)

The densification module stores, along the dense scene in `MVS` format, also the depth-maps for every processed image in `DMAP` format. `Viewer` module can be used to visualize the `DMAP` files and export them as `PLY` point-clouds.

![dense point-cloud](https://github.com/cdcseacave/openMVS_sample/blob/master/depth0001.dmap.jpg)

</details>

<details>
<summary><strong>Dense Point-Cloud Reconstruction using Semi-Global Matching (optional)</strong></summary>

Alternatively, the dense reconstruction module can estimate a dense point-cloud using Semi-Global Matching (SGM), in two steps: first estimating disparity-maps between all valid image pairs, followed by a second step fusing them in the final point-cloud:

```
DensifyPointCloud scene.mvs --fusion-mode -1
DensifyPointCloud scene.mvs --fusion-mode -2
```

</details>

<details>
<summary><strong>Dense Point-Cloud Reconstruction using available depth-maps (optional)</strong></summary>

The densification module can skip depth-maps estimation if these are known for certain images. In order to use pre-computed depth-maps, all you need to do is to store them in `depthXXXX.dmap` files, where `XXXX` is the ID of the image, using the very simple/portable format explained in [Interface.h](https://github.com/cdcseacave/openMVS/blob/master/libs/MVS/Interface.h#L631). Once depth-maps exported as `DMAP` files, simply run `DensifyPointCloud` as usual, and it will only estimate missing depth-maps, and continue by fusing them in a dense point-cloud.

</details>

<details>
<summary><strong>Rough Mesh Reconstruction</strong></summary>

The sparse or dense point-cloud obtained in the previous steps is used as the input of the mesh reconstruction module:

```
ReconstructMesh scene_dense.mvs -p scene_dense.ply
```

The obtained mesh:

![rough mesh](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_dense_mesh.jpg)

</details>

<details>
<summary><strong>Mesh Refinement (optional)</strong></summary>

The mesh obtained either from the sparse or dense point-cloud can be further refined to recover all fine details or even bigger missing parts. Next the rough mesh obtained only from the sparse point-cloud is refined:

```
RefineMesh scene.mvs -m scene_mesh.ply -o scene_mesh_refine.mvs
```

The mesh before and after refinement:

![rough mesh](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_mesh.jpg)
![refined mesh](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_mesh_refine.jpg)

Similarly, the rough mesh obtained from the dense point-cloud can be refined:

```
RefineMesh scene_dense.mvs -m scene_dense_mesh.ply -o scene_dense_mesh_refine.mvs --scales 1 --max-face-area 16
```

The mesh before and after refinement:

![rough mesh](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_dense_mesh.jpg)
![refined mesh](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_dense_mesh_refine.jpg)
</details>

<details>
<summary><strong>Mesh Texturing</strong></summary>

The mesh obtained in the previous steps is used as the input of the mesh texturing module:

```
TextureMesh scene_dense.mvs -m scene_dense_mesh_refine.ply -o scene_dense_mesh_refine_texture.mvs
```

The obtained mesh plus texture:

![textured mesh](https://github.com/cdcseacave/openMVS_sample/blob/master/scene_dense_mesh_refine_texture.jpg)

Note that the triangles textured in orange (default) are not visible in any of the input images, and can be colored differently or removed.

</details>

<details>
<summary><strong>Exporting and Viewing Results</strong></summary>

Each of the above commands also writes a `PLY` file that can be used with many third-party tools. The `Viewer` can additionally export the loaded `MVS` projects to `PLY`, `OBJ` or `glTF` (`.glb`). For batch / scripted export — including web-ready formats — use the `TransformScene` app:

```
TransformScene scene_dense_mesh_refine_texture.mvs --convert 1 --export-type glb     # → .glb
TransformScene scene_dense_mesh_refine_texture.mvs --convert 1 --export-type gltf    # → .gltf (text + bin sidecars)
TransformScene scene_dense.mvs                     --convert 1 --export-type potree  # → folder of Potree 2.0 tiles
```

<details>
<summary><strong>glTF (mesh + point-cloud, ASCII or binary)</strong></summary>

`glTF` is supported end-to-end for both **meshes** and **point clouds**, in both directions. Files with the `.gltf` extension are ASCII glTF (with binary buffers in sidecar files); `.glb` is the self-contained binary variant. Loading and saving auto-detect the extension and round-trip vertices, faces, vertex colors / normals (point clouds) and the diffuse texture map (textured meshes).

- Library entry points: `MVS::Mesh::LoadGLTF` / `MVS::Mesh::SaveGLTF` and `MVS::PointCloud::LoadGLTF` / `MVS::PointCloud::SaveGLTF` ([`libs/MVS/Mesh.cpp`](https://github.com/cdcseacave/openMVS/blob/master/libs/MVS/Mesh.cpp), [`libs/MVS/PointCloud.cpp`](https://github.com/cdcseacave/openMVS/blob/master/libs/MVS/PointCloud.cpp)). Underlying serializer is the header-only `tiny_gltf` library.
- App exposure: `TextureMesh --export-type glb|gltf`, `TransformScene --export-type glb|gltf`, and the `Viewer` (`File ▸ Export…`, or `--export-type` on the CLI). `ReconstructMesh` / `RefineMesh` also produce glTF when the output filename has a `.gltf` / `.glb` extension (the extension drives `Mesh::Save`'s format dispatch, regardless of `--export-type`).
- Drag-and-drop and `Viewer -i scene.glb` are both supported for inspection.

</details>

<details>
<summary><strong>Potree (web-streamable point-cloud LOD octree)</strong></summary>

Dense point-clouds can be exported to the **Potree 2.0** out-of-core tile format — a multi-resolution octree (`metadata.json` + `hierarchy.bin` + `octree.bin`) consumable by the [Potree](https://potree.org) web viewer. This format is point-cloud only; it is not produced from a mesh.

Trigger the export from `TransformScene` (or by saving a `.potree`-extension file from any program that calls `PointCloud::Save`):

```
TransformScene scene_dense.mvs --convert 1 --export-type potree
```

The resulting `scene_dense.potree/` directory can be served and inspected in a browser with the bundled helper:

```
python scripts/python/potree_server.py scene_dense.potree --browser
```

The script starts a small static HTTP server on port `8080` (override with `--port`) and serves a self-contained HTML page that wires up [Potree.js](https://github.com/potree/potree) from a CDN, opens the cloud with EDL shading enabled, sets a 2M point budget and fits the camera to the data. Press `Ctrl+C` to stop.

</details>

<details>
<summary><strong>The Viewer App</strong></summary>

The `Viewer` is a full-featured interactive GUI for inspecting, editing and exporting OpenMVS projects. It doubles as a hub from which the MVS pipeline can be launched step-by-step on the currently loaded scene.

#### Launching

```
Viewer [--input-file|-i] scene.mvs
```

The input file (or any of the supported formats listed below) can be passed on the command line or dragged and dropped onto the viewer window at any time.

**Supported input formats:** `.mvs` (native scene), `.sfm` (native SfM project), `.dmap` (single depth-map), `.ply`, `.obj`, `.gltf`, `.glb`.

**Viewer-specific CLI options:**

- `--input-file, -i` — project file containing cameras and geometry.
- `--geometry-file, -g` — external mesh or point-cloud that overlays / replaces the scene geometry.
- `--output-file, -o` — output filename for programmatic mesh export.
- `--export-type` — export format override (`ply` or `obj`).
- `--max-memory` — hard memory cap in MB (`0` = unlimited).
- `--screenshot-file, -S` — render the scene off-screen to this image file and exit, without opening the interactive window.
- `--view-file` — optional viewpoint for the screenshot: a transform file of 12 or 16 whitespace-separated values, row-major, interpreted as a camera-to-world pose (columns are the camera X, Y, Z axes in world space, the last column is the camera center).
- `--view-camera` — alternative viewpoint for the screenshot: index of a scene camera view point to use (`-1` disables it). Ignored when `--view-file` is given.
- `--screenshot-show` — which layers to render in the screenshot, as a string of flags: `p` point-cloud, `m` mesh, `t` textured, `c` cameras, `w` wireframe, `b` bounding-box, `u` UI (e.g. `p`, `m`, `mt`). When omitted the interactive defaults are kept.

#### GUI

The user interface is built on Dear ImGui and organized into menus, dockable panels and overlays:

- **File menu** — Open / Save / Save As / Close / Export, plus screenshot capture (with or without UI visible).
- **View menu** — toggles Scene Info, Camera Info, Camera Controls, Selection, Render Settings, Bounding Box, Console and the Performance / Workflow / Viewport / Selection overlays.
- **Render toggles** — one-key switches for Point-cloud, Mesh, Cameras, Wireframe, Textured, and Bounding-box visibility.
- **Workflow menu** — launches the OpenMVS pipeline steps on the loaded scene: *Estimate ROI*, *Densify*, *Reconstruct Mesh*, *Refine*, *Texture*. Each opens a parameter panel with the corresponding module's options.
- **Help / About dialogs** — `F1` shows the complete in-app keybinding reference.

#### Mouse controls

The viewer supports two navigation modes, switched with `Tab`:

- **Arcball mode** (default) — left-drag rotates around the focus point, middle-drag (or `Ctrl` + scroll) pans, scroll-wheel zooms.
- **First-person mode** — `W` / `A` / `S` / `D` move the camera, left-drag looks around, scroll adjusts movement speed.

In **selection mode** (`G`), left-drag performs a rectangle pick and a single left-click selects or deselects points or faces — selected vertex indices are printed to the console. In the **bounding-box editor** (`Shift+B`) the 8 corner handles, 6 face handles and 3 rotation rings can be dragged directly in the viewport to resize, translate and orient the region of interest.

#### Keyboard shortcuts

File & app
- `Ctrl+O` open · `Ctrl+S` save · `Ctrl+Shift+S` save as · `Ctrl+X` screenshot · `F11` fullscreen · `ESC` close window · `F1` help

Navigation
- `Tab` switch arcball ↔ first-person · `R` reset view · `←` / `→` step to previous / next camera pose

Display toggles
- `P` point cloud · `M` mesh · `C` cameras · `W` wireframe · `T` textured · `B` bounding box

Panels (Shift + letter)
- `Shift+A` Scene Info · `Shift+Q` Camera Info · `Shift+C` Camera Controls · `Shift+S` Selection · `Shift+R` Render Settings · `Shift+B` Bounding-Box editor

Tools
- `G` toggle selection mode · `Ctrl+B` run ROI estimation

#### Headline features

- **ROI editing.** Press `Shift+B` to overlay an interactive oriented bounding box on the scene; drag the corner, face or rotation handles to define the region that subsequent pipeline stages will focus on. ROI can also be auto-estimated from the scene (`Ctrl+B`) or loaded from a file.
- **Camera trajectory colouring.** Camera frustums are coloured along a Jet colormap according to their index in the capture sequence — blue at the start, red at the end — making it easy to spot loop closures, gaps or ordering issues. Cameras can be rendered as full frustums or as simple dots; the currently selected camera and its neighbours are highlighted distinctly.
- **Camera-pose navigation.** The left / right arrow keys step through the scene's registered views; `Shift+Q` pins a panel showing the selected camera's intrinsics and extrinsics.
- **Depth-map inspection.** Load a `.dmap` file directly (command-line, `Ctrl+O` or drag-and-drop) to render the depth map as a coloured 3D surface; the viewer also exports depth maps to `.ply` for use in external tools.
- **In-GUI pipeline.** The *Workflow* menu launches `DensifyPointCloud`, `ReconstructMesh`, `RefineMesh` and `TextureMesh` on the loaded scene without leaving the viewer, with the result live-reloaded once each step completes.
- **Export.** `File ▸ Export...` opens a dialog that writes the current point-cloud and/or mesh to `.ply`, `.obj` or `.glb`; the format can also be forced with the `--export-type` CLI override. Screenshots can be captured to `.png`, `.jpg` or `.jxl` interactively with `Ctrl+X`, or non-interactively (off-screen, for reproducible documentation) with the `--screenshot-file` / `--view-file` command-line options described above.

</details>

</details>

