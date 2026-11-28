# Image Import, Cameras and Known-Pose Import

## Purpose and scope

`Scene::Import` turns a folder or list of image files into the scene's images and cameras: it reads each image's header and
EXIF metadata, chooses a camera model and an initial focal length for it, shares identical cameras, optionally imports camera
poses and intrinsics from a file, and applies forced intrinsics. Nothing is decoded here beyond the header and the EXIF
block. The code is `Scene::Import` (`Scene.cpp`), `Image::LoadMetadata` (`Image.cpp`), `Camera.h` and `PoseIO.h/cpp`. What
the pipeline later does with the cameras is in `TwoViewVerificationAndLens.md` (trust, lens fit, focal check) and
`BundleAdjustment.md` (intrinsic refinement); spherical cameras are in `SphericalCameras.md`.

## Image list

`--source` is a folder, a semicolon-separated list of files, or a single `.sfm` scene (resumed with `Scene::Load`).
Folder entries with the extensions jpg, jpeg, png, tif, tiff, jxl, exr, webp, heic, heif are collected and sorted: by
numeric file stem when every compared stem is a number, else by path. This order is the *import order*, which the
sequential pair selection relies on (`PairSelection.md`). At least two images are required.

## Working orientation

An image whose stored width is smaller than its height is a *rotated* image: the scene works on it rotated 90 degrees
clockwise, so every working raster is landscape (`View::ToWorkingOrientation`); features, intrinsics and poses all live in
that raster. For EXIF orientations 5 to 8, which `cv::imread` applies by swapping the axes, the flag is set to match the
decoded pixel layout. For HEIF the container's own rotation is
already applied by the decoder, so the EXIF orientation tag is ignored there to avoid rotating twice.

## Camera models and their selection

| model | parameters | notes |
|---|---|---|
| `PinholeCamera` | `fx, fy, cx, cy`; radial `k1, k2, k3`; tangential `p1, p2`; optional `k4..k6` behind `useAdditionalDistortion` | Brown-Conrady; the principal point defaults to `((w-1)/2, (h-1)/2)` (integer coordinates are pixel centres); a `trustIntrinsics` flag |
| `SphericalCamera` | image size only (width = 2 x height, asserted by the constructor) | equirectangular; always trusted; no intrinsics to refine |

`--camera-type`: -1 (default) reads the model from the metadata (EXIF `ProjectionType == 2`, equirectangular, gives a
spherical camera, anything else a pinhole), 0 forces pinhole, 1 forces spherical (a panorama whose file carries no
projection tag).

## Initial focal length and trust

A pinhole camera's focal is the first usable source of this ladder; a value no lens can carry (a ratio to the long side
`max(w, h)` under 0.2 or over 100, or a non-finite value) is skipped and the next source is tried:

| rank | source | trusted |
|---|---|---|
| 1 | EXIF `FocalLength` (mm) times the focal-plane resolution (pixels per unit, with its unit), scaled to the image size; also gives the sensor size | yes |
| 2 | EXIF 35 mm-equivalent focal: `diagonal * f35 / 43.27` | no |
| 3 | EXIF calibration focal in pixels | yes |
| 4 | `--default-focal-ratio` (1.2) times `max(w, h)` | no |

The 35 mm equivalent describes the lens on the full sensor, which a cropped video frame does not use, so it is a guess and
the lens stage fits the focal from the pairs instead (`TwoViewVerificationAndLens.md`). The camera's name and model are
the EXIF make and model. Incomplete or implausible focal-plane metadata drops the rung entirely and falls through. EXIF
GPS position, accuracy and yaw, pitch, roll are stored in the image's metadata for the GPS alignment
(`--align-gps-threshold`, default 5 m, 0 = off).

**Trusted intrinsics** are measured, forced or calibrated; consumers branch on them. A pair of two trusted cameras is
verified as a calibrated relative pose, otherwise through F. The lens stage fits the lens of trusted pinhole cameras that
have none and corrects a trusted focal the pairs clearly reject; an untrusted camera with no lens is first made trusted at
its import focal, which the fit then estimates with no bar, and the view-graph calibration estimates what remains untrusted
(`TwoViewVerificationAndLens.md`). The bundle adjustment refines intrinsics according to `--refine-intrinsics`. The flag
lives on the camera, so one decision holds for every image that shares it.

## Camera sharing

After the (optional) pose import, cameras with identical content are merged into one shared camera: type, size, `fx, fy, cx,
cy`, every distortion coefficient, the trust flag, the camera name and model, and the sensor size must all match exactly. A
capture whose images declare identical intrinsics therefore uses one camera, which is what lets the lens fit and the focal
calibration pool all its pairs.

## Forced intrinsics

`--focal-length` (`-f`, pixels, 0 = off), `--k1` and `--k2` (0 = not used) force the focal and the first two radial
coefficients, for the images named by `--image-indices` (ranges such as `0 5-10 15`; empty = all images). A forced camera
sets `fx = fy`, sets the coefficients, and is marked trusted, so the lens stage does not fit its focal or its lens (a forced
`--k1` or `--k2` makes it a given lens: held in the fit, never re-fitted). When a selected image shares its camera with an
unselected one, the camera is cloned for the selected images only. Spherical images refuse forced parameters. When a scene
is resumed from a `.sfm`, cameras are already shared, and intrinsics from a poses file are ignored (poses are still imported).

## Known-pose import

`--import-poses-file` (default `poses.csv`) with `--import-poses-mode`: 0 none, 1 poses and intrinsics, 2 poses only,
3 camera positions only. Modes 1 and 2 with a file select the known-poses reconstruction path (`HasKnownPoses`;
`SfMPipeline.md`) and, unless `--match-mode` is given, the KNOWN_POSES pair selection (`PairSelection.md`); mode 3 only
sets camera centres. The import runs before the cameras are shared.

- **OpenMVS CSV** (`ExportPosesCSV`/`ImportPosesCSV`): per row `filename stem, fx, fy, cx, cy, qx, qy, qz, qw` (world-to-
  camera quaternion), `Cx, Cy, Cz` (camera centre), `score` (0 invalid or unknown to 1 accurate; a score of 0 or less
  leaves the image unposed). Rows match images by file-name stem, case-insensitively; an ambiguous stem is rejected.
  Mode 1 sets the intrinsics of a pinhole image and marks them trusted.
- **frames.json** (Polycam-style): an array of `{name, transform[16], params?}` with a column-major 4x4 camera-to-world
  `transform`. Entries match images by full file name, then by stem, both case-insensitively; ambiguous names and
  duplicates are rejected; images without an entry stay unposed. The rotation block is re-orthonormalized when within 1e-3
  of orthonormal, else the entry is rejected. `params` (mode 1 only) declares an OPENCV pinhole model (`w, h, fx, fy, cx,
  cy, k1, k2, p1, p2`) at a possibly different resolution or orientation: it is rescaled by the width ratio (the height
  ratio must agree within 0.1%), rotated with the image when the declared raster is the transpose of the working one, and
  marked trusted; without it the EXIF intrinsics are kept.
- **EXIF-rotated images.** The imported pose describes the image as stored; the importer composes the in-plane rotation
  (`Rz(+90 deg)`) into the pose.

**Pose frame of a frames.json.** The file does not say which camera axes its `transform` uses (ARKit/OpenGL: X right, Y up,
Z backward; OpenCV: X right, Y down, Z forward; they differ by a half turn about X) nor, for EXIF-rotated images, how many
quarter turns about the optical axis take the camera frame to the working raster. `--known-poses-convention arkit|opencv`
(default auto) fixes the axes; auto imports as ARKit. After matching, `ResolveFramesConvention` /
`DetectFramesConvention` decides among up to eight hypotheses (two axes times four quarter turns when rotated images are
present): the imported relative rotations are compared with the match-verified ones and the lowest median angular error
wins if it is 3 times better than the runner-up; otherwise the highest-weighted pairs are triangulated under every
hypothesis and the one with most cheirality-positive, low-reprojection inliers wins by the same margin. An inconclusive
result fails the run and asks for an explicit convention. The winning frame rewrites the poses' rotations
(`ApplyFramesPoseFrame`), leaving the camera centres alone.

## Parameters and defaults

| option | default |
|---|---|
| `--camera-type` | -1 (from metadata) |
| `--default-focal-ratio` | 1.2 |
| `--focal-length`, `-f` | 0 (off) |
| `--k1`, `--k2` | 0 (not used) |
| `--image-indices` | all images |
| `--import-poses-file` | `poses.csv` |
| `--import-poses-mode` | 0 (none) |
| `--known-poses-convention` | auto |
| `--align-gps-threshold` | 5 m |

## Invariants

- Every image has a camera after import, shared with identical ones; an image whose header cannot be read is reported and
  left without one.
- A `SphericalCamera` has width exactly twice its height; a spherical image of another shape fails its metadata load
  with an error naming it and stays without a camera.
- A pose from a file is applied at most once and only to an image matched unambiguously.
- A pose-frame detection that cannot decide never guesses.

## Open items

- `--k1` or `--k2` without `--focal-length` marks the camera trusted although its focal still comes from the ladder above.
