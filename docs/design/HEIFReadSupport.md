# HEIF/HEIC read support — implementation plan

**Status: IMPLEMENTED** — see "Deviations found during implementation" at the end for the
four places reality differed from this plan.

## Goal & scope

Read Apple-style `.heic` / `.heif` images everywhere OpenMVS reads photographs:
SfM feature extraction and metadata (focal prior, GPS), MVS densification,
InterfaceMVS undistortion. **Write support is out of scope** (readers return
`false` from the Write* methods). AVIF read is a cheap optional bonus at the end.

## Current architecture (what the change plugs into)

There are two independent pixel-decode paths plus one EXIF path; HEIF must cover all three:

| Path | Entry point | Used by | HEIF action |
|---|---|---|---|
| SEACAVE `CImage::Create` extension factory | `libs/IO/Image.cpp:870` | `MVS::Image::ReloadImage` (densify/fusion/texturing), `SFM::Image::LoadMetadata` (header only), `SFM::UndistortDMAP` (`libs/SFM/InterfaceMVS.cpp:281`) | new `CImageHEIF` reader |
| `SEACAVE::LoadImage` → `cv::imread` | `libs/Common/Types.inl:3003` | `SFM::Image::LoadPixels` (feature-extraction pixels), `TImage::Load` | OpenCV has **no** HEIF codec (vcpkg `opencv4` features checked: none) → CImage-based fallback in `SFM::Image::LoadPixels` |
| TinyEXIF parse of the raw JPEG stream | `libs/SFM/Image.cpp:152` | `SFM::Image::LoadMetadata` (focal ladder F1–F3, GPS, orientation) | EXIF blob bridge via libheif metadata API + `TinyEXIF::parseFromEXIFSegment` (public, designed for exactly this) |

Precedent: JPEG-XL support = `CImageJXL` in libs/IO **plus** OpenCV's `jpegxl`
vcpkg feature for the `cv::imread` path. HEIF has no OpenCV feature, hence the
explicit fallback in step 5.

## Step 1 — dependency + build wiring (~30 min)

1. `vcpkg.json`: add to dependencies
   ```json
   { "name": "libheif", "default-features": false }
   ```
   - `default-features: false` is **essential**: the port's default `hevc`
     feature is the **x265 encoder (GPL-2.0)** — unneeded for read-only and a
     license downgrade. The HEVC **decoder** `libde265` (LGPL-3.0, same as
     libheif itself) is a hard dependency and always present.
   - Optional AVIF read later: add `"features": ["aom"]`.
   - After install, verify `vcpkg_installed` contains **no x265**.
2. `build/Templates/ConfigLocal.h.in`: add `#cmakedefine _USE_HEIF` next to
   `_USE_JXL` (line 41). This is how the CMake flag becomes a preprocessor define.
3. `libs/IO/CMakeLists.txt` (mirror the JXL block at line 63–72):
   ```cmake
   pkg_check_modules_fullpath_libs(HEIF libheif)
   if(HEIF_FOUND)
       SET(_USE_HEIF TRUE CACHE INTERNAL "")
       set(HEIF_LIBRARIES ${HEIF_FULLPATH_LIBRARIES})
   else()
       SET(HEIF_LIBRARIES "")
   endif()
   ```
   and append `${HEIF_LIBRARIES}` to `TARGET_LINK_LIBRARIES(IO ...)` (line 96).
   vcpkg's libheif ships a patched `libheif.pc`, and `pkgconf` is already in
   the manifest, so the JXL discovery pattern carries over unchanged.
4. `libs/IO/Common.h` (lines 57–58 / 83–84 pattern): map `_USE_HEIF` →
   `_IMAGE_HEIF`; include `ImageHEIF.h` under `#ifdef _IMAGE_HEIF`.
5. New files ⇒ run `cmake make` to reconfigure (sources are GLOB'd into the vcxprojs).

## Step 2 — `CImageHEIF` reader (`libs/IO/ImageHEIF.h/.cpp`, ~250 lines)

Model on `ImageJXL` (312 lines total), but simpler: libheif's high-level API is
whole-image, no streaming state machine.

Header mirrors `ImageJXL.h`: `class IO_API CImageHEIF : public CImage` with
`Close/ReadHeader/ReadData/WriteHeader/WriteData` + opaque `void* m_state`.

State kept between `ReadHeader` and `ReadData`:
```cpp
struct HeifState {
    std::vector<uint8_t> fileData; // whole compressed file
    heif_context* ctx = nullptr;
    heif_image_handle* handle = nullptr; // primary image
};
```

`ReadHeader()`:
1. Read the entire stream into `fileData` (`m_pStream->getInputStream()`,
   `setPos(0)` first — HEIC files are a few MB, whole-file buffering is fine).
2. `heif_context_alloc()` + `heif_context_read_from_memory_without_copy()`.
3. `heif_context_get_primary_image_handle()` — multi-image containers
   (bursts, Live Photos, thumbnails, aux depth) resolve to the primary image.
4. Dimensions: `heif_image_handle_get_width/height` — these already reflect
   the container `irot`/`imir` transforms that libheif applies at decode, so
   header dims and decoded dims are consistent. Set
   `m_width/m_height/m_dataWidth/m_dataHeight`, `m_numLevels = 1`.
5. Native format: `heif_image_handle_has_alpha_channel()` →
   `PF_R8G8B8A8`/stride 4 (`heif_chroma_interleaved_RGBA`), else
   `PF_R8G8B8`/stride 3 (`heif_chroma_interleaved_RGB`); `m_lineWidth = m_width*m_stride`.
   - **Channel-order caveat**: `CImageJXL` declares `PF_B8G8R8` for 3-channel
     output — do NOT trust naming; pick whichever PF constant makes the
     JPG-vs-HEIC parity test (step 6) pass.
   - 10/12-bit HDR sources: libheif converts to 8-bit when 8-bit interleaved
     chroma is requested. Acceptable for SfM/MVS; a 16-bit PF path is future work.

`ReadData(pData, dataFormat, nStride, lineWidth)`:
1. `heif_decode_image(handle, &img, heif_colorspace_RGB, <chroma from header>, nullptr)`
   (transforms applied by default — do NOT set `ignore_transformations`).
2. `heif_image_get_plane_readonly(img, heif_channel_interleaved, &srcStride)` —
   note `srcStride` may exceed `m_width*m_stride`; copy row-wise.
3. Row loop exactly like `CImageJXL::ReadData`: direct `memcpy` per row when
   `dataFormat == m_format && nStride == m_stride`, else
   `FilterFormat(dst, dataFormat, nStride, src, m_format, m_stride, m_width)` per row.
4. `heif_image_release(img)` at the end of ReadData; `handle`/`ctx` freed in `Close()`.
5. Thread safety: each reader instance owns its own `heif_context` — safe with
   the pipeline's multi-threaded image loads (ImageCache OpenMP prefetch).
   libde265 threads internally; leave defaults, and only if oversubscription
   shows up set `heif_decoding_options::max_threads = 1`.

`WriteHeader/WriteData`: `LOG(...); return false;` — read-only by design (the
GPL encoder is deliberately not built).

EXIF accessor consumed by step 4:
```cpp
// raw EXIF payload, normalized to start with "Exif\0\0" (TinyEXIF segment form)
virtual bool GetMetadataEXIF(std::vector<uint8_t>& blob) const override;
```
Implementation: `heif_image_handle_get_list_of_metadata_block_IDs(handle, "Exif", ...)`
+ `heif_image_handle_get_metadata`. The stored item = 4-byte big-endian
`exif_tiff_header_offset` followed by the EXIF payload (usually
`"Exif\0\0"+TIFF`, sometimes bare TIFF). Normalize: skip the 4-byte prefix,
locate/prepend `"Exif\0\0"` so the blob satisfies
`TinyEXIF::parseFromEXIFSegment` ("a blob starting with the bytes Exif\0\0",
public API, TinyEXIF.h:97).

## Step 3 — factory dispatch (`libs/IO/Image.cpp:899`, next to JXL)

```cpp
#ifdef _IMAGE_HEIF
else if (_tcsncicmp(fext, _T(".heic"), 5) == 0 || _tcsncicmp(fext, _T(".heif"), 5) == 0)
    pImage = new CImageHEIF();
#endif
```
Comparison is already case-insensitive (`.HEIC` iPhone naming covered).

## Step 4 — EXIF bridge + orientation normalization (~40 lines)

1. `CImage` base (`libs/IO/Image.h`): add
   `virtual bool GetMetadataEXIF(std::vector<uint8_t>&) const { return false; }` —
   non-breaking default; only `CImageHEIF` overrides.
2. `SFM::Image::LoadMetadata` (`libs/SFM/Image.cpp:150`): try the blob first,
   fall back to the existing stream parse:
   ```cpp
   TinyEXIF::EXIFInfo exif;
   bool parsed = false, containerOriented = false;
   std::vector<uint8_t> blob;
   if (pImage->GetMetadataEXIF(blob)) {
       parsed = exif.parseFromEXIFSegment(blob.data(), (unsigned)blob.size()) == TinyEXIF::PARSE_SUCCESS;
       containerOriented = true; // decoder already applied irot/imir
   }
   if (!parsed) {
       IOStreamEXIFWrapper exifStream(pImage->GetStream());
       parsed = exif.parseFrom(exifStream) == TinyEXIF::PARSE_SUCCESS;
   }
   ```
3. **Orientation rule — the #1 correctness hazard of this whole feature.**
   When `containerOriented`, force `exif.Orientation = 1` before the existing
   handling (line 158–162). HEIF stores rotation in container-level
   `irot`/`imir`, which libheif applies during decode and reflects in the
   header dims; many writers ALSO set the EXIF Orientation tag. Honoring it
   would rotate a second time and desync `metadata.rotated` from the actual
   pixels — which feeds the known-poses rotated-image flip (`diag(-1,1,-1)`),
   i.e. a silent pose-import breaker. One line + comment; fixture test locks it.
4. Everything downstream (focal ladder F1→F3, GPS, spherical flag, rotated
   flag) consumes the parsed `EXIFInfo` unchanged.

## Step 5 — SFM pixel-path fallback (`libs/SFM/Image.cpp::LoadPixels`, ~25 lines)

`LoadPixels` → `SEACAVE::LoadImage` → `cv::imread` cannot decode HEIC, and the
fallback cannot live in `libs/Common/Types.inl` (layering: Common ← IO ← SFM).
Add it in `SFM::Image::LoadPixels`, triggered on **imread failure** (not on
extension), so any IO-only format is future-proofed:

```cpp
if (!LoadImage(fileName, pixels, gray ? 1 : -1)) {
    // fallback: route formats OpenCV cannot decode (e.g. HEIC) through CImage
    if (!LoadPixelsViaCImage(fileName, pixels, gray))
        { VERBOSE(...); return false; }
}
```
`LoadPixelsViaCImage`: `CImage::Create` → `ReadHeader` → `ReadData` into a
`CV_8UC3` (or `CV_8UC1` for gray) `cv::Mat`, mirroring the 10 lines of
`MVS::Image::ReadImage` (`libs/MVS/Image.cpp:86`). Channel order must match
what `cv::imread` would produce (BGR) — covered by the parity test.

## Step 6 — tests + fixtures

Binary fixture convention already exists: `apps/Tests/data/images/00000–00003.jpg`
(1.3 MB total). Add:

- `apps/Tests/data/images/heif/sample.heic` — small (≤50 KB, e.g. 256×192)
  fixture converted **offline** from `00000.jpg` (encoding is not built into
  OpenMVS: use macOS `sips -s format heic`, libheif-examples `heif-enc`, or
  ImageMagick), with EXIF grafted (`exiftool -tagsFromFile 00000.jpg sample.heic`).
- A second fixture with rotation stored BOTH as `irot` and EXIF
  Orientation=6 — locks the double-rotation guard forever.

New `ImageIOHEIFTest()` (gated `#ifdef _IMAGE_HEIF`, wired into `UnitTests()`
in `apps/Tests/Tests.cpp`):
1. **Decode parity**: `MVS::Image::ReadImage(sample.heic)` vs the source JPG:
   identical dims, mean |diff| below codec-noise threshold (~3/255), and an
   asserted colored-corner pixel to catch RGB/BGR swaps.
2. **Header-only**: `ReadImageHeader` returns the transformed (post-irot) dims.
3. **EXIF bridge**: `SFM::Image::LoadMetadata` on the fixture → focal within
   tolerance of the JPG's, `metadata.orientation == 1`, `rotated` consistent
   with pixel dims (use the orientation-6 fixture).
4. **SFM pixels**: `Image::LoadPixels` succeeds (exercises the imread→CImage
   fallback), size matches metadata.

## Step 7 — validation, CI, docs

- Windows: vcpkg manifest install pulls libheif+libde265 (small ports, minutes);
  `cmake make` reconfigure; full build; ctest all 3 suites serial.
- End-to-end smoke: convert a small dataset (e.g. SceauxCastle) to HEIC, run
  CreateStructure → DensifyPointCloud in a run subfolder (never the dataset
  root), compare registered-image count and dense-point count to the JPG
  baseline of the same scene.
- Linux/macOS: covered automatically by the vcpkg manifest — watch the first
  CI run for libheif/libde265 port build breakage.
- Docs: add `.heic/.heif` to the supported input formats in
  `docs/wiki/Usage.md`; one line in `libs/IO/README.md`; note the read-only /
  no-x265 licensing rationale in the commit message (vcpkg.json cannot carry
  comments).

## Risks / gotchas

| Risk | Mitigation |
|---|---|
| Double rotation (irot + EXIF tag both set) | `orientation := 1` whenever the EXIF blob came from a container-oriented decoder; orientation-6 fixture test |
| RGB/BGR channel-order mistake (JXL precedent declares `PF_B8G8R8`!) | parity test vs JPG reader on a colored fixture |
| GPL x265 sneaking into the build | `default-features: false`; post-install check for x265 in `vcpkg_installed` |
| H.265 decode patent encumbrance (libde265 is LGPL; patents are separate) | flag to product/legal; fallback plan = per-OS decoders (WIC / ImageIO), ~3× effort, no Linux path |
| 10-bit/HDR HEICs | libheif converts to 8-bit interleaved on request; 16-bit PF path deferred |
| Multi-image containers (bursts / Live Photos / aux depth) | primary image handle only; documented |
| libde265 internal threads × pipeline OpenMP prefetch oversubscription | leave defaults; `decoding_options.max_threads = 1` if it shows up |
| TinyEXIF blob offset quirks (bare-TIFF payloads) | normalization in `GetMetadataEXIF` (skip 4-byte offset, ensure `Exif\0\0` prefix) |

## Sequencing & effort (~1–2 dev-days, ~350–450 lines)

1. Step 1 wiring + step 2 reader skeleton → decode a real iPhone HEIC standalone (½ day)
2. Steps 3 + 5 → pixels flow through both pipeline paths (2h)
3. Step 4 EXIF bridge + orientation rule (2h, incl. blob-offset quirks)
4. Step 6 fixtures + tests (2h)
5. Step 7 validation + CI watch (elapsed)

Bonus after landing: add `"features": ["aom"]` to the libheif dependency and
`.avif` to the factory dispatch — same reader, AVIF read for free.

## Deviations found during implementation

1. **A fourth path was missing from the architecture table above: folder enumeration.**
   `SFM::Scene` lists a source directory against a hardcoded extension whitelist
   (`ext == ".jpg" || ...`). Decoding worked while `CreateStructure -s <folder>` still found
   **zero** images, and every unit test passed regardless because fixtures are opened by
   explicit path. `.heic`/`.heif` had to be added there too. Only the end-to-end run caught it.
2. **`SFM::Image::pixels` is a publish flag, not just a buffer.** `HasPixels()` is
   `!pixels.empty()`, and `FeaturesExtractor` pre-loads the next image on a **detached,
   un-awaited** thread-pool task while the consumer gates on `HasPixels()`. `cv::imread` is
   safe because `LoadImage` assigns the destination only after decoding. The first cut of
   `LoadPixelsViaCImage` did `pixels.create()` then filled row-by-row, publishing a non-empty
   **unfilled** buffer: SIFT ran on zeros (0 features on the prefetched half of the images,
   in a deterministic alternating pattern) and segfaulted under real parallelism. The helper
   must decode into a local `cv::Mat` and assign at the end.
3. **`PF_B8G8R8` is the correct native format, and the reason matters.** `PF_*` names list
   channels most- to least-significant bit, so on little-endian `PF_B8G8R8` is R,G,B *in
   memory* (what libheif emits) and `PF_R8G8B8` is B,G,R (what `cv::imread` produces and what
   the fallback must request). `libs/IO/README.md` used to call `PF_B8G8R8` "OpenCV's native
   order", which is backwards; it has been corrected with a byte-order table and the decisive
   argument — `CImagePNG::ReadData` calls libpng's `png_set_bgr()` precisely when a `PF_B8G8R8`
   file is read into a `PF_R8G8B8` request, and libpng natively emits R,G,B, so `PF_R8G8B8` is
   the order needing the swap.
4. **Real iPhone HEIFs carry an alpha channel, and honouring it silently destroys the gray
   path.** This plan's step 2 says to advertise `PF_R8G8B8A8`/stride 4 when
   `heif_image_handle_has_alpha_channel()` is true. Every image from a real iPhone 15 Pro Max
   capture reports alpha, and a 32-bit source sends gray loads into `CImage::FilterFormat`'s
   `case PF_A8: case PF_GRAY8:` branch for 32-bit input, which **copies the alpha byte**
   instead of computing a luminance of RGB ("just copy the alpha channel"). Alpha is constant
   on real photos, so feature extraction received a flat image: **0 features on all 94 images,
   0/94 registered, and not one error logged anywhere**. Fixed in two places:
   `ReadHeader` still reports the true native format (so a caller *can* ask for alpha), but
   `ReadData` selects the libheif chroma from the **requested** `dataFormat` — RGBA only when
   the request can hold it, plain RGB otherwise — so a 3-channel or gray request never decodes
   or touches the alpha plane, keeping HEIF on the same well-tested 24-bit path as `CImageJPG`.
   And `FilterFormat` itself was fixed, so the bug class is gone for PNG/TIFF too rather than
   merely avoided here: `PF_GRAY8` from a 32-bit source now computes a luminance instead of
   falling into the alpha copy (which stays, correctly, for a `PF_A8` destination). An earlier
   note here claimed `pSrc += 3` does not land on the alpha byte — it does; for the `*A8` forms
   the RGB triple is name-reversed but alpha stays last in memory. The defect was conflating
   `PF_A8` with `PF_GRAY8`, not an offset error.
   Fixing `FilterFormat` also exposed a second, independent bug in the same switch: 24-bit→gray
   shared one code path for `PF_B8G8R8` and `PF_R8G8B8`, so the **red and blue luminance weights
   were swapped for every codec-native source**. The two cases are now separate; the gray-load
   stddev of the alpha fixture moved from 64.26 to 59.57, matching what `generate_fixtures.py`
   independently predicts from the correct weights.
5. **Two step-7 items did not apply as written.** `docs/wiki/Usage.md` has no supported-
   input-format list to extend (only a Viewer screenshot-format line), so the format note
   lives in `libs/IO/README.md` alone. And the EXIF fixtures carry *synthetic* GPS: the source
   `apps/Tests/data/images/*.jpg` have no GPS tags at all, so "graft GPS from the JPG" was
   impossible; the focal tag is genuinely grafted, and the test asserts GPS presence only.

Also note `heif_init()` is deliberately not called: the built-in libde265 decoder is
available without it, and it is only needed for loading external plugins.

Validation:

- Unit tests, plus an end-to-end SceauxCastle-converted-to-HEIC A/B against its JPG baseline:
  11/11 images registered (baseline 11/11), 21010 sparse points (baseline 20966),
  566191 dense points (baseline 552729).
- A real 94-image iPhone 15 Pro Max capture
  (`Datasets/polycam/e5d27033-1345-4d0a-b0ad-a61aa24f8283`, native `.heif`):
  94/94 images registered, 173202 sparse points, 6232211 dense points. All 94 files carry a
  container `irot` **and** EXIF `Orientation = Rotate 90 CW`, so this is the double-rotation
  case from the risk table occurring in real data on every single frame — the guard is
  confirmed against ground truth, not just against the synthetic fixture. Those files also
  all carry an alpha channel, which is what exposed deviation 4 above.

Observed on that capture but **not** caused by HEIF, and since resolved separately: the
**known-poses** path (`--import-poses-file frames.json --import-poses-mode 2 --match-mode 3`)
degraded badly — poses imported cleanly (94/94, arkit, 0 unmatched) but convention detection
reported a median relative rotation error of 68.05 deg as arkit vs 66.93 as opencv, declared it
ambiguous, and the reconstruction kept only 4/94 images. A JPG control set built from the same
libheif-decoded pixels (Orientation forced to 1, so `cv::imread` is used instead of
`CImageHEIF`, with identical dimensions, focal and rotated flag) reproduced it at 68.53/67.36
deg and 6/94 images, ruling the reader out. The cause was that ARKit reports camera transforms in
the **sensor-native landscape orientation**, which sits 180 deg from the landscape reached by
rotating a display-oriented portrait raster 90 deg clockwise — so the correct in-plane rotation is
`Rz(180)`, one quarter turn more than `ImportFramesJSON` composes. Getting it wrong conjugates
every rotation by a multiple of `Rz(90)`, which preserves each rotation's angle but tilts its axis,
and which no camera-axes flip can repair (hence both hypotheses landing at ~68 deg). Measured
median relative-rotation error over 2255 verified pairs: 0.55 deg at `Rz(180)`+arkit versus
68.31/67.04 at the assumed `Rz(90)`. The in-plane rotation is now a hypothesis searched alongside
the camera axes; see `FRAMES_IN_PLANE_TURNS` in `libs/SFM/PoseIO.h`.
