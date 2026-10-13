# IO Library

File format I/O subsystem for OpenMVS. Handles reading and writing of 3D geometry formats (PLY, glTF) and image formats (BMP, TGA, DDS, PNG, JPEG, TIFF, JpegXL). Used by MVS library for point cloud, mesh, and texture serialization.

## Geometry Formats

### PLY (`PLY.h`, `PLY.cpp`)
PLY reader/writer, the primary format for point clouds and meshes. Values go straight between the file and the caller's arrays through `PLY::Binding`s (property name + memory of the first record's value + stride), so there is no per-record structure and no per-scalar dispatch.

**Reading:** `Open(path)` parses the header (and refuses element counts the file is too short to hold, so callers may size arrays by them); `GetElements()` / `Element::FindProperty()` / `Element::HasProperties()` describe the file; `ReadElement(idx, bindings)` reads one element, in file order, skipping unbound properties and unread elements and converting each value to the bound type. It returns false on a missing bound property, a fixed-size list of another size, or a truncated/malformed file — never throws or exits. `GetComments()` holds the header comments.

**Writing:** `AddComment()`, `AddElement(name, count, bindings)` in file order, then `Save(path, type, legacyTypeNames)`; the bound memory must stay valid until `Save`.

**Bindings:** `PLY::Scalar(name, &arr[0].x, stride[, fileType])`, `PLY::List(name, &faces[0].x, numItems, stride)` (fixed-size list), `PLY::Lists(name, containerOfContainers)` (any size per record).

**File types:** `PLY::BINARY_LE` (default), `PLY::ASCII` (exact text for reals), `PLY::BINARY_BE` (read only).

**Data types:** Int8/16/32, Uint8/16/32, Float32/64 (also read under their legacy names: char, uchar, short, int, float, double...).

### glTF (`tiny_gltf.h`, from vcpkg)
Header-only third-party library for binary (.glb) and ASCII (.gltf) 3D format. Not vendored here - it comes from the `tinygltf` port, and halfmesh compiles the single `TINYGLTF_IMPLEMENTATION` unit.

## Image Format System

### Base Class: `CImage` (`Image.h`)
Factory pattern with format auto-detection by file extension.

**Key methods:**
- `CImage::Create(fileName, mode)` - Factory: detects format, returns format-specific subclass
- `ReadHeader()` / `WriteHeader()` - Format-specific header I/O
- `ReadData()` / `WriteData()` - Pixel data I/O
- `FilterFormat()` - Pixel format conversion

**Free function:** `LoadImage(fileName, cv::Mat&, PIXELFORMAT)` - overload of Common's `LoadImage()` that decodes with OpenCV when it supports the format and with the `CImage` readers when it does not (HEIF); `PF_GRAY8` or `PF_R8G8B8` (OpenCV's BGR memory order).

### Pixel Formats
`PF_GRAY8`, `PF_GRAY32F` (depth maps), `PF_R8G8B8`, `PF_R8G8B8A8`, `PF_B8G8R8`, `PF_B8G8R8A8`, `PF_DXT1`-`PF_DXT5` (S3TC compressed).

### Format Implementations

| Format | Class | Header | Availability | Notes |
|--------|-------|--------|-------------|-------|
| BMP | `CImageBMP` | `ImageBMP.h` | Always | Uncompressed bitmap |
| TGA | `CImageTGA` | `ImageTGA.h` | Always | RLE compression support |
| DDS | `CImageDDS` | `ImageDDS.h` | Always | Mipmaps, DXT compression |
| PNG | `CImagePNG` | `ImagePNG.h` | Optional (`_USE_PNG`) | Lossless, libpng |
| JPEG | `CImageJPG` | `ImageJPG.h` | Optional (`_USE_JPG`) | Lossy, libjpeg |
| TIFF | `CImageTIFF` | `ImageTIFF.h` | Optional (`_USE_TIFF`) | Multi-page, libtiff |
| JpegXL | `CImageJXL` | `ImageJXL.h` | Optional (`_USE_JXL`) | Modern codec |
| HEIF | `CImageHEIF` | `ImageHEIF.h` | Optional (`_USE_HEIF`) | Read-only, libheif; OpenCV has no codec for it |
| SCI | `CImageSCI` | `ImageSCI.h` | Always | Custom OpenMVS format |

## Integration with MVS Library

**PointCloud I/O:** `LoadPLY()`, `SavePLY(fileName, bViews, bLegacyTypes, bBinary)`, `LoadGLTF()`, `SaveGLTF()`

**Mesh I/O:** `LoadPLY()`, `SavePLY(fileName, comments, bBinary, bTexLossless)` use this library's PLY codec. OBJ and glTF are halfmesh's: `Mesh::LoadOBJ()` / `SaveOBJ(fileName, bTexLossless)` and `Mesh::LoadGLTF()` / `SaveGLTF(fileName, bBinary, bTexLossless)` are thin wrappers in `MVS/MeshHalfMesh.cpp` (halfmesh owns the only `TINYGLTF_IMPLEMENTATION` in the build; only `PointCloud` still drives tinygltf directly).

**Texture storage:** Embedded in PLY comments as `TextureFile <filename>`, saved separately as PNG/JPEG.

## Third-Party Components
- `json.hpp` - nlohmann JSON (header-only)

## Build & Dependencies
- **Optional deps**: libpng, libjpeg, libtiff, libjxl, exiv2 (EXIF metadata)
- **Links**: Common library
- **Precompiled header**: `Common.h`
