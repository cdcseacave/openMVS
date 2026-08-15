# IO Library

File format I/O subsystem for OpenMVS. Handles reading and writing of 3D geometry formats (PLY, OBJ, glTF) and image formats (BMP, TGA, DDS, PNG, JPEG, TIFF, JpegXL). Used by MVS library for point cloud, mesh, and texture serialization.

## Geometry Formats

### PLY (`PLY.h`, `PLY.cpp`)
Full-featured PLY polygon format parser/writer. Primary format for point clouds and meshes.

**Key methods:**
- `read(path)` / `write(path, numElems, elemNames, fileType, numComments)` - File I/O
- `element_count()`, `describe_element()`, `describe_property()` - Header setup
- `get_element(void*)` / `put_element(void*)` - Read/write individual elements
- `find_element()`, `find_property()` - Query schema
- `append_comment()`, `get_comments()` - Metadata

**File types:** `PLY::ASCII`, `PLY::BINARY_BE`, `PLY::BINARY_LE` (little-endian binary preferred)

**Data types:** Int8/16/32, Uint8/16/32, Float32/64. Properties can be scalar or list (variable-length arrays).

**Property combine rules** (for mesh operations): `AVERAGE_RULE`, `MAJORITY_RULE`, `MINIMUM_RULE`, `MAXIMUM_RULE`, `SAME_RULE`, `RANDOM_RULE`.

### OBJ (`OBJ.h`, `OBJ.cpp`)
Wavefront OBJ with material library support.

**Key types:**
- `ObjModel` - Main container with vertices, texcoords, normals, groups
- `ObjModel::MaterialLib` - Material definitions with diffuse textures
- `ObjModel::Face` - Triangle with vertex/texcoord/normal indices
- `ObjModel::Group` - Faces grouped by material

**Methods:** `Load(fileName)`, `Save(fileName, precision, texLossless)`, `AddGroup()`, `GetMaterial()`

### glTF (`tiny_gltf.h`)
Header-only third-party library for binary (.glb) and ASCII (.gltf) 3D format.

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

**Mesh I/O:** `LoadPLY()`, `LoadOBJ()`, `LoadGLTF()`, `SavePLY(fileName, comments, bBinary, bTexLossless)`, `SaveOBJ()`, `SaveGLTF()`

**Texture storage:** Embedded in PLY comments as `TextureFile <filename>`, saved separately as PNG/JPEG.

## Third-Party Components
- `tiny_gltf.h` - glTF loader (header-only)
- `json.hpp` - nlohmann JSON (header-only)
- `TinyXML2.h/cpp` - XML parser

## Build & Dependencies
- **Optional deps**: libpng, libjpeg, libtiff, libjxl, exiv2 (EXIF metadata)
- **Links**: Common library
- **Precompiled header**: `Common.h`
