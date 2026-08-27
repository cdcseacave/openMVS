# IO Library

The IO library handles all file format reading and writing in OpenMVS -- both 3D geometry (point clouds, meshes) and 2D images. If data enters or leaves the system through a file, it goes through this library.

## What You Need to Know First

### The library is a format abstraction layer

You rarely interact with IO directly. Instead, the MVS library's `PointCloud`, `Mesh`, and `Scene` classes call into IO when you do `LoadPLY()`, `SaveOBJ()`, etc. Understanding IO matters when you need to add a new format, debug file loading issues, or extend what's stored in an existing format.

### PLY is the primary 3D format

While multiple formats are supported, PLY (Polygon File Format) is the workhorse. It's used for:
- Sparse and dense point clouds (with optional colors, normals, view indices)
- Triangle meshes (with optional textures)
- Intermediate pipeline outputs

PLY files are typically saved in **little-endian binary** (`PLY::BINARY_LE`) for performance. ASCII mode is available for debugging.

## 3D Geometry Formats

### PLY (`PLY.h`, `PLY.cpp`)

The PLY parser is a full implementation of the PLY specification. Key concepts:

**Elements and Properties**: A PLY file is organized into elements (like "vertex" or "face"), each with typed properties:
```
element vertex 1000
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
element face 500
property list uchar int vertex_indices
```

**Reading pattern**:
```cpp
PLY ply;
if (!ply.read(fileName))
    return false;

// Find and read vertex element
if (ply.find_element("vertex")) {
    ply.describe_element("vertex", vertexCount);
    // Set up property handlers...
    for (int i = 0; i < vertexCount; i++)
        ply.get_element(&vertex);
}
```

**Writing pattern**:
```cpp
PLY ply;
ply.write(fileName, numElements, elementNames, PLY::BINARY_LE, 0);
// Describe properties, then header_complete()
for (auto& v : vertices)
    ply.put_element(&v);
```

**Comments**: PLY comments store metadata. OpenMVS uses them for texture file references:
```
comment TextureFile texture_0.png
comment TextureFile texture_1.jpg
```

**Property combine rules**: When merging mesh data, properties can be combined using rules like `AVERAGE_RULE`, `MAJORITY_RULE`, `MINIMUM_RULE`, etc.

### OBJ (`OBJ.h`, `OBJ.cpp`)

Wavefront OBJ support with full material library (.mtl) handling:

- **`ObjModel`**: Container with vertices, texture coordinates, normals, and face groups
- **`MaterialLib`**: Material definitions including diffuse textures
- **Groups**: Faces are organized by material name

Textures are saved as separate image files (PNG for lossless, JPEG for lossy) and referenced from the .mtl file.

```cpp
ObjModel model;
model.Load("mesh.obj");      // Loads .obj + .mtl + texture images
model.Save("output.obj", 6); // 6 decimal places for vertex precision
```

### glTF (`tiny_gltf.h`, from vcpkg)

Modern 3D format support via the third-party header-only `tiny_gltf` library, whose single implementation unit is compiled by halfmesh. Supports both binary (`.glb`) and ASCII (`.gltf`) variants. `PointCloud::LoadGLTF()` / `SaveGLTF()` drive tinygltf directly; the mesh side delegates to halfmesh (see `MVS/MeshHalfMesh.cpp`), which puts the glTF z-up to y-up rotation on the root node and undoes it on load, making the round-trip an identity.

## Image Format System

### Factory pattern with auto-detection

The image system uses a factory pattern. You call `CImage::Create(fileName, mode)` and it detects the format from the file extension, returning the appropriate subclass:

```cpp
CImage* img = CImage::Create("photo.jpg", CImage::READ);
img->ReadHeader();
img->ReadData(buffer, PF_R8G8B8, stride, width);
delete img;
```

### Supported formats

| Format | Class | Always Available? | Notes |
|--------|-------|-------------------|-------|
| BMP | `CImageBMP` | Yes | Simple uncompressed bitmap |
| TGA | `CImageTGA` | Yes | Supports RLE compression |
| DDS | `CImageDDS` | Yes | DirectX format with DXT compression and mipmaps |
| PNG | `CImagePNG` | Optional (`_USE_PNG`) | Lossless, requires libpng |
| JPEG | `CImageJPG` | Optional (`_USE_JPG`) | Lossy, requires libjpeg |
| TIFF | `CImageTIFF` | Optional (`_USE_TIFF`) | Multi-page support, requires libtiff |
| JpegXL | `CImageJXL` | Optional (`_USE_JXL`) | Modern codec, requires libjxl |
| HEIF | `CImageHEIF` | Optional (`_USE_HEIF`) | Read-only high-efficiency image format, requires libheif |

libheif is pinned with `"default-features": false` in `vcpkg.json` on purpose: that keeps the
GPL-2.0 x265 **encoder** out of the shipped binary, leaving the LGPL libde265 decoder, which is
all OpenMVS needs since HEIF support is read-only. Do not enable the default features to "fix" a
missing encoder — there is no HEIF write path to feed it. Note that two of the bundled test
images (`apps/Tests/data/images/00001.heic`, `00002.heic`) are HEIC, so the SFM and MVS pipeline
tests require libheif; OpenCV has no HEIF codec to fall back on.
| SCI | `CImageSCI` | Yes | Custom OpenMVS binary format |

The optional formats are enabled at build time based on available system libraries. The CMake build auto-detects them and sets `_USE_PNG`, `_USE_JPG`, etc.

### Pixel formats

The library defines a rich set of pixel formats for conversion between different representations:

- **Grayscale**: `PF_GRAY8` (8-bit), `PF_GRAY32F` (32-bit float, used for depth maps)
- **Colour**: `PF_R8G8B8` / `PF_B8G8R8` (24-bit) and `PF_R8G8B8A8` / `PF_B8G8R8A8` (32-bit with alpha)
- **Compressed**: `PF_DXT1` through `PF_DXT5` (S3TC block compression)

`CImage::FilterFormat()` handles conversion between formats.

**Channel order is the reverse of the name.** The names list channels from the most- to the
least-significant bit (see the comment above `PIXELFORMAT` in `Image.h`), so on a little-endian
machine the bytes in memory come out reversed:

| Constant | Bytes in memory | Who uses it |
| --- | --- | --- |
| `PF_B8G8R8` | R, G, B | declared by every codec reader (JPG/PNG/JXL/HEIF) as its native format |
| `PF_R8G8B8` | B, G, R | OpenCV's order — what `MVS::Image::ReadImage` and the rest of OpenMVS request |

The unambiguous anchor is `CImagePNG::ReadData()`, which calls libpng's `png_set_bgr()` *precisely
when* a `PF_B8G8R8` file is read into a `PF_R8G8B8` request: libpng natively emits R,G,B, so the
request that needs the swap is `PF_R8G8B8`. `CImageJPG` agrees, declaring `PF_B8G8R8` for
libjpeg's `JCS_RGB`. Requesting `PF_R8G8B8` is therefore what gives you an OpenCV-ready buffer.

## How Other Libraries Use IO

**PointCloud** (in MVS lib):
```cpp
pointCloud.LoadPLY("sparse.ply");
pointCloud.SavePLY("dense.ply", /*bViews=*/true, /*bLegacy=*/false, /*bBinary=*/true);
```

**Mesh** (in MVS lib):
```cpp
mesh.LoadPLY("mesh.ply");
mesh.SaveOBJ("textured.obj");       // Creates .obj + .mtl + textures
mesh.SaveGLTF("model.glb", true);   // Binary glTF
```

**Scene** (in MVS lib): Uses `.mvs` native format (Boost serialization, not part of IO) but IO handles all import/export to standard formats.

## Third-Party Code

- **`json.hpp`**: nlohmann JSON (header-only, MIT license)
- **`TinyXML2.h/cpp`**: Lightweight XML parser

## File Organization

```
libs/IO/
├── Common.h/cpp        # Library entry point, conditional includes
├── PLY.h/cpp           # PLY parser/writer (~2000+ lines)
├── OBJ.h/cpp           # Wavefront OBJ with materials
├── Image.h/cpp         # CImage base class + factory
├── ImageBMP.h/cpp      # BMP format
├── ImageTGA.h/cpp      # TGA format (with RLE)
├── ImageDDS.h/cpp      # DDS format (with DXT)
├── ImagePNG.h/cpp      # PNG format (optional)
├── ImageJPG.h/cpp      # JPEG format (optional)
├── ImageTIFF.h/cpp     # TIFF format (optional)
├── ImageJXL.h/cpp      # JPEG XL format (optional)
├── ImageHEIF.h/cpp     # HEIF/HEIC format, read-only (optional)
├── ImageSCI.h/cpp      # Custom OpenMVS format
├── json.hpp            # JSON library (third-party, header-only)
├── TinyXML2.h/cpp      # XML parser (third-party)
└── CMakeLists.txt      # Build config with optional dependency detection
```

## Dependencies

- **Common** (required): Base types and utilities
- **libpng** (optional): PNG support
- **libjpeg** (optional): JPEG support
- **libtiff** (optional): TIFF support
- **libjxl** (optional): JPEG XL support
- **exiv2** (optional): EXIF metadata extraction
