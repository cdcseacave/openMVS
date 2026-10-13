# IO Library

The IO library handles all file format reading and writing in OpenMVS -- both 3D geometry (point clouds, meshes) and 2D images. If data enters or leaves the system through a file, it goes through this library.

## What You Need to Know First

### The library is a format abstraction layer

You rarely interact with IO directly. Instead, the MVS library's `PointCloud`, `Mesh`, and `Scene` classes call into IO when you do `LoadPLY()`, `SavePLY()`, etc. (OBJ and glTF meshes go through the halfmesh library instead, see `MVS/MeshHalfMesh.cpp`) Understanding IO matters when you need to add a new format, debug file loading issues, or extend what's stored in an existing format.

### PLY is the primary 3D format

While multiple formats are supported, PLY (Polygon File Format) is the workhorse. It's used for:
- Sparse and dense point clouds (with optional colors, normals, view indices)
- Triangle meshes (with optional textures)
- Intermediate pipeline outputs

PLY files are saved in **little-endian binary** (`PLY::BINARY_LE`) by default; ASCII is available for debugging and writes every real exactly (the shortest text that reads back to the same value); an ASCII record is one line, and a line holding fewer or more values than its record is refused. Big-endian files are read, not written.

## 3D Geometry Formats

### PLY (`PLY.h`, `PLY.cpp`)

**Elements and Properties**: A PLY file is organized into elements (like "vertex" or "face"), each a table of records whose columns are typed properties — scalars, or lists of scalars preceded by their count:
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

**Bindings**: values are read straight into, and written straight from, the caller's arrays. A `PLY::Binding` names a property and gives the memory of its value in the first record plus the stride to the next record (struct-of-arrays and array-of-structs layouts both work):
- `PLY::Scalar(name, &arr[0].x, stride)` — a scalar; on write an optional file type (e.g. `PLY::Int32` for a `uint8_t` in memory)
- `PLY::List(name, &faces[0].x, 3, stride)` — a list of a fixed number of items (reading a record of any other size fails: a quad is refused, not misread)
- `PLY::Lists(name, lists[, countType])` — a list of any size per record, kept in a container of containers (`lists[i].resize/data/size`); when writing, the count type (`Uint8` by default) must hold the longest list: `PLY::CountTypeOf(lists)` gives the smallest that does

**Reading pattern** (elements are read in file order; the ones not read, and the unbound properties, are skipped; values are converted from the file's type to the bound type):
```cpp
PLY ply;
if (!ply.Open(fileName))
    return false;
for (size_t e = 0; e < ply.GetElements().size(); ++e) {
    const PLY::Element& elem = ply.GetElements()[e];
    if (elem.name != "vertex")
        continue;
    points.resize(elem.count);  // safe: Open() refuses counts the file is too short to hold
    PLY::Bindings bindings{PLY::Scalar("x", &points[0].x, sizeof(Point)), /* y, z */};
    if (elem.HasProperties({"nx", "ny", "nz"}))
        /* resize normals, add their bindings */;
    if (!ply.ReadElement(e, bindings))
        return false;  // missing bound property, list of another size, truncated file...
}
```

**Writing pattern** (describe the elements in file order, then save; the bound memory must stay valid until `Save`):
```cpp
PLY ply;
ply.AddComment("TextureFile texture_0.png");
ply.AddElement("vertex", vertices.size(), {
    PLY::Scalar("x", &vertices[0].x, sizeof(Vertex)), /* y, z */});
ply.AddElement("face", faces.size(), {PLY::List("vertex_indices", &faces[0].x, 3, sizeof(Face))});
return ply.Save(fileName);  // PLY::BINARY_LE by default; legacy type names ("float", "uchar") on request
```

**Speed**: a record is processed as runs of fixed-size properties at fixed offsets; adjacent plain copies (x, y, z into a `Point3f`) merge into one copy; an element whose records are laid out exactly as in memory (vertex x/y/z into a vertex array) is read or written in one block, straight between the file and the array.

**Comments**: PLY comments store metadata. OpenMVS uses them for texture file references:
```
comment TextureFile texture_0.png
comment TextureFile texture_1.jpg
```

### OBJ (halfmesh)

Wavefront OBJ is not implemented here: `Mesh::LoadOBJ()` / `Mesh::SaveOBJ()` delegate to halfmesh's codec (streamed parallel parsing with exact float conversion, polygons triangulated, MTL materials; written in the shortest text that reads back to the same float). Textures are saved beside the file as PNG (lossless) or JPEG and referenced from the .mtl file.

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
mesh.Save("textured.obj");          // Creates .obj + .mtl + textures (halfmesh)
mesh.SaveGLTF("model.glb", true);   // Binary glTF
```

**Scene** (in MVS lib): Uses `.mvs` native format (Boost serialization, not part of IO) but IO handles all import/export to standard formats.

## Third-Party Code

- **`json.hpp`**: nlohmann JSON (header-only, MIT license)

## File Organization

```
libs/IO/
├── Common.h/cpp        # Library entry point, conditional includes
├── PLY.h/cpp           # PLY reader/writer bound to the caller's arrays
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
└── CMakeLists.txt      # Build config with optional dependency detection
```

## Dependencies

- **Common** (required): Base types and utilities
- **libpng** (optional): PNG support
- **libjpeg** (optional): JPEG support
- **libtiff** (optional): TIFF support
- **libjxl** (optional): JPEG XL support
- **exiv2** (optional): EXIF metadata extraction
