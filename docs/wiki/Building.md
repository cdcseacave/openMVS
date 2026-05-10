------------
Dependencies
------------

*OpenMVS* relies on a number of open source libraries, some optional, which are managed automatically by [vcpkg](https://github.com/Microsoft/vcpkg). For details on customizing the build process, see the build instructions.
* [Eigen](http://eigen.tuxfamily.org) version 3.4 or higher
* [OpenCV](http://opencv.org) version 2.4 or higher
* [Ceres](http://ceres-solver.org) version 1.10 or higher (required for the native Structure-from-Motion module; optional otherwise)
* [CGAL](http://www.cgal.org) version 4.2 or higher
* [Boost](http://www.boost.org) version 1.56 or higher
* [CUDA](https://developer.nvidia.com/cuda-downloads) (optional)
* [GLFW](http://www.glfw.org) (optional)

------------------
Build instructions
------------------

Required tools:
* [CMake](http://www.cmake.org)
* [git](https://git-scm.com)
* C/C++ compiler like Visual Studio 2019, GCC or Clang

The dependencies can be fetched and built automatically using `vcpkg` on all major platform, by setting the environment variable `VCPKG_ROOT` to point to its path or by using the `cmake` parameter `-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake`.

Pre-built stable binaries for every supported platform (Windows x64, Windows x64 with CUDA, Ubuntu x64 and macOS arm64) are published with each tagged release on the [OpenMVS releases page](https://github.com/cdcseacave/openMVS/releases/latest).

```
#Install necesary system packages, for ex. on Debian OS:
#  The libav*-dev / libsw*-dev packages provide system FFmpeg, which the bundled
#  ports/opencv4 overlay links against (via pkg-config) so OpenCV's videoio can
#  decode video files. Windows and macOS need no extra packages — OpenCV's
#  videoio uses Media Foundation / DirectShow (Windows) or AVFoundation (macOS).
sudo apt install git cmake autoconf autoconf-archive automake libtool bison gfortran pkg-config libxi-dev libx11-dev libxft-dev libxtst-dev libxext-dev libxrandr-dev libxinerama-dev libxcursor-dev xorg-dev libgl-dev libglu1-mesa-dev nasm libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libswresample-dev

#Clone OpenMVS
git clone --recurse-submodules https://github.com/cdcseacave/openMVS.git

#Make build directory:
cd openMVS
mkdir make
cd make

#Run CMake:
cmake ..

#Build:
cmake --build . -j4

#Install OpenMVS library (optional):
cmake --install .
```

-------------------
Library usage
-------------------

In order to use *OpenMVS* as a third-party library in your project, first compile it as described above or simply use `vcpgk`:
```
vcpkg install openmvs
```

Inside your project CMake script, use:
```
find_package(OpenMVS)
if(OpenMVS_FOUND)
	include_directories(${OpenMVS_INCLUDE_DIRS})
	add_definitions(${OpenMVS_DEFINITIONS})
endif()

add_executable(your_project source_code.cpp)
target_link_libraries(your_project PRIVATE OpenMVS::MVS)
```

-------------------
Python API
-------------------

The Python API can be enable by setting the `OpenMVS_USE_PYTHON` option to `ON` when running `cmake`. The bindings are exposed by the `pyOpenMVS` extension module itself, and the install step also places an `openmvs` package wrapper next to it. Use `import openmvs as omvs` if you are importing the installed package wrapper; on Windows the wrapper adds the CUDA and co-located DLL search paths before re-exporting the same API.

Use `import pyOpenMVS as omvs` if you are importing the extension module directly from the build or install location.

Example:
```
import openmvs as omvs

def run_mvs():
    # set the working folder; all files used next are relative to this folder (optional)
    omvs.set_working_folder("folder/containing/the/scene")
    # create an empty scene
    scene = omvs.Scene()
    # load a MVS scene from a file
    if not scene.load("scene.mvs"):
        print("ERROR: scene could not be loaded")
        return
    # estimate depth-maps and fuse them into a point-cloud
    if not scene.dense_reconstruction():
        print("ERROR: could not dense reconstruct the scene")
        return
    scene.save_pointcloud("pointcloud.ply")
    # reconstruct a mesh from the point-cloud
    if not scene.reconstruct_mesh():
        print("ERROR: could not reconstruct the mesh for this scene")
        return
    scene.save_mesh("mesh.ply")
    # refine the mesh using gradient descent optimization (optional)
    if not scene.refine_mesh():
        print("ERROR: could not refine the mesh for this scene")
        return
    scene.save_mesh("refined_mesh.ply")
    # texture the mesh using the input images
    if not scene.texture_mesh():
        print("ERROR: could not texture the mesh for this scene")
        return
    scene.save_mesh("textured_mesh.ply")

if __name__ == "__main__":
    run_mvs()
```
