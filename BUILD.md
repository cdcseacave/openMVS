------------
Dependencies
------------

*OpenMVS* relies on a number of open source libraries, some of which are optional. For details on customizing the build process, see the compilation instructions.
* [Eigen](http://eigen.tuxfamily.org) version 3.2 or higher
* [OpenCV](http://opencv.org) version 2.4 or higher
* [Ceres](http://ceres-solver.org) version 1.10 or higher
* [CGAL](http://www.cgal.org) version 4.2 or higher
* [Boost](http://www.boost.org) version 1.56 or higher
* [VCG](http://vcg.isti.cnr.it/vcglib)
* [OpenMVG](https://github.com/openMVG/openMVG) version 0.8.1 or higher

------------------
Build instructions
------------------

Required tools:
* [CMake](http://www.cmake.org)
* [git](https://git-scm.com)
* C/C++ compiler like Visual Studio or GCC

-------------------
Windows compilation
-------------------

Visual Studion 2008 or newer is supported. Please not that the development is done mainly on Windows, so this platform build is tested the most.

```
# Make a toplevel directory for deps & build & src somewhere:
mkdir OpenMVS
cd OpenMVS

# Get dependencies, unpack and build them as subdirectories:
like in OpenMVS/Eigen, OpenMVS/Ceres, etc

# Get and unpack OpenMVS in OpenMVS/src:
git clone https://github.com/cdcseacave/openMVS.git src

# Make build directory:
mkdir build
cd build

# Run CMake:
cmake . ../src -DCMAKE_BUILD_TYPE=RELEASE -DEIGEN_DIR="../OpenMVS/Eigen" -DOPENCV_DIR="../OpenMVS/OpenCV" -DCERES_DIR="../OpenMVS/Ceres" -DCGAL_DIR="../OpenMVS/CGAL" -DVCG_DIR="../OpenMVS/VCG"

# Open the solution and build it in MSVC
```

-----------------
Linux compilation
-----------------

[Ubuntu](http://www.ubuntu.com) is used next as the example linux distribution.

```
#Prepare and empty machine for building:
sudo apt-get update -qq && sudo apt-get install -qq
sudo apt-get -y install git subversion cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev
main_path=`pwd`

#Eigen (Required)
sudo apt-get -y install libeigen3-dev

#Boost (Required)
sudo apt-get -y install libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

#OpenCV (Required)
sudo apt-get -y install libopencv-dev

#CGAL (Required)
sudo apt-get -y install libcgal-dev

#VCGLib (Required)
svn checkout svn://svn.code.sf.net/p/vcg/code/trunk/vcglib vcglib

#Ceres (Required)
sudo apt-get install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
mkdir ceres_build && cd ceres_build/
cmake . ../ceres-solver/ -DMINIGLOG=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make
sudo make install
cd ..

#OpenMVG (Optional)
sudo apt-get install libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev graphviz
git clone --recursive https://github.com/openMVG/openMVG.git openMVG
mkdir openMVG_build && cd openMVG_build
cmake -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/ -DCMAKE_INSTALL_PREFIX=$main_path/openMVG_build/openMVG_install
make

#OpenMVS
git clone https://github.com/cdcseacave/openMVS.git openMVS
mkdir openMVS_build && cd openMVS_build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_DIR="$main_path/vcglib" -DOpenCV_CAN_BREAK_BINARY_COMPATIBILITY=OFF

#If you want to use OpenMVS as shared library, add to the cmake command:
-DBUILD_SHARED_LIBS=ON

#If you want to use OpenMVG as optional third party library, add to the cmake command:
-DOpenMVG_DIR:STRING="$main_path/openMVG_build/openMVG_install/share/openMVG/cmake/"

#Install OpenMVS library (optional):
sudo make install
```

Default compilation settings of OpenMVS reveals some tiny compilation error in VCG. Here the fixes:

```
Index: vcg/complex/algorithms/clean.h
===================================================================
--- vcg/complex/algorithms/clean.h	(revision 5553)
+++ vcg/complex/algorithms/clean.h	(working copy)
@@ -452,7 +452,7 @@
     return count_removed;
   }
 
-  static int SplitSelectedVertexOnEdgeMesh(MeshType& m)
+  static void SplitSelectedVertexOnEdgeMesh(MeshType& m)
   {
     tri::RequireCompactness(m);
     tri::UpdateFlags<MeshType>::VertexClearV(m);
Index: vcg/complex/algorithms/hole.h
===================================================================
--- vcg/complex/algorithms/hole.h	(revision 5553)
+++ vcg/complex/algorithms/hole.h	(working copy)
@@ -98,6 +98,7 @@
     ComputeQuality();
     ComputeAngle();
   }
+  virtual ~TrivialEar() {}
 
   /// Compute the angle of the two edges of the ear.
   // it tries to make the computation in a precision safe way.
```

--------------------
Mac OS X compilation
--------------------

Not tested, any help testing on this platform is welcome.

```
# Install dependencies using [MacPorts](http://www.macports.org):
sudo port install opencv boost cgal ceres-solver eigen3

# Getting the OpenMVS sources:
git clone https://github.com/cdcseacave/openMVS.git

# Build
mkdir bin
cd bin
cmake . <OpenMVS_path> -DCMAKE_BUILD_TYPE=RELEASE -DVCG_DIR="<vcglib_path>" -DCGAL_DIR="/opt/local/share/CGAL" -DCERES_DIR="/opt/local/share/Ceres"
make -j4
```
