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

Visual Studion 2008 or newer is supported. Please note that the development is done mainly on Windows, so this platform build is tested the most. The latest pre-built binaries for fast testing can be download from [here](https://github.com/cdcseacave/openMVS_sample/releases/latest).

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

# Open the solution in MSVC and build it
```

-----------------
Linux compilation
-----------------

[Ubuntu](http://www.ubuntu.com) 16.04 is used next as the example linux distribution.

```
#Prepare and empty machine for building:
sudo apt-get update -qq && sudo apt-get install -qq
sudo apt-get -y install git mercurial cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev
main_path=`pwd`

#Eigen (Required)
hg clone https://bitbucket.org/eigen/eigen#3.2
mkdir eigen_build && cd eigen_build
cmake . ../eigen
make && sudo make install
cd ..

#Boost (Required)
sudo apt-get -y install libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

#OpenCV (Required)
sudo apt-get -y install libopencv-dev

#CGAL (Required)
sudo apt-get -y install libcgal-dev libcgal-qt5-dev

#VCGLib (Required)
git clone https://github.com/cdcseacave/VCG.git vcglib

#Ceres (Required)
sudo apt-get -y install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
mkdir ceres_build && cd ceres_build
cmake . ../ceres-solver/ -DMINIGLOG=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j2 && sudo make install
cd ..

#GLFW3 (Optional)
sudo apt-get -y install freeglut3-dev libglew-dev libglfw3-dev

#OpenMVS
git clone https://github.com/cdcseacave/openMVS.git openMVS
mkdir openMVS_build && cd openMVS_build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_DIR="$main_path/vcglib"

#If you want to use OpenMVS as shared library, add to the CMake command:
-DBUILD_SHARED_LIBS=ON

#Install OpenMVS library (optional):
make -j2 && sudo make install
```

--------------------
Mac OS X compilation
--------------------

Install dependencies, run CMake and make.

```
#Install dependencies
brew update
brew tap homebrew/science
brew install boost eigen opencv cgal ceres-solver
main_path=`pwd`

#VCGLib (Required)
git clone https://github.com/cdcseacave/VCG.git vcglib

#Getting the OpenMVS sources:
git clone https://github.com/cdcseacave/openMVS.git

#Build OpenMVS
mkdir openMVS_build && cd openMVS_build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_DIR="$main_path/vcglib"

#If you want to use OpenMVS as shared library, add to the CMake command:
-DBUILD_SHARED_LIBS=ON

#Install OpenMVS library (optional):
make && sudo make install
```
