------------
Dependencies
------------

OpenMVS relies on a number of open source libraries, some of which are optional. For details on customizing the build process, see the compilation instructions.
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
# Getting the OpenMVG sources:
git clone https://github.com/cdcseacave/openMVS.git

##
#Setup the required external library:
##

#Ceres 3rd parties
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libsuitesparse-dev

#Ceres (Required)
git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir ceres_Build
cd ceres_Build/
cmake . ../ceres-solver/
make
sudo make install
cd ..

#CGAL (Required)
sudo apt-get install libcgal-dev

#OpenCV (Required)
sudo apt-get install libopencv-dev

#VCGLib (Required)
sudo apt-get install subversion
svn checkout svn://svn.code.sf.net/p/vcg/code/trunk/vcglib vcglib

#MESA (Required)
sudo apt-get install mesa-dev

#Boost (Required)
sudo apt-get install libboost-iostreams-dev libboost-program_options-dev libboost-system-dev libboost-serialization-dev

#OpenMVG (Optional)
sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev graphviz
git clone --recursive https://github.com/openMVG/openMVG.git
mkdir openMVG_Build
cd openMVG_Build
current_path=`pwd`
cmake -DCMAKE_BUILD_TYPE=RELEASE . ../openMVG/src/ -DCMAKE_INSTALL_PREFIX=$current_path/openMVG_install
make

#OpenMVS build
main_path=`pwd`
mkdir openMVS_Build
cd openMVS_Build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=RELEASE -DVCG_DIR="$main_path/vcglib" -DCERES_DIR="/usr/local/share/Ceres" -DOpenCV_CAN_BREAK_BINARY_COMPATIBILITY=OFF

#If you want use OpenMVG as optional third party add to the cmake command:
-DOpenMVG_DIR:STRING="$main_path/openMVG_Build/openMVG_install/share/openMVG/cmake/"
```

--------------------
Mac OS X compilation
--------------------

Not tested, any help testing on this platform is welcome.
