------------------
Build instructions
------------------

Required tools:
* Cmake 
* Git
* c/c++ compiler

-----------------
Linux compilation
-----------------

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

#CVGLib (Required)
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

# OpenMVG checkout and build
main_path=`pwd`
mkdir openMVS_Build
cd openMVS_Build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=RELEASE -DVCG_DIR="$main_path/vcglib" -DCERES_DIR="/usr/local/share/Ceres" -DOpenCV_CAN_BREAK_BINARY_COMPATIBILITY=OFF

#If you want use OpenMVG as optional third party add:
-DOpenMVG_DIR:STRING="$main_path/openMVG_Build/openMVG_install/share/openMVG/cmake/"

```

