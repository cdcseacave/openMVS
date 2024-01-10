# How to build for BMG use case

0. Tools
	- cmake
    $ "C:\Program Files\CMake\bin\cmake.exe" .. \
		-DCMAKE_TOOLCHAIN_FILE=E:/Users/yyk99/Downloads/vcpkg/vcpkg/scripts/buildsystems/vcpkg.cmake \
		-DOpenMVS_USE_CUDA=OFF

1. Required
	- boost
		-DBoost_NO_WARN_NEW_VERSIONS=1
	- OpenCV
	- Eigen3
		-DEIGEN3_INCLUDE_DIR=C:\opt\eigen3.4\include\eigen3
	- CGAL
	- GMP
		https://github.com/gx/gmp
	
	Optional:
		- BREAKPAD
		
