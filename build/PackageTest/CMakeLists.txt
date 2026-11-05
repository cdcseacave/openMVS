# A project using an installed OpenMVS through its CMake package, as described in
# docs/wiki/Building.md; built by the CI against the install tree:
#  cmake --install <build> --prefix <install>
#  cmake -S build/PackageTest -B <test> -DCMAKE_PREFIX_PATH=<install> [the same toolchain]
#  cmake --build <test>
cmake_minimum_required(VERSION 3.24)
project(OpenMVSPackageTest CXX)

find_package(OpenMVS CONFIG REQUIRED)
message(STATUS "OpenMVS ${OpenMVS_VERSION} found (include: ${OpenMVS_INCLUDE_DIRS})")

add_executable(PackageTest main.cpp)
target_link_libraries(PackageTest PRIVATE OpenMVS::MVS OpenMVS::SFM)
