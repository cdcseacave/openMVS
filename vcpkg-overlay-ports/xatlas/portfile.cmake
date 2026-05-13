# xatlas overlay port (jpcy/xatlas)
# Upstream isn't a CMake project; we create a tiny CMake wrapper.

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO jpcy/xatlas
    REF f700c7790aaa030e794b52ba7791a05c085faf0c
    HEAD_REF master
    SHA512 b6a285f7c5872c0549f19f56bbf3401e67eacae4d6e99ea110336a3760015f02d91371530bab6b8dc4bdbe5db321844495f9ff650e2e92701ad48591578b448a
)

set(XATLAS_UPSTREAM_DIR "${SOURCE_PATH}/source/xatlas")

if(NOT EXISTS "${XATLAS_UPSTREAM_DIR}/xatlas.cpp" OR NOT EXISTS "${XATLAS_UPSTREAM_DIR}/xatlas.h")
    message(FATAL_ERROR "Expected xatlas.cpp/xatlas.h in ${XATLAS_UPSTREAM_DIR}. Upstream layout may have changed.")
endif()

set(PROJ_DIR "${CURRENT_BUILDTREES_DIR}/src/${PORT}")
file(REMOVE_RECURSE "${PROJ_DIR}")
file(MAKE_DIRECTORY "${PROJ_DIR}")

file(COPY
    "${XATLAS_UPSTREAM_DIR}/xatlas.cpp"
    "${XATLAS_UPSTREAM_DIR}/xatlas.h"
    DESTINATION "${PROJ_DIR}"
)

# Important: escape ${...} so they are evaluated by the wrapper project, not now.
file(WRITE "${PROJ_DIR}/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.15)\n"
"project(xatlas LANGUAGES CXX)\n"
"\n"
"add_library(xatlas STATIC xatlas.cpp)\n"
"add_library(xatlas::xatlas ALIAS xatlas)\n"
"\n"
"target_compile_features(xatlas PUBLIC cxx_std_11)\n"
"target_include_directories(xatlas PUBLIC\n"
"  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>\n"
"  $<INSTALL_INTERFACE:include>\n"
")\n"
"\n"
"include(GNUInstallDirs)\n"
"\n"
"install(TARGETS xatlas\n"
"  EXPORT xatlasTargets\n"
"  ARCHIVE DESTINATION \${CMAKE_INSTALL_LIBDIR}\n"
"  LIBRARY DESTINATION \${CMAKE_INSTALL_LIBDIR}\n"
"  RUNTIME DESTINATION \${CMAKE_INSTALL_BINDIR}\n"
")\n"
"\n"
"install(FILES \"\${CMAKE_CURRENT_LIST_DIR}/xatlas.h\"\n"
"  DESTINATION \${CMAKE_INSTALL_INCLUDEDIR}\n"
")\n"
"\n"
"install(EXPORT xatlasTargets\n"
"  FILE xatlasTargets.cmake\n"
"  NAMESPACE xatlas::\n"
"  DESTINATION share/xatlas\n"
")\n"
"\n"
"# Minimal config so find_package(xatlas CONFIG) works\n"
"file(WRITE \"\${CMAKE_CURRENT_BINARY_DIR}/xatlasConfig.cmake\"\n"
"  \"include(\\\"\\\${CMAKE_CURRENT_LIST_DIR}/xatlasTargets.cmake\\\")\\n\"\n"
")\n"
"\n"
"install(FILES \"\${CMAKE_CURRENT_BINARY_DIR}/xatlasConfig.cmake\"\n"
"  DESTINATION share/xatlas\n"
")\n"
)

vcpkg_cmake_configure(
    SOURCE_PATH "${PROJ_DIR}"
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(PACKAGE_NAME xatlas CONFIG_PATH share/xatlas)

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
