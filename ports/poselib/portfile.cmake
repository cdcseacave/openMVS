vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/PoseLib
    REF 368cae51777bfe669a47412bb29f8830e6dc2a2e
    SHA512 7dd6961f0a288e8b337556116fd322bec07a4efefc6b735ad92a274cd109eddd811ff70e9a9ea28db1fe5df32eb893015d3daa10cd3453a103a6205ff1dcaab5
    HEAD_REF feature/generalized-absolute-pose-scale
)

# PoseLib headers do not export symbols (no __declspec(dllexport)),
# so a Windows DLL build produces no import library. Force static linkage on Windows only.
if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DMARCH_NATIVE=OFF
        -DWITH_BENCHMARK=OFF
        -DBUILD_TESTS=OFF
        -DPYTHON_PACKAGE=OFF
)
vcpkg_cmake_install()
vcpkg_copy_pdbs()

vcpkg_cmake_config_fixup(PACKAGE_NAME PoseLib CONFIG_PATH lib/cmake/PoseLib)

file(INSTALL "${SOURCE_PATH}/LICENSE"
     DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}"
     RENAME copyright)

# Remove duplicate headers from debug directory
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
