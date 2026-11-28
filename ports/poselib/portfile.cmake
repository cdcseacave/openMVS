vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/PoseLib
    REF 2845f80b908360b4ebb9dddba94be80e5499d374
    SHA512 697666404d21dd8c92dc22beed0eff12e70382cc67d37c2ab61a924074d3ed5b83e5e86bb7038ea1c1ed60ab24d6c5bfe8a00ea87b8088f3c815e3f099caa7fe
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
        # a package build must not fail on new compiler warnings (GCC 15 reports a false
        # -Warray-bounds in Eigen's vectorized QR inside p2p2pl.cc)
        -DWERROR=OFF
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
