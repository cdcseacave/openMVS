vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/PoseLib
    REF 26d397d5ad487959b0c4243d61ef6f75715eb040
    SHA512 85345F76C7816665F6FD5CB86E6B35823D209CB1E7A6EE54521143CC05E1700FA3D54E808CC2D17A9CBC08D2B277313D3B977B1B8723AEDF7D9782FE8CFF12CD
    HEAD_REF feature/spherical-camera-support
)

# PoseLib headers do not export symbols (no __declspec(dllexport)),
# so a Windows DLL build produces no import library. Force static linkage on Windows only.
if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

# PoseLib builds with -Wall -Werror; GCC 15 reports a false -Warray-bounds inside the AVX loads of
# Eigen's vectorized QR on fixed 5x5 matrices (p2p2pl.cc): keep that one a warning
if(VCPKG_TARGET_IS_LINUX)
    string(APPEND VCPKG_CXX_FLAGS " -Wno-error=array-bounds")
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
