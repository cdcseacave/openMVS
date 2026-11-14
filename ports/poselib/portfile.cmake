vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/PoseLib
    REF f81608046322944fae8a11d453f71fb552bbc4ce
    SHA512 9CF67ECBF4F132B6E50A5F125B01AA85879224F3BF484502196238BBAD16A70A49EEA11ABFA09BF57E1E2DCC048E688F723E2445691270160D1A162AA04D40AC
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
