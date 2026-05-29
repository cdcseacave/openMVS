vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/PoseLib
    REF ccdd2f62d7ee91b41a1dce4dfd619b688b6c247a
    SHA512 3A14AA97D04D9700E77BA908EBE607477BE3210A981B4A73917E63B33801C0905A2629F5B715BBC8B97DCB2D68C92D23EAB04B0014FC0FAF9A15413D92CDA1C4
    HEAD_REF feature/spherical-camera-support
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
