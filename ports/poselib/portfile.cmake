vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/PoseLib
    REF 53acfda7b889f01f86a377a27dd5b3500a7d06d5
    SHA512 A9BCB5888B4E3B214FD9D920E08439E33EDA6DC283E7F8642925FF0ED181DB49C7D0BFF232FA6D3E226F370C62E395D3CB443DBA88A9CBF182F06FBA61B2BCA4
    HEAD_REF feature/spherical-camera-support
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DMARCH_NATIVE=OFF
        -DWITH_BENCHMARK=OFF
        -DBUILD_TESTS=ON
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
