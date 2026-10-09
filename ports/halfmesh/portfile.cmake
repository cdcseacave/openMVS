if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

# halfmesh's CMakeLists fails the configure if the version in vcpkg.json disagrees
# with the one it declares, so the two move together.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/halfmesh
    REF v${VERSION}
    SHA512 0
    HEAD_REF develop
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DHALFMESH_BUILD_TESTS=OFF
        -DHALFMESH_BUILD_TOOLS=OFF
        -DHALFMESH_BUILD_PYTHON=OFF
        -DHALFMESH_BUILD_PERF=OFF
        -DHALFMESH_BUILD_CROSSCHECKS=OFF
        -DHALFMESH_BUILD_BENCH=OFF
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH "lib/cmake/halfmesh")

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
