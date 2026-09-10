if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

# 0.3.0 carries the mesh-repair, rect-packing and selected-fill work that this
# port used to apply as patches, so no patch is needed any more. 0.4.0 releases the
# per-vertex decimation error bound (Simplify(..., vertexMaxError)) and the caller-supplied
# remesh sizing field (RemeshParams::vertexSizing) -- what --simplify-tolerance and
# --adaptive-face-size are built on -- which this port used to reach by pinning a develop
# commit. halfmesh's CMakeLists fails the configure if the version in vcpkg.json disagrees
# with the one it declares, so the two move together.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/halfmesh
    REF v${VERSION}
    SHA512 cbbaaa30a03b0de94c64a6744e95a1d5d0ececcf18db0b54479ef11eb8d0b678f0db052acd2ef7d742110a1ca0c19c69e62f098aa90862f118fd9439d0103966
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
