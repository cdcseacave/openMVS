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
    # 0.4.0 head; becomes REF v${VERSION} again once the tag is re-cut on the merge of halfmesh PR #6
    REF 5cc53cd4996d25f2e77373df8b2470a497d1e58d
    SHA512 b5507e5d93f40af055aa642da2b65b5df24c3c65705514af24a074b1d64a6c5316e4c4068b5e8e824111c795a79cb03956a1dc2d7291bc4716f4341e804c2030
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
