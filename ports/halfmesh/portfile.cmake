if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

# 0.3.0 carries the mesh-repair, rect-packing and selected-fill work that this
# port used to apply as patches, so no patch is needed any more.
# REF is a develop commit and not a tag on purpose: the per-vertex decimation error bound
# (Simplify(..., vertexMaxError)) and the caller-supplied remesh sizing field
# (RemeshParams::vertexSizing) -- what --simplify-tolerance and --adaptive-face-size are built
# on -- are merged upstream but not released, so halfmesh still declares 0.3.0 and its
# CMakeLists fails the configure if the version below disagrees with it. Point REF at the tag
# once a release carrying them is cut; bump port-version whenever REF moves.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/halfmesh
    REF 90778de1d9b18dccbd39a7df684efb75204265af
    SHA512 4a17d4e0be93236fbdab4c7971a70b9c829de37e3622f11d36618cd3b2b022ca827d00dc6988eca904bfed0f822d547cb55b7f4b43e53c4721985cd19aa75294
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
