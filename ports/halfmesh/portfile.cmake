if(VCPKG_TARGET_IS_WINDOWS)
    vcpkg_check_linkage(ONLY_STATIC_LIBRARY)
endif()

# 0.3.0 carries the mesh-repair, rect-packing and selected-fill work that this
# port used to apply as patches, so no patch is needed any more.
# The v0.3.0 tag was re-cut (now 37131ca: per-vertex-normal transport and
# BakeOntoAtlas), so the SHA512 below no longer matches an older download of
# the same tag -- bump port-version alongside it whenever the tag moves again.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO cdcseacave/halfmesh
    REF "v${VERSION}"
    SHA512 f62334ccf18529fff5bd1d8231fb19292ea85d6de4274295629b7009c13f6ef3ac44d3e3b7d91849c1dd86233242a7fc5940591724b97752fd7df67d264ddf6a
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
