# Find required packages
FIND_PACKAGE(CGAL REQUIRED)
if(CGAL_FOUND)
	include_directories(${CGAL_INCLUDE_DIRS})
	add_definitions(${CGAL_DEFINITIONS})
	link_directories(${CGAL_LIBRARY_DIRS})
endif()

set(CERES_LIBS "")
if(OpenMVS_USE_CERES)
	FIND_PACKAGE(Ceres)
	if(CERES_FOUND)
		include_directories(${CERES_INCLUDE_DIRS})
		add_definitions(${CERES_DEFINITIONS})
	else()
		set(OpenMVS_USE_CERES OFF)
		message("-- Can't find CERES. Continuing without it.")
	endif()
endif()

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")
if(_USE_CUDA)
	FILE(GLOB LIBRARY_FILES_CUDA "*.cu")
	LIST(APPEND LIBRARY_FILES_C ${LIBRARY_FILES_CUDA})
	FILE(GLOB CUDA_LIBRARY_FILES_C "CUDA/*.cpp")
	FILE(GLOB CUDA_LIBRARY_FILES_H "CUDA/*.h" "CUDA/*.inl")
	FILE(GLOB CUDA_LIBRARY_FILES_CUDA "CUDA/*.cu")
	LIST(APPEND CUDA_LIBRARY_FILES_C ${CUDA_LIBRARY_FILES_CUDA})
	SOURCE_GROUP("CUDA" FILES ${CUDA_LIBRARY_FILES_C} ${CUDA_LIBRARY_FILES_H})
endif()

GET_FILENAME_COMPONENT(PATH_PythonWrapper_cpp ${CMAKE_CURRENT_SOURCE_DIR}/PythonWrapper.cpp ABSOLUTE)
LIST(REMOVE_ITEM LIBRARY_FILES_C "${PATH_PythonWrapper_cpp}")

# Metal backend: the Objective-C++ source is not matched by the *.cpp glob.
# The MSL kernels (PatchMatchMetal.metal) are the single source of truth; embed
# them as a string (compiled at runtime via newLibraryWithSource) by generating
# PatchMatchMetal_msl.h into the build dir at configure time.
if(_USE_METAL)
	file(READ "${CMAKE_CURRENT_SOURCE_DIR}/PatchMatchMetal.metal" _PATCHMATCH_MSL_SRC)
	file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/PatchMatchMetal_msl.h"
"// AUTO-GENERATED at configure time from PatchMatchMetal.metal -- do not edit.
#pragma once
static const char* kPatchMatchMSL = R\"METALSRC(
${_PATCHMATCH_MSL_SRC})METALSRC\";
")
	LIST(APPEND LIBRARY_FILES_C "${CMAKE_CURRENT_SOURCE_DIR}/PatchMatchMetal.mm")
endif()

cxx_library_with_type(MVS "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
	${CUDA_LIBRARY_FILES_C} ${CUDA_LIBRARY_FILES_H}
)

# Metal backend: compile the .mm as Objective-C++ with ARC and skip the C++ PCH.
# The Metal/Foundation frameworks are linked centrally via OpenMVS_EXTRA_LIBS
# (root CMakeLists) and propagate here transitively through Common.
if(_USE_METAL)
	set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/PatchMatchMetal.mm" PROPERTIES
		SKIP_PRECOMPILE_HEADERS ON
		COMPILE_OPTIONS "-fobjc-arc")
	TARGET_INCLUDE_DIRECTORIES(MVS PRIVATE "${CMAKE_CURRENT_BINARY_DIR}") # generated PatchMatchMetal_msl.h
endif()

# MSVC scalpel: Camera.cpp and Scene.cpp drag in a template graph (CGAL/Eigen/OpenCV)
# that can land MSVC's optimizer in a pathological codegen path and take hours on
# constrained builders.
#
# Policy:
#   * Release       — full /O2 by default. OpenMVS_MSVC_FAST_RELEASE opts these two
#                     files into /Od for constrained builders such as hosted CI.
#   * RelWithDebInfo/Debug — /Od for just these two files, keeping dev builds under 30 min.
#
# Implementation notes:
#   * PCH for MVS is built with /O2, so /Od on a TU reusing that PCH triggers warning
#     C4653 and cl.exe silently snaps the TU back to /O2 — the /Od becomes a no-op.
#     To actually apply /Od we must also skip the PCH on these files.
#   * SKIP_PRECOMPILE_HEADERS is set unconditionally (no genex): in Release it means
#     Camera/Scene re-parse Common.h instead of reusing the PCH (2-3s overhead, dwarfed
#     by the 10+ min optimizer cost), and in non-Release it enables the /Od override.
if(MSVC)
	set_source_files_properties(
		"${CMAKE_CURRENT_SOURCE_DIR}/Camera.cpp"
		"${CMAKE_CURRENT_SOURCE_DIR}/Scene.cpp"
		PROPERTIES
			COMPILE_OPTIONS "$<$<OR:$<BOOL:${OpenMVS_MSVC_FAST_RELEASE}>,$<NOT:$<CONFIG:Release>>>:/Od>"
			SKIP_PRECOMPILE_HEADERS ON
	)
	if(OpenMVS_MSVC_FAST_RELEASE)
		message(STATUS "OpenMVS_MSVC_FAST_RELEASE: compiling MVS Camera.cpp and Scene.cpp with /Od")
	else()
		message(WARNING "MVS build speed notice:\n"
		        "  * Release builds compile Camera.cpp and Scene.cpp at full /O2. These two\n"
		        "    files can dominate Release wall time due to heavy CGAL/Eigen/OpenCV\n"
		        "    template instantiation.\n"
		        "  * RelWithDebInfo (and Debug) use /Od for just these two files to keep\n"
		        "    dev iteration fast; the rest of the project stays at /O2.\n"
		        "  * To speed up Release at the cost of less-optimized Camera.cpp/Scene.cpp,\n"
		        "    configure with -DOpenMVS_MSVC_FAST_RELEASE=ON.")
	endif()
endif()

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(MVS PRIVATE "Common.h")
endif()

# Link its dependencies
TARGET_LINK_LIBRARIES(MVS PUBLIC Common Math IO CGAL::CGAL ${CERES_LIBRARIES})

if(_USE_BOOST_PYTHON)
	# SFM-side wrapper lives in libs/SFM/PythonWrapper.cpp; compile it into the
	# same pyOpenMVS module so a single .pyd exposes both Scene (MVS) and
	# SfMScene (SFM) without juggling two extension modules.
	GET_FILENAME_COMPONENT(PATH_SFMPythonWrapper_cpp
		${CMAKE_CURRENT_SOURCE_DIR}/../SFM/PythonWrapper.cpp ABSOLUTE)
	# Create the Python wrapper
	cxx_library_with_type(pyOpenMVS "Libs" "SHARED" "${cxx_default}"
		${PATH_PythonWrapper_cpp}
		${PATH_SFMPythonWrapper_cpp}
	)
	# Link its dependencies; CUDA_ARCHITECTURES intentionally not set here so
	# pyOpenMVS inherits the global CMAKE_CUDA_ARCHITECTURES (defaulted to
	# "native" by the root CMakeLists when undefined) and respects any
	# -DCMAKE_CUDA_ARCHITECTURES=... the user or toolchain provides.
	TARGET_LINK_LIBRARIES(pyOpenMVS PRIVATE MVS SFM ${OpenMVS_PYTHON_LIBS} ${Boost_PYTHON_LIBS})
	# Output as a Python extension module: no "lib" prefix, ".pyd" suffix on
	# Windows so the standard `import pyOpenMVS` machinery loads it directly.
	SET_TARGET_PROPERTIES(pyOpenMVS PROPERTIES PREFIX "")
	IF(WIN32)
		SET_TARGET_PROPERTIES(pyOpenMVS PROPERTIES SUFFIX ".pyd")
	ENDIF()
	# Install pyOpenMVS.pyd alongside the openmvs/__init__.py loader so users
	# get a self-contained package: import openmvs (which adds CUDA + co-located
	# DLL search paths, then re-exports pyOpenMVS).
	INSTALL(TARGETS pyOpenMVS DESTINATION "${PYTHON_INSTALL_PATH}/openmvs")
	INSTALL(FILES ${CMAKE_SOURCE_DIR}/build/python/__init__.py
		DESTINATION "${PYTHON_INSTALL_PATH}/openmvs")
endif()

# Install
SET_TARGET_PROPERTIES(MVS PROPERTIES
	PUBLIC_HEADER "${LIBRARY_FILES_H}")
INSTALL(TARGETS MVS
	EXPORT OpenMVSTargets
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/MVS")
