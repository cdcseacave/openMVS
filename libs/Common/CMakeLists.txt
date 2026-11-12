# Find required packages
find_exported_dependency(nanoflann REQUIRED)
SET(COMMON_EXTRA_LIBS "")
if(nanoflann_FOUND)
	ADD_DEFINITIONS(${nanoflann_DEFINITIONS})
	LIST(APPEND COMMON_EXTRA_LIBS nanoflann::nanoflann)
	MESSAGE(STATUS "nanoflann ${nanoflann_VERSION} found")
endif()

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.hpp" "*.inl")
if(_USE_METAL)
	FILE(GLOB LIBRARY_FILES_METAL "*.mm")
	LIST(APPEND LIBRARY_FILES_C ${LIBRARY_FILES_METAL})
endif()

cxx_library_with_type(Common "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(Common PRIVATE "Common.h")
endif()

if(_USE_METAL)
	set_source_files_properties(${LIBRARY_FILES_METAL} PROPERTIES
		SKIP_PRECOMPILE_HEADERS ON
		COMPILE_OPTIONS "-fobjc-arc")
endif()

if(_USE_CUDA AND NOT _USE_HIP)
	# the compute capabilities the CUDA code is compiled for, see initDevices()
	TARGET_COMPILE_DEFINITIONS(Common PRIVATE _CUDA_ARCHS_SASS=${_CUDA_ARCHS_SASS} _CUDA_ARCHS_SASS_EXACT=${_CUDA_ARCHS_SASS_EXACT} _CUDA_ARCH_PTX=${_CUDA_ARCH_PTX})
endif()

# Add public include directories that propagate to dependent targets
target_include_directories(Common PUBLIC
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
	$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>
)
# the headers, also in the projects using OpenMVS, depend on the C++ standard (ex. the
# BS::thread_pool layout) and on the library type: tell them we are building shared
# libraries, so they pick dllimport and the cross-DLL-friendly path for things like
# boost exception-unwinding (see Types.h)
TARGET_COMPILE_FEATURES(Common PUBLIC cxx_std_${CMAKE_CXX_STANDARD})
if(BUILD_SHARED_LIBS)
	TARGET_COMPILE_DEFINITIONS(Common PUBLIC OPENMVS_SHARED)
endif()
if(CLANG)
	# the libraries are built against libc++ (see ConfigCompilerAndLinker), so the
	# installed projects must use it too; the build tree gets it from CMAKE_CXX_FLAGS
	TARGET_COMPILE_OPTIONS(Common INTERFACE $<INSTALL_INTERFACE:$<$<COMPILE_LANGUAGE:CXX>:-stdlib=libc++>>)
	TARGET_LINK_OPTIONS(Common INTERFACE $<INSTALL_INTERFACE:-stdlib=libc++>)
endif()

# Link its dependencies with PUBLIC visibility so they propagate
TARGET_LINK_LIBRARIES(Common PUBLIC ${OpenMVS_EXTRA_LIBS} ${COMMON_EXTRA_LIBS})

# Install
SET_TARGET_PROPERTIES(Common PROPERTIES
	PUBLIC_HEADER "${LIBRARY_FILES_H}")
INSTALL(TARGETS Common
	EXPORT OpenMVSTargets
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/Common")
