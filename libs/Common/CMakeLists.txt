# Find required packages
FIND_PACKAGE(nanoflann REQUIRED)
SET(COMMON_EXTRA_LIBS "")
if(nanoflann_FOUND)
	ADD_DEFINITIONS(${nanoflann_DEFINITIONS})
	LIST(APPEND COMMON_EXTRA_LIBS nanoflann::nanoflann)
	MESSAGE(STATUS "nanoflann ${nanoflann_VERSION} found")
endif()

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")
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

# Add public include directories that propagate to dependent targets
target_include_directories(Common PUBLIC
	$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
)

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
