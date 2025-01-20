if(NOT VIEWER_NAME)
	set(VIEWER_NAME "Viewer")
endif()

# Find required packages
FIND_PACKAGE(glad QUIET)
if(glad_FOUND)
	MESSAGE(STATUS "GLAD ${glad_VERSION} found")
else()
	MESSAGE("-- Can't find GLAD. Continuing without it.")
	RETURN()
endif()
FIND_PACKAGE(glfw3 QUIET)
if(glfw3_FOUND)
	MESSAGE(STATUS "GLFW3 ${glfw3_VERSION} found")
else()
	MESSAGE("-- Can't find GLFW3. Continuing without it.")
	RETURN()
endif()

# List sources files
if(MSVC)
	create_rc_files(${VIEWER_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/Viewer.ico")
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "${CMAKE_CURRENT_BINARY_DIR}/*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_executable_with_flags(${VIEWER_NAME} "Apps" "${cxx_default}" "MVS;glad::glad;${GLFW_STATIC_LIBRARIES};${glfw3_LIBRARY};${GLFW3_LIBRARY};glfw;${OpenMVS_EXTRA_LIBS}" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(${VIEWER_NAME} PRIVATE "Common.h")
endif()

# Install
INSTALL(TARGETS ${VIEWER_NAME}
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
