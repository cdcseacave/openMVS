if((NOT OpenMVS_USE_OPENGL) OR (NOT _USE_OPENGL))
	RETURN()
endif()

if(NOT VIEWER_NAME)
	set(VIEWER_NAME "Viewer")
endif()

# Find required packages
FIND_PACKAGE(GLEW QUIET)
if(GLEW_FOUND)
	INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIRS})
	ADD_DEFINITIONS(${GLEW_DEFINITIONS})
	MESSAGE(STATUS "GLEW ${GLEW_VERSION} found (include: ${GLEW_INCLUDE_DIRS})")
else()
	MESSAGE("-- Can't find GLEW. Continuing without it.")
	RETURN()
endif()
FIND_PACKAGE(glfw3 QUIET)
if(glfw3_FOUND)
	INCLUDE_DIRECTORIES(${glfw3_INCLUDE_DIRS})
	ADD_DEFINITIONS(${glfw3_DEFINITIONS})
	MESSAGE(STATUS "GLFW3 ${glfw3_VERSION} found (include: ${glfw3_INCLUDE_DIRS})")
else()
	MESSAGE("-- Can't find GLFW3. Continuing without it.")
	RETURN()
endif()

# List sources files
if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_executable_with_flags(${VIEWER_NAME} "Apps" "${cxx_default}" "MVS;${OPENGL_LIBRARIES};${GLEW_LIBRARY};${GLFW_STATIC_LIBRARIES};GLEW::GLEW;${glfw3_LIBRARY};${GLFW3_LIBRARY};glfw;${OpenMVS_EXTRA_LIBS}" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(${VIEWER_NAME} PRIVATE "Common.h")
endif()

# Install
INSTALL(TARGETS ${VIEWER_NAME}
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
