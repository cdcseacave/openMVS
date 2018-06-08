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
if(CMAKE_COMPILER_IS_GNUCXX)
	FIND_PACKAGE(PkgConfig QUIET)
	pkg_search_module(GLFW QUIET glfw3)
	if(GLFW_FOUND)
		INCLUDE_DIRECTORIES(${GLFW_INCLUDE_DIRS})
		ADD_DEFINITIONS(${GLFW_DEFINITIONS})
		MESSAGE(STATUS "GLFW3 ${GLFW_VERSION} found (include: ${GLFW_INCLUDE_DIRS})")
	else()
		MESSAGE("-- Can't find GLFW3. Continuing without it.")
		RETURN()
	endif()
else()
	FIND_PACKAGE(glfw3 QUIET)
	if(glfw3_FOUND)
		INCLUDE_DIRECTORIES(${glfw3_INCLUDE_DIRS})
		ADD_DEFINITIONS(${glfw3_DEFINITIONS})
		MESSAGE(STATUS "GLFW3 ${glfw3_VERSION} found (include: ${glfw3_INCLUDE_DIRS})")
	else()
		MESSAGE("-- Can't find GLFW3. Continuing without it.")
		RETURN()
	endif()
endif()

# List sources files
FILE(GLOB PCH_C "Common.cpp")

if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# Place Common.cpp as the first file in the list
# needed by cotire when setting PCH manually
LIST(REMOVE_ITEM LIBRARY_FILES_C ${PCH_C})
SET(LIBRARY_FILES_C "${PCH_C};${LIBRARY_FILES_C}")

cxx_executable_with_flags_no_pch(${VIEWER_NAME} "Apps" "${cxx_default}" "MVS;${OPENGL_LIBRARIES};${GLEW_LIBRARY};${GLFW_STATIC_LIBRARIES};GLEW::GLEW;${glfw3_LIBRARY};${GLFW3_LIBRARY};glfw" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

# Manually set Common.h as the precompiled header
set_target_pch(${VIEWER_NAME} Common.h)

if(OpenMVS_USE_BREAKPAD AND BREAKPAD_FOUND)
	target_link_libraries(${VIEWER_NAME} ${BREAKPAD_LIBS})
endif()

# Install
INSTALL(TARGETS ${VIEWER_NAME}
	EXPORT 3DnovatorTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
