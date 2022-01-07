# CMake powered build system for OpenMVS

########################################################################
#
# Project-wide settings
CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)
IF(POLICY CMP0011)
	CMAKE_POLICY(SET CMP0011 NEW)
ENDIF()
IF(POLICY CMP0074)
	CMAKE_POLICY(SET CMP0074 NEW)
ENDIF()
IF(POLICY CMP0104)
	CMAKE_POLICY(SET CMP0104 NEW)
ENDIF()

# Load automatically VCPKG toolchain if available
IF(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
	SET(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
	IF(NOT DEFINED VCPKG_TARGET_TRIPLET AND DEFINED ENV{VCPKG_DEFAULT_TRIPLET})
		SET(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}" CACHE STRING "")
	ENDIF()
ENDIF()

# Name of the project.
#
# CMake files in this project can refer to the root source directory
# as ${OpenMVS_SOURCE_DIR} and to the root binary directory as
# ${OpenMVS_BINARY_DIR}.
PROJECT(OpenMVS)

SET(OpenMVS_MAJOR_VERSION 2)
SET(OpenMVS_MINOR_VERSION 0)
SET(OpenMVS_PATCH_VERSION 0)
SET(OpenMVS_VERSION ${OpenMVS_MAJOR_VERSION}.${OpenMVS_MINOR_VERSION}.${OpenMVS_PATCH_VERSION})

# List configuration options
SET(OpenMVS_BUILD_TOOLS ON CACHE BOOL "Build example applications")
SET(OpenMVS_USE_NONFREE ON CACHE BOOL "Build non-free (patented) functionality")
SET(OpenMVS_USE_OPENMP ON CACHE BOOL "Enable OpenMP library")
SET(OpenMVS_USE_OPENGL ON CACHE BOOL "Enable OpenGL library")
SET(OpenMVS_USE_BREAKPAD ON CACHE BOOL "Enable BreakPad library")
SET(OpenMVS_USE_CERES OFF CACHE BOOL "Enable CERES optimization library")
SET(OpenMVS_USE_CUDA ON CACHE BOOL "Enable CUDA library")
SET(OpenMVS_USE_FAST_FLOAT2INT ON CACHE BOOL "Use an optimized code to convert real numbers to int")
SET(OpenMVS_USE_FAST_INVSQRT OFF CACHE BOOL "Use an optimized code to compute the inverse square root (slower in fact on modern compilers)")
SET(OpenMVS_USE_FAST_CBRT ON CACHE BOOL "Use an optimized code to compute the cubic root")
SET(OpenMVS_USE_SSE ON CACHE BOOL "Enable SSE optimizations")
SET(OpenMVS_MAX_CUDA_COMPATIBILITY OFF CACHE BOOL "Build for maximum CUDA device compatibility")

# Define helper functions and macros.
INCLUDE(build/Utils.cmake)

# Init session with macros defined in Utils.cmake
GetOperatingSystemArchitectureBitness(SYSTEM)
ComposePackageLibSuffix()
ConfigCompilerAndLinker()
ConfigLibrary()

# Find dependencies:
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/Modules)

# Find required packages
SET(OpenMVS_EXTRA_INCLUDES "")
SET(OpenMVS_DEFINITIONS "")
SET(OpenMVS_EXTRA_LIBS "")

if(OpenMVS_USE_OPENMP)
	SET(OpenMP_LIBS "")
	FIND_PACKAGE(OpenMP)
	if(OPENMP_FOUND)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		LIST(APPEND OpenMVS_DEFINITIONS -D_USE_OPENMP)
		SET(_USE_OPENMP TRUE)
		#cmake only check for separate OpenMP library on AppleClang 7+
		#https://github.com/Kitware/CMake/blob/42212f7539040139ecec092547b7d58ef12a4d72/Modules/FindOpenMP.cmake#L252
		if (CMAKE_CXX_COMPILER_ID MATCHES "AppleClang" AND (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0"))
			SET(OpenMP_LIBS ${OpenMP_libomp_LIBRARY})
			LIST(APPEND OpenMVS_EXTRA_LIBS ${OpenMP_LIBS})
		endif()
	else()
		MESSAGE("-- Can't find OpenMP. Continuing without it.")
	endif()
endif()

if(OpenMVS_USE_OPENGL)
	if(POLICY CMP0072)
		cmake_policy(SET CMP0072 NEW)
	endif()
	FIND_PACKAGE(OpenGL)
	if(OPENGL_FOUND)
		INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
		LIST(APPEND OpenMVS_DEFINITIONS -D_USE_OPENGL)
		ADD_DEFINITIONS(${OpenGL_DEFINITIONS})
		SET(_USE_OPENGL TRUE)
	else()
		MESSAGE("-- Can't find OpenGL. Continuing without it.")
	endif()
endif()

if(OpenMVS_USE_CUDA)
	FIND_PACKAGE(CUDA)
	if(CUDA_FOUND)
		ENABLE_LANGUAGE(CUDA)
		# CUDA-11.x can not be compiled using C++14 standard on Windows
		string(REGEX MATCH "^[0-9]+" CUDA_MAJOR ${CMAKE_CUDA_COMPILER_VERSION})
		if(${CUDA_MAJOR} GREATER 10 AND CMAKE_CXX_COMPILER MATCHES "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "19.29")
			message("Working around windows build failure with visual studio. Visual studio 16.10 introduced a compiler bug with compilng CUDA code with C++14. Set the cuda standard to 17 as a workaround.")
			set(CMAKE_CUDA_STANDARD 17)
		endif()
		if(NOT OpenMVS_MAX_CUDA_COMPATIBILITY)
			if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
				SET(CMAKE_CUDA_ARCHITECTURES 75)
			endif()
		else()
			# Build for maximum compatibility
			# https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/

			# Extract list of arch and gencodes
			EXECUTE_PROCESS(COMMAND "${CMAKE_CUDA_COMPILER}" --list-gpu-arch 
				OUTPUT_VARIABLE LIST_GPU_ARCH 
				ERROR_QUIET)
			STRING(REPLACE "\r" "" LIST_GPU_ARCH ${LIST_GPU_ARCH})
			STRING(REPLACE "\n" ";" LIST_GPU_ARCH ${LIST_GPU_ARCH})

			EXECUTE_PROCESS(COMMAND "${CMAKE_CUDA_COMPILER}" --list-gpu-code 
				OUTPUT_VARIABLE LIST_GPU_CODE 
				ERROR_QUIET)
			STRING(REPLACE "\r" "" LIST_GPU_CODE ${LIST_GPU_CODE})
			STRING(REPLACE "\n" ";" LIST_GPU_CODE ${LIST_GPU_CODE})

			LIST(GET LIST_GPU_CODE 0 TARGET_GPU_CODE)
			SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=${TARGET_GPU_CODE}")

			SET(IDX 0)
			foreach(GPU_ARCH ${LIST_GPU_ARCH})
				LIST(GET LIST_GPU_CODE ${IDX} GPU_CODE)
				SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=${GPU_ARCH},code=${GPU_CODE}")
				MATH(EXPR IDX "${IDX}+1")
			endforeach()
			MESSAGE("-- Set CUDA flags: " ${CMAKE_CUDA_FLAGS})
		endif()
		SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
		INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
		LIST(APPEND OpenMVS_DEFINITIONS -D_USE_CUDA)
		SET(_USE_CUDA TRUE)
	else()
		SET(CUDA_CUDA_LIBRARY "")
		MESSAGE("-- Can't find CUDA. Continuing without it.")
	endif()
else()
	SET(CUDA_CUDA_LIBRARY "")
endif()

if(OpenMVS_USE_BREAKPAD)
	FIND_PACKAGE(BREAKPAD)
	if(BREAKPAD_FOUND)
		INCLUDE_DIRECTORIES(${BREAKPAD_INCLUDE_DIRS})
		LIST(APPEND OpenMVS_DEFINITIONS -D_USE_BREAKPAD)
		ADD_DEFINITIONS(${BREAKPAD_DEFINITIONS})
		SET(_USE_BREAKPAD TRUE)
		LIST(APPEND OpenMVS_EXTRA_LIBS ${BREAKPAD_LIBS})
	else()
		MESSAGE("-- Can't find BreakPad. Continuing without it.")
	endif()
endif()

FIND_PACKAGE(Boost COMPONENTS iostreams program_options system serialization REQUIRED)
if(Boost_FOUND)
	LIST(APPEND OpenMVS_EXTRA_INCLUDES ${Boost_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_BOOST)
	ADD_DEFINITIONS(${Boost_DEFINITIONS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
	SET(_USE_BOOST TRUE)
endif()

FIND_PACKAGE(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
	LIST(APPEND OpenMVS_EXTRA_INCLUDES ${EIGEN3_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_EIGEN)
	ADD_DEFINITIONS(${EIGEN3_DEFINITIONS})
	SET(_USE_EIGEN TRUE)
	MESSAGE(STATUS "Eigen ${EIGEN3_VERSION} found (include: ${EIGEN3_INCLUDE_DIR})")
endif()

FIND_PACKAGE(OpenCV REQUIRED)
if(OpenCV_FOUND)
	LIST(APPEND OpenMVS_EXTRA_INCLUDES ${OpenCV_INCLUDE_DIRS})
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
	ADD_DEFINITIONS(${OpenCV_DEFINITIONS})
	SET(_USE_OPENCV TRUE)
	MESSAGE(STATUS "OpenCV ${OpenCV_VERSION} found (include: ${OpenCV_INCLUDE_DIRS})")
else()
	MESSAGE("-- Can't find OpenCV. Please specify OpenCV directory using OpenCV_DIR variable")
endif()

LIST(REMOVE_DUPLICATES OpenMVS_EXTRA_INCLUDES)
LIST(REMOVE_DUPLICATES OpenMVS_EXTRA_LIBS)

# Set defines
if(OpenMVS_USE_NONFREE)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_NONFREE)
	SET(_USE_NONFREE TRUE)
endif()
if(OpenMVS_USE_FAST_FLOAT2INT)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_FAST_FLOAT2INT)
	SET(_USE_FAST_FLOAT2INT TRUE)
endif()
if(OpenMVS_USE_FAST_INVSQRT)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_FAST_INVSQRT)
	SET(_USE_FAST_INVSQRT TRUE)
endif()
if(OpenMVS_USE_FAST_CBRT)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_FAST_CBRT)
	SET(_USE_FAST_CBRT TRUE)
endif()
if(OpenMVS_USE_SSE)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_SSE)
	SET(_USE_SSE TRUE)
endif()
if(OpenMVS_USE_CERES)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_CERES)
	SET(_USE_CERES TRUE)
endif()

ADD_DEFINITIONS(${OpenMVS_DEFINITIONS})

INCLUDE_DIRECTORIES("${OpenMVS_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_BINARY_DIR}")

# Add modules
ADD_SUBDIRECTORY(libs)
if(OpenMVS_BUILD_TOOLS)
	ADD_SUBDIRECTORY(apps)
endif()
ADD_SUBDIRECTORY(docs)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE OpenMVS)

# Install the export set for use with the install-tree
INSTALL(EXPORT OpenMVSTargets
	NAMESPACE OpenMVS::
	DESTINATION "${INSTALL_CMAKE_DIR}")

# Install configuration file
CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/build/Templates/ConfigLocal.h.in" "${CMAKE_BINARY_DIR}/ConfigLocal.h")
INSTALL(FILES "${CMAKE_BINARY_DIR}/ConfigLocal.h" DESTINATION "${INSTALL_INCLUDE_DIR}")

# Create the OpenMVSConfig.cmake and OpenMVSConfigVersion files
INCLUDE(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_BINARY_DIR}/OpenMVSConfigVersion.cmake"
	VERSION ${OpenMVS_VERSION}
	COMPATIBILITY AnyNewerVersion)
SET(INSTALL_INCLUDE_DIR_IN ${INSTALL_INCLUDE_DIR_PREFIX} ${OpenMVS_EXTRA_INCLUDES})
SET(INSTALL_CMAKE_DIR_IN ${INSTALL_CMAKE_DIR_PREFIX})
configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/build/Templates/OpenMVSConfig.cmake.in"
	"${PROJECT_BINARY_DIR}/OpenMVSConfig.cmake"
	INSTALL_DESTINATION ${PROJECT_BINARY_DIR}
	NO_SET_AND_CHECK_MACRO)
# Install the OpenMVSConfig.cmake and OpenMVSConfigVersion.cmake
INSTALL(FILES
	"${PROJECT_BINARY_DIR}/OpenMVSConfig.cmake"
	"${PROJECT_BINARY_DIR}/OpenMVSConfigVersion.cmake"
	DESTINATION "${INSTALL_CMAKE_DIR}")

# uninstall target
CONFIGURE_FILE(
	"${CMAKE_CURRENT_SOURCE_DIR}/build/Templates/cmake_uninstall.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
	IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
	COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
