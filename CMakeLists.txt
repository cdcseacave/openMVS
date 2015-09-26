# CMake powered build system for OpenMVS

########################################################################
#
# Project-wide settings
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)

# Find dependencies:
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/build/Modules)

# Name of the project.
#
# CMake files in this project can refer to the root source directory
# as ${OpenMVS_SOURCE_DIR} and to the root binary directory as
# ${OpenMVS_BINARY_DIR}.
PROJECT(OpenMVS)

# fix CMake IntDir variable
if(MSVC AND "${MSVC_VERSION}" STRGREATER "1500")
	set(CMAKE_CFG_INTDIR "$(Platform)/$(Configuration)")
endif()
set(COTIRE_INTDIR "cotire")

# Define helper functions and macros.
cmake_policy(SET CMP0011 OLD)
INCLUDE(build/Utils.cmake)
if(ENABLE_PRECOMPILED_HEADERS)
	INCLUDE(build/Cotire.cmake)
endif()


# Init session with macros defined in Utils.cmake
GetOperatingSystemArchitectureBitness(SYSTEM)
ComposePackageLibSuffix()
ConfigCompilerAndLinker()

# List config options
SET(OpenMVS_USE_NONFREE ON CACHE BOOL "Build non-free (patented) functionality")
SET(OpenMVS_USE_EXIV2 OFF CACHE BOOL "Link and use EXIV2 library")
SET(OpenMVS_USE_FAST_FLOAT2INT ON CACHE BOOL "Use an optimized code to convert real numbers to int")
SET(OpenMVS_USE_FAST_INVSQRT OFF CACHE BOOL "Use an optimized code to compute the inverse square root (slower in fact on modern compilers)")
SET(OpenMVS_USE_FAST_CBRT ON CACHE BOOL "Use an optimized code to compute the cubic root")
SET(OpenMVS_USE_SSE ON CACHE BOOL "Enable SSE optimizations")
SET(OpenMVS_USE_OPENMP ON CACHE BOOL "Enable OpenMP library")
SET(OpenMVS_USE_CUDA ON CACHE BOOL "Enable CUDA library")
SET(OpenMVS_USE_BREAKPAD ON CACHE BOOL "Enable BreakPad library")

include_directories("${OpenMVS_SOURCE_DIR}")

# Find required packages
if(OpenMVS_USE_OPENMP)
	FIND_PACKAGE(OpenMP)
	if(OPENMP_FOUND)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		add_definitions(-D_USE_OPENMP)
	else()
		message("-- Can't find OpenMP. Continuing without it.")
	endif()
endif()

if(OpenMVS_USE_CUDA)
	FIND_PACKAGE(CUDA)
	if(CUDA_FOUND)
		include_directories(${CUDA_INCLUDE_DIRS})
		add_definitions(-D_USE_CUDA)
	else()
		set(CUDA_LIBRARIES "")
		message("-- Can't find CUDA. Continuing without it.")
	endif()
endif()

if(OpenMVS_USE_BREAKPAD)
	FIND_PACKAGE(BREAKPAD)
	if(BREAKPAD_FOUND)
		include_directories(${BREAKPAD_INCLUDE_DIRS})
		add_definitions(-D_USE_BREAKPAD)
	else()
		message("-- Can't find BreakPad. Continuing without it.")
	endif()
endif()

FIND_PACKAGE(Boost ${SYSTEM_PACKAGE_REQUIRED} COMPONENTS iostreams program_options system serialization)
if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	add_definitions(${Boost_DEFINITIONS})
	link_directories(${Boost_LIBRARY_DIRS})
endif()

FIND_PACKAGE(Eigen ${SYSTEM_PACKAGE_REQUIRED})
if(EIGEN_FOUND)
	include_directories(${EIGEN_INCLUDE_DIRS})
	add_definitions(${EIGEN_DEFINITIONS})
endif()

FIND_PACKAGE(OpenCV ${SYSTEM_PACKAGE_REQUIRED})
if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	add_definitions(${OpenCV_DEFINITIONS})
endif()

# Set defines
SET(OpenMVS_DEFINITIONS "")
if(OpenMVS_USE_NONFREE)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_NONFREE) 
endif()
if(OpenMVS_USE_EXIV2)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_EXIV2) 
endif()
if(OpenMVS_USE_FAST_FLOAT2INT)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_FAST_FLOAT2INT) 
endif()
if(OpenMVS_USE_FAST_INVSQRT)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_FAST_INVSQRT) 
endif()
if(OpenMVS_USE_FAST_CBRT)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_FAST_CBRT) 
endif()
if(OpenMVS_USE_SSE)
	LIST(APPEND OpenMVS_DEFINITIONS -D_USE_SSE) 
endif()
LIST(APPEND OpenMVS_DEFINITIONS -D_USE_BOOST) 
ADD_DEFINITIONS(${OpenMVS_DEFINITIONS})

# Add modules
ADD_SUBDIRECTORY(libs)
ADD_SUBDIRECTORY(apps)
ADD_SUBDIRECTORY(docs)
