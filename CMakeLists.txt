# CMake powered build system for OpenMVS

########################################################################
#
# Project-wide settings
CMAKE_MINIMUM_REQUIRED(VERSION 3.18)
IF(POLICY CMP0011)
	# Included scripts do automatic cmake_policy() PUSH and POP.
	CMAKE_POLICY(SET CMP0011 NEW)
ENDIF()
IF(POLICY CMP0074)
	# find_package() uses <PackageName>_ROOT variables.
	CMAKE_POLICY(SET CMP0074 NEW)
ENDIF()
IF(POLICY CMP0104)
	# Initialize CMAKE_CUDA_ARCHITECTURES when CMAKE_CUDA_COMPILER_ID is NVIDIA. Raise an error if CUDA_ARCHITECTURES is empty.
	CMAKE_POLICY(SET CMP0104 NEW)
ENDIF()
IF(POLICY CMP0146)
	# Use CMake's first-class CUDA language support instead of FindCUDA().
	# note: not required, but forced by OpenCV old support for CUDA
	CMAKE_POLICY(SET CMP0146 OLD)
ENDIF()
IF(POLICY CMP0167)
	# The FindBoost module is removed.
	CMAKE_POLICY(SET CMP0167 NEW)
ENDIF()

# List configuration options
OPTION(OpenMVS_BUILD_TOOLS "Build example applications" ON)
OPTION(OpenMVS_BUILD_VIEWER "Build viewer application" ON)
OPTION(OpenMVS_USE_OPENMP "Enable OpenMP library" ON)
OPTION(OpenMVS_USE_BREAKPAD "Enable BreakPad library" ON)
OPTION(OpenMVS_USE_PYTHON "Enable Python library bindings" ON)
OPTION(OpenMVS_USE_CERES "Enable CERES optimization library" OFF)
OPTION(OpenMVS_USE_CUDA "Enable CUDA library" ON)
OPTION(OpenMVS_USE_FAST_FLOAT2INT "Use an optimized code to convert real numbers to int" OFF)
OPTION(OpenMVS_USE_FAST_INVSQRT "Use an optimized code to compute the inverse square root (slower in fact on modern compilers)" OFF)
OPTION(OpenMVS_USE_FAST_CBRT "Use an optimized code to compute the cubic root" OFF)
OPTION(OpenMVS_USE_SSE "Enable SSE optimizations" ON)
OPTION(OpenMVS_MAX_CUDA_COMPATIBILITY "Build for maximum CUDA device compatibility" OFF)
OPTION(OpenMVS_ENABLE_IPO "Whether to enable interprocedural optimization" ON)
OPTION(OpenMVS_ENABLE_TESTS "Enable test code" ON)

# Disable CUDA on MacOS
IF(APPLE)
	SET(OpenMVS_USE_CUDA OFF)
	MESSAGE(STATUS "Disabling CUDA on MacOS")
ENDIF()

# Load automatically VCPKG toolchain if available
IF(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
	SET(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
	IF(NOT DEFINED VCPKG_TARGET_TRIPLET AND DEFINED ENV{VCPKG_DEFAULT_TRIPLET})
		SET(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_DEFAULT_TRIPLET}" CACHE STRING "")
	ENDIF()
ENDIF()
IF(OpenMVS_BUILD_TOOLS AND OpenMVS_BUILD_VIEWER)
	LIST(APPEND VCPKG_MANIFEST_FEATURES "viewer")
ENDIF()
IF(OpenMVS_USE_CUDA)
	LIST(APPEND VCPKG_MANIFEST_FEATURES "cuda")
ENDIF()
IF(OpenMVS_USE_PYTHON)
	LIST(APPEND VCPKG_MANIFEST_FEATURES "python")
	SET(PARTIAL_BUILD_SHARED_LIBS ON)
ENDIF()

# Name of the project:
#   CMake files in this project can refer to the root source directory
#   as ${OpenMVS_SOURCE_DIR} and to the root binary directory as
#   ${OpenMVS_BINARY_DIR}.
PROJECT(OpenMVS LANGUAGES CXX)

SET(OpenMVS_MAJOR_VERSION 2)
SET(OpenMVS_MINOR_VERSION 3)
SET(OpenMVS_PATCH_VERSION 0)
SET(OpenMVS_VERSION ${OpenMVS_MAJOR_VERSION}.${OpenMVS_MINOR_VERSION}.${OpenMVS_PATCH_VERSION})

# Disable SSE on unsupported platforms
IF(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|ARM|aarch64|AARCH64)")
	SET(OpenMVS_USE_SSE OFF)
	MESSAGE(STATUS "Disabling SSE on ARM platform")
ENDIF()

# Set default build type to Release if not already set and using single-configuration generator.
IF(CMAKE_CONFIGURATION_TYPES)
	MESSAGE(STATUS "Multi-configuration generator: ${CMAKE_GENERATOR}")
ELSE()
	IF(NOT CMAKE_BUILD_TYPE)
		SET(CMAKE_BUILD_TYPE "Release" CACHE STRING
			"Choose the type of build, options are: None(debug information only), Debug, Release, RelWithDebInfo, MinSizeRel."
			FORCE) # FORCE to override user settings from command line if they are empty
	ENDIF()
	MESSAGE(STATUS "Build configuration: ${CMAKE_GENERATOR} - ${CMAKE_BUILD_TYPE}")
ENDIF()

# Build position-independent code, so that shared libraries can link against OpenMVS static libraries.
SET(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Set global property to avoid cyclic dependencies.
SET_PROPERTY(GLOBAL PROPERTY GLOBAL_DEPENDS_NO_CYCLES ON)

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
SET(OpenMVS_EXTRA_LIBS "")

if(OpenMVS_ENABLE_IPO)
	INCLUDE(CheckIPOSupported)
	check_ipo_supported(RESULT IPO_SUPPORTED OUTPUT IPO_ERROR)
	if(IPO_SUPPORTED)
		SET_PROPERTY(GLOBAL PROPERTY CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
		MESSAGE(STATUS "Interprocedural optimization enabled")
	else()
		MESSAGE(WARNING "Interprocedural optimization is not supported: ${IPO_ERROR}")
	endif()
endif()

if(OpenMVS_USE_OPENMP)
	FIND_PACKAGE(OpenMP QUIET)
	if(OPENMP_FOUND)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		SET(_USE_OPENMP TRUE)
		# cmake only check for separate OpenMP library on AppleClang 7+
		# https://github.com/Kitware/CMake/blob/42212f7539040139ecec092547b7d58ef12a4d72/Modules/FindOpenMP.cmake#L252
		if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang" AND (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0"))
			LIST(APPEND OpenMVS_EXTRA_LIBS ${OpenMP_libomp_LIBRARY})
		endif()
		MESSAGE(STATUS "Found OpenMP " ${OpenMP_VERSION} " " ${OpenMP_LIBRARY})
	else()
		MESSAGE(STATUS "Can't find OpenMP. Continuing without it.")
	endif()
endif()

if(OpenMVS_USE_CUDA)
	INCLUDE(CheckLanguage)
	CHECK_LANGUAGE(CUDA)
	if(CMAKE_CUDA_COMPILER)
		# Finding CUDA fails on some systems if paths to nvcc / cuda library are not set; ex. on linux:
		# export PATH="/usr/local/cuda/bin:$PATH"
		# export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
		if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
			SET(CMAKE_CUDA_ARCHITECTURES "native")
		endif()
		ENABLE_LANGUAGE(CUDA)
		SET(CUDA_FOUND TRUE)
		SET(CMAKE_CUDA_STANDARD 17)
		SET(CMAKE_CUDA_STANDARD_REQUIRED ON)
		FIND_PACKAGE(CUDAToolkit REQUIRED)
		if(OpenMVS_MAX_CUDA_COMPATIBILITY)
			EXECUTE_PROCESS(COMMAND "${CMAKE_CUDA_COMPILER}" --list-gpu-arch
				OUTPUT_VARIABLE LIST_GPU_ARCH
				ERROR_QUIET)
			if(NOT LIST_GPU_ARCH)
				MESSAGE(WARNING "Cannot compile for max CUDA compatibility, nvcc does not support --list-gpu-arch")
				SET(OpenMVS_MAX_CUDA_COMPATIBILITY OFF)
			endif()
		endif()
		if(OpenMVS_MAX_CUDA_COMPATIBILITY)
			# Build for maximum compatibility
			# https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
			UNSET(CMAKE_CUDA_ARCHITECTURES)
			# Extract list of arch and gencodes
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
			MESSAGE(STATUS "Set CUDA flags: " ${CMAKE_CUDA_FLAGS})
		endif()
		SET(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
		SET(CMAKE_CUDA_RESOLVE_DEVICE_SYMBOLS ON)
		SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
		LIST(APPEND OpenMVS_EXTRA_INCLUDES ${CUDAToolkit_INCLUDE_DIRS})
		LIST(APPEND OpenMVS_EXTRA_LIBS ${CUDAToolkit_LIBRARIES} cuda cudart curand)
		SET(_USE_CUDA TRUE)
		MESSAGE(STATUS "Found CUDA: " ${CMAKE_CUDA_COMPILER_VERSION} " with GPU arch: " ${CMAKE_CUDA_ARCHITECTURES})
	else()
		MESSAGE(STATUS "Can't find CUDA. Continuing without it.")
	endif()
endif()

if(OpenMVS_USE_BREAKPAD)
	FIND_PACKAGE(BREAKPAD QUIET)
	if(BREAKPAD_FOUND)
		INCLUDE_DIRECTORIES(${BREAKPAD_INCLUDE_DIRS})
		ADD_DEFINITIONS(${BREAKPAD_DEFINITIONS})
		SET(_USE_BREAKPAD TRUE)
		LIST(APPEND OpenMVS_EXTRA_LIBS ${BREAKPAD_LIBS})
	else()
		MESSAGE(STATUS "Can't find BreakPad. Continuing without it.")
	endif()
endif()

SET(Boost_EXTRA_COMPONENTS "")
if(OpenMVS_USE_PYTHON)
	FIND_PACKAGE(Python3 COMPONENTS Interpreter Development REQUIRED)
	if(Python3_FOUND)
		LIST(APPEND OpenMVS_EXTRA_INCLUDES ${Python3_INCLUDE_DIRS})
		LIST(APPEND OpenMVS_EXTRA_LIBS ${Python3_LIBRARIES})
		LIST(APPEND Boost_EXTRA_COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
		MESSAGE(STATUS "Python ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR} found (include: ${Python3_INCLUDE_DIRS})")
	else()
		MESSAGE(STATUS "Can't find Python. Continuing without it.")
	endif()
endif()

FIND_PACKAGE(Boost REQUIRED COMPONENTS iostreams program_options system serialization OPTIONAL_COMPONENTS ${Boost_EXTRA_COMPONENTS})
if(Boost_FOUND)
	LIST(APPEND OpenMVS_EXTRA_INCLUDES ${Boost_INCLUDE_DIRS})
	ADD_DEFINITIONS(${Boost_DEFINITIONS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
	SET(_USE_BOOST TRUE)
	if(OpenMVS_USE_PYTHON AND Boost_${Boost_EXTRA_COMPONENTS}_FOUND)
		SET(_USE_BOOST_PYTHON TRUE)
	endif()
endif()

FIND_PACKAGE(Eigen3 3.4 REQUIRED)
if(EIGEN3_FOUND)
	LIST(APPEND OpenMVS_EXTRA_INCLUDES ${EIGEN3_INCLUDE_DIR})
	ADD_DEFINITIONS(${EIGEN3_DEFINITIONS})
	SET(_USE_EIGEN TRUE)
	MESSAGE(STATUS "Eigen ${EIGEN3_VERSION} found (include: ${EIGEN3_INCLUDE_DIR})")
endif()

FIND_PACKAGE(OpenCV REQUIRED)
if(OpenCV_FOUND)
	LIST(APPEND OpenMVS_EXTRA_INCLUDES ${OpenCV_INCLUDE_DIRS})
	ADD_DEFINITIONS(${OpenCV_DEFINITIONS})
	SET(_USE_OPENCV TRUE)
	MESSAGE(STATUS "OpenCV ${OpenCV_VERSION} found (include: ${OpenCV_INCLUDE_DIRS})")
else()
	MESSAGE(STATUS "Can't find OpenCV. Please specify OpenCV directory using OpenCV_DIR variable")
endif()

LIST(REMOVE_DUPLICATES OpenMVS_EXTRA_INCLUDES)
LIST(REMOVE_DUPLICATES OpenMVS_EXTRA_LIBS)

# Set defines
if(OpenMVS_USE_FAST_FLOAT2INT)
	SET(_USE_FAST_FLOAT2INT TRUE)
endif()
if(OpenMVS_USE_FAST_INVSQRT)
	SET(_USE_FAST_INVSQRT TRUE)
endif()
if(OpenMVS_USE_FAST_CBRT)
	SET(_USE_FAST_CBRT TRUE)
endif()
if(OpenMVS_USE_SSE)
	SET(_USE_SSE TRUE)
endif()
if(OpenMVS_USE_CERES)
	SET(_USE_CERES TRUE)
endif()

INCLUDE_DIRECTORIES(${OpenMVS_EXTRA_INCLUDES})
INCLUDE_DIRECTORIES(${OpenMVS_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})

# Add modules
ADD_SUBDIRECTORY(libs)
if(OpenMVS_BUILD_TOOLS)
	ADD_SUBDIRECTORY(apps)
endif()
ADD_SUBDIRECTORY(docs)

if (OpenMVS_ENABLE_TESTS)
	# enable testing functionality
	ENABLE_TESTING()
	# define tests
	ADD_TEST(NAME UnitTests COMMAND $<TARGET_FILE:Tests> "0")
	ADD_TEST(NAME PipelineTest COMMAND $<TARGET_FILE:Tests> "1")
endif()

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE OpenMVS)

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
