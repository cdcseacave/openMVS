# CMake powered build system for OpenMVS

########################################################################
#
# Project-wide settings
CMAKE_MINIMUM_REQUIRED(VERSION 3.24)
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
OPTION(OpenMVS_USE_SIFTGPU "Enable SIFTGPU library" ON)
OPTION(OpenMVS_USE_FAST_FLOAT2INT "Use an optimized code to convert real numbers to int" OFF)
OPTION(OpenMVS_USE_FAST_INVSQRT "Use an optimized code to compute the inverse square root (slower in fact on modern compilers)" OFF)
OPTION(OpenMVS_USE_FAST_CBRT "Use an optimized code to compute the cubic root" OFF)
OPTION(OpenMVS_USE_SSE "Enable SSE optimizations" ON)
OPTION(OpenMVS_MAX_CUDA_COMPATIBILITY "Build for maximum CUDA device compatibility" OFF)
OPTION(OpenMVS_ENABLE_IPO "Whether to enable interprocedural optimization" ON)
OPTION(OpenMVS_ENABLE_TESTS "Enable test code" ON)
OPTION(OpenMVS_HEADLESS_DEBUG "Print asserts to stderr and continue; do not redirect cout/cerr or pop modal dialogs (for CI / non-interactive debug runs)" OFF)

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

# Add local ports to overlay
SET(VCPKG_OVERLAY_PORTS "${CMAKE_SOURCE_DIR}/ports")

# Define helper functions and macros (must come before the project() call
# below: the opencv4 overlay generator declared in Utils.cmake mutates
# VCPKG_OVERLAY_PORTS, which the vcpkg toolchain consumes during project()).
INCLUDE(build/Utils.cmake)

# On Linux: regenerate a build-tree opencv4 overlay against the upstream port
# at $VCPKG_ROOT and append it to VCPKG_OVERLAY_PORTS. Skipped on
# Windows/macOS — see Utils.cmake for the full rationale.
OpenMVS_GenerateOpencv4Overlay(VCPKG_OVERLAY_PORTS)

IF(OpenMVS_BUILD_TOOLS AND OpenMVS_BUILD_VIEWER)
	LIST(APPEND VCPKG_MANIFEST_FEATURES "viewer")
ENDIF()
IF(OpenMVS_USE_CUDA)
	LIST(APPEND VCPKG_MANIFEST_FEATURES "cuda")
ENDIF()
IF(OpenMVS_USE_SIFTGPU)
	LIST(APPEND VCPKG_MANIFEST_FEATURES "siftgpu")
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
SET(OpenMVS_MINOR_VERSION 4)
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

# Hidden symbol visibility on non-Windows so explicit *_API macros control the export
# surface uniformly across MSVC / GCC / Clang (matches the __declspec(dllexport) model).
SET(CMAKE_CXX_VISIBILITY_PRESET hidden)
SET(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# build/Utils.cmake was already included above (it must run before project()
# so the opencv4 overlay generator can append to VCPKG_OVERLAY_PORTS before
# the vcpkg toolchain consumes it). Among other things, it declares
# OPTION(BUILD_SHARED_LIBS ...), so the OPENMVS_SHARED / IPO logic that
# follows can rely on BUILD_SHARED_LIBS being defined here.
# Tell C++ code we are building shared libraries, so headers (e.g. Types.h) can
# pick the cross-DLL-friendly path for things like boost exception-unwinding.
IF(BUILD_SHARED_LIBS)
	ADD_COMPILE_DEFINITIONS(OPENMVS_SHARED)
	# MSVC's /GL+/LTCG is catastrophic on shared builds: the linker tries to
	# re-do whole-program optimization across each DLL's .obj graph, peaking at
	# 30-40 GB RAM and many minutes per DLL link. Per-DLL native code is fine
	# without IPO; force-disable here so the user doesn't have to remember.
	IF(OpenMVS_ENABLE_IPO)
		MESSAGE(STATUS "Disabling OpenMVS_ENABLE_IPO under BUILD_SHARED_LIBS=ON (IPO under shared builds is memory-pathological).")
	ENDIF()
	SET(OpenMVS_ENABLE_IPO OFF CACHE BOOL "Whether to enable interprocedural optimization" FORCE)
ENDIF()

# Init session with macros defined in Utils.cmake
GetOperatingSystemArchitectureBitness(SYSTEM)
ComposePackageLibSuffix()
ConfigCompilerAndLinker()
ConfigLibrary()

# Co-locate executables and DLLs so apps find dependent shared libraries at runtime
# (Windows resolves DLLs via the executable's directory). Multi-config generators
# automatically append per-config subfolders (e.g. bin/vc18/x64/Release).
# Honor user-supplied -DCMAKE_*_OUTPUT_DIRECTORY=... cache values; only fill in
# the defaults when nothing has been set. Utils.cmake's fix_default_compiler_settings
# already seeds CMAKE_RUNTIME_OUTPUT_DIRECTORY with the same suffix-aware path,
# so the runtime guard normally short-circuits — these defaults exist for the
# library/archive variables which Utils.cmake does not touch.
IF(NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY OR NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin${PACKAGE_LIB_SUFFIX}" CACHE PATH "Runtime output directory")
ENDIF()
IF(NOT DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY OR NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin${PACKAGE_LIB_SUFFIX}" CACHE PATH "Library output directory")
ENDIF()
IF(NOT DEFINED CMAKE_ARCHIVE_OUTPUT_DIRECTORY OR NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib${PACKAGE_LIB_SUFFIX}" CACHE PATH "Archive output directory")
ENDIF()

# Find dependencies:
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/Modules)

# Find required packages
# Note: With modern CMake and target-based propagation, we only collect libraries in OpenMVS_EXTRA_LIBS
# Include directories are now handled via target_include_directories in the Common library
SET(OpenMVS_EXTRA_LIBS "")

if(OpenMVS_ENABLE_IPO)
	INCLUDE(CheckIPOSupported)
	check_ipo_supported(RESULT IPO_SUPPORTED OUTPUT IPO_ERROR)
	if(IPO_SUPPORTED)
		# On MSVC, /GL + /LTCG are injected into the global Release flags by
		# fix_default_compiler_settings() in build/Utils.cmake; do not also set
		# CMAKE_INTERPROCEDURAL_OPTIMIZATION there, otherwise the per-target
		# DISABLE_IPO opt-out (/GL- + /LTCG:OFF) becomes inconsistent.
		# On non-MSVC toolchains we rely on CMake's IPO/LTO mechanism instead.
		if(NOT MSVC)
			SET_PROPERTY(GLOBAL PROPERTY CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
			MESSAGE(STATUS "Interprocedural optimization enabled (CMake IPO/LTO)")
		else()
			MESSAGE(STATUS "Interprocedural optimization enabled (MSVC: /GL + /LTCG in Release)")
		endif()
	else()
		MESSAGE(WARNING "Interprocedural optimization is not supported: ${IPO_ERROR}")
		SET(OpenMVS_ENABLE_IPO OFF CACHE BOOL "Whether to enable interprocedural optimization" FORCE)
	endif()
endif()

if(OpenMVS_USE_OPENMP)
	# MAC OS ARM64 OPENMP FIX: hint Homebrew paths
	if(APPLE AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
		set(OpenMP_ROOT "/opt/homebrew/opt/libomp")
	endif()
	FIND_PACKAGE(OpenMP QUIET)
	if(OPENMP_FOUND)
		SET(_USE_OPENMP TRUE)
		LIST(APPEND OpenMVS_EXTRA_LIBS OpenMP::OpenMP_CXX)
		if(NOT APPLE)
			# Apply OpenMP flags only to C/C++ compilation, not CUDA
			# CUDA will get OpenMP support via linking with OpenMP::OpenMP_CXX
			string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")
			string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
		endif()
		MESSAGE(STATUS "Found OpenMP " ${OpenMP_VERSION} " " ${OpenMP_CXX_INCLUDE_DIRS})
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
			# `native` (CMake >= 3.24) tells nvcc to detect and target the build
			# host's GPU; the project's CMAKE_MINIMUM_REQUIRED guarantees support.
			SET(CMAKE_CUDA_ARCHITECTURES "native")
		endif()
		SET(CMAKE_CUDA_FLAGS_INIT "${CMAKE_CUDA_FLAGS_INIT} -allow-unsupported-compiler")
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
		# Add OpenMP support to CUDA host compiler
		if(_USE_OPENMP)
			SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler ${OpenMP_CXX_FLAGS}")
		endif()
		# Ensure CUDA toolkit include dirs are available for all targets
		INCLUDE_DIRECTORIES(${CUDAToolkit_INCLUDE_DIRS})
		LIST(APPEND OpenMVS_EXTRA_LIBS ${CUDAToolkit_LIBRARIES} CUDA::cuda_driver CUDA::cudart CUDA::curand)
		SET(_USE_CUDA TRUE)
		MESSAGE(STATUS "Found CUDA: " ${CMAKE_CUDA_COMPILER_VERSION} " with GPU arch: " ${CMAKE_CUDA_ARCHITECTURES})
	else()
		MESSAGE(STATUS "Can't find CUDA. Continuing without it.")
	endif()
endif()

if(OpenMVS_USE_BREAKPAD)
	FIND_PACKAGE(BREAKPAD QUIET)
	if(BREAKPAD_FOUND)
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
		LIST(APPEND OpenMVS_EXTRA_LIBS ${Python3_LIBRARIES})
		LIST(APPEND Boost_EXTRA_COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
		MESSAGE(STATUS "Python ${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR} found (include: ${Python3_INCLUDE_DIRS})")
	else()
		MESSAGE(STATUS "Can't find Python. Continuing without it.")
	endif()
endif()

FIND_PACKAGE(Boost REQUIRED COMPONENTS iostreams program_options system serialization exception OPTIONAL_COMPONENTS ${Boost_EXTRA_COMPONENTS})
if(Boost_FOUND)
	ADD_DEFINITIONS(${Boost_DEFINITIONS})
	LIST(APPEND OpenMVS_EXTRA_LIBS ${Boost_LIBRARIES})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
	SET(_USE_BOOST TRUE)
	if(OpenMVS_USE_PYTHON AND Boost_${Boost_EXTRA_COMPONENTS}_FOUND)
		SET(_USE_BOOST_PYTHON TRUE)
	endif()
endif()

FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
if(Eigen3_FOUND)
	if(Eigen3_VERSION VERSION_LESS "3.4")
		MESSAGE(FATAL_ERROR "Eigen ${Eigen3_VERSION} found, but at least 3.4 is required")
	endif()
	LIST(APPEND OpenMVS_EXTRA_LIBS Eigen3::Eigen)
	SET(_USE_EIGEN TRUE)
	MESSAGE(STATUS "Eigen ${Eigen3_VERSION} found")
endif()

FIND_PACKAGE(OpenCV REQUIRED)
if(OpenCV_FOUND)
	ADD_DEFINITIONS(${OpenCV_DEFINITIONS})
	LIST(APPEND OpenMVS_EXTRA_LIBS ${OpenCV_LIBS})
	SET(_USE_OPENCV TRUE)
	MESSAGE(STATUS "OpenCV ${OpenCV_VERSION} found")
else()
	MESSAGE(STATUS "Can't find OpenCV. Please specify OpenCV directory using OpenCV_DIR variable")
endif()

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
if(OpenMVS_HEADLESS_DEBUG)
	SET(_HEADLESS_DEBUG TRUE)
endif()
if(OpenMVS_USE_CERES)
	SET(_USE_CERES TRUE)
endif()

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
	ADD_TEST(NAME CommonUnitTests COMMAND $<TARGET_FILE:Tests> "0")
	ADD_TEST(NAME SFMPipelineTest COMMAND $<TARGET_FILE:Tests> "1")
	ADD_TEST(NAME MVSPipelineTest COMMAND $<TARGET_FILE:Tests> "2")
endif()

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
EXPORT(PACKAGE OpenMVS)

# Install the export set for use with the install-tree
INSTALL(EXPORT OpenMVSTargets
	NAMESPACE OpenMVS::
	DESTINATION "${INSTALL_CMAKE_DIR}")

# Detect git commit hash and modified status at configure time
FIND_PACKAGE(Git QUIET)
SET(OpenMVS_GIT_COMMIT "")
SET(OpenMVS_GIT_MODIFIED 0)
IF(Git_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
	# Get short commit hash
	EXECUTE_PROCESS(
		COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
		WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
		OUTPUT_VARIABLE OpenMVS_GIT_COMMIT
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)
	# Check for uncommitted changes
	EXECUTE_PROCESS(
		COMMAND ${GIT_EXECUTABLE} diff-index --quiet HEAD
		WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
		RESULT_VARIABLE OpenMVS_GIT_RESULT
		ERROR_QUIET
	)
	SET(OpenMVS_GIT_SUFFIX "")
	IF(NOT OpenMVS_GIT_RESULT EQUAL 0)
		SET(OpenMVS_GIT_MODIFIED 1)
		SET(OpenMVS_GIT_SUFFIX "*")
	ENDIF()
	MESSAGE(STATUS "OpenMVS git info: ${OpenMVS_GIT_COMMIT}${OpenMVS_GIT_SUFFIX}")
ENDIF()

# Install configuration file
CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/build/Templates/ConfigLocal.h.in" "${CMAKE_BINARY_DIR}/ConfigLocal.h")
INSTALL(FILES "${CMAKE_BINARY_DIR}/ConfigLocal.h" DESTINATION "${INSTALL_INCLUDE_DIR}")

# Create the OpenMVSConfig.cmake and OpenMVSConfigVersion files
INCLUDE(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_BINARY_DIR}/OpenMVSConfigVersion.cmake"
	VERSION ${OpenMVS_VERSION}
	COMPATIBILITY AnyNewerVersion)
# Note: Include paths are handled via imported targets and target_include_directories
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
