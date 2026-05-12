cmake_minimum_required(VERSION 3.24)
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)  # prefer GLVND
endif()

# ---- Options ----
option(CUDA_ENABLED "Build CUDA-accelerated parts" ON)
option(SIMD_ENABLED "Enable SIMD optimizations (SSE) on x86" ON)
option(DEVIL_ENABLED "Enable DevIL image library support" OFF)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Disable CUDA on MacOS
IF(APPLE)
	SET(CUDA_ENABLED OFF)
	MESSAGE(STATUS "Disabling CUDA on MacOS")
ENDIF()
IF(CUDA_ENABLED)
	LIST(APPEND VCPKG_MANIFEST_FEATURES "cuda")
ENDIF()

project(siftgpu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
set(COMPILE_DEFINITIONS SIFTGPU_AVAILABLE=1)
set(OPTIONAL_LINK_LIBS)
find_package(OpenGL REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG QUIET)
if(glfw3_FOUND)
  list(APPEND OPTIONAL_LINK_LIBS glfw)
  message(STATUS "GLFW found: ${glfw3_VERSION}")
else()
  message(STATUS "GLFW not found, continuing without GLFW support")
endif()
if(NOT APPLE)
  set(EGL_FOUND_VIA_PKGCONFIG OFF)
  find_package(EGL CONFIG QUIET)
  if(NOT EGL_FOUND)
    # Try pkg-config if EGL CONFIG module not found
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
      pkg_check_modules(EGL QUIET egl)
      if(EGL_FOUND)
        set(EGL_FOUND_VIA_PKGCONFIG ON)
        if(NOT TARGET EGL::EGL)
          add_library(EGL::EGL INTERFACE IMPORTED)
          target_include_directories(EGL::EGL INTERFACE ${EGL_INCLUDE_DIRS})
          target_link_directories(EGL::EGL INTERFACE ${EGL_LIBRARY_DIRS})
          target_link_libraries(EGL::EGL INTERFACE ${EGL_LIBRARIES})
        endif()
      endif()
    endif()
  endif()
  if(EGL_FOUND)
    list(APPEND COMPILE_DEFINITIONS SIFTGPU_EGL=1)
    list(APPEND OPTIONAL_LINK_LIBS EGL::EGL)
    message(STATUS "EGL found: ${EGL_INCLUDE_DIRS}")
  else()
    message(STATUS "EGL not found, continuing without EGL support")
  endif()
endif()

if(CUDA_ENABLED)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES "native")
  endif()
  set(CMAKE_CUDA_FLAGS_INIT "${CMAKE_CUDA_FLAGS_INIT} -allow-unsupported-compiler")
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  list(APPEND COMPILE_DEFINITIONS SIFTGPU_CUDA=1 CUDA_SIFTGPU_ENABLED)
  list(APPEND OPTIONAL_LINK_LIBS CUDA::cudart CUDA::curand)
  message(STATUS "CUDA found: ${CUDAToolkit_VERSION}")
endif()

# DevIL (optional)
if(DEVIL_ENABLED)
  find_package(DevIL REQUIRED)
  list(APPEND COMPILE_DEFINITIONS SIFTGPU_DEVIL=1)
  list(APPEND OPTIONAL_LINK_LIBS IL::IL)
  message(STATUS "DevIL found")
endif()

# ---- SIMD detection ----
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _proc_lc)
if(_proc_lc MATCHES "x86|amd64|i.86")
  set(IS_X86 TRUE)
else()
  set(IS_X86 FALSE)
endif()

# ---- Sources ----
set(SIFTGPU_CORE_SRCS
  src/FrameBufferObject.h src/FrameBufferObject.cpp
  src/GlobalUtil.h        src/GlobalUtil.cpp
  src/LiteWindow.h        src/LiteWindowGLFW.cpp src/LiteWindowEGL.cpp
  src/GLTexImage.h        src/GLTexImage.cpp
  src/ProgramGLSL.h       src/ProgramGLSL.cpp
  src/ProgramGPU.h
  src/PyramidGL.h         src/PyramidGL.cpp
  src/ShaderMan.h         src/ShaderMan.cpp
  src/SiftGPU.h           src/SiftGPU.cpp
  src/SiftMatch.h         src/SiftMatch.cpp
  src/SiftPyramid.h       src/SiftPyramid.cpp
)

set(OPTIONAL_CUDA_SRCS)
if(CUDA_ENABLED)
  list(APPEND OPTIONAL_CUDA_SRCS
    src/CuTexImage.h  src/CuTexImage.cpp
    src/ProgramCU.cu  src/ProgramCU.h
    src/PyramidCU.h   src/PyramidCU.cpp
    src/SiftMatchCU.h src/SiftMatchCU.cpp
  )
endif()

# ---- Library target ----
add_library(siftgpu
  ${SIFTGPU_CORE_SRCS}
  ${OPTIONAL_CUDA_SRCS}
)

# Enable position-independent code (PIC) for the siftgpu target
# This allows the compiled object files to be used in both static and shared libraries
# Required for building shared libraries on position-independent execution (PIE) systems
# Equivalent to compiling with -fPIC flag on GCC/Clang
set_target_properties(siftgpu PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(siftgpu
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(siftgpu PUBLIC ${COMPILE_DEFINITIONS})

# Add DLL export/import definitions for Windows
if(WIN32 AND BUILD_SHARED_LIBS)
  target_compile_definitions(siftgpu
    PUBLIC SIFTGPU_DLL
    PRIVATE DLL_EXPORT
  )
endif()

if(SIMD_ENABLED AND IS_X86)
  target_compile_definitions(siftgpu PRIVATE USE_SSE_FOR_SIFTGPU)
  if(MSVC)
    target_compile_definitions(siftgpu PRIVATE __SSE__)
  endif()
endif()

# ---- Linking ----
target_link_libraries(siftgpu
  PUBLIC
    OpenGL::GL
    glad::glad
  PRIVATE
    ${OPTIONAL_LINK_LIBS}
)

# ---- Install rules (same as before) ----
include(GNUInstallDirs)

install(TARGETS siftgpu
  EXPORT siftgpuTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/siftgpu
  FILES_MATCHING PATTERN "*.h"
)

install(EXPORT siftgpuTargets
  FILE siftgpuTargets.cmake
  NAMESPACE siftgpu::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/siftgpu
)

message(STATUS "Using package config template: ${CMAKE_CURRENT_SOURCE_DIR}/cmake/siftgpuConfig.cmake.in")

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/siftgpuConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/siftgpuConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/siftgpu
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/siftgpuConfig.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/siftgpu
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" FILES
  ${SIFTGPU_CORE_SRCS} ${OPTIONAL_CUDA_SRCS}
)

