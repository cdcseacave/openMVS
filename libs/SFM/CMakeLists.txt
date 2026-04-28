# Find required packages are inherited from parent CMakeLists.txt

# Additional SfM-specific packages (all available via vcpkg)
FIND_PACKAGE(TinyEXIF CONFIG REQUIRED)
FIND_PACKAGE(TinyNPY CONFIG REQUIRED)
FIND_PACKAGE(PoseLib CONFIG REQUIRED)
FIND_PACKAGE(Ceres CONFIG REQUIRED)

# Check which SfM features are available
SET(SFM_EXTRA_LIBS "")
IF(TinyEXIF_FOUND)
	MESSAGE(STATUS "TinyEXIF found via CMake config")
	LIST(APPEND SFM_EXTRA_LIBS TinyEXIF::TinyEXIF)
ELSE()
	MESSAGE(FATAL_ERROR "TinyEXIF not found. Install tinyexif via vcpkg (it's listed in vcpkg.json).")
ENDIF()

IF(TinyNPY_FOUND)
	MESSAGE(STATUS "TinyNPY found via CMake config")
	LIST(APPEND SFM_EXTRA_LIBS TinyNPY::TinyNPY)
ELSE()
	MESSAGE(FATAL_ERROR "TinyNPY not found. Install tinynpy via vcpkg (it's listed in vcpkg.json).")
ENDIF()

IF(PoseLib_FOUND)
	MESSAGE(STATUS "PoseLib found - modern pose estimation enabled")
	ADD_DEFINITIONS(-DSFM_USE_POSELIB)
	LIST(APPEND SFM_EXTRA_LIBS PoseLib::PoseLib)
ELSE()
	MESSAGE(WARNING "PoseLib not found - pose estimation will use fallback methods")
ENDIF()

IF(Ceres_FOUND)
	MESSAGE(STATUS "Ceres Solver found - bundle adjustment enabled")
	ADD_DEFINITIONS(-DSFM_USE_CERES)
	LIST(APPEND SFM_EXTRA_LIBS Ceres::ceres)
ELSE()
	MESSAGE(WARNING "Ceres Solver not found - bundle adjustment disabled")
ENDIF()

# Optional SiftGPU support via vcpkg manifest feature
if(OpenMVS_USE_SIFTGPU)
	FIND_PACKAGE(siftgpu CONFIG QUIET)
	if(siftgpu_FOUND)
		ADD_DEFINITIONS(-D_USE_SIFTGPU)
		LIST(APPEND SFM_EXTRA_LIBS siftgpu::siftgpu)
		message(STATUS "SiftGPU found and linked")
	else()
		message(STATUS "Can't find SiftGPU. Continuing without it.")
	endif()
endif()

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# PythonWrapper.cpp is compiled into the pyOpenMVS extension module by libs/MVS/CMakeLists.txt;
# exclude it from the SFM library so it isn't built twice.
GET_FILENAME_COMPONENT(PATH_SFMPythonWrapper_cpp ${CMAKE_CURRENT_SOURCE_DIR}/PythonWrapper.cpp ABSOLUTE)
LIST(REMOVE_ITEM LIBRARY_FILES_C "${PATH_SFMPythonWrapper_cpp}")

cxx_library_with_type(SFM "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(SFM PRIVATE "Common.h")
endif()

# Link its dependencies
TARGET_LINK_LIBRARIES(SFM PUBLIC Common Math IO ${SFM_EXTRA_LIBS})

# Install
SET_TARGET_PROPERTIES(SFM PROPERTIES
	PUBLIC_HEADER "${LIBRARY_FILES_H}")
INSTALL(TARGETS SFM
	EXPORT OpenMVSTargets
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/SFM")

