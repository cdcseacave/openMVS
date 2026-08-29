# Find required packages are inherited from parent CMakeLists.txt

# Additional Math-specific packages (all available via vcpkg)
FIND_PACKAGE(SuiteSparse_config QUIET)
FIND_PACKAGE(CHOLMOD QUIET)  # Required by SuiteSparse_config but explicitly needed for linking

# Check which Math features are available
SET(MATH_EXTRA_LIBS "")
IF(SuiteSparse_config_FOUND AND CHOLMOD_FOUND)
	MESSAGE(STATUS "SuiteSparse found via CMake config")
	ADD_DEFINITIONS(-D_USE_SUITESPARSE)
	LIST(APPEND MATH_EXTRA_LIBS SuiteSparse::CHOLMOD SuiteSparse::SuiteSparseConfig)
ELSE()
	MESSAGE(STATUS "Can't find SuiteSparse. Continuing without it.")
ENDIF()

INCLUDE_DIRECTORIES(".")

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

FILE(GLOB LMFit_LIBRARY_FILES_C "LMFit/*.cpp")
FILE(GLOB LMFit_LIBRARY_FILES_H "LMFit/*.h" "LMFit/*.inl")
SOURCE_GROUP("LMFit" FILES ${LMFit_LIBRARY_FILES_C} ${LMFit_LIBRARY_FILES_H})

cxx_library_with_type(Math "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
	${LMFit_LIBRARY_FILES_C} ${LMFit_LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(Math PRIVATE "Common.h")
endif()

# Link its dependencies
TARGET_LINK_LIBRARIES(Math PUBLIC Common ${MATH_EXTRA_LIBS})

# Install
INSTALL(FILES ${LIBRARY_FILES_H} DESTINATION "${INSTALL_INCLUDE_DIR}/Math")
INSTALL(FILES ${LMFit_LIBRARY_FILES_H} DESTINATION "${INSTALL_INCLUDE_DIR}/Math/LMFit")
INSTALL(TARGETS Math
	EXPORT OpenMVSTargets
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/Math")
