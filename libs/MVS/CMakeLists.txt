# Find required packages
FIND_PACKAGE(CGAL ${SYSTEM_PACKAGE_REQUIRED})
if(CGAL_FOUND)
	include_directories(${CGAL_INCLUDE_DIRS})
	add_definitions(${CGAL_DEFINITIONS})
	link_directories(${CGAL_LIBRARY_DIRS})
endif()

FIND_PACKAGE(VCG ${SYSTEM_PACKAGE_REQUIRED})
if(VCG_FOUND)
	include_directories(${VCG_INCLUDE_DIRS})
	add_definitions(${VCG_DEFINITIONS})
endif()

FIND_PACKAGE(CERES ${SYSTEM_PACKAGE_REQUIRED})
if(CERES_FOUND)
	include_directories(${CERES_INCLUDE_DIRS})
	add_definitions(${CERES_DEFINITIONS})
endif()

# List sources files
FILE(GLOB PCH_C "Common.cpp")

FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# Place Common.cpp as the first file in the list
# needed by cotire when setting PCH manually
LIST(REMOVE_ITEM LIBRARY_FILES_C ${PCH_C})
SET(LIBRARY_FILES_C "${PCH_C};${LIBRARY_FILES_C}")

cxx_library_with_type_no_pch(MVS "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
set_target_pch(MVS Common.h)

# Link its dependencies
TARGET_LINK_LIBRARIES(MVS Common Math IO ${CERES_LIBS} ${CGAL_LIBS} ${CUDA_LIBRARIES})

# Install
INSTALL(FILES ${LIBRARY_FILES_H} DESTINATION "include/${PROJECT_NAME}/MVS")
INSTALL(TARGETS MVS DESTINATION "lib/${PROJECT_NAME}")
