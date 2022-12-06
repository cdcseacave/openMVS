# Find required packages
FIND_PACKAGE(CGAL REQUIRED)
if(CGAL_FOUND)
	include_directories(${CGAL_INCLUDE_DIRS})
	add_definitions(${CGAL_DEFINITIONS})
	link_directories(${CGAL_LIBRARY_DIRS})
endif()

FIND_PACKAGE(VCG REQUIRED)
if(VCG_FOUND)
	include_directories(${VCG_INCLUDE_DIRS})
	add_definitions(${VCG_DEFINITIONS})
endif()

set(CERES_LIBS "")
if(OpenMVS_USE_CERES)
	FIND_PACKAGE(Ceres)
	if(CERES_FOUND)
		include_directories(${CERES_INCLUDE_DIRS})
		add_definitions(${CERES_DEFINITIONS})
	else()
		set(OpenMVS_USE_CERES OFF)
		message("-- Can't find CERES. Continuing without it.")
	endif()
endif()

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")
if(_USE_CUDA)
	FILE(GLOB LIBRARY_FILES_CUDA "*.cu")
	LIST(APPEND LIBRARY_FILES_C ${LIBRARY_FILES_CUDA})
endif()

GET_FILENAME_COMPONENT(PATH_PythonWrapper_cpp ${CMAKE_CURRENT_SOURCE_DIR}/PythonWrapper.cpp ABSOLUTE)
LIST(REMOVE_ITEM LIBRARY_FILES_C "${PATH_PythonWrapper_cpp}")

cxx_library_with_type(MVS "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(MVS PRIVATE "Common.h")
endif()

# Link its dependencies
if(_USE_CUDA)
	SET_TARGET_PROPERTIES(MVS PROPERTIES CUDA_ARCHITECTURES "50;72;75")
endif()
TARGET_LINK_LIBRARIES(MVS PRIVATE Common Math IO CGAL::CGAL ${CERES_LIBRARIES} ${CUDA_CUDA_LIBRARY})

if(OpenMVS_USE_PYTHON)
	# Create the Python wrapper
	cxx_library_with_type(pyOpenMVS "Libs" "SHARED" "${cxx_default}"
		${PATH_PythonWrapper_cpp}
	)
	# Link its dependencies
	if(_USE_CUDA)
		SET_TARGET_PROPERTIES(pyOpenMVS PROPERTIES CUDA_ARCHITECTURES "50;72;75")
	endif()
	TARGET_LINK_LIBRARIES(pyOpenMVS PRIVATE MVS ${OpenMVS_EXTRA_LIBS})
	# Suppress prefix "lib" because Python does not allow this prefix
	SET_TARGET_PROPERTIES(pyOpenMVS PROPERTIES PREFIX "")
	# Install
	INSTALL(TARGETS pyOpenMVS DESTINATION "${PYTHON_INSTALL_PATH}")
endif()

# Install
SET_TARGET_PROPERTIES(MVS PROPERTIES
	PUBLIC_HEADER "${LIBRARY_FILES_H}")
INSTALL(TARGETS MVS
	EXPORT OpenMVSTargets
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/MVS")
