# Macro to check pkg-config modules and produce a full-path library list
macro(pkg_check_modules_fullpath_libs PREFIX MODULE_NAME)
	# Call pkg_check_modules
	pkg_check_modules(${PREFIX} IMPORTED_TARGET ${MODULE_NAME})
	if(${PREFIX}_FOUND)
		set(${PREFIX}_FULLPATH_LIBRARIES "" PARENT_SCOPE)
		foreach(lib ${${PREFIX}_LIBRARIES})
			set(lib_path "")
			set(lib_prefix "")
			set(lib_ext "")
			if(WIN32)
				set(lib_ext ".lib")
			else()
				set(lib_prefix "lib")
				set(lib_ext ".a")
			endif()
			# Skip system libraries like 'm' on Windows
			if(WIN32 AND (lib STREQUAL "m"))
				continue()
			endif()
			# If lib already has a path, use as is
			if(EXISTS "${lib_prefix}${lib}${lib_ext}")
				set(lib_path "${lib_prefix}${lib}${lib_ext}")
			else()
				foreach(dir ${${PREFIX}_LIBRARY_DIRS})
					if(EXISTS "${dir}/${lib_prefix}${lib}${lib_ext}")
						set(lib_path "${dir}/${lib_prefix}${lib}${lib_ext}")
						break()
					endif()
				endforeach()
			endif()
			if(NOT lib_path STREQUAL "")
				list(APPEND ${PREFIX}_FULLPATH_LIBRARIES "${lib_path}")
			else()
				list(APPEND ${PREFIX}_FULLPATH_LIBRARIES "${lib}")
			endif()
		endforeach()
		set(${PREFIX}_FULLPATH_LIBRARIES "${${PREFIX}_FULLPATH_LIBRARIES}" PARENT_SCOPE)
		message(STATUS "Found ${MODULE_NAME}: ${${PREFIX}_VERSION} libs: ${${PREFIX}_FULLPATH_LIBRARIES}")
	else()
		set(${PREFIX}_FULLPATH_LIBRARIES "" PARENT_SCOPE)
		message(STATUS "${MODULE_NAME} not found, support will be disabled")
	endif()
endmacro()

# Find required packages
FIND_PACKAGE(PNG QUIET)
if(PNG_FOUND)
	INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIRS})
	ADD_DEFINITIONS(${PNG_DEFINITIONS})
	SET(_USE_PNG TRUE CACHE INTERNAL "")
else()
	SET(PNG_LIBRARIES "")
endif()
FIND_PACKAGE(JPEG QUIET)
if(JPEG_FOUND)
	INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
	ADD_DEFINITIONS(${JPEG_DEFINITIONS})
	SET(_USE_JPG TRUE CACHE INTERNAL "")
else()
	SET(JPEG_LIBRARIES "")
endif()
FIND_PACKAGE(PkgConfig QUIET)
if(PkgConfig_FOUND)
	pkg_check_modules_fullpath_libs(JPEGXL libjxl)
endif()
if(JPEGXL_FOUND)
	SET(_USE_JXL TRUE CACHE INTERNAL "")
	set(JPEGXL_LIBRARIES ${JPEGXL_FULLPATH_LIBRARIES})
else()
	SET(JPEGXL_LIBRARIES "")
endif()
FIND_PACKAGE(TIFF QUIET)
if(TIFF_FOUND)
	INCLUDE_DIRECTORIES(${TIFF_INCLUDE_DIR})
	ADD_DEFINITIONS(${TIFF_DEFINITIONS})
	SET(_USE_TIFF TRUE CACHE INTERNAL "")
else()
	SET(TIFF_LIBRARIES "")
endif()

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_library_with_type(IO "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(IO PRIVATE "Common.h")
endif()

# Link its dependencies
TARGET_LINK_LIBRARIES(IO PUBLIC Common ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${JPEGXL_LIBRARIES} ${TIFF_LIBRARIES} ${EXIV2_LIBS})

# Install
SET_TARGET_PROPERTIES(IO PROPERTIES
	PUBLIC_HEADER "${LIBRARY_FILES_H}")
INSTALL(TARGETS IO
	EXPORT OpenMVSTargets
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/IO")
