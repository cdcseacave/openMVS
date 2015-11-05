# Find required packages
FIND_PACKAGE(PNG QUIET)
if(PNG_FOUND)
	include_directories(${PNG_INCLUDE_DIRS})
	add_definitions(${PNG_DEFINITIONS} -D_USE_PNG)
endif()
FIND_PACKAGE(JPEG QUIET)
if(JPEG_FOUND)
	include_directories(${JPEG_INCLUDE_DIR})
	add_definitions(${JPEG_DEFINITIONS} -D_USE_JPG)
endif()
FIND_PACKAGE(TIFF QUIET)
if(TIFF_FOUND)
	include_directories(${TIFF_INCLUDE_DIR})
	add_definitions(${TIFF_DEFINITIONS} -D_USE_TIFF)
endif()
if(OpenMVS_USE_EXIV2)
	FIND_PACKAGE(EXIV2 QUIET)
	if(EXIV2_FOUND)
		include_directories(${EXIV2_INCLUDE_DIRS})
	endif()
endif()

# List sources files
FILE(GLOB PCH_C "Common.cpp")

FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# Place Common.cpp as the first file in the list
# needed by cotire when setting PCH manually
LIST(REMOVE_ITEM LIBRARY_FILES_C ${PCH_C})
SET(LIBRARY_FILES_C "${PCH_C};${LIBRARY_FILES_C}")

cxx_library_with_type_no_pch(IO "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
set_target_pch(IO Common.h)

# Link its dependencies
TARGET_LINK_LIBRARIES(IO Common ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${TIFF_LIBRARIES} ${EXIV2_LIBS})

# Install
INSTALL(FILES ${LIBRARY_FILES_H} DESTINATION "include/${PROJECT_NAME}/IO")
INSTALL(TARGETS IO DESTINATION "lib/${PROJECT_NAME}")
