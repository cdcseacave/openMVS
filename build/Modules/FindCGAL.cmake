###########################################################
#                  Find CGAL Library
#----------------------------------------------------------
#  CGAL_FOUND            - True if headers and requested libraries were found
#  CGAL_INCLUDE_DIRS     - CGAL include directories
#  CGAL_LIBRARY_DIRS     - Link directories for CGAL libraries
#----------------------------------------------------------

find_path(CGAL_DIR "include/CGAL/config.h"
    HINTS "${CGAL_ROOT}" "$ENV{CGAL_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/CGAL" "$ENV{PROGRAMW6432}/CGAL" "/usr" "/usr/local"
    PATH_SUFFIXES ""
    DOC "Root directory of CGAL library")

##====================================================
## Include CGAL library
##----------------------------------------------------
if(EXISTS "${CGAL_DIR}" AND NOT "${CGAL_DIR}" STREQUAL "")
	set(CGAL_FOUND TRUE)
	set(CGAL_INCLUDE_DIRS "${CGAL_DIR}/include" "${CGAL_DIR}/auxiliary/gmp/include")
	set(CGAL_LIBRARY_DIRS "${CGAL_DIR}/lib${PACKAGE_LIB_SUFFIX}")
	set(CGAL_DIR "${CGAL_DIR}" CACHE PATH "" FORCE)
	mark_as_advanced(CGAL_DIR)
	message(STATUS "CGAL found (include: ${CGAL_INCLUDE_DIRS})")
else()
	package_report_not_found(CGAL "Please specify CGAL directory using CGAL_ROOT env. variable")
endif()
##====================================================
