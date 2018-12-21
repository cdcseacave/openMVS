###########################################################
#                  Find CGAL Library
#----------------------------------------------------------
#  CGAL_FOUND            - True if headers and requested libraries were found
#  CGAL_INCLUDE_DIRS     - CGAL include directories
#  CGAL_LIBRARY_DIRS     - Link directories for CGAL libraries
#  CGAL_LIBS             - CGAL libraries
#  CGAL_VERSION          - MAJOR.MINOR
#----------------------------------------------------------

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

if(NOT CGAL_DIR)
  # Get the system search path as a list.
  if(UNIX)
    string(REGEX MATCHALL "[^:]+" CGAL_DIR_SEARCH1 "$ENV{PATH}")
  else()
    string(REGEX REPLACE "\\\\" "/" CGAL_DIR_SEARCH1 "$ENV{PATH}")
  endif()
  string(REGEX REPLACE "/;" ";" CGAL_DIR_SEARCH2 "${CGAL_DIR_SEARCH1}")
  # Construct a set of paths relative to the system search path.
  set(CGAL_DIR_SEARCH "")
  foreach(dir ${CGAL_DIR_SEARCH2})
    set(CGAL_DIR_SEARCH ${CGAL_DIR_SEARCH} ${dir}/../lib/CGAL)
  endforeach()
  set(CGAL_DIR_SEARCH ${CGAL_DIR_SEARCH} "lib" "lib64")

  #
  # Look for an installation or build tree.
  #
  find_path(CGAL_DIR "CGALConfig.cmake"
    # Look for an environment variable CGAL_DIR.
    HINTS "${CGAL_ROOT}" "$ENV{CGAL_ROOT}" "$ENV{CGAL_DIR}" "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}"

    # Look in places relative to the system executable search path.
    ${CGAL_DIR_SEARCH}

    # Look in standard UNIX install locations.
    PATHS "/usr" "/usr/local" "/usr/share" "/usr/local/share" "/usr/lib/cmake" "/usr/local/lib/cmake" "/usr/include" "/usr/lib/x86_64-linux-gnu/cmake"

    # Read from the CMakeSetup registry entries.  It is likely that
    # CGAL will have been recently built.
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild1]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild2]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild3]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild4]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild5]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild6]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild7]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild8]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild9]
    [HKEY_CURRENT_USER\\Software\\Kitware\\CMakeSetup\\Settings\\StartPath;WhereBuild10]
	
	PATH_SUFFIXES "CGAL" "share" "share/cgal" "share/cmake" "share/cmake/cgal"
	
	DOC "Root directory of CGAL library"
  )
endif()

##====================================================
## Include CGAL library
##----------------------------------------------------
set(CGAL_VERSION "")
if(EXISTS "${CGAL_DIR}" AND NOT "${CGAL_DIR}" STREQUAL "")
	if(EXISTS "${CGAL_DIR}/CGALConfig.cmake")
		include("${CGAL_DIR}/CGALConfig.cmake")
		set(CGAL_LIBS ${CGAL_LIBS} ${CGAL_LIBRARIES} ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY} ${CGAL_ImageIO_LIBRARY} ${CGAL_3RD_PARTY_LIBRARIES} ${CGAL_Core_3RD_PARTY_LIBRARIES} ${CGAL_ImageIO_3RD_PARTY_LIBRARIES} ${MPFR_LIBRARIES} ${GMP_LIBRARIES} ${ZLIB_LIBRARIES})
		set(CGAL_VERSION "${CGAL_MAJOR_VERSION}.${CGAL_MINOR_VERSION}")
	else()
		set(CGAL_INCLUDE_DIRS "${CGAL_DIR}/include" "${CGAL_DIR}/auxiliary/gmp/include")
		set(CGAL_LIBRARY_DIRS "${CGAL_DIR}/lib${PACKAGE_LIB_SUFFIX}")
	endif()
	set(CGAL_FOUND TRUE)
	set(CGAL_DIR "${CGAL_DIR}" CACHE PATH "" FORCE)
	mark_as_advanced(CGAL_DIR)
	message(STATUS "CGAL ${CGAL_VERSION} found (include: ${CGAL_INCLUDE_DIRS})")
else()
	package_report_not_found(CGAL "Please specify CGAL directory using CGAL_ROOT env. variable")
endif()
##====================================================
