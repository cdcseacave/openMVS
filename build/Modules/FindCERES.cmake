# - try to find CERES headers
#
# Users may optionally supply:
#  CERES_DIR - a prefix to start searching for the toon headers.
#
# Cache Variables: (probably not for direct use in your scripts)
#  CERES_INCLUDE_DIR
#
# Non-cache variables you might use in your CMakeLists.txt:
#  CERES_FOUND
#  CERES_INCLUDE_DIRS
#  CERES_LIBS
#  CERES_DEFINITIONS
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)

#try to use the Config script
if(NOT EXISTS "${CERES_DIR}")
	find_path(CERES_DIR "CeresConfig.cmake"
		HINTS "${CERES_ROOT}" "$ENV{CERES_ROOT}" "$ENV{CERES_DIR}" "$ENV{CERES_ROOT}/CMake"
		PATHS "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}" "/usr" "/usr/local" "/usr/share" "/usr/local/share" "/usr/lib/cmake" "/usr/local/lib/cmake" "/usr/include" "/usr/lib/x86_64-linux-gnu/cmake"
		PATH_SUFFIXES "Ceres"
		DOC "Root directory of CERES")
endif()

set(CERES_VERSION "")
if(EXISTS "${CERES_DIR}")

 	## Include the standard CMake script
	include("${CERES_DIR}/CeresConfig.cmake")
	set(CERES_LIBS ${CERES_LIBS} ${CERES_LIBRARIES})

else()

	# Find required packages
	FIND_PACKAGE(Eigen ${SYSTEM_PACKAGE_QUIET})
	FIND_PACKAGE(SUITESPARSE ${SYSTEM_PACKAGE_QUIET})
	if(SUITESPARSE_FOUND)
		set(CERES_LIBS ${CERES_LIBS} ${SUITESPARSE_LIBS})
	endif()
	FIND_PACKAGE(GLOG ${SYSTEM_PACKAGE_QUIET})
	if(GLOG_FOUND)
		set(CERES_INCLUDE_DIRS ${CERES_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
		set(CERES_LIBS ${CERES_LIBS} ${GLOG_LIBS})
	endif()

	if(NOT CERES_DIR OR "${CERES_DIR}" STREQUAL "")
		set(CERES_DIR "$ENV{CERES_ROOT}")
	endif()
	set(CERES_DIR "${CERES_DIR}" CACHE PATH "Root directory of CERES library")

 	#try to guess path
	find_path(CERES_INCLUDE_DIR
		NAMES "ceres/ceres.h"
		HINTS "${CERES_DIR}" "$ENV{CERES_ROOT}" "/usr" "/usr/local"
		PATH_SUFFIXES "include")

	set(CERES_FOUND FALSE)
	if(EXISTS "${CERES_INCLUDE_DIR}" AND NOT "${CERES_INCLUDE_DIR}" STREQUAL "")
		set(CERES_FOUND TRUE)
		
		find_library(CERES_LIBRARY_DEBUG "libceres" "ceres" "ceres_shared" PATHS "${CERES_DIR}/lib${PACKAGE_LIB_SUFFIX_DBG}" "$ENV{OpenCV_ROOT}/lib${PACKAGE_LIB_SUFFIX_DBG}" NO_DEFAULT_PATH)
		find_library(CERES_LIBRARY_RELEASE "libceres" "ceres" "ceres_shared" PATHS "${CERES_DIR}/lib${PACKAGE_LIB_SUFFIX_REL}" "$ENV{OpenCV_ROOT}/lib${PACKAGE_LIB_SUFFIX_REL}" NO_DEFAULT_PATH)
		find_library(CERES_LIBRARY_ALL NAMES "ceres" PATH_SUFFIXES "ceres")
		
		#Remove the cache value
		set(CERES_LIBRARY "" CACHE STRING "" FORCE)
		
		#both debug/release
		if(CERES_LIBRARY_DEBUG AND CERES_LIBRARY_RELEASE)
			set(CERES_LIBRARY debug ${CERES_LIBRARY_DEBUG} optimized ${CERES_LIBRARY_RELEASE} CACHE STRING "" FORCE)
		#only debug
		elseif(CERES_LIBRARY_DEBUG)
			set(CERES_LIBRARY ${CERES_LIBRARY_DEBUG} CACHE STRING "" FORCE)
		#only release
		elseif(CERES_LIBRARY_RELEASE)
			set(CERES_LIBRARY ${CERES_LIBRARY_RELEASE} CACHE STRING "" FORCE)
		#both debug/release
		elseif(CERES_LIBRARY_ALL)
			set(CERES_LIBRARY ${CERES_LIBRARY_ALL} CACHE STRING "" FORCE)
		#no library found
		else()
			message("CERES library NOT found")
			set(CERES_FOUND FALSE)
		endif()
		
		#Add to the general list
		if(CERES_LIBRARY)
			set(CERES_LIBS ${CERES_LIBS} ${CERES_LIBRARY})
		endif()
	endif()

endif()

if(CERES_FOUND)
	set(CERES_INCLUDE_DIRS ${CERES_INCLUDE_DIRS} "${CERES_INCLUDE_DIR}")
	set(CERES_DIR "${CERES_DIR}" CACHE PATH "" FORCE)
	mark_as_advanced(CERES_DIR)
	message(STATUS "CERES ${CERES_VERSION} found (include: ${CERES_INCLUDE_DIRS})")
else()
	package_report_not_found(CERES "Please specify CERES directory using CERES_ROOT env. variable")
endif()
