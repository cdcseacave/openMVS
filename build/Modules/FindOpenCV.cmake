###########################################################
#                  Find OpenCV Library
# See http://sourceforge.net/projects/opencvlibrary/
#----------------------------------------------------------
#
## 1: Setup:
# The following variables are optionally searched for defaults
#  OpenCV_DIR:            Base directory of OpenCv tree to use.
#
## 2: Variable
# The following are set after configuration is done: 
#  
#  OpenCV_FOUND
#  OpenCV_INCLUDE_DIRS
#  OpenCV_LIBS
#  OpenCV_DEFINITIONS
#  OpenCV_VERSION (OpenCV_VERSION_MAJOR, OpenCV_VERSION_MINOR, OpenCV_VERSION_PATCH)
#
#----------------------------------------------------------

##====================================================
## Find OpenCV libraries
##----------------------------------------------------
#try to use the Config script
if(NOT EXISTS "${OpenCV_DIR}")
	find_path(OpenCV_DIR "OpenCVConfig.cmake"
		HINTS "${OpenCV_ROOT}" "$ENV{OpenCV_ROOT}" "$ENV{OpenCV_DIR}"
		PATHS "CMake" "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}" "/usr" "/usr/local" "/usr/share" "/usr/local/share" "/usr/lib/cmake" "/usr/local/lib/cmake"
		PATH_SUFFIXES "OpenCV" "include"
		DOC "Root directory of OpenCV")
endif()

set(OpenCV_VERSION "")
if(EXISTS "${OpenCV_DIR}")

 	## Include the standard CMake script
	include("${OpenCV_DIR}/OpenCVConfig.cmake")

	## Search for a specific version
	set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

else()

 	#try to guess it
	find_path(OpenCV_INCLUDE_DIR "opencv.hpp"
		HINTS "${OpenCV_ROOT}" "$ENV{OpenCV_ROOT}"
		PATHS "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}" "/usr" "/usr/local"
		PATH_SUFFIXES "include" "include/opencv" "include/opencv2"
		DOC "Include directory of OpenCV")
 	if(EXISTS "${OpenCV_INCLUDE_DIR}")
		set(OpenCV_DIR "${OpenCV_INCLUDE_DIR}")
		set(OpenCV_LIB_COMPONENTS core features2d calib3d flann legacy contrib imgproc highgui)

		#Find OpenCV version by looking at version.hpp
		file(STRINGS ${OpenCV_INCLUDE_DIR}/core/version.hpp OpenCV_VERSIONS_TMP REGEX "^#define CV_VERSION_[A-Z]+[ \t]+[0-9]+$")
		string(REGEX REPLACE ".*#define CV_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MAJOR ${OpenCV_VERSIONS_TMP})
		string(REGEX REPLACE ".*#define CV_VERSION_MINOR[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MINOR ${OpenCV_VERSIONS_TMP})
		string(REGEX REPLACE ".*#define CV_VERSION_REVISION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_PATCH ${OpenCV_VERSIONS_TMP})
		set(OpenCV_VERSION ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH} CACHE STRING "" FORCE)
		set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")
		set(OpenCV_VERSION "${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH}")
	endif()

endif()

if(EXISTS "${OpenCV_DIR}")
	## Initiate the variable before the loop
	set(OpenCV_FOUND TRUE)
	set(OpenCV_LIBS "")

	## Loop over each component and add its dependencies also
	set(OpenCV_LIB_COMPONENTS_ALL "${OpenCV_LIB_COMPONENTS}")
	set(OpenCV_LIB_COMPONENTS_EXTRA "")
	foreach(__CVLIB ${OpenCV_LIB_COMPONENTS})

		# append internal components
		list(APPEND OpenCV_LIB_COMPONENTS_ALL "${OpenCV_${__CVLIB}_DEPS_DBG}")
		list(APPEND OpenCV_LIB_COMPONENTS_ALL "${OpenCV_${__CVLIB}_DEPS_OPT}")
			
		# append external components
		list(APPEND OpenCV_LIB_COMPONENTS_EXTRA "${OpenCV_${__CVLIB}_EXTRA_DEPS_DBG}")
		list(APPEND OpenCV_LIB_COMPONENTS_EXTRA "${OpenCV_${__CVLIB}_EXTRA_DEPS_OPT}")

	endforeach()
	list(REMOVE_DUPLICATES OpenCV_LIB_COMPONENTS_ALL)
	list(REMOVE_DUPLICATES OpenCV_LIB_COMPONENTS_EXTRA)

	## Loop over each internal component and find its library file
	foreach(__CVLIB ${OpenCV_LIB_COMPONENTS_ALL})
			
		find_library(OpenCV_${__CVLIB}_LIBRARY_DEBUG NAMES "${__CVLIB}" "${__CVLIB}${CVLIB_SUFFIX}d" "lib${__CVLIB}${CVLIB_SUFFIX}d" PATHS "${OpenCV_DIR}/lib${PACKAGE_LIB_SUFFIX_DBG}" "$ENV{OpenCV_ROOT}/lib${PACKAGE_LIB_SUFFIX_DBG}" "${OpenCV_LIB_DIR_OPT}" "${OpenCV_3RDPARTY_LIB_DIR_OPT}" "${OpenCV_DIR}/lib" "${OpenCV_LIB_PATHS}" NO_DEFAULT_PATH)
		find_library(OpenCV_${__CVLIB}_LIBRARY_RELEASE NAMES "${__CVLIB}" "${__CVLIB}${CVLIB_SUFFIX}" "lib${__CVLIB}${CVLIB_SUFFIX}" PATHS "${OpenCV_DIR}/lib${PACKAGE_LIB_SUFFIX_REL}" "$ENV{OpenCV_ROOT}/lib${PACKAGE_LIB_SUFFIX_REL}" "${OpenCV_LIB_DIR_DBG}" "${OpenCV_3RDPARTY_LIB_DIR_DBG}" "${OpenCV_DIR}/lib" "${OpenCV_LIB_PATHS}" NO_DEFAULT_PATH)
		find_library(OpenCV_${__CVLIB}_LIBRARY_ALL NAMES "${__CVLIB}" "opencv_${__CVLIB}" PATH_SUFFIXES "opencv")
		
		#Remove the cache value
		set(OpenCV_${__CVLIB}_LIBRARY "" CACHE STRING "" FORCE)
		
		#both debug/release
		if(OpenCV_${__CVLIB}_LIBRARY_DEBUG AND OpenCV_${__CVLIB}_LIBRARY_RELEASE)
			set(OpenCV_${__CVLIB}_LIBRARY debug ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} optimized ${OpenCV_${__CVLIB}_LIBRARY_RELEASE} CACHE STRING "" FORCE)
		#only debug
		elseif(OpenCV_${__CVLIB}_LIBRARY_DEBUG)
			set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} CACHE STRING "" FORCE)
		#only release
		elseif(OpenCV_${__CVLIB}_LIBRARY_RELEASE)
			set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_RELEASE} CACHE STRING "" FORCE)
		#both debug/release
		elseif(OpenCV_${__CVLIB}_LIBRARY_ALL)
			set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_ALL} CACHE STRING "" FORCE)
		#no library found
		else()
			message("OpenCV component NOT found: ${__CVLIB}")
			set(OpenCV_FOUND FALSE)
		endif()
		
		#Add to the general list
		if(OpenCV_${__CVLIB}_LIBRARY)
			set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
		endif()
			
	endforeach()

	## Loop over each external component and add its library file
	foreach(__CVLIB ${OpenCV_LIB_COMPONENTS_EXTRA})
			
		#Add to the general list
		if("${SYSTEM_OS}" STREQUAL "${OS_NAME_WIN}")
			list(APPEND OpenCV_LIBS "${__CVLIB}.lib")
		else()
			list(APPEND OpenCV_LIBS "${__CVLIB}.a")
		endif()
			
	endforeach()

	set(OpenCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIRS}" "${OpenCV_DIR}/include")
	message(STATUS "OpenCV ${OpenCV_VERSION} found (include: ${OpenCV_INCLUDE_DIRS})")

else()

	package_report_not_found(OpenCV "Please specify OpenCV directory using OpenCV_ROOT env. variable")

endif()
##====================================================


##====================================================
## Backward compatibility
##----------------------------------------------------
if(OpenCV_FOUND)
	set(OpenCV_DIR "${OpenCV_DIR}" CACHE PATH "" FORCE)
	mark_as_advanced(OpenCV_DIR)
	set(OpenCV_LIBS "${OpenCV_LIBS}" CACHE PATH "" FORCE)
	mark_as_advanced(OpenCV_LIBS)
	if(OpenCV_VERSION_MAJOR LESS 3)
		option(OpenCV_CAN_BREAK_BINARY_COMPATIBILITY "Allow changes breaking binary compatibility with previous OpenCV version" ON)
		if(OpenCV_CAN_BREAK_BINARY_COMPATIBILITY)
			set(OpenCV_DEFINITIONS -DOPENCV_CAN_BREAK_BINARY_COMPATIBILITY)
		endif()
	endif()
endif()
##====================================================
