###########################################################
#                  Find EIGEN Library
#----------------------------------------------------------

find_path(EIGEN_DIR "Eigen/Core"
    HINTS "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}"
    PATHS "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}" "/usr" "/usr/local" "/usr/share" "/usr/local/share" "/usr/lib/x86_64-linux-gnu/cmake"
    PATH_SUFFIXES "eigen" "eigen3" "include"
    DOC "Root directory of EIGEN library")

##====================================================
## Include EIGEN library
##----------------------------------------------------
if(EXISTS "${EIGEN_DIR}" AND NOT "${EIGEN_DIR}" STREQUAL "")
	set(EIGEN_FOUND TRUE)
	set(EIGEN_INCLUDE_DIRS ${EIGEN_DIR})
	set(EIGEN_DIR "${EIGEN_DIR}" CACHE PATH "" FORCE)
	mark_as_advanced(EIGEN_DIR)

	# Extract Eigen version from Eigen/src/Core/util/Macros.h
	SET(EIGEN_VERSION_FILE ${EIGEN_INCLUDE_DIRS}/Eigen/src/Core/util/Macros.h)
	IF (NOT EXISTS ${EIGEN_VERSION_FILE})
		EIGEN_REPORT_NOT_FOUND(
		  "Could not find file: ${EIGEN_VERSION_FILE} "
		  "containing version information in Eigen install located at: "
		  "${EIGEN_INCLUDE_DIRS}.")
	ELSE (NOT EXISTS ${EIGEN_VERSION_FILE})
		FILE(READ ${EIGEN_VERSION_FILE} EIGEN_VERSION_FILE_CONTENTS)

		STRING(REGEX MATCH "#define EIGEN_WORLD_VERSION [0-9]+"
		  EIGEN_WORLD_VERSION "${EIGEN_VERSION_FILE_CONTENTS}")
		STRING(REGEX REPLACE "#define EIGEN_WORLD_VERSION ([0-9]+)" "\\1"
		  EIGEN_WORLD_VERSION "${EIGEN_WORLD_VERSION}")

		STRING(REGEX MATCH "#define EIGEN_MAJOR_VERSION [0-9]+"
		  EIGEN_MAJOR_VERSION "${EIGEN_VERSION_FILE_CONTENTS}")
		STRING(REGEX REPLACE "#define EIGEN_MAJOR_VERSION ([0-9]+)" "\\1"
		  EIGEN_MAJOR_VERSION "${EIGEN_MAJOR_VERSION}")

		STRING(REGEX MATCH "#define EIGEN_MINOR_VERSION [0-9]+"
		  EIGEN_MINOR_VERSION "${EIGEN_VERSION_FILE_CONTENTS}")
		STRING(REGEX REPLACE "#define EIGEN_MINOR_VERSION ([0-9]+)" "\\1"
		  EIGEN_MINOR_VERSION "${EIGEN_MINOR_VERSION}")

		# This is on a single line s/t CMake does not interpret it as a list of
		# elements and insert ';' separators which would result in 3.;2.;0 nonsense.
		SET(EIGEN_VERSION "${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION}")
	ENDIF (NOT EXISTS ${EIGEN_VERSION_FILE})
	set(EIGEN_INCLUDE_DIR ${EIGEN_DIR})

	message(STATUS "Eigen ${EIGEN_VERSION} found (include: ${EIGEN_INCLUDE_DIRS})")
else()
	package_report_not_found(EIGEN "Please specify EIGEN directory using EIGEN_ROOT env. variable")
endif()
##====================================================
