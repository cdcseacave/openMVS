FIND_PACKAGE(OpenMVG QUIET)
IF (OPENMVG_FOUND)
	find_package(Ceres REQUIRED)
	INCLUDE_DIRECTORIES(${OPENMVG_INCLUDE_DIRS})

	add_definitions(-D_USE_OPENMVG)
	set(LIBS_DEPEND "MVS;Common;OpenMVG::openMVG_sfm;OpenMVG::openMVG_camera;OpenMVG::openMVG_image")
ELSE()
	set(LIBS_DEPEND "MVS;Common")
	MESSAGE("OPENMVG_NOT FOUND : OpenMVG importer with JSON support will not be build")
ENDIF()

if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_executable_with_flags(InterfaceOpenMVG "Apps" "${cxx_default}" "${LIBS_DEPEND};${OpenMVS_EXTRA_LIBS}" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

# Install
INSTALL(TARGETS InterfaceOpenMVG
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
