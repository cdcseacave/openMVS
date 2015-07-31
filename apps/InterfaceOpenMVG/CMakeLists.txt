
cxx_executable_with_flags_no_pch(InterfaceOpenMVG "Apps" "${cxx_default}" "MVS" InterfaceOpenMVG.cpp "")

if(OpenMVS_USE_BREAKPAD AND BREAKPAD_FOUND)
	target_link_libraries(InterfaceOpenMVG ${BREAKPAD_LIBS})
endif()

FIND_PACKAGE(OpenMVG QUIET)
IF (OPENMVG_FOUND)
  INCLUDE_DIRECTORIES(${OPENMVG_INCLUDE_DIRS})
  cxx_executable_with_flags_no_pch(InterfaceOpenMVG_2 "Apps" "${cxx_default}" "MVS;${OPENMVG_LIBRARIES}" InterfaceOpenMVG_2.cpp "")
ELSE()
  MESSAGE("OPENMVG_NOT FOUND : OpenMVG importer will not be build")
ENDIF()
