if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_executable_with_flags(ReconstructMesh "Apps" "${cxx_default}" "MVS;${OpenMVS_EXTRA_LIBS}" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

# Ensure libCommon, libIO, and other dependencies are linked using the plain signature
target_link_libraries(ReconstructMesh Common IO)  # Add tinyxml2 if required

# Install
INSTALL(TARGETS ReconstructMesh
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
