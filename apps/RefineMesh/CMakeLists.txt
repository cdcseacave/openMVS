if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_executable_with_flags_no_pch(RefineMesh "Apps" "${cxx_default}" "MVS" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

if(OpenMVS_USE_BREAKPAD AND BREAKPAD_FOUND)
	target_link_libraries(RefineMesh ${BREAKPAD_LIBS})
endif()

link_libraries(RefineMesh ${CERES_LIBS} ${GLOG_LIBS})

target_link_libraries(RefineMesh ${CERES_LIBRARIES} ${GLOG_LIBRARY})

# Install
INSTALL(TARGETS RefineMesh
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
