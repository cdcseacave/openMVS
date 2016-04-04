if(MSVC)
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_executable_with_flags_no_pch(InterfaceVisualSFM "Apps" "${cxx_default}" "MVS" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H})

if(OpenMVS_USE_BREAKPAD AND BREAKPAD_FOUND)
	target_link_libraries(InterfaceVisualSFM ${BREAKPAD_LIBS})
endif()

# Install
INSTALL(TARGETS InterfaceVisualSFM
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
