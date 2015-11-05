# List sources files
FILE(GLOB PCH_C "Common.cpp")

FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

# Place Common.cpp as the first file in the list
# needed by cotire when setting PCH manually
LIST(REMOVE_ITEM LIBRARY_FILES_C ${PCH_C})
SET(LIBRARY_FILES_C "${PCH_C};${LIBRARY_FILES_C}")

cxx_library_with_type_no_pch(Common "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
set_target_pch(Common Common.h)

# Link its dependencies
TARGET_LINK_LIBRARIES(Common ${Boost_LIBRARIES} ${OpenCV_LIBS})

# Install
INSTALL(FILES ${LIBRARY_FILES_H} DESTINATION "include/${PROJECT_NAME}/Common")
INSTALL(TARGETS Common DESTINATION "lib/${PROJECT_NAME}")
