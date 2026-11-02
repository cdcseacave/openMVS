if(NOT VIEWER_NAME)
	set(VIEWER_NAME "Viewer")
endif()

# Find required packages
# GLAD 2 (its CMake package generates the loader at build time) or GLAD 1
FIND_PACKAGE(Glad CONFIG QUIET)
if(Glad_FOUND AND COMMAND glad_add_library)
	MESSAGE(STATUS "GLAD 2 found")
	enable_language(C)
	# static, linked into the Viewer (glad 2.0.8 ignores STATIC when
	# BUILD_SHARED_LIBS is ON)
	set(_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
	set(BUILD_SHARED_LIBS OFF)
	glad_add_library(glad_gl STATIC REPRODUCIBLE QUIET API gl:compatibility=4.6)
	set(BUILD_SHARED_LIBS ${_BUILD_SHARED_LIBS})
	set(GLAD_LIBRARY glad_gl)
	add_definitions(-D_USE_GLAD2)
else()
	FIND_PACKAGE(glad QUIET)
	if(glad_FOUND)
		MESSAGE(STATUS "GLAD ${glad_VERSION} found")
		set(GLAD_LIBRARY glad::glad)
	else()
		MESSAGE("-- Can't find GLAD. Continuing without it.")
		RETURN()
	endif()
endif()
FIND_PACKAGE(glfw3 QUIET)
if(glfw3_FOUND)
	MESSAGE(STATUS "GLFW3 ${glfw3_VERSION} found")
else()
	MESSAGE("-- Can't find GLFW3. Continuing without it.")
	RETURN()
endif()
FIND_PACKAGE(imgui QUIET)
if(imgui_FOUND)
	MESSAGE(STATUS "ImGUI ${imgui_VERSION} found")
else()
	MESSAGE("-- Can't find ImGUI. Continuing without it.")
	RETURN()
endif()
find_path(PORTABLE_FILE_DIALOGS_INCLUDE_DIRS "portable-file-dialogs.h")
if(PORTABLE_FILE_DIALOGS_INCLUDE_DIRS)
	include_directories(${PORTABLE_FILE_DIALOGS_INCLUDE_DIRS})
	MESSAGE(STATUS "portable-file-dialogs found")
else()
	MESSAGE("-- Can't find portable-file-dialogs. Using fallback implementation.")
	RETURN()
endif()

# List sources files
if(MSVC)
	create_rc_files(${VIEWER_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/Viewer.ico")
	FILE(GLOB LIBRARY_FILES_C "*.cpp" "${CMAKE_CURRENT_BINARY_DIR}/*.rc")
elseif(APPLE)
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
	FILE(GLOB OBJC_MM_FILES "*.mm")
	list(APPEND LIBRARY_FILES_C ${OBJC_MM_FILES})
else()
	FILE(GLOB LIBRARY_FILES_C "*.cpp")
endif()
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")
FILE(GLOB SHADERS_LIBRARY_FILES "shaders/*.*")
SOURCE_GROUP("Shaders" FILES ${SHADERS_LIBRARY_FILES})

cxx_executable_with_flags(${VIEWER_NAME} "Apps" "${cxx_default}" "MVS;${GLAD_LIBRARY};${GLFW_STATIC_LIBRARIES};${glfw3_LIBRARY};${GLFW3_LIBRARY};glfw;imgui::imgui" ${LIBRARY_FILES_C} ${LIBRARY_FILES_H} ${SHADERS_LIBRARY_FILES})

# On macOS, link Cocoa explicitly for our ObjC++ bridge and disable PCH for .mm
if(APPLE)
    set_source_files_properties(${OBJC_MM_FILES} PROPERTIES SKIP_PRECOMPILE_HEADERS ON)
    target_link_libraries(${VIEWER_NAME} "-framework Cocoa")
endif()

# Build the Viewer as a GUI application on platforms that support it so
# launching from the desktop does not open or attach a console/terminal.
if(WIN32)
	# On Windows: tell CMake/Visual Studio to build a Win32 GUI executable (no console)
	set_target_properties(${VIEWER_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
elseif(APPLE)
	# On macOS: create a macOS app bundle so it's treated as a GUI app by Finder
	set_target_properties(${VIEWER_NAME} PROPERTIES MACOSX_BUNDLE TRUE)
endif()

# Link portable-file-dialogs if available
if(portable-file-dialogs_FOUND)
	target_link_libraries(${VIEWER_NAME} PRIVATE portable-file-dialogs::portable-file-dialogs)
endif()

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(${VIEWER_NAME} PRIVATE "Common.h")
endif()

if(APPLE)
	# Configure Info.plist from the templates directory and attach it to the
	# macOS bundle. Use the project's OpenMVS version variables and the
	# Viewer icon basename for the bundle icon.
	set(INFO_PLIST_IN "${CMAKE_CURRENT_SOURCE_DIR}/templates/Info.plist.in")
	set(ICON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/Viewer.icns")
	get_filename_component(ICON_NAME "${ICON_SRC}" NAME_WE)
	# Copy the icns into the build dir and add it to the bundle resources
	configure_file(${ICON_SRC} ${CMAKE_CURRENT_BINARY_DIR}/Viewer.icns COPYONLY)
	# Add the icns as a resource so CMake includes it in the built .app
	target_sources(${VIEWER_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/Viewer.icns")
	set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/Viewer.icns" PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")
	# Tell CMake which icon file to use for the bundle (filename only)
	set_target_properties(${VIEWER_NAME} PROPERTIES MACOSX_BUNDLE_ICON_FILE "${ICON_NAME}.icns")
	configure_file(${INFO_PLIST_IN} ${CMAKE_CURRENT_BINARY_DIR}/Info.plist @ONLY)
	set_target_properties(${VIEWER_NAME} PROPERTIES MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_BINARY_DIR}/Info.plist")
endif()

if(LINUX)
	# Prepare a Linux desktop entry for better integration. Exec path is configured
	# to the installed binary location (prefix + install bin dir). Install the
	# resulting .desktop file into share/applications so desktop environments pick it up.
	set(DESKTOP_IN "${CMAKE_CURRENT_SOURCE_DIR}/templates/openMVS-Viewer.desktop.in")
	set(EXEC_PATH "${CMAKE_INSTALL_PREFIX}/${INSTALL_BIN_DIR}/${VIEWER_NAME}")
	get_filename_component(ICON_NAME "${CMAKE_CURRENT_SOURCE_DIR}/Viewer.svg" NAME_WE)
	# Register both .mvs and .dmap MIME types so desktop environments treat
	# both as owned by the OpenMVS Viewer. openmvs-mime.xml.in defines both
	# application/x-openmvs-mvs and application/x-openmvs-dmap.
	set(MIME_TYPE "application/x-openmvs-mvs;application/x-openmvs-dmap")
	configure_file(${DESKTOP_IN} ${CMAKE_CURRENT_BINARY_DIR}/openMVS-Viewer.desktop @ONLY)
	INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/openMVS-Viewer.desktop"
		DESTINATION "share/applications" COMPONENT desktop)
	# Install an SVG icon into hicolor icon theme (scalable) if present
	INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/Viewer.svg"
		DESTINATION "share/icons/hicolor/scalable/apps" COMPONENT desktop)
	# Install shared-mime-info XML so the system knows about .mvs files, then
	# update the MIME database so the desktop file association works immediately.
	set(MIME_IN "${CMAKE_CURRENT_SOURCE_DIR}/templates/openmvs-mime.xml.in")
	configure_file(${MIME_IN} ${CMAKE_CURRENT_BINARY_DIR}/openmvs-mime.xml @ONLY)
	INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/openmvs-mime.xml"
		DESTINATION "share/mime/packages" COMPONENT desktop)
	FIND_PROGRAM(MIME_UPDATE_CMD update-mime-database)
	if(MIME_UPDATE_CMD)
		# run update-mime-database at install time (best effort)
		INSTALL(CODE "execute_process(COMMAND ${MIME_UPDATE_CMD} \"${CMAKE_INSTALL_PREFIX}/share/mime\" RESULT_VARIABLE _res) if(NOT _res EQUAL 0) message(WARNING \"update-mime-database failed: \${_res}\") endif()")
	endif()
endif()

if(WIN32)
	# Configure a Windows registry template for packagers/installers (manual import)
	set(REG_IN "${CMAKE_CURRENT_SOURCE_DIR}/templates/Viewer-fileassoc.reg.in")
	configure_file(${REG_IN} ${CMAKE_CURRENT_BINARY_DIR}/Viewer-fileassoc.reg @ONLY)
	INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/Viewer-fileassoc.reg"
	    DESTINATION "share/doc/${VIEWER_NAME}" COMPONENT doc)
endif()

# Install
if(APPLE)
	# When installing a MACOSX_BUNDLE target, CMake requires a BUNDLE DESTINATION.
	# Install the .app bundle into the same bin install directory variable the project uses.
	INSTALL(TARGETS ${VIEWER_NAME}
		EXPORT OpenMVSTargets
		BUNDLE DESTINATION "${INSTALL_BIN_DIR}"
		RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
	INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/Info.plist"
		DESTINATION "${INSTALL_BIN_DIR}/${VIEWER_NAME}.app/Contents" COMPONENT bin)
	INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/Viewer.icns"
		DESTINATION "${INSTALL_BIN_DIR}/${VIEWER_NAME}.app/Contents/Resources" COMPONENT bin)
else()
	INSTALL(TARGETS ${VIEWER_NAME}
		EXPORT OpenMVSTargets
		RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
endif()
