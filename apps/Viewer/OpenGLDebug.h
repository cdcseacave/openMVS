/*
 * OpenGLDebug.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#pragma once


// I N C L U D E S /////////////////////////////////////////////////

#include <glad/glad.h>


// D E F I N E S ///////////////////////////////////////////////////

// Configuration
#ifndef _RELEASE
#define OPENGL_DEBUG_ENABLE
#endif

// Macros for convenient usage
#ifdef OPENGL_DEBUG_ENABLE
#define GL_CHECK(call) \
		do { \
			OPENGL_DEBUG::ClearOpenGLErrors(); \
			call; \
			OPENGL_DEBUG::CheckOpenGLError(#call, __FILE__, __LINE__); \
		} while(0)

#define GL_DEBUG_SCOPE(name) \
		OPENGL_DEBUG::ScopeErrorChecker _gl_scope_checker(name, __FILE__, __LINE__)

#define GL_CLEAR_ERRORS() OPENGL_DEBUG::ClearOpenGLErrors()

// statement macro in both configurations: the value is discarded here so that the
// disabled form is a no-op statement and not a value-less expression (-Wunused-value);
// call OPENGL_DEBUG::EnableOpenGLDebugOutput() directly to test whether it succeeded
#define GL_ENABLE_DEBUG_OUTPUT() (void)OPENGL_DEBUG::EnableOpenGLDebugOutput()
#else
#define GL_CHECK(call) call
#define GL_DEBUG_SCOPE(name)
#define GL_CLEAR_ERRORS()
#define GL_ENABLE_DEBUG_OUTPUT() ((void)0)
#endif


// S T R U C T S ///////////////////////////////////////////////////

// Comprehensive OpenGL error checking and debugging utilities
// for automatic detection of OpenGL errors after function calls.
//
// Usage:
// 1. Define OPENGL_DEBUG_ENABLE before including this header for debug builds
// 2. Use GL_CHECK macro after OpenGL calls: GL_CHECK(glDrawElements(...));
// 3. Use GL_DEBUG_SCOPE for automatic checking within a scope
// 4. Use EnableOpenGLDebugOutput() for OpenGL 4.3+ debug contexts

namespace OPENGL_DEBUG {

// Error checking function
inline std::pair<GLenum, std::string> GetOpenGLError() {
	GLenum error = glGetError();
	if (error == GL_NO_ERROR)
		return {GL_NO_ERROR, ""};
	std::string errorString;
	switch (error) {
		case GL_INVALID_ENUM:
			errorString = "GL_INVALID_ENUM";
			break;
		case GL_INVALID_VALUE:
			errorString = "GL_INVALID_VALUE";
			break;
		case GL_INVALID_OPERATION:
			errorString = "GL_INVALID_OPERATION";
			break;
		case GL_OUT_OF_MEMORY:
			errorString = "GL_OUT_OF_MEMORY";
			break;
		case GL_INVALID_FRAMEBUFFER_OPERATION:
			errorString = "GL_INVALID_FRAMEBUFFER_OPERATION";
			break;
		default:
			errorString = "UNKNOWN_ERROR_" + std::to_string(error);
			break;
	}
	return {error, errorString};
}
inline bool CheckOpenGLError(const char* function, const char* file, int line) {
    auto [error, errorString] = GetOpenGLError();
    if (error == GL_NO_ERROR)
        return true; // No error, everything is fine
    DEBUG("OpenGL Error: %s (0x%X)\n  Function: %s\n  File: %s:%d", errorString.c_str(), error, function, file, line);
    ASSERT("OpenGL error detected!" == NULL);
    return false;
}

// Clear all pending OpenGL errors
inline void ClearOpenGLErrors() {
	while (glGetError() != GL_NO_ERROR) {
		// Clear all errors
	}
}

// OpenGL Debug Message Callback (for OpenGL 4.3+ debug contexts)
#ifdef GL_VERSION_4_3
inline void APIENTRY OpenGLDebugCallback(GLenum source, GLenum type, GLuint id, 
                                       GLenum severity, GLsizei length, 
                                       const GLchar* message, const void* userParam) {
    // Ignore non-significant error/warning codes
    if (id == 131169 || id == 131185 || id == 131218 || id == 131204) return;
    DEBUG("OpenGL Debug Message (%u): %s", id, message);
    const char* src = "";
    switch (source) {
        case GL_DEBUG_SOURCE_API:             src = "API"; break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   src = "Window System"; break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER: src = "Shader Compiler"; break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:     src = "Third Party"; break;
        case GL_DEBUG_SOURCE_APPLICATION:     src = "Application"; break;
        case GL_DEBUG_SOURCE_OTHER:           src = "Other"; break;
    }
    DEBUG("  Source: %s", src);
    const char* typ = "";
    switch (type) {
        case GL_DEBUG_TYPE_ERROR:               typ = "Error"; break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: typ = "Deprecated Behaviour"; break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  typ = "Undefined Behaviour"; break;
        case GL_DEBUG_TYPE_PORTABILITY:         typ = "Portability"; break;
        case GL_DEBUG_TYPE_PERFORMANCE:         typ = "Performance"; break;
        case GL_DEBUG_TYPE_MARKER:              typ = "Marker"; break;
        case GL_DEBUG_TYPE_PUSH_GROUP:          typ = "Push Group"; break;
        case GL_DEBUG_TYPE_POP_GROUP:           typ = "Pop Group"; break;
        case GL_DEBUG_TYPE_OTHER:               typ = "Other"; break;
    }
    DEBUG("  Type: %s", typ);
    const char* sev = "";
    switch (severity) {
        case GL_DEBUG_SEVERITY_HIGH:         sev = "high"; break;
        case GL_DEBUG_SEVERITY_MEDIUM:       sev = "medium"; break;
        case GL_DEBUG_SEVERITY_LOW:          sev = "low"; break;
        case GL_DEBUG_SEVERITY_NOTIFICATION: sev = "notification"; break;
    }
    DEBUG("  Severity: %s\n", sev);
}

// Enable OpenGL debug output (requires OpenGL 4.3+ debug context)
inline bool EnableOpenGLDebugOutput() {
    if (GLAD_GL_VERSION_4_3) {
        GLint flags;
        glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
        if (flags & GL_CONTEXT_FLAG_DEBUG_BIT) {
            glEnable(GL_DEBUG_OUTPUT);
            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
            glDebugMessageCallback(OpenGLDebugCallback, nullptr);
            glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
            DEBUG("OpenGL debug output enabled");
            return true;
        } else {
            DEBUG("OpenGL debug context not available");
            return false;
        }
    } else {
        DEBUG("OpenGL debug context not available: OpenGL 4.3+ required");
        return false;
    }
}
#else
inline bool EnableOpenGLDebugOutput() {
    DEBUG("OpenGL debug output requires OpenGL 4.3+");
    return false;
}
#endif // GL_VERSION_4_3

// RAII scope-based error checker
class ScopeErrorChecker {
public:
	ScopeErrorChecker(const char* scopeName, const char* file, int line) 
		: scopeName_(scopeName), file_(file), line_(line) {
		// Clear any existing errors at scope entry
		ClearOpenGLErrors();
	}

	~ScopeErrorChecker() {
		CheckOpenGLError(scopeName_, file_, line_);
	}
	
private:
	const char* scopeName_;
	const char* file_;
	int line_;
};
/*----------------------------------------------------------------*/

} // namespace OPENGL_DEBUG
