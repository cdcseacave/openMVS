#include "LiteWindow.h"

#if defined(__APPLE__)

#define GL_SILENCE_DEPRECATION
#include <OpenGL/OpenGL.h>
#include <iostream>

LiteWindowCGL::LiteWindowCGL() : _context(nullptr) {}

LiteWindowCGL::~LiteWindowCGL() {
	if (_context) {
		if (CGLGetCurrentContext() == static_cast<CGLContextObj>(_context))
			CGLSetCurrentContext(nullptr);
		CGLDestroyContext(static_cast<CGLContextObj>(_context));
		_context = nullptr;
	}
}

int LiteWindowCGL::IsValid() const { return _context != nullptr; }

void LiteWindowCGL::MakeCurrent() {
	if (_context)
		CGLSetCurrentContext(static_cast<CGLContextObj>(_context));
}

void LiteWindowCGL::Create(int /*x*/, int /*y*/, const char* /*display*/) {
	if (_context) return;

	// SiftGPU needs the legacy (OpenGL 2.1) profile, on a hardware renderer;
	// offline renderers are allowed so that GPUs not driving a display are usable
	const CGLPixelFormatAttribute attribs[] = {
		kCGLPFAOpenGLProfile, (CGLPixelFormatAttribute)kCGLOGLPVersion_Legacy,
		kCGLPFAAccelerated,
		kCGLPFAAllowOfflineRenderers,
		kCGLPFAColorSize, (CGLPixelFormatAttribute)24,
		kCGLPFAAlphaSize, (CGLPixelFormatAttribute)8,
		(CGLPixelFormatAttribute)0
	};
	CGLPixelFormatObj pixelFormat = nullptr;
	GLint numPixelFormats = 0;
	CGLError err = CGLChoosePixelFormat(attribs, &pixelFormat, &numPixelFormats);
	if (err != kCGLNoError || pixelFormat == nullptr) {
		std::cerr << "CGL: no accelerated pixel format: " << CGLErrorString(err) << std::endl;
		return;
	}
	CGLContextObj ctx = nullptr;
	err = CGLCreateContext(pixelFormat, nullptr, &ctx);
	CGLReleasePixelFormat(pixelFormat);
	if (err != kCGLNoError) {
		std::cerr << "CGL: failed to create context: " << CGLErrorString(err) << std::endl;
		return;
	}
	err = CGLSetCurrentContext(ctx);
	if (err != kCGLNoError) {
		std::cerr << "CGL: failed to make context current: " << CGLErrorString(err) << std::endl;
		CGLDestroyContext(ctx);
		return;
	}
	_context = ctx;
}

#endif
