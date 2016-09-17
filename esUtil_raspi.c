#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include "esUtil_raspi.h"
#include "bcm_host.h"

EGLBoolean CreateEGLContext(EGLNativeWindowType hWnd, EGLDisplay* eglDisplay, EGLContext* eglContext, EGLSurface* eglSurface,
		EGLint attribList[]) {
	EGLint numConfigs;
	EGLint majorVersion;
	EGLint minorVersion;
	EGLDisplay display;
	EGLContext context;
	EGLSurface surface;
	EGLConfig config;
	EGLint contextAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };

	// Get Display
	display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	if (display == EGL_NO_DISPLAY) {
		return EGL_FALSE;
	}

	// Initialize EGL
	if (!eglInitialize(display, &majorVersion, &minorVersion)) {
		return EGL_FALSE;
	}

	// Get configs
	if (!eglGetConfigs(display, NULL, 0, &numConfigs)) {
		return EGL_FALSE;
	}

	// Choose config
	if (!eglChooseConfig(display, attribList, &config, 1, &numConfigs)) {
		return EGL_FALSE;
	}

	// Create a surface
	surface = eglCreateWindowSurface(display, config, (EGLNativeWindowType) hWnd, NULL);
	if (surface == EGL_NO_SURFACE) {
		return EGL_FALSE;
	}

	// Create a GL context
	context = eglCreateContext(display, config, EGL_NO_CONTEXT, contextAttribs);
	if (context == EGL_NO_CONTEXT) {
		return EGL_FALSE;
	}

	// Make the context current
	if (!eglMakeCurrent(display, surface, surface, context)) {
		return EGL_FALSE;
	}

	*eglDisplay = display;
	*eglSurface = surface;
	*eglContext = context;
	return EGL_TRUE;
}

///
//  WinCreate() - RaspberryPi, direct surface (No X, Xlib)
//
//      This function initialized the display and window for EGL
//
EGLBoolean WinCreate(ESContext *esContext, const char *title) {
	int32_t success = 0;

	static EGL_DISPMANX_WINDOW_T nativewindow;

	DISPMANX_ELEMENT_HANDLE_T dispman_element;
	DISPMANX_DISPLAY_HANDLE_T dispman_display;
	DISPMANX_UPDATE_HANDLE_T dispman_update;
	VC_RECT_T dst_rect;
	VC_RECT_T src_rect;

	uint32_t display_width;
	uint32_t display_height;

	// create an EGL window surface, passing context width/height
	success = graphics_get_display_size(0 /* LCD */, &display_width, &display_height);
	if (success < 0) {
		return EGL_FALSE;
	}

	// You can hardcode the resolution here:
	display_width = 800;
	display_height = 480;

	dst_rect.x = 0;
	dst_rect.y = 0;
	dst_rect.width = display_width;
	dst_rect.height = display_height;

	src_rect.x = 0;
	src_rect.y = 0;
	src_rect.width = display_width << 16;
	src_rect.height = display_height << 16;

	dispman_display = vc_dispmanx_display_open(0 /* LCD */);
	dispman_update = vc_dispmanx_update_start(0);

	dispman_element =
			vc_dispmanx_element_add(dispman_update, dispman_display, 0/*layer*/, &dst_rect, 0/*src*/, &src_rect, DISPMANX_PROTECTION_NONE, 0 /*alpha*/, 0/*clamp*/, 0/*transform*/);

	nativewindow.element = dispman_element;
	nativewindow.width = display_width;
	nativewindow.height = display_height;
	vc_dispmanx_update_submit_sync(dispman_update);

	esContext->hWnd = &nativewindow;

	return EGL_TRUE;
}
///
//  userInterrupt()
//
//      Reads from X11 event loop and interrupt program if there is a keypress, or
//      window close action.
//
GLboolean userInterrupt(ESContext *esContext) {
	//GLboolean userinterrupt = GL_FALSE;
	//return userinterrupt;

	// Ctrl-C for now to stop

	return GL_FALSE;
}

//////////////////////////////////////////////////////////////////
//
//  Public Functions
//
//

///
//  esInitContext()
//
//      Initialize ES utility context.  This must be called before calling any other
//      functions.
//
void ESUTIL_API esInitContext(ESContext *esContext) {
	bcm_host_init();
	if (esContext != NULL) {
		memset(esContext, 0, sizeof(ESContext));
	}
}

///
//  esCreateWindow()
//
//      title - name for title bar of window
//      width - width of window to create
//      height - height of window to create
//      flags  - bitwise or of window creation flags 
//          ES_WINDOW_ALPHA       - specifies that the framebuffer should have alpha
//          ES_WINDOW_DEPTH       - specifies that a depth buffer should be created
//          ES_WINDOW_STENCIL     - specifies that a stencil buffer should be created
//          ES_WINDOW_MULTISAMPLE - specifies that a multi-sample buffer should be created
//
GLboolean ESUTIL_API esCreateWindow(ESContext *esContext, const char* title, GLint width, GLint height, GLuint flags) {
	EGLint attribList[] = {
	EGL_RED_SIZE, 5,
	EGL_GREEN_SIZE, 6,
	EGL_BLUE_SIZE, 5,
	EGL_ALPHA_SIZE, (flags & ES_WINDOW_ALPHA) ? 8 : EGL_DONT_CARE,
	EGL_DEPTH_SIZE, (flags & ES_WINDOW_DEPTH) ? 8 : EGL_DONT_CARE,
	EGL_STENCIL_SIZE, (flags & ES_WINDOW_STENCIL) ? 8 : EGL_DONT_CARE,
	EGL_SAMPLE_BUFFERS, (flags & ES_WINDOW_MULTISAMPLE) ? 1 : 0,
	EGL_NONE };

	if (esContext == NULL) {
		return GL_FALSE;
	}

	esContext->width = width;
	esContext->height = height;

	if (!WinCreate(esContext, title)) {
		return GL_FALSE;
	}

	if (!CreateEGLContext(esContext->hWnd, &esContext->eglDisplay, &esContext->eglContext, &esContext->eglSurface, attribList)) {
		return GL_FALSE;
	}

	return GL_TRUE;
}
/*
void ESUTIL_API esMainLoop(ESContext *esContext) {
	struct timeval t1, t2;
	struct timezone tz;
	float deltatime, deltatime2;
	float totaltime = 0.0f;
	unsigned int frames = 0;
	long int nsec_old = 0;
	unsigned int us = 30000;

	gettimeofday(&t1, &tz);
	struct timespec spec1, spec2, spec3;
	unsigned int ms_min = 4294967295, ms_max = 0, ms, ms_avg = 0;
	while (userInterrupt(esContext) == GL_FALSE) {
		gettimeofday(&t2, &tz);
		deltatime = (float) (t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) * 1e-6) * 1000;
		t1 = t2;

		clock_gettime(CLOCK_MONOTONIC, &spec1);

		esContext->updateFunc(esContext, deltatime);
		esContext->drawFunc(esContext);

		eglSwapBuffers(esContext->eglDisplay, esContext->eglSurface);
		clock_gettime(CLOCK_MONOTONIC, &spec2);
		totaltime += deltatime / 1000.0;
		frames++;
		ms = (spec2.tv_sec - spec1.tv_sec) * 1000 + (spec2.tv_nsec - spec1.tv_nsec) * 1e-6;

		ms_avg += ms;
		if (ms < ms_min)
			ms_min = ms;
		if (ms > ms_max)
			ms_max = ms;

		if (totaltime > 4.0f) {
			printf("FPS: %3.4f frametime: %i %4.1f %i US: %i\n", frames / totaltime, ms_min,(float) ms_avg / frames, ms_max, us);

			if ((frames / totaltime) > 30.0) {
				us += 2000;
			} else {
				us = us > 1000 ? us - 1000 : us;
			}

			totaltime -= 4.0f;
			frames = 0;
			ms_avg = 0;
			ms_min = 4294967295;
			ms_max = 0;
		}
		usleep(us);
	}
}*/
///
//  esRegisterDrawFunc()
//
void ESUTIL_API esRegisterDrawFunc(ESContext *esContext, void (ESCALLBACK *drawFunc)(ESContext*)) {
	esContext->drawFunc = drawFunc;
}

///
//  esRegisterUpdateFunc()
//
void ESUTIL_API esRegisterUpdateFunc(ESContext *esContext, void (ESCALLBACK *updateFunc)(ESContext*, float)) {
	esContext->updateFunc = updateFunc;
}

///
//  esRegisterKeyFunc()
//
void ESUTIL_API esRegisterKeyFunc(ESContext *esContext, void (ESCALLBACK *keyFunc)(ESContext*, unsigned char, int, int)) {
	esContext->keyFunc = keyFunc;
}

///
// esLogMessage()
//
//    Log an error message to the debug output for the platform
//
void ESUTIL_API esLogMessage(const char *formatStr, ...) {
	va_list params;
	char buf[BUFSIZ];

	va_start(params, formatStr);
	vsprintf(buf, formatStr, params);

	printf("%s", buf);

	va_end(params);
}
