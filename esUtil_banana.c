#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>
#include "esUtil_raspi.h"

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

void ESUTIL_API esInitContext(ESContext *esContext) {
    if (esContext != NULL) {
        memset(esContext, 0, sizeof (ESContext));
    }
}

static EGLint const config_attribute_list[] = {
    EGL_RED_SIZE, 8,
    EGL_GREEN_SIZE, 8,
    EGL_BLUE_SIZE, 8,
    EGL_ALPHA_SIZE, 8,
    EGL_BUFFER_SIZE, 32,
    EGL_STENCIL_SIZE, 0,
    EGL_DEPTH_SIZE, 0,
    EGL_SAMPLES, 4,
    EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
    EGL_SURFACE_TYPE, EGL_WINDOW_BIT | EGL_PIXMAP_BIT,
    EGL_NONE
};
static const EGLint context_attribute_list[] = {
    EGL_CONTEXT_CLIENT_VERSION, 2,
    EGL_NONE
};

struct mali_native_window native_window = {
    .width = 1,
    .height = 1,
};
static EGLint window_attribute_list[] = {
    EGL_NONE
};

GLboolean ESUTIL_API esCreateWindow(ESContext *esContext, const char* title, GLint width, GLint height, GLuint flags) {
    EGLint egl_major, egl_minor;
    EGLConfig config;
    EGLint num_config;
    EGLContext context;

    native_window.width = width;
    native_window.height = height;

    esContext->eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    if (esContext->eglDisplay == EGL_NO_DISPLAY) {
        fprintf(stderr, "Error: No display found!\n");
        return GL_FALSE;
    }

    if (!eglInitialize(esContext->eglDisplay, &egl_major, &egl_minor)) {
        fprintf(stderr, "Error: eglInitialise failed!\n");
        return GL_FALSE;
    }

    eglChooseConfig(esContext->eglDisplay, config_attribute_list, &config, 1,
            &num_config);

    context = eglCreateContext(esContext->eglDisplay, config, EGL_NO_CONTEXT,
            context_attribute_list);
    if (context == EGL_NO_CONTEXT) {
        fprintf(stderr, "Error: eglCreateContext failed: 0x%08X\n",
                eglGetError());
        return GL_FALSE;
    }
    esContext->eglSurface = eglCreateWindowSurface(esContext->eglDisplay, config,
            &native_window,
            window_attribute_list);

    if (esContext->eglSurface == EGL_NO_SURFACE) {
        fprintf(stderr, "Error: eglCreateWindowSurface failed: "
                "0x%08X\n", eglGetError());
        return GL_FALSE;
    }

    if (!eglQuerySurface(esContext->eglDisplay, esContext->eglSurface, EGL_WIDTH, &width) ||
            !eglQuerySurface(esContext->eglDisplay, esContext->eglSurface, EGL_HEIGHT, &height)) {
        fprintf(stderr, "Error: eglQuerySurface failed: 0x%08X\n",
                eglGetError());
        return GL_FALSE;
    }

    if (!eglMakeCurrent(esContext->eglDisplay, esContext->eglSurface, esContext->eglSurface, context)) {
        fprintf(stderr, "Error: eglMakeCurrent() failed: 0x%08X\n",
                eglGetError());
        return GL_FALSE;
    }

    return GL_TRUE;
}

void ESUTIL_API esRegisterDrawFunc(ESContext *esContext, void (ESCALLBACK *drawFunc)(ESContext*)) {
    esContext->drawFunc = drawFunc;
}

void ESUTIL_API esRegisterUpdateFunc(ESContext *esContext, void (ESCALLBACK *updateFunc)(ESContext*, float)) {
    esContext->updateFunc = updateFunc;
}

void ESUTIL_API esRegisterKeyFunc(ESContext *esContext, void (ESCALLBACK *keyFunc)(ESContext*, unsigned char, int, int)) {
    esContext->keyFunc = keyFunc;
}

void ESUTIL_API esLogMessage(const char *formatStr, ...) {
    va_list params;
    char buf[BUFSIZ];

    va_start(params, formatStr);
    vsprintf(buf, formatStr, params);

    printf("%s", buf);

    va_end(params);
}
