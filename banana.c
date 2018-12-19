#include <stdlib.h>
#include <stdio.h>
#include <GLES2/gl2.h>
#include <EGL/egl.h>

#define PRINTGLERROR printOglError(__FILE__, __LINE__);
int printOglError(char *file, int line) {
    //
    // Returns 1 if an OpenGL error occurred, 0 otherwise.
    //
    GLenum glErr;
    int retCode = 0;

    glErr = glGetError();
    while (glErr != GL_NO_ERROR) {
        printf("\rglError in file %s @ line %i - ", file, line);

        switch (glErr) {
            case GL_INVALID_OPERATION: printf("INVALID_OPERATION");
                break;
            case GL_INVALID_ENUM: printf("INVALID_ENUM");
                break;
            case GL_INVALID_VALUE: printf("INVALID_VALUE");
                break;
            case GL_OUT_OF_MEMORY: printf("OUT_OF_MEMORY");
                break;
            case GL_INVALID_FRAMEBUFFER_OPERATION: printf("INVALID_FRAMEBUFFER_OPERATION");
                break;
            default: printf("else");
        }
        printf("\n");
        retCode = 1;
        glErr = glGetError();
    }
    return retCode;
}

GLuint programTest;
static const char *vertex_shader_source =
	"attribute vec4 aPosition;    \n"
	"attribute vec4 aColor;       \n"
	"                             \n"
	"varying vec4 vColor;         \n"
	"                             \n"
	"void main()                  \n"
	"{                            \n"
	"    vColor = aColor;         \n"
	"    gl_Position = aPosition; \n"
	"}                            \n";
static const char *fragment_shader_source =
	"precision mediump float;     \n"
	"                             \n"
	"varying vec4 vColor;         \n"
	"                             \n"
	"void main()                  \n"
	"{                            \n"
	"    gl_FragColor = vColor;   \n"
	"}                            \n";

int init_GL() {
    printf("\r%s\n\r%s\n\r%s\n\r%s\n", glGetString(GL_VERSION), glGetString(GL_VENDOR), glGetString(GL_RENDERER), glGetString(GL_SHADING_LANGUAGE_VERSION));

   


    return GL_TRUE;
}