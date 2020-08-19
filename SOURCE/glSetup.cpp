#include "glSetup.h"

#include <string.h>
#include <iostream>
using namespace std;

bool fullScreen = false;
bool noMenuBar = false;
bool perspectiveView = true;

float screenScale = 0.5; // Portion of the screen when not using full screen
int screenW = 0, screenH = 0; // ScreenScale portion of the screen
int windowW, windowH; // Framebuffer size
float aspect;
float dpiScaling = 0; // for HIDPI: windowW = dpiScaling x screenW

int vsync = 1; // Vertical sync on/off

float fovy = 46.4; // fovy of 28mm lens in degree
				   //float fovy = 37.8; // fovy of 35mm lens in degree
				   //float fovy = 27.0; // fovy of 50mm lens in degree
				   //float fovy = 16.1; // fovy of 85mm lens in degree
				   //float fovy = 11.4; // fovy of 120mm lens in degree
				   //float fovy = 6.9; // fovy of 200mm lens in degree

void errorCallback(int error, const char* description)
{
	cerr << "####" << description << endl;
}

void reshape(GLFWwindow* window, int w, int h)
{
	aspect = (float)w / h;

	windowW = w;
	windowH = h;
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (perspectiveView)
	{
		float nearDist = 1.0;
		float farDist = 20.0;

		gluPerspective(fovy, aspect, nearDist, farDist);
	}
	else {
		glOrtho(-1.0*aspect, 1.0*aspect, -1.0, 1.0, -1.0, 1.0);
	}

	// The Screen size is required for mouse interaction.
	glfwGetWindowSize(window, &screenW, &screenH);
	cerr << "reshpae(" << w << ", " << h << ")";
	cerr << " with screen " << screenW << " x " << screenH << endl;
}

GLFWwindow* initializeOpenGL(int argc, char* argv[], GLfloat bgColor[4], bool modern)
{
	glfwSetErrorCallback(errorCallback);

	// Init GLFW
	if (!glfwInit()) exit(EXIT_FAILURE);

	if (modern) // Enable OpenGL 4.1 in OS X
	{
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

		glfwWindowHint(GLFW_SAMPLES, 4);
	}
	else { // Enable OpenGL 2.1 in OS X
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	}

	// Create the window
	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	int monitorW, monitorH;
	glfwGetMonitorPhysicalSize(monitor, &monitorW, &monitorH);
	cerr << "Status: Monitor " << monitorW << "mm x " << monitorH << "mm" << endl;

	// Full screen
	if (fullScreen) screenScale = 1.0;

	const GLFWvidmode* videoMode = glfwGetVideoMode(monitor);
	if (screenW == 0) screenW = videoMode->width * screenScale;
	if (screenH == 0) screenH = videoMode->height * screenScale;

	if (!fullScreen || !noMenuBar) monitor = NULL;
	GLFWwindow* window = glfwCreateWindow(screenW, screenH, argv[0], monitor, NULL);
	if (!window)
	{
		glfwTerminate();
		cerr << "Failed in glfwCreateWindow()" << endl;
		return NULL;
	}

	// Context
	glfwMakeContextCurrent(window);

	// Clear the background ASAP
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT);
	glFlush();
	glfwSwapBuffers(window);

	// Check the size of the window
	glfwGetWindowSize(window, &screenW, &screenH);
	cerr << "Status: Screen" << screenW << " x " << screenH << endl;

	glfwGetFramebufferSize(window, &windowW, &windowH);
	cerr << "Status: Framebuffer " << windowW << " x " << windowH << endl;

	// DPI scaling
	if (dpiScaling == 0) dpiScaling = float(windowW) / screenW;

	// Callbacks
	glfwSetFramebufferSizeCallback(window, reshape);

	// Get the OpenGL version and renderer
	cout << "Status: Renderer " << glGetString(GL_RENDERER) << endl;
	cout << "Status: Ventor " << glGetString(GL_VENDOR) << endl;
	cout << "Status: OpenGL " << glGetString(GL_VERSION) << endl;

	// Vertical sync ...
	glfwSwapInterval(vsync); // 0 for immediate mode (Tearing possible)

	if (modern)
	{
		// GLSL version for shader loading
		char glslVersion[256];
		strcpy(glslVersion, (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
		cout << "Status: GLSL " << glslVersion << endl;

		// GLEW: Supported version can be verified in glew.sourceforge.net/basic.html
		cerr << "Status: GLEW " << glewGetString(GLEW_VERSION) << endl;

		// Initializing GLEW
		GLenum error = glewInit();
		if (error != GLEW_OK)
		{
			cerr << "ERROR: " << glewGetErrorString(error) << endl;
			return 0;
		}
	}

	return window;
}

// Draw the coordinate frame with the xyz axes
void drawAxes(float l, float w)
{
	glLineWidth(w*dpiScaling);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(1, 0, 0); // x-axis
	glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 1, 0); // y-axis
	glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1); // z-axis
	glEnd();
}