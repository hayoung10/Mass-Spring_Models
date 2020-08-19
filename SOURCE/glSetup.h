#pragma once
#ifndef __GL_SETUP_H_
#define __GL_SETUP_H_

#include "include/GL/glew.h"
#include "include/GLFW/glfw3.h"

#pragma comment(lib, "lib/glfw3.lib")
#pragma comment(lib, "lib/glew32.lib")

extern bool fullScreen; // Start window with full screen
extern bool noMenuBar; // with no menu bar

extern int vsync; // Vertical sync on/off
extern bool perspectiveView; // Perspective or orthographic viewing
extern float fovy; // Field of view in the y direction

extern float screenScale; // Portion of the screen when not using full screen
extern int screenW, screenH; // screenScale portion of the screen
extern int windowW, windowH; // Framebuffer size
extern float aspect; // Aspective ratio = windowW/windowH
extern float dpiScaling; // DPI scaling for HIDPI

GLFWwindow* initializeOpenGL(int argc, char* argv[], GLfloat bg[4], bool modern = false);
void reshape(GLFWwindow* window, int w, int h);

void drawAxes(float l, float w);

#endif // GL_SETUP_H_