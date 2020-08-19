#include "glSetup.h"

#include <Windows.h>
#include "include/GLFW/glfw3.h"
#include "include/GL/gl.h"
#include "include/GL/glut.h"

#pragma comment(lib, "lib/glfw3.lib")
#pragma comment(lib, "lib/opengl32.lib")
#pragma comment(lib, "lib/glut32.lib")

#include "Eigen/Dense"
using namespace Eigen;

#include <iostream>
using namespace std;

void init();
void quit();
void initializeParticleSystem();
void update();
void solveODE();
void collisionHandling();
void keyboard(GLFWwindow* window, int key, int code, int action, int mods);
void mouse(GLFWwindow* window, int button, int action, int mods); // mouse
void cursor(GLFWwindow* window, double xpos, double ypos); // cursor
void render(GLFWwindow* window);
void setupLight();
void setupMaterial();
void drawSphere(float radius, const Vector3f& color, int N);

// Play configuration
bool pause = true;
int frame = 0;

// Light configuration
Vector4f light(0.0, 0.0, 5.0, 1); // Light position

								  // Global coordinate frame
float AXIS_LENGTH = 3;
float AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1,1,1,1 };

Vector3f ee_goal;
bool keyc = false;
bool keya = false;
bool keyd = false;
bool keyn = false;
bool keyr = false;

bool keyp = false; // point damping
bool keys = false; // damped spring

// Particles
int nParticles = 0;
Vector3f p[10]; // Particle position
Vector3f v[10]; // Particle velocity

int nEdges = 0, click = 0, enum1, enum2;
int e1[20]; // One end of the edge
int e2[20]; // The other end of the edge
float l[20]; // Rest length between particles
float k[20]; // Spring constants
float k0 = 1.0; // Global spring constant

// Sphere
GLUquadricObj* sphere = NULL;

bool useConst = true; // Contraints
bool constrained[10];

// Geometry and mass
float radius = 0.02; // 2cm
float m = 0.01; // 10g

// Time stepping
int N_SUBSTEPS = 1; // Sub-time steps per frame
float h = 1.0 / 60.0 / N_SUBSTEPS; // Time step

 // External force
float useGravity = true;
Vector3f gravity(0, -9.8, 0); // Gravity -9.8m/s^2

 // Collision
float k_r = 0.75;
float epsilon = 1.0E-4;

bool contact[10]; // Contact state
Vector3f contactN[10]; // Contact normal vector

const int nWalls = 4;
Vector3f wallP[nWalls]; // Points in the walls
Vector3f wallN[nWalls]; // Normal vectors of the walls

						// Method
enum IntegrationMethod
{
	EULER = 1,
	MODIFIED_EULER,
} intMethod = MODIFIED_EULER;

int main(int argc, char* argv[])
{
	// Orthographics viewing
	perspectiveView = false;

	// Initialize the OpenGL system
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor);
	if (window == NULL) return -1;

	// Vertical sync for 60fps
	glfwSwapInterval(1);

	// Callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetMouseButtonCallback(window, mouse);
	glfwSetCursorPosCallback(window, cursor);

	// Depth test
	glEnable(GL_DEPTH_TEST);

	// Normal vectors are normalized after transformation.
	glEnable(GL_NORMALIZE);

	// Back face culling
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Initialization - Main loop - Finalization
	init();
	initializeParticleSystem();

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		if (!pause) update();

		render(window); // Draw one frame
		glfwSwapBuffers(window); // Swap buffers
		glfwPollEvents(); // Events
	}

	// Finalization
	quit();

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void rebuildSpringK()
{
	cout << "Spring constant = " << k0 << endl;

	// Spring constant
	for (int i = 0; i < nEdges; i++)
		k[i] = k0 / l[i]; // Inversely proportion to the spring length
}

void initializeParticleSystem()
{
	// To generate the same random values at each execution
	srand(0);

	// Normal vectors of the 4 surrounding walls
	wallN[0] = Vector3f(1.0, 0, 0); // Left
	wallN[1] = Vector3f(-1.0, 0, 0); // Right
	wallN[2] = Vector3f(0, 1.0, 0); // Bottom
	wallN[3] = Vector3f(0, -1.0, 0); // Top

	for (int i = 0; i < nWalls; i++)
		wallN[i].normalize();

	// Collision handling
	collisionHandling();
}

void quit()
{
	// Delete quadric shapes
	gluDeleteQuadric(sphere);
}

void update()
{
	// Solve ordinary differential equation
	for (int i = 0; i < N_SUBSTEPS; i++)
		solveODE();

	// Time increment
	frame++;
}

void solveODE()
{
	// Total force
	Vector3f f[10];

	for (int i = 0; i < nParticles; i++)
	{
		// Initialization
		f[i].setZero();

		// Gravity
		if (useGravity) f[i] += m*gravity;

		if (keyp)
			f[i] -= 0.01*v[i];
 	}

	for (int i = 0; i < nEdges; i++)
	{
		Vector3f v_i = p[e1[i]] - p[e2[i]];
		float L_i = v_i.norm();
		Vector3f n_i = v_i.normalized();
		Vector3f f_i = k[i] * (L_i - l[i])*n_i;

		f[e2[i]] += f_i;
		f[e1[i]] -= f_i;

		if (keys)
		{
			Vector3f vel_i = v[e1[i]] - v[e2[i]];
			Vector3f f2_i = 0.05* vel_i.dot(n_i)*n_i;

			f[e2[i]] += f2_i;
			f[e1[i]] -= f2_i;
		}
	}

	for (int i = 0; i < nParticles; i++)
	{
		// Constraint
		if (constrained[i]) continue;

		// Contact force
		if (contact[i]) f[i] -= contactN[i].dot(f[i])*contactN[i];

		// Time stepping
		switch (intMethod)
		{
		case EULER:
			p[i] += h*v[i];
			v[i] += h*f[i] / m;
			break;

		case MODIFIED_EULER:
			v[i] += h*f[i] / m;
			p[i] += h*v[i];
			break;
		}
	}

	// Collision handling
	collisionHandling();
}

void collisionHandling()
{
	// Points of the 4 surrounding walls: It can be changed.
	wallP[0] = Vector3f(-1.0*aspect, 0, 0); // Left
	wallP[1] = Vector3f(1.0*aspect, 0, 0); // Right
	wallP[2] = Vector3f(0, -1.0, 0); // Bottom
	wallP[3] = Vector3f(0, 1.0, 0); // Top

									// Collision wrt the walls
	for (int i = 0; i < nParticles; i++)
	{
		contact[i] = false;

		for (int j = 0; j < nWalls; j++)
		{
			float d_N = wallN[j].dot(p[i] - wallP[j]);
			if (d_N < radius)
			{
				// Position correction
				p[i] += (radius - d_N)*wallN[j];

				// Normal velocity
				float v_N = wallN[j].dot(v[i]);

				if (fabs(v_N) < epsilon) // Contact check
				{
					contact[i] = true;
					contactN[i] = wallN[j];
				}
				else if (v_N < 0) // Velocity correction
				{
					v[i] -= (1 + k_r)*v_N*wallN[j];
				}
			}
		}
	}
}

int findIndex(float x, float y, float z)
{
	float length = 0.25, dis;
	int index;
	for (int i = 0; i < nParticles; i++)
	{
		dis = sqrt(pow((p[i][0] - x), 2) + pow((p[i][1] - y), 2) + pow((p[i][2] - z), 2));
		if (length > dis) {
			length = dis;
			index = i;
		}
	}

	if (length < 0.25)
		return index;
	else
		return 15;
}

void movePoint(float x, float y, float z)
{
	int index = findIndex(x, y, z);

	if (index != 15)
		p[index] = Vector3f(x,y,z);
}

void selectParticle(float x, float y, float z)
{
	int index = findIndex(x, y, z);

	if (index != 15)
		constrained[index] = true;
}

void attachEdge(float x, float y, float z)
{
	int index = findIndex(x, y, z);

	if (click == 0)
	{
		click = 0;
		if (index != 15)
		{
			enum1 = index;
			click = 1;
		}
	}
	else if (click == 1)
	{
		click = 1;
		if (index != 15)
		{
			enum2 = index;
			if (enum1 != enum2)
			{
				e1[nEdges] = enum1; e2[nEdges] = enum2;
				l[nEdges] = (p[e1[nEdges]] - p[e2[nEdges]]).norm();
				click = 2;
				enum1 = enum2 = 15;
			}
		}
	}
}

void render(GLFWwindow* window)
{
	// Background color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Lighting
	setupLight();

	// Material
	setupMaterial();

	// Particles
	if (nParticles > 0)
	{
		for (int i = 0; i < nParticles; i++)
		{
			glPushMatrix();
			glTranslatef(p[i][0], p[i][1], p[i][2]);
			if (constrained[i]) drawSphere(radius, Vector3f(1, 1, 0), 20);
			else drawSphere(radius, Vector3f(0, 1, 0), 20);
			glPopMatrix();
		}
	}

	// Edges
	if (nEdges > 0)
	{
		glLineWidth(7 * dpiScaling);
		glColor3f(0, 0, 1);
		glBegin(GL_LINES);
		for (int i = 0; i < nEdges; i++)
		{
			glVertex3fv(p[e1[i]].data());
			glVertex3fv(p[e2[i]].data());
		}
		glEnd();
	}
}

void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
			// Quit
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;

			// Controls
		case GLFW_KEY_SPACE: pause = !pause; break; // Play on/off
		case GLFW_KEY_G: useGravity = !useGravity; break; // Gravity on/off
		case GLFW_KEY_C: keyc = true; keya = false; keyd = false; keyn = false; keyr = false; break;
		case GLFW_KEY_A: keyc = false; keya = true; keyd = false; keyn = false; keyr = false; break;
		case GLFW_KEY_D: keyc = false; keya = false; keyd = true; keyn = false; keyr = false; pause = true; break;
		case GLFW_KEY_N: keyc = false; keya = false; keyd = false; keyn = true; keyr = false; break;
		case GLFW_KEY_R: keyc = false; keya = false; keyd = false; keyn = false; keyr = true; break;

		case GLFW_KEY_P: keyp = !keyp; break;
		case GLFW_KEY_S: keys = !keys; break;

		case GLFW_KEY_E: intMethod = EULER; break;
		case GLFW_KEY_M: intMethod = MODIFIED_EULER; break;

		// Spring constants
		case GLFW_KEY_UP: k0 = min(k0 + 0.1, 10.0); rebuildSpringK(); break;
		case GLFW_KEY_DOWN: k0 = max(k0 - 0.1, 0.1); rebuildSpringK(); break;

		// Sub-time steps
		case GLFW_KEY_1: N_SUBSTEPS = 1; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_2: N_SUBSTEPS = 2; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_3: N_SUBSTEPS = 3; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_4: N_SUBSTEPS = 4; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_5: N_SUBSTEPS = 5; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_6: N_SUBSTEPS = 6; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_7: N_SUBSTEPS = 7; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_8: N_SUBSTEPS = 8; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		case GLFW_KEY_9: N_SUBSTEPS = 9; h = 1.0 / 60.0 / N_SUBSTEPS; break;
		}
		if (keyr) // Remove all nail constraints
		{
			for (int i = 0; i < nParticles; i++)
				constrained[i] = false;
			keyr = false;
		}
		if (keyp)
			cout << "point damping on" << endl;
		if (keys)
			cout << "damped spring on" << endl;
	}
}

bool drag = false;
bool attach = false;

void mouse(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		// In the screen coordinate
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		ee_goal = Vector3f(xpos, ypos, 0);

		// In the workspace. See reshape() in glSetup.cpp
		float aspect = (float)screenW / screenH;
		ee_goal[0] = 2.0 * (ee_goal[0] / screenW - 0.5)*aspect;
		ee_goal[1] = -2.0 * (ee_goal[1] / screenH - 0.5);

		if (nParticles == 10)
		{
			keyc = false;
		}
		else if (keyc) {
			p[nParticles] = Vector3f(ee_goal[0], ee_goal[1], 0);
			v[nParticles].setZero(); // Velocity
			contact[nParticles] = false; // Contact
			constrained[nParticles] = false; // constrained
			nParticles++;
		}
		else if (keyn)
		{
			if (nParticles > 0 && nParticles < 11)
				selectParticle(ee_goal[0], ee_goal[1], 0);
		}

		if (keyd)
			drag = true;
		
		if (nParticles < 2)
		{
			keya = false;
		}
		else if (keya)
		{
			ee_goal = Vector3f(xpos, ypos, 0);

			float aspect = (float)screenW / screenH;
			ee_goal[0] = 2.0 * (ee_goal[0] / screenW - 0.5)*aspect;
			ee_goal[1] = -2.0 * (ee_goal[1] / screenH - 0.5);

			attachEdge(ee_goal[0], ee_goal[1], 0);
			if (click == 2)
			{
				nEdges++;
				click = 0;
				rebuildSpringK();
			}
		}
		else
			click = 0;
	}
	else if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (keyd)
			if (drag)
				drag = false;
	}
}

void cursor(GLFWwindow* window, double xpos, double ypos)
{
	if (drag)
	{
		ee_goal = Vector3f(xpos, ypos, 0);

		float aspect = (float)screenW / screenH;
		ee_goal[0] = 2.0 * (ee_goal[0] / screenW - 0.5)*aspect;
		ee_goal[1] = -2.0 * (ee_goal[1] / screenH - 0.5);

		movePoint(ee_goal[0], ee_goal[1], 0);
	}
}

void init()
{
	// Prepare quadric shapes
	sphere = gluNewQuadric();
	gluQuadricDrawStyle(sphere, GLU_FILL);
	gluQuadricNormals(sphere, GLU_SMOOTH);
	gluQuadricOrientation(sphere, GLU_OUTSIDE);
	gluQuadricTexture(sphere, GL_FALSE);

	// Keyboard and mouse
	cout << "Keyboard Input: space for play on/off" << endl;
	cout << "Keyboard Input: g for gravity on/off" << endl;
	cout << "Keyboard Input: c for create" << endl;
	cout << "Keyboard Input: a for attach" << endl;
	cout << "keyboard Input: d for drag" << endl;
	cout << "Keyboard Input: n for nail" << endl;
	cout << "Keyboard Input: r for remove all nail constraints" << endl;

	cout << "Keyboard Input: p for point damping on/off" << endl;
	cout << "Keyboard Input: s for damped spring on/off" << endl;

	cout << "Keyboard Input: e for the Euler integration" << endl;
	cout << "Keyboard Input: m for the modified Euler integration" << endl;
	cout << "Keyboard Input: [1:9] for the # of sub-time steps" << endl;

	cout << "Keyboard Input: up to increase the spring constants" << endl;
	cout << "Keyboard Input: down to decrease the spring constant" << endl;
}

// Light
void setupLight()
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	GLfloat ambient[4] = { 0.1,0.1,0.1,1 };
	GLfloat diffuse[4] = { 1.0,1.0,1.0,1 };
	GLfloat specular[4] = { 1.0,1.0,1.0,1 };

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light.data());
}

// Material
void setupMaterial()
{
	// Material
	GLfloat mat_ambient[4] = { 0.1,0.1,0.1,1 };
	GLfloat mat_specular[4] = { 0.5,0.5,0.5,1 };
	GLfloat mat_shininess = 128;

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

void setDiffuseColor(const Vector3f& color)
{
	GLfloat mat_diffuse[4] = { color[0], color[1], color[2], 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
}

// Draw a sphere after setting up its material
void drawSphere(float radius, const Vector3f& color, int N)
{
	// Material
	setDiffuseColor(color);

	// Sphere using GLU quadrics
	gluSphere(sphere, radius, N, N);
}