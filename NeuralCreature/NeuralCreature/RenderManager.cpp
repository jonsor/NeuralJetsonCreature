/**
RenderManager.cpp
Purpose: Sets up and manages the render library OpenGL.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "RenderManager.h"

/**
	Initializes a GLFW render window for the application to draw to.

	@param width The height of the window in pixels.
	@param height The width of the window in pixels.
*/
GLFWwindow* RenderManager::initWindow(GLuint width, GLuint height)
{
	initGLFW();
	GLFWwindow* window = glfwCreateWindow(width, height, "Neural Creature", nullptr, nullptr);

	if (window == nullptr) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return nullptr;
	}
	glfwMakeContextCurrent(window);
	return window;
}

/**
	Sets up and initilalizes the GLFW library. 
*/
void RenderManager::initGLFW()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwSwapInterval(1);
}

/**
	Sets up and initializes the Glew library.

	@param window The applications GLFW render window.
*/
void RenderManager::initGLEW(GLFWwindow* window) {
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cout << "Failed to initialize GLEW" << std::endl;
		return;
	}
	initView(window);
}

/**
	Sets up the viewport and a few other OpenGL options.

	@param window The applications GLFW render window.
*/
void RenderManager::initView(GLFWwindow* window)
{
	//Set Viewport
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	//OpenGL Options
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glEnable(GL_DEPTH_TEST);
}

RenderManager::~RenderManager()
{
}
