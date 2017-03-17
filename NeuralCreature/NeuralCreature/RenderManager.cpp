#include "stdafx.h"
#include "RenderManager.h"



RenderManager::RenderManager()
{
}

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

RenderManager::~RenderManager()
{
}
