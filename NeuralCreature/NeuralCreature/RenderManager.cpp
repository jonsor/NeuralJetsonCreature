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

void RenderManager::initGLEW(GLFWwindow* window) {
	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cout << "Failed to initialize GLEW" << std::endl;
		return;
	}
	initView(window);
}

void RenderManager::initView(GLFWwindow* window)
{
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
}

RenderManager::~RenderManager()
{
}
