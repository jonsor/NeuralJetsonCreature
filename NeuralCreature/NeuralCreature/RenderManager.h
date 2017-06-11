#pragma once

//GLEW
#define GLEW_STATIC
#include <GL/glew.h>

//GLFW
#include <GLFW/glfw3.h>

//Additional includes
#include <iostream>

class RenderManager
{
private:
	void initGLFW();
	void initView(GLFWwindow* window);

public:
	GLFWwindow* initWindow(GLuint width, GLuint height);
	void initGLEW(GLFWwindow* window);
	~RenderManager();
};

