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
public:
	RenderManager();
	GLFWwindow* initWindow(GLuint width, GLuint height);
	~RenderManager();
};

