#include "stdafx.h"
#include <GL/glew.h> 
#include <GLFW/glfw3.h> 
#include <iostream>

class OpenGlTest
{
public:
	OpenGlTest();
	~OpenGlTest();
	void controls(GLFWwindow * window, int key, int scancode, int action, int mods);
	void init();
	GLFWwindow* initWindow(const int resX, const int resY);
	void drawCube();
	void display(GLFWwindow * window);
	void start();
};
