#pragma once

//GLEW
#define GLEW_STATIC
#include <GL/glew.h>

//GLFW
#include <GLFW/glfw3.h>

//GLM Mathematics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//Additional includes
#include <iostream>
#include "Shader.h"
#include "Camera.h"
#include "Cube.h"


class OpenGLTutorial
{
private:
	// Shaders
	const GLchar* vertexShaderSource = "#version 330 core\n"
		"layout (location = 0) in vec3 position;\n"
		"void main()\n"
		"{\n"
		"gl_Position = vec4(position.x, position.y, position.z, 1.0);\n"
		"}\0";
	const GLchar* fragmentShaderSource = "#version 330 core\n"
		"out vec4 color;\n"
		"uniform vec4 customColor;\n"
		"void main()\n"
		"{\n"
		"color = customColor;\n"
		"}\n\0";

	const GLuint WIDTH = 1280, HEIGHT = 720;

public:
	OpenGLTutorial();
	void init();
	void initGLEW();
	void initView(GLFWwindow * window);
	void renderLoop(GLFWwindow * window, GLint shaderProgram, GLint planeVAO, GLint cubeVAO, GLint lightVAO, Shader lightingShader, Cube cube);
	void renderLoop(GLFWwindow * window, GLint shaderProgram, GLint planeVAO, GLint cubeVAO, GLint lightVAO, Shader lightingShader);
	GLint initShaders();
	void doMovement();
	~OpenGLTutorial();
};

