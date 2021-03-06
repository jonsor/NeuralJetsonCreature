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
#include "Box.h"
#include <math.h>
#include <btBulletDynamicsCommon.h>
#include "PhysicsManager.h"
#include "RenderManager.h"
#include "NeuralNetwork.h"
#include "GeneticAlgorithm.h"

#define _USE_MATH_DEFINES

class World
{
private:
	RenderManager rm;
	PhysicsManager pm;
	//const GLuint WIDTH = 3072, HEIGHT = 1728;	//Laptop
	const GLuint WIDTH = 1280, HEIGHT = 720;	//Normal
	const double PI = 3.141592653589793238463;
	void incrementTargetAngles();

public:
	void init();
	void renderLoop(GLFWwindow * window, GLint planeVAO, GLint lightVAO, Shader lightingShader);
	void doMovement();
	void initPlaneAndLight(GLuint* lightVAO, GLuint* planeVAO);

	~World();
};

