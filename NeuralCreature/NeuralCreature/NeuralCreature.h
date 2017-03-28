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
#include <vector>
#include <math.h>
#include <btBulletDynamicsCommon.h>
#include "PhysicsManager.h"
#include "RenderManager.h"
#include "Creature.h"

#define _USE_MATH_DEFINES

class NeuralCreature
{
private:
	RenderManager rm;
	PhysicsManager pm;
	//const GLuint WIDTH = 3072, HEIGHT = 1728;	//Sjurs Laptop.
	const GLuint WIDTH = 1920, HEIGHT = 1080;	//Normal
	const double PI = 3.141592653589793238463;

public:
	void init();
	void renderLoop(GLFWwindow * window, GLint planeVAO, GLint lightVAO, Shader lightingShader, std::vector<Cube> cubes);
	//btRigidBody * createRigidBody(btCollisionShape * collisionShape, btScalar mass, const btTransform & transform) const;
	//GLint initShaders();
	void doMovement();
	void initPlaneAndLight(GLuint* lightVAO, GLuint* planeVAO);
	~NeuralCreature();
};

