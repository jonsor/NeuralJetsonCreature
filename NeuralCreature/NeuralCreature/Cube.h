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
#include <btBulletDynamicsCommon.h>

class Cube
{
private:
	glm::vec3 position;
	glm::vec3 color;
	GLfloat with;
	GLfloat height;
	GLuint cubeVAO;
	GLfloat angle;
	btScalar mass;
	glm::vec3 axisOfRotation;

	//Bullet physics variables

	btCollisionShape* collisionShape;
	btRigidBody* rigidBody;


public:
	Cube(glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth, btScalar mass);
	void render(Shader shader);
	void setColor(glm::vec3 color);
	glm::vec3 getColor();
	void setPosition(glm::vec3 position);
	glm::vec3 getPosition();
	void setRotation(GLfloat angle, glm::vec3 angleMatrix);
	void setUpPhysicsCube();
	btRigidBody* getRigidBody();
	void updatePhysics();
	
	~Cube();
};

