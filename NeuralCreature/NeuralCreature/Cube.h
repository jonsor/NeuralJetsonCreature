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
#include "Util.h"
#include <vector>
#include <map>
#include "PhysicsManager.h"

class Cube
{
private:
	glm::vec3 position;
	glm::vec3 color;
	GLfloat width;
	GLfloat height;
	GLfloat depth;
	GLuint cubeVAO;
	GLfloat angle;
	btScalar mass;
	glm::vec3 axisOfRotation;
	
	//Bullet physics variables
	btCollisionShape* collisionShape;
	btRigidBody* rigidBody;
	std::map<std::string, btHingeConstraint*> hinges;
	const double PI = 3.141592653589793238463;


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
	void addHinge(glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Cube* cubeB, bool notCollision, PhysicsManager* pm, std::string name);
	btHingeConstraint* getHinge(std::string name);
	~Cube();
};

