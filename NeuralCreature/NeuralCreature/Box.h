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
#include <iterator>

class Box
{
private:
	glm::vec3 position;
	glm::vec3 previousPosition;
	glm::vec3 color;
	GLfloat width;
	GLfloat height;
	GLfloat depth;
	GLuint BoxVAO;
	GLfloat angle;
	btScalar mass;
	glm::vec3 axisOfRotation;

	//Bullet physics variables
	btCollisionShape* fallShape;
	btDefaultMotionState* fallMotionState;
	btCollisionShape* collisionShape;
	btRigidBody* rigidBody;
	std::map<std::string, btHingeConstraint*> hinges;
	std::map<std::string, btPoint2PointConstraint*> joints;
	std::map<std::string, btGeneric6DofConstraint*> dofConstraints;

	const double PI = 3.141592653589793238463;
	btTransform startPos;
	bool groundCollision;


public:
	Box(glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth, btScalar mass);
	void render(Shader shader);
	void setColor(glm::vec3 color);
	glm::vec3 getColor();
	void setPosition(glm::vec3 position);
	glm::vec3 getPosition();
	void setRotation(GLfloat angle, glm::vec3 angleMatrix);
	void setUpPhysicsBox();
	btRigidBody* getRigidBody();
	void updatePhysics();
	void addHinge(glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Box* BoxB, bool notCollision, PhysicsManager* pm, std::string name);
	void addHinge(glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Box* BoxB, bool notCollision, const btScalar minAngle, const btScalar maxAngle, PhysicsManager* pm, std::string name);
	void addDOFConstraint(Box * cubeB, bool notCollision, btScalar xOffset, PhysicsManager * pm, std::string name);
	void addJoint(glm::vec3 pivotA, glm::vec3 pivotB, Box* BoxB, bool notCollision, PhysicsManager* pm, std::string name);
	btHingeConstraint* getHinge(std::string name);
	btGeneric6DofConstraint* getdofConstraint(std::string name);
	btPoint2PointConstraint* getJoint(std::string name);
	void setHingeAngles(std::string name, const btScalar minAngle, const btScalar maxAngle);
	GLfloat getWidth();
	GLfloat getHeight();
	GLfloat getDepth();
	void setCollidingWithGround(bool colliding);
	bool isCollidingWithGround();
	void remove(PhysicsManager * pm);
	void removeConstraint(PhysicsManager * pm);
	void reset();
	glm::vec3 getPreviousPosition();
	~Box();
};

