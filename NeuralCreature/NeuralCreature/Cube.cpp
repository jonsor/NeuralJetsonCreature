/**
Cube.cpp
Purpose: Creates cubes to use in the world. Also renders, updates physics and creates hinges to attach other cubes.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/


#include "stdafx.h"
#include "Cube.h"

/**
	Creates a new OpenGL cube and initializes the physics.

	@param position Vector position of the cube.
	@param color Vector color of the cube.
	@param width The width of the cube.
	@param height The heigth of the cube.
	@param depth The depth of the cube.
	@param mass The Mass of the cube.
*/
Cube::Cube(glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth, btScalar mass) : position(position), color(color), mass(mass), width(width), height(height), depth(depth)
{
	// Cubes
	GLfloat cubeVertices[] = {
		//BACK
		-width, -height, -depth,  0.0f,  0.0f, -1.0f,
		width, -height, -depth,  0.0f,  0.0f, -1.0f,
		width,  height, -depth,  0.0f,  0.0f, -1.0f,
		width,  height, -depth,  0.0f,  0.0f, -1.0f,
		-width,  height, -depth,  0.0f,  0.0f, -1.0f,
		-width, -height, -depth,  0.0f,  0.0f, -1.0f,
		//FRONT
		-width, -height,  depth,  0.0f,  0.0f, 1.0f,
		width, -height,  depth,  0.0f,  0.0f, 1.0f,
		width,  height,  depth,  0.0f,  0.0f, 1.0f,
		width,  height,  depth,  0.0f,  0.0f, 1.0f,
		-width,  height,  depth,  0.0f,  0.0f, 1.0f,
		-width, -height,  depth,  0.0f,  0.0f, 1.0f,
		//LEFT SIDE
		-width,  height,  depth, -1.0f,  0.0f,  0.0f,
		-width,  height, -depth, -1.0f,  0.0f,  0.0f,
		-width, -height, -depth, -1.0f,  0.0f,  0.0f,
		-width, -height, -depth, -1.0f,  0.0f,  0.0f,
		-width, -height,  depth, -1.0f,  0.0f,  0.0f,
		-width,  height,  depth, -1.0f,  0.0f,  0.0f,
		//RIGHT SIDE
		width,  height,  depth,  1.0f,  0.0f,  0.0f,
		width,  height, -depth,  1.0f,  0.0f,  0.0f,
		width, -height, -depth,  1.0f,  0.0f,  0.0f,
		width, -height, -depth,  1.0f,  0.0f,  0.0f,
		width, -height,  depth,  1.0f,  0.0f,  0.0f,
		width,  height,  depth,  1.0f,  0.0f,  0.0f,
		//BOTTOM
		-width, -height, -depth,  0.0f, -1.0f,  0.0f,
		width, -height, -depth,  0.0f, -1.0f,  0.0f,
		width, -height,  depth,  0.0f, -1.0f,  0.0f,
		width, -height,  depth,  0.0f, -1.0f,  0.0f,
		-width, -height,  depth,  0.0f, -1.0f,  0.0f,
		-width, -height, -depth,  0.0f, -1.0f,  0.0f,
		//TOP
		-width,  height, -depth,  0.0f,  1.0f,  0.0f,
		width,  height, -depth,  0.0f,  1.0f,  0.0f,
		width,  height,  depth,  0.0f,  1.0f,  0.0f,
		width,  height,  depth,  0.0f,  1.0f,  0.0f,
		-width,  height,  depth,  0.0f,  1.0f,  0.0f,
		-width,  height, -depth,  0.0f,  1.0f,  0.0f
	};

	angle = 0.0f;
	axisOfRotation = glm::vec3(1.0f, 1.0f, 1.0f);
	//CUBE STUFF
	GLuint cubeVBO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &cubeVBO);

	glBindVertexArray(cubeVAO);

	glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	// Normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0); // Unbind VAO

	//mass = 1;
	//Set up physics
	Cube::setUpPhysicsCube();

	rigidBody->getMotionState()->getWorldTransform(startPos);

}

/**
	Renders the cube with OpenGL.

	@param shader Shader object for rendering.
*/
void Cube::render(Shader shader)
{
	shader.use();
	glBindVertexArray(cubeVAO);
	GLint objectColorLoc = glGetUniformLocation(shader.program, "objectColor");
	glUniform3f(objectColorLoc, color.x, color.y, color.z);

	glm::mat4 model;
	model = glm::translate(model, position);
	//Hacky roation fix - doesnt render if axis is equal to zero
	if (axisOfRotation.x == 0 || axisOfRotation.x == 0 || axisOfRotation.x == 0) {
		axisOfRotation.x += 0.000000001f;
		axisOfRotation.y += 0.000000001f;
		axisOfRotation.z += 0.000000001f;
	}
	model = glm::rotate(model, angle, axisOfRotation);
	GLint modelLoc = glGetUniformLocation(shader.program, "model"); 
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model)); 
	//GLfloat angle = 20.0f * i;
	//model = glm::rotate(model, angle, glm::vec3(1.0f, 0.3f, 0.5f));
	glDrawArrays(GL_TRIANGLES, 0, 36);

	glBindVertexArray(0);
}

void Cube::setColor(glm::vec3 color)
{
	this->color = color;
}

glm::vec3 Cube::getColor()
{
	return color;
}

void Cube::setPosition(glm::vec3 position)
{
	this->position = position;
}

void Cube::reset() {

	rigidBody->getMotionState()->setWorldTransform(startPos);
	rigidBody->setWorldTransform(startPos);
	rigidBody->clearForces();
}
glm::vec3 Cube::getPosition()
{
	return position;
}

void Cube::setRotation(GLfloat angle, glm::vec3 axisOfRotation)
{
	this->angle = angle;
	this->axisOfRotation = axisOfRotation;
}

/**
	Sets up physics for the cube.
	Does not add it to the physics world.
*/
void Cube::setUpPhysicsCube()
{
	//TODO: delete fall shape at the end of game loop
	fallShape = new btBoxShape(btVector3(width, height, depth));
	fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), Util::convertToBtVector3(position)));
	btVector3 fallInertia(0, 0, 0);
	if(mass != 0) fallShape->calculateLocalInertia(mass, fallInertia);

	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
	rigidBody = new btRigidBody(fallRigidBodyCI);
}


btRigidBody* Cube::getRigidBody()
{
	return rigidBody;
}

/**
	Calculates and applies the next phyical step for the Cube.
	Should be called in the main loop.
*/
void Cube::updatePhysics()
{
	btTransform trans;
	rigidBody->getMotionState()->getWorldTransform(trans);
	float mat[16];
	trans.getOpenGLMatrix(mat);
	trans.getRotation().getX();
	setPosition(glm::vec3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
	
	float x = trans.getRotation().getX();
	float y = trans.getRotation().getY();
	float z = trans.getRotation().getZ();
	float angle = trans.getRotation().getAngle();

	setRotation(angle, glm::vec3(x, y, z));
}

/**
	Adds a new hinge to this cube that attaches cubeB.

	@param pivotA
	@param pivotB
	@param axisA The vector direction of rotation of this cube in relation to cubeB.
	@param axisB The vector direction of rotation of cubeB in relation to this cube.
	@param cubeB A pointer to the cube that you want to attach to this one.
	@param notCollision Wether or not the two joint cubes should be able to collide.
	@param pm A pointer to the Pysics Manager to add the hinges to the world.
	@param name The key for hinge storage.


*/
void Cube::addHinge(glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Cube* cubeB, bool notCollision, PhysicsManager* pm, std::string name)
{
	bool useReferenceFrameA = false;
	btHingeConstraint* hingeConstraint = new btHingeConstraint(
		*rigidBody,
		*cubeB->getRigidBody(),
		Util::convertToBtVector3(pivotA),
		Util::convertToBtVector3(pivotB),
		Util::convertToBtVector3(axisA),
		Util::convertToBtVector3(axisB),
		useReferenceFrameA);
	

	//Set constraint limit
	const btScalar low = -PI;
	const btScalar high = PI;
	hingeConstraint->setLimit(low, high);
	pm->addNewConstraint(hingeConstraint, notCollision);

	//Add to hinge array
	hinges[name] = hingeConstraint;
	hingeConstraint = nullptr;


	
}
/**
	Adds a new hinge to this cube that attaches cubeB.

	@param pivotA
	@param pivotB
	@param axisA The vector direction of rotation of this cube in relation to cubeB.
	@param axisB The vector direction of rotation of cubeB in relation to this cube.
	@param cubeB A pointer to the cube that you want to attach to this one.
	@param notCollision Wether or not the two joint cubes should be able to collide.
	@param minAngle The minimum angle of rotation on the hinge.
	@param maxAngle The maximum angle of rotation on the hinge.
	@param pm A pointer to the Pysics Manager to add the hinges to the world.
	@param name The key for hinge storage.
*/
void Cube::addHinge(glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Cube* cubeB, bool notCollision, const btScalar minAngle, const btScalar maxAngle, PhysicsManager * pm, std::string name)
{
	bool useReferenceFrameA = false;
	btHingeConstraint* hingeConstraint = new btHingeConstraint(
		*rigidBody,
		*cubeB->getRigidBody(),
		Util::convertToBtVector3(pivotA),
		Util::convertToBtVector3(pivotB),
		Util::convertToBtVector3(axisA),
		Util::convertToBtVector3(axisB),
		useReferenceFrameA);


	//Set constraint limit
	hingeConstraint->setLimit(minAngle, maxAngle);
	pm->addNewConstraint(hingeConstraint, notCollision);

	//Add to hinge array
	hinges[name] = hingeConstraint;
	hingeConstraint = nullptr;


}
/**
	Adds a new hinge to this cube that attaches cubeB.

	@param pivotA
	@param pivotB
	@param cubeB A pointer to the cube that you want to attach to this one.
	@param notCollision Wether or not the two joint cubes should be able to collide.
	@param pm A pointer to the Pysics Manager to add the hinges to the world.
	@param name The key for joint storage.
*/
void Cube::addJoint(glm::vec3 pivotA, glm::vec3 pivotB, Cube* cubeB, bool notCollision, PhysicsManager* pm, std::string name)
{
	btPoint2PointConstraint* jointConstraint = new btPoint2PointConstraint(*rigidBody, *cubeB->getRigidBody(), Util::convertToBtVector3(pivotA), Util::convertToBtVector3(pivotB));
	pm->addNewConstraint(jointConstraint, notCollision);
	joints[name] = jointConstraint;
	jointConstraint = nullptr;

}

btHingeConstraint* Cube::getHinge(std::string name)
{
	return hinges[name];
}

btPoint2PointConstraint* Cube::getJoint(std::string name)
{
	return joints[name];
}

/**
	Sets the maximum and minimum angle for the hinge, should be within +-PI.

	@param name Key name of the hinge.
	@param minAngle min angle.
	@param maxAngle max angle.
*/
void Cube::setHingeAngles(std::string name, const btScalar minAngle, const btScalar maxAngle)
{
	hinges[name]->setLimit(minAngle, maxAngle);
}

GLfloat Cube::getWidth()
{
	return width;
}

GLfloat Cube::getHeight()
{
	return height;
}

GLfloat Cube::getDepth()
{
	return depth;
}

void Cube::remove(PhysicsManager * pm)
{

	pm->removeBody(rigidBody);
}

void Cube::removeConstraint(PhysicsManager * pm)
{


	for (std::map<std::string, btHingeConstraint*>::iterator itr = hinges.begin(); itr != hinges.end(); itr++)
	{

		pm->removeConstraint(itr->second);
		//delete (&itr->first);
		delete (itr->second);
	}

}

Cube::~Cube()
{

	delete fallShape;
	delete collisionShape;
	delete fallMotionState;
	delete rigidBody;
	//rigidBody = nullptr;
	hinges.clear();
	joints.clear();

}
