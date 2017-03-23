#include "stdafx.h"
#include "Cube.h"

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

}

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
	//std::cout << position.y << std::endl;
	this->position = position;
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

void Cube::setUpPhysicsCube()
{
	//TODO: delete fall shape at the end of game loop
	btCollisionShape* fallShape = new btBoxShape(btVector3(width, height, depth));
	btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), Util::convertToBtVector3(position)));
	btVector3 fallInertia(0, 0, 0);
	if(mass != 0) fallShape->calculateLocalInertia(mass, fallInertia);

	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
	rigidBody = new btRigidBody(fallRigidBodyCI);
}


btRigidBody* Cube::getRigidBody()
{
	return rigidBody;
}

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
	

	// set constraint limit
	const btScalar low = -PI;
	const btScalar high = PI;
	hingeConstraint->setLimit(low, high);
	std::cout << pm << std::endl;
	pm->addNewConstraint(hingeConstraint, notCollision);

	//Add to hinge array
	hinges[name] = hingeConstraint;

	
}

btHingeConstraint * Cube::getHinge(std::string name)
{
	return hinges[name];
}



Cube::~Cube()
{
}
