/**
Box.cpp
Purpose: Creates Boxs to use in the world. Also renders, updates physics and creates hinges to attach other Boxs.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "Box.h"

/**
	Creates a new OpenGL Box and initializes the physics.

	@param position Vector position of the Box.
	@param color Vector color of the Box.
	@param width The width of the Box.
	@param height The heigth of the Box.
	@param depth The depth of the Box.
	@param mass The Mass of the Box.
*/
Box::Box(glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth, btScalar mass, std::string shapetype) : position(position), color(color), mass(mass), width(width), height(height), depth(depth), m_shapeType(shapetype)
{
	// Boxs
	GLfloat BoxVertices[] = {
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
	
	//Box STUFF
	glGenVertexArrays(1, &BoxVAO);
	glGenBuffers(1, &BoxVBO);

	glBindVertexArray(BoxVAO);

	glBindBuffer(GL_ARRAY_BUFFER, BoxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(BoxVertices), BoxVertices, GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	// Normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0); // Unbind VAO
	glDeleteBuffers(1, &BoxVBO);

	if (shapetype != BOX_SHAPE && shapetype != CAPSULE_SHAPE) {
		throw std::invalid_argument("invalid shape");
	}

	//mass = 1;
	//Set up physics
	Box::setUpPhysicsBox();

	rigidBody->getMotionState()->getWorldTransform(startPos);
	groundCollision = false;

	previousPosition = position;

}

Box::Box(glm::vec3 position, glm::vec3 color, GLfloat width, GLfloat height, GLfloat depth, btScalar mass) : position(position), color(color), mass(mass), width(width), height(height), depth(depth)
{
	// Boxs
	GLfloat BoxVertices[] = {
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

	//Box STUFF
	glGenVertexArrays(1, &BoxVAO);
	glBindVertexArray(BoxVAO);

	glGenBuffers(1, &BoxVBO);
	glBindBuffer(GL_ARRAY_BUFFER, BoxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(BoxVertices), BoxVertices, GL_STATIC_DRAW);

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	// Normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindVertexArray(0); // Unbind VAO
	glDeleteBuffers(1, &BoxVBO);

						  //mass = 1;
						  //Set up physics
	m_shapeType = BOX_SHAPE;
	Box::setUpPhysicsBox();

	rigidBody->getMotionState()->getWorldTransform(startPos);
	groundCollision = false;

	previousPosition = position;
}

/**
	Renders the Box with OpenGL.

	@param shader Shader object for rendering.
*/
void Box::render(Shader shader)
{
	shader.use();
	glBindVertexArray(BoxVAO);
	GLint objectColorLoc = glGetUniformLocation(shader.program, "objectColor");
	glUniform3f(objectColorLoc, color.x, color.y, color.z);
	glm::mat4 model;
	model = glm::translate(model, position);

	//Hacky roation fix - doesnt render if axis is equal to zero
	if (axisOfRotation.x == 0 || axisOfRotation.y == 0 || axisOfRotation.z == 0) {
		axisOfRotation.x += 0.000000001f;
		axisOfRotation.y += 0.000000001f;
		axisOfRotation.z += 0.000000001f;
	}
	model = glm::rotate(model, angle, axisOfRotation);
	GLint modelLoc = glGetUniformLocation(shader.program, "model"); 

	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model)); 
	glDrawArrays(GL_TRIANGLES, 0, 36);

	glBindVertexArray(0);
}

void Box::setColor(glm::vec3 color)
{
	this->color = color;
}

glm::vec3 Box::getColor()
{
	return color;
}

void Box::setPosition(glm::vec3 position)
{
	this->position = position;
}

void Box::reset() 
{
	rigidBody->getMotionState()->setWorldTransform(startPos);
	rigidBody->setWorldTransform(startPos);
	rigidBody->clearForces();
}
glm::vec3 Box::getPreviousPosition()
{
	return previousPosition;
}
double Box::getCollisionImpulse()
{
	return m_collisionImpulse;
}
void Box::setCollisionImpulse(double collisionImpulse)
{
	m_collisionImpulse = collisionImpulse;
}
glm::vec3 Box::getPosition()
{
	return position;
}

void Box::setRotation(GLfloat angle, glm::vec3 axisOfRotation)
{
	this->angle = angle;
	this->axisOfRotation = axisOfRotation;
}

/**
	Sets up physics for the Box.
	Does not add it to the physics world.
*/
void Box::setUpPhysicsBox()
{

	if (m_shapeType == CAPSULE_SHAPE) {
		fallShape = new btCapsuleShape(width, height);
	}
	else {
		fallShape = new btBoxShape(btVector3(width, height, depth));
	}

	fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), Util::convertToBtVector3(position)));
	btVector3 fallInertia(0, 0, 0);
	if(mass != 0) fallShape->calculateLocalInertia(mass, fallInertia);

	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
	rigidBody = new btRigidBody(fallRigidBodyCI);

	rigidBody->setRestitution(0.05);
	rigidBody->setDamping(0.05, 0.85);
	rigidBody->setDeactivationTime(0.8);
	rigidBody->setSleepingThresholds(0.5f, 0.5f);

	//Default damping: 0.1 -- stiffness: 10000
	//rigidBody->setContactStiffnessAndDamping(10000.0f, 0.1f);
	//std::cout << "stiffness: " << rigidBody->getContactStiffness() << "\n";
	//std::cout << "damping: " << rigidBody->getContactDamping() << "\n";
}

//void Box::setUpPhysicsBox()
//{
//	//TODO: delete fall shape at the end of game loop
//	fallShape = new btCapsuleShape(width, height);
//	fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), Util::convertToBtVector3(position)));
//	btVector3 fallInertia(0, 0, 0);
//	if (mass != 0) fallShape->calculateLocalInertia(mass, fallInertia);
//
//	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
//	rigidBody = new btRigidBody(fallRigidBodyCI);
//
//	rigidBody->setRestitution(0.05);
//	rigidBody->setDamping(0.05, 0.85);
//	rigidBody->setDeactivationTime(0.8);
//	rigidBody->setSleepingThresholds(0.5f, 0.5f);
//
//	//Default damping: 0.1 -- stiffness: 10000
//	//rigidBody->setContactStiffnessAndDamping(10000.0f, 0.1f);
//	//std::cout << "stiffness: " << rigidBody->getContactStiffness() << "\n";
//	//std::cout << "damping: " << rigidBody->getContactDamping() << "\n";
//}


btRigidBody* Box::getRigidBody()
{
	return rigidBody;
}

/**
	Calculates and applies the next phyical step for the Box.
	Should be called in the main loop.
*/
void Box::updatePhysics()
{
	previousPosition = position;
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

	prevGroundCollision = groundCollision;
}

/**
	Adds a new hinge to this Box that attaches BoxB.

	@param pivotA
	@param pivotB
	@param axisA The vector direction of rotation of this Box in relation to BoxB.
	@param axisB The vector direction of rotation of BoxB in relation to this Box.
	@param BoxB A pointer to the Box that you want to attach to this one.
	@param notCollision Wether or not the two joint Boxs should be able to collide.
	@param pm A pointer to the Pysics Manager to add the hinges to the world.
	@param name The key for hinge storage.
*/
//void Box::addHinge(glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Box* BoxB, bool notCollision, PhysicsManager* pm, std::string name)
//{
//	bool useReferenceFrameA = false;
//	btHingeConstraint* hingeConstraint = new btHingeConstraint(
//		*rigidBody,
//		*BoxB->getRigidBody(),
//		Util::convertToBtVector3(pivotA),
//		Util::convertToBtVector3(pivotB),
//		Util::convertToBtVector3(axisA),
//		Util::convertToBtVector3(axisB),
//		useReferenceFrameA
//	);
//	
//	//Set constraint limit
//	const btScalar low = -PI;
//	const btScalar high = PI;
//	hingeConstraint->setLimit(low, high);
//	pm->addNewConstraint(hingeConstraint, notCollision);
//
//	//Add to hinge array
//	hinges[name] = hingeConstraint;
//	hingeConstraint = nullptr;
//}

/**
	Adds a new hinge to this Box that attaches BoxB.

	@param pivotA
	@param pivotB
	@param axisA The vector direction of rotation of this Box in relation to BoxB.
	@param axisB The vector direction of rotation of BoxB in relation to this Box.
	@param BoxB A pointer to the Box that you want to attach to this one.
	@param notCollision Wether or not the two joint Boxs should be able to collide.
	@param minAngle The minimum angle of rotation on the hinge.
	@param maxAngle The maximum angle of rotation on the hinge.
	@param pm A pointer to the Pysics Manager to add the hinges to the world.
	@param name The key for hinge storage.
*/
void Box::addHinge(glm::vec3 pivotA, glm::vec3 pivotB, glm::vec3 axisA, glm::vec3 axisB, Box* BoxB, bool notCollision, const btScalar minAngle, const btScalar maxAngle, PhysicsManager * pm, std::string name)
{
	bool useReferenceFrameA = false;
	btHingeConstraint* hingeConstraint = new btHingeConstraint(
		*rigidBody,
		*BoxB->getRigidBody(),
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

void Box::addDOFConstraint(Box * cubeB, bool notCollision, btScalar xOffset, PhysicsManager * pm, std::string name)
{

	btTransform frameA = btTransform::getIdentity();
	frameA.getBasis().setEulerZYX(0, 0, PI / 2);
	//frameA.setOrigin(btVector3(btScalar(xOffset), btScalar(-2.2), btScalar(0.)));
	frameA.setOrigin(btVector3(xOffset, -getHeight()/2, 0.f));

	btTransform frameB = btTransform::getIdentity();
	frameB.getBasis().setEulerZYX(0, 0, PI / 2);
	//frameB.setOrigin(btVector3(btScalar(0.), btScalar(2.2), btScalar(0.)));
	frameB.setOrigin(btVector3(0.f, cubeB->getHeight(), 0.f));

	btGeneric6DofConstraint* dofConstraint = new btGeneric6DofConstraint(*rigidBody,
		*cubeB->getRigidBody(), frameA, frameB, true);

	dofConstraint->setLinearLowerLimit(btVector3(0.f, 0.f, 0.f));
	dofConstraint->setLinearUpperLimit(btVector3(0.f, 0.f, 0.f));

	// x = rotasjon, y = fremover/bakover, z = side til side
	if (name == "rightHip") {
		dofConstraint->setAngularLowerLimit(btVector3(-PI / 8, -PI / 8, -0.1f));
		dofConstraint->setAngularUpperLimit(btVector3(PI / 8, PI / 2.5, PI / 6));
	}
	else {
		dofConstraint->setAngularLowerLimit(btVector3(-PI / 8, -PI / 8, -PI / 6));
		dofConstraint->setAngularUpperLimit(btVector3(PI / 8, PI / 2.5, 0.1f));
	}

	for (int i = 0; i<3; i++)
	{
		dofConstraint->getRotationalLimitMotor(i)->m_maxLimitForce = SIMD_INFINITY;
		dofConstraint->getRotationalLimitMotor(i)->m_stopERP = btScalar(1.0);
		dofConstraint->getRotationalLimitMotor(i)->m_limitSoftness = btScalar(1.0);
	}

	pm->addNewConstraint(dofConstraint, notCollision);

	// 0 = spin, 1 = fremover/bakover, 2 = side til side
	//int v = 0;
	//dofConstraint->getRotationalLimitMotor(v)->m_enableMotor = true;
	//dofConstraint->getRotationalLimitMotor(v)->m_maxMotorForce = 100.0f;
	//dofConstraint->getRotationalLimitMotor(v)->m_targetVelocity = -10.0f;

	//Add to hinge array
	dofConstraints[name] = dofConstraint;
	dofConstraint = nullptr;
}

void Box::addDOFConstraintDog(Box * cubeB, bool notCollision, btScalar xOffset, btScalar zOffset, PhysicsManager * pm, std::string name)
{

	btTransform frameA = btTransform::getIdentity();
	frameA.getBasis().setEulerZYX(0, 0, PI / 2);
	frameA.setOrigin(btVector3(btScalar(xOffset), btScalar(-0.1), btScalar(zOffset)));


	btTransform frameB = btTransform::getIdentity();
	frameB.getBasis().setEulerZYX(0, 0, PI / 2);
	frameB.setOrigin(btVector3(btScalar(0.), btScalar(1.4), btScalar(0.f)));

	btGeneric6DofConstraint* dofConstraint = new btGeneric6DofConstraint(*rigidBody,
		*cubeB->getRigidBody(), frameA, frameB, true);

	dofConstraint->setLinearLowerLimit(btVector3(0.f, 0.f, 0.f));
	dofConstraint->setLinearUpperLimit(btVector3(0.f, 0.f, 0.f));

	// x = rotasjon, y = fremover/bakover, z = side til side
	if (name == "rightHip") {
		dofConstraint->setAngularLowerLimit(btVector3(-PI / 8, -PI / 8, -0.1f));
		dofConstraint->setAngularUpperLimit(btVector3(PI / 8, PI / 2.5, PI / 6));
	}
	else {
		dofConstraint->setAngularLowerLimit(btVector3(-PI / 8, -PI / 8, -PI / 6));
		dofConstraint->setAngularUpperLimit(btVector3(PI / 8, PI / 2.5, 0.1f));
	}

	for (int i = 0; i<3; i++)
	{
		dofConstraint->getRotationalLimitMotor(i)->m_maxLimitForce = SIMD_INFINITY;
		dofConstraint->getRotationalLimitMotor(i)->m_stopERP = btScalar(1.0);
		dofConstraint->getRotationalLimitMotor(i)->m_limitSoftness = btScalar(1.0);
	}

	pm->addNewConstraint(dofConstraint, notCollision);

	// 0 = spin, 1 = fremover/bakover, 2 = side til side
	//int v = 0;
	//dofConstraint->getRotationalLimitMotor(v)->m_enableMotor = true;
	//dofConstraint->getRotationalLimitMotor(v)->m_maxMotorForce = 100.0f;
	//dofConstraint->getRotationalLimitMotor(v)->m_targetVelocity = -10.0f;

	//Add to hinge array
	dofConstraints[name] = dofConstraint;
	dofConstraint = nullptr;
}


/**
	Adds a new hinge to this Box that attaches BoxB.

	@param pivotA
	@param pivotB
	@param BoxB A pointer to the Box that you want to attach to this one.
	@param notCollision Wether or not the two joint Boxs should be able to collide.
	@param pm A pointer to the Pysics Manager to add the hinges to the world.
	@param name The key for joint storage.
*/
void Box::addJoint(glm::vec3 pivotA, glm::vec3 pivotB, Box* BoxB, bool notCollision, PhysicsManager* pm, std::string name)
{
	btPoint2PointConstraint* jointConstraint = new btPoint2PointConstraint(*rigidBody, *BoxB->getRigidBody(), Util::convertToBtVector3(pivotA), Util::convertToBtVector3(pivotB));
	pm->addNewConstraint(jointConstraint, notCollision);
	joints[name] = jointConstraint;
	jointConstraint = nullptr;

}

btHingeConstraint* Box::getHinge(std::string name)
{
	return hinges[name];
}

btGeneric6DofConstraint * Box::getdofConstraint(std::string name)
{
	return dofConstraints[name];
}

btPoint2PointConstraint* Box::getJoint(std::string name)
{
	return joints[name];
}

/**
	Sets the maximum and minimum angle for the hinge, should be within +-PI.

	@param name Key name of the hinge.
	@param minAngle min angle.
	@param maxAngle max angle.
*/
void Box::setHingeAngles(std::string name, const btScalar minAngle, const btScalar maxAngle)
{
	hinges[name]->setLimit(minAngle, maxAngle);
}

GLfloat Box::getWidth()
{
	return width;
}

GLfloat Box::getHeight()
{
	return height;
}

GLfloat Box::getDepth()
{
	return depth;
	
}

void Box::setCollidingWithGround(bool colliding) {

	//prevGroundCollision = groundCollision;
	groundCollision = colliding;
}

bool Box::isCollidingWithGround() {
	return groundCollision;
}

bool Box::isPrevStepCollidingWithGround() {
	return prevGroundCollision;
}

void Box::remove(PhysicsManager * pm)
{
	pm->removeBody(rigidBody);
}

bool alreadyRemoved = false;
void Box::removeConstraint(PhysicsManager * pm)
{
	if (!alreadyRemoved) {
		for (std::map<std::string, btHingeConstraint*>::iterator itr = hinges.begin(); itr != hinges.end(); itr++)
		{
			pm->removeConstraint(itr->second);
			delete (itr->second);
		}

		for (std::map<std::string, btGeneric6DofConstraint*>::iterator itr = dofConstraints.begin(); itr != dofConstraints.end(); itr++)
		{
			pm->removeConstraint(itr->second);
			delete (itr->second);
		}
		//alreadyRemoved = true;
	}

}

Box::~Box()
{
	delete fallShape;
	delete collisionShape;
	delete fallMotionState;
	delete rigidBody;
	hinges.clear();
	joints.clear();
	dofConstraints.clear();

	glDeleteVertexArrays(1, &BoxVAO);
	glDeleteBuffers(1, &BoxVBO);

}
