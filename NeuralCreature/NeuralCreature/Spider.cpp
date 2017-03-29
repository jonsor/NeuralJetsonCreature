/**
Spider.cpp
Purpose: Sets up, renders and updates a complete, hardcoded spider creature.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 29.03.2017
*/

#include "stdafx.h"
#include "Spider.h"

/**
	Constructor for the spider. Sets up a complete hardcoded
	spider (a body and six legs with thighs, shins).
	Also links them together with mobile joints (point to point joints).

	@param pm pointer to the PhysicsManager class so that the limbs and joints can be added to the world simulation.
*/
Spider::Spider(PhysicsManager* pm)
{
	//Limbs:
	body = new Cube(glm::vec3(-5.0f, 50.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 1.0f, 0.3f, 1.5f, 10);
	
	leftFrontUpper = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	leftFrontLower = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	leftMiddleUpper = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	leftMiddleLower = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	leftBackUpper = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	leftBackLower = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);

	rightFrontUpper = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	rightFrontLower = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	rightMiddleUpper = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	rightMiddleLower = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	rightBackUpper = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);
	rightBackLower = new Cube(glm::vec3(-5.0f, 10.0f, -5.0f), glm::vec3(0.1f, 0.9f, 0.9f), 0.1f, 1.0f, 0.1f, 10);

	pm->addBody(body->getRigidBody());
	
	pm->addBody(leftFrontUpper->getRigidBody());
	pm->addBody(leftFrontLower->getRigidBody());
	pm->addBody(leftMiddleUpper->getRigidBody());
	pm->addBody(leftMiddleLower->getRigidBody());
	pm->addBody(leftBackUpper->getRigidBody());
	pm->addBody(leftBackLower->getRigidBody());

	pm->addBody(rightFrontUpper->getRigidBody());
	pm->addBody(rightFrontLower->getRigidBody());
	pm->addBody(rightMiddleUpper->getRigidBody());
	pm->addBody(rightMiddleLower->getRigidBody());
	pm->addBody(rightBackUpper->getRigidBody());
	pm->addBody(rightBackLower->getRigidBody());

	//Hinges:
	bool noCol = true;

	body->addJoint(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(1.0f, 1.8f, 1.0f), leftFrontUpper, noCol, pm, "leftFront");
	body->addJoint(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(0.0f, 1.8f, 1.0f), leftMiddleUpper, noCol, pm, "leftMiddle");
	body->addJoint(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(-1.0f, 1.8f, 1.0f), leftBackUpper, noCol, pm, "leftBack");
	
	body->addJoint(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(1.0f, 1.8f, -1.0f), rightFrontUpper, noCol, pm, "rightFront");
	body->addJoint(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(0.0f, 1.8f, -1.0f), rightMiddleUpper, noCol, pm, "rightMiddle");
	body->addJoint(glm::vec3(0.0f, -1.8f, 0.0f), glm::vec3(-1.0f, 1.8f, -1.0f), rightBackUpper, noCol, pm, "rightBack");


	leftFrontUpper->addJoint(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), leftFrontLower, noCol, pm, "leftFront");
	leftMiddleUpper->addJoint(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), leftMiddleLower, noCol, pm, "leftMiddle");
	leftBackUpper->addJoint(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), leftBackLower, noCol, pm, "leftBack");

	rightFrontUpper->addJoint(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), rightFrontLower, noCol, pm, "rightFront");
	rightMiddleUpper->addJoint(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), rightMiddleLower, noCol, pm, "rightMiddle");
	rightBackUpper->addJoint(glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), rightBackLower, noCol, pm, "rightBack");
	
	
}

/**
Calls the render function for each limb of the creature.

@param shader takes the shader for rendering puposes.
*/
void Spider::render(Shader shader)
{
	body->render(shader);

	leftFrontUpper->render(shader);
	leftFrontLower->render(shader);
	leftMiddleUpper->render(shader);
	leftMiddleLower->render(shader);
	leftBackUpper->render(shader);
	leftBackLower->render(shader);

	rightFrontUpper->render(shader);
	rightFrontLower->render(shader);
	rightMiddleUpper->render(shader);
	rightMiddleLower->render(shader);
	rightBackUpper->render(shader);
	rightBackLower->render(shader);
}

/**
	Updates the physics for each limb in the spider.
*/
void Spider::updatePhysics()
{
	body->updatePhysics();
	body->getRigidBody()->setDamping(20, 10);
	body->getRigidBody()->setRestitution(20);

	leftFrontUpper->updatePhysics();
	leftFrontLower->updatePhysics();
	leftMiddleUpper->updatePhysics();
	leftMiddleLower->updatePhysics();
	leftBackUpper->updatePhysics();
	leftBackLower->updatePhysics();

	rightFrontUpper->updatePhysics();
	rightFrontLower->updatePhysics();
	rightMiddleUpper->updatePhysics();
	rightMiddleLower->updatePhysics();
	rightBackUpper->updatePhysics();
	rightBackLower->updatePhysics();
}

Cube * Spider::getBody()
{
	return body;
}

Cube * Spider::getLeftFrontUpper()
{
	return leftFrontUpper;
}

Cube * Spider::getLeftFrontLower()
{
	return leftFrontLower;
}

Cube * Spider::getLeftMiddleUpper()
{
	return leftMiddleUpper;
}

Cube * Spider::getLeftMiddleLower()
{
	return leftMiddleLower;
}

Cube * Spider::getLeftBackUpper()
{
	return leftBackUpper;
}

Cube * Spider::getLeftBackLower()
{
	return leftBackLower;
}

Cube * Spider::getRightFrontUpper()
{
	return rightFrontUpper;
}

Cube * Spider::getRightFrontLower()
{
	return rightFrontLower;
}

Cube * Spider::getRightMiddleUpper()
{
	return rightMiddleUpper;
}

Cube * Spider::getRightMiddleLower()
{
	return rightMiddleLower;
}

Cube * Spider::getRightBackUpper()
{
	return rightBackUpper;
}

Cube * Spider::getRightBackLower()
{
	return rightBackLower;
}


Spider::~Spider()
{
	delete body;
	delete leftFrontUpper;
	delete leftFrontLower;
	delete leftMiddleUpper;
	delete leftMiddleLower;
	delete leftBackUpper;
	delete leftBackLower;

	delete rightFrontUpper;
	delete rightFrontLower;
	delete rightMiddleUpper;
	delete rightMiddleLower;
	delete rightBackUpper;
	delete rightBackLower;
}
