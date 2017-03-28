/**
Creature.cpp
Purpose: Sets up, renders and updates a complete hardcoded creature.

@author Sjur Barndon, Jonas S�rsdal
@version 1.0 23.03.2017
*/


#include "stdafx.h"
#include "Creature.h"

/**
	Constructor for the creature. Sets up a complete hardcoded 
	creature (a hip and two legs with thighs, shins and feet).
	Also links them together with mobile joints (hinges).
	TODO: Create this class customizable, not hardcoded.

	@param pm pointer to the PhysicsManager class so that the limbs and joints can be added to the world simulation.
*/
Creature::Creature(PhysicsManager* pm) 
{
	//Limbs:
	hips = new Cube(glm::vec3(6.0f, 11.0f, 1.0f), glm::vec3(0.9f, 0.1f, 0.1f), 2.0f, 1.0f, 0.5f, 10);
	
	rightThigh = new Cube(glm::vec3(4.5f, 8.0f, 1.0f), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.5f, 10);
	rightShin = new Cube(glm::vec3(4.5f, 4.0f, 1.0f), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.5f, 10);
	rightFoot = new Cube(glm::vec3(4.5f, 1.8f, 1.3f), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.3f, 1.0f, 5);
	
	leftThigh = new Cube(glm::vec3(7.5f, 8.0f, 1.0f), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.5f, 10);
	leftShin = new Cube(glm::vec3(7.5f, 4.0f, 1.0f), glm::vec3(0.2f, 0.3f, 0.7f), 0.5f, 1.8f, 0.5f, 10);
	leftFoot = new Cube(glm::vec3(7.5f, 1.8f, 1.3f), glm::vec3(0.2f, 0.3f, 0.7f), 0.6f, 0.3f, 1.0f, 5);

	pm->addBody(hips->getRigidBody());

	pm->addBody(rightThigh->getRigidBody());
	pm->addBody(rightShin->getRigidBody());
	pm->addBody(rightFoot->getRigidBody());

	pm->addBody(leftThigh->getRigidBody());
	pm->addBody(leftShin->getRigidBody());
	pm->addBody(leftFoot->getRigidBody());

	//Hinges:
	bool noCol = false;
	hips->addHinge(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.5f, 3.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightThigh, noCol, pm, "rightHip");
	hips->addHinge(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-1.5f, 3.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftThigh, noCol, pm, "leftHip");

	rightThigh->addHinge(glm::vec3(0.0f, -2.0f, 0.0f), glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightShin, noCol, pm, "rightKnee");
	leftThigh->addHinge(glm::vec3(0.0f, -2.0f, 0.0f), glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftShin, noCol, pm, "leftKnee");

	rightShin->addHinge(glm::vec3(0.0f, -2.3f, 0.0f), glm::vec3(0.0f, 0.0f, 0.5f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), rightFoot, noCol, pm, "rightAnkle");
	leftShin->addHinge(glm::vec3(0.0f, -2.3f, 0.0f), glm::vec3(0.0f, 0.0f, 0.5f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), leftFoot, noCol, pm, "leftAnkle");


}

/**
	Calls the render function for each limb of the creature.

	@param shader takes the shader for rendering puposes.
*/
void Creature::render(Shader shader)
{
	hips->render(shader);

	rightThigh->render(shader);
	rightShin->render(shader);
	rightFoot->render(shader);

	leftThigh->render(shader);
	leftShin->render(shader);
	leftFoot->render(shader);
}

/**
	Updates the physics for each limb in the creature. 
	Also sets dampening and restitution.
*/
void Creature::updatePhysics()
{
	hips->getRigidBody()->setDamping(0, 0);
	hips->getRigidBody()->setRestitution(0);
	hips->updatePhysics();

	rightThigh->updatePhysics();
	rightShin->updatePhysics();
	rightFoot->updatePhysics();

	leftThigh->updatePhysics();
	leftShin->updatePhysics();
	leftFoot->updatePhysics();


}


Creature::~Creature()
{
	delete hips;
	delete rightThigh;
	delete rightShin;
	delete rightFoot;
	delete leftThigh;
	delete leftShin;
	delete leftFoot;
}
