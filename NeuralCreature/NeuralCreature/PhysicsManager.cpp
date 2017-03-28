/**
PhysicsManager.cpp
Purpose: Sets up and manages the physics simulation with the Bullet Library.

@author Sjur Barndon, Jonas S�rsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "PhysicsManager.h"


/**
	Initializes the physics library and sets up a few of the settings in the world.
*/
void PhysicsManager::initPhysics()
{
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

}

/**
	Steps to the next stage of the physics simulation.

	@param deltaTime
	@param steps Number of steps to take.
*/
void PhysicsManager::update(GLfloat deltaTime, int steps)
{
	dynamicsWorld->stepSimulation(deltaTime, steps);
}

btDiscreteDynamicsWorld * PhysicsManager::getDynamicsWorld()
{
	return dynamicsWorld;
}

void PhysicsManager::addBody(btRigidBody* rigidBody)
{
	dynamicsWorld->addRigidBody(rigidBody);
}

void PhysicsManager::addNewConstraint(btHingeConstraint * hingeConstraint, bool isDisableCollisionsBetweenLinkedBodies)
{
	dynamicsWorld->addConstraint(hingeConstraint, isDisableCollisionsBetweenLinkedBodies);
}

void PhysicsManager::removeBody(btRigidBody * rigidBody)
{
	dynamicsWorld->removeRigidBody(rigidBody);

}

PhysicsManager::~PhysicsManager()
{
	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;
}
