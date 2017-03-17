#include "stdafx.h"
#include "PhysicsManager.h"

PhysicsManager::PhysicsManager()
{
}

void PhysicsManager::initPhysics()
{
	broadphase = new btDbvtBroadphase();

	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

}

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
	dynamicsWorld->addConstraint(hingeConstraint,
		isDisableCollisionsBetweenLinkedBodies);
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
