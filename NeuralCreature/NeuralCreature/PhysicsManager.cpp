/**
PhysicsManager.cpp
Purpose: Sets up and manages the physics simulation with the Bullet Library.

@author Sjur Barndon, Jonas Sørsdal
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
	dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));//-9.81f

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

void PhysicsManager::addBody(btRigidBody* rigidBody, int id, int collidesWith)
{

	dynamicsWorld->addRigidBody(rigidBody, id, collidesWith);

}


void PhysicsManager::addNewConstraint(btHingeConstraint * hingeConstraint, bool isDisableCollisionsBetweenLinkedBodies)
{
	dynamicsWorld->addConstraint(hingeConstraint, isDisableCollisionsBetweenLinkedBodies);
}
void PhysicsManager::addNewConstraint(btPoint2PointConstraint* jointConstraint, bool isDisableCollisionsBetweenLinkedBodies)
{
	dynamicsWorld->addConstraint(jointConstraint, isDisableCollisionsBetweenLinkedBodies);
}


void PhysicsManager::removeBody(btRigidBody * rigidBody)
{
	dynamicsWorld->removeRigidBody(rigidBody);

}

void PhysicsManager::removeConstraint(btHingeConstraint* hingeConstraint)
{
	dynamicsWorld->removeConstraint(hingeConstraint);
}

void PhysicsManager::reset()
{
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	groundRigidBody->setFriction(2.0f);
	addBody(groundRigidBody, 2, 1);
}

PhysicsManager::~PhysicsManager()
{
	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;
}
