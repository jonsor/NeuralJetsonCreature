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

	double x = ((double)rand()) / ((double)RAND_MAX) / 2 -0.25;
	x /= 100;
	std::cout << "x: " << x << "\n";
	x = 0; 
	groundShape = new btStaticPlaneShape(btVector3(x, 1, x), 1);

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	groundRigidBody = new btRigidBody(groundRigidBodyCI);
	groundRigidBody->setFriction(2.0f);
	addBody(groundRigidBody, 2, 1);

	groundShape2 = new btStaticPlaneShape(btVector3(x, 1, x), 1.15);

	btDefaultMotionState* groundMotionState2 = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI2(0, groundMotionState2, groundShape2, btVector3(0, 0, 0));
	groundRigidBody2 = new btRigidBody(groundRigidBodyCI2);
	groundRigidBody2->setFriction(2.0f);
	groundRigidBody2->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	addBody(groundRigidBody2, 2, 1);

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
void PhysicsManager::addNewConstraint(btGeneric6DofConstraint* hingeConstraint, bool isDisableCollisionsBetweenLinkedBodies)
{
	dynamicsWorld->addConstraint(hingeConstraint, isDisableCollisionsBetweenLinkedBodies);
}


void PhysicsManager::removeBody(btRigidBody * rigidBody)
{
	dynamicsWorld->removeRigidBody(rigidBody);

}

void PhysicsManager::removeConstraint(btHingeConstraint* hingeConstraint)
{
	dynamicsWorld->removeConstraint(hingeConstraint);
}

void PhysicsManager::removeConstraint(btGeneric6DofConstraint* hingeConstraint)
{
	dynamicsWorld->removeConstraint(hingeConstraint);
}

btRigidBody* PhysicsManager::getGroundShape()
{
	return groundRigidBody2;
}


void PhysicsManager::reset()
{
	delete dynamicsWorld;
	delete broadphase;
	delete collisionConfiguration;
	delete dispatcher;
	delete solver;

	delete groundShape;
	delete groundMotionState;
	delete groundRigidBody;

	delete groundShape2;
	delete groundMotionState2;
	delete groundRigidBody2;


	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));


	double x = ((double)rand()) / ((double)RAND_MAX) / 2 - 0.25;
	//x /= 100;
	x = 0;
	groundShape = new btStaticPlaneShape(btVector3(x, 1, x), 1);

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	groundRigidBody = new btRigidBody(groundRigidBodyCI);
	groundRigidBody->setFriction(2.0f);
	addBody(groundRigidBody, 2, 1);


	groundShape2 = new btStaticPlaneShape(btVector3(x, 1, x), 1.15);

	btDefaultMotionState* groundMotionState2 = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI2(0, groundMotionState2, groundShape2, btVector3(0, 0, 0));
	groundRigidBody2 = new btRigidBody(groundRigidBodyCI2);
	groundRigidBody2->setFriction(2.0f);
	groundRigidBody2->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	addBody(groundRigidBody2, 2, 1);
}


PhysicsManager::~PhysicsManager()
{
	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;

	delete groundShape;
	delete groundMotionState;
	delete groundRigidBody;
}
