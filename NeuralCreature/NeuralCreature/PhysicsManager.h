#pragma once

//Additional includes
#include <iostream>
#include "Cube.h"
#include <vector>
#include <math.h>
#include <btBulletDynamicsCommon.h>

#define _USE_MATH_DEFINES

class PhysicsManager
{
private:
	btDiscreteDynamicsWorld* dynamicsWorld;
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
public:
	PhysicsManager();
	void initPhysics();
	void update(GLfloat deltaTime, int steps);
	btDiscreteDynamicsWorld* getDynamicsWorld();
	void addBody(btRigidBody* rigidBody);
	void addNewConstraint(btHingeConstraint* hingeConstraint, bool isDisableCollisionsBetweenLinkedBodies);
	void removeBody(btRigidBody* rigidBody);
	~PhysicsManager();
};

