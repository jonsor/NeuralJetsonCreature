#pragma once

//Additional includes
#include <iostream>
#include <GL/glew.h>
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

	btCollisionShape* groundShape;
	btDefaultMotionState* groundMotionState;
	btRigidBody* groundRigidBody;

public:
	void initPhysics();
	void update(GLfloat deltaTime, int steps);
	btDiscreteDynamicsWorld* getDynamicsWorld();
	void addBody(btRigidBody* rigidBody);
	void addBody(btRigidBody* rigidBody, int id, int collidesWith);
	void addNewConstraint(btHingeConstraint* hingeConstraint, bool isDisableCollisionsBetweenLinkedBodies);
	void addNewConstraint(btPoint2PointConstraint* jointConstraint, bool isDisableCollisionsBetweenLinkedBodies);
	void removeBody(btRigidBody* rigidBody);
	void removeConstraint(btHingeConstraint * hingeConstraint);
	void reset();
	~PhysicsManager();
};

