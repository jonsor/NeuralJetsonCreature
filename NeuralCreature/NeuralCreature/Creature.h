#pragma once
#include "Cube.h"


class Creature
{

private:
	Cube* chest;
	Cube* hips;
	Cube* rightThigh;
	Cube* rightShin;
	Cube* rightFoot;
	Cube* leftThigh;
	Cube* leftShin;
	Cube* leftFoot;
	const double PI = 3.141592653589793238463;
	glm::vec3 centerPosition;

public:
	Creature(PhysicsManager* pm);
	void render(Shader shader);
	void updatePhysics();
	Cube* getChest();
	Cube* getHips();
	Cube* getRightThigh();
	Cube* getRightShin();
	Cube* getRightFoot();
	Cube* getLeftThigh();
	Cube* getLeftShin();
	Cube* getLeftFoot();
	void getCenterPosition();
	glm::vec3 getRelativePosition(Cube* cube);
	double get2DAngle(Cube * cube1, Cube* cube2);
	void activate();
	~Creature();
};

