#pragma once
#include "Cube.h"


class Creature
{

private:
	Cube* hips;
	Cube* rightThigh;
	Cube* rightShin;
	Cube* rightFoot;
	Cube* leftThigh;
	Cube* leftShin;
	Cube* leftFoot;
	const double PI = 3.141592653589793238463;

public:
	Creature(PhysicsManager* pm);
	void render(Shader shader);
	void updatePhysics();
	Cube* getHips();
	Cube* getRightThigh();
	Cube* getRightShin();
	Cube* getRightFoot();
	Cube* getLeftThigh();
	Cube* getLeftShin();
	Cube* getLeftFoot();
	~Creature();
};

