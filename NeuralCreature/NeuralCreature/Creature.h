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

public:
	Creature(PhysicsManager* pm);
	void render(Shader shader);
	void updatePhysics();
	~Creature();
};

