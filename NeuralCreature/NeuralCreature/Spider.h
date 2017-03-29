#pragma once
#include "Cube.h"


class Spider
{
private:
	Cube* body;
	Cube* leftFrontUpper;
	Cube* leftFrontLower;
	Cube* leftMiddleUpper;
	Cube* leftMiddleLower;
	Cube* leftBackUpper;
	Cube* leftBackLower;
	
	Cube* rightFrontUpper;
	Cube* rightFrontLower;
	Cube* rightMiddleUpper;
	Cube* rightMiddleLower;
	Cube* rightBackUpper;
	Cube* rightBackLower;

public:
	Spider(PhysicsManager* pm);
	void render(Shader shader);
	void updatePhysics();
	Cube* getBody();
	Cube* getLeftFrontUpper();
	Cube* getLeftFrontLower();
	Cube* getLeftMiddleUpper();
	Cube* getLeftMiddleLower();
	Cube* getLeftBackUpper();
	Cube* getLeftBackLower();

	Cube* getRightFrontUpper();
	Cube* getRightFrontLower();
	Cube* getRightMiddleUpper();
	Cube* getRightMiddleLower();
	Cube* getRightBackUpper();
	Cube* getRightBackLower();
	~Spider();
};

