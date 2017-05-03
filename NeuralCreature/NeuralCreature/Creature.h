#pragma once
#include "Cube.h"
#include "NeuralNetwork.h"
#include "Util.h"

class Creature
{

private:
	//Cube* chest;
	Cube* hips;
	Cube* rightThigh;
	Cube* rightShin;
	Cube* rightFoot;
	Cube* leftThigh;
	Cube* leftShin;
	Cube* leftFoot;
	const double PI = 3.141592653589793238463;
	glm::vec3 centerPosition;
	glm::vec3 m_startPosition;

	NeuralNetwork* m_neuralNetwork;
	std::vector<double> resultVec;
	int yo = 0;


public:
	Creature(PhysicsManager* pm, glm::vec3 startPosition);
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
	void calcCenterPosition();
	glm::vec3 getRelativePosition(Cube* cube);
	double get2DAngle(Cube * cube1, Cube* cube2);
	void activate();
	std::vector<double> getAllAngles();
	std::vector<double> calculateInputs();
	std::vector<double> getAllAngularVelocities();
	void setAllTargetVelocities(std::vector<double> &resultVec);
	void setMaxMotorImpulses(double maxMotorImpulse);
	void createNeuralNetwork(std::vector<int> topology);
	void setNeuralNetwork(NeuralNetwork neuralNetwork);
	void updateNeuralNetwork();
	~Creature();
};

