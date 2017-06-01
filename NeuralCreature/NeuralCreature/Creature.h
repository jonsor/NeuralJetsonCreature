#pragma once
#include "Cube.h"
#include "NeuralNetwork.h"
#include "Util.h"
#include <thread>

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

	NeuralNetwork m_neuralNetwork;
	std::vector<double> resultVec;
	int yo = 0;

	double m_fitness;
	double average = 0;
	double avgSize = 0;
	double maxHeight = 0;
	double timeOnGround = 0;
	double timeOnTwoLegs = 0;
	int numTimesLegsCrossed = 0;
	bool reachedUpperR = false;
	bool reachedUpperL = false;

public:
	//static struct LessThanByFitness
	//{
	//	bool operator()(const Creature* lhs, const Creature* rhs) const
	//	{
	//		return lhs->m_fitness < rhs->m_fitness;
	//	}
	//};

	Creature(PhysicsManager* pm, glm::vec3 startPosition);
	void render(Shader shader);
	void updatePhysics();
	Cube* getHips();
	Cube* getRightThigh();
	Cube* getRightShin();
	Cube* getRightFoot();
	Cube* getLeftThigh();
	Cube* getLeftShin();
	Cube* getLeftFoot();
	void calcCenterPosition();
	glm::vec3 getPosition();
	glm::vec3 getStartPosition();
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
	NeuralNetwork getNeuralNetwork();
	void updateNeuralNetwork();
	void mutate(double mutationRate);
	void reset();
	void setFitness(double fitness);
	double getFitness();
	double getHeight();
	void removeBodies(PhysicsManager * pm);
	void removeConstraints(PhysicsManager * pm);
	void setColor(glm::vec3 color);
	void incrementToAverage();
	double getAverageHeight();
	double getMaxHeight();
	void updateMaxHeight(double height);

	double getTimeOnGround();
	void setTimeOnGround(double time);

	double getTimeOnTwoLegs();
	void setTimeOnTwoLegs(double time);
	void checkIfLegsCrossed();
	double getNumTimesCrossed();
	~Creature();
};

