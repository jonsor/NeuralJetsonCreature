#pragma once
#include "Box.h"
#include "NeuralNetwork.h"
#include "Util.h"
#include <thread>

class Creature
{

private:
	Box* hips;
	Box* rightThigh;
	Box* rightShin;
	Box* rightFoot;
	Box* leftThigh;
	Box* leftShin;
	Box* leftFoot;
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
	Creature(PhysicsManager* pm, glm::vec3 startPosition, std::default_random_engine &engine);
	void render(Shader shader);
	void updatePhysics();
	Box* getHips();
	Box* getRightThigh();
	Box* getRightShin();
	Box* getRightFoot();
	Box* getLeftThigh();
	Box* getLeftShin();
	Box* getLeftFoot();
	void calcCenterPosition();
	glm::vec3 getPosition();
	glm::vec3 getStartPosition();
	glm::vec3 getRelativePosition(Box* Box);
	double get2DAngle(Box * Box1, Box* Box2);
	void activate();
	std::vector<double> getAllAngles();
	std::vector<double> calculateInputs();
	std::vector<double> getAllAngularVelocities();
	void setAllTargetVelocities(std::vector<double> &resultVec);
	void setMaxMotorImpulses(double maxMotorImpulse);
	void createNeuralNetwork(std::vector<int> topology, std::default_random_engine &engine);
	void setNeuralNetwork(NeuralNetwork neuralNetwork);
	NeuralNetwork getNeuralNetwork();
	NeuralNetwork * getNN();
	void updateNeuralNetwork();
	void mutate(double mutationRate, double mutationChance, std::default_random_engine engine);
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

