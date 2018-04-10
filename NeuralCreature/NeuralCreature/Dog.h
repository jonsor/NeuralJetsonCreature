#pragma once
#include "Box.h"
#include "RecurrentNeuralNetwork.h"
#include "Util.h"
#include <thread>
#include "NEATNetwork.h"
#include "Creature.h"

class Dog : public Creature
{

private:
	Box* body;

	Box* frontRightTigh;
	Box* frontRightShin;

	Box* frontLeftTigh;
	Box* frontLeftShin;

	Box* backRightTigh;
	Box* backRightShin;

	Box* backLeftTigh;
	Box* backLeftShin;

	const double PI = 3.141592653589793238463;
	glm::vec3 centerPosition;
	glm::vec3 m_startPosition;
	glm::vec3 m_previousPosition;
	glm::vec3 m_targetPosition;
	double maxDistanceToTarget;
	std::vector<double> maxMinAngles;
	std::vector<double> memoryNeurons;
	RecurrentNeuralNetwork m_neuralNetwork;
	NEATNetwork m_neatNeuralNetwork;
	std::vector<double> resultVec;
	int numberOfSteps;
	char lastFootThatStepped;

	double previousRightFootHeight;
	double previousLeftFootHeight;
	double m_fitness;
	double average = 0;
	double avgSize = 0;
	double m_maxHeight;
	double timeOnGround = 0;
	double timeOnTwoLegs = 0;
	int numTimesLegsCrossed = 0;
	bool reachedUpperR = false;
	bool reachedUpperL = false;
	double averageFeetStartPos;
	bool m_shouldUpdate;
	double rotationAmount = 0;
	double timeAlive = 0;
	double noMovementPenalty;
	double m_totalSpeed = 0;
	double jointsAtLimitPenalty = 0;
	bool targetReached = false;
	double tighMovement = 0;
	std::vector<double> stepState;
	std::vector<double> inputAngles;
	std::vector<double> prevInputAngles;
	int m_id = -1;
public:
	Dog(PhysicsManager* pm, glm::vec3 startPosition, std::default_random_engine &engine);
	void render(Shader shader);
	void updatePhysics();
	void setId(int id);
	int getId();
	Box* getBody();

	Box* getFrontRightTigh();
	Box* getFrontRightShin();

	Box* getFrontLeftTigh();
	Box* getFrontLeftShin();

	Box* getBackRightTigh();
	Box* getBackRightShin();

	Box* getBackLeftTigh();
	Box* getBackLeftShin();

	glm::vec3 getPositionOfBody();
	glm::vec3 getCenterPosition();
	glm::vec3 getStartPosition();
	void activate();
	void createNeuralNetwork(std::vector<int> topology, std::default_random_engine &engine);
	void setNEATNeuralNetwork(NEATNetwork neatNeuralNetwork);
	NEATNetwork getNEATNeuralNetwork();
	void setNeuralNetwork(RecurrentNeuralNetwork neuralNetwork);
	RecurrentNeuralNetwork getNeuralNetwork();
	RecurrentNeuralNetwork * getNN();
	void updateNeuralNetwork();
	std::vector<double> calculateInputs();
	void setAllTargetVelocities(std::vector<double> &resultVec);
	std::vector<double> getAllAngularVelocities();
	void getAllMaxMinAngles();
	std::vector<double> getAllAngles();
	void mutate(double mutationRate, double mutationChance, std::default_random_engine engine);
	void reset();
	void setFitness(double fitness);
	double getFitness();
	double getHeight();
	void removeBodies(PhysicsManager * pm);
	void removeConstraints(PhysicsManager * pm);
	void setColor(glm::vec3 color);
	void incrementToAverageHeight();
	double getAverageHeight();
	double getMaxHeight();
	void updateMaxHeight();
	double getCenterOfMassHeight();

	double getTimeOnGround();
	void setTimeOnGround(double time);

	int getNumerOfSteps();
	double getDistanceFromHips(Box * box);
	void setShouldUpdate(bool update);
	bool shouldUpdate();

	void checkRotation();
	double getRotationAmount();
	double getTimeAlive();
	void checkIfMoving();
	double getNoMovementPenalty();
	void calculateSpeed();
	double getTotalSpeed();
	double getDistanceWalked();
	void setMaxMotorImpulses(double maxMotorImpulse);

	bool noMovement(int currentStep);
	void checkIfJointsAtLimit();
	double getJointsAtlimitPenalty();
	std::vector<double> getRelativePositions();

	btVector3 getRelativePosition(Box * limb);

	glm::vec3 getTargetPosition();
	void setTargetPosition(glm::vec3 target);
	void checkIfTargetReached();
	bool isTargetReached();
	void disableSimulation();
	void calculateTighMovement();
	double getTighMovement();
	~Dog();
};

