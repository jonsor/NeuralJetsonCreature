#pragma once
#include "Box.h"
#include "RecurrentNeuralNetwork.h"
#include "Util.h"
#include <thread>

class Biped
{

private:
	Box* lowerBody;
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
	glm::vec3 m_previousPosition;
	std::vector<double> maxMinAngles;
	std::vector<double> memoryNeurons;
	RecurrentNeuralNetwork m_neuralNetwork;
	std::vector<double> resultVec;
	int numberOfSteps;
	char lastFootThatStepped;

	glm::vec3 m_targetPosition;
	double maxDistanceToTarget;

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
	bool targetReached;

	std::vector<double> stepState;
public:
	Biped(PhysicsManager* pm, glm::vec3 startPosition, std::default_random_engine &engine);
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
	void activate();
	void getAllMaxMinAngles();
	void getAllMaxMinAngles2();
	std::vector<double> getAllAngles();
	std::vector<double> calculateInputs();
	std::vector<double> getAllAngularVelocities();
	void setAllTargetVelocities(std::vector<double> &resultVec);
	btScalar computeTargetVelocity(btScalar hingeAngle, btScalar targetAngle, double dt);
	void setMaxMotorImpulses(double maxMotorImpulse);
	void quaternionToEuler(btQuaternion & quat, double & rotx, double & roty, double & rotz);
	void createNeuralNetwork(std::vector<int> topology, std::default_random_engine &engine);
	void setNeuralNetwork(RecurrentNeuralNetwork neuralNetwork);
	RecurrentNeuralNetwork getNeuralNetwork();
	RecurrentNeuralNetwork * getNN();
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
	bool leftFootMovingDownward();
	bool rightFootMovingDownward();
	int getNumerOfSteps();
	double getAverageFeetPosition();
	double getAverageFeetStartPos();
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

	glm::vec3 getPositionOfBody();
	glm::vec3 getTargetPosition();
	void setTargetPosition(glm::vec3 target);
	void checkIfTargetReached();
	bool isTargetReached();
	~Biped();
};

