#pragma once
#include "Box.h"
#include "NeuralNetwork.h"


class Spider
{
private:
	Box* body;

	Box* leftFrontUpper;
	Box* leftFrontLower;

	Box* rightFrontUpper;
	Box* rightFrontLower;


	Box* leftBackUpper;
	Box* leftBackLower;

	Box* rightBackUpper;
	Box* rightBackLower;

	glm::vec3 m_startPosition;
	NeuralNetwork m_neuralNetwork;
	double m_fitness;
	std::vector<double> resultVec;


	glm::vec3 getCenterPosition();
public:
	Spider(PhysicsManager* pm, glm::vec3 startPosition, std::default_random_engine engine);
	void render(Shader shader);
	void updatePhysics();

	void setAllTargetVelocities(std::vector<double>& resultVec);

	void setMaxMotorImpulses(double maxMotorImpulse);
	void createNeuralNetwork(std::vector<int> topology, std::default_random_engine engine);
	void setNeuralNetwork(NeuralNetwork neuralNetwork);

	NeuralNetwork getNeuralNetwork();

	void updateNeuralNetwork();

	void mutate(double mutationRate, double mutationChance, std::default_random_engine engine);

	void setFitness(double fitness);

	double getFitness();
	
	std::vector<double> calculateInputs();

	std::vector<double> getAllAngularVelocities();

	std::vector<double> getAllAngles();

	void activate();

	Box* getBody();
	Box* getLeftFrontUpper();
	Box* getLeftFrontLower();

	Box* getRightFrontUpper();
	Box* getRightFrontLower();
	

	Box* getLeftBackUpper();
	Box* getLeftBackLower();

	Box* getRightBackUpper();
	Box* getRightBackLower();

	glm::vec3 getPosition();
	glm::vec3 getStartPosition();
	~Spider();
};

