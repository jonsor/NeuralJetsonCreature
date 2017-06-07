#pragma once
#include "Cube.h"
#include "NeuralNetwork.h"


class Spider
{
private:
	Cube* body;

	Cube* leftFrontUpper;
	Cube* leftFrontLower;

	Cube* rightFrontUpper;
	Cube* rightFrontLower;


	Cube* leftBackUpper;
	Cube* leftBackLower;

	Cube* rightBackUpper;
	Cube* rightBackLower;

	glm::vec3 m_startPosition;
	NeuralNetwork m_neuralNetwork;
	double m_fitness;
	std::vector<double> resultVec;


	glm::vec3 getCenterPosition();
public:
	Spider(PhysicsManager* pm, glm::vec3 startPosition);
	void render(Shader shader);
	void updatePhysics();

	void setAllTargetVelocities(std::vector<double>& resultVec);

	void setMaxMotorImpulses(double maxMotorImpulse);
	void createNeuralNetwork(std::vector<int> topology);
	void setNeuralNetwork(NeuralNetwork neuralNetwork);

	NeuralNetwork getNeuralNetwork();

	void updateNeuralNetwork();

	void mutate(double mutationRate, double mutationChance);

	void setFitness(double fitness);

	double getFitness();
	
	std::vector<double> calculateInputs();

	std::vector<double> getAllAngularVelocities();

	std::vector<double> getAllAngles();

	void activate();

	Cube* getBody();
	Cube* getLeftFrontUpper();
	Cube* getLeftFrontLower();

	Cube* getRightFrontUpper();
	Cube* getRightFrontLower();
	

	Cube* getLeftBackUpper();
	Cube* getLeftBackLower();

	Cube* getRightBackUpper();
	Cube* getRightBackLower();

	glm::vec3 getPosition();
	glm::vec3 getStartPosition();
	~Spider();
};

