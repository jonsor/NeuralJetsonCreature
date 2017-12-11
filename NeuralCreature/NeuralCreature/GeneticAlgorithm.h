#pragma once
#include "Biped.h"
#include "Spider.h"
#include "Dog.h"
#include <vector>
#include <queue>
#include <thread>
#include "NetworkWriter.h"

#include "NEATGene.h"
#include "NEATController.h"
#include "NEATNetwork.h"
#include "NEATNeuron.h"
#include "NEATNode.h"

const int FITNESS_WALKING = 0;
const int FITNESS_JUMPING = 1;
const int FITNESS_STANDING = 2;

const int BIPED = 0;
const int DOG = 1;

class GeneticAlgorithm
{
private:
	int creatureType;
	double m_mutationRate;
	double m_crossoverProb;
	int m_populationSize;
	int m_numElites;
	int m_numCreatures;
	int timesNoImprovement;
	std::vector<Biped*> bipeds;
	std::vector<Dog*> dogs;
	int generation;
	double lastFitness;
	int m_currentStep;
	int timeNotWritten;
	double m_mutationChance;
	double initalMutationRate;
	unsigned m_overSeed;
	std::default_random_engine m_overEngine;
	std::vector<std::default_random_engine> m_randomEngines;
	bool m_keepRunning;
	int fitnessType;
	double allTimeBestFitness;

	glm::vec3 targetPos;
	Box* targetPosBox;

public:
	GeneticAlgorithm(double mutationRate, double mutationChance, double crossoverProb, const int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	RecurrentNeuralNetwork crossOver(RecurrentNeuralNetwork * parentA, RecurrentNeuralNetwork * parentB);
	static double evaluateFitnessBiped(Biped* creature, int fitnessType, int currentStep);
	static double evaluateFitnessDog(Dog* creature, int fitnessType, int currentStep, int timesNoImprovement);
	void updateCreatures(Shader shader, bool render, PhysicsManager* pm);
	static void updateBiped(Biped* creature, int fitnessType, int currentStep);
	static void updateDog(Dog* creature, int fitnessType, int currentStep, int timesNoImprovement);
	void createNewGeneration(PhysicsManager * pm);
	bool keepRunning();
	std::vector<unsigned> getSeedsForCreatures();
	~GeneticAlgorithm();
};

