#pragma once
#include "Biped.h"
#include "Spider.h"
#include <vector>
#include <queue>
#include <thread>
#include "NetworkWriter.h"

const int FITNESS_WALKING = 0;
const int FITNESS_JUMPING = 1;
const int FITNESS_STANDING = 2;

// 0 = biped, 1 = dog
const int CREATURE = 1;

class GeneticAlgorithm
{
private:
	double m_mutationRate;
	double m_crossoverProb;
	const int m_populationSize;
	int m_numElites;
	int timesNoImprovement;
	std::vector<Biped*> bipeds;
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

public:
	GeneticAlgorithm(double mutationRate, double mutationChance, double crossoverProb, const int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	NeuralNetwork crossOver(NeuralNetwork * parentA, NeuralNetwork * parentB);
	static double evaluateFitness(Biped* creature, int fitnessType, int currentStep);
	void mutate(Biped * creature, double mutationRate, double mutationChance, std::default_random_engine engine);
	void updateCreatures(Shader shader, bool render, PhysicsManager* pm);
	static void updateCreature(Biped* creature, int fitnessType, int currentStep);
	void createNewGeneration(PhysicsManager * pm);
	static double getDistanceWalked(Biped* creature);
	bool keepRunning();
	std::vector<unsigned> getSeedsForCreatures();
	~GeneticAlgorithm();
};

