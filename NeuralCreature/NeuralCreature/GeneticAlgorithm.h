#pragma once
#include "Creature.h"
#include "Spider.h"
#include <vector>
#include <queue>
#include <thread>
#include "NetworkWriter.h"

const int FITNESS_WALKING = 0;
const int FITNESS_JUMPING = 1;
const int FITNESS_STANDING = 2;

class GeneticAlgorithm
{
private:
	double m_mutationRate;
	double m_crossoverProb;
	const int m_populationSize;
	int m_numElites;
	int timesNoImprovement;
	std::vector<Creature*> creatures;
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

public:
	GeneticAlgorithm(double mutationRate, double mutationChance, double crossoverProb, const int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	NeuralNetwork crossOver(NeuralNetwork * parentA, NeuralNetwork * parentB);
	static double evaluateFitness(Creature* creature, int fitnessType, int currentStep);
	void mutate(Creature * creature, double mutationRate, double mutationChance, std::default_random_engine engine);
	void updateCreatures(Shader shader, bool render, PhysicsManager* pm);
	static void updateCreature(Creature* creature, int fitnessType, int currentStep);
	void createNewGeneration(PhysicsManager * pm);
	static double getDistanceWalked(Creature* creature);
	bool keepRunning();
	std::vector<unsigned> getSeedsForCreatures();
	~GeneticAlgorithm();
};

