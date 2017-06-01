#pragma once
#include "Creature.h"
#include "Spider.h"
#include <vector>
#include <queue>
#include <thread>

class GeneticAlgorithm
{
private:
	double m_mutationRate;
	double m_crossoverProb;
	const int m_populationSize;
	int m_numElites;
	int timesNoImprovement;
	std::vector<Spider*> creatures;
	int generation;
	double lastFitness;
public:
	GeneticAlgorithm(double mutationRate, double crossoverProb, int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	NeuralNetwork* crossOver(NeuralNetwork * parentA, NeuralNetwork * parentB);
	double evaluateFitness(Spider* creature);
	void mutate(Spider* creature, double mutationRate);
	void updateCreatures(Shader shader, bool render);
	static void updateCreature(Shader shader, Spider* creature);
	void createNewGeneration(PhysicsManager * pm);
	double getDistanceWalked(Spider* creature);
	~GeneticAlgorithm();
};

