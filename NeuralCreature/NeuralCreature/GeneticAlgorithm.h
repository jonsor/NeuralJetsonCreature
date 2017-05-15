#pragma once
#include "Creature.h"
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
	std::vector<Creature*> creatures;
	int generation;
	double lastFitness;
public:
	GeneticAlgorithm(double mutationRate, double crossoverProb, int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	NeuralNetwork* crossOver(NeuralNetwork * parentA, NeuralNetwork * parentB);
	double evaluateFitness(Creature* creature);
	void mutate(Creature* creature, double mutationRate);
	void updateCreatures(Shader shader, bool render);
	static void updateCreature(Shader shader,Creature * creature);
	void createNewGeneration(PhysicsManager * pm);
	double getDistanceWalked(Creature* creature);
	~GeneticAlgorithm();
};

