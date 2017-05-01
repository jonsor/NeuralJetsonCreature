#pragma once
#include "Creature.h"
#include <vector>
class GeneticAlgorithm
{
private:
	double m_mutationRate;
	double m_crossoverProb;
	int m_populationSize;
	int m_numElites;
	std::vector<Creature> creatures;
public:
	GeneticAlgorithm(double mutationRate, double crossoverProb, int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	void crossOver();
	double evaluateFitness();
	void mutate();
	void updateCreatures();
	void createNewGeneration();
	~GeneticAlgorithm();
};

