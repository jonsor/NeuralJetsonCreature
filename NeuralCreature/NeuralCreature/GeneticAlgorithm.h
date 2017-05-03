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
	std::vector<Creature*> creatures;
	int generation;
public:
	GeneticAlgorithm(double mutationRate, double crossoverProb, int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	void crossOver();
	double evaluateFitness(Creature* creature);
	void mutate(Creature* creature);
	void updateCreatures(Shader shader);
	void createNewGeneration();
	~GeneticAlgorithm();
};

