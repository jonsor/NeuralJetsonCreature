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
	void updateCreatures(Shader shader, bool render);
	static void updateCreature(Shader shader,Creature * creature);
	void createNewGeneration(PhysicsManager * pm);
	double getDistanceWalked(Creature* creature);
	~GeneticAlgorithm();
};

