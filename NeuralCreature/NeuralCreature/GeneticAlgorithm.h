#pragma once
#include "Creature.h"
#include "Spider.h"
#include <vector>
#include <queue>
#include <thread>
#include "NetworkWriter.h"

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
	int l = 0;
	int timeNotWritten;
	bool stillStanding;
	double m_mutationChance;
	double initalMutationRate;
public:
	GeneticAlgorithm(double mutationRate, double mutationChance, double crossoverProb, const int populationSize, int numElites, PhysicsManager * pm);
	void initCreatures(PhysicsManager * pm);
	NeuralNetwork crossOver(NeuralNetwork * parentA, NeuralNetwork * parentB);
	double evaluateFitness(Creature* creature);
	void mutate(Creature * creature, double mutationRate, double mutationChance);
	void updateCreatures(Shader shader, bool render, PhysicsManager* pm);
	static void updateCreature(Shader shader, Creature* creature);
	bool isStillStanding();
	void createNewGeneration(PhysicsManager * pm);
	double getDistanceWalked(Creature* creature);
	~GeneticAlgorithm();
};

