#include "stdafx.h"
#include "GeneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm(double mutationRate, double crossoverProb, int populationSize, int numElites, PhysicsManager* pm)
	: m_mutationRate(mutationRate), m_crossoverProb(crossoverProb), m_populationSize(populationSize), m_numElites(numElites)
{
	initCreatures(pm);
}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{
	
	for (int i = 0; i < m_populationSize; i++) {
		//Creature tempCret(pm, glm::vec3(20.0f, 20.0f, 20.0f));
		//creatures.push_back(Creature(pm, glm::vec3(0.0f, 0.0f, 0.0f)));
	}
}

void GeneticAlgorithm::crossOver()
{
}

double GeneticAlgorithm::evaluateFitness()
{
	return 0.0;
}

void GeneticAlgorithm::mutate()
{
}

void GeneticAlgorithm::updateCreatures()
{
}

void GeneticAlgorithm::createNewGeneration()
{
}

GeneticAlgorithm::~GeneticAlgorithm()
{
}
