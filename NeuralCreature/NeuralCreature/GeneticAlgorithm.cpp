#include "stdafx.h"
#include "GeneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm(double mutationRate, double crossoverProb, int populationSize, int numElites, PhysicsManager* pm)
	: m_mutationRate(mutationRate), m_crossoverProb(crossoverProb), m_populationSize(populationSize), m_numElites(numElites)
{
	initCreatures(pm);
}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{

	//std::vector<Creature> testVec(m_populationSize);
	creatures.reserve(m_populationSize);
	for (int i = 0; i < m_populationSize; i++) {
		
		Creature* tempCret = new Creature(pm, glm::vec3(i*5, 12.0, 0.0));
		creatures.push_back(tempCret);

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

void GeneticAlgorithm::updateCreatures(Shader shader)
{
	for (int i = 0; i < creatures.size(); i++) {
		creatures.at(i)->activate();
		creatures.at(i)->updatePhysics();
		creatures[i]->render(shader);
	}
}

void GeneticAlgorithm::createNewGeneration()
{
}

GeneticAlgorithm::~GeneticAlgorithm()
{
}
