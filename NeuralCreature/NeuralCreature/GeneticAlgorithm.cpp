#include "stdafx.h"
#include "GeneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm(double mutationRate, double crossoverProb, int populationSize, int numElites, PhysicsManager* pm)
	: m_mutationRate(mutationRate), m_crossoverProb(crossoverProb), m_populationSize(populationSize), m_numElites(numElites)
{
	initCreatures(pm);
	generation = 0;
}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{

	//std::vector<Creature> testVec(m_populationSize);
	creatures.reserve(m_populationSize);
	for (int i = 0; i < m_populationSize; i++) {
		
		Creature* tempCret = new Creature(pm, glm::vec3(1.0, 12.0, 0.0));
		creatures.push_back(tempCret);

	}
}

void GeneticAlgorithm::createNewGeneration()
{
	for (int i = 0; i < creatures.size(); i++) {
		Creature* tempCret = creatures[i];
		evaluateFitness(creatures[i]);
		//crossOver(tempCret, Bounds);
		mutate(creatures[i]);

		creatures[i]->reset();
	}
	generation++;
	std::cout << "Generation: " << generation << std::endl;

}

double GeneticAlgorithm::getDistanceWalked(Creature* creature)
{
	glm::vec3 end = creature->getPosition();
	glm::vec3 start = creature->getStartPosition();
	return glm::distance(end, start);
}

void GeneticAlgorithm::crossOver()
{
}


double GeneticAlgorithm::evaluateFitness(Creature* creature)
{
	return getDistanceWalked();
}

void GeneticAlgorithm::mutate(Creature* creature)
{
	creature->mutate(m_mutationRate);
}

void GeneticAlgorithm::updateCreatures(Shader shader)
{
	for (int i = 0; i < creatures.size(); i++) {
		creatures.at(i)->activate();
		creatures.at(i)->updatePhysics();
		creatures[i]->render(shader);
	}
}

GeneticAlgorithm::~GeneticAlgorithm()
{
}
