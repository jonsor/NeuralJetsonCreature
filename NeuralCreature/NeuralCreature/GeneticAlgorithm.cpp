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

	std::vector<Creature*> fitCret;
	fitCret.reserve(m_populationSize/2);
	double lowest = 0.0;
	int unfittestIndex = 0;

	for (int i = 0; i < creatures.size(); i++) {
		//Creature* tempCret = creatures[i];
		double tempFit = evaluateFitness(creatures[i]);
		if (i == 0) {
			lowest = tempFit;
		}
		if (fitCret.size() < m_populationSize/2) {
				fitCret.push_back(creatures[i]);
				if (tempFit <= lowest) {
					lowest = tempFit;
					unfittestIndex = i;
				}
		}
		else {
			if (tempFit >= lowest) {
				fitCret[unfittestIndex] = creatures[i];
				
				double lowestFit = fitCret[0]->getFitness();		
				for (int k = 0; k < fitCret.size(); k++) {
					double fitFinder = fitCret[k]->getFitness();
					if (fitFinder < lowestFit) {
						lowestFit = fitFinder;
						lowest = lowestFit;
						unfittestIndex = k;
					}

				}

				std::cout << "tempFit: " << tempFit << " Lowest: " << lowest << std::endl;
			}
		}
		
		//crossOver(tempCret, Bounds);


		mutate(creatures[i]);

		creatures[i]->reset();
	}

	for (int i = 0; i < fitCret.size(); i++) {
		std::cout << fitCret[i]->getFitness() << " ";
	}
	std::cout << "\n************************************************" << std::endl;

	for (int i = 0; i < creatures.size(); i++) {
		std::cout << creatures[i]->getFitness() << " " ;
	}
	std::cout << std::endl;
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
	double fitness = getDistanceWalked(creature);
	creature->setFitness(fitness);
	return fitness;
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
