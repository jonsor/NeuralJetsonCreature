#include "stdafx.h"
#include "GeneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm(double mutationRate, double crossoverProb, const int populationSize, int numElites, PhysicsManager* pm)
	: m_mutationRate(mutationRate), m_crossoverProb(crossoverProb), m_populationSize(populationSize), m_numElites(numElites)
{
	initCreatures(pm);
	generation = 0;
	timesNoImprovement = 0;
	lastFitness = 0;
}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{

	//std::vector<Creature> testVec(m_populationSize);
	//creatures.reserve(m_populationSize);
	for (int i = 0; i < m_populationSize; i++) {
		
		Creature* tempCret = new Creature(pm, glm::vec3(1.0, 10.0, 0.0));
		creatures.push_back(tempCret);

	}
}

bool moreThanByFitness(Creature* lhs, Creature* rhs) { return (lhs->getFitness() > rhs->getFitness()); }

int l = 0;
void GeneticAlgorithm::createNewGeneration(PhysicsManager * pm)
{
	l = 0;
	//std::vector<Creature*> fitCret;
	//fitCret.reserve(m_populationSize/2);
	//double lowestOfBest = 0.0;
	//int unfittestOfBestIndex = 0;

	//for (int i = 0; i < creatures.size(); i++) {
	//	double tempFit = evaluateFitness(creatures[i]);
	//	creatures[i]->setFitness(tempFit);


	//	if (i == 0) {
	//		lowestOfBest = tempFit;
	//	}
	//	if (fitCret.size() < m_populationSize/2) {
	//			fitCret.push_back(creatures[i]);
	//			if (tempFit <= lowestOfBest) {
	//				lowestOfBest = tempFit;
	//				unfittestOfBestIndex = i;
	//			}
	//	}
	//	else {
	//		if (tempFit >= lowestOfBest) {
	//			fitCret[unfittestOfBestIndex] = creatures[i];
	//			
	//			double lowestFit = fitCret[0]->getFitness();		
	//			for (int k = 0; k < fitCret.size(); k++) {
	//				double fitFinder = fitCret[k]->getFitness();
	//				if (fitFinder <= lowestFit) {
	//					lowestFit = fitFinder;
	//					lowestOfBest = lowestFit;
	//					unfittestOfBestIndex = k;
	//				}

	//			}

	//			//std::cout << "tempFit: " << tempFit << " Lowest: " << lowestOfBest << std::endl;
	//		}
	//	}
	//	//std::cout << "tempFit: " << tempFit << " LowestOfBest: " << lowestOfBest << std::endl;
	//	//crossOver(tempCret, Bounds);


	//	//mutate(creatures[i]);

	//	//creatures[i]->reset();
	//}

	////for (int i = 0; i < fitCret.size(); i++) {
	////	std::cout << fitCret[i]->getFitness() << " ";
	////}
	////std::cout << "\n************************************************" << std::endl;

	////for (int i = 0; i < creatures.size(); i++) {
	////	std::cout << creatures[i]->getFitness() << " " ;
	////}
	////std::cout << std::endl;

	////std::cout << "proqueue: " << std::endl;
	////for (int i = 0; i < creaturesQueue.size(); i++) {
	////	std::cout << creaturesQueue.top()->getFitness() << std::endl;
	////	creaturesQueue.pop();
	////}

	for (int i = 0; i < creatures.size(); i++) {
		double tempFit = evaluateFitness(creatures[i]);
		creatures[i]->setFitness(tempFit);
	}

	std::sort(creatures.begin(), creatures.end(), moreThanByFitness);
	std::cout << "sorted vector: " << std::endl;
	for (int i = 0; i < creatures.size(); i++) {
		std::cout << creatures[i]->getFitness() << std::endl;
	}

	int bestFitIndex = 0;
	double bestFit = creatures[0]->getFitness();
	std::cout << "BestFit: " << bestFit << std::endl;
	//for (int i = 0; i < fitCret.size(); i++) {
	//	if (fitCret[i]->getFitness() > bestFit) {
	//		bestFit = fitCret[i]->getFitness();
	//		bestFitIndex = i;
	//	}
	//}
	pm->reset();
	double divider = 2.5;
	int partSize = creatures.size() / divider;
	int j = 0;
	for (int i = 0; i < creatures.size(); i++) {
		if (j == creatures.size()/ divider) {
			j = 0;
		}

		Creature* tempCret = new Creature(pm, glm::vec3(1.0, 10.0, 0.0));
		if (i == 0) {
			tempCret->setColor(glm::vec3(0.8f, 0.1f, 0.6f));
		}else if (i == 1) {
			tempCret->setColor(glm::vec3(0.1f, 0.8f, 0.1f));
		}

		if (i != creatures.size() - 1) {
			if (i < partSize) {
				tempCret->setNeuralNetwork(new NeuralNetwork(*creatures[i]->getNeuralNetwork()));
			}
			else {
				if (i - partSize == 1) {
					tempCret->setNeuralNetwork(crossOver(creatures[0]->getNeuralNetwork(), creatures[i + 1 - partSize]->getNeuralNetwork()));
				}
				else {
					tempCret->setNeuralNetwork(crossOver(creatures[i - partSize]->getNeuralNetwork(), creatures[i + 1 - partSize]->getNeuralNetwork()));
				}

			}
		}

		Creature* oldCret = creatures[i];
		creatures[i] = tempCret;
		tempCret = nullptr;
		//oldCret->removeConstraints(pm);
		//oldCret->removeBodies(pm);
		//delete oldCret;
		//creatures[i]->~Creature();
		//creatures[i]->setNeuralNetwork(fitCret[j]->getNeuralNetwork());
		if (i >= 1) {
			//double muRate = rand() % 10;
			double muRate = ((double)rand() / (RAND_MAX));
			mutate(creatures[i], m_mutationRate * muRate);
		}
		j++;
	}


	if (lastFitness == bestFit) {
		std::cout << "times are not changin\': " << timesNoImprovement <<"\n";
		timesNoImprovement++;
	} else {
		timesNoImprovement = 0;
	}
	lastFitness = bestFit;
	if(timesNoImprovement >= 3) {
		if (m_mutationRate > 0.001) {
			//m_mutationRate *= 0.9;
		}
		std::cout << "mutation rate: " << m_mutationRate << "\n";
	} else {
		m_mutationRate = 0.1;
	}

	std::cout << "Generation: " << generation  << " creatures size: " << creatures.size() << std::endl;
	generation++;


	//for (int i = 0; i < creatures.size(); i++) {
	//	Creature* tempCret = new Creature(pm, glm::vec3(1.0, 12.0, 0.0));
	//	tempCret->setNeuralNetwork(creatures[i]->getNeuralNetwork());
	//	creatures[i] = tempCret;
	//}

}

double GeneticAlgorithm::getDistanceWalked(Creature* creature)
{
	glm::vec3 end = creature->getPosition();
	glm::vec3 start = creature->getStartPosition();
	double distancex = pow((end.x - start.x),2);
	double distancey = pow((end.z - start.z),2);

	//return sqrt(distancex + distancey);

	return start.z - end.z;
	//return glm::distance(end, start);
}

NeuralNetwork * GeneticAlgorithm::crossOver(NeuralNetwork * parent, NeuralNetwork * crossOverRecipient)
{

	std::vector<Layer> lA = parent->getLayers();
	std::vector<Layer> lB = crossOverRecipient->getLayers();
	//lA[1][1].setOutputVal(59);
	//std::cout << "lA: " << lA[1][1].getOutputVal() << "\n";
	//std::cout << "lA: " << parent->getLayers()[1][1].getOutputVal() << "\n";
	int numLayersToCross = lA.size() /2;
	std::vector<int> crossOverLayerIndices = Util::getRandomIndices(lA.size(), numLayersToCross);

	for (int i = 0; i < crossOverLayerIndices.size(); i++) {
		int lI = crossOverLayerIndices[i];
		int layerSize = lA[lI].size();
		int numNeuronsToCross = layerSize / 2;
		std::vector<int> crossOverNeuronIndices = Util::getRandomIndices(layerSize, numNeuronsToCross);

		for (int j = 0; j < crossOverNeuronIndices.size(); j++) {
			int nI = crossOverNeuronIndices[j];
			lB[lI][nI] = lA[lI][nI];
		}
		
	}

	NeuralNetwork* child = new NeuralNetwork(lB);
	return child;

}


double GeneticAlgorithm::evaluateFitness(Creature* creature)
{
	double fitnessD = getDistanceWalked(creature);
	double fitnessH = creature->getAverageHeight();
	//double fitnessH = creature->getMaxHeight();
	//std::cout << creature->getTimeOnGround() << "\n";
	double groundTimeN = Util::normalize(creature->getTimeOnGround(), 0, 600);
	double standTimeN = Util::normalize(creature->getTimeOnTwoLegs(), 0, 600);
	double groundModifier = 0;
	if (creature->getTimeOnGround() >= 1) {
		groundModifier = 10;
	}
	std::cout << "crossed: " << creature->getNumTimesCrossed() << "\n";
	//double fitness = fitnessD * 4 - creature->getTimeOnGround() - creature->getTimeOnTwoLegs();
	double fitness = fitnessD * 2 - creature->getTimeOnGround() - creature->getTimeOnTwoLegs();
	//double fitness = creature->getNumTimesCrossed();
	//std::cout << fitness << "\n";
	//double fitness = fitnessD;
	creature->setFitness(fitness);
	return fitness;
}

void GeneticAlgorithm::mutate(Creature* creature, double mutationRate)
{
	creature->mutate(mutationRate);
}

void GeneticAlgorithm::updateCreatures(Shader shader, bool render)
{
	l++;
	if (l == 250) {
		//std::cout << creatures[0]->getPosition().y << std::endl;
		l = 0;
	}

	const int NUM_THREADS = 10;
	std::thread threads[NUM_THREADS];
	int rc;
	int i;

	for (i = 0; i < NUM_THREADS; i++) {
		threads[i] = std::thread(updateCreature, shader, creatures[i]);

	}
	for (int i = 0; i < NUM_THREADS; i++) {
		threads[i].join();
	}

	//creatures[0]->checkIfLegsCrossed();
	for (int i = 0; i < creatures.size(); i++) {

		//creatures.at(i)->activate();

		//creatures.at(i)->updatePhysics();

		//creatures.at(i)->incrementToAverage();

		//if(l > 40)creatures[i]->updateMaxHeight(creatures[i]->getHeight());
		//if (creatures[i]->getHeight() < 2.f) {
		//	creatures[i]->setTimeOnGround(creatures[i]->getTimeOnGround() + 0.1);
		//}
		//if (creatures[i]->getHeight() < 5.5f) {
		//	creatures[i]->setTimeOnTwoLegs(creatures[i]->getTimeOnTwoLegs() + 0.1);
		//}
		if (render) {
			creatures[i]->render(shader);
		}
	}
}

void GeneticAlgorithm::updateCreature(Shader shader, Creature * creature)
{
	creature->activate();
	creature->updatePhysics();
	creature->incrementToAverage();
	if (creature->getHeight() < 2.f) {
		creature->setTimeOnGround(creature->getTimeOnGround() + 0.1);
	}
	if (creature->getHeight() < 5.5f) {
		creature->setTimeOnTwoLegs(creature->getTimeOnTwoLegs() + 0.1);
	}
	creature->checkIfLegsCrossed();
}

GeneticAlgorithm::~GeneticAlgorithm()
{
}
