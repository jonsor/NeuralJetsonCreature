#include "stdafx.h"
#include "GeneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm(double mutationRate, double mutationChance, double crossoverProb, const int populationSize, int numElites, PhysicsManager* pm)
	: m_mutationRate(mutationRate), m_crossoverProb(crossoverProb), m_populationSize(populationSize), m_numElites(numElites), m_mutationChance(mutationChance), initalMutationRate(mutationRate)
{
	initCreatures(pm);
	generation = 0;
	timesNoImprovement = 0;
	lastFitness = 0;
	timeNotWritten = 0;
	stillStanding = false;
}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{

	//std::vector<Creature> testVec(m_populationSize);
	//creatures.reserve(m_populationSize);
	for (int i = 0; i < m_populationSize; i++) {
		
		Creature* tempCret = new Creature(pm, glm::vec3(1.0, 10.0, 0.0));
		creatures.push_back(tempCret);

	}
	//NetworkWriter::readFromFile(creatures);

	//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[0] << "\n";
	//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[1] << "\n";
	//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[2] << "\n";
}

bool moreThanByFitness(Creature* lhs, Creature* rhs) { return (lhs->getFitness() > rhs->getFitness()); }


void GeneticAlgorithm::createNewGeneration(PhysicsManager * pm) //TODO: FIKS MINNELEKASJE HER:
{
	l = 0;

	timeNotWritten++;
	if (timeNotWritten >= 5) {
		NetworkWriter::writeToFile(creatures);
		//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[0] << "\n";
		//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[1] << "\n";
		//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[2] << "\n";
		timeNotWritten = 0;
	}

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

	for (int i = 0; i < creatures.size(); i++) {
		creatures[i]->removeConstraints(pm);

		creatures[i]->removeBodies(pm);
	}

	pm->reset();
	double divider = 2.5;
	int partSize = creatures.size() / divider;
	for (int i = 0; i < creatures.size(); i++) {

		Creature* tempCret = new Creature(pm, glm::vec3(1.0, 10.0, 0.0));
		
		if (i == 0) {
			tempCret->setColor(glm::vec3(0.8f, 0.1f, 0.6f));
		}else if (i == 1) {
			tempCret->setColor(glm::vec3(0.1f, 0.8f, 0.1f));
		}

		//if (i != creatures.size() - 1) {
			if (i < partSize) {
				tempCret->setNeuralNetwork(NeuralNetwork(creatures[i]->getNeuralNetwork()));
			} else {
				if (i - partSize == 1) {
					tempCret->setNeuralNetwork(crossOver(&creatures[0]->getNeuralNetwork(), &creatures[i + 1 - partSize]->getNeuralNetwork()));
				}
				else {
					tempCret->setNeuralNetwork(crossOver(&creatures[i - partSize]->getNeuralNetwork(), &creatures[i + 1 - partSize]->getNeuralNetwork()));
				}

			}
		//}

		Creature* oldCret = creatures[i];
		creatures[i] = tempCret;
		tempCret = nullptr;
		//oldCret->removeConstraints(pm);
		//oldCret->removeBodies(pm);
		delete oldCret;
		//creatures[i]->~Creature();
		//creatures[i]->setNeuralNetwork(fitCret[j]->getNeuralNetwork());
		if (i >= 1) {
			//double muRate = rand() % 10;
			//double muRate = ((double)rand() / (RAND_MAX));
			mutate(creatures[i], m_mutationRate, m_mutationChance);
		}
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
			m_mutationRate *= 0.9;
		}
		std::cout << "mutation rate: " << m_mutationRate << "\n";
	} else {
		m_mutationRate = initalMutationRate;
	}


	std::cout << "Generation: " << generation  << " creatures size: " << creatures.size() << std::endl;
	generation++;

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

NeuralNetwork GeneticAlgorithm::crossOver(NeuralNetwork * parent, NeuralNetwork * crossOverRecipient)
{

	std::vector<Layer> lA = parent->getLayers();
	std::vector<Layer> lB = crossOverRecipient->getLayers();
	//lA[1][1].setOutputVal(59);
	//std::cout << "lA: " << lA[1][1].getOutputVal() << "\n";
	//std::cout << "lA: " << parent->getLayers()[1][1].getOutputVal() << "\n";

	double prob = ((double)rand() / (RAND_MAX));
	NeuralNetwork child;
	if (prob > m_crossoverProb) {
		//int numLayersToCross = lA.size() / 2;
		//std::vector<int> crossOverLayerIndices = Util::getRandomIndices(lA.size(), numLayersToCross);

		//for (int i = 0; i < crossOverLayerIndices.size(); i++) {
		//	int lI = crossOverLayerIndices[i];
		//	int layerSize = lA[lI].size();
		//	int numNeuronsToCross = layerSize / 2;
		//	std::vector<int> crossOverNeuronIndices = Util::getRandomIndices(layerSize, numNeuronsToCross);

		//	for (int j = 0; j < crossOverNeuronIndices.size(); j++) {
		//		int nI = crossOverNeuronIndices[j];
		//		lB[lI][nI] = lA[lI][nI];
		//	}

		//}


		for (int i = 0; i < lA.size(); i++) {
			for (int j = 0; j < lA[i].size(); j++) {
				if ((double)rand() > (RAND_MAX)) {
					lB[i][j] = lA[i][j];
				}
			}
		}



		child = NeuralNetwork(lB);
	}
	else {
		double parentProb = ((double)rand() / (RAND_MAX));
		if (parentProb >= 0.5) {
			child = NeuralNetwork(lA);
		}
		else {
			child = NeuralNetwork(lB);
		}
	}

	return child;

}


double GeneticAlgorithm::evaluateFitness(Creature* creature)
{
	double fitnessD = getDistanceWalked(creature);
	double fitnessH = creature->getAverageHeight();
	double fitnessMH = creature->getMaxHeight();
	////std::cout << creature->getTimeOnGround() << "\n";
	double groundTimeN = Util::normalizeSigned(creature->getTimeOnGround(), 0, 100);
	double standTimeN = Util::normalizeSigned(creature->getTimeOnTwoLegs(), 0, 100);
	//double groundModifier = 0;
	//if (creature->getTimeOnGround() >= 1) {
	//	groundModifier = 10;
	//}
	//std::cout << "crossed: " << creature->getNumTimesCrossed() << "\n";
	//double fitness = fitnessD * 4 - creature->getTimeOnGround() - creature->getTimeOnTwoLegs();
	//double fitness = fitnessD * 2 - creature->getTimeOnGround() - creature->getTimeOnTwoLegs();
	//double fitness = creature->getNumTimesCrossed();
	//std::cout << fitness << "\n";
	//std::cout << "ground/stand: " << groundTimeN << "  " <<standTimeN << "\n";
	double fitness = fitnessD + fitnessH;
	creature->setFitness(fitness);
	return fitness;
}

void GeneticAlgorithm::mutate(Creature* creature, double mutationRate, double mutationChance)
{
	creature->mutate(mutationRate, mutationChance);
}

void GeneticAlgorithm::updateCreatures(Shader shader, bool render, PhysicsManager* pm)
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
	stillStanding = false;
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
		creatures[i]->getRightFoot()->setCollidingWithGround(false);
		creatures[i]->getLeftFoot()->setCollidingWithGround(false);
		int numManifolds = pm->getDynamicsWorld()->getDispatcher()->getNumManifolds();
		for (int l = 0; l < numManifolds; l++)
		{
			btPersistentManifold* contactManifold = pm->getDynamicsWorld()->getDispatcher()->getManifoldByIndexInternal(l);
			const btCollisionObject* obA = contactManifold->getBody0();
			const btCollisionObject* obB = contactManifold->getBody1();

			int numContacts = contactManifold->getNumContacts();
			for (int j = 0; j < numContacts; j++)
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (pt.getDistance() < 0.f)
				{
					const btVector3& ptA = pt.getPositionWorldOnA();
					const btVector3& ptB = pt.getPositionWorldOnB();
					const btVector3& normalOnB = pt.m_normalWorldOnB;

					if ((btRigidBody*)obA == creatures[i]->getRightFoot()->getRigidBody() || (btRigidBody*)obB == creatures[i]->getRightFoot()->getRigidBody())
					{
						creatures[i]->getRightFoot()->setCollidingWithGround(true);
					}

					if ((btRigidBody*)obA == creatures[i]->getLeftFoot()->getRigidBody() || (btRigidBody*)obB == creatures[i]->getLeftFoot()->getRigidBody())
					{
						creatures[i]->getLeftFoot()->setCollidingWithGround(true);
					}

				}
			}
		}

		if (creatures[i]->getHeight() > 3.6f) {
			stillStanding = true;
		}

		if (render) {
			creatures[i]->render(shader);
		}
		else {
			creatures[0]->render(shader);
		}
	}
}

void GeneticAlgorithm::updateCreature(Shader shader, Creature* creature)
{
	creature->activate();
	creature->updatePhysics();
	creature->incrementToAverage();
	if (creature->getHeight() < 2.f) {
		creature->setTimeOnGround(creature->getTimeOnGround() + 0.1);
	}
	if (creature->getHeight() < 4.5f) {
		creature->setTimeOnTwoLegs(creature->getTimeOnTwoLegs() + 0.1);
	}
	creature->checkIfLegsCrossed();
	creature->updateMaxHeight(creature->getHeight());
}

bool GeneticAlgorithm::isStillStanding() {
	return stillStanding;
}

GeneticAlgorithm::~GeneticAlgorithm()
{
}
