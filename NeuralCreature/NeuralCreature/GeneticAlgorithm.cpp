#include "stdafx.h"
#include "GeneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm(double mutationRate, double mutationChance, double crossoverProb, const int populationSize, int numElites, PhysicsManager* pm)
	: m_mutationRate(mutationRate), m_crossoverProb(crossoverProb), m_populationSize(populationSize), m_numElites(numElites), m_mutationChance(mutationChance), initalMutationRate(mutationRate)
{

	//Random Setup:
	m_overSeed = std::chrono::system_clock::now().time_since_epoch().count();
	//m_overSeed = 0;
	m_overEngine = std::default_random_engine(m_overSeed);
	std::uniform_int_distribution<int> overDistribution(0, INT_MAX); //INT_MIN instead of 0?
	m_randomEngines = std::vector<std::default_random_engine>();

	for (int i = 0; i < m_populationSize; i++) {
		int randomSeed = overDistribution(m_overEngine);
		m_randomEngines.push_back(std::default_random_engine(randomSeed));
	}

	initCreatures(pm);
	generation = 0;
	timesNoImprovement = 0;
	lastFitness = 0;
	timeNotWritten = 0;

	currentStep = 0;
	m_keepRunning = true;

}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{

	//std::vector<Creature> testVec(m_populationSize);
	//creatures.reserve(m_populationSize);
	for (int i = 0; i < m_populationSize; i++) {
		
		std::default_random_engine &rand = m_randomEngines[i];
		Creature* tempCret = new Creature(pm, glm::vec3(1.0, 10.0, 0.0), rand);
		creatures.push_back(tempCret);

	}
	//NetworkWriter::readFromFile(creatures, m_overSeed);

	//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[0] << "\n";
	//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[1] << "\n";
	//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[2] << "\n";
}

bool moreThanByFitness(Creature* lhs, Creature* rhs) { return (lhs->getFitness() > rhs->getFitness()); }


void GeneticAlgorithm::createNewGeneration(PhysicsManager * pm) //TODO: FIKS MINNELEKASJE HER:
{
	currentStep = 0;
	m_keepRunning = true;

	timeNotWritten++;
	if (timeNotWritten >= 5) {
		NetworkWriter::writeToFile(creatures, generation, m_overSeed);
		//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[0] << "\n";
		//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[1] << "\n";
		//std::cout << creatures[0]->getNeuralNetwork().getLayers()[0][0].getOutputWeights()[2] << "\n";
		timeNotWritten = 0;
	}

	//for (int i = 0; i < creatures.size(); i++) {
	//	double tempFit = evaluateFitness(creatures[i]);
	//	creatures[i]->setFitness(tempFit);
	//}

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
	NetworkWriter::writeFitness(bestFit);

	std::vector<Creature*> newGeneration;

	pm->reset();
	double divider = 2.5;
	int partSize = creatures.size() / divider;
	int numParentCreatures = 5;
	int crossInd = 0;
	for (int i = 0; i < creatures.size(); i++) {


		std::default_random_engine &rand = m_randomEngines[i];
		Creature* tempCret = new Creature(pm, glm::vec3(1.0, 10.0, 0.0), rand);
		
		if (i == 0) {
			tempCret->setColor(glm::vec3(0.8f, 0.1f, 0.6f));
		}
		//else if (i == 1) {
		//	tempCret->setColor(glm::vec3(0.1f, 0.8f, 0.1f));
		//}

		//if (i != creatures.size() - 1) {
			//if (i < partSize) {
			//	tempCret->setNeuralNetwork(NeuralNetwork(creatures[i]->getNeuralNetwork()));
			//} else {
				//if (i - partSize == 1) {
				//	tempCret->setNeuralNetwork(crossOver(&creatures[0]->getNeuralNetwork(), &creatures[i + 1 - partSize]->getNeuralNetwork()));
				//}
				//else {
					//tempCret->setNeuralNetwork(crossOver(&creatures[i - partSize]->getNeuralNetwork(), &creatures[i + 1 - partSize]->getNeuralNetwork()));
				//}

			//}
		//}

		if (i <= 1) {
			tempCret->setNeuralNetwork(NeuralNetwork(creatures[i]->getNeuralNetwork()));
		}
		else {
			/*int parentAind = rand() % (numParentCreatures + 1);*/
			std::uniform_int_distribution<int> distribution(0, i);
			double prob = distribution(m_overEngine);

			//int parentAind = rand() % (i + 1);
			//int parentBind = rand() % (i + 1);

			int parentAind = distribution(m_overEngine);
			int parentBind = distribution(m_overEngine);
			while (parentAind == parentBind) {
				parentAind = distribution(m_overEngine);
				parentBind = distribution(m_overEngine);
			}
			tempCret->setNeuralNetwork(crossOver(&creatures[parentAind]->getNeuralNetwork(), &creatures[parentBind]->getNeuralNetwork()));

		}
		newGeneration.push_back(tempCret);


		crossInd++;
		if (crossInd > numParentCreatures) {
			crossInd = 0;
		}
		//Creature* oldCret = creatures[i];
		//creatures[i] = tempCret;
		//tempCret = nullptr;
		//oldCret->removeConstraints(pm);
		//oldCret->removeBodies(pm);
		//delete oldCret;
		//creatures[i]->~Creature();
		//creatures[i]->setNeuralNetwork(fitCret[j]->getNeuralNetwork());
		if (i != 0) {
			//double muRate = rand() % 10;
			//double muRate = ((double)rand() / (RAND_MAX));
			mutate(newGeneration[i], m_mutationRate, m_mutationChance, m_randomEngines[i]);
		}
	}

	for (int i = 0; i < creatures.size(); i++) {
		Creature* oldCret = creatures[i];
		delete oldCret;
	}
	creatures = newGeneration;

	if (lastFitness == bestFit) {
		std::cout << "times are not changin\': " << timesNoImprovement <<"\n";
		timesNoImprovement++;
	} else {
		timesNoImprovement = 0;
	}
	lastFitness = bestFit;
	if(timesNoImprovement >= 3) {
		if (m_mutationRate < 0.3) {
			m_mutationRate *= 1.1;
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
	//double distancex = pow((end.x - start.x),2);
	//double distancey = pow((end.z - start.z),2);

	//return sqrt(distancex + distancey);
	return start.z - end.z;
	//return glm::distance(end, start);
}

bool GeneticAlgorithm::keepRunning()
{
	return m_keepRunning;
}

std::vector<unsigned> GeneticAlgorithm::getSeedsForCreatures()
{
	std::vector<unsigned> seeds;
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_int_distribution<int> distribution(0, 2147483647);

	for (int i = 0; i < m_populationSize; i++) {
		seeds.push_back(distribution(generator));
	}


	return seeds;
}

NeuralNetwork GeneticAlgorithm::crossOver(NeuralNetwork * parent, NeuralNetwork * crossOverRecipient)
{

	std::vector<Layer> lA = parent->getLayers();
	std::vector<Layer> lB = crossOverRecipient->getLayers();
	//lA[1][1].setOutputVal(59);
	//std::cout << "lA: " << lA[1][1].getOutputVal() << "\n";
	//std::cout << "lA: " << parent->getLayers()[1][1].getOutputVal() << "\n";

	std::uniform_real_distribution<double> distribution(0, 1);
	double prob = distribution(m_overEngine);
	//std::cout << "crossover: " << prob << "\n";
	NeuralNetwork child;
	if (prob < m_crossoverProb) {
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
				if (distribution(m_overEngine) > 0.5) {
					lB[i][j] = lA[i][j];
				}
			}
		}



		child = NeuralNetwork(lB);
	}
	else {
		double parentProb = distribution(m_overEngine);
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
	int numSteps = creature->getNumerOfSteps();
	double distanceWalked = getDistanceWalked(creature);
	double averageHeight = creature->getAverageHeight();
	double maxHeight = creature->getMaxHeight();

	double averageFeetDistance = creature->getAverageFeetStartPos() - creature->getAverageFeetPosition();
	////std::cout << creature->getTimeOnGround() << "\n";
	//double groundTimeN = Util::normalizeSigned(creature->getTimeOnGround(), 0, 100);
	double groundTimeN = creature->getTimeOnGround();
	//double standTimeN = Util::normalizeSigned(creature->getTimeOnTwoLegs(), 0, 100);
	double standTimeN = creature->getTimeOnTwoLegs();
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
	//std::cout << "numSteps: " << numSteps << "\n";
	//std::cout << "distanceWalked: " << distanceWalked << "\n";
	//std::cout << "averageHeight: " << averageHeight << "\n";
	//std::cout << "averageFeetDistance: " << averageFeetDistance << "\n";
	double fitness = distanceWalked * averageHeight * numSteps;
	//if (distanceWalked < 0 && averageFeetDistance < 0) {
	//	fitness *= -1;
	//}
	return fitness;
}

void GeneticAlgorithm::mutate(Creature* creature, double mutationRate, double mutationChance, std::default_random_engine engine)
{
	creature->mutate(mutationRate, mutationChance, engine);
}


void GeneticAlgorithm::updateCreatures(Shader shader, bool render, PhysicsManager* pm)
{
	currentStep++;

	const int NUM_THREADS = 10;
	std::thread threads[NUM_THREADS];
	int i;

	for (i = 0; i < NUM_THREADS; i++) {
		threads[i] = std::thread(updateCreature, shader, creatures[i]);

	}
	for (int i = 0; i < NUM_THREADS; i++) {
		threads[i].join();
	}

	bool allCreaturesStanding = false;
	//creatures[0]->checkIfLegsCrossed();
	for (int i = 0; i < creatures.size(); i++) {

		//creatures.at(i)->activate();

		//creatures.at(i)->updatePhysics();

		//creatures.at(i)->incrementToAverage();
		if (currentStep > 100) {
			creatures[i]->updateMaxHeight(creatures[i]->getHeight());

		}
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

		if (creatures[i]->getHeight() > 3.6) {
			allCreaturesStanding = true;
		}

		if (render) {
			creatures[i]->render(shader);
		}
		else {
			creatures[0]->render(shader);
		}
	}

	if (!allCreaturesStanding) {
		m_keepRunning = false;
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

	if (creature->getHeight() > 3.6) {
		creature->setFitness(evaluateFitness(creature));
	}
}


GeneticAlgorithm::~GeneticAlgorithm()
{
}
