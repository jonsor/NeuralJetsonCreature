/**
GeneticAlgorithm.cpp
Purpose: Sets up generations, simulates evolution.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "GeneticAlgorithm.h"

GeneticAlgorithm::GeneticAlgorithm(double mutationRate, double mutationChance, double crossoverProb, const int populationSize, int numElites, PhysicsManager* pm)
	: m_mutationRate(mutationRate), m_crossoverProb(crossoverProb), m_populationSize(populationSize), m_numElites(numElites), m_mutationChance(mutationChance), initalMutationRate(mutationRate)
{
	//Random Setup:
	m_overSeed = std::chrono::system_clock::now().time_since_epoch().count();
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

	m_currentStep = 0;
	allTimeBestFitness = 0;
	m_keepRunning = true;

	fitnessType = FITNESS_WALKING;

	if (fitnessType == FITNESS_WALKING) {
		std::cout << "Fitness type: walking\n";
	}
	else if (fitnessType == FITNESS_JUMPING) {
		std::cout << "Fitness type: jumping\n";

	}
	else if (fitnessType == FITNESS_STANDING) {
		std::cout << "Fitness type: standing\n";

	}

}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{
	for (int i = 0; i < m_populationSize; i++) {
		
		std::default_random_engine &rand = m_randomEngines[i];
		//5.0*i - m_populationSize*5.0/2
		Biped* tempCret = new Biped(pm, glm::vec3(1.0, 12.0, 0.0), rand);
		bipeds.push_back(tempCret);

	}
	//NetworkWriter::readFromFile(bipeds);
}

bool moreThanByFitness(Biped* lhs, Biped* rhs) { return (lhs->getFitness() > rhs->getFitness()); }


void GeneticAlgorithm::createNewGeneration(PhysicsManager * pm) //TODO: FIKS MINNELEKASJE HER:
{

	//std::cout << "Number of steps: \n";

	//for (int i = 0; i < bipeds.size(); i++) {
	//	double numSteps = bipeds[i]->getNumerOfSteps() + 1;
	//	double distanceWalked = getDistanceWalked(bipeds[i]);
	//	double averageHeight = bipeds[i]->getAverageHeight();
	//	double maxHeight = bipeds[i]->getMaxHeight();
	//	double timeOn = bipeds[i]->getTimeOnTwoLegs();

	//	std::cout << "numSteps: " << numSteps << " ";
	//	std::cout << "distanceWalked: " << distanceWalked << " ";
	//	std::cout << "averageHeight: " << averageHeight << "\n";
	//	std::cout << "timeOn: " << timeOn << "\n";

	//}

	m_currentStep = 0;
	m_keepRunning = true;

	timeNotWritten++;
	if (timeNotWritten >= 5) {
		NetworkWriter::writeToFile(bipeds, generation, m_overSeed);
		timeNotWritten = 0;
	}

	std::sort(bipeds.begin(), bipeds.end(), moreThanByFitness);
	std::cout << "sorted vector: " << std::endl;
	for (int i = 0; i < bipeds.size(); i++) {
		std::cout << bipeds[i]->getFitness() << std::endl;
	}

	int bestFitIndex = 0;
	double bestFit = bipeds[0]->getFitness();
	std::cout << "BestFit: " << bestFit << std::endl;
	std::cout << "distanceWalked: " << getDistanceWalked(bipeds[0]) << "\nnumSteps: " << bipeds[0]->getNumerOfSteps() + 1 << "\nrotationAmount: " << bipeds[0]->getRotationAmount() + 1 <<"\n";
	std::cout << "allTimeBestFitness: " << allTimeBestFitness << std::endl;

	for (int i = 0; i < bipeds.size(); i++) {
		bipeds[i]->removeConstraints(pm);

		bipeds[i]->removeBodies(pm);
	}
	NetworkWriter::writeFitness(bestFit);
	NetworkWriter::writeDistance(getDistanceWalked(bipeds[0]));

	std::vector<Biped*> newGeneration;

	pm->reset();
	double divider = 2.5;
	int partSize = bipeds.size() / divider;
	int numParentCreatures = 5;
	int crossInd = 0;
	for (int i = 0; i < bipeds.size(); i++) {


		std::default_random_engine &rand = m_randomEngines[i];
		Biped* tempCret = new Biped(pm, glm::vec3(1.0, 12.0, 0.0), rand);
		
		if (i == 0) {
			tempCret->setColor(glm::vec3(0.8f, 0.1f, 0.6f));
		}
		//else if (i == 1) {
		//	tempCret->setColor(glm::vec3(0.1f, 0.8f, 0.1f));
		//}

		//if (i != bipeds.size() - 1) {
			//if (i < partSize) {
			//	tempCret->setNeuralNetwork(NeuralNetwork(bipeds[i]->getNeuralNetwork()));
			//} else {
				//if (i - partSize == 1) {
				//	tempCret->setNeuralNetwork(crossOver(&bipeds[0]->getNeuralNetwork(), &bipeds[i + 1 - partSize]->getNeuralNetwork()));
				//}
				//else {
					//tempCret->setNeuralNetwork(crossOver(&bipeds[i - partSize]->getNeuralNetwork(), &bipeds[i + 1 - partSize]->getNeuralNetwork()));
				//}

			//}
		//}

		/*if (i <= 1) {*/
		if (i <= 0) {
			tempCret->setNeuralNetwork(NeuralNetwork(bipeds[i]->getNeuralNetwork()));
		}
		else {
			int index = i;
			if (index >= partSize) {
				index = partSize;
			}
			std::uniform_int_distribution<int> distribution(0, index);
			double prob = distribution(m_overEngine);

			int parentAind = 0;
			int parentBind = 0;
			do {
				parentAind = distribution(m_overEngine);
				parentBind = distribution(m_overEngine);
			} while (parentAind == parentBind);

			tempCret->setNeuralNetwork(crossOver(&bipeds[parentAind]->getNeuralNetwork(), &bipeds[parentBind]->getNeuralNetwork()));
		}

		newGeneration.push_back(tempCret);

		crossInd++;
		if (crossInd > numParentCreatures) {
			crossInd = 0;
		}
		
		if (i != 0 || timesNoImprovement > 10) {
			//double muRate = rand() % 10;
			//double muRate = ((double)rand() / (RAND_MAX));
			//std::vector<std::vector<Neuron>> tempNetLayers = newGeneration[i]->getNeuralNetwork().getLayers();
			//Layer& tempLayer = tempNetLayers[0];
			//std::vector<double> outputWeights = tempLayer[0].getOutputWeights();
			//std::cout << i << ": " << outputWeights[0];
			mutate(newGeneration[i], m_mutationRate, m_mutationChance, m_randomEngines[i]);
			//std::vector<std::vector<Neuron>> tempNetLayers2 = newGeneration[i]->getNeuralNetwork().getLayers();
			//Layer& tempLayer2 = tempNetLayers2[0];
			//std::vector<double> outputWeights2 = tempLayer2[0].getOutputWeights();
			//std::cout << "  " << outputWeights2[0] << "\n";
		}

		if (i == 0) {
			allTimeBestFitness = (allTimeBestFitness < bestFit) ? bestFit : allTimeBestFitness;
		}
	}

	for (int i = 0; i < bipeds.size(); i++) {
		Biped* oldCret = bipeds[i];
		delete oldCret;
	}
	bipeds = newGeneration;

	if (lastFitness == bestFit) {
		std::cout << "times are not changin\': " << timesNoImprovement <<"\n";
		timesNoImprovement++;
	} else {
		timesNoImprovement = 0;
	}
	lastFitness = bestFit;
	//if(timesNoImprovement >= 3) {
	//	if (m_mutationChance < 0.005) {
	//		m_mutationChance *= 0.9;
	//	}
	//	if (m_mutationRate > 0.005) {
	//		m_mutationRate *= 0.9;
	//	}
	//	std::cout << "mutation rate: " << m_mutationRate << "\n";
	//} else {
	//	m_mutationRate = initalMutationRate;
	//	m_mutationChance = 0.05;
	//}


	std::cout << "Generation: " << generation  << " bipeds size: " << bipeds.size() << std::endl;
	generation++;

}

double GeneticAlgorithm::getDistanceWalked(Biped* creature)
{
	glm::vec3 end = creature->getPosition();
	glm::vec3 start = creature->getStartPosition();
	return start.z - end.z;
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
	//if (prob < m_crossoverProb) {
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
	//}
	//else {
	//	double parentProb = distribution(m_overEngine);
	//	if (parentProb >= 0.5) {
	//		child = NeuralNetwork(lA);
	//	}
	//	else {
	//		child = NeuralNetwork(lB);
	//	}
	//}

	return child;

}


double GeneticAlgorithm::evaluateFitness(Biped* creature, int fitnessType, int currentStep)
{
	double numSteps = creature->getNumerOfSteps() + 1;
	double distanceWalked = getDistanceWalked(creature);
	double averageHeight = creature->getAverageHeight();
	double maxHeight = creature->getMaxHeight();
	double rotationAmount = creature->getRotationAmount() + 1;
	double timeAlive = creature->getTimeAlive();
	double noMovementPenalty = creature->getNoMovementPenalty()+1;
	double averageFeetDistance = creature->getAverageFeetStartPos() - creature->getAverageFeetPosition();
	glm::vec3 rightFoot = creature->getRightFoot()->getPosition();
	glm::vec3 leftFoot = creature->getLeftFoot()->getPosition();

	double feetDistanceFromEachOther = glm::distance(rightFoot, leftFoot);
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

	//std::cout << "rotationAmount: " << rotationAmount << "\n";
	double fitness = 0;
	if (fitnessType == FITNESS_WALKING) {
		//fitness = ((distanceWalked*averageHeight*numSteps*timeAlive)/rotationAmount) /noMovementPenalty;
		fitness = numSteps*6 + distanceWalked * 2 - rotationAmount;
		std::cout << "numsteps: " << numSteps << "\n";
	}else if (fitnessType == FITNESS_JUMPING) {
		fitness = maxHeight;
	}else if (fitnessType == FITNESS_STANDING) {
		fitness = averageHeight * (double)currentStep / feetDistanceFromEachOther;
	}

	/*std::cout << "distanceWalked: " << distanceWalked << "\naverageHeight: " << averageHeight << "\nnumSteps: " << numSteps << "\nrotationAmount: " << rotationAmount << "\nnoMovementPenalty: " << noMovementPenalty << "\n";*/

	return fitness;
}

void GeneticAlgorithm::mutate(Biped* creature, double mutationRate, double mutationChance, std::default_random_engine engine)
{
	creature->mutate(mutationRate, mutationChance, engine);
}

const int NUM_THREADS = 30;
std::thread threadPool[NUM_THREADS];
void GeneticAlgorithm::updateCreatures(Shader shader, bool render, PhysicsManager* pm)
{
	m_currentStep++;
	int max_threads = std::thread::hardware_concurrency();
	int i;
	for (i = 0; i < NUM_THREADS; i++) {
		threadPool[i] = std::thread(updateCreature, bipeds[i], fitnessType, m_currentStep);

	}
	for (int i = 0; i < NUM_THREADS; i++) {
		threadPool[i].join();
	}

	bool allCreaturesStanding = false;
	//bipeds[0]->checkIfLegsCrossed();
	for (int i = 0; i < bipeds.size(); i++) {
		if (bipeds[i]->shouldUpdate()) {
			//bipeds.at(i)->activate();

			//bipeds.at(i)->updatePhysics();

			//bipeds.at(i)->incrementToAverage();
			if (m_currentStep > 100) {
				bipeds[i]->updateMaxHeight(bipeds[i]->getHeight());

			}
			//if (bipeds[i]->getHeight() < 2.f) {
			//	bipeds[i]->setTimeOnGround(bipeds[i]->getTimeOnGround() + 0.1);
			//}
			//if (bipeds[i]->getHeight() < 5.5f) {
			//	bipeds[i]->setTimeOnTwoLegs(bipeds[i]->getTimeOnTwoLegs() + 0.1);
			//}
			bipeds[i]->getRightFoot()->setCollidingWithGround(false);
			bipeds[i]->getLeftFoot()->setCollidingWithGround(false);
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

						if ((btRigidBody*)obA == bipeds[i]->getRightFoot()->getRigidBody() || (btRigidBody*)obB == bipeds[i]->getRightFoot()->getRigidBody())
						{
							bipeds[i]->getRightFoot()->setCollidingWithGround(true);
						}

						if ((btRigidBody*)obA == bipeds[i]->getLeftFoot()->getRigidBody() || (btRigidBody*)obB == bipeds[i]->getLeftFoot()->getRigidBody())
						{
							bipeds[i]->getLeftFoot()->setCollidingWithGround(true);
						}

					}
				}
			}

			if (bipeds[i]->getHeight() >= 7.0f) {
				allCreaturesStanding = true;
			}

			if (render) {
				if (bipeds[i]->shouldUpdate()) {
					bipeds[i]->render(shader);
				}
			}
			else {
				bipeds[0]->render(shader);
			}
		}else {
			//TODO: THIS CAUSES CRASH ON NEW GENERATION
			//bipeds[i]->removeConstraints(pm);
			//bipeds[i]->removeBodies(pm);
		}
	}

	if (!allCreaturesStanding) {
		m_keepRunning = false;
	}
}

void GeneticAlgorithm::updateCreature(Biped* creature, int fitnessType, int currentStep)
{
	if (creature->shouldUpdate()) {
		creature->activate();
		creature->updatePhysics();
		creature->incrementToAverage();
		creature->checkRotation();
		creature->checkIfMoving();
		creature->calculateSpeed();
		if (creature->getHeight() < 2.f) {
			creature->setTimeOnGround(creature->getTimeOnGround() + 0.1);
		}
		if (creature->getHeight() < 5.6f) {
			creature->setTimeOnTwoLegs(creature->getTimeOnTwoLegs() + 0.1);
		}
		//creature->checkIfLegsCrossed();

		if (creature->getHeight() < 7.0f) {
			creature->setFitness(evaluateFitness(creature, fitnessType, currentStep));
			creature->setShouldUpdate(false);
		}
	}
}


GeneticAlgorithm::~GeneticAlgorithm()
{
}
