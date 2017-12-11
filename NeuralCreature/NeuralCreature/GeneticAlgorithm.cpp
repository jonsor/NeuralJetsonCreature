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

	creatureType = DOG;
	initCreatures(pm);
	generation = 0;
	timesNoImprovement = 0;
	lastFitness = 0;
	timeNotWritten = 0;

	m_currentStep = 0;
	allTimeBestFitness = -100000;
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
	NEATGene neatGene(1, 1, 2, 1, true);
	//int numInputs, int numOutputs, double sameSpeciesThreshold, double disjointCoefficient, double excessCoefficient, double averageWeightDifferenceCoefficient
	NEATController controller(10, 10, 0.4f, 0.3f, 0.2f, 0.5f);
	//NEATController controller, int netId, int numInputs, int numOutputs
	NEATNetwork(controller, 1, 10, 10);
	//NEATNetwork int id, double outputValue
	NEATNeuron neuron(1, 2);
	//NEATNode int id, int type
	NEATNode(1, 1);
	if (creatureType == DOG) {
		targetPos = { 2, 1.6, 200 };
		targetPosBox = new Box(targetPos, glm::vec3(0.1f, 0.1f, 0.9f), 1.f, 1.0f, 1.f, 0);
	}
	else if (creatureType == BIPED) {
		targetPos = { 2, 3.6, -200 };
		targetPosBox = new Box(targetPos, glm::vec3(0.1f, 0.1f, 0.9f), 1.f, 1.0f, 1.f, 0);
	}
}

void GeneticAlgorithm::initCreatures(PhysicsManager* pm)
{

	for (int i = 0; i < m_populationSize; i++) {
		
		if (creatureType == BIPED) {
			std::default_random_engine &rand = m_randomEngines[i];
			//5.0*i - m_populationSize*5.0/2
			Biped* tempCret = new Biped(pm, glm::vec3(1.0, 9.0, 0.0), rand);
			bipeds.push_back(tempCret);
		} else {
			std::default_random_engine &rand = m_randomEngines[i];
			//5.0*i - m_populationSize*5.0/2
			Dog* tempCret = new Dog(pm, glm::vec3(1.0, 5.5, 0.0), rand);
			dogs.push_back(tempCret);
		}

	}
	if (creatureType == BIPED) {
		NetworkWriter<Biped>::readFromFile(bipeds);
	}
	else {
		NetworkWriter<Dog>::readFromFile(dogs);
	}
}

bool moreThanByFitness(Biped* lhs, Biped* rhs) { return (lhs->getFitness() > rhs->getFitness()); }
bool moreThanByFitnessDog(Dog* lhs, Dog* rhs) { return (lhs->getFitness() > rhs->getFitness()); }


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

	double bestFit = 0;
	int bestFitIndex = 0;
	if (creatureType == BIPED) {
		timeNotWritten++;
		if (timeNotWritten >= 5) {
			NetworkWriter<Biped>::writeToFile(bipeds, generation, m_overSeed);
			timeNotWritten = 0;
		}

		for (int i = 0; i < bipeds.size(); i++) {
			if (bipeds[i]->shouldUpdate()) {
				bipeds[i]->setFitness(evaluateFitnessBiped(bipeds[i], fitnessType, 1500));
				bipeds[i]->setShouldUpdate(false);
			}
		}
		std::sort(bipeds.begin(), bipeds.end(), moreThanByFitness);
		std::cout << "sorted vector: " << std::endl;
		for (int i = 0; i < bipeds.size(); i++) {
			std::cout << bipeds[i]->getFitness() << std::endl;
		}

		bestFit = bipeds[0]->getFitness();
		std::cout << "BestFit: " << bestFit << std::endl;
		std::cout << "distanceWalked: " << bipeds[0]->getDistanceWalked() << "\nnumSteps: " << bipeds[0]->getNumerOfSteps() + 1 << "\nrotationAmount: " << bipeds[0]->getRotationAmount() + 1 << "\n";
		std::cout << "allTimeBestFitness: " << allTimeBestFitness << std::endl;

		for (int i = 0; i < bipeds.size(); i++) {
			bipeds[i]->removeConstraints(pm);

			bipeds[i]->removeBodies(pm);
		}
		//NetworkWriter::writeFitness(bestFit);
		//NetworkWriter::writeDistance(bipeds[0]->getDistanceWalked());

	}else{
		timeNotWritten++;
		//if (timeNotWritten >= 5) {
			NetworkWriter<Dog>::writeToFile(dogs, generation, m_overSeed);
			timeNotWritten = 0;
		//}

		for (int i = 0; i < dogs.size(); i++) {
			if (dogs[i]->shouldUpdate()) {
				dogs[i]->setFitness(evaluateFitnessDog(dogs[i], fitnessType, 1500, timesNoImprovement));
				dogs[i]->setShouldUpdate(false);
			}
		}
		std::sort(dogs.begin(), dogs.end(), moreThanByFitnessDog);
		std::cout << "network: \n";
		//dogs[0]->getNEATNeuralNetwork().printNetwork();
		std::cout << "sorted vector: " << std::endl;
		for (int i = 0; i < dogs.size(); i++) {
			std::cout << dogs[i]->getFitness() << std::endl;
		}

		bestFit = dogs[0]->getFitness();
		std::cout << "BestFit: " << bestFit << std::endl;
		std::cout << "distanceWalked: " << dogs[0]->getDistanceWalked() << "\nrotationAmount: " << dogs[0]->getRotationAmount() + 1 << "\n";
		std::cout << "allTimeBestFitness: " << allTimeBestFitness << std::endl;

		for (int i = 0; i < dogs.size(); i++) {
			dogs[i]->removeConstraints(pm);

			dogs[i]->removeBodies(pm);
		}
		//NetworkWriter::writeFitness(bestFit);
		//NetworkWriter::writeDistance(bipeds[0]->getDistanceWalked());
	}

	std::vector<Dog*> newGenerationDog;
	std::vector<Biped*> newGenerationBiped;

	pm->reset();

	if (creatureType == BIPED) {
		double divider = 2.5;
		//int oldPopulationSize = bipeds.size();
		int indexForOld = 0;
		int partSize = m_populationSize / divider;
		int numParentCreatures = 5;
		int crossInd = 0;
		//if (m_populationSize > oldPopulationSize) {
		//	std::uniform_int_distribution<int> overDistribution(0, INT_MAX); //INT_MIN instead of 0?
		//	m_randomEngines = std::vector<std::default_random_engine>();

		//	for (int i = 0; i < m_populationSize - oldPopulationSize; i++) {
		//		int randomSeed = overDistribution(m_overEngine);
		//		m_randomEngines.push_back(std::default_random_engine(randomSeed));
		//	}
		//}
		//std::cout << "dogs: " << bipeds.size() << " ,pop: " << m_populationSize << "\n";
		for (int i = 0; i < bipeds.size(); i++) {


			std::default_random_engine &rand = m_randomEngines[i];
			Biped* tempCret = new Biped(pm, glm::vec3(1.0, 9.0, 0.0), rand);

			if (i == 0) {
				tempCret->setColor(glm::vec3(0.8f, 0.1f, 0.6f));
			}

			if (i <= 0) {
				tempCret->setNeuralNetwork(RecurrentNeuralNetwork(bipeds[i]->getNeuralNetwork()));
			} else {
				int index = i;

				//Smaller fraction of potential candidates for cross over
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

			//indexForOld++;
			//if (indexForOld >= oldPopulationSize) {
			//	indexForOld = 1;
			//}

			newGenerationBiped.push_back(tempCret);

			crossInd++;
			if (crossInd > numParentCreatures) {
				crossInd = 0;
			}

			if (i != 0 || timesNoImprovement > 1000) {
				//double muRate = rand() % 10;
				//double muRate = ((double)rand() / (RAND_MAX));
				//std::vector<std::vector<Neuron>> tempNetLayers = newGeneration[i]->getNeuralNetwork().getLayers();
				//Layer& tempLayer = tempNetLayers[0];
				//std::vector<double> outputWeights = tempLayer[0].getOutputWeights();
				//std::cout << i << ": " << outputWeights[0];
				double mutationRate = m_mutationRate / (timesNoImprovement+1) * (timesNoImprovement + 1);
				//double mutationChance = m_mutationChance / (timesNoImprovement+1);
				if (mutationRate < 0.0000001f) {
					std::cout << "mutationRate: " << mutationRate << "\n";
					mutationRate = 0.0000001f;
				}
				newGenerationBiped[i]->mutate(mutationRate, m_mutationChance, m_randomEngines[i]);
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
		bipeds = newGenerationBiped;

		std::uniform_real_distribution<double> realDist(0, 1);
		double prob1 = realDist(m_overEngine);
		double prob2 = realDist(m_overEngine);

		std::uniform_real_distribution<double> distribution(-10, 10);
		std::uniform_real_distribution<double> distribution2(60, 100);

		if (prob1 < 0.5f) {

		}
		double targetAddX = distribution(m_overEngine);
		double targetAddZ = distribution2(m_overEngine);
		//targetAddX = (prob1 < 0.5f) ? targetAddX*-1 : targetAddX;
		targetAddZ = (prob2 < 0.5f) ? targetAddZ*-1 : targetAddZ;
		targetPos[0] = targetAddX;
		//targetPos[2] = targetAddZ;
		targetPosBox->setPosition(targetPos);
		double prob = distribution(m_overEngine);
		for (int i = 0; i < bipeds.size(); i++) {
			bipeds[i]->setTargetPosition(targetPos);
		}

	}
	else {
		double divider = 1.5;
		int partSize = dogs.size() / divider;
		int viableIndex = 0;
		int index = 0;
		std::vector<int> taken;
		//std::uniform_real_distribution<double> distribution(0, 1);
		//double height = distribution(m_overEngine);
		for (int i = 0; i < dogs.size(); i++) {


			std::default_random_engine &rand = m_randomEngines[i];

			Dog* tempCret = new Dog(pm, glm::vec3(1.0, 5.5, 0.0), rand);

			if (i == 0) {
				tempCret->setColor(glm::vec3(0.8f, 0.1f, 0.6f));
			}

			if (i <= 0) {
				std::cout << "Elite transfer\n";
				tempCret->setNeuralNetwork(RecurrentNeuralNetwork(dogs[i]->getNeuralNetwork()));
				//tempCret->setNEATNeuralNetwork(NEATNetwork(dogs[i]->getNEATNeuralNetwork()));
			}
			else {
				std::uniform_real_distribution<double> indDist(0, 1);
				double indDistNum = indDist(m_overEngine);

				index++;
				if (index >= partSize) {
					index = 1;
				}
				std::uniform_int_distribution<int> distribution(0, index);
				//std::uniform_int_distribution<int> distribution(0, index);
				double prob = distribution(m_overEngine);
				int parentAind = 0;
				int parentBind = 0;
				bool takenBool = false;
				//std::cout << "Crossover...\n";
				do {
					takenBool = false;
					parentAind = distribution(m_overEngine);
					parentBind = distribution(m_overEngine);
					for (int i = 0; i < taken.size(); i++) {
						if (taken[i] == parentAind) {
							takenBool = true;
							//std::cout << "takenBool: " << parentAind << "\n";
						}
					}
					//if (!takenBool) {
					//	std::cout << "push_back: " << parentAind << "\n";
					//	taken.push_back(parentAind);
					//}
					//std::cout << "taken.size(): " << taken.size() <<"  " << index << "\n";
					if (taken.size() >= index) {
						taken.clear();
						//std::cout << "clearclearclear(): " << taken.size() << "  " << index << "\n";
					}
				} while (parentAind == parentBind || takenBool);
				//std::cout << parentAind << " " << parentBind << " | ";
				//std::cout << "parentAind: " << parentAind << "\n";
				taken.push_back(parentAind);
				tempCret->setNeuralNetwork(crossOver(&dogs[parentAind]->getNeuralNetwork(), &dogs[parentBind]->getNeuralNetwork()));

				//std::geometric_distribution<int> expoDist(0.2);
				//std::exponential_distribution<double> expoDist(1.0);
				//double randExpo = expoDist(m_overEngine);
				//int ind = randExpo * dogs.size();
				//std::cout << "randExpo: " << randExpo << " ind: " << ind << "\n";
				//tempCret->setNEATNeuralNetwork(NEATNetwork(dogs[index]->getNEATNeuralNetwork()));

				viableIndex++;
				if (viableIndex >= partSize) {
					viableIndex = 0;
				}

			}
			newGenerationDog.push_back(tempCret);


			if (i != 0 || timesNoImprovement > 50) {
				double bestFitModified = bestFit;
				if (bestFitModified < 0) {
					bestFitModified = 1;
				}
				double mutationRate = m_mutationRate/ (timesNoImprovement+1);
				double mutationChance = m_mutationChance;
				//if(i == 1) std::cout << "mutationRate: " << mutationRate << " mutationChance: " << mutationChance << "\n";

				std::uniform_real_distribution<double> distribution(0, 1);
				double prob = distribution(m_overEngine);

				//if (prob > 0.4) {
					newGenerationDog[i]->mutate(mutationRate, mutationChance, m_randomEngines[i]);
				//}
			}

			if (i == 0) {
				allTimeBestFitness = (allTimeBestFitness < bestFit) ? bestFit : allTimeBestFitness;
			}
		}

		for (int i = 0; i < dogs.size(); i++) {
			Dog* oldCret = dogs[i];
			delete oldCret;
		}
		dogs = newGenerationDog;

		newGenerationDog.clear();

		//if (generation % 2 == 0) {
			//glm::vec3 prevTarget = dogs[0]->getTargetPosition();
		std::uniform_real_distribution<double> realDist(0, 1);
		double prob1 = realDist(m_overEngine);
		double prob2 = realDist(m_overEngine);

			std::uniform_real_distribution<double> distribution(60, 100);
			//std::uniform_real_distribution<double> distribution(-10, 10);
			std::uniform_real_distribution<double> distribution2(60, 100);

			if (prob1 < 0.5f) {

			}
			double targetAddX = distribution(m_overEngine);
			double targetAddZ = distribution2(m_overEngine);
			targetAddX = (prob1 < 0.5f) ? targetAddX*-1 : targetAddX;
			targetAddZ = (prob2 < 0.5f) ? targetAddZ*-1 : targetAddZ;
			targetPos[0] = targetAddX;
			targetPos[2] = targetAddZ;
			targetPosBox->setPosition(targetPos);
			double prob = distribution(m_overEngine);
			for (int i = 0; i < dogs.size(); i++) {
				dogs[i]->setTargetPosition(targetPos);
			}
		//}
	}


	//glm::vec3 targetPos = { 20, 1.6, 50 };
	//Box targetPosBox(targetPos, glm::vec3(0.1f, 0.1f, 0.9f), 1.f, 1.0f, 1.f, 0);


	if (lastFitness == bestFit) {
		std::cout << "times are not changin\': fix this, its set to 1000:  " << timesNoImprovement <<"\n";
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
	m_populationSize += 10;
	std::cout << "Poulation Size: " << m_populationSize << "\n";

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

RecurrentNeuralNetwork GeneticAlgorithm::crossOver(RecurrentNeuralNetwork * parent, RecurrentNeuralNetwork * crossOverRecipient)
{

	std::vector<Layer> lA = parent->getLayers();
	std::vector<Layer> lB = crossOverRecipient->getLayers();
	std::vector<int> topology = crossOverRecipient->getTopology();
	//lA[1][1].setOutputVal(59);
	//std::cout << "lA: " << lA[1][1].getOutputVal() << "\n";
	//std::cout << "lA: " << parent->getLayers()[1][1].getOutputVal() << "\n";

	std::uniform_real_distribution<double> distribution(0, 1);
	double prob = distribution(m_overEngine);
	//std::cout << "crossover: " << prob << "\n";
	RecurrentNeuralNetwork child;
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

		//Random choice crossover
		//for (int i = 0; i < lA.size(); i++) {
		//	for (int j = 0; j < lA[i].size(); j++) {
		//		if (distribution(m_overEngine) > 0.5) {
		//			lB[i][j] = lA[i][j];
		//		}
		//	}
		//}

		//Random point crossover
		std::uniform_int_distribution<int> layerPointDist(0, lB.size() - 1);
		double layerPoint = layerPointDist(m_overEngine);

		std::uniform_int_distribution<int> nodePointDist(0, lB[layerPoint].size() - 1);
		double nodePoint = nodePointDist(m_overEngine);

		for (int i = layerPoint; i < lB.size(); i++) {
			if (i == layerPoint) {
				for (int j = nodePoint; j < lB[i].size(); j++) {
					lB[i][j] = lA[i][j];
				}
			}
			else {
				for (int j = 0; j < lB[i].size(); j++) {
					lB[i][j] = lA[i][j];
				}
			}
		}

		std::uniform_real_distribution<double> dist(0, 1);
		double probDivider = dist(m_overEngine);

		//child = (probDivider > 0.5) ? RecurrentNeuralNetwork(lB, topology, crossOverRecipient->getDivider()) : RecurrentNeuralNetwork(lA, topology, parent->getDivider());
		child = RecurrentNeuralNetwork(lA, topology, crossOverRecipient->getDivider());
		//child = RecurrentNeuralNetwork(lB, lBTopology);
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


double GeneticAlgorithm::evaluateFitnessBiped(Biped* creature, int fitnessType, int currentStep)
{
	double numSteps = creature->getNumerOfSteps() + 1;
	double distanceWalked = creature->getDistanceWalked();
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
		//fitness = numSteps*6 + distanceWalked * 2 - rotationAmount;
		fitness = -distanceWalked;
		if (creature->isTargetReached()) {
			fitness += 100;
		}
		//fitness = numSteps + distanceWalked ;
		//std::cout << "numsteps: " << numSteps << "\n";
	}else if (fitnessType == FITNESS_JUMPING) {
		fitness = maxHeight;
	}else if (fitnessType == FITNESS_STANDING) {
		fitness = averageHeight * (double)currentStep / feetDistanceFromEachOther;
	}

	/*std::cout << "distanceWalked: " << distanceWalked << "\naverageHeight: " << averageHeight << "\nnumSteps: " << numSteps << "\nrotationAmount: " << rotationAmount << "\nnoMovementPenalty: " << noMovementPenalty << "\n";*/
	
	return fitness;
}

double GeneticAlgorithm::evaluateFitnessDog(Dog* creature, int fitnessType, int currentStep, int timesNoImprovement)
{
	double numSteps = creature->getNumerOfSteps() + 1;
	double distanceWalked = creature->getDistanceWalked();
	double averageHeight = creature->getAverageHeight();
	double maxHeight = creature->getMaxHeight();
	double rotationAmount = creature->getRotationAmount() + 1;
	double timeAlive = creature->getTimeAlive();
	double noMovementPenalty = creature->getNoMovementPenalty() + 1;
	double jointsAtlimitPenalty = creature->getJointsAtlimitPenalty();
	double groundTimeN = creature->getTimeOnGround();
	double tighMovement = creature->getTighMovement();
	///SEE HERE CALCULATE FITNESS FOR DESIRED HEIGHT
	double fitness = 0;
	//creature->getPosition();
	//std::cout << "x: " << creature->getPosition()[0] << "y: " << creature->getPosition()[1] << "z: " << creature->getPosition()[2] << "\n";
	if (fitnessType == FITNESS_WALKING) {
		//if (timesNoImprovement == 0) {
		//	fitness = distanceWalked - rotationAmount;
		//} else if (timesNoImprovement == 1) {
		//	fitness = distanceWalked + timeAlive;                                                          
		//} else {
		//	fitness = distanceWalked + averageHeight*2;
		//}
		//std::cout << "jointsAtlimitPenalty: " << jointsAtlimitPenalty << "\n";
		//std::cout << "tighMovement: " << tighMovement << "\n";
		fitness = -distanceWalked;
		if (creature->isTargetReached()) {
			fitness += 100;
		}
		//std::cout << "fitness: " << fitness << "\n";
	
		// - timeAlive/100
		/*std::cout << "fitness: " << fitness << " rotationAmount: " << rotationAmount << "\n";*/
	}
	else if (fitnessType == FITNESS_JUMPING) {
		fitness = maxHeight;
	}
	else if (fitnessType == FITNESS_STANDING) {
		fitness = averageHeight * timeAlive;
	}

	//std::cout << "divider: " << creature->getNeuralNetwork().getDivider() << "\n";
	/*std::cout << "distanceWalked: " << distanceWalked << "\naverageHeight: " << averageHeight << "\nnumSteps: " << numSteps << "\nrotationAmount: " << rotationAmount << "\nnoMovementPenalty: " << noMovementPenalty << "\n";*/

	return fitness;
}

const int NUM_THREADS = 50;
std::thread threadPool[NUM_THREADS];
void GeneticAlgorithm::updateCreatures(Shader shader, bool render, PhysicsManager* pm)
{

	targetPosBox->render(shader);

	m_currentStep++;
	int max_threads = std::thread::hardware_concurrency();
	int i;

	if (creatureType == BIPED) {
		for (i = 0; i < bipeds.size(); i++) {
			threadPool[i] = std::thread(updateBiped, bipeds[i], fitnessType, m_currentStep);
		}
		for (int i = 0; i < bipeds.size(); i++) {
			threadPool[i].join();
		}
	}
	else {
		//timesNoImprovement = rand() % 3;
		for (i = 0; i < dogs.size(); i++) {
			threadPool[i] = std::thread(updateDog, dogs[i], fitnessType, m_currentStep, timesNoImprovement);
		}
		for (int i = 0; i < dogs.size(); i++) {
			threadPool[i].join();
		}
	}

	bool allCreaturesStanding = false;
	//bipeds[0]->checkIfLegsCrossed();
	if (creatureType == BIPED) {
		for (int i = 0; i < bipeds.size(); i++) {
			if (bipeds[i]->shouldUpdate()) {
				//bipeds.at(i)->activate();

				//bipeds.at(i)->updatePhysics();

				//bipeds.at(i)->incrementToAverage();
				if (m_currentStep > 100) {
					bipeds[i]->updateMaxHeight(bipeds[i]->getHeight());

				}
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

				if (bipeds[i]->getHeight() >= 5.0f) {
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
			}
			else {
				//TODO: THIS CAUSES CRASH ON NEW GENERATION
				//bipeds[i]->removeConstraints(pm);
				//bipeds[i]->removeBodies(pm);
			}
		}
	}
	else {
		for (int i = 0; i < dogs.size(); i++) {
			if (dogs[i]->shouldUpdate()) {
				//if (m_currentStep > 100) {
				//	dogs[i]->updateMaxHeight();

				//}

				dogs[i]->getFrontRightShin()->setCollidingWithGround(false);
				dogs[i]->getFrontLeftShin()->setCollidingWithGround(false);

				dogs[i]->getBackRightShin()->setCollidingWithGround(false);
				dogs[i]->getBackLeftShin()->setCollidingWithGround(false);
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
						if (pt.getDistance() < 0.05f)
						{
							const btVector3& ptA = pt.getPositionWorldOnA();
							const btVector3& ptB = pt.getPositionWorldOnB();
							const btVector3& normalOnB = pt.m_normalWorldOnB;

							//std::cout << "contact point impulse: " << pt.getAppliedImpulse() << "\n";

							dogs[i]->getFrontRightShin()->setCollisionImpulse(0.f);
							dogs[i]->getFrontLeftShin()->setCollisionImpulse(0.f);
							dogs[i]->getBackRightShin()->setCollisionImpulse(0.f);
							dogs[i]->getBackLeftShin()->setCollisionImpulse(0.f);
							if ((btRigidBody*)obA == dogs[i]->getFrontRightShin()->getRigidBody() || (btRigidBody*)obB == dogs[i]->getFrontRightShin()->getRigidBody())
							{
								dogs[i]->getFrontRightShin()->setCollidingWithGround(true);
								dogs[i]->getFrontRightShin()->setCollisionImpulse(pt.getAppliedImpulse());
								//std::cout << "front shin col imp: " << dogs[i]->getFrontRightShin()->getCollisionImpulse() << "\n";
							}

							if ((btRigidBody*)obA == dogs[i]->getFrontLeftShin()->getRigidBody() || (btRigidBody*)obB == dogs[i]->getFrontLeftShin()->getRigidBody())
							{
								dogs[i]->getFrontLeftShin()->setCollidingWithGround(true);
								dogs[i]->getFrontLeftShin()->setCollisionImpulse(pt.getAppliedImpulse());
							}

							if ((btRigidBody*)obA == dogs[i]->getBackRightShin()->getRigidBody() || (btRigidBody*)obB == dogs[i]->getBackRightShin()->getRigidBody())
							{
								dogs[i]->getBackRightShin()->setCollidingWithGround(true);
								dogs[i]->getBackRightShin()->setCollisionImpulse(pt.getAppliedImpulse());
							}

							if ((btRigidBody*)obA == dogs[i]->getBackLeftShin()->getRigidBody() || (btRigidBody*)obB == dogs[i]->getBackLeftShin()->getRigidBody())
							{
								dogs[i]->getBackLeftShin()->setCollidingWithGround(true);
								dogs[i]->getBackLeftShin()->setCollisionImpulse(pt.getAppliedImpulse());
							}

						}
					}
				}


				if (dogs[i]->getHeight() >= 2.5f) {
					allCreaturesStanding = true;
				}

				if (render) {
					if (dogs[i]->shouldUpdate()) {
						dogs[i]->render(shader);
					}
				}
				else {
					dogs[0]->render(shader);
				}
			}
		}
	}
		if (!allCreaturesStanding) {
			m_keepRunning = false;
		}
}
void GeneticAlgorithm::updateBiped(Biped* creature, int fitnessType, int currentStep)
{
	if (creature->shouldUpdate()) {
		//std::cout << "update Biped\n";
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

		if (creature->getHeight() < 5.0f) {
			creature->setFitness(evaluateFitnessBiped(creature, fitnessType, currentStep));
			creature->setShouldUpdate(false);
		}
	}
}

void GeneticAlgorithm::updateDog(Dog* creature, int fitnessType, int currentStep, int timesNoImprovement)
{
	if (creature->shouldUpdate()) {
		creature->activate();
		creature->calculateTighMovement();
		creature->updatePhysics();
		creature->incrementToAverageHeight();
		creature->checkRotation();
		creature->checkIfMoving();
		creature->calculateSpeed();
		creature->checkIfJointsAtLimit();
		creature->checkIfTargetReached();
		creature->updateMaxHeight();
		if (creature->getHeight() < 2.f) {
			creature->setTimeOnGround(creature->getTimeOnGround() + 0.1);
		}
		//creature->checkIfLegsCrossed();
		//|| creature->noMovement(currentStep)
		if (creature->getHeight() < 2.5f) {
			creature->setFitness(evaluateFitnessDog(creature, fitnessType, currentStep, timesNoImprovement));
			creature->setShouldUpdate(false);
			creature->disableSimulation();
			//std::cout << "height: " << creature->getCenterOfMassHeight() << " no movement: " << creature->noMovement(currentStep) << "\n";
		}
	}
}


GeneticAlgorithm::~GeneticAlgorithm()
{
}
