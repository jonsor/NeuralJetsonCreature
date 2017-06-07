#include "stdafx.h"
#include "Neuron.h"

#include <iomanip>

Neuron::Neuron(int numOutputs, int neuronIndex) : neuronIndex(neuronIndex)
{
	//Set random weights on neuron outputs
	for (int con = 0; con < numOutputs; ++con) {
		outputWeights.push_back(Neuron::getRandomWeight());
		//outputWeights.push_back(0.0);
	}
}

void Neuron::setOutputVal(double value)
{
	outputVal = value;
}

double Neuron::getOutputVal() const
{
	return outputVal;
}

void Neuron::forward(Layer & prevLayer)
{
	double sum = 0.0;
	//Sum outputs of prevLayer
	for (int i = 0; i < prevLayer.size(); i++) {
		sum += prevLayer[i].getOutputVal() * prevLayer[i].outputWeights[neuronIndex];
	}

	outputVal = Neuron::activationFunction(sum);
	
	/*std::cout << std::fixed;
	std::cout << std::setprecision(3);
	if (neuronIndex == 9 && outputVal < 0) {
		std::cout << "outputVal: " << outputVal << " sum: " << sum << std::endl;
	}
	else if (neuronIndex == 9) {
		std::cout << "outputVal:  " << outputVal << " sum: " << sum << std::endl;
	}*/
	

}

void Neuron::mutate(double mutationRate, double mutationChance)
{
	//bool contains = false;
	//int numWeightsToMutate = 1;
	//std::vector<int> indiciesToMutate;
	//if (outputWeights.size() != 0) {
	//	numWeightsToMutate = rand() % outputWeights.size()/2 + 1;
	//	//std::cout << numWeightsToMutate << "\n";
	//	for (int i = 0; i < numWeightsToMutate; i++) {
	//		int tempIndex = rand() % outputWeights.size();
	//		contains = false;
	//		for (int j = 0; j < indiciesToMutate.size(); j++) {
	//			if (tempIndex == indiciesToMutate[j]) {
	//				contains = true;
	//				i--;
	//				break;
	//			}
	//		}
	//		if(!contains) indiciesToMutate.push_back(tempIndex);
	//	}
	//}

	//double mutateChance = ((double)rand() / (RAND_MAX));
	////std::cout << "size: " << outputWeights.size() << std::endl;
	//for (int i = 0; i < indiciesToMutate.size(); i++) {
	//	//if (mutateChance <= 0.2) {
	//	//	outputWeights[indiciesToMutate[i]] = getRandomWeight();
	//	//}
	//	//else {
	//		outputWeights[indiciesToMutate[i]] += getRandomWeight() * mutationRate;
	//	//}
	//}
	int ind = 0;
	for (int i = 0; i < outputWeights.size(); i++) {
		double mutRand = ((double)rand() / (RAND_MAX));
		if (mutRand <= mutationChance) {
			ind++;
			outputWeights[i] += getRandomWeight() * mutationRate;
		}
	}
}

std::vector<double> Neuron::getOutputWeights()
{
	return outputWeights;
}

void Neuron::setOutputWeights(std::vector<double> weights)
{
	outputWeights = weights;
}

double Neuron::getRandomWeight()
{

	//TODO change to <random> library
	//double ran = rand() / double(RAND_MAX);
	//std::cout << ran << std::endl;
	//return  ran;

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	//unsigned seed = 1;
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> distribution(-1.0, 1.0);
	double randomNumber = distribution(generator);
	//std::cout << randomNumber << std::endl;
	return randomNumber;
}

double Neuron::activationFunction(double value)
{
	//fast sigmoid output function
	//return value / (1 + abs(value));
	//std::cout << "outputValsssssss: " << value << std::endl;

	//Use tanh for -1 .. 1 range
	return tanh(value);
}

Neuron::~Neuron()
{
}
