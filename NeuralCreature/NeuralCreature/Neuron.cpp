/**
Neuron.cpp
Purpose: Creates a neuron

@author Jonas Sørsdal
@version 1.0 07.04.2017
*/

#include "stdafx.h"
#include "Neuron.h"

#include <iomanip>

Neuron::Neuron(int numOutputs, int neuronIndex, std::default_random_engine &engine) : neuronIndex(neuronIndex)
{
	outputVal = 0;
	//Set random weights on neuron outputs
	for (int con = 0; con < numOutputs; ++con) {
		outputWeights.push_back(Neuron::getRandomWeight(engine));
	}
}

Neuron::Neuron(int numOutputs, int neuronIndex) : neuronIndex(neuronIndex)
{
	//Set random weights on neuron outputs
	for (int con = 0; con < numOutputs; ++con) {
		outputWeights.push_back(0.f);
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

void Neuron::forward(Layer & prevLayer, int numForwards)
{
	double sum = 0.0;
	//Sum outputs of prevLayer including the bias
	//std::cout << "size: " << prevLayer.size() << "\n";
	for (int i = 0; i < prevLayer.size(); i++) {
		sum += prevLayer[i].getOutputVal() * prevLayer[i].outputWeights[neuronIndex];
		//std::cout << "o: " << prevLayer[i].getOutputVal() << " ";
		//std::cout << "w: " << prevLayer[i].outputWeights[neuronIndex] << " ";
	}
	//std::cout << "\n\n";
	outputVal = Neuron::activationFunction(sum * sin(numForwards/divider));
}

void Neuron::forward(Layer & prevLayer, int numForwards, double alternativeDivider)
{
	double sum = 0.0;
	//Sum outputs of prevLayer including the bias
	//std::cout << "size: " << prevLayer.size() << "\n";
	for (int i = 0; i < prevLayer.size(); i++) {
		sum += prevLayer[i].getOutputVal() * prevLayer[i].outputWeights[neuronIndex];
		//std::cout << "o: " << prevLayer[i].getOutputVal() << " ";
		//std::cout << "w: " << prevLayer[i].outputWeights[neuronIndex] << " ";
	}
	//std::cout << "\n\n";
	outputVal = Neuron::activationFunction(sum * sin(numForwards / alternativeDivider));
}

void Neuron::forwardRecurrent(Layer& prevLayer, int numForwards, Layer& lastTimestepPrevLayer)
{
	double sum = 0.0;
	//Sum outputs of prevLayer
	for (int i = 0; i < prevLayer.size(); i++) {
		sum += prevLayer[i].getOutputVal() * prevLayer[i].outputWeights[neuronIndex] + lastTimestepPrevLayer[i].getOutputVal() * lastTimestepPrevLayer[i].outputWeights[neuronIndex];
		//std::cout << "o: " << prevLayer[i].getOutputVal() << " ";
	}
	//std::cout << "\n\n";
	outputVal = Neuron::activationFunction(sum);
}

void Neuron::mutate(double mutationRate, double mutationChance, std::default_random_engine &engine)
{
	for (int i = 0; i < outputWeights.size(); i++) {
		double mutRand = getRandomWeight(0, 1, engine);
		if (mutRand <= mutationChance) {
			double chanceNewWeight = getRandomWeight(0, 1, engine);
			//divider += getRandomWeight(engine) * mutationRate;
			if (chanceNewWeight <= 0.05) {
				outputWeights[i] = getRandomWeight(engine);
			} else {
				outputWeights[i] += getRandomWeight(engine) * mutationRate;
			}
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

double Neuron::getRandomWeight(std::default_random_engine &generator)
{
	std::uniform_real_distribution<double> distribution(-1.0, 1.0);
	double randomNumber = distribution(generator);
	return randomNumber;
}

double Neuron::getRandomWeight(double min, double max, std::default_random_engine &generator)
{
	std::uniform_real_distribution<double> distribution(min, max);
	double randomNumber = distribution(generator);
	return randomNumber;
}

void Neuron::setOutputWeightsToZero() {
	for (int i = 0; outputWeights.size() < i; i++) {
		outputWeights[i] = 0;
	}
}

double Neuron::activationFunction(double value)
{
	//Use tanh for -1 .. 1 range

	//double sigmoid = 1 / (1 + exp(-value));
	//return (sigmoid * 2 - 1);
	return tanh(value);
}

Neuron::~Neuron()
{
}
