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
	//Set random weights on neuron outputs
	for (int con = 0; con < numOutputs; ++con) {
		outputWeights.push_back(Neuron::getRandomWeight(engine));
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
}

void Neuron::mutate(double mutationRate, double mutationChance, std::default_random_engine &engine)
{
	for (int i = 0; i < outputWeights.size(); i++) {
		double mutRand = getRandomWeight(0, 1, engine);
		if (mutRand <= mutationChance) {
			double chanceNewWeight = getRandomWeight(0, 1, engine);
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



double Neuron::activationFunction(double value)
{
	//Use tanh for -1 .. 1 range

	double sigmoid = 1 / (1 + exp(-value));
	return (sigmoid * 2 - 1);
	//return tanh(value);
}

Neuron::~Neuron()
{
}
