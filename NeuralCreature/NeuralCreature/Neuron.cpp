/**
Neuron.cpp
Purpose: Creates a neuron

@author Jonas Sørsdal
@version 1.0 07.04.2017
*/

#include "stdafx.h"
#include "Neuron.h"

#include <iomanip>

Neuron::Neuron(int numOutputs, int neuronIndex, std::default_random_engine &engine) : m_neuronIndex(neuronIndex)
{
	m_outputVal = 0;
	//Set random weights on neuron outputs
	for (int con = 0; con < numOutputs; ++con) {
		m_outputWeights.push_back(Neuron::getRandomWeight(engine));
	}
	m_bias = Neuron::getRandomWeight(engine);

}

Neuron::Neuron(int numOutputs, int neuronIndex) : m_neuronIndex(neuronIndex)
{
	//Set random weights on neuron outputs
	for (int con = 0; con < numOutputs; ++con) {
		m_outputWeights.push_back(0.f);
	}
}

void Neuron::setOutputVal(double value)
{
	m_outputVal = value;
}

double Neuron::getOutputVal() const
{
	return m_outputVal;
}

void Neuron::forward(Layer & prevLayer, int numForwards)
{
	double sum = 0.0;
	//Sum outputs of prevLayer including the bias
	//std::cout << "size: " << prevLayer.size() << "\n";
	for (int i = 0; i < prevLayer.size(); i++) {
		sum += prevLayer[i].getOutputVal() * prevLayer[i].m_outputWeights[m_neuronIndex];
		//std::cout << "o: " << prevLayer[i].getOutputVal() << " ";
		//std::cout << "w: " << prevLayer[i].outputWeights[neuronIndex] << " ";
	}
	//std::cout << "\n\n";
	//m_outputVal = Neuron::activationFunction(sum * sin((numForwards/ m_divider) + m_bias));
	m_outputVal = Neuron::activationFunction(sum);
}

void Neuron::forward(Layer & prevLayer, int numForwards, double alternativeDivider)
{
	double sum = 0.0;
	//Sum outputs of prevLayer including the bias
	//std::cout << "size: " << prevLayer.size() << "\n";
	for (int i = 0; i < prevLayer.size(); i++) {
		sum += prevLayer[i].getOutputVal() * prevLayer[i].m_outputWeights[m_neuronIndex];
		//std::cout << "o: " << prevLayer[i].getOutputVal() << " ";
		//std::cout << "w: " << prevLayer[i].outputWeights[neuronIndex] << " ";
	}
	//double sign = (sin((numForwards / 16) + m_bias) < 0) ? -1 : 1;
	//sign = 1;

	//double sign = (sin(numForwards * 4) + (sin(numForwards * 16) / 4)) * 2 * (-floor(sin(numForwards * 2)) + 0.1);

	double offset = 0.3;
	//alternativeDivider = 10;
	double rythmBase = sin((numForwards / alternativeDivider) + m_bias);
	double rythm = (rythmBase > 0) ? rythmBase + offset : rythmBase - offset;
	//rythm = 1;
	m_outputVal = Neuron::activationFunction(sum * rythm);

}

void Neuron::forwardRecurrent(Layer& prevLayer, int numForwards, Layer& lastTimestepPrevLayer)
{
	double sum = 0.0;
	//Sum outputs of prevLayer
	for (int i = 0; i < prevLayer.size(); i++) {
		sum += prevLayer[i].getOutputVal() * prevLayer[i].m_outputWeights[m_neuronIndex] + lastTimestepPrevLayer[i].getOutputVal() * lastTimestepPrevLayer[i].m_outputWeights[m_neuronIndex];
		//std::cout << "o: " << prevLayer[i].getOutputVal() << " ";
	}
	//std::cout << "\n\n";

	m_outputVal = Neuron::activationFunction(sum);
}

void Neuron::mutate(double mutationRate, double mutationChance, std::default_random_engine &engine)
{


	std::uniform_real_distribution<double> distribution(0, 1);
	double mutationChoice = distribution(engine);
	if (mutationChoice < 0.35) {
		m_bias += getRandomWeight(engine) * mutationRate;
	}
	else {
		std::uniform_real_distribution<double> distribution(0, m_outputWeights.size()-1);
		double weightIndex = distribution(engine);
		double chanceNewWeight = getRandomWeight(0, 1, engine);
		//divider += getRandomWeight(engine) * mutationRate;
		if (chanceNewWeight <= 0.1) {
			m_outputWeights[weightIndex] = getRandomWeight(engine);
		}
		else {
			m_outputWeights[weightIndex] += getRandomWeight(engine) * mutationRate;
		}
	}

	//for (int i = 0; i < m_outputWeights.size(); i++) {
	//	double mutRand = getRandomWeight(0, 1, engine);
	//	if (mutRand <= mutationChance) {
	//		std::uniform_real_distribution<double> distribution(0, 1);
	//		double mutationChoice = distribution(engine);
	//		if (mutationChoice < 0.35) {
	//			m_bias += getRandomWeight(engine) * mutationRate;
	//		}
	//		else {
	//			double chanceNewWeight = getRandomWeight(0, 1, engine);
	//			//divider += getRandomWeight(engine) * mutationRate;
	//			if (chanceNewWeight <= 0.05) {
	//				m_outputWeights[i] = getRandomWeight(engine);
	//			}
	//			else {
	//				m_outputWeights[i] += getRandomWeight(engine) * mutationRate;
	//			}
	//		}
	//	}
	//}

}

std::vector<double> Neuron::getOutputWeights()
{
	return m_outputWeights;
}

void Neuron::setOutputWeights(std::vector<double> weights)
{
	m_outputWeights = weights;
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
	for (int i = 0; m_outputWeights.size() < i; i++) {
		m_outputWeights[i] = 0;
	}
}

double Neuron::getBias()
{
	return m_bias;
}

void Neuron::setBias(double bias)
{
	m_bias = bias;
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
