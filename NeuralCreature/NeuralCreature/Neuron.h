#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <ctime>
#include <chrono>
#include <random>

class Neuron
{
private:
	typedef std::vector<Neuron> Layer;
	double outputVal;
	std::vector<double> outputWeights;
	int neuronIndex;
	static double getRandomWeight();
	static double activationFunction(double value);
	
public:
	Neuron(int numOutputs, int neuronIndex);
	void setOutputVal(double value);
	double getOutputVal() const;
	void forward(Layer& prevLayer);
	void mutate(double mutationRate, double mutationChance);
	std::vector<double> getOutputWeights();
	void setOutputWeights(std::vector<double> weights);
	~Neuron();
};

