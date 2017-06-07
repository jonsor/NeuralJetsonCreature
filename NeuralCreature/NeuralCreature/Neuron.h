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
	static double getRandomWeight(std::default_random_engine &generator);
	static double getRandomWeight(double min, double max, std::default_random_engine &generator);

	static double activationFunction(double value);
	
public:
	Neuron(int numOutputs, int neuronIndex, std::default_random_engine &engine);
	void setOutputVal(double value);
	double getOutputVal() const;
	void forward(Layer& prevLayer);
	void mutate(double mutationRate, double mutationChance, std::default_random_engine &engine);
	std::vector<double> getOutputWeights();
	void setOutputWeights(std::vector<double> weights);
	~Neuron();
};

