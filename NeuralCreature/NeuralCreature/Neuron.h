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
	double divider = 10;
	double outputVal;
	std::vector<double> outputWeights;
	int neuronIndex;
	static double getRandomWeight(std::default_random_engine &generator);
	static double getRandomWeight(double min, double max, std::default_random_engine &generator);
	static double activationFunction(double value);
	
public:
	Neuron(int numOutputs, int neuronIndex, std::default_random_engine &engine);
	Neuron(int numOutputs, int neuronIndex);
	void setOutputVal(double value);
	double getOutputVal() const;
	void forward(Layer & prevLayer, int numForwards);
	void forward(Layer & prevLayer, int numForwards, double alternativeDivider);
	void forwardRecurrent(Layer & prevLayer, int numForwards, Layer & lastTimestepPrevLayer);
	void mutate(double mutationRate, double mutationChance, std::default_random_engine &engine);
	std::vector<double> getOutputWeights();
	void setOutputWeights(std::vector<double> weights);
	void setOutputWeightsToZero();
	~Neuron();
};

