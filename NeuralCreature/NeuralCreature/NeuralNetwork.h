#pragma once
#include <vector>
#include <iostream>
#include "Neuron.h"

typedef std::vector<Neuron> Layer;
class NeuralNetwork
{
private:

	std::vector<Layer> layers; //layers[layerNum][neuronNum]
public:
	NeuralNetwork(const std::vector<int> &topology);
	NeuralNetwork(const NeuralNetwork &neuralNet);
	void forward(const std::vector<double>& inputVals);
	void getResults(std::vector<double>& resultsVals) const;
	void mutate(double mutationRate);
	~NeuralNetwork();
};

