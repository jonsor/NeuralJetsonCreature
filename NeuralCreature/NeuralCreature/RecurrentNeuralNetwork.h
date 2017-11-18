#pragma once

#include <vector>
#include <iostream>
#include "Neuron.h"

typedef std::vector<Neuron> Layer;

class RecurrentNeuralNetwork
{
private:
	int numForwards = 0;
	double divider = 8;
	std::vector<Layer> layers; //layers[layerNum][neuronNum]
	std::vector<Layer> previousLayers;
	std::vector<int> topology;
public:
	RecurrentNeuralNetwork();
	RecurrentNeuralNetwork(const std::vector<int> &topology, std::default_random_engine &engine);
	RecurrentNeuralNetwork(const RecurrentNeuralNetwork &neuralNet);
	RecurrentNeuralNetwork(const std::vector<Layer> networkLayers, const std::vector<int>& topology);
	RecurrentNeuralNetwork(const std::vector<Layer> layers);
	void forward(std::vector<double>& inputVals);
	void getResults(std::vector<double>& resultsVals) const;
	void mutate(double mutationRate, double mutationChance, std::default_random_engine engine);
	std::vector<Layer> getLayers();
	std::vector<Layer>* getL();
	std::vector<int> getTopology();
	~RecurrentNeuralNetwork();
};

