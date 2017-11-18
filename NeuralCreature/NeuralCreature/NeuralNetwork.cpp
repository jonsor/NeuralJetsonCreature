/**
NeuralNetwork.cpp
Purpose: Creates a neural network

@author Jonas Sørsdal
@version 1.0 07.04.2017
*/

#include "stdafx.h"
#include "NeuralNetwork.h"


/**
Sets up a neural network

@param topology Topology of network, the first and last variable in vector are input and output respectively

*/
NeuralNetwork::NeuralNetwork()
{
}

NeuralNetwork::NeuralNetwork(const std::vector<int> &topology, std::default_random_engine &engine)
{
	int numLayers = topology.size();

	//Creates neural network
	//Adds one bias neuron for each layer
	for (int layerNum = 0; layerNum < numLayers; layerNum++) {
		//Add a new layer
		layers.push_back(Layer());


		int numOutputs = 0;
		if (layerNum != topology.size() - 1) {
			numOutputs = topology[layerNum + 1];
		}

		//Fill the added layer with neurons
		for (int neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum) {
			//Get last element (back()) and push a new Neuron
			layers.back().push_back(Neuron(numOutputs, neuronNum, engine));
			//std::cout << "Neuron added!" << std::endl;
		}
		//std::cout << std::endl;

		//Force bias to 1.0
		layers.back().back().setOutputVal(1.0);
	}
}

NeuralNetwork::NeuralNetwork(const NeuralNetwork & neuralNet)
{
	layers = neuralNet.layers;
}

NeuralNetwork::NeuralNetwork(const std::vector<Layer> networkLayers)
{
	layers = networkLayers;
}


void NeuralNetwork::forward(const std::vector<double>& inputVals)
{
	numForwards++;
	//Set input values on first layer
	for (int i = 0; i < inputVals.size(); i++) {
		layers[0][i].setOutputVal(inputVals[i]);
	}

	//Forward popagate input values though the hidden layers and the output layer
	for (int layerNum = 1; layerNum < layers.size(); layerNum++) {
		Layer &prevLayer = layers[layerNum - 1];
		//size() - 1 to exclude bias layer
		for (int i = 0; i < layers[layerNum].size() - 1; i++) {
			layers[layerNum][i].forward(prevLayer, numForwards);
		}
	}

}

void NeuralNetwork::getResults(std::vector<double>& resultsVals) const
{
	resultsVals.clear();

	for (int i = 0; i < layers.back().size() - 1; i++) {
		resultsVals.push_back(layers.back()[i].getOutputVal());
	}

}

void NeuralNetwork::mutate(double mutationRate, double mutationChance, std::default_random_engine engine)
{
	//TODO one layer doesnt contain weights, exclude it.
	//THIS IS POSSIBLY WRONG! HAS A BIAS TOWARDS CENTRE LAYERS
	//std::cout << layers[0][layers[0].size()-1].getOutputVal() << std::endl;
	//int endPoint = rand() % layers.size()+1;
	//int startPoint = rand() % endPoint;
	////rand() % (max - min + 1) + min;
	//int startPoint = endPoint -1;

	for (int i = 0; i < layers.size(); i++) {
		Layer& tempLayer = layers[i];
		for (int j = 0; j < tempLayer.size(); j++) {
			//double mutRand = ((double)rand() / (RAND_MAX));
			//if (mutRand <= mutationChance) {
				tempLayer[i].mutate(mutationRate, mutationChance, engine);
			//}
		}
	}

	//int layerPoint = rand() % layers.size();
	//Layer& tempLayer = layers[layerPoint];
	//int neuronPoint = rand() % tempLayer.size();
	//tempLayer[neuronPoint].mutate(mutationRate);

	//std::cout << "\n";
}

std::vector<Layer> NeuralNetwork::getLayers()
{
	return layers;
}

std::vector<Layer>* NeuralNetwork::getL()
{
	return &layers;
}

void NeuralNetwork::printNetwork()
{
	std::cout << "layerSize: " << layers.size() << "\n";
	for (int i = 0; i < layers.size(); i++) {
		std::cout << "layer: " << i << "\n";
		for (int j = 0; j < layers[i].size(); j++) {
			std::cout << "\nVal: ";
			std::cout << layers[i][j].getOutputVal() << " \n";
			std::cout << "weights:\n";
			for (int k = 0; k < layers[i][j].getOutputWeights().size(); k++) {
				std::cout << layers[i][j].getOutputWeights()[k] << " ";
			}
		}
	}

}

NeuralNetwork::~NeuralNetwork()
{
}
