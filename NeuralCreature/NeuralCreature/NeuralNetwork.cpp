#include "stdafx.h"
#include "NeuralNetwork.h"

/**
NeuralNetwork.cpp
Purpose: Creates a neural network

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 07.04.2017
*/

/**
Sets up a neural network

@param topology Topology of network, the first and last variable in vector are input and output respectively

*/
NeuralNetwork::NeuralNetwork(const std::vector<int> &topology)
{
	int numLayers = topology.size();

	//Creates neural network
	//Adds one bias neuron for each layer
	for (int layerNum = 0; layerNum < numLayers; ++layerNum) {
		//Add a new layer
		layers.push_back(Layer());


		int numOutputs = 0;
		if (layerNum != topology.size() - 1) {
			numOutputs = topology[layerNum + 1];
		}

		//Fill the added layer with neurons
		for (int neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum) {
			//Get last element (back()) and push a new Neuron
			layers.back().push_back(Neuron(numOutputs, neuronNum));
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
	//Set input values on first layer
	for (int i = 0; i < inputVals.size(); i++) {
		layers[0][i].setOutputVal(inputVals[i]);
	}

	//Forward popagate input values though the hidden layers and the output layer
	for (int layerNum = 1; layerNum < layers.size(); layerNum++) {
		Layer &prevLayer = layers[layerNum - 1];
		//size() - 1 to exclude bias layer
		for (int i = 0; i < layers[layerNum].size() - 1; i++) {
			layers[layerNum][i].forward(prevLayer);
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

void NeuralNetwork::mutate(double mutationRate)
{
	//TODO one layer doesnt contain weights, exclude it.
	//std::cout << layers[0][layers[0].size()-1].getOutputVal() << std::endl;
	int endPoint = rand() % layers.size()+1;
	int startPoint = rand() % endPoint;
	//int startPoint = endPoint -1;
	for (int i = startPoint; i < endPoint; i++) {
		Layer& tempLayer = layers[i];

		int endPoint2 = rand() % tempLayer.size() + 1;
		int startPoint2 = rand() % endPoint;
		//int startPoint2 = endPoint2 -1;
		for (int j = startPoint2; j < endPoint2; j++) {
			//std::cout << "mutateweight";
			tempLayer[i].mutate(mutationRate);
		}
	}
	//std::cout << "\n";
}

std::vector<Layer> NeuralNetwork::getLayers()
{
	return layers;
}

NeuralNetwork::~NeuralNetwork()
{
}
