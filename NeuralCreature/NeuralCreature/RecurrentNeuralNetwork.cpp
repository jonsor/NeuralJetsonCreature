#include "stdafx.h"
#include "RecurrentNeuralNetwork.h"


/**
NeuralNetwork.cpp
Purpose: Creates a neural network

@author Jonas Sørsdal
@version 1.0 07.04.2017
*/

#include "stdafx.h"
#include "RecurrentNeuralNetwork.h"


/**
Sets up a neural network

@param topology Topology of network, the first and last variable in vector are input and output respectively

*/
RecurrentNeuralNetwork::RecurrentNeuralNetwork()
{
}

RecurrentNeuralNetwork::RecurrentNeuralNetwork(const std::vector<int> &topology, std::default_random_engine &engine) : topology(topology)
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

	//for (int i = 0; i < layers.size(); i++) {
	//	for (int j = 0; j < layers[i].size(); j++) {
	//		std::cout << "o: " << layers[i][j].getOutputVal() << " ";
	//	}
	//	std::cout << "\n\n";
	//}
	//std::cout << "\n";

	//Initalize previous network with weights as zero
	for (int layerNum = 0; layerNum < numLayers; layerNum++) {
		//Add a new layer
		previousLayers.push_back(Layer());


		int numOutputs = 0;
		if (layerNum != topology.size() - 1) {
			numOutputs = topology[layerNum + 1];
		}

		//Fill the added layer with neurons
		for (int neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum) {
			//Get last element (back()) and push a new Neuron
			Neuron tempNeuron = Neuron(numOutputs, neuronNum);
			previousLayers.back().push_back(tempNeuron);
		}

		//Force bias to 1.0
		previousLayers.back().back().setOutputVal(1.0);
	}

	std::uniform_real_distribution<double> distribution2(5, 60);
	double randNum = distribution2(engine);
	divider = randNum;
	//std::cout << "divider: " << divider << "\n";
	numForwards = 0;
}

RecurrentNeuralNetwork::RecurrentNeuralNetwork(const RecurrentNeuralNetwork & neuralNet)
{
	layers = neuralNet.layers;
	topology = neuralNet.topology;
	divider = neuralNet.divider;
	//std::cout << "divider: " << divider << "\n";
	numForwards = 0;
	int numLayers = topology.size();
	for (int layerNum = 0; layerNum < numLayers; layerNum++) {
		//Add a new layer
		previousLayers.push_back(Layer());


		int numOutputs = 0;
		if (layerNum != topology.size() - 1) {
			numOutputs = topology[layerNum + 1];
		}

		//Fill the added layer with neurons
		for (int neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum) {
			//Get last element (back()) and push a new Neuron
			Neuron tempNeuron = Neuron(numOutputs, neuronNum);
			previousLayers.back().push_back(tempNeuron);
		}

		//Force bias to 1.0
		previousLayers.back().back().setOutputVal(1.0);
	}
}

RecurrentNeuralNetwork::RecurrentNeuralNetwork(const std::vector<Layer> networkLayers, const std::vector<int> &topology, double div) : topology(topology)
{
	layers = networkLayers;
	int numLayers = topology.size();
	divider = div;
	for (int layerNum = 0; layerNum < numLayers; layerNum++) {
		//Add a new layer
		previousLayers.push_back(Layer());


		int numOutputs = 0;
		if (layerNum != topology.size() - 1) {
			numOutputs = topology[layerNum + 1];
		}

		//Fill the added layer with neurons
		for (int neuronNum = 0; neuronNum <= topology[layerNum]; ++neuronNum) {
			//Get last element (back()) and push a new Neuron
			Neuron tempNeuron = Neuron(numOutputs, neuronNum);
			previousLayers.back().push_back(tempNeuron);
		}

		//Force bias to 1.0
		previousLayers.back().back().setOutputVal(1.0);
	}
}


void RecurrentNeuralNetwork::forward(std::vector<double>& inputVals)
{

	//Set input values on first layer
	for (int i = 0; i < inputVals.size(); i++) {
		layers[0][i].setOutputVal(inputVals[i]);
	}

	//Forward popagate input values though the hidden layers and the output layer
	for (int layerNum = 1; layerNum < layers.size(); layerNum++) {
		Layer &prevLayer = layers[layerNum - 1];
		Layer &lastTimestepPrevLayer = previousLayers[layerNum - 1];
		if (layerNum == layers.size() - 1) {
		//size() - 1 to exclude bias layer
			for (int i = 0; i < layers[layerNum].size() - 1; i++) {
				//layers[layerNum][i].forward(prevLayer, numForwards);
				layers[layerNum][i].forward(prevLayer, numForwards, divider);
			}
		}
		else {
			for (int i = 0; i < layers[layerNum].size() - 1; i++) {
				layers[layerNum][i].forwardRecurrent(prevLayer, numForwards, lastTimestepPrevLayer);
				//layers[layerNum][i].forward(prevLayer, numForwards);
			}
		}
	}
	numForwards++;
	previousLayers = layers;
}

void RecurrentNeuralNetwork::getResults(std::vector<double>& resultsVals) const
{
	resultsVals.clear();

	for (int i = 0; i < layers.back().size() - 1; i++) {
		resultsVals.push_back(layers.back()[i].getOutputVal());
	}

}

void RecurrentNeuralNetwork::mutate(double mutationRate, double mutationChance, std::default_random_engine engine)
{
	//TODO one layer doesnt contain weights, exclude it.
	//THIS IS POSSIBLY WRONG! HAS A BIAS TOWARDS CENTRE LAYERS
	//std::cout << layers[0][layers[0].size()-1].getOutputVal() << std::endl;
	//int endPoint = rand() % layers.size()+1;
	//int startPoint = rand() % endPoint;
	////rand() % (max - min + 1) + min;
	//int startPoint = endPoint -1;
	std::uniform_real_distribution<double> distribution(0, 1);
	double mutationChoice = distribution(engine);
	if (mutationChoice < 0.35) {
		//double mutationRandomNum = distribution(engine);
		//if (mutationRandomNum <= 0.3) {
			std::uniform_real_distribution<double> distribution2(-1.0, 1.0);
			double randNum = distribution2(engine);
			divider += randNum * mutationRate;
		//}

		double probNewDivider = distribution(engine);
		if (probNewDivider < 0.03) {
			divider = distribution(engine) * 60;
		}
	}
	else {
		std::uniform_real_distribution<double> choiceDist(0, layers.size()-1);
		double mutateLayer = choiceDist(engine);
		Layer& tempLayer = layers[mutateLayer];
		std::uniform_real_distribution<double> choiceDist2(0, tempLayer.size()-1);
		double mutateIndex = choiceDist2(engine);
		tempLayer[mutateIndex].mutate(mutationRate, mutationChance, engine);

		//for (int i = 0; i < layers.size(); i++) {
		//	Layer& tempLayer = layers[i];
		//	for (int j = 0; j < tempLayer.size(); j++) {
		//		//double mutRand = ((double)rand() / (RAND_MAX));
		//		//if (mutRand <= mutationChance) {
		//		tempLayer[i].mutate(mutationRate, mutationChance, engine);
		//		//}
		//	}
		//}
	}

	//int layerPoint = rand() % layers.size();
	//Layer& tempLayer = layers[layerPoint];
	//int neuronPoint = rand() % tempLayer.size();
	//tempLayer[neuronPoint].mutate(mutationRate);

	//std::cout << "\n";
}

std::vector<Layer> RecurrentNeuralNetwork::getLayers()
{
	return layers;
}

std::vector<Layer>* RecurrentNeuralNetwork::getL()
{
	return &layers;
}

std::vector<int> RecurrentNeuralNetwork::getTopology()
{
	return topology;
}

double RecurrentNeuralNetwork::getDivider()
{
	return divider;
}

void RecurrentNeuralNetwork::setDivider(double div)
{
	divider = div;
}

RecurrentNeuralNetwork::~RecurrentNeuralNetwork()
{
}

