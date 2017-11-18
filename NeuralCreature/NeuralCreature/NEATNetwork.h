#pragma once
#include <vector>
#include <queue>
#include "NEATNeuron.h"
#include "NEATNode.h"
#include "NEATController.h"
#include <iostream>

class NEATNetwork
{
private:
	static const int INPUT_NEURON = 0;
	static const int INPUT_BIAS_NEURON = 1;
	static const int HIDDEN_NEURON = 2;
	static const int OUTPUT_NEURON = 3;

	std::vector<NEATNeuron> m_network;
	//NEATNeuron[] neuronArray;
	int m_usedHiddenNeuronIndex;

	NEATController m_controller; //Handles consultor genome sequence

	std::vector<NEATGene> m_genes; //list of the genome sequence for this neural network
	std::vector<NEATNode> m_nodes; //list of nodes for this neural network

	int m_numInputs; //Number of input perceptrons of neural network (including bias)
	int m_numOutputs; //Number of output perceptrons
	int m_netID; //ID of this neural network

	double m_timeAlive; //time the neural network actually lived in the test enviroment
	double m_fitness; //fitness of this neural network

public:
	void printFirstWeight();
	NEATNetwork();
	void generateNeuralNetworkFromGene();
	std::vector<double> forward(std::vector<double>& inputVals);
	void getResults(std::vector<double>& resultsVals);
	NEATNetwork(NEATNetwork& network);
	NEATNetwork(NEATController controller, int netId, int numInputs, int numOutputs);
	//NEATNetwork(NEATController consultor, int numInputs, int numOutputs, std::vector<NEATNode> copyNodes, std::vector<NEATGene> copyGenes);
	void createNodes();
	void createGenes();

	double getFitness();
	double getTimeAlive();
	void setId(int id);
	void addToFitness(double fitness);
	void setTimeAlive(double timeAlive);
	void addToTimeAlive(double timeAlive);
	//int getId();
	//double getTestTime();
	//void setTestTime();
	int getNodeCount();
	//int getGeneCount();
	int getNumInputNodes();
	int getNumOutputNodes();
	NEATController& getController();
	//std::vector<std::vector<double>> getGeneDrawConnections();
	void setInputValues(std::vector<double> inputs);
	//std::vector<double> getAllNodeValues();
	//std::vector<double> getInputValues();
	std::vector<double> getOutputValues();
	//std::vector<double> getHiddenValues();
	//void setNodes(std::vector<NEATNode> nodes);
	//void setGenes(std::vector<NEATGene> genes);
	void mutate();
	void addConnection();
	void addNode();
	void mutateWeight();
	bool connectionExists(int inId, int outId);
	//void clearNodeValues();
	void insertNewGene(NEATGene gene);
	int findInnovationInsertIndex(int innovationNumber);
	//static NEATNetwork createMutateCopy(NEATNetwork network);
	static NEATNetwork crossover(NEATNetwork parent1, NEATNetwork parent2);
	static NEATGene crossoverCopyGene(NEATGene gene, int compareValue);
	//static bool checkIfSameSpecies(NEATNetwork network1, NEATNetwork network2);

	void printNetwork();


	~NEATNetwork();
};

