#include "stdafx.h"
#include "NetworkWriter.h"


NetworkWriter::NetworkWriter()
{
}

void NetworkWriter::writeToFile(std::vector<Creature*> creatures, int generation, unsigned mainSeed) {
	writeToFile(creatures, "network.txt", generation, mainSeed);
}

void NetworkWriter::writeFitness(double bestFitness)
{
	writeFitness(bestFitness, "fitnessPlot.txt");
}

void NetworkWriter::writeDistance(double distance)
{
	writeDistance(distance, "distancePlot.txt");
}

void NetworkWriter::readFromFile(std::vector<Creature*> creatures) {
	readFromFile(creatures, "network.txt");

}


void NetworkWriter::writeToFile(std::vector<Creature*> creatures, std::string fileName, int generation, unsigned mainSeed)
{
	std::cout << "Writing networks to file...\n";
	std::ofstream myfile;
	myfile.open(fileName);
	myfile << creatures.size() << " " << generation << " " << std::setprecision(20) << mainSeed << "\n";
	for (int i = 0; i < creatures.size(); i++) {

		std::vector<std::vector<Neuron>> tempNetLayers = creatures[i]->getNeuralNetwork().getLayers();
		for (int l = 0; l < tempNetLayers.size(); l++) {
			Layer& tempLayer = tempNetLayers[l];

			for (int j = 0; j < tempLayer.size(); j++) {
				std::vector<double> outputWeights = tempLayer[j].getOutputWeights();
				for (int k = 0; k < outputWeights.size(); k++) {
					myfile << std::setprecision(20) << outputWeights[k] << " ";
				}
				myfile << "\n";
			}
		}
	}
	myfile.close();
}

void NetworkWriter::writeFitness(double bestFitness, std::string fileName)
{
	std::ofstream myfile;
	myfile.open(fileName, std::ios::out | std::ios::app);
	myfile << std::setprecision(5) << bestFitness << "\n";
	myfile.close();
}

void NetworkWriter::readFromFile(std::vector<Creature*> creatures, std::string fileName)
{

	std::cout << "Reading networks from file...\n";
	std::ifstream myfile;
	myfile.open(fileName);
	if (myfile.is_open())
	{
		std::string line;
		std::getline(myfile, line);
		double numCreatures = std::stod(line);

		if (numCreatures != creatures.size()){
			std::cout << "Not the same size of creatures, net: " << numCreatures << " creature list: " << creatures.size() << "\n";
			return;
		}


		for (int i = 0; i < creatures.size(); i++) {

			std::vector<std::vector<Neuron>> tempNetLayers = creatures[i]->getNeuralNetwork().getLayers();
			for (int l = 0; l < tempNetLayers.size(); l++) {
				Layer& tempLayer = tempNetLayers[l];

				for (int j = 0; j < tempLayer.size(); j++) {

					std::vector<double> outputWeights = tempLayer[j].getOutputWeights();
					std::getline(myfile, line);

					std::istringstream buf(line);
					std::istream_iterator<std::string> beg(buf), end;

					std::vector<std::string> tokens(beg, end);
					for (int k = 0; k < outputWeights.size(); k++) {
						double weight = std::stod(tokens[k]);
						outputWeights[k] = weight;
					}
					tempLayer[j].setOutputWeights(outputWeights);
					creatures[i]->getNN()->getL()->at(l).at(j).setOutputWeights(outputWeights);
				}
			}
		}
		myfile.close();
	}
}

void NetworkWriter::writeDistance(double distance, std::string fileName)
{
	std::ofstream myfile;
	myfile.open(fileName, std::ios::out | std::ios::app);
	myfile << std::setprecision(5) << distance << "\n";
	myfile.close();
}


NetworkWriter::~NetworkWriter()
{
}
