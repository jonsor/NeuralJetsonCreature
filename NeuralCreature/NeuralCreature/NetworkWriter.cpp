/**
NetworkWriter.cpp
Purpose: Writes and stores network, generation, fitness and distance data. Also loads.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 5.06.2017
*/

#include "stdafx.h"
#include "NetworkWriter.h"

//template <class T>

//NetworkWriter::NetworkWriter()
//{
//}
template class NetworkWriter<Biped>;
template class NetworkWriter<Dog>;
//template class NetworkWriter<RecurrentNeuralNetwork>;
template <class T>
void NetworkWriter<T>::writeToFile(std::vector<T*> creatures, int nonEvolvedGen, int generation, unsigned mainSeed) {
	std::string fileName = "";
	if (std::is_same<T, RecurrentNeuralNetwork>::value) {
		std::string fileName = "networks" + std::to_string(nonEvolvedGen) + ".txt";
	}
	else {
		std::string fileName = (std::is_same<T, Dog>::value) ? "networksDogs" + std::to_string(nonEvolvedGen) + ".txt" : "networksBipeds.txt";
	}
	writeToFile(creatures, fileName, generation, mainSeed);
}

template <class T>
void NetworkWriter<T>::writeToFile(std::vector<T*> creatures, int generation, unsigned mainSeed) {
	std::string fileName = (std::is_same<T, Dog>::value) ? "networksDogs.txt" : "networksBipeds.txt";
	writeToFile(creatures, fileName, generation, mainSeed);

}

//void NetworkWriter::writeFitness(double bestFitness)
//{
//	writeFitness(bestFitness, "fitnessPlot.txt");
//}
//
//void NetworkWriter::writeDistance(double distance)
//{
//	writeDistance(distance, "distancePlot.txt");
//}
template <class T>
void NetworkWriter<T>::readFromFile(std::vector<T*> creatures) {
	std::string dogsName = "networksDogs.txt";
	std::string fileName = (std::is_same<T, Dog>::value) ? dogsName : "networksBipeds.txt";
	readFromFile(creatures, fileName);

}

template <class T>
void NetworkWriter<T>::readFromFile(int num, std::vector<T*> creatures) {
	std::string dogsName = "networksDogs" + std::to_string(num) +".txt";
	std::string fileName = (std::is_same<T, Dog>::value) ? dogsName : "networksBipeds.txt";
	readFromFile(creatures, fileName);

}

template <class T>
void NetworkWriter<T>::writeToFile(std::vector<T*> creatures, std::string fileName, int generation, unsigned mainSeed)
{
	std::cout << "Writing networks to file...\n";
	std::ofstream myfile(fileName);
	//myfile.open(fileName);
	myfile << creatures.size() << " " << generation << " " << std::setprecision(20) << mainSeed << std::endl;
	for (int i = 0; i < creatures.size(); i++) {
		//RecurrentNeuralNetwork network = (std::is_same<T, RecurrentNeuralNetwork>::value) ? creatures[i] : creatures[i]->getNeuralNetwork();
		myfile << std::setprecision(20) << creatures[i]->getNeuralNetwork().getDivider() << std::endl;

		std::vector<std::vector<Neuron>> tempNetLayers = creatures[i]->getNeuralNetwork().getLayers();
		for (int l = 0; l < tempNetLayers.size(); l++) {
			Layer& tempLayer = tempNetLayers[l];

			for (int j = 0; j < tempLayer.size(); j++) {
				std::vector<double> outputWeights = tempLayer[j].getOutputWeights();
				myfile << std::setprecision(20) << tempLayer[j].getBias() << std::endl;
				for (int k = 0; k < outputWeights.size(); k++) {
					myfile << std::setprecision(20) << outputWeights[k] << " ";
				}
				myfile << std::endl;
			}
		}
	}
	myfile.close();
}

template <class T>
void NetworkWriter<T>::readFromFile(std::vector<T*> creatures, std::string fileName)
{

	std::cout << "Reading networks from file...\n";
	std::ifstream myfile;
	myfile.open(fileName);
	if (myfile.is_open())
	{
		std::string line;
		std::getline(myfile, line);
		double numCreatures = std::stod(line);

		//if (numCreatures != creatures.size()) {
		//	std::cout << "Not the same size of creatures, net: " << numCreatures << " creature list: " << creatures.size() << "\n";
		//	return;
		//}


		for (int i = 0; i < numCreatures; i++) {

			//Get divider
			std::getline(myfile, line);
			double divider = std::stod(line);
			//std::cout << "divider: " << divider << "\n";
			creatures[i]->getNN()->setDivider(divider);

			std::vector<std::vector<Neuron>> tempNetLayers = creatures[i]->getNeuralNetwork().getLayers();
			for (int l = 0; l < tempNetLayers.size(); l++) {
				Layer& tempLayer = tempNetLayers[l];

				for (int j = 0; j < tempLayer.size(); j++) {

					//Get bias
					std::getline(myfile, line);
					double bias = std::stod(line);
					//std::cout << "bias: " << bias << "\n";
					creatures[i]->getNN()->getL()->at(l).at(j).setBias(bias);

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

//void NetworkWriter::writeFitness(double bestFitness, std::string fileName)
//{
//	std::ofstream myfile;
//	myfile.open(fileName, std::ios::out | std::ios::app);
//	myfile << std::setprecision(5) << bestFitness << "\n";
//	myfile.close();
//}

//void NetworkWriter::writeDistance(double distance, std::string fileName)
//{
//	std::ofstream myfile;
//	myfile.open(fileName, std::ios::out | std::ios::app);
//	myfile << std::setprecision(5) << distance << "\n";
//	myfile.close();
//}
//
//
//NetworkWriter::~NetworkWriter()
//{
//}
