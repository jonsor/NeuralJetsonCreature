#include "stdafx.h"
#include "NetworkWriter.h"


NetworkWriter::NetworkWriter()
{
}

void NetworkWriter::writeToFile(std::vector<Creature*> creatures)
{
	std::ofstream myfile;
	myfile.open("network.txt");
	myfile << creatures.size() << "\n";
	for (int i = 0; i < creatures.size(); i++) {

		std::vector<std::vector<Neuron>> tempNetLayers = creatures[i]->getNeuralNetwork().getLayers();
		for (int i = 0; i < tempNetLayers.size(); i++) {
			Layer& tempLayer = tempNetLayers[i];

			for (int j = 0; j < tempLayer.size(); j++) {
				std::vector<double> outputWeights = tempLayer[j].getOutputWeights();
				for (int k = 0; k < outputWeights.size(); k++) {
					myfile << outputWeights[k] << " ";
				}
				myfile << "\n";
			}
		}
	}
	myfile.close();
}

void NetworkWriter::readFromFile(std::vector<Creature*> creatures)
{
	/*
	std::ifstream myfile;
	myfile.open("network.txt");
	if (myfile.is_open())
	{
		std::string line;
		std::getline(myfile, line);

		for (int i = 0; i < creatures.size(); i++) {

			std::vector<std::vector<Neuron>> tempNetLayers = creatures[i]->getNeuralNetwork().getLayers();
			for (int i = 0; i < tempNetLayers.size(); i++) {
				Layer& tempLayer = tempNetLayers[i];

				for (int j = 0; j < tempLayer.size(); j++) {
					std::vector<double> outputWeights = tempLayer[j].getOutputWeights();
					std::getline(myfile, line);
					std::istringstream iss(line);
					std::vector<std::string> outputWeights((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());

					//for (int k = 0; k < outputWeights.size(); k++) {
					//std::cout << outputWeights[i] << " ";
					//}
				}
			}
		}
		myfile.close();
	}
	*/
	//}
}


NetworkWriter::~NetworkWriter()
{
}
