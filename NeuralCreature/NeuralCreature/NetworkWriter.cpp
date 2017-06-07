#include "stdafx.h"
#include "NetworkWriter.h"


NetworkWriter::NetworkWriter()
{
}

void NetworkWriter::writeToFile(std::vector<Creature*> creatures)
{
	std::cout << "Writing networks to file...\n";
	std::ofstream myfile;
	myfile.open("network.txt");
	myfile << creatures.size() << "\n";
	for (int i = 0; i < creatures.size(); i++) {

		std::vector<std::vector<Neuron>> tempNetLayers = creatures[i]->getNeuralNetwork().getLayers();
		for (int l = 0; l < tempNetLayers.size(); l++) {
			Layer& tempLayer = tempNetLayers[l];

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

	std::cout << "Reading networks from file...\n";
	std::ifstream myfile;
	myfile.open("network.txt");
	if (myfile.is_open())
	{
		std::string line;
		std::getline(myfile, line);
		double numCreatures = std::stod(line);

		if (numCreatures != creatures.size()){
			std::cout << "Not the same size of creatures, net: " << numCreatures << " creature list: " << creatures.size() << "\n";
			return;
		}

		//std::istringstream buf(line);
		//std::istream_iterator<std::string> beg(buf), end;

		//std::vector<std::string> tokens(beg, end);

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
					//HVA FN!!! HVORFOR FUNKER IKKE DETTE
					creatures[i]->getNN()->getL()->at(l).at(j).setOutputWeights(outputWeights);
					//std::cout << creatures[i]->getNeuralNetwork().getLayers()[l][j].getOutputWeights()[0] << " ";
					//std::getline(myfile, line);
					//std::istringstream iss(line);
					//std::vector<std::string> outputWeights((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());

					//for (int k = 0; k < outputWeights.size(); k++) {
					//std::cout << outputWeights[i] << " ";
					//}
				}
			}
		}
		myfile.close();
	}
	//}
}


NetworkWriter::~NetworkWriter()
{
}
