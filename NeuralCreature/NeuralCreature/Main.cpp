/**
Main.cpp
Purpose: Defines the entry point for the console application.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"

#include <btBulletDynamicsCommon.h>
#include <iostream>
#include "NeuralCreature.h"
#include "NeuralNetwork.h"
int main(void) {
	//To get true random variables
	//TODO save seeds
	srand(time(0));


	std::vector<int> v{5, 4, 3};
	NeuralNetwork nn(v);
	std::vector<double> inputs{ 0.5, 0.8, 0.1, 0.4, 0.9};
	nn.forward(inputs);
	std::vector<double> resultVec;
	nn.getResults(resultVec);
	for (int i = 0; i < resultVec.size(); i++) {
		std::cout << resultVec[i] << " ";
	}
	std::cout << std::endl;

	NeuralCreature world;
	world.init();

}