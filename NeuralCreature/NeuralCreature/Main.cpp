/**
Main.cpp
Purpose: Defines the entry point for the console application.

@author Sjur Barndon, Jonas S�rsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"

#include <btBulletDynamicsCommon.h>
#include <iostream>
#include "NeuralCreature.h"
#include "NeuralNetwork.h"

int main(void) {
	std::srand(time(NULL));
	NeuralCreature world;
	world.init();

}