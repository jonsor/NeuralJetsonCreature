/**
Main.cpp
Purpose: Defines the entry point for the console application.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"

#include <btBulletDynamicsCommon.h>
#include <iostream>
#include "World.h"
#include "NeuralNetwork.h"

int main(void) {
	std::srand(time(NULL));
	World world;
	world.init();

}