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

	//double iapp = 0;
	//double d = 8;
	//double a = 0.02;
	//double b = 0.2;
	//double c = -65;
	//double v = -70;
	//double u = -14;
	//double vH = (0.04*v + 5)*v + 150 - u - iapp;
	//double uH = a*(b*v - u);
	//for (int i = 0; i < 1000; i++) {

	//}

	//double tmax = 1000;
	//double dt = 0.5;
	//double a = 0.02;
	//double b = 0.2;
	//double c = -65;
	//double d = 8;

	double v = -70;
	double u = -14;
	double a = 0.02;
	double b = 0.2;
	double c = -65;
	double d = 8;
	double I = 0;

	for (int t = 0; t < 1000; t++) {
		//I = ((double)rand() / (RAND_MAX));
		//double deltaV = .04f*v*v + 5 * v + 140 - u + I;
		double deltaV = (0.04*v + 5)*v + 150 - u - I;
		double deltaU = a*(b*v - u);
		//std::cout << deltaV << "\n";
		v += deltaV;
		u += deltaU;
		I = 0;

		if (v > 30) {
			std::cout <<  v << "\n";
			v = c;
			u = u + d;
			//std::cout << v << "\n";
		}
		else {
			//std::cout << v << "\n";
			//std::cout<< "in: " << v << "\n";
		}
		std::cout << v << "\n";
	}

	std::srand(time(NULL));
	World world;
	world.init();

}