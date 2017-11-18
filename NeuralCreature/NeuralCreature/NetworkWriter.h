#pragma once
#include "Biped.h"
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

class NetworkWriter
{
public:
	NetworkWriter();
	static void writeToFile(std::vector<Biped*> creatures, int generation, unsigned mainSeed);
	static void writeFitness(double bestFitness);
	static void writeDistance(double distance);
	static void readFromFile(std::vector<Biped*> creatures);

	static void writeToFile(std::vector<Biped*> creatures, std::string fileName, int generation, unsigned mainSeed);
	static void writeFitness(double bestFitness, std::string fileName);
	static void readFromFile(std::vector<Biped*> creatures, std::string fileName);
	static void writeDistance(double distance, std::string fileName);
	~NetworkWriter();
};

