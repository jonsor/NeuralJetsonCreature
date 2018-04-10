#pragma once
#include "Biped.h"
#include "Dog.h";
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
template <class T>
class NetworkWriter
{

public:
	//template <class T>
	static void writeToFile(std::vector<T*> creatures, int nonEvolvedGen, int generation, unsigned mainSeed);
	static void writeToFile(std::vector<T*> creatures, int generation, unsigned mainSeed);
	static void writeFitness(double bestFitness);
	static void writeDistance(double distance);
	static void readFromFile(std::vector<T*> creatures);
	static void readFromFile(int num, std::vector<T*> creatures);

	static void writeToFile(std::vector<T*> creatures, std::string fileName, int generation, unsigned mainSeed);
	static void writeFitness(double bestFitness, std::string fileName);
	static void readFromFile(std::vector<T*> creatures, std::string fileName);
	static void writeDistance(double distance, std::string fileName);
};

