#pragma once
#include "Creature.h"
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

class NetworkWriter
{
public:
	NetworkWriter();
	static void writeToFile(std::vector<Creature*> creatures, unsigned mainSeed);
	static void readFromFile(std::vector<Creature*> creatures, unsigned mainSeed);

	static void writeToFile(std::vector<Creature*> creatures, std::string fileName, unsigned mainSeed);
	static void readFromFile(std::vector<Creature*> creatures, std::string fileName, unsigned mainSeed);
	~NetworkWriter();
};

