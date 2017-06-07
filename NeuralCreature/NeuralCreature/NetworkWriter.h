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
	static void writeToFile(std::vector<Creature*> creatures);
	static void readFromFile(std::vector<Creature*> creatures);
	~NetworkWriter();
};

