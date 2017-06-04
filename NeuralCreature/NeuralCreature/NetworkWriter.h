#pragma once
#include "Creature.h"
#include <iostream>
#include <fstream>
#include <string>

class NetworkWriter
{
public:
	NetworkWriter();
	static void writeToFile(std::vector<Creature*> creatures);
	static void readFromFile(std::vector<Creature*> creatures);
	~NetworkWriter();
};

