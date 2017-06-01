#pragma once
#include "Creature.h"
#include <iostream>
#include <fstream>
#include <string>

class NetworkWriter
{
public:
	NetworkWriter();
	void writeToFile(std::vector<Creature*> creatures);
	void readFromFile(std::vector<Creature*> creatures);
	~NetworkWriter();
};

