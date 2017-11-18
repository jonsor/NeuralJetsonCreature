#pragma once

#include <math.h>
#include <vector>
#include "NEATGene.h"
class NEATNeuron
{
private:

	//NEATGene[] inArray;
	int m_id;
	double m_outputValue;

public:
	std::vector<NEATGene> inc;

	//NEATNeuron();
	NEATNeuron(int id, double outputValue);
	NEATNeuron(const NEATNeuron& neuron);
	int getId();
	void setId(int id);
	void setOutputValue(double outputValue);
	double getOutputValue();

	~NEATNeuron();
};

