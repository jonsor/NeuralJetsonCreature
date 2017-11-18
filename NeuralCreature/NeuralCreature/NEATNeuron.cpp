#include "stdafx.h"
#include "NEATNeuron.h"

#include <iostream>
//NEATNeuron::NEATNeuron()
//{
//}


NEATNeuron::NEATNeuron(int id, double outputValue): m_id(id), m_outputValue(outputValue)
{
}

NEATNeuron::NEATNeuron(const NEATNeuron & neuron)
{
	m_id = neuron.m_id;
	m_outputValue = neuron.m_outputValue;
	inc = neuron.inc;
}

int NEATNeuron::getId()
{
	return m_id;
}

void NEATNeuron::setId(int id)
{
	m_id = id;
}

void NEATNeuron::setOutputValue(double outputValue)
{
	m_outputValue = outputValue;
}

double NEATNeuron::getOutputValue()
{
	return m_outputValue;
}

NEATNeuron::~NEATNeuron()
{
}
