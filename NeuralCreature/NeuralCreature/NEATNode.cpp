#include "stdafx.h"
#include "NEATNode.h"

NEATNode::NEATNode()
{
}

NEATNode::NEATNode(const NEATNode & neuron)
{
	m_id = neuron.m_id;
	m_type = neuron.m_type;

	if (m_type == INPUT_BIAS_NEURON) {
		m_outputValue = 1.f;
	}
	else {
		m_outputValue = 0.f;
	}
}

NEATNode::NEATNode(int id, int type) : m_id(id), m_type(type)
{
	if (m_type == INPUT_BIAS_NEURON) {
		m_outputValue = 1.f;
	}
	else {
		m_outputValue = 0.f;
	}
}


int NEATNode::getNeuronId()
{
	return m_id;
}

int NEATNode::getNeuronType()
{
	return m_type;
}

int NEATNode::getOutputValue()
{
	return m_outputValue;
}

void NEATNode::setOutputValue(double value)
{
	if (m_type = !INPUT_BIAS_NEURON) {
		m_outputValue = value;
	}
}

void NEATNode::activationFunction()
{
	m_outputValue = tanh(m_outputValue);
}

NEATNode::~NEATNode()
{
}

