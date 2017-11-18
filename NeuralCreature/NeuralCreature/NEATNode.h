#pragma once
#include <math.h>
class NEATNode
{
private:
	static const int INPUT_NEURON = 0;
	static const int INPUT_BIAS_NEURON = 1;
	static const int HIDDEN_NEURON = 2;
	static const int OUTPUT_NEURON = 3;

	int m_id;
	int m_type;
	double m_outputValue;

public:
	NEATNode();
	NEATNode(const NEATNode& neuron);
	NEATNode(int id, int type);

	int getNeuronId();
	int getNeuronType();
	int getOutputValue();
	void setOutputValue(double value);
	void activationFunction();
	~NEATNode();
};


