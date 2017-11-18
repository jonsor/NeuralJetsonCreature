#pragma once
#include <vector>
#include "NEATGene.h"
class NEATController
{
private:
	double m_sameSpeciesThreshold;
	double m_disjointCoefficient;
	double m_excessCoefficient;
	double m_averageWeightDifferenceCoefficient;

	int m_innovationNumber = 0;

	int m_numInputs;
	int m_numOutputs;

	std::vector<NEATGene> genes;

public:
	NEATController();
	NEATController(int numInputs, int numOutputs, double sameSpeciesThreshold, double disjointCoefficient, double excessCoefficient, double averageWeightDifferenceCoefficient);
	void createInitalGenes();
	void setSameSpeciesThreshold(double threshold);
	void addNewGene(int innovationNumber, int inNodeId, int outNodeId);
	int checkGeneExistence(int inNodeID, int outNodeID);

	int getGeneCount();
	double getExcessCoefficent();
	double getDeltaThreshold();
	double getDisjointCoefficient();
	double getAverageWeightDifferenceCoefficient();
	~NEATController();
};

