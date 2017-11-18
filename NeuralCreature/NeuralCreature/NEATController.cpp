#include "stdafx.h"
#include "NEATController.h"


NEATController::NEATController()
{
}

NEATController::NEATController(int numInputs, int numOutputs, double sameSpeciesThreshold, double disjointCoefficient, double excessCoefficient, double averageWeightDifferenceCoefficient):
	m_numInputs(numInputs), m_numOutputs(numOutputs), m_sameSpeciesThreshold(sameSpeciesThreshold), m_disjointCoefficient(disjointCoefficient), m_excessCoefficient(excessCoefficient), m_averageWeightDifferenceCoefficient(averageWeightDifferenceCoefficient)
{
	createInitalGenes();
}

void NEATController::createInitalGenes()
{
	for (int i = 0; i < m_numInputs; i++) {
		for (int j = m_numInputs; j < m_numInputs + m_numOutputs; j++) {
			addNewGene(m_innovationNumber, i, j);
			m_innovationNumber++;
		}
	}
}

void NEATController::addNewGene(int innovationNumber, int inNodeId, int outNodeId)
{
	NEATGene gene(innovationNumber, inNodeId, outNodeId, 1.f, true);
	genes.push_back(gene);
}

int NEATController::checkGeneExistence(int inNodeID, int outNodeID)
{
	int oldInnovationNumber = m_innovationNumber;
	int numberOfGenes = getGeneCount();

	for (int i = 0; i < numberOfGenes; i++) {
		NEATGene gene = genes[i];
		int inID = gene.getInId();
		int outID = gene.getOutId();
		if (inID == inNodeID && outID == outNodeID) {
			return gene.getInnovationNumber();
		}
	}

	addNewGene(m_innovationNumber, inNodeID, outNodeID); 
	m_innovationNumber++;

	return oldInnovationNumber; 
}

void NEATController::setSameSpeciesThreshold(double threshold)
{
	m_sameSpeciesThreshold = threshold;
}
int NEATController::getGeneCount()
{
	return genes.size();
}

double NEATController::getExcessCoefficent()
{
	return m_excessCoefficient;
}

double NEATController::getDeltaThreshold()
{
	return m_sameSpeciesThreshold;
}

double NEATController::getDisjointCoefficient()
{
	return m_disjointCoefficient;
}

double NEATController::getAverageWeightDifferenceCoefficient()
{
	return m_averageWeightDifferenceCoefficient;
}

NEATController::~NEATController()
{
}
