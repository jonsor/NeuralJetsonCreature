#include "stdafx.h"
#include "NEATGene.h"


NEATGene::NEATGene()
{

}

NEATGene::NEATGene(const NEATGene & gene)
{
	innovationNumber = gene.innovationNumber;
	inID = gene.inID;
	outID = gene.outID;
	weight = gene.weight;
	on = gene.on;
}

NEATGene::NEATGene(int innovationNumber, int inID, int outID, float weight, bool on) : innovationNumber(innovationNumber), inID(inID), outID(outID), weight(weight), on(on)
{
}

int NEATGene::getInId()
{
	return inID;
}

int NEATGene::getOutId()
{
	return outID;
}

int NEATGene::getInnovationNumber()
{
	return innovationNumber;
}


double NEATGene::getWeight()
{
	return weight;
}

void NEATGene::setWeight(double newWeight)
{
	weight = newWeight;
}

bool NEATGene::getGeneState()
{
	return on;
}

void NEATGene::setGeneState(bool state)
{
	on = state;
}

bool NEATGene::equals(NEATGene & gene)
{

	if (inID == gene.inID && outID == gene.outID) {
		return true;
	}

	return false;
}

NEATGene::~NEATGene()
{
}
