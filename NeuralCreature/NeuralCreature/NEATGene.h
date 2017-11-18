#pragma once
class NEATGene
{
private:
	//const int GENE_INFORMATION_SIZE = 4; 

	int innovationNumber;
	int inID;
	int outID;
	double weight;
	bool on;

public:
	NEATGene();
	NEATGene(const NEATGene& gene);
	NEATGene(int innovationNumber, int inID, int outID, float weight, bool on);
	int getInId();
	int getOutId();
	int getInnovationNumber();
	double getWeight();
	void setWeight(double newWeight);
	bool getGeneState();
	void setGeneState(bool state);
	bool equals(NEATGene& gene);
	~NEATGene();
};

