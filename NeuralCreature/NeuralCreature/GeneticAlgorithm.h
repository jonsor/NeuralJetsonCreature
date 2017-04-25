#pragma once
class GeneticAlgorithm
{
private:
	double m_mutationRate;
public:
	GeneticAlgorithm(int mutationRate, int crossOverProb, int populationSize);
	void crossOver();
	double evaluateFitness();
	void mutate();
	~GeneticAlgorithm();
};

