/**
Util.cpp
Purpose: Static utility class for various helper functions.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "Util.h"
#include <iostream>



/**
	Converts a glm::vec3 vector to a btVector3.

	@param vector The vector to convert.
*/
btVector3 Util::convertToBtVector3(glm::vec3 vector)
{
	return btVector3(vector.x, vector.y, vector.z);
}

double Util::normalize(double x, double min, double max)
{
	double result = 2*(x - min) / (max - min)-1;
	return result;
}

double Util::scaleToRange(double x, double min, double max)
{
	//Normalize to 0, 1
	double result = (x + 1) / 2;
	//Scale to range
	double scaled = (result*(max - min)) + min;
	return scaled;
}

double Util::normalizeSigned(double x, double min, double max)
{
	return (x - min) / (max - min);
}

bool Util::intContains(std::vector<int>* vec, int num)
{
	bool contains = false;
	if (vec->size() != 0) {
		for (int i = 0; i < vec->size(); i++) {
			if (vec->at(i) == num) {
				contains = true;
			}
		}
	}
	return false;
}

std::vector<int> Util::getRandomIndices(int sizeOfVec, int numIndices) {
	std::vector<int> randomIndices;
	for (int j = 0; j < numIndices; j++) {
		int randInd = rand() % sizeOfVec;
		if (!Util::intContains(&randomIndices, randInd)) {
			randomIndices.push_back(randInd);
		}
	}
	
	return randomIndices;
}

