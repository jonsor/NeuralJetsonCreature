#pragma once

//GLM Mathematics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//Additional includes
#include <btBulletDynamicsCommon.h>
#include <vector>


class Util
{
public:
	static btVector3 convertToBtVector3(glm::vec3 vector);
	static double normalize(double x, double min, double max);
	static double scaleToRange(double x, double min, double max);
	static double normalizeSigned(double x, double min, double max);
	static bool intContains(std::vector<int> * vec, int num);
	static std::vector<int> Util::getRandomIndices(int sizeOfVec, int numIndices);
};

