#include "stdafx.h"
#include "Util.h"


Util::Util()
{
}

btVector3 Util::convertToBtVector3(glm::vec3 vector)
{
	return btVector3(vector.x, vector.y, vector.z);
}


Util::~Util()
{
}
