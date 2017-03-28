/**
Util.cpp
Purpose: Static utility class for various helper functions.

@author Sjur Barndon, Jonas Sørsdal
@version 1.0 23.03.2017
*/

#include "stdafx.h"
#include "Util.h"



/**
	Converts a glm::vec3 vector to a btVector3.

	@param vector The vector to convert.
*/
btVector3 Util::convertToBtVector3(glm::vec3 vector)
{
	return btVector3(vector.x, vector.y, vector.z);
}

