#pragma once

//GLM Mathematics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//Additional includes
#include <btBulletDynamicsCommon.h>


class Util
{
public:
	static btVector3 convertToBtVector3(glm::vec3 vector);

};

