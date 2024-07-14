#include "CollisionPlane.h"
#include <exception>

CollisionPlane::CollisionPlane()
{
	this->_normal = glm::vec3(0.0f, 1.0f, 0.0f);
	this->_point = glm::vec3(0.0f);
}

void CollisionPlane::setMass(float mass)
{
	int value{ (int)mass };

	if (value == 0)
		throw std::exception();

	this->_mass = mass;
}