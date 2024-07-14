#pragma once

#ifndef _COLLISION_PLANE_H_
#define _COLLISION_PLANE_H_

#include "phy.h"
#include "glm/glm.hpp"

class CollisionPlane : public Object
{
private:

	glm::vec3 _velocity;
	glm::vec3 _normal;
	glm::vec3 _point;

	float _mass;

public:

	CollisionPlane();

	void setVelocity(glm::vec3 velocity) { this->_velocity = velocity; }
	void setNormal(glm::vec3 normal)	 { this->_normal = normal; }
	void setPoint(glm::vec3 point)		 { this->_point = point;   }
	void setMass(float mass);

	glm::vec3 getVelocity() const { return this->_velocity; }
	glm::vec3 getNormal()   const { return this->_normal;   }
	glm::vec3 getPoint()    const { return this->_point;    }
	float getMass()		    const { return this->_mass;     }
};

#endif